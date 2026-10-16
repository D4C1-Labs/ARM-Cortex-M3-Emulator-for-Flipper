/*
 * emulator.c - EFM32 Cortex-M3 hardware model for the Pandora D-605 keyfob.
 *
 * Ported from Flipper-ARF/applications/system/PandoraARM/main.c (the reference
 * emulator that already works on the Flipper). The port keeps the HW model
 * BYTE-FOR-BYTE faithful; only the Flipper-specific plumbing (GUI, paged flash
 * cache, CC1101/speaker/LED HAL, FURI logging) is replaced:
 *   - flash is RAM-resident (direct region), not a paged LRU cache;
 *   - logs go through emu_log() (overridable by the host/ESP32 UART layer);
 *   - the RF bridge only tracks state + a TX edge ring (the Flipper does radio).
 */

#include "emulator.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>

/* ------------------------------------------------------------ logging ------- */
/* Weak hook so main.c (ESP32) or the host test harness can capture logs and
 * forward them over UART / to stdout. Default = no-op. */
#if defined(__GNUC__)
__attribute__((weak))
#endif
void emu_log(const char* line) {
    (void)line;
}

/* forward decls (functions reference each other across sections) */
static void nvic_tick(EmuState* app);

static void emu_logf(const char* fmt, ...) {
    char buf[160];
    va_list ap;
    __builtin_va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    __builtin_va_end(ap);
    emu_log(buf);
}

/* ------------------------------------------------------------ tuning --------
 * Button/unlock timing is pure HARDWARE order-of-magnitude (instructions as a
 * proxy for time), NOT a per-firmware code address. Mirrors pandora_tui.py
 * press_button / auto_unlock durations. */
#define BTN_PRESS_INSN 300000ull   /* short click: pin held low ~this long */
#define BTN_HOLD_INSN 1200000ull   /* long press (above typical long-press window) */
#define BTN_SETTLE_INSN 500000ull  /* settle after release (adaptive early-out) */
#define BTN_BURST 50000u

#define KNOCK_BOOT_SETTLE_INSN 1200000ull /* boot settle before the knock */
#define KNOCK_CLICK_INSN 150000ull        /* short knock click (brief pulses) */
#define KNOCK_CLICK_SETTLE_INSN 200000ull
#define KNOCK_GATE_HOLD_INSN 900000ull
#define KNOCK_GATE_SETTLE_INSN 400000ull
#define KNOCK_FINAL_SETTLE_INSN 2000000ull

/* ------------------------------------------------------------ HW profile ----
 * ONE agnostic hardware profile (ported from pandora_tui.py's `HW` dict). It
 * describes ONLY the physical keyfob hardware -- buttons, radio pins, OLED DC/CS
 * -- which is IDENTICAL for every firmware of this model. NO firmware code
 * address lives here. The SAME profile runs ANY .hex for this MCU. */
const ArmProfile PROFILE_PANDORA_HW = {
    .name = "pandora-d650-hw",
    /* 6 physical buttons -> (port, pin). BACK=(0,15) for ALL firmwares (the old
     * pandora profile's BACK=(2,7) was a vestigial OUTPUT, not a button). */
    .up_port = 5, .up_pin = 7,
    .down_port = 5, .down_pin = 5,
    .ok_port = 2, .ok_pin = 9,
    .back_port = 0, .back_pin = 15,
    .b5_port = 0, .b5_pin = 1,
    .b6_port = 0, .b6_pin = 6,
    /* radio Si4432 (direct mode) pins */
    .data_port = 1, .data_pin = 0,       /* PB0 = RX-data */
    .tx_data_port = 1, .tx_data_pin = 3, /* PB3 = TX-data */
    .rf_cs_port = 4, .rf_cs_pin = 9,     /* PE9 = Si4432 CS (active-high) */
    .rf_tx_cell_us = 400,
};

/* ------------------------------------------------------------ Si4432 -------- */

static void si4432_init(Si4432* s) {
    memset(s->regs, 0, sizeof(s->regs));
    s->regs[0x00] = 0x08;
    s->regs[0x01] = 0x06;
    s->regs[0x26] = 0x60;
    s->mode = 0;
}

static uint8_t si4432_read(Si4432* s, uint8_t reg) {
    reg &= 0x7F;
    uint8_t v = s->regs[reg];
    if(reg == 0x03 || reg == 0x04) s->regs[reg] = 0; /* INT_STATUS clears on read */
    return v;
}

/* Returns the new mode if it changed, -1 otherwise. REG[0x07]: bit3=TX bit2=RX
 * bit0=READY. (PROTO_RF_* numbering: 3 TX, 2 RX, 1 READY, 0 IDLE.) */
static int si4432_write(Si4432* s, uint8_t reg, uint8_t val) {
    reg &= 0x7F;
    s->regs[reg] = val;
    if(reg == 0x07) {
        uint8_t nm = s->mode;
        if(val & 0x08) nm = 3;
        else if(val & 0x04) nm = 2;
        else if(val & 0x01) nm = 1;
        else nm = 0;
        if(nm != s->mode) {
            emu_logf("si4432: REG[0x07]=0x%02X -> mode %u (%s)", val, nm,
                     nm == 3 ? "TX" : nm == 2 ? "RX" : nm == 1 ? "READY" : "IDLE");
            s->mode = nm;
            return (int)nm;
        }
    }
    return -1;
}

/* ------------------------------------------------------------ buttons ------- */

static void btn_port_pin(const ArmProfile* p, int btn, uint8_t* port, uint8_t* pin) {
    switch(btn) {
    case B_UP: *port = p->up_port; *pin = p->up_pin; break;
    case B_DOWN: *port = p->down_port; *pin = p->down_pin; break;
    case B_OK: *port = p->ok_port; *pin = p->ok_pin; break;
    case B_BACK: *port = p->back_port; *pin = p->back_pin; break;
    case B5: *port = p->b5_port; *pin = p->b5_pin; break;
    case B6: *port = p->b6_port; *pin = p->b6_pin; break;
    default: *port = 0; *pin = 0; break;
    }
}

void emu_set_button(EmuState* app, int btn, bool pressed) {
    if(btn < 0 || btn >= BTN_COUNT) return;
    uint8_t port, pin;
    btn_port_pin(app->prof, btn, &port, &pin);
    bool was = ((app->gpio_din[port] >> pin) & 1u) == 0u; /* prev pressed? */
    if(pressed)
        app->gpio_din[port] &= (uint16_t)~(1u << pin); /* 0 = pressed */
    else
        app->gpio_din[port] |= (uint16_t)(1u << pin); /* 1 = released */
    app->btn_pressed[btn] = pressed ? 1 : 0;

    /* GPIO button-edge -> GPIO IRQ, delivered ONLY on the RELEASE edge.
     *
     * MEASURED FACT (root-cause fix validated on the G82 against both firmwares
     * and the Unicorn oracle): NEITHER Pandora firmware uses the EFM32 GPIO EXTI
     * path for the nav buttons -- it NEVER writes GPIO_IEN (0x40006110) and NEVER
     * reads the EXTI IF register (0x40006114); it scans the button DIN ports by
     * polling. Raising the GPIO IRQ on BOTH edges made the firmware process the
     * key while the pin was still held (press edge), double/mis-detecting and
     * giving erratic navigation (pandora 4/10). A real keyfob commits a short
     * click on RELEASE, not on first contact. Nudging the scanner on the RELEASE
     * edge ONLY reproduces the oracle exactly (pandora 7/10, pmax 6/10,
     * deterministic 3/3) -- usable, hardware-faithful navigation. The latch +
     * parity IRQ are kept (and gated by the firmware's own ISER at dispatch) so a
     * firmware that DID enable EXTI would still be served. */
    if(was != pressed && !pressed) { /* RELEASE edge only */
        app->gpio_if |= (uint16_t)(1u << pin);
        if(pin & 1u) app->pending_gpio_odd = 1;
        else         app->pending_gpio_even = 1;
    }
}

static void init_buttons_released(EmuState* app) {
    for(int i = 0; i < 16; i++) app->gpio_din[i] = 0xFFFF;
    for(int b = 0; b < BTN_COUNT; b++) app->btn_pressed[b] = 0;
}

/* ------------------------------------------------------------ RF bridge ----- */

/* TX capture: the firmware bit-bangs the TX-data pin (PB3). Each level change
 * becomes an edge with its duration (measured in instructions ~ us). */
static void tx_capture_data_edge(EmuState* app, bool level) {
    if(level == app->tx_last_level) return;
    uint64_t now = app->ninsn;
    uint32_t dur = (uint32_t)(now - app->tx_last_insn);
    uint32_t next = (app->tx_head + 1u) % TX_RING_LEN;
    if(next != app->tx_tail) { /* ring not full */
        app->tx_ring[app->tx_head].level = app->tx_last_level;
        app->tx_ring[app->tx_head].duration = dur ? dur : 1u;
        app->tx_head = next;
    }
    app->tx_last_level = level;
    app->tx_last_insn = now;
}

static void tx_on_si_mode(EmuState* app, int new_mode) {
    if(new_mode == PROTO_RF_TX) {
        app->tx_capturing = true;
        app->tx_last_level =
            (app->gpio_dout[app->prof->tx_data_port] >> app->prof->tx_data_pin) & 1u;
        app->tx_last_insn = app->ninsn;
    } else {
        app->tx_capturing = false;
    }
    app->rf_mode = (uint8_t)new_mode;
}

bool emu_tx_pop_edge(EmuState* app, RfEdge* out) {
    if(app->tx_tail == app->tx_head) return false;
    *out = app->tx_ring[app->tx_tail];
    app->tx_tail = (app->tx_tail + 1u) % TX_RING_LEN;
    return true;
}

/* Agnostic RX injection, ported from pandora_tui.py's hardware-faithful path:
 * the edge is presented on the REAL RX-data pin (PB0 = data_port.data_pin) and
 * the pulse WIDTH is written to TIMER1's capture CNT (rx_pulse_ticks), which the
 * firmware's own demod reads via TIMER1_CNT (0x40010424) -- see EMU_MMIO_UNIFIED
 * section 7. We DO NOT call the firmware's demod ISR by address, and we DO NOT
 * poke any profile RAM offset. The firmware then detects/processes the edge
 * through its own GPIO/EXTI + TIMER1-capture path, driven by the NVIC scheduler
 * (TIMER1 = IRQ12), exactly like real hardware. Agnostic for ANY firmware.
 *
 * duration_us -> CNT ticks: the Python core keeps ~6 ticks/us (HFPERCLK/presc
 * proxy); we mirror that. The level is held on PB0 until the next edge. A few
 * foreground bursts let the firmware poll/service the edge before we return. */
void emu_rx_inject_edge(EmuState* app, bool level, uint32_t duration_us) {
    const ArmProfile* p = app->prof;
    uint8_t port = p->data_port, pin = p->data_pin;

    /* present the edge on the real RX-data pin (DIN) */
    if(level)
        app->gpio_din[port] |= (uint16_t)(1u << pin);
    else
        app->gpio_din[port] &= (uint16_t)~(1u << pin);

    /* set TIMER1 capture CNT = pulse width (the demod reads TIMER1_CNT) */
    app->rx_pulse_ticks = (duration_us * 6u) & 0xFFFFu;
    app->rx_feeding = true;
    app->rx_primed = true;

    /* let the firmware's own foreground + TIMER1/GPIO IRQ path service the edge.
     * Pure time (bursts); no PC/address sync. */
    for(int i = 0; i < 8; i++) emu_run_burst(app, 20000u);

    app->rx_feeding = false;
}

/* ------------------------------------------------------------ MMIO read ----- */

static uint32_t mmio_read(void* ctx, uint32_t addr, int size) {
    EmuState* app = (EmuState*)ctx;
    uint32_t base = addr & 0xFFFFF000u;
    uint32_t off = addr & 0xFFFu;
    uint32_t v = 0;

    /* Flash is a DIRECT region now (RAM-resident), so flash reads do NOT reach
     * this callback. If one ever does (above flash_used), return open-bus 0xFF. */
    if(addr < 0x42000u) {
        if(addr < app->flash_used) {
            /* unaligned/odd-size fallback (direct region handles the fast path) */
            uint32_t r = 0;
            for(int i = 0; i < size; i++) {
                uint32_t a = addr + (uint32_t)i;
                r |= (uint32_t)(a < app->flash_used ? app->flash[a] : 0xFF) << (8 * i);
            }
            return r;
        }
        return (size == 4) ? 0xFFFFFFFFu : (size == 2 ? 0xFFFFu : 0xFFu);
    }
    if(base == 0x4000C000u) { /* USART0 (SPI Si4432 + OLED) */
        if(off == 0x10) {
            v = (1u << 6) | (1u << 5); /* TXBL + RXDATAV ready */
        } else if(off == 0x1C) { /* RXDATA */
            if(app->spi_rx_head != app->spi_rx_tail) {
                v = app->spi_rx[app->spi_rx_head];
                app->spi_rx_head = (uint8_t)((app->spi_rx_head + 1) % SPI_RX_QUEUE_LEN);
            }
        }
    } else if(base == 0x400C4000u) { /* 2nd USART (EEPROM/config) */
        if(off == 0x10) v = (1u << 6) | (1u << 5);
    } else if(addr == 0x40006114u) { /* GPIO EXTI IF (interrupt flags) */
        v = app->gpio_if;
    } else if(addr >= 0x40006000u && addr < 0x40006000u + 36u * 16u) { /* GPIO */
        uint32_t port = (addr - 0x40006000u) / 36u;
        uint32_t poff = (addr - 0x40006000u) - port * 36u;
        if(poff == 0x1C && port < 16) v = app->gpio_din[port]; /* DIN */
    } else if(base == 0x40002000u) { /* ADC0 (battery) */
        if(off == 0x08) v = (1u << 16); /* SINGLEDV ready */
        else if(off == 0x24) v = 0x0C00; /* data ~3.9V */
    } else if(base == 0x400C8000u) { /* CMU (clock mgmt) */
        /* +0x2C STATUS/SYNCBUSY (clock-select ready): bits 3,7,9 = 1 (0x288).
         * +0x50 SYNCBUSY: 0 (bit0=0 = not busy). Both are busy-wait polls that
         * otherwise spin forever. All other CMU reads -> 0. */
        if(off == 0x2Cu)
            v = (1u << 3) | (1u << 7) | (1u << 9); /* 0x288 clock-ready */
        else
            v = 0; /* incl. +0x50 SYNCBUSY = 0 */
    } else if(base == 0x40080000u) { /* osc-ready 2nd block */
        /* +0x24 STATUS (osc ready): the init polls `and #7; bne`, so 0 = ready. */
        v = 0;
    } else if(base == 0x400C0000u) { /* EMU (energy mode / regulator) */
        /* +0x1C STATUS: report regulator-ready in case the fw polls it. */
        v = (off == 0x1Cu) ? 0x1u : 0u;
    } else if(base == 0x400C2000u || base == 0x400C3000u) { /* RMU / control */
        /* 0x400C202C read as 0xFFFFFFFF (reset-cause clear pattern); rest 0. */
        v = (off == 0x2Cu) ? 0xFFFFFFFFu : 0u;
    } else if(base == 0x400E0000u) { /* PRS */
        /* +0x08 polled with `and #1`; 0 lets the poll pass. */
        v = 0;
    } else if(base == 0x40010000u) { /* TIMER0..3 */
        uint32_t tn = off >> 10;
        uint32_t sub = off & 0x3FFu;
        if(sub == 0x24 && tn < 4) { /* CNT (free-running) */
            if(tn == 1 && app->rx_feeding) return app->rx_pulse_ticks & 0xFFFFu;
            uint32_t top = app->timer_top[tn] ? app->timer_top[tn] : 0xFFFFu;
            uint64_t elapsed =
                (app->ninsn - app->timer_t0[tn]) * (uint64_t)app->timer_rate[tn];
            v = (uint32_t)(elapsed % ((uint64_t)top + 1u));
        } else if(sub == 0x3C && tn < 4) { /* overflow proxy */
            uint32_t top = app->timer_top[tn] ? app->timer_top[tn] : 0xFFFFu;
            uint64_t elapsed =
                (app->ninsn - app->timer_t0[tn]) * (uint64_t)app->timer_rate[tn];
            v = (elapsed >= top) ? 1u : 0u;
        }
    }
    (void)size;
    return v;
}

/* ------------------------------------------------------------ OLED capture -- */

#define OLED_DC_PORT 0u
#define OLED_DC_PIN 9u /* A9: HIGH=command, LOW=pixel data */

static int oled_dc_level(EmuState* app) {
    return (app->gpio_dout[OLED_DC_PORT] >> OLED_DC_PIN) & 1u;
}

static void oled_commit(EmuState* app) {
    if(!app->oled_work_dirty) return;
    bool any = false;
    for(int i = 0; i < OLED_FB_SIZE; i++) {
        if(app->oled_work[i]) { any = true; break; }
    }
    if(!any) return; /* blank frame = internal clear; do not make it visible */
    memcpy(app->oled_fb, app->oled_work, OLED_FB_SIZE);
    app->oled_work_dirty = 0;
    app->oled_frames++;
}

static void oled_cmd(EmuState* app, uint8_t b) {
    uint8_t c = b;
    if(c < OLED_PAGES) { /* 0x00..0x0F = raw page number */
        app->oled_page = c;
        app->oled_col = 0;
    } else if(c == 0x60 || c == 0x70) { /* set column -> col 0 */
        app->oled_col = 0;
    } else if(c == 0xAF) { /* display ON = end of flush -> commit */
        oled_commit(app);
    }
}

static void oled_data(EmuState* app, uint8_t b) {
    if(app->oled_page < OLED_PAGES && app->oled_col < OLED_COLS) {
        uint32_t idx = (uint32_t)app->oled_page * OLED_COLS + app->oled_col;
        if(idx < (uint32_t)OLED_FB_SIZE) {
            app->oled_work[idx] = b;
            app->oled_work_dirty = 1;
        }
    }
    if(app->oled_col < 0xFF) app->oled_col++;
}

/* ------------------------------------------------------------ SPI byte ------ */

static void spi_byte(EmuState* app, uint8_t b) {
    /* Bus routing by CHIP SELECT: OLED and Si4432 share USART0. Si4432 CS = PE9
     * (active-high). CS LOW -> OLED (cmd vs data by DC pin A9). CS HIGH -> Si4432. */
    uint8_t si_cs =
        (uint8_t)((app->gpio_dout[app->prof->rf_cs_port] >> app->prof->rf_cs_pin) & 1u);

    if(!si_cs) {
        app->spi_phase = -1; /* abort any partial Si4432 transaction */
        if(oled_dc_level(app))
            oled_cmd(app, b);
        else
            oled_data(app, b);
        return;
    }

    /* Si4432 transaction */
    if(app->spi_phase < 0) {
        app->spi_kind = (b & 0x80) ? 'w' : 'r';
        app->spi_reg = b & 0x7F;
        app->spi_phase = 0;
        /* junk echo when sending the address */
        app->spi_rx[app->spi_rx_tail] = 0;
        app->spi_rx_tail = (uint8_t)((app->spi_rx_tail + 1) % SPI_RX_QUEUE_LEN);
    } else {
        if(app->spi_kind == 'w') {
            uint8_t wreg = app->spi_reg;
            int nm = si4432_write(&app->si, wreg, b);
            if(wreg == 0x75 || wreg == 0x76 || wreg == 0x77) {
                /* Si4432 carrier: fb[4:0]=REG75[4:0], hbsel=REG75[5],
                 * fc[15:0]=REG76/77. Preserve the fractional channel term;
                 * the previous approximation reported the wrong frequency. */
                uint8_t reg75 = app->si.regs[0x75];
                uint8_t fb = reg75 & 0x1F;
                uint8_t hbsel = (reg75 >> 5) & 1;
                uint32_t fc = ((uint32_t)app->si.regs[0x76] << 8) | app->si.regs[0x77];
                uint64_t multiplier = (uint64_t)hbsel + 1u;
                app->rf_freq_hz = (uint32_t)(
                    10000000ull * multiplier * ((uint64_t)fb + 24u) +
                    (uint64_t)fc * 10000000ull * multiplier / 64000u);
            } else if(wreg == 0x71 || wreg == 0x72) {
                uint8_t mt = app->si.regs[0x71] & 0x03;
                app->rf_mod = mt; /* 0 unmod 1 OOK 2 FSK 3 GFSK */
            }
            if(nm >= 0) {
                tx_on_si_mode(app, nm);
            }
        } else {
            uint8_t rv = si4432_read(&app->si, app->spi_reg);
            app->spi_rx[app->spi_rx_tail] = rv;
            app->spi_rx_tail = (uint8_t)((app->spi_rx_tail + 1) % SPI_RX_QUEUE_LEN);
        }
        app->spi_phase = -1;
    }
}

/* ------------------------------------------------------------ MMIO write ---- */

static void mmio_write(void* ctx, uint32_t addr, uint32_t val, int size) {
    EmuState* app = (EmuState*)ctx;
    uint32_t base = addr & 0xFFFFF000u;
    uint32_t off = addr & 0xFFFu;
    if(base == 0x4000C000u && off == 0x34) { /* USART0 TXDATA -> SPI */
        spi_byte(app, (uint8_t)(val & 0xFF));
    } else if(addr == 0x4000611Cu) { /* GPIO EXTI IFC: clear interrupt flags */
        app->gpio_if &= (uint16_t)~(val & 0xFFFFu);
    } else if(addr == 0x40006118u) { /* GPIO EXTI IFS: set interrupt flags (SW) */
        app->gpio_if |= (uint16_t)(val & 0xFFFFu);
    } else if(addr >= 0x40006000u && addr < 0x40006000u + 36u * 16u) { /* GPIO DOUT* */
        uint32_t port = (addr - 0x40006000u) / 36u;
        uint32_t poff = (addr - 0x40006000u) - port * 36u;
        if(port < 16) {
            uint16_t before = app->gpio_dout[port];
            if(poff == 0x10) {
                app->gpio_dout[port] &= (uint16_t)~val; /* DOUTCLR */
            } else if(poff == 0x14) {
                app->gpio_dout[port] |= (uint16_t)val; /* DOUTSET */
            } else if(poff == 0x0C) {
                app->gpio_dout[port] = (uint16_t)val; /* DOUT */
            }
            /* Pandora's firmware buzzer is GPIO A12 (confirmed by the internal
             * emulator's toggle trace). Defer sound handling to the frontend. */
            if(port == 0 && ((before ^ app->gpio_dout[port]) & (1u << 12)))
                app->buzzer_event = true;
            /* TX capture: firmware bit-bangs the TX-data pin (PB3) in direct mode. */
            if(app->tx_capturing && port == app->prof->tx_data_port) {
                bool level = (app->gpio_dout[port] >> app->prof->tx_data_pin) & 1u;
                tx_capture_data_edge(app, level);
            }
        }
    } else if(base == 0x40010000u) { /* TIMER0..3: track CNT reset + TOP */
        uint32_t tn = off >> 10;
        uint32_t sub = off & 0x3FFu;
        if(tn < 4) {
            if(sub == 0x24) {
                app->timer_t0[tn] = app->ninsn; /* re-anchor CNT */
            } else if(sub == 0x3C) {
                app->timer_top[tn] = val ? val : 0xFFFFu; /* TOP */
            }
        }
    } else if(addr == 0xE000ED08u) {
        /* SCB->VTOR: the firmware installs its active vector table base. Store it
         * so cur_vtor()/the NVIC dispatch honour the DYNAMIC table (flash-XIP or
         * RAM). Agnostic: no assumption about where the app ends up. */
        app->ppb_vtor = val;
    } else if(addr == 0xE000E100u) {
        /* NVIC ISER0 (set-enable, IRQ0..31): writing 1 ENABLES IRQ N. Track the
         * live enabled mask so the scheduler only fires IRQs the firmware really
         * wants. Ported from pandora_tui.py _on_ppb_write. */
        app->nvic_enabled |= (uint64_t)(val & 0xFFFFFFFFu);
    } else if(addr == 0xE000E104u) {
        /* NVIC ISER1 (set-enable, IRQ32..63): this MCU uses up to ~IRQ36. */
        app->nvic_enabled |= (uint64_t)(val & 0xFFFFFFFFu) << 32;
    } else if(addr == 0xE000E180u) {
        /* NVIC ICER0 (clear-enable, IRQ0..31): writing 1 DISABLES IRQ N. */
        app->nvic_enabled &= ~((uint64_t)(val & 0xFFFFFFFFu));
    } else if(addr == 0xE000E184u) {
        /* NVIC ICER1 (clear-enable, IRQ32..63). */
        app->nvic_enabled &= ~((uint64_t)(val & 0xFFFFFFFFu) << 32);
    }
    (void)size;
}

/* ------------------------------------------------------------ minimal NVIC --
 *
 * AGNOSTIC NVIC, ported from pandora_tui.py (_irq_scheduler_tick / _enter_irq /
 * _on_ppb_write). The old C port had two bugs: (1) it read VTOR via a callback
 * that returned 0, so IRQs NEVER actually fired (the UI render came entirely
 * from the knock PC-sync + foreground loop); (2) it round-robined {12,13,14,1}
 * blindly with a hardcoded default-handler address (0x82DB). Both are fixed
 * here: VTOR is honoured (stored in ppb_vtor), only IRQs the firmware ENABLED in
 * the real ISER fire, and the default/weak handler is detected GENERICALLY by
 * the branch-to-self opcode, not an address. */

static uint32_t cur_vtor(EmuState* app) {
    return app->ppb_vtor;
}

static uint32_t irq_handler_addr(EmuState* app, int irqn) {
    uint32_t vt = cur_vtor(app);
    uint32_t off = vt + 0x40u + (uint32_t)irqn * 4u;
    return thumb_read_mem(app->cpu, off, 4);
}

/* EFM32 Gecko TIMER IRQ lines (TIMER0..2 = IRQ12/13/14). These are IRQ NUMBERS
 * (indices into the VTOR-pointed vector table), NOT firmware code addresses. The
 * UI refresh tick of both firmwares is IRQ13 (TIMER2). IRQ1 is intentionally
 * EXCLUDED (firing an always-on helper on every tick floods the fw when it masks
 * IRQ13 in a critical section -> pmax UI livelock). Same as the Python core. */
static const int TIMER_IRQS[] = {12, 13, 14};

/* Try to enter the exception for IRQ 'irqn'. Returns true if it actually entered
 * a VALID handler. Rejects: empty/0 vectors, handlers outside flash/RAM, and the
 * default/weak handler (a `b .` self-branch, opcode 0xE7FE) -- all detected
 * GENERICALLY (no firmware address hardcoded). Ported from _enter_irq. */
static bool try_enter_irq(EmuState* app, int irqn) {
    uint32_t handler = irq_handler_addr(app, irqn);
    uint32_t h = handler & ~1u;
    bool in_flash = (h < PA_FLASH_BASE + app->flash_used);
    bool in_ram = (h >= PA_RAM_BASE && h < PA_RAM_BASE + PA_RAM_SIZE);
    if(!handler || !(in_flash || in_ram)) return false;
    /* skip a branch-to-self default handler (infinite loop) */
    uint16_t op = (uint16_t)thumb_read_mem(app->cpu, h, 2);
    if(op == 0xE7FEu) return false;
    app->tick_count++;
    app->in_irq = 1;
    thumb_enter_exception(app->cpu, handler, THUMB_EXC_RETURN_MSP_THREAD,
                          (uint32_t)(irqn + 16));
    app->next_pc = thumb_get_reg(app->cpu, 15);
    return true;
}

/* Fire the NEXT valid TIMER IRQ the firmware has ENABLED now in the real ISER.
 * Walk TIMER_IRQS in round-robin, skipping IRQs not enabled in nvic_enabled and
 * ones try_enter_irq rejects. If none is valid this tick (fw masked the timers
 * in a critical section), fire NOTHING (no hang, no flood). If the fw has not
 * written any ISER yet (mask==0, very early boot), don't filter by ISER and let
 * try_enter_irq be the only judge. Ported from _irq_scheduler_tick. */
static void nvic_tick(EmuState* app) {
    if(app->in_irq) return;
    uint32_t vtor = cur_vtor(app);
    if(vtor < 0x8000u) return; /* table not installed yet (pre-app) */
    int n = (int)(sizeof(TIMER_IRQS) / sizeof(TIMER_IRQS[0]));
    uint64_t mask = app->nvic_enabled;
    bool use_mask = (mask != 0);
    for(int k = 0; k < n; k++) {
        int irqn = TIMER_IRQS[app->irq_rr % n];
        app->irq_rr++;
        if(use_mask && !(mask & ((uint64_t)1u << irqn))) continue; /* not enabled now */
        if(try_enter_irq(app, irqn)) return;
    }
    /* no valid slot this tick */
}

/* GPIO EXTI delivery: if a button pin edge raised a pending GPIO interrupt and
 * the firmware has the matching GPIO IRQ ENABLED (IRQ1=GPIO_EVEN for even pins,
 * IRQ11=GPIO_ODD for odd pins), enter its ISR so the firmware processes the
 * button. The ISR reads the EXTI IF register (0x40006114) and clears it via IFC.
 * Returns true if an ISR was entered. AGNOSTIC: triggered by the observed button
 * pin edge, gated by the firmware's own ISER; no firmware code address used. */
static bool gpio_tick(EmuState* app) {
    if(app->in_irq) return false;
    uint32_t vtor = cur_vtor(app);
    if(vtor < 0x8000u) return false;
    uint64_t mask = app->nvic_enabled;
    bool use_mask = (mask != 0);
    if(app->pending_gpio_odd) {
        if(!use_mask || (mask & ((uint64_t)1u << 11))) {
            if(try_enter_irq(app, 11)) { app->pending_gpio_odd = 0; return true; }
        }
    }
    if(app->pending_gpio_even) {
        if(!use_mask || (mask & ((uint64_t)1u << 1))) {
            if(try_enter_irq(app, 1)) { app->pending_gpio_even = 0; return true; }
        }
    }
    return false;
}

/* ------------------------------------------------------------ WFI scan ------
 * Scan the code for WFI(0xBF30)/WFE(0xBF20) opcodes and remember their absolute
 * addresses. AGNOSTIC: we scan the whole code space (flash from 0x40, past the
 * vector table) and -- after boot -- the RAM too (for app-in-RAM loaders like
 * pmax). No firmware address is assumed. Ported from _find_wfi/_rescan_wfi_ram.
 * The old code hardcoded a scan start of 0x8100 (pandora's app base), which was
 * NOT agnostic. */
static void wfi_add(EmuState* app, uint32_t addr) {
    if(app->wfi_count >= app->wfi_cap) {
        uint32_t ncap = app->wfi_cap ? app->wfi_cap * 2u : 64u;
        uint32_t* nb = (uint32_t*)realloc(app->wfi_set, ncap * sizeof(uint32_t));
        if(!nb) return;
        app->wfi_set = nb;
        app->wfi_cap = ncap;
    }
    app->wfi_set[app->wfi_count++] = addr;
}

static bool wfi_contains(EmuState* app, uint32_t addr) {
    for(uint32_t i = 0; i < app->wfi_count; i++)
        if(app->wfi_set[i] == addr) return true;
    return false;
}

static void scan_wfi_buf(EmuState* app, const uint8_t* buf, uint32_t base,
                         uint32_t len, uint32_t start) {
    for(uint32_t off = start; off + 1u < len; off += 2) {
        uint16_t hw = (uint16_t)(buf[off] | (buf[off + 1] << 8));
        if(hw == 0xBF30u || hw == 0xBF20u) wfi_add(app, base + off);
    }
}

static void find_wfi(EmuState* app) {
    app->wfi_count = 0;
    uint32_t lim = app->flash_used < 0x20000u ? app->flash_used : 0x20000u;
    /* start past the initial vector table (0x40) so vector data is not mistaken
     * for opcodes, like the Python core. */
    scan_wfi_buf(app, app->flash, PA_FLASH_BASE, lim, 0x40u);
    app->wfi_pc_hint = app->wfi_count ? app->wfi_set[0] : 0xFFFFFFFFu;
    app->wfi_ram_scanned = 0;
}

static void rescan_wfi_ram(EmuState* app) {
    /* RAM holds the app (and its WFI) when a loader copied code there (pmax). */
    scan_wfi_buf(app, app->ram, PA_RAM_BASE, PA_RAM_SIZE, 0u);
}

/* ------------------------------------------------------------ unlock signal -
 * AGNOSTIC: a locked keyfob shows a BLACK screen; once unlocked the firmware
 * draws its UI. We derive 'unlocked' from the OBSERVABLE framebuffer having lit
 * pixels, NOT from a firmware code address (the old code hooked unlock_pc).
 * Ported from pandora_tui.py @property unlocked. */
bool emu_is_unlocked(EmuState* app) {
    for(int i = 0; i < OLED_FB_SIZE; i++)
        if(app->oled_fb[i]) return true;
    return false;
}

/* ------------------------------------------------------------ code hook ----- */

static void code_hook(void* ctx, uint32_t pc) {
    EmuState* app = (EmuState*)ctx;
    app->ninsn++;
    app->pc = pc;
    /* AGNOSTIC WFI skip (ported from _on_code): if the firmware parks in a
     * WFI/WFE, step over it so the foreground polling loop keeps spinning and
     * the NVIC tick can advance. Detected by O(1)-ish lookup in the scanned set
     * (no per-firmware address). */
    if(app->wfi_count && wfi_contains(app, pc & ~1u)) {
        /* advance PC past the 16-bit WFI/WFE so the core does not re-park. */
        thumb_set_reg(app->cpu, 15, (pc & ~1u) + 2u + 1u);
    }
}

/* ------------------------------------------------------------ execution ----- */

/* Execute ~count instructions of REAL firmware, INTERLEAVING NVIC ticks at a
 * realistic cadence. Ported from pandora_tui.py run_burst: it LOOPS, running the
 * code in sub-bursts and, when a tick is due (stuck in WFI, or periodically),
 * enters the TIMER handler, then CONTINUES the rest of the budget. The bug in
 * the old C port was checking the tick only ONCE per call, so with 50k-insn
 * callers the IRQ13 UI tick fired ~7x too slowly and the firmware's button
 * scanner never caught the injected key edges (UI stayed black). AGNOSTIC: ticks
 * are IRQ numbers via the dynamic VTOR; nothing is firmware-address specific. */
#define RUN_SUBBURST 15000u

void emu_run_burst(EmuState* app, uint32_t count) {
    uint32_t remaining = count;
    int guard = 0;
    while(remaining > 0 && guard < 512) {
        guard++;
        bool app_running = cur_vtor(app) >= 0x8000u;
        /* Once the app is running, rescan RAM ONCE for WFI/WFE in case a loader
         * copied the app (and its WFI) to SRAM (app-in-RAM, e.g. pmax). */
        if(app_running && !app->wfi_ram_scanned) {
            rescan_wfi_ram(app);
            app->wfi_ram_scanned = 1;
        }
        bool stuck_wfi =
            app_running && app->wfi_count && wfi_contains(app, app->next_pc & ~1u);
        bool periodic = (app->ninsn - app->last_tick_insn) > RUN_SUBBURST;
        /* GPIO EXTI first: deliver a pending button-edge interrupt BEFORE the
         * periodic timer tick so a press is processed promptly (not starved by
         * the round-robin timer scheduler). Only fires on a real button pin edge
         * when the firmware enabled the GPIO IRQ. */
        if(app_running && !app->in_irq &&
           (app->pending_gpio_even || app->pending_gpio_odd)) {
            gpio_tick(app);
        }
        /* Match Unicorn run_burst: fire a tick when a per-burst pending_tick was
         * raised by a driver (press/boot), when stuck in WFI, or periodically.
         * The pending_tick is consumed on the first sub-iteration that ticks. */
        if(app_running && !app->in_irq && (app->pending_tick || stuck_wfi || periodic)) {
            app->pending_tick = 0;
            app->last_tick_insn = app->ninsn;
            nvic_tick(app);
        }
        uint32_t step = remaining < RUN_SUBBURST ? remaining : RUN_SUBBURST;
        uint64_t before = app->ninsn;
        thumb_set_reg(app->cpu, 15, app->next_pc);
        /* Run until 'step' CODE-HOOK-COUNTED instructions have executed, matching
         * Unicorn's emu_start(count=step): QEMU's instruction budget counts the
         * same executed instructions the code hook sees (an IT-block instruction
         * whose condition fails is a NOP that neither QEMU nor this core's hook
         * counts). The old loop iterated 'step' thumb_step() CALLS instead, so an
         * IT-skipped NOP made a sub-burst consume fewer counted instructions than
         * Unicorn (observed: the very first sub-burst consumed 14996 vs 15000),
         * drifting every subsequent sub-burst boundary by a few instructions. That
         * phase drift landed the periodic NVIC tick at a different code position
         * than the oracle, which cascaded (via the TIMER0-CNT delay loops) into
         * total state divergence and broke cumulative navigation. Counting by
         * ninsn delta keeps the sub-burst boundaries -- and thus the tick phase --
         * bit-aligned with Unicorn. A hard guard (2*step thumb_step calls) bounds
         * the worst case where a run is dominated by IT-skipped NOPs. */
        uint32_t guard2 = 0;
        uint32_t guard_lim = step * 2u + 16u;
        while((uint32_t)(app->ninsn - before) < step && guard2 < guard_lim) {
            guard2++;
            int rc = thumb_step(app->cpu);
            if(rc < 0) break;
            uint32_t pc = thumb_get_reg(app->cpu, 15);
            if((pc & 0xFFFFFFF0u) == 0xFFFFFFF0u) { /* EXC_RETURN */
                thumb_exc_return(app->cpu, pc);
                app->in_irq = 0;
                break;
            }
        }
        app->next_pc = thumb_get_reg(app->cpu, 15);
        uint64_t consumed = app->ninsn - before;
        remaining -= (uint32_t)(consumed > 0 ? (consumed < remaining ? consumed : remaining) : 1);
    }
    /* keep the observable unlock flag up to date (no code-address hook) */
    if(!app->unlocked && emu_is_unlocked(app)) app->unlocked = 1;

    /* Raise a UI tick request for the NEXT burst. The reference (Unicorn
     * pandora_tui) sets self._pending_tick=True after EVERY run_burst call it
     * makes (boot settle, knock presses, button presses, the TUI/idle loop -- all
     * of them, without exception; see pandora_tui.py). That guarantees each burst
     * boundary forces exactly one timer IRQ tick in addition to the periodic
     * ones. Centralising it here reproduces that cadence for every caller
     * (auto_unlock, emu_press_button, and the FFI/UART run path) without the
     * pattern having to be repeated at each call site, and keeps the IRQ13 UI
     * tick in phase with Unicorn so the post-unlock navigation path matches. */
    app->pending_tick = 1;
}

static uint32_t fb_hash(EmuState* app) {
    uint32_t h = 2166136261u;
    for(uint32_t i = 0; i < (uint32_t)OLED_FB_SIZE; i++) {
        h ^= app->oled_fb[i];
        h *= 16777619u;
    }
    return h;
}

/* Press+release ONE button PURELY over GPIO with realistic timing, exactly like
 * the hardware / pandora_tui.py press_button. Drive the DIN pin low (pressed),
 * run the firmware a while (its own polling scan -- powered by the faithful
 * TIMERs + NVIC -- sees the pin), then drive it high (released). The firmware
 * distinguishes short click vs long press by the TIME the pin was low, like a
 * real finger. AGNOSTIC: no PC/code-address sync (no scan_head/nav_reader). */
/* Parameterised press+release, mirroring Unicorn pandora_tui.press_button exactly
 * (down-phase burst = 50000, settle-phase burst = 60000, adaptive early-out when
 * the UI redraws after >=120000 settle insns). down_insn/settle_insn are the
 * HARDWARE-timing proxies (short click vs long hold); the knock uses the SAME
 * mechanism as a regular press (just different durations), which is what keeps the
 * IRQ-tick phase at nav time identical to the oracle. */
static bool press_button_timed(EmuState* app, int btn, uint64_t down_insn,
                               uint64_t settle_insn) {
    if(btn < 0 || btn >= BTN_COUNT) return false;

    /* 1) press (pin low) */
    emu_set_button(app, btn, true);
    uint64_t done = 0;
    while(done < down_insn) {
        emu_run_burst(app, BTN_BURST);
        done += BTN_BURST;
    }
    /* 2) release (pin high) -> firmware dispatches the action on the edge */
    emu_set_button(app, btn, false);

    /* 3) adaptive settle: run until the UI redraws (fb changes), bounded. */
    uint32_t prev = fb_hash(app);
    done = 0;
    while(done < settle_insn) {
        emu_run_burst(app, 60000u);
        done += 60000u;
        uint32_t cur = fb_hash(app);
        if(cur != prev && done >= 120000u) break; /* redrew: early-out */
        prev = cur;
    }
    return true;
}

bool emu_press_button(EmuState* app, int btn, bool hold) {
    if(btn < 0 || btn >= BTN_COUNT) return false;
    return press_button_timed(app, btn, hold ? BTN_HOLD_INSN : BTN_PRESS_INSN,
                              BTN_SETTLE_INSN);
}

/* COMPAT alias (main.c calls emu_nav_button). Now a pure-GPIO press, no PC-sync. */
bool emu_nav_button(EmuState* app, int btn, bool hold) {
    return emu_press_button(app, btn, hold);
}

void emu_set_knock(EmuState* app, const uint8_t* seq, uint8_t len, int gate) {
    if(len > sizeof(app->knock_seq)) len = (uint8_t)sizeof(app->knock_seq);
    app->knock_len = len;
    if(seq && len) memcpy(app->knock_seq, seq, len);
    app->knock_gate = gate;
}

/* Press the knock-code PURELY over GPIO as an autotest (knock = FIRMWARE CONFIG,
 * provided by the host, NOT baked into the agnostic core). Mirrors
 * pandora_tui.py auto_unlock: settle the boot with FINE bursts (so the keyboard
 * scanner reaches a phase where it samples all ports, incl. the B5 gate port),
 * then press the sequence with brief clicks, then hold the gate. 'unlocked' is
 * the OBSERVABLE display signal. Returns true if the display drew content. */
static bool auto_unlock(EmuState* app) {
    if(app->knock_len == 0) {
        /* no knock configured: just settle the boot and report the observable. */
        uint64_t done = 0;
        while(done < KNOCK_BOOT_SETTLE_INSN) {
            emu_run_burst(app, 20000u);
            done += 20000u;
        }
        app->unlocked = emu_is_unlocked(app) ? 1 : 0;
        return app->unlocked != 0;
    }

    /* 1) settle the boot with FINE bursts so the scanner samples every port */
    uint64_t done = 0;
    while(done < KNOCK_BOOT_SETTLE_INSN) {
        emu_run_burst(app, 20000u);
        done += 20000u;
    }
    /* 2) press the knock sequence as short clicks. Use the SAME press mechanism
     * as a regular button press (press_button_timed), exactly like the reference
     * (Unicorn auto_unlock -> press_button(press_insn=150000, settle=200000)).
     * The old code hand-rolled the click with 20000-insn bursts, a different
     * burst granularity than the oracle, which shifted the IRQ-tick phase by
     * ~200000 insns by the time navigation started -> post-unlock UP/OK/DOWN
     * landed at the wrong point in the tick cycle and did nothing. */
    for(uint8_t i = 0; i < app->knock_len; i++) {
        int btn = app->knock_seq[i];
        if(btn < 0 || btn >= BTN_COUNT) continue;
        press_button_timed(app, btn, KNOCK_CLICK_INSN, KNOCK_CLICK_SETTLE_INSN);
    }
    /* 3) final gate: hold it (reference: press_button(gate, hold=True,
     * hold_insn=900000, settle=400000)). */
    if(app->knock_gate >= 0 && app->knock_gate < BTN_COUNT) {
        press_button_timed(app, app->knock_gate, KNOCK_GATE_HOLD_INSN,
                           KNOCK_GATE_SETTLE_INSN);
    }
    /* 4) final settle: let the firmware finish unlocking + draw the UI */
    done = 0;
    while(done < KNOCK_FINAL_SETTLE_INSN && !emu_is_unlocked(app)) {
        emu_run_burst(app, 100000u);
        done += 100000u;
    }
    app->unlocked = emu_is_unlocked(app) ? 1 : 0;
    emu_logf("auto_unlock: knock_len=%u gate=%d -> unlocked=%d (observable)",
             app->knock_len, app->knock_gate, app->unlocked);
    return app->unlocked != 0;
}

/* ------------------------------------------------------------ framebuffer --- */

void emu_get_fb(EmuState* app, uint8_t* dst) {
    memcpy(dst, app->oled_fb, OLED_FB_SIZE);
}

/* ASCII art of the VISIBLE framebuffer (debug). 88 visible cols x 64 rows,
 * downsampled 2bpp -> on/off. One char per pixel, line-wrapped per row. */
size_t emu_fb_ascii(EmuState* app, char* out, size_t cap) {
    size_t n = 0;
    const uint8_t* fbv = app->oled_fb;
    for(int page = 0; page < OLED_PAGES; page++) {
        for(int k = 0; k < 4; k++) { /* 4 logical rows per page */
            for(int col = 0; col < OLED_VISIBLE_COLS; col++) {
                uint8_t v = fbv[page * OLED_COLS + col];
                int lvl = ((v >> (2 * k)) & 1) + ((v >> (2 * k + 1)) & 1);
                if(n + 1 < cap) out[n++] = lvl ? '#' : ' ';
            }
            if(n + 1 < cap) out[n++] = '\n';
        }
    }
    if(n < cap) out[n] = '\0';
    else if(cap) out[cap - 1] = '\0';
    return n;
}

/* ------------------------------------------------------------ boot ---------- */

void emu_reset_and_unlock(EmuState* app) {
    uint32_t sp0 = (uint32_t)(app->flash[0] | (app->flash[1] << 8) |
                              (app->flash[2] << 16) | (app->flash[3] << 24));
    uint32_t rst = (uint32_t)(app->flash[4] | (app->flash[5] << 8) |
                              (app->flash[6] << 16) | (app->flash[7] << 24));
    memset(app->ram, 0, PA_RAM_SIZE);
    thumb_reset(app->cpu, sp0, rst);
    app->next_pc = rst | 1u;
    app->ninsn = 0;
    app->in_irq = 0;
    app->irq_rr = 0;
    app->pending_tick = 0;
    app->tick_count = 0;
    app->last_tick_insn = 0;
    app->unlocked = 0;
    app->max_step = -1;
    app->nvic_enabled = 0;   /* agnostic: fw re-writes ISER during boot */
    app->ppb_vtor = 0;       /* agnostic: fw installs VTOR during boot */
    app->wfi_ram_scanned = 0;
    app->spi_phase = -1;
    app->spi_rx_head = app->spi_rx_tail = 0;
    for(int i = 0; i < 4; i++) {
        app->timer_t0[i] = 0;
        app->timer_top[i] = 0xFFFF;
        app->timer_rate[i] = (i == 0) ? 0xC00u : 0x400u;
    }
    memset(app->oled_fb, 0, OLED_FB_SIZE);
    memset(app->oled_work, 0, OLED_FB_SIZE);
    app->oled_page = 0;
    app->oled_col = 0;
    app->oled_work_dirty = 0;
    app->oled_frames = 0;
    app->rf_mode = PROTO_RF_IDLE;
    app->rf_mod = PROTO_MOD_UNMOD;
    app->rf_freq_hz = 0;
    app->tx_head = app->tx_tail = 0;
    app->tx_capturing = false;
    app->tx_last_level = false;
    app->tx_last_insn = 0;
    init_buttons_released(app);
    si4432_init(&app->si);
    app->gpio_if = 0;
    app->pending_gpio_even = 0;
    app->pending_gpio_odd = 0;
    auto_unlock(app);
}

/* ------------------------------------------------------------ init/deinit --- */

int emu_init(EmuState* app, const ArmProfile* prof, const uint8_t* fw_data,
             uint32_t fw_len) {
    memset(app, 0, sizeof(*app));
    app->prof = prof;
    app->knock_gate = -1;

    /* RAM-resident flash. AGNOSTIC: the flash size is derived from the firmware
     * image length (like the Python core's `top = max(code)+1`), NOT a hardcoded
     * per-firmware ArmProfile.flash_used. We round up to a small minimum and add
     * a little margin so odd-length images and tail reads stay in-bounds. The
     * image is a flat flash dump (gaps = 0xFF). */
    uint32_t sz = fw_len ? fw_len : 0x1000u;
    if(sz < 0x1000u) sz = 0x1000u;
    /* page-align up to 4 KB and add one page of open-bus margin */
    sz = ((sz + 0xFFFu) & ~0xFFFu) + 0x1000u;
    app->flash_used = sz;
    app->flash = (uint8_t*)malloc(app->flash_used);
    if(!app->flash) return -1;
    memset(app->flash, 0xFF, app->flash_used);
    uint32_t cpy = fw_len < app->flash_used ? fw_len : app->flash_used;
    memcpy(app->flash, fw_data, cpy);

    app->ram = (uint8_t*)malloc(PA_RAM_SIZE);
    if(!app->ram) { free(app->flash); app->flash = NULL; return -1; }
    memset(app->ram, 0, PA_RAM_SIZE);

    app->cpu = (ThumbCore*)malloc(sizeof(ThumbCore));
    if(!app->cpu) {
        free(app->ram); free(app->flash);
        app->ram = NULL; app->flash = NULL;
        return -1;
    }
    thumb_init(app->cpu);

    /* Flash AND RAM are both DIRECT regions -> everything is resident, no cache. */
    thumb_set_regions(app->cpu, app->flash, PA_FLASH_BASE, app->flash_used,
                      app->ram, PA_RAM_BASE, PA_RAM_SIZE);
    thumb_set_mem_cb(app->cpu, mmio_read, mmio_write, app);
    thumb_set_hook(app->cpu, code_hook, app);

    find_wfi(app); /* agnostic flash scan (from 0x40); RAM rescanned post-boot */
    emu_logf("emu_init: hw=%s flash_used=%lu fw_len=%lu wfi_count=%lu first=0x%06lX",
             prof->name, (unsigned long)app->flash_used, (unsigned long)fw_len,
             (unsigned long)app->wfi_count, (unsigned long)app->wfi_pc_hint);
    return 0;
}

void emu_deinit(EmuState* app) {
    if(app->cpu) { free(app->cpu); app->cpu = NULL; }
    if(app->ram) { free(app->ram); app->ram = NULL; }
    if(app->flash) { free(app->flash); app->flash = NULL; }
    if(app->wfi_set) { free(app->wfi_set); app->wfi_set = NULL; app->wfi_count = app->wfi_cap = 0; }
}
