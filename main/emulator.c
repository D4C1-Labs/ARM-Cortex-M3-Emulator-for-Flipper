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

static void emu_logf(const char* fmt, ...) {
    char buf[160];
    va_list ap;
    __builtin_va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    __builtin_va_end(ap);
    emu_log(buf);
}

/* ------------------------------------------------------------ tuning -------- */
/* Same semantics as the Flipper app. On the ESP32 @240MHz the interpreter runs
 * much faster than on the Cortex-M4 @64MHz, so these windows are generous. */
#define KNOCK_SETTLE_INSN 250000ull
#define KNOCK_STEP_BUDGET 1500000ull
#define KNOCK_RELEASE_AFTER 4

#define NAV_HOLD_READS 40u
#define NAV_LONG_READS 120u
#define NAV_BUDGET_INSN 1000000ull
#define NAV_SETTLE_INSN 180000ull
#define NAV_SETTLE_MIN_INSN 30000ull
#define NAV_SETTLE_BURST 60000u

/* ------------------------------------------------------------ HW profiles --- */

const ArmProfile PROFILE_PANDORA = {
    .name = "pandora",
    .flash_used = 0x13800u,
    .fb_addr = 0x20001152u,
    .up_port = 5, .up_pin = 7,
    .down_port = 5, .down_pin = 5,
    .ok_port = 2, .ok_pin = 9,
    .back_port = 2, .back_pin = 7,
    .b5_port = 0, .b5_pin = 1,
    .b6_port = 0, .b6_pin = 6,
    .knock = {B_UP, B_OK, B_OK, B_UP, B_UP, B_DOWN, 0xFF, 0xFF},
    .knock_len = 6,
    .gate = B5,
    .scan_head = 0xFDFAu,
    .spin_heads = {0xFE5E, 0xFE82, 0xFE9A, 0xFEB2, 0xFECC, 0xFEF4, 0xFF12, 0},
    .spin_count = 7,
    .unlock_pc = 0xD108u,
    .nav_reader = 0x97ACu,
    .demod_isr = 0x09554u,
    .rx_gate = 0x2000000Fu, .rx_carrier_gate = 0x20000009u,
    .rx_pending = 0x200005C0u, .rx_state = 0x200005C1u,
    .rx_width_cur = 0x200005D8u, .rx_width_prev = 0x200005DCu, .rx_width_prev2 = 0x200005E0u,
    .rx_struct = 0x20000000u, .rx_filter_taps = 5,
    .data_port = 1, .data_pin = 0, /* PB0 = RX-data */
    .tx_data_port = 1, .tx_data_pin = 3, /* PB3 = TX-data */
    .rf_cs_port = 4, .rf_cs_pin = 9, /* PE9 = Si4432 CS (active-high) */
    .rf_tx_cell_us = 400,
};

const ArmProfile PROFILE_PANDORAMAX = {
    .name = "pandoramax",
    .flash_used = 0x15400u,
    .fb_addr = 0x200005DAu,
    .up_port = 5, .up_pin = 7,
    .down_port = 5, .down_pin = 5,
    .ok_port = 2, .ok_pin = 9,
    .back_port = 0, .back_pin = 15,
    .b5_port = 0, .b5_pin = 1,
    .b6_port = 0, .b6_pin = 6,
    .knock = {B_UP, B_OK, B_DOWN, B_OK, B_UP, B_DOWN, 0xFF, 0xFF},
    .knock_len = 6,
    .gate = B5,
    .scan_head = 0x112E4u,
    .spin_heads = {0x1135A, 0x11382, 0x1139A, 0x113B2, 0x113CA, 0x113E2, 0x1144C, 0x11464},
    .spin_count = 8,
    .unlock_pc = 0xD6E4u,
    .nav_reader = 0x96ECu,
    .demod_isr = 0x0945Cu,
    .rx_gate = 0x20000010u, .rx_carrier_gate = 0x2000000Au,
    .rx_pending = 0x2000003Du, .rx_state = 0,
    .rx_width_cur = 0x20000054u, .rx_width_prev = 0x20000058u, .rx_width_prev2 = 0x2000005Cu,
    .rx_struct = 0x20000000u, .rx_filter_taps = 0,
    .data_port = 1, .data_pin = 0,
    .tx_data_port = 1, .tx_data_pin = 3,
    .rf_cs_port = 4, .rf_cs_pin = 9,
    .rf_tx_cell_us = 500,
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
    if(pressed)
        app->gpio_din[port] &= (uint16_t)~(1u << pin); /* 0 = pressed */
    else
        app->gpio_din[port] |= (uint16_t)(1u << pin); /* 1 = released */
    app->btn_pressed[btn] = pressed ? 1 : 0;
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

#define RX_CALL_SENTINEL 0x001FFFF0u
#define RX_CALL_SCRATCH_SP 0x20007000u

void emu_rx_inject_edge(EmuState* app, bool level, uint32_t duration_us) {
    const ArmProfile* p = app->prof;
    ThumbCore* cpu = app->cpu;
    if(!app->rx_primed) {
        app->ram[p->rx_gate - PA_RAM_BASE] = 1u;
        app->ram[p->rx_pending - PA_RAM_BASE] = 0u;
        if(p->rx_state) app->ram[p->rx_state - PA_RAM_BASE] = 0u;
        if(p->rx_filter_taps) {
            for(uint32_t off = 0x10u; off < 0x1Cu; off++)
                app->ram[p->rx_struct - PA_RAM_BASE + off] = 1u;
        }
        app->ram[p->rx_struct - PA_RAM_BASE + 9u] = 1u;
        app->ram[p->rx_struct - PA_RAM_BASE + 0xAu] = 1u;
        app->ram[p->rx_carrier_gate - PA_RAM_BASE] = 1u;
        app->rx_primed = true;
    }
    app->rx_pulse_ticks = (duration_us * 6u) & 0xFFFFu;
    app->rx_feeding = true;
    uint8_t port = p->data_port, pin = p->data_pin;
    if(level) app->gpio_din[port] |= (uint16_t)(1u << pin);
    else app->gpio_din[port] &= (uint16_t)~(1u << pin);
    app->ram[p->rx_carrier_gate - PA_RAM_BASE] = level ? 1u : 0u;

    uint32_t regs[16]; memcpy(regs, cpu->r, sizeof(regs));
    uint64_t cycles_before = cpu->cycles;
    uint32_t xpsr = cpu->xpsr;
    uint8_t it_cond = cpu->it_cond, it_mask = cpu->it_mask, halted = cpu->halted;
    ThumbHook hook = cpu->hook; void* hook_ctx = cpu->hook_ctx;
    thumb_set_hook(cpu, NULL, NULL);
    uint8_t repetitions = p->rx_filter_taps ? p->rx_filter_taps : 6u;
    for(uint8_t n = 0; n < repetitions; n++) {
        thumb_set_reg(cpu, 13, RX_CALL_SCRATCH_SP);
        thumb_set_reg(cpu, 14, RX_CALL_SENTINEL | 1u);
        thumb_set_reg(cpu, 15, p->demod_isr | 1u);
        cpu->halted = 0;
        bool returned = false;
        for(uint32_t i = 0; i < 200000u; i++) {
            if((cpu->r[15] & ~1u) == RX_CALL_SENTINEL) { returned = true; break; }
            if(thumb_step(cpu) < 0) break;
        }
        thumb_set_reg(cpu, 0, regs[0]);
        thumb_set_reg(cpu, 1, regs[1]);
        cpu->halted = halted;
        if(!returned || app->ram[p->rx_pending - PA_RAM_BASE] == 0xFFu) break;
    }
    app->ninsn += (uint32_t)(cpu->cycles - cycles_before);
    app->pc = cpu->r[15] & ~1u;
    app->rx_feeding = false;
    thumb_set_hook(cpu, hook, hook_ctx);
    memcpy(cpu->r, regs, sizeof(regs)); cpu->xpsr = xpsr;
    cpu->it_cond = it_cond; cpu->it_mask = it_mask; cpu->halted = halted;
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
    } else if(addr >= 0x40006000u && addr < 0x40006000u + 36u * 16u) { /* GPIO */
        uint32_t port = (addr - 0x40006000u) / 36u;
        uint32_t poff = (addr - 0x40006000u) - port * 36u;
        if(poff == 0x1C && port < 16) v = app->gpio_din[port]; /* DIN */
    } else if(base == 0x40002000u) { /* ADC0 (battery) */
        if(off == 0x08) v = (1u << 16); /* SINGLEDV ready */
        else if(off == 0x24) v = 0x0C00; /* data ~3.9V */
    } else if(base == 0x400C8000u) { /* CMU #2 */
        v = (off == 0x2Cu) ? ((1u << 3) | (1u << 7) | (1u << 9)) : 0u; /* clock-ready 0x288 */
    } else if(base == 0x40080000u) { /* CMU #1 */
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
    }
    (void)size;
}

/* ------------------------------------------------------------ minimal NVIC -- */

static uint32_t cur_vtor(EmuState* app) {
    return thumb_read_mem(app->cpu, 0xE000ED08u, 4);
}

static uint32_t irq_handler_addr(EmuState* app, int irqn) {
    uint32_t vt = cur_vtor(app);
    uint32_t off = vt + 0x40u + (uint32_t)irqn * 4u;
    return thumb_read_mem(app->cpu, off, 4);
}

static const int TIMER_IRQS[] = {12, 13, 14, 1};

static void nvic_tick(EmuState* app) {
    if(app->in_irq) return;
    uint32_t vtor = cur_vtor(app);
    if(vtor < 0x8000u) return; /* table not installed yet */
    int irqn = TIMER_IRQS[app->irq_rr % (int)(sizeof(TIMER_IRQS) / sizeof(int))];
    app->irq_rr++;
    uint32_t handler = irq_handler_addr(app, irqn);
    if(handler == 0 || handler == 0x82DBu) return; /* default handler: skip */
    app->tick_count++;
    app->in_irq = 1;
    thumb_enter_exception(app->cpu, handler, THUMB_EXC_RETURN_MSP_THREAD,
                          (uint32_t)(irqn + 16));
    app->next_pc = thumb_get_reg(app->cpu, 15);
}

/* Scan the RAM-resident flash for a WFI (0xBF30) to detect sleep. */
static uint32_t find_wfi(EmuState* app) {
    uint32_t size = app->flash_used;
    uint32_t lim = size < 0x20000u ? size : 0x20000u;
    for(uint32_t off = 0x8100u; off + 1 < lim; off += 2) {
        uint16_t hw = (uint16_t)(app->flash[off] | (app->flash[off + 1] << 8));
        if(hw == 0xBF30u) return off;
    }
    return 0xFFFFFFFFu;
}

/* ------------------------------------------------------------ knock FSM ----- */

static void knock_step(EmuState* app, uint32_t a) {
    if(!app->knock_active) return;
    if(app->knock_sub == 0 && a == app->prof->scan_head) {
        if(app->knock_idx < app->knock_len) {
            app->knock_cur = app->knock_seq[app->knock_idx];
            emu_set_button(app, app->knock_cur, true);
            app->knock_sub = 1;
            app->knock_spins = 0;
        } else {
            app->knock_active = 0;
        }
    } else if(app->knock_sub == 1) {
        for(int i = 0; i < app->prof->spin_count; i++) {
            if(a == app->prof->spin_heads[i]) {
                app->knock_spins++;
                if(app->knock_spins >= KNOCK_RELEASE_AFTER) {
                    emu_set_button(app, app->knock_cur, false);
                    app->knock_idx++;
                    app->knock_sub = 0;
                }
                break;
            }
        }
    }
}

/* ------------------------------------------------------------ nav FSM ------- */

static void nav_step(EmuState* app, uint32_t a) {
    if(!app->nav_active) return;
    if(a != app->prof->nav_reader) return;
    uint32_t r0 = thumb_get_reg(app->cpu, 0); /* port */
    uint32_t r1 = thumb_get_reg(app->cpu, 1); /* pin */
    bool is_our_pin = (r0 == app->nav_port && r1 == app->nav_pin);
    if(app->nav_phase == 0) { /* press */
        app->gpio_din[app->nav_port] &= (uint16_t)~(1u << app->nav_pin);
        app->btn_pressed[app->nav_btn] = 1;
        app->nav_phase = 1;
        app->nav_reads = 0;
    } else if(app->nav_phase == 1) { /* hold */
        if(is_our_pin) {
            app->nav_reads++;
            if(app->nav_reads >= app->nav_reads_target) {
                app->gpio_din[app->nav_port] |= (uint16_t)(1u << app->nav_pin);
                app->btn_pressed[app->nav_btn] = 0;
                app->nav_phase = 2;
            }
        }
    }
}

/* ------------------------------------------------------------ code hook ----- */

static void code_hook(void* ctx, uint32_t pc) {
    EmuState* app = (EmuState*)ctx;
    app->ninsn++;
    app->pc = pc;
    if(!app->unlocked && pc == app->prof->unlock_pc) {
        app->unlocked = 1;
    }
    if(app->knock_active) knock_step(app, pc);
    if(app->nav_active) nav_step(app, pc);
}

/* ------------------------------------------------------------ execution ----- */

void emu_run_burst(EmuState* app, uint32_t count) {
    bool app_running = cur_vtor(app) >= 0x8000u;
    bool stuck_wfi =
        app_running && (app->next_pc & ~1u) == (app->wfi_pc_hint & ~1u);
    bool want_tick =
        app_running && (stuck_wfi || (app->ninsn - app->last_tick_insn) > 20000ull);
    if(want_tick && !app->in_irq) {
        app->last_tick_insn = app->ninsn;
        nvic_tick(app);
    }
    thumb_set_reg(app->cpu, 15, app->next_pc);
    for(uint32_t i = 0; i < count; i++) {
        int rc = thumb_step(app->cpu);
        if(rc < 0) break;
        uint32_t pc = thumb_get_reg(app->cpu, 15);
        if((pc & 0xFFFFFFF0u) == 0xFFFFFFF0u) { /* EXC_RETURN */
            thumb_exc_return(app->cpu, pc);
            app->in_irq = 0;
            app->next_pc = thumb_get_reg(app->cpu, 15);
            return;
        }
    }
    app->next_pc = thumb_get_reg(app->cpu, 15);
}

static uint32_t fb_hash(EmuState* app) {
    uint32_t h = 2166136261u;
    for(uint32_t i = 0; i < (uint32_t)OLED_FB_SIZE; i++) {
        h ^= app->oled_fb[i];
        h *= 16777619u;
    }
    return h;
}

bool emu_nav_button(EmuState* app, int btn, bool hold) {
    if(btn < 0 || btn >= BTN_COUNT) return false;
    emu_logf("nav_button: btn=%d hold=%d unlocked=%d", btn, hold, app->unlocked);
    uint8_t port, pin;
    btn_port_pin(app->prof, btn, &port, &pin);
    app->nav_btn = (uint8_t)btn;
    app->nav_port = port;
    app->nav_pin = pin;
    app->nav_phase = 0;
    app->nav_reads = 0;
    app->nav_reads_target = hold ? NAV_LONG_READS : NAV_HOLD_READS;
    app->nav_active = 1;

    uint64_t done = 0;
    while(app->nav_phase != 2 && done < NAV_BUDGET_INSN) {
        emu_run_burst(app, 50000);
        done += 50000;
    }
    app->nav_active = 0;
    app->gpio_din[port] |= (uint16_t)(1u << pin); /* guarantee release */
    app->btn_pressed[btn] = 0;

    bool completed = (app->nav_phase == 2);

    uint32_t prev = fb_hash(app);
    done = 0;
    bool redrew = false;
    while(done < NAV_SETTLE_INSN) {
        emu_run_burst(app, NAV_SETTLE_BURST);
        done += NAV_SETTLE_BURST;
        uint32_t cur = fb_hash(app);
        if(cur != prev && done >= NAV_SETTLE_MIN_INSN) { redrew = true; break; }
        prev = cur;
    }
    emu_logf("nav_button: done btn=%d completed=%d redrew=%d settle=%llu", btn,
             completed, redrew, (unsigned long long)done);
    return completed;
}

static bool auto_unlock(EmuState* app) {
    app->unlocked = 0;
    app->max_step = -1;
    /* 1) settle until the scanner is active */
    uint64_t done = 0;
    while(done < KNOCK_SETTLE_INSN) {
        emu_run_burst(app, 100000);
        done += 100000;
    }
    /* 2) inject the full sequence synchronized with the scanner */
    if(app->knock_user_len > 0) {
        app->knock_seq = app->knock_user;
        app->knock_len = app->knock_user_len;
        emu_logf("knock: using user sequence (len=%u)", app->knock_user_len);
    } else {
        app->knock_seq = app->prof->knock;
        app->knock_len = app->prof->knock_len;
        emu_logf("knock: using default of profile %s (len=%u)", app->prof->name,
                 app->prof->knock_len);
    }
    app->knock_idx = 0;
    app->knock_sub = 0;
    app->knock_cur = 0;
    app->knock_active = 1;
    uint64_t budget = KNOCK_STEP_BUDGET * (app->knock_len ? app->knock_len : 1);
    done = 0;
    while(app->knock_active && !app->unlocked && done < budget) {
        emu_run_burst(app, 40000);
        done += 40000;
        if(app->knock_idx >= app->knock_len) app->knock_active = 0;
    }
    app->knock_active = 0;
    /* 3) final gate: hold B5 for a while */
    emu_set_button(app, app->prof->gate, true);
    done = 0;
    while(done < 600000) {
        emu_run_burst(app, 50000);
        done += 50000;
    }
    emu_set_button(app, app->prof->gate, false);
    /* 4) let it run so the app redraws the UI (with ticks) */
    done = 0;
    while(done < 600000) {
        emu_run_burst(app, 100000);
        done += 100000;
    }
    if(app->unlocked) {
        emu_logf("unlock OK (knock-code accepted, PC unlock=0x%06lX)",
                 (unsigned long)app->prof->unlock_pc);
    } else {
        emu_logf("unlock FAILED (knock-code incomplete / not accepted)");
    }
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
    app->tick_count = 0;
    app->last_tick_insn = 0;
    app->unlocked = 0;
    app->max_step = -1;
    app->knock_active = 0;
    app->nav_active = 0;
    app->nav_phase = 0;
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
    auto_unlock(app);
}

/* ------------------------------------------------------------ init/deinit --- */

int emu_init(EmuState* app, const ArmProfile* prof, const uint8_t* fw_data,
             uint32_t fw_len) {
    memset(app, 0, sizeof(*app));
    app->prof = prof;

    /* RAM-resident flash: allocate flash_used bytes, fill 0xFF (open-bus),
     * then copy the embedded image on top. The image is already a flat flash
     * dump (gaps = 0xFF), so flash_used may exceed fw_len (margin). */
    app->flash_used = prof->flash_used;
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

    app->wfi_pc_hint = find_wfi(app);
    emu_logf("emu_init: profile=%s flash_used=%lu fw_len=%lu wfi=0x%06lX",
             prof->name, (unsigned long)app->flash_used, (unsigned long)fw_len,
             (unsigned long)app->wfi_pc_hint);
    return 0;
}

void emu_deinit(EmuState* app) {
    if(app->cpu) { free(app->cpu); app->cpu = NULL; }
    if(app->ram) { free(app->ram); app->ram = NULL; }
    if(app->flash) { free(app->flash); app->flash = NULL; }
}
