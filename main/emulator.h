/*
 * emulator.h - EFM32 Cortex-M3 hardware model for the Pandora D-605 keyfob,
 *              ported from the Flipper app (PandoraARM/main.c) to the ESP32.
 *
 * DIFFERENCES vs the Flipper app (intentional, for the ESP32):
 *   - FLASH IS RAM-RESIDENT. The whole firmware (85KB for pmax) is malloc'd into
 *     one contiguous block and passed to thumb_set_regions as a DIRECT flash
 *     region. The ESP32 has ~320KB heap, so the Flipper's paged 4KB LRU flash
 *     cache (FlashCache) is NOT needed and is removed. This simplifies boot and
 *     makes flash fetch as fast as RAM fetch.
 *   - NO Flipper GUI / Canvas / view_dispatcher. The OLED is captured into a
 *     double-buffered framebuffer and shipped over UART to the Flipper.
 *   - NO CC1101 / furi_hal_subghz / speaker / LED HAL. Those are the Flipper
 *     frontend's job now. The emulator exposes the Si4432 mode + a TX edge ring
 *     and an RX inject hook; the UART layer bridges them to the real radio on
 *     the Flipper.
 *
 * Faithfully ported (same logic, adapted struct): the HW profiles, the Si4432
 * model, mmio_read/mmio_write, spi_byte, oled_cmd/oled_data/oled_commit,
 * nvic_tick, run_burst, knock_step/nav_step/code_hook, auto_unlock, nav_button,
 * emu_reset_and_unlock.
 */
#ifndef PANDORA_EMULATOR_H
#define PANDORA_EMULATOR_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "thumb_core.h"
#include "protocol.h"

/* ------------------------------------------------------------ memory map ---- */
/* Emulated EFM32 (NOT the ESP32). */
#define PA_FLASH_BASE 0x00000000u
#define PA_RAM_BASE 0x20000000u
#define PA_RAM_SIZE 0x00008000u /* 32 KB */

/* OLED geometry (keyfob UC16xx/ST7528): 92 columns x 16 pages, 2bpp. */
#define OLED_COLS 92
#define OLED_VISIBLE_COLS 88 /* physical glass width (cols 88..91 off-glass) */
#define OLED_PAGES 16
#define OLED_FB_SIZE (OLED_COLS * OLED_PAGES) /* 1472 */

#define SPI_RX_QUEUE_LEN 64

/* TX edge ring: edges captured from the firmware bit-banging PB3 (OOK/FSK).
 * Shipped to the Flipper over UART for the real CC1101 to replay. */
#define TX_RING_LEN 512u

/* ------------------------------------------------------------ HW profile ---- */
/* logical button indices (order matters: mirrored by PROTO_BTN_* bits). */
enum { B_UP = 0, B_DOWN, B_OK, B_BACK, B5, B6, BTN_COUNT };

typedef struct {
    const char* name;
    uint32_t flash_used; /* real code size in flash */
    uint32_t fb_addr; /* framebuffer address in RAM (diagnostic only) */
    /* buttons (port, pin) */
    uint8_t up_port, up_pin;
    uint8_t down_port, down_pin;
    uint8_t ok_port, ok_pin;
    uint8_t back_port, back_pin;
    uint8_t b5_port, b5_pin;
    uint8_t b6_port, b6_pin;
    /* knock-code: up to 8 steps, terminated with 0xFF */
    uint8_t knock[8];
    uint8_t knock_len;
    uint8_t gate; /* button of the 2nd gate (B_* index) */
    uint32_t scan_head; /* head of the knock scanner */
    uint32_t spin_heads[8];
    uint8_t spin_count;
    uint32_t unlock_pc; /* bl unlock routine = completed */
    uint32_t nav_reader; /* read_gpio_pin inlined used POST-unlock */
    /* RF bridge (Si4432 direct mode) */
    uint32_t demod_isr;
    uint32_t rx_gate, rx_carrier_gate, rx_pending, rx_state;
    uint32_t rx_width_cur, rx_width_prev, rx_width_prev2;
    uint32_t rx_struct;
    uint8_t rx_filter_taps;
    uint8_t data_port, data_pin; /* RX-data (PB0) */
    uint8_t tx_data_port, tx_data_pin; /* TX-data (PB3) */
    uint8_t rf_cs_port, rf_cs_pin; /* Si4432 CS (PE9) */
    uint16_t rf_tx_cell_us;
} ArmProfile;

extern const ArmProfile PROFILE_PANDORA;
extern const ArmProfile PROFILE_PANDORAMAX;

/* ------------------------------------------------------------ Si4432 -------- */
typedef struct {
    uint8_t regs[0x80];
    uint8_t mode; /* 0 idle, 1 ready, 2 rx, 3 tx (PROTO_RF_*) */
} Si4432;

/* ------------------------------------------------------------ TX edge ------- */
typedef struct {
    bool level;
    uint32_t duration; /* us (approx) */
} RfEdge;

/* ------------------------------------------------------------ emulator ------ */
typedef struct EmuState {
    ThumbCore* cpu;
    const ArmProfile* prof;

    uint8_t* flash; /* firmware RAM-resident (flash_used bytes) */
    uint32_t flash_used;
    uint8_t* ram; /* 32KB EFM32 RAM */

    Si4432 si;
    /* SPI RX queue (Si4432 responses) */
    uint8_t spi_rx[SPI_RX_QUEUE_LEN];
    uint8_t spi_rx_head, spi_rx_tail;
    int spi_phase; /* -1 idle, 0 pending-after-addr */
    uint8_t spi_kind; /* 'w'/'r' */
    uint8_t spi_reg;

    /* GPIO */
    uint16_t gpio_din[16]; /* bit=1 released (pull-up) */
    uint16_t gpio_dout[16];

    /* timers (faithful free-running model) */
    uint64_t timer_t0[4];
    uint32_t timer_top[4];
    uint32_t timer_rate[4];
    uint32_t rx_pulse_ticks;
    bool rx_feeding, rx_primed;
    bool buzzer_event;

    /* minimal NVIC */
    uint8_t in_irq;
    uint8_t irq_rr;
    uint32_t tick_count;
    uint64_t last_tick_insn;
    uint32_t wfi_pc_hint;

    /* knock FSM (PC-synchronized button injection) */
    const uint8_t* knock_seq;
    uint8_t knock_len;
    uint8_t knock_idx;
    uint8_t knock_sub; /* 0 press, 1 waitrelease */
    uint8_t knock_cur;
    uint8_t knock_spins;
    uint8_t knock_active;
    uint8_t unlocked;
    int max_step;

    /* nav FSM (POST-unlock button injection) */
    uint8_t nav_active;
    uint8_t nav_phase; /* 0 press, 1 hold, 2 done */
    uint8_t nav_port;
    uint8_t nav_pin;
    uint8_t nav_btn;
    uint32_t nav_reads;
    uint32_t nav_reads_target;

    /* buttons (logical mask) */
    uint8_t btn_pressed[BTN_COUNT];

    /* OLED double buffering */
    uint8_t oled_fb[OLED_FB_SIZE]; /* VISIBLE buffer (shipped over UART) */
    uint8_t oled_work[OLED_FB_SIZE]; /* WORK buffer (written byte-by-byte) */
    uint8_t oled_page;
    uint8_t oled_col;
    uint8_t oled_work_dirty;
    uint32_t oled_frames; /* commit counter (diagnostic) */

    /* RF bridge (Si4432 -> UART) */
    uint8_t rf_mode; /* PROTO_RF_* */
    uint8_t rf_mod; /* PROTO_MOD_* */
    uint32_t rf_freq_hz;

    /* TX capture ring (firmware bit-bangs PB3 -> edges) */
    RfEdge tx_ring[TX_RING_LEN];
    uint32_t tx_head, tx_tail;
    bool tx_capturing;
    bool tx_last_level;
    uint64_t tx_last_insn;

    uint64_t ninsn;
    uint32_t pc;
    uint32_t next_pc;

    /* user-defined knock-code (optional) */
    uint8_t knock_user[16];
    uint8_t knock_user_len;
} EmuState;

/* ------------------------------------------------------------ API ----------- */

/* Allocate+init the core, load the firmware image (flat, flash_used bytes) into
 * RAM-resident flash, allocate the 32KB RAM, wire the core regions/callbacks.
 * fw_data/fw_len = the embedded flat firmware image. Returns 0 on success,
 * <0 on allocation failure. */
int emu_init(EmuState* emu, const ArmProfile* prof, const uint8_t* fw_data, uint32_t fw_len);

/* Free the heap owned by the emulator (core, flash, ram). */
void emu_deinit(EmuState* emu);

/* Reset the core and run the full boot + auto-unlock (blocking: runs millions of
 * instructions). After this returns, emu->unlocked tells if the knock worked. */
void emu_reset_and_unlock(EmuState* emu);

/* Run one burst of instructions with WFI/tick + EXC_RETURN handling. */
void emu_run_burst(EmuState* emu, uint32_t count);

/* Inject ONE navigation button POST-unlock (press+hold+release, synchronized
 * with the firmware's nav_reader). hold=true for a long press. Blocking. */
bool emu_nav_button(EmuState* emu, int btn, bool hold);

/* Set a button level directly (0=pressed). Used for continuous/raw input when a
 * full nav_button cycle is not wanted. */
void emu_set_button(EmuState* emu, int btn, bool pressed);

/* Inject one RX edge into the firmware demodulator (PB0). Stage 1 stub that
 * queues the level; the real demod feed is a Stage 2 item. */
void emu_rx_inject_edge(EmuState* emu, bool level, uint32_t duration_us);

/* Pop one captured TX edge (firmware bit-bang on PB3). Returns false if empty. */
bool emu_tx_pop_edge(EmuState* emu, RfEdge* out);

/* Copy the current VISIBLE OLED framebuffer (OLED_FB_SIZE bytes) into dst. */
void emu_get_fb(EmuState* emu, uint8_t* dst);

/* Render the OLED framebuffer as ASCII art into a caller buffer (debug). Writes
 * up to cap bytes including the trailing NUL. Returns the length written. */
size_t emu_fb_ascii(EmuState* emu, char* out, size_t cap);

#endif /* PANDORA_EMULATOR_H */
