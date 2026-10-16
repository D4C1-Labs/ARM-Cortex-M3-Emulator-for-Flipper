/*
 * main.c - ESP32-WROOM-32 entry point for the Pandora D-605 emulator.
 *
 * GOAL: run the REAL keyfob firmware on the Thumb-2 core entirely on the ESP32
 * (everything RAM-resident, no paging), boot + auto-unlock, then drive the link
 * to a Flipper Zero over UART:
 *   - periodically ship the OLED framebuffer (PROTO_MSG_FB),
 *   - receive button state (PROTO_MSG_BTN) and inject it,
 *   - mirror the Si4432 radio state (PROTO_MSG_RF_MODE),
 *   - forward bit-banged TX edges (PROTO_MSG_RF_TX),
 *   - inject received RF edges from the Flipper (PROTO_MSG_RX_EDGE).
 *
 * FIRMWARE SOURCE (new): the firmware is NO LONGER embedded in the ESP32. The
 * Flipper reads the firmware .hex (Intel HEX, ASCII) from its SD card and streams
 * it over this UART link. It is AGNOSTIC: ANY .hex for this MCU loads onto the
 * single hardware model -- there is NO profile selection.
 *
 *   FW_BEGIN {total_hex_bytes}            -> (re)init the HEX parser onto a flat
 *                                            buffer sized to the flash WINDOW
 *                                            (agnostic; NOT a per-firmware size)
 *   FW_CHUNK <raw .hex bytes> ...          -> parsed INCREMENTALLY, line by line,
 *                                            straight into the flat buffer
 *   FW_END   {crc32?}                      -> emu_init(flat, real_len)
 *   ESP32 -> FW_ACK {ok, flat_size}        -> load result
 *   UNLOCK  {knock sequence}               -> emu_set_knock + emu_reset_and_unlock
 *
 * UNLOCK: the knock-code is FIRMWARE CONFIG (a button combo), so the Flipper owns
 * it and sends it explicitly as a button SEQUENCE. The agnostic emulator just
 * presses it over GPIO at reset. There is no per-firmware knock table here.
 *
 * MEMORY: the ~193 KB .hex text is NEVER buffered whole. The parser keeps at most
 * one partial line (<600 B) between chunks and writes decoded bytes directly into
 * the flat flash image. The emulator then derives its real flash size from the
 * parsed image length (agnostic -- no flash_used constants). After the flat image
 * is handed to emu_init (which keeps its own copy), the temporary flat buffer is
 * freed, so steady-state uses only: emu flash (image-sized) + emu RAM (32 KB) +
 * core + the small line buffer.
 *
 * FALLBACK: build with -DPANDORA_EMBED_FW to also embed firmware.bin and boot it
 * immediately (the old behaviour), still accepting an over-the-wire reload.
 *
 * DEBUG: define PANDORA_DEBUG_ASCII to dump the framebuffer as ASCII art to the
 * ESP-IDF log, so the boot/render can be validated without a Flipper frontend.
 */
#include "emulator.h"
#include "ihex.h"
#include "protocol.h"
#include "uart_proto.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include <stdlib.h>
#include <string.h>

static const char* TAG = "pandora";

#ifdef PANDORA_EMBED_FW
/* Optional embedded firmware image (flat .flash.bin). See main/CMakeLists.txt. */
extern const uint8_t fw_start[] asm("_binary_firmware_bin_start");
extern const uint8_t fw_end[] asm("_binary_firmware_bin_end");
/* The single agnostic hardware profile runs any embedded image. */
#define PANDORA_PROFILE (&PROFILE_PANDORA_HW)
/* Default embedded knock-code (FIRMWARE config) pressed over GPIO after boot.
 * Override by sending PROTO_MSG_UNLOCK from the Flipper. Matches the classic
 * Pandora combo: UP,OK,OK,UP,UP,DOWN then hold B5 as the gate. */
static const uint8_t EMBED_KNOCK_SEQ[] = {B_UP, B_OK, B_OK, B_UP, B_UP, B_DOWN};
#define EMBED_KNOCK_GATE B5
#endif

/* How many instructions to execute between UART service passes. On the ESP32 at
 * 240 MHz this runs far faster than the 70k/s Flipper, so we can afford a large
 * burst and still service UART at a high rate. */
#define EMU_INSN_PER_TICK 120000u

/* Minimum interval between full framebuffer pushes (ms), unless it changed. */
#define FB_PUSH_INTERVAL_MS 50

/* Button hold threshold (ms). The Flipper sends a LEVEL bitmask (button
 * pressed/released), not press/release events. The ESP32 measures how long the
 * bitmask kept a button set; a release after >= this long is injected as a
 * keyfob LONG-press (emu_press_button hold=true), anything shorter as a short
 * tap. This sits below the keyfob's own ~500ms-1s long-press window so a
 * deliberate Flipper hold maps to a keyfob long-press, while quick navigation
 * taps stay short. Pure hardware timing; no firmware address. */
#define BTN_HOLD_THRESHOLD_MS 350u

/* Report upload progress to the Flipper every this many raw .hex bytes. */
#define FW_PROGRESS_EVERY 16384u

/* AGNOSTIC flat-flash window for the incremental HEX parser. The EFM32 keyfob
 * flash is 256KB; any address at/above this is dropped (far EFM32 info pages at
 * 0x0FE00000 etc). This transient buffer is freed right after emu_init, which
 * derives the REAL flash size from the parsed image length -- so there are no
 * per-firmware flash_used constants anywhere. */
#define FW_FLAT_WINDOW 0x00044000u

typedef struct {
    EmuState* emu;
    uart_proto_t* up;
    uint32_t last_oled_frames;
    uint8_t last_rf_mode, last_rf_mod;
    uint32_t last_rf_freq;
    uint8_t btn_mask;
    /* Per-logical-button press timestamp (us), 0 = not currently pressed. Used
     * to translate the Flipper's LEVEL bitmask into a proper press/hold/release
     * cycle: the rising edge records when the button went down, the falling edge
     * injects ONE emu_press_button() whose hold flag is chosen from how long the
     * bitmask kept the button set (short tap vs long press). */
    int64_t btn_down_us[BTN_COUNT];
    uint32_t last_beep_push_ms;

    /* ---- firmware-over-UART load state ---- */
    bool emu_ready; /* true once a firmware is loaded (unlock happens on UNLOCK) */
    bool loading; /* true between FW_BEGIN and FW_END */
    const ArmProfile* load_prof; /* HW profile (agnostic; always the single HW) */
    uint8_t* flat; /* temporary flat image being filled by the parser */
    uint32_t flat_cap; /* flat HEX window (agnostic: fixed max flash window) */
    ihex_parser_t hex; /* incremental Intel HEX parser */
    uint32_t next_progress; /* next hex_bytes threshold to report */
    int load_err; /* first IHEX_ERR_* seen during this load, 0 = none */
} AppCtx;

/* ------------------------------------------------------- outbound helpers ---- */
static void push_fb(AppCtx* a) {
    uint8_t fb[OLED_FB_SIZE];
    emu_get_fb(a->emu, fb);
    uart_proto_send(a->up, PROTO_MSG_FB, fb, (uint16_t)sizeof(fb));
    a->last_oled_frames = a->emu->oled_frames;
}

static void push_rf_if_changed(AppCtx* a) {
    EmuState* e = a->emu;
    if(e->rf_mode == a->last_rf_mode && e->rf_mod == a->last_rf_mod &&
       e->rf_freq_hz == a->last_rf_freq)
        return;
    proto_rf_mode_t m = {
        .mode = e->rf_mode,
        .modulation = e->rf_mod,
        .freq_hz = e->rf_freq_hz,
    };
    uart_proto_send(a->up, PROTO_MSG_RF_MODE, (const uint8_t*)&m, (uint16_t)sizeof(m));
    a->last_rf_mode = e->rf_mode;
    a->last_rf_mod = e->rf_mod;
    a->last_rf_freq = e->rf_freq_hz;
}

static void push_tx_edges(AppCtx* a) {
    RfEdge e;
    /* Pack up to a batch of captured edges per pass to limit frame overhead. */
    uint8_t buf[PROTO_MAX_PAYLOAD];
    uint16_t off = 2; /* reserve uint16 count prefix */
    uint16_t count = 0;
    while(off + sizeof(proto_rf_edge_t) <= sizeof(buf) && emu_tx_pop_edge(a->emu, &e)) {
        proto_rf_edge_t pe = { .level = e.level ? 1 : 0, .duration_us = e.duration };
        memcpy(buf + off, &pe, sizeof(pe));
        off += sizeof(pe);
        count++;
    }
    if(count) {
        buf[0] = (uint8_t)(count & 0xFF);
        buf[1] = (uint8_t)(count >> 8);
        uart_proto_send(a->up, PROTO_MSG_RF_TX, buf, off);
    }
}

#ifdef PANDORA_DEBUG_ASCII
static void dump_ascii(EmuState* emu) {
    static char ascii[4096];
    size_t n = emu_fb_ascii(emu, ascii, sizeof(ascii));
    ESP_LOGI(TAG, "OLED (%u bytes):\n%.*s", (unsigned)n, (int)n, ascii);
}
#endif

/* ------------------------------------------------- firmware-load handlers ---- */
/* Release any in-flight load scratch buffer. */
static void load_cleanup(AppCtx* a) {
    if(a->flat) { free(a->flat); a->flat = NULL; }
    a->loading = false;
    a->flat_cap = 0;
    a->load_err = 0;
}

static void fw_begin(AppCtx* a, const uint8_t* payload, uint16_t len) {
    if(len < sizeof(proto_fw_begin_t)) {
        ESP_LOGW(TAG, "FW_BEGIN too short (%u)", len);
        return;
    }
    proto_fw_begin_t hb;
    memcpy(&hb, payload, sizeof(hb));

    /* If a previous load left scratch around, drop it and start fresh. */
    load_cleanup(a);

    /* Agnostic: there is a SINGLE hardware profile for every firmware of this
     * keyfob (no profile selection on the wire). The flat window is the flash
     * size, NOT a per-firmware flash_used. emu_init later trims to the real
     * image length. */
    a->load_prof = &PROFILE_PANDORA_HW;
    a->flat_cap = FW_FLAT_WINDOW;
    a->flat = (uint8_t*)malloc(a->flat_cap);
    if(!a->flat) {
        ESP_LOGE(TAG, "FW_BEGIN: OOM allocating %u-byte flat image", (unsigned)a->flat_cap);
        a->load_err = -100;
        return;
    }
    /* Erased flash = 0xFF (open-bus); the parser only overwrites decoded bytes. */
    memset(a->flat, 0xFF, a->flat_cap);
    ihex_init(&a->hex, a->flat, a->flat_cap);
    a->loading = true;
    a->next_progress = FW_PROGRESS_EVERY;
    ESP_LOGI(TAG, "FW_BEGIN hw=%s hex=%u bytes, flat_cap=0x%X",
             a->load_prof->name, (unsigned)hb.total_hex_bytes,
             (unsigned)a->flat_cap);
}

static void fw_chunk(AppCtx* a, const uint8_t* payload, uint16_t len) {
    if(!a->loading || !a->flat) return; /* ignore chunks outside a load */
    if(a->load_err) return; /* already failed; swallow the rest */
    int rc = ihex_feed(&a->hex, payload, len);
    if(rc != IHEX_OK) {
        a->load_err = rc;
        ESP_LOGE(TAG, "FW_CHUNK parse error rc=%d after %u hex bytes", rc,
                 (unsigned)a->hex.hex_bytes);
        return;
    }
    if(a->hex.hex_bytes >= a->next_progress) {
        proto_fw_progress_t pr = {
            .hex_bytes_seen = a->hex.hex_bytes,
            .flat_bytes = a->hex.highest,
        };
        uart_proto_send(a->up, PROTO_MSG_FW_PROGRESS, (const uint8_t*)&pr, sizeof(pr));
        a->next_progress += FW_PROGRESS_EVERY;
    }
}

static void fw_end(AppCtx* a, const uint8_t* payload, uint16_t len) {
    (void)payload;
    (void)len; /* optional proto_fw_end_t CRC32 not verified in stage 1 */
    proto_fw_ack_t ack = { .ok = 0, .flat_size = 0 };

    if(!a->loading || !a->flat) {
        ESP_LOGW(TAG, "FW_END with no active load");
        uart_proto_send(a->up, PROTO_MSG_FW_ACK, (const uint8_t*)&ack, sizeof(ack));
        return;
    }
    /* Flush a final non-newline-terminated line, if any. */
    if(a->load_err == 0 && a->hex.line_len > 0) {
        int rc = ihex_feed(&a->hex, (const uint8_t*)"\n", 1);
        if(rc != IHEX_OK) a->load_err = rc;
    }
    if(a->load_err != 0) {
        ESP_LOGE(TAG, "FW_END: load failed (rc=%d)", a->load_err);
        load_cleanup(a);
        uart_proto_send(a->up, PROTO_MSG_FW_ACK, (const uint8_t*)&ack, sizeof(ack));
        return;
    }
    if(!a->hex.eof) {
        ESP_LOGW(TAG, "FW_END: no EOF record seen (continuing anyway), highest=0x%X",
                 (unsigned)a->hex.highest);
    }

    ESP_LOGI(TAG, "FW_END: parsed %u hex bytes -> flat highest=0x%X (cap=0x%X)",
             (unsigned)a->hex.hex_bytes, (unsigned)a->hex.highest, (unsigned)a->flat_cap);

    /* Tear down any previously running emulator before (re)initializing. */
    if(a->emu_ready) {
        emu_deinit(a->emu);
        a->emu_ready = false;
    }

    /* emu_init keeps its own copy of the flat image, so free our scratch after.
     * Pass the REAL parsed image length (hex.highest), not the flat window, so
     * the emulator sizes its flash from the firmware itself (agnostic). */
    uint32_t image_len = a->hex.highest ? a->hex.highest : a->flat_cap;
    int ir = emu_init(a->emu, a->load_prof, a->flat, image_len);
    if(ir != 0) {
        ESP_LOGE(TAG, "emu_init failed (rc=%d, out of memory?)", ir);
        load_cleanup(a);
        uart_proto_send(a->up, PROTO_MSG_FW_ACK, (const uint8_t*)&ack, sizeof(ack));
        return;
    }
    free(a->flat);
    a->flat = NULL;
    a->loading = false;

    /* Boot the firmware WITHOUT a knock. The Flipper drives the unlock later via
     * PROTO_MSG_UNLOCK (with the user's knock sequence), decoupling firmware load
     * from unlock. Until then the firmware runs its locked screen. */
    emu_set_knock(a->emu, NULL, 0, -1);
    ESP_LOGI(TAG, "booting firmware (hw=%s), awaiting UNLOCK knock...", a->load_prof->name);
    int64_t t0 = esp_timer_get_time();
    emu_reset_and_unlock(a->emu);
    int64_t t1 = esp_timer_get_time();
    a->emu_ready = true;
    ESP_LOGI(TAG, "boot done in %lld ms, unlocked=%d, PC=0x%08X, free heap=%u",
             (t1 - t0) / 1000, a->emu->unlocked,
             (unsigned)thumb_get_reg(a->emu->cpu, 15),
             (unsigned)esp_get_free_heap_size());

#ifdef PANDORA_DEBUG_ASCII
    for(int i = 0; i < 40; i++) emu_run_burst(a->emu, 50000);
    dump_ascii(a->emu);
#endif

    ack.ok = 1;
    ack.flat_size = a->hex.highest;
    uart_proto_send(a->up, PROTO_MSG_FW_ACK, (const uint8_t*)&ack, sizeof(ack));

    /* Give a freshly-connected Flipper an initial frame + radio state. */
    push_fb(a);
    push_rf_if_changed(a);
}

/* ------------------------------------------------- auto-unlock knock handler --
 * PROTO_MSG_UNLOCK: payload = proto_unlock_t + count * proto_knock_step_t. The
 * Flipper owns the knock-code (FIRMWARE config, a button combo); here we install
 * it with emu_set_knock() and press it over GPIO via emu_reset_and_unlock(). The
 * `hold` flag on each step is accepted on the wire; the agnostic GPIO unlock
 * presses the taps + holds the final gate, matching the real keyfob combo. */
static void fw_unlock(AppCtx* a, const uint8_t* payload, uint16_t len) {
    if(!a->emu_ready) {
        ESP_LOGW(TAG, "UNLOCK ignored: no firmware loaded yet");
        return;
    }
    if(len < sizeof(proto_unlock_t)) {
        ESP_LOGW(TAG, "UNLOCK too short (%u)", len);
        return;
    }
    proto_unlock_t uh;
    memcpy(&uh, payload, sizeof(uh));
    const uint8_t* steps = payload + sizeof(uh);
    size_t avail = len - sizeof(uh);
    uint8_t count = uh.count;
    if(count > PROTO_KNOCK_MAX_STEPS) count = PROTO_KNOCK_MAX_STEPS;
    if((size_t)count * sizeof(proto_knock_step_t) > avail)
        count = (uint8_t)(avail / sizeof(proto_knock_step_t));

    /* Flatten the step list to the B_* sequence the core expects. The emulator
     * core presses each as a tap; the gate is held during validation. */
    uint8_t seq[PROTO_KNOCK_MAX_STEPS];
    for(uint8_t i = 0; i < count; i++) {
        proto_knock_step_t st;
        memcpy(&st, steps + (size_t)i * sizeof(st), sizeof(st));
        seq[i] = st.button; /* PROTO_BTN_ID_* == B_* index */
    }
    int gate = (uh.gate == PROTO_BTN_ID_NONE) ? -1 : (int)uh.gate;

    emu_set_knock(a->emu, count ? seq : NULL, count, gate);
    ESP_LOGI(TAG, "UNLOCK: knock len=%u gate=%d -> GPIO auto-unlock...", count, gate);
    int64_t t0 = esp_timer_get_time();
    emu_reset_and_unlock(a->emu);
    int64_t t1 = esp_timer_get_time();
    ESP_LOGI(TAG, "unlock done in %lld ms, unlocked=%d, PC=0x%08X",
             (t1 - t0) / 1000, a->emu->unlocked,
             (unsigned)thumb_get_reg(a->emu->cpu, 15));

    push_fb(a);
    push_rf_if_changed(a);
}

/* Let the firmware's foreground loop consume an injected RF edge before the next
 * edge in the same UART batch. AGNOSTIC: emu_rx_inject_edge already presents the
 * edge on the real RX-data pin (PB0) + TIMER1 capture and runs a few foreground
 * bursts itself; here we just give a little extra foreground time. There is NO
 * profile-RAM poke and NO decoder-address sync any more (the old code watched a
 * per-firmware rx_pending RAM offset, which was not agnostic). */
static void rx_service_pending(AppCtx* a) {
    if(!a->emu) return;
    for(uint32_t i = 0; i < 8u; i++) emu_run_burst(a->emu, 30000u);
}

/* ------------------------------------------------------- inbound dispatch ---- */
static void on_frame(uint8_t type, const uint8_t* payload, uint16_t len, void* ctx) {
    AppCtx* a = (AppCtx*)ctx;

    /* Firmware-load and frontend teardown messages are always accepted. */
    switch(type) {
    case PROTO_MSG_STOP:
        if(a->emu_ready) {
            for(int b = 0; b < BTN_COUNT; b++) {
                emu_set_button(a->emu, b, false);
                a->btn_down_us[b] = 0;
            }
            a->btn_mask = 0;
            a->emu->rf_mode = PROTO_RF_IDLE;
            a->emu->tx_head = a->emu->tx_tail = 0;
            emu_deinit(a->emu);
            memset(a->emu, 0, sizeof(*a->emu));
            a->emu_ready = false;
            ESP_LOGI(TAG, "frontend disconnected; emulator stopped and released");
        }
        return;
    case PROTO_MSG_FW_BEGIN:
        fw_begin(a, payload, len);
        return;
    case PROTO_MSG_FW_CHUNK:
        fw_chunk(a, payload, len);
        return;
    case PROTO_MSG_FW_END:
        fw_end(a, payload, len);
        return;
    case PROTO_MSG_PING: {
        const char* banner = a->emu_ready ? "PANDORA-ESP32 READY" : "PANDORA-ESP32 NO-FW";
        ESP_LOGI(TAG, "PING received; replying %s", banner);
        uart_proto_send(a->up, PROTO_MSG_PONG, (const uint8_t*)banner, (uint16_t)strlen(banner));
        return;
    }
    default:
        break;
    }

    /* The rest only make sense once a firmware is loaded and running. */
    if(!a->emu_ready) {
        ESP_LOGW(TAG, "inbound type 0x%02X ignored: no firmware loaded yet", type);
        return;
    }

    switch(type) {
    case PROTO_MSG_BTN: {
        if(len < 1) break;
        uint8_t mask = payload[0];
        uint8_t prev = a->btn_mask;
        uint8_t rising = mask & (uint8_t)~prev;  /* 0->1: button went down */
        uint8_t falling = prev & (uint8_t)~mask; /* 1->0: button released */
        a->btn_mask = mask;

        /* The Flipper sends a LEVEL bitmask (one bit per logical keyfob button,
         * set while held). Translate that into a proper keyfob key CYCLE:
         *
         *   rising edge  -> remember WHEN the button went down (btn_down_us).
         *   falling edge -> inject ONE emu_press_button(btn, hold) over GPIO,
         *                   with hold chosen from how long the bitmask kept the
         *                   button set (>= BTN_HOLD_THRESHOLD_MS => long press).
         *
         * Injecting on the RELEASE edge (not the press edge) matches real
         * hardware -- a keyfob commits a click on release -- and the oracle
         * (pandora_tui press_button: pin low for the hold duration, then high,
         * firmware dispatches on the edge). emu_press_button() runs the full
         * press->bursts->release->settle cycle with the correct timing/tick
         * cadence, so the firmware's polling scanner actually sees the key.
         *
         * We also enforce a FALLBACK release: if a button bit is set but we have
         * no recorded press time (e.g. we missed its rising edge, or a mask was
         * coalesced), a later clearing still injects a short tap rather than
         * being dropped. */
        static const struct {
            uint8_t bit;
            int button;
        } button_map[] = {
            {PROTO_BTN_UP, B_UP},
            {PROTO_BTN_DOWN, B_DOWN},
            {PROTO_BTN_OK, B_OK},
            {PROTO_BTN_BACK, B_BACK},
            {PROTO_BTN_B5, B5},
            {PROTO_BTN_B6, B6},
        };
        int64_t now_us = esp_timer_get_time();
        for(size_t i = 0; i < sizeof(button_map) / sizeof(button_map[0]); i++) {
            int btn = button_map[i].button;
            if(rising & button_map[i].bit) {
                /* button pressed: start the hold timer. No injection yet. */
                a->btn_down_us[btn] = now_us ? now_us : 1;
            }
            if(falling & button_map[i].bit) {
                /* button released: inject the full press cycle now. */
                int64_t t0 = a->btn_down_us[btn];
                uint32_t held_ms =
                    (t0 > 0 && now_us > t0) ? (uint32_t)((now_us - t0) / 1000) : 0u;
                a->btn_down_us[btn] = 0;
                bool hold = held_ms >= BTN_HOLD_THRESHOLD_MS;
                bool handled = emu_press_button(a->emu, btn, hold);
                ESP_LOGI(TAG,
                         "BTN cycle bit=0x%02X button=%d held=%ums hold=%d "
                         "handled=%d oled=%lu",
                         button_map[i].bit, btn, (unsigned)held_ms, hold, handled,
                         (unsigned long)a->emu->oled_frames);
            }
        }
        break;
    }
    case PROTO_MSG_RX_EDGE: {
        if(len < sizeof(proto_rf_edge_t)) break;
        proto_rf_edge_t e;
        memcpy(&e, payload, sizeof(e));
        emu_rx_inject_edge(a->emu, e.level != 0, e.duration_us);
        rx_service_pending(a);
        break;
    }
    case PROTO_MSG_RX_EDGES: {
        if(len < sizeof(uint16_t)) break;
        uint16_t count = (uint16_t)(payload[0] | ((uint16_t)payload[1] << 8));
        const uint8_t* edges = payload + sizeof(uint16_t);
        size_t available = len - sizeof(uint16_t);
        if(count > available / sizeof(proto_rf_edge_t))
            count = available / sizeof(proto_rf_edge_t);
        for(uint16_t i = 0; i < count; i++) {
            proto_rf_edge_t edge;
            memcpy(&edge, edges + (size_t)i * sizeof(edge), sizeof(edge));
            emu_rx_inject_edge(a->emu, edge.level != 0, edge.duration_us);
            rx_service_pending(a);
        }
        break;
    }
    case PROTO_MSG_REQ_FB: {
        uint8_t fb[OLED_FB_SIZE];
        emu_get_fb(a->emu, fb);
        uart_proto_send(a->up, PROTO_MSG_FB, fb, (uint16_t)sizeof(fb));
        break;
    }
    case PROTO_MSG_UNLOCK:
        fw_unlock(a, payload, len);
        break;
    case PROTO_MSG_RESET:
        /* Re-run the last installed knock (same as the previous UNLOCK). */
        emu_reset_and_unlock(a->emu);
        break;
    default:
        ESP_LOGW(TAG, "unknown inbound type 0x%02X (%u bytes)", type, len);
        break;
    }
}

/* --------------------------------------------------------------- app_main ---- */
void app_main(void) {
    ESP_LOGI(TAG, "Pandora D-605 emulator on ESP32");
    ESP_LOGI(TAG, "free heap: %u bytes", (unsigned)esp_get_free_heap_size());

    static EmuState emu;
    memset(&emu, 0, sizeof(emu));

    static uart_proto_t up;
    memset(&up, 0, sizeof(up));
    static AppCtx app;
    memset(&app, 0, sizeof(app));
    app.emu = &emu;
    app.up = &up;
    up.cb = on_frame;
    up.cb_ctx = &app;

    /* Init the UART FIRST, then wait for the Flipper to stream the firmware. */
    if(uart_proto_init(&up) != 0) {
        ESP_LOGE(TAG, "uart_proto_init failed");
        return;
    }

#ifdef PANDORA_EMBED_FW
    /* Optional: boot an embedded image immediately (old behaviour). An
     * over-the-wire FW_BEGIN/END can still reload a different firmware later. */
    {
        uint32_t fw_len = (uint32_t)(fw_end - fw_start);
        ESP_LOGI(TAG, "embedded firmware: %u bytes", (unsigned)fw_len);
        const ArmProfile* prof = PANDORA_PROFILE;
        if(emu_init(&emu, prof, fw_start, fw_len) == 0) {
            /* Embedded default knock (firmware config). Override via the wire
             * with PROTO_MSG_UNLOCK (the user's knock sequence). */
            emu_set_knock(&emu, EMBED_KNOCK_SEQ, (uint8_t)sizeof(EMBED_KNOCK_SEQ),
                          EMBED_KNOCK_GATE);
            emu_reset_and_unlock(&emu);
            app.emu_ready = true;
            ESP_LOGI(TAG, "embedded boot: hw=%s unlocked=%d free heap=%u",
                     prof->name, emu.unlocked, (unsigned)esp_get_free_heap_size());
#ifdef PANDORA_DEBUG_ASCII
            for(int i = 0; i < 40; i++) emu_run_burst(&emu, 50000);
            dump_ascii(&emu);
#endif
            push_fb(&app);
            push_rf_if_changed(&app);
        } else {
            ESP_LOGE(TAG, "embedded emu_init failed; waiting for UART upload");
        }
    }
#else
    ESP_LOGI(TAG, "waiting for firmware over UART (FW_BEGIN/CHUNK/END)...");
#endif

    int64_t last_fb = esp_timer_get_time();
    for(;;) {
        /* Always service inbound frames (firmware upload + control + input). */
        uart_proto_poll(&up);

        if(app.emu_ready) {
            /* 1) advance the emulated MCU. */
            emu_run_burst(&emu, EMU_INSN_PER_TICK);
            uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
            if(emu.buzzer_event && (uint32_t)(now_ms - app.last_beep_push_ms) >= 20u) {
                emu.buzzer_event = false;
                app.last_beep_push_ms = now_ms;
                uart_proto_send(&up, PROTO_MSG_BEEP, NULL, 0);
            }

            /* 2) push outbound state. */
            push_tx_edges(&app);
            push_rf_if_changed(&app);

            int64_t now = esp_timer_get_time();
            bool changed = (emu.oled_frames != app.last_oled_frames);
            if(changed || (now - last_fb) >= FB_PUSH_INTERVAL_MS * 1000) {
                push_fb(&app);
                last_fb = now;
            }
        }

        /* Yield briefly so the IDLE task / watchdog are happy. */
        vTaskDelay(1);
    }
}
