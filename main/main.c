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
 * it over this UART link:
 *
 *   FW_BEGIN {total_hex_bytes, profile}   -> (re)init the HEX parser onto a flat
 *                                            buffer sized flash_used for profile
 *   FW_CHUNK <raw .hex bytes> ...          -> parsed INCREMENTALLY, line by line,
 *                                            straight into the flat buffer
 *   FW_END   {crc32?}                      -> emu_init(flat) + emu_reset_and_unlock
 *   ESP32 -> FW_ACK {ok, flat_size}        -> load result
 *
 * MEMORY: the ~193 KB .hex text is NEVER buffered whole. The parser keeps at most
 * one partial line (<600 B) between chunks and writes decoded bytes directly into
 * the flat flash image (flash_used = 0x15400 pmax / 0x13800 pandora). After the
 * flat image is handed to emu_init (which keeps its own copy), the temporary flat
 * buffer is freed, so steady-state uses only: emu flash (~85 KB) + emu RAM (32 KB)
 * + core + the small line buffer.
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
/* Default embedded profile; override at build time. */
#ifndef PANDORA_PROFILE
#define PANDORA_PROFILE (&PROFILE_PANDORAMAX)
#endif
#endif

/* How many instructions to execute between UART service passes. On the ESP32 at
 * 240 MHz this runs far faster than the 70k/s Flipper, so we can afford a large
 * burst and still service UART at a high rate. */
#define EMU_INSN_PER_TICK 120000u

/* Minimum interval between full framebuffer pushes (ms), unless it changed. */
#define FB_PUSH_INTERVAL_MS 50

/* Report upload progress to the Flipper every this many raw .hex bytes. */
#define FW_PROGRESS_EVERY 16384u

typedef struct {
    EmuState* emu;
    uart_proto_t* up;
    uint32_t last_oled_frames;
    uint8_t last_rf_mode, last_rf_mod;
    uint32_t last_rf_freq;
    uint8_t btn_mask;
    uint32_t last_beep_push_ms;

    /* ---- firmware-over-UART load state ---- */
    bool emu_ready; /* true once a firmware is loaded and unlocked */
    bool loading; /* true between FW_BEGIN and FW_END */
    const ArmProfile* load_prof; /* profile selected by FW_BEGIN */
    uint8_t* flat; /* temporary flat image being filled by the parser */
    uint32_t flat_cap; /* = load_prof->flash_used */
    ihex_parser_t hex; /* incremental Intel HEX parser */
    uint32_t next_progress; /* next hex_bytes threshold to report */
    int load_err; /* first IHEX_ERR_* seen during this load, 0 = none */
} AppCtx;

/* Map a wire profile id to the emulator's profile table. */
static const ArmProfile* profile_from_id(uint8_t id) {
    switch(id) {
    case PROTO_FW_PROFILE_PANDORA:
        return &PROFILE_PANDORA;
    case PROTO_FW_PROFILE_PANDORAMAX:
    default:
        return &PROFILE_PANDORAMAX;
    }
}

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

    a->load_prof = profile_from_id(hb.profile);
    a->flat_cap = a->load_prof->flash_used;
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
    ESP_LOGI(TAG, "FW_BEGIN profile=%s (id=%u) hex=%u bytes, flat_cap=0x%X",
             a->load_prof->name, hb.profile, (unsigned)hb.total_hex_bytes,
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

    /* emu_init keeps its own copy of the flat image, so free our scratch after. */
    int ir = emu_init(a->emu, a->load_prof, a->flat, a->flat_cap);
    if(ir != 0) {
        ESP_LOGE(TAG, "emu_init failed (rc=%d, out of memory?)", ir);
        load_cleanup(a);
        uart_proto_send(a->up, PROTO_MSG_FW_ACK, (const uint8_t*)&ack, sizeof(ack));
        return;
    }
    free(a->flat);
    a->flat = NULL;
    a->loading = false;

    ESP_LOGI(TAG, "booting firmware + auto-unlock (profile=%s)...", a->load_prof->name);
    int64_t t0 = esp_timer_get_time();
    emu_reset_and_unlock(a->emu);
    int64_t t1 = esp_timer_get_time();
    a->emu_ready = true;
    ESP_LOGI(TAG, "unlock done in %lld ms, unlocked=%d, PC=0x%08X, free heap=%u",
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

/* Let Pandora's foreground loop consume a completed decoder frame before
 * subsequent edges in the same UART batch are injected. This mirrors the
 * internal frontend's one-frame-at-a-time replay discipline. */
static void rx_service_pending(AppCtx* a) {
    uint32_t off = a->emu && a->emu->prof ? a->emu->prof->rx_pending : 0;
    if(!off || off < PA_RAM_BASE || off >= PA_RAM_BASE + PA_RAM_SIZE) return;
    uint8_t* pending = &a->emu->ram[off - PA_RAM_BASE];
    if(*pending != 0xFFu) return;
    for(uint32_t i = 0; i < 40u && *pending == 0xFFu; i++)
        emu_run_burst(a->emu, 30000u);
    if(*pending == 0xFFu)
        ESP_LOGW(TAG, "RX decoder frame pending after 1.2M foreground instructions");
    else
        ESP_LOGI(TAG, "RX decoder frame consumed by firmware foreground loop");
}

/* ------------------------------------------------------- inbound dispatch ---- */
static void on_frame(uint8_t type, const uint8_t* payload, uint16_t len, void* ctx) {
    AppCtx* a = (AppCtx*)ctx;

    /* Firmware-load and frontend teardown messages are always accepted. */
    switch(type) {
    case PROTO_MSG_STOP:
        if(a->emu_ready) {
            for(int b = 0; b < BTN_COUNT; b++) emu_set_button(a->emu, b, false);
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
        uint8_t pressed = mask & (uint8_t)~a->btn_mask;
        a->btn_mask = mask;

        /* Match the internal Flipper emulator: a physical press must pass
         * through the firmware's post-unlock GPIO scan loop. Keeping a raw
         * level set in the UART callback can miss the short press between
         * emulator bursts, so synthesize one debounced key cycle per rising
         * edge. emu_nav_button() releases the GPIO after the firmware reads it. */
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
        for(size_t i = 0; i < sizeof(button_map) / sizeof(button_map[0]); i++) {
            if(pressed & button_map[i].bit) {
                bool handled = emu_nav_button(a->emu, button_map[i].button, false);
                ESP_LOGI(TAG, "BTN cycle bit=0x%02X button=%d handled=%d oled=%lu",
                         button_map[i].bit, button_map[i].button, handled,
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
    case PROTO_MSG_RESET:
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
            emu_reset_and_unlock(&emu);
            app.emu_ready = true;
            ESP_LOGI(TAG, "embedded boot: profile=%s unlocked=%d free heap=%u",
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
