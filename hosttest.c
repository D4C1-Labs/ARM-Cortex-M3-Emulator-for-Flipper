/* hosttest.c - Host-side validation of the AGNOSTIC emulator (no ESP-IDF).
 *
 * Boots the real keyfob firmware on the Thumb-2 core from the REAL reset vector,
 * honours the firmware's dynamic VTOR, fires only the IRQs the firmware enabled
 * (ISER-driven NVIC), injects the knock-code PURELY over GPIO, and dumps the
 * OLED framebuffer as ASCII. The SAME binary runs ANY keyfob .hex/.bin: there
 * are no per-firmware profiles or hardcoded code addresses in the emulator.
 *
 * The knock-code is FIRMWARE CONFIG (not hardware), so the HOST supplies it here
 * via emu_set_knock() -- the agnostic core itself neither knows nor needs it.
 *
 * Usage:
 *   hosttest <firmware.bin> [pandora|pmax|none] [--keys UP,OK,DOWN]
 *     arg2 selects which knock autotest to press over GPIO (default: pmax).
 *          'none' = just boot, no knock (verify boot + IRQ without unlock).
 *     --keys = after unlock, press these buttons over GPIO (agnostic nav).
 */
#include "main/emulator.h"
#include "main/ihex.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint8_t* load_file(const char* path, uint32_t* len_out) {
    FILE* f = fopen(path, "rb");
    if(!f) { fprintf(stderr, "cannot open %s\n", path); return NULL; }
    fseek(f, 0, SEEK_END);
    long n = ftell(f);
    fseek(f, 0, SEEK_SET);
    uint8_t* buf = malloc((size_t)n);
    if(fread(buf, 1, (size_t)n, f) != (size_t)n) { fclose(f); free(buf); return NULL; }
    fclose(f);
    *len_out = (uint32_t)n;
    return buf;
}

/* Knock-codes are FIRMWARE CONFIG supplied by the host (the keyfob model/build),
 * NOT part of the agnostic emulator. Pressed purely over GPIO like a finger. */
static const uint8_t KNOCK_PANDORA[] = {B_UP, B_OK, B_OK, B_UP, B_UP, B_DOWN};
static const uint8_t KNOCK_PMAX[]    = {B_UP, B_OK, B_DOWN, B_OK, B_UP, B_DOWN};

static int btn_from_name(const char* s) {
    if(!strcmp(s, "UP")) return B_UP;
    if(!strcmp(s, "DOWN")) return B_DOWN;
    if(!strcmp(s, "OK")) return B_OK;
    if(!strcmp(s, "BACK")) return B_BACK;
    if(!strcmp(s, "B5")) return B5;
    if(!strcmp(s, "B6")) return B6;
    return -1;
}

static int fb_nonzero(EmuState* e) {
    uint8_t fb[OLED_FB_SIZE];
    emu_get_fb(e, fb);
    int nz = 0;
    for(int i = 0; i < OLED_FB_SIZE; i++) if(fb[i]) nz++;
    return nz;
}

int main(int argc, char** argv) {
    const char* path = argc > 1 ? argv[1] : "../bins/PANDORA_MAX.flash.bin";
    const char* knock_sel = argc > 2 ? argv[2] : "pmax";
    const char* keys = NULL;
    for(int i = 3; i < argc - 1; i++)
        if(!strcmp(argv[i], "--keys")) keys = argv[i + 1];

    uint32_t raw_len = 0;
    uint8_t* raw = load_file(path, &raw_len);
    if(!raw) return 1;

    /* AGNOSTIC firmware load: accept a flat .bin (base-0 image) OR an Intel HEX
     * (.hex) parsed into a flat image -- exactly like the Python core's
     * load_firmware. The emulator does not care which; it just gets a flat image. */
    uint8_t* fw = raw;
    uint32_t fw_len = raw_len;
    size_t plen = strlen(path);
    bool is_hex = plen >= 4 && strcmp(path + plen - 4, ".hex") == 0;
    if(is_hex) {
        uint32_t cap = 0x44000u; /* flash window (256KB+); addrs >= cap dropped */
        uint8_t* flat = malloc(cap);
        memset(flat, 0xFF, cap);
        ihex_parser_t hp;
        ihex_init(&hp, flat, cap);
        if(ihex_parse_all(&hp, raw, raw_len) != IHEX_OK) {
            fprintf(stderr, "ihex parse failed\n");
            return 3;
        }
        fw = flat;
        fw_len = hp.highest;
        printf("parsed Intel HEX: highest=0x%X eof=%d\n", hp.highest, hp.eof);
    }

    /* The SAME single HW profile runs every firmware (agnostic). */
    const ArmProfile* prof = &PROFILE_PANDORA_HW;
    printf("loaded %s (%u bytes flat), hw=%s\n", path, fw_len, prof->name);

    EmuState emu;
    memset(&emu, 0, sizeof(emu));
    if(emu_init(&emu, prof, fw, fw_len) != 0) { fprintf(stderr, "emu_init failed\n"); return 2; }

    /* HOST supplies the knock autotest (firmware config, pressed over GPIO). */
    if(!strcmp(knock_sel, "pandora"))
        emu_set_knock(&emu, KNOCK_PANDORA, sizeof(KNOCK_PANDORA), B5);
    else if(!strcmp(knock_sel, "none"))
        emu_set_knock(&emu, NULL, 0, -1);
    else
        emu_set_knock(&emu, KNOCK_PMAX, sizeof(KNOCK_PMAX), B5);
    printf("knock=%s\n", knock_sel);

    emu_reset_and_unlock(&emu);
    printf("after reset+unlock: PC=0x%08X vtor=0x%08X nvic_enabled=0x%llX\n",
           thumb_get_reg(emu.cpu, 15), emu.ppb_vtor,
           (unsigned long long)emu.nvic_enabled);
    printf("  ninsn=%llu tick_count(IRQ fires)=%u oled_frames=%u unlocked=%d fb_nz=%d\n",
           (unsigned long long)emu.ninsn, emu.tick_count, emu.oled_frames,
           emu_is_unlocked(&emu), fb_nonzero(&emu));

    /* run a few more bursts and watch the IRQ tick keep firing */
    uint32_t t0 = emu.tick_count;
    for(int i = 0; i < 40; i++) emu_run_burst(&emu, 50000);
    printf("ran 40x50000 insn: oled_frames=%u IRQ fires +%u (total %u) fb_nz=%d\n",
           emu.oled_frames, emu.tick_count - t0, emu.tick_count, fb_nonzero(&emu));

    /* optional post-unlock navigation (agnostic GPIO presses) */
    if(keys) {
        char buf[128];
        strncpy(buf, keys, sizeof(buf) - 1);
        buf[sizeof(buf) - 1] = 0;
        for(char* tok = strtok(buf, ","); tok; tok = strtok(NULL, ",")) {
            int b = btn_from_name(tok);
            if(b < 0) { printf("  (unknown key %s)\n", tok); continue; }
            emu_press_button(&emu, b, false);
            printf("  pressed %s -> fb_nz=%d oled_frames=%u\n", tok,
                   fb_nonzero(&emu), emu.oled_frames);
        }
    }

    char ascii[4096];
    size_t n = emu_fb_ascii(&emu, ascii, sizeof(ascii));
    printf("--- OLED framebuffer (ASCII, %zu bytes) ---\n%.*s\n", n, (int)n, ascii);

    emu_deinit(&emu);
    free(fw);
    return 0;
}
