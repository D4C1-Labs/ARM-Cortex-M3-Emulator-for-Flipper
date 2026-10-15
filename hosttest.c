/* hosttest.c - Host-side validation of the ported emulator (no ESP-IDF).
 * Boots the real Pandora firmware on the Thumb-2 core, runs auto-unlock,
 * runs a few bursts, and dumps the OLED framebuffer as ASCII to confirm the
 * ported MMIO/Si4432/OLED/run_burst/auto_unlock logic works before flashing. */
#include "main/emulator.h"
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

int main(int argc, char** argv) {
    const char* path = argc > 1 ? argv[1] : "../bins/PANDORA_MAX.flash.bin";
    const ArmProfile* prof = &PROFILE_PANDORAMAX;
    if(argc > 2 && strcmp(argv[2], "pandora") == 0) prof = &PROFILE_PANDORA;

    uint32_t fw_len = 0;
    uint8_t* fw = load_file(path, &fw_len);
    if(!fw) return 1;
    printf("loaded %s (%u bytes), profile=%s\n", path, fw_len, prof->name);

    EmuState emu;
    memset(&emu, 0, sizeof(emu));
    if(emu_init(&emu, prof, fw, fw_len) != 0) { fprintf(stderr, "emu_init failed\n"); return 2; }

    emu_reset_and_unlock(&emu);
    printf("after reset+unlock: PC=0x%08X\n", thumb_get_reg(emu.cpu, 15));

    for(int i = 0; i < 40; i++) emu_run_burst(&emu, 50000);
    printf("ran 40x50000 insn, oled_frames=%u\n", emu.oled_frames);

    char ascii[4096];
    size_t n = emu_fb_ascii(&emu, ascii, sizeof(ascii));
    printf("--- OLED framebuffer (ASCII, %zu bytes) ---\n%.*s\n", n, (int)n, ascii);

    emu_deinit(&emu);
    free(fw);
    return 0;
}
