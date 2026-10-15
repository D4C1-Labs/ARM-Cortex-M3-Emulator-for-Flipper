/*
 * ihex_test.c - HOST validation for the Intel HEX parser (no ESP-IDF needed).
 *
 * Parses a .hex into a flat image and compares it byte-for-byte against the
 * reference .flash.bin produced by emu/hex2bin.py, in the used range.
 *
 * Build & run (host):
 *   gcc -std=c11 -O2 -I main main/ihex.c main/ihex_test.c -o /tmp/iht
 *   /tmp/iht PANDORA_MAX.hex bins/PANDORA_MAX.flash.bin 0x15400
 *
 * It also exercises the INCREMENTAL path (random-sized chunks) to prove that
 * streaming the .hex over UART yields the exact same flat image.
 */
#include "ihex.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint8_t* read_file(const char* path, long* out_len) {
    FILE* f = fopen(path, "rb");
    if(!f) { fprintf(stderr, "cannot open %s\n", path); return NULL; }
    fseek(f, 0, SEEK_END);
    long n = ftell(f);
    fseek(f, 0, SEEK_SET);
    uint8_t* buf = (uint8_t*)malloc((size_t)n);
    if(!buf) { fclose(f); return NULL; }
    if(fread(buf, 1, (size_t)n, f) != (size_t)n) { free(buf); fclose(f); return NULL; }
    fclose(f);
    *out_len = n;
    return buf;
}

/* Compare a freshly-parsed flat image against the reference .flash.bin. */
static int compare_flat(const uint8_t* flat, uint32_t cap, uint32_t highest,
                        const uint8_t* ref, long ref_len, const char* label) {
    int mismatches = 0;
    long first = -1;
    /* The .flash.bin spans [0, ref_len); our flat spans [0, cap). Compare the
     * overlap and treat bytes beyond the shorter as 0xFF (erased). */
    uint32_t n = cap > (uint32_t)ref_len ? cap : (uint32_t)ref_len;
    for(uint32_t i = 0; i < n; i++) {
        uint8_t a = (i < cap) ? flat[i] : 0xFF;
        uint8_t b = (i < (uint32_t)ref_len) ? ref[i] : 0xFF;
        if(a != b) {
            if(first < 0) first = (long)i;
            mismatches++;
        }
    }
    printf("[%s] cap=0x%X highest=0x%X ref=0x%lX : %s\n", label, cap, highest,
           (unsigned long)ref_len, mismatches ? "MISMATCH" : "IDENTICAL");
    if(mismatches) {
        printf("    %d mismatching bytes, first at 0x%lX (flat=0x%02X ref=0x%02X)\n",
               mismatches, first,
               (first < (long)cap) ? flat[first] : 0xFF,
               (first < ref_len) ? ref[first] : 0xFF);
    }
    return mismatches == 0 ? 0 : 1;
}

int main(int argc, char** argv) {
    if(argc < 4) {
        fprintf(stderr, "usage: %s <in.hex> <ref.flash.bin> <cap_hex>\n", argv[0]);
        return 2;
    }
    const char* hexpath = argv[1];
    const char* refpath = argv[2];
    uint32_t cap = (uint32_t)strtoul(argv[3], NULL, 0);

    long hex_len = 0, ref_len = 0;
    uint8_t* hex = read_file(hexpath, &hex_len);
    uint8_t* ref = read_file(refpath, &ref_len);
    if(!hex || !ref) return 2;

    int fail = 0;

    /* ---- Pass 1: parse the whole .hex in one shot. ---- */
    uint8_t* flat = (uint8_t*)malloc(cap);
    memset(flat, 0xFF, cap);
    ihex_parser_t p;
    ihex_init(&p, flat, cap);
    int rc = ihex_parse_all(&p, hex, (uint32_t)hex_len);
    if(rc != IHEX_OK) { printf("one-shot parse rc=%d\n", rc); fail = 1; }
    if(!p.eof) printf("WARNING: no EOF record seen\n");
    fail |= compare_flat(flat, cap, p.highest, ref, ref_len, "one-shot");

    /* ---- Pass 2: stream the .hex in odd-sized chunks (UART simulation). ---- */
    uint8_t* flat2 = (uint8_t*)malloc(cap);
    memset(flat2, 0xFF, cap);
    ihex_parser_t p2;
    ihex_init(&p2, flat2, cap);
    const uint32_t chunk_sizes[] = { 1, 7, 37, 64, 255, 500, 1337 };
    uint32_t off = 0, ci = 0;
    while(off < (uint32_t)hex_len) {
        uint32_t cs = chunk_sizes[ci++ % (sizeof(chunk_sizes) / sizeof(chunk_sizes[0]))];
        if(off + cs > (uint32_t)hex_len) cs = (uint32_t)hex_len - off;
        rc = ihex_feed(&p2, hex + off, cs);
        if(rc != IHEX_OK) { printf("streaming feed rc=%d at off=%u\n", rc, off); fail = 1; break; }
        off += cs;
    }
    /* Flush a possibly non-terminated last line. */
    if(p2.line_len > 0) rc = ihex_feed(&p2, (const uint8_t*)"\n", 1);
    fail |= compare_flat(flat2, cap, p2.highest, ref, ref_len, "streaming");

    /* ---- Cross-check the two flat images match each other. ---- */
    if(memcmp(flat, flat2, cap) != 0) {
        printf("[cross] one-shot vs streaming: MISMATCH\n");
        fail = 1;
    } else {
        printf("[cross] one-shot vs streaming: IDENTICAL\n");
    }

    free(hex); free(ref); free(flat); free(flat2);
    printf("\nRESULT: %s\n", fail ? "FAIL" : "PASS");
    return fail;
}
