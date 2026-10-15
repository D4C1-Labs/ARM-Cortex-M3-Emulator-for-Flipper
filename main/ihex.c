/*
 * ihex.c - Incremental Intel HEX parser (see ihex.h).
 *
 * Mirrors emu/ihex.py exactly: same record handling, same address math, same
 * flat window semantics as hex2bin.py's "flash.bin (codigo)". Verified on the
 * host to produce byte-identical output to bins/PANDORA_MAX.flash.bin in the
 * used range (see ihex_test.c).
 */
#include "ihex.h"

/* Decode one ASCII hex nibble, or -1 if not a hex digit. */
static int hex_nib(char c) {
    if(c >= '0' && c <= '9') return c - '0';
    if(c >= 'a' && c <= 'f') return c - 'a' + 10;
    if(c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

/* Decode one ASCII hex byte (two nibbles) at s. Returns the byte or -1. */
static int hex_byte(const char* s) {
    int hi = hex_nib(s[0]);
    int lo = hex_nib(s[1]);
    if(hi < 0 || lo < 0) return -1;
    return (hi << 4) | lo;
}

void ihex_init(ihex_parser_t* p, uint8_t* flat, uint32_t cap) {
    p->flat = flat;
    p->cap = cap;
    p->base_linear = 0;
    p->base_segment = 0;
    p->highest = 0;
    p->hex_bytes = 0;
    p->eof = false;
    p->line_len = 0;
}

/* Parse one complete Intel HEX record line (without the trailing newline). The
 * line must begin with ':'. Non-record lines (blank, or not starting with ':')
 * are ignored, matching the Python loader. */
static int ihex_parse_line(ihex_parser_t* p, const char* line, uint16_t len) {
    /* Ignore blank lines and anything not starting with a record marker. */
    if(len == 0 || line[0] != ':') return IHEX_OK;

    /* Minimum record: ':' + count(1) + addr(2) + type(1) + checksum(1) = 11 chars. */
    if(len < 11) return IHEX_ERR_LINE;

    int count = hex_byte(line + 1);
    if(count < 0) return IHEX_ERR_LINE;

    /* Expected total length: ':' + (count + 5 bytes) * 2 hex chars. */
    uint16_t need = (uint16_t)(1 + (count + 5) * 2);
    if(len < need) return IHEX_ERR_LINE;

    int ah = hex_byte(line + 3);
    int al = hex_byte(line + 5);
    int rtype = hex_byte(line + 7);
    if(ah < 0 || al < 0 || rtype < 0) return IHEX_ERR_LINE;
    uint16_t addr = (uint16_t)((ah << 8) | al);

    /* Running checksum: two's complement of the sum of all record bytes incl the
     * checksum byte itself must be zero (i.e. sum of all bytes & 0xFF == 0). */
    uint8_t sum = (uint8_t)count + (uint8_t)ah + (uint8_t)al + (uint8_t)rtype;

    /* Decode the data bytes. */
    uint8_t data[256];
    const char* dp = line + 9;
    for(int i = 0; i < count; i++) {
        int b = hex_byte(dp + i * 2);
        if(b < 0) return IHEX_ERR_LINE;
        data[i] = (uint8_t)b;
        sum += (uint8_t)b;
    }

    int cks = hex_byte(dp + count * 2);
    if(cks < 0) return IHEX_ERR_LINE;
    sum += (uint8_t)cks;
    if(sum != 0) return IHEX_ERR_CHECKSUM;

    switch(rtype) {
    case 0x00: { /* data */
        uint32_t base = p->base_linear + p->base_segment + addr;
        for(int i = 0; i < count; i++) {
            uint32_t a = base + (uint32_t)i;
            if(a < p->cap) { /* inside the flat code window */
                p->flat[a] = data[i];
                if(a + 1 > p->highest) p->highest = a + 1;
            }
            /* addresses >= cap (e.g. EFM32 user/lock @0x0FE00000) are dropped,
             * matching hex2bin.py's CODE_LIMIT behaviour. */
        }
        break;
    }
    case 0x01: /* EOF */
        p->eof = true;
        break;
    case 0x02: /* extended segment address */
        if(count < 2) return IHEX_ERR_LINE;
        p->base_segment = (uint32_t)(((data[0] << 8) | data[1]) << 4);
        break;
    case 0x04: /* extended linear address */
        if(count < 2) return IHEX_ERR_LINE;
        p->base_linear = (uint32_t)(((data[0] << 8) | data[1]) << 16);
        break;
    case 0x03: /* start segment address - entry point, ignore */
    case 0x05: /* start linear address  - entry point, ignore */
        break;
    default:
        return IHEX_ERR_RECTYPE;
    }
    return IHEX_OK;
}

int ihex_feed(ihex_parser_t* p, const uint8_t* data, uint32_t len) {
    p->hex_bytes += len;
    for(uint32_t i = 0; i < len; i++) {
        char c = (char)data[i];
        if(c == '\n' || c == '\r') {
            /* End of a line: parse what we have (if any) and reset the carry. */
            if(p->line_len > 0) {
                int rc = ihex_parse_line(p, p->line, p->line_len);
                p->line_len = 0;
                if(rc != IHEX_OK) return rc;
            }
            continue;
        }
        if(p->line_len >= IHEX_LINE_MAX) {
            /* Line longer than any valid record: corrupt stream. */
            return IHEX_ERR_OVERFLOW;
        }
        p->line[p->line_len++] = c;
    }
    return IHEX_OK;
}

int ihex_parse_all(ihex_parser_t* p, const uint8_t* data, uint32_t len) {
    int rc = ihex_feed(p, data, len);
    if(rc != IHEX_OK) return rc;
    /* Flush a final line that was not newline-terminated. */
    if(p->line_len > 0) {
        rc = ihex_parse_line(p, p->line, p->line_len);
        p->line_len = 0;
    }
    return rc;
}
