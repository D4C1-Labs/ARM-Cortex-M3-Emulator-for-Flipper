/*
 * ihex.h - Incremental Intel HEX parser that writes straight into a flat image.
 *
 * Designed for streaming the keyfob firmware over UART: the Flipper reads the
 * .hex (ASCII) from its SD card and sends it in chunks (PROTO_MSG_FW_CHUNK). We
 * parse each chunk LINE BY LINE as it arrives, decoding every Intel HEX record
 * directly into a caller-provided flat buffer. The raw .hex text (~193 KB) is
 * therefore NEVER buffered whole: at most one partial line is carried over
 * between chunks.
 *
 * Supported record types (same semantics as emu/ihex.py, the reference parser):
 *   0x00 data                       - placed at base_linear + base_segment + addr
 *   0x01 end of file                - marks parsing complete
 *   0x02 extended segment address   - base_segment = (hi<<8|lo) << 4
 *   0x03 start segment address      - ignored (entry point)
 *   0x04 extended linear address    - base_linear  = (hi<<8|lo) << 16
 *   0x05 start linear address       - ignored (entry point)
 *
 * FLAT MODEL: the keyfob code lives at linear address 0. We mirror hex2bin.py's
 * "flash.bin (codigo)": only bytes whose absolute address falls inside the flat
 * window [0, cap) are kept (special EFM32 user/lock pages at 0x0FE00000 are far
 * above cap and are dropped, exactly like CODE_LIMIT in hex2bin.py). The flat
 * buffer must be pre-filled with 0xFF by the caller (open-bus / erased flash),
 * matching the Python filler.
 *
 * Every data/structural byte is checksum-verified per record.
 */
#ifndef PANDORA_IHEX_H
#define PANDORA_IHEX_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* Result codes returned by the parser calls. */
enum {
    IHEX_OK = 0, /* chunk consumed OK (EOF record may or may not have been seen) */
    IHEX_ERR_LINE = -1, /* malformed record (too short / bad hex) */
    IHEX_ERR_CHECKSUM = -2, /* record checksum mismatch */
    IHEX_ERR_RECTYPE = -3, /* unknown record type */
    IHEX_ERR_OVERFLOW = -4, /* partial line longer than the line buffer */
};

/* Max characters in one Intel HEX line (": " + up to 255 data bytes). The keyfob
 * .hex uses 0x20 (32) data bytes/line, so 1 + 2 + 4 + 2 + 255*2 + 2 + slack is
 * plenty; we size for the 0xFF data-count worst case. */
#define IHEX_LINE_MAX 600

typedef struct {
    uint8_t* flat; /* caller-owned flat image, pre-filled with 0xFF */
    uint32_t cap; /* flat buffer capacity (= flash_used); addrs >= cap dropped */

    uint32_t base_linear; /* extended linear address base (type 04) */
    uint32_t base_segment; /* extended segment address base (type 02) */

    uint32_t highest; /* highest flat offset written + 1 (effective size) */
    uint32_t hex_bytes; /* total raw .hex bytes fed so far (diagnostic) */
    bool eof; /* set once a type-01 record is parsed */

    /* Carry buffer for a line split across chunk boundaries. */
    char line[IHEX_LINE_MAX];
    uint16_t line_len;
} ihex_parser_t;

/* (Re)initialize the parser onto a flat buffer. flat must be pre-filled 0xFF. */
void ihex_init(ihex_parser_t* p, uint8_t* flat, uint32_t cap);

/* Feed one chunk of raw .hex text. Completed lines are parsed and written into
 * the flat buffer; a trailing partial line is carried into the next call.
 * Returns IHEX_OK, or a negative IHEX_ERR_* on the first malformed record. */
int ihex_feed(ihex_parser_t* p, const uint8_t* data, uint32_t len);

/* Parse a whole in-memory .hex buffer in one shot (used by the host test).
 * Equivalent to ihex_feed() over the entire buffer. */
int ihex_parse_all(ihex_parser_t* p, const uint8_t* data, uint32_t len);

#endif /* PANDORA_IHEX_H */
