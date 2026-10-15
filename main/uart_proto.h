/*
 * uart_proto.h - ESP32-side transport for the Pandora emulator <-> Flipper link.
 *
 * Wraps the ESP-IDF UART driver and the wire framing in protocol.h. One UART is
 * used (default UART0 on GPIO1/GPIO3, matching the user's wiring to the Flipper
 * GPIO TX/RX pins). Provides:
 *   - uart_proto_init()        : configure the UART + install the driver.
 *   - uart_proto_send()        : frame + write one message (SYNC/TYPE/LEN/CRC8).
 *   - uart_proto_poll()        : non-blocking receive; dispatches complete frames
 *                                to a user callback. Handles resync on SYNC.
 *
 * All encode/decode uses the header-only CRC8 and constants from protocol.h, so
 * the Flipper companion app links the same protocol.h and stays byte-compatible.
 */
#ifndef PANDORA_UART_PROTO_H
#define PANDORA_UART_PROTO_H

#include "protocol.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Called for every fully-received, CRC-valid inbound frame. payload may be NULL
 * when len == 0. The buffer is only valid for the duration of the call. */
typedef void (*uart_proto_rx_cb)(uint8_t type, const uint8_t* payload, uint16_t len, void* ctx);

typedef struct {
    int uart_num;         /* UART_NUM_0 by default */
    int tx_gpio;          /* GPIO1 by default */
    int rx_gpio;          /* GPIO3 by default */
    int baud;             /* 115200 (or higher) */
    uart_proto_rx_cb cb;  /* inbound frame callback */
    void* cb_ctx;

    /* Internal receive state machine (do not touch from outside). */
    uint8_t rx_state;
    uint8_t rx_type;
    uint16_t rx_len;
    uint16_t rx_got;
    uint8_t rx_lenbuf[2];
    uint8_t rx_payload[PROTO_MAX_PAYLOAD];
} uart_proto_t;

/* Configure + install the UART driver. Returns 0 on success. */
int uart_proto_init(uart_proto_t* up);

/* Encode and transmit one frame. Returns 0 on success, <0 on error. */
int uart_proto_send(uart_proto_t* up, uint8_t type, const uint8_t* payload, uint16_t len);

/* Drain the UART RX FIFO; call the callback for each complete valid frame.
 * Non-blocking (timeout 0). Returns the number of frames dispatched. */
int uart_proto_poll(uart_proto_t* up);

#endif /* PANDORA_UART_PROTO_H */
