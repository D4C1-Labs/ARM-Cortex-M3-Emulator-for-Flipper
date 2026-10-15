/*
 * uart_proto.c - ESP32 UART transport for the Pandora emulator link.
 * See uart_proto.h / protocol.h for the wire format.
 */
#include "uart_proto.h"

#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <string.h>

static const char* TAG = "uart_proto";

/* Receive state machine phases. */
enum {
    RX_SYNC = 0,  /* waiting for 0xA5 */
    RX_TYPE,      /* reading TYPE byte */
    RX_LEN0,      /* reading LEN low byte */
    RX_LEN1,      /* reading LEN high byte */
    RX_PAYLOAD,   /* reading PAYLOAD[LEN] */
    RX_CRC,       /* reading CRC8 */
};

#define RX_RING_SIZE 4096
#define TX_RING_SIZE 2048

int uart_proto_init(uart_proto_t* up) {
    if(!up) return -1;
    if(up->uart_num == 0 && up->baud == 0) {
        /* Defaults matching the user's wiring: UART0 on GPIO1(TX)/GPIO3(RX). */
        up->tx_gpio = 1;
        up->rx_gpio = 3;
        up->baud = 230400;
    }
    up->rx_state = RX_SYNC;

    uart_config_t cfg = {
        .baud_rate = up->baud,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err = uart_driver_install(up->uart_num, RX_RING_SIZE, TX_RING_SIZE, 0, NULL, 0);
    if(err != ESP_OK) { ESP_LOGE(TAG, "driver_install: %d", err); return -2; }
    err = uart_param_config(up->uart_num, &cfg);
    if(err != ESP_OK) { ESP_LOGE(TAG, "param_config: %d", err); return -3; }
    err = uart_set_pin(up->uart_num, up->tx_gpio, up->rx_gpio,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if(err != ESP_OK) { ESP_LOGE(TAG, "set_pin: %d", err); return -4; }

    ESP_LOGI(TAG, "UART%d ready: TX=GPIO%d RX=GPIO%d @ %d baud",
             up->uart_num, up->tx_gpio, up->rx_gpio, up->baud);
    return 0;
}

int uart_proto_send(uart_proto_t* up, uint8_t type, const uint8_t* payload, uint16_t len) {
    if(!up) return -1;
    if(len > PROTO_MAX_PAYLOAD) return -2;

    /* Header: TYPE + LEN(2 LE). CRC covers TYPE + LEN + PAYLOAD. */
    uint8_t hdr[4];
    hdr[0] = PROTO_SYNC;
    hdr[1] = type;
    hdr[2] = (uint8_t)(len & 0xFF);
    hdr[3] = (uint8_t)(len >> 8);

    uint8_t crc = 0x00u;
    crc = proto_crc8(&hdr[1], 3); /* TYPE + LEN */
    if(len) {
        /* Continue CRC over the payload (re-run; proto_crc8 is init 0, so we
         * compute incrementally by hashing the concatenation manually). */
        uint8_t tmp[3 + 0]; /* not used; kept for clarity */
        (void)tmp;
        /* Recompute over header+payload in one pass for correctness. */
        uint8_t c = 0x00u;
        for(int i = 1; i < 4; i++) {
            c ^= hdr[i];
            for(int b = 0; b < 8; b++)
                c = (c & 0x80u) ? (uint8_t)((c << 1) ^ 0x07u) : (uint8_t)(c << 1);
        }
        for(uint16_t i = 0; i < len; i++) {
            c ^= payload[i];
            for(int b = 0; b < 8; b++)
                c = (c & 0x80u) ? (uint8_t)((c << 1) ^ 0x07u) : (uint8_t)(c << 1);
        }
        crc = c;
    }

    uart_write_bytes(up->uart_num, (const char*)hdr, 4);
    if(len) uart_write_bytes(up->uart_num, (const char*)payload, len);
    uart_write_bytes(up->uart_num, (const char*)&crc, 1);
    return 0;
}

/* Running CRC over TYPE+LEN+PAYLOAD as bytes arrive, so the final compare is
 * cheap. We recompute fully here for simplicity/correctness. */
static uint8_t frame_crc(uint8_t type, uint16_t len, const uint8_t* payload) {
    uint8_t buf[3];
    buf[0] = type;
    buf[1] = (uint8_t)(len & 0xFF);
    buf[2] = (uint8_t)(len >> 8);
    uint8_t c = 0x00u;
    for(int i = 0; i < 3; i++) {
        c ^= buf[i];
        for(int b = 0; b < 8; b++)
            c = (c & 0x80u) ? (uint8_t)((c << 1) ^ 0x07u) : (uint8_t)(c << 1);
    }
    for(uint16_t i = 0; i < len; i++) {
        c ^= payload[i];
        for(int b = 0; b < 8; b++)
            c = (c & 0x80u) ? (uint8_t)((c << 1) ^ 0x07u) : (uint8_t)(c << 1);
    }
    return c;
}

int uart_proto_poll(uart_proto_t* up) {
    if(!up) return 0;
    int dispatched = 0;
    uint8_t byte;

    while(uart_read_bytes(up->uart_num, &byte, 1, 0) == 1) {
        switch(up->rx_state) {
        case RX_SYNC:
            if(byte == PROTO_SYNC) up->rx_state = RX_TYPE;
            break;
        case RX_TYPE:
            up->rx_type = byte;
            up->rx_state = RX_LEN0;
            break;
        case RX_LEN0:
            up->rx_lenbuf[0] = byte;
            up->rx_state = RX_LEN1;
            break;
        case RX_LEN1:
            up->rx_lenbuf[1] = byte;
            up->rx_len = (uint16_t)(up->rx_lenbuf[0] | (up->rx_lenbuf[1] << 8));
            up->rx_got = 0;
            if(up->rx_len > PROTO_MAX_PAYLOAD) {
                up->rx_state = RX_SYNC; /* bad length, resync */
            } else {
                up->rx_state = up->rx_len ? RX_PAYLOAD : RX_CRC;
            }
            break;
        case RX_PAYLOAD:
            up->rx_payload[up->rx_got++] = byte;
            if(up->rx_got >= up->rx_len) up->rx_state = RX_CRC;
            break;
        case RX_CRC: {
            uint8_t want = frame_crc(up->rx_type, up->rx_len, up->rx_payload);
            if(want == byte) {
                if(up->cb)
                    up->cb(up->rx_type,
                           up->rx_len ? up->rx_payload : NULL,
                           up->rx_len, up->cb_ctx);
                dispatched++;
            } else {
                ESP_LOGW(TAG, "CRC mismatch type=0x%02X len=%u", up->rx_type, up->rx_len);
            }
            up->rx_state = RX_SYNC;
            break;
        }
        default:
            up->rx_state = RX_SYNC;
            break;
        }
    }
    return dispatched;
}
