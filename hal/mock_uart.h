#ifndef MOCK_UART_H
#define MOCK_UART_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "hal_uart.h"

/*
 * Fake UART for tests: script RX bytes, advance a fake ms clock, inspect
 * what got written to TX. fail_read simulates a peripheral error so the
 * error paths get exercised too.
 */

#define MOCK_UART_BUF 1024

typedef struct {
    uint8_t rx[MOCK_UART_BUF];
    size_t rx_len;   /* bytes scripted */
    size_t rx_pos;   /* bytes consumed */
    uint8_t tx[MOCK_UART_BUF];
    size_t tx_len;   /* bytes written by the app */
    uint32_t now;    /* fake millisecond clock */
    bool fail_read;
    hal_uart_t uart; /* interface wired to this mock */
} mock_uart_t;

void mock_uart_init(mock_uart_t *m);

/* Script `len` bytes for the app to read (appends to the RX fifo). */
void mock_uart_feed_rx(mock_uart_t *m, const uint8_t *data, size_t len);

/* Move the fake clock forward (drives RTU silence detection). */
void mock_uart_advance_ms(mock_uart_t *m, uint32_t ms);

/* Inspect what the app transmitted. */
const uint8_t *mock_uart_tx_data(const mock_uart_t *m);
size_t mock_uart_tx_len(const mock_uart_t *m);
void mock_uart_tx_clear(mock_uart_t *m);

/* Make the next read() call fail with -EIO. */
void mock_uart_fail_next_read(mock_uart_t *m);

const hal_uart_t *mock_uart_bus(mock_uart_t *m);

#endif /* MOCK_UART_H */
