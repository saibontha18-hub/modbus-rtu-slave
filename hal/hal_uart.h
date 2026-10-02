#ifndef HAL_UART_H
#define HAL_UART_H

#include <stddef.h>
#include <stdint.h>

/*
 * Tiny UART abstraction: byte stream + ms clock.
 *
 * The stack never touches hardware directly. Implement these against your
 * UART peripheral on target; on the host, mock_uart replays scripted RX,
 * captures TX, and fakes the clock for the RTU silence detection.
 */

typedef struct {
    /* Read up to cap bytes, waiting at most timeout_ms.
       Returns bytes read (0 = timeout), negative on error. */
    int (*read)(void *ctx, uint8_t *buf, size_t cap, uint32_t timeout_ms);
    /* Write `len` bytes. Returns 0 on success, negative on error. */
    int (*write)(void *ctx, const uint8_t *buf, size_t len);
    /* Monotonic millisecond clock (wraps every ~49 days; uint32 math is safe). */
    uint32_t (*now_ms)(void *ctx);
    void *ctx;
} hal_uart_t;

#endif /* HAL_UART_H */
