#ifndef HAL_UART_H
#define HAL_UART_H

#include <stddef.h>
#include <stdint.h>

/*
 * Minimal UART hardware-abstraction layer (byte stream + millisecond clock).
 *
 * The protocol stack never touches hardware directly. On a real target,
 * implement these with your UART peripheral (blocking or DMA-backed);
 * on the host, use mock_uart, which replays scripted RX bytes, captures
 * TX bytes, and provides a controllable clock for testing the RTU
 * inter-frame silence detection.
 */

typedef struct {
    /*
     * Read up to `cap` bytes, waiting at most `timeout_ms`.
     * Returns bytes read (0 = timeout), negative errno-style on error.
     */
    int (*read)(void *ctx, uint8_t *buf, size_t cap, uint32_t timeout_ms);
    /* Write `len` bytes. Returns 0 on success, negative on error. */
    int (*write)(void *ctx, const uint8_t *buf, size_t len);
    /* Monotonic millisecond clock (wraps every ~49 days; uint32 math is safe). */
    uint32_t (*now_ms)(void *ctx);
    void *ctx;
} hal_uart_t;

#endif /* HAL_UART_H */
