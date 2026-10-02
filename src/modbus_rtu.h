#ifndef MODBUS_RTU_H
#define MODBUS_RTU_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * Modbus RTU slave protocol stack (no dynamic allocation, no libc I/O).
 *
 * Frame layout on the wire:
 *   [slave addr][function][data ...][CRC lo][CRC hi]
 *
 * Supported function codes:
 *   0x01  Read Coils                    (up to 2000 coils)
 *   0x03  Read Holding Registers        (up to 125 registers)
 *   0x04  Read Input Registers          (up to 125 registers)
 *   0x05  Write Single Coil             (0xFF00 = ON, 0x0000 = OFF)
 *   0x06  Write Single Register
 *   0x0F  Write Multiple Coils          (up to 1968 coils)
 *   0x10  Write Multiple Registers      (up to 123 registers)
 *
 * Anything else gets exception 0x01 (illegal function); out-of-range
 * accesses get 0x02 (illegal data address); bad quantities get 0x03
 * (illegal data value). Frames with a bad CRC, or addressed to a
 * different slave, are silently ignored per the spec. Broadcast
 * writes (address 0) are applied but never answered.
 *
 * RTU framing on real hardware uses a 3.5-character silence to delimit
 * frames; modbus_rx_t implements that with an injectable millisecond
 * clock so it is unit-testable without timers.
 */

#define MODBUS_BROADCAST_ADDR 0x00u
#define MODBUS_MAX_FRAME      256u

/* Exception codes. */
#define MODBUS_EX_ILLEGAL_FUNCTION 0x01u
#define MODBUS_EX_ILLEGAL_ADDRESS  0x02u
#define MODBUS_EX_ILLEGAL_VALUE    0x03u

/* Register/coil image backing the slave. Coils are bit-packed LSB-first:
 * coil N lives in bit (N % 8) of byte (N / 8), per the Modbus spec. */
typedef struct {
    uint16_t *holding;
    size_t n_holding;
    uint16_t *input;
    size_t n_input;
    uint8_t *coils;
    size_t n_coils;
} modbus_map_t;

typedef struct {
    uint8_t slave_addr; /* 1..247 */
    modbus_map_t map;
} modbus_slave_t;

/* CRC-16/Modbus (poly 0xA001, init 0xFFFF). */
uint16_t modbus_crc16(const uint8_t *data, size_t len);

/*
 * Process one complete RTU frame (`req_len` bytes INCLUDING the 2 CRC bytes).
 * Writes the response frame into `resp` (capacity `resp_cap`) and returns
 * the response length. Returns 0 when no response is due: broadcast frame,
 * CRC mismatch, or frame for another slave. Never writes more than
 * MODBUS_MAX_FRAME bytes.
 */
size_t modbus_process_frame(modbus_slave_t *slave,
                            const uint8_t *req, size_t req_len,
                            uint8_t *resp, size_t resp_cap);

/* ---- RTU receiver: silence-delimited framing ---- */

typedef struct {
    uint8_t buf[MODBUS_MAX_FRAME];
    size_t len;
    uint32_t last_ms;
    bool overflow; /* set when bytes arrived faster than we could frame them */
} modbus_rx_t;

void modbus_rx_init(modbus_rx_t *rx);

/* Feed one received byte. Returns 0, or -ENOSPC if the frame buffer is full. */
int modbus_rx_put(modbus_rx_t *rx, uint8_t byte, uint32_t now_ms);

/*
 * Returns true when a complete frame is sitting in rx->buf: at least 4 bytes
 * received and the line has been silent for >= frame_gap_ms (use the
 * 3.5-character time for your baud rate on target, e.g. ~4 ms at 9600 baud).
 * Call modbus_rx_reset() after consuming the frame.
 */
bool modbus_rx_ready(const modbus_rx_t *rx, uint32_t now_ms,
                     uint32_t frame_gap_ms);

void modbus_rx_reset(modbus_rx_t *rx);

#endif /* MODBUS_RTU_H */
