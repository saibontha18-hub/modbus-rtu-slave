#ifndef MODBUS_RTU_H
#define MODBUS_RTU_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * Modbus RTU slave stack. No malloc, no libc I/O in here.
 *
 * Wire layout: [addr][function][data...][CRC lo][CRC hi]
 *
 * Handles 0x01/0x03/0x04 reads, 0x05/0x06 single writes, 0x0F/0x10 block
 * writes, plus the standard exceptions (0x01 bad function, 0x02 bad address,
 * 0x03 bad value). Bad CRC or wrong address gets silence, per the spec.
 * Broadcast writes (addr 0) apply without a reply.
 *
 * On real hardware, frames are delimited by 3.5 chars of line silence;
 * modbus_rx_t does that with an injectable ms clock so tests don't need
 * timers.
 */

#define MODBUS_BROADCAST_ADDR 0x00u
#define MODBUS_MAX_FRAME      256u

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
 * Handle one complete frame (req_len includes the 2 CRC bytes). Writes the
 * reply into resp and returns its length, or 0 when no reply is owed
 * (broadcast, bad CRC, wrong slave). Never writes more than
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
    bool overflow; /* sticky: frame buffer filled up and a byte was dropped */
} modbus_rx_t;

void modbus_rx_init(modbus_rx_t *rx);

/* Feed one received byte. Returns 0, or -ENOSPC if the frame buffer is full. */
int modbus_rx_put(modbus_rx_t *rx, uint8_t byte, uint32_t now_ms);

/*
 * True once a full frame is buffered: at least 4 bytes in, and the line
 * quiet for >= frame_gap_ms (use the 3.5-char time for your baud rate,
 * e.g. ~4 ms at 9600). Reset after consuming the frame.
 */
bool modbus_rx_ready(const modbus_rx_t *rx, uint32_t now_ms,
                     uint32_t frame_gap_ms);

void modbus_rx_reset(modbus_rx_t *rx);

#endif /* MODBUS_RTU_H */
