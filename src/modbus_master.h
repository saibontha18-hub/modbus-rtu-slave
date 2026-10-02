#ifndef MODBUS_MASTER_H
#define MODBUS_MASTER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * Master-side request builders: slave addr + function + payload + CRC.
 * Handy for test harnesses or a real master.
 *
 * Each writes the full frame into `out` and returns its length, or 0 when
 * anything is out of spec (bad quantity, NULL pointer, cap too small) —
 * so 0 always means "no frame produced".
 *
 * Quantity limits per the spec:
 *   reads:  1..2000 coils, 1..125 registers
 *   writes: 1..1968 coils, 1..123 registers
 */

size_t modbus_build_read_coils(uint8_t slave, uint16_t start, uint16_t qty,
                               uint8_t *out, size_t cap);
size_t modbus_build_read_holding(uint8_t slave, uint16_t start, uint16_t qty,
                                 uint8_t *out, size_t cap);
size_t modbus_build_read_input(uint8_t slave, uint16_t start, uint16_t qty,
                               uint8_t *out, size_t cap);
size_t modbus_build_write_single_coil(uint8_t slave, uint16_t coil, bool on,
                                      uint8_t *out, size_t cap);
size_t modbus_build_write_single_reg(uint8_t slave, uint16_t reg, uint16_t val,
                                     uint8_t *out, size_t cap);
size_t modbus_build_write_multiple_coils(uint8_t slave, uint16_t start,
                                         const bool *vals, uint16_t qty,
                                         uint8_t *out, size_t cap);
size_t modbus_build_write_multiple_regs(uint8_t slave, uint16_t start,
                                        const uint16_t *vals, uint16_t qty,
                                        uint8_t *out, size_t cap);

#endif /* MODBUS_MASTER_H */
