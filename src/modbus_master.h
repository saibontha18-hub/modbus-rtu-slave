#ifndef MODBUS_MASTER_H
#define MODBUS_MASTER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * Master-side request builders: construct valid Modbus RTU request frames
 * (slave address + function + payload + CRC-16) for use by a master
 * implementation, test harnesses, or integration scripts.
 *
 * Each builder writes the complete frame into `out` (capacity `cap`) and
 * returns the frame length. It returns 0 when any argument is out of spec
 * (bad quantity, coil index, NULL pointer) or when `cap` is too small —
 * so a zero return always means "no valid frame was produced".
 *
 * Quantity limits follow the Modbus specification:
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
