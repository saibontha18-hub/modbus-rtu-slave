#include "modbus_master.h"

#include "modbus_rtu.h"

/* Spec quantity limits (shared with the slave's validation). */
#define M_MAX_READ_REGS   125u
#define M_MAX_WRITE_REGS  123u
#define M_MAX_READ_COILS  2000u
#define M_MAX_WRITE_COILS 1968u

/* Append the CRC-16 to a frame of `len` payload bytes; return total length. */
static size_t finish(uint8_t *out, size_t len)
{
    uint16_t crc = modbus_crc16(out, len);
    out[len] = (uint8_t)(crc & 0xFFu);
    out[len + 1] = (uint8_t)(crc >> 8);
    return len + 2;
}

static bool qty_ok(uint16_t qty, uint16_t lo, uint16_t hi)
{
    return qty >= lo && qty <= hi;
}

size_t modbus_build_read_coils(uint8_t slave, uint16_t start, uint16_t qty,
                               uint8_t *out, size_t cap)
{
    if (!out || !qty_ok(qty, 1, M_MAX_READ_COILS) || cap < 8)
        return 0;
    out[0] = slave;
    out[1] = 0x01;
    out[2] = (uint8_t)(start >> 8);
    out[3] = (uint8_t)start;
    out[4] = (uint8_t)(qty >> 8);
    out[5] = (uint8_t)qty;
    return finish(out, 6);
}

size_t modbus_build_read_holding(uint8_t slave, uint16_t start, uint16_t qty,
                                 uint8_t *out, size_t cap)
{
    if (!out || !qty_ok(qty, 1, M_MAX_READ_REGS) || cap < 8)
        return 0;
    out[0] = slave;
    out[1] = 0x03;
    out[2] = (uint8_t)(start >> 8);
    out[3] = (uint8_t)start;
    out[4] = (uint8_t)(qty >> 8);
    out[5] = (uint8_t)qty;
    return finish(out, 6);
}

size_t modbus_build_read_input(uint8_t slave, uint16_t start, uint16_t qty,
                               uint8_t *out, size_t cap)
{
    if (!out || !qty_ok(qty, 1, M_MAX_READ_REGS) || cap < 8)
        return 0;
    out[0] = slave;
    out[1] = 0x04;
    out[2] = (uint8_t)(start >> 8);
    out[3] = (uint8_t)start;
    out[4] = (uint8_t)(qty >> 8);
    out[5] = (uint8_t)qty;
    return finish(out, 6);
}

size_t modbus_build_write_single_coil(uint8_t slave, uint16_t coil, bool on,
                                      uint8_t *out, size_t cap)
{
    if (!out || cap < 8)
        return 0;
    out[0] = slave;
    out[1] = 0x05;
    out[2] = (uint8_t)(coil >> 8);
    out[3] = (uint8_t)coil;
    out[4] = on ? 0xFFu : 0x00u;
    out[5] = 0x00u;
    return finish(out, 6);
}

size_t modbus_build_write_single_reg(uint8_t slave, uint16_t reg, uint16_t val,
                                     uint8_t *out, size_t cap)
{
    if (!out || cap < 8)
        return 0;
    out[0] = slave;
    out[1] = 0x06;
    out[2] = (uint8_t)(reg >> 8);
    out[3] = (uint8_t)reg;
    out[4] = (uint8_t)(val >> 8);
    out[5] = (uint8_t)val;
    return finish(out, 6);
}

size_t modbus_build_write_multiple_coils(uint8_t slave, uint16_t start,
                                         const bool *vals, uint16_t qty,
                                         uint8_t *out, size_t cap)
{
    uint8_t byte_count;
    size_t i, total;

    if (!out || !vals || !qty_ok(qty, 1, M_MAX_WRITE_COILS))
        return 0;
    byte_count = (uint8_t)((qty + 7u) / 8u);
    total = (size_t)7 + byte_count + 2;
    if (cap < total)
        return 0;
    out[0] = slave;
    out[1] = 0x0F;
    out[2] = (uint8_t)(start >> 8);
    out[3] = (uint8_t)start;
    out[4] = (uint8_t)(qty >> 8);
    out[5] = (uint8_t)qty;
    out[6] = byte_count;
    for (i = 0; i < byte_count; i++)
        out[7 + i] = 0;
    for (i = 0; i < qty; i++)
        if (vals[i])
            out[7 + i / 8u] |= (uint8_t)(1u << (i % 8u));
    return finish(out, 7 + byte_count);
}

size_t modbus_build_write_multiple_regs(uint8_t slave, uint16_t start,
                                        const uint16_t *vals, uint16_t qty,
                                        uint8_t *out, size_t cap)
{
    size_t i, total;

    if (!out || !vals || !qty_ok(qty, 1, M_MAX_WRITE_REGS))
        return 0;
    total = (size_t)7 + (size_t)qty * 2 + 2;
    if (cap < total)
        return 0;
    out[0] = slave;
    out[1] = 0x10;
    out[2] = (uint8_t)(start >> 8);
    out[3] = (uint8_t)start;
    out[4] = (uint8_t)(qty >> 8);
    out[5] = (uint8_t)qty;
    out[6] = (uint8_t)(qty * 2u);
    for (i = 0; i < qty; i++) {
        out[7 + i * 2] = (uint8_t)(vals[i] >> 8);
        out[8 + i * 2] = (uint8_t)(vals[i] & 0xFFu);
    }
    return finish(out, 7 + (size_t)qty * 2);
}
