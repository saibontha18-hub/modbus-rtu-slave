#include "modbus_rtu.h"

#include <errno.h>
#include <string.h>

/* Function codes. */
#define FC_READ_HOLDING   0x03u
#define FC_READ_INPUT     0x04u
#define FC_WRITE_SINGLE   0x06u
#define FC_WRITE_MULTIPLE 0x10u

#define MAX_READ_QTY   125u
#define MAX_WRITE_QTY  123u

uint16_t modbus_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFFu;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 1u) ? (uint16_t)((crc >> 1) ^ 0xA001u)
                             : (uint16_t)(crc >> 1);
    }
    return crc;
}

void modbus_rx_init(modbus_rx_t *rx)
{
    memset(rx, 0, sizeof(*rx));
}

int modbus_rx_put(modbus_rx_t *rx, uint8_t byte, uint32_t now_ms)
{
    if (rx->len >= MODBUS_MAX_FRAME) {
        rx->overflow = true;
        return -ENOSPC;
    }
    rx->buf[rx->len++] = byte;
    rx->last_ms = now_ms;
    return 0;
}

bool modbus_rx_ready(const modbus_rx_t *rx, uint32_t now_ms,
                     uint32_t frame_gap_ms)
{
    return rx->len >= 4 && (now_ms - rx->last_ms) >= frame_gap_ms;
}

void modbus_rx_reset(modbus_rx_t *rx)
{
    rx->len = 0;
    rx->overflow = false;
}

/* ---- frame processing ---- */

static size_t build_exception(uint8_t addr, uint8_t func, uint8_t code,
                              uint8_t *resp, size_t cap)
{
    if (cap < 5)
        return 0;
    resp[0] = addr;
    resp[1] = (uint8_t)(func | 0x80u);
    resp[2] = code;
    uint16_t crc = modbus_crc16(resp, 3);
    resp[3] = (uint8_t)(crc & 0xFFu);
    resp[4] = (uint8_t)(crc >> 8);
    return 5;
}

static bool in_range(uint16_t start, uint16_t qty, size_t n)
{
    return (size_t)start + (size_t)qty <= n;
}

/* Returns response length, or 0 for "no response". */
size_t modbus_process_frame(modbus_slave_t *s,
                            const uint8_t *req, size_t req_len,
                            uint8_t *resp, size_t resp_cap)
{
    uint16_t crc_rx, crc_calc;
    uint8_t addr, func;
    bool broadcast;

    if (!s || !req || !resp || req_len < 4 || req_len > MODBUS_MAX_FRAME)
        return 0;

    /* CRC check first: never answer a corrupted frame. */
    crc_rx = (uint16_t)req[req_len - 2] | ((uint16_t)req[req_len - 1] << 8);
    crc_calc = modbus_crc16(req, req_len - 2);
    if (crc_rx != crc_calc)
        return 0;

    addr = req[0];
    func = req[1];
    if (addr != s->slave_addr && addr != MODBUS_BROADCAST_ADDR)
        return 0; /* not for us */
    broadcast = (addr == MODBUS_BROADCAST_ADDR);

    switch (func) {
    case FC_READ_HOLDING:
    case FC_READ_INPUT: {
        const uint16_t *bank;
        size_t n_bank;
        uint16_t start, qty;
        size_t rlen;

        if (broadcast)
            return 0; /* reads are never broadcast */
        if (req_len != 8)
            return 0; /* malformed: ignore */
        if (func == FC_READ_HOLDING) {
            bank = s->map.holding;
            n_bank = s->map.n_holding;
        } else {
            bank = s->map.input;
            n_bank = s->map.n_input;
        }
        start = (uint16_t)((req[2] << 8) | req[3]);
        qty = (uint16_t)((req[4] << 8) | req[5]);
        if (qty < 1 || qty > MAX_READ_QTY)
            return build_exception(addr, func, MODBUS_EX_ILLEGAL_VALUE,
                                   resp, resp_cap);
        if (!bank || !in_range(start, qty, n_bank))
            return build_exception(addr, func, MODBUS_EX_ILLEGAL_ADDRESS,
                                   resp, resp_cap);

        rlen = 3 + (size_t)qty * 2 + 2;
        if (rlen > resp_cap)
            return 0;
        resp[0] = addr;
        resp[1] = func;
        resp[2] = (uint8_t)(qty * 2);
        for (uint16_t i = 0; i < qty; i++) {
            resp[3 + i * 2] = (uint8_t)(bank[start + i] >> 8);
            resp[4 + i * 2] = (uint8_t)(bank[start + i] & 0xFFu);
        }
        crc_calc = modbus_crc16(resp, rlen - 2);
        resp[rlen - 2] = (uint8_t)(crc_calc & 0xFFu);
        resp[rlen - 1] = (uint8_t)(crc_calc >> 8);
        return rlen;
    }

    case FC_WRITE_SINGLE: {
        uint16_t reg, val;

        if (req_len != 8)
            return 0;
        reg = (uint16_t)((req[2] << 8) | req[3]);
        val = (uint16_t)((req[4] << 8) | req[5]);
        if (!s->map.holding || !in_range(reg, 1, s->map.n_holding)) {
            if (broadcast)
                return 0;
            return build_exception(addr, func, MODBUS_EX_ILLEGAL_ADDRESS,
                                   resp, resp_cap);
        }
        s->map.holding[reg] = val;
        if (broadcast)
            return 0; /* applied, never answered */
        /* Normal response echoes the request. */
        if (resp_cap < 8)
            return 0;
        memcpy(resp, req, 6);
        crc_calc = modbus_crc16(resp, 6);
        resp[6] = (uint8_t)(crc_calc & 0xFFu);
        resp[7] = (uint8_t)(crc_calc >> 8);
        return 8;
    }

    case FC_WRITE_MULTIPLE: {
        uint16_t start, qty;
        uint8_t byte_count;
        size_t rlen;

        if (req_len < 9)
            return 0;
        start = (uint16_t)((req[2] << 8) | req[3]);
        qty = (uint16_t)((req[4] << 8) | req[5]);
        byte_count = req[6];
        if (qty < 1 || qty > MAX_WRITE_QTY || byte_count != qty * 2 ||
            req_len != (size_t)7 + byte_count + 2) {
            if (broadcast)
                return 0;
            return build_exception(addr, func, MODBUS_EX_ILLEGAL_VALUE,
                                   resp, resp_cap);
        }
        if (!s->map.holding || !in_range(start, qty, s->map.n_holding)) {
            if (broadcast)
                return 0;
            return build_exception(addr, func, MODBUS_EX_ILLEGAL_ADDRESS,
                                   resp, resp_cap);
        }
        for (uint16_t i = 0; i < qty; i++)
            s->map.holding[start + i] =
                (uint16_t)((req[7 + i * 2] << 8) | req[8 + i * 2]);
        if (broadcast)
            return 0;
        if (resp_cap < 8)
            return 0;
        resp[0] = addr;
        resp[1] = func;
        resp[2] = req[2];
        resp[3] = req[3];
        resp[4] = req[4];
        resp[5] = req[5];
        crc_calc = modbus_crc16(resp, 6);
        resp[6] = (uint8_t)(crc_calc & 0xFFu);
        resp[7] = (uint8_t)(crc_calc >> 8);
        rlen = 8;
        return rlen;
    }

    default:
        if (broadcast)
            return 0;
        return build_exception(addr, func, MODBUS_EX_ILLEGAL_FUNCTION,
                               resp, resp_cap);
    }
}
