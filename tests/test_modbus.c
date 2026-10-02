/*
 * Self-contained test suite for the Modbus RTU slave (no external framework).
 * Each TEST() counts a pass/fail; main() returns nonzero on any failure.
 */
#include <stdio.h>
#include <string.h>

#include "modbus_rtu.h"

static int passes, failures;

#define TEST(name) \
    do { printf("  %-58s", name); } while (0)
/* Inside each test block, declare `int _ok = 1;` first, then use CHECK. */
#define CHECK(cond) \
    do { if (!(cond)) _ok = 0; } while (0)
#define END_T \
    do { \
        if (_ok) { passes++; printf("ok\n"); } \
        else { failures++; printf("FAIL\n"); } \
    } while (0)

/* Build a frame: addr, func, payload, CRC. Returns total length. */
static size_t frame(uint8_t addr, uint8_t func,
                    const uint8_t *p, size_t pn,
                    uint8_t *out)
{
    out[0] = addr;
    out[1] = func;
    if (pn)
        memcpy(out + 2, p, pn);
    uint16_t crc = modbus_crc16(out, 2 + pn);
    out[2 + pn] = (uint8_t)(crc & 0xFFu);
    out[3 + pn] = (uint8_t)(crc >> 8);
    return 4 + pn;
}

static bool crc_ok(const uint8_t *f, size_t n)
{
    uint16_t rx = (uint16_t)f[n - 2] | ((uint16_t)f[n - 1] << 8);
    return rx == modbus_crc16(f, n - 2);
}

static modbus_slave_t make_slave(uint16_t *h, size_t nh,
                                 uint16_t *in, size_t ni)
{
    modbus_slave_t s;
    s.slave_addr = 0x01;
    s.map.holding = h;
    s.map.n_holding = nh;
    s.map.input = in;
    s.map.n_input = ni;
    return s;
}

int main(void)
{
    uint8_t req[MODBUS_MAX_FRAME], resp[MODBUS_MAX_FRAME];
    size_t rlen;

    printf("crc16\n");
    TEST("known vector: crc16(\"123456789\") == 0x4B37");
    {
        int _ok = 1;
        const uint8_t v[] = "123456789";
        /* CRC-16/Modbus check value (init 0xFFFF); cross-checked against
           an independent Python implementation. */
        CHECK(modbus_crc16(v, 9) == 0x4B37u);
        END_T;
    }

    printf("read holding registers (0x03)\n");
    TEST("reads back programmed values with valid CRC");
    {
        int _ok = 1;
        uint16_t h[8] = { 0x1234, 0x5678 };
        uint16_t in[8] = { 0 };
        modbus_slave_t s = make_slave(h, 8, in, 8);
        uint8_t p[] = { 0x00, 0x00, 0x00, 0x02 };
        size_t qlen = frame(0x01, 0x03, p, sizeof(p), req);
        rlen = modbus_process_frame(&s, req, qlen, resp, sizeof(resp));
        CHECK(rlen == 9);
        CHECK(resp[0] == 0x01 && resp[1] == 0x03 && resp[2] == 0x04);
        CHECK(resp[3] == 0x12 && resp[4] == 0x34);
        CHECK(resp[5] == 0x56 && resp[6] == 0x78);
        CHECK(crc_ok(resp, rlen));
        END_T;
    }
    TEST("out-of-range read -> exception 0x02");
    {
        int _ok = 1;
        uint16_t h[8] = { 0 };
        uint16_t in[8] = { 0 };
        modbus_slave_t s = make_slave(h, 8, in, 8);
        uint8_t p[] = { 0x00, 0x07, 0x00, 0x02 }; /* reg 7 + qty 2 > 8 */
        size_t qlen = frame(0x01, 0x03, p, sizeof(p), req);
        rlen = modbus_process_frame(&s, req, qlen, resp, sizeof(resp));
        CHECK(rlen == 5);
        CHECK(resp[1] == (0x03 | 0x80) && resp[2] == 0x02);
        CHECK(crc_ok(resp, rlen));
        END_T;
    }
    TEST("quantity 0 -> exception 0x03");
    {
        int _ok = 1;
        uint16_t h[8] = { 0 };
        uint16_t in[8] = { 0 };
        modbus_slave_t s = make_slave(h, 8, in, 8);
        uint8_t p[] = { 0x00, 0x00, 0x00, 0x00 };
        size_t qlen = frame(0x01, 0x03, p, sizeof(p), req);
        rlen = modbus_process_frame(&s, req, qlen, resp, sizeof(resp));
        CHECK(rlen == 5 && resp[2] == 0x03);
        END_T;
    }

    printf("read input registers (0x04)\n");
    TEST("reads from the input bank, not holding");
    {
        int _ok = 1;
        uint16_t h[8] = { 0xAAAA };
        uint16_t in[8] = { 0x5555 };
        modbus_slave_t s = make_slave(h, 8, in, 8);
        uint8_t p[] = { 0x00, 0x00, 0x00, 0x01 };
        size_t qlen = frame(0x01, 0x04, p, sizeof(p), req);
        rlen = modbus_process_frame(&s, req, qlen, resp, sizeof(resp));
        CHECK(rlen == 7);
        CHECK(resp[3] == 0x55 && resp[4] == 0x55);
        CHECK(crc_ok(resp, rlen));
        END_T;
    }

    printf("write single register (0x06)\n");
    TEST("write applies and echo response matches request");
    {
        int _ok = 1;
        uint16_t h[8] = { 0 };
        uint16_t in[8] = { 0 };
        modbus_slave_t s = make_slave(h, 8, in, 8);
        uint8_t p[] = { 0x00, 0x03, 0xAB, 0xCD };
        size_t qlen = frame(0x01, 0x06, p, sizeof(p), req);
        rlen = modbus_process_frame(&s, req, qlen, resp, sizeof(resp));
        CHECK(rlen == 8);
        CHECK(h[3] == 0xABCDu);
        CHECK(memcmp(resp, req, 6) == 0 && crc_ok(resp, rlen));
        END_T;
    }
    TEST("write past the end -> exception 0x02, register untouched");
    {
        int _ok = 1;
        uint16_t h[8] = { 0x1111 };
        uint16_t in[8] = { 0 };
        modbus_slave_t s = make_slave(h, 8, in, 8);
        uint8_t p[] = { 0x00, 0x20, 0x00, 0x01 };
        size_t qlen = frame(0x01, 0x06, p, sizeof(p), req);
        rlen = modbus_process_frame(&s, req, qlen, resp, sizeof(resp));
        CHECK(rlen == 5 && resp[2] == 0x02);
        CHECK(h[0] == 0x1111); /* existing register untouched too */
        END_T;
    }

    printf("write multiple registers (0x10)\n");
    TEST("block write applies in order, response echoes address/quantity");
    {
        int _ok = 1;
        uint16_t h[8] = { 0 };
        uint16_t in[8] = { 0 };
        modbus_slave_t s = make_slave(h, 8, in, 8);
        uint8_t p[] = { 0x00, 0x02, 0x00, 0x03, 0x06,
                        0x11, 0x11, 0x22, 0x22, 0x33, 0x33 };
        size_t qlen = frame(0x01, 0x10, p, sizeof(p), req);
        rlen = modbus_process_frame(&s, req, qlen, resp, sizeof(resp));
        CHECK(rlen == 8);
        CHECK(h[2] == 0x1111 && h[3] == 0x2222 && h[4] == 0x3333);
        CHECK(resp[2] == 0x00 && resp[3] == 0x02);
        CHECK(resp[4] == 0x00 && resp[5] == 0x03);
        CHECK(crc_ok(resp, rlen));
        END_T;
    }
    TEST("byte count mismatch -> exception 0x03");
    {
        int _ok = 1;
        uint16_t h[8] = { 0 };
        uint16_t in[8] = { 0 };
        modbus_slave_t s = make_slave(h, 8, in, 8);
        uint8_t p[] = { 0x00, 0x00, 0x00, 0x02, 0x03, 0xAA, 0xBB };
        size_t qlen = frame(0x01, 0x10, p, sizeof(p), req);
        rlen = modbus_process_frame(&s, req, qlen, resp, sizeof(resp));
        CHECK(rlen == 5 && resp[2] == 0x03);
        END_T;
    }

    printf("framing and addressing rules\n");
    TEST("bad CRC -> silence (no response)");
    {
        int _ok = 1;
        uint16_t h[8] = { 0 };
        uint16_t in[8] = { 0 };
        modbus_slave_t s = make_slave(h, 8, in, 8);
        uint8_t p[] = { 0x00, 0x00, 0x00, 0x01 };
        size_t qlen = frame(0x01, 0x03, p, sizeof(p), req);
        req[qlen - 1] ^= 0xFF; /* corrupt the CRC */
        rlen = modbus_process_frame(&s, req, qlen, resp, sizeof(resp));
        CHECK(rlen == 0);
        END_T;
    }
    TEST("frame for another slave -> silence");
    {
        int _ok = 1;
        uint16_t h[8] = { 0 };
        uint16_t in[8] = { 0 };
        modbus_slave_t s = make_slave(h, 8, in, 8);
        uint8_t p[] = { 0x00, 0x00, 0x00, 0x01 };
        size_t qlen = frame(0x02, 0x03, p, sizeof(p), req);
        rlen = modbus_process_frame(&s, req, qlen, resp, sizeof(resp));
        CHECK(rlen == 0);
        END_T;
    }
    TEST("broadcast write applies but is never answered");
    {
        int _ok = 1;
        uint16_t h[8] = { 0 };
        uint16_t in[8] = { 0 };
        modbus_slave_t s = make_slave(h, 8, in, 8);
        uint8_t p[] = { 0x00, 0x05, 0xBE, 0xEF };
        size_t qlen = frame(0x00, 0x06, p, sizeof(p), req);
        rlen = modbus_process_frame(&s, req, qlen, resp, sizeof(resp));
        CHECK(rlen == 0);
        CHECK(h[5] == 0xBEEFu);
        END_T;
    }
    TEST("unsupported function -> exception 0x01");
    {
        int _ok = 1;
        uint16_t h[8] = { 0 };
        uint16_t in[8] = { 0 };
        modbus_slave_t s = make_slave(h, 8, in, 8);
        size_t qlen = frame(0x01, 0x2B, NULL, 0, req);
        rlen = modbus_process_frame(&s, req, qlen, resp, sizeof(resp));
        CHECK(rlen == 5);
        CHECK(resp[1] == (0x2B | 0x80) && resp[2] == 0x01);
        CHECK(crc_ok(resp, rlen));
        END_T;
    }

    printf("rtu receiver (silence-delimited framing)\n");
    TEST("frame becomes ready after the silence gap");
    {
        int _ok = 1;
        modbus_rx_t rx;
        modbus_rx_init(&rx);
        uint32_t t = 1000;
        for (int i = 0; i < 8; i++)
            CHECK(modbus_rx_put(&rx, (uint8_t)i, t) == 0);
        CHECK(!modbus_rx_ready(&rx, t + 2, 5));
        CHECK(modbus_rx_ready(&rx, t + 5, 5));
        CHECK(rx.len == 8 && rx.buf[0] == 0 && rx.buf[7] == 7);
        modbus_rx_reset(&rx);
        CHECK(rx.len == 0);
        END_T;
    }
    TEST("short runt (< 4 bytes) is never a frame");
    {
        int _ok = 1;
        modbus_rx_t rx;
        modbus_rx_init(&rx);
        modbus_rx_put(&rx, 0x01, 1000);
        modbus_rx_put(&rx, 0x03, 1000);
        CHECK(!modbus_rx_ready(&rx, 2000, 5));
        END_T;
    }
    TEST("buffer overflow is reported, not silently wrapped");
    {
        int _ok = 1;
        modbus_rx_t rx;
        modbus_rx_init(&rx);
        int rc = 0;
        for (size_t i = 0; i < MODBUS_MAX_FRAME + 10; i++)
            rc = modbus_rx_put(&rx, 0xAA, 1000);
        CHECK(rc != 0 && rx.overflow);
        END_T;
    }

    printf("\n%d passed, %d failed\n", passes, failures);
    return failures ? 1 : 0;
}
