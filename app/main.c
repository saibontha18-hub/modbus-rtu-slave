/*
 * Demo: slave on a fake UART, scripted master session.
 * Polls for bytes, frames them by line silence, answers, hex-dumps it all.
 */
#include <stdio.h>

#include "mock_uart.h"
#include "modbus_rtu.h"

#define SLAVE_ADDR   0x01u
#define FRAME_GAP_MS 5u

/* Demo register image: holding[0] = temperature in 0.01 degC, etc. */
static uint16_t holding[16];
static uint16_t input_regs[16];
static uint8_t coils[4]; /* 32 coils, bit-packed LSB-first */

/* frame builder: addr + func + payload + CRC into out */
static size_t make_req(uint8_t addr, uint8_t func,
                       const uint8_t *payload, size_t payload_len,
                       uint8_t *out)
{
    out[0] = addr;
    out[1] = func;
    for (size_t i = 0; i < payload_len; i++)
        out[2 + i] = payload[i];
    uint16_t crc = modbus_crc16(out, 2 + payload_len);
    out[2 + payload_len] = (uint8_t)(crc & 0xFFu);
    out[3 + payload_len] = (uint8_t)(crc >> 8);
    return 4 + payload_len;
}

static void hexdump(const char *tag, const uint8_t *p, size_t n)
{
    printf("%-8s (%2zu bytes): ", tag, n);
    for (size_t i = 0; i < n; i++)
        printf("%02X ", p[i]);
    printf("\n");
}

/* one frame, byte by byte with 1 ms gaps, then silence to delimit it */
static void script_frame(mock_uart_t *muart, uint8_t addr, uint8_t func,
                         const uint8_t *payload, size_t payload_len)
{
    uint8_t frame[MODBUS_MAX_FRAME];
    size_t n = make_req(addr, func, payload, payload_len, frame);
    for (size_t i = 0; i < n; i++) {
        mock_uart_feed_rx(muart, &frame[i], 1);
        mock_uart_advance_ms(muart, 1);
    }
    mock_uart_advance_ms(muart, FRAME_GAP_MS + 1);
}

/* poll until the scripted bytes are drained and answered */
static void run_cycle(const hal_uart_t *uart, mock_uart_t *muart,
                      modbus_slave_t *slave, modbus_rx_t *rx)
{
    uint8_t resp[MODBUS_MAX_FRAME];
    uint8_t byte;

    for (;;) {
        int rc = uart->read(uart->ctx, &byte, 1, 0);
        if (rc < 0) {
            printf("uart read error\n");
            return;
        }
        if (rc == 1)
            modbus_rx_put(rx, byte, uart->now_ms(uart->ctx));

        if (modbus_rx_ready(rx, uart->now_ms(uart->ctx), FRAME_GAP_MS)) {
            hexdump("RX", rx->buf, rx->len);
            size_t rlen = modbus_process_frame(slave, rx->buf, rx->len,
                                               resp, sizeof(resp));
            if (rlen > 0) {
                hexdump("TX", resp, rlen);
                uart->write(uart->ctx, resp, rlen);
            } else {
                printf("no response (broadcast / bad frame)\n");
            }
            modbus_rx_reset(rx);
        }

        if (muart->rx_pos == muart->rx_len && rx->len == 0)
            break;
        mock_uart_advance_ms(muart, 1);
    }
}

int main(void)
{
    mock_uart_t muart;
    const hal_uart_t *uart;
    modbus_slave_t slave;
    modbus_rx_t rx;

    mock_uart_init(&muart);
    uart = mock_uart_bus(&muart);
    modbus_rx_init(&rx);

    holding[0] = 2345;  /* 23.45 C */
    holding[1] = 0x0000;
    input_regs[0] = 0x00A5;

    slave.slave_addr = SLAVE_ADDR;
    slave.map.holding = holding;
    slave.map.n_holding = 16;
    slave.map.input = input_regs;
    slave.map.n_input = 16;
    slave.map.coils = coils;
    slave.map.n_coils = 32;

    /* read regs -> write a reg -> read it back -> coil write -> coil read */
    printf("--- read holding regs 0..1 ---\n");
    {
        uint8_t p[] = { 0x00, 0x00, 0x00, 0x02 };
        script_frame(&muart, SLAVE_ADDR, 0x03, p, sizeof(p));
        run_cycle(uart, &muart, &slave, &rx);
    }
    printf("--- write holding reg 1 = 0x1234 ---\n");
    {
        uint8_t p[] = { 0x00, 0x01, 0x12, 0x34 };
        script_frame(&muart, SLAVE_ADDR, 0x06, p, sizeof(p));
        run_cycle(uart, &muart, &slave, &rx);
    }
    printf("--- read holding reg 1 ---\n");
    {
        uint8_t p[] = { 0x00, 0x01, 0x00, 0x01 };
        script_frame(&muart, SLAVE_ADDR, 0x03, p, sizeof(p));
        run_cycle(uart, &muart, &slave, &rx);
    }
    printf("--- write single coil 5 = ON ---\n");
    {
        uint8_t p[] = { 0x00, 0x05, 0xFF, 0x00 };
        script_frame(&muart, SLAVE_ADDR, 0x05, p, sizeof(p));
        run_cycle(uart, &muart, &slave, &rx);
    }
    printf("--- read coils 0..9 ---\n");
    {
        uint8_t p[] = { 0x00, 0x00, 0x00, 0x0A };
        script_frame(&muart, SLAVE_ADDR, 0x01, p, sizeof(p));
        run_cycle(uart, &muart, &slave, &rx);
    }

    printf("demo done: holding[1] = 0x%04X, coil 5 = %s\n",
           holding[1], (coils[0] & 0x20u) ? "ON" : "OFF");
    return 0;
}
