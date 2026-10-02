#include "mock_uart.h"

#include <errno.h>
#include <string.h>

static int mock_read(void *ctx, uint8_t *buf, size_t cap, uint32_t timeout_ms)
{
    mock_uart_t *m = (mock_uart_t *)ctx;
    (void)timeout_ms; /* the fake clock never blocks: data is either there or not */

    if (m->fail_read) {
        m->fail_read = false;
        return -EIO;
    }
    size_t avail = m->rx_len - m->rx_pos;
    size_t n = avail < cap ? avail : cap;
    memcpy(buf, &m->rx[m->rx_pos], n);
    m->rx_pos += n;
    return (int)n;
}

static int mock_write(void *ctx, const uint8_t *buf, size_t len)
{
    mock_uart_t *m = (mock_uart_t *)ctx;

    if (m->tx_len + len > MOCK_UART_BUF)
        return -ENOSPC;
    memcpy(&m->tx[m->tx_len], buf, len);
    m->tx_len += len;
    return 0;
}

static uint32_t mock_now(void *ctx)
{
    return ((mock_uart_t *)ctx)->now;
}

void mock_uart_init(mock_uart_t *m)
{
    memset(m, 0, sizeof(*m));
    m->uart.read = mock_read;
    m->uart.write = mock_write;
    m->uart.now_ms = mock_now;
    m->uart.ctx = m;
}

void mock_uart_feed_rx(mock_uart_t *m, const uint8_t *data, size_t len)
{
    if (m->rx_len + len <= MOCK_UART_BUF) {
        memcpy(&m->rx[m->rx_len], data, len);
        m->rx_len += len;
    }
}

void mock_uart_advance_ms(mock_uart_t *m, uint32_t ms)
{
    m->now += ms;
}

const uint8_t *mock_uart_tx_data(const mock_uart_t *m)
{
    return m->tx;
}

size_t mock_uart_tx_len(const mock_uart_t *m)
{
    return m->tx_len;
}

void mock_uart_tx_clear(mock_uart_t *m)
{
    m->tx_len = 0;
}

void mock_uart_fail_next_read(mock_uart_t *m)
{
    m->fail_read = true;
}

const hal_uart_t *mock_uart_bus(mock_uart_t *m)
{
    return &m->uart;
}
