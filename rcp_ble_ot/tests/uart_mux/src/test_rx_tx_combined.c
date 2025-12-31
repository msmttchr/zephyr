#include <zephyr/ztest.h>
#include <zephyr/drivers/uart.h>
#include "uart_mux.h"
#include "test_helpers.h"

extern void fake_uart_inject_rx(const uint8_t *data, size_t len);
extern void fake_uart_reset_tx(void);

static uint8_t rx_buf[16];
static size_t rx_len;

static void cb(const struct device *dev,
	       struct uart_event *evt,
	       void *ud)
{
	if (evt->type == UART_RX_RDY) {
		memcpy(rx_buf, evt->data.rx.buf, evt->data.rx.len);
		rx_len = evt->data.rx.len;
	}
}

ZTEST(uart_mux_combined, test_rx_tx_parallel)
{
	const struct device *dev;

	dev = DEVICE_DT_GET(DT_NODELABEL(uart_mux_ch_high));
	zassert_true(device_is_ready(dev), NULL);

	uart_callback_set(dev, cb, NULL);
	uart_rx_enable(dev, NULL, 0, SYS_FOREVER_MS);

	fake_uart_reset_tx();
	rx_len = 0;

	uint8_t payload[] = { 0x55 };

	/* TX */
	uart_tx(dev, payload, sizeof(payload), SYS_FOREVER_MS);

	/* RX frame */
	uint8_t frame[] = {
		0xC0, 0x02, 0x01, 0x00, 0x10,
		0x55, 0x00, 0x00
	};
	patch_crc_to_frame(frame, sizeof(frame));
	fake_uart_inject_rx(frame, sizeof(frame));

	zassert_equal(rx_len, 1, NULL);
	zassert_equal(rx_buf[0], 0x55, NULL);
}
ZTEST_SUITE(uart_mux_combined, NULL, NULL, NULL, NULL, NULL);
