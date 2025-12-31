#include <zephyr/ztest.h>
#include <zephyr/drivers/uart.h>
#include "test_helpers.h"

extern void fake_uart_reset_tx(void);

static void cb(const struct device *dev,
			 struct uart_event *evt,
			 void *user_data)
{
	size_t *rx_count = user_data;

	if (evt->type == UART_RX_RDY) {
		(*rx_count)++;
	}
}

ZTEST(uart_mux_stress, test_rx_tx_stress)
{
	const struct device *dev;
	size_t rx_count;
	uint8_t tx_buf;
	uint8_t rx_buf;

	dev = DEVICE_DT_GET(DT_NODELABEL(uart_mux_ch_high));
	zassert_true(device_is_ready(dev), NULL);

	rx_count = 0;
	uart_callback_set(dev, cb, &rx_count);

	uart_rx_enable(dev, NULL, 0, SYS_FOREVER_MS);

	for (int i = 0; i < 100; i++) {
		tx_buf = (uint8_t)i;
		rx_buf = (uint8_t)i;

		/* TX */
		uart_tx(dev, &tx_buf, 1, SYS_FOREVER_MS);

		/* RX */
		inject_valid_frame(0x10, &rx_buf, 1);
	}

	zassert_equal(rx_count, 100, NULL);
}
ZTEST_SUITE(uart_mux_stress,
	    NULL,  /* suite setup */
	    NULL,  /* suite teardown */
	    NULL,  /* test setup */
	    NULL,  /* test teardown */
	    NULL); /* fixture */

