#include <zephyr/ztest.h>
#include <zephyr/drivers/uart.h>
#include "test_helpers.h"

extern void fake_uart_inject_rx(const uint8_t *data, size_t len);

static void cb(const struct device *dev,
			 struct uart_event *evt,
			 void *user_data)
{
	size_t *rx_count = user_data;

	if (evt->type == UART_RX_RDY) {
		(*rx_count)++;
	}
}

ZTEST(uart_mux_rx, test_rx_timeout_resets_state)
{
	const struct device *dev;
	size_t rx_count = 0;

	dev = DEVICE_DT_GET(DT_NODELABEL(uart_mux_ch_high));
	zassert_true(device_is_ready(dev), NULL);

	uart_callback_set(dev, cb, &rx_count);

	uart_rx_enable(dev, NULL, 0, SYS_FOREVER_MS);

	/* Send incomplete header */
	uint8_t partial[] = { 0xC0, 0x02 };
	fake_uart_inject_rx(partial, sizeof(partial));

	/* Wait past timeout */
	k_sleep(K_MSEC(CONFIG_UART_MUX_RX_TIMEOUT_MS + 10));

	/* Now valid frame must be accepted */
	uint8_t p[] = { 0x55 };
	inject_valid_frame(0x10, p, sizeof(p));

	zassert_equal(rx_count, 1, NULL);
}

