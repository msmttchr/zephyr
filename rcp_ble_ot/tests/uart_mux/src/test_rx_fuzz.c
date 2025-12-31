#include <zephyr/ztest.h>
#include "uart_mux.h"
#include "test_helpers.h"

static size_t rx_count;

static void cb(const struct device *dev,
	       struct uart_event *evt,
	       void *ud)
{
	if (evt->type == UART_RX_RDY) {
		rx_count++;
	}
}

ZTEST(uart_mux_rx, test_rx_fuzz_resync)
{
	const struct device *dev;

	dev = DEVICE_DT_GET(DT_NODELABEL(uart_mux_ch_high));
	zassert_true(device_is_ready(dev), NULL);

	uart_callback_set(dev, cb, NULL);
	uart_rx_enable(dev, NULL, 0, SYS_FOREVER_MS);

	rx_count = 0;

	/* Garbage storm */
	for (int i = 0; i < 50; i++) {
		inject_garbage(7);
	}

	zassert_equal(rx_count, 0, "Garbage produced RX!");

	/* Valid frame must recover */
	uint8_t p[] = { 0xAA };
	inject_valid_frame(0x10, p, sizeof(p));

	zassert_equal(rx_count, 1, "Did not resync on valid frame");
}

