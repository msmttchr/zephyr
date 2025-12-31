#include <zephyr/ztest.h>
#include <zephyr/drivers/uart.h>

extern void fake_uart_reset_tx(void);
extern int fake_uart_get_tx_count(void);

ZTEST(uart_mux_tx, test_tx_back_to_back)
{
	const struct device *dev;

	dev = DEVICE_DT_GET(DT_NODELABEL(uart_mux_ch_high));
	zassert_true(device_is_ready(dev), NULL);

	fake_uart_reset_tx();

	uint8_t p1[] = { 0x01 };
	uint8_t p2[] = { 0x02 };

	uart_tx(dev, p1, sizeof(p1), SYS_FOREVER_MS);
	uart_tx(dev, p2, sizeof(p2), SYS_FOREVER_MS);

	/* Two frames = 6 TX segments */
	zassert_equal(fake_uart_get_tx_count(), 6, NULL);
}

