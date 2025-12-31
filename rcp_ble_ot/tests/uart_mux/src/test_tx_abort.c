#include <zephyr/ztest.h>
#include <zephyr/drivers/uart.h>

extern void fake_uart_reset_tx(void);
extern void fake_uart_inject_tx_abort(void);
extern int fake_uart_get_tx_count(void);
ZTEST(uart_mux_tx, test_tx_abort_recovery)
{
	const struct device *dev;

	dev = DEVICE_DT_GET(DT_NODELABEL(uart_mux_ch_high));
	zassert_true(device_is_ready(dev), NULL);

	fake_uart_reset_tx();

	uint8_t p[] = { 0xAA };

	uart_tx(dev, p, sizeof(p), SYS_FOREVER_MS);

	/* Abort during header */
	fake_uart_inject_tx_abort();

	/* Retry */
	uart_tx(dev, p, sizeof(p), SYS_FOREVER_MS);

	zassert_true(fake_uart_get_tx_count() >= 3, NULL);
}

