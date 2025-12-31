#include <zephyr/ztest.h>
#include <zephyr/drivers/uart.h>

extern void fake_uart_reset_tx(void);
extern int fake_uart_get_tx_count(void);
extern const uint8_t *fake_uart_get_tx_buf(int idx);


ZTEST(uart_mux_tx, test_tx_priority_arbitration)
{
	const struct device *high;
	const struct device *low;

	high = DEVICE_DT_GET(DT_NODELABEL(uart_mux_ch_high));
	low  = DEVICE_DT_GET(DT_NODELABEL(uart_mux_ch_low));

	zassert_true(device_is_ready(high), NULL);
	zassert_true(device_is_ready(low), NULL);

	fake_uart_reset_tx();

	uint8_t p1[] = { 0x01 };
	uint8_t p2[] = { 0x02 };

	/* Enqueue low first */
	uart_tx(low, p2, sizeof(p2), SYS_FOREVER_MS);
	uart_tx(high, p1, sizeof(p1), SYS_FOREVER_MS);

	/* fake UART executes immediately */
	zassert_equal(fake_uart_get_tx_count(), 6, NULL);

	/* First header belongs to high priority channel */
	const uint8_t *hdr = fake_uart_get_tx_buf(0);
	zassert_equal(hdr[4], 0x10, "High priority PLC expected");
}

