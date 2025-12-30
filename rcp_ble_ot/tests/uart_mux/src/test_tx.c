#include <zephyr/ztest.h>
#include <zephyr/drivers/uart.h>
#include "uart_mux.h"

/* fake uart helpers */
extern void fake_uart_reset_tx(void);
extern int fake_uart_get_tx_count(void);
extern const uint8_t *fake_uart_get_tx_buf(int idx);
extern size_t fake_uart_get_tx_len(int idx);

ZTEST(uart_mux_tx, test_single_frame_tx)
{
	const struct device *vdev;

	vdev = DEVICE_DT_GET(DT_NODELABEL(uart_mux_test));
	zassert_true(device_is_ready(vdev), NULL);

	fake_uart_reset_tx();

	uint8_t payload[] = { 0x10, 0x20, 0x30 };

	int ret = uart_tx(vdev, payload, sizeof(payload), SYS_FOREVER_MS);
	zassert_equal(ret, 0, NULL);

	/* TX is synchronous in fake uart */
	zassert_equal(fake_uart_get_tx_count(), 3, NULL);

	/* 0: header */
	zassert_equal(fake_uart_get_tx_len(0), 5, NULL);

	/* 1: payload */
	zassert_equal(fake_uart_get_tx_len(1), sizeof(payload), NULL);
	zassert_mem_equal(fake_uart_get_tx_buf(1), payload, sizeof(payload), NULL);

	/* 2: CRC */
	zassert_equal(fake_uart_get_tx_len(2), 2, NULL);
}

ZTEST_SUITE(uart_mux_tx, NULL, NULL, NULL, NULL, NULL);

