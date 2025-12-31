#include <zephyr/ztest.h>
#include <zephyr/drivers/uart.h>
#include "uart_mux.h"
#include "test_helpers.h"

/* captured RX */
static uint8_t rx_buf[64];
static size_t rx_len;

static void test_uart_cb(const struct device *dev,
			 struct uart_event *evt,
			 void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);

	if (evt->type == UART_RX_RDY) {
		memcpy(rx_buf, evt->data.rx.buf, evt->data.rx.len);
		rx_len = evt->data.rx.len;
	}
}
extern void fake_uart_inject_rx(const uint8_t *data, size_t len);

ZTEST(uart_mux_rx, test_valid_frame)
{
	uint8_t frame[] = {
		0xC0,       /* FI */
		0x02,       /* FF */
		0x00, /* LEN LSB, filled later */
		0x00,   /* LEN MSB, filled later */
		0x40,       /* PLC */
		0x41, 0x01, 0x02, /* payload */
		0x00, 0x00  /* CRC (filled below) */
	};

	const struct device *vdev;
	uint32_t frames_ok;
	struct uart_mux_rx_stats stats;
	uint8_t plc_is_first_bye_of_payload = 0;
	uint16_t payload_offset = plc_is_first_bye_of_payload ? 4 : 5;
	uint16_t length_in_header = sizeof(frame) - 5 - 2;

	uart_mux_get_rx_stats(&stats);
	frames_ok = stats.frames_ok;

	rx_len = 0;

	/* Virtual UART device from DTS */
	vdev = DEVICE_DT_GET(DT_NODELABEL(uart_mux_test));
	zassert_true(device_is_ready(vdev), NULL);

	uart_callback_set(vdev, test_uart_cb, NULL);
	uart_rx_enable(vdev, NULL, 0, SYS_FOREVER_MS);
	uint8_t *payload;
	uint16_t len;

	payload = frame + payload_offset;
	len =  sizeof(frame) - payload_offset - 2;

	frame[2] = length_in_header & 0xFF; /* LEN LSB */
	frame[3] = length_in_header >> 8;   /* LEN MSB */

	patch_crc_to_frame(frame, sizeof(frame));
	fake_uart_inject_rx(frame, sizeof(frame));

	zassert_equal(rx_len, len, NULL);
	zassert_mem_equal(rx_buf, payload, len, NULL);

	uart_mux_get_rx_stats(&stats);
	zassert_equal(stats.frames_ok, frames_ok + 1, NULL);
}

ZTEST(uart_mux_rx, test_valid_frame2)
{
	uint8_t frame[] = {
		0xC0,       /* FI */
		0x02,       /* FF */
		0x00, /* LEN LSB, filled later */
		0x00,   /* LEN MSB, filled later */
		0x42,       /* PLC */
		0x01, 0x02, 0x03, /* payload */
		0x00, 0x00  /* CRC (filled below) */
	};

	const struct device *vdev;
	uint32_t frames_ok;
	struct uart_mux_rx_stats stats;
	uint8_t plc_is_first_bye_of_payload = 1;
	uint16_t payload_offset = plc_is_first_bye_of_payload ? 4 : 5;
	uint16_t length_in_header = sizeof(frame) - 5 - 2;

	uart_mux_get_rx_stats(&stats);
	frames_ok = stats.frames_ok;

	rx_len = 0;

	/* Virtual UART device from DTS */
	vdev = DEVICE_DT_GET(DT_NODELABEL(uart_mux_test_plc_in_payload));
	zassert_true(device_is_ready(vdev), NULL);

	uart_callback_set(vdev, test_uart_cb, NULL);
	uart_rx_enable(vdev, NULL, 0, SYS_FOREVER_MS);
	uint8_t *payload;
	uint16_t len;

	payload = frame + payload_offset;
	len =  sizeof(frame) - payload_offset - 2;

	frame[2] = length_in_header & 0xFF; /* LEN LSB */
	frame[3] = length_in_header >> 8;   /* LEN MSB */

	patch_crc_to_frame(frame, sizeof(frame));
	fake_uart_inject_rx(frame, sizeof(frame));

	zassert_equal(rx_len, len, NULL);
	zassert_mem_equal(rx_buf, payload, len, NULL);

	uart_mux_get_rx_stats(&stats);
	zassert_equal(stats.frames_ok, frames_ok + 1, NULL);
}

ZTEST_SUITE(uart_mux_rx, NULL, NULL, NULL, NULL, NULL);

