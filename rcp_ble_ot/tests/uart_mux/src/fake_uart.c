#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>

static uart_callback_t fake_cb;
static void *fake_cb_data;

/* RX buffer provided by mux */
static uint8_t *rx_buf;
static size_t rx_buf_len;

/* TX observation */
static const uint8_t *last_tx_buf;
static size_t last_tx_len;
static int tx_count;
static const uint8_t *tx_bufs[4];
static size_t tx_lens[4];

#define DT_DRV_COMPAT zephyr_fake_uart
/* --- helpers for tests --- */

void fake_uart_inject_rx(const uint8_t *data, size_t len)
{
	struct uart_event evt = { 0 };

	for (size_t i = 0; i < len; i++) {
		rx_buf[i] = data[i];
	}

	evt.type = UART_RX_RDY;
	evt.data.rx.buf = rx_buf;
	evt.data.rx.len = len;
	evt.data.rx.offset = 0;

	fake_cb(NULL, &evt, fake_cb_data);
}

const uint8_t *fake_uart_last_tx_buf(void)
{
	return last_tx_buf;
}

size_t fake_uart_last_tx_len(void)
{
	return last_tx_len;
}

static int fake_uart_callback_set(const struct device *dev,
				  uart_callback_t cb,
				  void *user_data)
{
	ARG_UNUSED(dev);
	fake_cb = cb;
	fake_cb_data = user_data;
	return 0;
}

static int fake_uart_rx_enable(const struct device *dev,
			       uint8_t *buf,
			       size_t len,
			       int32_t timeout)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(timeout);

	rx_buf = buf;
	rx_buf_len = len;
	return 0;
}

static int fake_uart_rx_disable(const struct device *dev)
{
	ARG_UNUSED(dev);
	rx_buf = NULL;
	rx_buf_len = 0;
	return 0;
}

void fake_uart_reset_tx(void)
{
	tx_count = 0;
}

int fake_uart_get_tx_count(void)
{
	return tx_count;
}

const uint8_t *fake_uart_get_tx_buf(int idx)
{
	return tx_bufs[idx];
}

size_t fake_uart_get_tx_len(int idx)
{
	return tx_lens[idx];
}

static int fake_uart_tx(const struct device *dev,
			const uint8_t *buf,
			size_t len,
			int32_t timeout)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(timeout);

	tx_bufs[tx_count] = buf;
	tx_lens[tx_count] = len;
	tx_count++;

	struct uart_event evt = {
		.type = UART_TX_DONE,
		.data.tx.buf = buf,
		.data.tx.len = len,
	};

	fake_cb(NULL, &evt, fake_cb_data);
	return 0;
}

static const struct uart_driver_api fake_uart_api = {
	.callback_set = fake_uart_callback_set,
	.rx_enable   = fake_uart_rx_enable,
	.rx_disable  = fake_uart_rx_disable,
	.tx          = fake_uart_tx,
};

DEVICE_DT_DEFINE(DT_NODELABEL(fake_uart),
		 NULL,
		 NULL,
		 NULL,
		 NULL,
		 POST_KERNEL,
		 CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		 &fake_uart_api);


