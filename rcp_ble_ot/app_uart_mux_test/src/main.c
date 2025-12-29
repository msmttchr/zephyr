#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_backend.h>

LOG_MODULE_REGISTER(uart_mux_test, LOG_LEVEL_INF);

/* ===================== Devices ===================== */

static const struct device *uart_ble =
	DEVICE_DT_GET(DT_NODELABEL(uart_mux_ble));
static const struct device *uart_ot =
	DEVICE_DT_GET(DT_NODELABEL(uart_mux_ot));
static const struct device *uart_sys =
	DEVICE_DT_GET(DT_NODELABEL(uart_mux_sys));
/* ===================== LED ===================== */

#define LED_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led =
	GPIO_DT_SPEC_GET(LED_NODE, gpios);

/* ===================== RX buffers ===================== */

static uint8_t ble_rx_buf[64];
static uint8_t ot_rx_buf[128];
static uint8_t sys_rx_buf[64];

/* ===================== UART callbacks ===================== */

static void ble_uart_cb(const struct device *dev,
			struct uart_event *evt,
			void *user_data)
{
	if (evt->type == UART_RX_RDY) {
		/* Respond with fixed "event" */
		uint8_t rsp[] = { evt->data.rx.buf[0], 0x0E, 0x00 };
		uart_tx(dev, rsp, sizeof(rsp), SYS_FOREVER_MS);
	}
}

static void ot_uart_cb(const struct device *dev,
		       struct uart_event *evt,
		       void *user_data)
{
	if (evt->type == UART_RX_RDY) {
		/* Echo payload back */
		uart_tx(dev,
			evt->data.rx.buf,
			evt->data.rx.len,
			SYS_FOREVER_MS);
	}
}

static void sys_uart_cb(const struct device *dev,
			struct uart_event *evt,
			void *user_data)
{
	if (evt->type == UART_RX_RDY) {
		gpio_pin_toggle_dt(&led);
	}
}

/* ===================== Threads ===================== */

void ble_thread(void)
{
	uint8_t evt[] = { 0x01, 0x0E, 0x01, 0x00 };

	while (1) {
		uart_tx(uart_ble, evt, sizeof(evt), SYS_FOREVER_MS);
		k_sleep(K_MSEC(CONFIG_TEST_BLE_EVENT_PERIOD_MS));
	}
}

void ot_thread(void)
{
	uint8_t frame[] = { 0x7E, 0xAA, 0x55, 0x00 };

	while (1) {
		frame[3]++;
		uart_tx(uart_ot, frame, sizeof(frame), SYS_FOREVER_MS);
		k_sleep(K_MSEC(CONFIG_TEST_OT_EVENT_PERIOD_MS));
	}
}

void sys_thread(void)
{
	char msg[32];

	while (1) {
		int len = snprintk(msg, sizeof(msg),
				   "SYS %u\n",
				   k_uptime_get_32());
		uart_tx(uart_sys, msg, len, SYS_FOREVER_MS);
		k_sleep(K_MSEC(CONFIG_TEST_SYS_PERIOD_MS));
	}
}

/* ===================== Thread definitions ===================== */
#if 0
K_THREAD_DEFINE(ble_tid, 1024, ble_thread, NULL, NULL, NULL,
		5, 0, 0);

K_THREAD_DEFINE(ot_tid, 1024, ot_thread, NULL, NULL, NULL,
		5, 0, 0);

K_THREAD_DEFINE(sys_tid, 1024, sys_thread, NULL, NULL, NULL,
		7, 0, 0);
#endif
/* ===================== Main ===================== */

int main(void)
{
	__ASSERT(device_is_ready(uart_ble), "BLE UART not ready");
	__ASSERT(device_is_ready(uart_ot), "OT UART not ready");
	__ASSERT(device_is_ready(uart_sys), "SYS UART not ready");

	gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);

	printk("UART mux test started\r\n");
	int count = 0;
	while (1) {
		printk("Loop %d\r\n", ++count);
		k_sleep(K_MSEC(1000));
	}
	uart_callback_set(uart_ble, ble_uart_cb, NULL);
	uart_callback_set(uart_ot, ot_uart_cb, NULL);
	uart_callback_set(uart_sys, sys_uart_cb, NULL);

	uart_rx_enable(uart_ble, ble_rx_buf, sizeof(ble_rx_buf), 100);
	uart_rx_enable(uart_ot, ot_rx_buf, sizeof(ot_rx_buf), 100);
	uart_rx_enable(uart_sys, sys_rx_buf, sizeof(sys_rx_buf), 100);

	printk("UART mux test started\n");

	return 0;
}

