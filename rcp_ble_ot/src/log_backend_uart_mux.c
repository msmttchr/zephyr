#include <zephyr/logging/log_backend.h>
#include <zephyr/logging/log_output.h>
#include <zephyr/drivers/uart.h>

#include "uart_mux.h"

/* Define a buffer to format strings into before transmission */
static uint8_t formatting_buffer[256];
static uint8_t uart_formatting_buffer[256];

/* System virtual UART (mux channel) */
static const struct device *sys_uart;

static atomic_t log_tx_busy;
static atomic_t log_dropped_cnt;
volatile int tx_done_cnt = 0;
static void log_tx_done_cb(const uint8_t *buf, size_t len, void *user_data)
{
	ARG_UNUSED(user_data);
	ARG_UNUSED(len);
	if (buf == uart_formatting_buffer) {
		atomic_set(&log_tx_busy, 0);
		tx_done_cnt++;
	}
}

/* This function defines WHERE the characters go (e.g., custom SPI, I2C, or a file) */
static int custom_output_func(uint8_t *data, size_t length, void *ctx)
{
	ARG_UNUSED(ctx);

	if (!sys_uart) {
		return length;
	}

	if (!atomic_cas(&log_tx_busy, 0, 1)) {
		atomic_inc(&log_dropped_cnt);
		return length; /* silently drop */
	}

	memcpy(uart_formatting_buffer, data, length);
	if (uart_tx(sys_uart, uart_formatting_buffer, length, SYS_FOREVER_MS) < 0) {
		atomic_set(&log_tx_busy, 0);
		atomic_inc(&log_dropped_cnt);
	}

	return length;
}

/* Initialize the helper that handles formatting, timestamps, and colors */
LOG_OUTPUT_DEFINE(my_log_output, custom_output_func, formatting_buffer, sizeof(formatting_buffer));

/* The 'process' function is called whenever a new log message is ready */
static void custom_process(const struct log_backend *const backend, union log_msg_generic *msg) {
	uint32_t flags = LOG_OUTPUT_FLAG_LEVEL | LOG_OUTPUT_FLAG_TIMESTAMP | LOG_OUTPUT_FLAG_FORMAT_SYSLOG;
	// The helper converts the internal log message into a readable string
	log_output_msg_process(&my_log_output, &msg->log, flags);
}

static void custom_init(const struct log_backend *const backend) {
	// Hardware initialization for your backend goes here
	sys_uart = DEVICE_DT_GET(DT_NODELABEL(uart_mux_sys));

	if (!device_is_ready(sys_uart)) {
		sys_uart = NULL;
		return;
	}

	uart_mux_register_tx_done_cb(sys_uart,
				     log_tx_done_cb,
				     NULL);
}

static void custom_panic(struct log_backend const *const backend) {
    // Called during a system crash. Flush buffers and switch to polling mode.
    log_output_flush(&my_log_output);
}

static const struct log_backend_api log_backend_uart_mux_api = {
	.process = custom_process,
	.panic = custom_panic,
	.init = custom_init,
};


/* --- Backend instance (autostart) --- */

LOG_BACKEND_DEFINE(log_backend_uart_mux,
		   log_backend_uart_mux_api,
		   true);

