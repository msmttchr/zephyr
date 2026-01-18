/* Copyright (c) 2023 Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include "uart_mux.h"
#include <zephyr/logging/log.h>
#include <zephyr/debug/thread_analyzer.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);
#define DEBUG_STACK_SIZE 1024
#define DEBUG_PRIORITY K_LOWEST_THREAD_PRIO

extern int hci_uart_main(void);
/* Retrieve the device pointer for the console */
static const struct device *const console_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
/* Retrieve the device pointer for the console */
static const struct device *const rcp_dev = DEVICE_DT_GET(DT_CHOSEN(uart_mux));
#define CONSOLE_BR DT_PROP(DT_CHOSEN(zephyr_console), current_speed)
#define RCP_BR DT_PROP(DT_CHOSEN(uart_mux), current_speed)
static bool console_has_flow_ctrl = DT_PROP_OR(DT_CHOSEN(zephyr_console), hw_flow_control, false);
static bool rcp_has_flow_ctrl = DT_PROP_OR(DT_CHOSEN(uart_mux), hw_flow_control, false);

void debug_thread(void *a, void *b, void *c)
{
    LOG_INF("Debug thread started");
    while (1) {
	struct uart_mux_rx_stats stats;

	/* Sleep most of the time */
	k_sleep(K_SECONDS(10));
	uart_mux_get_rx_stats(&stats);
	LOG_INF("UART Mux RX Stats:");
	LOG_INF("  Frames OK:     %u", stats.frames_ok);
	LOG_INF("  CRC Errors:    %u", stats.crc_errors);
	LOG_INF("  Header Errors: %u", stats.header_errors);
	LOG_INF("  Length Errors: %u", stats.length_errors);
	LOG_INF("  No Channel:    %u", stats.no_channel);
	thread_analyzer_print(0);
    }
}

K_THREAD_DEFINE(debug_tid,
                DEBUG_STACK_SIZE,
                debug_thread,
                NULL, NULL, NULL,
                DEBUG_PRIORITY,
                0,
                0);

int main(void)
{
	printf("BLE-Openthread RCP on %s\n", CONFIG_BOARD_TARGET);
	printf("Console on %s (baudrate:%d, hardware flow_control:%s)\n",
		console_dev->name,
		CONSOLE_BR,
		console_has_flow_ctrl ? "RTS/CTS" : "None");
	printf("RCP UART on %s (baudrate:%d, flow_control:%s)\n",
		rcp_dev->name,
		RCP_BR,
		rcp_has_flow_ctrl ? "RTS/CTS" : "None");
	hci_uart_main();
	return 0;
}
