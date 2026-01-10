/* Copyright (c) 2023 Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>

extern int hci_uart_main(void);
/* Retrieve the device pointer for the console */
static const struct device *const console_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
/* Retrieve the device pointer for the console */
static const struct device *const rcp_dev = DEVICE_DT_GET(DT_CHOSEN(uart_mux));
#define CONSOLE_BR DT_PROP(DT_CHOSEN(zephyr_console), current_speed)
#define RCP_BR DT_PROP(DT_CHOSEN(uart_mux), current_speed)
static bool console_has_flow_ctrl = DT_PROP_OR(DT_CHOSEN(zephyr_console), hw_flow_control, false);
static bool rcp_has_flow_ctrl = DT_PROP_OR(DT_CHOSEN(uart_mux), hw_flow_control, false);

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
