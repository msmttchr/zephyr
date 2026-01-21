#include <stdio.h>
#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include "uart_mux.h"
#include <zephyr/logging/log.h>
#include <zephyr/debug/thread_analyzer.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);
#define DEBUG_STACK_SIZE 1024
#define DEBUG_PRIORITY   K_LOWEST_THREAD_PRIO

/* States for periodic messages */
enum periodic_mode {
	MODE_OFF = 0,
	MODE_STACK_ONLY,
	MODE_UART_ONLY,
	MODE_BOTH,
	MODE_COUNT
};

static char *messages[] = {"OFF", "Thread stack", "UART RX", "Thread stack & UART RX"};
/* Global Control Variables */
static volatile enum periodic_mode current_mode = MODE_OFF;
static volatile uint32_t sleep_time_s = 10;
static struct k_sem input_sem;
static char last_key;

extern int hci_uart_main(void);
/* Retrieve the device pointer for the console */

static const struct device *const console_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
/* Retrieve the device pointer for the UART MUX */
static const struct device *const rcp_dev = DEVICE_DT_GET(DT_CHOSEN(uart_mux));
#define CONSOLE_BR DT_PROP(DT_CHOSEN(zephyr_console), current_speed)
#define RCP_BR     DT_PROP(DT_CHOSEN(uart_mux), current_speed)
static bool console_has_flow_ctrl = DT_PROP_OR(DT_CHOSEN(zephyr_console), hw_flow_control, false);
static bool rcp_has_flow_ctrl = DT_PROP_OR(DT_CHOSEN(uart_mux), hw_flow_control, false);

/* Helper to print UART stats */
void print_uart_stats(void)
{
	struct uart_mux_rx_stats stats;

	uart_mux_get_rx_stats(&stats);
	printk("\n--- UART Mux RX Stats ---\n");
	printk("Frames OK:      %u\n", stats.frames_ok);
	printk("CRC Errors:     %u\n", stats.crc_errors);
	printk("Header Errors:  %u\n", stats.header_errors);
	printk("Length Errors:  %u\n", stats.length_errors);
}

/* UART ISR: Fires when you press a key */
void serial_cb(const struct device *dev, void *user_data)
{
	uint8_t c;

	if (!uart_irq_update(dev)) {
		return;
	}

	while (uart_irq_is_pending(dev)) {
		if (uart_irq_rx_ready(dev)) {
			uart_fifo_read(dev, &c, 1);
			last_key = c;
			k_sem_give(&input_sem);
		}
	}
}

void debug_thread(void *a, void *b, void *c)
{
	LOG_INF("Debug thread started. Press 's', 'u', 'space', or 't'");

	while (1) {
		/* Wait for either a keypress OR the periodic timeout */
		int ret = k_sem_take(&input_sem, K_SECONDS(sleep_time_s));

		if (ret == 0) {
			/* Handle Keypress */
			switch (last_key) {
			case 's':
				thread_analyzer_print(0);
				break;
			case 'u':
				print_uart_stats();
				break;
			case ' ':
				current_mode = (current_mode + 1) % MODE_COUNT;
				printk("\nPeriodic mode: %s\n", messages[current_mode]);
				break;
			case 't':
				printk("\nEnter interval (sec): ");
				/* Simple logic: next few keys would be numbers, */
				/* for brevity we just toggle between 2, 5, 10 */
				sleep_time_s =
					(sleep_time_s == 10) ? 2 : (sleep_time_s == 2 ? 5 : 10);
				printk("%d seconds\n", sleep_time_s);
				break;
			}
		} else {
			/* Handle Periodic Output */
			if (current_mode == MODE_STACK_ONLY || current_mode == MODE_BOTH) {
				thread_analyzer_print(0);
			}
			if (current_mode == MODE_UART_ONLY || current_mode == MODE_BOTH) {
				print_uart_stats();
			}
		}
	}
}

K_THREAD_DEFINE(debug_tid, DEBUG_STACK_SIZE, debug_thread, NULL, NULL, NULL, DEBUG_PRIORITY, 0, 0);

int main(void)
{
	k_sem_init(&input_sem, 0, 1);

	/* Setup UART Interrupts */
	if (!device_is_ready(console_dev)) {
		return 0;
	}
	uart_irq_callback_user_data_set(console_dev, serial_cb, NULL);
	uart_irq_rx_enable(console_dev);

	printf("BLE-Openthread RCP on %s\n", CONFIG_BOARD_TARGET);
	printf("Console on %s (baudrate:%d, hardware flow_control:%s)\n", console_dev->name,
	       CONSOLE_BR, console_has_flow_ctrl ? "RTS/CTS" : "None");
	printf("RCP UART on %s (baudrate:%d, flow_control:%s)\n", rcp_dev->name, RCP_BR,
	       rcp_has_flow_ctrl ? "RTS/CTS" : "None");
	printf("Console commands: [s] Stack, [u] UART, [Space] Cycle Periodic, [t] Cycle "
	       "Interval\n");

	hci_uart_main();
	return 0;
}
