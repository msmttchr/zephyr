/* uart_mux.h */

#ifndef UART_MUX_H_
#define UART_MUX_H_

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/slist.h>
#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

enum uart_mux_plc_class {
	UART_MUX_PLC_BLE,
	UART_MUX_PLC_OT,
	UART_MUX_PLC_SYS,
};

/* Virtual UART channel (one per DTS node) */
struct uart_mux_channel {
	const struct device *dev;

	uint8_t channel_id;
	uint8_t priority;     /* lower = higher priority */

	const uint8_t *plc_rx;
	int plc_tx;
	uint8_t plc_rx_len;

	uart_callback_t cb;
	void *cb_data;

	sys_snode_t node;

	/* TX queue for this channel */
	struct k_fifo tx_fifo;
	enum uart_mux_plc_class plc_class;
};

/* Internal TX buffer descriptor */
struct uart_mux_tx {
	sys_snode_t node;
	size_t len;
	uint8_t *buf;
	uint8_t plc;
	bool plc_from_payload;
};

#ifdef __cplusplus
}
#endif

#endif /* UART_MUX_H_ */
