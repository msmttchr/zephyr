/*
 * uart_mux.h
 *
 * Application-level UART multiplexer.
 *
 * This module exposes multiple virtual UART devices that are multiplexed
 * over a single physical UART using a framed protocol.
 *
 * All routing, prioritization, and PLC behavior is configured via devicetree.
 * The mux itself is protocol-agnostic.
 */

#ifndef UART_MUX_H_
#define UART_MUX_H_

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/slist.h>

/*
 * Virtual UART channel instance.
 *
 * One instance corresponds to one devicetree node with compatible "st,uart-mux".
 * The channel does not know protocol semantics; it only follows configuration
 * provided via devicetree (priority, PLC routing, PLC TX behavior).
 */
struct uart_mux_channel {
	/* Zephyr device backing this virtual UART */
	const struct device *dev;

	/* Logical channel identifier (from DTS, informational) */
	uint8_t channel_id;

	/* TX arbitration priority (lower value = higher priority) */
	uint8_t priority;

	/* Fixed PLC value for TX, or -1 if PLC comes from payload */
	int plc_tx;

	/* List of PLC values routed to this channel on RX */
	const uint8_t *plc_rx;
	size_t plc_rx_len;

	/* Registered async UART callback */
	uart_callback_t cb;
	void *cb_data;

	/* Channel registry node */
	sys_snode_t node;

	/* FIFO of pending TX descriptors */
	struct k_fifo tx_fifo;
};

/*
 * TX descriptor.
 *
 * One descriptor represents one logical transmission request issued via
 * uart_tx() on a virtual UART.
 *
 * A single logical TX is serialized into multiple physical uart_tx() calls
 * (header, payload, CRC), but completion is reported only once.
 */
struct uart_mux_tx {
	sys_snode_t node;

	/* Payload buffer provided by virtual UART client */
	const uint8_t *buf;
	size_t len;

	/* PLC value to be inserted into the frame */
	uint8_t plc;
};

#endif /* UART_MUX_H_ */

