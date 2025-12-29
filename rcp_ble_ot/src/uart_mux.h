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

/* ============================= */
/* Protocol constants            */
/* ============================= */

#define UART_MUX_FI              0xC0
#define UART_MUX_FF              0x02

#define UART_MUX_HEADER_LEN      5
#define UART_MUX_CRC_LEN         2
#define UART_MUX_MAX_PAYLOAD_LEN  260  /* choose appropriately */
#define UART_MUX_MAX_FRAME_LEN \
	(1 + 1 + 2 + 1 + UART_MUX_MAX_PAYLOAD_LEN + 2)

typedef void (*uart_mux_tx_done_cb_t)(const uint8_t *buf, size_t len, void *user_data);

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

	uart_mux_tx_done_cb_t tx_done_cb;

	void *tx_done_user_data;
};

enum uart_mux_tx_stage {
	UART_MUX_TX_IDLE,
	UART_MUX_TX_HDR,
	UART_MUX_TX_PAYLOAD,
	UART_MUX_TX_CRC,
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

	/* --- Added fields for staged TX --- */

	/* Prepared frame header (FI, FF, LEN[2], PLC) */
	uint8_t hdr[UART_MUX_HEADER_LEN];

	/* CRC-16-CCITT over header + payload */
	uint16_t crc;
	
	/* poll out char */
	uint8_t c;
};

int uart_mux_register_tx_done_cb(const struct device *dev,
				 uart_mux_tx_done_cb_t cb,
				 void *user_data);


#endif /* UART_MUX_H_ */

