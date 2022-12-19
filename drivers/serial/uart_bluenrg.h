/*
 * Copyright (c) 2016 Open-RnD Sp. z o.o.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Driver for UART port on BLUENRG family processor.
 *
 */

#ifndef ZEPHYR_DRIVERS_SERIAL_UART_BLUENRG_H_
#define ZEPHYR_DRIVERS_SERIAL_UART_BLUENRG_H_

#include <zephyr/drivers/pinctrl.h>

#include <rf_driver_ll_usart.h>

#define BLUENRG_EXTI_LINE_NONE	0xFFFFFFFFU

/* device config */
struct uart_bluenrg_config {
	/* USART instance */
	USART_TypeDef *usart;
	/* clock subsystem driving this peripheral */
	const struct bluenrg_pclken *pclken;
	/* number of clock subsystems */
	size_t pclk_len;
	/* initial hardware flow control, 1 for RTS/CTS */
	bool hw_flow_control;
	/* initial parity, 0 for none, 1 for odd, 2 for even */
	int  parity;
	/* switch to enable single wire / half duplex feature */
	bool single_wire;
	/* enable tx/rx pin swap */
	bool tx_rx_swap;
	/* enable rx pin inversion */
	bool rx_invert;
	/* enable tx pin inversion */
	bool tx_invert;
	const struct pinctrl_dev_config *pcfg;
};

/* driver data */
struct uart_bluenrg_data {
	/* Baud rate */
	uint32_t baud_rate;
	/* clock device */
	const struct device *clock;
	/* Reset controller device configuration */
	const struct reset_dt_spec reset;
};

#endif	/* ZEPHYR_DRIVERS_SERIAL_UART_BLUENRG_H_ */
