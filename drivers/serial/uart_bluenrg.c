/*
 * Copyright (c) 2016 Open-RnD Sp. z o.o.
 * Copyright (c) 2016 Linaro Limited.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT st_bluenrg_uart

/**
 * @brief Driver for UART port on BlueNRG family processor.
 * @note  LPUART and U(S)ART have the same base and
 *        majority of operations are performed the same way.
 *        Please validate for newly added series.
 */
#include <zephyr/sys/printk.h>


#include <zephyr/kernel.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/sys/__assert.h>
#include <soc.h>
#include <zephyr/init.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/reset.h>

#include <zephyr/linker/sections.h>
//#include <zephyr/drivers/clock_control/bluenrg_clock_control.h>
#include "uart_bluenrg.h"
#include "bluenrg_lpx.h"

#include <rf_driver_ll_usart.h>
#include <rf_driver_ll_lpuart.h>
#include <rf_driver_ll_system.h>
#include <rf_driver_ll_bus.h>
#include <rf_driver_ll_rcc.h>

#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
LOG_MODULE_REGISTER(uart_bluenrg, CONFIG_UART_LOG_LEVEL);

static int uart_bluenrg_poll_in(const struct device *dev, unsigned char *c)
{
	const struct uart_bluenrg_config *config = dev->config;

	/* Clear overrun error flag */
	if (LL_USART_IsActiveFlag_ORE(config->usart)) {
		LL_USART_ClearFlag_ORE(config->usart);
	}

	/*
	 * On bluenrg F4X, F1X, and F2X, the RXNE flag is affected (cleared) by
	 * the uart_err_check function call (on errors flags clearing)
	 */
	if (!LL_USART_IsActiveFlag_RXNE(config->usart)) {
		return -1;
	}

	*c = (unsigned char)LL_USART_ReceiveData8(config->usart);

	return 0;
}



static void uart_bluenrg_poll_out(const struct device *dev,
					unsigned char c)
{
	const struct uart_bluenrg_config *config = dev->config;

	unsigned int key;

	/* Wait for TXE flag to be raised
	 * When TXE flag is raised, we lock interrupts to prevent interrupts (notably that of usart)
	 * or thread switch. Then, we can safely send our character. The character sent will be
	 * interlaced with the characters potentially send with interrupt transmission API
	 */
	while (1) {
		if (LL_USART_IsActiveFlag_TXE(config->usart)) {
			key = irq_lock();
			if (LL_USART_IsActiveFlag_TXE(config->usart)) {
				break;
			}
			irq_unlock(key);
		}
	}

	LL_USART_TransmitData8(config->usart, (uint8_t)c);
	irq_unlock(key);
}



static int uart_bluenrg_err_check(const struct device *dev)
{
	const struct uart_bluenrg_config *config = dev->config;
	uint32_t err = 0U;

	/* Check for errors, then clear them.
	 * Some SoC clear all error flags when at least
	 * one is cleared. (e.g. F4X, F1X, and F2X).
	 * The bluenrg F4X, F1X, and F2X also reads the usart DR when clearing Errors
	 */
	if (LL_USART_IsActiveFlag_ORE(config->usart)) {
		err |= UART_ERROR_OVERRUN;
	}

	if (LL_USART_IsActiveFlag_PE(config->usart)) {
		err |= UART_ERROR_PARITY;
	}

	if (LL_USART_IsActiveFlag_FE(config->usart)) {
		err |= UART_ERROR_FRAMING;
	}


	/* Clearing error :
	 * the bluenrg F4X, F1X, and F2X sw sequence is reading the usart SR
	 * then the usart DR to clear the Error flags ORE, PE, FE, NE
	 * --> so is the RXNE flag also cleared !
	 */
	if (err & UART_ERROR_OVERRUN) {
		LL_USART_ClearFlag_ORE(config->usart);
	}

	if (err & UART_ERROR_PARITY) {
		LL_USART_ClearFlag_PE(config->usart);
	}

	if (err & UART_ERROR_FRAMING) {
		LL_USART_ClearFlag_FE(config->usart);
	}
	/* Clear noise error as well,
	 * it is not represented by the errors enum
	 */
	LL_USART_ClearFlag_NE(config->usart);

	return err;
}


static const struct uart_driver_api uart_bluenrg_driver_api = {
	.poll_in = uart_bluenrg_poll_in,
	.poll_out = uart_bluenrg_poll_out,
	.err_check = uart_bluenrg_err_check,
};


/**
 * @brief Initialize UART channel
 *
 * This routine is called to reset the chip in a quiescent state.
 * It is assumed that this function is called only once per UART.
 *
 * @param dev UART device struct
 *
 * @return 0
 */
static int uart_bluenrg_init(const struct device *dev)
{
	const struct uart_bluenrg_config *config = dev->config;
	struct uart_bluenrg_data *data = dev->data;
	uint32_t ll_parity;
	uint32_t ll_datawidth;
	int err;

	//__uart_bluenrg_get_clock(dev);

	// if (!device_is_ready(data->clock)) {
	// 	LOG_ERR("clock control device not ready");
	// 	return -ENODEV;
	// }

	/* enable clock */
	LL_RCC_HSE_SetCapacitorTuning(32);
	LL_RCC_HSE_SetCurrentControl((0x2U << (4UL) | 0x1U << (4UL))); // LL_RCC_HSE_CURRENTMAX_3 : (RCC_RFSWHSECR_GMC_1| RCC_RFSWHSECR_GMC_0) : (0x2U << (4UL) | 0x1U << (4UL))
	//SystemTimer_TimeoutConfig(32000000, 100, TRUE);
	LL_RCC_HSE_Enable();
	LL_APB1_EnableClock(0x400UL);

	// err = clock_control_on(data->clock, (clock_control_subsys_t)&config->pclken[0]);
	// if (err != 0) {
	// 	LOG_ERR("Could not enable (LP)UART clock");
	// 	return err;
	// }

	// if (IS_ENABLED(BLUENRG_UART_DOMAIN_CLOCK_SUPPORT) && (config->pclk_len > 1)) {
	// 	err = clock_control_configure(DEVICE_DT_GET(BLUENRG_CLOCK_CONTROL_NODE),
	// 				      (clock_control_subsys_t) &config->pclken[1],
	// 				      NULL);
	// 	if (err != 0) {
	// 		LOG_ERR("Could not select UART domain clock");
	// 		return err;
	// 	}
	// }

	/* Configure dt provided device signals when available */
	// err = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	// if (err < 0) {
	// 	return err;
	// }

	LL_USART_Disable(config->usart);

	// if (!device_is_ready(data->reset.dev)) {
	// 	LOG_ERR("reset controller not ready");
	// 	return -ENODEV;
	// }

	/* Reset UART to default state using RCC */
	//reset_line_toggle_dt(&data->reset);

	/* TX/RX direction */
	LL_USART_SetTransferDirection(config->usart,
				      LL_USART_DIRECTION_TX_RX);

	/* Determine the datawidth and parity. If we use other parity than
	 * 'none' we must use datawidth = 9 (to get 8 databit + 1 parity bit).
	 */
	if (config->parity == 2) {
		/* 8 databit, 1 parity bit, parity even */
		ll_parity = LL_USART_PARITY_EVEN;
		ll_datawidth = LL_USART_DATAWIDTH_9B;
	} else if (config->parity == 1) {
		/* 8 databit, 1 parity bit, parity odd */
		ll_parity = LL_USART_PARITY_ODD;
		ll_datawidth = LL_USART_DATAWIDTH_9B;
	} else {  /* Default to 8N0, but show warning if invalid value */
		if (config->parity != 0) {
			LOG_WRN("Invalid parity setting '%d'."
				"Defaulting to 'none'.", config->parity);
		}
		/* 8 databit, parity none */
		ll_parity = LL_USART_PARITY_NONE;
		ll_datawidth = LL_USART_DATAWIDTH_8B;
	}

	/* Set datawidth and parity, 1 start bit, 1 stop bit  */
	LL_USART_ConfigCharacter(config->usart,
				 ll_datawidth,
				 ll_parity,
				 LL_USART_STOPBITS_1);

	if (config->hw_flow_control) {
		uart_bluenrg_set_hwctrl(dev, LL_USART_HWCONTROL_RTS_CTS);
	}

	/* Set the default baudrate */
	uart_bluenrg_set_baudrate(dev, data->baud_rate);

	/* Enable the single wire / half-duplex mode */
	if (config->single_wire) {
		LL_USART_EnableHalfDuplex(config->usart);
	}

	LL_USART_Enable(config->usart);
}


#define BLUENRG_UART_INIT(index)						\
BLUENRG_UART_IRQ_HANDLER_DECL(index)					\
									\
PINCTRL_DT_INST_DEFINE(index);						\
									\
static const struct bluenrg_pclken pclken_##index[] =			\
					    BLUENRG_DT_INST_CLOCKS(index);\
									\
static const struct uart_bluenrg_config uart_bluenrg_cfg_##index = {	\
	.usart = (USART_TypeDef *)DT_INST_REG_ADDR(index),		\
	.pclken = pclken_##index,					\
	.pclk_len = DT_INST_NUM_CLOCKS(index),				\
	.hw_flow_control = DT_INST_PROP(index, hw_flow_control),	\
	.parity = DT_INST_ENUM_IDX_OR(index, parity, UART_CFG_PARITY_NONE),	\
	.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(index),			\
	.single_wire = DT_INST_PROP_OR(index, single_wire, false),	\
	.tx_rx_swap = DT_INST_PROP_OR(index, tx_rx_swap, false),	\
	.rx_invert = DT_INST_PROP(index, rx_invert),			\
	.tx_invert = DT_INST_PROP(index, tx_invert),			\
	BLUENRG_UART_IRQ_HANDLER_FUNC(index)				\
	BLUENRG_UART_PM_WAKEUP(index)					\
};									\
									\
static struct uart_bluenrg_data uart_bluenrg_data_##index = {		\
	.baud_rate = DT_INST_PROP(index, current_speed),		\
	.reset = RESET_DT_SPEC_GET(DT_DRV_INST(index)),			\
	UART_DMA_CHANNEL(index, rx, RX, PERIPHERAL, MEMORY)		\
	UART_DMA_CHANNEL(index, tx, TX, MEMORY, PERIPHERAL)		\
};									\
									\
PM_DEVICE_DT_INST_DEFINE(index, uart_bluenrg_pm_action);		        \
									\
DEVICE_DT_INST_DEFINE(index,						\
		    &uart_bluenrg_init,					\
		    PM_DEVICE_DT_INST_GET(index),			\
		    &uart_bluenrg_data_##index, &uart_bluenrg_cfg_##index,	\
		    PRE_KERNEL_1, CONFIG_SERIAL_INIT_PRIORITY,		\
		    &uart_bluenrg_driver_api);				\
									\
BLUENRG_UART_IRQ_HANDLER(index)

DT_INST_FOREACH_STATUS_OKAY(BLUENRG_UART_INIT)
