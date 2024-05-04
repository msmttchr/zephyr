/*
 * Copyright (c) 2016 Open-RnD Sp. z o.o.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_GPIO_GPIO_BLUENRG_H_
#define ZEPHYR_DRIVERS_GPIO_GPIO_BLUENRG_H_

/**
 * @file header for STM32 GPIO
 */

#include <zephyr/drivers/clock_control/bluenrg_clock_control.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/dt-bindings/pinctrl/stm32-pinctrl.h>

/* GPIO buses definitions */

#define STM32_PORT_NOT_AVAILABLE 0xFFFFFFFF

#define STM32_CLOCK_BUS_GPIO BLUENRG_CLOCK_BUS_AHB0
#define STM32_PERIPH_GPIOA LL_AHB_PERIPH_GPIOA
#define STM32_PERIPH_GPIOB LL_AHB_PERIPH_GPIOB

#define STM32_PINCFG_MODE_OUTPUT        STM32_MODER_OUTPUT_MODE
#define STM32_PINCFG_MODE_INPUT         STM32_MODER_INPUT_MODE
#define STM32_PINCFG_MODE_ANALOG        STM32_MODER_ANALOG_MODE
#define STM32_PINCFG_PUSH_PULL          STM32_OTYPER_PUSH_PULL
#define STM32_PINCFG_OPEN_DRAIN         STM32_OTYPER_OPEN_DRAIN
#define STM32_PINCFG_PULL_UP            STM32_PUPDR_PULL_UP
#define STM32_PINCFG_PULL_DOWN          STM32_PUPDR_PULL_DOWN
#define STM32_PINCFG_FLOATING           STM32_PUPDR_NO_PULL

#if defined(CONFIG_GPIO_GET_CONFIG)
/**
 * @brief structure of a GPIO pin (stm32 LL values) use to get the configuration
 */
struct gpio_stm32_pin {
	unsigned int type; /* LL_GPIO_OUTPUT_PUSHPULL or LL_GPIO_OUTPUT_OPENDRAIN */
	unsigned int pupd; /* LL_GPIO_PULL_NO or LL_GPIO_PULL_UP or LL_GPIO_PULL_DOWN */
	unsigned int mode; /* LL_GPIO_MODE_INPUT or LL_GPIO_MODE_OUTPUT or other */
	unsigned int out_state; /* 1 (high level) or 0 (low level) */
};
#endif /* CONFIG_GPIO_GET_CONFIG */

/**
 * @brief configuration of GPIO device
 */
struct gpio_stm32_config {
	/* gpio_driver_config needs to be first */
	struct gpio_driver_config common;
	/* port base address */
	uint32_t *base;
	/* IO port */
	int port;
	struct stm32_pclken pclken;
};

/**
 * @brief driver data
 */
struct gpio_stm32_data {
	/* gpio_driver_data needs to be first */
	struct gpio_driver_data common;
	/* device's owner of this data */
	const struct device *dev;
	/* user ISR cb */
	sys_slist_t cb;
};

/**
 * @brief helper for configuration of GPIO pin
 *
 * @param dev GPIO port device pointer
 * @param pin IO pin
 * @param conf GPIO mode
 * @param func Pin function
 *
 * @return 0 on success, negative errno code on failure
 */
int gpio_bluenrg_configure(const struct device *dev, int pin, int conf, int func);

#endif /* ZEPHYR_DRIVERS_GPIO_GPIO_BLUENRG_H_ */
