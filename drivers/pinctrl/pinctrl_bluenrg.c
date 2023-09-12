/*
 * Copyright (c) 2016 Open-RnD Sp. z o.o.
 * Copyright (c) 2021 Linaro Limited
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/init.h>
#include <zephyr/drivers/pinctrl.h>
#include <gpio/gpio_bluenrg.h>

#include <rf_driver_ll_bus.h>
#include <rf_driver_ll_gpio.h>
#include <rf_driver_ll_system.h>

/** Helper to extract IO port number from STM32PIN() encoded value */
#define STM32_PORT(__pin) \
	((__pin) >> 4)

/** Helper to extract IO pin number from STM32PIN() encoded value */
#define STM32_PIN(__pin) \
	((__pin) & 0xf)

/** Helper to extract IO port number from STM32_PINMUX() encoded value */
#define STM32_DT_PINMUX_PORT(__pin) \
	(((__pin) >> STM32_PORT_SHIFT) & STM32_PORT_MASK)

/** Helper to extract IO pin number from STM32_PINMUX() encoded value */
#define STM32_DT_PINMUX_LINE(__pin) \
	(((__pin) >> STM32_LINE_SHIFT) & STM32_LINE_MASK)

/** Helper to extract IO pin func from STM32_PINMUX() encoded value */
#define STM32_DT_PINMUX_FUNC(__pin) \
	(((__pin) >> STM32_MODE_SHIFT) & STM32_MODE_MASK)

/**
 * @brief Array containing pointers to each GPIO port.
 *
 * Entries will be NULL if the GPIO port is not enabled.
 */
static const struct device *const gpio_ports[] = {
	DEVICE_DT_GET_OR_NULL(DT_NODELABEL(gpioa)),
	DEVICE_DT_GET_OR_NULL(DT_NODELABEL(gpiob)),
};

/** Number of GPIO ports. */
static const size_t gpio_ports_cnt = ARRAY_SIZE(gpio_ports);

static int stm32_pin_configure(uint32_t pin, uint32_t pin_cgf, uint32_t pin_func)
{
	const struct device *port_device;

	if (STM32_PORT(pin) >= gpio_ports_cnt) {
		return -EINVAL;
	}

	port_device = gpio_ports[STM32_PORT(pin)];

	if ((port_device == NULL) || (!device_is_ready(port_device))) {
		return -ENODEV;
	}

	return gpio_bluenrg_configure(port_device, STM32_PIN(pin), pin_cgf, pin_func);
}

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt,
			   uintptr_t reg)
{
	uint32_t pin, mux;
	uint32_t pin_cgf = 0;
	int ret = 0;

	ARG_UNUSED(reg);

	for (uint8_t i = 0U; i < pin_cnt; i++) {
		mux = pins[i].pinmux;

		if (STM32_DT_PINMUX_FUNC(mux) < STM32_ANALOG) {
			pin_cgf = pins[i].pincfg | STM32_MODER_ALT_MODE;
		} else if (STM32_DT_PINMUX_FUNC(mux) == STM32_ANALOG) {
			pin_cgf = STM32_MODER_ANALOG_MODE;
		} else if (STM32_DT_PINMUX_FUNC(mux) == STM32_GPIO) {
			pin_cgf = pins[i].pincfg;
		} else {
			/* Not supported */
			__ASSERT_NO_MSG(STM32_DT_PINMUX_FUNC(mux));
		}

		pin = STM32PIN(STM32_DT_PINMUX_PORT(mux),
			       STM32_DT_PINMUX_LINE(mux));

		ret = stm32_pin_configure(pin, pin_cgf, STM32_DT_PINMUX_FUNC(mux));
		if (ret < 0) {
			return ret;
		}
	}

	return 0;
}
