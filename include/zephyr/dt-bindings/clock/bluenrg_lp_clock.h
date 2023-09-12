/*
 * Copyright (c) 2022 Linaro Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_BLUENRG_CLOCK_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_BLUENRG_CLOCK_H_

/** Bus gatting clocks */
#define BLUENRG_CLOCK_BUS_AHB0    0x050
#define BLUENRG_CLOCK_BUS_APB0    0x054
#define BLUENRG_CLOCK_BUS_APB1    0x058
#define BLUENRG_CLOCK_BUS_APB2    0x060

#define BLUENRG_PERIPH_BUS_MIN	BLUENRG_CLOCK_BUS_APB2
#define BLUENRG_PERIPH_BUS_MAX	BLUENRG_CLOCK_BUS_APB0

// La suite est à modifier

// /** Domain clocks */
// /* RM0479, §7.3.20 Clock configuration register (RCC_CCIPR) */

// /** Fixed clocks  */
// #define BLUENRG_SRC_HSE		0x001
// #define BLUENRG_SRC_LSE		0x002
// #define BLUENRG_SRC_LSI		0x003
// #define BLUENRG_SRC_HSI		0x004
// #define BLUENRG_SRC_HSI48		0x005
// /** System clock */
// #define BLUENRG_SRC_SYSCLK	0x006
// /** Bus clock */
// #define BLUENRG_SRC_PCLK		0x007

// #define BLUENRG_CLOCK_REG_MASK    0xFFU
// #define BLUENRG_CLOCK_REG_SHIFT   0U
// #define BLUENRG_CLOCK_SHIFT_MASK  0x1FU
// #define BLUENRG_CLOCK_SHIFT_SHIFT 8U
// #define BLUENRG_CLOCK_MASK_MASK   0x7U
// #define BLUENRG_CLOCK_MASK_SHIFT  13U
// #define BLUENRG_CLOCK_VAL_MASK    0x7U
// #define BLUENRG_CLOCK_VAL_SHIFT   16U

// /**
//  * @brief BLUENRG clock configuration bit field.
//  *
//  * - reg   (1/2/3)         [ 0 : 7 ]
//  * - shift (0..31)         [ 8 : 12 ]
//  * - mask  (0x1, 0x3, 0x7) [ 13 : 15 ]
//  * - val   (0..7)          [ 16 : 18 ]
//  *
//  * @param reg RCC_CCIPRx register offset
//  * @param shift Position within RCC_CCIPRx.
//  * @param mask Mask for the RCC_CCIPRx field.
//  * @param val Clock value (0, 1, ... 7).
//  */
#if 0
#define BLUENRG_CLOCK(val, mask, shift, reg)					\
	((((reg) & BLUENRG_CLOCK_REG_MASK) << BLUENRG_CLOCK_REG_SHIFT) |		\
	 (((shift) & BLUENRG_CLOCK_SHIFT_MASK) << BLUENRG_CLOCK_SHIFT_SHIFT) |	\
	 (((mask) & BLUENRG_CLOCK_MASK_MASK) << BLUENRG_CLOCK_MASK_SHIFT) |		\
	 (((val) & BLUENRG_CLOCK_VAL_MASK) << BLUENRG_CLOCK_VAL_SHIFT))
#endif
// /** @brief RCC_CCIPR register offset */
// #define CCIPR_REG		0x4C

// /** @brief RCC_CSR register offset */
// #define CSR_REG		0x50

// /** @brief Device domain clocks selection helpers */
// /** CCIPR devices */
// #define USART1_SEL(val)		BLUENRG_CLOCK(val, 3, 0, CCIPR_REG)
// #define USART2_SEL(val)		BLUENRG_CLOCK(val, 3, 2, CCIPR_REG)
// #define LPUART1_SEL(val)	BLUENRG_CLOCK(val, 3, 10, CCIPR_REG)
// #define I2C1_SEL(val)		BLUENRG_CLOCK(val, 3, 12, CCIPR_REG)
// #define I2C3_SEL(val)		BLUENRG_CLOCK(val, 3, 16, CCIPR_REG)
// #define LPTIM1_SEL(val)		BLUENRG_CLOCK(val, 3, 18, CCIPR_REG)
// #define HSI48_SEL(val)		BLUENRG_CLOCK(val, 1, 26, CCIPR_REG)
// /** CSR devices */
// #define RTC_SEL(val)		BLUENRG_CLOCK(val, 3, 16, CSR_REG)

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_BLUENRGL0_CLOCK_H_ */
