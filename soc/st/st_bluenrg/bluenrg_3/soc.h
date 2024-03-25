// /*
//  * Copyright (c) 2016 Open-RnD Sp. z o.o.
//  * Copyright (c) 2016 BayLibre, SAS
//  *
//  * SPDX-License-Identifier: Apache-2.0
//  */

/**
 * @file SoC configuration macros for the STM32L4 family processors.
 *
 * Based on reference manual:
 *   STM32L4x1, STM32L4x2, STM32L431xx STM32L443xx STM32L433xx, STM32L4x5,
 *   STM32l4x6 advanced ARM(r)-based 32-bit MCUs
 *
 * Chapter 2.2.2: Memory map and register boundary addresses
 */

#ifndef _BLUENRG_LP_SOC_H_
#define _BLUENRG_LP_SOC_H_

#include <autoconf.h>

#if defined(CONFIG_SOC_BLUENRG_LP)
#include <BlueNRG_LP.h>

#elif defined(CONFIG_SOC_BLUENRG_LPS)
#include <BlueNRG_LPS.h>

#else
	#error "Board not supported"
#endif /* defined(CONFIG_SOC_BLUENRG_LP) */

/* Add include for DTS generated information */
#include <rf_driver_ll_flash.h>
#include <rf_driver_ll_rcc.h>
#include <rf_driver_ll_pwr.h>
#include <rf_driver_ll_system.h>
#endif /* _BLUENRG_LP_SOC_H_ */
