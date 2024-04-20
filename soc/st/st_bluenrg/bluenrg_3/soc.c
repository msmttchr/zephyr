/*
 * Copyright (c) 2016 Open-RnD Sp. z o.o.
 * Copyright (c) 2016 BayLibre, SAS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief System/hardware module for STM32L4 processor
 */

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/arch/arm/nmi.h>
#include <zephyr/irq.h>
#include <zephyr/linker/linker-defs.h>
#include <string.h>
#include <soc.h>
#include "system_BlueNRG_LP.h"

#ifndef CONFIG_NUM_MAX_LINKS
#define CONFIG_NUM_MAX_LINKS 0
#endif

Z_GENERIC_SECTION("RAM_VR_section")
RAM_VR_TypeDef __used RAM_VR;
Z_GENERIC_SECTION("FILL_GAP_section")
static uint32_t __used fill_gap[36];
Z_GENERIC_SECTION("BLUE_RAM_section")

#if defined(CONFIG_SOC_BLUENRG_LP)
uint8_t __used __blue_RAM[CONFIG_NUM_MAX_LINKS*80+28] = {0,}; /* TBD_st Do it with DTS??? */

#elif defined(CONFIG_SOC_BLUENRG_LPS) || defined(CONFIG_SOC_BLUENRG_LPF)
uint8_t __used __blue_RAM[CONFIG_NUM_MAX_LINKS*92+28] = {0,};

#else
#error "Unknown device"

#endif /* CONFIG_SOC_BLUENRG_LP */

extern void * _vector_table;

/**
 * @brief Perform basic hardware initialization at boot.
 *
 * This needs to be run from the very beginning.
 * So the init priority has to be 0 (zero).
 *
 * @return 0
 */
static int bluenrg_init(void)
{
	/* Remap address 0 to user flash memory */
	LL_SYSCFG_SetRemapMemory(LL_SYSCFG_REMAP_FLASH);
	return SystemInit(0, 0);
}

SYS_INIT(bluenrg_init, PRE_KERNEL_1, 0);
