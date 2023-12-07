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
/* TRIMMING Defines */
#define VALIDITY_TAG 0xFCBCECCC

#define VALIDITY_LOCATION    0x10001EF8
#define TRIMMING_LOCATION    0x10001EE4
#define MR_TRIMMING_LOCATION 0x10001EE8

#define MAIN_REGULATOR_TRIM_Pos (0)
#define MAIN_REGULATOR_TRIM_Msk (0x0F << MAIN_REGULATOR_TRIM_Pos)
#define SMPS_TRIM_Pos           (4)
#define SMPS_TRIM_Msk           (0x07 << SMPS_TRIM_Pos)
#define LSI_LPMU_TRIM_Pos       (8)
#define LSI_LPMU_TRIM_Msk       (0x0F << LSI_LPMU_TRIM_Pos)
#define LSI_BW_TRIM_Pos         (12)
#define LSI_BW_TRIM_Msk         (0x0F << LSI_BW_TRIM_Pos)
#define HSI_TRIM_Pos            (16)
#define HSI_TRIM_Msk            (0x3F << HSI_TRIM_Pos)

/**
  * @brief  SMPS and Trimming value Configuration 
  */
static uint8_t SetTrimConfig(void)
{
  uint8_t ret_val=SUCCESS;
  uint32_t main_regulator, smps_out_voltage, lsi_bw, hsi_calib;
  uint8_t eng_lsi_bw_flag;
#ifdef CONFIG_DEVICE_BLUENRG_LP
  uint32_t lsi_lpmu;
#endif /* CONFIG_DEVICE_BLUENRG_LP */

  /* Retrieve Trimming values from engineering flash locations */
  if (*(volatile uint32_t*)VALIDITY_LOCATION == VALIDITY_TAG) {
    main_regulator    = ((*(volatile uint32_t*)TRIMMING_LOCATION) & MAIN_REGULATOR_TRIM_Msk) >> MAIN_REGULATOR_TRIM_Pos;
    smps_out_voltage  = ((*(volatile uint32_t*)TRIMMING_LOCATION) & SMPS_TRIM_Msk) >> SMPS_TRIM_Pos;
#ifdef CONFIG_DEVICE_BLUENRG_LP
    lsi_lpmu          = ((*(volatile uint32_t*)TRIMMING_LOCATION) & LSI_LPMU_TRIM_Msk) >> LSI_LPMU_TRIM_Pos;
#endif /* CONFIG_DEVICE_BLUENRG_LP */
    lsi_bw            = ((*(volatile uint32_t*)TRIMMING_LOCATION) & LSI_BW_TRIM_Msk) >> LSI_BW_TRIM_Pos;
    hsi_calib         = ((*(volatile uint32_t*)TRIMMING_LOCATION) & HSI_TRIM_Msk) >> HSI_TRIM_Pos;
    eng_lsi_bw_flag   = TRUE;
  } else {
#ifdef CONFIG_DEVICE_BLUENRG_LP
    main_regulator    = 0x08;
    lsi_lpmu          = 0x08;
    hsi_calib         = 0x1E;
    eng_lsi_bw_flag   = FALSE;
#endif
#if defined(CONFIG_DEVICE_BLUENRG_LPS)
    main_regulator    = 0x0A;
    hsi_calib         = 0x1F;
    lsi_bw            = 8;
    eng_lsi_bw_flag   = TRUE;
#endif
    smps_out_voltage  = 0x03;
  }
  
  /* Set HSI Calibration Trimming value */
  LL_RCC_HSI_SetCalibTrimming(hsi_calib);

  /* Low speed internal RC trimming value set by software */
  if (eng_lsi_bw_flag)
    LL_RCC_LSI_SetTrimming(lsi_bw);
  
#ifdef CONFIG_DEVICE_BLUENRG_LP
  /* Set LSI LPMU Trimming value */
  LL_PWR_SetLSILPMUTrim(lsi_lpmu);
#endif /* CONFIG_DEVICE_BLUENRG_LP */
	
  /* Set Main Regulator voltage Trimming value */ 
  LL_PWR_SetMRTrim(main_regulator);

  /* Set SMPS output voltage Trimming value */
  LL_PWR_SetSMPSTrim(smps_out_voltage);
  
  /* Set SMPS in LP Open */
  LL_PWR_SetSMPSOpenMode(LL_PWR_SMPS_LPOPEN);
  
#ifdef CONFIG_HW_SMPS_NONE
  /* No SMPS configuration */
  LL_PWR_SetSMPSMode(LL_PWR_NO_SMPS);
#endif

  return ret_val;
}

NO_INIT_SECTION(REQUIRED(RAM_VR_TypeDef RAM_VR), "RAM_VR_section");
NO_INIT_SECTION(REQUIRED(static uint32_t fill_gap[36]), "FILL_GAP_section");
SECTION("BLUE_RAM_section")
REQUIRED(uint8_t __blue_RAM[1*80+28]) = {0,};
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
	int ret_val = 0;
	/* Remap address 0 to user flash memory */
	LL_SYSCFG_SetRemapMemory(LL_SYSCFG_REMAP_FLASH);
	RAM_VR.AppBase = (unsigned int) &_vector_table;
	/* Clear Blue RAM first word */
	*(uint32_t *)__blue_RAM = 0;
	
	/* Enable all the RAM banks in retention during DEEPSTOP */
	LL_PWR_EnableRAMBankRet(LL_PWR_RAMRET_1);
#if defined(LL_PWR_RAMRET_2)
	LL_PWR_EnableRAMBankRet(LL_PWR_RAMRET_2);
#endif
#if defined(LL_PWR_RAMRET_3)
	LL_PWR_EnableRAMBankRet(LL_PWR_RAMRET_3);
#endif
  
#if defined(PWR_CR2_GPIORET)
      /* Disable the GPIO retention in DEEPSTOP configuration */
      LL_PWR_DisableGPIORET();
#endif
      /* HW SMPS and HW Trimming value Configuration */
      ret_val = SetTrimConfig();

      return ret_val;
}

SYS_INIT(bluenrg_init, PRE_KERNEL_1, 0);
