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
#include <zephyr/arch/arm/aarch32/nmi.h>
#include <zephyr/irq.h>
#include <zephyr/linker/linker-defs.h>
#include <string.h>
#include <soc.h>

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

#if 0
	key = irq_lock();


	/* Install default handler that simply resets the CPU
	 * if configured in the kernel, NOP otherwise
	 */
	// NMI_INIT();

	irq_unlock(key);
	
	/* enable clock */
	// LL_Power à ajouter (pour le sleep mode donc pas obligé mtn)
	// // BEGIN of SystemInit() 
	LL_PWR_EnableRAMBankRet(LL_PWR_RAMRET_1);
	// BEGIN of SmpsTrimConfig() 
	SystemReadyWait(200, LL_PWR_IsSMPSReady, 1);
	/* Config HW SMPS 10uH */
	LL_PWR_SetSMPSBOM(LL_PWR_SMPS_BOM3);
	/* SMPS Clock 4Mhz configuration */
	LL_RCC_SetSMPSPrescaler(LL_RCC_SMPS_DIV_4);

	uint32_t main_regulator, smps_out_voltage, lsi_bw, hsi_calib, lsi_lpmu;
	uint8_t eng_lsi_bw_flag;
	
	/* Retrieve Trimming values from engineering flash locations */
	if (*(volatile uint32_t*) 0x10001EF8 == 0xFCBCECCC) {
		main_regulator    = ((*(volatile uint32_t*)0x10001EE4) & (0x0F << 0)) >> 0;
		smps_out_voltage  = ((*(volatile uint32_t*)0x10001EE4) & (0x07 << 4)) >> 4;
		lsi_lpmu          = ((*(volatile uint32_t*)0x10001EE4) & (0x0F << 8)) >> 8;
		lsi_bw            = ((*(volatile uint32_t*)0x10001EE4) & (0x0F << 12)) >> 12;
		hsi_calib         = ((*(volatile uint32_t*)0x10001EE4) & (0x3F << 16)) >> 16;
		eng_lsi_bw_flag   = 1U; //TRUE
	} 
	else {
		main_regulator    = 0x08;
		lsi_lpmu          = 0x08;
		hsi_calib         = 0x1E;
		eng_lsi_bw_flag   = 0U; //FALSE
		smps_out_voltage  = 0x03;
	}
	
	/* Set HSI Calibration Trimming value */
	LL_RCC_HSI_SetCalibTrimming(hsi_calib);

	/* Low speed internal RC trimming value set by software */
	if (eng_lsi_bw_flag)
		LL_RCC_LSI_SetTrimming(lsi_bw);
	
	/* Set LSI LPMU Trimming value */
	LL_PWR_SetLSILPMUTrim(lsi_lpmu);
		
	/* Set Main Regulator voltage Trimming value */ 
	LL_PWR_SetMRTrim(main_regulator);

	/* Set SMPS output voltage Trimming value */
	LL_PWR_SetSMPSTrim(smps_out_voltage);
	
	/* Set SMPS in LP Open */
	LL_PWR_SetSMPSOpenMode(LL_PWR_SMPS_LPOPEN);	
	//END SmpsTrimConfig() function
	
	// BEGIN LSConfig()
	// Low speed crystal configuration 
	
	LL_PWR_SetNoPullB(LL_PWR_PUPD_IO12|LL_PWR_PUPD_IO13);
	LL_RCC_LSCO_SetSource(LL_RCC_LSCO_CLKSOURCE_LSE);
	
	// Set LSE oscillator drive capability 
	LL_RCC_LSE_SetDriveCapability(LL_RCC_LSEDRIVE_HIGH);
	
	LL_RCC_LSI_Disable();
	
	// Need to explicitly disable LSE to make LSERDY flag go to 0 even without POR, in case it was enabled. 
	LL_RCC_LSE_Disable();  
	SystemReadyWait(300, LL_RCC_LSE_IsReady, 0);

	LL_RCC_LSE_Enable();
	SystemReadyWait(300, LL_RCC_LSE_IsReady, 1);
	// END LSConfig()
	

	// Set current and capacitors for High Speed Crystal Oscillator
	LL_RCC_HSE_SetCapacitorTuning(32); //centrer crystal
	LL_RCC_HSE_SetCurrentControl(LL_RCC_HSE_CURRENTMAX_3); // LL_RCC_HSE_CURRENTMAX_3 : (RCC_RFSWHSECR_GMC_1| RCC_RFSWHSECR_GMC_0) : (0x2U << (4UL) | 0x1U << (4UL))


	// BEGIN SystemClockConfig()
	//SystemTimer_TimeoutConfig(32000000, 100, TRUE);
	LL_RCC_HSE_Enable();
	SystemReadyWait(100, LL_RCC_HSE_IsReady, 1);
	LL_RCC_SetRC64MPLLPrescaler(LL_RCC_RC64MPLL_DIV_4); // RCC 16Mhz : (0x4U) << (5UL)
	LL_RCC_RC64MPLL_Enable();
	if(SystemReadyWait(200, LL_RCC_RC64MPLL_IsReady, 1) == 0U);
	else { // DIRECT HSE configuration
		LL_RCC_SetRC64MPLLPrescaler(LL_RCC_RC64MPLL_DIV_2);
		LL_RCC_DIRECT_HSE_Enable();
		SystemCoreClock = 32000000;
	}
	
	// FLASH Wait State configuration
	if (SystemCoreClock != 64000000) {
		LL_FLASH_SetWaitStates(FLASH, LL_FLASH_WAIT_STATES_0);
	}
	// END SystemClockConfig()

	/* Set Systick to 1ms using system clock frequency */
	LL_InitTick(SystemCoreClock, 1000);

	// BEGIN Configure_GPIO()
	LL_AHB_EnableClock(0x4UL);
	LL_AHB_EnableClock(LL_AHB_PERIPH_GPIOB);
	LL_APB0_EnableClock(0x100UL);
	LL_APB1_EnableClock(0x400UL);

	// /* Configure IO in output */
	// LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_8, LL_GPIO_MODE_OUTPUT);
	// /* Reset value is LL_GPIO_OUTPUT_PUSHPULL */
	// LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_8, LL_GPIO_OUTPUT_PUSHPULL);
	// /* Reset value is LL_GPIO_SPEED_FREQ_LOW */
	// LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_8, LL_GPIO_SPEED_FREQ_LOW);
	// /* Reset value is LL_GPIO_PULL_NO */
	// LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_8, LL_GPIO_PULL_NO);
	// // END Configure_GPIO()


	// BEGIN MX_GPIO_Init()
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
	/* LED2 GPIO CLK ENABLE */
	// LL_AHB_EnableClock(LL_AHB_PERIPH_GPIOB);

	// LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_8);
	// GPIO_InitStruct.Pin = LL_GPIO_PIN_8;
	// GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	// GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	// GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	// GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	// LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	// END MX_GPIO_Init()

	// BEGIN MX_USART_UART_Init()
	// LL_USART_InitTypeDef USART_InitStruct = {0};

	/* Peripheral clock enable */
	// LL_APB1_EnableClock(LL_APB1_PERIPH_USART);

	GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	// GPIO_InitStruct.Pin = LL_GPIO_PIN_8;
	// GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	// GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	// GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	// GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
	// GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
	// LL_GPIO_Init(GPIOA, &GPIO_InitStruct);


	// USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
	// USART_InitStruct.BaudRate = 115200;
	// USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
	// USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
	// USART_InitStruct.Parity = LL_USART_PARITY_NONE;
	// USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
	// USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
	// USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;

	// LL_USART_Init(USART1, &USART_InitStruct);
	// LL_USART_SetTXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);
	// LL_USART_SetRXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);

	// LL_USART_EnableFIFO(USART1);
	// LL_USART_ConfigAsyncMode(USART1);
	// LL_USART_Enable(USART1);

	// /* Polling USART initialisation */
	// while((!(LL_USART_IsActiveFlag_TEACK(USART1))) || (!(LL_USART_IsActiveFlag_REACK(USART1))))
	// { 
	// }
	// // END MX_USART_UART_Init()

	// uint8_t ubSend = 0;
	// const uint8_t aStringToSend[] = "TEST\n\r";
	// while (ubSend < sizeof(aStringToSend))
	// {
	// 	/* Wait for TXE flag to be raised */
	// 	while (!LL_USART_IsActiveFlag_TXE(USART1))
	// 	{
	// 	}

	// 	/* If last char to be sent, clear TC flag */
	// 	if (ubSend == (sizeof(aStringToSend) - 1))
	// 	{
	// 	LL_USART_ClearFlag_TC(USART1); 
	// 	}

	// 	/* Write character in Transmit Data register.
	// 	TXE flag is cleared by writing data in TDR register */
	// 	LL_USART_TransmitData8(USART1, aStringToSend[ubSend++]);
	// }

	// /* Wait for TC flag to be raised for last char */
	// while (!LL_USART_IsActiveFlag_TC(USART1))
	// {
	// }

	// LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_8);




	// Sortir la HSE sur MCO
	
	/*
	//Avant de faire un UART, toggle un gpio dans une boucle infinie pour voir si mes inits clocks fonctionnent
	//Il faut qd même activer la clock du gpio et ensuite taper dans le registre en write 1, wait, write 0.
	LL_AHB_PERIPH_ALL
	Smps -> alimentation
	LSCOnfig -> choisir config RO
	SysclkCOnfig -> Mettre la sysclk en direct HSE. C'est la que je lance la HSE et que je peux voir sur mon crystal Q2 si j'ai ma clock 32MHz
	Radio clock config je m'en fou pour l'instant
	D'abord je fais toggle mon IO ici et qd ça marche j'essaie de le toggle dans le main à la place d'un printk
	Dans mon main, je met plus printk mais toogle IO
	
	Si ça marche, faire du debug
	Soit gdb breakpoint voir si on y va
	Soit flash depuis PC linux. Brancher board sur PC Windows, ouvrir keil sans projet. File -> attach to target et regarder les registres.
	Qd je fais du debug, pour rester dans la zone de code jusqu'à activation j'utilise des boucles d'attentes actives. Exemple : while(LL_PWR_IsSMPSReady()) pour pas que le code passe à la suite avant que SMPS activer.
	//init UART
	//MX_USART_UART_Init();
	*/
#endif
	return ret_val;
}

SYS_INIT(bluenrg_init, PRE_KERNEL_1, 0);
