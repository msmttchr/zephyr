/*
 * Copyright (c) 2017-2022 Linaro Limited.
 * Copyright (c) 2017 RnDity Sp. z o.o.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <soc.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include "system_BlueNRG_LP.h"

/* Macros to fill up prescaler values */
#define z_hsi_divider(v) LL_RCC_HSI_DIV_ ## v
#define hsi_divider(v) z_hsi_divider(v)

#define fn_ahb_prescaler(v) LL_RCC_SYSCLK_DIV_ ## v
#define ahb_prescaler(v) fn_ahb_prescaler(v)

#define fn_apb1_prescaler(v) LL_RCC_APB1_DIV_ ## v
#define apb1_prescaler(v) fn_apb1_prescaler(v)

#if DT_NODE_HAS_PROP(DT_NODELABEL(rcc), apb2_prescaler)
#define fn_apb2_prescaler(v) LL_RCC_APB2_DIV_ ## v
#define apb2_prescaler(v) fn_apb2_prescaler(v)
#endif

#if DT_NODE_HAS_PROP(DT_NODELABEL(rcc), ahb4_prescaler)
#define RCC_CALC_FLASH_FREQ __LL_RCC_CALC_HCLK4_FREQ
#define GET_CURRENT_FLASH_PRESCALER LL_RCC_GetAHB4Prescaler
#elif DT_NODE_HAS_PROP(DT_NODELABEL(rcc), ahb3_prescaler)
#define RCC_CALC_FLASH_FREQ __LL_RCC_CALC_HCLK3_FREQ
#define GET_CURRENT_FLASH_PRESCALER LL_RCC_GetAHB3Prescaler
#else
#define RCC_CALC_FLASH_FREQ __LL_RCC_CALC_HCLK_FREQ
#define GET_CURRENT_FLASH_PRESCALER LL_RCC_GetAHBPrescaler
#endif

#if defined(RCC_PLLCFGR_PLLPEN)
#define RCC_PLLP_ENABLE() SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLPEN)
#else
#define RCC_PLLP_ENABLE()
#endif /* RCC_PLLCFGR_PLLPEN */
#if defined(RCC_PLLCFGR_PLLQEN)
#define RCC_PLLQ_ENABLE() SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLQEN)
#else
#define RCC_PLLQ_ENABLE()
#endif /* RCC_PLLCFGR_PLLQEN */

uint32_t SystemCoreClock;

/**
  * @brief Wait for a peripheral to be ready or not.
  *
  * This function waits until the value returned by the checking function, i.e.
  * ready_func(), returns the specified value. The function returns also after
  * the given timeout.
  * @param timeout_ms Interval (in milliseconds) after which the function will
  *        returns even if the checking function has not returned the desired
  *        value.
  * @param ready_func Pointer to the function checking for the readiness of a
  *        peripheral.
  * @param ready_val The SystemReadyWait() function returns tot he caller after
  *        the ready_func() has returned the value specified in ready_val.
  * @retval TRUE if the ready_func() returned the value specified in ready_val
  *         before the given timeout. It returns FALSE if the timeout expired.
  */ 
uint8_t SystemReadyWait(uint32_t timeout_ms, uint32_t (*ready_func)(void), uint32_t ready_val)
{
  uint8_t ret_val = TRUE;
  volatile unsigned int timeout = 0xFFFFFF;
  
  while((*ready_func)() != ready_val) 
  {
    timeout--;
    if (timeout == 0) {
      ret_val = FALSE;
      break;
    }
  }
  
  return ret_val;
}

/**
 * @brief Return frequency for pll with 2 dividers and a multiplier
 */
__unused
static uint32_t get_pll_div_frequency(uint32_t pllsrc_freq,
				      int pllm_div,
				      int plln_mul,
				      int pllout_div)
{
	__ASSERT_NO_MSG(pllm_div && pllout_div);

	return pllsrc_freq / pllm_div * plln_mul / pllout_div;
}

static uint32_t get_bus_clock(uint32_t clock, uint32_t prescaler)
{
	return clock / prescaler;
}

__unused
static uint32_t get_msi_frequency(void)
{
#if defined(STM32_MSI_ENABLED)
#if !defined(LL_RCC_MSIRANGESEL_RUN)
	return __LL_RCC_CALC_MSI_FREQ(LL_RCC_MSI_GetRange());
#else
	return __LL_RCC_CALC_MSI_FREQ(LL_RCC_MSIRANGESEL_RUN,
				      LL_RCC_MSI_GetRange());
#endif
#endif
	return 0;
}

/** @brief Verifies clock is part of active clock configuration */
__unused
static int enabled_clock(uint32_t src_clk)
{
	int r = 0;

	switch (src_clk) {
#if defined(STM32_SRC_SYSCLK)
	case STM32_SRC_SYSCLK:
		break;
#endif /* STM32_SRC_SYSCLK */
#if defined(STM32_SRC_PCLK)
	case STM32_SRC_PCLK:
		break;
#endif /* STM32_SRC_PCLK */
#if defined(STM32_SRC_HSE)
	case STM32_SRC_HSE:
		if (!IS_ENABLED(STM32_HSE_ENABLED)) {
			r = -ENOTSUP;
		}
		break;
#endif /* STM32_SRC_HSE */
#if defined(STM32_SRC_HSI)
	case STM32_SRC_HSI:
		if (!IS_ENABLED(STM32_HSI_ENABLED)) {
			r = -ENOTSUP;
		}
		break;
#endif /* STM32_SRC_HSI */
#if defined(STM32_SRC_LSE)
	case STM32_SRC_LSE:
		if (!IS_ENABLED(STM32_LSE_ENABLED)) {
			r = -ENOTSUP;
		}
		break;
#endif /* STM32_SRC_LSE */
#if defined(STM32_SRC_LSI)
	case STM32_SRC_LSI:
		if (!IS_ENABLED(STM32_LSI_ENABLED)) {
			r = -ENOTSUP;
		}
		break;
#endif /* STM32_SRC_LSI */
#if defined(STM32_SRC_HSI14)
	case STM32_SRC_HSI14:
		if (!IS_ENABLED(STM32_HSI14_ENABLED)) {
			r = -ENOTSUP;
		}
		break;
#endif /* STM32_SRC_HSI14 */
#if defined(STM32_SRC_HSI48)
	case STM32_SRC_HSI48:
		if (!IS_ENABLED(STM32_HSI48_ENABLED)) {
			r = -ENOTSUP;
		}
		break;
#endif /* STM32_SRC_HSI48 */
#if defined(STM32_SRC_MSI)
	case STM32_SRC_MSI:
		if (!IS_ENABLED(STM32_MSI_ENABLED)) {
			r = -ENOTSUP;
		}
		break;
#endif /* STM32_SRC_MSI */
#if defined(STM32_SRC_PLLCLK)
	case STM32_SRC_PLLCLK:
		if (!IS_ENABLED(STM32_PLL_ENABLED)) {
			r = -ENOTSUP;
		}
		break;
#endif /* STM32_SRC_PLLCLK */
#if defined(STM32_SRC_PLL_P)
	case STM32_SRC_PLL_P:
		if (!IS_ENABLED(STM32_PLL_P_ENABLED)) {
			r = -ENOTSUP;
		}
		break;
#endif /* STM32_SRC_PLL_P */
#if defined(STM32_SRC_PLL_Q)
	case STM32_SRC_PLL_Q:
		if (!IS_ENABLED(STM32_PLL_Q_ENABLED)) {
			r = -ENOTSUP;
		}
		break;
#endif /* STM32_SRC_PLL_Q */
#if defined(STM32_SRC_PLL_R)
	case STM32_SRC_PLL_R:
		if (!IS_ENABLED(STM32_PLL_R_ENABLED)) {
			r = -ENOTSUP;
		}
		break;
#endif /* STM32_SRC_PLL_R */
#if defined(STM32_SRC_PLLI2S_R)
	case STM32_SRC_PLLI2S_R:
		if (!IS_ENABLED(STM32_PLLI2S_R_ENABLED)) {
			r = -ENOTSUP;
		}
		break;
#endif /* STM32_SRC_PLLI2S_R */
	default:
		return -ENOTSUP;
	}

	return r;
}

static inline int bluenrg_clock_control_on(const struct device *dev,
					 clock_control_subsys_t sub_system)
{
	struct stm32_pclken *pclken = (struct stm32_pclken *)(sub_system);

	ARG_UNUSED(dev);

	if (IN_RANGE(pclken->bus, 0x50, 0x60) == 0) {
		/* Attemp to change a wrong periph clock bit */
		return -ENOTSUP;
	}

	sys_set_bits(DT_REG_ADDR(DT_NODELABEL(rcc)) + pclken->bus,
		     pclken->enr);

	return 0;
}

static inline int bluenrg_clock_control_off(const struct device *dev,
					  clock_control_subsys_t sub_system)
{
	struct stm32_pclken *pclken = (struct stm32_pclken *)(sub_system);

	ARG_UNUSED(dev);

	if (IN_RANGE(pclken->bus, 0x50, 0x60) == 0) {
		/* Attemp to toggle a wrong periph clock bit */
		return -ENOTSUP;
	}

	sys_clear_bits(DT_REG_ADDR(DT_NODELABEL(rcc)) + pclken->bus,
		       pclken->enr);

	return 0;
}

/**
  * @brief  Low Speed Configuration
  */
#define CONFIG_HW_LS_XTAL
static uint8_t LSConfig(void)
{
  uint8_t ret_val=SUCCESS;
  
  /* Low speed crystal configuration */
#ifdef CONFIG_HW_LS_XTAL
  LL_PWR_SetNoPullB(LL_PWR_PUPD_IO12|LL_PWR_PUPD_IO13);
  LL_RCC_LSCO_SetSource(LL_RCC_LSCO_CLKSOURCE_LSE);
  
   /* Set LSE oscillator drive capability */
  LL_RCC_LSE_SetDriveCapability(LL_RCC_LSEDRIVE_HIGH);
  
  LL_RCC_LSI_Disable();
  
  /* Need to explicitly disable LSE to make LSERDY flag go to 0 even without
    POR, in case it was enabled. */
  LL_RCC_LSE_Disable();  
  if(SystemReadyWait(300, LL_RCC_LSE_IsReady, 0) == FALSE)
  {
    ret_val = SYSTEM_CONFIG_LSE_READY_ERROR;
  }

  LL_RCC_LSE_Enable();
  if(SystemReadyWait(300, LL_RCC_LSE_IsReady, 1) == FALSE)
  {
    ret_val = SYSTEM_CONFIG_LSE_READY_ERROR;
  }
#elif defined(CONFIG_HW_LS_RO)
  LL_RCC_LSI_Disable();
  if(SystemReadyWait(300, LL_RCC_LSI_IsReady, 0) == FALSE)
  {
    ret_val = SYSTEM_CONFIG_LSI_READY_ERROR;
  }  
  LL_RCC_LSE_Disable();
  LL_RCC_LSCO_SetSource(LL_RCC_LSCO_CLKSOURCE_LSI);
  
  LL_RCC_LSI_Enable();
  if(SystemReadyWait(300, LL_RCC_LSI_IsReady, 1) == FALSE)
  {
    ret_val = SYSTEM_CONFIG_LSI_READY_ERROR;
  } 
#else
#ifndef SUPPRESS_CONFIG_HW_WARNINGS
  #warning "No Low Speed Crystal definition!!!"
#endif
#endif
  
  return ret_val;
}


static inline int bluenrg_clock_control_configure(const struct device *dev,
						  clock_control_subsys_t sub_system,
						  void *data)
{
#if defined(STM32_SRC_SYSCLK)
	/* At least one alt src clock available */
	struct stm32_pclken *pclken = (struct stm32_pclken *)(sub_system);
	int err;

	ARG_UNUSED(dev);
	ARG_UNUSED(data);

	err = enabled_clock(pclken->bus);
	if (err < 0) {
		/* Attempt to configure a src clock not available or not valid */
		return err;
	}

	if (pclken->enr == NO_SEL) {
		/* Domain clock is fixed. Nothing to set. Exit */
		return 0;
	}

	sys_clear_bits(DT_REG_ADDR(DT_NODELABEL(rcc)) + STM32_CLOCK_REG_GET(pclken->enr),
		       STM32_CLOCK_MASK_GET(pclken->enr) << STM32_CLOCK_SHIFT_GET(pclken->enr));
	sys_set_bits(DT_REG_ADDR(DT_NODELABEL(rcc)) + STM32_CLOCK_REG_GET(pclken->enr),
		     STM32_CLOCK_VAL_GET(pclken->enr) << STM32_CLOCK_SHIFT_GET(pclken->enr));

	return 0;
#else
	/* No src clock available: Not supported */
	return -ENOTSUP;
#endif
}

static int bluenrg_clock_control_get_subsys_rate(const struct device *clock,
						clock_control_subsys_t sub_system,
						uint32_t *rate)
{
	struct stm32_pclken *pclken = (struct stm32_pclken *)(sub_system);
	/*
	 * Get AHB Clock (= SystemCoreClock = SYSCLK/prescaler)
	 * SystemCoreClock is preferred to CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC
	 * since it will be updated after clock configuration and hence
	 * more likely to contain actual clock speed
	 */
	uint32_t KERNEL_CLK = 16000000;
	uint32_t CLK_SYS_BLE = 16000000;
	uint32_t ahb_clock = SystemCoreClock;
	uint32_t apb0_clock = 32768;
	uint32_t apb1_clock = KERNEL_CLK;
	uint32_t apb2_clock = CLK_SYS_BLE;


	ARG_UNUSED(clock);

	switch (pclken->bus) {
	case 0x50: /* AHB: DMA, GPIOA, GPIOB, CRC, PKA, RNG */
		*rate = ahb_clock;
		break;
	case 0x54: /* APB0: TIM1, SYSCFG, RTC, WDG */
		*rate = apb0_clock;
		break;
	case 0x58: /* APB1: SPI1, ADCDIG, ADCANA, LPUART, USART, SPI2, SPI3, I2C1, I2C2 */
		*rate = apb1_clock;
		break;
	case 0x60: /* APB2: MRBLE */
		*rate = apb2_clock;
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static enum clock_control_status bluenrg_clock_control_get_status(const struct device *dev,
								clock_control_subsys_t sub_system)
{
	struct stm32_pclken *pclken = (struct stm32_pclken *)sub_system;

	ARG_UNUSED(dev);
#if 0
	if (IN_RANGE(pclken->bus, STM32_PERIPH_BUS_MIN, STM32_PERIPH_BUS_MAX) == true) {
		/* Gated clocks */
		if ((sys_read32(DT_REG_ADDR(DT_NODELABEL(rcc)) + pclken->bus) & pclken->enr)
		    == pclken->enr) {
			return CLOCK_CONTROL_STATUS_ON;
		} else {
			return CLOCK_CONTROL_STATUS_OFF;
		}
	} else {
		/* Domain clock sources */
		if (enabled_clock(pclken->bus) == 0) {
			return CLOCK_CONTROL_STATUS_ON;
		} else {
			return CLOCK_CONTROL_STATUS_OFF;
		}
	}
#endif
	return CLOCK_CONTROL_STATUS_OFF;
}

static struct clock_control_driver_api bluenrg_clock_control_api = {
	.on = bluenrg_clock_control_on,
	.off = bluenrg_clock_control_off,
	.get_rate = bluenrg_clock_control_get_subsys_rate,
	.get_status = bluenrg_clock_control_get_status,
	.configure = bluenrg_clock_control_configure,
};


void config_enable_default_clocks(void)
{
}
/**
  * @brief  System Clock Configuration for System Core, low speed
  *         BLE RF and AHB bus
  * @param  SysClk system clock divided factor from HSI64MPLL
  *         This parameter can be one of the following values:
  *         SYSCLK_64M
  *         SYSCLK_32M
  *         SYSCLK_16M
  *         SYSCLK_8M
  *         SYSCLK_4M
  *         SYSCLK_2M
  *         SYSCLK_1M
  *         SYSCLK_DIRECT_HSE
  * @retval SUCCESS or error code
  */
uint8_t SystemClockConfig(uint8_t SysClk)
{ 
  uint8_t ret_val=SUCCESS;
    
  /* High speed crystal configuration: BlueNRG-LP supports only HSE 32 MHz */
  LL_RCC_HSE_Enable();
  
  if(SystemReadyWait(100, LL_RCC_HSE_IsReady, 1) == FALSE)
  {
    ret_val = SYSTEM_CONFIG_HSE_READY_ERROR;
    return ret_val;
  }
  
#if defined(CONFIG_SOC_BLUENRG_LP)
  /* BlueNRG_LP cut 1.0 not support DIRECT HSE configuration */
  if ((SysClk == SYSCLK_DIRECT_HSE) && (((LL_SYSCFG_GetDeviceVersion()<<4)|LL_SYSCFG_GetDeviceRevision()) == LL_BLUENRG_LP_CUT_10)) {
    return SYSTEM_CONFIG_DIRECT_HSE_NOT_SUPPORTED;
  }
#endif /* defined(CONFIG_SOC_BLUENRG_LP) */
  
  if (SysClk != SYSCLK_DIRECT_HSE) {
    /* System PLL Clock configuration */
    switch(SysClk)
    {
    case SYSCLK_64M:
      LL_FLASH_SetWaitStates(FLASH, LL_FLASH_WAIT_STATES_1);
      LL_RCC_SetRC64MPLLPrescaler(LL_RCC_RC64MPLL_DIV_1);
      SystemCoreClock = 64000000;
      break;
    case SYSCLK_32M:
      LL_RCC_SetRC64MPLLPrescaler(LL_RCC_RC64MPLL_DIV_2);
      SystemCoreClock = 32000000;
      break;
    case SYSCLK_16M:
      LL_RCC_SetRC64MPLLPrescaler(LL_RCC_RC64MPLL_DIV_4);
      SystemCoreClock = 16000000;
      break;
    case SYSCLK_8M:
      LL_RCC_SetRC64MPLLPrescaler(LL_RCC_RC64MPLL_DIV_8);
      SystemCoreClock = 8000000;
      break;
    case SYSCLK_4M:
      LL_RCC_SetRC64MPLLPrescaler(LL_RCC_RC64MPLL_DIV_16);
      SystemCoreClock = 4000000;
      break;
    case SYSCLK_2M:
      LL_RCC_SetRC64MPLLPrescaler(LL_RCC_RC64MPLL_DIV_32);
      SystemCoreClock = 2000000;
      break;
    case SYSCLK_1M:
      LL_RCC_SetRC64MPLLPrescaler(LL_RCC_RC64MPLL_DIV_64);
      SystemCoreClock = 1000000;
      break;
    default:
      /* Error, wrong choice */
      while(1);
    }
    LL_RCC_RC64MPLL_Enable();
    
    if(SystemReadyWait(200, LL_RCC_RC64MPLL_IsReady, 1) == FALSE)
    {
      ret_val = SYSTEM_CONFIG_PLL_READY_ERROR;
      return ret_val;
    }  
  } else { // DIRECT HSE configuration
    LL_RCC_SetRC64MPLLPrescaler(LL_RCC_RC64MPLL_DIV_2);
    LL_RCC_DIRECT_HSE_Enable();
    SystemCoreClock = 32000000;
  }
  
  /* FLASH Wait State configuration */
  if (SystemCoreClock != 64000000) {
    LL_FLASH_SetWaitStates(FLASH, LL_FLASH_WAIT_STATES_0);
  }
  
  return SUCCESS;
}

/**
 * @brief Initialize clocks for the stm32
 *
 * This routine is called to enable and configure the clocks and PLL
 * of the soc on the board. It depends on the board definition.
 * This function is called on the startup and also to restore the config
 * when exiting for low power mode.
 *
 * @param dev clock device struct
 *
 * @return 0
 */
int bluenrg_clock_control_init(const struct device *dev)
{
  uint32_t ret_val;
	ARG_UNUSED(dev);

	/* Some clocks would be activated by default */
	config_enable_default_clocks();

  /* Low Speed Crystal Configuration */
  ret_val = LSConfig();
  if (ret_val!= SUCCESS)
      return ret_val;

  /* Set current and capacitors for High Speed Crystal Oscillator */
#ifdef CONFIG_HW_HSE_TUNE
  LL_RCC_HSE_SetCapacitorTuning(CONFIG_HW_HSE_TUNE);
#else
#ifndef SUPPRESS_CONFIG_HW_WARNINGS
	#warning "No HSE Tune configuration!!!"
#endif	
#endif
  LL_RCC_HSE_SetCurrentControl(LL_RCC_HSE_CURRENTMAX_3);

  /* System Clock Configuration */
  ret_val = SystemClockConfig(SYSCLK_64M);
  if (ret_val!= SUCCESS)
    return ret_val;

	return 0;
}

/**
 * @brief RCC device, note that priority is intentionally set to 1 so
 * that the device init runs just after SOC init
 */
DEVICE_DT_DEFINE(DT_NODELABEL(rcc),
		    &bluenrg_clock_control_init,
		    NULL,
		    NULL, NULL,
		    PRE_KERNEL_1,
		    CONFIG_CLOCK_CONTROL_INIT_PRIORITY,
		    &bluenrg_clock_control_api);
