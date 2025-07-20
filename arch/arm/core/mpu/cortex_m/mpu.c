/*
 * Copyright 2025 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/arch/arm/cortex_m/arm_core_mpu.h>

#define LOG_LEVEL CONFIG_MPU_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(mpu_cortex_m);

#if defined(CONFIG_CPU_CORTEX_M33) || defined(CONFIG_CPU_CORTEX_M23) || \
	defined(CONFIG_CPU_CORTEX_M55) || defined(CONFIG_CPU_CORTEX_M85)
	/* Armv8-M MPU: save RBAR and RLAR */
#define ATTRIBUTE_AND_SIZE_REG_NAME RLAR
#elif defined(CONFIG_CPU_CORTEX_M0PLUS) || defined(CONFIG_CPU_CORTEX_M3) || \
	defined(CONFIG_CPU_CORTEX_M4) || defined(CONFIG_CPU_CORTEX_M7)
	/* Classic MPU: save RBAR and RASR */
#define ATTRIBUTE_AND_SIZE_REG_NAME RASR
#else
#error "Unsupported CortexM version"
#endif


extern uint32_t get_num_regions(void);

#if defined(CONFIG_CPU_HAS_ARM_MPU)
/**
 * @brief Save the current MPU configuration into the provided context struct.
 */
void z_arm_save_mpu_context(struct z_mpu_context_retained *ctx)
{
	uint32_t regions = get_num_regions();

	if (regions == 0 || regions > Z_ARM_MPU_MAX_REGIONS) {
		LOG_DBG("Invalid MPU region count: %u", regions);
		ctx->valid_count = 0;
		return;
	}

	ctx->valid_count = regions;

	for (uint32_t i = 0; i < regions; i++) {
		MPU->RNR = i;
		__DSB(); /* Ensure MPU->RNR write completes before reading registers */
		__ISB();
		ctx->rbar[i] = MPU->RBAR;
		ctx->rasr_rlar[i] = MPU->ATTRIBUTE_AND_SIZE_REG_NAME;
#if defined(CONFIG_CPU_CORTEX_M33) || defined(CONFIG_CPU_CORTEX_M23) || \
	defined(CONFIG_CPU_CORTEX_M55) || defined(CONFIG_CPU_CORTEX_M85)
		ctx->mair[0] = MPU->MAIR0;
		ctx->mair[1] = MPU->MAIR1;
#endif
	}
	ctx->ctrl = MPU->CTRL;
}

/**
 * @brief Restore the MPU configuration from the provided context struct.
 */
void z_arm_restore_mpu_context(const struct z_mpu_context_retained *ctx)
{
	if (ctx->valid_count == 0) {
		LOG_DBG("Invalid MPU context valid_count: %u", ctx->valid_count);
		return;
	}

	/* Disable MPU before reprogramming */
	MPU->CTRL = 0;
	__DSB();
	__ISB();

	for (uint32_t i = 0; i < ctx->valid_count; i++) {
		MPU->RNR = i;
		MPU->RBAR = ctx->rbar[i];
		MPU->ATTRIBUTE_AND_SIZE_REG_NAME = ctx->rasr_rlar[i];
#if defined(CONFIG_CPU_CORTEX_M33) || defined(CONFIG_CPU_CORTEX_M23) || \
	defined(CONFIG_CPU_CORTEX_M55) || defined(CONFIG_CPU_CORTEX_M85)
		MPU->MAIR0 = ctx->mair[0];
		MPU->MAIR1 = ctx->mair[1];
#endif
	}

	/* Restore MPU control register (including enable bit if set) */
	MPU->CTRL = ctx->ctrl;

	/* Ensure MPU settings take effect before continuing */
	__DSB();
	__ISB();
}

#endif /* CONFIG_CPU_HAS_ARM_MPU */
