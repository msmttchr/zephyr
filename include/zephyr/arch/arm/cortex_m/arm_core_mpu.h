/*
 * Copyright (c) 2025, STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_ARCH_ARM_CORTEX_M_ARM_CORE_MPU_H_
#define ZEPHYR_INCLUDE_ARCH_ARM_CORTEX_M_ARM_CORE_MPU_H_

#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>

#ifdef __cplusplus
extern "C" {
#endif

#define Z_ARM_MPU_MAX_REGIONS 16U

/**
 * @brief MPU context structure to retain MPU register state across deep sleep.
 *
 * This structure holds the MPU region base and attribute registers,
 * as well as the MPU control register and a valid region count.
 *
 * The register layout depends on the Cortex-M variant:
 * - Armv8-M MPU (Cortex-M23, M33, M55, M85) uses RBAR and RLAR registers.
 * - Classic MPU (Cortex-M0+, M3, M4, M7) uses RBAR and RASR registers.
 */
struct z_mpu_context_retained {
	uint32_t rbar[Z_ARM_MPU_MAX_REGIONS];
	uint32_t rasr_rlar[Z_ARM_MPU_MAX_REGIONS];
#if defined(CONFIG_CPU_CORTEX_M33) || defined(CONFIG_CPU_CORTEX_M23) || \
	defined(CONFIG_CPU_CORTEX_M55) || defined(CONFIG_CPU_CORTEX_M85)
	uint32_t mair[2];
#elif defined(CONFIG_CPU_CORTEX_M0PLUS) || defined(CONFIG_CPU_CORTEX_M3) || \
	defined(CONFIG_CPU_CORTEX_M4) || defined(CONFIG_CPU_CORTEX_M7)
#endif
	uint32_t ctrl;
	uint32_t valid_count;
};

/**
 * @brief Save the current MPU configuration into the provided context struct.
 *
 * @param ctx Pointer to the MPU context structure to save into.
 */
void z_arm_save_mpu_context(struct z_mpu_context_retained *ctx);

/**
 * @brief Restore the MPU configuration from the provided context struct.
 *
 * @param ctx Pointer to the MPU context structure to restore from.
 */
void z_arm_restore_mpu_context(const struct z_mpu_context_retained *ctx);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_ARCH_ARM_CORTEX_M_ARM_CORE_MPU_H_ */
