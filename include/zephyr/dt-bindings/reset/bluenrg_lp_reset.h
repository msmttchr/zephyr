/*
 * Copyright (c) 2022 Google Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_RESET_BLUENRG_LP_RESET_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_RESET_BLUENRG_LP_RESET_H_

#include "bluenrg-common.h"

/* RCC bus reset register offset */
#define BLUENRG_RESET_BUS_AHB0 0x30
#define BLUENRG_RESET_BUS_APB0 0x34
#define BLUENRG_RESET_BUS_APB1 0x38
#define BLUENRG_RESET_BUS_APB2 0x40

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_RESET_BLUENRG_LP_RESET_H_ */
