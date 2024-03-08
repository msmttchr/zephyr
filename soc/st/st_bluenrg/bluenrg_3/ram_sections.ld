/*
 * Copyright 2023 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

RAM_VR_section : {
	. += 4;
	KEEP(*(RAM_VR_section));
} > RAM

FILL_GAP_section : {
	. = ALIGN(4);
	KEEP(*(FILL_GAP_section));
} > RAM

BLUE_RAM_section : {
	. = ALIGN(4);
	KEEP(*(BLUE_RAM_section));
} > RAM
