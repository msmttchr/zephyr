# SPDX-License-Identifier: Apache-2.0

add_subdirectory(${SOC_SERIES})
zephyr_include_directories(${ZEPHYR_BASE}/soc/st/${SOC_FAMILY}/${SOC_SERIES})

