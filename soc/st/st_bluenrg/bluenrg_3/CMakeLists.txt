set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/include/zephyr/arch/arm/cortex_m/scripts/linker.ld CACHE INTERNAL "")
zephyr_linker_sources(RAM_SECTIONS ram_sections.ld)
zephyr_sources(soc.c)
zephyr_include_directories(${ZEPHYR_BASE}/drivers)
