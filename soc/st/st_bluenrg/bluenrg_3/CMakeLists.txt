zephyr_linker_sources(RAM_SECTIONS ram_sections.ld)
zephyr_sources(soc.c)
zephyr_include_directories(${ZEPHYR_BASE}/drivers)
