# This sets the default runner to openocd specifically for this board
message(STATUS "DEBUG: Loading my custom runner settings for ${BOARD}")
set(BOARD_FLASH_RUNNER openocd CACHE STRING "" FORCE)
set(BOARD_DEBUG_RUNNER openocd CACHE STRING "" FORCE)
