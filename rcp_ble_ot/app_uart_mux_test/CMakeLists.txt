# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

# Add the parent directory to the DTS search path
# CMAKE_CURRENT_SOURCE_DIR is the folder containing this CMakeLists.txt
set(DTS_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/..)

# Manually include the board-specific runner settings
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/boards/${BOARD}.cmake")
    include("${CMAKE_CURRENT_SOURCE_DIR}/boards/${BOARD}.cmake")
endif()

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(uart_mux_test)

target_sources(app PRIVATE
	../src/uart_mux.c
	../src/log_backend_uart_mux.c
	src/main.c
)

target_include_directories(app PRIVATE
	../src
)

