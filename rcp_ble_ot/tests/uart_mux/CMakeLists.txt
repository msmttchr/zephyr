# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

# Add the parent directory to the DTS search path
# CMAKE_CURRENT_SOURCE_DIR is the folder containing this CMakeLists.txt
set(DTS_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../..)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(rcp_ble_ot)

zephyr_include_directories(${CMAKE_CURRENT_LIST_DIR}/../../src)

target_sources(app PRIVATE
  ../../src/uart_mux.c
  src/test_rx.c
  src/test_tx.c
  src/fake_uart.c
)

