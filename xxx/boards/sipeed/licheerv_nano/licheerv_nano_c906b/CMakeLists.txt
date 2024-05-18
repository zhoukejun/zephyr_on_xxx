# Copyright (c) 2024 Kejun Zhou <zhoukejun@outlook.com>
# SPDX-License-Identifier: Apache-2.0

zephyr_library_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../../../include)

zephyr_library_sources(uart0.c)
zephyr_library_sources(pinmux.c)
