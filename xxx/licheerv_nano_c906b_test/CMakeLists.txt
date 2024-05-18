# Copyright (c) 2024 Kejun Zhou <zhoukejun@outlook.com>
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

set(BOARD licheerv_nano_c906b)

list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/..)
list(APPEND SOC_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/..)
list(APPEND DTS_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/..)
list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/../drivers)

# Find Zephyr. This also loads Zephyr's build system.
find_package(Zephyr REQUIRED HINTS ../zephyr})

project(licheerv_nano_c906b_test)

#zephyr_library_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../include)

target_sources(app PRIVATE src/main.c)
