# Copyright (c) 2024 Kejun Zhou <zhoukejun@outlook.com>
# SPDX-License-Identifier: Apache-2.0

zephyr_library_sources_ifdef(CONFIG_RISCV_MACHINE_TIMER_FOR_THEAD riscv_machine_timer.c)
