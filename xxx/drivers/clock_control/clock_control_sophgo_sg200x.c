/*
 * Copyright (c) 2024 Kejun Zhou <zhoukejun@outlook.com> 
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT sophgo_sg200x_clock

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/clock_control.h>

LOG_MODULE_REGISTER(clock_control, CONFIG_CLOCK_CONTROL_LOG_LEVEL);

static int sophgo_sg200x_clock_on(const struct device *dev,
			    clock_control_subsys_t sub_system)
{

	return 0;
}

static int sophgo_sg200x_clock_off(const struct device *dev,
			     clock_control_subsys_t sub_system)
{

	return 0;
}

static int sophgo_sg200x_clock_get_rate(const struct device *dev,
				  clock_control_subsys_t sub_system,
				  uint32_t rate)
{

	return 0;
}

static int sophgo_sg200x_clock_set_rate(const struct device *dev,
				  clock_control_subsys_t sub_system,
				   clock_control_subsys_rate_t rate)
{

	return 0;
}

static int sophgo_sg200x_clock_init(const struct device *dev)
{
	return 0;
}

static const struct clock_control_driver_api sophgo_sg200x_clock_driver_api = {
	.on = sophgo_sg200x_clock_on,
	.off = sophgo_sg200x_clock_off,
	.get_rate = sophgo_sg200x_clock_get_rate,
	.set_rate = sophgo_sg200x_clock_get_rate,
};

DEVICE_DT_DEFINE(DT_NODELABEL(clk),
		      &sophgo_sg200x_clock_init,
		      NULL, NULL, NULL,
		      PRE_KERNEL_1, CONFIG_CLOCK_CONTROL_INIT_PRIORITY,
		      &sophgo_sg200x_clock_driver_api);
