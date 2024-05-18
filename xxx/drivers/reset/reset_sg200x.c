/*
 * Copyright (c) 2024 Kejun Zhou <zhoukejun@outlook.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT sophgo_sg200x_reset

#include <zephyr/arch/cpu.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/reset.h>


#define SG200X_RESET_SET_OFFSET(id)	(0x4 * (id / 32))
#define SG200X_RESET_REG_BIT(id)	(0x1 << (id / 32))

struct reset_sg200x_config {
	uintptr_t base;
};

static int reset_sg200x_status(const struct device *dev, uint32_t id,
			      uint8_t *status)
{
	const struct reset_sg200x_config *config = dev->config;

	*status = !!sys_test_bit(config->base + SG200X_RESET_SET_OFFSET(id),
				 SG200X_RESET_REG_BIT(id));

	return 0;
}

static int reset_sg200x_line_assert(const struct device *dev, uint32_t id)
{
	const struct reset_sg200x_config *config = dev->config;

	sys_clear_bit(config->base + SG200X_RESET_SET_OFFSET(id),
		      SG200X_RESET_REG_BIT(id));

	return 0;
}

static int reset_sg200x_line_deassert(const struct device *dev, uint32_t id)
{
	const struct reset_sg200x_config *config = dev->config;

	sys_set_bit(config->base + SG200X_RESET_SET_OFFSET(id),
		    SG200X_RESET_REG_BIT(id));

	return 0;
}

static int reset_sg200x_line_toggle(const struct device *dev, uint32_t id)
{
	reset_sg200x_line_assert(dev, id);
	reset_sg200x_line_deassert(dev, id);

	return 0;
}

static const struct reset_driver_api reset_sg200x_driver_api = {
	.status = reset_sg200x_status,
	.line_assert = reset_sg200x_line_assert,
	.line_deassert = reset_sg200x_line_deassert,
	.line_toggle = reset_sg200x_line_toggle,
};

static const struct reset_sg200x_config reset_sg200x_config = {
	.base = DT_INST_REG_ADDR(0),
};

DEVICE_DT_INST_DEFINE(0, NULL, NULL, NULL, &reset_sg200x_config, PRE_KERNEL_1,
		      CONFIG_RESET_INIT_PRIORITY, &reset_sg200x_driver_api);
