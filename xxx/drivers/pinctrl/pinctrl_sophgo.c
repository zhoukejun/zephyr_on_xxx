/*
 * Copyright (c) 2024 Kejun Zhou <zhoukejun@outlook.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT sophgo_pinctrl

#include <stdio.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pinctrl.h>

#include <zephyr/dt-bindings/pinctrl/sophgo-sg200x-pinctrl.h>


#define PINCTRL_BASE_ADDR	DT_INST_REG_ADDR(0)

#define MUX_MASK	(0x7)	
#define MUX_OFFSET	(0)


#define PINMUX_CONFIG(PIN_ID, FUNC_TYPE) \
	sys_write32((sys_read32(PINCTRL_BASE_ADDR + (PIN_ID * 0x4)) & \
		 ~(MUX_MASK << MUX_OFFSET)) | FUNC_TYPE << MUX_OFFSET, \
		PINCTRL_BASE_ADDR + (PIN_ID * 0x4))

static int pinctrl_sophgo_sg200x_set(uint32_t pin, uint32_t type)
{
	PINMUX_CONFIG(pin, type);

	return 0;
}

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt, uintptr_t reg)
{
	ARG_UNUSED(reg);
	int i;

	for (i = 0; i < pin_cnt; i++) {
		pinctrl_sophgo_sg200x_set(pins[i].pin, pins[i].type);
	}

	return 0;
}

