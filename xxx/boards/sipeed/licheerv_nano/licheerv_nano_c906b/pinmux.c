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

static int licheerv_nano_c906b_pinmux_init(void)
{
	PINMUX_CONFIG(FMUX_REG_SD0_PWR_EN, SD0_PWR_EN__XGPIOA_14);
	PINMUX_CONFIG(FMUX_REG_AUX0, AUX0__XGPIOA_30);
	return 0;
}

SYS_INIT(licheerv_nano_c906b_pinmux_init, EARLY,
		CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
