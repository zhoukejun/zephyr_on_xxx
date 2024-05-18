/*
 * Copyright (c) 2024 Kejun Zhou <zhoukejun@outlook.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_SOC_SOPHGO_COMMON_PINCTRL_SOC_H_
#define ZEPHYR_SOC_SOPHGO_COMMON_PINCTRL_SOC_H_

#include <zephyr/devicetree.h>
#include <zephyr/types.h>

typedef struct pinctrl_soc_pin_t {
	uint8_t pin;
	uint8_t type;
} pinctrl_soc_pin_t;

#define SOPHGO_DT_PIN(node_id)					\
	{							\
		.pin = DT_PROP_BY_IDX(node_id, pinmux, 0),	\
		.type = DT_PROP_BY_IDX(node_id, pinmux, 1)	\
	},

#define Z_PINCTRL_STATE_PIN_INIT(node_id, prop, idx)		\
	SOPHGO_DT_PIN(DT_PROP_BY_IDX(node_id, prop, idx))

#define Z_PINCTRL_STATE_PINS_INIT(node_id, prop)		\
	{ DT_FOREACH_PROP_ELEM(node_id, prop, Z_PINCTRL_STATE_PIN_INIT) }

#endif /* ZEPHYR_SOC_SOPHGO_COMMON_PINCTRL_SOC_H_ */

