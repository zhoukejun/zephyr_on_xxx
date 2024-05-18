/*
 * Copyright (c) 2024 Kejun Zhou <zhoukejun@outlook.com>
 * SPDX-License-Identifier: Apache-2.0
 */


#define DT_DRV_COMPAT sophgo_sg200x_sdhc

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sdhc.h>
#include <zephyr/sd/sd_spec.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/irq.h>

#include "sg200x_sdhc.h"

LOG_MODULE_REGISTER(sg200x_sdhc, CONFIG_SDHC_LOG_LEVEL);

enum {
	ERROR_OK,
	ERROR_CMD,
	ERROR_DATA,
};

struct sg200x_sdhc_config {
	volatile uintptr_t *base;
	const struct pinctrl_dev_config *pincfg;
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
	void (*irq_config_func)(const struct device *dev);
	uint32_t data_timeout;
	uint32_t bus_width;
	uint32_t min_bus_freq;
	uint32_t max_bus_freq;
	uint32_t irq_id;
	const struct gpio_dt_spec pwr_en_gpio;
};

struct sg200x_sdhc_data {
	const struct device *dev;
	struct sdhc_io host_io;
	struct k_mutex access_mutex;
	struct k_sem sem_cmd;
	struct k_sem sem_data;
	struct k_event irq_event;
	volatile uint32_t transfer_status;
	enum sdhc_bus_width bus_width;
	enum sdhc_clock_speed clock_speed;
	struct sdhc_host_props props;
	int cmd_error;
	int data_error;
	uint32_t response_type;
	uint32_t response[4];
	bool card_present;
};

static void sdhc_cmd_irq(const struct device *dev, uint32_t intmask)
{
	const struct sg200x_sdhc_config *cfg = dev->config;
	struct sg200x_sdhc_data *data = dev->data;
	int i;

	if (intmask & (SDIF_INT_CMD_TOUT_ERR | SDIF_INT_CMD_CRC_ERR |
		       SDIF_INT_CMD_ENDBIT_ERR | SDIF_INT_CMD_IDX_ERR)) {

		if (intmask & SDIF_INT_CMD_TOUT_ERR) {

			data->cmd_error = -ERROR_CMD;

			LOG_ERR("SDIF_INT_CMD_TOUT_ERR");
		}

		return;
	}

	if (intmask & SDIF_INT_XFER_CMPL) {

		data->cmd_error = ERROR_OK;

		if (data->response_type == SD_RSP_TYPE_R2) {

			/* CRC is stripped so we need to do some shifting. */
			for (i = 0; i < 4; i++) {
				data->response[i] = sys_read32(cfg->base + SDIF_RESPONSE_01 + (3-i)*4) << 8;
				if (i != 3)
					data->response[i] |= sys_read8(cfg->base + SDIF_RESPONSE_01 + (3-i)*4-1);
			}
			LOG_DBG("sdhc->response: [%08x %08x %08x %08x]", data->response[0], data->response[1], data->response[2], data->response[3]);

		} else {
			data->response[0] = sys_read32(cfg->base + SDIF_RESPONSE_01);
			LOG_DBG("data->response: [%08x]", data->response[0]);
		}
	}

	return;
}

static void sdhc_data_irq(const struct device *dev, uint32_t intmask)
{
	const struct sg200x_sdhc_config *cfg = dev->config;
	struct sg200x_sdhc_data *data = dev->data;
	uint32_t command;

	/* CMD19 generates only Buffer Read Ready interrupt */
	if (intmask & SDIF_INT_BUF_RRDY) {
	//orig	command = SDIF_GET_CMD(sys_read16(cfg->base + SDIF_COMMAND));
		command = sys_read32(cfg->base + SDIF_COMMAND) >> 24 & 0x3F;
		if (command == MMC_CMD19 ||
		    command == MMC_CMD21) {
			//host->tuning_done =1;
			return;
		}
	}

	if ((intmask & SDIF_INT_DAT_TOUT_ERR) || (intmask & SDIF_INT_DAT_ENDBIT_ERR) || intmask & SDIF_INT_DAT_CRC_ERR || (intmask & SDIF_INT_ADMA_ERR)) {
		data->data_error = -ERROR_DATA;
		return;
	}

	if (intmask & SDIF_INT_XFER_CMPL) {
		data->data_error = -ERROR_OK;
		return;
	}

	if (intmask & SDIF_INT_DMA_INT) {
		uint64_t dma_addr;
		dma_addr = sys_read32(cfg->base + SDIF_ADMA_SADDR_L);
		sys_write32 (cfg->base + SDIF_ADMA_SADDR_L, dma_addr);
		sys_write32 (cfg->base + SDIF_ADMA_SADDR_H, 0);
	}

	return;
}

static void sg200x_sdhc_isr(const struct device *dev)
{
	const struct sg200x_sdhc_config *cfg = dev->config;
	struct sg200x_sdhc_data *data = dev->data;

	int max_loop = 16;
	uint32_t intmask;
	uint32_t mask;
	uint32_t unexpected;

	intmask = sys_read32(cfg->base + SDIF_NORM_AND_ERR_INT_STS);

	if (!intmask || intmask == 0xFFFFFFFF) {
		LOG_ERR("never be here");
		return;
	}

	do {
		mask = intmask & (SDIF_INT_CMD_MASK | SDIF_INT_DATA_MASK | SDIF_INT_BUS_POWER);
		sys_write32(mask, cfg->base + SDIF_NORM_AND_ERR_INT_STS);

		if (intmask & SDIF_INT_CMD_MASK) {
			sdhc_cmd_irq(dev, intmask & SDIF_INT_CMD_MASK);
			k_sem_give(&data->sem_cmd);
		}

		if (intmask & SDIF_INT_DMA_INT) {
			uint64_t dma_addr;
			dma_addr = sys_read32(cfg->base + SDIF_ADMA_SADDR_L);
			sys_write32(dma_addr, cfg->base + SDIF_ADMA_SADDR_L);
			sys_write32(0, cfg->base + SDIF_ADMA_SADDR_H);
			return;
		}

		if (intmask & SDIF_INT_DATA_MASK) {
			sdhc_data_irq(dev, intmask & SDIF_INT_DATA_MASK);
			k_sem_give(&data->sem_data);
		}

		if (intmask & SDIF_INT_CARD_INT) {
			LOG_DBG("int_card_int");
		}

		intmask &= ~(SDIF_INT_CARD_INSERT_INT | SDIF_INT_CARD_REMOV_INT |
			SDIF_INT_CMD_MASK | SDIF_INT_DATA_MASK |
			SDIF_INT_ERR_INT | SDIF_INT_CURR_LIMIT_ERR |
			SDIF_INT_RE_TUNE_EVENT | SDIF_INT_CARD_INT);

		if (intmask) {
			unexpected = intmask;
			sys_write32(intmask, cfg->base + SDIF_NORM_AND_ERR_INT_STS);
			LOG_DBG("unexpacted interrupt: 0x%08x.", unexpected);
		}

		intmask = sys_read32(cfg->base + SDIF_NORM_AND_ERR_INT_STS);
	} while (intmask && --max_loop);

	return;
}

static void sdhc_enable_card_clock(volatile uintptr_t *base, bool enable)
{
	if (enable) {
		/* TODO */
		sys_write16(sys_read32(base + SDIF_CLK_CTL_SWRST) | (0x1<<2), base + SDIF_CLK_CTL_SWRST); // start SD clock
	} else {
		sys_write16(sys_read32(base + SDIF_CLK_CTL_SWRST) & ~(0x1<<2), base + SDIF_CLK_CTL_SWRST); // stop SD clock
	}
}

static void sg200x_sdhc_hw_reset(volatile uintptr_t *base)
{
	sys_write16(sys_read16(base + SDIF_CLK_CTL_SWRST) & 0x3F | DEFAULT_DIV_SD_INIT_CLOCK << 8, base + SDIF_CLK_CTL_SWRST);
//	sys_write32(sys_read32(base + SDIF_CLK_CTL_SWRST) & 0x3F | DEFAULT_DIV_SD_INIT_CLOCK << 8, base + SDIF_CLK_CTL_SWRST);

	k_msleep(1);

	sys_write8(0x7, base + SDIF_SOFTWARE_RESET);

	while (sys_read8(base + SDIF_SOFTWARE_RESET)){
		LOG_DBG("");
	}
}

static void sg200x_sdhc_phy_init(volatile uintptr_t *base)
{
//	volatile uintptr_t vendor_base = base + (sys_read16(base + P_VENDOR_SPECIFIC_AREA) & ((1 << 12) - 1));
	volatile uintptr_t vendor_base = base + (sys_read32((volatile uintptr_t *)(base + P_VENDOR_SPECIFIC_AREA)) & ((1 << 12) - 1));

	//sg200x_sdhc_hw_reset(base);

	k_msleep(3);

	if (base == EMMC_BASE) {
		//reg_0x200[0] = 1 for sd
		sys_write32(sys_read32(vendor_base) | BIT(0), vendor_base);
	}

	//reg_0x200[1] = 1
	sys_write32(sys_read32(vendor_base) | BIT(1), vendor_base);

	if (base == SD_BASE) {
		//reg_0x200[16] = 1 for sd1
		sys_write32(sys_read32(vendor_base) | BIT(16), vendor_base);
	}

	sys_write32(sys_read32(vendor_base + SDIF_PHY_CONFIG) | BIT(0), vendor_base + SDIF_PHY_CONFIG);

	sys_write32(0x1000100, vendor_base + SDIF_PHY_TX_RX_DLY);
}	

static void sdhc_init(volatile uintptr_t *base)
{
	//orig sys_write8(0x6, base + SDIF_SOFTWARE_RESET);
	sys_write32(SW_RST_CMD | SW_RST_DAT, base + SDIF_CLK_CTL_SWRST);

	sys_write8((0x7 << 1), base + SDIF_PWR_CONTROL);
	sys_write8(base + SDIF_TOUT_CTRL, 0xe);

#if 0
	sys_write16(sys_read16(base + SDIF_HOST_CONTROL2) | 1<<11, base + SDIF_HOST_CONTROL2);

	sys_write16(sys_read16(base + SDIF_CLK_CTL_SWRST) & ~(0x1 << 5), base + SDIF_CLK_CTL_SWRST);

	sys_write16(sys_read16(base + SDIF_HOST_CONTROL2) | SDIF_HOST_VER4_ENABLE, base + SDIF_HOST_CONTROL2);

	sys_write16(sys_read16(base + SDIF_HOST_CONTROL2) | 0x1<<13, base + SDIF_HOST_CONTROL2);

	if (sys_read32(base + SDIF_CAPABILITIES1) & (0x1<<29))
	{
	    sys_write16(sys_read16(base + SDIF_HOST_CONTROL2) | (0x1<<14), base + SDIF_HOST_CONTROL2); // enable async int
	}

	k_msleep(20);

	sys_write16(sys_read16(base + SDIF_HOST_CONTROL2) & ~(0x1<<8), base + SDIF_HOST_CONTROL2); // clr UHS2_IF_ENABLE
	sys_write8(sys_read8(base + SDIF_PWR_CONTROL) | 0x1, base + SDIF_PWR_CONTROL); // set SD_BUS_PWR_VDD1
	sys_write16(sys_read16(base + SDIF_HOST_CONTROL2) & ~0x7, base + SDIF_HOST_CONTROL2); // clr UHS_MODE_SEL

	k_msleep(50);

	sys_write16(sys_read16(base + SDIF_CLK_CTL_SWRST) | (0x1<<2), base + SDIF_CLK_CTL_SWRST); // supply SD clock

	k_usleep(400); // wait for voltage ramp up time at least 74 cycle, 400us is 80 cycles for 200Khz

	sys_write16(sys_read16(base + SDIF_NORM_AND_ERR_INT_STS) | (0x1 << 6), base + SDIF_NORM_AND_ERR_INT_STS);

	sys_write16(sys_read16(base + SDIF_NORM_AND_ERR_INT_STS_EN) | 0xFFFF, base + SDIF_NORM_AND_ERR_INT_STS_EN);
	sys_write16(sys_read16(base + SDIF_ERR_INT_STATUS_EN) | 0xFFFF, base + SDIF_ERR_INT_STATUS_EN);
#endif
}

void sg200x_sdhc_set_config(struct device *dev)
{
	const struct sg200x_sdhc_config *cfg = dev->config;

	uint32_t pio_irqs = SDIF_INT_BUF_RRDY | SDIF_INT_BUF_WRDY;
	uint32_t dma_irqs = SDIF_INT_DMA_INT | SDIF_INT_ADMA_ERR;
	uint32_t int_status;

	static bool sd_clock_state = false;
	static bool sdio_clock_state = false;
	static bool emmc_clock_state = false;

	LOG_DBG("");

	/* TODO ??? to use clock_controller api ???*/
	if (cfg->base == SD_BASE) {
		LOG_DBG("MMC_FLAG_SDCARD");
		if (sd_clock_state == false) {
			sys_write32(MMC_MAX_CLOCK_DIV_VALUE, CLK_DIV_DIV_CLK_SD0);
			sys_clear_bit(CLK_DIV_CLK_BYP_0, BIT(6));
			sd_clock_state = true;
		/* ??? TODO */
		//	ret = clock_control_get_rate(cfg->cdns_clk_dev,
		//			cfg->clkid, &data->params.clk_rate);

		}
	} else if (cfg->base == SDIO_BASE) {
		LOG_DBG("MMC_FLAG_SDIO");
		if (sdio_clock_state == false) {
			sys_write32(MMC_MAX_CLOCK_DIV_VALUE, CLK_DIV_DIV_CLK_SD1);
			sys_clear_bit(CLK_DIV_CLK_BYP_0, BIT(7));
			sdio_clock_state = true;
		}
	} else if (cfg->base == EMMC_BASE) {
		LOG_DBG("MMC_FLAG_EMMC");
		if (emmc_clock_state == false) {
			sys_write32(MMC_MAX_CLOCK_DIV_VALUE, CLK_DIV_DIV_CLK_EMMC);
			sys_clear_bit(CLK_DIV_CLK_BYP_0, BIT(5));
			emmc_clock_state = true;
		}
	}

	sg200x_sdhc_phy_init(cfg->base);
	sdhc_init(cfg->base);

	int_status = SDIF_INT_CURR_LIMIT_ERR | SDIF_INT_DAT_ENDBIT_ERR |
		SDIF_INT_DAT_CRC_ERR | SDIF_INT_DAT_TOUT_ERR |
		SDIF_INT_CMD_IDX_ERR | SDIF_INT_CMD_ENDBIT_ERR | SDIF_INT_CMD_CRC_ERR |
		SDIF_INT_CMD_TOUT_ERR | SDIF_INT_XFER_CMPL | SDIF_INT_CMD_CMPL;

	int_status = (int_status & ~pio_irqs) | dma_irqs;

	if (int_status) {
	/* TODO */
	//	irq_enable(cfg->irq_id);

		sys_write32(int_status, cfg->base + SDIF_NORM_AND_ERR_INT_SIG_EN);
	} else {
	/* TODO */
		sys_write32(0, cfg->base + SDIF_NORM_AND_ERR_INT_SIG_EN);
	}
}

static void sg200x_sdhc_init_host_props(const struct device *dev)
{
	const struct sg200x_sdhc_config *cfg = dev->config;
	struct sg200x_sdhc_data *data = dev->data;
	struct sdhc_host_props *props = &data->props;

	LOG_DBG("");

	memset(props, 0, sizeof(struct sdhc_host_props));

	props->f_min = cfg->min_bus_freq;
	props->f_max = cfg->max_bus_freq;

	props->host_caps.vol_180_support = true;
	props->host_caps.vol_300_support = false;
	props->host_caps.vol_330_support = true;

//???	props->host_caps.sdr12_support = true;
//???	props->host_caps.sdr25_support = true;
	props->host_caps.sdr50_support = true;
	props->host_caps.sdr104_support = true;

	props->host_caps.bus_8_bit_support = false;
	props->host_caps.bus_4_bit_support = true;

}

/*
 * Perform early system init for SDHC
 */
static int sg200x_sdhc_init(const struct device *dev)
{
	const struct sg200x_sdhc_config *cfg = dev->config;
	struct sg200x_sdhc_data *data = dev->data;
	int ret = 0;

	LOG_DBG("");

//	if (!device_is_ready(cfg->clock_dev)) {
//		LOG_ERR("clock control device not ready");
//		return -ENODEV;
//	}

	ret = pinctrl_apply_state(cfg->pincfg, PINCTRL_STATE_DEFAULT);
	if (ret) {
		return ret;
	}

	cfg->irq_config_func(dev);

	/* Read host controller properties */
	sg200x_sdhc_init_host_props(dev);

	/* Set power GPIO low, so card starts powered off */
	if (cfg->pwr_en_gpio.port) {
		ret = gpio_pin_configure_dt(&cfg->pwr_en_gpio, GPIO_OUTPUT_INACTIVE);
		if (ret) {
			return ret;
		}
	}

	data->dev = dev;

	k_mutex_init(&data->access_mutex);
	k_sem_init(&data->sem_cmd, 0, 1);
	k_sem_init(&data->sem_data, 0, 1);
	k_event_init(&data->irq_event);

	sg200x_sdhc_set_config(dev);

	return ret;
}

static int sg200x_sdhc_reset(const struct device *dev)
{
	const struct sg200x_sdhc_config *cfg = dev->config;
	int ret = 0;

	LOG_DBG("");

	return ret;
}

static int sg200x_sdhc_request(const struct device *dev, struct sdhc_command *cmd, struct sdhc_data *data)
{
	const struct sg200x_sdhc_config *cfg = dev->config;
	int ret = 0;

	LOG_DBG("");

	/* TODO */
	if (data) {
		LOG_DBG("cmd->opcode: 0x%08x", cmd->opcode);
		switch (cmd->opcode) {
		case SD_WRITE_SINGLE_BLOCK:
			break;
		case SD_WRITE_MULTIPLE_BLOCK:
			break;
		default:
			break;
		}
	}

	return ret;
}

static int sg200x_sdhc_set_io(const struct device *dev, struct sdhc_io *ios)
{
	const struct sg200x_sdhc_config *cfg = dev->config;
	struct sg200x_sdhc_data *data = dev->data;
	struct sdhc_io *host_io = &data->host_io;
	uint32_t src_clk_hz, bus_clk;

	LOG_DBG("");
	LOG_DBG("SDHC I/O: bus width %d, clock %dHz, card power %s, voltage %s",
		ios->bus_width, ios->clock,
		ios->power_mode == SDHC_POWER_ON ? "ON" : "OFF",
		ios->signal_voltage == SD_VOL_1_8_V ? "1.8V" : "3.3V");

	if (clock_control_get_rate(cfg->clock_dev, cfg->clock_subsys,
				   &src_clk_hz)) {
		LOG_ERR("Failed to call clock_control_get_rate()");
		return -EINVAL;
	}

	if (ios->clock && (ios->clock > data->props.f_max || ios->clock < data->props.f_min)) {
		LOG_ERR("Invalid argument for clock freq: %d Support max:%d and Min:%d", ios->clock,
			data->props.f_max, data->props.f_min);
		return -EINVAL;
	}

	/* Set SDHC clock */
	if (host_io->clock != ios->clock) {
		LOG_DBG("Clock: %d", host_io->clock);
		if (ios->clock != 0) {
			/* Enable clock */
			LOG_DBG("CLOCK: %d", ios->clock);
//TODO			if (!sg200x_sdhc_clock_set(dev, ios->clock)) {
//				return -ENOTSUP;
//			}
		} else {
//TODO			sg200x_sdhc_disable_clock(dev);
		}
		host_io->clock = ios->clock;
	}

	/* TODO */
	/* Set bus width */
	if (host_io->bus_width != ios->bus_width) {
		switch (ios->bus_width) {
		case SDHC_BUS_WIDTH1BIT:
			LOG_DBG("Set to 1 bit bus width!");
			break;
		case SDHC_BUS_WIDTH4BIT:
			LOG_DBG("Set to 4 bit bus width!");
			break;
		case SDHC_BUS_WIDTH8BIT:
			LOG_DBG("Not support 8 bit bus width!");
			break;
		default:
			return -ENOTSUP;
		}
		host_io->bus_width = ios->bus_width;
	}

	/* Set card power */
	if ((host_io->power_mode != ios->power_mode) && (cfg->pwr_en_gpio.port)) {
		if (ios->power_mode == SDHC_POWER_OFF) {
			gpio_pin_set_dt(&cfg->pwr_en_gpio, 0);
		} else if (ios->power_mode == SDHC_POWER_ON) {
			gpio_pin_set_dt(&cfg->pwr_en_gpio, 1);
		}
		host_io->power_mode = ios->power_mode;
	}

	return 0;
}

static int sg200x_sdhc_get_card_present(const struct device *dev)
{
	const struct sg200x_sdhc_config *cfg = dev->config;
	int ret = -1;
	uint32_t status;

	LOG_DBG("");
	
	status = sys_read32(cfg->base + SDIF_PRESENT_STS);
	if (status == SDIF_CARD_STABLE) {
		LOG_DBG("CARD_STABLE");
		ret = 0;
	}

	if (status == SDIF_CARD_INSERTED) {
		LOG_DBG("CARD_INSERTED");
		ret = 0;
	}

	return ret;
}

static int sg200x_sdhc_execute_tuning(const struct device *dev)
{
	LOG_DBG("");

	return 0;
}

static int sg200x_sdhc_card_busy(const struct device *dev)
{
	LOG_DBG("");

	return 0;
}

static int sg200x_sdhc_get_host_props(const struct device *dev,
				      struct sdhc_host_props *props)
{
	LOG_DBG("");
	const struct sg200x_sdhc_config *config = dev->config;
	struct sg200x_sdhc_data *data = dev->data;

	memcpy(props, &data->props, sizeof(struct sdhc_host_props));
	return 0;
}

static int sg200x_sdhc_enable_interrupt(const struct device *dev,
					sdhc_interrupt_cb_t callback,
					int sources, void *user_data)
{
	LOG_DBG("");
//	const struct sg200x_sdhc_config *config = dev->config;
	struct sg200x_sdhc_data *data = dev->data;

	return 0;
}

static int sg200x_sdhc_disable_interrupt(const struct device *dev, int sources)
{
	LOG_DBG("");
	const struct sg200x_sdhc_config *config = dev->config;
	struct sg200x_sdhc_data *data = dev->data;

	return 0;
}

static const struct sdhc_driver_api sdhc_api = {
	.reset = sg200x_sdhc_reset,
	.request = sg200x_sdhc_request,
	.set_io = sg200x_sdhc_set_io,
	.get_card_present = sg200x_sdhc_get_card_present,
	.execute_tuning = sg200x_sdhc_execute_tuning,
	.card_busy = sg200x_sdhc_card_busy,
	.get_host_props = sg200x_sdhc_get_host_props,
	.enable_interrupt = sg200x_sdhc_enable_interrupt,
	.disable_interrupt = sg200x_sdhc_disable_interrupt,
};



#define SG200X_SDHC_INIT(n)							\
	static void sdhc_##n##_irq_config_func(const struct device *dev)		\
	{									\
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority),		\
			sg200x_sdhc_isr, DEVICE_DT_INST_GET(n), 0);		\
		irq_enable(DT_INST_IRQN(n));					\
	}									\
										\
	PINCTRL_DT_INST_DEFINE(n);						\
										\
	static const struct sg200x_sdhc_config sdhc_##n##_config = {		\
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),			\
		.base = (uintptr_t *) DT_INST_REG_ADDR(n),			\
	/*	.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)), */		\
	/*	.clock_subsys = 						\
			(clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, name),*/	\
		.pwr_en_gpio = GPIO_DT_SPEC_INST_GET_OR(n, pwr_en_gpios, {0}),	\
		.bus_width = DT_INST_PROP(n, bus_width),			\
		.min_bus_freq = DT_INST_PROP_OR(n, min_frequency, 400000),	\
		.max_bus_freq = DT_INST_PROP_OR(n, max_frequency, 50*1000*1000),\
		.irq_config_func = sdhc_##n##_irq_config_func,			\
		.irq_id = DT_INST_IRQN(n),					\
	};									\
										\
	static struct sg200x_sdhc_data sdhc_##n##_data = {			\
		.card_present = false,						\
	};									\
										\
	DEVICE_DT_INST_DEFINE(n,						\
			&sg200x_sdhc_init,					\
			NULL,							\
			&sdhc_##n##_data,					\
			&sdhc_##n##_config,					\
			POST_KERNEL,						\
			CONFIG_SDHC_INIT_PRIORITY,				\
			&sdhc_api);

DT_INST_FOREACH_STATUS_OKAY(SG200X_SDHC_INIT)
