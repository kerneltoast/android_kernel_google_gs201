// SPDX-License-Identifier: GPL-2.0-only
/**
 * i2c-exynos5.c - Samsung Exynos5 I2C Controller Driver
 *
 * Copyright (C) 2013 Samsung Electronics Co., Ltd.
 */

#include <linux/kernel.h>
#include <linux/module.h>

#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/time.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include "i2c-exynos5.h"

#include <soc/google/exynos-cpupm.h>

/*
 * HSI2C controller from Samsung supports 2 modes of operation
 * 1. Auto mode: Where in master automatically controls the whole transaction
 * 2. Manual mode: Software controls the transaction by issuing commands
 *    START, READ, WRITE, STOP, RESTART in I2C_MANUAL_CMD register.
 *
 * Operation mode can be selected by setting AUTO_MODE bit in I2C_CONF register
 *
 * Special bits are available for both modes of operation to set commands
 * and for checking transfer status
 */

/* Register Map */
#define HSI2C_CTL		0x00
#define HSI2C_FIFO_CTL		0x04
#define HSI2C_TRAILIG_CTL	0x08
#define HSI2C_CLK_CTL		0x0C
#define HSI2C_CLK_SLOT		0x10
#define HSI2C_INT_ENABLE	0x20
#define HSI2C_INT_STATUS	0x24
#define HSI2C_ERR_STATUS	0x2C
#define HSI2C_FIFO_STATUS	0x30
#define HSI2C_TX_DATA		0x34
#define HSI2C_RX_DATA		0x38
#define HSI2C_CONF		0x40
#define HSI2C_AUTO_CONF		0x44
#define HSI2C_TIMEOUT		0x48
#define HSI2C_MANUAL_CMD	0x4C
#define HSI2C_TRANS_STATUS	0x50
#define HSI2C_TIMING_HS1	0x54
#define HSI2C_TIMING_HS2	0x58
#define HSI2C_TIMING_HS3	0x5C
#define HSI2C_TIMING_FS1	0x60
#define HSI2C_TIMING_FS2	0x64
#define HSI2C_TIMING_FS3	0x68
#define HSI2C_TIMING_SLA	0x6C
#define HSI2C_ADDR		0x70

/* I2C_CTL Register bits */
#define HSI2C_FUNC_MODE_I2C			BIT(0)
#define HSI2C_MASTER				BIT(3)
#define HSI2C_RXCHON				BIT(6)
#define HSI2C_TXCHON				BIT(7)
#define HSI2C_NO_LOSE_ARBITRATION		BIT(22)
#define HSI2C_EXT_MSB				BIT(29)
#define HSI2C_EXT_ADDR				BIT(30)
#define HSI2C_SW_RST				BIT(31)

/* I2C_FIFO_CTL Register bits */
#define HSI2C_RXFIFO_EN				BIT(0)
#define HSI2C_TXFIFO_EN				BIT(1)
#define HSI2C_FIFO_MAX				(0x40)
#define HSI2C_RXFIFO_TRIGGER_LEVEL(x)	((x) << 4)
#define HSI2C_TXFIFO_TRIGGER_LEVEL(x)	((x) << 16)
/* I2C_TRAILING_CTL Register bits */
#define HSI2C_TRAILING_COUNT			(0xffffff)

/* I2C_INT_EN Register bits */
#define HSI2C_INT_TX_ALMOSTEMPTY_EN		BIT(0)
#define HSI2C_INT_RX_ALMOSTFULL_EN		BIT(1)
#define HSI2C_INT_TRAILING_EN			BIT(6)
#define HSI2C_INT_TRANSFER_DONE			BIT(7)
#define HSI2C_INT_I2C_EN			BIT(9)
#define HSI2C_INT_CHK_TRANS_STATE		(0xf << 8)

/* I2C_INT_STAT Register bits */
#define HSI2C_INT_TX_ALMOSTEMPTY		BIT(0)
#define HSI2C_INT_RX_ALMOSTFULL			BIT(1)
#define HSI2C_INT_TX_UNDERRUN			BIT(2)
#define HSI2C_INT_TX_OVERRUN			BIT(3)
#define HSI2C_INT_RX_UNDERRUN			BIT(4)
#define HSI2C_INT_RX_OVERRUN			BIT(5)
#define HSI2C_INT_TRAILING			BIT(6)
#define HSI2C_INT_I2C				BIT(9)
#define HSI2C_INT_NODEV				BIT(10)
#define HSI2C_RX_INT				(HSI2C_INT_RX_ALMOSTFULL | \
						 HSI2C_INT_RX_UNDERRUN | \
						 HSI2C_INT_RX_OVERRUN | \
						 HSI2C_INT_TRAILING)

/* I2C_FIFO_STAT Register bits */
#define HSI2C_RX_FIFO_EMPTY			BIT(24)
#define HSI2C_RX_FIFO_FULL			BIT(23)
#define HSI2C_RX_FIFO_LVL(x)			(((x) >> 16) & 0x7f)
#define HSI2C_RX_FIFO_LVL_MASK			(0x7F << 16)
#define HSI2C_TX_FIFO_EMPTY			BIT(8)
#define HSI2C_TX_FIFO_FULL			BIT(7)
#define HSI2C_TX_FIFO_LVL(x)			(((x) >> 0) & 0x7f)
#define HSI2C_TX_FIFO_LVL_MASK			(0x7F << 0)
#define HSI2C_FIFO_EMPTY			(HSI2C_RX_FIFO_EMPTY |	\
						HSI2C_TX_FIFO_EMPTY)

/* I2C_CONF Register bits */
#define HSI2C_AUTO_MODE				BIT(31)
#define HSI2C_10BIT_ADDR_MODE			BIT(30)
#define HSI2C_HS_MODE				BIT(29)
#define HSI2C_FILTER_EN_SCL			BIT(28)
#define HSI2C_FILTER_EN_SDA			BIT(27)
#define HSI2C_FTL_CYCLE_SCL_MASK		(0x7 << 16)
#define HSI2C_FTL_CYCLE_SDA_MASK		(0x7 << 13)

/* I2C_AUTO_CONF Register bits */
#define HSI2C_READ_WRITE			BIT(16)
#define HSI2C_STOP_AFTER_TRANS			BIT(17)
#define HSI2C_MASTER_RUN			BIT(31)

/* I2C_TIMEOUT Register bits */
#define HSI2C_TIMEOUT_EN			BIT(31)

/* I2C_MANUAL_CMD register bits */
#define HSI2C_CMD_READ_DATA			BIT(4)
#define HSI2C_CMD_SEND_STOP			BIT(2)

/* I2C_TRANS_STATUS register bits */
#define HSI2C_MASTER_BUSY			BIT(17)
#define HSI2C_SLAVE_BUSY			BIT(16)

/* I2C_TRANS_STATUS register bits for Exynos5 variant */
#define HSI2C_TIMEOUT_AUTO			BIT(4)
#define HSI2C_NO_DEV				BIT(3)
#define HSI2C_NO_DEV_ACK			BIT(2)
#define HSI2C_TRANS_ABORT			BIT(1)
#define HSI2C_TRANS_DONE			BIT(0)
#define HSI2C_MAST_ST_MASK			(0xf << 0)
#define HSI2C_MASTER_ST_INIT			(0x1)

/* I2C_TRANS_STATUS register bits for Exynos7 variant */
#define HSI2C_MASTER_ST_MASK			0xf
#define HSI2C_MASTER_ST_IDLE			0x0
#define HSI2C_MASTER_ST_START			0x1
#define HSI2C_MASTER_ST_RESTART			0x2
#define HSI2C_MASTER_ST_STOP			0x3
#define HSI2C_MASTER_ST_MASTER_ID		0x4
#define HSI2C_MASTER_ST_ADDR0			0x5
#define HSI2C_MASTER_ST_ADDR1			0x6
#define HSI2C_MASTER_ST_ADDR2			0x7
#define HSI2C_MASTER_ST_ADDR_SR			0x8
#define HSI2C_MASTER_ST_READ			0x9
#define HSI2C_MASTER_ST_WRITE			0xa
#define HSI2C_MASTER_ST_NO_ACK			0xb
#define HSI2C_MASTER_ST_LOSE			0xc
#define HSI2C_MASTER_ST_WAIT			0xd
#define HSI2C_MASTER_ST_WAIT_CMD		0xe

/* I2C_ADDR register bits */
#define HSI2C_SLV_ADDR_SLV(x)			(((x) & 0x3ff) << 0)
#define HSI2C_SLV_ADDR_MAS(x)			(((x) & 0x3ff) << 10)
#define HSI2C_MASTER_ID(x)			(((x) & 0xff) << 24)
#define MASTER_ID(x)			      (((((x) << 1) + 0x1) & 0x7) + 0x8)

/*
 * Controller operating frequency, timing values for operation
 * are calculated against this frequency
 */
#define HSI2C_HS_TX_CLOCK			3400000
#define HSI2C_FAST_PLUS_TX_CLOCK	1000000
#define HSI2C_FS_PLUS_HIGH_TX_CLOCK	900000
#define HSI2C_FS_TX_CLOCK			400000
#define HSI2C_STAND_TX_CLOCK		100000

#define HSI2C_STAND_SPD			3
#define HSI2C_FAST_PLUS_SPD		2
#define HSI2C_HIGH_SPD			1
#define HSI2C_FAST_SPD			0

#define HSI2C_POLLING 0
#define HSI2C_INTERRUPT 1

#define EXYNOS5_I2C_TIMEOUT (msecs_to_jiffies(100))
#define EXYNOS5_FIFO_SIZE		16

#define EXYNOS5_HSI2C_RUNTIME_PM_DELAY	(100)

#define USI_CON				(0xC4)
#define USI_OPTION			(0xC8)

#define USI_RESET			(0 << 0)
#define USI_HWACG_CLKREQ_ON		BIT(1)
#define USI_HWACG_CLKSTOP_ON		BIT(2)

#define FIFO_TRIG_CRITERIA	(8)

#define USI_SW_CONF_MASK	(0x7 << 0)
#define USI_I2C_SW_CONF		BIT(2)

static const struct of_device_id exynos5_i2c_match[] = {
	{ .compatible = "samsung,exynos5-hsi2c" },
	{},
};
MODULE_DEVICE_TABLE(of, exynos5_i2c_match);

#ifdef CONFIG_GPIOLIB
static void change_i2c_gpio(struct exynos5_i2c *i2c)
{
	struct pinctrl_state *default_i2c_pins;
	struct pinctrl *default_i2c_pinctrl;
	int status = 0;

	default_i2c_pinctrl = devm_pinctrl_get(i2c->dev);
	if (IS_ERR(default_i2c_pinctrl)) {
		dev_err(i2c->dev, "Can't get i2c pinctrl!!!\n");
		return;
	}

	default_i2c_pins = pinctrl_lookup_state(default_i2c_pinctrl,
						"default");
	if (!IS_ERR(default_i2c_pins)) {
		status = pinctrl_select_state(default_i2c_pinctrl,
					      default_i2c_pins);
		if (status)
			dev_err(i2c->dev, "Can't set default i2c pins!!!\n");
	} else {
		dev_err(i2c->dev, "Can't get default pinstate!!!\n");
	}
}

static void recover_gpio_pins(struct exynos5_i2c *i2c)
{
	int gpio_sda, gpio_scl;
	int sda_val, scl_val, clk_cnt;
	unsigned long timeout;
	struct device_node *np = i2c->adap.dev.of_node;

	dev_err(i2c->dev, "Recover GPIO pins\n");

	gpio_sda = of_get_named_gpio(np, "gpio_sda", 0);
	if (!gpio_is_valid(gpio_sda)) {
		dev_err(i2c->dev, "Can't get gpio_sda!!!\n");
		return;
	}
	gpio_scl = of_get_named_gpio(np, "gpio_scl", 0);
	if (!gpio_is_valid(gpio_scl)) {
		dev_err(i2c->dev, "Can't get gpio_scl!!!\n");
		return;
	}

	sda_val = gpio_get_value(gpio_sda);
	scl_val = gpio_get_value(gpio_scl);

	dev_err(i2c->dev, "SDA line : %s, SCL line : %s\n",
		sda_val ? "HIGH" : "LOW", scl_val ? "HIGH" : "LOW");

	if (sda_val == 1)
		return;

	/* Wait for SCL as high for 500msec */
	if (scl_val == 0) {
		timeout = jiffies + msecs_to_jiffies(500);
		while (time_before(jiffies, timeout)) {
			if (gpio_get_value(gpio_scl) != 0) {
				timeout = 0;
				break;
			}
			usleep_range(10000, 20000);
		}
		if (timeout)
			dev_err(i2c->dev, "SCL line is still LOW!!!\n");
	}

	sda_val = gpio_get_value(gpio_sda);

	if (sda_val == 0) {
		gpio_direction_output(gpio_scl, 1);
		gpio_direction_input(gpio_sda);

		for (clk_cnt = 0; clk_cnt < 100; clk_cnt++) {
			/* Make clock for slave */
			gpio_set_value(gpio_scl, 0);
			udelay(5);
			gpio_set_value(gpio_scl, 1);
			udelay(5);
			if (gpio_get_value(gpio_sda) == 1) {
				dev_err(i2c->dev, "SDA line is recovered.\n");
				break;
			}
		}
		if (clk_cnt == 100)
			dev_err(i2c->dev, "SDA line is not recovered!!!\n");
	}

	/* Change I2C GPIO as default function */
	change_i2c_gpio(i2c);
}
#endif

static inline void dump_i2c_register(struct exynos5_i2c *i2c)
{
	dev_err(i2c->dev, "Register dump(suspended : %d)\n"
		"CTL          0x%08x   FIFO_CTL     0x%08x   INT_EN       0x%08x   INT_STAT     0x%08x\n"
		"FIFO_STAT    0x%08x   CONF         0x%08x   CONF2        0x%08x   TRANS_STAT   0x%08x\n"
		"TIMING_HS1   0x%08x   TIMING_HS2   0x%08x   TIMING_HS3   0x%08x   TIMING_FS1   0x%08x\n"
		"TIMING_FS2   0x%08x   TIMING_FS3   0x%08x   TRAILING_CTL 0x%08x   ADDR         0x%08x\n"
		, i2c->suspended
		, readl(i2c->regs + HSI2C_CTL)
		, readl(i2c->regs + HSI2C_FIFO_CTL)
		, readl(i2c->regs + HSI2C_INT_ENABLE)
		, readl(i2c->regs + HSI2C_INT_STATUS)
		, readl(i2c->regs + HSI2C_FIFO_STATUS)
		, readl(i2c->regs + HSI2C_CONF)
		, readl(i2c->regs + HSI2C_AUTO_CONF)
		, readl(i2c->regs + HSI2C_TRANS_STATUS)
		, readl(i2c->regs + HSI2C_TIMING_HS1)
		, readl(i2c->regs + HSI2C_TIMING_HS2)
		, readl(i2c->regs + HSI2C_TIMING_HS3)
		, readl(i2c->regs + HSI2C_TIMING_FS1)
		, readl(i2c->regs + HSI2C_TIMING_FS2)
		, readl(i2c->regs + HSI2C_TIMING_FS3)
		, readl(i2c->regs + HSI2C_TRAILIG_CTL)
		, readl(i2c->regs + HSI2C_ADDR)
	);

#ifdef CONFIG_GPIOLIB
	recover_gpio_pins(i2c);
#endif
}

static void exynos5_i2c_clr_pend_irq(struct exynos5_i2c *i2c)
{
	writel(readl(i2c->regs + HSI2C_INT_STATUS),
	       i2c->regs + HSI2C_INT_STATUS);
}

/*
 * exynos5_i2c_set_timing: updates the registers with appropriate
 * timing values calculated
 *
 * Timing values for operation are calculated against either 100kHz
 * or 1MHz controller operating frequency.
 *
 * Returns 0 on success, -EINVAL if the cycle length cannot
 * be calculated.
 */
static int exynos5_i2c_set_timing(struct exynos5_i2c *i2c, int mode)
{
	int ret;
	unsigned int ipclk;
	unsigned int op_clk;

	u32 hs_div, utscl_h_hs, utscl_l_hs, utstart_hd_hs;
	u32 fs_div, utscl_h_fs, utscl_l_fs, utstart_hd_fs;
	u32 utemp;

	if (i2c->default_clk) {
		ret = clk_set_rate(i2c->rate_clk, i2c->default_clk);

		if (ret < 0)
			dev_err(i2c->dev, "Failed to set clock\n");
	}

	ipclk = (unsigned int)clk_get_rate(i2c->rate_clk);

	if (mode == HSI2C_STAND_SPD) {
		op_clk = i2c->stand_clock;

		if (!op_clk)
			op_clk = HSI2C_STAND_TX_CLOCK;

		fs_div = ipclk / (op_clk * 16);
		fs_div &= 0xFF;
		utemp = readl(i2c->regs + HSI2C_TIMING_FS3) & ~0x00FF0000;
		writel(utemp | (fs_div << 16), i2c->regs + HSI2C_TIMING_FS3);

		if (!i2c->tscl_h) {
			utscl_h_fs = (25 * (ipclk / (1000 * 1000))) /
				((fs_div + 1) * 10);
			utscl_h_fs = (0xFF << utscl_h_fs) & 0xFF;
		} else {
			utscl_h_fs = i2c->tscl_h;
		}
		utemp = readl(i2c->regs + HSI2C_TIMING_FS2) & ~0x000000FF;
		writel(utemp | (utscl_h_fs << 0), i2c->regs + HSI2C_TIMING_FS2);

		if (i2c->tscl_l) {
			utscl_l_fs = i2c->tscl_l;
			utemp = readl(i2c->regs + HSI2C_TIMING_FS2) &
				~0x0000FF00;
			writel(utemp | (utscl_l_fs << 8), i2c->regs +
					HSI2C_TIMING_FS2);
		}

		utstart_hd_fs = (25 * (ipclk / (1000 * 1000))) /
			((fs_div + 1) * 10) - 1;
		utstart_hd_fs = (0xFF << utstart_hd_fs) & 0xFF;
		utemp = readl(i2c->regs + HSI2C_TIMING_FS1) & ~0x00FF0000;
		writel(utemp | (utstart_hd_fs << 16), i2c->regs +
				HSI2C_TIMING_FS1);

		dev_info(i2c->dev, "%s IPCLK = %d OP_CLK = %d DIV = %d TIMING FS1(STAND) = 0x%X TIMING FS2(STAND) = 0x%X TIMING FS3(STAND) = 0x%X\n",
			 __func__, ipclk, op_clk, fs_div,
				readl(i2c->regs + HSI2C_TIMING_FS1),
				readl(i2c->regs + HSI2C_TIMING_FS2),
				readl(i2c->regs + HSI2C_TIMING_FS3));
	} else if (mode == HSI2C_FAST_PLUS_SPD) {
		unsigned int sample_factor;

		op_clk = i2c->fs_plus_clock;

		if (!op_clk)
			op_clk = HSI2C_FAST_PLUS_TX_CLOCK;

		if (op_clk > HSI2C_FS_PLUS_HIGH_TX_CLOCK)
			sample_factor = 17;
		else
			sample_factor = 16;

		fs_div = ipclk / (op_clk * sample_factor);
		fs_div &= 0xFF;
		utemp = readl(i2c->regs + HSI2C_TIMING_FS3) & ~0x00FF0000;
		writel(utemp | (fs_div << 16), i2c->regs + HSI2C_TIMING_FS3);

		if (!i2c->tscl_h) {
			utscl_h_fs = ((ipclk / (1000 * 1000)) * 35 / 10) /
				((fs_div + 1) * 10);
			utscl_h_fs = (0xFF << utscl_h_fs) & 0xFF;
		} else {
			utscl_h_fs = i2c->tscl_h;
		}
		utemp = readl(i2c->regs + HSI2C_TIMING_FS2) & ~0x000000FF;
		writel(utemp | (utscl_h_fs << 0), i2c->regs + HSI2C_TIMING_FS2);

		if (i2c->tscl_l) {
			utscl_l_fs = i2c->tscl_l;
			utemp = readl(i2c->regs + HSI2C_TIMING_FS2) &
				~0x0000FF00;
			writel(utemp | (utscl_l_fs << 8), i2c->regs +
					HSI2C_TIMING_FS2);
		}

		utstart_hd_fs = (4 * (ipclk / (1000 * 1000))) /
			((fs_div + 1) * 10) - 1;
		utstart_hd_fs = (0xFF << utstart_hd_fs) & 0xFF;
		utemp = readl(i2c->regs + HSI2C_TIMING_FS1) & ~0x00FF0000;
		writel(utemp | (utstart_hd_fs << 16), i2c->regs +
				HSI2C_TIMING_FS1);

		dev_info(i2c->dev, "%s IPCLK = %d OP_CLK = %d DIV = %d TIMING FS1(FS+) = 0x%X TIMING FS2(FS+) = 0x%X TIMING FS3(FS+) = 0x%X\n",
			 __func__, ipclk, op_clk, fs_div,
				readl(i2c->regs + HSI2C_TIMING_FS1),
				readl(i2c->regs + HSI2C_TIMING_FS2),
				readl(i2c->regs + HSI2C_TIMING_FS3));
	} else if (mode == HSI2C_HIGH_SPD) {
		/* ipclk's unit is Hz, op_clk's unit is Hz */
		op_clk = i2c->hs_clock;

		if (!op_clk)
			op_clk = HSI2C_HS_TX_CLOCK;

		hs_div = ipclk / (op_clk * 17);
		hs_div &= 0xFF;
		utemp = readl(i2c->regs + HSI2C_TIMING_HS3) & ~0x00FF0000;
		writel(utemp | (hs_div << 16), i2c->regs + HSI2C_TIMING_HS3);

		if (!i2c->tscl_h) {
			utscl_h_hs = ((7 * ipclk) / (1000 * 1000)) /
				((hs_div + 1) * 100);
			/* make to 0 into TSCL_H_HS from LSB */
			utscl_h_hs = (0xFFFFFFFF >> utscl_h_hs) << utscl_h_hs;
			utscl_h_hs &= 0xFF;
		} else {
			utscl_h_hs = i2c->tscl_h;
		}
		utemp = readl(i2c->regs + HSI2C_TIMING_HS2) & ~0x000000FF;
		writel(utemp | (utscl_h_hs << 0), i2c->regs + HSI2C_TIMING_HS2);

		if (i2c->tscl_l) {
			utscl_l_hs = i2c->tscl_l;
			utemp = readl(i2c->regs + HSI2C_TIMING_FS2) &
				~0x0000FF00;
			writel(utemp | (utscl_l_hs << 8), i2c->regs +
					HSI2C_TIMING_FS2);
		}

		utstart_hd_hs = (7 * ipclk / (1000 * 1000)) /
			((hs_div + 1) * 100) - 1;
		/* make to 0 into utstart_hd_hs from LSB */
		utstart_hd_hs = (0xFFFFFFFF >> utstart_hd_hs) << utstart_hd_hs;
		utstart_hd_hs &= 0xFF;
		utemp = readl(i2c->regs + HSI2C_TIMING_HS1) & ~0x00FF0000;
		writel(utemp | (utstart_hd_hs << 16), i2c->regs +
				HSI2C_TIMING_HS1);

		dev_info(i2c->dev, "%s IPCLK = %d OP_CLK = %d DIV = %d TIMING HS1 = 0x%08X TIMING HS2 = 0x%08X TIMING HS3 = 0x%08X\n",
			 __func__, ipclk, op_clk, hs_div,
				readl(i2c->regs + HSI2C_TIMING_HS1),
				readl(i2c->regs + HSI2C_TIMING_HS2),
				readl(i2c->regs + HSI2C_TIMING_HS3));
	} else {
		/* Fast speed mode */
		/* ipclk's unit is Hz, op_clk's unit is Hz */
		op_clk = i2c->fs_clock;

		if (!op_clk)
			op_clk = HSI2C_FS_TX_CLOCK;

		fs_div = ipclk / (op_clk * 16);
		fs_div &= 0xFF;
		utemp = readl(i2c->regs + HSI2C_TIMING_FS3) & ~0x00FF0000;
		writel(utemp | (fs_div << 16), i2c->regs + HSI2C_TIMING_FS3);

		if (!i2c->tscl_h) {
			utscl_h_fs = ((9 * ipclk) / (1000 * 1000)) /
				((fs_div + 1) * 10);
			/* make to 0 into TSCL_H_FS from LSB */
			utscl_h_fs = (0xFFFFFFFF >> utscl_h_fs) << utscl_h_fs;
			utscl_h_fs &= 0xFF;
		} else {
			utscl_h_fs = i2c->tscl_h;
		}
		utemp = readl(i2c->regs + HSI2C_TIMING_FS2) & ~0x000000FF;
		writel(utemp | (utscl_h_fs << 0), i2c->regs + HSI2C_TIMING_FS2);

		if (i2c->tscl_l) {
			utscl_l_fs = i2c->tscl_l;
			utemp = readl(i2c->regs + HSI2C_TIMING_FS2) &
				~0x0000FF00;
			writel(utemp | (utscl_l_fs << 8), i2c->regs +
					HSI2C_TIMING_FS2);
		}

		utstart_hd_fs = (9 * ipclk / (1000 * 1000)) /
			((fs_div + 1) * 10) - 1;
		/* make to 0 into utstart_hd_fs from LSB */
		utstart_hd_fs = (0xFFFFFFFF >> utstart_hd_fs) << utstart_hd_fs;
		utstart_hd_fs &= 0xFF;
		utemp = readl(i2c->regs + HSI2C_TIMING_FS1) & ~0x00FF0000;
		writel(utemp | (utstart_hd_fs << 16), i2c->regs +
				HSI2C_TIMING_FS1);

		dev_info(i2c->dev, "%s IPCLK = %d OP_CLK = %d DIV = %d Timing FS1 = 0x%X TIMING FS2 = 0x%X TIMING FS3 = 0x%X\n",
			 __func__, ipclk, op_clk, fs_div,
				readl(i2c->regs + HSI2C_TIMING_FS1),
				readl(i2c->regs + HSI2C_TIMING_FS2),
				readl(i2c->regs + HSI2C_TIMING_FS3));
	}

	return 0;
}

static int exynos5_hsi2c_clock_setup(struct exynos5_i2c *i2c)
{
	/*
	 * Configure the Fast speed timing values for High Speed mode
	 * Because High Speed mode initially starts with Fast mode
	 */
	if (i2c->speed_mode == HSI2C_HIGH_SPD) {
		if (exynos5_i2c_set_timing(i2c, HSI2C_FAST_SPD)) {
			dev_err(i2c->dev, "HSI2C FS Clock set up failed\n");
			return -EINVAL;
		}
	}

	if (exynos5_i2c_set_timing(i2c, i2c->speed_mode)) {
		dev_err(i2c->dev, "HSI2C mode(%d) Clock set up failed\n",
			i2c->speed_mode);
		return -EINVAL;
	}

	return 0;
}

static void exynos_usi_init(struct exynos5_i2c *i2c)
{
	/* USI_RESET is active High signal.
	 * Reset value of USI_RESET is 'h1 to drive stable value to PAD.
	 * Due to this feature, the USI_RESET must be cleared (set as '0')
	 * before transaction starts.
	 */
	writel(USI_RESET, i2c->regs + USI_CON);
}

/*
 * exynos5_i2c_init: configures the controller for I2C functionality
 * Programs I2C controller for Master mode operation
 */
static void exynos5_i2c_init(struct exynos5_i2c *i2c)
{
	u32 i2c_conf = readl(i2c->regs + HSI2C_CONF);

	writel(HSI2C_MASTER, i2c->regs + HSI2C_CTL);
	writel(HSI2C_TRAILING_COUNT, i2c->regs + HSI2C_TRAILIG_CTL);

	if (i2c->speed_mode == HSI2C_HIGH_SPD) {
		writel(HSI2C_MASTER_ID(MASTER_ID(i2c->bus_id)),
		       i2c->regs + HSI2C_ADDR);
		i2c_conf |= HSI2C_HS_MODE;
	}

	writel(i2c_conf | HSI2C_AUTO_MODE, i2c->regs + HSI2C_CONF);

	i2c->need_hw_init = 0;
}

static void exynos5_i2c_reset(struct exynos5_i2c *i2c)
{
	u32 i2c_ctl;

	/* Set and clear the bit for reset */
	i2c_ctl = readl(i2c->regs + HSI2C_CTL);
	i2c_ctl |= HSI2C_SW_RST;
	writel(i2c_ctl, i2c->regs + HSI2C_CTL);

	i2c_ctl = readl(i2c->regs + HSI2C_CTL);
	i2c_ctl &= ~HSI2C_SW_RST;
	writel(i2c_ctl, i2c->regs + HSI2C_CTL);

	/* We don't expect calculations to fail during the run */
	exynos5_hsi2c_clock_setup(i2c);
	/* Initialize the configure registers */
	exynos5_i2c_init(i2c);
}

static inline void exynos5_i2c_stop(struct exynos5_i2c *i2c)
{
	writel(0, i2c->regs + HSI2C_INT_ENABLE);

	complete(&i2c->msg_complete);
}

/*
 * exynos5_i2c_irq: top level IRQ servicing routine
 *
 * INT_STATUS registers gives the interrupt details. Further,
 * FIFO_STATUS or TRANS_STATUS registers are to be check for detailed
 * state of the bus.
 */

static irqreturn_t exynos5_i2c_irq(int irqno, void *dev_id)
{
	struct exynos5_i2c *i2c = dev_id;
	unsigned long reg_val;
	unsigned long trans_status;
	unsigned char byte;

	if (!i2c) {
		pr_err("irq nodev (irqno:%d)\n", irqno);
		return IRQ_HANDLED;
	}

	if (!i2c->msg || readl(i2c->regs + HSI2C_INT_ENABLE) == 0x0) {
		exynos5_i2c_clr_pend_irq(i2c);
		return IRQ_NONE;
	}

	if (i2c->msg->flags & I2C_M_RD) {
		while ((readl(i2c->regs + HSI2C_FIFO_STATUS) &
			HSI2C_RX_FIFO_EMPTY) == 0) {
			byte = (unsigned char)readl(i2c->regs + HSI2C_RX_DATA);
			i2c->msg->buf[i2c->msg_ptr++] = byte;
		}

		if (i2c->msg_ptr >= i2c->msg->len) {
			reg_val = readl(i2c->regs + HSI2C_INT_ENABLE);
			reg_val &= ~(HSI2C_INT_RX_ALMOSTFULL_EN);
			writel(reg_val, i2c->regs + HSI2C_INT_ENABLE);
			exynos5_i2c_stop(i2c);
		}
	} else {
		while ((readl(i2c->regs + HSI2C_FIFO_STATUS) &
			HSI2C_TX_FIFO_FULL) == 0) {
			if (i2c->msg_ptr >= i2c->msg->len) {
				reg_val = readl(i2c->regs + HSI2C_INT_ENABLE);
				reg_val &= ~(HSI2C_INT_TX_ALMOSTEMPTY_EN);
				writel(reg_val, i2c->regs + HSI2C_INT_ENABLE);
				break;
			}
			byte = i2c->msg->buf[i2c->msg_ptr++];
			writel(byte, i2c->regs + HSI2C_TX_DATA);
		}
	}

	reg_val = readl(i2c->regs + HSI2C_INT_STATUS);

	/*
	 * Checking Error State in INT_STATUS register
	 */
	if (reg_val & HSI2C_INT_CHK_TRANS_STATE) {
		trans_status = readl(i2c->regs + HSI2C_TRANS_STATUS);
		dev_err(i2c->dev, "HSI2C Error Interrupt occurred(IS:0x%08x, TR:0x%08x)\n",
			(unsigned int)reg_val, (unsigned int)trans_status);

		if (reg_val & HSI2C_INT_NODEV) {
			dev_err(i2c->dev, "HSI2C NO ACK occurred\n");
			if (i2c->nack_restart) {
				if (reg_val & HSI2C_INT_TRANSFER_DONE)
					exynos5_i2c_stop(i2c);
				goto out;
			}
		}

		i2c->trans_done = -ENXIO;
		exynos5_i2c_stop(i2c);
		goto out;
	}
	/* Checking INT_TRANSFER_DONE */
	if ((reg_val & HSI2C_INT_TRANSFER_DONE) &&
	    i2c->msg_ptr >= i2c->msg->len &&
		!(i2c->msg->flags & I2C_M_RD))
		exynos5_i2c_stop(i2c);

out:
	writel(reg_val, i2c->regs +  HSI2C_INT_STATUS);

	return IRQ_HANDLED;
}

static int exynos5_i2c_xfer_msg(struct exynos5_i2c *i2c,
				struct i2c_msg *msgs, int stop)
{
	unsigned long timeout;
	unsigned long timeout_max;
	unsigned long trans_status;
	unsigned long i2c_ctl;
	unsigned long i2c_auto_conf;
	unsigned long i2c_timeout;
	unsigned long i2c_addr;
	unsigned long i2c_int_en;
	unsigned long i2c_fifo_ctl;
	unsigned char byte;
	int ret = 0;
	int operation_mode = i2c->operation_mode;

	i2c->msg = msgs;
	i2c->msg_ptr = 0;
	i2c->trans_done = 0;

	/* (length * (bits + ack) * (s/ms) * / freq) * (tolerance) */
	timeout_max = (i2c->msg->len * 9 * 1000 / i2c->clock_frequency) * 2;
	/* Minimum timeout is 100ms */
	if (timeout_max < 100)
		timeout_max = 100;

	reinit_completion(&i2c->msg_complete);

	i2c_ctl = readl(i2c->regs + HSI2C_CTL);
	i2c_auto_conf = readl(i2c->regs + HSI2C_AUTO_CONF);
	i2c_timeout = readl(i2c->regs + HSI2C_TIMEOUT);
	i2c_timeout &= ~HSI2C_TIMEOUT_EN;
	writel(i2c_timeout, i2c->regs + HSI2C_TIMEOUT);

	/*
	 * In case of short length request it'd be better to set
	 * trigger level as msg length
	 */
	if (i2c->msg->len >= FIFO_TRIG_CRITERIA) {
		i2c_fifo_ctl = HSI2C_RXFIFO_EN | HSI2C_TXFIFO_EN |
			HSI2C_RXFIFO_TRIGGER_LEVEL(FIFO_TRIG_CRITERIA) |
			HSI2C_TXFIFO_TRIGGER_LEVEL(FIFO_TRIG_CRITERIA);
	} else {
		i2c_fifo_ctl = HSI2C_RXFIFO_EN | HSI2C_TXFIFO_EN |
			HSI2C_RXFIFO_TRIGGER_LEVEL(i2c->msg->len) |
			HSI2C_TXFIFO_TRIGGER_LEVEL(i2c->msg->len);
	}

	writel(i2c_fifo_ctl, i2c->regs + HSI2C_FIFO_CTL);

	i2c_int_en = 0;
	if (msgs->flags & I2C_M_RD) {
		i2c_ctl &= ~HSI2C_TXCHON;
		i2c_ctl |= HSI2C_RXCHON;

		i2c_auto_conf |= HSI2C_READ_WRITE;

		i2c_int_en |= (HSI2C_INT_RX_ALMOSTFULL_EN |
			HSI2C_INT_TRAILING_EN);
	} else {
		i2c_ctl &= ~HSI2C_RXCHON;
		i2c_ctl |= HSI2C_TXCHON;

		i2c_auto_conf &= ~HSI2C_READ_WRITE;

		i2c_int_en |= HSI2C_INT_TX_ALMOSTEMPTY_EN;
	}

	if (operation_mode == HSI2C_INTERRUPT)
		exynos5_i2c_clr_pend_irq(i2c);

	if (stop == 1 || i2c->stop_after_trans == 1)
		i2c_auto_conf |= HSI2C_STOP_AFTER_TRANS;
	else
		i2c_auto_conf &= ~HSI2C_STOP_AFTER_TRANS;

	if (i2c->is_no_arbitration)
		i2c_ctl |= HSI2C_NO_LOSE_ARBITRATION;

	i2c_addr = readl(i2c->regs + HSI2C_ADDR);
	i2c_addr &= ~(0x3ff << 10);
	i2c_addr &= ~(0x3ff << 0);
	if (i2c->speed_mode != HSI2C_HIGH_SPD) {
		i2c_addr &= ~(0xff << 24);
		i2c_addr |= (0x7 << 24);
	}
	i2c_addr |= ((msgs->addr & 0x7f) << 10);
	writel(i2c_addr, i2c->regs + HSI2C_ADDR);

	writel(i2c_ctl, i2c->regs + HSI2C_CTL);

	if (operation_mode == HSI2C_INTERRUPT) {
		unsigned int cpu = raw_smp_processor_id();

		i2c_int_en |= HSI2C_INT_CHK_TRANS_STATE |
			HSI2C_INT_TRANSFER_DONE;
		writel(i2c_int_en, i2c->regs + HSI2C_INT_ENABLE);

		irq_set_affinity_hint(i2c->irq, cpumask_of(cpu));
		enable_irq(i2c->irq);
	} else {
		writel(HSI2C_INT_TRANSFER_DONE, i2c->regs + HSI2C_INT_ENABLE);
	}

	i2c_auto_conf &= ~(0xffff);
	i2c_auto_conf |= i2c->msg->len;
	writel(i2c_auto_conf, i2c->regs + HSI2C_AUTO_CONF);

	i2c_auto_conf = readl(i2c->regs + HSI2C_AUTO_CONF);
	i2c_auto_conf |= HSI2C_MASTER_RUN;
	writel(i2c_auto_conf, i2c->regs + HSI2C_AUTO_CONF);

	ret = -EAGAIN;
	if (msgs->flags & I2C_M_RD && operation_mode == HSI2C_POLLING) {
		timeout = jiffies + msecs_to_jiffies(timeout_max);
		while (time_before(jiffies, timeout)) {
			if ((readl(i2c->regs + HSI2C_FIFO_STATUS) &
						HSI2C_RX_FIFO_EMPTY) == 0) {
				/* RX FIFO is not empty */
				byte = (unsigned char)readl
					(i2c->regs + HSI2C_RX_DATA);
				i2c->msg->buf[i2c->msg_ptr++] = byte;
			}

			if (i2c->msg_ptr >= i2c->msg->len) {
				ret = 0;
				break;
			}
		}

		if (ret == -EAGAIN) {
			dump_i2c_register(i2c);
			exynos5_i2c_reset(i2c);
			dev_warn(i2c->dev, "rx timeout\n");
			return ret;
		}
	} else if (msgs->flags & I2C_M_RD &&
			operation_mode == HSI2C_INTERRUPT) {
		timeout = wait_for_completion_timeout
			(&i2c->msg_complete,
			 msecs_to_jiffies(timeout_max));

		ret = 0;

		if (i2c->scl_clk_stretch) {
			unsigned long timeout = jiffies + msecs_to_jiffies(100);

			do {
				trans_status = readl(i2c->regs +
						HSI2C_TRANS_STATUS);
				if ((trans_status & HSI2C_MAST_ST_MASK) ==
						HSI2C_MASTER_ST_INIT){
					timeout = 0;
					break;
				}
			} while (time_before(jiffies, timeout));

			if (timeout)
				dev_err(i2c->dev, "SDA check timeout AT READ!!! = 0x%8lx\n",
					trans_status);
		}

		disable_irq(i2c->irq);

		if (i2c->trans_done < 0) {
			dev_err(i2c->dev, "ack was not received at read\n");
			ret = i2c->trans_done;
			exynos5_i2c_reset(i2c);
		}

		if (timeout == 0) {
			dump_i2c_register(i2c);
			exynos5_i2c_reset(i2c);
			dev_warn(i2c->dev, "rx timeout\n");
			ret = -EAGAIN;
			return ret;
		}
	} else if (!(msgs->flags & I2C_M_RD) &&
			operation_mode == HSI2C_POLLING) {
		unsigned long int_status;
		unsigned long fifo_status;

		timeout = jiffies + msecs_to_jiffies(timeout_max);
		while (time_before(jiffies, timeout) &&
		       (i2c->msg_ptr < i2c->msg->len)) {
			if ((readl(i2c->regs + HSI2C_FIFO_STATUS)
						& HSI2C_TX_FIFO_LVL_MASK) <
					EXYNOS5_FIFO_SIZE) {
				byte = i2c->msg->buf[i2c->msg_ptr++];
				writel(byte, i2c->regs + HSI2C_TX_DATA);
			}
		}
		while (time_before(jiffies, timeout)) {
			int_status = readl(i2c->regs + HSI2C_INT_STATUS);
			fifo_status = readl(i2c->regs + HSI2C_FIFO_STATUS);
			if (int_status & HSI2C_INT_TRANSFER_DONE &&
			    fifo_status & HSI2C_TX_FIFO_EMPTY) {
				writel(int_status, i2c->regs +
						HSI2C_INT_STATUS);
				ret = 0;
				break;
			}
			udelay(1);
		}
		if (ret == -EAGAIN) {
			dump_i2c_register(i2c);
			exynos5_i2c_reset(i2c);
			dev_warn(i2c->dev, "tx timeout\n");
			return ret;
		}
	} else if (!(msgs->flags & I2C_M_RD) &&
			operation_mode == HSI2C_INTERRUPT) {
		timeout = wait_for_completion_timeout
			(&i2c->msg_complete,
			 msecs_to_jiffies(timeout_max));
		disable_irq(i2c->irq);

		if (timeout == 0) {
			dump_i2c_register(i2c);
			exynos5_i2c_reset(i2c);
			dev_warn(i2c->dev, "tx timeout\n");
			return ret;
		}

		timeout = jiffies + timeout;
		if (i2c->trans_done < 0) {
			dev_err(i2c->dev, "ack was not received at write\n");
			ret = i2c->trans_done;
			exynos5_i2c_reset(i2c);
			return ret;
		}

		if (i2c->scl_clk_stretch) {
			unsigned long timeout = jiffies + msecs_to_jiffies(100);

			do {
				trans_status = readl(i2c->regs +
						HSI2C_TRANS_STATUS);
				if ((trans_status & HSI2C_MAST_ST_MASK) ==
						HSI2C_MASTER_ST_INIT){
					timeout = 0;
					break;
				}
			} while (time_before(jiffies, timeout));

			if (timeout)
				dev_err(i2c->dev, "SDA check timeout AT WRITE!!! = 0x%8lx\n",
					trans_status);
		}

		ret = 0;
	}

	return ret;
}

static int exynos5_i2c_xfer(struct i2c_adapter *adap,
			    struct i2c_msg *msgs, int num)
{
	struct exynos5_i2c *i2c = (struct exynos5_i2c *)adap->algo_data;
	struct i2c_msg *msgs_ptr = msgs;
	int retry, i = 0;
	int ret = 0;
	int stop = 0;

#ifdef CONFIG_PM
	int clk_ret = 0;
#endif

	if (i2c->suspended) {
		dev_err(i2c->dev, "HS-I2C is not initialized.\n");
		return -EIO;
	}

#ifdef CONFIG_PM
	clk_ret = pm_runtime_get_sync(i2c->dev);
	if (clk_ret < 0) {
		exynos_update_ip_idle_status(i2c->idle_ip_index, 0);
		ret = clk_enable(i2c->clk);
		if (ret) {
			exynos_update_ip_idle_status(i2c->idle_ip_index, 1);
			return ret;
		}
	}
#else
	exynos_update_ip_idle_status(i2c->idle_ip_index, 0);
	ret = clk_enable(i2c->clk);
	if (ret) {
		exynos_update_ip_idle_status(i2c->idle_ip_index, 1);
		return ret;
	}
#endif

	/* If master is in arbitration lost state before transfer */
	/* master should be reset */
	if (i2c->reset_before_trans) {
		if (unlikely((readl(i2c->regs + HSI2C_TRANS_STATUS)
			& HSI2C_MAST_ST_MASK) == 0x6)) {
			pr_info("%s trans status and reset = %s\n",
				__func__, adap->name);
			i2c->need_hw_init = 1;
		}
	}

	if (i2c->need_hw_init)
		exynos5_i2c_reset(i2c);

	if (unlikely(!(readl(i2c->regs + HSI2C_CONF)
			& HSI2C_AUTO_MODE))) {
		dev_err(i2c->dev, "HSI2C should be reconfigured\n");
		exynos5_hsi2c_clock_setup(i2c);
		exynos5_i2c_init(i2c);
	}

	for (retry = 0; retry < adap->retries; retry++) {
		for (i = 0; i < num; i++) {
			stop = (i == num - 1);

			if (i2c->transfer_delay)
				udelay(i2c->transfer_delay);

			ret = exynos5_i2c_xfer_msg(i2c, msgs_ptr, stop);

			msgs_ptr++;

			if (ret == -EAGAIN) {
				msgs_ptr = msgs;
				break;
			} else if (ret < 0) {
				goto out;
			}
		}

		if (i == num && (ret != -EAGAIN))
			break;

		dev_dbg(i2c->dev, "retrying transfer (%d)\n", retry);

		usleep_range(100, 200);
	}

	if (i == num) {
		ret = num;
	} else {
		ret = -EREMOTEIO;
		dev_warn(i2c->dev, "xfer message failed\n");
	}

 out:
#ifdef CONFIG_PM
	if (clk_ret < 0) {
		clk_disable(i2c->clk);
		exynos_update_ip_idle_status(i2c->idle_ip_index, 1);
	} else {
		pm_runtime_mark_last_busy(i2c->dev);
		pm_runtime_put_autosuspend(i2c->dev);
	}
#else
	clk_disable(i2c->clk);
	exynos_update_ip_idle_status(i2c->idle_ip_index, 1);
#endif

	return ret;
}

static u32 exynos5_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | (I2C_FUNC_SMBUS_EMUL & ~I2C_FUNC_SMBUS_QUICK);
}

static const struct i2c_algorithm exynos5_i2c_algorithm = {
	.master_xfer		= exynos5_i2c_xfer,
	.functionality		= exynos5_i2c_func,
};

static int exynos5_i2c_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct device *dev = &pdev->dev;
	struct exynos5_i2c *i2c;
	int ret;

	if (!np) {
		dev_err(&pdev->dev, "no device node\n");
		return -ENOENT;
	}

	i2c = devm_kzalloc(&pdev->dev, sizeof(struct exynos5_i2c), GFP_KERNEL);
	if (!i2c)
		return -ENOMEM;

	if (of_property_read_u32(np, "samsung,usi-offset", &i2c->usi_offset))
		dev_warn(dev, "usi offset is not specified\n");

	i2c->usi_reg = syscon_regmap_lookup_by_phandle(dev->of_node,
						       "samsung,usi-phandle");
	if (IS_ERR(i2c->usi_reg)) {
		dev_info(dev, "no lookup for usi-phandle.\n");
	} else {
		u32 val;

		regmap_update_bits(i2c->usi_reg, i2c->usi_offset,
				   USI_SW_CONF_MASK, USI_I2C_SW_CONF);
		regmap_read(i2c->usi_reg, i2c->usi_offset, &val);
		dev_info(dev, "set usi mode - 0x%x\n", val & USI_SW_CONF_MASK);
	}

	if (of_property_read_u32(np, "default-clk", &i2c->default_clk))
		dev_err(i2c->dev, "Failed to get default clk info\n");

	/* Mode of operation High/Fast/Fast+ Speed mode */
	if (of_property_read_u32(np, "clock-frequency",
				 &i2c->clock_frequency)) {
		i2c->clock_frequency = HSI2C_STAND_TX_CLOCK;
		i2c->stand_clock = i2c->clock_frequency;
		i2c->speed_mode = HSI2C_STAND_SPD;
	} else {
		if (i2c->clock_frequency <= HSI2C_STAND_TX_CLOCK) {
			i2c->stand_clock = i2c->clock_frequency;
			i2c->speed_mode = HSI2C_STAND_SPD;
		} else if (i2c->clock_frequency <= HSI2C_FS_TX_CLOCK) {
			i2c->fs_clock = i2c->clock_frequency;
			i2c->speed_mode = HSI2C_FAST_SPD;
		} else if (i2c->clock_frequency <= HSI2C_FAST_PLUS_TX_CLOCK) {
			i2c->fs_plus_clock = i2c->clock_frequency;
			i2c->speed_mode = HSI2C_FAST_PLUS_SPD;
		} else {
			if (of_get_property(np, "samsung,hs-mode", NULL)) {
				i2c->hs_clock = i2c->clock_frequency;
				i2c->speed_mode = HSI2C_HIGH_SPD;
			} else {
				i2c->fs_plus_clock = i2c->clock_frequency;
				i2c->speed_mode = HSI2C_FAST_PLUS_SPD;
			}
		}
	}

	ret = of_property_read_u32(np, "samsung,tscl-h", &i2c->tscl_h);
	if (!ret)
		dev_warn(&pdev->dev, "tSCL_HIGH val: 0x%x\n", i2c->tscl_h);

	ret = of_property_read_u32(np, "samsung,tscl-l", &i2c->tscl_l);
	if (!ret)
		dev_warn(&pdev->dev, "tSCL_LOW val: 0x%x\n", i2c->tscl_l);

	/* Mode of operation Polling/Interrupt mode */
	if (of_get_property(np, "samsung,polling-mode", NULL))
		i2c->operation_mode = HSI2C_POLLING;
	else
		i2c->operation_mode = HSI2C_INTERRUPT;

	if (of_get_property(np, "samsung,scl-clk-stretching", NULL))
		i2c->scl_clk_stretch = 1;
	else
		i2c->scl_clk_stretch = 0;

	ret = of_property_read_u32(np, "samsung,transfer_delay",
				   &i2c->transfer_delay);
	if (!ret)
		dev_warn(&pdev->dev, "Transfer delay is not needed.\n");

	if (of_get_property(np, "samsung,stop-after-trans", NULL))
		i2c->stop_after_trans = 1;
	else
		i2c->stop_after_trans = 0;

	if (of_get_property(np, "samsung,reset-before-trans", NULL))
		i2c->reset_before_trans = 1;
	else
		i2c->reset_before_trans = 0;

	if (of_get_property(np, "samsung,no-dev-restart", NULL))
		i2c->nack_restart = 1;
	else
		i2c->nack_restart = 0;

	if (of_get_property(np, "samsung,no_lose_arbitration", NULL))
		i2c->is_no_arbitration = 1;
	else
		i2c->is_no_arbitration = 0;

	i2c->idle_ip_index = exynos_get_idle_ip_index(dev_name(&pdev->dev));

	strlcpy(i2c->adap.name, "exynos5-i2c", sizeof(i2c->adap.name));
	i2c->adap.owner   = THIS_MODULE;
	i2c->adap.algo    = &exynos5_i2c_algorithm;
	i2c->adap.retries = 2;
	i2c->adap.class   = I2C_CLASS_HWMON | I2C_CLASS_SPD;

	i2c->dev = &pdev->dev;
	i2c->clk = devm_clk_get(&pdev->dev, "gate_hsi2c_clk");
	if (IS_ERR(i2c->clk)) {
		dev_err(&pdev->dev, "cannot get clock\n");
		return -ENOENT;
	}

	i2c->rate_clk = devm_clk_get(&pdev->dev, "ipclk_hsi2c");
	if (IS_ERR(i2c->rate_clk)) {
		dev_err(&pdev->dev, "cannot get rate clock\n");
		return -ENOENT;
	}

	ret = clk_prepare(i2c->clk);
	if (ret) {
		dev_err(&pdev->dev, "Clock prepare failed\n");
			return ret;
	}

#ifdef CONFIG_PM
	pm_runtime_use_autosuspend(&pdev->dev);
	pm_runtime_set_autosuspend_delay(&pdev->dev,
					 EXYNOS5_HSI2C_RUNTIME_PM_DELAY);
	pm_runtime_enable(&pdev->dev);
#endif

	i2c->regs = devm_platform_ioremap_resource(pdev, 0);
	if (!i2c->regs) {
		dev_err(&pdev->dev, "cannot map HS-I2C IO\n");
		ret = PTR_ERR(i2c->regs);
		goto err_clk1;
	}

	i2c->adap.dev.of_node = np;
	i2c->adap.algo_data = i2c;
	i2c->adap.dev.parent = &pdev->dev;

	init_completion(&i2c->msg_complete);

	platform_set_drvdata(pdev, i2c);
#ifdef CONFIG_PM
	pm_runtime_get_sync(&pdev->dev);
#else
	exynos_update_ip_idle_status(i2c->idle_ip_index, 0);
	ret = clk_enable(i2c->clk);
	if (ret) {
		exynos_update_ip_idle_status(i2c->idle_ip_index, 1);
		return ret;
	}
#endif
	exynos_usi_init(i2c);

	/* Clear pending interrupts from u-boot or misc causes */
	exynos5_i2c_clr_pend_irq(i2c);
	/* Reset i2c SFR from u-boot or misc causes */
	exynos5_i2c_reset(i2c);

	if (i2c->operation_mode == HSI2C_INTERRUPT) {
		i2c->irq = irq_of_parse_and_map(np, 0);
		if (i2c->irq <= 0) {
			dev_err(&pdev->dev, "cannot find HS-I2C IRQ\n");
			ret = -EINVAL;
			goto err_clk1;
		}

		ret = devm_request_irq(&pdev->dev, i2c->irq, exynos5_i2c_irq,
				       0, dev_name(&pdev->dev), i2c);
		disable_irq(i2c->irq);

		if (ret != 0) {
			dev_err(&pdev->dev, "cannot request HS-I2C IRQ %d\n",
				i2c->irq);
			goto err_clk1;
		}
	}

	i2c->bus_id = of_alias_get_id(i2c->adap.dev.of_node, "hsi2c");

	exynos5_i2c_init(i2c);

	i2c->adap.nr = -1;
	ret = i2c_add_numbered_adapter(&i2c->adap);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to add bus to i2c core\n");
		goto err_clk2;
	}

#ifdef CONFIG_PM
	pm_runtime_mark_last_busy(&pdev->dev);
	pm_runtime_put_autosuspend(&pdev->dev);
#else
	clk_disable(i2c->clk);
	exynos_update_ip_idle_status(i2c->idle_ip_index, 1);
#endif
	return 0;

 err_clk2:
#ifdef CONFIG_PM
	pm_runtime_mark_last_busy(&pdev->dev);
	pm_runtime_put_autosuspend(&pdev->dev);
#else
	clk_disable_unprepare(i2c->clk);
	exynos_update_ip_idle_status(i2c->idle_ip_index, 1);
#endif
 err_clk1:
	return ret;
}

static int exynos5_i2c_remove(struct platform_device *pdev)
{
	struct exynos5_i2c *i2c = platform_get_drvdata(pdev);

	i2c_del_adapter(&i2c->adap);

	clk_unprepare(i2c->clk);

	return 0;
}

#ifdef CONFIG_PM
static int exynos5_i2c_runtime_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct exynos5_i2c *i2c = platform_get_drvdata(pdev);

	clk_disable(i2c->clk);
	exynos_update_ip_idle_status(i2c->idle_ip_index, 1);
	i2c->runtime_resumed = 0;

	return 0;
}

static int exynos5_i2c_runtime_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct exynos5_i2c *i2c = platform_get_drvdata(pdev);
	int ret = 0;

	exynos_update_ip_idle_status(i2c->idle_ip_index, 0);
	ret = clk_enable(i2c->clk);
	i2c->runtime_resumed = 1;
	if (ret) {
		exynos_update_ip_idle_status(i2c->idle_ip_index, 1);
		return ret;
	}

	return 0;
}
#endif /* CONFIG_PM */

#ifdef CONFIG_PM
static int exynos5_i2c_suspend_noirq(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct exynos5_i2c *i2c = platform_get_drvdata(pdev);
#ifdef CONFIG_I2C_SAMSUNG_HWACG
	int ret = 0;
#endif

	i2c_lock_bus(&i2c->adap, I2C_LOCK_ROOT_ADAPTER);
#ifdef CONFIG_I2C_SAMSUNG_HWACG
	exynos_update_ip_idle_status(i2c->idle_ip_index, 0);
	ret = clk_enable(i2c->clk);
	if (ret) {
		exynos_update_ip_idle_status(i2c->idle_ip_index, 1);
		i2c_unlock_bus(&i2c->adap, I2C_LOCK_ROOT_ADAPTER);
		return ret;
	}
	writel(HSI2C_SW_RST, i2c->regs + HSI2C_CTL);
	clk_disable(i2c->clk);
	exynos_update_ip_idle_status(i2c->idle_ip_index, 1);
#endif

	if (!pm_runtime_status_suspended(dev))
		exynos5_i2c_runtime_suspend(dev);

	i2c->suspended = 1;
	i2c_unlock_bus(&i2c->adap, I2C_LOCK_ROOT_ADAPTER);

	return 0;
}

static int exynos5_i2c_resume_noirq(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct exynos5_i2c *i2c = platform_get_drvdata(pdev);
	int ret = 0;

	if (!IS_ERR(i2c->usi_reg))
		regmap_update_bits(i2c->usi_reg, i2c->usi_offset,
				   USI_SW_CONF_MASK, USI_I2C_SW_CONF);

	i2c_lock_bus(&i2c->adap, I2C_LOCK_ROOT_ADAPTER);

	if (!pm_runtime_status_suspended(dev))
		exynos5_i2c_runtime_resume(dev);

	exynos_update_ip_idle_status(i2c->idle_ip_index, 0);
	ret = clk_enable(i2c->clk);
	if (ret) {
		exynos_update_ip_idle_status(i2c->idle_ip_index, 1);
		i2c_unlock_bus(&i2c->adap, I2C_LOCK_ROOT_ADAPTER);
		return ret;
	}

	exynos_usi_init(i2c);
	exynos5_i2c_reset(i2c);
	clk_disable(i2c->clk);
	exynos_update_ip_idle_status(i2c->idle_ip_index, 1);
	i2c->suspended = 0;
	i2c_unlock_bus(&i2c->adap, I2C_LOCK_ROOT_ADAPTER);

	return 0;
}

#else
static int exynos5_i2c_suspend_noirq(struct device *dev)
{
	return 0;
}

static int exynos5_i2c_resume_noirq(struct device *dev)
{
	return 0;
}
#endif

static const struct dev_pm_ops exynos5_i2c_pm = {
	SET_NOIRQ_SYSTEM_SLEEP_PM_OPS(exynos5_i2c_suspend_noirq,
				      exynos5_i2c_resume_noirq)
	SET_RUNTIME_PM_OPS(exynos5_i2c_runtime_suspend,
			   exynos5_i2c_runtime_resume, NULL)
};

static struct platform_driver exynos5_i2c_driver = {
	.probe		= exynos5_i2c_probe,
	.remove		= exynos5_i2c_remove,
	.driver		= {
		.name	= "exynos5-hsi2c",
		.pm	= &exynos5_i2c_pm,
		.of_match_table = exynos5_i2c_match,
	},
};

module_platform_driver(exynos5_i2c_driver);

MODULE_SOFTDEP("pre: vh_i2c");
MODULE_DESCRIPTION("Exynos5 HS-I2C Bus driver");
MODULE_AUTHOR("Naveen Krishna Chatradhi <ch.naveen@samsung.com>");
MODULE_AUTHOR("Taekgyun Ko <taeggyun.ko@samsung.com>");
MODULE_LICENSE("GPL v2");
