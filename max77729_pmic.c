/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2020 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": %s " fmt, __func__

#include <linux/ctype.h>
#include <linux/i2c.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/gpio.h>
#include <linux/gpio/driver.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/interrupt.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/irqdomain.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include "max_m5.h"
#include "max77759.h"
#include "max77759_maxq.h"
#include "gbms_storage.h"
#include "google_bms.h"


enum max77729_pmic_register {
	MAX77729_PMIC_ID         = 0x00,
	MAX77729_PMIC_REVISION   = 0x01,
	MAX77729_PMIC_MAINCTRL   = 0x02,
	MAX77729_PMIC_INTSRC     = 0x22,
	MAX77729_PMIC_INTSRCMASK = 0x23,
	MAX77729_PMIC_TOPSYS_INT = 0x24,
	MAX77729_PMIC_TOPSYS_INT_MASK = 0x26,
};

#define MUSBC 0x08
#define MFUEL 0x04
#define MTOPS 0x02
#define MCHGR 0x01

#define MAX77759_GPIO_DIR_IN  0
#define MAX77759_GPIO_DIR_OUT 1

#define MAX77759_GPIO5_DIR_MASK (1 << 2)
#define MAX77759_GPIO5_DIR(x) ((x) << 2)
#define MAX77759_GPIO5_VAL_MASK (1 << 3)
#define MAX77759_GPIO5_VAL(x) ((x) << 3)
#define MAX77759_GPIO6_DIR_MASK (1 << 5)
#define MAX77759_GPIO6_DIR(x) ((x) << 5)
#define MAX77759_GPIO6_VAL_MASK (1 << 6)
#define MAX77759_GPIO6_VAL(x) ((x) << 6)

#define MAX77759_GPIO5_OFF 4
#define MAX77759_GPIO6_OFF 5
#define MAX77759_MIN_GPIO_OFF 4
#define MAX77759_MAX_GPIO_OFF 5
#define MAX77759_NUM_GPIOS 6

#define MAX77759_GPIO_CONTROL_READ 0x23
#define MAX77759_GPIO_CONTROL_WRITE 0x24

#define MDEFAULT (0xF0 | ~(MUSBC | MFUEL | MCHGR))


#define MAX77729_PMIC_INTSRCMASK_DEFAULT	MDEFAULT
#define MAX77729_PMIC_TOPSYS_INT_MASK_DEFAULT	0xff

#define MAX77759_PMIC_INTSRCMASK_DEFAULT \
		~(MAX77759_PMIC_INTSRCMASK_MAXQ_INT_M | \
		  MAX77759_PMIC_INTSRCMASK_CHGR_INT_M)

/* b/156527175: *_PMIC_TOPSYS_INT_MASK_SPR_7 is reserved */
#define MAX77759_PMIC_TOPSYS_INT_MASK_MASK \
		(MAX77759_PMIC_TOPSYS_INT_MASK_TSHDN_INT_M | \
		 MAX77759_PMIC_TOPSYS_INT_MASK_SYSOVLO_INT_M | \
		 MAX77759_PMIC_TOPSYS_INT_MASK_SYSUVLO_INT_M)

#define MAX77759_PMIC_TOPSYS_INT_MASK_DEFAULT \
		(MAX77759_PMIC_TOPSYS_INT_MASK_TSHDN_INT_M)

#define MAX77759_STORAGE_SIZE	16
#define MAX77759_STORAGE_BASE	(MAX77759_PMIC_AP_DATAOUT0 + MAX77759_STORAGE_SIZE)

#define MAX77729_PMIC_NUM_REGS	(MAX77759_PMIC_UIC_SWRST - MAX77729_PMIC_ID + 1)

struct max77729_pmic_data {
	struct device        *dev;
	struct regmap        *regmap;
	uint8_t pmic_id;
	uint8_t rev_id;

#if IS_ENABLED(CONFIG_GPIOLIB)
	struct mutex irq_lock;
	struct gpio_chip     gpio;

	/* threaded irq */
	int irq_trig_falling[2];
	u8 irq_trig_u;
	u8 irq_mask;
	u8 irq_mask_u;
#endif
	struct max77759_maxq *maxq;

	struct i2c_client *fg_i2c_client;
	struct i2c_client *pmic_i2c_client;
	void *ovp_client_data;
	struct mutex io_lock;
	struct mutex reg_dump_lock;
	int batt_id;

	atomic_t sysuvlo_cnt;
	atomic_t sysovlo_cnt;
	struct dentry *de;

	struct delayed_work storage_init_work;

	/* debug interface, register to read or write */
	u32 debug_reg_address;

};

static bool max77729_pmic_is_reg(struct device *dev, unsigned int reg)
{
	int ret;

	switch (reg) {
	case MAX77729_PMIC_ID:
	case MAX77729_PMIC_REVISION:
	case MAX77729_PMIC_MAINCTRL:
	case MAX77729_PMIC_INTSRC:
	case MAX77729_PMIC_INTSRCMASK:
	case MAX77729_PMIC_TOPSYS_INT:
	case MAX77729_PMIC_TOPSYS_INT_MASK:
		ret = true;
		break;
	case MAX77759_PMIC_I2C_CNFG:
	case MAX77759_PMIC_SWRESET:
	case MAX77759_PMIC_CONTROL_FG:
		ret = true;
		break;
	case MAX77759_PMIC_DEVICE_ID:
	case MAX77759_PMIC_DEVICE_REV:
	case MAX77759_PMIC_FW_REV:
	case MAX77759_PMIC_FW_SUB_REV:
	case MAX77759_PMIC_UIC_INT1...MAX77759_PMIC_UIC_INT4_M:
	case MAX77759_PMIC_AP_DATAOUT0...MAX77759_PMIC_AP_DATAIN32:
	case MAX77759_PMIC_UIC_SWRST:
		ret = true;
		break;
	default:
		ret = false;
		break;
	}

	return ret;
}

static struct regmap_config max777x9_pmic_regmap_cfg = {
	.name = "max777x9_pmic",
	.reg_bits = 8,
	.val_bits = 8,
	.val_format_endian = REGMAP_ENDIAN_NATIVE,
	.max_register = MAX77729_PMIC_INTSRCMASK,
	.readable_reg = max77729_pmic_is_reg,
	.volatile_reg = max77729_pmic_is_reg,
};

static inline int max77729_pmic_readn(struct max77729_pmic_data *data,
				      int addr, u8 *val, int len)
{
	int rc;

	rc = regmap_bulk_read(data->regmap, addr, val, len);
	if (rc < 0)
		pr_err("regmap_read failed for address %04x rc=%d\n",
			addr, rc);

	return rc;
}

static inline int max77729_pmic_writen(struct max77729_pmic_data *data,
				       int addr, const u8 *val, int len)
{
	int rc;

	rc = regmap_bulk_write(data->regmap, addr, val, len);
	if (rc < 0)
		pr_err("regmap_write failed for address %04x rc=%d\n",
			addr, rc);

	return 0;
}


#define max77729_pmic_rd8(data, addr, val) \
		max77729_pmic_readn(data, addr, val, 1)
#define max77729_pmic_wr8(data, addr, val) \
		max77729_pmic_writen(data, addr, (const u8[]){ val }, 1)

/* no need for caching */
static inline int max77729_pmic_rmw8(struct max77729_pmic_data *data,
				     int reg, u8 mask, u8 value)
{
	return regmap_write_bits(data->regmap, reg, mask, value);
}

int max777x9_pmic_reg_read(struct i2c_client *client,
			   u8 addr, u8 *val, int len)
{
	struct max77729_pmic_data *data;

	if (!client)
		return -EINVAL;

	data = i2c_get_clientdata(client);
	if (!data || !data->regmap)
		return -ENXIO;

	return max77729_pmic_readn(data, addr, val, len);
}
EXPORT_SYMBOL_GPL(max777x9_pmic_reg_read);

int max777x9_pmic_reg_write(struct i2c_client *client,
			    u8 addr, const u8 *val, int len)
{
	struct max77729_pmic_data *data;

	if (!client)
		return -EINVAL;

	data = i2c_get_clientdata(client);
	if (!data || !data->regmap)
		return -ENXIO;

	return max77729_pmic_writen(data, addr, val, len);
}
EXPORT_SYMBOL_GPL(max777x9_pmic_reg_write);

int max777x9_pmic_reg_update(struct i2c_client *client,
			     u8 reg, u8 mask, u8 value)
{
	struct max77729_pmic_data *data;

	if (!client)
		return -EINVAL;

	data = i2c_get_clientdata(client);
	if (!data || !data->regmap)
		return -ENXIO;

	return max77729_pmic_rmw8(data, reg, mask, value);
}
EXPORT_SYMBOL_GPL(max777x9_pmic_reg_update);


int max777x9_pmic_get_id(struct i2c_client *client, u8 *id, u8 *rev)
{
	struct max77729_pmic_data *data;

	if (!client)
		return -EINVAL;

	data = i2c_get_clientdata(client);
	if (!data)
		return -ENXIO;

	*rev = data->rev_id;
	*id = data->pmic_id;
	return 0;
}
EXPORT_SYMBOL_GPL(max777x9_pmic_get_id);

static int max77729_gpio_to_irq_mask(int offset, unsigned int *mask, unsigned int *val)
{
	if (offset != MAX77759_GPIO5_OFF && offset != MAX77759_GPIO6_OFF)
		return -EINVAL;

	/* gpio5 is bit 0 of MAX77759_PMIC_UIC_INT1, gpio6 is bit 1 */
	*mask = 1 << (offset - MAX77759_GPIO5_OFF);
	*val = 1 << (offset - MAX77759_GPIO5_OFF) ;
	return 0;
}

static int max77729_gpio_clear_int(struct max77729_pmic_data *data, unsigned int offset)
{
	unsigned int mask, val;
	int ret;

	ret = max77729_gpio_to_irq_mask(offset, &mask, &val);
	if (ret == 0)
		ret = max77729_pmic_rmw8(data, MAX77759_PMIC_UIC_INT1, mask, val);

	pr_debug("offset=%d clear int1 mask=%x val=%x (%d)\n",
		 offset, mask, val, ret);

	return ret;
}

static irqreturn_t max777x9_pmic_route_irq(struct max77729_pmic_data *data,
					   int offset)
{
	int ret = 0, sub_irq;

	/* NOTE: clearing before handle_nested_irq() assumes EDGE-type IRQ */
	ret = max77729_gpio_clear_int(data, offset);
	if (ret < 0)
		pr_err("gpio%d cannot clear int1 (%d)", offset + 1, ret);

	sub_irq = irq_find_mapping(data->gpio.irq.domain, offset);
	pr_debug("offset=%d sub_irq=%d\n", offset, sub_irq);
	if (sub_irq)
		handle_nested_irq(sub_irq);

	return ret;
}

/* this interrupt is read to clear, in max77759 it should be write to clear */
static irqreturn_t max777x9_pmic_irq(int irq, void *ptr)
{
	struct max77729_pmic_data *data = ptr;
	uint8_t intsrc = 0, uic_int[4];
	int ret;

	/* INTSRC is read to clear on MW and max77729f */
	ret = max77729_pmic_rd8(data, MAX77729_PMIC_INTSRC, &intsrc);
	if (ret < 0) {
		dev_err_ratelimited(data->dev, "INTSRC: read error %d\n", ret);
		return IRQ_NONE;
	}

	if (intsrc == 0) {
		dev_err_ratelimited(data->dev, "%s intsrc 0\n", __func__);
		return IRQ_NONE;
	}

	/* just clear for max77729f */
	dev_info_ratelimited(data->dev, "irq=%d INTSRC:%x\n", irq, intsrc);
	if (data->pmic_id != MAX77759_PMIC_PMIC_ID_MW)
		return IRQ_HANDLED;

	/* UIC_INT are write to clear */
	if (intsrc & MAX77759_PMIC_INTSRC_MAXQ_INT) {
		ret = max77729_pmic_readn(data, MAX77759_PMIC_UIC_INT1,
					  uic_int, sizeof(uic_int));
		if (ret < 0) {
			dev_err_ratelimited(data->dev,
				"UIC_INT1: read error %d\n", ret);
			return IRQ_NONE;
		}

		/* TODO: implement / handle comms with maxq */
		if (uic_int[0] & MAX77759_PMIC_UIC_INT1_APCMDRESI) {
			maxq_irq(data->maxq);
			max77729_pmic_wr8(data, MAX77759_PMIC_UIC_INT1,
					  MAX77759_PMIC_UIC_INT1_APCMDRESI);
		}

		if (uic_int[0] & MAX77759_PMIC_UIC_INT1_GPIO5I)
			max777x9_pmic_route_irq(data, MAX77759_GPIO5_OFF);
		if (uic_int[0] & MAX77759_PMIC_UIC_INT1_GPIO6I)
			max777x9_pmic_route_irq(data, MAX77759_GPIO6_OFF);
	}

	if (intsrc & MAX77759_PMIC_TOPSYS_INT_SYSUVLO_INT)
		atomic_inc(&data->sysuvlo_cnt);

	if (intsrc & MAX77759_PMIC_TOPSYS_INT_SYSOVLO_INT)
		atomic_inc(&data->sysovlo_cnt);

	if (intsrc & MAX77759_PMIC_INTSRC_TOPSYS_INT) {
		uint8_t tsi;

		ret = max77729_pmic_rd8(data, MAX77729_PMIC_TOPSYS_INT, &tsi);
		if (ret < 0) {
			dev_err_ratelimited(data->dev,
					"TOPSYS_INT: read error %d\n", ret);
			return IRQ_NONE;
		}

		ret = max77729_pmic_wr8(data, MAX77729_PMIC_TOPSYS_INT, tsi);
		if (ret < 0) {
			dev_err_ratelimited(data->dev,
				"TOPSYS_INT:%x clr error %d\n", tsi, ret);
			return IRQ_NONE;
		}

		pr_info("TOPSYS_INT:%x\n", tsi);

		/* TODO: handle TSHDN_INT, SYSOVLO_INT, SYSUVLO_INT */
	}

	/* just clear CHG_INT, no FG intr for MW */

	return IRQ_HANDLED;
}

/*
 * Bootloader has everything masked clear this on boot
 * GPIO irqs are enabled later
 */
static int max777x9_pmic_set_irqmask(struct max77729_pmic_data *data)
{
	int ret;

	if (data->pmic_id == MAX77759_PMIC_PMIC_ID_MW) {
		const u8 uic_mask[] = {0x7f, 0xff, 0xff, 0xff};
		u8 reg;

		ret = max77729_pmic_rd8(data, MAX77759_PMIC_INTSRC, &reg);
		if (ret < 0 || reg)
			dev_info(data->dev, "INTSRC :%x (%d)\n", reg, ret);

		ret = max77729_pmic_rd8(data, MAX77759_PMIC_TOPSYS_INT, &reg);
		if (ret < 0 || reg)
			dev_info(data->dev, "TOPSYS_INT :%x (%d)\n", reg, ret);

		ret = max77729_pmic_wr8(data, MAX77759_PMIC_INTSRCMASK,
					MAX77759_PMIC_INTSRCMASK_DEFAULT);

		/* b/156527175, *_PMIC_TOPSYS_INT_MASK_SPR_7 is reserved */
		ret |= max77729_pmic_rmw8(data, MAX77759_PMIC_TOPSYS_INT_MASK,
					  MAX77759_PMIC_TOPSYS_INT_MASK_MASK,
					  MAX77759_PMIC_TOPSYS_INT_MASK_DEFAULT);

		/* clear all, unmask MAX77759_PMIC_UIC_INT1_APCMDRESI */
		ret |= max77729_pmic_wr8(data, MAX77759_PMIC_UIC_INT1,
					 MAX77759_PMIC_UIC_INT1_GPIO5I |
					 MAX77759_PMIC_UIC_INT1_GPIO6I |
					 MAX77759_PMIC_UIC_INT1_APCMDRESI);
		ret |= max77729_pmic_writen(data, MAX77759_PMIC_UIC_INT1_M,
					    uic_mask, sizeof(uic_mask));
	} else {
		ret = max77729_pmic_wr8(data, MAX77729_PMIC_INTSRCMASK,
					MAX77729_PMIC_INTSRCMASK_DEFAULT);
		ret |= max77729_pmic_wr8(data, MAX77729_PMIC_TOPSYS_INT_MASK,
					 MAX77729_PMIC_TOPSYS_INT_MASK_DEFAULT);
	}

	return ret ? -EIO : 0;
}

static int max77759_find_fg(struct max77729_pmic_data *data)
{
	struct device_node *dn;

	if (data->fg_i2c_client)
		return 0;

	dn = of_parse_phandle(data->dev->of_node, "max77759,max_m5", 0);
	if (!dn)
		return -ENXIO;

	data->fg_i2c_client = of_find_i2c_device_by_node(dn);
	if (!data->fg_i2c_client)
		return -EAGAIN;

	return 0;
}

#define NTC_CURVE_THRESHOLD	185
#define NTC_CURVE_1_BASE	960
#define NTC_CURVE_1_SHIFT	2
#define NTC_CURVE_2_BASE	730
#define NTC_CURVE_2_SHIFT	3

/*
 * WARNING: The FG will behave erratically when the recovery path fails.
 */
static int max77759_read_thm(struct max77729_pmic_data *data, int mux,
			     unsigned int *value)
{
	unsigned int ain0, config, check_config = 0;
	u8 pmic_ctrl, check_pmic = 0;
	int tmp, ret;

	if (!data->fg_i2c_client)
		return -EINVAL;

	/* TODO: prevent the FG from loading the FG model */

	/* set TEX=1 in Config 0x1D, make sure that TEN is enabled */
	ret = max_m5_reg_read(data->fg_i2c_client, MAX77759_FG_CONFIG, &config);
	if (ret == 0) {
		const u16 val = config | MAX77759_FG_CONFIG_TEN | MAX77759_FG_CONFIG_TEX;

		/* TEN should be enabled for this to work */
		WARN_ON(!(config & MAX77759_FG_CONFIG_TEN));

		ret = max_m5_reg_write(data->fg_i2c_client, MAX77759_FG_CONFIG,
				       val);

		pr_info("%s: config:%x->%x (%d)\n", __func__, config, val, ret);
	}
	if (ret == -ENODEV) {
		pr_err("%s: no support for max_m5 FG (%d)\n", __func__, ret);
		*value = 25;
		return 0;
	} else if (ret < 0) {
		pr_err("%s: cannot change FG Config (%d)\n", __func__, ret);
		return -EIO;
	}

	/* set THMIO_MUX */
	ret = max77729_pmic_rd8(data, MAX77759_PMIC_CONTROL_FG, &pmic_ctrl);
	if (ret == 0) {
		const u8 val = _pmic_control_fg_thmio_mux_set(pmic_ctrl, mux);

		ret = max77729_pmic_wr8(data, MAX77759_PMIC_CONTROL_FG, val);

		pr_info("%s: pmic_ctrl:%x->%x (%d)\n", __func__, pmic_ctrl, val, ret);
	}

	if (ret < 0) {
		pr_err("%s: cannot change MUX config (%d)\n", __func__, ret);
		goto restore_fg;
	}

	/* msleep is uninterruptible */
	msleep(1500);

	ret = max_m5_reg_read(data->fg_i2c_client, MAX77759_FG_AIN0, &ain0);
	pr_debug("%s: AIN0=%d (%d)\n", __func__, ain0, ret);
	if (ret < 0) {
		pr_err("%s: cannot read AIN0 (%d)\n", __func__, ret);
	} else if (mux == THMIO_MUX_USB_TEMP || mux == THMIO_MUX_BATT_PACK) {
		/* convert form 1.8V to 2.4V and get higher 10 bits */
		const unsigned int conv_adc = ((ain0 * 1800) / 2400) >> 6;

		/* Temp = (rawadc < 185)? (960-rawadc/4) : (730-rawadc/8) */
		/* unit: 0.1 degree C */
		if (conv_adc < NTC_CURVE_THRESHOLD)
			*value = NTC_CURVE_1_BASE - ((conv_adc * 10) >> NTC_CURVE_1_SHIFT);
		else
			*value = NTC_CURVE_2_BASE  - ((conv_adc * 10) >> NTC_CURVE_2_SHIFT);
	} else {
		/* AIN0 is ratiometric on THM, 0xffff = 100%, lsb is 2^-16 */
		*value = (100000 * (unsigned long)ain0) / (0x10000 - ain0);
	}

	/* restore THMIO_MUX */
	tmp = max77729_pmic_wr8(data, MAX77759_PMIC_CONTROL_FG, pmic_ctrl);
	WARN_ON(tmp != 0);

	/* And reset the pmic if cannot restore (b/191319560) */
	tmp = max77729_pmic_rd8(data, MAX77759_PMIC_CONTROL_FG, &check_pmic);
	if (tmp != 0 || pmic_ctrl != check_pmic) {
		dev_err(data->dev, "Cannot restore TMUX ret=%d\n", tmp);
		BUG_ON(tmp != 0 || pmic_ctrl != check_pmic);
	}

restore_fg:

	/* Clear TEX=0 in Config, restore 0x1D (b/191319560) */
	config &= ~MAX77759_FG_CONFIG_TEX;
	tmp = max_m5_reg_write(data->fg_i2c_client, MAX77759_FG_CONFIG, config);
	WARN_ON(tmp != 0);

	/* And reset the FG if this fails (b/191319560) */
	tmp = max_m5_reg_read(data->fg_i2c_client, MAX77759_FG_CONFIG, &check_config);
	if (tmp != 0 || config != check_config) {
		tmp = max17x0x_sw_reset(data->fg_i2c_client);
		dev_err(data->dev, "Cannot restore FG Config, FG reset ret=%d\n", tmp);
		BUG_ON(tmp != 0);
	} else if (!(check_config & MAX77759_FG_CONFIG_TEN)) {
		dev_warn(data->dev, "TEN bit is not set in Config=%x\n", check_config);
	}

	/* TODO: allow the FG to load the FG model */

	pr_info("%s: check_pmic=%x check_config=%x (%d)\n", __func__,
		check_pmic, check_config, ret);

	return ret;
}

/* THMIO_MUX=0 in CONTROL_FG (0x51) */
int max77759_read_batt_conn(struct i2c_client *client, int *temp)
{
	struct max77729_pmic_data *data = i2c_get_clientdata(client);
	unsigned int val;
	int ret;

	mutex_lock(&data->io_lock);
	ret = max77759_find_fg(data);
	if (ret == 0)
		ret = max77759_read_thm(data, THMIO_MUX_BATT_PACK, &val);
	mutex_unlock(&data->io_lock);

	if (ret == 0) {
		/* TODO: b/160737498 convert voltage to temperature */
		*temp = val;
	}

	return ret;
}
EXPORT_SYMBOL_GPL(max77759_read_batt_conn);

/* THMIO_MUX=1 in CONTROL_FG (0x51) */
int max77759_read_usb_temp(struct i2c_client *client, int *temp)
{
	struct max77729_pmic_data *data = i2c_get_clientdata(client);
	unsigned int val;
	int ret;

	mutex_lock(&data->io_lock);
	ret = max77759_find_fg(data);
	if (ret == 0)
		ret = max77759_read_thm(data, THMIO_MUX_USB_TEMP, &val);
	if (ret == 0)
		*temp = val;
	mutex_unlock(&data->io_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(max77759_read_usb_temp);

/* THMIO_MUX=2 in CONTROL_FG (0x51) */
int max77759_read_batt_id(struct i2c_client *client, unsigned int *id)
{
	struct max77729_pmic_data *data = i2c_get_clientdata(client);
	unsigned int val;
	int ret;

	mutex_lock(&data->io_lock);
	ret = max77759_find_fg(data);
	if (ret == 0)
		ret = max77759_read_thm(data, THMIO_MUX_BATT_ID, &val);
	if (ret == 0)
		*id = val;
	mutex_unlock(&data->io_lock);

	return 0;
}
EXPORT_SYMBOL_GPL(max77759_read_batt_id);

/* must use repeated starts b/152373060 */
static int max77729_pmic_read_id(struct i2c_client *i2c)
{
	struct i2c_msg xfer[2];
	u8 reg = MAX77729_PMIC_ID;
	u8 pmic_id;
	int ret;

	xfer[0].addr = i2c->addr;
	xfer[0].flags = 0;
	xfer[0].len = 1;
	xfer[0].buf = &reg;

	xfer[1].addr = i2c->addr;
	xfer[1].flags = I2C_M_RD;
	xfer[1].len = 1;
	xfer[1].buf = &pmic_id;

	ret = i2c_transfer(i2c->adapter, xfer, 2);
	if (ret == 2)
		return pmic_id;
	return -EIO;
}

/* cahe the value */
static int debug_batt_thm_id_get(void *d, u64 *val)
{
	struct max77729_pmic_data *data = d;
	int ret, value;

	ret = max77759_read_batt_id(data->pmic_i2c_client, &value);
	if (ret == 0)
		*val = value;
	return ret;
}
DEFINE_SIMPLE_ATTRIBUTE(debug_batt_thm_id_fops, debug_batt_thm_id_get, NULL, "%llu\n");

static int debug_batt_thm_conn_get(void *d, u64 *val)
{
	struct max77729_pmic_data *data = d;
	int ret, value;

	ret = max77759_read_batt_conn(data->pmic_i2c_client, &value);
	if (ret == 0)
		*val = value;

	return ret;
}
DEFINE_SIMPLE_ATTRIBUTE(debug_batt_thm_conn_fops, debug_batt_thm_conn_get, NULL, "%llu\n");

static int max777x9_pmic_debug_reg_read(void *d, u64 *val)
{
	struct max77729_pmic_data *data = d;
	u8 reg = 0;
	int ret;

	ret = max77729_pmic_rd8(data, data->debug_reg_address, &reg);
	if (ret)
		return ret;
	*val = reg;
	return 0;
}

static int max777x9_pmic_debug_reg_write(void *d, u64 val)
{
	struct max77729_pmic_data *data = d;
	u8 reg = (u8) val;

	pr_warn("debug write reg 0x%x, 0x%x", data->debug_reg_address, reg);
	return max77729_pmic_wr8(data, data->debug_reg_address, reg);
}
DEFINE_SIMPLE_ATTRIBUTE(debug_reg_rw_fops, max777x9_pmic_debug_reg_read,
			max777x9_pmic_debug_reg_write, "%02llx\n");

static ssize_t registers_dump_show(struct device *dev, struct device_attribute *attr,
				   char *buf)
{
	struct max77729_pmic_data *data = dev_get_drvdata(dev);
	static u8 *dump;
	int ret = 0, offset = 0, i;

	if (!data->regmap) {
		pr_err("Failed to read, no regmap\n");
		return -EIO;
	}

	mutex_lock(&data->reg_dump_lock);

	dump = kzalloc(MAX77729_PMIC_NUM_REGS * sizeof(u8), GFP_KERNEL);
	if (!dump) {
		dev_err(dev, "[%s]: Failed to allocate mem ret:%d\n", __func__, ret);
		goto unlock;
	}

	ret = max77729_pmic_readn(data, MAX77729_PMIC_ID, dump, MAX77729_PMIC_NUM_REGS);
	if (ret < 0) {
		dev_err(dev, "[%s]: Failed to dump ret:%d\n", __func__, ret);
		goto done;
	}

	for (i = 0; i < MAX77729_PMIC_NUM_REGS; i++) {
		u32 reg_address = i + MAX77729_PMIC_ID;

		if (!max77729_pmic_is_reg(dev, reg_address))
			continue;

		ret = sysfs_emit_at(buf, offset, "%02x: %02x\n", reg_address, dump[i]);
		if (!ret) {
			dev_err(dev, "[%s]: Not all registers printed. last:%x\n", __func__,
				reg_address - 1);
			break;
		}
		offset += ret;
	}

done:
	kfree(dump);
unlock:
	mutex_unlock(&data->reg_dump_lock);
	return offset;
}
static DEVICE_ATTR_RO(registers_dump);

static int max77759_pmic_storage_iter(int index, gbms_tag_t *tag, void *ptr)
{
	if (index < 0 || index > (GBMS_TAG_RRS7 - GBMS_TAG_RRS0))
		return -ENOENT;

	*tag = GBMS_TAG_RRS0 + index;
	return 0;
}

static int max77759_pmic_storage_read(gbms_tag_t tag, void *buff, size_t size, void *ptr)
{
	const int base = MAX77759_STORAGE_BASE + tag - GBMS_TAG_RRS0;
	struct max77729_pmic_data *data = ptr;
	int ret;

	if (tag < GBMS_TAG_RRS0 || tag > GBMS_TAG_RRS7)
		return -ENOENT;
	if ((tag + size - 1) > GBMS_TAG_RRS7)
		return -ERANGE;

	ret = max77729_pmic_readn(data, base, buff, size);
	if (ret < 0)
		ret = -EIO;
	return ret;
}

static int max77759_pmic_storage_write(gbms_tag_t tag, const void *buff, size_t size, void *ptr)
{
	const int base = MAX77759_STORAGE_BASE + tag - GBMS_TAG_RRS0;
	struct max77729_pmic_data *data = ptr;
	int ret;

	if (tag < GBMS_TAG_RRS0 || tag > GBMS_TAG_RRS7)
		return -ENOENT;
	if ((tag + size - 1) > GBMS_TAG_RRS7)
		return -ERANGE;

	ret = max77729_pmic_writen(data, base, buff, size);
	if (ret < 0)
		ret = -EIO;
	return ret;
}

static struct gbms_storage_desc max77759_pmic_storage_dsc = {
	.iter = max77759_pmic_storage_iter,
	.read = max77759_pmic_storage_read,
	.write = max77759_pmic_storage_write,
};

#define STORAGE_INIT_DELAY_MS	100
#define STORAGE_INIT_MAX_RETRY	3
static void max777x9_pmic_storage_init_work(struct work_struct *work)
{
	struct max77729_pmic_data *data = container_of(work, struct max77729_pmic_data,
						       storage_init_work.work);
	static int retry_cnt;
	int ret = 0;

	ret = gbms_storage_register(&max77759_pmic_storage_dsc,
				    "max777x9_pmic_storage", data);

	if (ret == 0) {
		pr_info("register storage done\n");
	} else if (retry_cnt >= STORAGE_INIT_MAX_RETRY) {
		pr_info("register storage:%d retry_cnt=%d, stop retry.\n", ret, retry_cnt);
	} else {
		schedule_delayed_work(&data->storage_init_work,
				      msecs_to_jiffies(STORAGE_INIT_DELAY_MS));
		retry_cnt++;
	}

	return;
}

static int dbg_init_fs(struct max77729_pmic_data *data)
{
	int ret;

	ret = device_create_file(data->dev, &dev_attr_registers_dump);
	if (ret != 0)
		dev_warn(data->dev, "Failed to create registers_dump, ret=%d\n", ret);

	data->de = debugfs_create_dir("max77729_pmic", 0);
	if (IS_ERR_OR_NULL(data->de))
		return -EINVAL;

	debugfs_create_atomic_t("sysuvlo_cnt", 0644, data->de, &data->sysuvlo_cnt);
	debugfs_create_atomic_t("sysovlo_cnt", 0644, data->de, &data->sysovlo_cnt);

	debugfs_create_file("batt_id", 0400, data->de, data,
			    &debug_batt_thm_id_fops);
	debugfs_create_file("batt_thm_conn", 0400, data->de, data,
			    &debug_batt_thm_conn_fops);

	debugfs_create_u32("address", 0600, data->de, &data->debug_reg_address);
	debugfs_create_file("data", 0600, data->de, data, &debug_reg_rw_fops);

	return 0;
}

#if IS_ENABLED(CONFIG_GPIOLIB)

/* offset is gpionum - 1 */
static int max77759_gpio_get_direction(struct gpio_chip *chip,
				       unsigned int offset)
{
	struct max77729_pmic_data *data = gpiochip_get_data(chip);
	int rc;
	uint8_t val;

	if ((offset < MAX77759_MIN_GPIO_OFF) || (offset > MAX77759_MAX_GPIO_OFF))
		return -EINVAL;

	rc = maxq_gpio_control_read(data->maxq, &val);
	if (rc < 0) {
		dev_err(data->dev, "opcode read 0x23 failed\n");
		return rc;
	}

	if (offset == MAX77759_GPIO5_OFF)
		return !(val & MAX77759_GPIO5_DIR_MASK);

	return !(val & MAX77759_GPIO6_DIR_MASK);
}

/* offset is gpionum - 1 */
static int max77759_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	struct max77729_pmic_data *data = gpiochip_get_data(chip);
	int rc;
	uint8_t val;

	if ((offset < MAX77759_MIN_GPIO_OFF) || (offset > MAX77759_MAX_GPIO_OFF))
		return -EINVAL;

	rc = maxq_gpio_control_read(data->maxq, &val);
	if (rc < 0) {
		dev_err(data->dev, "opcode read 0x23 failed\n");
		return rc;
	}

	if (offset == MAX77759_GPIO5_OFF)
		return !!(val & MAX77759_GPIO5_VAL_MASK);

	return !!(val & MAX77759_GPIO6_VAL_MASK);
}

/* offset is gpionum - 1 */
static void max77759_gpio_set(struct gpio_chip *chip,
			      unsigned int offset, int value)
{
	struct max77729_pmic_data *data = gpiochip_get_data(chip);
	int rc;
	uint8_t val;
	uint8_t new_val;
	uint8_t dir;

	if ((offset < MAX77759_MIN_GPIO_OFF) || (offset > MAX77759_MAX_GPIO_OFF))
		return;

	rc = maxq_gpio_control_read(data->maxq, &val);
	if (rc < 0) {
		dev_err(data->dev, "opcode read 0x23 failed\n");
		return;
	}

	if (offset == MAX77759_GPIO5_OFF) {
		dir = !(val & MAX77759_GPIO5_DIR_MASK);
		if (dir != GPIOF_DIR_OUT)  {
			dev_err(data->dev, "not output\n");
			return;
		}
		new_val = val & ~MAX77759_GPIO5_VAL_MASK;
		new_val |= MAX77759_GPIO5_VAL(value);
	} else {  /* MAX77759_GPIO6_OFF */
		dir = !(val & MAX77759_GPIO6_DIR_MASK);
		if (dir != GPIOF_DIR_OUT)  {
			dev_err(data->dev, "not output\n");
			return;
		}
		new_val = val & ~MAX77759_GPIO6_VAL_MASK;
		new_val |= MAX77759_GPIO6_VAL(value);
	}

	if (new_val != val) {
		rc = maxq_gpio_control_write(data->maxq, new_val);
		if (rc < 0) {
			dev_err(data->dev, "opcode write 0x24 failed\n");
			return;
		}
	}
}

/* offset is gpionum - 1 */
static int max77759_gpio_direction_input(struct gpio_chip *chip,
					 unsigned int offset)
{
	struct max77729_pmic_data *data = gpiochip_get_data(chip);
	int rc;
	uint8_t val;
	uint8_t new_val;

	if ((offset < MAX77759_MIN_GPIO_OFF) || (offset > MAX77759_MAX_GPIO_OFF))
		return -EINVAL;

	rc = maxq_gpio_control_read(data->maxq, &val);
	if (rc < 0) {
		dev_err(data->dev, "opcode read 0x23 failed\n");
		return rc;
	}

	if (offset == MAX77759_GPIO5_OFF) {
		new_val = val & ~MAX77759_GPIO5_DIR_MASK;
		new_val |= MAX77759_GPIO5_DIR(MAX77759_GPIO_DIR_IN);
	} else { /* MAX77759_GPIO6_OFF */
		new_val = val & ~MAX77759_GPIO6_DIR_MASK;
		new_val |= MAX77759_GPIO6_DIR(MAX77759_GPIO_DIR_IN);
	}

	if (new_val != val) {
		rc = maxq_gpio_control_write(data->maxq, new_val);
		if (rc < 0) {
			dev_err(data->dev, "opcode write 0x24 failed\n");
			return rc;
		}
	}

	return 0;
}

/* offset is gpionum - 1 */
static int max77759_gpio_direction_output(struct gpio_chip *chip,
					 unsigned int offset, int value)
{
	struct max77729_pmic_data *data = gpiochip_get_data(chip);
	int rc;
	uint8_t val;
	uint8_t new_val;

	if ((offset < MAX77759_MIN_GPIO_OFF) || (offset > MAX77759_MAX_GPIO_OFF))
		return -EINVAL;

	rc = maxq_gpio_control_read(data->maxq, &val);
	if (rc < 0) {
		dev_err(data->dev, "opcode read 0x23 failed\n");
		return rc;
	}

	if (offset == MAX77759_GPIO5_OFF) {
		new_val = val & ~MAX77759_GPIO5_DIR_MASK;
		new_val |= MAX77759_GPIO5_DIR(MAX77759_GPIO_DIR_OUT);
	} else { /* MAX77759_GPIO6_OFF */
		new_val = val & ~MAX77759_GPIO6_DIR_MASK;
		new_val |= MAX77759_GPIO6_DIR(MAX77759_GPIO_DIR_OUT);
	}

	if (new_val != val) {
		rc = maxq_gpio_control_write(data->maxq, new_val);
		if (rc < 0) {
			dev_err(data->dev, "opcode write 0x24 failed\n");
			return rc;
		}
	}

	return 0;
}

/* d->hwirq is same as offset */
static void max77729_gpio_irq_mask(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct max77729_pmic_data *data = gpiochip_get_data(gc);

	data->irq_mask |= 1 << d->hwirq;
	data->irq_mask_u |= 1 << d->hwirq;
}

static void max77729_gpio_irq_unmask(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct max77729_pmic_data *data = gpiochip_get_data(gc);

	data->irq_mask &= ~(1 << d->hwirq);
	data->irq_mask_u |= 1 << d->hwirq;
}

static void max77729_gpio_irq_enable(struct irq_data *d)
{
	max77729_gpio_irq_unmask(d);
}

static void max77729_gpio_irq_disable(struct irq_data *d)
{
	max77729_gpio_irq_mask(d);
}

/* called in atomic context */
static int max77729_gpio_set_irq_type(struct irq_data *d, unsigned int type)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct max77729_pmic_data *data = gpiochip_get_data(gc);
	const int index = d->hwirq - MAX77759_GPIO5_OFF;

	switch (type) {
	case IRQF_TRIGGER_FALLING:
		data->irq_trig_falling[index] = 1;
		break;
	case IRQF_TRIGGER_RISING:
		data->irq_trig_falling[index] = 0;
		break;
	default:
		return -EINVAL;
	}

	data->irq_trig_u |= 1 << d->hwirq;
	return 0;
}

static void max77729_gpio_bus_lock(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct max77729_pmic_data *data = gpiochip_get_data(gc);

	mutex_lock(&data->irq_lock);
}

static int max77729_gpio_bus_irq_update_trig(struct max77729_pmic_data *data,
					     int offset, int trig)
{
	int ret;

	/* direction works with offset, trigger works with gpio number */
	ret = max77759_gpio_direction_input(&data->gpio, offset);
	if (ret == 0)
		ret = 	maxq_gpio_trigger_write(data->maxq, offset + 1, trig);

	pr_debug("gpio%d: trig=%d (%d)\n", offset + 1, trig, ret);

	return ret;
}

static int max77729_gpio_bus_irq_update_mask(struct max77729_pmic_data *data,
					     int offset, int value)
{
	unsigned int mask = 0, val = 0;
	int ret;

	ret = max77729_gpio_to_irq_mask(offset, &mask, &val);
	if (ret == 0)
		ret = max77729_pmic_rmw8(data, MAX77759_PMIC_UIC_INT1_M, mask,
					 value ? val : 0);
	if (ret < 0) {
		dev_err(data->dev, "gpio%d: cannot change mask=%x to %x (%d)\n",
			offset + 1, mask, value ? val : 0, ret);
		return ret;
	}

	pr_debug("gpio%d: value=%d mask=%x, val=%x (%d)\n",
		 offset +1, value, mask, val, ret);

	return 0;
}

/* cannot call any maxq function in atomic */
static void max77729_gpio_bus_sync_unlock(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct max77729_pmic_data *data = gpiochip_get_data(gc);
	unsigned int offset, value;

	while (data->irq_trig_u) {
		offset = __ffs(data->irq_trig_u);
		value = data->irq_trig_falling[offset - MAX77759_GPIO5_OFF];

		max77729_gpio_bus_irq_update_trig(data, offset, value);
		data->irq_trig_u &= ~(1 << offset);
	}

	while (data->irq_mask_u) {
		offset = __ffs(data->irq_mask_u);
		value = data->irq_mask & (1 << offset);

		max77729_gpio_bus_irq_update_mask(data, offset, value);
		data->irq_mask_u &= ~(1 << offset);
	}

	mutex_unlock(&data->irq_lock);
}

/* bits 4 and 5 */
static void max77729_gpio_set_irq_valid_mask(struct gpio_chip *chip,
					     unsigned long *valid_mask,
					     unsigned int ngpios)
{
	bitmap_clear(valid_mask, 0, ngpios);
	*valid_mask = (1 << MAX77759_GPIO5_OFF) | (1 << MAX77759_GPIO6_OFF);
}

/* only support 5 and 6, 5 is output */
static int max77729_gpio_irq_init_hw(struct gpio_chip *gc)
{
	struct max77729_pmic_data *data = gpiochip_get_data(gc);
	const u8 mask_gpio = MAX77759_PMIC_UIC_INT1_GPIO5I |
			    MAX77759_PMIC_UIC_INT1_GPIO6I;
	int ret;

	/* mask both */
	ret = max77729_pmic_rmw8(data, MAX77759_PMIC_UIC_INT1_M,
				 mask_gpio, mask_gpio);
	if (ret < 0)
		dev_err(data->dev, "cannot mask IRQs\n");

	/* ...and clear both */
	ret = max77729_pmic_wr8(data, MAX77759_PMIC_UIC_INT1,
				MAX77759_PMIC_UIC_INT1_GPIO5I |
				MAX77759_PMIC_UIC_INT1_GPIO6I);
	if (ret < 0)
		dev_err(data->dev, "cannot clear IRQs\n");

	return 0;
}
static struct irq_chip max77729_gpio_irq_chip = {
	.name		= "max777x9_irq",
	.irq_enable	= max77729_gpio_irq_enable,
	.irq_disable	= max77729_gpio_irq_disable,
	.irq_mask	= max77729_gpio_irq_mask,
	.irq_unmask	= max77729_gpio_irq_unmask,
	.irq_set_type	=max77729_gpio_set_irq_type,
	.irq_bus_lock = max77729_gpio_bus_lock,
	.irq_bus_sync_unlock = max77729_gpio_bus_sync_unlock,
};

#endif

/* ----------------------------------------------------------------------- */


static int max77729_pmic_probe(struct i2c_client *client,
			       const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct max77729_pmic_data *data;
	int irq_gpio, pmic_id, ret =0;

	pmic_id = max77729_pmic_read_id(client);
	if (pmic_id < 0)
		return -ENODEV;
	if (pmic_id == MAX77759_PMIC_PMIC_ID_MW)
		max777x9_pmic_regmap_cfg.max_register = 0xe0;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->dev = dev;
	data->pmic_id = pmic_id;
	data->batt_id = -1;
	mutex_init(&data->io_lock);
	mutex_init(&data->reg_dump_lock);
	atomic_set(&data->sysuvlo_cnt, 0);
	atomic_set(&data->sysovlo_cnt, 0);
	i2c_set_clientdata(client, data);
	data->pmic_i2c_client = client;

	INIT_DELAYED_WORK(&data->storage_init_work, max777x9_pmic_storage_init_work);

	data->regmap = devm_regmap_init_i2c(client, &max777x9_pmic_regmap_cfg);
	if (IS_ERR(data->regmap)) {
		dev_err(dev, "Failed to initialize regmap\n");
		return -EINVAL;
	}

	if (pmic_id == MAX77759_PMIC_PMIC_ID_MW) {
		const int poll_en = of_property_read_bool(dev->of_node,
							  "goog,maxq-poll");
		u8 pmic_ctrl;


		data->maxq = maxq_init(dev, data->regmap, poll_en);
		if (IS_ERR_OR_NULL(data->maxq)) {
			dev_err(dev, "Maxq init failed!\n");
			ret = PTR_ERR(data->maxq);
		}

		ret = max77729_pmic_rd8(data, MAX77759_PMIC_CONTROL_FG, &pmic_ctrl);
		WARN_ON(ret != 0 ||((pmic_ctrl & MAX77759_PMIC_CONTROL_FG_THMIO_MUX_MASK)
			!= THMIO_MUX_BATT_PACK));
		if (ret == 0) {
			const u8 val = _pmic_control_fg_thmio_mux_set(pmic_ctrl,
								      THMIO_MUX_BATT_PACK);

			ret = max77729_pmic_wr8(data, MAX77759_PMIC_CONTROL_FG, val);
			WARN_ON(ret != 0);
		}
	}

	irq_gpio = of_get_named_gpio(dev->of_node, "max777x9,irq-gpio", 0);
	if (irq_gpio < 0) {
		dev_err(dev, "irq is not defined\n");
	} else {
		client->irq = gpio_to_irq(irq_gpio);

		/* NOTE: all interrupts are masked here */
		ret = devm_request_threaded_irq(data->dev, client->irq, NULL,
						max777x9_pmic_irq,
						IRQF_TRIGGER_LOW |
						IRQF_SHARED |
						IRQF_ONESHOT,
						"max777x9_pmic",
						data);
		if (ret < 0) {
			dev_err(dev, "failed get irq thread\n");
		} else {
			/* force clear pending before unmasking */
			max777x9_pmic_irq(0, data);

			/* NOTE: only enable the maxq interrupt */
			ret = max777x9_pmic_set_irqmask(data);
			if (ret < 0)
				dev_err(dev, "failed to apply irq mask\n");
		}
	}

	ret = dbg_init_fs(data);
	if (ret < 0)
		dev_err(dev, "Failed to initialize debug fs\n");

	if (pmic_id == MAX77759_PMIC_PMIC_ID_MW) {
		u8 rev_reg;
		int rc = 0;

		rc = max77729_pmic_rd8(data, MAX77759_PMIC_PMIC_REVISION, &rev_reg);
		if (rc < 0) {
			dev_err(dev, "Failed to read revision\n");
			data->rev_id = 0;
		} else {
			data->rev_id = _pmic_pmic_revision_rev_get(rev_reg);
		}

		if (data->rev_id == MAX77759_PMIC_REV_A0)
			schedule_delayed_work(&data->storage_init_work, 0);
	}

#if IS_ENABLED(CONFIG_GPIOLIB)
	mutex_init(&data->irq_lock);

	if (pmic_id == MAX77759_PMIC_PMIC_ID_MW) {
		struct gpio_irq_chip *girq = &data->gpio.irq;

		/* Setup GPIO controller */
		data->gpio.owner = THIS_MODULE;
		data->gpio.parent = dev;
		data->gpio.label = "max777x9_gpio";
		data->gpio.get_direction = max77759_gpio_get_direction;
		data->gpio.direction_input = max77759_gpio_direction_input;
		data->gpio.direction_output = max77759_gpio_direction_output;
		data->gpio.get = max77759_gpio_get;
		data->gpio.set = max77759_gpio_set;
		data->gpio.ngpio = MAX77759_NUM_GPIOS;
		data->gpio.can_sleep = true;
		data->gpio.base	= -1;
		data->gpio.of_node = of_find_node_by_name(dev->of_node,
							  data->gpio.label);
		if (!data->gpio.of_node)
			dev_err(dev, "Failed to find %s DT node\n", data->gpio.label);

		/* check regmap-irq */
		girq->chip = &max77729_gpio_irq_chip;
		girq->default_type = IRQ_TYPE_NONE;
		girq->handler = handle_simple_irq;
		girq->parent_handler = NULL;
		girq->num_parents = 0;
		girq->parents = NULL;
		girq->threaded = true;
		girq->init_hw = max77729_gpio_irq_init_hw;
		girq->init_valid_mask = max77729_gpio_set_irq_valid_mask;
		girq->first = 0;

		ret = devm_gpiochip_add_data(dev, &data->gpio, data);
		if (ret)
			dev_err(dev, "Failed to initialize gpio chip\n");

	}
#endif

	dev_info(dev, "probe_done pmic_id = %x, rev_id= %x\n", pmic_id, data->rev_id);
	return ret;
}

static void max77729_pmic_remove(struct i2c_client *client)
{
	struct max77729_pmic_data *data = i2c_get_clientdata(client);

	maxq_remove(data->maxq);
}

static const struct of_device_id max77729_pmic_of_match_table[] = {
	{ .compatible = "maxim,max77729pmic" },
	{ .compatible = "maxim,max77759pmic" },
	{},
};
MODULE_DEVICE_TABLE(of, max77729_pmic_of_match_table);

static const struct i2c_device_id max77729_pmic_id[] = {
	{"max77729_pmic", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, max77729_pmic_id);

static struct i2c_driver max77729_pmic_i2c_driver = {
	.driver = {
		.name = "max777x9-pmic",
		.owner = THIS_MODULE,
		.of_match_table = max77729_pmic_of_match_table,
	},
	.id_table = max77729_pmic_id,
	.probe = max77729_pmic_probe,
	.remove = max77729_pmic_remove,
};

module_i2c_driver(max77729_pmic_i2c_driver);
MODULE_DESCRIPTION("Maxim 77729 PMIC driver");
MODULE_LICENSE("GPL");
