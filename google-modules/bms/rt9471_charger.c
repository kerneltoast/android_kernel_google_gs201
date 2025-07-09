// SPDX-License-Identifier: GPL-2.0
/*
 *  Driver for Richtek RT9471 Charger
 *
 *  Copyright (C) 2018 Richtek Technology Corp.
 *  shufan_lee <shufan_lee@richtek.com>
 *  lucas_tsai <lucas_tsai@richtek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include <linux/i2c.h>
#include <linux/of_device.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/gpio.h>
#include <linux/gpio/driver.h>
#include <linux/regmap.h>
#include "rt9471_charger.h"
#include "gbms_power_supply.h"
#include "google_psy.h"
#define RT9471_DRV_VERSION	"1.0.3_G"

#define RT9471_GPIO_USB_OTG_EN	0
#define RT9471_NUM_GPIOS	1

enum rt9471_stat_idx {
	RT9471_STATIDX_STAT0 = 0,
	RT9471_STATIDX_STAT1,
	RT9471_STATIDX_STAT2,
	RT9471_STATIDX_STAT3,
	RT9471_STATIDX_MAX,
};

enum rt9471_irq_idx {
	RT9471_IRQIDX_IRQ0 = 0,
	RT9471_IRQIDX_IRQ1,
	RT9471_IRQIDX_IRQ2,
	RT9471_IRQIDX_IRQ3,
	RT9471_IRQIDX_MAX,
};

enum rt9471_ic_stat {
	RT9471_ICSTAT_SLEEP = 0,
	RT9471_ICSTAT_VBUSRDY,
	RT9471_ICSTAT_TRICKLECHG,
	RT9471_ICSTAT_PRECHG,
	RT9471_ICSTAT_FASTCHG,
	RT9471_ICSTAT_IEOC,
	RT9471_ICSTAT_BGCHG,
	RT9471_ICSTAT_CHGDONE,
	RT9471_ICSTAT_CHGFAULT,
	RT9471_ICSTAT_OTG = 15,
	RT9471_ICSTAT_MAX,
};

static const char *rt9471_ic_stat_name[RT9471_ICSTAT_MAX] = {
	"hz/sleep", "ready", "trickle-charge", "pre-charge",
	"fast-charge", "ieoc-charge", "background-charge",
	"done", "fault", "RESERVED", "RESERVED", "RESERVED",
	"RESERVED", "RESERVED", "RESERVED", "OTG",
};

enum rt9471_mivr_track {
	RT9471_MIVRTRACK_REG = 0,
	RT9471_MIVRTRACK_VBAT_200MV,
	RT9471_MIVRTRACK_VBAT_250MV,
	RT9471_MIVRTRACK_VBAT_300MV,
	RT9471_MIVRTRACK_MAX,
};

enum rt9471_port_stat {
	RT9471_PORTSTAT_NOINFO = 0,
	RT9471_PORTSTAT_APPLE_10W = 8,
	RT9471_PORTSTAT_SAMSUNG_10W,
	RT9471_PORTSTAT_APPLE_5W,
	RT9471_PORTSTAT_APPLE_12W,
	RT9471_PORTSTAT_NSDP,
	RT9471_PORTSTAT_SDP,
	RT9471_PORTSTAT_CDP,
	RT9471_PORTSTAT_DCP,
	RT9471_PORTSTAT_MAX,
};

struct rt9471_desc {
	u32 ichg;
	u32 aicr;
	u32 mivr;
	u32 vac_ovp;
	u32 cv;
	u32 ieoc;
	u32 safe_tmr;
	u32 wdt;
	u32 mivr_track;
	bool en_safe_tmr;
	bool en_te;
	bool en_jeita;
	bool dis_i2c_tout;
	bool en_qon_rst;
	bool auto_aicr;
	const char *chg_name;
};

/* These default values will be applied if there's no property in dts */
static struct rt9471_desc rt9471_default_desc = {
	.ichg = 2000000,
	.aicr = 500000,
	.mivr = 4500000,
	.vac_ovp = 6500,
	.cv = 4200000,
	.ieoc = 200000,
	.safe_tmr = 10,
	.wdt = 40,
	.mivr_track = RT9471_MIVRTRACK_REG,
	.en_safe_tmr = true,
	.en_te = true,
	.en_jeita = true,
	.dis_i2c_tout = false,
	.en_qon_rst = true,
	.auto_aicr = true,
	.chg_name = "rt9471",
};

static const u8 rt9471_irq_maskall[RT9471_IRQIDX_MAX] = {
	0xFF, 0xFF, 0xFF, 0xFF,
};

static const u32 rt9471_wdt[] = {
	0, 40, 80, 160,
};

static const u32 rt9471_vac_ovp[] = {
	5800, 6500, 10900, 14000,
};

static const u8 rt9471_val_en_hidden_mode[] = {
	0x69, 0x96,
};

static const char *rt9471_port_name[RT9471_PORTSTAT_MAX] = {
	"NOINFO",
	"RESERVED", "RESERVED", "RESERVED", "RESERVED",
	"RESERVED", "RESERVED", "RESERVED",
	"APPLE_10W",
	"SAMSUNG_10W",
	"APPLE_5W",
	"APPLE_12W",
	"NSDP",
	"SDP",
	"CDP",
	"DCP",
};

struct rt9471_chip {
	struct i2c_client *client;
	struct device *dev;
	struct mutex io_lock;
	struct mutex bc12_lock;
	struct mutex hidden_mode_lock;
	int hidden_mode_cnt;
	u8 dev_id;
	u8 dev_rev;
	u8 chip_rev;
	struct rt9471_desc *desc;
	u32 intr_gpio;
	u32 ceb_gpio;
	int irq;
	u8 irq_mask[RT9471_IRQIDX_MAX];
	struct work_struct init_work;
	atomic_t vbus_gd;
	bool attach;
	enum rt9471_port_stat port;
	struct power_supply *psy;
	struct power_supply_desc psy_desc;
	struct power_supply_config psy_cfg;
	bool chg_done_once;
	struct delayed_work buck_dwork;
	struct regmap *rm_dev;
	u32 rten_gpio_default;
#if IS_ENABLED(CONFIG_GPIOLIB)
	struct gpio_chip gpio;
#endif
};

static const u8 rt9471_reg_addr[] = {
	RT9471_REG_OTGCFG,
	RT9471_REG_TOP,
	RT9471_REG_FUNCTION,
	RT9471_REG_IBUS,
	RT9471_REG_VBUS,
	RT9471_REG_PRECHG,
	RT9471_REG_REGU,
	RT9471_REG_VCHG,
	RT9471_REG_ICHG,
	RT9471_REG_CHGTIMER,
	RT9471_REG_EOC,
	RT9471_REG_INFO,
	RT9471_REG_JEITA,
	RT9471_REG_DPDMDET,
	RT9471_REG_STATUS,
	RT9471_REG_STAT0,
	RT9471_REG_STAT1,
	RT9471_REG_STAT2,
	RT9471_REG_STAT3,
	/* Skip IRQs to prevent reading clear while dumping registers */
	RT9471_REG_MASK0,
	RT9471_REG_MASK1,
	RT9471_REG_MASK2,
	RT9471_REG_MASK3,
};

static bool rt9471_is_reg(struct device *dev, unsigned int reg)
{
	return (reg >= RT9471_REG_OTGCFG && reg <= RT9471_REG_BUCK_HDEN5);
}

static const struct regmap_config rt9471_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xAA,
	.cache_type = REGCACHE_NONE,
	.readable_reg = rt9471_is_reg,
	.volatile_reg = rt9471_is_reg,
};

static int rt9471_register_regmap(struct rt9471_chip *chip)
{
	dev_info(chip->dev, "%s\n", __func__);

	chip->rm_dev = devm_regmap_init_i2c(chip->client,
					    &rt9471_regmap_config);
	if (IS_ERR(chip->rm_dev)) {
		dev_notice(chip->dev, "%s fail(%ld)\n",
				      __func__, PTR_ERR(chip->rm_dev));
		return -EIO;
	}

	return 0;
}

static inline int __rt9471_i2c_write_byte(struct rt9471_chip *chip, u8 cmd,
					  u8 data)
{
	int ret;

	ret = regmap_write(chip->rm_dev, cmd, data);
	if (ret < 0)
		dev_notice(chip->dev, "%s reg0x%02X = 0x%02X fail(%d)\n",
				      __func__, cmd, data, ret);
	else
		dev_dbg(chip->dev, "%s reg0x%02X = 0x%02X\n", __func__, cmd,
			data);

	return ret;
}

static int rt9471_i2c_write_byte(struct rt9471_chip *chip, u8 cmd, u8 data)
{
	int ret;

	mutex_lock(&chip->io_lock);
	ret = __rt9471_i2c_write_byte(chip, cmd, data);
	mutex_unlock(&chip->io_lock);

	return ret;
}

static inline int __rt9471_i2c_read_byte(struct rt9471_chip *chip, u8 cmd,
					 u8 *data)
{
	int ret;
	unsigned int regval;

	ret = regmap_read(chip->rm_dev, cmd, &regval);
	if (ret < 0) {
		dev_notice(chip->dev, "%s reg0x%02X fail(%d)\n",
				      __func__, cmd, ret);
		return ret;
	}

	dev_dbg(chip->dev, "%s reg0x%02X = 0x%02X\n", __func__, cmd, regval);
	*data = regval & 0xFF;
	return 0;
}

static int rt9471_i2c_read_byte(struct rt9471_chip *chip, u8 cmd, u8 *data)
{
	int ret;

	mutex_lock(&chip->io_lock);
	ret = __rt9471_i2c_read_byte(chip, cmd, data);
	mutex_unlock(&chip->io_lock);

	return ret;
}

static int rt9471_i2c_block_write(struct rt9471_chip *chip, u8 cmd, u32 len,
				  const u8 *data)
{
	int ret;

	mutex_lock(&chip->io_lock);
	ret = regmap_bulk_write(chip->rm_dev, cmd, data, len);
	mutex_unlock(&chip->io_lock);

	return ret;
}

static int rt9471_i2c_block_read(struct rt9471_chip *chip, u8 cmd, u32 len,
				 u8 *data)
{
	int ret;

	mutex_lock(&chip->io_lock);
	ret = regmap_bulk_read(chip->rm_dev, cmd, data, len);
	mutex_unlock(&chip->io_lock);

	return ret;
}

static int rt9471_i2c_test_bit(struct rt9471_chip *chip, u8 cmd, u8 shift,
			       bool *is_one)
{
	int ret;
	u8 regval;

	ret = rt9471_i2c_read_byte(chip, cmd, &regval);
	if (ret < 0) {
		*is_one = false;
		return ret;
	}

	regval &= 1 << shift;
	*is_one = (regval ? true : false);

	return ret;
}

static int rt9471_i2c_update_bits(struct rt9471_chip *chip, u8 cmd, u8 data,
				  u8 mask)
{
	int ret;
	u8 regval;

	mutex_lock(&chip->io_lock);
	ret = __rt9471_i2c_read_byte(chip, cmd, &regval);
	if (ret < 0)
		goto out;

	regval &= ~mask;
	regval |= (data & mask);

	ret = __rt9471_i2c_write_byte(chip, cmd, regval);
out:
	mutex_unlock(&chip->io_lock);
	return ret;
}

static inline int rt9471_set_bit(struct rt9471_chip *chip, u8 cmd, u8 mask)
{
	return rt9471_i2c_update_bits(chip, cmd, mask, mask);
}

static inline int rt9471_clr_bit(struct rt9471_chip *chip, u8 cmd, u8 mask)
{
	return rt9471_i2c_update_bits(chip, cmd, 0x00, mask);
}

static inline u8 rt9471_closest_reg(u32 min, u32 max, u32 step, u32 target)
{
	if (target < min)
		return 0;

	if (target >= max)
		return (max - min) / step;

	return (target - min) / step;
}

static inline u8 rt9471_closest_reg_via_tbl(const u32 *tbl, u32 tbl_size,
					    u32 target)
{
	u32 i;

	if (target < tbl[0])
		return 0;

	for (i = 0; i < tbl_size - 1; i++) {
		if (target >= tbl[i] && target < tbl[i + 1])
			return i;
	}

	return tbl_size - 1;
}

static inline u32 rt9471_closest_value(u32 min, u32 max, u32 step, u8 regval)
{
	u32 val;

	val = min + regval * step;
	if (val > max)
		val = max;

	return val;
}

static bool rt9471_is_vbusgd(struct rt9471_chip *chip)
{
	int ret;
	bool vbus_gd = false;

	ret = rt9471_i2c_test_bit(chip, RT9471_REG_STAT0,
				  RT9471_ST_VBUSGD_SHIFT, &vbus_gd);
	if (ret < 0)
		dev_notice(chip->dev, "%s check stat fail(%d)\n",
				      __func__, ret);
	dev_dbg(chip->dev, "%s vbus_gd = %d\n", __func__, vbus_gd);

	return vbus_gd;
}

static int rt9471_enable_bc12(struct rt9471_chip *chip, bool en)
{
	int ret;

	if (chip->dev_id != RT9470D_DEVID && chip->dev_id != RT9471D_DEVID)
		return 0;

	dev_info(chip->dev, "%s en = %d\n", __func__, en);

	if (en)
		ret = rt9471_set_bit(chip, RT9471_REG_DPDMDET, RT9471_BC12_EN_MASK);
	else
		ret = rt9471_clr_bit(chip, RT9471_REG_DPDMDET, RT9471_BC12_EN_MASK);

	return ret;
}

static int rt9471_enable_hidden_mode(struct rt9471_chip *chip, bool en)
{
	int ret = 0;

	mutex_lock(&chip->hidden_mode_lock);

	if (en) {
		if (chip->hidden_mode_cnt == 0) {
			ret = rt9471_i2c_block_write(chip, 0xA0,
				ARRAY_SIZE(rt9471_val_en_hidden_mode),
				rt9471_val_en_hidden_mode);
			if (ret < 0)
				goto err;
		}
		chip->hidden_mode_cnt++;
	} else {
		if (chip->hidden_mode_cnt == 1) /* last one */
			ret = rt9471_i2c_write_byte(chip, 0xA0, 0x00);
		chip->hidden_mode_cnt--;
		if (ret < 0)
			goto err;
	}
	dev_dbg(chip->dev, "%s en = %d, cnt = %d\n", __func__,
			   en, chip->hidden_mode_cnt);
	goto out;

err:
	dev_notice(chip->dev, "%s en = %d fail(%d)\n", __func__, en, ret);
out:
	mutex_unlock(&chip->hidden_mode_lock);
	return ret;
}

static int __rt9471_get_ic_stat(struct rt9471_chip *chip,
				enum rt9471_ic_stat *stat)
{
	int ret;
	u8 regval;

	ret = rt9471_i2c_read_byte(chip, RT9471_REG_STATUS, &regval);
	if (ret < 0)
		return ret;
	*stat = (regval & RT9471_ICSTAT_MASK) >> RT9471_ICSTAT_SHIFT;

	return 0;
}

static int __rt9471_get_mivr(struct rt9471_chip *chip, u32 *mivr)
{
	int ret;
	u8 regval;

	ret = rt9471_i2c_read_byte(chip, RT9471_REG_VBUS, &regval);
	if (ret < 0)
		return ret;

	regval = (regval & RT9471_MIVR_MASK) >> RT9471_MIVR_SHIFT;
	*mivr = rt9471_closest_value(RT9471_MIVR_MIN, RT9471_MIVR_MAX,
				     RT9471_MIVR_STEP, regval);

	return 0;
}

static int __rt9471_get_ichg(struct rt9471_chip *chip, u32 *ichg)
{
	int ret;
	u8 regval;

	ret = rt9471_i2c_read_byte(chip, RT9471_REG_ICHG, &regval);
	if (ret < 0)
		return ret;

	regval = (regval & RT9471_ICHG_MASK) >> RT9471_ICHG_SHIFT;
	*ichg = rt9471_closest_value(RT9471_ICHG_MIN, RT9471_ICHG_MAX,
				     RT9471_ICHG_STEP, regval);

	return 0;
}

static int __rt9471_get_aicr(struct rt9471_chip *chip, u32 *aicr)
{
	int ret;
	u8 regval;

	ret = rt9471_i2c_read_byte(chip, RT9471_REG_IBUS, &regval);
	if (ret < 0)
		return ret;

	regval = (regval & RT9471_AICR_MASK) >> RT9471_AICR_SHIFT;
	*aicr = rt9471_closest_value(RT9471_AICR_MIN, RT9471_AICR_MAX,
				     RT9471_AICR_STEP, regval);
	if (*aicr > RT9471_AICR_MIN && *aicr < RT9471_AICR_MAX)
		*aicr -= RT9471_AICR_STEP;

	return 0;
}

static int __rt9471_get_cv(struct rt9471_chip *chip, u32 *cv)
{
	int ret;
	u8 regval;

	ret = rt9471_i2c_read_byte(chip, RT9471_REG_VCHG, &regval);
	if (ret < 0)
		return ret;

	regval = (regval & RT9471_CV_MASK) >> RT9471_CV_SHIFT;
	*cv = rt9471_closest_value(RT9471_CV_MIN, RT9471_CV_MAX, RT9471_CV_STEP,
				   regval);

	return 0;
}

static int __rt9471_get_ieoc(struct rt9471_chip *chip, u32 *ieoc)
{
	int ret;
	u8 regval;

	ret = rt9471_i2c_read_byte(chip, RT9471_REG_EOC, &regval);
	if (ret < 0)
		return ret;

	regval = (regval & RT9471_IEOC_MASK) >> RT9471_IEOC_SHIFT;
	*ieoc = rt9471_closest_value(RT9471_IEOC_MIN, RT9471_IEOC_MAX,
				     RT9471_IEOC_STEP, regval);

	return 0;
}

static int __rt9471_is_chg_enabled(struct rt9471_chip *chip, bool *en)
{
	return rt9471_i2c_test_bit(chip, RT9471_REG_FUNCTION,
				   RT9471_CHG_EN_SHIFT, en);
}

static int __rt9471_is_hz_enabled(struct rt9471_chip *chip, bool *en)
{
	return rt9471_i2c_test_bit(chip, RT9471_REG_FUNCTION,
				   RT9471_HZ_SHIFT, en);
}

static int __rt9471_is_shipmode(struct rt9471_chip *chip, bool *en)
{
	return rt9471_i2c_test_bit(chip, RT9471_REG_FUNCTION,
				   RT9471_BATFETDIS_SHIFT, en);
}

static int __rt9471_enable_shipmode(struct rt9471_chip *chip, bool en)
{
	int ret;

	dev_info(chip->dev, "%s en = %d\n", __func__, en);

	if (en)
		ret = rt9471_set_bit(chip, RT9471_REG_FUNCTION, RT9471_BATFETDIS_MASK);
	else
		ret = rt9471_clr_bit(chip, RT9471_REG_FUNCTION, RT9471_BATFETDIS_MASK);

	return ret;
}

static int __rt9471_enable_safe_tmr(struct rt9471_chip *chip, bool en)
{
	int ret;

	dev_info(chip->dev, "%s en = %d\n", __func__, en);

	if (en)
		ret = rt9471_set_bit(chip, RT9471_REG_CHGTIMER, RT9471_SAFETMR_EN_MASK);
	else
		ret = rt9471_clr_bit(chip, RT9471_REG_CHGTIMER, RT9471_SAFETMR_EN_MASK);

	return ret;
}

static int __rt9471_enable_te(struct rt9471_chip *chip, bool en)
{
	int ret;

	dev_info(chip->dev, "%s en = %d\n", __func__, en);

	if (en)
		ret = rt9471_set_bit(chip, RT9471_REG_EOC, RT9471_TE_MASK);
	else
		ret = rt9471_clr_bit(chip, RT9471_REG_EOC, RT9471_TE_MASK);

	return ret;
}

static int __rt9471_enable_jeita(struct rt9471_chip *chip, bool en)
{
	int ret;

	dev_info(chip->dev, "%s en = %d\n", __func__, en);

	if (en)
		ret = rt9471_set_bit(chip, RT9471_REG_JEITA, RT9471_JEITA_EN_MASK);
	else
		ret = rt9471_clr_bit(chip, RT9471_REG_JEITA, RT9471_JEITA_EN_MASK);

	return ret;
}

static int __rt9471_disable_i2c_tout(struct rt9471_chip *chip, bool en)
{
	int ret;

	dev_info(chip->dev, "%s en = %d\n", __func__, en);

	if (en)
		ret = rt9471_set_bit(chip, RT9471_REG_TOP, RT9471_DISI2CTO_MASK);
	else
		ret = rt9471_clr_bit(chip, RT9471_REG_TOP, RT9471_DISI2CTO_MASK);

	return ret;
}

static int __rt9471_enable_qon_rst(struct rt9471_chip *chip, bool en)
{
	int ret;

	dev_info(chip->dev, "%s en = %d\n", __func__, en);

	if (en)
		ret = rt9471_set_bit(chip, RT9471_REG_TOP, RT9471_QONRST_MASK);
	else
		ret = rt9471_clr_bit(chip, RT9471_REG_TOP, RT9471_QONRST_MASK);

	return ret;
}

static int __rt9471_enable_autoaicr(struct rt9471_chip *chip, bool en)
{
	int ret;

	dev_info(chip->dev, "%s en = %d\n", __func__, en);

	if (en)
		ret = rt9471_set_bit(chip, RT9471_REG_IBUS, RT9471_AUTOAICR_MASK);
	else
		ret = rt9471_clr_bit(chip, RT9471_REG_IBUS, RT9471_AUTOAICR_MASK);

	return ret;
}

static int __rt9471_enable_hz(struct rt9471_chip *chip, bool en)
{
	int ret;

	dev_info(chip->dev, "%s en = %d\n", __func__, en);

	if (en)
		ret = rt9471_set_bit(chip, RT9471_REG_FUNCTION, RT9471_HZ_MASK);
	else
		ret = rt9471_clr_bit(chip, RT9471_REG_FUNCTION, RT9471_HZ_MASK);

	return ret;
}

static int __rt9471_enable_otg(struct rt9471_chip *chip, bool en)
{
	int ret;

	dev_info(chip->dev, "%s en = %d\n", __func__, en);

	if (en)
		ret = rt9471_set_bit(chip, RT9471_REG_FUNCTION, RT9471_OTG_EN_MASK);
	else
		ret = rt9471_clr_bit(chip, RT9471_REG_FUNCTION, RT9471_OTG_EN_MASK);

	return ret;
}

static int __rt9471_set_wdt(struct rt9471_chip *chip, u32 sec)
{
	u8 regval;

	/* 40s is the minimum, set to 40 except sec == 0 */
	if (sec <= 40 && sec > 0)
		sec = 40;
	regval = rt9471_closest_reg_via_tbl(rt9471_wdt, ARRAY_SIZE(rt9471_wdt),
					    sec);

	dev_info(chip->dev, "%s time = %d(0x%02X)\n", __func__, sec, regval);

	return rt9471_i2c_update_bits(chip, RT9471_REG_TOP,
				      regval << RT9471_WDT_SHIFT,
				      RT9471_WDT_MASK);
}

static int __rt9471_set_ichg(struct rt9471_chip *chip, u32 ichg)
{
	u8 regval;

	regval = rt9471_closest_reg(RT9471_ICHG_MIN, RT9471_ICHG_MAX,
				    RT9471_ICHG_STEP, ichg);

	dev_info(chip->dev, "%s ichg = %d(0x%02X)\n", __func__, ichg, regval);

	return rt9471_i2c_update_bits(chip, RT9471_REG_ICHG,
				      regval << RT9471_ICHG_SHIFT,
				      RT9471_ICHG_MASK);
}

static int __rt9471_set_aicr(struct rt9471_chip *chip, u32 aicr)
{
	u8 regval;

	regval = rt9471_closest_reg(RT9471_AICR_MIN, RT9471_AICR_MAX,
				    RT9471_AICR_STEP, aicr);
	/* 0 & 1 are both 50mA */
	if (aicr < RT9471_AICR_MAX)
		regval += 1;

	dev_info(chip->dev, "%s aicr = %d(0x%02X)\n", __func__, aicr, regval);

	return rt9471_i2c_update_bits(chip, RT9471_REG_IBUS,
				      regval << RT9471_AICR_SHIFT,
				      RT9471_AICR_MASK);
}

static int __rt9471_set_mivr(struct rt9471_chip *chip, u32 mivr)
{
	u8 regval;

	regval = rt9471_closest_reg(RT9471_MIVR_MIN, RT9471_MIVR_MAX,
				    RT9471_MIVR_STEP, mivr);

	dev_info(chip->dev, "%s mivr = %d(0x%02X)\n", __func__, mivr, regval);

	return rt9471_i2c_update_bits(chip, RT9471_REG_VBUS,
				      regval << RT9471_MIVR_SHIFT,
				      RT9471_MIVR_MASK);
}

static int __rt9471_set_vac_ovp(struct rt9471_chip *chip, u32 vac_ovp)
{
	u8 regval;

	/* 5.8V is the minimum */
	if (vac_ovp <= 5800)
		vac_ovp = 5800;
	/* 14V is the maximum */
	if (vac_ovp >= 14000)
		vac_ovp = 14000;
	regval = rt9471_closest_reg_via_tbl(rt9471_vac_ovp, ARRAY_SIZE(rt9471_vac_ovp),
					    vac_ovp);

	dev_info(chip->dev, "%s vac_ovp = %d(0x%02X)\n", __func__, vac_ovp, regval);

	return rt9471_i2c_update_bits(chip, RT9471_REG_VBUS,
				      regval << RT9471_VOVP_SHIFT,
				      RT9471_VOVP_MASK);
}

static int __rt9471_set_cv(struct rt9471_chip *chip, u32 cv)
{
	u8 regval;

	regval = rt9471_closest_reg(RT9471_CV_MIN, RT9471_CV_MAX,
				    RT9471_CV_STEP, cv);

	dev_info(chip->dev, "%s cv = %d(0x%02X)\n", __func__, cv, regval);

	return rt9471_i2c_update_bits(chip, RT9471_REG_VCHG,
				      regval << RT9471_CV_SHIFT,
				      RT9471_CV_MASK);
}

static int __rt9471_set_ieoc(struct rt9471_chip *chip, u32 ieoc)
{
	u8 regval;

	regval = rt9471_closest_reg(RT9471_IEOC_MIN, RT9471_IEOC_MAX,
				    RT9471_IEOC_STEP, ieoc);

	dev_info(chip->dev, "%s ieoc = %d(0x%02X)\n", __func__, ieoc, regval);

	return rt9471_i2c_update_bits(chip, RT9471_REG_EOC,
				      regval << RT9471_IEOC_SHIFT,
				      RT9471_IEOC_MASK);
}

static int __rt9471_set_safe_tmr(struct rt9471_chip *chip, u32 hr)
{
	u8 regval;

	regval = rt9471_closest_reg(RT9471_SAFETMR_MIN, RT9471_SAFETMR_MAX,
				    RT9471_SAFETMR_STEP, hr);

	dev_info(chip->dev, "%s time = %d(0x%02X)\n", __func__, hr, regval);

	return rt9471_i2c_update_bits(chip, RT9471_REG_CHGTIMER,
				      regval << RT9471_SAFETMR_SHIFT,
				      RT9471_SAFETMR_MASK);
}

static int __rt9471_set_mivrtrack(struct rt9471_chip *chip, u32 mivr_track)
{
	if (mivr_track >= RT9471_MIVRTRACK_MAX)
		mivr_track = RT9471_MIVRTRACK_VBAT_300MV;

	dev_info(chip->dev, "%s mivrtrack = %d\n", __func__, mivr_track);

	return rt9471_i2c_update_bits(chip, RT9471_REG_VBUS,
				      mivr_track << RT9471_MIVRTRACK_SHIFT,
				      RT9471_MIVRTRACK_MASK);
}

static int __rt9471_kick_wdt(struct rt9471_chip *chip)
{
	dev_info(chip->dev, "%s\n", __func__);
	return rt9471_set_bit(chip, RT9471_REG_TOP, RT9471_WDTCNTRST_MASK);
}

static void rt9471_buck_dwork_handler(struct work_struct *work)
{
	int ret, i;
	struct rt9471_chip *chip =
		container_of(work, struct rt9471_chip, buck_dwork.work);
	bool chg_rdy = false, chg_done = false;
	u8 reg_addrs[] = {RT9471_REG_BUCK_HDEN4, RT9471_REG_BUCK_HDEN1,
			  RT9471_REG_BUCK_HDEN2, RT9471_REG_BUCK_HDEN4,
			  RT9471_REG_BUCK_HDEN2, RT9471_REG_BUCK_HDEN1};
	u8 reg_vals[] = {0x77, 0x2F, 0xA2, 0x71, 0x22, 0x2D};

	dev_info(chip->dev, "%s chip_rev = %d\n", __func__, chip->chip_rev);
	if (chip->chip_rev > 4)
		return;
	ret = rt9471_i2c_test_bit(chip, RT9471_REG_STAT0,
				  RT9471_ST_CHGRDY_SHIFT, &chg_rdy);
	if (ret < 0)
		return;
	dev_info(chip->dev, "%s chg_rdy = %d\n", __func__, chg_rdy);
	if (!chg_rdy)
		return;
	ret = rt9471_i2c_test_bit(chip, RT9471_REG_STAT0,
				  RT9471_ST_CHGDONE_SHIFT, &chg_done);
	if (ret < 0)
		return;
	dev_info(chip->dev, "%s chg_done = %d, chg_done_once = %d\n",
			    __func__, chg_done, chip->chg_done_once);

	ret = rt9471_enable_hidden_mode(chip, true);
	if (ret < 0)
		return;

	for (i = 0; i < ARRAY_SIZE(reg_addrs); i++) {
		ret = rt9471_i2c_write_byte(chip, reg_addrs[i], reg_vals[i]);
		if (ret < 0)
			dev_notice(chip->dev,
				   "%s reg0x%02X = 0x%02X fail(%d)\n",
				   __func__, reg_addrs[i], reg_vals[i], ret);
		if (i == 1)
			udelay(1000);
	}

	rt9471_enable_hidden_mode(chip, false);

	if (chg_done && !chip->chg_done_once) {
		chip->chg_done_once = true;
		mod_delayed_work(system_wq, &chip->buck_dwork,
				 msecs_to_jiffies(100));
	}
}

static int rt9471_bc12_preprocess(struct rt9471_chip *chip)
{
	if (chip->dev_id != RT9470D_DEVID && chip->dev_id != RT9471D_DEVID)
		return 0;

	if (atomic_read(&chip->vbus_gd)) {
		rt9471_enable_bc12(chip, false);
		rt9471_enable_bc12(chip, true);
	}

	return 0;
}

static int rt9471_bc12_postprocess(struct rt9471_chip *chip)
{
	int ret = 0;
	bool attach = false, inform_psy = true;
	u8 port = RT9471_PORTSTAT_NOINFO;
	enum power_supply_type type = POWER_SUPPLY_TYPE_UNKNOWN;

	if (chip->dev_id != RT9470D_DEVID && chip->dev_id != RT9471D_DEVID)
		return 0;

	attach = atomic_read(&chip->vbus_gd);
	if (chip->attach == attach) {
		dev_info(chip->dev, "%s attach(%d) is the same\n",
				    __func__, attach);
		inform_psy = !attach;
		goto same_out;
	}
	chip->attach = attach;
	dev_info(chip->dev, "%s attach = %d\n", __func__, attach);

	if (!attach)
		goto out;

	ret = rt9471_i2c_read_byte(chip, RT9471_REG_STATUS, &port);
	if (ret < 0)
		port = RT9471_PORTSTAT_NOINFO;
	else
		port = (port & RT9471_PORTSTAT_MASK) >> RT9471_PORTSTAT_SHIFT;

	switch (port) {
	case RT9471_PORTSTAT_NOINFO:
		type = POWER_SUPPLY_TYPE_UNKNOWN;
		break;
	case RT9471_PORTSTAT_SDP:
		type = POWER_SUPPLY_TYPE_USB;
		break;
	case RT9471_PORTSTAT_CDP:
		type = POWER_SUPPLY_TYPE_USB_CDP;
		break;
	case RT9471_PORTSTAT_APPLE_10W:
	case RT9471_PORTSTAT_SAMSUNG_10W:
	case RT9471_PORTSTAT_APPLE_5W:
	case RT9471_PORTSTAT_APPLE_12W:
	case RT9471_PORTSTAT_DCP:
		type = POWER_SUPPLY_TYPE_USB_DCP;
		break;
	case RT9471_PORTSTAT_NSDP:
	default:
		type = POWER_SUPPLY_TYPE_USB;
		break;
	}
out:
	chip->port = port;
	chip->psy_desc.type = type;
same_out:
	if (type != POWER_SUPPLY_TYPE_USB_DCP)
		rt9471_enable_bc12(chip, false);
	if (inform_psy)
		power_supply_changed(chip->psy);

	return 0;
}

static int rt9471_detach_irq_handler(struct rt9471_chip *chip)
{
	dev_info(chip->dev, "%s\n", __func__);
	mutex_lock(&chip->bc12_lock);
	atomic_set(&chip->vbus_gd, rt9471_is_vbusgd(chip));
	rt9471_bc12_postprocess(chip);
	mutex_unlock(&chip->bc12_lock);
	return 0;
}

static int rt9471_rechg_irq_handler(struct rt9471_chip *chip)
{
	dev_info(chip->dev, "%s\n", __func__);
	return 0;
}

static void rt9471_bc12_done_handler(struct rt9471_chip *chip)
{
	int ret;
	u8 regval;
	bool bc12_done = false, chg_rdy = false;

	if (chip->dev_id != RT9470D_DEVID && chip->dev_id != RT9471D_DEVID)
		return;

	dev_info(chip->dev, "%s\n", __func__);

	ret = rt9471_i2c_read_byte(chip, RT9471_REG_STAT0, &regval);
	if (ret < 0)
		dev_notice(chip->dev, "%s check stat fail(%d)\n",
				      __func__, ret);
	bc12_done = (regval & RT9471_ST_BC12_DONE_MASK ? true : false);
	chg_rdy = (regval & RT9471_ST_CHGRDY_MASK ? true : false);
	dev_info(chip->dev, "%s bc12_done = %d, chg_rdy = %d\n",
			    __func__, bc12_done, chg_rdy);
	if (bc12_done) {
		if (chip->chip_rev <= 3 && !chg_rdy) {
			/* Workaround waiting for chg_rdy */
			dev_info(chip->dev, "%s wait chg_rdy\n", __func__);
			return;
		}
		mutex_lock(&chip->bc12_lock);
		ret = rt9471_bc12_postprocess(chip);
		dev_info(chip->dev, "%s %d %s\n", __func__, chip->port,
				    rt9471_port_name[chip->port]);
		mutex_unlock(&chip->bc12_lock);
	}
}

static int rt9471_bc12_done_irq_handler(struct rt9471_chip *chip)
{
	dev_info(chip->dev, "%s\n", __func__);
	rt9471_bc12_done_handler(chip);
	return 0;
}

static int rt9471_chg_done_irq_handler(struct rt9471_chip *chip)
{
	dev_info(chip->dev, "%s\n", __func__);
	if (chip->chip_rev > 4)
		return 0;
	cancel_delayed_work_sync(&chip->buck_dwork);
	chip->chg_done_once = false;
	mod_delayed_work(system_wq, &chip->buck_dwork, msecs_to_jiffies(100));
	return 0;
}

static int rt9471_bg_chg_irq_handler(struct rt9471_chip *chip)
{
	dev_info(chip->dev, "%s\n", __func__);
	return 0;
}

static int rt9471_ieoc_irq_handler(struct rt9471_chip *chip)
{
	dev_info(chip->dev, "%s\n", __func__);
	return 0;
}

static int rt9471_chg_rdy_irq_handler(struct rt9471_chip *chip)
{
	dev_info(chip->dev, "%s\n", __func__);
	if (chip->chip_rev > 4)
		return 0;
	if (chip->chip_rev <= 3)
		rt9471_bc12_done_handler(chip);
	mod_delayed_work(system_wq, &chip->buck_dwork, msecs_to_jiffies(100));
	return 0;
}

static int rt9471_vbus_gd_irq_handler(struct rt9471_chip *chip)
{
	dev_info(chip->dev, "%s\n", __func__);
	mutex_lock(&chip->bc12_lock);
	atomic_set(&chip->vbus_gd, rt9471_is_vbusgd(chip));
	rt9471_bc12_preprocess(chip);
	mutex_unlock(&chip->bc12_lock);
	return 0;
}

static int rt9471_chg_batov_irq_handler(struct rt9471_chip *chip)
{
	dev_info(chip->dev, "%s\n", __func__);
	return 0;
}

static int rt9471_chg_sysov_irq_handler(struct rt9471_chip *chip)
{
	dev_info(chip->dev, "%s\n", __func__);
	return 0;
}

static int rt9471_chg_tout_irq_handler(struct rt9471_chip *chip)
{
	dev_info(chip->dev, "%s\n", __func__);
	return 0;
}

static int rt9471_chg_busuv_irq_handler(struct rt9471_chip *chip)
{
	dev_info(chip->dev, "%s\n", __func__);
	return 0;
}

static int rt9471_chg_threg_irq_handler(struct rt9471_chip *chip)
{
	dev_info(chip->dev, "%s\n", __func__);
	return 0;
}

static int rt9471_chg_aicr_irq_handler(struct rt9471_chip *chip)
{
	dev_info(chip->dev, "%s\n", __func__);
	return 0;
}

static int rt9471_chg_mivr_irq_handler(struct rt9471_chip *chip)
{
	int ret = 0;
	bool mivr = false;

	ret = rt9471_i2c_test_bit(chip, RT9471_REG_STAT1, RT9471_ST_MIVR_SHIFT,
				  &mivr);
	if (ret < 0) {
		dev_notice(chip->dev, "%s check stat fail(%d)\n",
				      __func__, ret);
		return ret;
	}
	dev_info(chip->dev, "%s mivr = %d\n", __func__, mivr);

	return 0;
}

static int rt9471_sys_short_irq_handler(struct rt9471_chip *chip)
{
	dev_info(chip->dev, "%s\n", __func__);
	return 0;
}

static int rt9471_sys_min_irq_handler(struct rt9471_chip *chip)
{
	dev_info(chip->dev, "%s\n", __func__);
	return 0;
}

static int rt9471_jeita_cold_irq_handler(struct rt9471_chip *chip)
{
	dev_info(chip->dev, "%s\n", __func__);
	return 0;
}

static int rt9471_jeita_cool_irq_handler(struct rt9471_chip *chip)
{
	dev_info(chip->dev, "%s\n", __func__);
	return 0;
}

static int rt9471_jeita_warm_irq_handler(struct rt9471_chip *chip)
{
	dev_info(chip->dev, "%s\n", __func__);
	return 0;
}

static int rt9471_jeita_hot_irq_handler(struct rt9471_chip *chip)
{
	dev_info(chip->dev, "%s\n", __func__);
	return 0;
}

static int rt9471_otg_fault_irq_handler(struct rt9471_chip *chip)
{
	dev_info(chip->dev, "%s\n", __func__);
	return 0;
}

static int rt9471_otg_lbp_irq_handler(struct rt9471_chip *chip)
{
	dev_info(chip->dev, "%s\n", __func__);
	return 0;
}

static int rt9471_otg_cc_irq_handler(struct rt9471_chip *chip)
{
	dev_info(chip->dev, "%s\n", __func__);
	return 0;
}

static int rt9471_wdt_irq_handler(struct rt9471_chip *chip)
{
	dev_info(chip->dev, "%s\n", __func__);
	return __rt9471_kick_wdt(chip);
}

static int rt9471_vac_ov_irq_handler(struct rt9471_chip *chip)
{
	int ret = 0;
	bool vacov = false;

	ret = rt9471_i2c_test_bit(chip, RT9471_REG_STAT3, RT9471_ST_VACOV_SHIFT,
				  &vacov);
	if (ret < 0) {
		dev_notice(chip->dev, "%s check stat fail(%d)\n",
				      __func__, ret);
		return ret;
	}
	dev_info(chip->dev, "%s vacov = %d\n", __func__, vacov);

	return 0;
}

static int rt9471_otp_irq_handler(struct rt9471_chip *chip)
{
	dev_info(chip->dev, "%s\n", __func__);
	return 0;
}

struct irq_mapping_tbl {
	const char *name;
	int (*hdlr)(struct rt9471_chip *chip);
	int num;
};

#define RT9471_IRQ_MAPPING(_name, _num) \
	{.name = #_name, .hdlr = rt9471_##_name##_irq_handler, .num = _num}

static const struct irq_mapping_tbl rt9471_irq_mapping_tbl[] = {
	RT9471_IRQ_MAPPING(wdt, 29),
	RT9471_IRQ_MAPPING(vbus_gd, 7),
	RT9471_IRQ_MAPPING(chg_rdy, 6),
	RT9471_IRQ_MAPPING(bc12_done, 0),
	RT9471_IRQ_MAPPING(detach, 1),
	RT9471_IRQ_MAPPING(rechg, 2),
	RT9471_IRQ_MAPPING(chg_done, 3),
	RT9471_IRQ_MAPPING(bg_chg, 4),
	RT9471_IRQ_MAPPING(ieoc, 5),
	RT9471_IRQ_MAPPING(chg_batov, 9),
	RT9471_IRQ_MAPPING(chg_sysov, 10),
	RT9471_IRQ_MAPPING(chg_tout, 11),
	RT9471_IRQ_MAPPING(chg_busuv, 12),
	RT9471_IRQ_MAPPING(chg_threg, 13),
	RT9471_IRQ_MAPPING(chg_aicr, 14),
	RT9471_IRQ_MAPPING(chg_mivr, 15),
	RT9471_IRQ_MAPPING(sys_short, 16),
	RT9471_IRQ_MAPPING(sys_min, 17),
	RT9471_IRQ_MAPPING(jeita_cold, 20),
	RT9471_IRQ_MAPPING(jeita_cool, 21),
	RT9471_IRQ_MAPPING(jeita_warm, 22),
	RT9471_IRQ_MAPPING(jeita_hot, 23),
	RT9471_IRQ_MAPPING(otg_fault, 24),
	RT9471_IRQ_MAPPING(otg_lbp, 25),
	RT9471_IRQ_MAPPING(otg_cc, 26),
	RT9471_IRQ_MAPPING(vac_ov, 30),
	RT9471_IRQ_MAPPING(otp, 31),
};

static irqreturn_t rt9471_irq_handler(int irq, void *data)
{
	int ret, i, irqnum = 0, irqbit = 0;
	u8 evt[RT9471_IRQIDX_MAX] = {0};
	u8 mask[RT9471_IRQIDX_MAX] = {0};
	struct rt9471_chip *chip = (struct rt9471_chip *)data;

	dev_info(chip->dev, "%s\n", __func__);

	pm_stay_awake(chip->dev);

	ret = rt9471_i2c_block_read(chip, RT9471_REG_IRQ0, RT9471_IRQIDX_MAX,
				    evt);
	if (ret < 0) {
		dev_notice(chip->dev, "%s read evt fail(%d)\n", __func__, ret);
		goto out;
	}

	ret = rt9471_i2c_block_read(chip, RT9471_REG_MASK0, RT9471_IRQIDX_MAX,
				    mask);
	if (ret < 0) {
		dev_notice(chip->dev, "%s read mask fail(%d)\n", __func__, ret);
		goto out;
	}

	for (i = 0; i < RT9471_IRQIDX_MAX; i++)
		evt[i] &= ~mask[i];
	for (i = 0; i < ARRAY_SIZE(rt9471_irq_mapping_tbl); i++) {
		irqnum = rt9471_irq_mapping_tbl[i].num / 8;
		if (irqnum >= RT9471_IRQIDX_MAX)
			continue;
		irqbit = rt9471_irq_mapping_tbl[i].num % 8;
		if (evt[irqnum] & (1 << irqbit))
			rt9471_irq_mapping_tbl[i].hdlr(chip);
	}
out:
	pm_relax(chip->dev);
	return IRQ_HANDLED;
}

static int rt9471_register_irq(struct rt9471_chip *chip)
{
	int ret, len;
	char *name = NULL;

	dev_info(chip->dev, "%s\n", __func__);

	len = strlen(chip->desc->chg_name);
	name = devm_kzalloc(chip->dev, len + 10, GFP_KERNEL);
	if (!name)
		return -ENOMEM;
	snprintf(name,  len + 10, "%s-irq-gpio", chip->desc->chg_name);
	ret = devm_gpio_request_one(chip->dev, chip->intr_gpio, GPIOF_IN, name);
	if (ret < 0) {
		dev_notice(chip->dev, "%s gpio request fail(%d)\n",
				      __func__, ret);
		return ret;
	}
	chip->irq = gpio_to_irq(chip->intr_gpio);
	if (chip->irq < 0) {
		dev_notice(chip->dev, "%s gpio2irq fail(%d)\n", __func__,
				      chip->irq);
		return chip->irq;
	}
	dev_info(chip->dev, "%s irq = %d\n", __func__, chip->irq);

	/* Request threaded IRQ */
	len = strlen(chip->desc->chg_name);
	name = devm_kzalloc(chip->dev, len + 5, GFP_KERNEL);
	if (!name)
		return -ENOMEM;
	snprintf(name, len + 5, "%s-irq", chip->desc->chg_name);
	ret = devm_request_threaded_irq(chip->dev, chip->irq, NULL,
					rt9471_irq_handler,
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					name, chip);
	if (ret < 0) {
		dev_notice(chip->dev, "%s request threaded irq fail(%d)\n",
				      __func__, ret);
		return ret;
	}
	device_init_wakeup(chip->dev, true);
	ret = enable_irq_wake(chip->irq);
	if (ret)
		dev_err(chip->dev, "Error enabling irq wake ret:%d\n", ret);

	return 0;
}

static int rt9471_init_irq(struct rt9471_chip *chip)
{
	dev_info(chip->dev, "%s\n", __func__);
	return rt9471_i2c_block_write(chip, RT9471_REG_MASK0,
				      ARRAY_SIZE(chip->irq_mask),
				      chip->irq_mask);
}

static inline int rt9471_get_irq_number(struct rt9471_chip *chip,
					const char *name)
{
	int i;

	if (!name) {
		dev_notice(chip->dev, "%s null name\n", __func__);
		return -EINVAL;
	}

	for (i = 0; i < ARRAY_SIZE(rt9471_irq_mapping_tbl); i++) {
		if (!strcmp(name, rt9471_irq_mapping_tbl[i].name))
			return rt9471_irq_mapping_tbl[i].num;
	}

	return -EINVAL;
}

static inline const char *rt9471_get_irq_name(int irqnum)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(rt9471_irq_mapping_tbl); i++) {
		if (rt9471_irq_mapping_tbl[i].num == irqnum)
			return rt9471_irq_mapping_tbl[i].name;
	}

	return "not found";
}

static inline void rt9471_irq_unmask(struct rt9471_chip *chip, int irqnum)
{
	dev_info(chip->dev, "%s irq(%d, %s)\n", __func__, irqnum,
		 rt9471_get_irq_name(irqnum));
	chip->irq_mask[irqnum / 8] &= ~(1 << (irqnum % 8));
}

static int rt9471_parse_dt(struct rt9471_chip *chip)
{
	int ret, len, irqcnt = 0, irqnum;
	struct device_node *parent_np = chip->dev->of_node, *np = NULL;
	struct rt9471_desc *desc = NULL;
	const char *name = NULL;
	char *ceb_name = NULL;
	unsigned long init_flags = GPIOF_DIR_OUT;

	dev_info(chip->dev, "%s\n", __func__);

	chip->desc = &rt9471_default_desc;

	if (!parent_np) {
		dev_notice(chip->dev, "%s no device node\n", __func__);
		return -EINVAL;
	}
	np = of_get_child_by_name(parent_np, "rt9471");
	if (!np) {
		dev_notice(chip->dev, "%s no rt9471 device node\n", __func__);
		return -EINVAL;
	}

	desc = devm_kzalloc(chip->dev, sizeof(*desc), GFP_KERNEL);
	if (!desc)
		return -ENOMEM;
	memcpy(desc, &rt9471_default_desc, sizeof(*desc));

	if (of_property_read_string(np, "charger_name", &desc->chg_name) < 0)
		dev_notice(chip->dev, "%s no charger name\n", __func__);
	dev_info(chip->dev, "%s name %s\n", __func__, desc->chg_name);

	ret = of_get_named_gpio(parent_np, "rt,intr_gpio", 0);
	if (ret < 0)
		return ret;
	chip->intr_gpio = ret;
	ret = of_get_named_gpio(parent_np, "rt,ceb_gpio", 0);
	if (ret < 0)
		return ret;
	chip->ceb_gpio = ret;
	dev_info(chip->dev, "%s intr_gpio %u\n", __func__, chip->intr_gpio);

	/* ceb gpio */
	len = strlen(desc->chg_name);
	ceb_name = devm_kzalloc(chip->dev, len + 10, GFP_KERNEL);
	if (!ceb_name)
		return -ENOMEM;
	snprintf(ceb_name,  len + 10, "%s-ceb-gpio", desc->chg_name);
	of_property_read_u32(parent_np, "google,rt-en-value",
			     &chip->rten_gpio_default);
	if (chip->rten_gpio_default)
		init_flags = GPIOF_OUT_INIT_HIGH;
	ret = devm_gpio_request_one(chip->dev, chip->ceb_gpio, init_flags,
				    ceb_name);
	if (ret < 0) {
		dev_notice(chip->dev, "%s gpio request fail(%d)\n",
				      __func__, ret);
		return ret;
	}

	/* Charger parameter */
	if (of_property_read_u32(np, "ichg", &desc->ichg) < 0)
		dev_info(chip->dev, "%s no ichg\n", __func__);

	if (of_property_read_u32(np, "aicr", &desc->aicr) < 0)
		dev_info(chip->dev, "%s no aicr\n", __func__);

	if (of_property_read_u32(np, "mivr", &desc->mivr) < 0)
		dev_info(chip->dev, "%s no mivr\n", __func__);

	if (of_property_read_u32(np, "vac_ovp", &desc->vac_ovp) < 0)
		dev_info(chip->dev, "%s no vac_ovp\n", __func__);

	if (of_property_read_u32(np, "cv", &desc->cv) < 0)
		dev_info(chip->dev, "%s no cv\n", __func__);

	if (of_property_read_u32(np, "ieoc", &desc->ieoc) < 0)
		dev_info(chip->dev, "%s no ieoc\n", __func__);

	if (of_property_read_u32(np, "safe-tmr", &desc->safe_tmr) < 0)
		dev_info(chip->dev, "%s no safety timer\n", __func__);

	if (of_property_read_u32(np, "wdt", &desc->wdt) < 0)
		dev_info(chip->dev, "%s no wdt\n", __func__);

	if (of_property_read_u32(np, "mivr-track", &desc->mivr_track) < 0)
		dev_info(chip->dev, "%s no mivr track\n", __func__);
	if (desc->mivr_track >= RT9471_MIVRTRACK_MAX)
		desc->mivr_track = RT9471_MIVRTRACK_VBAT_300MV;

	desc->en_safe_tmr = of_property_read_bool(np, "en-safe-tmr");
	desc->en_te = of_property_read_bool(np, "en-te");
	desc->en_jeita = of_property_read_bool(np, "en-jeita");
	desc->dis_i2c_tout = of_property_read_bool(np, "dis-i2c-tout");
	desc->en_qon_rst = of_property_read_bool(np, "en-qon-rst");
	desc->auto_aicr = of_property_read_bool(np, "auto-aicr");

	chip->desc = desc;

	memcpy(chip->irq_mask, rt9471_irq_maskall, RT9471_IRQIDX_MAX);
	while (true) {
		ret = of_property_read_string_index(np, "interrupt-names",
						    irqcnt, &name);
		if (ret < 0)
			break;
		irqcnt++;
		irqnum = rt9471_get_irq_number(chip, name);
		if (irqnum >= 0)
			rt9471_irq_unmask(chip, irqnum);
	}

	return 0;
}

static int rt9471_sw_workaround(struct rt9471_chip *chip)
{
	int ret;
	u8 regval;

	dev_info(chip->dev, "%s\n", __func__);

	ret = rt9471_enable_hidden_mode(chip, true);
	if (ret < 0)
		return ret;

	ret = rt9471_i2c_read_byte(chip, RT9471_REG_HIDDEN_0, &regval);
	if (ret < 0) {
		dev_notice(chip->dev, "%s read HIDDEN_0 fail(%d)\n",
				      __func__, ret);
		goto out;
	}
	chip->chip_rev = (regval & RT9471_CHIP_REV_MASK) >>
			 RT9471_CHIP_REV_SHIFT;
	dev_info(chip->dev, "%s chip_rev = %d\n", __func__, chip->chip_rev);

	/* OTG load transient improvement */
	if (chip->chip_rev <= 3)
		ret = rt9471_i2c_update_bits(chip, RT9471_REG_OTG_HDEN2, 0x10,
					     RT9471_REG_OTG_RES_COMP_MASK);

out:
	rt9471_enable_hidden_mode(chip, false);
	return ret;
}

static int rt9471_init_setting(struct rt9471_chip *chip)
{
	int ret;
	struct rt9471_desc *desc = chip->desc;
	u8 evt[RT9471_IRQIDX_MAX] = {0};

	dev_info(chip->dev, "%s\n", __func__);

	/* Disable WDT during IRQ masked period */
	ret = __rt9471_set_wdt(chip, 0);
	if (ret < 0)
		dev_notice(chip->dev, "%s set wdt fail(%d)\n", __func__, ret);

	/* Mask all IRQs */
	ret = rt9471_i2c_block_write(chip, RT9471_REG_MASK0,
				     ARRAY_SIZE(rt9471_irq_maskall),
				     rt9471_irq_maskall);
	if (ret < 0)
		dev_notice(chip->dev, "%s mask irq fail(%d)\n", __func__, ret);

	/* Clear all IRQs */
	ret = rt9471_i2c_block_read(chip, RT9471_REG_IRQ0, RT9471_IRQIDX_MAX,
				    evt);
	if (ret < 0)
		dev_notice(chip->dev, "%s clear irq fail(%d)\n", __func__, ret);

	ret = __rt9471_set_ichg(chip, desc->ichg);
	if (ret < 0)
		dev_notice(chip->dev, "%s set ichg fail(%d)\n", __func__, ret);

	ret = __rt9471_set_aicr(chip, desc->aicr);
	if (ret < 0)
		dev_notice(chip->dev, "%s set aicr fail(%d)\n", __func__, ret);

	ret = __rt9471_set_mivr(chip, desc->mivr);
	if (ret < 0)
		dev_notice(chip->dev, "%s set mivr fail(%d)\n", __func__, ret);

	ret = __rt9471_set_vac_ovp(chip, desc->vac_ovp);
	if (ret < 0)
		dev_notice(chip->dev, "%s set vac_ovp fail(%d)\n", __func__, ret);

	ret = __rt9471_set_cv(chip, desc->cv);
	if (ret < 0)
		dev_notice(chip->dev, "%s set cv fail(%d)\n", __func__, ret);

	ret = __rt9471_set_ieoc(chip, desc->ieoc);
	if (ret < 0)
		dev_notice(chip->dev, "%s set ieoc fail(%d)\n", __func__, ret);

	ret = __rt9471_set_safe_tmr(chip, desc->safe_tmr);
	if (ret < 0)
		dev_notice(chip->dev, "%s set safe tmr fail(%d)\n",
				      __func__, ret);

	ret = __rt9471_set_mivrtrack(chip, desc->mivr_track);
	if (ret < 0)
		dev_notice(chip->dev, "%s set mivrtrack fail(%d)\n",
				      __func__, ret);

	ret = __rt9471_enable_safe_tmr(chip, desc->en_safe_tmr);
	if (ret < 0)
		dev_notice(chip->dev, "%s en safe tmr fail(%d)\n",
				      __func__, ret);

	ret = __rt9471_enable_te(chip, desc->en_te);
	if (ret < 0)
		dev_notice(chip->dev, "%s en te fail(%d)\n", __func__, ret);

	ret = __rt9471_enable_jeita(chip, desc->en_jeita);
	if (ret < 0)
		dev_notice(chip->dev, "%s en jeita fail(%d)\n", __func__, ret);

	ret = __rt9471_disable_i2c_tout(chip, desc->dis_i2c_tout);
	if (ret < 0)
		dev_notice(chip->dev, "%s dis i2c tout fail(%d)\n",
				      __func__, ret);

	ret = __rt9471_enable_qon_rst(chip, desc->en_qon_rst);
	if (ret < 0)
		dev_notice(chip->dev, "%s en qon rst fail(%d)\n",
				      __func__, ret);

	ret = __rt9471_enable_autoaicr(chip, desc->auto_aicr);
	if (ret < 0)
		dev_notice(chip->dev, "%s en autoaicr fail(%d)\n",
				      __func__, ret);

	rt9471_enable_bc12(chip, false);

	ret = rt9471_sw_workaround(chip);
	if (ret < 0)
		dev_notice(chip->dev, "%s set sw workaround fail(%d)\n",
				      __func__, ret);

	return 0;
}

static int rt9471_reset_register(struct rt9471_chip *chip)
{
	int ret;

	dev_info(chip->dev, "%s\n", __func__);

	ret = rt9471_set_bit(chip, RT9471_REG_INFO, RT9471_REGRST_MASK);
	if (ret < 0)
		return ret;

	regcache_mark_dirty(chip->rm_dev);

	return 0;
}

static bool rt9471_check_devinfo(struct rt9471_chip *chip)
{
	int ret;

	ret = i2c_smbus_read_byte_data(chip->client, RT9471_REG_INFO);
	if (ret < 0) {
		dev_notice(chip->dev, "%s get devinfo fail(%d)\n",
				      __func__, ret);
		return false;
	}
	chip->dev_id = (ret & RT9471_DEVID_MASK) >> RT9471_DEVID_SHIFT;
	if (chip->dev_id != RT9470_DEVID && chip->dev_id != RT9470D_DEVID &&
		chip->dev_id != RT9471_DEVID && chip->dev_id != RT9471D_DEVID) {
		dev_notice(chip->dev, "%s incorrect devid 0x%02X\n",
				      __func__, chip->dev_id);
		return false;
	}
	chip->dev_rev = (ret & RT9471_DEVREV_MASK) >> RT9471_DEVREV_SHIFT;
	dev_info(chip->dev, "%s id = 0x%02X, rev = 0x%02X\n", __func__,
		 chip->dev_id, chip->dev_rev);

	return true;
}

static int __rt9471_dump_registers(struct rt9471_chip *chip)
{
	int i, ret;
	u32 ichg = 0, aicr = 0, mivr = 0, ieoc = 0, cv = 0;
	bool chg_en = 0;
	enum rt9471_ic_stat ic_stat = RT9471_ICSTAT_SLEEP;
	u8 stats[RT9471_STATIDX_MAX] = {0}, regval = 0;

	ret = __rt9471_kick_wdt(chip);

	ret = __rt9471_get_ichg(chip, &ichg);
	ret = __rt9471_get_aicr(chip, &aicr);
	ret = __rt9471_get_mivr(chip, &mivr);
	ret = __rt9471_get_ieoc(chip, &ieoc);
	ret = __rt9471_get_cv(chip, &cv);
	ret = __rt9471_is_chg_enabled(chip, &chg_en);
	ret = __rt9471_get_ic_stat(chip, &ic_stat);
	ret = rt9471_i2c_block_read(chip, RT9471_REG_STAT0, RT9471_STATIDX_MAX,
				    stats);

	if (ic_stat == RT9471_ICSTAT_CHGFAULT) {
		for (i = 0; i < ARRAY_SIZE(rt9471_reg_addr); i++) {
			ret = rt9471_i2c_read_byte(chip, rt9471_reg_addr[i],
						   &regval);
			if (ret < 0)
				continue;
			dev_info(chip->dev, "%s reg0x%02X = 0x%02X\n", __func__,
					    rt9471_reg_addr[i], regval);
		}
	}

	dev_info(chip->dev,
		 "%s ICHG = %dmA, AICR = %dmA, MIVR = %dmV\n",
		 __func__, ichg / 1000, aicr / 1000, mivr / 1000);

	dev_info(chip->dev, "%s IEOC = %dmA, CV = %dmV\n",
		 __func__, ieoc / 1000, cv / 1000);

	dev_info(chip->dev, "%s CHG_EN = %d, IC_STAT = %s\n",
		 __func__, chg_en, rt9471_ic_stat_name[ic_stat]);

	dev_info(chip->dev,
		 "%s STAT0 = 0x%02X, STAT1 = 0x%02X\n", __func__,
		 stats[RT9471_STATIDX_STAT0], stats[RT9471_STATIDX_STAT1]);

	dev_info(chip->dev,
		 "%s STAT2 = 0x%02X, STAT3 = 0x%02X\n", __func__,
		 stats[RT9471_STATIDX_STAT2], stats[RT9471_STATIDX_STAT3]);

	return 0;
}

static void rt9471_init_work_handler(struct work_struct *work)
{
	struct rt9471_chip *chip = container_of(work, struct rt9471_chip,
						init_work);

	mutex_lock(&chip->bc12_lock);
	atomic_set(&chip->vbus_gd, rt9471_is_vbusgd(chip));
	rt9471_bc12_preprocess(chip);
	mutex_unlock(&chip->bc12_lock);
	__rt9471_dump_registers(chip);
}

/* ------------------------------------------------------------------------ */

#if IS_ENABLED(CONFIG_GPIOLIB)
static int rt9471_gpio_get_direction(struct gpio_chip *chip, unsigned int offset)
{
	return GPIOF_DIR_OUT;
}

static int rt9471_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	return 0;
}

static void rt9471_gpio_set(struct gpio_chip *chip, unsigned int offset, int value)
{
	struct rt9471_chip *data = gpiochip_get_data(chip);
	int ret;

	switch (offset) {
	case RT9471_GPIO_USB_OTG_EN:
		ret = __rt9471_enable_otg(data, value);
		break;

	default:
		ret = -EINVAL;
		break;
	}

	pr_debug("%s: GPIO offset=%d value=%d ret:%d\n", __func__, offset, value, ret);

	if (ret < 0)
		dev_err(data->dev, "GPIO%d: value=%d ret:%d\n", offset, value, ret);
}

static void rt9471_gpio_init(struct rt9471_chip *chip)
{
	chip->gpio.owner = THIS_MODULE;
	chip->gpio.label = "rt9471_gpio";
	chip->gpio.get_direction = rt9471_gpio_get_direction;
	chip->gpio.get = rt9471_gpio_get;
	chip->gpio.set = rt9471_gpio_set;
	chip->gpio.base = -1;
	chip->gpio.ngpio = RT9471_NUM_GPIOS;
	chip->gpio.can_sleep = true;
}
#endif

/* ------------------------------------------------------------------------ */

static ssize_t registers_dump_show(struct device *dev, struct device_attribute *attr,
				   char *buf)
{
	struct rt9471_chip *chip = dev_get_drvdata(dev);
	u8 tmp[RT9471_REG_BUCK_HDEN5 - RT9471_REG_OTGCFG + 1];
	int ret = 0, i;
	int len = 0;

	mutex_lock(&chip->io_lock);
	ret = regmap_bulk_read(chip->rm_dev, RT9471_REG_OTGCFG, &tmp, sizeof(tmp));
	mutex_unlock(&chip->io_lock);
	if (ret < 0)
		return ret;

	for (i = 0; i < sizeof(tmp); i++)
		len += scnprintf(&buf[len], PAGE_SIZE - len, "%02x: %02x\n", i, tmp[i]);

	return len;
}

static DEVICE_ATTR_RO(registers_dump);

static int rt9471_create_fs_entries(struct rt9471_chip *chip)
{
	device_create_file(chip->dev, &dev_attr_registers_dump);

	return 0;
}

/* ------------------------------------------------------------------------ */

static enum power_supply_property rt9471_psy_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CHARGE_EMPTY_DESIGN, /* Shipping mode */
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_EMPTY, /* HZ */
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_MANUFACTURER,
};

static int rt9471_psy_get_property(struct power_supply *psy,
				   enum power_supply_property prop,
				   union power_supply_propval *val)
{
	int ret = 0;
	bool en = false;
	enum rt9471_ic_stat ic_stat = RT9471_ICSTAT_SLEEP;
	u8 stat1 = 0, stat3 = 0;
	struct rt9471_chip *chip = power_supply_get_drvdata(psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_STATUS:
		ret = __rt9471_get_ic_stat(chip, &ic_stat);
		switch (ic_stat) {
		case RT9471_ICSTAT_SLEEP:
		case RT9471_ICSTAT_VBUSRDY:
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
			break;
		case RT9471_ICSTAT_TRICKLECHG:
		case RT9471_ICSTAT_PRECHG:
		case RT9471_ICSTAT_FASTCHG:
		case RT9471_ICSTAT_IEOC:
		case RT9471_ICSTAT_BGCHG:
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
			break;
		case RT9471_ICSTAT_CHGDONE:
			val->intval = POWER_SUPPLY_STATUS_FULL;
			break;
		case RT9471_ICSTAT_OTG:
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
			break;
		case RT9471_ICSTAT_CHGFAULT:
		default:
			val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
			break;
		}
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		ret = __rt9471_get_ic_stat(chip, &ic_stat);
		switch (ic_stat) {
		case RT9471_ICSTAT_SLEEP:
		case RT9471_ICSTAT_VBUSRDY:
		case RT9471_ICSTAT_CHGDONE:
		case RT9471_ICSTAT_OTG:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_NONE;
			break;
		case RT9471_ICSTAT_TRICKLECHG:
		case RT9471_ICSTAT_PRECHG:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
			break;
		case RT9471_ICSTAT_FASTCHG:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_FAST;
			break;
		case RT9471_ICSTAT_CHGFAULT:
		default:
			val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
			break;
		}
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = POWER_SUPPLY_HEALTH_UNKNOWN;
		ret = rt9471_i2c_read_byte(chip, RT9471_REG_STAT1, &stat1);
		if (ret < 0)
			break;
		ret = rt9471_i2c_read_byte(chip, RT9471_REG_STAT3, &stat3);
		if (ret < 0)
			break;

		if (stat1 & RT9471_ST_SYSOV_MASK ||
		    stat1 & RT9471_ST_BATOV_MASK ||
		    stat3 & RT9471_ST_VACOV_MASK)
			val->intval = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		else if (stat3 & RT9471_ST_OTP_MASK)
			val->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
		else if (stat3 & RT9471_ST_WDT_MASK)
			val->intval = POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE;
		else if (stat1 & RT9471_ST_TOUT_MASK)
			val->intval = POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE;
		else
			val->intval = POWER_SUPPLY_HEALTH_GOOD;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = atomic_read(&chip->vbus_gd);
		break;
	case POWER_SUPPLY_PROP_CHARGE_EMPTY_DESIGN:
		ret = __rt9471_is_shipmode(chip, &en);
		val->intval = en ? 1 : 0;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		ret = __rt9471_get_ieoc(chip, &val->intval);
		break;
	case POWER_SUPPLY_PROP_CHARGE_EMPTY:
		ret = __rt9471_is_hz_enabled(chip, &en);
		val->intval = en ? 1 : 0;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		ret = __rt9471_get_ichg(chip, &val->intval);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		ret = __rt9471_get_cv(chip, &val->intval);
		break;
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = "Richtek Technology Corporation";
		break;
	default:
		return -ENODATA;
	}

	return ret;
}

static int rt9471_psy_set_property(struct power_supply *psy,
				   enum power_supply_property prop,
				   const union power_supply_propval *val)
{
	int ret = 0;
	struct rt9471_chip *chip = power_supply_get_drvdata(psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGE_EMPTY_DESIGN:
		ret = __rt9471_enable_shipmode(chip, val->intval);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		ret = __rt9471_set_ieoc(chip, val->intval);
		break;
	case POWER_SUPPLY_PROP_CHARGE_EMPTY:
		ret = __rt9471_enable_hz(chip, val->intval);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		ret = __rt9471_set_ichg(chip, val->intval);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		ret = __rt9471_set_cv(chip, val->intval);
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static int rt9471_psy_is_writeable(struct power_supply *psy,
				   enum power_supply_property prop)
{
	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGE_EMPTY_DESIGN:
	case POWER_SUPPLY_PROP_CHARGE_FULL:
	case POWER_SUPPLY_PROP_CHARGE_EMPTY:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		return 1;
	default:
		return 0;
	}

	return 0;
}

static int rt9471_register_psy(struct rt9471_chip *chip)
{
	dev_info(chip->dev, "%s\n", __func__);

	chip->psy_desc.name = chip->desc->chg_name;
	chip->psy_desc.type = POWER_SUPPLY_TYPE_UNKNOWN;
	chip->psy_desc.properties = rt9471_psy_props;
	chip->psy_desc.num_properties = ARRAY_SIZE(rt9471_psy_props);
	chip->psy_desc.set_property = rt9471_psy_set_property;
	chip->psy_desc.get_property = rt9471_psy_get_property;
	chip->psy_desc.property_is_writeable = rt9471_psy_is_writeable;
	chip->psy_cfg.of_node = chip->dev->of_node;
	chip->psy_cfg.drv_data = chip;
	chip->psy = devm_power_supply_register(chip->dev, &chip->psy_desc,
					       &chip->psy_cfg);
	if (IS_ERR(chip->psy))
		return PTR_ERR(chip->psy);
	return 0;
}

static int rt9471_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int ret = 0;
	struct rt9471_chip *chip = NULL;

	dev_info(&client->dev, "%s (%s)\n", __func__, RT9471_DRV_VERSION);

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;
	chip->client = client;
	chip->dev = &client->dev;
	mutex_init(&chip->io_lock);
	mutex_init(&chip->bc12_lock);
	mutex_init(&chip->hidden_mode_lock);
	chip->hidden_mode_cnt = 0;
	INIT_WORK(&chip->init_work, rt9471_init_work_handler);
	atomic_set(&chip->vbus_gd, 0);
	chip->attach = false;
	chip->port = RT9471_PORTSTAT_NOINFO;
	chip->chg_done_once = false;
	INIT_DELAYED_WORK(&chip->buck_dwork, rt9471_buck_dwork_handler);
	i2c_set_clientdata(client, chip);

	if (!rt9471_check_devinfo(chip)) {
		ret = -ENODEV;
		goto err_nodev;
	}

	ret = rt9471_parse_dt(chip);
	if (ret < 0) {
		dev_notice(chip->dev, "%s parse dt fail(%d)\n", __func__, ret);
		goto err_parse_dt;
	}

	ret = rt9471_register_regmap(chip);
	if (ret < 0) {
		dev_notice(chip->dev, "%s register regmap fail(%d)\n",
				      __func__, ret);
		goto err_register_rm;
	}

	ret = rt9471_reset_register(chip);
	if (ret < 0)
		dev_notice(chip->dev, "%s reset register fail(%d)\n",
				      __func__, ret);

	ret = rt9471_init_setting(chip);
	if (ret < 0) {
		dev_notice(chip->dev, "%s init fail(%d)\n", __func__, ret);
		goto err_init;
	}

	ret = rt9471_register_psy(chip);
	if (ret < 0) {
		dev_notice(chip->dev, "%s register psy fail(%d)\n",
				      __func__, ret);
		goto err_psy;
	}

	ret = rt9471_register_irq(chip);
	if (ret < 0) {
		dev_notice(chip->dev, "%s register irq fail(%d)\n",
				      __func__, ret);
		goto err_register_irq;
	}

	ret = rt9471_init_irq(chip);
	if (ret < 0) {
		dev_notice(chip->dev, "%s init irq fail(%d)\n", __func__, ret);
		goto err_init_irq;
	}

#if IS_ENABLED(CONFIG_GPIOLIB)
	if (chip->dev_id == RT9470_DEVID) {
		rt9471_gpio_init(chip);
		chip->gpio.parent = chip->dev;
		chip->gpio.of_node = of_find_node_by_name(client->dev.of_node,
							  chip->gpio.label);
		if (!chip->gpio.of_node)
			dev_warn(chip->dev, "Failed to find %s DT node\n",
				chip->gpio.label);

		ret = devm_gpiochip_add_data(chip->dev, &chip->gpio, chip);
		dev_info(chip->dev, "%d GPIOs registered ret:%d\n",
			 chip->gpio.ngpio, ret);
	}
#endif

	(void)rt9471_create_fs_entries(chip);

	schedule_work(&chip->init_work);
	dev_info(chip->dev, "%s successfully\n", __func__);
	return 0;

err_init_irq:
err_register_irq:
	power_supply_unregister(chip->psy);
err_psy:
err_init:
err_register_rm:
err_parse_dt:
err_nodev:
	mutex_destroy(&chip->io_lock);
	mutex_destroy(&chip->bc12_lock);
	mutex_destroy(&chip->hidden_mode_lock);
	devm_kfree(chip->dev, chip);
	return ret;
}

static void rt9471_shutdown(struct i2c_client *client)
{
	struct rt9471_chip *chip = i2c_get_clientdata(client);

	dev_info(chip->dev, "%s\n", __func__);
	disable_irq(chip->irq);
	rt9471_reset_register(chip);
}

static void rt9471_remove(struct i2c_client *client)
{
	struct rt9471_chip *chip = i2c_get_clientdata(client);

	dev_info(chip->dev, "%s\n", __func__);
	disable_irq(chip->irq);
	disable_irq_wake(chip->irq);
	mutex_destroy(&chip->io_lock);
	mutex_destroy(&chip->bc12_lock);
	mutex_destroy(&chip->hidden_mode_lock);
}

static int rt9471_suspend(struct device *dev)
{
	struct rt9471_chip *chip = dev_get_drvdata(dev);

	pm_runtime_get_sync(chip->dev);
	dev_dbg(dev, "%s\n", __func__);
	disable_irq(chip->irq);
	pm_runtime_put_sync(chip->dev);

	return 0;
}

static int rt9471_resume(struct device *dev)
{
	struct rt9471_chip *chip = dev_get_drvdata(dev);

	pm_runtime_get_sync(chip->dev);
	dev_dbg(dev, "%s\n", __func__);
	enable_irq(chip->irq);
	pm_runtime_put_sync(chip->dev);

	return 0;
}

static SIMPLE_DEV_PM_OPS(rt9471_pm_ops, rt9471_suspend, rt9471_resume);

static const struct of_device_id rt9471_of_device_id[] = {
	{ .compatible = "richtek,rt9471", },
	{ .compatible = "richtek,swchg", },
	{ },
};
MODULE_DEVICE_TABLE(of, rt9471_of_device_id);

static const struct i2c_device_id rt9471_i2c_device_id[] = {
	{ "rt9471", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, rt9471_i2c_device_id);

static struct i2c_driver rt9471_i2c_driver = {
	.driver = {
		.name = "rt9471",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(rt9471_of_device_id),
		.pm = &rt9471_pm_ops,
	},
	.probe = rt9471_probe,
	.shutdown = rt9471_shutdown,
	.remove = rt9471_remove,
	.id_table = rt9471_i2c_device_id,
};
module_i2c_driver(rt9471_i2c_driver);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("ShuFanLee <shufan_lee@richtek.com>");
MODULE_AUTHOR("Lucas Tsai <lucas_tsai@richtek.com>");
MODULE_AUTHOR("Jack Wu <wjack@google.com>");
MODULE_DESCRIPTION("RT9471 Charger Driver");
MODULE_VERSION(RT9471_DRV_VERSION);
