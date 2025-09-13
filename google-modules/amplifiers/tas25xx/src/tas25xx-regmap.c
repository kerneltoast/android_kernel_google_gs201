/*
 * ALSA SoC Texas Instruments TAS25XX High Performance 4W Smart Amplifier
 *
 * Copyright (C) 2016 Texas Instruments, Inc.
 *
 * Author: Niranjan H Y, saiprasad
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */

//#define DEBUG 5
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/firmware.h>
#include <linux/regmap.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <sound/soc.h>
#include <sound/tlv.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/pm.h>
#include <linux/version.h>
#include "inc/tas25xx.h"
#include "inc/tas25xx-logic.h"
#include "inc/tas25xx-device.h"
#include "inc/tas25xx-codec.h"
#include "inc/tas25xx-regmap.h"
#include "inc/tas25xx-regbin-parser.h"
#include "inc/tas25xx-misc.h"
#include <linux/kobject.h>
//#if IS_ENABLED(CONFIG_TAS25XX_ALGO)
//#include "algo/inc/tas_smart_amp_v2.h"
//#endif /*CONFIG_TAS25XX_ALGO*/
/*For mixer_control implementation*/
#define MAX_STRING	200
#define SYS_NODE

#define CHK_ONE_BIT_SET(x) (((x) & ((x)-1)) == 0)

static const char *dts_tag[][2] = {
	{
		"ti,reset-gpio",
		"ti,irq-gpio"
	},
	{
		"ti,reset-gpio2",
		"ti,irq-gpio2"
	}
};

static const char *reset_gpio_label[2] = {
	"TAS25XX_RESET", "TAS25XX_RESET2"
};

static void (*tas_i2c_err_fptr)(uint32_t);

void tas25xx_register_i2c_error_callback(void (*i2c_err_cb)(uint32_t))
{
	tas_i2c_err_fptr = i2c_err_cb;
}
EXPORT_SYMBOL_GPL(tas25xx_register_i2c_error_callback);

static inline void tas25xx_post_i2c_err_to_platform(uint32_t i2caddr)
{
	if (tas_i2c_err_fptr)
		tas_i2c_err_fptr(i2caddr);
}

static int tas25xx_regmap_write(void *plat_data, uint32_t i2c_addr,
	uint32_t reg, uint32_t value, uint32_t channel)
{
	int ret;
	int retry_count = TAS25XX_I2C_RETRY_COUNT;
	struct linux_platform *platform_data =
		(struct linux_platform *)plat_data;

	platform_data->client->addr = i2c_addr;
	while (retry_count > 0) {
		ret = regmap_write(platform_data->regmap[channel], reg,
			value);
		if (!ret)
			break;
		tas25xx_post_i2c_err_to_platform(i2c_addr);
		mdelay(20);
		retry_count--;
	}

	return ret;
}

static int tas25xx_regmap_bulk_write(void *plat_data, uint32_t i2c_addr,
	uint32_t reg, uint8_t *pData,
	uint32_t nLength, uint32_t channel)
{
	int ret;
	int retry_count = TAS25XX_I2C_RETRY_COUNT;
	struct linux_platform *platform_data =
		(struct linux_platform *)plat_data;

	platform_data->client->addr = i2c_addr;
	while (retry_count > 0) {
		ret = regmap_bulk_write(platform_data->regmap[channel], reg,
			 pData, nLength);
		if (!ret)
			break;
		tas25xx_post_i2c_err_to_platform(i2c_addr);
		mdelay(20);
		retry_count--;
	}

	return ret;
}

static int tas25xx_regmap_read(void *plat_data, uint32_t i2c_addr,
	uint32_t reg, uint32_t *value, uint32_t channel)
{
	int ret;
	int retry_count = TAS25XX_I2C_RETRY_COUNT;
	struct linux_platform *platform_data =
		(struct linux_platform *)plat_data;

	platform_data->client->addr = i2c_addr;
	while (retry_count > 0) {
		ret = regmap_read(platform_data->regmap[channel], reg,
			value);
		if (!ret)
			break;
		tas25xx_post_i2c_err_to_platform(i2c_addr);
		mdelay(20);
		retry_count--;
	}

	return ret;
}

static int tas25xx_regmap_bulk_read(void *plat_data, uint32_t i2c_addr,
	uint32_t reg, uint8_t *pData,
	uint32_t nLength, uint32_t channel)
{
	int ret;
	int retry_count = TAS25XX_I2C_RETRY_COUNT;
	struct linux_platform *platform_data =
		(struct linux_platform *)plat_data;

	platform_data->client->addr = i2c_addr;
	while (retry_count > 0) {
		ret = regmap_bulk_read(platform_data->regmap[channel], reg,
			 pData, nLength);
		if (!ret)
			break;
		tas25xx_post_i2c_err_to_platform(i2c_addr);
		mdelay(20);
		retry_count--;
	}

	return ret;
}

static int tas25xx_regmap_update_bits(void *plat_data, uint32_t i2c_addr,
	uint32_t reg, uint32_t mask,
	uint32_t value, uint32_t channel)
{
	int ret;
	int retry_count = TAS25XX_I2C_RETRY_COUNT;
	struct linux_platform *platform_data =
		(struct linux_platform *)plat_data;

	platform_data->client->addr = i2c_addr;
	while (retry_count > 0) {
		ret = regmap_update_bits(platform_data->regmap[channel], reg,
			mask, value);
		if (!ret)
			break;
		tas25xx_post_i2c_err_to_platform(i2c_addr);
		mdelay(20);
		retry_count--;
	}

	return ret;
}

static int tas25xx_dev_read(struct tas25xx_priv *p_tas25xx,
	int32_t chn, uint32_t reg, uint32_t *pValue)
{
	int ret = -EINVAL;

	mutex_lock(&p_tas25xx->dev_lock);

	/* Switch book */
	if (TAS25XX_BOOK_ID(reg)) {
		ret = tas25xx_change_book(p_tas25xx, chn,
			TAS25XX_BOOK_ID(reg));
		if (ret)
			goto end;
	}

	ret = p_tas25xx->plat_read(p_tas25xx->platform_data,
			p_tas25xx->devs[chn]->mn_addr,
			TAS25XX_REG_NO_BOOK(reg), pValue, chn);
	if (ret) {
		pr_err("%s, ERROR, L=%d, E=%d\n",
			__func__, __LINE__, ret);
	} else {
		pr_debug(
			"%s: chn:%x:BOOK:PAGE:REG 0x%02x:0x%02x:0x%02x,0x%02x\n",
			__func__,
			p_tas25xx->devs[chn]->mn_addr, TAS25XX_BOOK_ID(reg),
			TAS25XX_PAGE_ID(reg),
			TAS25XX_PAGE_REG(reg), *pValue);
	}

	/* Switch to Book-0 */
	if (TAS25XX_BOOK_ID(reg))
		ret = tas25xx_change_book(p_tas25xx, chn, 0);

end:
	mutex_unlock(&p_tas25xx->dev_lock);
	return ret;
}

static int tas25xx_dev_write(struct tas25xx_priv *p_tas25xx, int32_t chn,
	uint32_t reg, uint32_t value)
{
	int ret = -EINVAL;

	mutex_lock(&p_tas25xx->dev_lock);

	/* Switch book */
	if (TAS25XX_BOOK_ID(reg)) {
		ret = tas25xx_change_book(p_tas25xx, chn,
			TAS25XX_BOOK_ID(reg));
		if (ret)
			goto end;
	}

	ret = p_tas25xx->plat_write(
			p_tas25xx->platform_data,
			p_tas25xx->devs[chn]->mn_addr,
			TAS25XX_REG_NO_BOOK(reg), value, chn);
	if (ret) {
		pr_err(
			"%s, ERROR, L=%u, chn=0x%02x, E=%d\n",
			__func__, __LINE__,
			p_tas25xx->devs[chn]->mn_addr, ret);
	} else {
		pr_debug(
			"%s: %u: chn:0x%02x:BOOK:PAGE:REG 0x%02x:0x%02x:0x%02x, VAL: 0x%02x\n",
			__func__, __LINE__,
			p_tas25xx->devs[chn]->mn_addr,
			TAS25XX_BOOK_ID(reg),
			TAS25XX_PAGE_ID(reg),
			TAS25XX_PAGE_REG(reg), value);

	}

	/* Switch book to 0 */
	if (TAS25XX_BOOK_ID(reg)) {
		ret = tas25xx_change_book(p_tas25xx, chn, 0);
		if (ret)
			goto end;
	}

end:
	mutex_unlock(&p_tas25xx->dev_lock);
	return ret;
}

static int tas25xx_dev_bulk_write(struct tas25xx_priv *p_tas25xx,
	int32_t chn, uint32_t reg, uint8_t *p_data, uint32_t n_length)
{
	int ret = -EINVAL;

	mutex_lock(&p_tas25xx->dev_lock);

	/* Switch book */
	if (TAS25XX_BOOK_ID(reg)) {
		ret = tas25xx_change_book(p_tas25xx, chn,
			TAS25XX_BOOK_ID(reg));
		if (ret)
			goto end;
	}

	ret = p_tas25xx->plat_bulk_write(
			p_tas25xx->platform_data,
			p_tas25xx->devs[chn]->mn_addr,
			TAS25XX_REG_NO_BOOK(reg),
			p_data, n_length, chn);
	if (ret) {
		pr_err(
			"%s, ERROR, L=%u, chn=0x%02x: E=%d\n",
			__func__, __LINE__,
			p_tas25xx->devs[chn]->mn_addr, ret);
	} else {
		pr_debug(
			"%s: chn%x:BOOK:PAGE:REG 0x%02x:0x%02x:0x%02x, len: %u\n",
			__func__, p_tas25xx->devs[chn]->mn_addr,
			TAS25XX_BOOK_ID(reg),
			TAS25XX_PAGE_ID(reg),
			TAS25XX_PAGE_REG(reg), n_length);
	}

	/* Switch book to 0*/
	if (TAS25XX_BOOK_ID(reg)) {
		ret = tas25xx_change_book(p_tas25xx, chn, 0);
		if (ret)
			goto end;
	}

end:
	mutex_unlock(&p_tas25xx->dev_lock);
	return ret;
}

static int tas25xx_dev_bulk_read(struct tas25xx_priv *p_tas25xx,
	int32_t chn, uint32_t reg, uint8_t *p_data, uint32_t n_length)
{
	int ret;

	mutex_lock(&p_tas25xx->dev_lock);

	/* Switch book */
	if (TAS25XX_BOOK_ID(reg)) {
		ret = tas25xx_change_book(p_tas25xx, chn,
			TAS25XX_BOOK_ID(reg));
		if (ret)
			goto end;
	}

	ret = p_tas25xx->plat_bulk_read(p_tas25xx->platform_data,
		p_tas25xx->devs[chn]->mn_addr, TAS25XX_REG_NO_BOOK(reg),
		p_data, n_length, chn);
	if (ret) {
		pr_err("%s, ERROR, L=%d, E=%d\n", __func__, __LINE__, ret);
	} else {
		pr_debug(
			"%s: chn%x:BOOK:PAGE:REG %u:%u:%u, len: 0x%02x\n",
			__func__, p_tas25xx->devs[chn]->mn_addr,
			TAS25XX_BOOK_ID(reg), TAS25XX_PAGE_ID(reg),
			TAS25XX_PAGE_REG(reg), n_length);
	}

	/* Switch book to 0 */
	if (TAS25XX_BOOK_ID(reg)) {
		ret = tas25xx_change_book(p_tas25xx, chn, 0);
		if (ret)
			goto end;
	}

end:
	mutex_unlock(&p_tas25xx->dev_lock);
	return ret;
}

static int tas25xx_dev_update_bits(struct tas25xx_priv *p_tas25xx,
	int32_t chn, uint32_t reg, uint32_t mask, uint32_t value)
{
	int ret;

	mutex_lock(&p_tas25xx->dev_lock);

	/* Switch book */
	if (TAS25XX_BOOK_ID(reg)) {
		ret = tas25xx_change_book(p_tas25xx, chn,
			TAS25XX_BOOK_ID(reg));
		if (ret)
			goto end;
	}

	ret = p_tas25xx->plat_update_bits(
			p_tas25xx->platform_data,
			p_tas25xx->devs[chn]->mn_addr,
			TAS25XX_REG_NO_BOOK(reg),
			mask, value, chn);
	if (ret) {
		pr_err("%s, ERROR, L=%u, chn=0x%02x: E=%d\n", __func__, __LINE__,
			p_tas25xx->devs[chn]->mn_addr, ret);
	} else {
		pr_debug("%s: chn%x:BOOK:PAGE:REG 0x%02x:0x%02x:0x%02x, mask: 0x%02x, val: 0x%02x\n",
			__func__, p_tas25xx->devs[chn]->mn_addr,
				TAS25XX_BOOK_ID(reg), TAS25XX_PAGE_ID(reg), TAS25XX_PAGE_REG(reg), mask, value);
	}

	/* Switch book to 0*/
	if (TAS25XX_BOOK_ID(reg)) {
		ret = tas25xx_change_book(p_tas25xx, chn, 0);
		if (ret)
			goto end;
	}

end:
	mutex_unlock(&p_tas25xx->dev_lock);
	return ret;
}

/*
 * consider all the registers as volatile,
 * this list is not known to the driver at the time of regmap init
 */
static bool tas25xx_volatile(struct device *dev, unsigned int reg)
{
	return true;
}

/*
 * consider all the registers as writable as
 * this list is not known to the driver at the time of regmap init
 */
static bool tas25xx_writeable(struct device *dev, unsigned int reg)
{
	return true;
}

struct regmap_range_cfg tas25xx_ranges[] = {
	{
		.name = "tas25xx range",
		.range_min = 0,
		.range_max = 256 * 128,
		.selector_reg = TAS25XX_PAGECTL_REG,
		.selector_mask = 0xFF,
		.selector_shift = 0,
		.window_start = 0,
		.window_len = 128,
	},
};

static const struct reg_default tas25xx_reg_defaults[] = {
	{ TAS25XX_PAGECTL_REG, 0x00 },
};

static const struct regmap_config tas25xx_i2c_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.writeable_reg = tas25xx_writeable,
	.volatile_reg = tas25xx_volatile,
	.reg_defaults = tas25xx_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(tas25xx_reg_defaults),
	.max_register = 256 * 128,
	.ranges = tas25xx_ranges,
	.num_ranges = ARRAY_SIZE(tas25xx_ranges),
	.cache_type = REGCACHE_RBTREE,
};

#define TEST_HW_RESET 0

static void tas25xx_hw_reset(struct tas25xx_priv *p_tas25xx)
{
	struct linux_platform *plat_data = NULL;
	int i = 0;
#if TEST_HW_RESET
	uint32_t regval_b = 0;
	uint32_t regval_c = 0;
	uint32_t regval_d = 0;
#endif

	plat_data = (struct linux_platform *) p_tas25xx->platform_data;

	dev_info(plat_data->dev, "Before HW reset\n");
#if TEST_HW_RESET
	p_tas25xx->update_bits(p_tas25xx, 0, 0xb, 0x40, 0x40);
	p_tas25xx->update_bits(p_tas25xx, 0, 0xb, 0x3F, 0x07);

	p_tas25xx->update_bits(p_tas25xx, 0, 0xc, 0x40, 0x40);
	p_tas25xx->update_bits(p_tas25xx, 0, 0xc, 0x3F, 0x08);

	p_tas25xx->update_bits(p_tas25xx, 0, 0xd, 0x40, 0x40);
	p_tas25xx->update_bits(p_tas25xx, 0, 0xd, 0x3F, 0x09);

	dev_info(plat_data->dev, "----START----\n");

	p_tas25xx->read(p_tas25xx, 0, 0xb, &regval_b);
	p_tas25xx->read(p_tas25xx, 0, 0xc, &regval_c);
	p_tas25xx->read(p_tas25xx, 0, 0xd, &regval_d);

	dev_info(plat_data->dev, "%02x %02x %02x",
		regval_b, regval_c, regval_d);
#endif

	for (i = 0; i < p_tas25xx->ch_count; i++) {
		if (gpio_is_valid(p_tas25xx->devs[i]->reset_gpio)) {
			dev_info(plat_data->dev, "GPIO is valid %d\n",
				p_tas25xx->devs[i]->reset_gpio);
			gpio_direction_output(p_tas25xx->devs[i]->reset_gpio, 0);
		} else {
			dev_dbg(plat_data->dev, "GPIO is not valid %d\n",
				p_tas25xx->devs[i]->reset_gpio);
		}
	}
	msleep(20);

	for (i = 0; i < p_tas25xx->ch_count; i++) {
		if (gpio_is_valid(p_tas25xx->devs[i]->reset_gpio))
			gpio_direction_output(p_tas25xx->devs[i]->reset_gpio, 1);
		p_tas25xx->devs[i]->mn_current_book = -1;
	}
	msleep(20);

	dev_info(plat_data->dev, "After HW reset\n");

#if TEST_HW_RESET
	p_tas25xx->read(p_tas25xx, 0, 0xb, &regval_b);
	p_tas25xx->read(p_tas25xx, 0, 0xc, &regval_c);
	p_tas25xx->read(p_tas25xx, 0, 0xd, &regval_d);

	dev_info(plat_data->dev, "%02x %02x %02x",
		regval_b, regval_c, regval_d);
	dev_info(plat_data->dev, "---- END ----\n");
#endif

	dev_info(plat_data->dev, "%s exit\n", __func__);
}

static void tas25xx_enable_irq(struct tas25xx_priv *p_tas25xx)
{
	struct irq_desc *desc = NULL;
	struct linux_platform *plat_data = NULL;
	int i = 0;
	int irq_no;
	int irq_gpio;
	uint32_t reg;
	uint32_t value;
	int32_t ret;

	plat_data = (struct linux_platform *) p_tas25xx->platform_data;

	for (i = 0; i < p_tas25xx->ch_count; i++) {
		if (p_tas25xx->ch_failed[i] == true) {
			dev_err(plat_data->dev, "%s: ch%d skip\n", __func__, i);
			continue;
		}
		irq_gpio = p_tas25xx->devs[i]->irq_gpio;
		if (!gpio_is_valid(irq_gpio))
			continue;

		if (p_tas25xx->irq_registered[i] == 0)
			continue;

		// Update IRQZ setting
		if (p_tas25xx->irqz_value != -1) {
			value = p_tas25xx->irqz_value;
			reg = TAS25XX_REG(0, 0, 0x1C);
			ret = p_tas25xx->write(p_tas25xx, i, reg, value);
			if (ret < 0) {
				dev_err(plat_data->dev, "%s: write reg %x fail=%d\n",
					__func__, reg, ret);
			}
		}

		if (p_tas25xx->irq_enabled[i] == 0) {
			irq_no = p_tas25xx->devs[i]->irq_no;
			desc = irq_data_to_desc(irq_get_irq_data(irq_no));
			if (desc && desc->depth > 0) {
				enable_irq(irq_no);
				dev_info(plat_data->dev, "irq_no=%d(gpio=%d), enabled irq",
					irq_no, irq_gpio);
				p_tas25xx->irq_enabled[i] = 1;
			} else if (desc) {
				p_tas25xx->irq_enabled[i] = 1;
				dev_info(plat_data->dev, "irq_no=%d already enabled", irq_no);
			} else {
				dev_info(plat_data->dev, "failed to get descritptor");
			}
		} else {
			dev_info(plat_data->dev, "GPIO %d, previous IRQ status=%s",
			irq_gpio, p_tas25xx->irq_enabled[i] ? "Enabled" : "Disabled");
		}
	}
}

static void tas25xx_disable_irq(struct tas25xx_priv *p_tas25xx)
{
	int i = 0;
	int irq_gpio;
	int irq_no;

	struct linux_platform *plat_data =
		(struct linux_platform *) p_tas25xx->platform_data;

	for (i = 0; i < p_tas25xx->ch_count; i++) {
		irq_gpio = p_tas25xx->devs[i]->irq_gpio;
		irq_no = p_tas25xx->devs[i]->irq_no;
		if (!gpio_is_valid(irq_gpio))
			continue;
		if (p_tas25xx->irq_registered[i] == 0)
			continue;
		if (p_tas25xx->irq_enabled[i] == 1) {
			dev_info(plat_data->dev, "irq_no=%d(gpio=%d), disabling IRQ", irq_no, irq_gpio);
			disable_irq_nosync(irq_no);
			p_tas25xx->irq_enabled[i] = 0;
		}
	}
}

static void irq_work_routine(struct work_struct *work)
{
	struct tas25xx_priv *p_tas25xx =
		container_of(work, struct tas25xx_priv, irq_work.work);
	struct linux_platform *plat_data = NULL;

	plat_data = (struct linux_platform *) p_tas25xx->platform_data;
	dev_info(plat_data->dev, "%s\n", __func__);

	/* Codec Lock Hold*/
	mutex_lock(&p_tas25xx->codec_lock);

	if (plat_data->mb_runtime_suspend) {
		pr_info("%s, Runtime Suspended\n", __func__);
	} else {
		/*Logical Layer IRQ function, return is ignored*/
		tas25xx_irq_work_func(p_tas25xx);
	}

	/* Codec Lock Release*/
	mutex_unlock(&p_tas25xx->codec_lock);

	return;
}

static void init_work_routine(struct work_struct *work)
{
	int ret, chn;
	struct tas_device *dev_tas25xx =
		container_of(work, struct tas_device, init_work.work);
	struct tas25xx_priv *p_tas25xx = dev_tas25xx->prv_data;
	struct linux_platform *plat_data = NULL;

	plat_data = (struct linux_platform *) p_tas25xx->platform_data;
	chn = dev_tas25xx->channel_no;

	mutex_lock(&p_tas25xx->codec_lock);

	if (p_tas25xx->m_power_state == TAS_POWER_ACTIVE) {
		tas25xx_init_work_func(p_tas25xx, dev_tas25xx);
		ret = tas25xx_update_kcontrol_data(p_tas25xx,
			KCNTR_POST_POWERUP, 1 << chn);
		if (ret)
			dev_err(plat_data->dev,
				"%s error during post powerup kcontrol update", __func__);
	} else {
		dev_info(plat_data->dev,
			"skipping init work as the device state was changed");
	}

	mutex_unlock(&p_tas25xx->codec_lock);
}

static int tas25xx_runtime_suspend(struct tas25xx_priv *p_tas25xx)
{
	struct linux_platform *plat_data = NULL;

	plat_data = (struct linux_platform *) p_tas25xx->platform_data;
	dev_dbg(plat_data->dev, "%s\n", __func__);

	plat_data->mb_runtime_suspend = true;

	if (delayed_work_pending(&p_tas25xx->irq_work)) {
		dev_dbg(plat_data->dev, "cancel IRQ work\n");
		cancel_delayed_work(&p_tas25xx->irq_work);
	}

	return 0;
}

static int tas25xx_runtime_resume(struct tas25xx_priv *p_tas25xx)
{
	struct linux_platform *plat_data = NULL;

	plat_data = (struct linux_platform *) p_tas25xx->platform_data;
	dev_dbg(plat_data->dev, "%s\n", __func__);

	plat_data->mb_runtime_suspend = false;

	return 0;
}

static int tas25xx_pm_suspend(struct device *dev)
{
	struct tas25xx_priv *p_tas25xx = dev_get_drvdata(dev);
	struct linux_platform *plat_data = NULL;
	int i = 0;

	if (!p_tas25xx) {
		pr_err("tas25xx:%s drvdata is NULL\n", __func__);
		return -EINVAL;
	}
	plat_data = (struct linux_platform *) p_tas25xx->platform_data;
	if (!plat_data) {
		pr_err("tas25xx:%s plat_data is NULL\n", __func__);
		return -EINVAL;
	}

	/* Codec Lock Hold*/
	mutex_lock(&p_tas25xx->codec_lock);
	plat_data->i2c_suspend = true;
	/* Only cache register writes and no actual I2C writes */
	for (i = 0; i < p_tas25xx->ch_count; i++)
		regcache_cache_only(plat_data->regmap[i], true);
	tas25xx_runtime_suspend(p_tas25xx);
	/* Codec Lock Release*/
	mutex_unlock(&p_tas25xx->codec_lock);
	return 0;
}

static int tas25xx_pm_resume(struct device *dev)
{
	struct tas25xx_priv *p_tas25xx = dev_get_drvdata(dev);
	struct linux_platform *plat_data = NULL;
	int i = 0;

	if (!p_tas25xx) {
		pr_err("tas25xx:%s drvdata is NULL\n", __func__);
		return -EINVAL;
	}
	plat_data = (struct linux_platform *) p_tas25xx->platform_data;
	if (!plat_data) {
		pr_err("tas25xx:%s plat_data is NULL\n", __func__);
		return -EINVAL;
	}

	/* Codec Lock Hold*/
	mutex_lock(&p_tas25xx->codec_lock);
	plat_data->i2c_suspend = false;

	/* Restore all I2C writes happend during supended state */
	for (i = 0; i < p_tas25xx->ch_count; i++) {
		regcache_cache_only(plat_data->regmap[i], false);
		regcache_sync(plat_data->regmap[i]);
	}

	tas25xx_runtime_resume(p_tas25xx);
	/* Codec Lock Release*/
	mutex_unlock(&p_tas25xx->codec_lock);
	return 0;
}


static void schedule_init_work(struct tas25xx_priv *p_tas25xx, int ch)
{
	/* actual init work will decide the delay */
	schedule_delayed_work(&p_tas25xx->devs[ch]->init_work,
		msecs_to_jiffies(0));
}

static void cancel_init_work(struct tas25xx_priv *p_tas25xx, int ch)
{
	//struct linux_platform *plat_data = NULL;

	//plat_data = (struct linux_platform *) p_tas25xx->platform_data;
	cancel_delayed_work(&p_tas25xx->devs[ch]->init_work);

}

static int tas25xx_parse_dt(struct device *dev,
				struct tas25xx_priv *p_tas25xx)
{

	struct device_node *np = dev->of_node;
	struct linux_platform *plat_data = NULL;
	int rc = 0, i = 0;
	uint32_t reset_gpios = 0;
	uint32_t irq_gpios = 0;
	uint8_t buf[32];

	plat_data = (struct linux_platform *) p_tas25xx->platform_data;

	rc = of_property_read_u32(np, "ti,max-channels", &p_tas25xx->ch_count);
	if (rc) {
		dev_err(plat_data->dev,
			"Looking up %s property in node %s, err=%d\n, using default=%d",
			"ti,max-channels", np->full_name, rc, MAX_CHANNELS);
		p_tas25xx->ch_count = MAX_CHANNELS;
	} else {
		dev_dbg(plat_data->dev, "ti,max-channels=%d",
			p_tas25xx->ch_count);
	}

	if (p_tas25xx->ch_count > MAX_CHANNELS) {
		dev_err(plat_data->dev,
			"ti,max-channels count exceeds limit(%d)", MAX_CHANNELS);
		goto EXIT;
	}

	/*the device structures array*/
	p_tas25xx->devs = kmalloc(p_tas25xx->ch_count * sizeof(struct tas_device *),
		GFP_KERNEL);
	for (i = 0; i < p_tas25xx->ch_count; i++) {
		p_tas25xx->devs[i] = kmalloc(sizeof(struct tas_device),
					GFP_KERNEL);
		if (!p_tas25xx->devs[i]) {
			rc = -ENOMEM;
			break;
		}

		p_tas25xx->devs[i]->reset_gpio = of_get_named_gpio(np, dts_tag[i][0], 0);
		if (!gpio_is_valid(p_tas25xx->devs[i]->reset_gpio)) {
			dev_dbg(plat_data->dev, "Looking up %s property in node %s failed %d\n",
				dts_tag[i][0], np->full_name, p_tas25xx->devs[i]->reset_gpio);
		} else {
			dev_info(plat_data->dev, "%s = %d", dts_tag[i][0],
				p_tas25xx->devs[i]->reset_gpio);
			reset_gpios |= 1 << i;
		}

		p_tas25xx->devs[i]->irq_gpio = of_get_named_gpio(np, dts_tag[i][1], 0);
		if (!gpio_is_valid(p_tas25xx->devs[i]->irq_gpio)) {
			dev_dbg(plat_data->dev,
				"Looking up %s property in node %s failed %d\n",
				dts_tag[i][1], np->full_name,
				p_tas25xx->devs[i]->irq_gpio);
		} else {
			dev_info(plat_data->dev, "%s = %d", dts_tag[i][1],
				p_tas25xx->devs[i]->irq_gpio);
			irq_gpios |= 1 << i;
		}
	}

	for (i = 0; i < p_tas25xx->ch_count; i++) {
		snprintf(buf, 32, "ti,channel-%d", i);
		rc = of_property_read_u32(np, buf,
			&p_tas25xx->devs[i]->mn_addr);
		if (rc) {
			dev_warn(plat_data->dev,
				"Looking up %s property in node %s failed %d\n",
				buf, np->full_name, rc);
			p_tas25xx->devs[i]->mn_addr = -1;
			rc = 0;
		} else {
			dev_info(plat_data->dev, "using %s = 0x%2x\n", buf,
				p_tas25xx->devs[i]->mn_addr);
		}
	}

	if (!irq_gpios)
		dev_err(plat_data->dev, "IRQ GPIOs not found");
	else if (CHK_ONE_BIT_SET(irq_gpios))
		dev_info(plat_data->dev, "Using commong IRQ gpio");

	if (!reset_gpios)
		dev_err(plat_data->dev, "Reset GPIOs not found");
	else if (CHK_ONE_BIT_SET(reset_gpios))
		dev_info(plat_data->dev, "Using commong reset gpio");

	rc = of_property_read_u32(np, "ti,irqz-value", &p_tas25xx->irqz_value);
	if (rc) {
		dev_warn(plat_data->dev,
			"Looking up %s property in node %s, err=%d, ignored\n",
			"ti,irqz-value", np->full_name, rc);
		p_tas25xx->irqz_value = -1;
		rc = 0;
	} else
		dev_dbg(plat_data->dev, "ti,irqz-value=%x", p_tas25xx->irqz_value);

EXIT:
	return rc;
}

static int tas25xx_i2c_probe(struct i2c_client *p_client,
			const struct i2c_device_id *id)
{
	struct tas25xx_priv *p_tas25xx;
	struct linux_platform *plat_data;
	int ret = 0;
	int i = 0;

	dev_info(&p_client->dev, "Driver Tag: %s, addr=0x%2x\n",
		TAS25XX_DRIVER_TAG, p_client->addr);

	p_tas25xx = devm_kzalloc(&p_client->dev,
		sizeof(struct tas25xx_priv), GFP_KERNEL);
	if (p_tas25xx == NULL) {
		ret = -ENOMEM;
		goto err;
	}

	plat_data = (struct linux_platform *)devm_kzalloc(&p_client->dev,
		sizeof(struct linux_platform), GFP_KERNEL);
	if (plat_data == NULL) {
		ret = -ENOMEM;
		goto err;
	}

	p_tas25xx->platform_data = plat_data;
	/* REGBIN related */
	//p_tas25xx->profile_cfg_id = -1;

	/* high level api */
	p_tas25xx->read = tas25xx_dev_read;
	p_tas25xx->write = tas25xx_dev_write;
	p_tas25xx->bulk_read = tas25xx_dev_bulk_read;
	p_tas25xx->bulk_write = tas25xx_dev_bulk_write;
	p_tas25xx->update_bits = tas25xx_dev_update_bits;

	/* low level api */
	p_tas25xx->plat_write = tas25xx_regmap_write;
	p_tas25xx->plat_read = tas25xx_regmap_read;
	p_tas25xx->plat_bulk_write = tas25xx_regmap_bulk_write;
	p_tas25xx->plat_bulk_read = tas25xx_regmap_bulk_read;
	p_tas25xx->plat_update_bits = tas25xx_regmap_update_bits;

	plat_data->client = p_client;
	plat_data->dev = &p_client->dev;
	i2c_set_clientdata(p_client, p_tas25xx);
	dev_set_drvdata(&p_client->dev, p_tas25xx);

	mutex_init(&p_tas25xx->dev_lock);
	p_tas25xx->hw_reset = tas25xx_hw_reset;
	p_tas25xx->enable_irq = tas25xx_enable_irq;
	p_tas25xx->disable_irq = tas25xx_disable_irq;
	p_tas25xx->schedule_init_work = schedule_init_work;
	p_tas25xx->cancel_init_work = cancel_init_work;
#if IS_ENABLED(CODEC_PM)
	plat_data->runtime_suspend = tas25xx_runtime_suspend;
	plat_data->runtime_resume = tas25xx_runtime_resume;
#endif
	p_tas25xx->m_power_state = TAS_POWER_SHUTDOWN;
	p_tas25xx->left_em_mode = -1;
	p_tas25xx->right_em_mode = -1;

	if (p_client->dev.of_node) {
		ret = tas25xx_parse_dt(&p_client->dev, p_tas25xx);
		if (ret)
			goto err;
	}

	for (i = 0; i < p_tas25xx->ch_count; i++) {
		if (gpio_is_valid(p_tas25xx->devs[i]->reset_gpio)) {
			ret = gpio_request(p_tas25xx->devs[i]->reset_gpio,
					reset_gpio_label[i]);
			if (ret) {
				dev_err(plat_data->dev,
					"%s: Failed to request gpio %d\n",
					__func__,
					p_tas25xx->devs[i]->reset_gpio);
				ret = -EINVAL;
				goto err;
			} else {
				/* make the gpio output always one */
				dev_info(plat_data->dev, "%s setting reset gpio %d always high, default",
					__func__, p_tas25xx->devs[i]->reset_gpio);
				gpio_direction_output(p_tas25xx->devs[i]->reset_gpio, 1);
			}

		}
		plat_data->regmap[i] = devm_regmap_init_i2c(p_client,
			&tas25xx_i2c_regmap);
		if (IS_ERR(plat_data->regmap[i])) {
			ret = PTR_ERR(plat_data->regmap[i]);
			dev_err(&p_client->dev,
				"Failed to allocate register map: %d channel %d\n",
				ret, i);
			goto err;
		}
	}

	ret = tas25xx_register_device(p_tas25xx);
	if (ret)
		goto err;

	ret = 0;
	for (i = 0; i < p_tas25xx->ch_count; i++) {
		if (p_tas25xx->devs[i]->mn_addr != -1) {
			/* all i2c addressess registered must be accessible */
			ret = p_tas25xx->read(p_tas25xx, i,
				TAS25XX_REVID_REG, &p_tas25xx->dev_revid);
			if (!ret) {
				dev_info(&p_client->dev,
					"successfully read revid 0x%x\n", p_tas25xx->dev_revid);
			} else {
				dev_err(&p_client->dev,
					"Unable to read rev id, i2c failure\n");
				break;
			}
		}
	}

	/* consider i2c error as fatal */
	if (ret)
		goto err;

	init_waitqueue_head(&p_tas25xx->fw_wait);

	init_waitqueue_head(&p_tas25xx->dev_init_wait);
	atomic_set(&p_tas25xx->dev_init_status, 0);

	INIT_DELAYED_WORK(&p_tas25xx->irq_work, irq_work_routine);
	for (i = 0; i < p_tas25xx->ch_count; i++) {
		p_tas25xx->devs[i]->prv_data = p_tas25xx;
		INIT_DELAYED_WORK(&p_tas25xx->devs[i]->init_work,
			init_work_routine);
	}

	mutex_init(&p_tas25xx->codec_lock);
	ret = tas25xx_register_codec(p_tas25xx);
	if (ret) {
		dev_err(plat_data->dev,
			"register codec failed, %d\n", ret);
		goto err;
	}

	mutex_init(&p_tas25xx->file_lock);
	ret = tas25xx_register_misc(p_tas25xx);
	if (ret) {
		dev_err(plat_data->dev, "register codec failed %d\n",
			ret);
		goto err;
	}

err:
	return ret;
}

static void tas25xx_i2c_remove(struct i2c_client *p_client)
{
	int i = 0;
	struct tas25xx_priv *p_tas25xx = i2c_get_clientdata(p_client);
	struct linux_platform *plat_data = NULL;

	plat_data = (struct linux_platform *) p_tas25xx->platform_data;
	dev_info(plat_data->dev, "%s\n", __func__);

	/*Cancel all the work routine before exiting*/
	for (i = 0; i < p_tas25xx->ch_count; i++)
		cancel_delayed_work_sync(&p_tas25xx->devs[i]->init_work);

	cancel_delayed_work_sync(&p_tas25xx->irq_work);
	cancel_delayed_work_sync(&p_tas25xx->dc_work);

	tas25xx_deregister_codec(p_tas25xx);
	mutex_destroy(&p_tas25xx->codec_lock);

	tas25xx_deregister_misc(p_tas25xx);
	mutex_destroy(&p_tas25xx->file_lock);

	mutex_destroy(&p_tas25xx->dev_lock);

	for (i = 0; i < p_tas25xx->ch_count; i++) {
		if (gpio_is_valid(p_tas25xx->devs[i]->reset_gpio))
			gpio_free(p_tas25xx->devs[i]->reset_gpio);
		if (gpio_is_valid(p_tas25xx->devs[i]->irq_gpio))
			gpio_free(p_tas25xx->devs[i]->irq_gpio);
		kfree(p_tas25xx->devs[i]);
	}

	kfree(p_tas25xx->devs);
}


static const struct i2c_device_id tas25xx_i2c_id[] = {
	{ "tas25xx", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, tas25xx_i2c_id);

#if defined(CONFIG_OF)
static const struct of_device_id tas25xx_of_match[] = {
	{ .compatible = "ti,tas25xx" },
	{},
};
MODULE_DEVICE_TABLE(of, tas25xx_of_match);
#endif

static const struct dev_pm_ops tas25xx_pm_ops = {
	.suspend = tas25xx_pm_suspend,
	.resume = tas25xx_pm_resume
};

static struct i2c_driver tas25xx_i2c_driver = {
	.driver = {
		.name   = "tas25xx",
		.owner  = THIS_MODULE,
#if defined(CONFIG_OF)
		.of_match_table = of_match_ptr(tas25xx_of_match),
#endif
		.pm = &tas25xx_pm_ops,
	},
	.probe      = tas25xx_i2c_probe,
	.remove     = tas25xx_i2c_remove,
	.id_table   = tas25xx_i2c_id,
};

module_i2c_driver(tas25xx_i2c_driver);

MODULE_AUTHOR("Texas Instruments Inc.");
MODULE_DESCRIPTION("TAS25XX I2C Smart Amplifier driver");
MODULE_LICENSE("GPL v2");
