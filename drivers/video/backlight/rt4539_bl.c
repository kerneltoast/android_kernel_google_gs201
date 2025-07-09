// SPDX-License-Identifier: GPL-2.0-only
/*
 * RT4539 Backlight Driver
 *
 * Copyright (C) 2021 Google LLC.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>
#include "rt4539.h"

#define DEFAULT_BL_NAME				"lcd-backlight"
#define BIT_SELECTION_MIN_BITS			8
#define BIT_SELECTION_MAX_BITS			12

#define RT4539_REG00				(0x00)
#define RT4539_REG01				(0x01)
#define RT4539_REG02				(0x02)
#define RT4539_REG03				(0x03)
#define RT4539_REG04				(0x04)
#define RT4539_REG05				(0x05)
#define RT4539_REG06				(0x06)
#define RT4539_REG07				(0x07)
#define RT4539_REG09				(0x09)
#define RT4539_REG0A				(0x0A)
#define RT4539_REG0B				(0x0B)

/* masks and shifts */
#define RT4539_REG00_DIMMING_MODE_MASK		(0x07)
#define RT4539_REG01_BOOST_SWITCH_FREQ_MASK	(0x0F)
#define RT4539_REG03_BIT_SELECTION_MASK		(0x07)
#define RT4539_REG03_LED_DRIVER_HEADROOM_MASK	(0x60)
#define RT4539_REG03_LED_DRIVER_HEADROOM_SHIFT	(5)
#define RT4539_REG03_ILED_MAPPING_MASK		(0x80)
#define RT4539_REG03_ILED_MAPPING_SHIFT		(7)
#define RT4539_REG04_BRIGHTNESS_MSB_MASK	(0x0F)
#define RT4539_REG06_FADE_IN_OUT_TIME_CTRL_MASK	(0x07)
#define RT4539_REG06_SLOPE_TIME_CTRL_MASK	(0x38)
#define RT4539_REG06_SLOPE_TIME_CTRL_SHIFT	(3)
#define RT4539_REG06_SLOPE_TIME_FILTER_MASK	(0xC0)
#define RT4539_REG06_SLOPE_TIME_FILTER_SHIFT	(6)
#define RT4539_REG07_ADV_BRIGHT_CTRL_MASK	(0x03)
#define RT4539_REG09_PFM_ENABLE_MASK		(0x01)
#define RT4539_REG09_LED_UNUSED_CHECK_MASK	(0x80)
#define RT4539_REG09_LED_UNUSED_CHECK_SHIFT	(7)
#define RT4539_REG0A_BOOST_OVP_MASK		(0x1F)
#define RT4539_REG0A_LED_SHORT_PROTECT_MASK	(0x80)
#define RT4539_REG0A_LED_SHORT_PROTECT_SHIFT	(7)
#define RT4539_REG0B_BL_EN_MASK			(0x80)
#define RT4539_REG0B_BL_EN_SHIFT		(7)
#define RT4539_REG0B_LED_EN_MASK		(0x7E)

struct rt4539 {
	struct i2c_client *client;
	struct backlight_device *bl;
	struct device *dev;
	struct rt4539_platform_data *pdata;

	/*
	 * There will exist three scenarios for the reg_en regulator.
	 *
	 * Case 1. No "enable-supply" in the device tree, reg_en will be set to NULL
	 * in rt4539_parse_dt. The regulator is configured with regulator-always-on
	 * and regulator-boot-on.
	 *
	 * Case 2. "enable-supply" is provided. The regulator is configured with
	 * regulator-always-on and regulator-boot-on.
	 *
	 * Case 3. "enable-supply" is provided, The regulator is configured with
	 * regulator-boot-on.
	 *
	 * regulator-boot-on is required to enable continuous splash: b/202947589.
	 */
	struct regulator *reg_en;	/* regulator for EN pin */

	/*
	 * true: rt4539 is forced to be blank when system enter suspend mode.
	 * reg_en case 1: brightness is set to 0 to blank the backlight.
	 * reg_en case 2: brightness is set to 0 to blank the backlight.
	 * reg_en case 3: the enable regulator is turned off to save power consumption
	 * and blank the backlight.
	 *
	 * false: rt4539 is not forced to be blank. The enable regulator is on and rt4539
	 * is ready for further operations.
	 */
	bool is_forced_blank;	/* Whether rt4539 is force to be blank or not */
};

static inline int rt4539_write_byte(struct rt4539 *rt, u8 reg, u8 data)
{
	int ret = i2c_smbus_write_byte_data(rt->client, reg, data);

	if (ret < 0)
		dev_err(rt->dev, "failed to write 0x%02x: 0x%02x, ret:%d\n", reg, data, ret);

	return ret;
}

static int rt4539_update_field(struct rt4539 *rt, u8 reg, u8 mask, u8 data)
{
	u8 tmp;
	int ret = i2c_smbus_read_byte_data(rt->client, reg);

	if (ret < 0) {
		dev_err(rt->dev, "failed to read 0x%02x, ret:%d\n", reg, ret);
		return ret;
	}

	tmp = (u8)ret;
	tmp &= ~mask;
	tmp |= data & mask;

	return rt4539_write_byte(rt, reg, tmp);
}

static inline int rt4539_set_brightness(struct rt4539 *rt, u32 brightness)
{
	u8 resolution = rt->pdata->bit_selection + BIT_SELECTION_MIN_BITS;
	u16 val = brightness & (BIT(resolution) - 1);
	int ret = 0;

	if (rt->pdata->bit_selection) {
		u8 msb = val >> 8;

		ret = rt4539_update_field(rt, RT4539_REG04, RT4539_REG04_BRIGHTNESS_MSB_MASK, msb);
	}
	return ret ? ret : rt4539_write_byte(rt, RT4539_REG05, (u8) (val & 0x00FF));
}

static int rt4539_configure(struct rt4539 *rt, u32 brightness)
{
	int ret;
	u8 mask, data;

	/* set dimming mode */
	ret = rt4539_update_field(rt, RT4539_REG00,
		RT4539_REG00_DIMMING_MODE_MASK, rt->pdata->dimming_mode);
	if (ret < 0)
		return ret;

	/* set boost switching frequency */
	ret = rt4539_update_field(rt, RT4539_REG01,
		RT4539_REG01_BOOST_SWITCH_FREQ_MASK, rt->pdata->boost_switch_freq);
	if (ret < 0)
		return ret;

	/* set max current */
	ret = i2c_smbus_write_byte_data(rt->client, RT4539_REG02,
		rt->pdata->current_max);
	if (ret < 0)
		return ret;

	/* set mapping mode and bit selection */
	mask = RT4539_REG03_ILED_MAPPING_MASK | RT4539_REG03_LED_DRIVER_HEADROOM_MASK |
			RT4539_REG03_BIT_SELECTION_MASK;
	data = (rt->pdata->exponential_mapping
			? (1 << RT4539_REG03_ILED_MAPPING_SHIFT) : 0) &
			RT4539_REG03_ILED_MAPPING_MASK;
	data |= (rt->pdata->led_headroom << RT4539_REG03_LED_DRIVER_HEADROOM_SHIFT) &
		RT4539_REG03_LED_DRIVER_HEADROOM_MASK;
	data |= rt->pdata->bit_selection & RT4539_REG03_BIT_SELECTION_MASK;
	ret = rt4539_update_field(rt, RT4539_REG03, mask, data);
	if (ret < 0)
		return ret;

	/* set brightness */
	ret = rt4539_set_brightness(rt, brightness);
	if (ret < 0)
		return ret;

	/* Set fade in/out time control, slope time control and filter */
	data = rt->pdata->fade_in_out_time_control & RT4539_REG06_FADE_IN_OUT_TIME_CTRL_MASK;
	data |= (rt->pdata->slope_time_control << RT4539_REG06_SLOPE_TIME_CTRL_SHIFT) &
		RT4539_REG06_SLOPE_TIME_CTRL_MASK;
	data |= (rt->pdata->slope_time_filter << RT4539_REG06_SLOPE_TIME_FILTER_SHIFT) &
		RT4539_REG06_SLOPE_TIME_FILTER_MASK;
	ret = rt4539_write_byte(rt, RT4539_REG06, data);
	if (ret < 0)
		return ret;

	/* set advanced brightness control */
	ret = rt4539_update_field(rt, RT4539_REG07,
		RT4539_REG07_ADV_BRIGHT_CTRL_MASK,
		rt->pdata->brightness_control);
	if (ret < 0)
		return ret;

	/* set PFM enable and LED unused check */
	mask = RT4539_REG09_PFM_ENABLE_MASK | RT4539_REG09_LED_UNUSED_CHECK_MASK;
	data = rt->pdata->pfm_enable & RT4539_REG09_PFM_ENABLE_MASK;
	data |= (rt->pdata->led_unused_check ?
		(1 << RT4539_REG09_LED_UNUSED_CHECK_SHIFT) : 0) &
		RT4539_REG09_LED_UNUSED_CHECK_MASK;
	ret = rt4539_update_field(rt, RT4539_REG09, mask, data);
	if (ret < 0)
		return ret;

	/* set boot OVP and LED short protect */
	mask = RT4539_REG0A_BOOST_OVP_MASK | RT4539_REG0A_LED_SHORT_PROTECT_MASK;
	data = rt->pdata->boost_ovp_selection & RT4539_REG0A_BOOST_OVP_MASK;
	data |= (rt->pdata->led_short_protection ?
		(1 << RT4539_REG0A_LED_SHORT_PROTECT_SHIFT) : 0) &
		RT4539_REG0A_LED_SHORT_PROTECT_MASK;
	ret = rt4539_update_field(rt, RT4539_REG0A, mask, data);
	if (ret < 0)
		return ret;

	/* set LED enable bits and enable LED outputs */
	mask = RT4539_REG0B_LED_EN_MASK | RT4539_REG0B_BL_EN_MASK;
	data = rt->pdata->enabled_leds & RT4539_REG0B_LED_EN_MASK;
	data |= (1 << RT4539_REG0B_BL_EN_SHIFT) & RT4539_REG0B_BL_EN_MASK;
	ret = rt4539_update_field(rt, RT4539_REG0B, mask, data);

	return ret;
}

/**
 * rt4539_enable - turn on the enable regulator and configure rt4539 if needed
 * @rt: the rt4539 structure.
 * @brightness: brightness value to be set.
 * @needs_configure: true if it is necessary to configure the rt4539, false otherwise
 */
static int rt4539_enable(struct rt4539 *rt, u32 brightness, bool needs_configure)
{
	int ret = 0;
	bool en_already_on = !rt->reg_en || regulator_is_enabled(rt->reg_en);

	if (rt->reg_en) {
		/*
		 * For reg_en case 3, although the enable regulator may already be turned on
		 * in bootloader. It is still necessary to trigger regulator_enable. Otherwise,
		 * the enable regulator will be turned off in regulator_late_cleanup due to
		 * use_count equal to 0.
		 *
		 * Because it is hard to distinguish between reg_en case 2 and case 3, so also
		 * trigger regulator_enable for reg_en case 2.
		 */
		ret = regulator_enable(rt->reg_en);
		if (ret < 0) {
			dev_err(rt->dev, "failed to turn on the enable regulator, ret: %d\n", ret);
			return ret;
		}

		/* When the enable regulator is just turned on, wait until rt4539 is ready. */
		if (en_already_on == false)
			usleep_range(1000, 2000);
	}

	/*
	 * is_forced_blank is set to false means the enable regulator is turned on and
	 * rt4539 is ready for further operations.
	 */
	rt->is_forced_blank = false;

	if (!en_already_on || needs_configure) {
		ret = rt4539_configure(rt, brightness);

		if (ret < 0)
			dev_err(rt->dev, "failed to configure. err: %d\n", ret);
	} else {
		/*
		 * If the enable regulator is already turned on and rt4539 has been configured
		 * at least once. Only need to set brightness here.
		 */
		ret = rt4539_set_brightness(rt, brightness);
		if (ret < 0)
			dev_err(rt->dev, "failed to set brightness. err: %d\n", ret);
	}

	return ret;
}

/**
 * rt4539_disable - turn off the enable regulator or force brightness to 0
 * @rt: the rt4539 structure
 */
static int rt4539_disable(struct rt4539 *rt)
{
	int ret = 0;

	if (rt->reg_en) {
		/* Turn off the enable regulator */
		ret = regulator_disable(rt->reg_en);

		if (ret < 0)
			dev_err(rt->dev, "error in regulator_disable: %d\n", ret);

		/*
		 * If the regulator is turned off, rt4539 is forced to be blank successfully.
		 * Return here to prevent further operations such as set_brightness.
		 */
		if (!regulator_is_enabled(rt->reg_en)) {
			rt->is_forced_blank = true;
			return 0;
		}
	}

	/*
	 * If regulator is not specified or still on, set the brightness
	 * to 0 instead.
	 */
	ret = rt4539_set_brightness(rt, 0);
	if (ret < 0)
		dev_err(rt->dev, "failed to set brightness. err: %d\n", ret);
	else
		rt->is_forced_blank = true;	/* Brightness is set to 0 successfully. */

	return ret;
}

static int rt4539_bl_update_status(struct backlight_device *bl)
{
	struct rt4539 *rt = bl_get_data(bl);
	int ret = 0, brightness = bl->props.brightness;
	bool is_blank = backlight_is_blank(bl);

	if (is_blank != rt->is_forced_blank) {
		/*
		 * If the request state is_blank is different from current state
		 * rt->is_forced_blank, then apply corresponding operation.
		 */
		if (is_blank)
			return rt4539_disable(rt);
		else
			return rt4539_enable(rt, brightness, false);
	}

	if (is_blank == false) {
		ret = rt4539_set_brightness(rt, brightness);
		if (ret < 0)
			dev_err(rt->dev, "failed to set brightness. err: %d\n", ret);
	}
	/*
	 * For the else case (is_blank is true), rt4539 is already turned off
	 * or set brightness to 0. We should not set the brightness again. Otherwise,
	 * we may encounter I2C read write error (reg_en case 3).
	 */

	return ret;
}

static const struct backlight_ops rt4539_bl_ops = {
	.options = BL_CORE_SUSPENDRESUME,
	.update_status = rt4539_bl_update_status,
};

static int rt4539_backlight_register(struct rt4539 *rt)
{
	struct backlight_device *bl;
	struct backlight_properties props;
	struct rt4539_platform_data *pdata = rt->pdata;
	const char *name = pdata->name ? : DEFAULT_BL_NAME;
	u8 resolution = rt->pdata->bit_selection + BIT_SELECTION_MIN_BITS;

	memset(&props, 0, sizeof(props));
	props.type = BACKLIGHT_PLATFORM;
	props.max_brightness = BIT(resolution) - 1;

	if (pdata->initial_brightness > props.max_brightness)
		pdata->initial_brightness = props.max_brightness;

	props.brightness = pdata->initial_brightness;

	bl = devm_backlight_device_register(rt->dev, name, rt->dev, rt,
				       &rt4539_bl_ops, &props);
	if (IS_ERR(bl))
		return -EPROBE_DEFER;

	rt->bl = bl;

	return 0;
}

#ifdef CONFIG_OF
static int rt4539_parse_dt(struct rt4539 *rt)
{
	struct device *dev = rt->dev;
	struct device_node *node = dev->of_node;
	struct rt4539_platform_data *pdata;
	u8 resolution = BIT_SELECTION_MIN_BITS;
	int ret;

	if (!node) {
		dev_err(dev, "no platform data\n");
		return -EINVAL;
	}

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	of_property_read_string(node, "bl-name", &pdata->name);

	of_property_read_u8(node, "led-headroom", &pdata->led_headroom);
	of_property_read_u8(node, "bit-selection", &resolution);
	if (resolution > BIT_SELECTION_MAX_BITS)
		resolution = BIT_SELECTION_MAX_BITS;
	else if (resolution < BIT_SELECTION_MIN_BITS)
		resolution = BIT_SELECTION_MIN_BITS;
	pdata->bit_selection = resolution - BIT_SELECTION_MIN_BITS;
	dev_info(dev, "%u bits brightness resolution\n", resolution);

	of_property_read_u8(node, "dimming-mode", &pdata->dimming_mode);
	of_property_read_u8(node, "fade-in-out-time-ctrl",
		&pdata->fade_in_out_time_control);
	of_property_read_u8(node, "slope-time-ctrl", &pdata->slope_time_control);
	of_property_read_u8(node, "slope-time-filter", &pdata->slope_time_filter);
	of_property_read_u8(node, "boost-switch-freq", &pdata->boost_switch_freq);
	of_property_read_u8(node, "current-max", &pdata->current_max);
	of_property_read_u8(node, "brightness-control",
		&pdata->brightness_control);
	of_property_read_u8(node, "enabled-leds", &pdata->enabled_leds);
	of_property_read_u16(node, "initial-brightness",
		&pdata->initial_brightness);
	of_property_read_u8(node, "boost-ovp-selection",
		&pdata->boost_ovp_selection);
	pdata->led_short_protection
		= of_property_read_bool(node, "led-short-protection");
	pdata->exponential_mapping
		= of_property_read_bool(node, "exponential-mapping");
	pdata->led_unused_check
		= of_property_read_bool(node, "led-unused-check");
	pdata->pfm_enable
		= of_property_read_bool(node, "pfm-enable");

	rt->reg_en = devm_regulator_get_optional(dev, "enable");
	if (IS_ERR(rt->reg_en)) {
		ret = PTR_ERR(rt->reg_en);
		if (ret == -ENODEV)
			rt->reg_en = NULL;
		else
			return dev_err_probe(dev, ret, "getting enable regulator\n");
	}

	rt->pdata = pdata;

	return 0;
}
#else
static int rt4539_parse_dt(struct rt4539 *rt)
{
	return -EINVAL;
}
#endif

static int rt4539_probe(struct i2c_client *cl, const struct i2c_device_id *id)
{
	struct rt4539 *rt;
	int ret;

	if (!i2c_check_functionality(cl->adapter, I2C_FUNC_SMBUS_I2C_BLOCK))
		return -EIO;

	rt = devm_kzalloc(&cl->dev, sizeof(struct rt4539), GFP_KERNEL);
	if (!rt)
		return -ENOMEM;

	rt->client = cl;
	rt->dev = &cl->dev;
	rt->pdata = dev_get_platdata(&cl->dev);

	if (!rt->pdata) {
		ret = rt4539_parse_dt(rt);
		if (ret < 0) {
			dev_err(rt->dev, "failed to parse dt. err: %d\n", ret);
			return -EPROBE_DEFER;
		}
	}

	i2c_set_clientdata(cl, rt);
	rt->is_forced_blank = rt->reg_en && !regulator_is_enabled(rt->reg_en);

	ret = rt4539_enable(rt, rt->pdata->initial_brightness, true);
	if (ret < 0) {
		dev_err(rt->dev, "failed to enable. err: %d\n", ret);
		return ret;
	}

	ret = rt4539_backlight_register(rt);
	if (ret) {
		dev_err(rt->dev,
			"failed to register backlight. err: %d\n", ret);
		return -EPROBE_DEFER;
	}

	return 0;
}

static void rt4539_remove(struct i2c_client *cl)
{
	struct rt4539 *rt = i2c_get_clientdata(cl);

	rt->bl->props.brightness = 0;
	backlight_update_status(rt->bl);
	if (rt->is_forced_blank == false)
		rt4539_disable(rt);
}

static const struct of_device_id rt4539_dt_ids[] = {
	{ .compatible = "richtek,rt4539", },
	{ }
};
MODULE_DEVICE_TABLE(of, rt4539_dt_ids);

static const struct i2c_device_id rt4539_ids[] = {
	{"rt4539", RT4539},
	{ }
};
MODULE_DEVICE_TABLE(i2c, rt4539_ids);

static struct i2c_driver rt4539_driver = {
	.driver = {
		   .name = "rt4539",
		   .of_match_table = of_match_ptr(rt4539_dt_ids),
		   },
	.probe = rt4539_probe,
	.remove = rt4539_remove,
	.id_table = rt4539_ids,
};

module_i2c_driver(rt4539_driver);

MODULE_DESCRIPTION("Richtek RT4539 Backlight driver");
MODULE_AUTHOR("Ting Yan <tingyan@google.com>");
MODULE_LICENSE("GPL v2");
