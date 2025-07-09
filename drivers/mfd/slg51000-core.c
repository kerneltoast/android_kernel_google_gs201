// SPDX-License-Identifier: GPL-2.0
/*
 * SLG51000 core driver
 *
 * Copyright 2020 Google LLC
 *
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/mfd/core.h>
#include <linux/mfd/slg51000.h>
#include <linux/of_gpio.h>
#include <linux/regmap.h>

#define SLG51000_CHIP_ID_LEN            3
#define TIMER_EXPIRED_SEC		(1 * HZ)

static const struct mfd_cell slg51000_devs[] = {
	{
		.name = "slg51000-regulator",
	},
	{
		.name = "slg51000_gpio",
	},
};

static const struct regmap_range slg51000_writeable_ranges[] = {
	regmap_reg_range(SLG51000_SYSCTL_MATRIX_CONF_A,
			SLG51000_SYSCTL_MATRIX_CONF_A),
	regmap_reg_range(SLG51000_LDO_HP_STARTUP_ILIM,
			SLG51000_LDO_HP_STARTUP_ILIM),
	regmap_reg_range(SLG51000_LDO1_VSEL, SLG51000_LDO1_VSEL),
	regmap_reg_range(SLG51000_LDO1_MINV, SLG51000_LDO1_MAXV),
	regmap_reg_range(SLG51000_LDO1_IRQ_MASK, SLG51000_LDO1_IRQ_MASK),
	regmap_reg_range(SLG51000_LDO2_VSEL, SLG51000_LDO2_VSEL),
	regmap_reg_range(SLG51000_LDO2_MINV, SLG51000_LDO2_MAXV),
	regmap_reg_range(SLG51000_LDO2_IRQ_MASK, SLG51000_LDO2_IRQ_MASK),
	regmap_reg_range(SLG51000_LDO3_VSEL, SLG51000_LDO3_VSEL),
	regmap_reg_range(SLG51000_LDO3_MINV, SLG51000_LDO3_MAXV),
	regmap_reg_range(SLG51000_LDO3_CONF1, SLG51000_LDO3_CONF1),
	regmap_reg_range(SLG51000_LDO3_IRQ_MASK, SLG51000_LDO3_IRQ_MASK),
	regmap_reg_range(SLG51000_LDO4_VSEL, SLG51000_LDO4_VSEL),
	regmap_reg_range(SLG51000_LDO4_MINV, SLG51000_LDO4_MAXV),
	regmap_reg_range(SLG51000_LDO4_IRQ_MASK, SLG51000_LDO4_IRQ_MASK),
	regmap_reg_range(SLG51000_LDO5_VSEL, SLG51000_LDO5_VSEL),
	regmap_reg_range(SLG51000_LDO5_MINV, SLG51000_LDO5_MAXV),
	regmap_reg_range(SLG51000_LDO5_IRQ_MASK, SLG51000_LDO5_IRQ_MASK),
	regmap_reg_range(SLG51000_LDO6_VSEL, SLG51000_LDO6_VSEL),
	regmap_reg_range(SLG51000_LDO6_MINV, SLG51000_LDO6_MAXV),
	regmap_reg_range(SLG51000_LDO6_IRQ_MASK, SLG51000_LDO6_IRQ_MASK),
	regmap_reg_range(SLG51000_LDO7_VSEL, SLG51000_LDO7_VSEL),
	regmap_reg_range(SLG51000_LDO7_MINV, SLG51000_LDO7_MAXV),
	regmap_reg_range(SLG51000_LDO7_IRQ_MASK, SLG51000_LDO7_IRQ_MASK),
	regmap_reg_range(SLG51000_OTP_IRQ_MASK, SLG51000_OTP_IRQ_MASK),
	regmap_reg_range(SLG51000_SW_TEST_MODE_1, SLG51000_SW_TEST_MODE_4),
	regmap_reg_range(SLG51000_MUXARRAY_INPUT_SEL_39,
			SLG51000_MUXARRAY_INPUT_SEL_39),
	regmap_reg_range(SLG51000_LUTARRAY_LUT_VAL_3,
			SLG51000_LUTARRAY_LUT_VAL_3),
	/* For GPIO and sequence control */
	regmap_reg_range(0x1101, 0x800F),
};

static const struct regmap_range slg51000_readable_ranges[] = {
	regmap_reg_range(SLG51000_SYSCTL_PATN_ID_B0,
			SLG51000_SYSCTL_PATN_ID_B2),
	regmap_reg_range(SLG51000_SYSCTL_SYS_CONF_A,
			SLG51000_SYSCTL_SYS_CONF_A),
	regmap_reg_range(SLG51000_SYSCTL_SYS_CONF_D,
			SLG51000_SYSCTL_MATRIX_CONF_B),
	regmap_reg_range(SLG51000_SYSCTL_REFGEN_CONF_C,
			SLG51000_SYSCTL_UVLO_CONF_A),
	regmap_reg_range(SLG51000_SYSCTL_FAULT_LOG1, SLG51000_SYSCTL_IRQ_MASK),
	regmap_reg_range(SLG51000_LDO_HP_STARTUP_ILIM,
			SLG51000_LDO_HP_STARTUP_ILIM),
	regmap_reg_range(SLG51000_IO_GPIO1_CONF, SLG51000_IO_GPIO_STATUS),
	regmap_reg_range(SLG51000_LUTARRAY_LUT_VAL_0,
			SLG51000_LUTARRAY_LUT_VAL_11),
	regmap_reg_range(SLG51000_MUXARRAY_INPUT_SEL_0,
			SLG51000_MUXARRAY_INPUT_SEL_63),
	regmap_reg_range(SLG51000_PWRSEQ_RESOURCE_EN_0,
			SLG51000_PWRSEQ_INPUT_SENSE_CONF_B),
	regmap_reg_range(SLG51000_LDO1_VSEL, SLG51000_LDO1_VSEL),
	regmap_reg_range(SLG51000_LDO1_MINV, SLG51000_LDO1_MAXV),
	regmap_reg_range(SLG51000_LDO1_TRIM2, SLG51000_LDO1_VSEL_ACTUAL),
	regmap_reg_range(SLG51000_LDO1_EVENT, SLG51000_LDO1_IRQ_MASK),
	regmap_reg_range(SLG51000_LDO2_VSEL, SLG51000_LDO2_VSEL),
	regmap_reg_range(SLG51000_LDO2_MINV, SLG51000_LDO2_MAXV),
	regmap_reg_range(SLG51000_LDO2_TRIM2, SLG51000_LDO2_VSEL_ACTUAL),
	regmap_reg_range(SLG51000_LDO2_EVENT, SLG51000_LDO2_IRQ_MASK),
	regmap_reg_range(SLG51000_LDO3_VSEL, SLG51000_LDO3_VSEL),
	regmap_reg_range(SLG51000_LDO3_MINV, SLG51000_LDO3_MAXV),
	regmap_reg_range(SLG51000_LDO3_TRIM2, SLG51000_LDO3_VSEL_ACTUAL),
	regmap_reg_range(SLG51000_LDO3_EVENT, SLG51000_LDO3_IRQ_MASK),
	regmap_reg_range(SLG51000_LDO4_VSEL, SLG51000_LDO4_VSEL),
	regmap_reg_range(SLG51000_LDO4_MINV, SLG51000_LDO4_MAXV),
	regmap_reg_range(SLG51000_LDO4_TRIM2, SLG51000_LDO4_VSEL_ACTUAL),
	regmap_reg_range(SLG51000_LDO4_EVENT, SLG51000_LDO4_IRQ_MASK),
	regmap_reg_range(SLG51000_LDO5_VSEL, SLG51000_LDO5_VSEL),
	regmap_reg_range(SLG51000_LDO5_MINV, SLG51000_LDO5_MAXV),
	regmap_reg_range(SLG51000_LDO5_TRIM2, SLG51000_LDO5_VSEL_ACTUAL),
	regmap_reg_range(SLG51000_LDO5_EVENT, SLG51000_LDO5_IRQ_MASK),
	regmap_reg_range(SLG51000_LDO6_VSEL, SLG51000_LDO6_VSEL),
	regmap_reg_range(SLG51000_LDO6_MINV, SLG51000_LDO6_MAXV),
	regmap_reg_range(SLG51000_LDO6_TRIM2, SLG51000_LDO6_VSEL_ACTUAL),
	regmap_reg_range(SLG51000_LDO6_EVENT, SLG51000_LDO6_IRQ_MASK),
	regmap_reg_range(SLG51000_LDO7_VSEL, SLG51000_LDO7_VSEL),
	regmap_reg_range(SLG51000_LDO7_MINV, SLG51000_LDO7_MAXV),
	regmap_reg_range(SLG51000_LDO7_TRIM2, SLG51000_LDO7_VSEL_ACTUAL),
	regmap_reg_range(SLG51000_LDO7_EVENT, SLG51000_LDO7_IRQ_MASK),
	regmap_reg_range(SLG51000_OTP_EVENT, SLG51000_OTP_EVENT),
	regmap_reg_range(SLG51000_OTP_IRQ_MASK, SLG51000_OTP_IRQ_MASK),
	regmap_reg_range(SLG51000_LOCK_GLOBAL_LOCK_CTRL1,
			SLG51000_LOCK_GLOBAL_LOCK_CTRL1),
	regmap_reg_range(SLG51000_SYSCTL_TEST_EN, SLG51000_SYSCTL_TEST_EN),
};

static const struct regmap_range slg51000_volatile_ranges[] = {
	regmap_reg_range(SLG51000_SYSCTL_FAULT_LOG1, SLG51000_SYSCTL_STATUS),
	regmap_reg_range(SLG51000_IO_GPIO_STATUS, SLG51000_IO_GPIO_STATUS),
	regmap_reg_range(SLG51000_LDO1_EVENT, SLG51000_LDO1_STATUS),
	regmap_reg_range(SLG51000_LDO2_EVENT, SLG51000_LDO2_STATUS),
	regmap_reg_range(SLG51000_LDO3_EVENT, SLG51000_LDO3_STATUS),
	regmap_reg_range(SLG51000_LDO4_EVENT, SLG51000_LDO4_STATUS),
	regmap_reg_range(SLG51000_LDO5_EVENT, SLG51000_LDO5_STATUS),
	regmap_reg_range(SLG51000_LDO6_EVENT, SLG51000_LDO6_STATUS),
	regmap_reg_range(SLG51000_LDO7_EVENT, SLG51000_LDO7_STATUS),
	regmap_reg_range(SLG51000_OTP_EVENT, SLG51000_OTP_EVENT),
};

static const struct regmap_access_table slg51000_writeable_table = {
	.yes_ranges	= slg51000_writeable_ranges,
	.n_yes_ranges	= ARRAY_SIZE(slg51000_writeable_ranges),
};

static const struct regmap_access_table slg51000_readable_table = {
	.yes_ranges	= slg51000_readable_ranges,
	.n_yes_ranges	= ARRAY_SIZE(slg51000_readable_ranges),
};

static const struct regmap_access_table slg51000_volatile_table = {
	.yes_ranges	= slg51000_volatile_ranges,
	.n_yes_ranges	= ARRAY_SIZE(slg51000_volatile_ranges),
};

static int slg51000_init_regs(struct slg51000_dev *chip)
{
	int ret;
	size_t i;
	int offset;
	u32 tmp;
	int num_output;
	u32 num_cells;
	struct slg51000_register_setting *regs;

	ret = of_property_read_u32(chip->dev->of_node,
			"dlg,reg-init-cells", &num_cells);
	if (ret)
		return -EINVAL;

	if (num_cells != 2)
		return -EINVAL;

	if (!of_get_property(chip->dev->of_node, "dlg,reg-init", &tmp))
		return -EINVAL;

	num_output = tmp / (sizeof(u32) * num_cells);

	regs = kcalloc(num_output, sizeof(struct slg51000_register_setting),
			GFP_KERNEL);
	if (!regs) {
		ret = -ENOMEM;
		goto out;
	}

	/* Apply the settings */
	for (i = 0; i < num_output; i++) {
		offset = i * num_cells;
		if (of_property_read_u32_index(chip->dev->of_node,
				"dlg,reg-init", offset, &tmp))
			goto out;
		regs[i].addr = tmp;

		if (of_property_read_u32_index(chip->dev->of_node,
				"dlg,reg-init", offset + 1, &tmp))
			goto out;
		regs[i].val = tmp;

		ret = regmap_write(chip->i2c_regmap, regs[i].addr, regs[i].val);
		if (ret < 0) {
			dev_err(chip->dev, "Failed to set addr 0x%02x\n",
					regs[i].addr);
			goto out;
		}
	}

out:
	kfree(regs);
	return ret;
}

static int slg51000_enter_sw_test_mode(struct regmap *map)
{
	unsigned int val = 0;
	int ret;
	const u8 sw_test_mode_on_vals[] = {
		SLG51000_SW_TEST_MODE_1_ON,
		SLG51000_SW_TEST_MODE_2_ON,
		SLG51000_SW_TEST_MODE_3_ON,
		SLG51000_SW_TEST_MODE_4_ON,
	};

	if (!map)
		return -EINVAL;

	ret = regmap_bulk_write(map, SLG51000_SW_TEST_MODE_1,
			sw_test_mode_on_vals, ARRAY_SIZE(sw_test_mode_on_vals));
	if (ret < 0) {
		dev_err(regmap_get_device(map),
				"Failed to write regs for sw test mode\n");
		return ret;
	}

	ret = regmap_read(map, SLG51000_SYSCTL_TEST_EN, &val);
	if (ret < 0) {
		dev_err(regmap_get_device(map),
				"Failed to read SLG51000_SYSCTL_TEST_EN\n");
		return ret;
	}

	/* Check if software test mode already enabled */
	if (val & SLG51000_TEST_EN_ON_MASK)
		return 0;

	return ret;
}

static int slg51000_exit_sw_test_mode(struct regmap *map)
{
	if (!map)
		return -EINVAL;

	return regmap_write(map, SLG51000_SYSCTL_TEST_EN,
			SLG51000_TEST_EN_OFF);
}

static int slg51000_config_tuning(struct slg51000_dev *chip)
{
	int ret;

	if (chip == NULL) {
		pr_err("[%s] Invalid arguments\n", __func__);
		return -EINVAL;
	}

	ret = slg51000_enter_sw_test_mode(chip->i2c_regmap);
	if (ret < 0)
		return ret;

	/* Initialize register settings */
	ret = slg51000_init_regs(chip);
	if (ret < 0)
		dev_info(chip->dev, "No init registers are overridden\n");

	ret = slg51000_exit_sw_test_mode(chip->i2c_regmap);
	if (ret < 0)
		return ret;

	return ret;
}

static void slg51000_clear_fault_log(struct slg51000_dev *chip)
{
	unsigned int val = 0;
	int ret = 0;

	ret = regmap_read(chip->regmap, SLG51000_SYSCTL_FAULT_LOG1, &val);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to read Fault log register\n");
		return;
	}

	if (val & SLG51000_FLT_OVER_TEMP_MASK)
		dev_dbg(chip->dev, "Fault log: FLT_OVER_TEMP\n");
	if (val & SLG51000_FLT_POWER_SEQ_CRASH_REQ_MASK)
		dev_dbg(chip->dev, "Fault log: FLT_POWER_SEQ_CRASH_REQ\n");
	if (val & SLG51000_FLT_RST_MASK)
		dev_dbg(chip->dev, "Fault log: FLT_RST\n");
	if (val & SLG51000_FLT_POR_MASK)
		dev_dbg(chip->dev, "Fault log: FLT_POR\n");
}

static int slg51000_power_on(struct slg51000_dev *chip)
{
	if (chip == NULL)
		return -EINVAL;

	mutex_lock(&chip->pwr_lock);
	if (chip->is_power_on)
		goto out;

	if (gpio_is_valid(chip->chip_bb_pin)) {
		gpio_set_value_cansleep(chip->chip_bb_pin, 1);
		usleep_range(2000, 2020);
	}

	if (gpio_is_valid(chip->chip_buck_pin)) {
		gpio_set_value_cansleep(chip->chip_buck_pin, 1);
		usleep_range(2000, 2020);
	}

	if (gpio_is_valid(chip->chip_cs_pin)) {
		gpio_set_value_cansleep(chip->chip_cs_pin, 1);

		/*
		* According to datasheet, turn-on time from CS HIGH to Ready
		* state is ~10ms
		*/
		usleep_range(SLEEP_10000_USEC,
				SLEEP_10000_USEC + SLEEP_RANGE_USEC);
	}

	chip->is_power_on = true;
	dev_dbg(chip->dev, "power on\n");

	slg51000_config_tuning(chip);

	if (gpio_is_valid(chip->chip_pu_pin)) {
		gpio_set_value_cansleep(chip->chip_pu_pin, 1);
		usleep_range(1000, 1020);
	}

out:
	mutex_unlock(&chip->pwr_lock);
	return 0;
}

static int slg51000_power_off(struct slg51000_dev *chip)
{
	unsigned int val = 0;
	int ret, idx;
	uint8_t gpio_vals[SLG51000_PHYSICAL_GPIO_NR];

	if (chip == NULL)
		return -EINVAL;

	mutex_lock(&chip->pwr_lock);
	if (!chip->is_power_on || chip->chip_always_on) {
		ret = 0;
		goto out;
	}

	/* Check control register */
	ret = regmap_read(chip->i2c_regmap, SLG51000_SYSCTL_MATRIX_CONF_A, &val);
	if (ret < 0)
		goto out;
	if (val) {
		ret = 0;
		goto out;
	}

	/*
	 * Need to check GPIO control registers for mode
	 * SLG51000_OP_MODE_LDO_GPIO
	 */
	if (chip->op_mode == SLG51000_OP_MODE_LDO_GPIO) {
		ret = regmap_bulk_read(chip->i2c_regmap,
				GPIO1_CTRL, gpio_vals, ARRAY_SIZE(gpio_vals));
		if (ret < 0)
			goto out;

		for (idx = 0; idx < ARRAY_SIZE(gpio_vals); idx++) {
			if (gpio_vals[idx] != 0) {
				ret = 0;
				goto out;
			}
		}
	}

	/* power off */
	if (gpio_is_valid(chip->chip_pu_pin)) {
		gpio_set_value_cansleep(chip->chip_pu_pin, 0);
		usleep_range(1000, 1020);
	}

	if (gpio_is_valid(chip->chip_cs_pin)) {
		gpio_set_value_cansleep(chip->chip_cs_pin, 0);
		/* Put SLG51000 back to Reset state */
		usleep_range(SLEEP_10000_USEC,
				SLEEP_10000_USEC + SLEEP_RANGE_USEC);
	}

	if (gpio_is_valid(chip->chip_buck_pin)) {
		gpio_set_value_cansleep(chip->chip_buck_pin, 0);
		usleep_range(1000, 1020);
	}

	if (gpio_is_valid(chip->chip_bb_pin)) {
		gpio_set_value_cansleep(chip->chip_bb_pin, 0);
		usleep_range(1000, 1020);
	}


	chip->is_power_on = false;
	dev_dbg(chip->dev, "power off\n");

out:
	mutex_unlock(&chip->pwr_lock);
	return ret;
}

static void slg51000_timeout_work(struct work_struct *work)
{
	struct slg51000_dev *slg51000 =
			container_of(work, struct slg51000_dev, timeout_work);
	slg51000_power_off(slg51000);
}

static void slg51000_timer_trigger(struct timer_list *t)
{
	struct slg51000_dev *slg51000 = from_timer(slg51000, t, timer);
	schedule_work(&slg51000->timeout_work);
}

static int slg51000_reg_read(void *context, unsigned int reg, unsigned int *val)
{
	int ret;
	struct i2c_client *client = context;
	struct slg51000_dev *slg51000 = i2c_get_clientdata(client);

	slg51000_power_on(slg51000);
	ret = regmap_read(slg51000->i2c_regmap, reg, val);
	mod_timer(&slg51000->timer, jiffies + TIMER_EXPIRED_SEC);
	return ret;
}

static int slg51000_reg_write(void *context, unsigned int reg, unsigned int val)
{
	int ret;
	struct i2c_client *client = context;
	struct slg51000_dev *slg51000 = i2c_get_clientdata(client);

	slg51000_power_on(slg51000);
	ret = regmap_write(slg51000->i2c_regmap, reg, val);
	mod_timer(&slg51000->timer, jiffies + TIMER_EXPIRED_SEC);
	return ret;
}

static int read_chip_id(struct slg51000_dev *chip)
{
	int ret;
	uint8_t val[SLG51000_CHIP_ID_LEN];

	if (chip == NULL)
		return -EINVAL;

	ret = regmap_bulk_read(chip->regmap,
			SLG51000_SYSCTL_PATN_ID_B0, val, ARRAY_SIZE(val));
	if (ret < 0) {
		dev_err(chip->dev, "Failed to read chip id registers(%d)\n",
			ret);
		return ret;
	}

	/* Format chip id */
	chip->chip_id = ((val[2] << 16) | (val[1] << 8) | val[0]);
	dev_info(chip->dev, "chip_id: 0x%x\n", chip->chip_id);

	return ret;
}

static ssize_t chip_id_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct slg51000_dev *chip;

	chip = dev_get_drvdata(dev);
	if (chip == NULL)
		return -EINVAL;

	return scnprintf(buf, PAGE_SIZE, "0x%x\n", chip->chip_id);
}
static DEVICE_ATTR_RO(chip_id);

static ssize_t chip_always_on_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct slg51000_dev *chip;

	chip = dev_get_drvdata(dev);
	if (chip == NULL)
		return -EINVAL;

	return scnprintf(buf, PAGE_SIZE, "%d\n", chip->chip_always_on);
}

static ssize_t chip_always_on_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct slg51000_dev *chip;
	unsigned long val;
	int ret;

	chip = dev_get_drvdata(dev);

	ret = kstrtoul(buf, 0, &val);
	if (ret < 0)
		return ret;

	chip->chip_always_on = val;
	dev_info(dev, "Set %s to %d\n", attr->attr.name, chip->chip_always_on);

	if (chip->chip_always_on) {
		slg51000_power_on(chip);
	} else {
		slg51000_power_off(chip);
	}

	return count;
}
static DEVICE_ATTR_RW(chip_always_on);

static struct attribute *attrs[] = {
	&dev_attr_chip_id.attr,
	&dev_attr_chip_always_on.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = attrs,
};

static const struct regmap_config slg51000_i2c_regmap_config = {
	.name = "i2c",
	.reg_bits = 16,
	.val_bits = 8,
	.cache_type = REGCACHE_NONE,
};

static const struct regmap_config slg51000_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.max_register = 0x800F,
	.wr_table = &slg51000_writeable_table,
	.rd_table = &slg51000_readable_table,
	.volatile_table = &slg51000_volatile_table,
	.reg_read = slg51000_reg_read,
	.reg_write = slg51000_reg_write,
};

static int slg51000_i2c_probe(struct i2c_client *client,
			      const struct i2c_device_id *id)
{
	struct slg51000_dev *slg51000;
	int gpio, ret;
	struct pinctrl *pinctrl;
	struct pinctrl_state *state;

	slg51000 = devm_kzalloc(&client->dev,
			sizeof(struct slg51000_dev), GFP_KERNEL);
	if (!slg51000)
		return -ENOMEM;

	if (of_property_count_strings(client->dev.of_node,
			"pinctrl-names") <= 0) {
		dev_dbg(&client->dev, "no pinctrl defined\n");
	} else {
		pinctrl = devm_pinctrl_get(&client->dev);
		if (IS_ERR(pinctrl)) {
			dev_err(&client->dev, "Cannot allocate pinctrl\n");
			return PTR_ERR(pinctrl);
		}

		state = pinctrl_lookup_state(pinctrl, "active");
		if (IS_ERR(state)) {
			dev_err(&client->dev,
					"Cannot find pinctrl state: active\n");
			devm_pinctrl_put(pinctrl);
			return PTR_ERR(state);
		}

		ret = pinctrl_select_state(pinctrl, state);
		if (ret) {
			dev_err(&client->dev, "Cannot select state: active\n");
			return ret;
		}
	}

	/* optional property */
	gpio = of_get_named_gpio(client->dev.of_node, "dlg,bb-gpios", 0);
	if (gpio_is_valid(gpio)) {
		ret = devm_gpio_request_one(&client->dev, gpio,
				GPIOF_OUT_INIT_HIGH, "slg51000_bb_pin");
		if (ret) {
			dev_err(&client->dev, "GPIO(%d) request failed(%d)\n",
					gpio, ret);
			return ret;
		}

		dev_dbg(&client->dev, "GPIO(%d) request (%d)\n", gpio, ret);

		slg51000->chip_bb_pin = gpio;
		usleep_range(2000, 2020);
	}

	/* optional property */
	gpio = of_get_named_gpio(client->dev.of_node, "dlg,buck-gpios", 0);
	if (gpio_is_valid(gpio)) {
		ret = devm_gpio_request_one(&client->dev, gpio,
				GPIOF_OUT_INIT_HIGH, "slg51000_buck_pin");
		if (ret) {
			dev_err(&client->dev, "GPIO(%d) request failed(%d)\n",
					gpio, ret);
			return ret;
		}

		dev_dbg(&client->dev, "GPIO(%d) request (%d)\n", gpio, ret);

		slg51000->chip_buck_pin = gpio;
		usleep_range(2000, 2020);
	}

	/* mandatory property. It wakes the chip from low-power reset state */
	gpio = of_get_named_gpio(client->dev.of_node, "dlg,cs-gpios", 0);
	if (gpio_is_valid(gpio)) {
		ret = devm_gpio_request_one(&client->dev, gpio,
				GPIOF_OUT_INIT_HIGH, "slg51000_cs_pin");
		if (ret) {
			dev_err(&client->dev, "GPIO(%d) request failed(%d)\n",
					gpio, ret);
			return ret;
		}

		slg51000->chip_cs_pin = gpio;

		/*
		 * According to datasheet, turn-on time from CS HIGH to Ready
		 * state is ~10ms
		 */
		usleep_range(SLEEP_10000_USEC,
			     SLEEP_10000_USEC + SLEEP_RANGE_USEC);
	} else {
		return gpio;
	}

	i2c_set_clientdata(client, slg51000);

	slg51000->chip_always_on = false;
	slg51000->is_power_on = true;
	INIT_WORK(&slg51000->timeout_work, slg51000_timeout_work);
	timer_setup(&slg51000->timer, slg51000_timer_trigger, 0);
	mutex_init(&slg51000->pwr_lock);

	slg51000->chip_irq = client->irq;
	slg51000->dev = &client->dev;
	slg51000->chip_id = 0;

	slg51000->i2c_regmap = devm_regmap_init_i2c(client, &slg51000_i2c_regmap_config);
	if (IS_ERR(slg51000->i2c_regmap)) {
		ret = PTR_ERR(slg51000->i2c_regmap);
		dev_err(&client->dev, "Failed to allocate register map: %d\n",
			ret);
		goto out;
	}

	slg51000->regmap = devm_regmap_init(&client->dev, NULL, client, &slg51000_regmap_config);
	if (IS_ERR(slg51000->regmap)) {
		ret = PTR_ERR(slg51000->regmap);
		dev_err(&client->dev, "Failed to allocate register map: %d\n",
				ret);
		goto out;
	}

	ret = of_property_read_u32(slg51000->dev->of_node,
			"dlg,op-mode", &slg51000->op_mode);
	if (ret < 0)
		slg51000->op_mode = SLG51000_OP_MODE_LDO_ONLY;

	dev_dbg(slg51000->dev, "op_mode: %d\n", slg51000->op_mode);

	slg51000->enter_sw_test_mode = slg51000_enter_sw_test_mode;
	slg51000->exit_sw_test_mode = slg51000_exit_sw_test_mode;

	ret = slg51000_config_tuning(slg51000);
	if (ret < 0) {
		dev_info(slg51000->dev, "No config tuning(%d)\n", ret);
	}

	/* optional property */
	gpio = of_get_named_gpio(client->dev.of_node, "dlg,pu-gpios", 0);
	if (gpio_is_valid(gpio)) {
		ret = devm_gpio_request_one(&client->dev, gpio,
				GPIOF_OUT_INIT_HIGH, "slg51000_pu_pin");
		if (ret) {
			dev_err(&client->dev, "GPIO(%d) request failed(%d)\n",
					gpio, ret);
			goto out;
		}

		dev_dbg(&client->dev, "GPIO(%d) request (%d)\n", gpio, ret);

		slg51000->chip_pu_pin = gpio;
		usleep_range(1000, 1020);
	}

	slg51000_clear_fault_log(slg51000);

	ret = read_chip_id(slg51000);
	if (ret < 0)
		goto out;

	ret = sysfs_create_group(&slg51000->dev->kobj, &attr_group);
	if (ret)
		dev_err(&client->dev, "Failed to create attribute group: %d\n", ret);

	return devm_mfd_add_devices(slg51000->dev, -1, slg51000_devs,
			ARRAY_SIZE(slg51000_devs), NULL, 0, NULL);

out:
	mutex_destroy(&slg51000->pwr_lock);
	del_timer_sync(&slg51000->timer);
	return ret;
}

static void slg51000_i2c_remove(struct i2c_client *client)
{
	struct slg51000_dev *slg51000 = i2c_get_clientdata(client);
	struct gpio_desc *desc;
	int ret = 0;

	sysfs_remove_group(&slg51000->dev->kobj, &attr_group);

	mfd_remove_devices(slg51000->dev);
	mutex_destroy(&slg51000->pwr_lock);
	del_timer_sync(&slg51000->timer);

	if (gpio_is_valid(slg51000->chip_pu_pin)) {
		desc = gpio_to_desc(slg51000->chip_pu_pin);
		ret |= gpiod_direction_output_raw(desc, GPIOF_INIT_LOW);
		usleep_range(1000, 1020);
	}
	if (gpio_is_valid(slg51000->chip_cs_pin)) {
		desc = gpio_to_desc(slg51000->chip_cs_pin);
		ret |= gpiod_direction_output_raw(desc, GPIOF_INIT_LOW);
		/* Put SLG51000 back to Reset state */
		usleep_range(SLEEP_10000_USEC,
				SLEEP_10000_USEC + SLEEP_RANGE_USEC);
	}
	if (gpio_is_valid(slg51000->chip_buck_pin)) {
		desc = gpio_to_desc(slg51000->chip_buck_pin);
		ret |= gpiod_direction_output_raw(desc, GPIOF_INIT_LOW);
		usleep_range(1000, 1020);
	}
	if (gpio_is_valid(slg51000->chip_bb_pin)) {
		desc = gpio_to_desc(slg51000->chip_bb_pin);
		ret |= gpiod_direction_output_raw(desc, GPIOF_INIT_LOW);
		usleep_range(1000, 1020);
	}

	if (ret)
		pr_err("Failed to update the gpiod direction on remove\n");
}

static const struct i2c_device_id slg51000_i2c_id[] = {
	{ "slg51000", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, slg51000_i2c_id);

#if defined(CONFIG_OF)
static const struct of_device_id slg51000_of_match[] = {
	{ .compatible = "dlg,slg51000", },
	{ },
};
MODULE_DEVICE_TABLE(of, slg51000_of_match);
#endif /* CONFIG_OF */

static struct i2c_driver slg51000_i2c_driver = {
	.driver = {
		.name = "slg51000",
#if defined(CONFIG_OF)
		.of_match_table = of_match_ptr(slg51000_of_match),
#endif /* CONFIG_OF */
	},
	.probe = slg51000_i2c_probe,
	.remove = slg51000_i2c_remove,
	.id_table = slg51000_i2c_id,
};

static int __init slg51000_i2c_init(void)
{
	return i2c_add_driver(&slg51000_i2c_driver);
}
/* init early so consumer devices can complete system boot */
subsys_initcall(slg51000_i2c_init);

static void __exit slg51000_i2c_exit(void)
{
	i2c_del_driver(&slg51000_i2c_driver);
}
module_exit(slg51000_i2c_exit);

MODULE_AUTHOR("CY Tseng <cytseng@google.com>");
MODULE_DESCRIPTION("slg51000 multi-function core driver");
MODULE_LICENSE("GPL");
