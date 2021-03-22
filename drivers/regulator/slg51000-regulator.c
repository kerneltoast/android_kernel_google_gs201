// SPDX-License-Identifier: GPL-2.0+
/*
 * SLG51000 High PSRR, Multi-Output Regulators
 *
 * Copyright (C) 2019 Dialog Semiconductor
 * Author: Eric Jeong <eric.jeong.opensource@diasemi.com>
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/mfd/slg51000.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>

#define SLG51000_SCTL_EVT               7
#define SLG51000_MAX_EVT_REGISTER       8
#define SLG51000_LDOHP_LV_MIN           1200000
#define SLG51000_LDOHP_HV_MIN           2200000

struct slg51000_evt_sta {
	unsigned int ereg;
	unsigned int sreg;
};

static const struct slg51000_evt_sta es_reg[SLG51000_MAX_EVT_REGISTER] = {
	{SLG51000_LDO1_EVENT, SLG51000_LDO1_STATUS},
	{SLG51000_LDO2_EVENT, SLG51000_LDO2_STATUS},
	{SLG51000_LDO3_EVENT, SLG51000_LDO3_STATUS},
	{SLG51000_LDO4_EVENT, SLG51000_LDO4_STATUS},
	{SLG51000_LDO5_EVENT, SLG51000_LDO5_STATUS},
	{SLG51000_LDO6_EVENT, SLG51000_LDO6_STATUS},
	{SLG51000_LDO7_EVENT, SLG51000_LDO7_STATUS},
	{SLG51000_SYSCTL_EVENT, SLG51000_SYSCTL_STATUS},
};

static int slg51000_get_status(struct regulator_dev *rdev)
{
	struct slg51000_dev *chip = rdev_get_drvdata(rdev);
	int ret, id = rdev_get_id(rdev);
	unsigned int status;

	ret = regulator_is_enabled_regmap(rdev);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to read enable register(%d)\n", ret);
		return ret;
	}

	if (!ret)
		return REGULATOR_STATUS_OFF;

	ret = regmap_read(chip->regmap, es_reg[id].sreg, &status);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to read status register(%d)\n", ret);
		return ret;
	}

	if (!(status & SLG51000_STA_ILIM_FLAG_MASK) &&
	    (status & SLG51000_STA_VOUT_OK_FLAG_MASK)) {
		if (rdev->desc->n_voltages == 0 &&
		    (id == SLG51000_REGULATOR_LDO5 ||
		     id == SLG51000_REGULATOR_LDO6))
			return REGULATOR_STATUS_BYPASS;
		else
			return REGULATOR_STATUS_ON;
	}

	return REGULATOR_STATUS_ERROR;
}

static int slg51000_regulator_set_voltage_sel_regmap(struct regulator_dev *rdev, unsigned int sel)
{
	struct slg51000_dev *chip = rdev_get_drvdata(rdev);

	if (chip->op_mode == SLG51000_OP_MODE_CONTROL_REG)
		return 0;

	return regulator_set_voltage_sel_regmap(rdev, sel);
}

static const struct regulator_ops slg51000_regl_ops = {
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
	.list_voltage = regulator_list_voltage_linear,
	.map_voltage = regulator_map_voltage_linear,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.set_voltage_sel = slg51000_regulator_set_voltage_sel_regmap,
	.get_status = slg51000_get_status,
};

static const struct regulator_ops slg51000_switch_ops = {
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
	.get_status = slg51000_get_status,
};

static int slg51000_of_parse_cb(struct device_node *np,
				const struct regulator_desc *desc,
				struct regulator_config *config)
{
	struct gpio_desc *ena_gpiod;

	ena_gpiod = fwnode_gpiod_get_index(of_fwnode_handle(np), "enable", 0,
					   GPIOD_OUT_LOW |
						GPIOD_FLAGS_BIT_NONEXCLUSIVE,
					   "gpio-en-ldo");
	if (!IS_ERR(ena_gpiod))
		config->ena_gpiod = ena_gpiod;

	return 0;
}

#define SLG51000_REGL_DESC(_id, _name, _s_name, _min, _step) \
	[SLG51000_REGULATOR_##_id] = {                             \
		.name = #_name,                                    \
		.supply_name = _s_name,                            \
		.id = SLG51000_REGULATOR_##_id,                    \
		.of_match = of_match_ptr(#_name),                  \
		.of_parse_cb = slg51000_of_parse_cb,               \
		.ops = &slg51000_regl_ops,                         \
		.regulators_node = of_match_ptr("regulators"),     \
		.n_voltages = 256,                                 \
		.min_uV = _min,                                    \
		.uV_step = _step,                                  \
		.linear_min_sel = 0,                               \
		.vsel_mask = SLG51000_VSEL_MASK,                   \
		.vsel_reg = SLG51000_##_id##_VSEL,                 \
		.enable_reg = SLG51000_SYSCTL_MATRIX_CONF_A,       \
		.enable_mask = BIT(SLG51000_REGULATOR_##_id),      \
		.type = REGULATOR_VOLTAGE,                         \
		.owner = THIS_MODULE,                              \
	}

static struct regulator_desc regls_desc[SLG51000_MAX_REGULATORS] = {
	SLG51000_REGL_DESC(LDO1, ldo1, NULL,   2200000,  5000),
	SLG51000_REGL_DESC(LDO2, ldo2, NULL,   2200000,  5000),
	SLG51000_REGL_DESC(LDO3, ldo3, "vin3", 1200000, 10000),
	SLG51000_REGL_DESC(LDO4, ldo4, "vin4", 1200000, 10000),
	SLG51000_REGL_DESC(LDO5, ldo5, "vin5",  400000,  5000),
	SLG51000_REGL_DESC(LDO6, ldo6, "vin6",  400000,  5000),
	SLG51000_REGL_DESC(LDO7, ldo7, "vin7", 1200000, 10000),
	SLG51000_REGL_DESC(LDO_DUMMY, ldo_dummy, NULL, 1200000, 10000),
};

static int slg51000_regulator_register(struct slg51000_dev *chip)
{
	struct regulator_config config = { };
	struct regulator_desc *rdesc;
	unsigned int reg, val;
	u8 vsel_range[2];
	int id, ret = 0;
	const unsigned int min_regs[SLG51000_MAX_REGULATORS] = {
		SLG51000_LDO1_MINV, SLG51000_LDO2_MINV, SLG51000_LDO3_MINV,
		SLG51000_LDO4_MINV, SLG51000_LDO5_MINV, SLG51000_LDO6_MINV,
		SLG51000_LDO7_MINV, SLG51000_LDO_DUMMY_MINV,
	};

	for (id = 0; id < SLG51000_MAX_REGULATORS; id++) {
		if (chip->op_mode != SLG51000_OP_MODE_CONTROL_REG &&
			id == SLG51000_REGULATOR_LDO_DUMMY)
			continue;

		chip->rdesc[id] = &regls_desc[id];
		rdesc = chip->rdesc[id];
		config.regmap = chip->regmap;
		config.dev = chip->dev;
		config.driver_data = chip;

		ret = regmap_bulk_read(chip->regmap, min_regs[id],
				vsel_range, ARRAY_SIZE(vsel_range));
		if (ret < 0) {
			dev_err(chip->dev,
				"Failed to read the MIN register\n");
			return ret;
		}

		switch (id) {
		case SLG51000_REGULATOR_LDO1:
		case SLG51000_REGULATOR_LDO2:
			if (id == SLG51000_REGULATOR_LDO1)
				reg = SLG51000_LDO1_MISC1;
			else
				reg = SLG51000_LDO2_MISC1;

			ret = regmap_read(chip->regmap, reg, &val);
			if (ret < 0) {
				dev_err(chip->dev,
					"Failed to read voltage range of ldo%d\n",
					id + 1);
				return ret;
			}

			rdesc->linear_min_sel = vsel_range[0];
			rdesc->n_voltages = vsel_range[1] + 1;
			if (val & SLG51000_SEL_VRANGE_MASK)
				rdesc->min_uV = SLG51000_LDOHP_HV_MIN
						+ (vsel_range[0]
						   * rdesc->uV_step);
			else
				rdesc->min_uV = SLG51000_LDOHP_LV_MIN
						+ (vsel_range[0]
						   * rdesc->uV_step);
			break;

		case SLG51000_REGULATOR_LDO5:
		case SLG51000_REGULATOR_LDO6:
			if (id == SLG51000_REGULATOR_LDO5)
				reg = SLG51000_LDO5_TRIM2;
			else
				reg = SLG51000_LDO6_TRIM2;

			ret = regmap_read(chip->regmap, reg, &val);
			if (ret < 0) {
				dev_err(chip->dev,
					"Failed to read LDO mode register\n");
				return ret;
			}

			if (val & SLG51000_SEL_BYP_MODE_MASK) {
				rdesc->ops = &slg51000_switch_ops;
				rdesc->n_voltages = 0;
				rdesc->min_uV = 0;
				rdesc->uV_step = 0;
				rdesc->linear_min_sel = 0;
				break;
			}
			fallthrough;	/* to the check below */

		default:
			rdesc->linear_min_sel = vsel_range[0];
			rdesc->n_voltages = vsel_range[1] + 1;
			rdesc->min_uV = rdesc->min_uV +
					(vsel_range[0] * rdesc->uV_step);
			break;
		}

		chip->rdev[id] = devm_regulator_register(
				chip->dev, rdesc, &config);
		if (IS_ERR(chip->rdev[id])) {
			ret = PTR_ERR(chip->rdev[id]);
			dev_err(chip->dev,
				"Failed to register regulator(%s):%d\n",
				chip->rdesc[id]->name, ret);
			return ret;
		}
	}

	return 0;
}

static void slg51000_work_func(struct work_struct *work)
{
	struct slg51000_dev *chip =
		container_of(work, struct slg51000_dev, slg51000_work);
	struct regmap *regmap = chip->regmap;
	enum { R0 = 0, R1, R2, REG_MAX };
	u8 evt[SLG51000_MAX_EVT_REGISTER][REG_MAX];
	int ret, i;
	unsigned int evt_otp, mask_otp;

	/* Read event[R0], status[R1] and mask[R2] register */
	for (i = 0; i < SLG51000_MAX_EVT_REGISTER; i++) {
		ret = regmap_bulk_read(regmap, es_reg[i].ereg, evt[i], REG_MAX);
		if (ret < 0) {
			dev_err(chip->dev,
				"Failed to read event registers(%d)\n", ret);
			return;
		}
	}

	ret = regmap_read(regmap, SLG51000_OTP_EVENT, &evt_otp);
	if (ret < 0) {
		dev_err(chip->dev,
			"Failed to read otp event registers(%d)\n", ret);
		return;
	}

	ret = regmap_read(regmap, SLG51000_OTP_IRQ_MASK, &mask_otp);
	if (ret < 0) {
		dev_err(chip->dev,
			"Failed to read otp mask register(%d)\n", ret);
		return;
	}

	if ((evt_otp & SLG51000_EVT_CRC_MASK) &&
	    !(mask_otp & SLG51000_IRQ_CRC_MASK))
		dev_info(chip->dev,
			 "OTP has been read or OTP crc is not zero\n");

	for (i = 0; i < SLG51000_MAX_REGULATORS; i++) {
		if (!(evt[i][R2] & SLG51000_IRQ_ILIM_FLAG_MASK) &&
		    (evt[i][R0] & SLG51000_EVT_ILIM_FLAG_MASK)) {
			regulator_notifier_call_chain(chip->rdev[i],
					    REGULATOR_EVENT_OVER_CURRENT, NULL);

			if (evt[i][R1] & SLG51000_STA_ILIM_FLAG_MASK)
				dev_warn(chip->dev,
					 "Over-current limit(ldo%d)\n", i + 1);
		}
	}

	if (!(evt[SLG51000_SCTL_EVT][R2] & SLG51000_IRQ_HIGH_TEMP_WARN_MASK) &&
	    (evt[SLG51000_SCTL_EVT][R0] & SLG51000_EVT_HIGH_TEMP_WARN_MASK)) {
		for (i = 0; i < SLG51000_MAX_REGULATORS; i++) {
			if (!(evt[i][R1] & SLG51000_STA_ILIM_FLAG_MASK) &&
			    (evt[i][R1] & SLG51000_STA_VOUT_OK_FLAG_MASK)) {
				regulator_notifier_call_chain(chip->rdev[i],
					       REGULATOR_EVENT_OVER_TEMP, NULL);
			}
		}
		if (evt[SLG51000_SCTL_EVT][R1] &
		    SLG51000_STA_HIGH_TEMP_WARN_MASK)
			dev_warn(chip->dev, "High temperature warning!\n");
	}
}

static irqreturn_t slg51000_irq_handler(int irq, void *data)
{
	struct slg51000_dev *chip = data;

	if (chip == NULL)
		return IRQ_NONE;

	queue_work(chip->slg51000_wq, &chip->slg51000_work);

	return IRQ_HANDLED;
}

static int slg51000_regulator_probe(struct platform_device *pdev)
{
	struct slg51000_dev *chip = dev_get_drvdata(pdev->dev.parent);
	int ret;

	if (chip->op_mode != SLG51000_OP_MODE_LDO_ONLY &&
			chip->op_mode != SLG51000_OP_MODE_LDO_GPIO &&
			chip->op_mode != SLG51000_OP_MODE_CONTROL_REG)
		return -ENODEV;

	ret = slg51000_regulator_register(chip);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to init regulator(%d)\n", ret);
		return ret;
	}

	if (chip->chip_irq) {
		ret = devm_request_threaded_irq(
			chip->dev, chip->chip_irq, NULL, slg51000_irq_handler,
			(IRQF_TRIGGER_HIGH | IRQF_ONESHOT), "slg51000-irq",
			chip);
		if (ret != 0) {
			dev_err(chip->dev, "Failed to request IRQ: %d\n",
				chip->chip_irq);
			return ret;
		}
	} else
		dev_dbg(chip->dev, "No IRQ configured\n");

	chip->slg51000_wq = create_workqueue("slg51000wq");
	INIT_WORK(&chip->slg51000_work, slg51000_work_func);

	return ret;
}

static int slg51000_regulator_remove(struct platform_device *pdev)
{
	struct slg51000_dev *chip = dev_get_drvdata(pdev->dev.parent);

	flush_workqueue(chip->slg51000_wq);
	destroy_workqueue(chip->slg51000_wq);

	return 0;
}

static const struct platform_device_id slg51000_regulator_id[] = {
	{"slg51000-regulator", 0},
	{},
};
MODULE_DEVICE_TABLE(platform, slg51000_regulator_id);

static struct platform_driver slg51000_regulator_driver = {
	.driver = {
		.name = "slg51000-regulator",
	},
	.probe = slg51000_regulator_probe,
	.remove = slg51000_regulator_remove,
	.id_table = slg51000_regulator_id,
};

static int __init slg51000_regulator_init(void)
{
	return platform_driver_register(&slg51000_regulator_driver);
}
subsys_initcall(slg51000_regulator_init);

static void __exit slg51000_regulator_exit(void)
{
	platform_driver_unregister(&slg51000_regulator_driver);
}
module_exit(slg51000_regulator_exit);

MODULE_AUTHOR("Holmes Chou <holmeschou@google.com>");
MODULE_AUTHOR("Sam Chang <samchchang@google.com>");
MODULE_AUTHOR("Speth Chang <spethchang@google.com>");
MODULE_AUTHOR("CY Tseng <cytseng@google.com>");
MODULE_AUTHOR("Eric Jeong <eric.jeong.opensource@diasemi.com>");
MODULE_DESCRIPTION("SLG51000 regulator driver");
MODULE_LICENSE("GPL");

