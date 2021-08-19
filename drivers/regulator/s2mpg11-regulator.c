// SPDX-License-Identifier: GPL-2.0+
/*
 * drivers/regulator/s2mpg11-regulator.c
 *
 * Copyright (c) 2015 Samsung Electronics Co., Ltd
 *              http://www.samsung.com
 */

#include <linux/async.h>
#include <linux/bug.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <../drivers/pinctrl/samsung/pinctrl-samsung.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/mfd/samsung/s2mpg11.h>
#include <linux/mfd/samsung/s2mpg11-register.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/debugfs.h>
#include <linux/interrupt.h>
#include <linux/regulator/pmic_class.h>
#include <dt-bindings/regulator/samsung,s2mpg-regulator.h>

static struct regulator_desc regulators[S2MPG11_REGULATOR_MAX];

static unsigned int s2mpg11_of_map_mode(unsigned int val)
{
	switch (val) {
	case SEC_OPMODE_SUSPEND: /* ON in Standby Mode */
		return 0x1;
	case SEC_OPMODE_MIF: /* ON in PWREN_MIF mode */
		return 0x2;
	case SEC_OPMODE_ON: /* ON in Normal Mode */
		return 0x3;
	default:
		return 0x3;
	}
}

static int s2m_set_mode(struct regulator_dev *rdev, unsigned int mode)
{
	struct s2mpg11_pmic *s2mpg11 = rdev_get_drvdata(rdev);
	unsigned int val;
	int id = rdev_get_id(rdev);
	int enable_mask = rdev->desc->enable_mask;
	int enable_shift = 0;

	while (1) {
		if (enable_mask & 0x1) {
			break;
		} else {
			enable_shift++;
			enable_mask = enable_mask >> 1;
		}

		if (enable_shift > 7) {
			pr_err("%s [%d]: error caculating enable_shift!\n",
			       __func__, id);
		}
	};

	val = (mode << enable_shift) & rdev->desc->enable_mask;

	s2mpg11->opmode[id] = val;
	return 0;
}

static int s2m_enable(struct regulator_dev *rdev)
{
	struct s2mpg11_pmic *s2mpg11 = rdev_get_drvdata(rdev);
	int id = rdev_get_id(rdev);

	if (atomic_read(&s2mpg11->need_sync[id]) > 0)
		s2mpg11->turn_off_on_sync[id] = false;

	return s2mpg11_update_reg(s2mpg11->i2c, rdev->desc->enable_reg,
				  s2mpg11->opmode[rdev_get_id(rdev)],
				  rdev->desc->enable_mask);
}

static int s2m_disable(struct regulator_dev *rdev)
{
	struct s2mpg11_pmic *s2mpg11 = rdev_get_drvdata(rdev);
	unsigned int val;
	int id = rdev_get_id(rdev);

	if (atomic_read(&s2mpg11->need_sync[id]) > 0) {
		s2mpg11->turn_off_on_sync[id] = true;
		return 0;
	}

	if (rdev->desc->enable_is_inverted)
		val = rdev->desc->enable_mask;
	else
		val = 0;

	return s2mpg11_update_reg(s2mpg11->i2c, rdev->desc->enable_reg, val,
				  rdev->desc->enable_mask);
}

static int s2m_is_enabled(struct regulator_dev *rdev)
{
	struct s2mpg11_pmic *s2mpg11 = rdev_get_drvdata(rdev);
	int ret;
	u8 val;

	ret = s2mpg11_read_reg(s2mpg11->i2c, rdev->desc->enable_reg, &val);
	if (ret)
		return ret;

	if (rdev->desc->enable_is_inverted)
		return (val & rdev->desc->enable_mask) == 0;
	else
		return (val & rdev->desc->enable_mask) != 0;
}

static int s2m_get_voltage_sel(struct regulator_dev *rdev)
{
	struct s2mpg11_pmic *s2mpg11 = rdev_get_drvdata(rdev);
	int ret;
	u8 val;

	ret = s2mpg11_read_reg(s2mpg11->i2c, rdev->desc->vsel_reg, &val);
	if (ret)
		return ret;

	val &= rdev->desc->vsel_mask;

	return val;
}

static int s2m_set_voltage_sel(struct regulator_dev *rdev, unsigned int sel)
{
	struct s2mpg11_pmic *s2mpg11 = rdev_get_drvdata(rdev);
	int ret;

	ret = s2mpg11_update_reg(s2mpg11->i2c, rdev->desc->vsel_reg, sel,
				 rdev->desc->vsel_mask);
	if (ret < 0)
		goto out;

	if (rdev->desc->apply_bit)
		ret = s2mpg11_update_reg(s2mpg11->i2c, rdev->desc->apply_reg,
					 rdev->desc->apply_bit,
					 rdev->desc->apply_bit);
	return ret;
out:
	pr_warn("%s: failed to set regulator voltage\n", rdev->desc->name);
	ret = -EINVAL;
	return ret;
}

static int s2m_set_voltage_time_sel(struct regulator_dev *rdev,
				    unsigned int old_selector,
				    unsigned int new_selector)
{
	unsigned int ramp_delay = 0;
	int old_volt, new_volt;

	if (rdev->constraints->ramp_delay)
		ramp_delay = rdev->constraints->ramp_delay;
	else if (rdev->desc->ramp_delay)
		ramp_delay = rdev->desc->ramp_delay;

	if (ramp_delay == 0) {
		pr_warn("%s: ramp_delay not set\n", rdev->desc->name);
		return -EINVAL;
	}

	/* sanity check */
	if (!rdev->desc->ops->list_voltage)
		return -EINVAL;

	old_volt = rdev->desc->ops->list_voltage(rdev, old_selector);
	new_volt = rdev->desc->ops->list_voltage(rdev, new_selector);

	if (old_selector < new_selector)
		return DIV_ROUND_UP(new_volt - old_volt, ramp_delay);
	else
		return DIV_ROUND_UP(old_volt - new_volt, ramp_delay);

	return 0;
}

static struct regulator_ops s2mpg11_regulator_ops = {
	.list_voltage = regulator_list_voltage_linear,
	.map_voltage = regulator_map_voltage_linear,
	.is_enabled = s2m_is_enabled,
	.enable = s2m_enable,
	.disable = s2m_disable,
	.get_voltage_sel = s2m_get_voltage_sel,
	.set_voltage_sel = s2m_set_voltage_sel,
	.set_voltage_time_sel = s2m_set_voltage_time_sel,
	.set_mode = s2m_set_mode,
};

#define _BUCK(macro) S2MPG11_BUCK##macro
#define _LDO(macro) S2MPG11_LDO##macro
#define _REG(ctrl) S2MPG11_PM##ctrl
#define _TIME(macro) S2MPG11_ENABLE_TIME##macro
#define _MIN(group) S2MPG11_REG_MIN##group
#define _STEP(group) S2MPG11_REG_STEP##group
#define _N_VOLTAGES(num) S2MPG11_REG_N_VOLTAGES_##num
#define _MASK(num) S2MPG11_REG_ENABLE_MASK##num

#define REG_DESC(_name, _id, g, v, n, e, em, t)                           \
	{                                                                 \
		.name = _name, .id = _id, .ops = &s2mpg11_regulator_ops,  \
		.type = REGULATOR_VOLTAGE, .owner = THIS_MODULE,          \
		.min_uV = _MIN(g), .uV_step = _STEP(g), .n_voltages = n,  \
		.vsel_reg = v, .vsel_mask = n - 1, .enable_reg = e,       \
		.enable_mask = em, .enable_time = t,                      \
		.of_map_mode = s2mpg11_of_map_mode                        \
	}

static struct regulator_desc regulators[S2MPG11_REGULATOR_MAX] = {
	/* name, id, voltage_group, vsel_reg, n_voltages, */
	/* enable_reg, enable_mask, enable_time */
	REG_DESC("LDO1S", _LDO(1), 4, _REG(_L1S_CTRL1), _N_VOLTAGES(128),
		 _REG(_LDO_CTRL1), _MASK(_5_4), _TIME(_LDO)),
	REG_DESC("LDO2S", _LDO(2), 4, _REG(_L2S_CTRL1), _N_VOLTAGES(128),
		 _REG(_LDO_CTRL1), _MASK(_7_6), _TIME(_LDO)),
	REG_DESC("LDO3S", _LDO(3), 6, _REG(_L3S_CTRL), _N_VOLTAGES(64),
		 _REG(_L3S_CTRL), _MASK(_7), _TIME(_LDO)),
	REG_DESC("LDO4S", _LDO(4), 7, _REG(_L4S_CTRL), _N_VOLTAGES(64),
		 _REG(_L4S_CTRL), _MASK(_7), _TIME(_LDO)),
	REG_DESC("LDO5S", _LDO(5), 8, _REG(_L5S_CTRL), _N_VOLTAGES(64),
		 _REG(_L5S_CTRL), _MASK(_7), _TIME(_LDO)),
	REG_DESC("LDO6S", _LDO(6), 7, _REG(_L6S_CTRL), _N_VOLTAGES(64),
		 _REG(_L6S_CTRL), _MASK(_7), _TIME(_LDO)),
	REG_DESC("LDO7S", _LDO(7), 6, _REG(_L7S_CTRL), _N_VOLTAGES(64),
		 _REG(_L7S_CTRL), _MASK(_7), _TIME(_LDO)),
	REG_DESC("LDO8S", _LDO(8), 9, _REG(_L8S_CTRL), _N_VOLTAGES(64),
		 _REG(_L8S_CTRL), _MASK(_7_6), _TIME(_LDO)),
	REG_DESC("LDO9S", _LDO(9), 5, _REG(_L9S_CTRL), _N_VOLTAGES(64),
		 _REG(_L9S_CTRL), _MASK(_7), _TIME(_LDO)),
	REG_DESC("LDO10S", _LDO(10), 6, _REG(_L10S_CTRL), _N_VOLTAGES(64),
		 _REG(_L10S_CTRL), _MASK(_7), _TIME(_LDO)),
	REG_DESC("LDO11S", _LDO(11), 6, _REG(_L11S_CTRL), _N_VOLTAGES(64),
		 _REG(_L11S_CTRL), _MASK(_7), _TIME(_LDO)),
	REG_DESC("LDO12S", _LDO(12), 6, _REG(_L12S_CTRL), _N_VOLTAGES(64),
		 _REG(_L12S_CTRL), _MASK(_7), _TIME(_LDO)),
	REG_DESC("LDO13S", _LDO(13), 7, _REG(_L13S_CTRL), _N_VOLTAGES(64),
		 _REG(_L13S_CTRL), _MASK(_7_6), _TIME(_LDO)),
	REG_DESC("LDO14S", _LDO(14), 6, _REG(_L14S_CTRL), _N_VOLTAGES(64),
		 _REG(_L14S_CTRL), _MASK(_7), _TIME(_LDO)),
	REG_DESC("LDO15S", _LDO(15), 6, _REG(_L15S_CTRL), _N_VOLTAGES(64),
		 _REG(_L15S_CTRL), _MASK(_7), _TIME(_LDO)),
	REG_DESC("BUCK1S", _BUCK(1), 1, _REG(_B1S_OUT1), _N_VOLTAGES(256),
		 _REG(_B1S_CTRL), _MASK(_7_6), _TIME(_BUCK)),
	REG_DESC("BUCK2S", _BUCK(2), 1, _REG(_B2S_OUT1), _N_VOLTAGES(256),
		 _REG(_B2S_CTRL), _MASK(_7_6), _TIME(_BUCK)),
	REG_DESC("BUCK3S", _BUCK(3), 1, _REG(_B3S_OUT1), _N_VOLTAGES(256),
		 _REG(_B3S_CTRL), _MASK(_7_6), _TIME(_BUCK)),
	REG_DESC("BUCK4S", _BUCK(4), 1, _REG(_B4S_OUT), _N_VOLTAGES(256),
		 _REG(_B4S_CTRL), _MASK(_7), _TIME(_BUCK)),
	REG_DESC("BUCK5S", _BUCK(5), 1, _REG(_B5S_OUT), _N_VOLTAGES(256),
		 _REG(_B5S_CTRL), _MASK(_7_6), _TIME(_BUCK)),
	REG_DESC("BUCK6S", _BUCK(6), 1, _REG(_B6S_OUT1), _N_VOLTAGES(256),
		 _REG(_B6S_CTRL), _MASK(_7), _TIME(_BUCK)),
	REG_DESC("BUCK7S", _BUCK(7), 2, _REG(_B7S_OUT1), _N_VOLTAGES(256),
		 _REG(_B7S_CTRL), _MASK(_7), _TIME(_BUCK)),
	REG_DESC("BUCK8S", _BUCK(8), 1, _REG(_B8S_OUT1), _N_VOLTAGES(256),
		 _REG(_B8S_CTRL), _MASK(_7_6), _TIME(_BUCK)),
	REG_DESC("BUCK9S", _BUCK(9), 1, _REG(_B9S_OUT1), _N_VOLTAGES(256),
		 _REG(_B9S_CTRL), _MASK(_7_6), _TIME(_BUCK)),
	REG_DESC("BUCK10S", _BUCK(10), 1, _REG(_B10S_OUT), _N_VOLTAGES(256),
		 _REG(_B10S_CTRL), _MASK(_7), _TIME(_BUCK)),
	REG_DESC("BUCKD", _BUCK(D), 2, _REG(_BUCKD_OUT), _N_VOLTAGES(256),
		 _REG(_BUCKD_CTRL), _MASK(_7_6), _TIME(_BUCK)),
	REG_DESC("BUCKA", _BUCK(A), 2, _REG(_BUCKA_OUT), _N_VOLTAGES(256),
		 _REG(_BUCKA_CTRL), _MASK(_7_6), _TIME(_BUCK)),
	REG_DESC("BUCKBOOST", S2MPG11_BUCKBOOST, 3, _REG(_BB_OUT1),
		 _N_VOLTAGES(128), _REG(_BB_CTRL), _MASK(_7), _TIME(_BUCK)),
};

#if IS_ENABLED(CONFIG_OF)
static int s2mpg11_pmic_dt_parse_pdata(struct s2mpg11_dev *iodev,
				       struct s2mpg11_platform_data *pdata)
{
	struct device_node *pmic_np, *regulators_np, *reg_np;
	struct s2mpg11_regulator_data *rdata;
	unsigned int i;
	int ret;
	u32 val;

	pmic_np = iodev->dev->of_node;
	if (!pmic_np) {
		dev_err(iodev->dev, "could not find pmic sub-node\n");
		return -ENODEV;
	}

	regulators_np = of_find_node_by_name(pmic_np, "regulators");
	if (!regulators_np) {
		dev_err(iodev->dev, "could not find regulators sub-node\n");
		return -EINVAL;
	}

	/* count the number of regulators to be supported in pmic */
	pdata->num_regulators = 0;
	for_each_child_of_node(regulators_np, reg_np) {
		pdata->num_regulators++;
	}

	rdata = devm_kzalloc(iodev->dev, sizeof(*rdata) * pdata->num_regulators,
			     GFP_KERNEL);
	if (!rdata)
		return -ENOMEM;

	pdata->regulators = rdata;
	for_each_child_of_node(regulators_np, reg_np) {
		for (i = 0; i < ARRAY_SIZE(regulators); i++)
			if (!of_node_cmp(reg_np->name, regulators[i].name))
				break;

		if (i == ARRAY_SIZE(regulators)) {
			dev_warn(iodev->dev,
				 "don't know how to configure regulator %s\n",
				 reg_np->name);
			continue;
		}

		rdata->id = i;
		rdata->initdata = of_get_regulator_init_data(iodev->dev, reg_np,
							     &regulators[i]);
		rdata->reg_node = reg_np;
		rdata++;
	}

	/* parse OCP_WARN information */
	pdata->b2_ocp_warn_pin = of_get_gpio(pmic_np, 0);
	if (pdata->b2_ocp_warn_pin < 0)
		dev_err(iodev->dev, "b2_ocp_warn_pin < 0: %d\n",
			pdata->b2_ocp_warn_pin);

	/* parse BUCK OCP Detection information */
	ret = of_property_read_u32(pmic_np, "buck_ocp_ctrl1", &val);
	pdata->buck_ocp_ctrl1 = ret ? 0 : val;

	ret = of_property_read_u32(pmic_np, "b2_ocp_warn_en", &val);
	pdata->b2_ocp_warn_en = ret ? 0 : val;

	ret = of_property_read_u32(pmic_np, "b2_ocp_warn_cnt", &val);
	pdata->b2_ocp_warn_cnt = ret ? 0 : val;

	ret = of_property_read_u32(pmic_np, "b2_ocp_warn_dvs_mask", &val);
	pdata->b2_ocp_warn_dvs_mask = ret ? 0 : val;

	ret = of_property_read_u32(pmic_np, "b2_ocp_warn_lvl", &val);
	pdata->b2_ocp_warn_lvl = ret ? 0 : val;

	/* parse SOFT_OCP_WARN information */
	pdata->b2_soft_ocp_warn_pin = of_get_gpio(pmic_np, 1);
	if (pdata->b2_soft_ocp_warn_pin < 0)
		dev_err(iodev->dev, "b2_soft_ocp_warn_pin < 0: %d\n",
			pdata->b2_soft_ocp_warn_pin);

	ret = of_property_read_u32(pmic_np, "b2_soft_ocp_warn_en", &val);
	pdata->b2_soft_ocp_warn_en = ret ? 0 : val;

	ret = of_property_read_u32(pmic_np, "b2_soft_ocp_warn_cnt", &val);
	pdata->b2_soft_ocp_warn_cnt = ret ? 0 : val;

	ret = of_property_read_u32(pmic_np, "b2_soft_ocp_warn_dvs_mask", &val);
	pdata->b2_soft_ocp_warn_dvs_mask = ret ? 0 : val;

	ret = of_property_read_u32(pmic_np, "b2_soft_ocp_warn_lvl", &val);
	pdata->b2_soft_ocp_warn_lvl = ret ? 0 : val;

	return 0;
}
#else
static int s2mpg11_pmic_dt_parse_pdata(struct s2mpg11_dev *iodev,
				       struct s2mpg11_platform_data *pdata)
{
	return 0;
}
#endif /* CONFIG_OF */

#if IS_ENABLED(CONFIG_DRV_SAMSUNG_PMIC)

#define I2C_ADDR_TOP 0x00
#define I2C_ADDR_PMIC 0x01
#define I2C_ADDR_METER 0x0A
#define I2C_ADDR_WLWP 0x0B
#define I2C_ADDR_TRIM 0x0F

static ssize_t s2mpg11_pmic_read_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t size)
{
	struct s2mpg11_pmic *s2mpg11 = dev_get_drvdata(dev);
	int ret;
	u16 reg_addr;

	if (!buf)
		return -1;

	ret = kstrtou16(buf, 0, &reg_addr);
	if (ret < 0) {
		pr_err("%s: fail to transform i2c address\n", __func__);
		return ret;
	}

	s2mpg11->read_addr = reg_addr;

	return size;
}

static ssize_t s2mpg11_pmic_read_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct s2mpg11_pmic *s2mpg11 = dev_get_drvdata(dev);
	struct i2c_client *client = NULL;
	u16 reg_addr = s2mpg11->read_addr;
	u8 val;
	int ret;

	switch (reg_addr >> 8){
	case I2C_ADDR_TOP:
		client = s2mpg11->iodev->i2c;
		break;
	case I2C_ADDR_PMIC:
		client = s2mpg11->iodev->pmic;
		break;
	case I2C_ADDR_METER:
		client = s2mpg11->iodev->meter;
		break;
	case I2C_ADDR_WLWP:
		client = s2mpg11->iodev->wlwp;
		break;
	case I2C_ADDR_TRIM:
		client = s2mpg11->iodev->trim;
		break;
	default:
		return -1;
	}

	ret = s2mpg11_read_reg(client, reg_addr, &val);
	if (ret < 0) {
		pr_err("%s: fail to read i2c address\n", __func__);
		return ret;
	}

	pr_debug("%s: reg(0x%04X) data(0x%02X)\n", __func__, reg_addr, val);

	return scnprintf(buf, PAGE_SIZE, "0x%04X: 0x%02X\n", reg_addr, val);
}

static ssize_t s2mpg11_pmic_write_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t size)
{
	struct s2mpg11_pmic *s2mpg11 = dev_get_drvdata(dev);
	struct i2c_client *client = NULL;
	int ret;
	u16 reg;
	u8 data;

	if (!buf) {
		pr_err("%s: empty buffer\n", __func__);
		return size;
	}

	ret = sscanf(buf, "%x %x", &reg, &data);
	if (ret != 2) {
		pr_err("%s: input error\n", __func__);
		return size;
	}

	pr_debug("%s: reg(0x%04X) data(0x%02X)\n", __func__, reg, data);

	switch (reg >> 8){
	case I2C_ADDR_TOP:
		client = s2mpg11->iodev->i2c;
		break;
	case I2C_ADDR_PMIC:
		client = s2mpg11->iodev->pmic;
		break;
	case I2C_ADDR_METER:
		client = s2mpg11->iodev->meter;
		break;
	case I2C_ADDR_WLWP:
		client = s2mpg11->iodev->wlwp;
		break;
	case I2C_ADDR_TRIM:
		client = s2mpg11->iodev->trim;
		break;
	default:
		return size;
	}

	ret = s2mpg11_write_reg(client, reg, data);
	if (ret < 0)
		pr_err("%s: fail to write i2c addr/data\n", __func__);

	return size;
}

static ssize_t s2mpg11_pmic_write_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "echo (register addr.) (data) > s2mpg11_write\n");
}

static DEVICE_ATTR_RW(s2mpg11_pmic_write);
static DEVICE_ATTR_RW(s2mpg11_pmic_read);

int create_s2mpg11_pmic_sysfs(struct s2mpg11_pmic *s2mpg11)
{
	struct device *s2mpg11_pmic = s2mpg11->dev;
	int err = -ENODEV;

	pr_info("%s: master pmic sysfs start\n", __func__);
	s2mpg11->read_addr = 0;

	s2mpg11_pmic = pmic_device_create(s2mpg11, "s2mpg11-pmic");

	err = device_create_file(s2mpg11_pmic, &dev_attr_s2mpg11_pmic_write);
	if (err) {
		pr_err("s2mpg11_sysfs: failed to create device file, %s\n",
		       dev_attr_s2mpg11_pmic_write.attr.name);
	}

	err = device_create_file(s2mpg11_pmic, &dev_attr_s2mpg11_pmic_read);
	if (err) {
		pr_err("s2mpg11_sysfs: failed to create device file, %s\n",
		       dev_attr_s2mpg11_pmic_read.attr.name);
	}

	return 0;
}
#endif

static irqreturn_t s2mpg11_buck_ocp_irq(int irq, void *data)
{
	struct s2mpg11_pmic *s2mpg11 = data;
	int i;

	mutex_lock(&s2mpg11->lock);

	for (i = 0; i < 12; i++) {
		if (s2mpg11->buck_ocp_irq[i] == irq) {
			pr_info_ratelimited("%s : BUCK[%d] OCP IRQ, %d\n",
					    __func__, i + 1, irq);
			break;
		}
	}

	mutex_unlock(&s2mpg11->lock);
	return IRQ_HANDLED;
}

void s2mpg11_ocp_detection_config(struct s2mpg11_pmic *s2mpg11,
				  struct s2mpg11_platform_data *pdata)
{
	int ret;

	pr_info("OCP BUCK_OCP_CTRL1: 0x%x\n", pdata->buck_ocp_ctrl1);
	ret = s2mpg11_write_reg(s2mpg11->i2c, S2MPG11_PM_BUCK_OCP_CTRL1, pdata->buck_ocp_ctrl1);
	if (ret)
		pr_err("i2c write error setting BUCK_OCP_CTRL1: %d\n", ret);
}

void s2mpg11_ocp_warn(struct s2mpg11_pmic *s2mpg11,
		      struct s2mpg11_platform_data *pdata)
{
	u8 val;
	int ret;

	val = (pdata->b2_ocp_warn_en << S2MPG11_OCP_WARN_EN_SHIFT) |
	      (pdata->b2_ocp_warn_cnt << S2MPG11_OCP_WARN_CNT_SHIFT) |
	      (pdata->b2_ocp_warn_dvs_mask << S2MPG11_OCP_WARN_DVS_MASK_SHIFT) |
	      (pdata->b2_ocp_warn_lvl << S2MPG11_OCP_WARN_LVL_SHIFT);

	pr_info("B2S_OCP_WARN : 0x%x\n", val);
	ret = s2mpg11_write_reg(s2mpg11->i2c, S2MPG11_PM_B2S_OCP_WARN, val);
	if (ret)
		pr_err("i2c write error setting b2s_ocp_warn\n");

	val = (pdata->b2_soft_ocp_warn_en << S2MPG11_OCP_WARN_EN_SHIFT) |
	      (pdata->b2_soft_ocp_warn_cnt << S2MPG11_OCP_WARN_CNT_SHIFT) |
	      (pdata->b2_soft_ocp_warn_dvs_mask
	       << S2MPG11_OCP_WARN_DVS_MASK_SHIFT) |
	      (pdata->b2_soft_ocp_warn_lvl << S2MPG11_OCP_WARN_LVL_SHIFT);

	pr_info("B2S_SOFT_OCP_WARN : 0x%x\n", val);
	ret = s2mpg11_write_reg(s2mpg11->i2c, S2MPG11_PM_B2S_SOFT_OCP_WARN,
				val);
	if (ret)
		pr_err("i2c write error setting b2s_soft_ocp_warn\n");
}

void s2mpg11_oi_function(struct s2mpg11_pmic *s2mpg11)
{
	/* add OI configuration code if necessary */

	/* OI function enable */

	/* OI power down disable */

	/* OI detection time window : 500us, OI comp. output count : 50 times */
}

static int s2mpg11_pmic_probe(struct platform_device *pdev)
{
	struct s2mpg11_dev *iodev = dev_get_drvdata(pdev->dev.parent);
	struct s2mpg11_platform_data *pdata = iodev->pdata;
	struct regulator_config config = {};
	struct s2mpg11_pmic *s2mpg11;
	int irq_base;
	int i, ret;

	if (iodev->dev->of_node) {
		ret = s2mpg11_pmic_dt_parse_pdata(iodev, pdata);
		if (ret)
			return ret;
	}

	if (!pdata) {
		dev_err(pdev->dev.parent, "Platform data not supplied\n");
		return -ENODEV;
	}

	s2mpg11 = devm_kzalloc(&pdev->dev, sizeof(struct s2mpg11_pmic),
			       GFP_KERNEL);
	if (!s2mpg11)
		return -ENOMEM;

	irq_base = pdata->irq_base;
	if (!irq_base) {
		dev_err(&pdev->dev, "Failed to get irq base %d\n", irq_base);
		return -ENODEV;
	}

	s2mpg11->rdev = devm_kzalloc(&pdev->dev,
				     sizeof(struct regulator_dev *) *
					     S2MPG11_REGULATOR_MAX,
				     GFP_KERNEL);
	s2mpg11->need_sync = devm_kzalloc(&pdev->dev,
					  sizeof(atomic_t) * S2MPG11_REGULATOR_MAX,
					  GFP_KERNEL);
	s2mpg11->turn_off_on_sync = devm_kzalloc(&pdev->dev,
						 sizeof(bool) * S2MPG11_REGULATOR_MAX,
						 GFP_KERNEL);
	s2mpg11->opmode =
		devm_kzalloc(&pdev->dev,
			     sizeof(unsigned int) * S2MPG11_REGULATOR_MAX,
			     GFP_KERNEL);
	s2mpg11->buck_ocp_irq = devm_kzalloc(&pdev->dev,
					     sizeof(int) * S2MPG11_BUCK_MAX,
					     GFP_KERNEL);

	s2mpg11->iodev = iodev;
	s2mpg11->i2c = iodev->pmic;

	mutex_init(&s2mpg11->lock);
	platform_set_drvdata(pdev, s2mpg11);

	/* setting for LDOs with exceptional register structure */

	for (i = 0; i < pdata->num_regulators; i++) {
		int id = pdata->regulators[i].id;

		config.dev = &pdev->dev;
		config.init_data = pdata->regulators[i].initdata;
		config.driver_data = s2mpg11;
		config.of_node = pdata->regulators[i].reg_node;
		s2mpg11->opmode[id] = regulators[id].enable_mask;

		s2mpg11->rdev[i] = regulator_register(&regulators[id], &config);
		if (s2m_is_enabled(s2mpg11->rdev[i]))
			atomic_set(&s2mpg11->need_sync[id], 1);
		else
			atomic_set(&s2mpg11->need_sync[id], 0);
		if (IS_ERR(s2mpg11->rdev[i])) {
			ret = PTR_ERR(s2mpg11->rdev[i]);
			dev_err(&pdev->dev, "regulator init failed for %d\n",
				i);
			s2mpg11->rdev[i] = NULL;
			goto err;
		}
	}

	s2mpg11->num_regulators = pdata->num_regulators;

	/* request IRQ */
	for (i = 0; i < S2MPG11_BUCK_MAX; i++) {
		s2mpg11->buck_ocp_irq[i] =
			irq_base + S2MPG11_IRQ_OCP_B1S_INT3 + i;

		ret = devm_request_threaded_irq(&pdev->dev,
						s2mpg11->buck_ocp_irq[i], NULL,
						s2mpg11_buck_ocp_irq, 0,
						"BUCK_OCP_IRQ", s2mpg11);
		if (ret < 0) {
			dev_err(&pdev->dev,
				"Failed to request BUCK[%d] OCP IRQ: %d: %d\n",
				i + 1, s2mpg11->buck_ocp_irq[i], ret);
		}
	}

	s2mpg11_ocp_detection_config(s2mpg11, pdata);
	s2mpg11_ocp_warn(s2mpg11, pdata);
	s2mpg11_oi_function(s2mpg11);

#if IS_ENABLED(CONFIG_DRV_SAMSUNG_PMIC)
	/* create sysfs */
	ret = create_s2mpg11_pmic_sysfs(s2mpg11);
	if (ret < 0)
		return ret;
#endif

	/* DCTRLSEL config for BUCK3S */
	s2mpg11_update_reg(s2mpg11->i2c, S2MPG11_PM_DCTRLSEL2,
			   DCTRLSEL_AP_ACTIVE_N, GENMASK(3, 0));

	return 0;
err:
	for (i = 0; i < S2MPG11_REGULATOR_MAX; i++)
		regulator_unregister(s2mpg11->rdev[i]);

	return ret;
}

static int s2mpg11_pmic_remove(struct platform_device *pdev)
{
	struct s2mpg11_pmic *s2mpg11 = platform_get_drvdata(pdev);
	int i;

	for (i = 0; i < S2MPG11_REGULATOR_MAX; i++)
		regulator_unregister(s2mpg11->rdev[i]);

#if IS_ENABLED(CONFIG_DRV_SAMSUNG_PMIC)
	pmic_device_destroy(s2mpg11->dev->devt);
#endif
	return 0;
}

static void s2mpg11_sync_state(struct s2mpg11_pmic *s2mpg11)
{
	int i;

	mutex_lock(&s2mpg11->lock);
	for (i = 0; i < s2mpg11->num_regulators; i++) {
		int need_sync;
		int id = rdev_get_id(s2mpg11->rdev[i]);

		need_sync = atomic_dec_return(&s2mpg11->need_sync[id]);
		if (need_sync < 0)
			continue;

		if (need_sync == 0 && s2mpg11->turn_off_on_sync[id])
			s2m_disable(s2mpg11->rdev[i]);
	}

	mutex_unlock(&s2mpg11->lock);
}


static void dev_sync_state(struct device *dev)
{
	struct s2mpg11_pmic *s2mpg11;

	s2mpg11 = platform_get_drvdata(to_platform_device(dev));
	s2mpg11_sync_state(s2mpg11);
}

static void s2mpg11_pmic_shutdown(struct platform_device *pdev)
{
}

#if IS_ENABLED(CONFIG_PM)
static int s2mpg11_pmic_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct s2mpg11_pmic *s2mpg11 = platform_get_drvdata(pdev);
	int ret;

	ret = s2mpg11_update_reg(s2mpg11->i2c, S2MPG11_PM_DCTRLSEL2,
				 DCTRLSEL_PWREN_MIF, GENMASK(3, 0));

	return ret;
}

static int s2mpg11_pmic_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct s2mpg11_pmic *s2mpg11 = platform_get_drvdata(pdev);
	int ret;

	ret = s2mpg11_update_reg(s2mpg11->i2c, S2MPG11_PM_DCTRLSEL2,
				 DCTRLSEL_AP_ACTIVE_N, GENMASK(3, 0));

	return ret;
}
#else
#define s2mpg11_pmic_suspend NULL
#define s2mpg11_pmic_resume NULL
#endif /* CONFIG_PM */

const struct dev_pm_ops s2mpg11_pmic_pm = {
	.suspend = s2mpg11_pmic_suspend,
	.resume = s2mpg11_pmic_resume,
};

static const struct platform_device_id s2mpg11_pmic_id[] = {
	{ "s2mpg11-regulator", 0 },
	{},
};

MODULE_DEVICE_TABLE(platform, s2mpg11_pmic_id);

static struct platform_driver s2mpg11_pmic_driver = {
	.driver = {
		   .name = "s2mpg11-regulator",
		   .owner = THIS_MODULE,
		   .sync_state = dev_sync_state,
#if IS_ENABLED(CONFIG_PM)
		   .pm = &s2mpg11_pmic_pm,
#endif
		   .suppress_bind_attrs = true,
		    },
	.probe = s2mpg11_pmic_probe,
	.remove = s2mpg11_pmic_remove,
	.shutdown = s2mpg11_pmic_shutdown,
	.id_table = s2mpg11_pmic_id,
};

static int __init s2mpg11_pmic_init(void)
{
	return platform_driver_register(&s2mpg11_pmic_driver);
}

subsys_initcall(s2mpg11_pmic_init);

static void __exit s2mpg11_pmic_exit(void)
{
	platform_driver_unregister(&s2mpg11_pmic_driver);
}

module_exit(s2mpg11_pmic_exit);

/* Module information */
MODULE_AUTHOR("Sangbeom Kim <sbkim73@samsung.com>");
MODULE_DESCRIPTION("SAMSUNG S2MPG11 Regulator Driver");
MODULE_LICENSE("GPL");
