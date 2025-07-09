// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2020, Google Inc
 *
 * MAX20339 OVP and LS driver
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": %s " fmt, __func__

#include <linux/gpio.h>
#include <linux/gpio/driver.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>
#include <linux/regmap.h>

#define MAX20339_STATUS1			0x1
#define MAX20339_STATUS1_VINVALID		BIT(5)
#define MAX20339_STATUS2			0x2
#define MAX20339_STATUS_SWITCH_CLOSED		BIT(0)
#define MAX20339_STATUS2_SWITCH_CLOSED_SHIFT	0
#define MAX20339_STATUS2_SWITCH_CLOSED_MASK	0x1

#define MAX20339_STATUS3			0x3
#define MAX20339_INT1				0x4
#define MAX20339_INT2				0x5
#define MAX20339_INT3				0x6
#define MAX20339_INTMASK1			0x7
#define MAX20339_OVLOSEL			0x11
#define MAX20339_OVLOSEL_INOVLOSEL_5_85		0x0
#define MAX20339_OVLOSEL_INOVLOSEL_14_5		0x2
#define MAX20339_OVLOSEL_INOVLOSEL_MASK		0x3
#define MAX20339_IN_CTR_REG			0x10
#define MAX20339_IN_CTR_SWEN_SHIFT		0
#define MAX20339_IN_CTR_SWEN_MASK		GENMASK(1, 0)
#define MAX20339_IN_CTR_SWEN_FORCE_ON		0x3
#define MAX20339_IN_CTR_SWEN_FORCE_OFF		0x0

#define MAX20339_POLL_ATTEMPTS			10
#define MAX20339_INT2_REG			0x5
#define MAX20339_INT2_LSW1CLOSEDI		(1 << 0)
#define MAX20339_INT3_REG			0x6
#define MAX20339_INT3_LSW2CLOSEDI		(1 << 0)

#define MAX20339_SW_CNTL_REG			0xA
#define MAX20339_SW_CNTL_LSW1_EN_SHIFT		0
#define MAX20339_SW_CNTL_LSW1_EN_MASK		0x1
#define MAX20339_SW_CNTL_LSW1_OV_SHIFT		1
#define MAX20338_SW_CNTL_LSW1_OV_EN_MASK	0x2
#define MAX20339_SW_CNTL_LSW2_EN_SHIFT		4
#define MAX20339_SW_CNTL_LSW2_EN_MASK		0x10
#define MAX20338_SW_CNTL_LSW2_OV_EN_SHIFT	5
#define MAX20338_SW_CNTL_LSW2_OV_EN_MASK	0x20


#define MAX20339_MIN_GPIO			0
#define MAX20339_MAX_GPIO			8
#define MAX20339_NUM_GPIOS			8
#define MAX20339_LSW1_OFF			0
#define MAX20339_LSW2_OFF			1
#define MAX20339_LSW1_STATUS_OFF		2
#define MAX20339_VIN_VALID_OFF			3
#define MAX20339_IN_CTR_SWEN_OFF		4
#define MAX20339_LSW1_IS_OPEN_OFF		5
#define MAX20339_LSW1_IS_CLOSED_OFF		6
#define MAX20339_OTG_ENA_OFF			7

#define MAX20339_LSW1_TIMEOUT_MS		100
#define MAX20339_VIN_VALID_TIMEOUT_MS		100

struct max20339_ovp {
	struct i2c_client *client;
	struct regmap *regmap;
	wait_queue_head_t gpio_get_wq;
#if IS_ENABLED(CONFIG_GPIOLIB)
	struct gpio_chip gpio;
#endif
	int irq_gpio;
};

static const struct regmap_range max20339_ovp_range[] = {
	regmap_reg_range(0x0, 0x2f)
};

const struct regmap_access_table max20339_ovp_write_table = {
	.yes_ranges = max20339_ovp_range,
	.n_yes_ranges = ARRAY_SIZE(max20339_ovp_range),
};

static const struct regmap_config max20339_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0x2f,
	.wr_table = &max20339_ovp_write_table,
};

static irqreturn_t max20339_irq(int irqno, void *data)
{
	struct max20339_ovp *ovp = data;
	struct device *dev;
	u8 buf[6];
	int ret;

	/* not really possible now */
	if (!ovp)
		return IRQ_NONE;

	wake_up_all(&ovp->gpio_get_wq);

	/* TODO: check the actual status and return IRQ_NONE if none is set */

	dev = &ovp->client->dev;
	ret = regmap_bulk_read(ovp->regmap, MAX20339_STATUS1, buf, ARRAY_SIZE(buf));
	if (!ret)
		dev_info(dev,
			 "OVP TRIGGERED: STATUS1:%#x STATUS2:%#x STATUS3:%#x INT1:%#x INT2:%#x INT3:%#x\n",
			 buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);
	else
		dev_err(dev, "OVP TRIGGERED: Failed on reading status:%d\n", ret);

	return IRQ_HANDLED;
}

static int max20339_init_regs(struct regmap *regmap, struct device *dev)
{
	int ret;
	unsigned int val;
	const u8 irq_mask[] = {0xff, 0xff, 0xff};

	ret = regmap_read(regmap, MAX20339_OVLOSEL, &val);
	if (ret < 0) {
		dev_err(dev, "OVLSEL read error: ret %d\n", ret);
		return ret;
	}

	dev_info(dev, "OVLOSEL default: %#x\n", val);

	ret = regmap_write(regmap, MAX20339_OVLOSEL,
			 MAX20339_OVLOSEL_INOVLOSEL_14_5);
	if (ret < 0) {
		dev_err(dev, "OVLSEL write error: ret %d\n", ret);
		return ret;
	}

	ret = regmap_read(regmap, MAX20339_IN_CTR_REG, &val);
	if (ret < 0) {
		dev_err(dev, "IN_CTR read error: ret %d\n", ret);
		return ret;
	}

	dev_info(dev, "IN_CTR default: %#x\n", val);

	/* Disable & enable to make OVLOSEL reflect */
	ret = regmap_write(regmap, MAX20339_IN_CTR_REG, 0);
	if (ret < 0) {
		dev_err(dev, "IN_CTR write error: ret %d\n", ret);
		return ret;
	}

	/* Enable Force on while re-enabling the switch */
	ret = regmap_write(regmap, MAX20339_IN_CTR_REG, val | MAX20339_IN_CTR_SWEN_FORCE_ON);
	if (ret < 0) {
		dev_err(dev, "IN_CTR write error: ret %d\n", ret);
		return ret;
	}

	ret = regmap_bulk_write(regmap, MAX20339_INTMASK1, irq_mask, ARRAY_SIZE(irq_mask));
	if (ret < 0) {
		dev_err(dev, "INTMASK1-3 enable failed: ret %d\n", ret);
		return ret;
	}

	return ret;
}

#if IS_ENABLED(CONFIG_GPIOLIB)
static int max20339_gpio_get_direction(struct gpio_chip *chip,
				       unsigned int offset)
{
	return GPIOF_DIR_OUT;
}

static bool max20339_is_lsw_closed(struct max20339_ovp *ovp, int offset)
{
	int ret;
	unsigned int val = 0;

	ret = regmap_read(ovp->regmap, offset ==  MAX20339_LSW1_OFF ?
			  MAX20339_STATUS2 : MAX20339_STATUS3, &val);
	if (ret < 0)
		return false;

	return (val & MAX20339_STATUS_SWITCH_CLOSED) != 0;
}

/* 1 same as state, 0 not same */
static int max20339_test_lsw1_state(struct max20339_ovp *ovp, int state)
{
	const int poll_interval_ms = 20;
	int ret = -ETIMEDOUT;
	unsigned int val;
	bool closed;
	int i;

	ret = regmap_read(ovp->regmap, MAX20339_STATUS2, &val);
	if (ret < 0)
		return ret;

	closed = (val & MAX20339_STATUS_SWITCH_CLOSED) != 0;
	if (closed == state)
		return 1;

	/* wait_event_timeout() timeout is not reliable */
	for (i = 0; i <= MAX20339_LSW1_TIMEOUT_MS; i += poll_interval_ms) {
		if (max20339_is_lsw_closed(ovp, MAX20339_LSW1_OFF) == state) {
			ret = 0;
			break;
		}

		mdelay(poll_interval_ms);
	}

	if (!ret)
		dev_warn(&ovp->client->dev, "Timeout for lsw1==%d\n", state);

	return max20339_is_lsw_closed(ovp, MAX20339_LSW1_OFF) == state;
}

static bool max20339_is_vin_valid(struct max20339_ovp *ovp)
{
	int ret;
	unsigned int val;

	ret = regmap_read(ovp->regmap, MAX20339_STATUS1, &val);
	if (ret < 0)
		return false;

	return (val & MAX20339_STATUS_SWITCH_CLOSED) != 0;
}

static int max20339_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	struct max20339_ovp *ovp = gpiochip_get_data(chip);
	unsigned int val;
	u8 mask, shift;
	int ret, reg;

	switch (offset) {
	case MAX20339_LSW1_OFF:
		mask = MAX20339_SW_CNTL_LSW1_EN_MASK;
		shift = MAX20339_SW_CNTL_LSW1_EN_SHIFT;
		reg = MAX20339_SW_CNTL_REG;
		break;
	case MAX20339_LSW2_OFF:
		mask = MAX20339_SW_CNTL_LSW2_EN_MASK;
		shift = MAX20339_SW_CNTL_LSW2_EN_SHIFT;
		reg = MAX20339_SW_CNTL_REG;
		break;
	case MAX20339_LSW1_STATUS_OFF:
		mask = MAX20339_STATUS2_SWITCH_CLOSED_MASK;
		shift = MAX20339_STATUS2_SWITCH_CLOSED_SHIFT;
		reg = MAX20339_STATUS2;
		break;
	case MAX20339_VIN_VALID_OFF:
		reg = MAX20339_STATUS1;
		break;
	case MAX20339_LSW1_IS_OPEN_OFF:
		return max20339_test_lsw1_state(ovp, 0);
	case MAX20339_LSW1_IS_CLOSED_OFF:
		return max20339_test_lsw1_state(ovp, 1);
	default:
		return -EINVAL;
	}

	ret = regmap_read(ovp->regmap, reg, &val);
	if (ret < 0) {
		dev_err(&ovp->client->dev, "%x read error: ret %d\n", reg, ret);
		return ret;
	}

	/* only checked for valid */
       if (offset == MAX20339_VIN_VALID_OFF) {

		if (val & MAX20339_STATUS1_VINVALID)
			return 1;

		wait_event_timeout(ovp->gpio_get_wq, max20339_is_vin_valid(ovp),
				   msecs_to_jiffies(MAX20339_VIN_VALID_TIMEOUT_MS));
		return max20339_is_vin_valid(ovp) ? 1 : 0;
	}

	return (val & mask) >> shift;
}

static void max20339_gpio_set(struct gpio_chip *chip,
			      unsigned int offset, int value)
{
	int ret;
	unsigned int tmp;
	bool change;
	u8 mask;
	u8 status_reg; /* status register to poll for update */
	u8 sw_cntl_reg;
	int i;
	struct max20339_ovp *ovp = gpiochip_get_data(chip);

	dev_dbg(&ovp->client->dev, "%s off=%u val=%d", __func__, offset, value);
	switch (offset) {
	case MAX20339_LSW1_OFF:
		sw_cntl_reg = MAX20339_SW_CNTL_REG;
		mask = MAX20339_SW_CNTL_LSW1_EN_MASK;
		status_reg = MAX20339_STATUS2;
		tmp = (!!value << MAX20339_SW_CNTL_LSW1_EN_SHIFT);
		break;
	case MAX20339_LSW2_OFF:
		sw_cntl_reg = MAX20339_SW_CNTL_REG;
		mask = MAX20339_SW_CNTL_LSW2_EN_MASK;
		status_reg = MAX20339_STATUS3;
		tmp = (!!value << MAX20339_SW_CNTL_LSW2_EN_SHIFT);
		break;
	case MAX20339_IN_CTR_SWEN_OFF:
		sw_cntl_reg = MAX20339_IN_CTR_REG;
		mask = MAX20339_IN_CTR_SWEN_MASK;
		status_reg = MAX20339_STATUS1;
		tmp = value ? MAX20339_IN_CTR_SWEN_FORCE_ON : MAX20339_IN_CTR_SWEN_FORCE_OFF;
		break;

	/* b/178458456 clear/reset INOVLO on enter/exit from OTG cases */
	case MAX20339_OTG_ENA_OFF:
		tmp = value ? MAX20339_OVLOSEL_INOVLOSEL_5_85 :
		      MAX20339_OVLOSEL_INOVLOSEL_14_5;
		ret = regmap_update_bits(ovp->regmap, MAX20339_OVLOSEL,
					 MAX20339_OVLOSEL_INOVLOSEL_MASK,
					 tmp);
		if (ret < 0)
			dev_err(&ovp->client->dev, "OVLOSEL update error: ret %d\n", ret);
		return;
	default:
		return;
	}

	ret = regmap_update_bits_base(ovp->regmap, sw_cntl_reg, mask,  tmp,
				      &change, false, false);
	if (ret < 0)
		dev_err(&ovp->client->dev, "SW_CNTL update error: ret %d\n", ret);

	/* poll until update seen */
	for (i = 0; i < MAX20339_POLL_ATTEMPTS; i++) {
		ret = regmap_read(ovp->regmap, status_reg, &tmp);
		if ((tmp & MAX20339_STATUS_SWITCH_CLOSED) == value)
			break;
		mdelay(20);
	}

}
#endif

/* HACK: will make max77729_pmic an interrupt controller and use the irq */
static int max20339_setup_irq(struct max20339_ovp *ovp)
{
	struct device *dev = &ovp->client->dev;
	int ret = -EINVAL;

	ovp->irq_gpio = of_get_named_gpio(dev->of_node, "max20339,irq-gpio", 0);
	if (ovp->irq_gpio < 0) {
		dev_err(dev, "failed to get irq-gpio (%d)\n", ovp->irq_gpio);
	} else {
		const int irq = gpio_to_irq(ovp->irq_gpio);

		ret = devm_request_threaded_irq(dev, irq, NULL,
						max20339_irq,
						IRQF_TRIGGER_FALLING |
						IRQF_SHARED |
						IRQF_ONESHOT,
						"max2339_ovp",
						ovp);

		dev_err(dev, "ovp->irq_gpio=%d found irq=%d registered %d\n",
			ovp->irq_gpio, irq, ret);
	}

	/* Read to clear interrupts */
	max20339_irq(-1, ovp);

	return ret;
}

static int max20339_probe(struct i2c_client *client,
			  const struct i2c_device_id *i2c_id)
{
	struct max20339_ovp *ovp;
	int rc, ret = 0;

	ovp = devm_kzalloc(&client->dev, sizeof(*ovp), GFP_KERNEL);
	if (!ovp)
		return -ENOMEM;

	ovp->client = client;
	ovp->regmap = devm_regmap_init_i2c(client, &max20339_regmap_config);
	if (IS_ERR(ovp->regmap)) {
		dev_err(&client->dev, "Regmap init failed\n");
		return PTR_ERR(ovp->regmap);
	}

	max20339_init_regs(ovp->regmap, &client->dev);
	i2c_set_clientdata(client, ovp);
	init_waitqueue_head(&ovp->gpio_get_wq);

#if IS_ENABLED(CONFIG_GPIOLIB)
	/* Setup GPIO controller */
	ovp->gpio.owner = THIS_MODULE;
	ovp->gpio.parent = &client->dev;
	ovp->gpio.label = "max20339_gpio";
	ovp->gpio.get_direction = max20339_gpio_get_direction;
	ovp->gpio.get = max20339_gpio_get;
	ovp->gpio.set = max20339_gpio_set;
	ovp->gpio.base = -1;
	ovp->gpio.ngpio = MAX20339_NUM_GPIOS;
	ovp->gpio.can_sleep = true;
	ovp->gpio.of_node = of_find_node_by_name(client->dev.of_node,
						 ovp->gpio.label);
	if (!ovp->gpio.of_node)
		dev_err(&client->dev, "Failed to find %s DT node\n",
			ovp->gpio.label);

	ret = devm_gpiochip_add_data(&client->dev, &ovp->gpio, ovp);
	if (ret)
		dev_err(&client->dev, "Failed to initialize gpio chip\n");
#endif

	rc = max20339_setup_irq(ovp);
	if (rc < 0)
		dev_err(&client->dev, "Init IRQ failed (%d)\n", rc);

	return ret;
}

static const struct i2c_device_id max20339_id[] = {
	{ "max20339ovp", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max20339_id);

#ifdef CONFIG_OF
static const struct of_device_id max20339_of_match[] = {
	{ .compatible = "max20339ovp", },
	{},
};
MODULE_DEVICE_TABLE(of, max20339_of_match);
#endif

static struct i2c_driver max20339_i2c_driver = {
	.driver = {
		.name = "max20339ovp",
		.of_match_table = of_match_ptr(max20339_of_match),
	},
	.probe = max20339_probe,
	.id_table = max20339_id,
};
module_i2c_driver(max20339_i2c_driver);

MODULE_DESCRIPTION("Maxim 20339 OVP driver");
MODULE_AUTHOR("Badhri Jagan Sridharan <badhri@google.com>");
MODULE_LICENSE("GPL");
