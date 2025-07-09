// SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 This file is part of VL53L1 and is dual licensed,
 either GPL-2.0+
 or 'BSD 3-clause "New" or "Revised" License' , at your option.
 ******************************************************************************
 */

/**
 * @file stmvl53l1_module-i2c.c
 *
 *  implement STM VL53L1 module interface i2c wrapper + control
 *  using linux native i2c + gpio + reg api
 */
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/miscdevice.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/time.h>
#include <linux/platform_device.h>
#include <linux/version.h>

/*
 * power specific includes
 */
#include <linux/pwm.h>
#include <linux/regulator/consumer.h>
#include <linux/pinctrl/consumer.h>
#include <linux/clk.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>

#include "stmvl53l1-i2c.h"
#include "stmvl53l1.h"

#define STMVL53L1_SLAVE_ADDR (0x52 >> 1)

#define VIO_VOLTAGE_MIN_DEFAULT 1800000
#define VIO_VOLTAGE_MAX_DEFAULT 1800000

#define VDD_VOLTAGE_MIN 2800000
#define VDD_VOLTAGE_MAX 2800000


static struct shared_i2c_data *shared_i2c_data;
static DEFINE_MUTEX(shared_i2c_mutex);

bool is_shared_i2c_with_stmvl53l1(struct pinctrl *pinctrl)
{
	/* The global shared_i2c_data can be NULL when allocate memory failed.
	 * And shared_i2c_data->pinctrl will be NULL if no stmvl53l1 module in
	 * device tree or stmvl53l1 probes failed. We don't need to care about
	 * the shared i2c in these two cases.
	 */
	if (shared_i2c_data == NULL || shared_i2c_data->pinctrl == NULL || pinctrl == NULL)
		return false;

	return (shared_i2c_data->pinctrl == pinctrl);
}
EXPORT_SYMBOL_GPL(is_shared_i2c_with_stmvl53l1);

void shared_i2c_data_release(struct kref *ref)
{
	struct shared_i2c_data *data =
		container_of(ref, struct shared_i2c_data, refcount);
	kfree(data);
}

int shared_i2c_set_state(struct device *dev, struct pinctrl *pinctrl,
			 const char *state_str)
{
	struct pinctrl_state *state;
	int ret;

	state = pinctrl_lookup_state(pinctrl, state_str);
	if (IS_ERR(state)) {
		return -EINVAL;
	}

	mutex_lock(&shared_i2c_mutex);
	if (strcmp(state_str, "on_i2c") == 0)
		kref_get(&shared_i2c_data->refcount);

	if (strcmp(state_str, "off_i2c") == 0) {
		kref_put(&shared_i2c_data->refcount, shared_i2c_data_release);
		if (kref_read(&shared_i2c_data->refcount) != 1) {
			mutex_unlock(&shared_i2c_mutex);
			return 0;
		}
	}

	ret = pinctrl_select_state(pinctrl, state);
	mutex_unlock(&shared_i2c_mutex);

	if (ret) {
		dev_err(dev, "Error selecting state %s (%d)\n",
			 state_str, ret);
		return ret;
	}

	dev_info(dev, "Shared i2c pinctrl state %s\n", state_str);
	return 0;
}
EXPORT_SYMBOL_GPL(shared_i2c_set_state);

/*
 * mutex to handle device i2c address changes. It allow to avoid multiple
 * device active with same i2c addresses at the same time. Note that we don't
 * support case where boot_reg has the same value as a final i2c address of
 * another device.
 */
static DEFINE_MUTEX(dev_addr_change_mutex);

/**
 * i2c client assigned to our driver
 *
 * this is use for stm test purpose as we fake client create and regstration
 * we stores the i2c client for release in clean-up overwise we wan't reload
 * the module multiple time
 *
 * in a normal dev tree prod system this is not required
 */
static struct i2c_client *stm_test_i2c_client;

/**
 * warn message
 *
 * @warning use only in scope where i2c_data ptr is present
 **/
#define modi2c_warn(fmt, ...) \
	dev_WARN(&i2c_data->client->dev, fmt, ##__VA_ARGS__)

/**
 * err message
 *
 * @warning use only in scope where i2c_data ptr is present
 */
#define modi2c_err(fmt, ...) \
	dev_err(&i2c_data->client->dev, fmt, ##__VA_ARGS__)

static int get_xsdn(struct device *dev, struct i2c_data *i2c_data)
{
	int rc = 0;

	i2c_data->io_flag.xsdn_owned = 0;
	if (i2c_data->xsdn_gpio == -1) {
		dev_err(dev, "reset gpio is required\n");
		rc = -ENODEV;
		goto no_gpio;
	}

	dev_dbg(dev, "request xsdn_gpio %d\n", i2c_data->xsdn_gpio);
	rc = gpio_request(i2c_data->xsdn_gpio, "vl53l1_xsdn");
	if (rc) {
		dev_err(dev, "fail to acquire xsdn %d\n", rc);
		goto request_failed;
	}

	rc = gpio_direction_output(i2c_data->xsdn_gpio, 0);
	if (rc) {
		dev_err(dev, "fail to configure xsdn as output %d\n", rc);
		goto direction_failed;
	}
	i2c_data->io_flag.xsdn_owned = 1;

	return rc;

direction_failed:
	gpio_free(i2c_data->xsdn_gpio);

request_failed:
no_gpio:
	return rc;
}

static void put_xsdn(struct i2c_data *i2c_data)
{
	struct device *dev = &i2c_data->client->dev;

	if (i2c_data->io_flag.xsdn_owned) {
		dev_dbg(dev, "release xsdn_gpio %d\n", i2c_data->xsdn_gpio);
		gpio_free(i2c_data->xsdn_gpio);
		i2c_data->io_flag.xsdn_owned = 0;
	}
	i2c_data->xsdn_gpio = -1;
}

static int get_pwren(struct device *dev, struct i2c_data *i2c_data)
{
	int rc = 0;

	i2c_data->io_flag.pwr_owned = 0;
	if (i2c_data->pwren_gpio == -1) {
		dev_warn(dev, "pwren gpio disable\n");
		goto no_gpio;
	}

	dev_dbg(dev, "request pwren_gpio %d\n",
		i2c_data->pwren_gpio);
	rc = gpio_request(i2c_data->pwren_gpio, "vl53l1_pwren");
	if (rc) {
		dev_err(dev, "fail to acquire pwren %d\n", rc);
		goto request_failed;
	}

	rc = gpio_direction_output(i2c_data->pwren_gpio, 0);
	if (rc) {
		dev_err(dev, "fail to configure pwren as output %d\n", rc);
		goto direction_failed;
	}
	i2c_data->io_flag.pwr_owned = 1;

	return rc;

direction_failed:
	gpio_free(i2c_data->xsdn_gpio);

request_failed:
no_gpio:
	return rc;
}

static void put_pwren(struct i2c_data *i2c_data)
{
	struct device *dev = &i2c_data->client->dev;

	if (i2c_data->io_flag.pwr_owned) {
		dev_dbg(dev, "release pwren_gpio %d\n", i2c_data->pwren_gpio);
		gpio_free(i2c_data->pwren_gpio);
		i2c_data->io_flag.pwr_owned = 0;
	}
	i2c_data->pwren_gpio = -1;
}

static int get_intr(struct device *dev, struct i2c_data *i2c_data)
{
	int rc = 0;
	struct pinctrl *pinctrl;

	if (of_property_count_strings(dev->of_node, "pinctrl-names") <= 0) {
		dev_err(dev, "no pinctrl defined\n");
	} else {
		pinctrl = devm_pinctrl_get(dev);
		stmvl53l1_pinctrl_set_state(dev, pinctrl, "default");
	}

	i2c_data->io_flag.intr_owned = 0;
	if (i2c_data->intr_gpio == -1) {
		dev_warn(dev, "no interrupt gpio\n");
		goto no_gpio;
	}

	dev_dbg(dev, "request intr_gpio %d\n", i2c_data->intr_gpio);
	rc = gpio_request(i2c_data->intr_gpio, "vl53l1_intr");
	if (rc) {
		dev_err(dev, "fail to acquire intr %d\n", rc);
		goto request_failed;
	}

	rc = gpio_direction_input(i2c_data->intr_gpio);
	if (rc) {
		dev_err(dev, "fail to configure intr as input %d\n", rc);
		goto direction_failed;
	}

	i2c_data->irq = gpio_to_irq(i2c_data->intr_gpio);
	if (i2c_data->irq < 0) {
		dev_err(dev, "fail to map GPIO: %d to interrupt:%d\n",
			i2c_data->intr_gpio, i2c_data->irq);
		goto irq_failed;
	}
	i2c_data->io_flag.intr_owned = 1;

	return rc;

irq_failed:
direction_failed:
	gpio_free(i2c_data->intr_gpio);

request_failed:
no_gpio:
	return rc;
}

static void put_intr(struct i2c_data *i2c_data)
{
	struct device *dev = &i2c_data->client->dev;

	if (i2c_data->io_flag.intr_owned) {
		if (i2c_data->io_flag.intr_started) {
			free_irq(i2c_data->irq, i2c_data);
			i2c_data->io_flag.intr_started = 0;
		}
		dev_dbg(dev, "release intr_gpio %d\n",
			i2c_data->intr_gpio);
		gpio_free(i2c_data->intr_gpio);
		i2c_data->io_flag.intr_owned = 0;
	}
	i2c_data->intr_gpio = -1;
}

static int get_vio(struct device *dev, struct i2c_data *i2c_data)
{
	int rc = 0;

	i2c_data->io_flag.vio_owned = 0;
	if (i2c_data->vio_gpio == -1) {
		dev_warn(dev, "vio gpio disable\n");
		goto no_gpio;
	}

	dev_dbg(dev, "request vio_gpio %d\n", i2c_data->vio_gpio);
	rc = gpio_request(i2c_data->vio_gpio, "vl53l1_vio");
	if (rc) {
		dev_err(dev, "fail to acquire vio %d\n", rc);
		goto request_failed;
	}

	rc = gpio_direction_output(i2c_data->vio_gpio, 0);
	if (rc) {
		dev_err(dev, "fail to configure vio as output %d\n", rc);
		goto direction_failed;
	}
	i2c_data->io_flag.vio_owned = 1;

	return rc;

direction_failed:
	gpio_free(i2c_data->vio_gpio);

request_failed:
no_gpio:
	return rc;
}

static void put_vio(struct i2c_data *i2c_data)
{
	struct device *dev = &i2c_data->client->dev;

	if (i2c_data->io_flag.vio_owned) {
		dev_dbg(dev, "release vio_gpio %d\n",
			i2c_data->vio_gpio);
		gpio_free(i2c_data->vio_gpio);
		i2c_data->io_flag.vio_owned = 0;
	}
}

/**
 *  parse dev tree for all platform specific input
 */
static int stmvl53l1_parse_tree(struct device *dev, struct i2c_data *i2c_data)
{
	int rc = 0;
	int pwren_gpio, xsdn_gpio, intr_gpio, vio_gpio;
	int voltage;
	const char *vio_type;

	i2c_data->vio = NULL;
	i2c_data->vio_gpio = -1;
	i2c_data->pwren_supply = NULL;

	/* vio power: vio-type is either gpio or regulator */
	rc = of_property_read_string(dev->of_node, "vio-type", &vio_type);
	if (!rc) {
		if (strcmp(vio_type, "gpio") == 0) {
			vio_gpio = of_get_named_gpio(dev->of_node, "vio-gpio", 0);

			if (vio_gpio > 0) {
				if (!gpio_is_valid(vio_gpio)) {
					dev_err(dev, "Invalid chip select pin vio\n");
					return -EPERM;
				}
				i2c_data->vio_gpio = vio_gpio;
			} else {
				i2c_data->vio_gpio = -1;
				dev_warn(dev, "Unable to find vio-gpio %d\n", vio_gpio);
			}
		} else if (strcmp(vio_type, "regulator") == 0) {
			i2c_data->vio = regulator_get_optional(dev, "vio");
			if (IS_ERR(i2c_data->vio) || i2c_data->vio == NULL) {
				i2c_data->vio = NULL;
				dev_err(dev, "Unable to get vio regulator\n");
			}

			if (of_property_read_u32(dev->of_node, "vio-voltage",
						 &voltage))
				i2c_data->vio_voltage = VIO_VOLTAGE_MIN_DEFAULT;
			else
				i2c_data->vio_voltage = voltage;
		}
	} else {
		dev_info(dev, "no vio-type in device tree\n");
	}

	/* power : AVDD by LDAF_en pin */
	pwren_gpio = of_get_named_gpio(dev->of_node, "pwren-gpio", 0);

	if (pwren_gpio > 0) {
		if (!gpio_is_valid(pwren_gpio)) {
			dev_err(dev, "Invalid chip select pin pwren\n");
			return -EPERM;
		}
		i2c_data->pwren_gpio = pwren_gpio;
	} else {
		i2c_data->pwren_gpio = -1;
		dev_warn(dev, "Unable to find pwren-gpio %d\n", pwren_gpio);
		/* try to find pwren-supply */
		i2c_data->pwren_supply = regulator_get_optional(dev, "pwren");
		if (IS_ERR(i2c_data->pwren_supply)) {
			i2c_data->pwren_supply = NULL;
			dev_err(dev, "neither pwren-gpio nor pwren_supply\n");
			return -EPERM;
		}
	}

	/* reset pin */
	xsdn_gpio = of_get_named_gpio(dev->of_node, "xsdn-gpio", 0);

	if (xsdn_gpio > 0) {
		if (!gpio_is_valid(xsdn_gpio)) {
			dev_err(dev, "Invalid chip select pin xsdn\n");
			return -EPERM;
		}
		i2c_data->xsdn_gpio = xsdn_gpio;
	} else {
		i2c_data->xsdn_gpio = -1;
		dev_warn(dev, "Unable to find xsdn-gpio %d\n", xsdn_gpio);
	}

	/* irq pin */
	intr_gpio = of_get_named_gpio(dev->of_node, "intr-gpio", 0);

	if (intr_gpio > 0) {
		if (!gpio_is_valid(intr_gpio)) {
			dev_err(dev, "Invalid chip select pin intr\n");
			return -EPERM;
		}
		i2c_data->intr_gpio = intr_gpio;
	} else {
		i2c_data->intr_gpio = -1;
		dev_warn(dev, "Unable to find intr-gpio %d\n", intr_gpio);
	}

	i2c_data->boot_reg = STMVL53L1_SLAVE_ADDR;

	/* configure gpios */
	rc = get_xsdn(dev, i2c_data);
	if (rc)
		goto no_xsdn;

	if (!i2c_data->pwren_supply) {
		rc = get_pwren(dev, i2c_data);
		if (rc)
			goto no_pwren;
	}

	rc = get_intr(dev, i2c_data);
	if (rc)
		goto no_intr;

	/* configure pinctrl */
	shared_i2c_data->pinctrl = devm_pinctrl_get(dev->parent->parent);
	if (IS_ERR(shared_i2c_data->pinctrl)) {
		dev_info(dev, "Unable to config shared_i2c_data->pinctrl\n");
		shared_i2c_data->pinctrl = NULL;
	}

	return rc;

no_intr:
	if (i2c_data->pwren_gpio != -1) {
		put_pwren(i2c_data);
	}
	if (i2c_data->pwren_supply) {
		regulator_put(i2c_data->pwren_supply);
		i2c_data->pwren_supply = NULL;
	}
no_pwren:
	put_xsdn(i2c_data);
no_xsdn:
	if (i2c_data->vio) {
		regulator_put(i2c_data->vio);
		i2c_data->vio = NULL;
	}
	return rc;
}

static void stmvl53l1_release_gpios(struct i2c_data *i2c_data)
{
	put_xsdn(i2c_data);
	if (i2c_data->pwren_gpio != -1) {
		put_pwren(i2c_data);
	}
	if (i2c_data->pwren_supply) {
		regulator_put(i2c_data->pwren_supply);
		i2c_data->pwren_supply = NULL;
	}
	put_intr(i2c_data);
	if (i2c_data->vio_gpio != -1) {
		put_vio(i2c_data);
		i2c_data->vio_gpio = -1;
	}
	if (i2c_data->vio) {
		regulator_put(i2c_data->vio);
		i2c_data->vio = NULL;
	}
}

static int stmvl53l1_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int rc = 0;
	struct stmvl53l1_data *vl53l1_data = NULL;
	struct i2c_data *i2c_data = NULL;

	dev_info(&client->dev, "%s : 0x%02x\n",client->name, client->addr);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE)) {
		rc = -EIO;
		return rc;
	}

	vl53l1_data = kzalloc(sizeof(struct stmvl53l1_data), GFP_KERNEL);
	if (!vl53l1_data) {
		rc = -ENOMEM;
		return rc;
	}
	if (vl53l1_data) {
		vl53l1_data->client_object =
				kzalloc(sizeof(struct i2c_data), GFP_KERNEL);
		if (!vl53l1_data)
			goto done_freemem;
		i2c_data = (struct i2c_data *)vl53l1_data->client_object;
	}
	i2c_data->client = client;
	i2c_data->vl53l1_data = vl53l1_data;
	i2c_data->irq = -1; /* init to no irq */

	shared_i2c_data = kzalloc(sizeof(struct shared_i2c_data), GFP_KERNEL);
	if (!shared_i2c_data)
		goto done_freemem;
	kref_init(&shared_i2c_data->refcount);

	/* parse and configure hardware */
	rc = stmvl53l1_parse_tree(&i2c_data->client->dev, i2c_data);
	if (rc)
		goto parse_fail;

	/* setup client data */
	i2c_set_clientdata(client, vl53l1_data);

	/* end up by core driver setup */
	rc = stmvl53l1_setup(vl53l1_data);
	if (rc)
		goto release_gpios;

	kref_init(&i2c_data->ref);

	return rc;

release_gpios:
	stmvl53l1_release_gpios(i2c_data);
parse_fail:
	/* Set pinctrl NULL either parse tree failed or probe failed. */
	shared_i2c_data->pinctrl = NULL;
done_freemem:
	/* kfree safe against NULL */
	kfree(vl53l1_data);
	kfree(i2c_data);
	return -EPERM;
}

static void stmvl53l1_remove(struct i2c_client *client)
{
	struct stmvl53l1_data *data = i2c_get_clientdata(client);
	struct i2c_data *i2c_data = (struct i2c_data *)data->client_object;

	mutex_lock(&data->work_mutex);
	/* main driver cleanup */
	stmvl53l1_cleanup(data);

	/* release gpios */
	stmvl53l1_release_gpios(i2c_data);

	mutex_unlock(&data->work_mutex);

	stmvl53l1_put(data->client_object);
	kref_put(&shared_i2c_data->refcount, shared_i2c_data_release);
}

#ifdef CONFIG_PM_SLEEP
static int stmvl53l1_suspend(struct device *dev)
{
	struct stmvl53l1_data *data = i2c_get_clientdata(to_i2c_client(dev));

	mutex_lock(&data->work_mutex);
	/* Stop ranging */
	stmvl53l1_pm_suspend_stop(data);

	mutex_unlock(&data->work_mutex);

	return 0;
}

static int stmvl53l1_resume(struct device *dev)
{
	/* do nothing user will restart measurements */
	return 0;
}
#endif


static SIMPLE_DEV_PM_OPS(stmvl53l1_pm_ops, stmvl53l1_suspend, stmvl53l1_resume);

static const struct i2c_device_id stmvl53l1_id[] = {
	{STMVL53L1_DRV_NAME, 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, stmvl53l1_id);

static const struct of_device_id st_stmvl53l1_dt_match[] = {
	{.compatible = "st,"STMVL53L1_DRV_NAME,},
	{},
};

static struct i2c_driver stmvl53l1_driver = {
	.driver = {
		.name	= STMVL53L1_DRV_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = st_stmvl53l1_dt_match,
		.pm	= &stmvl53l1_pm_ops,
	},
	.probe	= stmvl53l1_probe,
	.remove	= stmvl53l1_remove,
	.id_table = stmvl53l1_id,

};

/**
 * give power to device
 *
 * @param object  the i2c layer object
 * @return
 */
int stmvl53l1_power_up_i2c(void *object)
{
	int rc = 0;
	struct i2c_data *data = (struct i2c_data *)object;
	struct device *dev = &data->client->dev;

	if (data->vl53l1_data->is_power_up)
		return rc;

	/* turn on power */
	if (data->vio_gpio != -1) {
		rc = get_vio(dev, data);
		if (rc) {
			/* This GPIO is shared with camera and suppose camera
			 * has already pulled up its power. Reset rc as well.
			 */
			dev_warn(dev, "request vio gpio failed\n");
			rc = 0;
		} else {
			rc = gpiod_direction_output(
				gpio_to_desc(data->vio_gpio), 1);
			if (rc) {
				dev_err(dev, "fail to set vio high %d\n", rc);
				return rc;
			}
		}
	} else if (data->vio != NULL) {
		rc = regulator_set_voltage(data->vio, data->vio_voltage,
					data->vio_voltage);
		if (rc < 0) {
			dev_err(dev, "set vio voltage failed\n");
			return rc;
		}
		rc = regulator_enable(data->vio);
		if (rc) {
			dev_err(dev, "fail to turn on vio regulator\n");
			return rc;
		}
	}

	if (data->pwren_gpio != -1) {
		gpio_set_value(data->pwren_gpio, 1);
		dev_info(dev, "slow power on\n");
	} else
		dev_warn(dev, "no power control\n");


	if (data->pwren_supply != NULL) {
		rc = regulator_enable(data->pwren_supply);
		if (rc) {
			dev_err(dev, "fail to turn on pwren_supply rc %d\n", rc);
			return rc;
		}
	}

	/* set sensor on pinctrl state */
	stmvl53l1_pinctrl_set_state(dev, devm_pinctrl_get(dev), "sensor_on");

	/* Enable shared I2C */
	if (shared_i2c_data->pinctrl != NULL) {
		rc = shared_i2c_set_state(dev, shared_i2c_data->pinctrl, "on_i2c");
		if (rc) {
			dev_err(dev, "Error enabling i2c bus %d\n", rc);
			return rc;
		}
	}

	if (data->vl53l1_data != NULL) {
		data->vl53l1_data->is_power_up = true;
		dev_info(dev, "Power up successfully\n");
	}

	return rc;
}

/**
 * remove power to device (reset it)
 *
 * @param i2c_object the i2c layer object
 * @return 0 on success
 */
int stmvl53l1_power_down_i2c(void *i2c_object)
{
	int rc = 0;
	struct i2c_data *data = (struct i2c_data *)i2c_object;
	struct device *dev = &data->client->dev;

	if (!data->vl53l1_data->is_power_up)
		return rc;

	/* turn off power */
	if (data->pwren_gpio != -1)
		gpio_set_value(data->pwren_gpio, 0);

	if (data->pwren_supply != NULL) {
		rc = regulator_disable(data->pwren_supply);
		if (rc)
			dev_err(dev, "fail to turn off pwren_supply rc %d\n", rc);
	}

	if (data->vio_gpio != -1 && data->io_flag.vio_owned) {
		rc = gpiod_direction_output(gpio_to_desc(data->vio_gpio), 0);
		if (rc) {
			dev_err(dev, "Failed to set value for vio GPIO %d\n",
				data->vio_gpio);
			return rc;
		}
		put_vio(data);
	} else if (data->vio != NULL) {
		rc = regulator_disable(data->vio);
		if (rc)
			dev_err(dev, "reg disable vio failed. rc=%d\n", rc);
	}

	/* set sensor off pinctrl state */
	stmvl53l1_pinctrl_set_state(dev, devm_pinctrl_get(dev), "sensor_off");

	/* Disable shared I2C */
	if (shared_i2c_data->pinctrl != NULL) {
		rc = shared_i2c_set_state(dev, shared_i2c_data->pinctrl, "off_i2c");
		if (rc) {
			dev_err(dev, "Error disabling i2c bus %d\n", rc);
			return rc;
		}
	}

	if (data->vl53l1_data != NULL) {
		data->vl53l1_data->is_power_up = false;
		dev_info(dev, "Power down successfully\n");
	}

	return rc;
}

/**
 * Set pin control state
 *
 * @param pinctrl  the pinctrl instance
 * @param state_str  new state name
 * @return
 */
void stmvl53l1_pinctrl_set_state(struct device *dev, struct pinctrl *pinctrl,
	const char *state_str)
{
	int ret;
	struct pinctrl_state *state;

	if (IS_ERR(pinctrl)) {
		dev_err(dev, "Invalid pinctrl\n");
		return;
	}

	state = pinctrl_lookup_state(pinctrl, state_str);
	if (IS_ERR(state)) {
		dev_info(dev, "State %s not found (%ld)\n",
			 state_str, PTR_ERR(state));
		return;
	}

	ret = pinctrl_select_state(pinctrl, state);
	if (ret) {
		dev_warn(dev, "Error selecting state %s (%d)\n",
			 state_str, ret);
		return;
	}

	dev_info(dev, "Selecting pinctrl state %s\n", state_str);
}

static int handle_i2c_address_device_change_lock(struct i2c_data *data)
{
	struct i2c_client *client = (struct i2c_client *)data->client;
	struct device *dev = &client->dev;
	uint8_t buffer[3];
	struct i2c_msg msg;
	int rc = 0;

	dev_dbg(dev, "change device i2c address from 0x%02x to 0x%02x\n",
		data->boot_reg, client->addr);
	/* no i2c-access must occur before fw boot time */
	usleep_range(VL53L1_FIRMWARE_BOOT_TIME_US,
		VL53L1_FIRMWARE_BOOT_TIME_US + 1);

	/* manually send message to update i2c address */
	buffer[0] = (VL53L1_I2C_SLAVE__DEVICE_ADDRESS >> 8) & 0xFF;
	buffer[1] = (VL53L1_I2C_SLAVE__DEVICE_ADDRESS >> 0) & 0xFF;
	buffer[2] = client->addr;
	msg.addr = data->boot_reg;
	msg.flags = client->flags;
	msg.buf = buffer;
	msg.len = 3;
	if (i2c_transfer(client->adapter, &msg, 1) != 1) {
		rc = -ENXIO;
		dev_err(dev, "Fail to change i2c address to 0x%02x\n",
			client->addr);
	}

	return rc;
}

/* reset release will also handle device address change. It will avoid state
 * where multiple stm53l1 are bring out of reset at the same time with the
 * same boot address.
 * Note that we don't manage case where boot_reg has the same value as a final
 * i2c address of another device. This case is not supported and will lead
 * to unpredictable behavior.
 */
static int release_reset(struct i2c_data *data)
{
	struct i2c_client *client = (struct i2c_client *)data->client;
	int rc = 0;
	bool is_address_change = client->addr != data->boot_reg;

	if (is_address_change)
		mutex_lock(&dev_addr_change_mutex);

	/* We must control reset pin in 2 ms. */
	fsleep(2000);
	gpio_set_value(data->xsdn_gpio, 1);
	if (is_address_change) {
		rc = handle_i2c_address_device_change_lock(data);
		if (rc)
			gpio_set_value(data->xsdn_gpio, 0);
	}

	if (is_address_change)
		mutex_unlock(&dev_addr_change_mutex);

	return rc;
}

/**
 * release device reset
 *
 * @param i2c_object the i2c layer object
 * @return 0 on success
 */
int stmvl53l1_reset_release_i2c(void *i2c_object)
{
	int rc;
	struct i2c_data *data = (struct i2c_data *)i2c_object;
	struct device *dev = &data->client->dev;

	rc = release_reset(data);
	if (rc)
		goto error;

	/* and now wait for device end of boot */
	data->vl53l1_data->is_delay_allowed = true;
	rc = VL53L1_WaitDeviceBooted(&data->vl53l1_data->stdev);
	data->vl53l1_data->is_delay_allowed = false;
	if (rc) {
		gpio_set_value(data->xsdn_gpio, 0);
		dev_err(dev, "boot fail with error %d\n", rc);
		data->vl53l1_data->last_error = rc;
		rc = -EIO;
	}

error:
	return rc;
}

/**
 * put device under reset
 *
 * @param i2c_object the i2c layer object
 * @return 0 on success
 */
int stmvl53l1_reset_hold_i2c(void *i2c_object)
{
	struct i2c_data *data = (struct i2c_data *)i2c_object;

	gpio_set_value(data->xsdn_gpio, 0);

	return 0;
}

int stmvl53l1_init_i2c(void)
{
	int ret = 0;

	/* register as a i2c client device */
	ret = i2c_add_driver(&stmvl53l1_driver);
	if (ret)
		pr_err("%d erro ret:%d\n", __LINE__, ret);

	return ret;
}


void stmvl53l1_clean_up_i2c(void)
{
	if (stm_test_i2c_client) {
		i2c_unregister_device(stm_test_i2c_client);
	}
}

static irqreturn_t stmvl53l1_irq_handler_i2c(int vec, void *info)
{
	struct i2c_data *i2c_data = (struct i2c_data *)info;
	struct device *dev = &i2c_data->client->dev;

	if (i2c_data->irq == vec) {
		dev_dbg(dev, "irq");
		stmvl53l1_intr_handler(i2c_data->vl53l1_data);
		dev_dbg(dev, "over");
	} else {
		if (!i2c_data->msg_flag.unhandled_irq_vec) {
			modi2c_warn("unmatching vec %d != %d\n",
					vec, i2c_data->irq);
			i2c_data->msg_flag.unhandled_irq_vec = 1;
		}
	}

	return IRQ_HANDLED;
}

/**
 * enable and start intr handling
 *
 * @param object  our i2c_data specific object
 * @param poll_mode [in/out] set to force mode clear to use irq
 * @return 0 on success and set ->poll_mode if it fail ranging wan't start
 */
int stmvl53l1_start_intr(void *object, int *poll_mode)
{
	struct i2c_data *i2c_data;
	struct device *dev;
	int rc;

	i2c_data = (struct i2c_data *)object;
	dev = &i2c_data->client->dev;

	/* irq and gpio acquire config done in parse_tree */
	if (i2c_data->irq < 0) {
		/* the i2c tree as no intr force polling mode */
		*poll_mode = -1;
		return 0;
	}
	/* clear irq warning report enabe it again for this session */
	i2c_data->msg_flag.unhandled_irq_vec = 0;
	/* if started do no nothing */
	if (i2c_data->io_flag.intr_started) {
		/* nothing to do */
		*poll_mode = 0;
		return 0;
	}

	dev_dbg(dev, "to register_irq:%d\n", i2c_data->irq);
	rc = request_threaded_irq(i2c_data->irq, NULL,
			stmvl53l1_irq_handler_i2c,
			IRQF_TRIGGER_FALLING|IRQF_ONESHOT,
			"vl53l1_interrupt",
			(void *)i2c_data);
	if (rc) {
		dev_err(dev, "fail to req threaded irq rc=%d\n", rc);
		*poll_mode = 0;
	} else {
		dev_info(dev, "irq %d now handled\n", i2c_data->irq);
		i2c_data->io_flag.intr_started = 1;
		*poll_mode = 0;
	}
	return rc;
}

void *stmvl53l1_get(void *object)
{
	struct i2c_data *data = (struct i2c_data *)object;

	kref_get(&data->ref);

	return object;
}

static void memory_release(struct kref *kref)
{
	struct i2c_data *data = container_of(kref, struct i2c_data, ref);

	kfree(data->vl53l1_data);
	kfree(data);
}

int stmvl53l1_put(void *object)
{
	struct i2c_data *data = (struct i2c_data *)object;

	return kref_put(&data->ref, memory_release);
}

void __exit stmvl53l1_exit_i2c(void *i2c_object)
{
	i2c_del_driver(&stmvl53l1_driver);
}
