// SPDX-License-Identifier: GPL-2.0
/*
 * Synaptics TouchCom touchscreen driver
 *
 * Copyright (C) 2017-2020 Synaptics Incorporated. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * INFORMATION CONTAINED IN THIS DOCUMENT IS PROVIDED "AS-IS," AND SYNAPTICS
 * EXPRESSLY DISCLAIMS ALL EXPRESS AND IMPLIED WARRANTIES, INCLUDING ANY
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE,
 * AND ANY WARRANTIES OF NON-INFRINGEMENT OF ANY INTELLECTUAL PROPERTY RIGHTS.
 * IN NO EVENT SHALL SYNAPTICS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, PUNITIVE, OR CONSEQUENTIAL DAMAGES ARISING OUT OF OR IN CONNECTION
 * WITH THE USE OF THE INFORMATION CONTAINED IN THIS DOCUMENT, HOWEVER CAUSED
 * AND BASED ON ANY THEORY OF LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, AND EVEN IF SYNAPTICS WAS ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE. IF A TRIBUNAL OF COMPETENT JURISDICTION DOES
 * NOT PERMIT THE DISCLAIMER OF DIRECT DAMAGES OR ANY OTHER DAMAGES, SYNAPTICS'
 * TOTAL CUMULATIVE LIABILITY TO ANY PARTY SHALL NOT EXCEED ONE HUNDRED U.S.
 * DOLLARS.
 */

/*
 * @file syna_tcm2_platform_i2c.c
 *
 * This file is the reference code of I2C module used for communicating with
 * Synaptics TouchCom device using I2C
 */

#include <linux/i2c.h>

#include "syna_tcm2.h"
#include "syna_tcm2_cdev.h"
#include "syna_tcm2_platform.h"

#define I2C_MODULE_NAME "synaptics_tcm_i2c"

#define XFER_ATTEMPTS 5

static struct platform_device *syna_i2c_device;


/*
 * syna_request_managed_device()
 *
 * Request and return the device pointer for managed
 *
 * @param
 *     none.
 *
 * @return
 *     a device pointer allocated previously
 */
#if defined(DEV_MANAGED_API) || defined(USE_DRM_PANEL_NOTIFIER)
struct device *syna_request_managed_device(void)
{
	if (!syna_i2c_device)
		return NULL;

	return syna_i2c_device->dev.parent;
}
#endif


/*
 * syna_i2c_request_gpio()
 *
 * Setup the given gpio
 *
 * @param
 *    [ in] gpio:   the target gpio
 *    [ in] config: '1' for setting up, and '0' to release the gpio
 *    [ in] dir:    default direction of gpio
 *    [ in] state:  default state of gpio
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_i2c_request_gpio(int gpio, bool config, int dir,
		int state, char *label)
{
	int retval;
#ifdef DEV_MANAGED_API
	struct device *dev = syna_request_managed_device();

	if (!dev) {
		LOGE("Invalid managed device\n");
		return -ENODEV;
	}
#endif

	if (gpio < 0) {
		LOGE("Invalid gpio pin\n");
		return -EINVAL;
	}

	if (config) {
		retval = scnprintf(label, 16, "tcm_gpio_%d\n", gpio);
		if (retval < 0) {
			LOGE("Fail to set GPIO label\n");
			return retval;
		}
#ifdef DEV_MANAGED_API
		retval = devm_gpio_request(dev, gpio, label);
#else /* Legacy API */
		retval = gpio_request(gpio, label);
#endif
		if (retval < 0) {
			LOGE("Fail to request GPIO %d\n", gpio);
			return retval;
		}

		if (dir == 0)
			retval = gpio_direction_input(gpio);
		else
			retval = gpio_direction_output(gpio, state);

		if (retval < 0) {
			LOGE("Fail to set GPIO %d direction\n", gpio);
			return retval;
		}
	} else {
#ifndef DEV_MANAGED_API
		gpio_free(gpio);
#endif
	}

	return 0;
}
/*
 * syna_i2c_free_gpios()
 *
 * Release the GPIOs requested previously
 *
 * @param
 *    [ in] hw_if: the handle of hw interface
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_i2c_free_gpios(struct syna_hw_interface *hw_if)
{
	struct syna_hw_attn_data *attn = &hw_if->bdata_attn;
	struct syna_hw_rst_data *rst = &hw_if->bdata_rst;
	struct syna_hw_bus_data *bus = &hw_if->bdata_io;

	/* release gpios */
	if (rst->reset_gpio > 0)
		syna_i2c_request_gpio(rst->reset_gpio, false, 0, 0, NULL);
	if (attn->irq_gpio > 0)
		syna_i2c_request_gpio(attn->irq_gpio, false, 0, 0, NULL);
	if (bus->switch_gpio > 0)
		syna_i2c_request_gpio(bus->switch_gpio, false, 0, 0, NULL);

	return 0;
}
/*
 * syna_i2c_config_gpios()
 *
 * Initialize the GPIOs defined in device tree
 *
 * @param
 *    [ in] hw_if: the handle of hw interface
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_i2c_config_gpios(struct syna_hw_interface *hw_if)
{
	int retval;
	static char str_irq_gpio[32] = {0};
	static char str_rst_gpio[32] = {0};
	static char str_io_switch_gpio[32] = {0};
	struct syna_hw_attn_data *attn = &hw_if->bdata_attn;
	struct syna_hw_rst_data *rst = &hw_if->bdata_rst;
	struct syna_hw_bus_data *bus = &hw_if->bdata_io;

	if (attn->irq_gpio > 0) {
		retval = syna_i2c_request_gpio(attn->irq_gpio,
				true, 0, 0, str_irq_gpio);
		if (retval < 0) {
			LOGE("Fail to configure interrupt GPIO %d\n",
				attn->irq_gpio);
			goto err_set_gpio_irq;
		}
	}

	if (rst->reset_gpio > 0) {
		retval = syna_i2c_request_gpio(rst->reset_gpio,
				true, 1, rst->reset_on_state,
				str_rst_gpio);
		if (retval < 0) {
			LOGE("Fail to configure reset GPIO %d\n",
				rst->reset_gpio);
			goto err_set_gpio_reset;
		}
	}

	if (bus->switch_gpio > 0) {
		retval = syna_i2c_request_gpio(bus->switch_gpio,
				true, 1, bus->switch_state,
				str_io_switch_gpio);
		if (retval < 0) {
			LOGE("Fail to configure switch GPIO %d\n",
				bus->switch_gpio);
			goto err_set_gpio_switch;
		}
	}

	return 0;

err_set_gpio_switch:
	if (rst->reset_gpio >= 0)
		syna_i2c_request_gpio(rst->reset_gpio, false, 0, 0, NULL);
err_set_gpio_reset:
	if (attn->irq_gpio >= 0)
		syna_i2c_request_gpio(attn->irq_gpio, false, 0, 0, NULL);
err_set_gpio_irq:
	return retval;
}

/*
 * syna_i2c_parse_dt()
 *
 * Parse and obtain board specific data from the device tree source file.
 * Keep the data in structure syna_tcm_hw_data for later using.
 *
 * @param
 *    [ in] hw_if: the handle of hw interface
 *    [ in] dev: device model
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
#ifdef CONFIG_OF
static int syna_i2c_parse_dt(struct syna_hw_interface *hw_if,
		struct device *dev)
{
	int retval;
	u32 value;
	struct property *prop;
	struct device_node *np = dev->of_node;
	const char *name;
	struct syna_hw_attn_data *attn = &hw_if->bdata_attn;
	struct syna_hw_pwr_data *pwr = &hw_if->bdata_pwr;
	struct syna_hw_rst_data *rst = &hw_if->bdata_rst;
	struct syna_hw_bus_data *bus = &hw_if->bdata_io;

	prop = of_find_property(np, "synaptics,irq-gpio", NULL);
	if (prop && prop->length) {
		attn->irq_gpio = of_get_named_gpio_flags(np,
				"synaptics,irq-gpio", 0,
				(enum of_gpio_flags *)&attn->irq_flags);
	} else {
		attn->irq_gpio = -1;
	}

	retval = of_property_read_u32(np, "synaptics,irq-on-state", &value);
	if (retval < 0)
		attn->irq_on_state = 0;
	else
		attn->irq_on_state = value;

	prop = of_find_property(np, "synaptics,power-supply", NULL);
	if (prop && prop->length) {
		retval = of_property_read_u32(np, "synaptics,power-supply",
				&value);
		if (retval < 0) {
			LOGE("Fail to read power-supply property\n");
			return retval;
		}

		pwr->psu = value;

	} else {
		pwr->psu = (int)PSU_REGULATOR;
	}

	retval = of_property_read_string(np, "synaptics,avdd-name", &name);
	if (retval < 0)
		pwr->avdd_reg_name = NULL;
	else
		pwr->avdd_reg_name = name;

	retval = of_property_read_string(np, "synaptics,vdd-name", &name);
	if (retval < 0)
		pwr->vdd_reg_name = NULL;
	else
		pwr->vdd_reg_name = name;

	prop = of_find_property(np, "synaptics,vdd-gpio", NULL);
	if (prop && prop->length) {
		pwr->vdd_gpio = of_get_named_gpio_flags(np,
				"synaptics,vdd-gpio", 0, NULL);
	} else {
		pwr->vdd_gpio = -1;
	}

	prop = of_find_property(np, "synaptics,avdd-gpio", NULL);
	if (prop && prop->length) {
		pwr->avdd_gpio = of_get_named_gpio_flags(np,
				"synaptics,avdd-gpio", 0, NULL);
	} else {
		pwr->avdd_gpio = -1;
	}

	prop = of_find_property(np, "synaptics,power-on-state", NULL);
	if (prop && prop->length) {
		retval = of_property_read_u32(np, "synaptics,power-on-state",
				&value);
		if (retval < 0) {
			LOGE("Fail to read power-on-state property\n");
			return retval;
		}

		pwr->power_on_state = value;

	} else {
		pwr->power_on_state = 0;
	}

	prop = of_find_property(np, "synaptics,power-delay-ms", NULL);
	if (prop && prop->length) {
		retval = of_property_read_u32(np, "synaptics,power-delay-ms",
				&value);
		if (retval < 0) {
			LOGE("Fail to read power-delay-ms property\n");
			return retval;
		}

		pwr->power_delay_ms = value;

	} else {
		pwr->power_delay_ms = 0;
	}

	prop = of_find_property(np, "synaptics,reset-gpio", NULL);
	if (prop && prop->length) {
		rst->reset_gpio = of_get_named_gpio_flags(np,
				"synaptics,reset-gpio", 0, NULL);
	} else {
		rst->reset_gpio = -1;
	}

	prop = of_find_property(np, "synaptics,reset-on-state", NULL);
	if (prop && prop->length) {
		retval = of_property_read_u32(np, "synaptics,reset-on-state",
				&value);
		if (retval < 0) {
			LOGE("Fail to read reset-on-state property\n");
			return retval;
		}

		rst->reset_on_state = value;

	} else {
		rst->reset_on_state = 0;
	}

	prop = of_find_property(np, "synaptics,reset-active-ms", NULL);
	if (prop && prop->length) {
		retval = of_property_read_u32(np, "synaptics,reset-active-ms",
				&value);
		if (retval < 0) {
			LOGE("Fail to read reset-active-ms property\n");
			return retval;
		}

		rst->reset_active_ms = value;

	} else {
		rst->reset_active_ms = 0;
	}

	prop = of_find_property(np, "synaptics,reset-delay-ms", NULL);
	if (prop && prop->length) {
		retval = of_property_read_u32(np, "synaptics,reset-delay-ms",
				&value);
		if (retval < 0) {
			LOGE("Fail to read reset-delay-ms property\n");
			return retval;
		}

		rst->reset_delay_ms = value;

	} else {
		rst->reset_delay_ms = 0;
	}

	prop = of_find_property(np, "synaptics,io-switch-gpio", NULL);
	if (prop && prop->length) {
		bus->switch_gpio = of_get_named_gpio_flags(np,
				"synaptics,io-switch-gpio", 0, NULL);
	} else {
		bus->switch_gpio = -1;
	}

	prop = of_find_property(np, "synaptics,io-switch", NULL);
	if (prop && prop->length) {
		retval = of_property_read_u32(np, "synaptics,io-switch",
				&value);
		if (retval < 0) {
			LOGE("Fail to read io-switch property\n");
			return retval;
		}

		bus->switch_state = value;

	} else {
		bus->switch_state = 1;
	}

	pwr->avdd_power_on_delay_ms = 0;
	prop = of_find_property(np, "synaptics,avdd-power-on-delay-ms", NULL);
	if (prop && prop->length) {
		of_property_read_u32(np, "synaptics,avdd-power-on-delay-ms",
				&pwr->avdd_power_on_delay_ms);
	}

	pwr->vdd_power_on_delay_ms = 0;
	prop = of_find_property(np, "synaptics,vdd-power-on-delay-ms", NULL);
	if (prop && prop->length) {
		of_property_read_u32(np, "synaptics,vdd-power-on-delay-ms",
				&pwr->vdd_power_on_delay_ms);
	}

	pwr->avdd_power_off_delay_ms = 0;
	prop = of_find_property(np, "synaptics,avdd-power-off-delay-ms", NULL);
	if (prop && prop->length) {
		of_property_read_u32(np, "synaptics,avdd-power-off-delay-ms",
				&pwr->avdd_power_off_delay_ms);
	}

	pwr->vdd_power_off_delay_ms = 0;
	prop = of_find_property(np, "synaptics,vdd-power-off-delay-ms", NULL);
	if (prop && prop->length) {
		of_property_read_u32(np, "synaptics,vdd-power-off-delay-ms",
				&pwr->vdd_power_off_delay_ms);
	}

	return 0;
}
#endif

/*
 * syna_i2c_read()
 *
 * TouchCom over I2C uses the normal I2C addressing and transaction direction
 * mechanisms to select the device and retrieve the data.
 *
 * @param
 *    [ in] hw_if:   the handle of hw interface
 *    [out] rd_data: buffer for storing data retrieved from device
 *    [ in] rd_len: number of bytes retrieved from device
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_i2c_read(struct syna_hw_interface *hw_if,
		unsigned char *rd_data, unsigned int rd_len)
{
	int retval;
	unsigned int attempt;
	struct i2c_msg msg;
	struct i2c_client *i2c = hw_if->pdev;
	struct syna_hw_bus_data *bus = &hw_if->bdata_io;

	if (!i2c) {
		LOGE("Invalid bus io device\n");
		return -ENXIO;
	}

	syna_pal_mutex_lock(&bus->io_mutex);

	if ((rd_len & 0xffff) == 0xffff) {
		LOGE("Invalid read length 0x%X\n", (rd_len & 0xffff));
		retval = -EINVAL;
		goto exit;
	}

	msg.addr = i2c->addr;
	msg.flags = I2C_M_RD;
	msg.len = rd_len;
	msg.buf = rd_data;

	for (attempt = 0; attempt < XFER_ATTEMPTS; attempt++) {
		retval = i2c_transfer(i2c->adapter, &msg, 1);
		if (retval == 1) {
			retval = rd_len;
			goto exit;
		}
		LOGE("Transfer attempt %d failed\n", attempt + 1);

		if (attempt + 1 == XFER_ATTEMPTS) {
			retval = -EIO;
			goto exit;
		}

		syna_pal_sleep_ms(20);
	}

exit:
	syna_pal_mutex_unlock(&bus->io_mutex);

	return retval;
}

/*
 * syna_i2c_write()
 *
 * TouchCom over I2C uses the normal I2C addressing and transaction direction
 * mechanisms to select the device and send the data to the device.
 *
 * @param
 *    [ in] hw_if:   the handle of hw interface
 *    [ in] wr_data: written data
 *    [ in] wr_len: length of written data in bytes
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_i2c_write(struct syna_hw_interface *hw_if,
		unsigned char *wr_data, unsigned int wr_len)
{
	int retval;
	unsigned int attempt;
	struct i2c_msg msg;
	struct i2c_client *i2c = hw_if->pdev;
	struct syna_hw_bus_data *bus = &hw_if->bdata_io;

	if (!i2c) {
		LOGE("Invalid bus io device\n");
		return -ENXIO;
	}

	syna_pal_mutex_lock(&bus->io_mutex);

	if ((wr_len & 0xffff) == 0xffff) {
		LOGE("Invalid write length 0x%X\n", (wr_len & 0xffff));
		retval = -EINVAL;
		goto exit;
	}

	msg.addr = i2c->addr;
	msg.flags = 0;
	msg.len = wr_len;
	msg.buf = wr_data;

	for (attempt = 0; attempt < XFER_ATTEMPTS; attempt++) {
		retval = i2c_transfer(i2c->adapter, &msg, 1);
		if (retval == 1) {
			retval = wr_len;
			goto exit;
		}
		LOGE("Transfer attempt %d failed\n", attempt + 1);

		if (attempt + 1 == XFER_ATTEMPTS) {
			retval = -EIO;
			goto exit;
		}

		syna_pal_sleep_ms(20);
	}

exit:
	syna_pal_mutex_unlock(&bus->io_mutex);

	return retval;
}

/*
 * syna_i2c_hw_reset()
 *
 * Toggle the hardware gpio pin to perform the chip reset
 *
 * @param
 *    [ in] hw_if: the handle of hw interface
 *
 * @return
 *     none.
 */
static void syna_i2c_hw_reset(struct syna_hw_interface *hw_if)
{
	struct syna_hw_rst_data *rst = &hw_if->bdata_rst;

	if (rst->reset_gpio == 0)
		return;

	LOGD("Prepare to toggle reset, hold:%d delay:%d\n",
		rst->reset_active_ms, rst->reset_delay_ms);

	gpio_set_value(rst->reset_gpio, (rst->reset_on_state & 0x01));
	syna_pal_sleep_ms(rst->reset_active_ms);
	gpio_set_value(rst->reset_gpio, ((!rst->reset_on_state) & 0x01));
	syna_pal_sleep_ms(rst->reset_delay_ms);
}


/*
 * syna_i2c_enable_pwr_gpio()
 *
 * Helper to enable power supply through GPIO
 *
 * @param
 *    [ in] hw_if: the handle of hw interface
 *    [ in] en:    '1' for enabling, and '0' for disabling
 *
 * @return
 *    none
 */
static int syna_i2c_enable_pwr_gpio(struct syna_hw_interface *hw_if,
		bool en)
{
	struct syna_hw_pwr_data *pwr = &hw_if->bdata_pwr;

	if (en) {
		if (pwr->avdd_gpio > 0)
			gpio_set_value(pwr->avdd_gpio, pwr->power_on_state);

		if (pwr->avdd_power_on_delay_ms > 0)
			syna_pal_sleep_ms(pwr->avdd_power_on_delay_ms);

		if (pwr->vdd_gpio > 0)
			gpio_set_value(pwr->vdd_gpio, pwr->power_on_state);

		if (pwr->vdd_power_on_delay_ms > 0)
			syna_pal_sleep_ms(pwr->vdd_power_on_delay_ms);
	} else {
		if (pwr->vdd_gpio > 0)
			gpio_set_value(pwr->vdd_gpio, !pwr->power_on_state);

		if (pwr->vdd_power_off_delay_ms > 0)
			syna_pal_sleep_ms(pwr->vdd_power_off_delay_ms);

		if (pwr->avdd_gpio > 0)
			gpio_set_value(pwr->avdd_gpio, !pwr->power_on_state);

		if (pwr->avdd_power_off_delay_ms > 0)
			syna_pal_sleep_ms(pwr->avdd_power_off_delay_ms);
	}

	return 0;
}

/*
 * syna_i2c_enable_regulator()
 *
 * Enable or disable the regulator
 *
 * @param
 *    [ in] hw_if: the handle of hw interface
 *    [ in] en:    '1' for enabling, and '0' for disabling
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_i2c_enable_regulator(struct syna_hw_interface *hw_if,
		bool en)
{
	int retval;
	struct syna_hw_pwr_data *pwr = &hw_if->bdata_pwr;
	struct regulator *vdd_reg = pwr->vdd_reg_dev;
	struct regulator *avdd_reg = pwr->avdd_reg_dev;

	if (!en) {
		retval = 0;
		goto disable_pwr_reg;
	}

	if (avdd_reg) {
		retval = regulator_enable(avdd_reg);
		if (retval < 0) {
			LOGE("Fail to enable avdd regulator\n");
			goto exit;
		}

		if (pwr->avdd_power_on_delay_ms > 0)
			syna_pal_sleep_ms(pwr->avdd_power_on_delay_ms);
	}

	if (vdd_reg) {
		retval = regulator_enable(vdd_reg);
		if (retval < 0) {
			LOGE("Fail to enable vdd regulator\n");
			goto disable_avdd_reg;
		}

		if (pwr->vdd_power_on_delay_ms > 0)
			syna_pal_sleep_ms(pwr->vdd_power_on_delay_ms);
	}

	return 0;

disable_pwr_reg:
	if (vdd_reg) {
		regulator_disable(vdd_reg);

		if (pwr->vdd_power_off_delay_ms > 0)
			syna_pal_sleep_ms(pwr->vdd_power_off_delay_ms);
	}

disable_avdd_reg:
	if (avdd_reg) {
		regulator_disable(avdd_reg);

		if (pwr->avdd_power_off_delay_ms > 0)
			syna_pal_sleep_ms(pwr->avdd_power_off_delay_ms);
	}
exit:
	return retval;
}

/*
 * syna_i2c_power_on()
 *
 * Power on touch controller through regulators or gpios for PWM
 *
 * @param
 *    [ in] hw_if: the handle of hw interface
 *    [ in] en:    '1' for powering on, and '0' for powering off
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_i2c_power_on(struct syna_hw_interface *hw_if,
		bool en)
{
	int retval;
	struct syna_hw_pwr_data *pwr = &hw_if->bdata_pwr;

	LOGD("Prepare to power %s device through %s ...\n",
		(en) ? "on" : "off",
		(pwr->psu == PSU_REGULATOR) ? "regulator" : "gpio control");

	if (pwr->psu == PSU_REGULATOR)
		retval = syna_i2c_enable_regulator(hw_if, en);
	else
		retval = syna_i2c_enable_pwr_gpio(hw_if, en);

	if (retval < 0) {
		LOGE("Fail to power %s device\n", (en) ? "on" : "off");
		return retval;
	}

	// syna_pal_sleep_ms(pwr->power_delay_ms);

	LOGI("Device power %s\n", (en) ? "on" : "off");

	return 0;
}

/*
 * syna_i2c_get_regulator()
 *
 * Acquire or release the regulator
 *
 * @param
 *    [ in] hw_if: the handle of hw interface
 *    [ in] get:   '1' for getting the regulator, and '0' for removing
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_i2c_get_regulator(struct syna_hw_interface *hw_if,
		bool get)
{
	int retval;
	struct device *dev = syna_i2c_device->dev.parent;
	struct syna_hw_pwr_data *pwr = &hw_if->bdata_pwr;

	if (!get) {
		retval = 0;
		goto regulator_put;
	}

	if (pwr->vdd_reg_name != NULL && *pwr->vdd_reg_name != 0) {
#ifdef DEV_MANAGED_API
		pwr->vdd_reg_dev = devm_regulator_get(dev, pwr->vdd_reg_name);
#else /* Legacy API */
		pwr->vdd_reg_dev = regulator_get(dev, pwr->vdd_reg_name);
#endif
		if (IS_ERR((struct regulator *)pwr->vdd_reg_dev)) {
			LOGW("Vdd regulator is not ready\n");
			retval = PTR_ERR((struct regulator *)pwr->vdd_reg_dev);
			goto exit;
		}
	}

	if (pwr->avdd_reg_name != NULL && *pwr->avdd_reg_name != 0) {
#ifdef DEV_MANAGED_API
		pwr->avdd_reg_dev = devm_regulator_get(dev, pwr->avdd_reg_name);
#else /* Legacy API */
		pwr->avdd_reg_dev = regulator_get(dev, pwr->avdd_reg_name);
#endif
		if (IS_ERR((struct regulator *)pwr->avdd_reg_dev)) {
			LOGW("AVdd regulator is not ready\n");
			retval = PTR_ERR((struct regulator *)pwr->avdd_reg_dev);
			goto regulator_vdd_put;
		}
	}

	return 0;

regulator_put:
	if (pwr->vdd_reg_dev) {
#ifdef DEV_MANAGED_API
		devm_regulator_put(pwr->vdd_reg_dev);
#else /* Legacy API */
		regulator_put(pwr->vdd_reg_dev);
#endif
		pwr->vdd_reg_dev = NULL;
	}
regulator_vdd_put:
	if (pwr->avdd_reg_dev) {
#ifdef DEV_MANAGED_API
		devm_regulator_put(pwr->avdd_reg_dev);
#else /* Legacy API */
		regulator_put(pwr->avdd_reg_dev);
#endif
		pwr->avdd_reg_dev = NULL;
	}
exit:
	return retval;
}

/*
 * syna_i2c_config_psu()
 *
 * Initialize the power supply unit
 *
 * @param
 *    [ in] hw_if: the handle of hw interface
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_i2c_config_psu(struct syna_hw_interface *hw_if)
{
	int retval;
	static char str_vdd_gpio[32] = {0};
	static char str_avdd_gpio[32] = {0};
	struct syna_hw_pwr_data *pwr = &hw_if->bdata_pwr;

	if (pwr->psu != PSU_REGULATOR) {
		/* set up power gpio */
		if (pwr->vdd_gpio > 0) {
			retval = syna_i2c_request_gpio(pwr->vdd_gpio,
					true, 1, !pwr->power_on_state,
					str_vdd_gpio);
			if (retval < 0) {
				LOGE("Fail to configure vdd GPIO %d\n",
					pwr->vdd_gpio);
				return retval;
			}
		}

		if (pwr->avdd_gpio > 0) {
			retval = syna_i2c_request_gpio(pwr->avdd_gpio,
					true, 1, !pwr->power_on_state,
					str_avdd_gpio);
			if (retval < 0) {
				LOGE("Fail to configure avdd GPIO %d\n",
					pwr->avdd_gpio);

				syna_i2c_request_gpio(pwr->vdd_gpio,
					false, 0, 0, NULL);
				return retval;
			}
		}
	} else {
		/* set up regulator */
		retval = syna_i2c_get_regulator(hw_if, true);
		if (retval < 0) {
			LOGE("Fail to configure regulators\n");
			return retval;
		}
	}

	return 0;
}

/*
 * syna_i2c_release_psu()
 *
 * Release the power supply unit
 *
 * @param
 *    [ in] hw_if: the handle of hw interface
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_i2c_release_psu(struct syna_hw_interface *hw_if)
{
	struct syna_hw_pwr_data *pwr = &hw_if->bdata_pwr;

	if (pwr->psu != PSU_GPIO) {
		if (pwr->avdd_gpio > 0)
			syna_i2c_request_gpio(pwr->avdd_gpio, false, 0, 0, NULL);
		if (pwr->vdd_gpio > 0)
			syna_i2c_request_gpio(pwr->vdd_gpio, false, 0, 0, NULL);
	} else {
		syna_i2c_get_regulator(hw_if, false);
	}

	return 0;
}

/*
 * syna_i2c_enable_irq()
 *
 * Enable or disable the handling of interrupt
 *
 * @param
 *    [ in] hw_if: the handle of hw interface
 *    [ in] en:    '1' for enabling, and '0' for disabling
 *
 * @return
 *    0 on success; otherwise, on error.
 */
static int syna_i2c_enable_irq(struct syna_hw_interface *hw_if,
		bool en)
{
	int retval = 0;
	struct syna_hw_attn_data *attn = &hw_if->bdata_attn;

	if (attn->irq_id == 0)
		return 0;

	syna_pal_mutex_lock(&attn->irq_en_mutex);

	/* enable the handling of interrupt */
	if (en) {
		if (attn->irq_enabled) {
			LOGD("Interrupt already enabled\n");
			retval = 0;
			goto exit;
		}

		enable_irq(attn->irq_id);
		attn->irq_enabled = true;

		LOGD("Interrupt enabled\n");
	}
	/* disable the handling of interrupt */
	else {
		if (!attn->irq_enabled) {
			LOGD("Interrupt already disabled\n");
			retval = 0;
			goto exit;
		}

		disable_irq_nosync(attn->irq_id);
		attn->irq_enabled = false;

		LOGD("Interrupt disabled\n");
	}

exit:
	syna_pal_mutex_unlock(&attn->irq_en_mutex);

	return retval;
}
/*
 * syna_hw_interface
 *
 * Provide the hardware specific settings in defaults.
 * Be noted the followings could be changed after .dtsi is parsed
 */
static struct syna_hw_interface syna_i2c_hw_if = {
	.bdata_io = {
		.type = BUS_TYPE_I2C,
		.rd_chunk_size = RD_CHUNK_SIZE,
		.wr_chunk_size = WR_CHUNK_SIZE,
	},
	.bdata_attn = {
		.irq_enabled = false,
		.irq_on_state = 0,
	},
	.bdata_rst = {
		.reset_on_state = 0,
		.reset_delay_ms = 200,
		.reset_active_ms = 20,
	},
	.bdata_pwr = {
		.power_on_state = 1,
		.power_delay_ms = 200,
	},
	.ops_power_on = syna_i2c_power_on,
	.ops_hw_reset = syna_i2c_hw_reset,
	.ops_read_data = syna_i2c_read,
	.ops_write_data = syna_i2c_write,
	.ops_enable_irq = syna_i2c_enable_irq,
};

/*
 * syna_i2c_probe()
 *
 * Prepare the specific hardware interface and register the platform i2c device
 *
 * @param
 *    [ in] i2c:    i2c client device
 *    [ in] dev_id: i2c device id
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_i2c_probe(struct i2c_client *i2c,
		const struct i2c_device_id *dev_id)
{
	int retval;
	struct syna_hw_attn_data *attn = &syna_i2c_hw_if.bdata_attn;
	struct syna_hw_bus_data *bus = &syna_i2c_hw_if.bdata_io;

	/* allocate an i2c platform device */
	syna_i2c_device = platform_device_alloc(PLATFORM_DRIVER_NAME, 0);
	if (!syna_i2c_device) {
		LOGE("Fail to allocate platform device\n");
		return -ENOMEM;
	}

#ifdef CONFIG_OF
	syna_i2c_parse_dt(&syna_i2c_hw_if, &i2c->dev);
#endif

	syna_pal_mutex_alloc(&attn->irq_en_mutex);
	syna_pal_mutex_alloc(&bus->io_mutex);

	/* keep the i/o device */
	syna_i2c_hw_if.pdev = i2c;

	syna_i2c_device->dev.parent = &i2c->dev;
	syna_i2c_device->dev.platform_data = &syna_i2c_hw_if;

	/* initialize power unit */
	retval = syna_i2c_config_psu(&syna_i2c_hw_if);
	if (retval < 0) {
		LOGE("Fail to config power unit\n");
		return retval;
	}

	/* initialize the gpio pins */
	retval = syna_i2c_config_gpios(&syna_i2c_hw_if);
	if (retval < 0) {
		LOGE("Fail to config gpio\n");
		return retval;
	}

	/* do i/o switch if defined */
	if (bus->switch_gpio > 0)
		gpio_set_value(bus->switch_gpio, bus->switch_state);

	/* register the i2c platform device */
	retval = platform_device_add(syna_i2c_device);
	if (retval < 0) {
		LOGE("Fail to add platform device\n");
		return retval;
	}

	return 0;
}

/*
 * syna_i2c_remove()
 *
 * Unregister the platform i2c device
 *
 * @param
 *    [ in] i2c: i2c client device
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_i2c_remove(struct i2c_client *i2c)
{
	struct syna_hw_attn_data *attn = &syna_i2c_hw_if.bdata_attn;
	struct syna_hw_bus_data *bus = &syna_i2c_hw_if.bdata_io;

	/* release gpios */
	syna_i2c_free_gpios(&syna_i2c_hw_if);

	/* release power unit */
	syna_i2c_release_psu(&syna_i2c_hw_if);

	/* release mutex */
	syna_pal_mutex_free(&attn->irq_en_mutex);
	syna_pal_mutex_free(&bus->io_mutex);

	/* remove the platform device */
	syna_i2c_device->dev.platform_data = NULL;
	platform_device_unregister(syna_i2c_device);

	return 0;
}

/*
 * Describe an i2c device driver and its related declarations
 */
static const struct i2c_device_id syna_i2c_id_table[] = {
	{I2C_MODULE_NAME, 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, syna_i2c_id_table);

#ifdef CONFIG_OF
static const struct of_device_id syna_i2c_of_match_table[] = {
	{
		.compatible = "synaptics,tcm-i2c",
	},
	{},
};
MODULE_DEVICE_TABLE(of, syna_i2c_of_match_table);
#else
#define syna_i2c_of_match_table NULL
#endif

static struct i2c_driver syna_i2c_driver = {
	.driver = {
		.name = I2C_MODULE_NAME,
		.owner = THIS_MODULE,
		.of_match_table = syna_i2c_of_match_table,
	},
	.probe = syna_i2c_probe,
	.remove = syna_i2c_remove,
	.id_table = syna_i2c_id_table,
};


/*
 * syna_hw_interface_init()
 *
 * Initialize the lower-level hardware interface module.
 * After returning, the handle of hw interface should be ready.
 *
 * @param
 *    void
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
int syna_hw_interface_init(void)
{
	return i2c_add_driver(&syna_i2c_driver);
}

/*
 * syna_hw_interface_exit()
 *
 * Delete the lower-level hardware interface module
 *
 * @param
 *    void
 *
 * @return
 *    none.
 */
void syna_hw_interface_exit(void)
{
	i2c_del_driver(&syna_i2c_driver);
}

MODULE_AUTHOR("Synaptics, Inc.");
MODULE_DESCRIPTION("Synaptics TouchCom I2C Bus Module");
MODULE_LICENSE("GPL v2");

