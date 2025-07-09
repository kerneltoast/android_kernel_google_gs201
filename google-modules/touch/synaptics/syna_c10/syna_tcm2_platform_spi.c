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

/**
 * @file syna_tcm2_platform_spi.c
 *
 * This file is the reference code of I2C module used for communicating with
 * Synaptics TouchCom device using I2C
 */

#include <linux/spi/spi.h>
#include <drm/drm_panel.h>

#include "syna_tcm2.h"
#include "syna_tcm2_platform.h"

#define SPI_MODULE_NAME "synaptics_tcm_spi"

static unsigned char *buf;

static unsigned int buf_size;

static struct spi_transfer *xfer;

static struct platform_device *syna_spi_device;


/**
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
	if (!syna_spi_device)
		return NULL;

	return syna_spi_device->dev.parent;
}
#endif

/**
 * syna_spi_hw_reset()
 *
 * Toggle the hardware gpio pin to perform the chip reset
 *
 * @param
 *    [ in] hw_if: the handle of hw interface
 *
 * @return
 *     none.
 */
static void syna_spi_hw_reset(struct syna_hw_interface *hw_if)
{
	struct syna_hw_rst_data *rst = &hw_if->bdata_rst;

	LOGI("Trigger hardware reset.\n");

	if (rst->reset_gpio >= 0) {
		gpio_set_value(rst->reset_gpio, rst->reset_on_state);
		syna_pal_sleep_ms(rst->reset_active_ms);
		gpio_set_value(rst->reset_gpio, !rst->reset_on_state);
		syna_pal_sleep_ms(rst->reset_delay_ms);
	}
}

/**
 * syna_spi_request_gpio()
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
static int syna_spi_request_gpio(int gpio, bool config, int dir,
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

/**
 * syna_spi_config_gpio()
 *
 * Initialize the GPIOs defined in device tree
 *
 * @param
 *    [ in] hw_if: the handle of hw interface
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_spi_config_gpio(struct syna_hw_interface *hw_if)
{
	int retval;
	static char str_irq_gpio[32] = {0};
	static char str_rst_gpio[32] = {0};
	static char str_vdd_gpio[32] = {0};
	static char str_avdd_gpio[32] = {0};
	struct syna_hw_attn_data *attn = &hw_if->bdata_attn;
	struct syna_hw_pwr_data *pwr = &hw_if->bdata_pwr;
	struct syna_hw_rst_data *rst = &hw_if->bdata_rst;

	if (attn->irq_gpio >= 0) {
		retval = syna_spi_request_gpio(attn->irq_gpio,
				true, 0, 0, str_irq_gpio);
		if (retval < 0) {
			LOGE("Fail to configure interrupt GPIO %d\n",
				attn->irq_gpio);
			goto err_set_gpio_irq;
		}
	}

	if (rst->reset_gpio >= 0) {
		retval = syna_spi_request_gpio(rst->reset_gpio,
				true, 1, !rst->reset_on_state,
				str_rst_gpio);
		if (retval < 0) {
			LOGE("Fail to configure reset GPIO %d\n",
				rst->reset_gpio);
			goto err_set_gpio_reset;
		}
	}

	if (pwr->vdd_gpio >= 0) {
		retval = syna_spi_request_gpio(pwr->vdd_gpio,
				true, 1, !pwr->power_on_state,
				str_vdd_gpio);
		if (retval < 0) {
			LOGE("Fail to configure vdd GPIO %d\n",
				pwr->vdd_gpio);
			goto err_set_gpio_vdd;
		}
	}

	if (pwr->avdd_gpio >= 0) {
		retval = syna_spi_request_gpio(pwr->avdd_gpio,
				true, 1, !pwr->power_on_state,
				str_avdd_gpio);
		if (retval < 0) {
			LOGE("Fail to configure avdd GPIO %d\n",
				pwr->avdd_gpio);
			goto err_set_gpio_avdd;
		}
	}
	return 0;

err_set_gpio_avdd:
	if (pwr->vdd_gpio >= 0)
		syna_spi_request_gpio(pwr->vdd_gpio, false, 0, 0, NULL);
err_set_gpio_vdd:
	if (rst->reset_gpio >= 0)
		syna_spi_request_gpio(rst->reset_gpio, false, 0, 0, NULL);
err_set_gpio_reset:
	if (attn->irq_gpio >= 0)
		syna_spi_request_gpio(attn->irq_gpio, false, 0, 0, NULL);
err_set_gpio_irq:
	return retval;
}

/**
 * syna_spi_enable_regulator()
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
static int syna_spi_enable_regulator(struct syna_hw_interface *hw_if,
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

	if (vdd_reg) {
		retval = regulator_enable(vdd_reg);
		if (retval < 0) {
			LOGE("Fail to enable vdd regulator\n");
			goto exit;
		}
	}

	if (avdd_reg) {
		retval = regulator_enable(avdd_reg);
		if (retval < 0) {
			LOGE("Fail to enable avdd regulator\n");
			goto disable_avdd_reg;
		}
		syna_pal_sleep_ms(pwr->power_on_delay_ms);
	}

	return 0;

disable_pwr_reg:
	if (vdd_reg)
		regulator_disable(vdd_reg);

disable_avdd_reg:
	if (avdd_reg)
		regulator_disable(avdd_reg);

exit:
	return retval;
}

/**
 * syna_spi_get_regulator()
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
static int syna_spi_get_regulator(struct syna_hw_interface *hw_if,
		bool get)
{
	int retval;
	struct device *dev = syna_spi_device->dev.parent;
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

/**
 * syna_spi_enable_irq()
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
static int syna_spi_enable_irq(struct syna_hw_interface *hw_if,
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
			LOGI("Interrupt already enabled\n");
			retval = 0;
			goto exit;
		}

		enable_irq(attn->irq_id);
		attn->irq_enabled = true;

		LOGD("irq enabled\n");
	}
	/* disable the handling of interrupt */
	else {
		if (!attn->irq_enabled) {
			LOGI("Interrupt already disabled\n");
			retval = 0;
			goto exit;
		}

		disable_irq_nosync(attn->irq_id);
		attn->irq_enabled = false;

		LOGD("irq disabled\n");
	}

exit:
	syna_pal_mutex_unlock(&attn->irq_en_mutex);

	return retval;
}


/**
 * syna_spi_parse_dt()
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
static int syna_spi_parse_dt(struct syna_hw_interface *hw_if,
		struct device *dev)
{
	int retval;
	int index;
	u32 value;
	u32 coords[2];
	struct property *prop;
	struct device_node *np = dev->of_node;
	const char *name;
	struct syna_hw_attn_data *attn = &hw_if->bdata_attn;
	struct syna_hw_pwr_data *pwr = &hw_if->bdata_pwr;
	struct syna_hw_rst_data *rst = &hw_if->bdata_rst;
	struct syna_hw_bus_data *bus = &hw_if->bdata_io;
	struct of_phandle_args panelmap;
	struct drm_panel *panel = NULL;

#if IS_ENABLED(CONFIG_TOUCHSCREEN_OFFLOAD)
	u8 offload_id[4];
#endif

	if (of_property_read_bool(np, "synaptics,panel_map")) {
		for (index = 0 ;; index++) {
			retval = of_parse_phandle_with_fixed_args(np,
								  "synaptics,panel_map",
								  0,
								  index,
								  &panelmap);
			if (retval)
				return -EPROBE_DEFER;
			panel = of_drm_find_panel(panelmap.np);
			of_node_put(panelmap.np);
			if (!IS_ERR_OR_NULL(panel)) {
				retval = of_property_read_string_index(np,
								       "synaptics,firmware_names",
								       index, &name);
				if (retval < 0)
					hw_if->fw_name = FW_IMAGE_NAME;
				else
					hw_if->fw_name = name;
				LOGI("Firmware name %s", hw_if->fw_name);
				break;
			}
		}
	} else {
		hw_if->fw_name = FW_IMAGE_NAME;
	}

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

		pwr->power_on_delay_ms = value;

	} else {
		pwr->power_on_delay_ms = 0;
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

	prop = of_find_property(np, "synaptics,spi-byte-delay-us", NULL);
	if (prop && prop->length) {
		retval = of_property_read_u32(np,
				"synaptics,spi-byte-delay-us", &value);
		if (retval < 0) {
			LOGE("Fail to read byte-delay-us property\n");
			return retval;
		}

		bus->spi_byte_delay_us = value;

	} else {
		bus->spi_byte_delay_us = 0;
	}

	prop = of_find_property(np, "synaptics,spi-block-delay-us", NULL);
	if (prop && prop->length) {
		retval = of_property_read_u32(np,
				"synaptics,spi-block-delay-us", &value);
		if (retval < 0) {
			LOGE("Fail to read block-delay-us property\n");
			return retval;
		}
			bus->spi_block_delay_us = value;

	} else {
		bus->spi_block_delay_us = 0;
	}

	prop = of_find_property(np, "synaptics,spi-mode", NULL);
	if (prop && prop->length) {
		retval = of_property_read_u32(np, "synaptics,spi-mode",
				&value);
		if (retval < 0) {
			LOGE("Fail to read synaptics,spi-mode property\n");
			return retval;
		}

		bus->spi_mode = value;

	} else {
		bus->spi_mode = 0;
	}

	prop = of_find_property(np, "synaptics,pixels-per-mm", NULL);
	if (prop && prop->length) {
		retval = of_property_read_u32(np, "synaptics,pixels-per-mm",
				&value);
		if (retval < 0) {
			LOGE("Fail to read synaptics,pixels-per-mm\n");
			return retval;
		}

		hw_if->pixels_per_mm = value;

	} else {
		/*
		 * Set default as 1 to let the driver report the value from the
		 * touch IC if pixels_per_mm is not set.
		 */
		hw_if->pixels_per_mm = 1;
	}

	prop = of_find_property(np, "synaptics,compression-threshold", NULL);
	if (prop && prop->length) {
		retval = of_property_read_u32(np, "synaptics,compression-threshold",
				&value);
		if (retval < 0) {
			LOGE("Fail to read synaptics,compression-threshold\n");
			return retval;
		}

		hw_if->compression_threhsold = value;
	} else {
		/*
		 * Set default as 15.
		 */
		hw_if->compression_threhsold = 15;
	}

	prop = of_find_property(np, "synaptics,grip-delta-threshold", NULL);
	if (prop && prop->length) {
		retval = of_property_read_u32(np, "synaptics,grip-delta-threshold",
				&value);
		if (retval < 0) {
			LOGE("Fail to read synaptics,grip-delta-threshold\n");
			return retval;
		}

		hw_if->grip_delta_threshold = value;
	} else {
		/*
		 * Set default as 50.
		 */
		hw_if->grip_delta_threshold = 50;
	}

	prop = of_find_property(np, "synaptics,grip-border-threshold", NULL);
	if (prop && prop->length) {
		retval = of_property_read_u32(np, "synaptics,grip-border-threshold",
				&value);
		if (retval < 0) {
			LOGE("Fail to read synaptics,grip-border-threshold\n");
			return retval;
		}

		hw_if->grip_border_threshold = value;
	} else {
		/*
		 * Set default as 50.
		 */
		hw_if->grip_border_threshold = 50;
	}

	hw_if->dynamic_report_rate = of_property_read_bool(np,"synaptics,dynamic-report-rate");

#if IS_ENABLED(CONFIG_TOUCHSCREEN_OFFLOAD)
	hw_if->offload_id = 0;
	retval = of_property_read_u8_array(np, "synaptics,touch_offload_id",
					    offload_id, 4);
	if (retval == -EINVAL)
		dev_err(dev,
			"Failed to read synaptics,touch_offload_id with error = %d\n",
			retval);
	else {
		hw_if->offload_id = *(u32 *)offload_id;
		dev_info(dev, "Offload device ID = \"%c%c%c%c\" / 0x%08X\n",
		    offload_id[0], offload_id[1], offload_id[2], offload_id[3],
		    hw_if->offload_id);
	}
#endif

	if (of_property_read_u32_array(np, "synaptics,udfps-coords", coords, 2)) {
		dev_err(dev, "synaptics,udfps-coords not found\n");
		coords[0] = 0;
		coords[1] = 0;
	}
	hw_if->udfps_x = coords[0];
	hw_if->udfps_y = coords[1];

	return 0;
}
#endif

/**
 * syna_tcm_spi_alloc_mem()
 *
 * Manage and allocate the memory to buf being as a temporary buffer for IO
 *
 * @param
 *    [ in] count: number of spi_transfer structures to send
 *    [ in] size:  size of temporary buffer
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_spi_alloc_mem(unsigned int count, unsigned int size)
{
	static unsigned int xfer_count;

	if (count > xfer_count) {
		syna_pal_mem_free((void *)xfer);
		xfer = syna_pal_mem_alloc(count, sizeof(*xfer));
		if (!xfer) {
			LOGE("Fail to allocate memory for xfer\n");
			xfer_count = 0;
			return -ENOMEM;
		}
		xfer_count = count;
	} else {
		syna_pal_mem_set(xfer, 0, count * sizeof(*xfer));
	}

	if (size > buf_size) {
		if (buf_size)
			syna_pal_mem_free((void *)buf);
		buf = syna_pal_mem_alloc(size, sizeof(unsigned char));
		if (!buf) {
			LOGE("Fail to allocate memory for buf\n");
			buf_size = 0;
			return -ENOMEM;
		}
		buf_size = size;
	}

	return 0;
}


/**
 * syna_spi_read()
 *
 * TouchCom over SPI requires the host to assert the SSB signal to address
 * the device and retrieve the data.
 *
 * @param
 *    [ in] hw_if:   the handle of hw interface
 *    [out] rd_data: buffer for storing data retrieved from device
 *    [ in] rd_len: number of bytes retrieved from device
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_spi_read(struct syna_hw_interface *hw_if,
		unsigned char *rd_data, unsigned int rd_len)
{
	int retval;
	unsigned int idx;
	struct spi_message msg;
	struct spi_device *spi = hw_if->pdev;
	struct syna_hw_bus_data *bus = &hw_if->bdata_io;

	if (!spi) {
		LOGE("Invalid bus io device\n");
		return -EINVAL;
	}

	syna_pal_mutex_lock(&bus->io_mutex);

	spi_message_init(&msg);

	if (bus->spi_byte_delay_us == 0)
		retval = syna_spi_alloc_mem(1, rd_len);
	else
		retval = syna_spi_alloc_mem(rd_len, 1);
	if (retval < 0) {
		LOGE("Fail to allocate memory\n");
		goto exit;
	}

	if (bus->spi_byte_delay_us == 0) {
		syna_pal_mem_set(buf, 0xff, rd_len);
		xfer[0].len = rd_len;
		xfer[0].tx_buf = buf;
		xfer[0].rx_buf = rd_data;
		if (bus->spi_block_delay_us) {
			xfer[0].delay.unit = SPI_DELAY_UNIT_USECS;
			xfer[0].delay.value = bus->spi_block_delay_us;
		}
		spi_message_add_tail(&xfer[0], &msg);
	} else {
		buf[0] = 0xff;
		for (idx = 0; idx < rd_len; idx++) {
			xfer[idx].len = 1;
			xfer[idx].tx_buf = buf;
			xfer[idx].rx_buf = &rd_data[idx];
			xfer[idx].delay.unit = SPI_DELAY_UNIT_USECS;
			xfer[idx].delay.value =  bus->spi_byte_delay_us;
			if (bus->spi_block_delay_us && (idx == rd_len - 1)) {
				xfer[idx].delay.unit = SPI_DELAY_UNIT_USECS;
				xfer[idx].delay.value =  bus->spi_block_delay_us;
			}
			spi_message_add_tail(&xfer[idx], &msg);
		}
	}

	retval = spi_sync(spi, &msg);
	if (retval != 0) {
		LOGE("Failed to complete SPI transfer, error = %d\n", retval);
		goto exit;
	}

	retval = rd_len;

exit:
	syna_pal_mutex_unlock(&bus->io_mutex);

	return retval;
}

/**
 * syna_spi_write()
 *
 * TouchCom over SPI requires the host to assert the SSB signal to address
 * the device and send the data to the device.
 *
 * @param
 *    [ in] hw_if:   the handle of hw interface
 *    [ in] wr_data: written data
 *    [ in] wr_len: length of written data in bytes
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_spi_write(struct syna_hw_interface *hw_if,
		unsigned char *wr_data, unsigned int wr_len)
{
	int retval;
	unsigned int idx;
	struct spi_message msg;
	struct spi_device *spi = hw_if->pdev;
	struct syna_hw_bus_data *bus = &hw_if->bdata_io;

	if (!spi) {
		LOGE("Invalid bus io device\n");
		return -EINVAL;
	}

	syna_pal_mutex_lock(&bus->io_mutex);

	spi_message_init(&msg);

	if (bus->spi_byte_delay_us == 0)
		retval = syna_spi_alloc_mem(1, 0);
	else
		retval = syna_spi_alloc_mem(wr_len, 0);
	if (retval < 0) {
		LOGE("Failed to allocate memory\n");
		goto exit;
	}

	if (bus->spi_byte_delay_us == 0) {
		xfer[0].len = wr_len;
		xfer[0].tx_buf = wr_data;
		if (bus->spi_block_delay_us) {
			xfer[0].delay.unit = SPI_DELAY_UNIT_USECS;
			xfer[0].delay.value = bus->spi_block_delay_us;
		}
		spi_message_add_tail(&xfer[0], &msg);
	} else {
		for (idx = 0; idx < wr_len; idx++) {
			xfer[idx].len = 1;
			xfer[idx].tx_buf = &wr_data[idx];
			xfer[idx].delay.unit = SPI_DELAY_UNIT_USECS;
			xfer[idx].delay.value = bus->spi_byte_delay_us;
			if (bus->spi_block_delay_us && (idx == wr_len - 1)) {
				xfer[idx].delay.unit = SPI_DELAY_UNIT_USECS;
				xfer[idx].delay.value = bus->spi_block_delay_us;
			}
			spi_message_add_tail(&xfer[idx], &msg);
		}
	}

	retval = spi_sync(spi, &msg);
	if (retval != 0) {
		LOGE("Fail to complete SPI transfer, error = %d\n", retval);
		goto exit;
	}

	retval = wr_len;

exit:
	syna_pal_mutex_unlock(&bus->io_mutex);

	return retval;
}


/**
 * syna_hw_interface
 *
 * Provide the hardware specific settings in defaults.
 * Be noted the followings could be changed after .dtsi is parsed
 */
static struct syna_hw_interface syna_spi_hw_if = {
	.bdata_io = {
		.type = BUS_TYPE_SPI,
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
		.power_on_delay_ms = 200,
	},
	.ops_power_on = syna_spi_enable_regulator,
	.ops_hw_reset = syna_spi_hw_reset,
	.ops_read_data = syna_spi_read,
	.ops_write_data = syna_spi_write,
	.ops_enable_irq = syna_spi_enable_irq,
};

/**
 * syna_spi_probe()
 *
 * Prepare the specific hardware interface and register the platform spi device
 *
 * @param
 *    [ in] spi: spi device
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_spi_probe(struct spi_device *spi)
{
	int retval;
	struct syna_hw_attn_data *attn = &syna_spi_hw_if.bdata_attn;
	struct syna_hw_bus_data *bus = &syna_spi_hw_if.bdata_io;

	if (spi->master->flags & SPI_MASTER_HALF_DUPLEX) {
		LOGE("Full duplex not supported by host\n");
		return -EIO;
	}

	/* allocate an spi platform device */
	syna_spi_device = platform_device_alloc(PLATFORM_DRIVER_NAME, 0);
	if (!syna_spi_device) {
		LOGE("Fail to allocate platform device\n");
		return _ENODEV;
	}

#ifdef CONFIG_OF
	syna_spi_parse_dt(&syna_spi_hw_if, &spi->dev);
#endif

	syna_pal_mutex_alloc(&attn->irq_en_mutex);
	syna_pal_mutex_alloc(&bus->io_mutex);

	switch (bus->spi_mode) {
	case 0:
		spi->mode = SPI_MODE_0;
		break;
	case 1:
		spi->mode = SPI_MODE_1;
		break;
	case 2:
		spi->mode = SPI_MODE_2;
		break;
	case 3:
		spi->mode = SPI_MODE_3;
		break;
	}

	/* keep the i/o device */
	syna_spi_hw_if.pdev = spi;

	syna_spi_device->dev.parent = &spi->dev;
	syna_spi_device->dev.platform_data = &syna_spi_hw_if;

	spi->bits_per_word = 8;
	spi->rt = true;

	/* set up spi driver */
	retval = spi_setup(spi);
	if (retval < 0) {
		LOGE("Fail to set up SPI protocol driver\n");
		return retval;
	}

	/* enable the regulators */
	retval = syna_spi_get_regulator(&syna_spi_hw_if, true);
	if (retval < 0)
		return retval;

	/* initialize the gpio pins */
	retval = syna_spi_config_gpio(&syna_spi_hw_if);
	if (retval < 0) {
		LOGE("Fail to config gpio\n");
		return retval;
	}

	/* register the spi platform device */
	retval = platform_device_add(syna_spi_device);
	if (retval < 0) {
		LOGE("Fail to add platform device\n");
		return retval;
	}

	return 0;
}

/**
 * syna_spi_remove()
 *
 * Unregister the platform spi device
 *
 * @param
 *    [ in] spi: spi device
 */
static void syna_spi_remove(struct spi_device *spi)
{
	struct syna_hw_attn_data *attn = &syna_spi_hw_if.bdata_attn;
	struct syna_hw_pwr_data *pwr = &syna_spi_hw_if.bdata_pwr;
	struct syna_hw_rst_data *rst = &syna_spi_hw_if.bdata_rst;
	struct syna_hw_bus_data *bus = &syna_spi_hw_if.bdata_io;

	/* disable gpios */
	if (pwr->avdd_gpio >= 0)
		syna_spi_request_gpio(pwr->avdd_gpio, false, 0, 0, NULL);
	if (pwr->vdd_gpio >= 0)
		syna_spi_request_gpio(pwr->vdd_gpio, false, 0, 0, NULL);
	if (rst->reset_gpio >= 0)
		syna_spi_request_gpio(rst->reset_gpio, false, 0, 0, NULL);
	if (attn->irq_gpio >= 0)
		syna_spi_request_gpio(attn->irq_gpio, false, 0, 0, NULL);

	/* disable the regulators */
	syna_spi_get_regulator(&syna_spi_hw_if, false);

	syna_pal_mutex_free(&attn->irq_en_mutex);
	syna_pal_mutex_free(&bus->io_mutex);

	/* remove the platform device */
	syna_spi_device->dev.platform_data = NULL;
	platform_device_unregister(syna_spi_device);
}

/**
 * Describe an spi device driver and its related declarations
 */
static const struct spi_device_id syna_spi_id_table[] = {
	{SPI_MODULE_NAME, 0},
	{},
};
MODULE_DEVICE_TABLE(spi, syna_spi_id_table);

#ifdef CONFIG_OF
static const struct of_device_id syna_spi_of_match_table[] = {
	{
		.compatible = "synaptics,tcm-spi",
	},
	{},
};
MODULE_DEVICE_TABLE(of, syna_spi_of_match_table);
#else
#define syna_spi_of_match_table NULL
#endif

static struct spi_driver syna_spi_driver = {
	.driver = {
		.name = SPI_MODULE_NAME,
		.owner = THIS_MODULE,
		.of_match_table = syna_spi_of_match_table,
	},
	.probe = syna_spi_probe,
	.remove = syna_spi_remove,
	.id_table = syna_spi_id_table,
};


/**
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
	return spi_register_driver(&syna_spi_driver);
}

/**
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
	syna_pal_mem_free((void *)buf);

	syna_pal_mem_free((void *)xfer);

	spi_unregister_driver(&syna_spi_driver);
}

MODULE_AUTHOR("Synaptics, Inc.");
MODULE_DESCRIPTION("Synaptics TouchCom SPI Bus Module");
MODULE_LICENSE("GPL v2");

