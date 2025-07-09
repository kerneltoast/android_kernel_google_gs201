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
 * @file syna_tcm2_sysfs.c
 *
 * This file implements sysfs attributes in the reference driver.
 */

#include <linux/string.h>

#include "syna_tcm2.h"
#include "synaptics_touchcom_core_dev.h"
#include "synaptics_touchcom_func_base.h"
#ifdef HAS_TESTING_FEATURE
#include "syna_tcm2_testing.h"
#endif

/* g_sysfs_dir represents the root directory of sysfs nodes being created
 */
static struct kobject *g_sysfs_dir;

/*
 * syna_get_fw_info()
 *
 * Output the device and driver information.
 *
 * @param
 *    [ in] tcm: the driver handle
 *    [out] buf:  string buffer for the firmware and the driver information
 *    [ in] buf_size: size of the buf
 *
 * @return
 *    on success, number of characters being output;
 *    otherwise, negative value on error.
 */
ssize_t syna_get_fw_info(struct syna_tcm *tcm, char *buf, size_t buf_size)
{
	int retval;
	int i;
	unsigned int count;
	struct tcm_dev *tcm_dev;

	tcm_dev = tcm->tcm_dev;

	count = 0;

	retval = scnprintf(buf, buf_size - count,
			"Driver version:     %d.%s\n",
			SYNAPTICS_TCM_DRIVER_VERSION,
			SYNAPTICS_TCM_DRIVER_SUBVER);
	if (retval < 0)
		goto exit;

	buf += retval;
	count += retval;

	retval = scnprintf(buf, buf_size - count,
			"Core lib version:   %d.%02d\n\n",
			(unsigned char)(SYNA_TCM_CORE_LIB_VERSION >> 8),
			(unsigned char)SYNA_TCM_CORE_LIB_VERSION);
	if (retval < 0)
		goto exit;

	buf += retval;
	count += retval;

	if (!tcm->is_connected) {
		retval = scnprintf(buf, buf_size - count,
				"Device is NOT connected\n");
		count += retval;
		retval = count;
		goto exit;
	}

	if (tcm->pwr_state == BARE_MODE) {
		retval = count;
		goto exit;
	}

	retval = scnprintf(buf, buf_size - count,
			"TouchComm version:  %d\n", tcm_dev->id_info.version);
	if (retval < 0)
		goto exit;

	buf += retval;
	count += retval;

	switch (tcm_dev->id_info.mode) {
	case MODE_APPLICATION_FIRMWARE:
		retval = scnprintf(buf, buf_size - count,
				"Firmware mode:      Application Firmware, 0x%02x\n",
				tcm_dev->id_info.mode);
		if (retval < 0)
			goto exit;
		break;
	case MODE_BOOTLOADER:
		retval = scnprintf(buf, buf_size - count,
				"Firmware mode:      Bootloader, 0x%02x\n",
				tcm_dev->id_info.mode);
		if (retval < 0)
			goto exit;
		break;
	case MODE_ROMBOOTLOADER:
		retval = scnprintf(buf, buf_size - count,
				"Firmware mode:      Rom Bootloader, 0x%02x\n",
				tcm_dev->id_info.mode);
		if (retval < 0)
			goto exit;
		break;
	default:
		retval = scnprintf(buf, buf_size - count,
				"Firmware mode:      Mode 0x%02x\n",
				tcm_dev->id_info.mode);
		if (retval < 0)
			goto exit;
		break;
	}
	buf += retval;
	count += retval;

	retval = scnprintf(buf, buf_size - count,
			"Part number:        %s", tcm_dev->id_info.part_number);
	if (retval < 0)
		goto exit;

	buf += retval;
	count += retval;

	retval = scnprintf(buf, buf_size - count, "\n");
	if (retval < 0)
		goto exit;

	buf += retval;
	count += retval;

	retval = scnprintf(buf, buf_size - count,
			"Packrat number:     %d\n\n", tcm_dev->packrat_number);
	if (retval < 0)
		goto exit;

	buf += retval;
	count += retval;

	if (tcm_dev->id_info.mode != MODE_APPLICATION_FIRMWARE) {
		retval = count;
		goto exit;
	}

	retval = scnprintf(buf, buf_size - count, "Config ID:          ");
	if (retval < 0)
		goto exit;

	buf += retval;
	count += retval;

	for (i = 0; i < MAX_SIZE_CONFIG_ID; i++) {
		retval = scnprintf(buf, buf_size - count,
			"0x%2x ", tcm_dev->config_id[i]);
		if (retval < 0)
			goto exit;
		buf += retval;
		count += retval;
	}

	retval = scnprintf(buf, buf_size - count, "\n");
	if (retval < 0)
		goto exit;

	buf += retval;
	count += retval;

	retval = scnprintf(buf, buf_size - count,
		"Max X & Y:          %d, %d\n", tcm_dev->max_x, tcm_dev->max_y);
	if (retval < 0)
		goto exit;

	buf += retval;
	count += retval;

	retval = scnprintf(buf, buf_size - count,
		"Num of objects:     %d\n", tcm_dev->max_objects);
	if (retval < 0)
		goto exit;

	buf += retval;
	count += retval;

	retval = scnprintf(buf, buf_size - count,
		"Num of cols & rows: %d, %d\n", tcm_dev->cols, tcm_dev->rows);
	if (retval < 0)
		goto exit;

	buf += retval;
	count += retval;

	retval = snprintf(buf, buf_size - count,
		"Max. Read Size:     %d bytes\n", tcm_dev->max_rd_size);
	if (retval < 0)
		goto exit;

	buf += retval;
	count += retval;

	retval = snprintf(buf, buf_size - count,
		"Max. Write Size:    %d bytes\n", tcm_dev->max_wr_size);
	if (retval < 0)
		goto exit;

	buf += retval;
	count += retval;

	retval = count;

exit:
	if (retval < 0)
		LOGE("Failed to get firmware info");

	return retval;
}
/*
 * syna_sysfs_info_show()
 *
 * Attribute to show the device and driver information to the console.
 *
 * @param
 *    [ in] kobj:  an instance of kobj
 *    [ in] attr:  an instance of kobj attribute structure
 *    [out] buf:  string buffer shown on console
 *
 * @return
 *    on success, number of characters being output;
 *    otherwise, negative value on error.
 */
static ssize_t syna_sysfs_info_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int retval = 0;
	struct device *p_dev;
	struct kobject *p_kobj;
	struct syna_tcm *tcm;

	p_kobj = g_sysfs_dir->parent;
	p_dev = container_of(p_kobj, struct device, kobj);
	tcm = dev_get_drvdata(p_dev);

	if (!tcm || !tcm->tcm_dev)
		return -ENODEV;

	retval = syna_tcm_identify(tcm->tcm_dev, &tcm->tcm_dev->id_info);
	if (retval < 0) {
		LOGE("Fail to get identification\n");
		return retval;
	}

	/* collect app info containing most of sensor information */
	retval = syna_tcm_get_app_info(tcm->tcm_dev, &tcm->tcm_dev->app_info);
	if (retval < 0) {
		LOGE("Fail to get application info\n");
		return retval;
	}

	return syna_get_fw_info(tcm, buf, PAGE_SIZE);
}

static struct kobj_attribute kobj_attr_info =
	__ATTR(info, 0444, syna_sysfs_info_show, NULL);

/*
 * syna_sysfs_irq_en_store()
 *
 * Attribute to disable/enable the irq
 *
 * @param
 *    [ in] kobj:  an instance of kobj
 *    [ in] attr:  an instance of kobj attribute structure
 *    [ in] buf:   string buffer input
 *    [ in] count: size of buffer input
 *
 * @return
 *    on success, return count; otherwise, return error code
 */
static ssize_t syna_sysfs_irq_en_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	int retval = 0;
	unsigned int input;
	struct device *p_dev;
	struct kobject *p_kobj;
	struct syna_tcm *tcm;

	p_kobj = g_sysfs_dir->parent;
	p_dev = container_of(p_kobj, struct device, kobj);
	tcm = dev_get_drvdata(p_dev);

	if (kstrtouint(buf, 10, &input))
		return -EINVAL;

	if (!tcm->hw_if->ops_enable_irq)
		return 0;

	if (!tcm->is_connected) {
		LOGW("Device is NOT connected\n");
		return count;
	}

	if (tcm->pwr_state == BARE_MODE) {
		LOGN("In bare connection mode, no irq support\n");
		retval = count;
		goto exit;
	}

	/* disable the interrupt line */
	if (input == 0) {
		retval = tcm->hw_if->ops_enable_irq(tcm->hw_if, false);
		if (retval < 0) {
			LOGE("Fail to disable interrupt\n");
			goto exit;
		}
	} else if (input == 1) {
	/* enable the interrupt line */
		retval = tcm->hw_if->ops_enable_irq(tcm->hw_if, true);
		if (retval < 0) {
			LOGE("Fail to enable interrupt\n");
			goto exit;
		}
	} else {
		LOGW("Unknown option %d (0:disable / 1:enable)\n", input);
		retval = -EINVAL;
		goto exit;
	}

	retval = count;

exit:
	return retval;
}

static struct kobj_attribute kobj_attr_irq_en =
	__ATTR(irq_en, 0220, NULL, syna_sysfs_irq_en_store);

/*
 * syna_sysfs_int2_store()
 *
 * Attribute to set int2.
 *
 * @param
 *    [ in] kobj:  an instance of kobj
 *    [ in] attr:  an instance of kobj attribute structure
 *    [ in] buf:   string buffer input
 *    [ in] count: size of buffer input
 *
 * @return
 *    on success, return count; otherwise, return error code
 */
static ssize_t syna_sysfs_int2_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	int retval = 0;
	u16 config;
	unsigned int input;
	struct device *p_dev;
	struct kobject *p_kobj;
	struct syna_tcm *tcm;

	p_kobj = g_sysfs_dir->parent;
	p_dev = container_of(p_kobj, struct device, kobj);
	tcm = dev_get_drvdata(p_dev);

	if (kstrtouint(buf, 10, &input))
		return -EINVAL;

	if (!tcm->is_connected) {
		LOGW("Device is NOT connected\n");
		return count;
	}

	/* Set int2 production mode disabled. */
	if (input == 0) {
		config = INT2_PRODUCTION_DISABLE;
		LOGI("Set INT2 production mode disabled");
	} else if (input == 1) {
	/*  Set int2 as high. */
		config = INT2_PRODUCTION_HIGH;
		LOGI("Set INT2 production mode high");
	} else if (input == 3) {
	/*  Set int2 as low. */
		config = INT2_PRODUCTION_LOW;
		LOGI("Set INT2 production mode low");
	} else {
		LOGE("Unknown option.");
		goto exit;
	}

	syna_tcm_set_dynamic_config(tcm->tcm_dev,
			DC_INT2_PRODUCTION_CMD,
			config,
			RESP_IN_ATTN);

exit:
	retval = count;
	return retval;
}

/*
 * syna_sysfs_int2_show()
 *
 * Attribute to show the int2 status.
 *
 * @param
 *    [ in] kobj:  an instance of kobj
 *    [ in] attr:  an instance of kobj attribute structure
 *    [out] buf:  string buffer shown on console
 *
 * @return
 *    on success, number of characters being output;
 *    otherwise, negative value on error.
 */
static ssize_t syna_sysfs_int2_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int retval = 0;
	u16 config;
	struct device *p_dev;
	struct kobject *p_kobj;
	struct syna_tcm *tcm;

	p_kobj = g_sysfs_dir->parent;
	p_dev = container_of(p_kobj, struct device, kobj);
	tcm = dev_get_drvdata(p_dev);

	tcm->hw_if->ops_enable_irq(tcm->hw_if, false);

	retval = syna_tcm_get_dynamic_config(tcm->tcm_dev, DC_INT2_PRODUCTION_CMD,
			&config, RESP_IN_POLLING);

	tcm->hw_if->ops_enable_irq(tcm->hw_if, true);

	if (retval < 0) {
		retval = scnprintf(buf, PAGE_SIZE, "Read failure.\n");
	} else {
		if (config == INT2_PRODUCTION_DISABLE)
			retval = scnprintf(buf, PAGE_SIZE, "Disabled\n");
		else if (config == INT2_PRODUCTION_HIGH)
			retval = scnprintf(buf, PAGE_SIZE, "High\n");
		else if (config == INT2_PRODUCTION_LOW)
			retval = scnprintf(buf, PAGE_SIZE, "Low\n");
		else
			retval = scnprintf(buf, PAGE_SIZE, "Unknown value %u\n", config);
	}

	return retval;
}

static struct kobj_attribute kobj_attr_int2 =
	__ATTR(int2, 0644, syna_sysfs_int2_show, syna_sysfs_int2_store);

/*
 * syna_sysfs_reset_store()
 *
 * Attribute to issue a reset.
 * "1" for a sw reset; "2" for a hardware reset
 *
 * @param
 *    [ in] kobj:  an instance of kobj
 *    [ in] attr:  an instance of kobj attribute structure
 *    [ in] buf:   string buffer input
 *    [ in] count: size of buffer input
 *
 * @return
 *    on success, return count; otherwise, return error code
 */
static ssize_t syna_sysfs_reset_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	int retval = 0;
	unsigned int input;
	struct device *p_dev;
	struct kobject *p_kobj;
	struct syna_tcm *tcm;

	p_kobj = g_sysfs_dir->parent;
	p_dev = container_of(p_kobj, struct device, kobj);
	tcm = dev_get_drvdata(p_dev);

	if (kstrtouint(buf, 10, &input))
		return -EINVAL;

	if (!tcm->is_connected) {
		LOGW("Device is NOT connected\n");
		return count;
	}

	if ((tcm->pwr_state == BARE_MODE) || (input == 2)) {
		if (!tcm->hw_if->ops_hw_reset) {
			LOGE("No hardware reset support\n");
			goto exit;
		}

		tcm->hw_if->ops_hw_reset(tcm->hw_if);

		/* enable the interrupt to process the identify report
		 * after the hardware reset.
		 */
		if (!tcm->hw_if->bdata_attn.irq_enabled) {
			tcm->hw_if->ops_enable_irq(tcm->hw_if, true);
			/* disable it and back to original status */
			syna_pal_sleep_ms(100);
			tcm->hw_if->ops_enable_irq(tcm->hw_if, false);
		}
	} else if (input == 1) {
		retval = syna_tcm_reset(tcm->tcm_dev);
		if (retval < 0) {
			LOGE("Fail to do reset\n");
			goto exit;
		}
	} else {
		LOGW("Unknown option %d (1:sw / 2:hw)\n", input);
		retval = -EINVAL;
		goto exit;
	}

	/* check the fw setup in case the settings is changed */
	if (IS_APP_FW_MODE(tcm->tcm_dev->dev_mode)) {
		retval = tcm->dev_set_up_app_fw(tcm);
		if (retval < 0) {
			LOGE("Fail to set up app fw\n");
			goto exit;
		}
	}

	retval = count;

exit:
	return retval;
}

static struct kobj_attribute kobj_attr_reset =
	__ATTR(reset, 0220, NULL, syna_sysfs_reset_store);


/*
 * syna_sysfs_pwr_store()
 *
 * Attribute to change the power state.
 *
 * @param
 *    [ in] kobj:  an instance of kobj
 *    [ in] attr:  an instance of kobj attribute structure
 *    [ in] buf:   string buffer input
 *    [ in] count: size of buffer input
 *
 * @return
 *    on success, return count; otherwise, return error code
 */
static ssize_t syna_sysfs_pwr_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	int retval = 0;
	struct device *p_dev;
	struct kobject *p_kobj;
	struct syna_tcm *tcm;

	p_kobj = g_sysfs_dir->parent;
	p_dev = container_of(p_kobj, struct device, kobj);
	tcm = dev_get_drvdata(p_dev);

	if (!tcm->is_connected) {
		LOGW("Device is NOT connected\n");
		return count;
	}

	if (strncmp(buf, "resume", 6) == 0) {
		if (tcm->dev_resume)
			tcm->dev_resume(p_dev);
	} else if (strncmp(buf, "suspend", 7) == 0) {
		if (tcm->dev_suspend)
			tcm->dev_suspend(p_dev);
	} else {
		LOGW("Unknown option %s\n", buf);
		retval = -EINVAL;
		goto exit;
	}

	retval = count;

exit:
	return retval;
}

static struct kobj_attribute kobj_attr_pwr =
	__ATTR(power_state, 0220, NULL, syna_sysfs_pwr_store);

/*
 * syna_sysfs_scan_mode_store()
 *
 * Attribute to set different scan mode.
 * 0 - Lock Normal Mode Active Mode.
 * 1 - Lock Normal Mode Doze Mode.
 * 2 - Lock Low Power Gesture Mode Active Mode.
 * 3 - Lock Low Power Gesture Mode Doze Mode.
 *
 * @param
 *    [ in] kobj:  an instance of kobj
 *    [ in] attr:  an instance of kobj attribute structure
 *    [ in] buf:   string buffer input
 *    [ in] count: size of buffer input
 *
 * @return
 *    on success, return count; otherwise, return error code
 */
static ssize_t syna_sysfs_scan_mode_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	int retval = 0;
	unsigned int input;
	unsigned char command = 0;
	struct device *p_dev;
	struct kobject *p_kobj;
	struct syna_tcm *tcm;
	struct syna_hw_interface *hw_if;

	p_kobj = g_sysfs_dir->parent;
	p_dev = container_of(p_kobj, struct device, kobj);
	tcm = dev_get_drvdata(p_dev);
	hw_if = tcm->hw_if;

	if (kstrtouint(buf, 10, &input))
		return -EINVAL;

	if (!tcm->is_connected) {
		LOGW("Device is NOT connected\n");
		retval = count;
		goto exit;
	}

	if (hw_if->ops_hw_reset) {
		hw_if->ops_hw_reset(hw_if);
	} else {
		retval = syna_tcm_reset(tcm->tcm_dev);
		if (retval < 0) {
			LOGE("Fail to do reset\n");
			goto exit;
		}
	}

	if (input == 0 || input == 2) {
		command = DC_DISABLE_DOZE;
	} else if (input == 1 || input == 3) {
		command = DC_FORCE_DOZE_MODE;
	} else {
		LOGW("Unsupport command %u\n", input);
		goto exit;
	}

	if (input == 2 || input == 3) {
		retval = syna_tcm_set_dynamic_config(tcm->tcm_dev,
				DC_ENABLE_WAKEUP_GESTURE_MODE,
				1,
				RESP_IN_ATTN);
		if (retval < 0) {
			LOGE("Fail to enable wakeup gesture via DC command\n");
			goto exit;
		}
	}

	retval = syna_tcm_set_dynamic_config(tcm->tcm_dev,
			command,
			1,
			RESP_IN_ATTN);
	if (retval < 0) {
		LOGE("Fail to set DC command %d\n", command);
		goto exit;
	}

	retval = count;

exit:
	return retval;
}

static struct kobj_attribute kobj_attr_scan_mode =
	__ATTR(scan_mode, 0220, NULL, syna_sysfs_scan_mode_store);

#if IS_ENABLED(CONFIG_GOOG_TOUCH_INTERFACE)
/*
 * syna_sysfs_force_active_store()
 *
 * Attribute to set different scan mode.
 * 0x10 - Set SYNA_BUS_REF_FORCE_ACTIVE bit 0.
 * 0x11 - Set SYNA_BUS_REF_FORCE_ACTIVE bit 1.
 * 0x20 - Set SYNA_BUS_REF_BUGREPORT bit 0.
 * 0x21 - Set SYNA_BUS_REF_BUGREPORT bit 1.
 *
 * @param
 *    [ in] kobj:  an instance of kobj
 *    [ in] attr:  an instance of kobj attribute structure
 *    [ in] buf:   string buffer input
 *    [ in] count: size of buffer input
 *
 * @return
 *    on success, return count; otherwise, return error code
 */
static ssize_t syna_sysfs_force_active_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	int retval = 0;
	unsigned char input;
	struct device *p_dev;
	struct kobject *p_kobj;
	struct syna_tcm *tcm;
	bool active;
	u32 ref = 0;

	p_kobj = g_sysfs_dir->parent;
	p_dev = container_of(p_kobj, struct device, kobj);
	tcm = dev_get_drvdata(p_dev);

	if (kstrtou8(buf, 16, &input))
		return -EINVAL;

	if (!tcm->is_connected) {
		LOGW("Device is NOT connected\n");
		retval = count;
		goto exit;
	}

	switch (input) {
	case 0x10:
		ref = GTI_PM_WAKELOCK_TYPE_FORCE_ACTIVE;
		active = false;
		break;
	case 0x11:
		ref = GTI_PM_WAKELOCK_TYPE_FORCE_ACTIVE;
		active = true;
		break;
	case 0x20:
		ref = GTI_PM_WAKELOCK_TYPE_BUGREPORT;
		active = false;
		break;
	case 0x21:
		ref = GTI_PM_WAKELOCK_TYPE_BUGREPORT;
		active = true;
		break;
	default:
		LOGE("Invalid input %#x.\n", input);
		retval = -EINVAL;
		goto exit;
	}

	LOGI("Set pm wake bit %#x %s.", ref,
	     active ? "enable" : "disable");

	if (active)
		retval = goog_pm_wake_lock(tcm->gti, ref, false);
	else
		retval = goog_pm_wake_unlock_nosync(tcm->gti, ref);

	if (retval < 0) {
		LOGE("Set pm wake bit %#x %s failed.", ref,
				active ? "enable" : "disable");
		goto exit;
	}

	retval = count;

exit:
	return retval;
}

static struct kobj_attribute kobj_attr_force_active =
	__ATTR(force_active, 0220, NULL, syna_sysfs_force_active_store);
#endif

/*
 * syna_sysfs_get_raw_data_show()
 *
 * Attribute to show the rawdata.
 *
 * @param
 *    [ in] kobj:  an instance of kobj
 *    [ in] attr:  an instance of kobj attribute structure
 *    [out] buf:  string buffer shown on console
 *
 * @return
 *    on success, number of characters being output;
 *    otherwise, negative value on error.
 */
static ssize_t syna_sysfs_get_raw_data_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int retval = 0;
	unsigned int count = 0;
	struct device *p_dev;
	struct kobject *p_kobj;
	struct syna_tcm *tcm;
	struct tcm_dev *tcm_dev;
	int i, j, mutual_length;
	bool is_signed;

	p_kobj = g_sysfs_dir->parent;
	p_dev = container_of(p_kobj, struct device, kobj);
	tcm = dev_get_drvdata(p_dev);
	tcm_dev = tcm->tcm_dev;
	mutual_length = tcm_dev->cols * tcm_dev->rows;
	is_signed = (tcm->raw_data_report_code == REPORT_DELTA);

	if (wait_for_completion_timeout(&tcm->raw_data_completion,
					msecs_to_jiffies(500)) == 0) {
		complete_all(&tcm->raw_data_completion);
		count += scnprintf(buf + count, PAGE_SIZE - count, "Timeout\n");
		goto exit;
	}

	if (!tcm->raw_data_buffer) {
		count += scnprintf(buf + count, PAGE_SIZE - count,
				   "Raw data buffer is NULL.\n");
		goto exit;
	}

	syna_pal_mutex_lock(&tcm->raw_data_mutex);
	/* Mutual raw. */
	count += scnprintf(buf + count, PAGE_SIZE - count, "Mutual\n");
	for (i = 0; i < tcm_dev->rows; i++) {
		for (j = 0; j < tcm_dev->cols; j++) {
			count += scnprintf(buf + count, PAGE_SIZE - count,
				(is_signed) ? "%d " : "%u ",
				(is_signed) ? tcm->raw_data_buffer[i * tcm_dev->cols + j] :
					      (u16) (tcm->raw_data_buffer[i * tcm_dev->cols + j]));
		}
		count += scnprintf(buf + count, PAGE_SIZE - count, "\n");
	}

	/* Self raw. */
	count += scnprintf(buf + count, PAGE_SIZE - count, "Self\n");
	for (i = 0; i < tcm_dev->cols; i++) {
		count += scnprintf(buf + count, PAGE_SIZE - count,
			(is_signed) ? "%d " : "%u ",
			(is_signed) ? tcm->raw_data_buffer[mutual_length + i] :
				      (u16) (tcm->raw_data_buffer[mutual_length + i]));
	}
	count += scnprintf(buf + count, PAGE_SIZE - count, "\n");

	for (j = 0; j < tcm_dev->rows; j++) {
		count += scnprintf(buf + count, PAGE_SIZE - count,
			(is_signed) ? "%d " : "%u ",
			(is_signed) ? tcm->raw_data_buffer[mutual_length + i + j] :
				      (u16) (tcm->raw_data_buffer[mutual_length + i + j]));
	}
	count += scnprintf(buf + count, PAGE_SIZE - count, "\n");

	syna_pal_mutex_unlock(&tcm->raw_data_mutex);

	LOGI("Got raw data, report code %#x\n", tcm->raw_data_report_code);

exit:
	retval = count;
	syna_tcm_set_dynamic_config(tcm->tcm_dev, DC_DISABLE_DOZE, 0, RESP_IN_ATTN);
	syna_tcm_enable_report(tcm_dev, tcm->raw_data_report_code, false);
	return retval;
}

/*
 * syna_sysfs_get_raw_data_store()
 *
 * Attribute to enable the rawdata report type.
 *
 * @param
 *    [ in] kobj:  an instance of kobj
 *    [ in] attr:  an instance of kobj attribute structure
 *    [ in] buf:   string buffer input
 *    [ in] count: size of buffer input
 *
 * @return
 *    on success, return count; otherwise, return error code
 */
static ssize_t syna_sysfs_get_raw_data_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	int retval = count;
	unsigned char input;
	unsigned char report_code;
	struct device *p_dev;
	struct kobject *p_kobj;
	struct syna_tcm *tcm;

	p_kobj = g_sysfs_dir->parent;
	p_dev = container_of(p_kobj, struct device, kobj);
	tcm = dev_get_drvdata(p_dev);

	if (kstrtou8(buf, 16, &input))
		return -EINVAL;

	switch (input) {
	case REPORT_DELTA:
		report_code = REPORT_DELTA;
		break;
	case REPORT_RAW:
		report_code = REPORT_RAW;
		break;
	case REPORT_BASELINE:
		report_code = REPORT_BASELINE;
		break;
	default:
		LOGE("Invalid input %#x.\n", input);
		retval = -EINVAL;
		goto exit;
	}

	LOGI("Enable raw data, report code %#x\n", report_code);

	syna_tcm_set_dynamic_config(tcm->tcm_dev, DC_DISABLE_DOZE, 1, RESP_IN_ATTN);

	tcm->raw_data_report_code = report_code;
	syna_tcm_enable_report(tcm->tcm_dev, report_code, true);
	reinit_completion(&tcm->raw_data_completion);

exit:
	return retval;
}

static struct kobj_attribute kobj_attr_get_raw_data =
	__ATTR(get_raw_data, 0644, syna_sysfs_get_raw_data_show, syna_sysfs_get_raw_data_store);

/*
 * syna_sysfs_high_sensitivity_show()
 *
 * Attribute to show current sensitivity mode.
 *
 * @param
 *    [ in] kobj:  an instance of kobj
 *    [ in] attr:  an instance of kobj attribute structure
 *    [out] buf:  string buffer shown on console
 *
 * @return
 *    on success, number of characters being output;
 *    otherwise, negative value on error.
 */
static ssize_t syna_sysfs_high_sensitivity_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int retval = 0;
	struct device *p_dev;
	struct kobject *p_kobj;
	struct syna_tcm *tcm;

	p_kobj = g_sysfs_dir->parent;
	p_dev = container_of(p_kobj, struct device, kobj);
	tcm = dev_get_drvdata(p_dev);

	retval = scnprintf(buf, PAGE_SIZE, "%d\n", tcm->high_sensitivity_mode);

	return retval;
}

/*
 * syna_sysfs_high_sensitivity_store()
 *
 * Attribute to set high sensitivity mode.
 *
 * @param
 *    [ in] kobj:  an instance of kobj
 *    [ in] attr:  an instance of kobj attribute structure
 *    [ in] buf:   string buffer input
 *    [ in] count: size of buffer input
 *
 * @return
 *    on success, return count; otherwise, return error code
 */
static ssize_t syna_sysfs_high_sensitivity_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	int retval = count;
	bool input;
	struct device *p_dev;
	struct kobject *p_kobj;
	struct syna_tcm *tcm;

	p_kobj = g_sysfs_dir->parent;
	p_dev = container_of(p_kobj, struct device, kobj);
	tcm = dev_get_drvdata(p_dev);

	if (kstrtobool(buf, &input)) {
		LOGE("Invalid input %s", buf);
		return -EINVAL;
	}

	tcm->high_sensitivity_mode = input;

#if IS_ENABLED(CONFIG_GOOG_TOUCH_INTERFACE)
	tcm->gti->cmd.screen_protector_mode_cmd.setting =
		input ? GTI_SCREEN_PROTECTOR_MODE_ENABLE : GTI_SCREEN_PROTECTOR_MODE_DISABLE;
#endif

	retval = syna_tcm_set_dynamic_config(tcm->tcm_dev,
				DC_HIGH_SENSITIVITY_MODE,
				input,
				RESP_IN_ATTN);

	LOGI("%s high sensitivity mode.\n",
	     tcm->high_sensitivity_mode ? "Enable" : "Disable");

	retval = count;

	return retval;
}

static struct kobj_attribute kobj_attr_high_sensitivity =
	__ATTR(high_sensitivity, 0644, syna_sysfs_high_sensitivity_show,
	       syna_sysfs_high_sensitivity_store);

/*
 * syna_sysfs_fw_grip_show()
 *
 * Attribute to show current grip suppression mode.
 *
 * @param
 *    [ in] kobj:  an instance of kobj
 *    [ in] attr:  an instance of kobj attribute structure
 *    [out] buf:  string buffer shown on console
 *
 * @return
 *    on success, number of characters being output;
 *    otherwise, negative value on error.
 */
static ssize_t syna_sysfs_fw_grip_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int retval = 0;
	struct device *p_dev;
	struct kobject *p_kobj;
	struct syna_tcm *tcm;

	p_kobj = g_sysfs_dir->parent;
	p_dev = container_of(p_kobj, struct device, kobj);
	tcm = dev_get_drvdata(p_dev);

	retval = scnprintf(buf, PAGE_SIZE, "%u\n", tcm->enable_fw_grip);

	return retval;
}

/*
 * syna_sysfs_fw_grip_store()
 *
 * Attribute to set grip suppression mode.
 * 0 - Disable fw grip suppression.
 * 1 - Enable fw grip suppression.
 * 2 - Force disable fw grip suppression.
 * 3 - Force enable fw grip suppression.
 *
 * @param
 *    [ in] kobj:  an instance of kobj
 *    [ in] attr:  an instance of kobj attribute structure
 *    [ in] buf:   string buffer input
 *    [ in] count: size of buffer input
 *
 * @return
 *    on success, return count; otherwise, return error code
 */
static ssize_t syna_sysfs_fw_grip_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	int retval = count;
	u8 input;
	struct device *p_dev;
	struct kobject *p_kobj;
	struct syna_tcm *tcm;

	p_kobj = g_sysfs_dir->parent;
	p_dev = container_of(p_kobj, struct device, kobj);
	tcm = dev_get_drvdata(p_dev);

	if (kstrtou8(buf, 16, &input)) {
		LOGE("Invalid input %s", buf);
		return -EINVAL;
	}

	tcm->enable_fw_grip = input;

#if IS_ENABLED(CONFIG_GOOG_TOUCH_INTERFACE)
	tcm->gti->cmd.grip_cmd.setting = (input & 0x01) ? GTI_GRIP_ENABLE : GTI_GRIP_DISABLE;
	tcm->gti->ignore_grip_update = (input >> 1) & 0x01;
#endif

	retval = syna_tcm_set_dynamic_config(tcm->tcm_dev,
			DC_ENABLE_GRIP_SUPPRESSION,
			(input & 0x01),
			RESP_IN_ATTN);

	LOGI("Set fw grip suppression mode %u.\n", tcm->enable_fw_grip);

	retval = count;

	return retval;
}

static struct kobj_attribute kobj_attr_fw_grip =
	__ATTR(fw_grip, 0644, syna_sysfs_fw_grip_show,
	       syna_sysfs_fw_grip_store);

/*
 * syna_sysfs_fw_palm_show()
 *
 * Attribute to show current palm rejection mode.
 *
 * @param
 *    [ in] kobj:  an instance of kobj
 *    [ in] attr:  an instance of kobj attribute structure
 *    [out] buf:  string buffer shown on console
 *
 * @return
 *    on success, number of characters being output;
 *    otherwise, negative value on error.
 */
static ssize_t syna_sysfs_fw_palm_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int retval = 0;
	struct device *p_dev;
	struct kobject *p_kobj;
	struct syna_tcm *tcm;

	p_kobj = g_sysfs_dir->parent;
	p_dev = container_of(p_kobj, struct device, kobj);
	tcm = dev_get_drvdata(p_dev);

	retval = scnprintf(buf, PAGE_SIZE, "%u\n", tcm->enable_fw_palm);

	return retval;
}

/*
 * syna_sysfs_fw_palm_store()
 *
 * Attribute to set palm rejection mode.
 * 0 - Disable fw palm rejection.
 * 1 - Enable fw palm rejection.
 * 2 - Force disable fw palm rejection.
 * 3 - Force enable fw palm rejection.
 *
 * @param
 *    [ in] kobj:  an instance of kobj
 *    [ in] attr:  an instance of kobj attribute structure
 *    [ in] buf:   string buffer input
 *    [ in] count: size of buffer input
 *
 * @return
 *    on success, return count; otherwise, return error code
 */
static ssize_t syna_sysfs_fw_palm_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	int retval = count;
	u8 input;
	struct device *p_dev;
	struct kobject *p_kobj;
	struct syna_tcm *tcm;

	p_kobj = g_sysfs_dir->parent;
	p_dev = container_of(p_kobj, struct device, kobj);
	tcm = dev_get_drvdata(p_dev);

	if (kstrtou8(buf, 16, &input)) {
		LOGE("Invalid input %s", buf);
		return -EINVAL;
	}

	tcm->enable_fw_palm = input;

#if IS_ENABLED(CONFIG_GOOG_TOUCH_INTERFACE)
	tcm->gti->cmd.palm_cmd.setting = (input & 0x01) ? GTI_PALM_ENABLE : GTI_PALM_DISABLE;
	tcm->gti->ignore_palm_update = (input >> 1) & 0x01;
#endif

	retval = syna_tcm_set_dynamic_config(tcm->tcm_dev,
			DC_ENABLE_PALM_REJECTION,
			(input & 0x01),
			RESP_IN_ATTN);

	LOGI("Set fw palm rejection mode %u.\n", tcm->enable_fw_palm);

	retval = count;

	return retval;
}

static struct kobj_attribute kobj_attr_fw_palm =
	__ATTR(fw_palm, 0644, syna_sysfs_fw_palm_show,
	       syna_sysfs_fw_palm_store);

/*
 * syna_sysfs_compression_threshold_show()
 *
 * Attribute get the heatmap compression threshold.
 *
 * @param
 *    [ in] kobj:  an instance of kobj
 *    [ in] attr:  an instance of kobj attribute structure
 *    [out] buf:  string buffer shown on console
 *
 * @return
 *    on success, number of characters being output;
 *    otherwise, negative value on error.
 */
static ssize_t syna_sysfs_compression_threshold_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int retval = 0;
	u16 output;
	struct device *p_dev;
	struct kobject *p_kobj;
	struct syna_tcm *tcm;

	p_kobj = g_sysfs_dir->parent;
	p_dev = container_of(p_kobj, struct device, kobj);
	tcm = dev_get_drvdata(p_dev);

	retval = syna_tcm_get_dynamic_config(tcm->tcm_dev,
			DC_COMPRESSION_THRESHOLD,
			&output,
			RESP_IN_ATTN);
	if (retval < 0) {
		LOGE("Failed to get compression threshold.\n");
		retval = scnprintf(buf, PAGE_SIZE, "-1\n");
	} else {
		retval = scnprintf(buf, PAGE_SIZE, "%u\n", output);
	}

	return retval;
}

/*
 * syna_sysfs_compression_threshold_store()
 *
 * Attribute set the heatmap compression threshold.
 *
 * @param
 *    [ in] kobj:  an instance of kobj
 *    [ in] attr:  an instance of kobj attribute structure
 *    [ in] buf:   string buffer input
 *    [ in] count: size of buffer input
 *
 * @return
 *    on success, return count; otherwise, return error code
 */
static ssize_t syna_sysfs_compression_threshold_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	int retval = count;
	u8 input;
	struct device *p_dev;
	struct kobject *p_kobj;
	struct syna_tcm *tcm;

	p_kobj = g_sysfs_dir->parent;
	p_dev = container_of(p_kobj, struct device, kobj);
	tcm = dev_get_drvdata(p_dev);

	if (kstrtou8(buf, 10, &input)) {
		LOGE("Invalid input %s", buf);
		return -EINVAL;
	}

	tcm->hw_if->compression_threshold = input;

	syna_tcm_set_dynamic_config(tcm->tcm_dev,
			DC_COMPRESSION_THRESHOLD,
			input,
			RESP_IN_ATTN);

	LOGI("Set the heatmap compression threshold as %u.\n",
	     tcm->hw_if->compression_threshold);

	return retval;
}

static struct kobj_attribute kobj_attr_compression_threshold =
	__ATTR(compression_threshold, 0644, syna_sysfs_compression_threshold_show,
	       syna_sysfs_compression_threshold_store);

/*
 * declaration of sysfs attributes
 */
static struct attribute *attrs[] = {
	&kobj_attr_info.attr,
	&kobj_attr_irq_en.attr,
	&kobj_attr_int2.attr,
	&kobj_attr_reset.attr,
	&kobj_attr_pwr.attr,
	&kobj_attr_scan_mode.attr,
#if IS_ENABLED(CONFIG_GOOG_TOUCH_INTERFACE)
	&kobj_attr_force_active.attr,
#endif
	&kobj_attr_get_raw_data.attr,
	&kobj_attr_high_sensitivity.attr,
	&kobj_attr_fw_grip.attr,
	&kobj_attr_fw_palm.attr,
	&kobj_attr_compression_threshold.attr,
	NULL,
};


static struct attribute_group attr_group = {
	.attrs = attrs,
};

/*
 * syna_sysfs_create_dir()
 *
 * Create a directory and register it with sysfs.
 * Then, create all defined sysfs files.
 *
 * @param
 *    [ in] tcm:  the driver handle
 *    [ in] pdev: an instance of platform device
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
int syna_sysfs_create_dir(struct syna_tcm *tcm,
		struct platform_device *pdev)
{
	int retval = 0;

	g_sysfs_dir = kobject_create_and_add("sysfs",
			&pdev->dev.kobj);
	if (!g_sysfs_dir) {
		LOGE("Fail to create sysfs directory\n");
		return -ENOTDIR;
	}

	tcm->sysfs_dir = g_sysfs_dir;

	retval = sysfs_create_group(g_sysfs_dir, &attr_group);
	if (retval < 0) {
		LOGE("Fail to create sysfs group\n");

		kobject_put(tcm->sysfs_dir);
		return retval;
	}

#ifdef HAS_TESTING_FEATURE
	retval = syna_testing_create_dir(tcm, g_sysfs_dir);
	if (retval < 0) {
		LOGE("Fail to create testing sysfs\n");

		sysfs_remove_group(tcm->sysfs_dir, &attr_group);
		kobject_put(tcm->sysfs_dir);
		return retval;
	}
#endif

	return 0;
}
/*
 * syna_sysfs_remove_dir()
 *
 * Remove the allocate sysfs directory
 *
 * @param
 *    [ in] tcm: the driver handle
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
void syna_sysfs_remove_dir(struct syna_tcm *tcm)
{
	if (!tcm) {
		LOGE("Invalid tcm device handle\n");
		return;
	}

	if (tcm->sysfs_dir) {
#ifdef HAS_TESTING_FEATURE
		syna_testing_remove_dir();
#endif

		sysfs_remove_group(tcm->sysfs_dir, &attr_group);

		kobject_put(tcm->sysfs_dir);
	}

}
