// SPDX-License-Identifier: GPL-2.0
/*
 * Sysfs APIs for Google Pixel devices.
 *
 * Copyright 2022 Google LLC.
 */

#include <linux/device.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/slab.h>

#include "touch_apis.h"

struct touch_apis_data *apis;

static ssize_t fw_ver_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	char fw_ver[0x100];
	int ret = 0;

	if (apis->get_fw_version != NULL) {
		ret = apis->get_fw_version(dev, fw_ver, 0x100);
		if (ret < 0) {
			ret = snprintf(buf, PAGE_SIZE, "error: %d\n", ret);
		} else {
			ret = snprintf(buf, PAGE_SIZE, "result: %s\n", fw_ver);
		}
	} else {
		ret = snprintf(buf, PAGE_SIZE, "error: not support\n");
	}
	return ret;
}

static ssize_t help_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	char fw_ver[0x100];
	int ret = 0;
	char *help_str = "APIs\n"
			 "  fw_ver\n"
			 "  help\n"
			 "  irq_enabled\n"
			 "  list_scan_mode\n"
			 "  ping\n"
			 "  reset\n"
			 "  scan_mode\n"
			 "  sensing_enabled\n"
			 "  wake_lock\n";

	ret = snprintf(buf, PAGE_SIZE, help_str, fw_ver);
	return ret;
}

static ssize_t irq_enabled_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;

	if (apis->get_irq_enabled != NULL && apis->set_irq_enabled != NULL) {
		ret = apis->get_irq_enabled(dev);
		if (ret < 0) {
			ret = snprintf(buf, PAGE_SIZE, "error: %d\n", ret);
		} else {
			ret = snprintf(buf, PAGE_SIZE, "result: %s\n",
				ret ? "enabled" : "disabled");
		}
	} else {
		ret = snprintf(buf, PAGE_SIZE, "error: not support\n");
	}
	return ret;
}

static ssize_t irq_enabled_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	bool enabled = false;

	if (buf == NULL || count <= 0)
		return -EINVAL;

	if (strncmp(buf, "1", 1) == 0) {
		enabled = true;
	} else if (strncmp(buf, "0", 1) == 0) {
		enabled = false;
	} else {
		return -EINVAL;
	}

	if (apis->set_irq_enabled != NULL) {
		apis->set_irq_enabled(dev, enabled);
	}
	return count;
}

static ssize_t list_scan_mode_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;
	enum scan_mode mode = SCAN_MODE_AUTO;
	int len = 0;
	static const char *mode_name[SCAN_MODE_MAX] = {
		"auto mode",
		"normal active mode",
		"normal idle mode",
		"low power active mode",
		"low power idle mode",
	};

	if (apis->is_scan_mode_supported != NULL) {
		len = snprintf(buf, PAGE_SIZE, "result:\n");
		for (mode = SCAN_MODE_AUTO; mode < SCAN_MODE_MAX; mode++) {
			ret = apis->is_scan_mode_supported(dev, mode);
			if (ret) {
				len += snprintf(buf + len, PAGE_SIZE - len,
					"%d: %s\n", mode, mode_name[mode]);
			}
		}
		ret = len;
	} else {
		ret = snprintf(buf, PAGE_SIZE, "error: not support\n");
	}
	return ret;
}

#if IS_ENABLED(CONFIG_TOUCHSCREEN_MOTION_FILTER)
static ssize_t mf_mode_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "result: %d\n", apis->mf_mode);
}

static ssize_t mf_mode_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	enum touch_mf_mode mode = 0;
	int ret = 0;

	if (buf == NULL || count < 0)
		return -EINVAL;

	if (kstrtoint(buf, 10, (int *)&mode)) {
		return -EINVAL;
	}

	if (mode < TOUCH_MF_MODE_UNFILTERED ||
		mode > TOUCH_MF_MODE_AUTO_REPORT) {
		return -EINVAL;
	}

	ret = touch_mf_set_mode(apis->tmf, mode);
	if (ret != 0) {
		return ret;
	}
	apis->mf_mode = mode;
	return count;
}
#endif

static ssize_t ping_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;

	if (apis->ping != NULL) {
		ret = apis->ping(dev);
		ret = snprintf(buf, PAGE_SIZE, "result: %s\n",
			ret == 0 ? "ack" : "non ack");
	} else {
		ret = snprintf(buf, PAGE_SIZE, "error: not support\n");
	}
	return ret;
}

static ssize_t reset_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;

	if (apis->reset_result > RESET_RESULT_NOT_SUPPORT) {
		ret = snprintf(buf, PAGE_SIZE, "result: %s\n",
			apis->reset_result == RESET_RESULT_SUCCESS ? "success"
			: apis->reset_result == RESET_RESULT_FAIL
				? "fail"
				: "haven't reset");
	} else {
		ret = snprintf(buf, PAGE_SIZE, "error: not support\n");
	}
	return ret;
}

static ssize_t reset_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	int ret = 0;

	if (buf == NULL || count <= 0)
		return -EINVAL;

	if (strncmp(buf, "1", 1) == 0) {
		if (apis->hardware_reset != NULL) {
			ret = apis->hardware_reset(dev);
			apis->reset_result = ret == 0 ? RESET_RESULT_SUCCESS
						      : RESET_RESULT_FAIL;
		} else {
			apis->reset_result = RESET_RESULT_NOT_SUPPORT;
		}
	} else if (strncmp(buf, "2", 0) == 0) {
		if (apis->software_reset != NULL) {
			ret = apis->software_reset(dev);
			apis->reset_result = ret == 0 ? RESET_RESULT_SUCCESS
						      : RESET_RESULT_FAIL;
		} else {
			apis->reset_result = RESET_RESULT_NOT_SUPPORT;
		}
	} else {
		apis->reset_result = RESET_RESULT_NOT_SUPPORT;
		return -EINVAL;
	}

	return count;
}

static ssize_t scan_mode_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "result: %d\n", apis->scan_mode);
}

static ssize_t scan_mode_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	enum scan_mode mode = 0;
	int ret = 0;

	if (buf == NULL || count < 0)
		return -EINVAL;

	if (kstrtoint(buf, 10, (int *)&mode)) {
		return -EINVAL;
	}

	if (apis->is_scan_mode_supported == NULL ||
		!apis->is_scan_mode_supported(dev, mode)) {
		return -EINVAL;
	}

	if (apis->set_scan_mode != NULL) {
		ret = apis->set_scan_mode(dev, mode);
		if (ret != 0) {
			return ret;
		}
		apis->scan_mode = mode;
	}
	return count;
}

static ssize_t sensing_enabled_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;

	if (apis->ping != NULL) {
		ret = apis->ping(dev);
		ret = snprintf(buf, PAGE_SIZE, "result: %s\n",
			ret == 0 ? "enabled" : "disabled");
	} else {
		ret = snprintf(buf, PAGE_SIZE, "error: not support\n");
	}
	return ret;
}

static ssize_t sensing_enabled_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	bool enabled = false;

	if (buf == NULL || count <= 0)
		return -EINVAL;

	if (strncmp(buf, "1", 1) == 0) {
		enabled = true;
	} else if (strncmp(buf, "0", 1) == 0) {
		enabled = false;
	} else {
		return -EINVAL;
	}

	if (apis->set_sensing_enabled != NULL) {
		apis->set_sensing_enabled(dev, enabled);
	}
	return count;
}

#if IS_ENABLED(CONFIG_GOOG_TOUCH_INTERFACE) && IS_ENABLED(CONFIG_GTI_PM)
static ssize_t wake_lock_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;
	bool locked = false;

	if (apis->get_wake_lock_state != NULL) {
		locked = apis->get_wake_lock_state(
			dev, GTI_PM_WAKELOCK_TYPE_FORCE_ACTIVE);
		ret = snprintf(buf, PAGE_SIZE, "result: %s\n",
			locked ? "locked" : "unlocked");
	} else {
		ret = snprintf(buf, PAGE_SIZE, "error: not support\n");
	}
	return ret;
}

static ssize_t wake_lock_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	bool locked = false;
	int ret = 0;

	if (buf == NULL || count <= 0)
		return -EINVAL;

	if (strncmp(buf, "1", 1) == 0) {
		locked = true;
	} else if (strncmp(buf, "0", 1) == 0) {
		locked = false;
	} else {
		return -EINVAL;
	}

	if (apis->set_wake_lock_state != NULL) {
		ret = apis->set_wake_lock_state(
			dev, GTI_PM_WAKELOCK_TYPE_FORCE_ACTIVE, locked);
		if (ret < 0) {
			return ret;
		}
	}
	return count;
}
#endif

static DEVICE_ATTR_RO(fw_ver);
static DEVICE_ATTR_RO(help);
static DEVICE_ATTR_RW(irq_enabled);
static DEVICE_ATTR_RO(list_scan_mode);
#if IS_ENABLED(CONFIG_TOUCHSCREEN_MOTION_FILTER)
static DEVICE_ATTR_RW(mf_mode);
#endif
static DEVICE_ATTR_RO(ping);
static DEVICE_ATTR_RW(reset);
static DEVICE_ATTR_RW(scan_mode);
static DEVICE_ATTR_RW(sensing_enabled);
#if IS_ENABLED(CONFIG_GOOG_TOUCH_INTERFACE) && IS_ENABLED(CONFIG_GTI_PM)
static DEVICE_ATTR_RW(wake_lock);
#endif

static struct attribute *sysfs_attrs[] = {
	&dev_attr_fw_ver.attr,
	&dev_attr_help.attr,
	&dev_attr_irq_enabled.attr,
	&dev_attr_list_scan_mode.attr,
#if IS_ENABLED(CONFIG_TOUCHSCREEN_MOTION_FILTER)
	&dev_attr_mf_mode.attr,
#endif
	&dev_attr_ping.attr,
	&dev_attr_reset.attr,
	&dev_attr_scan_mode.attr,
	&dev_attr_sensing_enabled.attr,
#if IS_ENABLED(CONFIG_GOOG_TOUCH_INTERFACE) && IS_ENABLED(CONFIG_GTI_PM)
	&dev_attr_wake_lock.attr,
#endif
	NULL,
};

static const struct attribute_group sysfs_group = {
	.attrs = sysfs_attrs,
};

int touch_apis_init(struct device *dev, struct touch_apis_data *data)
{
	int ret = 0;

	apis = data;
	apis->scan_mode = SCAN_MODE_AUTO;
	apis->reset_result = RESET_RESULT_NOT_READY;

	ret = sysfs_create_group(&dev->kobj, &sysfs_group);
	if (ret) {
		dev_err(dev, "failed create core sysfs group");
		return ret;
	}
	return ret;
}

void touch_apis_deinit(struct device *dev)
{
	sysfs_remove_group(&dev->kobj, &sysfs_group);
	if (apis != NULL) {
		apis = NULL;
	}
}
