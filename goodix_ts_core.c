/*
 * Goodix Touchscreen Driver
 * Copyright (C) 2020 - 2021 Goodix, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be a reference
 * to you, when you are integrating the GOODiX's CTP IC into your system,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 */
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include <drm/drm_panel.h>

#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 38)
#include <linux/input/mt.h>
#define INPUT_TYPE_B_PROTOCOL
#endif

#include "goodix_ts_core.h"
/* goodix fb test */
// #include "../../../video/fbdev/core/fb_firefly.h"

#define GOODIX_DEFAULT_CFG_NAME "goodix_cfg_group.cfg"

struct goodix_device_manager goodix_devices;

static const struct dev_pm_ops dev_pm_ops; /* [GOOG] */

/*
 * [GOOG]
 * Wait device to complete the init stage2 by order.
 */
static void goodix_wait_for_init_stage2_start(struct goodix_ts_core *current_cd)
{
	struct goodix_device_resource *res, *next;
	struct goodix_ts_core *cd;

	if (!goodix_devices.initialized)
		return;

	if (list_empty(&goodix_devices.list))
		return;

	list_for_each_entry_safe(res, next, &goodix_devices.list, list) {
		cd = &res->core_data;
		if (res->id >= current_cd->pdev->id ||
			cd->init_stage != CORE_INIT_STAGE1) {
			continue;
		}

		/* Wait device to complete the init stage1 */
		if (wait_for_completion_timeout(&cd->init_stage2_complete,
				msecs_to_jiffies(2 * MSEC_PER_SEC)) == 0)
			ts_info("device#%d wait device#%d timeout to complete init state2!",
				current_cd->pdev->id, res->id);
		else
			ts_info("device#%d complete init stage2", res->id);
	}
}


static void goodix_device_manager_init(void)
{
	if (goodix_devices.initialized)
		return;
	goodix_devices.initialized = true;
	INIT_LIST_HEAD(&goodix_devices.list);
	mutex_init(&goodix_devices.mutex);
}

static void goodix_device_manager_exit(void)
{
	struct goodix_device_resource *res, *next;

	if (!list_empty(&goodix_devices.list)) {
		list_for_each_entry_safe(res, next, &goodix_devices.list, list) {
			platform_device_unregister(&res->pdev);
			kfree(res);
		}
	}
}

int goodix_device_register(struct goodix_device_resource *device)
{
	u32 dev_id; /* [GOOG] */

	if (!device)
		return -ENXIO;

	mutex_lock(&goodix_devices.mutex);
	list_add(&device->list, &goodix_devices.list);
	dev_id = goodix_devices.nums++;
	if (device->bus.dev) {
		of_property_read_u32(device->bus.dev->of_node,
			"goodix,dev-id", &dev_id); /* [GOOG] */
	}
	device->id = dev_id;
	sprintf(device->name, "%s.%d", GOODIX_CORE_DRIVER_NAME, device->id);
	mutex_unlock(&goodix_devices.mutex);
	init_completion(&device->core_data.init_stage2_complete); /* [GOOG] */
	ts_info("register device %s", device->name);

	return 0;
}

static int goodix_send_ic_config(struct goodix_ts_core *cd, int type);

/* show driver information */
static ssize_t driver_info_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "DriverVersion:%s\n",
			GOODIX_DRIVER_VERSION);
}

/* show chip infoamtion */
static ssize_t chip_info_show(struct device  *dev,
		struct device_attribute *attr, char *buf)
{
	struct goodix_ts_core *cd = dev_get_drvdata(dev);
	struct goodix_ts_hw_ops *hw_ops = cd->hw_ops;
	struct goodix_fw_version chip_ver;
	struct goodix_ic_info ic_info;
	u8 temp_pid[8] = { 0 };
	int ret;
	int cnt = -EINVAL;

	if (hw_ops->read_version) {
		ret = hw_ops->read_version(cd, &chip_ver);
		if (!ret) {
			memcpy(temp_pid, chip_ver.rom_pid,
				sizeof(chip_ver.rom_pid));
			cnt = snprintf(&buf[0], PAGE_SIZE,
				"rom_pid:%s\nrom_vid:%02x%02x%02x\n", temp_pid,
				chip_ver.rom_vid[0], chip_ver.rom_vid[1],
				chip_ver.rom_vid[2]);
			cnt += snprintf(&buf[cnt], PAGE_SIZE,
				"patch_pid:%s\npatch_vid:%02x%02x%02x%02x\n",
				chip_ver.patch_pid, chip_ver.patch_vid[0],
				chip_ver.patch_vid[1], chip_ver.patch_vid[2],
				chip_ver.patch_vid[3]);
			cnt += snprintf(&buf[cnt], PAGE_SIZE, "sensorid:%d\n",
				chip_ver.sensor_id);
		}
	}

	if (hw_ops->get_ic_info) {
		ret = hw_ops->get_ic_info(cd, &ic_info);
		if (!ret) {
			cnt += snprintf(&buf[cnt], PAGE_SIZE, "config_id:%x\n",
				ic_info.version.config_id);
			cnt += snprintf(&buf[cnt], PAGE_SIZE,
				"config_version:%x\n",
				ic_info.version.config_version);
		}
	}

	return cnt;
}

/* reset chip */
static ssize_t goodix_ts_reset_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf,
				     size_t count)
{
	struct goodix_ts_core *core_data = dev_get_drvdata(dev);
	struct goodix_ts_hw_ops *hw_ops = core_data->hw_ops;

	if (!buf || count <= 0)
		return -EINVAL;
	if (buf[0] != '0')
		hw_ops->reset(core_data, goodix_get_normal_reset_delay(core_data));
	return count;
}

/* read config */
static ssize_t read_cfg_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct goodix_ts_core *core_data = dev_get_drvdata(dev);
	struct goodix_ts_hw_ops *hw_ops = core_data->hw_ops;
	int ret;
	int i;
	int offset;
	char *cfg_buf = NULL;

	cfg_buf = kzalloc(PAGE_SIZE, GFP_KERNEL);
	if (!cfg_buf)
		return -ENOMEM;

	if (hw_ops->read_config)
		ret = hw_ops->read_config(core_data, cfg_buf, PAGE_SIZE);
	else
		ret = -EINVAL;

	if (ret > 0) {
		offset = 0;
		for (i = 0; i < 200; i++) { // only print 200 bytes
			offset += snprintf(&buf[offset], PAGE_SIZE - offset,
				"%02x,", cfg_buf[i]);
			if ((i + 1) % 20 == 0)
				buf[offset++] = '\n';
		}
	}

	kfree(cfg_buf);
	if (ret <= 0)
		return ret;

	return offset;
}

static u8 ascii2hex(u8 a)
{
	s8 value = 0;

	if (a >= '0' && a <= '9')
		value = a - '0';
	else if (a >= 'A' && a <= 'F')
		value = a - 'A' + 0x0A;
	else if (a >= 'a' && a <= 'f')
		value = a - 'a' + 0x0A;
	else
		value = 0xff;

	return value;
}

static int goodix_ts_convert_0x_data(
	const u8 *buf, int buf_size, u8 *out_buf, int *out_buf_len)
{
	int i, m_size = 0;
	int temp_index = 0;
	u8 high, low;

	for (i = 0; i < buf_size; i++) {
		if (buf[i] == 'x' || buf[i] == 'X')
			m_size++;
	}

	if (m_size <= 1) {
		ts_err("cfg file ERROR, valid data count:%d", m_size);
		return -EINVAL;
	}
	*out_buf_len = m_size;

	for (i = 0; i < buf_size; i++) {
		if (buf[i] != 'x' && buf[i] != 'X')
			continue;

		if (temp_index >= m_size) {
			ts_err("exchange cfg data error, overflow, temp_index:%d,m_size:%d",
				temp_index, m_size);
			return -EINVAL;
		}
		high = ascii2hex(buf[i + 1]);
		low = ascii2hex(buf[i + 2]);
		if (high == 0xff || low == 0xff) {
			ts_err("failed convert: 0x%x, 0x%x", buf[i + 1],
				buf[i + 2]);
			return -EINVAL;
		}
		out_buf[temp_index++] = (high << 4) + low;
	}
	return 0;
}

/* send config */
static ssize_t goodix_ts_send_cfg_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct goodix_ts_core *core_data = dev_get_drvdata(dev);
	struct goodix_ts_hw_ops *hw_ops = core_data->hw_ops;
	struct goodix_ic_config *config = NULL;
	const struct firmware *cfg_img = NULL;
	int ret;

	if (buf[0] != '1')
		return -EINVAL;

	hw_ops->irq_enable(core_data, false);

	ret = request_firmware(&cfg_img, GOODIX_DEFAULT_CFG_NAME, dev);
	if (ret < 0) {
		ts_err("cfg file [%s] not available,errno:%d",
			GOODIX_DEFAULT_CFG_NAME, ret);
		goto exit;
	} else {
		ts_info("cfg file [%s] is ready", GOODIX_DEFAULT_CFG_NAME);
	}

	config = kzalloc(sizeof(*config), GFP_KERNEL);
	if (!config)
		goto exit;

	if (goodix_ts_convert_0x_data(
		    cfg_img->data, cfg_img->size, config->data, &config->len)) {
		ts_err("convert config data FAILED");
		goto exit;
	}

	if (hw_ops->send_config) {
		ret = hw_ops->send_config(core_data, config->data, config->len);
		if (ret < 0)
			ts_err("send config failed");
	}

exit:
	hw_ops->irq_enable(core_data, true);
	kfree(config);
	if (cfg_img)
		release_firmware(cfg_img);

	return count;
}

/* reg read/write */
static u32 rw_addr;
static u32 rw_len;
static u8 rw_flag;
static u8 store_buf[32];
static u8 show_buf[PAGE_SIZE];
static ssize_t goodix_ts_reg_rw_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct goodix_ts_core *core_data = dev_get_drvdata(dev);
	struct goodix_ts_hw_ops *hw_ops = core_data->hw_ops;
	int ret;

	if (!rw_addr || !rw_len) {
		ts_err("address(0x%x) and length(%d) can't be null", rw_addr,
			rw_len);
		return -EINVAL;
	}

	if (rw_flag != 1) {
		ts_err("invalid rw flag %d, only support [1/2]", rw_flag);
		return -EINVAL;
	}

	ret = hw_ops->read(core_data, rw_addr, show_buf, rw_len);
	if (ret < 0) {
		ts_err("failed read addr(%x) length(%d)", rw_addr, rw_len);
		return snprintf(buf, PAGE_SIZE,
			"failed read addr(%x), len(%d)\n", rw_addr, rw_len);
	}

	return snprintf(buf, PAGE_SIZE, "0x%x,%d {%*ph}\n", rw_addr, rw_len,
		rw_len, show_buf);
}

static ssize_t goodix_ts_reg_rw_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct goodix_ts_core *core_data = dev_get_drvdata(dev);
	struct goodix_ts_hw_ops *hw_ops = core_data->hw_ops;
	char *pos = NULL;
	char *token = NULL;
	long result = 0;
	int ret;
	int i;

	if (!buf || !count) {
		ts_err("invalid parame");
		goto err_out;
	}

	if (buf[0] == 'r') {
		rw_flag = 1;
	} else if (buf[0] == 'w') {
		rw_flag = 2;
	} else {
		ts_err("string must start with 'r/w'");
		goto err_out;
	}

	/* get addr */
	pos = (char *)buf;
	pos += 2;
	token = strsep(&pos, ":");
	if (!token) {
		ts_err("invalid address info");
		goto err_out;
	} else {
		if (kstrtol(token, 16, &result)) {
			ts_err("failed get addr info");
			goto err_out;
		}
		rw_addr = (u32)result;
		ts_info("rw addr is 0x%x", rw_addr);
	}

	/* get length */
	token = strsep(&pos, ":");
	if (!token) {
		ts_err("invalid length info");
		goto err_out;
	} else {
		if (kstrtol(token, 0, &result)) {
			ts_err("failed get length info");
			goto err_out;
		}
		rw_len = (u32)result;
		ts_info("rw length info is %d", rw_len);
		if (rw_len > sizeof(store_buf)) {
			ts_err("data len > %lu", sizeof(store_buf));
			goto err_out;
		}
	}

	if (rw_flag == 1)
		return count;

	for (i = 0; i < rw_len; i++) {
		token = strsep(&pos, ":");
		if (!token) {
			ts_err("invalid data info");
			goto err_out;
		} else {
			if (kstrtol(token, 16, &result)) {
				ts_err("failed get data[%d] info", i);
				goto err_out;
			}
			store_buf[i] = (u8)result;
			ts_info("get data[%d]=0x%x", i, store_buf[i]);
		}
	}
	ret = hw_ops->write(core_data, rw_addr, store_buf, rw_len);
	if (ret < 0) {
		ts_err("failed write addr(%x) data %*ph", rw_addr, rw_len,
			store_buf);
		goto err_out;
	}

	ts_info("%s write to addr (%x) with data %*ph", "success", rw_addr,
		rw_len, store_buf);

	return count;
err_out:
	snprintf(show_buf, PAGE_SIZE, "%s\n",
		"invalid params, format{r/w:4100:length:[41:21:31]}");
	return -EINVAL;
}

/* show irq information */
static ssize_t goodix_ts_irq_info_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct goodix_ts_core *core_data = dev_get_drvdata(dev);
	struct irq_desc *desc;
	size_t offset = 0;
	int r;

	r = snprintf(&buf[offset], PAGE_SIZE, "irq:%u\n", core_data->irq);
	if (r < 0)
		return -EINVAL;

	offset += r;
	r = snprintf(&buf[offset], PAGE_SIZE - offset, "state:%s\n",
		atomic_read(&core_data->irq_enabled) ? "enabled" : "disabled");
	if (r < 0)
		return -EINVAL;

	desc = irq_to_desc(core_data->irq);
	offset += r;
	r = snprintf(&buf[offset], PAGE_SIZE - offset, "disable-depth:%d\n",
		desc->depth);
	if (r < 0)
		return -EINVAL;

	offset += r;
	r = snprintf(&buf[offset], PAGE_SIZE - offset, "trigger-count:%zu\n",
		core_data->irq_trig_cnt);
	if (r < 0)
		return -EINVAL;

	offset += r;
	r = snprintf(&buf[offset], PAGE_SIZE - offset,
		"echo 0/1 > irq_info to disable/enable irq\n");
	if (r < 0)
		return -EINVAL;

	offset += r;
	return offset;
}

/* enable/disable irq */
static ssize_t goodix_ts_irq_info_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct goodix_ts_core *core_data = dev_get_drvdata(dev);
	struct goodix_ts_hw_ops *hw_ops = core_data->hw_ops;

	if (!buf || count <= 0)
		return -EINVAL;

	if (buf[0] != '0')
		hw_ops->irq_enable(core_data, true);
	else
		hw_ops->irq_enable(core_data, false);
	return count;
}

/* show esd status */
static ssize_t goodix_ts_esd_info_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct goodix_ts_core *core_data = dev_get_drvdata(dev);
	struct goodix_ts_esd *ts_esd = &core_data->ts_esd;
	int r = 0;

	r = snprintf(buf, PAGE_SIZE, "state:%s\n",
		atomic_read(&ts_esd->esd_on) ? "enabled" : "disabled");

	return r;
}

/* enable/disable esd */
static ssize_t goodix_ts_esd_info_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct goodix_ts_core *cd = dev_get_drvdata(dev);

	if (!buf || count <= 0)
		return -EINVAL;

	if (buf[0] != '0')
		goodix_ts_esd_on(cd);
	else
		goodix_ts_esd_off(cd);
	return count;
}

/* debug level show */
static ssize_t goodix_ts_debug_log_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	int r = 0;

	r = snprintf(buf, PAGE_SIZE, "state:%s\n",
		debug_log_flag ? "enabled" : "disabled");

	return r;
}

/* debug level store */
static ssize_t goodix_ts_debug_log_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	if (!buf || count <= 0)
		return -EINVAL;

	if (buf[0] != '0')
		debug_log_flag = true;
	else
		debug_log_flag = false;
	return count;
}

static int goodix_refresh_pen_pair(struct goodix_ts_core *cd)
{
	struct goodix_ble_data *ble_data = &cd->ble_data;
	u8 checksum = 0;
	int i;

	mutex_lock(&ble_data->lock);
	ble_data->cmd.cmd = 0xC5;
	ble_data->cmd.len = 5;
	ble_data->cmd.data[0] = 1;
	ble_data->cmd.data[1] = ble_data->tx1_freq_index;
	ble_data->cmd.data[2] = ble_data->tx2_freq_index;
	ble_data->cmd.data[3] = 0;
	ble_data->cmd.data[4] = 0;
	for (i = 0; i < 7; i++)
		checksum += ble_data->cmd.buf[i];
	ble_data->cmd.data[5] = checksum;
	mutex_unlock(&ble_data->lock);
	sysfs_notify(&cd->pdev->dev.kobj, NULL, "pen_get");
	ts_info("pen pair event");
	return 0;
}

/* debug level show */
static ssize_t goodix_ts_pen_get_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct goodix_ts_core *core_data = dev_get_drvdata(dev);
	struct goodix_ble_data *ble_data = &core_data->ble_data;

	mutex_lock(&ble_data->lock);
	memcpy(buf, ble_data->cmd.buf, sizeof(ble_data->cmd));
	mutex_unlock(&ble_data->lock);
	return sizeof(ble_data->cmd);
}

/* debug level store */
static ssize_t goodix_ts_pen_debug_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct goodix_ts_core *core_data = dev_get_drvdata(dev);

	sysfs_notify(&core_data->pdev->dev.kobj, NULL, "pen_get");
	return count;
}

static ssize_t goodix_ts_pen_set_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct goodix_ts_core *core_data = dev_get_drvdata(dev);
	struct goodix_ble_data *ble_data = &core_data->ble_data;
	struct goodix_ble_cmd temp_cmd;
	struct goodix_ts_cmd cmd;

	if (count > sizeof(temp_cmd)) {
		ts_err("data count to long");
		return -EINVAL;
	}
	ts_debug("get ble cmd:%*ph", (int)count, buf);

	mutex_lock(&ble_data->lock);
	memcpy(temp_cmd.buf, buf, count);
	switch (temp_cmd.cmd) {
	case 0x4B:
		ble_data->pressure = (temp_cmd.data[1] << 8) | temp_cmd.data[0];
		ble_data->hogp_ready = 1;
		break;
	case 0xC4:
		cmd.cmd = 0xAA;
		cmd.len = 6;
		cmd.data[0] = temp_cmd.data[1];
		cmd.data[1] = temp_cmd.data[2];
		core_data->hw_ops->send_cmd(core_data, &cmd);
		cmd.cmd = 0xBB;
		cmd.len = 5;
		cmd.data[0] = temp_cmd.data[4];
		core_data->hw_ops->send_cmd(core_data, &cmd);
		break;
	}
	mutex_unlock(&ble_data->lock);

	return count;
}

static DEVICE_ATTR(driver_info, 0440, driver_info_show, NULL);
static DEVICE_ATTR(chip_info, 0440, chip_info_show, NULL);
static DEVICE_ATTR(hw_reset, 0220, NULL, goodix_ts_reset_store); /* [GOOG] */
static DEVICE_ATTR(send_cfg, 0220, NULL, goodix_ts_send_cfg_store);
static DEVICE_ATTR(read_cfg, 0440, read_cfg_show, NULL);
static DEVICE_ATTR(reg_rw, 0664, goodix_ts_reg_rw_show, goodix_ts_reg_rw_store);
static DEVICE_ATTR(
	irq_info, 0664, goodix_ts_irq_info_show, goodix_ts_irq_info_store);
static DEVICE_ATTR(
	esd_info, 0664, goodix_ts_esd_info_show, goodix_ts_esd_info_store);
static DEVICE_ATTR(
	debug_log, 0664, goodix_ts_debug_log_show, goodix_ts_debug_log_store);
static DEVICE_ATTR(pen_get, 0440, goodix_ts_pen_get_show, NULL);
static DEVICE_ATTR(pen_debug, 0220, NULL, goodix_ts_pen_debug_store);
static DEVICE_ATTR(pen_set, 0220, NULL, goodix_ts_pen_set_store);

static struct attribute *sysfs_attrs[] = {
	&dev_attr_driver_info.attr,
	&dev_attr_chip_info.attr,
	&dev_attr_hw_reset.attr,/* [GOOG] use touch_apis.c to create `reset` sysfs instead */
	&dev_attr_send_cfg.attr,
	&dev_attr_read_cfg.attr,
	&dev_attr_reg_rw.attr,
	&dev_attr_irq_info.attr,
	&dev_attr_esd_info.attr,
	&dev_attr_debug_log.attr,
	&dev_attr_pen_get.attr,
	&dev_attr_pen_debug.attr,
	&dev_attr_pen_set.attr,
	NULL,
};

static const struct attribute_group sysfs_group = {
	.attrs = sysfs_attrs,
};

static int goodix_ts_sysfs_init(struct goodix_ts_core *core_data)
{
	int ret;

	ret = sysfs_create_group(&core_data->pdev->dev.kobj, &sysfs_group);
	if (ret) {
		ts_err("failed create core sysfs group");
		return ret;
	}

	return ret;
}

static void goodix_ts_sysfs_exit(struct goodix_ts_core *core_data)
{
	sysfs_remove_group(&core_data->pdev->dev.kobj, &sysfs_group);
}

/* [GOOG] */
#if IS_ENABLED(CONFIG_TOUCHSCREEN_MOTION_FILTER)
static int set_continuously_report_enabled(struct device *dev, bool enabled)
{
	struct goodix_ts_core *cd = dev_get_drvdata(dev);
	return cd->hw_ops->set_continuously_report_enabled(cd, enabled);
}
#endif

static int get_fw_version(struct device *dev, char *buf, size_t buf_size)
{
	struct goodix_ts_core *cd = dev_get_drvdata(dev);
	int ret = 0;

	ret = cd->hw_ops->read_version(cd, &cd->fw_version);
	if (ret) {
		return ret;
	}

	snprintf(buf, buf_size, "%02x.%02x.%02x.%02x",
		cd->fw_version.patch_vid[0], cd->fw_version.patch_vid[1],
		cd->fw_version.patch_vid[2], cd->fw_version.patch_vid[3]);
	return ret;
}

static int get_irq_enabled(struct device *dev)
{
	struct goodix_ts_core *cd = dev_get_drvdata(dev);
	return atomic_read(&cd->irq_enabled);
}

static int set_irq_enabled(struct device *dev, bool enabled)
{
	struct goodix_ts_core *cd = dev_get_drvdata(dev);
	return cd->hw_ops->irq_enable(cd, enabled);
}

static bool is_scan_mode_supported(struct device *dev, enum scan_mode mode)
{
	return mode == SCAN_MODE_AUTO || mode == SCAN_MODE_NORMAL_ACTIVE ||
	       mode == SCAN_MODE_NORMAL_IDLE;
}

static int ping(struct device *dev)
{
	struct goodix_ts_core *cd = dev_get_drvdata(dev);
	return cd->hw_ops->ping(cd);
}

static int hardware_reset(struct device *dev)
{
	struct goodix_ts_core *cd = dev_get_drvdata(dev);
	return cd->hw_ops->reset(cd, goodix_get_normal_reset_delay(cd));
}

static int set_scan_mode(struct device *dev, enum scan_mode mode)
{
	struct goodix_ts_core *cd = dev_get_drvdata(dev);
	return cd->hw_ops->set_scan_mode(cd, (enum raw_scan_mode)mode);
}

static int set_sensing_enabled(struct device *dev, bool enabled)
{
	struct goodix_ts_core *cd = dev_get_drvdata(dev);
	if (enabled) {
		cd->hw_ops->resume(cd);
		cd->hw_ops->irq_enable(cd, true);
		goodix_ts_esd_on(cd);
		ts_info("set sense ON");
	} else {
		goodix_ts_esd_off(cd);
		cd->hw_ops->irq_enable(cd, false);
		cd->hw_ops->suspend(cd);
		ts_info("set sense OFF");
	}
	return 0;
}

#if IS_ENABLED(CONFIG_GOOG_TOUCH_INTERFACE)
#if IS_ENABLED(CONFIG_GTI_PM)
static bool get_wake_lock_state(struct device *dev, enum gti_pm_wakelock_type type)
{
	struct goodix_ts_core *cd = dev_get_drvdata(dev);
	return goog_pm_wake_check_locked(cd->gti, type);
}

static int set_wake_lock_state(
	struct device *dev, enum gti_pm_wakelock_type type, bool locked)
{
	struct goodix_ts_core *cd = dev_get_drvdata(dev);
	int ret = 0;

	if (locked)
		ret = goog_pm_wake_lock(cd->gti, type, false);
	else
		ret = goog_pm_wake_unlock(cd->gti, type);
	return ret;
}
#endif

static int gti_default_handler(void *private_data, enum gti_cmd_type cmd_type,
	struct gti_union_cmd_data *cmd)
{
	int err = 0;

	switch (cmd_type) {
	case GTI_CMD_NOTIFY_DISPLAY_STATE:
	case GTI_CMD_NOTIFY_DISPLAY_VREFRESH:
		err = -EOPNOTSUPP;
		break;
	default:
		err = -ESRCH;
		break;
	}
	return err;
}

static int get_mutual_sensor_data(
	void *private_data, struct gti_sensor_data_cmd *cmd)
{
	struct goodix_ts_core *cd = private_data;
	int tx = cd->ic_info.parm.drv_num;
	int rx = cd->ic_info.parm.sen_num;
	int ret = 0;

	if (cmd->type == GTI_SENSOR_DATA_TYPE_MS) {
		cmd->buffer = (u8 *)cd->mutual_data;
		cmd->size = tx * rx * sizeof(uint16_t);
	} else {
		/* disable esd */
		goodix_ts_esd_off(cd);

		ret = -EINVAL;
		if (cmd->type == GTI_SENSOR_DATA_TYPE_MS_DIFF) {
			ret = cd->hw_ops->get_mutual_data(cd, FRAME_DATA_TYPE_DIFF);
		} else if (cmd->type == GTI_SENSOR_DATA_TYPE_MS_RAW) {
			ret = cd->hw_ops->get_mutual_data(cd, FRAME_DATA_TYPE_RAW);
		} else if (cmd->type == GTI_SENSOR_DATA_TYPE_MS_BASELINE) {
			ret = cd->hw_ops->get_mutual_data(cd, FRAME_DATA_TYPE_BASE);
		}

		if (ret == 0) {
			cmd->buffer = (u8 *)cd->mutual_data_manual;
			cmd->size = tx * rx * sizeof(uint16_t);
		}
		/* enable esd */
		goodix_ts_esd_on(cd);
	}
	return ret;
}

static int get_self_sensor_data(
	void *private_data, struct gti_sensor_data_cmd *cmd)
{
	struct goodix_ts_core *cd = private_data;
	int tx = cd->ic_info.parm.drv_num;
	int rx = cd->ic_info.parm.sen_num;
	int ret = 0;

	if (cmd->type == GTI_SENSOR_DATA_TYPE_SS) {
		cmd->buffer = (u8 *)cd->self_sensing_data;
		cmd->size = (tx + rx) * sizeof(uint16_t);
	} else {
		/* disable esd */
		goodix_ts_esd_off(cd);

		ret = -EINVAL;
		if (cmd->type == GTI_SENSOR_DATA_TYPE_SS_DIFF) {
			ret = cd->hw_ops->get_self_sensing_data(cd, FRAME_DATA_TYPE_DIFF);
		} else if (cmd->type == GTI_SENSOR_DATA_TYPE_SS_RAW) {
			ret = cd->hw_ops->get_self_sensing_data(cd, FRAME_DATA_TYPE_RAW);
		} else if (cmd->type == GTI_SENSOR_DATA_TYPE_SS_BASELINE) {
			ret = cd->hw_ops->get_self_sensing_data(cd, FRAME_DATA_TYPE_BASE);
		}

		if (ret == 0) {
			cmd->buffer = (u8 *)cd->self_sensing_data_manual;
			cmd->size = (tx + rx) * sizeof(uint16_t);
		}

		/* enable esd */
		goodix_ts_esd_on(cd);
	}
	return ret;
}

static int set_continuous_report(
	void *private_data, struct gti_continuous_report_cmd *cmd)
{
	struct goodix_ts_core *cd = private_data;
	return cd->hw_ops->set_continuously_report_enabled(cd,
		cmd->setting == GTI_CONTINUOUS_REPORT_ENABLE);
}

static int set_grip_enabled(struct goodix_ts_core *cd, bool enabled)
{
	return cd->hw_ops->set_grip_enabled(cd, enabled);
}

static int set_grip_mode(void *private_data, struct gti_grip_cmd *cmd)
{
	struct goodix_ts_core *cd = private_data;
	return set_grip_enabled(cd, cmd->setting == GTI_GRIP_ENABLE);
}

static int get_grip_mode(void *private_data, struct gti_grip_cmd *cmd)
{
	struct goodix_ts_core *cd = private_data;
	bool enabled = false;

	cd->hw_ops->get_grip_enabled(cd, &enabled);
	cmd->setting = enabled ? GTI_GRIP_ENABLE : GTI_GRIP_DISABLE;
	return 0;
}

static int set_palm_enabled(struct goodix_ts_core *cd, bool enabled)
{
	return cd->hw_ops->set_palm_enabled(cd, enabled);
}

static int set_palm_mode(void *private_data, struct gti_palm_cmd *cmd)
{
	struct goodix_ts_core *cd = private_data;
	return set_palm_enabled(cd, cmd->setting == GTI_PALM_ENABLE);
}

static int get_palm_mode(void *private_data, struct gti_palm_cmd *cmd)
{
	struct goodix_ts_core *cd = private_data;
	bool enabled = false;

	cd->hw_ops->get_palm_enabled(cd, &enabled);
	cmd->setting = enabled ? GTI_PALM_ENABLE : GTI_PALM_DISABLE;
	return 0;
}

static int goodix_set_screen_protector_mode_enabled(
	struct goodix_ts_core *cd, bool enabled)
{
	int ret = 0;
	ret = cd->hw_ops->set_screen_protector_mode_enabled(cd, enabled);
	if (ret == 0)
		cd->screen_protector_mode_enabled = enabled;
	return ret;
}

static int set_screen_protector_mode(
	void *private_data, struct gti_screen_protector_mode_cmd *cmd)
{
	struct goodix_ts_core *cd = private_data;
	return goodix_set_screen_protector_mode_enabled(
		cd, cmd->setting == GTI_SCREEN_PROTECTOR_MODE_ENABLE);
}

static int get_screen_protector_mode(
	void *private_data, struct gti_screen_protector_mode_cmd *cmd)
{
	struct goodix_ts_core *cd = private_data;
	bool enabled = false;

	cd->hw_ops->get_screen_protector_mode_enabled(cd, &enabled);
	cmd->setting = enabled ? GTI_SCREEN_PROTECTOR_MODE_ENABLE :
		GTI_SCREEN_PROTECTOR_MODE_DISABLE;
	return 0;
}

static int set_coord_filter_enabled(void *private_data,
	struct gti_coord_filter_cmd *cmd)
{
	struct goodix_ts_core *cd = private_data;
	return cd->hw_ops->set_coord_filter_enabled(cd,
		cmd->setting == GTI_COORD_FILTER_ENABLE);
}

static int get_coord_filter_enabled(void *private_data,
	struct gti_coord_filter_cmd *cmd)
{
	struct goodix_ts_core *cd = private_data;
	bool enabled = false;

	cd->hw_ops->get_coord_filter_enabled(cd, &enabled);
	cmd->setting = enabled ? GTI_COORD_FILTER_ENABLE : GTI_COORD_FILTER_DISABLE;
	return 0;
}

static int set_heatmap_enabled(
	void *private_data, struct gti_heatmap_cmd *cmd)
{
	struct goodix_ts_core *cd = private_data;
	return cd->hw_ops->set_heatmap_enabled(cd, cmd->setting == GTI_HEATMAP_ENABLE);
}

static int gti_get_fw_version(void *private_data,
	struct gti_fw_version_cmd *cmd)
{
	struct goodix_ts_core *cd = private_data;
	int ret = 0;

	ret = cd->hw_ops->read_version(cd, &cd->fw_version);
	if (ret) {
		return ret;
	}

	snprintf(cmd->buffer, sizeof(cmd->buffer), "%02x.%02x.%02x.%02x",
		cd->fw_version.patch_vid[0], cd->fw_version.patch_vid[1],
		cd->fw_version.patch_vid[2], cd->fw_version.patch_vid[3]);
	return ret;
}

static int gti_set_irq_mode(void *private_data, struct gti_irq_cmd *cmd)
{
	struct goodix_ts_core *cd = private_data;
	return cd->hw_ops->irq_enable(cd, cmd->setting == GTI_IRQ_MODE_ENABLE);
}

static int gti_get_irq_mode(void *private_data, struct gti_irq_cmd *cmd)
{
	struct goodix_ts_core *cd = private_data;

	if (atomic_read(&cd->irq_enabled) == 1)
		cmd->setting = GTI_IRQ_MODE_ENABLE;
	else
		cmd->setting = GTI_IRQ_MODE_DISABLE;

	return 0;
}

static int gti_reset(void *private_data, struct gti_reset_cmd *cmd)
{
	struct goodix_ts_core *cd = private_data;

	if (cmd->setting == GTI_RESET_MODE_HW || cmd->setting == GTI_RESET_MODE_AUTO)
		return cd->hw_ops->reset(cd, goodix_get_normal_reset_delay(cd));
	else
		return -EOPNOTSUPP;
}

static int gti_ping(void *private_data, struct gti_ping_cmd *cmd)
{
	struct goodix_ts_core *cd = private_data;
	return cd->hw_ops->ping(cd);
}

static int gti_calibrate(void *private_data, struct gti_calibrate_cmd *cmd)
{
	(void)private_data;

	/* Return successful calibration since there is nothing to do. */
	cmd->result = GTI_CALIBRATE_RESULT_DONE;
	return 0;
}

static int gti_selftest(void *private_data, struct gti_selftest_cmd *cmd)
{
	struct goodix_ts_core *cd = private_data;
	int ret = 0;
	bool test_result = true;

	ret =  driver_test_selftest(cd, cmd->buffer, &test_result, cmd->is_ical);
	cmd->result = test_result ? GTI_SELFTEST_RESULT_DONE :
		GTI_SELFTEST_RESULT_FAIL;
	return ret;
}

static int gti_get_context_driver(void *private_data,
	struct gti_context_driver_cmd *cmd)
{
	/* There is no context from this driver. */
	return 0;
}

static int gti_set_report_rate(void *private_data,
	struct gti_report_rate_cmd *cmd)
{
	struct goodix_ts_core *cd = private_data;
	return cd->hw_ops->set_report_rate(cd, cmd->setting);
}

static int gti_set_panel_speed_mode(void *private_data,
	struct gti_panel_speed_mode_cmd *cmd)
{
	struct goodix_ts_core *cd = private_data;
	return cd->hw_ops->set_panel_speed_mode(cd,
		cmd->setting == GTI_PANEL_SPEED_MODE_HS);
}

#endif
/*~[GOOG] */

/* prosfs create */
static int rawdata_proc_show(struct seq_file *m, void *v)
{
	struct ts_rawdata_info *info;
	struct goodix_ts_core *cd = m->private;
	int tx;
	int rx;
	int ret;
	int i;
	int index;

	if (!m || !v || !cd)
		return -EIO;

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	ret = cd->hw_ops->get_capacitance_data(cd, info);
	if (ret < 0) {
		ts_err("failed to get_capacitance_data, exit!");
		goto exit;
	}

	rx = info->buff[0];
	tx = info->buff[1];
	seq_printf(m, "TX:%d  RX:%d\n", tx, rx);
	seq_puts(m, "mutual_rawdata:\n");
	index = 2;
	for (i = 0; i < tx * rx; i++) {
		seq_printf(m, "%5d,", info->buff[index + i]);
		if ((i + 1) % tx == 0)
			seq_puts(m, "\n");
	}
	seq_puts(m, "mutual_diffdata:\n");
	index += tx * rx;
	for (i = 0; i < tx * rx; i++) {
		seq_printf(m, "%3d,", info->buff[index + i]);
		if ((i + 1) % tx == 0)
			seq_puts(m, "\n");
	}

exit:
	kfree(info);
	return ret;
}

static int rawdata_proc_open(struct inode *inode, struct file *file)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 17, 0))
	return single_open_size(
		file, rawdata_proc_show, pde_data(inode), PAGE_SIZE * 10);
#else
	return single_open_size(
		file, rawdata_proc_show, PDE_DATA(inode), PAGE_SIZE * 10);
#endif
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0))
static const struct proc_ops rawdata_proc_fops = {
	.proc_open = rawdata_proc_open,
	.proc_read = seq_read,
	.proc_lseek = seq_lseek,
	.proc_release = single_release,
};
#else
static const struct file_operations rawdata_proc_fops = {
	.open = rawdata_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};
#endif

static int goodix_ts_procfs_init(struct goodix_ts_core *core_data)
{
	int dev_id = core_data->pdev->id;
	struct proc_dir_entry *proc_entry;
	char proc_node[32] = {0};
	int ret = 0; /* [GOOG] */

	sprintf(proc_node, "goodix_ts.%d", dev_id);

	core_data->proc_dir_entry = proc_mkdir(proc_node, NULL);
	if (!core_data->proc_dir_entry)
		return -ENOMEM;
	proc_entry = proc_create_data("tp_capacitance_data",
			0664, core_data->proc_dir_entry, &rawdata_proc_fops, core_data);
	if (!proc_entry) {
		ts_err("failed to create proc entry: goodix_ts.%d/tp_capacitance_data",
			dev_id);
		ret = -ENOMEM;
		goto err_create_data;
	}

	ret = driver_test_proc_init(core_data);
	if (ret != 0) {
		ts_err("failed to create proc entry: goodix_ts.%d/driver_test", dev_id);
		ret = -ENOMEM;
		goto err_create_driver;
	}

	/*
	 * [GOOG]
	 * Create symlink `goodix_ts` to `goodix_ts.0` for backward compatibility.
	 */
	if (dev_id == 0)
		proc_symlink("goodix_ts", NULL, proc_node);

	return 0;

err_create_driver:
	remove_proc_entry("tp_capacitance_data", core_data->proc_dir_entry);
err_create_data:
	remove_proc_entry(proc_node, NULL);
	return ret;
}

static void goodix_ts_procfs_exit(struct goodix_ts_core *core_data)
{
	int dev_id = core_data->pdev->id;
	char proc_node[32] = {0};

	sprintf(proc_node, "goodix_ts.%d", dev_id);

	driver_test_proc_remove(core_data);
	remove_proc_entry("tp_capacitance_data", core_data->proc_dir_entry);
	remove_proc_entry(proc_node, NULL);
}

#if IS_ENABLED(CONFIG_OF)
/**
 * goodix_parse_dt_resolution - parse resolution from dt
 * @node: devicetree node
 * @board_data: pointer to board data structure
 * return: 0 - no error, <0 error
 */
static int goodix_parse_dt_resolution(
	struct device_node *node, struct goodix_ts_board_data *board_data)
{
	int ret;

	ret = of_property_read_u32(
		node, "goodix,panel-max-x", &board_data->panel_max_x);
	if (ret) {
		ts_err("failed get panel-max-x");
		return ret;
	}

	ret = of_property_read_u32(
		node, "goodix,panel-max-y", &board_data->panel_max_y);
	if (ret) {
		ts_err("failed get panel-max-y");
		return ret;
	}

	ret = of_property_read_u32(
		node, "goodix,panel-max-w", &board_data->panel_max_w);
	if (ret) {
		ts_err("failed get panel-max-w");
		return ret;
	}

	ret = of_property_read_u32(
		node, "goodix,panel-max-p", &board_data->panel_max_p);
	if (ret) {
		ts_err("failed get panel-max-p, use default");
		board_data->panel_max_p = GOODIX_PEN_MAX_PRESSURE;
	}

	ret = of_property_read_u32(
		node, "goodix,panel-height-mm", &board_data->panel_height_mm);
	if (ret) {
		ts_err("failed get panel-height-mm");
		return ret;
	}

	return 0;
}

/**
 * goodix_parse_dt - parse board data from dt
 * @dev: pointer to device
 * @board_data: pointer to board data structure
 * return: 0 - no error, <0 error
 */
static int goodix_parse_dt(
	struct device_node *node, struct goodix_ts_board_data *board_data)
{
	const char *name_tmp;
	int r;
	int index;
	struct of_phandle_args panelmap;
	struct drm_panel *panel = NULL;
	const char *name;
#if IS_ENABLED(CONFIG_GOOG_TOUCH_INTERFACE)
	int panel_id = -1;
#endif

	if (!board_data) {
		ts_err("invalid board data");
		return -EINVAL;
	}

	r = of_get_named_gpio(node, "goodix,avdd-gpio", 0);
	if (r < 0) {
		ts_info("can't find avdd-gpio, use other power supply");
		board_data->avdd_gpio = 0;
	} else {
		ts_info("get avdd-gpio[%d] from dt", r);
		board_data->avdd_gpio = r;
	}

	r = of_get_named_gpio(node, "goodix,iovdd-gpio", 0);
	if (r < 0) {
		ts_info("can't find iovdd-gpio, use other power supply");
		board_data->iovdd_gpio = 0;
	} else {
		ts_info("get iovdd-gpio[%d] from dt", r);
		board_data->iovdd_gpio = r;
	}

	r = of_get_named_gpio(node, "goodix,reset-gpio", 0);
	if (r < 0) {
		ts_err("invalid reset-gpio in dt: %d", r);
		return -EINVAL;
	}
	ts_info("get reset-gpio[%d] from dt", r);
	board_data->reset_gpio = r;

	r = of_get_named_gpio(node, "goodix,irq-gpio", 0);
	if (r < 0) {
		ts_err("invalid irq-gpio in dt: %d", r);
		return -EINVAL;
	}
	ts_info("get irq-gpio[%d] from dt", r);
	board_data->irq_gpio = r;

	r = of_property_read_u32(
		node, "goodix,irq-flags", &board_data->irq_flags);
	if (r) {
		ts_err("invalid irq-flags");
		return -EINVAL;
	}

	memset(board_data->avdd_name, 0, sizeof(board_data->avdd_name));
	r = of_property_read_string(node, "goodix,avdd-name", &name_tmp);
	if (!r) {
		ts_info("avdd name from dt: %s", name_tmp);
		if (strlen(name_tmp) < sizeof(board_data->avdd_name))
			strncpy(board_data->avdd_name, name_tmp,
				sizeof(board_data->avdd_name));
		else
			ts_info("invalied avdd name length: %ld > %ld",
				strlen(name_tmp),
				sizeof(board_data->avdd_name));
	}

	memset(board_data->iovdd_name, 0, sizeof(board_data->iovdd_name));
	r = of_property_read_string(node, "goodix,iovdd-name", &name_tmp);
	if (!r) {
		ts_info("iovdd name from dt: %s", name_tmp);
		if (strlen(name_tmp) < sizeof(board_data->iovdd_name))
			strncpy(board_data->iovdd_name, name_tmp,
				sizeof(board_data->iovdd_name));
		else
			ts_info("invalied iovdd name length: %ld > %ld",
				strlen(name_tmp),
				sizeof(board_data->iovdd_name));
	}

	/* get use-one-binary flag */
	board_data->use_one_binary =
		of_property_read_bool(node, "goodix,use-one-binary");
	if (board_data->use_one_binary)
		ts_info("use one binary");

#if IS_ENABLED(CONFIG_GOOG_TOUCH_INTERFACE)
	if (of_property_read_bool(node, "goog,panel_map")) {
		panel_id = goog_get_panel_id(node);
		if (panel_id < 0)
			return -EPROBE_DEFER;
		goog_get_firmware_name(node, panel_id, board_data->fw_name, sizeof(board_data->fw_name));
		if (!board_data->use_one_binary)
			goog_get_config_name(node, panel_id, board_data->cfg_bin_name, sizeof(board_data->cfg_bin_name));
		goog_get_test_limits_name(node, panel_id, board_data->test_limits_name, sizeof(board_data->test_limits_name));
	} else if (of_property_read_bool(node, "goodix,panel_map")) {
#else
	if (of_property_read_bool(node, "goodix,panel_map")) {
#endif
		for (index = 0;; index++) {
			r = of_parse_phandle_with_fixed_args(
				node, "goodix,panel_map", 1, index, &panelmap);
			if (r)
				return -EPROBE_DEFER;
			panel = of_drm_find_panel(panelmap.np);
			of_node_put(panelmap.np);
			if (!IS_ERR_OR_NULL(panel)) {
				r = of_property_read_string_index(node,
					"goodix,firmware_names", panelmap.args[0], &name);
				if (r < 0)
					name = TS_DEFAULT_FIRMWARE;

				strncpy(board_data->fw_name, name,
					sizeof(board_data->fw_name));
				ts_info("Firmware name %s",
					board_data->fw_name);

				if (!board_data->use_one_binary) {
					r = of_property_read_string_index(node,
						"goodix,config_names",
						panelmap.args[0], &name);
					if (r < 0)
						name = TS_DEFAULT_CFG_BIN;

					strncpy(board_data->cfg_bin_name, name,
						sizeof(board_data->cfg_bin_name));
					ts_info("Config name %s",
						board_data->cfg_bin_name);
				}

				r = of_property_read_string_index(node,
					"goodix,test_limits_names", panelmap.args[0], &name);
				if (r < 0)
					name = TS_DEFAULT_TEST_LIMITS;

				strncpy(board_data->test_limits_name, name,
					sizeof(board_data->test_limits_name));
				ts_info("test limits name %s",
					board_data->test_limits_name);

				break;
			}
		}
	} else {
		/* get firmware file name */
		r = of_property_read_string(
			node, "goodix,firmware-name", &name_tmp);
		if (!r) {
			ts_info("firmware name from dt: %s", name_tmp);
			strncpy(board_data->fw_name, name_tmp,
				sizeof(board_data->fw_name));
		} else {
			ts_info("can't find firmware name, use default: %s",
				TS_DEFAULT_FIRMWARE);
			strncpy(board_data->fw_name, TS_DEFAULT_FIRMWARE,
				sizeof(board_data->fw_name));
		}

		/* get config file name */
		if (!board_data->use_one_binary) {
			r = of_property_read_string(
				node, "goodix,config-name", &name_tmp);
			if (!r) {
				ts_info("config name from dt: %s", name_tmp);
				strncpy(board_data->cfg_bin_name, name_tmp,
					sizeof(board_data->cfg_bin_name));
			} else {
				ts_info("can't find config name, use default: %s",
					TS_DEFAULT_CFG_BIN);
				strncpy(board_data->cfg_bin_name, TS_DEFAULT_CFG_BIN,
					sizeof(board_data->cfg_bin_name));
			}
		}

		/* get test limits file name */
		r = of_property_read_string(
			node, "goodix,test-limits-name", &name_tmp);
		if (!r) {
			ts_info("test limits name from dt: %s", name_tmp);
			strncpy(board_data->test_limits_name, name_tmp,
				sizeof(board_data->test_limits_name));
		} else {
			/* use default test limits name */
			ts_info("can't find test limits name, use default: %s\n",
				TS_DEFAULT_TEST_LIMITS);
			strncpy(board_data->test_limits_name, TS_DEFAULT_TEST_LIMITS,
				sizeof(board_data->test_limits_name));
		}
	}

	/* get xyz resolutions */
	r = goodix_parse_dt_resolution(node, board_data);
	if (r) {
		ts_err("Failed to parse resolutions:%d", r);
		return r;
	}

	r = of_property_read_u32(node, "goodix,udfps-x", &board_data->udfps_x);
	if (r)
		ts_info("undefined udfps-x(optional)!");

	r = of_property_read_u32(node, "goodix,udfps-y", &board_data->udfps_y);
	if (r)
		ts_info("undefined udfps-y(optional)!");

	/* get sleep mode flag */
	board_data->sleep_enable =
		of_property_read_bool(node, "goodix,sleep-enable");

	/*get pen-enable switch and pen keys, must after "key map"*/
	board_data->pen_enable =
		of_property_read_bool(node, "goodix,pen-enable");

	board_data->noise_test_disable_cmd =
		of_property_read_bool(node, "goodix,noise-test-disable-cmd");

	ts_info("[DT]x:%d, y:%d, w:%d, p:%d sleep_enable:%d pen_enable:%d",
		board_data->panel_max_x, board_data->panel_max_y,
		board_data->panel_max_w, board_data->panel_max_p,
		board_data->sleep_enable, board_data->pen_enable);
	return 0;
}
#endif

static void goodix_ts_report_pen(
	struct goodix_ts_core *cd, struct goodix_pen_data *pen_data)
{
	struct input_dev *dev = cd->pen_dev;
	int i;
	struct goodix_ble_data *ble_data = &cd->ble_data;
	char trace_tag[128];
	ktime_t pen_ktime;

	mutex_lock(&dev->mutex);
	input_set_timestamp(dev, cd->coords_timestamp);
	pen_ktime = ktime_get();

	if (pen_data->coords.status == TS_TOUCH) {
		scnprintf(trace_tag, sizeof(trace_tag),
			"stylus-active: IN_TS=%lld TS=%lld DELTA=%lld ns.\n",
			ktime_to_ns(cd->coords_timestamp), ktime_to_ns(pen_ktime),
			ktime_to_ns(ktime_sub(pen_ktime, cd->coords_timestamp)));
		ATRACE_BEGIN(trace_tag);
		if (pen_data->is_hover)
			input_report_key(dev, BTN_TOUCH, 0);
		else
			input_report_key(dev, BTN_TOUCH, 1);
		input_report_key(dev, BTN_TOOL_PEN, 1);
		input_report_abs(dev, ABS_X, pen_data->coords.x);
		input_report_abs(dev, ABS_Y, pen_data->coords.y);
		mutex_lock(&ble_data->lock);
		if (ble_data->hogp_ready) {
			cd->pen_pressure = ble_data->pressure;
			ts_debug("update pen pressure from ble %d",
				cd->pen_pressure);
		}
		ble_data->hogp_ready = 0;
		mutex_unlock(&ble_data->lock);

		if (pen_data->coords.p && cd->pen_pressure)
			pen_data->coords.p = cd->pen_pressure;
		input_report_abs(dev, ABS_PRESSURE, pen_data->coords.p);
		if (pen_data->coords.p == 0)
			input_report_abs(dev, ABS_DISTANCE, 1);
		else
			input_report_abs(dev, ABS_DISTANCE, 0);
		input_report_abs(dev, ABS_TILT_X, pen_data->coords.tilt_x);
		input_report_abs(dev, ABS_TILT_Y, pen_data->coords.tilt_y);
		ts_debug(
			"pen_data:x %d, y %d, p %d, tilt_x %d tilt_y %d key[%d %d]",
			pen_data->coords.x, pen_data->coords.y,
			pen_data->coords.p, pen_data->coords.tilt_x,
			pen_data->coords.tilt_y,
			pen_data->keys[0].status == TS_TOUCH ? 1 : 0,
			pen_data->keys[1].status == TS_TOUCH ? 1 : 0);

		if (pen_data->custom_flag) {
			if (ble_data->tx1_freq_index != pen_data->tx1_freq_index ||
					ble_data->tx2_freq_index != pen_data->tx2_freq_index) {
				ble_data->tx1_freq_index = pen_data->tx1_freq_index;
				ble_data->tx2_freq_index = pen_data->tx2_freq_index;
				goodix_refresh_pen_pair(cd);
			}
		}
	} else {
		scnprintf(trace_tag, sizeof(trace_tag),
			"stylus-inactive: IN_TS=%lld TS=%lld DELTA=%lld ns.\n",
			ktime_to_ns(cd->coords_timestamp), ktime_to_ns(pen_ktime),
			ktime_to_ns(ktime_sub(pen_ktime, cd->coords_timestamp)));
		ATRACE_BEGIN(trace_tag);
		cd->pen_pressure = 0;
		input_report_key(dev, BTN_TOUCH, 0);
		input_report_key(dev, BTN_TOOL_PEN, 0);
	}
	/* report pen button */
	for (i = 0; i < GOODIX_MAX_PEN_KEY; i++) {
		if (pen_data->keys[i].status == TS_TOUCH)
			input_report_key(dev, pen_data->keys[i].code, 1);
		else
			input_report_key(dev, pen_data->keys[i].code, 0);
	}
	input_sync(dev);
	ATRACE_END();
	mutex_unlock(&dev->mutex);
}

#if !IS_ENABLED(CONFIG_GOOG_TOUCH_INTERFACE)
static void goodix_ts_report_finger(
	struct goodix_ts_core *cd, struct goodix_touch_data *touch_data)
{
	struct input_dev *dev = cd->input_dev;
	unsigned int touch_num = touch_data->touch_num;
	int i;
	int panel_height_mm = cd->board_data.panel_height_mm;
	int panel_height_pixel = cd->board_data.panel_max_y + 1;

	mutex_lock(&dev->mutex);

	for (i = 0; i < GOODIX_MAX_TOUCH; i++) {
		if (touch_data->coords[i].status == TS_TOUCH) {
			ts_debug(
				"report: id[%d], x %d, y %d, w %d, p %d, major %d, minor %d, angle %d",
				i, touch_data->coords[i].x,
				touch_data->coords[i].y,
				touch_data->coords[i].w,
				touch_data->coords[i].p,
				touch_data->coords[i].major,
				touch_data->coords[i].minor,
				touch_data->coords[i].angle);
			input_mt_slot(dev, i);
			input_mt_report_slot_state(dev, MT_TOOL_FINGER, true);
			input_report_abs(dev, ABS_MT_POSITION_X,
				touch_data->coords[i].x);
			input_report_abs(dev, ABS_MT_POSITION_Y,
				touch_data->coords[i].y);
			input_report_abs(
				dev, ABS_MT_PRESSURE, touch_data->coords[i].p);
			input_report_abs(dev, ABS_MT_TOUCH_MAJOR,
				(touch_data->coords[i].major * panel_height_pixel) /
				(10 * panel_height_mm));
			input_report_abs(dev, ABS_MT_TOUCH_MINOR,
				(touch_data->coords[i].minor * panel_height_pixel) /
				(10 * panel_height_mm));
			input_report_abs(dev, ABS_MT_ORIENTATION,
				(touch_data->coords[i].angle * 2048) / 45);
		} else {
			input_mt_slot(dev, i);
			input_mt_report_slot_state(dev, MT_TOOL_FINGER, false);
		}
	}

	input_report_key(dev, BTN_TOUCH, touch_num > 0 ? 1 : 0);
	input_set_timestamp(dev, cd->coords_timestamp);
	input_sync(dev);

#if IS_ENABLED(CONFIG_TOUCHSCREEN_MOTION_FILTER)
	touch_mf_update_state(&cd->tmf, touch_num);
#endif

	mutex_unlock(&dev->mutex);
}
#endif

#if IS_ENABLED(CONFIG_GOOG_TOUCH_INTERFACE)
static void goodix_ts_report_finger_goog(
	struct goodix_ts_core *cd, struct goodix_touch_data *touch_data)
{
	struct input_dev *dev = cd->input_dev;
	struct goog_touch_interface *gti = cd->gti;
	unsigned int touch_num = touch_data->touch_num;
	int i;
	int panel_height_mm = cd->board_data.panel_height_mm;
	int panel_height_pixel = cd->board_data.panel_max_y + 1;

	goog_input_lock(gti);

	goog_input_set_timestamp(gti, dev, cd->coords_timestamp);
	for (i = 0; i < GOODIX_MAX_TOUCH; i++) {
		struct goodix_ts_coords *coord = &touch_data->coords[i];
		if (coord->status == TS_TOUCH) {
			goog_input_mt_slot(gti, dev, i);
			goog_input_mt_report_slot_state(
				gti, dev, MT_TOOL_FINGER, true);
			goog_input_report_abs(
				gti, dev, ABS_MT_POSITION_X, coord->x);
			goog_input_report_abs(
				gti, dev, ABS_MT_POSITION_Y, coord->y);
			goog_input_report_abs(
				gti, dev, ABS_MT_PRESSURE, coord->p);
			goog_input_report_abs(gti, dev, ABS_MT_TOUCH_MAJOR,
				(touch_data->coords[i].major * panel_height_pixel) /
				(10 * panel_height_mm));
			goog_input_report_abs(gti, dev, ABS_MT_TOUCH_MINOR,
				(touch_data->coords[i].minor * panel_height_pixel) /
				(10 * panel_height_mm));
			goog_input_report_abs(
				gti, dev, ABS_MT_ORIENTATION, (coord->angle * 2048) / 45);
		} else {
			goog_input_mt_slot(gti, dev, i);
			goog_input_mt_report_slot_state(
				gti, dev, MT_TOOL_FINGER, false);
		}
	}

	goog_input_report_key(gti, dev, BTN_TOUCH, touch_num > 0 ? 1 : 0);
	goog_input_sync(gti, dev);

	goog_input_unlock(gti);

#if IS_ENABLED(CONFIG_TOUCHSCREEN_MOTION_FILTER)
	touch_mf_update_state(&cd->tmf, touch_num);
#endif
}
#endif

static void goodix_ts_report_gesture_up(struct goodix_ts_core *cd)
{
	struct input_dev *dev = cd->input_dev;

	ts_info("goodix_ts_report_gesture_up");

	mutex_lock(&dev->mutex);

	input_set_timestamp(dev, cd->coords_timestamp);

	/* Finger down on UDFPS area. */
	input_mt_slot(dev, 0);
	input_report_key(dev, BTN_TOUCH, 1);
	input_mt_report_slot_state(dev, MT_TOOL_FINGER, 1);
	input_report_abs(dev, ABS_MT_POSITION_X, cd->board_data.udfps_x);
	input_report_abs(dev, ABS_MT_POSITION_Y, cd->board_data.udfps_y);
	input_report_abs(dev, ABS_MT_TOUCH_MAJOR, 200);
	input_report_abs(dev, ABS_MT_TOUCH_MINOR, 200);
#ifndef SKIP_PRESSURE
	input_report_abs(dev, ABS_MT_PRESSURE, 1);
#endif
	/*input_report_abs(dev, ABS_MT_ORIENTATION,
		ts_data->fts_gesture_data.orientation[0]);*/
	input_sync(dev);

	/* Report MT_TOOL_PALM for canceling the touch event. */
	input_mt_slot(dev, 0);
	input_report_key(dev, BTN_TOUCH, 1);
	input_mt_report_slot_state(dev, MT_TOOL_PALM, 1);
	input_sync(dev);

	/* Release touches. */
	input_mt_slot(dev, 0);
#ifndef SKIP_PRESSURE
	input_report_abs(dev, ABS_MT_PRESSURE, 0);
#endif
	input_mt_report_slot_state(dev, MT_TOOL_FINGER, 0);
	input_report_abs(dev, ABS_MT_TRACKING_ID, -1);
	input_report_key(dev, BTN_TOUCH, 0);
	input_sync(dev);

	mutex_unlock(&dev->mutex);
}

static int goodix_ts_request_handle(
	struct goodix_ts_core *cd, struct goodix_ts_event *ts_event)
{
	struct goodix_ts_hw_ops *hw_ops = cd->hw_ops;
	int ret = -1;

	if (ts_event->request_code == REQUEST_TYPE_CONFIG)
		ret = goodix_send_ic_config(cd, CONFIG_TYPE_NORMAL);
	else if (ts_event->request_code == REQUEST_TYPE_RESET)
		ret = hw_ops->reset(cd, goodix_get_normal_reset_delay(cd));
	else if (ts_event->request_code == REQUEST_TYPE_UPDATE)
		ret = goodix_do_fw_update(cd, UPDATE_MODE_FORCE | UPDATE_MODE_BLOCK |
			UPDATE_MODE_SRC_REQUEST);
	else
		ts_info("can not handle request type 0x%x",
			ts_event->request_code);
	if (ret)
		ts_err("failed handle request 0x%x", ts_event->request_code);
	else
		ts_info("success handle ic request 0x%x",
			ts_event->request_code);
	return ret;
}

static irqreturn_t goodix_ts_isr(int irq, void *data)
{
	struct goodix_ts_core *core_data = data;

	core_data->isr_timestamp = ktime_get();

	return IRQ_WAKE_THREAD;
}

void goodix_ts_report_status(struct goodix_ts_core *core_data,
	struct goodix_ts_event *ts_event)
{
	struct goodix_status_data *st = &ts_event->status_data;
	int i;
	u8 checksum = 0;
	int len = sizeof(ts_event->status_data);
	u8 *data = (u8 *)st;
#if IS_ENABLED(CONFIG_GOOG_TOUCH_INTERFACE)
	struct gti_fw_status_data status_data = { 0 };
#endif

	for (i = 0; i < len - 1; i++)
		checksum += data[i];
	if (checksum != st->checksum) {
		ts_err("status data checksum error");
		return;
	}

	ts_info("others_change[%d] grip_change[%d] noise_lv_change[%d] palm_change[%d]"
		"soft_reset[%d] base_update[%d] hop_change[%d] water_change[%d]",
		st->others_change, st->grip_change, st->noise_lv_change,
		st->palm_change, st->soft_reset, st->base_update,
		st->hop_change, st->water_change);
	ts_info("water_status[%d] before_factorA[%d] after_factorA[%d] base_update_type[0x%x]\n"
		"soft_reset_type[0x%x] palm_status[%d] noise_lv[%d] grip_type[%d] \n"
		"wireless_mode[%d] fw_sta[%x] sys_cmd[%x] fw_hs_ns[%x] hsync_err[%x] event_id[%d] \n"
		"clear_count1[%d] clear_count2[%d]",
		st->water_sta, st->before_factorA, st->after_factorA,
		st->base_update_type, st->soft_reset_type, st->palm_sta,
		st->noise_lv, st->grip_type, st->wireless_mode,
		st->fw_sta, st->sys_cmd, st->fw_hs_ns, st->hsync_error,
		st->event_id, ts_event->clear_count1, ts_event->clear_count2);
#if IS_ENABLED(CONFIG_GOOG_TOUCH_INTERFACE)
	if (st->soft_reset)
		goog_notify_fw_status_changed(core_data->gti, GTI_FW_STATUS_RESET,
			&status_data);

	if (st->palm_change) {
		goog_notify_fw_status_changed(core_data->gti,
			st->palm_sta ? GTI_FW_STATUS_PALM_ENTER : GTI_FW_STATUS_PALM_EXIT,
			&status_data);
	}

	if (st->grip_change) {
		goog_notify_fw_status_changed(core_data->gti,
			st->grip_type ? GTI_FW_STATUS_GRIP_ENTER : GTI_FW_STATUS_GRIP_EXIT,
			&status_data);
	}

	if (st->water_change) {
		goog_notify_fw_status_changed(core_data->gti,
			st->water_sta ? GTI_FW_STATUS_WATER_ENTER :
			GTI_FW_STATUS_WATER_EXIT, &status_data);
	}

	if (st->noise_lv_change) {
		status_data.noise_level = st->noise_lv;
		goog_notify_fw_status_changed(core_data->gti, GTI_FW_STATUS_NOISE_MODE,
			&status_data);
	}
#endif
}

/**
 * goodix_ts_threadirq_func - Bottom half of interrupt
 * This functions is excuted in thread context,
 * sleep in this function is permit.
 *
 * @data: pointer to touch core data
 * return: 0 ok, <0 failed
 */
static irqreturn_t goodix_ts_threadirq_func(int irq, void *data)
{
	struct goodix_ts_core *core_data = data;
	struct goodix_ts_hw_ops *hw_ops = core_data->hw_ops;
	struct goodix_ts_event *ts_event = &core_data->ts_event;
	struct goodix_ts_esd *ts_esd = &core_data->ts_esd;
	int ret;

/* [GOOG]
 * Remove the control to enable/disable the interrupt for bottom-half.
	disable_irq_nosync(core_data->irq);
 */

	/* [GOOG]
	 * Since we received an interrupt from touch firmware, it means touch
	 * firmware is still alive. So skip esd check once.
	 */
	ts_esd->skip_once = true;

	core_data->irq_trig_cnt++;

	/* read touch data from touch device */
	ret = hw_ops->event_handler(core_data, ts_event);
	if (likely(!ret)) {
		if (ts_event->event_type & EVENT_TOUCH) {
			/* report touch */
			core_data->coords_timestamp = core_data->isr_timestamp;
#if IS_ENABLED(CONFIG_GOOG_TOUCH_INTERFACE)
			goodix_ts_report_finger_goog(
				core_data, &ts_event->touch_data);
#else
			goodix_ts_report_finger(
				core_data, &ts_event->touch_data);
#endif
		}
		if (ts_event->event_type & EVENT_GESTURE) {
			core_data->coords_timestamp = core_data->isr_timestamp;
			mutex_lock(&core_data->gesture_data_lock);
			memcpy(&core_data->gesture_data, &core_data->ts_event.temp_gesture_data,
				sizeof(core_data->gesture_data));
			mutex_unlock(&core_data->gesture_data_lock);
		}
		if (core_data->board_data.pen_enable &&
			ts_event->event_type & EVENT_PEN) {
			core_data->coords_timestamp = core_data->isr_timestamp;
			goodix_ts_report_pen(core_data, &ts_event->pen_data);
		}
		/* [GOOG]
		 * Move to goodix_ts_post_threadirq_func.
		if (ts_event->event_type & EVENT_REQUEST)
			goodix_ts_request_handle(core_data, ts_event);
		if (ts_event->event_type & EVENT_STATUS)
			goodix_ts_report_status(core_data, ts_event);
		 */
		/* [GOOG]
		 * Don't need to report gesture events in our use cases.
		if (ts_event->event_type & EVENT_GESTURE)
			goodix_ts_report_gesture(core_data, ts_event);
		 */
	}

/* [GOOG]
 * Remove the control to enable/disable the interrupt for bottom-half.
	enable_irq(core_data->irq);
 */

	return IRQ_HANDLED;
}

static irqreturn_t goodix_ts_post_threadirq_func(int irq, void *data)
{
	struct goodix_ts_core *core_data = data;
	struct goodix_ts_hw_ops *hw_ops = core_data->hw_ops;
	struct goodix_ts_event *ts_event = &core_data->ts_event;

	if (ts_event->event_type != EVENT_INVALID) {
		if (ts_event->event_type & EVENT_REQUEST)
			goodix_ts_request_handle(core_data, ts_event);

		if (ts_event->event_type & EVENT_STATUS) {
			hw_ops->read(core_data, 0x1021C, (u8 *)&ts_event->status_data,
				sizeof(ts_event->status_data));
			goodix_ts_report_status(core_data, ts_event);
		}

		/* read done */
		hw_ops->after_event_handler(core_data); /* [GOOG] */
	}

	return IRQ_HANDLED;
}

/**
 * goodix_ts_init_irq - Request interrput line from system
 * @core_data: pointer to touch core data
 * return: 0 ok, <0 failed
 */
static int goodix_ts_irq_setup(struct goodix_ts_core *core_data)
{
	const struct goodix_ts_board_data *ts_bdata = board_data(core_data);
	int ret;

	/* if ts_bdata-> irq is invalid */
	core_data->irq = gpio_to_irq(ts_bdata->irq_gpio);
	if (core_data->irq < 0) {
		ts_err("failed get irq num %d", core_data->irq);
		return -EINVAL;
	}

	ts_info("IRQ:%u,flags:%d", core_data->irq, (int)ts_bdata->irq_flags);
#if IS_ENABLED(CONFIG_GOOG_TOUCH_INTERFACE)
	ret = goog_devm_request_threaded_irq(core_data->gti,
#else
	ret = devm_request_threaded_irq(
#endif
		&core_data->pdev->dev, core_data->irq,
		goodix_ts_isr, goodix_ts_threadirq_func,
		ts_bdata->irq_flags | IRQF_ONESHOT, GOODIX_CORE_DRIVER_NAME,
		core_data);
	if (ret < 0)
		ts_err("Failed to requeset threaded irq:%d", ret);
	else
		atomic_set(&core_data->irq_enabled, 1);

	return ret;
}

/**
 * goodix_ts_power_init - Get regulator for touch device
 * @core_data: pointer to touch core data
 * return: 0 ok, <0 failed
 */
static int goodix_ts_power_init(struct goodix_ts_core *core_data)
{
	struct goodix_ts_board_data *ts_bdata = board_data(core_data);
	struct device *dev = core_data->bus->dev;
	int ret = 0;

	ts_info("Power init");
	if (strlen(ts_bdata->avdd_name)) {
		core_data->avdd = devm_regulator_get(dev, ts_bdata->avdd_name);
		if (IS_ERR_OR_NULL(core_data->avdd)) {
			ret = PTR_ERR(core_data->avdd);
			ts_err("Failed to get regulator avdd:%d", ret);
			core_data->avdd = NULL;
			return ret;
		}
	} else {
		ts_info("Avdd name is NULL");
	}

	if (strlen(ts_bdata->iovdd_name)) {
		core_data->iovdd =
			devm_regulator_get(dev, ts_bdata->iovdd_name);
		if (IS_ERR_OR_NULL(core_data->iovdd)) {
			ret = PTR_ERR(core_data->iovdd);
			ts_err("Failed to get regulator iovdd:%d", ret);
			core_data->iovdd = NULL;
		}
	} else {
		ts_info("iovdd name is NULL");
	}

	return ret;
}

/**
 * goodix_ts_power_on - Turn on power to the touch device
 * @core_data: pointer to touch core data
 * return: 0 ok, <0 failed
 */
static int goodix_ts_power_on(struct goodix_ts_core *cd)
{
	int ret = 0;

	ts_info("Device power on");
	if (cd->power_on)
		return 0;

	ret = cd->hw_ops->power_on(cd, true);
	if (!ret)
		cd->power_on = 1;
	else
		ts_err("failed power on, %d", ret);
	return ret;
}

/**
 * goodix_ts_power_off - Turn off power to the touch device
 * @core_data: pointer to touch core data
 * return: 0 ok, <0 failed
 */
static int goodix_ts_power_off(struct goodix_ts_core *cd)
{
	int ret;

	ts_info("Device power off");
	if (!cd->power_on)
		return 0;

	ret = cd->hw_ops->power_on(cd, false);
	if (!ret)
		cd->power_on = 0;
	else
		ts_err("failed power off, %d", ret);

	return ret;
}

/**
 * goodix_ts_gpio_setup - Request gpio resources from GPIO subsysten
 * @core_data: pointer to touch core data
 * return: 0 ok, <0 failed
 */
static int goodix_ts_gpio_setup(struct goodix_ts_core *core_data)
{
	struct goodix_ts_board_data *ts_bdata = board_data(core_data);
	int r = 0;

	ts_info("GPIO setup,reset-gpio:%d, irq-gpio:%d", ts_bdata->reset_gpio,
		ts_bdata->irq_gpio);
	/*
	 * after kenerl3.13, gpio_ api is deprecated, new
	 * driver should use gpiod_ api.
	 */
	r = devm_gpio_request_one(&core_data->pdev->dev, ts_bdata->reset_gpio,
		GPIOF_OUT_INIT_LOW, "ts_reset_gpio");
	if (r < 0) {
		ts_err("Failed to request reset gpio, r:%d", r);
		return r;
	}

	r = devm_gpio_request_one(&core_data->pdev->dev, ts_bdata->irq_gpio,
		GPIOF_IN, "ts_irq_gpio");
	if (r < 0) {
		ts_err("Failed to request irq gpio, r:%d", r);
		return r;
	}

	if (ts_bdata->avdd_gpio > 0) {
		r = devm_gpio_request_one(&core_data->pdev->dev,
			ts_bdata->avdd_gpio, GPIOF_OUT_INIT_LOW,
			"ts_avdd_gpio");
		if (r < 0) {
			ts_err("Failed to request avdd-gpio, r:%d", r);
			return r;
		}
	}

	if (ts_bdata->iovdd_gpio > 0) {
		r = devm_gpio_request_one(&core_data->pdev->dev,
			ts_bdata->iovdd_gpio, GPIOF_OUT_INIT_LOW,
			"ts_iovdd_gpio");
		if (r < 0) {
			ts_err("Failed to request iovdd-gpio, r:%d", r);
			return r;
		}
	}

	return 0;
}

static int goodix_pinctrl_init(struct goodix_ts_core *core_data)
{
	struct goodix_ts_board_data *ts_bdata = board_data(core_data);

	ts_bdata->pinctrl = devm_pinctrl_get(core_data->bus->dev);
	ts_bdata->state_active =
		pinctrl_lookup_state(ts_bdata->pinctrl, "ts_active");
	if (IS_ERR_OR_NULL(ts_bdata->state_active)) {
		ts_err("Could not get active pinstate\n");
		return -ENODEV;
	}

	ts_bdata->state_suspend =
		pinctrl_lookup_state(ts_bdata->pinctrl, "ts_suspend");
	if (IS_ERR_OR_NULL(ts_bdata->state_suspend)) {
		ts_err("Could not get suspend pinstate\n");
		return -ENODEV;
	}

	return 0;
}

static int goodix_set_pinctrl_state(
	struct goodix_ts_core *core_data, enum PINCTRL_MODE mode)
{
	struct goodix_ts_board_data *ts_bdata = board_data(core_data);
	struct pinctrl_state *state;

	ts_debug("goodix_set_pinctrl_state: %s\n",
		mode == PINCTRL_MODE_ACTIVE ? "ACTIVE" : "SUSPEND");

	state = mode == PINCTRL_MODE_ACTIVE ? ts_bdata->state_active
					    : ts_bdata->state_suspend;
	return pinctrl_select_state(ts_bdata->pinctrl, state);
}

/**
 * goodix_ts_input_dev_config - Request and config a input device
 *  then register it to input sybsystem.
 * @core_data: pointer to touch core data
 * return: 0 ok, <0 failed
 */
static int goodix_ts_input_dev_config(struct goodix_ts_core *core_data)
{
	struct goodix_ts_board_data *ts_bdata = board_data(core_data);
	struct input_dev *input_dev = NULL;
	int max_x = ts_bdata->panel_max_x;
	int max_y = ts_bdata->panel_max_y;
	int dev_id = core_data->pdev->id;
	int r;

	input_dev = input_allocate_device();
	if (!input_dev) {
		ts_err("Failed to allocated input device");
		return -ENOMEM;
	}

	sprintf(core_data->input_name, "%s%d", GOODIX_CORE_DRIVER_NAME, dev_id);
	input_dev->dev.parent = &core_data->pdev->dev; /* [GOOG] */
	input_dev->name = core_data->input_name;
        input_dev->uniq = "google_touchscreen";
	input_dev->phys = input_dev->name;
	input_dev->id.bustype = core_data->bus->bus_type;
	input_dev->id.product = 0x0100 + dev_id;
	input_dev->id.vendor = 0x27C6;
	input_dev->id.version = 0x0100;

	set_bit(EV_SYN, input_dev->evbit);
	set_bit(EV_KEY, input_dev->evbit);
	set_bit(EV_ABS, input_dev->evbit);
	set_bit(BTN_TOUCH, input_dev->keybit);
	set_bit(BTN_TOOL_FINGER, input_dev->keybit);
	set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

	if (core_data->ic_info.other.screen_max_x > 0 &&
		core_data->ic_info.other.screen_max_y > 0) {
		max_x = core_data->ic_info.other.screen_max_x;
		max_y = core_data->ic_info.other.screen_max_y;
	}

	/* set input parameters */
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, max_x - 1, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, max_y - 1, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, ts_bdata->panel_max_y, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MINOR, 0, ts_bdata->panel_max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_ORIENTATION, -4096, 4096, 0, 0);
	input_set_abs_params(
		input_dev, ABS_MT_TOOL_TYPE, MT_TOOL_FINGER, MT_TOOL_PALM, 0, 0);
#ifdef INPUT_TYPE_B_PROTOCOL
#if LINUX_VERSION_CODE > KERNEL_VERSION(3, 7, 0)
	input_mt_init_slots(input_dev, GOODIX_MAX_TOUCH, INPUT_MT_DIRECT);
#else
	input_mt_init_slots(input_dev, GOODIX_MAX_TOUCH);
#endif
#endif

	input_set_capability(input_dev, EV_KEY, KEY_POWER);
	input_set_capability(input_dev, EV_KEY, KEY_WAKEUP);
	input_set_capability(input_dev, EV_KEY, KEY_GOTO);

	core_data->ble_data.tx1_freq_index = 0xFF;
	core_data->ble_data.tx2_freq_index = 0xFF;

	r = input_register_device(input_dev);
	if (r < 0) {
		ts_err("Unable to register input device");
		input_free_device(input_dev);
		return r;
	}

	core_data->input_dev = input_dev;
	input_set_drvdata(input_dev, core_data);

	return 0;
}

static int goodix_ts_pen_dev_config(struct goodix_ts_core *core_data)
{
	struct goodix_ts_board_data *ts_bdata = board_data(core_data);
	struct input_dev *pen_dev = NULL;
	int dev_id = core_data->pdev->id;
	int r;

	pen_dev = input_allocate_device();
	if (!pen_dev) {
		ts_err("Failed to allocated pen device");
		return -ENOMEM;
	}

	sprintf(core_data->input_pen_name, "%s%d%s", GOODIX_CORE_DRIVER_NAME, dev_id, ",pen");
	pen_dev->dev.parent = &core_data->pdev->dev; /* [GOOG] */
	pen_dev->name = core_data->input_pen_name;
	pen_dev->uniq = pen_dev->name;
	pen_dev->phys = pen_dev->name;
	pen_dev->id.bustype = core_data->bus->bus_type;
	pen_dev->id.product = 0x0200 + dev_id;
	pen_dev->id.vendor = 0x27C6;
	pen_dev->id.version = 0x0100;

	pen_dev->evbit[0] |= BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	set_bit(ABS_X, pen_dev->absbit);
	set_bit(ABS_Y, pen_dev->absbit);
	set_bit(ABS_TILT_X, pen_dev->absbit);
	set_bit(ABS_TILT_Y, pen_dev->absbit);
	set_bit(BTN_STYLUS, pen_dev->keybit);
	set_bit(BTN_STYLUS2, pen_dev->keybit);
	set_bit(BTN_TOUCH, pen_dev->keybit);
	set_bit(BTN_TOOL_PEN, pen_dev->keybit);
	set_bit(INPUT_PROP_DIRECT, pen_dev->propbit);
	input_set_abs_params(pen_dev, ABS_X, 0, ts_bdata->panel_max_x, 0, 0);
	input_set_abs_params(pen_dev, ABS_Y, 0, ts_bdata->panel_max_y, 0, 0);
	input_set_abs_params(
		pen_dev, ABS_PRESSURE, 0, ts_bdata->panel_max_p, 0, 0);
	input_set_abs_params(pen_dev, ABS_DISTANCE, 0, 255, 0, 0);
	input_set_abs_params(pen_dev, ABS_TILT_X, -GOODIX_PEN_MAX_TILT,
		GOODIX_PEN_MAX_TILT, 0, 0);
	input_set_abs_params(pen_dev, ABS_TILT_Y, -GOODIX_PEN_MAX_TILT,
		GOODIX_PEN_MAX_TILT, 0, 0);

	r = input_register_device(pen_dev);
	if (r < 0) {
		ts_err("Unable to register pen device");
		input_free_device(pen_dev);
		return r;
	}

	core_data->pen_dev = pen_dev;
	input_set_drvdata(pen_dev, core_data);

	return 0;
}

static void goodix_ts_input_dev_remove(struct goodix_ts_core *core_data)
{
	if (!core_data->input_dev)
		return;
	input_unregister_device(core_data->input_dev);
	core_data->input_dev = NULL;
}

static void goodix_ts_pen_dev_remove(struct goodix_ts_core *core_data)
{
	if (!core_data->pen_dev)
		return;
	mutex_destroy(&core_data->ble_data.lock);
	input_unregister_device(core_data->pen_dev);
	core_data->pen_dev = NULL;
}

/**
 * goodix_ts_esd_work - check hardware status and recovery
 *  the hardware if needed.
 */
static void goodix_ts_esd_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct goodix_ts_esd *ts_esd =
		container_of(dwork, struct goodix_ts_esd, esd_work);
	struct goodix_ts_core *cd =
		container_of(ts_esd, struct goodix_ts_core, ts_esd);
	const struct goodix_ts_hw_ops *hw_ops = cd->hw_ops;
	int ret = 0;

	if (ts_esd->skip_once)
		goto exit;

	if (!atomic_read(&ts_esd->esd_on) || atomic_read(&cd->suspended))
		return;

	if (!hw_ops->esd_check)
		return;

	ret = hw_ops->esd_check(cd);
	if (ret) {
		ts_err("esd check failed");
		gpio_direction_output(cd->board_data.reset_gpio, 0);
		if (cd->iovdd)
			ret = regulator_disable(cd->iovdd);
		if (cd->avdd)
			ret = regulator_disable(cd->avdd);

		usleep_range(5000, 5100);

		if (cd->iovdd) {
			ret = regulator_enable(cd->iovdd);
			usleep_range(3000, 3100);
		}
		if (cd->avdd)
			ret = regulator_enable(cd->avdd);
		usleep_range(15000, 15100);
		gpio_direction_output(cd->board_data.reset_gpio, 1);
	}

exit:
	ts_esd->skip_once = false;
	if (atomic_read(&ts_esd->esd_on))
		schedule_delayed_work(&ts_esd->esd_work, 2 * HZ);
}

/**
 * goodix_ts_esd_on - turn on esd protection
 */
void goodix_ts_esd_on(struct goodix_ts_core *cd)
{
	struct goodix_ic_info_misc *misc = &cd->ic_info.misc;
	struct goodix_ts_esd *ts_esd = &cd->ts_esd;

	if (!misc->esd_addr)
		return;

	if (atomic_read(&ts_esd->esd_on))
		return;

	atomic_set(&ts_esd->esd_on, 1);
	if (!schedule_delayed_work(&ts_esd->esd_work, 2 * HZ))
		ts_info("esd work already in workqueue");

	ts_info("esd on");
}

/**
 * goodix_ts_esd_off - turn off esd protection
 */
void goodix_ts_esd_off(struct goodix_ts_core *cd)
{
	struct goodix_ts_esd *ts_esd = &cd->ts_esd;
	int ret;

	if (!atomic_read(&ts_esd->esd_on))
		return;

	atomic_set(&ts_esd->esd_on, 0);
	ret = cancel_delayed_work_sync(&ts_esd->esd_work);
	ts_info("Esd off, esd work state %d", ret);
}

/**
 * goodix_ts_esd_init - initialize esd protection
 */
static int goodix_ts_esd_init(struct goodix_ts_core *cd)
{
	struct goodix_ic_info_misc *misc = &cd->ic_info.misc;
	struct goodix_ts_esd *ts_esd = &cd->ts_esd;

	if (!cd->hw_ops->esd_check || !misc->esd_addr) {
		ts_info("missing key info for esd check");
		return 0;
	}

	INIT_DELAYED_WORK(&ts_esd->esd_work, goodix_ts_esd_work);
	ts_esd->ts_core = cd;
	atomic_set(&ts_esd->esd_on, 0);
	goodix_ts_esd_on(cd);

	return 0;
}

static void goodix_ts_esd_uninit(struct goodix_ts_core *cd)
{
	struct goodix_ts_esd *ts_esd = &cd->ts_esd;
	if (atomic_read(&ts_esd->esd_on))
		goodix_ts_esd_off(cd);
}

#if IS_ENABLED(CONFIG_GOOG_TOUCH_INTERFACE)
static void goodix_ts_release_connects(struct goodix_ts_core *core_data)
{
}
#else
static void goodix_ts_release_connects(struct goodix_ts_core *core_data)
{
	struct input_dev *input_dev = core_data->input_dev;
	int i;

	mutex_lock(&input_dev->mutex);

	for (i = 0; i < GOODIX_MAX_TOUCH; i++) {
		input_mt_slot(input_dev, i);
		input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, false);
	}
	input_report_key(input_dev, BTN_TOUCH, 0);
	input_mt_sync_frame(input_dev);
	input_sync(input_dev);

	mutex_unlock(&input_dev->mutex);
}
#endif

/**
 * goodix_ts_suspend - Touchscreen suspend function
 * Called by PM/FB/EARLYSUSPEN module to put the device to sleep
 */
static int goodix_ts_suspend(struct goodix_ts_core *core_data)
{
	struct goodix_ts_hw_ops *hw_ops = core_data->hw_ops;

	if (core_data->init_stage < CORE_INIT_STAGE2 ||
		atomic_read(&core_data->suspended))
		return 0;

	ts_info("Suspend start");
	atomic_set(&core_data->suspended, 1);
	/* disable irq */
	hw_ops->disable_irq_nosync(core_data);
	goodix_ts_esd_off(core_data);

	if (core_data->gesture_type) {
		/* enter gesture mode */
		hw_ops->gesture(core_data, 0);
		hw_ops->irq_enable(core_data, true);
		enable_irq_wake(core_data->irq);
	} else {
		/* enter sleep mode or power off */
		if (core_data->board_data.sleep_enable)
			hw_ops->suspend(core_data);
		else
			goodix_ts_power_off(core_data);
	}
	goodix_ts_release_connects(core_data);

	goodix_set_pinctrl_state(core_data, PINCTRL_MODE_SUSPEND); /* [GOOG] */
	ts_info("Suspend end");
	return 0;
}

static bool check_gesture_mode(struct goodix_ts_core *core_data)
{
	enum raw_scan_mode scan_mode = RAW_SCAN_MODE_AUTO;
	int err = 0;

	err = core_data->hw_ops->get_scan_mode(core_data, &scan_mode);
	if (err != 0) {
		return false;
	}
	return (scan_mode == RAW_SCAN_MODE_LOW_POWER_ACTIVE) ||
		(scan_mode == RAW_SCAN_MODE_LOW_POWER_IDLE);
}

static void monitor_gesture_event(struct work_struct *work)
{
	struct delayed_work *delayed_work = container_of(
		work, struct delayed_work, work);
	struct goodix_ts_core *cd = container_of(delayed_work, struct goodix_ts_core,
		monitor_gesture_work);
	struct goodix_gesture_data* gesture_data = &cd->gesture_data;
	unsigned char event_type = GOODIX_GESTURE_UNKNOWN;
	ktime_t now = ktime_get();
	bool timeout = false;

	mutex_lock(&cd->gesture_data_lock);
	event_type = gesture_data->event_type;
	mutex_unlock(&cd->gesture_data_lock);

	timeout = event_type == GOODIX_GESTURE_FOD_DOWN ?
		now >= cd->gesture_up_timeout : now >= cd->gesture_down_timeout;

	if (event_type != GOODIX_GESTURE_FOD_UP && !timeout) {
		queue_delayed_work(cd->event_wq, &cd->monitor_gesture_work,
			msecs_to_jiffies(5));
		return;
	}

	if (event_type == GOODIX_GESTURE_FOD_UP ||
		event_type == GOODIX_GESTURE_UNKNOWN) {
		if (event_type == GOODIX_GESTURE_UNKNOWN)
			cd->coords_timestamp = now;
		goodix_ts_report_gesture_up(cd);
	}

	/* reset device or power on*/
	if (cd->board_data.sleep_enable)
		cd->hw_ops->reset(cd, goodix_get_normal_reset_delay(cd));
	else
		goodix_ts_power_on(cd);
}

/**
 * goodix_ts_resume - Touchscreen resume function
 * Called by PM/FB/EARLYSUSPEN module to wakeup device
 */
static int goodix_ts_resume(struct goodix_ts_core *core_data)
{
	struct goodix_ts_hw_ops *hw_ops = core_data->hw_ops;
	if (core_data->init_stage < CORE_INIT_STAGE2 ||
		!atomic_read(&core_data->suspended))
		return 0;

	ts_info("Resume start");
	goodix_set_pinctrl_state(core_data, PINCTRL_MODE_ACTIVE); /* [GOOG] */

	atomic_set(&core_data->suspended, 0);
	/* [GOOG]
	 * This will cause a deadlock with wakelock. Since we already disable irq
	 * when touch is suspended, we don't need to disable irq here again.
	 */
	//hw_ops->irq_enable(core_data, false);

	/* [GOOG] */
	if (check_gesture_mode(core_data)) {
		struct goodix_gesture_data *gesture_data = &core_data->gesture_data;

		gesture_data->event_type = GOODIX_GESTURE_UNKNOWN;
		core_data->gesture_down_timeout = ktime_add_ms(ktime_get(), 100);
		core_data->gesture_up_timeout = ktime_add_ms(ktime_get(), 200);
		queue_delayed_work(core_data->event_wq, &core_data->monitor_gesture_work,
			msecs_to_jiffies(5));
	} else {
		if (core_data->gesture_type) {
			disable_irq_wake(core_data->irq);
			hw_ops->reset(core_data, goodix_get_normal_reset_delay(core_data));
		} else {
			/* [GOOG]
			 * Force to reset T-IC as touch resume process instead using brl_resume().
			 */
			/* reset device or power on*/
			if (core_data->board_data.sleep_enable) {
				hw_ops->reset(core_data, goodix_get_normal_reset_delay(core_data));
				//hw_ops->resume(core_data); /* [GOOG] */
			} else {
				goodix_ts_power_on(core_data);
			}
		}
	}

	/* enable irq */
	hw_ops->irq_enable(core_data, true);
	/* open esd */
	goodix_ts_esd_on(core_data);
	ts_info("Resume end");
	return 0;
}

#if IS_ENABLED(CONFIG_FB)
/**
 * goodix_ts_fb_notifier_callback - Framebuffer notifier callback
 * Called by kernel during framebuffer blanck/unblank phrase
 */
static int goodix_ts_fb_notifier_callback(
	struct notifier_block *self, unsigned long event, void *data)
{
	struct goodix_ts_core *core_data =
		container_of(self, struct goodix_ts_core, fb_notifier);
	struct fb_event *fb_event = data;

	if (fb_event && fb_event->data && core_data) {
		if (event == FB_EVENT_BLANK) {
			int *blank = fb_event->data;

			if (*blank == FB_BLANK_UNBLANK)
				goodix_ts_resume(core_data);
			else if (*blank == FB_BLANK_POWERDOWN)
				goodix_ts_suspend(core_data);
		}
	}

	return 0;
}
#endif

#if IS_ENABLED(CONFIG_PM)
#if !IS_ENABLED(CONFIG_FB) && !IS_ENABLED(CONFIG_HAS_EARLYSUSPEND)
/**
 * goodix_ts_pm_suspend - PM suspend function
 * Called by kernel during system suspend phrase
 */
static int goodix_ts_pm_suspend(struct device *dev)
{
	struct goodix_ts_core *core_data = dev_get_drvdata(dev);

	return goodix_ts_suspend(core_data);
}
/**
 * goodix_ts_pm_resume - PM resume function
 * Called by kernel during system wakeup
 */
static int goodix_ts_pm_resume(struct device *dev)
{
	struct goodix_ts_core *core_data = dev_get_drvdata(dev);

	return goodix_ts_resume(core_data);
}
#endif
#endif

static int goodix_ts_stage2_init(struct goodix_ts_core *cd)
{
	int ret;
	int tx = cd->ic_info.parm.drv_num;
	int rx = cd->ic_info.parm.sen_num;
	size_t mutual_size = tx * rx * sizeof(s16);
	size_t self_sensing_size = (tx + rx) * sizeof(s16);
	struct goodix_ic_info_misc *misc = &cd->ic_info.misc;
	size_t touch_frame_size =
		misc->frame_data_addr - misc->touch_data_addr +
		misc->frame_data_head_len + misc->fw_attr_len +
		misc->fw_log_len + sizeof(struct goodix_mutual_data) +
		mutual_size + sizeof(struct goodix_self_sensing_data) +
		self_sensing_size;
#if IS_ENABLED(CONFIG_GOOG_TOUCH_INTERFACE)
	struct gti_optional_configuration *options;
#endif

	/* alloc/config/register input device */
	ret = goodix_ts_input_dev_config(cd);
	if (ret < 0) {
		ts_err("failed set input device");
		return ret;
	}

	if (cd->board_data.pen_enable) {
		ret = goodix_ts_pen_dev_config(cd);
		if (ret < 0) {
			ts_err("failed set pen device");
			goto err_finger;
		}
		mutex_init(&cd->ble_data.lock);
	}

#if IS_ENABLED(CONFIG_FB)
	cd->fb_notifier.notifier_call = goodix_ts_fb_notifier_callback;
	if (fb_register_client(&cd->fb_notifier))
		ts_err("Failed to register fb notifier client:%d", ret);
#endif

#if IS_ENABLED(CONFIG_TOUCHSCREEN_MOTION_FILTER)
	cd->tmf.pdev = cd->pdev;
	cd->tmf.set_continuously_report_enabled =
		set_continuously_report_enabled;
	touch_mf_init(&cd->tmf);
#endif

	/* create sysfs files */
	ret = goodix_ts_sysfs_init(cd);
	if (ret < 0) {
		ts_err("failed set init sysfs");
		goto err_init_sysfs;
	}

	/* create sysfs files for our own APIs */
	cd->apis_data.get_fw_version = get_fw_version;
	cd->apis_data.get_irq_enabled = get_irq_enabled;
	cd->apis_data.set_irq_enabled = set_irq_enabled;
	cd->apis_data.is_scan_mode_supported = is_scan_mode_supported;
	cd->apis_data.ping = ping;
	cd->apis_data.hardware_reset = hardware_reset;
	cd->apis_data.set_scan_mode = set_scan_mode;
	cd->apis_data.set_sensing_enabled = set_sensing_enabled;
#if IS_ENABLED(CONFIG_GOOG_TOUCH_INTERFACE) && IS_ENABLED(CONFIG_GTI_PM)
	cd->apis_data.get_wake_lock_state = get_wake_lock_state;
	cd->apis_data.set_wake_lock_state = set_wake_lock_state;
#endif
#if IS_ENABLED(CONFIG_TOUCHSCREEN_MOTION_FILTER)
	cd->apis_data.tmf = &cd->tmf;
#endif

	ret = touch_apis_init(&cd->pdev->dev, &cd->apis_data);
	if (ret < 0) {
		ts_err("failed set init apis");
		goto err_init_apis;
	}

	cd->event_wq = alloc_workqueue("goodix_wq", WQ_UNBOUND |
		WQ_HIGHPRI | WQ_CPU_INTENSIVE, 1);
	if (!cd->event_wq) {
		ts_err("Cannot create work thread\n");
		ret = -ENOMEM;
		goto err_alloc_workqueue;
	}
	INIT_DELAYED_WORK(&cd->monitor_gesture_work, monitor_gesture_event);

#if IS_ENABLED(CONFIG_GOOG_TOUCH_INTERFACE)
	options = devm_kzalloc(&cd->pdev->dev,
		sizeof(struct gti_optional_configuration), GFP_KERNEL);
	if (options == NULL) {
		ts_err("Failed to alloc gti options\n");
		ret = -ENOMEM;
		goto err_alloc_gti_options;
	}
	options->get_mutual_sensor_data = get_mutual_sensor_data;
	options->get_self_sensor_data = get_self_sensor_data;
	options->set_continuous_report = set_continuous_report;
	options->set_grip_mode = set_grip_mode;
	options->get_grip_mode = get_grip_mode;
	options->set_palm_mode = set_palm_mode;
	options->get_palm_mode = get_palm_mode;
	options->set_screen_protector_mode = set_screen_protector_mode;
	options->get_screen_protector_mode = get_screen_protector_mode;
	options->set_coord_filter_enabled = set_coord_filter_enabled;
	options->get_coord_filter_enabled = get_coord_filter_enabled;
	options->set_heatmap_enabled = set_heatmap_enabled;
	options->get_fw_version = gti_get_fw_version;
	options->set_irq_mode = gti_set_irq_mode;
	options->get_irq_mode = gti_get_irq_mode;
	options->reset = gti_reset;
	options->ping = gti_ping;
	options->calibrate = gti_calibrate;
	options->selftest = gti_selftest;
	options->get_context_driver = gti_get_context_driver;
	options->set_report_rate = gti_set_report_rate;
	options->set_panel_speed_mode = gti_set_panel_speed_mode;
	options->post_irq_thread_fn = goodix_ts_post_threadirq_func;

	cd->gti = goog_touch_interface_probe(
		cd, cd->bus->dev, cd->input_dev, gti_default_handler, options);

#if IS_ENABLED(CONFIG_GTI_PM)
	ret = goog_pm_register_notification(cd->gti, &dev_pm_ops);
	if (ret < 0) {
		ts_info("Failed to register gti pm");
		goto err_init_tpm;
	}
#endif
#endif

	/* create procfs files */
	ret = goodix_ts_procfs_init(cd);
	if (ret < 0) {
		ts_err("failed set init procfs");
		goto err_init_procfs;
	}

	/* esd protector */
	ret = goodix_ts_esd_init(cd);
	if (ret < 0) {
		ts_err("failed set init procfs");
		goto err_init_esd;
	}

#if IS_ENABLED(CONFIG_GOODIX_GESTURE)
	/* gesture init */
	ret = gesture_module_init(cd);
	if (ret < 0) {
		ts_err("failed set init gesture");
		goto err_init_gesture;
	}
#endif

	/* inspect init */
	ret = inspect_module_init(cd);
	if (ret < 0) {
		ts_err("failed set init inspect");
		goto err_init_inspect;
	}

/*
 * [GOOG]
 * Touch frame package will read into `struct goodix_rx_package`.
 * The total read size for SPI is `touch_frame_size` + 8 bytes(SPI prefix header).
 * Therefore, `touch_frame_package` will need to allocate 8 extra bytes for SPI I/O.
 */
	if (cd->bus->sub_ic_type == IC_TYPE_SUB_GT7986) {
		touch_frame_size = misc->touch_data_head_len +
			misc->point_struct_len * GOODIX_MAX_TOUCH + 2;
	}

	cd->touch_frame_size = touch_frame_size;
	cd->touch_frame_package =
		devm_kzalloc(&cd->pdev->dev, touch_frame_size + 8, GFP_KERNEL);
	if (cd->touch_frame_package == NULL) {
		ts_err("failed to alloc touch_frame_package");
		ret = -ENOMEM;
		goto err_setup_irq;
	}
	cd->mutual_data = devm_kzalloc(&cd->pdev->dev, mutual_size, GFP_KERNEL);
	if (cd->mutual_data == NULL) {
		ts_err("failed to alloc mutual_data");
		ret = -ENOMEM;
		goto err_setup_irq;
	}
	cd->mutual_data_manual = devm_kzalloc(&cd->pdev->dev, mutual_size,
		GFP_KERNEL);
	if (cd->mutual_data_manual == NULL) {
		ts_err("failed to alloc mutual_data_manual");
		ret = -ENOMEM;
		goto err_setup_irq;
	}
	cd->self_sensing_data =
		devm_kzalloc(&cd->pdev->dev, self_sensing_size, GFP_KERNEL);
	if (cd->self_sensing_data == NULL) {
		ts_err("failed to alloc self_sensing_data");
		ret = -ENOMEM;
		goto err_setup_irq;
	}
	cd->self_sensing_data_manual =
		devm_kzalloc(&cd->pdev->dev, self_sensing_size, GFP_KERNEL);
	if (cd->self_sensing_data_manual == NULL) {
		ts_err("failed to alloc self_sensing_data_manual");
		ret = -ENOMEM;
		goto err_setup_irq;
	}
/*~[GOOG]*/

	/* request irq line */
	ret = goodix_ts_irq_setup(cd);
	if (ret < 0) {
		ts_info("failed set irq");
		goto err_setup_irq;
	}
	ts_info("success register irq");

	return 0;

err_setup_irq:
	inspect_module_exit(cd);
err_init_inspect:
#if IS_ENABLED(CONFIG_GOODIX_GESTURE)
	gesture_module_exit(cd);
err_init_gesture:
#endif
	goodix_ts_esd_uninit(cd);
err_init_esd:
	goodix_ts_procfs_exit(cd);
err_init_procfs:
#if IS_ENABLED(CONFIG_GOOG_TOUCH_INTERFACE)
#if IS_ENABLED(CONFIG_GTI_PM)
	goog_pm_unregister_notification(cd->gti);
err_init_tpm:
#endif
err_alloc_gti_options:
#endif
	destroy_workqueue(cd->event_wq);
err_alloc_workqueue:
	touch_apis_deinit(&cd->pdev->dev);
err_init_apis:
	goodix_ts_sysfs_exit(cd);
err_init_sysfs:
#if IS_ENABLED(CONFIG_FB)
	fb_unregister_client(&cd->fb_notifier);
#endif
	goodix_ts_pen_dev_remove(cd);
err_finger:
	goodix_ts_input_dev_remove(cd);
	return ret;
}

/* try send the config specified with type */
static int goodix_send_ic_config(struct goodix_ts_core *cd, int type)
{
	u32 config_id;
	struct goodix_ic_config *cfg;

	if (cd->board_data.use_one_binary)
		return 0;

	if (type >= GOODIX_MAX_CONFIG_GROUP) {
		ts_err("unsupported config type %d", type);
		return -EINVAL;
	}

	cfg = cd->ic_configs[type];
	if (!cfg || cfg->len <= 0) {
		ts_info("no valid normal config found");
		return -EINVAL;
	}

	config_id = goodix_get_file_config_id(cfg->data);
	if (cd->ic_info.version.config_id == config_id) {
		ts_info("config id is equal 0x%x, skiped", config_id);
		return 0;
	}

	ts_info("try send config, id=0x%x", config_id);
	return cd->hw_ops->send_config(cd, cfg->data, cfg->len);
}

/**
 * goodix_later_init_thread - init IC fw and config
 * @data: point to goodix_ts_core
 *
 * This function respond for get fw version and try upgrade fw and config.
 * Note: when init encounter error, need release all resource allocated here.
 */
static int goodix_later_init_thread(void *data)
{
	int ret;
	int update_flag = UPDATE_MODE_BLOCK | UPDATE_MODE_SRC_REQUEST;
	struct goodix_ts_core *cd = data;
	struct goodix_ts_hw_ops *hw_ops = cd->hw_ops;

	goodix_wait_for_init_stage2_start(cd); /* [GOOG] */

	/* step 1: read version */
	ret = cd->hw_ops->read_version(cd, &cd->fw_version);
	if (ret < 0) {
		ts_err("failed to get version info, try to upgrade");
		update_flag |= UPDATE_MODE_FORCE;
	}

	/* step 2: read ic info */
	ret = hw_ops->get_ic_info(cd, &cd->ic_info);
	if (ret < 0) {
		ts_err("failed to get ic info, try to upgrade");
		update_flag |= UPDATE_MODE_FORCE;
	}

	/* step 3: get config data from config bin */
	ret = goodix_get_config_proc(cd);
	if (ret < 0)
		ts_info("no valid ic config found");
	else if (ret == 0)
		ts_info("success get valid ic config");
	else
		ts_info("one binary, no need find config");

	/* step 4: init fw struct add try do fw upgrade */
	ret = goodix_fw_update_init(cd);
	if (ret) {
		ts_err("failed init fw update module");
		goto err_out;
	}

	/* step 5: do upgrade */
	ts_info("update flag: 0x%X", update_flag);
	ret = goodix_do_fw_update(cd, update_flag);
	if (ret)
		ts_err("failed do fw update");

	print_ic_info(&cd->ic_info);

	/* the recommend way to update ic config is throuth ISP,
	 * if not we will send config with interactive mode
	 */
	goodix_send_ic_config(cd, CONFIG_TYPE_NORMAL);

	/* init other resources */
	ret = goodix_ts_stage2_init(cd);
	if (ret) {
		ts_err("stage2 init failed");
		goto uninit_fw;
	}
	cd->init_stage = CORE_INIT_STAGE2;

	complete_all(&cd->init_stage2_complete); /* [GOOG] */

	return 0;

uninit_fw:
	goodix_fw_update_uninit(cd);
err_out:
	ts_err("stage2 init failed");
	cd->init_stage = CORE_INIT_FAIL;
	return ret;
}

static int goodix_start_later_init(struct goodix_ts_core *ts_core)
{
	struct task_struct *init_thrd;
	/* create and run update thread */
	init_thrd = kthread_run(
		goodix_later_init_thread, ts_core, "goodix_init_thread");
	if (IS_ERR_OR_NULL(init_thrd)) {
		ts_err("Failed to create update thread:%ld",
			PTR_ERR(init_thrd));
		return -EFAULT;
	}
	return 0;
}

/* goodix fb test */
// static void test_suspend(void)
// {
// 	goodix_ts_suspend(goodix_modules.core_data);
// }

// static void test_resume(void)
// {
// 	goodix_ts_resume(goodix_modules.core_data);
// }

/**
 * goodix_ts_probe - called by kernel when Goodix touch
 *  platform driver is added.
 */
static int goodix_ts_probe(struct platform_device *pdev)
{
	struct goodix_device_resource *dev_res =
			container_of(pdev, struct goodix_device_resource, pdev);
	struct goodix_ts_core *core_data;
	struct goodix_bus_interface *bus_interface;
	int ret;

	ts_info("IN");

	core_data = &dev_res->core_data;
	bus_interface = &dev_res->bus;

	if (IS_ENABLED(CONFIG_OF) && bus_interface->dev->of_node) {
		/* parse devicetree property */
		ret = goodix_parse_dt(
			bus_interface->dev->of_node, &core_data->board_data);
		if (ret) {
			ts_err("failed parse device info form dts, %d", ret);
			return -EINVAL;
		}
	} else {
		ts_err("no valid device tree node found");
		return -ENODEV;
	}

	core_data->hw_ops = goodix_get_hw_ops();
	if (!core_data->hw_ops) {
		ts_err("hw ops is NULL");
		return -EINVAL;
	}
	mutex_init(&core_data->cmd_lock);
	mutex_init(&core_data->gesture_data_lock);

	/* touch core layer is a platform driver */
	core_data->pdev = pdev;
	core_data->bus = bus_interface;
	platform_set_drvdata(pdev, core_data);
	dev_set_drvdata(bus_interface->dev, core_data);

	ret = goodix_pinctrl_init(core_data);
	if (ret) {
		ts_err("failed init pinctrl");
		goto err_out;
	}

	ret = goodix_set_pinctrl_state(core_data, PINCTRL_MODE_ACTIVE);
	if (ret) {
		ts_err("failed set pinctrl state");
		goto err_out;
	}

	/* get GPIO resource */
	ret = goodix_ts_gpio_setup(core_data);
	if (ret) {
		ts_err("failed init gpio");
		goto err_setup_gpio;
	}

	ret = goodix_ts_power_init(core_data);
	if (ret) {
		ts_err("failed init power");
		goto err_setup_gpio;
	}

	ret = goodix_ts_power_on(core_data);
	if (ret) {
		ts_err("failed power on");
		goto err_setup_gpio;
	}

	/* debug node init */
	ret = goodix_tools_init(core_data);
	if (ret) {
		ts_err("failed init tools");
		goto err_init_tools;
	}

	/* goodix fb test */
	// fb_firefly_register(test_suspend, test_resume);

	core_data->init_stage = CORE_INIT_STAGE1;

	/* Try start a thread to get config-bin info */
	ret = goodix_start_later_init(core_data);
	if (ret) {
		ts_err("failed start late init");
		goto err_start_late_init;
	}

	ts_info("%s: goodix_ts_core probe success", __func__);
	return 0;

err_start_late_init:
	goodix_tools_exit(core_data);
err_init_tools:
	goodix_ts_power_off(core_data);
err_setup_gpio:
	goodix_set_pinctrl_state(core_data, PINCTRL_MODE_SUSPEND);
err_out:
	mutex_destroy(&core_data->gesture_data_lock);
	mutex_destroy(&core_data->cmd_lock);
	core_data->init_stage = CORE_INIT_FAIL;
	ts_err("goodix_ts_core failed, ret:%d", ret);
	return ret;
}

static int goodix_ts_remove(struct platform_device *pdev)
{
	struct goodix_ts_core *core_data = platform_get_drvdata(pdev);
	struct goodix_ts_hw_ops *hw_ops = core_data->hw_ops;
	struct goodix_ts_esd *ts_esd = &core_data->ts_esd;


	if (core_data->init_stage >= CORE_INIT_STAGE2) {
/* [GOOG]
 * Follow the reversed order of probe() to release resources.
 */
		hw_ops->irq_enable(core_data, false);

		/* goodix_ts_stage2_init() */
		inspect_module_exit(core_data);
#if IS_ENABLED(CONFIG_GOODIX_GESTURE)
		gesture_module_exit(core_data);
#endif
		if (atomic_read(&ts_esd->esd_on))
			goodix_ts_esd_off(core_data);
		goodix_ts_procfs_exit(core_data);
#if IS_ENABLED(CONFIG_GOOG_TOUCH_INTERFACE)
#if IS_ENABLED(CONFIG_GTI_PM)
		goog_pm_unregister_notification(core_data->gti);
#endif
		goog_touch_interface_remove(core_data->gti);
		destroy_workqueue(core_data->event_wq);
		touch_apis_deinit(&core_data->pdev->dev);
#endif
		goodix_ts_sysfs_exit(core_data);
#if IS_ENABLED(CONFIG_FB)
		fb_unregister_client(&core_data->fb_notifier);
#endif
		goodix_ts_pen_dev_remove(core_data);
		goodix_ts_input_dev_remove(core_data);
		/* goodix_later_init_thread() */
		goodix_fw_update_uninit(core_data);
	}

	/* goodix_ts_probe() */
	goodix_tools_exit(core_data);
	goodix_ts_power_off(core_data);
/*~[GOOG] */
	goodix_set_pinctrl_state(core_data, PINCTRL_MODE_SUSPEND);
	mutex_destroy(&core_data->gesture_data_lock);
	mutex_destroy(&core_data->cmd_lock);

	return 0;
}

#if IS_ENABLED(CONFIG_PM)
static const struct dev_pm_ops dev_pm_ops = {
	.suspend = goodix_ts_pm_suspend,
	.resume = goodix_ts_pm_resume,
};
#endif

static const struct platform_device_id ts_core_ids[] = {
	{ .name = GOODIX_CORE_DRIVER_NAME }, {}
};
MODULE_DEVICE_TABLE(platform, ts_core_ids);

static struct platform_driver goodix_ts_driver = {
	.driver = {
		.name = GOODIX_CORE_DRIVER_NAME,
		.owner = THIS_MODULE,
#if IS_ENABLED(CONFIG_PM)
#if !IS_ENABLED(CONFIG_FB) && !IS_ENABLED(CONFIG_HAS_EARLYSUSPEND) &&          \
	!IS_ENABLED(CONFIG_GTI_PM)
		.pm = &dev_pm_ops,
#endif
#endif
	},
	.probe = goodix_ts_probe,
	.remove = goodix_ts_remove,
	.id_table = ts_core_ids,
};

static int __init goodix_ts_core_init(void)
{
	int ret;

	ts_info("Core layer init:%s", GOODIX_DRIVER_VERSION);
	goodix_device_manager_init();

#ifdef CONFIG_TOUCHSCREEN_GOODIX_BRL_SPI
	ret = goodix_spi_bus_init();
	if (ret) {
		ts_err("failed add spi bus driver");
		return ret;
	}
#endif
#ifdef CONFIG_TOUCHSCREEN_GOODIX_BRL_I2C
	ret = goodix_i2c_bus_init();
	if (ret) {
		ts_err("failed add i2c bus driver");
		return ret;
	}
#endif

	return platform_driver_register(&goodix_ts_driver);
}

static void __exit goodix_ts_core_exit(void)
{
	ts_info("Core layer exit");
	platform_driver_unregister(&goodix_ts_driver);
#ifdef CONFIG_TOUCHSCREEN_GOODIX_BRL_SPI
	goodix_spi_bus_exit();
#endif
#ifdef CONFIG_TOUCHSCREEN_GOODIX_BRL_I2C
	goodix_i2c_bus_exit();
#endif
	goodix_device_manager_exit();
}

late_initcall(goodix_ts_core_init);
module_exit(goodix_ts_core_exit);

MODULE_DESCRIPTION("Goodix Touchscreen Core Module");
MODULE_AUTHOR("Goodix, Inc.");
MODULE_LICENSE("GPL v2");
