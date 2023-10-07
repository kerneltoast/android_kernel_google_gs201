// SPDX-License-Identifier: GPL-2.0
/*
 * Google Touch Interface for Pixel devices.
 *
 * Copyright 2022 Google LLC.
 */

#include <linux/module.h>
#include <linux/input/mt.h>
#include <linux/of.h>
#include <linux/power_supply.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <samsung/exynos_drm_connector.h>

#include "goog_touch_interface.h"
#include "touch_bus_negotiator.h"
#include "../../../gs-google/drivers/soc/google/vh/kernel/systrace.h"

static struct class *gti_class;
static u8 gti_dev_num;

/*-----------------------------------------------------------------------------
 * GTI/common: forward declarations, structures and functions.
 */
static void goog_offload_set_running(struct goog_touch_interface *gti, bool running);
static void goog_lookup_touch_report_rate(struct goog_touch_interface *gti);
static int goog_precheck_heatmap(struct goog_touch_interface *gti);
static void goog_set_display_state(struct goog_touch_interface *gti,
	enum gti_display_state_setting display_state);

/*-----------------------------------------------------------------------------
 * GTI/proc: forward declarations, structures and functions.
 */
static int goog_proc_ms_base_show(struct seq_file *m, void *v);
static int goog_proc_ms_diff_show(struct seq_file *m, void *v);
static int goog_proc_ms_raw_show(struct seq_file *m, void *v);
static int goog_proc_ss_base_show(struct seq_file *m, void *v);
static int goog_proc_ss_diff_show(struct seq_file *m, void *v);
static int goog_proc_ss_raw_show(struct seq_file *m, void *v);
static struct proc_dir_entry *gti_proc_dir_root;
static char *gti_proc_name[GTI_PROC_NUM] = {
	[GTI_PROC_MS_BASE] = "ms_base",
	[GTI_PROC_MS_DIFF] = "ms_diff",
	[GTI_PROC_MS_RAW] = "ms_raw",
	[GTI_PROC_SS_BASE] = "ss_base",
	[GTI_PROC_SS_DIFF] = "ss_diff",
	[GTI_PROC_SS_RAW] = "ss_raw",
};
static int (*gti_proc_show[GTI_PROC_NUM]) (struct seq_file *, void *) = {
	[GTI_PROC_MS_BASE] = goog_proc_ms_base_show,
	[GTI_PROC_MS_DIFF] = goog_proc_ms_diff_show,
	[GTI_PROC_MS_RAW] = goog_proc_ms_raw_show,
	[GTI_PROC_SS_BASE] = goog_proc_ss_base_show,
	[GTI_PROC_SS_DIFF] = goog_proc_ss_diff_show,
	[GTI_PROC_SS_RAW] = goog_proc_ss_raw_show,
};
DEFINE_PROC_SHOW_ATTRIBUTE(goog_proc_ms_base);
DEFINE_PROC_SHOW_ATTRIBUTE(goog_proc_ms_diff);
DEFINE_PROC_SHOW_ATTRIBUTE(goog_proc_ms_raw);
DEFINE_PROC_SHOW_ATTRIBUTE(goog_proc_ss_base);
DEFINE_PROC_SHOW_ATTRIBUTE(goog_proc_ss_diff);
DEFINE_PROC_SHOW_ATTRIBUTE(goog_proc_ss_raw);

static void goog_proc_heatmap_show(struct seq_file *m, void *v)
{
	struct goog_touch_interface *gti = m->private;
	struct gti_sensor_data_cmd *cmd = &gti->cmd.manual_sensor_data_cmd;
	u16 tx = gti->offload.caps.tx_size;
	u16 rx = gti->offload.caps.rx_size;
	int x, y;

	if (cmd->size == 0 || cmd->buffer == NULL) {
		seq_puts(m, "result: N/A!\n");
		GOOG_WARN(gti, "result: N/A!\n");
		return;
	}

	switch (cmd->type) {
	case GTI_SENSOR_DATA_TYPE_MS_BASELINE:
	case GTI_SENSOR_DATA_TYPE_MS_DIFF:
	case GTI_SENSOR_DATA_TYPE_MS_RAW:
		if (cmd->size == TOUCH_OFFLOAD_DATA_SIZE_2D(rx, tx)) {
			seq_puts(m, "result:\n");
			for (y = 0; y < rx; y++) {
				for (x = 0; x < tx; x++)
					seq_printf(m,  "%5d,", ((s16 *)cmd->buffer)[y * tx + x]);
				seq_puts(m, "\n");
			}
		} else {
			seq_printf(m, "error: invalid buffer %p or size %d!\n",
				cmd->buffer, cmd->size);
			GOOG_WARN(gti, "error: invalid buffer %p or size %d!\n",
				cmd->buffer, cmd->size);
		}
		break;

	case GTI_SENSOR_DATA_TYPE_SS_BASELINE:
	case GTI_SENSOR_DATA_TYPE_SS_DIFF:
	case GTI_SENSOR_DATA_TYPE_SS_RAW:
		if (cmd->size == TOUCH_OFFLOAD_DATA_SIZE_1D(rx, tx)) {
			seq_puts(m, "result:\n");
			seq_puts(m, "TX:");
			for (x = 0; x < tx; x++)
				seq_printf(m, "%5d,", ((s16 *)cmd->buffer)[x]);
			seq_puts(m, "\nRX:");
			for (y = 0; y < rx; y++)
				seq_printf(m, "%5d,", ((s16 *)cmd->buffer)[tx + y]);
			seq_puts(m, "\n");
		} else {
			seq_printf(m, "error: invalid buffer %p or size %d!\n",
				cmd->buffer, cmd->size);
			GOOG_WARN(gti, "error: invalid buffer %p or size %d!\n",
				cmd->buffer, cmd->size);
		}
		break;

	default:
		seq_printf(m, "error: invalid type %#x!\n", cmd->type);
		GOOG_ERR(gti, "error: invalid type %#x!\n", cmd->type);
		break;
	}
}

static int goog_proc_heatmap_process(struct seq_file *m, void *v, enum gti_sensor_data_type type)
{
	struct goog_touch_interface *gti = m->private;
	struct gti_sensor_data_cmd *cmd = &gti->cmd.manual_sensor_data_cmd;
	int ret = 0;

	ret = goog_precheck_heatmap(gti);
	if (ret) {
		seq_puts(m, "N/A!\n");
		goto heatmap_process_err;
	}

	switch (type) {
	case GTI_SENSOR_DATA_TYPE_MS_BASELINE:
	case GTI_SENSOR_DATA_TYPE_MS_DIFF:
	case GTI_SENSOR_DATA_TYPE_MS_RAW:
	case GTI_SENSOR_DATA_TYPE_SS_BASELINE:
	case GTI_SENSOR_DATA_TYPE_SS_DIFF:
	case GTI_SENSOR_DATA_TYPE_SS_RAW:
		cmd->type = type;
		break;

	default:
		seq_printf(m, "error: invalid type %#x!\n", type);
		GOOG_ERR(gti, "error: invalid type %#x!\n", type);
		ret = -EINVAL;
		break;
	}

	if (ret)
		goto heatmap_process_err;

	cmd->buffer = NULL;
	cmd->size = 0;
	ret = goog_process_vendor_cmd(gti, GTI_CMD_GET_SENSOR_DATA_MANUAL);
	if (ret) {
		seq_printf(m, "error: %d!\n", ret);
		GOOG_ERR(gti, "error: %d!\n", ret);
	} else {
		GOOG_INFO(gti, "type %#x.\n", type);
	}

heatmap_process_err:
	if (ret) {
		cmd->buffer = NULL;
		cmd->size = 0;
	}
	return ret;
}

static int goog_proc_ms_base_show(struct seq_file *m, void *v)
{
	struct goog_touch_interface *gti = m->private;
	int ret;

	ret = mutex_lock_interruptible(&gti->input_process_lock);
	if (ret) {
		seq_puts(m, "error: has been interrupted!\n");
		GOOG_WARN(gti, "error: has been interrupted!\n");
		return ret;
	}

	ret = goog_proc_heatmap_process(m, v, GTI_SENSOR_DATA_TYPE_MS_BASELINE);
	if (!ret)
		goog_proc_heatmap_show(m, v);
	mutex_unlock(&gti->input_process_lock);

	return ret;
}

static int goog_proc_ms_diff_show(struct seq_file *m, void *v)
{
	struct goog_touch_interface *gti = m->private;
	int ret;

	ret = mutex_lock_interruptible(&gti->input_process_lock);
	if (ret) {
		seq_puts(m, "error: has been interrupted!\n");
		GOOG_WARN(gti, "error: has been interrupted!\n");
		return ret;
	}
	ret = goog_proc_heatmap_process(m, v, GTI_SENSOR_DATA_TYPE_MS_DIFF);
	if (!ret)
		goog_proc_heatmap_show(m, v);
	mutex_unlock(&gti->input_process_lock);

	return ret;
}

static int goog_proc_ms_raw_show(struct seq_file *m, void *v)
{
	struct goog_touch_interface *gti = m->private;
	int ret;

	ret = mutex_lock_interruptible(&gti->input_process_lock);
	if (ret) {
		seq_puts(m, "error: has been interrupted!\n");
		GOOG_WARN(gti, "error: has been interrupted!\n");
		return ret;
	}

	ret = goog_proc_heatmap_process(m, v, GTI_SENSOR_DATA_TYPE_MS_RAW);
	if (!ret)
		goog_proc_heatmap_show(m, v);
	mutex_unlock(&gti->input_process_lock);

	return ret;
}

static int goog_proc_ss_base_show(struct seq_file *m, void *v)
{
	struct goog_touch_interface *gti = m->private;
	int ret;

	ret = mutex_lock_interruptible(&gti->input_process_lock);
	if (ret) {
		seq_puts(m, "error: has been interrupted!\n");
		GOOG_WARN(gti, "error: has been interrupted!\n");
		return ret;
	}

	ret = goog_proc_heatmap_process(m, v, GTI_SENSOR_DATA_TYPE_SS_BASELINE);
	if (!ret)
		goog_proc_heatmap_show(m, v);
	mutex_unlock(&gti->input_process_lock);

	return ret;
}

static int goog_proc_ss_diff_show(struct seq_file *m, void *v)
{
	struct goog_touch_interface *gti = m->private;
	int ret;

	ret = mutex_lock_interruptible(&gti->input_process_lock);
	if (ret) {
		seq_puts(m, "error: has been interrupted!\n");
		GOOG_WARN(gti, "error: has been interrupted!\n");
		return ret;
	}

	ret = goog_proc_heatmap_process(m, v, GTI_SENSOR_DATA_TYPE_SS_DIFF);
	if (!ret)
		goog_proc_heatmap_show(m, v);
	mutex_unlock(&gti->input_process_lock);

	return ret;
}

static int goog_proc_ss_raw_show(struct seq_file *m, void *v)
{
	struct goog_touch_interface *gti = m->private;
	int ret;

	ret = mutex_lock_interruptible(&gti->input_process_lock);
	if (ret) {
		seq_puts(m, "error: has been interrupted!\n");
		GOOG_WARN(gti, "error: has been interrupted!\n");
		return ret;
	}

	ret = goog_proc_heatmap_process(m, v, GTI_SENSOR_DATA_TYPE_SS_RAW);
	if (!ret)
		goog_proc_heatmap_show(m, v);
	mutex_unlock(&gti->input_process_lock);

	return ret;
}

static void goog_init_proc(struct goog_touch_interface *gti)
{
	int type;

	if (!gti_proc_dir_root) {
		gti_proc_dir_root = proc_mkdir(GTI_NAME, NULL);
		if (!gti_proc_dir_root) {
			pr_err("%s: proc_mkdir failed for %s!\n", __func__, GTI_NAME);
			return;
		}
	}

	gti->proc_dir = proc_mkdir_data(dev_name(gti->dev), 0555, gti_proc_dir_root, gti);
	if (!gti->proc_dir) {
		GOOG_ERR(gti, "proc_mkdir_data failed!\n");
		return;
	}

	for (type = GTI_PROC_MS_BASE; type < GTI_PROC_NUM; type++) {
		char *name = gti_proc_name[type];

		if (gti_proc_show[type])
			gti->proc_heatmap[type] = proc_create_single_data(
				name, 0555, gti->proc_dir, gti_proc_show[type], gti);
		if (!gti->proc_heatmap[type])
			GOOG_ERR(gti, "proc_create_single_data failed for %s!\n", name);
	}
}

/*-----------------------------------------------------------------------------
 * GTI/sysfs: forward declarations, structures and functions.
 */
static ssize_t force_active_show(
	struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t force_active_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size);
static ssize_t fw_coord_filter_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t fw_coord_filter_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static ssize_t fw_grip_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t fw_grip_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static ssize_t fw_palm_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t fw_palm_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static ssize_t fw_ver_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t irq_enabled_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t irq_enabled_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static ssize_t mf_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t mf_mode_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static ssize_t offload_enabled_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t offload_enabled_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static ssize_t ping_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t reset_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t reset_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static ssize_t scan_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t scan_mode_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static ssize_t screen_protector_mode_enabled_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static ssize_t screen_protector_mode_enabled_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t self_test_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t sensing_enabled_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t sensing_enabled_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static ssize_t v4l2_enabled_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t v4l2_enabled_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static ssize_t vrr_enabled_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t vrr_enabled_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);

static DEVICE_ATTR_RW(force_active);
static DEVICE_ATTR_RW(fw_coord_filter);
static DEVICE_ATTR_RW(fw_grip);
static DEVICE_ATTR_RW(fw_palm);
static DEVICE_ATTR_RO(fw_ver);
static DEVICE_ATTR_RW(irq_enabled);
static DEVICE_ATTR_RW(mf_mode);
static DEVICE_ATTR_RW(offload_enabled);
static DEVICE_ATTR_RO(ping);
static DEVICE_ATTR_RW(reset);
static DEVICE_ATTR_RW(scan_mode);
static DEVICE_ATTR_RW(screen_protector_mode_enabled);
static DEVICE_ATTR_RO(self_test);
static DEVICE_ATTR_RW(sensing_enabled);
static DEVICE_ATTR_RW(v4l2_enabled);
static DEVICE_ATTR_RW(vrr_enabled);

static struct attribute *goog_attributes[] = {
	&dev_attr_force_active.attr,
	&dev_attr_fw_coord_filter.attr,
	&dev_attr_fw_grip.attr,
	&dev_attr_fw_palm.attr,
	&dev_attr_fw_ver.attr,
	&dev_attr_irq_enabled.attr,
	&dev_attr_mf_mode.attr,
	&dev_attr_offload_enabled.attr,
	&dev_attr_ping.attr,
	&dev_attr_reset.attr,
	&dev_attr_scan_mode.attr,
	&dev_attr_screen_protector_mode_enabled.attr,
	&dev_attr_self_test.attr,
	&dev_attr_sensing_enabled.attr,
	&dev_attr_v4l2_enabled.attr,
	&dev_attr_vrr_enabled.attr,
	NULL,
};

static struct attribute_group goog_attr_group = {
	.attrs = goog_attributes,
};

static ssize_t force_active_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct goog_touch_interface *gti = dev_get_drvdata(dev);
	ssize_t buf_idx = 0;
	bool locked = false;

	if (gti->ignore_force_active) {
		GOOG_WARN(gti, "operation not supported!\n");
		return -EOPNOTSUPP;
	}

	locked = goog_pm_wake_check_locked(gti, GTI_PM_WAKELOCK_TYPE_FORCE_ACTIVE);
	buf_idx += scnprintf(buf, PAGE_SIZE - buf_idx, "result: %s\n",
		locked ? "locked" : "unlocked");
	GOOG_INFO(gti, "%s", buf);

	return buf_idx;
}

static ssize_t force_active_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct goog_touch_interface *gti = dev_get_drvdata(dev);
	u32 locked = 0;
	int ret = 0;

	if (buf == NULL || size < 0) {
		GOOG_INFO(gti, "error: invalid input!\n");
		return -EINVAL;
	}

	if (kstrtou32(buf, 10, &locked)) {
		GOOG_INFO(gti, "error: invalid input!\n");
		return -EINVAL;
	}

	if (locked > 1) {
		GOOG_INFO(gti, "error: invalid input!\n");
		return -EINVAL;
	}

	if (locked) {
		gti_debug_hc_dump(gti);
		gti_debug_input_dump(gti);
		if (gti->ignore_force_active)
			GOOG_WARN(gti, "operation not supported!\n");
		else
			ret = goog_pm_wake_lock(gti, GTI_PM_WAKELOCK_TYPE_FORCE_ACTIVE, false);
	} else {
		if (gti->ignore_force_active)
			GOOG_WARN(gti, "operation not supported!\n");
		else
			ret = goog_pm_wake_unlock(gti, GTI_PM_WAKELOCK_TYPE_FORCE_ACTIVE);
	}

	if (ret < 0) {
		GOOG_INFO(gti, "error: %d!\n", ret);
		return ret;
	}
	return size;
}

static ssize_t fw_coord_filter_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	ssize_t buf_idx = 0;
	struct goog_touch_interface *gti = dev_get_drvdata(dev);
	struct gti_coord_filter_cmd *cmd = &gti->cmd.coord_filter_cmd;

	if (!gti->coord_filter_enabled) {
		buf_idx += scnprintf(buf + buf_idx, PAGE_SIZE - buf_idx,
			"error: not supported!\n");
		GOOG_INFO(gti, "%s", buf);
		return buf_idx;
	}

	cmd->setting = GTI_COORD_FILTER_DISABLE;
	ret = goog_process_vendor_cmd(gti, GTI_CMD_GET_COORD_FILTER_ENABLED);
	if (ret == -EOPNOTSUPP) {
		buf_idx += scnprintf(buf + buf_idx, PAGE_SIZE - buf_idx,
			"error: not supported!\n");
	} else if (ret) {
		buf_idx += scnprintf(buf + buf_idx, PAGE_SIZE - buf_idx,
			"error: %d!\n", ret);
	} else {
		buf_idx += scnprintf(buf + buf_idx, PAGE_SIZE - buf_idx,
			"result: %u\n", cmd->setting | (gti->ignore_coord_filter_update << 1));
	}
	GOOG_INFO(gti, "%s", buf);

	return buf_idx;
}

static ssize_t fw_coord_filter_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int ret = 0;
	struct goog_touch_interface *gti = dev_get_drvdata(dev);
	int fw_coord_filter;

	if (kstrtou32(buf, 10, &fw_coord_filter)) {
		GOOG_INFO(gti, "error: invalid input!\n");
		return -EINVAL;
	}

	if (!gti->coord_filter_enabled) {
		GOOG_INFO(gti, "error: not supported!\n");
		return -EOPNOTSUPP;
	}

	gti->fw_coord_filter_enabled = fw_coord_filter & 0x01;
	gti->ignore_coord_filter_update = (fw_coord_filter >> 1) & 0x01;
	gti->cmd.coord_filter_cmd.setting = gti->fw_coord_filter_enabled ?
			GTI_COORD_FILTER_ENABLE : GTI_COORD_FILTER_DISABLE;
	ret = goog_process_vendor_cmd(gti, GTI_CMD_SET_COORD_FILTER_ENABLED);
	if (ret == -EOPNOTSUPP)
		GOOG_INFO(gti, "error: not supported!\n");
	else if (ret)
		GOOG_INFO(gti, "error: %d!\n", ret);
	else
		GOOG_INFO(gti, "fw_coord_filter= %u\n", fw_coord_filter);

	return size;
}

static ssize_t fw_grip_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	ssize_t buf_idx = 0;
	struct goog_touch_interface *gti = dev_get_drvdata(dev);
	struct gti_grip_cmd *cmd = &gti->cmd.grip_cmd;

	cmd->setting = GTI_GRIP_DISABLE;
	ret = goog_process_vendor_cmd(gti, GTI_CMD_GET_GRIP_MODE);
	if (ret == -EOPNOTSUPP) {
		buf_idx += scnprintf(buf + buf_idx, PAGE_SIZE - buf_idx,
			"error: not supported!\n");
	} else if (ret) {
		buf_idx += scnprintf(buf + buf_idx, PAGE_SIZE - buf_idx,
			"error: %d!\n", ret);
	} else {
		buf_idx += scnprintf(buf + buf_idx, PAGE_SIZE - buf_idx,
			"result: %u\n", cmd->setting | (gti->ignore_grip_update << 1));
	}
	GOOG_INFO(gti, "%s", buf);

	return buf_idx;
}

static ssize_t fw_grip_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int ret = 0;
	struct goog_touch_interface *gti = dev_get_drvdata(dev);
	int fw_grip_mode = 0;
	bool enabled = false;

	if (kstrtou32(buf, 10, &fw_grip_mode)) {
		GOOG_INFO(gti, "error: invalid input!\n");
		return size;
	}

	enabled = fw_grip_mode & 0x01;
	gti->ignore_grip_update = (fw_grip_mode >> 1) & 0x01;
	gti->cmd.grip_cmd.setting = enabled ? GTI_GRIP_ENABLE : GTI_GRIP_DISABLE;
	ret = goog_process_vendor_cmd(gti, GTI_CMD_SET_GRIP_MODE);
	if (ret == -EOPNOTSUPP)
		GOOG_INFO(gti, "error: not supported!\n");
	else if (ret)
		GOOG_INFO(gti, "error: %d!\n", ret);
	else
		GOOG_INFO(gti, "fw_grip_mode: %u\n", fw_grip_mode);

	return size;
}

static ssize_t fw_palm_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	ssize_t buf_idx = 0;
	struct goog_touch_interface *gti = dev_get_drvdata(dev);
	struct gti_palm_cmd *cmd = &gti->cmd.palm_cmd;

	cmd->setting = GTI_PALM_DISABLE;
	ret = goog_process_vendor_cmd(gti, GTI_CMD_GET_PALM_MODE);
	if (ret == -EOPNOTSUPP) {
		buf_idx += scnprintf(buf + buf_idx, PAGE_SIZE - buf_idx,
			"error: not supported!\n");
	} else if (ret) {
		buf_idx += scnprintf(buf + buf_idx, PAGE_SIZE - buf_idx,
			"error: %d!\n", ret);
	} else {
		buf_idx += scnprintf(buf + buf_idx, PAGE_SIZE - buf_idx,
			"result: %u\n", cmd->setting | (gti->ignore_palm_update << 1));
	}
	GOOG_INFO(gti, "%s", buf);

	return buf_idx;
}

static ssize_t fw_palm_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int ret = 0;
	struct goog_touch_interface *gti = dev_get_drvdata(dev);
	int fw_palm_mode;
	bool enabled;

	if (kstrtou32(buf, 10, &fw_palm_mode)) {
		GOOG_INFO(gti, "error: invalid input!\n");
		return -EINVAL;
	}

	enabled = fw_palm_mode & 0x01;
	gti->ignore_palm_update = (fw_palm_mode >> 1) & 0x01;
	gti->cmd.palm_cmd.setting = enabled ? GTI_PALM_ENABLE : GTI_PALM_DISABLE;
	ret = goog_process_vendor_cmd(gti, GTI_CMD_SET_PALM_MODE);
	if (ret == -EOPNOTSUPP)
		GOOG_INFO(gti, "error: not supported!\n");
	else if (ret)
		GOOG_INFO(gti, "error: %d!\n", ret);
	else
		GOOG_INFO(gti, "fw_palm_mode= %u\n", fw_palm_mode);

	return size;
}

static ssize_t fw_ver_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret;
	ssize_t buf_idx = 0;
	struct goog_touch_interface *gti = dev_get_drvdata(dev);

	memset(gti->cmd.fw_version_cmd.buffer, 0, sizeof(gti->cmd.fw_version_cmd.buffer));
	ret = goog_process_vendor_cmd(gti, GTI_CMD_GET_FW_VERSION);
	if (ret == -EOPNOTSUPP) {
		buf_idx += scnprintf(buf + buf_idx, PAGE_SIZE - buf_idx,
			"error: not supported!\n");
	} else if (ret) {
		buf_idx += scnprintf(buf + buf_idx, PAGE_SIZE - buf_idx,
			"error: %d!\n", ret);
	} else {
		buf_idx += scnprintf(buf + buf_idx, PAGE_SIZE - buf_idx,
			"result: %s\n", gti->cmd.fw_version_cmd.buffer);
	}
	GOOG_INFO(gti, "%s", buf);

	return buf_idx;
}

static ssize_t irq_enabled_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret;
	ssize_t buf_idx = 0;
	struct goog_touch_interface *gti = dev_get_drvdata(dev);

	gti->cmd.irq_cmd.setting = GTI_IRQ_MODE_NA;
	ret = goog_process_vendor_cmd(gti, GTI_CMD_GET_IRQ_MODE);
	if (ret == -EOPNOTSUPP) {
		buf_idx += scnprintf(buf + buf_idx, PAGE_SIZE - buf_idx,
			"error: not supported!\n");
	} else if (ret) {
		buf_idx += scnprintf(buf + buf_idx, PAGE_SIZE - buf_idx,
			"error: %d!\n", ret);
	} else {
		buf_idx += scnprintf(buf + buf_idx, PAGE_SIZE - buf_idx,
			"result: %u\n", gti->cmd.irq_cmd.setting);
	}
	GOOG_INFO(gti, "%s", buf);

	return buf_idx;
}

static ssize_t irq_enabled_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int ret;
	bool enabled;
	struct goog_touch_interface *gti = dev_get_drvdata(dev);

	if (kstrtobool(buf, &enabled)) {
		GOOG_ERR(gti, "error: invalid input!\n");
		return size;
	}

	gti->cmd.irq_cmd.setting = enabled;
	ret = goog_process_vendor_cmd(gti, GTI_CMD_SET_IRQ_MODE);
	if (ret == -EOPNOTSUPP)
		GOOG_INFO(gti, "error: not supported!\n");
	else if (ret)
		GOOG_INFO(gti, "error: %d!\n", ret);
	else
		GOOG_INFO(gti, "irq_enabled= %u\n", gti->cmd.irq_cmd.setting);

	return size;
}

static ssize_t mf_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t buf_idx = 0;
	struct goog_touch_interface *gti = dev_get_drvdata(dev);

	buf_idx += scnprintf(buf + buf_idx, PAGE_SIZE - buf_idx,
		"result: %u\n", gti->mf_mode);
	GOOG_INFO(gti, "%s", buf);

	return buf_idx;
}

static ssize_t mf_mode_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct goog_touch_interface *gti = dev_get_drvdata(dev);
	enum gti_mf_mode mode = 0;

	if (buf == NULL || size < 0) {
		GOOG_INFO(gti, "error: invalid input!\n");
		return size;
	}

	if (kstrtou32(buf, 10, &mode)) {
		GOOG_INFO(gti, "error: invalid input!\n");
		return size;
	}

	if (mode < GTI_MF_MODE_UNFILTER ||
		mode > GTI_MF_MODE_AUTO_REPORT) {
		GOOG_INFO(gti, "error: invalid input!\n");
		return size;
	}

	gti->mf_mode = mode;
	GOOG_INFO(gti, "mf_mode= %u\n", gti->mf_mode);

	return size;
}

static ssize_t offload_enabled_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t buf_idx = 0;
	struct goog_touch_interface *gti = dev_get_drvdata(dev);

	buf_idx += scnprintf(buf + buf_idx, PAGE_SIZE - buf_idx,
		"result: %d\n", gti->offload_enabled);
	GOOG_INFO(gti, "%s", buf);

	return buf_idx;
}

static ssize_t offload_enabled_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct goog_touch_interface *gti = dev_get_drvdata(dev);

	if (kstrtobool(buf, &gti->offload_enabled)) {
		GOOG_INFO(gti, "error: invalid input!\n");
	} else {
		GOOG_INFO(gti, "offload_enabled= %d\n", gti->offload_enabled);
		/* Force to turn off offload by request. */
		if (!gti->offload_enabled)
			goog_offload_set_running(gti, false);
	}

	return size;
}

static ssize_t ping_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret;
	ssize_t buf_idx = 0;
	struct goog_touch_interface *gti = dev_get_drvdata(dev);

	gti->cmd.ping_cmd.setting = GTI_PING_ENABLE;
	ret = goog_process_vendor_cmd(gti, GTI_CMD_PING);
	if (ret == -EOPNOTSUPP) {
		buf_idx += scnprintf(buf + buf_idx, PAGE_SIZE - buf_idx,
			"error: not supported!\n");
		gti->cmd.ping_cmd.setting = GTI_PING_NA;
	} else if (ret) {
		buf_idx += scnprintf(buf + buf_idx, PAGE_SIZE - buf_idx,
			"error: %d!\n", ret);
		gti->cmd.ping_cmd.setting = GTI_PING_NA;
	} else {
		buf_idx += scnprintf(buf + buf_idx, PAGE_SIZE - buf_idx,
			"result: success.\n");
	}
	GOOG_INFO(gti, "%s", buf);

	return buf_idx;
}

static ssize_t reset_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t buf_idx = 0;
	struct goog_touch_interface *gti = dev_get_drvdata(dev);

	if (gti->cmd.reset_cmd.setting == GTI_RESET_MODE_NOP ||
		gti->cmd.reset_cmd.setting == GTI_RESET_MODE_NA) {
		buf_idx += scnprintf(buf + buf_idx, PAGE_SIZE - buf_idx,
			"error: %d!\n", gti->cmd.reset_cmd.setting);
	} else {
		buf_idx += scnprintf(buf + buf_idx, PAGE_SIZE - buf_idx,
			"result: success.\n");
	}
	GOOG_INFO(gti, "%s", buf);

	return buf_idx;
}

static ssize_t reset_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int ret;
	struct goog_touch_interface *gti = dev_get_drvdata(dev);
	enum gti_reset_mode mode = 0;

	if (buf == NULL || size < 0) {
		GOOG_INFO(gti, "error: invalid input!\n");
		return -EINVAL;
	}

	if (kstrtou32(buf, 10, &mode)) {
		GOOG_INFO(gti, "error: invalid input!\n");
		return -EINVAL;
	}

	if (mode <= GTI_RESET_MODE_NOP ||
		mode > GTI_RESET_MODE_AUTO) {
		GOOG_INFO(gti, "error: invalid input!\n");
		return -EINVAL;
	}

	gti->cmd.reset_cmd.setting = mode;
	ret = goog_process_vendor_cmd(gti, GTI_CMD_RESET);
	if (ret == -EOPNOTSUPP) {
		GOOG_INFO(gti, "error: not supported!\n");
		gti->cmd.reset_cmd.setting = GTI_RESET_MODE_NA;
	} else if (ret) {
		GOOG_INFO(gti, "error: %d!\n", ret);
		gti->cmd.reset_cmd.setting = GTI_RESET_MODE_NA;
	} else {
		GOOG_INFO(gti, "reset= 0x%x\n", mode);
	}

	return size;
}

static ssize_t scan_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret;
	ssize_t buf_idx = 0;
	struct goog_touch_interface *gti = dev_get_drvdata(dev);

	gti->cmd.scan_cmd.setting = GTI_SCAN_MODE_NA;
	ret = goog_process_vendor_cmd(gti, GTI_CMD_GET_SCAN_MODE);
	if (ret == -EOPNOTSUPP) {
		buf_idx += scnprintf(buf + buf_idx, PAGE_SIZE - buf_idx,
			"error: not supported!\n");
	} else if (ret) {
		buf_idx += scnprintf(buf + buf_idx, PAGE_SIZE - buf_idx,
			"error: %d!\n", ret);
	} else {
		buf_idx += scnprintf(buf + buf_idx, PAGE_SIZE - buf_idx,
			"result: %u\n", gti->cmd.scan_cmd.setting);
	}
	GOOG_INFO(gti, "%s", buf);

	return buf_idx;
}

static ssize_t scan_mode_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int ret;
	struct goog_touch_interface *gti = dev_get_drvdata(dev);
	enum gti_scan_mode mode = 0;

	if (buf == NULL || size < 0) {
		GOOG_INFO(gti, "error: invalid input!\n");
		return size;
	}

	if (kstrtou32(buf, 10, &mode)) {
		GOOG_ERR(gti, "error: invalid input!\n");
		return size;
	}

	if (mode < GTI_SCAN_MODE_AUTO ||
		mode > GTI_SCAN_MODE_LP_IDLE) {
		GOOG_INFO(gti, "error: invalid input!\n");
		return size;
	}

	gti->cmd.scan_cmd.setting = mode;
	ret = goog_process_vendor_cmd(gti, GTI_CMD_SET_SCAN_MODE);
	if (ret == -EOPNOTSUPP)
		GOOG_ERR(gti, "error: not supported!\n");
	else if (ret)
		GOOG_ERR(gti, "error: %d!\n", ret);
	else
		GOOG_INFO(gti, "scan_mode= %u\n", mode);

	return size;
}

static ssize_t screen_protector_mode_enabled_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int ret = 0;
	struct goog_touch_interface *gti = dev_get_drvdata(dev);
	struct gti_screen_protector_mode_cmd *cmd = &gti->cmd.screen_protector_mode_cmd;
	bool enabled = false;

	if (kstrtobool(buf, &enabled)) {
		GOOG_ERR(gti, "invalid input!\n");
		return -EINVAL;
	}

	cmd->setting = enabled ? GTI_SCREEN_PROTECTOR_MODE_ENABLE : GTI_SCREEN_PROTECTOR_MODE_DISABLE;
	ret = goog_process_vendor_cmd(gti, GTI_CMD_SET_SCREEN_PROTECTOR_MODE);
	if (ret == -EOPNOTSUPP)
		GOOG_ERR(gti, "error: not supported!\n");
	else if (ret)
		GOOG_ERR(gti, "error: %d!\n", ret);
	else
		GOOG_INFO(gti, "enabled= %u\n", enabled);
	gti->screen_protector_mode_setting = enabled ?
			GTI_SCREEN_PROTECTOR_MODE_ENABLE : GTI_SCREEN_PROTECTOR_MODE_DISABLE;
	return size;
}

static ssize_t screen_protector_mode_enabled_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	ssize_t buf_idx = 0;
	struct goog_touch_interface *gti = dev_get_drvdata(dev);
	struct gti_screen_protector_mode_cmd *cmd = &gti->cmd.screen_protector_mode_cmd;

	cmd->setting = GTI_SCREEN_PROTECTOR_MODE_NA;
	ret = goog_process_vendor_cmd(gti, GTI_CMD_GET_SCREEN_PROTECTOR_MODE);
	if (ret == 0) {
		buf_idx += scnprintf(buf, PAGE_SIZE - buf_idx, "result: %d\n",
				cmd->setting == GTI_SCREEN_PROTECTOR_MODE_ENABLE);
	} else {
		buf_idx += scnprintf(buf, PAGE_SIZE - buf_idx, "error: %d\n", ret);
	}
	GOOG_INFO(gti, "%s", buf);
	return buf_idx;
}

static ssize_t self_test_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret;
	ssize_t buf_idx = 0;
	struct goog_touch_interface *gti = dev_get_drvdata(dev);

	gti->cmd.selftest_cmd.result = GTI_SELFTEST_RESULT_NA;
	memset(gti->cmd.selftest_cmd.buffer, 0, sizeof(gti->cmd.selftest_cmd.buffer));
	ret = goog_process_vendor_cmd(gti, GTI_CMD_SELFTEST);
	if (ret == -EOPNOTSUPP) {
		buf_idx += scnprintf(buf + buf_idx, PAGE_SIZE - buf_idx,
			"error: not supported!\n");
	} else if (ret) {
		buf_idx += scnprintf(buf + buf_idx, PAGE_SIZE - buf_idx,
			"error: %d!\n", ret);
	} else {
		if (gti->cmd.selftest_cmd.result == GTI_SELFTEST_RESULT_DONE) {
			buf_idx += scnprintf(buf + buf_idx, PAGE_SIZE - buf_idx,
				"result: %s\n", gti->cmd.selftest_cmd.buffer);
		} else if (gti->cmd.selftest_cmd.result ==
				GTI_SELFTEST_RESULT_SHELL_CMDS_REDIRECT) {
			buf_idx += scnprintf(buf + buf_idx, PAGE_SIZE - buf_idx,
				"redirect: %s\n", gti->cmd.selftest_cmd.buffer);
		} else {
			buf_idx += scnprintf(buf + buf_idx, PAGE_SIZE - buf_idx, "error: N/A!\n");
		}
	}
	GOOG_INFO(gti, "%s", buf);

	return buf_idx;
}

static ssize_t sensing_enabled_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret;
	ssize_t buf_idx = 0;
	struct goog_touch_interface *gti = dev_get_drvdata(dev);

	gti->cmd.sensing_cmd.setting = GTI_SENSING_MODE_NA;
	ret = goog_process_vendor_cmd(gti, GTI_CMD_GET_SENSING_MODE);
	if (ret == -EOPNOTSUPP) {
		buf_idx += scnprintf(buf + buf_idx, PAGE_SIZE - buf_idx,
			"error: not supported!\n");
	} else if (ret) {
		buf_idx += scnprintf(buf + buf_idx, PAGE_SIZE - buf_idx,
			"error: %d!\n", ret);
	} else {
		buf_idx += scnprintf(buf + buf_idx, PAGE_SIZE - buf_idx,
			"result: %u\n", gti->cmd.sensing_cmd.setting);
	}
	GOOG_INFO(gti, "%s", buf);

	return buf_idx;
}

static ssize_t sensing_enabled_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int ret;
	bool enabled;
	struct goog_touch_interface *gti = dev_get_drvdata(dev);

	if (kstrtobool(buf, &enabled)) {
		GOOG_INFO(gti, "error: invalid input!\n");
		return size;
	}

	gti->cmd.sensing_cmd.setting = enabled;
	ret = goog_process_vendor_cmd(gti, GTI_CMD_SET_SENSING_MODE);
	if (ret == -EOPNOTSUPP)
		GOOG_INFO(gti, "error: not supported!\n");
	else if (ret)
		GOOG_INFO(gti, "error: %d!\n", ret);
	else
		GOOG_INFO(gti, "sensing_enabled= %u\n", gti->cmd.sensing_cmd.setting);

	return size;
}

static ssize_t v4l2_enabled_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t buf_idx = 0;
	struct goog_touch_interface *gti = dev_get_drvdata(dev);

	buf_idx += scnprintf(buf + buf_idx, PAGE_SIZE - buf_idx,
		"result: %d\n", gti->v4l2_enabled);
	GOOG_INFO(gti, "%s", buf);

	return buf_idx;
}

static ssize_t v4l2_enabled_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct goog_touch_interface *gti = dev_get_drvdata(dev);

	if (kstrtobool(buf, &gti->v4l2_enabled))
		GOOG_INFO(gti, "error: invalid input!\n");
	else
		GOOG_INFO(gti, "v4l2_enabled= %d\n", gti->v4l2_enabled);

	return size;
}

static ssize_t vrr_enabled_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t buf_idx = 0;
	struct goog_touch_interface *gti = dev_get_drvdata(dev);

	buf_idx += scnprintf(buf + buf_idx, PAGE_SIZE,
		"result: %d\n", gti->vrr_enabled);
	GOOG_INFO(gti, "%s", buf);

	return buf_idx;
}

static ssize_t vrr_enabled_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct goog_touch_interface *gti = dev_get_drvdata(dev);

	if (kstrtobool(buf, &gti->vrr_enabled)) {
		GOOG_INFO(gti, "error: invalid input!\n");
	} else if (gti->report_rate_table_size == 0) {
		GOOG_INFO(gti, "error: No valid report rate table!\n");
	} else {
		GOOG_INFO(gti, "vrr_enabled= %d\n", gti->vrr_enabled);
		if (gti->vrr_enabled)
			goog_lookup_touch_report_rate(gti);
	}

	return size;
}

/*-----------------------------------------------------------------------------
 * Debug: functions.
 */
#ifdef GTI_DEBUG_KFIFO_LEN
inline void gti_debug_hc_push(struct goog_touch_interface *gti)
{
	/*
	 * Use kfifo as circular buffer by skipping one element
	 * when fifo is full.
	 */
	if (kfifo_is_full(&gti->debug_fifo_hc))
		kfifo_skip(&gti->debug_fifo_hc);
	kfifo_in(&gti->debug_fifo_hc, &gti->debug_hc, 1);
}

inline int gti_debug_hc_pop(struct goog_touch_interface *gti,
	struct gti_debug_health_check *fifo, unsigned int len)
{
	if (len > GTI_DEBUG_KFIFO_LEN) {
		GOOG_ERR(gti, "invalid fifo pop len(%d)!\n", len);
		return -EINVAL;
	}
	/*
	 * Keep data without pop-out to support different timing
	 * print-out by each caller.
	 */
	return kfifo_out_peek(&gti->debug_fifo_hc, fifo, len) == len ? 0 : -EFAULT;
}

inline void gti_debug_hc_update(struct goog_touch_interface *gti, bool from_top_half)
{
	if (from_top_half) {
		gti->debug_hc.irq_time = ktime_get();
		gti->debug_hc.irq_index = gti->irq_index;
	} else {
		gti->debug_hc.input_index = gti->input_index;
		gti->debug_hc.slot_bit_active = gti->slot_bit_active;
		gti_debug_hc_push(gti);
	}
}

void gti_debug_hc_dump(struct goog_touch_interface *gti)
{
	int ret;
	u64 i, count;
	s64 delta;
	s64 sec_delta;
	u32 ms_delta;
	ktime_t current_time = ktime_get();
	struct gti_debug_health_check last_fifo[GTI_DEBUG_KFIFO_LEN] = { 0 };

	count = min_t(u64, gti->irq_index, ARRAY_SIZE(last_fifo));
	ret = gti_debug_hc_pop(gti, last_fifo, count);
	if (ret) {
		GOOG_ERR(gti, "Failed to peek debug hc, err: %d\n", ret);
		return;
	}
	for (i = 0 ; i < count ; i++) {
		sec_delta = -1;
		ms_delta = 0;
		/*
		 * Calculate the delta time between irq triggered and current time.
		 */
		delta = ktime_ms_delta(current_time, last_fifo[i].irq_time);
		if (delta > 0)
			sec_delta = div_u64_rem(delta, MSEC_PER_SEC, &ms_delta);
		GOOG_LOG(gti, "dump-int: #%llu(%lld.%u): C#%llu(0x%lx).\n",
			last_fifo[i].irq_index, sec_delta, ms_delta,
			last_fifo[i].input_index, last_fifo[i].slot_bit_active);
	}
}

inline void gti_debug_input_push(struct goog_touch_interface *gti, int slot)
{
	struct gti_debug_input fifo;

	if (slot < 0 || slot >= MAX_SLOTS) {
		GOOG_ERR(gti, "Invalid slot: %d\n", slot);
		return;
	}

	/*
	 * Use kfifo as circular buffer by skipping one element
	 * when fifo is full.
	 */
	if (kfifo_is_full(&gti->debug_fifo_input))
		kfifo_skip(&gti->debug_fifo_input);

	memcpy(&fifo, &gti->debug_input[slot], sizeof(struct gti_debug_input));
	kfifo_in(&gti->debug_fifo_input, &fifo, 1);
}

inline int gti_debug_input_pop(struct goog_touch_interface *gti,
	struct gti_debug_input *fifo, unsigned int len)
{
	if (len > GTI_DEBUG_KFIFO_LEN) {
		GOOG_ERR(gti, "invalid fifo pop len(%d)!\n", len);
		return -EINVAL;
	}

	/*
	 * Keep coords without pop-out to support different timing
	 * print-out by each caller.
	 */
	return kfifo_out_peek(&gti->debug_fifo_input, fifo, len) == len ? 0 : -EFAULT;
}

inline void gti_debug_input_update(struct goog_touch_interface *gti)
{
	int slot;
	u64 irq_index = gti->irq_index;
	ktime_t time = ktime_get();

	for_each_set_bit(slot, &gti->slot_bit_changed, MAX_SLOTS) {
		if (test_bit(slot, &gti->slot_bit_active)) {
			gti->debug_input[slot].pressed.time = time;
			gti->debug_input[slot].pressed.irq_index = irq_index;
			memcpy(&gti->debug_input[slot].pressed.coord,
				&gti->offload.coords[slot],
				sizeof(struct TouchOffloadCoord));
		} else {
			gti->released_index++;
			gti->debug_input[slot].released.time = time;
			gti->debug_input[slot].released.irq_index = irq_index;
			memcpy(&gti->debug_input[slot].released.coord,
				&gti->offload.coords[slot],
				sizeof(struct TouchOffloadCoord));
			gti_debug_input_push(gti, slot);
		}
	}
	gti->slot_bit_changed = 0;
}

void gti_debug_input_dump(struct goog_touch_interface *gti)
{
	int slot, ret;
	u64 i, count;
	s64 delta;
	s64 sec_delta_down;
	u32 ms_delta_down;
	s64 sec_delta_duration;
	u32 ms_delta_duration;
	s32 px_delta_x, px_delta_y;
	ktime_t current_time = ktime_get();
	struct gti_debug_input last_fifo[GTI_DEBUG_KFIFO_LEN] = { 0 };

	count = min_t(u64, gti->released_index, ARRAY_SIZE(last_fifo));
	ret = gti_debug_input_pop(gti, last_fifo, count);
	if (ret) {
		GOOG_ERR(gti, "Failed to peek debug input, err: %d\n", ret);
		return;
	}
	for (i = 0 ; i < count ; i++) {
		if (last_fifo[i].slot < 0 ||
			last_fifo[i].slot >= MAX_SLOTS) {
			GOOG_INFO(gti, "dump: #%d: invalid slot #!\n", last_fifo[i].slot);
			continue;
		}
		sec_delta_down = -1;
		ms_delta_down = 0;
		/*
		 * Calculate the delta time of finger down from current time.
		 */
		delta = ktime_ms_delta(current_time, last_fifo[i].pressed.time);
		if (delta > 0)
			sec_delta_down = div_u64_rem(delta, MSEC_PER_SEC, &ms_delta_down);

		/*
		 * Calculate the delta time of finger duration from finger up to down.
		 */
		sec_delta_duration = -1;
		ms_delta_duration = 0;
		px_delta_x = 0;
		px_delta_y = 0;
		if (ktime_compare(last_fifo[i].released.time,
			last_fifo[i].pressed.time) > 0) {
			delta = ktime_ms_delta(last_fifo[i].released.time,
					last_fifo[i].pressed.time);
			if (delta > 0) {
				sec_delta_duration = div_u64_rem(delta, MSEC_PER_SEC,
									&ms_delta_duration);
				px_delta_x = last_fifo[i].released.coord.x -
					last_fifo[i].pressed.coord.x;
				px_delta_y = last_fifo[i].released.coord.y -
					last_fifo[i].pressed.coord.y;
			}
		}

		GOOG_LOG(gti, "dump: #%d: %lld.%u(%lld.%u) D(%d, %d) I(%llu, %llu).\n",
			last_fifo[i].slot,
			sec_delta_down, ms_delta_down,
			sec_delta_duration, ms_delta_duration,
			px_delta_x, px_delta_y,
			last_fifo[i].pressed.irq_index, last_fifo[i].released.irq_index);
		GOOG_DBG(gti, "dump-dbg: #%d: P(%u, %u) -> R(%u, %u).\n\n",
			last_fifo[i].slot,
			last_fifo[i].pressed.coord.x, last_fifo[i].pressed.coord.y,
			last_fifo[i].released.coord.x, last_fifo[i].released.coord.y);
	}
	/* Extra check for unexpected case. */
	for_each_set_bit(slot, &gti->slot_bit_active, MAX_SLOTS) {
		GOOG_INFO(gti, "slot #%d is active!\n", slot);
	}
}
#endif /* GTI_DEBUG_KFIFO_LEN */

/*-----------------------------------------------------------------------------
 * DRM: functions and structures.
 */
static void panel_bridge_enable(struct drm_bridge *bridge)
{
	struct goog_touch_interface *gti =
		container_of(bridge, struct goog_touch_interface, panel_bridge);

	if (gti->panel_is_lp_mode) {
		GOOG_DBG(gti, "skip screen-on because of panel_is_lp_mode enabled!\n");
		return;
	}

	goog_set_display_state(gti, GTI_DISPLAY_STATE_ON);
}

static void panel_bridge_disable(struct drm_bridge *bridge)
{
	struct goog_touch_interface *gti =
		container_of(bridge, struct goog_touch_interface, panel_bridge);

	if (bridge->encoder && bridge->encoder->crtc) {
		const struct drm_crtc_state *crtc_state = bridge->encoder->crtc->state;

		if (drm_atomic_crtc_effectively_active(crtc_state))
			return;
	}

	goog_set_display_state(gti, GTI_DISPLAY_STATE_OFF);
}

struct drm_connector *get_bridge_connector(struct drm_bridge *bridge)
{
	struct drm_connector *connector;
	struct drm_connector_list_iter conn_iter;

	drm_connector_list_iter_begin(bridge->dev, &conn_iter);
	drm_for_each_connector_iter(connector, &conn_iter) {
		if (connector->encoder == bridge->encoder)
			break;
	}
	drm_connector_list_iter_end(&conn_iter);
	return connector;
}

static bool panel_bridge_is_lp_mode(struct drm_connector *connector)
{
	if (connector && connector->state) {
		struct exynos_drm_connector_state *s =
			to_exynos_connector_state(connector->state);

		return s->exynos_mode.is_lp_mode;
	}
	return false;
}

static void panel_bridge_mode_set(struct drm_bridge *bridge,
				  const struct drm_display_mode *mode,
				  const struct drm_display_mode *adjusted_mode)
{
	int ret = 0;
	bool panel_is_lp_mode;
	struct goog_touch_interface *gti =
		container_of(bridge, struct goog_touch_interface, panel_bridge);

	if (!gti->connector || !gti->connector->state)
		gti->connector = get_bridge_connector(bridge);

	panel_is_lp_mode = panel_bridge_is_lp_mode(gti->connector);
	if (gti->panel_is_lp_mode != panel_is_lp_mode) {
		GOOG_INFO(gti, "panel_is_lp_mode changed from %d to %d.\n",
			gti->panel_is_lp_mode, panel_is_lp_mode);

		if (panel_is_lp_mode)
			goog_set_display_state(gti, GTI_DISPLAY_STATE_OFF);
		else
			goog_set_display_state(gti, GTI_DISPLAY_STATE_ON);
	}
	gti->panel_is_lp_mode = panel_is_lp_mode;

	if (mode) {
		int vrefresh = drm_mode_vrefresh(mode);

		if (gti->display_vrefresh != vrefresh) {
			GOOG_DBG(gti, "display_vrefresh(Hz) changed to %d from %d.\n",
				vrefresh, gti->display_vrefresh);
			gti->display_vrefresh = vrefresh;
			gti->cmd.display_vrefresh_cmd.setting = vrefresh;
			gti->context_changed.display_refresh_rate = 1;
			ret = goog_process_vendor_cmd(gti, GTI_CMD_NOTIFY_DISPLAY_VREFRESH);
			if (ret && ret != -EOPNOTSUPP)
				GOOG_WARN(gti, "unexpected return(%d)!", ret);

			if (gti->vrr_enabled)
				goog_lookup_touch_report_rate(gti);
		}
	}
}

static const struct drm_bridge_funcs panel_bridge_funcs = {
	.enable = panel_bridge_enable,
	.disable = panel_bridge_disable,
	.mode_set = panel_bridge_mode_set,
};

static int register_panel_bridge(struct goog_touch_interface *gti)
{
	GOOG_INFO(gti, "\n");
#ifdef CONFIG_OF
	gti->panel_bridge.of_node = gti->vendor_dev->of_node;
#endif
	gti->panel_bridge.funcs = &panel_bridge_funcs;
	drm_bridge_add(&gti->panel_bridge);

	return 0;
}

static void unregister_panel_bridge(struct drm_bridge *bridge)
{
	struct goog_touch_interface *gti =
		container_of(bridge, struct goog_touch_interface, panel_bridge);
	struct drm_bridge *node;

	GOOG_INFO(gti, "\n");
	drm_bridge_remove(bridge);

	if (!bridge->dev) /* not attached */
		return;

	drm_modeset_lock(&bridge->dev->mode_config.connection_mutex, NULL);
	list_for_each_entry(node, &bridge->encoder->bridge_chain, chain_node) {
		if (node == bridge) {
			if (bridge->funcs->detach)
				bridge->funcs->detach(bridge);
			list_del(&bridge->chain_node);
			break;
		}
	}
	drm_modeset_unlock(&bridge->dev->mode_config.connection_mutex);
	bridge->dev = NULL;
}

/*-----------------------------------------------------------------------------
 * GTI: functions.
 */
static int goog_precheck_heatmap(struct goog_touch_interface *gti)
{
	int ret = 0;

	/*
	 * Check the PM wakelock state and pm state for bus ownership before
	 * data request.
	 */
	if (!goog_pm_wake_get_locks(gti) || gti->pm.state == GTI_PM_SUSPEND) {
		GOOG_WARN(gti, "N/A during inactive bus!\n");
		ret = -ENODATA;
	}

	return ret;
}

static void goog_set_display_state(struct goog_touch_interface *gti,
	enum gti_display_state_setting display_state)
{
	int ret = 0;

	if (gti->display_state == display_state)
		return;

	switch (display_state) {
	case GTI_DISPLAY_STATE_OFF:
		GOOG_INFO(gti, "screen-off.\n");
		ret = goog_pm_wake_unlock_nosync(gti, GTI_PM_WAKELOCK_TYPE_SCREEN_ON);
		if (ret < 0)
			GOOG_INFO(gti, "Error while obtaining screen-off wakelock: %d!\n", ret);

		break;
	case GTI_DISPLAY_STATE_ON:
		GOOG_INFO(gti, "screen-on.\n");
		ret = goog_pm_wake_lock_nosync(gti, GTI_PM_WAKELOCK_TYPE_SCREEN_ON, false);
		if (ret < 0)
			GOOG_INFO(gti, "Error while obtaining screen-on wakelock: %d!\n", ret);

		break;
	default:
		GOOG_ERR(gti, "Unexpected value(0x%X) of display state parameter.\n",
			display_state);
		return;
	}

	gti->context_changed.screen_state = 1;
	gti->display_state = display_state;
	gti->cmd.display_state_cmd.setting = display_state;
	ret = goog_process_vendor_cmd(gti, GTI_CMD_NOTIFY_DISPLAY_STATE);
	if (ret && ret != -EOPNOTSUPP)
		GOOG_WARN(gti, "Unexpected vendor_cmd return(%d)!\n", ret);
}

bool goog_check_spi_dma_enabled(struct spi_device *spi_dev)
{
	bool ret = false;

	if (spi_dev && spi_dev->controller) {
		struct device_node *np = spi_dev->controller->dev.of_node;

		/*
		 * Check the SPI controller(s3c64xx-spi) whether support DMA
		 * or not.
		 */
		ret = of_property_read_bool(np, "dma-mode");
	}

	return ret;
}
EXPORT_SYMBOL(goog_check_spi_dma_enabled);

int goog_process_vendor_cmd(struct goog_touch_interface *gti, enum gti_cmd_type cmd_type)
{
	void *private_data = gti->vendor_private_data;
	int ret = -ESRCH;

	/* Use optional vendor operation if available. */
	switch (cmd_type) {
	case GTI_CMD_PING:
		ret = gti->options.ping(private_data, &gti->cmd.ping_cmd);
		break;
	case GTI_CMD_RESET:
		ret = gti->options.reset(private_data, &gti->cmd.reset_cmd);
		break;
	case GTI_CMD_SELFTEST:
		ret = gti->options.selftest(private_data, &gti->cmd.selftest_cmd);
		break;
	case GTI_CMD_GET_CONTEXT_DRIVER:
		ret = gti->options.get_context_driver(private_data, &gti->cmd.context_driver_cmd);
		break;
	case GTI_CMD_GET_CONTEXT_STYLUS:
		ret = gti->options.get_context_stylus(private_data, &gti->cmd.context_stylus_cmd);
		break;
	case GTI_CMD_GET_COORD_FILTER_ENABLED:
		ret = gti->options.get_coord_filter_enabled(private_data,
				&gti->cmd.coord_filter_cmd);
		break;
	case GTI_CMD_GET_FW_VERSION:
		ret = gti->options.get_fw_version(private_data, &gti->cmd.fw_version_cmd);
		break;
	case GTI_CMD_GET_GRIP_MODE:
		ret = gti->options.get_grip_mode(private_data, &gti->cmd.grip_cmd);
		break;
	case GTI_CMD_GET_IRQ_MODE:
		ret = gti->options.get_irq_mode(private_data, &gti->cmd.irq_cmd);
		break;
	case GTI_CMD_GET_PALM_MODE:
		ret = gti->options.get_palm_mode(private_data, &gti->cmd.palm_cmd);
		break;
	case GTI_CMD_GET_SCAN_MODE:
		ret = gti->options.set_scan_mode(private_data, &gti->cmd.scan_cmd);
		break;
	case GTI_CMD_GET_SCREEN_PROTECTOR_MODE:
		ret = gti->options.get_screen_protector_mode(private_data,
				&gti->cmd.screen_protector_mode_cmd);
		break;
	case GTI_CMD_GET_SENSING_MODE:
		ret = gti->options.get_sensing_mode(private_data, &gti->cmd.sensing_cmd);
		break;
	case GTI_CMD_GET_SENSOR_DATA:
		if (gti->cmd.sensor_data_cmd.type & TOUCH_SCAN_TYPE_MUTUAL) {
			ret = gti->options.get_mutual_sensor_data(
				private_data, &gti->cmd.sensor_data_cmd);
		} else if (gti->cmd.sensor_data_cmd.type & TOUCH_SCAN_TYPE_SELF) {
			ret = gti->options.get_self_sensor_data(
				private_data, &gti->cmd.sensor_data_cmd);
		}
		break;
	case GTI_CMD_GET_SENSOR_DATA_MANUAL:
		if (gti->cmd.manual_sensor_data_cmd.type & TOUCH_SCAN_TYPE_MUTUAL) {
			ret = gti->options.get_mutual_sensor_data(
				private_data, &gti->cmd.manual_sensor_data_cmd);
		} else if (gti->cmd.manual_sensor_data_cmd.type & TOUCH_SCAN_TYPE_SELF) {
			ret = gti->options.get_self_sensor_data(
				private_data, &gti->cmd.manual_sensor_data_cmd);
		}
		break;
	case GTI_CMD_NOTIFY_DISPLAY_STATE:
		ret = gti->options.notify_display_state(private_data,
				&gti->cmd.display_state_cmd);
		break;
	case GTI_CMD_NOTIFY_DISPLAY_VREFRESH:
		ret = gti->options.notify_display_vrefresh(private_data,
				&gti->cmd.display_vrefresh_cmd);
		break;
	case GTI_CMD_SET_CONTINUOUS_REPORT:
		ret = gti->options.set_continuous_report(private_data,
				&gti->cmd.continuous_report_cmd);
		break;
	case GTI_CMD_SET_COORD_FILTER_ENABLED:
		ret = gti->options.set_coord_filter_enabled(private_data,
				&gti->cmd.coord_filter_cmd);
		break;
	case GTI_CMD_SET_GRIP_MODE:
		GOOG_INFO(gti, "Set firmware grip %s",
				gti->cmd.grip_cmd.setting == GTI_GRIP_ENABLE ?
				"enabled" : "disabled");
		ret = gti->options.set_grip_mode(private_data, &gti->cmd.grip_cmd);
		break;
	case GTI_CMD_SET_HEATMAP_ENABLED:
		ret = gti->options.set_heatmap_enabled(private_data, &gti->cmd.heatmap_cmd);
		break;
	case GTI_CMD_SET_IRQ_MODE:
		ret = gti->options.set_irq_mode(private_data, &gti->cmd.irq_cmd);
		break;
	case GTI_CMD_SET_PALM_MODE:
		GOOG_INFO(gti, "Set firmware palm %s",
				gti->cmd.palm_cmd.setting == GTI_PALM_ENABLE ?
				"enabled" : "disabled");
		ret = gti->options.set_palm_mode(private_data, &gti->cmd.palm_cmd);
		break;
	case GTI_CMD_SET_REPORT_RATE:
		GOOG_INFO(gti, "Set touch report rate as %d Hz", gti->cmd.report_rate_cmd.setting);
		ret = gti->options.set_report_rate(private_data, &gti->cmd.report_rate_cmd);
		break;
	case GTI_CMD_SET_SCAN_MODE:
		ret = gti->options.set_scan_mode(private_data, &gti->cmd.scan_cmd);
		break;
	case GTI_CMD_SET_SCREEN_PROTECTOR_MODE:
		GOOG_INFO(gti, "Set screen protector mode %s",
				gti->cmd.screen_protector_mode_cmd.setting ==
				GTI_SCREEN_PROTECTOR_MODE_ENABLE
				? "enabled" : "disabled");
		ret = gti->options.set_screen_protector_mode(private_data,
				&gti->cmd.screen_protector_mode_cmd);
		break;
	case GTI_CMD_SET_SENSING_MODE:
		ret = gti->options.set_sensing_mode(private_data, &gti->cmd.sensing_cmd);
		break;
	default:
		break;
	}

	/* Back to vendor default handler if no optional operation available. */
	if (ret == -ESRCH)
		ret = gti->vendor_default_handler(private_data, cmd_type, &gti->cmd);

	/* Take unsupported cmd_type as debug logs for compatibility check. */
	if (ret == -EOPNOTSUPP) {
		GOOG_DBG(gti, "unsupported request cmd_type %#x!\n", cmd_type);
		ret = 0;
	} else if (ret == -ESRCH) {
		GOOG_WARN(gti, "No handler for cmd_type %#x!\n", cmd_type);
		ret = 0;
	}

	return ret;
}

void goog_update_motion_filter(struct goog_touch_interface *gti, unsigned long slot_bit)
{
	int ret = 0;
	const u32 mf_timeout_ms = 500;
	unsigned long touches = hweight_long(slot_bit);
	u32 next_state = gti->mf_state;

	switch (gti->mf_mode) {
	case GTI_MF_MODE_AUTO_REPORT:
	case GTI_MF_MODE_UNFILTER:
		next_state = GTI_MF_STATE_UNFILTERED;
		break;
	case GTI_MF_MODE_FILTER:
		next_state = GTI_MF_STATE_FILTERED;
		break;
	case GTI_MF_MODE_DYNAMIC:
	default:
		/*
		* Determine the next filter state. The motion filter is enabled by
		* default and it is disabled while a single finger is touching the
		* screen. If another finger is touched down or if a timeout expires,
		* the motion filter is reenabled and remains enabled until all fingers
		* are lifted.
		*/
		switch (next_state) {
		case GTI_MF_STATE_FILTERED:
			if (touches == 1) {
				next_state = GTI_MF_STATE_UNFILTERED;
				gti->mf_downtime = ktime_get();
			}
			break;
		case GTI_MF_STATE_UNFILTERED:
			if (touches == 0) {
				next_state = GTI_MF_STATE_FILTERED;
			} else if (touches > 1 ||
					ktime_after(ktime_get(),
					ktime_add_ms(gti->mf_downtime, mf_timeout_ms))) {
				next_state = GTI_MF_STATE_FILTERED_LOCKED;
			}
			break;
		case GTI_MF_STATE_FILTERED_LOCKED:
			if (touches == 0)
				next_state = GTI_MF_STATE_FILTERED;
			break;
		}
		break;
	}

	/* Send command to setup continuous report. */
	if ((next_state == GTI_MF_STATE_UNFILTERED) !=
		(gti->mf_state == GTI_MF_STATE_UNFILTERED)) {
		gti->cmd.continuous_report_cmd.setting = GTI_CONTINUOUS_REPORT_DISABLE;

		if (next_state == GTI_MF_STATE_UNFILTERED)
			gti->cmd.continuous_report_cmd.setting = GTI_CONTINUOUS_REPORT_ENABLE;

		ret = goog_process_vendor_cmd(gti, GTI_CMD_SET_CONTINUOUS_REPORT);
		if (ret)
			GOOG_WARN(gti, "unexpected return(%d)!", ret);
	}

	gti->mf_state = next_state;
}

bool goog_v4l2_read_frame_cb(struct v4l2_heatmap *v4l2)
{
	struct goog_touch_interface *gti = container_of(v4l2, struct goog_touch_interface, v4l2);
	bool ret = false;
	u32 v4l2_size = gti->v4l2.width * gti->v4l2.height * 2;

	if (gti->heatmap_buf && v4l2_size == gti->heatmap_buf_size) {
		memcpy(v4l2->frame, gti->heatmap_buf, v4l2_size);
		ret = true;
	} else {
		GOOG_ERR(gti, "wrong pointer(%p) or size (W: %lu, H: %lu) vs %u\n",
		gti->heatmap_buf, gti->v4l2.width, gti->v4l2.height, gti->heatmap_buf_size);
	}

	return ret;
}

void goog_v4l2_read(struct goog_touch_interface *gti, ktime_t timestamp)
{
	if (gti->v4l2_enabled)
		heatmap_read(&gti->v4l2, ktime_to_ns(timestamp));
}

int goog_get_driver_status(struct goog_touch_interface *gti,
		struct gti_context_driver_cmd *driver_cmd)
{
	gti->context_changed.offload_timestamp = 1;

	driver_cmd->context_changed.value = gti->context_changed.value;
	driver_cmd->screen_state = gti->display_state;
	driver_cmd->display_refresh_rate = gti->display_vrefresh;
	driver_cmd->touch_report_rate = gti->report_rate_setting;
	driver_cmd->noise_state = gti->fw_status.noise_level;
	driver_cmd->water_mode = gti->fw_status.water_mode;
	driver_cmd->charger_state = gti->charger_state;
	driver_cmd->offload_timestamp = ktime_get();

	/* vendor driver overwrite the context */
	return goog_process_vendor_cmd(gti, GTI_CMD_GET_CONTEXT_DRIVER);
}

void goog_offload_populate_coordinate_channel(struct goog_touch_interface *gti,
		struct touch_offload_frame *frame, int channel)
{
	int i;
	struct TouchOffloadDataCoord *dc;

	if (channel < 0 || channel >= MAX_CHANNELS) {
		GOOG_ERR(gti, "Invalid channel: %d\n", channel);
		return;
	}

	dc = (struct TouchOffloadDataCoord *)frame->channel_data[channel];
	memset(dc, 0, frame->channel_data_size[channel]);
	dc->header.channel_type = TOUCH_DATA_TYPE_COORD;
	dc->header.channel_size = TOUCH_OFFLOAD_FRAME_SIZE_COORD;

	for (i = 0; i < MAX_SLOTS; i++) {
		dc->coords[i].x = gti->offload.coords[i].x;
		dc->coords[i].y = gti->offload.coords[i].y;
		dc->coords[i].major = gti->offload.coords[i].major;
		dc->coords[i].minor = gti->offload.coords[i].minor;
		dc->coords[i].pressure = gti->offload.coords[i].pressure;
		dc->coords[i].rotation = gti->offload.coords[i].rotation;
		dc->coords[i].status = gti->offload.coords[i].status;
	}
}

void goog_offload_populate_mutual_channel(struct goog_touch_interface *gti,
		struct touch_offload_frame *frame, int channel, u8 *buffer, u32 size)
{
	struct TouchOffloadData2d *mutual;

	if (channel < 0 || channel >= MAX_CHANNELS) {
		GOOG_ERR(gti, "Invalid channel: %d\n", channel);
		return;
	}

	mutual = (struct TouchOffloadData2d *)frame->channel_data[channel];
	mutual->tx_size = gti->offload.caps.tx_size;
	mutual->rx_size = gti->offload.caps.rx_size;
	mutual->header.channel_type = frame->channel_type[channel];
	mutual->header.channel_size =
		TOUCH_OFFLOAD_FRAME_SIZE_2D(mutual->rx_size, mutual->tx_size);

	memcpy(mutual->data, buffer, size);
}

void goog_offload_populate_self_channel(struct goog_touch_interface *gti,
		struct touch_offload_frame *frame, int channel, u8 *buffer, u32 size)
{
	struct TouchOffloadData1d *self;

	if (channel < 0 || channel >= MAX_CHANNELS) {
		GOOG_ERR(gti, "Invalid channel: %d\n", channel);
		return;
	}

	self = (struct TouchOffloadData1d *)frame->channel_data[channel];
	self->tx_size = gti->offload.caps.tx_size;
	self->rx_size = gti->offload.caps.rx_size;
	self->header.channel_type = frame->channel_type[channel];
	self->header.channel_size =
		TOUCH_OFFLOAD_FRAME_SIZE_1D(self->rx_size, self->tx_size);

	memcpy(self->data, buffer, size);
}

static void goog_offload_populate_driver_status_channel(
		struct goog_touch_interface *gti,
		struct touch_offload_frame *frame, int channel,
		struct gti_context_driver_cmd *driver_cmd)
{
	struct TouchOffloadDriverStatus *ds =
		(struct TouchOffloadDriverStatus *)frame->channel_data[channel];

	memset(ds, 0, frame->channel_data_size[channel]);
	ds->header.channel_type = (u32)CONTEXT_CHANNEL_TYPE_DRIVER_STATUS;
	ds->header.channel_size = sizeof(struct TouchOffloadDriverStatus);

	ds->contents.screen_state = driver_cmd->context_changed.screen_state;
	ds->screen_state = driver_cmd->screen_state;

	ds->contents.display_refresh_rate = driver_cmd->context_changed.display_refresh_rate;
	ds->display_refresh_rate = driver_cmd->display_refresh_rate;

	ds->contents.touch_report_rate = driver_cmd->context_changed.touch_report_rate;
	ds->touch_report_rate = driver_cmd->touch_report_rate;

	ds->contents.noise_state = driver_cmd->context_changed.noise_state;
	ds->noise_state = driver_cmd->noise_state;

	ds->contents.water_mode = driver_cmd->context_changed.water_mode;
	ds->water_mode = driver_cmd->water_mode;

	ds->contents.charger_state = driver_cmd->context_changed.charger_state;
	ds->charger_state = driver_cmd->charger_state;

	ds->contents.offload_timestamp = driver_cmd->context_changed.offload_timestamp;
	ds->offload_timestamp = driver_cmd->offload_timestamp;
}

static void goog_offload_populate_stylus_status_channel(
		struct goog_touch_interface *gti,
		struct touch_offload_frame *frame, int channel,
		struct gti_context_stylus_cmd *stylus_cmd)
{
	struct TouchOffloadStylusStatus *ss =
		(struct TouchOffloadStylusStatus *)frame->channel_data[channel];

	memset(ss, 0, frame->channel_data_size[channel]);
	ss->header.channel_type = (u32)CONTEXT_CHANNEL_TYPE_STYLUS_STATUS;
	ss->header.channel_size = sizeof(struct TouchOffloadStylusStatus);

	ss->contents.coords = stylus_cmd->contents.coords;
	ss->coords[0] = stylus_cmd->pen_offload_coord;

	ss->contents.coords_timestamp = stylus_cmd->contents.coords_timestamp;
	ss->coords_timestamp = stylus_cmd->pen_offload_coord_timestamp;

	ss->contents.pen_paired = stylus_cmd->contents.pen_paired;
	ss->pen_paired = stylus_cmd->pen_paired;

	ss->contents.pen_active = stylus_cmd->contents.pen_active;
	ss->pen_active = stylus_cmd->pen_active;
}

static int goog_get_sensor_data(struct goog_touch_interface *gti,
		struct gti_sensor_data_cmd *cmd, bool reset_data)
{
	int ret = 0;
	int err = 0;
	u16 tx = gti->offload.caps.tx_size;
	u16 rx = gti->offload.caps.rx_size;

	if (reset_data) {
		if (cmd->type == GTI_SENSOR_DATA_TYPE_MS) {
			cmd->size = TOUCH_OFFLOAD_DATA_SIZE_2D(rx, tx);
		} else if (cmd->type == GTI_SENSOR_DATA_TYPE_SS) {
			cmd->size = TOUCH_OFFLOAD_DATA_SIZE_1D(rx, tx);
		} else {
			ret = -EINVAL;
			goto exit;
		}

		memset(gti->heatmap_buf, 0, cmd->size);
		cmd->buffer = gti->heatmap_buf;
		goto exit;
	}

	err = goog_pm_wake_lock(gti, GTI_PM_WAKELOCK_TYPE_SENSOR_DATA, true);
	if (err < 0) {
		GOOG_WARN(gti, "Failed to lock GTI_PM_WAKELOCK_TYPE_SENSOR_DATA: %d!\n", err);
		ret = err;
		goto exit;
	}

	err = goog_process_vendor_cmd(gti, GTI_CMD_GET_SENSOR_DATA);
	if (err < 0) {
		GOOG_WARN(gti, "Failed to get sensor data: %d!\n", err);
		ret = err;
	}

	err = goog_pm_wake_unlock(gti, GTI_PM_WAKELOCK_TYPE_SENSOR_DATA);
	if (err < 0)
		GOOG_WARN(gti, "Failed to unlock GTI_PM_WAKELOCK_TYPE_SENSOR_DATA: %d!\n", err);

exit:
	return ret;
}

void goog_offload_populate_frame(struct goog_touch_interface *gti,
		struct touch_offload_frame *frame, bool reset_data)
{
	static u64 index;
	char trace_tag[128];
	u32 channel_type;
	int i;
	int ret;
	u16 tx = gti->offload.caps.tx_size;
	u16 rx = gti->offload.caps.rx_size;
	struct gti_sensor_data_cmd *cmd = &gti->cmd.sensor_data_cmd;

	scnprintf(trace_tag, sizeof(trace_tag), "%s: IDX=%llu IN_TS=%lld.\n",
		__func__, index, gti->input_timestamp);
	ATRACE_BEGIN(trace_tag);

	frame->header.index = index++;
	frame->header.timestamp = gti->input_timestamp;

	/*
	 * TODO(b/201610482):
	 * Porting for other channels, like driver status, stylus status
	 * and others.
	 */

	/* Populate all channels */
	for (i = 0; i < frame->num_channels; i++) {
		channel_type = frame->channel_type[i];
		GOOG_DBG(gti, "#%d: get data(type %#x) from vendor driver", i, channel_type);
		ret = 0;
		cmd->buffer = NULL;
		cmd->size = 0;
		if (channel_type == CONTEXT_CHANNEL_TYPE_DRIVER_STATUS) {
			ATRACE_BEGIN("populate driver context");
			ret = goog_get_driver_status(gti, &gti->cmd.context_driver_cmd);
			if (ret == 0)
				goog_offload_populate_driver_status_channel(
						gti, frame, i,
						&gti->cmd.context_driver_cmd);
			ATRACE_END();
		} else if (channel_type == CONTEXT_CHANNEL_TYPE_STYLUS_STATUS) {
			ATRACE_BEGIN("populate stylus context");
			ret = goog_process_vendor_cmd(gti, GTI_CMD_GET_CONTEXT_STYLUS);
			if (ret == 0)
				goog_offload_populate_stylus_status_channel(
						gti, frame, i,
						&gti->cmd.context_stylus_cmd);
			ATRACE_END();
		} else if (channel_type == TOUCH_DATA_TYPE_COORD) {
			ATRACE_BEGIN("populate coord");
			goog_offload_populate_coordinate_channel(gti, frame, i);
			ATRACE_END();
		} else if (channel_type & TOUCH_SCAN_TYPE_MUTUAL) {
			ATRACE_BEGIN("populate mutual data");
			cmd->type = GTI_SENSOR_DATA_TYPE_MS;

			ret = goog_get_sensor_data(gti, cmd, reset_data);
			if (ret == 0 && cmd->buffer &&
				cmd->size == TOUCH_OFFLOAD_DATA_SIZE_2D(rx, tx)) {
				goog_offload_populate_mutual_channel(gti, frame, i,
					cmd->buffer, cmd->size);
				/* Backup strength data for v4l2. */
				if (channel_type & TOUCH_DATA_TYPE_STRENGTH)
					memcpy(gti->heatmap_buf, cmd->buffer, cmd->size);
			}
			ATRACE_END();
		} else if (channel_type & TOUCH_SCAN_TYPE_SELF) {
			ATRACE_BEGIN("populate self data");
			cmd->type = GTI_SENSOR_DATA_TYPE_SS;

			ret = goog_get_sensor_data(gti, cmd, reset_data);
			if (ret == 0 && cmd->buffer &&
				cmd->size == TOUCH_OFFLOAD_DATA_SIZE_1D(rx, tx)) {
				goog_offload_populate_self_channel(gti, frame, i,
					cmd->buffer, cmd->size);
			}
			ATRACE_END();
		} else {
			GOOG_ERR(gti, "unrecognized channel_type %#x.\n", channel_type);
		}

		if (ret) {
			GOOG_DBG(gti, "skip to populate data(type %#x, ret %d)!\n",
				channel_type, ret);
		}
	}

	ATRACE_END();
}

void goog_update_fw_settings(struct goog_touch_interface *gti)
{
	int error;
	int ret = 0;
	bool enabled = false;

	error = goog_pm_wake_lock_nosync(gti, GTI_PM_WAKELOCK_TYPE_FW_SETTINGS, true);
	if (error < 0) {
		GOOG_DBG(gti, "Error while obtaining FW_SETTINGS wakelock: %d!\n", error);
		return;
	}

	if(!gti->ignore_grip_update) {
		if (gti->offload.offload_running && gti->offload.config.filter_grip)
			gti->cmd.grip_cmd.setting = GTI_GRIP_DISABLE;
		else
			gti->cmd.grip_cmd.setting = gti->default_grip_enabled;
		ret = goog_process_vendor_cmd(gti, GTI_CMD_SET_GRIP_MODE);
		if (ret)
			GOOG_WARN(gti, "unexpected return(%d)!", ret);
	}

	if(!gti->ignore_palm_update) {
		if (gti->offload.offload_running && gti->offload.config.filter_palm)
			gti->cmd.palm_cmd.setting = GTI_PALM_DISABLE;
		else
			gti->cmd.palm_cmd.setting = gti->default_palm_enabled;
		ret = goog_process_vendor_cmd(gti, GTI_CMD_SET_PALM_MODE);
		if (ret)
			GOOG_WARN(gti, "unexpected return(%d)!", ret);
	}

	if (gti->coord_filter_enabled) {
		if (!gti->ignore_coord_filter_update) {
			if (gti->offload.offload_running && gti->offload.config.coord_filter)
				enabled = false;
			else
				enabled = gti->default_coord_filter_enabled;
		} else {
			enabled = gti->fw_coord_filter_enabled;
		}

		gti->cmd.coord_filter_cmd.setting = enabled ?
				GTI_COORD_FILTER_ENABLE : GTI_COORD_FILTER_DISABLE;
		ret = goog_process_vendor_cmd(gti, GTI_CMD_SET_COORD_FILTER_ENABLED);
		if (ret)
			GOOG_WARN(gti, "unexpected return(%d)!", ret);
	}

	gti->cmd.screen_protector_mode_cmd.setting = gti->screen_protector_mode_setting;
	ret = goog_process_vendor_cmd(gti, GTI_CMD_SET_SCREEN_PROTECTOR_MODE);
	if (ret != 0)
		GOOG_ERR(gti, "Failed to %s screen protector mode!\n",
			gti->screen_protector_mode_setting == GTI_SCREEN_PROTECTOR_MODE_ENABLE ?
			"enable" : "disable");

	gti->cmd.heatmap_cmd.setting = GTI_HEATMAP_ENABLE;
	ret = goog_process_vendor_cmd(gti, GTI_CMD_SET_HEATMAP_ENABLED);
	if (ret != 0)
		GOOG_ERR(gti, "Failed to enable heatmap!\n");

	if (gti->vrr_enabled) {
		gti->cmd.report_rate_cmd.setting = gti->report_rate_setting_next;
		ret = goog_process_vendor_cmd(gti, GTI_CMD_SET_REPORT_RATE);
		if (ret != 0)
			GOOG_ERR(gti, "Failed to set report rate!\n");
	}

	error = goog_pm_wake_unlock_nosync(gti, GTI_PM_WAKELOCK_TYPE_FW_SETTINGS);
	if (error < 0)
		GOOG_DBG(gti, "Error while releasing FW_SETTING wakelock: %d!\n", error);
}

static void goog_offload_set_running(struct goog_touch_interface *gti, bool running)
{
	if (gti->offload.offload_running != running) {
		GOOG_INFO(gti, "Set offload_running=%d, irq_index=%d, input_index=%d\n",
			running, gti->irq_index, gti->input_index);

		gti->offload.offload_running = running;
		goog_update_fw_settings(gti);
	}
}

void goog_offload_input_report(void *handle,
		 struct TouchOffloadIocReport *report)
{
	struct goog_touch_interface *gti = (struct goog_touch_interface *)handle;
	bool touch_down = 0;
	unsigned int tool_type = MT_TOOL_FINGER;
	int i;
	int error;
	unsigned long slot_bit_active = 0;
	char trace_tag[128];
	ktime_t ktime = ktime_get();

	scnprintf(trace_tag, sizeof(trace_tag),
		"%s: IDX=%lld IN_TS=%lld TS=%lld DELTA=%lld ns.\n",
		__func__, report->index,
		ktime_to_ns(report->timestamp), ktime_to_ns(ktime),
		ktime_to_ns(ktime_sub(ktime, report->timestamp)));
	ATRACE_BEGIN(trace_tag);

	goog_input_lock(gti);
	input_set_timestamp(gti->vendor_input_dev, report->timestamp);
	for (i = 0; i < MAX_SLOTS; i++) {
		if (report->coords[i].status != COORD_STATUS_INACTIVE) {
			switch (report->coords[i].status) {
			case COORD_STATUS_EDGE:
			case COORD_STATUS_PALM:
			case COORD_STATUS_CANCEL:
				tool_type = MT_TOOL_PALM;
				break;
			case COORD_STATUS_FINGER:
			case COORD_STATUS_PEN:
			default:
				tool_type = MT_TOOL_FINGER;
				break;
			}
			set_bit(i, &slot_bit_active);
			input_mt_slot(gti->vendor_input_dev, i);
			touch_down = 1;
			input_report_key(gti->vendor_input_dev, BTN_TOUCH, touch_down);
			input_mt_report_slot_state(gti->vendor_input_dev, tool_type, 1);
			input_report_abs(gti->vendor_input_dev, ABS_MT_POSITION_X,
				report->coords[i].x);
			input_report_abs(gti->vendor_input_dev, ABS_MT_POSITION_Y,
				report->coords[i].y);
			input_report_abs(gti->vendor_input_dev, ABS_MT_TOUCH_MAJOR,
				report->coords[i].major);
			input_report_abs(gti->vendor_input_dev, ABS_MT_TOUCH_MINOR,
				report->coords[i].minor);
			input_report_abs(gti->vendor_input_dev, ABS_MT_PRESSURE,
				report->coords[i].pressure);
			if (gti->offload.caps.rotation_reporting)
				input_report_abs(gti->vendor_input_dev, ABS_MT_ORIENTATION,
					report->coords[i].rotation);
		} else {
			clear_bit(i, &slot_bit_active);
			input_mt_slot(gti->vendor_input_dev, i);
			input_report_abs(gti->vendor_input_dev, ABS_MT_PRESSURE, 0);
			input_mt_report_slot_state(gti->vendor_input_dev, MT_TOOL_FINGER, 0);
			input_report_abs(gti->vendor_input_dev, ABS_MT_TRACKING_ID, -1);
		}
	}
	input_report_key(gti->vendor_input_dev, BTN_TOUCH, touch_down);
	input_sync(gti->vendor_input_dev);
	goog_input_unlock(gti);

	if (touch_down)
		goog_v4l2_read(gti, report->timestamp);

	error = goog_pm_wake_lock(gti, GTI_PM_WAKELOCK_TYPE_OFFLOAD_REPORT, true);
	if (error < 0) {
		GOOG_WARN(gti, "Error while obtaining OFFLOAD_REPORT wakelock: %d!\n", error);
		ATRACE_END();
		return;
	}
	goog_update_motion_filter(gti, slot_bit_active);
	error = goog_pm_wake_unlock(gti, GTI_PM_WAKELOCK_TYPE_OFFLOAD_REPORT);
	if (error < 0)
		GOOG_WARN(gti, "Error while releasing OFFLOAD_REPORT wakelock: %d!\n", error);
	ATRACE_END();
}

int gti_charger_state_change(struct notifier_block *nb, unsigned long action,
			     void *data)
{
	struct goog_touch_interface *gti =
		(struct goog_touch_interface *)container_of(nb,
			struct goog_touch_interface, charger_notifier);
	struct power_supply *psy = (struct power_supply *)data;
	int ret;

	/* Attempt actual status parsing */
	if (psy && psy->desc->type == POWER_SUPPLY_TYPE_USB) {
		union power_supply_propval present_val = { 0 };

		ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_PRESENT,
						&present_val);
		if (ret < 0)
			GOOG_DBG(gti,
				 "Error while getting power supply property: %d!\n",
				 ret);
		else if ((u8)present_val.intval != gti->charger_state) {
			/* Note: the expected values for present_val.intval are
			 * 0 and 1. Cast to unsigned byte to ensure the
			 * comparison is handled in the same variable data type.
			 */
			gti->context_changed.charger_state = 1;
			gti->charger_state = (u8)present_val.intval;
		}
	}

	return 0;
}

int goog_offload_probe(struct goog_touch_interface *gti)
{
	int ret;
	u16 values[2];
	struct device_node *np = gti->vendor_dev->of_node;
	const char *offload_dev_name = NULL;

	/*
	 * TODO(b/201610482): rename DEVICE_NAME in touch_offload.h for more specific.
	 */
	if (!of_property_read_string(np, "goog,offload-device-name", &offload_dev_name)) {
		scnprintf(gti->offload.device_name, sizeof(gti->offload.device_name),
			"%s_%s", DEVICE_NAME, offload_dev_name);
	}

	if (of_property_read_u8_array(np, "goog,touch_offload_id",
					  gti->offload_id_byte, 4)) {
		GOOG_INFO(gti, "set default offload id: GOOG!\n");
		gti->offload_id_byte[0] = 'G';
		gti->offload_id_byte[1] = 'O';
		gti->offload_id_byte[2] = 'O';
		gti->offload_id_byte[3] = 'G';
	}

	gti->offload.caps.touch_offload_major_version = TOUCH_OFFLOAD_INTERFACE_MAJOR_VERSION;
	gti->offload.caps.touch_offload_minor_version = TOUCH_OFFLOAD_INTERFACE_MINOR_VERSION;
	gti->offload.caps.device_id = gti->offload_id;

	if (of_property_read_u16_array(np, "goog,display-resolution",
					  values, 2) == 0) {
		gti->offload.caps.display_width = values[0];
		gti->offload.caps.display_height = values[1];
	} else {
		GOOG_ERR(gti, "Please set \"goog,display-resolution\" in dts!");
	}

	if (of_property_read_u16_array(np, "goog,channel-num",
					  values, 2) == 0) {
		gti->offload.caps.tx_size = values[0];
		gti->offload.caps.rx_size = values[1];
	} else {
		GOOG_ERR(gti, "Please set \"goog,channel-num\" in dts!");
		ret = -EINVAL;
		goto err_offload_probe;
	}

	/*
	 * TODO(b/201610482): Set more offload caps from parameters or from dtsi?
	 */
	gti->offload.caps.heatmap_size = HEATMAP_SIZE_FULL;
	gti->offload.caps.bus_type = BUS_TYPE_SPI;
	if (of_property_read_u32(np, "spi-max-frequency",
			&gti->offload.caps.bus_speed_hz))
		gti->offload.caps.bus_speed_hz = 0;

	if (of_property_read_u16(np, "goog,offload-caps-data-types",
			&gti->offload.caps.touch_data_types)) {
		gti->offload.caps.touch_data_types =
			TOUCH_DATA_TYPE_COORD | TOUCH_DATA_TYPE_STRENGTH |
			TOUCH_DATA_TYPE_RAW | TOUCH_DATA_TYPE_BASELINE;
	}
	if (of_property_read_u16(np, "goog,offload-caps-scan-types",
			&gti->offload.caps.touch_scan_types)) {
		gti->offload.caps.touch_scan_types =
			TOUCH_SCAN_TYPE_MUTUAL;
	}
	if (of_property_read_u16(np, "goog,offload-caps-context-channel-types",
			&gti->offload.caps.context_channel_types)) {
		gti->offload.caps.context_channel_types = 0;
	}
	GOOG_INFO(gti, "offload.caps: data_types %#x, scan_types %#x, context_channel_types %#x.\n",
		gti->offload.caps.touch_data_types,
		gti->offload.caps.touch_scan_types,
		gti->offload.caps.context_channel_types);

	gti->offload.caps.continuous_reporting = true;
	gti->offload.caps.noise_reporting = false;
	gti->offload.caps.cancel_reporting =
		of_property_read_bool(np, "goog,offload-caps-cancel-reporting");
	gti->offload.caps.size_reporting = true;
	gti->offload.caps.filter_grip = true;
	gti->offload.caps.filter_palm = true;
	gti->offload.caps.coord_filter = gti->coord_filter_enabled &&
		of_property_read_bool(np, "goog,offload-caps-coord-filter");
	gti->offload.caps.num_sensitivity_settings = 1;
	gti->offload.caps.rotation_reporting = of_property_read_bool(np,
		"goog,offload-caps-rotation-reporting");

	gti->offload.hcallback = (void *)gti;
	gti->offload.report_cb = goog_offload_input_report;
	ret = touch_offload_init(&gti->offload);
	if (ret) {
		GOOG_ERR(gti, "offload init failed, ret %d!\n", ret);
		goto err_offload_probe;
	}

	gti->offload_enabled = of_property_read_bool(np, "goog,offload-enabled");
	GOOG_INFO(gti, "offload.caps: display W/H: %d * %d (Tx/Rx: %d * %d).\n",
		gti->offload.caps.display_width, gti->offload.caps.display_height,
		gti->offload.caps.tx_size, gti->offload.caps.rx_size);

	GOOG_INFO(gti, "offload ID: \"%c%c%c%c\" / 0x%08X, offload_enabled=%d.\n",
		gti->offload_id_byte[0], gti->offload_id_byte[1], gti->offload_id_byte[2],
		gti->offload_id_byte[3], gti->offload_id, gti->offload_enabled);

	gti->default_grip_enabled = of_property_read_bool(np,
			"goog,default-grip-disabled") ? GTI_GRIP_DISABLE : GTI_GRIP_ENABLE;
	gti->default_palm_enabled = of_property_read_bool(np,
			"goog,default-palm-disabled") ? GTI_PALM_DISABLE : GTI_PALM_ENABLE;
	gti->default_coord_filter_enabled = of_property_read_bool(np,
			"goog,default-coord-filter-disabled") ?
			GTI_COORD_FILTER_DISABLE : GTI_COORD_FILTER_ENABLE;

	gti->heatmap_buf_size = gti->offload.caps.tx_size * gti->offload.caps.rx_size * sizeof(u16);
	gti->heatmap_buf = devm_kzalloc(gti->vendor_dev, gti->heatmap_buf_size, GFP_KERNEL);
	if (!gti->heatmap_buf) {
		GOOG_ERR(gti, "heamap alloc failed!\n");
		ret = -ENOMEM;
		goto err_offload_probe;
	}

	/*
	 * Heatmap_probe must be called before irq routine is registered,
	 * because heatmap_read is called from the irq context.
	 * If the ISR runs before heatmap_probe is finished, it will invoke
	 * heatmap_read and cause NPE, since read_frame would not yet be set.
	 */
	gti->v4l2.parent_dev = gti->vendor_dev;
	gti->v4l2.input_dev = gti->vendor_input_dev;
	gti->v4l2.read_frame = goog_v4l2_read_frame_cb;
	gti->v4l2.width = gti->offload.caps.tx_size;
	gti->v4l2.height = gti->offload.caps.rx_size;

	/* 120 Hz operation */
	gti->v4l2.timeperframe.numerator = 1;
	if (of_property_read_u32(np, "goog,report-rate",
			&gti->v4l2.timeperframe.denominator))
		gti->v4l2.timeperframe.denominator = 120;

	ret = heatmap_probe(&gti->v4l2);
	if (ret) {
		GOOG_ERR(gti, "v4l2 init failed, ret %d!\n", ret);
		goto err_offload_probe;
	}
	gti->v4l2_enabled = of_property_read_bool(np, "goog,v4l2-enabled");
	GOOG_INFO(gti, "v4l2 W/H=(%lu, %lu), v4l2_enabled=%d.\n",
		gti->v4l2.width, gti->v4l2.height, gti->v4l2_enabled);

	/* Register for charger plugging status */
	gti->charger_notifier.notifier_call = gti_charger_state_change;
	ret = power_supply_reg_notifier(&gti->charger_notifier);
	if (!ret) {
		GOOG_ERR(gti, "Failed to register power_supply_reg_notifier!\n");
		goto err_offload_probe;
	}

err_offload_probe:
	return ret;
}

void goog_offload_remove(struct goog_touch_interface *gti)
{
	touch_offload_cleanup(&gti->offload);
}

bool goog_input_legacy_report(struct goog_touch_interface *gti)
{
	if (!gti->offload.offload_running)
		return true;

	return false;
}

int goog_input_process(struct goog_touch_interface *gti, bool reset_data)
{
	int ret = 0;
	struct touch_offload_frame **frame = &gti->offload_frame;

	/*
	 * Only do the input process if active slot(s) update
	 * or slot(s) state change or resetting frame data.
	 */
	if (!(gti->slot_bit_active & gti->slot_bit_in_use) &&
		!gti->slot_bit_changed && !reset_data)
		return -EPERM;

	/*
	 * Increase the input index when any slot bit changed which
	 * means the finger is down or up.
	 */
	if (gti->slot_bit_changed)
		gti->input_index++;

	if (gti->offload_enabled) {
		ret = touch_offload_reserve_frame(&gti->offload, frame);
		if (ret != 0 || frame == NULL) {
			GOOG_DBG(gti, "could not reserve a frame(ret %d)!\n", ret);

			/* Stop offload when there are no buffers available. */
			goog_offload_set_running(gti, false);
			/*
			 * TODO(b/193467748):
			 * How to handle current coord if offload running
			 * terminating in the halfway(not beginning case)?
			 */
			ret = -EBUSY;
		} else {
			goog_offload_set_running(gti, true);
			goog_offload_populate_frame(gti, *frame, reset_data);
			ret = touch_offload_queue_frame(&gti->offload, *frame);
			if (ret)
				GOOG_ERR(gti, "failed to queue reserved frame(ret %d)!\n", ret);
			else
				gti->offload_frame = NULL;
		}
	}

	/*
	 * If offload is NOT running, read heatmap directly by callback.
	 * Otherwise, heatmap will be handled for both offload and v4l2
	 * during goog_offload_populate_frame().
	 */
	if (!gti->offload.offload_running && gti->v4l2_enabled) {
		int ret;
		struct gti_sensor_data_cmd *cmd = &gti->cmd.sensor_data_cmd;


		cmd->buffer = NULL;
		cmd->size = 0;
		cmd->type = GTI_SENSOR_DATA_TYPE_MS;
		ret = goog_get_sensor_data(gti, cmd, reset_data);
		if (ret == 0 && cmd->buffer && cmd->size)
			memcpy(gti->heatmap_buf, cmd->buffer, cmd->size);
		goog_v4l2_read(gti, gti->input_timestamp);
		goog_update_motion_filter(gti, gti->slot_bit_active);
	}

	gti_debug_input_update(gti);
	gti->input_timestamp_changed = false;
	gti->slot_bit_in_use = 0;

	return ret;
}
EXPORT_SYMBOL(goog_input_process);

void goog_input_lock(struct goog_touch_interface *gti)
{
	mutex_lock(&gti->input_lock);
}
EXPORT_SYMBOL(goog_input_lock);

void goog_input_unlock(struct goog_touch_interface *gti)
{
	mutex_unlock(&gti->input_lock);
}
EXPORT_SYMBOL(goog_input_unlock);

void goog_input_set_timestamp(
		struct goog_touch_interface *gti,
		struct input_dev *dev, ktime_t timestamp)
{
	if (goog_input_legacy_report(gti))
		input_set_timestamp(dev, timestamp);

	gti->input_timestamp = timestamp;
	gti->input_timestamp_changed = true;
}
EXPORT_SYMBOL(goog_input_set_timestamp);

void goog_input_mt_slot(
		struct goog_touch_interface *gti,
		struct input_dev *dev, int slot)
{
	if (slot < 0 || slot >= MAX_SLOTS) {
		GOOG_ERR(gti, "Invalid slot: %d\n", slot);
		return;
	}

	if (goog_input_legacy_report(gti))
		input_mt_slot(dev, slot);

	gti->slot = slot;
	/*
	 * Make sure the input timestamp should be set before updating 1st mt_slot.
	 * This is for input report switch between offload and legacy.
	 */
	if (!gti->slot_bit_in_use && !gti->input_timestamp_changed)
		GOOG_ERR(gti, "please exec goog_input_set_timestamp before %s!\n", __func__);
	set_bit(slot, &gti->slot_bit_in_use);
}
EXPORT_SYMBOL(goog_input_mt_slot);

void goog_input_mt_report_slot_state(
		struct goog_touch_interface *gti,
		struct input_dev *dev, unsigned int tool_type, bool active)
{
	if (goog_input_legacy_report(gti))
		input_mt_report_slot_state(dev, tool_type, active);

	switch (tool_type) {
	case MT_TOOL_FINGER:
		if (active) {
			gti->offload.coords[gti->slot].status = COORD_STATUS_FINGER;
			if (!test_and_set_bit(gti->slot,
					&gti->slot_bit_active)) {
				set_bit(gti->slot, &gti->slot_bit_changed);
			}
		} else {
			gti->offload.coords[gti->slot].status = COORD_STATUS_INACTIVE;
			if (test_and_clear_bit(gti->slot,
					&gti->slot_bit_active)) {
				set_bit(gti->slot, &gti->slot_bit_changed);
			}
		}
		break;

	default:
		if (!goog_input_legacy_report(gti)) {
			GOOG_WARN(gti, "unexcepted input tool_type(%#x) active(%d)!\n",
				tool_type, active);
		}
		break;
	}

}
EXPORT_SYMBOL(goog_input_mt_report_slot_state);

void goog_input_report_abs(
		struct goog_touch_interface *gti,
		struct input_dev *dev, unsigned int code, int value)
{
	if (goog_input_legacy_report(gti))
		input_report_abs(dev, code, value);

	switch (code) {
	case ABS_MT_POSITION_X:
		gti->offload.coords[gti->slot].x = value;
		break;
	case ABS_MT_POSITION_Y:
		gti->offload.coords[gti->slot].y = value;
		break;
	case ABS_MT_TOUCH_MAJOR:
		gti->offload.coords[gti->slot].major = value;
		break;
	case ABS_MT_TOUCH_MINOR:
		gti->offload.coords[gti->slot].minor = value;
		break;
	case ABS_MT_PRESSURE:
		gti->offload.coords[gti->slot].pressure = value;
		break;
	case ABS_MT_ORIENTATION:
		gti->offload.coords[gti->slot].rotation = value;
		break;
	default:
		break;
	}
}
EXPORT_SYMBOL(goog_input_report_abs);

void goog_input_report_key(
		struct goog_touch_interface *gti,
		struct input_dev *dev, unsigned int code, int value)
{
	if (goog_input_legacy_report(gti))
		input_report_key(dev, code, value);
}
EXPORT_SYMBOL(goog_input_report_key);

void goog_input_sync(struct goog_touch_interface *gti, struct input_dev *dev)
{
	if (goog_input_legacy_report(gti))
		input_sync(dev);
}
EXPORT_SYMBOL(goog_input_sync);

void goog_input_release_all_fingers(struct goog_touch_interface *gti)
{
	int i;

	goog_input_lock(gti);

	goog_input_set_timestamp(gti, gti->vendor_input_dev, ktime_get());
	for (i = 0; i < MAX_SLOTS; i++) {
		goog_input_mt_slot(gti, gti->vendor_input_dev, i);
		goog_input_mt_report_slot_state(
			gti, gti->vendor_input_dev, MT_TOOL_FINGER, false);
	}
	goog_input_report_key(gti, gti->vendor_input_dev, BTN_TOUCH, 0);
	goog_input_sync(gti, gti->vendor_input_dev);

	goog_input_unlock(gti);

	goog_input_process(gti, true);
}

void goog_register_tbn(struct goog_touch_interface *gti)
{
	struct device_node *np = gti->vendor_dev->of_node;

	gti->tbn_enabled = of_property_read_bool(np, "goog,tbn-enabled");
	if (gti->tbn_enabled) {
		if (register_tbn(&gti->tbn_register_mask)) {
			GOOG_ERR(gti, "failed to register tbn context!\n");
			gti->tbn_enabled = false;
		} else {
			GOOG_INFO(gti, "tbn_register_mask = %#x.\n", gti->tbn_register_mask);
		}
	}
}

static int goog_get_context_driver_nop(
		void *private_data, struct gti_context_driver_cmd *cmd)
{
	return -ESRCH;
}

static int goog_get_context_stylus_nop(
		void *private_data, struct gti_context_stylus_cmd *cmd)
{
	return -ESRCH;
}

static int goog_get_coord_filter_enabled_nop(
		void *private_data, struct gti_coord_filter_cmd *cmd)
{
	return -ESRCH;
}

static int goog_get_fw_version_nop(
		void *private_data, struct gti_fw_version_cmd *cmd)
{
	return -ESRCH;
}

static int goog_get_grip_mode_nop(
		void *private_data, struct gti_grip_cmd *cmd)
{
	return -ESRCH;
}

static int goog_get_irq_mode_nop(
		void *private_data, struct gti_irq_cmd *cmd)
{
	return -ESRCH;
}

static int goog_get_mutual_sensor_data_nop(
		void *private_data, struct gti_sensor_data_cmd *cmd)
{
	return -ESRCH;
}

static int goog_get_palm_mode_nop(
		void *private_data, struct gti_palm_cmd *cmd)
{
	return -ESRCH;
}

static int goog_get_scan_mode_nop(
		void *private_data, struct gti_scan_cmd *cmd)
{
	return -ESRCH;
}

static int goog_get_screen_protector_mode_nop(
		void *private_data, struct gti_screen_protector_mode_cmd *cmd)
{
	return -ESRCH;
}

static int goog_get_self_sensor_data_nop(
		void *private_data, struct gti_sensor_data_cmd *cmd)
{
	return -ESRCH;
}

static int goog_get_sensing_mode_nop(
		void *private_data, struct gti_sensing_cmd *cmd)
{
	return -ESRCH;
}

static int goog_notify_display_state_nop(
		void *private_data, struct gti_display_state_cmd *cmd)
{
	return -ESRCH;
}

static int goog_notify_display_vrefresh_nop(
		void *private_data, struct gti_display_vrefresh_cmd *cmd)
{
	return -ESRCH;
}

static int goog_ping_nop(
		void *private_data, struct gti_ping_cmd *cmd)
{
	return -ESRCH;
}

static int goog_reset_nop(
		void *private_data, struct gti_reset_cmd *cmd)
{
	return -ESRCH;
}

static int goog_selftest_nop(
		void *private_data, struct gti_selftest_cmd *cmd)
{
	return -ESRCH;
}

static int goog_set_continuous_report_nop(
		void *private_data, struct gti_continuous_report_cmd *cmd)
{
	return -ESRCH;
}

static int goog_set_coord_filter_enabled_nop(
		void *private_data, struct gti_coord_filter_cmd *cmd)
{
	return -ESRCH;
}

static int goog_set_grip_mode_nop(
		void *private_data, struct gti_grip_cmd *cmd)
{
	return -ESRCH;
}

static int goog_set_heatmap_enabled_nop(
		void *private_data, struct gti_heatmap_cmd *cmd)
{
	return -ESRCH;
}

static int goog_set_irq_mode_nop(
		void *private_data, struct gti_irq_cmd *cmd)
{
	return -ESRCH;
}

static int goog_set_palm_mode_nop(
		void *private_data, struct gti_palm_cmd *cmd)
{
	return -ESRCH;
}

static int goog_set_report_rate_nop(
		void *private_data, struct gti_report_rate_cmd *cmd)
{
	return -ESRCH;
}

static int goog_set_scan_mode_nop(
		void *private_data, struct gti_scan_cmd *cmd)
{
	return -ESRCH;
}

static int goog_set_screen_protector_mode_nop(
		void *private_data, struct gti_screen_protector_mode_cmd *cmd)
{
	return -ESRCH;
}

static int goog_set_sensing_mode_nop(
		void *private_data, struct gti_sensing_cmd *cmd)
{
	return -ESRCH;
}

void goog_init_input(struct goog_touch_interface *gti)
{
	int i;

	if (!gti)
		return;

	INIT_KFIFO(gti->debug_fifo_hc);
	INIT_KFIFO(gti->debug_fifo_input);
	for (i = 0 ; i < MAX_SLOTS ; i++)
		gti->debug_input[i].slot = i;

	if (gti->vendor_dev && gti->vendor_input_dev) {
		/*
		 * Initialize the ABS_MT_ORIENTATION to support orientation reporting.
		 * Initialize the ABS_MT_TOUCH_MAJOR and ABS_MT_TOUCH_MINOR depending on
		 * the larger values of ABS_MT_POSITION_X and ABS_MT_POSITION_Y to support
		 * shape algo reporting.
		 */
		if (gti->offload.caps.rotation_reporting) {
			int abs_x_max = input_abs_get_max(gti->vendor_input_dev, ABS_MT_POSITION_X);
			int abs_x_min = input_abs_get_min(gti->vendor_input_dev, ABS_MT_POSITION_X);
			int abs_x_res = input_abs_get_res(gti->vendor_input_dev, ABS_MT_POSITION_X);
			int abs_y_max = input_abs_get_max(gti->vendor_input_dev, ABS_MT_POSITION_Y);
			int abs_y_min = input_abs_get_min(gti->vendor_input_dev, ABS_MT_POSITION_Y);
			int abs_y_res = input_abs_get_res(gti->vendor_input_dev, ABS_MT_POSITION_Y);
			int abs_major_max = abs_x_max;
			int abs_major_min = abs_x_min;
			int abs_major_res = abs_x_res;
			int abs_minor_max = abs_y_max;
			int abs_minor_min = abs_y_min;
			int abs_minor_res = abs_y_res;

			if (abs_x_max < abs_y_max) {
				swap(abs_major_max, abs_minor_max);
				swap(abs_major_min, abs_minor_min);
				swap(abs_major_res, abs_minor_res);
			}
			input_set_abs_params(gti->vendor_input_dev, ABS_MT_ORIENTATION,
				-4096, 4096, 0, 0);
			input_set_abs_params(gti->vendor_input_dev, ABS_MT_TOUCH_MAJOR,
				abs_major_min, abs_major_max, 0, 0);
			input_set_abs_params(gti->vendor_input_dev, ABS_MT_TOUCH_MINOR,
				abs_minor_min, abs_minor_max, 0, 0);
			input_abs_set_res(gti->vendor_input_dev, ABS_MT_TOUCH_MAJOR, abs_major_res);
			input_abs_set_res(gti->vendor_input_dev, ABS_MT_TOUCH_MINOR, abs_minor_res);
		}

		/*
		 * Initialize the ABS_MT_TOOL_TYPE to support touch cancel.
		 */
		input_set_abs_params(gti->vendor_input_dev, ABS_MT_TOOL_TYPE,
			MT_TOOL_FINGER, MT_TOOL_PALM, 0, 0);
	}
}

void goog_init_options(struct goog_touch_interface *gti,
		struct gti_optional_configuration *options)
{
	/* Initialize the common features. */
	gti->mf_mode = GTI_MF_MODE_DEFAULT;
	gti->screen_protector_mode_setting = GTI_SCREEN_PROTECTOR_MODE_DISABLE;
	gti->display_state = GTI_DISPLAY_STATE_ON;

	if (gti->vendor_dev) {
		struct device_node *np = gti->vendor_dev->of_node;

		gti->ignore_force_active = of_property_read_bool(np, "goog,ignore-force-active");
		gti->coord_filter_enabled = of_property_read_bool(np, "goog,coord-filter-enabled");
	}

	/* Initialize default functions. */
	gti->options.get_context_driver = goog_get_context_driver_nop;
	gti->options.get_context_stylus = goog_get_context_stylus_nop;
	gti->options.get_coord_filter_enabled = goog_get_coord_filter_enabled_nop;
	gti->options.get_fw_version = goog_get_fw_version_nop;
	gti->options.get_grip_mode = goog_get_grip_mode_nop;
	gti->options.get_irq_mode = goog_get_irq_mode_nop;
	gti->options.get_mutual_sensor_data = goog_get_mutual_sensor_data_nop;
	gti->options.get_palm_mode = goog_get_palm_mode_nop;
	gti->options.get_scan_mode = goog_get_scan_mode_nop;
	gti->options.get_screen_protector_mode = goog_get_screen_protector_mode_nop;
	gti->options.get_self_sensor_data = goog_get_self_sensor_data_nop;
	gti->options.get_sensing_mode = goog_get_sensing_mode_nop;
	gti->options.notify_display_state = goog_notify_display_state_nop;
	gti->options.notify_display_vrefresh = goog_notify_display_vrefresh_nop;
	gti->options.ping = goog_ping_nop;
	gti->options.reset = goog_reset_nop;
	gti->options.selftest = goog_selftest_nop;
	gti->options.set_continuous_report = goog_set_continuous_report_nop;
	gti->options.set_coord_filter_enabled = goog_set_coord_filter_enabled_nop;
	gti->options.set_grip_mode = goog_set_grip_mode_nop;
	gti->options.set_heatmap_enabled = goog_set_heatmap_enabled_nop;
	gti->options.set_irq_mode = goog_set_irq_mode_nop;
	gti->options.set_palm_mode = goog_set_palm_mode_nop;
	gti->options.set_report_rate = goog_set_report_rate_nop;
	gti->options.set_scan_mode = goog_set_scan_mode_nop;
	gti->options.set_screen_protector_mode = goog_set_screen_protector_mode_nop;
	gti->options.set_sensing_mode = goog_set_sensing_mode_nop;

	/* Set optional operation if available. */
	if (options) {
		if (options->get_context_driver)
			gti->options.get_context_driver = options->get_context_driver;
		if (options->get_context_stylus)
			gti->options.get_context_stylus = options->get_context_stylus;
		if (options->get_coord_filter_enabled)
			gti->options.get_coord_filter_enabled = options->get_coord_filter_enabled;
		if (options->get_fw_version)
			gti->options.get_fw_version = options->get_fw_version;
		if (options->get_grip_mode)
			gti->options.get_grip_mode = options->get_grip_mode;
		if (options->get_irq_mode)
			gti->options.get_irq_mode = options->get_irq_mode;
		if (options->get_mutual_sensor_data)
			gti->options.get_mutual_sensor_data = options->get_mutual_sensor_data;
		if (options->get_palm_mode)
			gti->options.get_palm_mode = options->get_palm_mode;
		if (options->get_scan_mode)
			gti->options.get_scan_mode = options->get_scan_mode;
		if (options->get_screen_protector_mode)
			gti->options.get_screen_protector_mode = options->get_screen_protector_mode;
		if (options->get_self_sensor_data)
			gti->options.get_self_sensor_data = options->get_self_sensor_data;
		if (options->get_sensing_mode)
			gti->options.get_sensing_mode = options->get_sensing_mode;
		if (options->notify_display_state)
			gti->options.notify_display_state = options->notify_display_state;
		if (options->notify_display_vrefresh) {
			gti->options.notify_display_vrefresh =
				options->notify_display_vrefresh;
		}
		if (options->ping)
			gti->options.ping = options->ping;
		if (options->reset)
			gti->options.reset = options->reset;
		if (options->selftest)
			gti->options.selftest = options->selftest;
		if (options->set_continuous_report)
			gti->options.set_continuous_report = options->set_continuous_report;
		if (options->set_coord_filter_enabled)
			gti->options.set_coord_filter_enabled = options->set_coord_filter_enabled;
		if (options->set_grip_mode)
			gti->options.set_grip_mode = options->set_grip_mode;
		if (options->set_heatmap_enabled)
			gti->options.set_heatmap_enabled = options->set_heatmap_enabled;
		if (options->set_irq_mode)
			gti->options.set_irq_mode = options->set_irq_mode;
		if (options->set_palm_mode)
			gti->options.set_palm_mode = options->set_palm_mode;
		if (options->set_report_rate)
			gti->options.set_report_rate = options->set_report_rate;
		if (options->set_scan_mode)
			gti->options.set_scan_mode = options->set_scan_mode;
		if (options->set_screen_protector_mode)
			gti->options.set_screen_protector_mode = options->set_screen_protector_mode;
		if (options->set_sensing_mode)
			gti->options.set_sensing_mode = options->set_sensing_mode;
	}
}

int goog_pm_wake_lock_nosync(struct goog_touch_interface *gti,
		enum gti_pm_wakelock_type type, bool skip_pm_resume)
{
	struct gti_pm* pm = NULL;

	if ((gti == NULL) || !gti->pm.enabled)
		return -ENODEV;
	pm = &gti->pm;

	mutex_lock(&pm->lock_mutex);

	if (pm->locks & type) {
		GOOG_DBG(gti, "unexpectedly lock: locks=0x%04X, type=0x%04X\n",
				pm->locks, type);
		mutex_unlock(&pm->lock_mutex);
		return -EINVAL;
	}

	/*
	 * If NON_WAKE_UP is set and the pm is suspend, we should ignore it.
	 * For example, IRQs should only keep the bus active. IRQs received
	 * while the pm is suspend should be ignored.
	 */
	if (skip_pm_resume && pm->locks == 0) {
		mutex_unlock(&pm->lock_mutex);
		return -EAGAIN;
	}

	pm->locks |= type;

	if (skip_pm_resume) {
		mutex_unlock(&pm->lock_mutex);
		return 0;
	}

	pm->new_state = GTI_PM_RESUME;
	pm->update_state = true;
	queue_work(pm->event_wq, &pm->state_update_work);
	mutex_unlock(&pm->lock_mutex);
	return 0;
}
EXPORT_SYMBOL(goog_pm_wake_lock_nosync);

int goog_pm_wake_lock(struct goog_touch_interface *gti,
		enum gti_pm_wakelock_type type, bool skip_pm_resume)
{
	struct gti_pm* pm = NULL;
	int ret = 0;

	if ((gti == NULL) || !gti->pm.enabled)
		return -ENODEV;
	pm = &gti->pm;

	ret = goog_pm_wake_lock_nosync(gti, type, skip_pm_resume);
	if (ret < 0) return ret;
	flush_workqueue(pm->event_wq);
	return ret;
}
EXPORT_SYMBOL(goog_pm_wake_lock);

int goog_pm_wake_unlock_nosync(struct goog_touch_interface *gti,
		enum gti_pm_wakelock_type type)
{
	struct gti_pm* pm = NULL;
	int ret = 0;

	if ((gti == NULL) || !gti->pm.enabled)
		return -ENODEV;
	pm = &gti->pm;

	mutex_lock(&pm->lock_mutex);

	if (!(pm->locks & type)) {
		GOOG_DBG(gti, "unexpectedly unlock: locks=0x%04X, type=0x%04X\n",
				pm->locks, type);
		mutex_unlock(&pm->lock_mutex);
		return -EINVAL;
	}

	pm->locks &= ~type;

	if (pm->locks == 0) {
		pm->new_state = GTI_PM_SUSPEND;
		pm->update_state = true;
		queue_work(pm->event_wq, &pm->state_update_work);
	}
	mutex_unlock(&pm->lock_mutex);

	return ret;
}
EXPORT_SYMBOL(goog_pm_wake_unlock_nosync);

int goog_pm_wake_unlock(struct goog_touch_interface *gti,
		enum gti_pm_wakelock_type type)
{
	struct gti_pm* pm = NULL;
	int ret = 0;

	if ((gti == NULL) || !gti->pm.enabled)
		return -ENODEV;
	pm = &gti->pm;

	ret = goog_pm_wake_unlock_nosync(gti, type);
	if (ret < 0) return ret;
	flush_workqueue(pm->event_wq);
	return ret;
}
EXPORT_SYMBOL(goog_pm_wake_unlock);

bool goog_pm_wake_check_locked(struct goog_touch_interface *gti,
		enum gti_pm_wakelock_type type)
{
	if ((gti == NULL) || !gti->pm.enabled)
		return -ENODEV;

	return gti->pm.locks & type ? true : false;
}
EXPORT_SYMBOL(goog_pm_wake_check_locked);

u32 goog_pm_wake_get_locks(struct goog_touch_interface *gti)
{
	if ((gti == NULL) || !gti->pm.enabled)
		return -ENODEV;

	return gti->pm.locks;
}
EXPORT_SYMBOL(goog_pm_wake_get_locks);

static void goog_pm_suspend(struct gti_pm *pm)
{
	struct goog_touch_interface *gti = container_of(pm,
			struct goog_touch_interface, pm);
	int ret = 0;

	/* exit directly if device is already in suspend state */
	if (pm->state == GTI_PM_SUSPEND) {
		GOOG_WARN(gti, "GTI already suspended!\n");
		return;
	}

	GOOG_INFO(gti, "irq_index: %llu, input_index: %llu.\n", gti->irq_index, gti->input_index);
	pm->state = GTI_PM_SUSPEND;

	if (pm->suspend)
		pm->suspend(gti->vendor_dev);

	if (gti->tbn_register_mask) {
		ret = tbn_release_bus(gti->tbn_register_mask);
		if (ret)
			GOOG_ERR(gti, "tbn_release_bus failed, ret %d!\n", ret);
	}
	gti_debug_hc_dump(gti);
	gti_debug_input_dump(gti);

	goog_input_release_all_fingers(gti);

	pm_relax(gti->dev);
}

static void goog_pm_resume(struct gti_pm *pm)
{
	struct goog_touch_interface *gti = container_of(pm,
			struct goog_touch_interface, pm);
	int ret = 0;

	/* exit directly if device isn't in suspend state */
	if (pm->state == GTI_PM_RESUME) {
		GOOG_WARN(gti, "GTI already resumed!\n");
		return;
	}

	pm_stay_awake(gti->dev);

	if (gti->tbn_register_mask) {
		gti->lptw_triggered = false;
		ret = tbn_request_bus_with_result(gti->tbn_register_mask, &gti->lptw_triggered);
		if (ret)
			GOOG_ERR(gti, "tbn_request_bus failed, ret %d!\n", ret);
	}

	if (pm->resume)
		pm->resume(gti->vendor_dev);

	pm->state = GTI_PM_RESUME;
}

void goog_pm_state_update_work(struct work_struct *work) {
	struct gti_pm *pm = container_of(work, struct gti_pm, state_update_work);
	enum gti_pm_state new_state;

	mutex_lock(&pm->lock_mutex);
	while (pm->update_state) {
		pm->update_state = false;
		new_state = pm->new_state;
		mutex_unlock(&pm->lock_mutex);
		if (new_state != pm->state) {
			if (new_state == GTI_PM_RESUME)
				goog_pm_resume(pm);
			else
				goog_pm_suspend(pm);
		}
		mutex_lock(&pm->lock_mutex);
	}
	mutex_unlock(&pm->lock_mutex);
}

int goog_pm_register_notification(struct goog_touch_interface *gti,
		const struct dev_pm_ops* ops)
{
	if ((gti == NULL) || !gti->pm.enabled)
		return -ENODEV;

	gti->pm.resume = ops->resume;
	gti->pm.suspend = ops->suspend;
	return 0;
}
EXPORT_SYMBOL(goog_pm_register_notification);

int goog_pm_unregister_notification(struct goog_touch_interface *gti)
{
	if ((gti == NULL) || !gti->pm.enabled)
		return -ENODEV;

	gti->pm.resume = NULL;
	gti->pm.suspend = NULL;
	return 0;
}
EXPORT_SYMBOL(goog_pm_unregister_notification);

void goog_notify_fw_status_changed(struct goog_touch_interface *gti,
		enum gti_fw_status status, struct gti_fw_status_data* data)
{
	switch (status) {
	case GTI_FW_STATUS_RESET:
		GOOG_INFO(gti, "Firmware has been reset\n");
		goog_input_release_all_fingers(gti);
		goog_update_fw_settings(gti);
		break;
	case GTI_FW_STATUS_PALM_ENTER:
		GOOG_INFO(gti, "Enter palm mode\n");
		break;
	case GTI_FW_STATUS_PALM_EXIT:
		GOOG_INFO(gti, "Exit palm mode\n");
		break;
	case GTI_FW_STATUS_GRIP_ENTER:
		GOOG_INFO(gti, "Enter grip mode\n");
		break;
	case GTI_FW_STATUS_GRIP_EXIT:
		GOOG_INFO(gti, "Exit grip mode\n");
		break;
	case GTI_FW_STATUS_WATER_ENTER:
		GOOG_INFO(gti, "Enter water mode\n");
		gti->fw_status.water_mode = 1;
		gti->context_changed.water_mode = 1;
		break;
	case GTI_FW_STATUS_WATER_EXIT:
		GOOG_INFO(gti, "Exit water mode\n");
		gti->fw_status.water_mode = 0;
		gti->context_changed.water_mode = 1;
		break;
	case GTI_FW_STATUS_NOISE_MODE:
		if (data == NULL) {
			GOOG_INFO(gti, "Noise level is changed, level: unknown\n");
		} else {
			if (data->noise_level == GTI_NOISE_MODE_EXIT) {
				GOOG_INFO(gti, "Exit noise mode\n");
				gti->fw_status.noise_level= 0;
			} else {
				GOOG_INFO(gti, "Enter noise mode, level: %d\n", data->noise_level);
				gti->fw_status.noise_level = data->noise_level;
			}
			gti->context_changed.noise_state = 1;
		}
		break;
	default:
		break;
	}
}
EXPORT_SYMBOL(goog_notify_fw_status_changed);

static int goog_pm_probe(struct goog_touch_interface *gti)
{
	struct gti_pm* pm = &gti->pm;
	int ret = 0;

	pm->state = GTI_PM_RESUME;
	pm->locks = GTI_PM_WAKELOCK_TYPE_SCREEN_ON;
	pm->event_wq = alloc_workqueue(
		"gti_pm_wq", WQ_UNBOUND | WQ_HIGHPRI | WQ_CPU_INTENSIVE, 1);
	if (!pm->event_wq) {
		GOOG_ERR(gti, "Failed to create work thread for pm!\n");
		ret = -ENOMEM;
		goto err_alloc_workqueue;
	}

	mutex_init(&pm->lock_mutex);
	INIT_WORK(&pm->state_update_work, goog_pm_state_update_work);

	/* init pm_qos. */
	cpu_latency_qos_add_request(&gti->pm_qos_req, PM_QOS_DEFAULT_VALUE);
	pm->enabled = true;

err_alloc_workqueue:
	return ret;
}

static int goog_pm_remove(struct goog_touch_interface *gti)
{
	struct gti_pm* pm = &gti->pm;

	if (pm->enabled) {
		pm->enabled = false;
		cpu_latency_qos_remove_request(&gti->pm_qos_req);
		if (pm->event_wq)
			destroy_workqueue(pm->event_wq);
	}

	return 0;
}

static void goog_lookup_touch_report_rate(struct goog_touch_interface *gti)
{
	int i;
	u32 next_report_rate = 0;

	for (i = 0; i < gti->report_rate_table_size; i++) {
		if (gti->display_vrefresh <= gti->display_refresh_rate_table[i]) {
			next_report_rate = gti->touch_report_rate_table[i];
			break;
		}
	}

	/*
	 * Set the touch report as minimum value if the display_vrefresh is smaller
	 * than the minimum value of goog,display-vrr-table.
	 */
	if (next_report_rate == 0)
		next_report_rate = gti->touch_report_rate_table[0];

	if (gti->report_rate_setting_next != next_report_rate) {
		cancel_delayed_work_sync(&gti->set_report_rate_work);
		gti->report_rate_setting_next = next_report_rate;
	}

	if (gti->report_rate_setting_next != gti->report_rate_setting &&
			gti->pm.state == GTI_PM_RESUME) {
		queue_delayed_work(gti->pm.event_wq, &gti->set_report_rate_work,
				(gti->report_rate_setting_next > gti->report_rate_setting) ?
				msecs_to_jiffies(gti->increase_report_rate_delay * MSEC_PER_SEC) :
				msecs_to_jiffies(gti->decrease_report_rate_delay * MSEC_PER_SEC));
	}
}

static void goog_set_report_rate_work(struct work_struct *work)
{
	int ret;
	struct goog_touch_interface *gti;
	struct delayed_work *delayed_work;
	delayed_work = container_of(work, struct delayed_work, work);
	gti = container_of(delayed_work, struct goog_touch_interface, set_report_rate_work);

	if (gti->pm.state == GTI_PM_SUSPEND)
		return;

	if (gti->report_rate_setting == gti->report_rate_setting_next)
		return;

	/* Retry it 10ms later if there is finger on the screen. */
	if (gti->slot_bit_active) {
		queue_delayed_work(gti->pm.event_wq, &gti->set_report_rate_work,
				msecs_to_jiffies(10));
		return;
	}

	gti->cmd.report_rate_cmd.setting = gti->report_rate_setting_next;
	ret = goog_process_vendor_cmd(gti, GTI_CMD_SET_REPORT_RATE);
	if (ret != 0) {
		GOOG_ERR(gti, "Failed to set report rate!\n");
		return;
	}

	gti->report_rate_setting = gti->report_rate_setting_next;
	gti->context_changed.touch_report_rate = 1;
}

static int goog_init_variable_report_rate(struct goog_touch_interface *gti)
{
	int table_size = 0;

	if (!gti->pm.event_wq) {
		GOOG_ERR(gti, "No workqueue for variable report rate.\n");
		return -ENODEV;
	}

	gti->vrr_enabled = of_property_read_bool(gti->vendor_dev->of_node,
			"goog,vrr-enabled");
	if (!gti->vrr_enabled)
		return 0;

	table_size = of_property_count_u32_elems(gti->vendor_dev->of_node,
			"goog,vrr-display-rate");
	if (table_size != of_property_count_u32_elems(gti->vendor_dev->of_node,
			"goog,vrr-touch-rate")) {
		GOOG_ERR(gti, "Table size mismatch!\n");
		goto init_variable_report_rate_failed;
	}

	gti->report_rate_table_size = table_size;

	gti->display_refresh_rate_table = devm_kzalloc(gti->vendor_dev,
			sizeof(u32) * table_size, GFP_KERNEL);
	if (!gti->display_refresh_rate_table) {
		GOOG_ERR(gti, "display_refresh_rate_table alloc failed.\n");
		goto init_variable_report_rate_failed;
	}

	gti->touch_report_rate_table = devm_kzalloc(gti->vendor_dev,
			sizeof(u32) * table_size, GFP_KERNEL);
	if (!gti->touch_report_rate_table) {
		GOOG_ERR(gti, "touch_report_rate_table alloc failed.\n");
		goto init_variable_report_rate_failed;
	}

	if (of_property_read_u32_array(gti->vendor_dev->of_node, "goog,vrr-display-rate",
			gti->display_refresh_rate_table, table_size)) {
		GOOG_ERR(gti, "Failed to parse goog,display-vrr-table.\n");
		goto init_variable_report_rate_failed;
	}

	if (of_property_read_u32_array(gti->vendor_dev->of_node, "goog,vrr-touch-rate",
			gti->touch_report_rate_table, table_size)) {
		GOOG_ERR(gti, "Failed to parse goog,touch-vrr-table.\n");
		goto init_variable_report_rate_failed;
	}

	if (of_property_read_u32(gti->vendor_dev->of_node, "goog,vrr-up-delay",
			&gti->increase_report_rate_delay)) {
		gti->increase_report_rate_delay = 0;
	}

	if (of_property_read_u32(gti->vendor_dev->of_node, "goog,vrr-down-delay",
			&gti->decrease_report_rate_delay)) {
		gti->decrease_report_rate_delay = 0;
	}

	GOOG_INFO(gti, "Default report rate: %uHz, report rate delay %u/%u)",
			gti->touch_report_rate_table[0],
			gti->increase_report_rate_delay,
			gti->decrease_report_rate_delay);

	gti->report_rate_setting = gti->touch_report_rate_table[0];
	gti->report_rate_setting_next = gti->touch_report_rate_table[0];
	INIT_DELAYED_WORK(&gti->set_report_rate_work, goog_set_report_rate_work);

	return 0;

init_variable_report_rate_failed:
	gti->vrr_enabled = false;
	devm_kfree(gti->vendor_dev, gti->display_refresh_rate_table);
	devm_kfree(gti->vendor_dev, gti->touch_report_rate_table);

	return 0;
}

int goog_get_lptw_triggered(struct goog_touch_interface *gti)
{
	if (gti == NULL)
		return -ENODEV;

	return gti->lptw_triggered;
}
EXPORT_SYMBOL(goog_get_lptw_triggered);

static irqreturn_t gti_irq_handler(int irq, void *data)
{
	irqreturn_t ret;
	struct goog_touch_interface *gti = (struct goog_touch_interface *)data;

	gti->irq_index++;
	if (gti->vendor_irq_handler)
		ret = gti->vendor_irq_handler(irq, gti->vendor_irq_cookie);
	else
		ret = IRQ_WAKE_THREAD;
	gti_debug_hc_update(gti, true);
	return ret;
}

static irqreturn_t gti_irq_thread_fn(int irq, void *data)
{
	int error;
	irqreturn_t ret = IRQ_NONE;
	struct goog_touch_interface *gti = (struct goog_touch_interface *)data;

	ATRACE_BEGIN(__func__);

	if (gti->tbn_enabled) {
		error = goog_pm_wake_lock(gti, GTI_PM_WAKELOCK_TYPE_IRQ, true);
		if (error < 0) {
			GOOG_WARN(gti, "Skipping stray interrupt, pm state: (%d, %d)\n",
					gti->pm.state, gti->pm.new_state);
			ATRACE_END();
			return IRQ_HANDLED;
		}
	}

	cpu_latency_qos_update_request(&gti->pm_qos_req, 100 /* usec */);

	/*
	 * Some vendor drivers read sensor data inside vendor_irq_thread_fn.
	 * We need to lock input_process_lock before vendor_irq_thread_fn to
	 * avoid thread safe issue.
	 */
	mutex_lock(&gti->input_process_lock);

	if (gti->vendor_irq_thread_fn)
		ret = gti->vendor_irq_thread_fn(irq, gti->vendor_irq_cookie);
	else
		ret = IRQ_HANDLED;

	goog_input_process(gti, false);

	mutex_unlock(&gti->input_process_lock);

	gti_debug_hc_update(gti, false);
	cpu_latency_qos_update_request(&gti->pm_qos_req, PM_QOS_DEFAULT_VALUE);
	if (gti->tbn_enabled)
		goog_pm_wake_unlock_nosync(gti, GTI_PM_WAKELOCK_TYPE_IRQ);
	ATRACE_END();

	return ret;
}

int goog_devm_request_threaded_irq(struct goog_touch_interface *gti,
		struct device *dev, unsigned int irq,
		irq_handler_t handler, irq_handler_t thread_fn,
		unsigned long irqflags, const char *devname,
		void *dev_id)
{
	int ret;

	if (gti) {
		ret = devm_request_threaded_irq(dev, irq, gti_irq_handler, gti_irq_thread_fn,
				irqflags, devname, gti);
		if (dev_id)
			gti->vendor_irq_cookie = dev_id;
		if (handler)
			gti->vendor_irq_handler = handler;
		if (thread_fn)
			gti->vendor_irq_thread_fn = thread_fn;
	} else {
		ret = devm_request_threaded_irq(dev, irq, handler, thread_fn,
				irqflags, devname, dev_id);
	}

	return ret;
}
EXPORT_SYMBOL(goog_devm_request_threaded_irq);

int goog_request_threaded_irq(struct goog_touch_interface *gti,
		unsigned int irq, irq_handler_t handler, irq_handler_t thread_fn,
		unsigned long irqflags, const char *devname, void *dev_id)
{
	int ret;

	if (gti) {
		ret = request_threaded_irq(irq, gti_irq_handler, gti_irq_thread_fn,
				irqflags, devname, gti);
		if (dev_id)
			gti->vendor_irq_cookie = dev_id;
		if (handler)
			gti->vendor_irq_handler = handler;
		if (thread_fn)
			gti->vendor_irq_thread_fn = thread_fn;
	} else {
		ret = request_threaded_irq(irq, handler, thread_fn, irqflags, devname, dev_id);
	}

	return ret;
}
EXPORT_SYMBOL(goog_request_threaded_irq);

struct goog_touch_interface *goog_touch_interface_probe(
		void *private_data,
		struct device *dev,
		struct input_dev *input_dev,
		int (*default_handler)(void *private_data,
			u32 cmd_type, struct gti_union_cmd_data *cmd),
		struct gti_optional_configuration *options)
{
	int ret;
	struct goog_touch_interface *gti;

	if (!dev || !input_dev || !default_handler) {
		pr_err("%s: error: invalid dev/input_dev or default_handler!\n", __func__);
		return NULL;
	}

	gti = devm_kzalloc(dev, sizeof(struct goog_touch_interface), GFP_KERNEL);
	if (gti) {
		gti->vendor_private_data = private_data;
		gti->vendor_dev = dev;
		gti->vendor_input_dev = input_dev;
		gti->vendor_default_handler = default_handler;
		mutex_init(&gti->input_lock);
		mutex_init(&gti->input_process_lock);
	}

	if (!gti_class)
		gti_class = class_create(THIS_MODULE, GTI_NAME);

	if (gti && gti_class) {
		u32 dev_id = gti_dev_num;
		char *name;

		if (gti->vendor_dev) {
			struct device_node *np = gti->vendor_dev->of_node;

			of_property_read_u32(np, "goog,dev-id", &dev_id);
		}
		name = kasprintf(GFP_KERNEL, "gti.%d", dev_id);

		if (name &&
			!alloc_chrdev_region(&gti->dev_id, 0, 1, name)) {
			gti->dev = device_create(gti_class, NULL,
					gti->dev_id, gti, name);
			if (gti->dev) {
				gti_dev_num++;
				GOOG_INFO(gti, "device create \"%s\".\n", name);
				if (gti->vendor_dev) {
					ret = sysfs_create_link(&gti->dev->kobj,
						&gti->vendor_dev->kobj, "vendor");
					if (ret) {
						GOOG_ERR(gti, "sysfs_create_link() failed for vendor, ret=%d!\n",
							ret);
					}
				}
				if (gti->vendor_input_dev) {
					ret = sysfs_create_link(&gti->dev->kobj,
						&gti->vendor_input_dev->dev.kobj, "vendor_input");
					if (ret) {
						GOOG_ERR(gti, "sysfs_create_link() failed for vendor_input, ret=%d!\n",
							 ret);
					}
				}
			}
		}
		kfree(name);
	}

	if (gti && gti->dev) {
		goog_init_proc(gti);
		goog_init_options(gti, options);
		goog_offload_probe(gti);
		/*
		 * goog_init_input() needs the offload.cap initialization by goog_offload_probe().
		 */
		goog_init_input(gti);
		goog_register_tbn(gti);
		goog_pm_probe(gti);
		register_panel_bridge(gti);
		goog_init_variable_report_rate(gti);
		goog_update_fw_settings(gti);

		ret = sysfs_create_group(&gti->dev->kobj, &goog_attr_group);
		if (ret)
			GOOG_ERR(gti, "sysfs_create_group() failed, ret= %d!\n", ret);
	}

	return gti;
}
EXPORT_SYMBOL(goog_touch_interface_probe);

int goog_touch_interface_remove(struct goog_touch_interface *gti)
{
	if (!gti)
		return -ENODEV;

	if (gti->dev) {
		sysfs_remove_group(&gti->dev->kobj, &goog_attr_group);
		if (gti->vendor_dev)
			sysfs_remove_link(&gti->dev->kobj, "vendor");
		if (gti->vendor_input_dev)
			sysfs_remove_link(&gti->dev->kobj, "vendor_input");
		device_destroy(gti_class, gti->dev_id);
		gti->dev = NULL;
		gti_dev_num--;
	}

	if (gti_class) {
		unregister_chrdev_region(gti->dev_id, 1);
		if (!gti_dev_num) {
			proc_remove(gti_proc_dir_root);
			gti_proc_dir_root = NULL;
			class_destroy(gti_class);
			gti_class = NULL;
		}
	}

	unregister_panel_bridge(&gti->panel_bridge);
	goog_pm_remove(gti);

	if (gti->tbn_enabled && gti->tbn_register_mask)
		unregister_tbn(&gti->tbn_register_mask);

	gti->offload_enabled = false;
	gti->v4l2_enabled = false;
	goog_offload_remove(gti);
	heatmap_remove(&gti->v4l2);
	devm_kfree(gti->vendor_dev, gti->heatmap_buf);
	devm_kfree(gti->vendor_dev, gti);

	return 0;
}
EXPORT_SYMBOL(goog_touch_interface_remove);

MODULE_DESCRIPTION("Google Touch Interface");
MODULE_AUTHOR("Super Liu<supercjliu@google.com>");
MODULE_LICENSE("GPL v2");
