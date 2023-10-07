// SPDX-License-Identifier: GPL-2.0
//
// cs40l26-debugfs.c -- CS40L26 Boosted Haptic Driver with Integrated DSP and
// Waveform Memory with Advanced Closed Loop Algorithms and LRA protection
//
// Copyright 2022 Cirrus Logic, Inc.
//
// Author: Fred Treven <fred.treven@cirrus.com>
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License version 2 as
// published by the Free Software Foundation.

#include "cs40l26.h"

#ifdef CONFIG_DEBUG_FS
static ssize_t cs40l26_fw_ctrl_name_read(struct file *file,
		char __user *user_buf, size_t count, loff_t *ppos)
{
	struct cs40l26_private *cs40l26 = file->private_data;
	ssize_t ret = 0;

	mutex_lock(&cs40l26->lock);

	if (cs40l26->dbg_fw_ctrl_name)
		ret = simple_read_from_buffer(user_buf, count, ppos,
				cs40l26->dbg_fw_ctrl_name,
				strlen(cs40l26->dbg_fw_ctrl_name));

	mutex_unlock(&cs40l26->lock);

	return ret;
}

static ssize_t cs40l26_fw_ctrl_name_write(struct file *file,
		const char __user *user_buf, size_t count, loff_t *ppos)
{
	struct cs40l26_private *cs40l26 = file->private_data;
	ssize_t ret = 0;

	mutex_lock(&cs40l26->lock);

	kfree(cs40l26->dbg_fw_ctrl_name);
	cs40l26->dbg_fw_ctrl_name = NULL;

	cs40l26->dbg_fw_ctrl_name = kzalloc(count, GFP_KERNEL);
	if (!cs40l26->dbg_fw_ctrl_name) {
		ret = -ENOMEM;
		goto err_mutex;
	}

	ret = simple_write_to_buffer(cs40l26->dbg_fw_ctrl_name,
			count, ppos, user_buf, count);

err_mutex:
	mutex_unlock(&cs40l26->lock);

	return ret ? ret : count;
}

static ssize_t cs40l26_fw_algo_id_read(struct file *file,
		char __user *user_buf, size_t count, loff_t *ppos)
{
	struct cs40l26_private *cs40l26 = file->private_data;
	ssize_t ret;
	char *str;

	str = kzalloc(CS40L26_ALGO_ID_MAX_STR_LEN, GFP_KERNEL);
	if (!str)
		return -ENOMEM;

	mutex_lock(&cs40l26->lock);

	snprintf(str, count, "0x%06X\n", cs40l26->dbg_fw_algo_id);

	mutex_unlock(&cs40l26->lock);

	ret = simple_read_from_buffer(user_buf, count, ppos, str, strlen(str));

	kfree(str);

	return ret;
}

static ssize_t cs40l26_fw_algo_id_write(struct file *file,
		const char __user *user_buf, size_t count, loff_t *ppos)
{
	struct cs40l26_private *cs40l26 = file->private_data;
	ssize_t ret;
	char *str;
	u32 val;

	str = kzalloc(count, GFP_KERNEL);
	if (!str)
		return -ENOMEM;

	simple_write_to_buffer(str, count, ppos, user_buf, count);

	ret = kstrtou32(str, 16, &val);
	if (ret)
		goto exit_free;

	mutex_lock(&cs40l26->lock);

	cs40l26->dbg_fw_algo_id = val;

	mutex_unlock(&cs40l26->lock);

exit_free:

	kfree(str);

	return ret ? ret : count;
}

static ssize_t cs40l26_fw_ctrl_val_read(struct file *file,
		char __user *user_buf, size_t count, loff_t *ppos)
{
	struct cs40l26_private *cs40l26 = file->private_data;
	u32 reg, val, mem_type;
	char *result, *input;
	ssize_t ret;

	if (!cs40l26->dbg_fw_ctrl_name || !cs40l26->dbg_fw_algo_id)
		return -ENODEV;

	if (strlen(cs40l26->dbg_fw_ctrl_name) == 0)
		return -ENODATA;

	ret = pm_runtime_get_sync(cs40l26->dev);
	if (ret < 0) {
		cs40l26_resume_error_handle(cs40l26->dev, (int) ret);
		return ret;
	}

	mutex_lock(&cs40l26->lock);

	mem_type  = cs40l26->dbg_fw_ym ?
			CL_DSP_YM_UNPACKED_TYPE : CL_DSP_XM_UNPACKED_TYPE;

	input = kzalloc(strlen(cs40l26->dbg_fw_ctrl_name), GFP_KERNEL);
	if (!input) {
		ret = -ENOMEM;
		goto err_mutex;
	}

	snprintf(input, strlen(cs40l26->dbg_fw_ctrl_name), "%s",
			cs40l26->dbg_fw_ctrl_name);

	ret = cl_dsp_get_reg(cs40l26->dsp, input, mem_type,
			cs40l26->dbg_fw_algo_id, &reg);
	kfree(input);
	if (ret)
		goto err_mutex;

	ret = regmap_read(cs40l26->regmap, reg, &val);
	if (ret) {
		dev_err(cs40l26->dev, "Failed to read fw control\n");
		goto err_mutex;
	}

	result = kzalloc(CS40L26_ALGO_ID_MAX_STR_LEN, GFP_KERNEL);
	if (!result) {
		ret = -ENOMEM;
		goto err_mutex;
	}

	snprintf(result, CS40L26_ALGO_ID_MAX_STR_LEN, "0x%08X\n", val);
	ret = simple_read_from_buffer(user_buf, count, ppos, result,
			strlen(result));

	kfree(result);

err_mutex:
	mutex_unlock(&cs40l26->lock);

	pm_runtime_mark_last_busy(cs40l26->dev);
	pm_runtime_put_autosuspend(cs40l26->dev);

	return ret;
}

static const struct {
	const char *name;
	const struct file_operations fops;
} cs40l26_debugfs_fops[] = {
	{
		.name = "fw_ctrl_name",
		.fops = {
			.open = simple_open,
			.read = cs40l26_fw_ctrl_name_read,
			.write = cs40l26_fw_ctrl_name_write,
		},
	},
	{
		.name = "fw_algo_id",
		.fops = {
			.open = simple_open,
			.read = cs40l26_fw_algo_id_read,
			.write = cs40l26_fw_algo_id_write,
		},
	},
	{
		.name = "fw_ctrl_val",
		.fops = {
			.open = simple_open,
			.read = cs40l26_fw_ctrl_val_read,
		},
	},
};

void cs40l26_debugfs_init(struct cs40l26_private *cs40l26)
{
	struct dentry *root = NULL;
	int i;

	cs40l26_debugfs_cleanup(cs40l26);

	root = debugfs_create_dir("cs40l26", NULL);
	if (!root)
		return;

	debugfs_create_bool("fw_ym_space", CL_DSP_DEBUGFS_RW_FILE_MODE,
			root, &cs40l26->dbg_fw_ym);

	for (i = 0; i < CS40L26_NUM_DEBUGFS; i++)
		debugfs_create_file(cs40l26_debugfs_fops[i].name,
				CL_DSP_DEBUGFS_RW_FILE_MODE, root, cs40l26,
				&cs40l26_debugfs_fops[i].fops);

	cs40l26->dbg_fw_ym = false;
	cs40l26->dbg_fw_algo_id = CS40L26_VIBEGEN_ALGO_ID;
	cs40l26->debugfs_root = root;

	if (cs40l26->fw_id == CS40L26_FW_ID &&
			cl_dsp_algo_is_present(cs40l26->dsp, CS40L26_EVENT_LOGGER_ALGO_ID)) {
		cs40l26->cl_dsp_db = cl_dsp_debugfs_create(cs40l26->dsp,
				cs40l26->debugfs_root,
				(u32) CS40L26_EVENT_LOGGER_ALGO_ID);

		if (IS_ERR(cs40l26->cl_dsp_db) || !cs40l26->cl_dsp_db)
			dev_err(cs40l26->dev, "Failed to create CL DSP Debugfs\n");
	}
}
EXPORT_SYMBOL(cs40l26_debugfs_init);

void cs40l26_debugfs_cleanup(struct cs40l26_private *cs40l26)
{
	cl_dsp_debugfs_destroy(cs40l26->cl_dsp_db);
	cs40l26->cl_dsp_db = NULL;
	kfree(cs40l26->dbg_fw_ctrl_name);
	cs40l26->dbg_fw_ctrl_name = NULL;
	debugfs_remove_recursive(cs40l26->debugfs_root);
}
EXPORT_SYMBOL(cs40l26_debugfs_cleanup);

#endif /* CONFIG_DEBUG_FS */
