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
#include "goodix_ts_core.h"
#include <linux/atomic.h>
#include <linux/fs.h>
#include <linux/ioctl.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/wait.h>

#define GOODIX_TOOLS_NAME			"gtp_tools"
#define GOODIX_TOOLS_VER_MAJOR		1
#define GOODIX_TOOLS_VER_MINOR		0
static const u16 goodix_tools_ver =
	((GOODIX_TOOLS_VER_MAJOR << 8) + (GOODIX_TOOLS_VER_MINOR));

#define GOODIX_TS_IOC_MAGIC 'G'
#define NEGLECT_SIZE_MASK (~(_IOC_SIZEMASK << _IOC_SIZESHIFT))

#define GTP_IRQ_ENABLE _IO(GOODIX_TS_IOC_MAGIC, 0)
#define GTP_DEV_RESET _IO(GOODIX_TS_IOC_MAGIC, 1)
#define GTP_SEND_COMMAND (_IOW(GOODIX_TS_IOC_MAGIC, 2, u8) & NEGLECT_SIZE_MASK)
#define GTP_SEND_CONFIG (_IOW(GOODIX_TS_IOC_MAGIC, 3, u8) & NEGLECT_SIZE_MASK)
#define GTP_ASYNC_READ (_IOR(GOODIX_TS_IOC_MAGIC, 4, u8) & NEGLECT_SIZE_MASK)
#define GTP_SYNC_READ (_IOR(GOODIX_TS_IOC_MAGIC, 5, u8) & NEGLECT_SIZE_MASK)
#define GTP_ASYNC_WRITE (_IOW(GOODIX_TS_IOC_MAGIC, 6, u8) & NEGLECT_SIZE_MASK)
#define GTP_READ_CONFIG (_IOW(GOODIX_TS_IOC_MAGIC, 7, u8) & NEGLECT_SIZE_MASK)
#define GTP_ESD_ENABLE _IO(GOODIX_TS_IOC_MAGIC, 8)
#define GTP_TOOLS_VER (_IOR(GOODIX_TS_IOC_MAGIC, 9, u8) & NEGLECT_SIZE_MASK)
#define GTP_TOOLS_CTRL_SYNC                                                    \
	(_IOW(GOODIX_TS_IOC_MAGIC, 10, u8) & NEGLECT_SIZE_MASK)

#define MAX_BUF_LENGTH (16 * 1024)

#define I2C_MSG_HEAD_LEN 20

/* read data asynchronous,
 * success return data length, otherwise return < 0
 */
static int async_read(struct goodix_tools_dev *dev, void __user *arg)
{
	u8 *databuf = NULL;
	int ret = 0;
	u32 reg_addr, length;
	u8 i2c_msg_head[I2C_MSG_HEAD_LEN];
	struct goodix_ts_core *ts_core =
			container_of(dev, struct goodix_ts_core, tools_dev);
	const struct goodix_ts_hw_ops *hw_ops = ts_core->hw_ops;

	ret = copy_from_user(&i2c_msg_head, arg, I2C_MSG_HEAD_LEN);
	if (ret)
		return -EFAULT;

	reg_addr = i2c_msg_head[0] + (i2c_msg_head[1] << 8) +
		   (i2c_msg_head[2] << 16) + (i2c_msg_head[3] << 24);
	length = i2c_msg_head[4] + (i2c_msg_head[5] << 8) +
		 (i2c_msg_head[6] << 16) + (i2c_msg_head[7] << 24);
	if (length > MAX_BUF_LENGTH) {
		ts_err("buffer too long:%d > %d", length, MAX_BUF_LENGTH);
		return -EINVAL;
	}
	databuf = kzalloc(length, GFP_KERNEL);
	if (!databuf) {
		ts_err("Alloc memory failed");
		return -ENOMEM;
	}

	if (hw_ops->read(ts_core, reg_addr, databuf, length)) {
		ret = -EBUSY;
		ts_err("Read i2c failed");
		goto err_out;
	}
	ret = copy_to_user((u8 *)arg + I2C_MSG_HEAD_LEN, databuf, length);
	if (ret) {
		ret = -EFAULT;
		ts_err("Copy_to_user failed");
		goto err_out;
	}
	ret = length;
err_out:
	kfree(databuf);
	return ret;
}

/* if success return config data length */
static int read_config_data(struct goodix_ts_core *ts_core, void __user *arg)
{
	int ret = 0;
	u32 reg_addr, length;
	u8 i2c_msg_head[I2C_MSG_HEAD_LEN];
	u8 *tmp_buf;

	ret = copy_from_user(&i2c_msg_head, arg, I2C_MSG_HEAD_LEN);
	if (ret) {
		ts_err("Copy data from user failed");
		return -EFAULT;
	}
	reg_addr = i2c_msg_head[0] + (i2c_msg_head[1] << 8) +
		   (i2c_msg_head[2] << 16) + (i2c_msg_head[3] << 24);
	length = i2c_msg_head[4] + (i2c_msg_head[5] << 8) +
		 (i2c_msg_head[6] << 16) + (i2c_msg_head[7] << 24);
	ts_info("read config,reg_addr=0x%x, length=%d", reg_addr, length);
	if (length > MAX_BUF_LENGTH) {
		ts_err("buffer too long:%d > %d", length, MAX_BUF_LENGTH);
		return -EINVAL;
	}
	tmp_buf = kzalloc(length, GFP_KERNEL);
	if (!tmp_buf) {
		ts_err("failed alloc memory");
		return -ENOMEM;
	}
	/* if reg_addr == 0, read config data with specific flow */
	if (!reg_addr) {
		if (ts_core->hw_ops->read_config)
			ret = ts_core->hw_ops->read_config(
				ts_core, tmp_buf, length);
		else
			ret = -EINVAL;
	} else {
		ret = ts_core->hw_ops->read(ts_core, reg_addr, tmp_buf, length);
		if (!ret)
			ret = length;
	}
	if (ret <= 0)
		goto err_out;

	if (copy_to_user((u8 *)arg + I2C_MSG_HEAD_LEN, tmp_buf, ret)) {
		ret = -EFAULT;
		ts_err("Copy_to_user failed");
	}

err_out:
	kfree(tmp_buf);
	return ret;
}

/* write data to i2c asynchronous,
 * success return bytes write, else return <= 0
 */
static int async_write(struct goodix_tools_dev *dev, void __user *arg)
{
	u8 *databuf;
	int ret = 0;
	u32 reg_addr, length;
	u8 i2c_msg_head[I2C_MSG_HEAD_LEN];
	struct goodix_ts_core *ts_core =
			container_of(dev, struct goodix_ts_core, tools_dev);
	const struct goodix_ts_hw_ops *hw_ops = ts_core->hw_ops;

	ret = copy_from_user(&i2c_msg_head, arg, I2C_MSG_HEAD_LEN);
	if (ret) {
		ts_err("Copy data from user failed");
		return -EFAULT;
	}
	reg_addr = i2c_msg_head[0] + (i2c_msg_head[1] << 8) +
		   (i2c_msg_head[2] << 16) + (i2c_msg_head[3] << 24);
	length = i2c_msg_head[4] + (i2c_msg_head[5] << 8) +
		 (i2c_msg_head[6] << 16) + (i2c_msg_head[7] << 24);
	if (length > MAX_BUF_LENGTH) {
		ts_err("buffer too long:%d > %d", length, MAX_BUF_LENGTH);
		return -EINVAL;
	}

	databuf = kzalloc(length, GFP_KERNEL);
	if (!databuf) {
		ts_err("Alloc memory failed");
		return -ENOMEM;
	}
	ret = copy_from_user(databuf, (u8 *)arg + I2C_MSG_HEAD_LEN, length);
	if (ret) {
		ret = -EFAULT;
		ts_err("Copy data from user failed");
		goto err_out;
	}

	if (hw_ops->write(ts_core, reg_addr, databuf, length)) {
		ret = -EBUSY;
		ts_err("Write data to device failed");
	} else {
		ret = length;
	}

err_out:
	kfree(databuf);
	return ret;
}

static int init_cfg_data(struct goodix_ic_config *cfg, void __user *arg)
{
	int ret = 0;
	u32 length;
	u8 i2c_msg_head[I2C_MSG_HEAD_LEN] = { 0 };

	ret = copy_from_user(&i2c_msg_head, arg, I2C_MSG_HEAD_LEN);
	if (ret) {
		ts_err("Copy data from user failed");
		return -EFAULT;
	}

	length = i2c_msg_head[4] + (i2c_msg_head[5] << 8) +
		 (i2c_msg_head[6] << 16) + (i2c_msg_head[7] << 24);
	if (length > GOODIX_CFG_MAX_SIZE) {
		ts_err("buffer too long:%d > %d", length, MAX_BUF_LENGTH);
		return -EINVAL;
	}
	ret = copy_from_user(cfg->data, (u8 *)arg + I2C_MSG_HEAD_LEN, length);
	if (ret) {
		ts_err("Copy data from user failed");
		return -EFAULT;
	}
	cfg->len = length;
	return 0;
}

/**
 * goodix_tools_ioctl - ioctl implementation
 *
 * @filp: Pointer to file opened
 * @cmd: Ioctl opertion command
 * @arg: Command data
 * Returns >=0 - succeed, else failed
 */
static long goodix_tools_ioctl(
	struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	struct goodix_tools_dev *dev = filp->private_data;
	struct goodix_ts_core *ts_core =
			container_of(dev, struct goodix_ts_core, tools_dev);
	const struct goodix_ts_hw_ops *hw_ops;
	struct goodix_ic_config *temp_cfg = NULL;

	if (ts_core == NULL) {
		ts_err("Tools module not register");
		return -EINVAL;
	}
	hw_ops = ts_core->hw_ops;

	if (_IOC_TYPE(cmd) != GOODIX_TS_IOC_MAGIC) {
		ts_err("Bad magic num:%c", _IOC_TYPE(cmd));
		return -ENOTTY;
	}

	switch (cmd & NEGLECT_SIZE_MASK) {
	case GTP_IRQ_ENABLE:
		if (arg == 1) {
			hw_ops->irq_enable(ts_core, true);
			ts_info("IRQ enabled");
		} else if (arg == 0) {
			hw_ops->irq_enable(ts_core, false);
			ts_info("IRQ disabled");
		} else {
			ts_info("Irq aready set with, arg = %ld", arg);
		}
		ret = 0;
		break;
	case GTP_ESD_ENABLE:
		if (arg == 0)
			goodix_ts_esd_off(ts_core);
		else
			goodix_ts_esd_on(ts_core);
		break;
	case GTP_DEV_RESET:
		hw_ops->reset(ts_core, goodix_get_normal_reset_delay(ts_core));
		break;
	case GTP_SEND_COMMAND:
		/* deprecated command */
		ts_err("the GTP_SEND_COMMAND function has been removed");
		ret = -EINVAL;
		break;
	case GTP_SEND_CONFIG:
		temp_cfg = kzalloc(sizeof(struct goodix_ic_config), GFP_KERNEL);
		if (temp_cfg == NULL) {
			ts_err("Memory allco err");
			ret = -ENOMEM;
			goto err_out;
		}
		ret = init_cfg_data(temp_cfg, (void __user *)arg);
		if (!ret && hw_ops->send_config) {
			ret = hw_ops->send_config(
				ts_core, temp_cfg->data, temp_cfg->len);
			if (ret) {
				ts_err("Failed send config");
				ret = -EAGAIN;
			} else {
				ts_info("Send config success");
				ret = 0;
			}
		}
		kfree(temp_cfg);
		temp_cfg = NULL;
		break;
	case GTP_READ_CONFIG:
		ret = read_config_data(ts_core, (void __user *)arg);
		if (ret > 0)
			ts_info("success read config:len=%d", ret);
		else
			ts_err("failed read config:ret=0x%x", ret);
		break;
	case GTP_ASYNC_READ:
		ret = async_read(dev, (void __user *)arg);
		if (ret < 0)
			ts_err("Async data read failed");
		break;
	case GTP_SYNC_READ:
		ts_info("unsupport sync read");
		break;
	case GTP_ASYNC_WRITE:
		ret = async_write(dev, (void __user *)arg);
		if (ret < 0)
			ts_err("Async data write failed");
		break;
	case GTP_TOOLS_VER:
		ret = copy_to_user((u8 *)arg, &goodix_tools_ver, sizeof(u16));
		if (ret)
			ts_err("failed copy driver version info to user");
		break;
	case GTP_TOOLS_CTRL_SYNC:
		ts_core->tools_ctrl_sync = !!arg;
		ts_info("set tools ctrl sync %d", ts_core->tools_ctrl_sync);
		break;
	default:
		ts_info("Invalid cmd");
		ret = -ENOTTY;
		break;
	}

err_out:
	return ret;
}

#ifdef CONFIG_COMPAT
static long goodix_tools_compat_ioctl(
	struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *arg32 = compat_ptr(arg);

	if (!file->f_op || !file->f_op->unlocked_ioctl)
		return -ENOTTY;
	return file->f_op->unlocked_ioctl(file, cmd, (unsigned long)arg32);
}
#endif

static struct goodix_ts_core *core_data_locate(int minor)
{
	struct goodix_device_resource *res, *next;

	if (!list_empty(&goodix_devices.list)) {
		list_for_each_entry_safe(res, next, &goodix_devices.list, list) {
			if (res->core_data.tools_dev.miscdev.minor == minor)
				return &res->core_data;
		}
	}

	return NULL;
}

static int goodix_tools_open(struct inode *inode, struct file *filp)
{
	struct goodix_ts_core *cd = core_data_locate(iminor(inode));

	if (!cd) {
		ts_err("can't find core data");
		return -ENODEV;
	}

	goodix_ts_esd_off(cd);
	cd->tools_dev.is_open = true;
	filp->private_data = &cd->tools_dev;
	ts_info("success open tools");
	return 0;
}

static int goodix_tools_release(
	struct inode *inode, struct file *filp)
{
	struct goodix_ts_core *cd = core_data_locate(iminor(inode));

	if (!cd) {
		ts_err("can't find core data");
		return -ENODEV;
	}
	/* when the last close this dev node unregister the module */
	cd->tools_dev.is_open = false;
	cd->tools_ctrl_sync = false;
	goodix_ts_esd_on(cd);
	return 0;
}

static const struct file_operations goodix_tools_fops = {
	.owner = THIS_MODULE,
	.open = goodix_tools_open,
	.release = goodix_tools_release,
	.unlocked_ioctl = goodix_tools_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = goodix_tools_compat_ioctl,
#endif
};

/**
 * goodix_tools_init - init goodix tools device and register a miscdevice
 *
 * return: 0 success, else failed
 */
int goodix_tools_init(struct goodix_ts_core *core_data)
{
	int ret;
	struct goodix_tools_dev *tools_dev = &core_data->tools_dev;

	sprintf(tools_dev->name, "%s.%d", GOODIX_TOOLS_NAME, core_data->pdev->id);
	tools_dev->miscdev.minor = MISC_DYNAMIC_MINOR;
	tools_dev->miscdev.name = tools_dev->name;
	tools_dev->miscdev.fops = &goodix_tools_fops;
	ret = misc_register(&tools_dev->miscdev);
	if (ret)
		ts_err("Debug tools miscdev register failed");
	else
		ts_info("Debug tools miscdev register success");

	return ret;
}

void goodix_tools_exit(struct goodix_ts_core *core_data)
{
	struct goodix_tools_dev *tools_dev = &core_data->tools_dev;

	misc_deregister(&tools_dev->miscdev);
	ts_info("Debug tools miscdev exit");
}
