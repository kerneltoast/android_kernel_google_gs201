/*
 ** ============================================================================
 ** Copyright (c) 2017  Texas Instruments Inc.
 **
 ** This program is free software; you can redistribute it and/or modify it
 ** under
 ** the terms of the GNU General Public License as published by the Free
 ** Software Foundation; version 2.
 **
 ** This program is distributed in the hope that it will be useful, but WITHOUT
 ** ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 ** FITNESS
 ** FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 ** details
 **
 ** File:
 **     tas25xx-calib.c
 **
 ** Description:
 **     misc driver for interfacing for  Texas Instruments TAS25xx algorithm
 **
 ** ============================================================================
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/device.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/semaphore.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include "algo/inc/tas_smart_amp_v2.h"
#include "tas25xx-algo-intf.h"
#include "algo/inc/tas25xx-calib.h"

#define TAS_PAYLOAD_SIZE 14

/* Holds the Packet data required for processing */
struct tas_dsp_pkt {
	u8 device_id;
	u8 book;
	u8 page;
	u8 offset;
	u8 data[TAS_PAYLOAD_SIZE * 4];
};

static int smartamp_params_ctrl(uint8_t *input, u8 dir, u8 count)
{
	u32 length = count / 4;
	u32 paramid = 0;
	u32 index;
	int ret = 0;
	int special_index = 0;
	struct tas_dsp_pkt *ppacket;

	if (length >= TAS_PAYLOAD_SIZE) {
		pr_err("TI-SmartPA: %s: Unsupported length!", __func__);
		return -EINVAL;
	}

	ppacket = kmalloc(sizeof(struct tas_dsp_pkt),
				GFP_KERNEL);
	if (!ppacket) {
		return -ENOMEM;
	}

	memset(ppacket, 0, sizeof(struct tas_dsp_pkt));
	ret = copy_from_user(ppacket, input, sizeof(struct tas_dsp_pkt));
	if (ret) {
		pr_err("TI-SmartPA: %s: Error copying from user\n", __func__);
		kfree(ppacket);
		return -EFAULT;
	}

	index = (ppacket->page - 1) * 30 + (ppacket->offset - 8) / 4;
	special_index = TAS_SA_IS_SPL_IDX(index);
	pr_info("TI-SmartPA: %s: index = %d", __func__, index);
	if (special_index == 0) {
		if ((index < 0 || index > MAX_DSP_PARAM_INDEX)) {
			pr_err("TI-SmartPA: %s: invalid index !\n", __func__);
			kfree(ppacket);
			return -EINVAL;
		}
	}
	pr_info("TI-SmartPA: %s: Valid Index. special = %s\n",
		__func__, special_index ? "Yes" : "No");

	/* speakers are differentiated by device ids */
	if (ppacket->device_id == DEVICE1) {
		paramid = TAS_CALC_PARAM_IDX(index, length, CHANNEL0);
		pr_debug("TI-SmartPA: %s: Rcvd device id for channel 0, %x\n",
			__func__, ppacket->device_id);
	} else if (ppacket->device_id == DEVICE2) {
		paramid = TAS_CALC_PARAM_IDX(index, length, CHANNEL1);
		pr_debug("TI-SmartPA: %s: Rcvd device id for channel 1, %x\n",
			__func__, ppacket->device_id);
	} else {
		pr_err("TI-SmartPA: %s: Wrong device id = %x\n",
			__func__, ppacket->device_id);
	}

	ret = tas25xx_smartamp_algo_ctrl(ppacket->data, paramid,
		dir, length * 4, 0/*Default to Rx*/);
	if (ret)
		pr_err("TI-SmartPA: %s: %s device id 0x%x params failed from afe, ret=%x\n",
				__func__,
				dir == TAS_GET_PARAM ? "get" : "set",
				ppacket->device_id, ret);
	else
		pr_info("TI-SmartPA: %s: Algo control returned %d\n",
			__func__, ret);

	if (dir == TAS_GET_PARAM) {
		ret = copy_to_user(input, ppacket, sizeof(struct tas_dsp_pkt));
		if (ret) {
			pr_err("TI-SmartPA: %s: Error copying to user after DSP",
				__func__);
			ret = -EFAULT;
		}
	}

	kfree(ppacket);
	return ret;
}

static int tas_calib_open(struct inode *inode, struct file *fd)
{
	return 0;
}

static ssize_t tas_calib_write(struct file *file,
		const char __user *buffer, size_t count, loff_t *offp)
{
	int rc = 0;

	rc = smartamp_params_ctrl((uint8_t *)buffer, TAS_SET_PARAM, count);
	return rc;
}

static ssize_t tas_calib_read(struct file *file, char __user *buffer,
		size_t count, loff_t *ptr)
{
	int rc;

	rc = smartamp_params_ctrl((uint8_t *)buffer, TAS_GET_PARAM, count);
	if (rc < 0)
		count = rc;
	return count;
}

static long tas_calib_ioctl(struct file *filp, uint cmd, ulong arg)
{
	return 0;
}

static int tas_calib_release(struct inode *inode, struct file *fd)
{
	return 0;
}

const struct file_operations tas_calib_fops = {
	.owner			= THIS_MODULE,
	.open			= tas_calib_open,
	.write			= tas_calib_write,
	.read			= tas_calib_read,
	.release		= tas_calib_release,
	.unlocked_ioctl	= tas_calib_ioctl,
};

static struct miscdevice tas_calib_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "tas_calib",
	.fops = &tas_calib_fops,
};

int tas_calib_init(void)
{
	int rc;

	pr_info("TI-SmartPA: %s", __func__);
	rc = misc_register(&tas_calib_misc);
	if (rc)
		pr_err("TI-SmartPA: %s: register calib misc failed\n",
			__func__);
	return rc;
}

void tas_calib_exit(void)
{
	misc_deregister(&tas_calib_misc);
}

/*
 *MODULE_AUTHOR("Texas Instruments Inc.");
 *MODULE_DESCRIPTION("tas2560 Misc driver");
 *MODULE_LICENSE("GPL v2");
 */
