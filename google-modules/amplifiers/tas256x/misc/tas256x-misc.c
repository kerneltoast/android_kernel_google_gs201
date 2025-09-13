/*
 * =============================================================================
 * Copyright (c) 2016  Texas Instruments Inc.
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation; version 2.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 * File:
 *     tas256x-misc.c
 *
 * Description:
 *     misc driver for Texas Instruments
 *     TAS256X High Performance 4W Smart Amplifier
 *
 * =============================================================================
 */

#ifdef CONFIG_DYNAMIC_DEBUG
#define DEBUG
#endif
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/firmware.h>
#include <linux/regmap.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/syscalls.h>
#include <linux/fcntl.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>

#include "physical_layer/inc/tas256x.h"
#include "tas256x-misc.h"
#include <linux/dma-mapping.h>

static struct tas256x_priv *g_tas256x;

static int tas256x_file_open(struct inode *inode, struct file *file)
{
	struct tas256x_priv *p_tas256x = g_tas256x;

	file->private_data = (void *)p_tas256x;

	pr_info("TAS256X %s\n", __func__);
	return 0;
}

static int tas256x_file_release(struct inode *inode, struct file *file)
{
	pr_info("TAS256X %s\n", __func__);

	file->private_data = (void *)NULL;

	return 0;
}

static ssize_t tas256x_file_read(struct file *file,
	char *buf, size_t count, loff_t *ppos)
{
	struct tas256x_priv *p_tas256x =
		(struct tas256x_priv *)file->private_data;
	int ret = 0;
	unsigned char *p_kbuf = NULL;
	unsigned int reg = 0;
	unsigned int len = 0;
	unsigned int channel = channel_left;

	mutex_lock(&p_tas256x->file_lock);

	p_kbuf = kzalloc(count, GFP_KERNEL);
	if (p_kbuf == NULL) {
		pr_err("TAS256X write no mem\n");
		goto err;
	}

	ret = copy_from_user(p_kbuf, buf, count);
	if (ret != 0) {
		pr_err("TAS256X copy_from_user failed.\n");
		goto err;
	}

	if ((p_kbuf[1] >= 0) && ((p_kbuf[1] <= 1)))
		channel = p_kbuf[1]+1;

	switch (p_kbuf[0]) {
	case TIAUDIO_CMD_REG_READ:
	{
		reg = ((unsigned int)p_kbuf[2] << 24) +
			((unsigned int)p_kbuf[3] << 16) +
			((unsigned int)p_kbuf[4] << 8) +
			(unsigned int)p_kbuf[5];

		pr_info("TAS256X TIAUDIO_CMD_REG_READ: current_reg = 0x%x, count=%d\n",
			reg, (int)count-6);
		len = count-6;
		if (len == 1) {
			unsigned int value = 0;
			ret = p_tas256x->read(p_tas256x, channel,
				reg, &value);
			if (ret < 0) {
				pr_err("TAS256X dev read fail %d\n", ret);
				break;
			}
			p_kbuf[6] = value;
			ret = copy_to_user(buf, p_kbuf, count);
			/* Failed to copy all the data, exit */
			if (ret != 0)
				pr_err("TAS256X copy to user fail %d\n", ret);
		} else if (len > 1) {
			ret = p_tas256x->bulk_read(p_tas256x, channel,
				reg, (unsigned char *)&p_kbuf[6], len);
			if (ret < 0) {
				pr_err("TAS256X dev bulk read fail %d\n", ret);
			} else {
				ret = copy_to_user(buf, p_kbuf, count);
				/* Failed to copy all the data, exit */
				if (ret != 0)
					pr_err("TAS256X copy to user fail %d\n", ret);
			}
		}
	}
	break;
	}
err:
	if (p_kbuf != NULL)
		kfree(p_kbuf);
	mutex_unlock(&p_tas256x->file_lock);
	return count;
}

static ssize_t tas256x_file_write(struct file *file,
	const char *buf, size_t count, loff_t *ppos)
{
	struct tas256x_priv *p_tas256x =
		(struct tas256x_priv *)file->private_data;
	int ret = 0;
	unsigned char *p_kbuf = NULL;
	unsigned int reg = 0;
	unsigned int len = 0;
	unsigned int channel = channel_left;

	mutex_lock(&p_tas256x->file_lock);

	p_kbuf = kzalloc(count, GFP_KERNEL);
	if (p_kbuf == NULL) {
		pr_err("TAS256X write no mem\n");
		goto err;
	}

	ret = copy_from_user(p_kbuf, buf, count);
	if (ret != 0) {
		pr_err("TAS256X copy_from_user failed.\n");
		goto err;
	}

	if ((p_kbuf[1] >= 0) && ((p_kbuf[1] <= 1)))
		channel = p_kbuf[1]+1;
	switch (p_kbuf[0]) {
	case TIAUDIO_CMD_REG_WITE:
		if (count > 5) {
			reg = ((unsigned int)p_kbuf[2] << 24) +
				((unsigned int)p_kbuf[3] << 16) +
				((unsigned int)p_kbuf[4] << 8) +
				(unsigned int)p_kbuf[5];
			len = count - 6;
			pr_info("TAS256X TIAUDIO_CMD_REG_WITE, Reg=0x%x, Val=0x%x\n",
				reg, p_kbuf[6]);
			if (len == 1) {
				unsigned int value = 0;
				value = p_kbuf[6];
				ret = p_tas256x->write(p_tas256x, channel, reg, value);
			} else if (len > 1) {
				ret = p_tas256x->bulk_write(p_tas256x, channel,
					reg, &p_kbuf[6], len);
			}
		} else {
			pr_err("TAS256X %s, write len fail, count=%d.\n",
				__func__, (int)count);
		}
	break;
	}
err:
	if (p_kbuf != NULL)
		kfree(p_kbuf);

	mutex_unlock(&p_tas256x->file_lock);

	return count;
}

static const struct file_operations fops = {
	.owner = THIS_MODULE,
	.read = tas256x_file_read,
	.write = tas256x_file_write,
	.unlocked_ioctl = NULL,
	.open = tas256x_file_open,
	.release = tas256x_file_release,
};

#define MODULE_NAME	"tas256x"
static struct miscdevice tas256x_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = MODULE_NAME,
	.fops = &fops,
};

int tas256x_register_misc(struct tas256x_priv *p_tas256x)
{
	int ret = 0;

	g_tas256x = p_tas256x;
	ret = misc_register(&tas256x_misc);
	if (ret)
		pr_err("TAS256X TAS256X misc fail: %d\n", ret);

	pr_info("TAS256X %s, leave\n", __func__);

	return ret;
}
EXPORT_SYMBOL(tas256x_register_misc);

int tas256x_deregister_misc(struct tas256x_priv *p_tas256x)
{
	misc_deregister(&tas256x_misc);
	return 0;
}
EXPORT_SYMBOL(tas256x_deregister_misc);

MODULE_AUTHOR("Texas Instruments Inc.");
MODULE_DESCRIPTION("TAS256X Misc Smart Amplifier driver");
MODULE_LICENSE("GPL v2");
