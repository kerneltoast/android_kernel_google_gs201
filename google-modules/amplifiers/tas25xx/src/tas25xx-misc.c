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
 *	tas25xx-misc.c
 *
 * Description:
 *	misc driver for Texas Instruments
 *	TAS25XX High Performance 4W Smart Amplifier
 *
 * =============================================================================
 */

#define DEBUG
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

#include "inc/tas25xx.h"
#include "inc/tas25xx-misc.h"
#include <linux/dma-mapping.h>

#define FMT "%02x: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n"

enum tas_devop_t {
	OP_REG_READ = 0,
	OP_PAGE_READ = 1,
	OP_SNG_WRITE = 2,
	OP_BURST_WRITE = 3,
	OP_PING = 4,
};

struct tas_audio_dev {
	uint8_t channel;
	uint8_t book;
	uint8_t page;
	uint8_t reg;
	uint8_t read_pending;
	enum tas_devop_t op;
};


static struct tas_audio_dev s_tasdevop;
static uint32_t s_r[128];
static struct tas25xx_priv *g_tas25xx;

static int32_t tas25xx_file_open(struct inode *inode, struct file *file)
{
	struct tas25xx_priv *p_tas25xx = g_tas25xx;

	file->private_data = (void *)p_tas25xx;

	pr_info("TAS25XX %s\n", __func__);
	return 0;
}

static int32_t tas25xx_file_release(struct inode *inode, struct file *file)
{
	pr_info("TAS25XX %s\n", __func__);

	file->private_data = (void *)NULL;

	return 0;
}

static ssize_t tas25xx_file_read(struct file *file,
	char *buf, size_t count, loff_t *ppos)
{
	struct tas25xx_priv *p_tas25xx =
		(struct tas25xx_priv *)file->private_data;
	int32_t ret = 0, i, count_l;
	uint8_t  *p_kbuf = NULL;
	uint32_t  reg = 0;
	uint32_t  len = 0;
	uint32_t  value = 0;
	uint32_t  channel = 0;

	mutex_lock(&p_tas25xx->file_lock);

	pr_info("%s size=%zu", __func__, count);

	if (count > 8) {
		if (!s_tasdevop.read_pending) {
			count = 0;
			goto done_read;
		}
		p_kbuf = kzalloc(count, GFP_KERNEL);
		if (p_kbuf == NULL)
			goto done_read;

		channel = s_tasdevop.channel;
		pr_info("%s ch=%d B:P %02x:%02x:%02x\n",
			__func__, channel, s_tasdevop.book, s_tasdevop.page, s_tasdevop.reg);

		switch (s_tasdevop.op) {
		case OP_PING:
			count = snprintf(p_kbuf, 64, "ch%d is %s\n",
					channel, p_tas25xx->ch_failed[channel]?"failed":"tas2572");
			pr_info("%s: %s\n", __func__, p_kbuf);
			break;
		case OP_REG_READ:
			reg = TAS25XX_REG(s_tasdevop.book, s_tasdevop.page, s_tasdevop.reg);
			ret = p_tas25xx->read(p_tas25xx, channel,
				reg, &value);
			if (ret < 0)
				count = snprintf(p_kbuf, 64, "%s\n", "Error");
			else
				count = snprintf(p_kbuf, 64, "%02x\n", value);
			pr_info("%s ch=%d B:P:R %02x:%02x:%02x(%d) value=%d\n",
				__func__, channel, s_tasdevop.book,
				s_tasdevop.page, s_tasdevop.reg, reg, value);
			break;

		case OP_PAGE_READ:
			reg = TAS25XX_REG(s_tasdevop.book, s_tasdevop.page, 0);
			for (i = 0; i < 128; i++) {
				ret = p_tas25xx->read(p_tas25xx, channel,
					reg + i, &s_r[i]);
				if (ret) {
					memset(s_r, 0, sizeof(s_r));
					break;
				}
			}

			if (ret) {
				count = snprintf(p_kbuf, 64, "error=%d\n", ret);
			} else {
				count_l = 0;
				if (count < ((52*8)+1)) {
					count = snprintf(p_kbuf, 64,
						"page dump not possible\n");
				} else {
					for (i = 0; i < 8; i++) {
						count_l += snprintf(p_kbuf, 64, FMT, i, s_r[(i*16) + 0],
							s_r[(i*16) + 1], s_r[(i*16) + 2], s_r[(i*16) + 3], s_r[(i*16) + 4],
							s_r[(i*16) + 5], s_r[(i*16) + 6], s_r[(i*16) + 7], s_r[(i*16) + 8],
							s_r[(i*16) + 9], s_r[(i*16) + 10], s_r[(i*16) + 11], s_r[(i*16) + 12],
							s_r[(i*16) + 13], s_r[(i*16) + 14], s_r[(i*16) + 15]);
						p_kbuf += 52;
					}
					count = count_l;
				}
			}
			break;

		default:
			count = snprintf(p_kbuf, 64, "%s\n", "invalid op");
			break;
		}

		ret = copy_to_user(buf, p_kbuf, count);
		/* Failed to copy all the data, exit */
		if (ret != 0)
			pr_err("TAS25XX copy to user fail %d\n", ret);

		s_tasdevop.read_pending = 0;
		goto done_read;
	}

	p_kbuf = kzalloc(count, GFP_KERNEL);
	if (p_kbuf == NULL)
		goto done_read;

	ret = copy_from_user(p_kbuf, buf, count);
	if (ret != 0) {
		pr_err("TAS25XX copy_from_user failed.\n");
		goto done_read;
	}

	if ((p_kbuf[1] >= 0) && ((p_kbuf[1] <= 1)))
		channel = p_kbuf[1];

	switch (p_kbuf[0]) {
	case TIAUDIO_CMD_REG_READ:
	{
		reg = ((uint32_t)p_kbuf[2] << 24) +
			((uint32_t)p_kbuf[3] << 16) +
			((uint32_t)p_kbuf[4] << 8) +
			(uint32_t)p_kbuf[5];

		pr_info("TAS25XX TIAUDIO_CMD_REG_READ: current_reg = 0x%x, count=%d\n",
			reg, (int)count-6);
		len = count-6;
		if (len == 1) {
			uint32_t  value = 0;

			ret = p_tas25xx->read(p_tas25xx, channel,
				reg, &value);
			if (ret < 0) {
				pr_err("TAS25XX dev read fail %d\n", ret);
				break;
			}
			p_kbuf[6] = value;
			ret = copy_to_user(buf, p_kbuf, count);
			/* Failed to copy all the data, exit */
			if (ret != 0)
				pr_err("TAS25XX copy to user fail %d\n", ret);
		} else if (len > 1) {
			ret = p_tas25xx->bulk_read(p_tas25xx, channel,
				reg, (uint8_t  *)&p_kbuf[6], len);
			if (ret < 0) {
				pr_err("TAS25XX dev bulk read fail %d\n", ret);
			} else {
				ret = copy_to_user(buf, p_kbuf, count);
				/* Failed to copy all the data, exit */
				if (ret != 0)
					pr_err("TAS25XX copy to user fail %d\n",
						ret);
			}
		}
	}
	break;
	}

done_read:
	kfree(p_kbuf);
	mutex_unlock(&p_tas25xx->file_lock);
	return count;
}

static int32_t handle_read_write(struct tas25xx_priv *p_tas25xx,
	int32_t read_write_op, int32_t count, uint8_t *buf)
{
	static uint8_t ch_bpr[8] = {0};
	int32_t val, buf_sz, i;
	int32_t ret = 0;
	int32_t reg;
	int8_t l_buf[3];

	buf_sz = count - 2;
	buf += 2;

	i = 0;
	l_buf[2] = 0;
	while (buf_sz >= 2) {
		memcpy(l_buf, buf, 2);
		if (kstrtoint(l_buf, 16, &val) != 0)
			return -EINVAL;
		if (i <= 7) {
			pr_info("tas25xx: %s i=%d, val=%d\n", __func__, i, val);
			ch_bpr[i] = (uint8_t)val;
		} else {
			pr_info("tas25xx: write supported only for 4 bytes,ignoring additional bytes\n");
			break;
		}
		buf += 3;
		buf_sz -= 3;
		i++;
	}

	pr_info("tas25xx: ch=%d, BPR %02x:%02x:%02x v=%d %d %d %d(cnt=%d)\n",
		ch_bpr[0], ch_bpr[1], ch_bpr[2], ch_bpr[3],
		ch_bpr[4], ch_bpr[5], ch_bpr[6], ch_bpr[7], i);

	s_tasdevop.channel = ch_bpr[0];
	s_tasdevop.book = ch_bpr[1];
	s_tasdevop.page = ch_bpr[2];
	s_tasdevop.reg = ch_bpr[3];
	if (read_write_op == 1 || read_write_op == 3)
		s_tasdevop.read_pending = 1;
	else
		s_tasdevop.read_pending = 0;

	if (read_write_op == 1) {
		if (i == 3) {
			pr_info("tas25xx: page read\n");
			s_tasdevop.op = OP_PAGE_READ;
		} else if (i == 4) {
			pr_info("tas25xx: single read\n");
			s_tasdevop.op = OP_REG_READ;
		} else {
			pr_info("tas25xx: page/single read is supported\n");
			ret = -EINVAL;
		}
	} else if (read_write_op == 2) {
		if (i == 5) {
			pr_info("tas25xx: single write\n");
			s_tasdevop.op = OP_SNG_WRITE;
		} else if (i == 8) {
			pr_info("tas25xx: burst write\n");
			s_tasdevop.op = OP_BURST_WRITE;
		} else {
			pr_info("tas25xx: signle/burst write is supported\n");
			ret = -EINVAL;
		}
	} else if (read_write_op == 3) {
		if (i == 1) {
			pr_info("tas25xx: ping ch = %d\n", s_tasdevop.channel);
			s_tasdevop.op = OP_PING;
		} else {
			pr_info("tas25xx: ping need assign ch\n");
			ret = -EINVAL;
		}
	} else {
		pr_info("tas25xx: Only read and write is supported\n");
		ret = -EINVAL;
	}

	if (read_write_op == 2) {
		val = ch_bpr[4];
		reg = TAS25XX_REG(s_tasdevop.book, s_tasdevop.page, s_tasdevop.reg);
		if (s_tasdevop.op == OP_SNG_WRITE) {
			ret = p_tas25xx->write(p_tas25xx,
					s_tasdevop.channel, reg, val);
		} else if (s_tasdevop.op == OP_BURST_WRITE) {
			ret = p_tas25xx->bulk_write(p_tas25xx,
				s_tasdevop.channel, reg, &ch_bpr[4], 4);
		}
	}

	if (ret < 0)
		count = ret;

	return count;
}

static ssize_t tas25xx_file_write(struct file *file,
	const char *buf, size_t count, loff_t *ppos)
{
	struct tas25xx_priv *p_tas25xx =
		(struct tas25xx_priv *)file->private_data;
	int32_t ret = 0;
	uint8_t  *p_kbuf = NULL;
	uint32_t  reg = 0;
	uint32_t  len = 0;
	uint32_t  channel = 0;
	int32_t read_write_op;

	mutex_lock(&p_tas25xx->file_lock);

	pr_info("%s size=%zu", __func__, count);

	if (count < 4) {
		pr_err("TAS25XX invalid size %zu\n", count);
		ret = -EINVAL;
		goto done_write;
	}

	p_kbuf = kzalloc(count, GFP_KERNEL);
	if (p_kbuf == NULL) {
		ret = -ENOMEM;
		goto done_write;
	}

	ret = copy_from_user(p_kbuf, buf, count);
	if (ret != 0) {
		pr_err("TAS25XX copy_from_user failed.\n");
		goto done_write;
	}

	if (p_kbuf[0] == 'w' || p_kbuf[1] == 'W')
		read_write_op = 2;
	else if (p_kbuf[0] == 'r' || p_kbuf[1] == 'R')
		read_write_op = 1;
	else if (p_kbuf[0] == 'p' || p_kbuf[1] == 'P') {
		read_write_op = 3;
	} else
		read_write_op = 0;

	if (read_write_op) {
		count = handle_read_write(p_tas25xx, read_write_op, count, p_kbuf);
		goto done_write;
	}

	if ((p_kbuf[1] >= 0) && ((p_kbuf[1] <= 1)))
		channel = p_kbuf[1]+1;
	switch (p_kbuf[0]) {
	case TIAUDIO_CMD_REG_WITE:
		if (count > 5) {
			reg = ((uint32_t)p_kbuf[2] << 24) +
				((uint32_t)p_kbuf[3] << 16) +
				((uint32_t)p_kbuf[4] << 8) +
				(uint32_t)p_kbuf[5];
			len = count - 6;
			pr_info("TAS25XX TIAUDIO_CMD_REG_WITE, Reg=0x%x, Val=0x%x\n",
				reg, p_kbuf[6]);
			if (len == 1) {
				uint32_t  value = 0;

				value = p_kbuf[6];
				ret = p_tas25xx->write(p_tas25xx, channel, reg,
					value);
			} else if (len > 1) {
				ret = p_tas25xx->bulk_write(p_tas25xx, channel,
					reg, &p_kbuf[6], len);
			}
		} else {
			pr_err("TAS25XX %s, write len fail, count=%d.\n",
				__func__, (int)count);
		}
	break;
	}

done_write:
	kfree(p_kbuf);
	mutex_unlock(&p_tas25xx->file_lock);

	return count;
}

static const struct file_operations fops = {
	.owner = THIS_MODULE,
	.read = tas25xx_file_read,
	.write = tas25xx_file_write,
	.unlocked_ioctl = NULL,
	.open = tas25xx_file_open,
	.release = tas25xx_file_release,
};

#define MODULE_NAME	"tas_audio_dev"
static struct miscdevice tas25xx_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = MODULE_NAME,
	.fops = &fops,
};

int32_t tas25xx_register_misc(struct tas25xx_priv *p_tas25xx)
{
	int32_t ret = 0;

	g_tas25xx = p_tas25xx;
	ret = misc_register(&tas25xx_misc);
	if (ret)
		pr_err("TI-SmartPA TAS25XX misc fail: %d\n", ret);

	pr_info("TI-SmartPA %s, leave\n", __func__);

	return ret;
}
EXPORT_SYMBOL(tas25xx_register_misc);

int32_t tas25xx_deregister_misc(struct tas25xx_priv *p_tas25xx)
{
	misc_deregister(&tas25xx_misc);
	g_tas25xx = NULL;
	return 0;
}
EXPORT_SYMBOL(tas25xx_deregister_misc);

/*
 * MODULE_AUTHOR("Texas Instruments Inc.");
 * MODULE_DESCRIPTION("TAS25XX Misc Smart Amplifier driver");
 * MODULE_LICENSE("GPL v2");
 */
