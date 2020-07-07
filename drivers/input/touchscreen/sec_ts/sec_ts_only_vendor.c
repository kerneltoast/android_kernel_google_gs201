/* drivers/input/touchscreen/sec_ts_fw.c
 *
 * Copyright (C) 2015 Samsung Electronics Co., Ltd.
 * http://www.samsungsemi.com/
 *
 * Core file for Samsung TSC driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/firmware.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/irq.h>
#include <linux/of_gpio.h>
#include <linux/time.h>
#include <linux/vmalloc.h>

#include <linux/uaccess.h>
/*#include <asm/gpio.h>*/

#include "sec_ts.h"

u8 lv1cmd;
u8 *read_lv1_buff;
static int lv1_readsize;
static int lv1_readremain;
static int lv1_readoffset;

u8 lv1cmd_manual;
static int lv1_readsize_manual;
static int lv1_readremain_manual;
static int lv1_readoffset_manual;

#define SEC_TS_CMD_BUF_SZ 64
static u8 cmd_buf[SEC_TS_CMD_BUF_SZ];
static int cmd_buf_num;

static ssize_t sec_ts_reg_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static ssize_t sec_ts_regreadsize_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static inline ssize_t sec_ts_store_error(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);
static ssize_t sec_ts_enter_recovery_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static ssize_t sec_ts_regread_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t sec_ts_gesture_status_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static inline ssize_t sec_ts_show_error(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t sec_ts_reg_manual_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static ssize_t sec_ts_regreadsize_manual_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static ssize_t sec_ts_regread_manual_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static DEVICE_ATTR_WO(sec_ts_reg);
static DEVICE_ATTR_WO(sec_ts_regreadsize);
static DEVICE_ATTR_WO(sec_ts_enter_recovery);
static DEVICE_ATTR_RO(sec_ts_regread);
static DEVICE_ATTR_RO(sec_ts_gesture_status);

static DEVICE_ATTR_WO(sec_ts_reg_manual);
static DEVICE_ATTR_WO(sec_ts_regreadsize_manual);
static DEVICE_ATTR_RO(sec_ts_regread_manual);

static struct attribute *cmd_attributes[] = {
	&dev_attr_sec_ts_reg.attr,
	&dev_attr_sec_ts_regreadsize.attr,
	&dev_attr_sec_ts_enter_recovery.attr,
	&dev_attr_sec_ts_regread.attr,
	&dev_attr_sec_ts_gesture_status.attr,
	&dev_attr_sec_ts_reg_manual.attr,
	&dev_attr_sec_ts_regreadsize_manual.attr,
	&dev_attr_sec_ts_regread_manual.attr,
	NULL,
};

static struct attribute_group cmd_attr_group = {
	.attrs = cmd_attributes,
};

/* for debugging-------------------------------------------------------------*/
static void sec_ts_parsing_cmds(struct device *dev,
	const char *buf, size_t size, bool write)
{
	u8 result, n = 0;
	char *p, *temp_buf, *token;
	size_t token_len = 0;
	struct sec_ts_data *ts = dev_get_drvdata(dev);

	/* clear cmd_buf */
	memset(cmd_buf, 0, sizeof(cmd_buf));
	cmd_buf_num = 0;

	/* pre-check input */
	if (write) {
		if (size < 2) {
			input_info(true, &ts->client->dev,
				"%s: invalid input size %d\n",
				__func__);
			return;
		}
	} else {
		if (size < 3) {
			input_info(true, &ts->client->dev,
				"%s: invalid input size %d\n",
				__func__);
			return;
		} else if (buf[0] != 'R' && buf[0] != 'r') {
			input_info(true, &ts->client->dev,
				"%s: invalid input %c, Have to start with R(r)\n",
				__func__, buf[0]);
			return;
		}
	}

	/* alloc temp_buf for parsing
	 * read case will skip 1st character
	 */
	temp_buf = kstrdup(buf, GFP_KERNEL);
	if (!temp_buf) {
		pr_err("%s: memory allocation failed!",
			__func__);
		return;
	}
	p = temp_buf;

	/* newline case at last char */
	if (p[size - 1] == '\n')
		p[size - 1] = '\0';

	/* skip 1st character for read case */
	if (!write)
		p++;

	/* parsing */
	while (p && (n < SEC_TS_CMD_BUF_SZ)) {

		while (isspace(*p))
			p++;

		token = strsep(&p, " ");
		if (!token || *token == '\0')
			break;

		token_len = strlen(token);
		if (token_len != 2) {
			pr_err("%s: bad len %zu\n", __func__, token_len);
			n = 0;
			break;
		}

		if (kstrtou8(token, 16, &result)) {
			/* Conversion failed due to bad input.
			 * Discard the entire buffer.
			 */
			pr_err("%s: bad input\n", __func__);
			n = 0;
			break;
		}
		/* found a valid cmd/args */
		cmd_buf[n] = result;
		n++;
	}
	kfree(temp_buf);
	cmd_buf_num = n;
}

/* sysfs file node to write reg
 *
 *     echo _REG_ _VAL_ ... > sec_ts_reg_manual
 *
 *     e.g. write reg 0xD7 with 0x02 0x04
 *     echo D7 02 04 > sec_ts_reg_manual
 */
static ssize_t sec_ts_reg_manual_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct sec_ts_data *ts = dev_get_drvdata(dev);

	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		input_info(true, &ts->client->dev,
			"%s: Power off state\n", __func__);
		return -EIO;
	}

	sec_ts_parsing_cmds(dev, buf, size, true);

	if (cmd_buf_num) {
		ts->sec_ts_write_burst(ts, cmd_buf, cmd_buf_num);

		input_info(true, &ts->client->dev,
			"%s: size %d, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x\n",
			__func__, cmd_buf_num, cmd_buf[0], cmd_buf[1],
			cmd_buf[2], cmd_buf[3], cmd_buf[4]);
	}

	return size;
}

/* sysfs file node to read reg
 *
 * step 1: set reg and size that want to read
 *     echo R_REG_ _SIZE_ > sec_ts_regreadsize_manual
 *
 *     e.g. read reg 0x52 for 3 bytes
 *        echo R52 03 > sec_ts_regreadsize_manual
 *
 * step 2: read reg
 *     cat sec_ts_regread_manual
 */
static ssize_t sec_ts_regreadsize_manual_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct sec_ts_data *ts = dev_get_drvdata(dev);

	sec_ts_parsing_cmds(dev, buf, size, false);

	if (cmd_buf_num == 2) {
		lv1cmd_manual = cmd_buf[0];
		lv1_readsize_manual = cmd_buf[1];
		lv1_readoffset_manual = 0;
		lv1_readremain_manual = 0;
		input_info(true, &ts->client->dev,
			"%s: read reg %X sz %d\n",
			__func__, lv1cmd_manual, lv1_readsize_manual);
	} else
		input_info(true, &ts->client->dev,
			"%s: invalid input to reg read! cmd_num %d, cmd %x %x\n",
			__func__, cmd_buf_num, cmd_buf[0], cmd_buf[1]);

	return size;
}


/* sysfs file node to read reg
 *     check sec_ts_regreadsize_manual_store() above for details.
 */
static ssize_t sec_ts_regread_manual_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	u8 *read_lv1_buff_manual;
	struct sec_ts_data *ts = dev_get_drvdata(dev);
	unsigned int str_len = 0;
	int ret = 0, i;

	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		input_err(true, &ts->client->dev,
			"%s: Power off state\n", __func__);
		return -EIO;
	}
	if (lv1_readsize_manual < 1) {
		input_err(true, &ts->client->dev,
			"%s: Nothing to read\n", __func__);
		return -EIO;
	}

	disable_irq(ts->client->irq);

	read_lv1_buff_manual = kzalloc(lv1_readsize_manual, GFP_KERNEL);
	if (!read_lv1_buff_manual)
		goto malloc_err;

	ret = ts->sec_ts_read_heap(ts, lv1cmd_manual,
		read_lv1_buff_manual, lv1_readsize_manual);
	if (ret < 0)
		input_err(true, &ts->client->dev,
			"%s: read reg %x failed!\n",
			__func__, lv1cmd_manual);
	else {
		for (i = 0 ; i < lv1_readsize_manual ; i++)
			str_len += scnprintf(buf + str_len,
				PAGE_SIZE - str_len,
				"%02X ",
				(u8)read_lv1_buff_manual[i]);
		str_len += scnprintf(buf + str_len, PAGE_SIZE - str_len, "\n");
		input_info(true, &ts->client->dev, "%s: reg %X sz %d -> %s\n",
			__func__, lv1cmd_manual, lv1_readsize_manual, buf);
	}

	kfree(read_lv1_buff_manual);

malloc_err:
	lv1_readremain_manual = 0;
	enable_irq(ts->client->irq);

	return str_len;
}

static ssize_t sec_ts_reg_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct sec_ts_data *ts = dev_get_drvdata(dev);

	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		input_info(true, &ts->client->dev, "%s: Power off state\n",
			    __func__);
		return -EIO;
	}

	if (size > 0)
		ts->sec_ts_write_burst(ts, (u8 *)buf, size);

	input_info(true, &ts->client->dev,
		"%s: 0x%x, 0x%x, size %d\n",
		__func__, buf[0], buf[1], (int)size);
	return size;
}

static ssize_t sec_ts_regread_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct sec_ts_data *ts = dev_get_drvdata(dev);
	int ret;

	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		input_err(true, &ts->client->dev, "%s: Power off state\n",
			    __func__);
		return -EIO;
	}

	disable_irq(ts->client->irq);

	mutex_lock(&ts->device_mutex);

	read_lv1_buff = kzalloc(lv1_readsize, GFP_KERNEL);
	if (!read_lv1_buff)
		goto malloc_err;

	ret = ts->sec_ts_read_heap(ts, lv1cmd, read_lv1_buff, lv1_readsize);
	if (ret < 0) {
		input_err(true, &ts->client->dev, "%s: read %x command fail\n",
			__func__, lv1cmd);
		goto i2c_err;
	}

	input_info(true, &ts->client->dev, "%s: lv1_readsize = %d\n",
		    __func__, lv1_readsize);
	memcpy(buf, read_lv1_buff + lv1_readoffset, lv1_readsize);

i2c_err:
	kfree(read_lv1_buff);
malloc_err:
	mutex_unlock(&ts->device_mutex);
	lv1_readremain = 0;
	enable_irq(ts->client->irq);

	return lv1_readsize;
}

static ssize_t sec_ts_gesture_status_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct sec_ts_data *ts = dev_get_drvdata(dev);

	mutex_lock(&ts->device_mutex);
	memcpy(buf, ts->gesture_status, sizeof(ts->gesture_status));
	input_info(true, &ts->client->dev,
		    "%s: GESTURE STATUS %x %x %x %x %x %x\n", __func__,
		    ts->gesture_status[0], ts->gesture_status[1],
		    ts->gesture_status[2], ts->gesture_status[3],
		    ts->gesture_status[4], ts->gesture_status[5]);
	mutex_unlock(&ts->device_mutex);

	return sizeof(ts->gesture_status);
}

static ssize_t sec_ts_regreadsize_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t size)
{
	struct sec_ts_data *ts = dev_get_drvdata(dev);

	mutex_lock(&ts->device_mutex);

	lv1cmd = buf[0];
	lv1_readsize = ((unsigned int)buf[4] << 24) |
			((unsigned int)buf[3] << 16) |
			((unsigned int) buf[2] << 8) |
			((unsigned int)buf[1] << 0);
	lv1_readoffset = 0;
	lv1_readremain = 0;

	mutex_unlock(&ts->device_mutex);

	return size;
}

static ssize_t sec_ts_enter_recovery_store(struct device *dev,
					    struct device_attribute *attr,
					    const char *buf, size_t size)
{
	struct sec_ts_data *ts = dev_get_drvdata(dev);
	struct sec_ts_plat_data *pdata = ts->plat_data;
	int ret;
	unsigned long on;

	ret = kstrtoul(buf, 10, &on);
	if (ret != 0) {
		input_err(true, &ts->client->dev, "%s: failed to read:%d\n",
					__func__, ret);
		return -EINVAL;
	}

	if (on == 1) {
		disable_irq(ts->client->irq);
		gpio_free(pdata->irq_gpio);

		input_info(true, &ts->client->dev,
			    "%s: gpio free\n", __func__);
		if (gpio_is_valid(pdata->irq_gpio)) {
			ret = gpio_request_one(pdata->irq_gpio,
					    GPIOF_OUT_INIT_LOW, "sec,tsp_int");
			input_info(true, &ts->client->dev,
				    "%s: gpio request one\n", __func__);
			if (ret < 0)
				input_err(true, &ts->client->dev,
				    "%s: Unable to request tsp_int [%d]: %d\n",
				    __func__, pdata->irq_gpio, ret);
		} else {
			input_err(true, &ts->client->dev,
				"%s: Failed to get irq gpio\n", __func__);
			return -EINVAL;
		}

		pdata->power(ts, false);
		sec_ts_delay(100);
		pdata->power(ts, true);
	} else {
		gpio_free(pdata->irq_gpio);

		if (gpio_is_valid(pdata->irq_gpio)) {
			ret = gpio_request_one(pdata->irq_gpio, GPIOF_DIR_IN,
						"sec,tsp_int");
			if (ret) {
				input_err(true, &ts->client->dev,
				    "%s: Unable to request tsp_int [%d]\n",
				    __func__, pdata->irq_gpio);
				return -EINVAL;
			}
		} else {
			input_err(true, &ts->client->dev,
				"%s: Failed to get irq gpio\n", __func__);
			return -EINVAL;
		}

		pdata->power(ts, false);
		sec_ts_delay(500);
		pdata->power(ts, true);
		sec_ts_delay(500);

		/* AFE Calibration */
		ret = ts->sec_ts_write(ts,
				    SEC_TS_CMD_CALIBRATION_AMBIENT, NULL, 0);
		if (ret < 0)
			input_err(true, &ts->client->dev,
				"%s: fail to write AFE_CAL\n", __func__);

		sec_ts_delay(1000);
		enable_irq(ts->client->irq);
	}

	sec_ts_read_information(ts);

	return size;
}

static inline ssize_t sec_ts_show_error(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sec_ts_data *ts = dev_get_drvdata(dev);

	input_err(true, &ts->client->dev,
		"%s: read only function, %s\n", __func__, attr->attr.name);
	return -EPERM;
}

static inline ssize_t sec_ts_store_error(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct sec_ts_data *ts = dev_get_drvdata(dev);

	input_err(true, &ts->client->dev,
		"%s: write only function, %s\n", __func__, attr->attr.name);
	return -EPERM;
}

int sec_ts_raw_device_init(struct sec_ts_data *ts)
{
	int ret;

#ifdef CONFIG_SEC_SYSFS
	ts->dev = sec_device_create(ts, "sec_ts");
#else
	ts->dev = device_create(sec_class, NULL, 0, ts, "sec_ts");
#endif
	ret = IS_ERR(ts->dev);
	if (ret) {
		input_err(true, &ts->client->dev,
			    "%s: fail - device_create\n", __func__);
		return ret;
	}

	ret = sysfs_create_group(&ts->dev->kobj, &cmd_attr_group);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
			    "%s: fail - sysfs_create_group\n", __func__);
		goto err_sysfs;
	}

	return ret;
err_sysfs:
	input_err(true, &ts->client->dev, "%s: fail\n", __func__);
	return ret;
}

void sec_ts_raw_device_exit(struct sec_ts_data *ts)
{
	sysfs_remove_group(&ts->dev->kobj, &cmd_attr_group);
#ifdef CONFIG_SEC_SYSFS
	sec_device_destroy(ts->dev->devt);
#else
	device_destroy(sec_class, 0);
#endif
}

