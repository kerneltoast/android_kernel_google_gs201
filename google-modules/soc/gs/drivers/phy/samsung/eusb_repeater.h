/* SPDX-License-Identifier: GPL-2.0 */
/*
 *
 * Copyright (C) 2021 Samsung Electronics Co., Ltd.
 * http://www.samsungsemi.com/
 *
 * Core file for Samsung EUSB Repeater driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __EUSB_REPEATER_H__
#define __EUSB_REPEATER_H__

#include <asm/unaligned.h>
#include <linux/completion.h>
#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/debugfs.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/pinctrl/consumer.h>
#include <linux/slab.h>
#include <linux/timekeeping.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>
//#include <linux/wakelock.h>
#include <linux/workqueue.h>

#if defined(CONFIG_TRUSTONIC_TRUSTED_UI)
#include <linux/t-base-tui.h>
#endif
#ifdef CONFIG_SEC_SYSFS
#include <linux/sec_sysfs.h>
#endif

#include "../../i2c/busses/i2c-exynos5.h"

#define I2C_WRITE_BUFFER_SIZE		(32 - 1)//10
#define TUSB_I2C_RETRY_CNT		3
#define TUSB_MODE_CONTROL_RETRY_CNT	3
#define EXYNOS_USB_TUNE_LAST		0xEF

#define U_TX_ADJUST_PORT1		0x70
#define U_HS_TX_PRE_EMPHASIS_P1		0x71
#define U_RX_ADJUST_PORT1		0x72
#define U_DISCONNECT_SQUELCH_PORT1	0x73
#define E_HS_TX_PRE_EMPHASIS_P1		0x77
#define E_TX_ADJUST_PORT1		0x78
#define E_RX_ADJUST_PORT1		0x79
#define GPIO0_CONFIG			0x0
#define GPIO1_CONFIG			0x40
#define UART_PORT1			0x50
#define REV_ID				0xB0
#define I2C_GLOBAL_CONFIG		0xB2
#define INT_ENABLE_1			0xB3
#define INT_ENABLE_2			0xB4
#define BC_CONTROL			0xB6
#define BC_STATUS_1			0xB7
#define INT_STATUS_1			0xA3
#define INT_STATUS_2			0xA4
#define CONFIG_PORT1			0x60

#define REG_DISABLE_P1			BIT(6)

#define TIME_RH_READY			(3 * 1000)

struct eusb_repeater_tune_param {
	char name[32];
	unsigned int reg;
	unsigned int value;
	unsigned int shift;
	unsigned int mask;
};

struct eusb_repeater_data {
	struct device			*dev;
	struct extcon_dev		*edev;
	struct i2c_client		*client;
	struct mutex			mutex;
	struct mutex			i2c_mutex;
	struct eusb_repeater_plat_data	*pdata;
	unsigned int comm_err_count;	/* i2c comm error count */

	/* Tune Parma list */
	struct eusb_repeater_tune_param *tune_param;
	u32 tune_cnt;
	struct pinctrl *pinctrl;
	struct pinctrl_state *init_state;
	int ctrl_sel_status;
	struct dentry *root;
	struct regulator *vdd33;
	bool eusb_pm_status;
	bool eusb_data_enabled;

	bool ready;
	ktime_t start_time;
};

struct eusb_repeater_plat_data {
	int reserved;
};

#endif
