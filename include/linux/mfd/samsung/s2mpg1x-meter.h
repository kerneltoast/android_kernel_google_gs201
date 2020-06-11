/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * include/linux/mfd/samsung/s2mpg1x-meter.h
 *
 * Copyright (C) 2015 Samsung Electronics
 *
 * header file including shared meter functions of s2mpg1x
 */

#ifndef __LINUX_MFD_S2MPG1X_METER_H
#define __LINUX_MFD_S2MPG1X_METER_H

#include <linux/i2c.h>

typedef enum {
	ID_S2MPG10,
	ID_S2MPG11,
} s2mpg1x_id_t;

int s2mpg1x_meter_set_async_blocking(s2mpg1x_id_t id, struct i2c_client *i2c,
				     unsigned long *jiffies_capture, u8 reg);
ssize_t s2mpg1x_format_meter_channel(char *buf, ssize_t count, int ch,
				     const char *name, const char *units,
				     u64 acc_data, u32 resolution,
				     u32 acc_count);

#endif /* __LINUX_MFD_S2MPG1X_METER_H */
