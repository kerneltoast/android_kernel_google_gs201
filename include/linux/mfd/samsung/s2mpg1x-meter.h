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
#include "s2mpg1x-register.h"

typedef enum {
	ID_S2MPG10,
	ID_S2MPG11,
	ID_COUNT,
} s2mpg1x_id_t;

int s2mpg1x_meter_set_async_blocking(s2mpg1x_id_t id, struct i2c_client *i2c,
				     unsigned long *jiffies_capture);
int s2mpg1x_meter_set_buck_channel_en(s2mpg1x_id_t id, struct i2c_client *i2c,
				      u8 *channels, int num_bytes);
int s2mpg1x_meter_set_ext_channel_en(s2mpg1x_id_t id, struct i2c_client *i2c,
				     u8 channels);
int s2mpg1x_meter_set_int_samp_rate(s2mpg1x_id_t id, struct i2c_client *i2c,
				    s2mpg1x_int_samp_rate hz);
int s2mpg1x_meter_set_ext_samp_rate(s2mpg1x_id_t id, struct i2c_client *i2c,
				    s2mpg1x_ext_samp_rate hz);

ssize_t s2mpg1x_meter_format_channel(char *buf, ssize_t count, int ch,
				     const char *name, const char *units,
				     u64 acc_data, u32 resolution,
				     u32 acc_count);

const u32 *s2mpg1x_meter_get_int_samping_rate_table(void);
const u32 *s2mpg1x_meter_get_ext_samping_rate_table(void);

#endif /* __LINUX_MFD_S2MPG1X_METER_H */
