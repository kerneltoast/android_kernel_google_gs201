/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * s2mpg14-meter.h
 *
 * Copyright (C) 2022 Samsung Electronics
 *
 * header file including meter information of s2mpg14
 */

#ifndef __LINUX_MFD_S2MPG14_METER_H
#define __LINUX_MFD_S2MPG14_METER_H

#include "s2mpg14-register.h"

struct s2mpg14_meter {
	struct s2mpg14_dev *iodev;
	struct i2c_client *i2c;

	/* mutex for s2mpg14 meter */
	struct mutex meter_lock;
	u8 chg_mux_sel[S2MPG1415_METER_CHANNEL_MAX];
	u32 lpf_data[S2MPG1415_METER_CHANNEL_MAX]; /* 21-bit data */
	struct device *dev;
};

/* Public s2mpg14 Meter functions */
int s2mpg14_meter_set_muxsel(struct s2mpg14_meter *s2mpg14, int channel,
			     enum s2mpg1415_meter_muxsel m);

int s2mpg14_meter_onoff(struct s2mpg14_meter *s2mpg14, bool onoff);
int s2mpg14_ext_meter_onoff(struct s2mpg14_meter *s2mpg14, bool onoff);
u32 s2mpg14_muxsel_to_power_resolution(enum s2mpg1415_meter_muxsel m);
u32 s2mpg14_muxsel_to_current_resolution(enum s2mpg1415_meter_muxsel m);
void s2mpg14_meter_read_lpf_data_reg(struct s2mpg14_meter *s2mpg14,
				     u32 *data);

#endif /* __LINUX_MFD_S2MPG14_METER_H */
