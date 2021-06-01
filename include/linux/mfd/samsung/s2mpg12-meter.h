/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * s2mpg12-meter.h
 *
 * Copyright (C) 2015 Samsung Electronics
 *
 * header file including meter information of s2mpg12
 */

#ifndef __LINUX_MFD_S2MPG12_METER_H
#define __LINUX_MFD_S2MPG12_METER_H

#include "s2mpg12-register.h"

struct s2mpg12_meter {
	struct s2mpg12_dev *iodev;
	struct i2c_client *i2c;

	/* mutex for s2mpg12 meter */
	struct mutex meter_lock;
	u8 meter_en;
	u8 ext_meter_en;
	u8 chg_mux_sel[S2MPG1X_METER_CHANNEL_MAX];
	u32 lpf_data[S2MPG1X_METER_CHANNEL_MAX]; /* 21-bit data */
	struct device *dev;
};

/* Public s2mpg12 Meter functions */
void s2mpg12_meter_load_measurement(struct s2mpg12_meter *s2mpg12,
				    s2mpg1x_meter_mode mode, u64 *data,
				   u32 *count, u64 *timestamp_capture);
int s2mpg12_meter_set_muxsel(struct s2mpg12_meter *s2mpg12, int channel,
			     s2mpg12_meter_muxsel m);

int s2mpg12_meter_onoff(struct s2mpg12_meter *s2mpg12, bool onoff);
int s2mpg12_ext_meter_onoff(struct s2mpg12_meter *s2mpg12, bool onoff);
u32 s2mpg12_muxsel_to_power_resolution(s2mpg12_meter_muxsel m);

#endif /* __LINUX_MFD_S2MPG12_METER_H */
