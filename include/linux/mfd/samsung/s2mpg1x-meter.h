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
#include <linux/mfd/samsung/s2mpg1x.h>
#include "s2mpg1x-register.h"
#include <linux/mfd/samsung/s2mpg1x-meter.h>
#include <linux/mfd/samsung/s2mpg10-register.h>
#include <linux/mfd/samsung/s2mpg11-register.h>
#include <linux/fs.h>

typedef enum {
	ADDRESS_CTRL1,
	ADDRESS_CTRL2,
	ADDRESS_BUCKEN1,
	ADDRESS_COUNT,
} address_t;

const int COMMON_ADDRESS[ADDRESS_COUNT][ID_COUNT] = {
	[ADDRESS_CTRL1] = { S2MPG10_METER_CTRL1, S2MPG11_METER_CTRL1 },
	[ADDRESS_CTRL2] = { S2MPG10_METER_CTRL2, S2MPG11_METER_CTRL2 },
	[ADDRESS_BUCKEN1] = { S2MPG10_METER_BUCKEN1, S2MPG11_METER_BUCKEN1 },
};

const u32 s2mpg1x_int_sample_rate_uhz[S2MPG1X_INT_FREQ_COUNT] = {
	[INT_7P_8125HZ] = 7812500, [INT_15P_625HZ] = 15625000,
	[INT_31P_25HZ] = 31250000, [INT_62P_5HZ] = 62500000,
	[INT_125HZ] = 125000000,   [INT_250HZ] = 250000000,
	[INT_500HZ] = 500000000,   [INT_1000HZ] = 1000000000,
};

const u32 s2mpg1x_ext_sample_rate_uhz[S2MPG1X_EXT_FREQ_COUNT] = {
	[EXT_7P_628125HZ] = 7628125, [EXT_15P_25625HZ] = 15256250,
	[EXT_30P_5125HZ] = 30512500, [EXT_61P_025HZ] = 61025000,
	[EXT_122P_05HZ] = 122050000,
};

#define ACQUISITION_TIME_US (40 * S2MPG1X_METER_CHANNEL_MAX)
static inline int s2mpg1x_meter_set_async_blocking(enum s2mpg1x_id id,
						   struct i2c_client *i2c,
						   u64 *timestamp_capture)
{
	u8 val = 0xFF;
	int ret;
	u8 reg = COMMON_ADDRESS[ADDRESS_CTRL2][id];

	/* When 1 is written into ASYNC_RD bit, */
	/* transfer the accumulator data to readable registers->self-cleared */
	ret = s2mpg1x_update_reg(id, i2c, reg, ASYNC_RD_MASK, ASYNC_RD_MASK);
	if (ret != 0) {
		// Immediate return if failed
		return ret;
	}

	if (timestamp_capture)
		*timestamp_capture = ktime_get_boottime_ns();

	/* Based on the s2mpg1x datasheets, (40 us * channel count) is the
	 * maximum time required for acquisition of all samples across all
	 * channels. However, typically, we do not write 1 to ASYNC during
	 * acquisition, so return immediately to reduce refresh time.
	 */
	ret = s2mpg1x_read_reg(id, i2c, reg, &val);

	if (ret == 0 && (val & ASYNC_RD_MASK) == 0x00)
		return ret; /* Read success */

	/* Reading has failed OR we sampled during acquisition, so wait the
	 * acquisition time and return based on the values read.
	 */
	usleep_range(ACQUISITION_TIME_US, ACQUISITION_TIME_US + 100);

	ret = s2mpg1x_read_reg(id, i2c, reg, &val);
	if (ret != 0 || (val & ASYNC_RD_MASK) == 0x00)
		return ret;

	return -1; /* ASYNC value has not changed */
}

static inline ssize_t s2mpg1x_meter_format_channel(char *buf, ssize_t count,
						   int ch, const char *name,
						   const char *units,
						   u64 acc_data, u32 resolution,
						   u32 acc_count)
{
	// Note: resolution is in iq30
	// --> Convert it back to a human readable decimal value
	const u32 one_billion = 1000000000;
	u64 resolution_max = _IQ30_to_int((u64)resolution * one_billion);

	return scnprintf(buf + count, PAGE_SIZE - count,
			 "CH%d[%s]: 0x%016x * %d.%09lu / 0x%08x %s\n", ch, name,
			 acc_data, resolution_max / one_billion,
			 resolution_max % one_billion, acc_count, units);
}

static inline int s2mpg1x_meter_set_buck_channel_en(enum s2mpg1x_id id,
						    struct i2c_client *i2c,
						    u8 *channels, int num_bytes)
{
	if (num_bytes > S2MPG1X_METER_BUCKEN_BUF)
		return -EINVAL;

	return s2mpg1x_bulk_write(id, i2c, COMMON_ADDRESS[ADDRESS_BUCKEN1][id],
				  num_bytes, channels);
}

static inline int s2mpg1x_meter_set_ext_channel_en(enum s2mpg1x_id id,
						   struct i2c_client *i2c,
						   u8 channels)
{
	return s2mpg1x_update_reg(id, i2c, COMMON_ADDRESS[ADDRESS_CTRL2][id],
				  channels << EXT_METER_CHANNEL_EN_OFFSET,
				  EXT_METER_CHANNEL_EN_MASK);
}

static inline int s2mpg1x_meter_set_int_samp_rate(enum s2mpg1x_id id,
						  struct i2c_client *i2c,
						  s2mpg1x_int_samp_rate hz)
{
	return s2mpg1x_update_reg(id, i2c, COMMON_ADDRESS[ADDRESS_CTRL1][id],
				  hz << INT_SAMP_RATE_SHIFT,
				  INT_SAMP_RATE_MASK);
}

static inline int s2mpg1x_meter_set_ext_samp_rate(enum s2mpg1x_id id,
						  struct i2c_client *i2c,
						  s2mpg1x_ext_samp_rate hz)
{
	return s2mpg1x_update_reg(id, i2c, COMMON_ADDRESS[ADDRESS_CTRL2][id],
				  hz, EXT_SAMP_RATE_MASK);
}

static inline const u32 *s2mpg1x_meter_get_int_samping_rate_table(void)
{
	return s2mpg1x_int_sample_rate_uhz;
}

static inline const u32 *s2mpg1x_meter_get_ext_samping_rate_table(void)
{
	return s2mpg1x_ext_sample_rate_uhz;
}

#endif /* __LINUX_MFD_S2MPG1X_METER_H */
