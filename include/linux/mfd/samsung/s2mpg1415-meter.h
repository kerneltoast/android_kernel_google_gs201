/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * include/linux/mfd/samsung/s2mpg1415-meter.h
 *
 * Copyright (C) 2022 Samsung Electronics
 *
 * header file including shared meter functions of s2mpg1415
 */

#ifndef __LINUX_MFD_S2MPG1415_METER_H
#define __LINUX_MFD_S2MPG1415_METER_H

#include <linux/i2c.h>
#include <linux/mfd/samsung/s2mpg1415.h>
#include "s2mpg1415-register.h"
#include <linux/mfd/samsung/s2mpg14-register.h>
#include <linux/mfd/samsung/s2mpg15-register.h>
#include <linux/fs.h>

#define ADDRESS_AT(id, suffix) \
		((id) == ID_S2MPG14 ? S2MPG14_METER_##suffix : S2MPG15_METER_##suffix)

static int s2mpg1415_meter_get_acquisition_time_us(enum s2mpg1415_int_samp_rate samp_rate_sel)
{
	/* The internal data and sample count are not updated at the same timing
	 * on Pro due to the duty cycling function, and the ASYNC_RD will depend
	 * on internal sampling rate.
	 * So at least waiting for internal rail to complete a sampling will be
	 * necessary to prevent the mismatch of irregular timing. (b/209886118)
	 */

	switch (samp_rate_sel) {
	case INT_7P_8125HZ:
		return INT_7P_8125HZ_ACQUISITION_TIME_US;
	case INT_15P_625HZ:
		return INT_15P_625HZ_ACQUISITION_TIME_US;
	case INT_31P_25HZ:
		return INT_31P_25HZ_ACQUISITION_TIME_US;
	case INT_62P_5HZ:
		return INT_62P_5HZ_ACQUISITION_TIME_US;
	case INT_125HZ:
		return INT_125HZ_ACQUISITION_TIME_US;
	case INT_250HZ:
		return INT_250HZ_ACQUISITION_TIME_US;
	case INT_1000HZ:
		return INT_1000HZ_ACQUISITION_TIME_US;
	case INT_FREQ_COUNT:
	case INT_FREQ_NONE:
		return 0;
	};

	return 0;
}

#define ACQUISITION_TIME_DIVISOR 16
static int s2mpg1415_meter_set_async_blocking(enum s2mpg1415_id id,
						     struct i2c_client *i2c,
						     u64 *timestamp_capture,
						     enum s2mpg1415_int_samp_rate samp_rate_sel)
{
	u8 val = 0xFF;
	int ret;
	u8 reg = ADDRESS_AT(id, CTRL2);

	const u32 acquisition_time_us =
		s2mpg1415_meter_get_acquisition_time_us(samp_rate_sel);
	const u32 min_acquisition_time_us = acquisition_time_us /
		ACQUISITION_TIME_DIVISOR;
	int acquisition_delay_count = 0;

	/* When 1 is written into ASYNC_RD bit, */
	/* transfer the accumulator data to readable registers->self-cleared */
	ret = s2mpg1415_update_reg(id, i2c, reg, ASYNC_RD_MASK, ASYNC_RD_MASK);
	if (ret != 0) {
		/* Immediate return if failed */
		return ret;
	}

	if (timestamp_capture)
		*timestamp_capture = ktime_get_boottime_ns();

	/* Verify if acquisition is already complete before a polled delay.
	 * Return immediately to reduce refresh time if so.
	 */
	ret = s2mpg1415_read_reg(id, i2c, reg, &val);

	if (ret == 0 && (val & ASYNC_RD_MASK) == 0x00)
		return ret; /* Read success */

	/* Reading has failed OR we sampled during acquisition, so wait the
	 * acquisition time and return based on the values read.
	 */
	do {
		usleep_range(min_acquisition_time_us,
			     min_acquisition_time_us + 100);
		ret = s2mpg1415_read_reg(id, i2c, reg, &val);
		if (ret != 0 || (val & ASYNC_RD_MASK) == 0x00)
			return ret;
		acquisition_delay_count++;
	} while (acquisition_delay_count < ACQUISITION_TIME_DIVISOR);

	pr_err("odpm: acquisition_time_us: %d not enough\n", acquisition_time_us);

	return -1; /* ASYNC value has not changed */
}

static inline ssize_t s2mpg1415_meter_format_channel(char *buf, ssize_t count,
						     int ch, const char *name,
						     const char *units,
						     u64 acc_data,
						     u32 resolution,
						     u32 acc_count)
{
	/* Note: resolution is in iq30
	 * --> Convert it back to a human readable decimal value
	 */
	const u32 one_billion = 1000000000;
	u64 resolution_max = _IQ30_to_int((u64)resolution * one_billion);

	return scnprintf(buf + count, PAGE_SIZE - count,
			 "CH%d[%s]: 0x%016llx * %lld.%09llu / 0x%08x %s\n", ch, name,
			 acc_data, resolution_max / one_billion,
			 resolution_max % one_billion, acc_count, units);
}

static inline int s2mpg1415_meter_set_buck_channel_en(enum s2mpg1415_id id,
						      struct i2c_client *i2c,
						      u8 *channels,
						      int num_bytes)
{
	if (num_bytes > S2MPG1415_METER_BUCKEN_BUF)
		return -EINVAL;

	return s2mpg1415_bulk_write(id, i2c, ADDRESS_AT(id, BUCKEN1),
				    num_bytes, channels);
}

static inline int s2mpg1415_meter_set_ext_channel_en(enum s2mpg1415_id id,
						     struct i2c_client *i2c,
						     u8 channels)
{
	return s2mpg1415_update_reg(id, i2c, ADDRESS_AT(id, CTRL2),
				    channels << EXT_METER_CHANNEL_EN_OFFSET,
				    EXT_METER_CHANNEL_EN_MASK);
}

static inline int s2mpg1415_meter_set_int_samp_rate(enum s2mpg1415_id id,
						    struct i2c_client *i2c,
						    enum s2mpg1415_int_samp_rate samp_rate_sel)
{
	return s2mpg1415_update_reg(id, i2c, ADDRESS_AT(id, CTRL1),
				    samp_rate_sel << INT_SAMP_RATE_SHIFT,
				    INT_SAMP_RATE_MASK);
}

static inline int s2mpg1415_meter_set_ext_samp_rate(enum s2mpg1415_id id,
						    struct i2c_client *i2c,
						    enum s2mpg1415_ext_samp_rate samp_rate_sel)
{
	return s2mpg1415_update_reg(id, i2c, ADDRESS_AT(id, CTRL2),
				    samp_rate_sel, EXT_SAMP_RATE_MASK);
}

static inline int s2mpg1415_meter_set_lpf_coefficient(enum s2mpg1415_id id,
						      struct i2c_client *i2c,
						      int ch,
						      u32 val)
{
	if (ch >= S2MPG1415_METER_CHANNEL_MAX)
		return -EINVAL;

	return s2mpg1415_write_reg(id, i2c, ADDRESS_AT(id, LPF_C0_0) +
				   ch, val);
}

static inline void s2mpg1415_meter_set_mode(enum s2mpg1415_id id,
					    struct i2c_client *i2c,
					    enum s2mpg1415_meter_mode mode,
					    bool is_acc_mode)
{
	int mode_addr = is_acc_mode ? ADDRESS_AT(id, CTRL4) : ADDRESS_AT(id, CTRL6);

	switch (mode) {
	case S2MPG1415_METER_POWER:
		s2mpg1415_write_reg(id, i2c, mode_addr, 0x00);
		s2mpg1415_update_reg(id, i2c, mode_addr + 1,
				     0x00, /* mask= */ 0x0F);
		break;
	case S2MPG1415_METER_CURRENT:
		s2mpg1415_write_reg(id, i2c, mode_addr, 0xFF);
		s2mpg1415_update_reg(id, i2c, mode_addr + 1,
				     0x0F, /* mask= */ 0x0F);
		break;
	}
}

static inline void s2mpg1415_meter_set_acc_mode(enum s2mpg1415_id id,
						struct i2c_client *i2c,
						enum s2mpg1415_meter_mode mode)
{
	s2mpg1415_meter_set_mode(id, i2c, mode, /* is_acc_mode= */ true);
}

static inline void s2mpg1415_meter_set_lpf_mode(enum s2mpg1415_id id,
						struct i2c_client *i2c,
						enum s2mpg1415_meter_mode mode)
{
	s2mpg1415_meter_set_mode(id, i2c, mode, /* is_acc_mode= */ false);
}

static inline void s2mpg1415_meter_read_acc_data_reg(enum s2mpg1415_id id,
						     struct i2c_client *i2c,
						     u64 *data)
{
	int i;
	u8 buf[S2MPG1415_METER_ACC_BUF];
	u8 reg = ADDRESS_AT(id, ACC_DATA_CH0_1); /* first acc data register */

	for (i = 0; i < S2MPG1415_METER_CHANNEL_MAX; i++) {
		s2mpg1415_bulk_read(id, i2c, reg, S2MPG1415_METER_ACC_BUF, buf);

		/* 41 bits of data */
		data[i] = ((u64)buf[0] << 0) | ((u64)buf[1] << 8) |
			  ((u64)buf[2] << 16) | ((u64)buf[3] << 24) |
			  ((u64)buf[4] << 32) | (((u64)buf[5] & 0x1) << 40);

		reg += S2MPG1415_METER_ACC_BUF;
	}
}

static inline void s2mpg1415_meter_read_acc_count(enum s2mpg1415_id id,
						  struct i2c_client *i2c,
						  u32 *count)
{
	u8 data[S2MPG1415_METER_COUNT_BUF];
	u8 reg = ADDRESS_AT(id, ACC_COUNT_1); /* first count register */

	s2mpg1415_bulk_read(id, i2c, reg, S2MPG1415_METER_COUNT_BUF, data);

	/* ACC_COUNT is 20-bit data */
	*count = (data[0] << 0) | (data[1] << 8) | ((data[2] & 0x0F) << 16);
}

static inline void s2mpg1415_meter_read_lpf_data_reg(enum s2mpg1415_id id,
						     struct i2c_client *i2c,
						     u32 *data)
{
	int i;
	u8 buf[S2MPG1415_METER_LPF_BUF];
	u8 reg = ADDRESS_AT(id, LPF_DATA_CH0_1); /* first lpf data register */

	for (i = 0; i < S2MPG1415_METER_CHANNEL_MAX; i++) {
		s2mpg1415_bulk_read(id, i2c, reg, S2MPG1415_METER_LPF_BUF, buf);

		/* LPF is 21-bit data */
		data[i] = buf[0] + (buf[1] << 8) + ((buf[2] & 0x1F) << 16);

		reg += S2MPG1415_METER_LPF_BUF;
	}
}

/**
 * Load measurement into registers and read measurement from the registers
 *
 * Note: data must be an array with length S2MPG1415_METER_CHANNEL_MAX
 */
static inline int s2mpg1415_meter_measure_acc(enum s2mpg1415_id id,
					      struct i2c_client *i2c,
					      struct mutex *meter_lock,
					      enum s2mpg1415_meter_mode mode,
					      u64 *data,
					      u32 *count,
					      u64 *timestamp_capture,
					      enum s2mpg1415_int_samp_rate samp_rate_sel)
{
	mutex_lock(meter_lock);

	s2mpg1415_meter_set_acc_mode(id, i2c, mode);

	s2mpg1415_meter_set_async_blocking(id, i2c,
					   timestamp_capture,
					   samp_rate_sel);

	if (data)
		s2mpg1415_meter_read_acc_data_reg(id, i2c, data);

	if (count)
		s2mpg1415_meter_read_acc_count(id, i2c, count);

	mutex_unlock(meter_lock);

	return 0;
}

#define SW_RESET_DELAYTIME_US 2
static inline int s2mpg1415_meter_sw_reset(enum s2mpg1415_id id,
					   struct i2c_client *i2c)
{
	int ret;

	ret = s2mpg1415_update_reg(id, i2c, ADDRESS_AT(id, CTRL5), 0x40,
				   SOFT_RST_MASK);
	if (ret != 0)
		pr_err("odpm: s2mpg1%d-odpm: failed to update meter_ctrl5 bit_6 to 1\n", id + 4);

	usleep_range(SW_RESET_DELAYTIME_US, SW_RESET_DELAYTIME_US + 100);

	ret = s2mpg1415_update_reg(id, i2c, ADDRESS_AT(id, CTRL1), 0x01,
				   METER_EN_MASK);
	if (ret != 0)
		pr_err("odpm: s2mpg1%d-odpm: failed to update meter_ctrl1 bit_0 to 1\n", id + 4);

	return ret;
}
#endif /* __LINUX_MFD_S2MPG1415_METER_H */
