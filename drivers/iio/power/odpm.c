// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2020 Google LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/configfs.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/kmod.h>
#include <linux/slab.h>
#include <linux/platform_device.h>

#include <linux/i2c.h>
#include <linux/iio/iio.h>
#include <linux/iio/configfs.h>
#include <linux/iio/sysfs.h>

#include <linux/mfd/samsung/s2mpg10-meter.h>
#include <linux/mfd/samsung/s2mpg11-meter.h>

/* Cache accumulated values to prevent too frequent updates,
 * allow a refresh only every 50 ms.
 * Note: s2mpg1x chips can take on average between 1-2 ms
 * for a refresh.
 */
#define ODPM_MIN_POLLING_TIME_MS 50 /* ms */

#define SWITCH_CHIP_FUNC(infop, func, args...)                                 \
	do {                                                                   \
		switch ((infop)->chip.id) {                                    \
		case ODPM_CHIP_S2MPG10:                                        \
			ret = s2mpg10_##func(args);                            \
			break;                                                 \
		case ODPM_CHIP_S2MPG11:                                        \
			ret = s2mpg11_##func(args);                            \
			break;                                                 \
		}                                                              \
	} while (0)

#define SWITCH_METER_FUNC(infop, func, args...)                                \
	SWITCH_CHIP_FUNC(infop, func, info->meter, args)

/* At this moment, this driver supports a static 8 channels */
#define ODPM_CHANNEL_MAX S2MPG1X_METER_CHANNEL_MAX

enum odpm_chip_id {
	ODPM_CHIP_S2MPG10,
	ODPM_CHIP_S2MPG11,
};

enum odpm_rail_type {
	ODPM_RAIL_TYPE_REGULATOR,
	ODPM_RAIL_TYPE_SHUNT,
};

struct odpm_rail_data {
	/* Config */
	const char *name;
	const char *schematic_name;
	const char *subsys_name;
	enum odpm_rail_type type;
	u32 mux_select;

	/* External rail specific config */
	u32 shunt_uohms;
	u8 chip_enable_index;

	/* Data */
	u64 acc_power_uW_sec;
};

struct odpm_chip {
	/* Config */
	const char *name;
	enum odpm_chip_id id;
	u32 max_refresh_time_ms;

	int num_rails;
	struct odpm_rail_data *rails;

	/* Data */
	u64 acc_count;
	u64 acc_timestamp;
	s2mpg1x_int_samp_rate int_sampling_rate_i;
	s2mpg1x_ext_samp_rate ext_sampling_rate_i;
};

struct odpm_channel_data {
	int rail_i;
	bool enabled;
};

/**
 * dynamic struct odpm_info
 */
struct odpm_info {
	struct odpm_chip chip;
	void *meter; /* Parent meter device data */

	struct odpm_channel_data channels[ODPM_CHANNEL_MAX];

	struct workqueue_struct *work_queue;
	struct work_struct work_refresh;
	struct timer_list timer_refresh;
	unsigned long jiffies_last_poll;
};

/* TODO(stayfan): b/156109515
 * Dynamically configure based on DTS
 */
static u32 int_sample_rate_i_to_uhz[] = {
	[INT_7P_8125HZ] = 7812500, [INT_15P_625HZ] = 15625000,
	[INT_31P_25HZ] = 31250000, [INT_62P_5HZ] = 62500000,
	[INT_125HZ] = 125000000,   [INT_250HZ] = 250000000,
	[INT_500HZ] = 500000000,   [INT_1000HZ] = 1000000000,
};

static u32 ext_sample_rate_i_to_uhz[] = {
	[EXT_7P_628125HZ] = 7628125, [EXT_15P_25625HZ] = 15256250,
	[EXT_30P_5125HZ] = 30512500, [EXT_61P_025HZ] = 61025000,
	[EXT_122P_05HZ] = 122050000,
};

/**
 * IIO driver specific channel configurations
 */
#define ODPM_ACC_CHANNEL(_index)                                               \
	{                                                                      \
		.type = IIO_ENERGY, .indexed = 1, .channel = (_index),         \
		.info_mask_separate = BIT(IIO_CHAN_INFO_AVERAGE_RAW),          \
		.info_mask_shared_by_dir = BIT(IIO_CHAN_INFO_SAMP_FREQ),       \
	}

static const struct iio_chan_spec s2mpg1x_single_channel[ODPM_CHANNEL_MAX] = {
	ODPM_ACC_CHANNEL(0), ODPM_ACC_CHANNEL(1), ODPM_ACC_CHANNEL(2),
	ODPM_ACC_CHANNEL(3), ODPM_ACC_CHANNEL(4), ODPM_ACC_CHANNEL(5),
	ODPM_ACC_CHANNEL(6), ODPM_ACC_CHANNEL(7),
};

static int odpm_take_snapshot(struct odpm_info *info);

static int odpm_io_set_channel(struct odpm_info *info, int channel)
{
	int ret = -1;
	int rail_i = info->channels[channel].rail_i;

	SWITCH_METER_FUNC(info, meter_set_muxsel, channel,
			  info->chip.rails[rail_i].mux_select);
	return ret;
}

static int odpm_io_set_int_sample_rate(struct odpm_info *info,
				       s2mpg1x_int_samp_rate hz)
{
	int ret = -1;

	SWITCH_METER_FUNC(info, set_int_samp_rate, hz);
	return ret;
}

static int odpm_io_set_ext_sample_rate(struct odpm_info *info,
				       s2mpg1x_ext_samp_rate hz)
{
	int ret = -1;

	SWITCH_METER_FUNC(info, set_ext_samp_rate, hz);
	return ret;
}

static int odpm_io_set_meter_on(struct odpm_info *info, bool is_on)
{
	int ret = -1;

	SWITCH_METER_FUNC(info, meter_onoff, is_on);
	return ret;
}

static int odpm_io_set_ext_meter_on(struct odpm_info *info, bool is_on)
{
	int ret = -1;

	SWITCH_METER_FUNC(info, ext_meter_onoff, is_on);
	return ret;
}

static int odpm_io_set_ext_channel_on(struct odpm_info *info, u8 channels)
{
	int ret = -1;

	SWITCH_METER_FUNC(info, meter_ext_channel_onoff, channels);
	return ret;
}

static int odpm_io_send_blank_async(struct odpm_info *info,
				    unsigned long *jiffies)
{
	int ret = -1;

	SWITCH_METER_FUNC(info, meter_set_async_blocking, jiffies);
	return ret;
}

int odpm_configure_chip(struct odpm_info *info)
{
	int ch;
	u8 ext_channels_en = 0;

	/* TODO(stayfan): b/156107234
	 * error conditions
	 */
	odpm_io_set_int_sample_rate(info, info->chip.int_sampling_rate_i);
	odpm_io_set_ext_sample_rate(info, info->chip.ext_sampling_rate_i);
	for (ch = 0; ch < ODPM_CHANNEL_MAX; ch++) {
		if (info->channels[ch].enabled) {
			int rail_i = info->channels[ch].rail_i;

			odpm_io_set_channel(info, ch);

			ext_channels_en |=
				info->chip.rails[rail_i].chip_enable_index;
		}
	}

	odpm_io_set_ext_channel_on(info, ext_channels_en);

	odpm_io_set_meter_on(info, true);
	odpm_io_set_ext_meter_on(info, true);

	return 0;
}

int odpm_configure_start_measurement(struct odpm_info *info)
{
	unsigned long jiffies_capture = 0;

	/* For s2mpg1x chips, clear ACC registers */
	int ret = odpm_io_send_blank_async(info, &jiffies_capture);

	info->jiffies_last_poll = jiffies_capture;

	return ret;
}

void odpm_periodic_refresh_timeout(struct timer_list *t)
{
	int ret;
	struct odpm_info *info =
		container_of(t, struct odpm_info, timer_refresh);
	ret = mod_timer(&info->timer_refresh,
			jiffies +
			msecs_to_jiffies(info->chip.max_refresh_time_ms));
	if (ret < 0)
		pr_err("odpm: Refresh timer cannot be modified!\n");

	/* schedule the periodic reading from the chip */
	queue_work(info->work_queue, &info->work_refresh);
}

static void odpm_periodic_refresh_work(struct work_struct *work)
{
	struct odpm_info *info =
		container_of(work, struct odpm_info, work_refresh);

	pr_info("odpm: Refresh %s registers!\n", info->chip.name);

	odpm_take_snapshot(info);
}

static void odpm_periodic_refresh_setup(struct odpm_info *info)
{
	info->work_queue = create_workqueue("wq_odpm");
	INIT_WORK(&info->work_refresh, odpm_periodic_refresh_work);

	/* setup the latest moment for reading the regs before saturation */
	/* register the timer */
	timer_setup(&info->timer_refresh, odpm_periodic_refresh_timeout, 0);
	info->timer_refresh.expires =
		jiffies + msecs_to_jiffies(info->chip.max_refresh_time_ms);
	add_timer(&info->timer_refresh);
}

static bool odpm_match_int_sample_rate(struct odpm_info *info, u32 sample_rate)
{
	bool success = false;
	int i;

	for (i = 0; i < ARRAY_SIZE(int_sample_rate_i_to_uhz); i++) {
		if (sample_rate == int_sample_rate_i_to_uhz[i]) {
			info->chip.int_sampling_rate_i = i;
			success = true;
			break;
		}
	}

	return success;
}

static bool odpm_match_ext_sample_rate(struct odpm_info *info, u32 sample_rate)
{
	bool success = false;
	int i;

	for (i = 0; i < ARRAY_SIZE(ext_sample_rate_i_to_uhz); i++) {
		if (sample_rate == ext_sample_rate_i_to_uhz[i]) {
			info->chip.ext_sampling_rate_i = i;
			success = true;
			break;
		}
	}

	return success;
}

static int odpm_parse_dt_rail(struct odpm_rail_data *rail_data,
			      struct device_node *node)
{
	if (!node->name) {
		pr_err("odpm: cannot read node name\n");
		return -EINVAL;
	}
	rail_data->name = node->name;

	/* If the read fails, the pointers are not assigned; thus the pointers
	 * are left NULL if the values don't exist in the DT
	 */
	of_property_read_string(node, "schematic-name",
				&rail_data->schematic_name);
	of_property_read_string(node, "subsys-name", &rail_data->subsys_name);

	if (of_property_read_bool(node, "external_rail"))
		rail_data->type = ODPM_RAIL_TYPE_SHUNT;
	else
		rail_data->type = ODPM_RAIL_TYPE_REGULATOR;

	if (of_property_read_u32(node, "channel-mux-selection",
				 &rail_data->mux_select)) {
		pr_err("odpm: cannot read channel-mux-selection\n");
		return -EINVAL;
	}

	if (rail_data->type == ODPM_RAIL_TYPE_SHUNT) {
		u32 external_chip_en;

		if (of_property_read_u32(node, "shunt-res-uohms",
					 &rail_data->shunt_uohms)) {
			pr_err("odpm: cannot read shunt-res-uohms\n");
			return -EINVAL;
		}

		if (of_property_read_u32(node, "external-chip-en-index",
					 &external_chip_en)) {
			pr_err("odpm: cannot read external-chip-en-index\n");
			return -EINVAL;
		}
		rail_data->chip_enable_index = external_chip_en;
	}

	return 0;
}

static int odpm_parse_dt_rails(struct device *dev, struct odpm_info *info,
			       struct device_node *pmic_np)
{
	struct device_node *iter_np, *regulators_np;
	bool use_regulators_as_rails = false;
	struct odpm_rail_data *rail_data;
	int rail_i = 0, num_rails = 0;

	struct device_node *rails_np = of_find_node_by_name(pmic_np, "rails");

	if (!rails_np) {
		pr_err("odpm: cannot find rails DT node!\n");
		return -EINVAL;
	}

	/* Count rails */
	use_regulators_as_rails =
		of_property_read_bool(rails_np, "use-regulators-as-rails");
	if (use_regulators_as_rails) {
		regulators_np = of_find_node_by_name(pmic_np, "regulators");
		if (!regulators_np) {
			pr_err("odpm: Could not find regulators sub-node\n");
			return -EINVAL;
		}

		num_rails += of_get_child_count(regulators_np);
	}
	num_rails += of_get_child_count(rails_np);
	if (num_rails <= 0) {
		pr_err("odpm: Could not find any rails\n");
		return -EINVAL;
	}
	info->chip.num_rails = num_rails;

	/* Allocate/Initialize rails */
	rail_data =
		devm_kzalloc(dev, sizeof(*rail_data) * num_rails, GFP_KERNEL);
	if (!rail_data) {
		dev_err(dev, "odpm: could not allocate memory for rail data\n");
		return -ENOMEM;
	}
	info->chip.rails = rail_data;

	/* Populate rail data */
	if (use_regulators_as_rails) {
		for_each_child_of_node(regulators_np, iter_np) {
			int ret =
				odpm_parse_dt_rail(&rail_data[rail_i], iter_np);
			if (ret != 0)
				return ret;
			rail_i++;
		}
	}
	for_each_child_of_node(rails_np, iter_np) {
		int ret = odpm_parse_dt_rail(&rail_data[rail_i], iter_np);

		if (ret != 0)
			return ret;
		rail_i++;
	}

	/* Confidence check rail count */
	if (rail_i != num_rails) {
		pr_err("odpm: expected %d rails, got %d\n", num_rails, rail_i);
		return -EINVAL;
	}

	return 0;
}

static int odpm_parse_dt_channels(struct odpm_info *info,
				  struct device_node *channels_np)
{
	int rail_i = 0, channel_i = 0;
	int num_channels = of_get_child_count(channels_np);
	struct device_node *iter_np;

	/* Check channel count */
	if (num_channels != ODPM_CHANNEL_MAX) {
		pr_err("odpm: expected %d channels, got %d\n", ODPM_CHANNEL_MAX,
		       num_channels);
		return -EINVAL;
	}

	/* Parse channels */
	for_each_child_of_node(channels_np, iter_np) {
		const char *rail_name;

		/* Explicitly set enabled to false until we find the
		 * associated rail
		 */
		info->channels[channel_i].enabled = false;

		/* Read rail name */
		if (of_property_read_string(iter_np, "rail-name", &rail_name)) {
			pr_err("odpm: invalid rail-name value on %s\n",
			       iter_np->full_name);
			return -EINVAL;
		}

		/* Match rail name */
		for (rail_i = 0; rail_i < info->chip.num_rails; rail_i++) {
			if (!strcmp(info->chip.rails[rail_i].name, rail_name)) {
				info->channels[channel_i].rail_i = rail_i;
				break;
			}
		}

		if (rail_i == info->chip.num_rails) {
			pr_err("odpm: Could not find rail-name %s\n",
			       rail_name);
			return -EINVAL;
		}

		/* Check if the channel is enabled or not */
		info->channels[channel_i].enabled =
			of_property_read_bool(iter_np, "channel_enabled");

		channel_i++;
	}

	return 0;
}

static int odpm_parse_dt(struct device *dev, struct odpm_info *info)
{
	struct device_node *pmic_np = dev->parent->parent->of_node;
	struct device_node *odpm_np, *channels_np;
	u32 sample_rate;
	int ret;

	if (!pmic_np) {
		pr_err("odpm: cannot find parent DT node!\n");
		return -EINVAL;
	}
	odpm_np = of_find_node_by_name(pmic_np, "odpm");
	if (!odpm_np) {
		pr_err("odpm: cannot find main DT node!\n");
		return -EINVAL;
	}
	channels_np = of_find_node_by_name(pmic_np, "channels");
	if (!channels_np) {
		pr_err("odpm: cannot find channels DT node!\n");
		return -EINVAL;
	}

	/* Get main properties; sample-rate, chip-name, etc. */
	if (of_property_read_string(odpm_np, "chip-name", &info->chip.name)) {
		pr_err("odpm: cannot read sample rate value\n");
		return -EINVAL;
	}
	if (of_property_read_u32(odpm_np, "sample-rate-uhz", &sample_rate)) {
		pr_err("odpm: cannot read sample rate value\n");
		return -EINVAL;
	}
	if (!odpm_match_int_sample_rate(info, sample_rate)) {
		pr_err("odpm: cannot parse sample rate value %d\n",
		       sample_rate);
		return -EINVAL;
	}
	if (of_property_read_u32(odpm_np, "sample-rate-external-uhz",
				 &sample_rate)) {
		pr_err("odpm: cannot read external sample rate value\n");
		return -EINVAL;
	}
	if (!odpm_match_ext_sample_rate(info, sample_rate)) {
		pr_err("odpm: cannot parse external sample rate value %d\n",
		       sample_rate);
		return -EINVAL;
	}
	if (of_property_read_u32(odpm_np, "max-refresh-time-ms",
				 &info->chip.max_refresh_time_ms)) {
		pr_err("odpm: cannot read max refresh time value\n");
		return -EINVAL;
	}

	ret = odpm_parse_dt_rails(dev, info, pmic_np);
	if (ret != 0)
		return ret;

	return odpm_parse_dt_channels(info, channels_np);
}

static u64 odpm_calculate_uW_sec(struct odpm_info *info, int rail_i,
				 u64 acc_data)
{
	u32 sampling_frequency_uhz;
	u64 sampling_period_ms_iq30;
	u32 sampling_period_ms_iq22;
	u32 resolution_mW_iq30 = INVALID_RESOLUTION;
	u32 ret = 0;
	__uint128_t power_acc_mW_iq30;
	__uint128_t power_acc_uW_s_iq52;

	switch (info->chip.rails[rail_i].type) {
	case ODPM_RAIL_TYPE_REGULATOR: {
		int sample_rate_i = info->chip.int_sampling_rate_i;

		sampling_frequency_uhz =
			int_sample_rate_i_to_uhz[sample_rate_i];

		SWITCH_CHIP_FUNC(info, muxsel_to_power_resolution,
				 info->chip.rails[rail_i].mux_select);

	} break;
	case ODPM_RAIL_TYPE_SHUNT: {
		u64 resolution_W_iq60;
		int sampling_rate_i = info->chip.ext_sampling_rate_i;

		sampling_frequency_uhz =
			ext_sample_rate_i_to_uhz[sampling_rate_i];

		/* Losing a fraction of resolution performing u64 divisions,
		 * as there is no support for 128 bit divisions
		 */
		resolution_W_iq60 = ((u64)EXTERNAL_RESOLUTION_VRAIL *
				     (u64)EXTERNAL_RESOLUTION_VSHUNT *
				     (u64)EXTERNAL_RESOLUTION_TRIM) /
				    info->chip.rails[rail_i].shunt_uohms;

		/* Scale back to iq30 (with conversion to mW) */
		ret = _IQ30_to_int(resolution_W_iq60 * 1000);

	} break;
	}
	resolution_mW_iq30 = ret;

	/* Maintain as much precision as possible computing period in ms */
	sampling_period_ms_iq30 =
		_IQ30(u64, (u64)(1000 * 1000000)) / sampling_frequency_uhz;

	/* Allocate 10-bits in u32 for sample rate in ms (max: 1023 ms) */
	sampling_period_ms_iq22 = _IQ30_to_IQ22(sampling_period_ms_iq30);

	/* Use 128 bit to prevent overflow on multiplied iq values */
	power_acc_mW_iq30 = (__uint128_t)acc_data * resolution_mW_iq30;
	power_acc_uW_s_iq52 = power_acc_mW_iq30 * sampling_period_ms_iq22;

	/* Scale back u128 from iq to value */
	return _IQ22_to_int(_IQ30_to_int(power_acc_uW_s_iq52));
}

static int odpm_refresh_registers(struct odpm_info *info)
{
	const u64 tstamp_ms = ktime_get_boottime_ns() / 1000000;
	int ch;
	int ret = 0;

	u64 acc_data[ODPM_CHANNEL_MAX];
	u32 acc_count = 0;
	unsigned long jiffies_capture = 0;

	SWITCH_METER_FUNC(info, meter_load_measurement,
			  S2MPG1X_METER_POWER, acc_data, &acc_count,
			  &jiffies_capture);

	/* TODO(stayfan): b/156107234
	 * check for success; store values only if success
	 */
	info->jiffies_last_poll = jiffies_capture;
	info->chip.acc_timestamp = tstamp_ms;
	info->chip.acc_count += acc_count;

	for (ch = 0; ch < ODPM_CHANNEL_MAX; ch++) {
		const int rail_i = info->channels[ch].rail_i;
		const u64 uW_sec =
			odpm_calculate_uW_sec(info, rail_i, acc_data[ch]);

		info->chip.rails[rail_i].acc_power_uW_sec += uW_sec;
	}

	return 0;
}

static int odpm_take_snapshot(struct odpm_info *info)
{
	int ret = 0;

	/* check if the minimal elapsed time has passed and if so,
	 * re-read the chip, otherwise the cached info is just fine
	 */
	unsigned long future_time = info->jiffies_last_poll +
				    msecs_to_jiffies(ODPM_MIN_POLLING_TIME_MS);
	if (time_after(jiffies, future_time)) {
		/* we need to re-read the chip values
		 */
		ret = odpm_refresh_registers(info);
		if (ret < 0)
			pr_err("odpm: could not refresh registers!\n");

		/* re-schedule the work for the read registers timeout
		 * (to prevent chip regs saturation)
		 */
		ret = mod_timer(&info->timer_refresh,
				info->jiffies_last_poll +
				msecs_to_jiffies(info->chip.max_refresh_time_ms)
					);
		if (ret < 0)
			pr_err("odpm: read timer can't be modified!\n");
	}
	return ret;
}

static ssize_t sampling_rate_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct odpm_info *info = iio_priv(indio_dev);

	return scnprintf(buf, PAGE_SIZE, "%d.%d\n",
		int_sample_rate_i_to_uhz[info->chip.int_sampling_rate_i] /
			1000000,
		int_sample_rate_i_to_uhz[info->chip.int_sampling_rate_i] %
			1000000);
}

static ssize_t sampling_rate_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	int sampling_rate;

	if (kstrtoint(buf, 10, &sampling_rate)) {
		pr_err("sampling_rate is not a number\n");
		return -EINVAL;
	}
	/* TODO(stayfan): 156107511
	 * Write sample value to the driver
	 * Force a snapshot on chip
	 */

	return count;
}

static ssize_t ext_sampling_rate_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct odpm_info *info = iio_priv(indio_dev);

	return scnprintf(buf, PAGE_SIZE, "%d.%d\n",
		ext_sample_rate_i_to_uhz[info->chip.ext_sampling_rate_i] /
			1000000,
		ext_sample_rate_i_to_uhz[info->chip.ext_sampling_rate_i] %
			1000000);
}

static ssize_t ext_sampling_rate_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	int sampling_rate;

	if (kstrtoint(buf, 10, &sampling_rate)) {
		pr_err("sampling_rate is not a number\n");
		return -EINVAL;
	}
	/* TODO(stayfan): 156107511
	 * Write sample value to the driver
	 * Force a snapshot on chip
	 */

	return count;
}

static ssize_t energy_value_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct odpm_info *info = iio_priv(indio_dev);
	ssize_t count = 0;
	int ch;

	/* take snapshot */
	odpm_take_snapshot(info);

	/**
	 * Output format:
	 * <time since boot in ms>
	 * <Schematic name>, <energy value in uW-secs>
	 */
	count += scnprintf(buf + count, PAGE_SIZE - count, "%ld\n",
			   info->chip.acc_timestamp);

	for (ch = 0; ch < ODPM_CHANNEL_MAX; ch++) {
		int rail_i = info->channels[ch].rail_i;

		count += scnprintf(buf + count, PAGE_SIZE - count,
				   "CH%d[%s], %ld\n", ch,
				   info->chip.rails[rail_i].schematic_name,
				   info->chip.rails[rail_i].acc_power_uW_sec);
	}

	return count;
}

static ssize_t available_rails_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct odpm_info *info = iio_priv(indio_dev);
	int rail_i;
	ssize_t count = 0;

	for (rail_i = 0; rail_i < info->chip.num_rails; rail_i++) {
		struct odpm_rail_data *rail = &info->chip.rails[rail_i];

		count += scnprintf(buf + count, PAGE_SIZE - count,
				   "%s(%s):%s\n", rail->name,
				   rail->schematic_name, rail->subsys_name);
	}

	return count;
}

static ssize_t enabled_rails_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct odpm_info *info = iio_priv(indio_dev);
	ssize_t count = 0;
	int ch;

	for (ch = 0; ch < ODPM_CHANNEL_MAX; ch++) {
		if (info->channels[ch].enabled) {
			size_t rail_i = info->channels[ch].rail_i;
			struct odpm_rail_data *rail = &info->chip.rails[rail_i];

			count += scnprintf(buf + count, PAGE_SIZE - count,
					   "CH%d[%s]:%s\n", ch,
					   rail->schematic_name,
					   rail->subsys_name);
		}
	}

	return count;
}

static IIO_DEVICE_ATTR_RW(ext_sampling_rate, 0);
static IIO_DEVICE_ATTR_RW(sampling_rate, 0);
static IIO_DEVICE_ATTR_RO(energy_value, 0);
static IIO_DEVICE_ATTR_RO(available_rails, 0);
static IIO_DEVICE_ATTR_RO(enabled_rails, 0);

/**
 * TODO(stayfan): b/156109194
 * Add attributes that allow you to dynamically configure channels
 * You should be able to configure all 8 channels and initiate the config change
 * from a sysfs file, and the previous values from the static device tree config
 * should be buffered while the dynamic config is in place. There should also be
 * a sysfs file to revert back to the device tree config with the previous
 * accumulated values.
 */

#define ODPM_DEV_ATTR(name) (&iio_dev_attr_##name.dev_attr.attr)

static struct attribute *odpm_custom_attributes[] = {
	ODPM_DEV_ATTR(sampling_rate), ODPM_DEV_ATTR(ext_sampling_rate),
	ODPM_DEV_ATTR(energy_value),  ODPM_DEV_ATTR(available_rails),
	ODPM_DEV_ATTR(enabled_rails), NULL
};

static const struct attribute_group odpm_group = {
	.attrs = odpm_custom_attributes,
};

/* TODO(stayfan): b/156108825
 * Understand the requirements for read_raw/write_raw
 * Populate if these functions are useful to userland
 */
static int odpm_read_raw(struct iio_dev *indio_dev,
			 struct iio_chan_spec const *chan, int *val, int *val2,
			 long mask)
{
	int ret = 0;

	switch (mask) {
	case IIO_CHAN_INFO_AVERAGE_RAW:
		switch (chan->type) {
		case IIO_ENERGY:
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
	return ret;
}

static int odpm_write_raw(struct iio_dev *indio_dev,
			  struct iio_chan_spec const *chan, int val, int val2,
			  long mask)
{
	int ret = -EINVAL;

	if (iio_buffer_enabled(indio_dev))
		return -EBUSY;

	return ret;
}

static const struct iio_info odpm_iio_info = {
	.attrs = &odpm_group,
	.read_raw = odpm_read_raw,
	.write_raw = odpm_write_raw,
};

static int odpm_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(&pdev->dev);
	struct odpm_info *info = iio_priv(indio_dev);
	int ret;

	ret = try_to_del_timer_sync(&info->timer_refresh);
	if (ret < 0) {
		pr_err("odpm: cannot delete the refresh timer\n");
		return ret;
	}
	if (info->work_queue) {
		cancel_work_sync(&info->work_refresh);
		flush_workqueue(info->work_queue);
		destroy_workqueue(info->work_queue);
	}

	/* free the channel attributes memory */
	kfree(indio_dev->channels);

	iio_device_unregister(indio_dev);

	return ret;
}

static int odpm_probe(struct platform_device *pdev)
{
	struct odpm_info *odpm_info;
	struct iio_dev *indio_dev;
	int ret;
	void *iodev;

	pr_info("odpm: %s: init\n", pdev->name);

	/* Allocate the memory for our private structure
	 * related to the chip info structure
	 */
	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*odpm_info));
	if (!indio_dev) {
		pr_err("odpm: Could not allocate device!\n");
		return -ENOMEM;
	}
	platform_set_drvdata(pdev, indio_dev);

	/* Point our chip info structure towards
	 * the address freshly allocated
	 */
	odpm_info = iio_priv(indio_dev);
	/* Make the link between the parent driver and the IIO */
	iodev = dev_get_drvdata(pdev->dev.parent);
	if (!iodev) {
		pr_err("odpm: Could not get parent data!\n");
		return -ENODEV;
	}
	odpm_info->meter = iodev;

	if (!pdev->id_entry) {
		pr_err("odpm: Could not find id_entry!\n");
		odpm_remove(pdev);
		return -ENODEV;
	}

	switch (pdev->id_entry->driver_data) {
	case ODPM_CHIP_S2MPG10:
	case ODPM_CHIP_S2MPG11:
		odpm_info->chip.id = pdev->id_entry->driver_data;
		break;
	default:
		pr_err("odpm: Could not identify driver!\n");
		odpm_remove(pdev);
		return -ENODEV;
	}

	/* Read device tree data */
	if (!pdev->dev.parent->parent ||
	    (!of_get_next_child(pdev->dev.parent->parent->of_node, NULL))) {
		pr_err("odpm: DT does not exist!\n");
		odpm_remove(pdev);
		return -EINVAL;
	}
	/* Note: This function will call devm_kzalloc() in order to
	 * dynamically allocate memory for the rails
	 */
	ret = odpm_parse_dt(&pdev->dev, odpm_info);
	if (ret != 0) {
		pr_err("odpm: DT parsing error!\n");
		odpm_remove(pdev);
		return ret;
	}

	/* Initialize other data in odpm_info */

	/* Configure ODPM channels based on device tree input */
	ret = odpm_configure_chip(odpm_info);
	if (ret < 0) {
		odpm_remove(pdev);
		return ret;
	}

	/* Configure work to kick off every XHz */
	odpm_periodic_refresh_setup(odpm_info);

	/* Start measurement of default rails */
	odpm_configure_start_measurement(odpm_info);

	/* Setup IIO */
	indio_dev->channels = s2mpg1x_single_channel;
	indio_dev->num_channels = ARRAY_SIZE(s2mpg1x_single_channel);
	indio_dev->info = &odpm_iio_info;
	indio_dev->name = pdev->name;
	indio_dev->dev.parent = &pdev->dev;
	indio_dev->modes = INDIO_DIRECT_MODE;
	ret = devm_iio_device_register(&pdev->dev, indio_dev);
	if (ret < 0) {
		odpm_remove(pdev);
		return ret;
	}

	pr_info("odpm: %s: init completed\n", pdev->name);

	return ret;
}

static const struct platform_device_id odpm_id[] = {
	{ "s2mpg10-odpm", ODPM_CHIP_S2MPG10 },
	{ "s2mpg11-odpm", ODPM_CHIP_S2MPG11 },
	{},
};
MODULE_DEVICE_TABLE(platform, odpm_id);

static struct platform_driver odpm_driver = {
	.driver = {
		   .name = "odpm",
		   .owner = THIS_MODULE,
		   },
	.probe = odpm_probe,
	.remove = odpm_remove,
	.id_table = odpm_id,
};

static int __init odpm_init(void)
{
	return platform_driver_register(&odpm_driver);
}

subsys_initcall(odpm_init);

static void __exit odpm_exit(void)
{
	platform_driver_unregister(&odpm_driver);
}

module_exit(odpm_exit);

MODULE_DESCRIPTION("IIO ODPM Driver");
MODULE_AUTHOR("Stephane Lee <stayfan@google.com>");
MODULE_LICENSE("GPL");
