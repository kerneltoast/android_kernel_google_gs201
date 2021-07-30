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

#include <linux/mfd/samsung/s2mpg1x-meter.h>
#include <linux/mfd/samsung/s2mpg10-meter.h>
#include <linux/mfd/samsung/s2mpg11-meter.h>
#include <linux/mfd/samsung/s2mpg10.h>
#include <linux/mfd/samsung/s2mpg11.h>

#define ODPM_PRINT_ESTIMATED_CLOCK_SKEW 0

/* Cache accumulated values to prevent too frequent updates, allow a refresh
 * only every X ms.
 *
 * In order to prevent quantization error, we need a minimum count per time
 * interval to accurately estimate the frequency of the ADC polling. If we
 * could rely on the accuracy of the PMIC clock, then we would assume the
 * configured frequency, and this wouldn't be a consideration.
 *
 * At 20 counts, the quantization error may be +-(1/20)/2 = +-0.025 = +-2.5%.
 * At max frequency (1 KHz), the minimum refresh time is 20 ms.
 * At min frequency (7.628125 Hz), the minimum refresh time is ~2.62 s.
 *
 * Note 1: The ODPM will be refreshed regardless of minimum interval if an
 *         instantaneous refresh is called (odpm_take_snapshot_instant_locked)
 * Note 2: s2mpg1x chips can take on average between 1-2 ms for a refresh.
 */
#define ODPM_MIN_INTERVAL_ACC_COUNT 20 /* accumulator counts */

#define UHZ_PER_HZ 1000000

#define str(val) #val
#define xstr(s) str(s)
#define ODPM_RAIL_NAME_STR_LEN_MAX 49
#define ODPM_FREQ_DECIMAL_UHZ_STR_LEN_MAX 6
#define ODPM_SAMPLING_FREQ_CHAR_LEN_MAX 20

#define SWITCH_CHIP_FUNC(infop, ret, func, args...)                            \
	do {                                                                   \
		switch ((infop)->chip.id) {                                    \
		case ODPM_CHIP_S2MPG10:                                        \
			ret = s2mpg10_##func(args);                            \
			break;                                                 \
		case ODPM_CHIP_S2MPG11:                                        \
			ret = s2mpg11_##func(args);                            \
			break;                                                 \
		case ODPM_CHIP_COUNT:                                          \
			break;                                                 \
		}                                                              \
	} while (0)

#define SWITCH_METER_FUNC(infop, ret, func, args...) \
	SWITCH_CHIP_FUNC(infop, ret, func, info->meter, args)

/* At this moment, this driver supports a static 8 channels */
#define ODPM_CHANNEL_MAX S2MPG1X_METER_CHANNEL_MAX
#define ODPM_BUCK_EN_BYTES S2MPG1X_METER_BUCKEN_BUF

enum odpm_chip_id {
	ODPM_CHIP_S2MPG10,
	ODPM_CHIP_S2MPG11,
	ODPM_CHIP_COUNT,
};

enum odpm_rail_type {
	ODPM_RAIL_TYPE_REGULATOR_LDO,
	ODPM_RAIL_TYPE_REGULATOR_BUCK,
	ODPM_RAIL_TYPE_SHUNT,
};

enum odpm_sampling_rate_type {
	ODPM_SAMPLING_RATE_INTERNAL,
	ODPM_SAMPLING_RATE_EXTERNAL,
	ODPM_SAMPLING_RATE_ALL,
};

struct odpm_rail_data {
	/* Config */
	const char *name;
	const char *schematic_name;
	const char *subsys_name;
	enum odpm_rail_type type;
	u32 mux_select;

	/* Buck specific */
	int channel_en_byte_offset;

	/* External rail specific config */
	u32 shunt_uohms;

	/* Bucks and external rails */
	u8 channel_en_index;

	/* Data */
	u64 acc_power_uW_sec_cached;
	u64 measurement_stop_ms;
	u64 measurement_start_ms_cached;

	bool disable_in_sleep;
};

struct odpm_chip {
	/* Config */
	const char *name;
	enum odpm_chip_id id;
	enum s2mpg1x_id hw_id;
	int hw_rev;
	u32 max_refresh_time_ms;

	int num_rails;
	struct odpm_rail_data *rails;

	const u32 *sampling_rate_int_uhz;
	int sampling_rate_int_count;
	const u32 *sampling_rate_ext_uhz;
	int sampling_rate_ext_count;

	s2mpg1x_int_samp_rate int_config_sampling_rate_i;
	s2mpg1x_ext_samp_rate ext_config_sampling_rate_i;

	/* Data */
	u64 acc_timestamp_ms;
	s2mpg1x_int_samp_rate int_sampling_rate_i;
	s2mpg1x_ext_samp_rate ext_sampling_rate_i;

	bool rx_ext_config_confirmation;
};

struct odpm_channel_data {
	int rail_i;
	bool enabled;

	u64 measurement_start_ms;
	u64 acc_power_uW_sec;
};

/**
 * dynamic struct odpm_info
 */
struct odpm_info {
	struct odpm_chip chip;
	void *meter; /* Parent meter device data */
	struct i2c_client *i2c;
	struct mutex lock; /* Global HW lock */

	struct odpm_channel_data channels[ODPM_CHANNEL_MAX];

	struct workqueue_struct *work_queue;
	struct work_struct work_refresh;
	struct timer_list timer_refresh;

	u64 last_poll_ktime_boot_ns;
	bool sleeping;
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
static int odpm_take_snapshot_locked(struct odpm_info *info);
static int odpm_take_snapshot_instant_locked(struct odpm_info *info,
					     bool resume);

static void odpm_print_new_sampling_rate(struct odpm_info *info, int ret,
					 enum odpm_sampling_rate_type type);

static u64 to_ms(u64 ns)
{
	return ns / NSEC_PER_MSEC;
}

static int odpm_io_set_channel(struct odpm_info *info, int channel)
{
	int ret = -1;
	int rail_i = info->channels[channel].rail_i;

	pr_info("odpm: %s: CH%d=%s\n", info->chip.name, channel,
		info->chip.rails[rail_i].schematic_name);

	SWITCH_METER_FUNC(info, ret, meter_set_muxsel, channel,
			  info->chip.rails[rail_i].mux_select);
	return ret;
}

static int odpm_io_set_int_sampling_rate(struct odpm_info *info,
					 s2mpg1x_int_samp_rate hz)
{
	if (hz >= S2MPG1X_INT_FREQ_COUNT)
		return -1;

	info->chip.int_sampling_rate_i = hz;
	return s2mpg1x_meter_set_int_samp_rate(info->chip.hw_id, info->i2c, hz);
}

static int odpm_io_set_ext_sampling_rate(struct odpm_info *info,
					 s2mpg1x_ext_samp_rate hz)
{
	if (hz >= S2MPG1X_EXT_FREQ_COUNT)
		return -1;

	info->chip.ext_sampling_rate_i = hz;
	return s2mpg1x_meter_set_ext_samp_rate(info->chip.hw_id, info->i2c, hz);
}

static int odpm_io_set_meter_on(struct odpm_info *info, bool is_on)
{
	int ret = -1;

	SWITCH_METER_FUNC(info, ret, meter_onoff, is_on);
	return ret;
}

static int odpm_io_set_ext_meter_on(struct odpm_info *info, bool is_on)
{
	int ret = -1;

	SWITCH_METER_FUNC(info, ret, ext_meter_onoff, is_on);
	return ret;
}

static int odpm_io_set_ext_channels_en(struct odpm_info *info, u8 channels)
{
	return s2mpg1x_meter_set_ext_channel_en(info->chip.hw_id, info->i2c,
						channels);
}

static int odpm_io_set_buck_channels_en(struct odpm_info *info, u8 *channels,
					int num_bytes)
{
	/* Disable BUCK5M if necessary (A0-specific) */
	if (info->chip.hw_id == ID_S2MPG10 &&
	    info->chip.hw_rev == S2MPG10_EVT0) {
		channels[0] &= ~0x10;
	}

	return s2mpg1x_meter_set_buck_channel_en(info->chip.hw_id, info->i2c,
						 channels, num_bytes);
}

static int odpm_io_send_blank_async(struct odpm_info *info,
				    u64 *timestamp_capture)
{
	return s2mpg1x_meter_set_async_blocking(info->chip.hw_id, info->i2c,
						timestamp_capture);
}

static int odpm_io_update_ext_enable_bits(struct odpm_info *info)
{
	/* As of b/181251561, we permanently enable all 3 external rail bits */
	/* This must be done while the ext meter en bit is set to 0 */
	return odpm_io_set_ext_channels_en(info, EXT_METER_CHANNEL_EN_ALL);
}

static int odpm_io_update_bucken_enable_bits(struct odpm_info *info,
					     bool on_sleep)
{
	u8 buck_channels_en[ODPM_BUCK_EN_BYTES] = { 0 };
	int ch;

	for (ch = 0; ch < ODPM_CHANNEL_MAX; ch++) {
		int rail_i = info->channels[ch].rail_i;
		struct odpm_rail_data *rail = &info->chip.rails[rail_i];

		u8 channel_en_i = rail->channel_en_index;
		int channel_offset_byte = rail->channel_en_byte_offset;

		if (!info->channels[ch].enabled)
			continue;

		/* Skip the enable bit if bucks should be disabled in sleep */
		if (on_sleep && rail->disable_in_sleep)
			continue;

		if (rail->type == ODPM_RAIL_TYPE_REGULATOR_BUCK &&
		    channel_offset_byte < ODPM_BUCK_EN_BYTES)
			buck_channels_en[channel_offset_byte] |= channel_en_i;
	}

	return odpm_io_set_buck_channels_en(info, buck_channels_en,
					    ODPM_BUCK_EN_BYTES);
}

int odpm_configure_chip(struct odpm_info *info)
{
	int ch;
	int ret;

	/* TODO(stayfan): b/156107234
	 * error conditions
	 */
	ret = odpm_io_set_int_sampling_rate(info,
					    info->chip.int_config_sampling_rate_i);
	odpm_print_new_sampling_rate(info, ret, ODPM_SAMPLING_RATE_INTERNAL);
	ret = odpm_io_set_ext_sampling_rate(info,
					    info->chip.ext_config_sampling_rate_i);
	odpm_print_new_sampling_rate(info, ret, ODPM_SAMPLING_RATE_EXTERNAL);

	for (ch = 0; ch < ODPM_CHANNEL_MAX; ch++) {
		if (info->channels[ch].enabled)
			odpm_io_set_channel(info, ch);
	}

	odpm_io_update_bucken_enable_bits(info, false /* on_sleep */);

	odpm_io_set_ext_meter_on(info, false);
	odpm_io_update_ext_enable_bits(info);

	odpm_io_set_meter_on(info, true);
	odpm_io_set_ext_meter_on(info, true);

	return 0;
}

int odpm_configure_start_measurement(struct odpm_info *info)
{
	u64 timestamp_capture_ns = 0;
	int ch;

	/* For s2mpg1x chips, clear ACC registers */
	int ret = odpm_io_send_blank_async(info, &timestamp_capture_ns);

	info->last_poll_ktime_boot_ns = timestamp_capture_ns;

	pr_info("odpm: Starting at timestamp (ms): %ld\n",
		to_ms(timestamp_capture_ns));

	/* Initialize boot measurement time to 0. This means that there will be
	 * some amount of time from boot where power measurements are not
	 * accounted for, because the driver needs to load and wait for the
	 * HAL to initialize (and send "CONFIG_COMPLETE"). Therefore, the true
	 * measurement duration will be incorrect at boot (duration = boot
	 * timestamp - start time), since we don't start truly measuring power
	 * until the HAL starts.
	 *
	 * The approximate unaccounted time was measured to be ~ 15s when this
	 * commit was created.
	 */
	for (ch = 0; ch < ODPM_CHANNEL_MAX; ch++) {
		if (info->channels[ch].enabled)
			info->channels[ch].measurement_start_ms = 0;
	}

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

	if (odpm_take_snapshot(info) < 0)
		pr_err("odpm: Cannot refresh %s registers periodically!\n",
		       info->chip.name);
	else
		pr_info("odpm: Refreshed %s registers!\n", info->chip.name);
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

static bool odpm_match_int_sampling_rate(struct odpm_info *info, u32 sampling_rate,
					 int *index)
{
	bool success = false;
	int i;

	for (i = 0; i < info->chip.sampling_rate_int_count; i++) {
		if (sampling_rate == info->chip.sampling_rate_int_uhz[i]) {
			*index = i;
			success = true;
			break;
		}
	}

	return success;
}

static bool odpm_match_ext_sampling_rate(struct odpm_info *info, u32 sampling_rate,
					 int *index)
{
	bool success = false;
	int i;

	for (i = 0; i < info->chip.sampling_rate_ext_count; i++) {
		if (sampling_rate == info->chip.sampling_rate_ext_uhz[i]) {
			*index = i;
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
	else if (of_property_read_bool(node, "buck_rail"))
		rail_data->type = ODPM_RAIL_TYPE_REGULATOR_BUCK;
	else
		rail_data->type = ODPM_RAIL_TYPE_REGULATOR_LDO;

	if (of_property_read_u32(node, "channel-mux-selection",
				 &rail_data->mux_select)) {
		pr_err("odpm: cannot read channel-mux-selection\n");
		return -EINVAL;
	}

	rail_data->disable_in_sleep =
		of_property_read_bool(node, "odpm_disable_in_sleep");

	switch (rail_data->type) {
	case ODPM_RAIL_TYPE_SHUNT: {
		u32 channel_en_index;

		if (of_property_read_u32(node, "shunt-res-uohms",
					 &rail_data->shunt_uohms)) {
			pr_err("odpm: cannot read shunt-res-uohms\n");
			return -EINVAL;
		}

		if (of_property_read_u32(node, "channel-en-index",
					 &channel_en_index)) {
			pr_err("odpm: cannot read channel-en-index\n");
			return -EINVAL;
		}
		rail_data->channel_en_index = channel_en_index;
	} break;
	case ODPM_RAIL_TYPE_REGULATOR_BUCK: {
		u32 channel_en_index;
		int channel_en_byte_offset;

		if (of_property_read_u32(node, "channel-en-index",
					 &channel_en_index)) {
			pr_err("odpm: cannot read channel-en-index\n");
			return -EINVAL;
		}
		rail_data->channel_en_index = channel_en_index;

		if (of_property_read_u32(node, "channel-en-byte-offset",
					 &channel_en_byte_offset)) {
			pr_err("odpm: cannot read channel-en-byte-offset\n");
			return -EINVAL;
		}
		rail_data->channel_en_byte_offset = channel_en_byte_offset;
	} break;

	case ODPM_RAIL_TYPE_REGULATOR_LDO:
	default:
		break;
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
	u32 sampling_rate;
	int sampling_rate_i;
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
	if (of_property_read_u32(odpm_np, "sample-rate-uhz", &sampling_rate)) {
		pr_err("odpm: cannot read sample rate value\n");
		return -EINVAL;
	}
	if (!odpm_match_int_sampling_rate(info, sampling_rate, &sampling_rate_i)) {
		pr_err("odpm: cannot parse sample rate value %d\n",
		       sampling_rate);
		return -EINVAL;
	}
	info->chip.int_config_sampling_rate_i = sampling_rate_i;
	info->chip.int_sampling_rate_i = sampling_rate_i;
	if (of_property_read_u32(odpm_np, "sample-rate-external-uhz",
				 &sampling_rate)) {
		pr_err("odpm: cannot read external sample rate value\n");
		return -EINVAL;
	}
	if (!odpm_match_ext_sampling_rate(info, sampling_rate, &sampling_rate_i)) {
		pr_err("odpm: cannot parse external sample rate value %d\n",
		       sampling_rate);
		return -EINVAL;
	}
	info->chip.ext_config_sampling_rate_i = sampling_rate_i;
	info->chip.ext_sampling_rate_i = sampling_rate_i;
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
				 u64 acc_data, u32 int_sampling_frequency_uhz)
{
	u64 sampling_period_ms_iq30;
	u32 sampling_period_ms_iq22;
	u32 resolution_mW_iq30 = INVALID_RESOLUTION;
	u32 ret = 0;
	__uint128_t power_acc_mW_iq30;
	__uint128_t power_acc_uW_s_iq52;

	switch (info->chip.rails[rail_i].type) {
	case ODPM_RAIL_TYPE_REGULATOR_BUCK:
	case ODPM_RAIL_TYPE_REGULATOR_LDO:
	default: {
		SWITCH_CHIP_FUNC(info, ret, muxsel_to_power_resolution,
				 info->chip.rails[rail_i].mux_select);

	} break;
	case ODPM_RAIL_TYPE_SHUNT: {
		u64 resolution_W_iq60;

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
		_IQ30(u64, (u64)(MSEC_PER_SEC * UHZ_PER_HZ)) /
		int_sampling_frequency_uhz;

	/* Allocate 10-bits in u32 for sample rate in ms (max: 1023 ms) */
	sampling_period_ms_iq22 = _IQ30_to_IQ22(sampling_period_ms_iq30);

	/* Use 128 bit to prevent overflow on multiplied iq values */
	power_acc_mW_iq30 = (__uint128_t)acc_data * resolution_mW_iq30;
	power_acc_uW_s_iq52 = power_acc_mW_iq30 * sampling_period_ms_iq22;

	/* Scale back u128 from iq to value */
	return _IQ22_to_int(_IQ30_to_int(power_acc_uW_s_iq52));
}

#if ODPM_PRINT_ESTIMATED_CLOCK_SKEW
static void odpm_print_clock_skew(struct odpm_info *info, u64 elapsed_ms,
				  u32 acc_count)
{
	u64 uHz_estimated = ((u64)acc_count * (MSEC_PER_SEC * UHZ_PER_HZ)) /
		elapsed_ms;
	int i = info->chip.int_sampling_rate_i;
	u64 uHz = info->chip.sampling_rate_int_uhz[i];

	u64 ratio_u = (1000000 * uHz_estimated) / uHz;
	s64 pct_u = (((s64)ratio_u - (1 * 1000000)) * 100);

	pr_info("odpm: %s: elapsed_ms: %d, acc_count: %d\n", info->chip.name,
		elapsed_ms, acc_count);
	pr_info("odpm: %s: internal clock skew: %d.%06d %%\n", info->chip.name,
		pct_u / 1000000, abs(pct_u) % 1000000);
}
#endif

static u32 odpm_estimate_sampling_frequency(struct odpm_info *info,
					    unsigned int elapsed_ms,
					    unsigned int elapsed_refresh_ms,
					    u32 acc_count)
{
	/* b/156680376
	 * Instead of using the configured sampling rate, we want to approximate
	 * the sampling rate based on acc_count
	 */

	u64 sampling_frequency_estimated_uhz;

	int i = info->chip.int_sampling_rate_i;
	u64 sampling_frequency_table_uhz = info->chip.sampling_rate_int_uhz[i];
	u64 freq_lower_bound;
	u64 freq_upper_bound;

	/* Don't bother calculating frequency when there aren't enough samples,
	 * otherwise we'd accumulate large quantization error. This may occur
	 * nominally if two instantaneous requests are followed back-to-back.
	 */
	if (acc_count < ODPM_MIN_INTERVAL_ACC_COUNT) {
		return sampling_frequency_table_uhz;
	}

	if (elapsed_ms == 0) {
		pr_err("odpm: %s: elapsed time is 0 ms\n", info->chip.name);

		/* Fall back to configured frequency */
		return sampling_frequency_table_uhz;
	}

#if ODPM_PRINT_ESTIMATED_CLOCK_SKEW
	odpm_print_clock_skew(info, elapsed_ms, acc_count);
#endif

	/* Estimate sampling rate based on acc_count */
	sampling_frequency_estimated_uhz =
		((u64)acc_count * (MSEC_PER_SEC * UHZ_PER_HZ)) / elapsed_ms;

	/**
	 * 100 ms check on register refresh...
	 * We want to verify that the transaction time isn't too long, as the
	 * process may have been pre-empted between the command to refresh
	 * registers and when the timestamp was captured.
	 */
	if (elapsed_refresh_ms >= 100) {
		pr_err("odpm: %s: refresh registers took too long; %ld ms\n",
		       info->chip.name, elapsed_refresh_ms);

		/* Fall back to configured frequency */
		return sampling_frequency_table_uhz;
	}

	/* +-12.5% error bounds check, per the datasheets, to ensure that our
	 * calculations are no worse than is expected from the configured
	 * frequency. This also ensures the new frequency fits in a u32 (max
	 * 1000 000 000 uHz * 1.125).
	 */
	freq_lower_bound = sampling_frequency_table_uhz * 7 / 8;
	freq_upper_bound = sampling_frequency_table_uhz * 9 / 8;
	if (sampling_frequency_estimated_uhz < freq_lower_bound ||
	    sampling_frequency_estimated_uhz > freq_upper_bound) {
		pr_err("odpm: %s: clock error too large! fsel: %d, fest: %ld, elapsed_ms: %d, acc_count: %d\n",
		       info->chip.name, sampling_frequency_table_uhz,
		       sampling_frequency_estimated_uhz,
		       elapsed_ms, acc_count);

		/* Fall back to configured frequency */
		return sampling_frequency_table_uhz;
	}

	return sampling_frequency_estimated_uhz;
}

static int odpm_refresh_registers(struct odpm_info *info, bool resume)
{
	int ch;
	int ret = 0;

	u64 acc_data[ODPM_CHANNEL_MAX];
	u32 acc_count = 0;
	u32 sampling_frequency_uhz;

	u64 timestamp_previous_sample = info->last_poll_ktime_boot_ns;
	u64 timestamp_before_async;
	u64 timestamp_after_async;

	unsigned int elapsed_ms;
	unsigned int elapsed_refresh_ms;

	timestamp_before_async = ktime_get_boottime_ns();

	SWITCH_METER_FUNC(info, ret, meter_load_measurement,
			  S2MPG1X_METER_POWER, acc_data, &acc_count,
			  &timestamp_after_async);

	if (ret < 0) {
		pr_err("odpm: %s: i2c error; count not measure interval\n",
		       info->chip.name);
		goto exit_refresh;
	}

	/* Store timestamps - the rest of the function will succeed */
	info->last_poll_ktime_boot_ns = timestamp_after_async;
	info->chip.acc_timestamp_ms = to_ms(timestamp_after_async);

	elapsed_ms = to_ms(timestamp_after_async - timestamp_previous_sample);
	elapsed_refresh_ms =
		to_ms(timestamp_after_async - timestamp_before_async);

	sampling_frequency_uhz =
		odpm_estimate_sampling_frequency(info,
						 elapsed_ms,
						 elapsed_refresh_ms,
						 acc_count);

	for (ch = 0; ch < ODPM_CHANNEL_MAX; ch++) {
		const int rail_i = info->channels[ch].rail_i;
		struct odpm_rail_data *rail = &info->chip.rails[rail_i];
		u64 uW_sec;

		/* Do not add energy on rails that were disabled during sleep */
		if (resume && rail->disable_in_sleep)
			continue;

		uW_sec = odpm_calculate_uW_sec(info, rail_i, acc_data[ch],
					       (u32)sampling_frequency_uhz);

		info->channels[ch].acc_power_uW_sec += uW_sec;
	}

exit_refresh:
	return ret;
}

static int odpm_reset_timer(struct odpm_info *info)
{
	unsigned long future_timer = jiffies +
		msecs_to_jiffies(info->chip.max_refresh_time_ms);

	/* re-schedule the work for the read registers timeout
	 * (to prevent chip regs saturation)
	 */
	int ret_timer = mod_timer(&info->timer_refresh, future_timer);

	if (ret_timer < 0)
		pr_err("odpm: read timer can't be modified!\n");

	return ret_timer;
}

static int odpm_take_snapshot(struct odpm_info *info)
{
	int ret;

	mutex_lock(&info->lock);
	ret = odpm_take_snapshot_locked(info);
	mutex_unlock(&info->lock);
	return ret;
}

static int odpm_take_snapshot_locked(struct odpm_info *info)
{
	/* check if the minimal elapsed time has passed and if so,
	 * re-read the chip, otherwise the cached info is just fine
	 */

	int i = info->chip.int_sampling_rate_i;
	u32 freq = info->chip.sampling_rate_int_uhz[i];
	u64 min_time_ns = (u64)ODPM_MIN_INTERVAL_ACC_COUNT * NSEC_PER_SEC *
		UHZ_PER_HZ / freq;
	u64 now_ns = ktime_get_boottime_ns();

	if (now_ns > info->last_poll_ktime_boot_ns + min_time_ns)
		return odpm_take_snapshot_instant_locked(info, false);

	return 0; /* Refresh skipped for min elapsed time */
}

static int odpm_take_snapshot_instant_locked(struct odpm_info *info,
					     bool resume)
{
	int ret_refresh;
	int ret_timer = odpm_reset_timer(info);

	if (info->sleeping) {
		pr_err("odpm: tried to refresh registers while sleeping!\n");
		return -1;
	}

	/* we need to re-read the chip values */
	ret_refresh = odpm_refresh_registers(info, resume);

	if (ret_refresh < 0) {
		pr_err("odpm: could not refresh registers!\n");
		return ret_refresh;
	}

	return ret_timer;
}

static void odpm_suspend_resume_operations(struct odpm_info *info, bool suspend)
{
	int ch;
	int ret;

	mutex_lock(&info->lock);

	if (suspend) {
		/* Send a refresh and store values */
		if (odpm_take_snapshot_instant_locked(info, false) < 0) {
			pr_err("odpm: cannot capture snapshot for suspend\n");
			goto suspend_resume_exit;
		}

		/* In case it saves on power, we set the muxsels for these bucks
		 * to 0x00.
		 */
		for (ch = 0; ch < ODPM_CHANNEL_MAX; ch++) {
			const int rail_i = info->channels[ch].rail_i;
			struct odpm_rail_data *rail = &info->chip.rails[rail_i];

			if (info->channels[ch].enabled &&
			    rail->disable_in_sleep)
				SWITCH_METER_FUNC(info, ret, meter_set_muxsel,
						  ch, MUXSEL_NONE);
		}

		/* Disable the powermeter for the bucks with disable_in_sleep */
		odpm_io_update_bucken_enable_bits(info, true /* on_sleep */);

		/* Prevents any external device from refreshing registers */
		info->sleeping = true;

	} else { /* resume: opposite operations */
		info->sleeping = false;

		/* Re-enable the bucks */
		odpm_io_update_bucken_enable_bits(info, false /* on_sleep */);

		/* Re-select the muxsel for the bucks */
		for (ch = 0; ch < ODPM_CHANNEL_MAX; ch++) {
			const int rail_i = info->channels[ch].rail_i;
			struct odpm_rail_data *rail = &info->chip.rails[rail_i];

			if (info->channels[ch].enabled &&
			    rail->disable_in_sleep)
				SWITCH_METER_FUNC(info, ret, meter_set_muxsel,
						  ch, rail->mux_select);
		}

		/* This refresh will erase the last data interval for specific
		 * bucks that have disable_in_sleep.
		 */
		if (odpm_take_snapshot_instant_locked(info, true /* resume */)
			< 0) {
			pr_err("odpm: cannot capture snapshot for resume\n");
			goto suspend_resume_exit;
		}
	}

suspend_resume_exit:
	mutex_unlock(&info->lock);
}

/**
 * Verify and return sampling rate
 *
 * @return Sampling rate if >= 0, else error number
 */
static int odpm_sampling_rate_verify(const char *buf)
{
	int hz;
	int i;
	char decimal_str[ODPM_FREQ_DECIMAL_UHZ_STR_LEN_MAX + 1];
	int decimal_len = 0;
	int decimal_uHz = 0;

	decimal_str[0] = 0;

	if (sscanf(buf, "%d.%" xstr(ODPM_FREQ_DECIMAL_UHZ_STR_LEN_MAX) "s", &hz,
		   decimal_str) != 2) {
		decimal_str[0] = 0;

		if (kstrtoint(buf, 10, &hz)) {
			pr_err("odpm: sampling rate is not a number\n");
			return -EINVAL;
		}
	}

	decimal_len = strlen(decimal_str);

	/* Got a decimal value */
	if (decimal_len > 0) {
		const int power_of_10 =
			(ODPM_FREQ_DECIMAL_UHZ_STR_LEN_MAX - decimal_len);

		/* Parse integer */
		if (kstrtoint(decimal_str, 10, &decimal_uHz)) {
			pr_err("odpm: sampling rate decimal is not a number\n");
			return -EINVAL;
		}

		/* Multiply by powers of 10 */
		for (i = 0; i < power_of_10; i++)
			decimal_uHz *= 10;
	}

	return UHZ_PER_HZ * hz + decimal_uHz;
}

static ssize_t scnprintf_sampling_rate(char *buf, size_t size, u32 freq)
{
	return scnprintf(buf, size, "%d.%06d\n", freq / UHZ_PER_HZ,
		freq % UHZ_PER_HZ);
}

static void odpm_print_new_sampling_rate(struct odpm_info *info, int ret,
					 enum odpm_sampling_rate_type type)
{
	char buf[ODPM_SAMPLING_FREQ_CHAR_LEN_MAX];
	u32 freq = 0;

	if (ret < 0) {
		pr_warn("odpm: cannot apply sampling frequency type: %d\n",
			type);
		return;
	}

	if (type == ODPM_SAMPLING_RATE_INTERNAL) {
		int i = info->chip.int_sampling_rate_i;

		freq = info->chip.sampling_rate_int_uhz[i];
	} else if (type == ODPM_SAMPLING_RATE_EXTERNAL) {
		int i = info->chip.ext_sampling_rate_i;

		freq = info->chip.sampling_rate_ext_uhz[i];
	} else {
		pr_warn("odpm: unsupported frequency type: %d\n", type);
		return;
	}

	/* Returns with a new line */
	scnprintf_sampling_rate(buf, ODPM_SAMPLING_FREQ_CHAR_LEN_MAX, freq);
	pr_info("odpm: %s: Applied new sampling frequency (type %d) in Hz: %s",
		info->chip.name, type, buf);
}

static void odpm_set_sampling_rate(struct odpm_info *info,
				   enum odpm_sampling_rate_type type,
		s2mpg1x_int_samp_rate int_sampling_rate_i,
		s2mpg1x_ext_samp_rate ext_sampling_rate_i)
{
	int ret = 0;

	mutex_lock(&info->lock);

	/* Send a refresh and store values */
	if (odpm_take_snapshot_instant_locked(info, false) < 0) {
		pr_err("odpm: cannot refresh to apply new sampling rate\n");
		goto sampling_rate_store_exit;
	}

	if (type == ODPM_SAMPLING_RATE_INTERNAL || type == ODPM_SAMPLING_RATE_ALL) {
		ret = odpm_io_set_int_sampling_rate(info, int_sampling_rate_i);
		odpm_print_new_sampling_rate(info, ret, ODPM_SAMPLING_RATE_INTERNAL);
	}
	if (type == ODPM_SAMPLING_RATE_EXTERNAL || type == ODPM_SAMPLING_RATE_ALL) {
		ret = odpm_io_set_ext_sampling_rate(info, ext_sampling_rate_i);
		odpm_print_new_sampling_rate(info, ret, ODPM_SAMPLING_RATE_EXTERNAL);
	}

	/* Send blank ASYNC, ignoring the latest set of data */
	if (odpm_io_send_blank_async(info, &info->last_poll_ktime_boot_ns) < 0)
		pr_err("odpm: Could not send blank async when applying sampling rate\n");

sampling_rate_store_exit:
	mutex_unlock(&info->lock);
}

static ssize_t sampling_rate_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct odpm_info *info = iio_priv(indio_dev);
	int i = info->chip.int_sampling_rate_i;
	u32 freq = info->chip.sampling_rate_int_uhz[i];

	return scnprintf_sampling_rate(buf, PAGE_SIZE, freq);
}

static ssize_t sampling_rate_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct odpm_info *info = iio_priv(indio_dev);

	int new_sampling_rate_i;
	int new_sampling_rate;

	int ret = odpm_sampling_rate_verify(buf);

	if (ret < 0)
		return ret;

	new_sampling_rate = ret;
	if (!odpm_match_int_sampling_rate(info, new_sampling_rate,
					  &new_sampling_rate_i)) {
		pr_err("odpm: cannot match new sampling rate value; %d uHz\n",
		       new_sampling_rate);
		return -EINVAL;
	}

	odpm_set_sampling_rate(info, ODPM_SAMPLING_RATE_INTERNAL,
			       new_sampling_rate_i, S2MPG1X_EXT_FREQ_NONE);

	return count;
}

static ssize_t ext_sampling_rate_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct odpm_info *info = iio_priv(indio_dev);
	int i = info->chip.ext_sampling_rate_i;
	u32 freq = info->chip.sampling_rate_ext_uhz[i];

	return scnprintf_sampling_rate(buf, PAGE_SIZE, freq);
}

static ssize_t ext_sampling_rate_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct odpm_info *info = iio_priv(indio_dev);

	int new_sampling_rate_i;
	int new_sampling_rate;

	int ret = odpm_sampling_rate_verify(buf);

	if (ret < 0)
		return ret;

	new_sampling_rate = ret;
	if (!odpm_match_ext_sampling_rate(info, new_sampling_rate,
					  &new_sampling_rate_i)) {
		pr_err("odpm: cannot match new sampling rate value; %d uHz\n",
		       new_sampling_rate);
		return -EINVAL;
	}

	odpm_set_sampling_rate(info, ODPM_SAMPLING_RATE_EXTERNAL,
			       S2MPG1X_INT_FREQ_NONE, new_sampling_rate_i);

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
	mutex_lock(&info->lock);
	if (odpm_take_snapshot_locked(info) < 0) {
		pr_err("odpm: cannot retrieve energy values");
		goto energy_value_show_exit;
	}

	/**
	 * Output format:
	 * t=<Measurement timestamp, ms>
	 * CH<N>(T=<Duration, ms>)[<Schematic name>], <Accumulated Energy, uWs>
	 */
	count += scnprintf(buf + count, PAGE_SIZE - count, "t=%ld\n",
			   info->chip.acc_timestamp_ms);

	for (ch = 0; ch < ODPM_CHANNEL_MAX; ch++) {
		int rail_i = info->channels[ch].rail_i;
		u64 start_ms = info->channels[ch].measurement_start_ms;
		u64 duration_ms = 0;

		if (info->chip.acc_timestamp_ms >= start_ms)
			duration_ms = info->chip.acc_timestamp_ms - start_ms;

		count += scnprintf(buf + count, PAGE_SIZE - count,
				   "CH%d(T=%ld)[%s], %ld\n", ch,
				   duration_ms,
				   info->chip.rails[rail_i].schematic_name,
				   info->channels[ch].acc_power_uW_sec);
	}

energy_value_show_exit:
	mutex_unlock(&info->lock);

	return count;
}

static ssize_t available_rails_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct odpm_info *info = iio_priv(indio_dev);
	int rail_i;
	ssize_t count = 0;

	/* No locking needed for rail specific information */
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

	mutex_lock(&info->lock);
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
	mutex_unlock(&info->lock);

	return count;
}

static ssize_t enabled_rails_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct odpm_info *info = iio_priv(indio_dev);

	char rail_name[ODPM_RAIL_NAME_STR_LEN_MAX + 1];
	int channel = -1;
	bool channel_valid = false;
	int rail_i;
	int scan_result;
	int current_rail;
	int new_rail;
	u64 timestamp_ns;

	if (strcmp(buf, "CONFIG_COMPLETE") == 0) {
		/**
		 * External configuration applied, so reset the timestamps and
		 * measurements. This can only be done once from boot.
		 */
		mutex_lock(&info->lock);
		if (!info->chip.rx_ext_config_confirmation) {
			info->chip.rx_ext_config_confirmation = true;
			odpm_reset_timer(info);
			if (odpm_configure_start_measurement(info))
				pr_err("odpm: Failed to start measurement\n");
			else
				pr_info("odpm: Boot config complete!\n");
		} else {
			pr_err("odpm: Boot config already applied\n");
		}
		mutex_unlock(&info->lock);

		return count;
	}

	scan_result = sscanf(buf, "CH%d=%" xstr(ODPM_RAIL_NAME_STR_LEN_MAX) "s",
			     &channel, rail_name);

	if (scan_result == 2 && channel >= 0 && channel < ODPM_CHANNEL_MAX) {
		for (rail_i = 0; rail_i < info->chip.num_rails; rail_i++) {
			if (strcmp(info->chip.rails[rail_i].name, rail_name) ==
			    0) {
				channel_valid = true;
				break;
			}
		}
	}

	if (!channel_valid)
		return -EINVAL; /* The buffer syntax was invalid */

	mutex_lock(&info->lock);
	new_rail = rail_i;
	current_rail = info->channels[channel].rail_i;

	if (new_rail == current_rail) {
		/* Do not apply rail selection if the same rail is being
		 * replaced.
		 */
		goto enabled_rails_store_exit;
	}

	/* Send a refresh and store values for old rails */
	if (odpm_take_snapshot_locked(info) < 0) {
		pr_err("odpm: cannot refresh values to swap rails\n");
		goto enabled_rails_store_exit;
	}

	/* Capture measurement time for current rail */
	info->chip.rails[current_rail].measurement_start_ms_cached =
		info->channels[channel].measurement_start_ms;
	info->chip.rails[current_rail].measurement_stop_ms =
		info->chip.acc_timestamp_ms;

	/* Reset stored energy for channel */
	info->chip.rails[current_rail].acc_power_uW_sec_cached =
		info->channels[channel].acc_power_uW_sec;
	info->channels[channel].acc_power_uW_sec = 0;

	/* Assign new rail to channel */
	info->channels[channel].rail_i = new_rail;

	/* Rail is swapped, update en bits */
	odpm_io_update_bucken_enable_bits(info, false /* on_sleep */);

	/* Update rail muxsel */
	odpm_io_set_channel(info, channel);

	/* Send blank ASYNC, ignoring the last data set, if we've already init.
	 * If we haven't init, there's a high probability that this is boot-time
	 * rail selection, which will clear with CONFIG_COMPLETE.
	 */
	if (!info->chip.rx_ext_config_confirmation) {
		odpm_io_send_blank_async(info, &timestamp_ns);
		info->last_poll_ktime_boot_ns = timestamp_ns;
	} else {
		timestamp_ns = ktime_get_boottime_ns();
	}

	/* Record measurement start time / reset stop time */
	info->channels[channel].measurement_start_ms = to_ms(timestamp_ns);
	info->chip.rails[new_rail].measurement_stop_ms = 0;

enabled_rails_store_exit:
	mutex_unlock(&info->lock);

	return count;
}

static ssize_t measurement_start_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct odpm_info *info = iio_priv(indio_dev);
	int ch;
	ssize_t count = 0;

	mutex_lock(&info->lock);
	for (ch = 0; ch < ODPM_CHANNEL_MAX; ch++) {
		int rail_i = info->channels[ch].rail_i;

		if (info->channels[ch].enabled) {
			count += scnprintf(buf + count, PAGE_SIZE - count,
				"CH%d[%s], %ld\n", ch,
				info->chip.rails[rail_i].schematic_name,
				info->channels[ch].measurement_start_ms);
		}
	}
	mutex_unlock(&info->lock);

	return count;
}

static ssize_t measurement_stop_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct odpm_info *info = iio_priv(indio_dev);
	int rail_i;
	ssize_t count = 0;

	mutex_lock(&info->lock);
	for (rail_i = 0; rail_i < info->chip.num_rails; rail_i++) {
		struct odpm_rail_data *rail = &info->chip.rails[rail_i];

		if (rail->measurement_stop_ms != 0) {
			count += scnprintf(buf + count, PAGE_SIZE - count,
					   "%s(%s), %ld, %ld, %ld\n",
					   rail->name, rail->schematic_name,
					   rail->measurement_start_ms_cached,
					   rail->measurement_stop_ms,
					   rail->acc_power_uW_sec_cached);
		}
	}
	mutex_unlock(&info->lock);

	return count;
}

static IIO_DEVICE_ATTR_RW(ext_sampling_rate, 0);
static IIO_DEVICE_ATTR_RW(sampling_rate, 0);
static IIO_DEVICE_ATTR_RO(energy_value, 0);
static IIO_DEVICE_ATTR_RO(available_rails, 0);
static IIO_DEVICE_ATTR_RW(enabled_rails, 0);
static IIO_DEVICE_ATTR_RO(measurement_start, 0);
static IIO_DEVICE_ATTR_RO(measurement_stop, 0);

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
	ODPM_DEV_ATTR(sampling_rate),	 ODPM_DEV_ATTR(ext_sampling_rate),
	ODPM_DEV_ATTR(energy_value),	 ODPM_DEV_ATTR(available_rails),
	ODPM_DEV_ATTR(enabled_rails),	 ODPM_DEV_ATTR(measurement_start),
	ODPM_DEV_ATTR(measurement_stop), NULL
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

static void odpm_probe_init_device_specific(struct odpm_info *info, int id)
{
	/* s2mpg1x specific data */
	switch (id) {
	case ODPM_CHIP_S2MPG10:
	case ODPM_CHIP_S2MPG11:
		info->chip.id = id;

		info->chip.sampling_rate_int_uhz =
			s2mpg1x_meter_get_int_samping_rate_table();
		info->chip.sampling_rate_int_count = S2MPG1X_INT_FREQ_COUNT;
		info->chip.sampling_rate_ext_uhz =
			s2mpg1x_meter_get_ext_samping_rate_table();
		info->chip.sampling_rate_ext_count = S2MPG1X_EXT_FREQ_COUNT;
		break;
	}

	switch (id) {
	case ODPM_CHIP_S2MPG10: {
		struct s2mpg10_meter *meter = info->meter;
		struct s2mpg10_dev *pmic = dev_get_drvdata(meter->dev->parent);

		info->chip.hw_id = ID_S2MPG10;
		info->chip.hw_rev = pmic->pmic_rev;
		info->i2c = meter->i2c;
	} break;
	case ODPM_CHIP_S2MPG11: {
		struct s2mpg11_meter *meter = info->meter;
		struct s2mpg11_dev *pmic = dev_get_drvdata(meter->dev->parent);

		info->chip.hw_id = ID_S2MPG11;
		info->chip.hw_rev = pmic->pmic_rev;
		info->i2c = meter->i2c;
	} break;
	}

	info->chip.rx_ext_config_confirmation = false;
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

	if (pdev->id_entry->driver_data >= ODPM_CHIP_COUNT) {
		pr_err("odpm: Could not identify driver!\n");
		odpm_remove(pdev);
		return -ENODEV;
	}

	/* Init device-specific data */
	odpm_probe_init_device_specific(odpm_info, pdev->id_entry->driver_data);

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

	/* Start measurement of default rails */
	if (odpm_configure_start_measurement(odpm_info))
		pr_err("odpm: Failed to start measurement at probe\n");

	/* Configure work to kick off every XHz */
	odpm_periodic_refresh_setup(odpm_info);

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
	device_enable_async_suspend(&pdev->dev);

	mutex_init(&odpm_info->lock);

	pr_info("odpm: %s: init completed\n", pdev->name);

	return ret;
}

static int odpm_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct iio_dev *indio_dev = dev_get_drvdata(&pdev->dev);
	struct odpm_info *info = iio_priv(indio_dev);

	odpm_suspend_resume_operations(info, true /* suspend */);

	return 0;
}

static int odpm_resume(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(&pdev->dev);
	struct odpm_info *info = iio_priv(indio_dev);

	odpm_suspend_resume_operations(info, false /* suspend */);

	return 0;
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
	.suspend = odpm_suspend,
	.resume = odpm_resume,
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
