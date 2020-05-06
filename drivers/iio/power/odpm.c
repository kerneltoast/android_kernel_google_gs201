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

struct odpm_rail_data {
	const char *name;
	u32 mux_select;

	u64 acc_power_uW_sec;
};

struct odpm_chip {
	const char *name;
	enum odpm_chip_id id;

	int num_rails;
	struct odpm_rail_data *rails;

	int max_refresh_time_ms;
	u64 acc_count;
	u64 acc_timestamp;
	s2mpg1x_int_samp_rate sampling_rate_i;
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
static int sample_rate_enum_to_int_hz[] = {
	[INT_7P_8125HZ] = 8, [INT_15P_625HZ] = 16, [INT_31P_25HZ] = 31,
	[INT_62P_5HZ] = 63,  [INT_125HZ] = 125,	 [INT_250HZ] = 250,
	[INT_500HZ] = 500,  [INT_1000HZ] = 1000,
};

static const char *const sample_rate_enum_to_str_hz[] = {
	[INT_7P_8125HZ] = "7.8125", [INT_15P_625HZ] = "15.625",
	[INT_31P_25HZ] = "31.25",   [INT_62P_5HZ] = "62.5",
	[INT_125HZ] = "125",	   [INT_250HZ] = "250",
	[INT_500HZ] = "500",	   [INT_1000HZ] = "1000",
};

static int sample_rate_enum_to_int_ms[] = {
	[INT_7P_8125HZ] = 128, [INT_15P_625HZ] = 64, [INT_31P_25HZ] = 32,
	[INT_62P_5HZ] = 16,    [INT_125HZ] = 8,	   [INT_250HZ] = 4,
	[INT_500HZ] = 2,      [INT_1000HZ] = 1,
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

static int odpm_io_set_channel(struct odpm_info *info, size_t channel)
{
	int ret = -1;
	int rail_i = info->channels[channel].rail_i;

	SWITCH_METER_FUNC(info, meter_set_muxsel, channel,
			  info->chip.rails[rail_i].mux_select);
	return ret;
}

static int odpm_io_set_sample_rate(struct odpm_info *info,
				   s2mpg1x_int_samp_rate hz)
{
	int ret = -1;

	SWITCH_METER_FUNC(info, set_int_samp_rate, hz);
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

int odpm_configure_chip(struct odpm_info *info)
{
	int ch;

	/* TODO(stayfan): b/156107234
	 * error conditions
	 */
	odpm_io_set_sample_rate(info, info->chip.sampling_rate_i);
	for (ch = 0; ch < ODPM_CHANNEL_MAX; ch++) {
		if (info->channels[ch].enabled)
			odpm_io_set_channel(info, ch);
	}
	odpm_io_set_meter_on(info, true);
	odpm_io_set_ext_meter_on(info, false);

	return 0;
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

bool odpm_match_sample_rate(struct odpm_info *info, int sample_rate)
{
	bool success = false;
	int i;

	for (i = 0; i < ARRAY_SIZE(sample_rate_enum_to_int_hz); i++) {
		if (sample_rate == sample_rate_enum_to_int_hz[i]) {
			info->chip.sampling_rate_i = i;
			success = true;
			break;
		}
	}

	return success;
}

static int odpm_parse_dt_rail(struct odpm_rail_data *rail_data,
			      struct device_node *node)
{
	u32 mux_sel = 0;

	if (of_property_read_u32(node, "channel-mux-selection", &mux_sel)) {
		pr_err("odpm: cannot read channel-mux-selection\n");
		return -EINVAL;
	}
	rail_data->mux_select = mux_sel;
	rail_data->name = node->name;

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
	int sample_rate, max_refresh_time_ms;
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
	if (of_property_read_u32(odpm_np, "sample-rate", &sample_rate)) {
		pr_err("odpm: cannot read sample rate value\n");
		return -EINVAL;
	}
	if (!odpm_match_sample_rate(info, sample_rate)) {
		pr_err("odpm: cannot parse sample rate value %d\n",
		       sample_rate);
		return -EINVAL;
	}
	if (of_property_read_u32(odpm_np, "max-refresh-time-ms",
				 &max_refresh_time_ms)) {
		pr_err("odpm: cannot read max refresh time value\n");
		return -EINVAL;
	}
	info->chip.max_refresh_time_ms = max_refresh_time_ms;

	ret = odpm_parse_dt_rails(dev, info, pmic_np);
	if (ret != 0)
		return ret;

	return odpm_parse_dt_channels(info, channels_np);
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
		int rail_i = info->channels[ch].rail_i;
		int sampling_period_ms =
			sample_rate_enum_to_int_ms[info->chip.sampling_rate_i];
		u32 resolution_mW_iq30 = INVALID_RESOLUTION;
		u32 ret = 0;
		__uint128_t power_acc_mW_iq30;
		__uint128_t power_acc_uW_sec_iq30;

		u32 muxsel = info->chip.rails[rail_i].mux_select;

		SWITCH_CHIP_FUNC(info,
				 muxsel_to_power_resolution, muxsel);
		resolution_mW_iq30 = ret;

		/* Use 128 bit to prevent overflow on multiplied iq30 values */
		power_acc_mW_iq30 =
			(__uint128_t)acc_data[ch] * resolution_mW_iq30;
		power_acc_uW_sec_iq30 = power_acc_mW_iq30 * sampling_period_ms;

		/* Scale back to value */
		info->chip.rails[rail_i].acc_power_uW_sec +=
			_IQ30_to_int(power_acc_uW_sec_iq30);
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

	return scnprintf(buf, PAGE_SIZE, "%s\n",
		sample_rate_enum_to_str_hz[info->chip.sampling_rate_i]);
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
	 * <RAIL NAME>, <energy value in uW-secs>
	 */
	count += scnprintf(buf + count, PAGE_SIZE - count, "%ld\n",
			   info->chip.acc_timestamp);

	for (ch = 0; ch < ODPM_CHANNEL_MAX; ch++) {
		int rail_i = info->channels[ch].rail_i;

		count += scnprintf(buf + count, PAGE_SIZE - count, "%s, %ld\n",
				   info->chip.rails[rail_i].name,
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
		count += scnprintf(buf + count, PAGE_SIZE - count, "%s\n",
				   info->chip.rails[rail_i].name);
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
		size_t rail_i = info->channels[ch].rail_i;

		if (info->channels[ch].enabled)
			count += scnprintf(buf + count, PAGE_SIZE - count,
					   "%s\n",
					   info->chip.rails[rail_i].name);
	}

	return count;
}

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
	ODPM_DEV_ATTR(sampling_rate), ODPM_DEV_ATTR(energy_value),
	ODPM_DEV_ATTR(available_rails), ODPM_DEV_ATTR(enabled_rails), NULL
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

	odpm_info->chip.name = NULL;
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

	/* 1. read device tree data */
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

	/* 2. initialize other data in odpm_info */

	/* 3. configure ODPM channels based on device tree input */
	ret = odpm_configure_chip(odpm_info);
	if (ret < 0) {
		odpm_remove(pdev);
		return ret;
	}

	/* 4. configure work to kick off every XHz */
	odpm_periodic_refresh_setup(odpm_info);

	/* 5. Setup IIO */
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
