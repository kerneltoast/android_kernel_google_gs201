/* SPDX-License-Identifier: GPL-2.0 */
/*
 * ODPM Support.
 *
 * Copyright 2022 Google LLC
 */

#ifndef __ODPM_H
#define __ODPM_H

#include <linux/mfd/samsung/s2mpg1415-meter.h>
#include <linux/mfd/samsung/s2mpg1415.h>

#define ODPM_CHANNEL_MAX S2MPG1415_METER_CHANNEL_MAX
#define ODPM_BUCK_EN_BYTES S2MPG1415_METER_BUCKEN_BUF

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
	enum s2mpg1415_id hw_id;
	int hw_rev;
	u32 max_refresh_time_ms;

	int num_rails;
	struct odpm_rail_data *rails;

	const u32 *sampling_rate_int_uhz;
	int sampling_rate_int_count;
	const u32 *sampling_rate_ext_uhz;
	int sampling_rate_ext_count;

	enum s2mpg1415_int_samp_rate int_config_sampling_rate_i;
	enum s2mpg1415_ext_samp_rate ext_config_sampling_rate_i;

	/* Data */
	u64 acc_timestamp_ms;
	enum s2mpg1415_int_samp_rate int_sampling_rate_i;
	enum s2mpg1415_ext_samp_rate ext_sampling_rate_i;

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
	struct mutex *meter_lock; /* Meter lock */
	struct mutex lock; /* Global HW lock */

	struct odpm_channel_data channels[ODPM_CHANNEL_MAX];

	struct workqueue_struct *work_queue;
	struct work_struct work_refresh;
	struct alarm alarmtimer_refresh;
	struct wakeup_source *ws;

	struct platform_device *odpm_vbatt_en_pdev;

	u64 last_poll_ktime_boot_ns;
	bool sleeping;
	bool ready;
};

void odpm_get_raw_lpf_values(struct odpm_info *info, enum s2mpg1415_meter_mode mode,
                             u32 micro_unit[ODPM_CHANNEL_MAX]);

#endif /* __ODPM_H */
