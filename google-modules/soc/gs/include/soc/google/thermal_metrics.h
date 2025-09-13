/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Support for Thermal metrics
 *
 * Copyright 2022 Google LLC
 */
#include <linux/thermal.h>
#include <linux/types.h>

#define MAX_SUPPORTED_THRESHOLDS             8
typedef int tr_handle;

enum abnormality_type {
	UNKNOWN = 0,
	SENSOR_STUCK,
	EXTREME_HIGH_TEMP,
	EXTREME_LOW_TEMP,
	ABNORMALITY_TYPE_END,
};

static const char * const abnormality_type_str[] = {
	[UNKNOWN] = "UNKNOWN",
	[SENSOR_STUCK] = "SENSOR_STUCK",
	[EXTREME_HIGH_TEMP] = "EXTREME_HIGH_TEMP",
	[EXTREME_LOW_TEMP] = "EXTREME_LOW_TEMP",
	[ABNORMALITY_TYPE_END] = "ABNORMALITY_TYPE_END"
};

struct temp_validity_config {
	int min_temp;
	int max_temp;
	u8 min_repeat_count;
	int min_repeat_msecs;
};

struct tr_sample {
	int temp;
	// real timestamp in seconds
	time64_t timestamp;
};

struct temp_residency_stats_callbacks {
	int (*set_thresholds)(tr_handle instance, const int *thresholds, int num_thresholds);
	int (*get_thresholds)(tr_handle instance, int *thresholds, int *num_thresholds);
	int (*get_stats)(tr_handle instance, atomic64_t *stats,
					       struct tr_sample *max, struct tr_sample *min);
	int (*reset_stats)(tr_handle instance);
};

int temp_residency_stats_update(tr_handle instance, int temp);
tr_handle register_temp_residency_stats(const char *name, char *group_name);
int register_temp_residency_stats_callbacks(tr_handle instance,
		struct temp_residency_stats_callbacks *ops);
int unregister_temp_residency_stats(tr_handle instance);
int temp_residency_stats_set_thresholds(tr_handle instance,
		const int *thresholds, int num_thresholds);
