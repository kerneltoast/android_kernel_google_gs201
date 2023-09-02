/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Support for Thermal metrics
 *
 * Copyright 2022 Google LLC
 */
#include <linux/thermal.h>

#define MAX_SUPPORTED_THRESHOLDS             8
typedef int tr_handle;

#ifdef CONFIG_PIXEL_METRICS
int temp_residency_stats_update(tr_handle instance, int temp);
tr_handle register_temp_residency_stats(const char *name);
int unregister_temp_residency_stats(tr_handle instance);
int temp_residency_stats_set_thresholds(tr_handle instance,
                const int *thresholds, int num_thresholds);
#else
static inline
int temp_residency_stats_update(tr_handle instance, int temp) { return 0; }
static inline
tr_handle register_temp_residency_stats(const char *name) { return 0; }
static inline
int unregister_temp_residency_stats(tr_handle instance) { return 0; }
static inline
int temp_residency_stats_set_thresholds(tr_handle instance,
                const int *thresholds, int num_thresholds) { return 0; }
#endif
