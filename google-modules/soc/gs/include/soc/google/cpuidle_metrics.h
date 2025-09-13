/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Support for CPU Idle metrics
 *
 * Copyright 2023 Google LLC
 */

#if IS_ENABLED(CONFIG_CPUIDLE_METRICS)
inline void cpuidle_metrics_histogram_append(int cluster_id, s64 time_us);
int cpuidle_metrics_init(struct kobject *metrics_kobj);
void cpuidle_metrics_histogram_register(char* name, int cluster_id, s64 target_residency_us);
#else
static inline void cpuidle_metrics_histogram_append(int cluster_id, s64 time_us) {};
static void cpuidle_metrics_histogram_register(char* name, int cluster_id,
                                s64 target_residency_us) {};
#endif
