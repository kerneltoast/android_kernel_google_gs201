/* SPDX-License-Identifier: <GPL-2.0-only> */
/*
 * acpm_power_stats_h
 *
 * Exported header from ACPM fw describing ACPM stats
 *
 * Copyright 2020 Google LLC
 *
 * Author: bsschwar@google.com
 */

#ifndef __ACPM_POWER_STATS_H__
#define __ACPM_POWER_STATS_H__

#ifndef CONFIG_EXYNOS_ACPM
#include "common.h"
#endif // CONFIG_EXYNOS_ACPM

enum {
#if IS_ENABLED(CONFIG_SOC_GS101) || IS_ENABLED(CONFIG_SOC_GS201)
	MAX_NUM_FREQS = 16,
#else
	/* The histogram will be truncated automatically we ever exceed this number.
	 * Must update header on both acpm and kernel sides if this ever changes. */
	MAX_NUM_FREQS = 20,
#endif
};

// Must be the same as sys_powermode in flexpmu_cal_system_<arch>.h
enum sys_powermode {
	SYS_SICD,
	SYS_SLEEP,
	SYS_SLEEP_SLCMON,
	SYS_SLEEP_HSI1ON,
	SYS_STOP,
	NUM_SYS_POWERMODE,
};

enum mif_users {
	MIF_USER_AOC,
	MIF_USER_GSA,
#if !IS_ENABLED(CONFIG_SOC_GS101)
	MIF_USER_TPU,
#endif
#if !IS_ENABLED(CONFIG_SOC_GS101) && !IS_ENABLED(CONFIG_SOC_GS201)
	MIF_USER_AUR,
#endif
	NUM_MIF_USERS, // NR_MIF_USERS (excluding AP) in plugins/flexpmu/<arch>/flexpmu_sfr_map.h
};

enum slc_users {
	SLC_USER_AOC,
	NUM_SLC_USERS, // NR_SLC_USERS (excluding AP) in plugins/flexpmu/<arch>/flexpmu_sfr_map.h
};

// Defined in flexpmu_cal_cpu_zuma.h
enum cpu_corenum {
	CORE00,
	CORE01,
	CORE02,
	CORE03,
	CORE10,
	CORE11,
#if IS_ENABLED(CONFIG_SOC_GS101) || IS_ENABLED(CONFIG_SOC_GS201)
	CORE20,
	CORE21,
#else
	CORE12,
	CORE13,
	CORE20,
#endif
	NUM_CORES,
};

// Defined in flexpmu_cal_cpu_zuma.h
enum cpu_clusternum {
	CLUSTER0,
	CLUSTER1,
	CLUSTER2,
	NUM_CLUSTERS,
};

enum fvp_domains {
	DOMAIN_MIF,
	DOMAIN_TPU,
	DOMAIN_CPUCL0,
	DOMAIN_CPUCL1,
	DOMAIN_CPUCL2,
#if !IS_ENABLED(CONFIG_SOC_GS101)
	DOMAIN_AUR,
#endif
	NUM_DOMAINS,
#if !IS_ENABLED(CONFIG_SOC_GS101) && !IS_ENABLED(CONFIG_SOC_GS201)
	INVALID_DOMAIN,
#endif
};

struct lpm_stat {
	u64 success_count;
	u64 early_wakeup_count;
	u64 error_count;
	u64 total_time;
	u64 last_entry_timestamp;
	u64 last_exit_timestamp;
};

struct resource_stat {
	u64 down_count;
	u64 total_down_time;
	u64 last_down_timestamp;
	u64 last_up_timestamp;
};

struct resource_req_stat {
	u64 req_up_count;
	u64 total_req_up_time;
	u64 last_req_up_timestamp;
	u64 last_req_down_timestamp;
};

struct min_max_stat {
	u64 total;
	u64 count;
	u64 min;
	u64 max;
};

struct soc_stats {
	struct lpm_stat lpm_stats[NUM_SYS_POWERMODE];
	struct resource_stat mif_stats[NUM_SYS_POWERMODE];
	struct resource_req_stat mif_req_stats[NUM_MIF_USERS];
	struct resource_stat slc_stats[NUM_SYS_POWERMODE];
	struct resource_req_stat slc_req_stats[NUM_SLC_USERS];
};

struct core_stats {
	struct resource_stat cluster_stats[NUM_CLUSTERS];
	struct resource_stat cpu_stats[NUM_CORES]; // "down" == C2
};

struct latency_stats {
	struct min_max_stat mif_down;
	struct min_max_stat mif_up;
	struct min_max_stat slc_down;
	struct min_max_stat slc_up;
};

struct freq_histogram {
	u64 freq_time[MAX_NUM_FREQS];
	u64 freq_count[MAX_NUM_FREQS];
	u64 freq_change_timestamp;
	u32 freqs[MAX_NUM_FREQS];
	u32 cur_freq;
};

struct fvp_stats {
	struct freq_histogram freq_hist[NUM_DOMAINS];
};

struct pmu_stats {
	struct resource_req_stat mif_pwr_gsa_req;
};

#ifndef CONFIG_EXYNOS_ACPM
/**
 * acpm stat accessory functions
 */
static inline void resource_down(struct resource_stat *stat, u64 cur_time)
{
	stat->down_count++;
	stat->last_down_timestamp = cur_time;
}

static inline void resource_up(struct resource_stat *stat, u64 cur_time)
{
	stat->last_up_timestamp = cur_time;
	stat->total_down_time +=
		stat->last_up_timestamp - stat->last_down_timestamp;
}

static inline void resource_req_up(struct resource_req_stat *stat, u64 cur_time)
{
	if (stat->last_req_up_timestamp < stat->last_req_down_timestamp ||
	    stat->req_up_count == 0) {
		stat->req_up_count++;
		stat->last_req_up_timestamp = cur_time;
	}
}

static inline void resource_req_down(struct resource_req_stat *stat,
				     u64 cur_time)
{
	if (stat->last_req_down_timestamp < stat->last_req_up_timestamp) {
		stat->last_req_down_timestamp = cur_time;
		stat->total_req_up_time += stat->last_req_down_timestamp -
					   stat->last_req_up_timestamp;
	}
}

static inline void update_min_max_stat(struct min_max_stat *stat, u64 val)
{
	stat->total += val;
	stat->count++;
	stat->min = stat->min < val ? stat->min : val;
	stat->max = stat->max > val ? stat->max : val;
}

#endif // CONFIG_EXYNOS_ACPM

#endif // __ACPM_POWER_STATS_H__
