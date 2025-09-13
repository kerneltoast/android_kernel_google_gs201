/* SPDX-License-Identifier: GPL-2.0 */
/*
 * BTS header for Exynos.
 *
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 */

#ifndef __EXYNOS_BTS_H_
#define __EXYNOS_BTS_H_

#include <linux/errno.h>
#include <linux/types.h>

/**
 * @BTS_HIST_BIN: Number of bins of the histogram.
 * @bw_trip:      The trip points for each histogram bin.
 *
 * The trip points are based on log-scale and in the unit
 * of KB/s.
 */
#define BTS_HIST_BIN 13
static const unsigned int bw_trip[BTS_HIST_BIN - 1] = {
	10000,
	17800,
	31600,
	56200,
	100000,
	178000,
	316000,
	562000,
	1000000,
	1780000,
	3160000,
	5620000,
};

/**
 * struct bw_stats - Bandwidth stats for histogram
 *
 */
struct bw_stats {
	int hist_idx;
	u32 count[BTS_HIST_BIN];
	u64 total_time[BTS_HIST_BIN];
};

/**
 * struct bts_bw_stats - BTS bandwidth voting stats
 *
 */
struct bts_bw_stats {
	u64 start_time;
	struct bw_stats total;
	struct bw_stats peak;
};

/**
 * struct bts_bw - BTS bandwidth information
 * @name:	name of IP
 * @peak:	IP Peak bandwidth over single port
 * @read:	Average read bandwidth
 * @write:	Average write bandwidth
 * @rt: 	IP total RT bandwidth - should be 0 for non-RT clients
 *
 */
struct bts_bw {
#if IS_ENABLED(CONFIG_SOC_ZUMA)
	struct list_head node;
	unsigned int bus_width;
#endif
	char *name;
	bool is_rt;
	unsigned int peak;
	unsigned int read;
	unsigned int write;
	unsigned int rt;
	struct bts_bw_stats stats;
};

/**
 * struct bus1_int_map - Freq mapping information
 * @bus1_freq:	Frequency of BUS1 in KHz
 * @int_freq:	Frequency of INT in KHz
 *
 */
struct bus1_int_map {
	unsigned int bus1_freq;
	unsigned int int_freq;
};

#if IS_ENABLED(CONFIG_EXYNOS_BTS) || IS_ENABLED(CONFIG_EXYNOS_BTS_MODULE)
int bts_get_bwindex(const char *name);
int bts_update_bw(unsigned int index, struct bts_bw bw);
unsigned int bts_get_scenindex(const char *name);
int bts_add_scenario(unsigned int index);
int bts_del_scenario(unsigned int index);

#if IS_ENABLED(CONFIG_SOC_ZUMA)
int bts_get_urgent_lat_bts_index(const char *name);
void bts_set_urgent_lat_read(int index, unsigned int lat_read);
#else
static inline int bts_get_urgent_lat_bts_index(const char *name)
{
	return -ENOSYS;
}
static inline void bts_set_urgent_lat_read(int index, unsigned int lat_read)
{
}
#endif

void bts_pd_sync(unsigned int cal_id, int on);

#else /* CONFIG_EXYNOS_BTS */

static inline int bts_get_bwindex(const char *name) { return -ENOSYS; }
static inline int bts_update_bw(unsigned int index, struct bts_bw bw) { return -ENOSYS; }
static inline unsigned int bts_get_scenindex(const char *name) { return 0; }
static inline int bts_add_scenario(unsigned int index) { return -ENOSYS; }
static inline int bts_del_scenario(unsigned int index) { return -ENOSYS; }
static inline int bts_get_urgent_lat_bts_index(const char *name)
{
	return -ENOSYS;
}
static inline void bts_set_urgent_lat_read(int index, unsigned int lat_read)
{
}
static inline void bts_pd_sync(unsigned int cal_id, int on) { }

#endif /* CONFIG_EXYNOS_BTS */

#define bts_update_scen(a, b) do {} while (0)
#define exynos_bts_scitoken_setting(a) do {} while (0)

#endif /* __EXYNOS_BTS_H_ */
