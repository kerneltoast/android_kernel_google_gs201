#ifndef _GOVERNOR_DSULAT_H_
#define _GOVERNOR_DSULAT_H_

#include <linux/kernel.h>
#include <linux/devfreq.h>
#include <soc/google/exynos-devfreq.h>

struct dsulat_node {
	unsigned int ratio_ceil_cl0;
	unsigned int ratio_ceil_cl1;
	unsigned int ratio_ceil_cl2;
	unsigned int wb_pct_thres_cl0;
	unsigned int wb_pct_thres_cl1;
	unsigned int wb_pct_thres_cl2;
	unsigned int wb_filter_ratio_cl0;
	unsigned int wb_filter_ratio_cl1;
	unsigned int wb_filter_ratio_cl2;
	unsigned int mem_stall_floor_cl0;
	unsigned int mem_stall_floor_cl1;
	unsigned int mem_stall_floor_cl2;
	unsigned int dsulat_cpuidle_state_aware_cl0;
	unsigned int dsulat_cpuidle_state_aware_cl1;
	unsigned int dsulat_cpuidle_state_aware_cl2;
	bool mon_started;
	bool already_zero;
	struct list_head list;
	void *orig_data;
	struct core_dev_map *freq_map_cl0_low_latency;
	struct core_dev_map *freq_map_cl1_low_latency;
	struct core_dev_map *freq_map_cl2_low_latency;
	struct core_dev_map *freq_map_cl0_base;
	struct core_dev_map *freq_map_cl1_base;
	struct core_dev_map *freq_map_cl2_base;
	struct core_dev_map *freq_map_dsu_bci;
	struct devfreq_governor *gov;
	struct attribute_group *attr_grp;
	unsigned long resume_freq;
	struct exynos_pm_qos_request		dsu_bci_qos_req;
};

#if IS_ENABLED(CONFIG_DEVFREQ_GOV_MEMLAT)
int register_dsulat(struct exynos_devfreq_data *dsu_data);
void set_dsu_data(struct exynos_devfreq_data *dsu_data);
#else
static inline int register_dsulat(struct exynos_devfreq_data *dsu_data)
{
	return 0;
}
static void set_dsu_data(struct exynos_devfreq_data *dsu_data) {}
#endif
#endif  // _GOVERNOR_DSULAT_H_
