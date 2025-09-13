/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 */

#ifndef __EXYNOS_DEVFREQ_H_
#define __EXYNOS_DEVFREQ_H_

#include <linux/devfreq.h>
#include <soc/google/exynos_pm_qos.h>
#include <linux/clk.h>
#include <soc/google/exynos-devfreq-dep.h>
#if IS_ENABLED(CONFIG_EXYNOS_DVFS_MANAGER)
#include <soc/google/exynos-dm.h>
#endif

#define EXYNOS_DEVFREQ_MODULE_NAME	"gs101-devfreq"
#define VOLT_STEP			25000
#define MAX_NR_CONSTRAINT		DM_TYPE_END
#define DATA_INIT			5
#define SET_CONST			1
#define RELEASE				2

/* DEVFREQ GOV TYPE */
#define SIMPLE_INTERACTIVE 0
#define MEM_LATENCY 1
#define DSU_LATENCY 2

int devfreq_simple_interactive_init(void);
#if IS_ENABLED(CONFIG_EXYNOS_ALT_DVFS)
#define LOAD_BUFFER_MAX			10
struct devfreq_alt_load {
	unsigned long long	delta;
	unsigned int		load;
};

struct devfreq_alt_track {
	struct devfreq_alt_load	buffer[LOAD_BUFFER_MAX];
	struct devfreq_alt_load	*front;
	struct devfreq_alt_load	*rear;

	unsigned long long	busy;
	unsigned long long	total;
	unsigned int		min_load;
	unsigned int		max_load;
	unsigned long long	max_spent;
};

struct mif_int_map {
	unsigned int mif_freq;
	unsigned int int_freq;
};

#define ALTDVFS_MIN_SAMPLE_TIME		15
#define ALTDVFS_HOLD_SAMPLE_TIME	100
#define ALTDVFS_TARGET_LOAD		75
#define ALTDVFS_NUM_TARGET_LOAD		1
#define ALTDVFS_HISPEED_LOAD		99
#define ALTDVFS_HISPEED_FREQ		1000000
#define ALTDVFS_TOLERANCE		1

struct devfreq_alt_dvfs_data {
	struct devfreq_alt_track	*track;
	unsigned int			track_group;

	/* ALT-DVFS parameter */
	unsigned int		*target_load;
	unsigned int		num_target_load;
	unsigned int		min_sample_time;
	unsigned int		hold_sample_time;
	unsigned int		hispeed_load;
	unsigned int		hispeed_freq;
	unsigned int		tolerance;

	/* MIF to INT mapping table */
	struct mif_int_map	*mif_int_tbl;
	unsigned int		map_row_cnt;
};
#endif /* ALT_DVFS */

#define DEFAULT_DELAY_TIME		10 /* msec */
#define DEFAULT_NDELAY_TIME		1
#define DELAY_TIME_RANGE		10
#define BOUND_CPU_NUM			0

struct devfreq_notifier_block {
	struct notifier_block nb;
	struct devfreq *df;
};

struct devfreq_simple_interactive_data {
	bool use_delay_time;
	int *delay_time;
	int ndelay_time;
	unsigned long prev_freq;
	u64 changed_time;
	struct timer_list freq_timer;
	struct timer_list freq_slack_timer;
	struct task_struct *change_freq_task;
	int pm_qos_class;
	int pm_qos_class_max;
	struct devfreq_notifier_block nb;
	struct devfreq_notifier_block nb_max;
	bool df_update_enable;
#if IS_ENABLED(CONFIG_EXYNOS_ALT_DVFS)
	struct devfreq_alt_dvfs_data alt_data;
	unsigned int governor_freq;
#endif
};

struct exynos_devfreq_opp_table {
	u32 idx;
	u32 freq;
	u32 volt;
};

struct ppc_data {
	u64 ccnt;
	u64 pmcnt0;
	u64 pmcnt1;
};

struct um_exynos {
	struct list_head node;
	void __iomem **va_base;
	u32 *pa_base;
	u32 *mask_v;
	u32 *mask_a;
	u32 *channel;
	u32 um_group;
	u32 *um_count;
	u32 um_count_total;
	struct ppc_data *ppc_val;
	bool ppc_read_reset_disable;
};

struct exynos_devfreq_data {
	struct device				*dev;
	struct devfreq				*devfreq;
	struct mutex				lock;			/* lock */
	struct clk				*clk;

	bool					devfreq_disabled;
	u32		cpu;

	u32		devfreq_type;

	struct exynos_devfreq_opp_table		*opp_list;

	u32					default_qos;

	u32					max_state;
	struct devfreq_dev_profile		devfreq_profile;

	u32		gov_type;
	const char				*governor_name;
	u32					cal_qos_max;
	void					*governor_data;
	struct devfreq_simple_interactive_data	simple_interactive_data;
	u32					dfs_id;
	s32					old_idx;
	s32					new_idx;
	u32					old_freq;
	u32					new_freq;
	u32					min_freq;
	u32					soft_max_freq;
	u32					max_freq;
	u32					reboot_freq;
	u32					boot_freq;
	unsigned long				suspend_freq;
	bool					suspend_flag;

	u32					old_volt;
	u32					new_volt;

	u32					pm_qos_class;
	u32					pm_qos_class_max;
	struct exynos_pm_qos_request		sys_pm_qos_min;
	struct exynos_pm_qos_request		debug_pm_qos_min;
	struct exynos_pm_qos_request		debug_pm_qos_max;
	struct exynos_pm_qos_request		pm_qos_soft_max_freq;
	struct exynos_pm_qos_request		default_pm_qos_min;
	struct exynos_pm_qos_request		default_pm_qos_max;
	struct exynos_pm_qos_request		thermal_pm_qos_max;
	struct exynos_pm_qos_request		boot_pm_qos;
#if IS_ENABLED(CONFIG_EXYNOS_ALT_DVFS)
	struct exynos_pm_qos_request		bus_pm_qos_min;
#endif
	u32					boot_qos_timeout;

	struct notifier_block			reboot_notifier;

	u32					ess_flag;

	s32					target_delay;

#if IS_ENABLED(CONFIG_EXYNOS_DVFS_MANAGER)
	u32		dm_type;
	u32		nr_constraint;
	struct exynos_dm_constraint		**constraint;
#endif
	void					*private_data;
	bool					use_acpm;
	bool					bts_update;
	bool					update_fvp;
	bool					use_get_dev;

	struct devfreq_notifier_block		*um_nb;
	struct um_exynos			um_data;
	u64					last_monitor_period;
	u64					last_monitor_time;
	u32					last_um_usage_rate;

	struct exynos_pm_domain *pm_domain;
	unsigned long previous_freq;
	unsigned long *time_in_state;
	unsigned long last_stat_updated;

	struct thermal_cooling_device *cooling_dev;
	unsigned long cooling_state;
	unsigned long sysfs_req;
	struct lock_class_key devfreq_lock_key;
};

struct exynos_profile_data {
	struct ppc_data *ppc_val;
	unsigned long long delta_time;
};

s32 exynos_devfreq_get_opp_idx(struct exynos_devfreq_opp_table *table,
			       unsigned int size, u32 freq);
#if IS_ENABLED(CONFIG_ARM_EXYNOS_DEVFREQ) && IS_ENABLED(CONFIG_EXYNOS_DVFS_MANAGER)
u32 exynos_devfreq_get_dm_type(u32 devfreq_type);
u32 exynos_devfreq_get_devfreq_type(int dm_type);
#endif

#if IS_ENABLED(CONFIG_ARM_EXYNOS_DEVFREQ)
unsigned long exynos_devfreq_get_domain_freq(unsigned int devfreq_type);
int exynos_devfreq_init_freq_table(struct exynos_devfreq_data *data);
int exynos_devfreq_lock_freq(unsigned int devfreq_type, unsigned int qos_value);
int exynos_devfreq_get_boundary(unsigned int devfreq_type,
				unsigned int *max_freq, unsigned int *min_freq);
#else
static inline unsigned long exynos_devfreq_get_domain_freq(unsigned int devfreq_type)
{
	return 0;
}
static inline int exynos_devfreq_init_freq_table(struct exynos_devfreq_data *data)
{
	return 0;
}

static inline int exynos_devfreq_lock_freq(unsigned int devfreq_type, unsigned int qos_value)
{
	return 0;
}

static inline int exynos_devfreq_get_boundary(unsigned int devfreq_type,
				       unsigned int *max_freq, unsigned int *min_freq)
{
	return 0;
}
#endif

#if IS_ENABLED(CONFIG_ECT)
int exynos_devfreq_parse_ect(struct exynos_devfreq_data *data,
				    const char *dvfs_domain_name);
#else
static inline int exynos_devfreq_parse_ect(struct exynos_devfreq_data *data,
				    const char *dvfs_domain_name)
{
	return 0;
}
#endif
#endif	/* __EXYNOS_DEVFREQ_H_ */
