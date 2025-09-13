/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * GS101 SoC Exynos ACME(A Cpufreq that Meets Every chipset) driver support
 */

#include <soc/google/exynos-dm.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>

struct exynos_cpufreq_dm {
	struct list_head		list;
	struct exynos_dm_constraint	c;
	int				driver_cal_id;
	int				constraint_cal_id;
};

typedef int (*target_fn)(struct cpufreq_policy *policy,
				unsigned int target_freq,
				unsigned int relation);

struct exynos_cpufreq_ready_block {
	struct list_head		list;

	/* callback function to update policy-dependent data */
	int (*update)(struct cpufreq_policy *policy);
	int (*get_target)(struct cpufreq_policy *policy, target_fn target);
};

struct exynos_cpufreq_file_operations {
	struct file_operations		fops;
	struct miscdevice		miscdev;
	struct freq_constraints		*freq_constraints;
	enum				freq_qos_req_type req_type;
	unsigned int			default_value;
};

enum thermal_actors {
	TJ = 0,
	TSKIN = 1
};
#define NR_THERMAL_ACTORS 2
#define THERMAL_PRESSURE_STR_LEN (25)
struct exynos_cpufreq_domain {
	/* list of domain */
	struct list_head		list;

	/* lock */
	struct mutex			lock;

	/* dt node */
	struct device_node		*dn;

	/* domain identity */
	unsigned int			id;
	struct cpumask			cpus;
	unsigned int			cal_id;
	int				dm_type;

	/* frequency scaling */
	bool				enabled;

	unsigned int			table_size;
	struct cpufreq_frequency_table	*freq_table;

	unsigned int			max_freq;
	unsigned int			min_freq;
	unsigned int			max_freq_qos;
	unsigned int			min_freq_qos;
	unsigned int			boot_freq;
	unsigned int			resume_freq;
	unsigned int			old;
	unsigned int			soft_max_freq;

	/* freq qos */
	struct freq_qos_request		min_qos_req;
	struct freq_qos_request		max_qos_req;
	struct freq_qos_request		user_min_qos_req;
	struct freq_qos_request		user_max_qos_req;
	struct delayed_work		work;

	/* fops node */
	struct exynos_cpufreq_file_operations	min_qos_fops;
	struct exynos_cpufreq_file_operations	max_qos_fops;

	/* list head of DVFS Manager constraints */
	struct list_head		dm_list;

	bool				need_awake;

	struct thermal_cooling_device *cdev;

	/* Thermal pressure */
	spinlock_t thermal_update_lock;
	unsigned long capped_freq[NR_THERMAL_ACTORS];
	char capped_freq_name[NR_THERMAL_ACTORS][THERMAL_PRESSURE_STR_LEN];
	unsigned int ect_table_offset;
};

/*
 * the time it takes on this CPU to switch between
 * two frequencies in nanoseconds
 */
#define TRANSITION_LATENCY	500000
