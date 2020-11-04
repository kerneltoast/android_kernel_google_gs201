// SPDX-License-Identifier: GPL-2.0-only
/* init.c
 *
 * Android Vendor Hook Support
 *
 * Copyright 2020 Google LLC
 */

#include <linux/module.h>
#include <trace/hooks/sched.h>
#include <trace/hooks/topology.h>

extern void rvh_find_energy_efficient_cpu_pixel_mod(void *data, struct task_struct *p, int prev_cpu,
						    int sync, int *new_cpu);
extern void vh_arch_set_freq_scale_pixel_mod(void *data, struct cpumask *cpus, unsigned long freq,
					     unsigned long max, unsigned long *scale);
extern void vh_set_sugov_sched_attr_pixel_mod(void *data, struct sched_attr *attr);
extern void rvh_set_iowait_pixel_mod(void *data, struct task_struct *p, int *should_iowait_boost);
extern int create_sysfs_node(void);

static int vh_sched_init(void)
{
	int ret;

	ret = register_trace_android_rvh_find_energy_efficient_cpu(
						rvh_find_energy_efficient_cpu_pixel_mod, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_vh_arch_set_freq_scale(vh_arch_set_freq_scale_pixel_mod, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_vh_set_sugov_sched_attr(
		vh_set_sugov_sched_attr_pixel_mod, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_rvh_set_iowait(rvh_set_iowait_pixel_mod, NULL);
	if (ret)
		return ret;

	ret = create_sysfs_node();
	if (ret)
		return ret;

	return 0;
}

module_init(vh_sched_init);
MODULE_LICENSE("GPL v2");
