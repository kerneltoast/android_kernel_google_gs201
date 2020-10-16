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

	return 0;
}

module_init(vh_sched_init);
MODULE_LICENSE("GPL v2");
