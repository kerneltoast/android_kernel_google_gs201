// SPDX-License-Identifier: GPL-2.0-only
/* init.c
 *
 * Android Vendor Hook Support
 *
 * Copyright 2020 Google LLC
 */

#include <kernel/sched/sched.h>
#include <linux/cpufreq.h>
#include <linux/module.h>
#include <trace/hooks/sched.h>
#include <trace/hooks/topology.h>

extern void init_uclamp_stats(void);
extern int create_sysfs_node(void);
extern void rvh_find_energy_efficient_cpu_pixel_mod(void *data, struct task_struct *p, int prev_cpu,
						    int sync, int *new_cpu);
extern void vh_arch_set_freq_scale_pixel_mod(void *data,
					     const struct cpumask *cpus,
					     unsigned long freq,
					     unsigned long max,
					     unsigned long *scale);
extern void vh_set_sugov_sched_attr_pixel_mod(void *data, struct sched_attr *attr);
extern void rvh_set_iowait_pixel_mod(void *data, struct task_struct *p, int *should_iowait_boost);
extern void rvh_select_task_rq_rt_pixel_mod(void *data, struct task_struct *p, int prev_cpu,
					    int sd_flag, int wake_flags, int *new_cpu);
extern void rvh_cpu_overutilized_pixel_mod(void *data, int cpu, int *overutilized);
extern void rvh_dequeue_task_pixel_mod(void *data, struct rq *rq, struct task_struct *p, int flags);
extern void rvh_uclamp_eff_get_pixel_mod(void *data, struct task_struct *p,
					 enum uclamp_id clamp_id, struct uclamp_se *uclamp_max,
					 struct uclamp_se *uclamp_eff, int *ret);
extern void rvh_util_est_update_pixel_mod(void *data, struct cfs_rq *cfs_rq, struct task_struct *p,
					   bool task_sleep, int *ret);
extern void rvh_post_init_entity_util_avg_pixel_mod(void *data, struct sched_entity *se);
extern void rvh_check_preempt_wakeup_pixel_mod(void *data, struct rq *rq, struct task_struct *p,
			bool *preempt, bool *nopreempt, int wake_flags, struct sched_entity *se,
			struct sched_entity *pse, int next_buddy_marked, unsigned int granularity);
extern void rvh_cpu_cgroup_online_pixel_mod(void *data, struct cgroup_subsys_state *css);
extern void vh_sched_setscheduler_uclamp_pixel_mod(void *data, struct task_struct *tsk,
						   int clamp_id, unsigned int value);
extern void init_uclamp_stats(void);
extern void rvh_sched_fork_pixel_mod(void *data, struct task_struct *tsk);
extern void vh_dup_task_struct_pixel_mod(void *data, struct task_struct *tsk,
					 struct task_struct *orig);

extern struct cpufreq_governor sched_pixel_gov;

static int vh_sched_init(void)
{
	int ret;

#if IS_ENABLED(CONFIG_UCLAMP_STATS)
	init_uclamp_stats();
#endif

	ret = create_sysfs_node();
	if (ret)
		return ret;

	ret = register_trace_android_rvh_find_energy_efficient_cpu(
						rvh_find_energy_efficient_cpu_pixel_mod, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_vh_arch_set_freq_scale(vh_arch_set_freq_scale_pixel_mod, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_rvh_set_iowait(rvh_set_iowait_pixel_mod, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_rvh_select_task_rq_rt(rvh_select_task_rq_rt_pixel_mod, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_rvh_cpu_overutilized(rvh_cpu_overutilized_pixel_mod, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_rvh_dequeue_task(rvh_dequeue_task_pixel_mod, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_rvh_uclamp_eff_get(rvh_uclamp_eff_get_pixel_mod, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_rvh_util_est_update(rvh_util_est_update_pixel_mod, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_rvh_post_init_entity_util_avg(
		rvh_post_init_entity_util_avg_pixel_mod, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_rvh_check_preempt_wakeup(
		rvh_check_preempt_wakeup_pixel_mod, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_rvh_cpu_cgroup_online(
		rvh_cpu_cgroup_online_pixel_mod, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_vh_setscheduler_uclamp(
		vh_sched_setscheduler_uclamp_pixel_mod, NULL);
	if (ret)
		return ret;

	ret = cpufreq_register_governor(&sched_pixel_gov);
	if (ret)
		return ret;

	ret = register_trace_android_rvh_sched_fork(rvh_sched_fork_pixel_mod, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_vh_dup_task_struct(vh_dup_task_struct_pixel_mod, NULL);
	if (ret)
		return ret;

	// Disable TTWU_QUEUE.
	sysctl_sched_features &= ~(1UL << __SCHED_FEAT_TTWU_QUEUE);
	static_key_disable(&sched_feat_keys[__SCHED_FEAT_TTWU_QUEUE]);

	return 0;
}

module_init(vh_sched_init);
MODULE_LICENSE("GPL v2");
