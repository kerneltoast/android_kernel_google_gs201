// SPDX-License-Identifier: GPL-2.0-only
/* rt.c
 *
 * Android Vendor Hook Support
 *
 * Copyright 2020 Google LLC
 */
#include <kernel/sched/sched.h>

#include "sched.h"

extern unsigned int vendor_sched_util_threshold;

extern unsigned long cpu_util(int cpu);
extern unsigned long task_util(struct task_struct *p);
extern bool task_may_not_preempt(struct task_struct *task, int cpu);
extern int cpu_is_idle(int cpu);
extern int sched_cpu_idle(int cpu);

/*****************************************************************************/
/*                       Upstream Code Section                               */
/*****************************************************************************/
/*
 * This part of code is copied from Android common GKI kernel and unmodified.
 * Any change for these functions in upstream GKI would require extensive review
 * to make proper adjustment in vendor hook.
 */

static inline bool rt_task_fits_capacity(struct task_struct *p, int cpu)
{
	unsigned int min_cap;
	unsigned int max_cap;
	unsigned int cpu_cap;

	min_cap = uclamp_eff_value(p, UCLAMP_MIN);
	max_cap = uclamp_eff_value(p, UCLAMP_MAX);

	cpu_cap = capacity_orig_of(cpu);

	return cpu_cap >= min(min_cap, max_cap);
}

/*****************************************************************************/
/*                       New Code Section                                    */
/*****************************************************************************/
/*
 * This part of code is new for this kernel, which are mostly helper functions.
 */

static int find_least_loaded_cpu(struct task_struct *p, struct cpumask *lowest_mask)
{
	int cpu, best_cpu = -1;
	unsigned long min_cpu_util = ULONG_MAX;
	unsigned long min_cpu_capacity = ULONG_MAX;
	unsigned int min_exit_lat = UINT_MAX;
	bool check_cpu_overutilized = true;

	rcu_read_lock();

redo:
	for_each_cpu(cpu, lowest_mask) {
		struct cpuidle_state *idle;
		unsigned long util;
		unsigned int exit_lat = 0;
		unsigned long capacity = capacity_orig_of(cpu);

		if (cpu_is_idle(cpu)) {
			util = 0;
			idle = idle_get_state(cpu_rq(cpu));

			if (idle)
				exit_lat = idle->exit_latency;

			if (sched_cpu_idle(cpu))
				exit_lat = 0;
		} else {
			util = cpu_util(cpu) + cpu_util_rt(cpu_rq(cpu));
		}

		if (check_cpu_overutilized &&
		    cpu_overutilized(uclamp_rq_util_with(cpu_rq(cpu), util, p), capacity))
			continue;

		/* Always prefer the least loaded cpu. */
		if (util > min_cpu_util)
			continue;

		/* If util is the same: */
		if (util == min_cpu_util) {
			/* Prefer lower exit latency. */
			if (exit_lat > min_exit_lat)
				continue;
			/* If exit latency is the same: */
			if (exit_lat == min_exit_lat) {
				/* Prefer lower capacity. */
				if (capacity > min_cpu_capacity )
					continue;
				/* If capacity is the same, prefer prev cpu */
				if (best_cpu == task_cpu(p))
					continue;
			}
		}

		min_cpu_util = util;
		min_cpu_capacity = capacity;
		min_exit_lat = exit_lat;
		best_cpu = cpu;
	}

	if (best_cpu == -1 && check_cpu_overutilized) {
		check_cpu_overutilized = false;
		goto redo;
	}

	rcu_read_unlock();

	return best_cpu;
}


/*****************************************************************************/
/*                       Modified Code Section                               */
/*****************************************************************************/
/*
 * This part of code is vendor hook functions, which modify or extend the original
 * functions.
 */

static int find_lowest_rq(struct task_struct *p)
{
	struct sched_domain *sd;
	struct cpumask lowest_mask;
	int this_cpu = smp_processor_id();
	int cpu;
	int ret;

	if (p->nr_cpus_allowed == 1) {
		return cpumask_first(p->cpus_ptr);
	}

	ret = cpupri_find_fitness(&task_rq(p)->rd->cpupri, p, &lowest_mask, rt_task_fits_capacity);
	if (!ret) {
		return -1;
	}

	cpu = find_least_loaded_cpu(p, &lowest_mask);
	if (cpu != -1) {
		return cpu;
	}

	cpu = task_cpu(p);
	if (cpumask_test_cpu(cpu, &lowest_mask)) {
		return cpu;
	}

	if (!cpumask_test_cpu(this_cpu, &lowest_mask))
		this_cpu = -1;

	rcu_read_lock();
	for_each_domain(cpu, sd) {
		if (sd->flags & SD_WAKE_AFFINE) {
			int best_cpu;

			if (this_cpu != -1 &&
			    cpumask_test_cpu(this_cpu, sched_domain_span(sd))) {
				rcu_read_unlock();
				return this_cpu;
			}

			best_cpu = cpumask_first_and(&lowest_mask,
						     sched_domain_span(sd));
			if (best_cpu < nr_cpu_ids) {
				rcu_read_unlock();
				return best_cpu;
			}
		}
	}
	rcu_read_unlock();

	if (this_cpu != -1) {
		return this_cpu;
	}

	cpu = cpumask_any(&lowest_mask);
	if (cpu < nr_cpu_ids) {
		return cpu;
	}

	return -1;
}

void rvh_select_task_rq_rt_pixel_mod(void *data, struct task_struct *p, int prev_cpu, int sd_flag,
				     int wake_flags, int *new_cpu)
{
	struct task_struct *curr, *tgt_task;
	struct rq *rq;
	int target;

	*new_cpu = prev_cpu;

	if (sd_flag != SD_BALANCE_WAKE && sd_flag != SD_BALANCE_FORK)
		goto out;

	rq = cpu_rq(prev_cpu);

	rcu_read_lock();
	curr = READ_ONCE(rq->curr);

	target = find_lowest_rq(p);

	if (target != -1) {
		tgt_task = READ_ONCE(cpu_rq(target)->curr);
		if (task_may_not_preempt(tgt_task, target))
			target = find_lowest_rq(p);
	}

	if (target != -1 &&
	   (p->prio < cpu_rq(target)->rt.highest_prio.curr ||
	   task_may_not_preempt(curr, prev_cpu))) {
		*new_cpu = target;
	}

	rcu_read_unlock();
out:
	return;
}
