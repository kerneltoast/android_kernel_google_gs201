// SPDX-License-Identifier: GPL-2.0-only
/* rt.c
 *
 * Android Vendor Hook Support
 *
 * Copyright 2020 Google LLC
 */
#include <kernel/sched/sched.h>
#include <kernel/sched/pelt.h>

#include "sched_priv.h"
#include "sched_events.h"

extern unsigned long cpu_util(int cpu);
extern unsigned long task_util(struct task_struct *p);
extern bool task_may_not_preempt(struct task_struct *task, int cpu);
extern int cpu_is_idle(int cpu);
extern int sched_cpu_idle(int cpu);
extern bool get_prefer_high_cap(struct task_struct *p);

extern ___update_load_sum(u64 now, struct sched_avg *sa,
			  unsigned long load, unsigned long runnable, int running);
extern ___update_load_avg(struct sched_avg *sa, unsigned long load);

/*****************************************************************************/
/*                       Upstream Code Section                               */
/*****************************************************************************/
/*
 * This part of code is copied from Android common GKI kernel and unmodified.
 * Any change for these functions in upstream GKI would require extensive review
 * to make proper adjustment in vendor hook.
 */

#if defined CONFIG_SMP || defined CONFIG_RT_GROUP_SCHED
static inline bool should_honor_rt_sync(struct rq *rq, struct task_struct *p,
					bool sync)
{
	/*
	 * If the waker is CFS, then an RT sync wakeup would preempt the waker
	 * and force it to run for a likely small time after the RT wakee is
	 * done. So, only honor RT sync wakeups from RT wakers.
	 */
	return sync && task_has_rt_policy(rq->curr) &&
		p->prio <= rq->rt.highest_prio.next &&
		rq->rt.rt_nr_running <= 2;
}
#else
static inline bool should_honor_rt_sync(struct rq *rq, struct task_struct *p,
					bool sync)
{
	return 0;
}
#endif

/*****************************************************************************/
/*                       New Code Section                                    */
/*****************************************************************************/
/*
 * This part of code is new for this kernel, which are mostly helper functions.
 */
static inline void rt_task_fits_capacity(struct task_struct *p, int cpu,
					 bool *fits, bool *fits_original)
{
	unsigned long uclamp_min = uclamp_eff_value(p, UCLAMP_MIN);
	unsigned long uclamp_max = uclamp_eff_value(p, UCLAMP_MAX);
	unsigned long util = task_util(p);

	if (get_prefer_high_cap(p) && cpu < MID_CAPACITY_CPU) {
		*fits = false;
		*fits_original = false;
		return;
	}

	*fits = util_fits_cpu(util, uclamp_min, uclamp_max, cpu);
	*fits_original = capacity_orig_of(cpu) >= clamp(util, uclamp_min, uclamp_max) ||
			 cpu >= MAX_CAPACITY_CPU;
}

static int find_least_loaded_cpu(struct task_struct *p, struct cpumask *lowest_mask,
				 struct cpumask *backup_mask)
{
	unsigned long util[CPU_NUM] = { 0 };
	unsigned long capacity[CPU_NUM] = { 0 };
	unsigned int cpu_importance[CPU_NUM] = { 0 };
	unsigned int exit_lat[CPU_NUM] = { 0 };
	bool task_fits[CPU_NUM] = { 0 };
	bool task_fits_original[CPU_NUM] = { 0 };
	bool overutilize[CPU_NUM] = { 0 };
	int cpu, best_cpu = -1, best_busy_cpu, least_used_best_cpu;
	unsigned long min_cpu_util, min_unimportant_cpu_util;
	unsigned long min_cpu_capacity;
	unsigned int min_exit_lat;
	unsigned int best_busy_importance, least_importance;
	struct cpuidle_state *idle;
	int prev_cpu = task_cpu(p);
	bool is_idle;
	bool fit_and_non_overutilized_found = false, fit_and_overutilized_found = false;
	bool fit_orig_and_non_overutilized_found = false, fit_orig_and_overutilized_found = false;
	unsigned long rq_util_min, rq_util_max;

	if (cpumask_weight(lowest_mask) == 1)
		return cpumask_first(lowest_mask);

	rcu_read_lock();

	cpumask_clear(backup_mask);
	min_exit_lat = UINT_MAX;
	min_cpu_util = ULONG_MAX;
	min_unimportant_cpu_util = ULONG_MAX;
	min_cpu_capacity = ULONG_MAX;
	best_busy_importance = UINT_MAX;
	least_importance = UINT_MAX;
	best_busy_cpu = -1;
	least_used_best_cpu = -1;

	for_each_cpu(cpu, lowest_mask) {
		is_idle = cpu_is_idle(cpu);
		exit_lat[cpu] = 0;
		capacity[cpu] = capacity_orig_of(cpu);
		cpu_importance[cpu] = READ_ONCE(cpu_rq(cpu)->uclamp[UCLAMP_MIN].value) +
				 READ_ONCE(cpu_rq(cpu)->uclamp[UCLAMP_MAX].value);
		rq_util_min = uclamp_rq_get(cpu_rq(cpu), UCLAMP_MIN);
		rq_util_max = uclamp_rq_get(cpu_rq(cpu), UCLAMP_MAX);

		if (is_idle) {
			idle = idle_get_state(cpu_rq(cpu));

			if (idle)
				exit_lat[cpu] = idle->exit_latency;

			if (sched_cpu_idle(cpu)) {
				exit_lat[cpu] = 0;
				cpu_importance[cpu] = 0;
			}
		}

		util[cpu] = cpu_util(cpu) + cpu_util_rt(cpu_rq(cpu));
		if (cpu != prev_cpu)
			util[cpu] += task_util(p);

		rt_task_fits_capacity(p, cpu, &task_fits[cpu], &task_fits_original[cpu]);
		overutilize[cpu] = !util_fits_cpu(util[cpu],
						  rq_util_min, rq_util_max, cpu);

		trace_sched_cpu_util_rt(cpu, capacity[cpu], capacity_of(cpu), util[cpu],
					exit_lat[cpu], cpu_importance[cpu], task_fits[cpu],
					task_fits_original[cpu], overutilize[cpu], is_idle);

		// To prefer idle cpu than non-idle cpu
		if (is_idle)
			util[cpu] = 0;

		if (task_fits[cpu]) {
			fit_and_non_overutilized_found |= !overutilize[cpu];
			fit_and_overutilized_found |= overutilize[cpu];
		} else if (task_fits_original[cpu]) {
			fit_orig_and_non_overutilized_found |= !overutilize[cpu];
			fit_orig_and_overutilized_found |= overutilize[cpu];
		}
	}

	// no CPU fits, select the biggest capacity CPU instead
	if (!fit_and_non_overutilized_found && !fit_and_overutilized_found &&
		!fit_orig_and_non_overutilized_found && !fit_orig_and_overutilized_found) {
		best_cpu = cpumask_last(lowest_mask);
		rcu_read_unlock();
		goto out;
	}

	for_each_cpu(cpu, lowest_mask) {
		if (fit_and_non_overutilized_found && (overutilize[cpu] || !task_fits[cpu]))
			continue;
		else if (fit_and_overutilized_found && (!task_fits[cpu]))
			continue;
		else if (fit_orig_and_non_overutilized_found &&
			(overutilize[cpu] || !task_fits_original[cpu]))
			continue;
		else if (fit_orig_and_overutilized_found && (!task_fits_original[cpu]))
			continue;

		/* select non-idle cpus with throttled(unimportant) tasks first */
		if (exit_lat[cpu] == 0 && cpu_importance[cpu] < DEFAULT_IMPRATANCE_THRESHOLD) {
			cpumask_set_cpu(cpu, backup_mask);

			/* Always prefer the least important cpu. */
			if (best_busy_importance < cpu_importance[cpu])
				continue;

			/* If importance is the same, choose the least loaded cpu. */
			if (best_busy_importance == cpu_importance[cpu] &&
					min_unimportant_cpu_util < util[cpu])
				continue;

			best_busy_importance = cpu_importance[cpu];
			best_busy_cpu = cpu;
			min_unimportant_cpu_util = util[cpu];
			continue;
		}

		/* Always prefer cpu with the least importance. */
		if (cpu_importance[cpu] > least_importance)
			continue;

		/* If cpu importance is the same: */
		if (cpu_importance[cpu] == least_importance) {
			/* Prefer the least loaded cpu. */
			if (util[cpu] > min_cpu_util)
				continue;

			/* If util is the same: */
			if (util[cpu] == min_cpu_util) {
				/* Prefer lower exit latency. */
				if (exit_lat[cpu] > min_exit_lat)
					continue;
				/* If exit latency is the same: */
				if (exit_lat[cpu] == min_exit_lat) {
					/* Prefer lower capacity. */
					if (capacity[cpu] > min_cpu_capacity )
						continue;
					/* If capacity is the same, prefer prev cpu */
					if (least_used_best_cpu == prev_cpu)
						continue;
				}
			}
		}

		least_importance = cpu_importance[cpu];
		min_cpu_util = util[cpu];
		min_cpu_capacity = capacity[cpu];
		min_exit_lat = exit_lat[cpu];
		least_used_best_cpu = cpu;
	}

	rcu_read_unlock();

	if (best_busy_cpu != -1) {
		best_cpu = best_busy_cpu;
		/* backup_mask always contains best_busy_cpu.  */
		cpumask_clear_cpu(best_busy_cpu, backup_mask);

		/* No other backup, use least_used_best_cpu as backup. */
		if (cpumask_empty(backup_mask) && least_used_best_cpu != -1)
			cpumask_set_cpu(least_used_best_cpu, backup_mask);
	} else if (least_used_best_cpu != -1) {
		/* Fall back to least_used_best_cpu if there is no non-important cpu. */
		best_cpu = least_used_best_cpu;
	}

out:
	trace_sched_find_least_loaded_cpu(p, get_vendor_group(p), uclamp_eff_value(p, UCLAMP_MIN),
					  uclamp_eff_value(p, UCLAMP_MAX), get_prefer_high_cap(p),
					  best_busy_cpu, least_used_best_cpu, prev_cpu, best_cpu,
					  *lowest_mask->bits, *backup_mask->bits);

	return best_cpu;
}

/*****************************************************************************/
/*                       Modified Code Section                               */
/*****************************************************************************/
/*
 * This part of code is vendor hook functions, which modify or extend the original
 * functions.
 */

static int find_lowest_rq(struct task_struct *p, struct cpumask *backup_mask)
{
	struct sched_domain *sd;
	struct cpumask lowest_mask;
	int this_cpu = smp_processor_id();
	int cpu;
	int ret;

	if (p->nr_cpus_allowed == 1) {
		return cpumask_first(p->cpus_ptr);
	}

	ret = cpupri_find_fitness(&task_rq(p)->rd->cpupri, p, &lowest_mask, NULL);
	if (!ret) {
		return -1;
	}

	cpu = find_least_loaded_cpu(p, &lowest_mask, backup_mask);
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
	struct rq *this_cpu_rq;
	int target = -1;
	bool sync = !!(wake_flags & WF_SYNC);
	int this_cpu;
	bool sync_wakeup = false;
	struct cpumask backup_mask;
	int i;
	bool fits;
	bool fits_original;

	*new_cpu = prev_cpu;

	if (sd_flag != SD_BALANCE_WAKE && sd_flag != SD_BALANCE_FORK)
		goto out;

	rq = cpu_rq(prev_cpu);

	rcu_read_lock();
	curr = READ_ONCE(rq->curr);
	this_cpu = smp_processor_id();
	this_cpu_rq = cpu_rq(this_cpu);

	rt_task_fits_capacity(p, this_cpu, &fits, &fits_original);
	/*
	 * Respect the sync flag as long as the task can run on this CPU.
	 */
	if (should_honor_rt_sync(this_cpu_rq, p, sync) &&
	    cpumask_test_cpu(this_cpu, p->cpus_ptr) && fits_original) {
		*new_cpu = this_cpu;
		sync_wakeup = true;
		goto out_unlock;
	}

	target = find_lowest_rq(p, &backup_mask);

	if (target != -1) {
		tgt_task = READ_ONCE(cpu_rq(target)->curr);
		if (task_may_not_preempt(tgt_task, target) ||
			p->prio >= cpu_rq(target)->rt.highest_prio.curr) {
			target = -1;

			for_each_cpu(i, &backup_mask) {
				tgt_task = READ_ONCE(cpu_rq(i)->curr);
				if (task_may_not_preempt(tgt_task, i) ||
					p->prio >= cpu_rq(i)->rt.highest_prio.curr) {
					continue;
				} else {
					target = i;
					break;
				}
			}
		}
	}

	if (target != -1 &&
	   (p->prio < cpu_rq(target)->rt.highest_prio.curr ||
	   task_may_not_preempt(curr, prev_cpu))) {
		*new_cpu = target;
	}

out_unlock:
	rcu_read_unlock();
out:
	trace_sched_select_task_rq_rt(p, task_util(p), prev_cpu, target, *new_cpu, sync_wakeup);

	return;
}

void init_vendor_rt_rq(void)
{
	int i;
	struct vendor_rq_struct *vrq;

	for (i = 0; i < CPU_NUM; i++) {
		vrq = get_vendor_rq_struct(cpu_rq(i));
		raw_spin_lock_init(&vrq->lock);
		vrq->util_removed = 0;
	}
}

static int update_load_avg_se(u64 now, struct sched_entity *se, int running)
{
	if (___update_load_sum(now, &se->avg, running, 0, running)) {
		___update_load_avg(&se->avg, se_weight(se));
		trace_pelt_se_tp(se);
		return 1;
	}

	return 0;
}

void rvh_update_rt_rq_load_avg_pixel_mod(void *data, u64 now, struct rq *rq, struct task_struct *p,
					 int running)
{
	unsigned long removed_util = 0;
	u32 divider = get_pelt_divider(&rq->avg_rt);
	struct vendor_rq_struct *vrq = get_vendor_rq_struct(rq);

	// RT task p got migrated to this cpu, add its load to the rt util of rq
	if (!p->se.avg.last_update_time) {
		p->se.avg.last_update_time = rq->avg_rt.last_update_time;
		p->se.avg.util_sum = p->se.avg.util_avg * divider;
		rq->avg_rt.util_avg += p->se.avg.util_avg;
		rq->avg_rt.util_sum += p->se.avg.util_sum;
	}

	// One or more rt tasks previously ran on this cpu got migrated to other cpus, need to
	// remove its/their loading from the rt util of rq.
	if (vrq->util_removed) {
		raw_spin_lock(&vrq->lock);
		swap(vrq->util_removed, removed_util);
		raw_spin_unlock(&vrq->lock);
		sub_positive(&rq->avg_rt.util_avg, removed_util);
		sub_positive(&rq->avg_rt.util_sum, removed_util * divider);
		rq->avg_rt.util_sum = max_t(unsigned long, rq->avg_rt.util_sum,
					    rq->avg_rt.util_avg * PELT_MIN_DIVIDER);
	}

	// Update rt task util
	update_load_avg_se(rq_clock_pelt(rq), &p->se, running);
}

// For RT task only, used for task migration.
void rvh_set_task_cpu_pixel_mod(void *data, struct task_struct *p, unsigned int new_cpu)
{
	struct vendor_rq_struct *vrq;
	unsigned long flags;

	if (p->prio >= MAX_RT_PRIO)
		return;

	vrq = get_vendor_rq_struct(cpu_rq(task_cpu(p)));

	raw_spin_lock_irqsave(&vrq->lock, flags);
	vrq->util_removed += p->se.avg.util_avg;
	raw_spin_unlock_irqrestore(&vrq->lock, flags);

	p->se.avg.last_update_time = 0;
}

void vh_dump_throttled_rt_tasks_mod(void *data, int cpu, u64 clock, ktime_t rt_period,
				    u64 rt_runtime, s64 rt_period_timer_expires)
{
#ifdef CONFIG_SCHED_INFO
	printk_deferred("RT %s (%d) run %llu ns over rt throttled threshold %llu ns on cpu %d",
			current->comm,
			current->pid,
			clock - current->sched_info.last_arrival,
			rt_runtime,
			cpu);
#else
	printk_deferred("RT %s (%d) run over rt throttled threshold %llu ns on cpu %d",
			current->comm, current->pid, rt_runtime, cpu);
#endif
}
