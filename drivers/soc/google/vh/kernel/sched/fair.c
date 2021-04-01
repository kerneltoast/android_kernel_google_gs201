// SPDX-License-Identifier: GPL-2.0-only
/* fair.c
 *
 * Android Vendor Hook Support
 *
 * Copyright 2020 Google LLC
 */
#include <kernel/sched/sched.h>
#include <kernel/sched/pelt.h>

#include "sched.h"
#include "sched_events.h"

#if IS_ENABLED(CONFIG_UCLAMP_STATS)
extern void update_uclamp_stats(int cpu, u64 time);
#endif

extern bool vendor_sched_enable_prefer_high_cap;
extern bool vendor_sched_task_spreading_enable;
extern unsigned int vendor_sched_uclamp_threshold;
extern unsigned int vendor_sched_util_threshold;
extern unsigned int vendor_sched_high_capacity_start_cpu;

static unsigned int sched_capacity_margin[CPU_NUM] = {
			[0 ... CPU_NUM-1] = DEF_UTIL_THRESHOLD};
static unsigned long scale_freq[CPU_NUM] = {
			[0 ... CPU_NUM-1] = SCHED_CAPACITY_SCALE };

unsigned long schedutil_cpu_util_pixel_mod(int cpu, unsigned long util_cfs,
				 unsigned long max, enum schedutil_type type,
				 struct task_struct *p);

/*****************************************************************************/
/*                       Upstream Code Section                               */
/*****************************************************************************/
/*
 * This part of code is copied from Android common GKI kernel and unmodified.
 * Any change for these functions in upstream GKI would require extensive review
 * to make proper adjustment in vendor hook.
 */
#define lsub_positive(_ptr, _val) do {				\
	typeof(_ptr) ptr = (_ptr);				\
	*ptr -= min_t(typeof(*ptr), *ptr, _val);		\
} while (0)

#define sub_positive(_ptr, _val) do {				\
	typeof(_ptr) ptr = (_ptr);				\
	typeof(*ptr) val = (_val);				\
	typeof(*ptr) res, var = READ_ONCE(*ptr);		\
	res = var - val;					\
	if (res > var)						\
		res = 0;					\
	WRITE_ONCE(*ptr, res);					\
} while (0)

#if IS_ENABLED(CONFIG_FAIR_GROUP_SCHED)
static inline struct task_struct *task_of(struct sched_entity *se)
{
	SCHED_WARN_ON(!entity_is_task(se));
	return container_of(se, struct task_struct, se);
}

static inline struct cfs_rq *cfs_rq_of(struct sched_entity *se)
{
	return se->cfs_rq;
}
#else
static inline struct task_struct *task_of(struct sched_entity *se)
{
	return container_of(se, struct task_struct, se);
}

static inline struct cfs_rq *cfs_rq_of(struct sched_entity *se)
{
	struct task_struct *p = task_of(se);
	struct rq *rq = task_rq(p);

	return &rq->cfs;
}
#endif

#if !IS_ENABLED(CONFIG_64BIT)
static inline u64 cfs_rq_last_update_time(struct cfs_rq *cfs_rq)
{
	u64 last_update_time_copy;
	u64 last_update_time;

	do {
		last_update_time_copy = cfs_rq->load_last_update_time_copy;
		smp_rmb();
		last_update_time = cfs_rq->avg.last_update_time;
	} while (last_update_time != last_update_time_copy);

	return last_update_time;
}
#else
static inline u64 cfs_rq_last_update_time(struct cfs_rq *cfs_rq)
{
	return cfs_rq->avg.last_update_time;
}
#endif

unsigned long task_util(struct task_struct *p)
{
	return READ_ONCE(p->se.avg.util_avg);
}

static inline unsigned long _task_util_est(struct task_struct *p)
{
	struct util_est ue = READ_ONCE(p->se.avg.util_est);

	return (max(ue.ewma, ue.enqueued) | UTIL_AVG_UNCHANGED);
}

static inline unsigned long task_util_est(struct task_struct *p)
{
	return max(task_util(p), _task_util_est(p));
}

#if IS_ENABLED(CONFIG_UCLAMP_TASK)
static inline unsigned long uclamp_task_util(struct task_struct *p)
{
	return clamp(task_util_est(p),
		     uclamp_eff_value(p, UCLAMP_MIN),
		     uclamp_eff_value(p, UCLAMP_MAX));
}
#else
static inline unsigned long uclamp_task_util(struct task_struct *p)
{
	return task_util_est(p);
}
#endif

static inline unsigned long capacity_of(int cpu)
{
	return cpu_rq(cpu)->cpu_capacity;
}

unsigned long cpu_util(int cpu)
{
	struct cfs_rq *cfs_rq;
	unsigned int util;

	cfs_rq = &cpu_rq(cpu)->cfs;
	util = READ_ONCE(cfs_rq->avg.util_avg);

	if (sched_feat(UTIL_EST))
		util = max(util, READ_ONCE(cfs_rq->avg.util_est.enqueued));

	return min_t(unsigned long, util, capacity_of(cpu));
}

static unsigned long cpu_util_without(int cpu, struct task_struct *p)
{
	struct cfs_rq *cfs_rq;
	unsigned int util;

	/* Task has no contribution or is new */
	if (cpu != task_cpu(p) || !READ_ONCE(p->se.avg.last_update_time))
		return cpu_util(cpu);

	cfs_rq = &cpu_rq(cpu)->cfs;
	util = READ_ONCE(cfs_rq->avg.util_avg);

	/* Discount task's util from CPU's util */
	lsub_positive(&util, task_util(p));

	/*
	 * Covered cases:
	 *
	 * a) if *p is the only task sleeping on this CPU, then:
	 *      cpu_util (== task_util) > util_est (== 0)
	 *    and thus we return:
	 *      cpu_util_without = (cpu_util - task_util) = 0
	 *
	 * b) if other tasks are SLEEPING on this CPU, which is now exiting
	 *    IDLE, then:
	 *      cpu_util >= task_util
	 *      cpu_util > util_est (== 0)
	 *    and thus we discount *p's blocked utilization to return:
	 *      cpu_util_without = (cpu_util - task_util) >= 0
	 *
	 * c) if other tasks are RUNNABLE on that CPU and
	 *      util_est > cpu_util
	 *    then we use util_est since it returns a more restrictive
	 *    estimation of the spare capacity on that CPU, by just
	 *    considering the expected utilization of tasks already
	 *    runnable on that CPU.
	 *
	 * Cases a) and b) are covered by the above code, while case c) is
	 * covered by the following code when estimated utilization is
	 * enabled.
	 */
	if (sched_feat(UTIL_EST)) {
		unsigned int estimated =
			READ_ONCE(cfs_rq->avg.util_est.enqueued);

		/*
		 * Despite the following checks we still have a small window
		 * for a possible race, when an execl's select_task_rq_fair()
		 * races with LB's detach_task():
		 *
		 *   detach_task()
		 *     p->on_rq = TASK_ON_RQ_MIGRATING;
		 *     ---------------------------------- A
		 *     deactivate_task()                   \
		 *       dequeue_task()                     + RaceTime
		 *         util_est_dequeue()              /
		 *     ---------------------------------- B
		 *
		 * The additional check on "current == p" it's required to
		 * properly fix the execl regression and it helps in further
		 * reducing the chances for the above race.
		 */
		if (unlikely(task_on_rq_queued(p) || current == p))
			lsub_positive(&estimated, _task_util_est(p));

		util = max(util, estimated);
	}

	/*
	 * Utilization (estimated) can exceed the CPU capacity, thus let's
	 * clamp to the maximum CPU capacity to ensure consistency with
	 * the cpu_util call.
	 */
	return min_t(unsigned long, util, capacity_of(cpu));
}

static void sync_entity_load_avg(struct sched_entity *se)
{
	struct cfs_rq *cfs_rq = cfs_rq_of(se);
	u64 last_update_time;

	last_update_time = cfs_rq_last_update_time(cfs_rq);
	__update_load_avg_blocked_se(last_update_time, se);
}

static unsigned long capacity_curr_of(int cpu)
{
	unsigned long max_cap = cpu_rq(cpu)->cpu_capacity_orig;

	return cap_scale(max_cap, scale_freq[cpu]);
}

/* Runqueue only has SCHED_IDLE tasks enqueued */
static int sched_idle_rq(struct rq *rq)
{
	return unlikely(rq->nr_running == rq->cfs.idle_h_nr_running &&
			rq->nr_running);
}

#ifdef CONFIG_SMP
int sched_cpu_idle(int cpu)
{
	return sched_idle_rq(cpu_rq(cpu));
}
#endif

static inline bool within_margin(int value, int margin)
{
	return ((unsigned int)(value + margin - 1) < (2 * margin - 1));
}

/*****************************************************************************/
/*                       New Code Section                                    */
/*****************************************************************************/
/*
 * This part of code is new for this kernel, which are mostly helper functions.
 */
static inline bool get_prefer_high_cap(struct task_struct *p)
{
	return vendor_sched_enable_prefer_high_cap && get_vendor_task_struct(p)->prefer_high_cap;
}

static inline bool get_task_spreading(struct task_struct *p)
{
	return vendor_sched_task_spreading_enable && get_vendor_task_struct(p)->task_spreading;
}

bool task_fits_capacity(struct task_struct *p, int cpu)
{
	unsigned long task_util;

	if (cpu >= MAX_CAPACITY_CPU)
		return true;

	task_util = get_task_spreading(p) ? task_util_est(p) : uclamp_task_util(p);

	return capacity_of(cpu) * 1024 > task_util * sched_capacity_margin[cpu];
}

static inline int find_start_cpu(struct task_struct *p, bool prefer_high_cap, bool sync_boost)
{
	if (!prefer_high_cap && !sync_boost && task_fits_capacity(p, MIN_CAPACITY_CPU))
		return MIN_CAPACITY_CPU;
	else
		return vendor_sched_high_capacity_start_cpu;
}

static inline bool is_min_capacity_cpu(int cpu)
{
	return cpu < HIGH_CAPACITY_CPU;
}

static inline bool cpu_is_in_target_set(struct task_struct *p, int cpu)
{
	int first_cpu, next_usable_cpu;

	if (get_prefer_high_cap(p)) {
		first_cpu = HIGH_CAPACITY_CPU;
	} else {
		first_cpu = MIN_CAPACITY_CPU;
	}

	next_usable_cpu = cpumask_next(first_cpu - 1, p->cpus_ptr);
	return cpu >= next_usable_cpu || next_usable_cpu >= nr_cpu_ids;
}

/**
 * cpu_is_idle - is a given CPU idle for enqueuing work.
 * @cpu: the CPU in question.
 *
 * Return: 1 if the CPU is currently idle. 0 otherwise.
 */
int cpu_is_idle(int cpu)
{
	if (available_idle_cpu(cpu) || sched_cpu_idle(cpu))
		return 1;

	return 0;
}

static unsigned long cpu_util_next(int cpu, struct task_struct *p, int dst_cpu)
{
	struct cfs_rq *cfs_rq, *pos;
	unsigned long util = 0, unclamped_util = 0;
	unsigned long util_est;
	struct task_group *tg;
	int delta = 0;
	unsigned long group_util = 0;
	struct rq *rq = cpu_rq(cpu);

	if (task_cpu(p) == cpu && dst_cpu != cpu)
		delta = -task_util(p);
	else if (task_cpu(p) != cpu && dst_cpu == cpu)
		delta = task_util(p);

	// For leaf groups
	for_each_leaf_cfs_rq_safe(rq, cfs_rq, pos) {
		if (&rq->cfs != cfs_rq) {
			tg = cfs_rq->tg;

			if (p->se.cfs_rq->tg == tg)
				group_util = max_t(int, READ_ONCE(cfs_rq->avg.util_avg) + delta, 0);
			else
				group_util = READ_ONCE(cfs_rq->avg.util_avg);

			unclamped_util += cfs_rq->avg.util_avg;
			util += min_t(unsigned long, group_util, tg->uclamp_req[UCLAMP_MAX].value);
		}
	}

	// For root group
	if (p->se.cfs_rq->tg == rq->cfs.tg)
		group_util = max_t(int, READ_ONCE(rq->cfs.avg.util_avg) - unclamped_util + delta,
				   0);
	else
		group_util = max_t(int, READ_ONCE(rq->cfs.avg.util_avg) - unclamped_util, 0);

	util += group_util;

	if (sched_feat(UTIL_EST)) {
		util_est = READ_ONCE(cfs_rq->avg.util_est.enqueued);

		if (dst_cpu == cpu)
			util_est += _task_util_est(p);

		util = max_t(unsigned long, util, util_est);
	}

	return min(util, capacity_of(cpu));
}

static long
compute_energy(struct task_struct *p, int dst_cpu, struct perf_domain *pd)
{
	unsigned int max_util, util_cfs, cpu_util, cpu_cap;
	unsigned long sum_util, energy = 0;
	struct task_struct *tsk;
	int cpu;

	for (; pd; pd = pd->next) {
		struct cpumask *pd_mask = perf_domain_span(pd);

		/*
		 * The energy model mandates all the CPUs of a performance
		 * domain have the same capacity.
		 */
		cpu_cap = arch_scale_cpu_capacity(cpumask_first(pd_mask));
		max_util = sum_util = 0;

		/*
		 * The capacity state of CPUs of the current rd can be driven by
		 * CPUs of another rd if they belong to the same performance
		 * domain. So, account for the utilization of these CPUs too
		 * by masking pd with cpu_online_mask instead of the rd span.
		 *
		 * If an entire performance domain is outside of the current rd,
		 * it will not appear in its pd list and will not be accounted
		 * by compute_energy().
		 */
		for_each_cpu_and(cpu, pd_mask, cpu_online_mask) {
			util_cfs = cpu_util_next(cpu, p, dst_cpu);

			/*
			 * Busy time computation: utilization clamping is not
			 * required since the ratio (sum_util / cpu_capacity)
			 * is already enough to scale the EM reported power
			 * consumption at the (eventually clamped) cpu_capacity.
			 */
			sum_util += schedutil_cpu_util_pixel_mod(cpu, util_cfs, cpu_cap,
						       ENERGY_UTIL, NULL);

			/*
			 * Performance domain frequency: utilization clamping
			 * must be considered since it affects the selection
			 * of the performance domain frequency.
			 * NOTE: in case RT tasks are running, by default the
			 * FREQUENCY_UTIL's utilization can be max OPP.
			 */
			tsk = cpu == dst_cpu ? p : NULL;
			cpu_util = schedutil_cpu_util_pixel_mod(cpu, util_cfs, cpu_cap,
						      FREQUENCY_UTIL, tsk);
			max_util = max(max_util, cpu_util);
		}

		energy += em_cpu_energy(pd->em_pd, max_util, sum_util);
	}

	return energy;
}

static void find_best_target(cpumask_t *cpus, struct task_struct *p, int prev_cpu, bool sync_boost)
{
	unsigned long target_capacity = ULONG_MAX;
	unsigned long min_wake_util = ULONG_MAX;
	unsigned long target_max_spare_cap = 0;
	unsigned long target_util = ULONG_MAX;
	int best_active_cpu = -1;
	int best_idle_cpu = -1;
	int target_cpu = -1;
	int backup_cpu = -1;
	bool prefer_idle;
	bool prefer_high_cap;
	bool prefer_prev = false;
	int i;
	int start_cpu = -1;
	unsigned int min_exit_lat = UINT_MAX;
	unsigned long best_idle_util = ULONG_MAX;

	/*
	 * In most cases, target_capacity tracks capacity of the most
	 * energy efficient CPU candidate, thus requiring to minimise
	 * target_capacity. For these cases target_capacity is already
	 * initialized to ULONG_MAX.
	 * However, for prefer_idle and prefer_high_cap tasks we look for a high
	 * performance CPU, thus requiring to maximise target_capacity. In this
	 * case we initialise target_capacity to 0.
	 */
	prefer_idle = uclamp_latency_sensitive(p);
	prefer_high_cap = get_prefer_high_cap(p);
	if (prefer_idle && prefer_high_cap)
		target_capacity = 0;

	start_cpu = find_start_cpu(p, prefer_high_cap, sync_boost);

	/* prefer prev cpu */
	// TODO: add idle index check later
	if (cpu_online(prev_cpu) && cpu_is_idle(prev_cpu) && task_fits_capacity(p, prev_cpu) &&
		(capacity_orig_of(prev_cpu) == capacity_orig_of(start_cpu))) {
		target_cpu = prev_cpu;
		prefer_prev = true;
		goto target;
	}

	for_each_cpu_wrap(i, p->cpus_ptr, start_cpu) {
		unsigned long capacity_curr = capacity_curr_of(i);
		unsigned long capacity_orig = capacity_orig_of(i);
		unsigned long capacity = capacity_of(i);
		unsigned long wake_util, new_util;
		long spare_cap;
		struct cpuidle_state *idle = NULL;
		int next_cpu = cpumask_next_wrap(i, p->cpus_ptr, i, false);
		unsigned int exit_lat = UINT_MAX;

		trace_sched_cpu_util(i, cpu_util(i), capacity_curr, capacity);

		if (!cpu_active(i))
			goto check;

		/*
		 * p's blocked utilization is still accounted for on prev_cpu
		 * so prev_cpu will receive a negative bias due to the double
		 * accounting. However, the blocked utilization may be zero.
		 */
		wake_util = cpu_util_without(i, p);
		new_util = wake_util + task_util_est(p);

		/*
		 * There is no need to check the boosted util of the task since
		 * it is checked in task_fits_capacity already. It only needs
		 * to check total util here.
		 * However, if task prefers idle and cpu is idle, we skip this check.
		 */
		if (!(prefer_idle && cpu_is_idle(i)) && new_util > capacity)
			goto check;

		if (is_min_capacity_cpu(i) &&
		    !task_fits_capacity(p, i))
			goto check;

		/*
		 * Pre-compute the maximum possible capacity we expect
		 * to have available on this CPU once the task is
		 * enqueued here.
		 */
		spare_cap = capacity - new_util;

		if (cpu_is_idle(i))
			idle = idle_get_state(cpu_rq(i));

		/*
		 * Case A) Latency sensitive tasks
		 *
		 * Unconditionally favoring tasks that prefer idle CPU to
		 * improve latency.
		 *
		 * Looking for:
		 * - an idle CPU, whatever its idle_state is, since
		 *   the first CPUs we explore are more likely to be
		 *   reserved for latency sensitive tasks.
		 * - a non idle CPU where the task fits in its current
		 *   capacity and has the maximum spare capacity.
		 * - a non idle CPU with lower contention from other
		 *   tasks and running at the lowest possible OPP.
		 *
		 * The last two goals tries to favor a non idle CPU
		 * where the task can run as if it is "almost alone".
		 * A maximum spare capacity CPU is favoured since
		 * the task already fits into that CPU's capacity
		 * without waiting for an OPP chance.
		 *
		 * The following code path is the only one in the CPUs
		 * exploration loop which is always used by
		 * prefer_idle tasks. It exits the loop with wither a
		 * best_active_cpu or a target_cpu which should
		 * represent an optimal choice for latency sensitive
		 * tasks.
		 */
		if (prefer_idle) {
			/*
			 * Case A.1: IDLE CPU
			 * Return the best IDLE CPU we find:
			 * - for prefer_high_cap tasks: the CPU with the highest
			 * performance (i.e. biggest capacity)
			 * - for !prefer_high_cap tasks: the most energy
			 * efficient CPU (i.e. smallest capacity)
			 */
			if (cpu_is_idle(i)) {
				if (prefer_high_cap &&
				    capacity_orig < target_capacity)
					goto check;

				if (!prefer_high_cap &&
				    capacity_orig > target_capacity)
					goto check;
				/*
				 * Minimise value of idle state: skip
				 * deeper idle states and pick the
				 * shallowest. If idle state is the same,
				 * pick the least utilized cpu.
				 */
				if (sched_cpu_idle(i))
					exit_lat = 0;
				else if (idle)
					exit_lat = idle->exit_latency;

				if (exit_lat > min_exit_lat ||
				    (exit_lat == min_exit_lat &&
				     best_idle_util <= new_util))
					goto check;

				min_exit_lat = exit_lat;
				target_capacity = capacity_orig;
				best_idle_util = new_util;
				best_idle_cpu = i;
				goto check;
			}
			if (best_idle_cpu != -1)
				goto check;

			/*
			 * Case A.2: Target ACTIVE CPU
			 * Favor CPUs with max spare capacity.
			 */
			if (capacity_curr > new_util &&
			    spare_cap > target_max_spare_cap) {
				target_max_spare_cap = spare_cap;
				target_cpu = i;
				goto check;
			}
			if (target_cpu != -1)
				goto check;

			/*
			 * Case A.3: Backup ACTIVE CPU
			 * Favor CPUs with:
			 * - lower utilization due to other tasks
			 * - lower utilization with the task in
			 */
			if (wake_util > min_wake_util)
				goto check;

			min_wake_util = wake_util;
			best_active_cpu = i;
			goto check;
		}

		/*
		 * Favor CPUs with smaller capacity for non latency
		 * sensitive tasks.
		 */
		if (capacity_orig > target_capacity)
			goto check;

		/*
		 * Case B) Non latency sensitive tasks on IDLE CPUs.
		 *
		 * Find an optimal backup IDLE CPU for non latency
		 * sensitive tasks.
		 *
		 * Looking for:
		 * - minimizing the capacity,
		 *   i.e. preferring LITTLE CPUs
		 * - favoring shallowest idle states
		 *   i.e. avoid to wakeup deep-idle CPUs
		 *
		 * The following code path is used by non latency
		 * sensitive tasks if IDLE CPUs are available. If at
		 * least one of such CPUs are available it sets the
		 * best_idle_cpu to the most suitable idle CPU to be
		 * selected.
		 *
		 * If idle CPUs are available, favour these CPUs to
		 * improve performances by spreading tasks.
		 * Indeed, the energy_diff() computed by the caller
		 * will take care to ensure the minimization of energy
		 * consumptions without affecting performance.
		 */
		if (cpu_is_idle(i)) {
			/*
			 * Skip CPUs in deeper idle state, but only
			 * if they are also less energy efficient.
			 * IOW, prefer a deep IDLE LITTLE CPU vs a
			 * shallow idle big CPU. If idle state is the same,
			 * pick the least utilized cpu.
			 */
			if (sched_cpu_idle(i))
				exit_lat = 0;
			else if (idle)
				exit_lat = idle->exit_latency;

			if (exit_lat > min_exit_lat ||
			    (exit_lat == min_exit_lat &&
			     best_idle_util <= new_util))
				goto check;

			min_exit_lat = exit_lat;
			target_capacity = capacity_orig;
			best_idle_util = new_util;
			best_idle_cpu = i;
			goto check;
		}

		/*
		 * Case C) Non latency sensitive tasks on ACTIVE CPUs.
		 *
		 * Pack tasks in the most energy efficient capacities.
		 *
		 * This task packing strategy prefers more energy
		 * efficient CPUs (i.e. pack on smaller maximum
		 * capacity CPUs) while also trying to spread tasks to
		 * run them all at the lower OPP.
		 *
		 * This assumes for example that it's more energy
		 * efficient to run two tasks on two CPUs at a lower
		 * OPP than packing both on a single CPU but running
		 * that CPU at an higher OPP.
		 *
		 * Thus, this case keep track of the CPU with the
		 * smallest maximum capacity and highest spare maximum
		 * capacity.
		 */

		/* Favor CPUs with maximum spare capacity */
		if (capacity_orig == target_capacity &&
		    spare_cap < target_max_spare_cap)
			goto check;

		target_max_spare_cap = spare_cap;
		target_capacity = capacity_orig;
		target_util = new_util;
		target_cpu = i;

check:
		if (capacity_orig == capacity_orig_of(next_cpu))
			continue;

		if ((prefer_idle && best_idle_cpu != -1) ||
			(!prefer_idle && (target_cpu != -1 || best_idle_cpu != -1))) {
			break;
		}
	}

	/*
	 * For non latency sensitive tasks, cases B and C in the previous loop,
	 * we pick the best IDLE CPU only if we was not able to find a target
	 * ACTIVE CPU.
	 *
	 * Policies priorities:
	 *
	 * - prefer_idle tasks:
	 *
	 *   a) IDLE CPU available: best_idle_cpu
	 *   b) ACTIVE CPU where task fits and has the bigger maximum spare
	 *      capacity (i.e. target_cpu)
	 *   c) ACTIVE CPU with less contention due to other tasks
	 *      (i.e. best_active_cpu)
	 *
	 * - NON prefer_idle tasks:
	 *
	 *   a) ACTIVE CPU: target_cpu
	 *   b) IDLE CPU: best_idle_cpu
	 */

	/* Prefer idle cpu for prefer_idle or active migration tasks */
	if (prefer_idle && best_idle_cpu != -1) {
		target_cpu = best_idle_cpu;
		goto target;
	}

	if (target_cpu == -1)
		target_cpu = prefer_idle
			? best_active_cpu
			: best_idle_cpu;
	else
		backup_cpu = prefer_idle
		? best_active_cpu
		: best_idle_cpu;

	if (backup_cpu >= 0)
		cpumask_set_cpu(backup_cpu, cpus);
	if (target_cpu >= 0) {
target:
		cpumask_set_cpu(target_cpu, cpus);
	}

	trace_sched_find_best_target(p, prefer_idle, prefer_high_cap, prefer_prev, sync_boost,
				     task_util_est(p), start_cpu, best_idle_cpu, best_active_cpu,
				     target_cpu, backup_cpu);
}

static DEFINE_PER_CPU(cpumask_t, energy_cpus);

/*****************************************************************************/
/*                       Modified Code Section                               */
/*****************************************************************************/
/*
 * This part of code is vendor hook functions, which modify or extend the original
 * functions.
 */
void rvh_find_energy_efficient_cpu_pixel_mod(void *data, struct task_struct *p, int prev_cpu,
					     int sync, int *new_cpu)
{
	unsigned long prev_energy = ULONG_MAX, best_energy = ULONG_MAX;
	struct root_domain *rd = cpu_rq(smp_processor_id())->rd;
	int weight, cpu, best_energy_cpu = prev_cpu;
	unsigned long cur_energy;
	struct perf_domain *pd;
	cpumask_t *candidates;
	bool sync_boost;
	bool sync_wakeup = false;

	cpu = smp_processor_id();
	if (sync && cpu_rq(cpu)->nr_running == 1 && cpumask_test_cpu(cpu, p->cpus_ptr) &&
	    cpu_is_in_target_set(p, cpu) && task_fits_capacity(p, cpu)) {
		*new_cpu = cpu;
		sync_wakeup = true;
		goto out;
	}

	sync_boost = sync && cpu >= HIGH_CAPACITY_CPU;

	rcu_read_lock();
	pd = rcu_dereference(rd->pd);
	if (!pd || READ_ONCE(rd->overutilized))
		goto fail;

	sync_entity_load_avg(&p->se);

	/* Pre-select a set of candidate CPUs. */
	candidates = this_cpu_ptr(&energy_cpus);
	cpumask_clear(candidates);

	find_best_target(candidates, p, prev_cpu, sync_boost);

	/* Bail out if no candidate was found. */
	weight = cpumask_weight(candidates);
	if (!weight)
		goto unlock;

	/* If there is only one sensible candidate, select it now. */
	cpu = cpumask_first(candidates);
	if (weight == 1 && ((uclamp_latency_sensitive(p) && cpu_is_idle(cpu)) ||
			    (cpu == prev_cpu))) {
		best_energy_cpu = cpu;
		goto unlock;
	}

	if (cpumask_test_cpu(prev_cpu, p->cpus_ptr))
		prev_energy = best_energy = compute_energy(p, prev_cpu, pd);
	else
		prev_energy = best_energy = ULONG_MAX;

	/* Select the best candidate energy-wise. */
	for_each_cpu(cpu, candidates) {
		if (cpu == prev_cpu)
			continue;
		cur_energy = compute_energy(p, cpu, pd);
		if (cur_energy < best_energy) {
			best_energy = cur_energy;
			best_energy_cpu = cpu;
		}
	}
unlock:
	rcu_read_unlock();

	/*
	 * Pick the best CPU if prev_cpu cannot be used, or if it saves at
	 * least 6% of the energy used by prev_cpu.
	 */
	if (prev_energy == ULONG_MAX) {
		*new_cpu = best_energy_cpu;
		goto out;
	}

	if ((prev_energy - best_energy) > (prev_energy >> 4)) {
		*new_cpu = best_energy_cpu;
		goto out;
	}

	*new_cpu = prev_cpu;
	goto out;

fail:
	rcu_read_unlock();
	*new_cpu = -1;
out:
	trace_sched_find_energy_efficient_cpu(p, sync_wakeup, *new_cpu, best_energy_cpu, prev_cpu);
}

void vh_arch_set_freq_scale_pixel_mod(void *data, const struct cpumask *cpus,
				      unsigned long freq,
				      unsigned long max, unsigned long *scale)
{
	int i;

	for_each_cpu(i, cpus)
		scale_freq[i] = *scale;
}

void rvh_set_iowait_pixel_mod(void *data, struct task_struct *p, int *should_iowait_boost)
{
	*should_iowait_boost = p->in_iowait && uclamp_boosted(p);
}

void rvh_cpu_overutilized_pixel_mod(void *data, int cpu, int *overutilized)
{
	*overutilized = cpu_overutilized(cpu_util(cpu), capacity_of(cpu));
}

unsigned long map_util_freq_pixel_mod(unsigned long util, unsigned long freq,
				      unsigned long cap)
{
	return (freq * vendor_sched_util_threshold >> SCHED_CAPACITY_SHIFT) * util / cap;
}

void rvh_dequeue_task_pixel_mod(void *data, struct rq *rq, struct task_struct *p, int flags)
{
#if IS_ENABLED(CONFIG_UCLAMP_STATS)
	if (rq->nr_running == 1)
		update_uclamp_stats(rq->cpu, rq_clock(rq));
#endif
}

static inline bool check_uclamp_threshold(struct task_struct *p, enum uclamp_id clamp_id)
{
	if (clamp_id == UCLAMP_MIN && !rt_task(p) &&
	    task_util_est(p) < vendor_sched_uclamp_threshold) {
		return true;
	}
	return false;
}

static inline struct uclamp_se
uclamp_tg_restrict_pixel_mod(struct task_struct *p, enum uclamp_id clamp_id)
{
	struct uclamp_se uc_req = p->uclamp_req[clamp_id];
#ifdef CONFIG_UCLAMP_TASK_GROUP
	struct uclamp_se uc_max;

	/*
	 * Tasks in autogroups or root task group will be
	 * restricted by system defaults.
	 */
	if (task_group_is_autogroup(task_group(p)))
		return uc_req;
	if (task_group(p) == &root_task_group)
		return uc_req;

	uc_max = task_group(p)->uclamp[clamp_id];
	if ((clamp_id == UCLAMP_MAX && (uc_req.value > uc_max.value || !uc_req.user_defined)) ||
	    (clamp_id == UCLAMP_MIN && (uc_req.value < uc_max.value || !uc_req.user_defined)))
		return uc_max;
#endif

	return uc_req;
}

void rvh_uclamp_eff_get_pixel_mod(void *data, struct task_struct *p, enum uclamp_id clamp_id,
				  struct uclamp_se *uclamp_max, struct uclamp_se *uclamp_eff,
				  int *ret)
{
	struct uclamp_se uc_req;

	*ret = 1;

	/* Apply threshold first. */
	if (check_uclamp_threshold(p, clamp_id)) {
		uclamp_eff->value = 0;
		uclamp_eff->bucket_id = 0;
		return;
	}

	uc_req = uclamp_tg_restrict_pixel_mod(p, clamp_id);

	/* System default restrictions always apply */
	if (unlikely(uc_req.value > uclamp_max->value)) {
		*uclamp_eff = *uclamp_max;
		return;
	}

	*uclamp_eff = uc_req;
	return;
}

void update_sched_capacity_margin(unsigned int util_threshold)
{
	int i;

	for (i = 0; i < CPU_NUM; i++)
		sched_capacity_margin[i] = util_threshold;
}

void rvh_util_est_update_pixel_mod(void *data, struct cfs_rq *cfs_rq, struct task_struct *p,
				    bool task_sleep, int *ret)
{
	long last_ewma_diff;
	struct util_est ue;
	int cpu;

	*ret = 1;

	if (!sched_feat(UTIL_EST))
		return;

	/*
	 * Skip update of task's estimated utilization when the task has not
	 * yet completed an activation, e.g. being migrated.
	 */
	if (!task_sleep)
		return;

	/*
	 * If the PELT values haven't changed since enqueue time,
	 * skip the util_est update.
	 */
	ue = p->se.avg.util_est;
	if (ue.enqueued & UTIL_AVG_UNCHANGED)
		return;

	/*
	 * Reset EWMA on utilization increases, the moving average is used only
	 * to smooth utilization decreases.
	 */
	ue.enqueued = (task_util(p) | UTIL_AVG_UNCHANGED);

	// TODO: make util_est to sub cfs-rq and aggregate.
#ifdef CONFIG_UCLAMP_TASK
	// Currently util_est is done only in the root group
	// Current solution apply the clamp in the per-task level for simplicity.
	// However it may
	// 1) over grow by the group limit
	// 2) out of sync when task migrated between cgroups (cfs_rq)
	ue.enqueued = min((unsigned long)ue.enqueued, uclamp_eff_value(p, UCLAMP_MAX));
#endif

	if (sched_feat(UTIL_EST_FASTUP)) {
		if (ue.ewma < ue.enqueued) {
			ue.ewma = ue.enqueued;
			goto done;
		}
	}

	/*
	 * Skip update of task's estimated utilization when its EWMA is
	 * already ~1% close to its last activation value.
	 */
	last_ewma_diff = ue.enqueued - ue.ewma;
	if (within_margin(last_ewma_diff, (SCHED_CAPACITY_SCALE / 100)))
		return;

	/*
	 * To avoid overestimation of actual task utilization, skip updates if
	 * we cannot grant there is idle time in this CPU.
	 */
	cpu = cpu_of(rq_of(cfs_rq));
	if (task_util(p) > capacity_orig_of(cpu))
		return;

	/*
	 * Update Task's estimated utilization
	 *
	 * When *p completes an activation we can consolidate another sample
	 * of the task size. This is done by storing the current PELT value
	 * as ue.enqueued and by using this value to update the Exponential
	 * Weighted Moving Average (EWMA):
	 *
	 *  ewma(t) = w *  task_util(p) + (1-w) * ewma(t-1)
	 *          = w *  task_util(p) +         ewma(t-1)  - w * ewma(t-1)
	 *          = w * (task_util(p) -         ewma(t-1)) +     ewma(t-1)
	 *          = w * (      last_ewma_diff            ) +     ewma(t-1)
	 *          = w * (last_ewma_diff  +  ewma(t-1) / w)
	 *
	 * Where 'w' is the weight of new samples, which is configured to be
	 * 0.25, thus making w=1/4 ( >>= UTIL_EST_WEIGHT_SHIFT)
	 */
	ue.ewma <<= UTIL_EST_WEIGHT_SHIFT;
	ue.ewma  += last_ewma_diff;
	ue.ewma >>= UTIL_EST_WEIGHT_SHIFT;
#ifdef CONFIG_UCLAMP_TASK
	ue.ewma = min((unsigned long)ue.enqueued, uclamp_eff_value(p, UCLAMP_MAX));
#endif
done:
	WRITE_ONCE(p->se.avg.util_est, ue);

	trace_sched_util_est_se_tp(&p->se);
}
