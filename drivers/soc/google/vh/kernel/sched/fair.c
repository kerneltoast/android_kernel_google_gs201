// SPDX-License-Identifier: GPL-2.0-only
/* fair.c
 *
 * Android Vendor Hook Support
 *
 * Copyright 2020 Google LLC
 */
#include <kernel/sched/sched.h>
#include <kernel/sched/pelt.h>

#include "sched_priv.h"
#include "sched_events.h"
#include "../systrace.h"

#if IS_ENABLED(CONFIG_UCLAMP_STATS)
extern void update_uclamp_stats(int cpu, u64 time);
#endif

extern unsigned int vendor_sched_uclamp_threshold;
extern unsigned int vendor_sched_high_capacity_start_cpu;
extern unsigned int vendor_sched_util_post_init_scale;

static struct vendor_group_property vg[VG_MAX];

unsigned int sched_capacity_margin[CPU_NUM] = {
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

	return max(ue.ewma, (ue.enqueued & ~UTIL_AVG_UNCHANGED));
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

static inline void update_load_add(struct load_weight *lw, unsigned long inc)
{
	lw->weight += inc;
	lw->inv_weight = 0;
}

#define WMULT_CONST	(~0U)
#define WMULT_SHIFT	32

static void __update_inv_weight(struct load_weight *lw)
{
	unsigned long w;

	if (likely(lw->inv_weight))
		return;

	w = scale_load_down(lw->weight);

	if (BITS_PER_LONG > 32 && unlikely(w >= WMULT_CONST))
		lw->inv_weight = 1;
	else if (unlikely(!w))
		lw->inv_weight = WMULT_CONST;
	else
		lw->inv_weight = WMULT_CONST / w;
}

static u64 __calc_delta(u64 delta_exec, unsigned long weight, struct load_weight *lw)
{
	u64 fact = scale_load_down(weight);
	int shift = WMULT_SHIFT;

	__update_inv_weight(lw);

	if (unlikely(fact >> 32)) {
		while (fact >> 32) {
			fact >>= 1;
			shift--;
		}
	}

	fact = mul_u32_u32(fact, lw->inv_weight);

	while (fact >> 32) {
		fact >>= 1;
		shift--;
	}

	return mul_u64_u32_shr(delta_exec, fact, shift);
}

static u64 __sched_period(unsigned long nr_running);

#define for_each_sched_entity(se) \
		for (; se; se = se->parent)

static u64 sched_slice(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	u64 slice = __sched_period(cfs_rq->nr_running + !se->on_rq);

	for_each_sched_entity(se) {
		struct load_weight *load;
		struct load_weight lw;

		cfs_rq = cfs_rq_of(se);
		load = &cfs_rq->load;

		if (unlikely(!se->on_rq)) {
			lw = cfs_rq->load;

			update_load_add(&lw, se->load.weight);
			load = &lw;
		}
		slice = __calc_delta(slice, se->load.weight, load);
	}
	return slice;
}

static void set_next_buddy(struct sched_entity *se)
{
	if (entity_is_task(se) && unlikely(task_has_idle_policy(task_of(se))))
		return;

	for_each_sched_entity(se) {
		if (SCHED_WARN_ON(!se->on_rq))
			return;
		cfs_rq_of(se)->next = se;
	}
}

static inline struct task_group *css_tg(struct cgroup_subsys_state *css)
{
	return css ? container_of(css, struct task_group, css) : NULL;
}

/*****************************************************************************/
/*                       New Code Section                                    */
/*****************************************************************************/
/*
 * This part of code is new for this kernel, which are mostly helper functions.
 */
static inline bool get_prefer_idle(struct task_struct *p)
{
	return vg[get_vendor_task_struct(p)->group].prefer_idle;
}

static inline bool get_prefer_high_cap(struct task_struct *p)
{
	return vg[get_vendor_task_struct(p)->group].prefer_high_cap;
}

static inline bool get_task_spreading(struct task_struct *p)
{
	return vg[get_vendor_task_struct(p)->group].task_spreading;
}

static inline unsigned int get_task_group_throttle(struct task_struct *p)
{
	return vg[get_vendor_task_struct(p)->group].group_throttle;
}

static inline unsigned int get_group_throttle(struct task_group *tg)
{
	return vg[get_vendor_task_group_struct(tg)->group].group_throttle;
}

#if defined(CONFIG_UCLAMP_TASK) && defined(CONFIG_FAIR_GROUP_SCHED)
static inline unsigned long cpu_util_cfs_group_mod_no_est(struct rq *rq)
{
	struct cfs_rq *cfs_rq, *pos;
	unsigned long util = 0, unclamped_util = 0;
	struct task_group *tg;
	unsigned long scale_cpu = arch_scale_cpu_capacity(rq->cpu);

	// cpu_util_cfs = root_util - subgroup_util_sum + throttled_subgroup_util_sum
	for_each_leaf_cfs_rq_safe(rq, cfs_rq, pos) {
		if (&rq->cfs != cfs_rq) {
			tg = cfs_rq->tg;
			unclamped_util += cfs_rq->avg.util_avg;
			util += min_t(unsigned long, READ_ONCE(cfs_rq->avg.util_avg),
				cap_scale(get_group_throttle(tg), scale_cpu));
		}
	}

	util += max_t(int, READ_ONCE(rq->cfs.avg.util_avg) - unclamped_util, 0);

	return util;
}

unsigned long cpu_util_cfs_group_mod(struct rq *rq)
{
	unsigned long util = cpu_util_cfs_group_mod_no_est(rq);

	if (sched_feat(UTIL_EST)) {
		// TODO: right now the limit of util_est is per task
		// consider to make it per group.
		util = max_t(unsigned long, util,
			     READ_ONCE(rq->cfs.avg.util_est.enqueued));
	}

	return util;
}
#else
#define cpu_util_cfs_group_mod cpu_util_cfs
#endif

unsigned long cpu_util(int cpu)
{
       struct rq *rq = cpu_rq(cpu);

       unsigned long util = cpu_util_cfs_group_mod(rq);

       return min_t(unsigned long, util, capacity_of(cpu));
}

/* Similar to cpu_util_without but only count the task's group util contribution */
static unsigned long group_util_without(int cpu, struct task_struct *p)
{
	struct cfs_rq *cfs_rq;
	unsigned int util;
	unsigned long scale_cpu = arch_scale_cpu_capacity(cpu);

	util = READ_ONCE(task_group(p)->cfs_rq[cpu]->avg.util_avg);

	if (cpu != task_cpu(p) || !READ_ONCE(p->se.avg.last_update_time))
		goto out;

	cfs_rq = &cpu_rq(cpu)->cfs;

	lsub_positive(&util, task_util(p));

out:
	return min_t(unsigned long, util,
				cap_scale(get_group_throttle(task_group(p)), scale_cpu));

}

static unsigned long cpu_util_without(int cpu, struct task_struct *p)
{
	struct cfs_rq *cfs_rq;
	unsigned int util;

	/* Task has no contribution or is new */
	if (cpu != task_cpu(p) || !READ_ONCE(p->se.avg.last_update_time))
		return cpu_util(cpu);

	cfs_rq = &cpu_rq(cpu)->cfs;
	util = cpu_util_cfs_group_mod_no_est(cpu_rq(cpu));

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

struct vendor_group_property *get_vendor_group_property(enum vendor_group group)
{
	SCHED_WARN_ON(group < VG_SYSTEM || group >= VG_MAX);

	return &(vg[group]);
}

bool task_fits_capacity(struct task_struct *p, int cpu)
{
	unsigned long task_util;

	if (cpu >= MAX_CAPACITY_CPU)
		return true;

	task_util = get_task_spreading(p) ? task_util_est(p) : uclamp_task_util(p);
	/* clamp task utilization against its per-cpu group limit */
	task_util = min_t(unsigned long, task_util, cap_scale(get_task_group_throttle(p),
							arch_scale_cpu_capacity(cpu)));

	return capacity_of(cpu) * SCHED_CAPACITY_SCALE > task_util * sched_capacity_margin[cpu];
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
	struct rq *rq = cpu_rq(cpu);
	unsigned long scale_cpu = arch_scale_cpu_capacity(cpu);

	if (task_cpu(p) == cpu && dst_cpu != cpu)
		delta = -task_util(p);
	else if (task_cpu(p) != cpu && dst_cpu == cpu)
		delta = task_util(p);

	// For leaf groups
	for_each_leaf_cfs_rq_safe(rq, cfs_rq, pos) {
		if (&rq->cfs != cfs_rq) {
			unsigned long group_util = 0;
			tg = cfs_rq->tg;

			if (p->se.cfs_rq->tg == tg)
				group_util = max_t(int, READ_ONCE(cfs_rq->avg.util_avg) + delta, 0);
			else
				group_util = READ_ONCE(cfs_rq->avg.util_avg);

			unclamped_util += READ_ONCE(cfs_rq->avg.util_avg);
			util += min_t(unsigned long, group_util,
				cap_scale(get_group_throttle(tg), scale_cpu));
		}
	}

	// For root group
	if (p->se.cfs_rq->tg == rq->cfs.tg)
		util = max_t(long, READ_ONCE(rq->cfs.avg.util_avg) - unclamped_util + util + delta,
				   0);
	else
		util = max_t(long, READ_ONCE(rq->cfs.avg.util_avg) - unclamped_util + util, 0);

	if (sched_feat(UTIL_EST)) {
		util_est = READ_ONCE(cfs_rq->avg.util_est.enqueued);

		if (dst_cpu == cpu)
			util_est += _task_util_est(p);

		util = max_t(unsigned long, util, util_est);
	}

	return min(util, capacity_of(cpu));
}

static inline unsigned long em_cpu_energy_pixel_mod(struct em_perf_domain *pd,
				unsigned long max_util, unsigned long sum_util)
{
	unsigned long freq, scale_cpu;
	struct em_perf_state *ps;
	int i, cpu;

	if (!sum_util)
		return 0;

	cpu = cpumask_first(to_cpumask(pd->cpus));
	scale_cpu = arch_scale_cpu_capacity(cpu);
	ps = &pd->table[pd->nr_perf_states - 1];
	freq = map_util_freq_pixel_mod(max_util, ps->frequency, scale_cpu, cpu);

	for (i = 0; i < pd->nr_perf_states; i++) {
		ps = &pd->table[i];
		if (ps->frequency >= freq)
			break;
	}

	return ps->cost * sum_util / scale_cpu;
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

		energy += em_cpu_energy_pixel_mod(pd->em_pd, max_util, sum_util);
	}

	return energy;
}

/* If a task_group is over its group limit on a particular CPU with margin considered */
static inline bool group_overutilized(int cpu, struct task_group *tg)
{
	unsigned long group_capacity = cap_scale(get_group_throttle(tg),
					arch_scale_cpu_capacity(cpu));
	unsigned long group_util = READ_ONCE(tg->cfs_rq[cpu]->avg.util_avg);
	return cpu_overutilized(group_util, group_capacity, cpu);
}

static void find_best_target(cpumask_t *cpus, struct task_struct *p, int prev_cpu, bool sync_boost,
			     int cpu)
{
	unsigned long target_max_curr_spare_cap = 0;
	unsigned long target_max_spare_cap = 0;
	int best_active_cpu = -1;
	int best_idle_cpu = -1;
	int best_importance_cpu = -1;
	int target_cpu = -1;
	int backup_cpu = -1;
	bool prefer_idle;
	bool prefer_high_cap;
	bool prefer_prev = false;
	int i;
	int start_cpu = -1;
	unsigned int min_exit_lat = UINT_MAX;
	unsigned long best_idle_util = ULONG_MAX;
	unsigned long best_cpu_importance = ULONG_MAX;
	long max_importance_spare_cap = LONG_MIN;
	unsigned long task_importance = uclamp_eff_value(p, UCLAMP_MIN) +
					uclamp_eff_value(p, UCLAMP_MAX);

	prefer_idle = get_prefer_idle(p);
	prefer_high_cap = get_prefer_high_cap(p);

	start_cpu = find_start_cpu(p, prefer_high_cap, sync_boost);

	/* prefer prev cpu */
	if (cpu_active(prev_cpu) && cpu_is_idle(prev_cpu) && task_fits_capacity(p, prev_cpu)
				&& !group_overutilized(cpu, task_group(p))) {
		struct cpuidle_state *idle = idle_get_state(cpu_rq(prev_cpu));
		unsigned int exit_lat = UINT_MAX;

		if (sched_cpu_idle(prev_cpu))
			exit_lat = 0;
		else if (idle)
			exit_lat = idle->exit_latency;

		if (exit_lat <= 1) {
			prefer_prev = true;
			cpumask_set_cpu(prev_cpu, cpus);
			goto out;
		}
	}

	for_each_cpu_wrap(i, p->cpus_ptr, start_cpu) {
		unsigned long capacity_curr;
		unsigned long capacity_orig;
		unsigned long capacity;
		unsigned long group_capacity;
		unsigned long wake_group_util;
		unsigned long wake_util, new_util;
		long spare_cap;
		struct cpuidle_state *idle = NULL;
		int next_cpu;
		unsigned int exit_lat = UINT_MAX;
		bool idle_cpu;
		unsigned long cpu_importance;

		if (i >= CPU_NUM)
			continue;

		capacity_curr = capacity_curr_of(i);
		capacity_orig = capacity_orig_of(i);
		capacity = capacity_of(i);
		group_capacity = cap_scale(get_task_group_throttle(p),
				arch_scale_cpu_capacity(i));
		wake_group_util = group_util_without(i, p);
		idle_cpu = cpu_is_idle(i);
		cpu_importance = READ_ONCE(cpu_rq(i)->uclamp[UCLAMP_MIN].value) +
				 READ_ONCE(cpu_rq(i)->uclamp[UCLAMP_MAX].value);

		next_cpu = cpumask_next(i, p->cpus_ptr);
		if (next_cpu >= CPU_NUM)
			next_cpu = 0;

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
		 * Pre-compute the maximum possible capacity we expect
		 * to have available on this CPU before the task is
		 * enqueued here.
		 * We use the minimal of spare cpu capacity and spare
		 * group capacity.
		 */
		spare_cap = min(capacity - wake_util, group_capacity - wake_group_util);

		trace_sched_cpu_util(i, cpu_util(i), capacity_curr, capacity, wake_util, idle_cpu,
				     cpu_importance, group_capacity, wake_group_util, spare_cap,
					 READ_ONCE(task_group(p)->cfs_rq[i]->avg.util_avg),
					 group_overutilized(i, task_group(p)));

		do {
			if (!(prefer_idle && task_importance > cpu_importance)) {
				break;
			}
			if (best_importance_cpu == -1) {
				best_cpu_importance = cpu_importance;
				max_importance_spare_cap = spare_cap;
				best_importance_cpu = i;
				break;
			}
			if (start_cpu >= HIGH_CAPACITY_CPU &&
			    capacity_orig > capacity_orig_of(best_importance_cpu)) {
				best_cpu_importance = cpu_importance;
				max_importance_spare_cap = spare_cap;
				best_importance_cpu = i;
				break;
			}
			if (capacity_orig < capacity_orig_of(best_importance_cpu)) {
				break;
			}
			/* capacity_orig == capacity_orig_of(best_importance_cpu) for all below */
			if (cpu_importance < best_cpu_importance) {
				best_cpu_importance = cpu_importance;
				max_importance_spare_cap = spare_cap;
				best_importance_cpu = i;
				break;
			}
			if (cpu_importance > best_cpu_importance)
				break;
			/* cpu_importance == best_cpu_importance for all below */
			if (spare_cap > max_importance_spare_cap) {
				max_importance_spare_cap = spare_cap;
				best_importance_cpu = i;
				break;
			}
			if (spare_cap < max_importance_spare_cap) {
				break;
			}
			/* spare_cap == max_importance_spare_cap for all below */
			if (best_importance_cpu != prev_cpu)
				best_importance_cpu = i;
		} while (false);

		/*
		 * If a task does not prefer idle cpu, check if it fits in this cpu.
		 * For tasks that prefer idle cpu, we don't do this check so that if
		 * it starts from high capacity cpu and cound not find a candidate cpu
		 * there, it could loop back to little cores to find candidates.
		 */
		if (!prefer_idle && !task_fits_capacity(p, i))
			goto check;

		/*
		 * Skip the cpu if the cpu is full, however,
		 * if task prefers idle and cpu is idle, we skip this check.
		 */
		if (!(prefer_idle && idle_cpu) && wake_util >= capacity)
			goto check;

		if (idle_cpu)
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
			if (idle_cpu) {
				/*
				 * If capacity are the same, skip CPUs in deeper idle state.
				 * If idle state is the same, pick the least utilized cpu.
				 */
				if (best_idle_cpu != -1 &&
				    capacity_orig == capacity_orig_of(best_idle_cpu)) {

					if (sched_cpu_idle(i))
						exit_lat = 0;
					else if (idle)
						exit_lat = idle->exit_latency;

					if (exit_lat > min_exit_lat ||
					    (exit_lat == min_exit_lat &&
					     (best_idle_cpu == prev_cpu ||
					     (i != prev_cpu && best_idle_util <= new_util))))
						goto check;
				}

				min_exit_lat = exit_lat;
				best_idle_util = new_util;
				best_idle_cpu = i;
				goto check;
			}

			if (best_idle_cpu != -1)
				goto check;

			/*
			 * Case A.2: Target CPU
			 * Favor CPU with max spare capacity.
			 */
			if (spare_cap > target_max_spare_cap) {
				target_max_spare_cap = spare_cap;
				target_cpu = i;
				goto check;
			}
		}

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
		if (idle_cpu) {
			if (best_idle_cpu != -1) {
				/*
				 * If capacity are the same, skip CPUs in deeper idle state.
				 * If idle state is the same, pick the least utilized cpu.
				 */
				if (capacity_orig == capacity_orig_of(best_idle_cpu)) {
					if (sched_cpu_idle(i))
						exit_lat = 0;
					else if (idle)
						exit_lat = idle->exit_latency;

					if (exit_lat > min_exit_lat ||
					    (exit_lat == min_exit_lat &&
					     (best_idle_cpu == prev_cpu ||
					     (i != prev_cpu && best_idle_util <= new_util))))
						goto check;
				/*
				 * If capacity are different, keep the first idle cpu
				 * that starts from start_cpu
				 */
				} else if (capacity_orig_of(best_idle_cpu) >=
					   capacity_orig_of(start_cpu)) {
					goto check;
				}
			}

			min_exit_lat = exit_lat;
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

		/*
		 * Case C.1: Target CPU
		 * Favor CPU with max spare capacity that
		 * will not incur cpu freq increase.
		 */
		if (capacity_curr >=
		    (((new_util + cpu_util_rt(cpu_rq(i))) * sched_capacity_margin[i])
		     >> SCHED_CAPACITY_SHIFT) && spare_cap > target_max_curr_spare_cap &&
		     !group_overutilized(i, task_group(p))) {
			target_max_curr_spare_cap = spare_cap;
			target_cpu = i;
			goto check;
		}

		/*
		 * Case C.2: Best Active CPU
		 * Favor CPUs with maximum spare capacity.
		 */
		if (spare_cap > target_max_spare_cap) {
			target_max_spare_cap = spare_cap;
			best_active_cpu = i;
			goto check;
		}
check:
		if (capacity_orig == capacity_orig_of(next_cpu))
			continue;

		/*
		 *  TODO(b/179454592): we probably want to do a deeper search and compare energy
		 *  arcoss different candidates.
		 */
		if ((prefer_idle && best_idle_cpu != -1) ||
		    (!prefer_idle && target_cpu != -1)) {
			if (start_cpu >= HIGH_CAPACITY_CPU) {
				if (capacity_orig_of(HIGH_CAPACITY_CPU) <= capacity_orig)
					break;
			} else {
				if (capacity_orig_of(next_cpu) > capacity_orig)
					break;
			}
		}
	}

	/*
	 * Whole system is probably full, choose between prev_cpu and current cpu.
	 */
	if (target_cpu == -1 && best_idle_cpu == -1 && best_active_cpu == -1) {
		if (capacity_orig_of(prev_cpu) > capacity_orig_of(cpu) ||
		    (capacity_orig_of(prev_cpu) == capacity_orig_of(cpu) &&
		     cpu_rq(prev_cpu)->nr_running <= cpu_rq(cpu)->nr_running)) {
			backup_cpu = prev_cpu;
		} else {
			backup_cpu = cpu;
		}
			cpumask_set_cpu(backup_cpu, cpus);
			goto out;
	}

	/*
	 * For non latency sensitive tasks, cases B and C in the previous loop,
	 * we pick target CPU first if available, otherwise, compare the energy
	 * of best_idle_cpu and best_active_cpu.
	 *
	 * Policies priorities:
	 *
	 * - prefer_idle tasks:
	 *
	 *   a) best_idle_cpu
	 *   b) best_importance_cpu
	 *   c) target_cpu: which has maximum spare capacity
	 *
	 * - NON prefer_idle tasks:
	 *
	 *   a) target_cpu: which has max spare capacity that will not incur cpu
	 *      freq increase
	 *   b) best_idle_cpu or best_active_cpu: select the one with less energy.
	 */
	if (prefer_idle) {
		if (best_idle_cpu != -1) {
			cpumask_set_cpu(best_idle_cpu, cpus);
			goto out;
		} else if (best_importance_cpu != -1) {
			cpumask_set_cpu(best_importance_cpu, cpus);
			goto out;
		}
	}

	if (target_cpu != -1) {
		cpumask_set_cpu(target_cpu, cpus);
		goto out;
	}

	if (best_idle_cpu != -1)
		cpumask_set_cpu(best_idle_cpu, cpus);

	if (best_active_cpu != -1)
		cpumask_set_cpu(best_active_cpu, cpus);

out:
	trace_sched_find_best_target(p, prefer_idle, prefer_high_cap, prefer_prev, sync_boost,
				     task_util_est(p), start_cpu, best_idle_cpu, best_active_cpu,
				     best_importance_cpu, backup_cpu, target_cpu);
}

static DEFINE_PER_CPU(cpumask_t, energy_cpus);

/*****************************************************************************/
/*                       Modified Code Section                               */
/*****************************************************************************/
/*
 * This part of code is vendor hook functions, which modify or extend the original
 * functions.
 */

/*
 * To avoid export more and more sysctl from GKI kernel,
 * use following setting directly dumped from running Android
 * and also assume the setting is not changed in runtime.
 *
 * sysctl_sched
 * .sysctl_sched_latency                    : 10.000000
 * .sysctl_sched_min_granularity            : 3.000000
 * .sysctl_sched_wakeup_granularity         : 2.000000
 * .sysctl_sched_child_runs_first           : 0
 * .sysctl_sched_features                   : 33477435
 * .sysctl_sched_tunable_scaling            : 0 (none)
 */
#define sysctl_sched_min_granularity 3000000ULL

static u64 __sched_period(unsigned long nr_running)
{
	unsigned int sched_nr_latency = DIV_ROUND_UP(sysctl_sched_latency,
					sysctl_sched_min_granularity);

	if (unlikely(nr_running > sched_nr_latency))
		return nr_running * sysctl_sched_min_granularity;
	else
		return sysctl_sched_latency;
}

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
	if (!pd)
		goto fail;

	sync_entity_load_avg(&p->se);

	/* Pre-select a set of candidate CPUs. */
	candidates = this_cpu_ptr(&energy_cpus);
	cpumask_clear(candidates);

	find_best_target(candidates, p, prev_cpu, sync_boost, cpu);

	/* Bail out if no candidate was found.
	 * Do not go to prev_cpu by going unlock, go to fail fallback path
	 */
	weight = cpumask_weight(candidates);
	if (!weight)
		goto fail;

	/* If there is only one candidate, select it if it is lantency sensitive and cpu is idle,
	 * or it is prev cpu, or it is sync_boost, or prev_cpu is overutilized or
	 * group overutilized.
	 */
	cpu = cpumask_first(candidates);
	if (weight == 1 && ((get_prefer_idle(p) && cpu_is_idle(cpu)) ||
			    (cpu == prev_cpu) || sync_boost ||
			    cpu_overutilized(cpu_util(prev_cpu), capacity_of(prev_cpu),
				prev_cpu) || group_overutilized(prev_cpu, task_group(p)))) {
		best_energy_cpu = cpu;
		goto unlock;
	}

	/* Skip prev_cpu if it is no longer allowed or it is over group budget. */
	if (cpumask_test_cpu(prev_cpu, p->cpus_ptr) &&
			!group_overutilized(prev_cpu, task_group(p)))
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
	trace_sched_find_energy_efficient_cpu(p, sync_wakeup, *new_cpu, best_energy_cpu, prev_cpu,
					      get_vendor_task_struct(p)->group,
					      uclamp_eff_value(p, UCLAMP_MIN),
					      uclamp_eff_value(p, UCLAMP_MAX));
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
	*overutilized = cpu_overutilized(cpu_util(cpu), capacity_of(cpu), cpu);
}

unsigned long map_util_freq_pixel_mod(unsigned long util, unsigned long freq,
				      unsigned long cap, int cpu)
{
	return (freq * sched_capacity_margin[cpu] >> SCHED_CAPACITY_SHIFT) * util / cap;
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
	struct vendor_task_struct *vp = get_vendor_task_struct(p);


#ifdef CONFIG_UCLAMP_TASK_GROUP
	struct uclamp_se uc_max;
	struct uclamp_se uc_vnd;

	// Task group restriction
	uc_max = task_group(p)->uclamp[clamp_id];
	// Vendor group restriction
	uc_vnd = vg[vp->group].uc_req[clamp_id];

	if ((clamp_id == UCLAMP_MAX && uc_max.value > uc_vnd.value) ||
	    (clamp_id == UCLAMP_MIN && uc_max.value < uc_vnd.value))
		uc_max = uc_vnd;

	if ((clamp_id == UCLAMP_MAX && (uc_req.value > uc_max.value || !uc_req.user_defined)) ||
	    (clamp_id == UCLAMP_MIN && (uc_req.value < uc_max.value || !uc_req.user_defined)))
		uc_req = uc_max;
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

void initialize_vendor_group_property(void)
{
	int i;
	unsigned int min_val = 0;
	unsigned int max_val = SCHED_CAPACITY_SCALE;

	for (i = 0; i < VG_MAX; i++) {
		vg[i].prefer_idle = false;
		vg[i].prefer_high_cap = false;
		vg[i].task_spreading = false;
		vg[i].group_throttle = max_val;
		vg[i].uc_req[UCLAMP_MIN].value = min_val;
		vg[i].uc_req[UCLAMP_MIN].bucket_id = get_bucket_id(min_val);
		vg[i].uc_req[UCLAMP_MIN].user_defined = false;
		vg[i].uc_req[UCLAMP_MAX].value = max_val;
		vg[i].uc_req[UCLAMP_MAX].bucket_id = get_bucket_id(max_val);
		vg[i].uc_req[UCLAMP_MAX].user_defined = false;
	}
}

void rvh_check_preempt_wakeup_pixel_mod(void *data, struct rq *rq, struct task_struct *p,
			bool *preempt, bool *nopreempt, int wake_flags, struct sched_entity *se,
			struct sched_entity *pse, int next_buddy_marked, unsigned int granularity)
{
	unsigned long ideal_runtime, delta_exec;

	if (entity_is_task(pse) || entity_is_task(se))
		return;

	ideal_runtime = sched_slice(cfs_rq_of(se), se);
	delta_exec = se->sum_exec_runtime - se->prev_sum_exec_runtime;
	/*
	 * If the current group has run enough time for its slice and the new
	 * group has bigger weight, go ahead and preempt.
	 */
	if (ideal_runtime <= delta_exec && se->load.weight < pse->load.weight) {
		if (!next_buddy_marked)
			set_next_buddy(pse);

		*preempt = true;
	}

}

void rvh_util_est_update_pixel_mod(void *data, struct cfs_rq *cfs_rq, struct task_struct *p,
				    bool task_sleep, int *ret)
{
	long last_ewma_diff;
	struct util_est ue;
	int cpu;
	unsigned long scale_cpu;

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
	ue.enqueued = task_util(p);

	cpu = cpu_of(rq_of(cfs_rq));
	scale_cpu = arch_scale_cpu_capacity(cpu);
	// TODO: make util_est to sub cfs-rq and aggregate.
#ifdef CONFIG_UCLAMP_TASK
	// Currently util_est is done only in the root group
	// Current solution apply the clamp in the per-task level for simplicity.
	// However it may
	// 1) over grow by the group limit
	// 2) out of sync when task migrated between cgroups (cfs_rq)
	ue.enqueued = min((unsigned long)ue.enqueued, uclamp_eff_value(p, UCLAMP_MAX));
	ue.enqueued = min_t(unsigned long, ue.enqueued,
			cap_scale(get_group_throttle(task_group(p)), scale_cpu));
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
	ue.ewma = min((unsigned long)ue.ewma, uclamp_eff_value(p, UCLAMP_MAX));
	ue.ewma = min_t(unsigned long, ue.ewma,
			cap_scale(get_group_throttle(task_group(p)), scale_cpu));
#endif
done:
	ue.enqueued |= UTIL_AVG_UNCHANGED;
	WRITE_ONCE(p->se.avg.util_est, ue);

	trace_sched_util_est_se_tp(&p->se);
}

void rvh_post_init_entity_util_avg_pixel_mod(void *data, struct sched_entity *se)
{
	struct cfs_rq *cfs_rq = cfs_rq_of(se);
	struct sched_avg *sa = &se->avg;
	long cpu_scale = arch_scale_cpu_capacity(cpu_of(rq_of(cfs_rq)));

	if (cfs_rq->avg.util_avg == 0) {
		sa->util_avg = vendor_sched_util_post_init_scale * cpu_scale / SCHED_CAPACITY_SCALE;
		sa->runnable_avg = sa->util_avg;
	}
}

void rvh_cpu_cgroup_online_pixel_mod(void *data, struct cgroup_subsys_state *css)
{
	struct vendor_task_group_struct *vtg;
	const char *name = css_tg(css)->css.cgroup->kn->name;

	vtg = get_vendor_task_group_struct(css_tg(css));

	if (strcmp(name, "system") == 0) {
		vtg->group = VG_SYSTEM;
	} else if (strcmp(name, "top-app") == 0) {
		vtg->group = VG_TOPAPP;
	} else if (strcmp(name, "foreground") == 0) {
		vtg->group = VG_FOREGROUND;
	} else if (strcmp(name, "camera-daemon") == 0) {
		vtg->group = VG_CAMERA;
	} else if (strcmp(name, "background") == 0) {
		vtg->group = VG_BACKGROUND;
	} else if (strcmp(name, "system-background") == 0) {
		vtg->group = VG_SYSTEM_BACKGROUND;
	} else if (strcmp(name, "nnapi-hal") == 0) {
		vtg->group = VG_NNAPI_HAL;
	} else if (strcmp(name, "rt") == 0) {
		vtg->group = VG_RT;
	} else if (strcmp(name, "dex2oat") == 0) {
		vtg->group = VG_DEX2OAT;
	} else {
		vtg->group = VG_SYSTEM;
	}
}

void vh_sched_setscheduler_uclamp_pixel_mod(void *data, struct task_struct *tsk, int clamp_id,
					    unsigned int value)
{
	trace_sched_setscheduler_uclamp(tsk, clamp_id, value);
	__ATRACE_INT_PID(tsk->pid, clamp_id  == UCLAMP_MIN ? "UCLAMP_MIN" : "UCLAMP_MAX", value);
}

void vh_dup_task_struct_pixel_mod(void *data, struct task_struct *tsk, struct task_struct *orig)
{
	struct vendor_task_struct *v_tsk, *v_orig;

	v_tsk = get_vendor_task_struct(tsk);
	v_orig = get_vendor_task_struct(orig);
	v_tsk->group = v_orig->group;
}