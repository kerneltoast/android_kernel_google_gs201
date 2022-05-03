// SPDX-License-Identifier: GPL-2.0-only
/* fair.c
 *
 * Android Vendor Hook Support
 *
 * Copyright 2020 Google LLC
 */
#include <kernel/sched/sched.h>
#include <kernel/sched/pelt.h>
#include <trace/events/power.h>

#include "sched_priv.h"
#include "sched_events.h"
#include "../systrace.h"

#if IS_ENABLED(CONFIG_UCLAMP_STATS)
extern void update_uclamp_stats(int cpu, u64 time);
#endif

#if IS_ENABLED(CONFIG_PIXEL_EM)
struct em_perf_domain **vendor_sched_cpu_to_em_pd;
EXPORT_SYMBOL_GPL(vendor_sched_cpu_to_em_pd);
#endif

extern unsigned int vendor_sched_uclamp_threshold;
extern unsigned int vendor_sched_util_post_init_scale;
extern bool vendor_sched_npi_packing;

extern ___update_load_sum(u64 now, struct sched_avg *sa,
			  unsigned long load, unsigned long runnable, int running);
extern ___update_load_avg(struct sched_avg *sa, unsigned long load);

static struct vendor_group_property vg[VG_MAX];

unsigned int sched_capacity_margin[CPU_NUM] = { [0 ... CPU_NUM - 1] = DEF_UTIL_THRESHOLD };
static unsigned long scale_freq[CPU_NUM] = { [0 ... CPU_NUM - 1] = SCHED_CAPACITY_SCALE };

static struct vendor_cfs_util vendor_cfs_util[VG_MAX][CPU_NUM];

unsigned long schedutil_cpu_util_pixel_mod(int cpu, unsigned long util_cfs,
				 unsigned long max, enum schedutil_type type,
				 struct task_struct *p);
unsigned int map_scaling_freq(int cpu, unsigned int freq);

/*****************************************************************************/
/*                       Upstream Code Section                               */
/*****************************************************************************/
/*
 * This part of code is copied from Android common GKI kernel and unmodified.
 * Any change for these functions in upstream GKI would require extensive review
 * to make proper adjustment in vendor hook.
 */
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


static inline unsigned long cfs_rq_load_avg(struct cfs_rq *cfs_rq)
{
	return cfs_rq->avg.load_avg;
}

/*****************************************************************************/
/*                       New Code Section                                    */
/*****************************************************************************/
/*
 * This part of code is new for this kernel, which are mostly helper functions.
 */
static inline int get_vendor_group(struct task_struct *p)
{
	return get_vendor_task_struct(p)->group;
}

static inline bool get_prefer_idle(struct task_struct *p)
{
	// For group based prefer_idle vote, filter our smaller or low prio or
	// have throttled uclamp.max settings
	// Ignore all checks, if the prefer_idle is from per-task API.
	return (vg[get_vendor_group(p)].prefer_idle &&
		task_util_est(p) >= vendor_sched_uclamp_threshold &&
		p->prio <= DEFAULT_PRIO &&
		uclamp_eff_value(p, UCLAMP_MAX) == SCHED_CAPACITY_SCALE) ||
			get_vendor_task_struct(p)->prefer_idle;
}

bool get_prefer_high_cap(struct task_struct *p)
{
	return vg[get_vendor_group(p)].prefer_high_cap;
}

static inline bool get_task_spreading(struct task_struct *p)
{
	return vg[get_vendor_group(p)].task_spreading;
}

static inline unsigned int get_task_group_throttle(struct task_struct *p)
{
	return vg[get_vendor_group(p)].group_throttle;
}

void init_vendor_group_util(void)
{
	int i, j;
	struct task_struct *p;
	struct rq *rq;
	int group;
	struct rq_flags rf;

	for (j = 0; j < CPU_NUM; j++) {
		rq = cpu_rq(j);

		for (i = 0; i < VG_MAX; i++) {
			raw_spin_lock_init(&vendor_cfs_util[i][j].lock);
			vendor_cfs_util[i][j].avg.util_avg = 0;
			vendor_cfs_util[i][j].avg.util_sum = 0;
			vendor_cfs_util[i][j].avg.last_update_time = rq_clock_pelt(cpu_rq(j));
			vendor_cfs_util[i][j].util_removed = 0;
			vendor_cfs_util[i][j].util_est = 0;
		}

		rq_lock(rq, &rf);
		list_for_each_entry(p, &rq->cfs_tasks, se.group_node) {
			group = get_vendor_group(p);
			vendor_cfs_util[group][j].avg.util_avg += READ_ONCE(p->se.avg.util_avg);
			vendor_cfs_util[group][j].avg.util_sum += READ_ONCE(p->se.avg.util_sum);
			vendor_cfs_util[group][rq->cpu].util_est += _task_util_est(p);
		}
		rq_unlock(rq, &rf);
	}
}


// This function is called when tasks migrate among vendor groups
void migrate_vendor_group_util(struct task_struct *p, unsigned int old, unsigned int new)
{
	int cpu = task_cpu(p);
	unsigned long flags;
	unsigned long util_avg, util_sum;
	unsigned int util_est;

	//remove util from old group
	raw_spin_lock_irqsave(&vendor_cfs_util[old][cpu].lock, flags);
	util_avg = min(vendor_cfs_util[old][cpu].avg.util_avg, p->se.avg.util_avg);
	util_sum = min(vendor_cfs_util[old][cpu].avg.util_sum, p->se.avg.util_sum);
	util_est = min_t(unsigned int, vendor_cfs_util[old][cpu].util_est, _task_util_est(p));
	vendor_cfs_util[old][cpu].avg.util_avg -= util_avg;
	vendor_cfs_util[old][cpu].avg.util_sum -= util_sum;
	if (p->on_rq)
		vendor_cfs_util[old][cpu].util_est -= util_est;
	raw_spin_unlock_irqrestore(&vendor_cfs_util[old][cpu].lock, flags);

	//move util to new group
	raw_spin_lock_irqsave(&vendor_cfs_util[new][cpu].lock, flags);
	vendor_cfs_util[new][cpu].avg.util_avg += util_avg;
	vendor_cfs_util[new][cpu].avg.util_sum += util_sum;
	if (p->on_rq)
		vendor_cfs_util[new][cpu].util_est += util_est;
	raw_spin_unlock_irqrestore(&vendor_cfs_util[new][cpu].lock, flags);
}

// This function hooks to update_load_avg
static inline void update_vendor_group_util(u64 now, struct cfs_rq *cfs_rq,
					    struct sched_entity *curr)
{
	int i, group = -1;
	struct sched_avg *sa;
	unsigned long removed_util;
	u32 divider;

	if (curr && entity_is_task(curr))
		group = get_vendor_group(task_of(curr));

	for (i = 0; i < VG_MAX; i++) {
		sa = &vendor_cfs_util[i][cfs_rq->rq->cpu].avg;
		if (vendor_cfs_util[i][cfs_rq->rq->cpu].util_removed) {
			removed_util = 0;
			divider = get_pelt_divider(sa);
			raw_spin_lock(&vendor_cfs_util[i][cfs_rq->rq->cpu].lock);
			swap(vendor_cfs_util[i][cfs_rq->rq->cpu].util_removed, removed_util);
			sub_positive(&sa->util_avg, removed_util);
			sub_positive(&sa->util_sum, removed_util * divider);
			sa->util_sum = max_t(unsigned long, sa->util_sum,
					     sa->util_avg * PELT_MIN_DIVIDER);
			raw_spin_unlock(&vendor_cfs_util[i][cfs_rq->rq->cpu].lock);
		}

		if (sa->util_sum != 0 || i == group) {
			raw_spin_lock(&vendor_cfs_util[i][cfs_rq->rq->cpu].lock);
			___update_load_sum(now, sa, i == group, 0, i == group);
			___update_load_avg(sa, 1);
			raw_spin_unlock(&vendor_cfs_util[i][cfs_rq->rq->cpu].lock);
		// if util_sum is 0 and not updating the group, only need to record last_update_time
		} else {
			u64 delta;

			delta = now - sa->last_update_time;

			if ((s64)delta < 0) {
				sa->last_update_time = now;
			} else {
				delta >>= 10;
				if (delta)
					sa->last_update_time += delta << 10;
			}
		}
	}
}

// This function hooks to attach_entity_load_avg
static inline void attach_vendor_group_util(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	int group = get_vendor_group(task_of(se));

	raw_spin_lock(&vendor_cfs_util[group][cfs_rq->rq->cpu].lock);
	vendor_cfs_util[group][cfs_rq->rq->cpu].avg.util_avg += se->avg.util_avg;
	vendor_cfs_util[group][cfs_rq->rq->cpu].avg.util_sum += se->avg.util_sum;
	raw_spin_unlock(&vendor_cfs_util[group][cfs_rq->rq->cpu].lock);
}

// This function hooks to detach_entity_load_avg
static inline void detach_vendor_group_util(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	int group = get_vendor_group(task_of(se));

	raw_spin_lock(&vendor_cfs_util[group][cfs_rq->rq->cpu].lock);
	sub_positive(&vendor_cfs_util[group][cfs_rq->rq->cpu].avg.util_avg, se->avg.util_avg);
	sub_positive(&vendor_cfs_util[group][cfs_rq->rq->cpu].avg.util_sum, se->avg.util_sum);
	vendor_cfs_util[group][cfs_rq->rq->cpu].avg.util_sum = max_t(unsigned long,
		vendor_cfs_util[group][cfs_rq->rq->cpu].avg.util_sum,
		vendor_cfs_util[group][cfs_rq->rq->cpu].avg.util_avg * PELT_MIN_DIVIDER);
	raw_spin_unlock(&vendor_cfs_util[group][cfs_rq->rq->cpu].lock);
}

// This function hooks to remove_entity_load_avg. It is called when tasks migrate to another cpu rq,
// and we need to keep the removed util, which will be deduced from the prev rq in the next update
// call on that rq.
static inline void remove_vendor_group_util(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	int group = get_vendor_group(task_of(se));
	unsigned long flags;

	raw_spin_lock_irqsave(&vendor_cfs_util[group][cfs_rq->rq->cpu].lock, flags);
	vendor_cfs_util[group][cfs_rq->rq->cpu].util_removed += se->avg.util_avg;
	raw_spin_unlock_irqrestore(&vendor_cfs_util[group][cfs_rq->rq->cpu].lock, flags);
}

static inline unsigned long get_vendor_util_est(struct rq *rq, bool with, struct task_struct *p)
{
	int i, group;
	unsigned long group_util, util_est = 0;
	unsigned long scale_cpu;

	group = get_vendor_group(p);
	scale_cpu = arch_scale_cpu_capacity(rq->cpu);

	for (i = 0; i < VG_MAX; i++) {
		group_util = vendor_cfs_util[i][rq->cpu].util_est;

		if (group == i && p) {
			if (with) {
				group_util += _task_util_est(p);
			} else {
				lsub_positive(&group_util, _task_util_est(p));
			}
		}

		group_util = min_t(unsigned long, group_util,
				   cap_scale(vg[i].group_throttle, scale_cpu));
		group_util = min_t(unsigned long, group_util, vg[i].uc_req[UCLAMP_MAX].value);

		util_est += group_util;
	}

	return util_est;
}

static inline unsigned long cpu_util_cfs_group_mod_est(struct rq *rq, bool with,
						       struct task_struct *p)
{
	int i, group;
	unsigned long group_util, util = 0;
	unsigned long group_util_est, util_est = 0;
	unsigned long scale_cpu;

	group = get_vendor_group(p);
	scale_cpu = arch_scale_cpu_capacity(rq->cpu);

	for (i = 0; i < VG_MAX; i++) {
		group_util = vendor_cfs_util[i][rq->cpu].avg.util_avg;
		group_util_est = vendor_cfs_util[i][rq->cpu].util_est;

		if (group == i && p) {
			if (with) {
				group_util += task_util(p);
				group_util_est += _task_util_est(p);
			} else {
				lsub_positive(&group_util, task_util(p));
				lsub_positive(&group_util_est, _task_util_est(p));
			}
		}


		group_util = min_t(unsigned long, group_util,
				   cap_scale(vg[i].group_throttle, scale_cpu));
		group_util = min_t(unsigned long, group_util, vg[i].uc_req[UCLAMP_MAX].value);

		group_util_est = min_t(unsigned long, group_util_est,
				       cap_scale(vg[i].group_throttle, scale_cpu));
		group_util_est = min_t(unsigned long, group_util_est,
				       vg[i].uc_req[UCLAMP_MAX].value);

		util += group_util;
		util_est += group_util_est;
	}

	return max(util, util_est);
}

static inline unsigned long cpu_util_cfs_group_mod_no_est(struct rq *rq, bool with,
							  struct task_struct *p)
{
	int i, group;
	unsigned long group_util, util = 0;
	unsigned long scale_cpu;

	group = get_vendor_group(p);
	scale_cpu = arch_scale_cpu_capacity(rq->cpu);

	for (i = 0; i < VG_MAX; i++) {
		group_util = vendor_cfs_util[i][rq->cpu].avg.util_avg;

		if (group == i && p) {
			if (with) {
				group_util += task_util(p);
			} else {
				lsub_positive(&group_util, task_util(p));
			}
		}

		group_util = min_t(unsigned long, group_util,
				   cap_scale(vg[i].group_throttle, scale_cpu));
		group_util = min_t(unsigned long, group_util, vg[i].uc_req[UCLAMP_MAX].value);

		util += group_util;
	}

	return util;
}

unsigned long cpu_util_cfs_group_mod(struct rq *rq)
{
	if (sched_feat(UTIL_EST)) {
		return cpu_util_cfs_group_mod_est(rq, false, NULL);
	} else {
		return cpu_util_cfs_group_mod_no_est(rq, false, NULL);
	}
}

unsigned long cpu_util(int cpu)
{
       struct rq *rq = cpu_rq(cpu);

       unsigned long util = cpu_util_cfs_group_mod(rq);

       return min_t(unsigned long, util, capacity_of(cpu));
}

/* Similar to cpu_util_without but only count the task's group util contribution */
static unsigned long group_util(int cpu, struct task_struct *p, unsigned long max, bool without)
{
	unsigned long util = vendor_cfs_util[get_vendor_group(p)][cpu].avg.util_avg;
	bool subtract = (cpu == task_cpu(p) && READ_ONCE(p->se.avg.last_update_time));

	if (without && subtract)
		lsub_positive(&util, task_util(p));

	if (sched_feat(UTIL_EST)) {
		unsigned long estimated = vendor_cfs_util[get_vendor_group(p)][cpu].util_est;

		if (without && (subtract && unlikely(task_on_rq_queued(p) || current == p)))
			lsub_positive(&estimated, _task_util_est(p));

		util = max(util, estimated);
	}

	return min(util, max);

}

static unsigned long cpu_util_without(int cpu, struct task_struct *p, unsigned long max)
{
	int group;
	unsigned long util, scale_cpu;

	/* Task has no contribution or is new */
	if (cpu != task_cpu(p) || !READ_ONCE(p->se.avg.last_update_time))
		return cpu_util(cpu);

	group = get_vendor_group(p);
	scale_cpu = arch_scale_cpu_capacity(cpu);
	util = cpu_util_cfs_group_mod_no_est(cpu_rq(cpu), false, p);

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
		unsigned int estimated;

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
		if (unlikely(task_on_rq_queued(p) || current == p)) {
			estimated = get_vendor_util_est(cpu_rq(cpu), false, p);
		} else {
			estimated = get_vendor_util_est(cpu_rq(cpu), false, NULL);
		}

		util = max_t(unsigned long, util, estimated);
	}

	/*
	 * Utilization (estimated) can exceed the CPU capacity, thus let's
	 * clamp to the maximum CPU capacity to ensure consistency with
	 * the cpu_util call.
	 */
	return min(util, max);
}

struct vendor_group_property *get_vendor_group_property(enum vendor_group group)
{
	SCHED_WARN_ON(group < VG_SYSTEM || group >= VG_MAX);

	return &(vg[group]);
}

static bool task_fits_capacity(struct task_struct *p, int cpu,  bool sync_boost)
{
	unsigned long task_util;

	if (cpu >= MAX_CAPACITY_CPU)
		return true;

	if ((get_prefer_high_cap(p) || sync_boost) && cpu < MID_CAPACITY_CPU)
		return false;

	task_util = get_task_spreading(p) ? task_util_est(p) : uclamp_task_util(p);
	/* clamp task utilization against its per-cpu group limit */
	task_util = min_t(unsigned long, task_util, cap_scale(get_task_group_throttle(p),
							arch_scale_cpu_capacity(cpu)));

	return capacity_of(cpu) * SCHED_CAPACITY_SCALE > task_util * sched_capacity_margin[cpu];
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
	unsigned long util;

	if (task_cpu(p) == cpu && dst_cpu != cpu)
		cpu_util_cfs_group_mod_no_est(cpu_rq(cpu), false, p);
	else if (task_cpu(p) != cpu && dst_cpu == cpu)
		util = cpu_util_cfs_group_mod_no_est(cpu_rq(cpu), true, p);
	else
		util = cpu_util_cfs_group_mod_no_est(cpu_rq(cpu), false, NULL);

	if (sched_feat(UTIL_EST)) {
		unsigned long estimated;

		if (dst_cpu == cpu) {
			estimated = get_vendor_util_est(cpu_rq(cpu), true, p);
		} else {
			estimated = get_vendor_util_est(cpu_rq(cpu), false, NULL);
		}

		util = max(util, estimated);
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

#if IS_ENABLED(CONFIG_PIXEL_EM)
	{
		struct em_perf_domain **cpu_to_em_pd = READ_ONCE(vendor_sched_cpu_to_em_pd);
		if (cpu_to_em_pd)
			pd = cpu_to_em_pd[cpu];
	}
#endif

	scale_cpu = arch_scale_cpu_capacity(cpu);
	ps = &pd->table[pd->nr_perf_states - 1];
	freq = map_util_freq_pixel_mod(max_util, ps->frequency, scale_cpu, cpu);
	freq = map_scaling_freq(cpu, freq);

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
	unsigned long max_util, util_cfs, cpu_util, cpu_cap;
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
static inline bool group_overutilized(int cpu, struct task_struct *p)
{
	unsigned long group_capacity = cap_scale(get_task_group_throttle(p),
						 arch_scale_cpu_capacity(cpu));
	unsigned long util = group_util(cpu, p, group_capacity, false);
	return cpu_overutilized(util, group_capacity, cpu);
}

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

static int find_energy_efficient_cpu(struct task_struct *p, int prev_cpu, bool sync_boost)
{
	struct root_domain *rd;
	struct perf_domain *pd;
	cpumask_t idle_fit = { CPU_BITS_NONE }, idle_unfit = { CPU_BITS_NONE },
		  unimportant_fit = { CPU_BITS_NONE }, unimportant_unfit = { CPU_BITS_NONE },
		  max_spare_cap = { CPU_BITS_NONE }, packing = { CPU_BITS_NONE },
		  candidates = { CPU_BITS_NONE };
	int i, weight, best_energy_cpu = -1, this_cpu = smp_processor_id();
	long cur_energy, best_energy = LONG_MAX;
	unsigned long spare_cap, target_max_spare_cap = 0;
	unsigned long task_importance =
			((p->prio <= DEFAULT_PRIO) ? uclamp_eff_value(p, UCLAMP_MIN) : 0) +
			uclamp_eff_value(p, UCLAMP_MAX);
	unsigned int exit_lat, pd_best_exit_lat, best_exit_lat;
	bool is_idle, group_overutilize, task_fits;
	bool idle_target_found = false, importance_target_found = false;
	bool prefer_idle = get_prefer_idle(p), prefer_high_cap = get_prefer_high_cap(p);
	unsigned long capacity, wake_util, group_capacity, wake_group_util, cpu_importance;
	unsigned long pd_max_spare_cap, pd_max_unimportant_spare_cap, pd_max_packing_spare_cap;
	int pd_max_spare_cap_cpu, pd_best_idle_cpu, pd_most_unimportant_cpu, pd_best_packing_cpu;
	struct cpuidle_state *idle_state;
	unsigned long util;
	unsigned long least_load = ULONG_MAX;
	int least_loaded_cpu = -1;
	bool prefer_fit = prefer_idle && get_vendor_task_struct(p)->uclamp_fork_reset;

	rd = cpu_rq(this_cpu)->rd;

	rcu_read_lock();
	pd = rcu_dereference(rd->pd);
	if (!pd)
		goto out;

	for (; pd; pd = pd->next) {
		pd_max_spare_cap = 0;
		pd_max_packing_spare_cap = 0;
		pd_max_unimportant_spare_cap = 0;
		pd_best_exit_lat = UINT_MAX;
		pd_max_spare_cap_cpu = -1;
		pd_best_idle_cpu = -1;
		pd_most_unimportant_cpu = -1;
		pd_best_packing_cpu = -1;

		for_each_cpu_and(i, perf_domain_span(pd), p->cpus_ptr) {
			if (i >= CPU_NUM)
				break;

			if (!cpu_active(i))
				continue;

			capacity = capacity_of(i);
			is_idle = cpu_is_idle(i);
			cpu_importance = READ_ONCE(cpu_rq(i)->uclamp[UCLAMP_MIN].value) +
					   READ_ONCE(cpu_rq(i)->uclamp[UCLAMP_MAX].value);
			wake_util = cpu_util_without(i, p, capacity);
			group_capacity = cap_scale(get_task_group_throttle(p),
					   arch_scale_cpu_capacity(i));
			wake_group_util = group_util(i, p, group_capacity, true);
			task_fits = task_fits_capacity(p, i, sync_boost);
			spare_cap = min_t(unsigned long, capacity - wake_util,
					  group_capacity - wake_group_util);
			group_overutilize = group_overutilized(i, p);
			exit_lat = 0;
			util = cpu_util(i);

			if (is_idle) {
				idle_state = idle_get_state(cpu_rq(i));
				if (idle_state)
					exit_lat = idle_state->exit_latency;
			}

			trace_sched_cpu_util_cfs(i, is_idle, exit_lat, cpu_importance, util,
						 capacity, wake_util, group_capacity,
						 wake_group_util, spare_cap, task_fits,
						 group_overutilize);

			if (prefer_fit && !task_fits)
				continue;

			if (prefer_idle) {
				/*
				 * For a cluster, the energy computation result will be the same for
				 * idle cpus on that cluster, so we could save some computation by
				 * just choosing 1 idle cpu for each cluster.
				 * If there are multiple idle cpus, compare their c states (exit
				 * latency).
				 */
				if (is_idle) {
					/* The first idle cpu will always fall into this case. */
					if (exit_lat < pd_best_exit_lat) {
						pd_best_idle_cpu = i;
						pd_best_exit_lat = exit_lat;
					} else if (exit_lat == pd_best_exit_lat) {
						/*
						 * A simple randomization, by choosing the first or
						 * the last cpu if pd_best_idle_cpu != prev_cpu.
						 */
						if (i == prev_cpu ||
						    (pd_best_idle_cpu != prev_cpu && this_cpu % 2))
							pd_best_idle_cpu = i;
					}

					idle_target_found = true;
				}

				if (idle_target_found)
					continue;

				/* Find an unimportant cpu with the max spare capacity. */
				if (task_importance > cpu_importance &&
				    spare_cap >= pd_max_unimportant_spare_cap) {
					pd_max_unimportant_spare_cap = spare_cap;
					pd_most_unimportant_cpu = i;
					importance_target_found = true;
				}

				if (importance_target_found)
					continue;

				if (prefer_high_cap && i < HIGH_CAPACITY_CPU)
					continue;

				// Used when no candidate is found.
				if (cfs_rq_load_avg(&cpu_rq(i)->cfs) <= least_load) {
					least_load = cfs_rq_load_avg(&cpu_rq(i)->cfs);
					least_loaded_cpu = i;
				}

				if (group_overutilize || cpu_overutilized(util, capacity, i))
					continue;

				/* find max spare capacity cpu, used as backup */
				if (spare_cap > target_max_spare_cap) {
					target_max_spare_cap = spare_cap;
					cpumask_clear(&max_spare_cap);
					cpumask_set_cpu(i, &max_spare_cap);
				} else if (spare_cap == target_max_spare_cap) {
					cpumask_set_cpu(i, &max_spare_cap);
				}
			} else {/* Below path is for non-prefer idle case*/
				// Used when no candidate is found.
				if (cfs_rq_load_avg(&cpu_rq(i)->cfs) <= least_load) {
					least_load = cfs_rq_load_avg(&cpu_rq(i)->cfs);
					least_loaded_cpu = i;
				}

				if (!task_fits || group_overutilize ||
				    cpu_overutilized(util, capacity, i))
					continue;

				if (spare_cap < min_t(unsigned long, task_util_est(p),
				    cap_scale(get_task_group_throttle(p),
					      arch_scale_cpu_capacity(i))))
					continue;

				/*
				 * Find the best packing CPU with the maximum spare capacity in
				 * the performance domain
				 */
				if (vendor_sched_npi_packing && !available_idle_cpu(i) &&
				    cpu_importance <= DEFAULT_IMPRATANCE_THRESHOLD &&
				    spare_cap > pd_max_packing_spare_cap && capacity_curr_of(i) >=
				    ((cpu_util_next(i, p, i) + cpu_util_rt(cpu_rq(i))) *
				    sched_capacity_margin[i]) >> SCHED_CAPACITY_SHIFT) {
					pd_max_packing_spare_cap = spare_cap;
					pd_best_packing_cpu = i;
				}

				if (pd_best_packing_cpu != -1)
					continue;

				/*
				 * Find the CPU with the maximum spare capacity in
				 * the performance domain
				 */
				if (spare_cap > pd_max_spare_cap) {
					pd_max_spare_cap = spare_cap;
					pd_max_spare_cap_cpu = i;
					pd_best_exit_lat = exit_lat;
				/* Candidates could be idle cpu, so compare their exit lat. */
				} else if (spare_cap == pd_max_spare_cap) {
					if (exit_lat < pd_best_exit_lat) {
						pd_max_spare_cap_cpu = i;
						pd_best_exit_lat = exit_lat;
					/* If exit lat is the same, compare their loading */
					} else if (exit_lat == pd_best_exit_lat) {
						if (cfs_rq_load_avg(&cpu_rq(i)->cfs) <
						    cfs_rq_load_avg(
						    &cpu_rq(pd_max_spare_cap_cpu)->cfs)) {
							pd_max_spare_cap_cpu = i;
						/*
						 * If loading is the same, use a simple
						 * randomization by choosing the first or the last
						 * cpu if pd_max_spare_cap_cpu != prev_cpu.
						 */
						} else if (i == prev_cpu ||
						    (pd_max_spare_cap_cpu != prev_cpu &&
						      this_cpu % 2)) {
							pd_max_spare_cap_cpu = i;
						}
					}
				}
			}
		}

		/* set the best_idle_cpu of each cluster */
		if (pd_best_idle_cpu != -1) {
			if (task_fits) {
				cpumask_set_cpu(pd_best_idle_cpu, &idle_fit);
			} else {
				cpumask_set_cpu(pd_best_idle_cpu, &idle_unfit);
			}
		}

		/* set the best_important_cpu of each cluster */
		if (pd_most_unimportant_cpu != -1) {
			if (task_fits) {
				cpumask_set_cpu(pd_most_unimportant_cpu, &unimportant_fit);
			} else {
				cpumask_set_cpu(pd_most_unimportant_cpu, &unimportant_unfit);
			}
		}

		/* set the packing cpu of max_spare_cap of each cluster */
		if (pd_best_packing_cpu != -1)
			cpumask_set_cpu(pd_best_packing_cpu, &packing);

		/* set the max_spare_cap_cpu of each cluster */
		if (pd_max_spare_cap_cpu != -1)
			cpumask_set_cpu(pd_max_spare_cap_cpu, &max_spare_cap);
	}

	/* Assign candidates based on search order. */
	if (!cpumask_empty(&idle_fit)) {
		cpumask_copy(&candidates, &idle_fit);
	} else if (!cpumask_empty(&idle_unfit)) {
		cpumask_copy(&candidates, &idle_unfit);
	} else if (!cpumask_empty(&unimportant_fit)) {
		cpumask_copy(&candidates, &unimportant_fit);
	} else if (!cpumask_empty(&unimportant_unfit)) {
		cpumask_copy(&candidates, &unimportant_unfit);
	} else if (!cpumask_empty(&packing)) {
		cpumask_copy(&candidates, &packing);
	} else if (!cpumask_empty(&max_spare_cap)) {
		cpumask_copy(&candidates, &max_spare_cap);
	}

	weight = cpumask_weight(&candidates);
	best_energy_cpu = least_loaded_cpu;

	/* Bail out if no candidate was found. */
	if (weight == 0)
		goto out;

	/* Bail out if only 1 candidate was found. */
	if (weight == 1) {
		best_energy_cpu = cpumask_first(&candidates);
		goto out;
	}

	/* Compute Energy */
	best_exit_lat = UINT_MAX;
	pd = rcu_dereference(rd->pd);
	for_each_cpu(i, &candidates) {
		exit_lat = 0;

		if (cpu_is_idle(i)) {
			idle_state = idle_get_state(cpu_rq(i));
			if (idle_state)
				exit_lat = idle_state->exit_latency;
		}

		cur_energy = compute_energy(p, i, pd);

		if (cur_energy < best_energy) {
			best_energy = cur_energy;
			best_energy_cpu = i;
			best_exit_lat = exit_lat;
		} else if (cur_energy == best_energy) {
			if (exit_lat < best_exit_lat) {
				best_energy_cpu = i;
				best_exit_lat = exit_lat;
			} else if (exit_lat == best_exit_lat) {
				/* Prefer prev cpu or this cpu. */
				if (i == prev_cpu ||
				    (best_energy_cpu != prev_cpu && i == this_cpu)) {
					best_energy_cpu = i;
				}
			}
		}
	}

out:
	rcu_read_unlock();
	trace_sched_find_energy_efficient_cpu(p, task_util_est(p), prefer_idle, prefer_high_cap,
				     task_importance, &idle_fit, &idle_unfit, &unimportant_fit,
				     &unimportant_unfit, &packing, &max_spare_cap, best_energy_cpu);
	return best_energy_cpu;
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
	unsigned int tg_min, tg_max, vnd_min, vnd_max, value;

	// Task group restriction
	tg_min = task_group(p)->uclamp[UCLAMP_MIN].value;
	tg_max = task_group(p)->uclamp[UCLAMP_MAX].value;
	// Vendor group restriction
	vnd_min = vg[vp->group].uc_req[UCLAMP_MIN].value;
	vnd_max = vg[vp->group].uc_req[UCLAMP_MAX].value;

	value = uc_req.value;
	value = clamp(value, max(tg_min, vnd_min),  min(tg_max, vnd_max));

	// For uclamp min, if task has a valid per-task setting that is lower than or equal to its
	// group value, increase the final uclamp value by 1. This would have effect only on
	// importance metrics which is used in task placement, and little effect on cpufreq.
	if (clamp_id == UCLAMP_MIN && uc_req.value <= max(tg_min, vnd_min) && uc_req.user_defined
		&& value < SCHED_CAPACITY_SCALE)
		value = value + 1;

	// For low prio unthrottled task, reduce its uclamp.max by 1 which
	// would affect task importance in cpu_rq thus affect task placement.
	// It should have no effect in cpufreq.
	if (clamp_id == UCLAMP_MAX && p->prio > DEFAULT_PRIO)
		value = min_t(unsigned int, UCLAMP_BUCKET_DELTA * (UCLAMP_BUCKETS - 1) - 1, value);

	uc_req.value = value;
	uc_req.bucket_id = get_bucket_id(value);
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

void vh_sched_setscheduler_uclamp_pixel_mod(void *data, struct task_struct *tsk, int clamp_id,
					    unsigned int value)
{
	trace_sched_setscheduler_uclamp(tsk, clamp_id, value);
	if (trace_clock_set_rate_enabled()) {
		char trace_name[32] = {0};
		scnprintf(trace_name, sizeof(trace_name), "%s_%d",
			clamp_id  == UCLAMP_MIN ? "UCLAMP_MIN" : "UCLAMP_MAX", tsk->pid);
		trace_clock_set_rate(trace_name, value, raw_smp_processor_id());
	}
}

void vh_dup_task_struct_pixel_mod(void *data, struct task_struct *tsk, struct task_struct *orig)
{
	struct vendor_task_struct *v_tsk, *v_orig;

	v_tsk = get_vendor_task_struct(tsk);
	v_orig = get_vendor_task_struct(orig);
	v_tsk->group = v_orig->group;
	v_tsk->prefer_idle = false;
}

void rvh_select_task_rq_fair_pixel_mod(void *data, struct task_struct *p, int prev_cpu, int sd_flag,
				       int wake_flags, int *target_cpu)
{
	int sync = (wake_flags & WF_SYNC) && !(current->flags & PF_EXITING);
	bool sync_wakeup = false, prefer_prev = false, sync_boost = false;
	int cpu;

	/* sync wake up */
	cpu = smp_processor_id();
	if (sync && cpu_rq(cpu)->nr_running == 1 && cpumask_test_cpu(cpu, p->cpus_ptr) &&
	     cpu_is_in_target_set(p, cpu) && task_fits_capacity(p, cpu, false)) {
		*target_cpu = cpu;
		sync_wakeup = true;
		goto out;
	}

	sync_boost = sync && cpu >= HIGH_CAPACITY_CPU;

	/* prefer prev cpu */
	if (cpu_active(prev_cpu) && cpu_is_idle(prev_cpu) && task_fits_capacity(p, prev_cpu,
	     sync_boost) && !group_overutilized(prev_cpu, p)) {
		struct cpuidle_state *idle_state;
		unsigned int exit_lat = UINT_MAX;

		rcu_read_lock();
		idle_state = idle_get_state(cpu_rq(prev_cpu));

		if (sched_cpu_idle(prev_cpu))
			exit_lat = 0;
		else if (idle_state)
			exit_lat = idle_state->exit_latency;

		rcu_read_unlock();

		if (exit_lat <= C1_EXIT_LATENCY) {
			prefer_prev = true;
			*target_cpu = prev_cpu;
			goto out;
		}
	}

	if (sd_flag & SD_BALANCE_WAKE) {
		*target_cpu = find_energy_efficient_cpu(p, prev_cpu, sync_boost);
	}

out:
	trace_sched_select_task_rq_fair(p, task_util_est(p),
					sync_wakeup, prefer_prev, sync_boost,
					get_vendor_group(p),
					uclamp_eff_value(p, UCLAMP_MIN),
					uclamp_eff_value(p, UCLAMP_MAX),
					prev_cpu, *target_cpu);
}

void rvh_attach_entity_load_avg_pixel_mod(void *data, struct cfs_rq *cfs_rq,
					  struct sched_entity *se)
{
	if (se && entity_is_task(se))
		attach_vendor_group_util(cfs_rq, se);
}

void rvh_detach_entity_load_avg_pixel_mod(void *data, struct cfs_rq *cfs_rq,
					  struct sched_entity *se)
{
	if (entity_is_task(se))
		detach_vendor_group_util(cfs_rq, se);
}

void rvh_update_load_avg_pixel_mod(void *data, u64 now, struct cfs_rq *cfs_rq,
				   struct sched_entity *se)
{
	if (entity_is_task(se))
		update_vendor_group_util(now, cfs_rq, cfs_rq->curr);
}

void rvh_remove_entity_load_avg_pixel_mod(void *data, struct cfs_rq *cfs_rq,
					  struct sched_entity *se)
{
	if (entity_is_task(se))
		remove_vendor_group_util(cfs_rq, se);
}

void rvh_update_blocked_fair_pixel_mod(void *data, struct rq *rq)
{
	update_vendor_group_util(rq_clock_pelt(rq), &rq->cfs, NULL);
}

void rvh_enqueue_task_fair_pixel_mod(void *data, struct rq *rq, struct task_struct *p, int flags)
{
	if (sched_feat(UTIL_EST)) {
		int group = get_vendor_group(p);

		raw_spin_lock(&vendor_cfs_util[group][rq->cpu].lock);
		vendor_cfs_util[group][rq->cpu].util_est += _task_util_est(p);
		raw_spin_unlock(&vendor_cfs_util[group][rq->cpu].lock);
	}
}

void rvh_dequeue_task_fair_pixel_mod(void *data, struct rq *rq, struct task_struct *p, int flags)
{
	if (sched_feat(UTIL_EST)) {
		int group = get_vendor_group(p);

		raw_spin_lock(&vendor_cfs_util[group][rq->cpu].lock);
		if (!rq->cfs.h_nr_running)
			vendor_cfs_util[group][rq->cpu].util_est = 0;
		else
			lsub_positive(&vendor_cfs_util[group][rq->cpu].util_est, _task_util_est(p));
		raw_spin_unlock(&vendor_cfs_util[group][rq->cpu].lock);
	}
}
