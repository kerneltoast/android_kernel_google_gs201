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

#if IS_ENABLED(CONFIG_PIXEL_EM)
#include "../../include/pixel_em.h"
struct pixel_em_profile **vendor_sched_pixel_em_profile;
EXPORT_SYMBOL_GPL(vendor_sched_pixel_em_profile);
struct pixel_idle_em *vendor_sched_pixel_idle_em;
EXPORT_SYMBOL_GPL(vendor_sched_pixel_idle_em);
#endif

#if IS_ENABLED(CONFIG_USE_VENDOR_GROUP_UTIL)
extern int ___update_load_sum(u64 now, struct sched_avg *sa,
			  unsigned long load, unsigned long runnable, int running);
extern void ___update_load_avg(struct sched_avg *sa, unsigned long load);

static struct vendor_util_group_property ug[UG_MAX];
static struct vendor_cfs_util vendor_cfs_util[UG_MAX - 1][CPU_NUM];
#endif

extern int vendor_sched_ug_bg_auto_prio;
extern unsigned int vendor_sched_util_post_init_scale;
extern bool vendor_sched_npi_packing;

unsigned int sched_capacity_margin[CPU_NUM] = { [0 ... CPU_NUM - 1] = DEF_UTIL_THRESHOLD };
unsigned int sched_dvfs_headroom[CPU_NUM] = { [0 ... CPU_NUM - 1] = DEF_UTIL_THRESHOLD };

struct vendor_group_property vg[VG_MAX];

extern struct vendor_group_list vendor_group_list[VG_MAX];

extern inline unsigned int uclamp_none(enum uclamp_id clamp_id);

bool wait_for_init = true;

unsigned long schedutil_cpu_util_pixel_mod(int cpu, unsigned long util_cfs,
				 unsigned long max, enum schedutil_type type,
				 struct task_struct *p);
unsigned int map_scaling_freq(int cpu, unsigned int freq);

extern void rvh_uclamp_eff_get_pixel_mod(void *data, struct task_struct *p, enum uclamp_id clamp_id,
					 struct uclamp_se *uclamp_max, struct uclamp_se *uclamp_eff,
					 int *ret);

/*****************************************************************************/
/*                       Upstream Code Section                               */
/*****************************************************************************/
/*
 * This part of code is copied from Android common GKI kernel and unmodified.
 * Any change for these functions in upstream GKI would require extensive review
 * to make proper adjustment in vendor hook.
 */

#define for_each_clamp_id(clamp_id) \
	for ((clamp_id) = 0; (clamp_id) < UCLAMP_CNT; (clamp_id)++)

static inline void uclamp_se_set(struct uclamp_se *uc_se,
				 unsigned int value, bool user_defined)
{
	uc_se->value = value;
	uc_se->bucket_id = get_bucket_id(value);
	uc_se->user_defined = user_defined;
}

static void attach_task(struct rq *rq, struct task_struct *p)
{
	lockdep_assert_held(&rq->lock);

	BUG_ON(task_rq(p) != rq);
	activate_task(rq, p, ENQUEUE_NOCLOCK);
	check_preempt_curr(rq, p, 0);
}

/*
 * attach_one_task() -- attaches the task returned from detach_one_task() to
 * its new rq.
 */
static void attach_one_task(struct rq *rq, struct task_struct *p)
{
	struct rq_flags rf;

	rq_lock(rq, &rf);
	update_rq_clock(rq);
	attach_task(rq, p);
	rq_unlock(rq, &rf);
}

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

static unsigned long capacity_curr_of(int cpu)
{
	unsigned long max_cap = cpu_rq(cpu)->cpu_capacity_orig;

	return cap_scale(max_cap, per_cpu(freq_scale, cpu));
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

u64 sched_slice(struct cfs_rq *cfs_rq, struct sched_entity *se)
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

static void
prio_changed_fair(struct rq *rq, struct task_struct *p, int oldprio)
{
	if (!task_on_rq_queued(p))
		return;

	if (rq->cfs.nr_running == 1)
		return;

	/*
	 * Reschedule if we are currently running on this runqueue and
	 * our priority decreased, or if we are not currently running on
	 * this runqueue and our priority is higher than the current's
	 */
	if (rq->curr == p) {
		if (p->prio > oldprio)
			resched_curr(rq);
	} else
		check_preempt_curr(rq, p, 0);
}

/*****************************************************************************/
/*                       New Code Section                                    */
/*****************************************************************************/
// This part of code is new for this kernel, which are mostly helper functions.

#if IS_ENABLED(CONFIG_USE_VENDOR_GROUP_UTIL)
static inline enum utilization_group get_utilization_group(struct task_struct *p, int group)
{
	if (vg[group].ug == UG_AUTO) {
		// Always consider task prio >= vendor_sched_ug_bg_auto_prio as background util
		if (p->prio >= vendor_sched_ug_bg_auto_prio)
			return UG_BG;

		return UG_FG;
	}

	return vg[group].ug;
}
#endif

bool get_prefer_high_cap(struct task_struct *p)
{
	return vg[get_vendor_group(p)].prefer_high_cap;
}

static inline bool get_task_spreading(struct task_struct *p)
{
	return vg[get_vendor_group(p)].task_spreading;
}

#if IS_ENABLED(CONFIG_USE_GROUP_THROTTLE)
static inline unsigned int get_task_group_throttle(struct task_struct *p)
{
#if IS_ENABLED(CONFIG_USE_VENDOR_GROUP_UTIL)
	return ug[get_utilization_group(p, get_vendor_group(p))].group_throttle;
#else
	return vg[get_vendor_group(p)].group_throttle;
#endif
}

#if !IS_ENABLED(CONFIG_USE_VENDOR_GROUP_UTIL)
static inline unsigned int get_group_throttle(struct task_group *tg)
{
	return vg[get_vendor_task_group_struct(tg)->group].group_throttle;
}
#endif
#endif

/*
 * If a task is in prefer_idle group, check if it could run on the cpu based on its prio and the
 * prefer_idle cpumask defined, but bail out for bulk wake (wake_q_count > 1).
 */
static inline bool is_preferred_idle_cpu(struct task_struct *p, int cpu)
{
	int vendor_group = get_vendor_group(p);

	if (!vg[vendor_group].prefer_idle)
		return true;

	if (p->wake_q_count > 1)
		return true;

	if (p->prio <= THREAD_PRIORITY_TOP_APP_BOOST) {
		return cpumask_test_cpu(cpu, &vg[vendor_group].preferred_idle_mask_high);
	} else if (p->prio <= DEFAULT_PRIO) {
		return cpumask_test_cpu(cpu, &vg[vendor_group].preferred_idle_mask_mid);
	} else {
		return cpumask_test_cpu(cpu, &vg[vendor_group].preferred_idle_mask_low);
	}
}

static inline const cpumask_t *get_preferred_idle_mask(struct task_struct *p)
{
	int vendor_group = get_vendor_group(p);

	if (p->wake_q_count > 1)
		return cpu_possible_mask;

	if (p->prio <= THREAD_PRIORITY_TOP_APP_BOOST) {
		return &vg[vendor_group].preferred_idle_mask_high;
	} else if (p->prio <= DEFAULT_PRIO) {
		return &vg[vendor_group].preferred_idle_mask_mid;
	} else {
		return &vg[vendor_group].preferred_idle_mask_low;
	}
}

void init_vendor_group_data(void)
{
	int i;
#if IS_ENABLED(CONFIG_USE_VENDOR_GROUP_UTIL)
	int j;
	struct rq *rq;
	int group;
	struct rq_flags rf;
	struct task_struct *p;
	u64 last_update_time;
#endif

	for (i = 0; i < VG_MAX; i++) {
		INIT_LIST_HEAD(&vendor_group_list[i].list);
		raw_spin_lock_init(&vendor_group_list[i].lock);
		vendor_group_list[i].cur_iterator = NULL;
	}

#if IS_ENABLED(CONFIG_USE_VENDOR_GROUP_UTIL)
	for (j = 0; j < CPU_NUM; j++) {
		rq = cpu_rq(j);

		for (i = 0; i < UG_MAX - 1; i++) {
			raw_spin_lock_init(&vendor_cfs_util[i][j].lock);
			vendor_cfs_util[i][j].avg.util_avg = 0;
			vendor_cfs_util[i][j].avg.util_sum = 0;
			vendor_cfs_util[i][j].util_removed = 0;
			vendor_cfs_util[i][j].util_est = 0;
		}

		rq_lock_irqsave(rq, &rf);
		update_rq_clock(rq);
		last_update_time = rq_clock_pelt(rq);
		for (i = 0; i < UG_MAX - 1; i++)
			vendor_cfs_util[i][j].avg.last_update_time = last_update_time;

		list_for_each_entry(p, &rq->cfs_tasks, se.group_node) {
			group = get_utilization_group(p, get_vendor_group(p));
			if (group == UG_FG)
				continue;
			vendor_cfs_util[group][j].avg.util_avg += READ_ONCE(p->se.avg.util_avg);
			vendor_cfs_util[group][j].avg.util_sum += READ_ONCE(p->se.avg.util_sum);
			vendor_cfs_util[group][j].util_est += _task_util_est(p);
		}
		rq_unlock_irqrestore(rq, &rf);
	}
#endif
}

#if IS_ENABLED(CONFIG_USE_VENDOR_GROUP_UTIL)
/* This function is called when tasks migrate among vendor groups */
void migrate_vendor_group_util(struct task_struct *p, unsigned int old, unsigned int new)
{
	int cpu = task_cpu(p);
	unsigned long flags;
	unsigned long util_avg, util_sum;
	unsigned long util_est;
	int old_mapping, new_mapping;

	old_mapping = get_utilization_group(p, old);
	new_mapping = get_utilization_group(p, new);

	if (old_mapping == new_mapping)
		return;

	if (old_mapping == UG_BG) {
		util_avg = min(vendor_cfs_util[old_mapping][cpu].avg.util_avg, p->se.avg.util_avg);
		util_sum = min(vendor_cfs_util[old_mapping][cpu].avg.util_sum, p->se.avg.util_sum);
		if (likely(sched_feat(UTIL_EST)))
			util_est = min(vendor_cfs_util[old_mapping][cpu].util_est,
				       _task_util_est(p));
	} else {
		util_avg = p->se.avg.util_avg;
		util_sum = p->se.avg.util_sum;
		if (likely(sched_feat(UTIL_EST)))
			util_est = _task_util_est(p);
	}

	//remove util from old group
	if (old_mapping == UG_BG) {
		raw_spin_lock_irqsave(&vendor_cfs_util[old_mapping][cpu].lock, flags);
		vendor_cfs_util[old_mapping][cpu].avg.util_avg -= util_avg;
		vendor_cfs_util[old_mapping][cpu].avg.util_sum -= util_sum;
		if (likely(sched_feat(UTIL_EST))) {
			if (p->on_rq)
				vendor_cfs_util[old_mapping][cpu].util_est -= util_est;
		}
		raw_spin_unlock_irqrestore(&vendor_cfs_util[old_mapping][cpu].lock, flags);
	}

	//move util to new group
	if (new_mapping == UG_BG) {
		raw_spin_lock_irqsave(&vendor_cfs_util[new_mapping][cpu].lock, flags);
		vendor_cfs_util[new_mapping][cpu].avg.util_avg += util_avg;
		vendor_cfs_util[new_mapping][cpu].avg.util_sum += util_sum;
		if (likely(sched_feat(UTIL_EST))) {
			if (p->on_rq)
				vendor_cfs_util[new_mapping][cpu].util_est += util_est;
		}
		raw_spin_unlock_irqrestore(&vendor_cfs_util[new_mapping][cpu].lock, flags);
	}
}

/* This function hooks to update_load_avg */
static inline void update_vendor_group_util(u64 now, struct cfs_rq *cfs_rq,
					    struct sched_entity *se)
{
	int curr_group = -1, se_group = get_utilization_group(task_of(se),
							      get_vendor_group(task_of(se)));
	struct sched_avg *sa;
	unsigned long removed_util;
	u32 divider;

	if (cfs_rq->curr && entity_is_task(cfs_rq->curr))
		curr_group = get_utilization_group(task_of(cfs_rq->curr),
						   get_vendor_group(task_of(cfs_rq->curr)));

	sa = &vendor_cfs_util[UG_BG][cfs_rq->rq->cpu].avg;
	if (vendor_cfs_util[UG_BG][cfs_rq->rq->cpu].util_removed) {
		removed_util = 0;
		divider = get_pelt_divider(sa);
		raw_spin_lock(&vendor_cfs_util[UG_BG][cfs_rq->rq->cpu].lock);
		swap(vendor_cfs_util[UG_BG][cfs_rq->rq->cpu].util_removed, removed_util);
		lsub_positive(&sa->util_avg, removed_util);
		lsub_positive(&sa->util_sum, removed_util * divider);
		sa->util_sum = max_t(unsigned long, sa->util_sum, sa->util_avg * PELT_MIN_DIVIDER);
		raw_spin_unlock(&vendor_cfs_util[UG_BG][cfs_rq->rq->cpu].lock);
	}

	if (curr_group == UG_BG || se_group == UG_BG || sa->util_avg != 0 || sa->util_sum != 0) {
		raw_spin_lock(&vendor_cfs_util[UG_BG][cfs_rq->rq->cpu].lock);
		if (___update_load_sum(now, sa, curr_group == UG_BG, 0, curr_group == UG_BG))
			___update_load_avg(sa, 1);
		raw_spin_unlock(&vendor_cfs_util[UG_BG][cfs_rq->rq->cpu].lock);
	}
}

/* This function hooks to update_blocked_fair */
static inline void update_vendor_group_util_blocked(u64 now, struct rq *rq)
{
	struct sched_avg *sa;

	sa = &vendor_cfs_util[UG_BG][rq->cpu].avg;
	if ((available_idle_cpu(rq->cpu) || task_is_realtime(rq->curr)) &&
	    (sa->util_avg != 0 || sa->util_sum != 0)) {
		raw_spin_lock(&vendor_cfs_util[UG_BG][rq->cpu].lock);
		if (___update_load_sum(now, sa, 0, 0, 0))
			___update_load_avg(sa, 1);
		raw_spin_unlock(&vendor_cfs_util[UG_BG][rq->cpu].lock);
	}
}

/* This function hooks to attach_entity_load_avg */
static inline void attach_vendor_group_util(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	int group = get_utilization_group(task_of(se), get_vendor_group(task_of(se)));

	if (group == UG_BG) {
		raw_spin_lock(&vendor_cfs_util[group][cfs_rq->rq->cpu].lock);
		vendor_cfs_util[group][cfs_rq->rq->cpu].avg.util_avg += se->avg.util_avg;
		vendor_cfs_util[group][cfs_rq->rq->cpu].avg.util_sum += se->avg.util_sum;
		raw_spin_unlock(&vendor_cfs_util[group][cfs_rq->rq->cpu].lock);
	}
}

/* This function hooks to detach_entity_load_avg */
static inline void detach_vendor_group_util(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	int group = get_utilization_group(task_of(se), get_vendor_group(task_of(se)));

	if (group == UG_BG) {
		raw_spin_lock(&vendor_cfs_util[group][cfs_rq->rq->cpu].lock);
		lsub_positive(&vendor_cfs_util[group][cfs_rq->rq->cpu].avg.util_avg,
			      se->avg.util_avg);
		lsub_positive(&vendor_cfs_util[group][cfs_rq->rq->cpu].avg.util_sum,
			      se->avg.util_sum);
		vendor_cfs_util[group][cfs_rq->rq->cpu].avg.util_sum = max_t(unsigned long,
			vendor_cfs_util[group][cfs_rq->rq->cpu].avg.util_sum,
			vendor_cfs_util[group][cfs_rq->rq->cpu].avg.util_avg * PELT_MIN_DIVIDER);
		raw_spin_unlock(&vendor_cfs_util[group][cfs_rq->rq->cpu].lock);
	}
}

/*
 * This function hooks to remove_entity_load_avg. It is called when tasks migrate to another cpu rq,
 * and we need to keep the removed util, which will be deduced from the prev rq in the next update
 * call on that rq.
 */
static inline void remove_vendor_group_util(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	int group = get_utilization_group(task_of(se), get_vendor_group(task_of(se)));
	unsigned long flags;

	if (group == UG_BG) {
		raw_spin_lock_irqsave(&vendor_cfs_util[group][cfs_rq->rq->cpu].lock, flags);
		vendor_cfs_util[group][cfs_rq->rq->cpu].util_removed += se->avg.util_avg;
		raw_spin_unlock_irqrestore(&vendor_cfs_util[group][cfs_rq->rq->cpu].lock, flags);
	}
}

static inline unsigned long cpu_vendor_group_util(int cpu, bool with, struct task_struct *p)
{
	int group = -1;
	unsigned long fg_group_util, util;
#if IS_ENABLED(CONFIG_USE_GROUP_THROTTLE)
	unsigned long scale_cpu = arch_scale_cpu_capacity(cpu);
#endif

	if (p)
		group = get_utilization_group(p, get_vendor_group(p));

	// Calculate bg group util first.
	util = vendor_cfs_util[UG_BG][cpu].avg.util_avg;

	if (group == UG_BG) {
		if (with) {
			util += task_util(p);
		} else {
			lsub_positive(&util, task_util(p));
		}
	}

#if IS_ENABLED(CONFIG_USE_GROUP_THROTTLE)
	util = min_t(unsigned long, util, cap_scale(ug[UG_BG].group_throttle, scale_cpu));
#endif
	util = min_t(unsigned long, util, ug[UG_BG].uc_req[UCLAMP_MAX].value);

	// For fg group, we use root util - bg group util.
	fg_group_util = cpu_rq(cpu)->cfs.avg.util_avg;
	lsub_positive(&fg_group_util, vendor_cfs_util[UG_BG][cpu].avg.util_avg);

	if (group == UG_FG) {
		if (with) {
			fg_group_util += task_util(p);
		} else {
			lsub_positive(&fg_group_util, task_util(p));
		}
	}

#if IS_ENABLED(CONFIG_USE_GROUP_THROTTLE)
		fg_group_util = min_t(unsigned long, fg_group_util,
				      cap_scale(ug[UG_FG].group_throttle, scale_cpu));
#endif
		fg_group_util = min_t(unsigned long, fg_group_util,
				      ug[UG_FG].uc_req[UCLAMP_MAX].value);

		util += fg_group_util;

	return util;
}

static inline unsigned long cpu_vendor_group_util_est(int cpu, bool with, struct task_struct *p)
{
	int group = -1;
	unsigned long fg_group_util_est, util_est;
#if IS_ENABLED(CONFIG_USE_GROUP_THROTTLE)
	unsigned long scale_cpu = arch_scale_cpu_capacity(cpu);;
#endif

	if (p)
		group = get_utilization_group(p, get_vendor_group(p));

	// Get bg group util_est first.
	util_est = vendor_cfs_util[UG_BG][cpu].util_est;

	if (group == UG_BG) {
		if (with) {
			util_est += _task_util_est(p);
		} else {
			lsub_positive(&util_est, _task_util_est(p));
		}
	}

#if IS_ENABLED(CONFIG_USE_GROUP_THROTTLE)
	util_est = min_t(unsigned long, util_est, cap_scale(ug[UG_BG].group_throttle, scale_cpu));
#endif
	util_est = min_t(unsigned long, util_est, ug[UG_BG].uc_req[UCLAMP_MAX].value);

	// For fg group, we use root util_est - bg group util_est.
	fg_group_util_est = cpu_rq(cpu)->cfs.avg.util_est.enqueued;
	lsub_positive(&fg_group_util_est, vendor_cfs_util[UG_BG][cpu].util_est);

	if (group == UG_FG) {
		if (with) {
			fg_group_util_est += _task_util_est(p);
		} else {
			lsub_positive(&fg_group_util_est, _task_util_est(p));
		}
	}

#if IS_ENABLED(CONFIG_USE_GROUP_THROTTLE)
		fg_group_util_est = min_t(unsigned long, fg_group_util_est,
					  cap_scale(ug[UG_FG].group_throttle, scale_cpu));
#endif
		fg_group_util_est = min_t(unsigned long, fg_group_util_est,
					  ug[UG_FG].uc_req[UCLAMP_MAX].value);

		util_est += fg_group_util_est;

	return util_est;
}
#endif

#if defined(CONFIG_UCLAMP_TASK) && defined(CONFIG_FAIR_GROUP_SCHED)
#if IS_ENABLED(CONFIG_USE_VENDOR_GROUP_UTIL)
unsigned long cpu_util_cfs_group_mod(struct rq *rq)
{
	if (likely(sched_feat(UTIL_EST))) {
		return max(cpu_vendor_group_util(rq->cpu, false, NULL),
			   cpu_vendor_group_util_est(rq->cpu, false, NULL));
	} else {
		return cpu_vendor_group_util(rq->cpu, false, NULL);
	}
}
#else
static inline unsigned long cpu_util_cfs_group_mod_no_est(struct rq *rq)
{
	struct cfs_rq *cfs_rq, *pos;
	unsigned long util = 0, unclamped_util = 0;
	struct task_group *tg;
#if IS_ENABLED(CONFIG_USE_GROUP_THROTTLE)
	unsigned long scale_cpu = arch_scale_cpu_capacity(rq->cpu);
#endif

	// cpu_util_cfs = root_util - subgroup_util_sum + throttled_subgroup_util_sum
	for_each_leaf_cfs_rq_safe(rq, cfs_rq, pos) {
		if (&rq->cfs != cfs_rq) {
			tg = cfs_rq->tg;
			unclamped_util += cfs_rq->avg.util_avg;
#if IS_ENABLED(CONFIG_USE_GROUP_THROTTLE)
			util += min_t(unsigned long, READ_ONCE(cfs_rq->avg.util_avg),
				cap_scale(get_group_throttle(tg), scale_cpu));
#else
			util += READ_ONCE(cfs_rq->avg.util_avg);
#endif
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
#endif
#else
#define cpu_util_cfs_group_mod cpu_util_cfs
#endif

unsigned long cpu_util(int cpu)
{
       return min(cpu_util_cfs_group_mod(cpu_rq(cpu)), capacity_of(cpu));
}

#if IS_ENABLED(CONFIG_USE_GROUP_THROTTLE)
/* Similar to cpu_util_without but only count the task's group util contribution */
#if IS_ENABLED(CONFIG_USE_VENDOR_GROUP_UTIL)
static inline unsigned long get_group_util(int cpu, struct task_struct *p, unsigned long max,
					   bool subtract)
{
	int group = get_utilization_group(p, get_vendor_group(p));
	unsigned long util = cpu_rq(cpu)->cfs.avg.util_avg;

	if (group == UG_BG) {
		util = vendor_cfs_util[UG_BG][cpu].avg.util_avg;

		if (subtract && (cpu == task_cpu(p) && READ_ONCE(p->se.avg.last_update_time)))
			lsub_positive(&util, task_util(p));

		if (likely(sched_feat(UTIL_EST))) {
			unsigned long util_est = vendor_cfs_util[UG_BG][cpu].util_est;

			if (subtract && unlikely(task_on_rq_queued(p) || current == p))
				lsub_positive(&util_est, _task_util_est(p));

			util = max(util, util_est);
		}
	// For fg group, we use root util - bg group util
	} else if (group == UG_FG) {
		util = cpu_rq(cpu)->cfs.avg.util_avg;
		lsub_positive(&util, vendor_cfs_util[UG_BG][cpu].avg.util_avg);

		if (subtract && (cpu == task_cpu(p) && READ_ONCE(p->se.avg.last_update_time)))
			lsub_positive(&util, task_util(p));

		if (likely(sched_feat(UTIL_EST))) {
			unsigned long util_est = cpu_rq(cpu)->cfs.avg.util_est.enqueued -
						 vendor_cfs_util[UG_BG][cpu].util_est;

			if (subtract && unlikely(task_on_rq_queued(p) || current == p))
				lsub_positive(&util_est, _task_util_est(p));

			util = max(util, util_est);
		}
	} else {
		SCHED_WARN_ON(1);
	}

	return min(util, max);
}
#else
static unsigned long group_util_without(int cpu, struct task_struct *p, unsigned long max)
{
	unsigned long util = READ_ONCE(task_group(p)->cfs_rq[cpu]->avg.util_avg);

	if (cpu != task_cpu(p) || !READ_ONCE(p->se.avg.last_update_time))
		goto out;

	lsub_positive(&util, task_util(p));

out:
	return min_t(unsigned long, util, max);

}
#endif
#endif

static unsigned long cpu_util_without(int cpu, struct task_struct *p, unsigned long max)
{
#if IS_ENABLED(CONFIG_USE_VENDOR_GROUP_UTIL)
	unsigned long util;

	/* Task has no contribution or is new */
	if (cpu != task_cpu(p) || !READ_ONCE(p->se.avg.last_update_time))
		util = cpu_vendor_group_util(cpu, false, NULL);
	else
		util = cpu_vendor_group_util(cpu, false, p);

	if (likely(sched_feat(UTIL_EST))) {
		unsigned long estimated;

		if (unlikely(task_on_rq_queued(p) || current == p)) {
			estimated = cpu_vendor_group_util_est(cpu, false, p);
		} else {
			estimated = cpu_vendor_group_util_est(cpu, false, NULL);
		}

		util = max(util, estimated);
	}

	return min(util, max);
#else
	struct cfs_rq *cfs_rq;
	unsigned long util;

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

		util = max_t(unsigned long, util, estimated);
	}

	/*
	 * Utilization (estimated) can exceed the CPU capacity, thus let's
	 * clamp to the maximum CPU capacity to ensure consistency with
	 * the cpu_util call.
	 */
	return min_t(unsigned long, util, max);
#endif
}

struct vendor_group_property *get_vendor_group_property(enum vendor_group group)
{
	SCHED_WARN_ON(group < VG_SYSTEM || group >= VG_MAX);

	return &(vg[group]);
}

#if IS_ENABLED(CONFIG_USE_VENDOR_GROUP_UTIL)
struct vendor_util_group_property *get_vendor_util_group_property(enum utilization_group group)
{
	SCHED_WARN_ON(group < UG_BG || group >= UG_MAX);

	return &(ug[group]);
}
#endif

static bool task_fits_capacity(struct task_struct *p, int cpu,  bool sync_boost)
{
	unsigned long uclamp_min = uclamp_eff_value(p, UCLAMP_MIN);
	unsigned long uclamp_max = uclamp_eff_value(p, UCLAMP_MAX);
	unsigned long task_util = task_util_est(p);

	if (cpu >= MAX_CAPACITY_CPU)
		return true;

	if ((get_prefer_high_cap(p) || sync_boost) && cpu < MID_CAPACITY_CPU)
		return false;

	/*
	 * Ignore uclamp if spreading the task
	 */
	if (get_task_spreading(p)) {
		uclamp_min = uclamp_none(UCLAMP_MIN);
		uclamp_max = uclamp_none(UCLAMP_MAX);
	}

#if IS_ENABLED(CONFIG_USE_GROUP_THROTTLE)
	/* clamp task utilization against its per-cpu group limit */
	task_util = min_t(unsigned long, task_util, cap_scale(get_task_group_throttle(p),
							arch_scale_cpu_capacity(cpu)));
#endif

	return util_fits_cpu(task_util, uclamp_min, uclamp_max, cpu);
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
#if IS_ENABLED(CONFIG_USE_VENDOR_GROUP_UTIL)
	unsigned long util;

	if (task_cpu(p) == cpu && dst_cpu != cpu)
		util = cpu_vendor_group_util(cpu, false, p);
	else if (task_cpu(p) != cpu && dst_cpu == cpu)
		util = cpu_vendor_group_util(cpu, true, p);
	else
		util = cpu_vendor_group_util(cpu, false, NULL);


	if (likely(sched_feat(UTIL_EST))) {
		unsigned long estimated;

		if (dst_cpu == cpu) {
			estimated = cpu_vendor_group_util_est(cpu, true, p);
		} else {
			estimated = cpu_vendor_group_util_est(cpu, false, NULL);
		}

		util = max(util, estimated);
	}

	return min(util, capacity_of(cpu));
#else
	struct cfs_rq *cfs_rq, *pos;
	unsigned long util = 0, unclamped_util = 0;
	unsigned long util_est;
	struct task_group *tg;
	long delta = 0;
	struct rq *rq = cpu_rq(cpu);
#if IS_ENABLED(CONFIG_USE_GROUP_THROTTLE)
	unsigned long scale_cpu = arch_scale_cpu_capacity(cpu);
#endif

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
#if IS_ENABLED(CONFIG_USE_GROUP_THROTTLE)
			util += min_t(unsigned long, group_util,
				cap_scale(get_group_throttle(tg), scale_cpu));
#else
			util += group_util;
#endif
		}
	}

	// For root group
	if (p->se.cfs_rq->tg == rq->cfs.tg)
		util = max_t(long, READ_ONCE(rq->cfs.avg.util_avg) - unclamped_util + util + delta,
				   0);
	else
		util = max_t(long, READ_ONCE(rq->cfs.avg.util_avg) - unclamped_util + util, 0);

	if (sched_feat(UTIL_EST)) {
		util_est = READ_ONCE(rq->cfs.avg.util_est.enqueued);

		if (dst_cpu == cpu)
			util_est += _task_util_est(p);

		util = max_t(unsigned long, util, util_est);
	}

	return min(util, capacity_of(cpu));
#endif
}

static inline unsigned long get_idle_cost(int cpu, int opp_level)
{
	if (vendor_sched_pixel_idle_em)
		return vendor_sched_pixel_idle_em->cpu_to_cluster[cpu]->opps[opp_level].cost;
	else
		return 0;
}

static inline unsigned long em_cpu_energy_pixel_mod(struct em_perf_domain *pd,
				unsigned long max_util, unsigned long sum_util, bool count_idle,
				int dst_cpu)
{
	unsigned long freq, scale_cpu, cost;
	struct em_perf_state *ps;
	int i, cpu;

	if (!sum_util)
		return 0;

	cpu = cpumask_first(to_cpumask(pd->cpus));

#if IS_ENABLED(CONFIG_PIXEL_EM)
	{
		struct pixel_em_profile **profile_ptr_snapshot;
		profile_ptr_snapshot = READ_ONCE(vendor_sched_pixel_em_profile);
		if (profile_ptr_snapshot) {
			struct pixel_em_profile *profile = READ_ONCE(*profile_ptr_snapshot);
			if (profile) {
				struct pixel_em_cluster *cluster = profile->cpu_to_cluster[cpu];
				struct pixel_em_opp *max_opp;
				struct pixel_em_opp *opp;

				max_opp = &cluster->opps[cluster->num_opps - 1];

				freq = map_util_freq_pixel_mod(max_util,
							       max_opp->freq,
							       max_opp->capacity);
				freq = map_scaling_freq(cpu, freq);

				for (i = 0; i < cluster->num_opps; i++) {
					opp = &cluster->opps[i];
					if (opp->freq >= freq)
						break;
				}

				cost = opp->cost * sum_util / max_opp->capacity;

				if (count_idle) {
					unsigned long cur_freq = arch_scale_freq_capacity(cpu) *
						max_opp->freq >> SCHED_CAPACITY_SHIFT;

					for (i = 0; i < cluster->num_opps; i++) {
						opp = &cluster->opps[i];
						if (opp->freq >= cur_freq)
							break;
					}

					cost += get_idle_cost(dst_cpu, i);
				}

				return cost;
			}
		}
	}
#endif

	scale_cpu = arch_scale_cpu_capacity(cpu);
	ps = &pd->table[pd->nr_perf_states - 1];
	freq = map_util_freq_pixel_mod(max_util, ps->frequency, scale_cpu);
	freq = map_scaling_freq(cpu, freq);

	for (i = 0; i < pd->nr_perf_states; i++) {
		ps = &pd->table[i];
		if (ps->frequency >= freq)
			break;
	}

	return ps->cost * sum_util / scale_cpu;
}

static long
compute_energy(struct task_struct *p, int dst_cpu, struct perf_domain *pd, unsigned long exit_lat)
{
	unsigned long max_util, util_cfs, cpu_util, cpu_cap;
	unsigned long sum_util, energy = 0;
	struct task_struct *tsk;
	int cpu;
	bool count_idle = false;

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

		if (cpumask_test_cpu(dst_cpu, pd_mask) && exit_lat > C1_EXIT_LATENCY)
			count_idle = true;

		energy += em_cpu_energy_pixel_mod(pd->em_pd, max_util, sum_util, count_idle,
						  dst_cpu);
	}

	trace_sched_compute_energy(p, dst_cpu, energy);
	return energy;
}

#if IS_ENABLED(CONFIG_USE_GROUP_THROTTLE)
/* If a task_group is over its group limit on a particular CPU with margin considered */
#if IS_ENABLED(CONFIG_USE_VENDOR_GROUP_UTIL)
static inline bool group_overutilized(int cpu, struct task_struct *p)
{
	unsigned long group_capacity = cap_scale(get_task_group_throttle(p),
						 arch_scale_cpu_capacity(cpu));
	unsigned long util = get_group_util(cpu, p, group_capacity, false);
	return cpu_overutilized(util, group_capacity, cpu);
}
#else
static inline bool group_overutilized(int cpu, struct task_group *tg)
{

	unsigned long group_capacity = cap_scale(get_group_throttle(tg),
					arch_scale_cpu_capacity(cpu));
	unsigned long group_util = READ_ONCE(tg->cfs_rq[cpu]->avg.util_avg);
	return cpu_overutilized(group_util, group_capacity, cpu);
}
#endif
#endif

static void prio_changed(struct task_struct *p, int old_prio, int new_prio, bool lock)
{
	struct rq *rq;
	bool queued, running;

	rq = task_rq(p);

	if (lock)
		update_rq_clock(rq);

	p->prio = new_prio;

	if (lock) {
		queued = task_on_rq_queued(p);
		running = task_current(rq, p);

		if (queued)
			p->sched_class->dequeue_task(rq, p, DEQUEUE_SAVE | DEQUEUE_NOCLOCK);
		if (running)
			put_prev_task(rq, p);
	}

	reweight_task(p, new_prio - MAX_RT_PRIO);

	if (lock) {
		if (queued)
			p->sched_class->enqueue_task(rq, p, ENQUEUE_RESTORE | ENQUEUE_NOCLOCK);
		if (running)
			set_next_task(rq, p);

		prio_changed_fair(rq, p, old_prio);
	}
}

void update_adpf_prio(struct task_struct *p, struct vendor_task_struct *vp, bool val)
{
	int new_prio, old_prio;

	if (p->prio < MAX_RT_PRIO)
		return;

	if (val) {
		raw_spin_lock(&vp->lock);
		vp->orig_prio = p->prio;
		raw_spin_unlock(&vp->lock);
	}

	old_prio = p->prio;

	if (val) {
		p->prio = NICE_TO_PRIO(MIN_NICE);
	} else {
		p->prio = vp->orig_prio;
	}

	new_prio = p->prio;

	if (old_prio != new_prio)
		prio_changed(p, old_prio, new_prio, true);
}

static void get_uclamp_on_nice(struct task_struct *p, enum uclamp_id clamp_id,
			       unsigned int *value)
{
	int group = get_vendor_group(p);

	if (clamp_id == UCLAMP_MIN) {
		if (!vg[group].uclamp_min_on_nice_enable)
			return;

		if (p->prio <= vg[group].uclamp_min_on_nice_high_prio) {
			*value = vg[group].uclamp_min_on_nice_high_value;
		} else if (p->prio <= vg[group].uclamp_min_on_nice_mid_prio) {
			*value = vg[group].uclamp_min_on_nice_mid_value;
		} else {
			*value = vg[group].uclamp_min_on_nice_low_value;
		}
	} else if (clamp_id == UCLAMP_MAX) {
		if (!vg[group].uclamp_max_on_nice_enable)
			return;

		if (p->prio <= vg[group].uclamp_max_on_nice_high_prio) {
			*value = vg[group].uclamp_max_on_nice_high_value;
		} else if (p->prio <= vg[group].uclamp_max_on_nice_mid_prio) {
			*value = vg[group].uclamp_max_on_nice_mid_value;
		} else {
			*value = vg[group].uclamp_max_on_nice_low_value;
		}
	}
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

/* UPSTREAM UCLAMP CODE - start */
static inline unsigned int
uclamp_idle_value(struct rq *rq, enum uclamp_id clamp_id,
		  unsigned int clamp_value)
{
	/*
	 * Avoid blocked utilization pushing up the frequency when we go
	 * idle (which drops the max-clamp) by retaining the last known
	 * max-clamp.
	 */
	if (clamp_id == UCLAMP_MAX) {
		rq->uclamp_flags |= UCLAMP_FLAG_IDLE;
		return clamp_value;
	}

	return uclamp_none(UCLAMP_MIN);
}

static inline void uclamp_idle_reset(struct rq *rq, enum uclamp_id clamp_id,
				     unsigned int clamp_value)
{
	/* Reset max-clamp retention only on idle exit */
	if (!(rq->uclamp_flags & UCLAMP_FLAG_IDLE))
		return;

	WRITE_ONCE(rq->uclamp[clamp_id].value, clamp_value);
}

/*
 * The effective clamp bucket index of a task depends on, by increasing
 * priority:
 * - the task specific clamp value, when explicitly requested from userspace
 * - the task group effective clamp value, for tasks not either in the root
 *   group or in an autogroup
 * - the system default clamp value, defined by the sysadmin
 */
static inline struct uclamp_se
uclamp_eff_get(struct task_struct *p, enum uclamp_id clamp_id)
{
	struct uclamp_se uc_max = uclamp_default[clamp_id];
	struct uclamp_se uc_eff;
	int ret;

	// This function will always return uc_eff
	rvh_uclamp_eff_get_pixel_mod(NULL, p, clamp_id, &uc_max, &uc_eff, &ret);

	return uc_eff;
}

static inline
unsigned int uclamp_rq_max_value(struct rq *rq, enum uclamp_id clamp_id,
				   unsigned int clamp_value)
{
	struct uclamp_bucket *bucket = rq->uclamp[clamp_id].bucket;
	int bucket_id = UCLAMP_BUCKETS - 1;

	/*
	 * Since both min and max clamps are max aggregated, find the
	 * top most bucket with tasks in.
	 */
	for ( ; bucket_id >= 0; bucket_id--) {
		if (!bucket[bucket_id].tasks)
			continue;
		return bucket[bucket_id].value;
	}

	/* No tasks -- default clamp values */
	return uclamp_idle_value(rq, clamp_id, clamp_value);
}


/*
 * When a task is enqueued on a rq, the clamp bucket currently defined by the
 * task's uclamp::bucket_id is refcounted on that rq. This also immediately
 * updates the rq's clamp value if required.
 *
 * Tasks can have a task-specific value requested from user-space, track
 * within each bucket the maximum value for tasks refcounted in it.
 * This "local max aggregation" allows to track the exact "requested" value
 * for each bucket when all its RUNNABLE tasks require the same clamp.
 */
inline void uclamp_rq_inc_id(struct rq *rq, struct task_struct *p,
			     enum uclamp_id clamp_id)
{
	struct uclamp_rq *uc_rq = &rq->uclamp[clamp_id];
	struct uclamp_se *uc_se = &p->uclamp[clamp_id];
	struct uclamp_bucket *bucket;

	lockdep_assert_held(&rq->lock);

	if(SCHED_WARN_ON(!uclamp_is_used()))
		return;

	/* Update task effective clamp */
	p->uclamp[clamp_id] = uclamp_eff_get(p, clamp_id);

	bucket = &uc_rq->bucket[uc_se->bucket_id];
	bucket->tasks++;
	uc_se->active = true;

	uclamp_idle_reset(rq, clamp_id, uc_se->value);

	/*
	 * Local max aggregation: rq buckets always track the max
	 * "requested" clamp value of its RUNNABLE tasks.
	 */
	if (bucket->tasks == 1 || uc_se->value > bucket->value)
		bucket->value = uc_se->value;

	if (uc_se->value > READ_ONCE(uc_rq->value))
		WRITE_ONCE(uc_rq->value, uc_se->value);
}

/*
 * When a task is dequeued from a rq, the clamp bucket refcounted by the task
 * is released. If this is the last task reference counting the rq's max
 * active clamp value, then the rq's clamp value is updated.
 *
 * Both refcounted tasks and rq's cached clamp values are expected to be
 * always valid. If it's detected they are not, as defensive programming,
 * enforce the expected state and warn.
 */
inline void uclamp_rq_dec_id(struct rq *rq, struct task_struct *p,
			     enum uclamp_id clamp_id)
{
	struct uclamp_rq *uc_rq = &rq->uclamp[clamp_id];
	struct uclamp_se *uc_se = &p->uclamp[clamp_id];
	struct uclamp_bucket *bucket;
	unsigned int bkt_clamp;
	unsigned int rq_clamp;

	lockdep_assert_held(&rq->lock);

	if(SCHED_WARN_ON(!uclamp_is_used()))
		return;

	/*
	 * If sched_uclamp_used was enabled after task @p was enqueued,
	 * we could end up with unbalanced call to uclamp_rq_dec_id().
	 *
	 * In this case the uc_se->active flag should be false since no uclamp
	 * accounting was performed at enqueue time and we can just return
	 * here.
	 *
	 * Need to be careful of the following enqeueue/dequeue ordering
	 * problem too
	 *
	 *	enqueue(taskA)
	 *	// sched_uclamp_used gets enabled
	 *	enqueue(taskB)
	 *	dequeue(taskA)
	 *	// Must not decrement bukcet->tasks here
	 *	dequeue(taskB)
	 *
	 * where we could end up with stale data in uc_se and
	 * bucket[uc_se->bucket_id].
	 *
	 * The following check here eliminates the possibility of such race.
	 */
	if (unlikely(!uc_se->active))
		return;

	bucket = &uc_rq->bucket[uc_se->bucket_id];

	SCHED_WARN_ON(!bucket->tasks);
	if (likely(bucket->tasks))
		bucket->tasks--;

	uc_se->active = false;

	/*
	 * Keep "local max aggregation" simple and accept to (possibly)
	 * overboost some RUNNABLE tasks in the same bucket.
	 * The rq clamp bucket value is reset to its base value whenever
	 * there are no more RUNNABLE tasks refcounting it.
	 */
	if (likely(bucket->tasks))
		return;

	rq_clamp = READ_ONCE(uc_rq->value);
	/*
	 * Defensive programming: this should never happen. If it happens,
	 * e.g. due to future modification, warn and fixup the expected value.
	 */
	SCHED_WARN_ON(bucket->value > rq_clamp);
	if (bucket->value >= rq_clamp) {
		bkt_clamp = uclamp_rq_max_value(rq, clamp_id, uc_se->value);
		WRITE_ONCE(uc_rq->value, bkt_clamp);
	}
}
/* UPSTREAM UCLAMP CODE - end */

static int find_energy_efficient_cpu(struct task_struct *p, int prev_cpu, bool sync_boost,
		cpumask_t *valid_mask)
{
	struct root_domain *rd;
	struct perf_domain *pd;
	cpumask_t idle_fit = { CPU_BITS_NONE }, idle_unfit = { CPU_BITS_NONE },
		  unimportant_fit = { CPU_BITS_NONE }, unimportant_unfit = { CPU_BITS_NONE },
		  max_spare_cap = { CPU_BITS_NONE }, packing = { CPU_BITS_NONE },
		  idle_unpreferred = { CPU_BITS_NONE }, candidates = { CPU_BITS_NONE };
	int i, weight, best_energy_cpu = -1, this_cpu = smp_processor_id();
	long cur_energy, best_energy = LONG_MAX;
	unsigned long p_util_min = uclamp_is_used() ? uclamp_eff_value(p, UCLAMP_MIN) : 0;
	unsigned long p_util_max = uclamp_is_used() ? uclamp_eff_value(p, UCLAMP_MAX) : 1024;
	unsigned long spare_cap, target_max_spare_cap = 0;
	unsigned long task_importance = ((p->prio <= DEFAULT_PRIO) ? p_util_min : 0) + p_util_max;
	unsigned int exit_lat, pd_best_exit_lat, best_exit_lat;
	bool is_idle, task_fits, util_fits;
	bool idle_target_found = false, importance_target_found = false;
	bool prefer_idle = get_prefer_idle(p), prefer_high_cap = get_prefer_high_cap(p);
	unsigned long capacity, wake_util, cpu_importance, pd_least_cpu_importantce;
#if IS_ENABLED(CONFIG_USE_GROUP_THROTTLE)
	bool group_overutilize;
	unsigned long group_capacity, wake_group_util;
#endif
	unsigned long pd_max_spare_cap, pd_max_unimportant_spare_cap, pd_max_packing_spare_cap;
	int pd_max_spare_cap_cpu, pd_best_idle_cpu, pd_most_unimportant_cpu, pd_best_packing_cpu;
	int most_spare_cap_cpu = -1, unimportant_max_spare_cap_cpu = -1, idle_max_cap_cpu = -1;
	struct cpuidle_state *idle_state;
	unsigned long util, unimportant_max_spare_cap = 0, idle_max_cap = 0;
	bool prefer_fit = prefer_idle && get_vendor_task_struct(p)->uclamp_fork_reset;
	const cpumask_t *preferred_idle_mask;

	rd = cpu_rq(this_cpu)->rd;

	rcu_read_lock();
	pd = rcu_dereference(rd->pd);
	if (!pd)
		goto out;

	for (; pd; pd = pd->next) {
		unsigned long util_min = p_util_min, util_max = p_util_max;
		unsigned long rq_util_min, rq_util_max;
		pd_max_spare_cap = 0;
		pd_max_packing_spare_cap = 0;
		pd_max_unimportant_spare_cap = 0;
		pd_best_exit_lat = UINT_MAX;
		pd_max_spare_cap_cpu = -1;
		pd_best_idle_cpu = -1;
		pd_most_unimportant_cpu = -1;
		pd_best_packing_cpu = -1;
		pd_least_cpu_importantce = SCHED_CAPACITY_SCALE << 2;

		for_each_cpu_and(i, perf_domain_span(pd), valid_mask ? valid_mask : p->cpus_ptr) {
			if (i >= CPU_NUM)
				break;

			if (!cpu_active(i))
				continue;

			capacity = capacity_of(i);
			is_idle = cpu_is_idle(i);
			cpu_importance = READ_ONCE(cpu_rq(i)->uclamp[UCLAMP_MIN].value) +
					   READ_ONCE(cpu_rq(i)->uclamp[UCLAMP_MAX].value);
			wake_util = cpu_util_without(i, p, capacity);
#if IS_ENABLED(CONFIG_USE_GROUP_THROTTLE)
			group_capacity = cap_scale(get_task_group_throttle(p),
					   arch_scale_cpu_capacity(i));
#if IS_ENABLED(CONFIG_USE_VENDOR_GROUP_UTIL)
			wake_group_util = get_group_util(i, p, group_capacity, true);
			group_overutilize = group_overutilized(i, p);
#else
			wake_group_util = group_util_without(i, p, group_capacity);
			group_overutilize = group_overutilized(i, task_group(p));
#endif
			spare_cap = min_t(unsigned long, capacity - wake_util,
					  group_capacity - wake_group_util);
#else
			spare_cap = capacity - wake_util;
#endif
			task_fits = task_fits_capacity(p, i, sync_boost);
			exit_lat = 0;
			util = cpu_util(i);

			if (is_idle) {
				idle_state = idle_get_state(cpu_rq(i));
				if (idle_state)
					exit_lat = idle_state->exit_latency;
			}

#if IS_ENABLED(CONFIG_USE_GROUP_THROTTLE)
			trace_sched_cpu_util_cfs(i, is_idle, exit_lat, cpu_importance, util,
						 capacity, wake_util, group_capacity,
						 wake_group_util, spare_cap, task_fits,
						 group_overutilize);
#else
			trace_sched_cpu_util_cfs(i, is_idle, exit_lat, cpu_importance, util,
						 capacity, wake_util, capacity,	wake_util,
						 spare_cap, task_fits, false);
#endif


			/*
			 * Skip CPUs that cannot satisfy the capacity request.
			 * IOW, placing the task there would make the CPU
			 * overutilized. Take uclamp into account to see how
			 * much capacity we can get out of the CPU; this is
			 * aligned with sched_cpu_util().
			 */
			if (uclamp_is_used() && !uclamp_rq_is_idle(cpu_rq(i))) {
				/*
				 * Open code uclamp_rq_util_with() except for
				 * the clamp() part. Ie: apply max aggregation
				 * only. util_fits_cpu() logic requires to
				 * operate on non clamped util but must use the
				 * max-aggregated uclamp_{min, max}.
				 */
				rq_util_min = uclamp_rq_get(cpu_rq(i), UCLAMP_MIN);
				rq_util_max = uclamp_rq_get(cpu_rq(i), UCLAMP_MAX);

				util_min = max(rq_util_min, p_util_min);
				util_max = max(rq_util_max, p_util_max);
			}

			util_fits = util_fits_cpu(util, util_min, util_max, i);

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

					/* Only keep the highest cpu */
					if (pd_best_idle_cpu > idle_max_cap_cpu) {
						idle_max_cap_cpu = pd_best_idle_cpu;
						idle_max_cap = capacity;
					}
				}

				if (idle_target_found)
					continue;

				/* Find an unimportant cpu. */
				if (task_importance > cpu_importance) {
					/* Choose the least important cpu*/
					if (cpu_importance < pd_least_cpu_importantce) {
						pd_least_cpu_importantce = cpu_importance;
						pd_max_unimportant_spare_cap = spare_cap;
						pd_most_unimportant_cpu = i;
						importance_target_found = true;

						/* Only keep the highest cpu */
						if (i > unimportant_max_spare_cap_cpu) {
							unimportant_max_spare_cap_cpu = i;
							unimportant_max_spare_cap = spare_cap;
						}
					/*
					 * If cpu importance is the same, choose the one with higher
					 * spare capacity.
					 */
					} else if (cpu_importance == pd_least_cpu_importantce) {
						if (spare_cap >= pd_max_unimportant_spare_cap) {
							pd_max_unimportant_spare_cap = spare_cap;
							pd_most_unimportant_cpu = i;
							importance_target_found = true;

							/* Only keep the highest cpu */
							if (i > unimportant_max_spare_cap_cpu) {
								unimportant_max_spare_cap_cpu = i;
								unimportant_max_spare_cap =
									spare_cap;
							}
						}
					}
				}

				if (importance_target_found)
					continue;

				if (prefer_high_cap && i < HIGH_CAPACITY_CPU)
					continue;

				/*
				 * Make srue prefer_fit task could find a candidate in high capacity
				 * clusters.
				 */
#if IS_ENABLED(CONFIG_USE_GROUP_THROTTLE)
				if (!prefer_fit &&
				    (group_overutilize || !util_fits))
					continue;
#else
				if (!prefer_fit && !util_fits)
					continue;
#endif

				if (prefer_fit && !task_fits)
					continue;

				/* find max spare capacity cpu, used as backup */
				if (spare_cap > target_max_spare_cap) {
					target_max_spare_cap = spare_cap;
					cpumask_clear(&max_spare_cap);
					cpumask_set_cpu(i, &max_spare_cap);
				} else if (spare_cap == target_max_spare_cap) {
					/*
					 * When spare capacity is the same, clear the choice
					 * randomly based on task_util.
					 */
					if ((task_util_est(p) % 2))
							cpumask_clear(&max_spare_cap);
					cpumask_set_cpu(i, &max_spare_cap);
				}
			} else { /* Below path is for non-prefer idle case */
#if IS_ENABLED(CONFIG_USE_GROUP_THROTTLE)
				if (group_overutilize || !util_fits)
					continue;
#else
				if (!util_fits)
					continue;
#endif

				if (spare_cap >= target_max_spare_cap) {
					target_max_spare_cap = spare_cap;
					most_spare_cap_cpu = i;
				}

				if (!task_fits)
					continue;

#if IS_ENABLED(CONFIG_USE_GROUP_THROTTLE)
				if (spare_cap < min_t(unsigned long, task_util_est(p),
				    cap_scale(get_task_group_throttle(p),
					      arch_scale_cpu_capacity(i))))
					continue;
#else
				if (spare_cap < task_util_est(p))
					continue;
#endif

				/*
				 * Find the best packing CPU with the maximum spare capacity in
				 * the performance domain
				 */
				if (vendor_sched_npi_packing && !is_idle &&
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
					} else if (exit_lat == pd_best_exit_lat) {
						/*
						 * A simple randomization by choosing the first or
						 * the last cpu if pd_max_spare_cap_cpu != prev_cpu.
						 */
						if (i == prev_cpu ||
						    (pd_max_spare_cap_cpu != prev_cpu &&
						      this_cpu % 2))
							pd_max_spare_cap_cpu = i;
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

	if (prefer_idle) {
		preferred_idle_mask = get_preferred_idle_mask(p);
		cpumask_or(&idle_unpreferred, &idle_fit, &idle_unfit);
		cpumask_andnot(&idle_unpreferred, &idle_unpreferred, preferred_idle_mask);
		cpumask_and(&idle_fit, &idle_fit, preferred_idle_mask);
		cpumask_and(&idle_unfit, &idle_unfit, preferred_idle_mask);
	}

	/* Assign candidates based on search order. */
	if (prefer_fit) {
		if (!cpumask_empty(&idle_fit)) {
			cpumask_copy(&candidates, &idle_fit);
		} else if (!cpumask_empty(&unimportant_fit)) {
			cpumask_copy(&candidates, &unimportant_fit);
		} else if (idle_max_cap_cpu != -1 && unimportant_max_spare_cap_cpu == -1) {
			cpumask_set_cpu(idle_max_cap_cpu, &candidates);
		} else if (idle_max_cap_cpu == -1 && unimportant_max_spare_cap_cpu != -1) {
			cpumask_set_cpu(unimportant_max_spare_cap_cpu, &candidates);
		} else if (idle_max_cap_cpu != -1 && unimportant_max_spare_cap_cpu != -1) {
			if (idle_max_cap >= unimportant_max_spare_cap)
				cpumask_set_cpu(idle_max_cap_cpu, &candidates);
			else
				cpumask_set_cpu(unimportant_max_spare_cap_cpu, &candidates);
		} else if (!cpumask_empty(&max_spare_cap)) {
			cpumask_copy(&candidates, &max_spare_cap);
		}
	} else {
		if (!cpumask_empty(&idle_fit)) {
			cpumask_copy(&candidates, &idle_fit);
		} else if (!cpumask_empty(&idle_unfit)) {
			/* Assign biggest cpu core found for unfit case. */
			cpumask_set_cpu(cpumask_last(&idle_unfit), &candidates);
		} else if (!cpumask_empty(&unimportant_fit)) {
			cpumask_copy(&candidates, &unimportant_fit);
		} else if (!cpumask_empty(&unimportant_unfit)) {
			cpumask_set_cpu(cpumask_last(&unimportant_unfit), &candidates);
		} else if (!cpumask_empty(&idle_unpreferred)) {
			cpumask_copy(&candidates, &idle_unpreferred);
		} else if (!cpumask_empty(&packing)) {
			cpumask_copy(&candidates, &packing);
		} else if (!cpumask_empty(&max_spare_cap)) {
			cpumask_copy(&candidates, &max_spare_cap);
		}
	}

	weight = cpumask_weight(&candidates);
	best_energy_cpu = most_spare_cap_cpu;

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

		cur_energy = compute_energy(p, i, pd, exit_lat);

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
	trace_sched_find_energy_efficient_cpu(p, prefer_idle, prefer_high_cap, task_importance,
				     &idle_fit, &idle_unfit, &unimportant_fit, &unimportant_unfit,
				     &packing, &max_spare_cap, &idle_unpreferred, best_energy_cpu);
	return best_energy_cpu;
}

#if IS_ENABLED(CONFIG_PIXEL_EM)
void vh_arch_set_freq_scale_pixel_mod(void *data, const struct cpumask *cpus,
				      unsigned long freq,
				      unsigned long max, unsigned long *scale)
{
	int i;
	struct pixel_em_profile **profile_ptr_snapshot;
	profile_ptr_snapshot = READ_ONCE(vendor_sched_pixel_em_profile);
	if (profile_ptr_snapshot) {
		struct pixel_em_profile *profile = READ_ONCE(*profile_ptr_snapshot);
		if (profile) {
			struct pixel_em_cluster *cluster;
			struct pixel_em_opp *max_opp;
			struct pixel_em_opp *opp;

			cluster = profile->cpu_to_cluster[cpumask_first(cpus)];
			max_opp = &cluster->opps[cluster->num_opps - 1];

			for (i = 0; i < cluster->num_opps; i++) {
				opp = &cluster->opps[i];
				if (opp->freq >= freq)
					break;
			}

			*scale = (opp->capacity << SCHED_CAPACITY_SHIFT) /
				  max_opp->capacity;
		}
	}
}
EXPORT_SYMBOL_GPL(vh_arch_set_freq_scale_pixel_mod);
#endif

void rvh_set_iowait_pixel_mod(void *data, struct task_struct *p, int *should_iowait_boost)
{
	*should_iowait_boost = p->in_iowait && uclamp_boosted(p);
}

void rvh_cpu_overutilized_pixel_mod(void *data, int cpu, int *overutilized)
{
	unsigned long rq_util_min = uclamp_rq_get(cpu_rq(cpu), UCLAMP_MIN);
	unsigned long rq_util_max = uclamp_rq_get(cpu_rq(cpu), UCLAMP_MAX);

	*overutilized = !util_fits_cpu(cpu_util(cpu), rq_util_min, rq_util_max, cpu);
}

/*
 * Implements a headroom function which gives the utilization (or the tasks
 * extra CPU bandwidth) to grow. The goal is to use the outcome to select the
 * frequency. We don't want an exact frequency selection so that if the tasks
 * running on the CPU don't go to sleep, they'll grow in that additional
 * headroom until we do the next frequency update to a higher one.
 */
unsigned long apply_dvfs_headroom(unsigned long util, int cpu, bool tapered)
{
	if (tapered && static_branch_unlikely(&tapered_dvfs_headroom_enable)) {
		unsigned long capacity = capacity_orig_of(cpu);
		unsigned long headroom;

		if (util >= capacity)
			return util;

		/*
		 * Taper the boosting at e top end as these are expensive and
		 * we don't need that much of a big headroom as we approach max
		 * capacity
		 *
		 */
		headroom = (capacity - util);
		/* formula: headroom * (1.X - 1) == headroom * 0.X */
		headroom = headroom *
			(sched_dvfs_headroom[cpu] - SCHED_CAPACITY_SCALE) >> SCHED_CAPACITY_SHIFT;
		return util + headroom;
	}

	return util * sched_dvfs_headroom[cpu] >> SCHED_CAPACITY_SHIFT;
}

/*
 * Caller must have called apply_dvfs_headroom() before doing the mapping to
 * frequency.
 */
unsigned long map_util_freq_pixel_mod(unsigned long util, unsigned long freq,
				      unsigned long cap)
{
	return freq * util / cap;
}

static inline struct uclamp_se
uclamp_tg_restrict_pixel_mod(struct task_struct *p, enum uclamp_id clamp_id)
{
	struct uclamp_se uc_req = p->uclamp_req[clamp_id];
	struct vendor_task_struct *vp = get_vendor_task_struct(p);
	struct vendor_binder_task_struct *vbinder = get_vendor_binder_task_struct(p);

#ifdef CONFIG_UCLAMP_TASK_GROUP
	unsigned int tg_min, tg_max, vnd_min, vnd_max, value;
	unsigned int nice_min = uclamp_none(UCLAMP_MIN), nice_max = uclamp_none(UCLAMP_MAX);

	// Task prio specific restriction
	get_uclamp_on_nice(p, UCLAMP_MIN, &nice_min);
	get_uclamp_on_nice(p, UCLAMP_MAX, &nice_max);

	// Task group restriction
	/*
	 * Tasks in autogroups or root task group should have uclamp_min is 0.
	 * uclamp_min is defined as a LIMIT rather than a PROTECTION at that
	 * level.
	 */
	if (task_group_is_autogroup(task_group(p)))
		tg_min = 0;
	else if (task_group(p) == &root_task_group)
		tg_min = 0;
	else
		tg_min = task_group(p)->uclamp[UCLAMP_MIN].value;
	tg_max = task_group(p)->uclamp[UCLAMP_MAX].value;
	// Vendor group restriction
	vnd_min = vg[vp->group].uc_req[UCLAMP_MIN].value;
	vnd_max = vg[vp->group].uc_req[UCLAMP_MAX].value;

	value = uc_req.value;
	value = clamp(value, max(nice_min, max(tg_min, vnd_min)),
		      min(nice_max, min(tg_max, vnd_max)));

	// Inherited uclamp restriction
	if (vbinder->active)
		value = clamp(value, vbinder->uclamp[UCLAMP_MIN], vbinder->uclamp[UCLAMP_MAX]);

	// RT_mutex inherited uclamp restriction
	value = clamp(value, vp->uclamp_pi[UCLAMP_MIN], vp->uclamp_pi[UCLAMP_MAX]);

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
#if !IS_ENABLED(CONFIG_USE_VENDOR_GROUP_UTIL)
		vg[i].group_throttle = max_val;
#endif
		cpumask_copy(&vg[i].preferred_idle_mask_low, cpu_possible_mask);
		cpumask_copy(&vg[i].preferred_idle_mask_mid, cpu_possible_mask);
		cpumask_copy(&vg[i].preferred_idle_mask_high, cpu_possible_mask);
		vg[i].uclamp_min_on_nice_low_value = uclamp_none(UCLAMP_MIN);
		vg[i].uclamp_min_on_nice_mid_value = uclamp_none(UCLAMP_MIN);
		vg[i].uclamp_min_on_nice_high_value = uclamp_none(UCLAMP_MIN);
		vg[i].uclamp_max_on_nice_low_value = uclamp_none(UCLAMP_MAX);
		vg[i].uclamp_max_on_nice_mid_value = uclamp_none(UCLAMP_MAX);
		vg[i].uclamp_max_on_nice_high_value = uclamp_none(UCLAMP_MAX);
		vg[i].uclamp_min_on_nice_low_prio = THREAD_PRIORITY_LOWEST;
		vg[i].uclamp_min_on_nice_mid_prio = THREAD_PRIORITY_BACKGROUND;
		vg[i].uclamp_min_on_nice_high_prio = DEFAULT_PRIO;
		vg[i].uclamp_max_on_nice_low_prio = THREAD_PRIORITY_LOWEST;
		vg[i].uclamp_max_on_nice_mid_prio = THREAD_PRIORITY_BACKGROUND;
		vg[i].uclamp_max_on_nice_high_prio = DEFAULT_PRIO;
		vg[i].uclamp_min_on_nice_enable = false;
		vg[i].uclamp_max_on_nice_enable = false;
		vg[i].uc_req[UCLAMP_MIN].value = min_val;
		vg[i].uc_req[UCLAMP_MIN].bucket_id = get_bucket_id(min_val);
		vg[i].uc_req[UCLAMP_MIN].user_defined = false;
		vg[i].uc_req[UCLAMP_MAX].value = max_val;
		vg[i].uc_req[UCLAMP_MAX].bucket_id = get_bucket_id(max_val);
		vg[i].uc_req[UCLAMP_MAX].user_defined = false;
		vg[i].ug = UG_AUTO;
	}

#if IS_ENABLED(CONFIG_USE_VENDOR_GROUP_UTIL)
	for (i = 0; i < UG_MAX; i++) {
#if IS_ENABLED(CONFIG_USE_GROUP_THROTTLE)
		ug[i].group_throttle = max_val;
#endif
		ug[i].uc_req[UCLAMP_MIN].value = min_val;
		ug[i].uc_req[UCLAMP_MAX].value = max_val;
	}
#endif
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

#if !IS_ENABLED(CONFIG_USE_VENDOR_GROUP_UTIL)
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
#if IS_ENABLED(CONFIG_USE_GROUP_THROTTLE)
	ue.enqueued = min_t(unsigned long, ue.enqueued,
			cap_scale(get_group_throttle(task_group(p)), scale_cpu));
#endif
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
#if IS_ENABLED(CONFIG_USE_GROUP_THROTTLE)
	ue.ewma = min_t(unsigned long, ue.ewma,
			cap_scale(get_group_throttle(task_group(p)), scale_cpu));
#endif
#endif
done:
	ue.enqueued |= UTIL_AVG_UNCHANGED;
	WRITE_ONCE(p->se.avg.util_est, ue);

	trace_sched_util_est_se_tp(&p->se);
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
#endif

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

static inline void uclamp_fork_pixel_mod(struct task_struct *p, struct task_struct *orig)
{
	enum uclamp_id clamp_id;
	struct vendor_task_struct *v_orig;

	v_orig = get_vendor_task_struct(orig);

	if (likely(!v_orig->uclamp_fork_reset))
		return;

	for_each_clamp_id(clamp_id) {
		uclamp_se_set(&p->uclamp_req[clamp_id],
			      uclamp_none(clamp_id), false);
	}
}

void vh_dup_task_struct_pixel_mod(void *data, struct task_struct *tsk, struct task_struct *orig)
{
	struct vendor_task_struct *v_tsk, *v_orig;

	if (wait_for_init)
		return;

	v_tsk = get_vendor_task_struct(tsk);
	v_orig = get_vendor_task_struct(orig);
	uclamp_fork_pixel_mod(tsk, orig);
	init_vendor_task_struct(v_tsk);
	v_tsk->group = v_orig->group;
	v_tsk->orig_prio = orig->static_prio;
}

void rvh_cpumask_any_and_distribute(void *data, struct task_struct *p,
	const struct cpumask *cpu_valid_mask,
	const struct cpumask *new_mask, int *dest_cpu)
{
	cpumask_t valid_mask;

	cpumask_and(&valid_mask, cpu_valid_mask, new_mask);

	/* find a cpu again for the running/runnable/waking tasks if their
	 * current cpu are not allowed
	 */
	if ((p->on_cpu || p->state == TASK_WAKING || task_on_rq_queued(p)) &&
		!cpumask_test_cpu(task_cpu(p), new_mask)) {
		*dest_cpu = find_energy_efficient_cpu(p, task_cpu(p), false, &valid_mask);

		if (*dest_cpu == -1)
			*dest_cpu = nr_cpu_ids;
	}

	trace_cpumask_any_and_distribute(p, &valid_mask, *dest_cpu);

	return;
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
	if (cpu_active(prev_cpu) && cpu_is_idle(prev_cpu) &&
	    task_fits_capacity(p, prev_cpu, sync_boost) && is_preferred_idle_cpu(p, prev_cpu)) {

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
		*target_cpu = find_energy_efficient_cpu(p, prev_cpu, sync_boost, NULL);
	}

out:
	trace_sched_select_task_rq_fair(p, task_util_est(p),
					sync_wakeup, prefer_prev, sync_boost,
					get_vendor_group(p),
					uclamp_eff_value(p, UCLAMP_MIN),
					uclamp_eff_value(p, UCLAMP_MAX),
					prev_cpu, *target_cpu);
}

void rvh_set_user_nice_pixel_mod(void *data, struct task_struct *p, long *nice, bool *allowed)
{
	struct vendor_task_struct *vp;
	unsigned long flags;

	if (p->prio < MAX_RT_PRIO)
		return;

	get_task_struct(p);

	vp = get_vendor_task_struct(p);
	if (vp->uclamp_fork_reset) {
		raw_spin_lock_irqsave(&vp->lock, flags);
		p->static_prio = vp->orig_prio = NICE_TO_PRIO(*nice);
		raw_spin_unlock_irqrestore(&vp->lock, flags);
	}

	put_task_struct(p);
}

void rvh_setscheduler_pixel_mod(void *data, struct task_struct *p)
{
	struct vendor_task_struct *vp;
	unsigned long flags;

	if (p->prio < MAX_RT_PRIO)
		return;

	vp = get_vendor_task_struct(p);
	if (vp->uclamp_fork_reset) {
		raw_spin_lock_irqsave(&vp->lock, flags);
		vp->orig_prio = p->prio;
		raw_spin_unlock_irqrestore(&vp->lock, flags);
		if (vg[vp->group].prefer_idle && p->prio != NICE_TO_PRIO(MIN_NICE)){
			p->prio = NICE_TO_PRIO(MIN_NICE);
			prio_changed(p, vp->orig_prio, p->prio, false);
		}
	}
}

void rvh_prepare_prio_fork_pixel_mod(void *data, struct task_struct *p)
{
	struct vendor_task_struct *vp;

	if (p->prio < MAX_RT_PRIO)
		return;

	vp = get_vendor_task_struct(current);
	if (vp->uclamp_fork_reset)
		p->prio = p->static_prio = vp->orig_prio;
}

static struct task_struct *detach_important_task(struct rq *src_rq, int dst_cpu)
{
	struct task_struct *p = NULL, *best_task = NULL, *backup = NULL,
		*backup_ui = NULL, *backup_unfit = NULL;

	lockdep_assert_held(&src_rq->lock);

	rcu_read_lock();

	list_for_each_entry_reverse(p, &src_rq->cfs_tasks, se.group_node) {
		struct vendor_task_struct *vp = get_vendor_task_struct(p);
		bool is_ui = false, is_boost = false;

		if (!cpumask_test_cpu(dst_cpu, p->cpus_ptr))
			continue;

		if (task_running(src_rq, p))
			continue;

		if (!get_prefer_idle(p))
			continue;

		if (vp && vp->uclamp_fork_reset)
			is_ui = true;
		else if (uclamp_eff_value(p, UCLAMP_MIN) > 0)
			is_boost = true;

		if (!is_ui && !is_boost)
			continue;

		if (task_fits_capacity(p, dst_cpu, false)) {
			if (!task_fits_capacity(p, src_rq->cpu, false)) {
				// if task is fit for new cpu but not old cpu
				// stop if we found an ADPF UI task
				// use it as backup if we found a boost task
				if (is_ui) {
					best_task = p;
					break;
				}

				backup = p;
			} else {
				if (is_ui) {
					backup_ui = p;
					continue;
				}

				if (!backup)
					backup = p;
			}
		} else {
			// if new idle is not capable, use it as backup but not for UI task.
			if (!is_ui)
				backup_unfit = p;
		}
	}

	if (best_task)
		p = best_task;
	else if (backup_ui)
		p = backup_ui;
	else if (backup)
		p = backup;
	else if (backup_unfit)
		p = backup_unfit;
	else
		p = NULL;

	if (p) {
		/* detach_task */
		deactivate_task(src_rq, p, DEQUEUE_NOCLOCK);
		set_task_cpu(p, dst_cpu);

		if (backup_unfit)
			cpu_rq(dst_cpu)->misfit_task_load = p->se.avg.load_avg;
		else
			cpu_rq(dst_cpu)->misfit_task_load = 0;
	}

	rcu_read_unlock();
	return p;
}

/*
 * In our newidle_balance, We ignore update next_interval, which could lead to
 * the next tick might being triggered prematurely. but that should be fine since
 * this is should not be happening often enough.
 */
void sched_newidle_balance_pixel_mod(void *data, struct rq *this_rq, struct rq_flags *rf,
		int *pulled_task, int *done)
{
	int cpu;
	struct rq *src_rq;
	struct task_struct *p = NULL;
	struct rq_flags src_rf;
	int this_cpu = this_rq->cpu;
	struct vendor_rq_struct *vrq = get_vendor_rq_struct(this_rq);

	if (SCHED_WARN_ON(atomic_read(&vrq->num_adpf_tasks)))
		atomic_set(&vrq->num_adpf_tasks, 0);


	/*
	 * We must set idle_stamp _before_ calling idle_balance(), such that we
	 * measure the duration of idle_balance() as idle time.
	 */
	this_rq->idle_stamp = rq_clock(this_rq);

	/*
	 * Do not pull tasks towards !active CPUs...
	 */
	if (!cpu_active(this_cpu))
		return;

	/*
	 * This is OK, because current is on_cpu, which avoids it being picked
	 * for load-balance and preemption/IRQs are still disabled avoiding
	 * further scheduler activity on it and we're being very careful to
	 * re-start the picking loop.
	 */
	rq_unpin_lock(this_rq, rf);
	raw_spin_unlock(&this_rq->lock);

	this_cpu = this_rq->cpu;
	for_each_cpu(cpu, cpu_active_mask) {
		int cpu_importnace = READ_ONCE(cpu_rq(cpu)->uclamp[UCLAMP_MIN].value) +
			READ_ONCE(cpu_rq(cpu)->uclamp[UCLAMP_MAX].value);

		if (cpu == this_cpu)
			continue;

		src_rq = cpu_rq(cpu);
		rq_lock_irqsave(src_rq, &src_rf);
		update_rq_clock(src_rq);

		if (src_rq->active_balance) {
			rq_unlock_irqrestore(src_rq, &src_rf);
			continue;
		}

		if (src_rq->nr_running <= 1) {
			rq_unlock_irqrestore(src_rq, &src_rf);
			continue;
		}

		if (cpu_importnace <= DEFAULT_IMPRATANCE_THRESHOLD || !src_rq->cfs.nr_running) {
			rq_unlock_irqrestore(src_rq, &src_rf);
			continue;
		}

		p = detach_important_task(src_rq, this_cpu);

		rq_unlock_irqrestore(src_rq, &src_rf);

		if (p) {
			attach_one_task(this_rq, p);
			break;
		}
	}

	raw_spin_lock(&this_rq->lock);
	/*
	 * While browsing the domains, we released the rq lock, a task could
	 * have been enqueued in the meantime. Since we're not going idle,
	 * pretend we pulled a task.
	 */
	if (this_rq->cfs.h_nr_running && !*pulled_task)
		*pulled_task = 1;

	/* Is there a task of a high priority class? */
	if (this_rq->nr_running != this_rq->cfs.h_nr_running)
		*pulled_task = -1;

	if (*pulled_task)
		this_rq->idle_stamp = 0;

	if (*pulled_task != 0) {
		*done = 1;
		/* TODO: need implement update_blocked_averages */
	}

	rq_repin_lock(this_rq, rf);

	return;
}

void rvh_can_migrate_task_pixel_mod(void *data, struct task_struct *mp,
	int dst_cpu, int *can_migrate)
{
	struct vendor_task_struct *mvp = get_vendor_task_struct(mp);
	struct vendor_rq_struct *vrq = get_vendor_rq_struct(cpu_rq(dst_cpu));

	if (!mvp->uclamp_fork_reset || !get_prefer_idle(mp))
		return;

	lockdep_assert_held(&cpu_rq(dst_cpu)->lock);

	if (atomic_read(&vrq->num_adpf_tasks))
		*can_migrate = 0;
}

#if IS_ENABLED(CONFIG_USE_VENDOR_GROUP_UTIL)
void rvh_attach_entity_load_avg_pixel_mod(void *data, struct cfs_rq *cfs_rq,
					  struct sched_entity *se)
{
	if (entity_is_task(se))
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
		update_vendor_group_util(now, cfs_rq, se);
}

void rvh_remove_entity_load_avg_pixel_mod(void *data, struct cfs_rq *cfs_rq,
					  struct sched_entity *se)
{
	if (entity_is_task(se))
		remove_vendor_group_util(cfs_rq, se);
}

void rvh_update_blocked_fair_pixel_mod(void *data, struct rq *rq)
{
	update_vendor_group_util_blocked(rq_clock_pelt(rq), rq);
}
#endif

void rvh_enqueue_task_fair_pixel_mod(void *data, struct rq *rq, struct task_struct *p, int flags)
{
	struct vendor_task_struct *vp = get_vendor_task_struct(p);
	struct vendor_rq_struct *vrq = get_vendor_rq_struct(rq);
	bool force_cpufreq_update = false;

	if (vp->uclamp_fork_reset)
		atomic_inc(&vrq->num_adpf_tasks);

#if IS_ENABLED(CONFIG_USE_VENDOR_GROUP_UTIL)
	if (likely(sched_feat(UTIL_EST))) {
		int group = get_utilization_group(p, get_vendor_group(p));

		if (group == UG_BG) {
			raw_spin_lock(&vendor_cfs_util[group][rq->cpu].lock);
			vendor_cfs_util[group][rq->cpu].util_est += _task_util_est(p);
			raw_spin_unlock(&vendor_cfs_util[group][rq->cpu].lock);
		}
	}
#endif

	/* Can only process uclamp after sched_slice() was updated */
	if (uclamp_is_used()) {
		if (uclamp_can_ignore_uclamp_max(rq, p)) {
			force_cpufreq_update = true;
			uclamp_set_ignore_uclamp_max(p);
			/* GKI has incremented it already, undo that */
			uclamp_rq_dec_id(rq, p, UCLAMP_MAX);
		}

		if (uclamp_can_ignore_uclamp_min(rq, p)) {
			force_cpufreq_update = true;
			uclamp_set_ignore_uclamp_min(p);
			/* GKI has incremented it already, undo that */
			uclamp_rq_dec_id(rq, p, UCLAMP_MIN);
		}
	}

	/*
	 * If we have applied the uclamp filter, we'll unconditionally request
	 * a frequency update which should take new changes into account.
	 */
	if (uclamp_is_used() && force_cpufreq_update)
		cpufreq_update_util(rq, SCHED_PIXEL_FORCE_UPDATE);
}

void rvh_dequeue_task_fair_pixel_mod(void *data, struct rq *rq, struct task_struct *p, int flags)
{
	struct vendor_task_struct *vp = get_vendor_task_struct(p);
	struct vendor_rq_struct *vrq = get_vendor_rq_struct(rq);

	if (vp->uclamp_fork_reset)
		atomic_dec(&vrq->num_adpf_tasks);

#if IS_ENABLED(CONFIG_USE_VENDOR_GROUP_UTIL)
	if (likely(sched_feat(UTIL_EST))) {
		int group = get_utilization_group(p, get_vendor_group(p));

		if (group == UG_BG) {
			raw_spin_lock(&vendor_cfs_util[group][rq->cpu].lock);
			if (!rq->cfs.h_nr_running)
				vendor_cfs_util[group][rq->cpu].util_est = 0;
			else
				lsub_positive(&vendor_cfs_util[group][rq->cpu].util_est,
					      _task_util_est(p));
			raw_spin_unlock(&vendor_cfs_util[group][rq->cpu].lock);
		}
	}
#endif

/* Resetting uclamp filter is handled in dequeue_task() */
}
