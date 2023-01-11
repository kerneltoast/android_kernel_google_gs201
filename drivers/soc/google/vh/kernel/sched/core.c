// SPDX-License-Identifier: GPL-2.0-only
/* core.c
 *
 * Android Vendor Hook Support
 *
 * Copyright 2021 Google LLC
 */
#include <linux/sched.h>
#include <kernel/sched/sched.h>

#include "../../../../../android/binder_internal.h"
#include "sched_events.h"
#include "sched_priv.h"

struct vendor_group_list vendor_group_list[VG_MAX];

#if IS_ENABLED(CONFIG_UCLAMP_STATS)
extern void update_uclamp_stats(int cpu, u64 time);
#endif


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

/*****************************************************************************/
/*                       New Code Section                                    */
/*****************************************************************************/
/*
 * This part of code is new for this kernel, which are mostly helper functions.
 */

/*****************************************************************************/
/*                       Modified Code Section                               */
/*****************************************************************************/
/*
 * This part of code is vendor hook functions, which modify or extend the original
 * functions.
 */
static inline void uclamp_fork_pixel_mod(struct task_struct *p)
{
	enum uclamp_id clamp_id;
	struct vendor_task_struct *vp;

	vp = get_vendor_task_struct(p);
	if (likely(!vp->uclamp_fork_reset))
		return;

	vp->uclamp_fork_reset = 0;

	for_each_clamp_id(clamp_id) {
		uclamp_se_set(&p->uclamp_req[clamp_id],
			      uclamp_none(clamp_id), false);
	}
}

void rvh_sched_fork_pixel_mod(void *data, struct task_struct *p)
{
	uclamp_fork_pixel_mod(p);
}

void rvh_enqueue_task_pixel_mod(void *data, struct rq *rq, struct task_struct *p, int flags)
{
	struct vendor_task_struct *vp = get_vendor_task_struct(p);
	int group;

	raw_spin_lock(&vp->lock);
	if (!vp->queued_to_list) {
		group = get_vendor_group(p);
		add_to_vendor_group_list(&vp->node, group);
		vp->queued_to_list = true;
	}
	raw_spin_unlock(&vp->lock);
}

void rvh_dequeue_task_pixel_mod(void *data, struct rq *rq, struct task_struct *p, int flags)
{
	struct vendor_task_struct *vp = get_vendor_task_struct(p);
	int group;

#if IS_ENABLED(CONFIG_UCLAMP_STATS)
	if (rq->nr_running == 1)
		update_uclamp_stats(rq->cpu, rq_clock(rq));
#endif

	raw_spin_lock(&vp->lock);
	if (vp->queued_to_list) {
		group = get_vendor_group(p);
		remove_from_vendor_group_list(&vp->node, group);
		vp->queued_to_list = false;
	}
	raw_spin_unlock(&vp->lock);
}

void vh_binder_set_priority_pixel_mod(void *data, struct binder_transaction *t,
	struct task_struct *p)
{
	struct vendor_binder_task_struct *vbinder = get_vendor_binder_task_struct(p);

	if (!t->from || vbinder->active)
		return;

	vbinder->active = true;

	/* inherit uclamp */
	vbinder->uclamp[UCLAMP_MIN] = uclamp_eff_value(current, UCLAMP_MIN);
	vbinder->uclamp[UCLAMP_MAX] = uclamp_eff_value(current, UCLAMP_MAX);

	/* inherit prefer_idle */
	vbinder->prefer_idle = get_prefer_idle(current);
}

void vh_binder_restore_priority_pixel_mod(void *data, struct binder_transaction *t,
	struct task_struct *p)
{
	struct vendor_binder_task_struct *vbinder = get_vendor_binder_task_struct(p);

	if (vbinder->active) {
		vbinder->uclamp[UCLAMP_MIN] = uclamp_none(UCLAMP_MIN);
		vbinder->uclamp[UCLAMP_MAX] = uclamp_none(UCLAMP_MAX);
		vbinder->prefer_idle = false;
		vbinder->active = false;
	}
}

void rvh_rtmutex_prepare_setprio_pixel_mod(void *data, struct task_struct *p,
	struct task_struct *pi_task)
{
	struct vendor_task_struct *vp = get_vendor_task_struct(p);

	if (pi_task) {
		unsigned long p_util = task_util(p);
		unsigned long p_uclamp_min = uclamp_eff_value(p, UCLAMP_MIN);
		unsigned long p_uclamp_max = uclamp_eff_value(p, UCLAMP_MAX);
		unsigned long pi_util = task_util(pi_task);
		unsigned long pi_uclamp_min = uclamp_eff_value(pi_task, UCLAMP_MIN);
		unsigned long pi_uclamp_max = uclamp_eff_value(pi_task, UCLAMP_MAX);

		/*
		 * Take task's util into consideration first to do full
		 * performance inheritance.
		 *
		 * If pi_uclamp_min = 612 but pi_util is 812, then setting
		 * p_uclamp_min to 612 is not enough as the task will still run
		 * slower.
		 *
		 * Or if pi_uclamp_min is 0 but pi_util is 800 while p_util is
		 * 100, then pi_task could wait for longer to acquire the lock
		 * because the performance of p is too low.
		 */
		p_uclamp_min = clamp(p_util, p_uclamp_min, p_uclamp_max);
		pi_uclamp_min = clamp(pi_util, pi_uclamp_min, pi_uclamp_max);

		/* Inherit unclamp_min/max if they're inverted */

		if (p_uclamp_min < pi_uclamp_min)
			vp->uclamp_pi[UCLAMP_MIN] = pi_uclamp_min;

		if (p_uclamp_max < pi_uclamp_max || pi_uclamp_min > p_uclamp_max)
			vp->uclamp_pi[UCLAMP_MAX] = pi_uclamp_max;

		return;
	}

	vp->uclamp_pi[UCLAMP_MIN] = uclamp_none(UCLAMP_MIN);
	vp->uclamp_pi[UCLAMP_MAX] = uclamp_none(UCLAMP_MAX);
}
