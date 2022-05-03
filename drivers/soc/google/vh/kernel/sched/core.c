// SPDX-License-Identifier: GPL-2.0-only
/* core.c
 *
 * Android Vendor Hook Support
 *
 * Copyright 2021 Google LLC
 */
#include <linux/sched.h>
#include <kernel/sched/sched.h>

#include "sched_priv.h"

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

static inline unsigned int uclamp_none(enum uclamp_id clamp_id)
{
	if (clamp_id == UCLAMP_MIN)
		return 0;
	return SCHED_CAPACITY_SCALE;
}

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

	for_each_clamp_id(clamp_id) {
		uclamp_se_set(&p->uclamp_req[clamp_id],
			      uclamp_none(clamp_id), false);
	}
}


void rvh_sched_fork_pixel_mod(void *data, struct task_struct *p)
{
	uclamp_fork_pixel_mod(p);
}
