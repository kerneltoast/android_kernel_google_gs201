// SPDX-License-Identifier: GPL-2.0-only
/* cgroup.c
 *
 * Android Vendor Hook Support
 *
 * Copyright 2020 Google LLC
 */

#include <linux/sched.h>
#include <linux/cgroup.h>

void rvh_cgroup_force_kthread_migration_pixel_mod(void *data, struct task_struct *tsk,
						  struct cgroup *dst_cgrp, bool *force_migration)
{
	/*
	 * RT kthreads may be born in a cgroup with no rt_runtime allocated.
	 * Just say no.
	 */
#ifdef CONFIG_RT_GROUP_SCHED
	if (tsk->prio < MAX_RT_PRIO && tsk->no_cgroup_migration &&
	    (dst_cgrp->root->subsys_mask & (1U << cpu_cgrp_id)))
		*force_migration = false;
#endif

	/*
	 * kthreads may acquire PF_NO_SETAFFINITY during initialization.
	 * If userland migrates such a kthread to a non-root cgroup, it can
	 * become trapped in a cpuset. Just say no.
	 */
#ifdef CONFIG_CPUSETS
	if ((tsk->no_cgroup_migration || (tsk->flags & PF_NO_SETAFFINITY)) &&
			(dst_cgrp->root->subsys_mask & (1U << cpuset_cgrp_id)))
		*force_migration = false;
#endif
	*force_migration = true;
}
