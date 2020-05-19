/* SPDX-License-Identifier: GPL-2.0-only */
//
// IO performance mode with UFS
//
// Copyright (C) 2019 Samsung Electronics Co., Ltd.
//
// Authors:
//	Kiwoong <kwmad.kim@samsung.com>
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
#ifndef _UFS_PERF_H_
#define _UFS_PERF_H_

#include <linux/types.h>
#include <linux/pm_qos.h>
#include <linux/kthread.h>
#include <linux/completion.h>
#include <linux/spinlock.h>
#include <linux/sched/clock.h>

enum ufs_perf_op {
	UFS_PERF_OP_NONE = 0,
	UFS_PERF_OP_R,
	UFS_PERF_OP_W,
	UFS_PERF_OP_MAX,
};

enum ufs_perf_ctrl {
	UFS_PERF_CTRL_NONE = 0,	/* Not used to run handler */
	UFS_PERF_CTRL_LOCK,
	UFS_PERF_CTRL_RELEASE,
};

struct ufs_perf_control {
	/* from device tree */
	u32 th_chunk_in_kb;	/* big vs little */
	u32 th_count_b;		/* count for big chunk */
	u32 th_count_l;		/* count for little chunk */
	u32 th_period_in_ms_b;	/* period for big chunk */
	u32 th_period_in_ms_l;	/* period for little chunk */
	u32 th_reset_in_ms;	/* timeout for reset */

	struct pm_qos_request	pm_qos_int;
	s32			pm_qos_int_value;
	struct pm_qos_request	pm_qos_mif;
	s32			pm_qos_mif_value;
	struct pm_qos_request	pm_qos_cluster1;
	s32			pm_qos_cluster1_value;
	struct pm_qos_request	pm_qos_cluster0;
	s32			pm_qos_cluster0_value;

	/* spin lock */
	spinlock_t lock;

	/* Control factors */
	bool is_locked;
	bool is_held;
	bool will_stop;

	/* Control factors, need to care for concurrency */
	u32 count_b;		/* big chunk */
	u32 count_l;		/* little chunk */
	s64 cp_time;		/* last check point time */
	s64 cur_time;		/* current point time */

	u32 ctrl_flag;
	u32 ctrl_flag_in_transit;

	struct task_struct *handler;	/* thread for PM QoS */
	bool is_active;			/* handler status */
	struct completion completion;	/* wake-up source */
	struct timer_list reset_timer;	/* stat reset timer */
};

/* EXTERNAL FUNCTIONS */
void ufs_perf_reset(void *data, bool boot);
void ufs_perf_update_stat(void *data, unsigned int len, enum ufs_perf_op op);
void ufs_perf_populate_dt(void *data, struct device_node *np);
bool ufs_perf_init(void **data, struct device *dev);
void ufs_perf_exit(void *data);

#endif /* _UFS_PERF_H_ */
