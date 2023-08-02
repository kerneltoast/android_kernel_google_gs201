/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _VH_SCHED_H
#define _VH_SCHED_H

static inline struct vendor_task_struct *get_vendor_task_struct(struct task_struct *p)
{
	return &p->vendor_ts;
}

static inline struct vendor_binder_task_struct *get_vendor_binder_task_struct(struct task_struct *p)
{
	return &get_vendor_task_struct(p)->binder_task;
}

static inline int get_vendor_group(struct task_struct *p)
{
	return get_vendor_task_struct(p)->group;
}

static inline void set_vendor_group(struct task_struct *p,  enum vendor_group group)
{
	struct vendor_task_struct *vendor_task = get_vendor_task_struct(p);
	vendor_task->group = group;
}
#endif
