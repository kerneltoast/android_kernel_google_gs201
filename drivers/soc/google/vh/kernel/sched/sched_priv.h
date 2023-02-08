/* SPDX-License-Identifier: GPL-2.0 */
#include "../../include/sched.h"

#define MIN_CAPACITY_CPU    CONFIG_VH_MIN_CAPACITY_CPU
#define MID_CAPACITY_CPU    CONFIG_VH_MID_CAPACITY_CPU
#define MAX_CAPACITY_CPU    CONFIG_VH_MAX_CAPACITY_CPU
#define HIGH_CAPACITY_CPU   CONFIG_VH_HIGH_CAPACITY_CPU
#define CPU_NUM             CONFIG_VH_SCHED_CPU_NR
#define CLUSTER_NUM         3
#define UCLAMP_STATS_SLOTS  21
#define UCLAMP_STATS_STEP   (100 / (UCLAMP_STATS_SLOTS - 1))
#define DEF_UTIL_THRESHOLD  1280
#define DEF_UTIL_POST_INIT_SCALE  512
#define C1_EXIT_LATENCY     1
/*
 * For cpu running normal tasks, its uclamp.min will be 0 and uclamp.max will be 1024,
 * and the sum will be 1024. We use this as index that cpu is not running important tasks.
 */
#define DEFAULT_IMPRATANCE_THRESHOLD	1024

#define UCLAMP_BUCKET_DELTA DIV_ROUND_CLOSEST(SCHED_CAPACITY_SCALE, UCLAMP_BUCKETS)

/* Iterate thr' all leaf cfs_rq's on a runqueue */
#define for_each_leaf_cfs_rq_safe(rq, cfs_rq, pos)			\
	list_for_each_entry_safe(cfs_rq, pos, &rq->leaf_cfs_rq_list,	\
				 leaf_cfs_rq_list)

#define get_bucket_id(__val)								      \
		min_t(unsigned int,							      \
		      __val / DIV_ROUND_CLOSEST(SCHED_CAPACITY_SCALE, UCLAMP_BUCKETS),	      \
		      UCLAMP_BUCKETS - 1)

#define cpu_overutilized(cap, max, cpu)	\
		((cap) * sched_capacity_margin[cpu] > (max) << SCHED_CAPACITY_SHIFT)

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

#define __container_of(ptr, type, member) ({			\
	void *__mptr = (void *)(ptr);				\
	((type *)(__mptr - offsetof(type, member))); })

#define remove_from_vendor_group_list(__node, __group) do {	\
	raw_spin_lock(&vendor_group_list[__group].lock);	\
	if (__node == vendor_group_list[__group].cur_iterator)	\
		vendor_group_list[__group].cur_iterator = (__node)->prev;	\
	list_del_init(__node);					\
	raw_spin_unlock(&vendor_group_list[__group].lock);	\
} while (0)

#define add_to_vendor_group_list(__node, __group) do {		\
	raw_spin_lock(&vendor_group_list[__group].lock);	\
	list_add_tail(__node, &vendor_group_list[__group].list);	\
	raw_spin_unlock(&vendor_group_list[__group].lock);	\
} while (0)

struct vendor_group_property {
	bool prefer_idle;
	bool prefer_high_cap;
	bool task_spreading;
	unsigned int group_throttle;
	struct uclamp_se uc_req[UCLAMP_CNT];
};

struct uclamp_stats {
	spinlock_t lock;
	bool last_min_in_effect;
	bool last_max_in_effect;
	unsigned int last_uclamp_min_index;
	unsigned int last_uclamp_max_index;
	unsigned int last_util_diff_min_index;
	unsigned int last_util_diff_max_index;
	u64 util_diff_min[UCLAMP_STATS_SLOTS];
	u64 util_diff_max[UCLAMP_STATS_SLOTS];
	u64 total_time;
	u64 last_update_time;
	u64 time_in_state_min[UCLAMP_STATS_SLOTS];
	u64 time_in_state_max[UCLAMP_STATS_SLOTS];
	u64 effect_time_in_state_min[UCLAMP_STATS_SLOTS];
	u64 effect_time_in_state_max[UCLAMP_STATS_SLOTS];
};

struct vendor_group_list {
	struct list_head list;
	raw_spinlock_t lock;
	struct list_head *cur_iterator;
};

unsigned long map_util_freq_pixel_mod(unsigned long util, unsigned long freq,
				      unsigned long cap, int cpu);

enum vendor_group_attribute {
	VTA_TASK_GROUP,
	VTA_PROC_GROUP,
};

struct vendor_task_group_struct {
	enum vendor_group group;
};

ANDROID_VENDOR_CHECK_SIZE_ALIGN(u64 android_vendor_data1[4], struct vendor_task_group_struct t);

extern unsigned int vendor_sched_uclamp_threshold;
extern bool vendor_sched_reduce_prefer_idle;
extern struct vendor_group_property vg[VG_MAX];

/*****************************************************************************/
/*                       Upstream Code Section                               */
/*****************************************************************************/
/*
 * This part of code is copied from Android common GKI kernel and unmodified.
 * Any change for these functions in upstream GKI would require extensive review
 * to make proper adjustment in vendor hook.
 */

static inline unsigned long task_util(struct task_struct *p)
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

/*****************************************************************************/
/*                       New Code Section                                    */
/*****************************************************************************/
/*
 * This part of code is new for this kernel, which are mostly helper functions.
 */

static inline struct vendor_task_group_struct *get_vendor_task_group_struct(struct task_group *tg)
{
	return (struct vendor_task_group_struct *)tg->android_vendor_data1;
}

struct vendor_rq_struct {
	raw_spinlock_t lock;
	unsigned long util_removed;
};

ANDROID_VENDOR_CHECK_SIZE_ALIGN(u64 android_vendor_data1[96], struct vendor_rq_struct t);

static inline struct vendor_rq_struct *get_vendor_rq_struct(struct rq *rq)
{
	return (struct vendor_rq_struct *)rq->android_vendor_data1;
}

static inline bool get_prefer_idle(struct task_struct *p)
{
	// For group based prefer_idle vote, filter our smaller or low prio or
	// have throttled uclamp.max settings
	// Ignore all checks, if the prefer_idle is from per-task API.

	struct vendor_task_struct *vp = get_vendor_task_struct(p);
	struct vendor_binder_task_struct *vbinder = get_vendor_binder_task_struct(p);

	if (vendor_sched_reduce_prefer_idle && !vp->uclamp_fork_reset)
		return (vg[vp->group].prefer_idle &&
			task_util_est(p) >= vendor_sched_uclamp_threshold &&
			p->prio <= DEFAULT_PRIO &&
			uclamp_eff_value(p, UCLAMP_MAX) == SCHED_CAPACITY_SCALE) ||
			vp->prefer_idle || vbinder->prefer_idle;
	else
		return vg[vp->group].prefer_idle || vp->prefer_idle || vbinder->prefer_idle;
}

int acpu_init(void);
extern struct proc_dir_entry *vendor_sched;
