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

struct vendor_cfs_util {
	raw_spinlock_t lock;
	struct sched_avg avg;
	unsigned long util_removed;
	unsigned int util_est;
};

unsigned long map_util_freq_pixel_mod(unsigned long util, unsigned long freq,
				      unsigned long cap, int cpu);

enum vendor_group_attribute {
	VTA_TASK_GROUP,
	VTA_PROC_GROUP,
};

struct vendor_rq_struct {
	raw_spinlock_t lock;
	unsigned long util_removed;
};

ANDROID_VENDOR_CHECK_SIZE_ALIGN(u64 android_vendor_data1[96], struct vendor_rq_struct t);

static inline struct vendor_rq_struct *get_vendor_rq_struct(struct rq *rq)
{
	return (struct vendor_rq_struct *)rq->android_vendor_data1;
}

int acpu_init(void);
extern struct proc_dir_entry *vendor_sched;
