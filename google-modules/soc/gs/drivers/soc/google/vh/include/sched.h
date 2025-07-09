/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _VH_SCHED_H
#define _VH_SCHED_H

#define ANDROID_VENDOR_CHECK_SIZE_ALIGN(_orig, _new)				\
		static_assert(sizeof(struct{_new;}) <= sizeof(struct{_orig;}),	\
			       __FILE__ ":" __stringify(__LINE__) ": "		\
			       __stringify(_new)				\
			       " is larger than "				\
			       __stringify(_orig) );				\
		static_assert(__alignof__(struct{_new;}) <= __alignof__(struct{_orig;}),	\
			       __FILE__ ":" __stringify(__LINE__) ": "		\
			       __stringify(_orig)				\
			       " is not aligned the same as "			\
			       __stringify(_new) );

// Maximum size: u64[2] for ANDROID_VENDOR_DATA_ARRAY(1, 2) in task_struct
#if IS_ENABLED(CONFIG_USE_VENDOR_GROUP_UTIL)
enum utilization_group {
	UG_BG = 0,
	UG_FG,
	UG_AUTO,
	UG_MAX = UG_AUTO,
};
#endif

enum vendor_group {
	VG_SYSTEM = 0,
	VG_TOPAPP,
	VG_FOREGROUND,
	VG_CAMERA,
	VG_CAMERA_POWER,
	VG_BACKGROUND,
	VG_SYSTEM_BACKGROUND,
	VG_NNAPI_HAL,
	VG_RT,
	VG_DEX2OAT,
	VG_OTA,
	VG_SF,
	VG_FOREGROUND_WINDOW,
	VG_MAX,
};

enum vendor_inheritnace_t {
	VI_BINDER = 0,
	VI_RTMUTEX,
	VI_MAX,
};

enum vendor_sched_qos {
	SCHED_QOS_NONE,
	SCHED_QOS_POWER_EFFICIENCY,
	SCHED_QOS_SENSITIVE_STANDARD,
	SCHED_QOS_SENSITIVE_HIGH,
	SCHED_QOS_SENSITIVE_EXTREME,
	SCHED_QOS_MAX,
};

struct vendor_inheritance_struct {
	unsigned int uclamp[VI_MAX][UCLAMP_CNT];
	short int adpf;
	short int prefer_idle;
	short int prefer_fit;
	short int prefer_high_cap;
	short int preempt_wakeup;
};

struct uclamp_filter {
	unsigned int uclamp_min_ignored : 1;
	unsigned int uclamp_max_ignored : 1;
};

struct thermal_cap {
	unsigned int uclamp_max;
	unsigned int freq;
};

/*
 * Always remember to initialize any new fields added here in
 * init_vendor_task_struct() or you'll find newly forked tasks inheriting
 * random states from the parent.
 */
struct vendor_task_struct {
	raw_spinlock_t lock;
	enum vendor_group group;
	unsigned long direct_reclaim_ts;
	struct list_head node;
	int queued_to_list;
	bool auto_prefer_high_cap;
	int auto_uclamp_max_flags;	// Relative to cpu instead of absolute
	struct uclamp_filter uclamp_filter;
	int orig_prio;
	int orig_policy;		/* Protected by task_rq_lock() */
	unsigned long iowait_boost;
	bool is_binder_task;

	/* parameters for inheritance */
	struct vendor_inheritance_struct vi;

	u64 runnable_start_ns;
	u64 prev_sum_exec_runtime;
	u64 delta_exec;
	u64 last_dequeue;
	unsigned long util_enqueued;
	unsigned long util_dequeued;
	unsigned long prev_util_dequeued;
	unsigned long prev_util;
	bool ignore_util_est_update;

	/* sched qos attributes */
	unsigned int rampup_multiplier;
	enum vendor_sched_qos sched_qos_profile;
	unsigned long sched_qos_user_defined_flag;
	unsigned long prev_sched_qos_user_defined_flag;

	/*
	 * A general field for time measurement in the same process context.
	 * Be careful it should be used for stackwise, use the wrapper
	 * functions to access this field:
	 * - set_vendor_task_struct_private
	 * - get_and_reset_vendor_task_struct_private
	 */
	unsigned long private;

	// ADPF scheduler hint value.
	int adpf_adj;
	// Definition of real_cap: the current cpu_cap that a task was actually running on.
	u64 real_cap_avg;
	// The total durtion for real_cap calculation.
	u64 real_cap_total_ns;
	// Last updated timestamp of real_cap calculation.
	u64 real_cap_update_ns;
};

ANDROID_VENDOR_CHECK_SIZE_ALIGN(u64 android_vendor_data1[64], struct vendor_task_struct t);

static inline struct vendor_task_struct *get_vendor_task_struct(struct task_struct *p)
{
	return (struct vendor_task_struct *)p->android_vendor_data1;
}

static inline struct vendor_inheritance_struct *get_vendor_inheritance_struct(struct task_struct *p)
{
	return &get_vendor_task_struct(p)->vi;
}

static inline int get_vendor_group(struct task_struct *p)
{
       return get_vendor_task_struct(p)->group;
}

static inline void set_vendor_group(struct task_struct *p,  enum vendor_group group)
{
	struct vendor_task_struct *vendor_task =
		(struct vendor_task_struct *)p->android_vendor_data1;
	vendor_task->group = group;
}

static inline void set_vendor_task_struct_private(struct vendor_task_struct *p, unsigned long val)
{
	WARN_ON(p->private != 0);
	p->private = val;
}

static inline unsigned long get_and_reset_vendor_task_struct_private(struct vendor_task_struct *p)
{
	unsigned long val = p->private;

	p->private = 0;
	return val;
}

int sched_thermal_freq_cap(unsigned int cpu, unsigned int freq);

void update_task_real_cap(struct task_struct *p);

#endif
