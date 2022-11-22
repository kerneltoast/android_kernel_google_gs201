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
enum vendor_util_group {
	VUG_BG = 0,
	// VUG_FG must be the last one so that we could skip it.
	VUG_FG,
	VUG_MAX,
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
	VG_MAX,
};

struct vendor_binder_task_struct {
	unsigned int uclamp[UCLAMP_CNT];
	bool prefer_idle;
	bool active;
};

struct uclamp_filter {
	unsigned int uclamp_min_ignored : 1;
	unsigned int uclamp_max_ignored : 1;
};

/*
 * Always remember to initialize any new fields added here in
 * vh_dup_task_struct_pixel_mod() or you'll find newly forked tasks inheriting
 * random states from the parent.
 */
struct vendor_task_struct {
	raw_spinlock_t lock;
	enum vendor_group group;
	unsigned long direct_reclaim_ts;
	struct list_head node;
	bool queued_to_list;
	bool uclamp_fork_reset;
	bool prefer_idle;
	struct uclamp_filter uclamp_filter;

	/* parameters for binder inheritance */
	struct vendor_binder_task_struct binder_task;
	/* parameters for RT inheritance */
	unsigned int uclamp_pi[UCLAMP_CNT];
};

ANDROID_VENDOR_CHECK_SIZE_ALIGN(u64 android_vendor_data1[64], struct vendor_task_struct t);


static inline struct vendor_task_struct *get_vendor_task_struct(struct task_struct *p)
{
	return (struct vendor_task_struct *)p->android_vendor_data1;
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
	struct vendor_task_struct *vendor_task =
		(struct vendor_task_struct *)p->android_vendor_data1;
	vendor_task->group = group;
}
#endif
