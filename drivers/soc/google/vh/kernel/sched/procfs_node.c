// SPDX-License-Identifier: GPL-2.0-only
/* sysfs_node.c
 *
 * Android Vendor Hook Support
 *
 * Copyright 2020 Google LLC
 */
#include <linux/lockdep.h>
#include <linux/kobject.h>
#include <linux/sched.h>
#include <linux/sched/signal.h>
#include <linux/sched/task.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <kernel/sched/sched.h>
#include <trace/events/power.h>

#include "sched_priv.h"

#if IS_ENABLED(CONFIG_UCLAMP_STATS)
extern void reset_uclamp_stats(void);
DECLARE_PER_CPU(struct uclamp_stats, uclamp_stats);
#endif

unsigned int __read_mostly vendor_sched_util_post_init_scale = DEF_UTIL_POST_INIT_SCALE;
bool __read_mostly vendor_sched_npi_packing = true; //non prefer idle packing
bool __read_mostly vendor_sched_idle_balancer = true; //prefer vendor idle balancer
bool __read_mostly vendor_sched_reduce_prefer_idle = true;
bool __read_mostly vendor_sched_boost_adpf_prio = true;
struct proc_dir_entry *vendor_sched;
extern struct vendor_group_list vendor_group_list[VG_MAX];

extern void initialize_vendor_group_property(void);

extern struct vendor_group_property *get_vendor_group_property(enum vendor_group group);

#if IS_ENABLED(CONFIG_USE_VENDOR_GROUP_UTIL)
int __read_mostly vendor_sched_ug_bg_auto_prio = THREAD_PRIORITY_BACKGROUND;

extern void migrate_vendor_group_util(struct task_struct *p, unsigned int old, unsigned int new);
extern struct vendor_util_group_property *get_vendor_util_group_property(
	enum utilization_group group);
#endif

extern void update_adpf_prio(struct task_struct *p, struct vendor_task_struct *vp, bool val);

static void apply_uclamp_change(enum vendor_group group, enum uclamp_id clamp_id);

struct uclamp_se uclamp_default[UCLAMP_CNT];
unsigned int pmu_poll_time_ms = 10;
bool pmu_poll_enabled;
extern int pmu_poll_enable(void);
extern void pmu_poll_disable(void);

extern unsigned int sysctl_sched_uclamp_min_filter_us;
extern unsigned int sysctl_sched_uclamp_max_filter_divider;

#define MAX_PROC_SIZE 128

static const char *GRP_NAME[VG_MAX] = {"sys", "ta", "fg", "cam", "cam_power", "bg", "sys_bg",
				       "nnapi", "rt", "dex2oat", "ota", "sf"};

#define PROC_OPS_RW(__name) \
		static int __name##_proc_open(\
			struct inode *inode, struct file *file) \
		{ \
			return single_open(file,\
			__name##_show, PDE_DATA(inode));\
		} \
		static const struct proc_ops  __name##_proc_ops = { \
			.proc_open	=  __name##_proc_open, \
			.proc_read	= seq_read, \
			.proc_lseek	= seq_lseek,\
			.proc_release = single_release,\
			.proc_write	=  __name##_store,\
		}

#define PROC_OPS_RO(__name) \
		static int __name##_proc_open(\
			struct inode *inode, struct file *file) \
		{ \
			return single_open(file,\
			__name##_show, PDE_DATA(inode));\
		} \
		static const struct proc_ops __name##_proc_ops = { \
			.proc_open	= __name##_proc_open, \
			.proc_read	= seq_read, \
			.proc_lseek	= seq_lseek,\
			.proc_release = single_release,\
		}

#define PROC_OPS_WO(__name) \
		static int __name##_proc_open(\
			struct inode *inode, struct file *file) \
		{ \
			return single_open(file,\
			NULL, NULL);\
		} \
		static const struct proc_ops __name##_proc_ops = { \
			.proc_open	= __name##_proc_open, \
			.proc_lseek	= seq_lseek,\
			.proc_release = single_release,\
			.proc_write	= __name##_store,\
		}

#define PROC_ENTRY(__name) {__stringify(__name), &__name##_proc_ops}

#define SET_VENDOR_GROUP_STORE(__grp, __vg)						      \
		static ssize_t set_task_group_##__grp##_store(struct file *filp, \
			const char __user *ubuf, \
			size_t count, loff_t *pos) \
		{									      \
			char buf[MAX_PROC_SIZE];	\
			int ret;   \
			if (count >= sizeof(buf))	\
				return -EINVAL;	\
			if (copy_from_user(buf, ubuf, count))	\
				return -EFAULT;	\
			buf[count] = '\0';	\
			ret = update_vendor_group_attribute(buf, VTA_TASK_GROUP, __vg);   \
			return ret ?: count;						      \
		}									      \
		PROC_OPS_WO(set_task_group_##__grp);		\
		static ssize_t set_proc_group_##__grp##_store(struct file *filp, \
			const char __user *ubuf, \
			size_t count, loff_t *pos)		\
		{									      \
			char buf[MAX_PROC_SIZE];	\
			int ret;   \
			if (count >= sizeof(buf))	\
				return -EINVAL;	\
			if (copy_from_user(buf, ubuf, count))	\
				return -EFAULT;	\
			buf[count] = '\0';	\
			ret = update_vendor_group_attribute(buf, VTA_PROC_GROUP, __vg);   \
			return ret ?: count;						      \
		}									      \
		PROC_OPS_WO(set_proc_group_##__grp);

#define VENDOR_GROUP_BOOL_ATTRIBUTE(__grp, __attr, __vg)				      \
		static int __grp##_##__attr##_show(struct seq_file *m, void *v) 	\
		{									      \
			struct vendor_group_property *gp = get_vendor_group_property(__vg);   \
			seq_printf(m, "%s\n", gp->__attr==true? "true":"false"); \
			return 0; 	\
		}									      \
		static ssize_t __grp##_##__attr##_store(struct file *filp, \
			const char __user *ubuf, \
			size_t count, loff_t *pos) \
		{									      \
			bool val;							      \
			struct vendor_group_property *gp = get_vendor_group_property(__vg);   \
			char buf[MAX_PROC_SIZE];	\
			if (count >= sizeof(buf))	\
				return -EINVAL;	\
			if (copy_from_user(buf, ubuf, count))	\
				return -EFAULT;	\
			buf[count] = '\0';	\
			if (kstrtobool(buf, &val))					      \
				return -EINVAL;						      \
			gp->__attr = val;						      \
			return count;							      \
		}									      \
		PROC_OPS_RW(__grp##_##__attr);

#define VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(__grp, __attr, __vg, __check_func)		      \
		static int __grp##_##__attr##_show(struct seq_file *m, void *v) 	      \
		{									      \
			struct vendor_group_property *gp = get_vendor_group_property(__vg);   \
			seq_printf(m, "%u\n", gp->__attr);	      \
			return 0;	      \
		}									      \
		static ssize_t __grp##_##__attr##_store(struct file *filp,		      \
			const char __user *ubuf, \
			size_t count, loff_t *pos) \
		{									      \
			unsigned int val, old_val;					      \
			bool (*check_func)(enum vendor_group group) = __check_func;	      \
			struct vendor_group_property *gp = get_vendor_group_property(__vg);   \
			char buf[MAX_PROC_SIZE];	\
			if (count >= sizeof(buf))	\
				return -EINVAL;	\
			if (copy_from_user(buf, ubuf, count))	\
				return -EFAULT;	\
			buf[count] = '\0';	\
			if (kstrtouint(buf, 10, &val))					      \
				return -EINVAL;						      \
			old_val = gp->__attr;					      \
			gp->__attr = val;						      \
			if (check_func && !check_func(__vg)) {				      \
				gp->__attr = old_val;					      \
				return -EINVAL;						      \
			}								      \
			return count;							      \
		}									      \
		PROC_OPS_RW(__grp##_##__attr);

#define VENDOR_GROUP_UINT_ATTRIBUTE(__grp, __attr, __vg)				      \
		VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(__grp, __attr, __vg, NULL)

#define VENDOR_GROUP_CPUMASK_ATTRIBUTE(__grp, __attr, __vg)				      \
		static int __grp##_##__attr##_show(struct seq_file *m, void *v) 	\
		{									      \
			struct vendor_group_property *gp = get_vendor_group_property(__vg);   \
			seq_printf(m, "0x%lx\n", *gp->__attr.bits);	      \
			return 0;	      \
		}									      \
		static ssize_t __grp##_##__attr##_store(struct file *filp,			\
			const char __user *ubuf, \
			size_t count, loff_t *pos) \
		{									      \
			unsigned long val;					              \
			struct vendor_group_property *gp = get_vendor_group_property(__vg);   \
			char buf[MAX_PROC_SIZE];	\
			if (count >= sizeof(buf))	\
				return -EINVAL;	\
			if (copy_from_user(buf, ubuf, count))	\
				return -EFAULT;	\
			buf[count] = '\0';	\
			if (kstrtoul(buf, 0, &val))					      \
				return -EINVAL;						      \
			*gp->__attr.bits = val;						      \
			return count;							      \
		}									      \
		PROC_OPS_RW(__grp##_##__attr);

#define VENDOR_GROUP_UCLAMP_ATTRIBUTE(__grp, __attr, __vg, __cid)			      \
		static int __grp##_##__attr##_show(struct seq_file *m, void *v) 	\
		{									      \
			struct vendor_group_property *gp = get_vendor_group_property(__vg);   \
			seq_printf(m, "%u\n", gp->uc_req[__cid].value);		      \
			return 0;	\
		}									      \
		static ssize_t __grp##_##__attr##_store(struct file *filp,			\
			const char __user *ubuf, \
			size_t count, loff_t *pos) \
		{									      \
			unsigned int val;						      \
			struct vendor_group_property *gp = get_vendor_group_property(__vg);   \
			char buf[MAX_PROC_SIZE];	\
			if (count >= sizeof(buf))	\
				return -EINVAL;	\
			if (copy_from_user(buf, ubuf, count))	\
				return -EFAULT;	\
			buf[count] = '\0';	\
			if (kstrtouint(buf, 0, &val))					      \
				return -EINVAL;						      \
			if (val > 1024)							      \
				return -EINVAL;						      \
			if (val == gp->uc_req[__cid].value)				      \
				return count;						      \
			gp->uc_req[__cid].value = val;					      \
			gp->uc_req[__cid].bucket_id = get_bucket_id(val);		      \
			gp->uc_req[__cid].user_defined = false;				      \
			apply_uclamp_change(__vg, __cid);				      \
			return count;							      \
		}									      \
		PROC_OPS_RW(__grp##_##__attr);

#define PER_TASK_BOOL_ATTRIBUTE(__attr)						      \
		static ssize_t __attr##_set##_store(struct file *filp, \
			const char __user *ubuf, \
			size_t count, loff_t *pos) \
		{									      \
			char buf[MAX_PROC_SIZE];	\
			int ret;	\
			if (count >= sizeof(buf))	\
				return -EINVAL;	\
			if (copy_from_user(buf, ubuf, count))	\
				return -EFAULT;	\
			buf[count] = '\0';	\
			ret = update_##__attr(buf, true);   \
			return ret ?: count;						      \
		}									      \
		PROC_OPS_WO(__attr##_set);	\
		static ssize_t __attr##_clear##_store(struct file *filp, \
			const char __user *ubuf, \
			size_t count, loff_t *pos) \
		{									      \
			char buf[MAX_PROC_SIZE];	\
			int ret;	\
			if (count >= sizeof(buf))	\
				return -EINVAL;	\
			if (copy_from_user(buf, ubuf, count))	\
				return -EFAULT;	\
			buf[count] = '\0';	\
			ret = update_##__attr(buf, false);   \
			return ret ?: count;						      \
		}									      \
		PROC_OPS_WO(__attr##_clear);

#if IS_ENABLED(CONFIG_USE_VENDOR_GROUP_UTIL)
#define UTILIZATION_GROUP_UINT_ATTRIBUTE(__grp, __attr, __ug)				      \
		static int __grp##_##__attr##_show(struct seq_file *m, void *v) 	\
		{									      \
			struct vendor_util_group_property *gp = \
				get_vendor_util_group_property(__ug);   \
			seq_printf(m, "%u\n", gp->__attr);	      \
			return 0;	      \
		}									      \
		static ssize_t __grp##_##__attr##_store(struct file *filp,			\
			const char __user *ubuf, \
			size_t count, loff_t *pos) \
		{									      \
			unsigned int val;					              \
			struct vendor_util_group_property *gp = \
				get_vendor_util_group_property(__ug);   \
			char buf[MAX_PROC_SIZE];	\
			if (count >= sizeof(buf))	\
				return -EINVAL;	\
			if (copy_from_user(buf, ubuf, count))	\
				return -EFAULT;	\
			buf[count] = '\0';	\
			if (kstrtouint(buf, 10, &val))					      \
				return -EINVAL;						      \
			gp->__attr = val;						      \
			return count;							      \
		}									      \
		PROC_OPS_RW(__grp##_##__attr);


#define UTILIZATION_GROUP_UCLAMP_ATTRIBUTE(__grp, __attr, __ug, __cid)			      \
		static int __grp##_##__attr##_show(struct seq_file *m, void *v) 	\
		{									      \
			struct vendor_util_group_property *gp = \
				get_vendor_util_group_property(__ug);   \
			seq_printf(m, "%u\n", gp->uc_req[__cid].value);		      \
			return 0;	\
		}									      \
		static ssize_t __grp##_##__attr##_store(struct file *filp,			\
			const char __user *ubuf, \
			size_t count, loff_t *pos) \
		{									      \
			unsigned int val;						      \
			struct vendor_util_group_property *gp = \
				get_vendor_util_group_property(__ug);   \
			char buf[MAX_PROC_SIZE];	\
			if (count >= sizeof(buf))	\
				return -EINVAL;	\
			if (copy_from_user(buf, ubuf, count))	\
				return -EFAULT;	\
			buf[count] = '\0';	\
			if (kstrtouint(buf, 0, &val))					      \
				return -EINVAL;						      \
			if (val > 1024)							      \
				return -EINVAL;						      \
			gp->uc_req[__cid].value = val;					      \
			return count;							      \
		}									      \
		PROC_OPS_RW(__grp##_##__attr);
#endif

#define UCLAMP_ON_NICE_PRIO_CHECK_FUN(__uclamp_id) \
static inline bool check_uclamp_##__uclamp_id##_on_nice_prio(enum vendor_group group) \
{ \
	if (vg[group].uclamp_##__uclamp_id##_on_nice_mid_prio < \
		vg[group].uclamp_##__uclamp_id##_on_nice_high_prio) \
		return false; \
	if (vg[group].uclamp_##__uclamp_id##_on_nice_low_prio < \
		vg[group].uclamp_##__uclamp_id##_on_nice_mid_prio) \
		return false; \
	if (vg[group].uclamp_##__uclamp_id##_on_nice_low_prio < \
		vg[group].uclamp_##__uclamp_id##_on_nice_high_prio) \
		return false; \
	return true; \
}

/// ******************************************************************************** ///
/// ********************* Create vendor group procfs nodes*************************** ///
/// ******************************************************************************** ///

UCLAMP_ON_NICE_PRIO_CHECK_FUN(min);
UCLAMP_ON_NICE_PRIO_CHECK_FUN(max);

static inline bool check_ug(enum vendor_group group)
{
	if (vg[group].ug < UG_BG || vg[group].ug > UG_AUTO)
		return false;

	return true;
}

VENDOR_GROUP_BOOL_ATTRIBUTE(ta, prefer_idle, VG_TOPAPP);
VENDOR_GROUP_BOOL_ATTRIBUTE(ta, prefer_high_cap, VG_TOPAPP);
VENDOR_GROUP_BOOL_ATTRIBUTE(ta, task_spreading, VG_TOPAPP);
#if !IS_ENABLED(CONFIG_USE_VENDOR_GROUP_UTIL)
VENDOR_GROUP_UINT_ATTRIBUTE(ta, group_throttle, VG_TOPAPP);
#endif
VENDOR_GROUP_CPUMASK_ATTRIBUTE(ta, preferred_idle_mask_low, VG_TOPAPP);
VENDOR_GROUP_CPUMASK_ATTRIBUTE(ta, preferred_idle_mask_mid, VG_TOPAPP);
VENDOR_GROUP_CPUMASK_ATTRIBUTE(ta, preferred_idle_mask_high, VG_TOPAPP);
VENDOR_GROUP_UCLAMP_ATTRIBUTE(ta, uclamp_min, VG_TOPAPP, UCLAMP_MIN);
VENDOR_GROUP_UCLAMP_ATTRIBUTE(ta, uclamp_max, VG_TOPAPP, UCLAMP_MAX);
VENDOR_GROUP_UINT_ATTRIBUTE(ta, uclamp_min_on_nice_low_value, VG_TOPAPP);
VENDOR_GROUP_UINT_ATTRIBUTE(ta, uclamp_min_on_nice_mid_value, VG_TOPAPP);
VENDOR_GROUP_UINT_ATTRIBUTE(ta, uclamp_min_on_nice_high_value, VG_TOPAPP);
VENDOR_GROUP_UINT_ATTRIBUTE(ta, uclamp_max_on_nice_low_value, VG_TOPAPP);
VENDOR_GROUP_UINT_ATTRIBUTE(ta, uclamp_max_on_nice_mid_value, VG_TOPAPP);
VENDOR_GROUP_UINT_ATTRIBUTE(ta, uclamp_max_on_nice_high_value, VG_TOPAPP);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(ta, uclamp_min_on_nice_low_prio, VG_TOPAPP, \
	check_uclamp_min_on_nice_prio);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(ta, uclamp_min_on_nice_mid_prio, VG_TOPAPP, \
	check_uclamp_min_on_nice_prio);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(ta, uclamp_min_on_nice_high_prio, VG_TOPAPP, \
	check_uclamp_min_on_nice_prio);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(ta, uclamp_max_on_nice_low_prio, VG_TOPAPP, \
	check_uclamp_max_on_nice_prio);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(ta, uclamp_max_on_nice_mid_prio, VG_TOPAPP, \
	check_uclamp_max_on_nice_prio);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(ta, uclamp_max_on_nice_high_prio, VG_TOPAPP, \
	check_uclamp_max_on_nice_prio);
VENDOR_GROUP_BOOL_ATTRIBUTE(ta, uclamp_min_on_nice_enable, VG_TOPAPP);
VENDOR_GROUP_BOOL_ATTRIBUTE(ta, uclamp_max_on_nice_enable, VG_TOPAPP);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(ta, ug, VG_TOPAPP, check_ug);

VENDOR_GROUP_BOOL_ATTRIBUTE(fg, prefer_idle, VG_FOREGROUND);
VENDOR_GROUP_BOOL_ATTRIBUTE(fg, prefer_high_cap, VG_FOREGROUND);
VENDOR_GROUP_BOOL_ATTRIBUTE(fg, task_spreading, VG_FOREGROUND);
#if !IS_ENABLED(CONFIG_USE_VENDOR_GROUP_UTIL)
VENDOR_GROUP_UINT_ATTRIBUTE(fg, group_throttle, VG_FOREGROUND);
#endif
VENDOR_GROUP_CPUMASK_ATTRIBUTE(fg, preferred_idle_mask_low, VG_FOREGROUND);
VENDOR_GROUP_CPUMASK_ATTRIBUTE(fg, preferred_idle_mask_mid, VG_FOREGROUND);
VENDOR_GROUP_CPUMASK_ATTRIBUTE(fg, preferred_idle_mask_high, VG_FOREGROUND);
VENDOR_GROUP_UCLAMP_ATTRIBUTE(fg, uclamp_min, VG_FOREGROUND, UCLAMP_MIN);
VENDOR_GROUP_UCLAMP_ATTRIBUTE(fg, uclamp_max, VG_FOREGROUND, UCLAMP_MAX);
VENDOR_GROUP_UINT_ATTRIBUTE(fg, uclamp_min_on_nice_low_value, VG_FOREGROUND);
VENDOR_GROUP_UINT_ATTRIBUTE(fg, uclamp_min_on_nice_mid_value, VG_FOREGROUND);
VENDOR_GROUP_UINT_ATTRIBUTE(fg, uclamp_min_on_nice_high_value, VG_FOREGROUND);
VENDOR_GROUP_UINT_ATTRIBUTE(fg, uclamp_max_on_nice_low_value, VG_FOREGROUND);
VENDOR_GROUP_UINT_ATTRIBUTE(fg, uclamp_max_on_nice_mid_value, VG_FOREGROUND);
VENDOR_GROUP_UINT_ATTRIBUTE(fg, uclamp_max_on_nice_high_value, VG_FOREGROUND);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(fg, uclamp_min_on_nice_low_prio, VG_FOREGROUND, \
	check_uclamp_min_on_nice_prio);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(fg, uclamp_min_on_nice_mid_prio, VG_FOREGROUND, \
	check_uclamp_min_on_nice_prio);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(fg, uclamp_min_on_nice_high_prio, VG_FOREGROUND, \
	check_uclamp_min_on_nice_prio);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(fg, uclamp_max_on_nice_low_prio, VG_FOREGROUND, \
	check_uclamp_max_on_nice_prio);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(fg, uclamp_max_on_nice_mid_prio, VG_FOREGROUND, \
	check_uclamp_max_on_nice_prio);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(fg, uclamp_max_on_nice_high_prio, VG_FOREGROUND, \
	check_uclamp_max_on_nice_prio);
VENDOR_GROUP_BOOL_ATTRIBUTE(fg, uclamp_min_on_nice_enable, VG_FOREGROUND);
VENDOR_GROUP_BOOL_ATTRIBUTE(fg, uclamp_max_on_nice_enable, VG_FOREGROUND);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(fg, ug, VG_FOREGROUND, check_ug);

VENDOR_GROUP_BOOL_ATTRIBUTE(sys, prefer_idle, VG_SYSTEM);
VENDOR_GROUP_BOOL_ATTRIBUTE(sys, prefer_high_cap, VG_SYSTEM);
VENDOR_GROUP_BOOL_ATTRIBUTE(sys, task_spreading, VG_SYSTEM);
#if !IS_ENABLED(CONFIG_USE_VENDOR_GROUP_UTIL)
VENDOR_GROUP_UINT_ATTRIBUTE(sys, group_throttle, VG_SYSTEM);
#endif
VENDOR_GROUP_CPUMASK_ATTRIBUTE(sys, preferred_idle_mask_low, VG_SYSTEM);
VENDOR_GROUP_CPUMASK_ATTRIBUTE(sys, preferred_idle_mask_mid, VG_SYSTEM);
VENDOR_GROUP_CPUMASK_ATTRIBUTE(sys, preferred_idle_mask_high, VG_SYSTEM);
VENDOR_GROUP_UCLAMP_ATTRIBUTE(sys, uclamp_min, VG_SYSTEM, UCLAMP_MIN);
VENDOR_GROUP_UCLAMP_ATTRIBUTE(sys, uclamp_max, VG_SYSTEM, UCLAMP_MAX);
VENDOR_GROUP_UINT_ATTRIBUTE(sys, uclamp_min_on_nice_low_value, VG_SYSTEM);
VENDOR_GROUP_UINT_ATTRIBUTE(sys, uclamp_min_on_nice_mid_value, VG_SYSTEM);
VENDOR_GROUP_UINT_ATTRIBUTE(sys, uclamp_min_on_nice_high_value, VG_SYSTEM);
VENDOR_GROUP_UINT_ATTRIBUTE(sys, uclamp_max_on_nice_low_value, VG_SYSTEM);
VENDOR_GROUP_UINT_ATTRIBUTE(sys, uclamp_max_on_nice_mid_value, VG_SYSTEM);
VENDOR_GROUP_UINT_ATTRIBUTE(sys, uclamp_max_on_nice_high_value, VG_SYSTEM);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(sys, uclamp_min_on_nice_low_prio, VG_SYSTEM, \
	check_uclamp_min_on_nice_prio);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(sys, uclamp_min_on_nice_mid_prio, VG_SYSTEM, \
	check_uclamp_min_on_nice_prio);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(sys, uclamp_min_on_nice_high_prio, VG_SYSTEM, \
	check_uclamp_min_on_nice_prio);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(sys, uclamp_max_on_nice_low_prio, VG_SYSTEM, \
	check_uclamp_max_on_nice_prio);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(sys, uclamp_max_on_nice_mid_prio, VG_SYSTEM, \
	check_uclamp_max_on_nice_prio);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(sys, uclamp_max_on_nice_high_prio, VG_SYSTEM, \
	check_uclamp_max_on_nice_prio);
VENDOR_GROUP_BOOL_ATTRIBUTE(sys, uclamp_min_on_nice_enable, VG_SYSTEM);
VENDOR_GROUP_BOOL_ATTRIBUTE(sys, uclamp_max_on_nice_enable, VG_SYSTEM);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(sys, ug, VG_SYSTEM, check_ug);

VENDOR_GROUP_BOOL_ATTRIBUTE(cam, prefer_idle, VG_CAMERA);
VENDOR_GROUP_BOOL_ATTRIBUTE(cam, prefer_high_cap, VG_CAMERA);
VENDOR_GROUP_BOOL_ATTRIBUTE(cam, task_spreading, VG_CAMERA);
#if !IS_ENABLED(CONFIG_USE_VENDOR_GROUP_UTIL)
VENDOR_GROUP_UINT_ATTRIBUTE(cam, group_throttle, VG_CAMERA);
#endif
VENDOR_GROUP_CPUMASK_ATTRIBUTE(cam, preferred_idle_mask_low, VG_CAMERA);
VENDOR_GROUP_CPUMASK_ATTRIBUTE(cam, preferred_idle_mask_mid, VG_CAMERA);
VENDOR_GROUP_CPUMASK_ATTRIBUTE(cam, preferred_idle_mask_high, VG_CAMERA);
VENDOR_GROUP_UCLAMP_ATTRIBUTE(cam, uclamp_min, VG_CAMERA, UCLAMP_MIN);
VENDOR_GROUP_UCLAMP_ATTRIBUTE(cam, uclamp_max, VG_CAMERA, UCLAMP_MAX);
VENDOR_GROUP_UINT_ATTRIBUTE(cam, uclamp_min_on_nice_low_value, VG_CAMERA);
VENDOR_GROUP_UINT_ATTRIBUTE(cam, uclamp_min_on_nice_mid_value, VG_CAMERA);
VENDOR_GROUP_UINT_ATTRIBUTE(cam, uclamp_min_on_nice_high_value, VG_CAMERA);
VENDOR_GROUP_UINT_ATTRIBUTE(cam, uclamp_max_on_nice_low_value, VG_CAMERA);
VENDOR_GROUP_UINT_ATTRIBUTE(cam, uclamp_max_on_nice_mid_value, VG_CAMERA);
VENDOR_GROUP_UINT_ATTRIBUTE(cam, uclamp_max_on_nice_high_value, VG_CAMERA);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(cam, uclamp_min_on_nice_low_prio, VG_CAMERA, \
	check_uclamp_min_on_nice_prio);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(cam, uclamp_min_on_nice_mid_prio, VG_CAMERA, \
	check_uclamp_min_on_nice_prio);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(cam, uclamp_min_on_nice_high_prio, VG_CAMERA, \
	check_uclamp_min_on_nice_prio);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(cam, uclamp_max_on_nice_low_prio, VG_CAMERA, \
	check_uclamp_max_on_nice_prio);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(cam, uclamp_max_on_nice_mid_prio, VG_CAMERA, \
	check_uclamp_max_on_nice_prio);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(cam, uclamp_max_on_nice_high_prio, VG_CAMERA, \
	check_uclamp_max_on_nice_prio);
VENDOR_GROUP_BOOL_ATTRIBUTE(cam, uclamp_min_on_nice_enable, VG_CAMERA);
VENDOR_GROUP_BOOL_ATTRIBUTE(cam, uclamp_max_on_nice_enable, VG_CAMERA);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(cam, ug, VG_CAMERA, check_ug);

VENDOR_GROUP_BOOL_ATTRIBUTE(cam_power, prefer_idle, VG_CAMERA_POWER);
VENDOR_GROUP_BOOL_ATTRIBUTE(cam_power, prefer_high_cap, VG_CAMERA_POWER);
VENDOR_GROUP_BOOL_ATTRIBUTE(cam_power, task_spreading, VG_CAMERA_POWER);
#if !IS_ENABLED(CONFIG_USE_VENDOR_GROUP_UTIL)
VENDOR_GROUP_UINT_ATTRIBUTE(cam_power, group_throttle, VG_CAMERA_POWER);
#endif
VENDOR_GROUP_CPUMASK_ATTRIBUTE(cam_power, preferred_idle_mask_low, VG_CAMERA_POWER);
VENDOR_GROUP_CPUMASK_ATTRIBUTE(cam_power, preferred_idle_mask_mid, VG_CAMERA_POWER);
VENDOR_GROUP_CPUMASK_ATTRIBUTE(cam_power, preferred_idle_mask_high, VG_CAMERA_POWER);
VENDOR_GROUP_UCLAMP_ATTRIBUTE(cam_power, uclamp_min, VG_CAMERA_POWER, UCLAMP_MIN);
VENDOR_GROUP_UCLAMP_ATTRIBUTE(cam_power, uclamp_max, VG_CAMERA_POWER, UCLAMP_MAX);
VENDOR_GROUP_UINT_ATTRIBUTE(cam_power, uclamp_min_on_nice_low_value, VG_CAMERA_POWER);
VENDOR_GROUP_UINT_ATTRIBUTE(cam_power, uclamp_min_on_nice_mid_value, VG_CAMERA_POWER);
VENDOR_GROUP_UINT_ATTRIBUTE(cam_power, uclamp_min_on_nice_high_value, VG_CAMERA_POWER);
VENDOR_GROUP_UINT_ATTRIBUTE(cam_power, uclamp_max_on_nice_low_value, VG_CAMERA_POWER);
VENDOR_GROUP_UINT_ATTRIBUTE(cam_power, uclamp_max_on_nice_mid_value, VG_CAMERA_POWER);
VENDOR_GROUP_UINT_ATTRIBUTE(cam_power, uclamp_max_on_nice_high_value, VG_CAMERA_POWER);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(cam_power, uclamp_min_on_nice_low_prio, VG_CAMERA_POWER, \
	check_uclamp_min_on_nice_prio);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(cam_power, uclamp_min_on_nice_mid_prio, VG_CAMERA_POWER, \
	check_uclamp_min_on_nice_prio);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(cam_power, uclamp_min_on_nice_high_prio, VG_CAMERA_POWER, \
	check_uclamp_min_on_nice_prio);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(cam_power, uclamp_max_on_nice_low_prio, VG_CAMERA_POWER, \
	check_uclamp_max_on_nice_prio);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(cam_power, uclamp_max_on_nice_mid_prio, VG_CAMERA_POWER, \
	check_uclamp_max_on_nice_prio);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(cam_power, uclamp_max_on_nice_high_prio, VG_CAMERA_POWER, \
	check_uclamp_max_on_nice_prio);
VENDOR_GROUP_BOOL_ATTRIBUTE(cam_power, uclamp_min_on_nice_enable, VG_CAMERA_POWER);
VENDOR_GROUP_BOOL_ATTRIBUTE(cam_power, uclamp_max_on_nice_enable, VG_CAMERA_POWER);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(cam_power, ug, VG_CAMERA_POWER, check_ug);

VENDOR_GROUP_BOOL_ATTRIBUTE(bg, prefer_idle, VG_BACKGROUND);
VENDOR_GROUP_BOOL_ATTRIBUTE(bg, prefer_high_cap, VG_BACKGROUND);
VENDOR_GROUP_BOOL_ATTRIBUTE(bg, task_spreading, VG_BACKGROUND);
#if !IS_ENABLED(CONFIG_USE_VENDOR_GROUP_UTIL)
VENDOR_GROUP_UINT_ATTRIBUTE(bg, group_throttle, VG_BACKGROUND);
#endif
VENDOR_GROUP_CPUMASK_ATTRIBUTE(bg, preferred_idle_mask_low, VG_BACKGROUND);
VENDOR_GROUP_CPUMASK_ATTRIBUTE(bg, preferred_idle_mask_mid, VG_BACKGROUND);
VENDOR_GROUP_CPUMASK_ATTRIBUTE(bg, preferred_idle_mask_high, VG_BACKGROUND);
VENDOR_GROUP_UCLAMP_ATTRIBUTE(bg, uclamp_min, VG_BACKGROUND, UCLAMP_MIN);
VENDOR_GROUP_UCLAMP_ATTRIBUTE(bg, uclamp_max, VG_BACKGROUND, UCLAMP_MAX);
VENDOR_GROUP_UINT_ATTRIBUTE(bg, uclamp_min_on_nice_low_value, VG_BACKGROUND);
VENDOR_GROUP_UINT_ATTRIBUTE(bg, uclamp_min_on_nice_mid_value, VG_BACKGROUND);
VENDOR_GROUP_UINT_ATTRIBUTE(bg, uclamp_min_on_nice_high_value, VG_BACKGROUND);
VENDOR_GROUP_UINT_ATTRIBUTE(bg, uclamp_max_on_nice_low_value, VG_BACKGROUND);
VENDOR_GROUP_UINT_ATTRIBUTE(bg, uclamp_max_on_nice_mid_value, VG_BACKGROUND);
VENDOR_GROUP_UINT_ATTRIBUTE(bg, uclamp_max_on_nice_high_value, VG_BACKGROUND);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(bg, uclamp_min_on_nice_low_prio, VG_BACKGROUND, \
	check_uclamp_min_on_nice_prio);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(bg, uclamp_min_on_nice_mid_prio, VG_BACKGROUND, \
	check_uclamp_min_on_nice_prio);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(bg, uclamp_min_on_nice_high_prio, VG_BACKGROUND, \
	check_uclamp_min_on_nice_prio);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(bg, uclamp_max_on_nice_low_prio, VG_BACKGROUND, \
	check_uclamp_max_on_nice_prio);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(bg, uclamp_max_on_nice_mid_prio, VG_BACKGROUND, \
	check_uclamp_max_on_nice_prio);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(bg, uclamp_max_on_nice_high_prio, VG_BACKGROUND, \
	check_uclamp_max_on_nice_prio);
VENDOR_GROUP_BOOL_ATTRIBUTE(bg, uclamp_min_on_nice_enable, VG_BACKGROUND);
VENDOR_GROUP_BOOL_ATTRIBUTE(bg, uclamp_max_on_nice_enable, VG_BACKGROUND);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(bg, ug, VG_BACKGROUND, check_ug);

VENDOR_GROUP_BOOL_ATTRIBUTE(sysbg, prefer_idle, VG_SYSTEM_BACKGROUND);
VENDOR_GROUP_BOOL_ATTRIBUTE(sysbg, prefer_high_cap, VG_SYSTEM_BACKGROUND);
VENDOR_GROUP_BOOL_ATTRIBUTE(sysbg, task_spreading, VG_SYSTEM_BACKGROUND);
#if !IS_ENABLED(CONFIG_USE_VENDOR_GROUP_UTIL)
VENDOR_GROUP_UINT_ATTRIBUTE(sysbg, group_throttle, VG_SYSTEM_BACKGROUND);
#endif
VENDOR_GROUP_CPUMASK_ATTRIBUTE(sysbg, preferred_idle_mask_low, VG_SYSTEM_BACKGROUND);
VENDOR_GROUP_CPUMASK_ATTRIBUTE(sysbg, preferred_idle_mask_mid, VG_SYSTEM_BACKGROUND);
VENDOR_GROUP_CPUMASK_ATTRIBUTE(sysbg, preferred_idle_mask_high, VG_SYSTEM_BACKGROUND);
VENDOR_GROUP_UCLAMP_ATTRIBUTE(sysbg, uclamp_min, VG_SYSTEM_BACKGROUND, UCLAMP_MIN);
VENDOR_GROUP_UCLAMP_ATTRIBUTE(sysbg, uclamp_max, VG_SYSTEM_BACKGROUND, UCLAMP_MAX);
VENDOR_GROUP_UINT_ATTRIBUTE(sysbg, uclamp_min_on_nice_low_value, VG_SYSTEM_BACKGROUND);
VENDOR_GROUP_UINT_ATTRIBUTE(sysbg, uclamp_min_on_nice_mid_value, VG_SYSTEM_BACKGROUND);
VENDOR_GROUP_UINT_ATTRIBUTE(sysbg, uclamp_min_on_nice_high_value, VG_SYSTEM_BACKGROUND);
VENDOR_GROUP_UINT_ATTRIBUTE(sysbg, uclamp_max_on_nice_low_value, VG_SYSTEM_BACKGROUND);
VENDOR_GROUP_UINT_ATTRIBUTE(sysbg, uclamp_max_on_nice_mid_value, VG_SYSTEM_BACKGROUND);
VENDOR_GROUP_UINT_ATTRIBUTE(sysbg, uclamp_max_on_nice_high_value, VG_SYSTEM_BACKGROUND);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(sysbg, uclamp_min_on_nice_low_prio, VG_SYSTEM_BACKGROUND, \
	check_uclamp_min_on_nice_prio);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(sysbg, uclamp_min_on_nice_mid_prio, VG_SYSTEM_BACKGROUND, \
	check_uclamp_min_on_nice_prio);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(sysbg, uclamp_min_on_nice_high_prio, VG_SYSTEM_BACKGROUND, \
	check_uclamp_min_on_nice_prio);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(sysbg, uclamp_max_on_nice_low_prio, VG_SYSTEM_BACKGROUND, \
	check_uclamp_max_on_nice_prio);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(sysbg, uclamp_max_on_nice_mid_prio, VG_SYSTEM_BACKGROUND, \
	check_uclamp_max_on_nice_prio);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(sysbg, uclamp_max_on_nice_high_prio, VG_SYSTEM_BACKGROUND, \
	check_uclamp_max_on_nice_prio);
VENDOR_GROUP_BOOL_ATTRIBUTE(sysbg, uclamp_min_on_nice_enable, VG_SYSTEM_BACKGROUND);
VENDOR_GROUP_BOOL_ATTRIBUTE(sysbg, uclamp_max_on_nice_enable, VG_SYSTEM_BACKGROUND);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(sysbg, ug, VG_SYSTEM_BACKGROUND, check_ug);

VENDOR_GROUP_BOOL_ATTRIBUTE(nnapi, prefer_idle, VG_NNAPI_HAL);
VENDOR_GROUP_BOOL_ATTRIBUTE(nnapi, prefer_high_cap, VG_NNAPI_HAL);
VENDOR_GROUP_BOOL_ATTRIBUTE(nnapi, task_spreading, VG_NNAPI_HAL);
#if !IS_ENABLED(CONFIG_USE_VENDOR_GROUP_UTIL)
VENDOR_GROUP_UINT_ATTRIBUTE(nnapi, group_throttle, VG_NNAPI_HAL);
#endif
VENDOR_GROUP_CPUMASK_ATTRIBUTE(nnapi, preferred_idle_mask_low, VG_NNAPI_HAL);
VENDOR_GROUP_CPUMASK_ATTRIBUTE(nnapi, preferred_idle_mask_mid, VG_NNAPI_HAL);
VENDOR_GROUP_CPUMASK_ATTRIBUTE(nnapi, preferred_idle_mask_high, VG_NNAPI_HAL);
VENDOR_GROUP_UCLAMP_ATTRIBUTE(nnapi, uclamp_min, VG_NNAPI_HAL, UCLAMP_MIN);
VENDOR_GROUP_UCLAMP_ATTRIBUTE(nnapi, uclamp_max, VG_NNAPI_HAL, UCLAMP_MAX);
VENDOR_GROUP_UINT_ATTRIBUTE(nnapi, uclamp_min_on_nice_low_value, VG_NNAPI_HAL);
VENDOR_GROUP_UINT_ATTRIBUTE(nnapi, uclamp_min_on_nice_mid_value, VG_NNAPI_HAL);
VENDOR_GROUP_UINT_ATTRIBUTE(nnapi, uclamp_min_on_nice_high_value, VG_NNAPI_HAL);
VENDOR_GROUP_UINT_ATTRIBUTE(nnapi, uclamp_max_on_nice_low_value, VG_NNAPI_HAL);
VENDOR_GROUP_UINT_ATTRIBUTE(nnapi, uclamp_max_on_nice_mid_value, VG_NNAPI_HAL);
VENDOR_GROUP_UINT_ATTRIBUTE(nnapi, uclamp_max_on_nice_high_value, VG_NNAPI_HAL);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(nnapi, uclamp_min_on_nice_low_prio, VG_NNAPI_HAL, \
	check_uclamp_min_on_nice_prio);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(nnapi, uclamp_min_on_nice_mid_prio, VG_NNAPI_HAL, \
	check_uclamp_min_on_nice_prio);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(nnapi, uclamp_min_on_nice_high_prio, VG_NNAPI_HAL, \
	check_uclamp_min_on_nice_prio);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(nnapi, uclamp_max_on_nice_low_prio, VG_NNAPI_HAL, \
	check_uclamp_max_on_nice_prio);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(nnapi, uclamp_max_on_nice_mid_prio, VG_NNAPI_HAL, \
	check_uclamp_max_on_nice_prio);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(nnapi, uclamp_max_on_nice_high_prio, VG_NNAPI_HAL, \
	check_uclamp_max_on_nice_prio);
VENDOR_GROUP_BOOL_ATTRIBUTE(nnapi, uclamp_min_on_nice_enable, VG_NNAPI_HAL);
VENDOR_GROUP_BOOL_ATTRIBUTE(nnapi, uclamp_max_on_nice_enable, VG_NNAPI_HAL);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(nnapi, ug, VG_NNAPI_HAL, check_ug);

VENDOR_GROUP_BOOL_ATTRIBUTE(rt, prefer_idle, VG_RT);
VENDOR_GROUP_BOOL_ATTRIBUTE(rt, prefer_high_cap, VG_RT);
VENDOR_GROUP_BOOL_ATTRIBUTE(rt, task_spreading, VG_RT);
#if !IS_ENABLED(CONFIG_USE_VENDOR_GROUP_UTIL)
VENDOR_GROUP_UINT_ATTRIBUTE(rt, group_throttle, VG_RT);
#endif
VENDOR_GROUP_CPUMASK_ATTRIBUTE(rt, preferred_idle_mask_low, VG_RT);
VENDOR_GROUP_CPUMASK_ATTRIBUTE(rt, preferred_idle_mask_mid, VG_RT);
VENDOR_GROUP_CPUMASK_ATTRIBUTE(rt, preferred_idle_mask_high, VG_RT);
VENDOR_GROUP_UCLAMP_ATTRIBUTE(rt, uclamp_min, VG_RT, UCLAMP_MIN);
VENDOR_GROUP_UCLAMP_ATTRIBUTE(rt, uclamp_max, VG_RT, UCLAMP_MAX);
VENDOR_GROUP_UINT_ATTRIBUTE(rt, uclamp_min_on_nice_low_value, VG_RT);
VENDOR_GROUP_UINT_ATTRIBUTE(rt, uclamp_min_on_nice_mid_value, VG_RT);
VENDOR_GROUP_UINT_ATTRIBUTE(rt, uclamp_min_on_nice_high_value, VG_RT);
VENDOR_GROUP_UINT_ATTRIBUTE(rt, uclamp_max_on_nice_low_value, VG_RT);
VENDOR_GROUP_UINT_ATTRIBUTE(rt, uclamp_max_on_nice_mid_value, VG_RT);
VENDOR_GROUP_UINT_ATTRIBUTE(rt, uclamp_max_on_nice_high_value, VG_RT);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(rt, uclamp_min_on_nice_low_prio, VG_RT, \
	check_uclamp_min_on_nice_prio);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(rt, uclamp_min_on_nice_mid_prio, VG_RT, \
	check_uclamp_min_on_nice_prio);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(rt, uclamp_min_on_nice_high_prio, VG_RT, \
	check_uclamp_min_on_nice_prio);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(rt, uclamp_max_on_nice_low_prio, VG_RT, \
	check_uclamp_max_on_nice_prio);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(rt, uclamp_max_on_nice_mid_prio, VG_RT, \
	check_uclamp_max_on_nice_prio);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(rt, uclamp_max_on_nice_high_prio, VG_RT, \
	check_uclamp_max_on_nice_prio);
VENDOR_GROUP_BOOL_ATTRIBUTE(rt, uclamp_min_on_nice_enable, VG_RT);
VENDOR_GROUP_BOOL_ATTRIBUTE(rt, uclamp_max_on_nice_enable, VG_RT);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(rt, ug, VG_RT, check_ug);

VENDOR_GROUP_BOOL_ATTRIBUTE(dex2oat, prefer_idle, VG_DEX2OAT);
VENDOR_GROUP_BOOL_ATTRIBUTE(dex2oat, prefer_high_cap, VG_DEX2OAT);
VENDOR_GROUP_BOOL_ATTRIBUTE(dex2oat, task_spreading, VG_DEX2OAT);
#if !IS_ENABLED(CONFIG_USE_VENDOR_GROUP_UTIL)
VENDOR_GROUP_UINT_ATTRIBUTE(dex2oat, group_throttle, VG_DEX2OAT);
#endif
VENDOR_GROUP_CPUMASK_ATTRIBUTE(dex2oat, preferred_idle_mask_low, VG_DEX2OAT);
VENDOR_GROUP_CPUMASK_ATTRIBUTE(dex2oat, preferred_idle_mask_mid, VG_DEX2OAT);
VENDOR_GROUP_CPUMASK_ATTRIBUTE(dex2oat, preferred_idle_mask_high, VG_DEX2OAT);
VENDOR_GROUP_UCLAMP_ATTRIBUTE(dex2oat, uclamp_min, VG_DEX2OAT, UCLAMP_MIN);
VENDOR_GROUP_UCLAMP_ATTRIBUTE(dex2oat, uclamp_max, VG_DEX2OAT, UCLAMP_MAX);
VENDOR_GROUP_UINT_ATTRIBUTE(dex2oat, uclamp_min_on_nice_low_value, VG_DEX2OAT);
VENDOR_GROUP_UINT_ATTRIBUTE(dex2oat, uclamp_min_on_nice_mid_value, VG_DEX2OAT);
VENDOR_GROUP_UINT_ATTRIBUTE(dex2oat, uclamp_min_on_nice_high_value, VG_DEX2OAT);
VENDOR_GROUP_UINT_ATTRIBUTE(dex2oat, uclamp_max_on_nice_low_value, VG_DEX2OAT);
VENDOR_GROUP_UINT_ATTRIBUTE(dex2oat, uclamp_max_on_nice_mid_value, VG_DEX2OAT);
VENDOR_GROUP_UINT_ATTRIBUTE(dex2oat, uclamp_max_on_nice_high_value, VG_DEX2OAT);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(dex2oat, uclamp_min_on_nice_low_prio, VG_DEX2OAT, \
	check_uclamp_min_on_nice_prio);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(dex2oat, uclamp_min_on_nice_mid_prio, VG_DEX2OAT, \
	check_uclamp_min_on_nice_prio);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(dex2oat, uclamp_min_on_nice_high_prio, VG_DEX2OAT, \
	check_uclamp_min_on_nice_prio);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(dex2oat, uclamp_max_on_nice_low_prio, VG_DEX2OAT, \
	check_uclamp_max_on_nice_prio);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(dex2oat, uclamp_max_on_nice_mid_prio, VG_DEX2OAT, \
	check_uclamp_max_on_nice_prio);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(dex2oat, uclamp_max_on_nice_high_prio, VG_DEX2OAT, \
	check_uclamp_max_on_nice_prio);
VENDOR_GROUP_BOOL_ATTRIBUTE(dex2oat, uclamp_min_on_nice_enable, VG_DEX2OAT);
VENDOR_GROUP_BOOL_ATTRIBUTE(dex2oat, uclamp_max_on_nice_enable, VG_DEX2OAT);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(dex2oat, ug, VG_DEX2OAT, check_ug);

VENDOR_GROUP_BOOL_ATTRIBUTE(ota, prefer_idle, VG_OTA);
VENDOR_GROUP_BOOL_ATTRIBUTE(ota, prefer_high_cap, VG_OTA);
VENDOR_GROUP_BOOL_ATTRIBUTE(ota, task_spreading, VG_OTA);
#if !IS_ENABLED(CONFIG_USE_VENDOR_GROUP_UTIL)
VENDOR_GROUP_UINT_ATTRIBUTE(ota, group_throttle, VG_OTA);
#endif
VENDOR_GROUP_CPUMASK_ATTRIBUTE(ota, preferred_idle_mask_low, VG_OTA);
VENDOR_GROUP_CPUMASK_ATTRIBUTE(ota, preferred_idle_mask_mid, VG_OTA);
VENDOR_GROUP_CPUMASK_ATTRIBUTE(ota, preferred_idle_mask_high, VG_OTA);
VENDOR_GROUP_UCLAMP_ATTRIBUTE(ota, uclamp_min, VG_OTA, UCLAMP_MIN);
VENDOR_GROUP_UCLAMP_ATTRIBUTE(ota, uclamp_max, VG_OTA, UCLAMP_MAX);
VENDOR_GROUP_UINT_ATTRIBUTE(ota, uclamp_min_on_nice_low_value, VG_OTA);
VENDOR_GROUP_UINT_ATTRIBUTE(ota, uclamp_min_on_nice_mid_value, VG_OTA);
VENDOR_GROUP_UINT_ATTRIBUTE(ota, uclamp_min_on_nice_high_value, VG_OTA);
VENDOR_GROUP_UINT_ATTRIBUTE(ota, uclamp_max_on_nice_low_value, VG_OTA);
VENDOR_GROUP_UINT_ATTRIBUTE(ota, uclamp_max_on_nice_mid_value, VG_OTA);
VENDOR_GROUP_UINT_ATTRIBUTE(ota, uclamp_max_on_nice_high_value, VG_OTA);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(ota, uclamp_min_on_nice_low_prio, VG_OTA, \
	check_uclamp_min_on_nice_prio);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(ota, uclamp_min_on_nice_mid_prio, VG_OTA, \
	check_uclamp_min_on_nice_prio);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(ota, uclamp_min_on_nice_high_prio, VG_OTA, \
	check_uclamp_min_on_nice_prio);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(ota, uclamp_max_on_nice_low_prio, VG_OTA, \
	check_uclamp_max_on_nice_prio);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(ota, uclamp_max_on_nice_mid_prio, VG_OTA, \
	check_uclamp_max_on_nice_prio);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(ota, uclamp_max_on_nice_high_prio, VG_OTA, \
	check_uclamp_max_on_nice_prio);
VENDOR_GROUP_BOOL_ATTRIBUTE(ota, uclamp_min_on_nice_enable, VG_OTA);
VENDOR_GROUP_BOOL_ATTRIBUTE(ota, uclamp_max_on_nice_enable, VG_OTA);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(ota, ug, VG_OTA, check_ug);

VENDOR_GROUP_BOOL_ATTRIBUTE(sf, prefer_idle, VG_SF);
VENDOR_GROUP_BOOL_ATTRIBUTE(sf, prefer_high_cap, VG_SF);
VENDOR_GROUP_BOOL_ATTRIBUTE(sf, task_spreading, VG_SF);
#if !IS_ENABLED(CONFIG_USE_VENDOR_GROUP_UTIL)
VENDOR_GROUP_UINT_ATTRIBUTE(sf, group_throttle, VG_SF);
#endif
VENDOR_GROUP_CPUMASK_ATTRIBUTE(sf, preferred_idle_mask_low, VG_SF);
VENDOR_GROUP_CPUMASK_ATTRIBUTE(sf, preferred_idle_mask_mid, VG_SF);
VENDOR_GROUP_CPUMASK_ATTRIBUTE(sf, preferred_idle_mask_high, VG_SF);
VENDOR_GROUP_UCLAMP_ATTRIBUTE(sf, uclamp_min, VG_SF, UCLAMP_MIN);
VENDOR_GROUP_UCLAMP_ATTRIBUTE(sf, uclamp_max, VG_SF, UCLAMP_MAX);
VENDOR_GROUP_UINT_ATTRIBUTE(sf, uclamp_min_on_nice_low_value, VG_SF);
VENDOR_GROUP_UINT_ATTRIBUTE(sf, uclamp_min_on_nice_mid_value, VG_SF);
VENDOR_GROUP_UINT_ATTRIBUTE(sf, uclamp_min_on_nice_high_value, VG_SF);
VENDOR_GROUP_UINT_ATTRIBUTE(sf, uclamp_max_on_nice_low_value, VG_SF);
VENDOR_GROUP_UINT_ATTRIBUTE(sf, uclamp_max_on_nice_mid_value, VG_SF);
VENDOR_GROUP_UINT_ATTRIBUTE(sf, uclamp_max_on_nice_high_value, VG_SF);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(sf, uclamp_min_on_nice_low_prio, VG_SF, \
	check_uclamp_min_on_nice_prio);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(sf, uclamp_min_on_nice_mid_prio, VG_SF, \
	check_uclamp_min_on_nice_prio);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(sf, uclamp_min_on_nice_high_prio, VG_SF, \
	check_uclamp_min_on_nice_prio);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(sf, uclamp_max_on_nice_low_prio, VG_SF, \
	check_uclamp_max_on_nice_prio);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(sf, uclamp_max_on_nice_mid_prio, VG_SF, \
	check_uclamp_max_on_nice_prio);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(sf, uclamp_max_on_nice_high_prio, VG_SF, \
	check_uclamp_max_on_nice_prio);
VENDOR_GROUP_BOOL_ATTRIBUTE(sf, uclamp_min_on_nice_enable, VG_SF);
VENDOR_GROUP_BOOL_ATTRIBUTE(sf, uclamp_max_on_nice_enable, VG_SF);
VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(sf, ug, VG_SF, check_ug);

#if IS_ENABLED(CONFIG_USE_VENDOR_GROUP_UTIL)
#if IS_ENABLED(CONFIG_USE_GROUP_THROTTLE)
UTILIZATION_GROUP_UINT_ATTRIBUTE(ug_fg, group_throttle, UG_FG);
#endif
UTILIZATION_GROUP_UCLAMP_ATTRIBUTE(ug_fg, uclamp_min, UG_FG, UCLAMP_MIN);
UTILIZATION_GROUP_UCLAMP_ATTRIBUTE(ug_fg, uclamp_max, UG_FG, UCLAMP_MAX);

#if IS_ENABLED(CONFIG_USE_GROUP_THROTTLE)
UTILIZATION_GROUP_UINT_ATTRIBUTE(ug_bg, group_throttle, UG_BG);
#endif
UTILIZATION_GROUP_UCLAMP_ATTRIBUTE(ug_bg, uclamp_min, UG_BG, UCLAMP_MIN);
UTILIZATION_GROUP_UCLAMP_ATTRIBUTE(ug_bg, uclamp_max, UG_BG, UCLAMP_MAX);
#endif

/// ******************************************************************************** ///
/// ********************* From upstream code for uclamp **************************** ///
/// ******************************************************************************** ///
/*
 * The effective clamp bucket index of a task depends on, by increasing
 * priority:
 * - the task specific clamp value, when explicitly requested from userspace
 * - the task group effective clamp value, for tasks not either in the root
 *   group or in an autogroup
 * - the system default clamp value, defined by the sysadmin
 */
static inline void
uclamp_update_active(struct task_struct *p, enum uclamp_id clamp_id)
{
	struct rq_flags rf;
	struct rq *rq;

	if (!uclamp_is_used())
		return;

	/*
	 * Lock the task and the rq where the task is (or was) queued.
	 *
	 * We might lock the (previous) rq of a !RUNNABLE task, but that's the
	 * price to pay to safely serialize util_{min,max} updates with
	 * enqueues, dequeues and migration operations.
	 * This is the same locking schema used by __set_cpus_allowed_ptr().
	 */
	rq = task_rq_lock(p, &rf);

	/*
	 * Setting the clamp bucket is serialized by task_rq_lock().
	 * If the task is not yet RUNNABLE and its task_struct is not
	 * affecting a valid clamp bucket, the next time it's enqueued,
	 * it will already see the updated clamp bucket value.
	 */
	if (p->uclamp[clamp_id].active) {
		uclamp_rq_dec_id(rq, p, clamp_id);
		uclamp_rq_inc_id(rq, p, clamp_id);

		if (rq->uclamp_flags & UCLAMP_FLAG_IDLE)
			rq->uclamp_flags &= ~UCLAMP_FLAG_IDLE;
	}

	task_rq_unlock(rq, p, &rf);
}

/// ******************************************************************************** ///
/// ********************* New code section ***************************************** ///
/// ******************************************************************************** ///
static inline bool check_cred(struct task_struct *p)
{
	const struct cred *cred, *tcred;
	bool ret = true;

	cred = current_cred();
	tcred = get_task_cred(p);
	if (!uid_eq(cred->euid, GLOBAL_ROOT_UID) &&
	    !uid_eq(cred->euid, tcred->uid) &&
	    !uid_eq(cred->euid, tcred->suid) &&
	    !ns_capable(tcred->user_ns, CAP_SYS_NICE)) {
		ret = false;
	}
	put_cred(tcred);
	return ret;
}


static int update_sched_capacity_margin(const char *buf, int count)
{
	char *tok, *str1, *str2;
	unsigned int val, tmp[CPU_NUM];
	int index = 0;

	str1 = kstrndup(buf, count, GFP_KERNEL);
	str2 = str1;

	if (!str2)
		return -EINVAL;

	while (1) {
		tok = strsep(&str2, " ");

		if (tok == NULL)
			break;

		if (kstrtouint(tok, 0, &val))
			goto fail;

		if (val > DEF_UTIL_THRESHOLD || val < SCHED_CAPACITY_SCALE)
			goto fail;

		tmp[index] = val;
		index++;

		if (index == CPU_NUM)
			break;
	}

	if (index == 1) {
		for (index = 0; index < CPU_NUM; index++) {
			sched_capacity_margin[index] = tmp[0];
		}
	} else if (index == CLUSTER_NUM) {
		for (index = MIN_CAPACITY_CPU; index < MID_CAPACITY_CPU; index++)
			sched_capacity_margin[index] = tmp[0];

		for (index = MID_CAPACITY_CPU; index < MAX_CAPACITY_CPU; index++)
			sched_capacity_margin[index] = tmp[1];

		for (index = MAX_CAPACITY_CPU; index < CPU_NUM; index++)
			sched_capacity_margin[index] = tmp[2];
	} else if (index == CPU_NUM) {
		memcpy(sched_capacity_margin, tmp, sizeof(sched_capacity_margin));
	} else {
		goto fail;
	}

	kfree(str1);
	return count;
fail:
	kfree(str1);
	return -EINVAL;
}

static int update_sched_dvfs_headroom(const char *buf, int count)
{
	char *tok, *str1, *str2;
	unsigned int val, tmp[CPU_NUM];
	int index = 0;

	str1 = kstrndup(buf, count, GFP_KERNEL);
	str2 = str1;

	if (!str2)
		return -EINVAL;

	while (1) {
		tok = strsep(&str2, " ");

		if (tok == NULL)
			break;

		if (kstrtouint(tok, 0, &val))
			goto fail;

		if (val > DEF_UTIL_THRESHOLD || val < SCHED_CAPACITY_SCALE)
			goto fail;

		tmp[index] = val;
		index++;

		if (index == CPU_NUM)
			break;
	}

	if (index == 1) {
		for (index = 0; index < CPU_NUM; index++) {
			sched_dvfs_headroom[index] = tmp[0];
		}
	} else if (index == CLUSTER_NUM) {
		for (index = MIN_CAPACITY_CPU; index < MID_CAPACITY_CPU; index++)
			sched_dvfs_headroom[index] = tmp[0];

		for (index = MID_CAPACITY_CPU; index < MAX_CAPACITY_CPU; index++)
			sched_dvfs_headroom[index] = tmp[1];

		for (index = MAX_CAPACITY_CPU; index < CPU_NUM; index++)
			sched_dvfs_headroom[index] = tmp[2];
	} else if (index == CPU_NUM) {
		memcpy(sched_dvfs_headroom, tmp, sizeof(sched_dvfs_headroom));
	} else {
		goto fail;
	}

	kfree(str1);
	return count;
fail:
	kfree(str1);
	return -EINVAL;
}

static inline struct task_struct *get_next_task(int group, struct list_head *head)
{
	unsigned long flags;
	struct task_struct *p;
	struct vendor_task_struct *vp;
	struct list_head *cur;

	raw_spin_lock_irqsave(&vendor_group_list[group].lock, flags);

	if (list_empty(head)) {
		vendor_group_list[group].cur_iterator = NULL;
		raw_spin_unlock_irqrestore(&vendor_group_list[group].lock, flags);
		return NULL;
	}

	if (vendor_group_list[group].cur_iterator)
		cur = vendor_group_list[group].cur_iterator;
	else
		cur = head;

	do {
		if (cur->next == head) {
			vendor_group_list[group].cur_iterator = NULL;
			raw_spin_unlock_irqrestore(&vendor_group_list[group].lock, flags);
			return NULL;
		}

		cur = cur->next;
		vp = list_entry(cur, struct vendor_task_struct, node);
		p = __container_of(vp, struct task_struct, android_vendor_data1);
	} while ((!task_on_rq_queued(p) || p->flags & PF_EXITING));

	get_task_struct(p);
	vendor_group_list[group].cur_iterator = cur;

	raw_spin_unlock_irqrestore(&vendor_group_list[group].lock, flags);

	return p;
}

static void apply_uclamp_change(enum vendor_group group, enum uclamp_id clamp_id)
{
	struct task_struct *p;
	unsigned long flags;
	struct list_head *head = &vendor_group_list[group].list;

	if (trace_clock_set_rate_enabled()) {
		char trace_name[32] = {0};
		struct vendor_group_property *gp = get_vendor_group_property(group);
		scnprintf(trace_name, sizeof(trace_name), "%s_grp_%s",
			clamp_id  == UCLAMP_MIN ? "UCLAMP_MIN" : "UCLAMP_MAX", GRP_NAME[group]);
		trace_clock_set_rate(trace_name, gp->uc_req[clamp_id].value,
				raw_smp_processor_id());
	}

	raw_spin_lock_irqsave(&vendor_group_list[group].lock, flags);
	vendor_group_list[group].cur_iterator = NULL;
	raw_spin_unlock_irqrestore(&vendor_group_list[group].lock, flags);

	while ((p = get_next_task(group, head))) {
		uclamp_update_active(p, clamp_id);
		put_task_struct(p);
	}
}

static int update_prefer_idle(const char *buf, bool val)
{
	struct vendor_task_struct *vp;
	struct task_struct *p;
	pid_t pid;

	if (kstrtoint(buf, 0, &pid) || pid <= 0)
		return -EINVAL;

	rcu_read_lock();
	p = find_task_by_vpid(pid);
	if (!p) {
		rcu_read_unlock();
		return -ESRCH;
	}

	get_task_struct(p);

	if (!check_cred(p)) {
		put_task_struct(p);
		rcu_read_unlock();
		return -EACCES;
	}

	vp = get_vendor_task_struct(p);
	vp->prefer_idle = val;

	put_task_struct(p);
	rcu_read_unlock();

	return 0;
}

static int update_uclamp_fork_reset(const char *buf, bool val)
{
	struct vendor_rq_struct *vrq;
	struct vendor_task_struct *vp;
	struct task_struct *p;
	struct rq_flags rf;
	struct rq *rq;
	pid_t pid;

	if (kstrtoint(buf, 0, &pid) || pid <= 0)
		return -EINVAL;

	rcu_read_lock();
	p = find_task_by_vpid(pid);
	if (!p) {
		rcu_read_unlock();
		return -ESRCH;
	}

	get_task_struct(p);

	if (!check_cred(p)) {
		put_task_struct(p);
		rcu_read_unlock();
		return -EACCES;
	}

	vp = get_vendor_task_struct(p);

	rq = task_rq_lock(p, &rf);
	if (task_on_rq_queued(p)) {
		vrq = get_vendor_rq_struct(rq);

		if (!vp->uclamp_fork_reset && val)
			atomic_inc(&vrq->num_adpf_tasks);
		else if (vp->uclamp_fork_reset && !val)
			atomic_dec(&vrq->num_adpf_tasks);
	}

	if (vp->uclamp_fork_reset != val) {
		vp->uclamp_fork_reset = val;

		if (vendor_sched_boost_adpf_prio)
			update_adpf_prio(p, vp, val);
	}
	task_rq_unlock(rq, p, &rf);

	put_task_struct(p);
	rcu_read_unlock();

	return 0;
}

static int update_vendor_group_attribute(const char *buf, enum vendor_group_attribute vta,
					 unsigned int new)
{
	struct vendor_task_struct *vp;
	struct task_struct *p, *t;
	enum uclamp_id clamp_id;
	pid_t pid;
	unsigned long flags;
	int old;

	if (kstrtoint(buf, 0, &pid) || pid <= 0)
		return -EINVAL;

	rcu_read_lock();
	p = find_task_by_vpid(pid);
	if (!p) {
		rcu_read_unlock();
		return -ESRCH;
	}

	get_task_struct(p);

	if (!check_cred(p)) {
		put_task_struct(p);
		rcu_read_unlock();
		return -EACCES;
	}

	switch (vta) {
	case VTA_TASK_GROUP:
		vp = get_vendor_task_struct(p);
		old = vp->group;
		raw_spin_lock_irqsave(&vp->lock, flags);
#if IS_ENABLED(CONFIG_USE_VENDOR_GROUP_UTIL)
		if (p->prio >= MAX_RT_PRIO)
			migrate_vendor_group_util(p, old, new);
#endif
		if (vp->queued_to_list) {
			remove_from_vendor_group_list(&vp->node, old);
			add_to_vendor_group_list(&vp->node, new);
		}
		vp->group = new;
		raw_spin_unlock_irqrestore(&vp->lock, flags);
		for (clamp_id = 0; clamp_id < UCLAMP_CNT; clamp_id++)
			uclamp_update_active(p, clamp_id);
		break;
	case VTA_PROC_GROUP:
		for_each_thread(p, t) {
			get_task_struct(t);
			vp = get_vendor_task_struct(t);
			old = vp->group;
			raw_spin_lock_irqsave(&vp->lock, flags);
			if (vp->queued_to_list) {
				remove_from_vendor_group_list(&vp->node, old);
				add_to_vendor_group_list(&vp->node, new);
			}
			vp->group = new;
			raw_spin_unlock_irqrestore(&vp->lock, flags);
#if IS_ENABLED(CONFIG_USE_VENDOR_GROUP_UTIL)
			if (p->prio >= MAX_RT_PRIO)
				migrate_vendor_group_util(t, old, new);
#endif
			for (clamp_id = 0; clamp_id < UCLAMP_CNT; clamp_id++)
				uclamp_update_active(t, clamp_id);
			put_task_struct(t);
		}
		break;
	default:
		break;
	}

	put_task_struct(p);
	rcu_read_unlock();

	return 0;
}

SET_VENDOR_GROUP_STORE(ta, VG_TOPAPP);
SET_VENDOR_GROUP_STORE(fg, VG_FOREGROUND);
// VG_SYSTEM is default setting so set to VG_SYSTEM is essentially clear vendor group
SET_VENDOR_GROUP_STORE(sys, VG_SYSTEM);
SET_VENDOR_GROUP_STORE(cam, VG_CAMERA);
SET_VENDOR_GROUP_STORE(cam_power, VG_CAMERA_POWER);
SET_VENDOR_GROUP_STORE(bg, VG_BACKGROUND);
SET_VENDOR_GROUP_STORE(sysbg, VG_SYSTEM_BACKGROUND);
SET_VENDOR_GROUP_STORE(nnapi, VG_NNAPI_HAL);
SET_VENDOR_GROUP_STORE(rt, VG_RT);
SET_VENDOR_GROUP_STORE(dex2oat, VG_DEX2OAT);
SET_VENDOR_GROUP_STORE(ota, VG_OTA);
SET_VENDOR_GROUP_STORE(sf, VG_SF);

// Create per-task attribute nodes
PER_TASK_BOOL_ATTRIBUTE(prefer_idle);
PER_TASK_BOOL_ATTRIBUTE(uclamp_fork_reset);

static int dump_task_show(struct seq_file *m, void *v)
{									      \
	struct task_struct *p, *t;
	struct vendor_task_struct *vp;
	int pid;
	unsigned int uclamp_min, uclamp_max, uclamp_eff_min, uclamp_eff_max;
	enum vendor_group group;
	const char *grp_name = "unknown";
	bool uclamp_fork_reset;
	bool prefer_idle;

	rcu_read_lock();

	for_each_process_thread(p, t) {
		get_task_struct(t);
		vp = get_vendor_task_struct(t);
		group = vp->group;
		if (group >= 0 && group < VG_MAX)
			grp_name = GRP_NAME[group];
		uclamp_min = t->uclamp_req[UCLAMP_MIN].value;
		uclamp_max = t->uclamp_req[UCLAMP_MAX].value;
		uclamp_eff_min = uclamp_eff_value(t, UCLAMP_MIN);
		uclamp_eff_max = uclamp_eff_value(t, UCLAMP_MAX);
		pid = t->pid;
		uclamp_fork_reset = vp->uclamp_fork_reset;
		prefer_idle = vp->prefer_idle;
		put_task_struct(t);
		seq_printf(m, "%u %s %u %u %u %u %d %d\n", pid, grp_name, uclamp_min, uclamp_max,
			uclamp_eff_min, uclamp_eff_max, uclamp_fork_reset, prefer_idle);
	}

	rcu_read_unlock();

	return 0;
}

PROC_OPS_RO(dump_task);

static int util_threshold_show(struct seq_file *m, void *v)
{
	int i;

	for (i = 0; i < CPU_NUM; i++) {
		seq_printf(m, "%u ", sched_capacity_margin[i]);
	}

	seq_printf(m, "\n");

	return 0;
}

static ssize_t util_threshold_store(struct file *filp,
							const char __user *ubuf,
							size_t count, loff_t *pos)
{
	char buf[MAX_PROC_SIZE];

	if (count >= sizeof(buf))
		return -EINVAL;

	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;

	buf[count] = '\0';

	return update_sched_capacity_margin(buf, count);
}

PROC_OPS_RW(util_threshold);

static int dvfs_headroom_show(struct seq_file *m, void *v)
{
	int i;

	for (i = 0; i < CPU_NUM; i++) {
		seq_printf(m, "%u ", sched_dvfs_headroom[i]);
	}

	seq_printf(m, "\n");

	return 0;
}
static ssize_t dvfs_headroom_store(struct file *filp,
				  const char __user *ubuf,
				  size_t count, loff_t *pos)
{
	char buf[MAX_PROC_SIZE];

	if (count >= sizeof(buf))
		return -EINVAL;

	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;

	buf[count] = '\0';

	return update_sched_dvfs_headroom(buf, count);
}
PROC_OPS_RW(dvfs_headroom);

static int tapered_dvfs_headroom_enable_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", static_branch_likely(&tapered_dvfs_headroom_enable) ? 1 : 0);
	return 0;
}
static ssize_t tapered_dvfs_headroom_enable_store(struct file *filp,
						  const char __user *ubuf,
						  size_t count, loff_t *pos)
{
	int enable = 0;
	char buf[MAX_PROC_SIZE];

	if (count >= sizeof(buf))
		return -EINVAL;

	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;

	buf[count] = '\0';

	if (kstrtoint(buf, 10, &enable))
		return -EINVAL;

	if (enable)
		static_branch_enable(&tapered_dvfs_headroom_enable);
	else
		static_branch_disable(&tapered_dvfs_headroom_enable);

	return count;
}
PROC_OPS_RW(tapered_dvfs_headroom_enable);

static int npi_packing_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", vendor_sched_npi_packing ? "true" : "false");

	return 0;
}

static ssize_t npi_packing_store(struct file *filp,
							const char __user *ubuf,
							size_t count, loff_t *pos)
{
	bool enable;
	char buf[MAX_PROC_SIZE];

	if (count >= sizeof(buf))
		return -EINVAL;

	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;

	buf[count] = '\0';

	if (kstrtobool(buf, &enable))
		return -EINVAL;

	vendor_sched_npi_packing = enable;

	return count;
}

PROC_OPS_RW(npi_packing);

static int idle_balancer_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", vendor_sched_idle_balancer ? "true" : "false");

	return 0;
}

static ssize_t idle_balancer_store(struct file *filp,
							const char __user *ubuf,
							size_t count, loff_t *pos)
{
	bool enable;
	char buf[MAX_PROC_SIZE];

	if (count >= sizeof(buf))
		return -EINVAL;

	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;

	buf[count] = '\0';

	if (kstrtobool(buf, &enable))
		return -EINVAL;

	vendor_sched_idle_balancer = enable;

	return count;
}

PROC_OPS_RW(idle_balancer);

static int reduce_prefer_idle_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", vendor_sched_reduce_prefer_idle ? "true" : "false");

	return 0;
}

static ssize_t reduce_prefer_idle_store(struct file *filp, const char __user *ubuf,
					size_t count, loff_t *pos)
{
	bool enable;
	char buf[MAX_PROC_SIZE];

	if (count >= sizeof(buf))
		return -EINVAL;

	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;

	buf[count] = '\0';

	if (kstrtobool(buf, &enable))
		return -EINVAL;

	vendor_sched_reduce_prefer_idle = enable;

	return count;
}

PROC_OPS_RW(reduce_prefer_idle);

static int boost_adpf_prio_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", vendor_sched_boost_adpf_prio ? "true" : "false");

	return 0;
}

static ssize_t boost_adpf_prio_store(struct file *filp, const char __user *ubuf,
				     size_t count, loff_t *pos)
{
	bool enable;
	int err;

	err = kstrtobool_from_user(ubuf, count, &enable);

	if (err)
		return err;

	vendor_sched_boost_adpf_prio = enable;

	return count;
}

PROC_OPS_RW(boost_adpf_prio);

#if IS_ENABLED(CONFIG_UCLAMP_STATS)
static int uclamp_stats_show(struct seq_file *m, void *v)
{
	int i, j, index;
	struct uclamp_stats *stats;

	seq_printf(m, "V, T(ms), %%\n");
	for (i = 0; i < CONFIG_VH_SCHED_CPU_NR; i++) {
		stats = &per_cpu(uclamp_stats, i);
		seq_printf(m, "CPU %d - total time: %llu ms\n", i, stats->total_time \
		/ NSEC_PER_MSEC);
		seq_printf(m, "uclamp.min\n");

		for (j = 0, index = 0; j < UCLAMP_STATS_SLOTS; j++, index += UCLAMP_STATS_STEP) {
			seq_printf(m, "%d, %llu, %llu%%\n", index,
					stats->time_in_state_min[j] / NSEC_PER_MSEC,
					stats->time_in_state_min[j] / (stats->total_time / 100));
		}

		seq_printf(m, "uclamp.max\n");

		for (j = 0, index = 0; j < UCLAMP_STATS_SLOTS; j++, index += UCLAMP_STATS_STEP) {
			seq_printf(m, "%d, %llu, %llu%%\n", index,
					stats->time_in_state_max[j] / NSEC_PER_MSEC,
					stats->time_in_state_max[j] / (stats->total_time / 100));
		}
	}

	return 0;
}

PROC_OPS_RO(uclamp_stats);

static int uclamp_effective_stats_show(struct seq_file *m, void *v)
{
	int i, j, index;
	struct uclamp_stats *stats;

	seq_printf(m, "V, T(ms), %%(Based on T in uclamp_stats)\n");
	for (i = 0; i < CONFIG_VH_SCHED_CPU_NR; i++) {
		stats = &per_cpu(uclamp_stats, i);

		seq_printf(m, "CPU %d\n", i);
		seq_printf(m, "uclamp.min\n");
		for (j = 0, index = 0; j < UCLAMP_STATS_SLOTS; j++, index += UCLAMP_STATS_STEP) {
			seq_printf(m, "%d, %llu, %llu%%\n", index,
					stats->effect_time_in_state_min[j] / NSEC_PER_MSEC,
					stats->effect_time_in_state_min[j] /
					(stats->time_in_state_min[j] / 100));
		}

		seq_printf(m, "uclamp.max\n");
		for (j = 0, index = 0; j < UCLAMP_STATS_SLOTS; j++, index += UCLAMP_STATS_STEP) {
			seq_printf(m, "%d, %llu, %llu%%\n", index,
					stats->effect_time_in_state_max[j] / NSEC_PER_MSEC,
					stats->effect_time_in_state_max[j] /
					(stats->time_in_state_max[j] / 100));
		}
	}

	return 0;
}

PROC_OPS_RO(uclamp_effective_stats);

static int uclamp_util_diff_stats_show(struct seq_file *m, void *v)
{
	int i, j, index;
	struct uclamp_stats *stats;

	seq_printf(m, "V, T(ms), %%\n");
	for (i = 0; i < CONFIG_VH_SCHED_CPU_NR; i++) {
		stats = &per_cpu(uclamp_stats, i);
		seq_printf(m, "CPU %d - total time: %llu ms\n",
				 i, stats->total_time / NSEC_PER_MSEC);
		seq_printf(m, "util_diff_min\n");
		for (j = 0, index = 0; j < UCLAMP_STATS_SLOTS; j++, index += UCLAMP_STATS_STEP) {
			seq_printf(m, "%d, %llu, %llu%%\n", index,
					stats->util_diff_min[j] / NSEC_PER_MSEC,
					stats->util_diff_min[j] / (stats->total_time / 100));
		}

		seq_printf(m, "util_diff_max\n");
		for (j = 0, index = 0; j < UCLAMP_STATS_SLOTS; j++, index -= UCLAMP_STATS_STEP) {
			seq_printf(m, "%d, %llu, %llu%%\n", index,
					stats->util_diff_max[j] / NSEC_PER_MSEC,
					stats->util_diff_max[j] / (stats->total_time / 100));
		}
	}

	return 0;
}

PROC_OPS_RO(uclamp_util_diff_stats);


static ssize_t reset_uclamp_stats_store(struct file *filp,
							const char __user *ubuf,
							size_t count, loff_t *pos)
{
	bool reset;
	char buf[MAX_PROC_SIZE];

	if (count >= sizeof(buf))
		return -EINVAL;

	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;

	buf[count] = '\0';

	if (kstrtobool(buf, &reset))
		return -EINVAL;

	if (reset)
		reset_uclamp_stats();

	return count;
}

PROC_OPS_WO(reset_uclamp_stats);
#endif

static int uclamp_max_filter_rt_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", sysctl_sched_uclamp_max_filter_rt);
	return 0;
}
static ssize_t uclamp_max_filter_rt_store(struct file *filp,
					  const char __user *ubuf,
					  size_t count, loff_t *pos)
{
	int val = 0;
	char buf[MAX_PROC_SIZE];

	if (count >= sizeof(buf))
		return -EINVAL;

	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;

	buf[count] = '\0';

	if (kstrtoint(buf, 10, &val))
		return -EINVAL;

	sysctl_sched_uclamp_max_filter_rt = val;
	return count;
}
PROC_OPS_RW(uclamp_max_filter_rt);

static int util_post_init_scale_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", vendor_sched_util_post_init_scale);
	return 0;
}

static ssize_t util_post_init_scale_store(struct file *filp,
							const char __user *ubuf,
							size_t count, loff_t *pos)
{
	unsigned int val;
	char buf[MAX_PROC_SIZE];

	if (count >= sizeof(buf))
		return -EINVAL;

	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;

	buf[count] = '\0';

	if (kstrtouint(buf, 0, &val))
		return -EINVAL;

	if (val > 1024)
		return -EINVAL;

	vendor_sched_util_post_init_scale = val;

	return count;
}

PROC_OPS_RW(util_post_init_scale);

static int pmu_poll_time_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%u\n", pmu_poll_time_ms);
	return 0;
}

static ssize_t pmu_poll_time_store(struct file *filp,
							const char __user *ubuf,
							size_t count, loff_t *pos)
{
	unsigned int val;
	char buf[MAX_PROC_SIZE];

	if (count >= sizeof(buf))
		return -EINVAL;

	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;

	buf[count] = '\0';

	if (kstrtouint(buf, 0, &val))
		return -EINVAL;

	if (val < 10 || val > 1000000)
		return -EINVAL;

	pmu_poll_time_ms = val;

	return count;
}

PROC_OPS_RW(pmu_poll_time);

static  pmu_poll_enable_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", pmu_poll_enabled ? "true" : "false");
	return 0;
}

static ssize_t pmu_poll_enable_store(struct file *filp,
							const char __user *ubuf,
							size_t count, loff_t *pos)
{
	bool enable;
	char buf[MAX_PROC_SIZE];
	int ret = 0;

	if (count >= sizeof(buf))
		return -EINVAL;

	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;

	buf[count] = '\0';

	if (kstrtobool(buf, &enable))
		return -EINVAL;

	if (enable)
		ret = pmu_poll_enable();
	else
		pmu_poll_disable();

	if (ret)
		return ret;

	return count;
}

PROC_OPS_RW(pmu_poll_enable);


extern unsigned int sched_lib_cpu_freq_cached_val;

static sched_lib_cpu_freq_cached_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%u\n", sched_lib_cpu_freq_cached_val);
	return 0;
}

static ssize_t sched_lib_cpu_freq_cached_store(struct file *filp,
					const char __user *ubuf,
					size_t count, loff_t *pos)
{
	int dup_sched_lib_cpu_freq_cached_val = 0;
	char buf[MAX_PROC_SIZE];

	if (count >= sizeof(buf))
		return -EINVAL;

	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;

	buf[count] = '\0';

	if (kstrtoint(buf, 10, &dup_sched_lib_cpu_freq_cached_val))
		return -EINVAL;

	sched_lib_cpu_freq_cached_val = dup_sched_lib_cpu_freq_cached_val;
	return count;

}

PROC_OPS_RW(sched_lib_cpu_freq_cached);

extern unsigned int sched_lib_freq_cpumask;
static sched_lib_freq_cpumask_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", sched_lib_freq_cpumask);
	return 0;
}

static ssize_t sched_lib_freq_cpumask_store(struct file *filp,
							const char __user *ubuf,
							size_t count, loff_t *pos)
{
	int dup_sched_lib_freq_cpumask = 0;
	char buf[MAX_PROC_SIZE];

	if (count >= sizeof(buf))
		return -EINVAL;

	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;

	buf[count] = '\0';

	if (kstrtoint(buf, 10, &dup_sched_lib_freq_cpumask))
		return -EINVAL;

	sched_lib_freq_cpumask = dup_sched_lib_freq_cpumask;
	return count;
}

PROC_OPS_RW(sched_lib_freq_cpumask);

extern unsigned int sched_lib_affinity_val;
static sched_lib_affinity_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", sched_lib_affinity_val);
	return 0;
}

static ssize_t sched_lib_affinity_store(struct file *filp,
							const char __user *ubuf,
							size_t count, loff_t *pos)
{
	int dup_sched_lib_affinity_val = 0;
	char buf[MAX_PROC_SIZE];

	if (count >= sizeof(buf))
		return -EINVAL;

	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;

	buf[count] = '\0';

	if (kstrtoint(buf, 10, &dup_sched_lib_affinity_val))
		return -EINVAL;

	sched_lib_affinity_val = dup_sched_lib_affinity_val;
	return count;
}

PROC_OPS_RW(sched_lib_affinity);

extern ssize_t sched_lib_name_store(struct file *filp,
				const char __user *ubuffer, size_t count,
				loff_t *ppos);
extern sched_lib_name_show(struct seq_file *m, void *v);


PROC_OPS_RW(sched_lib_name);

/* uclamp filters controls */
static uclamp_min_filter_enable_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", static_branch_likely(&uclamp_min_filter_enable) ? 1 : 0);
	return 0;
}
static ssize_t uclamp_min_filter_enable_store(struct file *filp,
					      const char __user *ubuf,
					      size_t count, loff_t *pos)
{
	int enable = 0;
	char buf[MAX_PROC_SIZE];

	if (count >= sizeof(buf))
		return -EINVAL;

	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;

	buf[count] = '\0';

	if (kstrtoint(buf, 10, &enable))
		return -EINVAL;

	if (enable)
		static_branch_enable(&uclamp_min_filter_enable);
	else
		static_branch_disable(&uclamp_min_filter_enable);

	return count;
}
PROC_OPS_RW(uclamp_min_filter_enable);

static uclamp_min_filter_us_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", sysctl_sched_uclamp_min_filter_us);
	return 0;
}
static ssize_t uclamp_min_filter_us_store(struct file *filp,
					  const char __user *ubuf,
					  size_t count, loff_t *pos)
{
	int val = 0;
	char buf[MAX_PROC_SIZE];

	if (count >= sizeof(buf))
		return -EINVAL;

	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;

	buf[count] = '\0';

	if (kstrtoint(buf, 10, &val))
		return -EINVAL;

	sysctl_sched_uclamp_min_filter_us = val;
	return count;
}
PROC_OPS_RW(uclamp_min_filter_us);

static int uclamp_min_filter_rt_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", sysctl_sched_uclamp_min_filter_rt);
	return 0;
}
static ssize_t uclamp_min_filter_rt_store(struct file *filp,
					  const char __user *ubuf,
					  size_t count, loff_t *pos)
{
	int val = 0;
	char buf[MAX_PROC_SIZE];

	if (count >= sizeof(buf))
		return -EINVAL;

	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;

	buf[count] = '\0';

	if (kstrtoint(buf, 10, &val))
		return -EINVAL;

	sysctl_sched_uclamp_min_filter_rt = val;
	return count;
}
PROC_OPS_RW(uclamp_min_filter_rt);

static uclamp_max_filter_enable_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", static_branch_likely(&uclamp_max_filter_enable) ? 1 : 0);
	return 0;
}
static ssize_t uclamp_max_filter_enable_store(struct file *filp,
					      const char __user *ubuf,
					      size_t count, loff_t *pos)
{
	int enable = 0;
	char buf[MAX_PROC_SIZE];

	if (count >= sizeof(buf))
		return -EINVAL;

	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;

	buf[count] = '\0';

	if (kstrtoint(buf, 10, &enable))
		return -EINVAL;

	if (enable)
		static_branch_enable(&uclamp_max_filter_enable);
	else
		static_branch_disable(&uclamp_max_filter_enable);

	return count;
}
PROC_OPS_RW(uclamp_max_filter_enable);

static uclamp_max_filter_divider_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", sysctl_sched_uclamp_max_filter_divider);
	return 0;
}
static ssize_t uclamp_max_filter_divider_store(struct file *filp,
					       const char __user *ubuf,
					       size_t count, loff_t *pos)
{
	int val = 0;
	char buf[MAX_PROC_SIZE];

	if (count >= sizeof(buf))
		return -EINVAL;

	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;

	buf[count] = '\0';

	if (kstrtoint(buf, 10, &val))
		return -EINVAL;

	sysctl_sched_uclamp_max_filter_divider = val;
	return count;
}
PROC_OPS_RW(uclamp_max_filter_divider);


#if IS_ENABLED(CONFIG_USE_VENDOR_GROUP_UTIL)
static int ug_bg_auto_prio_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", vendor_sched_ug_bg_auto_prio);
	return 0;
}

static ssize_t ug_bg_auto_prio_store(struct file *filp, const char __user *ubuf,
				     size_t count, loff_t *pos)
{
	int val, err;

	err = kstrtoint_from_user(ubuf, count, 0, &val);

	if (err)
		return err;

	if (val < MAX_RT_PRIO || val >= MAX_PRIO)
		return -EINVAL;

	vendor_sched_ug_bg_auto_prio = val;

	return count;
}

PROC_OPS_RW(ug_bg_auto_prio);
#endif

struct pentry {
	const char *name;
	const struct proc_ops *fops;
};
static struct pentry entries[] = {
	// Topapp group attributes
	PROC_ENTRY(ta_prefer_idle),
	PROC_ENTRY(ta_prefer_high_cap),
	PROC_ENTRY(ta_task_spreading),
#if !IS_ENABLED(CONFIG_USE_VENDOR_GROUP_UTIL)
	PROC_ENTRY(ta_group_throttle),
#endif
	PROC_ENTRY(ta_preferred_idle_mask_low),
	PROC_ENTRY(ta_preferred_idle_mask_mid),
	PROC_ENTRY(ta_preferred_idle_mask_high),
	PROC_ENTRY(ta_uclamp_min),
	PROC_ENTRY(ta_uclamp_max),
	PROC_ENTRY(ta_uclamp_min_on_nice_low_value),
	PROC_ENTRY(ta_uclamp_min_on_nice_mid_value),
	PROC_ENTRY(ta_uclamp_min_on_nice_high_value),
	PROC_ENTRY(ta_uclamp_max_on_nice_low_value),
	PROC_ENTRY(ta_uclamp_max_on_nice_mid_value),
	PROC_ENTRY(ta_uclamp_max_on_nice_high_value),
	PROC_ENTRY(ta_uclamp_min_on_nice_low_prio),
	PROC_ENTRY(ta_uclamp_min_on_nice_mid_prio),
	PROC_ENTRY(ta_uclamp_min_on_nice_high_prio),
	PROC_ENTRY(ta_uclamp_max_on_nice_low_prio),
	PROC_ENTRY(ta_uclamp_max_on_nice_mid_prio),
	PROC_ENTRY(ta_uclamp_max_on_nice_high_prio),
	PROC_ENTRY(ta_uclamp_min_on_nice_enable),
	PROC_ENTRY(ta_uclamp_max_on_nice_enable),
	PROC_ENTRY(ta_ug),
	// Foreground group attributes
	PROC_ENTRY(fg_prefer_idle),
	PROC_ENTRY(fg_prefer_high_cap),
	PROC_ENTRY(fg_task_spreading),
#if !IS_ENABLED(CONFIG_USE_VENDOR_GROUP_UTIL)
	PROC_ENTRY(fg_group_throttle),
#endif
	PROC_ENTRY(fg_preferred_idle_mask_low),
	PROC_ENTRY(fg_preferred_idle_mask_mid),
	PROC_ENTRY(fg_preferred_idle_mask_high),
	PROC_ENTRY(fg_uclamp_min),
	PROC_ENTRY(fg_uclamp_max),
	PROC_ENTRY(fg_uclamp_min_on_nice_low_value),
	PROC_ENTRY(fg_uclamp_min_on_nice_mid_value),
	PROC_ENTRY(fg_uclamp_min_on_nice_high_value),
	PROC_ENTRY(fg_uclamp_max_on_nice_low_value),
	PROC_ENTRY(fg_uclamp_max_on_nice_mid_value),
	PROC_ENTRY(fg_uclamp_max_on_nice_high_value),
	PROC_ENTRY(fg_uclamp_min_on_nice_low_prio),
	PROC_ENTRY(fg_uclamp_min_on_nice_mid_prio),
	PROC_ENTRY(fg_uclamp_min_on_nice_high_prio),
	PROC_ENTRY(fg_uclamp_max_on_nice_low_prio),
	PROC_ENTRY(fg_uclamp_max_on_nice_mid_prio),
	PROC_ENTRY(fg_uclamp_max_on_nice_high_prio),
	PROC_ENTRY(fg_uclamp_min_on_nice_enable),
	PROC_ENTRY(fg_uclamp_max_on_nice_enable),
	PROC_ENTRY(fg_ug),
	// System group attributes
	PROC_ENTRY(sys_prefer_idle),
	PROC_ENTRY(sys_prefer_high_cap),
	PROC_ENTRY(sys_task_spreading),
#if !IS_ENABLED(CONFIG_USE_VENDOR_GROUP_UTIL)
	PROC_ENTRY(sys_group_throttle),
#endif
	PROC_ENTRY(sys_preferred_idle_mask_low),
	PROC_ENTRY(sys_preferred_idle_mask_mid),
	PROC_ENTRY(sys_preferred_idle_mask_high),
	PROC_ENTRY(sys_uclamp_min),
	PROC_ENTRY(sys_uclamp_max),
	PROC_ENTRY(sys_uclamp_min_on_nice_low_value),
	PROC_ENTRY(sys_uclamp_min_on_nice_mid_value),
	PROC_ENTRY(sys_uclamp_min_on_nice_high_value),
	PROC_ENTRY(sys_uclamp_max_on_nice_low_value),
	PROC_ENTRY(sys_uclamp_max_on_nice_mid_value),
	PROC_ENTRY(sys_uclamp_max_on_nice_high_value),
	PROC_ENTRY(sys_uclamp_min_on_nice_low_prio),
	PROC_ENTRY(sys_uclamp_min_on_nice_mid_prio),
	PROC_ENTRY(sys_uclamp_min_on_nice_high_prio),
	PROC_ENTRY(sys_uclamp_max_on_nice_low_prio),
	PROC_ENTRY(sys_uclamp_max_on_nice_mid_prio),
	PROC_ENTRY(sys_uclamp_max_on_nice_high_prio),
	PROC_ENTRY(sys_uclamp_min_on_nice_enable),
	PROC_ENTRY(sys_uclamp_max_on_nice_enable),
	PROC_ENTRY(sys_ug),
	// Camera group attributes
	PROC_ENTRY(cam_prefer_idle),
	PROC_ENTRY(cam_prefer_high_cap),
	PROC_ENTRY(cam_task_spreading),
#if !IS_ENABLED(CONFIG_USE_VENDOR_GROUP_UTIL)
	PROC_ENTRY(cam_group_throttle),
#endif
	PROC_ENTRY(cam_preferred_idle_mask_low),
	PROC_ENTRY(cam_preferred_idle_mask_mid),
	PROC_ENTRY(cam_preferred_idle_mask_high),
	PROC_ENTRY(cam_uclamp_min),
	PROC_ENTRY(cam_uclamp_max),
	PROC_ENTRY(cam_uclamp_min_on_nice_low_value),
	PROC_ENTRY(cam_uclamp_min_on_nice_mid_value),
	PROC_ENTRY(cam_uclamp_min_on_nice_high_value),
	PROC_ENTRY(cam_uclamp_max_on_nice_low_value),
	PROC_ENTRY(cam_uclamp_max_on_nice_mid_value),
	PROC_ENTRY(cam_uclamp_max_on_nice_high_value),
	PROC_ENTRY(cam_uclamp_min_on_nice_low_prio),
	PROC_ENTRY(cam_uclamp_min_on_nice_mid_prio),
	PROC_ENTRY(cam_uclamp_min_on_nice_high_prio),
	PROC_ENTRY(cam_uclamp_max_on_nice_low_prio),
	PROC_ENTRY(cam_uclamp_max_on_nice_mid_prio),
	PROC_ENTRY(cam_uclamp_max_on_nice_high_prio),
	PROC_ENTRY(cam_uclamp_min_on_nice_enable),
	PROC_ENTRY(cam_uclamp_max_on_nice_enable),
	PROC_ENTRY(cam_ug),
	// Camera_power group attributes
	PROC_ENTRY(cam_power_prefer_idle),
	PROC_ENTRY(cam_power_prefer_high_cap),
	PROC_ENTRY(cam_power_task_spreading),
#if !IS_ENABLED(CONFIG_USE_VENDOR_GROUP_UTIL)
	PROC_ENTRY(cam_power_group_throttle),
#endif
	PROC_ENTRY(cam_power_preferred_idle_mask_low),
	PROC_ENTRY(cam_power_preferred_idle_mask_mid),
	PROC_ENTRY(cam_power_preferred_idle_mask_high),
	PROC_ENTRY(cam_power_uclamp_min),
	PROC_ENTRY(cam_power_uclamp_max),
	PROC_ENTRY(cam_power_uclamp_min_on_nice_low_value),
	PROC_ENTRY(cam_power_uclamp_min_on_nice_mid_value),
	PROC_ENTRY(cam_power_uclamp_min_on_nice_high_value),
	PROC_ENTRY(cam_power_uclamp_max_on_nice_low_value),
	PROC_ENTRY(cam_power_uclamp_max_on_nice_mid_value),
	PROC_ENTRY(cam_power_uclamp_max_on_nice_high_value),
	PROC_ENTRY(cam_power_uclamp_min_on_nice_low_prio),
	PROC_ENTRY(cam_power_uclamp_min_on_nice_mid_prio),
	PROC_ENTRY(cam_power_uclamp_min_on_nice_high_prio),
	PROC_ENTRY(cam_power_uclamp_max_on_nice_low_prio),
	PROC_ENTRY(cam_power_uclamp_max_on_nice_mid_prio),
	PROC_ENTRY(cam_power_uclamp_max_on_nice_high_prio),
	PROC_ENTRY(cam_power_uclamp_min_on_nice_enable),
	PROC_ENTRY(cam_power_uclamp_max_on_nice_enable),
	PROC_ENTRY(cam_power_ug),
	// Background group attributes
	PROC_ENTRY(bg_prefer_idle),
	PROC_ENTRY(bg_prefer_high_cap),
	PROC_ENTRY(bg_task_spreading),
#if !IS_ENABLED(CONFIG_USE_VENDOR_GROUP_UTIL)
	PROC_ENTRY(bg_group_throttle),
#endif
	PROC_ENTRY(bg_preferred_idle_mask_low),
	PROC_ENTRY(bg_preferred_idle_mask_mid),
	PROC_ENTRY(bg_preferred_idle_mask_high),
	PROC_ENTRY(bg_uclamp_min),
	PROC_ENTRY(bg_uclamp_max),
	PROC_ENTRY(bg_uclamp_min_on_nice_low_value),
	PROC_ENTRY(bg_uclamp_min_on_nice_mid_value),
	PROC_ENTRY(bg_uclamp_min_on_nice_high_value),
	PROC_ENTRY(bg_uclamp_max_on_nice_low_value),
	PROC_ENTRY(bg_uclamp_max_on_nice_mid_value),
	PROC_ENTRY(bg_uclamp_max_on_nice_high_value),
	PROC_ENTRY(bg_uclamp_min_on_nice_low_prio),
	PROC_ENTRY(bg_uclamp_min_on_nice_mid_prio),
	PROC_ENTRY(bg_uclamp_min_on_nice_high_prio),
	PROC_ENTRY(bg_uclamp_max_on_nice_low_prio),
	PROC_ENTRY(bg_uclamp_max_on_nice_mid_prio),
	PROC_ENTRY(bg_uclamp_max_on_nice_high_prio),
	PROC_ENTRY(bg_uclamp_min_on_nice_enable),
	PROC_ENTRY(bg_uclamp_max_on_nice_enable),
	PROC_ENTRY(bg_ug),
	// System Background group attributes
	PROC_ENTRY(sysbg_prefer_idle),
	PROC_ENTRY(sysbg_prefer_high_cap),
	PROC_ENTRY(sysbg_task_spreading),
#if !IS_ENABLED(CONFIG_USE_VENDOR_GROUP_UTIL)
	PROC_ENTRY(sysbg_group_throttle),
#endif
	PROC_ENTRY(sysbg_preferred_idle_mask_low),
	PROC_ENTRY(sysbg_preferred_idle_mask_mid),
	PROC_ENTRY(sysbg_preferred_idle_mask_high),
	PROC_ENTRY(sysbg_uclamp_min),
	PROC_ENTRY(sysbg_uclamp_max),
	PROC_ENTRY(sysbg_uclamp_min_on_nice_low_value),
	PROC_ENTRY(sysbg_uclamp_min_on_nice_mid_value),
	PROC_ENTRY(sysbg_uclamp_min_on_nice_high_value),
	PROC_ENTRY(sysbg_uclamp_max_on_nice_low_value),
	PROC_ENTRY(sysbg_uclamp_max_on_nice_mid_value),
	PROC_ENTRY(sysbg_uclamp_max_on_nice_high_value),
	PROC_ENTRY(sysbg_uclamp_min_on_nice_low_prio),
	PROC_ENTRY(sysbg_uclamp_min_on_nice_mid_prio),
	PROC_ENTRY(sysbg_uclamp_min_on_nice_high_prio),
	PROC_ENTRY(sysbg_uclamp_max_on_nice_low_prio),
	PROC_ENTRY(sysbg_uclamp_max_on_nice_mid_prio),
	PROC_ENTRY(sysbg_uclamp_max_on_nice_high_prio),
	PROC_ENTRY(sysbg_uclamp_min_on_nice_enable),
	PROC_ENTRY(sysbg_uclamp_max_on_nice_enable),
	PROC_ENTRY(sysbg_ug),
	// Nnapi-HAL group attributes
	PROC_ENTRY(nnapi_prefer_idle),
	PROC_ENTRY(nnapi_prefer_high_cap),
	PROC_ENTRY(nnapi_task_spreading),
#if !IS_ENABLED(CONFIG_USE_VENDOR_GROUP_UTIL)
	PROC_ENTRY(nnapi_group_throttle),
#endif
	PROC_ENTRY(nnapi_preferred_idle_mask_low),
	PROC_ENTRY(nnapi_preferred_idle_mask_mid),
	PROC_ENTRY(nnapi_preferred_idle_mask_high),
	PROC_ENTRY(nnapi_uclamp_min),
	PROC_ENTRY(nnapi_uclamp_max),
	PROC_ENTRY(nnapi_uclamp_min_on_nice_low_value),
	PROC_ENTRY(nnapi_uclamp_min_on_nice_mid_value),
	PROC_ENTRY(nnapi_uclamp_min_on_nice_high_value),
	PROC_ENTRY(nnapi_uclamp_max_on_nice_low_value),
	PROC_ENTRY(nnapi_uclamp_max_on_nice_mid_value),
	PROC_ENTRY(nnapi_uclamp_max_on_nice_high_value),
	PROC_ENTRY(nnapi_uclamp_min_on_nice_low_prio),
	PROC_ENTRY(nnapi_uclamp_min_on_nice_mid_prio),
	PROC_ENTRY(nnapi_uclamp_min_on_nice_high_prio),
	PROC_ENTRY(nnapi_uclamp_max_on_nice_low_prio),
	PROC_ENTRY(nnapi_uclamp_max_on_nice_mid_prio),
	PROC_ENTRY(nnapi_uclamp_max_on_nice_high_prio),
	PROC_ENTRY(nnapi_uclamp_min_on_nice_enable),
	PROC_ENTRY(nnapi_uclamp_max_on_nice_enable),
	PROC_ENTRY(nnapi_ug),
	// RT group attributes
	PROC_ENTRY(rt_prefer_idle),
	PROC_ENTRY(rt_prefer_high_cap),
	PROC_ENTRY(rt_task_spreading),
#if !IS_ENABLED(CONFIG_USE_VENDOR_GROUP_UTIL)
	PROC_ENTRY(rt_group_throttle),
#endif
	PROC_ENTRY(rt_preferred_idle_mask_low),
	PROC_ENTRY(rt_preferred_idle_mask_mid),
	PROC_ENTRY(rt_preferred_idle_mask_high),
	PROC_ENTRY(rt_uclamp_min),
	PROC_ENTRY(rt_uclamp_max),
	PROC_ENTRY(rt_uclamp_min_on_nice_low_value),
	PROC_ENTRY(rt_uclamp_min_on_nice_mid_value),
	PROC_ENTRY(rt_uclamp_min_on_nice_high_value),
	PROC_ENTRY(rt_uclamp_max_on_nice_low_value),
	PROC_ENTRY(rt_uclamp_max_on_nice_mid_value),
	PROC_ENTRY(rt_uclamp_max_on_nice_high_value),
	PROC_ENTRY(rt_uclamp_min_on_nice_low_prio),
	PROC_ENTRY(rt_uclamp_min_on_nice_mid_prio),
	PROC_ENTRY(rt_uclamp_min_on_nice_high_prio),
	PROC_ENTRY(rt_uclamp_max_on_nice_low_prio),
	PROC_ENTRY(rt_uclamp_max_on_nice_mid_prio),
	PROC_ENTRY(rt_uclamp_max_on_nice_high_prio),
	PROC_ENTRY(rt_uclamp_min_on_nice_enable),
	PROC_ENTRY(rt_uclamp_max_on_nice_enable),
	PROC_ENTRY(rt_ug),
	// DEX2OAT group attributes
	PROC_ENTRY(dex2oat_prefer_idle),
	PROC_ENTRY(dex2oat_prefer_high_cap),
	PROC_ENTRY(dex2oat_task_spreading),
#if !IS_ENABLED(CONFIG_USE_VENDOR_GROUP_UTIL)
	PROC_ENTRY(dex2oat_group_throttle),
#endif
	PROC_ENTRY(dex2oat_preferred_idle_mask_low),
	PROC_ENTRY(dex2oat_preferred_idle_mask_mid),
	PROC_ENTRY(dex2oat_preferred_idle_mask_high),
	PROC_ENTRY(dex2oat_uclamp_min),
	PROC_ENTRY(dex2oat_uclamp_max),
	PROC_ENTRY(dex2oat_uclamp_min_on_nice_low_value),
	PROC_ENTRY(dex2oat_uclamp_min_on_nice_mid_value),
	PROC_ENTRY(dex2oat_uclamp_min_on_nice_high_value),
	PROC_ENTRY(dex2oat_uclamp_max_on_nice_low_value),
	PROC_ENTRY(dex2oat_uclamp_max_on_nice_mid_value),
	PROC_ENTRY(dex2oat_uclamp_max_on_nice_high_value),
	PROC_ENTRY(dex2oat_uclamp_min_on_nice_low_prio),
	PROC_ENTRY(dex2oat_uclamp_min_on_nice_mid_prio),
	PROC_ENTRY(dex2oat_uclamp_min_on_nice_high_prio),
	PROC_ENTRY(dex2oat_uclamp_max_on_nice_low_prio),
	PROC_ENTRY(dex2oat_uclamp_max_on_nice_mid_prio),
	PROC_ENTRY(dex2oat_uclamp_max_on_nice_high_prio),
	PROC_ENTRY(dex2oat_uclamp_min_on_nice_enable),
	PROC_ENTRY(dex2oat_uclamp_max_on_nice_enable),
	PROC_ENTRY(dex2oat_ug),
	// OTA group attributes
	PROC_ENTRY(ota_prefer_idle),
	PROC_ENTRY(ota_prefer_high_cap),
	PROC_ENTRY(ota_task_spreading),
#if !IS_ENABLED(CONFIG_USE_VENDOR_GROUP_UTIL)
	PROC_ENTRY(ota_group_throttle),
#endif
	PROC_ENTRY(ota_preferred_idle_mask_low),
	PROC_ENTRY(ota_preferred_idle_mask_mid),
	PROC_ENTRY(ota_preferred_idle_mask_high),
	PROC_ENTRY(ota_uclamp_min),
	PROC_ENTRY(ota_uclamp_max),
	PROC_ENTRY(ota_uclamp_min_on_nice_low_value),
	PROC_ENTRY(ota_uclamp_min_on_nice_mid_value),
	PROC_ENTRY(ota_uclamp_min_on_nice_high_value),
	PROC_ENTRY(ota_uclamp_max_on_nice_low_value),
	PROC_ENTRY(ota_uclamp_max_on_nice_mid_value),
	PROC_ENTRY(ota_uclamp_max_on_nice_high_value),
	PROC_ENTRY(ota_uclamp_min_on_nice_low_prio),
	PROC_ENTRY(ota_uclamp_min_on_nice_mid_prio),
	PROC_ENTRY(ota_uclamp_min_on_nice_high_prio),
	PROC_ENTRY(ota_uclamp_max_on_nice_low_prio),
	PROC_ENTRY(ota_uclamp_max_on_nice_mid_prio),
	PROC_ENTRY(ota_uclamp_max_on_nice_high_prio),
	PROC_ENTRY(ota_uclamp_min_on_nice_enable),
	PROC_ENTRY(ota_uclamp_max_on_nice_enable),
	PROC_ENTRY(ota_ug),
	// SF group attributes
	PROC_ENTRY(sf_prefer_idle),
	PROC_ENTRY(sf_prefer_high_cap),
	PROC_ENTRY(sf_task_spreading),
#if !IS_ENABLED(CONFIG_USE_VENDOR_GROUP_UTIL)
	PROC_ENTRY(sf_group_throttle),
#endif
	PROC_ENTRY(sf_preferred_idle_mask_low),
	PROC_ENTRY(sf_preferred_idle_mask_mid),
	PROC_ENTRY(sf_preferred_idle_mask_high),
	PROC_ENTRY(sf_uclamp_min),
	PROC_ENTRY(sf_uclamp_max),
	PROC_ENTRY(sf_uclamp_min_on_nice_low_value),
	PROC_ENTRY(sf_uclamp_min_on_nice_mid_value),
	PROC_ENTRY(sf_uclamp_min_on_nice_high_value),
	PROC_ENTRY(sf_uclamp_max_on_nice_low_value),
	PROC_ENTRY(sf_uclamp_max_on_nice_mid_value),
	PROC_ENTRY(sf_uclamp_max_on_nice_high_value),
	PROC_ENTRY(sf_uclamp_min_on_nice_low_prio),
	PROC_ENTRY(sf_uclamp_min_on_nice_mid_prio),
	PROC_ENTRY(sf_uclamp_min_on_nice_high_prio),
	PROC_ENTRY(sf_uclamp_max_on_nice_low_prio),
	PROC_ENTRY(sf_uclamp_max_on_nice_mid_prio),
	PROC_ENTRY(sf_uclamp_max_on_nice_high_prio),
	PROC_ENTRY(sf_uclamp_min_on_nice_enable),
	PROC_ENTRY(sf_uclamp_max_on_nice_enable),
	PROC_ENTRY(sf_ug),
#if IS_ENABLED(CONFIG_USE_VENDOR_GROUP_UTIL)
	// FG util group attributes
#if IS_ENABLED(CONFIG_USE_GROUP_THROTTLE)
	PROC_ENTRY(ug_fg_group_throttle),
#endif
	PROC_ENTRY(ug_fg_uclamp_min),
	PROC_ENTRY(ug_fg_uclamp_max),
	// BG util group attributes
#if IS_ENABLED(CONFIG_USE_GROUP_THROTTLE)
	PROC_ENTRY(ug_bg_group_throttle),
#endif
	PROC_ENTRY(ug_bg_uclamp_min),
	PROC_ENTRY(ug_bg_uclamp_max),
	PROC_ENTRY(ug_bg_auto_prio),
#endif
	// Vendor group attributes
	PROC_ENTRY(set_task_group_ta),
	PROC_ENTRY(set_task_group_fg),
	PROC_ENTRY(set_task_group_sys),
	PROC_ENTRY(set_task_group_cam),
	PROC_ENTRY(set_task_group_cam_power),
	PROC_ENTRY(set_task_group_bg),
	PROC_ENTRY(set_task_group_sysbg),
	PROC_ENTRY(set_task_group_nnapi),
	PROC_ENTRY(set_task_group_rt),
	PROC_ENTRY(set_task_group_dex2oat),
	PROC_ENTRY(set_task_group_ota),
	PROC_ENTRY(set_task_group_sf),
	PROC_ENTRY(set_proc_group_ta),
	PROC_ENTRY(set_proc_group_fg),
	PROC_ENTRY(set_proc_group_sys),
	PROC_ENTRY(set_proc_group_cam),
	PROC_ENTRY(set_proc_group_cam_power),
	PROC_ENTRY(set_proc_group_bg),
	PROC_ENTRY(set_proc_group_sysbg),
	PROC_ENTRY(set_proc_group_nnapi),
	PROC_ENTRY(set_proc_group_rt),
	PROC_ENTRY(set_proc_group_dex2oat),
	PROC_ENTRY(set_proc_group_ota),
	PROC_ENTRY(set_proc_group_sf),
	// Uclamp stats
#if IS_ENABLED(CONFIG_UCLAMP_STATS)
	PROC_ENTRY(uclamp_stats),
	PROC_ENTRY(uclamp_effective_stats),
	PROC_ENTRY(uclamp_util_diff_stats),
	PROC_ENTRY(reset_uclamp_stats),
#endif
	PROC_ENTRY(util_threshold),
	PROC_ENTRY(util_post_init_scale),
	PROC_ENTRY(npi_packing),
	PROC_ENTRY(idle_balancer),
	PROC_ENTRY(reduce_prefer_idle),
	PROC_ENTRY(boost_adpf_prio),
	PROC_ENTRY(dump_task),
	// pmu limit attribute
	PROC_ENTRY(pmu_poll_time),
	PROC_ENTRY(pmu_poll_enable),
	// per-task attribute
	PROC_ENTRY(prefer_idle_set),
	PROC_ENTRY(prefer_idle_clear),
	PROC_ENTRY(uclamp_fork_reset_set),
	PROC_ENTRY(uclamp_fork_reset_clear),
	// sched lib
	PROC_ENTRY(sched_lib_cpu_freq_cached),
	PROC_ENTRY(sched_lib_freq_cpumask),
	PROC_ENTRY(sched_lib_affinity),
	PROC_ENTRY(sched_lib_name),
	// uclamp filter
	PROC_ENTRY(uclamp_min_filter_enable),
	PROC_ENTRY(uclamp_min_filter_us),
	PROC_ENTRY(uclamp_min_filter_rt),
	PROC_ENTRY(uclamp_max_filter_enable),
	PROC_ENTRY(uclamp_max_filter_divider),
	PROC_ENTRY(uclamp_max_filter_rt),
	// dvfs headroom
	PROC_ENTRY(dvfs_headroom),
	PROC_ENTRY(tapered_dvfs_headroom_enable),
};


int create_procfs_node(void)
{
	int i;
	struct uclamp_se uc_max = {};
	enum uclamp_id clamp_id;

	vendor_sched = proc_mkdir("vendor_sched", NULL);

	if (!vendor_sched)
		goto out;

	/* create procfs */
	for (i = 0; i < ARRAY_SIZE(entries); i++) {
		umode_t mode;

		if (entries[i].fops->proc_write == NULL) {
			mode = 0444;
		} else if(entries[i].fops->proc_read== NULL) {
			mode = 0200;
		} else {
			mode = 0644;
		}

		if (!proc_create(entries[i].name, mode,
					vendor_sched, entries[i].fops)) {
			pr_debug("%s(), create %s failed\n",
					__func__, entries[i].name);
			remove_proc_entry("vendor_sched", NULL);

			goto out;
		}
	}

	uc_max.value = uclamp_none(UCLAMP_MAX);
	uc_max.bucket_id = get_bucket_id(uc_max.value);
	uc_max.user_defined = false;
	for (clamp_id = 0; clamp_id < UCLAMP_CNT; clamp_id++) {
		uclamp_default[clamp_id] = uc_max;
	}

	initialize_vendor_group_property();

	return 0;

out:
	return -ENOMEM;
}
