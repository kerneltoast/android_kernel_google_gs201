// SPDX-License-Identifier: GPL-2.0-only
/* sysfs_node.c
 *
 * Android Vendor Hook Support
 *
 * Copyright 2020 Google LLC
 */
#include <linux/cpuidle.h>
#include <linux/cpumask.h>
#include <linux/cpuset.h>
#include <linux/lockdep.h>
#include <linux/kobject.h>
#include <linux/sched.h>
#include <linux/sched/cputime.h>
#include <linux/sched/signal.h>
#include <linux/sched/task.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/idle_inject.h>
#include <kernel/sched/sched.h>
#include <trace/events/power.h>

#include "sched_priv.h"
#include "sched_events.h"

#if IS_ENABLED(CONFIG_UCLAMP_STATS)
extern void reset_uclamp_stats(void);
DECLARE_PER_CPU(struct uclamp_stats, uclamp_stats);
#endif

unsigned int __read_mostly vendor_sched_util_post_init_scale = DEF_UTIL_POST_INIT_SCALE;
bool __read_mostly vendor_sched_npi_packing = true; //non prefer idle packing
bool __read_mostly vendor_sched_reduce_prefer_idle = true;
bool __read_mostly vendor_sched_auto_prefer_idle = false;
bool __read_mostly vendor_sched_boost_adpf_prio = true;
unsigned int __read_mostly vendor_sched_adpf_rampup_multiplier = 1;
struct cpumask cpu_skip_mask_rt;
struct cpumask skip_prefer_prev_mask;
unsigned int __read_mostly vendor_sched_priority_task_boost_value = 0;

static struct proc_dir_entry *vendor_sched;
struct proc_dir_entry *group_dirs[VG_MAX];
extern struct vendor_group_list vendor_group_list[VG_MAX];

static struct idle_inject_device *iidev_l;
static struct idle_inject_device *iidev_m;
static struct idle_inject_device *iidev_b;

extern void initialize_vendor_group_property(void);

extern struct vendor_group_property *get_vendor_group_property(enum vendor_group group);

extern void vh_sched_setscheduler_uclamp_pixel_mod(void *data, struct task_struct *tsk,
		int clamp_id, unsigned int value);

#if IS_ENABLED(CONFIG_USE_VENDOR_GROUP_UTIL)
int __read_mostly vendor_sched_ug_bg_auto_prio = THREAD_PRIORITY_BACKGROUND;

extern void migrate_vendor_group_util(struct task_struct *p, unsigned int old, unsigned int new);
extern struct vendor_util_group_property *get_vendor_util_group_property(
	enum utilization_group group);
#endif

extern void update_task_prio(struct task_struct *p, struct vendor_task_struct *vp, bool val);

static void apply_uclamp_change(enum vendor_group group, enum uclamp_id clamp_id);

struct uclamp_se uclamp_default[UCLAMP_CNT];
unsigned int pmu_poll_time_ms = 10;
bool pmu_poll_enabled;
extern int pmu_poll_enable(void);
extern void pmu_poll_disable(void);

extern unsigned int sysctl_sched_uclamp_min_filter_us;
extern unsigned int sysctl_sched_uclamp_max_filter_divider;

extern char priority_task_name[LIB_PATH_LENGTH];
extern spinlock_t priority_task_name_lock;

extern int set_prefer_idle_task_name(void);
extern char prefer_idle_task_name[LIB_PATH_LENGTH];
extern spinlock_t prefer_idle_task_name_lock;

#define MAX_PROC_SIZE 128

static const char *GRP_NAME[VG_MAX] = {"sys", "ta", "fg", "cam", "cam_power", "bg", "sys_bg",
				       "nnapi", "rt", "dex2oat", "ota", "sf", "fg_wi"};

static const unsigned int SCHED_QOS_PROFILES[SCHED_QOS_MAX] = {
	/* SCHED_QOS_NONE */
	0,
	/* SCHED_QOS_POWER_EFFICIENCY */
	BIT(SCHED_QOS_AUTO_UCLAMP_MAX_BIT),
	/* SCHED_QOS_SENSITIVE_STANDARD */
	BIT(SCHED_QOS_ADPF_BIT) | BIT(SCHED_QOS_PREFER_IDLE_BIT) | BIT(SCHED_QOS_PREFER_FIT_BIT),
	/* SCHED_QOS_SENSITIVE_HIGH */
	BIT(SCHED_QOS_ADPF_BIT) | BIT(SCHED_QOS_PREFER_IDLE_BIT) | BIT(SCHED_QOS_PREFER_FIT_BIT) |
	BIT(SCHED_QOS_PREEMPT_WAKEUP_BIT),
	/* SCHED_QOS_SENSITIVE_EXTREME */
	BIT(SCHED_QOS_ADPF_BIT) | BIT(SCHED_QOS_PREFER_IDLE_BIT) | BIT(SCHED_QOS_PREFER_FIT_BIT) |
	BIT(SCHED_QOS_PREEMPT_WAKEUP_BIT) | BIT(SCHED_QOS_BOOST_PRIO_BIT) };

enum vendor_procfs_type {
	DEFAULT_TYPE = 0,
	GROUPED_CONTROL,
	SCHED_QOS_CONTROL,
};

#define PROC_OPS_RW(__name) \
		static int __name##_proc_open(\
			struct inode *inode, struct file *file) \
		{ \
			return single_open(file,\
			__name##_show, pde_data(inode));\
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
			__name##_show, pde_data(inode));\
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

#define PROC_ENTRY(__name) {__stringify(__name), DEFAULT_TYPE, -1, &__name##_proc_ops}

#define __PROC_GROUP_ENTRY(__name, __group_name, __vg) \
		{__stringify(__name), GROUPED_CONTROL, __vg, &__group_name##_##__name##_proc_ops}

#define __PROC_SET_GROUP_ENTRY(__name, __group_name, __vg) \
		{__stringify(__name), GROUPED_CONTROL, __vg, &__name##_##__group_name##_proc_ops}

#define __PROC_GROUP_ENTRIES(__group_name, __vg)	\
		__PROC_GROUP_ENTRY(prefer_idle, __group_name, __vg),	\
		__PROC_GROUP_ENTRY(prefer_high_cap, __group_name, __vg),	\
		__PROC_GROUP_ENTRY(task_spreading, __group_name, __vg),	\
		__PROC_GROUP_ENTRY(auto_prefer_fit, __group_name, __vg),	\
		__PROC_GROUP_ENTRY(group_cfs_skip_mask, __group_name, __vg),	\
		__PROC_GROUP_ENTRY(preferred_idle_mask_low, __group_name, __vg),	\
		__PROC_GROUP_ENTRY(preferred_idle_mask_mid, __group_name, __vg),	\
		__PROC_GROUP_ENTRY(preferred_idle_mask_high, __group_name, __vg),	\
		__PROC_GROUP_ENTRY(uclamp_min, __group_name, __vg),	\
		__PROC_GROUP_ENTRY(uclamp_min_on_nice_enable, __group_name, __vg),	\
		__PROC_GROUP_ENTRY(uclamp_min_on_nice_low_value, __group_name, __vg),	\
		__PROC_GROUP_ENTRY(uclamp_min_on_nice_mid_value, __group_name, __vg),	\
		__PROC_GROUP_ENTRY(uclamp_min_on_nice_high_value, __group_name, __vg),	\
		__PROC_GROUP_ENTRY(uclamp_min_on_nice_low_prio, __group_name, __vg),	\
		__PROC_GROUP_ENTRY(uclamp_min_on_nice_mid_prio, __group_name, __vg),	\
		__PROC_GROUP_ENTRY(uclamp_min_on_nice_high_prio, __group_name, __vg),	\
		__PROC_GROUP_ENTRY(uclamp_max, __group_name, __vg),	\
		__PROC_GROUP_ENTRY(uclamp_max_on_nice_enable, __group_name, __vg),	\
		__PROC_GROUP_ENTRY(uclamp_max_on_nice_low_value, __group_name, __vg),	\
		__PROC_GROUP_ENTRY(uclamp_max_on_nice_mid_value, __group_name, __vg),	\
		__PROC_GROUP_ENTRY(uclamp_max_on_nice_high_value, __group_name, __vg),	\
		__PROC_GROUP_ENTRY(uclamp_max_on_nice_low_prio, __group_name, __vg),	\
		__PROC_GROUP_ENTRY(uclamp_max_on_nice_mid_prio, __group_name, __vg),	\
		__PROC_GROUP_ENTRY(uclamp_max_on_nice_high_prio, __group_name, __vg),	\
		__PROC_GROUP_ENTRY(rampup_multiplier, __group_name, __vg),	\
		__PROC_GROUP_ENTRY(disable_util_est, __group_name, __vg),	\
		__PROC_GROUP_ENTRY(qos_adpf_enable, __group_name, __vg),	\
		__PROC_GROUP_ENTRY(qos_prefer_idle_enable, __group_name, __vg),	\
		__PROC_GROUP_ENTRY(qos_prefer_fit_enable, __group_name, __vg),	\
		__PROC_GROUP_ENTRY(qos_boost_prio_enable, __group_name, __vg),	\
		__PROC_GROUP_ENTRY(qos_preempt_wakeup_enable, __group_name, __vg),	\
		__PROC_GROUP_ENTRY(qos_auto_uclamp_max_enable, __group_name, __vg),	\
		__PROC_GROUP_ENTRY(qos_prefer_high_cap_enable, __group_name, __vg),	\
		__PROC_GROUP_ENTRY(qos_rampup_multiplier_enable, __group_name, __vg),	\
		__PROC_GROUP_ENTRY(disable_sched_setaffinity, __group_name, __vg),	\
		__PROC_GROUP_ENTRY(use_batch_policy, __group_name, __vg),		\
		__PROC_SET_GROUP_ENTRY(set_task_group, __group_name, __vg),	\
		__PROC_SET_GROUP_ENTRY(set_proc_group, __group_name, __vg)

#if IS_ENABLED(CONFIG_USE_VENDOR_GROUP_UTIL)
#define PROC_GROUP_ENTRIES(__group_name, __vg)	\
		__PROC_GROUP_ENTRIES(__group_name, __vg),	\
		__PROC_GROUP_ENTRY(ug, __group_name, __vg)
#else
#define PROC_GROUP_ENTRIES(__group_name, __vg)	\
		__PROC_GROUP_ENTRIES(__group_name, __vg),	\
		__PROC_GROUP_ENTRY(group_throttle, __group_name, __vg)
#endif

#define PROC_SCHED_QOS_ENTRY(__name)	\
		 {__stringify(__name), SCHED_QOS_CONTROL, -1, &__name##_proc_ops}

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
			seq_printf(m, "%d\n", gp->uc_req[__cid].value);		      \
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
			if (kstrtoint(buf, 0, &val))					      \
				return -EINVAL;						      \
			if (val > 1024 && val != AUTO_UCLAMP_MAX_MAGIC)			      \
				return -EINVAL;						      \
			if (val == gp->uc_req[__cid].value && (__cid != UCLAMP_MAX ||	      \
			    !gp->auto_uclamp_max))					      \
				return count;						      \
			if (__cid == UCLAMP_MAX) {					      \
				if (val == AUTO_UCLAMP_MAX_MAGIC) {			      \
					gp->auto_uclamp_max = true;			      \
					val = uclamp_none(UCLAMP_MAX);			      \
				} else {						      \
					gp->auto_uclamp_max = false;			      \
				}							      \
			}								      \
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

#define PER_TASK_UINT_ATTRIBUTE(__attr)						      \
		static ssize_t __attr##_store(struct file *filp, \
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
			ret = update_##__attr(buf, count);   \
			return ret ?: count;						      \
		}									      \
		PROC_OPS_WO(__attr);

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

static inline bool check_rampup_multiplier(enum vendor_group group)
{
	return true;
}

static inline bool reset_group_sched_setaffinity(enum vendor_group group);
static inline bool reset_group_batch_policy(enum vendor_group group);

#if IS_ENABLED(CONFIG_USE_VENDOR_GROUP_UTIL)
#define CREATE_VENDOR_GROUP_UTIL_ATTRIBUTES(__grp, __vg)				\
	VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(__grp, ug, __vg, check_ug);
#else
#define CREATE_VENDOR_GROUP_UTIL_ATTRIBUTES(__grp, __vg)				\
	VENDOR_GROUP_UINT_ATTRIBUTE(__grp, group_throttle, __vg);
#endif

#define CREATE_VENDOR_GROUP_ATTRIBUTES(__grp, __vg)					\
	VENDOR_GROUP_BOOL_ATTRIBUTE(__grp, prefer_idle, __vg);				\
	VENDOR_GROUP_BOOL_ATTRIBUTE(__grp, prefer_high_cap, __vg);			\
	VENDOR_GROUP_BOOL_ATTRIBUTE(__grp, task_spreading, __vg);			\
	VENDOR_GROUP_BOOL_ATTRIBUTE(__grp, auto_prefer_fit, __vg);			\
	VENDOR_GROUP_CPUMASK_ATTRIBUTE(__grp, group_cfs_skip_mask, __vg);		\
	VENDOR_GROUP_CPUMASK_ATTRIBUTE(__grp, preferred_idle_mask_low, __vg);		\
	VENDOR_GROUP_CPUMASK_ATTRIBUTE(__grp, preferred_idle_mask_mid, __vg);		\
	VENDOR_GROUP_CPUMASK_ATTRIBUTE(__grp, preferred_idle_mask_high, __vg);		\
	VENDOR_GROUP_UCLAMP_ATTRIBUTE(__grp, uclamp_min, __vg, UCLAMP_MIN);		\
	VENDOR_GROUP_UCLAMP_ATTRIBUTE(__grp, uclamp_max, __vg, UCLAMP_MAX);		\
	VENDOR_GROUP_UINT_ATTRIBUTE(__grp, uclamp_min_on_nice_low_value, __vg);		\
	VENDOR_GROUP_UINT_ATTRIBUTE(__grp, uclamp_min_on_nice_mid_value, __vg);		\
	VENDOR_GROUP_UINT_ATTRIBUTE(__grp, uclamp_min_on_nice_high_value, __vg);	\
	VENDOR_GROUP_UINT_ATTRIBUTE(__grp, uclamp_max_on_nice_low_value, __vg);		\
	VENDOR_GROUP_UINT_ATTRIBUTE(__grp, uclamp_max_on_nice_mid_value, __vg);		\
	VENDOR_GROUP_UINT_ATTRIBUTE(__grp, uclamp_max_on_nice_high_value, __vg);	\
	VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(__grp, uclamp_min_on_nice_low_prio, __vg,	\
					  check_uclamp_min_on_nice_prio);		\
	VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(__grp, uclamp_min_on_nice_mid_prio, __vg,	\
					  check_uclamp_min_on_nice_prio);		\
	VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(__grp, uclamp_min_on_nice_high_prio, __vg,	\
					  check_uclamp_min_on_nice_prio);		\
	VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(__grp, uclamp_max_on_nice_low_prio, __vg,	\
					  check_uclamp_max_on_nice_prio);		\
	VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(__grp, uclamp_max_on_nice_mid_prio, __vg,	\
					  check_uclamp_max_on_nice_prio);		\
	VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(__grp, uclamp_max_on_nice_high_prio, __vg,	\
					  check_uclamp_max_on_nice_prio);		\
	VENDOR_GROUP_BOOL_ATTRIBUTE(__grp, uclamp_min_on_nice_enable, __vg);		\
	VENDOR_GROUP_BOOL_ATTRIBUTE(__grp, uclamp_max_on_nice_enable, __vg);		\
	VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(__grp, rampup_multiplier, __vg,		\
					  check_rampup_multiplier);			\
	VENDOR_GROUP_BOOL_ATTRIBUTE(__grp, disable_util_est, __vg);			\
	VENDOR_GROUP_BOOL_ATTRIBUTE(__grp, qos_adpf_enable, __vg);			\
	VENDOR_GROUP_BOOL_ATTRIBUTE(__grp, qos_prefer_idle_enable, __vg);		\
	VENDOR_GROUP_BOOL_ATTRIBUTE(__grp, qos_prefer_fit_enable, __vg);		\
	VENDOR_GROUP_BOOL_ATTRIBUTE(__grp, qos_boost_prio_enable, __vg);		\
	VENDOR_GROUP_BOOL_ATTRIBUTE(__grp, qos_preempt_wakeup_enable, __vg);		\
	VENDOR_GROUP_BOOL_ATTRIBUTE(__grp, qos_auto_uclamp_max_enable, __vg);		\
	VENDOR_GROUP_BOOL_ATTRIBUTE(__grp, qos_prefer_high_cap_enable, __vg);		\
	VENDOR_GROUP_BOOL_ATTRIBUTE(__grp, qos_rampup_multiplier_enable, __vg);		\
	VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(__grp, disable_sched_setaffinity, __vg,	\
					  reset_group_sched_setaffinity);		\
	VENDOR_GROUP_UINT_ATTRIBUTE_CHECK(__grp, use_batch_policy, __vg,		\
					  reset_group_batch_policy);			\
	CREATE_VENDOR_GROUP_UTIL_ATTRIBUTES(__grp, __vg);

/// ******************************************************************************** ///
/// ********************* Create vendor group procfs nodes*************************** ///
/// ******************************************************************************** ///

UCLAMP_ON_NICE_PRIO_CHECK_FUN(min);
UCLAMP_ON_NICE_PRIO_CHECK_FUN(max);

#if IS_ENABLED(CONFIG_USE_VENDOR_GROUP_UTIL)
static inline bool check_ug(enum vendor_group group)
{
	if (vg[group].ug < UG_BG || vg[group].ug > UG_AUTO)
		return false;

	return true;
}
#endif

CREATE_VENDOR_GROUP_ATTRIBUTES(ta, VG_TOPAPP);
CREATE_VENDOR_GROUP_ATTRIBUTES(fg, VG_FOREGROUND);
CREATE_VENDOR_GROUP_ATTRIBUTES(sys, VG_SYSTEM);
CREATE_VENDOR_GROUP_ATTRIBUTES(cam, VG_CAMERA);
CREATE_VENDOR_GROUP_ATTRIBUTES(cam_power, VG_CAMERA_POWER);
CREATE_VENDOR_GROUP_ATTRIBUTES(bg, VG_BACKGROUND);
CREATE_VENDOR_GROUP_ATTRIBUTES(sysbg, VG_SYSTEM_BACKGROUND);
CREATE_VENDOR_GROUP_ATTRIBUTES(nnapi, VG_NNAPI_HAL);
CREATE_VENDOR_GROUP_ATTRIBUTES(rt, VG_RT);
CREATE_VENDOR_GROUP_ATTRIBUTES(dex2oat, VG_DEX2OAT);
CREATE_VENDOR_GROUP_ATTRIBUTES(ota, VG_OTA);
CREATE_VENDOR_GROUP_ATTRIBUTES(sf, VG_SF);
CREATE_VENDOR_GROUP_ATTRIBUTES(fg_wi, VG_FOREGROUND_WINDOW);

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
/// ********************* New code section ***************************************** ///
/// ******************************************************************************** ///
static int update_vendor_tunables(const char *buf, int count, int type)
{
	char *tok, *str1, *str2;
	unsigned int val, tmp[CONFIG_VH_SCHED_MAX_CPU_NR];
	int index = 0;
	unsigned int *updated_tunables;

	str1 = kstrndup(buf, count, GFP_KERNEL);
	str2 = str1;

	if (!str2)
		return -ENOMEM;

	while (1) {
		tok = strsep(&str2, " ");

		if (tok == NULL)
			break;

		if (kstrtouint(tok, 0, &val))
			goto fail;

		switch (type) {
			case SCHED_CAPACITY_MARGIN:
				if (val < SCHED_CAPACITY_SCALE)
					goto fail;
				updated_tunables = sched_capacity_margin;
				break;
			case THERMAL_CAP_MARGIN:
				if (val < SCHED_CAPACITY_SCALE)
					goto fail;
				updated_tunables = thermal_cap_margin;
				break;
			case SCHED_AUTO_UCLAMP_MAX:
				if (val > SCHED_CAPACITY_SCALE)
					goto fail;
				updated_tunables = sched_auto_uclamp_max;
				break;
			case SCHED_DVFS_HEADROOM:
				if (val > DEF_UTIL_THRESHOLD || val < SCHED_CAPACITY_SCALE)
					goto fail;
				updated_tunables = sched_dvfs_headroom;
				break;
			case SCHED_IOWAIT_BOOST_MAX:
				if (val > SCHED_CAPACITY_SCALE)
					goto fail;
				updated_tunables = sched_per_cpu_iowait_boost_max_value;
				break;
			default:
				goto fail;
		}
		tmp[index] = val;
		index++;

		if (index == pixel_cpu_num)
			break;
	}

	if (index == 1) {
		for (index = 0; index < pixel_cpu_num; index++) {
			updated_tunables[index] = tmp[0];
		}
	} else if (index == pixel_cluster_num) {
		for (index = pixel_cluster_start_cpu[0]; index < pixel_cluster_start_cpu[1]; index++)
			updated_tunables[index] = tmp[0];

		for (index = pixel_cluster_start_cpu[1]; index < pixel_cluster_start_cpu[2]; index++)
			updated_tunables[index] = tmp[1];

		for (index = pixel_cluster_start_cpu[2]; index < pixel_cpu_num; index++)
			updated_tunables[index] = tmp[2];
	} else if (index == pixel_cpu_num) {
		memcpy(updated_tunables, tmp, sizeof(tmp));
	} else {
		goto fail;
	}

	kfree(str1);
	return count;
fail:
	kfree(str1);
	return -EINVAL;
}

static int update_teo_util_threshold(const char *buf, int count)
{
	char *tok, *str1, *str2;
	unsigned int val, tmp[CONFIG_VH_SCHED_MAX_CPU_NR];
	int index = 0;

	str1 = kstrndup(buf, count, GFP_KERNEL);
	str2 = str1;

	if (!str2)
		return -ENOMEM;

	while (1) {
		tok = strsep(&str2, " ");

		if (tok == NULL)
			break;

		if (kstrtouint(tok, 0, &val))
			goto fail;

		if (val > SCHED_CAPACITY_SCALE)
			goto fail;

		tmp[index] = val;
		index++;

		if (index == pixel_cpu_num)
			break;
	}

	if (index == 1) {
		for (index = 0; index < pixel_cpu_num; index++) {
			teo_cpu_set_util_threshold(index, tmp[0]);
		}
	} else if (index == pixel_cluster_num) {
		for (index = pixel_cluster_start_cpu[0]; index < pixel_cluster_start_cpu[1]; index++)
			teo_cpu_set_util_threshold(index, tmp[0]);

		for (index = pixel_cluster_start_cpu[1]; index < pixel_cluster_start_cpu[2]; index++)
			teo_cpu_set_util_threshold(index, tmp[1]);

		for (index = pixel_cluster_start_cpu[2]; index < pixel_cpu_num; index++)
			teo_cpu_set_util_threshold(index, tmp[2]);
	} else if (index == pixel_cpu_num) {
		for (index = 0; index < pixel_cpu_num; index++) {
			teo_cpu_set_util_threshold(index, tmp[index]);
		}
	} else {
		goto fail;
	}

	kfree(str1);
	return count;
fail:
	kfree(str1);
	return -EINVAL;
}

inline void __reset_task_affinity(struct task_struct *p)
{
	struct cpumask out_mask;

	if (p->flags & (PF_SUPERPRIV | PF_WQ_WORKER | PF_IDLE | PF_NO_SETAFFINITY | PF_KTHREAD))
		return;

	cpuset_cpus_allowed(p, &out_mask);
	set_cpus_allowed_ptr(p, &out_mask);
}

/*
 * Reset cpumask to task's cpuset for all tasks in the system.
 */
static inline void reset_sched_setaffinity(void)
{
	struct task_struct *p, *t;

	rcu_read_lock();
	for_each_process_thread(p, t)
		__reset_task_affinity(t);
	rcu_read_unlock();
}

/*
 * Reset cpumask to task's cpuset for all tasks in the group.
 */
static inline bool reset_group_sched_setaffinity(enum vendor_group group)
{
	struct task_struct *p, *t;

	if (!vg[group].disable_sched_setaffinity)
		return true;

	rcu_read_lock();
	for_each_process_thread(p, t) {
		if (get_vendor_group(t) == group)
			__reset_task_affinity(t);
	}
	rcu_read_unlock();

	return true;
}

static inline void __set_batch_policy(struct task_struct *p, int group)
{
	struct vendor_task_struct *vp;
	struct rq_flags rf;
	struct rq *rq;

	rq = task_rq_lock(p, &rf);

	if (!fair_policy(p->policy))
		goto out;

	vp = get_vendor_task_struct(p);
	if (vg[group].use_batch_policy)
		p->policy = SCHED_BATCH;
	else
		p->policy = vp->orig_policy;

out:
	task_rq_unlock(rq, p, &rf);
}

static inline void set_batch_policy(struct task_struct *p, int old, int new)
{
	if (vg[old].use_batch_policy == vg[new].use_batch_policy)
		return;

	__set_batch_policy(p, new);
}

static inline bool reset_group_batch_policy(enum vendor_group group)
{
	struct task_struct *p, *t;

	rcu_read_lock();
	for_each_process_thread(p, t) {
		if (get_vendor_group(t) == group)
			__set_batch_policy(t, group);
	}
	rcu_read_unlock();

	return true;
}

static inline struct task_struct *get_next_task(int group, struct list_head *head)
{
	unsigned long irqflags;
	struct task_struct *p;
	struct vendor_task_struct *vp;
	struct list_head *cur;

	raw_spin_lock_irqsave(&vendor_group_list[group].lock, irqflags);

	if (list_empty(head)) {
		vendor_group_list[group].cur_iterator = NULL;
		raw_spin_unlock_irqrestore(&vendor_group_list[group].lock, irqflags);
		return NULL;
	}

	if (vendor_group_list[group].cur_iterator)
		cur = vendor_group_list[group].cur_iterator;
	else
		cur = head;

	do {
		if (cur->next == head) {
			vendor_group_list[group].cur_iterator = NULL;
			raw_spin_unlock_irqrestore(&vendor_group_list[group].lock, irqflags);
			return NULL;
		}

		cur = cur->next;
		vp = list_entry(cur, struct vendor_task_struct, node);
		p = __container_of(vp, struct task_struct, android_vendor_data1);
	} while ((!task_on_rq_queued(p) || p->flags & PF_EXITING));

	get_task_struct(p);
	vendor_group_list[group].cur_iterator = cur;

	raw_spin_unlock_irqrestore(&vendor_group_list[group].lock, irqflags);

	return p;
}

static void apply_uclamp_change(enum vendor_group group, enum uclamp_id clamp_id)
{
	struct vendor_group_list *vgl = &vendor_group_list[group];
	struct task_struct *p;

	if (trace_clock_set_rate_enabled()) {
		char trace_name[32] = {0};
		struct vendor_group_property *gp = get_vendor_group_property(group);
		scnprintf(trace_name, sizeof(trace_name), "%s_grp_%s",
			clamp_id  == UCLAMP_MIN ? "UCLAMP_MIN" : "UCLAMP_MAX", GRP_NAME[group]);
		trace_clock_set_rate(trace_name, gp->uc_req[clamp_id].value,
				raw_smp_processor_id());
	}

	mutex_lock(&vgl->iter_mutex);
	if (WARN_ON(vgl->cur_iterator)) {
		unsigned long irqflags;
		raw_spin_lock_irqsave(&vendor_group_list[group].lock, irqflags);
		vgl->cur_iterator = NULL;
		raw_spin_unlock_irqrestore(&vendor_group_list[group].lock, irqflags);
	}

	while ((p = get_next_task(group, &vgl->list))) {
		uclamp_update_active(p, clamp_id);
		put_task_struct(p);
	}
	mutex_unlock(&vgl->iter_mutex);
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

	if (val)
		set_bit(SCHED_QOS_PREFER_IDLE_BIT, &vp->sched_qos_user_defined_flag);
	else
		clear_bit(SCHED_QOS_PREFER_IDLE_BIT, &vp->sched_qos_user_defined_flag);

	put_task_struct(p);
	rcu_read_unlock();

	return 0;
}

static int update_boost_prio(const char *buf, bool val)
{
	struct vendor_task_struct *vp;
	struct task_struct *p;
	pid_t pid;
	struct rq_flags rf;
	struct rq *rq;
	bool prev_boost_prio;

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
	prev_boost_prio = !!(vp->sched_qos_user_defined_flag & BIT(SCHED_QOS_BOOST_PRIO_BIT));

	if (val)
		set_bit(SCHED_QOS_BOOST_PRIO_BIT, &vp->sched_qos_user_defined_flag);
	else
		clear_bit(SCHED_QOS_BOOST_PRIO_BIT, &vp->sched_qos_user_defined_flag);

	if (prev_boost_prio != val) {
		/* Only boost task prio when both val and group qos_boost_prio_enable are true. */
		if (val && vg[vp->group].qos_boost_prio_enable) {
			rq = task_rq_lock(p, &rf);
			update_task_prio(p, vp, true);
			task_rq_unlock(rq, p, &rf);
		} else {
			rq = task_rq_lock(p, &rf);
			update_task_prio(p, vp, false);
			task_rq_unlock(rq, p, &rf);
		}
	}

	put_task_struct(p);
	rcu_read_unlock();

	return 0;
}

static int update_prefer_fit(const char *buf, bool val)
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

	if (val)
		set_bit(SCHED_QOS_PREFER_FIT_BIT, &vp->sched_qos_user_defined_flag);
	else
		clear_bit(SCHED_QOS_PREFER_FIT_BIT, &vp->sched_qos_user_defined_flag);

	put_task_struct(p);
	rcu_read_unlock();

	return 0;
}

static int update_adpf(const char *buf, bool val)
{
	struct vendor_task_struct *vp;
	struct task_struct *p;
	struct rq_flags rf;
	struct rq *rq;
	pid_t pid;
	bool old_adpf;

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

	old_adpf = !!(vp->sched_qos_user_defined_flag & BIT(SCHED_QOS_ADPF_BIT));

	if (old_adpf != val) {
		if (val)
			set_bit(SCHED_QOS_ADPF_BIT, &vp->sched_qos_user_defined_flag);
		else
			clear_bit(SCHED_QOS_ADPF_BIT, &vp->sched_qos_user_defined_flag);

		update_adpf_counter(p, old_adpf);
	}

	task_rq_unlock(rq, p, &rf);
	put_task_struct(p);
	rcu_read_unlock();

	return 0;
}

static int update_preempt_wakeup(const char *buf, bool val)
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

	if (val)
		set_bit(SCHED_QOS_PREEMPT_WAKEUP_BIT, &vp->sched_qos_user_defined_flag);
	else
		clear_bit(SCHED_QOS_PREEMPT_WAKEUP_BIT, &vp->sched_qos_user_defined_flag);

	put_task_struct(p);
	rcu_read_unlock();

	return 0;
}

static int update_auto_uclamp_max(const char *buf, bool val)
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

	if (val)
		set_bit(SCHED_QOS_AUTO_UCLAMP_MAX_BIT, &vp->sched_qos_user_defined_flag);
	else
		clear_bit(SCHED_QOS_AUTO_UCLAMP_MAX_BIT, &vp->sched_qos_user_defined_flag);

	put_task_struct(p);
	rcu_read_unlock();

	return 0;
}

static int update_prefer_high_cap(const char *buf, bool val)
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

	if (val)
		set_bit(SCHED_QOS_PREFER_HIGH_CAP_BIT, &vp->sched_qos_user_defined_flag);
	else
		clear_bit(SCHED_QOS_PREFER_HIGH_CAP_BIT, &vp->sched_qos_user_defined_flag);

	put_task_struct(p);
	rcu_read_unlock();

	return 0;
}

/*
 * To set per task rampup multiplier, write the format of pid:multiplier.
 */
static int update_rampup_multiplier_set(const char *buf, int count)
{
	struct vendor_task_struct *vp;
	struct task_struct *p;
	pid_t pid;
	unsigned int multiplier;
	char *str1, *str2, *pid_str, *multiplier_str;
	int ret = 0;

	str1 = kstrndup(buf, count, GFP_KERNEL);
	if (!str1)
		return -ENOMEM;

	str2 = str1;
	pid_str = strsep(&str2, ":");
	multiplier_str = str2;

	if (pid_str == NULL || multiplier_str == NULL) {
		ret = -EINVAL;
		goto error_free;
	}

	if (kstrtouint(pid_str, 0, &pid) || kstrtouint(multiplier_str, 0, &multiplier)) {
		ret = -EINVAL;
		goto error_free;
	}

	rcu_read_lock();
	p = find_task_by_vpid(pid);
	if (!p) {
		ret = -ESRCH;
		goto error_unlock;
	}

	get_task_struct(p);

	if (!check_cred(p)) {
		ret = -EACCES;
		goto error_put_task;
	}

	vp = get_vendor_task_struct(p);
	vp->rampup_multiplier = multiplier;
	set_bit(SCHED_QOS_RAMPUP_MULTIPLIER_BIT, &vp->sched_qos_user_defined_flag);

error_put_task:
	put_task_struct(p);
error_unlock:
	rcu_read_unlock();
error_free:
	kfree(str1);

	return ret;
}

/*
 * To clear per task rampup multiplier, just write pid.
 */
static int update_rampup_multiplier_clear(const char *buf, int count)
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
	vp->rampup_multiplier = 1;
	clear_bit(SCHED_QOS_RAMPUP_MULTIPLIER_BIT, &vp->sched_qos_user_defined_flag);

	put_task_struct(p);
	rcu_read_unlock();

	return 0;
}

/*
 * sched qos profiels that need to take effect immediately.
 */
static void update_sched_qos_profiles(struct task_struct *p, struct vendor_task_struct *vp)
{
	struct rq_flags rf;
	struct rq *rq;
	bool old, new;

	/* boost prio */
	old = !!(vp->prev_sched_qos_user_defined_flag & BIT(SCHED_QOS_BOOST_PRIO_BIT));
	new = !!(vp->sched_qos_user_defined_flag & BIT(SCHED_QOS_BOOST_PRIO_BIT));

	if (old != new) {
		/* Only boost task prio when both new and group qos_boost_prio_enable are true. */
		if (new && vg[vp->group].qos_boost_prio_enable) {
			rq = task_rq_lock(p, &rf);
			update_task_prio(p, vp, true);
			task_rq_unlock(rq, p, &rf);
		} else {
			rq = task_rq_lock(p, &rf);
			update_task_prio(p, vp, false);
			task_rq_unlock(rq, p, &rf);
		}
	}
}

static int update_sched_qos_none(const char *buf, int count)
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
	vp->sched_qos_profile = SCHED_QOS_NONE;
	vp->prev_sched_qos_user_defined_flag = vp->sched_qos_user_defined_flag;
	/*
	 * Clear all bits except SCHED_QOS_RAMPUP_MULTIPLIER_BIT.
	 */
	vp->sched_qos_user_defined_flag &= BIT(SCHED_QOS_RAMPUP_MULTIPLIER_BIT);
	update_sched_qos_profiles(p, vp);

	put_task_struct(p);
	rcu_read_unlock();

	return 0;
}

static int update_sched_qos_power_efficiency(const char *buf, int count)
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
	vp->sched_qos_profile = SCHED_QOS_POWER_EFFICIENCY;
	vp->prev_sched_qos_user_defined_flag = vp->sched_qos_user_defined_flag;
	/*
	 Clear all bits except SCHED_QOS_RAMPUP_MULTIPLIER_BIT.
	*/
	vp->sched_qos_user_defined_flag &= BIT(SCHED_QOS_RAMPUP_MULTIPLIER_BIT);

	vp->sched_qos_user_defined_flag |= SCHED_QOS_PROFILES[SCHED_QOS_POWER_EFFICIENCY];
	update_sched_qos_profiles(p, vp);

	put_task_struct(p);
	rcu_read_unlock();

	return 0;
}

static int update_sched_qos_sensitive_standard(const char *buf, int count)
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
	vp->sched_qos_profile = SCHED_QOS_SENSITIVE_STANDARD;
	vp->prev_sched_qos_user_defined_flag = vp->sched_qos_user_defined_flag;
	/*
	 Clear all bits except SCHED_QOS_RAMPUP_MULTIPLIER_BIT.
	*/
	vp->sched_qos_user_defined_flag &= BIT(SCHED_QOS_RAMPUP_MULTIPLIER_BIT);

	vp->sched_qos_user_defined_flag |= SCHED_QOS_PROFILES[SCHED_QOS_SENSITIVE_STANDARD];
	update_sched_qos_profiles(p, vp);

	put_task_struct(p);
	rcu_read_unlock();

	return 0;
}

static int update_sched_qos_sensitive_high(const char *buf, int count)
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
	vp->sched_qos_profile = SCHED_QOS_SENSITIVE_HIGH;
	vp->prev_sched_qos_user_defined_flag = vp->sched_qos_user_defined_flag;
	/*
	 Clear all bits except SCHED_QOS_RAMPUP_MULTIPLIER_BIT.
	*/
	vp->sched_qos_user_defined_flag &= BIT(SCHED_QOS_RAMPUP_MULTIPLIER_BIT);

	vp->sched_qos_user_defined_flag |= SCHED_QOS_PROFILES[SCHED_QOS_SENSITIVE_HIGH];
	update_sched_qos_profiles(p, vp);

	put_task_struct(p);
	rcu_read_unlock();

	return 0;
}

static int update_sched_qos_sensitive_extreme(const char *buf, int count)
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
	vp->sched_qos_profile = SCHED_QOS_SENSITIVE_EXTREME;
	vp->prev_sched_qos_user_defined_flag = vp->sched_qos_user_defined_flag;
	/*
	 Clear all bits except SCHED_QOS_RAMPUP_MULTIPLIER_BIT.
	*/
	vp->sched_qos_user_defined_flag &= BIT(SCHED_QOS_RAMPUP_MULTIPLIER_BIT);

	vp->sched_qos_user_defined_flag |= SCHED_QOS_PROFILES[SCHED_QOS_SENSITIVE_EXTREME];
	update_sched_qos_profiles(p, vp);

	put_task_struct(p);
	rcu_read_unlock();

	return 0;
}

static inline void migrate_boost_prio(struct task_struct *p, unsigned int old, unsigned int new)
{
	struct rq_flags rf;
	struct rq *rq;
	struct vendor_task_struct *vp = get_vendor_task_struct(p);

	if (vp->sched_qos_user_defined_flag & BIT(SCHED_QOS_BOOST_PRIO_BIT) &&
	    vg[old].qos_boost_prio_enable != vg[new].qos_boost_prio_enable) {
		/* Boost prio to 100. */
		if (vg[new].qos_boost_prio_enable) {
			rq = task_rq_lock(p, &rf);
			update_task_prio(p, vp, true);
			task_rq_unlock(rq, p, &rf);
		/* Restore to original prio. */
		} else {
			rq = task_rq_lock(p, &rf);
			update_task_prio(p, vp, false);
			task_rq_unlock(rq, p, &rf);
		}
	}
}

static int update_vendor_group_attribute(const char *buf, enum vendor_group_attribute vta,
					 unsigned int new)
{
	struct vendor_task_struct *vp;
	struct task_struct *p, *t;
	enum uclamp_id clamp_id;
	pid_t pid;
	unsigned long irqflags;
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
	rcu_read_unlock();

	switch (vta) {
	case VTA_TASK_GROUP:
		vp = get_vendor_task_struct(p);
		raw_spin_lock_irqsave(&vp->lock, irqflags);
		old = vp->group;
		if (old == new || p->flags & PF_EXITING) {
			raw_spin_unlock_irqrestore(&vp->lock, irqflags);
			break;
		}
		if (vp->queued_to_list == LIST_QUEUED) {
			remove_from_vendor_group_list(&vp->node, old);
			add_to_vendor_group_list(&vp->node, new);
		}
		vp->group = new;
		raw_spin_unlock_irqrestore(&vp->lock, irqflags);
		if (vg[new].disable_sched_setaffinity)
			__reset_task_affinity(p);
		set_batch_policy(p, old, new);
		if (p->prio >= MAX_RT_PRIO) {
			migrate_boost_prio(p, old, new);
#if IS_ENABLED(CONFIG_USE_VENDOR_GROUP_UTIL)
			migrate_vendor_group_util(p, old, new);
#endif
		}
		for (clamp_id = 0; clamp_id < UCLAMP_CNT; clamp_id++)
			uclamp_update_active(p, clamp_id);
		break;
	case VTA_PROC_GROUP:
		rcu_read_lock();
		for_each_thread(p, t) {
			get_task_struct(t);
			vp = get_vendor_task_struct(t);
			raw_spin_lock_irqsave(&vp->lock, irqflags);
			old = vp->group;
			if (old == new || t->flags & PF_EXITING) {
				raw_spin_unlock_irqrestore(&vp->lock, irqflags);
				put_task_struct(t);
				continue;
			}
			if (vp->queued_to_list == LIST_QUEUED) {
				remove_from_vendor_group_list(&vp->node, old);
				add_to_vendor_group_list(&vp->node, new);
			}
			vp->group = new;
			raw_spin_unlock_irqrestore(&vp->lock, irqflags);
			if (vg[new].disable_sched_setaffinity)
				__reset_task_affinity(t);
			set_batch_policy(t, old, new);
			if (p->prio >= MAX_RT_PRIO) {
				migrate_boost_prio(t, old, new);
#if IS_ENABLED(CONFIG_USE_VENDOR_GROUP_UTIL)
				migrate_vendor_group_util(t, old, new);
#endif
			}
			for (clamp_id = 0; clamp_id < UCLAMP_CNT; clamp_id++)
				uclamp_update_active(t, clamp_id);
			put_task_struct(t);
		}
		rcu_read_unlock();
		break;
	default:
		break;
	}

	put_task_struct(p);

	return 0;
}

static void apply_adpf_adj_change(struct task_struct *p, int adj)
{
	int ucmax, ucmin, pct, pct2util;
	unsigned long irqflags;
	struct rq_flags rf;
	struct rq *rq;
	struct vendor_task_struct *vtp;

	vtp = get_vendor_task_struct(p);
	vtp->adpf_adj = adj;

	// Reserved 11 bits for uclamp min and max and 10 bits for percentage hint.
	ucmin = adj & 0x7FF;
	ucmax = (adj >> 11) & 0x7FF;
	pct = (adj >> 22) & 0x3FF;
	pct2util = vtp->real_cap_avg * pct / 100;

	ucmin = max(ucmin, pct2util);
	ucmin = min(ucmin, ucmax);

	rq = task_rq_lock(p, &rf);

	ucmin = min(ucmin, (int)SCHED_CAPACITY_SCALE);
	if (p->uclamp[UCLAMP_MIN].active) {
		uclamp_rq_dec_id(rq, p, UCLAMP_MIN);
		uclamp_se_set(&p->uclamp_req[UCLAMP_MIN], ucmin, true);
		uclamp_rq_inc_id(rq, p, UCLAMP_MIN);
	} else {
		uclamp_se_set(&p->uclamp_req[UCLAMP_MIN], ucmin, true);
	}
	vh_sched_setscheduler_uclamp_pixel_mod(NULL, p, UCLAMP_MIN, ucmin);

	task_rq_unlock(rq, p, &rf);

	raw_spin_lock_irqsave(&vtp->lock, irqflags);
	vtp->real_cap_avg = 0;
	vtp->real_cap_total_ns = 0;
	raw_spin_unlock_irqrestore(&vtp->lock, irqflags);
}

static int update_sched_adpf_adjustment(const char *buf, int count)
{
	char *tok, *str1, *str2, *pid_str, *adj_str;
	unsigned int pid;
	int adj;
	struct task_struct *p;

	str1 = kstrndup(buf, count, GFP_KERNEL);
	str2 = str1;

	if (!str2)
		return -ENOMEM;

	while (1) {
		tok = strsep(&str2, ",");

		if (tok == NULL)
			break;

		pid_str = strsep(&tok, ":");
		adj_str = tok;

		if (kstrtouint(pid_str, 0, &pid))
			goto fail;
		if (kstrtoint(adj_str, 0, &adj))
			goto fail;

		rcu_read_lock();
		p = find_task_by_vpid(pid);
		if (!p) {
			kfree(str1);
			rcu_read_unlock();
			return -ESRCH;
		}

		get_task_struct(p);
		if (!check_cred(p)) {
			kfree(str1);
			put_task_struct(p);
			rcu_read_unlock();
			return -EACCES;
		}
		rcu_read_unlock();
		if (get_adpf(p, false))
			apply_adpf_adj_change(p, adj);
		put_task_struct(p);
	}

	kfree(str1);
	return count;
fail:
	kfree(str1);
	return -EINVAL;
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
SET_VENDOR_GROUP_STORE(fg_wi, VG_FOREGROUND_WINDOW);

// Create per-task attribute nodes
PER_TASK_BOOL_ATTRIBUTE(prefer_idle);
PER_TASK_BOOL_ATTRIBUTE(boost_prio);
PER_TASK_BOOL_ATTRIBUTE(prefer_fit);
PER_TASK_BOOL_ATTRIBUTE(adpf);
PER_TASK_BOOL_ATTRIBUTE(preempt_wakeup);
PER_TASK_BOOL_ATTRIBUTE(auto_uclamp_max);
PER_TASK_BOOL_ATTRIBUTE(prefer_high_cap);
PER_TASK_UINT_ATTRIBUTE(rampup_multiplier_set);
PER_TASK_UINT_ATTRIBUTE(rampup_multiplier_clear);
PER_TASK_UINT_ATTRIBUTE(sched_qos_none);
PER_TASK_UINT_ATTRIBUTE(sched_qos_power_efficiency);
PER_TASK_UINT_ATTRIBUTE(sched_qos_sensitive_standard);
PER_TASK_UINT_ATTRIBUTE(sched_qos_sensitive_high);
PER_TASK_UINT_ATTRIBUTE(sched_qos_sensitive_extreme);

static int dump_task_show(struct seq_file *m, void *v)
{
	struct task_struct *p, *t;
	struct vendor_task_struct *vp;
	u64 real_cap_avg;
	unsigned int uclamp_min, uclamp_max, uclamp_eff_min, uclamp_eff_max, adpf_adj;
	enum vendor_group group;
	const char *grp_name = "unknown";
	unsigned int rampup_multiplier;

	seq_printf(m, "pid comm group uclamp_min uclamp_max uclamp_eff_min uclamp_eff_max " \
		   "adpf_adj real_cap_avg sched_qos_user_defined_flag rampup_multiplier\n");

	rcu_read_lock();

	for_each_process_thread(p, t) {
		get_task_struct(t);
		vp = get_vendor_task_struct(t);
		adpf_adj = vp->adpf_adj;
		real_cap_avg = vp->real_cap_avg;
		group = vp->group;
		if (group >= 0 && group < VG_MAX)
			grp_name = GRP_NAME[group];
		uclamp_min = t->uclamp_req[UCLAMP_MIN].value;
		uclamp_max = t->uclamp_req[UCLAMP_MAX].value;
		uclamp_eff_min = uclamp_eff_value_pixel_mod(t, UCLAMP_MIN);
		uclamp_eff_max = uclamp_eff_value_pixel_mod(t, UCLAMP_MAX);
		rampup_multiplier = vp->rampup_multiplier;
		put_task_struct(t);

		seq_printf(m, "%u %s %s %u %u %u %u 0x%X %llu 0x%lx %u\n",
			   t->pid, t->comm, grp_name, uclamp_min, uclamp_max, uclamp_eff_min,
			   uclamp_eff_max, adpf_adj, real_cap_avg, vp->sched_qos_user_defined_flag,
			   rampup_multiplier);
	}

	rcu_read_unlock();

	return 0;
}

PROC_OPS_RO(dump_task);

static int util_threshold_show(struct seq_file *m, void *v)
{
	int i;

	for (i = 0; i < pixel_cpu_num; i++) {
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

	return update_vendor_tunables(buf, count, SCHED_CAPACITY_MARGIN);
}

PROC_OPS_RW(util_threshold);

static int thermal_cap_margin_show(struct seq_file *m, void *v)
{
	int i;

	for (i = 0; i < pixel_cpu_num; i++) {
		seq_printf(m, "%u ", thermal_cap_margin[i]);
	}

	seq_printf(m, "\n");

	return 0;
}

static ssize_t thermal_cap_margin_store(struct file *filp,
							const char __user *ubuf,
							size_t count, loff_t *pos)
{
	char buf[MAX_PROC_SIZE];

	if (count >= sizeof(buf))
		return -EINVAL;

	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;

	buf[count] = '\0';

	return update_vendor_tunables(buf, count, THERMAL_CAP_MARGIN);
}

PROC_OPS_RW(thermal_cap_margin);

static int dvfs_headroom_show(struct seq_file *m, void *v)
{
	int i;

	for (i = 0; i < pixel_cpu_num; i++) {
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

	return update_vendor_tunables(buf, count, SCHED_DVFS_HEADROOM);
}
PROC_OPS_RW(dvfs_headroom);

static int teo_util_threshold_show(struct seq_file *m, void *v)
{
	int i;

	for (i = 0; i < pixel_cpu_num; i++) {
		seq_printf(m, "%lu ", teo_cpu_get_util_threshold(i));
	}

	seq_printf(m, "\n");

	return 0;
}
static ssize_t teo_util_threshold_store(struct file *filp,
					const char __user *ubuf,
					size_t count, loff_t *pos)
{
	char buf[MAX_PROC_SIZE];

	if (count >= sizeof(buf))
		return -EINVAL;

	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;

	buf[count] = '\0';

	return update_teo_util_threshold(buf, count);
}
PROC_OPS_RW(teo_util_threshold);

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

static int auto_dvfs_headroom_enable_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", static_branch_likely(&auto_dvfs_headroom_enable) ? 1 : 0);
	return 0;
}
static ssize_t auto_dvfs_headroom_enable_store(struct file *filp,
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
		static_branch_enable(&auto_dvfs_headroom_enable);
	else
		static_branch_disable(&auto_dvfs_headroom_enable);

	return count;
}
PROC_OPS_RW(auto_dvfs_headroom_enable);

static int auto_migration_margins_enable_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", static_branch_likely(&auto_migration_margins_enable) ? 1 : 0);
	return 0;
}
static ssize_t auto_migration_margins_enable_store(struct file *filp,
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
		static_branch_enable(&auto_migration_margins_enable);
	else
		static_branch_disable(&auto_migration_margins_enable);

	return count;
}
PROC_OPS_RW(auto_migration_margins_enable);

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

static int auto_prefer_idle_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", vendor_sched_auto_prefer_idle ? "true" : "false");

	return 0;
}

static ssize_t auto_prefer_idle_store(struct file *filp, const char __user *ubuf,
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

	vendor_sched_auto_prefer_idle = enable;

	return count;
}

PROC_OPS_RW(auto_prefer_idle);

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

static int skip_prefer_prev_mask_show(struct seq_file *m, void *v)
{
	seq_printf(m, "0x%lx\n", skip_prefer_prev_mask.bits[0]);

	return 0;
}
static ssize_t skip_prefer_prev_mask_store(struct file *filp,
					  const char __user *ubuf,
					  size_t count, loff_t *pos)
{
	int ret;

	ret = cpumask_parse_user(ubuf, count, &skip_prefer_prev_mask);
	if (ret)
		return ret;

	return count;
}
PROC_OPS_RW(skip_prefer_prev_mask);

#if IS_ENABLED(CONFIG_UCLAMP_STATS)
static int uclamp_stats_show(struct seq_file *m, void *v)
{
	int i, j, index;
	struct uclamp_stats *stats;

	seq_printf(m, "V, T(ms), %%\n");
	for (i = 0; i < pixel_cpu_num; i++) {
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
	for (i = 0; i < pixel_cpu_num; i++) {
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
	for (i = 0; i < pixel_cpu_num; i++) {
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

/* uclamp filters controls */
static int uclamp_min_filter_enable_show(struct seq_file *m, void *v)
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

static int uclamp_min_filter_us_show(struct seq_file *m, void *v)
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

static int uclamp_max_filter_enable_show(struct seq_file *m, void *v)
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

static int uclamp_max_filter_divider_show(struct seq_file *m, void *v)
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

static int auto_uclamp_max_show(struct seq_file *m, void *v)
{
	int i;

	for (i = 0; i < pixel_cpu_num; i++) {
		seq_printf(m, "%u ", sched_auto_uclamp_max[i]);
	}

	seq_printf(m, "\n");

	return 0;
}
static ssize_t auto_uclamp_max_store(struct file *filp,
				     const char __user *ubuf,
				     size_t count, loff_t *pos)
{
	char buf[MAX_PROC_SIZE];

	if (count >= sizeof(buf))
		return -EINVAL;

	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;

	buf[count] = '\0';

	return update_vendor_tunables(buf, count, SCHED_AUTO_UCLAMP_MAX);
}
PROC_OPS_RW(auto_uclamp_max);

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

	if (val > SCHED_CAPACITY_SCALE)
		return -EINVAL;

	vendor_sched_util_post_init_scale = val;

	return count;
}
PROC_OPS_RW(util_post_init_scale);

static int adpf_rampup_multiplier_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%u\n", vendor_sched_adpf_rampup_multiplier);
	return 0;
}
static ssize_t adpf_rampup_multiplier_store(struct file *filp,
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

	vendor_sched_adpf_rampup_multiplier = val;

	return count;
}
PROC_OPS_RW(adpf_rampup_multiplier);

static int per_task_iowait_boost_max_value_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", sched_per_task_iowait_boost_max_value);
	return 0;
}
static ssize_t per_task_iowait_boost_max_value_store(struct file *filp,
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

	if (val > SCHED_CAPACITY_SCALE)
		return -EINVAL;

	sched_per_task_iowait_boost_max_value = val;

	return count;
}
PROC_OPS_RW(per_task_iowait_boost_max_value);

static int per_cpu_iowait_boost_max_value_show(struct seq_file *m, void *v)
{
	int i;

	for (i = 0; i < pixel_cpu_num; i++) {
		seq_printf(m, "%u ", sched_per_cpu_iowait_boost_max_value[i]);
	}

	seq_printf(m, "\n");

	return 0;
}
static ssize_t per_cpu_iowait_boost_max_value_store(struct file *filp,
						    const char __user *ubuf,
						    size_t count, loff_t *pos)
{
	char buf[MAX_PROC_SIZE];

	if (count >= sizeof(buf))
		return -EINVAL;

	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;

	buf[count] = '\0';

	return update_vendor_tunables(buf, count, SCHED_IOWAIT_BOOST_MAX);
}
PROC_OPS_RW(per_cpu_iowait_boost_max_value);

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

static int pmu_poll_enable_show(struct seq_file *m, void *v)
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

static int max_load_balance_interval_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%lu\n", max_load_balance_interval);
	return 0;
}
static ssize_t max_load_balance_interval_store(struct file *filp,
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

	vh_sched_max_load_balance_interval = val;
	max_load_balance_interval = val;

	return count;
}
PROC_OPS_RW(max_load_balance_interval);

static int min_granularity_ns_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", sysctl_sched_min_granularity);
	return 0;
}
static ssize_t min_granularity_ns_store(struct file *filp,
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

	vh_sched_min_granularity_ns = val;
	vh_sched_wakeup_granularity_ns = val;
	sysctl_sched_min_granularity = val;
	sysctl_sched_wakeup_granularity = val;

	return count;
}
PROC_OPS_RW(min_granularity_ns);

static int latency_ns_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", sysctl_sched_latency);
	return 0;
}
static ssize_t latency_ns_store(struct file *filp,
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

	vh_sched_latency_ns = val;
	sysctl_sched_latency = val;

	return count;
}
PROC_OPS_RW(latency_ns);

static int enable_hrtick_show(struct seq_file *m, void *v)
{
	bool enabled;

	enabled = static_key_enabled(&sched_feat_keys[__SCHED_FEAT_HRTICK]);
	seq_printf(m, "%d\n", enabled);
	return 0;
}
static ssize_t enable_hrtick_store(struct file *filp,
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

	if (!val) {
		sysctl_sched_features &= ~(1UL << __SCHED_FEAT_HRTICK);
		static_key_disable(&sched_feat_keys[__SCHED_FEAT_HRTICK]);
	} else {
		sysctl_sched_features |= 1UL << __SCHED_FEAT_HRTICK;
		static_key_enable(&sched_feat_keys[__SCHED_FEAT_HRTICK]);
	}

	return count;
}
PROC_OPS_RW(enable_hrtick);

static int skip_inefficient_opps_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", static_branch_likely(&skip_inefficient_opps_enable) ? 1 : 0);
	return 0;
}
static ssize_t skip_inefficient_opps_store(struct file *filp,
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

	if (!val) {
		static_branch_disable(&skip_inefficient_opps_enable);
	} else {
		static_branch_enable(&skip_inefficient_opps_enable);
	}

	return count;
}
PROC_OPS_RW(skip_inefficient_opps);

#if IS_ENABLED(CONFIG_RVH_SCHED_LIB)
extern unsigned long sched_lib_mask_out_val;

static int sched_lib_mask_out_show(struct seq_file *m, void *v)
{
	seq_printf(m, "0x%lx\n", sched_lib_mask_out_val);
	return 0;
}

static ssize_t sched_lib_mask_out_store(struct file *filp,
					const char __user *ubuf,
					size_t count, loff_t *pos)
{
	unsigned long val = 0;
	char buf[MAX_PROC_SIZE];

	if (count >= sizeof(buf))
		return -EINVAL;

	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;

	buf[count] = '\0';

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;

	sched_lib_mask_out_val = val;
	return count;

}

PROC_OPS_RW(sched_lib_mask_out);

extern unsigned long sched_lib_mask_in_val;
static int sched_lib_mask_in_show(struct seq_file *m, void *v)
{
	seq_printf(m, "0x%lx\n", sched_lib_mask_in_val);
	return 0;
}

static ssize_t sched_lib_mask_in_store(struct file *filp,
							const char __user *ubuf,
							size_t count, loff_t *pos)
{
	unsigned long val = 0;
	char buf[MAX_PROC_SIZE];

	if (count >= sizeof(buf))
		return -EINVAL;

	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;

	buf[count] = '\0';

	if (kstrtoul(buf, 0, & val))
		return -EINVAL;

	sched_lib_mask_in_val = val;
	return count;
}

PROC_OPS_RW(sched_lib_mask_in);

extern ssize_t sched_lib_name_store(struct file *filp,
				const char __user *ubuffer, size_t count,
				loff_t *ppos);
extern int sched_lib_name_show(struct seq_file *m, void *v);

PROC_OPS_RW(sched_lib_name);

extern bool disable_sched_setaffinity;
static int disable_sched_setaffinity_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", disable_sched_setaffinity);
	return 0;
}
static ssize_t disable_sched_setaffinity_store(struct file *filp,
						   const char __user *ubuf,
						   size_t count, loff_t *pos)
{
	bool val = 0;
	char buf[MAX_PROC_SIZE];

	if (count >= sizeof(buf))
		return -EINVAL;

	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;

	buf[count] = '\0';

	if (kstrtobool(buf, &val))
		return -EINVAL;

	disable_sched_setaffinity = val;
	if (disable_sched_setaffinity)
		reset_sched_setaffinity();
	return count;
}
PROC_OPS_RW(disable_sched_setaffinity);
#endif /* CONFIG_RVH_SCHED_LIB */

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

/* LITTLE idle injection knobs */
static int iidev_little_started;
static int idle_inject_little_trigger_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", iidev_little_started);
	return 0;
}
static ssize_t idle_inject_little_trigger_store(struct file *filp,
						const char __user *ubuf,
						size_t count, loff_t *pos)
{
	int val, ret;

	ret = kstrtoint_from_user(ubuf, count, 0, &val);
	if (ret)
		return ret;

	if (val == iidev_little_started)
		goto out;

	iidev_little_started = !!val;

	if (!iidev_little_started)
		idle_inject_stop(iidev_l);
	else
		idle_inject_start(iidev_l);

out:
	return count;
}
PROC_OPS_RW(idle_inject_little_trigger);

static int idle_inject_little_run_duration_us_show(struct seq_file *m, void *v)
{
	unsigned int run_duration = 0;
	unsigned int idle_duration = 0;

	idle_inject_get_duration(iidev_l, &run_duration, &idle_duration);

	seq_printf(m, "%u\n", run_duration);
	return 0;
}
static ssize_t idle_inject_little_run_duration_us_store(struct file *filp,
							 const char __user *ubuf,
							 size_t count, loff_t *pos)
{
	unsigned int run_duration = 0;
	unsigned int idle_duration = 0;
	int val, ret;

	ret = kstrtouint_from_user(ubuf, count, 0, &val);
	if (ret)
		return ret;

	idle_inject_get_duration(iidev_l, &run_duration, &idle_duration);
	run_duration = val;
	idle_inject_set_duration(iidev_l, run_duration, idle_duration);

	return count;
}
PROC_OPS_RW(idle_inject_little_run_duration_us);

static int idle_inject_little_idle_duration_us_show(struct seq_file *m, void *v)
{
	unsigned int run_duration = 0;
	unsigned int idle_duration = 0;

	idle_inject_get_duration(iidev_l, &run_duration, &idle_duration);

	seq_printf(m, "%u\n", idle_duration);
	return 0;
}
static ssize_t idle_inject_little_idle_duration_us_store(struct file *filp,
							 const char __user *ubuf,
							 size_t count, loff_t *pos)
{
	unsigned int run_duration = 0;
	unsigned int idle_duration = 0;
	int val, ret;

	ret = kstrtouint_from_user(ubuf, count, 0, &val);
	if (ret)
		return ret;

	idle_inject_get_duration(iidev_l, &run_duration, &idle_duration);
	idle_duration = val;
	idle_inject_set_duration(iidev_l, run_duration, idle_duration);

	return count;
}
PROC_OPS_RW(idle_inject_little_idle_duration_us);

static ssize_t idle_inject_little_latency_us_store(struct file *filp,
						const char __user *ubuf,
						size_t count, loff_t *pos)
{
	int val, ret;

	ret = kstrtouint_from_user(ubuf, count, 0, &val);
	if (ret)
		return ret;

	idle_inject_set_latency(iidev_l, val);

	return count;
}
PROC_OPS_WO(idle_inject_little_latency_us);

/* MID idle injections knobs */
static int iidev_mid_started;
static int idle_inject_mid_trigger_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", iidev_mid_started);
	return 0;
}
static ssize_t idle_inject_mid_trigger_store(struct file *filp,
					     const char __user *ubuf,
					     size_t count, loff_t *pos)
{
	int val, ret;

	ret = kstrtoint_from_user(ubuf, count, 0, &val);
	if (ret)
		return ret;

	if (val == iidev_mid_started)
		goto out;

	iidev_mid_started = !!val;

	if (!iidev_mid_started)
		idle_inject_stop(iidev_m);
	else
		idle_inject_start(iidev_m);

out:
	return count;
}
PROC_OPS_RW(idle_inject_mid_trigger);

static int idle_inject_mid_run_duration_us_show(struct seq_file *m, void *v)
{
	unsigned int run_duration = 0;
	unsigned int idle_duration = 0;

	idle_inject_get_duration(iidev_m, &run_duration, &idle_duration);

	seq_printf(m, "%u\n", run_duration);
	return 0;
}
static ssize_t idle_inject_mid_run_duration_us_store(struct file *filp,
						     const char __user *ubuf,
						     size_t count, loff_t *pos)
{
	unsigned int run_duration = 0;
	unsigned int idle_duration = 0;
	int val, ret;

	ret = kstrtouint_from_user(ubuf, count, 0, &val);
	if (ret)
		return ret;

	idle_inject_get_duration(iidev_m, &run_duration, &idle_duration);
	run_duration = val;
	idle_inject_set_duration(iidev_m, run_duration, idle_duration);

	return count;
}
PROC_OPS_RW(idle_inject_mid_run_duration_us);

static int idle_inject_mid_idle_duration_us_show(struct seq_file *m, void *v)
{
	unsigned int run_duration = 0;
	unsigned int idle_duration = 0;

	idle_inject_get_duration(iidev_m, &run_duration, &idle_duration);

	seq_printf(m, "%u\n", idle_duration);
	return 0;
}
static ssize_t idle_inject_mid_idle_duration_us_store(struct file *filp,
						      const char __user *ubuf,
						      size_t count, loff_t *pos)
{
	unsigned int run_duration = 0;
	unsigned int idle_duration = 0;
	int val, ret;

	ret = kstrtouint_from_user(ubuf, count, 0, &val);
	if (ret)
		return ret;

	idle_inject_get_duration(iidev_m, &run_duration, &idle_duration);
	idle_duration = val;
	idle_inject_set_duration(iidev_m, run_duration, idle_duration);

	return count;
}
PROC_OPS_RW(idle_inject_mid_idle_duration_us);

static ssize_t idle_inject_mid_latency_us_store(struct file *filp,
						const char __user *ubuf,
						size_t count, loff_t *pos)
{
	int val, ret;

	ret = kstrtouint_from_user(ubuf, count, 0, &val);
	if (ret)
		return ret;

	idle_inject_set_latency(iidev_m, val);

	return count;
}
PROC_OPS_WO(idle_inject_mid_latency_us);

/* BIG idle injections knobs */
static int iidev_big_started;
static int idle_inject_big_trigger_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", iidev_big_started);
	return 0;
}
static ssize_t idle_inject_big_trigger_store(struct file *filp,
					     const char __user *ubuf,
					     size_t count, loff_t *pos)
{
	int val, ret;

	ret = kstrtoint_from_user(ubuf, count, 0, &val);
	if (ret)
		return ret;

	if (val == iidev_big_started)
		goto out;

	iidev_big_started = !!val;

	if (!iidev_big_started)
		idle_inject_stop(iidev_b);
	else
		idle_inject_start(iidev_b);

out:
	return count;
}
PROC_OPS_RW(idle_inject_big_trigger);

static int idle_inject_big_run_duration_us_show(struct seq_file *m, void *v)
{
	unsigned int run_duration = 0;
	unsigned int idle_duration = 0;

	idle_inject_get_duration(iidev_b, &run_duration, &idle_duration);

	seq_printf(m, "%u\n", run_duration);
	return 0;
}
static ssize_t idle_inject_big_run_duration_us_store(struct file *filp,
						     const char __user *ubuf,
						     size_t count, loff_t *pos)
{
	unsigned int run_duration = 0;
	unsigned int idle_duration = 0;
	int val, ret;

	ret = kstrtouint_from_user(ubuf, count, 0, &val);
	if (ret)
		return ret;

	idle_inject_get_duration(iidev_b, &run_duration, &idle_duration);
	run_duration = val;
	idle_inject_set_duration(iidev_b, run_duration, idle_duration);

	return count;
}
PROC_OPS_RW(idle_inject_big_run_duration_us);

static int idle_inject_big_idle_duration_us_show(struct seq_file *m, void *v)
{
	unsigned int run_duration = 0;
	unsigned int idle_duration = 0;

	idle_inject_get_duration(iidev_b, &run_duration, &idle_duration);

	seq_printf(m, "%u\n", idle_duration);
	return 0;
}
static ssize_t idle_inject_big_idle_duration_us_store(struct file *filp,
						      const char __user *ubuf,
						      size_t count, loff_t *pos)
{
	unsigned int run_duration = 0;
	unsigned int idle_duration = 0;
	int val, ret;

	ret = kstrtouint_from_user(ubuf, count, 0, &val);
	if (ret)
		return ret;

	idle_inject_get_duration(iidev_b, &run_duration, &idle_duration);
	idle_duration = val;
	idle_inject_set_duration(iidev_b, run_duration, idle_duration);

	return count;
}
PROC_OPS_RW(idle_inject_big_idle_duration_us);

static ssize_t idle_inject_big_latency_us_store(struct file *filp,
						const char __user *ubuf,
						size_t count, loff_t *pos)
{
	int val, ret;

	ret = kstrtouint_from_user(ubuf, count, 0, &val);
	if (ret)
		return ret;

	idle_inject_set_latency(iidev_b, val);

	return count;
}
PROC_OPS_WO(idle_inject_big_latency_us);

/* Sync Trigger mid and big clusters */
static int idle_inject_sync_trigger_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", iidev_mid_started && iidev_big_started);
	return 0;
}
static ssize_t idle_inject_sync_trigger_store(struct file *filp,
					      const char __user *ubuf,
					      size_t count, loff_t *pos)
{
	int val, ret;

	ret = kstrtoint_from_user(ubuf, count, 0, &val);
	if (ret)
		return ret;

	if (val == iidev_little_started &&
	    val == iidev_mid_started &&
	    val == iidev_big_started)
		goto out;

	iidev_little_started = !!val;
	iidev_mid_started = !!val;
	iidev_big_started = !!val;

	if (!val) {
		idle_inject_stop(iidev_l);
		idle_inject_stop(iidev_m);
		idle_inject_stop(iidev_b);
	} else {
		idle_inject_start(iidev_l);
		idle_inject_start(iidev_m);
		idle_inject_start(iidev_b);
	}

out:
	return count;
}
PROC_OPS_RW(idle_inject_sync_trigger);

static int cpu_skip_mask_show(struct seq_file *m, void *v)
{
	seq_printf(m, "0x%lx\n", cpu_skip_mask_rt.bits[0]);

	return 0;
}
static ssize_t cpu_skip_mask_store(struct file *filp,
				  const char __user *ubuf,
				  size_t count, loff_t *pos)
{
	int ret;
	unsigned long val;

	ret = kstrtoul_from_user(ubuf, count, 0, &val);
	if (ret)
		return ret;

	cpu_skip_mask_rt.bits[0] = val;

	return count;
}
PROC_OPS_RW(cpu_skip_mask);

int priority_task_name_show(struct seq_file *m, void *v)
{
	unsigned long irqflags;

	spin_lock_irqsave(&priority_task_name_lock, irqflags);
	seq_printf(m, "%s\n", priority_task_name);
	spin_unlock_irqrestore(&priority_task_name_lock, irqflags);
	return 0;
}

/*
 * Accept multiple partial task names with comma separated
 */
ssize_t priority_task_name_store(struct file *filp, const char __user *ubuf, size_t count,
				 loff_t *ppos)
{
	char tmp[sizeof(priority_task_name)];
	unsigned long irqflags;

	if (count >= sizeof(priority_task_name))
		return -EINVAL;

	if (copy_from_user(tmp, ubuf, count))
		return -EFAULT;
	tmp[count] = '\0';

	spin_lock_irqsave(&priority_task_name_lock, irqflags);
	strlcpy(priority_task_name, tmp, sizeof(priority_task_name));
	spin_unlock_irqrestore(&priority_task_name_lock, irqflags);
	return count;
}
PROC_OPS_RW(priority_task_name);

static int priority_task_boost_value_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%u\n", vendor_sched_priority_task_boost_value);
	return 0;
}
static ssize_t priority_task_boost_value_store(struct file *filp, const char __user *ubuf,
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

	if (val > SCHED_CAPACITY_SCALE)
		return -EINVAL;

	vendor_sched_priority_task_boost_value = val;

	return count;
}
PROC_OPS_RW(priority_task_boost_value);

int prefer_idle_task_name_show(struct seq_file *m, void *v)
{

	spin_lock(&prefer_idle_task_name_lock);
	seq_printf(m, "%s\n", prefer_idle_task_name);
	spin_unlock(&prefer_idle_task_name_lock);
	return 0;
}

/*
 * Accept multiple partial task names with comma separated
 */
ssize_t prefer_idle_task_name_store(struct file *filp, const char __user *ubuf, size_t count,
				 loff_t *ppos)
{
	char tmp[sizeof(prefer_idle_task_name)];

	if (count >= sizeof(prefer_idle_task_name))
		return -EINVAL;

	if (copy_from_user(tmp, ubuf, count))
		return -EFAULT;
	tmp[count] = '\0';

	spin_lock(&prefer_idle_task_name_lock);
	strlcpy(prefer_idle_task_name, tmp, sizeof(prefer_idle_task_name));
	spin_unlock(&prefer_idle_task_name_lock);

	if (set_prefer_idle_task_name())
		return -EINVAL;

	return count;
}
PROC_OPS_RW(prefer_idle_task_name);

/*
 * TODO(guibing): remove "is_tgid_system_ui" procfs node once the powerhal
 * switch to use the new "check_tgid_type" procfs node.
 */
static ssize_t is_tgid_system_ui_store(struct file *filp,
						const char __user *ubuf,
						size_t count, loff_t *pos)
{
	unsigned int val;
	char buf[MAX_PROC_SIZE];
	struct task_struct *p;
	char tgid_comm[TASK_COMM_LEN] = {0};

	if (count >= sizeof(buf))
		return -EINVAL;

	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;

	buf[count] = '\0';

	if (kstrtouint(buf, 0, &val) || val > PID_MAX_LIMIT)
		return -EINVAL;

	rcu_read_lock();
	p = find_task_by_vpid(val);
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

	strlcpy(tgid_comm, p->comm, TASK_COMM_LEN);
	put_task_struct(p);
	rcu_read_unlock();

	if (strstr(tgid_comm, "systemui") || strstr(tgid_comm, "nexuslauncher")) {
		return count;
	}  else {
		return -ENOMSG;
	}
}
PROC_OPS_WO(is_tgid_system_ui);

/*
 * Check the type of applications to which the tgid belongs.
 * normal return values:
 *   1 : systemui or nexuslauncher related
 *   2 : chrome related
 *   -ENOMSG : others
 *
 * It's designed intentionally not to return the number of written characters, so it could be used
 * to check different types of the tgid tasks based on its return value. This helps reducing the
 * number of syscalls when used frequently in user space.
 */
static ssize_t check_tgid_type_store(struct file *filp,
					const char __user *ubuf,
					size_t count, loff_t *pos)
{
	unsigned int val;
	char buf[MAX_PROC_SIZE];
	struct task_struct *p;
	char tgid_comm[TASK_COMM_LEN] = {0};

	if (count >= sizeof(buf))
		return -EINVAL;

	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;

	buf[count] = '\0';

	if (kstrtouint(buf, 0, &val) || val > PID_MAX_LIMIT)
		return -EINVAL;

	rcu_read_lock();
	p = find_task_by_vpid(val);
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

	strlcpy(tgid_comm, p->comm, TASK_COMM_LEN);
	put_task_struct(p);
	rcu_read_unlock();

	if (strstr(tgid_comm, "systemui") || strstr(tgid_comm, "nexuslauncher")) {
		return 1;
	} else if (strstr(tgid_comm, "android.chrome") || strstr(tgid_comm, "ileged_process")) {
		return 2;
	} else {
		return -ENOMSG;
	}
}
PROC_OPS_WO(check_tgid_type);

int boost_at_fork_task_name_show(struct seq_file *m, void *v)
{
	unsigned long irqflags;

	raw_spin_lock_irqsave(&boost_at_fork_task_name_lock, irqflags);
	seq_printf(m, "%s\n", boost_at_fork_task_name);
	raw_spin_unlock_irqrestore(&boost_at_fork_task_name_lock, irqflags);
	return 0;
}

/*
 * Accepts a single value only.
 */
ssize_t boost_at_fork_task_name_store(struct file *filp, const char __user *ubuf, size_t count,
				 loff_t *ppos)
{
	char tmp[sizeof(boost_at_fork_task_name)];
	unsigned long irqflags;

	if (count >= sizeof(boost_at_fork_task_name))
		return -EINVAL;

	if (copy_from_user(tmp, ubuf, count))
		return -EFAULT;
	tmp[count] = '\0';

	raw_spin_lock_irqsave(&boost_at_fork_task_name_lock, irqflags);
	strlcpy(boost_at_fork_task_name, tmp, sizeof(boost_at_fork_task_name));
	raw_spin_unlock_irqrestore(&boost_at_fork_task_name_lock, irqflags);
	return count;
}
PROC_OPS_RW(boost_at_fork_task_name);

static int boost_at_fork_value_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%lu\n", vendor_sched_boost_at_fork_value);
	return 0;
}
static ssize_t boost_at_fork_value_store(struct file *filp,
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

	if (val > SCHED_CAPACITY_SCALE)
		return -EINVAL;

	vendor_sched_boost_at_fork_value = val;

	return count;
}
PROC_OPS_RW(boost_at_fork_value);

static ssize_t adpf_adjustment_store(struct file *filp,
				  const char __user *ubuf,
				  size_t count, loff_t *pos)
{
	char buf[MAX_PROC_SIZE];

	if (count >= sizeof(buf))
		return -EINVAL;

	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;

	buf[count] = '\0';

	return update_sched_adpf_adjustment(buf, count);
}
PROC_OPS_WO(adpf_adjustment);

struct pentry {
	const char *name;
	enum vendor_procfs_type type;
	/*
	 * Vendor group the procfs belongs to.
	 *  -1 if it doesn't follow into any group.
	 */
	const int vg;
	const struct proc_ops *fops;
};
static struct pentry entries[] = {
	PROC_GROUP_ENTRIES(sys, VG_SYSTEM),
	PROC_GROUP_ENTRIES(ta, VG_TOPAPP),
	PROC_GROUP_ENTRIES(fg, VG_FOREGROUND),
	PROC_GROUP_ENTRIES(cam, VG_CAMERA),
	PROC_GROUP_ENTRIES(cam_power, VG_CAMERA_POWER),
	PROC_GROUP_ENTRIES(bg, VG_BACKGROUND),
	PROC_GROUP_ENTRIES(sysbg, VG_SYSTEM_BACKGROUND),
	PROC_GROUP_ENTRIES(nnapi, VG_NNAPI_HAL),
	PROC_GROUP_ENTRIES(rt, VG_RT),
	PROC_GROUP_ENTRIES(dex2oat, VG_DEX2OAT),
	PROC_GROUP_ENTRIES(ota, VG_OTA),
	PROC_GROUP_ENTRIES(sf, VG_SF),
	PROC_GROUP_ENTRIES(fg_wi, VG_FOREGROUND_WINDOW),
	// sched qos attributes
	PROC_SCHED_QOS_ENTRY(boost_prio_set),
	PROC_SCHED_QOS_ENTRY(boost_prio_clear),
	PROC_SCHED_QOS_ENTRY(prefer_fit_set),
	PROC_SCHED_QOS_ENTRY(prefer_fit_clear),
	PROC_SCHED_QOS_ENTRY(prefer_idle_set),
	PROC_SCHED_QOS_ENTRY(prefer_idle_clear),
	PROC_SCHED_QOS_ENTRY(adpf_set),
	PROC_SCHED_QOS_ENTRY(adpf_clear),
	PROC_SCHED_QOS_ENTRY(preempt_wakeup_set),
	PROC_SCHED_QOS_ENTRY(preempt_wakeup_clear),
	PROC_SCHED_QOS_ENTRY(auto_uclamp_max_set),
	PROC_SCHED_QOS_ENTRY(auto_uclamp_max_clear),
	PROC_SCHED_QOS_ENTRY(prefer_high_cap_set),
	PROC_SCHED_QOS_ENTRY(prefer_high_cap_clear),
	PROC_SCHED_QOS_ENTRY(rampup_multiplier_set),
	PROC_SCHED_QOS_ENTRY(rampup_multiplier_clear),
	PROC_SCHED_QOS_ENTRY(sched_qos_none),
	PROC_SCHED_QOS_ENTRY(sched_qos_power_efficiency),
	PROC_SCHED_QOS_ENTRY(sched_qos_sensitive_standard),
	PROC_SCHED_QOS_ENTRY(sched_qos_sensitive_high),
	PROC_SCHED_QOS_ENTRY(sched_qos_sensitive_extreme),
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
	// Uclamp stats
#if IS_ENABLED(CONFIG_UCLAMP_STATS)
	PROC_ENTRY(uclamp_stats),
	PROC_ENTRY(uclamp_effective_stats),
	PROC_ENTRY(uclamp_util_diff_stats),
	PROC_ENTRY(reset_uclamp_stats),
#endif
	PROC_ENTRY(util_threshold),
	PROC_ENTRY(thermal_cap_margin),
	PROC_ENTRY(util_post_init_scale),
	PROC_ENTRY(npi_packing),
	PROC_ENTRY(reduce_prefer_idle),
	PROC_ENTRY(auto_prefer_idle),
	PROC_ENTRY(boost_adpf_prio),
	PROC_ENTRY(dump_task),
	// pmu limit attribute
	PROC_ENTRY(pmu_poll_time),
	PROC_ENTRY(pmu_poll_enable),
#if IS_ENABLED(CONFIG_RVH_SCHED_LIB)
	// sched lib
	PROC_ENTRY(sched_lib_mask_out),
	PROC_ENTRY(sched_lib_mask_in),
	PROC_ENTRY(sched_lib_name),
	PROC_ENTRY(disable_sched_setaffinity),
#endif /* CONFIG_RVH_SCHED_LIB */
	// uclamp filter
	PROC_ENTRY(uclamp_min_filter_enable),
	PROC_ENTRY(uclamp_min_filter_us),
	PROC_ENTRY(uclamp_min_filter_rt),
	PROC_ENTRY(uclamp_max_filter_enable),
	PROC_ENTRY(uclamp_max_filter_divider),
	PROC_ENTRY(uclamp_max_filter_rt),
	PROC_ENTRY(auto_uclamp_max),
	PROC_ENTRY(adpf_adjustment),
	// dvfs headroom
	PROC_ENTRY(dvfs_headroom),
	PROC_ENTRY(tapered_dvfs_headroom_enable),
	PROC_ENTRY(auto_dvfs_headroom_enable),
	PROC_ENTRY(adpf_rampup_multiplier),
	// teo
	PROC_ENTRY(teo_util_threshold),
	// iowait boost
	PROC_ENTRY(per_task_iowait_boost_max_value),
	PROC_ENTRY(per_cpu_iowait_boost_max_value),
	// load balance
	PROC_ENTRY(max_load_balance_interval),
	PROC_ENTRY(min_granularity_ns),
	PROC_ENTRY(latency_ns),
	PROC_ENTRY(enable_hrtick),
	// auto migration margins
	PROC_ENTRY(auto_migration_margins_enable),
	// idle injection
	PROC_ENTRY(idle_inject_little_trigger),
	PROC_ENTRY(idle_inject_little_run_duration_us),
	PROC_ENTRY(idle_inject_little_idle_duration_us),
	PROC_ENTRY(idle_inject_little_latency_us),
	PROC_ENTRY(idle_inject_mid_trigger),
	PROC_ENTRY(idle_inject_mid_run_duration_us),
	PROC_ENTRY(idle_inject_mid_idle_duration_us),
	PROC_ENTRY(idle_inject_mid_latency_us),
	PROC_ENTRY(idle_inject_big_trigger),
	PROC_ENTRY(idle_inject_big_run_duration_us),
	PROC_ENTRY(idle_inject_big_idle_duration_us),
	PROC_ENTRY(idle_inject_big_latency_us),
	PROC_ENTRY(idle_inject_sync_trigger),
	// pixel_em
	PROC_ENTRY(skip_inefficient_opps),
	// skip mask for RT wake up
	PROC_ENTRY(cpu_skip_mask),
	// skip mask for prefer prev cpu
	PROC_ENTRY(skip_prefer_prev_mask),
	// names for the priority task
	PROC_ENTRY(priority_task_name),
	// boost value for the priority task
	PROC_ENTRY(priority_task_boost_value),
	// names for the prefer_idle task
	PROC_ENTRY(prefer_idle_task_name),
	// check whether tgid belongs to systemui/nexuslauncher
	PROC_ENTRY(is_tgid_system_ui),
	/* boost at fork */
	PROC_ENTRY(boost_at_fork_task_name),
	PROC_ENTRY(boost_at_fork_value),
	// check the type of application to which the tgid belongs
	PROC_ENTRY(check_tgid_type),
};


int create_procfs_node(void)
{
	int i;
	struct uclamp_se uc_max = {};
	enum uclamp_id clamp_id;
	struct proc_dir_entry *parent_directory;
	struct proc_dir_entry *group_root_dir;
	struct proc_dir_entry *sched_qos_dir;
	cpumask_t cpumask;

	/* create vendor sched root directory */
	vendor_sched = proc_mkdir("vendor_sched", NULL);
	if (!vendor_sched)
		goto out;

	/* create vendor group directories */
	group_root_dir = proc_mkdir("groups", vendor_sched);
	if (!group_root_dir)
		goto out;

	/* create sched qos directory */
	sched_qos_dir = proc_mkdir("sched_qos", vendor_sched);
	if (!sched_qos_dir)
		goto out;

	for (i = 0; i < VG_MAX; i++) {
		group_dirs[i] = proc_mkdir(GRP_NAME[i], group_root_dir);
		if (!group_dirs[i]) {
			goto out;
		}
	}

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

		if (entries[i].type == GROUPED_CONTROL) {
			if (entries[i].vg >= 0 && entries[i].vg < VG_MAX) {
				parent_directory = group_dirs[entries[i].vg];
			} else {
				parent_directory = group_root_dir;
			}
		} else if (entries[i].type == SCHED_QOS_CONTROL) {
			parent_directory = sched_qos_dir;
		} else {
			parent_directory = vendor_sched;
		}

		if (!proc_create(entries[i].name, mode,
					parent_directory, entries[i].fops)) {
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

	/* Register idle injection */
	cpumask_clear(&cpumask);
	for (i = pixel_cluster_start_cpu[0]; i < pixel_cluster_start_cpu[1]; i++)
		cpumask_set_cpu(i, &cpumask);
	iidev_l = idle_inject_register(&cpumask);
	if (!iidev_l)
		goto out;
	idle_inject_set_duration(iidev_l, 2000, 14000);
	idle_inject_set_latency(iidev_l, 5000);

	cpumask_clear(&cpumask);
	for (i = pixel_cluster_start_cpu[1]; i < pixel_cluster_start_cpu[2]; i++)
		cpumask_set_cpu(i, &cpumask);
	iidev_m = idle_inject_register(&cpumask);
	if (!iidev_m)
		goto out;
	idle_inject_set_duration(iidev_m, 2000, 14000);
	idle_inject_set_latency(iidev_m, 5000);

	cpumask_clear(&cpumask);
	for (i = pixel_cluster_start_cpu[2]; i < pixel_cpu_num; i++)
		cpumask_set_cpu(i, &cpumask);
	iidev_b = idle_inject_register(&cpumask);
	if (!iidev_b)
		goto out;
	idle_inject_set_duration(iidev_b, 2000, 14000);
	idle_inject_set_latency(iidev_b, 5000);

	return 0;

out:
	return -ENOMEM;
}
