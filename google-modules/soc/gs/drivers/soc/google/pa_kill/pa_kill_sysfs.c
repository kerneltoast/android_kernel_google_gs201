#define pr_fmt(fmt) "pa_kill: " fmt

#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/atomic.h>
#include <linux/sched/task.h>
#include "pa_kill_sysfs.h"
#include "pa_kill_core.h"

extern struct kobject *vendor_mm_kobj;
static struct kobject *pa_kill_parent_kobj;
static struct kobject pa_kill_kobj;
extern void reclaim_memory(unsigned long nr_demand_pages);
extern void destroy_kill_threads(void);
extern int create_kill_threads(unsigned int nr_thread);
extern unsigned long extra_free_kb;
extern atomic_long_t pa_kill_count;
atomic_long_t pa_nr_attempt;
extern atomic_long_t pa_nr_done;
extern unsigned int poll_interval_ms;
extern unsigned int killable_min_oom_adj;
extern bool movable_allowable;
extern unsigned int nr_kill_thread;

extern void pa_set_cpu_affinity(void);
extern cpumask_t pa_task_cpu_affinity;
static DEFINE_MUTEX(sysfs_lock);

/*
 * This all compiles without CONFIG_SYSFS, but is a waste of space.
 */
#define PA_KILL_ATTR_RO(_name) \
	static struct kobj_attribute _name##_attr = __ATTR_RO(_name)

#define PA_KILL_ATTR_RW(_name) \
	static struct kobj_attribute _name##_attr = __ATTR_RW(_name)

#define PA_KILL_ATTR_WO(_name) \
	static struct kobj_attribute _name##_attr = __ATTR_WO(_name)

static ssize_t reclaim_kb_store(struct kobject *kobj,
			  struct kobj_attribute *attr,
			  const char *buf, size_t len)
{
	unsigned long reclaim_kb;

	if (kstrtoul(buf, 10, &reclaim_kb))
		return -EINVAL;

	if (reclaim_kb > 0) {
		atomic_long_inc(&pa_nr_attempt);
		reclaim_memory(reclaim_kb >> (PAGE_SHIFT - 10));
	}

	return len;
}
PA_KILL_ATTR_WO(reclaim_kb);

static ssize_t extra_free_kb_store(struct kobject *kobj,
				   struct kobj_attribute *attr,
				   const char *buf, size_t len)
{
	unsigned long val;

	mutex_lock(&sysfs_lock);
	if (kstrtoul(buf, 10, &val)) {
		mutex_unlock(&sysfs_lock);
		return -EINVAL;
	}

	extra_free_kb = val;
	mutex_unlock(&sysfs_lock);

	return len;
}

static ssize_t extra_free_kb_show(struct kobject *kobj,
				  struct kobj_attribute *attr,
				  char *buf)
{
	unsigned long val;

	mutex_lock(&sysfs_lock);
	val = extra_free_kb;
	mutex_unlock(&sysfs_lock);

	return sysfs_emit(buf, "%lu\n", val);
}
PA_KILL_ATTR_RW(extra_free_kb);

static ssize_t poll_interval_ms_store(struct kobject *kobj,
				      struct kobj_attribute *attr,
				      const char *buf, size_t len)
{
	unsigned int val;

	mutex_lock(&sysfs_lock);
	if (kstrtouint(buf, 10, &val)) {
		mutex_unlock(&sysfs_lock);
		return -EINVAL;
	}


	poll_interval_ms = val;
	mutex_unlock(&sysfs_lock);

	return len;
}

static ssize_t poll_interval_ms_show(struct kobject *kobj,
				     struct kobj_attribute *attr,
				     char *buf)
{
	unsigned int val;

	mutex_lock(&sysfs_lock);
	val = poll_interval_ms;
	mutex_unlock(&sysfs_lock);

	return sysfs_emit(buf, "%u\n", val);
}
PA_KILL_ATTR_RW(poll_interval_ms);

static ssize_t cpu_affinity_store(struct kobject *kobj,
				  struct kobj_attribute *attr, const char *buf, size_t len)
{
	cpumask_t cpumask;
	int ret;

	mutex_lock(&sysfs_lock);
	ret = cpumask_parse(buf, &cpumask);
	if (ret < 0 || cpumask_empty(&cpumask)) {
		mutex_unlock(&sysfs_lock);
		return -EINVAL;
	}

	cpumask_and(&pa_task_cpu_affinity, &cpumask, cpu_possible_mask);
	pa_set_cpu_affinity();
	mutex_unlock(&sysfs_lock);

	return len;
}

static ssize_t cpu_affinity_show(struct kobject *kobj,
				 struct kobj_attribute *attr, char *buf)
{
	ssize_t ret;

	mutex_lock(&sysfs_lock);
	ret = cpumap_print_to_pagebuf(false, buf, &pa_task_cpu_affinity);
	mutex_unlock(&sysfs_lock);

	return ret;
}
PA_KILL_ATTR_RW(cpu_affinity);

static ssize_t killable_min_oom_adj_store(struct kobject *kobj,
			       struct kobj_attribute *attr,
			       const char *buf, size_t len)
{
	unsigned int val;
	int ret = -EINVAL;

	mutex_lock(&sysfs_lock);
	if (kstrtouint(buf, 10, &val))
		goto out;

	if (val == OOM_SCORE_ADJ_MIN)
		goto out;

	killable_min_oom_adj = val;
	ret = len;
out:
	mutex_unlock(&sysfs_lock);

	return ret;
}

static ssize_t killable_min_oom_adj_show(struct kobject *kobj,
			       struct kobj_attribute *attr,
			       char *buf)
{
	unsigned int val;

	mutex_lock(&sysfs_lock);
	val = killable_min_oom_adj;
	mutex_unlock(&sysfs_lock);

	return sysfs_emit(buf, "%d\n", val);
}
PA_KILL_ATTR_RW(killable_min_oom_adj);

static ssize_t kill_count_store(struct kobject *kobj,
			       struct kobj_attribute *attr,
			       const char *buf, size_t len)
{
	atomic_long_set(&pa_kill_count, 0);

	return len;
}

static ssize_t kill_count_show(struct kobject *kobj,
			       struct kobj_attribute *attr,
			       char *buf)
{
	unsigned long val;

	val = atomic_long_read(&pa_kill_count);

	return sysfs_emit(buf, "%lu\n", val);
}
PA_KILL_ATTR_RW(kill_count);

static ssize_t nr_attempt_show(struct kobject *kobj,
				  struct kobj_attribute *attr,
				  char *buf)
{
	unsigned long val;

	val = atomic_long_read(&pa_nr_attempt);

	return sysfs_emit(buf, "%lu\n", val);
}
PA_KILL_ATTR_RO(nr_attempt);

static ssize_t nr_done_show(struct kobject *kobj,
				    struct kobj_attribute *attr,
				    char *buf)
{
	unsigned long val;

	val = atomic_long_read(&pa_nr_done);

	return sysfs_emit(buf, "%lu\n", val);
}
PA_KILL_ATTR_RO(nr_done);

static ssize_t movable_allowable_store(struct kobject *kobj,
			       struct kobj_attribute *attr,
			       const char *buf, size_t len)
{
	unsigned int val;

	mutex_lock(&sysfs_lock);
	if (kstrtouint(buf, 10, &val)) {
		mutex_unlock(&sysfs_lock);
		return -EINVAL;
	}

	if (!val)
		movable_allowable = true;
	else
		movable_allowable = false;
	mutex_unlock(&sysfs_lock);

	return len;
}

static ssize_t movable_allowable_show(struct kobject *kobj,
			       struct kobj_attribute *attr,
			       char *buf)
{
	bool ret;

	mutex_lock(&sysfs_lock);
	ret = movable_allowable;
	mutex_unlock(&sysfs_lock);

	return sysfs_emit(buf, "%d\n", ret);
}
PA_KILL_ATTR_RW(movable_allowable);

static ssize_t nr_kill_thread_store(struct kobject *kobj,
			       struct kobj_attribute *attr,
			       const char *buf, size_t len)
{
	unsigned int val;
	ssize_t ret = len;

	mutex_lock(&sysfs_lock);
	if (kstrtouint(buf, 10, &val)) {
		ret = -EINVAL;
		goto out;
	}

	if (val > num_online_cpus()) {
		ret = -EINVAL;
		goto out;
	}

	ret = len;
	if (val != nr_kill_thread) {
		int err;

		destroy_kill_threads();
		err = create_kill_threads(val);
		if (err)
			ret = err;
	}
out:
	mutex_unlock(&sysfs_lock);

	return ret;
}

static ssize_t nr_kill_thread_show(struct kobject *kobj,
			       struct kobj_attribute *attr,
			       char *buf)
{
	unsigned long val;

	mutex_lock(&sysfs_lock);
	val = nr_kill_thread;
	mutex_unlock(&sysfs_lock);

	return sysfs_emit(buf, "%lu\n", val);
}
PA_KILL_ATTR_RW(nr_kill_thread);

static struct attribute *pa_kill_attrs[] = {
	&reclaim_kb_attr.attr,
	&extra_free_kb_attr.attr,
	&poll_interval_ms_attr.attr,
	&cpu_affinity_attr.attr,
	&killable_min_oom_adj_attr.attr,
	&kill_count_attr.attr,
	&nr_attempt_attr.attr,
	&nr_done_attr.attr,
	&movable_allowable_attr.attr,
	&nr_kill_thread_attr.attr,
	NULL,
};

static const struct attribute_group pa_kill_attr_group = {
	.attrs = pa_kill_attrs,
};

static const struct attribute_group *pa_kill_attr_groups[] = {
	&pa_kill_attr_group,
	NULL,
};

static void pa_kill_kobj_release(struct kobject *obj)
{
	/* Never released the static objects */
}

static struct kobj_type pa_kill_ktype = {
	.release = pa_kill_kobj_release,
	.sysfs_ops = &kobj_sysfs_ops,
	.default_groups = pa_kill_attr_groups,
};

int __init pa_kill_sysfs_init(void)
{
#ifdef CONFIG_ANDROID_VENDOR_HOOKS
	pa_kill_parent_kobj = vendor_mm_kobj;
#else
	pa_kill_parent_kobj = mm_kobj;
#endif
	return kobject_init_and_add(&pa_kill_kobj, &pa_kill_ktype, pa_kill_parent_kobj, "pa_kill");
}
