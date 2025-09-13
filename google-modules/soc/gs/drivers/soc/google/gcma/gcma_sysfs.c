#include <linux/kobject.h>
#include <linux/sysfs.h>
#include "gcma_sysfs.h"

#if IS_ENABLED(CONFIG_VH_MM)
extern struct kobject *vendor_mm_kobj;
#endif
static struct kobject *gcma_parent_kobj;
static struct kobject gcma_kobj;

static atomic64_t gcma_stats[NUM_OF_GCMA_STAT];

enum LATENCY_LEVEL {
	/* Do not change order */
	LATENCY_LOW,
	LATENCY_MID,
	LATENCY_HIGH,
	LATENCY_EXTREME_HIGH,
	LATENCY_NUM_LEVELS,
};

/* gcma_alloc_range per-page latency by ns */
static struct gcma_alloc_latency {
	unsigned long stat[LATENCY_NUM_LEVELS];
	unsigned long threshold[LATENCY_NUM_LEVELS];
	spinlock_t lock;
} alloc_latency = {
	.lock = __SPIN_LOCK_UNLOCKED(alloc_latency.lock),
	.threshold = {1000, 2000, 4000, ULONG_MAX},
};

void inc_gcma_stat(enum gcma_stat_type type)
{
	atomic64_inc(&gcma_stats[type]);
}

void dec_gcma_stat(enum gcma_stat_type type)
{
	atomic64_dec(&gcma_stats[type]);
}

void add_gcma_stat(enum gcma_stat_type type, unsigned long delta)
{
	atomic64_add(delta, &gcma_stats[type]);
}

void account_gcma_per_page_alloc_latency(unsigned long count,
					 unsigned long latency_ns)
{
	unsigned long pp_latency = latency_ns / count;

	spin_lock_irq(&alloc_latency.lock);
	if (pp_latency < alloc_latency.threshold[LATENCY_LOW])
		alloc_latency.stat[LATENCY_LOW]++;
	else if (pp_latency < alloc_latency.threshold[LATENCY_MID])
		alloc_latency.stat[LATENCY_MID]++;
	else if (pp_latency < alloc_latency.threshold[LATENCY_HIGH])
		alloc_latency.stat[LATENCY_HIGH]++;
	else
		alloc_latency.stat[LATENCY_EXTREME_HIGH]++;
	spin_unlock_irq(&alloc_latency.lock);
}

/*
 * This all compiles without CONFIG_SYSFS, but is a waste of space.
 */

#define GCMA_ATTR_RO(_name) \
	static struct kobj_attribute _name##_attr = __ATTR_RO(_name)

#define GCMA_ATTR_RW(_name) \
	static struct kobj_attribute _name##_attr = __ATTR_RW(_name)

static ssize_t stored_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%llu\n", (u64)atomic64_read(&gcma_stats[STORED_PAGE]));
}
GCMA_ATTR_RO(stored);

static ssize_t loaded_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%llu\n", (u64)atomic64_read(&gcma_stats[LOADED_PAGE]));
}
GCMA_ATTR_RO(loaded);

static ssize_t evicted_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%llu\n", (u64)atomic64_read(&gcma_stats[EVICTED_PAGE]));
}
GCMA_ATTR_RO(evicted);

static ssize_t cached_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%llu\n", (u64)atomic64_read(&gcma_stats[CACHED_PAGE]));
}
GCMA_ATTR_RO(cached);

static ssize_t discarded_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%llu\n", (u64)atomic64_read(&gcma_stats[DISCARDED_PAGE]));
}
GCMA_ATTR_RO(discarded);

static struct attribute *gcma_attrs[] = {
	&stored_attr.attr,
	&loaded_attr.attr,
	&evicted_attr.attr,
	&cached_attr.attr,
	&discarded_attr.attr,
	NULL,
};

static const struct attribute_group gcma_attr_group = {
	.attrs = gcma_attrs,
};

static inline unsigned long get_latency_stat(enum LATENCY_LEVEL level)
{
	unsigned long val;

	spin_lock_irq(&alloc_latency.lock);
	val = alloc_latency.stat[level];
	spin_unlock_irq(&alloc_latency.lock);

	return val;
}

static inline unsigned long get_latency_threshold(enum LATENCY_LEVEL level)
{
	unsigned long val;

	spin_lock_irq(&alloc_latency.lock);
	val = alloc_latency.threshold[level];
	spin_unlock_irq(&alloc_latency.lock);

	return val;
}

static inline int set_latency_threshold(enum LATENCY_LEVEL level, unsigned long latency)
{
	if (level == LATENCY_EXTREME_HIGH)
		return -EPERM;

	spin_lock_irq(&alloc_latency.lock);
	if (level != LATENCY_LOW &&
			alloc_latency.threshold[level - 1] >= latency) {
		spin_unlock_irq(&alloc_latency.lock);
		return -EINVAL;
	}

	alloc_latency.threshold[level] = latency;
	spin_unlock_irq(&alloc_latency.lock);

	return 0;
}

#define LATENCY_ATTR(level_name, sysfs_name)					\
static ssize_t sysfs_name##_show(struct kobject *kobj,				\
				 struct kobj_attribute *attr, char *buf)	\
{										\
	return sysfs_emit(buf, "%lu\n", get_latency_stat(level_name));		\
}										\
										\
static ssize_t sysfs_name##_threshold_show(struct kobject *kobj,		\
				 struct kobj_attribute *attr, char *buf)	\
{										\
	return sysfs_emit(buf, "%lu\n",						\
				get_latency_threshold(level_name));		\
}										\
										\
static ssize_t sysfs_name##_threshold_store(struct kobject *kobj,		\
				  struct kobj_attribute *attr,			\
				  const char *buf, size_t len)			\
{										\
	unsigned long threshold;						\
	int err;								\
										\
	if (kstrtoul(buf, 10, &threshold))					\
		return -EINVAL;							\
										\
	err = set_latency_threshold(level_name, threshold);			\
	return err? : len;							\
}										\
GCMA_ATTR_RO(sysfs_name);							\
GCMA_ATTR_RW(sysfs_name##_threshold)						\

LATENCY_ATTR(LATENCY_LOW, latency_low);
LATENCY_ATTR(LATENCY_MID, latency_mid);
LATENCY_ATTR(LATENCY_HIGH, latency_high);
LATENCY_ATTR(LATENCY_EXTREME_HIGH, latency_extreme_high);

static struct attribute *gcma_latency_attrs[] = {
	&latency_low_attr.attr,
	&latency_mid_attr.attr,
	&latency_high_attr.attr,
	&latency_extreme_high_attr.attr,
	NULL,
};

static const struct attribute_group gcma_latency_attr_group = {
	.attrs = gcma_latency_attrs,
};

static struct attribute *gcma_latency_threshold_attrs[] = {
	&latency_low_threshold_attr.attr,
	&latency_mid_threshold_attr.attr,
	&latency_high_threshold_attr.attr,
	&latency_extreme_high_threshold_attr.attr,
	NULL,
};

static const struct attribute_group gcma_latency_threshold_attr_group = {
	.attrs = gcma_latency_threshold_attrs,
};


static const struct attribute_group *gcma_attr_groups[] = {
	&gcma_attr_group,
	&gcma_latency_attr_group,
	&gcma_latency_threshold_attr_group,
	NULL,
};

static void gcma_kobj_release(struct kobject *obj)
{
	/* Never released the static objects */
}

static struct kobj_type gcma_ktype = {
	.release = gcma_kobj_release,
	.sysfs_ops = &kobj_sysfs_ops,
	.default_groups = gcma_attr_groups,
};

int __init gcma_sysfs_init(void)
{
#if IS_ENABLED(CONFIG_VH_MM)
	gcma_parent_kobj = vendor_mm_kobj;
#else
	gcma_parent_kobj = kobject_create_and_add("gcma", kernel_kobj);
#endif
	return kobject_init_and_add(&gcma_kobj, &gcma_ktype, gcma_parent_kobj, "gcma");
}
