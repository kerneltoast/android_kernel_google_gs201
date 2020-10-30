// SPDX-License-Identifier: GPL-2.0-only
/* sysfs_node.c
 *
 * Android Vendor Hook Support
 *
 * Copyright 2020 Google LLC
 */

#include <linux/kobject.h>
#include <linux/sched.h>
#include <linux/sched/task.h>
#include <linux/sysfs.h>

#include "sched.h"

bool __read_mostly vendor_sched_enable_prefer_high_cap;

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
	rcu_read_unlock();

	vp = get_vendor_task_struct(p);
	vp->prefer_high_cap = val;

	put_task_struct(p);

	return 0;
}

static ssize_t set_prefer_high_cap_store(struct kobject *kobj,
					struct kobj_attribute *attr,
					const char *buf, size_t count)
{
	int ret = update_prefer_high_cap(buf, true);

	return ret ?: count;
}

static struct kobj_attribute set_prefer_high_cap_attribute = __ATTR_WO(set_prefer_high_cap);

static ssize_t clear_prefer_high_cap_store(struct kobject *kobj,
					struct kobj_attribute *attr,
					const char *buf, size_t count)
{
	int ret = update_prefer_high_cap(buf, false);

	return ret ?: count;
}

static struct kobj_attribute clear_prefer_high_cap_attribute = __ATTR_WO(clear_prefer_high_cap);

static ssize_t prefer_high_cap_enable_show(struct kobject *kobj,
					struct kobj_attribute *attr,
					char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", vendor_sched_enable_prefer_high_cap);
}

static ssize_t prefer_high_cap_enable_store(struct kobject *kobj,
					struct kobj_attribute *attr,
					const char *buf, size_t count)
{
	bool enable;

	if (kstrtobool(buf, &enable))
		return -EINVAL;

	vendor_sched_enable_prefer_high_cap = enable;

	return count;
}

static struct kobj_attribute prefer_high_cap_enable_attribute = __ATTR_RW(prefer_high_cap_enable);

static struct kobject *vendor_sched_kobj;
static struct attribute *attrs[] = {
	&set_prefer_high_cap_attribute.attr,
	&clear_prefer_high_cap_attribute.attr,
	&prefer_high_cap_enable_attribute.attr,
	NULL,
};
static struct attribute_group attr_group = {
	.attrs = attrs,
};

int create_sysfs_node(void)
{
	int ret;

	vendor_sched_kobj = kobject_create_and_add("vendor_sched", kernel_kobj);

	if (!vendor_sched_kobj)
		return -ENOMEM;

	ret = sysfs_create_group(vendor_sched_kobj, &attr_group);

	if (ret)
		kobject_put(vendor_sched_kobj);

	return ret;
}
