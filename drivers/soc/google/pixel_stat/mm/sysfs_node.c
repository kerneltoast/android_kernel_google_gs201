// SPDX-License-Identifier: GPL-2.0-only
/* sysfs_node.c
 *
 * Android Vendor Hook Support
 *
 * Copyright 2020 Google LLC
 */

#include <linux/cpu.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>

DEFINE_PER_CPU(unsigned long, pgalloc_costly_order);

static ssize_t vmstat_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int cpu;
	unsigned long pages = 0;

	get_online_cpus();
	for_each_online_cpu(cpu)
		pages += per_cpu(pgalloc_costly_order, cpu);

	put_online_cpus();

	return sprintf(buf, "%s %d\n", "pgalloc_costly_order", pages);
}

static struct kobj_attribute vmstat_attribute = __ATTR_RO(vmstat);

extern struct kobject *pixel_stat_kobj;
static struct kobject *pixel_stat_mm_kobj;
static struct attribute *attrs[] = {
	&vmstat_attribute.attr,
	NULL,
};
static struct attribute_group attr_group = {
	.attrs = attrs,
};

int pixel_mm_sysfs(void)
{
	int ret;

	if (pixel_stat_kobj)
		pixel_stat_mm_kobj = kobject_create_and_add("mm", pixel_stat_kobj);

	if (!pixel_stat_mm_kobj)
		return -ENOMEM;

	ret = sysfs_create_group(pixel_stat_mm_kobj, &attr_group);

	if (ret)
		kobject_put(pixel_stat_mm_kobj);

	return ret;
}
