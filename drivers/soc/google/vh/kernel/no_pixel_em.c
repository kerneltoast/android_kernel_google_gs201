// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2023 Sultan Alsawaf <sultan@kerneltoast.com>.
 */

#include <linux/kobject.h>

/* Keep libperfmgr happy when pixel_em is gone */
static ssize_t sysfs_active_profile_store(struct kobject *kobj,
					  struct kobj_attribute *attr,
					  const char *buf, size_t count)
{
	return count;
}

static struct kobj_attribute active_profile_attr =
	__ATTR(active_profile, 0220, NULL, sysfs_active_profile_store);

static int __init no_pixel_em_init(void)
{
	struct kobject *kobj;

	kobj = kobject_create_and_add("pixel_em", kernel_kobj);
	if (!kobj)
		return -EINVAL;

	if (sysfs_create_file(kobj, &active_profile_attr.attr)) {
		kobject_put(kobj);
		return -EINVAL;
	}

	return 0;
}
late_initcall(no_pixel_em_init);
