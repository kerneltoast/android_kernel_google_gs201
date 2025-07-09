// SPDX-License-Identifier: GPL-2.0-only
/* pixel_stat_sysfs.c
 *
 * Android Vendor Hook Support
 *
 * Copyright 2020 Google LLC
 */

#include <linux/kobject.h>
#include <linux/module.h>
#include <linux/sysfs.h>

struct kobject *pixel_stat_kobj;
EXPORT_SYMBOL_GPL(pixel_stat_kobj);

static int pixel_stat_init(void)
{
	pixel_stat_kobj = kobject_create_and_add("pixel_stat", kernel_kobj);

	if (!pixel_stat_kobj)
		return -ENOMEM;

	return 0;
}
fs_initcall(pixel_stat_init);
MODULE_LICENSE("GPL v2");
