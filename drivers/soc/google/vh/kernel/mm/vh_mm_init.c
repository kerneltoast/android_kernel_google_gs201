// SPDX-License-Identifier: GPL-2.0-only
/* vendor_mm_init.c
 *
 * Android Vendor Hook Support
 *
 * Copyright 2022 Google LLC
 */

#include <linux/kobject.h>
#include <linux/module.h>

struct kobject *vendor_mm_kobj;
EXPORT_SYMBOL_GPL(vendor_mm_kobj);

extern int pixel_mm_cma_sysfs(struct kobject *parent);

static int vh_mm_init(void)
{
	int ret;

	vendor_mm_kobj = kobject_create_and_add("vendor_mm", kernel_kobj);
	if (!vendor_mm_kobj)
		return -ENOMEM;

	ret = pixel_mm_cma_sysfs(vendor_mm_kobj);
	if (ret)
		kobject_put(vendor_mm_kobj);

	return ret;
}
module_init(vh_mm_init);
MODULE_LICENSE("GPL v2");
