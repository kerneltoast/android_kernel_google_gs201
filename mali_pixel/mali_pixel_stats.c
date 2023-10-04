// SPDX-License-Identifier: GPL-2.0

#include "mali_pixel_mod.h"
#include <linux/module.h>

MODULE_SOFTDEP("pre: pixel_stat_sysfs");

extern struct kobject *pixel_stat_kobj;

struct kobject *pixel_stat_gpu_kobj;

int mali_pixel_init_pixel_stats(void)
{
	struct kobject *pixel_stat = pixel_stat_kobj;

	if (pixel_stat_kobj == NULL)
		return -EPROBE_DEFER;

	pixel_stat_gpu_kobj = kobject_create_and_add("gpu", pixel_stat);
	if (!pixel_stat_gpu_kobj)
		return -ENOMEM;

	return 0;
}
