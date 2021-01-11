// SPDX-License-Identifier: GPL-2.0-only
/* init.c
 *
 * Android Vendor Hook Support
 *
 * Copyright 2020 Google LLC
 */

#include <linux/module.h>
#include <trace/hooks/mm.h>

extern void vh_rmqueue_mod(void *data, struct zone *preferred_zone,
		struct zone *zone, unsigned int order, gfp_t gfp_flags,
		unsigned int alloc_flags, int migratetype);
extern int pixel_mm_sysfs(void);

static int pixel_stat_mm_init(void)
{
	int ret;

	ret = register_trace_android_vh_rmqueue(vh_rmqueue_mod, NULL);
	if (ret)
		return ret;

	ret = pixel_mm_sysfs();
	if (ret)
		return ret;

	return 0;
}

module_init(pixel_stat_mm_init);
MODULE_SOFTDEP("pre: pixel_stat_sysfs");
MODULE_LICENSE("GPL v2");
