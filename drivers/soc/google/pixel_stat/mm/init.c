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

static int pixel_stat_mm_init(void)
{
	int ret;

	ret = register_trace_android_vh_rmqueue(vh_rmqueue_mod, NULL);
	if (ret)
		return ret;

	return 0;
}

module_init(pixel_stat_mm_init);
MODULE_LICENSE("GPL v2");
