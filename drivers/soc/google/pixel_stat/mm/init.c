// SPDX-License-Identifier: GPL-2.0-only
/* init.c
 *
 * Android Vendor Hook Support
 *
 * Copyright 2020 Google LLC
 */

#include <linux/module.h>
#include <trace/hooks/mm.h>
#include "cma.h"
#include "meminfo.h"
#include "vmscan.h"

extern void vh_rmqueue_mod(void *data, struct zone *preferred_zone,
		struct zone *zone, unsigned int order, gfp_t gfp_flags,
		unsigned int alloc_flags, int migratetype);
extern int pixel_mm_sysfs(void);
extern void vh_pagecache_get_page_mod(void *data,
		struct address_space *mapping, pgoff_t index,
		int fgp_flags, gfp_t gfp_mask, struct page *page);

static int pixel_stat_mm_init(void)
{
	int ret;

	ret = register_trace_android_vh_rmqueue(vh_rmqueue_mod, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_vh_pagecache_get_page(
				vh_pagecache_get_page_mod, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_vh_cma_alloc_start(vh_cma_alloc_start, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_vh_cma_alloc_finish(vh_cma_alloc_finish, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_vh_meminfo_proc_show(vh_meminfo_proc_show, NULL);

	ret = register_trace_mm_vmscan_direct_reclaim_begin(vh_direct_reclaim_begin, NULL);
	if (ret)
		return ret;

	ret = register_trace_mm_vmscan_direct_reclaim_end(vh_direct_reclaim_end, NULL);
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
