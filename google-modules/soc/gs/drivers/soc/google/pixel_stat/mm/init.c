// SPDX-License-Identifier: GPL-2.0-only
/* init.c
 *
 * Android Vendor Hook Support
 *
 * Copyright 2020 Google LLC
 */

#include <linux/module.h>
#include <soc/google/meminfo.h>
#include <trace/hooks/mm.h>
#include <trace/hooks/vmscan.h>
#include "cma.h"
#include "vmscan.h"
#include "compaction.h"

extern void vh_rmqueue_mod(void *data, struct zone *preferred_zone,
		struct zone *zone, unsigned int order, gfp_t gfp_flags,
		unsigned int alloc_flags, int migratetype);
extern int pixel_mm_sysfs(void);
extern void vh_filemap_get_folio_mod(void *data,
		struct address_space *mapping, pgoff_t index,
		int fgp_flags, gfp_t gfp_mask, struct folio *folio);
extern void rvh_mapping_shrinkable(void *data, bool *shrinkable);

extern int create_mm_procfs_node(void);

static int pixel_stat_mm_init(void)
{
	int ret;

	ret = create_mm_procfs_node();
	if (ret)
		return ret;

	ret = pixel_mm_sysfs();
	if (ret)
		return ret;

	ret = register_trace_android_vh_rmqueue(vh_rmqueue_mod, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_vh_filemap_get_folio(
				vh_filemap_get_folio_mod, NULL);
	if (ret)
		return ret;

	ret = register_trace_cma_alloc_start(vh_cma_alloc_start, NULL);
	if (ret)
		return ret;

	ret = register_trace_cma_alloc_finish(vh_cma_alloc_finish, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_rvh_meminfo_proc_show(rvh_meminfo_proc_show, NULL);
	if (ret)
		return ret;

	ret = register_trace_mm_vmscan_direct_reclaim_begin(vh_direct_reclaim_begin, NULL);
	if (ret)
		return ret;

	ret = register_trace_mm_vmscan_direct_reclaim_end(vh_direct_reclaim_end, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_vh_mm_compaction_begin(vh_compaction_begin,
			NULL);
	if (ret)
		return ret;

	ret = register_trace_android_vh_mm_compaction_end(vh_compaction_end, NULL);
	if (ret)
		return ret;

	/* rvh_madvise_pageout_end depends on rvh_madvise_pageout_begin so do not reorder */
	ret = register_trace_android_rvh_madvise_pageout_end(rvh_madvise_pageout_end, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_rvh_madvise_pageout_begin(rvh_madvise_pageout_begin, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_rvh_reclaim_folio_list(rvh_reclaim_folio_list, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_rvh_mapping_shrinkable(rvh_mapping_shrinkable, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_rvh_vmscan_kswapd_wake(rvh_vmscan_kswapd_wake, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_rvh_vmscan_kswapd_done(rvh_vmscan_kswapd_done, NULL);
	if (ret)
		return ret;

	return 0;
}

module_init(pixel_stat_mm_init);
MODULE_SOFTDEP("pre: pixel_stat_sysfs");
MODULE_LICENSE("GPL v2");
