// SPDX-License-Identifier: GPL-2.0-only
/* vendor_mm_init.c
 *
 * Android Vendor Hook Support
 *
 * Copyright 2022 Google LLC
 */

#include <linux/kobject.h>
#include <linux/module.h>
#include <trace/hooks/gup.h>
#include <trace/hooks/mm.h>
#include <trace/hooks/buffer.h>
#include "../../include/gup.h"
#include "../../include/mm.h"
#include "../../include/buffer.h"

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
	if (ret) {
		kobject_put(vendor_mm_kobj);
		return ret;
	}

	/*
	 * Not sure this error handling is meaningful for vendor hook.
	 * Maybe better to rely on the just BUG_ON?
	 */
	ret = register_trace_android_vh_try_grab_compound_head(
				vh_try_grab_compound_head, NULL);
	if (ret)
		return ret;
	ret = register_trace_android_vh___get_user_pages_remote(
				vh___get_user_pages_remote, NULL);
	if (ret)
		return ret;
	ret = register_trace_android_vh_get_user_pages(
				vh_android_vh_get_user_pages, NULL);
	if (ret)
		return ret;
	ret = register_trace_android_vh_internal_get_user_pages_fast(
				vh_internal_get_user_pages_fast, NULL);
	if (ret)
		return ret;
	ret = register_trace_android_vh_pin_user_pages(
				vh_pin_user_pages, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_vh_pagevec_drain(
			vh_pagevec_drain, NULL);
	if (ret)
		return ret;

	/*
	 * Do not reorder pte_range_tlb_end and pte_range_tlb_start
	 * Otherwise, depending on module load timing, the pair can
	 * be broken.
	 */
	ret = register_trace_android_vh_zap_pte_range_tlb_end(
			vh_zap_pte_range_tlb_end, NULL);
	if (ret)
		return ret;
	ret = register_trace_android_vh_zap_pte_range_tlb_force_flush(
			vh_zap_pte_range_tlb_force_flush, NULL);
	if (ret)
		return ret;
	ret = register_trace_android_vh_zap_pte_range_tlb_start(
			vh_zap_pte_range_tlb_start, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_vh_skip_lru_disable(
			vh_skip_lru_disable, NULL);
	if (ret)
		return ret;
	ret = register_trace_android_vh_bh_lru_install(
			vh_bh_lru_install, NULL);
	if (ret)
		return ret;
	ret = register_trace_android_vh_do_madvise_blk_plug(
			vh_do_madvise_blk_plug, NULL);
	if (ret)
		return ret;
	ret = register_trace_android_vh_shrink_inactive_list_blk_plug(
			vh_shrink_inactive_list_blk_plug, NULL);
	if (ret)
		return ret;
	ret = register_trace_android_vh_shrink_lruvec_blk_plug(
			vh_shrink_lruvec_blk_plug, NULL);
	if (ret)
		return ret;
	ret = register_trace_android_vh_reclaim_pages_plug(
			vh_reclaim_pages_plug, NULL);

	return ret;
}
module_init(vh_mm_init);
MODULE_LICENSE("GPL v2");
