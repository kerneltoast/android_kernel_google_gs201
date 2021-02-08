// SPDX-License-Identifier: GPL-2.0
/*
 * debug-snapshot-debug-kinfo.c - use DPM to enable/disable kernel information backup
 *
 * Copyright 2020 Google LLC
 */

#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_reserved_mem.h>
#include <soc/google/debug-snapshot.h>
#include "../../../../drivers/staging/android/debug_kinfo.h"

static int debug_snapshot_debug_kinfo_probe(struct platform_device *pdev)
{
	int i;
	struct reserved_mem *rmem;
	struct device_node *mem_region;
	phys_addr_t base, size;
	unsigned int num_pages;
	unsigned long flags = VM_NO_GUARD | VM_MAP;
	pgprot_t prot = __pgprot(PROT_NORMAL_NC);
	struct page **pages;
	void *vaddr;

	if (!dbg_snapshot_get_dpm_status())
		return -EPROBE_DEFER;

	mem_region = of_parse_phandle(pdev->dev.of_node, "memory-region", 0);
	if (!mem_region) {
		dev_err(&pdev->dev, "no such memory-region\n");
		return 0;
	}

	rmem = of_reserved_mem_lookup(mem_region);
	if (!rmem) {
		dev_err(&pdev->dev, "no such reserved mem of node name %s\n",
				&pdev->dev.of_node->name);
		return 0;
	}

	base = rmem->base;
	size = rmem->size;

	if (!base || !size || size < sizeof(struct kernel_all_info)) {
		dev_err(&pdev->dev, "unexpected reserved memory\n");
		return 0;
	}

	num_pages = (size >> PAGE_SHIFT);
	if (size > (num_pages << PAGE_SHIFT))
		num_pages++;

	pages = kcalloc(num_pages, sizeof(struct page *), GFP_KERNEL);
	for (i = 0; i < num_pages; i++) {
		pages[i] = phys_to_page(base);
		base += PAGE_SIZE;
	}

	vaddr = vmap(pages, num_pages, flags, prot);
	kfree(pages);
	if (!vaddr) {
		dev_err(&pdev->dev, "paddr:%pK page_size:0x%x failed to vmap\n",
				rmem->base, rmem->size);
		return 0;
	}

	/* Introduce DPM to disable the feature for production */
	if (dbg_snapshot_get_enabled_debug_kinfo()) {
		rmem->priv = vaddr;
	} else {
		vunmap(vaddr);
	}

	return 0;
}

static const struct of_device_id debug_snapshot_debug_kinfo_of_match[] = {
	{ .compatible	= "google,debug-snapshot-debug-kinfo" },
	{},
};
MODULE_DEVICE_TABLE(of, debug_snapshot_debug_kinfo_of_match);

static struct platform_driver debug_snapshot_debug_kinfo_driver = {
	.probe = debug_snapshot_debug_kinfo_probe,
	.driver = {
		.name = "debug-snapshot-debug-kinfo",
		.of_match_table = of_match_ptr(debug_snapshot_debug_kinfo_of_match),
	},
};
module_platform_driver(debug_snapshot_debug_kinfo_driver);

MODULE_AUTHOR("Jone Chou <jonechou@google.com>");
MODULE_DESCRIPTION("Debug Snapshot Debug Kinfo");
MODULE_LICENSE("GPL v2");
