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
#include <linux/workqueue.h>
#include <generated/utsrelease.h>
#include <soc/google/debug-snapshot.h>
#include "../../../../drivers/staging/android/debug_kinfo.h"

#define UPDATE_VENDOR_KERNEL_INFO_PERIOD_MS		10

static struct workqueue_struct *update_vendor_kernel_info_wq;
static struct delayed_work vendor_kernel_info_work;
static struct kernel_all_info *aosp_all_info;
static struct kernel_info *aosp_info;
struct vendor_kernel_all_info *all_info;
struct vendor_kernel_info *info;

struct vendor_kernel_info {
	/* For linux banner */
	__u8 uts_release[__NEW_UTS_LEN];
	/* For module kallsyms */
	uint64_t mod_root_pa;
} __packed;

struct vendor_kernel_all_info {
	__u32 magic_number;
	__u32 combined_checksum;
	struct vendor_kernel_info info;
} __packed;

static unsigned int kallsyms_aosp_expand_symbol(unsigned int off, char *result, size_t maxlen)
{
	const u8 *kallsyms_names = phys_to_virt(aosp_info->_names_pa);
	const char *kallsyms_token_table = phys_to_virt(aosp_info->_token_table_pa);
	const u16 *kallsyms_token_index = phys_to_virt(aosp_info->_token_index_pa);
	int len, skipped_first = 0;
	const char *tptr;
	const u8 *data;

	/* Get the compressed symbol length from the first symbol byte. */
	data = &kallsyms_names[off];
	len = *data;
	data++;

	/*
	 * Update the offset to return the offset for the next symbol on
	 * the compressed stream.
	 */
	off += len + 1;

	/*
	 * For every byte on the compressed symbol data, copy the table
	 * entry for that byte.
	 */
	while (len) {
		tptr = &kallsyms_token_table[kallsyms_token_index[*data]];
		data++;
		len--;

		while (*tptr) {
			if (skipped_first) {
				if (maxlen <= 1)
					goto tail;
				*result = *tptr;
				result++;
				maxlen--;
			} else
				skipped_first = 1;
			tptr++;
		}
	}

tail:
	if (maxlen)
		*result = '\0';

	/* Return to offset to the next symbol. */
	return off;
}

static unsigned long kallsyms_aosp_sym_address(int idx)
{
	const unsigned long *kallsyms_addresses = phys_to_virt(aosp_info->_addresses_pa);
	const unsigned long kallsyms_relative_base =
		(unsigned long)phys_to_virt(aosp_info->_relative_pa);
	const int *kallsyms_offsets = phys_to_virt(aosp_info->_offsets_pa);

	if (!aosp_info->enabled_base_relative)
		return kallsyms_addresses[idx];

	/* values are unsigned offsets if --absolute-percpu is not in effect */
	if (!aosp_info->enabled_absolute_percpu)
		return kallsyms_relative_base + (u32)kallsyms_offsets[idx];

	/* ...otherwise, positive offsets are absolute values */
	if (kallsyms_offsets[idx] >= 0)
		return kallsyms_offsets[idx];

	/* ...and negative offsets are relative to kallsyms_relative_base - 1 */
	return kallsyms_relative_base - 1 - kallsyms_offsets[idx];
}

/* Lookup the address for this symbol. Returns 0 if not found. */
static volatile void *kallsyms_aosp_lookup_name(const char *name)
{
	char *namebuf;
	unsigned int i, off;
	bool matched = false;

	namebuf = kmalloc(aosp_info->name_len, GFP_KERNEL);
	if (!namebuf)
		return NULL;

	for (i = 0, off = 0; i < aosp_info->num_syms; i++) {
		off = kallsyms_aosp_expand_symbol(off, namebuf, aosp_info->name_len);

		if (strcmp(namebuf, name) == 0) {
			matched = true;
			break;
		}
	}

	kfree(namebuf);
	return matched ? (volatile void *)kallsyms_aosp_sym_address(i) : NULL;
}

static void update_vendor_kernel_all_info(void)
{
	int index;
	struct vendor_kernel_info *info;
	u32 *vendor_checksum_info;

	all_info->magic_number = DEBUG_KINFO_MAGIC;
	all_info->combined_checksum = 0;

	info = &all_info->info;
	strscpy(info->uts_release, UTS_RELEASE, sizeof(info->uts_release));
	if (aosp_info->enabled_modules_tree_lookup) {
		info->mod_root_pa =
			virt_to_phys(kallsyms_aosp_lookup_name("mod_tree"));
	} else {
		info->mod_root_pa =
			virt_to_phys(kallsyms_aosp_lookup_name("modules"));
	}

	vendor_checksum_info = (u32 *)info;
	for (index = 0; index < sizeof(*info) / sizeof(u32); index++)
		all_info->combined_checksum ^= vendor_checksum_info[index];
}

static void vendor_kernel_info_work_fn(struct work_struct *work)
{
	if (!aosp_all_info->combined_checksum || aosp_all_info->magic_number != DEBUG_KINFO_MAGIC) {
		queue_delayed_work(update_vendor_kernel_info_wq, &vendor_kernel_info_work,
				   msecs_to_jiffies(UPDATE_VENDOR_KERNEL_INFO_PERIOD_MS));
		return;
	}

	update_vendor_kernel_all_info();
	cancel_delayed_work_sync(&vendor_kernel_info_work);
}

static int debug_snapshot_debug_kinfo_probe(struct platform_device *pdev)
{
	int i;
	struct reserved_mem *rmem;
	struct device_node *mem_region;
	phys_addr_t base, size, offset;
	unsigned int num_pages;
	unsigned long flags = VM_NO_GUARD | VM_MAP;
	pgprot_t prot = __pgprot(PROT_NORMAL_NC);
	struct page **pages;
	void *vaddr;

	/* Introduce DPM to disable the feature for production */
	if (!dbg_snapshot_get_dpm_status())
		return -EPROBE_DEFER;

	if (!dbg_snapshot_get_enabled_debug_kinfo())
		return 0;

	mem_region = of_parse_phandle(pdev->dev.of_node, "memory-region", 0);
	if (!mem_region) {
		dev_err(&pdev->dev, "no such memory-region\n");
		return 0;
	}

	rmem = of_reserved_mem_lookup(mem_region);
	if (!rmem) {
		dev_err(&pdev->dev, "no such reserved mem of node %pOF\n",
				dev_of_node(&pdev->dev));
		return 0;
	}

	base = rmem->base;
	size = rmem->size;
	offset = size / 2;

	if (!base || !size || offset < sizeof(struct kernel_all_info) ||
	    (size - offset) < sizeof(struct vendor_kernel_all_info)) {
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
		dev_err(&pdev->dev, "paddr:%pap page_size:0x%pap failed to vmap\n",
				&rmem->base, &rmem->size);
		return 0;
	}

	/* Prepare for AOSP kernel info. backup */
	rmem->priv = vaddr;

	/* Backup aosp kernel info. */
	aosp_all_info = (struct kernel_all_info *)vaddr;
	aosp_info = &(aosp_all_info->info);

	/* Backup vendor kernel info. */
	all_info = (struct vendor_kernel_all_info *)(vaddr + offset);
	memset(all_info, 0, sizeof(*all_info));
	info = &all_info->info;

	/* Wait for AOSP kernel info. backup */
	update_vendor_kernel_info_wq = create_workqueue("vendor_kernel_info");
	if (!update_vendor_kernel_info_wq) {
		dev_err(&pdev->dev, "failed to create workqueue\n");
	}
	INIT_DELAYED_WORK(&vendor_kernel_info_work, vendor_kernel_info_work_fn);
	queue_delayed_work(update_vendor_kernel_info_wq, &vendor_kernel_info_work,
			   msecs_to_jiffies(UPDATE_VENDOR_KERNEL_INFO_PERIOD_MS));

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
