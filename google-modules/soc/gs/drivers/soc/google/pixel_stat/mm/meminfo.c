// SPDX-License-Identifier: GPL-2.0-only
/* cma.c
 *
 * Android Vendor Hook Support
 *
 * Copyright 2021 Google LLC
 */

#include <linux/module.h>
#include <linux/seq_file.h>
#include <soc/google/meminfo.h>
#include "../../../../dma-buf/heaps/samsung/samsung-dma-heap.h"

/*****************************************************************************/
/*                       Modified Code Section                               */
/*****************************************************************************/
/*
 * This part of code is vendor hook functions, which modify or extend the
 * original functions.
 */

static LIST_HEAD(meminfo_list);
static DEFINE_MUTEX(meminfo_lock);

void rvh_meminfo_proc_show(void *data, struct seq_file *m)
{
	struct meminfo *meminfo;
	struct sysinfo i;
	int lru;
	long long misc_kb = 0;
	unsigned long pages[NR_LRU_LISTS];
	unsigned long sreclaimable, sunreclaim;
	unsigned long known_pages = 0;
	unsigned long others_kb = 0;
	char name[16];

	si_meminfo(&i);

        for (lru = LRU_BASE; lru < NR_LRU_LISTS; lru++)
                pages[lru] = global_node_page_state(NR_LRU_BASE + lru);

        sreclaimable = global_node_page_state_pages(NR_SLAB_RECLAIMABLE_B);
        sunreclaim = global_node_page_state_pages(NR_SLAB_UNRECLAIMABLE_B);

	mutex_lock(&meminfo_lock);
	list_for_each_entry(meminfo, &meminfo_list, list) {
		unsigned long size = meminfo->size_kb(meminfo->private);
		others_kb += size;
		snprintf(name, sizeof(name), "%s:", meminfo->name);
		seq_printf(m, "%-16s%8lu kB\n", name, size);
	}
	mutex_unlock(&meminfo_lock);

	known_pages = i.freeram + pages[LRU_ACTIVE_ANON] + pages[LRU_INACTIVE_ANON] +
		      pages[LRU_ACTIVE_FILE] + pages[LRU_INACTIVE_FILE] +
		      pages[LRU_UNEVICTABLE] + sreclaimable + sunreclaim +
		      global_node_page_state(NR_PAGETABLE) +
		      vmalloc_nr_pages() + pcpu_nr_pages();

	misc_kb = (i.totalram << (PAGE_SHIFT - 10)) - ((known_pages << (PAGE_SHIFT - 10)) +
			        global_node_page_state(NR_KERNEL_STACK_KB) +
			        others_kb);

	seq_printf(m, "Misc:           %8lld kB\n", misc_kb < 0 ? 0 : misc_kb);
}

void register_meminfo(struct meminfo *info)
{
	mutex_lock(&meminfo_lock);
	list_add(&info->list, &meminfo_list);
	mutex_unlock(&meminfo_lock);
}
EXPORT_SYMBOL_GPL(register_meminfo);

void unregister_meminfo(struct meminfo *info)
{
	mutex_lock(&meminfo_lock);
	list_del(&info->list);
	mutex_unlock(&meminfo_lock);
}
EXPORT_SYMBOL_GPL(unregister_meminfo);
