// SPDX-License-Identifier: GPL-2.0
/*
 * ION Memory Allocator exynos feature support
 *
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 */

#include <linux/module.h>
#include <linux/highmem.h>

#include "ion_exynos.h"

void ion_page_clean(struct page *pages, unsigned long size)
{
	unsigned long nr_pages, i;

	if (!PageHighMem(pages)) {
		memset(page_address(pages), 0, size);
		return;
	}

	nr_pages = PAGE_ALIGN(size) >> PAGE_SHIFT;

	for (i = 0; i < nr_pages; i++) {
		void *vaddr = kmap_atomic(&pages[i]);

		memset(vaddr, 0, PAGE_SIZE);
		kunmap_atomic(vaddr);
	}
}

static int __init ion_exynos_init(void)
{
	return ion_exynos_cma_heap_init();
}

static void __exit ion_exynos_exit(void)
{
	ion_exynos_cma_heap_exit();
}

module_init(ion_exynos_init);
module_exit(ion_exynos_exit);
MODULE_LICENSE("GPL v2");
