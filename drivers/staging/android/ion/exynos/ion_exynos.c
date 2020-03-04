// SPDX-License-Identifier: GPL-2.0
/*
 * ION Memory Allocator exynos feature support
 *
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 */

#include <linux/mm.h>
#include <linux/device.h>
#include <linux/of_device.h>
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

void exynos_fdt_setup(struct device *dev, struct exynos_fdt_attrs *attrs)
{
	of_property_read_u32(dev->of_node, "ion,alignment", &attrs->alignment);
	attrs->alignment = 1UL << (get_order(attrs->alignment) + PAGE_SHIFT);
}

static int __init ion_exynos_init(void)
{
	int ret;

	ret = ion_exynos_cma_heap_init();
	if (ret)
		return ret;

	ret = ion_carveout_heap_init();
	if (ret) {
		ion_exynos_cma_heap_exit();
		return ret;
	}

	return 0;
}

static void __exit ion_exynos_exit(void)
{
	ion_exynos_cma_heap_exit();
	ion_carveout_heap_exit();
}

module_init(ion_exynos_init);
module_exit(ion_exynos_exit);
MODULE_LICENSE("GPL v2");
