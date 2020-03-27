// SPDX-License-Identifier: GPL-2.0
/*
 * ION Memory Allocator carveout heap exporter
 *
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 */

#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/genalloc.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/mod_devicetable.h>
#include <linux/of_device.h>
#include <linux/of_reserved_mem.h>
#include <linux/highmem.h>
#include <linux/ion.h>

#include "ion_exynos.h"

struct ion_carveout_heap {
	struct ion_heap heap;
	struct gen_pool *pool;
	unsigned int alignment;
};

#define to_carveout_heap(x) container_of(x, struct ion_carveout_heap, heap)

static phys_addr_t ion_carveout_allocate(struct ion_heap *heap,
					 unsigned long size)
{
	struct ion_carveout_heap *carveout_heap = to_carveout_heap(heap);

	return gen_pool_alloc(carveout_heap->pool,
			      ALIGN(size, carveout_heap->alignment));
}

static void ion_carveout_free(struct ion_heap *heap, phys_addr_t addr,
			      unsigned long size)
{
	struct ion_carveout_heap *carveout_heap = to_carveout_heap(heap);

	gen_pool_free(carveout_heap->pool, addr,
		      ALIGN(size, carveout_heap->alignment));
}

static int ion_carveout_heap_allocate(struct ion_heap *heap,
				      struct ion_buffer *buffer,
				      unsigned long size,
				      unsigned long flags)
{
	struct sg_table *table;
	phys_addr_t paddr;
	int ret;

	table = kmalloc(sizeof(*table), GFP_KERNEL);
	if (!table)
		return -ENOMEM;
	ret = sg_alloc_table(table, 1, GFP_KERNEL);
	if (ret) {
		perr("failed to alloc sgtable with 1 entry (err %d)", -ret);
		goto err_free;
	}

	paddr = ion_carveout_allocate(heap, size);
	if (!paddr) {
		perr("failed to allocate from %s(id %u/type %d), size %lu",
		       heap->name, heap->id, heap->type, size);
		ret = -ENOMEM;
		goto err_free_table;
	}

	sg_set_page(table->sgl, phys_to_page(paddr), size, 0);
	buffer->sg_table = table;

	ion_buffer_prep_noncached(buffer);

	return 0;

err_free_table:
	sg_free_table(table);
err_free:
	kfree(table);
	return ret;
}

static void ion_carveout_heap_free(struct ion_buffer *buffer)
{
	struct ion_heap *heap = buffer->heap;
	struct sg_table *table = buffer->sg_table;
	struct page *page = sg_page(table->sgl);
	phys_addr_t paddr = PFN_PHYS(page_to_pfn(page));

	/*
	 * There is no need to map and memset with write combine
	 * for non-cachable buffer because that is flushed when allocation
	 */
	ion_page_clean(page, buffer->size);

	ion_carveout_free(heap, paddr, buffer->size);
	sg_free_table(table);
	kfree(table);
}

static struct ion_heap_ops carveout_heap_ops = {
	.allocate = ion_carveout_heap_allocate,
	.free = ion_carveout_heap_free,
};

static int ion_carveout_heap_probe(struct platform_device *pdev)
{
	struct ion_carveout_heap *carveout_heap;
	struct reserved_mem *rmem;
	struct device_node *rmem_np;
	struct exynos_fdt_attrs attrs;
	int ret;

	rmem_np = of_parse_phandle(pdev->dev.of_node, "memory-region", 0);
	rmem = of_reserved_mem_lookup(rmem_np);
	if (!rmem) {
		perrfn("%s: memory-region handle not found", pdev->name);
		return -ENODEV;
	}

	carveout_heap = devm_kzalloc(&pdev->dev, sizeof(*carveout_heap),
				     GFP_KERNEL);
	if (!carveout_heap)
		return -ENOMEM;

	carveout_heap->pool = gen_pool_create(PAGE_SHIFT, -1);
	if (!carveout_heap->pool)
		return -ENOMEM;

	gen_pool_add(carveout_heap->pool, rmem->base, rmem->size, -1);

	carveout_heap->heap.ops = &carveout_heap_ops;
	carveout_heap->heap.name = rmem->name;
	carveout_heap->heap.type =
		(enum ion_heap_type)ION_EXYNOS_HEAP_TYPE_CARVEOUT;

	ion_page_clean(phys_to_page(rmem->base), rmem->size);

	exynos_fdt_setup(&pdev->dev, &attrs);
	carveout_heap->alignment = attrs.alignment;

	ret = ion_device_add_heap(&carveout_heap->heap);
	if (ret) {
		gen_pool_destroy(carveout_heap->pool);
		return ret;
	}

	platform_set_drvdata(pdev, carveout_heap);

	pr_info("Register ion carveout heap %s successfully\n", pdev->name);

	return 0;
}

static int ion_carveout_heap_remove(struct platform_device *pdev)
{
	struct ion_carveout_heap *carveout_heap = platform_get_drvdata(pdev);

	ion_device_remove_heap(&carveout_heap->heap);
	gen_pool_destroy(carveout_heap->pool);

	return 0;
}

static const struct of_device_id carveout_heap_of_match[] = {
	{ .compatible = "exynos,ion,carveout_heap", },
	{ },
};

MODULE_DEVICE_TABLE(of, carveout_heap_of_match);

static struct platform_driver ion_carveout_heap_driver = {
	.driver		= {
		.name	= "ion_carveout_heap",
		.of_match_table = carveout_heap_of_match,
	},
	.probe		= ion_carveout_heap_probe,
	.remove		= ion_carveout_heap_remove,
};

int __init ion_carveout_heap_init(void)
{
	return platform_driver_register(&ion_carveout_heap_driver);
}

void ion_carveout_heap_exit(void)
{
	platform_driver_unregister(&ion_carveout_heap_driver);
}
