// SPDX-License-Identifier: GPL-2.0
/*
 * ION Memory Allocator cma heap exporter
 *
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 */

#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/ion.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/cma.h>
#include <linux/scatterlist.h>
#include <linux/mod_devicetable.h>
#include <linux/of_reserved_mem.h>
#include <linux/kernel.h>

#include "ion_exynos.h"

struct ion_exynos_cma_heap {
	struct ion_heap heap;
	struct platform_device *pdev;
	struct cma *cma;
	struct exynos_fdt_attrs attrs;
};

#define to_cma_heap(x) container_of(x, struct ion_exynos_cma_heap, heap)

/* ION CMA heap operations functions */
static int ion_exynos_cma_allocate(struct ion_heap *heap,
				   struct ion_buffer *buffer,
				   unsigned long len,
				   unsigned long flags)
{
	struct ion_exynos_cma_heap *cma_heap = to_cma_heap(heap);
	unsigned long alloc_size = ALIGN(len, cma_heap->attrs.alignment);
	struct sg_table *table;
	struct page *pages;
	int ret;

	pages = cma_alloc(cma_heap->cma, alloc_size / PAGE_SIZE,
			  get_order(cma_heap->attrs.alignment), false);
	if (!pages) {
		perr("failed to allocate from %s(id %u/type %d), size %lu",
		     cma_heap->heap.name, cma_heap->heap.id, cma_heap->heap.type, len);
		return -ENOMEM;
	}

	table = kmalloc(sizeof(*table), GFP_KERNEL);
	if (!table)
		goto err;

	ret = sg_alloc_table(table, 1, GFP_KERNEL);
	if (ret) {
		perr("failed to alloc sgtable with 1 entry (err %d)", -ret);
		goto free_mem;
	}

	sg_set_page(table->sgl, pages, alloc_size, 0);

	buffer->sg_table = table;
	buffer->size = alloc_size;

	ion_page_clean(pages, alloc_size);

	/*
	 * Pages from the CMA heap tends to be cached in the CPU caches with
	 * dirty state since they are served as anon and page cache pages for
	 * the userspace.
	 * Flushing caches on buffer allocation is intended for preventing
	 * corruption from writing back to DRAM from the dirty cache lines
	 * while updating the buffer from DMA. However, cache flush should be
	 * performed on the entire allocated area if the buffer is to be
	 * protected from non-secure access to prevent the dirty write-back
	 * to the protected area.
	 */
	ion_buffer_prep_noncached(buffer);

	if (cma_heap->attrs.secure && ion_buffer_protected(buffer)) {
		buffer->priv_virt =
			ion_buffer_protect(&cma_heap->pdev->dev,
					   cma_heap->attrs.protection_id,
					   (unsigned int)alloc_size,
					   page_to_phys(pages),
					   cma_heap->attrs.alignment);
		if (IS_ERR(buffer->priv_virt)) {
			ret = PTR_ERR(buffer->priv_virt);
			goto err_prot;
		}
	}

	return 0;
err_prot:
	sg_free_table(buffer->sg_table);
free_mem:
	kfree(table);
err:
	cma_release(cma_heap->cma, pages, alloc_size / PAGE_SIZE);
	return -ENOMEM;
}

static void ion_exynos_cma_free(struct ion_buffer *buffer)
{
	struct ion_exynos_cma_heap *cma_heap = to_cma_heap(buffer->heap);
	struct page *pages = sg_page(buffer->sg_table->sgl);
	unsigned long alloc_size = buffer->sg_table->sgl->length;
	int unprot_err = 0;

	if (cma_heap->attrs.secure && ion_buffer_protected(buffer))
		unprot_err = ion_buffer_unprotect(buffer->priv_virt);

	sg_free_table(buffer->sg_table);
	kfree(buffer->sg_table);

	/*
	 * If releasing H/W protection to a buffer fails, the buffer might not
	 * be unusable in Linux forever because we do not have an idea to
	 * determine if the buffer is accessible in Linux. So, we should mark
	 * that buffer unusable with holding allocated buffer.
	 */
	if (!unprot_err)
		cma_release(cma_heap->cma, pages, alloc_size / PAGE_SIZE);
}

static struct ion_heap_ops ion_exynos_cma_ops = {
	.allocate = ion_exynos_cma_allocate,
	.free = ion_exynos_cma_free,
};

static void rmem_remove_callback(void *p)
{
	of_reserved_mem_device_release((struct device *)p);
}

static int ion_exynos_cma_heap_probe(struct platform_device *pdev)
{
	struct ion_exynos_cma_heap *cma_heap;
	int ret;

	cma_heap = devm_kzalloc(&pdev->dev, sizeof(*cma_heap), GFP_KERNEL);
	if (!cma_heap)
		return -ENOMEM;

	ret = of_reserved_mem_device_init(&pdev->dev);
	if (ret) {
		perrfn("%s: Failed to get reserved memory region", pdev->name);
		return ret;
	}

	ret = devm_add_action(&pdev->dev, rmem_remove_callback, &pdev->dev);
	if (ret) {
		of_reserved_mem_device_release(&pdev->dev);
		return ret;
	}

	cma_heap->cma = pdev->dev.cma_area;
	if (!cma_heap->cma) {
		perrfn("%s: No CMA region found", pdev->name);
		return -ENODEV;
	}

	cma_heap->heap.ops = &ion_exynos_cma_ops;
	cma_heap->heap.type = ION_HEAP_TYPE_DMA;
	cma_heap->heap.name = cma_get_name(cma_heap->cma);
	cma_heap->heap.buf_ops = exynos_dma_buf_ops;

	cma_heap->pdev = pdev;

	pdev->dev.dma_coherent = false;

	dma_coerce_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(36));

	exynos_fdt_setup(&pdev->dev, &cma_heap->attrs);

	ret = ion_device_add_heap(&cma_heap->heap);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, cma_heap);

	pr_info("Registered CMA heap %s successfully\n", pdev->name);

	return 0;
}

static int ion_exynos_cma_heap_remove(struct platform_device *pdev)
{
	struct ion_exynos_cma_heap *cma_heap = platform_get_drvdata(pdev);

	ion_device_remove_heap(&cma_heap->heap);

	return 0;
}

static const struct of_device_id ion_exynos_cma_heap_of_match[] = {
	{ .compatible = "exynos,ion,cma_heap", },
	{ },
};

MODULE_DEVICE_TABLE(of, ion_exynos_cma_heap_of_match);

static struct platform_driver ion_exynos_cma_heap_driver = {
	.driver		= {
		.name	= "ion_exynos_cma_heap",
		.of_match_table = ion_exynos_cma_heap_of_match,
	},
	.probe		= ion_exynos_cma_heap_probe,
	.remove		= ion_exynos_cma_heap_remove,
};

int __init ion_exynos_cma_heap_init(void)
{
	return platform_driver_register(&ion_exynos_cma_heap_driver);
}

void ion_exynos_cma_heap_exit(void)
{
	platform_driver_unregister(&ion_exynos_cma_heap_driver);
}
