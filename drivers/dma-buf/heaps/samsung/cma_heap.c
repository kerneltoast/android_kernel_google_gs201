// SPDX-License-Identifier: GPL-2.0
/*
 * DMABUF CMA heap exporter for Samsung
 *
 * Copyright (C) 2021 Samsung Electronics Co., Ltd.
 * Author: <hyesoo.yu@samsung.com> for Samsung
 */

#include <linux/cma.h>
#include <linux/device.h>
#include <linux/dma-buf.h>
#include <linux/dma-heap.h>
#include <linux/err.h>
#include <linux/highmem.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>

#include "samsung-dma-heap.h"

struct cma_heap {
	struct cma *cma;
};

static struct dma_buf *cma_heap_allocate(struct dma_heap *heap, unsigned long len,
					 unsigned long fd_flags, unsigned long heap_flags)
{
	struct samsung_dma_heap *samsung_dma_heap = dma_heap_get_drvdata(heap);
	struct cma_heap *cma_heap = samsung_dma_heap->priv;
	struct samsung_dma_buffer *buffer;
	struct dma_buf *dmabuf;
	struct page *pages;
	unsigned int alignment = samsung_dma_heap->alignment;
	unsigned long size, nr_pages;
	int protret = 0, ret = -ENOMEM;

	if (dma_heap_flags_video_aligned(samsung_dma_heap->flags))
		len = dma_heap_add_video_padding(len);

	size = ALIGN(len, alignment);
	nr_pages = size >> PAGE_SHIFT;

	buffer = samsung_dma_buffer_alloc(samsung_dma_heap, size, 1);
	if (IS_ERR(buffer))
		return ERR_PTR(-ENOMEM);

	pages = cma_alloc(cma_heap->cma, nr_pages, get_order(alignment), false);
	if (!pages) {
		perrfn("failed to allocate from %s, size %lu", dma_heap_get_name(heap), size);
		goto free_cma;
	}

	dma_heap_inc_inuse(nr_pages);
	sg_set_page(buffer->sg_table.sgl, pages, size, 0);
	heap_page_clean(pages, size);
	heap_cache_flush(buffer);

	if (dma_heap_flags_protected(samsung_dma_heap->flags)) {
		buffer->priv = samsung_dma_buffer_protect(buffer, size, 1,
							  page_to_phys(pages));
		if (IS_ERR(buffer->priv)) {
			ret = PTR_ERR(buffer->priv);
			buffer->priv = NULL;
			goto free_prot;
		}
	}

	dmabuf = samsung_export_dmabuf(buffer, fd_flags);
	if (IS_ERR(dmabuf)) {
		ret = PTR_ERR(dmabuf);
		goto free_export;
	}

	return dmabuf;

free_export:
	protret = samsung_dma_buffer_unprotect(buffer);
free_prot:
	if (!protret) {
		cma_release(cma_heap->cma, pages, nr_pages);
		dma_heap_dec_inuse(nr_pages);
	}
free_cma:
	samsung_dma_buffer_free(buffer);

	return ERR_PTR(ret);
}

static void cma_heap_release(struct samsung_dma_buffer *buffer)
{
	struct samsung_dma_heap *dma_heap = buffer->heap;
	struct cma_heap *cma_heap = dma_heap->priv;
	unsigned long nr_pages = buffer->len >> PAGE_SHIFT;
	int ret = 0;

	if (dma_heap_flags_protected(dma_heap->flags))
		ret = samsung_dma_buffer_unprotect(buffer);

	if (!ret) {
		cma_release(cma_heap->cma, sg_page(buffer->sg_table.sgl), nr_pages);
		dma_heap_dec_inuse(nr_pages);
	}
	samsung_dma_buffer_free(buffer);
}

static void rmem_remove_callback(void *p)
{
	of_reserved_mem_device_release((struct device *)p);
}

static const struct dma_heap_ops cma_heap_ops = {
	.allocate = cma_heap_allocate,
};

static int cma_heap_probe(struct platform_device *pdev)
{
	struct cma_heap *cma_heap;
	int ret;

	ret = of_reserved_mem_device_init(&pdev->dev);
	if (ret || !pdev->dev.cma_area) {
		dev_err(&pdev->dev, "The CMA reserved area is not assigned (ret %d)\n", ret);
		return -EINVAL;
	}

	ret = devm_add_action(&pdev->dev, rmem_remove_callback, &pdev->dev);
	if (ret) {
		of_reserved_mem_device_release(&pdev->dev);
		return ret;
	}

	cma_heap = devm_kzalloc(&pdev->dev, sizeof(*cma_heap), GFP_KERNEL);
	if (!cma_heap)
		return -ENOMEM;
	cma_heap->cma = pdev->dev.cma_area;

	ret = samsung_heap_add(&pdev->dev, cma_heap, cma_heap_release, &cma_heap_ops);

	if (ret == -ENODEV)
		return 0;

	return ret;
}

static const struct of_device_id cma_heap_of_match[] = {
	{ .compatible = "samsung,dma-heap-cma", },
	{ },
};
MODULE_DEVICE_TABLE(of, cma_heap_of_match);

static struct platform_driver cma_heap_driver = {
	.driver		= {
		.name	= "samsung,dma-heap-cma",
		.of_match_table = cma_heap_of_match,
	},
	.probe		= cma_heap_probe,
};

int __init cma_dma_heap_init(void)
{
	return platform_driver_register(&cma_heap_driver);
}

void cma_dma_heap_exit(void)
{
	platform_driver_unregister(&cma_heap_driver);
}
