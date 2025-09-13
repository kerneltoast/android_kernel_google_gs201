// SPDX-License-Identifier: GPL-2.0
/*
 * DMABUF Carveout heap exporter for Samsung
 *
 * Copyright (c) 2021 Samsung Electronics Co., Ltd.
 * Author: <hyesoo.yu@samsung.com> for Samsung
 */

#include <linux/device.h>
#include <linux/dma-buf.h>
#include <linux/dma-heap.h>
#include <linux/err.h>
#include <linux/genalloc.h>
#include <linux/highmem.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>

#include "samsung-dma-heap.h"

struct carveout_heap {
	struct gen_pool *pool;
	struct reserved_mem *rmem;
};

static struct dma_buf *carveout_heap_allocate(struct dma_heap *heap, unsigned long len,
					      unsigned long fd_flags, unsigned long heap_flags)
{
	struct samsung_dma_heap *samsung_dma_heap = dma_heap_get_drvdata(heap);
	struct carveout_heap *carveout_heap = samsung_dma_heap->priv;
	struct samsung_dma_buffer *buffer;
	struct dma_buf *dmabuf;
	struct page *pages;
	unsigned int alignment = samsung_dma_heap->alignment;
	unsigned long size;
	phys_addr_t paddr;
	int protret = 0, ret = -ENOMEM;

	if (dma_heap_flags_video_aligned(samsung_dma_heap->flags))
		len = dma_heap_add_video_padding(len);

	size = ALIGN(len, alignment);

	buffer = samsung_dma_buffer_alloc(samsung_dma_heap, size, 1);
	if (IS_ERR(buffer))
		return ERR_PTR(-ENOMEM);

	paddr = gen_pool_alloc(carveout_heap->pool, size);
	if (!paddr) {
		perrfn("failed to allocate from %s, size %lu", carveout_heap->rmem->name, size);
		goto free_gen;
	}

	pages = phys_to_page(paddr);
	sg_set_page(buffer->sg_table.sgl, pages, size, 0);

	heap_page_clean(pages, size);
	heap_cache_flush(buffer);

	if (dma_heap_flags_protected(samsung_dma_heap->flags)) {
		buffer->priv = samsung_dma_buffer_protect(buffer, size, 1, paddr);
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
	if (!protret)
		gen_pool_free(carveout_heap->pool, paddr, size);
free_gen:
	samsung_dma_buffer_free(buffer);

	return ERR_PTR(ret);
}

static void carveout_heap_release(struct samsung_dma_buffer *buffer)
{
	struct samsung_dma_heap *samsung_dma_heap = buffer->heap;
	struct carveout_heap *carveout_heap = samsung_dma_heap->priv;
	int ret = 0;

	if (dma_heap_flags_protected(samsung_dma_heap->flags))
		ret = samsung_dma_buffer_unprotect(buffer);

	if (!ret)
		gen_pool_free(carveout_heap->pool, sg_phys(buffer->sg_table.sgl), buffer->len);
	samsung_dma_buffer_free(buffer);
}

static const struct dma_heap_ops carveout_heap_ops = {
	.allocate = carveout_heap_allocate,
};

static int carveout_heap_probe(struct platform_device *pdev)
{
	struct carveout_heap *carveout_heap;
	struct reserved_mem *rmem;
	struct device_node *rmem_np;
	int ret;

	rmem_np = of_parse_phandle(pdev->dev.of_node, "memory-region", 0);
	rmem = of_reserved_mem_lookup(rmem_np);
	if (!rmem) {
		perrdev(&pdev->dev, "memory-region handle not found");
		return -ENODEV;
	}

	carveout_heap = devm_kzalloc(&pdev->dev, sizeof(*carveout_heap), GFP_KERNEL);
	if (!carveout_heap)
		return -ENOMEM;

	carveout_heap->rmem = rmem;
	carveout_heap->pool = devm_gen_pool_create(&pdev->dev, PAGE_SHIFT, -1, 0);
	if (!carveout_heap->pool)
		return -ENOMEM;

	ret = gen_pool_add(carveout_heap->pool, rmem->base, rmem->size, -1);
	if (ret)
		return ret;

	ret = samsung_heap_add(&pdev->dev, carveout_heap, carveout_heap_release,
			       &carveout_heap_ops);
	if (ret == -ENODEV)
		return 0;

	return ret;
}

static const struct of_device_id carveout_heap_of_match[] = {
	{ .compatible = "samsung,dma-heap-carveout", },
	{ },
};
MODULE_DEVICE_TABLE(of, carveout_heap_of_match);

static struct platform_driver carveout_heap_driver = {
	.driver		= {
		.name	= "samsung,dma-heap-carveout",
		.of_match_table = carveout_heap_of_match,
	},
	.probe		= carveout_heap_probe,
};

int __init carveout_dma_heap_init(void)
{
	return platform_driver_register(&carveout_heap_driver);
}

void carveout_dma_heap_exit(void)
{
	platform_driver_unregister(&carveout_heap_driver);
}
