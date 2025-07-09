// SPDX-License-Identifier: GPL-2.0
/*
 * DMABUF Chunk heap exporter for Samsung
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
#include <linux/sort.h>

#include "samsung-dma-heap.h"

#define GFP_CHUNK_HEAP_NORETRY_NOWARN (__GFP_NORETRY | __GFP_NOWARN)

struct chunk_heap {
	struct cma *cma;
};

static int chunk_pages_compare(const void *p1, const void *p2)
{
	if (*((const unsigned long *)p1) > (*((const unsigned long *)p2)))
		return 1;
	else if (*((const unsigned long *)p1) < (*((const unsigned long *)p2)))
		return -1;
	return 0;
}

static int chunk_heap_buffer_allocate(struct cma *cma, unsigned int need_count,
				      struct page **pages, unsigned int chunk_order)
{
	unsigned int i, alloc_count = 0;
	unsigned int alloc_order = max_t(unsigned int, pageblock_order, chunk_order);
	unsigned int nr_chunks_per_alloc = 1 << (alloc_order - chunk_order);
	gfp_t gfp_flags = GFP_KERNEL | GFP_CHUNK_HEAP_NORETRY_NOWARN;

	while (alloc_count < need_count) {
		struct page *page;

		while (need_count - alloc_count < nr_chunks_per_alloc) {
			alloc_order--;
			nr_chunks_per_alloc >>= 1;
		}

		page = __cma_alloc(cma, 1 << alloc_order, alloc_order, gfp_flags);
		if (!page) {
			/* Try without GFP_NORETRY first */
			if (gfp_flags & __GFP_NORETRY) {
				gfp_flags &= ~GFP_CHUNK_HEAP_NORETRY_NOWARN;
			/* Try half alloc_order to allocate from splited block second */
			} else {
				gfp_flags |= GFP_CHUNK_HEAP_NORETRY_NOWARN;
				alloc_order--;
				nr_chunks_per_alloc >>= 1;

				if (alloc_order < chunk_order)
					break;
			}
			continue;
		}

		dma_heap_inc_inuse(1 << alloc_order);
		for (i = 0; i < nr_chunks_per_alloc; i++, alloc_count++) {
			pages[alloc_count] = page;
			page += 1 << chunk_order;
		}
	}

	if (alloc_count < need_count) {
		for (i = 0; i < alloc_count; i++) {
			cma_release(cma, pages[i], 1 << chunk_order);
			dma_heap_dec_inuse(1 << chunk_order);
		}
		return -ENOMEM;
	}

	sort(pages, alloc_count, sizeof(*pages), chunk_pages_compare, NULL);

	return 0;
}

static void *chunk_heap_protect(struct samsung_dma_buffer *buffer,
				unsigned int chunk_size, struct page **pages,
				unsigned long nr_pages)
{
	unsigned long *paddr_array = NULL;
	unsigned long paddr, i;
	void *priv;

	/*
	 * LDFW in secure world interprets paddr differently depending on whether nr_pages == 1.
	 * If it is 1, then it interprets it as the physical address of physically contiguous
	 * memory. If it is > 1, then it interprets it as a physical address of an array of
	 * physical addresses of chunks page like 64KB.
	 */
	if (nr_pages == 1) {
		paddr = page_to_phys(pages[0]);
	} else {
		paddr_array = kmalloc_array(nr_pages, sizeof(*paddr_array), GFP_KERNEL);
		if (!paddr_array)
			return ERR_PTR(-ENOMEM);

		for (i = 0; i < nr_pages; i++)
			paddr_array[i] = page_to_phys(pages[i]);

		paddr = virt_to_phys(paddr_array);
	}

	priv = samsung_dma_buffer_protect(buffer, chunk_size, nr_pages, paddr);
	if (IS_ERR(priv))
		kfree(paddr_array);

	return priv;
}

static int chunk_heap_unprotect(struct samsung_dma_buffer *buffer)
{
	struct buffer_prot_info *protdesc = buffer->priv;
	void *paddr_array = NULL;
	int ret;

	if (protdesc->chunk_count > 1)
		paddr_array = phys_to_virt(protdesc->bus_address);

	ret = samsung_dma_buffer_unprotect(buffer);

	kfree(paddr_array);

	return ret;
}

static struct dma_buf *chunk_heap_allocate(struct dma_heap *heap, unsigned long len,
					   unsigned long fd_flags,
					   unsigned long heap_flags __maybe_unused)
{
	struct samsung_dma_heap *samsung_dma_heap = dma_heap_get_drvdata(heap);
	struct chunk_heap *chunk_heap = samsung_dma_heap->priv;
	struct samsung_dma_buffer *buffer;
	struct scatterlist *sg;
	struct dma_buf *dmabuf;
	struct page **pages;
	unsigned long size, nr_chunks;
	unsigned int chunk_order = get_order(samsung_dma_heap->alignment);
	unsigned int chunk_size = PAGE_SIZE << chunk_order;
	int ret = -ENOMEM, protret = 0;
	pgoff_t pg;

	size = ALIGN(len, chunk_size);
	nr_chunks = size / chunk_size;

	pages = kvmalloc_array(nr_chunks, sizeof(*pages), GFP_KERNEL);
	if (!pages)
		return ERR_PTR(-ENOMEM);

	ret = chunk_heap_buffer_allocate(chunk_heap->cma, nr_chunks, pages, chunk_order);
	if (ret)
		goto err_alloc;

	buffer = samsung_dma_buffer_alloc(samsung_dma_heap, size, nr_chunks);
	if (IS_ERR(buffer)) {
		ret = PTR_ERR(buffer);
		goto err_buffer;
	}

	sg = buffer->sg_table.sgl;
	for (pg = 0; pg < nr_chunks; pg++) {
		sg_set_page(sg, pages[pg], chunk_size, 0);
		sg = sg_next(sg);
	}

	heap_sgtable_pages_clean(&buffer->sg_table);
	heap_cache_flush(buffer);

	if (dma_heap_flags_protected(samsung_dma_heap->flags)) {
		buffer->priv = chunk_heap_protect(buffer, chunk_size, pages, nr_chunks);
		if (IS_ERR(buffer->priv)) {
			ret = PTR_ERR(buffer->priv);
			buffer->priv = NULL;
			goto err_prot;
		}
	}

	dmabuf = samsung_export_dmabuf(buffer, fd_flags);
	if (IS_ERR(dmabuf)) {
		ret = PTR_ERR(dmabuf);
		goto err_export;
	}
	kvfree(pages);

	return dmabuf;

err_export:
	protret = chunk_heap_unprotect(buffer);
err_prot:
	samsung_dma_buffer_free(buffer);
err_buffer:
	if (!protret) {
		for (pg = 0; pg < nr_chunks; pg++) {
			cma_release(chunk_heap->cma, pages[pg], 1 << chunk_order);
			dma_heap_dec_inuse(1 << chunk_order);
		}
	}
err_alloc:
	kvfree(pages);
	return ERR_PTR(ret);
}

static void chunk_heap_release(struct samsung_dma_buffer *buffer)
{
	struct samsung_dma_heap *dma_heap = buffer->heap;
	struct chunk_heap *chunk_heap = dma_heap->priv;
	struct sg_table *table;
	struct scatterlist *sg;
	unsigned int i;
	int ret = 0, chunk_order = get_order(dma_heap->alignment);

	table = &buffer->sg_table;

	if (dma_heap_flags_protected(dma_heap->flags))
		ret = chunk_heap_unprotect(buffer);

	if (!ret) {
		for_each_sgtable_sg(table, sg, i) {
			cma_release(chunk_heap->cma, sg_page(sg), 1 << chunk_order);
			dma_heap_dec_inuse(1 << chunk_order);
		}
	}
	samsung_dma_buffer_free(buffer);
}

static void rmem_remove_callback(void *p)
{
	of_reserved_mem_device_release((struct device *)p);
}

static const struct dma_heap_ops chunk_heap_ops = {
	.allocate = chunk_heap_allocate,
};

static int chunk_heap_probe(struct platform_device *pdev)
{
	struct chunk_heap *chunk_heap;
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

	chunk_heap = devm_kzalloc(&pdev->dev, sizeof(*chunk_heap), GFP_KERNEL);
	if (!chunk_heap)
		return -ENOMEM;
	chunk_heap->cma = pdev->dev.cma_area;

	ret = samsung_heap_add(&pdev->dev, chunk_heap, chunk_heap_release, &chunk_heap_ops);

	if (ret == -ENODEV)
		return 0;

	return ret;
}

static const struct of_device_id chunk_heap_of_match[] = {
	{ .compatible = "samsung,dma-heap-chunk", },
	{ },
};
MODULE_DEVICE_TABLE(of, chunk_heap_of_match);

static struct platform_driver chunk_heap_driver = {
	.driver		= {
		.name	= "samsung,dma-heap-chunk",
		.of_match_table = chunk_heap_of_match,
	},
	.probe		= chunk_heap_probe,
};

int __init chunk_dma_heap_init(void)
{
	return platform_driver_register(&chunk_heap_driver);
}

void chunk_dma_heap_exit(void)
{
	platform_driver_unregister(&chunk_heap_driver);
}
