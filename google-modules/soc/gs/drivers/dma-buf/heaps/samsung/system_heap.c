// SPDX-License-Identifier: GPL-2.0
/*
 * DMABUF System heap exporter for Samsung
 *
 * Copyright (c) 2021 Samsung Electronics Co., Ltd.
 * Copyright (C) 2011 Google, Inc.
 * Copyright (C) 2019, 2020 Linaro Ltd.
 *
 * Portions based off of Andrew Davis' SRAM heap:
 * Copyright (C) 2019 Texas Instruments Incorporated - http://www.ti.com/
 *	Andrew F. Davis <afd@ti.com>
 */

#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/dma-heap.h>
#include <linux/err.h>
#include <linux/highmem.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/of.h>
#include <soc/google/meminfo.h>

#include <heaps/page_pool.h>

#include "samsung-dma-heap.h"

#define HIGH_ORDER_GFP  (((GFP_HIGHUSER | __GFP_ZERO | __GFP_NOWARN \
				| __GFP_NORETRY) & ~__GFP_RECLAIM) \
				| __GFP_COMP)
#define LOW_ORDER_GFP (GFP_HIGHUSER | __GFP_ZERO | __GFP_COMP)
static gfp_t order_flags[] = {HIGH_ORDER_GFP, HIGH_ORDER_GFP, HIGH_ORDER_GFP, LOW_ORDER_GFP};
/*
 * The selection of the orders used for allocation (2MB, 1MB, 64K, 4K) is designed
 * to match with the sizes often found in IOMMUs. Using high order pages instead
 * of order 0 pages can significantly improve the performance of many IOMMUs
 * by reducing TLB pressure and time spent updating page tables.
 */
static const unsigned int orders[] = {9, 8, 4, 0};
#define NUM_ORDERS ARRAY_SIZE(orders)
struct dmabuf_page_pool *pools[NUM_ORDERS];

static unsigned long dma_heap_system_inuse_pages(void)
{
	return atomic64_read(&inuse_pages);
}

static unsigned long dma_heap_system_pool_pages(void)
{
	int i;
	unsigned long pages = 0;

	for (i = 0; i < NUM_ORDERS; i++)
		pages += dmabuf_page_pool_get_size(pools[i]) / PAGE_SIZE;

	return pages;
}

static struct page *alloc_largest_available(unsigned long size,
					    unsigned int max_order)
{
	struct page *page;
	int i;

	for (i = 0; i < NUM_ORDERS; i++) {
		if (size < (PAGE_SIZE << orders[i]))
			continue;
		if (max_order < orders[i])
			continue;

		page = dmabuf_page_pool_alloc(pools[i]);
		if (!page)
			continue;
		dma_heap_inc_inuse(1 << orders[i]);
		return page;
	}
	return NULL;
}

/*
 * free @page directly without caching it to page pool if @discard is true
 * since it's not likely to be reused since the pool is draining now(e.g.,
 * memory pressure) so page zeroing is pointess.
 */
static void free_dma_heap_page(struct page *page, bool discard)
{
	unsigned int order = compound_order(page);

	if (discard) {
		__free_pages(page, order);
	} else {
		int pool_idx;
		int i, numpages = 1 << order;

		for (i = 0; i < numpages; i++)
			clear_highpage(page + i);

		for (pool_idx = 0; pool_idx < NUM_ORDERS; pool_idx++) {
			if (order == orders[pool_idx])
				break;
		}
		dmabuf_page_pool_free(pools[pool_idx], page);
	}
	dma_heap_dec_inuse(1 << order);
}

static struct dma_buf *system_heap_allocate(struct dma_heap *heap, unsigned long len,
					    unsigned long fd_flags, unsigned long heap_flags)
{
	struct samsung_dma_heap *samsung_dma_heap = dma_heap_get_drvdata(heap);
	struct samsung_dma_buffer *buffer;
	struct scatterlist *sg;
	struct dma_buf *dmabuf;
	struct list_head pages;
	struct page *page, *tmp_page;
	unsigned long size_remaining;
	unsigned int max_order = orders[0];
	int i, ret = -ENOMEM;

	if (dma_heap_flags_video_aligned(samsung_dma_heap->flags))
		len = dma_heap_add_video_padding(len);

	if (len / PAGE_SIZE > totalram_pages() / 2) {
		pr_err("pid %d requested too large allocation of size %lu from system heap\n",
		       current->pid, len);
		return ERR_PTR(ret);
	}

	size_remaining = len;

	INIT_LIST_HEAD(&pages);
	i = 0;
	while (size_remaining > 0) {
		/*
		 * Avoid trying to allocate memory if the process
		 * has been killed by SIGKILL
		 */
		if (fatal_signal_pending(current)) {
			perrfn("Fatal signal pending pid #%d", current->pid);
			ret = -EINTR;
			goto free_buffer;
		}

		page = alloc_largest_available(size_remaining, max_order);
		if (!page)
			goto free_buffer;

		list_add_tail(&page->lru, &pages);
		size_remaining -= page_size(page);
		max_order = compound_order(page);
		i++;
	}

	buffer = samsung_dma_buffer_alloc(samsung_dma_heap, len, i);
	if (IS_ERR(buffer)) {
		ret = PTR_ERR(buffer);
		goto free_buffer;
	}

	sg = buffer->sg_table.sgl;
	list_for_each_entry_safe(page, tmp_page, &pages, lru) {
		sg_set_page(sg, page, page_size(page), 0);
		sg = sg_next(sg);
		list_del(&page->lru);
	}

	heap_cache_flush(buffer);

	dmabuf = samsung_export_dmabuf(buffer, fd_flags);
	if (IS_ERR(dmabuf)) {
		ret = PTR_ERR(dmabuf);
		goto free_export;
	}

	return dmabuf;

free_export:
	for_each_sgtable_sg(&buffer->sg_table, sg, i)
		free_dma_heap_page(sg_page(sg), false);
	samsung_dma_buffer_free(buffer);
free_buffer:
	list_for_each_entry_safe(page, tmp_page, &pages, lru)
		free_dma_heap_page(page, false);

	return ERR_PTR(ret);
}

static long system_heap_get_pool_size(struct dma_heap *heap)
{
	/*
	 * All system heaps share the same page pool. Return the size of the pool
	 * only for *the* system heap (required by VTS). Otherwise, the heap pool
	 * use will be overcalculated by the number of registered system heaps.
	 */
	if (strcmp(dma_heap_get_name(heap), "system"))
		return 0;

	return dma_heap_system_pool_pages() << PAGE_SHIFT;
}

static const struct dma_heap_ops system_heap_ops = {
	.allocate = system_heap_allocate,
	.get_pool_size = system_heap_get_pool_size,
};

static void system_heap_free(struct deferred_freelist_item *item, enum df_reason reason)
{
	struct samsung_dma_buffer *buffer;
	struct sg_table *table;
	struct scatterlist *sg;
	int i;

	buffer = container_of(item, struct samsung_dma_buffer, deferred_free);
	table = &buffer->sg_table;
	for_each_sgtable_sg(table, sg, i)
		free_dma_heap_page(sg_page(sg), reason != DF_NORMAL);
	samsung_dma_buffer_free(buffer);
}

static void system_heap_release(struct samsung_dma_buffer *buffer)
{
	int npages = PAGE_ALIGN(buffer->len) / PAGE_SIZE;

	deferred_free(&buffer->deferred_free, system_heap_free, npages);
}

static int system_heap_probe(struct platform_device *pdev)
{
	return samsung_heap_add(&pdev->dev, NULL, system_heap_release, &system_heap_ops);
}

static const struct of_device_id system_heap_of_match[] = {
	{ .compatible = "samsung,dma-heap-system", },
	{ },
};
MODULE_DEVICE_TABLE(of, system_heap_of_match);

static struct platform_driver system_heap_driver = {
	.driver		= {
		.name	= "samsung,dma-heap-system",
		.of_match_table = system_heap_of_match,
	},
	.probe		= system_heap_probe,
};

static unsigned long ion_heap_size(void *private)
{
	return (dma_heap_system_inuse_pages() + dma_heap_gcma_inuse_pages()) << (PAGE_SHIFT - 10);
}

static struct meminfo dma_heap_meminfo = {
	.name = "ION_heap",
	.size_kb = ion_heap_size,
};

static unsigned long ion_heap_pool_size(void *private)
{
	unsigned long pages;

	pages = dma_heap_system_pool_pages();
	return pages << (PAGE_SHIFT - 10);
}

static struct meminfo dma_heap_pool_meminfo = {
	.name = "ION_heap_pool",
	.size_kb = ion_heap_pool_size,
};

int __init system_dma_heap_init(void)
{
	int i;

	for (i = 0; i < NUM_ORDERS; i++) {
		pools[i] = dmabuf_page_pool_create(order_flags[i], orders[i]);
		if (!pools[i]) {
			int j;

			pr_err("%s: page pool creation failed!\n", __func__);
			for (j = 0; j < i; j++)
				dmabuf_page_pool_destroy(pools[j]);
			return -ENOMEM;
		}
	}

	register_meminfo(&dma_heap_meminfo);
	register_meminfo(&dma_heap_pool_meminfo);

	return platform_driver_register(&system_heap_driver);
}

void system_dma_heap_exit(void)
{
	platform_driver_unregister(&system_heap_driver);
}
