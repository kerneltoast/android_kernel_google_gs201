// SPDX-License-Identifier: GPL-2.0
/*
 * DMABUF GCMA heap
 *
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
#include <linux/pfn.h>
#include <soc/google/gcma.h>

#include "samsung-dma-heap.h"
#include "gcma_heap.h"
#include "gcma_heap_sysfs.h"

#define BASE_GFP (GFP_HIGHUSER | __GFP_ZERO | __GFP_COMP)
#define LIGHT_EFFORT_GFP  ((BASE_GFP | __GFP_NOWARN | __GFP_NORETRY) & ~__GFP_RECLAIM)
#define HARD_EFFORT_GFP BASE_GFP
/*
 * The selection of the orders used for allocation (2MB, 1MB, 64K, 4K) is designed
 * to match with the sizes often found in IOMMUs. Using high order pages instead
 * of order 0 pages can significantly improve the performance of many IOMMUs
 * by reducing TLB pressure and time spent updating page tables.
 */
static const unsigned int buddy_pages_orders[] = {9, 8, 4, 0};
static const unsigned int gcma_pages_orders[] = {4};

struct heap_pages {
  struct list_head pages_list;
  unsigned int count;
};

unsigned long dma_heap_gcma_inuse_pages(void)
{
	return atomic64_read(&inuse_pages);
}

static inline unsigned long gcma_get_size(struct page *page)
{
	return page_private(page);
}

static inline void gcma_set_size(struct page *page, unsigned long size)
{
	return set_page_private(page, size);
}

static inline bool page_is_gcma(struct page *page)
{
	return page_private(page) ? true : false;
}

struct page *gcma_alloc(struct gcma_heap *gcma_heap, unsigned long size)
{
	struct gen_pool *pool = gcma_heap->pool;
	phys_addr_t paddr;
	unsigned long pfn;
	struct page *page = NULL;

	paddr = gen_pool_alloc(pool, size);
	if (!paddr)
		return NULL;

	pfn = PFN_DOWN(paddr);
	page = phys_to_page(paddr);
	gcma_alloc_range(pfn, pfn + (size >> PAGE_SHIFT) - 1);
	gcma_set_size(page, size);
	inc_gcma_heap_stat(gcma_heap, USAGE, size);
	/*
	 * zero out pages to align with the strategy in buddy allocator GFP flag
	 */
	if (BASE_GFP & __GFP_ZERO)
		heap_page_clean(page, size);

	return page;
}

void gcma_free(struct gen_pool *pool, struct page *page)
{
	unsigned long size, pfn;

	size = gcma_get_size(page);
	pfn = page_to_pfn(page);
	gcma_free_range(pfn, pfn + (size >> PAGE_SHIFT) - 1);
	gen_pool_free(pool, page_to_phys(page), size);
}

static void free_gcma_heap_page(struct gcma_heap *gcma_heap, struct page *page)
{
	if (unlikely(!page))
		return;

	if (page_is_gcma(page)) {
		gcma_free(gcma_heap->pool, page);
		dec_gcma_heap_stat(gcma_heap, USAGE, gcma_get_size(page));
	} else {
		unsigned int order = compound_order(page);
		__free_pages(page, order);
		dma_heap_dec_inuse(1 << order);
	}
}

/*
 * 1. Try all orders(higher to lower) with light effort
 * 2. Try GCMA allocation
 * 3. Try order-0 page with hard effort.
 */
static struct page *alloc_largest_available(struct gcma_heap *gcma_heap,
					    unsigned long size,
					    unsigned int max_order)
{
	struct page *page = NULL;
	int i;

	for (i = 0; i < ARRAY_SIZE(buddy_pages_orders); i++) {
		unsigned long buddy_size = PAGE_SIZE << buddy_pages_orders[i];
		if (size < buddy_size && i != ARRAY_SIZE(buddy_pages_orders) - 1)
			continue;
		if (max_order < buddy_pages_orders[i])
			continue;

		page = alloc_pages(LIGHT_EFFORT_GFP, buddy_pages_orders[i]);
		if (page)
			goto out;
	}

	for (i = 0; i < ARRAY_SIZE(gcma_pages_orders); i++) {
		unsigned long gcma_size = PAGE_SIZE << gcma_pages_orders[i];
		if (size < gcma_size && i != ARRAY_SIZE(gcma_pages_orders) - 1)
			continue;

		page = gcma_alloc(gcma_heap, gcma_size);
		if (page)
			goto out;
	}

	page = alloc_pages(HARD_EFFORT_GFP, 0);
	inc_gcma_heap_stat(gcma_heap, ALLOCSTALL, PAGE_SIZE);
out:
	if (page && !page_is_gcma(page))
		dma_heap_inc_inuse(1 << compound_order(page));
	return page;
}

static int allocate_flexible_pages(struct gcma_heap *gcma_heap, unsigned long len,
					    struct heap_pages *heap_pages)
{
	struct page *page, *tmp_page;
	unsigned long size_remaining = len;
	unsigned int max_order = buddy_pages_orders[0];
	unsigned int count = 0;
	int ret = 0;

	while (size_remaining > 0) {
		unsigned long allocated_size;
		/*
		 * Avoid trying to allocate memory if the process
		 * has been killed by SIGKILL
		 */
		if (fatal_signal_pending(current)) {
			pr_err("Fatal signal pending pid #%d", current->pid);
			ret = -EINTR;
			goto free_flexible_pages;
		}

		page = alloc_largest_available(gcma_heap, size_remaining, max_order);
		if (!page) {
			ret = -ENOMEM;
			goto free_flexible_pages;
		}

		list_add_tail(&page->lru, &heap_pages->pages_list);
		allocated_size = page_is_gcma(page) ? gcma_get_size(page) : page_size(page);
		if (allocated_size > size_remaining)
			size_remaining = 0;
		else
			size_remaining -= allocated_size;
		max_order = page_is_gcma(page) ? max_order : compound_order(page);
		count++;
	}
	heap_pages->count = count;
	goto out_flexible_alloc;

free_flexible_pages:
	list_for_each_entry_safe(page, tmp_page, &heap_pages->pages_list, lru)
		free_gcma_heap_page(gcma_heap, page);
out_flexible_alloc:
	return ret;
}

static int allocate_fixed_pages(struct gcma_heap *gcma_heap, unsigned long len,
					    struct heap_pages *heap_pages)
{
	struct page *page = gcma_alloc(gcma_heap, len);
	if (page) {
		list_add_tail(&page->lru, &heap_pages->pages_list);
		heap_pages->count = 1;
		return 0;
	}
	return -ENOMEM;
}

static struct dma_buf *gcma_heap_allocate(struct dma_heap *heap, unsigned long len,
					    unsigned long fd_flags, unsigned long heap_flaga)
{
	struct samsung_dma_heap *samsung_dma_heap = dma_heap_get_drvdata(heap);
	struct gcma_heap *gcma_heap = samsung_dma_heap->priv;
	bool is_secure_heap = dma_heap_flags_protected(samsung_dma_heap->flags);
	bool flexible_alloc = gcma_heap->flexible_alloc;
	struct samsung_dma_buffer *buffer;
	struct scatterlist *sg;
	struct dma_buf *dmabuf;
	unsigned int alignment = samsung_dma_heap->alignment;
	struct page *page, *tmp_page;
	struct heap_pages heap_pages;
	int ret = -ENOMEM;

	/* We don't support a secure heap with the flexible_alloc strategy */
	if (dma_heap_flags_video_aligned(samsung_dma_heap->flags))
		len = dma_heap_add_video_padding(len);

	if (len / PAGE_SIZE > totalram_pages() / 2) {
		pr_err("pid %d requested too large allocation of size %lu from %s heap\n",
		       current->pid, len, samsung_dma_heap->name);
		return ERR_PTR(ret);
	}

	INIT_LIST_HEAD(&heap_pages.pages_list);
	len = ALIGN(len, alignment);

	if (flexible_alloc) {
		ret = allocate_flexible_pages(gcma_heap, len, &heap_pages);
	} else {
		ret = allocate_fixed_pages(gcma_heap, len, &heap_pages);
	}

	if (ret)
		goto out;

	buffer = samsung_dma_buffer_alloc(samsung_dma_heap, len, heap_pages.count);
	if (IS_ERR(buffer)) {
		ret = PTR_ERR(buffer);
		goto free_buffer;
	}

	sg = buffer->sg_table.sgl;
	list_for_each_entry_safe(page, tmp_page, &heap_pages.pages_list, lru) {
		sg_set_page(sg, page,
			   page_is_gcma(page) ? gcma_get_size(page) : page_size(page), 0);
		sg = sg_next(sg);
		list_del(&page->lru);
	}

	heap_cache_flush(buffer);

	if (is_secure_heap) {
		buffer->priv = samsung_dma_buffer_protect(
				buffer, len, heap_pages.count,
				page_to_phys(list_first_entry(&heap_pages.pages_list,
							     struct page, lru)));
		if (IS_ERR(buffer->priv)) {
			ret = PTR_ERR(buffer->priv);
			buffer->priv = NULL;
			goto free_export;
		}
	}

	dmabuf = samsung_export_dmabuf(buffer, fd_flags);
	if (IS_ERR(dmabuf)) {
		ret = PTR_ERR(dmabuf);
		goto free_export;
	}

	return dmabuf;

free_export:
	if (is_secure_heap ? !samsung_dma_buffer_unprotect(buffer) : 1)
		for_each_sgtable_sg(&buffer->sg_table, sg, heap_pages.count)
			free_gcma_heap_page(gcma_heap, sg_page(sg));
free_buffer:
	list_for_each_entry_safe(page, tmp_page, &heap_pages.pages_list, lru)
		free_gcma_heap_page(gcma_heap, page);

	samsung_dma_buffer_free(buffer);
out:
	pr_err("failed to allocate from %s heap, size %lu ret %d",
		      samsung_dma_heap->name, len, ret);

	return ERR_PTR(ret);
}

static void gcma_heap_release(struct samsung_dma_buffer *buffer)
{
	struct samsung_dma_heap *samsung_dma_heap = buffer->heap;
	struct gcma_heap *gcma_heap = samsung_dma_heap->priv;
	bool is_secure_heap = dma_heap_flags_protected(samsung_dma_heap->flags);
	int ret = 0;

	/* We don't support a secure heap with the flexible_alloc strategy */
	if (is_secure_heap)
		ret = samsung_dma_buffer_unprotect(buffer);

	if (!ret) {
		struct sg_table *table;
		struct scatterlist *sg;
		int i;

		gcma_heap = buffer->heap->priv;
		table = &buffer->sg_table;
		/*
		 * free @page directly without caching it to page pool now as after a
		 * long time use, we won't have high order pages anyway.
		 */
		for_each_sgtable_sg(table, sg, i)
			free_gcma_heap_page(gcma_heap, sg_page(sg));
	}
	samsung_dma_buffer_free(buffer);
}

static const struct dma_heap_ops gcma_heap_ops = {
	.allocate = gcma_heap_allocate,
};

static int gcma_heap_probe(struct platform_device *pdev)
{
	struct gcma_heap *gcma_heap;
	struct reserved_mem *rmem;
	struct device_node *rmem_np;
	bool is_secure_heap;
	int ret;

	rmem_np = of_parse_phandle(pdev->dev.of_node, "memory-region", 0);
	if (!rmem_np)
		return -ENODEV;

	rmem = of_reserved_mem_lookup(rmem_np);
	if (!rmem) {
		perrdev(&pdev->dev, "memory-region handle not found");
		return -ENODEV;
	}

	gcma_heap = devm_kzalloc(&pdev->dev, sizeof(*gcma_heap), GFP_KERNEL);
	if (!gcma_heap)
		return -ENOMEM;

	ret = register_gcma_area(rmem->name, rmem->base, rmem->size);
	if (ret)
		return ret;

	gcma_heap->pool = devm_gen_pool_create(&pdev->dev, PAGE_SHIFT, -1, 0);
	if (!gcma_heap->pool)
		return -ENOMEM;

	ret = gen_pool_add(gcma_heap->pool, rmem->base, rmem->size, -1);
	if (ret)
		return ret;

	gcma_heap->flexible_alloc =
		of_property_read_bool(pdev->dev.of_node,"dma-heap-gcam,fleixble-alloc");

	is_secure_heap = of_property_read_bool(pdev->dev.of_node,"dma-heap,secure");

	if (is_secure_heap && gcma_heap->flexible_alloc) {
		perrfn("Don't support a secure heap with a flexible_alloc strategy");
		return -EPERM;
	}

	ret = samsung_heap_add(&pdev->dev, gcma_heap, gcma_heap_release,
			       &gcma_heap_ops);
	if (ret == -ENODEV)
		return 0;

	register_heap_sysfs(gcma_heap, pdev->name);

	return ret;
}

static const struct of_device_id gcma_heap_of_match[] = {
	{ .compatible = "google,dma-heap-gcma", },
	{ },
};
MODULE_DEVICE_TABLE(of, gcma_heap_of_match);

static struct platform_driver gcma_heap_driver = {
	.driver		= {
		.name	= "google,dma-heap-gcma",
		.of_match_table = gcma_heap_of_match,
	},
	.probe		= gcma_heap_probe,
};

int __init gcma_dma_heap_init(void)
{
	gcma_heap_sysfs_init();
	return platform_driver_register(&gcma_heap_driver);
}

void gcma_dma_heap_exit(void)
{
	platform_driver_unregister(&gcma_heap_driver);
}
