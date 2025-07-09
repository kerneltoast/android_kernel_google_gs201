// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2011,2020 Google LLC
 */
#include <linux/spinlock.h>
#include <linux/dma-map-ops.h>
#include <linux/err.h>
#include <linux/genalloc.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>

#include <asm/cacheflush.h>

#include "ion_physical_heap.h"

#define ION_PHYSICAL_ALLOCATE_FAIL -1

static int _clear_pages(struct page **pages, int num, pgprot_t pgprot)
{
	void *addr = vmap(pages, num, VM_MAP, pgprot);

	if (!addr)
		return -ENOMEM;
	memset(addr, 0, PAGE_SIZE * num);
	vunmap(addr);

	return 0;
}

static int _sglist_zero(struct scatterlist *sgl, unsigned int nents,
			pgprot_t pgprot)
{
	int p = 0;
	int ret = 0;
	struct sg_page_iter piter;
	struct page *pages[32];

	for_each_sg_page(sgl, &piter, nents, 0) {
		pages[p++] = sg_page_iter_page(&piter);
		if (p == ARRAY_SIZE(pages)) {
			ret = _clear_pages(pages, p, pgprot);
			if (ret)
				return ret;
			p = 0;
		}
	}
	if (p)
		ret = _clear_pages(pages, p, pgprot);

	return ret;
}

static int _buffer_zero(struct samsung_dma_buffer *buffer)
{
	pgprot_t pgprot = pgprot_writecombine(PAGE_KERNEL);

	return _sglist_zero(buffer->sg_table.sgl, buffer->sg_table.orig_nents, pgprot);
}

static int _pages_zero(struct page *page, size_t size, pgprot_t pgprot)
{
	struct scatterlist sg;

	sg_init_table(&sg, 1);
	sg_set_page(&sg, page, size, 0);
	return _sglist_zero(&sg, 1, pgprot);
}

static int ion_physical_heap_do_allocate(struct dma_heap *heap,
					 struct samsung_dma_buffer *buffer,
					 unsigned long size)
{
	struct samsung_dma_heap *samsung_dma_heap = dma_heap_get_drvdata(heap);
	struct ion_physical_heap *physical_heap = samsung_dma_heap->priv;
	unsigned long aligned_size = ALIGN(size, samsung_dma_heap->alignment);
	phys_addr_t paddr;

	paddr = gen_pool_alloc(physical_heap->pool, aligned_size);
	if (!paddr) {
		pr_err("%s: failed to allocate from AOC physical heap size %lu",
		       __func__, size);
		return -ENOMEM;
	}

	sg_set_page(buffer->sg_table.sgl, pfn_to_page(PFN_DOWN(paddr)), size, 0);
	buffer->priv = (void *)(uintptr_t)hash_long(paddr, 32);

	if (physical_heap->allocate_cb)
		physical_heap->allocate_cb(buffer, physical_heap->allocate_ctx);

	return 0;
}

static void ion_physical_heap_buffer_free(struct samsung_dma_buffer *buffer)
{
	struct ion_physical_heap *physical_heap = buffer->heap->priv;
	struct page *page;
	phys_addr_t paddr;

	page = sg_page(buffer->sg_table.sgl);
	paddr = PFN_PHYS(page_to_pfn(page));

	if (physical_heap->free_cb)
		physical_heap->free_cb(buffer, physical_heap->free_ctx);

	_buffer_zero(buffer);
	if (paddr)
		gen_pool_free(physical_heap->pool, paddr,
			      ALIGN(buffer->len, buffer->heap->alignment));

	samsung_dma_buffer_free(buffer);
}

static struct dma_buf *ion_physical_heap_allocate(struct dma_heap *heap,
						  unsigned long len,
						  unsigned long fd_flags,
						  unsigned long heap_flags)
{
	int ret;
	struct dma_buf *dmabuf;
	struct samsung_dma_buffer *buffer;
	struct samsung_dma_heap *samsung_dma_heap = dma_heap_get_drvdata(heap);

	buffer = samsung_dma_buffer_alloc(samsung_dma_heap, len, 1);
	if (IS_ERR(buffer))
		return ERR_CAST(buffer);

	ret = ion_physical_heap_do_allocate(heap, buffer, len);
	if (ret) {
		samsung_dma_buffer_free(buffer);
		return ERR_PTR(ret);
	}

	dmabuf = samsung_export_dmabuf(buffer, fd_flags);
	if (IS_ERR(dmabuf))
		ion_physical_heap_buffer_free(buffer);

	return dmabuf;
}

static struct dma_heap_ops physical_heap_ops = {
	.allocate = ion_physical_heap_allocate,
};

static struct dma_heap *samsung_dma_heap_create_helper(const char *name,
						       size_t align,
						       void *priv)
{
	struct samsung_dma_heap *samsung_dma_heap;
	struct dma_heap *heap;
	struct dma_heap_export_info exp_info;

	samsung_dma_heap = kzalloc(sizeof(*samsung_dma_heap), GFP_KERNEL);
	if (!samsung_dma_heap)
		return ERR_PTR(-ENOMEM);

	samsung_dma_heap->name = name;
	samsung_dma_heap->release = ion_physical_heap_buffer_free;
	samsung_dma_heap->priv = priv;
	samsung_dma_heap->flags = DMA_HEAP_FLAG_UNCACHED;
	samsung_dma_heap->alignment = align;

	exp_info.name = name;
	exp_info.ops = &physical_heap_ops;
	exp_info.priv = samsung_dma_heap;

	heap = dma_heap_add(&exp_info);
	if (IS_ERR(heap))
		kfree(samsung_dma_heap);

	return heap;
}

struct dma_heap *ion_physical_heap_create(phys_addr_t base, size_t size,
					  size_t align, const char *name,
					  ion_physical_heap_allocate_callback alloc_cb,
					  ion_physical_heap_free_callback free_cb,
					  void *ctx)
{
	struct ion_physical_heap *physical_heap;
	int ret;
	struct page *page;
	struct dma_heap *physical_dmabuf_heap;

	page = pfn_to_page(PFN_DOWN(base));

	ret = _pages_zero(page, size, pgprot_writecombine(PAGE_KERNEL));
	if (ret)
		return ERR_PTR(ret);

	physical_heap = kzalloc(sizeof(*physical_heap), GFP_KERNEL);
	if (!physical_heap)
		return ERR_PTR(-ENOMEM);

	physical_heap->pool = gen_pool_create(get_order(align) + PAGE_SHIFT, -1);
	if (!physical_heap->pool) {
		kfree(physical_heap);
		return ERR_PTR(-ENOMEM);
	}

	physical_heap->base = base;
	gen_pool_add(physical_heap->pool, physical_heap->base, size, -1);

	physical_heap->size = size;

	physical_heap->allocate_cb = alloc_cb;
	physical_heap->allocate_ctx = ctx;

	physical_heap->free_cb = free_cb;
	physical_heap->free_ctx = ctx;

	physical_dmabuf_heap = samsung_dma_heap_create_helper(name, align,
							      (void *)physical_heap);
	if (IS_ERR(physical_dmabuf_heap)) {
		gen_pool_destroy(physical_heap->pool);
		kfree(physical_heap);
	}

	return physical_dmabuf_heap;
}
