// SPDX-License-Identifier: GPL-2.0
/*
 * DMABUF Heap Allocator - dmabuf interface
 *
 * Copyright (C) 2011 Google, Inc.
 * Copyright (C) 2019, 2020 Linaro Ltd.
 * Copyright (C) 2021 Samsung Electronics Co., Ltd.
 *
 * Portions based off of Andrew Davis' SRAM heap:
 * Copyright (C) 2019 Texas Instruments Incorporated - http://www.ti.com/
 *	Andrew F. Davis <afd@ti.com>
 */

#include <linux/dma-buf.h>
#include <linux/dma-heap.h>
#include <linux/err.h>
#include <linux/highmem.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/samsung-dma-heap.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>

static struct sg_table *dup_sg_table(struct sg_table *table)
{
	struct sg_table *new_table;
	int ret, i;
	struct scatterlist *sg, *new_sg;

	new_table = kzalloc(sizeof(*new_table), GFP_KERNEL);
	if (!new_table)
		return ERR_PTR(-ENOMEM);

	ret = sg_alloc_table(new_table, table->orig_nents, GFP_KERNEL);
	if (ret) {
		kfree(new_table);
		return ERR_PTR(-ENOMEM);
	}

	new_sg = new_table->sgl;
	for_each_sgtable_sg(table, sg, i) {
		sg_set_page(new_sg, sg_page(sg), sg->length, sg->offset);
		new_sg = sg_next(new_sg);
	}

	return new_table;
}

static int samsung_heap_attach(struct dma_buf *dmabuf, struct dma_buf_attachment *attachment)
{
	struct samsung_dma_buffer *buffer = dmabuf->priv;
	struct samsung_map_attachment *a;
	struct sg_table *table;

	a = kzalloc(sizeof(*a), GFP_KERNEL);
	if (!a)
		return -ENOMEM;

	table = dup_sg_table(&buffer->sg_table);
	if (IS_ERR(table)) {
		kfree(a);
		return -ENOMEM;
	}

	a->table = table;
	a->dev = attachment->dev;
	a->flags = buffer->flags;

	attachment->priv = a;

	mutex_lock(&buffer->lock);
	list_add(&a->list, &buffer->attachments);
	mutex_unlock(&buffer->lock);

	return 0;
}

static void samsung_heap_detach(struct dma_buf *dmabuf, struct dma_buf_attachment *attachment)
{
	struct samsung_dma_buffer *buffer = dmabuf->priv;
	struct samsung_map_attachment *a = attachment->priv;

	mutex_lock(&buffer->lock);
	list_del(&a->list);
	mutex_unlock(&buffer->lock);

	sg_free_table(a->table);
	kfree(a->table);
	kfree(a);
}

static struct sg_table *samsung_heap_map_dma_buf(struct dma_buf_attachment *attachment,
						 enum dma_data_direction direction)
{
	struct samsung_map_attachment *a = attachment->priv;
	struct sg_table *table = a->table;
	unsigned int attr = 0;
	int ret;

	if (dma_heap_tzmp_buffer(a)) {
		struct samsung_dma_buffer *buffer = attachment->dmabuf->priv;
		struct buffer_prot_info *info = buffer->priv;

		sg_dma_address(table->sgl) = info->dma_addr;
		sg_dma_len(table->sgl) = info->chunk_count * info->chunk_size;
		table->nents = 1;

		return table;
	}
	if (dma_heap_skip_cache_ops(a->flags))
		attr |= DMA_ATTR_SKIP_CPU_SYNC;

	ret = dma_map_sgtable(attachment->dev, table, direction, attr);
	if (ret)
		return ERR_PTR(ret);
	a->mapped = true;
	return table;
}

static void samsung_heap_unmap_dma_buf(struct dma_buf_attachment *attachment,
				       struct sg_table *table,
				       enum dma_data_direction direction)
{
	struct samsung_map_attachment *a = attachment->priv;
	unsigned int attr = 0;

	if (dma_heap_tzmp_buffer(a))
		return;

	if (dma_heap_skip_cache_ops(a->flags))
		attr |= DMA_ATTR_SKIP_CPU_SYNC;

	a->mapped = false;
	dma_unmap_sgtable(attachment->dev, table, direction, attr);
}

static int samsung_heap_dma_buf_begin_cpu_access(struct dma_buf *dmabuf,
						 enum dma_data_direction direction)
{
	struct samsung_dma_buffer *buffer = dmabuf->priv;
	struct samsung_map_attachment *a;

	if (dma_heap_skip_cache_ops(buffer->flags))
		return 0;

	mutex_lock(&buffer->lock);
	list_for_each_entry(a, &buffer->attachments, list) {
		if (!a->mapped)
			continue;
		dma_sync_sgtable_for_cpu(a->dev, a->table, direction);
	}
	mutex_unlock(&buffer->lock);

	return 0;
}

static int samsung_heap_dma_buf_end_cpu_access(struct dma_buf *dmabuf,
					       enum dma_data_direction direction)
{
	struct samsung_dma_buffer *buffer = dmabuf->priv;
	struct samsung_map_attachment *a;

	if (dma_heap_skip_cache_ops(buffer->flags))
		return 0;

	mutex_lock(&buffer->lock);
	list_for_each_entry(a, &buffer->attachments, list) {
		if (!a->mapped)
			continue;
		dma_sync_sgtable_for_device(a->dev, a->table, direction);
	}
	mutex_unlock(&buffer->lock);

	return 0;
}

static int samsung_heap_mmap(struct dma_buf *dmabuf, struct vm_area_struct *vma)
{
	struct samsung_dma_buffer *buffer = dmabuf->priv;
	struct sg_table *table = &buffer->sg_table;
	unsigned long addr = vma->vm_start;
	struct sg_page_iter piter;
	int ret;

	if (dma_heap_flags_protected(buffer->flags)) {
		perr("mmap() to protected buffer is not allowed");
		return -EACCES;
	}

	if (dma_heap_flags_uncached(buffer->flags))
		vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);

	for_each_sgtable_page(table, &piter, vma->vm_pgoff) {
		struct page *page = sg_page_iter_page(&piter);

		ret = remap_pfn_range(vma, addr, page_to_pfn(page), PAGE_SIZE,
				      vma->vm_page_prot);
		if (ret)
			return ret;
		addr += PAGE_SIZE;
		if (addr >= vma->vm_end)
			return 0;
	}
	return 0;
}

static void *samsung_heap_do_vmap(struct samsung_dma_buffer *buffer)
{
	struct sg_table *table = &buffer->sg_table;
	unsigned int npages = PAGE_ALIGN(buffer->len) / PAGE_SIZE;
	struct page **pages = vmalloc(sizeof(struct page *) * npages);
	struct page **tmp = pages;
	struct sg_page_iter piter;
	pgprot_t pgprot;
	void *vaddr;

	if (!pages)
		return ERR_PTR(-ENOMEM);

	if (dma_heap_flags_protected(buffer->flags)) {
		perr("vmap() to protected buffer is not allowed");
		return ERR_PTR(-EACCES);
	}

	pgprot = dma_heap_flags_uncached(buffer->flags) ?
		pgprot_writecombine(PAGE_KERNEL) : PAGE_KERNEL;

	for_each_sgtable_page(table, &piter, 0) {
		WARN_ON(tmp - pages >= npages);
		*tmp++ = sg_page_iter_page(&piter);
	}

	vaddr = vmap(pages, npages, VM_MAP, pgprot);
	vfree(pages);

	if (!vaddr)
		return ERR_PTR(-ENOMEM);

	return vaddr;
}

static void *samsung_heap_vmap(struct dma_buf *dmabuf)
{
	struct samsung_dma_buffer *buffer = dmabuf->priv;
	void *vaddr;

	mutex_lock(&buffer->lock);
	if (buffer->vmap_cnt) {
		buffer->vmap_cnt++;
		vaddr = buffer->vaddr;
		goto out;
	}

	vaddr = samsung_heap_do_vmap(buffer);
	if (IS_ERR(vaddr))
		goto out;

	buffer->vaddr = vaddr;
	buffer->vmap_cnt++;
out:
	mutex_unlock(&buffer->lock);

	return vaddr;
}

static void samsung_heap_vunmap(struct dma_buf *dmabuf, void *vaddr)
{
	struct samsung_dma_buffer *buffer = dmabuf->priv;

	mutex_lock(&buffer->lock);
	if (!--buffer->vmap_cnt) {
		vunmap(buffer->vaddr);
		buffer->vaddr = NULL;
	}
	mutex_unlock(&buffer->lock);
}

static void samsung_heap_dma_buf_release(struct dma_buf *dmabuf)
{
	struct samsung_dma_buffer *buffer = dmabuf->priv;

	buffer->heap->release(buffer);
}

static int samsung_heap_dma_buf_get_flags(struct dma_buf *dmabuf, unsigned long *flags)
{
	struct samsung_dma_buffer *buffer = dmabuf->priv;

	*flags = buffer->flags;

	return 0;
}

const struct dma_buf_ops samsung_dma_buf_ops = {
	.attach = samsung_heap_attach,
	.detach = samsung_heap_detach,
	.map_dma_buf = samsung_heap_map_dma_buf,
	.unmap_dma_buf = samsung_heap_unmap_dma_buf,
	.begin_cpu_access = samsung_heap_dma_buf_begin_cpu_access,
	.end_cpu_access = samsung_heap_dma_buf_end_cpu_access,
	.mmap = samsung_heap_mmap,
	.vmap = samsung_heap_vmap,
	.vunmap = samsung_heap_vunmap,
	.release = samsung_heap_dma_buf_release,
	.get_flags = samsung_heap_dma_buf_get_flags,
};
