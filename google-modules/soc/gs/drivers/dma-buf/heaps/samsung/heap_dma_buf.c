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
#include <linux/dma-direct.h>
#include <linux/dma-heap.h>
#include <linux/dma-map-ops.h>
#include <linux/err.h>
#include <linux/highmem.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/samsung-dma-mapping.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <uapi/linux/dma-buf.h>

#include "dmabuf_heap_trace.h"
#include "samsung-dma-heap.h"

struct dma_iovm_map {
	struct list_head list;
	struct device *dev;
	struct sg_table table;
	unsigned long attrs;
	unsigned int mapcnt;
	enum dma_data_direction dir;
};

static struct dma_iovm_map *dma_iova_create(struct dma_buf_attachment *a,
					    enum dma_data_direction dir)
{
	struct samsung_dma_buffer *buffer = a->dmabuf->priv;
	struct dma_iovm_map *iovm_map;
	struct scatterlist *sg, *new_sg;
	struct sg_table *table = &buffer->sg_table;
	int i;

	iovm_map = kzalloc(sizeof(*iovm_map), GFP_KERNEL);
	if (!iovm_map)
		return NULL;

	if (sg_alloc_table(&iovm_map->table, table->orig_nents, GFP_KERNEL)) {
		kfree(iovm_map);
		return NULL;
	}

	new_sg = iovm_map->table.sgl;
	for_each_sgtable_sg(table, sg, i) {
		sg_set_page(new_sg, sg_page(sg), sg->length, sg->offset);
		new_sg = sg_next(new_sg);
	}

	iovm_map->dev = a->dev;
	iovm_map->attrs = a->dma_map_attrs;
	iovm_map->dir = dir;

	return iovm_map;
}

static void dma_iova_remove(struct dma_iovm_map *iovm_map)
{
	sg_free_table(&iovm_map->table);
	kfree(iovm_map);
}

static void dma_iova_release(struct dma_buf *dmabuf)
{
	struct samsung_dma_buffer *buffer = dmabuf->priv;
	struct dma_iovm_map *iovm_map, *tmp;

	list_for_each_entry_safe(iovm_map, tmp, &buffer->attachments, list) {
		if (iovm_map->mapcnt)
			WARN(1, "iova_map refcount leak found for %s\n",
			     dev_name(iovm_map->dev));

		list_del(&iovm_map->list);
		if (!dma_heap_tzmp_buffer(iovm_map->dev, buffer->flags))
			dma_unmap_sgtable(iovm_map->dev, &iovm_map->table,
					  iovm_map->dir, DMA_ATTR_SKIP_CPU_SYNC);
		dma_iova_remove(iovm_map);
	}
}

#define DMA_MAP_ATTRS_MASK	DMA_ATTR_PRIVILEGED
#define DMA_MAP_ATTRS(attrs)	((attrs) & DMA_MAP_ATTRS_MASK)

/* this function should only be called while buffer->lock is held */
static struct dma_iovm_map *dma_find_iovm_map(struct dma_buf_attachment *a,
					      enum dma_data_direction dir)
{
	struct samsung_dma_buffer *buffer = a->dmabuf->priv;
	struct dma_iovm_map *iovm_map;
	unsigned long attrs;

	if (dma_heap_flags_uncached(buffer->flags)) {
		/*
		 * If the device of sharable domain would access non-cachable
		 * memory with sharable mapping, device could access prefetched clean
		 * cache data which is not coherent with memory, so we need to map
		 * non-sharable for non-cached. To support non-sharable mapping,
		 * DMA_ATTR_PRIVILEGED is set because samsung sysmmu driver clear
		 * the sharable bit when DMA_ATTR_PRIVILEGED (i.e. IOMMU_PRIV) is set.
		 */
		a->dma_map_attrs |= (DMA_ATTR_PRIVILEGED | DMA_ATTR_SKIP_CPU_SYNC);
	}
	attrs = DMA_MAP_ATTRS(a->dma_map_attrs);

	list_for_each_entry(iovm_map, &buffer->attachments, list) {
		if ((iommu_get_domain_for_dev(iovm_map->dev) ==
		    iommu_get_domain_for_dev(a->dev)) &&
		    (DMA_MAP_ATTRS(iovm_map->attrs) == attrs) &&
		    (iovm_map->dir == dir)) {
			return iovm_map;
		}
	}
	return NULL;
}

static struct dma_iovm_map *dma_put_iovm_map(struct dma_buf_attachment *a,
					     enum dma_data_direction dir)
{
	struct samsung_dma_buffer *buffer = a->dmabuf->priv;
	struct dma_iovm_map *iovm_map;

	mutex_lock(&buffer->lock);
	iovm_map = dma_find_iovm_map(a, dir);
	if (iovm_map) {
		iovm_map->mapcnt--;

		if (!iovm_map->mapcnt && (a->dma_map_attrs & DMA_ATTR_SKIP_LAZY_UNMAP)) {
			list_del(&iovm_map->list);
			if (!dma_heap_tzmp_buffer(iovm_map->dev, buffer->flags))
				dma_unmap_sgtable(iovm_map->dev, &iovm_map->table,
						  iovm_map->dir, DMA_ATTR_SKIP_CPU_SYNC);
			dma_iova_remove(iovm_map);
			iovm_map = NULL;
		}
	}
	mutex_unlock(&buffer->lock);

	return iovm_map;
}

static struct dma_iovm_map *dma_get_iovm_map(struct dma_buf_attachment *a,
					     enum dma_data_direction direction)
{
	struct samsung_dma_buffer *buffer = a->dmabuf->priv;
	struct dma_iovm_map *iovm_map, *dup_iovm_map;
	int ret;

	mutex_lock(&buffer->lock);
	iovm_map = dma_find_iovm_map(a, direction);
	if (iovm_map) {
		iovm_map->mapcnt++;
		mutex_unlock(&buffer->lock);
		return iovm_map;
	}
	mutex_unlock(&buffer->lock);

	iovm_map = dma_iova_create(a, direction);
	if (!iovm_map)
		return NULL;

	if (dma_heap_tzmp_buffer(iovm_map->dev, buffer->flags)) {
		struct buffer_prot_info *info = buffer->priv;

		sg_dma_address(iovm_map->table.sgl) = info->dma_addr;
		sg_dma_len(iovm_map->table.sgl) = info->chunk_count * info->chunk_size;

		iovm_map->table.nents = 1;
	} else {
		ret = dma_map_sgtable(iovm_map->dev, &iovm_map->table, iovm_map->dir,
				      iovm_map->attrs | DMA_ATTR_SKIP_CPU_SYNC);
		if (ret) {
			dma_iova_remove(iovm_map);
			return NULL;
		}
	}

	mutex_lock(&buffer->lock);
	dup_iovm_map = dma_find_iovm_map(a, direction);
	if (!dup_iovm_map) {
		list_add(&iovm_map->list, &buffer->attachments);
	} else {
		if (!dma_heap_tzmp_buffer(iovm_map->dev, buffer->flags))
			dma_unmap_sgtable(iovm_map->dev, &iovm_map->table, iovm_map->dir,
					  DMA_ATTR_SKIP_CPU_SYNC);
		dma_iova_remove(iovm_map);
		iovm_map = dup_iovm_map;
	}
	iovm_map->mapcnt++;
	mutex_unlock(&buffer->lock);

	return iovm_map;
}

static struct sg_table *samsung_heap_map_dma_buf(struct dma_buf_attachment *a,
						 enum dma_data_direction direction)
{
	struct dma_iovm_map *iovm_map;
	struct samsung_dma_buffer *buffer = a->dmabuf->priv;

	iovm_map = dma_get_iovm_map(a, direction);
	if (!iovm_map)
		return ERR_PTR(-ENOMEM);

	if (!(dma_heap_skip_cache_ops(buffer->flags) || a->dma_map_attrs & DMA_ATTR_SKIP_CPU_SYNC))
		dma_sync_sgtable_for_device(iovm_map->dev, &iovm_map->table, direction);

	return &iovm_map->table;
}

static void samsung_heap_unmap_dma_buf(struct dma_buf_attachment *a,
				       struct sg_table *table,
				       enum dma_data_direction direction)
{
	struct samsung_dma_buffer *buffer = a->dmabuf->priv;

	if (!(dma_heap_skip_cache_ops(buffer->flags) || a->dma_map_attrs & DMA_ATTR_SKIP_CPU_SYNC))
		dma_sync_sgtable_for_cpu(a->dev, table, direction);

	dma_put_iovm_map(a, direction);
}

static int samsung_heap_dma_buf_begin_cpu_access(struct dma_buf *dmabuf,
						 enum dma_data_direction direction)
{
	struct samsung_dma_buffer *buffer = dmabuf->priv;
	struct dma_iovm_map *iovm_map;

	if (dma_heap_skip_cache_ops(buffer->flags))
		return 0;

	mutex_lock(&buffer->lock);
	list_for_each_entry(iovm_map, &buffer->attachments, list) {
		if (iovm_map->mapcnt && !dev_is_dma_coherent(iovm_map->dev)) {
			dma_sync_sgtable_for_cpu(iovm_map->dev, &iovm_map->table, direction);
			break;
		}
	}
	mutex_unlock(&buffer->lock);

	return 0;
}

static int samsung_heap_dma_buf_end_cpu_access(struct dma_buf *dmabuf,
					       enum dma_data_direction direction)
{
	struct samsung_dma_buffer *buffer = dmabuf->priv;
	struct dma_iovm_map *iovm_map;

	if (dma_heap_skip_cache_ops(buffer->flags))
		return 0;

	mutex_lock(&buffer->lock);
	list_for_each_entry(iovm_map, &buffer->attachments, list) {
		if (iovm_map->mapcnt && !dev_is_dma_coherent(iovm_map->dev)) {
			dma_sync_sgtable_for_device(iovm_map->dev, &iovm_map->table, direction);
			break;
		}
	}
	mutex_unlock(&buffer->lock);

	return 0;
}

static void dma_sync_sg_partial(struct dma_buf *dmabuf, enum dma_data_direction direction,
				unsigned int offset, unsigned int len, unsigned long flag)
{
	struct samsung_dma_buffer *buffer = dmabuf->priv;
	struct device *dev = dma_heap_get_dev(buffer->heap->dma_heap);
	struct dma_iovm_map *iovm_map;
	struct sg_table *sgt = NULL;
	struct scatterlist *sg;
	int i;
	unsigned int size;

	if (dma_heap_skip_cache_ops(buffer->flags))
		return;

	mutex_lock(&buffer->lock);
	list_for_each_entry(iovm_map, &buffer->attachments, list) {
		if (iovm_map->mapcnt && !dev_is_dma_coherent(iovm_map->dev)) {
			sgt = &iovm_map->table;
			break;
		}
	}
	mutex_unlock(&buffer->lock);

	if (!sgt)
		return;

	for_each_sgtable_sg(sgt, sg, i) {
		dma_addr_t dma_addr;

		if (offset >= sg->length) {
			offset -= sg->length;
			continue;
		}

		size = min_t(unsigned int, len, sg->length - offset);
		len -= size;

		dma_addr = phys_to_dma(dev, sg_phys(sg));

		if (flag & DMA_BUF_SYNC_END)
			dma_sync_single_range_for_device(dev, dma_addr, offset, size, direction);
		else
			dma_sync_single_range_for_cpu(dev, dma_addr, offset, size, direction);

		offset = 0;

		if (!len)
			break;
	}
}

static int samsung_heap_dma_buf_begin_cpu_access_partial(struct dma_buf *dmabuf,
							 enum dma_data_direction direction,
							 unsigned int offset, unsigned int len)
{
	dma_sync_sg_partial(dmabuf, direction, offset, len, DMA_BUF_SYNC_START);

	return 0;
}

static int samsung_heap_dma_buf_end_cpu_access_partial(struct dma_buf *dmabuf,
						       enum dma_data_direction direction,
						       unsigned int offset, unsigned int len)
{
	dma_sync_sg_partial(dmabuf, direction, offset, len, DMA_BUF_SYNC_END);

	return 0;
}

static int samsung_heap_mmap(struct dma_buf *dmabuf, struct vm_area_struct *vma)
{
	struct samsung_dma_buffer *buffer = dmabuf->priv;
	struct sg_table *table = &buffer->sg_table;
	unsigned long addr = vma->vm_start;
	struct sg_page_iter piter;
	int ret;

	if (dma_heap_flags_static_protected(buffer->flags)) {
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

	if (dma_heap_flags_static_protected(buffer->flags)) {
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

static int samsung_heap_vmap(struct dma_buf *dmabuf, struct iosys_map *map)
{
	struct samsung_dma_buffer *buffer = dmabuf->priv;
	void *vaddr;
	int ret = 0;

	mutex_lock(&buffer->lock);
	if (buffer->vmap_cnt) {
		buffer->vmap_cnt++;
		iosys_map_set_vaddr(map, buffer->vaddr);
		goto out;
	}

	vaddr = samsung_heap_do_vmap(buffer);
	if (IS_ERR(vaddr)) {
		ret = PTR_ERR(vaddr);
		goto out;
	}

	buffer->vaddr = vaddr;
	buffer->vmap_cnt++;
	iosys_map_set_vaddr(map, buffer->vaddr);
out:
	mutex_unlock(&buffer->lock);

	return ret;
}

static void samsung_heap_vunmap(struct dma_buf *dmabuf, struct iosys_map *map)
{
	struct samsung_dma_buffer *buffer = dmabuf->priv;

	mutex_lock(&buffer->lock);
	if (!--buffer->vmap_cnt) {
		vunmap(buffer->vaddr);
		buffer->vaddr = NULL;
	}
	mutex_unlock(&buffer->lock);
	iosys_map_clear(map);
}

static void samsung_heap_dma_buf_release(struct dma_buf *dmabuf)
{
	struct samsung_dma_buffer *buffer = dmabuf->priv;

	dma_iova_release(dmabuf);

	samsung_track_buffer_destroyed(buffer);
	buffer->heap->release(buffer);
}

static int samsung_heap_dma_buf_get_flags(struct dma_buf *dmabuf, unsigned long *flags)
{
	struct samsung_dma_buffer *buffer = dmabuf->priv;

	*flags = buffer->flags;

	return 0;
}

const struct dma_buf_ops samsung_dma_buf_ops = {
	.map_dma_buf = samsung_heap_map_dma_buf,
	.unmap_dma_buf = samsung_heap_unmap_dma_buf,
	.begin_cpu_access = samsung_heap_dma_buf_begin_cpu_access,
	.end_cpu_access = samsung_heap_dma_buf_end_cpu_access,
	.begin_cpu_access_partial = samsung_heap_dma_buf_begin_cpu_access_partial,
	.end_cpu_access_partial = samsung_heap_dma_buf_end_cpu_access_partial,
	.mmap = samsung_heap_mmap,
	.vmap = samsung_heap_vmap,
	.vunmap = samsung_heap_vunmap,
	.release = samsung_heap_dma_buf_release,
	.get_flags = samsung_heap_dma_buf_get_flags,
};
