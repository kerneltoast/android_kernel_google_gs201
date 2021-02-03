// SPDX-License-Identifier: GPL-2.0
/*
 * ION Memory Allocator exynos feature support
 *
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 */

#include <linux/mm.h>
#include <linux/device.h>
#include <linux/of_device.h>
#include <linux/module.h>
#include <linux/highmem.h>
#include <linux/slab.h>
#include <linux/ion.h>
#include <linux/dma-mapping.h>
#include <linux/scatterlist.h>

#include "ion_exynos.h"
#include "ion_exynos_prot.h"

void ion_page_clean(struct page *pages, unsigned long size)
{
	unsigned long nr_pages, i;

	if (!PageHighMem(pages)) {
		memset(page_address(pages), 0, size);
		return;
	}

	nr_pages = PAGE_ALIGN(size) >> PAGE_SHIFT;

	for (i = 0; i < nr_pages; i++) {
		void *vaddr = kmap_atomic(&pages[i]);

		memset(vaddr, 0, PAGE_SIZE);
		kunmap_atomic(vaddr);
	}
}

static inline bool ion_buffer_cached(struct ion_buffer *buffer)
{
	return !!(buffer->flags & ION_FLAG_CACHED);
}

void exynos_fdt_setup(struct device *dev, struct exynos_fdt_attrs *attrs)
{
	of_property_read_u32(dev->of_node, "ion,alignment", &attrs->alignment);
	attrs->alignment = 1UL << (get_order(attrs->alignment) + PAGE_SHIFT);

	attrs->secure = of_property_read_bool(dev->of_node, "ion,secure");
	of_property_read_u32(dev->of_node, "ion,protection_id",
			     &attrs->protection_id);
}

static int exynos_dma_buf_mmap(struct dma_buf *dmabuf,
			       struct vm_area_struct *vma)
{
	struct ion_buffer *buffer = dmabuf->priv;
	int ret;

	if (ion_buffer_protected(buffer)) {
		perr("mmap() to protected buffer is not allowed");
		return -EACCES;
	}

	mutex_lock(&buffer->lock);
	if (!ion_buffer_cached(buffer))
		vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);

	ret = ion_heap_map_user(buffer->heap, buffer, vma);
	mutex_unlock(&buffer->lock);

	if (ret)
		perr("failure mapping buffer to userspace");

	return ret;
}

static int exynos_dma_buf_attach(struct dma_buf *dmabuf,
				 struct dma_buf_attachment *att)
{
	return 0;
}

static void exynos_dma_buf_detach(struct dma_buf *dmabuf,
				  struct dma_buf_attachment *att)
{
}

struct exynos_iovm_map {
	struct list_head list;
	struct device *dev;
	struct sg_table table;
	unsigned long attrs;
	unsigned int mapcnt;
};

static struct exynos_iovm_map *exynos_iova_create(struct dma_buf_attachment *a)
{
	struct ion_buffer *buffer = a->dmabuf->priv;
	struct exynos_iovm_map *iovm_map;
	struct scatterlist *sg, *new_sg;
	struct sg_table *table = buffer->sg_table;
	int i;

	iovm_map = kzalloc(sizeof(*iovm_map), GFP_KERNEL);
	if (!iovm_map)
		return NULL;

	if (sg_alloc_table(&iovm_map->table, table->nents, GFP_KERNEL)) {
		kfree(iovm_map);
		return NULL;
	}

	new_sg = iovm_map->table.sgl;
	for_each_sg(table->sgl, sg, table->nents, i) {
		memcpy(new_sg, sg, sizeof(*sg));
		new_sg->dma_address = 0;
		new_sg = sg_next(new_sg);
	}

	iovm_map->dev = a->dev;
	iovm_map->attrs = a->dma_map_attrs;
	iovm_map->mapcnt = 1;

	return iovm_map;
}

static void exynos_iova_remove(struct exynos_iovm_map *iovm_map)
{
	sg_free_table(&iovm_map->table);
	kfree(iovm_map);
}

static void exynos_iova_release(struct dma_buf *dmabuf)
{
	struct ion_buffer *buffer = dmabuf->priv;
	struct exynos_iovm_map *iovm_map, *tmp;
	bool prot = IS_ENABLED(CONFIG_EXYNOS_CONTENT_PATH_PROTECTION) &&
		ion_buffer_protected(buffer);

	list_for_each_entry_safe(iovm_map, tmp, &buffer->attachments, list) {
		if (iovm_map->mapcnt)
			WARN(1, "iova_map refcount leak found for %s\n",
			     dev_name(iovm_map->dev));

		list_del(&iovm_map->list);
		if (!prot)
			dma_unmap_sg_attrs(iovm_map->dev, iovm_map->table.sgl,
					   iovm_map->table.orig_nents,
					   DMA_TO_DEVICE,
					   DMA_ATTR_SKIP_CPU_SYNC);
		exynos_iova_remove(iovm_map);
	}
}

/*
 * If dma_map_attrs affects device virtual address mapping
 * that should be added on here.
 *
 * DMA_ATTR_PRIVILEGED : No sharable mapping.
 */
#define DMA_MAP_ATTRS_MASK	DMA_ATTR_PRIVILEGED
#define DMA_MAP_ATTRS(attrs)	((attrs) & DMA_MAP_ATTRS_MASK)

static void exynos_set_noncached_attrs(struct dma_buf_attachment *a)
{
	struct ion_buffer *buffer = a->dmabuf->priv;

	if (ion_buffer_cached(buffer))
		return;

	/*
	 * Non-cached buffer is mapped with DMA_ATTR_PRIVILEGED for
	 * unsharable mapping because non-cached buffer is flushed when
	 * allocated and mapped as non-cached.
	 * The device of sharable domain doesn't need to observe the memory.
	 */
	a->dma_map_attrs |= (DMA_ATTR_PRIVILEGED | DMA_ATTR_SKIP_CPU_SYNC);
}

/* this function should only be called while buffer->lock is held */
static struct exynos_iovm_map *find_iovm_map(struct dma_buf_attachment *a)
{
	struct ion_buffer *buffer = a->dmabuf->priv;
	struct exynos_iovm_map *iovm_map;
	unsigned long attrs;

	exynos_set_noncached_attrs(a);
	attrs = DMA_MAP_ATTRS(a->dma_map_attrs);

	list_for_each_entry(iovm_map, &buffer->attachments, list) {
		// device virtual mapping doesn't consider direction currently.
		if (iovm_map->dev == a->dev &&
		    DMA_MAP_ATTRS(iovm_map->attrs) == attrs) {
			return iovm_map;
		}
	}
	return NULL;
}

static struct exynos_iovm_map *exynos_put_iovm_map(struct dma_buf_attachment *a)
{
	struct exynos_iovm_map *iovm_map;
	struct ion_buffer *buffer = a->dmabuf->priv;

	mutex_lock(&buffer->lock);
	iovm_map = find_iovm_map(a);
	if (iovm_map)
		iovm_map->mapcnt--;
	mutex_unlock(&buffer->lock);

	return iovm_map;
}

static struct exynos_iovm_map *exynos_get_iovm_map(struct dma_buf_attachment *a,
						   enum dma_data_direction dir)
{
	struct ion_buffer *buffer = a->dmabuf->priv;
	struct exynos_iovm_map *iovm_map;
	bool prot = IS_ENABLED(CONFIG_EXYNOS_CONTENT_PATH_PROTECTION) &&
		    ion_buffer_protected(buffer);

	mutex_lock(&buffer->lock);
	iovm_map = find_iovm_map(a);
	if (iovm_map) {
		iovm_map->mapcnt++;
		mutex_unlock(&buffer->lock);
		return iovm_map;
	}
	mutex_unlock(&buffer->lock);

	iovm_map = exynos_iova_create(a);
	if (!iovm_map)
		return NULL;

	if (prot && buffer->priv_virt &&
	    is_dev_samsung_tzmp(a->dev)) {
		struct ion_buffer_prot_info *info = buffer->priv_virt;

		iovm_map->table.sgl->dma_address = info->dma_addr;
	} else if (!dma_map_sg_attrs(iovm_map->dev,
				     iovm_map->table.sgl,
				     iovm_map->table.nents, dir,
				     iovm_map->attrs |
				     DMA_ATTR_SKIP_CPU_SYNC)) {
		exynos_iova_remove(iovm_map);
		return NULL;
	}

	mutex_lock(&buffer->lock);
	list_add(&iovm_map->list, &buffer->attachments);
	mutex_unlock(&buffer->lock);

	return iovm_map;
}

static struct sg_table *exynos_map_dma_buf(struct dma_buf_attachment *att,
					   enum dma_data_direction direction)
{
	struct ion_buffer *buffer = att->dmabuf->priv;
	struct exynos_iovm_map *iovm_map;
	bool prot = IS_ENABLED(CONFIG_EXYNOS_CONTENT_PATH_PROTECTION) &&
		ion_buffer_protected(buffer);

	iovm_map = exynos_get_iovm_map(att, direction);
	if (!iovm_map)
		return ERR_PTR(-ENOMEM);

	if (!prot && ion_buffer_cached(buffer))
		dma_sync_sg_for_device(iovm_map->dev,
				       iovm_map->table.sgl,
				       iovm_map->table.nents,
				       direction);

	return &iovm_map->table;
}

static void exynos_unmap_dma_buf(struct dma_buf_attachment *att,
				 struct sg_table *table,
				 enum dma_data_direction direction)
{
	struct ion_buffer *buffer = att->dmabuf->priv;
	struct exynos_iovm_map *iovm_map;
	bool prot = IS_ENABLED(CONFIG_EXYNOS_CONTENT_PATH_PROTECTION) &&
		ion_buffer_protected(buffer);

	iovm_map = exynos_put_iovm_map(att);
	if (!iovm_map || !ion_buffer_cached(buffer) ||  prot)
		return;

	dma_sync_sg_for_cpu(iovm_map->dev, iovm_map->table.sgl,
			    iovm_map->table.nents, direction);
}

static void exynos_dma_buf_release(struct dma_buf *dmabuf)
{
	struct ion_buffer *buffer = dmabuf->priv;

	exynos_iova_release(dmabuf);

	ion_free(buffer);
}

void *exynos_buffer_kmap_get(struct ion_buffer *buffer)
{
	void *vaddr;

	if (buffer->kmap_cnt) {
		buffer->kmap_cnt++;
		return buffer->vaddr;
	}
	vaddr = ion_heap_map_kernel(buffer->heap, buffer);
	if (IS_ERR(vaddr)) {
		perrfn("failed to alloc kernel address of %zu buffer\n",
		       buffer->size);
		return vaddr;
	}
	buffer->vaddr = vaddr;
	buffer->kmap_cnt++;
	return vaddr;
}

void exynos_buffer_kmap_put(struct ion_buffer *buffer)
{
	buffer->kmap_cnt--;
	if (!buffer->kmap_cnt) {
		ion_heap_unmap_kernel(buffer->heap, buffer);
		buffer->vaddr = NULL;
	}
}

static int exynos_dma_buf_begin_cpu_access(struct dma_buf *dmabuf,
					   enum dma_data_direction direction)
{
	struct ion_buffer *buffer = dmabuf->priv;
	void *vaddr;
	struct exynos_iovm_map *iovm_map;
	int ret;

	/*
	 * TODO: Move this elsewhere because we don't always need a vaddr
	 * FIXME: Why do we need a vaddr here?
	 */
	ret = 0;
	mutex_lock(&buffer->lock);
	vaddr = exynos_buffer_kmap_get(buffer);
	if (IS_ERR(vaddr)) {
		ret = PTR_ERR(vaddr);
		goto unlock;
	}

	if (!ion_buffer_cached(buffer))
		goto unlock;

	list_for_each_entry(iovm_map, &buffer->attachments, list) {
		dma_sync_sg_for_cpu(iovm_map->dev, iovm_map->table.sgl,
				    iovm_map->table.nents, direction);
	}

unlock:
	mutex_unlock(&buffer->lock);

	return ret;
}

static int exynos_dma_buf_end_cpu_access(struct dma_buf *dmabuf,
					 enum dma_data_direction direction)
{
	struct ion_buffer *buffer = dmabuf->priv;
	struct exynos_iovm_map *iovm_map;

	mutex_lock(&buffer->lock);

	exynos_buffer_kmap_put(buffer);

	if (!ion_buffer_cached(buffer))
		goto unlock;

	list_for_each_entry(iovm_map, &buffer->attachments, list) {
		dma_sync_sg_for_device(iovm_map->dev, iovm_map->table.sgl,
				       iovm_map->table.nents, direction);
	}
unlock:
	mutex_unlock(&buffer->lock);

	return 0;
}

const struct dma_buf_ops exynos_dma_buf_ops = {
	.attach = exynos_dma_buf_attach,
	.detach = exynos_dma_buf_detach,
	.map_dma_buf = exynos_map_dma_buf,
	.unmap_dma_buf = exynos_unmap_dma_buf,
	.release = exynos_dma_buf_release,
	.begin_cpu_access = exynos_dma_buf_begin_cpu_access,
	.end_cpu_access = exynos_dma_buf_end_cpu_access,
	.mmap = exynos_dma_buf_mmap,
};

static int __init ion_exynos_init(void)
{
	int ret;

	ret = ion_exynos_cma_heap_init();
	if (ret)
		return ret;

	ret = ion_carveout_heap_init();
	if (ret) {
		ion_exynos_cma_heap_exit();
		return ret;
	}

	ion_secure_itmon_init();

	return 0;
}

static void __exit ion_exynos_exit(void)
{
	ion_secure_itmon_exit();
	ion_exynos_cma_heap_exit();
	ion_carveout_heap_exit();
}

module_init(ion_exynos_init);
module_exit(ion_exynos_exit);
MODULE_LICENSE("GPL v2");
