// SPDX-License-Identifier: GPL-2.0
/*
 * DMABUF Heap Allocator - Common implementation
 *
 * Copyright (C) 2021 Samsung Electronics Co., Ltd.
 * Author: <hyesoo.yu@samsung.com> for Samsung
 */

#include <linux/dma-buf.h>
#include <linux/dma-heap.h>
#include <linux/err.h>
#include <linux/highmem.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/of.h>

#include "heap_private.h"

void heap_cache_flush(struct samsung_dma_buffer *buffer)
{
	struct device *dev = dma_heap_get_dev(buffer->heap->dma_heap);

	if (!dma_heap_flags_uncached(buffer->flags))
		return;

	dma_map_sgtable(dev, &buffer->sg_table, DMA_TO_DEVICE, 0);
	dma_unmap_sgtable(dev, &buffer->sg_table, DMA_FROM_DEVICE, 0);
}

/*
 * It should be called by physically contiguous buffer.
 */
void heap_page_clean(struct page *pages, unsigned long size)
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

struct samsung_dma_buffer *samsung_dma_buffer_alloc(struct samsung_dma_heap *samsung_dma_heap,
						    unsigned long size, unsigned int nents)
{
	struct samsung_dma_buffer *buffer;

	buffer = kzalloc(sizeof(*buffer), GFP_KERNEL);
	if (!buffer)
		return ERR_PTR(-ENOMEM);

	if (sg_alloc_table(&buffer->sg_table, nents, GFP_KERNEL)) {
		kfree(buffer);
		return ERR_PTR(-ENOMEM);
	}

	INIT_LIST_HEAD(&buffer->attachments);
	mutex_init(&buffer->lock);
	buffer->heap = samsung_dma_heap;
	buffer->len = size;
	buffer->flags = samsung_dma_heap->flags;

	return buffer;
}

void samsung_dma_buffer_free(struct samsung_dma_buffer *buffer)
{
	sg_free_table(&buffer->sg_table);
	kfree(buffer);
}

static const char *samsung_add_heap_name(unsigned long flags)
{
	if (flags & DMA_HEAP_FLAG_UNCACHED)
		return "-uncached";
	return "";
}

/*
 * registers cachable heap and uncachable heap both.
 */
static const unsigned int samsung_heap_type[] = {
	0,
	DMA_HEAP_FLAG_UNCACHED,
};

#define NUM_HEAP_TYPE ARRAY_SIZE(samsung_heap_type)

static struct samsung_dma_heap *__samsung_heap_add(struct device *dev, void *priv,
						   void (*release)(struct samsung_dma_buffer *),
						   const struct dma_heap_ops *ops,
						   unsigned int flags)
{
	struct samsung_dma_heap *heap;
	unsigned int alignment = PAGE_SIZE, order;
	struct dma_heap_export_info exp_info;
	const char *name;
	char *heap_name;

	if (of_property_read_string(dev->of_node, "dma-heap,name", &name)) {
		perrfn("The heap should define name on device node");
		return ERR_PTR(-EINVAL);
	}

	of_property_read_u32(dev->of_node, "dma-heap,alignment", &alignment);
	order = min_t(unsigned int, get_order(alignment), MAX_ORDER);

	heap = devm_kzalloc(dev, sizeof(*heap), GFP_KERNEL);
	if (!heap)
		return ERR_PTR(-ENOMEM);
	heap->flags = flags;

	heap_name = devm_kasprintf(dev, GFP_KERNEL, "%s%s", name, samsung_add_heap_name(flags));
	if (!heap_name)
		return ERR_PTR(-ENOMEM);

	heap->alignment = 1 << (order + PAGE_SHIFT);
	heap->release = release;
	heap->priv = priv;
	heap->name = heap_name;

	exp_info.name = heap_name;
	exp_info.ops = ops;
	exp_info.priv = heap;

	heap->dma_heap = dma_heap_add(&exp_info);
	if (IS_ERR(heap->dma_heap))
		return heap;

	pr_info("Registered %s dma-heap successfully\n", heap_name);

	dma_coerce_mask_and_coherent(dma_heap_get_dev(heap->dma_heap), DMA_BIT_MASK(36));

	return heap;
}

int samsung_heap_add(struct device *dev, void *priv,
		     void (*release)(struct samsung_dma_buffer *buffer),
		     const struct dma_heap_ops *ops)
{
	struct samsung_dma_heap *heap[NUM_HEAP_TYPE];
	int i, ret;

	for (i = 0; i < NUM_HEAP_TYPE; i++) {
		heap[i] = __samsung_heap_add(dev, priv, release, ops, samsung_heap_type[i]);
		if (IS_ERR(heap[i])) {
			ret = PTR_ERR(heap[i]);
			goto heap_put;
		}
	}

	return 0;
heap_put:
	while (i-- > 0)
		dma_heap_put(heap[i]->dma_heap);

	return ret;
}

struct dma_buf *samsung_export_dmabuf(struct samsung_dma_buffer *buffer, unsigned long fd_flags)
{
	DEFINE_DMA_BUF_EXPORT_INFO(exp_info);

	exp_info.ops = &samsung_dma_buf_ops;
	exp_info.size = buffer->len;
	exp_info.flags = fd_flags;
	exp_info.priv = buffer;

	return dma_buf_export(&exp_info);
}

static int __init samsung_dma_heap_init(void)
{
	int ret;

	ret = cma_dma_heap_init();
	if (ret)
		return ret;

	ret = carveout_dma_heap_init();
	if (ret)
		goto err_carveout;

	ret = system_dma_heap_init();
	if (ret)
		goto err_system;

	return 0;
err_system:
	carveout_dma_heap_exit();
err_carveout:
	cma_dma_heap_exit();

	return ret;
}

static void __exit samsung_dma_heap_exit(void)
{
	system_dma_heap_exit();
	carveout_dma_heap_exit();
	cma_dma_heap_exit();
}

module_init(samsung_dma_heap_init);
module_exit(samsung_dma_heap_exit);
MODULE_DESCRIPTION("DMA-BUF Samsung Heap");
MODULE_LICENSE("GPL v2");
