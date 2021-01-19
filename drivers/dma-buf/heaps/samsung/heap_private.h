/* SPDX-License-Identifier: GPL-2.0
 *
 * DMABUF Heap Allocator - Internal header
 *
 * Copyright (C) 2021 Samsung Electronics Co., Ltd.
 * Author: <hyesoo.yu@samsung.com> for Samsung
 */

#ifndef _HEAP_PRIVATE_H
#define _HEAP_PRIVATE_H

#include <linux/dma-buf.h>
#include <linux/dma-heap.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/highmem.h>
#include <linux/module.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/iommu.h>
#include <linux/device.h>

struct samsung_dma_buffer {
	struct samsung_dma_heap *heap;
	struct list_head attachments;
	/* Manage buffer resource of attachments and vaddr, vmap_cnt */
	struct mutex lock;
	unsigned long len;
	struct sg_table sg_table;
	void *vaddr;
	void *priv;
	unsigned long flags;
	int vmap_cnt;
};

struct samsung_dma_heap {
	struct dma_heap *dma_heap;
	void (*release)(struct samsung_dma_buffer *buffer);
	void *priv;
	const char *name;
	unsigned long flags;
	unsigned int alignment;
	unsigned int protection_id;
};

struct samsung_map_attachment {
	struct device *dev;
	struct sg_table *table;
	struct list_head list;
	unsigned long flags;
	bool mapped;
};

extern const struct dma_buf_ops samsung_dma_buf_ops;

#define DEFINE_SAMSUNG_DMA_BUF_EXPORT_INFO(name, heap_name)	\
	struct dma_buf_export_info name = { .exp_name = heap_name, \
					 .owner = THIS_MODULE }

void heap_cache_flush(struct samsung_dma_buffer *buffer);
void heap_page_clean(struct page *pages, unsigned long size);
struct samsung_dma_buffer *samsung_dma_buffer_alloc(struct samsung_dma_heap *samsung_dma_heap,
						    unsigned long size, unsigned int nents);
void samsung_dma_buffer_free(struct samsung_dma_buffer *buffer);
int samsung_heap_add(struct device *dev, void *priv,
		     void (*release)(struct samsung_dma_buffer *buffer),
		     const struct dma_heap_ops *ops);
struct dma_buf *samsung_export_dmabuf(struct samsung_dma_buffer *buffer, unsigned long fd_flags);

#define DMA_HEAP_VIDEO_PADDING (512)
#define dma_heap_add_video_padding(len) (PAGE_ALIGN((len) + DMA_HEAP_VIDEO_PADDING))

#define DMA_HEAP_FLAG_UNCACHED  BIT(0)
#define DMA_HEAP_FLAG_PROTECTED BIT(1)
#define DMA_HEAP_FLAG_VIDEO_ALIGNED BIT(2)

static inline bool dma_heap_flags_uncached(unsigned long flags)
{
	return !!(flags & DMA_HEAP_FLAG_UNCACHED);
}

static inline bool dma_heap_flags_protected(unsigned long flags)
{
	return !!(flags & DMA_HEAP_FLAG_PROTECTED);
}

static inline bool dma_heap_skip_cache_ops(unsigned long flags)
{
	return dma_heap_flags_protected(flags) || dma_heap_flags_uncached(flags);
}

static inline bool dma_heap_flags_video_aligned(unsigned long flags)
{
	return !!(flags & DMA_HEAP_FLAG_VIDEO_ALIGNED);
}

/*
 * Use pre-mapped protected device virtual address instead of dma-mapping.
 */
static inline bool dma_heap_tzmp_buffer(struct samsung_map_attachment *a)
{
	return dma_heap_flags_protected(a->flags) && !!dev_iommu_fwspec_get(a->dev);
}

/*
 * struct buffer_prot_info - buffer protection information
 * @chunk_count: number of physically contiguous memory chunks to protect
 *               each chunk should has the same size.
 * @dma_addr:    device virtual address for protected memory access
 * @flags:       protection flags but actually, protectid
 * @chunk_size:  length in bytes of each chunk.
 * @bus_address: if @chunk_count is 1, this is the physical address the chunk.
 *               if @chunk_count > 1, this is the physical address of unsigned
 *               long array of @chunk_count elements that contains the physical
 *               address of each chunk.
 */
struct buffer_prot_info {
	unsigned int chunk_count;
	unsigned int dma_addr;
	unsigned int flags;
	unsigned int chunk_size;
	unsigned long bus_address;
};

#if IS_ENABLED(CONFIG_EXYNOS_CONTENT_PATH_PROTECTION)
int secure_iova_pool_create(void);
void secure_iova_pool_destroy(void);
void *samsung_dma_buffer_protect(struct samsung_dma_heap *heap,
				 unsigned int size, unsigned long phys);
int samsung_dma_buffer_unprotect(void *priv, struct device *dev);
#else
static inline int secure_iova_pool_create(void)
{
	return 0;
}

#define secure_iova_pool_destroy() do { } while (0)

static inline void *samsung_dma_buffer_protect(struct samsung_dma_heap *heap,
					       unsigned int size, unsigned long phys)
{
	return NULL;
}

static inline int samsung_dma_buffer_unprotect(void *priv, struct device *dev)
{
	return 0;
}
#endif

#if defined(CONFIG_DMABUF_HEAPS_SAMSUNG_SYSTEM)
int __init system_dma_heap_init(void);
void system_dma_heap_exit(void);
#else
static inline int __init system_dma_heap_init(void)
{
	return 0;
}

#define system_dma_heap_exit() do { } while (0)
#endif

#if defined(CONFIG_DMABUF_HEAPS_SAMSUNG_CMA)
int __init cma_dma_heap_init(void);
void cma_dma_heap_exit(void);
#else
static inline int __init cma_dma_heap_init(void)
{
	return 0;
}

#define cma_dma_heap_exit() do { } while (0)
#endif

#if defined(CONFIG_DMABUF_HEAPS_SAMSUNG_CARVEOUT)
int __init carveout_dma_heap_init(void);
void carveout_dma_heap_exit(void);
#else
static inline int __init carveout_dma_heap_init(void)
{
	return 0;
}

#define carveout_dma_heap_exit() do { } while (0)
#endif

#define DMAHEAP_PREFIX "[Exynos][DMA-HEAP] "
#define perr(format, arg...) \
	pr_err(DMAHEAP_PREFIX format "\n", ##arg)

#define perrfn(format, arg...) \
	pr_err(DMAHEAP_PREFIX "%s: " format "\n", __func__, ##arg)

#define perrdev(dev, format, arg...) \
	dev_err(dev, DMAHEAP_PREFIX format "\n", ##arg)

#define perrfndev(dev, format, arg...) \
	dev_err(dev, DMAHEAP_PREFIX "%s: " format "\n", __func__, ##arg)

#endif /* _HEAP_PRIVATE_H */
