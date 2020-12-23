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
#include <linux/device.h>

struct samsung_dma_buffer {
	struct samsung_dma_heap *heap;
	struct list_head attachments;
	/* Manage buffer resource of attachments and vaddr, vmap_cnt */
	struct mutex lock;
	unsigned long len;
	struct sg_table sg_table;
	void *vaddr;
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
};

struct samsung_map_attachment {
	struct device *dev;
	struct sg_table *table;
	struct list_head list;
	unsigned long flags;
	bool mapped;
};

extern const struct dma_buf_ops samsung_dma_buf_ops;

void heap_cache_flush(struct samsung_dma_buffer *buffer);
void heap_page_clean(struct page *pages, unsigned long size);
struct samsung_dma_buffer *samsung_dma_buffer_alloc(struct samsung_dma_heap *samsung_dma_heap,
						    unsigned long size, unsigned int nents);
void samsung_dma_buffer_free(struct samsung_dma_buffer *buffer);
int samsung_heap_add(struct device *dev, void *priv,
		     void (*release)(struct samsung_dma_buffer *buffer),
		     const struct dma_heap_ops *ops);
struct dma_buf *samsung_export_dmabuf(struct samsung_dma_buffer *buffer, unsigned long fd_flags);

#define DMA_HEAP_FLAG_UNCACHED BIT(0)

static inline bool dma_heap_flags_uncached(unsigned long flags)
{
	return !!(flags & DMA_HEAP_FLAG_UNCACHED);
}

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
