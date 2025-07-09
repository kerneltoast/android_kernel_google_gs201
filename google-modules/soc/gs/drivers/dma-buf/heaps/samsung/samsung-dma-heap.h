/* SPDX-License-Identifier: GPL-2.0
 *
 * DMABUF Heap Allocator - Internal header
 *
 * Copyright (C) 2021 Samsung Electronics Co., Ltd.
 * Author: <hyesoo.yu@samsung.com> for Samsung
 */

#ifndef _SAMSUNG_DMA_HEAP_H
#define _SAMSUNG_DMA_HEAP_H

#include <linux/dma-buf.h>
#include <linux/dma-heap.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/highmem.h>
#include <linux/module.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/trusty/trusty.h>
#include <linux/platform_device.h>
#include <linux/iommu.h>
#include <linux/device.h>

/* This header is in ACK under drivers/dma-buf/heaps. */
#include <heaps/deferred-free-helper.h>

/* the number of pages that allocated from heaps and currently used */
static atomic64_t inuse_pages = ATOMIC_INIT(0);

static inline void dma_heap_inc_inuse(unsigned long pages)
{
	atomic64_add(pages, &inuse_pages);
}

static inline void dma_heap_dec_inuse(unsigned long pages)
{
	WARN_ON_ONCE(pages > atomic64_read(&inuse_pages));

	atomic64_sub(pages, &inuse_pages);
}

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
	struct deferred_freelist_item deferred_free;
	unsigned long ino;
	trusty_shared_mem_id_t mem_id;
};

struct samsung_dma_heap {
	struct dma_heap *dma_heap;
	void (*release)(struct samsung_dma_buffer *buffer);
	void *priv;
	const char *name;
	unsigned long flags;
	unsigned int alignment;
	unsigned int protection_id;
	struct device *trusty_dev;
};

extern const struct dma_buf_ops samsung_dma_buf_ops;

#define DEFINE_SAMSUNG_DMA_BUF_EXPORT_INFO(name, heap_name)	\
	struct dma_buf_export_info name = { .exp_name = heap_name, \
					 .owner = THIS_MODULE }

void heap_sgtable_pages_clean(struct sg_table *sgt);
void heap_cache_flush(struct samsung_dma_buffer *buffer);
void heap_page_clean(struct page *pages, unsigned long size);
struct samsung_dma_buffer *samsung_dma_buffer_alloc(struct samsung_dma_heap *samsung_dma_heap,
						    unsigned long size, unsigned int nents);
void samsung_dma_buffer_free(struct samsung_dma_buffer *buffer);
int samsung_heap_add(struct device *dev, void *priv,
		     void (*release)(struct samsung_dma_buffer *buffer),
		     const struct dma_heap_ops *ops);
struct dma_buf *samsung_export_dmabuf(struct samsung_dma_buffer *buffer, unsigned long fd_flags);
void samsung_track_buffer_destroyed(struct samsung_dma_buffer *buffer);

#define DMA_HEAP_VIDEO_PADDING (512)
#define dma_heap_add_video_padding(len) (PAGE_ALIGN((len) + DMA_HEAP_VIDEO_PADDING))

#define DMA_HEAP_FLAG_UNCACHED  BIT(0)
#define DMA_HEAP_FLAG_PROTECTED BIT(1)
#define DMA_HEAP_FLAG_VIDEO_ALIGNED BIT(2)
#define DMA_HEAP_FLAG_DYNAMIC_PROTECTED BIT(3)

static inline bool dma_heap_flags_uncached(unsigned long flags)
{
	return !!(flags & DMA_HEAP_FLAG_UNCACHED);
}

static inline bool dma_heap_flags_static_protected(unsigned long flags)
{
	return !!(flags & DMA_HEAP_FLAG_PROTECTED);
}

static inline bool dma_heap_flags_dynamic_protected(unsigned long flags)
{
	return !!(flags & DMA_HEAP_FLAG_DYNAMIC_PROTECTED);
}

static inline bool dma_heap_flags_protected(unsigned long flags)
{
	return dma_heap_flags_static_protected(flags) ||
		dma_heap_flags_dynamic_protected(flags);
}

static inline bool dma_heap_skip_cache_ops(unsigned long flags)
{
	return dma_heap_flags_static_protected(flags) || dma_heap_flags_uncached(flags);
}

static inline bool dma_heap_flags_video_aligned(unsigned long flags)
{
	return !!(flags & DMA_HEAP_FLAG_VIDEO_ALIGNED);
}

/*
 * Use pre-mapped protected device virtual address instead of dma-mapping.
 */
static inline bool dma_heap_tzmp_buffer(struct device *dev, unsigned long flags)
{
	return dma_heap_flags_static_protected(flags) && !!dev_iommu_fwspec_get(dev);
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
void *samsung_dma_buffer_protect(struct samsung_dma_buffer *buffer,
				 unsigned int chunk_size, unsigned int nr_pages,
				 unsigned long paddr);
int samsung_dma_buffer_unprotect(struct samsung_dma_buffer *buffer);
#else
static inline void *samsung_dma_buffer_protect(struct samsung_dma_buffer *buffer,
					       unsigned int chunk_size,
					       unsigned int nr_pages,
					       unsigned long paddr)
{
	return NULL;
}

static inline int samsung_dma_buffer_unprotect(struct samsung_dma_buffer *buffer)
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

#if defined(CONFIG_DMABUF_HEAPS_GOOGLE_GCMA)
int __init gcma_dma_heap_init(void);
void gcma_dma_heap_exit(void);
unsigned long dma_heap_gcma_inuse_pages(void);
#else
static inline int __init gcma_dma_heap_init(void)
{
	return 0;
}

#define gcma_dma_heap_exit() do { } while (0)

static inline unsigned long dma_heap_gcma_inuse_pages(void)
{
	return 0;
}
#endif

#if defined(CONFIG_DMABUF_HEAPS_SAMSUNG_CHUNK)
int __init chunk_dma_heap_init(void);
void chunk_dma_heap_exit(void);
#else
static inline int __init chunk_dma_heap_init(void)
{
	return 0;
}

#define chunk_dma_heap_exit() do { } while (0)
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

#endif /* _SAMSUNG_DMA_HEAP_H */
