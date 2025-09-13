/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 Google LLC
 */

#include <samsung/samsung-dma-heap.h>
#include <linux/genalloc.h>

typedef void(ion_physical_heap_allocate_callback)(struct samsung_dma_buffer *buffer,
						  void *ctx);
typedef void(ion_physical_heap_free_callback)(struct samsung_dma_buffer *buffer,
					      void *ctx);

struct dma_heap *ion_physical_heap_create(phys_addr_t base, size_t size,
					  size_t align, const char *name,
					  ion_physical_heap_allocate_callback alloc_cb,
					  ion_physical_heap_free_callback free_cb,
					  void *ctx);

struct ion_physical_heap {
	struct gen_pool *pool;
	phys_addr_t base;
	size_t size;

	ion_physical_heap_allocate_callback *allocate_cb;
	void *allocate_ctx;

	ion_physical_heap_free_callback *free_cb;
	void *free_ctx;
};
