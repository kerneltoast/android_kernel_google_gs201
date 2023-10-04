/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 Google LLC
 */

#include <linux/samsung-dma-heap.h>

typedef void(ion_physical_heap_allocate_callback)(struct samsung_dma_buffer *buffer,
						  void *ctx);
typedef void(ion_physical_heap_free_callback)(struct samsung_dma_buffer *buffer,
					      void *ctx);

struct dma_heap *ion_physical_heap_create(phys_addr_t base, size_t size,
					  size_t align, const char *name,
					  ion_physical_heap_allocate_callback alloc_cb,
					  ion_physical_heap_free_callback free_cb,
					  void *ctx);
