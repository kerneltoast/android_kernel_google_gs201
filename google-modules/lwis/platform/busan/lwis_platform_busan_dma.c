// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google LWIS Busan Platform-Specific DMA Functions
 *
 * Copyright (c) 2021 Google, LLC
 */

#include <linux/slab.h>

#include <linux/dma-buf.h>
#include <linux/dma-heap.h>
#include "lwis_commands.h"
#include "lwis_platform.h"
#include "lwis_platform_dma.h"
struct dma_buf *lwis_platform_dma_buffer_alloc(size_t len, unsigned int flags)
{
	const char *heap_name;
	struct dma_heap *heap;
	struct dma_buf *dmabuf;

	if ((flags & LWIS_DMA_BUFFER_SECURE) && IS_ENABLED(CONFIG_EXYNOS_CONTENT_PATH_PROTECTION))
		heap_name = "farawimg-secure";
	else if (flags & LWIS_DMA_BUFFER_CACHED)
		heap_name = "system";
	else
		heap_name = "system-uncached";

	heap = dma_heap_find(heap_name);
	if (!heap) {
		pr_err("Could not find %s DMA-BUF heap\n", heap_name);
		return NULL;
	}

	dmabuf = dma_heap_buffer_alloc(heap, len, O_RDWR, 0);
	if (IS_ERR_OR_NULL(dmabuf)) {
		pr_err("DMA-BUF heap failed to alloc %#zx bytes. Error code %lu\n", len,
		       PTR_ERR(dmabuf));
		dmabuf = NULL;
	}

	dma_heap_put(heap);

	return dmabuf;
}

dma_addr_t lwis_platform_dma_buffer_map(struct lwis_device *lwis_dev,
					struct lwis_enrolled_buffer *lwis_buffer, off_t offset,
					size_t size, int flags)
{
	return sg_dma_address(lwis_buffer->sg_table->sgl);
}

/*
 * We don't ever do dma_buf_vmap before. Instead, use the upstream dma-buf
 * interface to map ION buffers, so we don't need to do dma_buf_vunmap.
 * Keep this function by default return 0
 */
int lwis_platform_dma_buffer_unmap(struct lwis_device *lwis_dev,
				   struct dma_buf_attachment *attachment, dma_addr_t address)
{
	return 0;
}
