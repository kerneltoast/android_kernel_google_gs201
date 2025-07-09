/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Google LWIS Declarations of Platform-specific DMA Functions
 *
 * Copyright (c) 2018 Google, LLC
 */

#ifndef LWIS_PLATFORM_DMA_H_
#define LWIS_PLATFORM_DMA_H_

#include <linux/dma-buf.h>

#include "lwis_device.h"
#include "lwis_buffer.h"

/* Allocates a DMA buffer of size of PAGE_ALIGN(len) with allocation flags.
 * This returns dma_buf structure or an ERR_PTR on error.
 */
struct dma_buf *lwis_platform_dma_buffer_alloc(size_t len, unsigned int flags);

/*
 * Does the actual platform specific parts of mapping a DMA buffer into the
 * device memory space, returning an IOMMU DMA virtual-address or an ERR_PTR
 * on error.
 */
dma_addr_t lwis_platform_dma_buffer_map(struct lwis_device *lwis_dev,
					struct lwis_enrolled_buffer *lwis_buffer, off_t offset,
					size_t size, int flags);

/*
 * Does the actual platform specific parts of unmapping a DMA buffer from the
 * device memory space.
 */
int lwis_platform_dma_buffer_unmap(struct lwis_device *lwis_dev,
				   struct dma_buf_attachment *attachment, dma_addr_t address);

#endif /* LWIS_PLATFORM_DMA_H_ */
