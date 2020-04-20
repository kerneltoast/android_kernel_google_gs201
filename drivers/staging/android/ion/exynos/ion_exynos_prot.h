/* SPDX-License-Identifier: GPL-2.0 */
/*
 * ION protected buffer support
 *
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 */

#ifndef _ION_EXYNOS_PROT_H
#define _ION_EXYNOS_PROT_H

#include <linux/of.h>

/*
 * allocation flags - the lower 16 bits are used by core ion, the upper 16
 * bits are reserved for use by the heaps themselves.
 */
#define ION_EXYNOS_FLAG_PROTECTED BIT(16)

static inline bool ion_buffer_protected(struct ion_buffer *buffer)
{
	return !!(buffer->flags & ION_EXYNOS_FLAG_PROTECTED);
}

static inline bool is_dev_samsung_tzmp(struct device *dev)
{
	return of_property_read_bool(dev->of_node, "samsung,tzmp");
}

/*
 * struct buffer_prot_info - buffer protection information
 * @chunk_count: number of physically contiguous memory chunks to protect
 *               each chunk should has the same size.
 * @dma_addr:    device virtual address for protected memory access
 * @flags:       protection flags but actually, protection_id
 * @chunk_size:  length in bytes of each chunk.
 * @bus_address: if @chunk_count is 1, this is the physical address the chunk.
 *               if @chunk_count > 1, this is the physical address of unsigned
 *               long array of @chunk_count elements that contains the physical
 *               address of each chunk.
 */
struct ion_buffer_prot_info {
	unsigned int chunk_count;
	unsigned int dma_addr;
	unsigned int flags;
	unsigned int chunk_size;
	unsigned long bus_address;
};
#endif
