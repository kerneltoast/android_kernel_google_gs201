// SPDX-License-Identifier: GPL-2.0
/*
 * DMABUF Heap Allocator - secure management
 *
 * Copyright (c) 2021 Samsung Electronics Co., Ltd.
 * Author: <hyesoo.yu@samsung.com> for Samsung
 */

#include <linux/slab.h>
#include <linux/genalloc.h>
#include <linux/kmemleak.h>
#include <linux/dma-mapping.h>
#include <linux/dma-direct.h>
#include <linux/samsung-secure-iova.h>
#include <linux/trusty/trusty.h>

#include "samsung-dma-heap.h"

static int buffer_protect_trusty(struct samsung_dma_buffer *buffer,
				 struct buffer_prot_info *protdesc)
{
	int ret;
	struct device *trusty_dev = buffer->heap->trusty_dev;
	u64 tag = (u64)protdesc->flags | ((u64)(protdesc->dma_addr) << 32);

	if (!trusty_dev) {
		perr("Trusty device missing");
		return -EINVAL;
	}

	ret = trusty_transfer_memory(trusty_dev, &buffer->mem_id,
				     buffer->sg_table.sgl,
				     buffer->sg_table.orig_nents,
				     PAGE_KERNEL, tag, true /* lend */);
	if (ret) {
		perr("trusty_transfer_memory failed: %d", ret);
	}

	return ret;
}

static int buffer_unprotect_trusty(struct samsung_dma_buffer *buffer)
{
	int ret;
	struct device *trusty_dev = buffer->heap->trusty_dev;

	if (!trusty_dev) {
		perr("Trusty device missing");
		return -EINVAL;
	}

	ret = trusty_reclaim_memory(trusty_dev, buffer->mem_id,
				    buffer->sg_table.sgl,
				    buffer->sg_table.orig_nents);
	if (ret) {
		perr("trusty_reclaim_memory failed %d: handle: 0x%llx", ret,
		     buffer->mem_id);
	}

	return ret;
}

void *samsung_dma_buffer_protect(struct samsung_dma_buffer *buffer,
				 unsigned int chunk_size, unsigned int nr_pages,
				 unsigned long paddr)
{
	struct samsung_dma_heap *heap = buffer->heap;
	struct device *dev = dma_heap_get_dev(heap->dma_heap);
	struct buffer_prot_info *protdesc;
	unsigned int protalign = heap->alignment;
	unsigned int array_size = 0;
	unsigned long buf_size;
	unsigned long dma_addr;
	int ret;

	protdesc = kmalloc(sizeof(*protdesc), GFP_KERNEL);
	if (!protdesc)
		return ERR_PTR(-ENOMEM);

	protdesc->chunk_count = nr_pages;
	protdesc->flags = heap->protection_id;
	protdesc->chunk_size = ALIGN(chunk_size, protalign);
	protdesc->bus_address = paddr;

	if (protdesc->chunk_count > 1) {
		void *vaddr = phys_to_virt(paddr);

		array_size = sizeof(paddr) * nr_pages;

		kmemleak_ignore(vaddr);
		dma_map_single(dev, vaddr, array_size, DMA_TO_DEVICE);
	}

	buf_size = protdesc->chunk_count * protdesc->chunk_size;

	dma_addr = secure_iova_alloc(buf_size, max_t(u32, protalign, PAGE_SIZE));
	if (!dma_addr) {
		kfree(protdesc);
		return ERR_PTR(-ENOMEM);
	}

	protdesc->dma_addr = (unsigned int)dma_addr;

	if (dma_heap_flags_dynamic_protected(heap->flags))
		return protdesc;

	ret = buffer_protect_trusty(buffer, protdesc);
	if (ret) {
		if (protdesc->chunk_count > 1)
			dma_unmap_single(dev, phys_to_dma(dev, paddr), array_size, DMA_TO_DEVICE);

		secure_iova_free(dma_addr, buf_size);
		kfree(protdesc);
		return ERR_PTR(ret);
	}

	return protdesc;
}

int samsung_dma_buffer_unprotect(struct samsung_dma_buffer *buffer)
{
	struct buffer_prot_info *protdesc = buffer->priv;
	struct samsung_dma_heap *heap = buffer->heap;
	struct device *dev;
	unsigned long buf_size;
	int ret = 0;

	if (!protdesc || !heap)
		return 0;

	buf_size = protdesc->chunk_count * protdesc->chunk_size;

	dev = dma_heap_get_dev(heap->dma_heap);

	if (protdesc->chunk_count > 1)
		dma_unmap_single(dev, phys_to_dma(dev, protdesc->bus_address),
				 sizeof(unsigned long) * protdesc->chunk_count, DMA_TO_DEVICE);

	if (!dma_heap_flags_dynamic_protected(heap->flags))
		ret = buffer_unprotect_trusty(buffer);

	/*
	 * retain the secure device address if unprotection to its area fails.
	 * It might be unusable forever since we do not know the state of the
	 * secure world before returning error from above.
	 */
	if (!ret)
		secure_iova_free(protdesc->dma_addr, buf_size);

	kfree(protdesc);

	return ret;
}
