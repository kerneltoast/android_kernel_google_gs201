// SPDX-License-Identifier: GPL-2.0
/*
 * DMABUF Heap Allocator - secure management
 *
 * Copyright (c) 2021 Samsung Electronics Co., Ltd.
 * Author: <hyesoo.yu@samsung.com> for Samsung
 */

#include <linux/slab.h>
#include <linux/genalloc.h>
#include <linux/smc.h>
#include <linux/kmemleak.h>
#include <linux/dma-mapping.h>
#include <linux/arm-smccc.h>
#include <linux/dma-direct.h>
#include <linux/samsung-secure-iova.h>

#include "heap_private.h"

#define SMC_DRM_PPMP_PROT		(0x82002110)
#define SMC_DRM_PPMP_UNPROT		(0x82002111)

static inline unsigned long ppmp_smc(unsigned long cmd, unsigned long arg0,
				     unsigned long arg1, unsigned long arg2)
{
	struct arm_smccc_res res;

	arm_smccc_smc(cmd, arg0, arg1, arg2, 0, 0, 0, 0, &res);
	return (unsigned long)res.a0;
}

static int buffer_protect_smc(struct device *dev, struct buffer_prot_info *protdesc,
			      unsigned int protalign)
{
	unsigned long size = protdesc->chunk_count * protdesc->chunk_size;
	unsigned long drmret = 0, dma_addr = 0;
	unsigned long ret;

	dma_addr = secure_iova_alloc(size, max_t(u32, protalign, PAGE_SIZE));
	if (!dma_addr)
		return -ENOMEM;

	protdesc->dma_addr = (unsigned int)dma_addr;

	dma_map_single(dev, protdesc, sizeof(*protdesc), DMA_TO_DEVICE);

	ret = ppmp_smc(SMC_DRM_PPMP_PROT, virt_to_phys(protdesc), 0, 0);
	if (ret) {
		secure_iova_free(dma_addr, size);
		perr("CMD %#x (err=%#lx,va=%#x,len=%#lx,cnt=%u,flg=%u)",
		     SMC_DRM_PPMP_PROT, drmret, protdesc->dma_addr, size,
		     protdesc->chunk_count, protdesc->flags);
		dma_unmap_single(dev, protdesc->dma_addr, sizeof(*protdesc),
				 DMA_TO_DEVICE);
		return -EACCES;
	}

	return 0;
}

static int buffer_unprotect_smc(struct device *dev,
				struct buffer_prot_info *protdesc)
{
	unsigned long size = protdesc->chunk_count * protdesc->chunk_size;
	unsigned long ret;

	ret = ppmp_smc(SMC_DRM_PPMP_UNPROT, virt_to_phys(protdesc), 0, 0);

	if (ret) {
		perr("CMD %#x (err=%#lx,va=%#x,len=%#lx,cnt=%u,flg=%u)",
		     SMC_DRM_PPMP_UNPROT, ret, protdesc->dma_addr,
		     size, protdesc->chunk_count, protdesc->flags);
		return -EACCES;
	}

	dma_unmap_single(dev, protdesc->dma_addr, sizeof(*protdesc),
			 DMA_TO_DEVICE);

	/*
	 * retain the secure device address if unprotection to its area fails.
	 * It might be unusable forever since we do not know the state of the
	 * secure world before returning error from ppmp_smc() above.
	 */
	secure_iova_free(protdesc->dma_addr, size);

	return 0;
}

void *samsung_dma_buffer_protect(struct samsung_dma_heap *heap, unsigned int size,
				 unsigned long phys)
{
	struct buffer_prot_info *protdesc;
	unsigned int protalign = heap->alignment;
	int ret;

	protdesc = kmalloc(sizeof(*protdesc), GFP_KERNEL);
	if (!protdesc)
		return ERR_PTR(-ENOMEM);

	protdesc->chunk_count = 1;
	protdesc->flags = heap->protection_id;
	protdesc->chunk_size = ALIGN(size, protalign);
	protdesc->bus_address = phys;

	ret = buffer_protect_smc(dma_heap_get_dev(heap->dma_heap), protdesc, protalign);
	if (ret) {
		perr("protection failure (id%u,len%u,base%#lx,align%#x",
		     heap->protection_id, size, phys, protalign);
		kfree(protdesc);
		return ERR_PTR(ret);
	}

	return protdesc;
}

int samsung_dma_buffer_unprotect(void *priv, struct device *dev)
{
	struct buffer_prot_info *protdesc = priv;
	int ret = 0;

	if (priv) {
		ret = buffer_unprotect_smc(dev, protdesc);
		kfree(protdesc);
	}

	return ret;
}
