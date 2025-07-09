// SPDX-License-Identifier: GPL-2.0-only
/*
 * GXP DMA implemented via IOMMU.
 *
 * Copyright (C) 2021-2024 Google LLC
 */

#include <linux/bits.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/iommu.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>

#include <gcip/gcip-iommu.h>

#include "gxp-config.h"
#include "gxp-dma.h"
#include "gxp-mailbox.h"
#include "gxp-mapping.h"
#include "gxp-pm.h"
#include "gxp.h"
#include "mobile-soc.h"

#include <trace/events/gxp.h>

struct gxp_dma_iommu_manager {
	struct gxp_dma_manager dma_mgr;
	struct gcip_iommu_domain *default_domain;
};

/* Fault handler */

static int iommu_fault_handler(struct iommu_fault *fault, void *token)
{
	struct device *dev = token;

	switch (fault->type) {
	case IOMMU_FAULT_DMA_UNRECOV:
		dev_err(dev, "Unrecoverable IOMMU fault!\n");
		dev_err(dev, "reason = %08X\n", fault->event.reason);
		dev_err(dev, "flags = %08X\n", fault->event.flags);
		dev_err(dev, "pasid = %08X\n", fault->event.pasid);
		dev_err(dev, "perm = %08X\n", fault->event.perm);
		dev_err(dev, "addr = %llX\n", fault->event.addr);
		dev_err(dev, "fetch_addr = %llX\n", fault->event.fetch_addr);
		break;
	case IOMMU_FAULT_PAGE_REQ:
		dev_err(dev, "IOMMU page request fault!\n");
		dev_err(dev, "flags = %08X\n", fault->prm.flags);
		dev_err(dev, "pasid = %08X\n", fault->prm.pasid);
		dev_err(dev, "grpid = %08X\n", fault->prm.grpid);
		dev_err(dev, "perm = %08X\n", fault->prm.perm);
		dev_err(dev, "addr = %llX\n", fault->prm.addr);
		break;
	default:
		dev_err(dev, "Unexpected IOMMU fault type (%d)\n", fault->type);
	}

	/* Tells the IOMMU driver to carry on. */
	return -EAGAIN;
}

#if GXP_HAS_LAP

/* No need to map CSRs when local access path exists. */

#define gxp_map_csrs(...) 0
#define gxp_unmap_csrs(...)

#else /* !GXP_HAS_LAP */

static int gxp_map_csrs(struct gxp_dev *gxp, struct gcip_iommu_domain *gdomain,
			struct gxp_mapped_resource *regs)
{
	return gcip_iommu_map(gdomain, GXP_IOVA_AURORA_TOP, gxp->regs.paddr, gxp->regs.size,
				 GCIP_MAP_FLAGS_DMA_RW);
}

static void gxp_unmap_csrs(struct gxp_dev *gxp, struct gcip_iommu_domain *gdomain,
			   struct gxp_mapped_resource *regs)
{
	gcip_iommu_unmap(gdomain, GXP_IOVA_AURORA_TOP, gxp->regs.size);
}

#endif /* GXP_HAS_LAP */

/* gxp-dma.h Interface */

struct gcip_iommu_domain *gxp_iommu_get_domain_for_dev(struct gxp_dev *gxp)
{
	struct gcip_iommu_domain *gdomain = gxp->default_domain;

	if (IS_ERR_OR_NULL(gdomain)) {
		gdomain = gcip_iommu_get_domain_for_dev(gxp->dev);
		if (IS_ERR_OR_NULL(gdomain))
			return gdomain;
		gxp->default_domain = gdomain;
	}

	return gdomain;
}

int gxp_dma_init(struct gxp_dev *gxp)
{
	struct gxp_dma_iommu_manager *mgr;
	int ret;

	/* Remove the limit of DMA ranges. */
	ret = dma_set_mask_and_coherent(gxp->dev, DMA_BIT_MASK(64));
	if (ret) {
		dev_err(gxp->dev, "Failed to set DMA mask\n");
		return ret;
	}

	mgr = devm_kzalloc(gxp->dev, sizeof(*mgr), GFP_KERNEL);
	if (!mgr)
		return -ENOMEM;

	mgr->default_domain = gxp_iommu_get_domain_for_dev(gxp);
	if (IS_ERR(mgr->default_domain)) {
		dev_err(gxp->dev, "Failed to find default IOMMU domain\n");
		return PTR_ERR(mgr->default_domain);
	}

	if (iommu_register_device_fault_handler(gxp->dev, iommu_fault_handler, gxp->dev)) {
		dev_err(gxp->dev, "Failed to register iommu fault handler\n");
		return -EIO;
	}

	gxp->dma_mgr = &(mgr->dma_mgr);

	return 0;
}

void gxp_dma_exit(struct gxp_dev *gxp)
{
	if (iommu_unregister_device_fault_handler(gxp->dev))
		dev_err(gxp->dev,
			"Failed to unregister IOMMU fault handler\n");
}

#define EXT_TPU_MBX_SIZE 0x2000

void gxp_dma_init_default_resources(struct gxp_dev *gxp)
{
	unsigned int core;
	int i;

	for (i = 0; i < GXP_NUM_MAILBOXES; i++)
		gxp->mbx[i].daddr = GXP_IOVA_MAILBOX(i);
	for (core = 0; core < GXP_NUM_CORES; core++)
		gxp->fwbufs[core].daddr = GXP_IOVA_FIRMWARE(core);
}

int gxp_dma_domain_attach_device(struct gxp_dev *gxp, struct gcip_iommu_domain *gdomain,
				 uint core_list)
{
	int pasid;

	if (gdomain == gxp_iommu_get_domain_for_dev(gxp))
		return 0;

	pasid = gcip_iommu_domain_pool_attach_domain(gxp->domain_pool, gdomain);
	if (pasid < 0) {
		dev_err(gxp->dev, "Attach IOMMU domain failed: %d", pasid);
		return pasid;
	}

	gxp_soc_activate_context(gxp, gdomain, core_list);

	return 0;
}

void gxp_dma_domain_detach_device(struct gxp_dev *gxp, struct gcip_iommu_domain *gdomain,
				  uint core_list)
{
	if (gdomain == gxp_iommu_get_domain_for_dev(gxp))
		return;

	gxp_soc_deactivate_context(gxp, gdomain, core_list);
	gcip_iommu_domain_pool_detach_domain(gxp->domain_pool, gdomain);
}

int gxp_dma_map_core_resources(struct gxp_dev *gxp, struct gcip_iommu_domain *gdomain,
			       uint core_list, u8 slice_index)
{
	int ret;
	uint i;

	if (!gxp_is_direct_mode(gxp))
		return 0;

	ret = gxp_map_csrs(gxp, gdomain, &gxp->regs);
	if (ret)
		goto err;

	for (i = 0; i < GXP_NUM_CORES; i++) {
		if (!(BIT(i) & core_list))
			continue;
		ret = gcip_iommu_map(gdomain, gxp->mbx[i].daddr,
				     gxp->mbx[i].paddr + MAILBOX_DEVICE_INTERFACE_OFFSET,
				     gxp->mbx[i].size, GCIP_MAP_FLAGS_DMA_RW);
		if (ret)
			goto err;
	}
	/* Only map the TPU mailboxes if they were found on probe */
	if (gxp->tpu_dev.mbx_paddr) {
		for (i = 0; i < GXP_NUM_CORES; i++) {
			if (!(BIT(i) & core_list))
				continue;
			ret = gcip_iommu_map(gdomain, GXP_IOVA_EXT_TPU_MBX + i * EXT_TPU_MBX_SIZE,
					     gxp->tpu_dev.mbx_paddr + i * EXT_TPU_MBX_SIZE,
					     EXT_TPU_MBX_SIZE, GCIP_MAP_FLAGS_DMA_RW);
			if (ret)
				goto err;
		}
	}
	return ret;

err:
	/*
	 * Attempt to unmap all resources.
	 * Any resource that hadn't been mapped yet will cause `iommu_unmap()`
	 * to return immediately, so its safe to try to unmap everything.
	 */
	gxp_dma_unmap_core_resources(gxp, gdomain, core_list);
	return ret;
}

void gxp_dma_unmap_core_resources(struct gxp_dev *gxp, struct gcip_iommu_domain *gdomain,
				  uint core_list)
{
	uint i;

	if (!gxp_is_direct_mode(gxp))
		return;

	/* Only unmap the TPU mailboxes if they were found on probe */
	if (gxp->tpu_dev.mbx_paddr) {
		for (i = 0; i < GXP_NUM_CORES; i++) {
			if (!(BIT(i) & core_list))
				continue;
			gcip_iommu_unmap(gdomain, GXP_IOVA_EXT_TPU_MBX + i * EXT_TPU_MBX_SIZE,
					 EXT_TPU_MBX_SIZE);
		}
	}
	for (i = 0; i < GXP_NUM_CORES; i++) {
		if (!(BIT(i) & core_list))
			continue;
		gcip_iommu_unmap(gdomain, gxp->mbx[i].daddr, gxp->mbx[i].size);
	}
	gxp_unmap_csrs(gxp, gdomain, &gxp->regs);
}

static inline struct sg_table *alloc_sgt_for_buffer(void *ptr, size_t size,
						    struct iommu_domain *domain,
						    dma_addr_t daddr)
{
	struct sg_table *sgt;
	ulong offset;
	uint num_ents;
	int ret;
	struct scatterlist *next;
	size_t size_in_page;
	struct page *page;
	void *va_base = ptr;

	/* Calculate the number of entries needed in the table */
	offset = offset_in_page(va_base);
	if (unlikely((size + offset) / PAGE_SIZE >= UINT_MAX - 1 ||
		     size + offset < size))
		return ERR_PTR(-EINVAL);
	num_ents = (size + offset) / PAGE_SIZE;
	if ((size + offset) % PAGE_SIZE)
		num_ents++;

	/* Allocate and setup the table for filling out */
	sgt = kmalloc(sizeof(*sgt), GFP_KERNEL);
	if (!sgt)
		return ERR_PTR(-ENOMEM);

	ret = sg_alloc_table(sgt, num_ents, GFP_KERNEL);
	if (ret) {
		kfree(sgt);
		return ERR_PTR(ret);
	}
	next = sgt->sgl;

	/*
	 * Fill in the first scatterlist entry.
	 * This is the only one which may start at a non-page-aligned address.
	 */
	size_in_page = size > (PAGE_SIZE - offset_in_page(ptr)) ?
			       PAGE_SIZE - offset_in_page(ptr) :
			       size;
	page = phys_to_page(iommu_iova_to_phys(domain, daddr));
	sg_set_page(next, page, size_in_page, offset_in_page(ptr));
	size -= size_in_page;
	ptr += size_in_page;
	next = sg_next(next);

	while (size > 0) {
		/*
		 * Fill in and link the next scatterlist entry.
		 * `ptr` is now page-aligned, so it is only necessary to check
		 * if this entire page is part of the buffer, or if the buffer
		 * ends part way through the page (which means this is the last
		 * entry in the list).
		 */
		size_in_page = size > PAGE_SIZE ? PAGE_SIZE : size;
		page = phys_to_page(iommu_iova_to_phys(
			domain, daddr + (unsigned long long)(ptr - va_base)));
		sg_set_page(next, page, size_in_page, 0);

		size -= size_in_page;
		ptr += size_in_page;
		next = sg_next(next);
	}

	return sgt;
}

#if HAS_TPU_EXT

int gxp_dma_map_tpu_buffer(struct gxp_dev *gxp,
			   struct gcip_iommu_domain *gdomain, uint core_list,
			   struct edgetpu_ext_mailbox_info *mbx_info)
{
	uint orig_core_list = core_list;
	u64 queue_iova;
	int core;
	int ret;
	int i = 0;

	while (core_list) {
		phys_addr_t cmdq_pa = mbx_info->mailboxes[i].cmdq_pa;
		phys_addr_t respq_pa = mbx_info->mailboxes[i++].respq_pa;

		core = ffs(core_list) - 1;
		queue_iova = GXP_IOVA_TPU_MBX_BUFFER(core);
		ret = gcip_iommu_map(gdomain, queue_iova, cmdq_pa, mbx_info->cmdq_size,
				     GCIP_MAP_FLAGS_DMA_RW);
		if (ret)
			goto error;
		ret = gcip_iommu_map(gdomain, queue_iova + mbx_info->cmdq_size, respq_pa,
				     mbx_info->respq_size, GCIP_MAP_FLAGS_DMA_RO);
		if (ret) {
			gcip_iommu_unmap(gdomain, queue_iova, mbx_info->cmdq_size);
			goto error;
		}
		core_list &= ~BIT(core);
	}
	return 0;

error:
	core_list ^= orig_core_list;
	while (core_list) {
		core = ffs(core_list) - 1;
		core_list &= ~BIT(core);
		queue_iova = GXP_IOVA_TPU_MBX_BUFFER(core);
		gcip_iommu_unmap(gdomain, queue_iova, mbx_info->cmdq_size);
		gcip_iommu_unmap(gdomain, queue_iova + mbx_info->cmdq_size, mbx_info->respq_size);
	}
	return ret;
}

void gxp_dma_unmap_tpu_buffer(struct gxp_dev *gxp, struct gcip_iommu_domain *gdomain,
			      struct gxp_tpu_mbx_desc mbx_desc)
{
	uint core_list = mbx_desc.phys_core_list;
	u64 queue_iova;
	int core;

	while (core_list) {
		core = ffs(core_list) - 1;
		core_list &= ~BIT(core);
		queue_iova = GXP_IOVA_TPU_MBX_BUFFER(core);
		gcip_iommu_unmap(gdomain, queue_iova, mbx_desc.cmdq_size);
		gcip_iommu_unmap(gdomain, queue_iova + mbx_desc.cmdq_size, mbx_desc.respq_size);
	}
}

#endif /* HAS_TPU_EXT */

int gxp_dma_map_allocated_coherent_buffer(struct gxp_dev *gxp,
					  struct gxp_coherent_buf *buf,
					  struct gcip_iommu_domain *gdomain,
					  uint gxp_dma_flags)
{
	struct gxp_dma_iommu_manager *mgr = container_of(
		gxp->dma_mgr, struct gxp_dma_iommu_manager, dma_mgr);
	struct sg_table *sgt;
	unsigned int nents_mapped;
	int ret = 0;
	u64 gcip_map_flags = GCIP_MAP_FLAGS_DMA_RW;

	if (gdomain == gxp_iommu_get_domain_for_dev(gxp))
		return 0;

	sgt = alloc_sgt_for_buffer(buf->vaddr, buf->size,
				   mgr->default_domain->domain, buf->dma_addr);
	if (IS_ERR(sgt)) {
		dev_err(gxp->dev,
			"Failed to allocate sgt for coherent buffer\n");
		return PTR_ERR(sgt);
	}

	nents_mapped =
		gcip_iommu_domain_map_sgt_to_iova(gdomain, sgt, buf->dsp_addr, &gcip_map_flags);
	if (!nents_mapped)
		ret = -ENOSPC;

	sg_free_table(sgt);
	kfree(sgt);
	return ret;
}

int gxp_dma_alloc_coherent_buf(struct gxp_dev *gxp,
			       struct gcip_iommu_domain *gdomain, size_t size,
			       gfp_t flag, uint gxp_dma_flags,
			       struct gxp_coherent_buf *buffer)
{
	void *buf;
	dma_addr_t daddr;
	int ret;

	size = size < PAGE_SIZE ? PAGE_SIZE : size;

	/* Allocate a coherent buffer in the default domain */
	buf = dma_alloc_coherent(gxp->dev, size, &daddr, flag);
	if (!buf) {
		dev_err(gxp->dev, "Failed to allocate coherent buffer\n");
		return -ENOMEM;
	}

	buffer->vaddr = buf;
	buffer->size = size;
	buffer->dma_addr = daddr;

	if (!gdomain)
		return 0;

	buffer->dsp_addr = gcip_iommu_alloc_iova(gdomain, size, 0);
	if (!buffer->dsp_addr) {
		ret = -ENOSPC;
		goto err_free_coherent;
	}
	ret = gxp_dma_map_allocated_coherent_buffer(gxp, buffer, gdomain, gxp_dma_flags);
	if (ret)
		goto err_free_iova;

	return 0;

err_free_iova:
	gcip_iommu_free_iova(gdomain, buffer->dsp_addr, size);
err_free_coherent:
	buffer->vaddr = NULL;
	buffer->size = 0;
	dma_free_coherent(gxp->dev, size, buf, daddr);
	return ret;
}

void gxp_dma_unmap_allocated_coherent_buffer(struct gxp_dev *gxp,
					     struct gcip_iommu_domain *gdomain,
					     struct gxp_coherent_buf *buf)
{
	if (gdomain == gxp_iommu_get_domain_for_dev(gxp))
		return;

	gcip_iommu_unmap(gdomain, buf->dsp_addr, buf->size);
}

void gxp_dma_free_coherent_buf(struct gxp_dev *gxp,
			       struct gcip_iommu_domain *gdomain,
			       struct gxp_coherent_buf *buf)
{
	if (gdomain) {
		gxp_dma_unmap_allocated_coherent_buffer(gxp, gdomain, buf);
		gcip_iommu_free_iova(gdomain, buf->dsp_addr, buf->size);
	}
	dma_free_coherent(gxp->dev, buf->size, buf->vaddr, buf->dma_addr);
}

void gxp_dma_sync_sg_for_cpu(struct gxp_dev *gxp, struct scatterlist *sg,
			     int nents, enum dma_data_direction direction)
{
	/*
	 * Syncing is not domain specific. Just call through to DMA API.
	 *
	 * This works even for buffers not mapped via the DMA API, since the
	 * dma-iommu implementation syncs buffers by their physical address
	 * ranges, taken from the scatterlist, without using the IOVA.
	 */
	dma_sync_sg_for_cpu(gxp->dev, sg, nents, direction);
}

void gxp_dma_sync_sg_for_device(struct gxp_dev *gxp, struct scatterlist *sg,
				int nents, enum dma_data_direction direction)
{
	/*
	 * Syncing is not domain specific. Just call through to DMA API.
	 *
	 * This works even for buffers not mapped via the DMA API, since the
	 * dma-iommu implementation syncs buffers by their physical address
	 * ranges, taken from the scatterlist, without using the IOVA.
	 */
	dma_sync_sg_for_device(gxp->dev, sg, nents, direction);
}

u64 gxp_dma_encode_gcip_map_flags(uint gxp_dma_flags, unsigned long dma_attrs)
{
	enum dma_data_direction dir = gxp_dma_flags & GXP_MAP_DIR_MASK;
	bool coherent = false;
	bool restrict_iova = false;

#ifdef GXP_IS_DMA_COHERENT
	coherent = gxp_dma_flags & GXP_MAP_COHERENT;
#endif

	return gcip_iommu_encode_gcip_map_flags(dir, coherent, dma_attrs, restrict_iova);
}
