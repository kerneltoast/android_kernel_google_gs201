// SPDX-License-Identifier: GPL-2.0
/*
 * GXP DMA implemented via IOMMU.
 *
 * Copyright (C) 2021 Google LLC
 */

#include <linux/dma-iommu.h>
#include <linux/dma-mapping.h>
#include <linux/iommu.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>

#include "gxp-config.h"
#include "gxp-dma.h"
#include "gxp-iova.h"
#include "gxp-mapping.h"
#include "gxp-pm.h"
#include "gxp-vd.h"

struct gxp_dma_iommu_manager {
	struct gxp_dma_manager dma_mgr;
	struct iommu_domain *default_domain;
	void __iomem *idma_ssmt_base;
	void __iomem *inst_data_ssmt_base;
};

/**
 * dma_info_to_prot - Translate DMA API directions and attributes to IOMMU API
 *                    page flags.
 * @dir: Direction of DMA transfer
 * @coherent: Is the DMA master cache-coherent?
 * @attrs: DMA attributes for the mapping
 *
 * From drivers/iommu/dma-iommu.c
 *
 * Return: corresponding IOMMU API page protection flags
 */
static int dma_info_to_prot(enum dma_data_direction dir, bool coherent,
			    unsigned long attrs)
{
	int prot = coherent ? IOMMU_CACHE : 0;

	if (attrs & DMA_ATTR_PRIVILEGED)
		prot |= IOMMU_PRIV;
	switch (dir) {
	case DMA_BIDIRECTIONAL:
		return prot | IOMMU_READ | IOMMU_WRITE;
	case DMA_TO_DEVICE:
		return prot | IOMMU_READ;
	case DMA_FROM_DEVICE:
		return prot | IOMMU_WRITE;
	default:
		return 0;
	}
}

/* SSMT handling */

#define INST_SID_FOR_CORE(_x_) ((1 << 6) | ((_x_) << 4) | (0 << 3))
#define DATA_SID_FOR_CORE(_x_) ((1 << 6) | ((_x_) << 4) | (1 << 3))
#define IDMA_SID_FOR_CORE(_x_) ((1 << 6) | ((_x_) << 4))

static inline void ssmt_set_vid_for_sid(void __iomem *ssmt, int vid, u8 sid)
{
	/* NS_READ_STREAM_VID_<sid> */
	writel(vid, (ssmt) + 0x1000u + (0x4u * (sid)));
	/* NS_WRITE_STREAM_VID_<sid> */
	writel(vid, (ssmt) + 0x1200u + (0x4u * (sid)));
}

static int gxp_dma_ssmt_program(struct gxp_dev *gxp,
				struct gxp_virtual_device *vd, uint virt_core,
				uint core)
{
/* SSMT is not supported in unittests */
#ifndef CONFIG_GXP_TEST
	struct gxp_dma_iommu_manager *mgr = container_of(
		gxp->dma_mgr, struct gxp_dma_iommu_manager, dma_mgr);
	int core_vid;

	core_vid = iommu_aux_get_pasid(vd->core_domains[virt_core], gxp->dev);
	dev_dbg(gxp->dev, "SysMMU: core%u assigned vid %d\n", core,
		   core_vid);
	ssmt_set_vid_for_sid(mgr->idma_ssmt_base, core_vid,
			     IDMA_SID_FOR_CORE(core));
	ssmt_set_vid_for_sid(mgr->inst_data_ssmt_base, core_vid,
			     INST_SID_FOR_CORE(core));
	ssmt_set_vid_for_sid(mgr->inst_data_ssmt_base, core_vid,
			     DATA_SID_FOR_CORE(core));
#endif
	return 0;
}


static inline int ssmt_init(struct gxp_dev *gxp,
			    struct gxp_dma_iommu_manager *mgr)
{
	struct platform_device *pdev =
		container_of(gxp->dev, struct platform_device, dev);
	struct resource *r;

	r = platform_get_resource_byname(pdev, IORESOURCE_MEM, "ssmt_idma");
	if (!r) {
		dev_err(gxp->dev, "Failed to find IDMA SSMT register base\n");
		return -EINVAL;
	}

	mgr->idma_ssmt_base = devm_ioremap_resource(gxp->dev, r);
	if (IS_ERR(mgr->idma_ssmt_base)) {
		dev_err(gxp->dev,
			"Failed to map IDMA SSMT register base (%ld)\n",
			PTR_ERR(mgr->idma_ssmt_base));
		return PTR_ERR(mgr->idma_ssmt_base);
	}

	r = platform_get_resource_byname(pdev, IORESOURCE_MEM,
					 "ssmt_inst_data");
	if (!r) {
		dev_err(gxp->dev,
			"Failed to find instruction/data SSMT register base\n");
		return -EINVAL;
	}

	mgr->inst_data_ssmt_base = devm_ioremap_resource(gxp->dev, r);
	if (IS_ERR(mgr->inst_data_ssmt_base)) {
		dev_err(gxp->dev,
			"Failed to map instruction/data SSMT register base (%ld)\n",
			PTR_ERR(mgr->inst_data_ssmt_base));
		return PTR_ERR(mgr->inst_data_ssmt_base);
	}

	return 0;
}

/* Fault handler */

static int sysmmu_fault_handler(struct iommu_fault *fault, void *token)
{
	struct gxp_dev *gxp = (struct gxp_dev *)token;

	switch (fault->type) {
	case IOMMU_FAULT_DMA_UNRECOV:
		dev_err(gxp->dev, "Unrecoverable IOMMU fault!\n");
		break;
	case IOMMU_FAULT_PAGE_REQ:
		dev_err(gxp->dev, "IOMMU page request fault!\n");
		break;
	default:
		dev_err(gxp->dev, "Unexpected IOMMU fault type (%d)\n",
			fault->type);
		return -EAGAIN;
	}

	/*
	 * Normally the iommu driver should fill out the `event` struct for
	 * unrecoverable errors, and the `prm` struct for page request faults.
	 * The SysMMU driver, instead, always fills out the `event` struct.
	 *
	 * Note that the `fetch_addr` and `perm` fields are never filled out,
	 * so we skip printing them.
	 */
	dev_err(gxp->dev, "reason = %08X\n", fault->event.reason);
	dev_err(gxp->dev, "flags = %08X\n", fault->event.flags);
	dev_err(gxp->dev, "pasid = %08X\n", fault->event.pasid);
	dev_err(gxp->dev, "addr = %llX\n", fault->event.addr);

	// Tell the IOMMU driver to carry on
	return -EAGAIN;
}

/* gxp-dma.h Interface */

int gxp_dma_init(struct gxp_dev *gxp)
{
	struct gxp_dma_iommu_manager *mgr;
	int ret;

	/* GXP can only address 32-bit IOVAs */
	ret = dma_set_mask_and_coherent(gxp->dev, DMA_BIT_MASK(32));
	if (ret) {
		dev_err(gxp->dev, "Failed to set DMA mask\n");
		return ret;
	}

	mgr = devm_kzalloc(gxp->dev, sizeof(*mgr), GFP_KERNEL);
	if (!mgr)
		return -ENOMEM;

/* TODO(b/201505925): remove this and prepare a of_node in unittests */
/* SSMT is not supported in unittests */
#ifndef CONFIG_GXP_TEST
	ret = ssmt_init(gxp, mgr);
	if (ret) {
		dev_err(gxp->dev, "Failed to find SSMT\n");
		return ret;
	}
#endif

	mgr->default_domain = iommu_get_domain_for_dev(gxp->dev);
	if (!mgr->default_domain) {
		dev_err(gxp->dev, "Failed to find default IOMMU domain\n");
		return -EIO;
	}

	if (iommu_register_device_fault_handler(gxp->dev, sysmmu_fault_handler,
						gxp)) {
		dev_err(gxp->dev, "Failed to register iommu fault handler\n");
		return -EIO;
	}

	iommu_dev_enable_feature(gxp->dev, IOMMU_DEV_FEAT_AUX);
	if (!iommu_dev_feature_enabled(gxp->dev, IOMMU_DEV_FEAT_AUX)) {
		dev_err(gxp->dev, "Failed to enable aux support in SysMMU\n");
		goto err_unreg_fault_handler;
	}

	/* Enable best fit algorithm to minimize fragmentation */
	iommu_dma_enable_best_fit_algo(gxp->dev);

	gxp->dma_mgr = &(mgr->dma_mgr);

	return 0;

err_unreg_fault_handler:
	if (iommu_unregister_device_fault_handler(gxp->dev))
		dev_err(gxp->dev,
			"Failed to unregister SysMMU fault handler\n");

	return -EIO;
}

void gxp_dma_exit(struct gxp_dev *gxp)
{
	if (iommu_unregister_device_fault_handler(gxp->dev))
		dev_err(gxp->dev,
			"Failed to unregister SysMMU fault handler\n");
}

#define SYNC_BARRIERS_SIZE       0x100000
#define SYNC_BARRIERS_TOP_OFFSET 0x100000
#define EXT_TPU_MBX_SIZE         0x2000

/* Offset from mailbox base to the device interface that needs to be mapped */
#define MAILBOX_DEVICE_INTERFACE_OFFSET 0x10000

void gxp_dma_init_default_resources(struct gxp_dev *gxp)
{
	unsigned int core;

	for (core = 0; core < GXP_NUM_CORES; core++) {
		gxp->mbx[core].daddr = GXP_IOVA_MAILBOX(core);
		gxp->fwbufs[core].daddr = GXP_IOVA_FIRMWARE(core);
	}
	gxp->regs.daddr = GXP_IOVA_AURORA_TOP;
	gxp->fwdatabuf.daddr = GXP_IOVA_FW_DATA;
}

int gxp_dma_domain_attach_device(struct gxp_dev *gxp,
				 struct gxp_virtual_device *vd, uint virt_core,
				 uint core)
{
	int ret;

	ret = iommu_aux_attach_device(vd->core_domains[virt_core], gxp->dev);
	if (ret)
		goto out;
	gxp_dma_ssmt_program(gxp, vd, virt_core, core);
out:
	return ret;
}

void gxp_dma_domain_detach_device(struct gxp_dev *gxp,
				  struct gxp_virtual_device *vd, uint virt_core)
{
	iommu_aux_detach_device(vd->core_domains[virt_core], gxp->dev);
}

int gxp_dma_map_core_resources(struct gxp_dev *gxp,
			       struct gxp_virtual_device *vd, uint virt_core,
			       uint core)
{
	int ret;

	ret = iommu_map(vd->core_domains[virt_core], gxp->regs.daddr,
			gxp->regs.paddr, gxp->regs.size,
			IOMMU_READ | IOMMU_WRITE);
	if (ret)
		goto err;
	/*
	 * Firmware expects to access the sync barriers at a separate
	 * address, lower than the rest of the AURORA_TOP registers.
	 */
	ret = iommu_map(vd->core_domains[virt_core], GXP_IOVA_SYNC_BARRIERS,
			gxp->regs.paddr + SYNC_BARRIERS_TOP_OFFSET,
			SYNC_BARRIERS_SIZE, IOMMU_READ | IOMMU_WRITE);
	if (ret)
		goto err;
	ret = iommu_map(vd->core_domains[virt_core], gxp->mbx[core].daddr,
			gxp->mbx[core].paddr + MAILBOX_DEVICE_INTERFACE_OFFSET,
			gxp->mbx[core].size, IOMMU_READ | IOMMU_WRITE);
	if (ret)
		goto err;
	/*
	 * TODO(b/202213606): Map FW regions of all cores in a VD for
	 * each other at VD creation.
	 */
	ret = iommu_map(vd->core_domains[virt_core], gxp->fwbufs[0].daddr,
			gxp->fwbufs[0].paddr,
			gxp->fwbufs[0].size * GXP_NUM_CORES,
			IOMMU_READ | IOMMU_WRITE);
	if (ret)
		goto err;
	ret = iommu_map(vd->core_domains[virt_core], gxp->fwdatabuf.daddr,
			gxp->fwdatabuf.paddr, gxp->fwdatabuf.size,
			IOMMU_READ | IOMMU_WRITE);
	if (ret)
		goto err;
	/* Only map the TPU mailboxes if they were found on probe */
	if (gxp->tpu_dev.mbx_paddr) {
		ret = iommu_map(
			vd->core_domains[virt_core],
			GXP_IOVA_EXT_TPU_MBX + core * EXT_TPU_MBX_SIZE,
			gxp->tpu_dev.mbx_paddr +
				core * EXT_TPU_MBX_SIZE,
			EXT_TPU_MBX_SIZE, IOMMU_READ | IOMMU_WRITE);
		if (ret)
			goto err;
	}
	return ret;

err:
	/*
	 * Attempt to unmap all resources.
	 * Any resource that hadn't been mapped yet will cause `iommu_unmap()`
	 * to return immediately, so its safe to try to unmap everything.
	 */
	gxp_dma_unmap_core_resources(gxp, vd, virt_core, core);
	return ret;
}

void gxp_dma_unmap_core_resources(struct gxp_dev *gxp,
				  struct gxp_virtual_device *vd, uint virt_core,
				  uint core)
{
	/* Only unmap the TPU mailboxes if they were found on probe */
	if (gxp->tpu_dev.mbx_paddr) {
		iommu_unmap(vd->core_domains[virt_core],
			    GXP_IOVA_EXT_TPU_MBX +
				    core * EXT_TPU_MBX_SIZE,
			    EXT_TPU_MBX_SIZE);
	}
	iommu_unmap(vd->core_domains[virt_core], gxp->fwdatabuf.daddr,
		    gxp->fwdatabuf.size);
	/*
	 * TODO(b/202213606): A core should only have access to the FW
	 * of other cores if they're in the same VD, and have the FW
	 * region unmapped on VD destruction.
	 */
	iommu_unmap(vd->core_domains[virt_core], gxp->fwbufs[0].daddr,
		    gxp->fwbufs[0].size * GXP_NUM_CORES);
	iommu_unmap(vd->core_domains[virt_core], gxp->mbx[core].daddr,
		    gxp->mbx[core].size);
	iommu_unmap(vd->core_domains[virt_core], GXP_IOVA_SYNC_BARRIERS,
		    SYNC_BARRIERS_SIZE);
	iommu_unmap(vd->core_domains[virt_core], gxp->regs.daddr,
		    gxp->regs.size);
}

static inline struct sg_table *
alloc_sgt_for_buffer(void *ptr, size_t size,
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

#if (IS_ENABLED(CONFIG_GXP_TEST) || IS_ENABLED(CONFIG_ANDROID)) && !IS_ENABLED(CONFIG_GXP_GEM5)
int gxp_dma_map_tpu_buffer(struct gxp_dev *gxp, struct gxp_virtual_device *vd,
			   uint virt_core_list, uint core_list,
			   struct edgetpu_ext_mailbox_info *mbx_info)
{
	uint orig_virt_core_list = virt_core_list;
	u64 queue_iova;
	uint virt_core;
	int core;
	int ret;
	int i = 0;

	while (virt_core_list) {
		phys_addr_t cmdq_pa = mbx_info->mailboxes[i].cmdq_pa;
		phys_addr_t respq_pa = mbx_info->mailboxes[i++].respq_pa;

		virt_core = ffs(virt_core_list) - 1;
		virt_core_list &= ~BIT(virt_core);
		core = ffs(core_list) - 1;
		core_list &= ~BIT(core);
		queue_iova = GXP_IOVA_TPU_MBX_BUFFER(core);
		ret = iommu_map(vd->core_domains[virt_core], queue_iova,
				cmdq_pa, mbx_info->cmdq_size, IOMMU_WRITE);
		if (ret)
			goto error;
		ret = iommu_map(vd->core_domains[virt_core],
				queue_iova + mbx_info->cmdq_size, respq_pa,
				mbx_info->respq_size, IOMMU_READ);
		if (ret) {
			iommu_unmap(vd->core_domains[virt_core], queue_iova,
				    mbx_info->cmdq_size);
			goto error;
		}
	}
	return 0;

error:
	virt_core_list ^= orig_virt_core_list;
	while (virt_core_list) {
		virt_core = ffs(virt_core_list) - 1;
		virt_core_list &= ~BIT(virt_core);
		core = ffs(core_list) - 1;
		core_list &= ~BIT(core);
		queue_iova = GXP_IOVA_TPU_MBX_BUFFER(core);
		iommu_unmap(vd->core_domains[virt_core], queue_iova,
			    mbx_info->cmdq_size);
		iommu_unmap(vd->core_domains[virt_core], queue_iova +
			    mbx_info->cmdq_size, mbx_info->respq_size);
	}
	return ret;
}

void gxp_dma_unmap_tpu_buffer(struct gxp_dev *gxp,
			      struct gxp_virtual_device *vd,
			      struct gxp_tpu_mbx_desc mbx_desc)
{
	uint virt_core_list = mbx_desc.virt_core_list;
	uint core_list = mbx_desc.phys_core_list;
	u64 queue_iova;
	int core;
	uint virt_core;

	while (virt_core_list) {
		virt_core = ffs(virt_core_list) - 1;
		virt_core_list &= ~BIT(virt_core);
		core = ffs(core_list) - 1;
		core_list &= ~BIT(core);
		queue_iova = GXP_IOVA_TPU_MBX_BUFFER(core);
		iommu_unmap(vd->core_domains[virt_core], queue_iova,
			    mbx_desc.cmdq_size);
		iommu_unmap(vd->core_domains[virt_core], queue_iova +
			    mbx_desc.cmdq_size, mbx_desc.respq_size);
	}
}
#endif  // (CONFIG_GXP_TEST || CONFIG_ANDROID) && !CONFIG_GXP_GEM5

int gxp_dma_map_allocated_coherent_buffer(struct gxp_dev *gxp, void *buf,
					  struct gxp_virtual_device *vd,
					  uint virt_core_list, size_t size,
					  dma_addr_t dma_handle,
					  uint gxp_dma_flags)
{
	struct gxp_dma_iommu_manager *mgr = container_of(
		gxp->dma_mgr, struct gxp_dma_iommu_manager, dma_mgr);
	struct sg_table *sgt;
	int virt_core;
	ssize_t size_mapped;

	size = size < PAGE_SIZE ? PAGE_SIZE : size;
	sgt = alloc_sgt_for_buffer(buf, size, mgr->default_domain, dma_handle);
	if (IS_ERR(sgt)) {
		dev_err(gxp->dev,
			"Failed to allocate sgt for coherent buffer\n");
		return -ENOMEM;
	}

	/* Create identical mappings in the specified cores' domains */
	for (virt_core = 0; virt_core < vd->num_cores; virt_core++) {
		if (!(virt_core_list & BIT(virt_core)))
			continue;
		/*
		 * In Linux 5.15 and beyond, `iommu_map_sg()` returns a
		 * `ssize_t` to encode errors that earlier versions throw out.
		 * Explicitly cast here for backwards compatibility.
		 */
		size_mapped = (ssize_t)iommu_map_sg(vd->core_domains[virt_core],
						    dma_handle, sgt->sgl,
						    sgt->orig_nents,
						    IOMMU_READ | IOMMU_WRITE);
		if (size_mapped != size)
			goto err;
	}

	sg_free_table(sgt);
	kfree(sgt);
	return 0;

err:
	for (virt_core -= 1; virt_core >= 0; virt_core--)
		iommu_unmap(vd->core_domains[virt_core], dma_handle, size);

	sg_free_table(sgt);
	kfree(sgt);
	return -EINVAL;
}

void *gxp_dma_alloc_coherent(struct gxp_dev *gxp, struct gxp_virtual_device *vd,
			     uint virt_core_list, size_t size,
			     dma_addr_t *dma_handle, gfp_t flag,
			     uint gxp_dma_flags)
{
	void *buf;
	dma_addr_t daddr;
	int ret;

	size = size < PAGE_SIZE ? PAGE_SIZE : size;

	/* Allocate a coherent buffer in the default domain */
	buf = dma_alloc_coherent(gxp->dev, size, &daddr, flag);
	if (!buf) {
		dev_err(gxp->dev, "Failed to allocate coherent buffer\n");
		return NULL;
	}
	if (vd != NULL) {
		ret = gxp_dma_map_allocated_coherent_buffer(gxp, buf, vd,
							    virt_core_list,
							    size, daddr,
							    gxp_dma_flags);
		if (ret) {
			dma_free_coherent(gxp->dev, size, buf, daddr);
			return NULL;
		}
	}

	if (dma_handle)
		*dma_handle = daddr;

	return buf;
}

void gxp_dma_unmap_allocated_coherent_buffer(struct gxp_dev *gxp,
					     struct gxp_virtual_device *vd,
					     uint virt_core_list, size_t size,
					     dma_addr_t dma_handle)
{
	int virt_core;

	size = size < PAGE_SIZE ? PAGE_SIZE : size;

	for (virt_core = 0; virt_core < vd->num_cores; virt_core++) {
		if (!(virt_core_list & BIT(virt_core)))
			continue;
		if (size !=
		    iommu_unmap(vd->core_domains[virt_core], dma_handle, size))
			dev_warn(gxp->dev, "Failed to unmap coherent buffer\n");
	}
}

void gxp_dma_free_coherent(struct gxp_dev *gxp, struct gxp_virtual_device *vd,
			   uint virt_core_list, size_t size, void *cpu_addr,
			   dma_addr_t dma_handle)
{
	if (vd != NULL)
		gxp_dma_unmap_allocated_coherent_buffer(gxp, vd, virt_core_list,
							size, dma_handle);
	dma_free_coherent(gxp->dev, size, cpu_addr, dma_handle);
}

dma_addr_t gxp_dma_map_single(struct gxp_dev *gxp,
			      struct gxp_virtual_device *vd,
			      uint virt_core_list, void *cpu_addr, size_t size,
			      enum dma_data_direction direction,
			      unsigned long attrs, uint gxp_dma_flags)
{
	struct gxp_dma_iommu_manager *mgr = container_of(
		gxp->dma_mgr, struct gxp_dma_iommu_manager, dma_mgr);
	dma_addr_t daddr;
	phys_addr_t paddr;
	int prot = dma_info_to_prot(direction, 0, attrs);
	int virt_core;

	daddr = dma_map_single_attrs(gxp->dev, cpu_addr, size, direction,
				     attrs);
	if (dma_mapping_error(gxp->dev, daddr))
		return DMA_MAPPING_ERROR;

	paddr = iommu_iova_to_phys(mgr->default_domain, daddr);
	for (virt_core = 0; virt_core < vd->num_cores; virt_core++) {
		if (!(virt_core_list & BIT(virt_core)))
			continue;
		if (iommu_map(vd->core_domains[virt_core], daddr, paddr, size,
			      prot))
			goto err;
	}

	return daddr;

err:
	for (virt_core -= 1; virt_core >= 0; virt_core--)
		iommu_unmap(vd->core_domains[virt_core], daddr, size);
	dma_unmap_single_attrs(gxp->dev, daddr, size, direction,
			       DMA_ATTR_SKIP_CPU_SYNC);
	return DMA_MAPPING_ERROR;
}

void gxp_dma_unmap_single(struct gxp_dev *gxp, struct gxp_virtual_device *vd,
			  uint virt_core_list, dma_addr_t dma_addr, size_t size,
			  enum dma_data_direction direction,
			  unsigned long attrs)
{
	int virt_core;

	for (virt_core = 0; virt_core < vd->num_cores; virt_core++) {
		if (!(virt_core_list & BIT(virt_core)))
			continue;
		if (size !=
		    iommu_unmap(vd->core_domains[virt_core], dma_addr, size))
			dev_warn(gxp->dev, "Failed to unmap single\n");
	}

	dma_unmap_single_attrs(gxp->dev, dma_addr, size, direction, attrs);
}

dma_addr_t gxp_dma_map_page(struct gxp_dev *gxp, struct gxp_virtual_device *vd,
			    uint virt_core_list, struct page *page,
			    unsigned long offset, size_t size,
			    enum dma_data_direction direction,
			    unsigned long attrs, uint gxp_dma_flags)
{
	struct gxp_dma_iommu_manager *mgr = container_of(
		gxp->dma_mgr, struct gxp_dma_iommu_manager, dma_mgr);
	dma_addr_t daddr;
	phys_addr_t paddr;
	int prot = dma_info_to_prot(direction, 0, attrs);
	int virt_core;

	daddr = dma_map_page_attrs(gxp->dev, page, offset, size, direction,
				   attrs);
	if (dma_mapping_error(gxp->dev, daddr))
		return DMA_MAPPING_ERROR;

	paddr = iommu_iova_to_phys(mgr->default_domain, daddr);
	for (virt_core = 0; virt_core < vd->num_cores; virt_core++) {
		if (!(virt_core_list & BIT(virt_core)))
			continue;
		if (iommu_map(vd->core_domains[virt_core], daddr, paddr, size,
			      prot))
			goto err;
	}

	return daddr;

err:
	for (virt_core -= 1; virt_core >= 0; virt_core--)
		iommu_unmap(vd->core_domains[virt_core], daddr, size);
	dma_unmap_page_attrs(gxp->dev, daddr, size, direction,
				DMA_ATTR_SKIP_CPU_SYNC);
	return DMA_MAPPING_ERROR;
}

void gxp_dma_unmap_page(struct gxp_dev *gxp, struct gxp_virtual_device *vd,
			uint virt_core_list, dma_addr_t dma_addr, size_t size,
			enum dma_data_direction direction, unsigned long attrs)
{
	int virt_core;

	for (virt_core = 0; virt_core < vd->num_cores; virt_core++) {
		if (!(virt_core_list & BIT(virt_core)))
			continue;
		if (size !=
		    iommu_unmap(vd->core_domains[virt_core], dma_addr, size))
			dev_warn(gxp->dev, "Failed to unmap page\n");
	}

	dma_unmap_page_attrs(gxp->dev, dma_addr, size, direction, attrs);
}

dma_addr_t gxp_dma_map_resource(struct gxp_dev *gxp,
				struct gxp_virtual_device *vd,
				uint virt_core_list, phys_addr_t phys_addr,
				size_t size, enum dma_data_direction direction,
				unsigned long attrs, uint gxp_dma_flags)
{
	dma_addr_t daddr;
	int prot = dma_info_to_prot(direction, 0, attrs);
	int virt_core;

	daddr = dma_map_resource(gxp->dev, phys_addr, size, direction, attrs);
	if (dma_mapping_error(gxp->dev, daddr))
		return DMA_MAPPING_ERROR;

	for (virt_core = 0; virt_core < vd->num_cores; virt_core++) {
		if (!(virt_core_list & BIT(virt_core)))
			continue;
		if (iommu_map(vd->core_domains[virt_core], daddr, phys_addr,
			      size, prot))
			goto err;
	}

	return daddr;

err:
	for (virt_core -= 1; virt_core >= 0; virt_core--)
		iommu_unmap(vd->core_domains[virt_core], daddr, size);
	dma_unmap_resource(gxp->dev, daddr, size, direction,
			   DMA_ATTR_SKIP_CPU_SYNC);
	return DMA_MAPPING_ERROR;
}

void gxp_dma_unmap_resource(struct gxp_dev *gxp, struct gxp_virtual_device *vd,
			    uint virt_core_list, dma_addr_t dma_addr,
			    size_t size, enum dma_data_direction direction,
			    unsigned long attrs)
{
	int virt_core;

	for (virt_core = 0; virt_core < vd->num_cores; virt_core++) {
		if (!(virt_core_list & BIT(virt_core)))
			continue;
		if (size !=
		    iommu_unmap(vd->core_domains[virt_core], dma_addr, size))
			dev_warn(gxp->dev, "Failed to unmap resource\n");
	}

	dma_unmap_resource(gxp->dev, dma_addr, size, direction, attrs);
}

int gxp_dma_map_sg(struct gxp_dev *gxp, struct gxp_virtual_device *vd,
		   int virt_core_list, struct scatterlist *sg, int nents,
		   enum dma_data_direction direction, unsigned long attrs,
		   uint gxp_dma_flags)
{
	int nents_mapped;
	dma_addr_t daddr;
	int prot = dma_info_to_prot(direction, 0, attrs);
	int virt_core;
	ssize_t size_mapped;
	/* Variables needed to cleanup if an error occurs */
	struct scatterlist *s;
	int i;
	size_t size = 0;

	nents_mapped = dma_map_sg_attrs(gxp->dev, sg, nents, direction, attrs);
	if (!nents_mapped)
		return 0;

	daddr = sg_dma_address(sg);

	for (virt_core = 0; virt_core < vd->num_cores; virt_core++) {
		if (!(virt_core_list & BIT(virt_core)))
			continue;
		/*
		 * In Linux 5.15 and beyond, `iommu_map_sg()` returns a
		 * `ssize_t` to encode errors that earlier versions throw out.
		 * Explicitly cast here for backwards compatibility.
		 */
		size_mapped = (ssize_t)iommu_map_sg(vd->core_domains[virt_core],
						    daddr, sg, nents, prot);
		if (size_mapped <= 0)
			goto err;
	}

	return nents_mapped;

err:
	for_each_sg(sg, s, nents, i) {
		size += sg_dma_len(s);
	}

	for (virt_core -= 1; virt_core >= 0; virt_core--)
		iommu_unmap(vd->core_domains[virt_core], daddr, size);
	dma_unmap_sg_attrs(gxp->dev, sg, nents, direction, attrs);
	return 0;
}

void gxp_dma_unmap_sg(struct gxp_dev *gxp, struct gxp_virtual_device *vd,
		      uint virt_core_list, struct scatterlist *sg, int nents,
		      enum dma_data_direction direction, unsigned long attrs)
{
	struct scatterlist *s;
	int i;
	size_t size = 0;
	int virt_core;

	for_each_sg(sg, s, nents, i) {
		size += sg_dma_len(s);
	}

	for (virt_core = 0; virt_core < vd->num_cores; virt_core++) {
		if (!(virt_core_list & BIT(virt_core)))
			continue;
		if (!iommu_unmap(vd->core_domains[virt_core], sg_dma_address(sg),
				 size))
			dev_warn(gxp->dev, "Failed to unmap sg\n");
	}

	dma_unmap_sg_attrs(gxp->dev, sg, nents, direction, attrs);
}

void gxp_dma_sync_single_for_cpu(struct gxp_dev *gxp, dma_addr_t dma_handle,
				 size_t size,
				 enum dma_data_direction direction)
{
	/* Syncing is not domain specific. Just call through to DMA API */
	dma_sync_single_for_cpu(gxp->dev, dma_handle, size, direction);
}

void gxp_dma_sync_single_for_device(struct gxp_dev *gxp, dma_addr_t dma_handle,
				    size_t size,
				    enum dma_data_direction direction)
{
	/* Syncing is not domain specific. Just call through to DMA API */
	dma_sync_single_for_device(gxp->dev, dma_handle, size, direction);
}

void gxp_dma_sync_sg_for_cpu(struct gxp_dev *gxp, struct scatterlist *sg,
			     int nents, enum dma_data_direction direction)
{
	/* Syncing is not domain specific. Just call through to DMA API */
	dma_sync_sg_for_cpu(gxp->dev, sg, nents, direction);
}

void gxp_dma_sync_sg_for_device(struct gxp_dev *gxp, struct scatterlist *sg,
				int nents, enum dma_data_direction direction)
{
	/* Syncing is not domain specific. Just call through to DMA API */
	dma_sync_sg_for_device(gxp->dev, sg, nents, direction);
}

struct sg_table *gxp_dma_map_dmabuf_attachment(
	struct gxp_dev *gxp, struct gxp_virtual_device *vd, uint virt_core_list,
	struct dma_buf_attachment *attachment,
	enum dma_data_direction direction)
{
	struct sg_table *sgt;
	int prot = dma_info_to_prot(direction, /*coherent=*/0, /*attrs=*/0);
	ssize_t size_mapped;
	int virt_core;
	int ret;
	/* Variables needed to cleanup if an error occurs */
	struct scatterlist *s;
	int i;
	size_t size = 0;

	/* Map the attachment into the default domain */
	sgt = dma_buf_map_attachment(attachment, direction);
	if (IS_ERR(sgt)) {
		dev_err(gxp->dev,
			"DMA: dma_buf_map_attachment failed (ret=%ld)\n",
			PTR_ERR(sgt));
		return sgt;
	}

	/* Map the sgt into the aux domain of all specified cores */
	for (virt_core = 0; virt_core < vd->num_cores; virt_core++) {
		if (!(virt_core_list & BIT(virt_core)))
			continue;
		/*
		 * In Linux 5.15 and beyond, `iommu_map_sg()` returns a
		 * `ssize_t` to encode errors that earlier versions throw out.
		 * Explicitly cast here for backwards compatibility.
		 */
		size_mapped =
			(ssize_t)iommu_map_sg(vd->core_domains[virt_core],
					      sg_dma_address(sgt->sgl),
					      sgt->sgl, sgt->orig_nents, prot);
		if (size_mapped <= 0) {
			dev_err(gxp->dev,
				"Failed to map dma-buf to virtual core %d (ret=%ld)\n",
				virt_core, size_mapped);
			/*
			 * Prior to Linux 5.15, `iommu_map_sg()` returns 0 for
			 * any failure. Return a generic IO error in this case.
			 */
			ret = size_mapped == 0 ? -EIO : (int)size_mapped;
			goto err;
		}
	}

	return sgt;

err:
	for_each_sg(sgt->sgl, s, sgt->nents, i)
		size += sg_dma_len(s);

	for (virt_core -= 1; virt_core >= 0; virt_core--)
		iommu_unmap(vd->core_domains[virt_core], sg_dma_address(sgt->sgl), size);
	dma_buf_unmap_attachment(attachment, sgt, direction);

	return ERR_PTR(ret);

}

void gxp_dma_unmap_dmabuf_attachment(struct gxp_dev *gxp,
				     struct gxp_virtual_device *vd,
				     uint virt_core_list,
				     struct dma_buf_attachment *attachment,
				     struct sg_table *sgt,
				     enum dma_data_direction direction)
{
	struct scatterlist *s;
	int i;
	size_t size = 0;
	int virt_core;

	/* Find the size of the mapping in IOVA-space */
	for_each_sg(sgt->sgl, s, sgt->nents, i)
		size += sg_dma_len(s);

	/* Unmap the dma-buf from the aux domain of all specified cores */
	for (virt_core = 0; virt_core < vd->num_cores; virt_core++) {
		if (!(virt_core_list & BIT(virt_core)))
			continue;
		if (!iommu_unmap(vd->core_domains[virt_core],
				 sg_dma_address(sgt->sgl), size))
			dev_warn(
				gxp->dev,
				"Failed to unmap dma-buf from virtual core %d\n",
				virt_core);
	}

	/* Unmap the attachment from the default domain */
	dma_buf_unmap_attachment(attachment, sgt, direction);
}
