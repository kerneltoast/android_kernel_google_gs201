// SPDX-License-Identifier: GPL-2.0
/*
 * GXP virtual device manager.
 *
 * Copyright (C) 2021 Google LLC
 */

#include <linux/atomic.h>
#include <linux/bitops.h>
#include <linux/idr.h>
#include <linux/iommu.h>
#include <linux/refcount.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#include <gcip/gcip-alloc-helper.h>
#include <gcip/gcip-image-config.h>

#include "gxp-config.h"
#include "gxp-core-telemetry.h"
#include "gxp-debug-dump.h"
#include "gxp-dma.h"
#include "gxp-domain-pool.h"
#include "gxp-doorbell.h"
#include "gxp-eventfd.h"
#include "gxp-firmware-data.h"
#include "gxp-firmware-loader.h"
#include "gxp-firmware.h"
#include "gxp-host-device-structs.h"
#include "gxp-internal.h"
#include "gxp-lpm.h"
#include "gxp-mailbox.h"
#include "gxp-notification.h"
#include "gxp-pm.h"
#include "gxp-vd.h"

static inline void hold_core_in_reset(struct gxp_dev *gxp, uint core)
{
	gxp_write_32(gxp, GXP_CORE_REG_ETM_PWRCTL(core),
		     BIT(GXP_REG_ETM_PWRCTL_CORE_RESET_SHIFT));
}

void gxp_vd_init(struct gxp_dev *gxp)
{
	uint core;

	init_rwsem(&gxp->vd_semaphore);

	/* All cores start as free */
	for (core = 0; core < GXP_NUM_CORES; core++)
		gxp->core_to_vd[core] = NULL;
	atomic_set(&gxp->next_vdid, 0);
	ida_init(&gxp->shared_slice_idp);
}

void gxp_vd_destroy(struct gxp_dev *gxp)
{
	ida_destroy(&gxp->shared_slice_idp);
}

/* Allocates an SGT and map @daddr to it. */
static int map_ns_region(struct gxp_virtual_device *vd, dma_addr_t daddr,
			 size_t size)
{
	struct gxp_dev *gxp = vd->gxp;
	struct sg_table *sgt;
	size_t idx;
	const size_t n_reg = ARRAY_SIZE(vd->ns_regions);
	int ret;

	for (idx = 0; idx < n_reg; idx++) {
		if (!vd->ns_regions[idx].sgt)
			break;
	}
	if (idx == n_reg) {
		dev_err(gxp->dev, "NS regions array %zx is full", n_reg);
		return -ENOSPC;
	}
	sgt = gcip_alloc_noncontiguous(gxp->dev, size, GFP_KERNEL);
	if (!sgt)
		return -ENOMEM;

	ret = gxp_dma_map_iova_sgt(gxp, vd->domain, daddr, sgt,
				   IOMMU_READ | IOMMU_WRITE);
	if (ret) {
		dev_err(gxp->dev, "NS map %pad with size %#zx failed", &daddr,
			size);
		gcip_free_noncontiguous(sgt);
		return ret;
	}
	vd->ns_regions[idx].daddr = daddr;
	vd->ns_regions[idx].sgt = sgt;

	return 0;
}

static void unmap_ns_region(struct gxp_virtual_device *vd, dma_addr_t daddr)
{
	struct gxp_dev *gxp = vd->gxp;
	struct sg_table *sgt;
	size_t idx;
	const size_t n_reg = ARRAY_SIZE(vd->ns_regions);

	for (idx = 0; idx < n_reg; idx++) {
		if (daddr == vd->ns_regions[idx].daddr)
			break;
	}
	if (idx == n_reg) {
		dev_warn(gxp->dev, "unable to find NS mapping @ %pad", &daddr);
		return;
	}

	sgt = vd->ns_regions[idx].sgt;
	vd->ns_regions[idx].sgt = NULL;
	vd->ns_regions[idx].daddr = 0;
	gxp_dma_unmap_iova_sgt(gxp, vd->domain, daddr, sgt);
	gcip_free_noncontiguous(sgt);
}

/* Maps the shared buffer region to @vd->domain. */
static int map_core_shared_buffer(struct gxp_virtual_device *vd)
{
	struct gxp_dev *gxp = vd->gxp;
	const size_t shared_size = GXP_SHARED_SLICE_SIZE;

	if (!gxp->shared_buf.paddr)
		return 0;
	return gxp_iommu_map(gxp, vd->domain, gxp->shared_buf.daddr,
			     gxp->shared_buf.paddr +
				     shared_size * vd->slice_index,
			     shared_size, IOMMU_READ | IOMMU_WRITE);
}

/* Reverts map_core_shared_buffer. */
static void unmap_core_shared_buffer(struct gxp_virtual_device *vd)
{
	struct gxp_dev *gxp = vd->gxp;
	const size_t shared_size = GXP_SHARED_SLICE_SIZE;

	if (!gxp->shared_buf.paddr)
		return;
	gxp_iommu_unmap(gxp, vd->domain, gxp->shared_buf.daddr, shared_size);
}

/* Maps @res->daddr to @res->paddr to @vd->domain. */
static int map_resource(struct gxp_virtual_device *vd,
			struct gxp_mapped_resource *res)
{
	if (res->daddr == 0)
		return 0;
	return gxp_iommu_map(vd->gxp, vd->domain, res->daddr, res->paddr,
			     res->size, IOMMU_READ | IOMMU_WRITE);
}

/* Reverts map_resource. */
static void unmap_resource(struct gxp_virtual_device *vd,
			   struct gxp_mapped_resource *res)
{
	if (res->daddr == 0)
		return;
	gxp_iommu_unmap(vd->gxp, vd->domain, res->daddr, res->size);
}

/*
 * Assigns @res's IOVA, size from image config.
 */
static void assign_resource(struct gxp_mapped_resource *res,
			    struct gcip_image_config *img_cfg,
			    enum gxp_imgcfg_idx idx)
{
	res->daddr = img_cfg->iommu_mappings[idx].virt_address;
	res->size = gcip_config_to_size(
		img_cfg->iommu_mappings[idx].image_config_value);
}

/*
 * This function does follows:
 *  - Get CORE_CFG, VD_CFG, SYS_CFG's IOVAs and sizes from image config.
 *  - Map above regions with this layout:
 * Pool
 *  +------------------------------------+
 *  |          SLICE_0: CORE_CFG         |
 *  |           SLICE_0: VD_CFG          |
 *  | <padding to GXP_SHARED_SLICE_SIZE> |
 *  +------------------------------------+
 *  |          SLICE_1: CORE_CFG         |
 *  |           SLICE_1: VD_CFG          |
 *  | <padding to GXP_SHARED_SLICE_SIZE> |
 *  +------------------------------------+
 *  |            ... SLICE_N             |
 *  +------------------------------------+
 *  |             <padding>              |
 *  +------------------------------------+
 *  |              SYS_CFG               |
 *  +------------------------------------+
 *
 * To keep compatibility, if not both mapping[0, 1] present then this function
 * falls back to map the MCU-core shared region with hard-coded IOVA and size.
 */
static int map_cfg_regions(struct gxp_virtual_device *vd,
			   struct gcip_image_config *img_cfg)
{
	struct gxp_dev *gxp = vd->gxp;
	struct gxp_mapped_resource pool;
	struct gxp_mapped_resource res;
	size_t offset;
	int ret;

	if (img_cfg->num_iommu_mappings < 3)
		return map_core_shared_buffer(vd);
	pool = gxp_fw_data_resource(gxp);

	assign_resource(&res, img_cfg, CORE_CFG_REGION_IDX);
	offset = vd->slice_index * GXP_SHARED_SLICE_SIZE;
	res.vaddr = pool.vaddr + offset;
	res.paddr = pool.paddr + offset;
	ret = map_resource(vd, &res);
	if (ret) {
		dev_err(gxp->dev, "map core config %pad -> offset %#zx failed",
			&res.daddr, offset);
		return ret;
	}
	vd->core_cfg = res;

	assign_resource(&res, img_cfg, VD_CFG_REGION_IDX);
	offset += vd->core_cfg.size;
	res.vaddr = pool.vaddr + offset;
	res.paddr = pool.paddr + offset;
	ret = map_resource(vd, &res);
	if (ret) {
		dev_err(gxp->dev, "map VD config %pad -> offset %#zx failed",
			&res.daddr, offset);
		goto err_unmap_core;
	}
	vd->vd_cfg = res;
	/* image config correctness check */
	if (vd->core_cfg.size + vd->vd_cfg.size > GXP_SHARED_SLICE_SIZE) {
		dev_err(gxp->dev,
			"Core CFG (%#llx) + VD CFG (%#llx) exceeds %#x",
			vd->core_cfg.size, vd->vd_cfg.size,
			GXP_SHARED_SLICE_SIZE);
		ret = -ENOSPC;
		goto err_unmap_vd;
	}
	assign_resource(&res, img_cfg, SYS_CFG_REGION_IDX);
	if (res.size != GXP_FW_DATA_SYSCFG_SIZE) {
		dev_err(gxp->dev, "invalid system cfg size: %#llx", res.size);
		ret = -EINVAL;
		goto err_unmap_vd;
	}
	res.vaddr = gxp_fw_data_system_cfg(gxp);
	offset = res.vaddr - pool.vaddr;
	res.paddr = pool.paddr + offset;
	ret = map_resource(vd, &res);
	if (ret) {
		dev_err(gxp->dev, "map sys config %pad -> offset %#zx failed",
			&res.daddr, offset);
		goto err_unmap_vd;
	}
	vd->sys_cfg = res;

	return 0;

err_unmap_vd:
	unmap_resource(vd, &vd->vd_cfg);
	vd->vd_cfg.daddr = 0;
err_unmap_core:
	unmap_resource(vd, &vd->core_cfg);
	vd->core_cfg.daddr = 0;
	return ret;
}

static void unmap_cfg_regions(struct gxp_virtual_device *vd)
{
	if (vd->core_cfg.daddr == 0)
		return unmap_core_shared_buffer(vd);

	unmap_resource(vd, &vd->sys_cfg);
	unmap_resource(vd, &vd->vd_cfg);
	unmap_resource(vd, &vd->core_cfg);
}

static int gxp_vd_imgcfg_map(void *data, dma_addr_t daddr, phys_addr_t paddr,
			     size_t size, unsigned int flags)
{
	struct gxp_virtual_device *vd = data;

	if (flags & GCIP_IMAGE_CONFIG_FLAGS_SECURE)
		return 0;

	return map_ns_region(vd, daddr, size);
}

static void gxp_vd_imgcfg_unmap(void *data, dma_addr_t daddr, size_t size,
				unsigned int flags)
{
	struct gxp_virtual_device *vd = data;

	if (flags & GCIP_IMAGE_CONFIG_FLAGS_SECURE)
		return;

	unmap_ns_region(vd, daddr);
}

static int
map_fw_image_config(struct gxp_dev *gxp, struct gxp_virtual_device *vd,
		    struct gxp_firmware_loader_manager *fw_loader_mgr)
{
	int ret;
	struct gcip_image_config *cfg;
	static const struct gcip_image_config_ops gxp_vd_imgcfg_ops = {
		.map = gxp_vd_imgcfg_map,
		.unmap = gxp_vd_imgcfg_unmap,
	};

	cfg = &fw_loader_mgr->core_img_cfg;
	ret = gcip_image_config_parser_init(&vd->cfg_parser, &gxp_vd_imgcfg_ops,
					    gxp->dev, vd);
	/* parser_init() never fails unless we pass invalid OPs. */
	if (unlikely(ret))
		return ret;
	ret = gcip_image_config_parse(&vd->cfg_parser, cfg);
	if (ret) {
		dev_err(gxp->dev, "Image config mapping failed");
		return ret;
	}
	ret = map_cfg_regions(vd, cfg);
	if (ret) {
		dev_err(gxp->dev, "Config regions mapping failed");
		gcip_image_config_clear(&vd->cfg_parser);
		return ret;
	}
	vd->fw_ro_size = cfg->firmware_size;
	/*
	 * To be compatible with image config without setting firmware_size,
	 * fall back to map the whole region to carveout.
	 */
	if (vd->fw_ro_size == 0)
		vd->fw_ro_size = gxp->fwbufs[0].size;

	return 0;
}

static void unmap_fw_image_config(struct gxp_dev *gxp,
				  struct gxp_virtual_device *vd)
{
	unmap_cfg_regions(vd);
	gcip_image_config_clear(&vd->cfg_parser);
}

/*
 * For each core,
 *  - fw_rw_size = fwbufs[core].size - fw_ro_size
 *  - allocates rwdata_sgt[core] with size fw_rw_size
 *  - maps fwbufs[core].daddr -> fwbufs[core].paddr with size fw_ro_size
 *  - maps fwbufs[core].daddr + fw_ro_size -> rwdata_sgt[core]
 */
static int alloc_and_map_fw_image(struct gxp_dev *gxp,
				  struct gxp_virtual_device *vd)
{
	size_t ro_size = vd->fw_ro_size, rw_size;
	struct gxp_iommu_domain *gdomain = vd->domain;
	int i, ret;

	/* Maps all FW regions together and no rwdata_sgt in this case. */
	if (ro_size == gxp->fwbufs[0].size)
		return gxp_iommu_map(gxp, gdomain, gxp->fwbufs[0].daddr,
				     gxp->fwbufs[0].paddr,
				     ro_size * GXP_NUM_CORES,
				     IOMMU_READ | IOMMU_WRITE);

	dev_info(gxp->dev, "mapping firmware RO size %#zx", ro_size);
	rw_size = gxp->fwbufs[0].size - ro_size;
	for (i = 0; i < GXP_NUM_CORES; i++) {
		vd->rwdata_sgt[i] =
			gcip_alloc_noncontiguous(gxp->dev, rw_size, GFP_KERNEL);
		if (!vd->rwdata_sgt[i]) {
			dev_err(gxp->dev,
				"allocate firmware data for core %d failed", i);
			ret = -ENOMEM;
			goto err_free_sgt;
		}
	}
	for (i = 0; i < GXP_NUM_CORES; i++) {
		ret = gxp_iommu_map(gxp, gdomain, gxp->fwbufs[i].daddr,
				    gxp->fwbufs[i].paddr, ro_size,
				    IOMMU_READ | IOMMU_WRITE);
		if (ret) {
			dev_err(gxp->dev, "map firmware RO for core %d failed",
				i);
			goto err_unmap;
		}
		ret = gxp_dma_map_iova_sgt(gxp, vd->domain,
					   gxp->fwbufs[i].daddr + ro_size,
					   vd->rwdata_sgt[i],
					   IOMMU_READ | IOMMU_WRITE);
		if (ret) {
			dev_err(gxp->dev, "map firmware RW for core %d failed",
				i);
			gxp_iommu_unmap(gxp, gdomain, gxp->fwbufs[i].daddr,
					ro_size);
			goto err_unmap;
		}
	}
	return 0;

err_unmap:
	while (i--) {
		gxp_iommu_unmap(gxp, gdomain, gxp->fwbufs[i].daddr, ro_size);
		gxp_dma_unmap_iova_sgt(gxp, vd->domain,
				       gxp->fwbufs[i].daddr + ro_size,
				       vd->rwdata_sgt[i]);
	}
err_free_sgt:
	for (i = 0; i < GXP_NUM_CORES; i++) {
		if (vd->rwdata_sgt[i])
			gcip_free_noncontiguous(vd->rwdata_sgt[i]);
		vd->rwdata_sgt[i] = NULL;
	}
	return ret;
}

static void unmap_and_free_fw_image(struct gxp_dev *gxp,
				    struct gxp_virtual_device *vd)
{
	size_t ro_size = vd->fw_ro_size;
	struct gxp_iommu_domain *gdomain = vd->domain;
	int i;

	if (ro_size == gxp->fwbufs[0].size) {
		gxp_iommu_unmap(gxp, gdomain, gxp->fwbufs[0].daddr,
				ro_size * GXP_NUM_CORES);
		return;
	}

	for (i = 0; i < GXP_NUM_CORES; i++) {
		gxp_iommu_unmap(gxp, gdomain, gxp->fwbufs[i].daddr, ro_size);
		gxp_dma_unmap_iova_sgt(gxp, vd->domain,
				       gxp->fwbufs[i].daddr + ro_size,
				       vd->rwdata_sgt[i]);
	}
	for (i = 0; i < GXP_NUM_CORES; i++) {
		if (vd->rwdata_sgt[i])
			gcip_free_noncontiguous(vd->rwdata_sgt[i]);
		vd->rwdata_sgt[i] = NULL;
	}
}

static int map_core_telemetry_buffers(struct gxp_dev *gxp,
				      struct gxp_virtual_device *vd,
				      uint core_list)
{
	struct buffer_data *data[2];
	int i, core, ret;

	if (!gxp->core_telemetry_mgr)
		return 0;

	mutex_lock(&gxp->core_telemetry_mgr->lock);
	data[0] = gxp->core_telemetry_mgr->logging_buff_data;
	data[1] = gxp->core_telemetry_mgr->tracing_buff_data;

	for (i = 0; i < ARRAY_SIZE(data); i++) {
		if (!data[i] || !data[i]->is_enabled)
			continue;
		for (core = 0; core < GXP_NUM_CORES; core++) {
			if (!(BIT(core) & core_list))
				continue;
			ret = gxp_dma_map_allocated_coherent_buffer(
				gxp, &data[i]->buffers[core], vd->domain, 0);
			if (ret) {
				dev_err(gxp->dev,
					"Mapping core telemetry buffer to core %d failed",
					core);
				goto error;
			}
		}
	}
	mutex_unlock(&gxp->core_telemetry_mgr->lock);
	return 0;
error:
	while (core--) {
		if (!(BIT(core) & core_list))
			continue;
		gxp_dma_unmap_allocated_coherent_buffer(
			gxp, vd->domain, &data[i]->buffers[core]);
	}
	while (i--) {
		if (!data[i] || !data[i]->is_enabled)
			continue;
		for (core = 0; core < GXP_NUM_CORES; core++) {
			if (!(BIT(core) & core_list))
				continue;
			gxp_dma_unmap_allocated_coherent_buffer(
				gxp, vd->domain, &data[i]->buffers[core]);
		}
	}
	mutex_unlock(&gxp->core_telemetry_mgr->lock);
	return ret;
}

static void unmap_core_telemetry_buffers(struct gxp_dev *gxp,
					 struct gxp_virtual_device *vd,
					 uint core_list)
{
	struct buffer_data *data[2];
	int i, core;

	if (!gxp->core_telemetry_mgr)
		return;
	mutex_lock(&gxp->core_telemetry_mgr->lock);
	data[0] = gxp->core_telemetry_mgr->logging_buff_data;
	data[1] = gxp->core_telemetry_mgr->tracing_buff_data;

	for (i = 0; i < ARRAY_SIZE(data); i++) {
		if (!data[i] || !data[i]->is_enabled)
			continue;
		for (core = 0; core < GXP_NUM_CORES; core++) {
			if (!(BIT(core) & core_list))
				continue;
			gxp_dma_unmap_allocated_coherent_buffer(
				gxp, vd->domain, &data[i]->buffers[core]);
		}
	}
	mutex_unlock(&gxp->core_telemetry_mgr->lock);
}

static int map_debug_dump_buffer(struct gxp_dev *gxp,
				 struct gxp_virtual_device *vd)
{
	if (!gxp->debug_dump_mgr)
		return 0;

	return gxp_dma_map_allocated_coherent_buffer(
		gxp, &gxp->debug_dump_mgr->buf, vd->domain, 0);
}

static void unmap_debug_dump_buffer(struct gxp_dev *gxp,
				    struct gxp_virtual_device *vd)
{
	if (!gxp->debug_dump_mgr)
		return;

	gxp_dma_unmap_allocated_coherent_buffer(gxp, vd->domain,
						&gxp->debug_dump_mgr->buf);
}

static int assign_cores(struct gxp_virtual_device *vd)
{
	struct gxp_dev *gxp = vd->gxp;
	uint core;
	uint available_cores = 0;

	if (!gxp_core_boot(gxp)) {
		/* We don't do core assignment when cores are managed by MCU. */
		vd->core_list = BIT(GXP_NUM_CORES) - 1;
		return 0;
	}
	vd->core_list = 0;
	for (core = 0; core < GXP_NUM_CORES; core++) {
		if (gxp->core_to_vd[core] == NULL) {
			if (available_cores < vd->num_cores)
				vd->core_list |= BIT(core);
			available_cores++;
		}
	}
	if (available_cores < vd->num_cores) {
		dev_err(gxp->dev,
			"Insufficient available cores. Available: %u. Requested: %u\n",
			available_cores, vd->num_cores);
		return -EBUSY;
	}
	for (core = 0; core < GXP_NUM_CORES; core++)
		if (vd->core_list & BIT(core))
			gxp->core_to_vd[core] = vd;
	return 0;
}

static void unassign_cores(struct gxp_virtual_device *vd)
{
	struct gxp_dev *gxp = vd->gxp;
	uint core;

	if (!gxp_core_boot(gxp))
		return;
	for (core = 0; core < GXP_NUM_CORES; core++) {
		if (gxp->core_to_vd[core] == vd)
			gxp->core_to_vd[core] = NULL;
	}
}

/* Saves the state of this VD's doorbells and clears them. */
static void vd_save_doorbells(struct gxp_virtual_device *vd)
{
	struct gxp_dev *gxp = vd->gxp;
	uint base_doorbell;
	uint i;

	if (!gxp_fw_data_use_per_vd_config(vd))
		return;
	base_doorbell = GXP_DOORBELLS_START +
			gxp_vd_hw_slot_id(vd) * GXP_NUM_DOORBELLS_PER_VD;
	for (i = 0; i < ARRAY_SIZE(vd->doorbells_state); i++) {
		vd->doorbells_state[i] =
			gxp_doorbell_status(gxp, base_doorbell + i);
		gxp_doorbell_clear(gxp, base_doorbell + i);
	}
}

/* Restores the state of this VD's doorbells. */
static void vd_restore_doorbells(struct gxp_virtual_device *vd)
{
	struct gxp_dev *gxp = vd->gxp;
	uint base_doorbell;
	uint i;

	if (!gxp_fw_data_use_per_vd_config(vd))
		return;
	base_doorbell = GXP_DOORBELLS_START +
			gxp_vd_hw_slot_id(vd) * GXP_NUM_DOORBELLS_PER_VD;
	for (i = 0; i < ARRAY_SIZE(vd->doorbells_state); i++)
		if (vd->doorbells_state[i])
			gxp_doorbell_set(gxp, base_doorbell + i);
		else
			gxp_doorbell_clear(gxp, base_doorbell + i);
}

static void set_config_version(struct gxp_dev *gxp,
			       struct gxp_virtual_device *vd)
{
	vd->config_version = gxp->fw_loader_mgr->core_img_cfg.config_version;
	/*
	 * Let gxp_dma_map_core_resources() map this region only when using the
	 * legacy protocol.
	 *
	 * TODO(b/265748027): remove this
	 */
	if (gxp_fw_data_use_per_vd_config(vd))
		gxp->fwdatabuf.daddr = 0;
}

static void debug_dump_lock(struct gxp_dev *gxp, struct gxp_virtual_device *vd)
{
	if (!mutex_trylock(&vd->debug_dump_lock)) {
		/*
		 * Release @gxp->vd_semaphore to let other virtual devices proceed
		 * their works and wait for the debug dump to finish.
		 */
		up_write(&gxp->vd_semaphore);
		mutex_lock(&vd->debug_dump_lock);
		down_write(&gxp->vd_semaphore);
	}
}

static inline void debug_dump_unlock(struct gxp_virtual_device *vd)
{
	mutex_unlock(&vd->debug_dump_lock);
}

struct gxp_virtual_device *gxp_vd_allocate(struct gxp_dev *gxp,
					   u16 requested_cores)
{
	struct gxp_virtual_device *vd;
	int i;
	int err;

	lockdep_assert_held_write(&gxp->vd_semaphore);
	/* Assumes 0 < requested_cores <= GXP_NUM_CORES */
	if (requested_cores == 0 || requested_cores > GXP_NUM_CORES)
		return ERR_PTR(-EINVAL);

	vd = kzalloc(sizeof(*vd), GFP_KERNEL);
	if (!vd)
		return ERR_PTR(-ENOMEM);

	vd->gxp = gxp;
	vd->num_cores = requested_cores;
	vd->state = GXP_VD_OFF;
	vd->slice_index = -1;
	vd->client_id = -1;
	vd->tpu_client_id = -1;
	spin_lock_init(&vd->credit_lock);
	refcount_set(&vd->refcount, 1);
	vd->credit = GXP_COMMAND_CREDIT_PER_VD;
	vd->first_open = true;
	vd->vdid = atomic_inc_return(&gxp->next_vdid);
	mutex_init(&vd->fence_list_lock);
	INIT_LIST_HEAD(&vd->gxp_fence_list);
	mutex_init(&vd->debug_dump_lock);

	vd->domain = gxp_domain_pool_alloc(gxp->domain_pool);
	if (!vd->domain) {
		err = -EBUSY;
		goto error_free_vd;
	}

	vd->slice_index = ida_alloc_max(&gxp->shared_slice_idp,
					GXP_NUM_SHARED_SLICES - 1, GFP_KERNEL);
	if (vd->slice_index < 0) {
		err = vd->slice_index;
		goto error_free_domain;
	}

	vd->mailbox_resp_queues = kcalloc(
		vd->num_cores, sizeof(*vd->mailbox_resp_queues), GFP_KERNEL);
	if (!vd->mailbox_resp_queues) {
		err = -ENOMEM;
		goto error_free_slice_index;
	}

	for (i = 0; i < vd->num_cores; i++) {
		INIT_LIST_HEAD(&vd->mailbox_resp_queues[i].wait_queue);
		INIT_LIST_HEAD(&vd->mailbox_resp_queues[i].dest_queue);
		spin_lock_init(&vd->mailbox_resp_queues[i].lock);
		init_waitqueue_head(&vd->mailbox_resp_queues[i].waitq);
	}

	vd->mappings_root = RB_ROOT;
	init_rwsem(&vd->mappings_semaphore);

	err = assign_cores(vd);
	if (err)
		goto error_free_resp_queues;

	/*
	 * Here assumes firmware is requested before allocating a VD, which is
	 * true because we request firmware on first GXP device open.
	 */
	err = map_fw_image_config(gxp, vd, gxp->fw_loader_mgr);
	if (err)
		goto error_unassign_cores;

	set_config_version(gxp, vd);
	/* After map_fw_image_config because it needs vd->vd/core_cfg. */
	gxp_fw_data_populate_vd_cfg(gxp, vd);
	err = gxp_dma_map_core_resources(gxp, vd->domain, vd->core_list,
					 vd->slice_index);
	if (err)
		goto error_unmap_imgcfg;
	err = alloc_and_map_fw_image(gxp, vd);
	if (err)
		goto error_unmap_core_resources;
	err = map_core_telemetry_buffers(gxp, vd, vd->core_list);
	if (err)
		goto error_unmap_fw_data;
	err = map_debug_dump_buffer(gxp, vd);
	if (err)
		goto error_unmap_core_telemetry_buffer;

	return vd;

error_unmap_core_telemetry_buffer:
	unmap_core_telemetry_buffers(gxp, vd, vd->core_list);
error_unmap_fw_data:
	unmap_and_free_fw_image(gxp, vd);
error_unmap_core_resources:
	gxp_dma_unmap_core_resources(gxp, vd->domain, vd->core_list);
error_unmap_imgcfg:
	unmap_fw_image_config(gxp, vd);
error_unassign_cores:
	unassign_cores(vd);
error_free_resp_queues:
	kfree(vd->mailbox_resp_queues);
error_free_slice_index:
	if (vd->slice_index >= 0)
		ida_free(&gxp->shared_slice_idp, vd->slice_index);
error_free_domain:
	gxp_domain_pool_free(gxp->domain_pool, vd->domain);
error_free_vd:
	kfree(vd);

	return ERR_PTR(err);
}

void gxp_vd_release(struct gxp_virtual_device *vd)
{
	struct rb_node *node;
	struct gxp_mapping *mapping;
	struct gxp_dev *gxp = vd->gxp;
	uint core_list = vd->core_list;

	lockdep_assert_held_write(&gxp->vd_semaphore);
	debug_dump_lock(gxp, vd);

	if (vd->is_secure) {
		mutex_lock(&gxp->secure_vd_lock);
		gxp->secure_vd = NULL;
		mutex_unlock(&gxp->secure_vd_lock);
	}

	unmap_debug_dump_buffer(gxp, vd);
	unmap_core_telemetry_buffers(gxp, vd, core_list);
	unmap_and_free_fw_image(gxp, vd);
	gxp_dma_unmap_core_resources(gxp, vd->domain, core_list);
	unmap_fw_image_config(gxp, vd);
	unassign_cores(vd);

	vd->gxp->mailbox_mgr->release_unconsumed_async_resps(vd);

	/*
	 * Release any un-mapped mappings
	 * Once again, it's not necessary to lock the mappings_semaphore here
	 * but do it anyway for consistency.
	 */
	down_write(&vd->mappings_semaphore);
	while ((node = rb_first(&vd->mappings_root))) {
		mapping = rb_entry(node, struct gxp_mapping, node);
		rb_erase(node, &vd->mappings_root);
		gxp_mapping_put(mapping);
	}
	up_write(&vd->mappings_semaphore);

	kfree(vd->mailbox_resp_queues);
	if (vd->slice_index >= 0)
		ida_free(&vd->gxp->shared_slice_idp, vd->slice_index);
	gxp_domain_pool_free(vd->gxp->domain_pool, vd->domain);

	if (vd->invalidate_eventfd)
		gxp_eventfd_put(vd->invalidate_eventfd);
	vd->invalidate_eventfd = NULL;

	vd->state = GXP_VD_RELEASED;
	debug_dump_unlock(vd);
	gxp_vd_put(vd);
}

int gxp_vd_block_ready(struct gxp_virtual_device *vd)
{
	struct gxp_dev *gxp = vd->gxp;
	enum gxp_virtual_device_state orig_state;
	int ret;

	lockdep_assert_held_write(&gxp->vd_semaphore);

	orig_state = vd->state;
	if (orig_state != GXP_VD_OFF && orig_state != GXP_VD_SUSPENDED)
		return -EINVAL;
	ret = gxp_dma_domain_attach_device(gxp, vd->domain, vd->core_list);
	if (ret)
		return ret;
	if (orig_state == GXP_VD_OFF)
		vd->state = GXP_VD_READY;
	if (gxp->after_vd_block_ready) {
		ret = gxp->after_vd_block_ready(gxp, vd);
		if (ret) {
			gxp_dma_domain_detach_device(gxp, vd->domain);
			vd->state = orig_state;
			return ret;
		}
	}
	return 0;
}

void gxp_vd_block_unready(struct gxp_virtual_device *vd)
{
	struct gxp_dev *gxp = vd->gxp;

	lockdep_assert_held_write(&gxp->vd_semaphore);

	if (gxp->before_vd_block_unready)
		gxp->before_vd_block_unready(gxp, vd);
	gxp_dma_domain_detach_device(gxp, vd->domain);
}

int gxp_vd_run(struct gxp_virtual_device *vd)
{
	struct gxp_dev *gxp = vd->gxp;
	int ret;
	enum gxp_virtual_device_state orig_state = vd->state;

	lockdep_assert_held_write(&gxp->vd_semaphore);
	if (orig_state != GXP_VD_READY && orig_state != GXP_VD_OFF)
		return -EINVAL;
	if (orig_state == GXP_VD_OFF) {
		ret = gxp_vd_block_ready(vd);
		/*
		 * The failure of `gxp_vd_block_ready` function means following two things:
		 *
		 * 1. The MCU firmware is not working for some reason and if it was crash,
		 *    @vd->state would be set to UNAVAILABLE by the crash handler. However, by the
		 *    race, if this function holds @gxp->vd_semaphore earlier than that handler,
		 *    it is reasonable to set @vd->state to UNAVAILABLE from here.
		 *
		 * 2. Some information of vd (or client) such as client_id, slice_index are
		 *    incorrect or not allowed by the MCU firmware for some reasons and the
		 *    `allocate_vmbox` or `link_offload_vmbox` has been failed. In this case,
		 *    setting the @vd->state to UNAVAILABLE and letting the runtime close its fd
		 *    and reallocate a vd would be better than setting @vd->state to OFF.
		 *
		 * Therefore, let's set @vd->state to UNAVAILABLE if it returns an error.
		 */
		if (ret)
			goto err_vd_unavailable;
	}

	debug_dump_lock(gxp, vd);
	/* Clear all doorbells */
	vd_restore_doorbells(vd);
	ret = gxp_firmware_run(gxp, vd, vd->core_list);
	if (ret)
		goto err_vd_block_unready;
	vd->state = GXP_VD_RUNNING;
	debug_dump_unlock(vd);

	return 0;

err_vd_block_unready:
	debug_dump_unlock(vd);
	/* Run this only when gxp_vd_block_ready was executed. */
	if (orig_state == GXP_VD_OFF)
		gxp_vd_block_unready(vd);
err_vd_unavailable:
	vd->state = GXP_VD_UNAVAILABLE;
	return ret;
}

/*
 * Caller must hold gxp->vd_semaphore.
 *
 * This function will be called from the `gxp_client_destroy` function if @vd->state is not
 * GXP_VD_OFF.
 *
 * Note for the case of the MCU firmware crahses:
 *
 * In the MCU mode, the `gxp_vd_suspend` function will redirect to this function, but it will not
 * happen when the @vd->state is GXP_VD_UNAVAILABLE. Therefore, if the MCU firmware crashes,
 * @vd->state will be changed to GXP_VD_UNAVAILABLE and this function will not be called even
 * though the runtime is going to release the vd wakelock.
 *
 * It means @vd->state will not be changed to GXP_VD_OFF when the vd wkelock is released (i.e., the
 * state will be kept as GXP_VD_UNAVAILABLE) and when the `gxp_vd_block_unready` function is called
 * by releasing the block wakelock, it will not send `release_vmbox` and `unlink_offload_vmbox` KCI
 * commands to the crashed MCU firmware. This function will be finally called when the runtime
 * closes the fd of the device file.
 */
void gxp_vd_stop(struct gxp_virtual_device *vd)
{
	struct gxp_dev *gxp = vd->gxp;
	uint phys_core;
	uint core_list = vd->core_list;
	uint lpm_state;

	lockdep_assert_held_write(&gxp->vd_semaphore);
	debug_dump_lock(gxp, vd);

	if (gxp_core_boot(gxp) &&
	    (vd->state == GXP_VD_OFF || vd->state == GXP_VD_READY ||
	     vd->state == GXP_VD_RUNNING) &&
	    gxp_pm_get_blk_state(gxp) != AUR_OFF) {

		for (phys_core = 0; phys_core < GXP_NUM_CORES; phys_core++) {
			if (core_list & BIT(phys_core)) {

				lpm_state = gxp_lpm_get_state(gxp, CORE_TO_PSM(phys_core));

				if (lpm_state == LPM_ACTIVE_STATE) {
					/*
					 * If the core is in PS0 (not idle), it should
					 * be held in reset before attempting SW PG.
					 */
					hold_core_in_reset(gxp, phys_core);
				} else {
					/*
					 * If the core is idle and has already transtioned to PS1,
					 * we can attempt HW PG. In this case, we should ensure
					 * that the core doesn't get awakened by an external
					 * interrupt source before we attempt to HW PG the core.
					 */
					gxp_firmware_disable_ext_interrupts(gxp, phys_core);
				}
			}
		}
	}

	gxp_firmware_stop(gxp, vd, core_list);
	if (vd->state != GXP_VD_UNAVAILABLE)
		vd->state = GXP_VD_OFF;

	debug_dump_unlock(vd);
}

static inline uint select_core(struct gxp_virtual_device *vd, uint virt_core,
			       uint phys_core)
{
	return gxp_fw_data_use_per_vd_config(vd) ? virt_core : phys_core;
}

static bool boot_state_is_suspend(struct gxp_dev *gxp,
				  struct gxp_virtual_device *vd, uint core,
				  u32 *boot_state)
{
	if (gxp_fw_data_use_per_vd_config(vd)) {
		*boot_state = gxp_firmware_get_boot_status(gxp, vd, core);
		return *boot_state == GXP_BOOT_STATUS_SUSPENDED;
	}

	*boot_state = gxp_firmware_get_boot_mode(gxp, vd, core);
	return *boot_state == GXP_BOOT_MODE_STATUS_SUSPEND_COMPLETED;
}

static bool boot_state_is_active(struct gxp_dev *gxp,
				 struct gxp_virtual_device *vd, uint core,
				 u32 *boot_state)
{
	if (gxp_fw_data_use_per_vd_config(vd)) {
		*boot_state = gxp_firmware_get_boot_status(gxp, vd, core);
		return *boot_state == GXP_BOOT_STATUS_ACTIVE;
	}

	*boot_state = gxp_firmware_get_boot_mode(gxp, vd, core);
	return *boot_state == GXP_BOOT_MODE_STATUS_RESUME_COMPLETED;
}

/*
 * Caller must have locked `gxp->vd_semaphore` for writing.
 *
 * This function will be called from the `gxp_client_release_vd_wakelock` function when the runtime
 * is going to release the vd wakelock only if the @vd->state is not GXP_VD_UNAVAILABLE.
 *
 * In the MCU mode, this function will redirect to the `gxp_vd_stop` function.
 */
void gxp_vd_suspend(struct gxp_virtual_device *vd)
{
	uint virt_core, phys_core;
	struct gxp_dev *gxp = vd->gxp;
	uint core_list = vd->core_list;
	u32 boot_state;
	uint failed_cores = 0;

	if (!gxp_is_direct_mode(gxp) && gxp_core_boot(gxp))
		return gxp_vd_stop(vd);
	lockdep_assert_held_write(&gxp->vd_semaphore);
	debug_dump_lock(gxp, vd);

	dev_info(gxp->dev, "Suspending VD vdid=%d client_id=%d...\n", vd->vdid,
		 vd->client_id);
	if (vd->state == GXP_VD_SUSPENDED) {
		dev_err(gxp->dev,
			"Attempt to suspend a virtual device twice\n");
		goto out;
	}
	if (!gxp_core_boot(gxp)) {
		vd->state = GXP_VD_SUSPENDED;
		goto out;
	}
	gxp_pm_force_clkmux_normal(gxp);
	/*
	 * Start the suspend process for all of this VD's cores without waiting
	 * for completion.
	 */
	virt_core = 0;
	for (phys_core = 0; phys_core < GXP_NUM_CORES; phys_core++) {
		uint core = select_core(vd, virt_core, phys_core);

		if (!(core_list & BIT(phys_core)))
			continue;
		if (!gxp_lpm_wait_state_ne(gxp, CORE_TO_PSM(phys_core),
					   LPM_ACTIVE_STATE)) {
			vd->state = GXP_VD_UNAVAILABLE;
			failed_cores |= BIT(phys_core);
			hold_core_in_reset(gxp, phys_core);
			dev_err(gxp->dev, "Core %u stuck at LPM_ACTIVE_STATE",
				phys_core);
			continue;
		}
		/* Mark the boot mode as a suspend event */
		if (gxp_fw_data_use_per_vd_config(vd)) {
			gxp_firmware_set_boot_status(gxp, vd, core,
						     GXP_BOOT_STATUS_NONE);
			gxp_firmware_set_boot_mode(gxp, vd, core,
						   GXP_BOOT_MODE_SUSPEND);
		} else {
			gxp_firmware_set_boot_mode(
				gxp, vd, core, GXP_BOOT_MODE_REQUEST_SUSPEND);
		}
		/*
		 * Request a suspend event by sending a mailbox
		 * notification.
		 */
		gxp_notification_send(gxp, phys_core,
				      CORE_NOTIF_SUSPEND_REQUEST);
		virt_core++;
	}
	/* Wait for all cores to complete core suspension. */
	virt_core = 0;
	for (phys_core = 0; phys_core < GXP_NUM_CORES; phys_core++) {
		uint core = select_core(vd, virt_core, phys_core);

		if (!(core_list & BIT(phys_core)))
			continue;
		virt_core++;
		if (failed_cores & BIT(phys_core))
			continue;
		if (!gxp_lpm_wait_state_eq(gxp, CORE_TO_PSM(phys_core),
					   LPM_PG_STATE)) {
			if (!boot_state_is_suspend(gxp, vd, core,
						   &boot_state)) {
				dev_err(gxp->dev,
					"Suspension request on core %u failed (status: %u)",
					phys_core, boot_state);
				vd->state = GXP_VD_UNAVAILABLE;
				failed_cores |= BIT(phys_core);
				hold_core_in_reset(gxp, phys_core);
			}
		} else {
			/* Re-set PS1 as the default low power state. */
			gxp_lpm_enable_state(gxp, CORE_TO_PSM(phys_core),
					     LPM_CG_STATE);
		}
	}
	if (vd->state == GXP_VD_UNAVAILABLE) {
		/* shutdown all cores if virtual device is unavailable */
		for (phys_core = 0; phys_core < GXP_NUM_CORES; phys_core++)
			if (core_list & BIT(phys_core))
				gxp_pm_core_off(gxp, phys_core);
	} else {
		/* Save and clear all doorbells. */
		vd_save_doorbells(vd);
		vd->blk_switch_count_when_suspended =
			gxp_pm_get_blk_switch_count(gxp);
		vd->state = GXP_VD_SUSPENDED;
	}
	gxp_pm_resume_clkmux(gxp);
out:
	debug_dump_unlock(vd);
}

/*
 * Caller must have locked `gxp->vd_semaphore` for writing.
 */
int gxp_vd_resume(struct gxp_virtual_device *vd)
{
	int ret = 0;
	uint phys_core, virt_core;
	uint core_list = vd->core_list;
	uint timeout;
	u32 boot_state;
	struct gxp_dev *gxp = vd->gxp;
	u64 curr_blk_switch_count;
	uint failed_cores = 0;

	lockdep_assert_held_write(&gxp->vd_semaphore);
	debug_dump_lock(gxp, vd);
	dev_info(gxp->dev, "Resuming VD vdid=%d client_id=%d...\n", vd->vdid,
		 vd->client_id);
	if (vd->state != GXP_VD_SUSPENDED) {
		dev_err(gxp->dev,
			"Attempt to resume a virtual device which was not suspended\n");
		ret = -EBUSY;
		goto out;
	}
	if (!gxp_core_boot(gxp)) {
		vd->state = GXP_VD_RUNNING;
		goto out;
	}
	gxp_pm_force_clkmux_normal(gxp);
	curr_blk_switch_count = gxp_pm_get_blk_switch_count(gxp);

	/* Restore the doorbells state for this VD. */
	vd_restore_doorbells(vd);

	/*
	 * Start the resume process for all of this VD's cores without waiting
	 * for completion.
	 */
	virt_core = 0;
	for (phys_core = 0; phys_core < GXP_NUM_CORES; phys_core++) {
		uint core = select_core(vd, virt_core, phys_core);

		if (!(core_list & BIT(phys_core)))
			continue;
		/*
		 * The comparison is to check if blk_switch_count is
		 * changed. If it's changed, it means the block is rebooted and
		 * therefore we need to set up the hardware again.
		 */
		if (vd->blk_switch_count_when_suspended !=
		    curr_blk_switch_count) {
			ret = gxp_firmware_setup_hw_after_block_off(
				gxp, core, phys_core,
				/*verbose=*/false);
			if (ret) {
				vd->state = GXP_VD_UNAVAILABLE;
				failed_cores |= BIT(phys_core);
				dev_err(gxp->dev,
					"Failed to power up core %u\n",
					phys_core);
				continue;
			}
		}
		/* Mark this as a resume power-up event. */
		if (gxp_fw_data_use_per_vd_config(vd)) {
			gxp_firmware_set_boot_status(gxp, vd, core,
						     GXP_BOOT_STATUS_NONE);
			gxp_firmware_set_boot_mode(gxp, vd, core,
						   GXP_BOOT_MODE_RESUME);
		} else {
			gxp_firmware_set_boot_mode(
				gxp, vd, core, GXP_BOOT_MODE_REQUEST_RESUME);
		}
		/*
		 * Power on the core by explicitly switching its PSM to
		 * PS0 (LPM_ACTIVE_STATE).
		 */
		gxp_lpm_set_state(gxp, CORE_TO_PSM(phys_core), LPM_ACTIVE_STATE,
				  /*verbose=*/false);
		virt_core++;
	}
	/* Wait for all cores to complete core resumption. */
	virt_core = 0;
	for (phys_core = 0; phys_core < GXP_NUM_CORES; phys_core++) {
		uint core = select_core(vd, virt_core, phys_core);

		if (!(core_list & BIT(phys_core)))
			continue;

		if (!(failed_cores & BIT(phys_core))) {
			/* in microseconds */
			timeout = 1000000;
			while (--timeout) {
				if (boot_state_is_active(gxp, vd, core,
							 &boot_state))
					break;
				udelay(1 * GXP_TIME_DELAY_FACTOR);
			}
			if (timeout == 0) {
				dev_err(gxp->dev,
					"Resume request on core %u failed (status: %u)",
					phys_core, boot_state);
				ret = -EBUSY;
				vd->state = GXP_VD_UNAVAILABLE;
				failed_cores |= BIT(phys_core);
			}
		}
		virt_core++;
	}
	if (vd->state == GXP_VD_UNAVAILABLE) {
		/* shutdown all cores if virtual device is unavailable */
		for (phys_core = 0; phys_core < GXP_NUM_CORES; phys_core++) {
			if (core_list & BIT(phys_core))
				gxp_pm_core_off(gxp, phys_core);
		}
	} else {
		vd->state = GXP_VD_RUNNING;
	}
	gxp_pm_resume_clkmux(gxp);
out:
	debug_dump_unlock(vd);
	return ret;
}

/* Caller must have locked `gxp->vd_semaphore` for reading */
int gxp_vd_virt_core_to_phys_core(struct gxp_virtual_device *vd, u16 virt_core)
{
	struct gxp_dev *gxp = vd->gxp;
	uint phys_core;
	uint virt_core_index = 0;

	for (phys_core = 0; phys_core < GXP_NUM_CORES; phys_core++) {
		if (vd->core_list & BIT(phys_core)) {
			if (virt_core_index == virt_core)
				return phys_core;

			virt_core_index++;
		}
	}

	dev_dbg(gxp->dev, "No mapping for virtual core %u\n", virt_core);
	return -EINVAL;
}

int gxp_vd_mapping_store(struct gxp_virtual_device *vd, struct gxp_mapping *map)
{
	struct rb_node **link;
	struct rb_node *parent = NULL;
	dma_addr_t device_address = map->device_address;
	struct gxp_mapping *mapping;

	link = &vd->mappings_root.rb_node;

	down_write(&vd->mappings_semaphore);

	/* Figure out where to put the new node */
	while (*link) {
		parent = *link;
		mapping = rb_entry(parent, struct gxp_mapping, node);

		if (mapping->device_address > device_address)
			link = &(*link)->rb_left;
		else if (mapping->device_address < device_address)
			link = &(*link)->rb_right;
		else
			goto out;
	}

	/* Add new node and rebalance the tree. */
	rb_link_node(&map->node, parent, link);
	rb_insert_color(&map->node, &vd->mappings_root);

	/* Acquire a reference to the mapping */
	gxp_mapping_get(map);

	up_write(&vd->mappings_semaphore);

	return 0;

out:
	up_write(&vd->mappings_semaphore);
	dev_err(vd->gxp->dev, "Duplicate mapping: %pad\n",
		&map->device_address);
	return -EEXIST;
}

void gxp_vd_mapping_remove(struct gxp_virtual_device *vd,
			   struct gxp_mapping *map)
{
	down_write(&vd->mappings_semaphore);
	gxp_vd_mapping_remove_locked(vd, map);
	up_write(&vd->mappings_semaphore);
}

void gxp_vd_mapping_remove_locked(struct gxp_virtual_device *vd, struct gxp_mapping *map)
{
	lockdep_assert_held_write(&vd->mappings_semaphore);

	/* Drop the mapping from this virtual device's records */
	rb_erase(&map->node, &vd->mappings_root);

	/* Release the reference obtained in gxp_vd_mapping_store() */
	gxp_mapping_put(map);
}

static bool is_device_address_in_mapping(struct gxp_mapping *mapping,
					 dma_addr_t device_address)
{
	return ((device_address >= mapping->device_address) &&
		(device_address < (mapping->device_address + mapping->size)));
}

static struct gxp_mapping *
gxp_vd_mapping_internal_search(struct gxp_virtual_device *vd,
			       dma_addr_t device_address, bool check_range)
{
	struct rb_node *node;
	struct gxp_mapping *mapping;

	lockdep_assert_held(&vd->mappings_semaphore);

	node = vd->mappings_root.rb_node;

	while (node) {
		mapping = rb_entry(node, struct gxp_mapping, node);
		if ((mapping->device_address == device_address) ||
		    (check_range &&
		     is_device_address_in_mapping(mapping, device_address))) {
			gxp_mapping_get(mapping);
			return mapping; /* Found it */
		} else if (mapping->device_address > device_address) {
			node = node->rb_left;
		} else {
			node = node->rb_right;
		}
	}

	return NULL;
}

struct gxp_mapping *gxp_vd_mapping_search(struct gxp_virtual_device *vd,
					  dma_addr_t device_address)
{
	struct gxp_mapping *mapping;

	down_read(&vd->mappings_semaphore);
	mapping = gxp_vd_mapping_search_locked(vd, device_address);
	up_read(&vd->mappings_semaphore);

	return mapping;
}

struct gxp_mapping *gxp_vd_mapping_search_locked(struct gxp_virtual_device *vd,
						 dma_addr_t device_address)
{
	return gxp_vd_mapping_internal_search(vd, device_address, false);
}

struct gxp_mapping *
gxp_vd_mapping_search_in_range(struct gxp_virtual_device *vd,
			       dma_addr_t device_address)
{
	struct gxp_mapping *mapping;

	down_read(&vd->mappings_semaphore);
	mapping = gxp_vd_mapping_internal_search(vd, device_address, true);
	up_read(&vd->mappings_semaphore);

	return mapping;
}

struct gxp_mapping *gxp_vd_mapping_search_host(struct gxp_virtual_device *vd,
					       u64 host_address)
{
	struct rb_node *node;
	struct gxp_mapping *mapping;

	/*
	 * dma-buf mappings can not be looked-up by host address since they are
	 * not mapped from a user-space address.
	 */
	if (!host_address) {
		dev_dbg(vd->gxp->dev,
			"Unable to get dma-buf mapping by host address\n");
		return NULL;
	}

	down_read(&vd->mappings_semaphore);

	/* Iterate through the elements in the rbtree */
	for (node = rb_first(&vd->mappings_root); node; node = rb_next(node)) {
		mapping = rb_entry(node, struct gxp_mapping, node);
		if (mapping->host_address == host_address) {
			gxp_mapping_get(mapping);
			up_read(&vd->mappings_semaphore);
			return mapping;
		}
	}

	up_read(&vd->mappings_semaphore);

	return NULL;
}

bool gxp_vd_has_and_use_credit(struct gxp_virtual_device *vd)
{
	bool ret = true;
	unsigned long flags;

	spin_lock_irqsave(&vd->credit_lock, flags);
	if (vd->credit == 0)
		ret = false;
	else
		vd->credit--;
	spin_unlock_irqrestore(&vd->credit_lock, flags);

	return ret;
}

void gxp_vd_release_credit(struct gxp_virtual_device *vd)
{
	unsigned long flags;

	spin_lock_irqsave(&vd->credit_lock, flags);
	if (unlikely(vd->credit >= GXP_COMMAND_CREDIT_PER_VD))
		dev_err(vd->gxp->dev, "unbalanced VD credit");
	else
		vd->credit++;
	spin_unlock_irqrestore(&vd->credit_lock, flags);
}

void gxp_vd_put(struct gxp_virtual_device *vd)
{
	if (!vd)
		return;
	if (refcount_dec_and_test(&vd->refcount))
		kfree(vd);
}

void gxp_vd_invalidate_with_client_id(struct gxp_dev *gxp, int client_id,
				      uint core_list)
{
	struct gxp_client *client = NULL, *c;
	uint core;

	/*
	 * Prevent @gxp->client_list is being changed while handling the crash.
	 * The user cannot open or close an FD until this function releases the lock.
	 */
	mutex_lock(&gxp->client_list_lock);

	/*
	 * Find corresponding vd with client_id.
	 * If it holds a block wakelock, we should discard all pending/unconsumed UCI responses
	 * and change the state of the vd to GXP_VD_UNAVAILABLE.
	 */
	list_for_each_entry (c, &gxp->client_list, list_entry) {
		down_write(&c->semaphore);
		down_write(&gxp->vd_semaphore);
		if (c->vd && c->vd->client_id == client_id) {
			client = c;
			break;
		}
		up_write(&gxp->vd_semaphore);
		up_write(&c->semaphore);
	}

	mutex_unlock(&gxp->client_list_lock);

	if (!client) {
		dev_err(gxp->dev, "Failed to find a VD, client_id=%d",
			client_id);
		/*
		 * Invalidate all debug dump segments if debug dump
		 * is enabled and core_list is not empty.
		 */
		if (!gxp_debug_dump_is_enabled() || !core_list)
			return;
		for (core = 0; core < GXP_NUM_CORES; core++) {
			if (!(BIT(core) & core_list))
				continue;
			mutex_lock(&gxp->debug_dump_mgr->debug_dump_lock);
			gxp_debug_dump_invalidate_segments(gxp, core);
			mutex_unlock(&gxp->debug_dump_mgr->debug_dump_lock);
		}
		return;
	}

	gxp_vd_invalidate(gxp, client->vd);
	gxp_vd_generate_debug_dump(gxp, client->vd, core_list);

	up_write(&gxp->vd_semaphore);
	up_write(&client->semaphore);
}

void gxp_vd_invalidate(struct gxp_dev *gxp, struct gxp_virtual_device *vd)
{
	lockdep_assert_held_write(&gxp->vd_semaphore);

	dev_err(gxp->dev, "Invalidate a VD, VDID=%d, client_id=%d", vd->vdid,
		vd->client_id);

	if (vd->state != GXP_VD_UNAVAILABLE) {
		if (gxp->mailbox_mgr->release_unconsumed_async_resps)
			gxp->mailbox_mgr->release_unconsumed_async_resps(vd);

		vd->state = GXP_VD_UNAVAILABLE;

		if (vd->invalidate_eventfd)
			gxp_eventfd_signal(vd->invalidate_eventfd);
	} else {
		dev_dbg(gxp->dev, "This VD is already invalidated");
	}
}

void gxp_vd_generate_debug_dump(struct gxp_dev *gxp,
				struct gxp_virtual_device *vd, uint core_list)
{
	int ret;

	if (!gxp_debug_dump_is_enabled() || !core_list)
		return;

	lockdep_assert_held_write(&gxp->vd_semaphore);

	/*
	 * We should increase the refcount of @vd because @gxp->vd_semaphore will be
	 * released below and the client can release it asynchronously.
	 */
	vd = gxp_vd_get(vd);

	/*
	 * Release @gxp->vd_semaphore before generating a debug dump and hold it
	 * again after completing debug dump to not block other virtual devices
	 * proceeding their work.
	 */
	up_write(&gxp->vd_semaphore);
	mutex_lock(&vd->debug_dump_lock);

	/*
	 * Process debug dump if its enabled and core_list is not empty.
	 * Keep on hold the client lock while processing the dumps. vd
	 * lock would be taken and released inside the debug dump
	 * implementation logic ahead.
	 */
	ret = gxp_debug_dump_process_dump_mcu_mode(gxp, core_list, vd);
	if (ret)
		dev_err(gxp->dev, "debug dump processing failed (ret=%d).\n",
			ret);

	mutex_unlock(&vd->debug_dump_lock);
	down_write(&gxp->vd_semaphore);
	gxp_vd_put(vd);
}
