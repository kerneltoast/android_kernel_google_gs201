// SPDX-License-Identifier: GPL-2.0-only
/*
 * Support for using dma-bufs.
 *
 * Copyright (C) 2022 Google LLC
 */

#include <linux/dma-buf.h>
#include <linux/module.h>
#include <linux/slab.h>

#include <gcip/gcip-config.h>
#include <gcip/gcip-iommu-reserve.h>
#include <gcip/gcip-iommu.h>

#include "gxp-dma.h"
#include "gxp-dmabuf.h"

#include <trace/events/gxp.h>

/* Mapping destructor for gxp_mapping_put() to call */
static void destroy_dmabuf_mapping(struct gxp_mapping *mapping)
{
	dma_addr_t device_address = mapping->gcip_mapping->device_address;
	size_t size = mapping->gcip_mapping->size;

	trace_gxp_dmabuf_mapping_destroy_start(device_address, size);

	gcip_iommu_mapping_unmap(mapping->gcip_mapping);
	kfree(mapping);

	trace_gxp_dmabuf_mapping_destroy_end(device_address, size);
}

struct gxp_mapping *gxp_dmabuf_map(struct gxp_dev *gxp, struct gcip_iommu_reserve_manager *mgr,
				   struct gcip_iommu_domain *domain, int fd, u32 flags,
				   dma_addr_t iova_hint)
{
	struct gxp_mapping *mapping;
	struct gcip_iommu_mapping *gcip_mapping;
	struct dma_buf *dmabuf;
	u64 gcip_map_flags;
	int ret;

	trace_gxp_dmabuf_mapping_create_start(fd);

	dmabuf = dma_buf_get(fd);
	if (IS_ERR(dmabuf))
		return ERR_CAST(dmabuf);

	/* Skip CPU cache syncs while mapping this dmabuf. */
	gcip_map_flags = gxp_dma_encode_gcip_map_flags(flags, 0) |
			 GCIP_MAP_FLAGS_DMA_ATTR_TO_FLAGS(DMA_ATTR_SKIP_CPU_SYNC);

	mapping = kzalloc(sizeof(*mapping), GFP_KERNEL);
	if (!mapping) {
		ret = -ENOMEM;
		goto err_dma_buf_put;
	}

	if (!iova_hint)
		gcip_mapping = gcip_iommu_domain_map_dma_buf(domain, dmabuf, gcip_map_flags);
	else
		gcip_mapping = gcip_iommu_reserve_map_dma_buf(mgr, dmabuf, gcip_map_flags,
							      iova_hint, mapping);
	if (IS_ERR(gcip_mapping)) {
		ret = PTR_ERR(gcip_mapping);
		dev_err(gxp->dev, "Failed to map dma-buf (ret=%d)\n", ret);
		goto err_free_mapping;
	}

	dma_buf_put(dmabuf);

	/* dma-buf mappings are indicated by a host_address of 0 */
	mapping->host_address = 0;
	mapping->gcip_mapping = gcip_mapping;
	mapping->destructor = destroy_dmabuf_mapping;
	mapping->gxp = gxp;
	refcount_set(&mapping->refcount, 1);

	trace_gxp_dmabuf_mapping_create_end(gcip_mapping->device_address, gcip_mapping->size);

	return mapping;

err_free_mapping:
	kfree(mapping);
err_dma_buf_put:
	dma_buf_put(dmabuf);
	return ERR_PTR(ret);
}

MODULE_IMPORT_NS(DMA_BUF);
