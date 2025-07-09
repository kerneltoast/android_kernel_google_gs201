/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Support for using dma-bufs.
 *
 * Copyright (C) 2022 Google LLC
 */
#ifndef __GXP_DMABUF_H__
#define __GXP_DMABUF_H__

#include <linux/iommu.h>
#include <linux/types.h>

#include "gxp-internal.h"
#include "gxp-mapping.h"

/**
 * gxp_dmabuf_map() - Map a dma-buf for access by the specified virtual device
 * @gxp: The GXP device to map the dma-buf for
 * @mgr: The manager of reserved IOVA regions. It can be NULL, if the buffer is going to be mapped
 *       to the non-reserved region (i.e., @iova_hint is 0).
 * @domain: The iommu domain the dma-buf is mapped for
 * @fd: A file descriptor for the dma-buf to be mapped
 * @flags: The flags passed from ioctl by user which contains direction and coherent info.
 * @iova_hint: If non-zero, the dma-buf will be mapped to the specific IOVA address indicated via
 *             this param. A region which can cover the dma-buf must be pre-reserved from @mgr.
 *
 * If successful, the mapping will be initialized with a reference count of 1
 *
 * Return: The structure that was created and is being tracked to describe the
 *         mapping of the dma-buf. Returns ERR_PTR on failure.
 */
struct gxp_mapping *gxp_dmabuf_map(struct gxp_dev *gxp, struct gcip_iommu_reserve_manager *mgr,
				   struct gcip_iommu_domain *domain, int fd, u32 flags,
				   dma_addr_t iova_hint);

/**
 * gxp_dmabuf_get_sgt(): Retrieve the sg table for dmabuf
 * @mapping: The mapping corresponding to dmabuf
 *
 * Return: sg table on success or NULL.
 */
struct sg_table *gxp_dmabuf_get_sgt(struct gxp_mapping *mapping);

#endif /* __GXP_DMABUF_H__ */
