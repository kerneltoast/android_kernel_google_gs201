/* SPDX-License-Identifier: GPL-2.0 */
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
 * @domain: The iommu domain the dma-buf is mapped for
 * @fd: A file descriptor for the dma-buf to be mapped
 * @flags: The type of mapping to create; Currently unused
 * @direction: DMA direction
 *
 * If successful, the mapping will be initialized with a reference count of 1
 *
 * Return: The structure that was created and is being tracked to describe the
 *         mapping of the dma-buf. Returns ERR_PTR on failure.
 */
struct gxp_mapping *gxp_dmabuf_map(struct gxp_dev *gxp,
				   struct gxp_iommu_domain *domain, int fd,
				   u32 flags, enum dma_data_direction dir);

#endif /* __GXP_DMABUF_H__ */
