/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * IOMMU domain allocator for gxp
 *
 * Copyright (C) 2022 Google LLC
 */

#ifndef __GXP_DOMAIN_POOL_H__
#define __GXP_DOMAIN_POOL_H__

#include <gcip/gcip-iommu.h>

#include "gxp-dma.h"

/* Default number of PASIDs if it's not configured. */
#define GXP_DEFAULT_NUM_PASIDS 8

/*
 * Initializes a domain pool.
 *
 * @gxp: pointer to gxp device.
 * @pool: caller-allocated pool structure.
 * @size: size of the pre-allocated domains pool.
 * Set to zero to fall back to dynamically allocated domains.
 *
 * returns 0 on success or negative error value.
 */
int gxp_domain_pool_init(struct gxp_dev *gxp,
			 struct gcip_iommu_domain_pool *pool,
			 unsigned int size);

/*
 * Allocates a domain from the pool
 * returns NULL on error.
 */
struct gcip_iommu_domain *
gxp_domain_pool_alloc(struct gcip_iommu_domain_pool *pool);

/* Releases a domain from the pool. */
void gxp_domain_pool_free(struct gcip_iommu_domain_pool *pool,
			  struct gcip_iommu_domain *gdomain);

/* Cleans up all resources used by the domain pool. */
void gxp_domain_pool_destroy(struct gcip_iommu_domain_pool *pool);
#endif /* __GXP_DOMAIN_POOL_H__ */
