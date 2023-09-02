/* SPDX-License-Identifier: GPL-2.0 */
/*
 * IOMMU domain allocator for gxp
 *
 * Copyright (C) 2022 Google LLC
 */

#ifndef __GXP_DOMAIN_POOL_H__
#define __GXP_DOMAIN_POOL_H__

#include <linux/idr.h>
#include <linux/iommu.h>

#include "gxp-internal.h"

struct gxp_domain_pool {
	struct ida idp;			/* ID allocator to keep track of used domains. */
	/*
	 * Size of the pool. Can be set to 0, in which case the implementation will fall back to
	 * dynamic domain allocation using the IOMMU API directly.
	 */
	unsigned int size;
	struct iommu_domain **array;	/* Array holding the pointers to pre-allocated domains. */
	struct gxp_dev *gxp;	/* The gxp device used for logging warnings/errors. */
};


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
int gxp_domain_pool_init(struct gxp_dev *gxp, struct gxp_domain_pool *pool,
			 unsigned int size);

/*
 * Allocates a domain from the pool
 * returns NULL on error.
 */
struct iommu_domain *gxp_domain_pool_alloc(struct gxp_domain_pool *pool);

/* Releases a domain from the pool. */
void gxp_domain_pool_free(struct gxp_domain_pool *pool, struct iommu_domain *domain);

/* Cleans up all resources used by the domain pool. */
void gxp_domain_pool_destroy(struct gxp_domain_pool *pool);

#endif /* __GXP_DOMAIN_POOL_H__ */
