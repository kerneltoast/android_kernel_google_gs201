/* SPDX-License-Identifier: GPL-2.0 */
/*
 * IOMMU domain allocator for edgetpu
 *
 * Copyright (C) 2022 Google LLC
 */

#ifndef __EDGETPU_DOMAIN_POOL_H__
#define __EDGETPU_DOMAIN_POOL_H__

#include <linux/idr.h>
#include <linux/iommu.h>

#include "edgetpu-internal.h"

struct edgetpu_domain_pool {
	struct ida idp;			/* ID allocator to keep track of used domains. */
	/*
	 * Size of the pool. Can be set to 0, in which case the implementation will fall back to
	 * dynamic domain allocation using the IOMMU API directly.
	 */
	unsigned int size;
	struct iommu_domain **array;	/* Array holding the pointers to pre-allocated domains. */
	struct edgetpu_dev *etdev;	/* The edgetpu device used for logging warnings/errors. */
};


/*
 * Initializes a domain pool.
 *
 * @etdev: pointer to edgeptu device.
 * @pool: caller-allocated pool structure.
 * @size: size of the pre-allocated domains pool.
 * Set to zero to fall back to dynamically allocated domains.
 *
 * returns 0 on success or negative error value.
 */
int edgetpu_domain_pool_init(struct edgetpu_dev *etdev, struct edgetpu_domain_pool *pool,
			     unsigned int size);

/*
 * Allocates a domain from the pool
 * returns NULL on error.
 */
struct iommu_domain *edgetpu_domain_pool_alloc(struct edgetpu_domain_pool *pool);

/* Releases a domain from the pool. */
void edgetpu_domain_pool_free(struct edgetpu_domain_pool *pool, struct iommu_domain *domain);

/* Cleans up all resources used by the domain pool. */
void edgetpu_domain_pool_destroy(struct edgetpu_domain_pool *pool);

#endif /* __EDGETPU_DOMAIN_POOL_H__ */
