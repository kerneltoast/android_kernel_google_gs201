/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * GCIP IOMMU domain allocator.
 *
 * Copyright (C) 2022 Google LLC
 */

#ifndef __GCIP_DOMAIN_POOL_H__
#define __GCIP_DOMAIN_POOL_H__

#include <linux/device.h>
#include <linux/idr.h>
#include <linux/iommu.h>
#include <linux/mutex.h>
#include <linux/types.h>

struct gcip_domain_pool {
	struct ida idp; /* ID allocator to keep track of used domains. */
	/*
	 * Size of the pool. Can be set to 0, in which case the implementation will fall back to
	 * dynamic domain allocation using the IOMMU API directly.
	 */
	unsigned int size;
	struct iommu_domain **array; /* Array holding the pointers to pre-allocated domains. */
	struct device *dev; /* The device used for logging warnings/errors. */
	struct list_head dynamic_domains; /* Tracks dynamically allocated domains for cleanup. */
	struct mutex lock; /* Protects dynamic_domains. */
};

/*
 * Initializes a domain pool.
 *
 * @dev: pointer to device structure.
 * @pool: caller-allocated pool structure.
 * @size: size of the pre-allocated domains pool.
 * Set to zero to fall back to dynamically allocated domains.
 *
 * returns 0 on success or negative error value.
 */
int gcip_domain_pool_init(struct device *dev, struct gcip_domain_pool *pool, unsigned int size);

/*
 * Allocates a domain from the pool
 * returns NULL on error.
 */
struct iommu_domain *gcip_domain_pool_alloc(struct gcip_domain_pool *pool);

/* Releases a domain from the pool. */
void gcip_domain_pool_free(struct gcip_domain_pool *pool, struct iommu_domain *domain);

/* Cleans up all resources used by the domain pool. */
void gcip_domain_pool_destroy(struct gcip_domain_pool *pool);

#endif /* __GCIP_DOMAIN_POOL_H__ */
