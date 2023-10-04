// SPDX-License-Identifier: GPL-2.0
/*
 * GXP IOMMU domain allocator.
 *
 * Copyright (C) 2022 Google LLC
 */

#include <linux/idr.h>
#include <linux/iommu.h>
#include <linux/slab.h>

#include "gxp-domain-pool.h"
#include "gxp-internal.h"

int gxp_domain_pool_init(struct gxp_dev *gxp, struct gxp_domain_pool *pool,
			 unsigned int size)
{
	unsigned int i;
	struct iommu_domain *domain;

	pool->size = size;
	pool->gxp = gxp;

	if (!size)
		return 0;

	dev_dbg(pool->gxp->dev, "Initializing domain pool with %u domains\n", size);

	ida_init(&pool->idp);
	pool->array = vzalloc(sizeof(*pool->array) * size);
	if (!pool->array) {
		dev_err(gxp->dev, "Failed to allocate memory for domain pool array\n");
		return -ENOMEM;
	}
	for (i = 0; i < size; i++) {
		domain = iommu_domain_alloc(pool->gxp->dev->bus);
		if (!domain) {
			dev_err(pool->gxp->dev,
				"Failed to allocate iommu domain %d of %u\n",
				i + 1, size);
			gxp_domain_pool_destroy(pool);
			return -ENOMEM;
		}
		pool->array[i] = domain;
	}
	return 0;
}

struct iommu_domain *gxp_domain_pool_alloc(struct gxp_domain_pool *pool)
{
	int id;

	if (!pool->size)
		return iommu_domain_alloc(pool->gxp->dev->bus);

	id = ida_alloc_max(&pool->idp, pool->size - 1, GFP_KERNEL);

	if (id < 0) {
		dev_err(pool->gxp->dev,
			"No more domains available from pool of size %u\n",
			pool->size);
		return NULL;
	}

	dev_dbg(pool->gxp->dev, "Allocated domain from pool with id = %d\n", id);

	return pool->array[id];
}

void gxp_domain_pool_free(struct gxp_domain_pool *pool, struct iommu_domain *domain)
{
	int id;

	if (!pool->size) {
		iommu_domain_free(domain);
		return;
	}
	for (id = 0; id < pool->size; id++) {
		if (pool->array[id] == domain) {
			dev_dbg(pool->gxp->dev, "Released domain from pool with id = %d\n", id);
			ida_free(&pool->idp, id);
			return;
		}
	}
	dev_err(pool->gxp->dev, "%s: domain not found in pool", __func__);
}

void gxp_domain_pool_destroy(struct gxp_domain_pool *pool)
{
	int i;

	if (!pool->size)
		return;

	dev_dbg(pool->gxp->dev, "Destroying domain pool with %u domains\n", pool->size);

	for (i = 0; i < pool->size; i++) {
		if (pool->array[i])
			iommu_domain_free(pool->array[i]);
	}

	ida_destroy(&pool->idp);
	vfree(pool->array);
}
