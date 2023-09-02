// SPDX-License-Identifier: GPL-2.0
/*
 * EdgeTPU IOMMU domain allocator.
 *
 * Copyright (C) 2022 Google, LLC.
 */

#include <linux/idr.h>
#include <linux/iommu.h>
#include <linux/slab.h>

#include "edgetpu-domain-pool.h"
#include "edgetpu-internal.h"

int edgetpu_domain_pool_init(struct edgetpu_dev *etdev, struct edgetpu_domain_pool *pool,
			     unsigned int size)
{
	unsigned int i;
	struct iommu_domain *domain;

	pool->size = size;
	pool->etdev = etdev;

	if (!size)
		return 0;

	etdev_dbg(pool->etdev, "Initializing domain pool with %u domains\n", size);

	ida_init(&pool->idp);
	pool->array = vzalloc(sizeof(*pool->array) * size);
	if (!pool->array) {
		etdev_err(etdev, "Failed to allocate memory for domain pool array\n");
		return -ENOMEM;
	}
	for (i = 0; i < size; i++) {
		domain = iommu_domain_alloc(pool->etdev->dev->bus);
		if (!domain) {
			etdev_err(pool->etdev, "Failed to allocate iommu domain %d of %u\n", i + 1,
				  size);
			edgetpu_domain_pool_destroy(pool);
			return -ENOMEM;
		}
		pool->array[i] = domain;
	}
	return 0;
}

struct iommu_domain *edgetpu_domain_pool_alloc(struct edgetpu_domain_pool *pool)
{
	int id;

	if (!pool->size)
		return iommu_domain_alloc(pool->etdev->dev->bus);

	id = ida_alloc_max(&pool->idp, pool->size - 1, GFP_KERNEL);

	if (id < 0) {
		etdev_err(pool->etdev, "No more domains available from pool of size %u\n",
			  pool->size);
		return NULL;
	}

	etdev_dbg(pool->etdev, "Allocated domain from pool with id = %d\n", id);

	return pool->array[id];
}

void edgetpu_domain_pool_free(struct edgetpu_domain_pool *pool, struct iommu_domain *domain)
{
	int id;

	if (!pool->size) {
		iommu_domain_free(domain);
		return;
	}
	for (id = 0; id < pool->size; id++) {
		if (pool->array[id] == domain) {
			etdev_dbg(pool->etdev, "Released domain from pool with id = %d\n", id);
			ida_free(&pool->idp, id);
			return;
		}
	}
	etdev_err(pool->etdev, "%s: domain not found in pool", __func__);
}

void edgetpu_domain_pool_destroy(struct edgetpu_domain_pool *pool)
{
	int i;

	if (!pool->size)
		return;

	etdev_dbg(pool->etdev, "Destroying domain pool with %u domains\n", pool->size);

	for (i = 0; i < pool->size; i++) {
		if (pool->array[i])
			iommu_domain_free(pool->array[i]);
	}

	ida_destroy(&pool->idp);
	vfree(pool->array);
}
