// SPDX-License-Identifier: GPL-2.0-only
/*
 * GCIP IOMMU domain allocator.
 *
 * Copyright (C) 2022 Google LLC
 */

#include <linux/device.h>
#include <linux/idr.h>
#include <linux/iommu.h>
#include <linux/vmalloc.h>

#include <gcip/gcip-domain-pool.h>

struct dynamic_domain {
	struct list_head list_entry;
	struct iommu_domain *domain;
};

int gcip_domain_pool_init(struct device *dev, struct gcip_domain_pool *pool, unsigned int size)
{
	unsigned int i;
	struct iommu_domain *domain;

	pool->size = size;
	pool->dev = dev;
	INIT_LIST_HEAD(&pool->dynamic_domains);
	mutex_init(&pool->lock);

	if (!size)
		return 0;

	dev_dbg(pool->dev, "Initializing domain pool with %u domains\n", size);

	ida_init(&pool->idp);
	pool->array = vzalloc(sizeof(*pool->array) * size);
	if (!pool->array) {
		ida_destroy(&pool->idp);
		return -ENOMEM;
	}
	for (i = 0; i < size; i++) {
		domain = iommu_domain_alloc(dev->bus);
		if (!domain) {
			dev_err(pool->dev, "Failed to allocate iommu domain %d of %u\n", i + 1,
				size);
			gcip_domain_pool_destroy(pool);
			return -ENOMEM;
		}

		pool->array[i] = domain;
	}
	return 0;
}

struct iommu_domain *gcip_domain_pool_alloc(struct gcip_domain_pool *pool)
{
	int id;
	struct dynamic_domain *ddomain;

	if (!pool->size) {
		ddomain = vzalloc(sizeof(*ddomain));
		if (!ddomain)
			return NULL;

		ddomain->domain = iommu_domain_alloc(pool->dev->bus);
		if (!ddomain->domain) {
			vfree(ddomain);
			return NULL;
		}
		mutex_lock(&pool->lock);
		list_add_tail(&ddomain->list_entry, &pool->dynamic_domains);
		mutex_unlock(&pool->lock);
		return ddomain->domain;
	}

	id = ida_alloc_max(&pool->idp, pool->size - 1, GFP_KERNEL);

	if (id < 0) {
		dev_err(pool->dev, "No more domains available from pool of size %u\n", pool->size);
		return NULL;
	}

	dev_dbg(pool->dev, "Allocated domain from pool with id = %d\n", id);

	return pool->array[id];
}

void gcip_domain_pool_free(struct gcip_domain_pool *pool, struct iommu_domain *domain)
{
	int id;
	struct dynamic_domain *ddomain;
	struct list_head *cur, *nxt;

	if (!pool->size) {
		mutex_lock(&pool->lock);
		list_for_each_safe(cur, nxt, &pool->dynamic_domains) {
			ddomain = container_of(cur, struct dynamic_domain, list_entry);
			if (ddomain->domain == domain) {
				list_del(&ddomain->list_entry);
				mutex_unlock(&pool->lock);
				iommu_domain_free(domain);
				vfree(ddomain);
				return;
			}
		}
		mutex_unlock(&pool->lock);
		return;
	}

	for (id = 0; id < pool->size; id++) {
		if (pool->array[id] == domain) {
			dev_dbg(pool->dev, "Released domain from pool with id = %d\n", id);
			ida_free(&pool->idp, id);
			return;
		}
	}
	dev_err(pool->dev, "Domain not found in pool\n");
}

void gcip_domain_pool_destroy(struct gcip_domain_pool *pool)
{
	int i;
	struct dynamic_domain *ddomain;
	struct list_head *cur, *nxt;

	if (!pool->size) {
		mutex_lock(&pool->lock);
		list_for_each_safe(cur, nxt, &pool->dynamic_domains) {
			ddomain = container_of(cur, struct dynamic_domain, list_entry);
			list_del(&ddomain->list_entry);
			iommu_domain_free(ddomain->domain);
			vfree(ddomain);
		}
		mutex_unlock(&pool->lock);
		return;
	}

	dev_dbg(pool->dev, "Destroying domain pool with %u domains\n", pool->size);

	for (i = 0; i < pool->size; i++) {
		if (pool->array[i])
			iommu_domain_free(pool->array[i]);
	}

	ida_destroy(&pool->idp);
	vfree(pool->array);
}
