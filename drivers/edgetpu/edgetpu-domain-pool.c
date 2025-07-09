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

	pool->min_pasid = 0;
	pool->max_pasid = 0;
#if HAS_IOMMU_PASID
	ida_init(&pool->pasid_pool);
#elif HAS_AUX_DOMAINS
	iommu_dev_enable_feature(etdev->dev, IOMMU_DEV_FEAT_AUX);
	if (!iommu_dev_feature_enabled(etdev->dev, IOMMU_DEV_FEAT_AUX))
		dev_warn(etdev->dev, "AUX domains not supported\n");
	else
		pool->aux_enabled = true;
#else
	dev_warn(etdev->dev, "Attaching additional domains not supported\n");
#endif
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
#if HAS_IOMMU_PASID
	ida_destroy(&pool->pasid_pool);
#endif
}

void edgetpu_domain_pool_set_pasid_range(struct edgetpu_domain_pool *pool, ioasid_t min,
					 ioasid_t max)
{
	pool->min_pasid = min;
	pool->max_pasid = max;
}

int edgetpu_domain_pool_attach_domain(struct edgetpu_domain_pool *pool, struct iommu_domain *domain)
{
#if HAS_IOMMU_PASID
	int ret, pasid;

	pasid = ida_alloc_range(&pool->pasid_pool, pool->min_pasid, pool->max_pasid, GFP_KERNEL);
	if (pasid < 0)
		return pasid;

	ret = iommu_attach_device_pasid(domain, pool->etdev->dev, pasid);
	if (ret) {
		ida_free(&pool->pasid_pool, pasid);
		return ret;
	}

	return pasid;
#elif HAS_AUX_DOMAINS
	int ret, pasid;

	if (!pool->aux_enabled)
		return -ENODEV;

	ret = iommu_aux_attach_device(domain, pool->etdev->dev);
	if (ret)
		return ret;

	pasid = iommu_aux_get_pasid(domain, pool->etdev->dev);
	if (pasid < 0) {
		etdev_warn(pool->etdev, "Failed to fetch PASID (%d)", pasid);
		goto err_detach_device;
	}
	if ((ioasid_t)pasid < pool->min_pasid || (ioasid_t)pasid > pool->max_pasid) {
		etdev_warn(pool->etdev, "Invalid PASID %d returned from iommu", pasid);
		goto err_detach_device;
	}

	return pasid;

err_detach_device:
	iommu_aux_detach_device(domain, pool->etdev->dev);
	return -EINVAL;
#else
	return -EOPNOTSUPP;
#endif
}

void edgetpu_domain_pool_detach_domain(struct edgetpu_domain_pool *pool,
				       struct iommu_domain *domain, ioasid_t pasid)
{
#if HAS_IOMMU_PASID
	iommu_detach_device_pasid(domain, pool->etdev->dev, pasid);
	ida_free(&pool->pasid_pool, pasid);
#elif HAS_AUX_DOMAINS
	if (pool->aux_enabled)
		iommu_aux_detach_device(domain, pool->etdev->dev);
#else
	/* No-op if attaching multiple domains is not supported */
	return;
#endif
}
