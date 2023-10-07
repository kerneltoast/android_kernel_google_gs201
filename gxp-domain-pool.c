// SPDX-License-Identifier: GPL-2.0
/*
 * GXP IOMMU domain allocator.
 *
 * Copyright (C) 2022 Google LLC
 */

#include <linux/iommu.h>
#include <linux/slab.h>

#include <gcip/gcip-domain-pool.h>

#include "gxp-dma.h"
#include "gxp-domain-pool.h"

int gxp_domain_pool_init(struct gxp_dev *gxp, struct gcip_domain_pool *pool,
			 unsigned int size)
{
	int ret = gcip_domain_pool_init(gxp->dev, pool, size);
	__maybe_unused int i;

	if (ret)
		return ret;

#if IS_ENABLED(CONFIG_GXP_GEM5)
	for (i = 0; i < size; i++) {
		struct iommu_domain *domain = pool->array[i];

		/*
		 * Gem5 uses arm-smmu-v3 which requires domain finalization to do iommu map. Calling
		 * iommu_aux_attach_device to finalize the allocated domain and detach the device
		 * right after that.
		 */
		ret = iommu_aux_attach_device(domain, gxp->dev);
		if (ret) {
			dev_err(gxp->dev,
				"Failed to attach device to iommu domain %d of %u, ret=%d\n",
				i + 1, size, ret);
			gxp_domain_pool_destroy(pool);
			return ret;
		}

		iommu_aux_detach_device(domain, gxp->dev);
	}
#endif /* CONFIG_GXP_GEM5 */

	return 0;
}

struct gxp_iommu_domain *gxp_domain_pool_alloc(struct gcip_domain_pool *pool)
{
	struct iommu_domain *domain = gcip_domain_pool_alloc(pool);
	struct gxp_iommu_domain *gdomain;

	if (!domain)
		return NULL;

	gdomain = kmalloc(sizeof(*gdomain), GFP_KERNEL);
	if (!gdomain) {
		gcip_domain_pool_free(pool, domain);
		return NULL;
	}

	gdomain->domain = domain;

	return gdomain;
}

void gxp_domain_pool_free(struct gcip_domain_pool *pool,
			  struct gxp_iommu_domain *gdomain)
{
	gcip_domain_pool_free(pool, gdomain->domain);
	kfree(gdomain);
}

void gxp_domain_pool_destroy(struct gcip_domain_pool *pool)
{
	gcip_domain_pool_destroy(pool);
}
