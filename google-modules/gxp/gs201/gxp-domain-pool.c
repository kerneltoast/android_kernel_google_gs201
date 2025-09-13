// SPDX-License-Identifier: GPL-2.0-only
/*
 * GXP IOMMU domain allocator.
 *
 * Copyright (C) 2022 Google LLC
 */

#include <linux/bits.h>
#include <linux/iommu.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>

#include <gcip/gcip-iommu.h>

#include "gxp-dma.h"
#include "gxp-domain-pool.h"

/*
 * See enum gcip_iommu_domain_type.
 * Default(0) = utilizing iova_domain
 */
static int gxp_gcip_iommu_domain_type;
module_param_named(gcip_iommu_domain_type, gxp_gcip_iommu_domain_type, int,
		   0660);

int gxp_domain_pool_init(struct gxp_dev *gxp,
			 struct gcip_iommu_domain_pool *pool, unsigned int size)
{
	int ret = gcip_iommu_domain_pool_init(pool, gxp->dev, 0, 0, SZ_4K, size,
					      gxp_gcip_iommu_domain_type);
	u32 num_bits, num_pasids;
	__maybe_unused int i;

	if (ret)
		return ret;

	ret = of_property_read_u32(gxp->dev->of_node, "pasid-num-bits", &num_bits);
	if (ret || num_bits > 31) {
		/* TODO(b/285949227) remove fallback once device-trees are updated */
		dev_warn(gxp->dev, "Failed to fetch pasid-num-bits, defaulting to %d PASIDs (%d)\n",
			 GXP_DEFAULT_NUM_PASIDS, ret);
		num_pasids = GXP_DEFAULT_NUM_PASIDS;
	} else {
		num_pasids = BIT(num_bits);
	}
	/* PASID 0 is reserved for the default domain */
	gcip_iommu_domain_pool_set_pasid_range(gxp->domain_pool, 1, num_pasids - 1);
	gcip_iommu_domain_pool_enable_best_fit_algo(pool);

	return 0;
}

struct gcip_iommu_domain *
gxp_domain_pool_alloc(struct gcip_iommu_domain_pool *pool)
{
	struct gcip_iommu_domain *gdomain =
		gcip_iommu_domain_pool_alloc_domain(pool);

	if (IS_ERR_OR_NULL(gdomain))
		return NULL;

	return gdomain;
}

void gxp_domain_pool_free(struct gcip_iommu_domain_pool *pool,
			  struct gcip_iommu_domain *gdomain)
{
	gcip_iommu_domain_pool_free_domain(pool, gdomain);
}

void gxp_domain_pool_destroy(struct gcip_iommu_domain_pool *pool)
{
	gcip_iommu_domain_pool_destroy(pool);
}
