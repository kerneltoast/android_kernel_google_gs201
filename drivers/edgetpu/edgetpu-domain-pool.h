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
#include <linux/version.h>

#include "edgetpu-internal.h"

#define HAS_IOMMU_PASID (!IS_ENABLED(CONFIG_EDGETPU_TEST))

#define HAS_AUX_DOMAINS (LINUX_VERSION_CODE <= KERNEL_VERSION(5, 17, 0))

#if HAS_IOMMU_PASID
#include <linux/idr.h>
#endif

struct edgetpu_domain_pool {
	struct ida idp;			/* ID allocator to keep track of used domains. */
	/*
	 * Size of the pool. Can be set to 0, in which case the implementation will fall back to
	 * dynamic domain allocation using the IOMMU API directly.
	 */
	unsigned int size;
	struct iommu_domain **array;	/* Array holding the pointers to pre-allocated domains. */
	struct edgetpu_dev *etdev;	/* The edgetpu device used for logging warnings/errors. */
	ioasid_t min_pasid;
	ioasid_t max_pasid;
#if HAS_IOMMU_PASID
	struct ida pasid_pool;
#elif HAS_AUX_DOMAINS
	bool aux_enabled;
#endif
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

/* Sets the range of valid PASIDs to be used when attaching a domain
 *
 * @min: The smallest acceptable value to be assigned to an attached domain
 * @max: The largest acceptable value to be assigned to an attached domain
 */
void edgetpu_domain_pool_set_pasid_range(struct edgetpu_domain_pool *pool, ioasid_t min,
					 ioasid_t max);

/*
 * Attaches an IOMMU domain
 *
 * Before calling this function, you must set the valid PASID range by calling
 * `edgetpu_domain_pool_set_pasid_range()`.
 *
 * @pool: IOMMU domain pool @domain was allocated from
 * @domain: The IOMMU domain to attach
 *
 * Returns:
 * * >= 0    - The PASID the domain was successfully attached with
 * * -ENOSYS - This device does not support attaching multiple domains
 * * other   - Failed to attach the domain or obtain a PASID for it
 */
int edgetpu_domain_pool_attach_domain(struct edgetpu_domain_pool *pool,
				      struct iommu_domain *domain);

/*
 * Detaches an IOMMU domain
 *
 * @pool: IOMMU domain pool @domain was allocated from and attached by
 * @domain: The IOMMU domain to detach
 * @pasid: The PASID returned when @domain was attached
 */
void edgetpu_domain_pool_detach_domain(struct edgetpu_domain_pool *pool,
				       struct iommu_domain *domain, ioasid_t pasid);

#endif /* __EDGETPU_DOMAIN_POOL_H__ */
