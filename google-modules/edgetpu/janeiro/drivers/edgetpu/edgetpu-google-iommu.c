// SPDX-License-Identifier: GPL-2.0
/*
 * Edge TPU IOMMU interface.
 *
 * Copyright (C) 2019 Google, Inc.
 */

#include <linux/bits.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/idr.h>
#include <linux/iommu.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/types.h>

#include "edgetpu-config.h"
#include "edgetpu-domain-pool.h"
#include "edgetpu-internal.h"
#include "edgetpu-mapping.h"
#include "edgetpu-mmu.h"

#if !defined(EDGETPU_NUM_PREALLOCATED_DOMAINS)
#define EDGETPU_NUM_PREALLOCATED_DOMAINS 0
#endif

#define HAS_BEST_FIT_ALGO                                                                          \
	(IS_ENABLED(CONFIG_ANDROID) && LINUX_VERSION_CODE <= KERNEL_VERSION(5, 15, 0))

#if HAS_BEST_FIT_ALGO
#include <linux/dma-iommu.h>
#endif

struct edgetpu_iommu {
	struct iommu_group *iommu_group;
	/*
	 * IOMMU domains currently attached.
	 * NULL for a slot that doesn't have an attached domain.
	 */
	struct iommu_domain *domains[EDGETPU_NCONTEXTS];
	/*
	 * Records IDs for all domains currently allocated, to support IOMMU (un)mapping
	 * when the domain is not attached.
	 */
	struct idr domain_id_pool;
	struct mutex pool_lock;		/* protects access of @domain_id_pool */
	bool context_0_default;		/* is context 0 domain the default? */
	/*
	 * Holds a pool of pre-allocated IOMMU domains if the chip config specifies this is
	 * required.
	 * The implementation will fall back to dynamically allocated domains otherwise.
	 */
	struct edgetpu_domain_pool domain_pool;

};

struct edgetpu_iommu_map_params {
	int prot;
	size_t size;
	struct iommu_domain *domain;
};

/*
 * Return context ID enumeration value as a Process Address Space ID.
 * Caller ensures context_id is valid, i.e. does not equal to
 * EDGETPU_CONTEXT_INVALID or OR'ed with EDGETPU_CONTEXT_DOMAIN_TOKEN.
 */
static uint context_id_to_pasid(enum edgetpu_context_id context_id)
{
	return (uint)context_id;
}

static struct iommu_domain *get_domain_by_token(struct edgetpu_iommu *etiommu,
						int token)
{
	struct iommu_domain *domain;

	mutex_lock(&etiommu->pool_lock);
	domain = idr_find(&etiommu->domain_id_pool, token);
	mutex_unlock(&etiommu->pool_lock);
	return domain;
}

static struct iommu_domain *
get_domain_by_context_id(struct edgetpu_dev *etdev,
			 enum edgetpu_context_id ctx_id)
{
	struct iommu_domain *domain = NULL;
	struct device *dev = etdev->dev;
	struct edgetpu_iommu *etiommu = etdev->mmu_cookie;
	uint pasid;

	if (ctx_id == EDGETPU_CONTEXT_INVALID)
		return NULL;
	if (ctx_id & EDGETPU_CONTEXT_DOMAIN_TOKEN)
		return get_domain_by_token(
			etiommu, ctx_id ^ EDGETPU_CONTEXT_DOMAIN_TOKEN);
	pasid = context_id_to_pasid(ctx_id);
	if (pasid < EDGETPU_NCONTEXTS)
		domain = etiommu->domains[pasid];

	/* Fall back to default domain. */
	if (!domain)
		domain = iommu_get_domain_for_dev(dev);
	return domain;
}

static int edgetpu_iommu_dev_fault_handler(struct iommu_fault *fault,
					   void *token)
{
	struct edgetpu_dev *etdev = (struct edgetpu_dev *)token;

	if (fault->type == IOMMU_FAULT_DMA_UNRECOV) {
		etdev_warn(etdev, "Unrecoverable IOMMU fault!\n");
		etdev_warn(etdev, "Reason = %08X\n", fault->event.reason);
		etdev_warn(etdev, "flags = %08X\n", fault->event.flags);
		etdev_warn(etdev, "pasid = %08X\n", fault->event.pasid);
		etdev_warn(etdev, "perms = %08X\n", fault->event.perm);
		etdev_warn(etdev, "addr = %llX\n", fault->event.addr);
		etdev_warn(etdev, "fetch_addr = %llX\n", fault->event.fetch_addr);
	} else if (fault->type == IOMMU_FAULT_PAGE_REQ) {
		etdev_dbg(etdev, "IOMMU page request fault!\n");
		etdev_dbg(etdev, "flags = %08X\n", fault->prm.flags);
		etdev_dbg(etdev, "pasid = %08X\n", fault->prm.pasid);
		etdev_dbg(etdev, "grpid = %08X\n", fault->prm.grpid);
		etdev_dbg(etdev, "perms = %08X\n", fault->prm.perm);
		etdev_dbg(etdev, "addr = %llX\n", fault->prm.addr);
	}
	// Tell the IOMMU driver to carry on
	return -EAGAIN;
}

static int edgetpu_register_iommu_device_fault_handler(struct edgetpu_dev *etdev)
{
	etdev_dbg(etdev, "Registering IOMMU device fault handler\n");
	return iommu_register_device_fault_handler(etdev->dev, edgetpu_iommu_dev_fault_handler,
						   etdev);
}

static int edgetpu_unregister_iommu_device_fault_handler(struct edgetpu_dev *etdev)
{
	etdev_dbg(etdev, "Unregistering IOMMU device fault handler\n");
	return iommu_unregister_device_fault_handler(etdev->dev);
}

/* A callback for idr_for_each to release the domains */
static int edgetpu_idr_free_domain_callback(int id, void *p, void *data)
{
	struct iommu_domain *domain = p;
	struct edgetpu_iommu *etiommu = data;

	edgetpu_domain_pool_free(&etiommu->domain_pool, domain);
	return 0;
}

static int edgetpu_iommu_fault_handler(struct iommu_domain *domain,
				       struct device *dev, unsigned long iova,
				       int flags, void *token)
{
	struct edgetpu_iommu_domain *etdomain =
		(struct edgetpu_iommu_domain *)token;

	dev_dbg(dev, "IOMMU fault on address %08lX. PASID = %u flags = %08X",
		iova, etdomain->pasid, flags);
	// Tell the IOMMU driver we are OK with this fault
	return 0;
}

static void edgetpu_init_etdomain(struct edgetpu_iommu_domain *etdomain,
				  struct iommu_domain *domain,
				  int token)
{
	etdomain->iommu_domain = domain;
	etdomain->pasid = IOMMU_PASID_INVALID;
	etdomain->token = token;
	iommu_set_fault_handler(domain, edgetpu_iommu_fault_handler, etdomain);
}

/*
 * Expect a default domain was already allocated for the group. If not try to allocate and attach
 * one.
 */
static int check_default_domain(struct edgetpu_dev *etdev,
				struct edgetpu_iommu *etiommu)
{
	struct iommu_domain *domain;
	int ret;

	domain = iommu_get_domain_for_dev(etdev->dev);
	/* if default domain exists then we are done */
	if (domain) {
		etiommu->context_0_default = true;
		goto out;
	}
	etdev_warn(etdev, "device group has no default iommu domain\n");

	domain = edgetpu_domain_pool_alloc(&etiommu->domain_pool);
	if (!domain) {
		etdev_warn(etdev, "iommu domain alloc failed");
		return -EINVAL;
	}
	ret = iommu_attach_device(domain, etdev->dev);
	if (ret) {
		etdev_warn(etdev, "Attach default domain failed: %d", ret);
		edgetpu_domain_pool_free(&etiommu->domain_pool, domain);
		return ret;
	}

out:
	etiommu->domains[0] = domain;
	return 0;
}

/* mmu_info is unused and NULL for IOMMU version, let IOMMU API supply info */
int edgetpu_mmu_attach(struct edgetpu_dev *etdev, void *mmu_info)
{
	struct edgetpu_iommu *etiommu;
	u32 num_bits, num_pasids;
	int ret;

	etiommu = kzalloc(sizeof(*etiommu), GFP_KERNEL);
	if (!etiommu)
		return -ENOMEM;
	ret = edgetpu_domain_pool_init(etdev, &etiommu->domain_pool,
				       EDGETPU_NUM_PREALLOCATED_DOMAINS);
	if (ret) {
		etdev_err(etdev, "Unable to create domain pool (%d)\n", ret);
		goto err_free_etiommu;
	}

	ret = of_property_read_u32(etdev->dev->of_node, "pasid-num-bits", &num_bits);
	if (ret || num_bits > 31) {
		/* TODO(b/285949227) remove fallback once device-trees are updated */
		etdev_warn(etdev, "Failed to fetch pasid-num-bits, defaulting to 8 PASIDs (%d)\n",
			   ret);
		num_pasids = 8;
	} else {
		num_pasids = BIT(num_bits);
	}

	/* PASID 0 is reserved for the default domain */
	edgetpu_domain_pool_set_pasid_range(&etiommu->domain_pool, 1, num_pasids - 1);

	idr_init(&etiommu->domain_id_pool);
	mutex_init(&etiommu->pool_lock);
	etiommu->iommu_group = iommu_group_get(etdev->dev);
	if (etiommu->iommu_group)
		iommu_group_set_name(etiommu->iommu_group, "edgetpu");
	else
		dev_warn(etdev->dev, "device has no iommu group\n");

#if HAS_BEST_FIT_ALGO
	/* Enable best fit algorithm to minimize fragmentation */
	ret = iommu_dma_enable_best_fit_algo(etdev->dev);
	if (ret)
		etdev_warn(etdev, "Failed to enable best-fit IOVA allocator (%d)\n", ret);
#endif

	ret = check_default_domain(etdev, etiommu);
	if (ret)
		goto err_destroy_pool;

	ret = edgetpu_register_iommu_device_fault_handler(etdev);
	if (ret)
		etdev_warn(etdev, "Failed to register fault handler! (%d)\n",
			   ret);

	/* etiommu initialization done */
	etdev->mmu_cookie = etiommu;
	return 0;

err_destroy_pool:
	edgetpu_domain_pool_destroy(&etiommu->domain_pool);
err_free_etiommu:
	kfree(etiommu);
	return ret;
}

void edgetpu_mmu_reset(struct edgetpu_dev *etdev)
{
	/* If need to reset IOMMU driver can issue here. */
}

void edgetpu_mmu_detach(struct edgetpu_dev *etdev)
{
	struct edgetpu_iommu *etiommu = etdev->mmu_cookie;
	int i, ret;

	if (!etiommu)
		return;

	ret = edgetpu_unregister_iommu_device_fault_handler(etdev);
	if (ret)
		etdev_warn(etdev,
			   "Failed to unregister device fault handler (%d)\n",
			   ret);
	edgetpu_mmu_reset(etdev);

	for (i = etiommu->context_0_default ? 1 : 0; i < EDGETPU_NCONTEXTS; i++) {
		if (etiommu->domains[i])
			edgetpu_domain_pool_detach_domain(&etiommu->domain_pool,
							  etiommu->domains[i], i);
	}

	if (etiommu->iommu_group)
		iommu_group_put(etiommu->iommu_group);

	/* free the domain if the context 0 domain is not default */
	if (!etiommu->context_0_default && etiommu->domains[0])
		edgetpu_domain_pool_free(&etiommu->domain_pool, etiommu->domains[0]);

	idr_for_each(&etiommu->domain_id_pool, edgetpu_idr_free_domain_callback,
		     etiommu);
	idr_destroy(&etiommu->domain_id_pool);
	edgetpu_domain_pool_destroy(&etiommu->domain_pool);
	kfree(etiommu);
	etdev->mmu_cookie = NULL;
}

int edgetpu_mmu_reattach(struct edgetpu_dev *etdev)
{
	return 0;
}

static int get_iommu_map_params(struct edgetpu_dev *etdev,
				struct edgetpu_mapping *map,
				enum edgetpu_context_id context_id,
				struct edgetpu_iommu_map_params *params, u32 mmu_flags)
{
	struct edgetpu_iommu *etiommu = etdev->mmu_cookie;
	size_t size = 0;
	int prot = mmu_flag_to_iommu_prot(mmu_flags, etdev->dev, map->dir);
	struct iommu_domain *domain;
	int i;
	struct scatterlist *sg;

	if (!etiommu)
		return -EINVAL;

	domain = get_domain_by_context_id(etdev, context_id);
	if (!domain) {
		etdev_err(etdev, "Unable to find an iommu domain\n");
		return -ENODEV;
	}

	for_each_sg(map->sgt.sgl, sg, map->sgt.orig_nents, i)
		size += sg->length;

	prot |= IOMMU_PBHA_PROT(EDGEPTU_MAP_PBHA_VALUE(map->flags));
	params->prot = prot;
	params->size = size;
	params->domain = domain;
	return 0;
}

int edgetpu_mmu_map(struct edgetpu_dev *etdev, struct edgetpu_mapping *map,
		    enum edgetpu_context_id context_id, u32 mmu_flags)
{
	int ret;
	unsigned long iova;
	struct edgetpu_iommu_map_params params;
	struct iommu_domain *default_domain =
		iommu_get_domain_for_dev(etdev->dev);

	ret = get_iommu_map_params(etdev, map, context_id, &params, mmu_flags);
	if (ret)
		return ret;

	ret = dma_map_sg_attrs(etdev->dev, map->sgt.sgl, map->sgt.nents, map->dir, map->dma_attrs);
	if (!ret)
		return -ENOSPC;
	map->sgt.nents = ret;
	iova = sg_dma_address(map->sgt.sgl);

	/*
	 * All mappings get added to the default domain by the call to
	 * dma_map_sg above.
	 * Per-context mappings are mirrored to their specific domains here
	 */
	if (params.domain != default_domain) {
		ssize_t mapped = (ssize_t)iommu_map_sg(params.domain, iova, map->sgt.sgl,
						       map->sgt.orig_nents, params.prot);

		/* iommu_map_sg returns 0 on failure before 5.15, returns -errno afterwards */
		if (mapped <= 0) {
			/* Undo the mapping in the default domain */
			dma_unmap_sg_attrs(etdev->dev, map->sgt.sgl, map->sgt.orig_nents, map->dir,
					   DMA_ATTR_SKIP_CPU_SYNC);
			return mapped == 0 ? -ENOMEM : (int)mapped;
		}
	}

	map->device_address = iova;
	etdev_dbg(etdev, "%s: ctx=%x iova=%pad dma=%pad size=%#x flags=%#x\n",
		  __func__, context_id, &map->device_address, &sg_dma_address(map->sgt.sgl),
		  sg_dma_len(map->sgt.sgl), mmu_flags);
	return 0;
}

void edgetpu_mmu_unmap(struct edgetpu_dev *etdev, struct edgetpu_mapping *map,
		       enum edgetpu_context_id context_id)
{
	int ret;
	struct edgetpu_iommu_map_params params;
	struct iommu_domain *default_domain =
		iommu_get_domain_for_dev(etdev->dev);

	etdev_dbg(etdev, "%s: ctx=%x iova=%pad\n", __func__, context_id, &map->device_address);
	ret = get_iommu_map_params(etdev, map, context_id, &params, 0);
	if (!ret && params.domain != default_domain) {
		/*
		 * If this is a per-context mapping, it was mirrored in the
		 * per-context domain. Undo that mapping first.
		 */
		iommu_unmap(params.domain, map->device_address, params.size);
	}

	/* Undo the mapping in the default domain */
	dma_unmap_sg_attrs(etdev->dev, map->sgt.sgl, map->sgt.orig_nents, map->dir, map->dma_attrs);
}

int edgetpu_mmu_map_iova_sgt(struct edgetpu_dev *etdev, tpu_addr_t iova,
			     struct sg_table *sgt, enum dma_data_direction dir,
			     u32 mmu_flags,
			     enum edgetpu_context_id context_id)
{
	const int prot = mmu_flag_to_iommu_prot(mmu_flags, etdev->dev, dir);
	const tpu_addr_t orig_iova = iova;
	struct scatterlist *sg;
	int i;
	int ret;

	for_each_sg(sgt->sgl, sg, sgt->orig_nents, i) {
		ret = edgetpu_mmu_add_translation(etdev, iova, sg_phys(sg),
						  sg->length, prot, context_id);
		if (ret)
			goto error;
		iova += sg->length;
	}
	etdev_dbg(etdev, "%s: ctx=%x iova=%pad size=%#llx dir=%d\n", __func__, context_id,
		  &sg_dma_address(sgt->sgl), iova - orig_iova, dir);
	return 0;

error:
	edgetpu_mmu_remove_translation(etdev, orig_iova, iova - orig_iova,
				       context_id);
	return ret;
}

void edgetpu_mmu_unmap_iova_sgt_attrs(struct edgetpu_dev *etdev,
				      tpu_addr_t iova, struct sg_table *sgt,
				      enum dma_data_direction dir,
				      enum edgetpu_context_id context_id,
				      unsigned long attrs)
{
	size_t size = 0;
	struct scatterlist *sg;
	int i;

	for_each_sg(sgt->sgl, sg, sgt->orig_nents, i)
		size += sg->length;
	etdev_dbg(etdev, "%s: ctx=%x iova=%pad size=%#zx\n", __func__, context_id, &iova, size);
	edgetpu_mmu_remove_translation(etdev, iova, size, context_id);
}

tpu_addr_t edgetpu_mmu_alloc(struct edgetpu_dev *etdev, size_t size,
			     u32 mmu_flags)
{
	return 0;
}

void edgetpu_mmu_reserve(struct edgetpu_dev *etdev, tpu_addr_t tpu_addr,
			 size_t size)
{
}

void edgetpu_mmu_free(struct edgetpu_dev *etdev, tpu_addr_t tpu_addr,
		      size_t size)
{
}

int edgetpu_mmu_add_translation(struct edgetpu_dev *etdev, unsigned long iova,
				phys_addr_t paddr, size_t size, int prot,
				enum edgetpu_context_id context_id)
{
	struct iommu_domain *domain;

	domain = get_domain_by_context_id(etdev, context_id);
	if (!domain)
		return -ENODEV;
	etdev_dbg(etdev, "%s: ctx=%x iova=%pad paddr=%pap size=%#zx prot=%#x\n",
		  __func__, context_id, &iova, &paddr, size, prot);
	return iommu_map(domain, iova, paddr, size, prot);
}

void edgetpu_mmu_remove_translation(struct edgetpu_dev *etdev,
				    unsigned long iova, size_t size,
				    enum edgetpu_context_id context_id)
{
	struct iommu_domain *domain;

	etdev_dbg(etdev, "%s: ctx=%x iova=%pad size=%#zx\n", __func__, context_id, &iova, size);
	domain = get_domain_by_context_id(etdev, context_id);
	if (domain)
		iommu_unmap(domain, iova, size);
}

/*
 * This function assumes [@down_addr, @down_addr + size) is mapped to
 * [phys_addr, phys_addr + size). This is true if @down_addr was mapped by
 * dma_alloc_* series, and may not be true when mapped by dma_map_sg*.
 */
tpu_addr_t edgetpu_mmu_tpu_map(struct edgetpu_dev *etdev, dma_addr_t down_addr,
			       size_t size, enum dma_data_direction dir,
			       enum edgetpu_context_id context_id,
			       u32 mmu_flags)
{
	struct iommu_domain *domain;
	struct iommu_domain *default_domain =
		iommu_get_domain_for_dev(etdev->dev);
	phys_addr_t paddr;
	int prot = mmu_flag_to_iommu_prot(mmu_flags, etdev->dev, dir);

	domain = get_domain_by_context_id(etdev, context_id);
	/*
	 * Either we don't have per-context domains or this mapping
	 * belongs to the default context, in which case we don't need
	 * to do anything
	 */
	if (!domain || domain == default_domain)
		return down_addr;
	paddr = iommu_iova_to_phys(default_domain, down_addr);
	if (!paddr)
		return 0;
	etdev_dbg(etdev, "%s: ctx=%x iova=%pad size=%zx flags=%#x\n",
		  __func__, context_id, &down_addr, size, mmu_flags);
	/* Map the address to the context-specific domain */
	if (iommu_map(domain, down_addr, paddr, size, prot))
		return 0;

	/* Return downstream IOMMU DMA address as TPU address. */
	return down_addr;
}

void edgetpu_mmu_tpu_unmap(struct edgetpu_dev *etdev, tpu_addr_t tpu_addr,
			   size_t size, enum edgetpu_context_id context_id)
{
	struct iommu_domain *domain;
	struct iommu_domain *default_domain =
		iommu_get_domain_for_dev(etdev->dev);

	etdev_dbg(etdev, "%s: ctx=%x iova=%pad\n",
		  __func__, context_id, &tpu_addr);
	domain = get_domain_by_context_id(etdev, context_id);
	/*
	 * Either we don't have per-context domains or this mapping
	 * belongs to the default context, in which case we don't need
	 * to do anything
	 */
	if (!domain || domain == default_domain)
		return;
	/* Unmap the address from the context-specific domain */
	iommu_unmap(domain, tpu_addr, size);
}

tpu_addr_t edgetpu_mmu_tpu_map_sgt(struct edgetpu_dev *etdev,
				   struct sg_table *sgt, enum dma_data_direction dir,
				   enum edgetpu_context_id context_id,
				   u32 mmu_flags)
{
	struct iommu_domain *domain;
	struct iommu_domain *default_domain =
		iommu_get_domain_for_dev(etdev->dev);
	phys_addr_t paddr;
	dma_addr_t iova, cur_iova;
	size_t size;
	int prot = mmu_flag_to_iommu_prot(mmu_flags, etdev->dev, dir);
	struct scatterlist *sg;
	int ret;
	int i;

	/*
	 * We cannot map the SG to a single TPU VA if the table contains more
	 * than one DMA address.
	 */
	if (sgt->nents != 1)
		return 0;
	iova = sg_dma_address(sgt->sgl);
	domain = get_domain_by_context_id(etdev, context_id);
	/*
	 * Either we don't have per-context domains or this mapping
	 * belongs to the default context, in which case we don't need
	 * to do anything.
	 */
	if (!domain || domain == default_domain)
		return iova;
	etdev_dbg(etdev, "%s: ctx=%x iova=%pad size=%#x flags=%#x\n",
		  __func__, context_id, &iova, sg_dma_len(sgt->sgl), mmu_flags);
	cur_iova = iova;
	for_each_sg(sgt->sgl, sg, sgt->orig_nents, i) {
		/* ignore sg->offset */
		paddr =  page_to_phys(sg_page(sg));
		size = sg->length + sg->offset;
		ret = iommu_map(domain, cur_iova, paddr, size, prot);
		if (ret)
			goto rollback;
		cur_iova += size;
	}

	return iova;
rollback:
	iommu_unmap(domain, iova, cur_iova - iova);
	etdev_err(etdev, "TPU map sgt failed: %d", ret);
	return 0;
}

void edgetpu_mmu_tpu_unmap_sgt(struct edgetpu_dev *etdev, tpu_addr_t tpu_addr,
			       struct sg_table *sgt,
			       enum edgetpu_context_id context_id)
{
	struct iommu_domain *domain;
	struct iommu_domain *default_domain =
		iommu_get_domain_for_dev(etdev->dev);

	domain = get_domain_by_context_id(etdev, context_id);
	if (!domain || domain == default_domain)
		return;
	etdev_dbg(etdev, "%s: ctx=%x iova=%pad\n", __func__, context_id, &tpu_addr);
	/*
	 * We have checked sgt->nents == 1 on map, sg_dma_len(sgt->sgl) should
	 * equal the total size.
	 */
	iommu_unmap(domain, tpu_addr, sg_dma_len(sgt->sgl));
}

void edgetpu_mmu_use_dev_dram(struct edgetpu_dev *etdev, bool use_dev_dram)
{
}

struct edgetpu_iommu_domain *edgetpu_mmu_alloc_domain(struct edgetpu_dev *etdev)
{
	struct edgetpu_iommu_domain *etdomain;
	struct edgetpu_iommu *etiommu = etdev->mmu_cookie;
	struct iommu_domain *domain;
	int token;

	domain = edgetpu_domain_pool_alloc(&etiommu->domain_pool);
	if (!domain) {
		etdev_warn(etdev, "iommu domain allocation failed");
		return NULL;
	}

	etdomain = kzalloc(sizeof(*etdomain), GFP_KERNEL);
	if (!etdomain) {
		edgetpu_domain_pool_free(&etiommu->domain_pool, domain);
		return NULL;
	}

	mutex_lock(&etiommu->pool_lock);
	token = idr_alloc(&etiommu->domain_id_pool, domain, 0,
			  EDGETPU_DOMAIN_TOKEN_END, GFP_KERNEL);
	mutex_unlock(&etiommu->pool_lock);
	if (token < 0) {
		etdev_warn(etdev, "alloc iommu domain token failed: %d", token);
		kfree(etdomain);
		edgetpu_domain_pool_free(&etiommu->domain_pool, domain);
		return NULL;
	}

	edgetpu_init_etdomain(etdomain, domain, token);
	return etdomain;
}

void edgetpu_mmu_free_domain(struct edgetpu_dev *etdev,
			     struct edgetpu_iommu_domain *etdomain)
{
	struct edgetpu_iommu *etiommu = etdev->mmu_cookie;

	if (!etdomain)
		return;
	if (etdomain->pasid != IOMMU_PASID_INVALID) {
		etdev_warn(etdev, "Domain should be detached before free");
		edgetpu_mmu_detach_domain(etdev, etdomain);
	}
	mutex_lock(&etiommu->pool_lock);
	idr_remove(&etiommu->domain_id_pool, etdomain->token);
	mutex_unlock(&etiommu->pool_lock);
	edgetpu_domain_pool_free(&etiommu->domain_pool, etdomain->iommu_domain);
	kfree(etdomain);
}

int edgetpu_mmu_attach_domain(struct edgetpu_dev *etdev,
			      struct edgetpu_iommu_domain *etdomain)
{
	struct edgetpu_iommu *etiommu = etdev->mmu_cookie;
	struct iommu_domain *domain;
	int pasid;

	if (etdomain->pasid != (ioasid_t)IOMMU_PASID_INVALID) {
		etdev_err(etdev, "Attempt to attach already-attached domain with PASID=%u",
			  etdomain->pasid);
		return -EINVAL;
	}

	domain = etdomain->iommu_domain;

	pasid = edgetpu_domain_pool_attach_domain(&etiommu->domain_pool, domain);
	if (pasid < 0) {
		etdev_warn(etdev, "Attach IOMMU domain failed (%d)\n", pasid);
		return pasid;
	}

	etiommu->domains[pasid] = domain;
	etdomain->pasid = pasid;
	return 0;
}

void edgetpu_mmu_detach_domain(struct edgetpu_dev *etdev,
			       struct edgetpu_iommu_domain *etdomain)
{
	struct edgetpu_iommu *etiommu = etdev->mmu_cookie;
	uint pasid = etdomain->pasid;

	if (pasid <= 0 || pasid >= EDGETPU_NCONTEXTS)
		return;

	etiommu->domains[pasid] = NULL;
	etdomain->pasid = IOMMU_PASID_INVALID;
	edgetpu_domain_pool_detach_domain(&etiommu->domain_pool, etdomain->iommu_domain, pasid);
}
