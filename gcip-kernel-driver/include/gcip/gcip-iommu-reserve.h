/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Interface of reserving an IOVA region and map buffer/dma-buf chunks to there.
 *
 * Expected functionality of this interface is:
 * 1. Reserve an IOVA region from the IOMMU domain.
 * 2. The user will request mapping a buffer/dma-buf to the reserved region with a specific IOVA
 *    address on demand.
 *
 * Copyright (C) 2023 Google LLC
 */

#ifndef __GCIP_IOMMU_RESERVE_H__
#define __GCIP_IOMMU_RESERVE_H__

#include <linux/dma-buf.h>
#include <linux/kref.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/rbtree.h>
#include <linux/types.h>

#include <gcip/gcip-iommu.h>

struct gcip_iommu_reserve_manager;

/* Operators of `struct gcip_iommu_reserve_manager`. */
struct gcip_iommu_reserve_manager_ops {
	/*
	 * Called when a region which was reserved from @mgr is going to be retired, but there are
	 * mappings which are not yet unmapped from the region. The IP driver should clean up their
	 * own mapping data which might be @data and try to unmap the mapping.
	 *
	 * It is fine to unmap the mapping later if other threads can access to the mapping by the
	 * race condition while this callback is executing. Just make sure that after this callback
	 * is called, the mapping must be eventually unmapped if there are no more threads accessing
	 * it by calling the `gcip_iommu_mapping_unmap` function.
	 *
	 * This callback is required.
	 */
	void (*unmap)(struct gcip_iommu_reserve_manager *mgr, struct gcip_iommu_mapping *mapping,
		      void *data);
};

/* Manages reserved regions. */
struct gcip_iommu_reserve_manager {
	/* The domain where the manager will reserve regions. */
	struct gcip_iommu_domain *domain;
	/* The red-black tree of the reserved regions. */
	struct rb_root regions;
	/* Refcount. */
	struct kref kref;
	/*
	 * If true, the manager is no longer in use and will be released when all of its reserved
	 * regions are returned.
	 */
	bool retired;
	/* Protects @retired and @regions. */
	struct mutex lock;
	/* Operators. */
	const struct gcip_iommu_reserve_manager_ops *ops;
	/*
	 * User-defined data which was passed from the `gcip_iommu_reserve_manager_create` function.
	 */
	void *data;
};

/*
 * The reserved IOVA region which is managed by @mgr and tracks buffers/dma-bufs mapped to here.
 *
 * The IP drivers may not access this structure directly.
 */
struct gcip_iommu_reserve_region {
	/* The manager which maintains the reserved regions. */
	struct gcip_iommu_reserve_manager *mgr;
	/* The domain wheere the region is reserved from. */
	struct gcip_iommu_domain *domain;
	/* The start address of the reserved region. */
	dma_addr_t base_daddr;
	/* The size of the reserved region. */
	size_t size;
	/* List of buffer/dma-buf mappings. */
	struct list_head mappings;
	/* The node to be added to the @mgr->regions red-black tree. */
	struct rb_node node;
	/* Refcount. */
	struct kref kref;
	/*
	 * If true, the region is no longer in use and will be released when all of its mappings
	 * are unmapped.
	 */
	bool retired;
	/* Protects @retired and @mappings. */
	struct mutex lock;
};

/*
 * Creates a manager which manages reserved regions.
 *
 * The caller must call the `gcip_iommu_reserve_manager_retire` function when they no longer use the
 * returned manager so that make it be able to be released whenever all of its reserved regions are
 * returned.
 *
 * Returns an allocated `struct gcip_iommu_reserve_manager` object on success. Otherwise, returns
 * a negative errno pointer.
 */
struct gcip_iommu_reserve_manager *
gcip_iommu_reserve_manager_create(struct gcip_iommu_domain *domain,
				  const struct gcip_iommu_reserve_manager_ops *ops, void *data);

/*
 * Makes the state of @mgr no longer in use.
 *
 * If there are regions which are not yet retired, it will retire all of them. (See the
 * `gcip_iommu_reserve_region_retire` function below)
 *
 * Note that this function doesn't guarantee that @mgr will be released. If there are regions which
 * are retired, but not yet released, @mgr will also be retired, but not yet released. @mgr will be
 * released once its all regions are released and its refcount becomes 0 eventually.
 *
 * Once this function is called, the user can't reserve a region from @mgr anymore and can't map
 * any buffers/dma-bufs to the regions which were reserved from @mgr.
 */
void gcip_iommu_reserve_manager_retire(struct gcip_iommu_reserve_manager *mgr);

/*
 * Reserves @size of IOVA space region which will be managed by @mgr.
 *
 * The caller must call the `gcip_iommu_reserve_region_retire` function when they no longer use the
 * reserved region.
 *
 * Note that this function won't map any buffers to the domain and only allocates a region from the
 * IOVA space to reserve.
 *
 * Returns the starting IOVA address of the reserved region. Otherwise, returns 0.
 */
dma_addr_t gcip_iommu_reserve_region_create(struct gcip_iommu_reserve_manager *mgr, size_t size,
					    u64 gcip_map_flags);

/*
 * Makes the state of the reserved region starting at @base_daddr no longer in use.
 *
 * This function will call the `unmap` operator to let the IP driver try to unmap all mappings which
 * are not yet unmapped from the region. Note that this function doesn't guarantee that the region
 * will return the reserved area and be released if there are mappings which are still not yet
 * unmapped even after this function returns by the race condition. The region will be released once
 * its all mappings are unmapped and its refcount eventually becomes 0.
 *
 * This function must be called once the caller doesn't need the reserved area anymore and makes
 * the region be able to return the area to the IOMMU domain whenever all of its mappings are
 * unmapped and its refcount becomes 0.
 *
 * Once this function is called, the user can't map a buffer or dma-buf to the region anymore.
 *
 * Returns 0 on success. Otherwise, a negative errno.
 */
int gcip_iommu_reserve_region_retire(struct gcip_iommu_reserve_manager *mgr, dma_addr_t base_daddr);

/*
 * This function basically works the same with the `gcip_iommu_domain_map_buffer` function, but
 * receives @mgr managing reserved regions and @iova address to be mapped. @mgr will find a proper
 * reserved region to map the buffer internally.
 *
 * Note that @iova must be page-aligned.
 *
 * If @host_address or @size is not page-aligned, the address and size of the area actually mapped
 * can be different from @iova and @size accordingly. It is because the kernel performs mapping in
 * the page unit. The caller can check both values by reading @device_address and @size fields of
 * the returned mapping object.
 *
 * E.g., if @iova is 0x2000, @host_address is 0x1800, @size is 0x900 and the page size is 0x1000,
 * it will allocate two pages from the reserved region, [0x2000, 0x3000) and [0x3000, 0x4000), and
 * will map the buffer to [0x2800, 0x3100) since the page masked offset of @host_address is 0x800
 * (0x1800 & ~PAGE_MASK = 0x800, 0x2000 + 0x800 = 0x2800). Therefore, @device_address and @size of
 * the returned mapping object will be 0x2800 and 0x2000 which are different from the inputs.
 *
 * To unmap the mapped buffer, use the `gcip_iommu_mapping_unmap` function.
 *
 * @data is the IP driver data which is nullable and will be passed to the operators of the
 * `struct gcip_iommu_reserve_manager_ops`.
 *
 * Returns the mapping instance. Otherwise, returns a negative errno pointer.
 */
struct gcip_iommu_mapping *gcip_iommu_reserve_map_buffer(struct gcip_iommu_reserve_manager *mgr,
							 u64 host_address, size_t size,
							 u64 gcip_map_flags,
							 struct mutex *pin_user_pages_lock,
							 dma_addr_t iova, void *data);

/*
 * This function basically works the same with the `gcip_iommu_domain_map_dma_buf` function, but
 * receives @mgr managing reserved regions and @iova address to be mapped. @mgr will find a proper
 * reserved region to map the dma-buf internally.
 *
 * Note that @iova must be page-aligned.
 *
 * Likewise the buffer mapping above, if the host address or size of any buffer in @dmabuf is not
 * page-aligned, the address and size of the area actually mapped can be different from @iova and
 * the buffer size accordingly. (See the `gcip_iommu_reserve_map_buffer` function above for the
 * details.) However, it will unlikely happen since @dmabuf is expected to be page-aligned and the
 * mapping itself would fail depending on the implementation of the Linux kernel and dma-buf module.
 *
 * To unmap the mapped dma-buf, use the `gcip_iommu_mapping_unmap` function.
 *
 * @data is the IP driver data which is nullable and will be passed to the operators of the
 * `struct gcip_iommu_reserve_manager_ops`.
 *
 * Returns the mapping instance. Otherwise, returns a negative errno pointer.
 */
struct gcip_iommu_mapping *gcip_iommu_reserve_map_dma_buf(struct gcip_iommu_reserve_manager *mgr,
							  struct dma_buf *dmabuf,
							  u64 gcip_map_flags, dma_addr_t iova,
							  void *data);

#endif /* __GCIP_IOMMU_RESERVE_H__ */
