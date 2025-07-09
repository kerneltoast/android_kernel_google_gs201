// SPDX-License-Identifier: GPL-2.0-only
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

#include <linux/dma-buf.h>
#include <linux/kref.h>
#include <linux/list.h>
#include <linux/mm.h>
#include <linux/mutex.h>
#include <linux/rbtree.h>
#include <linux/slab.h>

#include <gcip/gcip-iommu-reserve.h>
#include <gcip/gcip-iommu.h>

/* Wrapping mapping structure to be managed by the `struct gcip_iommu_reserve_region`. */
struct gcip_iommu_reserve_mapping {
	struct gcip_iommu_reserve_region *region;
	struct gcip_iommu_mapping *mapping;
	struct list_head node;
	struct kref kref;
	void *data;
};

/* Releases @mgr. */
static void gcip_iommu_reserve_manager_release(struct kref *kref)
{
	struct gcip_iommu_reserve_manager *mgr =
		container_of(kref, struct gcip_iommu_reserve_manager, kref);

	kfree(mgr);
}

/* Increments the refcount of @mgr. */
static void gcip_iommu_reserve_manager_get(struct gcip_iommu_reserve_manager *mgr)
{
	kref_get(&mgr->kref);
}

/* Decrements the refcount of @mgr. If it becomes 0, @mgr will be released. */
static void gcip_iommu_reserve_manager_put(struct gcip_iommu_reserve_manager *mgr)
{
	kref_put(&mgr->kref, gcip_iommu_reserve_manager_release);
}

/* Checks whether @ops has valid operators and set it to @mgr. */
static int gcip_iommu_reserve_manager_set_ops(struct gcip_iommu_reserve_manager *mgr,
					      const struct gcip_iommu_reserve_manager_ops *ops)
{
	if (!ops || !ops->unmap)
		return -EINVAL;

	mgr->ops = ops;

	return 0;
}

struct gcip_iommu_reserve_manager *
gcip_iommu_reserve_manager_create(struct gcip_iommu_domain *domain,
				  const struct gcip_iommu_reserve_manager_ops *ops, void *data)
{
	struct gcip_iommu_reserve_manager *mgr = kzalloc(sizeof(*mgr), GFP_KERNEL);
	int ret;

	if (!mgr)
		return ERR_PTR(-ENOMEM);

	ret = gcip_iommu_reserve_manager_set_ops(mgr, ops);
	if (ret) {
		kfree(mgr);
		return ERR_PTR(ret);
	}

	mgr->domain = domain;
	mgr->data = data;
	mgr->regions = RB_ROOT;
	kref_init(&mgr->kref);
	mutex_init(&mgr->lock);

	return mgr;
}

/* Returns the reserved region to the IOMMU domain and releases it. */
static void gcip_iommu_reserve_region_release(struct kref *kref)
{
	struct gcip_iommu_reserve_region *region =
		container_of(kref, struct gcip_iommu_reserve_region, kref);

	/*
	 * Releases a refcount of @mgr which was held by the `gcip_iommu_reserve_region_create`
	 * function.
	 */
	gcip_iommu_reserve_manager_put(region->mgr);
	gcip_iommu_free_iova(region->domain, region->base_daddr, region->size);
	kfree(region);
}

/* Increments the refcount of @region. */
static void gcip_iommu_reserve_region_get(struct gcip_iommu_reserve_region *region)
{
	if (!region)
		return;
	kref_get(&region->kref);
}

/* Decrements the refcount of @region. */
static void gcip_iommu_reserve_region_put(struct gcip_iommu_reserve_region *region)
{
	if (!region)
		return;
	kref_put(&region->kref, gcip_iommu_reserve_region_release);
}

/*
 * Releases the `struct gcip_iommu_reserve_mapping` instance which was allocated by the
 * `gcip_iommu_reserve_mapping_alloc_locked` function.
 */
static void gcip_iommu_reserve_mapping_release(struct kref *kref)
{
	struct gcip_iommu_reserve_mapping *reserve_mapping =
		container_of(kref, struct gcip_iommu_reserve_mapping, kref);

	/* Releases the refcount which was held by `gcip_iommu_reserve_mapping_alloc_locked`. */
	gcip_iommu_reserve_region_put(reserve_mapping->region);
	kfree(reserve_mapping);
}

/* Increments the refcount of @reserve_mapping. */
static void gcip_iommu_reserve_mapping_get(struct gcip_iommu_reserve_mapping *reserve_mapping)
{
	kref_get(&reserve_mapping->kref);
}

/* Decrements the refcount of @reserve_mapping. */
static void gcip_iommu_reserve_mapping_put(struct gcip_iommu_reserve_mapping *reserve_mapping)
{
	kref_put(&reserve_mapping->kref, gcip_iommu_reserve_mapping_release);
}

/* The callback which will be called when @reserve_mapping->mapping is unmapped. */
static void gcip_iommu_reserve_mapping_after_unmap(void *data)
{
	struct gcip_iommu_reserve_mapping *reserve_mapping = data;
	struct gcip_iommu_reserve_region *region = reserve_mapping->region;

	mutex_lock(&region->lock);
	if (!region->retired)
		list_del_init(&reserve_mapping->node);
	mutex_unlock(&region->lock);

	gcip_iommu_reserve_mapping_put(reserve_mapping);
}

static const struct gcip_iommu_mapping_ops reserve_mapping_ops = {
	.after_unmap = gcip_iommu_reserve_mapping_after_unmap,
};

/*
 * Allocates a `struct gcip_iommu_reserve_mapping` instance which wraps @mapping containing mapping
 * information of mapped buffer/dma-buf to @region.
 *
 * The caller must hold @region->lock.
 */
static struct gcip_iommu_reserve_mapping *
gcip_iommu_reserve_mapping_alloc_locked(struct gcip_iommu_reserve_region *region,
					struct gcip_iommu_mapping *mapping, void *data)
{
	struct gcip_iommu_reserve_mapping *reserve_mapping;

	lockdep_assert_held(&region->lock);

	reserve_mapping = kzalloc(sizeof(*reserve_mapping), GFP_KERNEL);
	if (!reserve_mapping)
		return ERR_PTR(-ENOMEM);

	/* The reserved region must not be returned until @mapping is unmapped. */
	gcip_iommu_reserve_region_get(region);

	reserve_mapping->region = region;
	reserve_mapping->mapping = mapping;
	reserve_mapping->data = data;
	kref_init(&reserve_mapping->kref);

	gcip_iommu_mapping_set_ops(mapping, &reserve_mapping_ops);
	gcip_iommu_mapping_set_data(mapping, reserve_mapping);

	return reserve_mapping;
}

/*
 * Adds @region to the @mgr->regions red-black tree and increments the refcount of @region.
 *
 * The caller must hold @mgr->lock.
 */
static int gcip_iommu_reserve_manager_add_region_locked(struct gcip_iommu_reserve_manager *mgr,
							struct gcip_iommu_reserve_region *region)
{
	struct rb_node **link = &mgr->regions.rb_node, *parent_node = NULL;
	struct gcip_iommu_reserve_region *parent;

	lockdep_assert_held(&mgr->lock);

	while (*link) {
		parent_node = *link;
		parent = rb_entry(parent_node, struct gcip_iommu_reserve_region, node);

		if (parent->base_daddr > region->base_daddr)
			link = &(*link)->rb_left;
		else if (parent->base_daddr < region->base_daddr)
			link = &(*link)->rb_right;
		else
			return -EEXIST;
	}

	gcip_iommu_reserve_region_get(region);
	rb_link_node(&region->node, parent_node, link);
	rb_insert_color(&region->node, &mgr->regions);

	return 0;
}

/*
 * Removes @region from the @mgr->regions red-black tree and decrements the refcount of @region.
 *
 * It is not mandatory to hold @mgr->lock if it is guaranteed that @mgr->regions won't be
 * manipulated in the middle of this function.
 */
static void gcip_iommu_reserve_manager_remove_region(struct gcip_iommu_reserve_manager *mgr,
						     struct gcip_iommu_reserve_region *region)
{
	rb_erase(&region->node, &region->mgr->regions);
	gcip_iommu_reserve_region_put(region);
}

/*
 * Finds the reserved region starting at @base_daddr from @mgr and increments its refocunt.
 *
 * The caller must call the `gcip_iommu_reserve_region_put` function once it doesn't need to access
 * the returned region anymore.
 *
 * Returns the pointer on success. Otherwise, returns a negative errno pointer.
 *
 * The caller must hold @mgr->lock.
 */
static struct gcip_iommu_reserve_region *
gcip_iommu_reserve_manager_get_region_locked(struct gcip_iommu_reserve_manager *mgr,
					     dma_addr_t base_daddr)
{
	struct gcip_iommu_reserve_region *region;
	struct rb_node *node;

	lockdep_assert_held(&mgr->lock);

	node = mgr->regions.rb_node;

	while (node) {
		region = rb_entry(node, struct gcip_iommu_reserve_region, node);

		if (region->base_daddr == base_daddr) {
			gcip_iommu_reserve_region_get(region);
			return region;
		} else if (region->base_daddr > base_daddr) {
			node = node->rb_left;
		} else {
			node = node->rb_right;
		}
	}

	return ERR_PTR(-EINVAL);
}

/*
 * Finds the reserved region which can map the @size of buffer/dma-buf to the @start IOVA address
 * from @mgr and increments its refcount.
 *
 * The caller must call the `gcip_iommu_reserve_region_put` function below once it doesn't need
 * to access the returned region anymore.
 *
 * Returns the pointer on success. Otherwise, returns a negative errno pointer.
 *
 * The caller must hold @mgr->lock.
 */
static struct gcip_iommu_reserve_region *
gcip_iommu_reserve_manager_get_region_fit_locked(struct gcip_iommu_reserve_manager *mgr,
						 dma_addr_t start, size_t size)
{
	struct gcip_iommu_reserve_region *region;
	struct rb_node *node;
	dma_addr_t end = start + size, region_end;

	lockdep_assert_held(&mgr->lock);

	if (end <= start)
		return ERR_PTR(-EINVAL);

	node = mgr->regions.rb_node;

	while (node) {
		region = rb_entry(node, struct gcip_iommu_reserve_region, node);
		region_end = region->base_daddr + region->size;

		if (region->base_daddr <= start) {
			if (region_end >= end) {
				/* Found. @region can cover the buffer. */
				gcip_iommu_reserve_region_get(region);
				return region;
			} else if (region_end <= start) {
				/* @region locates too left from the buffer. Try the right child. */
				node = node->rb_right;
				continue;
			}
		} else if (region->base_daddr >= end) {
			/* @region locates too right from the buffer. Try the left child. */
			node = node->rb_left;
			continue;
		}

		/*
		 * The region and the buffer are overlapped, but the region doesn't fully cover it.
		 * We can't proceed anymore.
		 */
		break;
	}

	return ERR_PTR(-EINVAL);
}

/* Retires @region if it is not retired yet. */
static void gcip_iommu_reserve_region_try_retire(struct gcip_iommu_reserve_region *region)
{
	struct gcip_iommu_reserve_mapping *cur, *tmp;

	mutex_lock(&region->lock);

	/* This function is already called before. */
	if (region->retired) {
		mutex_unlock(&region->lock);
		return;
	}

	/* Makes @region->mappings won't be changed anymore. */
	region->retired = true;

	/*
	 * The `gcip_iommu_reserve_mapping_after_unmap` function still can be called while iterating
	 * @region->mappings below if the IP driver unmaps any mapping. Therefore, even though it is
	 * guaranteed that @region->mappings won't be changed anymore, the refcount of each mapping
	 * can be decremented in parallel. To prevent them from being released during the iteration,
	 * increments the refcount of them.
	 */
	list_for_each_entry(cur, &region->mappings, node) {
		gcip_iommu_reserve_mapping_get(cur);
	}

	mutex_unlock(&region->lock);

	/* Unmaps all mappings which haven't unmapped yet. */
	list_for_each_entry_safe(cur, tmp, &region->mappings, node) {
		list_del_init(&cur->node);
		/*
		 * Calls @unmap callback instead of calling the `gcip_iommu_mapping_unmap` function
		 * directly because IP driver may access @cur->mapping even after this callback
		 * returns by the race condition. The IP driver will prepare their resources for
		 * unmapping the mapping immediately whenever they can guarantee that no one will
		 * access it anymore. The IP driver must call the `gcip_iommu_mapping_unmap`
		 * function by itself at that moment.
		 */
		region->mgr->ops->unmap(region->mgr, cur->mapping, cur->data);
		/*
		 * Releases the refcount which was held above. If the mapping was actually unmapped,
		 * @cur will be released right away.
		 */
		gcip_iommu_reserve_mapping_put(cur);
	}
}

void gcip_iommu_reserve_manager_retire(struct gcip_iommu_reserve_manager *mgr)
{
	struct gcip_iommu_reserve_region *region;
	struct rb_node *node;

	mutex_lock(&mgr->lock);

	/* This function is already called before. */
	if (mgr->retired) {
		mutex_unlock(&mgr->lock);
		return;
	}

	/*
	 * From now on, this is the only function which can retire regions of @mgr. It is guaranteed
	 * that @mgr->regions won't be changed, but also each region won't be released while
	 * iterating @mgr->regions below.
	 */
	mgr->retired = true;

	mutex_unlock(&mgr->lock);

	/* If there are regions which are not yet retired, retires all of them. */
	while ((node = rb_first(&mgr->regions))) {
		region = rb_entry(node, struct gcip_iommu_reserve_region, node);
		/* Retires @region. */
		gcip_iommu_reserve_region_try_retire(region);
		/*
		 * Removes @region from @mgr->regions and decrements the refcount of @region. If
		 * there are no mappings which aren't yet unmapped from @region, @region will be
		 * released.
		 */
		gcip_iommu_reserve_manager_remove_region(mgr, region);
	}

	gcip_iommu_reserve_manager_put(mgr);
}

dma_addr_t gcip_iommu_reserve_region_create(struct gcip_iommu_reserve_manager *mgr, size_t size,
					    u64 gcip_map_flags)
{
	struct gcip_iommu_reserve_region *region;
	struct device *dev = mgr->domain->dev;
	int ret;

	if (!size)
		return 0;

	mutex_lock(&mgr->lock);

	if (mgr->retired) {
		dev_err(dev, "The IOMMU reserve manager is already retired");
		goto err_out;
	}

	region = kzalloc(sizeof(*region), GFP_KERNEL);
	if (!region)
		goto err_out;

	region->base_daddr = gcip_iommu_alloc_iova(mgr->domain, size, gcip_map_flags);
	if (!region->base_daddr) {
		dev_err(dev, "The domain doesn't have enough space to reserve a region, size=%zu",
			size);
		goto err_kfree;
	}

	/* @mgr must not be released until @region is released. */
	gcip_iommu_reserve_manager_get(mgr);

	region->mgr = mgr;
	region->domain = mgr->domain;
	region->size = size;
	mutex_init(&region->lock);
	kref_init(&region->kref);
	INIT_LIST_HEAD(&region->mappings);

	/*
	 * Puts @region to @mgr->regions and increments the refcount of @region. It will be reverted
	 * when @region is retired and the `gcip_iommu_reserve_manager_remove_region` function is
	 * called.
	 */
	ret = gcip_iommu_reserve_manager_add_region_locked(mgr, region);
	if (ret) {
		dev_err(dev, "Failed to add the reserved region to the manager (ret=%d)", ret);
		goto err_put_mgr;
	}

	/* We can decrement the refcount as @mgr->regions is holding it. */
	gcip_iommu_reserve_region_put(region);

	mutex_unlock(&mgr->lock);
	return region->base_daddr;

err_put_mgr:
	gcip_iommu_reserve_manager_put(mgr);
	gcip_iommu_free_iova(mgr->domain, region->base_daddr, size);
err_kfree:
	kfree(region);
err_out:
	mutex_unlock(&mgr->lock);
	return 0;
}

int gcip_iommu_reserve_region_retire(struct gcip_iommu_reserve_manager *mgr, dma_addr_t base_daddr)
{
	struct gcip_iommu_reserve_region *region;

	mutex_lock(&mgr->lock);

	/*
	 * @mgr is retired. all regions must be already or being retired by the
	 * `gcip_iommu_reserve_manager_retire` function.
	 */
	if (mgr->retired) {
		mutex_unlock(&mgr->lock);
		return -EPERM;
	}

	/* Finds the region which starts at @base_daddr and increments its refcount. */
	region = gcip_iommu_reserve_manager_get_region_locked(mgr, base_daddr);
	if (IS_ERR(region)) {
		mutex_unlock(&mgr->lock);
		return PTR_ERR(region);
	}

	/* Removes @region from @mgr->regions and decrements the refcount of @region. */
	gcip_iommu_reserve_manager_remove_region(mgr, region);

	mutex_unlock(&mgr->lock);

	/* Retires @region. */
	gcip_iommu_reserve_region_try_retire(region);

	/*
	 * Decrements the refcount which was held by the `get_region` function above. If there are
	 * no mappings which aren't yet unmapped from @region, @region will be released.
	 */
	gcip_iommu_reserve_region_put(region);

	return 0;
}

struct gcip_iommu_mapping *gcip_iommu_reserve_map_buffer(struct gcip_iommu_reserve_manager *mgr,
							 u64 host_address, size_t size,
							 u64 gcip_map_flags,
							 struct mutex *pin_user_pages_lock,
							 dma_addr_t iova, void *data)
{
	struct gcip_iommu_reserve_region *region;
	struct gcip_iommu_reserve_mapping *reserve_mapping;
	struct gcip_iommu_mapping *mapping;
	u64 offset = host_address & (PAGE_SIZE - 1);
	int ret;

	if (!size || !PAGE_ALIGNED(iova))
		return ERR_PTR(-EINVAL);

	mutex_lock(&mgr->lock);

	if (mgr->retired) {
		mutex_unlock(&mgr->lock);
		return ERR_PTR(-EPERM);
	}

	/*
	 * If @host_address is not page-aligned, the buffer will be mapped to the address which is
	 * shifted from @iova as much as @offset. Therefore, we should consider @offset as the part
	 * of the buffer size while searching a region.
	 *
	 * Additionally, we don't need to care about the page alignment of (size + offset) since
	 * we are just going to find any region where the buffer can fit in. That alignment will be
	 * considered when we actually map it.
	 */
	region = gcip_iommu_reserve_manager_get_region_fit_locked(mgr, iova, size + offset);
	mutex_unlock(&mgr->lock);

	if (IS_ERR(region))
		return ERR_CAST(region);

	mutex_lock(&region->lock);

	if (region->retired) {
		ret = -EPERM;
		goto err_out;
	}

	/*
	 * The mapping function will consider the page alignment of @host_address and @size
	 * internally. If the range of aligned buffer overlaps with other mapped buffers, it will
	 * fail.
	 *
	 * Note that if the reserved region is [0x1000, 0x1500) and we going to map a buffer to
	 * [0x1000, 0x1300), the actual mapped area will be [0x1000, 0x2000) which exceeds the
	 * region. However, it's fine since the region also reserves an area from the IOMMU domain
	 * considering the page alignment. (i.e., the region actually reserves [0x1000, 0x2000))
	 * Therefore, we allow this case since the buffer actually fits into the region.
	 */
	mapping = gcip_iommu_domain_map_buffer_to_iova(region->domain, host_address, size, iova,
						       gcip_map_flags, pin_user_pages_lock);
	if (IS_ERR(mapping)) {
		ret = PTR_ERR(mapping);
		goto err_out;
	}

	reserve_mapping = gcip_iommu_reserve_mapping_alloc_locked(region, mapping, data);
	if (IS_ERR(reserve_mapping)) {
		ret = PTR_ERR(reserve_mapping);
		goto err_unmap;
	}

	list_add_tail(&reserve_mapping->node, &region->mappings);

	mutex_unlock(&region->lock);
	gcip_iommu_reserve_region_put(region);

	return mapping;

err_unmap:
	gcip_iommu_mapping_unmap(mapping);
err_out:
	mutex_unlock(&region->lock);
	gcip_iommu_reserve_region_put(region);
	return ERR_PTR(ret);
}

struct gcip_iommu_mapping *gcip_iommu_reserve_map_dma_buf(struct gcip_iommu_reserve_manager *mgr,
							  struct dma_buf *dmabuf,
							  u64 gcip_map_flags, dma_addr_t iova,
							  void *data)
{
	struct gcip_iommu_reserve_region *region;
	struct gcip_iommu_reserve_mapping *reserve_mapping;
	struct gcip_iommu_mapping *mapping;
	int ret;

	if (!dmabuf || !dmabuf->size || !PAGE_ALIGNED(iova))
		return ERR_PTR(-EINVAL);

	mutex_lock(&mgr->lock);

	if (mgr->retired) {
		mutex_unlock(&mgr->lock);
		return ERR_PTR(-EPERM);
	}

	region = gcip_iommu_reserve_manager_get_region_fit_locked(mgr, iova, dmabuf->size);
	mutex_unlock(&mgr->lock);

	if (IS_ERR(region))
		return ERR_CAST(region);

	mutex_lock(&region->lock);

	if (region->retired) {
		ret = -EPERM;
		goto err_out;
	}

	mapping =
		gcip_iommu_domain_map_dma_buf_to_iova(region->domain, dmabuf, iova, gcip_map_flags);
	if (IS_ERR(mapping)) {
		ret = PTR_ERR(mapping);
		goto err_out;
	}

	reserve_mapping = gcip_iommu_reserve_mapping_alloc_locked(region, mapping, data);
	if (IS_ERR(reserve_mapping)) {
		ret = PTR_ERR(reserve_mapping);
		goto err_unmap;
	}

	list_add_tail(&reserve_mapping->node, &region->mappings);

	mutex_unlock(&region->lock);
	gcip_iommu_reserve_region_put(region);

	return mapping;

err_unmap:
	gcip_iommu_mapping_unmap(mapping);
err_out:
	mutex_unlock(&region->lock);
	gcip_iommu_reserve_region_put(region);
	return ERR_PTR(ret);
}
