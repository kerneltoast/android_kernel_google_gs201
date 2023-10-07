// SPDX-License-Identifier: GPL-2.0
/*
 * Records the mapped device addresses.
 *
 * Copyright (C) 2021 Google LLC
 */

#include <linux/dma-mapping.h>
#include <linux/ktime.h>
#include <linux/mm.h>
#include <linux/mmap_lock.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include "gxp-client.h"
#include "gxp-debug-dump.h"
#include "gxp-dma.h"
#include "gxp-internal.h"
#include "gxp-mapping.h"

#if IS_ENABLED(CONFIG_GXP_TEST)
/* expose this variable to have unit tests set it dynamically */
bool gxp_log_iova;
#else
static bool gxp_log_iova;
#endif

module_param_named(log_iova, gxp_log_iova, bool, 0660);

void gxp_mapping_iova_log(struct gxp_client *client, struct gxp_mapping *map,
			  u8 mask)
{
	static bool is_first_log = true;
	struct device *dev = client->gxp->dev;
	const char *op = mask & GXP_IOVA_LOG_MAP ? "MAP" : "UNMAP";
	const char *buf_type = mask & GXP_IOVA_LOG_DMABUF ? "DMABUF" : "BUFFER";

	if (likely(!gxp_log_iova))
		return;

	if (is_first_log) {
		dev_info(
			dev,
			"iova_log_start: operation, buf_type, tgid, pid, host_address, device_address, size");
		is_first_log = false;
	}

	dev_info(dev, "iova_log: %s, %s, %d, %d, %#llx, %#llx, %zu", op,
		 buf_type, client->pid, client->tgid, map->host_address,
		 map->device_address, map->size);
}

/* Destructor for a mapping created with `gxp_mapping_create()` */
static void destroy_mapping(struct gxp_mapping *mapping)
{
	struct sg_page_iter sg_iter;
	struct page *page;

	mutex_destroy(&mapping->vlock);
	mutex_destroy(&mapping->sync_lock);

	/*
	 * Unmap the user pages
	 *
	 * Normally on unmap, the entire mapping is synced back to the CPU.
	 * Since mappings are made at a page granularity regardless of the
	 * underlying buffer's size, they can cover other data as well. If a
	 * user requires a mapping be synced before unmapping, they are
	 * responsible for calling `gxp_mapping_sync()` before hand.
	 */
	gxp_dma_unmap_sg(mapping->gxp, mapping->domain, mapping->sgt.sgl,
			 mapping->sgt.orig_nents, mapping->dir,
			 DMA_ATTR_SKIP_CPU_SYNC);

	/* Unpin the user pages */
	for_each_sg_page(mapping->sgt.sgl, &sg_iter, mapping->sgt.orig_nents,
			 0) {
		page = sg_page_iter_page(&sg_iter);
		if (mapping->dir == DMA_FROM_DEVICE ||
		    mapping->dir == DMA_BIDIRECTIONAL) {
			set_page_dirty(page);
		}

		unpin_user_page(page);
	}

	/* Free the mapping book-keeping */
	sg_free_table(&mapping->sgt);
	kfree(mapping);
}

struct gxp_mapping *gxp_mapping_create(struct gxp_dev *gxp,
				       struct gxp_iommu_domain *domain,
				       u64 user_address, size_t size, u32 flags,
				       enum dma_data_direction dir)
{
	struct gxp_mapping *mapping = NULL;
	uint num_pages = 0;
	struct page **pages;
	ulong offset;
	int ret, i;
	struct vm_area_struct *vma;
	unsigned int foll_flags = FOLL_LONGTERM | FOLL_WRITE;

	/* Check whether dir is valid or not */
	if (!valid_dma_direction(dir))
		return ERR_PTR(-EINVAL);

	if (!access_ok((const void *)user_address, size)) {
		dev_err(gxp->dev, "invalid address range in buffer map request");
		return ERR_PTR(-EFAULT);
	}

	/*
	 * The host pages might be read-only and could fail if we attempt to pin
	 * it with FOLL_WRITE.
	 * default to read/write if find_extend_vma returns NULL
	 */
	mmap_read_lock(current->mm);
	vma = find_extend_vma(current->mm, user_address & PAGE_MASK);
	if (vma) {
		if (!(vma->vm_flags & VM_WRITE))
			foll_flags &= ~FOLL_WRITE;
	} else {
		dev_dbg(gxp->dev,
			"unable to find address in VMA, assuming buffer writable");
	}
	mmap_read_unlock(current->mm);

	/* Pin the user pages */
	offset = user_address & (PAGE_SIZE - 1);
	if (unlikely((size + offset) / PAGE_SIZE >= UINT_MAX - 1 ||
		     size + offset < size))
		return ERR_PTR(-EFAULT);
	num_pages = (size + offset) / PAGE_SIZE;
	if ((size + offset) % PAGE_SIZE)
		num_pages++;

	/*
	 * "num_pages" is decided from user-space arguments, don't show warnings
	 * when facing malicious input.
	 */
	pages = kvmalloc((num_pages * sizeof(*pages)), GFP_KERNEL | __GFP_NOWARN);
	if (!pages) {
		dev_err(gxp->dev, "Failed to alloc pages for mapping: num_pages=%u",
			num_pages);
		return ERR_PTR(-ENOMEM);
	}

	/*
	 * Provide protection around `pin_user_pages_fast` since it fails if
	 * called by more than one thread simultaneously.
	 */
	mutex_lock(&gxp->pin_user_pages_lock);
	ret = pin_user_pages_fast(user_address & PAGE_MASK, num_pages,
				  foll_flags, pages);
	if (ret == -EFAULT && !vma) {
		dev_warn(gxp->dev,
			 "pin failed with fault, assuming buffer is read-only");
		ret = pin_user_pages_fast(user_address & PAGE_MASK, num_pages,
					  foll_flags & ~FOLL_WRITE, pages);
	}
	mutex_unlock(&gxp->pin_user_pages_lock);
	if (ret == -ENOMEM)
		dev_err(gxp->dev, "system out of memory locking %u pages",
			num_pages);
	if (ret == -EFAULT)
		dev_err(gxp->dev, "address fault mapping %s buffer",
			dir == DMA_TO_DEVICE ? "read-only" : "writeable");
	if (ret < 0 || ret < num_pages) {
		dev_dbg(gxp->dev,
			"Get user pages failed: user_add=%pK, num_pages=%u, ret=%d\n",
			(void *)user_address, num_pages, ret);
		num_pages = ret < 0 ? 0 : ret;
		ret = ret >= 0 ? -EFAULT : ret;
		goto error_unpin_pages;
	}

	/* Initialize mapping book-keeping */
	mapping = kzalloc(sizeof(*mapping), GFP_KERNEL);
	if (!mapping) {
		ret = -ENOMEM;
		goto error_unpin_pages;
	}
	refcount_set(&mapping->refcount, 1);
	mapping->destructor = destroy_mapping;
	mapping->host_address = user_address;
	mapping->gxp = gxp;
	mapping->domain = domain;
	mapping->size = size;
	mapping->gxp_dma_flags = flags;
	mapping->dir = dir;
	ret = sg_alloc_table_from_pages(&mapping->sgt, pages, num_pages, 0,
					num_pages * PAGE_SIZE, GFP_KERNEL);
	if (ret) {
		dev_err(gxp->dev, "Failed to alloc sgt for mapping (ret=%d)\n",
			ret);
		goto error_free_sgt;
	}

	/* map the user pages */
	ret = gxp_dma_map_sg(gxp, mapping->domain, mapping->sgt.sgl,
			     mapping->sgt.nents, mapping->dir,
			     DMA_ATTR_SKIP_CPU_SYNC, mapping->gxp_dma_flags);
	if (!ret) {
		dev_err(gxp->dev, "Failed to map sgt (ret=%d)\n", ret);
		ret = -EINVAL;
		goto error_free_sgt;
	}
	mapping->sgt.nents = ret;
	mapping->device_address =
		sg_dma_address(mapping->sgt.sgl) + offset;

	mutex_init(&mapping->sync_lock);
	mutex_init(&mapping->vlock);

	kvfree(pages);
	return mapping;

error_free_sgt:
	sg_free_table(&mapping->sgt);
	kfree(mapping);
error_unpin_pages:
	for (i = 0; i < num_pages; i++)
		unpin_user_page(pages[i]);
	kvfree(pages);

	return ERR_PTR(ret);
}

bool gxp_mapping_get(struct gxp_mapping *mapping)
{
	return refcount_inc_not_zero(&mapping->refcount);
}

void gxp_mapping_put(struct gxp_mapping *mapping)
{
	/* `refcount_dec_and_test()` returns true if the refcount drops to 0 */
	if (refcount_dec_and_test(&mapping->refcount))
		mapping->destructor(mapping);
}

int gxp_mapping_sync(struct gxp_mapping *mapping, u32 offset, u32 size,
		     bool for_cpu)
{
	struct gxp_dev *gxp = mapping->gxp;
	struct scatterlist *sg, *start_sg = NULL, *end_sg = NULL;
	int nelems = 0, cur_offset = 0, ret = 0, i;
	u64 start, end;
	unsigned int start_diff = 0, end_diff = 0;

	if (!gxp_mapping_get(mapping))
		return -ENODEV;

	/* Only mappings with valid `host_address`es can be synced */
	if (!mapping->host_address) {
		ret = -EINVAL;
		goto out;
	}

	/*
	 * Valid input requires
	 * - size > 0 (offset + size != offset)
	 * - offset + size does not overflow (offset + size > offset)
	 * - the mapped range falls within [0 : mapping->size]
	 */
	if (offset + size <= offset ||
	    offset + size > mapping->size) {
		ret = -EINVAL;
		goto out;
	}

	/*
	 * Since the scatter-gather list of the mapping is modified while it is
	 * being synced, only one sync for a given mapping can occur at a time.
	 * Rather than maintain a mutex for every mapping, lock the mapping list
	 * mutex, making all syncs mutually exclusive.
	 */
	mutex_lock(&mapping->sync_lock);
	/*
	 * Mappings are created at a PAGE_SIZE granularity, however other data
	 * which is not part of the mapped buffer may be present in the first
	 * and last pages of the buffer's scattergather list.
	 *
	 * To ensure only the intended data is actually synced, iterate through
	 * the scattergather list, to find the first and last `scatterlist`s
	 * which contain the range of the buffer to sync.
	 *
	 * After those links are found, change their offset/lengths so that
	 * `dma_map_sg_for_*()` will only sync the requested region.
	 */
	start = (mapping->host_address & ~PAGE_MASK) + offset;
	end = start + size;
	for_each_sg(mapping->sgt.sgl, sg, mapping->sgt.orig_nents, i) {
		if (end <= cur_offset)
			break;
		if (cur_offset <= start && start < cur_offset + sg->length) {
			start_sg = sg;
			start_diff = start - cur_offset;
		}
		if (start_sg)
			nelems++;
		cur_offset += sg->length;
		end_sg = sg;
	}
	end_diff = cur_offset - end;

	/* Make sure a valid starting scatterlist was found for the start */
	if (!start_sg) {
		ret = -EINVAL;
		goto out_unlock;
	}

	start_sg->offset += start_diff;
	start_sg->dma_address += start_diff;
	start_sg->length -= start_diff;
	start_sg->dma_length -= start_diff;
	end_sg->length -= end_diff;
	end_sg->dma_length -= end_diff;

	if (for_cpu)
		gxp_dma_sync_sg_for_cpu(gxp, start_sg, nelems, mapping->dir);
	else
		gxp_dma_sync_sg_for_device(gxp, start_sg, nelems, mapping->dir);

	/*
	 * Return the start and end scatterlists' offset/lengths to their
	 * original values for the next time they need to be synced/unmapped.
	 */
	end_sg->length += end_diff;
	end_sg->dma_length += end_diff;
	start_sg->offset -= start_diff;
	start_sg->dma_address -= start_diff;
	start_sg->length += start_diff;
	start_sg->dma_length += start_diff;

out_unlock:
	mutex_unlock(&mapping->sync_lock);
out:
	gxp_mapping_put(mapping);

	return ret;
}

void *gxp_mapping_vmap(struct gxp_mapping *mapping)
{
	struct sg_table *sgt;
	struct sg_page_iter sg_iter;
	struct page **pages;
	void *vaddr;
	int i = 0;
	u32 page_count = 0;

	if (!gxp_mapping_get(mapping))
		return ERR_PTR(-ENODEV);

	mutex_lock(&mapping->vlock);

	/* Check if user buffer has already been mapped to kernel */
	if (mapping->vmap_count) {
		vaddr = mapping->virtual_address;
		mapping->vmap_count++;
		goto out;
	}

	sgt = &mapping->sgt;
	for_each_sg_page(sgt->sgl, &sg_iter, sgt->orig_nents, 0)
		page_count++;

	pages = kvmalloc((page_count * sizeof(*pages)), GFP_KERNEL);
	if (!pages) {
		vaddr = ERR_PTR(-ENOMEM);
		goto out;
	}

	for_each_sg_page(sgt->sgl, &sg_iter, sgt->orig_nents, 0)
		pages[i++] = sg_page_iter_page(&sg_iter);

	vaddr = vmap(pages, page_count, VM_MAP, PAGE_KERNEL);
	kvfree(pages);
	if (vaddr == NULL) {
		dev_err(mapping->gxp->dev,
			"Failed to map user buffer to kernel");
		vaddr = ERR_PTR(-ENOMEM);
		goto out;
	}

	mapping->virtual_address = vaddr;
	mapping->page_count = page_count;
	mapping->vmap_count = 1;

	/* Hold a reference to the mapping so long as it is vmapped */
	gxp_mapping_get(mapping);

out:
	mutex_unlock(&mapping->vlock);

	gxp_mapping_put(mapping);

	return vaddr;
}

void gxp_mapping_vunmap(struct gxp_mapping *mapping)
{
	if (!gxp_mapping_get(mapping))
		return;

	mutex_lock(&mapping->vlock);

	/*
	 * Exit immediately if the mapping was never vmapped, or still has
	 * other users expecting it to be vmapped.
	 */
	if (!mapping->vmap_count || --mapping->vmap_count)
		goto out;

	vunmap(mapping->virtual_address);
	mapping->virtual_address = 0;
	mapping->page_count = 0;

	/* Release the reference from gxp_mapping_vmap() */
	gxp_mapping_put(mapping);

out:
	mutex_unlock(&mapping->vlock);

	gxp_mapping_put(mapping);
}
