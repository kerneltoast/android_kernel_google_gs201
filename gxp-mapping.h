/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Records the mapped device addresses.
 *
 * Copyright (C) 2020 Google LLC
 */
#ifndef __GXP_MAPPING_H__
#define __GXP_MAPPING_H__

#include <linux/dma-direction.h>
#include <linux/mutex.h>
#include <linux/rbtree.h>
#include <linux/refcount.h>
#include <linux/scatterlist.h>
#include <linux/types.h>

#include "gxp-internal.h"

struct gxp_mapping {
	struct rb_node node;
	refcount_t refcount;
	void (*destructor)(struct gxp_mapping *mapping);
	/*
	 * User-space address of the mapped buffer.
	 * If this value is 0, it indicates this mapping is for a dma-buf and
	 * should not be used if a regular buffer mapping was expected.
	 */
	u64 host_address;
	struct gxp_dev *gxp;
	uint virt_core_list;
	struct gxp_virtual_device *vd;
	/*
	 * `device_address` and `size` are the base address and size of the
	 * user buffer a mapping represents.
	 *
	 * Due to alignment requirements from hardware, the actual IOVA space
	 * allocated may be larger and start at a different address, but that
	 * information is contained in the scatter-gather table, `sgt` below.
	 */
	dma_addr_t device_address;
	size_t size;
	uint gxp_dma_flags;
	enum dma_data_direction dir;
	struct sg_table sgt;
	/* A mapping can only be synced by one thread at a time */
	struct mutex sync_lock;
	/*
	 * `virtual_address` and `page_count` are set when gxp_mapping_vmap(..)
	 * is called, and unset when gxp_mapping_vunmap(..) is called
	 */
	void *virtual_address;
	u32 page_count;
	uint vmap_count;
	/* Protects `virtual_address`, `page_count`, and `vmap_count` */
	struct mutex vlock;
};

/**
 * gxp_mapping_create() - Create a mapping for a user buffer
 * @gxp: The GXP device to create the mapping for
 * @vd: The virtual device to create the mapping for
 * @virt_core_list: A bitfield indicating the cores in @vd to map the buffer to
 * @user_address: The user-space address of the buffer to map
 * @size: The size of the buffer to be mapped
 * @flags: Flags describing the type of mapping to create; currently unused
 * @dir: DMA direction
 *
 * Upon successful creation, the mapping will be created with a reference count
 * of 1.
 *
 * Return: A pointer to the newly created mapping on success; otherwise an
 *        ERR_PTR:
 * * -ENOMEM: Insufficient memory to create the mapping
 * * -EFAULT: Unable to pin the user pages
 * * -EINVAL: Attempting to map read-only pages for writing by device or failed
 *            to map the buffer for the device.
 */
struct gxp_mapping *gxp_mapping_create(struct gxp_dev *gxp,
				       struct gxp_virtual_device *vd,
				       uint virt_core_list, u64 user_address,
				       size_t size, u32 flags,
				       enum dma_data_direction dir);

/**
 * gxp_mapping_get() - Increment a mapping's reference count
 * @map: The mapping to obtain a reference to
 *
 * Return: True if the mapping's reference count was non-zero and incremented
 *         successfully; false otherwise.
 */
bool gxp_mapping_get(struct gxp_mapping *mapping);

/**
 * gxp_mapping_put() - Decrement a mapping's reference
 * @mapping: The mapping to release a reference to
 */
void gxp_mapping_put(struct gxp_mapping *mapping);

/**
 * gxp_mapping_sync() - Sync a mapped buffer for either CPU or device
 * @mapping: The mapping to sync
 * @offset: The offset, in bytes, into the mapped buffer where the region to
 *          be synced begins
 * @size: The size, in bytes, of the region to be synced
 * @for_cpu: True to sync for CPU access (cache invalidate), false to sync for
 *           device access (cache flush)
 *
 * Return:
 * * 0: Success
 * * -ENODEV: A reference to the mapping could not be obtained
 * * -EINVAL: The specified @offset and @size were not valid
 */
int gxp_mapping_sync(struct gxp_mapping *mapping, u32 offset, u32 size,
		     bool for_cpu);

/**
 * gxp_mapping_vmap() - Map a mapping's buffer into kernel address space
 * @mapping: Tha mapping to map into kernel space
 *
 * If the buffer is already mapped, increments a reference count and returns
 * the existing virtual address instead.
 *
 * Obtains a reference to @mapping if the buffer had not been mapped yet.
 *
 * Return: A pointer to the mapped buffer if successful; otherwise an ERR_PTR:
 * * -ENODEV: A reference to the mapping could not be obtained
 * * -ENOMEM: Insufficient memory to map the buffer
 */
void *gxp_mapping_vmap(struct gxp_mapping *mapping);

/**
 * gxp_mapping_vunmap() - Unmap a mapping from kernel address space
 * @mapping: The mapping to unmap from kernel space
 *
 * Decrements the mapping's vmap reference count, and unmaps the buffer if that
 * count drops to zero.
 *
 * Releases a reference to @mapping if the buffer is unmapped
 */
void gxp_mapping_vunmap(struct gxp_mapping *mapping);

#endif /* __GXP_MAPPING_H__ */
