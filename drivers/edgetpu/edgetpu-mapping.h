/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Records the mapped TPU IOVA in a device group.
 *
 * Copyright (C) 2019 Google, Inc.
 */
#ifndef __EDGETPU_MAPPING_H__
#define __EDGETPU_MAPPING_H__

#include <linux/device.h>
#include <linux/dma-direction.h>
#include <linux/dma-map-ops.h>
#include <linux/iommu.h>
#include <linux/mutex.h>
#include <linux/rbtree.h>
#include <linux/scatterlist.h>
#include <linux/seq_file.h>
#include <linux/types.h>
#include <linux/version.h>

#include "edgetpu-internal.h"
#include "edgetpu-mmu.h"

struct edgetpu_mapping_root {
	struct rb_root rb;
	struct mutex lock;
	size_t count;
};

/*
 * Use this constant for die index if the mapping is mapped to all dies in a
 * device group.
 */
#define ALL_DIES ((u32)~0)

struct edgetpu_mapping {
	struct rb_node node;
	u64 host_address;
	u32 die_index; /* this mapping is mapped on the @die_index-th die */
	tpu_addr_t device_address;
	/* Size of buffer mapped in bytes. Always set. */
	size_t map_size;
	/*
	 * The size used for allocating @alloc_iova in bytes. This field may be
	 * set by edgetpu_mmu_map().
	 */
	size_t alloc_size;
	/*
	 * This might be different from @device_address since edgetpu_mmu_map()
	 * may allocate more space than the size requested in the reason of
	 * alignment. This field and @alloc_size are expected to be used in
	 * edgetpu_mmu_unmap() and/or to reserve the IOVA space before calling
	 * edgetpu_mmu_map_iova_sgt().
	 */
	tpu_addr_t alloc_iova;
	edgetpu_map_flag_t flags; /* the flag passed by the runtime */
	/* DMA attributes to be performed for dma_(un)map calls. */
	unsigned long dma_attrs;
	struct sg_table sgt;
	enum dma_data_direction dir;
	/* Private data set by whom created this mapping. */
	void *priv;
	/*
	 * This callback will be called when the mappings in
	 * edgetpu_mapping_root are wiped out, i.e. in edgetpu_mapping_clear().
	 * Release/un-map allocated TPU address in this callback.
	 *
	 * The lock of edgetpu_mapping_root is held when calling this callback.
	 *
	 * This callback is called after @map is unlinked from the RB tree, it's
	 * safe to free @map here.
	 *
	 * Note: edgetpu_mapping_unlink() will NOT call this callback.
	 *
	 * This field is mandatory.
	 */
	void (*release)(struct edgetpu_mapping *map);
	/*
	 * Callback for showing the map.
	 *
	 * The lock of edgetpu_mapping_root is held when calling this callback.
	 *
	 * This callback is optional. If this callback is not set, the mapping
	 * will be skipped on showing.
	 */
	void (*show)(struct edgetpu_mapping *map, struct seq_file *s);
};

static inline void edgetpu_mapping_lock(struct edgetpu_mapping_root *root)
{
	mutex_lock(&root->lock);
}

static inline void edgetpu_mapping_unlock(struct edgetpu_mapping_root *root)
{
	mutex_unlock(&root->lock);
}

/* Initializes the mapping structure. */
void edgetpu_mapping_init(struct edgetpu_mapping_root *mappings);

/*
 * Inserts @map into @mappings.
 *
 * Returns 0 on success.
 * Returns -EBUSY if the map already exists.
 */
int edgetpu_mapping_add(struct edgetpu_mapping_root *mappings,
			struct edgetpu_mapping *map);
/*
 * Finds the mapping previously added with edgetpu_mapping_add().
 *
 * Caller holds the mappings lock.
 *
 * Returns NULL if the mapping is not found.
 */
struct edgetpu_mapping *
edgetpu_mapping_find_locked(struct edgetpu_mapping_root *mappings,
			    u32 die_index, tpu_addr_t iova);

/*
 * Removes @map from @mappings.
 *
 * Caller holds the mappings lock.
 */
void edgetpu_mapping_unlink(struct edgetpu_mapping_root *mappings,
			    struct edgetpu_mapping *map);

/*
 * Returns the first map in @mappings.
 *
 * Caller holds the mappings lock.
 *
 * Returns NULL if @mappings is empty.
 */
struct edgetpu_mapping *
edgetpu_mapping_first_locked(struct edgetpu_mapping_root *mappings);

/*
 * Clears added mappings.
 */
void edgetpu_mapping_clear(struct edgetpu_mapping_root *mappings);

/* dump mappings to seq file @s */
void edgetpu_mappings_show(struct edgetpu_mapping_root *mappings,
			   struct seq_file *s);

static inline int __dma_dir_to_iommu_prot(enum dma_data_direction dir)
{
	switch (dir) {
	case DMA_BIDIRECTIONAL:
		return IOMMU_READ | IOMMU_WRITE;
	case DMA_TO_DEVICE:
		return IOMMU_READ;
	case DMA_FROM_DEVICE:
		return IOMMU_WRITE;
	default:
		return 0;
	}
}

/* Returns iommu prot based on @flags and @dir */
static inline int mmu_flag_to_iommu_prot(u32 mmu_flags, struct device *dev,
					 enum dma_data_direction dir)
{
	int prot = 0;

	if (mmu_flags & EDGETPU_MMU_COHERENT) {
#ifdef EDGETPU_IS_DMA_COHERENT
		prot = IOMMU_CACHE;
#endif
	}
	prot |= __dma_dir_to_iommu_prot(dir);
	return prot;
}

/* Return total size of mappings under the supplied root. */
size_t edgetpu_mappings_total_size(struct edgetpu_mapping_root *mappings);

#endif /* __EDGETPU_MAPPING_H__ */
