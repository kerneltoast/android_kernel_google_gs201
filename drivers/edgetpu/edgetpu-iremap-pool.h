/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Lightweight gen_pool based allocator for memory that is placed at a specific
 * location in the TPU address space (such as a carveout memory)
 *
 * Copyright (C) 2020 Google, Inc.
 */
#ifndef __EDGETPU_IREMAP_POOL_H_
#define __EDGETPU_IREMAP_POOL_H_

#include "edgetpu-internal.h"

/*
 * Create a memory pool with the provided addresses.
 * The etdev->iremap_pool token will be set and used internally in the calls
 * below.
 */
int edgetpu_iremap_pool_create(struct edgetpu_dev *etdev, void *base_vaddr,
			       dma_addr_t base_dma_addr,
			       tpu_addr_t base_tpu_addr,
			       phys_addr_t base_phys_addr, size_t size,
			       size_t granule);

/* Release the resources allocated by the memory pool (if any) */
void edgetpu_iremap_pool_destroy(struct edgetpu_dev *etdev);

/*
 * Attempt to allocate memory in the instruction remap pool if the device
 * has one.
 * Fall back to dma_alloc_coherent and edgetpu_mmu_tpu_map otherwise.
 */
int edgetpu_iremap_alloc(struct edgetpu_dev *etdev, size_t size,
			 struct edgetpu_coherent_mem *mem,
			 enum edgetpu_context_id context_id);

/*
 * Free memory allocated by the function above, either from the instruction
 * remap pool or from dma coherent memory.
 */
void edgetpu_iremap_free(struct edgetpu_dev *etdev,
			 struct edgetpu_coherent_mem *mem,
			 enum edgetpu_context_id context_id);

/*
 * Map memory in the pool to user space. Falls back to dma_mmap_coherent when
 * the device does not have an instruction remap pool.
 */
int edgetpu_iremap_mmap(struct edgetpu_dev *etdev, struct vm_area_struct *vma,
			struct edgetpu_coherent_mem *mem);

#endif /* __EDGETPU_IREMAP_POOL_H_ */
