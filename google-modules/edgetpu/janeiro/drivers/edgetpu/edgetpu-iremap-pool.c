// SPDX-License-Identifier: GPL-2.0
/*
 * Lightweight gen_pool based allocator for memory that is placed at a specific
 * location in the TPU address space (such as a carveout memory)
 *
 * Copyright (C) 2020 Google, Inc.
 */

#include <linux/dma-mapping.h>
#include <linux/genalloc.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/slab.h>

#include "edgetpu-internal.h"
#include "edgetpu-iremap-pool.h"
#include "edgetpu-mmu.h"

struct edgetpu_mempool {
	struct gen_pool *gen_pool;
	void *base_vaddr;
	dma_addr_t base_dma_addr;
	tpu_addr_t base_tpu_addr;
	phys_addr_t base_phys_addr;
	size_t granule;
};

int edgetpu_iremap_pool_create(struct edgetpu_dev *etdev, void *base_vaddr,
			       dma_addr_t base_dma_addr,
			       tpu_addr_t base_tpu_addr,
			       phys_addr_t base_phys_addr, size_t size,
			       size_t granule)
{
	struct edgetpu_mempool *pool;

	if (etdev->iremap_pool) {
		etdev_err(etdev, "Refusing to replace existing iremap pool\n");
		return -EEXIST;
	}

	pool = kmalloc(sizeof(*pool), GFP_KERNEL);
	if (!pool)
		return -ENOMEM;

	pool->gen_pool = gen_pool_create(ilog2(granule), -1);
	if (!pool->gen_pool) {
		kfree(pool);
		etdev_err(etdev, "Failed to create iremap pool\n");
		return -ENOMEM;
	}
	pool->base_vaddr = base_vaddr;
	pool->base_dma_addr = base_dma_addr;
	pool->base_tpu_addr = base_tpu_addr;
	pool->base_phys_addr = base_phys_addr;
	pool->granule = granule;
	if (gen_pool_add(pool->gen_pool, (unsigned long)base_vaddr, size, -1)) {
		gen_pool_destroy(pool->gen_pool);
		kfree(pool);
		etdev_err(etdev, "Failed to add memory to iremap pool\n");
		return -ENOMEM;
	}
	etdev->iremap_pool = pool;
	return 0;
}

void edgetpu_iremap_pool_destroy(struct edgetpu_dev *etdev)
{
	struct edgetpu_mempool *etmempool = etdev->iremap_pool;

	if (!etmempool)
		return;
	gen_pool_destroy(etmempool->gen_pool);
	kfree(etmempool);
	etdev->iremap_pool = NULL;
}

int edgetpu_iremap_alloc(struct edgetpu_dev *etdev, size_t size,
			 struct edgetpu_coherent_mem *mem,
			 enum edgetpu_context_id context_id)
{
	struct edgetpu_mempool *etmempool = etdev->iremap_pool;
	unsigned long addr;
	size_t offset;

	if (!etmempool)
		return edgetpu_alloc_coherent(etdev, size, mem, context_id);

	size = __ALIGN_KERNEL(size, etmempool->granule);
	addr = gen_pool_alloc(etmempool->gen_pool, size);
	if (!addr)
		return -ENOMEM;

	mem->vaddr = (void *)addr;
	offset = mem->vaddr - etmempool->base_vaddr;
	mem->dma_addr = etmempool->base_dma_addr + offset;
	mem->tpu_addr = etmempool->base_tpu_addr + offset;
	mem->phys_addr = etmempool->base_phys_addr + offset;
	mem->size = size;
	etdev_dbg(etdev, "%s @ %pK IOVA = %pad size = %zu", __func__, mem->vaddr, &mem->dma_addr,
		  size);

	return 0;
}

void edgetpu_iremap_free(struct edgetpu_dev *etdev,
			 struct edgetpu_coherent_mem *mem,
			 enum edgetpu_context_id context_id)
{
	struct edgetpu_mempool *etmempool = etdev->iremap_pool;

	if (!etmempool) {
		edgetpu_free_coherent(etdev, mem, context_id);
		return;
	}

	etdev_dbg(etdev, "%s @ %pK IOVA = %pad size = %zu", __func__, mem->vaddr, &mem->dma_addr,
		  mem->size);
	gen_pool_free(etmempool->gen_pool, (unsigned long)mem->vaddr,
		      mem->size);
	mem->vaddr = NULL;
}

int edgetpu_iremap_mmap(struct edgetpu_dev *etdev, struct vm_area_struct *vma,
			struct edgetpu_coherent_mem *mem)
{
	struct edgetpu_mempool *etmempool = etdev->iremap_pool;
	size_t offset;
	phys_addr_t phys;
	int ret;
	unsigned long orig_pgoff = vma->vm_pgoff;
	ulong vma_size, map_size;

#ifdef CONFIG_ARM64
	/*
	 * ARM64 will crash on unaligned access to uncached mappings,
	 * which is the attribute set in edgetpu_mmap before this function is
	 * called.
	 * Mark the VMA's pages as writecombine to avoid this.
	 */
	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);
#endif

	vma->vm_pgoff = 0;
	if (!etmempool) {
		edgetpu_x86_coherent_mem_set_uc(mem);
		ret = dma_mmap_coherent(etdev->dev, vma, mem->vaddr,
					mem->dma_addr, mem->size);
		vma->vm_pgoff = orig_pgoff;
		return ret;
	}

	offset = mem->vaddr - etmempool->base_vaddr;
	phys = etmempool->base_phys_addr + offset;
	etdev_dbg(etdev, "%s: virt = %pK phys = %pap\n", __func__, mem->vaddr, &phys);
	vma_size = vma->vm_end - vma->vm_start;
	map_size = min(vma_size, mem->size);
	ret = remap_pfn_range(vma, vma->vm_start, phys >> PAGE_SHIFT,
			      map_size, vma->vm_page_prot);
	vma->vm_pgoff = orig_pgoff;
	return ret;
}
