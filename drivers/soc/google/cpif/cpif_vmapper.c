// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2021 Samsung Electronics.
 *
 */

#include "cpif_vmapper.h"
#include <linux/dma-direction.h>
#include <soc/samsung/exynos-cpif-iommu.h>
#include "modem_v1.h"

struct cpif_va_mapper *cpif_vmap_create(u64 va_start, u64 va_size, u64 item_size,
					u64 instance_size)
{
	struct cpif_va_mapper *vmap;

	vmap = kzalloc(sizeof(struct cpif_va_mapper), GFP_ATOMIC);
	if (vmap == NULL)
		return NULL;

	vmap->va_start = va_start;
	vmap->va_size = va_size;
	vmap->va_end = va_start + va_size;
	vmap->item_size = item_size;
	vmap->instance_size = instance_size;

	if (va_size == item_size) /* no need to use kfifo */
		goto skip_kfifo;
	if (kfifo_alloc(&vmap->sequential_table, va_size / item_size *
			sizeof(struct cpif_vmap_item), GFP_ATOMIC)) {
		mif_err("error on creating sequential table\n");
		kfree(vmap);
		vmap = NULL;
		return NULL;
	}

skip_kfifo:
	cpif_sysmmu_enable();

	return vmap;
}
EXPORT_SYMBOL(cpif_vmap_create);

void cpif_vmap_free(struct cpif_va_mapper *vmap)
{
	if (unlikely(!vmap)) {
		mif_err("no vmap to free\n");
		return;
	}

	kfifo_free(&vmap->sequential_table);
	kfree(vmap);
	vmap = NULL;
}
EXPORT_SYMBOL(cpif_vmap_free);

u64 cpif_vmap_map_area(struct cpif_va_mapper *vmap, u64 item_paddr, u64 instance_paddr)
{
	int err;
	u64 ret;
	struct cpif_vmap_item *temp;

	if (vmap->va_size == vmap->item_size) { /* when va and pa is mapped at once */
		if (vmap->out) {
			mif_err("whole range mapping is done already\n");
			return 0;
		}

		err = cpif_iommu_map(vmap->va_start, item_paddr, vmap->va_size,
					DMA_BIDIRECTIONAL);
		if (unlikely(err)) {
			mif_err("failed to perform iommu mapping\n");
			return 0;
		}
		temp = kzalloc(sizeof(struct cpif_vmap_item), GFP_ATOMIC);
		if (!temp)
			return 0;
		temp->vaddr_base = vmap->va_start;
		temp->paddr_base = item_paddr;
		atomic_set(&temp->ref, 1);
		vmap->out = temp; /* need to be positioned at out for easy unmap */

		mif_debug("va_start: 0x%lX item_paddr: 0x%lX va_size: 0x%lX",
				vmap->va_start, item_paddr, vmap->va_size);

		return vmap->out->vaddr_base;
	}

	if (!vmap->in) {/* first time to map */
		err = cpif_iommu_map(vmap->va_start, item_paddr, vmap->item_size,
					DMA_BIDIRECTIONAL);
		if (unlikely(err)) {
			mif_err_limited("failed to perform iommu mapping\n");
			return 0;
		}
		temp = kzalloc(sizeof(struct cpif_vmap_item), GFP_ATOMIC);
		if (!temp)
			return 0;
		temp->vaddr_base = vmap->va_start;
		temp->paddr_base = item_paddr;
		atomic_set(&temp->ref, 1);
		vmap->in = temp;

		mif_debug("first map: CP addr: 0x%lX AP addr: 0x%lX size: 0x%lX\n",
				vmap->va_start, item_paddr, vmap->item_size);

		return vmap->in->vaddr_base + vmap->item_size - vmap->instance_size;
	}

	/* normal case
	 * if in's vmap item is fully mapped, enqueue that item to sequential_table,
	 *  and create new item
	 */
	if (vmap->in->paddr_base != item_paddr) {
		u64 next_vaddr_base = vmap->in->vaddr_base + vmap->item_size;

		if (next_vaddr_base >= vmap->va_end) /* back to va start */
			next_vaddr_base = vmap->va_start;

		err = cpif_iommu_map(next_vaddr_base, item_paddr, vmap->item_size,
					DMA_BIDIRECTIONAL);

		if (unlikely(err)) {
			mif_err_limited("failed to perform iommu mapping\n");
			return 0;
		}
		temp = kzalloc(sizeof(struct cpif_vmap_item), GFP_ATOMIC);
		if (!temp)
			return 0;
		temp->vaddr_base = next_vaddr_base;
		temp->paddr_base = item_paddr;
		atomic_set(&temp->ref, 1);

		kfifo_in(&vmap->sequential_table, vmap->in,
				sizeof(struct cpif_vmap_item));
		kfree(vmap->in);
		vmap->in = temp;

		mif_debug("normal map: CP addr: 0x%lX AP addr: 0x%lX size: 0x%lX\n",
				next_vaddr_base, item_paddr, vmap->item_size);

		ret = vmap->in->vaddr_base + vmap->item_size - vmap->instance_size;
	} else { /* item "in" still has room to use, no need to iommu this time */
		atomic_inc(&vmap->in->ref);
		ret = vmap->in->vaddr_base + vmap->item_size -
				(atomic_read(&vmap->in->ref) * vmap->instance_size);
	}

	return ret;
}
EXPORT_SYMBOL(cpif_vmap_map_area);

u64 cpif_vmap_unmap_area(struct cpif_va_mapper *vmap, u64 vaddr)
{
	int err = 0;
	u64 ret = 0;
	struct cpif_vmap_item *temp;
	struct cpif_vmap_item *target;

	if (vmap->va_size == vmap->item_size) { /* when va and pa is mapped at once */

		err = cpif_iommu_unmap(vmap->va_start, vmap->va_size);
		if (unlikely(err == 0)) {
			mif_err_limited("failed to perform iommu mapping\n");
			return 0;
		}
		kfree(vmap->out);
		vmap->out = NULL;

		return vmap->va_start;
	}

	if (unlikely(!vmap->out)) { /* first time to unmap */
		temp = kzalloc(sizeof(struct cpif_vmap_item), GFP_ATOMIC);
		if (unlikely(!temp)) {
			mif_err_limited("failed to kzalloc for kfifo_out\n");
			return 0;
		}
		if (kfifo_out(&vmap->sequential_table, temp,
				sizeof(struct cpif_vmap_item)) !=
				sizeof(struct cpif_vmap_item)) {
			mif_err("kfifo_out fails\n");
			kfree(temp);
			return 0;
		}
		vmap->out = temp;
	}

	target = vmap->out;

	if (unlikely(vaddr < target->vaddr_base || vaddr > target->vaddr_base +
				vmap->item_size)) {
		mif_err_limited("invalid vaddr 0x%lX vbase: 0x%lX vend: 0x%lX\n",
				vaddr, target->vaddr_base,
				target->vaddr_base + vmap->item_size);
		panic("invalid vaddr\n");
		return 0;
	}

	atomic_dec(&target->ref);

	ret = target->paddr_base + (vaddr - target->vaddr_base);

	/* unmap this item when ref count goes to 0 */
	if (atomic_read(&target->ref) == 0) {
		mif_debug("about to iommu unmap vaddr_base: 0x%lX item_size: 0x%lX\n",
				target->vaddr_base, vmap->item_size);
		err = cpif_iommu_unmap(target->vaddr_base, vmap->item_size);
		if (err == 0) {
			mif_err_limited("failed to unmap\n");
			return 0;
		}
		kfree(vmap->out);
		/* update vmap->out to the next item to be unmapped */
		temp = kzalloc(sizeof(struct cpif_vmap_item), GFP_ATOMIC);
		if (unlikely(!temp))
			return 0;
		if (kfifo_out(&vmap->sequential_table, temp,
				sizeof(struct cpif_vmap_item)) !=
				sizeof(struct cpif_vmap_item)) {
			mif_err_limited("kfifo_out fails: fifo might be empty\n");
			kfree(temp);
			if (vmap->in) {
				/* drain out rest, next map will start from beginning */
				mif_info("drain out vmap->in\n");
				vmap->out = vmap->in;
				vmap->in = NULL;
			} else /* last of last, initialize vmap->out */
				vmap->out = NULL;

			return ret;
		}
		vmap->out = temp;
	}

	return ret;
}
EXPORT_SYMBOL(cpif_vmap_unmap_area);
