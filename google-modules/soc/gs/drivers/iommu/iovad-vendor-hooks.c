// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright © 2006-2009, Intel Corporation.
 *
 * Author: Anil S Keshavamurthy <anil.s.keshavamurthy@intel.com>
 */

#include <linux/iova.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <trace/hooks/iommu.h>

union iovad_vendor_hooks {
	struct  {
		bool enable_best_fit  : 1;
		bool enable_max_align : 1;
		u32  max_align_shift  : 4;
		u32  rsvd : 26;
	};
	u64 val;
};

static_assert(sizeof(union iovad_vendor_hooks) == 8);

static void iommu_limit_align_shift(void *unused, struct iova_domain *iovad, unsigned long size,
				    unsigned long *shift)
{
	unsigned long max_align_shift;
	union iovad_vendor_hooks iovad_hooks;

	iovad_hooks.val = iovad->val;
	// if not set "iommu-max-align-shift", keep *shift untouched and return
	if (!iovad_hooks.enable_max_align)
		return;

	max_align_shift = iovad_hooks.max_align_shift + PAGE_SHIFT - iova_shift(iovad);
	*shift = min_t(unsigned long, max_align_shift, *shift);
}

static struct iova *__to_iova(struct rb_node *node)
{
	return rb_entry(node, struct iova, node);
}

/* Insert the iova into domain rbtree by holding writer lock */
static void __iova_insert_rbtree(struct rb_root *root, struct iova *iova, struct rb_node *start)
{
	struct rb_node **new, *parent = NULL;

	new = (start) ? &start : &(root->rb_node);
	/* Figure out where to put new node */
	while (*new) {
		struct iova *this = __to_iova(*new);

		parent = *new;

		if (iova->pfn_lo < this->pfn_lo)
			new = &((*new)->rb_left);
		else if (iova->pfn_lo > this->pfn_lo)
			new = &((*new)->rb_right);
		else {
			WARN_ON(1); /* this should not happen */
			return;
		}
	}
	/* Add new node and rebalance tree. */
	rb_link_node(&iova->node, parent, new);
	rb_insert_color(&iova->node, root);
}

static int __alloc_and_insert_iova_best_fit(struct iova_domain *iovad, unsigned long size,
					    unsigned long limit_pfn, struct iova *new,
					    bool size_aligned)
{
	struct rb_node *curr, *prev;
	struct iova *curr_iova, *prev_iova;
	unsigned long flags;
	unsigned long align_mask = ~0UL;
	struct rb_node *candidate_rb_parent;
	unsigned long new_pfn, candidate_pfn = ~0UL;
	unsigned long gap, candidate_gap = ~0UL;

	if (size_aligned) {
		unsigned long shift = fls_long(size - 1);

		trace_android_rvh_iommu_limit_align_shift(iovad, size, &shift);
		align_mask <<= shift;
	}

	/* Walk the tree backwards */
	spin_lock_irqsave(&iovad->iova_rbtree_lock, flags);
	curr = &iovad->anchor.node;
	prev = rb_prev(curr);
	for (; prev; curr = prev, prev = rb_prev(curr)) {
		curr_iova = rb_entry(curr, struct iova, node);
		prev_iova = rb_entry(prev, struct iova, node);

		limit_pfn = min(limit_pfn, curr_iova->pfn_lo);
		new_pfn = (limit_pfn - size) & align_mask;
		gap = curr_iova->pfn_lo - prev_iova->pfn_hi - 1;
		if ((limit_pfn >= size) && (new_pfn > prev_iova->pfn_hi) && (gap < candidate_gap)) {
			candidate_gap = gap;
			candidate_pfn = new_pfn;
			candidate_rb_parent = curr;
			if (gap == size)
				goto insert;
		}
	}

	curr_iova = rb_entry(curr, struct iova, node);
	limit_pfn = min(limit_pfn, curr_iova->pfn_lo);
	new_pfn = (limit_pfn - size) & align_mask;
	gap = curr_iova->pfn_lo - iovad->start_pfn;
	if (limit_pfn >= size && new_pfn >= iovad->start_pfn && gap < candidate_gap) {
		candidate_gap = gap;
		candidate_pfn = new_pfn;
		candidate_rb_parent = curr;
	}

insert:
	if (candidate_pfn == ~0UL) {
		spin_unlock_irqrestore(&iovad->iova_rbtree_lock, flags);
		return -ENOMEM;
	}

	/* pfn_lo will point to size aligned address if size_aligned is set */
	new->pfn_lo = candidate_pfn;
	new->pfn_hi = new->pfn_lo + size - 1;

	/* If we have 'prev', it's a valid place to start the insertion. */
	__iova_insert_rbtree(&iovad->rbroot, new, candidate_rb_parent);
	spin_unlock_irqrestore(&iovad->iova_rbtree_lock, flags);
	return 0;
}

static void iommu_alloc_insert_iova(void *unused, struct iova_domain *iovad, unsigned long size,
				    unsigned long limit_pfn, struct iova *new_iova,
				    bool size_aligned, int *ret)
{
	union iovad_vendor_hooks iovad_hooks;

	if (!iovad || !ret)
		return;

	iovad_hooks.val = iovad->val;
	if (!iovad_hooks.enable_best_fit) {
		// use default
		*ret = 1;
		return;
	}

	*ret = __alloc_and_insert_iova_best_fit(iovad, size, limit_pfn, new_iova, size_aligned);
}

static void iommu_iovad_init_alloc_algo(void *unused, struct device *dev, struct iova_domain *iovad)
{
	union iovad_vendor_hooks iovad_hooks = { .val = 0 };
	u32 shift = 0;

	if (of_property_read_bool(dev->of_node, "iommu-best-fit-algo") ||
	    of_property_read_bool(dev->of_node, "lwis,iommu-best-fit-algo")) {
		iovad_hooks.enable_best_fit = true;
		dev_info(dev, "using IOVA best fit algorithm.\n");
	}

	if (of_property_read_u32(dev->of_node, "iommu-max-align-shift", &shift) == 0) {
		iovad_hooks.enable_max_align = true;
		iovad_hooks.max_align_shift = shift;
		dev_info(dev, "IOVA max alignment shift %u\n", iovad_hooks.max_align_shift);
	}

	iovad->val = iovad_hooks.val;
}

static int __init iovad_vendor_hooks_init(void)
{
	register_trace_android_rvh_iommu_limit_align_shift(iommu_limit_align_shift, NULL);
	register_trace_android_rvh_iommu_alloc_insert_iova(iommu_alloc_insert_iova, NULL);
	register_trace_android_rvh_iommu_iovad_init_alloc_algo(iommu_iovad_init_alloc_algo, NULL);

	return 0;
}

module_init(iovad_vendor_hooks_init);
MODULE_SOFTDEP("post: samsung_iommu_v9");
MODULE_SOFTDEP("post: samsung_iommu");
MODULE_DESCRIPTION("Google Pixel IOVA Vendor Hooks Module");
MODULE_LICENSE("GPL");
