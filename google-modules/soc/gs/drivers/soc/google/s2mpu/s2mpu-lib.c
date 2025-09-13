// SPDX-License-Identifier: GPL-2.0-only
/*
 * Platform device driver for S2MPU.
 *
 * Copyright (C) 2020 Google LLC.
 */
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>
#include <linux/bits.h>
#include <linux/irq.h>

#include "s2mpu-regs.h"
#include "s2mpu-lib.h"
#include "ssmt.h"

/* we can use __raw_readl() and __raw_writel() here instead of readl() and
 * writel() which are overkill and slower. memory mapped as nGnRE so cpu
 * wouldn't re-order those writes instructions. __raw_read/writel() use
 * volatile keyword so compiler wouldn't re-order those instructions either.
 * use readl() and writel() when accessing nodes whose physical path from cpu
 * diverges.
 */

#define WHI_PHY_ADDR_MASK (0xfffffffff)
#define WHI_RESERVED_START (0x100000000)
#define WHI_RESERVED_END (0x87fffffff)

#define INT_SHIFT 32

#define extend_to_byte(two_bits) ((u8)(((two_bits) << 6) | ((two_bits) << 4) | ((two_bits) << 2) \
				       | ((two_bits) << 0)))
#define s2_table_size(gshift) (((1 << 30) >> (gshift)) >> 2)
#define is_gran_aligned(val, g) (((val) & (WHI_PHY_ADDR_MASK << gran_shift[(g)] \
					   & WHI_PHY_ADDR_MASK)) != (val))

enum s2_gran {
	S2_GRAN_INVALID = -1,
	S2_GRAN_4KB = 0,
	S2_GRAN_64KB = 1,
	S2_GRAN_2MB = 2,
	S2_GRAN_1GB = 3,
};

enum dir_mask {
	DIR_NONE,
	DIR_READ = 1,
	DIR_WRITE,
	DIR_BIDRECTIONAL
};

const u32 gran_shift[] = {
	[S2_GRAN_4KB] = 12,
	[S2_GRAN_64KB] = 16,
	[S2_GRAN_2MB] = 21,
	[S2_GRAN_1GB] = 30,
};

static struct smpt_region *smpt_region_for_pa(struct s2mpu_info *info, dma_addr_t pa)
{
	struct smpt_region *tmp, *r = NULL;

	list_for_each_entry(tmp, &info->smpt_regions, list) {
		if (tmp->pa == pa) {
			r = tmp;
			break;
		}
	}

	return r;
}

/* for smpt we need physically contiguous chunks of following sizes which are
 * also aligned:
 * - 64KB aligned for 4KB granularity
 * - 4KB aligned for 64KB granularity
 * - 128 bytes aligned for 2MB granularity
 * this should be okay with kmalloc using SLAB underneath which is default.
 * SLUB will also be fine when debugging is not enabled. however, following two
 * scenarios will be problematic:
 * 1. SLUB with debugging enabled
 * 2. SLOB allocator
 *
 * it seems quite a bit of code in kernel relies upon kmalloc to return aligned
 * buffers. e.g. xfs depends upon this.
 *
 * for more info see https://lwn.net/Articles/787740/
 *
 * NOTE: in 5.4 kernel kmalloc guarantees returned buffer to be aligned to the
 * size if size is power of 2: https://elixir.bootlin.com/linux/v5.4.51/source/include/linux/slab.h#L497
 */
static struct smpt_region *alloc_smpt(struct s2mpu_info *info, size_t len)
{
	struct smpt_region *r;

	/* len must be one of 64kb, 4kb or 128b */
	if (WARN_ON(len != 128 && len != 4096 && len != (64 * 1024)))
		return NULL;

	r = kmalloc(sizeof(*r), GFP_ATOMIC);
	if (unlikely(!r))
		return NULL;

	r->va = dma_alloc_coherent(info->dev, len, &r->pa, GFP_ATOMIC);
	if (unlikely(!r->va)) {
		dev_err(info->dev, "dma_alloc_coherent failed len=%zu\n", len);
		goto out_free;
	}

	r->len = len;
	memset(r->va, 0, len);
	list_add(&r->list, &info->smpt_regions);

	return r;

out_free:
	kfree(r);
	r = NULL;
	return r;
}

static void free_smpt(struct s2mpu_info *info, void *buf)
{
	struct smpt_region *tmp, *r = NULL;

	if (WARN_ON(!buf))
		return;

	list_for_each_entry(tmp, &info->smpt_regions, list) {
		if (tmp->va == buf) {
			r = tmp;
			break;
		}
	}

	if (!r) {
		dev_warn(info->dev, "region not found. failed to free it. something wrong\n");
		return;
	}

	list_del(&r->list);
	dma_free_coherent(info->dev, r->len, r->va, r->pa);
	kfree(r);
}

void s2mpu_lib_deinit(struct s2mpu_info *info)
{
	u32 reg, gb_index;
	void __iomem *base = info->base;
	struct smpt_region *curr, *next;

	/* disable access to all physical memory at L1 level */
	reg = (0 << S2MPU_L1ENTRY_ATTR_L2TABLE_EN_SHIFT) |
			(0 << S2MPU_L1ENTRY_ATTR_RD_ACCESS_SHIFT) |
			(0 << S2MPU_L1ENTRY_ATTR_WR_ACCESS_SHIFT) |
			(0 << S2MPU_L1ENTRY_ATTR_L2TABLE_GRANULE_SHIFT);

	/* note that info->vid is used. info->vid is either 0 or 1. whatever
	 * value of vid was used, we just need to block that.
	 */
	for (gb_index = 0; gb_index < 4; gb_index++)
		__raw_writel(reg, S2MPU_L1ENTRY_ATTR(base, info->vid, gb_index));
	/* accommodate the gap in physical address space */
	for (gb_index = 34; gb_index < 64; gb_index++)
		__raw_writel(reg, S2MPU_L1ENTRY_ATTR(base, info->vid, gb_index));

	/* make sure that the S2MPU is enabled and hence blocking traffic */
	reg = __raw_readl(S2MPU_CTRL0(base));
	if ((reg & 1) == 0) {
		reg = reg | 1;
		__raw_writel(reg, S2MPU_CTRL0(base));
	}

	/* invalidate all memory protection cache */
	reg = (1 << S2MPU_ALL_INVALIDATION_INVALIDATE_SHIFT) |
			(0 << S2MPU_ALL_INVALIDATION_VID_SPECIFIC_SHIFT);
	__raw_writel(reg, S2MPU_ALL_INVALIDATION(base));

	/* unmap SMPT and free up memory allocated in s2mpu_dev_init() */
	list_for_each_entry_safe(curr, next, &info->smpt_regions, list) {
		list_del(&curr->list);

		dma_free_coherent(info->dev, curr->len, curr->va, curr->pa);
		kfree(curr);
	}
}

/* print fault info and clear fault status in s2mpu fault register */
irqreturn_t s2mpu_lib_irq_handler(int irq, void *data)
{
	u32 fault_status, pa_low, pa_high, fault_info, fault_type;
	struct s2mpu_info *info = data;
	phys_addr_t full_pa;

	fault_status = __raw_readl(S2MPU_FAULT_STATUS(info->base));
	pa_low = __raw_readl(S2MPU_FAULT_PA_LOW(info->base, info->vid));
	pa_high = __raw_readl(S2MPU_FAULT_PA_HIGH(info->base, info->vid)) & 0xf;
	fault_info = __raw_readl(S2MPU_FAULT_INFO(info->base, info->vid));
	fault_type = S2MPU_FAULT_TYPE(fault_info);

	full_pa = pa_high;
	full_pa = (full_pa << INT_SHIFT) | pa_low;

	dev_err(info->dev, "IRQ (%d): FAULT_STATUS=0x%x\nPA=0x%pap\nDIRECTION=%s\nFAULT_TYPE=%s\nFAULT_INFO=0x%x\n",
		irq, fault_status, &full_pa, fault_info & BIT(20) ? "write" : "read",
		fault_type == 2 ? "access perm fault" : fault_type == 1 ? "ptw fault" : "[n/a]",
		fault_info);

	__raw_writel(0xff, S2MPU_INTERRUPT_CLEAR(info->base));

	return IRQ_HANDLED;
}

/* this does two things:
 * 1. initialise s2mpu_info and related data structures
 * 2. initialise the hardware registers of the device
 */
struct s2mpu_info *s2mpu_lib_init(struct device *dev, void __iomem *base,
				  void __iomem *ssmt_base, u8 vid, u32 *sids,
				  unsigned int sidcount)
{
	u32 reg;
	int count, gb_index;
	struct s2mpu_info *info;

	if (vid > 1) {
		pr_err("vid (%u) must be 0 or 1. failed to initialise s2mpu.\n", vid);
		return NULL;
	}

	info = devm_kmalloc(dev, sizeof(struct s2mpu_info), GFP_KERNEL);
	if (!info)
		return ERR_PTR(-ENOMEM);
	info->base = base;
	info->ssmt_base = ssmt_base;
	info->dev = dev;
	info->sids = sids;
	info->sidcount = sidcount;
	info->vid = vid;
	INIT_LIST_HEAD(&info->smpt_regions);

	spin_lock_init(&info->lock);

	/* first make sure that S2MPU is disabled */
	__raw_writel(0, S2MPU_CTRL0(base));

	/* ensure we don't skip SysMMU PTW permission checks */
	reg = (0 << S2MPU_CTRL1_DISABLE_CHK_S1L1PTW_SHIFT) |
			(0 << S2MPU_CTRL1_DISABLE_CHK_S1L2PTW_SHIFT) |
			/* TODO: page-size awareness is an optimisation we
			 * should return to after we have everything working
			 */
			(0 << S2MPU_CTRL1_ENABLE_PAGE_SIZE_AWARENESS_SHIFT) |
			/* this only applies when S2MPU is in path but no
			 * SysMMU. WHI has no such situation.
			 */
			(0 << S2MPU_CTRL1_DISABLE_CHK_USER_MATCHED_REQ_SHIFT);
	__raw_writel(reg, S2MPU_CTRL1(base));

	/* no SLC caching for S2MPU PTWs */
	reg = (0 << S2MPU_CFG_MPTW_CACHE_OVERRIDE_SHIFT) |
			(0 << S2MPU_CFG_MPTW_CACHE_VALUE_SHIFT) |
			(0 << S2MPU_CFG_MPTW_QOS_OVERRIDE_SHIFT) |
			(0 << S2MPU_CFG_MPTW_SHAREABLE_SHIFT);
	__raw_writel(reg, S2MPU_CFG(base));

	/* based on discussions with SLC PoC, PID = 0 effectively means SLC
	 * bypass.
	 */
	reg = (0 << S2MPU_CFG_MPTW_USER_LOW_PBHA_SHIFT) |
			(0 << S2MPU_CFG_MPTW_USER_LOW_PID_SHIFT) |
			/* TODO: check with SLC PoC that VC related to SLC.
			 * if it is then it shouldn't matter what value we set
			 * as long as we bypass SLC with PID = 0
			 */
			(0 << S2MPU_CFG_MPTW_USER_LOW_VC_SHIFT);
	__raw_writel(reg, S2MPU_CFG_MPTW_USER_LOW(base));

	/* TODO: check if this is related to SLC and whether it matters if
	 * we bypass SLC with PID = 0
	 */
	reg = (0 << S2MPU_CFG_MPTW_USER_HIGH_EXT_DOMAIN_SHIFT);
	__raw_writel(reg, S2MPU_CFG_MPTW_USER_HIGH(base));

	/* enable per-vid interrupts */
	__raw_writel(0xff, S2MPU_INTERRUPT_ENABLE_PER_VID_SET(base));

	/* TODO: come back to this when optimising performance */
	reg = (0 << S2MPU_MPC_CTRL_RD_CH_TKN_SHIFT) |
			(0 << S2MPU_MPC_CTRL_WR_CH_TKN_SHIFT);
	__raw_writel(reg, S2MPU_MPC_CTRL(base));

	/* it doesn't matter what we set for page aware decoding registers
	 * because we have disabled this feature. here setting them to reset
	 * values
	 */
	reg = 0x00f200f1;
	__raw_writel(reg, S2MPU_PAGE_AWARE_DECODING_0(base));
	__raw_writel(reg, S2MPU_PAGE_AWARE_DECODING_1(base));

	/* disable performance measurement counter */
	reg = (0 << S2MPU_PM_CFG_CNT_EN_SHIFT);
	__raw_writel(reg, S2MPU_PM_CFG(base));

	/* TODO: we may have to enable reading from MPTC for testing purposes */

	/* disable access to all physical memory at L1 level */
	reg = (0 << S2MPU_L1ENTRY_ATTR_L2TABLE_EN_SHIFT) |
			(0 << S2MPU_L1ENTRY_ATTR_RD_ACCESS_SHIFT) |
			(0 << S2MPU_L1ENTRY_ATTR_WR_ACCESS_SHIFT) |
			(0 << S2MPU_L1ENTRY_ATTR_L2TABLE_GRANULE_SHIFT);

	for (count = 0; count < 8; count++)
		for (gb_index = 0; gb_index < 4; gb_index++)
			__raw_writel(reg, S2MPU_L1ENTRY_ATTR(base, count, gb_index));

	for (gb_index = 0; gb_index < 4; gb_index++)
		info->pt.l1entry_attrs[gb_index] = reg;

	/* physical address range from 0x1_0000_0000 to 0x8_7FFF_FFFF is
	 * a reserved region and not mapped to any slaves, so we put in
	 * a gap here.
	 */
	for (count = 0; count < 8; count++)
		for (gb_index = 34; gb_index < 64; gb_index++)
			__raw_writel(reg, S2MPU_L1ENTRY_ATTR(base, count, gb_index));

	for (gb_index = 34; gb_index < 64; gb_index++)
		info->pt.l1entry_attrs[gb_index] = reg;

	/* if vid is 1 then open s2mpu for vid 0 as there are other devices
	 * behind the s2mpu which should not be restricted. we also need to
	 * program ssmt to assign vid 1 to the stream id's that we want to
	 * restrict.
	 */
	if (info->vid == 1) {
		reg = (0 << S2MPU_L1ENTRY_ATTR_L2TABLE_EN_SHIFT) |
			(1 << S2MPU_L1ENTRY_ATTR_RD_ACCESS_SHIFT) |
			(1 << S2MPU_L1ENTRY_ATTR_WR_ACCESS_SHIFT) |
			(0 << S2MPU_L1ENTRY_ATTR_L2TABLE_GRANULE_SHIFT);
		for (gb_index = 0; gb_index < 4; gb_index++)
			__raw_writel(reg, S2MPU_L1ENTRY_ATTR(base, 0, gb_index));
		for (gb_index = 34; gb_index < 64; gb_index++)
			__raw_writel(reg, S2MPU_L1ENTRY_ATTR(base, 0, gb_index));

		/* assign vid 1 to the sids specified in device tree */
		ssmt_set_vid(info);
	}

	/* clear interrupts before enabling them */
	__raw_writel(0xff, S2MPU_INTERRUPT_CLEAR(base));
#ifndef S2MPU_TEST
	reg = (1 << S2MPU_CTRL0_ENABLE_SHIFT) |
#else
	/* if testing, don't enable S2MPU upon init. instead use debugfs
	 * interface to enable and disable it.
	 */
	reg = (0 << S2MPU_CTRL0_ENABLE_SHIFT) |
#endif
			(1 << S2MPU_CTRL0_INTERRUPT_ENABLE_SHIFT) |
			/* don't send OKAY in case of a S2MPU violation */
			(1 << S2MPU_CTRL0_FAULT_RESP_TYPE_SHIFT);
	__raw_writel(reg, S2MPU_CTRL0(base));

	/* invalidate all memory protection cache */
	reg = (1 << S2MPU_ALL_INVALIDATION_INVALIDATE_SHIFT) |
			(0 << S2MPU_ALL_INVALIDATION_VID_SPECIFIC_SHIFT);
	__raw_writel(reg, S2MPU_ALL_INVALIDATION(base));

	/* add dsb to make sure the writes are complete by the time we return from this function */
	mb();

	return info;
}

static enum s2_gran l1_existing_granularity(u32 l1_entry)
{
	u32 granule;

	if (!S2MPU_L1ENTRY_ATTR_L2TABLE_EN_VALUE(l1_entry))
		return S2_GRAN_1GB;

	granule = S2MPU_L1ENTRY_ATTR_L2TABLE_GRANULE_VALUE(l1_entry);
	BUG_ON(granule > S2_GRAN_2MB);

	return granule;
}

static size_t get_byte_offset(u64 addr, enum s2_gran g)
{
	size_t granularity_shift = gran_shift[g];

	/* byte offset within 1gb */
	addr &= 0x3fffffff;
	/* get entry number */
	addr >>= granularity_shift;
	/* there are 4 of these per byte so divide by 4 to get the byte */
	addr >>= 2;

	return addr;
}

static size_t get_bit_offset(u64 addr, enum s2_gran g)
{
	size_t granularity_shift = gran_shift[g];

	addr &= 0x3fffffff;
	/* get page number */
	addr >>= granularity_shift;
	/* last two bits (i.e. modulo 4) should give us entry number within byte */
	addr &= 3;
	/* every entry is 2 bits long so multiply by 2 to get bit offset */
	addr <<= 1;

	return addr;
}

static void range_invalidate_mptc_vid_specific(void __iomem *base, u32 vmid,
					       phys_addr_t start, enum s2_gran g)
{
	size_t shift = gran_shift[g];
	u64 mask = (WHI_PHY_ADDR_MASK << shift) & WHI_PHY_ADDR_MASK;
	u32 val;

	start = start & mask;
	val = start >> 12;
	__raw_writel(val, S2MPU_RANGE_INVALIDATION_START_PPN(base));
	start = start + (1 << shift);
	val = start >> 12;
	__raw_writel(val, S2MPU_RANGE_INVALIDATION_END_PPN(base));

	val = (vmid << S2MPU_RANGE_INVALIDATION_VID_SHIFT) |
			(1 << S2MPU_RANGE_INVALIDATION_VID_SPECIFIC_SHIFT) |
			(1 << S2MPU_RANGE_INVALIDATION_INVALIDATE_SHIFT);
	__raw_writel(val, S2MPU_RANGE_INVALIDATION(base));
}

static void copy_decompose(size_t curr_gran_shift, u8 *curr_smpt,
			   size_t new_gran_shift, void *new_smpt)
{
	size_t new_per_curr = (1 << curr_gran_shift) >> new_gran_shift;
	size_t curr_len =  ((1 << 30) >> curr_gran_shift) >> 2;
	size_t i;

	for (i = 0; i < curr_len; i++) {
		u8 cb, entry;
		size_t sh, nidx;
		u8 *narr;
		size_t j;

		sh = 0;
		cb = curr_smpt[i];
		/* i << 2 -> curr entry no;
		 * << (new_shift - curr_shift) -> curr entry no * new entries
		 * per curr, i.e. new entry no;
		 * >> 2 -> divide new entry no by 4 to get byte containing new
		 * entry corresponding to curr entry. in othe words, first
		 * of set of bytes into which a single byte of curr table will
		 * expand.
		 */
		nidx = ((i << 2) << (curr_gran_shift - new_gran_shift)) >> 2;
		narr = new_smpt + nidx;
		j = 0;

		while (sh < BITS_PER_BYTE) {
			u8 b;
			/* limit = j + number of bytes that the current 2-bit
			 * entry will expand into.
			 */
			size_t limit = j + (new_per_curr >> 2);

			entry = (cb >> sh) & 3;
			b = extend_to_byte(entry);
			for (; j < limit; j++)
				narr[j] = b;
			sh += 2;
		}
	}
}

static void copy_decompose_gb(u8 two_bits, u8 *new_smpt, size_t len)
{
	u8 b = extend_to_byte(two_bits & 3);
	size_t i;

	for (i = 0; i < len; i++)
		new_smpt[i] = b;
}

static struct smpt_region *get_smpt_region(struct s2mpu_info *info, u32 gb_index)
{
	u64 pa = ((u64)info->pt.l2table_addrs[gb_index]) << 4;

	struct smpt_region *r = smpt_region_for_pa(info, pa);

	return r;
}

static void decompose(struct s2mpu_info *info, u32 gb_index, enum s2_gran curr_gran,
		      enum s2_gran new_gran)
{
	struct smpt_region *r = alloc_smpt(info, s2_table_size(gran_shift[new_gran]));
	u8 *new_smpt, *curr_smpt;
	u32 new_l1;

	if (!r) {
		dev_err(info->dev, "failed to allocate smpt\n");
		return;
	}

	new_smpt = r->va;

	if (curr_gran != S2_GRAN_1GB) {
		struct smpt_region *curr_r = get_smpt_region(info, gb_index);

		if (WARN_ON(!curr_r)) {
			dev_err(info->dev, "failed to find smpt region\n");
			return;
		}

		curr_smpt = curr_r->va;

		copy_decompose(gran_shift[curr_gran], curr_smpt,
			       gran_shift[new_gran], new_smpt);
	} else {
		u32 l1_entry = info->pt.l1entry_attrs[gb_index];
		u8 two_bits = (S2MPU_L1ENTRY_ATTR_WR_ACCESS_VALUE(l1_entry) << 1) |
				(S2MPU_L1ENTRY_ATTR_RD_ACCESS_VALUE(l1_entry) << 0);
		copy_decompose_gb(two_bits, new_smpt,
				  s2_table_size(gran_shift[new_gran]));
	}

	__raw_writel((((u64)r->pa) >> 4) & 0xffffffff,
		     S2MPU_L1ENTRY_L2TABLE_ADDR(info->base, info->vid, gb_index));
	new_l1 = (1 << S2MPU_L1ENTRY_ATTR_L2TABLE_EN_SHIFT) |
			(0 << S2MPU_L1ENTRY_ATTR_RD_ACCESS_SHIFT) |
			(0 << S2MPU_L1ENTRY_ATTR_WR_ACCESS_SHIFT) |
			(new_gran << S2MPU_L1ENTRY_ATTR_L2TABLE_GRANULE_SHIFT);

	__raw_writel(new_l1, S2MPU_L1ENTRY_ATTR(info->base, info->vid, gb_index));

	info->pt.l2table_addrs[gb_index] = (((u64)r->pa) >> 4) & 0xffffffff;
	info->pt.l1entry_attrs[gb_index] = new_l1;

	if (curr_gran != S2_GRAN_1GB)
		free_smpt(info, curr_smpt);
}

static void open_close_granules(struct s2mpu_info *info, u64 start,
				enum s2_gran win_gran, size_t gcount,
				bool open, enum dma_data_direction dir);

static void open_close_granule(struct s2mpu_info *info, phys_addr_t start, enum s2_gran g,
			       bool open, enum dma_data_direction dir)
{
	u32 gb_index = start >> gran_shift[S2_GRAN_1GB];
	enum s2_gran eg;
	u32 l1_entry;
	unsigned long flags;
	enum dir_mask dm;

	if (WARN_ON(dir == DMA_NONE))
		return;

	if (dir == DMA_BIDIRECTIONAL)
		dm = DIR_BIDRECTIONAL;
	else if (dir == DMA_TO_DEVICE)
		dm = DIR_READ;
	else if (dir == DMA_FROM_DEVICE)
		dm = DIR_WRITE;

	if (WARN_ON(gb_index >= 64))
		return;

	l1_entry = info->pt.l1entry_attrs[gb_index];
	eg = l1_existing_granularity(l1_entry);
	if (eg == g) {
		if (eg == S2_GRAN_1GB) {
			u32 one_gb_rw = dm << S2MPU_L1ENTRY_ATTR_RD_ACCESS_SHIFT;

			if (open)
				l1_entry = l1_entry | one_gb_rw;
			else
				l1_entry = l1_entry & ~(3U << S2MPU_L1ENTRY_ATTR_RD_ACCESS_SHIFT);
			__raw_writel(l1_entry, S2MPU_L1ENTRY_ATTR(info->base, info->vid, gb_index));
			/* invalidate S2MPU cache */
			range_invalidate_mptc_vid_specific(info->base, info->vid, start, eg);

			/* update page tables for resume from suspend case */
			info->pt.l1entry_attrs[gb_index] = l1_entry;
		} else {
			/* we need to modify SMPT */
			struct smpt_region *r = get_smpt_region(info, gb_index);
			size_t byte_offset = get_byte_offset(start, eg);
			size_t bit_offset;
			u8 *smpt;
			u8 b;

			if (WARN_ON(!r))
				return;

			smpt = r->va;

			bit_offset = get_bit_offset(start, eg);
			if (WARN_ON(bit_offset >= BITS_PER_BYTE || (bit_offset & 1) != 0))
				return;

			spin_lock_irqsave(&info->lock, flags);
			b = smpt[byte_offset];
			if (open)
				b = b | (dm << bit_offset);
			else
				b = b & ~(3 << bit_offset);
			smpt[byte_offset] = b;

			spin_unlock_irqrestore(&info->lock, flags);

			/* invalidate MPTC */
			range_invalidate_mptc_vid_specific(info->base, info->vid, start, eg);
		}
	} else {
		/* This is the more complicated case where existing granularity
		 * is different from requested granularity. There are two
		 * possibilities here:
		 * 1. existing_granularity < window_granularity
		 * 2. existing_granularity > window_granularity
		 *
		 * For the first case, we will break window_granularity to be
		 * same as existing_granularity. For the second case, we will have
		 * to break existing granularity in SMPT to be the same as
		 * window_granularity. The second case is more involved.
		 */
		if (eg < g) {
			/* divide window_granularity by existing_granularity */
			size_t gcount = 1 << (gran_shift[g] - gran_shift[eg]);

			open_close_granules(info, start, eg, gcount, open, dir);
		} else {
			/* break existing granularity to match win gran */
			decompose(info, gb_index, eg, g);
			open_close_granule(info, start, g, open, dir);
		}
	}
}

static void open_close_granules(struct s2mpu_info *info, phys_addr_t start,
				enum s2_gran win_gran, size_t gcount,
				bool open, enum dma_data_direction dir)
{
	size_t i;

	for (i = 0; i < gcount; i++) {
		open_close_granule(info, start, win_gran, open, dir);
		start += (1 << gran_shift[win_gran]);
	}
}

static enum s2_gran get_gran(u64 num)
{
	/* could use arm's rbit + clz instructions to count the number of
	 * trailing zeros but don't see justifiable benefit.
	 */
	if ((num & (0xffffffffffffffff << gran_shift[S2_GRAN_1GB])) == num)
		return S2_GRAN_1GB;
	if ((num & (0xffffffffffffffff << gran_shift[S2_GRAN_2MB])) == num)
		return S2_GRAN_2MB;
	if ((num & (0xffffffffffffffff << gran_shift[S2_GRAN_64KB])) == num)
		return S2_GRAN_64KB;
	if ((num & (0xffffffffffffffff << gran_shift[S2_GRAN_4KB])) == num)
		return S2_GRAN_4KB;

	/* we should not reach here. validation checks in s2_open and s2_close
	 * should have ensured that we only proceed with correctly aligned
	 * and sized windows.
	 */
	return S2_GRAN_INVALID;
}

static int validate(struct device *dev, phys_addr_t pa, size_t len)
{
	/* ensure that window is not in reserved range 0x1_0000_0000 - 0x8_7FFF_FFFF
	 * which is not mapped to any target
	 */
	if (pa < WHI_RESERVED_START) {
		if (pa + len > WHI_RESERVED_START) {
			dev_warn(dev, "window overlaps with reserved region\n");
			return -1;
		}
	} else { /* pa >= WHI_RESERVED_START */
		if (pa <= WHI_RESERVED_END) {
			dev_warn(dev, "window overlaps with reserved region\n");
			return -1;
		}
	}

	/* start address must be a multiple of one of granularities */
	if (is_gran_aligned(pa, S2_GRAN_4KB)) {
		dev_warn(dev, "window start not aligned to any granularity\n");
		return -1;
	}

	/* size of window must be a multiple of one of granularities */
	if (is_gran_aligned(len, S2_GRAN_4KB)) {
		dev_warn(dev, "window start not aligned to any granularity\n");
		return -1;
	}

	return 0;
}

int s2mpu_lib_open_close(struct s2mpu_info *info, phys_addr_t start, size_t len,
			 bool open, enum dma_data_direction dir)
{
	/* ag = alignment granularity, lg = length granularity */
	enum s2_gran ag, lg, winnerg;
	size_t gcount;
	int ret;

	ret = validate(info->dev, start, len);
	if (ret)
		goto out;

	ag = get_gran(start);
	lg = get_gran(len);
	if (WARN_ON(ag == S2_GRAN_INVALID || lg == S2_GRAN_INVALID)) {
		ret = -EINVAL;
		goto out;
	}
	/* three possibilities here:
	 * 1. ag == lg:  gcount = window->size >> gran_shift[lg]
	 * 2. ag < lg: gcount = window->size >> gran_shift[ag]
	 * 3. ag > lg: gcount = window->size >> gran_shift[lg]
	 *
	 * TODO: there are potential optimisations here when larger
	 * granularities are contained within smaller ag. let's consider them
	 * later.
	 */
	winnerg = ag < lg ? ag : lg;
	gcount = len >> gran_shift[winnerg];

	open_close_granules(info, start, winnerg, gcount, open, dir);
out:
	return ret;
}

int s2mpu_lib_restore(struct s2mpu_info *info)
{
	void __iomem *base = info->base;
	u32 gb_index, reg;
	unsigned long flags;

	spin_lock_irqsave(&info->lock, flags);

	for (gb_index = 0; gb_index < 4; gb_index++) {
		__raw_writel(info->pt.l2table_addrs[gb_index],
			     S2MPU_L1ENTRY_L2TABLE_ADDR(base, info->vid, gb_index));
		__raw_writel(info->pt.l1entry_attrs[gb_index],
			     S2MPU_L1ENTRY_ATTR(base, info->vid, gb_index));
	}

	for (gb_index = 34; gb_index < WHI_MAX_GB_GRANULES; gb_index++) {
		__raw_writel(info->pt.l2table_addrs[gb_index],
			     S2MPU_L1ENTRY_L2TABLE_ADDR(base, info->vid, gb_index));
		__raw_writel(info->pt.l1entry_attrs[gb_index],
			     S2MPU_L1ENTRY_ATTR(base, info->vid, gb_index));
	}

	if (info->vid == 1) {
		reg = (0 << S2MPU_L1ENTRY_ATTR_L2TABLE_EN_SHIFT) |
			(1 << S2MPU_L1ENTRY_ATTR_RD_ACCESS_SHIFT) |
			(1 << S2MPU_L1ENTRY_ATTR_WR_ACCESS_SHIFT) |
			(0 << S2MPU_L1ENTRY_ATTR_L2TABLE_GRANULE_SHIFT);
		for (gb_index = 0; gb_index < 4; gb_index++)
			__raw_writel(reg, S2MPU_L1ENTRY_ATTR(base, 0, gb_index));
		for (gb_index = 34; gb_index < 64; gb_index++)
			__raw_writel(reg, S2MPU_L1ENTRY_ATTR(base, 0, gb_index));

		ssmt_set_vid(info);
	}

#ifndef S2MPU_TEST
	reg = BIT(S2MPU_CTRL0_ENABLE_SHIFT) |
#else
	/* if testing, restore whatever enabled state was set through debugfs */
	reg = (info->enabled << S2MPU_CTRL0_ENABLE_SHIFT) |
#endif
			(1 << S2MPU_CTRL0_INTERRUPT_ENABLE_SHIFT) |
			/* don't send OKAY in case of a S2MPU violation */
			(1 << S2MPU_CTRL0_FAULT_RESP_TYPE_SHIFT);
	__raw_writel(reg, S2MPU_CTRL0(base));

	/* invalidate all memory protection cache */
	reg = (1 << S2MPU_ALL_INVALIDATION_INVALIDATE_SHIFT) |
			(0 << S2MPU_ALL_INVALIDATION_VID_SPECIFIC_SHIFT);
	__raw_writel(reg, S2MPU_ALL_INVALIDATION(base));

	spin_unlock_irqrestore(&info->lock, flags);

	return 0;
}

#ifdef S2MPU_TEST
u32 s2mpu_get_enabled(struct s2mpu_info *info)
{
	void __iomem *base = info->base;
	u32 reg;

	reg = __raw_readl(S2MPU_CTRL0(base));

	return reg & (1 << S2MPU_CTRL0_ENABLE_SHIFT);
}

void s2mpu_set_enabled(struct s2mpu_info *info, u32 val)
{
	void __iomem *base = info->base;
	u32 reg;

	reg = __raw_readl(S2MPU_CTRL0(base));
	if ((reg & 1) == val)
		return;

	if (val)
		reg = reg | (1 << S2MPU_CTRL0_ENABLE_SHIFT);
	else
		reg = reg & ~(1 << S2MPU_CTRL0_ENABLE_SHIFT);

	__raw_writel(reg, S2MPU_CTRL0(base));

	/* save this for restoring the value upon resume from suspend */
	info->enabled = val;
}
#endif
