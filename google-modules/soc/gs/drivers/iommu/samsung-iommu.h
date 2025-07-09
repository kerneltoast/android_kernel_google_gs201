/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 */

#ifndef __SAMSUNG_IOMMU_H
#define __SAMSUNG_IOMMU_H

#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/iommu.h>

#include <soc/google/debug-snapshot.h>

#define PT_BASE_SHIFT 12

#define MAX_VIDS			8U

struct tlb_config {
	unsigned int index;
	u32 cfg;
	u32 match_cfg;
	u32 match_id;
};

struct tlb_props {
	int id_cnt;
	u32 default_cfg;
	struct tlb_config *cfg;
};

struct sysmmu_drvdata {
	struct list_head list[MAX_VIDS];
	struct iommu_device iommu;
	struct device *dev;
	struct iommu_group *group;
	void __iomem *sfrbase;
	struct clk *clk;
	phys_addr_t pgtable[MAX_VIDS];
	spinlock_t lock; /* protect atomic update to H/W status */
	u32 version;
	unsigned int num_tlb;
	int qos;
	int attached_count[MAX_VIDS];
	int secure_irq;
	unsigned int secure_base;
	const unsigned int *reg_set;
	struct tlb_props tlb_props;
	bool no_block_mode;
	bool has_vcr;
	bool no_s2pf;		/* Disable stage 2 prefetch */
	bool rpm_resume;	/* true if .runtime_resume() is called */
	bool async_fault_mode;
	bool hide_page_fault;
	unsigned int panic_action;
};

struct sysmmu_clientdata {
	struct sysmmu_drvdata **sysmmus;
	struct device_link **dev_link;
	unsigned int sysmmu_count;
};

struct sysmmu_groupdata {
	struct list_head sysmmu_list[MAX_VIDS];
	spinlock_t sysmmu_list_lock[MAX_VIDS]; /* Protects .sysmmu_list */
	unsigned long vid_map;
	bool has_vcr;
};


enum {
	REG_IDX_DEFAULT = 0,
	REG_IDX_VM,

	MAX_SET_IDX,
};

enum {
	IDX_CTRL_VM = 0,
	IDX_CFG_VM,
	IDX_FLPT_BASE,
	IDX_ALL_INV,
	IDX_VPN_INV,
	IDX_RANGE_INV,
	IDX_RANGE_INV_START,
	IDX_RANGE_INV_END,
	IDX_FAULT_VA,
	IDX_FAULT_TRANS_INFO,
	IDX_TLB_READ,
	IDX_TLB_VPN,
	IDX_TLB_PPN,
	IDX_TLB_ATTR,
	IDX_SBB_READ,
	IDX_SBB_VPN,
	IDX_SBB_LINK,
	IDX_SBB_ATTR,
	IDX_SEC_FLPT_BASE,

	MAX_REG_IDX,
};

#define MMU_VM_REG_MULT(idx)		(((idx) == IDX_FAULT_VA || (idx) == IDX_FAULT_TRANS_INFO) \
					 ? 0x10 : 0x1000)

#define MMU_REG(data, idx)		((data)->sfrbase + (data)->reg_set[idx])
#define MMU_VM_REG(data, idx, vmid)	(MMU_REG(data, idx) + (vmid) * MMU_VM_REG_MULT(idx))
#define MMU_SEC_REG(data, offset_idx)	((data)->secure_base + (data)->reg_set[offset_idx])
#define MMU_SEC_VM_REG(data, offset_idx, vmid) (MMU_SEC_REG(data, offset_idx) + \
						(vmid) * MMU_VM_REG_MULT(offset_idx))

static inline unsigned int __max_vids(struct sysmmu_drvdata *data)
{
	if (data->has_vcr)
		return MAX_VIDS;
	return 1;
}

typedef u32 sysmmu_iova_t;
typedef u32 sysmmu_pte_t;

#define SECT_ORDER 20
#define LPAGE_ORDER 16
#define SPAGE_ORDER 12

#define SECT_SIZE  (1UL << SECT_ORDER)
#define LPAGE_SIZE (1UL << LPAGE_ORDER)
#define SPAGE_SIZE (1UL << SPAGE_ORDER)

#define SECT_MASK (~(SECT_SIZE - 1))
#define LPAGE_MASK (~(LPAGE_SIZE - 1))
#define SPAGE_MASK (~(SPAGE_SIZE - 1))

#define SECT_ENT_MASK	~((SECT_SIZE >> PG_ENT_SHIFT) - 1)
#define LPAGE_ENT_MASK	~((LPAGE_SIZE >> PG_ENT_SHIFT) - 1)
#define SPAGE_ENT_MASK	~((SPAGE_SIZE >> PG_ENT_SHIFT) - 1)

#define SPAGES_PER_LPAGE	(LPAGE_SIZE / SPAGE_SIZE)

#define NUM_LV1ENTRIES	65536
#define NUM_LV2ENTRIES (SECT_SIZE / SPAGE_SIZE)
#define LV1TABLE_SIZE (NUM_LV1ENTRIES * sizeof(sysmmu_pte_t))
#define LV2TABLE_SIZE (NUM_LV2ENTRIES * sizeof(sysmmu_pte_t))

#define lv1ent_offset(iova) ((iova) >> SECT_ORDER)
#define lv2ent_offset(iova) (((iova) & ~SECT_MASK) >> SPAGE_ORDER)

#define FLPD_FLAG_MASK	7
#define SLPD_FLAG_MASK	3

#define SECT_FLAG	2
#define SLPD_FLAG	1

#define LPAGE_FLAG	1
#define SPAGE_FLAG	2

#define PG_ENT_SHIFT	4
#define lv1ent_unmapped(sent)	((*(sent) & 7) == 0)
#define lv1ent_page(sent)	((*(sent) & 7) == 1)

#define lv1ent_section(sent)	((*(sent) & FLPD_FLAG_MASK) == SECT_FLAG)
#define lv2table_base(sent)	((phys_addr_t)(*(sent) & ~0x3FU) << PG_ENT_SHIFT)
#define lv2ent_unmapped(pent)	((*(pent) & SLPD_FLAG_MASK) == 0)
#define lv2ent_small(pent)	((*(pent) & SLPD_FLAG_MASK) == SPAGE_FLAG)
#define lv2ent_large(pent)	((*(pent) & SLPD_FLAG_MASK) == LPAGE_FLAG)

#define PGBASE_TO_PHYS(pgent)	((phys_addr_t)(pgent) << PG_ENT_SHIFT)
#define ENT_TO_PHYS(ent) (phys_addr_t)(*(ent))
#define section_phys(sent) PGBASE_TO_PHYS(ENT_TO_PHYS(sent) & SECT_ENT_MASK)
#define section_offs(iova) ((iova) & (SECT_SIZE - 1))
#define lpage_phys(pent) PGBASE_TO_PHYS(ENT_TO_PHYS(pent) & LPAGE_ENT_MASK)
#define lpage_offs(iova) ((iova) & (LPAGE_SIZE - 1))
#define spage_phys(pent) PGBASE_TO_PHYS(ENT_TO_PHYS(pent) & SPAGE_ENT_MASK)
#define spage_offs(iova) ((iova) & (SPAGE_SIZE - 1))

static inline sysmmu_pte_t *page_entry(sysmmu_pte_t *sent, sysmmu_iova_t iova)
{
	return (sysmmu_pte_t *)(phys_to_virt(lv2table_base(sent))) +
				lv2ent_offset(iova);
}

static inline sysmmu_pte_t *section_entry(sysmmu_pte_t *pgtable,
					  sysmmu_iova_t iova)
{
	return pgtable + lv1ent_offset(iova);
}

#define REG_MMU_CTRL			0x000
#define REG_MMU_CFG			0x004
#define REG_MMU_STATUS			0x008
#define REG_MMU_FLPT_BASE		0x00C
#define REG_MMU_VERSION			0x034
#define REG_MMU_CAPA0_V7		0x870
#define REG_MMU_CAPA1_V7		0x874

#define MMU_CAPA_NUM_TLB_WAY(reg)	((reg) & 0xFF)
#define MMU_CAPA_NUM_SBB_ENTRY(reg)	(((reg) >> 12) & 0xF)
#define MMU_CAPA1_EXIST(reg)		(((reg) >> 11) & 0x1)
#define MMU_CAPA1_TYPE(reg)		(((reg) >> 28) & 0xF)
#define MMU_CAPA1_NO_BLOCK_MODE(reg)	(((reg) >> 15) & 0x1)
#define MMU_CAPA1_VCR_ENABLED(reg)	(((reg) >> 14) & 0x1)
#define MMU_CAPA1_NUM_TLB(reg)		(((reg) >> 4) & 0xFF)
#define MMU_CAPA1_NUM_PORT(reg)		((reg) & 0xF)

#define MMU_MAJ_VER(val)	((val) >> 11)
#define MMU_MIN_VER(val)	(((val) >> 4) & 0x7F)
#define MMU_REV_VER(val)	((val) & 0xF)
#define MMU_RAW_VER(reg)	(((reg) >> 17) & 0x7FFF)

#define CTRL_VID_ENABLE			BIT(0)
#define CTRL_MMU_ENABLE			BIT(0)
#define CTRL_MMU_BLOCK			BIT(1)
#define CTRL_INT_ENABLE			BIT(2)
#define CTRL_FAULT_STALL_MODE		BIT(3)

#define CFG_MASK_GLOBAL			0x00000F80 /* Bit 11, 10-7 */
#define CFG_MASK_VM			0xB00F1004 /* Bit 31, 29, 28, 19-16, 12, 2 */
#define CFG_QOS_OVRRIDE			BIT(11)
#define CFG_QOS(n)			(((n) & 0xFU) << 7)

irqreturn_t samsung_sysmmu_irq_thread(int irq, void *dev_id);
irqreturn_t samsung_sysmmu_irq(int irq, void *dev_id);

#endif /* __SAMSUNG_IOMMU_H */

