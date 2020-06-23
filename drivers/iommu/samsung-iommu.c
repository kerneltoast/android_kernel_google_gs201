// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 */

#define pr_fmt(fmt) "sysmmu: " fmt

#include <linux/clk.h>
#include <linux/dma-iommu.h>
#include <linux/interrupt.h>
#include <linux/iommu.h>
#include <linux/kmemleak.h>
#include <linux/module.h>
#include <linux/of_iommu.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>

#include <dt-bindings/soc/samsung,sysmmu-v8.h>

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

#define NUM_LV1ENTRIES	4096
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
#define lv2table_base(sent)	((phys_addr_t)(*(sent) & ~0x3F) << PG_ENT_SHIFT)
#define lv2ent_unmapped(pent)	((*(pent) & SLPD_FLAG_MASK) == 0)
#define lv2ent_small(pent)	((*(pent) & SLPD_FLAG_MASK) == SPAGE_FLAG)
#define lv2ent_large(pent)	((*(pent) & SLPD_FLAG_MASK) == LPAGE_FLAG)

#define FLPD_SHAREABLE_FLAG	BIT(6)
#define SLPD_SHAREABLE_FLAG	BIT(4)

#define PGBASE_TO_PHYS(pgent)	((phys_addr_t)(pgent) << PG_ENT_SHIFT)
#define ENT_TO_PHYS(ent) (phys_addr_t)(*(ent))
#define section_phys(sent) PGBASE_TO_PHYS(ENT_TO_PHYS(sent) & SECT_ENT_MASK)
#define section_offs(iova) ((iova) & (SECT_SIZE - 1))
#define lpage_phys(pent) PGBASE_TO_PHYS(ENT_TO_PHYS(pent) & LPAGE_ENT_MASK)
#define lpage_offs(iova) ((iova) & (LPAGE_SIZE - 1))
#define spage_phys(pent) PGBASE_TO_PHYS(ENT_TO_PHYS(pent) & SPAGE_ENT_MASK)
#define spage_offs(iova) ((iova) & (SPAGE_SIZE - 1))

#define MMU_MAJ_VER(val)	((val) >> 11)
#define MMU_MIN_VER(val)	(((val) >> 4) & 0x7F)
#define MMU_REV_VER(val)	((val) & 0xF)
#define MMU_RAW_VER(reg)	(((reg) >> 17) & 0x7FFF)

#define REG_MMU_CTRL			0x000
#define CTRL_DISABLE			0x0
#define CTRL_VID_ENABLE			0x1
#define CTRL_BLOCK_DISABLE		0x3
#define CTRL_ENABLE			0x5
#define CTRL_BLOCK			0x7
#define CTRL_FAULT_STALL_MODE		0x8
#define REG_MMU_CFG			0x004
#define CFG_MASK_GLOBAL			0x00000F80	/* Bit 11, 10-7 */
					/* Bit 31, 29, 28, 19-16, 12, 2 */
#define CFG_MASK_VM			0xB00F1004
#define CFG_QOS_OVRRIDE			BIT(11)
#define CFG_QOS(n)			(((n) & 0xF) << 7)
#define REG_MMU_STATUS			0x008
#define REG_MMU_FLPT_BASE		0x00C

#define REG_MMU_INV_ALL			0x010
#define REG_MMU_INV_RANGE		0x018
#define REG_MMU_INV_START		0x020
#define REG_MMU_INV_END			0x024

#define REG_MMU_VERSION			0x034

#define REG_MMU_INT_STATUS		0x060
#define REG_MMU_FAULT_VA		0x070
#define REG_MMU_FAULT_TRANS_INFO	0x078
#define REG_MMU_FAULT_RW_MASK		(0x1 << 20)
#define IS_READ_FAULT(x)		(((x) & REG_MMU_FAULT_RW_MASK) == 0)

#define SYSMMU_FAULT_PTW_ACCESS   0
#define SYSMMU_FAULT_PAGE_FAULT   1
#define SYSMMU_FAULT_ACCESS       3
#define SYSMMU_FAULT_SECURITY     4
#define SYSMMU_FAULT_UNKNOWN      5

#define SYSMMU_FAULTS_NUM         (SYSMMU_FAULT_UNKNOWN + 1)

#define REG_MMU_CAPA0_V7		0x870
#define REG_MMU_CAPA1_V7		0x874

#define REG_PUBLIC_WAY_CFG		0x120
#define REG_PRIVATE_WAY_CFG(n)		(0x200 + ((n) * 0x10))
#define REG_PRIVATE_ADDR_START(n)	(0x204 + ((n) * 0x10))
#define REG_PRIVATE_ADDR_END(n)		(0x208 + ((n) * 0x10))
#define REG_PRIVATE_ID(n)		(0x20C + ((n) * 0x10))

#define MMU_WAY_CFG_MASK_PREFETCH	GENMASK(1, 1)
#define MMU_WAY_CFG_MASK_PREFETCH_DIR	GENMASK(3, 2)
#define MMU_WAY_CFG_MASK_MATCH_METHOD	GENMASK(4, 4)
#define MMU_WAY_CFG_MASK_FETCH_SIZE	GENMASK(7, 5)
#define MMU_WAY_CFG_MASK_TARGET_CH	GENMASK(9, 8)

#define MMU_WAY_CFG_ID_MATCHING		(1 << 4)
#define MMU_WAY_CFG_ADDR_MATCHING	(0 << 4)
#define MMU_WAY_CFG_PRIVATE_ENABLE	(1 << 0)

#define MMU_PUBLIC_WAY_MASK	(MMU_WAY_CFG_MASK_PREFETCH |	\
		MMU_WAY_CFG_MASK_PREFETCH_DIR | MMU_WAY_CFG_MASK_FETCH_SIZE)
#define MMU_PRIVATE_WAY_MASK	(MMU_PUBLIC_WAY_MASK |		\
		MMU_WAY_CFG_MASK_MATCH_METHOD | MMU_WAY_CFG_MASK_TARGET_CH)

#define REG_MMU_CTRL_VM			0x8000
#define REG_MMU_CFG_VM			0x8004

#define MMU_TLB_CFG_MASK(reg)		((reg) & (GENMASK(7, 5) | GENMASK(3, 2) | GENMASK(1, 1)))
#define MMU_TLB_MATCH_CFG_MASK(reg)	((reg) & (GENMASK(31, 16) | GENMASK(9, 8)))

#define MMU_CAPA1_EXIST(reg)		(((reg) >> 11) & 0x1)
#define MMU_CAPA1_TYPE(reg)		(((reg) >> 28) & 0xF)
#define MMU_CAPA1_NO_BLOCK_MODE(reg)	(((reg) >> 15) & 0x1)
#define MMU_CAPA1_VCR_ENABLED(reg)	(((reg) >> 14) & 0x1)

#define MMU_CAPA_NUM_TLB_WAY(reg)	((reg) & 0xFF)
#define MMU_CAPA_NUM_SBB_ENTRY(reg)	(((reg) >> 12) & 0xF)
#define MMU_CAPA1_NUM_TLB(reg)		(((reg) >> 4) & 0xFF)
#define MMU_CAPA1_NUM_PORT(reg)		((reg) & 0xF)
#define REG_MMU_TLB_CFG(n)		(0x2000 + ((n) * 0x20) + 0x4)
#define REG_MMU_TLB_MATCH_CFG(n)	(0x2000 + ((n) * 0x20) + 0x8)
#define REG_MMU_TLB_MATCH_SVA(n)	(0x2000 + ((n) * 0x20) + 0xC)
#define REG_MMU_TLB_MATCH_EVA(n)	(0x2000 + ((n) * 0x20) + 0x10)
#define REG_MMU_TLB_MATCH_ID(n)		(0x2000 + ((n) * 0x20) + 0x14)

#define MMU_TLB_INFO(n)			(0x2000 + ((n) * 0x20))
#define MMU_CAPA1_NUM_TLB_SET(reg)	(((reg) >> 16) & 0xFF)
#define MMU_CAPA1_NUM_TLB_WAY(reg)	((reg) & 0xFF)
#define MMU_CAPA1_SET_TLB_READ_ENTRY(tid, set, way, line)		\
					((set) | ((way) << 8) |		\
					 ((line) << 16) | ((tid) << 20))

#define MMU_TLB_ENTRY_VALID(reg)	((reg) >> 28)
#define MMU_SBB_ENTRY_VALID(reg)	((reg) >> 28)

#define MMU_VADDR_FROM_TLB(reg, idx)	(((reg) & 0xFFFFC | (idx) & 0x3) << 12)
#define MMU_PADDR_FROM_TLB(reg)		(((reg) & 0xFFFFFF) << 12)
#define MMU_VADDR_FROM_SBB(reg)		(((reg) & 0xFFFFF) << 12)
#define MMU_PADDR_FROM_SBB(reg)		(((reg) & 0x3FFFFFF) << 10)

#define REG_SLOT_RSV(n)			(0x4000 + ((n) * 0x20))

#define MMU_REG(data, idx)		((data)->sfrbase + (data)->reg_set[idx])
#define MMU_VM_REG(data, idx, vmid)	((data)->sfrbase + \
					 (data)->reg_set[idx] + (vmid) * 0x10)

#define DEFAULT_QOS_VALUE	-1

enum {
	REG_IDX_DEFAULT = 0,
	REG_IDX_VM,

	MAX_SET_IDX,
};

enum {
	IDX_FLPT_BASE = 0,
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

static const unsigned int sysmmu_reg_set[MAX_SET_IDX][MAX_REG_IDX] = {
	/* Default without VM */
	{
		/* FLPT base, TLB invalidation, Fault information */
		0x000C,	0x0010,	0x0014,	0x0018,
		0x0020,	0x0024,	0x0070,	0x0078,
		/* TLB information */
		0x8000,	0x8004,	0x8008,	0x800C,
		/* SBB information */
		0x8020,	0x8024,	0x8028,	0x802C,
		/* secure FLPT base (same as non-secure) */
		0x000C,
	},
	/* VM */
	{
		/* FLPT base, TLB invalidation, Fault information */
		0x800C,	0x8010,	0x8014,	0x8018,
		0x8020,	0x8024,	0x1000,	0x1004,
		/* TLB information */
		0x3000,	0x3004,	0x3008,	0x300C,
		/* SBB information */
		0x3020,	0x3024,	0x3028,	0x302C,
		/* secure FLPT base */
		0x000C,
	},
};

static char *sysmmu_fault_name[SYSMMU_FAULTS_NUM] = {
	"PTW ACCESS FAULT",
	"PAGE FAULT",
	"RESERVED",
	"ACCESS FAULT",
	"SECURITY FAULT",
	"UNKNOWN FAULT"
};

static int sysmmu_fault_type[SYSMMU_FAULTS_NUM] = {
	IOMMU_FAULT_REASON_WALK_EABT,
	IOMMU_FAULT_REASON_PTE_FETCH,
	IOMMU_FAULT_REASON_UNKNOWN,
	IOMMU_FAULT_REASON_ACCESS,
	IOMMU_FAULT_REASON_PERMISSION,
	IOMMU_FAULT_REASON_UNKNOWN,
};

struct tlb_priv_addr {
	unsigned int cfg;
};

struct tlb_priv_id {
	unsigned int cfg;
	unsigned int id;
};

struct tlb_port_cfg {
	unsigned int cfg;
	unsigned int id;
};

/*
 * flags[7:4] specifies TLB matching types.
 * 0x1 : TLB way dedication
 * 0x2 : TLB port dedication
 */
#define TLB_TYPE_MASK(x)	((x) & (0xF << 4))
#define TLB_TYPE_WAY		(0x1 << 4)
#define TLB_TYPE_PORT		(0x2 << 4)
#define IS_TLB_WAY_TYPE(data)	(TLB_TYPE_MASK((data)->tlb_props.flags)	\
				== TLB_TYPE_WAY)
#define IS_TLB_PORT_TYPE(data)	(TLB_TYPE_MASK((data)->tlb_props.flags)	\
				== TLB_TYPE_PORT)

#define TLB_WAY_PRIVATE_ID	BIT(0)
#define TLB_WAY_PRIVATE_ADDR	BIT(1)
#define TLB_WAY_PUBLIC		BIT(2)

struct tlb_way_props {
	int priv_id_cnt;
	int priv_addr_cnt;
	unsigned int public_cfg;
	struct tlb_priv_id *priv_id_cfg;
	struct tlb_priv_addr *priv_addr_cfg;
};

struct tlb_port_props {
	int port_id_cnt;
	int slot_cnt;
	struct tlb_port_cfg *port_cfg;
	unsigned int *slot_cfg;
};

struct tlb_props {
	int flags;
	union {
		struct tlb_way_props way_props;
		struct tlb_port_props port_props;
	};
};

struct sysmmu_drvdata {
	struct list_head list;
	struct iommu_device iommu;
	struct device *dev;
	struct iommu_group *group;
	void __iomem *sfrbase;
	struct clk *clk;
	phys_addr_t pgtable;
	spinlock_t lock; /* protect atomic update to H/W status */
	u32 version;
	int qos;
	int attached_count;
	const unsigned int *reg_set;
	struct tlb_props tlb_props;
	bool no_block_mode;
	bool has_vcr;
};

struct sysmmu_clientdata {
	struct device *dev;
	struct sysmmu_drvdata **sysmmus;
	struct device_link **dev_link;
	int sysmmu_count;
};

static struct iommu_ops samsung_sysmmu_ops;
static struct platform_driver samsung_sysmmu_driver;

struct samsung_sysmmu_domain {
	struct iommu_domain domain;
	struct iommu_group *group;
	sysmmu_pte_t *page_table;
	atomic_t *lv2entcnt;
	spinlock_t pgtablelock; /* serialize races to page table updates */
};

struct samsung_sysmmu_fault_info {
	struct sysmmu_drvdata *drvdata;
	struct iommu_fault_event event;
};

static bool sysmmu_global_init_done;
static struct device sync_dev;
static struct kmem_cache *flpt_cache, *slpt_cache;

static inline u32 __sysmmu_get_hw_version(struct sysmmu_drvdata *data)
{
	return MMU_RAW_VER(readl_relaxed(data->sfrbase + REG_MMU_VERSION));
}

static inline bool __sysmmu_has_capa1(struct sysmmu_drvdata *data)
{
	return MMU_CAPA1_EXIST(readl_relaxed(data->sfrbase + REG_MMU_CAPA0_V7));
}

static inline u32 __sysmmu_get_capa_type(struct sysmmu_drvdata *data)
{
	return MMU_CAPA1_TYPE(readl_relaxed(data->sfrbase + REG_MMU_CAPA1_V7));
}

static inline bool __sysmmu_get_capa_no_block_mode(struct sysmmu_drvdata *data)
{
	return MMU_CAPA1_NO_BLOCK_MODE(readl_relaxed(data->sfrbase +
						     REG_MMU_CAPA1_V7));
}

static inline bool __sysmmu_get_capa_vcr_enabled(struct sysmmu_drvdata *data)
{
	return MMU_CAPA1_VCR_ENABLED(readl_relaxed(data->sfrbase +
						   REG_MMU_CAPA1_V7));
}

static inline void __sysmmu_tlb_invalidate_all(struct sysmmu_drvdata *data)
{
	writel(0x1, MMU_REG(data, IDX_ALL_INV));
}

static inline void __sysmmu_tlb_invalidate(struct sysmmu_drvdata *data,
					   dma_addr_t start, dma_addr_t end)
{
	writel_relaxed(start, MMU_REG(data, IDX_RANGE_INV_START));
	writel_relaxed(end - 1, MMU_REG(data, IDX_RANGE_INV_END));
	writel(0x1, MMU_REG(data, IDX_RANGE_INV));
}

static inline void __sysmmu_disable(struct sysmmu_drvdata *data)
{
	if (data->no_block_mode) {
		__sysmmu_tlb_invalidate_all(data);
	} else {
		if (data->has_vcr) {
			writel_relaxed(0, data->sfrbase + REG_MMU_CFG_VM);
			writel_relaxed(CTRL_BLOCK_DISABLE,
				       data->sfrbase + REG_MMU_CTRL_VM);
		}

		writel_relaxed(0, data->sfrbase + REG_MMU_CFG);
		writel_relaxed(CTRL_BLOCK_DISABLE,
			       data->sfrbase + REG_MMU_CTRL);
	}
}

static inline void __sysmmu_set_public_way(struct sysmmu_drvdata *data,
					   unsigned int public_cfg)
{
	u32 cfg = readl_relaxed(data->sfrbase + REG_PUBLIC_WAY_CFG);

	cfg &= ~MMU_PUBLIC_WAY_MASK;
	cfg |= public_cfg;

	writel_relaxed(cfg, data->sfrbase + REG_PUBLIC_WAY_CFG);

	dev_dbg(data->dev, "public_cfg : %#x\n", cfg);
}

static inline void __sysmmu_set_private_way_id(struct sysmmu_drvdata *data,
					       unsigned int way_idx)
{
	struct tlb_priv_id *priv_cfg = data->tlb_props.way_props.priv_id_cfg;
	u32 cfg = readl_relaxed(data->sfrbase + REG_PRIVATE_WAY_CFG(way_idx));

	cfg &= ~MMU_PRIVATE_WAY_MASK;
	cfg |= MMU_WAY_CFG_ID_MATCHING | MMU_WAY_CFG_PRIVATE_ENABLE;
	cfg |= priv_cfg[way_idx].cfg;

	writel_relaxed(cfg, data->sfrbase + REG_PRIVATE_WAY_CFG(way_idx));
	writel_relaxed(priv_cfg[way_idx].id,
		       data->sfrbase + REG_PRIVATE_ID(way_idx));

	dev_dbg(data->dev, "priv ID way[%d] cfg : %#x, id : %#x\n",
		way_idx, cfg, priv_cfg[way_idx].id);
}

static inline void __sysmmu_set_private_way_addr(struct sysmmu_drvdata *data,
						 unsigned int idx)
{
	struct tlb_priv_addr *privcfg = data->tlb_props.way_props.priv_addr_cfg;
	unsigned int way_idx = data->tlb_props.way_props.priv_id_cnt + idx;
	u32 cfg = readl_relaxed(data->sfrbase + REG_PRIVATE_WAY_CFG(way_idx));

	cfg &= ~MMU_PRIVATE_WAY_MASK;
	cfg |= MMU_WAY_CFG_ADDR_MATCHING | MMU_WAY_CFG_PRIVATE_ENABLE;
	cfg |= privcfg[idx].cfg;

	writel_relaxed(cfg, data->sfrbase + REG_PRIVATE_WAY_CFG(way_idx));

	dev_dbg(data->dev, "priv ADDR way[%d] cfg : %#x\n", way_idx, cfg);
}

static inline void __sysmmu_set_tlb_way_type(struct sysmmu_drvdata *data)
{
	u32 cfg = readl_relaxed(data->sfrbase + REG_MMU_CAPA0_V7);
	u32 waycnt = MMU_CAPA_NUM_TLB_WAY(cfg);
	struct tlb_props *tlb_props = &data->tlb_props;
	int priv_id_cnt = tlb_props->way_props.priv_id_cnt;
	int priv_addr_cnt = tlb_props->way_props.priv_addr_cnt;
	u32 setcnt = 0;
	unsigned int i;

	if (tlb_props->flags & TLB_WAY_PUBLIC)
		__sysmmu_set_public_way(data, tlb_props->way_props.public_cfg);

	if (tlb_props->flags & TLB_WAY_PRIVATE_ID)
		for (i = 0; i < priv_id_cnt && setcnt < waycnt; i++, setcnt++)
			__sysmmu_set_private_way_id(data, i);

	if (tlb_props->flags & TLB_WAY_PRIVATE_ADDR)
		for (i = 0; i < priv_addr_cnt && setcnt < waycnt; i++, setcnt++)
			__sysmmu_set_private_way_addr(data, i);

	if (priv_id_cnt + priv_addr_cnt > waycnt) {
		dev_warn(data->dev,
			 "Ignoring larger TLB way count than %d!\n", waycnt);
		dev_warn(data->dev, "Number of private way id/addr = %d/%d\n",
			 priv_id_cnt, priv_addr_cnt);
	}
}

static inline void __sysmmu_set_tlb_port(struct sysmmu_drvdata *data,
					 unsigned int port_idx)
{
	struct tlb_port_cfg *port_cfg = data->tlb_props.port_props.port_cfg;

	writel_relaxed(MMU_TLB_CFG_MASK(port_cfg[port_idx].cfg),
		       data->sfrbase + REG_MMU_TLB_CFG(port_idx));

	/* port_idx 0 is default port. */
	if (port_idx == 0) {
		dev_dbg(data->dev, "port[%d] cfg : %#x for common\n", port_idx,
			MMU_TLB_CFG_MASK(port_cfg[port_idx].cfg));
		return;
	}

	writel_relaxed(MMU_TLB_MATCH_CFG_MASK(port_cfg[port_idx].cfg),
		       data->sfrbase + REG_MMU_TLB_MATCH_CFG(port_idx));
	writel_relaxed(port_cfg[port_idx].id,
		       data->sfrbase + REG_MMU_TLB_MATCH_ID(port_idx));

	dev_dbg(data->dev, "port[%d] cfg : %#x, match : %#x, id : %#x\n",
		port_idx,
		MMU_TLB_CFG_MASK(port_cfg[port_idx].cfg),
		MMU_TLB_MATCH_CFG_MASK(port_cfg[port_idx].cfg),
		port_cfg[port_idx].id);
}

static inline void __sysmmu_set_tlb_port_type(struct sysmmu_drvdata *data)
{
	u32 cfg = readl_relaxed(data->sfrbase + REG_MMU_CAPA1_V7);
	u32 tlb_num = MMU_CAPA1_NUM_TLB(cfg);
	struct tlb_props *tlb_props = &data->tlb_props;
	int port_id_cnt = tlb_props->port_props.port_id_cnt;
	int slot_cnt = tlb_props->port_props.slot_cnt;
	unsigned int i;

	if (port_id_cnt > tlb_num) {
		dev_warn(data->dev, "Ignoring %d larger than TLB count %d\n",
			 port_id_cnt, tlb_num);
		port_id_cnt = tlb_num;
	}

	for (i = 0; i < port_id_cnt; i++)
		__sysmmu_set_tlb_port(data, i);

	for (i = 0; i < slot_cnt; i++)
		writel_relaxed(tlb_props->port_props.slot_cfg[i],
			       data->sfrbase + REG_SLOT_RSV(i));
}

static inline void __sysmmu_init_config(struct sysmmu_drvdata *data)
{
	unsigned long cfg = 0, cfg_vm = 0;

	if (data->qos != DEFAULT_QOS_VALUE)
		cfg |= CFG_QOS_OVRRIDE | CFG_QOS(data->qos);

	if (!data->no_block_mode)
		writel_relaxed(CTRL_BLOCK, data->sfrbase + REG_MMU_CTRL);

	if (IS_TLB_WAY_TYPE(data))
		__sysmmu_set_tlb_way_type(data);
	else if (IS_TLB_PORT_TYPE(data))
		__sysmmu_set_tlb_port_type(data);

	if (data->has_vcr) {
		cfg_vm = cfg & ~CFG_MASK_VM;
		cfg &= ~CFG_MASK_GLOBAL;
		writel_relaxed(cfg, data->sfrbase + REG_MMU_CFG);
		writel_relaxed(cfg_vm, data->sfrbase + REG_MMU_CFG_VM);
	} else {
		writel_relaxed(cfg, data->sfrbase + REG_MMU_CFG);
	}
}

static inline void __sysmmu_enable(struct sysmmu_drvdata *data)
{
	__sysmmu_init_config(data);

	writel_relaxed(data->pgtable / SPAGE_SIZE,
		       MMU_REG(data, IDX_FLPT_BASE));
	__sysmmu_tlb_invalidate_all(data);

	writel(CTRL_ENABLE, data->sfrbase + REG_MMU_CTRL);

	if (data->has_vcr)
		writel(CTRL_VID_ENABLE | CTRL_FAULT_STALL_MODE,
		       data->sfrbase + REG_MMU_CTRL_VM);
}

static inline u32 __sysmmu_get_intr_status(struct sysmmu_drvdata *data)
{
	return readl_relaxed(data->sfrbase + REG_MMU_INT_STATUS);
}

static inline u32 __sysmmu_get_fault_address(struct sysmmu_drvdata *data)
{
	return readl_relaxed(MMU_VM_REG(data, IDX_FAULT_VA, 0));
}

static struct samsung_sysmmu_domain *to_sysmmu_domain(struct iommu_domain *dom)
{
	return container_of(dom, struct samsung_sysmmu_domain, domain);
}

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

static inline void pgtable_flush(void *vastart, void *vaend)
{
	dma_sync_single_for_device(&sync_dev, virt_to_phys(vastart),
				   vaend - vastart, DMA_TO_DEVICE);
}

static bool samsung_sysmmu_capable(enum iommu_cap cap)
{
	return cap == IOMMU_CAP_CACHE_COHERENCY;
}

static struct iommu_domain *samsung_sysmmu_domain_alloc(unsigned int type)
{
	struct samsung_sysmmu_domain *domain;

	if (type != IOMMU_DOMAIN_UNMANAGED &&
	    type != IOMMU_DOMAIN_DMA &&
	    type != IOMMU_DOMAIN_IDENTITY) {
		pr_err("invalid domain type %u\n", type);
		return NULL;
	}

	domain = kzalloc(sizeof(*domain), GFP_KERNEL);
	if (!domain)
		return NULL;

	domain->page_table =
		(sysmmu_pte_t *)kmem_cache_alloc(flpt_cache,
						 GFP_KERNEL | __GFP_ZERO);
	if (!domain->page_table)
		goto err_pgtable;

	domain->lv2entcnt = kcalloc(NUM_LV1ENTRIES, sizeof(*domain->lv2entcnt),
				    GFP_KERNEL);
	if (!domain->lv2entcnt)
		goto err_counter;

	if (type == IOMMU_DOMAIN_DMA) {
		int ret = iommu_get_dma_cookie(&domain->domain);

		if (ret) {
			pr_err("failed to get dma cookie (%d)\n", ret);
			goto err_get_dma_cookie;
		}
	}

	pgtable_flush(domain->page_table, domain->page_table + NUM_LV1ENTRIES);

	spin_lock_init(&domain->pgtablelock);

	return &domain->domain;

err_get_dma_cookie:
	kfree(domain->lv2entcnt);
err_counter:
	kmem_cache_free(flpt_cache, domain->page_table);
err_pgtable:
	kfree(domain);
	return NULL;
}

static void samsung_sysmmu_domain_free(struct iommu_domain *dom)
{
	struct samsung_sysmmu_domain *domain = to_sysmmu_domain(dom);

	iommu_put_dma_cookie(dom);
	kmem_cache_free(flpt_cache, domain->page_table);
	kfree(domain->lv2entcnt);
	kfree(domain);
}

static inline void samsung_sysmmu_detach_drvdata(struct sysmmu_drvdata *data)
{
	unsigned long flags;

	spin_lock_irqsave(&data->lock, flags);
	if (--data->attached_count == 0) {
		if (pm_runtime_active(data->dev))
			__sysmmu_disable(data);

		list_del(&data->list);
		data->pgtable = 0;
		data->group = NULL;
	}
	spin_unlock_irqrestore(&data->lock, flags);
}

static int samsung_sysmmu_attach_dev(struct iommu_domain *dom,
				     struct device *dev)
{
	struct iommu_fwspec *fwspec = dev_iommu_fwspec_get(dev);
	struct sysmmu_clientdata *client;
	struct samsung_sysmmu_domain *domain;
	struct list_head *group_list;
	struct sysmmu_drvdata *drvdata;
	struct iommu_group *group = dev->iommu_group;
	unsigned long flags;
	phys_addr_t page_table;
	int i;

	if (!fwspec || fwspec->ops != &samsung_sysmmu_ops) {
		dev_err(dev, "failed to attach, IOMMU instance data %s.\n",
			!fwspec ? "is not initialized" : "has different ops");
		return -ENXIO;
	}

	if (!fwspec->iommu_priv) {
		dev_err(dev, "has no IOMMU\n");
		return -ENODEV;
	}

	domain = to_sysmmu_domain(dom);
	domain->group = group;
	group_list = iommu_group_get_iommudata(group);
	page_table = virt_to_phys(domain->page_table);

	client = fwspec->iommu_priv;
	for (i = 0; i < client->sysmmu_count; i++) {
		drvdata = client->sysmmus[i];

		spin_lock_irqsave(&drvdata->lock, flags);
		if (drvdata->attached_count++ == 0) {
			list_add(&drvdata->list, group_list);
			drvdata->group = group;
			drvdata->pgtable = page_table;

			if (pm_runtime_active(drvdata->dev))
				__sysmmu_enable(drvdata);
		} else if (drvdata->pgtable != page_table) {
			dev_err(dev, "%s is already attached to other domain\n",
				dev_name(drvdata->dev));
			spin_unlock_irqrestore(&drvdata->lock, flags);
			goto err_drvdata_add;
		}
		spin_unlock_irqrestore(&drvdata->lock, flags);
	}

	dev_info(dev, "attached with pgtable %pa\n", &domain->page_table);

	return 0;

err_drvdata_add:
	while (i-- > 0) {
		drvdata = client->sysmmus[i];

		samsung_sysmmu_detach_drvdata(drvdata);
	}

	return -EINVAL;
}

static void samsung_sysmmu_detach_dev(struct iommu_domain *dom,
				      struct device *dev)
{
	struct iommu_fwspec *fwspec = dev_iommu_fwspec_get(dev);
	struct sysmmu_clientdata *client;
	struct samsung_sysmmu_domain *domain;
	struct list_head *group_list;
	struct sysmmu_drvdata *drvdata;
	struct iommu_group *group = dev->iommu_group;
	int i;

	domain = to_sysmmu_domain(dom);
	group_list = iommu_group_get_iommudata(group);

	client = fwspec->iommu_priv;
	for (i = 0; i < client->sysmmu_count; i++) {
		drvdata = client->sysmmus[i];

		samsung_sysmmu_detach_drvdata(drvdata);
	}

	dev_info(dev, "detached from pgtable %pa\n", &domain->page_table);
}

static inline sysmmu_pte_t make_sysmmu_pte(phys_addr_t paddr,
					   int pgsize, int attr)
{
	return ((sysmmu_pte_t)((paddr) >> PG_ENT_SHIFT)) | pgsize | attr;
}

static sysmmu_pte_t *alloc_lv2entry(struct samsung_sysmmu_domain *domain,
				    sysmmu_pte_t *sent, sysmmu_iova_t iova,
				    atomic_t *pgcounter)
{
	if (lv1ent_section(sent)) {
		WARN(1, "trying mapping on %#08x mapped with 1MiB page", iova);
		return ERR_PTR(-EADDRINUSE);
	}

	if (lv1ent_unmapped(sent)) {
		unsigned long flags;
		sysmmu_pte_t *pent;

		pent = kmem_cache_zalloc(slpt_cache, GFP_KERNEL);
		if (!pent)
			return ERR_PTR(-ENOMEM);

		spin_lock_irqsave(&domain->pgtablelock, flags);
		if (lv1ent_unmapped(sent)) {
			*sent = make_sysmmu_pte(virt_to_phys(pent),
						SLPD_FLAG, 0);
			kmemleak_ignore(pent);
			atomic_set(pgcounter, 0);
			pgtable_flush(pent, pent + NUM_LV2ENTRIES);
			pgtable_flush(sent, sent + 1);
		} else {
			/* allocated entry is not used, so free it. */
			kmem_cache_free(slpt_cache, pent);
		}
		spin_unlock_irqrestore(&domain->pgtablelock, flags);
	}

	return page_entry(sent, iova);
}

static inline void clear_lv2_page_table(sysmmu_pte_t *ent, int n)
{
	memset(ent, 0, sizeof(*ent) * n);
}

static int lv1set_section(struct samsung_sysmmu_domain *domain,
			  sysmmu_pte_t *sent, sysmmu_iova_t iova,
			  phys_addr_t paddr, int prot, atomic_t *pgcnt)
{
	int attr = !!(prot & IOMMU_CACHE) ? FLPD_SHAREABLE_FLAG : 0;
	bool need_sync = false;

	if (lv1ent_section(sent)) {
		WARN(1, "Trying mapping 1MB@%#08x on valid FLPD", iova);
		return -EADDRINUSE;
	}

	if (lv1ent_page(sent)) {
		if (WARN_ON(atomic_read(pgcnt) != 0)) {
			WARN(1, "Trying mapping 1MB@%#08x on valid SLPD", iova);
			return -EADDRINUSE;
		}

		kmem_cache_free(slpt_cache, page_entry(sent, 0));
		atomic_set(pgcnt, NUM_LV2ENTRIES);
		need_sync = true;
	}

	*sent = make_sysmmu_pte(paddr, SECT_FLAG, attr);
	pgtable_flush(sent, sent + 1);

	if (need_sync) {
		struct iommu_iotlb_gather gather = {
			.start = iova,
			.end = iova + SECT_SIZE,
		};

		iommu_tlb_sync(&domain->domain, &gather);
	}

	return 0;
}

static int lv2set_page(sysmmu_pte_t *pent, phys_addr_t paddr,
		       size_t size, int prot, atomic_t *pgcnt)
{
	int attr = !!(prot & IOMMU_CACHE) ? SLPD_SHAREABLE_FLAG : 0;

	if (size == SPAGE_SIZE) {
		if (WARN_ON(!lv2ent_unmapped(pent)))
			return -EADDRINUSE;

		*pent = make_sysmmu_pte(paddr, SPAGE_FLAG, attr);
		pgtable_flush(pent, pent + 1);
		atomic_inc(pgcnt);
	} else {	/* size == LPAGE_SIZE */
		int i;

		for (i = 0; i < SPAGES_PER_LPAGE; i++, pent++) {
			if (WARN_ON(!lv2ent_unmapped(pent))) {
				clear_lv2_page_table(pent - i, i);
				return -EADDRINUSE;
			}

			*pent = make_sysmmu_pte(paddr, LPAGE_FLAG, attr);
		}
		pgtable_flush(pent - SPAGES_PER_LPAGE, pent);
		atomic_add(SPAGES_PER_LPAGE, pgcnt);
	}

	return 0;
}

static int samsung_sysmmu_map(struct iommu_domain *dom, unsigned long l_iova,
			      phys_addr_t paddr, size_t size, int prot)
{
	struct samsung_sysmmu_domain *domain = to_sysmmu_domain(dom);
	sysmmu_iova_t iova = (sysmmu_iova_t)l_iova;
	atomic_t *lv2entcnt = &domain->lv2entcnt[lv1ent_offset(iova)];
	sysmmu_pte_t *entry;
	int ret = -ENOMEM;

	entry = section_entry(domain->page_table, iova);

	if (size == SECT_SIZE) {
		ret = lv1set_section(domain, entry, iova, paddr, prot,
				     lv2entcnt);
	} else {
		sysmmu_pte_t *pent;

		pent = alloc_lv2entry(domain, entry, iova, lv2entcnt);

		if (IS_ERR(pent))
			ret = PTR_ERR(pent);
		else
			ret = lv2set_page(pent, paddr, size, prot, lv2entcnt);
	}

	if (ret)
		pr_err("failed to map %#zx @ %#x, ret:%d\n", size, iova, ret);

	return ret;
}

static size_t samsung_sysmmu_unmap(struct iommu_domain *dom,
				   unsigned long l_iova, size_t size,
				   struct iommu_iotlb_gather *gather)
{
	struct samsung_sysmmu_domain *domain = to_sysmmu_domain(dom);
	sysmmu_iova_t iova = (sysmmu_iova_t)l_iova;
	atomic_t *lv2entcnt = &domain->lv2entcnt[lv1ent_offset(iova)];
	sysmmu_pte_t *sent, *pent;
	size_t err_pgsize;

	sent = section_entry(domain->page_table, iova);

	if (lv1ent_section(sent)) {
		if (WARN_ON(size < SECT_SIZE)) {
			err_pgsize = SECT_SIZE;
			goto err;
		}

		*sent = 0;
		pgtable_flush(sent, sent + 1);
		size = SECT_SIZE;
		goto done;
	}

	if (unlikely(lv1ent_unmapped(sent))) {
		if (size > SECT_SIZE)
			size = SECT_SIZE;
		goto done;
	}

	/* lv1ent_page(sent) == true here */

	pent = page_entry(sent, iova);

	if (unlikely(lv2ent_unmapped(pent))) {
		size = SPAGE_SIZE;
		goto done;
	}

	if (lv2ent_small(pent)) {
		*pent = 0;
		size = SPAGE_SIZE;
		pgtable_flush(pent, pent + 1);
		atomic_dec(lv2entcnt);
		goto done;
	}

	/* lv1ent_large(pent) == true here */
	if (WARN_ON(size < LPAGE_SIZE)) {
		err_pgsize = LPAGE_SIZE;
		goto err;
	}

	clear_lv2_page_table(pent, SPAGES_PER_LPAGE);
	pgtable_flush(pent, pent + SPAGES_PER_LPAGE);
	size = LPAGE_SIZE;
	atomic_sub(SPAGES_PER_LPAGE, lv2entcnt);

done:
	iommu_iotlb_gather_add_page(dom, gather, iova, size);

	return size;

err:
	pr_err("failed: size(%#zx) @ %#x is smaller than page size %#zx\n",
	       size, iova, err_pgsize);

	return 0;
}

static void samsung_sysmmu_flush_iotlb_all(struct iommu_domain *dom)
{
	unsigned long flags;
	struct samsung_sysmmu_domain *domain = to_sysmmu_domain(dom);
	struct list_head *sysmmu_list;
	struct sysmmu_drvdata *drvdata;

	/*
	 * domain->group might be NULL if flush_iotlb_all is called
	 * before attach_dev. Just ignore it.
	 */
	if (!domain->group)
		return;

	sysmmu_list = iommu_group_get_iommudata(domain->group);

	list_for_each_entry(drvdata, sysmmu_list, list) {
		spin_lock_irqsave(&drvdata->lock, flags);
		if (drvdata->attached_count &&
		    !pm_runtime_suspended(drvdata->dev))
			__sysmmu_tlb_invalidate_all(drvdata);
		spin_unlock_irqrestore(&drvdata->lock, flags);
	}
}

static void samsung_sysmmu_iotlb_sync(struct iommu_domain *dom,
				      struct iommu_iotlb_gather *gather)
{
	unsigned long flags;
	struct samsung_sysmmu_domain *domain = to_sysmmu_domain(dom);
	struct list_head *sysmmu_list;
	struct sysmmu_drvdata *drvdata;

	/*
	 * domain->group might be NULL if iotlb_sync is called
	 * before attach_dev. Just ignore it.
	 */
	if (!domain->group)
		return;

	sysmmu_list = iommu_group_get_iommudata(domain->group);

	list_for_each_entry(drvdata, sysmmu_list, list) {
		spin_lock_irqsave(&drvdata->lock, flags);
		if (drvdata->attached_count &&
		    !pm_runtime_suspended(drvdata->dev))
			__sysmmu_tlb_invalidate(drvdata,
						gather->start, gather->end);
		spin_unlock_irqrestore(&drvdata->lock, flags);
	}
}

static phys_addr_t samsung_sysmmu_iova_to_phys(struct iommu_domain *dom,
					       dma_addr_t d_iova)
{
	struct samsung_sysmmu_domain *domain = to_sysmmu_domain(dom);
	sysmmu_iova_t iova = (sysmmu_iova_t)d_iova;
	sysmmu_pte_t *entry;
	phys_addr_t phys = 0;

	entry = section_entry(domain->page_table, iova);

	if (lv1ent_section(entry)) {
		phys = section_phys(entry) + section_offs(iova);
	} else if (lv1ent_page(entry)) {
		entry = page_entry(entry, iova);

		if (lv2ent_large(entry))
			phys = lpage_phys(entry) + lpage_offs(iova);
		else if (lv2ent_small(entry))
			phys = spage_phys(entry) + spage_offs(iova);
	}

	return phys;
}

void samsung_sysmmu_dump_pagetable(struct device *dev, dma_addr_t iova)
{
}

static int samsung_sysmmu_add_device(struct device *dev)
{
	struct iommu_fwspec *fwspec = dev_iommu_fwspec_get(dev);
	struct iommu_group *group;
	struct sysmmu_clientdata *client;
	int i;

	if (!fwspec) {
		dev_dbg(dev, "IOMMU instance data is not initialized\n");
		return -ENODEV;
	}

	if (fwspec->ops != &samsung_sysmmu_ops) {
		dev_err(dev, "has different IOMMU ops\n");
		return -ENODEV;
	}

	group = iommu_group_get_for_dev(dev);
	if (IS_ERR(group)) {
		dev_err(dev, "has no suitable IOMMU group. (err:%ld)\n",
			PTR_ERR(group));
		return PTR_ERR(group);
	}

	iommu_group_put(group);

	client = (struct sysmmu_clientdata *)fwspec->iommu_priv;
	client->dev_link = kcalloc(client->sysmmu_count,
				   sizeof(**client->dev_link), GFP_KERNEL);
	if (!client->dev_link)
		return -ENOMEM;

	for (i = 0; i < client->sysmmu_count; i++) {
		client->dev_link[i] =
			device_link_add(dev,
					client->sysmmus[i]->dev,
					DL_FLAG_STATELESS | DL_FLAG_PM_RUNTIME);
		if (!client->dev_link[i]) {
			dev_err(dev, "failed to add device link of %s\n",
				dev_name(client->sysmmus[i]->dev));
			while (i-- > 0)
				device_link_del(client->dev_link[i]);
			return -EINVAL;
		}
		dev_info(dev, "device link to %s\n",
			 dev_name(client->sysmmus[i]->dev));
	}

	return 0;
}

static void samsung_sysmmu_remove_device(struct device *dev)
{
	struct iommu_fwspec *fwspec = dev_iommu_fwspec_get(dev);
	struct sysmmu_clientdata *client;
	int i;

	if (!fwspec || fwspec->ops != &samsung_sysmmu_ops)
		return;

	client = (struct sysmmu_clientdata *)fwspec->iommu_priv;
	for (i = 0; i < client->sysmmu_count; i++)
		device_link_del(client->dev_link[i]);
	kfree(client->dev_link);

	iommu_group_remove_device(dev);
	iommu_fwspec_free(dev);
}

static void samsung_sysmmu_group_data_release(void *iommu_data)
{
	kfree(iommu_data);
}

static struct iommu_group *samsung_sysmmu_device_group(struct device *dev)
{
	struct iommu_group *group;
	struct device_node *np;
	struct platform_device *pdev;
	struct list_head *list;
	bool need_unmanaged_domain = false;

	if (device_iommu_mapped(dev))
		return iommu_group_get(dev);

	np = of_parse_phandle(dev->of_node, "samsung,iommu-group", 0);
	if (!np) {
		dev_err(dev, "group is not registered\n");
		return ERR_PTR(-ENODEV);
	}

	pdev = of_find_device_by_node(np);
	if (!pdev) {
		dev_err(dev, "no device in device_node[%s]\n", np->name);
		of_node_put(np);
		return ERR_PTR(-ENODEV);
	}

	if (of_property_read_bool(np, "samsung,unmanaged-domain"))
		need_unmanaged_domain = true;

	of_node_put(np);

	group = platform_get_drvdata(pdev);
	if (!group) {
		dev_err(dev, "no group in device_node[%s]\n", np->name);
		return ERR_PTR(-EPROBE_DEFER);
	}

	if (iommu_group_get_iommudata(group))
		return group;

	list = kzalloc(sizeof(*list), GFP_KERNEL);
	if (!list)
		return ERR_PTR(-ENOMEM);

	INIT_LIST_HEAD(list);
	iommu_group_set_iommudata(group, list,
				  samsung_sysmmu_group_data_release);

	if (need_unmanaged_domain) {
		int ret;
		struct iommu_domain *domain =
				iommu_domain_alloc(&platform_bus_type);

		ret = iommu_attach_group(domain, group);
		if (ret) {
			dev_err(dev, "failed to attach group, ret:%d\n", ret);
			return ERR_PTR(ret);
		}
	}

	return group;
}

static void samsung_sysmmu_clientdata_release(struct device *dev, void *res)
{
	struct sysmmu_clientdata *client = res;

	kfree(client->sysmmus);
}

static int samsung_sysmmu_of_xlate(struct device *dev,
				   struct of_phandle_args *args)
{
	struct platform_device *sysmmu = of_find_device_by_node(args->np);
	struct sysmmu_drvdata *data = platform_get_drvdata(sysmmu);
	struct sysmmu_drvdata **new_link;
	struct sysmmu_clientdata *client;
	struct iommu_fwspec *fwspec;
	unsigned int fwid = 0;
	int ret;

	iommu_device_link(&data->iommu, dev);
	ret = iommu_fwspec_add_ids(dev, &fwid, 1);
	if (ret) {
		dev_err(dev, "failed to add fwspec. (err:%d)\n", ret);
		iommu_device_unlink(&data->iommu, dev);
		return ret;
	}

	fwspec = dev_iommu_fwspec_get(dev);
	if (!fwspec->iommu_priv) {
		client = devres_alloc(samsung_sysmmu_clientdata_release,
				      sizeof(*client), GFP_KERNEL);
		if (!client)
			return -ENOMEM;
		client->dev = dev;
		fwspec->iommu_priv = client;
		devres_add(dev, client);
	}

	client = (struct sysmmu_clientdata *)fwspec->iommu_priv;
	new_link = krealloc(client->sysmmus,
			    sizeof(*data) * (client->sysmmu_count + 1),
			    GFP_KERNEL);
	if (!new_link)
		return -ENOMEM;

	client->sysmmus = new_link;
	client->sysmmus[client->sysmmu_count++] = data;

	dev_info(dev, "has sysmmu %s (total count:%d)\n",
		 dev_name(data->dev), client->sysmmu_count);

	return ret;
}

static struct iommu_ops samsung_sysmmu_ops = {
	.capable		= samsung_sysmmu_capable,
	.domain_alloc		= samsung_sysmmu_domain_alloc,
	.domain_free		= samsung_sysmmu_domain_free,
	.attach_dev		= samsung_sysmmu_attach_dev,
	.detach_dev		= samsung_sysmmu_detach_dev,
	.map			= samsung_sysmmu_map,
	.unmap			= samsung_sysmmu_unmap,
	.flush_iotlb_all	= samsung_sysmmu_flush_iotlb_all,
	.iotlb_sync		= samsung_sysmmu_iotlb_sync,
	.iova_to_phys		= samsung_sysmmu_iova_to_phys,
	.add_device		= samsung_sysmmu_add_device,
	.remove_device		= samsung_sysmmu_remove_device,
	.device_group		= samsung_sysmmu_device_group,
	.of_xlate		= samsung_sysmmu_of_xlate,
	.pgsize_bitmap		= SECT_SIZE | LPAGE_SIZE | SPAGE_SIZE,
};

static int samsung_sysmmu_fault_notifier(struct device *dev, void *data)
{
	struct iommu_fwspec *fwspec = dev_iommu_fwspec_get(dev);
	struct samsung_sysmmu_fault_info *fi;
	struct sysmmu_clientdata *client;
	struct sysmmu_drvdata *drvdata;
	int i;

	fi = (struct samsung_sysmmu_fault_info *)data;
	drvdata = fi->drvdata;

	client = (struct sysmmu_clientdata *)fwspec->iommu_priv;

	for (i = 0; i < client->sysmmu_count; i++) {
		if (drvdata == client->sysmmus[i]) {
			iommu_report_device_fault(dev, &fi->event);
			break;
		}
	}

	return 0;
}

static inline void sysmmu_tlb_compare(phys_addr_t pgtable,
				      int idx_sub, u32 vpn, u32 ppn)
{
	sysmmu_pte_t *entry;
	unsigned long vaddr = MMU_VADDR_FROM_TLB((unsigned long)vpn, idx_sub);
	unsigned long paddr = MMU_PADDR_FROM_TLB((unsigned long)ppn);
	unsigned long phys = 0;

	if (!pgtable)
		return;

	entry = section_entry(phys_to_virt(pgtable), vaddr);

	if (lv1ent_section(entry)) {
		phys = section_phys(entry);
	} else if (lv1ent_page(entry)) {
		entry = page_entry(entry, vaddr);

		if (lv2ent_large(entry))
			phys = lpage_phys(entry);
		else if (lv2ent_small(entry))
			phys = spage_phys(entry);
	} else {
		pr_crit(">> Invalid address detected! entry: %#lx",
			(unsigned long)*entry);
		return;
	}

	if (paddr != phys) {
		pr_crit(">> TLB mismatch detected!\n");
		pr_crit("   TLB: %#010lx, PT entry: %#010lx\n", paddr, phys);
	}
}

static inline void sysmmu_sbb_compare(u32 sbb_vpn, u32 sbb_link,
				      phys_addr_t pgtable)
{
	sysmmu_pte_t *entry;
	unsigned long vaddr = MMU_VADDR_FROM_SBB((unsigned long)sbb_vpn);
	unsigned long paddr = MMU_PADDR_FROM_SBB((unsigned long)sbb_link);
	unsigned long phys = 0;

	if (!pgtable)
		return;

	entry = section_entry(phys_to_virt(pgtable), vaddr);

	if (lv1ent_page(entry)) {
		phys = lv2table_base(entry);

		if (paddr != phys) {
			pr_crit(">> SBB mismatch detected!\n");
			pr_crit("   entry addr: %lx / SBB addr %lx\n",
				paddr, phys);
		}
	} else {
		pr_crit(">> Invalid address detected! entry: %#lx",
			(unsigned long)*entry);
	}
}

static inline
unsigned int dump_tlb_entry_port_type(struct sysmmu_drvdata *drvdata,
				      phys_addr_t pgtable,
				      int idx_way, int idx_set, int idx_sub)
{
	u32 attr = readl_relaxed(MMU_REG(drvdata, IDX_TLB_ATTR));

	if (MMU_TLB_ENTRY_VALID(attr)) {
		u32 vpn, ppn;

		vpn = readl_relaxed(MMU_REG(drvdata, IDX_TLB_VPN)) + idx_sub;
		ppn = readl_relaxed(MMU_REG(drvdata, IDX_TLB_PPN));

		pr_crit("[%02d][%02d] VPN: %#010x, PPN: %#010x, ATTR: %#010x\n",
			idx_way, idx_set, vpn, ppn, attr);
		sysmmu_tlb_compare(pgtable, idx_sub, vpn, ppn);

		return 1;
	}

	return 0;
}

#define MMU_NUM_TLB_SUBLINE		4
static unsigned int dump_tlb_entry_port(struct sysmmu_drvdata *drvdata,
					phys_addr_t pgtable,
					int tlb, int way, int num_set)
{
	int cnt = 0;
	int set, line, val;

	for (set = 0; set < num_set; set++) {
		for (line = 0; line < MMU_NUM_TLB_SUBLINE; line++) {
			val = MMU_CAPA1_SET_TLB_READ_ENTRY(tlb, set, way, line);
			writel_relaxed(val, MMU_REG(drvdata, IDX_TLB_READ));
			cnt += dump_tlb_entry_port_type(drvdata, pgtable,
							way, set, line);
		}
	}

	return cnt;
}

static inline void dump_sysmmu_tlb_port(struct sysmmu_drvdata *drvdata,
					phys_addr_t pgtable)
{
	int t, i;
	u32 capa0, capa1, info;
	unsigned int cnt;
	int num_tlb, num_port, num_sbb;
	void __iomem *sfrbase = drvdata->sfrbase;

	capa0 = readl_relaxed(sfrbase + REG_MMU_CAPA0_V7);
	capa1 = readl_relaxed(sfrbase + REG_MMU_CAPA1_V7);

	num_tlb = MMU_CAPA1_NUM_TLB(capa1);
	num_port = MMU_CAPA1_NUM_PORT(capa1);
	num_sbb = 1 << MMU_CAPA_NUM_SBB_ENTRY(capa0);

	pr_crit("SysMMU has %d TLBs, %d ports, %d sbb entries\n",
		num_tlb, num_port, num_sbb);

	for (t = 0; t < num_tlb; t++) {
		int num_set, num_way;

		info = readl_relaxed(sfrbase + MMU_TLB_INFO(t));
		num_way = MMU_CAPA1_NUM_TLB_WAY(info);
		num_set = MMU_CAPA1_NUM_TLB_SET(info);

		pr_crit("TLB.%d has %d way, %d set.\n", t, num_way, num_set);
		pr_crit("------------- TLB[WAY][SET][ENTRY] -------------\n");
		for (i = 0, cnt = 0; i < num_way; i++)
			cnt += dump_tlb_entry_port(drvdata, pgtable,
						   t, i, num_set);
	}
	if (!cnt)
		pr_crit(">> No Valid TLB Entries\n");

	pr_crit("--- SBB(Second-Level Page Table Base Address Buffer) ---\n");
	for (i = 0, cnt = 0; i < num_sbb; i++) {
		u32 sbb_vpn, sbblink;

		writel_relaxed(i, MMU_REG(drvdata, IDX_SBB_READ));
		sbb_vpn = readl_relaxed(MMU_REG(drvdata, IDX_SBB_VPN));

		if (MMU_SBB_ENTRY_VALID(sbb_vpn)) {
			sbblink = readl_relaxed(MMU_REG(drvdata, IDX_SBB_LINK));

			pr_crit("[%02d] VPN: %#010x, PPN: %#010x, ATTR: %#010x",
				i, sbb_vpn, sbblink,
				readl_relaxed(MMU_REG(drvdata, IDX_SBB_ATTR)));
			sysmmu_sbb_compare(sbb_vpn, sbblink, pgtable);
			cnt++;
		}
	}
	if (!cnt)
		pr_crit(">> No Valid SBB Entries\n");
}

static inline void dump_sysmmu_status(struct sysmmu_drvdata *drvdata,
				      phys_addr_t pgtable)
{
	int info;
	void __iomem *sfrbase = drvdata->sfrbase;

	info = MMU_RAW_VER(readl_relaxed(sfrbase + REG_MMU_VERSION));

	pr_crit("ADDR: (VA: %p), MMU_CTRL: %#010x, PT_BASE: %#010x\n",
		sfrbase,
		readl_relaxed(sfrbase + REG_MMU_CTRL),
		readl_relaxed(MMU_REG(drvdata, IDX_FLPT_BASE)));
	pr_crit("VERSION %d.%d.%d, MMU_CFG: %#010x, MMU_STATUS: %#010x\n",
		MMU_MAJ_VER(info), MMU_MIN_VER(info), MMU_REV_VER(info),
		readl_relaxed(sfrbase + REG_MMU_CFG),
		readl_relaxed(sfrbase + REG_MMU_STATUS));

	if (drvdata->has_vcr)
		pr_crit("MMU_CTRL_VM: %#010x, MMU_CFG_VM: %#010x\n",
			readl_relaxed(sfrbase + REG_MMU_CTRL_VM),
			readl_relaxed(sfrbase + REG_MMU_CFG_VM));

	if (IS_TLB_PORT_TYPE(drvdata))
		dump_sysmmu_tlb_port(drvdata, pgtable);
}

static inline void sysmmu_show_fault_information(struct sysmmu_drvdata *drvdata,
						 int intr_type,
						 unsigned long fault_addr)
{
	unsigned int info;
	phys_addr_t pgtable;

	pgtable = readl_relaxed(MMU_REG(drvdata, IDX_FLPT_BASE));
	pgtable <<= PAGE_SHIFT;

	info = readl_relaxed(MMU_REG(drvdata, IDX_FAULT_TRANS_INFO));

	pr_crit("----------------------------------------------------------\n");
	pr_crit("From [%s], SysMMU %s %s at %#010lx (page table @ %pa)\n",
		dev_name(drvdata->dev), IS_READ_FAULT(info) ? "READ" : "WRITE",
		sysmmu_fault_name[intr_type], fault_addr, &pgtable);

	if (intr_type == SYSMMU_FAULT_UNKNOWN) {
		pr_crit("The fault is not caused by this System MMU.\n");
		pr_crit("Please check IRQ and SFR base address.\n");
		goto finish;
	}

	pr_crit("AxID: %#x, AxLEN: %#x\n", info & 0xFFFF, (info >> 16) & 0xF);

	if (pgtable != drvdata->pgtable)
		pr_crit("Page table base of driver: %pa\n",
			&drvdata->pgtable);

	if (!pfn_valid(pgtable >> PAGE_SHIFT)) {
		pr_crit("Page table base is not in a valid memory region\n");
		pgtable = 0;
	} else {
		sysmmu_pte_t *ent;

		ent = section_entry(phys_to_virt(pgtable), fault_addr);
		pr_crit("Lv1 entry: %#010x\n", *ent);

		if (lv1ent_page(ent)) {
			ent = page_entry(ent, fault_addr);
			pr_crit("Lv2 entry: %#010x\n", *ent);
		}
	}

	if (intr_type == SYSMMU_FAULT_PTW_ACCESS) {
		pr_crit("System MMU has failed to access page table\n");
		pgtable = 0;
	}

	dump_sysmmu_status(drvdata, pgtable);
finish:
	pr_crit("----------------------------------------------------------\n");
}

static void sysmmu_get_interrupt_info(struct sysmmu_drvdata *data,
				      int *intr_type, unsigned long *addr)
{
	*intr_type =  __ffs(__sysmmu_get_intr_status(data));
	*intr_type %= 4;
	*addr = __sysmmu_get_fault_address(data);
}

static irqreturn_t samsung_sysmmu_irq(int irq, void *dev_id)
{
	int itype;
	unsigned long addr;
	struct sysmmu_drvdata *drvdata = dev_id;

	dev_info(drvdata->dev, "irq(%d) happened\n", irq);

	sysmmu_get_interrupt_info(drvdata, &itype, &addr);
	sysmmu_show_fault_information(drvdata, itype, addr);

	return IRQ_WAKE_THREAD;
}

static irqreturn_t samsung_sysmmu_irq_thread(int irq, void *dev_id)
{
	int itype;
	unsigned long addr;
	struct sysmmu_drvdata *drvdata = dev_id;
	struct iommu_group *group = drvdata->group;
	struct samsung_sysmmu_fault_info fi = {
		.drvdata = drvdata,
		.event.fault.type = IOMMU_FAULT_DMA_UNRECOV,
	};

	sysmmu_get_interrupt_info(drvdata, &itype, &addr);
	fi.event.fault.event.addr = addr;
	fi.event.fault.event.reason = sysmmu_fault_type[itype];
	iommu_group_for_each_dev(group, &fi, samsung_sysmmu_fault_notifier);

	panic("Unrecoverable System MMU Fault!!");

	return IRQ_HANDLED;
}

static int sysmmu_get_hw_info(struct sysmmu_drvdata *data)
{
	struct tlb_props *tlb_props = &data->tlb_props;

	data->version = __sysmmu_get_hw_version(data);

	/* Default value */
	data->reg_set = sysmmu_reg_set[REG_IDX_DEFAULT];

	/*
	 * If CAPA1 doesn't exist, sysmmu uses TLB way dedication.
	 * If CAPA1[31:28] is zero, sysmmu uses TLB port dedication.
	 */
	if (!__sysmmu_has_capa1(data)) {
		tlb_props->flags |= TLB_TYPE_WAY;
	} else {
		if (__sysmmu_get_capa_type(data) == 0)
			tlb_props->flags |= TLB_TYPE_PORT;
		if (__sysmmu_get_capa_vcr_enabled(data)) {
			data->reg_set = sysmmu_reg_set[REG_IDX_VM];
			data->has_vcr = true;
		}
		if (__sysmmu_get_capa_no_block_mode(data))
			data->no_block_mode = true;
	}

	return 0;
}

static int sysmmu_parse_tlb_way_dt(struct device *dev,
				   struct sysmmu_drvdata *drvdata)
{
	const char *props_name = "sysmmu,tlb_property";
	struct tlb_props *tlb_props = &drvdata->tlb_props;
	struct tlb_priv_id *id_cfg = NULL;
	struct tlb_priv_addr *addr_cfg = NULL;
	int i, cnt, id_cnt = 0, addr_cnt = 0;
	unsigned int id_idx = 0, addr_idx = 0;
	unsigned int prop;
	int ret;

	/* Parsing TLB way properties */
	cnt = of_property_count_u32_elems(dev->of_node, props_name);
	for (i = 0; i < cnt; i += 2) {
		ret = of_property_read_u32_index(dev->of_node, props_name, i,
						 &prop);
		if (ret) {
			dev_err(dev,
				"failed to get property. cnt = %d, ret = %d\n",
				i, ret);
			return -EINVAL;
		}

		switch (prop & WAY_TYPE_MASK) {
		case _PRIVATE_WAY_ID:
			id_cnt++;
			tlb_props->flags |= TLB_WAY_PRIVATE_ID;
			break;
		case _PRIVATE_WAY_ADDR:
			addr_cnt++;
			tlb_props->flags |= TLB_WAY_PRIVATE_ADDR;
			break;
		case _PUBLIC_WAY:
			tlb_props->flags |= TLB_WAY_PUBLIC;
			tlb_props->way_props.public_cfg = prop & ~WAY_TYPE_MASK;
			break;
		default:
			dev_err(dev, "Undefined properties!: %#x\n", prop);
			break;
		}
	}

	if (id_cnt) {
		id_cfg = kcalloc(id_cnt, sizeof(*id_cfg), GFP_KERNEL);
		if (!id_cfg)
			return -ENOMEM;
	}

	if (addr_cnt) {
		addr_cfg = kcalloc(addr_cnt, sizeof(*addr_cfg), GFP_KERNEL);
		if (!addr_cfg) {
			ret = -ENOMEM;
			goto err_priv_id;
		}
	}

	for (i = 0; i < cnt; i += 2) {
		ret = of_property_read_u32_index(dev->of_node, props_name, i,
						 &prop);
		if (ret) {
			dev_err(dev, "failed to get TLB property of %d/%d\n",
				i, cnt);
			ret = -EINVAL;
			goto err_priv_addr;
		}

		switch (prop & WAY_TYPE_MASK) {
		case _PRIVATE_WAY_ID:
			id_cfg[id_idx].cfg = prop & ~WAY_TYPE_MASK;
			ret = of_property_read_u32_index(dev->of_node,
							 props_name, i + 1,
							 &id_cfg[id_idx].id);
			if (ret) {
				dev_err(dev,
					"failed to get ID property of %d/%d\n",
					i + 1, cnt);
				goto err_priv_addr;
			}
			id_idx++;
			break;
		case _PRIVATE_WAY_ADDR:
			addr_cfg[addr_idx].cfg = prop & ~WAY_TYPE_MASK;
			addr_idx++;
			break;
		case _PUBLIC_WAY:
			break;
		}
	}

	tlb_props->way_props.priv_id_cfg = id_cfg;
	tlb_props->way_props.priv_id_cnt = id_cnt;

	tlb_props->way_props.priv_addr_cfg = addr_cfg;
	tlb_props->way_props.priv_addr_cnt = addr_cnt;

	return 0;

err_priv_addr:
	kfree(addr_cfg);
err_priv_id:
	kfree(id_cfg);

	return ret;
}

static int sysmmu_parse_tlb_port_dt(struct device *dev,
				    struct sysmmu_drvdata *drvdata)
{
	const char *props_name = "sysmmu,tlb_property";
	const char *slot_props_name = "sysmmu,slot_property";
	struct tlb_props *tlb_props = &drvdata->tlb_props;
	struct tlb_port_cfg *port_cfg = NULL;
	unsigned int *slot_cfg = NULL;
	int i, cnt, ret;
	int port_id_cnt = 0;

	cnt = of_property_count_u32_elems(dev->of_node, slot_props_name);
	if (cnt > 0) {
		slot_cfg = kcalloc(cnt, sizeof(*slot_cfg), GFP_KERNEL);
		if (!slot_cfg)
			return -ENOMEM;

		for (i = 0; i < cnt; i++) {
			ret = of_property_read_u32_index(dev->of_node,
							 slot_props_name,
							 i, &slot_cfg[i]);
			if (ret) {
				dev_err(dev,
					"failed to read slot_property %d/%d\n",
					i, cnt);
				ret = -EINVAL;
				goto err_slot_prop;
			}
		}

		tlb_props->port_props.slot_cnt = cnt;
		tlb_props->port_props.slot_cfg = slot_cfg;
	}

	cnt = of_property_count_u32_elems(dev->of_node, props_name);
	if (cnt <= 0) {
		dev_info(dev, "No TLB port propeties found.\n");
		return 0;
	}

	port_cfg = kcalloc(cnt / 2, sizeof(*port_cfg), GFP_KERNEL);
	if (!port_cfg) {
		ret = -ENOMEM;
		goto err_slot_prop;
	}

	for (i = 0; i < cnt; i += 2) {
		ret = of_property_read_u32_index(dev->of_node,
						 props_name, i,
						 &port_cfg[port_id_cnt].cfg);
		if (ret) {
			dev_err(dev, "failed to read tlb_property of %d/%d\n",
				i, cnt);
			ret = -EINVAL;
			goto err_port_prop;
		}

		ret = of_property_read_u32_index(dev->of_node,
						 props_name, i + 1,
						 &port_cfg[port_id_cnt].id);
		if (ret) {
			dev_err(dev, "failed to read tlb_property of %d/%d\n",
				i + 1, cnt);
			ret = -EINVAL;
			goto err_port_prop;
		}
		port_id_cnt++;
	}

	tlb_props->port_props.port_id_cnt = port_id_cnt;
	tlb_props->port_props.port_cfg = port_cfg;

	return 0;

err_port_prop:
	kfree(port_cfg);

err_slot_prop:
	kfree(slot_cfg);

	return ret;
}

static int sysmmu_parse_dt(struct device *sysmmu, struct sysmmu_drvdata *data)
{
	unsigned int qos = DEFAULT_QOS_VALUE;
	int ret;

	/* Parsing QoS */
	ret = of_property_read_u32_index(sysmmu->of_node, "qos", 0, &qos);
	if (!ret && qos > 15) {
		dev_err(sysmmu, "Invalid QoS value %d, use default.\n", qos);
		qos = DEFAULT_QOS_VALUE;
	}

	data->qos = qos;

	if (IS_TLB_WAY_TYPE(data)) {
		ret = sysmmu_parse_tlb_way_dt(sysmmu, data);
		if (ret)
			dev_err(sysmmu, "Failed to parse TLB way property\n");
	} else if (IS_TLB_PORT_TYPE(data)) {
		ret = sysmmu_parse_tlb_port_dt(sysmmu, data);
		if (ret)
			dev_err(sysmmu, "Failed to parse TLB port property\n");
	};

	return ret;
}

static void sysmmu_release_tlb_info(struct sysmmu_drvdata *data)
{
	if (IS_TLB_WAY_TYPE(data)) {
		kfree(data->tlb_props.way_props.priv_id_cfg);
		kfree(data->tlb_props.way_props.priv_addr_cfg);
	} else if (IS_TLB_PORT_TYPE(data)) {
		kfree(data->tlb_props.port_props.slot_cfg);
		kfree(data->tlb_props.port_props.port_cfg);
	}
}

static int samsung_sysmmu_init_global(void)
{
	int ret = 0;

	flpt_cache = kmem_cache_create("samsung-iommu-lv1table",
				       LV1TABLE_SIZE, LV1TABLE_SIZE,
				       0, NULL);
	if (!flpt_cache)
		return -ENOMEM;

	slpt_cache = kmem_cache_create("samsung-iommu-lv2table",
				       LV2TABLE_SIZE, LV2TABLE_SIZE,
				       0, NULL);
	if (!slpt_cache) {
		ret = -ENOMEM;
		goto err_init_slpt_fail;
	}

	bus_set_iommu(&platform_bus_type, &samsung_sysmmu_ops);

	device_initialize(&sync_dev);
	sysmmu_global_init_done = true;

	return 0;

err_init_slpt_fail:
	kmem_cache_destroy(flpt_cache);

	return ret;
}

static int samsung_sysmmu_device_probe(struct platform_device *pdev)
{
	struct sysmmu_drvdata *data;
	struct device *dev = &pdev->dev;
	struct resource *res;
	int irq, ret, err = 0;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "failed to get resource info\n");
		return -ENOENT;
	}

	data->sfrbase = devm_ioremap_resource(dev, res);
	if (IS_ERR(data->sfrbase))
		return PTR_ERR(data->sfrbase);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	ret = devm_request_threaded_irq(dev, irq, samsung_sysmmu_irq,
					samsung_sysmmu_irq_thread,
					IRQF_ONESHOT, dev_name(dev), data);
	if (ret) {
		dev_err(dev, "unabled to register handler of irq %d\n", irq);
		return ret;
	}

	data->clk = devm_clk_get(dev, "gate");
	if (PTR_ERR(data->clk) == -ENOENT) {
		dev_info(dev, "no gate clock exists. it's okay.\n");
		data->clk = NULL;
	} else if (IS_ERR(data->clk)) {
		dev_err(dev, "failed to get clock!\n");
		return PTR_ERR(data->clk);
	}

	ret = sysmmu_get_hw_info(data);
	if (ret) {
		dev_err(dev, "failed to get h/w info\n");
		return ret;
	}

	INIT_LIST_HEAD(&data->list);
	spin_lock_init(&data->lock);
	data->dev = dev;

	ret = sysmmu_parse_dt(data->dev, data);
	if (ret)
		return ret;

	err = iommu_device_sysfs_add(&data->iommu, data->dev,
				     NULL, dev_name(dev));
	if (err) {
		dev_err(dev, "failed to register iommu in sysfs\n");
		goto err_sysfs_add;
	}

	iommu_device_set_ops(&data->iommu, &samsung_sysmmu_ops);
	iommu_device_set_fwnode(&data->iommu, dev->fwnode);

	err = iommu_device_register(&data->iommu);
	if (err) {
		dev_err(dev, "failed to register iommu\n");
		goto err_iommu_register;
	}

	if (!sysmmu_global_init_done) {
		err = samsung_sysmmu_init_global();
		if (err) {
			dev_err(dev, "failed to initialize global data\n");
			goto err_global_init;
		}
	}
	pm_runtime_enable(dev);

	platform_set_drvdata(pdev, data);

	dev_info(dev, "initialized IOMMU. Ver %d.%d.%d\n",
		 MMU_MAJ_VER(data->version),
		 MMU_MIN_VER(data->version),
		 MMU_REV_VER(data->version));
	return 0;

err_sysfs_add:
	sysmmu_release_tlb_info(data);
err_global_init:
	iommu_device_unregister(&data->iommu);
err_iommu_register:
	iommu_device_sysfs_remove(&data->iommu);
	return err;
}

static void samsung_sysmmu_device_shutdown(struct platform_device *pdev)
{
}

static int __maybe_unused samsung_sysmmu_runtime_suspend(struct device *sysmmu)
{
	unsigned long flags;
	struct sysmmu_drvdata *drvdata = dev_get_drvdata(sysmmu);

	spin_lock_irqsave(&drvdata->lock, flags);
	if (drvdata->attached_count > 0)
		__sysmmu_disable(drvdata);
	spin_unlock_irqrestore(&drvdata->lock, flags);

	return 0;
}

static int __maybe_unused samsung_sysmmu_runtime_resume(struct device *sysmmu)
{
	unsigned long flags;
	struct sysmmu_drvdata *drvdata = dev_get_drvdata(sysmmu);

	spin_lock_irqsave(&drvdata->lock, flags);
	if (drvdata->attached_count > 0)
		__sysmmu_enable(drvdata);
	spin_unlock_irqrestore(&drvdata->lock, flags);

	return 0;
}

static int __maybe_unused samsung_sysmmu_suspend(struct device *dev)
{
	if (pm_runtime_suspended(dev))
		return 0;

	return samsung_sysmmu_runtime_suspend(dev);
}

static int __maybe_unused samsung_sysmmu_resume(struct device *dev)
{
	if (pm_runtime_suspended(dev))
		return 0;

	return samsung_sysmmu_runtime_resume(dev);
}

static const struct dev_pm_ops samsung_sysmmu_pm_ops = {
	SET_RUNTIME_PM_OPS(samsung_sysmmu_runtime_suspend,
			   samsung_sysmmu_runtime_resume, NULL)
	SET_LATE_SYSTEM_SLEEP_PM_OPS(samsung_sysmmu_suspend,
				     samsung_sysmmu_resume)
};

static const struct of_device_id sysmmu_of_match[] = {
	{ .compatible = "samsung,sysmmu-v8" },
	{ }
};

static struct platform_driver samsung_sysmmu_driver = {
	.driver	= {
		.name			= "samsung-sysmmu",
		.of_match_table		= of_match_ptr(sysmmu_of_match),
		.pm			= &samsung_sysmmu_pm_ops,
		.suppress_bind_attrs	= true,
	},
	.probe	= samsung_sysmmu_device_probe,
	.shutdown = samsung_sysmmu_device_shutdown,
};
module_platform_driver(samsung_sysmmu_driver);
MODULE_SOFTDEP("pre: samsung-iommu-group");
MODULE_LICENSE("GPL v2");
