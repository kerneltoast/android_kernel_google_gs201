// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 */

#define pr_fmt(fmt) "sysmmu: " fmt

#include <linux/dma-iommu.h>
#include <linux/kmemleak.h>
#include <linux/module.h>
#include <linux/of_iommu.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>

#include "samsung-iommu.h"

#define FLPD_SHAREABLE_FLAG	BIT(6)
#define SLPD_SHAREABLE_FLAG	BIT(4)

#define MMU_TLB_CFG_MASK(reg)		((reg) & (GENMASK(7, 5) | GENMASK(3, 2) | GENMASK(1, 1)))
#define MMU_TLB_MATCH_CFG_MASK(reg)	((reg) & (GENMASK(31, 16) | GENMASK(9, 8)))

#define REG_MMU_TLB_CFG(n)		(0x2000 + ((n) * 0x20) + 0x4)
#define REG_MMU_TLB_MATCH_CFG(n)	(0x2000 + ((n) * 0x20) + 0x8)
#define REG_MMU_TLB_MATCH_ID(n)		(0x2000 + ((n) * 0x20) + 0x14)

#define DEFAULT_QOS_VALUE	-1
#define DEFAULT_TLB_NONE	~0U
#define UNUSED_TLB_INDEX	~0U

#define REG_MMU_S2PF_ENABLE	0x7000
#define MMU_S2PF_ENABLE		BIT(0)

static const unsigned int sysmmu_reg_set[MAX_SET_IDX][MAX_REG_IDX] = {
	/* Default without VM */
	{
		/*
		 * SysMMUs without VM support do not have the two registers CTRL_VM and CFG_VM.
		 * Setting the offsets to 1 will trigger an unaligned access exception.
		 */
		0x1,	0x1,
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
		/* CTRL_VM, CFG_VM */
		0x8000,	0x8004,
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

static struct iommu_ops samsung_sysmmu_ops;
static struct platform_driver samsung_sysmmu_driver;

struct samsung_sysmmu_domain {
	struct iommu_domain domain;
	struct iommu_group *group;
	struct sysmmu_drvdata *vm_sysmmu; /* valid only if vid != 0 */
	/* if vid != 0, domain is aux domain attached to only one device and sysmmu */
	unsigned int vid;
	sysmmu_pte_t *page_table;
	atomic_t *lv2entcnt;
	spinlock_t pgtablelock; /* serialize races to page table updates */
};

static bool sysmmu_global_init_done;
static struct device sync_dev;
static struct kmem_cache *flpt_cache, *slpt_cache;

static inline u32 __sysmmu_get_tlb_num(struct sysmmu_drvdata *data)
{
	return MMU_CAPA1_NUM_TLB(readl_relaxed(data->sfrbase +
					       REG_MMU_CAPA1_V7));
}

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

static inline void __sysmmu_tlb_invalidate_all(struct sysmmu_drvdata *data, unsigned int vid)
{
	writel(0x1, MMU_VM_REG(data, IDX_ALL_INV, vid));
}

static inline void __sysmmu_tlb_invalidate(struct sysmmu_drvdata *data, unsigned int vid,
					   dma_addr_t start, dma_addr_t end)
{
	writel_relaxed(ALIGN_DOWN(start, SPAGE_SIZE), MMU_VM_REG(data, IDX_RANGE_INV_START, vid));
	writel_relaxed(ALIGN_DOWN(end, SPAGE_SIZE), MMU_VM_REG(data, IDX_RANGE_INV_END, vid));
	writel(0x1, MMU_VM_REG(data, IDX_RANGE_INV, vid));
}

static inline void __sysmmu_disable_vid(struct sysmmu_drvdata *data, unsigned int vid)
{
	u32 ctrl_val = readl_relaxed(MMU_VM_REG(data, IDX_CTRL_VM, vid));

	ctrl_val &= ~CTRL_VID_ENABLE;
	writel(ctrl_val, MMU_VM_REG(data, IDX_CTRL_VM, vid));
	__sysmmu_tlb_invalidate_all(data, vid);
	writel_relaxed(0, MMU_VM_REG(data, IDX_FLPT_BASE, vid));
}

static inline void __sysmmu_disable(struct sysmmu_drvdata *data)
{
	if (data->no_block_mode) {
		__sysmmu_tlb_invalidate_all(data, 0);
	} else {
		u32 ctrl_val = readl_relaxed(data->sfrbase + REG_MMU_CTRL);

		ctrl_val &= ~CTRL_MMU_ENABLE;
		writel(ctrl_val | CTRL_MMU_BLOCK, data->sfrbase + REG_MMU_CTRL);
	}
}

static inline void __sysmmu_set_tlb(struct sysmmu_drvdata *data)
{
	struct tlb_props *tlb_props = &data->tlb_props;
	struct tlb_config *cfg = tlb_props->cfg;
	int id_cnt = tlb_props->id_cnt;
	int i;
	unsigned int index;

	if (tlb_props->default_cfg != DEFAULT_TLB_NONE)
		writel_relaxed(MMU_TLB_CFG_MASK(tlb_props->default_cfg),
			       data->sfrbase + REG_MMU_TLB_CFG(0));

	for (i = 0; i < id_cnt; i++) {
		if (cfg[i].index == UNUSED_TLB_INDEX)
			continue;

		index = cfg[i].index;
		writel_relaxed(MMU_TLB_CFG_MASK(cfg[i].cfg),
			       data->sfrbase + REG_MMU_TLB_CFG(index));
		writel_relaxed(MMU_TLB_MATCH_CFG_MASK(cfg[i].match_cfg),
			       data->sfrbase + REG_MMU_TLB_MATCH_CFG(index));
		writel_relaxed(cfg[i].match_id,
			       data->sfrbase + REG_MMU_TLB_MATCH_ID(index));
	}
}

static inline void __sysmmu_init_config(struct sysmmu_drvdata *data)
{
	u32 cfg = readl_relaxed(data->sfrbase + REG_MMU_CFG);

	if (data->qos != DEFAULT_QOS_VALUE) {
		cfg &= ~CFG_QOS(0xF);
		cfg |= CFG_QOS_OVRRIDE | CFG_QOS((u32)data->qos);
	}

	if (data->no_s2pf) {
		u32 val = readl_relaxed(data->sfrbase + REG_MMU_S2PF_ENABLE);

		writel_relaxed(val & ~MMU_S2PF_ENABLE, data->sfrbase + REG_MMU_S2PF_ENABLE);
	}

	__sysmmu_set_tlb(data);

	writel_relaxed(cfg, data->sfrbase + REG_MMU_CFG);
}

static inline void __sysmmu_enable_vid(struct sysmmu_drvdata *data, unsigned int vid)
{
	u32 ctrl_val;

	writel_relaxed(data->pgtable[vid] / SPAGE_SIZE, MMU_VM_REG(data, IDX_FLPT_BASE, vid));

	__sysmmu_tlb_invalidate_all(data, vid);

	ctrl_val = readl_relaxed(MMU_VM_REG(data, IDX_CTRL_VM, vid));
	if (!data->async_fault_mode)
		ctrl_val |= CTRL_FAULT_STALL_MODE;
	else
		ctrl_val &= ~CTRL_FAULT_STALL_MODE;
	writel(ctrl_val | CTRL_VID_ENABLE, MMU_VM_REG(data, IDX_CTRL_VM, vid));
}

static inline void __sysmmu_enable(struct sysmmu_drvdata *data)
{
	unsigned int vid;
	u32 ctrl_val = readl_relaxed(data->sfrbase + REG_MMU_CTRL);

	if (!data->no_block_mode)
		writel_relaxed(ctrl_val | CTRL_MMU_BLOCK, data->sfrbase + REG_MMU_CTRL);

	__sysmmu_init_config(data);

	for (vid = 0; vid < __max_vids(data); vid++)
		if (data->pgtable[vid])
			__sysmmu_enable_vid(data, vid);

	writel(ctrl_val | CTRL_MMU_ENABLE, data->sfrbase + REG_MMU_CTRL);
}

static struct samsung_sysmmu_domain *to_sysmmu_domain(struct iommu_domain *dom)
{
	return container_of(dom, struct samsung_sysmmu_domain, domain);
}

static inline void pgtable_flush(void *vastart, void *vaend)
{
	dma_sync_single_for_device(&sync_dev, virt_to_phys(vastart),
				   (size_t)(vaend - vastart), DMA_TO_DEVICE);
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
		data->pgtable[0] = 0;
		data->group = NULL;
	}
	spin_unlock_irqrestore(&data->lock, flags);
}

static int samsung_sysmmu_set_domain_range(struct iommu_domain *dom,
					   struct device *dev)
{
	struct iommu_domain_geometry *geom = &dom->geometry;
	dma_addr_t start, end;
	size_t size;

	if (of_get_dma_window(dev->of_node, NULL, 0, NULL, &start, &size))
		return 0;

	end = start + size;

	if (end > DMA_BIT_MASK(32))
		end = DMA_BIT_MASK(32);

	if (geom->force_aperture) {
		dma_addr_t d_start, d_end;

		d_start = max(start, geom->aperture_start);
		d_end = min(end, geom->aperture_end);

		if (d_start >= d_end) {
			dev_err(dev, "current range is [%pad..%pad]\n",
				&geom->aperture_start, &geom->aperture_end);
			dev_err(dev, "requested range [%zx @ %pad] is not allowed\n",
				size, &start);
			return -ERANGE;
		}

		geom->aperture_start = d_start;
		geom->aperture_end = d_end;
	} else {
		geom->aperture_start = start;
		geom->aperture_end = end;
		/*
		 * All CPUs should observe the change of force_aperture after
		 * updating aperture_start and aperture_end because dma-iommu
		 * restricts dma virtual memory by this aperture when
		 * force_aperture is set.
		 * We allow allocating dma virtual memory during changing the
		 * aperture range because the current allocation is free from
		 * the new restricted range.
		 */
		smp_wmb();
		geom->force_aperture = true;
	}

	dev_info(dev, "changed DMA range [%pad..%pad] successfully.\n",
		 &geom->aperture_start, &geom->aperture_end);

	return 0;
}

static struct samsung_sysmmu_domain *attach_helper(struct iommu_domain *dom, struct device *dev)
{
	struct iommu_fwspec *fwspec = dev_iommu_fwspec_get(dev);
	struct samsung_sysmmu_domain *domain;

	if (!fwspec || fwspec->ops != &samsung_sysmmu_ops) {
		dev_err(dev, "failed to attach, IOMMU instance data %s.\n",
			!fwspec ? "is not initialized" : "has different ops");
		return ERR_PTR(-ENXIO);
	}

	if (!dev_iommu_priv_get(dev)) {
		dev_err(dev, "has no IOMMU\n");
		return ERR_PTR(-ENODEV);
	}

	domain = to_sysmmu_domain(dom);
	if (domain->vm_sysmmu) {
		dev_err(dev, "IOMMU domain is already used as AUX domain\n");
		return ERR_PTR(-EBUSY);
	}

	return domain;
}

static int samsung_sysmmu_attach_dev(struct iommu_domain *dom,
				     struct device *dev)
{
	struct sysmmu_clientdata *client;
	struct samsung_sysmmu_domain *domain;
	struct list_head *group_list;
	struct sysmmu_drvdata *drvdata;
	struct iommu_group *group = dev->iommu_group;
	unsigned long flags;
	phys_addr_t page_table;
	int i, ret = -EINVAL;

	domain = attach_helper(dom, dev);
	if (IS_ERR(domain))
		return (int)PTR_ERR(domain);

	domain->group = group;
	group_list = iommu_group_get_iommudata(group);
	page_table = virt_to_phys(domain->page_table);

	client = dev_iommu_priv_get(dev);
	for (i = 0; i < (int)client->sysmmu_count; i++) {
		drvdata = client->sysmmus[i];

		spin_lock_irqsave(&drvdata->lock, flags);
		if (drvdata->attached_count++ == 0) {
			list_add(&drvdata->list, group_list);
			drvdata->group = group;
			drvdata->pgtable[0] = page_table;

			if (pm_runtime_active(drvdata->dev))
				__sysmmu_enable(drvdata);
		} else if (drvdata->pgtable[0] != page_table) {
			dev_err(dev, "%s is already attached to other domain\n",
				dev_name(drvdata->dev));
			spin_unlock_irqrestore(&drvdata->lock, flags);
			goto err_drvdata_add;
		}
		spin_unlock_irqrestore(&drvdata->lock, flags);
	}

	ret = samsung_sysmmu_set_domain_range(dom, dev);
	if (ret)
		goto err_drvdata_add;

	dev_info(dev, "attached with pgtable %pa\n", &domain->page_table);

	return 0;

err_drvdata_add:
	while (i-- > 0) {
		drvdata = client->sysmmus[i];

		samsung_sysmmu_detach_drvdata(drvdata);
	}

	return ret;
}

static void samsung_sysmmu_detach_dev(struct iommu_domain *dom,
				      struct device *dev)
{
	struct sysmmu_clientdata *client;
	struct samsung_sysmmu_domain *domain;
	struct list_head *group_list;
	struct sysmmu_drvdata *drvdata;
	struct iommu_group *group = dev->iommu_group;
	unsigned int i;

	domain = to_sysmmu_domain(dom);
	group_list = iommu_group_get_iommudata(group);

	client = dev_iommu_priv_get(dev);
	for (i = 0; i < client->sysmmu_count; i++) {
		drvdata = client->sysmmus[i];

		samsung_sysmmu_detach_drvdata(drvdata);
	}

	dev_info(dev, "detached from pgtable %pa\n", &domain->page_table);
}

static inline sysmmu_pte_t make_sysmmu_pte(phys_addr_t paddr,
					   unsigned int pgsize, unsigned int attr)
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

static inline void clear_lv2_page_table(sysmmu_pte_t *ent, unsigned int n)
{
	memset(ent, 0, sizeof(*ent) * n);
}

static int lv1set_section(struct samsung_sysmmu_domain *domain,
			  sysmmu_pte_t *sent, sysmmu_iova_t iova,
			  phys_addr_t paddr, int prot, atomic_t *pgcnt)
{
	unsigned int attr = !!(prot & IOMMU_CACHE) ? FLPD_SHAREABLE_FLAG : 0;
	sysmmu_pte_t *pent_to_free = NULL;

	if (lv1ent_section(sent)) {
		WARN(1, "Trying mapping 1MB@%#08x on valid FLPD", iova);
		return -EADDRINUSE;
	}

	if (lv1ent_page(sent)) {
		if (WARN_ON(atomic_read(pgcnt) != 0)) {
			WARN(1, "Trying mapping 1MB@%#08x on valid SLPD", iova);
			return -EADDRINUSE;
		}

		pent_to_free = page_entry(sent, 0);
		atomic_set(pgcnt, NUM_LV2ENTRIES);
	}

	*sent = make_sysmmu_pte(paddr, SECT_FLAG, attr);
	pgtable_flush(sent, sent + 1);

	if (pent_to_free) {
		struct iommu_iotlb_gather gather = {
			.start = iova,
			.end = iova + SECT_SIZE - 1,
		};

		iommu_iotlb_sync(&domain->domain, &gather);
		kmem_cache_free(slpt_cache, pent_to_free);
	}

	return 0;
}

static int lv2set_page(sysmmu_pte_t *pent, phys_addr_t paddr,
		       size_t size, int prot, atomic_t *pgcnt)
{
	unsigned int attr = !!(prot & IOMMU_CACHE) ? SLPD_SHAREABLE_FLAG : 0;

	if (size == SPAGE_SIZE) {
		if (WARN_ON(!lv2ent_unmapped(pent)))
			return -EADDRINUSE;

		*pent = make_sysmmu_pte(paddr, SPAGE_FLAG, attr);
		pgtable_flush(pent, pent + 1);
		atomic_inc(pgcnt);
	} else {	/* size == LPAGE_SIZE */
		unsigned int i;

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
			      phys_addr_t paddr, size_t size, int prot,
			      gfp_t unused)
{
	struct samsung_sysmmu_domain *domain = to_sysmmu_domain(dom);
	sysmmu_iova_t iova = (sysmmu_iova_t)l_iova;
	atomic_t *lv2entcnt = &domain->lv2entcnt[lv1ent_offset(iova)];
	sysmmu_pte_t *entry;
	int ret = -ENOMEM;

	/* Do not use IO coherency if iOMMU_PRIV exists */
	if (!!(prot & IOMMU_PRIV))
		prot &= ~IOMMU_CACHE;

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
	struct sysmmu_drvdata *drvdata;

	if (domain->vm_sysmmu) {
		/* Domain is used as AUX domain */
		drvdata = domain->vm_sysmmu;
		spin_lock_irqsave(&drvdata->lock, flags);
		if (drvdata->attached_count && drvdata->rpm_resume)
			__sysmmu_tlb_invalidate_all(drvdata, domain->vid);
		spin_unlock_irqrestore(&drvdata->lock, flags);
	} else if (domain->group) {
		/* Domain is used as regular domain */
		/*
		 * domain->group might be NULL if flush_iotlb_all is called
		 * before attach_dev. Just ignore it.
		 */
		struct list_head *sysmmu_list = iommu_group_get_iommudata(domain->group);

		list_for_each_entry(drvdata, sysmmu_list, list) {
			spin_lock_irqsave(&drvdata->lock, flags);
			if (drvdata->attached_count && drvdata->rpm_resume)
				__sysmmu_tlb_invalidate_all(drvdata, 0);
			spin_unlock_irqrestore(&drvdata->lock, flags);
		}
	}
}

static void samsung_sysmmu_iotlb_sync(struct iommu_domain *dom,
				      struct iommu_iotlb_gather *gather)
{
	unsigned long flags;
	struct samsung_sysmmu_domain *domain = to_sysmmu_domain(dom);
	struct sysmmu_drvdata *drvdata;

	if (domain->vm_sysmmu) {
		/* Domain is used as AUX domain */
		drvdata = domain->vm_sysmmu;
		spin_lock_irqsave(&drvdata->lock, flags);
		if (drvdata->attached_count && drvdata->rpm_resume)
			__sysmmu_tlb_invalidate(drvdata, domain->vid, gather->start, gather->end);
		spin_unlock_irqrestore(&drvdata->lock, flags);
	} else if (domain->group) {
		/* Domain is used as regular domain */
		/*
		 * domain->group might be NULL if iotlb_sync is called
		 * before attach_dev. Just ignore it.
		 */
		struct list_head *sysmmu_list = iommu_group_get_iommudata(domain->group);

		list_for_each_entry(drvdata, sysmmu_list, list) {
			spin_lock_irqsave(&drvdata->lock, flags);
			if (drvdata->attached_count && drvdata->rpm_resume)
				__sysmmu_tlb_invalidate(drvdata, 0, gather->start, gather->end);
			spin_unlock_irqrestore(&drvdata->lock, flags);
		}
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

static struct iommu_device *samsung_sysmmu_probe_device(struct device *dev)
{
	struct iommu_fwspec *fwspec = dev_iommu_fwspec_get(dev);
	struct sysmmu_clientdata *client;
	int i;

	if (!fwspec) {
		dev_dbg(dev, "IOMMU instance data is not initialized\n");
		return ERR_PTR(-ENODEV);
	}

	if (fwspec->ops != &samsung_sysmmu_ops) {
		dev_err(dev, "has different IOMMU ops\n");
		return ERR_PTR(-ENODEV);
	}

	client = (struct sysmmu_clientdata *) dev_iommu_priv_get(dev);
	client->dev_link = kcalloc(client->sysmmu_count,
				   sizeof(**client->dev_link), GFP_KERNEL);
	if (!client->dev_link)
		return ERR_PTR(-ENOMEM);

	for (i = 0; i < (int)client->sysmmu_count; i++) {
		client->dev_link[i] =
			device_link_add(dev,
					client->sysmmus[i]->dev,
					DL_FLAG_STATELESS | DL_FLAG_PM_RUNTIME);
		if (!client->dev_link[i]) {
			dev_err(dev, "failed to add device link of %s\n",
				dev_name(client->sysmmus[i]->dev));
			while (i-- > 0)
				device_link_del(client->dev_link[i]);
			return ERR_PTR(-EINVAL);
		}
		dev_dbg(dev, "device link to %s\n",
			dev_name(client->sysmmus[i]->dev));
	}

	return &client->sysmmus[0]->iommu;
}

static void samsung_sysmmu_release_device(struct device *dev)
{
	struct iommu_fwspec *fwspec = dev_iommu_fwspec_get(dev);
	struct sysmmu_clientdata *client;
	unsigned int i;

	if (!fwspec || fwspec->ops != &samsung_sysmmu_ops)
		return;

	client = (struct sysmmu_clientdata *) dev_iommu_priv_get(dev);
	for (i = 0; i < client->sysmmu_count; i++)
		device_link_del(client->dev_link[i]);
	kfree(client->dev_link);

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

	ret = iommu_fwspec_add_ids(dev, &fwid, 1);
	if (ret) {
		dev_err(dev, "failed to add fwspec. (err:%d)\n", ret);
		iommu_device_unlink(&data->iommu, dev);
		return ret;
	}

	fwspec = dev_iommu_fwspec_get(dev);
	if (!dev_iommu_priv_get(dev)) {
		client = devres_alloc(samsung_sysmmu_clientdata_release,
				      sizeof(*client), GFP_KERNEL);
		if (!client)
			return -ENOMEM;
		dev_iommu_priv_set(dev, client);
		devres_add(&sysmmu->dev, client);
	}

	client = (struct sysmmu_clientdata *) dev_iommu_priv_get(dev);
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

static int samsung_sysmmu_aux_attach_dev(struct iommu_domain *dom, struct device *dev)
{
	struct sysmmu_clientdata *client;
	struct samsung_sysmmu_domain *domain;
	struct sysmmu_drvdata *drvdata;
	unsigned long flags;
	unsigned int vid;

	domain = attach_helper(dom, dev);
	if (IS_ERR(domain))
		return (int)PTR_ERR(domain);

	if (domain->group) {
		dev_err(dev, "IOMMU domain is already in use as vid 0 domain\n");
		return -EBUSY;
	}
	client = (struct sysmmu_clientdata *) dev_iommu_priv_get(dev);
	if (client->sysmmu_count != 1) {
		dev_err(dev, "IOMMU AUX domains not supported for devices served by more than one IOMMU\n");
		return -ENXIO;
	}
	drvdata = client->sysmmus[0];
	if (!drvdata->has_vcr) {
		dev_err(dev, "SysMMU does not support IOMMU AUX domains\n");
		return -ENXIO;
	}
	spin_lock_irqsave(&drvdata->lock, flags);
	if (!drvdata->attached_count) {
		dev_err(dev, "IOMMU needs to be enabled to attach AUX domain\n");
		spin_unlock_irqrestore(&drvdata->lock, flags);
		return -ENXIO;
	}
	for (vid = 1; vid < MAX_VIDS; vid++)
		if (!drvdata->pgtable[vid])
			break;
	if (vid == MAX_VIDS) {
		dev_err(dev, "Unable to allocate vid for AUX domain\n");
		spin_unlock_irqrestore(&drvdata->lock, flags);
		return -EBUSY;
	}
	drvdata->pgtable[vid] = virt_to_phys(domain->page_table);
	if (pm_runtime_active(drvdata->dev))
		__sysmmu_enable_vid(drvdata, vid);
	spin_unlock_irqrestore(&drvdata->lock, flags);
	domain->vm_sysmmu = drvdata;
	domain->vid = vid;
	return 0;
}

static void samsung_sysmmu_aux_detach_dev(struct iommu_domain *dom, struct device *dev)
{
	struct samsung_sysmmu_domain *domain;
	struct sysmmu_drvdata *drvdata;
	unsigned long flags;
	unsigned int vid;

	domain = to_sysmmu_domain(dom);

	if (WARN_ON(!domain->vm_sysmmu || !domain->vid))
		return;

	drvdata = domain->vm_sysmmu;
	vid = domain->vid;

	spin_lock_irqsave(&drvdata->lock, flags);
	drvdata->pgtable[vid] = 0;
	__sysmmu_disable_vid(drvdata, vid);
	spin_unlock_irqrestore(&drvdata->lock, flags);

	domain->vm_sysmmu = NULL;
	domain->vid = 0;
}

static int samsung_sysmmu_aux_get_pasid(struct iommu_domain *dom, struct device *dev)
{
	struct samsung_sysmmu_domain *domain;

	domain = to_sysmmu_domain(dom);

	if (!domain->vm_sysmmu)
		return -EINVAL;

	return (int)domain->vid;
}

static bool samsung_sysmmu_dev_has_feat(struct device *dev, enum iommu_dev_features f)
{
	struct iommu_fwspec *fwspec = dev_iommu_fwspec_get(dev);
	struct sysmmu_clientdata *client;
	struct sysmmu_drvdata *drvdata;

	if (f != IOMMU_DEV_FEAT_AUX)
		return false;

	client = (struct sysmmu_clientdata *) dev_iommu_priv_get(dev);
	if (!fwspec || !client || fwspec->ops != &samsung_sysmmu_ops)
		return false;

	if (client->sysmmu_count != 1)
		return false;
	drvdata = client->sysmmus[0];
	return !!drvdata->has_vcr;
}

static bool samsung_sysmmu_dev_feat_enabled(struct device *dev, enum iommu_dev_features f)
{
	return samsung_sysmmu_dev_has_feat(dev, f);
}

static int samsung_sysmmu_dev_enable_feat(struct device *dev, enum iommu_dev_features f)
{
	if (!samsung_sysmmu_dev_has_feat(dev, f))
		return -EINVAL;
	return 0;
}

static int samsung_sysmmu_dev_disable_feat(struct device *dev, enum iommu_dev_features f)
{
	return -EINVAL;
}

static void samsung_sysmmu_get_resv_regions(struct device *dev, struct list_head *head)
{
	enum iommu_resv_type resvtype[] = {
		IOMMU_RESV_DIRECT, IOMMU_RESV_RESERVED
	};
	const char *propname[ARRAY_SIZE(resvtype)] = {
		"samsung,iommu-identity-map",
		"samsung,iommu-reserved-map"
	};
	int n_addr_cells = of_n_addr_cells(dev->of_node);
	int n_size_cells = of_n_size_cells(dev->of_node);
	int n_all_cells = n_addr_cells + n_size_cells;
	unsigned int type;

	for (type = 0; type < ARRAY_SIZE(propname); type++) {
		const __be32 *prop;
		u64 base, size;
		int i, cnt;

		prop = of_get_property(dev->of_node, propname[type], &cnt);
		if (!prop)
			continue;

		cnt /=  sizeof(u32);
		if (cnt % n_all_cells != 0) {
			dev_err(dev, "Invalid number(%d) of values in %s\n", cnt, propname[type]);
			break;
		}

		for (i = 0; i < cnt; i += n_all_cells) {
			struct iommu_resv_region *region;

			base = of_read_number(prop + i, n_addr_cells);
			size = of_read_number(prop + i + n_addr_cells, n_size_cells);
			if (base & ~dma_get_mask(dev) || (base + size) & ~dma_get_mask(dev)) {
				dev_err(dev, "Unreachable DMA region in %s, [%#lx..%#lx)\n",
					propname[type], (unsigned long)base,
					(unsigned long)(base + size));
				continue;
			}

			region = iommu_alloc_resv_region(base, size, 0, resvtype[type]);
			if (!region)
				continue;

			list_add_tail(&region->list, head);
			dev_info(dev, "Reserved IOMMU mapping [%#lx..%#lx)\n",
				 (unsigned long)base,
				 (unsigned long)(base + size));
		}
	}
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
	.probe_device		= samsung_sysmmu_probe_device,
	.release_device		= samsung_sysmmu_release_device,
	.device_group		= samsung_sysmmu_device_group,
	.of_xlate		= samsung_sysmmu_of_xlate,
	.get_resv_regions	= samsung_sysmmu_get_resv_regions,
	.put_resv_regions	= generic_iommu_put_resv_regions,
	.dev_has_feat		= samsung_sysmmu_dev_has_feat,
	.dev_feat_enabled	= samsung_sysmmu_dev_feat_enabled,
	.dev_enable_feat	= samsung_sysmmu_dev_enable_feat,
	.dev_disable_feat	= samsung_sysmmu_dev_disable_feat,
	.aux_attach_dev		= samsung_sysmmu_aux_attach_dev,
	.aux_detach_dev		= samsung_sysmmu_aux_detach_dev,
	.aux_get_pasid		= samsung_sysmmu_aux_get_pasid,
	.pgsize_bitmap		= SECT_SIZE | LPAGE_SIZE | SPAGE_SIZE,
};

static int sysmmu_get_hw_info(struct sysmmu_drvdata *data)
{
	data->version = __sysmmu_get_hw_version(data);
	data->num_tlb = __sysmmu_get_tlb_num(data);

	/* Default value */
	data->reg_set = sysmmu_reg_set[REG_IDX_DEFAULT];

	if (__sysmmu_get_capa_vcr_enabled(data)) {
		data->reg_set = sysmmu_reg_set[REG_IDX_VM];
		data->has_vcr = true;
	}
	if (__sysmmu_get_capa_no_block_mode(data))
		data->no_block_mode = true;

	return 0;
}

static int sysmmu_parse_tlb_property(struct device *dev,
				     struct sysmmu_drvdata *drvdata)
{
	const char *default_props_name = "sysmmu,default_tlb";
	const char *props_name = "sysmmu,tlb_property";
	struct tlb_props *tlb_props = &drvdata->tlb_props;
	struct tlb_config *cfg;
	int i, cnt, ret;
	size_t readsize;

	if (of_property_read_u32(dev->of_node, default_props_name,
				 &tlb_props->default_cfg))
		tlb_props->default_cfg = DEFAULT_TLB_NONE;

	cnt = of_property_count_elems_of_size(dev->of_node, props_name,
					      sizeof(*cfg));
	if (cnt <= 0)
		return 0;

	cfg = devm_kcalloc(dev, (unsigned int)cnt, sizeof(*cfg), GFP_KERNEL);
	if (!cfg)
		return -ENOMEM;

	readsize = (unsigned int)cnt * sizeof(*cfg) / sizeof(u32);
	ret = of_property_read_variable_u32_array(dev->of_node, props_name,
						  (u32 *)cfg,
						  readsize, readsize);
	if (ret < 0) {
		dev_err(dev, "failed to get tlb property, return %d\n", ret);
		return ret;
	}

	for (i = 0; i < cnt; i++) {
		if (cfg[i].index >= drvdata->num_tlb) {
			dev_err(dev, "invalid index %d is ignored. (max:%d)\n",
				cfg[i].index, drvdata->num_tlb);
			cfg[i].index = UNUSED_TLB_INDEX;
		}
	}

	tlb_props->id_cnt = cnt;
	tlb_props->cfg = cfg;

	return 0;
}

static int __sysmmu_secure_irq_init(struct device *sysmmu,
				    struct sysmmu_drvdata *data)
{
	struct platform_device *pdev = to_platform_device(sysmmu);
	int ret;

	ret = platform_get_irq(pdev, 1);
	if (ret <= 0) {
		dev_err(sysmmu, "unable to find secure IRQ resource\n");
		return -EINVAL;
	}
	data->secure_irq = ret;

	ret = devm_request_threaded_irq(sysmmu, (unsigned int)data->secure_irq,
					samsung_sysmmu_irq,
					samsung_sysmmu_irq_thread,
					IRQF_ONESHOT, dev_name(sysmmu), data);
	if (ret) {
		dev_err(sysmmu, "failed to set secure irq handler %d, ret:%d\n",
			data->secure_irq, ret);
		return ret;
	}

	ret = of_property_read_u32(sysmmu->of_node, "sysmmu,secure_base",
				   &data->secure_base);
	if (ret) {
		dev_err(sysmmu, "failed to get secure base\n");
		return ret;
	}

	return ret;
}

static int sysmmu_parse_dt(struct device *sysmmu, struct sysmmu_drvdata *data)
{
	int qos = DEFAULT_QOS_VALUE;
	int ret;

	/* Parsing QoS */
	ret = of_property_read_u32_index(sysmmu->of_node, "qos", 0, &qos);
	if (!ret && qos > 15) {
		dev_err(sysmmu, "Invalid QoS value %d, use default.\n", qos);
		qos = DEFAULT_QOS_VALUE;
	}

	data->qos = qos;
	data->no_s2pf = of_property_read_bool(sysmmu->of_node, "sysmmu,no-s2pf");

	/* Secure IRQ */
	if (of_find_property(sysmmu->of_node, "sysmmu,secure-irq", NULL)) {
		ret = __sysmmu_secure_irq_init(sysmmu, data);
		if (ret) {
			dev_err(sysmmu, "failed to init secure irq\n");
			return ret;
		}
	}

	data->hide_page_fault = of_property_read_bool(sysmmu->of_node,
						      "sysmmu,hide-page-fault");
	/* use async fault mode */
	data->async_fault_mode = of_property_read_bool(sysmmu->of_node,
						       "sysmmu,async-fault");

	ret = sysmmu_parse_tlb_property(sysmmu, data);
	if (ret)
		dev_err(sysmmu, "Failed to parse TLB property\n");

	return ret;
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

	ret = devm_request_threaded_irq(dev, (unsigned int)irq, samsung_sysmmu_irq,
					samsung_sysmmu_irq_thread,
					IRQF_ONESHOT, dev_name(dev), data);
	if (ret) {
		dev_err(dev, "unabled to register handler of irq %d\n", irq);
		return ret;
	}

	data->clk = devm_clk_get(dev, "gate");
	if (PTR_ERR(data->clk) == -ENOENT) {
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

	ret = iommu_device_sysfs_add(&data->iommu, data->dev,
				     NULL, dev_name(dev));
	if (ret) {
		dev_err(dev, "failed to register iommu in sysfs\n");
		return ret;
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

	dev_info(dev, "initialized IOMMU. Ver %d.%d.%d, %sgate clock\n",
		 MMU_MAJ_VER(data->version),
		 MMU_MIN_VER(data->version),
		 MMU_REV_VER(data->version),
		 data->clk ? "" : "no ");
	return 0;

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
	drvdata->rpm_resume = false;
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
	drvdata->rpm_resume = true;
	if (drvdata->attached_count > 0)
		__sysmmu_enable(drvdata);
	spin_unlock_irqrestore(&drvdata->lock, flags);

	return 0;
}

static int __maybe_unused samsung_sysmmu_suspend(struct device *dev)
{
	if (pm_runtime_status_suspended(dev))
		return 0;

	dev->power.must_resume = true;
	return samsung_sysmmu_runtime_suspend(dev);
}

static int __maybe_unused samsung_sysmmu_resume(struct device *dev)
{
	if (pm_runtime_status_suspended(dev))
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
