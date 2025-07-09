// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021 Samsung Electronics Co., Ltd.
 */

#define pr_fmt(fmt) "sysmmu: " fmt

#include <linux/smc.h>
#include <linux/arm-smccc.h>
#include <linux/pm_runtime.h>
#include "samsung-iommu-v9.h"

#define SYSMMU_FAULT_PTW_ACCESS		0
#define SYSMMU_FAULT_PAGE		1
#define SYSMMU_FAULT_ACCESS		2
#define SYSMMU_FAULT_CONTEXT		3
#define SYSMMU_FAULT_UNKNOWN		4
#define SYSMMU_FAULTS_NUM		(SYSMMU_FAULT_UNKNOWN + 1)

#define SYSMMU_SEC_FAULT_MASK		(BIT(SYSMMU_FAULT_PTW_ACCESS) | \
					 BIT(SYSMMU_FAULT_PAGE) | \
					 BIT(SYSMMU_FAULT_ACCESS))

#define REG_MMU_PMMU_PTLB_INFO(n)		(0x3400 + ((n) * 0x4))
#define REG_MMU_STLB_INFO(n)			(0x3800 + ((n) * 0x4))
#define REG_MMU_S1L1TLB_INFO			0x3C00

#define REG_MMU_READ_PTLB			0x5000
#define REG_MMU_READ_PTLB_TPN			0x5004
#define REG_MMU_READ_PTLB_PPN			0x5008
#define REG_MMU_READ_PTLB_ATTRIBUTE		0x500C

#define REG_MMU_READ_STLB			0x5010
#define REG_MMU_READ_STLB_TPN			0x5014
#define REG_MMU_READ_STLB_PPN			0x5018
#define REG_MMU_READ_STLB_ATTRIBUTE		0x501C

#define REG_MMU_READ_S1L1TLB			0x5020
#define REG_MMU_READ_S1L1TLB_VPN		0x5024
#define REG_MMU_READ_S1L1TLB_SLPT_OR_PPN	0x5028
#define REG_MMU_READ_S1L1TLB_ATTRIBUTE		0x502C

#define REG_MMU_FAULT_STATUS_VM			0x8060
#define REG_MMU_FAULT_CLEAR_VM			0x8064
#define REG_MMU_FAULT_VA_VM			0x8070
#define REG_MMU_FAULT_INFO0_VM			0x8074
#define REG_MMU_FAULT_INFO1_VM			0x8078
#define REG_MMU_FAULT_INFO2_VM			0x807C
#define REG_MMU_FAULT_RW_MASK			GENMASK(20, 20)
#define IS_READ_FAULT(x)			(((x) & REG_MMU_FAULT_RW_MASK) == 0)

#define MMU_PMMU_INFO_NUM_PTLB(reg)			(((reg) >> 1) & 0x7FFF)
#define MMU_PMMU_PTLB_INFO_NUM_WAY(reg)			(((reg) >> 16) & 0xFFFF)
#define MMU_PMMU_PTLB_INFO_NUM_SET(reg)			((reg) & 0xFFFF)
#define MMU_READ_PTLB_TPN_VALID(reg)			(((reg) >> 28) & 0x1)
#define MMU_READ_PTLB_TPN_S1_ENABLE(reg)		(((reg) >> 24) & 0x1)
#define MMU_VADDR_FROM_PTLB(reg)			(((reg) & 0xFFFFFF) << SPAGE_ORDER)
#define MMU_PADDR_FROM_PTLB(reg)			(((reg) & 0xFFFFFF) << SPAGE_ORDER)
#define MMU_SET_READ_PTLB_ENTRY(way, set, ptlb, pmmu)	((pmmu) | ((ptlb) << 4) |		\
							((set) << 16) | ((way) << 24))

#define MMU_SWALKER_INFO_NUM_STLB(reg)			(((reg) >> 16) & 0xFFFF)
#define MMU_STLB_INFO_NUM_WAY(reg)			(((reg) >> 16) & 0xFFFF)
#define MMU_STLB_INFO_NUM_SET(reg)			((reg) & 0xFFFF)
#define MMU_READ_STLB_TPN_VALID(reg)			(((reg) >> 28) & 0x1)
#define MMU_READ_STLB_TPN_S1_ENABLE(reg)		(((reg) >> 24) & 0x1)
#define MMU_VADDR_FROM_STLB(reg)			(((reg) & 0xFFFFFF) << SPAGE_ORDER)
#define MMU_PADDR_FROM_STLB(reg)			(((reg) & 0xFFFFFF) << SPAGE_ORDER)
#define MMU_SET_READ_STLB_ENTRY(way, set, stlb, line)	((set) | ((way) << 8) |			\
							((line) << 16) | ((stlb) << 20))

#define MMU_VID_FROM_TLB(reg)				(((reg) >> 19) & 0x7)

#define MMU_S1L1TLB_INFO_NUM_SET(reg)			(((reg) >> 16) & 0xFFFF)
#define MMU_S1L1TLB_INFO_NUM_WAY(reg)			(((reg) >> 12) & 0xF)
#define MMU_SET_READ_S1L1TLB_ENTRY(way, set)		((set) | ((way) << 8))
#define MMU_READ_S1L1TLB_VPN_VALID(reg)			(((reg) >> 28) & 0x1)
#define MMU_VADDR_FROM_S1L1TLB(reg)			(((reg) & 0xFFFFFF) << SPAGE_ORDER)
#define MMU_PADDR_FROM_S1L1TLB_PPN(reg)			(((reg) & 0xFFFFFF) << SPAGE_ORDER)
#define MMU_PADDR_FROM_S1L1TLB_BASE(reg)		(((reg) & 0x3FFFFFF) << 10)
#define MMU_S1L1TLB_ATTRIBUTE_PS(reg)			(((reg) >> 8) & 0x7)

#define MMU_FAULT_INFO0_VA_36(reg)			(((reg) >> 21) & 0x1)
#define MMU_FAULT_INFO0_VA_HIGH(reg)			(((reg) & 0x3C00000) << 10)
#define MMU_FAULT_INFO0_LEN(reg)			(((reg) >> 16) & 0xF)
#define MMU_FAULT_INFO0_ASID(reg)			((reg) & 0xFFFF)
#define MMU_FAULT_INFO1_AXID(reg)			(reg)
#define MMU_FAULT_INFO2_PMMU_ID(reg)			(((reg) >> 24) & 0xFF)
#define MMU_FAULT_INFO2_STREAM_ID(reg)			((reg) & 0xFFFFFF)

#define SLPT_BASE_FLAG		0x6

#if IS_ENABLED(CONFIG_EXYNOS_CONTENT_PATH_PROTECTION)
#define SMC_DRM_SEC_SMMU_INFO		(0x820020D0)
#define SMC_DRM_SEC_SYSMMU_INT_CLEAR	(0x820020D7)

/* secure SysMMU SFR access */
enum sec_sysmmu_sfr_access_t {
	SEC_SMMU_SFR_READ,
	SEC_SMMU_SFR_WRITE,
};

#define is_secure_info_fail(x)		((((x) >> 16) & 0xffff) == 0xdead)
static inline u32 read_sec_info(unsigned int addr)
{
	struct arm_smccc_res res;

	arm_smccc_smc(SMC_DRM_SEC_SMMU_INFO,
		      (unsigned long)addr, 0, SEC_SMMU_SFR_READ, 0, 0, 0, 0,
		      &res);
	if (is_secure_info_fail(res.a0))
		pr_err("Invalid value returned, %#lx\n", res.a0);

	return (u32)res.a0;
}

static inline u32 clear_sec_fault(unsigned int addr, unsigned int val)
{
	struct arm_smccc_res res;

	arm_smccc_smc(SMC_DRM_SEC_SYSMMU_INT_CLEAR,
		      (unsigned long)addr, (unsigned long)val, 0, 0, 0, 0, 0,
		      &res);
	return (u32)res.a0;
}

#else
static inline u32 read_sec_info(unsigned int addr)
{
	return 0xdead;
}

static inline u32 clear_sec_fault(unsigned int addr, unsigned int val)
{
	return 0;
}

#endif

static char *sysmmu_fault_name[SYSMMU_FAULTS_NUM] = {
	"PTW ACCESS FAULT",
	"PAGE FAULT",
	"ACCESS FAULT",
	"CONTEXT FAULT",
	"UNKNOWN FAULT"
};

static int sysmmu_fault_type[SYSMMU_FAULTS_NUM] = {
	IOMMU_FAULT_REASON_WALK_EABT,
	IOMMU_FAULT_REASON_PTE_FETCH,
	IOMMU_FAULT_REASON_ACCESS,
	IOMMU_FAULT_REASON_PASID_FETCH,
	IOMMU_FAULT_REASON_UNKNOWN,
};

struct samsung_sysmmu_fault_info {
	struct sysmmu_drvdata *drvdata;
	struct iommu_fault_event event;
};

static inline u32 __sysmmu_get_intr_status(struct sysmmu_drvdata *data, bool is_secure, int *vmid)
{
	int i;
	u32 val = 0x0;

	for (i = 0; i < data->max_vm; i++) {
		if (is_secure)
			val = read_sec_info(MMU_VM_ADDR(data->secure_base +
					    REG_MMU_FAULT_STATUS_VM, i));
		else
			val = readl_relaxed(MMU_VM_ADDR(data->sfrbase +
					    REG_MMU_FAULT_STATUS_VM, i));

		if (val & GENMASK(SYSMMU_FAULTS_NUM - 1, 0)) {
			*vmid = i;
			break;
		}
	}

	return val;
}

static inline sysmmu_iova_t __sysmmu_get_fault_address(struct sysmmu_drvdata *data,
						       bool is_secure, int vmid)
{
	sysmmu_iova_t va = 0x0;
	u32 val;

	if (is_secure) {
		va = read_sec_info(MMU_VM_ADDR(data->secure_base + REG_MMU_FAULT_VA_VM, vmid));
		val = read_sec_info(MMU_VM_ADDR(data->secure_base + REG_MMU_FAULT_INFO0_VM, vmid));
	} else {
		va = readl_relaxed(MMU_VM_ADDR(data->sfrbase + REG_MMU_FAULT_VA_VM, vmid));
		val = readl_relaxed(MMU_VM_ADDR(data->sfrbase + REG_MMU_FAULT_INFO0_VM, vmid));
	}

	if (MMU_FAULT_INFO0_VA_36(val))
		va += MMU_FAULT_INFO0_VA_HIGH(val);

	return va;
}

static void sysmmu_tlb_compare(phys_addr_t pgtable[MAX_VIDS], u32 vpn, u32 ppn, u32 attr)
{
	sysmmu_pte_t *entry;
	unsigned long iova = MMU_VADDR_FROM_PTLB((unsigned long)vpn);
	unsigned long paddr = MMU_PADDR_FROM_PTLB((unsigned long)ppn);
	unsigned int vid = MMU_VID_FROM_TLB(attr);
	unsigned long phys;

	if (!pgtable[vid])
		return;

	entry = section_entry(phys_to_virt(pgtable[vid]), iova);

	if (lv1ent_section(entry)) {
		phys = section_phys(entry) + section_offs(iova);
	} else if (lv1ent_page(entry)) {
		entry = page_entry(entry, iova);

		if (lv2ent_large(entry)) {
			phys = lpage_phys(entry) + lpage_offs(iova);
		} else if (lv2ent_small(entry)) {
			phys = spage_phys(entry) + spage_offs(iova);
		} else {
			pr_crit(">> Invalid SLPD detected: %#010lx\n", (unsigned long)*entry);
			return;
		}
	} else {
		pr_crit(">> Invalid FLPD detected: %#010lx\n", (unsigned long)*entry);
		return;
	}

	if (paddr != phys) {
		pr_crit(">> TLB mismatch detected!\n");
		pr_crit("   TLB: %#011lx, PT entry: %#011lx\n", paddr, phys);
	}
}

static inline int __dump_ptlb_entry(struct sysmmu_drvdata *drvdata,  phys_addr_t pgtable[MAX_VIDS],
				    int idx_way, int idx_set)
{
	u32 tpn = readl_relaxed(drvdata->sfrbase + REG_MMU_READ_PTLB_TPN);

	if (MMU_READ_PTLB_TPN_VALID(tpn)) {
		u32 attr, ppn;

		ppn = readl_relaxed(drvdata->sfrbase + REG_MMU_READ_PTLB_PPN);
		attr = readl_relaxed(drvdata->sfrbase + REG_MMU_READ_PTLB_ATTRIBUTE);

		if (MMU_READ_PTLB_TPN_S1_ENABLE(tpn)) {
			pr_crit("[%02d][%02d] VPN: %#010x, PPN: %#010x, ATTR: %#010x\n",
				idx_way, idx_set, tpn, ppn, attr);

			sysmmu_tlb_compare(pgtable, tpn, ppn, attr);
		} else {
			pr_crit("[%02d][%02d] TPN: %#010x, PPN: %#010x, ATTR: %#010x\n",
				idx_way, idx_set, tpn, ppn, attr);
		}

		return 1;
	}

	return 0;
}

static inline int dump_ptlb_entry(struct sysmmu_drvdata *drvdata, phys_addr_t pgtable[MAX_VIDS],
				  int ptlb_id, int way, int num_set, int pmmu_id)
{
	int cnt = 0;
	int set;
	u32 val;

	for (set = 0; set < num_set; set++) {
		val = MMU_SET_READ_PTLB_ENTRY(way, set, ptlb_id, pmmu_id);
		writel_relaxed(val, drvdata->sfrbase + REG_MMU_READ_PTLB);
		cnt += __dump_ptlb_entry(drvdata, pgtable, way, set);
	}

	return cnt;
}

static inline void dump_sysmmu_ptlb_status(struct sysmmu_drvdata *drvdata,
					   phys_addr_t pgtable[MAX_VIDS], int num_ptlb, int pmmu_id)
{
	int way, t;
	unsigned int cnt = 0;
	u32 info;

	for (t = 0; t < num_ptlb; t++) {
		int num_way, num_set;

		info = readl_relaxed(drvdata->sfrbase + REG_MMU_PMMU_PTLB_INFO(t));
		num_way = MMU_PMMU_PTLB_INFO_NUM_WAY(info);
		num_set = MMU_PMMU_PTLB_INFO_NUM_SET(info);

		pr_crit("PMMU.%d PTLB.%d has %d way, %d set.\n", pmmu_id, t, num_way, num_set);
		pr_crit("------------- PTLB[WAY][SET][ENTRY] -------------\n");
		for (way = 0; way < num_way; way++)
			cnt += dump_ptlb_entry(drvdata, pgtable, t, way, num_set, pmmu_id);
	}
	if (!cnt)
		pr_crit(">> No Valid PTLB Entries\n");
}

static inline int __dump_stlb_entry(struct sysmmu_drvdata *drvdata, phys_addr_t pgtable[MAX_VIDS],
				    int idx_way, int idx_set, int idx_sub)
{
	u32 tpn = readl_relaxed(drvdata->sfrbase + REG_MMU_READ_STLB_TPN);

	if (MMU_READ_STLB_TPN_VALID(tpn)) {
		u32 ppn, attr;

		ppn = readl_relaxed(drvdata->sfrbase + REG_MMU_READ_STLB_PPN);
		attr = readl_relaxed(drvdata->sfrbase + REG_MMU_READ_STLB_ATTRIBUTE);

		if (MMU_READ_STLB_TPN_S1_ENABLE(tpn)) {
			tpn += idx_sub;

			pr_crit("[%02d][%02d] VPN: %#010x, PPN: %#010x, ATTR: %#010x\n",
				idx_way, idx_set, tpn, ppn, attr);

			sysmmu_tlb_compare(pgtable, tpn, ppn, attr);
		} else {
			pr_crit("[%02d][%02d] TPN: %#010x, PPN: %#010x, ATTR: %#010x\n",
				idx_way, idx_set, tpn, ppn, attr);
		}

		return 1;
	}

	return 0;
}

#define MMU_NUM_STLB_SUBLINE		4
static unsigned int dump_stlb_entry(struct sysmmu_drvdata *drvdata, phys_addr_t pgtable[MAX_VIDS],
				    int stlb_id, int way, int num_set)
{
	int cnt = 0;
	int set, line;
	u32 val;

	for (set = 0; set < num_set; set++) {
		for (line = 0; line < MMU_NUM_STLB_SUBLINE; line++) {
			val = MMU_SET_READ_STLB_ENTRY(way, set, stlb_id, line);
			writel_relaxed(val, drvdata->sfrbase + REG_MMU_READ_STLB);
			cnt += __dump_stlb_entry(drvdata, pgtable, way, set, line);
		}
	}

	return cnt;
}

static inline void dump_sysmmu_stlb_status(struct sysmmu_drvdata *drvdata,
					   phys_addr_t pgtable[MAX_VIDS], int num_stlb)
{
	int way, t;
	unsigned int cnt = 0;
	u32 info;

	for (t = 0; t < num_stlb; t++) {
		int num_way, num_set;

		info = readl_relaxed(drvdata->sfrbase + REG_MMU_STLB_INFO(t));
		num_way = MMU_STLB_INFO_NUM_WAY(info);
		num_set = MMU_STLB_INFO_NUM_SET(info);

		pr_crit("STLB.%d has %d way, %d set.\n", t, num_way, num_set);
		pr_crit("------------- STLB[WAY][SET][ENTRY] -------------\n");
		for (way = 0; way < num_way; way++)
			cnt += dump_stlb_entry(drvdata, pgtable, t, way, num_set);
	}
	if (!cnt)
		pr_crit(">> No Valid STLB Entries\n");
}

static inline void sysmmu_s1l1tlb_compare(phys_addr_t pgtable[MAX_VIDS], u32 vpn, u32 base_or_ppn,
					  u32 attr, bool is_slptbase_tlb)
{
	sysmmu_pte_t *sent;
	sysmmu_iova_t iova = MMU_VADDR_FROM_S1L1TLB((unsigned long)vpn);
	unsigned int vid = MMU_VID_FROM_TLB(attr);
	unsigned long paddr_tlb;
	unsigned long paddr_pt = 0;
	bool is_slptbase_pt = false;

	if (!pgtable[vid])
		return;

	sent = section_entry(phys_to_virt(pgtable[vid]), iova);

	if (is_slptbase_tlb)
		paddr_tlb = MMU_PADDR_FROM_S1L1TLB_BASE((unsigned long)base_or_ppn);
	else
		paddr_tlb = MMU_PADDR_FROM_S1L1TLB_PPN((unsigned long)base_or_ppn);

	if (lv1ent_section(sent)) {
		paddr_pt = section_phys(sent);
	} else if (lv1ent_page(sent)) {
		is_slptbase_pt = true;
		paddr_pt = lv2table_base(sent);
	} else {
		pr_crit(">> Invalid FLPD detected: %#010lx\n", (unsigned long)*sent);
		return;
	}

	if (paddr_tlb != paddr_pt || is_slptbase_tlb != is_slptbase_pt) {
		pr_crit(">> S1L1TLB mismatch detected!\n");
		pr_crit("   TLB: %#011lx SLPT base address: %s\n",
			paddr_tlb, is_slptbase_tlb ? "yes" : "no");
		pr_crit("    PT: %#011lx SLPT base address: %s\n",
			paddr_pt, is_slptbase_pt ? "yes" : "no");
	}
}

static inline int dump_s1l1tlb_entry(struct sysmmu_drvdata *drvdata, phys_addr_t pgtable[MAX_VIDS],
				     int way, int set)
{
	bool is_slptbase;
	u32 vpn = readl_relaxed(drvdata->sfrbase + REG_MMU_READ_S1L1TLB_VPN);

	if (MMU_READ_S1L1TLB_VPN_VALID(vpn)) {
		u32 base_or_ppn, attr;

		base_or_ppn = readl_relaxed(drvdata->sfrbase + REG_MMU_READ_S1L1TLB_SLPT_OR_PPN);
		attr = readl_relaxed(drvdata->sfrbase + REG_MMU_READ_S1L1TLB_ATTRIBUTE);
		is_slptbase = (MMU_S1L1TLB_ATTRIBUTE_PS(attr) == SLPT_BASE_FLAG);

		pr_crit("[%02d][%02d] VPN: %#010x, PPN: %#010x, ATTR: %#010x\n",
			way, set, vpn, base_or_ppn, attr);
		sysmmu_s1l1tlb_compare(pgtable, vpn, base_or_ppn, attr, is_slptbase);

		return 1;
	}

	return 0;
}

static inline void dump_sysmmu_s1l1tlb_status(struct sysmmu_drvdata *drvdata,
					      phys_addr_t pgtable[MAX_VIDS])
{
	int way, set;
	unsigned int cnt;
	u32 info;
	int num_way, num_set;

	info = readl_relaxed(drvdata->sfrbase + REG_MMU_S1L1TLB_INFO);
	num_way = MMU_S1L1TLB_INFO_NUM_WAY(info);
	num_set = MMU_S1L1TLB_INFO_NUM_SET(info);

	pr_crit("S1L1TLB has %d way, %d set.\n", num_way, num_set);
	pr_crit("------------- S1L1TLB[WAY][SET][ENTRY] -------------\n");
	for (way = 0, cnt = 0; way < num_way; way++) {
		for (set = 0; set < num_set; set++) {
			writel_relaxed(MMU_SET_READ_S1L1TLB_ENTRY(way, set),
				       drvdata->sfrbase + REG_MMU_READ_S1L1TLB);
			cnt += dump_s1l1tlb_entry(drvdata, pgtable, way, set);
		}
	}
	if (!cnt)
		pr_crit(">> No Valid S1L1TLB Entries\n");
}

static inline void dump_sysmmu_tlb_status(struct sysmmu_drvdata *drvdata,
					  phys_addr_t pgtable[MAX_VIDS], int pmmu_id)
{
	u32 pmmu, swalker;
	int num_stlb, num_ptlb;
	void __iomem *sfrbase = drvdata->sfrbase;

	swalker = readl_relaxed(sfrbase + REG_MMU_SWALKER_INFO);
	num_stlb = MMU_SWALKER_INFO_NUM_STLB(swalker);

	writel_relaxed(MMU_SET_PMMU_INDICATOR(pmmu_id), sfrbase + REG_MMU_PMMU_INDICATOR);
	readl_relaxed(sfrbase + REG_MMU_PMMU_INDICATOR);
	pmmu = readl_relaxed(sfrbase + REG_MMU_PMMU_INFO);
	num_ptlb = MMU_PMMU_INFO_NUM_PTLB(pmmu);

	pr_crit("SysMMU has %d PTLBs(PMMU %d), %d STLBs, 1 S1L1TLB\n",
		num_ptlb, pmmu_id, num_stlb);

	dump_sysmmu_ptlb_status(drvdata, pgtable, num_ptlb, pmmu_id);
	dump_sysmmu_stlb_status(drvdata, pgtable, num_stlb);
	dump_sysmmu_s1l1tlb_status(drvdata, pgtable);
}

static inline void dump_sysmmu_status(struct sysmmu_drvdata *drvdata, phys_addr_t pgtable[MAX_VIDS],
				      int vmid, int pmmu_id)
{
	int info;
	void __iomem *sfrbase = drvdata->sfrbase;

	info = MMU_VERSION_RAW(readl_relaxed(sfrbase + REG_MMU_VERSION));

	pr_crit("MMU_CTRL: %#010x, PT_BASE: %#010x, VID: %d\n",
		readl_relaxed(sfrbase + REG_MMU_CTRL),
		readl_relaxed(MMU_VM_ADDR(sfrbase + REG_MMU_CONTEXT0_CFG_FLPT_BASE_VM, vmid)),
			      vmid);
	pr_crit("VERSION %d.%d.%d, MMU_STATUS: %#010x\n",
		MMU_VERSION_MAJOR(info), MMU_VERSION_MINOR(info), MMU_VERSION_REVISION(info),
		readl_relaxed(sfrbase + REG_MMU_STATUS));

	pr_crit("MMU_CTRL_VM: %#010x, MMU_CFG_VM: %#010x\n",
		readl_relaxed(MMU_VM_ADDR(sfrbase + REG_MMU_CTRL_VM, vmid)),
		readl_relaxed(MMU_VM_ADDR(sfrbase + REG_MMU_CONTEXT0_CFG_ATTRIBUTE_VM, vmid)));

	dump_sysmmu_tlb_status(drvdata, pgtable, pmmu_id);
}

static void sysmmu_get_fault_msg(struct sysmmu_drvdata *drvdata, int intr_type,
				 unsigned int vmid, sysmmu_iova_t fault_addr,
				 bool is_secure, char *fault_msg, size_t fault_msg_sz)
{
	const char *port_name = NULL;
	unsigned int info0, info1, info2;

	of_property_read_string(drvdata->dev->of_node, "port-name", &port_name);

	if (is_secure) {
		unsigned int sfrbase = drvdata->secure_base;

		info0 = read_sec_info(MMU_VM_ADDR(sfrbase + REG_MMU_FAULT_INFO0_VM, vmid));
		scnprintf(fault_msg, fault_msg_sz,
			  "SysMMU %s %s from %s (secure) VID %u at %#011llx",
			  IS_READ_FAULT(info0) ? "READ" : "WRITE",
			  sysmmu_fault_name[intr_type],
			  port_name ? port_name : dev_name(drvdata->dev), vmid,
			  fault_addr);
	} else {
		info0 = readl_relaxed(MMU_VM_ADDR(drvdata->sfrbase + REG_MMU_FAULT_INFO0_VM, vmid));
		info1 = readl_relaxed(MMU_VM_ADDR(drvdata->sfrbase + REG_MMU_FAULT_INFO1_VM, vmid));
		info2 = readl_relaxed(MMU_VM_ADDR(drvdata->sfrbase + REG_MMU_FAULT_INFO2_VM, vmid));
		scnprintf(fault_msg, fault_msg_sz,
			  "SysMMU %s %s from %s PMMU %u VID %u Stream ID %#x AXI ID %#x at %#011llx",
			  IS_READ_FAULT(info0) ? "READ" : "WRITE",
			  sysmmu_fault_name[intr_type],
			  port_name ? port_name : dev_name(drvdata->dev),
			  MMU_FAULT_INFO2_PMMU_ID(info2), vmid,
			  MMU_FAULT_INFO2_STREAM_ID(info2), MMU_FAULT_INFO1_AXID(info1),
			  fault_addr);
	}
}

static void sysmmu_show_secure_fault_information(struct sysmmu_drvdata *drvdata, int intr_type,
						 sysmmu_iova_t fault_addr, int vmid)
{
	unsigned int info0, info1, info2;
	phys_addr_t pgtable;
	unsigned int sfrbase = drvdata->secure_base;
	char err_msg[128];

	pgtable = read_sec_info(MMU_VM_ADDR(sfrbase + REG_MMU_CONTEXT0_CFG_FLPT_BASE_VM, vmid));
	pgtable <<= PT_BASE_SHIFT;

	info0 = read_sec_info(MMU_VM_ADDR(sfrbase + REG_MMU_FAULT_INFO0_VM, vmid));
	info1 = read_sec_info(MMU_VM_ADDR(sfrbase + REG_MMU_FAULT_INFO1_VM, vmid));
	info2 = read_sec_info(MMU_VM_ADDR(sfrbase + REG_MMU_FAULT_INFO2_VM, vmid));

	pr_crit("----------------------------------------------------------\n");

	sysmmu_get_fault_msg(drvdata, intr_type, vmid, fault_addr,
			     true, err_msg, sizeof(err_msg));

	pr_crit("%s (pgtable @ %pa)\n", err_msg, &pgtable);

	if (intr_type == SYSMMU_FAULT_UNKNOWN) {
		pr_crit("The fault is not caused by this System MMU.\n");
		pr_crit("Please check IRQ and SFR base address.\n");
		goto finish;
	}

	if (intr_type == SYSMMU_FAULT_CONTEXT) {
		pr_crit("Context fault\n");
		goto finish;
	}

	pr_crit("ASID: %#x, Burst LEN: %#x, AXI ID: %#x, PMMU ID: %#x, STREAM ID: %#x\n",
		MMU_FAULT_INFO0_ASID(info0), MMU_FAULT_INFO0_LEN(info0),
		MMU_FAULT_INFO1_AXID(info1), MMU_FAULT_INFO2_PMMU_ID(info2),
		MMU_FAULT_INFO2_STREAM_ID(info2));

	if (!pfn_valid(PFN_DOWN(pgtable))) {
		pr_crit("Page table base is not in a valid memory region\n");
		pgtable = 0;
	}

	if (intr_type == SYSMMU_FAULT_PTW_ACCESS) {
		pr_crit("System MMU has failed to access page table\n");
		pgtable = 0;
	}

	info0 = MMU_VERSION_RAW(read_sec_info(sfrbase + REG_MMU_VERSION));

	pr_crit("ADDR: %#x, MMU_CTRL: %#010x, PT_BASE: %#010x\n",
		sfrbase,
		read_sec_info(sfrbase + REG_MMU_CTRL),
		read_sec_info(MMU_VM_ADDR(sfrbase + REG_MMU_CONTEXT0_CFG_FLPT_BASE_VM, vmid)));
	pr_crit("VERSION %d.%d.%d, MMU_STATUS: %#010x\n",
		MMU_VERSION_MAJOR(info0), MMU_VERSION_MINOR(info0), MMU_VERSION_REVISION(info0),
		read_sec_info(sfrbase + REG_MMU_STATUS));

	pr_crit("MMU_CTRL_VM: %#010x, MMU_CFG_VM: %#010x\n",
		read_sec_info(MMU_VM_ADDR(sfrbase + REG_MMU_CTRL_VM, vmid)),
		read_sec_info(MMU_VM_ADDR(sfrbase + REG_MMU_CONTEXT0_CFG_ATTRIBUTE_VM, vmid)));

finish:
	pr_crit("----------------------------------------------------------\n");
}

static void sysmmu_show_fault_info_simple(struct sysmmu_drvdata *drvdata, int intr_type,
					  sysmmu_iova_t fault_addr, int vmid)
{
	phys_addr_t pgtable;
	char err_msg[128];

	pgtable = readl_relaxed(MMU_VM_ADDR(drvdata->sfrbase + REG_MMU_CONTEXT0_CFG_FLPT_BASE_VM,
					    vmid));
	pgtable <<= PT_BASE_SHIFT;

	sysmmu_get_fault_msg(drvdata, intr_type, vmid, fault_addr,
			     false, err_msg, sizeof(err_msg));

	pr_crit("%s (pgtable @ %pa)\n", err_msg, &pgtable);
}

static void sysmmu_show_fault_information(struct sysmmu_drvdata *drvdata, int intr_type,
					  sysmmu_iova_t fault_addr, int vmid)
{
	unsigned int i;
	phys_addr_t pgtable[MAX_VIDS];
	u32 info0, info2;
	int pmmu_id;

	for (i = 0; i < MAX_VIDS; i++) {
		pgtable[i] = readl_relaxed(MMU_VM_ADDR(drvdata->sfrbase +
						       REG_MMU_CONTEXT0_CFG_FLPT_BASE_VM, i));
		pgtable[i] <<= PT_BASE_SHIFT;
	}

	pr_crit("----------------------------------------------------------\n");
	sysmmu_show_fault_info_simple(drvdata, intr_type, fault_addr, vmid);

	info0 = readl_relaxed(MMU_VM_ADDR(drvdata->sfrbase + REG_MMU_FAULT_INFO0_VM, vmid));
	info2 = readl_relaxed(MMU_VM_ADDR(drvdata->sfrbase + REG_MMU_FAULT_INFO2_VM, vmid));
	pmmu_id = MMU_FAULT_INFO2_PMMU_ID(info2);

	pr_crit("ASID: %d, Burst LEN: %d\n",
		MMU_FAULT_INFO0_ASID(info0), MMU_FAULT_INFO0_LEN(info0));

	if (intr_type == SYSMMU_FAULT_UNKNOWN) {
		pr_crit("The fault is not caused by this System MMU.\n");
		pr_crit("Please check IRQ and SFR base address.\n");
		goto finish;
	}

	if (intr_type == SYSMMU_FAULT_CONTEXT) {
		pr_crit("Context fault\n");
		goto finish;
	}

	if (pgtable[vmid] != drvdata->pgtable[vmid])
		pr_crit("Page table base of driver: %p\n", &drvdata->pgtable[vmid]);

	if (!pfn_valid(PFN_DOWN(pgtable[vmid]))) {
		pr_crit("Page table base is not in a valid memory region\n");
		pgtable[vmid] = 0;
	} else {
		sysmmu_pte_t *ent;
		phys_addr_t phys;

		ent = section_entry(phys_to_virt(pgtable[vmid]), fault_addr);
		phys = virt_to_phys(ent);
		pr_crit("Lv1 entry: %#010x @ %pap\n", *ent, &phys);

		if (lv1ent_page(ent)) {
			ent = page_entry(ent, fault_addr);
			phys = virt_to_phys(ent);
			pr_crit("Lv2 entry: %#010x @ %pap\n", *ent, &phys);
		}
	}

	if (intr_type == SYSMMU_FAULT_PTW_ACCESS) {
		pr_crit("System MMU has failed to access page table\n");
		pgtable[vmid] = 0;
	}

	dump_sysmmu_status(drvdata, pgtable, vmid, pmmu_id);
finish:
	pr_crit("----------------------------------------------------------\n");
}

static void sysmmu_get_interrupt_info(struct sysmmu_drvdata *data,
				      int *intr_type, sysmmu_iova_t *addr,
				      int *vmid, bool is_secure)
{
	u32 intr_status = __sysmmu_get_intr_status(data, is_secure, vmid);

	if (!intr_status) {
		dev_err_ratelimited(data->dev, "Spurious interrupt\n");
		*intr_type = SYSMMU_FAULT_UNKNOWN;
		*addr = 0;
		return;
	}

	*intr_type = __ffs(intr_status);
	*addr = __sysmmu_get_fault_address(data, is_secure, *vmid);
}

static int sysmmu_clear_interrupt(struct sysmmu_drvdata *data, bool is_secure, int *vmid)
{
	u32 val = __sysmmu_get_intr_status(data, is_secure, vmid);

	if (!val) {
		dev_err_ratelimited(data->dev, "Cannot clear spurious interrupt\n");
		return -EINVAL;
	}

	if (is_secure) {
		if (val & ~SYSMMU_SEC_FAULT_MASK) {
			dev_warn(data->dev, "Unknown secure fault (%x)\n", val);
			val &= SYSMMU_SEC_FAULT_MASK;
		}
		/* LDFW calculates the physical address of the
		 * MMU_FAULT_CLEAR_VM_n using the formula
		 * base + REG_MMU_FAULT_CLEAR_VM
		 * where base is the value of the first parameter that we pass
		 * to clear_sec_fault().  If we pass
		 * MMU_VM_ADDR(data->secure_base, *vmid) as base, then the
		 * resulting address will be
		 * MMU_VM_ADDR(data->secure_base, *vmid) + REG_MMU_FAULT_CLEAR_VM
		 * which is the same as
		 * MMU_VM_ADDR(data->secure_base + REG_MMU_FAULT_CLEAR_VM, *vmid)
		 */
		return clear_sec_fault(MMU_VM_ADDR(data->secure_base, *vmid), val);
	}
	writel(val, MMU_VM_ADDR(data->sfrbase + REG_MMU_FAULT_CLEAR_VM, *vmid));
	return 0;
}

irqreturn_t samsung_sysmmu_irq(int irq, void *dev_id)
{
	int itype, vmid;
	sysmmu_iova_t addr;
	struct sysmmu_drvdata *drvdata = dev_id;
	bool is_secure = (irq == drvdata->secure_irq);

	if (drvdata->hide_page_fault)
		return IRQ_WAKE_THREAD;

	dev_info(drvdata->dev, "[%s] interrupt (%d) happened\n",
		 is_secure ? "Secure" : "Non-secure", irq);

	if (drvdata->async_fault_mode)
		return IRQ_WAKE_THREAD;

	sysmmu_get_interrupt_info(drvdata, &itype, &addr, &vmid, is_secure);
	if (is_secure)
		sysmmu_show_secure_fault_information(drvdata, itype, addr, vmid);
	else
		sysmmu_show_fault_information(drvdata, itype, addr, vmid);

	return IRQ_WAKE_THREAD;
}

static int samsung_sysmmu_fault_notifier(struct device *dev, void *data)
{
	struct samsung_sysmmu_fault_info *fi;
	struct sysmmu_clientdata *client;
	struct sysmmu_drvdata *drvdata;
	int i, ret, result = 0;

	fi = (struct samsung_sysmmu_fault_info *)data;
	drvdata = fi->drvdata;

	client = (struct sysmmu_clientdata *)dev_iommu_priv_get(dev);

	for (i = 0; i < client->sysmmu_count; i++) {
		if (drvdata == client->sysmmus[i]) {
			ret = iommu_report_device_fault(dev, &fi->event);
			if (ret == -EAGAIN)
				result = ret;
			break;
		}
	}

	return result;
}

irqreturn_t samsung_sysmmu_irq_thread(int irq, void *dev_id)
{
	int itype, vmid, ret;
	sysmmu_iova_t addr;
	struct sysmmu_drvdata *drvdata = dev_id;
	bool is_secure = (irq == drvdata->secure_irq);
	struct iommu_group *group = drvdata->group;
	enum iommu_fault_reason reason;
	struct samsung_sysmmu_fault_info fi = {
		.drvdata = drvdata,
		.event.fault.type = IOMMU_FAULT_DMA_UNRECOV,
	};
	char fault_msg[128];

	/* Prevent power down while handling faults */
	pm_runtime_get_sync(drvdata->dev);

	sysmmu_get_interrupt_info(drvdata, &itype, &addr, &vmid, is_secure);
	reason = sysmmu_fault_type[itype];

	fi.event.fault.event.addr = addr;
	fi.event.fault.event.pasid = vmid;
	if (vmid)
		fi.event.fault.event.flags |= IOMMU_FAULT_UNRECOV_PASID_VALID;
	fi.event.fault.event.reason = reason;
	if (reason == IOMMU_FAULT_REASON_PTE_FETCH)
		fi.event.fault.type = IOMMU_FAULT_PAGE_REQ;

	ret = iommu_group_for_each_dev(group, &fi,
				       samsung_sysmmu_fault_notifier);
	if (ret == -EAGAIN) {
		if (is_secure) {
			if (drvdata->async_fault_mode && !drvdata->hide_page_fault)
				sysmmu_show_secure_fault_information(drvdata, itype, addr, vmid);
			ret = sysmmu_clear_interrupt(drvdata, true, &vmid);
			if (ret) {
				if (drvdata->hide_page_fault)
					sysmmu_show_secure_fault_information(drvdata,
									     itype, addr, vmid);
				dev_err(drvdata->dev, "Failed to clear secure fault (%d)\n", ret);
				goto out;
			}
		} else  {
			if (!drvdata->hide_page_fault) {
				if (drvdata->always_dump_full_fault_info)
					sysmmu_show_fault_information(drvdata, itype, addr, vmid);
				else
					sysmmu_show_fault_info_simple(drvdata, itype, addr, vmid);
			}
			sysmmu_clear_interrupt(drvdata, false, &vmid);
		}
		pm_runtime_put(drvdata->dev);
		return IRQ_HANDLED;
	}

	if (drvdata->async_fault_mode || drvdata->hide_page_fault) {
		if (is_secure)
			sysmmu_show_secure_fault_information(drvdata, itype, addr, vmid);
		else
			sysmmu_show_fault_information(drvdata, itype, addr, vmid);
	}

out:
	sysmmu_get_fault_msg(drvdata, itype, vmid, addr, is_secure, fault_msg, sizeof(fault_msg));

	pm_runtime_put(drvdata->dev);

	if (drvdata->panic_action == GO_PANIC_ID)
		panic(fault_msg);
	else
		dbg_snapshot_do_dpm_policy(drvdata->panic_action, fault_msg);

	return IRQ_HANDLED;
}
