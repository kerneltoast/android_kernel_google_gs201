// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 */

#define pr_fmt(fmt) "sysmmu: " fmt

#include <linux/smc.h>
#include <linux/arm-smccc.h>

#include "samsung-iommu.h"

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

#define REG_MMU_INT_STATUS		0x060
#define REG_MMU_INT_CLEAR		0x064
#define REG_MMU_FAULT_VA		0x070
#define REG_MMU_FAULT_TRANS_INFO	0x078
#define REG_MMU_FAULT_RW_MASK		GENMASK(20, 20)
#define IS_READ_FAULT(x)		(((x) & REG_MMU_FAULT_RW_MASK) == 0)

#define SYSMMU_FAULT_PTW_ACCESS   0
#define SYSMMU_FAULT_PAGE_FAULT   1
#define SYSMMU_FAULT_ACCESS       3
#define SYSMMU_FAULT_SECURITY     4
#define SYSMMU_FAULT_UNKNOWN      5

#define SYSMMU_FAULTS_NUM         (SYSMMU_FAULT_UNKNOWN + 1)

#if IS_ENABLED(CONFIG_EXYNOS_CONTENT_PATH_PROTECTION)
#define SMC_DRM_SEC_SMMU_INFO          (0x820020D0)
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
#else
static inline u32 read_sec_info(unsigned int addr)
{
	return 0xdead;
}
#endif

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

struct samsung_sysmmu_fault_info {
	struct sysmmu_drvdata *drvdata;
	struct iommu_fault_event event;
};

static inline u32 __sysmmu_get_intr_status(struct sysmmu_drvdata *data,
					   bool is_secure)
{
	if (is_secure)
		return read_sec_info(data->secure_base + REG_MMU_INT_STATUS);
	else
		return readl_relaxed(data->sfrbase + REG_MMU_INT_STATUS);
}

static inline u32 __sysmmu_get_fault_address(struct sysmmu_drvdata *data,
					     bool is_secure)
{
	if (is_secure)
		return read_sec_info(MMU_SEC_VM_REG(data, IDX_FAULT_VA, 0));
	else
		return readl_relaxed(MMU_VM_REG(data, IDX_FAULT_VA, 0));
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

static inline void dump_sysmmu_tlb_status(struct sysmmu_drvdata *drvdata,
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

	dump_sysmmu_tlb_status(drvdata, pgtable);
}

static void sysmmu_show_secure_fault_information(struct sysmmu_drvdata *drvdata,
						 int intr_type, unsigned long fault_addr)
{
	const char *port_name = NULL;
	unsigned int info;
	phys_addr_t pgtable;
	unsigned int sfrbase = drvdata->secure_base;

	pgtable = read_sec_info(MMU_SEC_REG(drvdata, IDX_SEC_FLPT_BASE));
	pgtable <<= PAGE_SHIFT;

	info = read_sec_info(MMU_SEC_REG(drvdata, IDX_FAULT_TRANS_INFO));

	of_property_read_string(drvdata->dev->of_node, "port-name", &port_name);

	pr_crit("----------------------------------------------------------\n");
	pr_crit("From [%s], SysMMU %s %s at %#010lx (page table @ %pa)\n",
		port_name ? port_name : dev_name(drvdata->dev),
		IS_READ_FAULT(info) ? "READ" : "WRITE",
		sysmmu_fault_name[intr_type], fault_addr, &pgtable);

	if (intr_type == SYSMMU_FAULT_UNKNOWN) {
		pr_crit("The fault is not caused by this System MMU.\n");
		pr_crit("Please check IRQ and SFR base address.\n");
		goto finish;
	}

	pr_crit("AxID: %#x, AxLEN: %#x\n", info & 0xFFFF, (info >> 16) & 0xF);

	if (!pfn_valid(pgtable >> PAGE_SHIFT)) {
		pr_crit("Page table base is not in a valid memory region\n");
		pgtable = 0;
	}

	if (intr_type == SYSMMU_FAULT_PTW_ACCESS) {
		pr_crit("System MMU has failed to access page table\n");
		pgtable = 0;
	}

	info = MMU_RAW_VER(read_sec_info(sfrbase + REG_MMU_VERSION));

	pr_crit("ADDR: %#x, MMU_CTRL: %#010x, PT_BASE: %#010x\n",
		sfrbase,
		read_sec_info(sfrbase + REG_MMU_CTRL),
		read_sec_info(MMU_SEC_REG(drvdata, IDX_SEC_FLPT_BASE)));
	pr_crit("VERSION %d.%d.%d, MMU_CFG: %#010x, MMU_STATUS: %#010x\n",
		MMU_MAJ_VER(info), MMU_MIN_VER(info), MMU_REV_VER(info),
		read_sec_info(sfrbase + REG_MMU_CFG),
		read_sec_info(sfrbase + REG_MMU_STATUS));

finish:
	pr_crit("----------------------------------------------------------\n");
}

static void sysmmu_show_fault_info_simple(struct sysmmu_drvdata *drvdata,
					  int intr_type, unsigned long fault_addr,
					  phys_addr_t *pt)
{
	const char *port_name = NULL;
	u32 info;
	phys_addr_t pgtable;

	pgtable = readl_relaxed(MMU_REG(drvdata, IDX_FLPT_BASE));
	pgtable <<= PAGE_SHIFT;

	info = readl_relaxed(MMU_REG(drvdata, IDX_FAULT_TRANS_INFO));

	of_property_read_string(drvdata->dev->of_node, "port-name", &port_name);

	pr_crit("From [%s], SysMMU %s %s at %#010lx (pgtable @ %pa, AxID: %#x)\n",
		port_name ? port_name : dev_name(drvdata->dev),
		IS_READ_FAULT(info) ? "READ" : "WRITE",
		sysmmu_fault_name[intr_type], fault_addr, &pgtable, info & 0xFFFF);

	if (pt)
		*pt = pgtable;
}

static void sysmmu_show_fault_information(struct sysmmu_drvdata *drvdata,
					  int intr_type, unsigned long fault_addr)
{
	phys_addr_t pgtable;

	pr_crit("----------------------------------------------------------\n");
	sysmmu_show_fault_info_simple(drvdata, intr_type, fault_addr, &pgtable);

	if (intr_type == SYSMMU_FAULT_UNKNOWN) {
		pr_crit("The fault is not caused by this System MMU.\n");
		pr_crit("Please check IRQ and SFR base address.\n");
		goto finish;
	}

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
				      int *intr_type, unsigned long *addr,
				      bool is_secure)
{
	*intr_type =  __ffs(__sysmmu_get_intr_status(data, is_secure));
	*intr_type %= 4;
	*addr = __sysmmu_get_fault_address(data, is_secure);
}

static void sysmmu_clear_interrupt(struct sysmmu_drvdata *data)
{
	u32 val = __sysmmu_get_intr_status(data, false);

	writel(val, data->sfrbase + REG_MMU_INT_CLEAR);
}

irqreturn_t samsung_sysmmu_irq(int irq, void *dev_id)
{
	int itype;
	unsigned long addr;
	struct sysmmu_drvdata *drvdata = dev_id;
	bool is_secure = (irq == drvdata->secure_irq);

	dev_info(drvdata->dev, "[%s] interrupt (%d) happened\n",
		 is_secure ? "Secure" : "Non-secure", irq);

	if (drvdata->async_fault_mode)
		return IRQ_WAKE_THREAD;

	sysmmu_get_interrupt_info(drvdata, &itype, &addr, is_secure);
	if (is_secure)
		sysmmu_show_secure_fault_information(drvdata, itype, addr);
	else
		sysmmu_show_fault_information(drvdata, itype, addr);

	return IRQ_WAKE_THREAD;
}

static int samsung_sysmmu_fault_notifier(struct device *dev, void *data)
{
	struct samsung_sysmmu_fault_info *fi;
	struct sysmmu_clientdata *client;
	struct sysmmu_drvdata *drvdata;
	int i, ret, result = -EFAULT;

	fi = (struct samsung_sysmmu_fault_info *)data;
	drvdata = fi->drvdata;

	client = (struct sysmmu_clientdata *) dev_iommu_priv_get(dev);

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
	int itype, ret;
	unsigned long addr;
	struct sysmmu_drvdata *drvdata = dev_id;
	bool is_secure = (irq == drvdata->secure_irq);
	struct iommu_group *group = drvdata->group;
	enum iommu_fault_reason reason;
	struct samsung_sysmmu_fault_info fi = {
		.drvdata = drvdata,
		.event.fault.type = IOMMU_FAULT_DMA_UNRECOV,
	};

	sysmmu_get_interrupt_info(drvdata, &itype, &addr, is_secure);
	reason = sysmmu_fault_type[itype];

	fi.event.fault.event.addr = addr;
	fi.event.fault.event.reason = reason;
	if (reason == IOMMU_FAULT_REASON_PTE_FETCH ||
	    reason == IOMMU_FAULT_REASON_PERMISSION)
		fi.event.fault.type = IOMMU_FAULT_PAGE_REQ;

	ret = iommu_group_for_each_dev(group, &fi,
				       samsung_sysmmu_fault_notifier);
	if (ret == -EAGAIN && !is_secure) {
		sysmmu_show_fault_info_simple(drvdata, itype, addr, NULL);
		sysmmu_clear_interrupt(drvdata);
		return IRQ_HANDLED;
	}

	if (drvdata->async_fault_mode) {
		if (is_secure)
			sysmmu_show_secure_fault_information(drvdata, itype, addr);
		else
			sysmmu_show_fault_information(drvdata, itype, addr);
	}
	panic("Unrecoverable System MMU Fault!!");

	return IRQ_HANDLED;
}
