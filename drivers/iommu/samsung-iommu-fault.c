// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 */

#define pr_fmt(fmt) "sysmmu: " fmt

#include <linux/smc.h>
#include <linux/arm-smccc.h>
#include <linux/pm_runtime.h>

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
#define MMU_VID_FROM_TLB(reg)		(((reg) >> 20) & 0x7U)
#define MMU_PADDR_FROM_TLB(reg)		((phys_addr_t)((reg) & 0xFFFFFF) << 12)
#define MMU_VADDR_FROM_SBB(reg)		(((reg) & 0xFFFFF) << 12)
#define MMU_VID_FROM_SBB(reg)		(((reg) >> 20) & 0x7U)
#define MMU_PADDR_FROM_SBB(reg)		((phys_addr_t)((reg) & 0x3FFFFFF) << 10)

#define REG_MMU_INT_STATUS		0x060
#define REG_MMU_INT_CLEAR		0x064
#define REG_MMU_FAULT_RW_MASK		GENMASK(20, 20)
#define IS_READ_FAULT(x)		(((x) & REG_MMU_FAULT_RW_MASK) == 0)

#define SYSMMU_FAULT_PTW_ACCESS   0
#define SYSMMU_FAULT_PAGE_FAULT   1
#define SYSMMU_FAULT_ACCESS       2
#define SYSMMU_FAULT_RESERVED     3
#define SYSMMU_FAULT_UNKNOWN      4

#define SYSMMU_SEC_FAULT_MASK		(BIT(SYSMMU_FAULT_PTW_ACCESS) | \
					 BIT(SYSMMU_FAULT_PAGE_FAULT) | \
					 BIT(SYSMMU_FAULT_ACCESS))

#define SYSMMU_FAULTS_NUM         (SYSMMU_FAULT_UNKNOWN + 1)

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
	"RESERVED",
	"UNKNOWN FAULT"
};

static unsigned int sysmmu_fault_type[SYSMMU_FAULTS_NUM] = {
	IOMMU_FAULT_REASON_WALK_EABT,
	IOMMU_FAULT_REASON_PTE_FETCH,
	IOMMU_FAULT_REASON_ACCESS,
	IOMMU_FAULT_REASON_UNKNOWN,
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

static inline sysmmu_iova_t __sysmmu_get_fault_address(struct sysmmu_drvdata *data,
						       unsigned int vid, bool is_secure)
{
	if (is_secure)
		return read_sec_info(MMU_SEC_VM_REG(data, IDX_FAULT_VA, vid));
	else
		return readl_relaxed(MMU_VM_REG(data, IDX_FAULT_VA, vid));
}

static inline void sysmmu_tlb_compare(phys_addr_t pgtable[MAX_VIDS],
				      unsigned int idx_sub, u32 vpn, u32 ppn, u32 attr)
{
	sysmmu_pte_t *entry;
	sysmmu_iova_t vaddr = MMU_VADDR_FROM_TLB(vpn, idx_sub);
	unsigned int vid = MMU_VID_FROM_TLB(attr);
	phys_addr_t paddr = MMU_PADDR_FROM_TLB(ppn);
	phys_addr_t phys = 0;

	if (!pgtable[vid])
		return;

	entry = section_entry(phys_to_virt(pgtable[vid]), vaddr);

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
		pr_crit("   TLB: %pa, PT entry: %pa\n", &paddr, &phys);
	}
}

static inline void sysmmu_sbb_compare(u32 sbb_vpn, u32 sbb_link, u32 sbbattr,
				      phys_addr_t pgtable[MAX_VIDS])
{
	sysmmu_pte_t *entry;
	sysmmu_iova_t vaddr = MMU_VADDR_FROM_SBB(sbb_vpn);
	unsigned int vid = MMU_VID_FROM_SBB(sbbattr);
	phys_addr_t paddr = MMU_PADDR_FROM_SBB(sbb_link);
	phys_addr_t phys = 0;

	if (!pgtable[vid])
		return;

	entry = section_entry(phys_to_virt(pgtable[vid]), vaddr);

	if (lv1ent_page(entry)) {
		phys = lv2table_base(entry);

		if (paddr != phys) {
			pr_crit(">> SBB mismatch detected!\n");
			pr_crit("   entry addr: %pa / SBB addr %pa\n",
				&paddr, &phys);
		}
	} else {
		pr_crit(">> Invalid address detected! entry: %#lx",
			(unsigned long)*entry);
	}
}

static inline
unsigned int dump_tlb_entry_port_type(struct sysmmu_drvdata *drvdata, phys_addr_t pgtable[MAX_VIDS],
				      unsigned int idx_way, unsigned int idx_set,
				      unsigned int idx_sub)
{
	u32 attr = readl_relaxed(MMU_REG(drvdata, IDX_TLB_ATTR));

	if (MMU_TLB_ENTRY_VALID(attr)) {
		u32 vpn, ppn;

		vpn = readl_relaxed(MMU_REG(drvdata, IDX_TLB_VPN)) + idx_sub;
		ppn = readl_relaxed(MMU_REG(drvdata, IDX_TLB_PPN));

		pr_crit("[%02u][%02u] VPN: %#010x, PPN: %#010x, ATTR: %#010x\n",
			idx_way, idx_set, vpn, ppn, attr);
		sysmmu_tlb_compare(pgtable, idx_sub, vpn, ppn, attr);

		return 1;
	}

	return 0;
}

#define MMU_NUM_TLB_SUBLINE		4
static unsigned int dump_tlb_entry_port(struct sysmmu_drvdata *drvdata,
					phys_addr_t pgtable[MAX_VIDS],
					unsigned int tlb, unsigned int way, unsigned int num_set)
{
	unsigned int cnt = 0;
	unsigned int set, line, val;

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
					  phys_addr_t pgtable[MAX_VIDS])
{
	unsigned int t, i;
	u32 capa0, capa1, info;
	unsigned int cnt;
	unsigned int num_tlb, num_port, num_sbb;
	void __iomem *sfrbase = drvdata->sfrbase;

	capa0 = readl_relaxed(sfrbase + REG_MMU_CAPA0_V7);
	capa1 = readl_relaxed(sfrbase + REG_MMU_CAPA1_V7);

	num_tlb = MMU_CAPA1_NUM_TLB(capa1);
	num_port = MMU_CAPA1_NUM_PORT(capa1);
	num_sbb = 1 << MMU_CAPA_NUM_SBB_ENTRY(capa0);

	pr_crit("SysMMU has %u TLBs, %u ports, %u sbb entries\n",
		num_tlb, num_port, num_sbb);

	for (t = 0; t < num_tlb; t++) {
		unsigned int num_set, num_way;

		info = readl_relaxed(sfrbase + MMU_TLB_INFO(t));
		num_way = MMU_CAPA1_NUM_TLB_WAY(info);
		num_set = MMU_CAPA1_NUM_TLB_SET(info);

		pr_crit("TLB.%u has %u way, %u set.\n", t, num_way, num_set);
		pr_crit("------------- TLB[WAY][SET][ENTRY] -------------\n");
		for (i = 0, cnt = 0; i < num_way; i++)
			cnt += dump_tlb_entry_port(drvdata, pgtable,
						   t, i, num_set);
	}
	if (!cnt)
		pr_crit(">> No Valid TLB Entries\n");

	pr_crit("--- SBB(Second-Level Page Table Base Address Buffer) ---\n");
	for (i = 0, cnt = 0; i < num_sbb; i++) {
		u32 sbb_vpn, sbblink, sbbattr;

		writel_relaxed(i, MMU_REG(drvdata, IDX_SBB_READ));
		sbb_vpn = readl_relaxed(MMU_REG(drvdata, IDX_SBB_VPN));

		if (MMU_SBB_ENTRY_VALID(sbb_vpn)) {
			sbblink = readl_relaxed(MMU_REG(drvdata, IDX_SBB_LINK));
			sbbattr = readl_relaxed(MMU_REG(drvdata, IDX_SBB_ATTR));

			pr_crit("[%02d] VPN: %#010x, PPN: %#010x, ATTR: %#010x",
				i, sbb_vpn, sbblink, sbbattr);
			sysmmu_sbb_compare(sbb_vpn, sbblink, sbbattr, pgtable);
			cnt++;
		}
	}
	if (!cnt)
		pr_crit(">> No Valid SBB Entries\n");
}

static inline void dump_sysmmu_status(struct sysmmu_drvdata *drvdata,
				      phys_addr_t pgtable[MAX_VIDS], unsigned int vid)
{
	int info;
	void __iomem *sfrbase = drvdata->sfrbase;

	info = MMU_RAW_VER(readl_relaxed(sfrbase + REG_MMU_VERSION));

	pr_crit("ADDR: (VA: %p), MMU_CTRL: %#010x, PT_BASE: %#010x\n",
		sfrbase,
		readl_relaxed(sfrbase + REG_MMU_CTRL),
		readl_relaxed(MMU_VM_REG(drvdata, IDX_FLPT_BASE, vid)));
	pr_crit("VERSION %d.%d.%d, MMU_CFG: %#010x, MMU_STATUS: %#010x\n",
		MMU_MAJ_VER(info), MMU_MIN_VER(info), MMU_REV_VER(info),
		readl_relaxed(sfrbase + REG_MMU_CFG),
		readl_relaxed(sfrbase + REG_MMU_STATUS));

	if (drvdata->has_vcr)
		pr_crit("MMU_CTRL_VM: %#010x, MMU_CFG_VM: %#010x\n",
			readl_relaxed(MMU_VM_REG(drvdata, IDX_CTRL_VM, vid)),
			readl_relaxed(MMU_VM_REG(drvdata, IDX_CFG_VM, vid)));

	dump_sysmmu_tlb_status(drvdata, pgtable);
}

static void sysmmu_get_fault_msg(struct sysmmu_drvdata *drvdata, unsigned int intr_type,
				 unsigned int vid, sysmmu_iova_t fault_addr,
				 bool is_secure, char *fault_msg, size_t fault_msg_sz)
{
	const char *port_name = NULL;
	unsigned int info;

	of_property_read_string(drvdata->dev->of_node, "port-name", &port_name);

	if (is_secure) {
		info = read_sec_info(MMU_SEC_REG(drvdata, IDX_FAULT_TRANS_INFO));
		scnprintf(fault_msg, fault_msg_sz,
			  "SysMMU %s %s from %s (secure) at %#010x",
			  IS_READ_FAULT(info) ? "READ" : "WRITE",
			  sysmmu_fault_name[intr_type],
			  port_name ? port_name : dev_name(drvdata->dev),
			  fault_addr);
	} else {
		info = readl_relaxed(MMU_VM_REG(drvdata, IDX_FAULT_TRANS_INFO, vid));
		scnprintf(fault_msg, fault_msg_sz,
			  "SysMMU %s %s from %s VID %u at %#010x",
			  IS_READ_FAULT(info) ? "READ" : "WRITE",
			  sysmmu_fault_name[intr_type],
			  port_name ? port_name : dev_name(drvdata->dev), vid,
			  fault_addr);
	}
}

static void sysmmu_show_secure_fault_information(struct sysmmu_drvdata *drvdata,
						 unsigned int intr_type, sysmmu_iova_t fault_addr)
{
	unsigned int info;
	phys_addr_t pgtable;
	unsigned int sfrbase = drvdata->secure_base;
	char err_msg[128];

	pgtable = read_sec_info(MMU_SEC_REG(drvdata, IDX_SEC_FLPT_BASE));
	pgtable <<= PAGE_SHIFT;

	info = read_sec_info(MMU_SEC_REG(drvdata, IDX_FAULT_TRANS_INFO));

	pr_crit("----------------------------------------------------------\n");

	sysmmu_get_fault_msg(drvdata, intr_type, 0, fault_addr,
			     true, err_msg, sizeof(err_msg));

	pr_crit("%s (pgtable @ %pa)\n", err_msg, &pgtable);

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
					  unsigned int intr_type, unsigned int vid,
					  sysmmu_iova_t fault_addr, phys_addr_t *pt)
{
	u32 info;
	char err_msg[128];

	info = readl_relaxed(MMU_VM_REG(drvdata, IDX_FAULT_TRANS_INFO, vid));

	sysmmu_get_fault_msg(drvdata, intr_type, vid, fault_addr,
			     false, err_msg, sizeof(err_msg));

	pr_crit("%s (pgtable @ %pa, AxID: %#x)\n", err_msg, pt, info & 0xFFFF);
}

static void sysmmu_show_fault_information(struct sysmmu_drvdata *drvdata,
					  unsigned int intr_type, unsigned int vid,
					  sysmmu_iova_t fault_addr)
{
	unsigned int i;
	phys_addr_t pgtable[MAX_VIDS];

	for (i = 0; i < __max_vids(drvdata); i++) {
		pgtable[i] = readl_relaxed(MMU_VM_REG(drvdata, IDX_FLPT_BASE, i));
		pgtable[i] <<= PAGE_SHIFT;
	}

	pr_crit("----------------------------------------------------------\n");
	sysmmu_show_fault_info_simple(drvdata, intr_type, vid, fault_addr, &pgtable[vid]);

	if (intr_type == SYSMMU_FAULT_UNKNOWN) {
		pr_crit("The fault is not caused by this System MMU.\n");
		pr_crit("Please check IRQ and SFR base address.\n");
		goto finish;
	}

	for (i = 0; i < __max_vids(drvdata); i++) {
		if (pgtable[i] != drvdata->pgtable[i])
			pr_crit("Page table (VID %u) base of driver: %pa\n", i,
				&drvdata->pgtable[i]);
		if (pgtable[i] && !pfn_valid(pgtable[i] >> PAGE_SHIFT)) {
			pr_crit("Page table (VID %u) base is not in a valid memory region\n", i);
			pgtable[i] = 0;
		}
	}

	if (pgtable[vid]) {
		sysmmu_pte_t *ent;

		ent = section_entry(phys_to_virt(pgtable[vid]), fault_addr);
		pr_crit("Lv1 entry: %#010x\n", *ent);

		if (lv1ent_page(ent)) {
			ent = page_entry(ent, fault_addr);
			pr_crit("Lv2 entry: %#010x\n", *ent);
		}
	}

	if (intr_type == SYSMMU_FAULT_PTW_ACCESS) {
		pr_crit("System MMU has failed to access page table\n");
		pgtable[vid] = 0;
	}

	dump_sysmmu_status(drvdata, pgtable, vid);
finish:
	pr_crit("----------------------------------------------------------\n");
}

static void sysmmu_get_interrupt_info(struct sysmmu_drvdata *data, unsigned int *intr_type,
				      unsigned int *vid, sysmmu_iova_t *addr, bool is_secure)
{
	u32 istatus;

	istatus = (unsigned int)__ffs(__sysmmu_get_intr_status(data, is_secure));
	*vid = istatus / 4;
	*intr_type = istatus % 4;
	*addr = __sysmmu_get_fault_address(data, *vid, is_secure);
}

static int sysmmu_clear_interrupt(struct sysmmu_drvdata *data, bool is_secure)
{
	u32 val = __sysmmu_get_intr_status(data, is_secure);

	if (is_secure) {
		if (val & ~SYSMMU_SEC_FAULT_MASK) {
			dev_warn(data->dev, "Unknown secure fault (%x)\n", val);
			val &= SYSMMU_SEC_FAULT_MASK;
		}
		return clear_sec_fault(data->secure_base, val);
	}
	writel(val, data->sfrbase + REG_MMU_INT_CLEAR);
	return 0;
}

irqreturn_t samsung_sysmmu_irq(int irq, void *dev_id)
{
	unsigned int itype;
	unsigned int vid;
	sysmmu_iova_t addr;
	struct sysmmu_drvdata *drvdata = dev_id;
	bool is_secure = (irq == drvdata->secure_irq);

	if (drvdata->hide_page_fault)
		return IRQ_WAKE_THREAD;

	dev_info(drvdata->dev, "[%s] interrupt (%d) happened\n",
		 is_secure ? "Secure" : "Non-secure", irq);

	if (drvdata->async_fault_mode)
		return IRQ_WAKE_THREAD;

	sysmmu_get_interrupt_info(drvdata, &itype, &vid, &addr, is_secure);
	if (is_secure)
		sysmmu_show_secure_fault_information(drvdata, itype, addr);
	else
		sysmmu_show_fault_information(drvdata, itype, vid, addr);

	return IRQ_WAKE_THREAD;
}

static int samsung_sysmmu_fault_notifier(struct device *dev, void *data)
{
	struct samsung_sysmmu_fault_info *fi;
	struct sysmmu_clientdata *client;
	struct sysmmu_drvdata *drvdata;
	unsigned int i;
	int ret, result = 0;

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
	unsigned int itype, vid;
	int ret;
	sysmmu_iova_t addr;
	struct sysmmu_drvdata *drvdata = dev_id;
	bool is_secure = (irq == drvdata->secure_irq);
	struct iommu_group *group = drvdata->group;
	enum iommu_fault_reason reason;
	struct samsung_sysmmu_fault_info fi = {
		.drvdata = drvdata,
		.event.fault.type = IOMMU_FAULT_DMA_UNRECOV,
	};
	char fault_msg[128] = "Unspecified SysMMU fault";

	/* Prevent power down while handling faults */
	pm_runtime_get(drvdata->dev);

	sysmmu_get_interrupt_info(drvdata, &itype, &vid, &addr, is_secure);
	reason = sysmmu_fault_type[itype];

	fi.event.fault.event.addr = addr;
	fi.event.fault.event.pasid = vid;
	if (vid)
		fi.event.fault.event.flags |= IOMMU_FAULT_UNRECOV_PASID_VALID;
	fi.event.fault.event.reason = reason;
	if (reason == IOMMU_FAULT_REASON_PTE_FETCH ||
	    reason == IOMMU_FAULT_REASON_PERMISSION)
		fi.event.fault.type = IOMMU_FAULT_PAGE_REQ;

	ret = iommu_group_for_each_dev(group, &fi,
				       samsung_sysmmu_fault_notifier);
	if (ret == -EAGAIN) {
		if (is_secure) {
			if (drvdata->async_fault_mode && !drvdata->hide_page_fault)
				sysmmu_show_secure_fault_information(drvdata, itype, addr);
			ret = sysmmu_clear_interrupt(drvdata, true);
			if (ret) {
				if (drvdata->hide_page_fault)
					sysmmu_show_secure_fault_information(drvdata, itype, addr);
				dev_err(drvdata->dev, "Failed to clear secure fault (%d)\n", ret);
				goto out;
			}
		} else  {
			phys_addr_t pgtable;

			pgtable = readl_relaxed(MMU_VM_REG(drvdata, IDX_FLPT_BASE, vid));
			pgtable <<= PAGE_SHIFT;
			if (!drvdata->hide_page_fault)
				sysmmu_show_fault_info_simple(drvdata, itype, vid, addr, &pgtable);
			sysmmu_clear_interrupt(drvdata, false);
		}
		pm_runtime_put(drvdata->dev);
		return IRQ_HANDLED;
	}

	if (drvdata->async_fault_mode || drvdata->hide_page_fault) {
		if (is_secure)
			sysmmu_show_secure_fault_information(drvdata, itype, addr);
		else
			sysmmu_show_fault_information(drvdata, itype, vid, addr);
	}

out:
	sysmmu_get_fault_msg(drvdata, itype, vid, addr, is_secure, fault_msg, sizeof(fault_msg));

	pm_runtime_put(drvdata->dev);

	panic(fault_msg);

	return IRQ_HANDLED;
}
