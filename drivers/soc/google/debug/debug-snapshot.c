// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 */

#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/kallsyms.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/module.h>
#include <linux/of_reserved_mem.h>
#include <linux/sched/clock.h>
#include <linux/of.h>
#include <linux/arm-smccc.h>
#include <linux/smc.h>
#include <linux/regmap.h>

#include <linux/soc/samsung/exynos-smc.h>
#include "debug-snapshot-local.h"

#define DSS_VERSION	0x80000000

struct dbg_snapshot_interface {
	struct dbg_snapshot_log *info_event;
};
struct dbg_snapshot_param dss_param;
struct dbg_snapshot_item dss_items[] = {
	[DSS_ITEM_HEADER_ID]	= {DSS_ITEM_HEADER,	{0, 0, 0, true}, true, NULL, NULL},
	[DSS_ITEM_KEVENTS_ID]	= {DSS_ITEM_KEVENTS,	{0, 0, 0, false}, false, NULL, NULL},
	[DSS_ITEM_BCM_ID]	= {DSS_ITEM_BCM,	{0, 0, 0, true}, false, NULL, NULL},
	[DSS_ITEM_S2D_ID]	= {DSS_ITEM_S2D,	{0, 0, 0, true}, false, NULL, NULL},
	[DSS_ITEM_ARRDUMP_RESET_ID] = {DSS_ITEM_ARRDUMP_RESET, {0, 0, 0, false}, false, NULL, NULL},
	[DSS_ITEM_ARRDUMP_PANIC_ID] = {DSS_ITEM_ARRDUMP_PANIC, {0, 0, 0, false}, false, NULL, NULL},
	[DSS_ITEM_SLCDUMP_ID]	= {DSS_ITEM_SLCDUMP,	{0, 0, 0, false}, false, NULL ,NULL},
	[DSS_ITEM_PRE_SLCDUMP_ID]   = {DSS_ITEM_PRE_SLCDUMP, {0, 0, 0, false}, false, NULL ,NULL},
	[DSS_ITEM_ITMON_ID]     = {DSS_ITEM_ITMON,      {0, 0, 0, true}, false, NULL, NULL},
};

/*  External interface variable for trace debugging */
static struct dbg_snapshot_interface dss_info __attribute__ ((used));
static struct dbg_snapshot_interface *ess_info __attribute__ ((used));

struct dbg_snapshot_base *dss_base;
struct dbg_snapshot_base *ess_base;
struct dbg_snapshot_log *dss_log;
struct dbg_snapshot_desc dss_desc;
struct itmon_logs *dss_itmon;

void __iomem *dbg_snapshot_get_header_vaddr(void)
{
	if (dbg_snapshot_get_item_enable(DSS_ITEM_HEADER))
		return (void __iomem *)(dss_items[DSS_ITEM_HEADER_ID].entry.vaddr);
	return NULL;
}
EXPORT_SYMBOL_GPL(dbg_snapshot_get_header_vaddr);

unsigned long dbg_snapshot_get_header_paddr(void)
{
	if (dbg_snapshot_get_item_enable(DSS_ITEM_HEADER))
		return (unsigned long)dss_items[DSS_ITEM_HEADER_ID].entry.paddr;
	return 0;
}
EXPORT_SYMBOL_GPL(dbg_snapshot_get_header_paddr);

unsigned int dbg_snapshot_get_val_offset(unsigned int offset)
{
	if (dbg_snapshot_get_item_enable(DSS_ITEM_HEADER))
		return __raw_readl(dbg_snapshot_get_header_vaddr() + offset);
	return 0;
}
EXPORT_SYMBOL_GPL(dbg_snapshot_get_val_offset);

void dbg_snapshot_set_val_offset(unsigned int val, unsigned int offset)
{
	if (dbg_snapshot_get_item_enable(DSS_ITEM_HEADER))
		__raw_writel(val, dbg_snapshot_get_header_vaddr() + offset);
}
EXPORT_SYMBOL_GPL(dbg_snapshot_set_val_offset);

static void dbg_snapshot_set_sjtag_status(void)
{
#ifdef SMC_CMD_GET_SJTAG_STATUS
	struct arm_smccc_res res;

	arm_smccc_smc(SMC_CMD_GET_SJTAG_STATUS, 0x3, 0, 0, 0, 0, 0, 0, &res);
	dss_desc.sjtag_status = res.a0;
	dev_info(dss_desc.dev, "SJTAG is %sabled\n",
			dss_desc.sjtag_status ? "en" : "dis");
#endif
}

int dbg_snapshot_get_sjtag_status(void)
{
	return dss_desc.sjtag_status;
}
EXPORT_SYMBOL_GPL(dbg_snapshot_get_sjtag_status);

bool dbg_snapshot_get_reboot_status(void)
{
	return dss_desc.in_reboot;
}
EXPORT_SYMBOL_GPL(dbg_snapshot_get_reboot_status);

bool dbg_snapshot_get_panic_status(void)
{
	return dss_desc.in_panic;
}
EXPORT_SYMBOL_GPL(dbg_snapshot_get_panic_status);

bool dbg_snapshot_get_warm_status(void)
{
	return dss_desc.in_warm;
}
EXPORT_SYMBOL_GPL(dbg_snapshot_get_warm_status);

void dbg_snapshot_scratch_reg(unsigned int val)
{
	dbg_snapshot_set_val_offset(val, DSS_OFFSET_SCRATCH);
}
EXPORT_SYMBOL_GPL(dbg_snapshot_scratch_reg);

void dbg_snapshot_scratch_clear(void)
{
	dbg_snapshot_scratch_reg(DSS_SIGN_RESET);
}
EXPORT_SYMBOL_GPL(dbg_snapshot_scratch_clear);

bool dbg_snapshot_is_scratch(void)
{
	return dbg_snapshot_get_val_offset(DSS_OFFSET_SCRATCH) == DSS_SIGN_SCRATCH;
}
EXPORT_SYMBOL_GPL(dbg_snapshot_is_scratch);

void dbg_snapshot_set_debug_test_buffer_addr(u64 paddr, unsigned int cpu)
{
	void __iomem *header = dbg_snapshot_get_header_vaddr();

	if (header)
		__raw_writeq(paddr, header + DSS_OFFSET_DEBUG_TEST_BUFFER(cpu));
}
EXPORT_SYMBOL_GPL(dbg_snapshot_set_debug_test_buffer_addr);

unsigned int dbg_snapshot_get_debug_test_buffer_addr(unsigned int cpu)
{
	void __iomem *header = dbg_snapshot_get_header_vaddr();

	return header ? __raw_readq(header + DSS_OFFSET_DEBUG_TEST_BUFFER(cpu)) : 0;
}
EXPORT_SYMBOL_GPL(dbg_snapshot_get_debug_test_buffer_addr);

unsigned long dbg_snapshot_get_last_pc_paddr(void)
{
	/* We want to save the pc value to NC region only if DSS is enabled. */
	unsigned long header = dbg_snapshot_get_header_paddr();

	return header ? header + DSS_OFFSET_CORE_LAST_PC : 0;
}
EXPORT_SYMBOL_GPL(dbg_snapshot_get_last_pc_paddr);

unsigned long dbg_snapshot_get_last_pc(unsigned int cpu)
{
	void __iomem *header = dbg_snapshot_get_header_vaddr();

	if (header)
		return __raw_readq(header + DSS_OFFSET_CORE_LAST_PC + cpu * 8);
	return 0;
}
EXPORT_SYMBOL_GPL(dbg_snapshot_get_last_pc);

unsigned int dbg_snapshot_get_hardlockup_magic(int cpu)
{
	void __iomem *header = dbg_snapshot_get_header_vaddr();

	if (header)
		return __raw_readl(header + DSS_OFFSET_CORE_LAST_PC +
			(DSS_NR_CPUS * sizeof(unsigned long)) +
			(cpu * sizeof(unsigned long)));
	return 0;
}
EXPORT_SYMBOL_GPL(dbg_snapshot_get_hardlockup_magic);

int dbg_snapshot_get_dpm_none_dump_mode(void)
{
	unsigned int val;
	void __iomem *header = dbg_snapshot_get_header_vaddr();

	if (!header)
		return -1;

	val = __raw_readl(header + DSS_OFFSET_NONE_DPM_DUMP_MODE);
	if ((val & GENMASK(31, 16)) == DSS_SIGN_MAGIC)
		return (val & GENMASK(15, 0));

	return -1;
}
EXPORT_SYMBOL_GPL(dbg_snapshot_get_dpm_none_dump_mode);

void dbg_snapshot_set_dpm_none_dump_mode(unsigned int mode)
{
	void __iomem *header = dbg_snapshot_get_header_vaddr();

	if (!header)
		return;

	if (mode)
		mode |= DSS_SIGN_MAGIC;
	else
		mode = 0;

	__raw_writel(mode, header + DSS_OFFSET_NONE_DPM_DUMP_MODE);
}
EXPORT_SYMBOL_GPL(dbg_snapshot_set_dpm_none_dump_mode);

void dbg_snapshot_set_qd_entry(unsigned long address)
{
	void __iomem *header = dbg_snapshot_get_header_vaddr();

	if (header)
		__raw_writeq((unsigned long)(virt_to_phys)((void *)address),
				header + DSS_OFFSET_QD_ENTRY);
}
EXPORT_SYMBOL_GPL(dbg_snapshot_set_qd_entry);

static struct dbg_snapshot_item *dbg_snapshot_get_item(const char *name)
{
	unsigned long i;

	for (i = 0; i < ARRAY_SIZE(dss_items); i++) {
		if (dss_items[i].name && !strcmp(name, dss_items[i].name))
			return &dss_items[i];
	}

	return NULL;
}

unsigned int dbg_snapshot_get_item_size(const char *name)
{
	struct dbg_snapshot_item *item = dbg_snapshot_get_item(name);

	return item && item->entry.enabled ? item->entry.size : 0;
}
EXPORT_SYMBOL_GPL(dbg_snapshot_get_item_size);

unsigned long dbg_snapshot_get_item_vaddr(const char *name)
{
	struct dbg_snapshot_item *item = dbg_snapshot_get_item(name);

	return item && item->entry.enabled ? item->entry.vaddr : 0;
}
EXPORT_SYMBOL_GPL(dbg_snapshot_get_item_vaddr);

unsigned int dbg_snapshot_get_item_paddr(const char *name)
{
	struct dbg_snapshot_item *item = dbg_snapshot_get_item(name);

	return item && item->entry.enabled ? item->entry.paddr : 0;
}
EXPORT_SYMBOL_GPL(dbg_snapshot_get_item_paddr);

int dbg_snapshot_get_item_enable(const char *name)
{
	struct dbg_snapshot_item *item = dbg_snapshot_get_item(name);

	return item && item->entry.enabled ? item->entry.enabled : 0;
}
EXPORT_SYMBOL_GPL(dbg_snapshot_get_item_enable);

void dbg_snapshot_set_item_enable(const char *name, int en)
{
	struct dbg_snapshot_item *item = NULL;

	if (!name || !dss_dpm.enabled_debug || dss_dpm.dump_mode == NONE_DUMP)
		return;

	/* This is default for debug-mode */
	item = dbg_snapshot_get_item(name);
	if (item) {
		item->entry.enabled = en;
		pr_info("item - %s is %sabled\n", name, en ? "en" : "dis");
	}
}
EXPORT_SYMBOL_GPL(dbg_snapshot_set_item_enable);

static void dbg_snapshot_set_enable(int en)
{
	dss_base->enabled = en;
	dev_info(dss_desc.dev, "%sabled\n", en ? "en" : "dis");
}

int dbg_snapshot_get_enable(void)
{
	return dss_base->enabled;
}
EXPORT_SYMBOL_GPL(dbg_snapshot_get_enable);

struct dbg_snapshot_item *dbg_snapshot_get_item_by_index(int index)
{
	if (index < 0 || index > ARRAY_SIZE(dss_items))
		return NULL;

	return &dss_items[index];
}
EXPORT_SYMBOL_GPL(dbg_snapshot_get_item_by_index);

int dbg_snapshot_get_num_items(void)
{
	return ARRAY_SIZE(dss_items);
}
EXPORT_SYMBOL_GPL(dbg_snapshot_get_num_items);

void dbg_snapshot_output(void)
{
	unsigned long i, size = 0;

	dev_info(dss_desc.dev, "debug-snapshot physical / virtual memory layout:\n");
	for (i = 0; i < ARRAY_SIZE(dss_items); i++) {
		if (!dss_items[i].entry.enabled)
			continue;
		pr_info("%-16s: phys:%pa / virt:%pK / size:0x%zx / en:%d\n",
				dss_items[i].name,
				&dss_items[i].entry.paddr,
				(void *) dss_items[i].entry.vaddr,
				dss_items[i].entry.size,
				dss_items[i].entry.enabled);
		size += dss_items[i].entry.size;
	}

	dev_info(dss_desc.dev, "total_item_size: %ldKB, dbg_snapshot_log struct size: %zdKB\n",
			size / SZ_1K, sizeof(struct dbg_snapshot_log) / SZ_1K);
}
EXPORT_SYMBOL_GPL(dbg_snapshot_output);

unsigned int dbg_snapshot_get_slcdump_base(void)
{
	if (!dbg_snapshot_get_enable())
		return 0;

	return __raw_readl(dbg_snapshot_get_header_vaddr() + DSS_OFFSET_SLCDUMP_BASE_REG);
}
EXPORT_SYMBOL_GPL(dbg_snapshot_get_slcdump_base);

unsigned int dbg_snapshot_get_pre_slcdump_base(void)
{
	if (!dbg_snapshot_get_enable())
		return 0;

	return __raw_readl(dbg_snapshot_get_header_vaddr() + DSS_OFFSET_PRE_SLCDUMP_BASE_REG);
}
EXPORT_SYMBOL_GPL(dbg_snapshot_get_pre_slcdump_base);

unsigned int dbg_snapshot_get_max_core_num(void)
{
	if (nr_cpu_ids > DSS_NR_CPUS)
		pr_err("unexpected nr_cpu_ids(%u) or DSS_NR_CPUS (%u)\n", nr_cpu_ids, DSS_NR_CPUS);

	return min(nr_cpu_ids, (unsigned int)DSS_NR_CPUS);
}
EXPORT_SYMBOL_GPL(dbg_snapshot_get_max_core_num);

static void dbg_snapshot_init_desc(struct device *dev)
{
	/* initialize dss_desc */
	memset((void *)&dss_desc, 0, sizeof(struct dbg_snapshot_desc));
	raw_spin_lock_init(&dss_desc.ctrl_lock);
	dss_desc.dev = dev;
	dbg_snapshot_set_sjtag_status();

	if (of_property_read_u32(dev->of_node, "panic-action",
			&dss_desc.panic_action))
		dss_desc.panic_action = GO_DEFAULT_ID;
}

static void dbg_snapshot_fixmap(void)
{
	size_t vaddr, paddr, size;
	unsigned long i;

	for (i = 0; i < ARRAY_SIZE(dss_items); i++) {
		if (!dss_items[i].entry.enabled)
			continue;

		/*  assign dss_item information */
		paddr = dss_items[i].entry.paddr;
		vaddr = dss_items[i].entry.vaddr;
		size = dss_items[i].entry.size;

		if (i == DSS_ITEM_HEADER_ID) {
			/*  initialize kernel event to 0 except only header */
			memset((void *)(vaddr + DSS_HDR_INFO_BLOCK_KEEP_SZ), 0,
					size - DSS_HDR_INFO_BLOCK_KEEP_SZ);
		} else {
			/*  initialized log to 0 if persist == false */
			if (!dss_items[i].persist)
				memset((void *)vaddr, 0, size);
		}
	}

	if (dss_items[DSS_ITEM_KEVENTS_ID].entry.enabled)
		dss_log = (struct dbg_snapshot_log *)(dss_items[DSS_ITEM_KEVENTS_ID].entry.vaddr);
	if (dss_items[DSS_ITEM_ITMON_ID].entry.enabled) {
		dss_itmon = (struct itmon_logs *)(dss_items[DSS_ITEM_ITMON_ID].entry.vaddr);
		dss_itmon->magic = DSS_ITMON_MAGIC_INITIALIZED;
	}

	/*  set fake translation to virtual address to debug trace */
	dss_info.info_event = dss_log;
	ess_info = &dss_info;

	dss_param.dss_log_misc = &dss_log_misc;
	dss_param.dss_items = &dss_items;
	dss_param.dss_log_items = &dss_log_items;
	dss_param.dss_log = dss_log;

	dss_base->param = &dss_param;

	/* output the information of debug-snapshot */
	dbg_snapshot_output();
}

static int dbg_snapshot_rmem_setup(struct device *dev)
{
	struct reserved_mem *rmem;
	struct device_node *rmem_np;
	struct dbg_snapshot_item *item;
	bool en;
	unsigned long i, j;
	unsigned long flags = VM_MAP;
	pgprot_t prot = __pgprot(PROT_NORMAL_NC);
	int page_size, mem_count = 0;
	struct page *page;
	struct page **pages;
	void *vaddr;

	mem_count = of_count_phandle_with_args(dev->of_node, "memory-region", NULL);
	if (!mem_count) {
		dev_err(dev, "no such memory-region\n");
		return -ENOMEM;
	}

	for (i = 0; i < mem_count; i++) {
		rmem_np = of_parse_phandle(dev->of_node, "memory-region", i);
		if (!rmem_np) {
			dev_err(dev, "no such memory-region of index %ld\n", i);
			continue;
		}

		en = of_device_is_available(rmem_np);
		if (!en) {
			dev_err(dev, "%s item is disabled, Skip alloc reserved memory\n",
					rmem_np->name);
			continue;
		}

		rmem = of_reserved_mem_lookup(rmem_np);
		if (!rmem) {
			dev_err(dev, "no such reserved mem of node name %s\n", rmem_np->name);
			continue;
		}

		dbg_snapshot_set_item_enable(rmem->name, en);
		item = dbg_snapshot_get_item(rmem->name);
		if (!item) {
			dev_err(dev, "no such %s item in dss_items\n", rmem->name);
			continue;
		}

		if (!rmem->base || !rmem->size) {
			dev_err(dev, "%s item wrong base(%pap) or size(%pap)\n",
					item->name, &rmem->base, &rmem->size);
			item->entry.enabled = false;
			continue;
		}
		page_size = rmem->size / PAGE_SIZE;
		pages = kcalloc(page_size, sizeof(struct page *), GFP_KERNEL);
		page = phys_to_page(rmem->base);

		for (j = 0; j < page_size; j++)
			pages[j] = page++;

		vaddr = vmap(pages, page_size, flags, prot);
		kfree(pages);
		if (!vaddr) {
			dev_err(dev, "%s: paddr:%pap page_size:%pap failed to vmap\n",
					item->name, &rmem->base, &rmem->size);
			item->entry.enabled = false;
			continue;
		}

		item->entry.paddr = rmem->base;
		item->entry.size = rmem->size;
		item->entry.vaddr = (size_t)vaddr;
		item->head_ptr = (unsigned char *)vaddr;
		item->curr_ptr = (unsigned char *)vaddr;

		if (item == &dss_items[DSS_ITEM_HEADER_ID]) {
			dss_base = (struct dbg_snapshot_base *)dbg_snapshot_get_header_vaddr();
			dss_base->vaddr = (size_t)vaddr;
			dss_base->paddr = rmem->base;
			dss_base->version = DSS_VERSION;
			ess_base = dss_base;
			rmem->priv = vaddr;
		}
		if (!dss_base) {
			dev_err(dev, "Header memory region needs to be first!");
			/* This has to be the first and only mapped entry. */
			vunmap(vaddr);
			return -EINVAL;
		}

		dss_base->size += rmem->size;
	}

	if (!dss_base) {
		dev_err(dev, "No/Invalid header memory region found");
		return -EINVAL;
	}

	for (i = 0; i < ARRAY_SIZE(dss_items); i++) {
		item = &dss_items[i];
		if (!item->entry.paddr || !item->entry.size)
			item->entry.enabled = false;
	}

	return 0;
}

static ssize_t dss_dpm_none_dump_mode_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "currnet DPM dump_mode: %x, DPM none dump_mode: %x\n",
			dss_dpm.dump_mode, dss_dpm.dump_mode_none);
}

static ssize_t dss_dpm_none_dump_mode_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	unsigned int mode;

	if (kstrtouint(buf, 10, &mode))
		return count;

	dbg_snapshot_set_dpm_none_dump_mode(mode);
	if (mode && dss_dpm.version) {
		dss_dpm.dump_mode_none = 1;
		dbg_snapshot_scratch_clear();
	} else {
		dss_dpm.dump_mode_none = 0;
		dbg_snapshot_scratch_reg(DSS_SIGN_SCRATCH);
	}

	dev_info(dss_desc.dev, "DPM: success to switch %sNONE_DUMP mode\n",
			dss_dpm.dump_mode_none ? "" : "NOT ");
	return count;
}

DEVICE_ATTR_RW(dss_dpm_none_dump_mode);

static ssize_t in_warm_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%sable\n", dss_desc.in_warm ? "en" : "dis");
}

static ssize_t in_warm_store(struct device *dev, struct device_attribute *attr,
					const char *buf, size_t count)
{
	unsigned long val;
	int ret;

	ret = kstrtoul(buf, 10, &val);

	if (!ret)
		dss_desc.in_warm = !!val;

	return count;
}

DEVICE_ATTR_RW(in_warm);

static void dbg_snapshot_set_slcdump_status(void)
{
	int i;

	__raw_writel(0, dbg_snapshot_get_header_vaddr() + DSS_OFFSET_SLCDUMP_MAGIC);
	__raw_writel(0, dbg_snapshot_get_header_vaddr() + DSS_OFFSET_SLCDUMP_STATUS);
	__raw_writel(0, dbg_snapshot_get_header_vaddr() + DSS_OFFSET_PRE_SLCDUMP_BASE_REG);

	for (i = 0; i < ARRAY_SIZE(dss_items); i++) {
		if (dss_items[i].entry.enabled &&
				dss_items[i].entry.paddr &&
				dss_items[i].entry.size) {
			if (!strcmp(dss_items[i].name, "log_slcdump")) {
				__raw_writel(DSS_SLCDUMP_MAGIC,
						dbg_snapshot_get_header_vaddr() +
						DSS_OFFSET_SLCDUMP_MAGIC);
				__raw_writel(dss_items[i].entry.paddr,
						dbg_snapshot_get_header_vaddr() +
						DSS_OFFSET_SLCDUMP_BASE_REG);
			}
			if (!strcmp(dss_items[i].name, "log_preslcdump")) {
				__raw_writel(dss_items[i].entry.paddr,
						dbg_snapshot_get_header_vaddr() +
						DSS_OFFSET_PRE_SLCDUMP_BASE_REG);
			}
		}
	}
}


static struct attribute *dss_sysfs_attrs[] = {
	&dev_attr_dss_dpm_none_dump_mode.attr,
	&dev_attr_in_warm.attr,
	NULL,
};

static struct attribute_group dss_sysfs_group = {
	.attrs = dss_sysfs_attrs,
};

static const struct attribute_group *dss_sysfs_groups[] = {
	&dss_sysfs_group,
	NULL,
};

static int dbg_snapshot_probe(struct platform_device *pdev)
{
	if (dbg_snapshot_dt_scan_dpm()) {
		dev_err(&pdev->dev, "no such dpm node\n");
		return -ENODEV;
	}

	dbg_snapshot_init_desc(&pdev->dev);
	if (dbg_snapshot_rmem_setup(&pdev->dev)) {
		dev_err(&pdev->dev, "%s failed\n", __func__);
		return -ENODEV;
	}

	dbg_snapshot_fixmap();
	dbg_snapshot_init_log();

	dbg_snapshot_init_utils();
	dbg_snapshot_init_dpm();

	dbg_snapshot_register_vh_log();

	dbg_snapshot_set_slcdump_status();

	dbg_snapshot_set_enable(true);
	dbg_snapshot_start_log();

	if (sysfs_create_groups(&pdev->dev.kobj, dss_sysfs_groups))
		dev_err(dss_desc.dev, "fail to register debug-snapshot sysfs\n");

	dev_info(&pdev->dev, "%s successful.\n", __func__);
	return 0;
}

static const struct of_device_id dss_of_match[] = {
	{ .compatible	= "google,debug-snapshot" },
	{},
};
MODULE_DEVICE_TABLE(of, dss_of_match);

static struct platform_driver dbg_snapshot_driver = {
	.probe = dbg_snapshot_probe,
	.driver  = {
		.name  = "debug-snapshot",
		.of_match_table = of_match_ptr(dss_of_match),
	},
};
module_platform_driver(dbg_snapshot_driver);

MODULE_DESCRIPTION("Debug-Snapshot");
MODULE_LICENSE("GPL v2");
