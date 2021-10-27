// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 */

#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/platform_device.h>
#include <linux/kallsyms.h>
#include <linux/sched/clock.h>
#include <asm-generic/sections.h>

#include <soc/google/debug-snapshot.h>
#include "debug-snapshot-local.h"

#define DSS_QD_MAX_NUM_ENTRIES		(48)
#define DSS_QD_MAX_NAME_LENGTH		(16)
#define DSS_QD_ATTR_LOG			(0)
#define DSS_QD_ATTR_ARRAY		(1)
#define DSS_QD_ATTR_STRUCT		(2)
#define DSS_QD_ATTR_STACK		(3)
#define DSS_QD_ATTR_SYMBOL		(4)
#define DSS_QD_ATTR_BINARY		(5)
#define DSS_QD_ATTR_ELF			(6)
#define DSS_QD_ATTR_SKIP_ENCRYPT	(1 << 31)

struct dbg_snapshot_qd_region {
	char	name[DSS_QD_MAX_NAME_LENGTH];
	char	struct_name[DSS_QD_MAX_NAME_LENGTH];
	u64	virt_addr;
	u64	phys_addr;
	u64	size;
	u32	attr;
	u32	magic;
};

struct dbg_snapshot_qd_table {
	struct dbg_snapshot_qd_region	*entry;
	u32				num_regions;
	u32				magic_key;
	u32				version;
};

struct dbg_snapshot_panic_backtrace {
	u64 paddr;
	char *vaddr;
	size_t size;
	u64 curr_idx;
	bool stop_logging;
};

#ifdef SAVE_PANIC_BACKTRACE
/*  Panic core's backtrace logging  */
static struct dbg_snapshot_panic_backtrace dss_panic_backtrace;
#endif
static struct dbg_snapshot_qd_table dss_qd_table;
static DEFINE_SPINLOCK(qd_lock);
static struct device *qdump_dev;

int dbg_snapshot_qd_add_region(void *v_entry, u32 attr)
{
	struct dbg_snapshot_qd_region *entry =
			(struct dbg_snapshot_qd_region *)v_entry;
	struct dbg_snapshot_qd_region *qd_entry;
	u32 entries, i;
	int ret = 0;

	if (!entry || !entry->virt_addr ||
			!entry->phys_addr || !strlen(entry->name)) {
		dev_err(qdump_dev, "Invalid entry details\n");
		return -EINVAL;
	}

	if ((strlen(entry->name) > sizeof(qd_entry->name) - 1) ||
		(strlen(entry->struct_name) > sizeof(qd_entry->struct_name) - 1)) {
		dev_err(qdump_dev, "name too long\n");
		return -EINVAL;
	}

	spin_lock(&qd_lock);

	entries = dss_qd_table.num_regions;
	if (entries >= DSS_QD_MAX_NUM_ENTRIES) {
		dev_err(qdump_dev, "entries is full, No allowed more\n");
		spin_unlock(&qd_lock);
		return -ENOMEM;
	}

	for (i = 0; i < entries; i++) {
		qd_entry = &dss_qd_table.entry[i];
		if (!strcmp(entry->name, qd_entry->name)) {
			dev_err(qdump_dev, "entry name is exist in array : %s",
				entry->name);
			spin_unlock(&qd_lock);
			return -EINVAL;
		}
	}

	qd_entry = &dss_qd_table.entry[entries];
	strcpy(qd_entry->name, entry->name);
	strcpy(qd_entry->struct_name, entry->struct_name);
	qd_entry->virt_addr = entry->virt_addr;
	qd_entry->phys_addr = entry->phys_addr;
	qd_entry->size = entry->size;
	qd_entry->magic = DSS_SIGN_MAGIC | entries;
	qd_entry->attr = attr;

	dss_qd_table.num_regions = entries + 1;

	spin_unlock(&qd_lock);

	dev_info(qdump_dev, "Success to add (0x%x: %s)\n",
			entries, qd_entry->name);

	return ret;
}
EXPORT_SYMBOL_GPL(dbg_snapshot_qd_add_region);

bool dbg_snapshot_qd_enable(void)
{
	return dss_dpm.dump_mode == QUICK_DUMP;
}
EXPORT_SYMBOL_GPL(dbg_snapshot_qd_enable);

void dbg_snapshot_qd_dump_stack(u64 sp)
{
	struct dbg_snapshot_qd_region ksp_entry, ktsk_entry;
	u32 cpu = smp_processor_id();

	if (!qdump_dev)
		return;

	if (is_idle_task(current)) {
		dev_info(qdump_dev, "CPU%d TASK/STACK didn't needed in idle\n",
				cpu);
		return;
	}

	if (sp < KIMAGE_VADDR || sp > -256UL)
		sp = current_stack_pointer;

	sp &= ~(THREAD_SIZE - 1);
	scnprintf(ksp_entry.name, sizeof(ksp_entry.name), "KSTACK%d", cpu);
	ksp_entry.struct_name[0] = '\0';
	ksp_entry.virt_addr = sp;
	ksp_entry.phys_addr = virt_to_phys((uintptr_t *)sp);
	ksp_entry.size = THREAD_SIZE;
	if (dbg_snapshot_qd_add_region(&ksp_entry, DSS_QD_ATTR_STACK))
		dev_err(qdump_dev, "failed to add stack of cpu %d\n", cpu);

	scnprintf(ktsk_entry.name, sizeof(ktsk_entry.name), "KTASK%d", cpu);
	strlcpy(ktsk_entry.struct_name, "task_struct",
			sizeof(ktsk_entry.struct_name));
	ktsk_entry.virt_addr = (u64)current;
	ktsk_entry.phys_addr = virt_to_phys((uintptr_t *)current);
	ktsk_entry.size = sizeof(struct task_struct);
	if (dbg_snapshot_qd_add_region(&ktsk_entry, DSS_QD_ATTR_STRUCT))
		dev_err(qdump_dev, "failed to add current task %d\n", cpu);
}
EXPORT_SYMBOL_GPL(dbg_snapshot_qd_dump_stack);

static void register_dss_log_item(void)
{
	struct dbg_snapshot_qd_region qd_entry;
	int item_num = dbg_snapshot_log_get_num_items();
	struct dbg_snapshot_log_item *item;
	int i;

	for (i = 0; i < item_num; i++) {
		item = dbg_snapshot_log_get_item_by_index(i);
		if (item->entry.enabled) {
			strlcpy(qd_entry.name, item->name,
					sizeof(qd_entry.name));
			strlcpy(qd_entry.struct_name, item->name,
					sizeof(qd_entry.struct_name));
			qd_entry.virt_addr = item->entry.vaddr;
			qd_entry.phys_addr = item->entry.paddr;
			qd_entry.size = item->entry.size;
			if (dbg_snapshot_qd_add_region(&qd_entry,
						DSS_QD_ATTR_STRUCT))
				dev_err(qdump_dev, "Failed to add dss_log_item : %s\n",
						item->name);
		}
	}
}

static void register_dss_item(void)
{
	struct dbg_snapshot_qd_region qd_entry;
	int item_num = dbg_snapshot_get_num_items();
	struct dbg_snapshot_item *item;
	int i;

	for (i = 0; i < item_num; i++) {
		if (i == DSS_ITEM_KEVENTS_ID)
			continue;

		item = dbg_snapshot_get_item_by_index(i);
		if (item->entry.enabled) {
			strlcpy(qd_entry.name, item->name,
					sizeof(qd_entry.name));
			qd_entry.struct_name[0] = '\0';
			qd_entry.virt_addr = item->entry.vaddr;
			qd_entry.phys_addr = item->entry.paddr;
			qd_entry.size = item->entry.size;
			if (dbg_snapshot_qd_add_region(&qd_entry,
						DSS_QD_ATTR_BINARY))
				dev_err(qdump_dev, "Failed to add dss_item : %s\n",
						item->name);
		}
	}
}

static void dbg_snapshot_qd_list_dump(void)
{
	struct dbg_snapshot_qd_region *entry;
	unsigned long i, size = 0;

	if (!qdump_dev)
		return;

	dev_info(qdump_dev, "quickdump physical / virtual memory layout:\n");
	for (i = 0; i < dss_qd_table.num_regions; i++) {
		entry = &dss_qd_table.entry[i];
		dev_info(qdump_dev, "%-16s: -%16s: phys:%pad / virt:%pK / size:0x%llx\n",
			entry->name,
			entry->struct_name,
			&entry->phys_addr,
			(void *)entry->virt_addr,
			entry->size);

		size += entry->size;
	}

	dev_info(qdump_dev, "total_quick_dump_size: %ldKB, quick_dump_entry: 0x%llx\n",
			size / SZ_1K,
			__raw_readq(dbg_snapshot_get_header_vaddr() + DSS_OFFSET_QD_ENTRY));
}

static int dbg_snapshot_qd_probe(struct platform_device *pdev)
{
	if (dss_dpm.dump_mode != QUICK_DUMP) {
		dev_err(&pdev->dev, "No Quick dump mode\n");
		return -ENODEV;
	}

	qdump_dev = &pdev->dev;
	dss_qd_table.entry = devm_kzalloc(qdump_dev, (DSS_QD_MAX_NUM_ENTRIES *
				sizeof(struct dbg_snapshot_qd_region)), GFP_KERNEL);
	if (!dss_qd_table.entry) {
		dbg_snapshot_set_qd_entry(0);
		return -ENOMEM;
	}

	qdump_dev = &pdev->dev;
	dbg_snapshot_set_qd_entry((unsigned long)dss_qd_table.entry);
	dss_qd_table.num_regions = 0;

	register_dss_item();
	register_dss_log_item();

	dbg_snapshot_qd_list_dump();

	dev_info(&pdev->dev, "%s successful.\n", __func__);
	return 0;
}

static const struct of_device_id dbg_snapshot_qd_matches[] = {
	{ .compatible = "google,debug-snapshot-qdump", },
	{},
};
MODULE_DEVICE_TABLE(of, dbg_snapshot_qd_matches);

static struct platform_driver dbg_snapshot_qd_driver = {
	.probe		= dbg_snapshot_qd_probe,
	.driver		= {
		.name	= "debug-snapshot-qdump",
		.of_match_table	= of_match_ptr(dbg_snapshot_qd_matches),
	},
};
module_platform_driver(dbg_snapshot_qd_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("DebugSnapshot Quickdump Driver");
