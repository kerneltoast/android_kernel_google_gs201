// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/of_address.h>
#include <linux/of_reserved_mem.h>
#include <linux/sched/clock.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/libfdt.h>
#include <linux/ftrace.h>

#include "debug-snapshot-local.h"

static struct dbg_snapshot_log_misc *orig_dss_log_misc;
static struct dbg_snapshot_item *orig_dss_items;
static struct dbg_snapshot_log_item *orig_dss_log_items;
static struct dbg_snapshot_base *orig_dss_base;
static struct dbg_snapshot_log *orig_dss_log;

static inline bool dbg_snapshot_is_accessible(void)
{
	return orig_dss_items && orig_dss_log_items && orig_dss_log &&
		orig_dss_log_misc && orig_dss_base;
}

static inline bool dbg_snapshot_is_log_item_enabled(int id)
{
	bool item_enabled = orig_dss_items[DSS_ITEM_KEVENTS_ID].entry.enabled;
	bool log_enabled = orig_dss_log_items[id].entry.enabled;

	return orig_dss_base->enabled && item_enabled && log_enabled;
}

void dbg_snapshot_task(int cpu, void *v_task)
{
	unsigned long i;

	if (unlikely(!dbg_snapshot_is_accessible()))
		return;

	if (unlikely(!dbg_snapshot_is_log_item_enabled(DSS_LOG_TASK_ID)))
		return;

	i = atomic_fetch_inc(&orig_dss_log_misc->task_log_idx[cpu]) &
		(ARRAY_SIZE(orig_dss_log->task[0]) - 1);
	orig_dss_log->task[cpu][i].time = cpu_clock(cpu);
	orig_dss_log->task[cpu][i].task = (struct task_struct *)v_task;
	orig_dss_log->task[cpu][i].pid = (int)((struct task_struct *)v_task)->pid;
	strncpy(orig_dss_log->task[cpu][i].task_comm,
			orig_dss_log->task[cpu][i].task->comm,
			TASK_COMM_LEN - 1);
}

void dbg_snapshot_work(void *worker, void *v_task, work_func_t fn, int en)
{
	int cpu = raw_smp_processor_id();
	unsigned long i;
	struct task_struct *task = (struct task_struct *)v_task;

	if (unlikely(!dbg_snapshot_is_accessible()))
		return;

	if (unlikely(!dbg_snapshot_is_log_item_enabled(DSS_LOG_WORK_ID)))
		return;

	i = atomic_fetch_inc(&orig_dss_log_misc->work_log_idx[cpu]) &
		(ARRAY_SIZE(orig_dss_log->work[0]) - 1);
	orig_dss_log->work[cpu][i].time = cpu_clock(cpu);
	orig_dss_log->work[cpu][i].worker = (struct worker *)worker;
	strncpy(orig_dss_log->work[cpu][i].task_comm, task->comm, TASK_COMM_LEN - 1);
	orig_dss_log->work[cpu][i].fn = fn;
	orig_dss_log->work[cpu][i].en = en;
}

void dbg_snapshot_cpuidle(char *modes, unsigned int state, s64 diff, int en)
{
	int cpu = raw_smp_processor_id();
	unsigned long i;

	if (unlikely(!dbg_snapshot_is_accessible()))
		return;

	if (unlikely(!dbg_snapshot_is_log_item_enabled(DSS_LOG_CPUIDLE_ID)))
		return;

	i = atomic_fetch_inc(&orig_dss_log_misc->cpuidle_log_idx[cpu]) &
		(ARRAY_SIZE(orig_dss_log->cpuidle[0]) - 1);
	orig_dss_log->cpuidle[cpu][i].time = cpu_clock(cpu);
	orig_dss_log->cpuidle[cpu][i].modes = modes;
	orig_dss_log->cpuidle[cpu][i].state = state;
	orig_dss_log->cpuidle[cpu][i].num_online_cpus = num_online_cpus();
	orig_dss_log->cpuidle[cpu][i].delta = (int)diff;
	orig_dss_log->cpuidle[cpu][i].en = en;
}

void dbg_snapshot_irq(int irq, void *fn, void *val, unsigned long long start_time, int en)
{
	unsigned long flags, i;
	int cpu = raw_smp_processor_id();
	unsigned long long time, latency;

	if (unlikely(!dbg_snapshot_is_accessible()))
		return;

	if (unlikely(!dbg_snapshot_is_log_item_enabled(DSS_LOG_IRQ_ID)))
		return;

	i = atomic_fetch_inc(&orig_dss_log_misc->irq_log_idx[cpu]) &
		(ARRAY_SIZE(orig_dss_log->irq[0]) - 1);
	flags = arch_local_irq_save();
	time = cpu_clock(cpu);
	if (start_time == 0)
		start_time = time;

	latency = time - start_time;
	orig_dss_log->irq[cpu][i].time = time;
	orig_dss_log->irq[cpu][i].irq = irq;
	orig_dss_log->irq[cpu][i].fn = fn;
	orig_dss_log->irq[cpu][i].desc = (struct irq_desc *)val;
	orig_dss_log->irq[cpu][i].latency = latency;
	orig_dss_log->irq[cpu][i].en = en;

	arch_local_irq_restore(flags);
}

void dbg_snapshot_hrtimer(void *timer, s64 *now, void *fn, int en)
{
	int cpu = raw_smp_processor_id();
	unsigned long i;

	if (unlikely(!dbg_snapshot_is_accessible()))
		return;

	if (unlikely(!dbg_snapshot_is_log_item_enabled(DSS_LOG_HRTIMER_ID)))
		return;

	i = atomic_fetch_inc(&orig_dss_log_misc->hrtimer_log_idx[cpu]) &
		(ARRAY_SIZE(orig_dss_log->hrtimer[0]) - 1);
	orig_dss_log->hrtimer[cpu][i].time = cpu_clock(cpu);
	orig_dss_log->hrtimer[cpu][i].now = *now;
	orig_dss_log->hrtimer[cpu][i].timer = (struct hrtimer *)timer;
	orig_dss_log->hrtimer[cpu][i].fn = fn;
	orig_dss_log->hrtimer[cpu][i].en = en;
}

static int dbg_snapshot_built_probe(struct platform_device *pdev)
{
	struct reserved_mem *rmem;
	struct device_node *rmem_np;

	rmem_np = of_parse_phandle(pdev->dev.of_node, "memory-region", 0);
	if (!rmem_np) {
		dev_err(&pdev->dev, "no such memory-region\n");
		return -ENODEV;
	}

	rmem = of_reserved_mem_lookup(rmem_np);
	if (!rmem) {
		dev_err(&pdev->dev, "no such reserved mem of node name %s\n",
				&pdev->dev.of_node->name);
		return -ENODEV;
	}

	if (!rmem->base || !rmem->size || !rmem->priv)
		return -EPROBE_DEFER;

	orig_dss_base = (struct dbg_snapshot_base *)rmem->priv;
	orig_dss_log_misc = orig_dss_base->param->dss_log_misc;
	orig_dss_items = orig_dss_base->param->dss_items;
	orig_dss_log_items = orig_dss_base->param->dss_log_items;
	orig_dss_log = orig_dss_base->param->dss_log;

	if (unlikely(!dbg_snapshot_is_accessible()))
		dev_err(&pdev->dev, "%s: is not accessible.\n", __func__);

	dev_info(&pdev->dev, "%s successful.\n", __func__);
	return 0;
}

static const struct of_device_id dss_built_of_match[] = {
	{ .compatible	= "google,debug-snapshot-built" },
	{},
};
MODULE_DEVICE_TABLE(of, dss_of_match);

static struct platform_driver dss_built_driver = {
	.probe = dbg_snapshot_built_probe,
	.driver  = {
		.name  = "debug-snapshot-built",
		.of_match_table = of_match_ptr(dss_built_of_match),
	},
};
module_platform_driver(dss_built_driver);

MODULE_DESCRIPTION("DebugSnapshot Builin Driver");
MODULE_LICENSE("GPL v2");
