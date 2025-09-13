// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2014-2018, 2019, The Linux Foundation. All rights reserved.
 */

#define pr_fmt(fmt) "arm-memlat-mon: " fmt

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/cpuidle.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/cpu_pm.h>
#include <linux/cpu.h>
#include <linux/of_fdt.h>
#include "governor.h"
#include "governor_memlat.h"
#include "governor_dsulat.h"
#include "arm-memlat-mon.h"
#include <linux/perf_event.h>
#include <linux/of_device.h>
#include <linux/mutex.h>
#include <trace/hooks/cpuidle.h>
#include <linux/spinlock.h>
#include <dt-bindings/soc/google/zuma-devfreq.h>

static struct exynos_devfreq_data *dsu_data;

static DEFINE_PER_CPU(bool, is_idle);
static DEFINE_PER_CPU(bool, is_on);
static DEFINE_PER_CPU(int, cpu_idle_state);

static struct workqueue_struct *memlat_wq;
static LIST_HEAD(cpu_grp_list);
static DEFINE_PER_CPU(struct memlat_cpu_grp*, cpu_grp_p);

#define MAX_COUNT_LIM 0xFFFFFFFFFFFFFFFF

int get_ev_data(int cpu, unsigned long *inst, unsigned long *cyc,
				      unsigned long *stall, unsigned long *l2_cachemiss,
				      unsigned long *l3_cachemiss, unsigned long *mem_stall,
				      unsigned long *l2_cache_wb, unsigned long *l3_cache_access,
				      unsigned long *mem_count, unsigned long *freq)
{
	struct memlat_cpu_grp *cpu_grp = per_cpu(cpu_grp_p, cpu);
	struct cpu_data *cpu_data = to_cpu_data(cpu_grp, cpu);

	if (!per_cpu(is_on, cpu))
		return -EINVAL;

	spin_lock(&cpu_data->pmu_lock);
	*inst = cpu_data->inst;
	*cyc = cpu_data->cyc;
	*freq = cpu_data->freq;
	*stall = cpu_data->stall;
	*l2_cachemiss = cpu_data->l2_cachemiss;
	*l3_cachemiss = cpu_data->l3_cachemiss;
	*l2_cache_wb = cpu_data->l2_cache_wb;
	*l3_cache_access = cpu_data->l3_cache_access;
	*mem_stall = cpu_data->mem_stall;
	*mem_count = cpu_data->l3_cachemiss;
	spin_unlock(&cpu_data->pmu_lock);

	return 0;
}
EXPORT_SYMBOL(get_ev_data);

void set_dsu_data(struct exynos_devfreq_data *dsu_data_drv) {
	dsu_data = dsu_data_drv;
}
EXPORT_SYMBOL(set_dsu_data);

static inline void read_event(struct event_data *event)
{
	unsigned long ev_count = 0;
	u64 total, enabled, running;

	if (!event->pevent)
		return;

	if (!per_cpu(is_on, event->pevent->cpu))
		return;

	total = per_cpu(is_idle, event->pevent->cpu) ?
		 event->total : perf_event_read_value(event->pevent, &enabled, &running);
	ev_count = total - event->prev_count;
	event->prev_count = total;
	event->last_delta = ev_count;
}

static inline void read_event_local(struct event_data *event)
{
	u64 total, enabled, running;
	int ret = 0;

	if (!event->pevent)
		return;

	if (event->pevent->oncpu == -1)
		return;

	if (!per_cpu(is_on, event->pevent->cpu))
		return;

	ret = perf_event_read_local(event->pevent, &total,
			  &enabled, &running);

	if (ret)
		pr_err("read event fail %d\n", ret);
	else
		event->total = total;

}

int read_perf_event_local(int cpu, unsigned int event_id, u64 *count)
{
	struct memlat_cpu_grp *cpu_grp;
	struct cpu_data *cpu_data;
	struct event_data *pmu_evs;

	if (cpu != raw_smp_processor_id())
		return -EINVAL;

	list_for_each_entry(cpu_grp, &cpu_grp_list, node) {
		if (!cpu_grp->initialized)
			continue;

		if (cpumask_test_cpu(cpu, &cpu_grp->cpus))
			break;
	}

	if (!cpu_grp || event_id >= NUM_COMMON_EVS)
		return -EINVAL;

	cpu_data = to_cpu_data(cpu_grp, cpu);
	if (cpu_grp->num_active_mons > 0 && cpu_grp->pmu_ev_ids[event_id] != UINT_MAX) {
		pmu_evs = cpu_data->pmu_evs;
		read_event_local(&pmu_evs[event_id]);
		*count = pmu_evs[event_id].total;
	} else if (event_id == CYCLE_IDX && cpu_grp->amu_ev_ids[CYCLE_IDX] != UINT_MAX) {
		*count = read_sysreg_s(SYS_AMEVCNTR0_CORE_EL0);
	} else if (event_id == INST_IDX && cpu_grp->amu_ev_ids[INST_IDX] != UINT_MAX) {
		*count = read_sysreg_s(SYS_AMEVCNTR0_INST_RET_EL0);
	} else if (event_id == STALL_BACKEND_MEM_IDX &&
			   cpu_grp->amu_ev_ids[STALL_BACKEND_MEM_IDX] != UINT_MAX) {
		*count = read_sysreg_s(SYS_AMEVCNTR0_MEM_STALL);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(read_perf_event_local);

static void read_amu_counters(struct memlat_cpu_grp *cpu_grp)
{
	unsigned long mem_stall, cpu_inst, cpu_cycle, delta;
	struct cpu_data *cpu_data;
	unsigned int cpu;

	for_each_cpu(cpu, &cpu_grp->cpus) {
		cpu_data = to_cpu_data(cpu_grp, cpu);

		if (cpu_grp->amu_ev_ids[CYCLE_IDX] != UINT_MAX) {
			cpu_cycle = read_sysreg_s(SYS_AMEVCNTR0_CORE_EL0);
			delta = cpu_cycle - cpu_data->amu_evs[CYCLE_IDX].prev_count;
			cpu_data->amu_evs[CYCLE_IDX].prev_count = cpu_cycle;
			cpu_data->amu_evs[CYCLE_IDX].last_delta = delta;
			cpu_data->cyc = cpu_data->amu_evs[CYCLE_IDX].last_delta;
		}

		if (cpu_grp->amu_ev_ids[INST_IDX] != UINT_MAX) {
			cpu_inst = read_sysreg_s(SYS_AMEVCNTR0_INST_RET_EL0);
			delta = cpu_inst - cpu_data->amu_evs[INST_IDX].prev_count;
			cpu_data->amu_evs[INST_IDX].prev_count = cpu_inst;
			cpu_data->amu_evs[INST_IDX].last_delta = delta;
			cpu_data->inst = cpu_data->amu_evs[INST_IDX].last_delta;
		}

		if (cpu_grp->amu_ev_ids[STALL_BACKEND_MEM_IDX] != UINT_MAX) {
			mem_stall = read_sysreg_s(SYS_AMEVCNTR0_MEM_STALL);
			delta = mem_stall - cpu_data->amu_evs[STALL_BACKEND_MEM_IDX].prev_count;
			cpu_data->amu_evs[STALL_BACKEND_MEM_IDX].prev_count = mem_stall;
			cpu_data->amu_evs[STALL_BACKEND_MEM_IDX].last_delta = delta;
			cpu_data->mem_stall = cpu_data->amu_evs[STALL_BACKEND_MEM_IDX].last_delta;
		}
	}
}

static void read_pmu_counters(struct memlat_cpu_grp *cpu_grp)
{
	struct cpu_data *cpu_data;
	struct event_data *pmu_evs;
	unsigned int cpu, i;

	for_each_cpu(cpu, &cpu_grp->cpus) {
		cpu_data = to_cpu_data(cpu_grp, cpu);
		pmu_evs = cpu_data->pmu_evs;
		for (i = 0; i < NUM_COMMON_EVS; i++)
			read_event(&pmu_evs[i]);

		if (cpu_grp->pmu_ev_ids[CYCLE_IDX] != UINT_MAX)
			cpu_data->cyc = pmu_evs[CYCLE_IDX].last_delta;
		if (cpu_grp->pmu_ev_ids[INST_IDX] != UINT_MAX)
			cpu_data->inst = pmu_evs[INST_IDX].last_delta;
		if (cpu_grp->pmu_ev_ids[STALL_BACKEND_MEM_IDX] != UINT_MAX)
			cpu_data->mem_stall = pmu_evs[STALL_BACKEND_MEM_IDX].last_delta;
		if (cpu_grp->pmu_ev_ids[STALL_IDX] != UINT_MAX)
			cpu_data->stall = pmu_evs[STALL_IDX].last_delta;
		if (cpu_grp->pmu_ev_ids[L2D_CACHE_REFILL_IDX] != UINT_MAX)
			cpu_data->l2_cachemiss = pmu_evs[L2D_CACHE_REFILL_IDX].last_delta;
		if (cpu_grp->pmu_ev_ids[L3_CACHE_MISS_IDX] != UINT_MAX)
			cpu_data->l3_cachemiss = pmu_evs[L3_CACHE_MISS_IDX].last_delta;
	}
}

static void vendor_update_event_cpu_idle_enter(void *data, int *state, struct cpuidle_device *dev)
{
	if (!__this_cpu_read(is_on))
		return;

	__this_cpu_write(is_idle, true);
	__this_cpu_write(cpu_idle_state, *state);
}

static void vendor_update_event_cpu_idle_exit(void *data, int state, struct cpuidle_device *dev)
{
	if (!__this_cpu_read(is_on))
		return;
	__this_cpu_write(is_idle, false);
	__this_cpu_write(cpu_idle_state, state);
}

int get_cpu_idle_state(unsigned int cpu)
{
	return per_cpu(is_idle, cpu) ? per_cpu(cpu_idle_state, cpu) : -1;
}
EXPORT_SYMBOL(get_cpu_idle_state);

static void update_counts(struct memlat_cpu_grp *cpu_grp)
{
	unsigned int cpu;
	ktime_t now = ktime_get();
	unsigned long delta = ktime_us_delta(now, cpu_grp->last_update_ts);
	struct cpu_data *cpu_data;

	cpu_grp->last_ts_delta_us = delta;
	cpu_grp->last_update_ts = now;

	read_pmu_counters(cpu_grp);
	read_amu_counters(cpu_grp);

	for_each_cpu(cpu, &cpu_grp->cpus) {
		cpu_data = to_cpu_data(cpu_grp, cpu);

		cpu_data->freq = cpu_data->cyc / delta;
		cpu_data->stall_pct = mult_frac(10000,
				cpu_data->mem_stall,
				cpu_data->cyc);
		cpu_data->l2_cache_wb = 0;
		cpu_data->l3_cache_access = 0;
	}

}

static unsigned long get_cnt(struct memlat_hwmon *hw)
{
	struct memlat_mon *mon = to_mon(hw);
	struct memlat_cpu_grp *cpu_grp = mon->cpu_grp;
	unsigned int cpu;

	for_each_cpu(cpu, &mon->cpus) {
		struct cpu_data *cpu_data = to_cpu_data(cpu_grp, cpu);
		struct dev_stats *devstats = to_devstats(mon, cpu);

		devstats->freq = cpu_data->freq;
		devstats->stall_pct = cpu_data->stall_pct;
		devstats->inst_count = cpu_data->inst;
		devstats->mem_stall_count = cpu_data->mem_stall;
		devstats->l2_cachemiss_count = cpu_data->l2_cachemiss;
		devstats->mem_count = cpu_data->l3_cachemiss;
	}

	return 0;
}

static void delete_event(struct event_data *event)
{
	event->prev_count = event->last_delta = 0;
	if (event->pevent) {
		perf_event_release_kernel(event->pevent);
		event->pevent = NULL;
	}
}

static struct perf_event_attr *alloc_attr(void)
{
	struct perf_event_attr *attr;

	attr = kzalloc(sizeof(struct perf_event_attr), GFP_KERNEL);
	if (!attr)
		return attr;

	attr->type = PERF_TYPE_RAW;
	attr->size = sizeof(struct perf_event_attr);
	attr->pinned = 1;

	return attr;
}

static int set_event(struct event_data *ev, int cpu, unsigned int event_id,
		     struct perf_event_attr *attr)
{
	struct perf_event *pevent;

	if (!event_id || event_id == UINT_MAX)
		return 0;

	attr->config = event_id;
	pevent = perf_event_create_kernel_counter(attr, cpu, NULL, NULL, NULL);
	if (IS_ERR(pevent))
		return PTR_ERR(pevent);

	ev->pevent = pevent;
	perf_event_enable(pevent);

	return 0;
}

static int memlat_cpuhp_up(unsigned int cpu)
{
	int ret = 0;
	unsigned int i;
	struct memlat_cpu_grp *cpu_grp;
	struct cpu_data *cpu_data;
	struct event_data *pmu_evs;
	struct perf_event_attr *attr = alloc_attr();

	if (!attr)
		return -ENOMEM;

	list_for_each_entry(cpu_grp, &cpu_grp_list, node) {
		if (!cpu_grp->initialized)
			continue;

		if (cpumask_test_cpu(cpu, &cpu_grp->cpus))
			break;
	}

	if (!cpu_grp) {
		kfree(attr);
		return -EINVAL;
	}

	mutex_lock(&cpu_grp->mons_lock);
	if (!cpu_grp->num_active_mons)
		goto unlock_out;

	cpu_data = to_cpu_data(cpu_grp, cpu);
	pmu_evs = cpu_data->pmu_evs;
	for (i = 0; i < NUM_COMMON_EVS; i++) {
		ret = set_event(&pmu_evs[i], cpu,
		  cpu_grp->pmu_ev_ids[i], attr);
		if (ret) {
			pr_err("set event %u on CPU %u fail: %d",
			  cpu_grp->pmu_ev_ids[i],
			  cpu, ret);
			goto unlock_out;
		}
	}

	per_cpu(is_on, cpu) = true;
unlock_out:
	mutex_unlock(&cpu_grp->mons_lock);
	kfree(attr);
	return ret;
}
static int memlat_cpuhp_down(unsigned int cpu)
{
	int ret = 0;
	unsigned int i;
	struct memlat_cpu_grp *cpu_grp;
	struct cpu_data *cpu_data;
	struct event_data *pmu_evs;

	list_for_each_entry(cpu_grp, &cpu_grp_list, node) {
		if (!cpu_grp->initialized)
			continue;
		if (cpumask_test_cpu(cpu, &cpu_grp->cpus))
			break;
	}

	if (!cpu_grp)
		return -EINVAL;

	mutex_lock(&cpu_grp->mons_lock);
	if (!cpu_grp->num_active_mons)
		goto unlock_out;

	cpu_data = to_cpu_data(cpu_grp, cpu);
	pmu_evs = cpu_data->pmu_evs;

	for (i = 0; i < NUM_COMMON_EVS; i++)
		delete_event(&pmu_evs[i]);

	per_cpu(is_on, cpu) = false;

unlock_out:
	mutex_unlock(&cpu_grp->mons_lock);
	return ret;
}

static int init_memlat_cpuhp(void)
{
	int ret = 0;

	ret = cpuhp_setup_state_nocalls(CPUHP_AP_ONLINE_DYN, "mem_latency",
	  memlat_cpuhp_up, memlat_cpuhp_down);

	if (ret < 0)
		pr_err("init cpuhp fail:%d\n", ret);

	return ret;
}


static int init_pmu_evs(struct memlat_cpu_grp *cpu_grp,
			   struct perf_event_attr *attr)
{
	unsigned int cpu, i;
	int ret = 0;

	cpu_grp->initialized = false;
	for_each_cpu(cpu, &cpu_grp->cpus) {
		struct event_data *pmu_evs = to_pmu_evs(cpu_grp, cpu);

		for (i = 0; i < NUM_COMMON_EVS; i++) {
			ret = set_event(&pmu_evs[i], cpu,
					cpu_grp->pmu_ev_ids[i], attr);
			if (ret)
				break;
		}
	}
	cpu_grp->initialized = true;

	return ret;
}

static inline void queue_cpugrp_work(struct memlat_cpu_grp *cpu_grp)
{
	if (cpumask_weight(&cpu_grp->cpus) == 1)
		queue_delayed_work_on(cpumask_first(&cpu_grp->cpus),  memlat_wq, &cpu_grp->work,
		  msecs_to_jiffies(cpu_grp->update_ms));
	else
		queue_delayed_work(memlat_wq, &cpu_grp->work,
		  msecs_to_jiffies(cpu_grp->update_ms));
}

static void free_pmu_evs(struct memlat_cpu_grp *cpu_grp)
{
	unsigned int cpu, i;

	for_each_cpu(cpu, &cpu_grp->cpus) {
		struct event_data *pmu_evs = to_pmu_evs(cpu_grp, cpu);

		for (i = 0; i < NUM_COMMON_EVS; i++)
			delete_event(&pmu_evs[i]);
	}
}

static void memlat_monitor_work(struct work_struct *work)
{
	int err;
	struct memlat_cpu_grp *cpu_grp =
		container_of(work, struct memlat_cpu_grp, work.work);
	struct memlat_mon *mon;
	unsigned int i;

	mutex_lock(&cpu_grp->mons_lock);
	if (!cpu_grp->num_active_mons)
		goto unlock_out;
	update_counts(cpu_grp);
	for (i = 0; i < cpu_grp->num_mons; i++) {
		struct devfreq *df;

		mon = &cpu_grp->mons[i];

		if (!mon->is_active)
			continue;

		df = mon->hw.df;
		mutex_lock(&df->lock);
		err = update_devfreq(df);
		if (err)
			dev_err(mon->hw.dev, "Memlat update failed: %d\n", err);
		mutex_unlock(&df->lock);

		// Add callback for DSU
		if (mon->update_dsu_df) {
			if (dsu_data && dsu_data->devfreq) {
				struct devfreq *dsu_df = dsu_data->devfreq;
				mutex_lock(&dsu_df->lock);
				err = update_devfreq(dsu_df);
				if (err)
					dev_err(&dsu_df->dev, "Dsulat update failed: %d\n", err);
				mutex_unlock(&dsu_df->lock);
			}
		}
	}

	queue_cpugrp_work(cpu_grp);

unlock_out:
	mutex_unlock(&cpu_grp->mons_lock);
}

static int start_hwmon(struct memlat_hwmon *hw)
{
	int ret = 0;
	struct memlat_mon *mon = to_mon(hw);
	struct memlat_cpu_grp *cpu_grp = mon->cpu_grp;
	bool should_init_cpu_grp;
	struct perf_event_attr *attr = alloc_attr();

	if (!attr)
		return -ENOMEM;

	mutex_lock(&cpu_grp->mons_lock);
	should_init_cpu_grp = !(cpu_grp->num_active_mons++);
	if (should_init_cpu_grp) {
		ret = init_pmu_evs(cpu_grp, attr);
		if (ret)
			goto unlock_out;

		INIT_LIST_HEAD(&cpu_grp->node);
		list_add(&cpu_grp->node, &cpu_grp_list);
		INIT_DEFERRABLE_WORK(&cpu_grp->work, &memlat_monitor_work);
	}


	mon->is_active = true;

	if (should_init_cpu_grp)
		queue_cpugrp_work(cpu_grp);

unlock_out:
	mutex_unlock(&cpu_grp->mons_lock);
	kfree(attr);

	return ret;
}

static void devfreq_dsulat_boost_freq(struct exynos_devfreq_data *dsu_data)
{
	unsigned int max_freq, min_freq;
	struct dsulat_node *node = dsu_data->governor_data;

	/* get maximal frequency for DSU */
	exynos_devfreq_get_boundary(DEVFREQ_DSU, &max_freq, &min_freq);
	if (exynos_pm_qos_request_active(&dsu_data->sys_pm_qos_min))
		exynos_pm_qos_update_request(&dsu_data->sys_pm_qos_min, max_freq);

	/* get maximal frequency for BCI */
	exynos_devfreq_get_boundary(DEVFREQ_BCI, &max_freq, &min_freq);
	if (exynos_pm_qos_request_active(&node->dsu_bci_qos_req))
		exynos_pm_qos_update_request(&node->dsu_bci_qos_req, max_freq);
}

static void stop_hwmon(struct memlat_hwmon *hw)
{
	unsigned int cpu;
	struct memlat_mon *mon = to_mon(hw);
	struct memlat_cpu_grp *cpu_grp = mon->cpu_grp;

	mutex_lock(&cpu_grp->mons_lock);
	mon->is_active = false;
	cpu_grp->num_active_mons--;

	for_each_cpu(cpu, &mon->cpus) {

		struct dev_stats *devstats = to_devstats(mon, cpu);

		devstats->inst_count = 0;
		devstats->mem_count = 0;
		devstats->freq = 0;
		devstats->stall_pct = 0;
		devstats->l2_cachemiss_count = 0;
	}

	if (!cpu_grp->num_active_mons) {
		cancel_delayed_work(&cpu_grp->work);
		free_pmu_evs(cpu_grp);
		cpu_grp->initialized=false;
		list_del(&cpu_grp->node);
	}
	mutex_unlock(&cpu_grp->mons_lock);

	/* boost DSU freqeuncy to max for simpleperf */
	devfreq_dsulat_boost_freq(dsu_data);
}

/**
 * We should set update_ms to the lowest requested_update_ms of all of the
 * active mons, or 0 (i.e. stop polling) if ALL active mons have 0.
 * This is expected to be called with cpu_grp->mons_lock taken.
 */
static void set_update_ms(struct memlat_cpu_grp *cpu_grp)
{
	struct memlat_mon *mon;
	unsigned int i, new_update_ms = UINT_MAX;

	for (i = 0; i < cpu_grp->num_mons; i++) {
		mon = &cpu_grp->mons[i];
		if (mon->is_active && mon->requested_update_ms)
			new_update_ms =
				min(new_update_ms, mon->requested_update_ms);
	}

	if (new_update_ms == UINT_MAX) {
		cancel_delayed_work(&cpu_grp->work);
	} else if (cpu_grp->update_ms == UINT_MAX) {
		queue_cpugrp_work(cpu_grp);
	} else if (new_update_ms > cpu_grp->update_ms) {
		cancel_delayed_work(&cpu_grp->work);
		queue_cpugrp_work(cpu_grp);
	}

	cpu_grp->update_ms = new_update_ms;
}

static void request_update_ms(struct memlat_hwmon *hw, unsigned int update_ms)
{
	struct devfreq *df = hw->df;
	struct memlat_mon *mon = to_mon(hw);
	struct memlat_cpu_grp *cpu_grp = mon->cpu_grp;

	mutex_lock(&df->lock);
	df->profile->polling_ms = update_ms;
	mutex_unlock(&df->lock);

	mutex_lock(&cpu_grp->mons_lock);
	mon->requested_update_ms = update_ms;
	set_update_ms(cpu_grp);
	mutex_unlock(&cpu_grp->mons_lock);
}

static int get_mask_from_dev_handle(struct platform_device *pdev,
					cpumask_t *mask)
{
	struct device *dev = &pdev->dev;
	struct device_node *dev_phandle;
	struct device *cpu_dev;
	int cpu, i = 0;
	int ret = -ENOENT;

	dev_phandle = of_parse_phandle(dev->of_node, "cpulist", i++);
	while (dev_phandle) {
		for_each_possible_cpu(cpu) {
			cpu_dev = get_cpu_device(cpu);
			if (cpu_dev && cpu_dev->of_node == dev_phandle) {
				cpumask_set_cpu(cpu, mask);
				ret = 0;
				break;
			}
		}
		dev_phandle = of_parse_phandle(dev->of_node,
						"cpulist", i++);
	}

	return ret;
}


#define DEFAULT_UPDATE_MS 100
static int memlat_cpu_grp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *pmu_node;
	struct device_node *amu_node;
	struct memlat_cpu_grp *cpu_grp;
	int ret = 0;
	unsigned int event_id, num_cpus, num_mons, num_child_nodes;
	unsigned int num_not_mon_nodes = 0;

	cpu_grp = devm_kzalloc(dev, sizeof(*cpu_grp), GFP_KERNEL);
	if (!cpu_grp)
		return -ENOMEM;

	if (get_mask_from_dev_handle(pdev, &cpu_grp->cpus)) {
		dev_err(dev, "No CPUs specified.\n");
		return -ENODEV;
	}

	num_child_nodes = of_get_available_child_count(dev->of_node);

	pmu_node = of_get_child_by_name(dev->of_node, "pmu");

	if (!pmu_node) {
		memset(cpu_grp->pmu_ev_ids, UINT_MAX, NUM_COMMON_EVS);
		dev_dbg(dev, "PMU node not defined.\n");
	} else {
		num_not_mon_nodes += 1;

		ret = of_property_read_u32(pmu_node, "inst-ev", &event_id);
		if (ret) {
			dev_dbg(dev, "inst event not specified. Skipping.\n");
			event_id = UINT_MAX;
		}
		cpu_grp->pmu_ev_ids[INST_IDX] = event_id;

		ret = of_property_read_u32(pmu_node, "cyc-ev", &event_id);
		if (ret) {
			dev_dbg(dev, "cyc event not specified. Skipping.\n");
			event_id = UINT_MAX;
		}
		cpu_grp->pmu_ev_ids[CYCLE_IDX] = event_id;

		ret = of_property_read_u32(pmu_node, "stall-ev", &event_id);
		if (ret) {
			dev_dbg(dev, "Stall event not specified. Skipping.\n");
			event_id = UINT_MAX;
		}
		cpu_grp->pmu_ev_ids[STALL_IDX] = event_id;

		ret = of_property_read_u32(pmu_node, "stall-backend-mem-ev", &event_id);
		if (ret) {
			dev_dbg(dev, "Stall backend event not specified. Skipping.\n");
			event_id = UINT_MAX;
		}
		cpu_grp->pmu_ev_ids[STALL_BACKEND_MEM_IDX] = event_id;

		ret = of_property_read_u32(pmu_node, "l2-cachemiss-ev", &event_id);
		if (ret) {
			dev_dbg(dev, "L2 cachemiss event not specified. Skipping.\n");
			event_id = UINT_MAX;
		}
		cpu_grp->pmu_ev_ids[L2D_CACHE_REFILL_IDX] = event_id;

		ret = of_property_read_u32(pmu_node, "l3-cachemiss-ev", &event_id);
		if (ret) {
			dev_dbg(dev, "L3 cachemiss event not specified. Skipping.\n");
			event_id = UINT_MAX;
		}
		cpu_grp->pmu_ev_ids[L3_CACHE_MISS_IDX] = event_id;

	}

	amu_node = of_get_child_by_name(dev->of_node, "amu");

	if (!amu_node) {
		memset(cpu_grp->amu_ev_ids, UINT_MAX, NUM_COMMON_EVS);
		dev_dbg(dev, "AMU node not defined.\n");
	} else {
		num_not_mon_nodes += 1;
		ret = of_property_read_u32(amu_node, "inst-ev", &event_id);
		if (ret) {
			dev_dbg(dev, "inst event not specified. Skipping.\n");
			event_id = UINT_MAX;
		}
		cpu_grp->amu_ev_ids[INST_IDX] = event_id;

		ret = of_property_read_u32(amu_node, "cyc-ev", &event_id);
		if (ret) {
			dev_dbg(dev, "cyc event not specified. Skipping.\n");
			event_id = UINT_MAX;
		}
		cpu_grp->amu_ev_ids[CYCLE_IDX] = event_id;

		ret = of_property_read_u32(amu_node, "stall-ev", &event_id);
		if (ret) {
			dev_dbg(dev, "Stall event not specified. Skipping.\n");
			event_id = UINT_MAX;
		}
		cpu_grp->amu_ev_ids[STALL_IDX] = event_id;

		ret = of_property_read_u32(amu_node, "stall-backend-mem-ev", &event_id);
		if (ret) {
			dev_dbg(dev, "Stall backend event not specified. Skipping.\n");
			event_id = UINT_MAX;
		}
		cpu_grp->amu_ev_ids[STALL_BACKEND_MEM_IDX] = event_id;

		ret = of_property_read_u32(amu_node, "l2-cachemiss-ev", &event_id);
		if (ret) {
			dev_dbg(dev, "L2 cachemiss event not specified. Skipping.\n");
			event_id = UINT_MAX;
		}
		cpu_grp->amu_ev_ids[L2D_CACHE_REFILL_IDX] = event_id;

		ret = of_property_read_u32(amu_node, "l3-cachemiss-ev", &event_id);
		if (ret) {
			dev_dbg(dev, "L3 cachemiss event not specified. Skipping.\n");
			event_id = UINT_MAX;
		}
		cpu_grp->amu_ev_ids[L3_CACHE_MISS_IDX] = event_id;


	}

	num_mons = num_child_nodes - num_not_mon_nodes;

	if (!num_mons || !num_not_mon_nodes) {
		dev_err(dev, "No mons or neither pmu/amu provided.\n");
		return -ENODEV;
	}

	cpu_grp->num_mons = num_mons;
	cpu_grp->num_inited_mons = 0;

	cpu_grp->mons =
		devm_kzalloc(dev, num_mons * sizeof(*cpu_grp->mons),
			     GFP_KERNEL);
	if (!cpu_grp->mons)
		return -ENOMEM;


	num_cpus = cpumask_weight(&cpu_grp->cpus);
	cpu_grp->cpus_data =
		devm_kzalloc(dev, num_cpus * sizeof(*cpu_grp->cpus_data),
			     GFP_KERNEL);
	if (!cpu_grp->cpus_data)
		return -ENOMEM;

	mutex_init(&cpu_grp->mons_lock);
	cpu_grp->update_ms = DEFAULT_UPDATE_MS;

	dev_set_drvdata(dev, cpu_grp);

	return 0;
}

static int memlat_mon_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int ret = 0;
	struct memlat_cpu_grp *cpu_grp;
	struct memlat_mon *mon;
	struct memlat_hwmon *hw;
	unsigned int num_cpus, cpu;
	struct cpu_data *cpu_data;
	static int arm_mon_probe_count = 0;

	if (!memlat_wq)
		memlat_wq = alloc_workqueue("memlat_wq",
		  __WQ_LEGACY | WQ_FREEZABLE | WQ_MEM_RECLAIM, 1);

	if (!memlat_wq) {
		dev_err(dev, "Couldn't create memlat workqueue.\n");
		return -ENOMEM;
	}

	cpu_grp = dev_get_drvdata(dev->parent);
	if (!cpu_grp) {
		dev_err(dev, "Mon initialized without cpu_grp.\n");
		return -ENODEV;
	}

	mutex_lock_nested(&cpu_grp->mons_lock, SINGLE_DEPTH_NESTING);
	mon = &cpu_grp->mons[cpu_grp->num_inited_mons];
	mon->is_active = false;
	mon->requested_update_ms = 0;
	mon->cpu_grp = cpu_grp;

	for_each_cpu (cpu, &cpu_grp->cpus) {
		per_cpu(cpu_grp_p, cpu) = cpu_grp;
		cpu_data = to_cpu_data(cpu_grp, cpu);
		spin_lock_init(&cpu_data->pmu_lock);
	}

	if (get_mask_from_dev_handle(pdev, &mon->cpus)) {
		cpumask_copy(&mon->cpus, &cpu_grp->cpus);
	} else {
		if (!cpumask_subset(&mon->cpus, &cpu_grp->cpus)) {
			dev_err(dev,
				"Mon CPUs must be a subset of cpu_grp CPUs. mon=%*pbl cpu_grp=%*pbl\n",
				cpumask_pr_args(&mon->cpus), cpumask_pr_args(&cpu_grp->cpus));
			ret = -EINVAL;
			goto unlock_out;
		}
	}

	num_cpus = cpumask_weight(&mon->cpus);

	hw = &mon->hw;
	hw->of_node = of_parse_phandle(dev->of_node, "target-dev", 0);
	if (!hw->of_node) {
		dev_err(dev, "Couldn't find a target device.\n");
		ret = -ENODEV;
		goto unlock_out;
	}
	hw->dev = dev;
	hw->num_cores = num_cpus;
	hw->should_ignore_df_monitor = true;
	hw->core_stats = devm_kzalloc(dev, num_cpus * sizeof(*(hw->core_stats)),
				      GFP_KERNEL);
	if (!hw->core_stats) {
		ret = -ENOMEM;
		goto unlock_out;
	}

	for_each_cpu(cpu, &mon->cpus)
		to_devstats(mon, cpu)->id = cpu;

	hw->start_hwmon = &start_hwmon;
	hw->stop_hwmon = &stop_hwmon;
	hw->get_cnt = &get_cnt;
	hw->get_cpu_idle_state = &get_cpu_idle_state;
	hw->request_update_ms = &request_update_ms;

	mon->update_dsu_df = of_property_read_bool(dev->of_node, "update-dsu-df");

	ret = register_memlat(dev, hw);

	arm_mon_probe_count++;
	if (!ret)
		cpu_grp->num_inited_mons++;

unlock_out:
	if (arm_mon_probe_count == num_possible_cpus())
		set_arm_mon_probe_done(true);
	mutex_unlock(&cpu_grp->mons_lock);
	return ret;
}

static bool hook_registered;
static bool cpuhp_registered;
static int arm_memlat_mon_driver_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int ret = 0;
	unsigned int cpu;
	const struct memlat_mon_spec *spec = of_device_get_match_data(dev);
	enum mon_type type = NUM_MON_TYPES;

	if (spec)
		type = spec->type;

	switch (type) {
	case MEMLAT_CPU_GRP:
		ret = memlat_cpu_grp_probe(pdev);
		if (of_get_available_child_count(dev->of_node))
			of_platform_populate(dev->of_node, NULL, NULL, dev);
		break;
	case MEMLAT_MON:
		ret = memlat_mon_probe(pdev);
		break;
	default:
		/*
		 * This should never happen.
		 */
		dev_err(dev, "Invalid memlat mon type specified: %u\n", type);
		return -EINVAL;
	}

	if (ret) {
		dev_err(dev, "Failure to probe memlat device: %d\n", ret);
		return ret;
	}

	if (!cpuhp_registered) {
		ret = init_memlat_cpuhp();

		if (ret < 0) {
			dev_err(dev, "Register CPU hotplug notifier fail %d\n", ret);
			return ret;
		}

		for_each_cpu(cpu, cpu_online_mask) {
			per_cpu(is_on, cpu) = true;
		}
		cpuhp_registered = true;
	}

	if (!hook_registered) {
		ret = register_trace_android_vh_cpu_idle_enter(
				vendor_update_event_cpu_idle_enter, NULL);
		if (ret) {
			dev_err(dev, "Register enter vendor hook fail %d\n", ret);
			return ret;
		}

		ret = register_trace_android_vh_cpu_idle_exit(
				vendor_update_event_cpu_idle_exit, NULL);
		if (ret) {
			dev_err(dev, "Register exit vendor hook fail %d\n", ret);
			return ret;
		}

		hook_registered = true;
	}

	return 0;
}

static const struct memlat_mon_spec spec[] = {
	[0] = { MEMLAT_CPU_GRP },
	[1] = { MEMLAT_MON },
};

static const struct of_device_id memlat_match_table[] = {
	{ .compatible = "arm-memlat-cpugrp", .data = &spec[0] },
	{ .compatible = "arm-memlat-mon", .data = &spec[1] },
	{}
};
MODULE_DEVICE_TABLE(of, memlat_match_table);

static struct platform_driver arm_memlat_mon_driver = {
	.probe = arm_memlat_mon_driver_probe,
	.driver = {
		.name = "arm-memlat-mon",
		.of_match_table = memlat_match_table,
		.suppress_bind_attrs = true,
	},
};

module_platform_driver(arm_memlat_mon_driver);
MODULE_LICENSE("GPL v2");
