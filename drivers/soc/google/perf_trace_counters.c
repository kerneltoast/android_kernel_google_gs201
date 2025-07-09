// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2013-2014, 2017 The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/cpu.h>
#include <linux/debugfs.h>
#include <linux/kobject.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/trace_events.h>
#include <linux/tracepoint.h>
#include <linux/uaccess.h>
#include <trace/events/sched.h>
#include <trace/hooks/sched.h>
#define CREATE_TRACE_POINTS
#include "perf_trace_counters.h"

static unsigned int tp_pid_state;

DEFINE_PER_CPU(u32, cntenset_val);
DEFINE_PER_CPU(u64[NUM_EVENTS], previous_cnts);
DEFINE_PER_CPU(u32, old_pid);
DEFINE_PER_CPU(u32, hotplug_flag);

static DEFINE_MUTEX(perf_trace_lock);

#if IS_ENABLED(CONFIG_GS_PERF_MON)

const unsigned int ev_idx[NUM_EVENTS] = {
    PERF_INST_IDX,
    PERF_CYCLE_IDX,
    PERF_STALL_BACKEND_MEM_IDX,
    PERF_L3_CACHE_MISS_IDX
};

#else

const unsigned int ev_idx[NUM_EVENTS] = {
    INST_IDX,
    CYCLE_IDX,
    STALL_BACKEND_MEM_IDX,
    L3_CACHE_MISS_IDX
};

#endif

enum tp_pid_state_type {
	TP_DISABLED = 0,
	TP_ENABLED,
};

static int tracectr_cpu_hotplug_coming_up(unsigned int cpu)
{
	per_cpu(hotplug_flag, cpu) = 1;

	return 0;
}

static void setup_prev_cnts(u32 cpu)
{
	int i;
	u64 count;

	/* Read the INST, CYC, L3DM counts from performance monitor. */
	for (i = 0; i < NUM_EVENTS; i++) {
		count = per_cpu(previous_cnts[i], cpu);
		per_cpu(previous_cnts[i], cpu) =
			read_perf_event_local(cpu, ev_idx[i], &count) ?
				0 : count;
	}
}

static void tracectr_notifier(void *data, bool preempt,
			struct task_struct *prev, struct task_struct *next,
			unsigned int prev_state)
{
	u32 cnten_val;
	int current_pid;
	u32 cpu = task_cpu(next);

	if (tp_pid_state != TP_ENABLED)
		return;
	current_pid = next->pid;
	if (per_cpu(old_pid, cpu) != -1) {
		cnten_val = read_sysreg(pmcntenset_el0);
		per_cpu(cntenset_val, cpu) = cnten_val;
		/* Disable all the counters that were enabled */
		write_sysreg(cnten_val, pmcntenclr_el0);

		if (per_cpu(hotplug_flag, cpu) == 1) {
			per_cpu(hotplug_flag, cpu) = 0;
			setup_prev_cnts(cpu);
		} else {
			trace_sched_switch_with_ctrs(prev);
		}

		/* Enable all the counters that were disabled */
		write_sysreg(cnten_val, pmcntenset_el0);
	}
	per_cpu(old_pid, cpu) = current_pid;
}

/**
 * enable_tp_pid_locked - register the tracepoint & PMU counters
 *
 * This function requires caller to take perf_trace_lock.
 */
static int enable_tp_pid_locked(void)
{
	if (tp_pid_state == TP_ENABLED)
		return 0;

	tp_pid_state = TP_ENABLED;
	register_trace_sched_switch(tracectr_notifier, NULL);

	return 0;
}

/**
 * disable_tp_pid_locked - unregister the tracepoint and free counters
 *
 * This function requires caller to take perf_trace_lock.
 */
static void disable_tp_pid_locked(void)
{
	if (tp_pid_state == TP_DISABLED)
		return;

	tp_pid_state = TP_DISABLED;
	unregister_trace_sched_switch(tracectr_notifier, NULL);
}

/*
 * Sysfs structures
 */
static struct kobject *perf_trace_ctrs_kobj;

static ssize_t sysfs_enable_show(struct kobject *kobj,
		struct kobj_attribute *attr,
		char *buf)
{
	return sysfs_emit(buf, "%c\n", (tp_pid_state == TP_DISABLED) ? '0' : '1');
}

static ssize_t sysfs_enable_store(struct kobject *kobj,
		struct kobj_attribute *attr,
		const char *buf,
		size_t count)
{
	bool enable;

	if (kstrtobool(buf, &enable))
		return -EINVAL;

	mutex_lock(&perf_trace_lock);

	if (enable)
		enable_tp_pid_locked();
	else
		disable_tp_pid_locked();

	mutex_unlock(&perf_trace_lock);

	return count;
}

static struct kobj_attribute enable_attr = __ATTR(enable,
                0644,
                sysfs_enable_show,
                sysfs_enable_store);

static enum cpuhp_state hp_state;

static int __init init_tracecounters(void)
{
	int rc;

	rc = cpuhp_setup_state_nocalls(CPUHP_AP_ONLINE_DYN,
		"tracectr_cpu_hotplug",
		tracectr_cpu_hotplug_coming_up,
		NULL);

	if (rc < 0) {
		pr_err("%s: failed, error: %d\n", __func__, rc);
		return rc;
	}

	hp_state = rc;

	perf_trace_ctrs_kobj = kobject_create_and_add("perf_debug_tp", kernel_kobj);
	if (!perf_trace_ctrs_kobj) {
		pr_err("cannot create kobj for perf_debug_tp!\n");
		return -ENOMEM;
	}
	if (sysfs_create_file(perf_trace_ctrs_kobj, &enable_attr.attr)) {
		pr_err("cannot create file in perf_debug_tp folder!\n");
		kobject_put(perf_trace_ctrs_kobj);
		return -ENOMEM;
	}

	return 0;
}

static void __exit exit_tracecounters(void)
{
	cpuhp_remove_state_nocalls(hp_state);
	if (perf_trace_ctrs_kobj) {
		sysfs_remove_file(perf_trace_ctrs_kobj, &enable_attr.attr);
	}
	kobject_put(perf_trace_ctrs_kobj);
	perf_trace_ctrs_kobj = NULL;
}

module_init(init_tracecounters);
module_exit(exit_tracecounters);

MODULE_LICENSE("GPL");
