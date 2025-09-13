// SPDX-License-Identifier: GPL-2.0-only
/*
 * SLC PMON driver for perf subsystem.
 *
 * Copyright (C) Google LLC, 2020.
 *    Author:  Vincent Palomares <paillon@google.com>
 *
 * Based on PPMU perf driver.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 */

#define pr_fmt(fmt) "slc-pmon: " fmt

#include <linux/atomic.h>
#include <linux/device.h>
#include <linux/debugfs.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/perf_event.h>
#include <linux/spinlock.h>
#include <linux/sysfs.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/timekeeping.h>
#include <soc/google/acpm_ipc_ctrl.h>

#if IS_ENABLED(CONFIG_SLC_PMON)

#include "slc_pmon.h"
#include "slc_pmon_internal.h"

static struct slc_pmon_global_state slc_pmon_state = { 0 };
static struct pt_acpm_closure acpm_pt;
static struct dentry *slc_pmon_dir;

static struct attribute *slc_pmon_pmu_format_attrs[] = {
	SLC_PMON_FORMAT_ATTR("event", "config:0-31"),
	NULL,
};

static const struct attribute_group slc_pmon_pmu_format_attr_group = {
	.name = "format",
	.attrs = slc_pmon_pmu_format_attrs,
};

static struct attribute_group slc_pmon_pmu_events_attr_group = {
	.name = "events",
	.attrs = NULL,
};

static struct attribute *slc_pmon_pmu_cpumask_attrs[] = {
	SLC_PMON_CPUMASK_ATTR("cpumask", "0"),
	NULL,
};

static const struct attribute_group slc_pmon_pmu_cpumask_attr_group = {
	.attrs = slc_pmon_pmu_cpumask_attrs,
};

static const struct attribute_group *slc_pmon_attr_groups[] = {
	&slc_pmon_pmu_format_attr_group,
	&slc_pmon_pmu_events_attr_group,
	&slc_pmon_pmu_cpumask_attr_group,
	NULL
};

static ssize_t slc_pmon_pmu_sysfs_format_show(struct device *dev,
					      struct device_attribute *attr,
					      char *buf)
{
	struct dev_ext_attribute *eattr =
		container_of(attr, struct dev_ext_attribute, attr);
	return sysfs_emit(buf, "%s\n", (const char *)eattr->var);
}

static ssize_t slc_pmon_pmu_sysfs_event_show(struct device *dev,
					     struct device_attribute *attr,
					     char *buf)
{
	struct dev_ext_attribute *eattr =
		container_of(attr, struct dev_ext_attribute, attr);
	return sysfs_emit(buf, "event=0x%lx\n", (unsigned long)eattr->var);
}

static ssize_t slc_pmon_pmu_sysfs_cpumask_show(struct device *dev,
					       struct device_attribute *attr,
					       char *buf)
{
	struct dev_ext_attribute *eattr =
		container_of(attr, struct dev_ext_attribute, attr);
	return sysfs_emit(buf, "%s\n", (const char *)eattr->var);
}

static struct pmu slc_pmon_pmu = {
	.module = THIS_MODULE,
	.name = "slc",
	.capabilities = PERF_PMU_CAP_NO_INTERRUPT,
	.task_ctx_nr = perf_invalid_context,
	.event_init = slc_pmon_pmu_event_init,
	.add = slc_pmon_pmu_add,
	.del = slc_pmon_pmu_del,
	.start = slc_pmon_pmu_start,
	.stop = slc_pmon_pmu_stop,
	.read = slc_pmon_pmu_read,
	.attr_groups = slc_pmon_attr_groups,
};

static ssize_t slc_pmon_refresh_write(struct file *file, const char __user *ubf,
				      size_t count, loff_t *ppos)
{
	/* implement refresh code (load event list) */
	int res = count;

	mutex_lock(&slc_pmon_state.lock);

	// cleanup the events and counters
	if (acpm_pt.driver_data) {
		perf_pmu_unregister(&slc_pmon_pmu);
		slc_pmon_event_cleanup();
		slc_pmon_counter_cleanup();
	}

	// re-initialize the events and counters
	res = slc_pmon_counter_init();
	if (res) {
		pr_err("Fail to re-init counters\n");
		goto refresh_counter_cleanup;
	}

	res = slc_pmon_event_init();
	if (res) {
		pr_err("Fail to re-init events\n");
		goto refresh_event_cleanup;
	}

	res = perf_pmu_register(&slc_pmon_pmu, slc_pmon_pmu.name, -1);
	if (res < 0) {
		pr_err("Fail to re-register PMU driver\n");
		goto refresh_event_cleanup;
	}

	goto refresh_finish;

refresh_event_cleanup:
	slc_pmon_event_cleanup();

refresh_counter_cleanup:
	slc_pmon_counter_cleanup();

refresh_finish:
	mutex_unlock(&slc_pmon_state.lock);
	return count;
}

static const struct file_operations slc_pmon_refresh_fops = {
	.write = slc_pmon_refresh_write,
};

static int slc_pmon_acpm(enum slc_pmon_command command, int arg,
			 uint32_t *opt_buffer)
{
	return acpm_pt.slc_acpm(acpm_pt.driver_data, command, arg, 0,
				opt_buffer);
}

static int slc_acpm_get_num_events(void)
{
	return slc_pmon_acpm(SLC_PMON_QUERY_NUM_EVENTS, 0, NULL);
}

static int slc_acpm_allocate_counter(int event_id)
{
	int res = slc_pmon_acpm(SLC_PMON_ALLOCATE_COUNTER, event_id, NULL);

	if (res < 0 || res >= slc_pmon_state.counters.num_counters)
		return INVALID_COUNTER;

	return res;
}

static int slc_acpm_release_counter(int counter_id)
{
	int res = slc_pmon_acpm(SLC_PMON_RELEASE_COUNTER, counter_id, NULL);

	if (res < 0 || res >= slc_pmon_state.counters.num_counters)
		return -EINVAL;

	return 0;
}

static void slc_acpm_update_accumulator(int counter_id, bool force_refresh)
{
	uint64_t tmp;
	uint64_t __iomem *target_acc =
		(uint64_t *)slc_pmon_state.counters.slc_pmon_accumulators +
		counter_id;
	uint64_t now;

	/*
	 * If one event is updating the state, no event should read the state.
	 * Calls to the APM to poll counters should be rate-limited.
	 */

	mutex_lock(&slc_pmon_state.lock);

	now = ktime_get_ns();

	if ((now - slc_pmon_state.last_poll_ns) > ACC_REFRESH_INTERVAL_NS ||
	    force_refresh) {
		slc_pmon_acpm(SLC_PMON_POLL_COUNTERS, 0, NULL);
		slc_pmon_state.last_poll_ns = now;
	}

	tmp = ioread64(target_acc);

	mutex_unlock(&slc_pmon_state.lock);

	atomic64_set(&slc_pmon_state.counters.counters[counter_id].total_count,
		     tmp);
}

static const char *slc_acpm_get_event_name(int event_index)
{
	const char *res = NULL;
	uint32_t opt_buffer[8];
	if ((slc_pmon_acpm(SLC_PMON_QUERY_EVENT, event_index, opt_buffer) &
	     EVENT_ID_MASK) == event_index) {
		uint32_t *buffer_swap = &opt_buffer[2];
		int i;
		for (i = 0; i < 6; i++)
			be32_to_cpus(&buffer_swap[i]);
		// Defensive coding; string should already be NULL-terminated.
		((char *)buffer_swap)[23] = 0;
		res = kstrdup((char *)buffer_swap, GFP_KERNEL);
	}
	return res;
}

static int slc_acpm_get_num_counters(void)
{
	return slc_pmon_acpm(SLC_PMON_QUERY_NUM_COUNTERS, 0, NULL);
}

static void slc_pmon_pmu_read(struct perf_event *event)
{
	struct hw_perf_event *hw = &event->hw;
	uint64_t delta;
	uint64_t prev_count;
	uint64_t new_count;

	if (event->hw.config == INVALID_COUNTER)
		return;

	do {
		slc_acpm_update_accumulator(hw->config,
					    /*force_refresh*/ false);
		prev_count = local64_read(&hw->prev_count);
		new_count = atomic64_read(
			&slc_pmon_state.counters.counters[hw->config]
				 .total_count);
	} while (local64_cmpxchg(&hw->prev_count, prev_count, new_count) !=
		 prev_count);

	if (!(event->hw.state & PERF_HES_STOPPED)) {
		delta = new_count - prev_count;
		local64_add(delta, &event->count);
	}
}

static void slc_pmon_pmu_start(struct perf_event *event, int pmu_flags)
{
	dev_dbg(slc_pmon_pmu.dev, "Start perf_event [event: %llu]",
		event->attr.config);
	if (event->hw.state & PERF_HES_STOPPED) {
		// Force the update of the latest accumulator value.
		slc_acpm_update_accumulator(event->hw.config,
					    /*force_refresh*/ true);
		slc_pmon_pmu_read(event);
	}

	event->hw.state = 0;
}

static void slc_pmon_pmu_stop(struct perf_event *event, int pmu_flags)
{
	dev_dbg(slc_pmon_pmu.dev, "Stop perf_event [event: %llu]",
		event->attr.config);
	if (event->hw.state & PERF_HES_STOPPED)
		return;
	if ((pmu_flags & PERF_EF_UPDATE) &&
	    !(event->hw.state & PERF_HES_UPTODATE)) {
		// Force the update of the latest accumulator value.
		slc_acpm_update_accumulator(event->hw.config,
					    /*force_refresh*/ true);
		slc_pmon_pmu_read(event);
	}
	event->hw.state |= PERF_HES_STOPPED | PERF_HES_UPTODATE;
}

static int slc_pmon_pmu_add(struct perf_event *event, int evflags)
{
	dev_dbg(slc_pmon_pmu.dev, "Add perf_event [event: %llu]",
		event->attr.config);

	if (event->hw.flags & COUNTER_OWNER) {
		event->hw.config =
			slc_acpm_allocate_counter(event->attr.config);
		if (event->hw.config == INVALID_COUNTER) {
			return -EBUSY;
		}
		event->hw.state = PERF_HES_STOPPED;
		if (evflags & PERF_EF_START) {
			slc_pmon_pmu_start(event, PERF_EF_UPDATE);
		}
	}

	return 0;
}

static void slc_pmon_pmu_del(struct perf_event *event, int evflags)
{
	dev_dbg(slc_pmon_pmu.dev, "Del perf_event [event: %llu, counter: %llu].",
		event->attr.config, event->hw.config);
	if (event->hw.config != INVALID_COUNTER) {
		slc_pmon_pmu_stop(event, PERF_EF_UPDATE);
		slc_acpm_release_counter(event->hw.config);
		event->hw.config = INVALID_COUNTER;
	}
}

static int slc_pmon_pmu_event_init(struct perf_event *event)
{
	if (event->attr.type != event->pmu->type)
		return -ENOENT;

	if (event->cpu < 0) {
		dev_err(slc_pmon_pmu.dev, "Can't support per-task counters\n");
		return -EINVAL;
	}

	if (has_branch_stack(event) || event->attr.exclude_user ||
	    event->attr.exclude_kernel || event->attr.exclude_hv ||
	    event->attr.exclude_idle || event->attr.exclude_host ||
	    event->attr.exclude_guest) {
		dev_err(slc_pmon_pmu.dev, "Can't support filtering\n");
		return -EINVAL;
	}

	if (event->cpu == ALWAYS_ON_CPU)
		event->hw.flags |= COUNTER_OWNER;

	event->hw.config = INVALID_COUNTER;

	event->cpu = ALWAYS_ON_CPU;

	return 0;
}

static void slc_pmon_init_debugfs(const char *dir_name)
{
	struct dentry *refresh;

	slc_pmon_dir = debugfs_create_dir(dir_name, NULL);
	if (!slc_pmon_dir) {
		pr_err("slc_pmon debugfs init failed\n");
		return;
	}

	refresh = debugfs_create_file("refresh", 0220, slc_pmon_dir, NULL, &slc_pmon_refresh_fops);
	if (!refresh)
		pr_err("Failed to create slc_pmon refresh node\n");

	return;
}

int slc_pmon_init(struct slc_acpm_driver_data *driver_data,
		  int (*slc_acpm)(struct slc_acpm_driver_data *, unsigned int,
				  unsigned int, unsigned long, uint32_t *))
{
	int res = 0;

	memset(&slc_pmon_state, 0, sizeof(slc_pmon_state));
	acpm_pt.driver_data = driver_data;
	acpm_pt.slc_acpm = slc_acpm;

	mutex_init(&slc_pmon_state.lock);

	res = slc_pmon_counter_init();
	if (res)
		goto counter_cleanup;

	res = slc_pmon_event_init();
	if (res)
		goto event_cleanup;

	sysfs_attr_init(slc_pmon_pmu_format_attrs[0]);

	res = perf_pmu_register(&slc_pmon_pmu, slc_pmon_pmu.name, -1);
	if (res < 0) {
		pr_err("Failed to register PMU driver!");
		goto event_cleanup;
	}

	pr_info("Loaded! (%d counters, %d events).",
		slc_pmon_state.counters.num_counters,
		slc_pmon_state.events.num_events);

	slc_pmon_init_debugfs("slc_pmon");

	return 0;

event_cleanup:
	slc_pmon_event_cleanup();

counter_cleanup:
	slc_pmon_counter_cleanup();

	acpm_pt.driver_data = NULL;

	return res;
}
EXPORT_SYMBOL_GPL(slc_pmon_init);

static int slc_pmon_counter_init(void)
{
	uint32_t acc_size = 0;
	int num_counters = slc_acpm_get_num_counters();

	if (num_counters <= 0) {
		pr_err("Error when fetching number of counters (%d).",
		       num_counters);
		return -EINVAL;
	}
	slc_pmon_state.counters.num_counters = num_counters;

	slc_pmon_state.counters.counters =
		kzalloc(slc_pmon_state.counters.num_counters *
				sizeof(struct slc_pmon_virtual_counter),
			GFP_KERNEL);
	if (!slc_pmon_state.counters.counters) {
		pr_err("Error allocating counters!");
		return -ENOMEM;
	}

	if (acpm_ipc_get_buffer("SLC_PMON",
				&slc_pmon_state.counters.slc_pmon_accumulators,
				&acc_size) < 0) {
		pr_err("Error when retrieving SLC PMON buffer!");
		return -EINVAL;
	}

	if (acc_size != num_counters * sizeof(uint64_t)) {
		pr_err("Unexpected size for SLC PMON buffer!");
		return -EINVAL;
	}

	return 0;
}

static void slc_pmon_counter_cleanup(void)
{
	kfree(slc_pmon_state.counters.counters);
	slc_pmon_state.counters.counters = NULL;
}

static int slc_pmon_event_init(void)
{
	int i;
	int num_events = slc_acpm_get_num_events();

	if (num_events <= 0) {
		pr_err("Error when fetching number of events (%d).",
		       num_events);
		return -EINVAL;
	}
	slc_pmon_state.events.num_events = num_events;

	slc_pmon_state.events.event_names =
		kzalloc(slc_pmon_state.events.num_events * sizeof(const char *),
			GFP_KERNEL);
	if (!slc_pmon_state.events.event_names) {
		pr_err("Error allocating events!");
		return -ENOMEM;
	}

	slc_pmon_state.events.platform_pmu_event_attrs = kzalloc(
		(slc_pmon_state.events.num_events + 1) *
			sizeof(*slc_pmon_state.events.platform_pmu_event_attrs),
		GFP_KERNEL);
	if (!slc_pmon_state.events.platform_pmu_event_attrs) {
		pr_err("Error allocating event attributes!");
		return -ENOMEM;
	}
	slc_pmon_pmu_events_attr_group.attrs =
		slc_pmon_state.events.platform_pmu_event_attrs;

	slc_pmon_state.events.platform_pmu_event_ext_attrs =
		kzalloc((slc_pmon_state.events.num_events) *
				sizeof(*slc_pmon_state.events
						.platform_pmu_event_ext_attrs),
			GFP_KERNEL);
	if (!slc_pmon_state.events.platform_pmu_event_ext_attrs) {
		pr_err("Error allocating extended event attributes!");
		return -ENOMEM;
	}

	for (i = 0; i < slc_pmon_state.events.num_events; i++) {
		slc_pmon_state.events.event_names[i] =
			slc_acpm_get_event_name(i + 1);
		if (!slc_pmon_state.events.event_names[i]) {
			pr_err("Error allocating event name for id %d!",
			       (i + 1));
			return -ENOMEM;
		}
		slc_pmon_state.events.platform_pmu_event_attrs[i] = kzalloc(
			sizeof(*slc_pmon_state.events.platform_pmu_event_attrs),
			GFP_KERNEL);
		if (!slc_pmon_state.events.platform_pmu_event_attrs[i]) {
			pr_err("Error allocating event attribute %d!", i);
			return -ENOMEM;
		}
		slc_pmon_state.events.platform_pmu_event_ext_attrs[i] =
			*SLC_PMON_EVENT_ATTR(
				slc_pmon_state.events.event_names[i], (i + 1));
		slc_pmon_state.events.platform_pmu_event_attrs[i] =
			&slc_pmon_state.events.platform_pmu_event_ext_attrs[i]
				 .attr.attr;
		sysfs_attr_init(slc_pmon_state.events.platform_pmu_event_attrs[i]);
	}

	return 0;
}

static void slc_pmon_event_cleanup(void)
{
	kfree(slc_pmon_state.events.platform_pmu_event_attrs);
	slc_pmon_state.events.platform_pmu_event_attrs = NULL;

	kfree(slc_pmon_state.events.platform_pmu_event_ext_attrs);
	slc_pmon_state.events.platform_pmu_event_ext_attrs = NULL;

	if (slc_pmon_state.events.event_names) {
		int i;
		for (i = 0; i < slc_pmon_state.events.num_events; i++) {
			kfree(slc_pmon_state.events.event_names[i]);
			slc_pmon_state.events.event_names[i] = NULL;
		}
		kfree(slc_pmon_state.events.event_names);
		slc_pmon_state.events.event_names = NULL;
	}
}

void slc_pmon_exit(void)
{
	if (acpm_pt.driver_data) {
		perf_pmu_unregister(&slc_pmon_pmu);
		slc_pmon_event_cleanup();
		slc_pmon_counter_cleanup();
		pr_info("Unloaded!");
	}

	if (slc_pmon_dir)
		debugfs_remove_recursive(slc_pmon_dir);

}
EXPORT_SYMBOL_GPL(slc_pmon_exit);

#endif // IS_ENABLED(CONFIG_SLC_PMON)

MODULE_DESCRIPTION("Perf driver for SLC PMON");
MODULE_AUTHOR("Vincent Palomares <paillon@google.com>");
MODULE_LICENSE("GPL v2");
