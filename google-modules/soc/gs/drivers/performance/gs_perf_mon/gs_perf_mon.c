// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2024 Google LLC.
 *
 * Module for Performance Monitoring.
 */
#define pr_fmt(fmt) "gs_perf_mon: " fmt

#include <linux/cpuidle.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/io.h>
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
#include <linux/perf_event.h>
#include <linux/of_device.h>
#include <linux/mutex.h>
#include <trace/hooks/cpuidle.h>
#include <linux/spinlock.h>
#include <trace/events/power.h>
#include <uapi/linux/sched/types.h>

#include <performance/gs_perf_mon/gs_perf_mon.h>
#include "gs_perf_mon_priv.h"

#define CREATE_TRACE_POINTS
#include "gs_perf_mon_trace.h"

static struct gs_perf_mon_config perf_mon_config;
static struct gs_perf_mon_state perf_mon_metadata;

int gs_perf_mon_get_data(unsigned int cpu, struct gs_cpu_perf_data *data_dest)
{
	int perf_idx;
	int ret = 0;
	struct cpu_perf_info *cpu_data;
	unsigned long flags;

	/* If this function gets called before we probe. */
	if (!perf_mon_metadata.perf_monitor_initialized)
		return -EINVAL;

	cpu_data = &perf_mon_metadata.cpu_data_arr[cpu];

	spin_lock_irqsave(&cpu_data->cpu_perf_lock, flags);

	/* If monitor not active, return error. */
	if (!cpu_data->mon_active) {
		spin_unlock_irqrestore(&cpu_data->cpu_perf_lock, flags);
		return -ENODATA;
	}

	/* Inform caller of monitor status. */
	data_dest->cpu_mon_on = true;

	for (perf_idx = 0; perf_idx < PERF_NUM_COMMON_EVS; perf_idx++) {
		data_dest->perf_ev_last_delta[perf_idx] =
			cpu_data->perf_ev_data[perf_idx].last_delta;
	}

	/* Copy over cpu metadata. */
	data_dest->time_delta_us = cpu_data->time_delta_us;
	data_dest->cpu_idle_state = READ_ONCE(cpu_data->idle_state);

	spin_unlock_irqrestore(&cpu_data->cpu_perf_lock, flags);

	return ret;
}
EXPORT_SYMBOL(gs_perf_mon_get_data);

static int gs_perf_mon_start(void);
static void gs_perf_mon_stop(void);
static void disable_perf_events(int cpu);

/**
 * read_perf_event - Reads PMU or AMU event from current cpu.
 *
 * Inputs:
 * @event:		The perf event to read.
 * @event_total:	Container for read result.
 *
 * Returns:		Non-zero on error.
 */
static inline int read_perf_event(struct gs_event_data *event, u64 *event_total)
{
	int ret = 0;

	if (event->counter_type == PMU) {
		/* Read this event from PMU. */
		ret = perf_event_read_local(event->pevent, event_total, NULL, NULL);
		if (ret) {
			*event_total = 0;
			return ret;
		}
	} else if (event->counter_type == AMU) {
		/* Read this event from AMU. */
		switch (event->element_idx) {
		case PERF_CYCLE_IDX:
			*event_total = read_sysreg_s(SYS_AMEVCNTR0_CORE_EL0);
			break;

		case PERF_INST_IDX:
			*event_total = read_sysreg_s(SYS_AMEVCNTR0_INST_RET_EL0);
			break;

		case PERF_STALL_BACKEND_MEM_IDX:
			*event_total = read_sysreg_s(SYS_AMEVCNTR0_MEM_STALL);
			break;

		default:
			*event_total = 0;
			return -EINVAL;
		}
	}
	/* UNUSED events fallthrough. */

	return 0;
}

/* TODO: b/323458771. Rename to be global namespace appropriate and remove cpu field. */
int read_perf_event_local(int cpu, unsigned int event_id, u64 *count)
{
	struct gs_event_data *event;
	struct cpu_perf_info *cpu_data;
	int mon_active;

	/* Ignoring input cpu parameter. */
	cpu = raw_smp_processor_id();

	if (!perf_mon_metadata.perf_monitor_initialized || event_id >= PERF_NUM_COMMON_EVS)
		return -EINVAL;

	cpu_data = &perf_mon_metadata.cpu_data_arr[cpu];

	spin_lock(&cpu_data->cpu_perf_lock);
	mon_active = cpu_data->mon_active;

	if (!mon_active) {
		spin_unlock(&cpu_data->cpu_perf_lock);
		return -ENODATA;
	}

	event = &cpu_data->perf_ev_data[event_id];
	read_perf_event(event, count);
	spin_unlock(&cpu_data->cpu_perf_lock);

	return 0;
}
EXPORT_SYMBOL(read_perf_event_local);

int gs_perf_mon_add_client(struct gs_perf_mon_client *client)
{
	if (!client)
		return -EINVAL;

	mutex_lock(&perf_mon_metadata.client_list_lock);
	INIT_LIST_HEAD(&client->node);
	list_add(&client->node, &perf_mon_metadata.client_list);
	mutex_unlock(&perf_mon_metadata.client_list_lock);
	return 0;
}
EXPORT_SYMBOL(gs_perf_mon_add_client);

void gs_perf_mon_remove_client(struct gs_perf_mon_client *client)
{
	if (!client)
		return;
	mutex_lock(&perf_mon_metadata.client_list_lock);
	list_del_init(&client->node);
	mutex_unlock(&perf_mon_metadata.client_list_lock);
}
EXPORT_SYMBOL(gs_perf_mon_remove_client);

void gs_perf_mon_tick_update_counters(void)
{
	unsigned int perf_idx;
	unsigned int cpu = raw_smp_processor_id();
	u64 total;
	struct cpu_perf_info *cpu_data;
	struct gs_event_data *ev_data;
	ktime_t now = ktime_get();
	ktime_t last_update_client_ts;
	unsigned long time_delta_us;

	/* If the node is not probed yet, do nothing. */
	if (!perf_mon_metadata.perf_monitor_initialized)
		return;

	cpu_data = &perf_mon_metadata.cpu_data_arr[cpu];

	spin_lock(&cpu_data->cpu_perf_lock);

	/* If this CPU is not monitored, do nothing. */
	if (!cpu_data->mon_active || perf_mon_metadata.is_active == false) {
		spin_unlock(&cpu_data->cpu_perf_lock);
		return;
	}

	time_delta_us = ktime_us_delta(now, cpu_data->last_update_ts);
	cpu_data->ticks_since_update += 1;

	/* Check if its time to poll. */
	if (cpu_data->ticks_since_update >= perf_mon_config.param_ticks_per_counter_update ||
	    time_delta_us > perf_mon_config.param_ticks_per_counter_update * USECS_PER_TICK) {
		/* Loop over all AMU/PMU counters and read them. */
		for (perf_idx = 0; perf_idx < PERF_NUM_COMMON_EVS; perf_idx++) {
			ev_data = &cpu_data->perf_ev_data[perf_idx];
			if (read_perf_event(ev_data, &total)) {
				pr_debug("Perf event read failed on cpu=%u for event_idx=%u",
				cpu, perf_idx);
				continue;
			}
			ev_data->prev_count = ev_data->curr_count;
			ev_data->curr_count = total;
			ev_data->last_delta = ev_data->curr_count - ev_data->prev_count;
		}
		cpu_data->time_delta_us = ktime_us_delta(now, cpu_data->last_update_ts);
		cpu_data->last_update_ts = now;
		cpu_data->ticks_since_update = 0;
	}

	spin_unlock(&cpu_data->cpu_perf_lock);

	/* Check if we need to wakeup backup client handling work. */
	last_update_client_ts = READ_ONCE(perf_mon_metadata.last_client_update_ts);
	time_delta_us = ktime_us_delta(now, last_update_client_ts);
	if (time_delta_us > perf_mon_config.client_update_backup_us)
		wake_up_process(perf_mon_metadata.perf_mon_task);
}
EXPORT_SYMBOL(gs_perf_mon_tick_update_counters);

/**
 * delete_event - Deallocates an event.
 *
 * Input:
 * @event:	The performance event to deallocate.
*/
static void delete_event(struct gs_event_data *event)
{
	if (event->pevent) {
		perf_event_release_kernel(event->pevent);
		event->pevent = NULL;
	}
}

/**
 * init_event - Initialize a single perf event.
 *
 * Input:
 * @event:	The event identifier to allocate.
 * @cpu:	The cpu to allocate this event on.
 *
 * Returns:	Non-zero on error.
*/
static int init_event(struct gs_event_data *event, unsigned int cpu)
{
	struct perf_event *pevent;
	unsigned int event_id = event->raw_event_id;
	struct perf_event_attr attr;

	attr.type = PERF_TYPE_RAW;
	attr.size = sizeof(struct perf_event_attr);
	attr.pinned = 1;
	attr.exclude_idle = 0;
	attr.config = event_id;


	/* The following allocation steps are only needed for PMU events. */
	if (event->counter_type == PMU) {
		/* Allocate the event from kernel. */
		pevent = perf_event_create_kernel_counter(&attr, cpu, NULL, NULL, NULL);
		if (IS_ERR(pevent))
			return PTR_ERR(pevent);

		event->pevent = pevent;

		/* Enable the event. */
		perf_event_enable(pevent);
	}
	return 0;
}

/**
 * enable_perf_events - Enables all pmu events on a cpu.
 *
 * Input:
 * @cpu:	Which CPU to allocate events for?
 *
 * Returns:	Non-zero on error.
 *
 * Synchronization: Must have perf_allocation_lock held.
*/
static int enable_perf_events(unsigned int cpu)
{
	struct cpu_perf_info *cpu_data = &perf_mon_metadata.cpu_data_arr[cpu];
	struct gs_event_data *ev_data;
	unsigned int perf_idx;
	int ret = 0;
	unsigned long flags;

	/* Loop on all events and initialize them.*/
	for (perf_idx = 0; perf_idx < PERF_NUM_COMMON_EVS; perf_idx++) {
		ev_data = &cpu_data->perf_ev_data[perf_idx];
		ret = init_event(ev_data, cpu);
		if (WARN_ON(ret))
			goto err_init;
	}

	spin_lock_irqsave(&cpu_data->cpu_perf_lock, flags);
	cpu_data->mon_active = true;
	spin_unlock_irqrestore(&cpu_data->cpu_perf_lock, flags);
	return 0;

err_init:
	/* On error of enabling any event, deallocate all events. */
	disable_perf_events(cpu);
	return ret;
}

/**
 * disable_perf_events - Disables all pmu events on a cpu.
 *
 * Input:
 * @cpu:	Which CPU to disable events for.
 *
 * Synchronization: Caller must hold perf_allocation_lock.
*/
static void disable_perf_events(int cpu)
{
	struct cpu_perf_info *cpu_data = &perf_mon_metadata.cpu_data_arr[cpu];
	struct gs_event_data *ev_data;
	unsigned int perf_idx;
	unsigned long flags;

	spin_lock_irqsave(&cpu_data->cpu_perf_lock, flags);
	cpu_data->mon_active = false;
	spin_unlock_irqrestore(&cpu_data->cpu_perf_lock, flags);
	for (perf_idx = 0; perf_idx < PERF_NUM_COMMON_EVS; perf_idx++) {
		ev_data = &cpu_data->perf_ev_data[perf_idx];
		delete_event(ev_data);
	}
}

/**
 * vendor_update_event_cpu_idle_enter - idle hook.
 *
 * Vendor hook for entering CPU idle.
 *
 * Input:
 * @data:	Unused.
 * @state:	The idle state this cpu transitioned to.
 * @dev:	Unused.
 *
 * Output:	Sets idle_state to:
 *		0 = PERF_CPU_IDLE_C1, 1 = PERF_CPU_IDLE_C2
*/
static void vendor_update_event_cpu_idle_enter(void *data, int *state, struct cpuidle_device *dev)
{
	unsigned int cpu = raw_smp_processor_id();
	struct cpu_perf_info *cpu_data = &perf_mon_metadata.cpu_data_arr[cpu];
	int idle_state = *state;

	if (idle_state == 0)
		WRITE_ONCE(cpu_data->idle_state, PERF_CPU_IDLE_C1);
	else if (idle_state == 1)
		WRITE_ONCE(cpu_data->idle_state, PERF_CPU_IDLE_C2);
}

/**
 * vendor_update_event_cpu_idle_exit - idle exit hook.
 *
 * Sets idle_state to PERF_CPU_ACTIVE to indicate busy.
*/
static void vendor_update_event_cpu_idle_exit(void *data, int state, struct cpuidle_device *dev)
{
	unsigned int cpu = raw_smp_processor_id();
	struct cpu_perf_info *cpu_data = &perf_mon_metadata.cpu_data_arr[cpu];
	WRITE_ONCE(cpu_data->idle_state, PERF_CPU_ACTIVE);
}

/**
 * gs_perf_mon_update_clients - Main function for updating clients.
 *
 * This is the function loops over client_list and services
 * their callbacks on a fixed interval.
 *
*/
void gs_perf_mon_update_clients(void)
{
	unsigned int cpu;
	struct gs_perf_mon_client *curr_client;
	ktime_t last_update_client_ts;
	unsigned long delta_us;
	ktime_t now = ktime_get();
	int ret = 0;

	if (!perf_mon_metadata.perf_monitor_initialized)
		return;

	/* Update the current cpu's performance counters. */
	if (mutex_trylock(&perf_mon_metadata.client_list_lock)) {
		/* Update last client service update ts and metadata. */
		last_update_client_ts = READ_ONCE(perf_mon_metadata.last_client_update_ts);
		delta_us = ktime_us_delta(now, last_update_client_ts);

		/* For spurious or stale wakeups. */
		if (!perf_mon_metadata.is_active ||
		    delta_us < perf_mon_config.client_update_interval_us) {
			mutex_unlock(&perf_mon_metadata.client_list_lock);
			return;
		}

		WRITE_ONCE(perf_mon_metadata.last_client_update_ts, now);

		/* Copy over all the performance information for all cpus. */
		for_each_possible_cpu (cpu) {
			ret = gs_perf_mon_get_data(cpu, &perf_mon_metadata.client_shared_data[cpu]);
			if (ret)
				perf_mon_metadata.client_shared_data[cpu].cpu_mon_on = false;
			trace_gs_perf_mon(cpu, &perf_mon_metadata.client_shared_data[cpu]);
		}

		/* Update all clients supplying a callback pointer to monitor data. */
		list_for_each_entry (curr_client, &perf_mon_metadata.client_list, node) {
			if (curr_client->client_callback)
				curr_client->client_callback(perf_mon_metadata.client_shared_data,
							     curr_client->private_data);
		}
		mutex_unlock(&perf_mon_metadata.client_list_lock);
	}
}
EXPORT_SYMBOL(gs_perf_mon_update_clients);

/**
 * gs_perf_mon_cpuhp_up - re-enables perf monitoring on cpu-up.
 *
 * Input:
 * @cpu:	Whichever CPU just came back online.
*/
static int gs_perf_mon_cpuhp_up(unsigned int cpu)
{
	int ret = 0;
	int mon_active;
	unsigned long flags;
	struct cpu_perf_info *cpu_data = &perf_mon_metadata.cpu_data_arr[cpu];

	mutex_lock(&cpu_data->perf_allocation_lock);
	spin_lock_irqsave(&cpu_data->cpu_perf_lock, flags);
	mon_active = cpu_data->mon_active;
	spin_unlock_irqrestore(&cpu_data->cpu_perf_lock, flags);

	/* Do nothing if already on. */
	if (mon_active) {
		mutex_unlock(&cpu_data->perf_allocation_lock);
		return 0;
	}

	/* Enable the events. */
	ret = enable_perf_events(cpu);
	mutex_unlock(&cpu_data->perf_allocation_lock);

	/*
	 * If we were unable to restart the performance events, we
	 * should stop the perf monitor entirely.
	 */
	if (WARN_ON(ret))
		gs_perf_mon_stop();
	return ret;
}

/**
 * gs_perf_mon_cpuhp_down - disables perf monitoring on cpu-down.
 *
 * Input:
 * @cpu:	Whichever CPU just went down.
*/
static int gs_perf_mon_cpuhp_down(unsigned int cpu)
{
	struct cpu_perf_info *cpu_data = &perf_mon_metadata.cpu_data_arr[cpu];
	mutex_lock(&cpu_data->perf_allocation_lock);
	disable_perf_events(cpu);
	mutex_unlock(&cpu_data->perf_allocation_lock);

	return 0;
}

/* Initializes CPU hotplugs*/
static int gs_init_perf_mon_cpuhp(void)
{
	int ret = 0;
	ret = cpuhp_setup_state_nocalls(CPUHP_AP_ONLINE_DYN, "gs_perf_mon", gs_perf_mon_cpuhp_up,
					gs_perf_mon_cpuhp_down);
	if (ret < 0)
		pr_err("init cpuhp fail:%d\n", ret);

	return ret;
}

/**
 * gs_perf_mon_start - Start and perf monitoring.
 *
 * This function will allocate PMU units and queue
 * monitoring work.
*/
static int gs_perf_mon_start(void)
{
	unsigned int cpu;
	struct cpu_perf_info *cpu_data;
	int ret = 0;

	mutex_lock(&perf_mon_metadata.active_state_lock);
	if (perf_mon_metadata.is_active)
		goto unlock_out;

	/* Allocate and enable perf events. */
	for_each_possible_cpu (cpu) {
		cpu_data = &perf_mon_metadata.cpu_data_arr[cpu];
		mutex_lock(&cpu_data->perf_allocation_lock);
		ret = enable_perf_events(cpu);
		mutex_unlock(&cpu_data->perf_allocation_lock);
		if (ret)
			goto unlock_out;
	}

	perf_mon_metadata.is_active = true;
unlock_out:
	mutex_unlock(&perf_mon_metadata.active_state_lock);
	return ret;
}

/**
 * gs_perf_mon_stop - Stop perf monitoring.
 *
 * This function will deallocate PMU units and cancel
 * monitor work.
*/
static void gs_perf_mon_stop(void)
{
	unsigned int cpu;
	struct cpu_perf_info *cpu_data;
	mutex_lock(&perf_mon_metadata.active_state_lock);

	/* If the monitor is already disabled. */
	if (!perf_mon_metadata.is_active)
		goto unlock_out;

	perf_mon_metadata.is_active = false;
	for_each_possible_cpu (cpu) {
		/* Deallocate all the perf events. */
		cpu_data = &perf_mon_metadata.cpu_data_arr[cpu];
		mutex_lock(&cpu_data->perf_allocation_lock);
		disable_perf_events(cpu);
		mutex_unlock(&cpu_data->perf_allocation_lock);
	}
unlock_out:
	mutex_unlock(&perf_mon_metadata.active_state_lock);
}

/* Helper function for parsing a single perf event. */
static int parse_perf_event(struct device_node *counter_type, struct gs_event_data *cpu_event,
			    enum gs_counter_type unit_id, enum gs_perf_event_idx ev_idx, char *prop_name)
{
	int ret = 0;
	unsigned int event_id;
	ret = of_property_read_u32(counter_type, prop_name, &event_id);
	if (ret) {
		return -EINVAL;
	} else {
		cpu_event->counter_type = unit_id;
		cpu_event->raw_event_id = event_id;
		cpu_event->element_idx = ev_idx;
	}
	return ret;
}

/**
 * parse_perf_counters - Populates the values for the performance unit.
 *
 * Inputs:
 * @dev:		The device for error messages.
 * @counter_type:	The perf node.
 * @cpu_events:		Container for parsed data.
 * @unit_id:		Processing AMU or PMU?
 *
 * Returns:		Non-zero if any perf event not allocateable.
*/
static int parse_perf_counters(struct device *dev, struct device_node *counter_type,
			       struct gs_event_data *cpu_events, enum gs_counter_type unit_id)
{
	int ret = 0;

	ret = parse_perf_event(counter_type, &cpu_events[PERF_L2D_CACHE_REFILL_IDX], unit_id,
			       PERF_L2D_CACHE_REFILL_IDX, "l2-cachemiss-ev");
	if (ret)
		dev_dbg(dev, "l2-cachemiss-ev event not specified. Skipping.\n");

	ret = parse_perf_event(counter_type, &cpu_events[PERF_STALL_BACKEND_MEM_IDX], unit_id,
			       PERF_STALL_BACKEND_MEM_IDX, "stall-backend-mem-ev");
	if (ret)
		dev_dbg(dev, "stall-backend-mem-event not specified. Skipping.\n");

	ret = parse_perf_event(counter_type, &cpu_events[PERF_INST_IDX], unit_id, PERF_INST_IDX,
			       "inst-ev");
	if (ret)
		dev_dbg(dev, "inst-ev not specified. Skipping.\n");

	ret = parse_perf_event(counter_type, &cpu_events[PERF_CYCLE_IDX], unit_id, PERF_CYCLE_IDX,
			       "cyc-ev");
	if (ret)
		dev_dbg(dev, "cyc-ev not specified. Skipping.\n");

	ret = parse_perf_event(counter_type, &cpu_events[PERF_L3_CACHE_MISS_IDX], unit_id,
			       PERF_L3_CACHE_MISS_IDX, "l3-cachemiss-ev");
	if (ret)
		dev_dbg(dev, "l3-cachemiss-ev not specified. Skipping.\n");

	return 0;
}

/**
 * initialize_cpu_data_info
 *
 * Inputs:
 * @dev:	Device for error reporting and node reading.
 * @cpu_node:	The individual CPU node to populate.
 * @cpu_data:	The container for the CPU data.
 *
 * Returns:	Non-zero on error.
*/
static int initialize_cpu_data_info(struct device *dev, struct device_node *cpu_node,
				    struct cpu_perf_info *cpu_data)
{
	struct device_node *pmu_node;
	struct device_node *amu_node;
	struct gs_event_data *cpu_event_data = cpu_data->perf_ev_data;
	unsigned int event_idx;
	int ret = 0;

	spin_lock_init(&cpu_data->cpu_perf_lock);
	mutex_init(&cpu_data->perf_allocation_lock);

	/* Default events to uninitialized. */
	for (event_idx = 0; event_idx < PERF_NUM_COMMON_EVS; event_idx++)
		cpu_event_data[event_idx].counter_type = UNUSED;

	/* Find and populate the pmu data. */
	pmu_node = of_get_child_by_name(cpu_node, "pmu_events");
	ret = parse_perf_counters(dev, pmu_node, cpu_event_data, PMU);
	if (ret) {
		dev_err(dev, "Couldn't parse pmu_node, skipping performance monitoring.\n");
		return ret;
	}

	/* Find and populate the amu data. */
	amu_node = of_get_child_by_name(cpu_node, "amu_events");
	ret = parse_perf_counters(dev, amu_node, cpu_event_data, AMU);
	if (ret) {
		dev_err(dev, "Couldn't parse amu_node, skipping performance monitoring.\n");
		return ret;
	}

	return ret;
}

/**
 * gs_perf_mon_parse_dt - Main parse function for the perf monitor.
 */
static int gs_perf_mon_parse_dt(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct cpu_perf_info *cpu_data;

	/* The node we currently have. */
	struct device_node *cpus_data_np;

	/* Containers for children nodes. */
	struct device_node *cpu_node;
	unsigned int cpu_idx;
	unsigned int num_children;
	int ret = 0;

	/* Populate metadata. */
	if (ret) {
		dev_err(dev, "num_cpu invalid, skipping performance monitoring.\n");
		return -EINVAL;
	}

	/* Set monitor's performance counter update interval in us. */
	ret = of_property_read_u32(dev->of_node, "param_ticks_per_counter_update",
				   &perf_mon_config.param_ticks_per_counter_update);
	if (ret) {
		dev_err(dev, "param_ticks_per_counter_update unspecified, using default value.\n");
		ret = 0;
		perf_mon_config.param_ticks_per_counter_update = DEFAULT_TICKS_PER_COUNTER_UPDATE;
	}

	/* Set monitor's tick client update interval in us. */
	ret = of_property_read_u32(dev->of_node, "client_update_backup_us",
				   &perf_mon_config.client_update_backup_us);
	if (ret) {
		dev_err(dev, "client_update_backup_us unspecified, using default value.\n");
		ret = 0;
		perf_mon_config.client_update_backup_us =
			DEFAULT_TICKS_PER_COUNTER_UPDATE * USECS_PER_TICK;
	}

	/* Set monitor's opportunistic client update interval in us. */
	ret = of_property_read_u32(dev->of_node, "client_update_interval_us",
				   &perf_mon_config.client_update_interval_us);
	if (ret) {
		dev_err(dev, "client_update_interval_us unspecified, using default value.\n");
		ret = 0;
		perf_mon_config.client_update_interval_us =
			DEFAULT_TICKS_PER_COUNTER_UPDATE * USECS_PER_TICK;
	}

	/* Allocate memory for cpu state representations. */
	perf_mon_metadata.cpu_data_arr =
		devm_kzalloc(dev, sizeof(struct cpu_perf_info) * num_possible_cpus(), GFP_KERNEL);
	if (!perf_mon_metadata.cpu_data_arr) {
		dev_err(dev, "Insufficient memory for cpu_data_arr. Aborting\n");
		return -ENOMEM;
	}

	/* Populate the cpu state representations. */
	cpu_idx = 0;
	cpu_node = NULL;
	cpus_data_np = of_get_child_by_name(dev->of_node, "gs_perf_cpu");
	if (!cpus_data_np) {
		dev_err(dev, "gs_perf_cpu invalid skipping performance monitoring.\n");
		ret = -EINVAL;
		goto err_probe;
	}

	num_children = of_get_child_count(cpus_data_np);

	/* Loop over all children nodes we account for. */
	while ((cpu_node = of_get_next_child(cpus_data_np, cpu_node)) != NULL) {
		/* Find the CPU index. */
		ret = of_property_read_u32(cpu_node, "cpu_idx", &cpu_idx);
		if (ret || cpu_idx > num_possible_cpus() - 1) {
			dev_err(dev, "cpu_idx invalid, skipping performance monitoring.\n");
			ret = -EINVAL;
			goto err_probe;
		}

		/* Retrieve probe target. */
		cpu_data = &perf_mon_metadata.cpu_data_arr[cpu_idx];
		cpu_data->cpu_id = cpu_idx;

		/* Initialize cpu data. */
		ret = initialize_cpu_data_info(dev, cpu_node, cpu_data);
		if (ret)
			goto err_probe;
	}

	return 0;

err_probe:
	kfree(perf_mon_metadata.cpu_data_arr);
	return ret;
}

/* Thread for servicing the latency governors.*/
static int perf_mon_task(void *data)
{
	while (!kthread_should_stop()) {
		set_current_state(TASK_INTERRUPTIBLE);

		/*
		 * Put this thread to sleep and wait
		 * for a wakeup from timer or scheduler.
		 *
		 * TODO: Try wait_on bits (b/323458771)
		 */
		schedule();
		set_current_state(TASK_RUNNING);
		gs_perf_mon_update_clients();
	}
	return 0;
}

/* Driver initialization code. */
int gs_perf_mon_driver_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int ret = 0;

	ret = gs_perf_mon_parse_dt(pdev);
	if (ret)
		return ret;

	/* Allocate and schedule the perf_mon task. */
	perf_mon_metadata.perf_mon_task =
		kthread_create(perf_mon_task, NULL, "perf_mon_update_client_task");
	if (IS_ERR(perf_mon_metadata.perf_mon_task)) {
		pr_err("%s: failed kthread_create for perf_mon \n", __func__);
		ret = PTR_ERR(perf_mon_metadata.perf_mon_task);
		return ret;
	}
	sched_set_fifo(perf_mon_metadata.perf_mon_task);

	/* Allocate container for client's shared data. */
	perf_mon_metadata.client_shared_data = devm_kzalloc(
		dev, sizeof(struct gs_cpu_perf_data) * num_possible_cpus(), GFP_KERNEL);
	if (!perf_mon_metadata.client_shared_data) {
		ret = -ENOMEM;
		goto err_client_data;
	}

	/* Register hooks. */
	ret = register_trace_android_vh_cpu_idle_enter(vendor_update_event_cpu_idle_enter, NULL);
	if (ret) {
		dev_err(dev, "Register idle enter vendor hook fail %d\n", ret);
		goto err_vh_idle_enter_register;
	}
	ret = register_trace_android_vh_cpu_idle_exit(vendor_update_event_cpu_idle_exit, NULL);
	if (ret) {
		dev_err(dev, "Register idle exit vendor hook fail %d\n", ret);
		goto err_vh_idle_exit_register;
	}

	/* Register cpu hotplugs. */
	ret = gs_init_perf_mon_cpuhp();
	if (ret < 0) {
		dev_err(dev, "gs_init_perf_mon_cpuhp errored with number %d\n", ret);
		goto err_cpuhp_init;
	}

	/* Start the perf monitor. */
	ret = gs_perf_mon_start();
	if (ret) {
		dev_err(dev, "gs_perf_mon could not stop with error code %d\n", ret);
		goto err_cpuhp_init;
	}
	perf_mon_metadata.perf_monitor_initialized = true;
	return 0;

/* If any of the above steps failed, we need to free resources and unregister hooks. */
err_cpuhp_init:
	unregister_trace_android_vh_cpu_idle_exit(vendor_update_event_cpu_idle_exit, NULL);
err_vh_idle_exit_register:
	unregister_trace_android_vh_cpu_idle_enter(vendor_update_event_cpu_idle_enter, NULL);
err_vh_idle_enter_register:
err_client_data:
	kthread_stop(perf_mon_metadata.perf_mon_task);
	return ret;
}

static const struct of_device_id gs_perf_mon_root_match[] = {
	{
		.compatible = "google,gs_perf_mon",
	},
	{}
};
MODULE_DEVICE_TABLE(of, gs_perf_mon_root_match);

static struct platform_driver gs_perf_mon_platform_driver = {
	.probe = gs_perf_mon_driver_probe,
	.driver = {
		.name = "gs_perf_mon",
		.owner = THIS_MODULE,
		.of_match_table = gs_perf_mon_root_match,
		.suppress_bind_attrs = true,
	},
};

/* Driver initialization step for lists and locks. */
static int __init gs_perf_mon_init(void)
{
	int ret = 0;
	mutex_init(&perf_mon_metadata.client_list_lock);
	mutex_init(&perf_mon_metadata.active_state_lock);
	INIT_LIST_HEAD(&perf_mon_metadata.client_list);

	ret = platform_driver_register(&gs_perf_mon_platform_driver);
	if (ret)
		pr_err("Error when registering driver!\n");

	return ret;
}

/* Driver exit step to stop monitor if module exits. */
static void __exit gs_perf_mon_exit(void)
{
	gs_perf_mon_stop();
}

/* A module parameter for frequency of counter updates. */
static int gs_perf_mon_param_set_ticks(const char *val, const struct kernel_param *kp)
{
	unsigned int ticks = 0;

	if (kstrtoint(val, 10, &ticks)) {
		pr_err("%s: gs_perf_mon parse error", __func__);
		return -EINVAL;
	}

	if (ticks < 1 || ticks > 32) {
		pr_err("%s: gs_perf_mon invalid number of ticks", __func__);
		return -EINVAL;
	}

	perf_mon_config.param_ticks_per_counter_update = ticks;
	perf_mon_config.client_update_interval_us = ticks * USECS_PER_TICK;
	perf_mon_config.client_update_backup_us = ticks * USECS_PER_TICK + USECS_PER_TICK / 2;

	return 0;
}

static int gs_perf_mon_param_get_ticks(char *buf, const struct kernel_param *kp)
{
	return sysfs_emit_at(buf, 0, "%u\n", perf_mon_config.param_ticks_per_counter_update);
}

static const struct kernel_param_ops param_tick = {
	.set = gs_perf_mon_param_set_ticks,
	.get = gs_perf_mon_param_get_ticks,
};

module_param_cb(gs_perf_mon_param_ticks, &param_tick, NULL, 0644);

static int gs_perf_mon_param_set_active(const char *val, const struct kernel_param *kp)
{
	bool is_active;

	if (kstrtobool(val, &is_active)) {
		pr_err("%s: gs_perf_mon parse error", __func__);
		return -EINVAL;
	}

	if (is_active)
		gs_perf_mon_start();
	else
		gs_perf_mon_stop();

	return 0;
}

static int gs_perf_mon_param_get_active(char *buf, const struct kernel_param *kp)
{
	return sysfs_emit_at(buf, 0, "%u\n", perf_mon_metadata.is_active);
}

static const struct kernel_param_ops param_is_active = {
	.set = gs_perf_mon_param_set_active,
	.get = gs_perf_mon_param_get_active,
};

module_param_cb(gs_perf_mon_param_on, &param_is_active, NULL, 0644);

module_init(gs_perf_mon_init);
module_exit(gs_perf_mon_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Google Performance Monitor");
MODULE_AUTHOR("Will Song <jinpengsong@google.com>");
