// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright 2024 Google, Inc.
 *
 * Private Header for Google Pixel Performance Monitor.
 */

#ifndef _GS_PERF_MON_PRIVATE_H_
#define _GS_PERF_MON_PRIVATE_H_

/* Num ticks between perf counter updates. */
#define DEFAULT_TICKS_PER_COUNTER_UPDATE 2

/* How many microseconds per arch_timer tick. */
#define USECS_PER_TICK (1000000 / CONFIG_HZ)

/**
 * struct cpu_perf_info - Internal container for per-cpu counter data.
 * @cpu_id:			The identifier for this cpu.
 * @perf_allocation_lock:	Syncs allocation and deallocation of perf events.
 * @idle_state:			The idle state of the CPU.
 * @cpu_perf_lock:		Syncs access to perf_ev_data, last_update_ts,
 * 				ticks_since_update, and mon_active.
 *
 * @mon_active:			Is the monitor servicing this CPU?
 * @time_delta_us:		Delta between current perf count and last perf count.
 * @last_update_ts:		Time since last perf update.
 * @ticks_since_update:		Number of ticks since last perf update.
 * @perf_ev_data:		Internal per-cpu perf event containers.
 */
struct cpu_perf_info {
	int cpu_id;
	struct mutex perf_allocation_lock;
	enum gs_perf_cpu_idle_state idle_state;

	spinlock_t cpu_perf_lock; /* This lock protects the below. */
	bool mon_active;
	unsigned long time_delta_us;
	ktime_t last_update_ts;
	unsigned int ticks_since_update;
	struct gs_event_data perf_ev_data[PERF_NUM_COMMON_EVS];
};

/**
 * struct gs_perf_mon_state - Container for Monitor Metadata.
 *
 * This struct contains internal gs_perf_mon meta data
 * It should be not be exposed to files outside of gs_perf_mon.
 *
 * @is_active:			Is the monitor enabled?
 * @perf_monitor_initialized:	Has the monitor initialized?
 * @active_state_lock:		Lock for tuning on/off the Monitor.
 * @last_client_update_ts:	Last time the clients were updated.
 * @client_list:		List of clients to service, num clients, and lock.
 * @client_list_lock:		Spin-lock for the client_list.
 * @client_shared_data:		Performance data to supply to clients.
 * @perf_mon_task:		Kernel thread servicing the clients.
 * @cpu_data_arr:		Array of per-cpu performance data.
 */
struct gs_perf_mon_state {
	bool is_active;
	bool perf_monitor_initialized;
	struct mutex active_state_lock;
	ktime_t last_client_update_ts;
	struct list_head client_list;
	struct mutex client_list_lock;
	struct gs_cpu_perf_data *client_shared_data;
	struct task_struct *perf_mon_task;
	struct cpu_perf_info *cpu_data_arr;
};

/**
 * struct gs_perf_mon_configs - Container for Monitor Configuration Data.
 *
 * This struct contains configuration data for the monitor.
 *
 * @client_update_backup_us:		Backup client interval from the tick.
 * @client_update_interval_us:		Opportunistic update interval for clients.
 * @param_ticks_per_counter_update:	Module parameter for how many ticks per monitor
 * 					data update.
 */
struct gs_perf_mon_config {
	unsigned int client_update_backup_us;
	unsigned int client_update_interval_us;
	unsigned int param_ticks_per_counter_update;
};

#endif // _GS_PERF_MON_PRIVATE_H_