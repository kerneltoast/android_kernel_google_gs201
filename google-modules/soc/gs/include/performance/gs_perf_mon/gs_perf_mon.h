// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright 2024 Google, Inc.
 *
 * Performance Monitor for Google Pixel.
 */

#ifndef _GS_PERF_MON_H_
#define _GS_PERF_MON_H_

/**
 * gs_perf_event_idx - A type for performance events.
 *
 * We expect a particular event to have
 * the same index across all ids and data arrays.
*/
enum gs_perf_event_idx {
	PERF_L2D_CACHE_REFILL_IDX,
	PERF_STALL_BACKEND_MEM_IDX,
	PERF_L3_CACHE_MISS_IDX,
	PERF_INST_IDX,
	PERF_CYCLE_IDX,
	PERF_NUM_COMMON_EVS
};

/**
 * gs_perf_cpu_idle_state - Enum for idle states.
 *
 * Used to supply clients cpu idle information.
*/
enum gs_perf_cpu_idle_state {
	PERF_CPU_ACTIVE,
	PERF_CPU_IDLE_C1,
	PERF_CPU_IDLE_C2
};

/**
 * gs_counter_type - AMU or PMU or UNUSED
 *
 * Different CPUs and platforms may choose between AMU or PMU
 * for a performance event (or UNUSED). So, for each event,
 * we store the source monitoring unit.
*/
enum gs_counter_type {
	UNUSED, /* Unused Performance Counter. */
	PMU,	/* Performance Monitoring Unit. */
	AMU	/* Activity Monitoring Unit. */
};

/**
 * struct gs_event_data - The representation of a perf event.
 *
 * Each performance counter on each CPU will have an associated
 * gs_event_data with it. This struct provides all the information
 * for the perf event including what the last values are and how
 * to read the event.
 *
 * @pevent:		The underlying event structure.
 * @element_idx:	The index of this event. Indicator for what this
 *			event *should* be. Useful for AMU reading.
 * @counter_type:	Is this event from the PMU or AMU?
 * @raw_event_id:	The hardware event id associated with this event.
 * @curr_count:		Total event count.
 * @prev_count:		The last total event count.
 * @last_delta:		The difference between curr_count and prev_count.
 */
struct gs_event_data {
	struct perf_event *pevent;
	enum gs_perf_event_idx element_idx;
	enum gs_counter_type counter_type;
	unsigned int raw_event_id;
	unsigned long prev_count;
	unsigned long curr_count;
	unsigned long last_delta;
};

/**
 * struct gs_cpu_perf_data - Container for per-cpu counter data.
 *
 * This struct provides the interface for clients desiring CPU profiling
 * data.
 *
 * @cpu_mon_on:		Is this cpu being monitored?
 * @last_update_ts:	The last time the counters were updated.
 * @time_delta_us:	Duration of last counter update.
 * @perf_ev_last_delta:	Counts for performance events.
 * @cpu_idle_state:	Current cpu_idle state.
 */
struct gs_cpu_perf_data {
	bool cpu_mon_on;
	ktime_t last_update_ts;
	unsigned long cpu_freq;
	unsigned long time_delta_us;
	unsigned long perf_ev_last_delta[PERF_NUM_COMMON_EVS];
	enum gs_perf_cpu_idle_state cpu_idle_state;
};

/**
 * gs_perf_mon_callback_func_t - A type for client callbacks.
 *
 * @gs_cpu_perf_data_arr:	We supply a copy of all CPU cores
 * @private_data:		Private data for the client metadata.
 */
typedef void (*gs_perf_mon_callback_func_t)(struct gs_cpu_perf_data* gs_cpu_perf_data_arr,
						void *private_data);

/**
 * struct gs_perf_mon_client - Callback data for a client.
 *
 * the client representation in monitor. Used to invoke callbacks to service
 * clients.
 *
 * @node:		The list entry for this client.
 * @name:		Identifier for the client.
 * @client_callback:	The client's callback function to be called.
 */
struct gs_perf_mon_client {
	struct list_head node;
	const char *name;
	void* private_data;
	gs_perf_mon_callback_func_t client_callback;
};

#if IS_ENABLED(CONFIG_GS_PERF_MON)

/***
 * gs_perf_mon_get_data - Primary function for retrieving cpu performance data.
 *
 * Inputs:
 * @cpu:	Which CPU's data should we supply?
 * @data_dest:	A location to store the perf data
 *
 * Returns:	0 on success and Populates data_dest.
 * 		-EINVAL if the monitor is not active.
*/
int gs_perf_mon_get_data(unsigned int cpu, struct gs_cpu_perf_data *data_dest);

/**
 * read_perf_event_local - Reading function for total count of a single event.
 *
 * REQUIREMENT: IRQs must be disabled.
 *
 * This function should be called with IRQs disabled since the calling
 * task could migrate to another CPU and return an unexpected result.
 *
 * Inputs:
 * @cpu:	Should be ignored, this function must be called on the cpu
 * 		the caller is running from.
 * @event_id:	Which event to read?
 * @count:	Place to store the total event count.
 * Returns:	0 if read is successful. Stores result in count.
 * 		-EINVAL if the monitor is not active
*/
int read_perf_event_local(int cpu, unsigned int event_id, u64 *count);

/**
 * gs_perf_mon_add_client - Registers a client for the perf monitor to service.
 *
 * Input:
 * @client:		Data to be added to a serviced list.
 *
 * Returns:		0 on success
 *
 * Side-Effects:	May call gs_perf_mon_start on first client.
*/
int gs_perf_mon_add_client(struct gs_perf_mon_client *client);

/**
 * gs_perf_mon_remove_client - Unregisters a client for the perf monitor to service.
 *
 * Input:
 * @client:		Client to be removed from the list.
 *
 * Side-Effects:	Stops counter collection when the last client gets deregistered.
*/
void gs_perf_mon_remove_client(struct gs_perf_mon_client *client);

/**
 * gs_perf_mon_tick_update_counters - Updates performance counters and triggers
 *				      latency governor servicing.
 *
 * Requirements:		IRQs must be off on local cpu.
*/
void gs_perf_mon_tick_update_counters(void);

/**
 * gs_perf_mon_update_clients - Checks and updates monitor clients.
 *
 * Side-Effects:	Could call client servicing function.
*/
void gs_perf_mon_update_clients(void);

#else

/* No-op functions if monitor is unused.*/
static inline int gs_perf_mon_get_data(unsigned int cpu, struct gs_cpu_perf_data *data_dest)
{
	return 0;
}
static inline int read_perf_event_local(int cpu, unsigned int event_id, u64 *count)
{
	return 0;
}
static inline int gs_perf_mon_add_client(struct gs_perf_mon_client *client)
{
	return 0;
}
static inline void gs_perf_mon_remove_client(struct gs_perf_mon_client *client)
{
	return;
}
static inline void gs_perf_mon_tick_update_counters(void)
{
	return;
}
static inline void gs_perf_mon_update_clients(void)
{
	return;
}

#endif

#endif // _GS_PERF_MON_H_