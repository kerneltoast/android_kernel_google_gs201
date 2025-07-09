/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * slc_pmon_internal.h
 *
 * SLC PMON counter management (internal header file).
 *
 * Copyright 2020 Google LLC
 *
 * Author: paillon@google.com
 */

#ifndef __GOOGLE_SLC_PMON_INTERNAL_H__
#define __GOOGLE_SLC_PMON_INTERNAL_H__

#include <linux/mutex.h>

// APM SLC command codes for PMON interface.
enum slc_pmon_command {
	SLC_PMON_QUERY_NUM_EVENTS = 0x20, // [] -> [num advertised events]
	SLC_PMON_QUERY_EVENT = 0x21, // [event id] -> [config info]
	SLC_PMON_POLL_COUNTERS = 0x22, // [] -> [], refresh accumulators
	SLC_PMON_ALLOCATE_COUNTER = 0x23, // [event id] -> [counter id]
	SLC_PMON_RELEASE_COUNTER = 0x24, // [counter id] -> [true|false]
	SLC_PMON_QUERY_NUM_COUNTERS = 0x27, // [] -> [num supported counters]
};

// CPU that never goes offline, and whose associated perf_events will be
// in exclusive charge of performing counter accesses.
#define ALWAYS_ON_CPU 0

// Tag for perf_events actually associated to a HW counter.
// This is a workaround for the fact that, for all desired events, a
// perf_event structure is instantiated for each CPU.
// SLC PMON metrics are SOC-wide (i.e. not per CPU), so only one of those
// needs to do any work.
#define COUNTER_OWNER 0x1

#define INVALID_COUNTER (-1)

// Information and callbacks from the main PT module, necessary to communicate
// with the APM.
struct pt_acpm_closure {
	struct slc_acpm_driver_data *driver_data;
	int (*slc_acpm)(struct slc_acpm_driver_data *, unsigned int,
			unsigned int, unsigned long, uint32_t *);
};

// The APM counter id is implied by the index of the virtual counter.
struct slc_pmon_virtual_counter {
	// Total event counts for the counter since the APM was initialized
	// (across collections and event reconfigurations).
	atomic64_t total_count;
};

// Structure keeping track of counters' availability and state.
struct slc_pmon_counters {
	// Number of counters advertised by the APM.
	int num_counters;
	// Number of counters currently used by driver. (Is this field even used? If so, why?)
	int used_counters;
	// Counter state for each advertised counter.
	struct slc_pmon_virtual_counter *counters;
	// Pointer to shared memory structure where counter accumulator values
	// are published by the APM.
	char __iomem *slc_pmon_accumulators;
};

// Structure containing all information about events, as necessary to integrate
// with the perf subsystem.
struct slc_pmon_events {
	// Number of events advertised by the APM.
	int num_events;
	// Even name string for all advertised events.
	const char **event_names;
	// Attributes used to spawn sysfs nodes for each event, along with
	// extra information for internal event tracking (i.e. to retrieve
	// the event id for a given sysfs node).
	struct dev_ext_attribute *platform_pmu_event_ext_attrs;
	// Pointer array for attributes from above, in form needed by perf
	// subsystem initialization.
	struct attribute **platform_pmu_event_attrs;
};

#define ACC_REFRESH_INTERVAL_NS 1000000

// Global structure containing all state information about PMON SLC.
struct slc_pmon_global_state {
	struct slc_pmon_counters counters;
	struct slc_pmon_events events;

	// Lock to serialize and rate-limit accumulator updates.
	struct mutex lock;
	// Timestamp for last APM accumulator poll.
	uint64_t last_poll_ns;
};

/*
 * Internal wrapper for the slc_acpm communication routine.
 *   command:    SLC plugin command to execute
 *   arg:        First (and only) argument to provide to the command. SLC PMON
 *               commands currently only need at most one.
 *   opt_buffer: Optional buffer to use for APC <-> APM communications. Only
 *               needed to retrieve secondary return values for some commands.
 *		 If provided, opt_buffer should be at least 32-byte long.
 *   returns:    Return code from APM.
 */
static int slc_pmon_acpm(enum slc_pmon_command command, int arg,
			 uint32_t *opt_buffer);

/*
 * Retrieve the number of advertised events from the APM.
 */
static int slc_acpm_get_num_events(void);

/*
 * Ask the APM to allocate a counter for the specified event.
 * Returns the counter's identifier on success, INVALID_COUNTER otherwise.
 */
static int slc_acpm_allocate_counter(int event_id);

/*
 * Ask the APM to release the specified counter, freeing it for other uses.
 * Returns true on success, false otherwise.
 */
static int slc_acpm_release_counter(int counter_id);

/*
 * Synchronize the target accumulator with the latest APM-published value.
 *   counter_id: Id of the accumulator to update.
 *   force_refresh: If true, ask the APM to update published counter values
 *                  in all cases. If not, only do so if the last publication
 *                  was performed more than ACC_REFRESH_INTERVAL_NS ago.
 */
static void slc_acpm_update_accumulator(int counter_id, bool force_refresh);

#define EVENT_ID_MASK 0xFFFFFF

/*
 * Ask the APM for the event name associated to the target event id.
 * Returns a string containing the name of the event for the specified index
 * on success, or NULL on failure.
 * The caller is responsible for freeing the generated string.
 */
static const char *slc_acpm_get_event_name(int event_index);

/*
 * Retrieve the number of supported counters from the APM.
 */
static int slc_acpm_get_num_counters(void);

/*
 * Callback for reads performed on the format sysfs file.
 */
static ssize_t slc_pmon_pmu_sysfs_format_show(struct device *dev,
					      struct device_attribute *attr,
					      char *buf);

/*
 * Callback for reads performed on the cpumask sysfs file.
 */
static ssize_t slc_pmon_pmu_sysfs_cpumask_show(struct device *dev,
					       struct device_attribute *attr,
					       char *buf);

/*
 * Callback for reads performed on event sysfs files.
 */
static ssize_t slc_pmon_pmu_sysfs_event_show(struct device *dev,
					     struct device_attribute *attr,
					     char *buf);

/*
 * Perf callback for updating the event count for the specified
 * perf_event structure. [?]
 */
static void slc_pmon_pmu_read(struct perf_event *event);

/*
 * Perf callback for starting (or restarting) to count event count increments
 * for the specified perf_event structure.
 * After allocation, SLC PMON counters are free-running. So 'stopping' and
 * 'starting' them is very much only checkpointing times when increments should
 * or should not be taken into account when updating event counts
 * in slc_pmon_pmu_read.
 */
static void slc_pmon_pmu_start(struct perf_event *event, int pmu_flags);

/*
 * Perf callback for stopping (or pausing) counting for the specified
 * perf_event structure.
 * After allocation, SLC PMON counters are free-running. So 'stopping' and
 * 'starting' them is very much only checkpointing times when increments should
 * or should not be taken into account when updating event counts
 * in slc_pmon_pmu_read.
 */
static void slc_pmon_pmu_stop(struct perf_event *event, int pmu_flags);

/*
 * Perf callback for allocating HW resources for the specified perf_event
 * structure.
 * Note that SLC PMON accumulators do *not* get reset on allocating or release,
 * so only accumulator deltas should be reflected in any event count update.
 * More information about what those flags mean and how they impact behavior?
 */
static int slc_pmon_pmu_add(struct perf_event *event, int evflags);

/*
 * Perf callback for releasing HW resources associated to the specified
 * perf_event structure.
 */
static void slc_pmon_pmu_del(struct perf_event *event, int evflags);

/*
 * Perf callback for initializing a perf_event structure.
 * This callback is performed once for each CPU core (on a different perf_event
 * structure) for each event users want to measure.
 * As SLC PMON counts are SOC-wide, only one of those should be elected to
 * actually count for each event.
 */
static int slc_pmon_pmu_event_init(struct perf_event *event);

/*
 * Initialization of counter structures.
 */
static int slc_pmon_counter_init(void);

/*
 * Cleanup function for counter structures.
 */
static void slc_pmon_counter_cleanup(void);

/*
 * Initialization of event structures.
 */
static int slc_pmon_event_init(void);

/*
 * Cleanup function for event structures.
 */
static void slc_pmon_event_cleanup(void);

// Helper macros for initializing perf-needed sysfs attributes.

#define DYN_ATTR(_name, _mode, _show, _store)                                  \
	{                                                                      \
		.attr = { .name = _name,                                       \
			  .mode = VERIFY_OCTAL_PERMISSIONS(_mode) },           \
		.show = _show, .store = _store,                                \
	}

#define SLC_PMON_EXT_ATTR(_name, _func, _config)                               \
	(&((struct dev_ext_attribute[]){                                       \
		{ .attr = DYN_ATTR(_name, 0444, _func, NULL),                  \
		  .var = (void *)_config } })[0]                               \
		  .attr.attr)

#define SLC_PMON_EXT_ATTR_FULL(_name, _func, _config)                          \
	(&((struct dev_ext_attribute[]){                                       \
		{ .attr = DYN_ATTR(_name, 0444, _func, NULL),                  \
		  .var = (void *)_config } })[0])

#define SLC_PMON_FORMAT_ATTR(_name, _config)                                   \
	SLC_PMON_EXT_ATTR(_name, slc_pmon_pmu_sysfs_format_show,               \
			  (char *)_config)

#define SLC_PMON_CPUMASK_ATTR(_name, _config)                                  \
	SLC_PMON_EXT_ATTR(_name, slc_pmon_pmu_sysfs_cpumask_show,              \
			  (char *)_config)

#define SLC_PMON_EVENT_ATTR(_name, _config)                                    \
	SLC_PMON_EXT_ATTR_FULL(_name, slc_pmon_pmu_sysfs_event_show,           \
			       (unsigned long)_config)

#endif // __GOOGLE_SLC_PMON_INTERNAL_H__
