/* SPDX-License-Identifier: GPL-2.0 */
/*
 * EdgeTPU usage stats header
 *
 * Copyright (C) 2020 Google, Inc.
 */
#ifndef __EDGETPU_USAGE_STATS_H__
#define __EDGETPU_USAGE_STATS_H__

#include <linux/hashtable.h>
#include <linux/mutex.h>

/* Header struct in the metric buffer. */
/* Must be kept in sync with firmware struct UsageTrackerHeader */
struct edgetpu_usage_header {
	uint32_t num_metrics;		/* Number of metrics being reported */
	uint32_t metric_size;		/* Size of each metric struct */
};

/*
 * Encapsulate TPU core usage information of a specific application for a
 * specific power state.
 * Must be kept in sync with firmware struct TpuUsage.
 */
struct tpu_usage {
	/* Unique identifier of the application. */
	int32_t uid;
	/* The power state of the device (values are chip dependent) */
	uint32_t power_state;
	/* Duration of usage in microseconds. */
	uint32_t duration_us;
};

/*
 * An enum to represent the different components we can track metrics for.
 * Must be kept in sync with firmware struct Component.
 */
enum edgetpu_usage_component {
	/* The device as a whole */
	EDGETPU_USAGE_COMPONENT_DEVICE = 0,
	/* Just the TPU core */
	EDGETPU_USAGE_COMPONENT_TPU = 1,
	EDGETPU_USAGE_COMPONENT_COUNT = 2, /* number of components above */
};

/*
 * Encapsulates information about activity of a component.
 * Must be kept in sync with firmware struct ComponentActivity.
 */
struct edgetpu_component_activity {
	enum edgetpu_usage_component component;
	/* Utilization as a percentage since the last read. */
	int32_t utilization;
};

/*
 * Defines different counter types we track.
 * Must be kept in sync with firmware enum class CounterType.
 */
enum edgetpu_usage_counter_type {
	/* TPU active cycles. */
	EDGETPU_COUNTER_TPU_ACTIVE_CYCLES = 0,
	/* Number of stalls caused by throttling. */
	EDGETPU_COUNTER_TPU_THROTTLE_STALLS = 1,
	/* Number of graph invocations. */
	EDGETPU_COUNTER_INFERENCES = 2,
	/* Number of TPU offload op invocations. */
	EDGETPU_COUNTER_TPU_OPS = 3,
	/* Number of times a TPU op invocation used its cached parameters. */
	EDGETPU_COUNTER_PARAM_CACHE_HITS = 4,
	/* Number of times a TPU op invocation had to cache its parameters. */
	EDGETPU_COUNTER_PARAM_CACHE_MISSES = 5,
	/* Number of times a context got preempted by another. */
	EDGETPU_COUNTER_CONTEXT_PREEMPTS = 6,
	/* Number of times a hardware preemption occurred. */
	EDGETPU_COUNTER_HARDWARE_PREEMPTS = 7,
	/* Total time(us) spent in saving hw ctx during hw preemption */
	EDGETPU_COUNTER_HARDWARE_CTX_SAVE_TIME_US = 8,
	/* Total time(us) spent in waiting to hit scalar fence during hw preemption */
	EDGETPU_COUNTER_SCALAR_FENCE_WAIT_TIME_US = 9,
	/* Number of times (firmware)suspend function takes longer than SLA time. */
	EDGETPU_COUNTER_LONG_SUSPEND = 10,

	EDGETPU_COUNTER_COUNT = 11, /* number of counters above */
};

/* Generic counter. Only reported if it has a value larger than 0. */
struct __packed edgetpu_usage_counter {
	/* What it counts. */
	enum edgetpu_usage_counter_type type;

	/* Accumulated value since last initialization. */
	uint64_t value;
};

/* Defines different max watermarks we track. */
/* Must be kept in sync with firmware MaxWatermarkType */
enum edgetpu_usage_max_watermark_type {
	/* Number of outstanding commands in VII trackers of all contexts. */
	EDGETPU_MAX_WATERMARK_OUT_CMDS = 0,
	/* Number of preempted contexts at any given time. */
	EDGETPU_MAX_WATERMARK_PREEMPT_DEPTH = 1,
	/* Max time(us) spent in saving hw ctx during hw preemption */
	EDGETPU_MAX_WATERMARK_HARDWARE_CTX_SAVE_TIME_US = 2,
	/* Max time(us) spent in waiting to hit scalar fence during hw preemption */
	EDGETPU_MAX_WATERMARK_SCALAR_FENCE_WAIT_TIME_US = 3,
	/* Max time(us) spent during (firmware)suspend function. */
	EDGETPU_MAX_WATERMARK_SUSPEND_TIME_US = 4,

	/* Number of watermark types above */
	EDGETPU_MAX_WATERMARK_TYPE_COUNT = 5,
};

/* Max watermark. Only reported if it has a value larger than 0. */
struct __packed edgetpu_usage_max_watermark {
	/* What it counts. */
	enum edgetpu_usage_max_watermark_type type;

	/*
	 * Maximum value since last initialization (virtual device join in
	 * non-mobile, firmware boot on mobile).
	 */
	uint64_t value;
};

/* An enum to identify the tracked firmware threads. */
/* Must be kept in sync with firmware enum class UsageTrackerThreadId. */
enum edgetpu_usage_threadid {
	/* Individual thread IDs are not tracked. */

	/* Number of task identifiers. */
	EDGETPU_FW_THREAD_COUNT = 12,
};

/* Statistics related to a single thread in firmware. */
/* Must be kept in sync with firmware struct ThreadStats. */
struct edgetpu_thread_stats {
	/* The thread in question. */
	enum edgetpu_usage_threadid thread_id;

	/* Maximum stack usage (in bytes) since last firmware boot. */
	uint32_t max_stack_usage_bytes;
};

/* Must be kept in sync with firmware enum class UsageTrackerMetric::Type */
enum edgetpu_usage_metric_type {
	EDGETPU_METRIC_TYPE_RESERVED = 0,
	EDGETPU_METRIC_TYPE_TPU_USAGE = 1,
	EDGETPU_METRIC_TYPE_COMPONENT_ACTIVITY = 2,
	EDGETPU_METRIC_TYPE_COUNTER = 3,
	EDGETPU_METRIC_TYPE_THREAD_STATS = 4,
	EDGETPU_METRIC_TYPE_MAX_WATERMARK = 5,
	EDGETPU_METRIC_TYPE_DVFS_FREQUENCY_INFO = 6,
};

/*
 * Encapsulates a single metric reported to the kernel.
 * Must be kept in sync with firmware struct UsageTrackerMetric.
 */
struct edgetpu_usage_metric {
	uint32_t type;
	uint8_t reserved[4];
	union {
		struct tpu_usage tpu_usage;
		struct edgetpu_component_activity component_activity;
		struct edgetpu_usage_counter counter;
		struct edgetpu_thread_stats thread_stats;
		struct edgetpu_usage_max_watermark max_watermark;
		uint32_t dvfs_frequency_info;
	};
};

#define UID_HASH_BITS 3

struct edgetpu_usage_stats {
	DECLARE_HASHTABLE(uid_hash_table, UID_HASH_BITS);
	/* component utilization values reported by firmware */
	int32_t component_utilization[EDGETPU_USAGE_COMPONENT_COUNT];
	int64_t counter[EDGETPU_COUNTER_COUNT];
	int64_t max_watermark[EDGETPU_MAX_WATERMARK_TYPE_COUNT];
	int32_t thread_stack_max[EDGETPU_FW_THREAD_COUNT];
	struct mutex usage_stats_lock;
};

int edgetpu_usage_add(struct edgetpu_dev *etdev, struct tpu_usage *tpu_usage);
int edgetpu_usage_get_utilization(struct edgetpu_dev *etdev,
				  enum edgetpu_usage_component component);
void edgetpu_usage_stats_process_buffer(struct edgetpu_dev *etdev, void *buf);
void edgetpu_usage_stats_init(struct edgetpu_dev *etdev);
void edgetpu_usage_stats_exit(struct edgetpu_dev *etdev);

#endif /* __EDGETPU_USAGE_STATS_H__ */
