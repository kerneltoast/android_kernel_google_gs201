/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Interface of managing the usage stats of IPs.
 *
 * Copyright (C) 2023 Google LLC
 */

#ifndef __GCIP_USAGE_STATS_H__
#define __GCIP_USAGE_STATS_H__

#include <linux/bits.h>
#include <linux/device.h>
#include <linux/hashtable.h>
#include <linux/mutex.h>
#include <linux/stringify.h>
#include <linux/types.h>

/* Attribute read/write mode (permissions). */
#define GCIP_USAGE_STATS_MODE_RO 0444
#define GCIP_USAGE_STATS_MODE_WO 0200
#define GCIP_USAGE_STATS_MODE_RW (GCIP_USAGE_STATS_MODE_RO | GCIP_USAGE_STATS_MODE_WO)

/* Macros which generate `struct gcip_usage_stats_attr` instances easily. */
#define GCIP_USAGE_STATS_ATTR(_metric, _type, _subcomponent, _name, _mode, _show, _store)          \
	struct gcip_usage_stats_attr gcip_usage_stats_attr_##_name = {                             \
		.metric = _metric,                                                                 \
		.type = _type,                                                                     \
		.subcomponent = _subcomponent,                                                     \
		.name = __stringify(_name),                                                        \
		.mode = _mode,                                                                     \
		.show = _show,                                                                     \
		.store = _store,                                                                   \
	}

#define GCIP_USAGE_STATS_ATTR_RW(metric, type, subcomponent, name, show, store)                    \
	GCIP_USAGE_STATS_ATTR(metric, type, subcomponent, name, GCIP_USAGE_STATS_MODE_RW, show,    \
			      store)

#define GCIP_USAGE_STATS_ATTR_RO(metric, type, subcomponent, name, show)                           \
	GCIP_USAGE_STATS_ATTR(metric, type, subcomponent, name, GCIP_USAGE_STATS_MODE_RO, show,    \
			      NULL)

#define GCIP_USAGE_STATS_ATTR_WO(metric, type, subcomponent, name, store)                          \
	GCIP_USAGE_STATS_ATTR(metric, type, subcomponent, name, GCIP_USAGE_STATS_MODE_WO, NULL,    \
			      store)

/*
 * Set a device attribute to show/store all subcomponents instead of one specific subcomponent.
 * See @subcomponents field of `struct gcip_usage_stats_attr`.
 */
#define GCIP_USAGE_STATS_ATTR_ALL_SUBCOMPONENTS -1

/*
 * Size of the metric v1 in bytes.
 *
 * We decided to use 20 bytes for the V1 metrics. However, from V2, we increased the size of it to
 * 24 bytes which is the same as `sizeof(struct gcip_usage_stats_metric)` to collect more stats
 * information. To verify whether the firmware sent valid V1 size metrics, keep it as a macro.
 */
#define GCIP_USAGE_STATS_METRIC_SIZE_V1 20

/* Max number of frequencies to support. */
#define GCIP_USAGE_STATS_MAX_DVFS_FREQ_NUM 25

struct gcip_usage_stats_attr;

typedef ssize_t (*gcip_usage_stats_show_t)(struct device *dev, struct gcip_usage_stats_attr *attr,
					   char *buf, void *data);
typedef ssize_t (*gcip_usage_stats_store_t)(struct device *dev, struct gcip_usage_stats_attr *attr,
					    const char *buf, size_t count, void *data);

/*
 * The version of metrics.
 * The format of header or the size of metrics would be different from the versions. However,
 * the metric implementation will be shared. See each metric implementation to know which fields
 * can be accessed according to the versions.
 */
enum gcip_usage_stats_version {
	/*
	 * In V1, the headers must have the format of the `struct gcip_usage_stats_header_v1` and
	 * the size of metrics must be GCIP_USAGE_STATS_METRIC_SIZE_V1.
	 */
	GCIP_USAGE_STATS_V1 = 1,
	/*
	 * In V2, the headers must have the format of the `struct gcip_usage_stats_header` and
	 * the size of metrics must be `sizeof(struct gcip_usage_stats_metric)`.
	 */
	GCIP_USAGE_STATS_V2 = 2,
	/* Version of metrics must be lower than this. */
	GCIP_USAGE_STATS_VERSION_UPPER_BOUND,
};

/* Hash bits which will be used when initializing @ustats->core_usage_htable. */
#define GCIP_USAGE_STATS_UID_HASH_BITS 3

/* Must be kept in sync with firmware `enum class UsageTrackerMetric::Type`. */
enum gcip_usage_stats_metric_type {
	GCIP_USAGE_STATS_METRIC_TYPE_RESERVED,
	GCIP_USAGE_STATS_METRIC_TYPE_CORE_USAGE,
	GCIP_USAGE_STATS_METRIC_TYPE_COMPONENT_UTILIZATION,
	GCIP_USAGE_STATS_METRIC_TYPE_COUNTER,
	GCIP_USAGE_STATS_METRIC_TYPE_THREAD_STATS,
	GCIP_USAGE_STATS_METRIC_TYPE_MAX_WATERMARK,
	GCIP_USAGE_STATS_METRIC_TYPE_DVFS_FREQUENCY_INFO,
};

/*
 * Encapsulates core usage information of a specific application.
 * Must be kept in sync with firmware `struct CoreUsage`.
 */
struct gcip_usage_stats_core_usage {
	/*
	 * The applications global identifier.
	 * This value IS NOT the virtual identifier assigned by the accelerator kernel driver,
	 * this is the user ID of the application assigned by Linux kernel.
	 */
	int32_t uid;
	/* The frequency (kHz) represented by this report. */
	uint32_t operating_point;
	/* Utilization time in microseconds (us). */
	uint32_t control_core_duration;

	/* Following fields must not be accessed in lower than V2. */

	/*
	 * The compute core is represented by this metric on DSP.
	 * The TPU does not personalize core usages by DVFS and can expect this value to be 0 at
	 * all times.  For DSP the value will be either 0, 1, or 2.
	 */
	uint8_t core_id;
	/* Reserved. */
	uint8_t reserved[3];
} __packed;

/*
 * Hash table entry which stores core usage per uid.
 * It will be added to the hash table of its subcomponent, @ustats->core_usage_htable[], using @uid
 * as the key and itself as the value.
 */
struct gcip_usage_stats_core_usage_uid_entry {
	int32_t uid;
	uint64_t time_in_state[GCIP_USAGE_STATS_MAX_DVFS_FREQ_NUM];
	struct hlist_node node;
};

/*
 * An enum to represent the different activity components we can track metrics for.
 * Must be kept in sync with firmware `enum class Component`.
 */
enum gcip_usage_stats_component_utilization_type {
	/* The entire IP Block. */
	GCIP_USAGE_STATS_COMPONENT_UTILIZATION_IP,
	/* A compute core. */
	GCIP_USAGE_STATS_COMPONENT_UTILIZATION_CORES,
	/* The DSP or TPU Control Core (R52). */
	GCIP_USAGE_STATS_COMPONENT_UTILIZATION_CONTROL,

	/* The number of total types. Must be located at the end of this enum. */
	GCIP_USAGE_STATS_COMPONENT_UTILIZATION_NUM_TYPES,
};

/*
 * Encapsulates information about utilization of a component.
 * Must be kept in sync with firmware `struct ComponentActivity`.
 */
struct gcip_usage_stats_component_utilization {
	/* Type of component. */
	enum gcip_usage_stats_component_utilization_type component;
	/*
	 * The percentage of time the component was active over the collection interval.
	 * This value is strictly between 0 and 100.
	 */
	int32_t utilization;

	/* Following fields must not be accessed in version 1. */

	/* Reserved. */
	uint32_t reserved[2];
} __packed;

/*
 * Defines different counter types we track.
 * Must be kept in sync with firmware `enum class CounterType`.
 */
enum gcip_usage_stats_counter_type {
	/* Active TPU cycles. */
	GCIP_USAGE_STATS_COUNTER_TPU_ACTIVIY_CYCLES,
	/* The number of stalls caused by throttling. */
	GCIP_USAGE_STATS_COUNTER_TPU_THROTTLE_STALLS,
	/* Number of TPU inferences / DSP workloads. */
	GCIP_USAGE_STATS_COUNTER_WORKLOAD,
	/* Number of TPU offload op invocations. */
	GCIP_USAGE_STATS_COUNTER_TPU_OP,
	/* Number of times a TPU op invocation used its cache parameters. */
	GCIP_USAGE_STATS_COUNTER_PARAM_CACHING_HIT,
	/* Number of times a TPU op invocation had to cache its parameters. */
	GCIP_USAGE_STATS_COUNTER_PARAM_CACHING_MISS,
	/*
	 * Number of times preemptions (either software or hardware) occurred between different
	 * clients.
	 * - Hardware preemption: The preemption which occurs between QoS classes. E.g., Realtime
	 *                        QoS class has a higher priority than Best-Effort QoS class. In
	 *                        this case, the current workload will be stopped at the next
	 *                        scalar fence, saved and will be restored after the new higher
	 *                        priority workload is completed.
	 * - Software preemption: The preemption which occurs between workloads with the same QoS
	 *                        class, but different priorities. The on-going workload is allowed
	 *                        to be completed before the higher priority one begins to execute.
	 *                        In the case of TPU, one TPU offload operation can be cut into
	 *                        multiple chunks (workloads) and it allows the higher priority
	 *                        offload to have the chance to preempt after the current chunk is
	 *                        processed.
	 */
	GCIP_USAGE_STATS_COUNTER_CONTEXT_PREEMPTIONS,
	/* Number of times hardware preemptions occurred. */
	GCIP_USAGE_STATS_COUNTER_HW_PREEMPTIONS,
	/*
	 * The total time in microseconds spent saving a hardware context during hardware
	 * preemption.
	 */
	GCIP_USAGE_STATS_COUNTER_TOTAL_HW_CONTEXT_SAVE_TIME,
	/* The total time in microseconds spent waiting to hit a scalar fence. */
	GCIP_USAGE_STATS_COUNTER_TOTAL_SCALAR_FENCE_WAIT_TIME,
	/* The number of times the Pipeline::Suspend function takes longer than SLA time. */
	GCIP_USAGE_STATS_COUNTER_NUM_OF_LONG_SUSPENDS,
	/* The number of times a compute core experienced a context switch. */
	GCIP_USAGE_STATS_COUNTER_CONTEXT_SWITCHES,
	/* The number of times a TPU cluster reconfiguration occurred. */
	GCIP_USAGE_STATS_COUNTER_NUM_OF_RECONFIGURATIONS,
	/*
	 * The number of times a TPU cluster reconfiguration occurred and was strictly motivated by
	 * a preemption.
	 */
	GCIP_USAGE_STATS_COUNTER_NUM_OF_RECONFIGURATIONS_BY_PREEMPTION,

	/* The number of total types. Must be located at the end of this enum. */
	GCIP_USAGE_STATS_COUNTER_NUM_TYPES,
};

/*
 * Generic counter. Only reported if it has a value larger than 0.
 * Must be kept in sync with firmware `struct Counter`.
 */
struct gcip_usage_stats_counter {
	/* What it counts. */
	enum gcip_usage_stats_counter_type type;
	/* Accumulated value since last initialization. */
	uint64_t value;

	/* Following fields must not be accessed in version 1. */

	/* An identifier that personalizes the represented hardware for counters. */
	uint8_t component_id;
	/* Reserved. */
	uint8_t reserved[3];
} __packed;

/*
 * An enum to identify the tracked firmware threads.
 * Must be kept in sync with firmware `enum class UsageTrackerThreadId`.
 */
enum gcip_usage_stats_thread_stats_thread_id {
	/* The entry thread for the firmware. */
	GCIP_USAGE_STATS_THREAD_MAIN_TASK,
	/*
	 * The thread that processes commands from the kernel and sends commands reversely in some
	 * cases, e.g., firmware crashes.
	 */
	GCIP_USAGE_STATS_THREAD_KCI_HANDLER,
	/* The thread that determines the commanded power state of the system. */
	GCIP_USAGE_STATS_THREAD_POWER_ADMINISTRATOR,
	/* The thread responsible for coordinating and dispatching workloads. */
	GCIP_USAGE_STATS_THREAD_SCHEDULER,
	/* The thread that handles VII commands from TPU clients. */
	GCIP_USAGE_STATS_THREAD_VII_HANDLER,
	/*
	 * The multi-core coordination thread that shares complex assignments with other TPU cores.
	 */
	GCIP_USAGE_STATS_THREAD_MCP_GRAPH_DRIVER,
	/* The single-core coordination thread that handles local-only graphs. */
	GCIP_USAGE_STATS_THREAD_SCP_GRAPH_DRIVER,
	/* The thread that coordinates the progress of scalar and tile TPU workloads. */
	GCIP_USAGE_STATS_THREAD_TPU_DRIVER,
	/* Orchestrates restarting client threads when there is a fatal error in the pipeline. */
	GCIP_USAGE_STATS_THREAD_RESTART_HANDLER,
	/* Used for polling some state but not blocking other threads from execution. */
	GCIP_USAGE_STATS_THREAD_POLL_SERVICE,
	/* Schedules DMAs on the main DMA engine. */
	GCIP_USAGE_STATS_THREAD_DMA_DRIVER,
	/* Used for driving AES DMA for random number generation. */
	GCIP_USAGE_STATS_THREAD_GRAPH_DMA_DRIVER,
	/*
	 * The multi-cluster scheduler, this dispatches complex workloads to the major and minor
	 * TPU clusters.
	 */
	GCIP_USAGE_STATS_THREAD_MC_SCHEDULER,
	/*
	 * The single-cluster scheduler that dispatches workloads to only a single TPU cluster at
	 * a time.
	 */
	GCIP_USAGE_STATS_THREAD_SC_SCHEDULER,
	/*
	 * The thread that dispatches scheduled workloads from the DSP control core directly to the
	 * DSP cores.
	 */
	GCIP_USAGE_STATS_THREAD_DSP_CORE_MANAGER,
	/* The driving thread for intercore message handling. */
	GCIP_USAGE_STATS_THREAD_INTERCORE_SUBORDINATE,
	/* The thread that executes callback when the timer expires. */
	GCIP_USAGE_STATS_THREAD_TIMER_SERVICE,

	/* The number of total threads. Must be located at the end of this enum. */
	GCIP_USAGE_STATS_THREAD_NUM_TYPES,
};

/*
 * Statistics related to a single thread in firmware.
 * Must be kept in sync with firmware `struct ThreadStats`.
 */
struct gcip_usage_stats_thread_stats {
	/* The thread in question. */
	enum gcip_usage_stats_thread_stats_thread_id thread_id;
	/* Maximum stack usage (in bytes) since last firmware boot. */
	uint32_t max_stack_usage_bytes;

	/* Following fields must not be accessed in version 1. */

	/* Reserved. */
	uint32_t reserved[2];
} __packed;

/*
 * Defines different max watermarks we track.
 * Must be kept in sync with firmware `enum class MaxWatermarkType`.
 */
enum gcip_usage_stats_max_watermark_type {
	/* The number of UCI/VII commands dequeued and not yet responded to. */
	GCIP_USAGE_STATS_MAX_WATERMARK_OUTSTANDING_CMDS,
	/* The maximum number of outstanding preempted workloads that must be resumed. */
	GCIP_USAGE_STATS_MAX_WATERMARK_PREEMPTION_DEPTH,
	/*
	 * The longest time in microseconds required to save a cluster-context so that another
	 * client can run on the same cluster.
	 */
	GCIP_USAGE_STATS_MAX_WATERMARK_MAX_HW_CONTEXT_SAVE_TIME,
	/*
	 * Maximum time in microseconds spent waiting to hit a scalar fence during hardware
	 * preemption.
	 */
	GCIP_USAGE_STATS_MAX_WATERMARK_MAX_SCALAR_FENCE_WAIT_TIME,
	/* Maximum time in microseconds spent during the Pipeline::Suspend function. */
	GCIP_USAGE_STATS_MAX_WATERMARK_MAX_SUSPEND_TIME,

	/* The number of total types. Must be located at the end of this enum. */
	GCIP_USAGE_STATS_MAX_WATERMARK_NUM_TYPES,
};

/*
 * Max watermark. Only reported if it has a value larger than 0.
 * Must be kept in sync with firmware `struct MaxWatermark`.
 */
struct gcip_usage_stats_max_watermark {
	/* What it counts. */
	enum gcip_usage_stats_max_watermark_type type;
	/* Maximum expressed value over the collection interval. */
	uint64_t value;

	/* Following fields must not be accessed in version 1. */

	/* Reporting component. */
	uint8_t component_id;
	/* Reserved. */
	uint8_t reserved[3];
} __packed;

/*
 * Used to report DVFS frequencies supported by the chip.
 * Must be kept in sync with firmware `struct DvfsFrequencyInfo`.
 */
struct gcip_usage_stats_dvfs_frequency_info {
	/* An actively supported DVFS Frequency (kHz). */
	uint32_t supported_frequency;

	/* Following fields must not be accessed in lower than V2. */

	/* Reserved. */
	uint32_t reserved[3];
} __packed;

/*
 * Header struct in the v1 metric buffer.
 * Keep this structure for the compatibility.
 */
struct gcip_usage_stats_header_v1 {
	/* Number of metrics being reported. */
	uint32_t num_metrics;
	/* Size of each metric struct. */
	uint32_t metric_size;
};

/*
 * Header struct in the metric buffer.
 * Must be kept in sync with firmware `struct UsageTrackerHeader`.
 */
struct gcip_usage_stats_header {
	/* Number of bytes in this header. */
	uint16_t header_bytes;
	/* Metrics version. */
	uint16_t version;
	/* Number of metrics being reported. */
	uint32_t num_metrics;
	/* Size of each metric struct. */
	uint32_t metric_size;
};

/*
 * Encapsulates a single metric reported to the kernel driver.
 * Must be kept in sync with firmware `struct UsageTrackerMetric`.
 */
struct gcip_usage_stats_metric {
	uint32_t type;
	uint8_t reserved[4];
	union {
		struct gcip_usage_stats_core_usage core_usage;
		struct gcip_usage_stats_component_utilization component_utilization;
		struct gcip_usage_stats_counter counter;
		struct gcip_usage_stats_thread_stats thread_stats;
		struct gcip_usage_stats_max_watermark max_watermark;
		struct gcip_usage_stats_dvfs_frequency_info dvfs_frequency_info;
		/* The implementation of each metric must fit to 16 bytes. */
		uint8_t impl_reserved[16];
	};
} __packed;

/* Operators which are needed while processing usage stats data. */
struct gcip_usage_stats_ops {
	/*
	 * The callback which sends `GET_USAGE` KCI to get the latest usage stats from the firmware
	 * synchronously and calls `gcip_usage_stats_process_buffer` function to process them.
	 *
	 * This callback is required and will be called when the user tries to read device
	 * statistics.
	 *
	 * Returns KCI response code on success or < 0 on error (typically -ETIMEDOUT).
	 */
	int (*update_usage_kci)(void *data);

	/*
	 * Returns the number of default DVFS frequencies.
	 * If the firmware has never sent `DVFS_FREQUENCY_INFO` metrics, it will use the default
	 * frequencies which are maintained by the kernel driver.
	 */
	int (*get_default_dvfs_freqs_num)(void *data);

	/*
	 * Returns the DVFS frequency of @idx.
	 * @idx will not exceed the number of default DVFS frequencies which is returned by the
	 * `get_default_dvfs_freqs_num` operator.
	 */
	unsigned int (*get_default_dvfs_freq)(int idx, void *data);
};

/* Structure manages the information of usage stats and device attributes. */
struct gcip_usage_stats {
	/* The version of metrics. */
	enum gcip_usage_stats_version version;
	/* The number of subcomponents. (e.g., TPU: clusters, DSP: cores) */
	unsigned int subcomponents;
	/* The device to register attributes. */
	struct device *dev;
	/* User-data. */
	void *data;

	/* Pointer array of attributes which will be registered to the device. */
	struct attribute **attrs;
	/* Attribute group which will contain @attrs. */
	struct attribute_group group;

	/* Operators. */
	const struct gcip_usage_stats_ops *ops;

	/*
	 * Core usage (per subcomponent).
	 * Stores stats as an UID to `struct gcip_usage_stats_core_usage_uid_entry` hash table.
	 * Declare it as a pointer to an array because we have to dynamically allocate multiple
	 * rows with the fixed column size.
	 * I.e., (@subcomponents (rows) * BIT(GCIP_USAGE_STATS_UID_HASH_BITS) (cols)) 2d array.
	 */
	struct hlist_head (*core_usage_htable)[BIT(GCIP_USAGE_STATS_UID_HASH_BITS)];
	/* Component utilization. */
	int32_t component_utilization[GCIP_USAGE_STATS_COMPONENT_UTILIZATION_NUM_TYPES];
	/*
	 * Counter (per subcomponents).
	 * Declare it as a pointer to an array because we have to dynamically allocate multiple
	 * rows with the fixed column size.
	 * I.e., (@subcomponents (rows) * GCIP_USAGE_STATS_COUNTER_NUM_TYPES (cols)) 2d array.
	 */
	int64_t (*counter)[GCIP_USAGE_STATS_COUNTER_NUM_TYPES];
	/* Thread statistics. */
	int32_t thread_max_stack_usage[GCIP_USAGE_STATS_THREAD_NUM_TYPES];
	/*
	 * Max watermark (per subcomponents).
	 * Declare it as a pointer to an array because we have to dynamically allocate multiple
	 * rows with the fixed column size.
	 * I.e., (@subcomponents (rows) * GCIP_USAGE_STATS_MAX_WATERMARK_NUM_TYPES (cols)) 2d
	 * array.
	 */
	int64_t (*max_watermark)[GCIP_USAGE_STATS_MAX_WATERMARK_NUM_TYPES];
	/* Protects the statistics above. */
	struct mutex usage_stats_lock;

	/*
	 * DVFS frequencies that the firmware returns via `DVFS_FREQUENCY_INFO` metric.
	 * If the firmware has sent `DVFS_FREQUENCY_INFO` metric, it will be used instead of
	 * getting the default ones from the kernel driver side via @get_default_dvfs_freq callback
	 * of the `struct gcip_usage_stats_ops`.
	 */
	uint32_t dvfs_freqs[GCIP_USAGE_STATS_MAX_DVFS_FREQ_NUM];
	/* The number of DVFS frequencies in @dvfs_freqs_num. */
	int dvfs_freqs_num;
	/*
	 * Default DVFS frequencies defined by the IP driver. These frequencies will be used until
	 * the firmware sends `DVFS_FREQUENCY_INFO` metrics. Those values will be fetched from the
	 * IP driver via the `get_default_dvfs_freq` callback, but we are going to store them here
	 * separately to removing duplicate ones.
	 */
	uint32_t default_dvfs_freqs[GCIP_USAGE_STATS_MAX_DVFS_FREQ_NUM];
	/* The number of default DVFS frequencies in @default_dvfs_freqs. */
	int default_dvfs_freqs_num;
	/* Protects DVFS frequencies. */
	struct mutex dvfs_freqs_lock;
};

/*
 * Structure which contains information of an attribute to be registered to the device.
 * One can directly create an instance, but it is recommneded to use `GCIP_USAGE_STATS_ATTR_*`
 * macros instead. A pointer array of this attributes must be passed to the @attrs of
 * `struct gcip_usage_stats_args`.
 */
struct gcip_usage_stats_attr {
	/* The metric to be collected. */
	enum gcip_usage_stats_metric_type metric;
	/*
	 * The sub-type of @metric to be collected.
	 * - COMPONENT_UTILIZATION: enum gcip_usage_stats_component_utilization_type
	 * - COUNTER: enum gcip_usage_stats_counter_type
	 * - MAX_WATERMARK: enum gcip_usage_stats_max_watermark_type
	 */
	unsigned int type;
	/*
	 * The 0-based index of subcomponent. (Ignored in V1 metrics.)
	 *
	 * One can specify the subcomponent to be read if there are multiple subcomponents.
	 *
	 * If this value is `GCIP_USAGE_STATS_ATTR_ALL_SUBCOMPONENTS`, its show or store function
	 * will involve in all subcomponents. In case of show, it will print statistics of all
	 * subcomponents in an array with whitespace separation. Note that according to the Linux
	 * documentation, one value per one attribute is a rule, but printing multiple same types
	 * in an array is acceptable. Therefore, use this way only when the printing format is
	 * simple. In case of store, it will update statistic values (mostly reset to 0) of all
	 * subcomponents.
	 *
	 * Note: when the metric is `CORE_USAGE`, using `GCIP_USAGE_STATS_ATTR_ALL_SUBCOMPONENTS`
	 * is invalid because its printing format is too complicated to print multiple
	 * subcomponents in one attribute.
	 */
	int subcomponent;
	/* The name of the attribute. */
	const char *name;
	/* Permission. It must be one of `GCIP_USAGE_STATS_MODE_*`. */
	umode_t mode;
	/*
	 * User-defined show callback.
	 *
	 * Mostly, one will set it as NULL to use the GCIP implementation.
	 * See the `gcip_usage_stats_alloc_attrs` function to find which function will be used for
	 * the show function according to the type of metric.
	 *
	 * However, if a customized show function is needed, one can pass its own function to this.
	 *
	 * It will be used only when @mode has the read permission.
	 */
	gcip_usage_stats_show_t show;

	/*
	 * User-defined store callback.
	 *
	 * Mostly, one will set it as NULL to use the GCIP implementation.
	 * See the `gcip_usage_stats_alloc_attrs` function to find which function will be used for
	 * the store function according to the type of metric.
	 *
	 * However, if a customized store function is needed, one can pass its own function to
	 * this.
	 *
	 * It will be used only when @mode has the write permission.
	 */
	gcip_usage_stats_store_t store;

	/* Following fields must not be touched by the caller. */
	struct device_attribute dev_attr;
	struct gcip_usage_stats *ustats;
};

/*
 * Arguments for `gcip_usage_stats_init`.
 *
 * `struct gcip_usage_stats` instance will be initialized according to this.
 */
struct gcip_usage_stats_args {
	/* The version of metrics. */
	enum gcip_usage_stats_version version;
	/*
	 * The number of subcomponents. (e.g., TPU: clusters, DSP: cores)
	 * Must be bigger than 0.
	 */
	unsigned int subcomponents;
	/* The device to register attributes. */
	struct device *dev;
	/* User-data. */
	void *data;
	/*
	 * Pointer array of attributes.
	 * This must not be freed before the `gcip_usage_stats_exit` is called.
	 */
	struct gcip_usage_stats_attr **attrs;
	/* The size of @attrs. */
	unsigned int num_attrs;
	/*
	 * Operators.
	 * See `struct gcip_usage_stats_ops` for the details.
	 */
	const struct gcip_usage_stats_ops *ops;
};

/*
 * Initializes @ustats.
 *
 * @ustats must be cleaned up with the `gcip_usage_stats_exit` function.
 */
int gcip_usage_stats_init(struct gcip_usage_stats *ustats,
			  const struct gcip_usage_stats_args *args);

/* Cleans up @ustats which is initialized by the `gcip_usage_stats_init` function. */
void gcip_usage_stats_exit(struct gcip_usage_stats *ustats);

/* Processes the buffer which is returned by the firmware via `GET_USAGE` KCI. */
void gcip_usage_stats_process_buffer(struct gcip_usage_stats *ustats, void *buf);

#endif /* __GCIP_USAGE_STATS_H__ */
