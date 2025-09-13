/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2020-2021 Google LLC.
 *
 * Author: Sidath Senanayake <sidaths@google.com>
 */

#ifndef _PIXEL_GPU_DVFS_H_
#define _PIXEL_GPU_DVFS_H_

/* Clocks & domains */

/**
 * enum gpu_dvfs_clk_index - GPU clock & power domains
 *
 * Stores the list of clocks on the GPU.
 */
enum gpu_dvfs_clk_index {
	/**
	 * &GPU_DVFS_CLK_TOP_LEVEL: Top level domain
	 *
	 * Corresponds to the domain which comprises the Job Manager, L2 cache
	 * and Tiler.
	 */
	GPU_DVFS_CLK_TOP_LEVEL = 0,

	/**
	 * &GPU_DVFS_CLK_SHADERS: Shader stack domain
	 *
	 * Corresponds to the domain clocking and powering the GPU shader
	 * cores.
	 */
	GPU_DVFS_CLK_SHADERS,

	/* All clock indices should be above this line */
	GPU_DVFS_CLK_COUNT,
};

/**
 * struct gpu_dvfs_clk - Stores data for a GPU clock
 *
 * @index:    &gpu_dvfs_clk_index for this clock
 * @cal_id:   ID for this clock domain. Set via DT.
 * @notifier: &blocking_notifier_head for reporting frequency changes on this clock.
 */
struct gpu_dvfs_clk {
	enum gpu_dvfs_clk_index index;
#if IS_ENABLED(CONFIG_CAL_IF)
	int cal_id;
#endif /* IS_ENABLED(CONFIG_CAL_IF) */
	struct blocking_notifier_head notifier;
};

/* Utilization */

/**
 * struct gpu_dvfs_utlization - Stores utilization statistics
 *
 * @util:     Overall utilization of the GPU
 * @mcu_util: Utilization of the MCU
 * @util_gl:  The share of utilization due to non-OpenCL work
 * @util_cl:  The share of utilization due ot OpenCL work
 */
struct gpu_dvfs_utlization {
	int util;
#if MALI_USE_CSF
	int mcu_util;
#endif
	int util_gl;
	int util_cl;
};

/* Governor */

/**
 * typedef gpu_dvfs_governor_logic_fn - Determines the next level based on utilization.
 *
 * @kbdev:     The &struct kbase_device of the GPU.
 * @util:      The integer utilization percentage the GPU is running at.
 * @util_gl:   Percentage of utilization from a GL context.
 * @util_cl:   Percentage of utilization from a CL context.
 *
 * This function is not expected to take any clock limits into consideration when
 * recommending the next level.
 *
 * Context: Expects the DVFS lock to be held by the caller.
 *
 * Return: The index of the next recommended level.
 */
typedef int (*gpu_dvfs_governor_logic_fn)(struct kbase_device *kbdev,
	struct gpu_dvfs_utlization *util_stats);

/**
 * enum gpu_dvfs_governor_type - Pixel GPU DVFS governor.
 *
 * This enum stores the list of available DVFS governors for the GPU. High-level.
 * documentation for each governor should be provided here.
 */
enum gpu_dvfs_governor_type {
	/**
	 * @GPU_DVFS_GOVERNOR_BASIC: A very simple GPU DVFS governor.
	 *
	 * The basic governor uses incoming GPU utilization data to determine
	 * whether the GPU should change levels.
	 *
	 * If the GPU's utilization is higher than the level's maximum threshold
	 * it will recommend a move to a higher throughput level.
	 *
	 * If the GPU's utilization is lower than the level's minimum threshold,
	 * and remains lower for a number of ticks set by the level's hysteresis
	 * value, then it will recommend a move to a lower throughput level.
	 */
	GPU_DVFS_GOVERNOR_BASIC = 0,
	GPU_DVFS_GOVERNOR_QUICKSTEP,
#if MALI_USE_CSF
	GPU_DVFS_GOVERNOR_QUICKSTEP_USE_MCU,
	GPU_DVFS_GOVERNOR_CAPACITY_USE_MCU,
#endif
	/* Insert new governors here */
	GPU_DVFS_GOVERNOR_COUNT,
	GPU_DVFS_GOVERNOR_INVALID,
};

/**
 * struct gpu_dvfs_governor_info - Data for a Pixel GPU DVFS governor.
 *
 * @name:     A human readable name for the governor.
 * @evaluate: A function pointer to the governor's evaluate function. See
 *            &gpu_dvfs_governor_logic_fn.
 */
struct gpu_dvfs_governor_info {
	const char *name;
	gpu_dvfs_governor_logic_fn evaluate;
};

int gpu_dvfs_governor_get_next_level(struct kbase_device *kbdev,
	struct gpu_dvfs_utlization *util_stats);
int gpu_dvfs_governor_set_governor(struct kbase_device *kbdev, enum gpu_dvfs_governor_type gov);
enum gpu_dvfs_governor_type gpu_dvfs_governor_get_id(const char *name);
ssize_t gpu_dvfs_governor_print_available(char *buf, ssize_t size);
ssize_t gpu_dvfs_governor_print_curr(struct kbase_device *kbdev, char *buf, ssize_t size);
int gpu_dvfs_governor_init(struct kbase_device *kbdev);
void gpu_dvfs_governor_term(struct kbase_device *kbdev);

/* Metrics */

/**
 * struct gpu_dvfs_metrics_uid_stats - Stores time in state data for a UID
 *
 * @active_kctx_count:  Count of active kernel contexts operating under this UID. Should only be
 *                      accessed while holding the kctx_list lock.
 * @uid:                The UID for this stats block.
 * @active_work_count:  Count of currently executing units of work on the GPU from this UID. Should
 *                      only be accessed while holding the hwaccess lock if using a job manager GPU
 *                      or holding the csf.scheduler.lock for CSF GPUs.
 * @period_start:       The time (in nanoseconds) that the current active period for this UID began
 *                      Should only be accessed while holding the hwaccess lock if using a job
 *                      manager GPU, CSF GPUs require holding the csf.scheduler.lock.
 * @tis_stats:          &struct gpu_dvfs_opp_metrics block storing time in state data for this UID.
 *                      Should only be accessed while holding the hwaccess lock if using a job
 *                      manager GPU, CSF GPUs require holding the csf.scheduler.lock.
 * @gpu_cycles_last:    Accumulates active cycles since last read of the gpu_top sysfs.
 * @gpu_active_ns_last: Accumulates time in ns since last read of the gpu_top sysfs.
 * @timestamp_ns_last:  Timestamp of the last time that the gpu_top sysfs was read,
 *                      or timestamp when this UID's context was created.
 * @uid_list_node:      Index this structure in a kernel hash table by UID.
 *                      Should only be accessed while holding the kctx_list lock.
 */
struct gpu_dvfs_metrics_uid_stats {
	int active_kctx_count;
	kuid_t uid;
	int active_work_count;
	u64 period_start;
	struct gpu_dvfs_opp_metrics *tis_stats;
	u64 gpu_cycles_last;
	u64 gpu_active_ns_last;
	u64 timestamp_ns_last;
	struct hlist_node uid_list_node;
};

/**
 * gpu_dvfs_hash_uid_stats - Macro to hash a UID to 8 bits.
 *
 * @uid: The &kuid_t corresponding to hash.
 *
 * Return: The hash of the UID.
 */
#define gpu_dvfs_hash_uid_stats(uid) \
	((u8)((uid) & 0xFF))

/**
 * gpu_dvfs_metrics_update() - Updates GPU metrics on level or power change.
 *
 * @kbdev:       The &struct kbase_device for the GPU.
 * @old_level:   The level that the GPU has just moved from. Can be the same as &new_level.
 * @new_level:   The level that the GPU has just moved to. Can be the same as &old_level. This
 *               parameter is ignored if &power_state is false.
 * @power_state: The current power state of the GPU. Can be the same as the current power state.
 *
 * This function should be called (1) right after a change in power state of the GPU, or (2) just
 * after changing the level of a powered on GPU. It will update the metrics for each of the GPU
 * DVFS level metrics and the power metrics as appropriate.
 *
 * Context: Expects the caller to hold the dvfs.lock & dvfs.metrics.lock.
 */
void gpu_dvfs_metrics_update(struct kbase_device *kbdev, int old_level, int new_level,
	bool power_state);

/**
 * gpu_dvfs_metrics_init() - Initializes DVFS metrics.
 *
 * @kbdev: The &struct kbase_device for the GPU.
 *
 * Context: Process context. Takes and releases the DVFS lock.
 *
 * Return: On success, returns 0 otherwise returns an error code.
 */
int gpu_dvfs_metrics_init(struct kbase_device *kbdev);

/**
 * gpu_dvfs_metrics_term() - Terminates DVFS metrics
 *
 * @kbdev: The &struct kbase_device for the GPU.
 */
void gpu_dvfs_metrics_term(struct kbase_device *kbdev);

/**
 * gpu_dvfs_metrics_transtab_size - Get the size of the transtab table
 *
 * @pc: Pointer to the Pixel Context
 *
 * Return: The size (in number of elements) of the transtab table
 */
#define gpu_dvfs_metrics_transtab_size(pc) ((pc)->dvfs.table_size * (pc)->dvfs.table_size)

/**
 * gpu_dvfs_metrics_transtab_entry - Macro to return array entry in transtab
 *
 * @pc: Pointer to the Pixel Context
 * @i:  The 'From' offset in the transtab table
 * @j:  The 'To' offset in the transtab table
 *
 * Return: Translates into code referring to the relevant array element in the transtab
 */
#define gpu_dvfs_metrics_transtab_entry(pc, i, j) \
	((pc)->dvfs.metrics.transtab[(i) * (pc)->dvfs.table_size + (j)])

/* QOS */

#ifdef CONFIG_MALI_PIXEL_GPU_QOS

/**
 * struct gpu_dvfs_qos_vote - Data for a QOS vote
 *
 * @enabled: A boolean tracking whether this vote has been enabled or not.
 * @req:     The underlying &struct exynos_pm_qos_request that implements this
 *           vote.
 */
struct gpu_dvfs_qos_vote {
	bool enabled;
	struct exynos_pm_qos_request req;
};

void gpu_dvfs_qos_set(struct kbase_device *kbdev, int level);
void gpu_dvfs_qos_reset(struct kbase_device *kbdev);
int gpu_dvfs_qos_init(struct kbase_device *kbdev);
void gpu_dvfs_qos_term(struct kbase_device *kbdev);

#endif /* CONFIG_MALI_PIXEL_GPU_QOS */

/* Thermal */

#ifdef CONFIG_MALI_PIXEL_GPU_THERMAL
int gpu_tmu_init(struct kbase_device *kbdev);
void gpu_tmu_term(struct kbase_device *kbdev);
#endif /* CONFIG_MALI_PIXEL_GPU_THERMAL*/

/* Common */

/**
 * enum gpu_dvfs_level_lock_type - Pixel GPU level lock sources.
 *
 * This enum stores the list of sources that can impose operating point limitations on the DVFS
 * subsystem. They are listed in increasing priority order in that if a later lock is more
 * restrictive than an earlier one, the value from the later lock is selected.
 */
enum gpu_dvfs_level_lock_type {
#if IS_ENABLED(CONFIG_CAL_IF)
	/**
	 * &GPU_DVFS_LEVEL_LOCK_ECT: ECT lock
	 *
	 * This lock is based on Fmax and Fmin obtained from ECT table of the chip.
	 */
	GPU_DVFS_LEVEL_LOCK_ECT,
#endif /* CONFIG_CAL_IF */
	/**
	 * &GPU_DVFS_LEVEL_LOCK_DEVICETREE: Devicetree lock
	 *
	 * This lock is used to enforce scaling limits set as part of the GPU device tree entry.
	 */
	GPU_DVFS_LEVEL_LOCK_DEVICETREE,
	/**
	 * &GPU_DVFS_LEVEL_LOCK_COMPUTE: Compute lock
	 *
	 * This lock is used to enforce level requests for when compute-heavy work is presently
	 * running on the GPU.
	 */
	GPU_DVFS_LEVEL_LOCK_COMPUTE,
	/**
	 * &GPU_DVFS_LEVEL_LOCK_HINT: Locks set by the usermode hints
	 *
	 * This lock is intended to be updated by usermode processes that want to influence the
	 * GPU DVFS scaling range. For manual updates use &GPU_DVFS_LEVEL_LOCK_SYSFS instead.
	 */
	GPU_DVFS_LEVEL_LOCK_HINT,
	/**
	 * &GPU_DVFS_LEVEL_LOCK_SYSFS: Locks set by the user via sysfs
	 *
	 * This lock is manipulated by the user updating the scaling frequencies in the GPU's sysfs
	 * node.
	 */
	GPU_DVFS_LEVEL_LOCK_SYSFS,
#ifdef CONFIG_MALI_PIXEL_GPU_THERMAL
	/**
	 * &GPU_DVFS_LEVEL_LOCK_THERMAL: Thermal mitigation lock
	 *
	 * This lock is set when the system is in a thermal situation where the GPU frequency needs
	 * to be controlled to stay in control of device temperature.
	 */
	GPU_DVFS_LEVEL_LOCK_THERMAL,
#endif /* CONFIG_MALI_PIXEL_GPU_THERMAL */
#if IS_ENABLED(CONFIG_GOOGLE_BCL)
	/**
	 * &GPU_DVFS_LEVEL_LOCK_BCL: Battery current limitation mitigation lock
	 *
	 * This lock is set when the system is in a current limited situation where the GPU frequency
         * needs to be controlled to stay in control of the maximum amount of current the battery
         * can deliver.
	 */
	GPU_DVFS_LEVEL_LOCK_BCL,
#endif /* CONFIG_GOOGLE_BCL */
	/* Insert new level locks here */
	GPU_DVFS_LEVEL_LOCK_COUNT,
};

/**
 * struct gpu_dvfs_level_lock - A level lock on DVFS
 *
 * @level_max: The maximum throughput level allowed by this level lock. This will either be a valid
 *             level from the DVFS table, or -1 to indicate no restrictions on the maximum
 *             frequency.
 * @level_min: The minimum throughput level imposed by this level lock. This will either be a valid
 *             level from the DVFS table, or -1 to indicate no restrictions on the minimum
 *             frequency.
 */
struct gpu_dvfs_level_lock {
	int level_min;
	int level_max;
};

void gpu_dvfs_select_level(struct kbase_device *kbdev);
void gpu_dvfs_update_level_lock(struct kbase_device *kbdev,
	enum gpu_dvfs_level_lock_type lock_type, int level_min, int level_max);

/**
 * gpu_dvfs_level_lock_is_set() - Checks if a lock level is set or valid
 *
 * @value: The lock level to evaluate.
 *
 * This macro checks whether the &value indicates either a lock level that has been set and will be
 * used when evaluating the DVFS scaling range. When passed in a value passed to
 * &gpu_dvfs_update_level_lock it returns whether the caller intended the level lock associated with
 * &value to be set or not.
 *
 * Return: True if @value corresponds to a set lock level.
 */
#define gpu_dvfs_level_lock_is_set(value) \
	((value) >= 0)

/**
 * gpu_dvfs_reset_level_lock() - Resets a level lock on DVFS
 *
 * @kbdev:     The &struct kbase_device for the GPU.
 * @lock_type: The type of level lock to be reset
 *
 * This macro is a helper that resets the given level lock and ensures that DVFS lock state is
 * updated.
 *
 * Context: Process context. Expects the caller to hold the DVFS lock.
 */
#define gpu_dvfs_reset_level_lock(kbdev, lock_type) \
		gpu_dvfs_update_level_lock((kbdev), (lock_type), \
			((struct pixel_context *)((kbdev)->platform_context))->dvfs.level_min, \
			((struct pixel_context *)((kbdev)->platform_context))->dvfs.level_max)

#endif /* _PIXEL_GPU_DVFS_H_ */
