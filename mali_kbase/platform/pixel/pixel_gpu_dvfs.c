// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2020-2021 Google LLC.
 *
 * Author: Sidath Senanayake <sidaths@google.com>
 */

/* Linux includes */
#ifdef CONFIG_OF
#include <linux/of.h>
#endif
#include <trace/events/power.h>

/* SOC includes */
#if IS_ENABLED(CONFIG_CAL_IF)
#include <soc/google/cal-if.h>
#endif

/* Mali core includes */
#include <mali_kbase.h>
#include <backend/gpu/mali_kbase_pm_internal.h>

/* Pixel integration includes */
#include "mali_kbase_config_platform.h"
#include "pixel_gpu_control.h"
#include "pixel_gpu_dvfs.h"
#include "pixel_gpu_trace.h"

#define DVFS_TABLE_ROW_MAX (14)
#define DVFS_TABLES_MAX (2)
static struct gpu_dvfs_opp gpu_dvfs_table[DVFS_TABLE_ROW_MAX];

/* DVFS event handling code */

/**
 * gpu_dvfs_set_freq() - Request a frequency change for a GPU domain
 *
 * @kbdev:   &struct kbase_device for the GPU.
 * @domain:  The GPU domain that shall have it's frequency changed.
 * @level:   The frequency level to set the GPU domain to.
 *
 * Context: Expects the caller to hold the domain access lock
 *
 * Return: See cal_dfs_set_rate
 */
static int gpu_dvfs_set_freq(struct kbase_device *kbdev, enum gpu_dvfs_clk_index domain, int level)
{
	struct pixel_context *pc = kbdev->platform_context;

	lockdep_assert_held(&pc->pm.domain->access_lock);

	return cal_dfs_set_rate(pc->dvfs.clks[domain].cal_id, pc->dvfs.table[level].clk[domain]);
}

/**
 * gpu_dvfs_set_new_level() - Updates the GPU operating point.
 *
 * @kbdev:      The &struct kbase_device for the GPU.
 * @next_level: The level to set the GPU to.
 *
 * Context: Process context. Takes and releases the GPU power domain lock. Expects the caller to
 *          hold the DVFS lock.
 */
static int gpu_dvfs_set_new_level(struct kbase_device *kbdev, int next_level)
{
	struct pixel_context *pc = kbdev->platform_context;

	lockdep_assert_held(&pc->dvfs.lock);

#ifdef CONFIG_MALI_PIXEL_GPU_QOS
	/* If we are clocking up, update QOS frequencies before GPU frequencies */
	if (next_level < pc->dvfs.level)
		gpu_dvfs_qos_set(kbdev, next_level);
#endif /* CONFIG_MALI_PIXEL_GPU_QOS */

	mutex_lock(&pc->pm.domain->access_lock);

	/* We must enforce the CLK_G3DL2 >= CLK_G3D constraint.
	 * When clocking down we must set G3D CLK first to avoid violating the constraint.
	 */
	if (next_level > pc->dvfs.level) {
		gpu_dvfs_set_freq(kbdev, GPU_DVFS_CLK_SHADERS, next_level);
		gpu_dvfs_set_freq(kbdev, GPU_DVFS_CLK_TOP_LEVEL, next_level);
	} else {
		gpu_dvfs_set_freq(kbdev, GPU_DVFS_CLK_TOP_LEVEL, next_level);
		gpu_dvfs_set_freq(kbdev, GPU_DVFS_CLK_SHADERS, next_level);
	}


	mutex_unlock(&pc->pm.domain->access_lock);

	gpu_dvfs_metrics_update(kbdev, pc->dvfs.level, next_level, true);

#ifdef CONFIG_MALI_PIXEL_GPU_QOS
	/* If we are clocking down, update QOS frequencies after GPU frequencies */
	if (next_level > pc->dvfs.level)
		gpu_dvfs_qos_set(kbdev, next_level);
#endif /* CONFIG_MALI_PIXEL_GPU_QOS */

	pc->dvfs.level = next_level;

	return 0;
}

/**
 * gpu_dvfs_process_level_locks() - Processes all level locks and sets overall scaling limits
 *
 * @kbdev: The &struct kbase_device for the GPU.
 *
 * This function loops over all level locks and determines what the final DVFS scaling limits are
 * taking into account the priority levels of each level lock. It ensures that votes on minimum and
 * maximum levels originating from different level lock types are supported.
 *
 * Context: Expects the caller to hold the DVFS lock
 *
 * Note: This is the only function that should write to &level_scaling_max or &level_scaling_min.
 */
static void gpu_dvfs_process_level_locks(struct kbase_device *kbdev)
{
	struct pixel_context *pc = kbdev->platform_context;
	int lock, new_min, new_max, lock_min, lock_max;

	lockdep_assert_held(&pc->dvfs.lock);

	new_min = pc->dvfs.level_min;
	new_max = pc->dvfs.level_max;

	for (lock = 0; lock < GPU_DVFS_LEVEL_LOCK_COUNT; lock++) {
		lock_min = pc->dvfs.level_locks[lock].level_min;
		lock_max = pc->dvfs.level_locks[lock].level_max;

		if (lock_min >= 0) {
			new_min = min(new_min, lock_min);
			if (new_min < new_max)
				new_max = new_min;
		}

		if (lock_max >= 0) {
			new_max = max(new_max, lock_max);
			if (new_max > new_min)
				new_min = new_max;
		}
	}

	pc->dvfs.level_scaling_min = new_min;
	pc->dvfs.level_scaling_max = new_max;

	/* Check if the current level needs to be adjusted */
	pc->dvfs.level_target = clamp(pc->dvfs.level,
		pc->dvfs.level_scaling_max,
		pc->dvfs.level_scaling_min);
}

/**
 * gpu_dvfs_update_level_lock() - Validates and sets a new level lock parameter.
 *
 * @kbdev:         The &struct kbase_device for the GPU.
 * @lock_type:     The type of level lock that is being updated.
 * @new_level_min: The new minimum level to enforce, or -1 to indicate no change.
 * @new_level_max: The new maximum level to enforce, or -1 to indicate no change.
 *
 * This function is called to update the levels imposed by a level lock. It validates the request
 * parameters, ensures that the lock type's min and max locks are in the correct order if both are
 * specified and then calls &gpu_dvfs_process_level_locks to re-evaluate the effective scaling
 * limits on the GPU.
 *
 * If the new limit results in locks being in the wrong order, the lock that is not being set will
 * be reset. For example, if the existing max lock is 100MHz and this function is called to set the
 * min lock to be 200Mhz, the max lock is reset.
 *
 * Context: Process context. Expects the caller to hold the DVFS lock.
 */
void gpu_dvfs_update_level_lock(struct kbase_device *kbdev,
	enum gpu_dvfs_level_lock_type lock_type, int new_level_min, int new_level_max)
{
	struct pixel_context *pc = kbdev->platform_context;
	struct gpu_dvfs_level_lock *curr = &pc->dvfs.level_locks[lock_type];

	lockdep_assert_held(&pc->dvfs.lock);

	/* If both locks are set, ensure they are in the right order */
	if (unlikely(gpu_dvfs_level_lock_is_set(new_level_min) &&
		gpu_dvfs_level_lock_is_set(new_level_max)))
		if (new_level_min < new_level_max)
			return;

	/* Update the limits for the relevant level lock. If the limits end up being in the wrong
	 * order, invalidate the lock that is not being set.
	 */
	if (gpu_dvfs_level_lock_is_set(new_level_min)) {
		curr->level_min = min(new_level_min, pc->dvfs.level_min);
		if (curr->level_max > new_level_min)
			curr->level_max = -1;
	}

	if (gpu_dvfs_level_lock_is_set(new_level_max)) {
		curr->level_max = max(new_level_max, pc->dvfs.level_max);
		if (curr->level_min < new_level_max)
			curr->level_min = -1;
	}

	/* Identify when a limit is not really being set */
	if (curr->level_min == pc->dvfs.level_min)
		curr->level_min = -1;
	if (curr->level_max == pc->dvfs.level_max)
		curr->level_max = -1;

	/* Re-evaluate the effective level lock */
	gpu_dvfs_process_level_locks(kbdev);
}

/**
 * gpu_dvfs_event_power_on() - DVFS event handler for when the GPU powers on.
 *
 * @kbdev: The &struct kbase_device for the GPU.
 *
 * This function ensures DVFS level is proper, updates GPU metrics and outputs trace events to track
 * the change in power status.
 *
 * Context: Process context. Takes and releases the DVFS lock
 */
void gpu_dvfs_event_power_on(struct kbase_device *kbdev)
{
	struct pixel_context *pc = kbdev->platform_context;

	mutex_lock(&pc->dvfs.lock);
	if (pc->dvfs.level_target != pc->dvfs.level)
		gpu_dvfs_select_level(kbdev);
	else
		gpu_dvfs_metrics_update(kbdev, pc->dvfs.level,
			pc->dvfs.level_target, true);

	mutex_unlock(&pc->dvfs.lock);

	cancel_delayed_work(&pc->dvfs.clockdown_work);
}

/**
 * gpu_dvfs_event_power_off() - DVFS event handler for when the GPU powers off.
 *
 * @kbdev: The &struct kbase_device for the GPU.
 *
 * This function updates GPU metrics and outputs trace events to track the change in power status.
 *
 * Context: Process context. Takes and releases the DVFS lock.
 */
void gpu_dvfs_event_power_off(struct kbase_device *kbdev)
{
	struct pixel_context *pc = kbdev->platform_context;

	mutex_lock(&pc->dvfs.lock);
	gpu_dvfs_metrics_update(kbdev, pc->dvfs.level, 0, false);
	mutex_unlock(&pc->dvfs.lock);

	queue_delayed_work(pc->dvfs.clockdown_wq, &pc->dvfs.clockdown_work,
		pc->dvfs.clockdown_hysteresis);
}

/**
 * gpu_dvfs_clockdown_worker() - Handles the GPU post-power down timeout
 *
 * @data: Delayed worker data structure, used to determine the corresponding GPU context.
 *
 * This function is called after the GPU has been powered down for a specified duration and is
 * responsible for reverting the GPU to its default, low-throughput operating point and releasing
 * any QOS votes that were previously made.
 *
 * Context: Process context. Takes and releases the DVFS lock.
 */
static void gpu_dvfs_clockdown_worker(struct work_struct *data)
{
	struct delayed_work *dw = to_delayed_work(data);
	struct pixel_context *pc = container_of(dw, struct pixel_context, dvfs.clockdown_work);
	struct kbase_device *kbdev = pc->kbdev;

	mutex_lock(&pc->dvfs.lock);

	/* Clear any transient level locks */
	gpu_dvfs_reset_level_lock(kbdev, GPU_DVFS_LEVEL_LOCK_COMPUTE);

	pc->dvfs.level_target = pc->dvfs.level_scaling_min;
#ifdef CONFIG_MALI_PIXEL_GPU_QOS
	gpu_dvfs_qos_reset(kbdev);
#endif /* CONFIG_MALI_PIXEL_GPU_QOS */

	mutex_unlock(&pc->dvfs.lock);
}

/**
 * gpu_dvfs_set_level_locks_from_util() - Updates level locks based on GPU util.
 *
 * @kbdev:      The &struct kbase_device for the GPU.
 * @util_stats: The current GPU utilization statistics.
 *
 * This function updates level locks depending on the current utlization on the GPU. Currently it is
 * used to detect OpenCL content and set the &GPU_DVFS_LEVEL_LOCK_COMPUTE level lock accordingly.
 */
static inline void gpu_dvfs_set_level_locks_from_util(struct kbase_device *kbdev,
	struct gpu_dvfs_utlization *util_stats)
{
#if !MALI_USE_CSF
	struct pixel_context *pc = kbdev->platform_context;
	bool cl_lock_set = (pc->dvfs.level_locks[GPU_DVFS_LEVEL_LOCK_COMPUTE].level_min != -1 ||
		pc->dvfs.level_locks[GPU_DVFS_LEVEL_LOCK_COMPUTE].level_max != -1);

	/* If we detect compute-only work is running on the GPU, we enforce a minimum frequency */
	if (unlikely(util_stats->util_cl > 0 && !cl_lock_set))
		gpu_dvfs_update_level_lock(kbdev, GPU_DVFS_LEVEL_LOCK_COMPUTE,
			pc->dvfs.level_scaling_compute_min, -1);
	else if (util_stats->util_cl == 0 && cl_lock_set)
		gpu_dvfs_reset_level_lock(kbdev, GPU_DVFS_LEVEL_LOCK_COMPUTE);
#endif /* !MALI_USE_CSF */
}

/**
 * gpu_dvfs_select_level() - The main DVFS entry point for the Pixel GPU integration.
 *
 * @kbdev: The &struct kbase_device for the GPU.
 *
 * This function handles the processing of incoming GPU utilization data from the core Mali driver
 * that was passed via &kbase_platform_dvfs_event or changes to DVFS policy.
 *
 * If the GPU is powered on, the reported utilization is used to determine whether a level change is
 * required via the current governor and if so, make that change.
 *
 * If the GPU is powered off, no action is taken.
 *
 * Context: Process context. Must be called with DVFS lock.
 */
void gpu_dvfs_select_level(struct kbase_device *kbdev)
{
	struct pixel_context *pc = kbdev->platform_context;
	struct gpu_dvfs_utlization util_stats;

	if (pc->dvfs.updates_enabled && gpu_pm_get_power_state(kbdev)) {
		util_stats.util = atomic_read(&pc->dvfs.util);
#if !MALI_USE_CSF
		util_stats.util_gl = atomic_read(&pc->dvfs.util_gl);
		util_stats.util_cl = atomic_read(&pc->dvfs.util_cl);
#endif

		gpu_dvfs_set_level_locks_from_util(kbdev, &util_stats);

		pc->dvfs.level_target = gpu_dvfs_governor_get_next_level(kbdev,	&util_stats);

#ifdef CONFIG_MALI_PIXEL_GPU_QOS
		/*
		 * If we have reset our QOS requests due to the GPU going idle, and haven't
		 * changed level, we need to request the QOS values for that level again
		 */
		if (pc->dvfs.level_target == pc->dvfs.level && !pc->dvfs.qos.enabled)
			gpu_dvfs_qos_set(kbdev, pc->dvfs.level_target);
#endif /* CONFIG_MALI_PIXEL_GPU_QOS */

		if (pc->dvfs.level_target != pc->dvfs.level) {
			dev_dbg(kbdev->dev, "util=%d results in level change (%d->%d)\n",
				util_stats.util, pc->dvfs.level, pc->dvfs.level_target);
			gpu_dvfs_set_new_level(kbdev, pc->dvfs.level_target);
		}
	}
}

#ifdef CONFIG_MALI_MIDGARD_DVFS
/**
 * gpu_dvfs_disable_updates() - Ensure DVFS updates are disabled
 *
 * @kbdev: The &struct kbase_device for the GPU.
 *
 * Ensure that no dvfs updates will occurr after this call completes.
 */
void gpu_dvfs_disable_updates(struct kbase_device *kbdev) {
	struct pixel_context *pc = kbdev->platform_context;

	mutex_lock(&pc->dvfs.lock);
	pc->dvfs.updates_enabled = false;
	mutex_unlock(&pc->dvfs.lock);

	flush_workqueue(pc->dvfs.control_wq);
}

/**
 * gpu_dvfs_enable_updates() - Ensure DVFS updates are enabled
 *
 * @kbdev: The &struct kbase_device for the GPU.
 *
 * Ensure that dvfs updates will occurr after this call completes, undoing the effect of
 * gpu_dvfs_disable_updates().
 */
void gpu_dvfs_enable_updates(struct kbase_device *kbdev) {
	struct pixel_context *pc = kbdev->platform_context;

	mutex_lock(&pc->dvfs.lock);
	pc->dvfs.updates_enabled = true;
	mutex_unlock(&pc->dvfs.lock);
}
#endif

/**
 * gpu_dvfs_control_worker() - The workqueue worker that changes DVFS on utilization change.
 *
 * @data: Worker data structure, used to determine the corresponding GPU context.
 *
 * This function handles the processing of incoming GPU utilization data by extracting wq entry and
 * passing to gpu_dvfs_select_level().
 */
static void gpu_dvfs_control_worker(struct work_struct *data)
{
	struct pixel_context *pc = container_of(data, struct pixel_context, dvfs.control_work);
	mutex_lock(&pc->dvfs.lock);
	gpu_dvfs_select_level(pc->kbdev);
	mutex_unlock(&pc->dvfs.lock);
}

#if MALI_USE_CSF
/**
 * kbase_platform_dvfs_event() - Callback from Mali driver to report updated utilization metrics.
 *
 * @kbdev:         The &struct kbase_device for the GPU.
 * @utilisation:   The calculated utilization as measured by the core Mali driver's metrics system.
 *
 * This is the function that bridges the core Mali driver and the Pixel integration code. As this is
 * made in interrupt context, it is swiftly handed off to a work_queue for further processing.
 *
 * Context: Interrupt context.
 *
 * Return: Returns 1 to signal success as specified in mali_kbase_pm_internal.h.
 */
int kbase_platform_dvfs_event(struct kbase_device *kbdev, u32 utilisation)
{
	struct pixel_context *pc = kbdev->platform_context;
	int proc = raw_smp_processor_id();

	/* TODO (b/187175695): Report this data via a custom ftrace event instead */
	trace_clock_set_rate("gpu_util", utilisation, proc);

	atomic_set(&pc->dvfs.util, utilisation);
	queue_work(pc->dvfs.control_wq, &pc->dvfs.control_work);

	return 1;
}
#else /* MALI_USE_CSF */
/**
 * kbase_platform_dvfs_event() - Callback from Mali driver to report updated utilization metrics.
 *
 * @kbdev:         The &struct kbase_device for the GPU.
 * @utilisation:   The calculated utilization as measured by the core Mali driver's metrics system.
 * @util_gl_share: The calculated GL share of utilization.
 * @util_cl_share: The calculated CL share of utilization per core group.
 *
 * This is the function that bridges the core Mali driver and the Pixel integration code. As this is
 * made in interrupt context, it is swiftly handed off to a work_queue for further processing.
 *
 * Context: Interrupt context.
 *
 * Return: Returns 1 to signal success as specified in mali_kbase_pm_internal.h.
 */
int kbase_platform_dvfs_event(struct kbase_device *kbdev, u32 utilisation,
	u32 util_gl_share, u32 util_cl_share[2])
{
	struct pixel_context *pc = kbdev->platform_context;
	int proc = raw_smp_processor_id();

	/* TODO (b/187175695): Report this data via a custom ftrace event instead */
	trace_clock_set_rate("gpu_util", utilisation, proc);
	trace_clock_set_rate("gpu_util_gl", util_gl_share, proc);
	trace_clock_set_rate("gpu_util_cl", util_cl_share[0] + util_cl_share[1], proc);

	atomic_set(&pc->dvfs.util, utilisation);
	atomic_set(&pc->dvfs.util_gl, util_gl_share);
	atomic_set(&pc->dvfs.util_cl, util_cl_share[0] + util_cl_share[1]);
	queue_work(pc->dvfs.control_wq, &pc->dvfs.control_work);

	return 1;
}
#endif

/* Initialization code */

/**
 * find_voltage_for_freq() - Retrieves voltage for a frequency from ECT.
 *
 * @kbdev:      The &struct kbase_device for the GPU.
 * @clock:      The frequency to search for.
 * @vol:        A pointer into which the voltage, if found, will be written.
 * @arr:        The &struct dvfs_rate_volt array to search through.
 * @arr_length: The size of @arr.
 *
 * Return: Returns 0 on success, -ENOENT if @clock doesn't exist in ECT.
 */
static int find_voltage_for_freq(struct kbase_device *kbdev, unsigned int clock,
	unsigned int *vol, struct dvfs_rate_volt *arr, unsigned int arr_length)
{
	int i;

	for (i = 0; i < arr_length; i++) {
		if (arr[i].rate == clock) {
			if (vol)
				*vol = arr[i].volt;
			return 0;
		}
	}

	return -ENOENT;
}

/**
 * validate_and_parse_dvfs_table() - Validate and populate the GPU's DVFS table from DT.
 *
 * @kbdev: The &struct kbase_device for the GPU.
 * @dvfs_table_num: DVFS table number to be validated and parsed.
 *
 * This function reads data out of the GPU's device tree entry, validates it, and
 * uses it to populate &gpu_dvfs_table. For each entry in the DVFS table, it makes
 * calls to determine voltages from ECT. It also checks for any level locks specified
 * in the devicetree and ensures that the effective scaling range is set up.
 *
 * This function will fail if the particular dvfs table's operating points does not
 * match the ECT table for the device.
 *
 * Return: Returns the number of opertaing points in the DVFS table on success, -EINVAL on failure.
 */
static int validate_and_parse_dvfs_table(struct kbase_device *kbdev, int dvfs_table_num)
{
	char table_name[64];
	char table_size_name[64];

	int i, idx, c;
	int of_data_int_array[OF_DATA_NUM_MAX];
	int dvfs_table_row_num = 0, dvfs_table_col_num = 0;
	int dvfs_table_size = 0;
	int scaling_level_max = -1, scaling_level_min = -1;
	int scaling_freq_max_devicetree = INT_MAX;
	int scaling_freq_min_devicetree = 0;
	int scaling_freq_min_compute = 0;
	int level_count[GPU_DVFS_CLK_COUNT];
	struct dvfs_rate_volt vf_map[GPU_DVFS_CLK_COUNT][16];

	struct device_node *np = kbdev->dev->of_node;
	struct pixel_context *pc = kbdev->platform_context;

	/* Get frequency -> voltage mapping */
	for (c = 0; c < GPU_DVFS_CLK_COUNT; c++) {
		level_count[c] = cal_dfs_get_lv_num(pc->dvfs.clks[c].cal_id);
		if (!cal_dfs_get_rate_asv_table(pc->dvfs.clks[c].cal_id, vf_map[c])) {
			dev_err(kbdev->dev, "failed to get gpu%d ASV table\n", c);
			goto err;
		}
	}

	sprintf(table_size_name, "gpu_dvfs_table_size_v%d", dvfs_table_num);
	if (of_property_read_u32_array(np, table_size_name, of_data_int_array, 2))
		goto err;

	dvfs_table_row_num = of_data_int_array[0];
	dvfs_table_col_num = of_data_int_array[1];
	dvfs_table_size = dvfs_table_row_num * dvfs_table_col_num;

	if (dvfs_table_row_num > DVFS_TABLE_ROW_MAX) {
		dev_err(kbdev->dev,
			"DVFS table %d has %d rows but only up to %d are supported",
			dvfs_table_num, dvfs_table_row_num, DVFS_TABLE_ROW_MAX);
		goto err;
	}

	if (dvfs_table_size > OF_DATA_NUM_MAX) {
		dev_err(kbdev->dev, "DVFS table %d is too big", dvfs_table_num);
		goto err;
	}
	sprintf(table_name, "gpu_dvfs_table_v%d", dvfs_table_num);
	if (of_property_read_u32_array(np, table_name, of_data_int_array, dvfs_table_size))
		goto err;

	of_property_read_u32(np, "gpu_dvfs_max_freq", &scaling_freq_max_devicetree);
	of_property_read_u32(np, "gpu_dvfs_min_freq", &scaling_freq_min_devicetree);
	of_property_read_u32(np, "gpu_dvfs_min_freq_compute", &scaling_freq_min_compute);

	/* Check if there is a voltage mapping for each frequency in the ECT table */
	for (i = 0; i < dvfs_table_row_num; i++) {
		idx = i * dvfs_table_col_num;

		/* Get and validate voltages from cal-if */
		for (c = 0; c < GPU_DVFS_CLK_COUNT; c++) {
			if (find_voltage_for_freq(kbdev, of_data_int_array[idx + c],
				NULL, vf_map[c], level_count[c])) {
				dev_dbg(kbdev->dev,
					"Failed to find voltage for clock %u frequency %u in gpu_dvfs_table_v%d\n",
					c, of_data_int_array[idx + c], dvfs_table_num);
				goto err;
			}
		}
	}

	/* Process DVFS table data from device tree and store it in OPP table */
	for (i = 0; i < dvfs_table_row_num; i++) {
		idx = i * dvfs_table_col_num;

		/* Read raw data from device tree table */
		gpu_dvfs_table[i].clk[GPU_DVFS_CLK_TOP_LEVEL] = of_data_int_array[idx + 0];
		gpu_dvfs_table[i].clk[GPU_DVFS_CLK_SHADERS]   = of_data_int_array[idx + 1];

		for (c = 0; c < GPU_DVFS_CLK_COUNT; c++) {
			find_voltage_for_freq(kbdev, gpu_dvfs_table[i].clk[c],
				&(gpu_dvfs_table[i].vol[c]), vf_map[c], level_count[c]);
		}

		gpu_dvfs_table[i].util_min     = of_data_int_array[idx + 2];
		gpu_dvfs_table[i].util_max     = of_data_int_array[idx + 3];
		gpu_dvfs_table[i].hysteresis   = of_data_int_array[idx + 4];

		gpu_dvfs_table[i].qos.int_min  = of_data_int_array[idx + 5];
		gpu_dvfs_table[i].qos.mif_min  = of_data_int_array[idx + 6];
		gpu_dvfs_table[i].qos.cpu0_min = of_data_int_array[idx + 7];
		gpu_dvfs_table[i].qos.cpu1_min = of_data_int_array[idx + 8];
		gpu_dvfs_table[i].qos.cpu2_max = of_data_int_array[idx + 9];

		/* Handle case where CPU cluster 2 has no limit set */
		if (!gpu_dvfs_table[i].qos.cpu2_max)
			gpu_dvfs_table[i].qos.cpu2_max = CPU_FREQ_MAX;

		/* Update level locks */
		if (gpu_dvfs_table[i].clk[GPU_DVFS_CLK_SHADERS] <= scaling_freq_max_devicetree)
			if (scaling_level_max == -1)
				scaling_level_max = i;

		if (gpu_dvfs_table[i].clk[GPU_DVFS_CLK_SHADERS] >= scaling_freq_min_devicetree)
			scaling_level_min = i;

		if (gpu_dvfs_table[i].clk[GPU_DVFS_CLK_SHADERS] >= scaling_freq_min_compute)
			pc->dvfs.level_scaling_compute_min = i;
	}

	pc->dvfs.level_max = 0;
	pc->dvfs.level_min = dvfs_table_row_num - 1;
	gpu_dvfs_update_level_lock(kbdev, GPU_DVFS_LEVEL_LOCK_DEVICETREE,
		scaling_level_min, scaling_level_max);

	return dvfs_table_row_num;

err:
	return -EINVAL;
}

/**
 * gpu_dvfs_update_asv_table() - Populate the GPU's DVFS table from DT.
 *
 * @kbdev: The &struct kbase_device for the GPU.
 *
 * This function iterates through the list of DVFS tables available in the device tree
 * and calls validate_and_parse_dvfs_table() to select the valid one for the device.
 *
 * This function will fail if the required data is not present in the GPU's device tree entry.
 *
 * Context: Expects the caller to hold the DVFS lock
 *
 * Return: Returns the number of opertaing points in the DVFS table on success, -EINVAL on failure.
 */
static int gpu_dvfs_update_asv_table(struct kbase_device *kbdev)
{
	int dvfs_table_idx, dvfs_table_row_num;
	struct pixel_context *pc = kbdev->platform_context;

	lockdep_assert_held(&pc->dvfs.lock);

	for (dvfs_table_idx = DVFS_TABLES_MAX; dvfs_table_idx > 0; dvfs_table_idx--) {
		dvfs_table_row_num = validate_and_parse_dvfs_table(kbdev, dvfs_table_idx);
		if (dvfs_table_row_num > 0)
			break;
	}
	if (dvfs_table_row_num <= 0) {
		dev_err(kbdev->dev, "failed to set GPU DVFS table");
	}

	return dvfs_table_row_num;
}

/**
 * gpu_dvfs_set_initial_level() - Set the initial GPU clocks
 *
 * @kbdev: The &struct kbase_device for the GPU
 *
 * This function sets the G3DL2 and G3D clocks to the values corresponding to the lowest throughput
 * level in the DVFS table.
 *
 * Return: 0 on success, or an error value on failure.
 */
static int gpu_dvfs_set_initial_level(struct kbase_device *kbdev)
{
	int level, ret, c;
	struct pixel_context *pc = kbdev->platform_context;

	level = pc->dvfs.level_min;

	dev_dbg(kbdev->dev,
		"Attempting to set GPU boot clocks to (gpu0: %d, gpu1: %d)\n",
		pc->dvfs.table[level].clk[GPU_DVFS_CLK_TOP_LEVEL],
		pc->dvfs.table[level].clk[GPU_DVFS_CLK_SHADERS]);

	mutex_lock(&pc->pm.domain->access_lock);

	for (c = 0; c < GPU_DVFS_CLK_COUNT; c++) {
		ret = gpu_dvfs_set_freq(kbdev, c, level);
		if (ret) {
			dev_err(kbdev->dev,
				"Failed to set boot frequency %d on clock index %d (err: %d)\n",
				pc->dvfs.table[level].clk[c], c, ret);
			break;
		}
	}

	mutex_unlock(&pc->pm.domain->access_lock);

	return ret;
}

/**
 * gpu_dvfs_init() - Initializes the Pixel GPU DVFS system.
 *
 * @kbdev: The &struct kbase_device for the GPU.
 *
 * Depending on the compile time options set, this function calls initializers for the subsystems
 * related to GPU DVFS: governors, metrics, qos & tmu.
 *
 * Return: On success, returns 0. -EINVAL on error.
 */
int gpu_dvfs_init(struct kbase_device *kbdev)
{
	int i, ret = 0;
	struct pixel_context *pc = kbdev->platform_context;
	struct device_node *np = kbdev->dev->of_node;

	/* Initialize lock */
	mutex_init(&pc->dvfs.lock);

	/* Initialize DVFS fields that are non-zero */
	for (i = 0; i < GPU_DVFS_LEVEL_LOCK_COUNT; i++) {
		pc->dvfs.level_locks[i].level_min = -1;
		pc->dvfs.level_locks[i].level_max = -1;
	}

	pc->dvfs.updates_enabled = true;

	/* Get data from DT */
	if (of_property_read_u32(np, "gpu0_cmu_cal_id",
		&pc->dvfs.clks[GPU_DVFS_CLK_TOP_LEVEL].cal_id) ||
	    of_property_read_u32(np, "gpu1_cmu_cal_id",
		&pc->dvfs.clks[GPU_DVFS_CLK_SHADERS].cal_id)) {
		ret = -EINVAL;
		goto done;
	}

	/* Get the ASV table */
	mutex_lock(&pc->dvfs.lock);
	pc->dvfs.table_size = gpu_dvfs_update_asv_table(kbdev);
	mutex_unlock(&pc->dvfs.lock);
	if (pc->dvfs.table_size < 0) {
		ret = -EINVAL;
		goto done;
	}

	pc->dvfs.table = gpu_dvfs_table;

	/* Set up initial level state */
	pc->dvfs.level = pc->dvfs.level_min;
	pc->dvfs.level_target = pc->dvfs.level_min;
	if (gpu_dvfs_set_initial_level(kbdev)) {
		ret = -EINVAL;
		goto done;
	}

	/* Setup dvfs step up value */
	if (of_property_read_u32(np, "gpu_dvfs_step_up_val", &pc->dvfs.step_up_val)) {
		ret = -EINVAL;
		goto done;
	}

	/* Initialize power down hysteresis */
	if (of_property_read_u32(np, "gpu_dvfs_clockdown_hysteresis",
		&pc->dvfs.clockdown_hysteresis)) {
		dev_err(kbdev->dev, "DVFS clock down hysteresis not set in DT\n");
		ret = -EINVAL;
		goto done;
	}
	atomic_set(&pc->dvfs.util, 0);

	/* Initialize DVFS governors */
	ret = gpu_dvfs_governor_init(kbdev);
	if (ret) {
		dev_err(kbdev->dev, "DVFS governor init failed\n");
		goto done;
	}

	/* Initialize DVFS metrics */
	ret = gpu_dvfs_metrics_init(kbdev);
	if (ret) {
		dev_err(kbdev->dev, "DVFS metrics init failed\n");
		goto fail_metrics_init;
	}

#ifdef CONFIG_MALI_PIXEL_GPU_QOS
	/* Initialize QOS framework */
	ret = gpu_dvfs_qos_init(kbdev);
	if (ret) {
		dev_err(kbdev->dev, "DVFS QOS init failed\n");
		goto fail_qos_init;
	}
#endif /* CONFIG_MALI_PIXEL_GPU_QOS */

#ifdef CONFIG_MALI_PIXEL_GPU_THERMAL
	/* Initialize thermal framework */
	ret = gpu_tmu_init(kbdev);
	if (ret) {
		dev_err(kbdev->dev, "DVFS thermal init failed\n");
		goto fail_tmu_init;
	}
#endif /* CONFIG_MALI_PIXEL_GPU_THERMAL */

	/* Initialize workqueues */
	pc->dvfs.control_wq = create_singlethread_workqueue("gpu-dvfs-control");
	INIT_WORK(&pc->dvfs.control_work, gpu_dvfs_control_worker);

	pc->dvfs.clockdown_wq = create_singlethread_workqueue("gpu-dvfs-clockdown");
	INIT_DELAYED_WORK(&pc->dvfs.clockdown_work, gpu_dvfs_clockdown_worker);

	/* Initialization was successful */
	goto done;

#ifdef CONFIG_MALI_PIXEL_GPU_THERMAL
fail_tmu_init:
#ifdef CONFIG_MALI_PIXEL_GPU_QOS
	gpu_dvfs_qos_term(kbdev);
#endif /* CONFIG_MALI_PIXEL_GPU_QOS */
#endif /* CONFIG_MALI_PIXEL_GPU_THERMAL*/

#ifdef CONFIG_MALI_PIXEL_GPU_QOS
fail_qos_init:
#endif /* CONFIG_MALI_PIXEL_GPU_QOS */
	gpu_dvfs_metrics_term(kbdev);

fail_metrics_init:
	gpu_dvfs_governor_term(kbdev);

done:
	return ret;
}

/**
 * gpu_dvfs_term() - Terminates the Pixel GPU DVFS system.
 *
 * @kbdev: The &struct kbase_device for the GPU.
 */
void gpu_dvfs_term(struct kbase_device *kbdev)
{
	struct pixel_context *pc = kbdev->platform_context;

	cancel_delayed_work_sync(&pc->dvfs.clockdown_work);
	destroy_workqueue(pc->dvfs.clockdown_wq);

	cancel_work_sync(&pc->dvfs.control_work);
	destroy_workqueue(pc->dvfs.control_wq);

#ifdef CONFIG_MALI_PIXEL_GPU_THERMAL
	gpu_tmu_term(kbdev);
#endif /* CONFIG_MALI_PIXEL_GPU_THERMAL */
#ifdef CONFIG_MALI_PIXEL_GPU_QOS
	gpu_dvfs_qos_term(kbdev);
#endif /* CONFIG_MALI_PIXEL_GPU_QOS */
	gpu_dvfs_metrics_term(kbdev);
	gpu_dvfs_governor_term(kbdev);
}
