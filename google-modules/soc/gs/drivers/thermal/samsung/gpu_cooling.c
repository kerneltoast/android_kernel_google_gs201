// SPDX-License-Identifier: GPL-2.0-only
/*
 * gpu_cooling.c - GPU cooling device
 *
 * Copyright (C) 2017 Samsung Electronics
 * Copyright 2020 Google LLC.
 *
 * Author: Sidath Senanayake <sidaths@google.com>
 *
 */
#include <linux/thermal.h>
#include <linux/cpufreq.h>

#include <soc/google/cal-if.h>
#include <soc/google/ect_parser.h>
#include <soc/google/gpu_cooling.h>
#include <soc/google/tmu.h>

#include <thermal_core.h>

#define CREATE_TRACE_POINTS
#include <trace/events/thermal_exynos_gpu.h>
#undef CREATE_TRACE_POINTS
#include <trace/events/power.h>

/**
 * struct power_table - Stores a frequency to power translation
 *
 * @frequency:	frequency in KHz
 * @power:	power in mW
 *
 */
struct power_table {
	u32 frequency;
	u32 power;
};

/**
 * struct gpufreq_cooling_device - data for cooling device with gpufreq
 *
 * @id:           unique identifier for thie GPU cooling device
 * @cool_dev:     &thermal_cooling_device backing this GPU cooling device
 * @tzd:          &thermal zone device backing this GPU cooling device
 *
 * @gpu_drv_data: Data passed unmodified to GPU query functions
 * @gpu_fns:      Function pointer block of GPU query functions for this GPU
 *
 * @gpu_freq_table: Table storing the frequency table available for this device
 * @gpufreq_state:  Stores the current cooling state for this device
 * @last_load:      The last seen utilization/load on the GPU
 *
 * @dyn_power_table: Stores the dynamic frequency -> power table for this GPU
 * @dyn_power_table_entries: The size of &dyn_power_table
 *
 * @var_table: Stores the voltage/temp -> static power table calculated from
 *             ECT data
 * @var_coeff: Stores a copy of the GPU's voltage->temp table from ECT
 * @asv_coeff: Stores a copy of the GPU's ASV coefficients from ECT
 *
 * @var_volt_size: The size of the voltage dimension in the volt/temp table
 * @var_temp_size: The size of the temperature dimension in the volt/temp table
 *
 * This structure is stores data associated with a particular
 * &gpufreq_cooling_device.
 */
struct gpufreq_cooling_device {
	int id;
	struct thermal_cooling_device *cool_dev;
	struct thermal_zone_device *tzd;
	void *gpu_drv_data;
	struct gpufreq_cooling_query_fns *gpu_fns;
	struct cpufreq_frequency_table *gpu_freq_table;
	unsigned long gpufreq_state;
	u32 last_load;
	struct power_table *dyn_power_table;
	int dyn_power_table_entries;
	int *var_table;
	int *var_coeff;
	int *asv_coeff;
	unsigned int var_volt_size;
	unsigned int var_temp_size;
	unsigned long sysfs_req;
};

static DEFINE_IDR(gpufreq_idr);
static DEFINE_MUTEX(gpu_freq_idr_lock);
static BLOCKING_NOTIFIER_HEAD(gpu_notifier);

/**
 * get_idr() - function to get a unique id.
 *
 * @idr: handle used to create a id.
 * @id:  stores the generated ID
 *
 * This function will populate @id with an unique ID
 *
 * Return: 0 on success, an error code on failure.
 */
static int get_idr(struct idr *idr, int *id)
{
	int ret;

	mutex_lock(&gpu_freq_idr_lock);
	ret = idr_alloc(idr, NULL, 0, 0, GFP_KERNEL);
	mutex_unlock(&gpu_freq_idr_lock);
	if (unlikely(ret < 0))
		return ret;
	*id = ret;

	return 0;
}

/**
 * release_idr() - function to free the unique id.
 *
 * @idr: struct idr * handle used for creating the id.
 * @id: int value representing the unique id.
 */
static void release_idr(struct idr *idr, int id)
{
	mutex_lock(&gpu_freq_idr_lock);
	idr_remove(idr, id);
	mutex_unlock(&gpu_freq_idr_lock);
}

/* Below code defines functions to be used for gpufreq as cooling device */

enum gpufreq_cooling_property {
	GET_LEVEL,
	GET_FREQ,
	GET_MAXL,
};

/**
 * get_property() - fetch a property of interest for a given GPU
 *
 * @gpufreq_cdev: GPU cooling device for which the property is required
 * @input:        query parameter
 * @output:       query return
 * @property:     type of query (frequency, level, max level)
 *
 * This is the common function to
 *   1. get maximum gpu cooling states
 *   2. translate frequency to cooling state
 *   3. translate cooling state to frequency
 *
 * Return: 0 on success, -EINVAL when invalid parameters are passed.
 */
static int get_property(struct gpufreq_cooling_device *gpufreq_cdev, unsigned long input,
			unsigned int *output, enum gpufreq_cooling_property property)
{
	int i = 0;
	unsigned long max_level = 0, level = 0;
	unsigned int freq = CPUFREQ_ENTRY_INVALID;
	int descend = -1;
	struct cpufreq_frequency_table *pos;
	struct cpufreq_frequency_table *table = gpufreq_cdev->gpu_freq_table;

	if (!output)
		return -EINVAL;

	cpufreq_for_each_valid_entry(pos, table) {
		/* ignore duplicate entry */
		if (freq == pos->frequency)
			continue;

		/* get the frequency order */
		if (freq != CPUFREQ_ENTRY_INVALID && descend == -1)
			descend = freq > pos->frequency;

		freq = pos->frequency;
		max_level++;
	}

	/* No valid cpu frequency entry */
	if (max_level == 0)
		return -EINVAL;

	/* max_level is an index, not a counter */
	max_level--;

	/* get max level */
	if (property == GET_MAXL) {
		*output = (unsigned int)max_level;
		return 0;
	}

	if (property == GET_FREQ)
		level = descend ? input : (max_level - input);

	cpufreq_for_each_valid_entry(pos, table) {
		/* ignore duplicate entry */
		if (freq == pos->frequency)
			continue;

		/* now we have a valid frequency entry */
		freq = pos->frequency;

		if (property == GET_LEVEL && (unsigned int)input == freq) {
			/* get level by frequency */
			*output = (unsigned int)(descend ? i : (max_level - i));
			return 0;
		}
		if (property == GET_FREQ && level == i) {
			/* get frequency by level */
			*output = freq;
			return 0;
		}
		i++;
	}

	return -EINVAL;
}

/**
 * gpufreq_cooling_get_level() - for a give gpu, return the cooling level.
 *
 * @gpu:  gpu for which the level is required
 * @freq: the frequency of interest
 *
 * This function will match the cooling level corresponding to the
 * requested @freq and return it.
 *
 * Return: The matched cooling level on success or THERMAL_CSTATE_INVALID
 *         otherwise.
 */
static unsigned long gpufreq_cooling_get_level(struct gpufreq_cooling_device *gpufreq_cdev,
	unsigned int freq)
{
	unsigned int val;

	if (get_property(gpufreq_cdev, (unsigned long)freq, &val, GET_LEVEL))
		return THERMAL_CSTATE_INVALID;

	return (unsigned long)val;
}

/**
 * build_dyn_power_table() - create a dynamic power to frequency table
 *
 * @gpufreq_cdev: the gpufreq cooling device in which to store the table
 * @capacitance:  dynamic power coefficient for these gpus
 *
 * Build a dynamic power to frequency table for this gpu and store it
 * in @gpufreq_cdev.  This table will be used in gpu_power_to_freq() and
 * gpu_freq_to_power() to convert between power and frequency
 * efficiently.  Power is stored in mW, frequency in KHz.  The
 * resulting table is in ascending order.
 *
 * Return: 0 on success, -EINVAL if there are no OPPs or -ENOMEM if memmory
 *         allocation requests fail.
 */
static int build_dyn_power_table(struct gpufreq_cooling_device *gpufreq_cdev,
				 u32 capacitance)
{
	struct power_table *power_table;
	int num_opps = 0, i, cnt = 0;
	unsigned int freq;

	num_opps = gpufreq_cdev->gpu_fns->get_num_levels(gpufreq_cdev->gpu_drv_data);

	if (num_opps == 0)
		return -EINVAL;

	power_table = kcalloc(num_opps, sizeof(*power_table), GFP_KERNEL);
	if (!power_table)
		return -ENOMEM;

	for (freq = 0, i = 0; i < num_opps; i++) {
		unsigned int voltage_mv;
		u64 power;

		/* Get frequency and voltage */
		if (gpufreq_cdev->gpu_fns->get_freqs_for_level(gpufreq_cdev->gpu_drv_data,
			num_opps - i - 1, NULL, &freq))
			return -EINVAL;

		if (gpufreq_cdev->gpu_fns->get_vols_for_level(gpufreq_cdev->gpu_drv_data,
			num_opps - i - 1, NULL, &voltage_mv))
			return -EINVAL;

		voltage_mv /= 1000;

		/*
		 * Do the multiplication with MHz and millivolt so as
		 * to not overflow.
		 */
		power = (u64)capacitance * (freq / 1000) * voltage_mv * voltage_mv;
		do_div(power, 1000000000);

		power_table[i].frequency = freq;

		/* power is stored in mW */
		power_table[i].power = power;
		cnt++;
	}

	gpufreq_cdev->dyn_power_table = power_table;
	gpufreq_cdev->dyn_power_table_entries = cnt;

	return 0;
}

/**
 * build_static_power_table() - create static power table from ECT data
 *
 * @np:           The device tree node for the GPU
 * @gpufreq_cdev: The GPU cooling device to associate the power table with
 *
 * This function queries ECT for the GPU defined in &np and populates static
 * power calculation data for this GPU cooling device based on the chip's ASV.
 *
 * Return: 0 on success, or an error code on failure.
 */
static int build_static_power_table(struct device_node *np,
				    struct gpufreq_cooling_device *gpufreq_cdev)
{
	int i, j;
	int ratio = 0, asv_group = 0, cal_id = 0, ret = 0;
	void *gen_block;
	struct ect_gen_param_table *volt_temp_param = NULL, *asv_param = NULL;
	int ratio_table[16] = { 0, 25, 29, 35, 41, 48, 57, 67, 79, 94,
				110, 130, 151, 162, 162, 162};

	/* The GPU has two clock domains, represented by gpu0_cmu_cal_id and
	 * gpu1_cmu_cal_id. The second clock drives the shader stacks which
	 * comprise the majority of silicon in the GPU and accounts for the vast
	 * majority of the GPU's power usage. We therefore use it to determine
	 * power values here.
	 */
	ret = of_property_read_u32(np, "gpu1_cmu_cal_id", &cal_id);
	if (ret) {
		pr_err("%s: Failed to get cal-id\n", __func__);
		return -EINVAL;
	}

	ratio = cal_asv_get_ids_info(cal_id);
	asv_group = cal_asv_get_grp(cal_id);

	if (asv_group < 0 || asv_group > 15)
		asv_group = 0;

	if (!ratio)
		ratio = ratio_table[asv_group];

	gen_block = ect_get_block("GEN");
	if (!gen_block) {
		pr_err("%s: Failed to get gen block from ECT\n", __func__);
		return -EINVAL;
	}

	volt_temp_param = ect_gen_param_get_table(gen_block, "DTM_G3D_VOLT_TEMP");
	asv_param = ect_gen_param_get_table(gen_block, "DTM_G3D_ASV");

	if (volt_temp_param && asv_param) {
		gpufreq_cdev->var_volt_size = volt_temp_param->num_of_row - 1;
		gpufreq_cdev->var_temp_size = volt_temp_param->num_of_col - 1;

		gpufreq_cdev->var_coeff = kzalloc(sizeof(int) *
							volt_temp_param->num_of_row *
							volt_temp_param->num_of_col,
							GFP_KERNEL);
		if (!gpufreq_cdev->var_coeff)
			goto err_mem;

		gpufreq_cdev->asv_coeff = kzalloc(sizeof(int) *
							asv_param->num_of_row *
							asv_param->num_of_col,
							GFP_KERNEL);
		if (!gpufreq_cdev->asv_coeff)
			goto free_var_coeff;

		gpufreq_cdev->var_table = kzalloc(sizeof(int) *
							volt_temp_param->num_of_row *
							volt_temp_param->num_of_col,
							GFP_KERNEL);
		if (!gpufreq_cdev->var_table)
			goto free_asv_coeff;

		memcpy(gpufreq_cdev->var_coeff, volt_temp_param->parameter,
		       sizeof(int) * volt_temp_param->num_of_row * volt_temp_param->num_of_col);
		memcpy(gpufreq_cdev->asv_coeff, asv_param->parameter,
		       sizeof(int) * asv_param->num_of_row * asv_param->num_of_col);
		memcpy(gpufreq_cdev->var_table, volt_temp_param->parameter,
		       sizeof(int) * volt_temp_param->num_of_row * volt_temp_param->num_of_col);
	} else {
		pr_err("%s: Failed to get param table from ECT\n", __func__);
		return -EINVAL;
	}

	for (i = 1; i <= gpufreq_cdev->var_volt_size; i++) {
		long asv_coeff = (long)gpufreq_cdev->asv_coeff[3 * i + 0] * asv_group * asv_group
				+ (long)gpufreq_cdev->asv_coeff[3 * i + 1] * asv_group
				+ (long)gpufreq_cdev->asv_coeff[3 * i + 2];
		asv_coeff = asv_coeff / 100;

		for (j = 1; j <= gpufreq_cdev->var_temp_size; j++) {
			long var_coeff = (long)gpufreq_cdev->var_coeff[i
							* (gpufreq_cdev->var_temp_size + 1) + j];

			var_coeff = ratio * var_coeff * asv_coeff;
			var_coeff = var_coeff / 100000;
			gpufreq_cdev->var_table[i * (gpufreq_cdev->var_temp_size + 1) + j] =
										(int)var_coeff;
		}
	}

	return 0;

free_asv_coeff:
	kfree(gpufreq_cdev->asv_coeff);
free_var_coeff:
	kfree(gpufreq_cdev->var_coeff);
err_mem:
	return -ENOMEM;
}

/**
 * lookup_static_power() - Returns the static power corresponding to the given
 *                         voltage/temperature combination
 *
 * @gpufreq_cdev: Pointer to a valid GPU cooling device
 * @voltage:      The voltage in mV
 * @temperature:  The temperature in millicelsius
 * @power:        The returned power in mW
 *
 * Return: Always returns 0
 */
static int lookup_static_power(struct gpufreq_cooling_device *gpufreq_cdev,
			       unsigned long voltage, int temperature, u32 *power)
{
	int volt_index = 0, temp_index = 0;
	int index = 0;

	voltage = voltage / 1000;
	temperature  = temperature / 1000;

	for (volt_index = 0; volt_index <= gpufreq_cdev->var_volt_size; volt_index++) {
		if (voltage < gpufreq_cdev->var_table[volt_index
						* ((int)gpufreq_cdev->var_temp_size + 1)]) {
			volt_index = volt_index - 1;
			break;
		}
	}

	if (volt_index == 0)
		volt_index = 1;

	if (volt_index > gpufreq_cdev->var_volt_size)
		volt_index = gpufreq_cdev->var_volt_size;

	for (temp_index = 0; temp_index <= gpufreq_cdev->var_temp_size; temp_index++) {
		if (temperature < gpufreq_cdev->var_table[temp_index]) {
			temp_index = temp_index - 1;
			break;
		}
	}

	if (temp_index == 0)
		temp_index = 1;

	if (temp_index > gpufreq_cdev->var_temp_size)
		temp_index = gpufreq_cdev->var_temp_size;

	index = (int)(volt_index * (gpufreq_cdev->var_temp_size + 1) + temp_index);
	*power = (unsigned int)gpufreq_cdev->var_table[index];

	return 0;
}

/**
 * gpu_freq_to_power() - Returns the dynamic power corresponding to a frequency
 *
 * @gpufreq_cdev: Pointer to a valid GPU cooling device
 * @freq:         The frequency in KHz.
 *
 * Return: The corresponding power in mW
 */
static u32 gpu_freq_to_power(struct gpufreq_cooling_device *gpufreq_cdev,
			     u32 freq)
{
	int i;
	struct power_table *pt = gpufreq_cdev->dyn_power_table;

	for (i = 1; i < gpufreq_cdev->dyn_power_table_entries; i++)
		if (freq < pt[i].frequency)
			break;

	return pt[i - 1].power;
}

/**
 * gpu_power_to_freq() - Returns the frequency corresponding to a dynamic power level
 *
 * @gpufreq_cdev: Pointer to a valid GPU cooling device
 * @power:        The power level in mW.
 *
 * Return: The frequency in KHz.
 */
static u32 gpu_power_to_freq(struct gpufreq_cooling_device *gpufreq_cdev,
			     u32 power)
{
	int i;
	struct power_table *pt = gpufreq_cdev->dyn_power_table;

	for (i = 1; i < gpufreq_cdev->dyn_power_table_entries; i++)
		if (power < pt[i].power)
			break;

	return pt[i - 1].frequency;
}

/**
 * get_static_power() - calculate the static power consumed by the gpus
 *
 * @gpufreq_cdev: struct &gpufreq_cooling_device for this gpu cdev
 * @level:        level in the GPU DVFS OPP table
 * @power:        pointer in which to store the calculated static power
 *
 * Calculate the static power consumed by the gpus described by
 * @gpu_actor running at frequency @freq.  This function relies on a
 * platform specific function that should have been provided when the
 * actor was registered.  If it wasn't, the static power is assumed to
 * be negligible.  The calculated static power is stored in @power.
 *
 * Return: 0 on success, or an error value on failure.
 */
static int get_static_power(struct gpufreq_cooling_device *gpufreq_cdev,
			    int level, u32 *power)
{
	struct thermal_zone_device *tz = gpufreq_cdev->tzd;
	unsigned int voltage;

	if (level < 0) {
		pr_warn("%s passed invalid level %d\n", __func__, level);
		*power = 0;
		return -EINVAL;
	}

	if (!tz) {
		pr_warn("%s no thermal zone\n", __func__);
		*power = 0;
		return -EINVAL;
	}

	if (gpufreq_cdev->gpu_fns->get_vols_for_level(gpufreq_cdev->gpu_drv_data, level, NULL,
		&voltage)) {
		pr_warn("Failed to get voltage for level %d\n", level);
		return -EINVAL;
	}

	return lookup_static_power(gpufreq_cdev, voltage, tz->temperature, power);
}

/**
 * get_dynamic_power() - calculate the dynamic power
 *
 * @gpufreq_cdev: &gpufreq_cooling_device for this cdev
 * @freq:	  current frequency
 *
 * Return: the dynamic power consumed by the gpus described by
 *         @gpufreq_cdev.
 */
static u32 get_dynamic_power(struct gpufreq_cooling_device *gpufreq_cdev,
			     unsigned long freq)
{
	u32 raw_gpu_power;

	raw_gpu_power = gpu_freq_to_power(gpufreq_cdev, freq);
	return (raw_gpu_power * gpufreq_cdev->last_load) / 100;
}

/**
 * gpufreq_set_cur_state() - callback function to set the current cooling state.
 *
 * @cdev:  thermal cooling device pointer.
 * @state: set this variable to the current cooling state.
 *
 * Callback for the thermal cooling device to change the gpufreq
 * current cooling state.
 *
 * Return: 0 on success, an error code otherwise.
*/

static int gpufreq_set_cur_state(struct thermal_cooling_device *cdev,
				unsigned long state)
{
	struct gpu_tmu_notification_data nd;
	struct gpufreq_cooling_device *gpufreq_cdev = cdev->devdata;
	unsigned int max_state;
	int ret;

	state = max(gpufreq_cdev->sysfs_req, state);
	/* Check if the old cooling action is same as new cooling action */
	if (gpufreq_cdev->gpufreq_state == state)
		return -EALREADY;

	ret = get_property(gpufreq_cdev, 0, &max_state, GET_MAXL);
	if (ret)
		return ret;

	if (WARN_ON(state < 0 || state > max_state))
		return -EINVAL;

	gpufreq_cdev->gpufreq_state = state;

	nd.gpu_drv_data = gpufreq_cdev->gpu_drv_data;
	nd.data = gpufreq_cdev->gpufreq_state;

	blocking_notifier_call_chain(&gpu_notifier, GPU_THROTTLING, &nd);
	trace_vendor_cdev_update(cdev->type, gpufreq_cdev->sysfs_req, state);
	trace_clock_set_rate(cdev->type, gpufreq_cdev->sysfs_req, raw_smp_processor_id());

	return 0;
}

/* gpufreq cooling device callback functions are defined below */

/**
 * gpufreq_get_max_state() - callback function to get the max cooling state.
 *
 * @cdev: thermal cooling device pointer.
 * @state: fill this variable with the max cooling state.
 *
 * Callback for the thermal cooling device to return the gpufreq
 * max cooling state.
 *
 * Return: 0 on success, an error code otherwise.
 */
static int gpufreq_get_max_state(struct thermal_cooling_device *cdev,
				 unsigned long *state)
{
	unsigned int count = 0;
	int ret;
	struct gpufreq_cooling_device *gpufreq_cdev = cdev->devdata;

	ret = get_property(gpufreq_cdev, 0, &count, GET_MAXL);

	if (count > 0)
		*state = count;

	return ret;
}

/**
 * gpufreq_get_cur_state() - callback function to get the current cooling state.
 *
 * @cdev:  thermal cooling device pointer.
 * @state: fill this variable with the current cooling state.
 *
 * Callback for the thermal cooling device to return the gpufreq
 * current cooling state.
 *
 * Return: 0 on success, an error code otherwise.
 */
static int gpufreq_get_cur_state(struct thermal_cooling_device *cdev,
				 unsigned long *state)
{
	struct gpufreq_cooling_device *gpufreq_cdev = cdev->devdata;

	*state = gpufreq_cdev->gpufreq_state;

	return 0;
}

/**
 * gpufreq_get_requested_power() - get the current power
 *
 * @cdev:  &thermal_cooling_device pointer
 * @power: pointer in which to store the resulting power
 *
 * Calculate the current power consumption of the gpus in milliwatts
 * and store it in @power.  This function should actually calculate
 * the requested power, but it's hard to get the frequency that
 * gpufreq would have assigned if there were no thermal limits.
 * Instead, we calculate the current power on the assumption that the
 * immediate future will look like the immediate past.
 *
 * We use the current frequency and the average load since this
 * function was last called.  In reality, there could have been
 * multiple opps since this function was last called and that affects
 * the load calculation.  While it's not perfectly accurate, this
 * simplification is good enough and works.  REVISIT this, as more
 * complex code may be needed if experiments show that it's not
 * accurate enough.
 *
 * Return: 0 on success, -E* if getting the static power failed.
 */
static int gpufreq_get_requested_power(struct thermal_cooling_device *cdev,
				       u32 *power)
{
	unsigned int freq;
	u32 static_power, dynamic_power, load;
	struct gpufreq_cooling_device *gpufreq_cdev = cdev->devdata;
	int level;
	int ret = 0;

	level = gpufreq_cdev->gpu_fns->get_cur_level(gpufreq_cdev->gpu_drv_data);
	gpufreq_cdev->gpu_fns->get_freqs_for_level(gpufreq_cdev->gpu_drv_data, level, NULL, &freq);
	load = gpufreq_cdev->gpu_fns->get_cur_util(gpufreq_cdev->gpu_drv_data);
	gpufreq_cdev->last_load = load;
	dynamic_power = get_dynamic_power(gpufreq_cdev, freq);

	ret = get_static_power(gpufreq_cdev, level, &static_power);
	if (ret)
		return ret;

	if (trace_thermal_exynos_power_gpu_get_power_enabled())
		trace_thermal_exynos_power_gpu_get_power(freq, load, dynamic_power, static_power);

	*power = static_power + dynamic_power;

	return 0;
}

/**
 * gpufreq_state2power() - convert a gpu cdev state to power consumed
 *
 * @cdev:  &thermal_cooling_device pointer
 * @state: cooling device state to be converted
 * @power: pointer in which to store the resulting power
 *
 * Convert cooling device state @state into power consumption in
 * milliwatts assuming 100% load.  Store the calculated power in
 * @power.
 *
 * Return: 0 on success, -EINVAL if the cooling device state could not
 *         be converted into a frequency or other -E* if there was an error
 *         when calculating the static power.
 */
static int gpufreq_state2power(struct thermal_cooling_device *cdev,
			       unsigned long state, u32 *power)
{
	unsigned int freq;
	u32 static_power, dynamic_power;
	int ret;
	struct gpufreq_cooling_device *gpufreq_cdev = cdev->devdata;

	freq = gpufreq_cdev->gpu_freq_table[state].frequency;
	if (!freq)
		return -EINVAL;

	dynamic_power = gpu_freq_to_power(gpufreq_cdev, freq);
	ret = get_static_power(gpufreq_cdev, state, &static_power);
	if (ret)
		return ret;

	*power = static_power + dynamic_power;
	return 0;
}

/**
 * gpufreq_power2state() - convert power to a cooling device state
 *
 * @cdev:  &thermal_cooling_device pointer
 * @power: power in milliwatts to be converted
 * @state: pointer in which to store the resulting state
 *
 * Calculate a cooling device state for the gpus described by @cdev
 * that would allow them to consume at most @power mW and store it in
 * @state.  Note that this calculation depends on external factors
 * such as the gpu load or the current static power.  Calling this
 * function with the same power as input can yield different cooling
 * device states depending on those external factors.
 *
 * Return: 0 on success, -ENODEV if no gpus are online or -EINVAL if
 *         the calculated frequency could not be converted to a valid state.
 *         The latter should not happen unless the frequencies available to
 *         gpufreq have changed since the initialization of the gpu cooling
 *         device.
 */
static int gpufreq_power2state(struct thermal_cooling_device *cdev,
			       u32 power, unsigned long *state)
{
	unsigned int target_freq;
	int ret;
	s32 dyn_power;
	u32 static_power;
	struct gpufreq_cooling_device *gpufreq_cdev = cdev->devdata;
	int level;

	level = gpufreq_cdev->gpu_fns->get_cur_level(gpufreq_cdev->gpu_drv_data);
	ret = get_static_power(gpufreq_cdev, level, &static_power);
	if (ret)
		return ret;

	dyn_power = power - static_power;
	dyn_power = dyn_power > 0 ? dyn_power : 0;
	target_freq = gpu_power_to_freq(gpufreq_cdev, dyn_power);

	*state = gpufreq_cooling_get_level(gpufreq_cdev, target_freq);
	if (*state == THERMAL_CSTATE_INVALID) {
		pr_warn("Failed to convert %dKHz for gpu into a cdev state\n",
			target_freq);
		return -EINVAL;
	}

	trace_thermal_exynos_power_gpu_limit(target_freq, *state, power);
	return 0;
}

/* Bind gpufreq callbacks to thermal cooling device ops */
static struct thermal_cooling_device_ops gpufreq_cooling_ops = {
	.get_max_state = gpufreq_get_max_state,
	.get_cur_state = gpufreq_get_cur_state,
	.set_cur_state = gpufreq_set_cur_state,
};

static struct thermal_cooling_device_ops gpufreq_power_cooling_ops = {
	.get_max_state = gpufreq_get_max_state,
	.get_cur_state = gpufreq_get_cur_state,
	.set_cur_state = gpufreq_set_cur_state,
	.get_requested_power = gpufreq_get_requested_power,
	.state2power = gpufreq_state2power,
	.power2state = gpufreq_power2state,
};

/**
 * gpufreq_cooling_add_notifier() - Register a notifier for GPU cooling events
 *
 * @nb: The &notifier_block storing the callback to be notified
 *
 * Return: Returns 0 on successful registration of the notifier.
 */
int gpufreq_cooling_add_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&gpu_notifier, nb);
}
EXPORT_SYMBOL_GPL(gpufreq_cooling_add_notifier);

/**
 * gpufreq_cooling_remove_notifier() - Register a notifier for GPU cooling events
 *
 * @nb: The &notifier_block storing the callback to be notified
 *
 * Return: Returns zero on success or %-ENOENT on failure.
 */
int gpufreq_cooling_remove_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&gpu_notifier, nb);
}
EXPORT_SYMBOL_GPL(gpufreq_cooling_remove_notifier);

static struct thermal_zone_device * parse_ect_cooling_level(
	struct thermal_cooling_device *cdev, char *cooling_name)
{
	struct thermal_instance *instance;
	struct thermal_zone_device *tz = NULL;
	bool foundtz = false;
	void *thermal_block;
	struct ect_ap_thermal_function *function;
	int i, temperature;
	unsigned int freq;
	struct gpufreq_cooling_device *gpufreq_cdev = cdev->devdata;

	mutex_lock(&cdev->lock);
	list_for_each_entry(instance, &cdev->thermal_instances, cdev_node) {
		tz = instance->tz;
		if (!strncasecmp(cooling_name, tz->type, THERMAL_NAME_LENGTH)) {
			foundtz = true;
			break;
		}
	}
	mutex_unlock(&cdev->lock);

	if (!foundtz)
		goto skip_ect;

	thermal_block = ect_get_block(BLOCK_AP_THERMAL);
	if (!thermal_block)
		goto skip_ect;

	function = ect_ap_thermal_get_function(thermal_block, cooling_name);
	if (!function)
		goto skip_ect;

	for (i = 0; i < function->num_of_range; ++i) {
		unsigned long max_level = 0;
		int level;

		temperature = function->range_list[i].lower_bound_temperature;
		freq = function->range_list[i].max_frequency;

		instance = get_thermal_instance(tz, cdev, i);
		if (!instance) {
			pr_err("%s: (%s, %d)instance isn't valid\n", __func__, cooling_name, i);
			goto skip_ect;
		}

		cdev->ops->get_max_state(cdev, &max_level);
		level = gpufreq_cooling_get_level(gpufreq_cdev, freq);

		if (level == THERMAL_CSTATE_INVALID)
			level = max_level;

		instance->upper = level;

		pr_info("Parsed From ECT : %s: [%d] Temperature : %d, frequency : %u, level: %d\n",
			cooling_name, i, temperature, freq, level);
	}
skip_ect:
	return tz;
}

/**
 * gpu_cooling_table_init() - function to make GPU throttling table.
 *
 * @gpu_drv_data: Pointer to GPU driver data
 *
 * Return : a valid struct gpu_freq_table pointer on success,
 *          on failture, it returns a corresponding ERR_PTR().
 */
static int gpu_cooling_table_init(struct gpufreq_cooling_device *gpufreq_cdev)
{
	unsigned int freq;
	int i = 0, num_level = 0, count = 0;
	void *gpu_drv_data = gpufreq_cdev->gpu_drv_data;
	struct cpufreq_frequency_table *table;

	num_level = gpufreq_cdev->gpu_fns->get_num_levels(gpu_drv_data);

	if (num_level == 0) {
		pr_err("Failed to determine number of GPU OPPs\n");
		return -EINVAL;
	}

	/* Table size can be num_of_range + 1 since last row has the value of TABLE_END */
	table = kzalloc(sizeof(*table) * (num_level + 1), GFP_KERNEL);
	if (!table)
		return -ENOMEM;

	for (i = 0; i < num_level; i++) {
		gpufreq_cdev->gpu_fns->get_freqs_for_level(gpu_drv_data, i, NULL, &freq);

		table[count].flags = 0;
		table[count].driver_data = count;
		table[count].frequency = (unsigned int)freq;

		pr_info("[GPU cooling] index : %d, frequency : %d\n",
			table[count].driver_data, table[count].frequency);

		count++;
	}

	if (i == num_level)
		table[count].frequency = GPU_TABLE_END;

	gpufreq_cdev->gpu_freq_table = table;

	return 0;
}

ssize_t
state2power_table_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct thermal_cooling_device *cdev = to_cooling_device(dev);
	struct gpufreq_cooling_device *gpufreq_cdev = cdev->devdata;
	int ret, i, count = 0;
	unsigned int max_state = 0;
	u32 power;

	if (!gpufreq_cdev)
		return -ENODEV;

	ret = get_property(gpufreq_cdev, 0, &max_state, GET_MAXL);
	if (ret)
		return ret;

	for (i = 0; i <= max_state; i++) {
		gpufreq_state2power(cdev, i, &power);
		count += sysfs_emit_at(buf, count, "%u ", power);
	}
	count += sysfs_emit_at(buf, count, "\n");

	return count;
}

ssize_t
user_vote_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct thermal_cooling_device *cdev = to_cooling_device(dev);
	struct gpufreq_cooling_device *gpufreq_cdev = cdev->devdata;

	if (!gpufreq_cdev)
		return -ENODEV;

	return sprintf(buf, "%lu\n", gpufreq_cdev->sysfs_req);
}

ssize_t user_vote_store(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct thermal_cooling_device *cdev = to_cooling_device(dev);
	struct gpufreq_cooling_device *gpufreq_cdev = cdev->devdata;
	int ret;
	unsigned int max_state = 0;
	unsigned long state;

	if (!gpufreq_cdev)
		return -ENODEV;

	ret = kstrtoul(buf, 0, &state);
	if (ret)
		return ret;

	ret = get_property(gpufreq_cdev, 0, &max_state, GET_MAXL);
	if (ret)
		return ret;

	if (state > max_state)
		return -EINVAL;

	mutex_lock(&cdev->lock);
	gpufreq_cdev->sysfs_req = state;
	cdev->updated = false;
	mutex_unlock(&cdev->lock);
	thermal_cdev_update(cdev);
	return count;
}

static DEVICE_ATTR_RW(user_vote);
static DEVICE_ATTR_RO(state2power_table);

/**
 * __gpufreq_cooling_register - helper function to create gpufreq cooling device
 *
 * @np:           a valid struct device_node to the GPU device tree node
 * @capacitance:  dynamic power coefficient for this GPU
 * @gpu_drv_data: Data passed unmodified to GPU query functions
 * @gpu_fns:      Function pointer block of GPU query functions for this GPU
 *
 * This interface function registers the gpufreq cooling device with the name
 * "thermal-gpufreq-%x". This api can support multiple instances of gpufreq
 * cooling devices. It also gives the opportunity to link the cooling device
 * with a device tree node, in order to bind it via the thermal DT code.
 *
 * Return: a valid struct thermal_cooling_device pointer on success,
 *         on failure, it returns a corresponding ERR_PTR().
 */
static struct thermal_cooling_device *__gpufreq_cooling_register(struct device_node *np,
	u32 capacitance, void *gpu_drv_data, struct gpufreq_cooling_query_fns *gpu_fns)
{
	struct thermal_cooling_device *cool_dev;
	struct gpufreq_cooling_device *gpufreq_cdev = NULL;
	char dev_name[THERMAL_NAME_LENGTH];
	struct thermal_cooling_device_ops *cooling_ops = &gpufreq_cooling_ops;
	int ret = 0;
	int num_level, i;
	u32 power;

	gpufreq_cdev = kzalloc(sizeof(*gpufreq_cdev),
			       GFP_KERNEL);
	if (!gpufreq_cdev)
		return ERR_PTR(-ENOMEM);

	ret = get_idr(&gpufreq_idr, &gpufreq_cdev->id);
	if (ret) {
		kfree(gpufreq_cdev);
		return ERR_PTR(ret);
	}

	if (gpu_drv_data == NULL || gpu_fns == NULL)
		return ERR_PTR(-EINVAL);

	gpufreq_cdev->gpu_drv_data = gpu_drv_data;
	gpufreq_cdev->gpu_fns = gpu_fns;

	ret = gpu_cooling_table_init(gpufreq_cdev);
	if (ret) {
		pr_err("Failed to initialize gpu_cooling_table\n");
		return ERR_PTR(ret);
	}

	if (capacitance && gpu_drv_data &&
		!build_dyn_power_table(gpufreq_cdev, capacitance) &&
		!build_static_power_table(np, gpufreq_cdev)) {
		cooling_ops = &gpufreq_power_cooling_ops;
	}

	snprintf(dev_name, sizeof(dev_name), "thermal-gpufreq-%d",
		 gpufreq_cdev->id);

	cool_dev = thermal_of_cooling_device_register(np, dev_name,
						      gpufreq_cdev, cooling_ops);
	if (IS_ERR(cool_dev)) {
		pr_warn("%s: register cooling device %s failed\n", __func__,
			dev_name);
		goto free_cool_dev;
	}

	ret = device_create_file(&cool_dev->device, &dev_attr_user_vote);
	if (ret) {
		thermal_cooling_device_unregister(cool_dev);
		goto free_cool_dev;
	}

	ret = device_create_file(&cool_dev->device, &dev_attr_state2power_table);
	if (ret) {
		thermal_cooling_device_unregister(cool_dev);
		goto free_cool_dev;
	}

	gpufreq_cdev->tzd = parse_ect_cooling_level(cool_dev, "G3D");

	gpufreq_cdev->cool_dev = cool_dev;
	gpufreq_cdev->gpufreq_state = 0;

	pr_info("gpu cooling registered for %s, capacitance: %d, power_callback: %s\n",
		dev_name, capacitance,
		cooling_ops == &gpufreq_power_cooling_ops ? "true" : "false");

	num_level = gpufreq_cdev->gpu_fns->get_num_levels(gpufreq_cdev->gpu_drv_data);
	for (i = 0; i < num_level; i++) {
		gpufreq_state2power(cool_dev, i, &power);
		pr_debug("[GPU cooling] index:%d: state:%d power:%u\n",
			gpufreq_cdev->id, i, power);
	}

	return cool_dev;

free_cool_dev:
	release_idr(&gpufreq_idr, gpufreq_cdev->id);
	kfree(gpufreq_cdev);
	return cool_dev;
}

/**
 * gpufreq_cooling_register() - create gpufreq cooling device with power extensions
 *
 * @np:           a valid struct device_node to the GPU device tree node
 * @gpu_drv_data: Data passed unmodified to GPU query functions
 * @gpu_fns:      Function pointer block of GPU query functions for this GPU
 *
 * This interface function registers a gpufreq cooling device with the name
 * "thermal-gpufreq-%x". This api can support multiple instances of gpufreq
 * cooling devices.
 *
 * Using this API, the gpufreq cooling device will be linked to the device tree
 * node provided via &np and may implent power extensions via a simple gpu power
 * model.
 *
 * Return: a valid struct thermal_cooling_device pointer on success,
 *         or on failure a corresponding ERR_PTR().
 */
struct thermal_cooling_device *gpufreq_cooling_register(struct device_node *np, void *gpu_drv_data,
	struct gpufreq_cooling_query_fns *gpu_fns)
{
	void *gen_block;
	u32 capacitance = 0, index;
	struct ect_gen_param_table *pwr_coeff;

	if (!np)
		return ERR_PTR(-EINVAL);

	of_property_read_u32(np, "gpu_power_coeff", &capacitance);

	if (of_property_read_bool(np, "use-em-coeff"))
		goto regist;

	if (!of_property_read_u32(np, "ect-coeff-index", &index)) {
		gen_block = ect_get_block("GEN");
		if (!gen_block) {
			pr_err("%s: Failed to get gen block from ECT\n", __func__);
			goto regist;
		}
		pwr_coeff = ect_gen_param_get_table(gen_block, "DTM_PWR_Coeff");
		if (!pwr_coeff) {
			pr_err("%s: Failed to get power coeff from ECT\n", __func__);
			goto regist;
		}
		capacitance = pwr_coeff->parameter[index];
	} else {
		pr_err("%s: could not find ect-coeff-index\n", __func__);
	}

regist:
	return __gpufreq_cooling_register(np, capacitance, gpu_drv_data, gpu_fns);
}
EXPORT_SYMBOL(gpufreq_cooling_register);

/**
 * gpufreq_cooling_unregister - function to remove gpufreq cooling device.
 *
 * @cdev: thermal cooling device pointer.
 *
 * This interface function unregisters the "thermal-gpufreq-%x" cooling device.
 */
void gpufreq_cooling_unregister(struct thermal_cooling_device *cdev)
{
	struct gpufreq_cooling_device *gpufreq_cdev;

	if (!cdev)
		return;

	gpufreq_cdev = cdev->devdata;

	thermal_cooling_device_unregister(gpufreq_cdev->cool_dev);

	release_idr(&gpufreq_idr, gpufreq_cdev->id);

	kfree(gpufreq_cdev->var_table);
	kfree(gpufreq_cdev->var_coeff);
	kfree(gpufreq_cdev->asv_coeff);
	kfree(gpufreq_cdev->dyn_power_table);
	kfree(gpufreq_cdev->gpu_freq_table);

	kfree(gpufreq_cdev);
}
EXPORT_SYMBOL_GPL(gpufreq_cooling_unregister);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Pixel GPU cooling");
MODULE_AUTHOR("<sidaths@google.com>");
MODULE_VERSION("1.0");
