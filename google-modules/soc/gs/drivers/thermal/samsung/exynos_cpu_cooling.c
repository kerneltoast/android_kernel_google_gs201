// SPDX-License-Identifier: GPL-2.0
/*
 *  linux/drivers/thermal/cpu_cooling.c
 *
 *  Copyright (C) 2012	Samsung Electronics Co., Ltd(http://www.samsung.com)
 *
 *  Copyright (C) 2012-2018 Linaro Limited.
 *
 *  Authors:	Amit Daniel <amit.kachhap@linaro.org>
 *		Viresh Kumar <viresh.kumar@linaro.org>
 *
 */
#include <linux/module.h>
#include <linux/thermal.h>
#include <linux/cpufreq.h>
#include <linux/err.h>
#include <linux/idr.h>
#include <linux/pm_opp.h>
#include <linux/pm_qos.h>
#include <linux/slab.h>
#include <linux/cpu.h>
#include <linux/units.h>

#include <soc/google/tmu.h>
#include <soc/google/cal-if.h>
#include <soc/google/ect_parser.h>
#include <soc/google/exynos_cpu_cooling.h>
#include <soc/google/exynos_pm_qos.h>
#include <thermal_core.h>

#include <trace/events/power.h>
#include <trace/events/thermal_exynos.h>

#if IS_ENABLED(CONFIG_PIXEL_EM)
#include "../../soc/google/vh/include/pixel_em.h"
struct pixel_em_profile **exynos_cpu_cooling_pixel_em_profile;
EXPORT_SYMBOL_GPL(exynos_cpu_cooling_pixel_em_profile);
#endif

/*
 * Cooling state <-> CPUFreq frequency
 *
 * Cooling states are translated to frequencies throughout this driver and this
 * is the relation between them.
 *
 * Highest cooling state corresponds to lowest possible frequency.
 *
 * i.e.
 *	level 0 --> 1st Max Freq
 *	level 1 --> 2nd Max Freq
 *	...
 */

/**
 * struct freq_table - frequency table along with power entries
 * @frequency:	frequency in KHz
 * @power:	power in mW
 *
 * This structure is built when the cooling device registers and helps
 * in translating frequency to power and vice versa.
 */
struct freq_table {
	u32 frequency;
	u32 power;
};

/**
 * struct time_in_idle - Idle time stats
 * @time: previous reading of the absolute time that this cpu was idle
 * @timestamp: wall time of the last invocation of get_cpu_idle_time_us()
 */
struct time_in_idle {
	u64 time;
	u64 timestamp;
};

/**
 * struct exynos_cpu_dsu_bci_freq - struct for constraint frequency mapping
 */
struct exynos_cpu_dsu_bci_freq {
	u32 cpu_freq;
	u32 dsu_freq;
	u32 bci_freq;
};

/**
 * struct exynos_cpu_cooling_device - data for cooling device with cpufreq
 * @id: unique integer value corresponding to each exynos_cpu_cooling_device
 *	registered.
 * @last_load: load measured by the latest call to cpufreq_get_requested_power()
 * @cpufreq_state: integer value representing the current state of cpufreq
 *	cooling	devices.
 * @max_level: maximum cooling level. One less than total number of valid
 *	cpufreq frequencies.
 * @freq_table: Freq table in descending order of frequencies
 * @cdev: thermal_cooling_device pointer to keep track of the
 *	registered cooling device.
 * @policy: cpufreq policy.
 * @node: list_head to link all exynos_cpu_cooling_device together.
 * @idle_time: idle time stats
 * @tzd: &thermal zone device backing this cpu cooling device.
 *
 * This structure is required for keeping information of each registered
 * exynos_cpu_cooling_device.
 */
struct exynos_cpu_cooling_device {
	int id;
	u32 last_load;
	unsigned int cpufreq_state;
	unsigned int max_level;
	struct freq_table *freq_table;	/* In descending order */
	struct cpufreq_policy *policy;
	struct list_head node;
	struct time_in_idle *idle_time;
	struct freq_qos_request qos_req;
	bool has_static;

	int *var_table;
	int *var_coeff;
	int *asv_coeff;
	unsigned int var_volt_size;
	unsigned int var_temp_size;

	struct thermal_zone_device *tzd;
	unsigned long sysfs_req;
	bool sysfs_req_bypass;
	bool apply_dsu_bci_constraints;
	struct exynos_pm_qos_request dsu_qos_max;
	struct exynos_pm_qos_request bci_qos_max;
	struct exynos_cpu_dsu_bci_freq *cpu_dsu_bci_map;
	int cpu_dsu_bci_map_size;
};

static DEFINE_IDA(cpufreq_ida);
static DEFINE_MUTEX(cooling_list_lock);
static LIST_HEAD(cpufreq_cdev_list);

static int init_dsu_bci_constraint_table_dt(struct exynos_cpu_cooling_device *cpufreq_cdev,
                                    struct device_node *dn)
{
	struct exynos_cpu_dsu_bci_freq *table;
	int size, num_rows, ret;

	/*
	 * Each DSU BCI constraint table row consists of CPU frequency, DSU frequency
	 * and BCI frequency values. The size of each row is 96 bytes (3 x 32 bytes)
	 * To get number of rows, divide the total size by 3
	 */
	size = of_property_count_u32_elems(dn, "dsu-bci-constraint-table");
	if ((size <= 0) || (size % 3 != 0))
		return -EINVAL;

	num_rows = size / 3;
	table = kcalloc(num_rows, sizeof(*table), GFP_KERNEL);
	if (!table)
		return -ENOMEM;

	ret = of_property_read_u32_array(dn, "dsu-bci-constraint-table", (u32 *)table, size);
	if (ret) {
		kfree(table);
		return -EINVAL;
	}

	cpufreq_cdev->cpu_dsu_bci_map = table;
	cpufreq_cdev->cpu_dsu_bci_map_size =  num_rows;
	return 0;
}


/* Below code defines functions to be used for cpufreq as cooling device */

/**
 * get_level: Find the level for a particular frequency
 * @cpufreq_cdev: cpufreq_cdev for which the property is required
 * @freq: Frequency
 *
 * Return: level corresponding to the frequency.
 */
static unsigned long get_level(struct exynos_cpu_cooling_device *cpufreq_cdev,
			       unsigned int freq)
{
	struct freq_table *freq_table = cpufreq_cdev->freq_table;
	unsigned long level;

	for (level = 1; level <= cpufreq_cdev->max_level; level++)
		if (freq > freq_table[level].frequency)
			break;

	return level - 1;
}

/**
 * update_freq_table() - Update the freq table with power numbers
 * @cpufreq_cdev:	the cpufreq cooling device in which to update the table
 * @capacitance: dynamic power coefficient for these cpus
 *
 * Update the freq table with power numbers.  This table will be used in
 * cpu_power_to_freq() and cpu_freq_to_power() to convert between power and
 * frequency efficiently.  Power is stored in mW, frequency in KHz.  The
 * resulting table is in descending order.
 *
 * Return: 0 on success, -EINVAL if there are no OPPs for any CPUs,
 * or -ENOMEM if we run out of memory.
 */
static int update_freq_table(struct exynos_cpu_cooling_device *cpufreq_cdev,
			     u32 capacitance)
{
	struct freq_table *freq_table = cpufreq_cdev->freq_table;
	struct dev_pm_opp *opp;
	struct device *dev = NULL;
	int num_opps = 0, cpu = cpufreq_cdev->policy->cpu, i;

	dev = get_cpu_device(cpu);
	if (unlikely(!dev)) {
		pr_warn("No cpu device for cpu %d\n", cpu);
		return -ENODEV;
	}

	num_opps = dev_pm_opp_get_opp_count(dev);
	if (num_opps < 0)
		return num_opps;

	/*
	 * The cpufreq table is also built from the OPP table and so the count
	 * should match.
	 */
	if (num_opps != cpufreq_cdev->max_level + 1) {
		dev_warn(dev, "Number of OPPs not matching with max_levels\n");
		return -EINVAL;
	}

	for (i = 0; i <= cpufreq_cdev->max_level; i++) {
		unsigned long freq = freq_table[i].frequency * 1000;
		u32 freq_mhz = freq_table[i].frequency / 1000;
		u64 power;
		u32 voltage_mv;

		/*
		 * Find ceil frequency as 'freq' may be slightly lower than OPP
		 * freq due to truncation while converting to kHz.
		 */
		opp = dev_pm_opp_find_freq_ceil(dev, &freq);
		if (IS_ERR(opp)) {
			dev_err(dev, "failed to get opp for %lu frequency\n",
				freq);
			return -EINVAL;
		}

		voltage_mv = dev_pm_opp_get_voltage(opp) / 1000;
		dev_pm_opp_put(opp);

		/*
		 * Do the multiplication with MHz and millivolt so as
		 * to not overflow.
		 */
		power = (u64)capacitance * freq_mhz * voltage_mv * voltage_mv;
		do_div(power, 1000000000);

		/* power is stored in mW */
		freq_table[i].power = power;
		pr_info("cpu_cooling %d: freq:%u power: %u, mV: %u, cap: %u\n", cpu,
			freq_table[i].frequency, freq_table[i].power, voltage_mv, capacitance);
	}

	return 0;
}

static int build_static_power_table(struct device_node *np,
				    struct exynos_cpu_cooling_device *cpufreq_cdev,
				    char *cooling_name)
{
	int i, j;
	int ratio, asv_group, cal_id, ret = 0;

	void *gen_block;
	struct ect_gen_param_table *volt_temp_param = NULL, *asv_param = NULL;
	char volt_param_name[32], asv_param_name[32];
	int ratio_table[16] = { 0, 18, 22, 27, 33, 40, 49, 60, 73, 89,
				108, 131, 159, 194, 232, 250};

	ret = of_property_read_u32(np, "cal-id", &cal_id);
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

	snprintf(volt_param_name, sizeof(volt_param_name),
		 "DTM_%s_VOLT_TEMP", cooling_name);
	snprintf(asv_param_name, sizeof(asv_param_name),
		 "DTM_%s_ASV", cooling_name);

	volt_temp_param = ect_gen_param_get_table(gen_block, volt_param_name);
	asv_param = ect_gen_param_get_table(gen_block, asv_param_name);

	if (volt_temp_param && asv_param) {
		cpufreq_cdev->var_volt_size = volt_temp_param->num_of_row - 1;
		cpufreq_cdev->var_temp_size = volt_temp_param->num_of_col - 1;

		cpufreq_cdev->var_coeff = kzalloc(sizeof(int) *
							volt_temp_param->num_of_row *
							volt_temp_param->num_of_col,
							GFP_KERNEL);
		if (!cpufreq_cdev->var_coeff)
			goto err_mem;

		cpufreq_cdev->asv_coeff = kzalloc(sizeof(int) *
							asv_param->num_of_row *
							asv_param->num_of_col,
							GFP_KERNEL);
		if (!cpufreq_cdev->asv_coeff)
			goto free_var_coeff;

		cpufreq_cdev->var_table = kzalloc(sizeof(int) *
							volt_temp_param->num_of_row *
							volt_temp_param->num_of_col,
							GFP_KERNEL);
		if (!cpufreq_cdev->var_table)
			goto free_asv_coeff;

		memcpy(cpufreq_cdev->var_coeff, volt_temp_param->parameter,
		       sizeof(int) * volt_temp_param->num_of_row * volt_temp_param->num_of_col);
		memcpy(cpufreq_cdev->asv_coeff, asv_param->parameter,
		       sizeof(int) * asv_param->num_of_row * asv_param->num_of_col);
		memcpy(cpufreq_cdev->var_table, volt_temp_param->parameter,
		       sizeof(int) * volt_temp_param->num_of_row * volt_temp_param->num_of_col);
	} else {
		pr_err("%s: Failed to get param table from ECT\n", __func__);
		return -EINVAL;
	}

	for (i = 1; i <= cpufreq_cdev->var_volt_size; i++) {
		long asv_coeff = (long)cpufreq_cdev->asv_coeff[3 * i + 0] * asv_group * asv_group
				+ (long)cpufreq_cdev->asv_coeff[3 * i + 1] * asv_group
				+ (long)cpufreq_cdev->asv_coeff[3 * i + 2];
		asv_coeff = asv_coeff / 100;

		for (j = 1; j <= cpufreq_cdev->var_temp_size; j++) {
			long var_coeff = (long)cpufreq_cdev->var_coeff[i
							* (cpufreq_cdev->var_temp_size + 1) + j];

			var_coeff =  ratio * var_coeff * asv_coeff;
			var_coeff = var_coeff / 100000;
			cpufreq_cdev->var_table[i * (cpufreq_cdev->var_temp_size + 1) + j] =
										(int)var_coeff;
		}
	}

	return 0;

free_asv_coeff:
	kfree(cpufreq_cdev->asv_coeff);
free_var_coeff:
	kfree(cpufreq_cdev->var_coeff);
err_mem:
	return -ENOMEM;
}

static int lookup_static_power(struct exynos_cpu_cooling_device *cpufreq_cdev,
			       unsigned long voltage, int temperature, u32 *power)
{
	int volt_index = 0, temp_index = 0;
	int index = 0;
	int num_cpus;
	int max_cpus;
	struct cpufreq_policy *policy = cpufreq_cdev->policy;
	cpumask_t tempmask;

	if (!cpufreq_cdev->has_static)
		return -EINVAL;

	cpumask_and(&tempmask, policy->related_cpus, cpu_online_mask);
	max_cpus = cpumask_weight(policy->related_cpus);
	num_cpus = cpumask_weight(&tempmask);
	voltage = voltage / 1000;
	temperature  = temperature / 1000;

	for (volt_index = 0; volt_index <= cpufreq_cdev->var_volt_size; volt_index++) {
		if (voltage < cpufreq_cdev->var_table[volt_index
						* ((int)cpufreq_cdev->var_temp_size + 1)]) {
			volt_index = volt_index - 1;
			break;
		}
	}

	if (volt_index == 0)
		volt_index = 1;

	if (volt_index > cpufreq_cdev->var_volt_size)
		volt_index = cpufreq_cdev->var_volt_size;

	for (temp_index = 0; temp_index <= cpufreq_cdev->var_temp_size; temp_index++) {
		if (temperature < cpufreq_cdev->var_table[temp_index]) {
			temp_index = temp_index - 1;
			break;
		}
	}

	if (temp_index == 0)
		temp_index = 1;

	if (temp_index > cpufreq_cdev->var_temp_size)
		temp_index = cpufreq_cdev->var_temp_size;

	index = (int)(volt_index * (cpufreq_cdev->var_temp_size + 1) + temp_index);
	*power = (unsigned int)cpufreq_cdev->var_table[index];

	return 0;
}

static u32 cpu_freq_to_power(struct exynos_cpu_cooling_device *cpufreq_cdev,
			     u32 freq)
{
	int i;
	struct freq_table *freq_table;

#if IS_ENABLED(CONFIG_PIXEL_EM)
	{
		struct pixel_em_profile **profile_ptr_snapshot;
		profile_ptr_snapshot = READ_ONCE(exynos_cpu_cooling_pixel_em_profile);
		if (profile_ptr_snapshot) {
			struct pixel_em_profile *profile = READ_ONCE(*profile_ptr_snapshot);
			if (profile) {
				int cpu = cpumask_first(cpufreq_cdev->policy->related_cpus);
				struct pixel_em_cluster *cluster = profile->cpu_to_cluster[cpu];
				int opp_id;
				for (opp_id = 0; opp_id < (cluster->num_opps - 1); opp_id++) {
					if (freq <= cluster->opps[opp_id].freq)
						break;
				}
				return cluster->opps[opp_id].power / MICROWATT_PER_MILLIWATT;
			}
		}
	}
#endif

	freq_table = cpufreq_cdev->freq_table;

	for (i = 1; i <= cpufreq_cdev->max_level; i++)
		if (freq > freq_table[i].frequency)
			break;

	return freq_table[i - 1].power;
}

static u32 cpu_power_to_freq(struct exynos_cpu_cooling_device *cpufreq_cdev,
			     u32 power)
{
	int i;
	struct freq_table *freq_table;

#if IS_ENABLED(CONFIG_PIXEL_EM)
	{
		struct pixel_em_profile **profile_ptr_snapshot;
		profile_ptr_snapshot = READ_ONCE(exynos_cpu_cooling_pixel_em_profile);
		if (profile_ptr_snapshot) {
			struct pixel_em_profile *profile = READ_ONCE(*profile_ptr_snapshot);
			if (profile) {
				int cpu = cpumask_first(cpufreq_cdev->policy->related_cpus);
				struct pixel_em_cluster *cluster = profile->cpu_to_cluster[cpu];
				int opp_id;
				for (opp_id = 0; opp_id < (cluster->num_opps - 1); opp_id++) {
					if (power <= cluster->opps[opp_id].power / MICROWATT_PER_MILLIWATT)
						break;
				}
				return cluster->opps[opp_id].freq;
			}
		}
	}
#endif

	freq_table = cpufreq_cdev->freq_table;

	for (i = 1; i <= cpufreq_cdev->max_level; i++)
		if (power > freq_table[i].power)
			break;

	return freq_table[i - 1].frequency;
}

/**
 * get_load() - get load for a cpu since last updated
 * @cpufreq_cdev:	&struct exynos_cpu_cooling_device for this cpu
 * @cpu:	cpu number
 * @cpu_idx:	index of the cpu in time_in_idle*
 *
 * Return: The average load of cpu @cpu in percentage since this
 * function was last called.
 */
static u32 get_load(struct exynos_cpu_cooling_device *cpufreq_cdev, int cpu,
		    int cpu_idx)
{
	u32 load;
	u64 now, now_idle, delta_time, delta_idle;
	struct time_in_idle *idle_time = &cpufreq_cdev->idle_time[cpu_idx];

	now_idle = get_cpu_idle_time(cpu, &now, 0);
	delta_idle = now_idle - idle_time->time;
	delta_time = now - idle_time->timestamp;

	if (delta_time <= delta_idle)
		load = 0;
	else
		load = div64_u64(100 * (delta_time - delta_idle), delta_time);

	idle_time->time = now_idle;
	idle_time->timestamp = now;

	return load;
}

/**
 * get_static_power() - calculate the static power consumed by the cpus
 * @cpufreq_cdev:	struct &cpufreq_cooling_device for this cpu cdev
 * @freq:	frequency in KHz
 * @power:	pointer in which to store the calculated static power
 *
 * Calculate the static power consumed by the cpus described by
 * @cpu_actor running at frequency @freq.  This function relies on a
 * platform specific function that should have been provided when the
 * actor was registered.  If it wasn't, the static power is assumed to
 * be negligible.  The calculated static power is stored in @power.
 *
 * Return: 0 on success, -E* on failure.
 */
static int get_static_power(struct exynos_cpu_cooling_device *cpufreq_cdev,
			    unsigned long freq, u32 *power)
{
	struct thermal_zone_device *tz = cpufreq_cdev->tzd;
	struct dev_pm_opp *opp;
	unsigned long voltage;
	struct cpufreq_policy *policy = cpufreq_cdev->policy;
	unsigned long freq_hz = freq * 1000;
	struct device *dev;
	cpumask_t tempmask;
	int num_cpus, max_cpus;
	u32 raw_cpu_power;

	*power = 0;

	dev = get_cpu_device(policy->cpu);

	if (!dev || !cpu_online(policy->cpu) || !cpufreq_cdev->has_static)
		return 0;

	if (!tz)
		return -EAGAIN;

	opp = dev_pm_opp_find_freq_exact(dev, freq_hz, true);
	if (IS_ERR(opp)) {
		dev_warn_ratelimited(dev, "Failed to find OPP for frequency %lu: %ld\n",
				     freq_hz, PTR_ERR(opp));
		return -EINVAL;
	}

	voltage = dev_pm_opp_get_voltage(opp);
	dev_pm_opp_put(opp);

	if (voltage == 0) {
		dev_err_ratelimited(dev, "Failed to get voltage for frequency %lu\n",
				    freq_hz);
		return -EINVAL;
	}

	lookup_static_power(cpufreq_cdev, voltage, tz->temperature, &raw_cpu_power);

	cpumask_and(&tempmask, policy->related_cpus, cpu_online_mask);
	num_cpus = cpumask_weight(&tempmask);
	max_cpus = cpumask_weight(policy->related_cpus);

	*power = (raw_cpu_power * (num_cpus + 1)) / (max_cpus + 1);

	return 0;
}

/**
 * get_dynamic_power() - calculate the dynamic power
 * @cpufreq_cdev:	&exynos_cpu_cooling_device for this cdev
 * @freq:	current frequency
 *
 * Return: the dynamic power consumed by the cpus described by
 * @cpufreq_cdev.
 */
static u32 get_dynamic_power(struct exynos_cpu_cooling_device *cpufreq_cdev,
			     unsigned long freq)
{
	u32 raw_cpu_power;

	raw_cpu_power = cpu_freq_to_power(cpufreq_cdev, freq);
	return (raw_cpu_power * cpufreq_cdev->last_load) / 100;
}

/* cpufreq cooling device callback functions are defined below */

/**
 * cpufreq_get_max_state - callback function to get the max cooling state.
 * @cdev: thermal cooling device pointer.
 * @state: fill this variable with the max cooling state.
 *
 * Callback for the thermal cooling device to return the cpufreq
 * max cooling state.
 *
 * Return: 0 on success, an error code otherwise.
 */
static int cpufreq_get_max_state(struct thermal_cooling_device *cdev,
				 unsigned long *state)
{
	struct exynos_cpu_cooling_device *cpufreq_cdev = cdev->devdata;

	*state = cpufreq_cdev->max_level;
	return 0;
}

/**
 * cpufreq_get_cur_state - callback function to get the current cooling state.
 * @cdev: thermal cooling device pointer.
 * @state: fill this variable with the current cooling state.
 *
 * Callback for the thermal cooling device to return the cpufreq
 * current cooling state.
 *
 * Return: 0 on success, an error code otherwise.
 */
static int cpufreq_get_cur_state(struct thermal_cooling_device *cdev,
				 unsigned long *state)
{
	struct exynos_cpu_cooling_device *cpufreq_cdev = cdev->devdata;

	*state = cpufreq_cdev->cpufreq_state;

	return 0;
}

/**
 * cpufreq_set_cur_state - callback function to set the current cooling state.
 * @cdev: thermal cooling device pointer.
 * @state: set this variable to the current cooling state.
 *
 * Callback for the thermal cooling device to change the cpufreq
 * current cooling state.
 *
 * Return: 0 on success, an error code otherwise.
 */
static int cpufreq_set_cur_state(struct thermal_cooling_device *cdev,
				 unsigned long state)
{
	struct exynos_cpu_cooling_device *cpufreq_cdev = cdev->devdata;
	int ret = 0;

	/* Request state should be less than max_level */
	if (WARN_ON(state > cpufreq_cdev->max_level))
		return -EINVAL;

	if (!cpufreq_cdev->sysfs_req_bypass)
		state = max(cpufreq_cdev->sysfs_req, state);

	/* Check if the old cooling action is same as new cooling action */
	if (cpufreq_cdev->cpufreq_state == state)
		return -EALREADY;

	cpufreq_cdev->cpufreq_state = state;

	ret = freq_qos_update_request(&cpufreq_cdev->qos_req,
				      cpufreq_cdev->freq_table[state].frequency);

	if (ret == 1) {
		ret = 0;
		trace_clock_set_rate(cdev->type, state, raw_smp_processor_id());
		trace_vendor_cdev_update(cdev->type, state, state);
	}

	if (cpufreq_cdev->apply_dsu_bci_constraints) {
		int i;
		int num_opps = cpufreq_cdev->cpu_dsu_bci_map_size;
		int cur_cpu_freq = cpufreq_cdev->freq_table[state].frequency;
		int dsu_freq = cpufreq_cdev->cpu_dsu_bci_map[0].dsu_freq;
		int bci_freq = cpufreq_cdev->cpu_dsu_bci_map[0].bci_freq;

		for (i = 0; i < num_opps; i++) {
			if (cur_cpu_freq >= cpufreq_cdev->cpu_dsu_bci_map[i].cpu_freq) {
				dsu_freq = cpufreq_cdev->cpu_dsu_bci_map[i].dsu_freq;
				bci_freq = cpufreq_cdev->cpu_dsu_bci_map[i].bci_freq;
				break;
			}
		}

		exynos_pm_qos_update_request(&cpufreq_cdev->dsu_qos_max, dsu_freq);
		exynos_pm_qos_update_request(&cpufreq_cdev->bci_qos_max, bci_freq);
		trace_thermal_exynos_dus_bci_freq_update_request(dsu_freq, bci_freq);
	}

	return ret;
}

/**
 * cpufreq_get_requested_power() - get the current power
 * @cdev:	&thermal_cooling_device pointer
 * @power:	pointer in which to store the resulting power
 *
 * Calculate the current power consumption of the cpus in milliwatts
 * and store it in @power.  This function should actually calculate
 * the requested power, but it's hard to get the frequency that
 * cpufreq would have assigned if there were no thermal limits.
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
static int cpufreq_get_requested_power(struct thermal_cooling_device *cdev,
				       u32 *power)
{
	unsigned long freq;
	int i = 0, cpu, ret;
	u32 static_power, dynamic_power, total_load = 0;
	struct exynos_cpu_cooling_device *cpufreq_cdev = cdev->devdata;
	struct cpufreq_policy *policy = cpufreq_cdev->policy;
	u32 *load_cpu = NULL;
	u32 ncpus;
	struct thermal_zone_device *tz = cpufreq_cdev->tzd;

	freq = cpufreq_quick_get(policy->cpu);

	if (freq == 0) {
		*power = 0;
		return 0;
	}

	/* Hack: Driver hasn't finished loading */
	if (!tz)
		return -EAGAIN;

	if (trace_thermal_exynos_power_cpu_get_power_enabled()) {
		ncpus = cpumask_weight(policy->related_cpus);
		load_cpu = kcalloc(ncpus, sizeof(*load_cpu), GFP_KERNEL);
	}

	for_each_cpu(cpu, policy->related_cpus) {
		u32 load;

		if (cpu_online(cpu))
			load = get_load(cpufreq_cdev, cpu, i);
		else
			load = 0;

		total_load += load;
		if (load_cpu)
			load_cpu[i] = load;

		i++;
	}

	cpufreq_cdev->last_load = total_load;

	dynamic_power = get_dynamic_power(cpufreq_cdev, freq);
	ret = get_static_power(cpufreq_cdev, freq, &static_power);
	if (ret) {
		kfree(load_cpu);
		return ret;
	}

	if (load_cpu) {
		trace_thermal_exynos_power_cpu_get_power(tz->id, policy->cpu, freq,
							 load_cpu, i, dynamic_power,
							 static_power);

		kfree(load_cpu);
	}

	*power = static_power + dynamic_power;
	return 0;
}

/**
 * cpufreq_state2power() - convert a cpu cdev state to power consumed
 * @cdev:	&thermal_cooling_device pointer
 * @state:	cooling device state to be converted
 * @power:	pointer in which to store the resulting power
 *
 * Convert cooling device state @state into power consumption in
 * milliwatts assuming 100% load.  Store the calculated power in
 * @power.
 *
 * Return: 0 on success, -EINVAL if the cooling device state could not
 * be converted into a frequency or other -E* if there was an error
 * when calculating the static power.
 */
static int cpufreq_state2power(struct thermal_cooling_device *cdev,
			       unsigned long state, u32 *power)
{
	unsigned int freq, num_cpus;
	struct exynos_cpu_cooling_device *cpufreq_cdev = cdev->devdata;
	u32 static_power, dynamic_power;
	int ret;

	/* Request state should be less than max_level */
	if (WARN_ON(state > cpufreq_cdev->max_level))
		return -EINVAL;

	num_cpus = cpumask_weight(cpufreq_cdev->policy->related_cpus);

	freq = cpufreq_cdev->freq_table[state].frequency;
	dynamic_power = cpu_freq_to_power(cpufreq_cdev, freq) * num_cpus;
	ret = get_static_power(cpufreq_cdev, freq, &static_power);
	if (ret)
		return ret;

	*power = static_power + dynamic_power;
	return ret;
}

/**
 * cpufreq_power2state() - convert power to a cooling device state
 * @cdev:	&thermal_cooling_device pointer
 * @power:	power in milliwatts to be converted
 * @state:	pointer in which to store the resulting state
 *
 * Calculate a cooling device state for the cpus described by @cdev
 * that would allow them to consume at most @power mW and store it in
 * @state.  Note that this calculation depends on external factors
 * such as the cpu load or the current static power.  Calling this
 * function with the same power as input can yield different cooling
 * device states depending on those external factors.
 *
 * Return: 0 on success, -ENODEV if no cpus are online or -EINVAL if
 * the calculated frequency could not be converted to a valid state.
 * The latter should not happen unless the frequencies available to
 * cpufreq have changed since the initialization of the cpu cooling
 * device.
 */
static int cpufreq_power2state(struct thermal_cooling_device *cdev,
			       u32 power, unsigned long *state)
{
	unsigned int cpu, cur_freq, target_freq;
	int ret;
	s32 dyn_power;
	u32 normalised_power, static_power;
	struct exynos_cpu_cooling_device *cpufreq_cdev = cdev->devdata;
	struct cpufreq_policy *policy = cpufreq_cdev->policy;
	int num_cpus;
	struct thermal_zone_device *tz = cpufreq_cdev->tzd;

	num_cpus = cpumask_weight(policy->related_cpus);
	cpu = cpumask_first(policy->related_cpus);

	/* None of our cpus are online */
	if (cpu >= nr_cpu_ids)
		return -ENODEV;

	/* Hack: Driver hasn't finished loading */
	if (!tz)
		return -EAGAIN;

	cur_freq = cpufreq_quick_get(policy->cpu);
	ret = get_static_power(cpufreq_cdev, cur_freq, &static_power);
	if (ret)
		return ret;

	dyn_power = power - static_power;
	dyn_power = dyn_power > 0 ? dyn_power : 0;
	normalised_power = dyn_power / num_cpus;
	target_freq = cpu_power_to_freq(cpufreq_cdev, normalised_power);

	*state = get_level(cpufreq_cdev, target_freq);
	trace_thermal_exynos_power_cpu_limit(tz->id, policy->cpu, target_freq, *state,
					     power);
	return 0;
}

/* Bind cpufreq callbacks to thermal cooling device ops */

static struct thermal_cooling_device_ops exynos_cpu_cooling_ops = {
	.get_max_state = cpufreq_get_max_state,
	.get_cur_state = cpufreq_get_cur_state,
	.set_cur_state = cpufreq_set_cur_state,
};

static struct thermal_cooling_device_ops exynos_cpu_power_cooling_ops = {
	.get_max_state		= cpufreq_get_max_state,
	.get_cur_state		= cpufreq_get_cur_state,
	.set_cur_state		= cpufreq_set_cur_state,
	.get_requested_power	= cpufreq_get_requested_power,
	.state2power		= cpufreq_state2power,
	.power2state		= cpufreq_power2state,
};

static unsigned int find_next_max(struct cpufreq_frequency_table *table,
				  unsigned int prev_max)
{
	struct cpufreq_frequency_table *pos;
	unsigned int max = 0;

	cpufreq_for_each_valid_entry(pos, table) {
		if (pos->frequency > max && pos->frequency < prev_max)
			max = pos->frequency;
	}

	return max;
}

static struct thermal_zone_device *parse_ect_cooling_level(
	struct thermal_cooling_device *cdev, char *cooling_name)
{
	struct thermal_instance *instance;
	struct thermal_zone_device *tz = NULL;
	bool foundtz = false;
	void *thermal_block;
	struct ect_ap_thermal_function *function;
	int i, temperature;
	unsigned int freq;

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
		struct exynos_cpu_cooling_device *cpufreq_cdev = cdev->devdata;
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
		level = get_level(cpufreq_cdev, freq);

		if (level == THERMAL_CSTATE_INVALID)
			level = max_level;

		instance->upper = level;

		pr_info("Parsed From ECT : %s: [%d] Temperature : %d, frequency : %u, level: %d\n",
			cooling_name, i, temperature, freq, level);
	}
skip_ect:
	return tz;
}

ssize_t
state2power_table_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct thermal_cooling_device *cdev = to_cooling_device(dev);
	struct exynos_cpu_cooling_device *cpufreq_cdev = cdev->devdata;
	int i, count = 0;
	u32 power;

	if (!cpufreq_cdev)
		return -ENODEV;

	for (i = 0; i <= cpufreq_cdev->max_level; i++) {
		cpufreq_state2power(cdev, i, &power);
		count += sysfs_emit_at(buf, count, "%u ", power);
	}
	count += sysfs_emit_at(buf, count, "\n");

	return count;
}

static DEVICE_ATTR_RO(state2power_table);

ssize_t
user_vote_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct thermal_cooling_device *cdev = to_cooling_device(dev);
	struct exynos_cpu_cooling_device *cpufreq_cdev = cdev->devdata;

	if (!cpufreq_cdev)
		return -ENODEV;

	return sprintf(buf, "%lu\n", cpufreq_cdev->sysfs_req);
}

ssize_t user_vote_store(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct thermal_cooling_device *cdev = to_cooling_device(dev);
	struct exynos_cpu_cooling_device *cpufreq_cdev = cdev->devdata;
	int ret;
	unsigned long state;

	if (!cpufreq_cdev)
		return -ENODEV;

	ret = kstrtoul(buf, 0, &state);
	if (ret)
		return ret;

	if (state > cpufreq_cdev->max_level)
		return -EINVAL;

	mutex_lock(&cdev->lock);
	cpufreq_cdev->sysfs_req = state;
	cdev->updated = false;
	mutex_unlock(&cdev->lock);
	thermal_cdev_update(cdev);
	return count;
}

static DEVICE_ATTR_RW(user_vote);

ssize_t
user_vote_bypass_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct thermal_cooling_device *cdev = to_cooling_device(dev);
	struct exynos_cpu_cooling_device *cpufreq_cdev = cdev->devdata;

	if (!cpufreq_cdev)
		return -ENODEV;

	return sysfs_emit(buf, "%d\n", cpufreq_cdev->sysfs_req_bypass);
}

ssize_t user_vote_bypass_store(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct thermal_cooling_device *cdev = to_cooling_device(dev);
	struct exynos_cpu_cooling_device *cpufreq_cdev = cdev->devdata;
	int ret;
	bool bypass;

	if (!cpufreq_cdev)
		return -ENODEV;

	ret = kstrtobool(buf, &bypass);
	if (ret)
		return ret;


	mutex_lock(&cdev->lock);
	cpufreq_cdev->sysfs_req_bypass = bypass;
	cdev->updated = false;
	mutex_unlock(&cdev->lock);
	thermal_cdev_update(cdev);
	return count;
}

static DEVICE_ATTR_RW(user_vote_bypass);

/**
 * __exynos_cpu_cooling_register - helper function to create cpufreq cooling device
 * @np: a valid struct device_node to the cooling device device tree node
 * @policy: cpufreq policy
 * Normally this should be same as cpufreq policy->related_cpus.
 * @capacitance: dynamic power coefficient for these cpus
 *
 * This interface function registers the cpufreq cooling device with the name
 * "thermal-cpufreq-%x". This api can support multiple instances of cpufreq
 * cooling devices. It also gives the opportunity to link the cooling device
 * with a device tree node, in order to bind it via the thermal DT code.
 *
 * Return: a valid struct thermal_cooling_device pointer on success,
 * on failure, it returns a corresponding ERR_PTR().
 */
static struct thermal_cooling_device *
__exynos_cpu_cooling_register(struct device_node *np,
			      struct cpufreq_policy *policy, u32 capacitance, char *cooling_name)
{
	struct thermal_cooling_device *cdev;
	struct exynos_cpu_cooling_device *cpufreq_cdev;
	char dev_name[THERMAL_NAME_LENGTH];
	unsigned int freq, i, num_cpus;
	struct device *dev;
	int ret;
	struct thermal_cooling_device_ops *cooling_ops = &exynos_cpu_cooling_ops;
	u32 power;

	dev = get_cpu_device(policy->cpu);
	if (unlikely(!dev)) {
		pr_warn("No cpu device for cpu %d\n", policy->cpu);
		return ERR_PTR(-ENODEV);
	}

	if (IS_ERR_OR_NULL(policy)) {
		pr_err("%s: cpufreq policy isn't valid: %p\n", __func__, policy);
		return ERR_PTR(-EINVAL);
	}

	i = cpufreq_table_count_valid_entries(policy);
	if (!i) {
		pr_debug("%s: CPUFreq table not found or has no valid entries\n",
			 __func__);
		return ERR_PTR(-ENODEV);
	}

	cpufreq_cdev = kzalloc(sizeof(*cpufreq_cdev), GFP_KERNEL);
	if (!cpufreq_cdev)
		return ERR_PTR(-ENOMEM);

	cpufreq_cdev->policy = policy;
	num_cpus = cpumask_weight(policy->related_cpus);
	cpufreq_cdev->idle_time = kcalloc(num_cpus,
					  sizeof(*cpufreq_cdev->idle_time),
					  GFP_KERNEL);
	if (!cpufreq_cdev->idle_time) {
		cdev = ERR_PTR(-ENOMEM);
		goto free_cdev;
	}

	/* max_level is an index, not a counter */
	cpufreq_cdev->max_level = i - 1;

	cpufreq_cdev->freq_table = kmalloc_array(i,
						 sizeof(*cpufreq_cdev->freq_table),
						 GFP_KERNEL);
	if (!cpufreq_cdev->freq_table) {
		cdev = ERR_PTR(-ENOMEM);
		goto free_idle_time;
	}

	ret = ida_simple_get(&cpufreq_ida, 0, 0, GFP_KERNEL);
	if (ret < 0) {
		cdev = ERR_PTR(ret);
		goto free_table;
	}
	cpufreq_cdev->id = ret;

	snprintf(dev_name, sizeof(dev_name), "thermal-cpufreq-%d",
		 cpufreq_cdev->id);

	/* Fill freq-table in descending order of frequencies */
	for (i = 0, freq = -1; i <= cpufreq_cdev->max_level; i++) {
		freq = find_next_max(policy->freq_table, freq);
		cpufreq_cdev->freq_table[i].frequency = freq;

		/* Warn for duplicate entries */
		if (!freq)
			pr_warn("%s: table has duplicate entries\n", __func__);
		else
			pr_info("cpu_cooling %d: freq:%u KHz\n", policy->cpu,
				freq);
	}

	if (capacitance && !update_freq_table(cpufreq_cdev, capacitance)) {
		cooling_ops = &exynos_cpu_power_cooling_ops;
		cpufreq_cdev->has_static = !build_static_power_table(np, cpufreq_cdev, cooling_name);
	}

	ret = freq_qos_add_request(&policy->constraints,
				   &cpufreq_cdev->qos_req, FREQ_QOS_MAX,
				   cpufreq_cdev->freq_table[0].frequency);
	if (ret < 0) {
		pr_err("%s: Failed to add freq constraint (%d)\n", __func__,
		       ret);
		cdev = ERR_PTR(ret);
		goto remove_ida;
	}

	cdev = thermal_of_cooling_device_register(np, dev_name, cpufreq_cdev,
						  cooling_ops);
	if (IS_ERR(cdev))
		goto remove_qos_req;

	ret = device_create_file(&cdev->device, &dev_attr_user_vote);
	if (ret) {
		thermal_cooling_device_unregister(cdev);
		goto remove_qos_req;
	}

	ret = device_create_file(&cdev->device, &dev_attr_user_vote_bypass);
	if (ret) {
		thermal_cooling_device_unregister(cdev);
		goto remove_qos_req;
	}

	ret = device_create_file(&cdev->device, &dev_attr_state2power_table);
	if (ret) {
		thermal_cooling_device_unregister(cdev);
		goto remove_qos_req;
	}

	/* This needs to be done before thermal_of_cooling_device_register ... */
	cpufreq_cdev->tzd = parse_ect_cooling_level(cdev, cooling_name);

	mutex_lock(&cooling_list_lock);
	list_add(&cpufreq_cdev->node, &cpufreq_cdev_list);
	mutex_unlock(&cooling_list_lock);

	pr_info("cpu cooling registered for cpu: %d, capacitance: %d, power_callback: %s, static_power: %s\n",
		policy->cpu, capacitance,
		cooling_ops == &exynos_cpu_power_cooling_ops ? "true" : "false",
		cpufreq_cdev->has_static ? "true" : "false");

	for (i = 0; i <= cpufreq_cdev->max_level; i++) {
		cpufreq_state2power(cdev, i, &power);
		pr_debug("cpu_cooling %d: state:%u power:%u\n", policy->cpu,
			i, power);
	}

	if (init_dsu_bci_constraint_table_dt(cpufreq_cdev, np) == 0) {
		exynos_pm_qos_add_request(&cpufreq_cdev->dsu_qos_max, PM_QOS_DSU_THROUGHPUT_MAX,
					  INT_MAX);
		exynos_pm_qos_add_request(&cpufreq_cdev->bci_qos_max, PM_QOS_BCI_THROUGHPUT_MAX,
					  INT_MAX);
		cpufreq_cdev->apply_dsu_bci_constraints = true;

	} else {
		cpufreq_cdev->apply_dsu_bci_constraints = false;
	}

	return cdev;

remove_qos_req:
	freq_qos_remove_request(&cpufreq_cdev->qos_req);
remove_ida:
	ida_simple_remove(&cpufreq_ida, cpufreq_cdev->id);
free_table:
	kfree(cpufreq_cdev->freq_table);
free_idle_time:
	kfree(cpufreq_cdev->idle_time);
free_cdev:
	kfree(cpufreq_cdev);
	return cdev;
}

/**
 * exynos_cpu_cooling_register - function to create cpufreq cooling device.
 * @policy: cpufreq policy
 *
 * This interface function registers the cpufreq cooling device with the name
 * "thermal-cpufreq-%x". This api can support multiple instances of cpufreq
 * cooling devices.
 *
 * Return: a valid struct thermal_cooling_device pointer on success,
 * on failure, it returns a corresponding ERR_PTR().
 */
struct thermal_cooling_device *
exynos_cpufreq_cooling_register(struct device_node *np, struct cpufreq_policy *policy)
{
	void *gen_block;
	struct ect_gen_param_table *pwr_coeff;
	u32 capacitance = 0;
	u32 index = 0;
	const char *name;
	char cooling_name[THERMAL_NAME_LENGTH];
	struct device_node *cpu_np = of_get_cpu_node(policy->cpu, NULL);

	if (!np)
		return ERR_PTR(-EINVAL);

	if (!cpu_np) {
		pr_err("cpu_cooling: OF node not available for cpu%d\n",
		       policy->cpu);
		of_node_put(cpu_np);
		return ERR_PTR(-EINVAL);
	}
	of_property_read_u32(cpu_np, "dynamic-power-coefficient", &capacitance);
	of_node_put(cpu_np);

	if (of_property_read_string(np, "tz-cooling-name", &name)) {
		pr_err("%s: could not find tz-cooling-name\n", __func__);
		return ERR_PTR(-EINVAL);
	}
	strncpy(cooling_name, name, sizeof(cooling_name));

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
	return __exynos_cpu_cooling_register(np, policy, capacitance, cooling_name);
}
EXPORT_SYMBOL_GPL(exynos_cpufreq_cooling_register);
