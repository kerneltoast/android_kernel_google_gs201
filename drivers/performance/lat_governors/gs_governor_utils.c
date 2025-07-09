// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2024 Google LLC.
 *
 * A Utility File for Shared Functions Between
 * Google Governors.
 */
#define pr_fmt(fmt) "gs_governor_utils: " fmt

#include <linux/cpu.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/of.h>
#include <performance/gs_perf_mon/gs_perf_mon.h>

#include "gs_governor_utils.h"

#define NUM_FREQ_TABLE_COLS 2

/* Initialize the tracepoints from this file. */
#define CREATE_TRACE_POINTS
#include "gs_lat_governors_trace.h"
EXPORT_TRACEPOINT_SYMBOL_GPL(gs_lat_governor);

long gs_governor_core_to_dev_freq(struct gs_governor_core_dev_map *map, unsigned long input_freq)
{
	unsigned long freq = 0;

	if (!map)
		goto out;

	/*
	 * We use the target frequency for the highest node with a frequency
	 * *less or equal* to the current frequency. I.e. the table entries
	 * should each represent new explicit thresholds for frequencies exceeding
	 * the specified values.
	 */
	while (map->core_khz && map->core_khz <= input_freq) {
		freq = map->target_freq;
		map++;
	}

out:
	pr_debug("freq: %lu -> dev: %lu\n", input_freq, freq);
	return freq;
}
EXPORT_SYMBOL(gs_governor_core_to_dev_freq);

struct gs_governor_core_dev_map *
gs_governor_init_core_dev_map(struct device *dev, struct device_node *of_node, char *prop_name)
{
	int len, nf, i, j;
	u32 data;
	struct gs_governor_core_dev_map *tbl;
	int ret;

	if (!of_node)
		of_node = dev->of_node;

	if (!of_find_property(of_node, prop_name, &len))
		return NULL;

	len /= sizeof(data);

	if (len %  NUM_FREQ_TABLE_COLS || len == 0)
		return NULL;

	nf = len /  NUM_FREQ_TABLE_COLS;
	tbl = devm_kzalloc(dev, (nf + 1) * sizeof(*tbl), GFP_KERNEL);
	if (!tbl)
		return NULL;

	for (i = 0; i < nf; i++) {
		j = 2*i;
		ret = of_property_read_u32_index(of_node, prop_name, j, &data);
		if (ret)
			return NULL;

		tbl[i].core_khz = data;

		ret = of_property_read_u32_index(of_node, prop_name, j + 1, &data);
		if (ret)
			return NULL;

		tbl[i].target_freq = data;
		pr_debug("Entry%d CPU:%u, Dev:%u\n", i, tbl[i].core_khz, tbl[i].target_freq);
	}

	/* This is a sentinel marking the end of the array. */
	tbl[i].core_khz = 0;

	return tbl;
}
EXPORT_SYMBOL(gs_governor_init_core_dev_map);

/**
 * parse_device_node_cpumask - General parser for cpu masks.
 *
 * Inputs:
 * @np:			Pointer to device tree node containing mask.
 * @mask:		Container for resulting cpu_mask.
 * @cpu_list_name:	Name of the cpu mask attribute.
 *
 * Returns:		Non-zero on error.
*/
static int parse_device_node_cpumask(struct device_node *np, cpumask_t *mask, char *cpu_list_name)
{
	struct device_node *dev_phandle;
	struct device *cpu_dev;
	int cpu, i = 0;
	int ret = -ENOENT;

	dev_phandle = of_parse_phandle(np, cpu_list_name, i++);
	while (dev_phandle) {
		for_each_possible_cpu(cpu) {
			cpu_dev = get_cpu_device(cpu);
			if (cpu_dev && cpu_dev->of_node == dev_phandle) {
				cpumask_set_cpu(cpu, mask);
				ret = 0;
				break;
			}
		}
		dev_phandle = of_parse_phandle(np, cpu_list_name, i++);
	}
	return ret;
}

int populate_cluster_config(struct device *dev, struct device_node *cluster_node,
			    struct cluster_config *cluster)
{
	/* Retrieve the list of CPUs in the current cluster. */
	int ret = 0;
	ret = parse_device_node_cpumask(cluster_node, &cluster->cpus, "cpulist");
	if (ret) {
		dev_err(dev, "Can't parse cpu_list.\n");
		return -EINVAL;
	}

	ret = of_property_read_u32(cluster_node, "ratio_ceil", &(cluster->ratio_ceil));
	if (ret) {
		dev_err(dev, "ratio_ceil unspecified.\n");
		return -EINVAL;
	}

	ret = of_property_read_u32(cluster_node, "stall_floor", &(cluster->stall_floor));
	if (ret) {
		dev_err(dev, "stall_floor unspecified.\n");
		return -EINVAL;
	}

	/*
	 * Idle Aware Config. Expected to be 1 for C1 idle or 2 for C2 idle.
	 */
	if (of_property_read_u32(cluster_node, "cpuidle_state_depth_threshold", &cluster->cpuidle_state_depth_threshold)) {
		dev_dbg(dev, "Config idle_aware mising. Defaulting to no idle awareness.\n");
		cluster->cpuidle_state_aware = false;
	}

	if (cluster->cpuidle_state_depth_threshold == PERF_CPU_IDLE_C1 || cluster->cpuidle_state_depth_threshold == PERF_CPU_IDLE_C2)
		cluster->cpuidle_state_aware = true;
	else
		cluster->cpuidle_state_aware = false;

	/* Parsing latency_freq_table. */
	cluster->latency_freq_table =
		gs_governor_init_core_dev_map(dev, cluster_node, "core-dev-table-latency");
	if (!cluster->latency_freq_table) {
		dev_err(dev, "Couldn't find the core-dev-table-latency! Aborting probe!\n");
		return -EINVAL;
	}

	/* Parsing base_freq_table. */
	cluster->base_freq_table =
		gs_governor_init_core_dev_map(dev, cluster_node, "core-dev-table-base");
	if (!cluster->base_freq_table) {
		dev_err(dev,
			"Couldn't find the core-dev-table-base! Disabling Base Vote!\n");
	}
	return ret;
}
EXPORT_SYMBOL(populate_cluster_config);

MODULE_AUTHOR("Will Song <jinpengsong@google.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Google Source Governor Utilities");