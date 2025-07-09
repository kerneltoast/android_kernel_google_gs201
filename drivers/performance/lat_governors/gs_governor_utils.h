/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright 2024 Google, Inc.
 */

#ifndef _GS_GOVERNOR_UTILS_H
#define _GS_GOVERNOR_UTILS_H

#include <linux/kernel.h>
#include <linux/devfreq.h>
#include <performance/gs_perf_mon/gs_perf_mon.h>

/* Conversion parameter */
#define MHZ_TO_KHZ 1000

/**
 * struct gs_governor_core_dev_map - Maps cpu frequency to desired device frequency.
 * @core_khz:			CPU frequency.
 * @target_freq:		The corresponding device frequency.
 */
struct gs_governor_core_dev_map {
	unsigned int core_khz;
	unsigned int target_freq;
};

/**
 * struct cluster_config - Per cluster decision information.
 * @name:			Component name (ex. MIF-CL0).
 * @cpus:			Which cpus this component considers.
 *
 * @ratio_ceil:			Ratio of performance metrics (e.g. instructions
 * 				executed / memory accesses) below which we consider
 * 				the CPU may be stalled by the target device.
 *
 * @stall_floor:		Ratio of performance metrics (e.g. memory stalls / CPU cycles)
 * 				above which we consider the CPU may be stalled by the
 * 				target device.
 *
 * @cpuidle_state_aware:	Whether to consider CPU idle for device frequency calculation.
 *
 * @base_freq_table:		Votes applied on behalf of the CPU cluster if it is not
 * 				determined to be currently stalling.
 *
 * @latency_freq_table:		Votes applied on behalf of the CPU cluster if it is determined
 * 				to be currently stalling. (This should be a more aggressive table
 * 				than base_freq_table).
 */
struct cluster_config {
	const char *name;
	cpumask_t cpus;
	unsigned int ratio_ceil;
	unsigned int stall_floor;
	enum gs_perf_cpu_idle_state cpuidle_state_depth_threshold;
	bool cpuidle_state_aware;
	struct gs_governor_core_dev_map *base_freq_table;
	struct gs_governor_core_dev_map *latency_freq_table;
};

/**
 * gs_governor_core_to_dev_freq - Utility to look up in frequency maps.
 *
 * Inputs:
 * @map:	The frequency table to lookup.
 * @input_freq:	The input lookup frequency in khz.
 *
 * Returns:	Device frequency in khz.
 */
long gs_governor_core_to_dev_freq(struct gs_governor_core_dev_map *map, unsigned long input_freq);

/**
 * gs_governor_init_core_dev_map - populate the frequency map
 *
 * Inputs:
 * @dev:	Device the map belongs to.
 * @of_node:	The device tree node containing the map.
 * @prop_name:	The name of the frequency table in the device tree node.
 *
 * Returns:	Device allocated frequency map.
*/
struct gs_governor_core_dev_map *
gs_governor_init_core_dev_map(struct device *dev, struct device_node *of_node, char *prop_name);

/**
 * populate_cluster_config - Initialize a cluster_config from an input device tree node.
 *
 * Inputs:
 * @dev:		Device this component is for.
 * @cluster_node: 	The device tree node containing latgov configs.
 * @cluster:		Container for the latgov configs.
 *
 * Returns:		Non-zero on error.
*/
int populate_cluster_config(struct device *dev, struct device_node *cluster_node,
			    struct cluster_config *cluster);

/****************************************************************
 *				SYSFS				*
 ****************************************************************/
#define MAKE_CLUSTER_ATTR(_node_name, _name)                                                       \
	static ssize_t _node_name##_##_name##_show(struct device *dev,                             \
						   struct device_attribute *attr, char *buf)       \
	{                                                                                          \
		int res = 0;                                                                       \
		int cluster_idx;                                                                   \
		for (cluster_idx = 0; cluster_idx < _node_name.num_cpu_clusters; cluster_idx++) {  \
			res += sysfs_emit_at(buf, res, "%u\n",                                     \
					     _node_name.cpu_configs_arr[cluster_idx]._name);       \
		}                                                                                  \
		return res;                                                                        \
	}                                                                                          \
                                                                                                   \
	static ssize_t _node_name##_##_name##_store(                                               \
		struct device *dev, struct device_attribute *attr, const char *buf, size_t count)  \
	{                                                                                          \
		int ret = 0;                                                                       \
		int cluster_idx;                                                                   \
		char *token;                                                                       \
		char *token_bookmark;                                                              \
		unsigned int val;                                                                  \
		char *internal_buffer =                                                            \
			kstrndup(buf, 10 * _node_name.num_cpu_clusters, GFP_KERNEL);               \
		if (!internal_buffer)                                                              \
			return -ENOMEM;                                                            \
		token_bookmark = internal_buffer;                                                  \
		for (cluster_idx = 0; cluster_idx < _node_name.num_cpu_clusters; cluster_idx++) {  \
			token = strsep(&token_bookmark, " ");                                      \
			if (!token)                                                                \
				goto free_out;                                                     \
			ret = kstrtouint(token, 0, &val);                                          \
			if (ret)                                                                   \
				goto free_out;                                                     \
			_node_name.cpu_configs_arr[cluster_idx]._name = val;                       \
		}                                                                                  \
	free_out:                                                                                  \
		kfree(internal_buffer);                                                            \
		return count;                                                                      \
	}                                                                                          \
	static DEVICE_ATTR_RW(_node_name##_##_name);

#define SHOW_CLUSTER_FREQ_MAP_ATTR(_node_name, _name)                                              \
	static ssize_t _node_name##_##_name##_show(struct device *dev,                             \
						   struct device_attribute *attr, char *buf)       \
	{                                                                                          \
		int cluster_idx;                                                                   \
		unsigned int cnt = 0;                                                              \
		struct gs_governor_core_dev_map *map = NULL;                                       \
		for (cluster_idx = 0; cluster_idx < _node_name.num_cpu_clusters; cluster_idx++) {  \
			map =  _node_name.cpu_configs_arr[cluster_idx]._name;                      \
			while (map->core_khz) {                                                    \
				cnt += sysfs_emit_at(buf, cnt, "cl%d: %10u\t%9u\n", cluster_idx,   \
				  map->core_khz, map->target_freq);                                \
				map++;                                                             \
			}                                                                          \
		}                                                                                  \
		return cnt;                                                                        \
	}                                                                                          \
                                                                                                   \
	static DEVICE_ATTR_RO(_node_name##_##_name);

#endif /*  _GS_GOVERNOR_UTILS_H */
