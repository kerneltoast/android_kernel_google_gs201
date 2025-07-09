// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2024 Google LLC.
 *
 * Memory Latency Governor Main Module.
 */
#define pr_fmt(fmt) "gs_governor_memlat: " fmt

#include <dt-bindings/soc/google/zuma-devfreq.h>
#include <governor.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <performance/gs_perf_mon/gs_perf_mon.h>
#include <soc/google/exynos-devfreq.h>
#include <trace/events/power.h>

#include "gs_governor_utils.h"
#include "gs_lat_governors_trace.h"

/**
 * struct frequency_vote - Contains configs and voting data.
 * @vote_name:		The name of the device we will vote frequency for.
 * @min_freq_req:	The min vote we assert for the device frequency.
 */
struct frequency_vote {
	const char* vote_name;
	struct exynos_pm_qos_request min_freq_req;
};

/**
 * struct memlat_data - Node containing memlat's global data.
 * @gov_is_on:		Governor's active state.
 * @attr_grp:		Tuneable governor parameters exposed to userspace.
 * @dev:		Reference to the governor's device.
 * @num_cpu_clusters:	Number of CPU clusters the governor will service.
 * @cpu_configs_arr:	Configurations for each cluster's latency vote.
 * @target_freq_vote:	Primary target domain.
 */
struct memlat_data {
	bool gov_is_on;
	struct attribute_group *attr_grp;
	struct device *dev;
	int num_cpu_clusters;
	struct cluster_config *cpu_configs_arr;
	struct frequency_vote target_freq_vote;
};

static void update_memlat_gov(struct gs_cpu_perf_data *data, void *private_data);

/* Global monitor client used to get callbacks when gs_perf_mon data is updated. */
static struct gs_perf_mon_client memlat_perf_client = {
	.client_callback = update_memlat_gov,
	.name = "memlat"
};

/* Memlat datastructure holding memlat governor configurations and metadata. */
static struct memlat_data memlat_node;

/* Macro expansions for sysfs nodes. */
MAKE_CLUSTER_ATTR(memlat_node, stall_floor);
MAKE_CLUSTER_ATTR(memlat_node, ratio_ceil);
MAKE_CLUSTER_ATTR(memlat_node, cpuidle_state_depth_threshold);

SHOW_CLUSTER_FREQ_MAP_ATTR(memlat_node, latency_freq_table);

/* These sysfs emitters also depend on the macro expansions from above. */
static struct attribute *memlat_dev_attr[] = {
	&dev_attr_memlat_node_stall_floor.attr,
	&dev_attr_memlat_node_ratio_ceil.attr,
	&dev_attr_memlat_node_cpuidle_state_depth_threshold.attr,
	&dev_attr_memlat_node_latency_freq_table.attr,
	NULL,
};

/* Sysfs files to expose. */
static struct attribute_group memlat_dev_attr_group = {
	.name = "memlat_attr",
	.attrs = memlat_dev_attr,
};

/**
 * gs_governor_memlat_update_target_freq_vote - Registers the vote for the memlat governor.
 *
 * Inputs:
 * @vote:		The domain to vote on.
 * @target_freq:	New target frequency.
 *
 * Outputs:
 *			Non-zero on error.
*/
static int gs_governor_memlat_update_target_freq_vote(struct frequency_vote *vote,
					    unsigned long target_freq)
{
	if (!exynos_pm_qos_request_active(&vote->min_freq_req))
		return -ENODEV;

	exynos_pm_qos_update_request_async(&vote->min_freq_req, target_freq);
	trace_clock_set_rate(vote->vote_name, target_freq, raw_smp_processor_id());

	return 0;
}

/**
 * gs_governor_memlat_compute_freq - Calculates memlat freq votes for each CPU cluster.
 *
 * This function determines the memlat target frequency.
 *
 * Input:
 * @cpu_perf_data_arr:	CPU data to use as input.
 *
 * Returns:
 * @max_freq:		The computed target frequency.
*/
static unsigned long gs_governor_memlat_compute_freq(struct gs_cpu_perf_data *cpu_perf_data_arr)
{
	int cpu;
	int cluster_idx;
	struct cluster_config *cluster;
	unsigned long max_freq = 0;
	char trace_name[] = { 'c', 'p', 'u', '0', 'm', 'i', 'f', '\0' };

	/* For each cluster, we make a frequency decision. */
	for (cluster_idx = 0; cluster_idx < memlat_node.num_cpu_clusters; cluster_idx++) {
		cluster = &memlat_node.cpu_configs_arr[cluster_idx];
		for_each_cpu(cpu, &cluster->cpus) {
			unsigned long ratio, mem_stall_pct, mem_stall_floor, ratio_ceil;
			unsigned long l3_cachemiss, mem_stall, cyc, last_delta_us, inst;
			unsigned long mif_freq = 0, effective_cpu_freq_khz;
			bool memlat_cpuidle_state_aware;
			enum gs_perf_cpu_idle_state memlat_configured_idle_depth_threshold;
			struct gs_cpu_perf_data *cpu_data = &cpu_perf_data_arr[cpu];
			trace_name[3] = '0' + cpu;

			/* Check if the cpu monitor is up. */
			if (!cpu_data->cpu_mon_on)
				goto early_exit;

			l3_cachemiss = cpu_data->perf_ev_last_delta[PERF_L3_CACHE_MISS_IDX];
			mem_stall = cpu_data->perf_ev_last_delta[PERF_STALL_BACKEND_MEM_IDX];
			cyc = cpu_data->perf_ev_last_delta[PERF_CYCLE_IDX];
			inst = cpu_data->perf_ev_last_delta[PERF_INST_IDX];
			last_delta_us = cpu_data->time_delta_us;

			ratio_ceil = cluster->ratio_ceil;
			mem_stall_floor = cluster->stall_floor;
			memlat_cpuidle_state_aware = cluster->cpuidle_state_aware;
			memlat_configured_idle_depth_threshold = cluster->cpuidle_state_depth_threshold;

			/* Compute threshold data. */
			if (l3_cachemiss != 0)
				ratio = inst / l3_cachemiss;
			else
				ratio = inst;

			mem_stall_pct = mult_frac(10000, mem_stall, cyc);
			effective_cpu_freq_khz = MHZ_TO_KHZ * cyc / last_delta_us;

			if (memlat_cpuidle_state_aware && cpu_data->cpu_idle_state >= memlat_configured_idle_depth_threshold)
   				goto early_exit; // Zeroing vote for sufficiently idle CPUs.

			/* If we pass the threshold, use the latency table. */
			if (ratio <= ratio_ceil && mem_stall_pct >= mem_stall_floor)
				mif_freq = gs_governor_core_to_dev_freq(cluster->latency_freq_table,
									effective_cpu_freq_khz);
			else if (cluster->base_freq_table)
				mif_freq = gs_governor_core_to_dev_freq(cluster->base_freq_table,
									effective_cpu_freq_khz);
			/* Keep a running max of the MIF frequency. */
			if (mif_freq > max_freq)
				max_freq = mif_freq;

			trace_gs_lat_governor("memlat", cpu, ratio, mem_stall_pct,
				mif_freq, effective_cpu_freq_khz);
		early_exit:
			/* Leave a trace for the cluster desired MIF frequency. */
			trace_clock_set_rate(trace_name, mif_freq, cpu);
		}
	}

	return max_freq;
}

/**
 * update_memlat_gov  -	Callback function from the perf monitor to service
 * 			the memlat governor.
 *
 * Input:
 * @data:		Performance data from the monitor.
 * @private_data:	Unused.
 *
*/
static void update_memlat_gov(struct gs_cpu_perf_data *data, void* private_data)
{
	unsigned long next_frequency;

	/* If the memlat governor is not active. Reset our vote to minimum. */
	if (!memlat_node.gov_is_on || !data) {
		dev_dbg(memlat_node.dev, "Memlat governor is not active. Leaving vote unchanged.\n");
		return;
	}

	/* Step 1: compute the frequency. */
	next_frequency = gs_governor_memlat_compute_freq(data);

	/* Step 2: send it as a vote. */
	gs_governor_memlat_update_target_freq_vote(&memlat_node.target_freq_vote, next_frequency);
}

/**
 * gs_memlat_governor_remove_all_votes - Removes all the votes for memlat governor.
*/
static void gs_memlat_governor_remove_all_votes(void) {
	/* Remove all votes. */
	struct frequency_vote *vote = &memlat_node.target_freq_vote;
	exynos_pm_qos_remove_request(&vote->min_freq_req);
}

/**
 * gov_start - Starts the governor.
*/
static int gov_start(void)
{
	int ret;
	if (memlat_node.gov_is_on)
		return 0;

	/* Add clients. */
	ret = gs_perf_mon_add_client(&memlat_perf_client);
	if (ret)
		return ret;

	memlat_node.gov_is_on = true;

	return 0;
}

/**
 * gov_stop - Stops the governor.
*/
static void gov_stop(void)
{
	if (!memlat_node.gov_is_on)
		return;

	memlat_node.gov_is_on = false;

	/* Remove the client. */
	gs_perf_mon_remove_client(&memlat_perf_client);

	/* Reset the vote to minimum. */
	gs_governor_memlat_update_target_freq_vote(&memlat_node.target_freq_vote, 0);

}

/**
 * gs_memlat_governor_initialize_vote - Initializes the votes for the memlat.
 *
 * Input:
 * @vote_node:	Node containing the vote config.
 * @dev:	The device the governor is binded on.
 *
 * Output:	Non-zero on error.
*/
static int gs_memlat_governor_initialize_vote(struct device_node *vote_node, struct device *dev) {
	struct frequency_vote *primary_frequency_vote = &memlat_node.target_freq_vote;
	u32 pm_qos_class;

	if (of_property_read_string(vote_node, "vote_name",
				    &primary_frequency_vote->vote_name)) {
		dev_err(dev, "The vote device name is undefined.\n");
		return -ENODEV;
	}

	if (of_property_read_u32(vote_node, "pm_qos_class",
				 &pm_qos_class)) {
		dev_err(dev, "The pm_qos_class is undefined.\n");
		return -ENODEV;
	}

	exynos_pm_qos_add_request(&primary_frequency_vote->min_freq_req, (int)pm_qos_class, 0);

	return 0;
}

/**
 * gs_memlat_governor_initialize - Initializes the dsulat governor from a DT Node.
 *
 * Inputs:
 * @governor_node:	The tree node contanin governor data.
 * @data:		The devfreq data to update frequencies.
 *
 * Returns:		Non-zero on error.
*/
static int gs_memlat_governor_initialize(struct device_node *governor_node, struct device *dev)
{
	int ret = 0;
	struct device_node *cluster_node = NULL;
	struct cluster_config *cluster;
	int cluster_idx;

	memlat_node.num_cpu_clusters = of_get_child_count(governor_node);

	/* Allocate a container for clusters. */
	memlat_node.cpu_configs_arr = devm_kzalloc(
		dev, sizeof(struct cluster_config) * memlat_node.num_cpu_clusters, GFP_KERNEL);
	if (!memlat_node.cpu_configs_arr) {
		dev_err(dev, "No memory for cluster_configs.\n");
		return -ENOMEM;
	}

	/* Populate the Components. */
	cluster_idx = 0;
	while ((cluster_node = of_get_next_child(governor_node, cluster_node)) != NULL) {
		cluster = &memlat_node.cpu_configs_arr[cluster_idx];
		if ((ret = populate_cluster_config(dev, cluster_node, cluster)))
			return ret;

		/* Increment pointer. */
		cluster_idx += 1;
	}
	return 0;
}

static int gs_governor_memlat_driver_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *governor_config_node, *frequency_vote_node;
	int ret;

	memlat_node.dev = &pdev->dev;
	memlat_node.attr_grp = &memlat_dev_attr_group;

	/* Find and intitialize frequency votes. */
	frequency_vote_node = of_get_child_by_name(dev->of_node, "primary_vote_config");
	if (!frequency_vote_node) {
		dev_err(dev, "Memlat frequency_votes not defined.\n");
		return -ENODEV;
	}

	ret = gs_memlat_governor_initialize_vote(frequency_vote_node, dev);
	if (ret) {
		dev_err(dev, "Failed to parse memlat governor node data.\n");
		return ret;
	}

	/* Find and initialize governor. */
	governor_config_node = of_get_child_by_name(dev->of_node, "governor_config");
	if (!governor_config_node) {
		dev_err(dev, "Memlat Governor node not defined.\n");
		ret = -ENODEV;
		goto err_out;
	}

	ret = gs_memlat_governor_initialize(governor_config_node, dev);
	if (ret) {
		dev_err(dev, "Failed to parse private governor data.\n");
		goto err_out;
	}

	/* Add sysfs nodes here. */
	ret = sysfs_create_group(&dev->kobj, memlat_node.attr_grp);
	if (ret) {
		dev_err(dev, "Failed to initialize governor sysfs groups.\n");
		goto err_out;
	}

	/* Start the governor servicing. */
	ret = gov_start();
	if (ret) {
		dev_err(dev, "Failed to start memlat governor.\n");
		goto err_gov_start;
	}

	return 0;

err_gov_start:
	sysfs_remove_group(&memlat_node.dev->kobj, memlat_node.attr_grp);
err_out:
	gs_memlat_governor_remove_all_votes();

	return ret;
}

static int gs_governor_memlat_driver_remove(struct platform_device *pdev)
{
	/* Stop governor servicing. */
	gov_stop();

	/* Remove Sysfs here. */
	sysfs_remove_group(&memlat_node.dev->kobj, memlat_node.attr_grp);

	/* Remove pm_qos vote here. */
	gs_memlat_governor_remove_all_votes();

	return 0;
}

static const struct of_device_id gs_governor_memlat_root_match[] = {
	{
		.compatible = "google,gs_governor_memlat",
	},
	{}
};
MODULE_DEVICE_TABLE(of, gs_governor_memlat_root_match);

static struct platform_driver gs_governor_memlat_platform_driver = {
	.probe = gs_governor_memlat_driver_probe,
	.remove = gs_governor_memlat_driver_remove,
	.driver = {
		.name = "gs_governor_memlat",
		.owner = THIS_MODULE,
		.of_match_table = gs_governor_memlat_root_match,
		.suppress_bind_attrs = true,
	},
};

module_platform_driver(gs_governor_memlat_platform_driver);
MODULE_AUTHOR("Will Song <jinpengsong@google.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Google Source Memlat Governor");
