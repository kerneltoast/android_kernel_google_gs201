// SPDX-License-Identifier: GPL-2.0-only
/* cpuidle_metrics.c
 *
 * Support for CPU Idle metrics
 *
 * Copyright 2023 Google LLC
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt
#include <linux/module.h>
#include <linux/init.h>
#include <linux/of_device.h>
#include <linux/cpu.h>
#include <linux/cpuidle.h>
#include <linux/sysfs.h>
#include <linux/kobject.h>
#include <trace/events/power.h>
#include <soc/google/cpuidle_metrics.h>

/* Constants for histogram bins */
#define NUM_TARGET_BINS 8
#define PERCENT_INCREMENT 25

/* Constants for cluster stats */
#define NAME_LEN 32
#define MAX_CLUSTERS 10

/* struct to store histogram statistics per state per cpu or cluster*/
struct histogram_stats {
	/* time entered power mode */
	ktime_t entry_time;

	/*
	 * histogram bins by % of target residency incremented by %increment -> 0%, 25%, 50%, 75%,
	 * 100%, 125%, 150%
	 */
	unsigned int target_time_hist[NUM_TARGET_BINS + 1];
};

struct power_stats {
	char name[NAME_LEN];
	int target_residency;
	int entered_state;
	struct histogram_stats hist_stats[CPUIDLE_STATE_MAX];
	bool initialized;
};

/* variables for cpu and cluster stats */
DEFINE_PER_CPU(struct power_stats, all_cpu_stats);
static DEFINE_PER_CPU(spinlock_t, cpu_spinlocks);
static struct power_stats all_cluster_stats[MAX_CLUSTERS];
static spinlock_t cluster_spinlocks[MAX_CLUSTERS];
static int cpuidle_state_max;
static bool histograms_enabled = false;

/* method to append to the cpuidle histogram */
static void histogram_append(struct histogram_stats *hist_stat, s64 time_us,
			     s64 target_residency_us)
{
	int percent_index;

	/* append to target residency histogram */
	percent_index = time_us * 100 / target_residency_us / PERCENT_INCREMENT;
	if (percent_index > NUM_TARGET_BINS)
		percent_index = NUM_TARGET_BINS;
	hist_stat->target_time_hist[percent_index] += 1;
}

/* method to register all of the cpu cluster stats */
void cpuidle_metrics_histogram_register(char *name, int cluster_id, s64 target_residency_us)
{
	if (cluster_id < 0 || cluster_id >= MAX_CLUSTERS) {
		pr_err("Invalid cluster_id %d passed for registration of histrogram stats",
				cluster_id);
		return;
	}
	spin_lock(&cluster_spinlocks[cluster_id]);
	if (all_cluster_stats[cluster_id].initialized) {
		spin_unlock(&cluster_spinlocks[cluster_id]);
		return;
	}
	strlcpy(all_cluster_stats[cluster_id].name, name, NAME_LEN);
	all_cluster_stats[cluster_id].target_residency = target_residency_us;
	all_cluster_stats[cluster_id].initialized = true;
	spin_unlock(&cluster_spinlocks[cluster_id]);
}
EXPORT_SYMBOL_GPL(cpuidle_metrics_histogram_register);

/* method to append to histogram from external modules */
inline void cpuidle_metrics_histogram_append(int cluster_id, s64 time_us)
{
	if (cluster_id < 0 || cluster_id >= MAX_CLUSTERS) {
		pr_err("Invalid cluster_id passed for histogram creation\n");
		return;
	}
	spin_lock(&cluster_spinlocks[cluster_id]);
	if (!all_cluster_stats[cluster_id].initialized) {
		pr_err("cluster_id %d not initialized for histogram creation", cluster_id);
		spin_unlock(&cluster_spinlocks[cluster_id]);
		return;
	}
	histogram_append(&all_cluster_stats[cluster_id].hist_stats[0], time_us,
			 all_cluster_stats[cluster_id].target_residency);
	spin_unlock(&cluster_spinlocks[cluster_id]);
}

EXPORT_SYMBOL_GPL(cpuidle_metrics_histogram_append);

static inline void reset_all_histograms(void)
{
	int idle_state, cluster, cpu = 0;
	struct power_stats *cpu_stat;
	struct power_stats *cluster_stat;

	/* clear all histograms per cpu per state */
	for (idle_state = 0; idle_state <= cpuidle_state_max; idle_state++) {
		for_each_possible_cpu (cpu) {
			spin_lock(&per_cpu(cpu_spinlocks, cpu));
			cpu_stat = &per_cpu(all_cpu_stats, cpu);
			memset(cpu_stat->hist_stats[idle_state].target_time_hist, 0,
				   sizeof(cpu_stat->hist_stats[idle_state].target_time_hist));
			spin_unlock(&per_cpu(cpu_spinlocks, cpu));
		}
	}

	/* clear all histograms per cluster */
	for (cluster = 0; cluster < MAX_CLUSTERS; cluster++) {
		spin_lock(&cluster_spinlocks[cluster]);
		if (all_cluster_stats[cluster].initialized) {
			cluster_stat = &all_cluster_stats[cluster];
			memset(cluster_stat->hist_stats[0].target_time_hist, 0,
				   sizeof(cluster_stat->hist_stats[0].target_time_hist));
		}
		spin_unlock(&cluster_spinlocks[cluster]);
	}
}

static void cpu_idle_hook(void *data, unsigned int state, unsigned int cpu)
{
	struct power_stats *cpu_stat;
	spin_lock(&per_cpu(cpu_spinlocks, cpu));
	cpu_stat = &per_cpu(all_cpu_stats, cpu);

	if (state != PWR_EVENT_EXIT) {
		/* log entered state and time */
		cpu_stat->entered_state = state;
		cpu_stat->hist_stats[cpu_stat->entered_state].entry_time = ktime_get_mono_fast_ns();
	} else {
		s64 time_delta;
		struct histogram_stats hist_stat = cpu_stat->hist_stats[cpu_stat->entered_state];

		/* find time delta and append to histogram */
		time_delta = ktime_to_us(ktime_sub(ktime_get_mono_fast_ns(), hist_stat.entry_time));
		histogram_append(&cpu_stat->hist_stats[cpu_stat->entered_state], time_delta,
				 cpu_stat->target_residency);
	}

	spin_unlock(&per_cpu(cpu_spinlocks, cpu));
}

static ssize_t cpuidle_histogram_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int idle_state, bin_num, cpu, ret = 0;
	struct power_stats cpu_stat;

	ret = sysfs_emit(buf,
		"\n0%% %d%% %d%% \n\n",
		NUM_TARGET_BINS * PERCENT_INCREMENT, PERCENT_INCREMENT);

	for (idle_state = 0; idle_state <= cpuidle_state_max; idle_state++) {
		// emit state
		ret += sysfs_emit_at(buf, ret, "%d\n", idle_state);
		for_each_possible_cpu (cpu) {
			spin_lock(&per_cpu(cpu_spinlocks, cpu));
			cpu_stat = per_cpu(all_cpu_stats, cpu);
			// emit cpu and target residency in us
			ret += sysfs_emit_at(buf, ret,
					"%d %d\n", cpu,
					cpu_stat.target_residency);

			/* output histogram */
			for (bin_num = 0; bin_num <= NUM_TARGET_BINS; bin_num++) {
				ret += sysfs_emit_at(buf, ret, "%d ",
						cpu_stat.hist_stats[idle_state].target_time_hist[bin_num]);
			}
			ret += sysfs_emit_at(buf, ret, "\n");
			spin_unlock(&per_cpu(cpu_spinlocks, cpu));
		}
		ret += sysfs_emit_at(buf, ret, "\n");
	}

	return ret;
}

static ssize_t cpucluster_histogram_show(struct kobject *kobj, struct kobj_attribute *attr,
					 char *buf)
{
	int cluster, bin_num, ret = 0;
	struct power_stats cluster_stat;

	ret = sysfs_emit(buf,
		"\n0%% %d%% %d %%\n\n",
		NUM_TARGET_BINS * PERCENT_INCREMENT, PERCENT_INCREMENT);

	for (cluster = 0; cluster < MAX_CLUSTERS; cluster++) {
		spin_lock(&cluster_spinlocks[cluster]);
		if (all_cluster_stats[cluster].initialized) {
			cluster_stat = all_cluster_stats[cluster];
			// output cluster name and target residency
			ret += sysfs_emit_at(buf, ret,
					"%-7s %d\n", cluster_stat.name,
					cluster_stat.target_residency);

			/* output histogram */
			for (bin_num = 0; bin_num <= NUM_TARGET_BINS; bin_num++) {
				ret += sysfs_emit_at(
					buf, ret, "%d ",
					cluster_stat.hist_stats[0].target_time_hist[bin_num]);
			}
			ret += sysfs_emit_at(buf, ret, "\n");
		}
		spin_unlock(&cluster_spinlocks[cluster]);
	}

	return ret;
}

static ssize_t cpuidle_histogram_enable_store(struct kobject *kobj,
					  struct kobj_attribute *attr,
					  const char *buf,
					  size_t count)
{
	bool enable_histograms;
	int err = kstrtobool(buf, &enable_histograms);
	if (err)
		return -EINVAL;
	if (enable_histograms && !histograms_enabled)
		register_trace_cpu_idle(cpu_idle_hook, NULL);
	else if (!enable_histograms && histograms_enabled) {
		unregister_trace_cpu_idle(cpu_idle_hook, NULL);
		reset_all_histograms();
	}

	histograms_enabled = enable_histograms;

	return count;
}

static ssize_t cpuidle_histogram_enable_show(struct kobject *kobj,
					struct kobj_attribute *attr,
					char *buf)
{
	return sysfs_emit(buf, "%d\n", histograms_enabled);
}

static ssize_t cpuidle_histogram_reset_store(struct kobject *kobj,
					  struct kobj_attribute *attr,
					  const char *buf,
					  size_t count)
{
	reset_all_histograms();
	return count;
}

static struct kobj_attribute cpuidle_histogram_attr =
	__ATTR(cpuidle_histogram, 0444, cpuidle_histogram_show, NULL);
static struct kobj_attribute cpucluster_histogram_attr =
	__ATTR(cpucluster_histogram, 0444, cpucluster_histogram_show, NULL);
static struct kobj_attribute cpuidle_histogram_enable_attr =
	__ATTR(enable, 0664, cpuidle_histogram_enable_show, cpuidle_histogram_enable_store);
static struct kobj_attribute cpuidle_histogram_reset_attr =
	__ATTR(reset, 0200, NULL, cpuidle_histogram_reset_store);


static struct attribute *cpuidle_histogram_attrs[] = {
	&cpuidle_histogram_attr.attr,
	&cpucluster_histogram_attr.attr,
	&cpuidle_histogram_enable_attr.attr,
	&cpuidle_histogram_reset_attr.attr,
	NULL };

static const struct attribute_group cpuidle_histogram_attr_group = {
	.attrs = cpuidle_histogram_attrs,
	.name = "cpuidle_histogram"
};

int cpuidle_metrics_init(struct kobject *metrics_kobj)
{
	int ret = 0;
	int cpu, target_residency, max, cluster_index;

	if (!metrics_kobj) {
		pr_err("metrics_kobj is not initialized\n");
		return -EINVAL;
	}
	ret = sysfs_create_group(metrics_kobj, &cpuidle_histogram_attr_group);
	if (ret) {
		pr_err("failed to create cpuidle_histogram folder\n");
		return ret;
	}

	for_each_possible_cpu (cpu) {
		struct device_node *cpu_node, *state_node;
		struct power_stats *stats = &per_cpu(all_cpu_stats, cpu);
		spin_lock_init(&per_cpu(cpu_spinlocks, cpu));

		/* find min residency per cpu */
		cpu_node = of_cpu_device_node_get(cpu);
		state_node = of_parse_phandle(cpu_node, "cpu-idle-states", 0);
		ret = of_property_read_u32(state_node, "min-residency-us", &target_residency);
		stats->target_residency = target_residency;

		/* find maximum idle state */
		max = of_count_phandle_with_args(cpu_node, "cpu-idle-states", NULL);
		if (max > cpuidle_state_max)
			cpuidle_state_max = max;
	}

	for (cluster_index = 0; cluster_index < MAX_CLUSTERS; cluster_index++) {
		all_cluster_stats[cluster_index].initialized = false;
		spin_lock_init(&cluster_spinlocks[cluster_index]);
	}

	histograms_enabled = false;

	pr_debug("cpuidle_metrics driver initialized!\n");
	return ret;
}
