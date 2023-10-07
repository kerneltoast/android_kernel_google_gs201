// SPDX-License-Identifier: GPL-2.0
/*
 * EdgeTPU usage stats
 *
 * Copyright (C) 2020 Google, Inc.
 */

#include <linux/slab.h>
#include <linux/sysfs.h>

#include "edgetpu-config.h"
#include "edgetpu-internal.h"
#include "edgetpu-kci.h"
#include "edgetpu-usage-stats.h"

/* Max number of frequencies to support */
#define EDGETPU_MAX_STATES	10

struct uid_entry {
	int32_t uid;
	uint64_t time_in_state[EDGETPU_MAX_STATES];
	struct hlist_node node;
};

static int tpu_state_map(struct edgetpu_dev *etdev, uint32_t state)
{
	int i, idx = 0;

	mutex_lock(&etdev->freq_lock);
	/* Use frequency table if f/w already reported via usage_stats */
	if (etdev->freq_table) {
		for (i = etdev->freq_count - 1; i >= 0; i--) {
			if (state == etdev->freq_table[i])
				idx = i;
		}
		mutex_unlock(&etdev->freq_lock);
		return idx;
	}

	mutex_unlock(&etdev->freq_lock);

	/*
	 * use predefined state table in case of no f/w reported supported
	 * frequencies.
	 */
	for (i = (EDGETPU_NUM_STATES - 1); i >= 0; i--) {
		if (state >= edgetpu_active_states[i])
			return i;
	}

	return 0;
}

/* Caller must hold usage_stats lock */
static struct uid_entry *
find_uid_entry_locked(int32_t uid, struct edgetpu_usage_stats *ustats)
{
	struct uid_entry *uid_entry;

	hash_for_each_possible(ustats->uid_hash_table, uid_entry, node, uid) {
		if (uid_entry->uid == uid)
			return uid_entry;
	}

	return NULL;
}

int edgetpu_usage_add(struct edgetpu_dev *etdev, struct tpu_usage *tpu_usage)
{
	struct edgetpu_usage_stats *ustats = etdev->usage_stats;
	struct uid_entry *uid_entry;
	int state = tpu_state_map(etdev, tpu_usage->power_state);

	if (!ustats)
		return 0;

	/* Note: as of metrics v2 the cluster_id is always zero and is ignored. */
	etdev_dbg(etdev, "%s: uid=%u state=%u dur=%u", __func__,
		  tpu_usage->uid, tpu_usage->power_state,
		  tpu_usage->duration_us);
	mutex_lock(&ustats->usage_stats_lock);

	/* Find the uid in uid_hash_table first */
	uid_entry = find_uid_entry_locked(tpu_usage->uid, ustats);
	if (uid_entry) {
		uid_entry->time_in_state[state] += tpu_usage->duration_us;
		mutex_unlock(&ustats->usage_stats_lock);
		return 0;
	}

	/* Allocate memory for this uid */
	uid_entry = kzalloc(sizeof(*uid_entry), GFP_KERNEL);
	if (!uid_entry) {
		mutex_unlock(&ustats->usage_stats_lock);
		return -ENOMEM;
	}

	uid_entry->uid = tpu_usage->uid;
	uid_entry->time_in_state[state] += tpu_usage->duration_us;

	/* Add uid_entry to the uid_hash_table */
	hash_add(ustats->uid_hash_table, &uid_entry->node, tpu_usage->uid);

	mutex_unlock(&ustats->usage_stats_lock);

	return 0;
}

static void edgetpu_utilization_update(
	struct edgetpu_dev *etdev,
	struct edgetpu_component_activity *activity)
{
	struct edgetpu_usage_stats *ustats = etdev->usage_stats;

	if (!ustats)
		return;

	etdev_dbg(etdev, "%s: comp=%d utilized %d%%\n", __func__,
		  activity->component, activity->utilization);

	mutex_lock(&ustats->usage_stats_lock);
	if (activity->utilization && activity->component >= 0 &&
	    activity->component < EDGETPU_USAGE_COMPONENT_COUNT)
		ustats->component_utilization[activity->component] =
			activity->utilization;
	mutex_unlock(&ustats->usage_stats_lock);
}

static void edgetpu_counter_update(struct edgetpu_dev *etdev, struct edgetpu_usage_counter *counter,
				   uint version)
{
	struct edgetpu_usage_stats *ustats = etdev->usage_stats;
	uint component = version > 1 ? counter->component_id : 0;

	if (!ustats)
		return;

	etdev_dbg(etdev, "%s: type=%d value=%llu comp=%u\n", __func__, counter->type,
		  counter->value, component);

	mutex_lock(&ustats->usage_stats_lock);
	if (counter->type >= 0 && counter->type < EDGETPU_COUNTER_COUNT)
		ustats->counter[counter->type][component] += counter->value;
	mutex_unlock(&ustats->usage_stats_lock);
}

static void edgetpu_counter_clear(struct edgetpu_dev *etdev,
				  enum edgetpu_usage_counter_type counter_type)
{
	struct edgetpu_usage_stats *ustats = etdev->usage_stats;
	int i;

	if (counter_type >= EDGETPU_COUNTER_COUNT)
		return;

	mutex_lock(&ustats->usage_stats_lock);
	for (i = 0; i < EDGETPU_TPU_CLUSTER_COUNT; i++)
		ustats->counter[counter_type][i] = 0;
	mutex_unlock(&ustats->usage_stats_lock);
}

static void edgetpu_max_watermark_update(struct edgetpu_dev *etdev,
					 struct edgetpu_usage_max_watermark *max_watermark,
					 uint version)
{
	struct edgetpu_usage_stats *ustats = etdev->usage_stats;
	uint component = version > 1 ? max_watermark->component_id : 0;

	if (!ustats)
		return;

	etdev_dbg(etdev, "%s: type=%d value=%llu comp=%u\n", __func__, max_watermark->type,
		  max_watermark->value, component);

	if (max_watermark->type < 0 ||
	    max_watermark->type >= EDGETPU_MAX_WATERMARK_TYPE_COUNT)
		return;

	mutex_lock(&ustats->usage_stats_lock);
	if (max_watermark->value > ustats->max_watermark[max_watermark->type][component])
		ustats->max_watermark[max_watermark->type][component] =
			max_watermark->value;
	mutex_unlock(&ustats->usage_stats_lock);
}

static void edgetpu_max_watermark_clear(struct edgetpu_dev *etdev,
					enum edgetpu_usage_max_watermark_type max_watermark_type)
{
	struct edgetpu_usage_stats *ustats = etdev->usage_stats;
	int i;

	if (max_watermark_type < 0 || max_watermark_type >= EDGETPU_MAX_WATERMARK_TYPE_COUNT)
		return;

	mutex_lock(&ustats->usage_stats_lock);
	for (i = 0; i < EDGETPU_TPU_CLUSTER_COUNT; i++)
		ustats->max_watermark[max_watermark_type][i] = 0;
	mutex_unlock(&ustats->usage_stats_lock);
}

static void edgetpu_thread_stats_update(
	struct edgetpu_dev *etdev,
	struct edgetpu_thread_stats *thread_stats)
{
	struct edgetpu_usage_stats *ustats = etdev->usage_stats;

	if (!ustats)
		return;

	etdev_dbg(etdev, "%s: id=%d stackmax=%u\n", __func__,
		  thread_stats->thread_id, thread_stats->max_stack_usage_bytes);

	if (thread_stats->thread_id < 0 ||
	    thread_stats->thread_id >= EDGETPU_FW_THREAD_COUNT)
		return;

	mutex_lock(&ustats->usage_stats_lock);
	if (thread_stats->max_stack_usage_bytes >
	    ustats->thread_stack_max[thread_stats->thread_id])
		ustats->thread_stack_max[thread_stats->thread_id] =
			thread_stats->max_stack_usage_bytes;
	mutex_unlock(&ustats->usage_stats_lock);
}

/* Record new supported frequencies if reported by firmware */
static void edgetpu_dvfs_frequency_update(struct edgetpu_dev *etdev, uint32_t frequency)
{
	uint32_t *freq_table, i;

	mutex_lock(&etdev->freq_lock);
	if (!etdev->freq_table) {
		freq_table = kvmalloc(EDGETPU_MAX_STATES * sizeof(uint32_t), GFP_KERNEL);
		if (!freq_table) {
			etdev_warn(etdev, "Unable to create supported frequencies table");
			goto out;
		}
		etdev->freq_count = 0;
		etdev->freq_table = freq_table;
	}

	freq_table = etdev->freq_table;

	for (i = 0; i < etdev->freq_count; i++) {
		if (freq_table[i] == frequency)
			goto out;
	}

	if (etdev->freq_count >= EDGETPU_MAX_STATES) {
		etdev_warn(etdev, "Unable to record supported frequencies");
		goto out;
	}

	freq_table[etdev->freq_count++] = frequency;
out:
	mutex_unlock(&etdev->freq_lock);
}

void edgetpu_usage_stats_process_buffer(struct edgetpu_dev *etdev, void *buf)
{
	struct edgetpu_usage_stats *ustats = etdev->usage_stats;
	struct edgetpu_usage_metric *metric;
	uint metric_size;
	uint num_metrics;
	uint version;
	int i;

	if (!ustats)
		return;

	/* TODO(b/271372136): remove v1 when v1 firmware no longer in use. */
	if (ustats->use_metrics_v1) {
		struct edgetpu_usage_header_v1 *header = buf;

		metric_size = header->metric_size;
		num_metrics = header->num_metrics;
		version = 1;
		metric = (struct edgetpu_usage_metric *)(header + 1);
	} else {
		struct edgetpu_usage_header *header = buf;

		metric_size = header->metric_size;
		num_metrics = header->num_metrics;
		version = header->version;
		metric = (struct edgetpu_usage_metric *)((char *)header + header->header_bytes);
	}

	etdev_dbg(etdev, "%s: v=%u n=%u sz=%u", __func__, version, num_metrics, metric_size);
	if (metric_size < EDGETPU_USAGE_METRIC_SIZE_V1) {
		etdev_warn_once(etdev, "fw metric size %u less than minimum %u",
				metric_size, EDGETPU_USAGE_METRIC_SIZE_V1);
		return;
	}

	if (metric_size > sizeof(struct edgetpu_usage_metric))
		etdev_dbg(etdev, "fw metrics are later version with unknown fields");

	for (i = 0; i < num_metrics; i++) {
		switch (metric->type) {
		case EDGETPU_METRIC_TYPE_TPU_USAGE:
			edgetpu_usage_add(etdev, &metric->tpu_usage);
			break;
		case EDGETPU_METRIC_TYPE_COMPONENT_ACTIVITY:
			edgetpu_utilization_update(
				etdev, &metric->component_activity);
			break;
		case EDGETPU_METRIC_TYPE_COUNTER:
			edgetpu_counter_update(etdev, &metric->counter, version);
			break;
		case EDGETPU_METRIC_TYPE_MAX_WATERMARK:
			edgetpu_max_watermark_update(etdev, &metric->max_watermark, version);
			break;
		case EDGETPU_METRIC_TYPE_THREAD_STATS:
			edgetpu_thread_stats_update(etdev, &metric->thread_stats);
			break;
		case EDGETPU_METRIC_TYPE_DVFS_FREQUENCY_INFO:
			edgetpu_dvfs_frequency_update(etdev, metric->dvfs_frequency_info);
			break;
		default:
			etdev_dbg(etdev, "%s: %d: skip unknown type=%u",
				  __func__, i, metric->type);
			break;
		}

		metric = (struct edgetpu_usage_metric *)((char *)metric + metric_size);
	}
}

int edgetpu_usage_get_utilization(struct edgetpu_dev *etdev,
				  enum edgetpu_usage_component component)
{
	struct edgetpu_usage_stats *ustats = etdev->usage_stats;
	int32_t val;

	if (component >= EDGETPU_USAGE_COMPONENT_COUNT)
		return -1;
	edgetpu_kci_update_usage(etdev);
	mutex_lock(&ustats->usage_stats_lock);
	val = ustats->component_utilization[component];
	ustats->component_utilization[component] = 0;
	mutex_unlock(&ustats->usage_stats_lock);
	return val;
}

/*
 * Resyncs firmware stats and formats the requested counter in the supplied buffer.
 *
 * If @report_per_cluster is true, and if the firmware implements metrics V2 or higher,
 * then one value is formatted per cluster (for chips with only one cluster only one value is
 * formatted).
 *
 * Returns the number of bytes written to buf.
 */
static ssize_t edgetpu_usage_format_counter(struct edgetpu_dev *etdev, char *buf,
					    enum edgetpu_usage_counter_type counter_type,
					    bool report_per_cluster)
{
	struct edgetpu_usage_stats *ustats = etdev->usage_stats;
	uint ncomponents = report_per_cluster && !etdev->usage_stats->use_metrics_v1 ?
		EDGETPU_TPU_CLUSTER_COUNT : 1;
	uint i;
	ssize_t ret = 0;

	if (counter_type >= EDGETPU_COUNTER_COUNT)
		return 0;
	edgetpu_kci_update_usage(etdev);
	mutex_lock(&ustats->usage_stats_lock);
	for (i = 0; i < ncomponents; i++) {
		if (i)
			ret += scnprintf(buf + ret, PAGE_SIZE - ret, " ");
		ret += scnprintf(buf + ret, PAGE_SIZE - ret, "%llu",
				 ustats->counter[counter_type][i]);
	}
	mutex_unlock(&ustats->usage_stats_lock);
	ret += scnprintf(buf + ret, PAGE_SIZE - ret, "\n");
	return ret;
}

/*
 * Resyncs firmware stats and formats the requested max watermark in the supplied buffer.
 *
 * If @report_per_cluster is true, and if the firmware implements metrics V2 or higher,
 * then one value is formatted per cluster (for chips with only one cluster only one value is
 * formatted).
 *
 * Returns the number of bytes written to buf.
 */
static ssize_t edgetpu_usage_format_max_watermark(
	struct edgetpu_dev *etdev, char *buf,
	enum edgetpu_usage_max_watermark_type max_watermark_type, bool report_per_cluster)
{
	struct edgetpu_usage_stats *ustats = etdev->usage_stats;
	uint ncomponents = report_per_cluster && !etdev->usage_stats->use_metrics_v1 ?
		EDGETPU_TPU_CLUSTER_COUNT : 1;
	uint i;
	ssize_t ret = 0;

	if (max_watermark_type >= EDGETPU_MAX_WATERMARK_TYPE_COUNT)
		return 0;
	edgetpu_kci_update_usage(etdev);
	mutex_lock(&ustats->usage_stats_lock);
	for (i = 0; i < ncomponents; i++) {
		if (i)
			ret += scnprintf(buf + ret, PAGE_SIZE - ret, " ");
		ret += scnprintf(buf + ret, PAGE_SIZE - ret, "%llu",
				 ustats->max_watermark[max_watermark_type][i]);
	}
	mutex_unlock(&ustats->usage_stats_lock);
	ret += scnprintf(buf + ret, PAGE_SIZE - ret, "\n");
	return ret;
}

static ssize_t tpu_usage_show(struct device *dev,
			      struct device_attribute *attr,
			      char *buf)
{
	struct edgetpu_dev *etdev = dev_get_drvdata(dev);
	struct edgetpu_usage_stats *ustats = etdev->usage_stats;
	int i;
	int ret = 0;
	unsigned int bkt;
	struct uid_entry *uid_entry;

	edgetpu_kci_update_usage(etdev);
	/* uid: state0speed state1speed ... */
	ret += scnprintf(buf, PAGE_SIZE, "uid:");

	mutex_lock(&etdev->freq_lock);
	if (!etdev->freq_table) {
		mutex_unlock(&etdev->freq_lock);
		for (i = 0; i < EDGETPU_NUM_STATES; i++)
			ret += scnprintf(buf + ret, PAGE_SIZE - ret, " %d",
					 edgetpu_states_display[i]);
	} else {
		for (i = 0; i < etdev->freq_count; i++)
			ret += scnprintf(buf + ret, PAGE_SIZE - ret, " %d",
					 etdev->freq_table[i]);
		mutex_unlock(&etdev->freq_lock);
	}

	ret += scnprintf(buf + ret, PAGE_SIZE - ret, "\n");

	mutex_lock(&ustats->usage_stats_lock);

	hash_for_each(ustats->uid_hash_table, bkt, uid_entry, node) {
		ret += scnprintf(buf + ret, PAGE_SIZE - ret, "%d:",
				 uid_entry->uid);

		for (i = 0; i < EDGETPU_NUM_STATES; i++)
			ret += scnprintf(buf + ret, PAGE_SIZE - ret, " %lld",
					 uid_entry->time_in_state[i]);

		ret += scnprintf(buf + ret, PAGE_SIZE - ret, "\n");
	}

	mutex_unlock(&ustats->usage_stats_lock);

	return ret;
}

static void usage_stats_remove_uids(struct edgetpu_usage_stats *ustats)
{
	unsigned int bkt;
	struct uid_entry *uid_entry;
	struct hlist_node *tmp;

	mutex_lock(&ustats->usage_stats_lock);

	hash_for_each_safe(ustats->uid_hash_table, bkt, tmp, uid_entry, node) {
		hash_del(&uid_entry->node);
		kfree(uid_entry);
	}

	mutex_unlock(&ustats->usage_stats_lock);
}

/* Write to clear all entries in uid_hash_table */
static ssize_t tpu_usage_clear(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf,
			       size_t count)
{
	struct edgetpu_dev *etdev = dev_get_drvdata(dev);
	struct edgetpu_usage_stats *ustats = etdev->usage_stats;

	usage_stats_remove_uids(ustats);

	return count;
}

static DEVICE_ATTR(tpu_usage, 0664, tpu_usage_show, tpu_usage_clear);

static ssize_t device_utilization_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct edgetpu_dev *etdev = dev_get_drvdata(dev);
	int32_t val;

	val = edgetpu_usage_get_utilization(
			etdev, EDGETPU_USAGE_COMPONENT_DEVICE);
	return scnprintf(buf, PAGE_SIZE, "%d\n", val);
}
static DEVICE_ATTR_RO(device_utilization);

static ssize_t tpu_utilization_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct edgetpu_dev *etdev = dev_get_drvdata(dev);
	int32_t val;

	val = edgetpu_usage_get_utilization(
			etdev, EDGETPU_USAGE_COMPONENT_TPU);
	return scnprintf(buf, PAGE_SIZE, "%d\n", val);
}
static DEVICE_ATTR_RO(tpu_utilization);

static ssize_t tpu_active_cycle_count_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct edgetpu_dev *etdev = dev_get_drvdata(dev);

	return edgetpu_usage_format_counter(etdev, buf, EDGETPU_COUNTER_TPU_ACTIVE_CYCLES, false);
}

static ssize_t tpu_active_cycle_count_store(struct device *dev,
					    struct device_attribute *attr,
					    const char *buf,
					    size_t count)
{
	struct edgetpu_dev *etdev = dev_get_drvdata(dev);

	edgetpu_counter_clear(etdev, EDGETPU_COUNTER_TPU_ACTIVE_CYCLES);
	return count;
}
static DEVICE_ATTR(tpu_active_cycle_count, 0664, tpu_active_cycle_count_show,
		   tpu_active_cycle_count_store);

static ssize_t tpu_throttle_stall_count_show(struct device *dev,
					     struct device_attribute *attr,
					     char *buf)
{
	struct edgetpu_dev *etdev = dev_get_drvdata(dev);

	return edgetpu_usage_format_counter(etdev, buf, EDGETPU_COUNTER_TPU_THROTTLE_STALLS, false);
}

static ssize_t tpu_throttle_stall_count_store(struct device *dev,
					      struct device_attribute *attr,
					      const char *buf,
					      size_t count)
{
	struct edgetpu_dev *etdev = dev_get_drvdata(dev);

	edgetpu_counter_clear(etdev, EDGETPU_COUNTER_TPU_THROTTLE_STALLS);
	return count;
}
static DEVICE_ATTR(tpu_throttle_stall_count, 0664,
		   tpu_throttle_stall_count_show,
		   tpu_throttle_stall_count_store);

static ssize_t inference_count_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct edgetpu_dev *etdev = dev_get_drvdata(dev);

	return edgetpu_usage_format_counter(etdev, buf, EDGETPU_COUNTER_INFERENCES, true);
}

static ssize_t inference_count_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf,
				     size_t count)
{
	struct edgetpu_dev *etdev = dev_get_drvdata(dev);

	edgetpu_counter_clear(etdev, EDGETPU_COUNTER_INFERENCES);
	return count;
}
static DEVICE_ATTR(inference_count, 0664, inference_count_show,
		   inference_count_store);

static ssize_t tpu_op_count_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct edgetpu_dev *etdev = dev_get_drvdata(dev);

	return edgetpu_usage_format_counter(etdev, buf, EDGETPU_COUNTER_TPU_OPS, true);
}

static ssize_t tpu_op_count_store(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct edgetpu_dev *etdev = dev_get_drvdata(dev);

	edgetpu_counter_clear(etdev, EDGETPU_COUNTER_TPU_OPS);
	return count;
}
static DEVICE_ATTR(tpu_op_count, 0664, tpu_op_count_show, tpu_op_count_store);

static ssize_t param_cache_hit_count_show(struct device *dev, struct device_attribute *attr,
					  char *buf)
{
	struct edgetpu_dev *etdev = dev_get_drvdata(dev);

	return edgetpu_usage_format_counter(etdev, buf, EDGETPU_COUNTER_PARAM_CACHE_HITS, false);
}

static ssize_t param_cache_hit_count_store(struct device *dev, struct device_attribute *attr,
					   const char *buf, size_t count)
{
	struct edgetpu_dev *etdev = dev_get_drvdata(dev);

	edgetpu_counter_clear(etdev, EDGETPU_COUNTER_PARAM_CACHE_HITS);
	return count;
}
static DEVICE_ATTR(param_cache_hit_count, 0664, param_cache_hit_count_show,
		   param_cache_hit_count_store);

static ssize_t param_cache_miss_count_show(struct device *dev, struct device_attribute *attr,
					   char *buf)
{
	struct edgetpu_dev *etdev = dev_get_drvdata(dev);

	return edgetpu_usage_format_counter(etdev, buf, EDGETPU_COUNTER_PARAM_CACHE_MISSES, false);
}

static ssize_t param_cache_miss_count_store(struct device *dev, struct device_attribute *attr,
					    const char *buf, size_t count)
{
	struct edgetpu_dev *etdev = dev_get_drvdata(dev);

	edgetpu_counter_clear(etdev, EDGETPU_COUNTER_PARAM_CACHE_MISSES);
	return count;
}
static DEVICE_ATTR(param_cache_miss_count, 0664, param_cache_miss_count_show,
		   param_cache_miss_count_store);

static ssize_t context_preempt_count_show(struct device *dev, struct device_attribute *attr,
					  char *buf)
{
	struct edgetpu_dev *etdev = dev_get_drvdata(dev);

	return edgetpu_usage_format_counter(etdev, buf, EDGETPU_COUNTER_CONTEXT_PREEMPTS, true);
}

static ssize_t context_preempt_count_store(struct device *dev, struct device_attribute *attr,
					   const char *buf, size_t count)
{
	struct edgetpu_dev *etdev = dev_get_drvdata(dev);

	edgetpu_counter_clear(etdev, EDGETPU_COUNTER_CONTEXT_PREEMPTS);
	return count;
}
static DEVICE_ATTR(context_preempt_count, 0664, context_preempt_count_show,
		   context_preempt_count_store);

static ssize_t hardware_preempt_count_show(struct device *dev, struct device_attribute *attr,
					   char *buf)
{
	struct edgetpu_dev *etdev = dev_get_drvdata(dev);

	return edgetpu_usage_format_counter(etdev, buf, EDGETPU_COUNTER_HARDWARE_PREEMPTS, true);
}

static ssize_t hardware_preempt_count_store(struct device *dev, struct device_attribute *attr,
					    const char *buf, size_t count)
{
	struct edgetpu_dev *etdev = dev_get_drvdata(dev);

	edgetpu_counter_clear(etdev, EDGETPU_COUNTER_HARDWARE_PREEMPTS);
	return count;
}
static DEVICE_ATTR(hardware_preempt_count, 0664, hardware_preempt_count_show,
		   hardware_preempt_count_store);

static ssize_t hardware_ctx_save_time_show(struct device *dev, struct device_attribute *attr,
					   char *buf)
{
	struct edgetpu_dev *etdev = dev_get_drvdata(dev);

	return edgetpu_usage_format_counter(etdev, buf, EDGETPU_COUNTER_HARDWARE_CTX_SAVE_TIME_US,
					    true);
}

static ssize_t hardware_ctx_save_time_store(struct device *dev, struct device_attribute *attr,
					    const char *buf, size_t count)
{
	struct edgetpu_dev *etdev = dev_get_drvdata(dev);

	edgetpu_counter_clear(etdev, EDGETPU_COUNTER_HARDWARE_CTX_SAVE_TIME_US);
	return count;
}
static DEVICE_ATTR(hardware_ctx_save_time, 0664, hardware_ctx_save_time_show,
		   hardware_ctx_save_time_store);

static ssize_t scalar_fence_wait_time_show(struct device *dev, struct device_attribute *attr,
					   char *buf)
{
	struct edgetpu_dev *etdev = dev_get_drvdata(dev);

	return edgetpu_usage_format_counter(etdev, buf, EDGETPU_COUNTER_SCALAR_FENCE_WAIT_TIME_US,
					    true);
}

static ssize_t scalar_fence_wait_time_store(struct device *dev, struct device_attribute *attr,
					    const char *buf, size_t count)
{
	struct edgetpu_dev *etdev = dev_get_drvdata(dev);

	edgetpu_counter_clear(etdev, EDGETPU_COUNTER_SCALAR_FENCE_WAIT_TIME_US);
	return count;
}
static DEVICE_ATTR(scalar_fence_wait_time, 0664, scalar_fence_wait_time_show,
		   scalar_fence_wait_time_store);

static ssize_t long_suspend_count_show(struct device *dev, struct device_attribute *attr,
				       char *buf)
{
	struct edgetpu_dev *etdev = dev_get_drvdata(dev);

	return edgetpu_usage_format_counter(etdev, buf, EDGETPU_COUNTER_LONG_SUSPEND, false);
}

static ssize_t long_suspend_count_store(struct device *dev, struct device_attribute *attr,
					    const char *buf, size_t count)
{
	struct edgetpu_dev *etdev = dev_get_drvdata(dev);

	edgetpu_counter_clear(etdev, EDGETPU_COUNTER_LONG_SUSPEND);
	return count;
}
static DEVICE_ATTR(long_suspend_count, 0664, long_suspend_count_show,
		   long_suspend_count_store);

#if EDGETPU_TPU_CLUSTER_COUNT > 1
static ssize_t reconfigurations_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct edgetpu_dev *etdev = dev_get_drvdata(dev);

	return edgetpu_usage_format_counter(etdev, buf, EDGETPU_COUNTER_RECONFIGURATIONS, false);
}

static ssize_t reconfigurations_store(struct device *dev, struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct edgetpu_dev *etdev = dev_get_drvdata(dev);

	edgetpu_counter_clear(etdev, EDGETPU_COUNTER_RECONFIGURATIONS);
	return count;
}
static DEVICE_ATTR(reconfigurations, 0664, reconfigurations_show, reconfigurations_store);

static ssize_t preempt_reconfigurations_show(struct device *dev, struct device_attribute *attr,
					     char *buf)
{
	struct edgetpu_dev *etdev = dev_get_drvdata(dev);

	return edgetpu_usage_format_counter(etdev, buf, EDGETPU_COUNTER_PREEMPT_RECONFIGURATIONS,
					    false);
}

static ssize_t preempt_reconfigurations_store(struct device *dev, struct device_attribute *attr,
					      const char *buf, size_t count)
{
	struct edgetpu_dev *etdev = dev_get_drvdata(dev);

	edgetpu_counter_clear(etdev, EDGETPU_COUNTER_PREEMPT_RECONFIGURATIONS);
	return count;
}
static DEVICE_ATTR(preempt_reconfigurations, 0664, preempt_reconfigurations_show,
		   preempt_reconfigurations_store);
#endif /* EDGETPU_TPU_CLUSTER_COUNT > 1 */


static ssize_t outstanding_commands_max_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct edgetpu_dev *etdev = dev_get_drvdata(dev);

	return edgetpu_usage_format_max_watermark(etdev, buf, EDGETPU_MAX_WATERMARK_OUT_CMDS,
						  false);
}

static ssize_t outstanding_commands_max_store(
	struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct edgetpu_dev *etdev = dev_get_drvdata(dev);

	edgetpu_max_watermark_clear(etdev, EDGETPU_MAX_WATERMARK_OUT_CMDS);
	return count;
}
static DEVICE_ATTR(outstanding_commands_max, 0664,
		   outstanding_commands_max_show,
		   outstanding_commands_max_store);

static ssize_t preempt_depth_max_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct edgetpu_dev *etdev = dev_get_drvdata(dev);

	return edgetpu_usage_format_max_watermark(etdev, buf, EDGETPU_MAX_WATERMARK_PREEMPT_DEPTH,
						  true);
}

static ssize_t preempt_depth_max_store(
	struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct edgetpu_dev *etdev = dev_get_drvdata(dev);

	edgetpu_max_watermark_clear(etdev, EDGETPU_MAX_WATERMARK_PREEMPT_DEPTH);
	return count;
}
static DEVICE_ATTR(preempt_depth_max, 0664, preempt_depth_max_show,
		   preempt_depth_max_store);

static ssize_t hardware_ctx_save_time_max_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct edgetpu_dev *etdev = dev_get_drvdata(dev);

	return edgetpu_usage_format_max_watermark(etdev, buf,
						  EDGETPU_MAX_WATERMARK_HARDWARE_CTX_SAVE_TIME_US,
						  true);
}

static ssize_t hardware_ctx_save_time_max_store(
	struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct edgetpu_dev *etdev = dev_get_drvdata(dev);

	edgetpu_max_watermark_clear(etdev, EDGETPU_MAX_WATERMARK_HARDWARE_CTX_SAVE_TIME_US);
	return count;
}
static DEVICE_ATTR(hardware_ctx_save_time_max, 0664, hardware_ctx_save_time_max_show,
		   hardware_ctx_save_time_max_store);

static ssize_t scalar_fence_wait_time_max_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct edgetpu_dev *etdev = dev_get_drvdata(dev);

	return edgetpu_usage_format_max_watermark(
			etdev, buf, EDGETPU_MAX_WATERMARK_SCALAR_FENCE_WAIT_TIME_US, true);
}

static ssize_t scalar_fence_wait_time_max_store(
	struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct edgetpu_dev *etdev = dev_get_drvdata(dev);

	edgetpu_max_watermark_clear(etdev, EDGETPU_MAX_WATERMARK_SCALAR_FENCE_WAIT_TIME_US);
	return count;
}
static DEVICE_ATTR(scalar_fence_wait_time_max, 0664, scalar_fence_wait_time_max_show,
		   scalar_fence_wait_time_max_store);

static ssize_t suspend_time_max_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct edgetpu_dev *etdev = dev_get_drvdata(dev);

	return edgetpu_usage_format_max_watermark(etdev, buf, EDGETPU_MAX_WATERMARK_SUSPEND_TIME_US,
						  false);
}

static ssize_t suspend_time_max_store(
	struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct edgetpu_dev *etdev = dev_get_drvdata(dev);

	edgetpu_max_watermark_clear(etdev, EDGETPU_MAX_WATERMARK_SUSPEND_TIME_US);
	return count;
}
static DEVICE_ATTR(suspend_time_max, 0664, suspend_time_max_show,
		   suspend_time_max_store);

static ssize_t fw_thread_stats_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct edgetpu_dev *etdev = dev_get_drvdata(dev);
	struct edgetpu_usage_stats *ustats = etdev->usage_stats;
	int i;
	ssize_t ret = 0;

	edgetpu_kci_update_usage(etdev);
	mutex_lock(&ustats->usage_stats_lock);

	for (i = 0; i < EDGETPU_FW_THREAD_COUNT; i++) {
		if (!ustats->thread_stack_max[i])
			continue;
		ret += scnprintf(buf + ret, PAGE_SIZE - ret,
				 "%u\t%u\n", i, ustats->thread_stack_max[i]);
		/* Not checking ret < PAGE_SIZE is intended. */
	}

	mutex_unlock(&ustats->usage_stats_lock);
	return ret;
}

static ssize_t fw_thread_stats_store(
	struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct edgetpu_dev *etdev = dev_get_drvdata(dev);
	struct edgetpu_usage_stats *ustats = etdev->usage_stats;
	int i;

	mutex_lock(&ustats->usage_stats_lock);
	for (i = 0; i < EDGETPU_FW_THREAD_COUNT; i++)
		ustats->thread_stack_max[i] = 0;
	mutex_unlock(&ustats->usage_stats_lock);
	return count;
}
static DEVICE_ATTR(fw_thread_stats, 0664, fw_thread_stats_show,
		   fw_thread_stats_store);

static struct attribute *usage_stats_dev_attrs[] = {
	&dev_attr_tpu_usage.attr,
	&dev_attr_device_utilization.attr,
	&dev_attr_tpu_utilization.attr,
	&dev_attr_tpu_active_cycle_count.attr,
	&dev_attr_tpu_throttle_stall_count.attr,
	&dev_attr_inference_count.attr,
	&dev_attr_tpu_op_count.attr,
	&dev_attr_param_cache_hit_count.attr,
	&dev_attr_param_cache_miss_count.attr,
	&dev_attr_context_preempt_count.attr,
	&dev_attr_hardware_preempt_count.attr,
	&dev_attr_hardware_ctx_save_time.attr,
	&dev_attr_scalar_fence_wait_time.attr,
	&dev_attr_long_suspend_count.attr,
#if EDGETPU_TPU_CLUSTER_COUNT > 1
	&dev_attr_reconfigurations.attr,
	&dev_attr_preempt_reconfigurations.attr,
#endif
	&dev_attr_outstanding_commands_max.attr,
	&dev_attr_preempt_depth_max.attr,
	&dev_attr_hardware_ctx_save_time_max.attr,
	&dev_attr_scalar_fence_wait_time_max.attr,
	&dev_attr_suspend_time_max.attr,
	&dev_attr_fw_thread_stats.attr,
	NULL,
};

static const struct attribute_group usage_stats_attr_group = {
	.attrs = usage_stats_dev_attrs,
};

void edgetpu_usage_stats_init(struct edgetpu_dev *etdev)
{
	struct edgetpu_usage_stats *ustats;
	int ret;

	ustats = devm_kzalloc(etdev->dev, sizeof(*etdev->usage_stats),
			      GFP_KERNEL);
	if (!ustats)
		return;

	hash_init(ustats->uid_hash_table);
	mutex_init(&ustats->usage_stats_lock);
	etdev->usage_stats = ustats;

	ret = device_add_group(etdev->dev, &usage_stats_attr_group);
	if (ret)
		etdev_warn(etdev, "failed to create the usage_stats attrs\n");

	etdev_dbg(etdev, "%s init\n", __func__);
}

void edgetpu_usage_stats_exit(struct edgetpu_dev *etdev)
{
	struct edgetpu_usage_stats *ustats = etdev->usage_stats;

	if (ustats) {
		usage_stats_remove_uids(ustats);
		device_remove_group(etdev->dev, &usage_stats_attr_group);
		/* free the frequency table if allocated */
		mutex_lock(&etdev->freq_lock);
		if (etdev->freq_table)
			kvfree(etdev->freq_table);
		etdev->freq_table = NULL;
		etdev->freq_count = 0;
		mutex_unlock(&etdev->freq_lock);
	}

	etdev_dbg(etdev, "%s exit\n", __func__);
}
