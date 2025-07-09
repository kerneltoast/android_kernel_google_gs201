// SPDX-License-Identifier: GPL-2.0-only
/*
 * Interface of managing the usage stats of IPs.
 *
 * Copyright (C) 2023 Google LLC
 */

#include <linux/device.h>
#include <linux/hashtable.h>
#include <linux/kernel.h>
#include <linux/lockdep.h>
#include <linux/mutex.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/types.h>

#include <gcip/gcip-usage-stats.h>

typedef ssize_t (*show_t)(struct device *dev, struct device_attribute *dev_attr, char *buf);
typedef ssize_t (*store_t)(struct device *dev, struct device_attribute *dev_attr, const char *buf,
			   size_t count);

/*
 * Show callback which simply redirects to the user defined one.
 * If GCIP doesn't have its own implementation because we expect that it won't be used in the real
 * cases or the user has their own implementation for the specific metric, this callback will
 * be used for the show function of the device attribute.
 */
static ssize_t gcip_usage_stats_user_defined_show(struct device *dev,
						  struct device_attribute *dev_attr, char *buf)
{
	struct gcip_usage_stats_attr *attr =
		container_of(dev_attr, struct gcip_usage_stats_attr, dev_attr);
	ssize_t ret = 0;

	if (attr->show)
		ret = attr->show(dev, attr, buf, attr->ustats->data);

	return ret;
}

/*
 * Store callback which simply redirects to the user defined one.
 * If GCIP doesn't have its own implementation because we expect that it won't be used in the real
 * cases or the user has their own implementation for the specific metric, this callback will
 * be used for the store function of the device attribute.
 */
static ssize_t gcip_usage_stats_user_defined_store(struct device *dev,
						   struct device_attribute *dev_attr,
						   const char *buf, size_t count)
{
	struct gcip_usage_stats_attr *attr =
		container_of(dev_attr, struct gcip_usage_stats_attr, dev_attr);
	ssize_t ret = 0;

	if (attr->store)
		ret = attr->store(dev, attr, buf, count, attr->ustats->data);

	return ret;
}

/* Following functions are related to `CORE_USAGE` metrics. */

/*
 * Returns the core usage entry of @uid and @core_id from @ustats->core_usage_htable hash table.
 * Caller must hold @ustats->usage_stats_lock.
 */
static struct gcip_usage_stats_core_usage_uid_entry *
gcip_usage_stats_find_core_usage_entry_locked(int32_t uid, uint8_t core_id,
					      struct gcip_usage_stats *ustats)
{
	struct gcip_usage_stats_core_usage_uid_entry *uid_entry;

	lockdep_assert_held(&ustats->usage_stats_lock);

	hash_for_each_possible (ustats->core_usage_htable[core_id], uid_entry, node, uid) {
		if (uid_entry->uid == uid)
			return uid_entry;
	}

	return NULL;
}

/* Returns the 0-based index of @dvfs_freq from the frequency table. */
static unsigned int gcip_usage_stats_find_dvfs_freq_index(struct gcip_usage_stats *ustats,
							  uint32_t dvfs_freq)
{
	int i, nums, closest_freq_idx, idx = 0;
	uint32_t cur_freq, closest_freq = 0;

	mutex_lock(&ustats->dvfs_freqs_lock);

	/*
	 * Uses the frequency table, @ustats->dvfs_freqs, if the firmware has ever reported
	 * frequencies via `DVFS_FREQUENCY_INFO` metrics.
	 */
	if (ustats->dvfs_freqs_num) {
		for (i = ustats->dvfs_freqs_num - 1; i >= 0; i--) {
			if (dvfs_freq == ustats->dvfs_freqs[i]) {
				idx = i;
				break;
			}
		}

		if (i < 0)
			dev_warn(ustats->dev,
				 "Failed to find the freq among the ones sent from the FW, freq=%u",
				 dvfs_freq);

		mutex_unlock(&ustats->dvfs_freqs_lock);
		return idx;
	}

	mutex_unlock(&ustats->dvfs_freqs_lock);

	/* Uses default one in case of the firmware has never reported supported frequencies. */
	nums = ustats->default_dvfs_freqs_num;
	if (nums <= 0) {
		dev_warn_once(ustats->dev, "The kernel driver doesn't have default DVFS freqs");
		return 0;
	}

	for (i = nums - 1; i >= 0; i--) {
		cur_freq = ustats->default_dvfs_freqs[i];

		if (dvfs_freq == cur_freq)
			return i;

		if (dvfs_freq > cur_freq && closest_freq < cur_freq) {
			closest_freq = cur_freq;
			closest_freq_idx = i;
		}
	}

	if (closest_freq)
		return closest_freq_idx;

	dev_warn(ustats->dev,
		 "Failed to find the freq from the default ones of the kernel driver, freq=%u",
		 dvfs_freq);

	return 0;
}

/*
 * Updates the entry of @uid in the core usage hash table of @core_id.
 * If there is no entry for @uid, it will create one and insert it into the table.
 *
 * Called when the FW sent `CORE_USAGE` metrics.
 */
static void gcip_usage_stats_update_core_usage(struct gcip_usage_stats *ustats,
					       struct gcip_usage_stats_core_usage *new,
					       int fw_metric_version)
{
	struct gcip_usage_stats_core_usage_uid_entry *uid_entry;
	unsigned int state = gcip_usage_stats_find_dvfs_freq_index(ustats, new->operating_point);
	uint8_t core_id = 0;

	if (fw_metric_version >= GCIP_USAGE_STATS_V2)
		core_id = new->core_id;

	if (core_id >= ustats->subcomponents) {
		dev_warn_once(ustats->dev,
			      "FW sent an invalid core_id for the core usage update, core_id=%u",
			      core_id);
		return;
	}

	mutex_lock(&ustats->usage_stats_lock);

	/* Finds the uid from @ustats->core_usage_htable first. */
	uid_entry = gcip_usage_stats_find_core_usage_entry_locked(new->uid, core_id, ustats);
	if (uid_entry) {
		uid_entry->time_in_state[state] += new->control_core_duration;
		mutex_unlock(&ustats->usage_stats_lock);
		return;
	}

	dev_dbg(ustats->dev, "FW sent a new uid for the core usage update, uid=%d, core_id=%u",
		new->uid, core_id);

	/* Allocates an entry for this uid. */
	uid_entry = devm_kzalloc(ustats->dev, sizeof(*uid_entry), GFP_KERNEL);
	if (!uid_entry) {
		dev_err(ustats->dev,
			"Failed to allocate an entry of core usage hash table, uid=%d, core_id=%u",
			new->uid, core_id);
		mutex_unlock(&ustats->usage_stats_lock);
		return;
	}

	uid_entry->uid = new->uid;
	uid_entry->time_in_state[state] += new->control_core_duration;

	/* Adds @uid_entry to the @ustats->core_usage_htable. */
	hash_add(ustats->core_usage_htable[core_id], &uid_entry->node, new->uid);

	mutex_unlock(&ustats->usage_stats_lock);
}

/* Releases all entries in the core usage hash table of @core_id. */
static void gcip_usage_stats_free_core_usage_core_entries_locked(struct gcip_usage_stats *ustats,
								 uint8_t core_id)
{
	unsigned int bkt;
	struct gcip_usage_stats_core_usage_uid_entry *uid_entry;
	struct hlist_node *tmp;

	lockdep_assert_held(&ustats->usage_stats_lock);

	hash_for_each_safe (ustats->core_usage_htable[core_id], bkt, tmp, uid_entry, node) {
		hash_del(&uid_entry->node);
		devm_kfree(ustats->dev, uid_entry);
	}
}

/* Releases all entries of all core usage hash tables. */
static void gcip_usage_stats_free_core_usage_all_entries(struct gcip_usage_stats *ustats)
{
	int i;

	mutex_lock(&ustats->usage_stats_lock);

	for (i = 0; i < ustats->subcomponents; i++)
		gcip_usage_stats_free_core_usage_core_entries_locked(ustats, i);

	mutex_unlock(&ustats->usage_stats_lock);
}

/*
 * Prints the core usage per uid in multiple arrays with the whitespace separation:
 * <uid_0> <core_usage_0_1> <core_usage_0_2> ...
 * <uid_1> <core_usage_1_1> <core_usage_1_2> ...
 * ...
 *
 * Called when the runtime reads the device attribute.
 */
static ssize_t gcip_usage_stats_core_usage_show(struct device *dev,
						struct device_attribute *dev_attr, char *buf)
{
	struct gcip_usage_stats_attr *attr =
		container_of(dev_attr, struct gcip_usage_stats_attr, dev_attr);
	struct gcip_usage_stats *ustats = attr->ustats;
	struct gcip_usage_stats_core_usage_uid_entry *uid_entry;
	int i, dvfs_freqs_num;
	unsigned int bkt;
	ssize_t written = 0;

	ustats->ops->update_usage_kci(ustats->data);

	mutex_lock(&ustats->dvfs_freqs_lock);

	dvfs_freqs_num = ustats->dvfs_freqs_num ? ustats->dvfs_freqs_num :
						  ustats->default_dvfs_freqs_num;

	mutex_unlock(&ustats->dvfs_freqs_lock);
	mutex_lock(&ustats->usage_stats_lock);

	hash_for_each (ustats->core_usage_htable[attr->subcomponent], bkt, uid_entry, node) {
		written += scnprintf(buf + written, PAGE_SIZE - written, "%d", uid_entry->uid);

		for (i = 0; i < dvfs_freqs_num; i++)
			written += scnprintf(buf + written, PAGE_SIZE - written, " %lld",
					     uid_entry->time_in_state[i]);

		written += scnprintf(buf + written, PAGE_SIZE - written, "\n");
	}

	mutex_unlock(&ustats->usage_stats_lock);

	return written;
}

/*
 * Releases all the entries of the core usage hash table of the given core id, @attr->subcomponent.
 *
 * Called when the runtime writes the device attribute.
 */
static ssize_t gcip_usage_stats_core_usage_store(struct device *dev,
						 struct device_attribute *dev_attr, const char *buf,
						 size_t count)
{
	struct gcip_usage_stats_attr *attr =
		container_of(dev_attr, struct gcip_usage_stats_attr, dev_attr);
	struct gcip_usage_stats *ustats = attr->ustats;

	mutex_lock(&ustats->usage_stats_lock);
	gcip_usage_stats_free_core_usage_core_entries_locked(ustats, attr->subcomponent);
	mutex_unlock(&ustats->usage_stats_lock);

	return count;
}

/* Following functions are related to `COMPONENT_UTILIZATION` metrics. */

/*
 * Updates the utilization of components.
 * The value of utilization must be [0, 100].
 *
 * Called when the FW sent `COMPONENT_UTILIZATION` metrics.
 */
static void
gcip_usage_stats_update_component_utilization(struct gcip_usage_stats *ustats,
					      struct gcip_usage_stats_component_utilization *new,
					      uint16_t fw_metric_version)
{
	if (new->component < 0 ||
	    new->component >= GCIP_USAGE_STATS_COMPONENT_UTILIZATION_NUM_TYPES) {
		dev_warn_once(ustats->dev, "FW sent an invalid component utilization type, type=%d",
			      new->component);
		return;
	}

	if (new->utilization < 0 || new->utilization > 100) {
		dev_warn_once(ustats->dev,
			      "FW sent an invalid component utilization value, type=%d, value=%d",
			      new->component, new->utilization);
		return;
	}

	mutex_lock(&ustats->usage_stats_lock);
	ustats->component_utilization[new->component] = new->utilization;
	mutex_unlock(&ustats->usage_stats_lock);
}

/*
 * Prints the utilization of the specific component.
 * Note that this function also resets the utilization to 0. Therefore, we don't have a specific
 * store function implementation for this stats. The `gcip_usage_stats_alloc_attrs` function will
 * register the `gcip_usage_stats_user_defined_store` function as the store function if the kernel
 * driver enables the write permission.
 *
 * Called when the runtime reads the device attribute.
 */
static ssize_t gcip_usage_stats_component_utilization_show(struct device *dev,
							   struct device_attribute *dev_attr,
							   char *buf)
{
	struct gcip_usage_stats_attr *attr =
		container_of(dev_attr, struct gcip_usage_stats_attr, dev_attr);
	struct gcip_usage_stats *ustats = attr->ustats;
	int32_t val;

	ustats->ops->update_usage_kci(ustats->data);

	mutex_lock(&ustats->usage_stats_lock);

	val = ustats->component_utilization[attr->type];
	ustats->component_utilization[attr->type] = 0;

	mutex_unlock(&ustats->usage_stats_lock);

	return scnprintf(buf, PAGE_SIZE, "%d\n", val);
}

/* Following functions are related to `COUNTER` metrics. */

/*
 * Updates the counter which represents monotonically increased occurrences such as workloads
 * and preemptions.
 *
 * Called when the FW sent `COUNTER` metrics.
 */
static void gcip_usage_stats_update_counter(struct gcip_usage_stats *ustats,
					    struct gcip_usage_stats_counter *new,
					    uint16_t fw_metric_version)
{
	uint8_t component_id = 0;

	if (new->type < 0 || new->type >= GCIP_USAGE_STATS_COUNTER_NUM_TYPES) {
		dev_warn_once(ustats->dev, "FW sent an invalid counter type, type=%d", new->type);
		return;
	}

	if (fw_metric_version >= GCIP_USAGE_STATS_V2)
		component_id = new->component_id;

	if (component_id >= ustats->subcomponents) {
		dev_warn_once(
			ustats->dev,
			"FW sent an invalid component_id for the counter update, component_id=%d",
			component_id);
		return;
	}

	mutex_lock(&ustats->usage_stats_lock);
	ustats->counter[component_id][new->type] += new->value;
	mutex_unlock(&ustats->usage_stats_lock);
}

/*
 * Prints the value(s) of counter.
 * If the @attr->subcomponent is `GCIP_USAGE_STATS_ATTR_ALL_SUBCOMPONENTS`, it will print the
 * counters of all subcomponents in one array with whitespace separation. Otherwise, it will print
 * the counter of the specific subcomponent.
 *
 * Called when the runtime reads the device attribute.
 */
static ssize_t gcip_usage_stats_counter_show(struct device *dev, struct device_attribute *dev_attr,
					     char *buf)
{
	struct gcip_usage_stats_attr *attr =
		container_of(dev_attr, struct gcip_usage_stats_attr, dev_attr);
	struct gcip_usage_stats *ustats = attr->ustats;
	ssize_t written = 0;
	int subcomponent, i;

	ustats->ops->update_usage_kci(ustats->data);

	/*
	 * We need to decide @subcomponent after calling `update_usage_kci` because IP kernel
	 * drivers may want to change the version of @ustats to lower one if the firmware doesn't
	 * support a higher version.
	 */
	subcomponent = ustats->version >= GCIP_USAGE_STATS_V2 ? attr->subcomponent : 0;

	mutex_lock(&ustats->usage_stats_lock);

	if (subcomponent == GCIP_USAGE_STATS_ATTR_ALL_SUBCOMPONENTS) {
		for (i = 0; i < ustats->subcomponents; i++) {
			/* Prints a blank only when @i is bigger than 0. */
			written += scnprintf(buf + written, PAGE_SIZE - written, "%.*s%lld", i, " ",
					     ustats->counter[i][attr->type]);
		}
	} else {
		written += scnprintf(buf + written, PAGE_SIZE - written, "%lld",
				     ustats->counter[subcomponent][attr->type]);
	}

	mutex_unlock(&ustats->usage_stats_lock);
	written += scnprintf(buf + written, PAGE_SIZE - written, "\n");

	return written;
}

/*
 * Clears the value(s) of counter.
 * As described in the show function, it will clears the counters of all subcomponents when
 * @attr->subcomponent is `GCIP_USAGE_STATS_ATTR_ALL_SUBCOMPONENTS`. Otherwise, it will clear the
 * counter of the specific subcomponent.
 *
 * Called when the runtime writes the device attribute.
 */
static ssize_t gcip_usage_stats_counter_store(struct device *dev, struct device_attribute *dev_attr,
					      const char *buf, size_t count)
{
	struct gcip_usage_stats_attr *attr =
		container_of(dev_attr, struct gcip_usage_stats_attr, dev_attr);
	struct gcip_usage_stats *ustats = attr->ustats;
	int subcomponent = ustats->version >= GCIP_USAGE_STATS_V2 ? attr->subcomponent : 0;
	int i;

	mutex_lock(&ustats->usage_stats_lock);

	if (subcomponent == GCIP_USAGE_STATS_ATTR_ALL_SUBCOMPONENTS) {
		for (i = 0; i < ustats->subcomponents; i++)
			ustats->counter[i][attr->type] = 0;
	} else {
		ustats->counter[subcomponent][attr->type] = 0;
	}

	mutex_unlock(&ustats->usage_stats_lock);

	return count;
}

/* Following functions are related to `THREAD_STATISTICS` metrics. */

/*
 * Updates the max stack usage of the @new->thread_id type of thread.
 *
 * Called when the FW sent `THREAD_STATISTICS` metrics.
 */
static void gcip_usage_stats_update_thread_stats(struct gcip_usage_stats *ustats,
						 struct gcip_usage_stats_thread_stats *new,
						 uint16_t fw_metric_version)
{
	if (new->thread_id < 0 || new->thread_id >= GCIP_USAGE_STATS_THREAD_NUM_TYPES) {
		dev_warn_once(ustats->dev, "FW sent an invalid thread_id, thread_id=%d",
			      new->thread_id);
		return;
	}

	mutex_lock(&ustats->usage_stats_lock);

	if (new->max_stack_usage_bytes > ustats->thread_max_stack_usage[new->thread_id])
		ustats->thread_max_stack_usage[new->thread_id] = new->max_stack_usage_bytes;

	mutex_unlock(&ustats->usage_stats_lock);
}

/*
 * Prints the max stack usage of each thread with tab separation.
 *
 * Called when the runtime reads the device attribute.
 */
static ssize_t gcip_usage_stats_thread_stats_show(struct device *dev,
						  struct device_attribute *dev_attr, char *buf)
{
	struct gcip_usage_stats_attr *attr =
		container_of(dev_attr, struct gcip_usage_stats_attr, dev_attr);
	struct gcip_usage_stats *ustats = attr->ustats;
	int i;
	ssize_t written = 0;

	ustats->ops->update_usage_kci(ustats->data);

	mutex_lock(&ustats->usage_stats_lock);

	for (i = 0; i < GCIP_USAGE_STATS_THREAD_NUM_TYPES; i++) {
		if (!ustats->thread_max_stack_usage[i])
			continue;
		written += scnprintf(buf + written, PAGE_SIZE - written, "%u\t%u\n", i,
				     ustats->thread_max_stack_usage[i]);
	}

	mutex_unlock(&ustats->usage_stats_lock);

	return written;
}

/*
 * Clears the max usage of all threads to 0.
 *
 * Called when the runtime writes the device attribute.
 */
static ssize_t gcip_usage_stats_thread_stats_store(struct device *dev,
						   struct device_attribute *dev_attr,
						   const char *buf, size_t count)
{
	struct gcip_usage_stats_attr *attr =
		container_of(dev_attr, struct gcip_usage_stats_attr, dev_attr);
	struct gcip_usage_stats *ustats = attr->ustats;

	mutex_lock(&ustats->usage_stats_lock);
	memset(ustats->thread_max_stack_usage, 0, sizeof(ustats->thread_max_stack_usage));
	mutex_unlock(&ustats->usage_stats_lock);
	return count;
}

/* Following functions are related to `MAX_WATERMARK` metrics. */

/*
 * Updates the @new->type type of max watermark of @new->component_id of component.
 *
 * Called when the FW sent `MAX_WATERMARK` metrics.
 */
static void gcip_usage_stats_update_max_watermark(struct gcip_usage_stats *ustats,
						  struct gcip_usage_stats_max_watermark *new,
						  uint16_t fw_metric_version)
{
	uint8_t component_id = 0;

	if (new->type < 0 || new->type >= GCIP_USAGE_STATS_MAX_WATERMARK_NUM_TYPES) {
		dev_warn_once(ustats->dev, "FW sent an invalid max watermark type, type=%d",
			      new->type);
		return;
	}

	if (fw_metric_version >= GCIP_USAGE_STATS_V2)
		component_id = new->component_id;

	if (component_id >= ustats->subcomponents) {
		dev_warn_once(
			ustats->dev,
			"FW sent an invalid component_id for the max watermark update, component_id=%d",
			component_id);
		return;
	}

	mutex_lock(&ustats->usage_stats_lock);

	if (new->value > ustats->max_watermark[component_id][new->type])
		ustats->max_watermark[component_id][new->type] = new->value;

	mutex_unlock(&ustats->usage_stats_lock);
}

/*
 * Printe the value(s) of max watermark.
 *
 * If the @attr->subcomponent is `GCIP_USAGE_STATS_ATTR_ALL_SUBCOMPONENTS`, it will print the
 * values of all subcomponents in one array with whitespace separation. Otherwise, it will print
 * the value of the specific subcomponent.
 *
 * Called when the runtime reads the device attribute.
 */
static ssize_t gcip_usage_stats_max_watermark_show(struct device *dev,
						   struct device_attribute *dev_attr, char *buf)
{
	struct gcip_usage_stats_attr *attr =
		container_of(dev_attr, struct gcip_usage_stats_attr, dev_attr);
	struct gcip_usage_stats *ustats = attr->ustats;
	ssize_t written = 0;
	int subcomponent, i;

	ustats->ops->update_usage_kci(ustats->data);

	/*
	 * We need to decide @subcomponent after calling `update_usage_kci` because IP kernel
	 * drivers may want to change the version of @ustats to lower one if the firmware doesn't
	 * support a higher version.
	 */
	subcomponent = ustats->version >= GCIP_USAGE_STATS_V2 ? attr->subcomponent : 0;

	mutex_lock(&ustats->usage_stats_lock);

	if (subcomponent == GCIP_USAGE_STATS_ATTR_ALL_SUBCOMPONENTS) {
		for (i = 0; i < ustats->subcomponents; i++) {
			/* Prints a blank only when @i is bigger than 0. */
			written += scnprintf(buf + written, PAGE_SIZE - written, "%.*s%lld", i, " ",
					     ustats->max_watermark[i][attr->type]);
		}
	} else {
		written += scnprintf(buf + written, PAGE_SIZE - written, "%lld",
				     ustats->max_watermark[subcomponent][attr->type]);
	}

	mutex_unlock(&ustats->usage_stats_lock);
	written += scnprintf(buf + written, PAGE_SIZE - written, "\n");

	return written;
}

/*
 * Clears the value(s) of max watermark.
 * As described in the show function, it will clears the values of all subcomponents when
 * @attr->subcomponent is `GCIP_USAGE_STATS_ATTR_ALL_SUBCOMPONENTS`. Otherwise, it will clear the
 * value of the specific subcomponent.
 *
 * Called when the runtime writes the device attribute.
 */
static ssize_t gcip_usage_stats_max_watermark_store(struct device *dev,
						    struct device_attribute *dev_attr,
						    const char *buf, size_t count)
{
	struct gcip_usage_stats_attr *attr =
		container_of(dev_attr, struct gcip_usage_stats_attr, dev_attr);
	struct gcip_usage_stats *ustats = attr->ustats;
	int subcomponent = ustats->version >= GCIP_USAGE_STATS_V2 ? attr->subcomponent : 0;
	int i;

	mutex_lock(&ustats->usage_stats_lock);

	if (subcomponent == GCIP_USAGE_STATS_ATTR_ALL_SUBCOMPONENTS) {
		for (i = 0; i < ustats->subcomponents; i++)
			ustats->max_watermark[i][attr->type] = 0;
	} else {
		ustats->max_watermark[subcomponent][attr->type] = 0;
	}

	mutex_unlock(&ustats->usage_stats_lock);

	return count;
}

/* Following functions are related to `DVFS_FREQUENCY_INFO` metrics. */

/*
 * Updates the DVFS freq table with the ones sent by the FW.
 *
 * Until the runtime flushes them by writing the device attribute (i.e., the
 * `gcip_usage_stats_dvfs_freqs_store` function is called), they will be used instead of the
 * default freqs of the kernel driver.
 *
 * Called when the FW sent `DVFS_FREQUENCY_INFO` metrics.
 */
static void gcip_usage_stats_update_dvfs_freq_info(struct gcip_usage_stats *ustats,
						   struct gcip_usage_stats_dvfs_frequency_info *new,
						   uint16_t fw_metric_version)
{
	int i;

	mutex_lock(&ustats->dvfs_freqs_lock);

	for (i = 0; i < ustats->dvfs_freqs_num; i++)
		if (ustats->dvfs_freqs[i] == new->supported_frequency)
			goto out;

	if (ustats->dvfs_freqs_num >= GCIP_USAGE_STATS_MAX_DVFS_FREQ_NUM) {
		dev_warn(ustats->dev, "FW sent more DVFS freqs than the kernel driver can handle");
		goto out;
	}

	ustats->dvfs_freqs[ustats->dvfs_freqs_num++] = new->supported_frequency;
out:
	mutex_unlock(&ustats->dvfs_freqs_lock);
}

/* Flushes the DVFS freqs from the FW by simply setting the number of freqs in the table to 0. */
static void gcip_usage_stats_reset_dvfs_freqs(struct gcip_usage_stats *ustats)
{
	mutex_lock(&ustats->dvfs_freqs_lock);
	ustats->dvfs_freqs_num = 0;
	mutex_unlock(&ustats->dvfs_freqs_lock);
}

/*
 * Prints the list of available DVFS freqs in an array with the whitespace separation.
 *
 * If the FW has sent `DVFS_FREQUENCY_INFO` metrics, they will be printed. Otherwise, the default
 * ones from the kernel driver will be printed.
 *
 * Called when the runtime reads the device attribute.
 */
static ssize_t gcip_usage_stats_dvfs_freqs_show(struct device *dev,
						struct device_attribute *dev_attr, char *buf)
{
	struct gcip_usage_stats_attr *attr =
		container_of(dev_attr, struct gcip_usage_stats_attr, dev_attr);
	struct gcip_usage_stats *ustats = attr->ustats;
	int i;
	ssize_t written = 0;

	ustats->ops->update_usage_kci(ustats->data);

	mutex_lock(&ustats->dvfs_freqs_lock);

	if (!ustats->dvfs_freqs_num) {
		mutex_unlock(&ustats->dvfs_freqs_lock);
		for (i = 0; i < ustats->default_dvfs_freqs_num; i++)
			written += scnprintf(buf + written, PAGE_SIZE - written, "%.*s%u", i, " ",
					     ustats->default_dvfs_freqs[i]);
	} else {
		for (i = 0; i < ustats->dvfs_freqs_num; i++)
			written += scnprintf(buf + written, PAGE_SIZE - written, "%.*s%u", i, " ",
					     ustats->dvfs_freqs[i]);
		mutex_unlock(&ustats->dvfs_freqs_lock);
	}

	written += scnprintf(buf + written, PAGE_SIZE - written, "\n");

	return written;
}

/*
 * Flushes the DVFS freqs sent by the FW.
 *
 * Called when the runtime writes the device attribute.
 */
static ssize_t gcip_usage_stats_dvfs_freqs_store(struct device *dev,
						 struct device_attribute *dev_attr, const char *buf,
						 size_t count)
{
	struct gcip_usage_stats_attr *attr =
		container_of(dev_attr, struct gcip_usage_stats_attr, dev_attr);

	gcip_usage_stats_reset_dvfs_freqs(attr->ustats);

	return count;
}

/* Parses header part of @buf. */
static void *gcip_usage_stats_parse_header(struct gcip_usage_stats *ustats, void *buf,
					   uint32_t *num_metrics, uint32_t *metric_size,
					   uint16_t *fw_metric_version)
{
	struct gcip_usage_stats_header *header = buf;
	struct gcip_usage_stats_header_v1 *header_v1 = buf;

	if (ustats->version <= GCIP_USAGE_STATS_V1) {
		*num_metrics = header_v1->num_metrics;
		*metric_size = header_v1->metric_size;
		*fw_metric_version = GCIP_USAGE_STATS_V1;
		buf += sizeof(*header_v1);
	} else {
		*num_metrics = header->num_metrics;
		*metric_size = header->metric_size;
		*fw_metric_version = header->version;
		buf += sizeof(*header);
	}

	return buf;
}

/*
 * Fills out required information of device attribute such as show and store callbacks. Also,
 * checks the validity of it.
 */
static int gcip_usage_stats_fill_attr(struct gcip_usage_stats *ustats,
				      struct gcip_usage_stats_attr *attr, const show_t show,
				      const store_t store)
{
	struct device_attribute *dev_attr = &attr->dev_attr;

	/*
	 * CORE_USAGE metric doesn't support showing stats for all subcomponents in one device
	 * attribute. Because its printing format is complicated.
	 */
	if (attr->metric == GCIP_USAGE_STATS_METRIC_TYPE_CORE_USAGE &&
	    attr->subcomponent == GCIP_USAGE_STATS_ATTR_ALL_SUBCOMPONENTS)
		return -EINVAL;

	/*
	 * For metrics which store stats per subcomponent, we have to check whether the caller set
	 * a proper subcomponent index.
	 */
	if (attr->subcomponent != GCIP_USAGE_STATS_ATTR_ALL_SUBCOMPONENTS &&
	    (attr->subcomponent < 0 || attr->subcomponent >= ustats->subcomponents))
		return -EINVAL;

	switch (attr->metric) {
	case GCIP_USAGE_STATS_METRIC_TYPE_COMPONENT_UTILIZATION:
		if (attr->type >= GCIP_USAGE_STATS_COMPONENT_UTILIZATION_NUM_TYPES)
			return -EINVAL;
		break;
	case GCIP_USAGE_STATS_METRIC_TYPE_COUNTER:
		if (attr->type >= GCIP_USAGE_STATS_COUNTER_NUM_TYPES)
			return -EINVAL;
		break;
	case GCIP_USAGE_STATS_METRIC_TYPE_MAX_WATERMARK:
		if (attr->type >= GCIP_USAGE_STATS_MAX_WATERMARK_NUM_TYPES)
			return -EINVAL;
		break;
	default:
		break;
	}

	/*
	 * If the user defines its own show/store callbacks (@attr->show or @attr->store), use the
	 * functions which simply redirect to the user-defined callbacks.
	 */
	if (attr->mode == GCIP_USAGE_STATS_MODE_RW || attr->mode == GCIP_USAGE_STATS_MODE_RO)
		dev_attr->show = attr->show ? gcip_usage_stats_user_defined_show : show;
	if (attr->mode == GCIP_USAGE_STATS_MODE_RW || attr->mode == GCIP_USAGE_STATS_MODE_WO)
		dev_attr->store = attr->store ? gcip_usage_stats_user_defined_store : store;

	attr->ustats = ustats;
	dev_attr->attr.mode = attr->mode;
	dev_attr->attr.name = attr->name;

	return 0;
}

/*
 * Allocates @ustats->attrs, a pointer array of `struct attribute`. Each pointers will indicate the
 * `struct attribute` instances of the @args->attrs[]->dev_attr.attr.
 * It will fill out show and store callbacks of each device attribute and the allocated
 * @ustats->attr will be set to the @ustats->group.attr to register them.
 */
static int gcip_usage_stats_alloc_attrs(struct gcip_usage_stats *ustats,
					const struct gcip_usage_stats_args *args)
{
	int i, ret;
	struct gcip_usage_stats_attr *attr;

	ustats->attrs =
		devm_kcalloc(ustats->dev, args->num_attrs + 1, sizeof(*ustats->attrs), GFP_KERNEL);
	if (!ustats->attrs)
		return -ENOMEM;

	/* TODO: fill @ustats->attrs according to the metrics. */
	for (i = 0; i < args->num_attrs; i++) {
		attr = args->attrs[i];

		switch (attr->metric) {
		case GCIP_USAGE_STATS_METRIC_TYPE_CORE_USAGE:
			ret = gcip_usage_stats_fill_attr(ustats, attr,
							 gcip_usage_stats_core_usage_show,
							 gcip_usage_stats_core_usage_store);
			break;
		case GCIP_USAGE_STATS_METRIC_TYPE_COMPONENT_UTILIZATION:
			ret = gcip_usage_stats_fill_attr(
				ustats, attr, gcip_usage_stats_component_utilization_show,
				gcip_usage_stats_user_defined_store);
			break;
		case GCIP_USAGE_STATS_METRIC_TYPE_COUNTER:
			ret = gcip_usage_stats_fill_attr(ustats, attr,
							 gcip_usage_stats_counter_show,
							 gcip_usage_stats_counter_store);
			break;
		case GCIP_USAGE_STATS_METRIC_TYPE_THREAD_STATS:
			ret = gcip_usage_stats_fill_attr(ustats, attr,
							 gcip_usage_stats_thread_stats_show,
							 gcip_usage_stats_thread_stats_store);
			break;
		case GCIP_USAGE_STATS_METRIC_TYPE_MAX_WATERMARK:
			ret = gcip_usage_stats_fill_attr(ustats, attr,
							 gcip_usage_stats_max_watermark_show,
							 gcip_usage_stats_max_watermark_store);
			break;
		case GCIP_USAGE_STATS_METRIC_TYPE_DVFS_FREQUENCY_INFO:
			ret = gcip_usage_stats_fill_attr(ustats, attr,
							 gcip_usage_stats_dvfs_freqs_show,
							 gcip_usage_stats_dvfs_freqs_store);
			break;
		default:
			dev_warn(
				ustats->dev,
				"Invalid usage stats metric, use user defined callbacks (metric=%d)",
				attr->metric);
			ret = gcip_usage_stats_fill_attr(ustats, attr,
							 gcip_usage_stats_user_defined_show,
							 gcip_usage_stats_user_defined_store);
			break;
		}

		if (ret) {
			devm_kfree(ustats->dev, ustats->attrs);
			return ret;
		}

		sysfs_attr_init(&attr->dev_attr.attr);
		ustats->attrs[i] = &attr->dev_attr.attr;
	}

	ustats->attrs[args->num_attrs] = NULL;
	ustats->group.attrs = ustats->attrs;

	return 0;
}

/* Releases @ustats->attrs allocated by the `gcip_usage_stats_alloc_attrs` function. */
static void gcip_usage_stats_free_attrs(struct gcip_usage_stats *ustats)
{
	devm_kfree(ustats->dev, ustats->attrs);
}

/* Allocates arrays which store the statistics per subcomponent. */
static int gcip_usage_stats_alloc_stats(struct gcip_usage_stats *ustats)
{
	int i;

	ustats->core_usage_htable = devm_kcalloc(ustats->dev, ustats->subcomponents,
						 sizeof(*ustats->core_usage_htable), GFP_KERNEL);
	if (!ustats->core_usage_htable)
		return -ENOMEM;

	ustats->counter = devm_kcalloc(ustats->dev, ustats->subcomponents, sizeof(*ustats->counter),
				       GFP_KERNEL);
	if (!ustats->counter)
		goto err_free_core_usage_htable;

	ustats->max_watermark = devm_kcalloc(ustats->dev, ustats->subcomponents,
					     sizeof(*ustats->max_watermark), GFP_KERNEL);
	if (!ustats->max_watermark)
		goto err_free_counter;

	for (i = 0; i < ustats->subcomponents; i++)
		hash_init(ustats->core_usage_htable[i]);

	return 0;

err_free_counter:
	devm_kfree(ustats->dev, ustats->counter);
err_free_core_usage_htable:
	devm_kfree(ustats->dev, ustats->core_usage_htable);
	return -ENOMEM;
}

/* Releases arrays which are allocated by the `gcip_usage_stats_alloc_stats` function. */
static void gcip_usage_stats_free_stats(struct gcip_usage_stats *ustats)
{
	devm_kfree(ustats->dev, ustats->max_watermark);
	devm_kfree(ustats->dev, ustats->counter);
	devm_kfree(ustats->dev, ustats->core_usage_htable);
}

/* Sets operators to the @ustats from @args. */
static int gcip_usage_stats_set_ops(struct gcip_usage_stats *ustats,
				    const struct gcip_usage_stats_args *args)
{
	if (!args->ops || !args->ops->update_usage_kci || !args->ops->get_default_dvfs_freqs_num ||
	    !args->ops->get_default_dvfs_freq)
		return -EINVAL;

	ustats->ops = args->ops;

	return 0;
}

/*
 * Fetches the default DVFS freqs from the IP driver and copies distinct values of them to
 * @ustats->default_dvfs_freqs.
 */
static int gcip_usage_stats_get_default_dvfs_freqs(struct gcip_usage_stats *ustats)
{
	int i, j, dvfs_freqs_num = ustats->ops->get_default_dvfs_freqs_num(ustats->data);
	uint32_t freq;

	for (i = 0; i < dvfs_freqs_num; i++) {
		freq = ustats->ops->get_default_dvfs_freq(i, ustats->data);

		/* See if there is a duplicate one. */
		for (j = 0; j < ustats->default_dvfs_freqs_num; j++) {
			if (ustats->default_dvfs_freqs[j] == freq)
				break;
		}

		/* If there was, skip appending it to the @default_dvfs_freqs array. */
		if (j < ustats->default_dvfs_freqs_num)
			continue;

		/* The value is distinct, but we can't store more values to the array. */
		if (ustats->default_dvfs_freqs_num == GCIP_USAGE_STATS_MAX_DVFS_FREQ_NUM) {
			dev_err(ustats->dev, "Exceeded the number limit of default DVFS freqs");
			return -EINVAL;
		}

		/* Appends the distinct one to the array. */
		ustats->default_dvfs_freqs[ustats->default_dvfs_freqs_num] = freq;
		ustats->default_dvfs_freqs_num++;
	}

	return 0;
}

int gcip_usage_stats_init(struct gcip_usage_stats *ustats, const struct gcip_usage_stats_args *args)
{
	int ret;

	if (args->version < GCIP_USAGE_STATS_V1 || args->version > GCIP_USAGE_STATS_V2)
		return -EINVAL;

	if (!args->dev)
		return -EINVAL;

	if (args->subcomponents < 1)
		return -EINVAL;

	ustats->version = args->version;
	ustats->subcomponents = args->subcomponents;
	ustats->dev = args->dev;
	ustats->data = args->data;
	mutex_init(&ustats->usage_stats_lock);
	mutex_init(&ustats->dvfs_freqs_lock);
	ustats->dvfs_freqs_num = 0;
	ustats->default_dvfs_freqs_num = 0;
	ustats->group.name = NULL;

	ret = gcip_usage_stats_set_ops(ustats, args);
	if (ret)
		return ret;

	ret = gcip_usage_stats_get_default_dvfs_freqs(ustats);
	if (ret)
		return ret;

	ret = gcip_usage_stats_alloc_stats(ustats);
	if (ret)
		return ret;

	ret = gcip_usage_stats_alloc_attrs(ustats, args);
	if (ret)
		goto err_free_stats;

	ret = device_add_group(ustats->dev, &ustats->group);
	if (ret)
		goto err_free_attrs;

	return 0;

err_free_attrs:
	gcip_usage_stats_free_attrs(ustats);
err_free_stats:
	gcip_usage_stats_free_stats(ustats);
	return ret;
}

void gcip_usage_stats_exit(struct gcip_usage_stats *ustats)
{
	device_remove_group(ustats->dev, &ustats->group);
	gcip_usage_stats_reset_dvfs_freqs(ustats);
	gcip_usage_stats_free_core_usage_all_entries(ustats);
	gcip_usage_stats_free_stats(ustats);
	gcip_usage_stats_free_attrs(ustats);
}

void gcip_usage_stats_process_buffer(struct gcip_usage_stats *ustats, void *buf)
{
	struct gcip_usage_stats_metric *metric;
	uint32_t num_metrics;
	uint32_t metric_size;
	/*
	 * Stores the version of metrics that the firmware is using.
	 * If the version of the firmware and the kernel driver are mismatching, we have to parse
	 * @buf according to the lower version.
	 */
	uint16_t fw_metric_version;
	int i;

	metric = gcip_usage_stats_parse_header(ustats, buf, &num_metrics, &metric_size,
					       &fw_metric_version);

	/* Firmware sent metrics which cannot be parsed. */
	if (fw_metric_version == GCIP_USAGE_STATS_V1 &&
	    metric_size != GCIP_USAGE_STATS_METRIC_SIZE_V1) {
		dev_err_once(ustats->dev,
			     "FW sent V1 metrics with invalid size (expected=%d, actual=%u)",
			     GCIP_USAGE_STATS_METRIC_SIZE_V1, metric_size);
		return;
	}

	/* The metric version of the firmware is higher than the kernel driver. */
	if (fw_metric_version >= GCIP_USAGE_STATS_VERSION_UPPER_BOUND ||
	    metric_size > sizeof(struct gcip_usage_stats_metric))
		dev_warn_once(
			ustats->dev,
			"FW metrics are later version with unknown fields (expected=%zu, actual=%u, fw_metric_version=%u)",
			sizeof(struct gcip_usage_stats_metric), metric_size, fw_metric_version);

	for (i = 0; i < num_metrics; i++) {
		switch (metric->type) {
		case GCIP_USAGE_STATS_METRIC_TYPE_CORE_USAGE:
			gcip_usage_stats_update_core_usage(ustats, &metric->core_usage,
							   fw_metric_version);
			break;
		case GCIP_USAGE_STATS_METRIC_TYPE_COMPONENT_UTILIZATION:
			gcip_usage_stats_update_component_utilization(
				ustats, &metric->component_utilization, fw_metric_version);
			break;
		case GCIP_USAGE_STATS_METRIC_TYPE_COUNTER:
			gcip_usage_stats_update_counter(ustats, &metric->counter,
							fw_metric_version);
			break;
		case GCIP_USAGE_STATS_METRIC_TYPE_THREAD_STATS:
			gcip_usage_stats_update_thread_stats(ustats, &metric->thread_stats,
							     fw_metric_version);
			break;
		case GCIP_USAGE_STATS_METRIC_TYPE_MAX_WATERMARK:
			gcip_usage_stats_update_max_watermark(ustats, &metric->max_watermark,
							      fw_metric_version);
			break;
		case GCIP_USAGE_STATS_METRIC_TYPE_DVFS_FREQUENCY_INFO:
			gcip_usage_stats_update_dvfs_freq_info(ustats, &metric->dvfs_frequency_info,
							       fw_metric_version);
			break;
		default:
			dev_warn(ustats->dev,
				 "Invalid usage stats metric, skip parsing it (type=%d)",
				 metric->type);
			break;
		}

		metric = (struct gcip_usage_stats_metric *)((void *)metric + metric_size);
	}
}
