// SPDX-License-Identifier: GPL-2.0-only
/* thermal_metrics.c
 *
 * Support for thermal metrics
 *
 * Copyright 2022 Google LLC
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/threads.h>
#include <linux/debugfs.h>
#include <linux/slab.h>
#include <soc/google/thermal_metrics.h>

#define MAX_NUM_SUPPORTED_THERMAL_ZONES        36
static const int default_thresholds[MAX_SUPPORTED_THRESHOLDS] = {
			60000, 80000, 90000, 100000, 103000, 105000, 110000, 115000};

struct temperature_sample {
	int temp;
	ktime_t update_time;
	int bucket;
};

struct temperature_residency_stats {
	spinlock_t lock;
	int threshold[MAX_SUPPORTED_THRESHOLDS];
	atomic64_t time_in_state_ms[MAX_SUPPORTED_THRESHOLDS + 1];
	bool started;
	char name[THERMAL_NAME_LENGTH + 1];
	struct temperature_sample prev;
	int num_thresholds;
};

struct temperature_residency_stats residency_stat_array[MAX_NUM_SUPPORTED_THERMAL_ZONES];
static tr_handle designated_handle;
static struct kobject *secondary_sysfs_folder;

/*********************************************************************
 *                          HELPER FUNCTIONS                         *
 *********************************************************************/

static void reset_residency_stats(tr_handle instance)
{
	int index;
	struct temperature_residency_stats *stats = &residency_stat_array[instance];
	for (index = 0; index < stats->num_thresholds + 1; index++) {
		atomic64_set(&(stats->time_in_state_ms[index]), 0);
	}
	return;
}

static void set_residency_thresholds(tr_handle instance, const int *thresholds)
{
	int index;
	struct temperature_residency_stats *stats = &residency_stat_array[instance];
	spin_lock(&stats->lock);
	for (index = 0; index < stats->num_thresholds; index++) {
		stats->threshold[index] = thresholds[index];
	}
	if (stats->started){
		reset_residency_stats(instance);
		stats->started = false;
	}
	spin_unlock(&stats->lock);
	return;
}

static int get_curr_bucket(tr_handle instance, int temp)
{
	int mid;
	struct temperature_residency_stats *stats = &residency_stat_array[instance];
	int low = 0, high = stats->num_thresholds;
	spin_lock(&stats->lock);
	while (low != high) {
		mid = (low + high) / 2;
		if (stats->threshold[mid] < temp) {
			low = mid + 1;
		} else {
			high = mid;
		}
	}
	spin_unlock(&stats->lock);
	return low;
}
static int get_next_available_handle(void)
{
	int index;
	for (index = 0; index < MAX_NUM_SUPPORTED_THERMAL_ZONES; index++) {
		if (residency_stat_array[index].name[0] == '\0')
			return index;
	}
	return -1;
}

/*********************************************************************
 *                       EXTERNAL REFERENCE APIs                     *
 *********************************************************************/

tr_handle register_temp_residency_stats(const char *name)
{
	tr_handle instance;
	struct temperature_residency_stats *stats;
	instance = get_next_available_handle();
	if (instance == -1)
		return -EINVAL;
	stats = &residency_stat_array[instance];
	spin_lock_init(&stats->lock);
	strncpy(stats->name, name, THERMAL_NAME_LENGTH);
	stats->num_thresholds = ARRAY_SIZE(default_thresholds);
	set_residency_thresholds(instance, default_thresholds);
	return instance;
}

EXPORT_SYMBOL_GPL(register_temp_residency_stats);

int unregister_temp_residency_stats(tr_handle instance)
{
	struct temperature_residency_stats *stats;
	if (instance < 0)
		return -EINVAL;
	stats = &residency_stat_array[instance];
	strncpy(stats->name, "", THERMAL_NAME_LENGTH);
	set_residency_thresholds(instance, default_thresholds);
	return 0;
}

EXPORT_SYMBOL_GPL(unregister_temp_residency_stats);

int temp_residency_stats_set_thresholds(tr_handle instance,
				const int *thresholds, int num_thresholds)
{
	struct temperature_residency_stats *stats;
	int index;
	if (instance < 0 || instance > MAX_NUM_SUPPORTED_THERMAL_ZONES)
		return -EINVAL;
	if (num_thresholds <= 0 || num_thresholds > MAX_SUPPORTED_THRESHOLDS)
		return -EINVAL;
	if (residency_stat_array[instance].name[0] == '\0')
		return -EINVAL;
	if (!thresholds)
		return -EINVAL;
	stats = &residency_stat_array[instance];
	/* check if threshold arr is sorted for binary search*/
	for (index = 0; index < num_thresholds - 1; index++) {
		if (thresholds[index] >= thresholds[index + 1])
			return -EINVAL;
	}
	stats->num_thresholds = num_thresholds;
	set_residency_thresholds(instance, thresholds);
	return 0;
}

EXPORT_SYMBOL_GPL(temp_residency_stats_set_thresholds);

/* Linear Model to approximate temperature residency */
int temp_residency_stats_update(tr_handle instance, int temp)
{
	int index, k, last_temp, curr_bucket;
	ktime_t curr_time = ktime_get();
	s64 latency_ms;
	struct temperature_residency_stats *stats;
	struct temperature_sample *prev_sample;
	if (instance < 0 || instance > MAX_NUM_SUPPORTED_THERMAL_ZONES)
		return -EINVAL;
	stats = &residency_stat_array[instance];
	prev_sample = &stats->prev;
	curr_bucket = get_curr_bucket(instance, temp);
	if (!stats->started) {
		stats->started = true;
		goto end;
	}
	latency_ms = ktime_to_ms(ktime_sub(curr_time, prev_sample->update_time));
	if (curr_bucket == prev_sample->bucket) {
		atomic64_add(latency_ms, &(stats->time_in_state_ms[curr_bucket]));
		goto end;
	}
	k = latency_ms / (temp - prev_sample->temp);
	last_temp = prev_sample->temp;
	index = prev_sample->bucket;

	while (index < curr_bucket) {
		atomic64_add(k * (stats->threshold[index] - last_temp),
						&(stats->time_in_state_ms[index]));
		last_temp = stats->threshold[index];
		index = index + 1;
	}
	//for the last bucket
	atomic64_add(k * (temp - last_temp), &(stats->time_in_state_ms[index]));
end:
	prev_sample->update_time = curr_time;
	prev_sample->temp = temp;
	prev_sample->bucket = curr_bucket;
	return 0;

}

EXPORT_SYMBOL_GPL(temp_residency_stats_update);

/*******************************************************************
 *                       		SYSFS			   				   *
 *******************************************************************/

static ssize_t temp_residency_all_stats_show(struct kobject *kobj,
					 struct kobj_attribute *attr,
					 char *buf)
{
	struct temperature_residency_stats *stats;
	int instance;
	int index;
	int len = 0;
	for (instance = 0 ; instance < MAX_NUM_SUPPORTED_THERMAL_ZONES ; instance++) {
		stats = &residency_stat_array[instance];
		temp_residency_stats_update(instance, stats->prev.temp);
		if (residency_stat_array[instance].name[0] != '\0') {
			len += sysfs_emit_at(buf, len, "THERMAL ZONE: %s\n", stats->name);
			len += sysfs_emit_at(buf, len,
				"NUM_TEMP_RESIDENCY_BUCKETS: %d\n", stats->num_thresholds + 1);
			len += sysfs_emit_at(buf, len, "-inf - %d ====> %lldms\n",
				stats->threshold[0], atomic64_read(&(stats->time_in_state_ms[0])));

			for (index = 0; index < stats->num_thresholds - 1; index++)
				len += sysfs_emit_at(buf, len, "%d - %d ====> %lldms\n",
					stats->threshold[index], stats->threshold[index + 1],
					atomic64_read(&(stats->time_in_state_ms[index + 1])));

			len += sysfs_emit_at(buf, len, "%d - inf ====> %lldms\n\n",
				stats->threshold[index],
				atomic64_read(&(stats->time_in_state_ms[stats->num_thresholds])));
		}
	}
	return len;
}

static ssize_t temp_residency_all_stats_reset_store(struct kobject *kobj,
					  struct kobj_attribute *attr,
					  const char *buf,
					  size_t count)
{
	int instance;
	for (instance = 0 ; instance < MAX_NUM_SUPPORTED_THERMAL_ZONES ; instance++) {
		if (residency_stat_array[instance].name[0] != '\0')
			reset_residency_stats(instance);
	}
	return count;
}

static ssize_t temp_residency_name_stats_show(struct kobject *kobj,
					 struct kobj_attribute *attr,
					 char *buf)
{
	struct temperature_residency_stats *designated_stats;
	int index;
	int len = 0;
	if (designated_handle == -1) {
		len += sysfs_emit_at(buf, len, "NO DESIGNATED THERMAL ZONE\n");
		return len;
	}
	designated_stats = &residency_stat_array[designated_handle];
	len += sysfs_emit_at(buf, len, "THERMAL ZONE: %s\n", designated_stats->name);
	len += sysfs_emit_at(buf, len, "-inf - %d ====> %lldms\n",
			designated_stats->threshold[0],
			atomic64_read(&(designated_stats->time_in_state_ms[0])));

	for (index = 0; index < designated_stats->num_thresholds - 1; index++)
		len += sysfs_emit_at(buf, len, "%d - %d ====> %lldms\n",
			designated_stats->threshold[index], designated_stats->threshold[index + 1],
			 atomic64_read(&(designated_stats->time_in_state_ms[index + 1])));

	len += sysfs_emit_at(buf, len, "%d - inf ====> %lldms\n\n",
		designated_stats->threshold[index],
	atomic64_read(&(designated_stats->time_in_state_ms[designated_stats->num_thresholds])));

	return len;
}

static ssize_t temp_residency_name_stats_reset_store(struct kobject *kobj,
					  struct kobj_attribute *attr,
					  const char *buf,
					  size_t count)
{
	if (designated_handle == -1)
		return -EINVAL;
	reset_residency_stats(designated_handle);
	return count;
}

static ssize_t temp_residency_name_show(struct kobject *kobj,
					 struct kobj_attribute *attr,
					 char *buf)
{
	struct temperature_residency_stats *designated_stats;
	int len = 0;
	if (designated_handle == -1) {
		len += sysfs_emit_at(buf, len, "NO DESIGNATED THERMAL ZONE\n");
		return len;
	}
	designated_stats = &residency_stat_array[designated_handle];
	len += sysfs_emit_at(buf, len, "THERMAL ZONE: %s\n", designated_stats->name);
	return len;
}

static ssize_t temp_residency_name_store(struct kobject *kobj,
					  struct kobj_attribute *attr,
					  const char *buf,
					  size_t count)
{
	int instance;
	char name[THERMAL_NAME_LENGTH];
	struct temperature_residency_stats *stats;
	sscanf(buf, "%s", name);
	for (instance = 0 ; instance < MAX_NUM_SUPPORTED_THERMAL_ZONES ; instance++) {
		stats = &residency_stat_array[instance];
		if (!strncmp(stats->name, name, THERMAL_NAME_LENGTH))
			designated_handle = instance;
	}
	return count;
}

static ssize_t temp_residency_all_thresholds_show(struct kobject *kobj,
					 struct kobj_attribute *attr,
					 char *buf)
{
	int instance;
	struct temperature_residency_stats *stats;
	int index;
	int len = 0;
	for (instance = 0 ; instance < MAX_NUM_SUPPORTED_THERMAL_ZONES ; instance++) {
		stats = &residency_stat_array[instance];
		if (stats->name[0] != '\0') {
			len += sysfs_emit_at(buf, len, "THERMAL ZONE: %s\n", stats->name);
			for (index = 0; index < stats->num_thresholds; index++)
				len += sysfs_emit_at(buf, len, "%d ",
						stats->threshold[index]);
			len += sysfs_emit_at(buf, len, "\n");
		}
	}
	return len;
}

static ssize_t temp_residency_all_thresholds_store(struct kobject *kobj,
					  struct kobj_attribute *attr,
					  const char *buf,
					  size_t count)
{
	int index;
	int instance;
	int ret;
	struct temperature_residency_stats *stats;
	int threshold[MAX_SUPPORTED_THRESHOLDS] = {0};
	ret  = sscanf(buf, "%d,%d,%d,%d,%d,%d,%d,%d", &threshold[0], &threshold[1],
			&threshold[2], &threshold[3], &threshold[4],
			&threshold[5], &threshold[6], &threshold[7]);
	if (ret != MAX_SUPPORTED_THRESHOLDS)
		return -EINVAL;
	/* check if threshold arr is sorted for binary search*/
	for (index = 0; index < ret - 1; index++) {
		if (threshold[index] >= threshold[index + 1]) {
			return -EINVAL;
		}
	}
	for (instance = 0 ; instance < MAX_NUM_SUPPORTED_THERMAL_ZONES ; instance++) {
		stats = &residency_stat_array[instance];
		if (stats->name[0] != '\0') {
			stats->num_thresholds = ret;
			set_residency_thresholds(instance, threshold);
		}
	}
	return count;
}

static ssize_t temp_residency_name_thresholds_show(struct kobject *kobj,
					 struct kobj_attribute *attr,
					 char *buf)
{
	struct temperature_residency_stats *designated_stats;
	int index;
	int len = 0;
	if (designated_handle == -1) {
		len += sysfs_emit_at(buf, len, "NO DESIGNATED THERMAL ZONE\n");
		return len;
	}
	designated_stats = &residency_stat_array[designated_handle];
	len += sysfs_emit_at(buf, len, "THERMAL ZONE: %s\n", designated_stats->name);
	for (index = 0; index < designated_stats->num_thresholds; index++)
		len += sysfs_emit_at(buf, len, "%d ",
				     designated_stats->threshold[index]);
	len += sysfs_emit_at(buf, len, "\n");
	return len;
}

static ssize_t temp_residency_name_thresholds_store(struct kobject *kobj,
					  struct kobj_attribute *attr,
					  const char *buf,
					  size_t count)
{
	int index;
	int ret;
	struct temperature_residency_stats *designated_stats;
	int threshold[MAX_SUPPORTED_THRESHOLDS] = {0};
	if (designated_handle == -1)
		return -EINVAL;
	ret  = sscanf(buf, "%d,%d,%d,%d,%d,%d,%d,%d", &threshold[0], &threshold[1],
			&threshold[2], &threshold[3], &threshold[4],
			&threshold[5], &threshold[6], &threshold[7]);
	if (ret != MAX_SUPPORTED_THRESHOLDS)
		return -EINVAL;
	/* check if threshold arr is sorted for binary search*/
	for (index = 0; index < ret - 1; index++) {
		if (threshold[index] >= threshold[index + 1]) {
			return -EINVAL;
		}
	}
	designated_stats = &residency_stat_array[designated_handle];
	designated_stats->num_thresholds = ret;
	set_residency_thresholds(designated_handle, threshold);

	return count;
}

static ssize_t all_tz_name_show(struct kobject *kobj,
					 struct kobj_attribute *attr,
					 char *buf)
{
	int instance;
	int len = 0;
	struct temperature_residency_stats *stats;
	for (instance = 0 ; instance < MAX_NUM_SUPPORTED_THERMAL_ZONES ; instance++) {
		stats = &residency_stat_array[instance];
		if (stats->name[0] != '\0')
			len += sysfs_emit_at(buf, len, "%s,", stats->name);
	}
	return len;
}

static struct kobj_attribute temp_residency_all_stats_attr = __ATTR(stats,
							0444,
							temp_residency_all_stats_show,
							NULL);

static struct kobj_attribute temp_residency_all_stats_reset_attr = __ATTR(
							stats_reset,
							0200,
							NULL,
							temp_residency_all_stats_reset_store);

static struct kobj_attribute temp_residency_name_stats_attr = __ATTR(name_stats,
							0444,
							temp_residency_name_stats_show,
							NULL);

static struct kobj_attribute temp_residency_name_stats_reset_attr = __ATTR(
							name_stats_reset,
							0200,
							NULL,
							temp_residency_name_stats_reset_store);

static struct kobj_attribute temp_residency_name_attr = __ATTR(name,
							0664,
							temp_residency_name_show,
							temp_residency_name_store);

static struct kobj_attribute temp_residency_all_thresholds_attr = __ATTR(
							thresholds,
							0664,
							temp_residency_all_thresholds_show,
							temp_residency_all_thresholds_store);

static struct kobj_attribute temp_residency_name_thresholds_attr = __ATTR(
							name_thresholds,
							0664,
							temp_residency_name_thresholds_show,
							temp_residency_name_thresholds_store);

static struct kobj_attribute all_tz_name_attr = __ATTR(all_tz_name,
							0444,
							all_tz_name_show,
							NULL);


static struct attribute *temp_residency_all_attrs[] = {
	&temp_residency_all_stats_attr.attr,
	&temp_residency_all_stats_reset_attr.attr,
	&temp_residency_all_thresholds_attr.attr,
	NULL
};

static struct attribute *temp_residency_name_attrs[] = {
	&temp_residency_name_stats_attr.attr,
	&temp_residency_name_stats_reset_attr.attr,
	&temp_residency_name_attr.attr,
	&temp_residency_name_thresholds_attr.attr,
	&all_tz_name_attr.attr,
	NULL
};
static const struct attribute_group temp_residency_all_attr_group = {
	.attrs = temp_residency_all_attrs,
	.name = "temp_residency_all"
};
static const struct attribute_group temp_residency_name_attr_group = {
	.attrs = temp_residency_name_attrs,
	.name = "temp_residency_name"
};

/*********************************************************************
 *                  		INITIALIZE DRIVER                        *
 *********************************************************************/

int thermal_metrics_init(struct kobject *metrics_kobj)
{
	int err = 0;
	designated_handle = -1;
	if (!metrics_kobj) {
		pr_err("metrics_kobj is not initialized\n");
		return -EINVAL;
	}
	secondary_sysfs_folder = kobject_create_and_add("temp_residency", metrics_kobj);
	if (!secondary_sysfs_folder) {
		pr_err("Failed to create secondary sysfs folder!\n");
		return -ENOMEM;
	}
	err = sysfs_create_group(secondary_sysfs_folder, &temp_residency_all_attr_group);
	if (err) {
		pr_err("failed to create temp_residency_all folder\n");
		return err;
	}
	err = sysfs_create_group(secondary_sysfs_folder, &temp_residency_name_attr_group);
	if (err) {
		pr_err("failed to create temp_residency_name folder\n");
		return err;
	}
	pr_info("thermal_metrics driver initialized! :D\n");
	return err;
}

