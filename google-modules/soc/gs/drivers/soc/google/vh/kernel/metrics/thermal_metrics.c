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
#define MAX_NUM_SUPPORTED_THERMAL_GROUPS       10
#define ABNORMAL_UEVENT_TIMEOUT_SEC            1200


static const int default_thresholds[MAX_SUPPORTED_THRESHOLDS] = {
			60000, 80000, 90000, 100000, 103000, 105000, 110000, 115000};

static const struct temp_validity_config default_temp_validity_config = {
	.min_temp = -20000,
	.max_temp = 131000,
	.min_repeat_count = 8,
	.min_repeat_msecs = 120000,
};

struct temperature_bucket_sample {
	int temp;
	ktime_t update_time;
	ktime_t start_time;
	int bucket;
	int repeat_ct;
};

struct thermal_group_entry {
	rwlock_t lock;
	struct kobject *kobj;
	char *name;
	struct temp_validity_config temp_validity_config;
};

struct temperature_residency_stats {
	spinlock_t lock;
	int threshold[MAX_SUPPORTED_THRESHOLDS];
	atomic64_t time_in_state_ms[MAX_SUPPORTED_THRESHOLDS + 1];
	struct tr_sample max_sample;
	struct tr_sample min_sample;
	bool started;
	char name[THERMAL_NAME_LENGTH + 1];
	struct temperature_bucket_sample prev;
	int num_thresholds;
	struct thermal_group_entry *thermal_group;
	bool use_callback;
	struct temp_residency_stats_callbacks ops;
};

static struct thermal_group_entry thermal_group_array[MAX_NUM_SUPPORTED_THERMAL_GROUPS];
static struct temperature_residency_stats residency_stat_array[MAX_NUM_SUPPORTED_THERMAL_ZONES];
static time64_t last_abnormal_uevent_time[MAX_NUM_SUPPORTED_THERMAL_ZONES][ABNORMALITY_TYPE_END];

static tr_handle designated_handle;
static struct kobject *tr_by_group_kobj;
static struct attribute_group temp_residency_all_attr_group,
				temp_abnormality_all_attr_group;

/*********************************************************************
 *                          HELPER FUNCTIONS                         *
 *********************************************************************/
static void set_temperature_sample(struct tr_sample *sample, int temp)
{
	sample->temp = temp;
	sample->timestamp = ktime_get_real_seconds();
}

static void reset_residency_stats(tr_handle instance)
{
	int index;
	struct temperature_residency_stats *stats = &residency_stat_array[instance];

	if (stats->use_callback) {
		stats->ops.reset_stats(instance);
		return;
	}
	for (index = 0; index < stats->num_thresholds + 1; index++)
		atomic64_set(&(stats->time_in_state_ms[index]), 0);
	set_temperature_sample(&stats->max_sample, INT_MIN);
	set_temperature_sample(&stats->min_sample, INT_MAX);
}

static void set_residency_thresholds(tr_handle instance, const int *thresholds)
{
	int index, err;
	struct temperature_residency_stats *stats = &residency_stat_array[instance];

	if (stats->use_callback) {
		err = stats->ops.set_thresholds(instance, thresholds, stats->num_thresholds);
		if (err < 0)
			pr_err("Fail setting residency thresholds to %s: %d\n", stats->name, err);
		return;
	}

	spin_lock(&stats->lock);
	for (index = 0; index < stats->num_thresholds; index++)
		stats->threshold[index] = thresholds[index];
	if (stats->started) {
		reset_residency_stats(instance);
		stats->started = false;
	}
	spin_unlock(&stats->lock);
}

static int get_curr_bucket(tr_handle instance, int temp)
{
	int mid;
	struct temperature_residency_stats *stats = &residency_stat_array[instance];
	int low = 0, high = stats->num_thresholds;

	spin_lock(&stats->lock);
	while (low != high) {
		mid = (low + high) / 2;
		if (stats->threshold[mid] < temp)
			low = mid + 1;
		else
			high = mid;
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

/**
 * get_thermal_group_by_kobj() - Find thermal group with given kobject member.
 * @kobj: Kobject corresponding to the thermal group sysfs.
 *
 * Find thermal_group_entry in thermal_group_array having the given kobj.
 *
 * Return: Pointer to thermal_group_entry if found else NULL.
 */
static struct thermal_group_entry *get_thermal_group_by_kobj(struct kobject *kobj)
{
	int idx;

	if (!kobj)
		return NULL;

	for (idx = 0; idx < MAX_NUM_SUPPORTED_THERMAL_GROUPS; idx++)
		if (kobj == thermal_group_array[idx].kobj)
			return &thermal_group_array[idx];

	return NULL;
}

/**
 * create_thermal_group() - Find thermal group with given name if exists
 *                          else create a new thermal_group with given name.
 * @name: name assigned to the thermal group.
 *
 * Find thermal_group_entry in thermal_group_array having the given name. Since
 * entries are added sequentially, empty name indicates end of entries. So add
 * a new entry at the index with the given name.
 *
 * Return: Pointer to thermal_group_entry if success else NULL on error.
 */
static struct thermal_group_entry *create_thermal_group(char *name)
{
	struct kobject *thermal_group_kobj;
	int index;
	int err;

	for (index = 0; index < MAX_NUM_SUPPORTED_THERMAL_GROUPS; index++) {
		if (!thermal_group_array[index].name || thermal_group_array[index].name[0] == '\0')
			break;
		if (!strncmp(name, thermal_group_array[index].name, THERMAL_NAME_LENGTH))
			return &thermal_group_array[index];
	}
	if (index >= MAX_NUM_SUPPORTED_THERMAL_GROUPS) {
		pr_err("Failed to support more thermal groups\n");
		return NULL;
	}
	thermal_group_kobj = kobject_create_and_add(name, tr_by_group_kobj);
	if (!thermal_group_kobj) {
		pr_err("Failed to create thermal_group folder!\n");
		return NULL;
	}
	err = sysfs_create_group(thermal_group_kobj, &temp_residency_all_attr_group);
	if (err) {
		pr_err("ERR:%d failed to create temp_residency_all files\n", err);
		goto put_thermal_kobj;
	}
	err = sysfs_create_group(thermal_group_kobj, &temp_abnormality_all_attr_group);
	if (err) {
		pr_err("ERR:%d failed to create temp_abnormality_all files\n", err);
		goto remove_residency_sysfs;
	}
	thermal_group_array[index].name = kstrdup(name, GFP_KERNEL);
	thermal_group_array[index].kobj = thermal_group_kobj;
	thermal_group_array[index].temp_validity_config = default_temp_validity_config;
	rwlock_init(&thermal_group_array[index].lock);
	return &thermal_group_array[index];

remove_residency_sysfs:
	sysfs_remove_group(thermal_group_kobj, &temp_residency_all_attr_group);
put_thermal_kobj:
	kobject_put(thermal_group_kobj);
	return NULL;
}

static int get_tz_cb_threshold(tr_handle instance, struct temperature_residency_stats *stats)
{
	return stats->ops.get_thresholds(instance, stats->threshold, &stats->num_thresholds);
}

static int get_tz_cb_stats(tr_handle instance, struct temperature_residency_stats *stats)
{
	return stats->ops.get_stats(instance, stats->time_in_state_ms,
					&stats->max_sample, &stats->min_sample);
}

/**
 * report_thermal_abnormal_uevent() - Send uevent to notify thermal abnormality.
 * @instance: tr_handle for the index of tz in residency_stat_array.
 * @type: type of abnormality_type detected.
 * @temp: temperature for which abnormality detected.
 *
 * Send uevent with environment parameters:
 *  THERMAL_ABNORMAL_TYPE(type)
 *  THERMAL_ABNORMAL_INFO(sensor, temp)
 * Rate limit logging to have time gap of at least ABNORMAL_UEVENT_TIMEOUT_SEC
 * between uevents of same abnormality_type.
 */
int report_thermal_abnormal_uevent(tr_handle instance, enum abnormality_type type,
					 int temp)
{
	time64_t now = ktime_get_seconds();
	struct kobject mod_kobj = (((struct module *)(THIS_MODULE))->mkobj).kobj;
	char env_abnormality_type[64], env_abnormality_info[64];
	char *envp[] = {env_abnormality_type, env_abnormality_info, NULL};
	const char *sensor;

	if (instance < 0 || instance >= MAX_NUM_SUPPORTED_THERMAL_ZONES)
		return -EINVAL;
	if (type < 0 || type >= ABNORMALITY_TYPE_END)
		return -EINVAL;

	sensor = residency_stat_array[instance].name;
	if (last_abnormal_uevent_time[instance][type] &&
		now - last_abnormal_uevent_time[instance][type] < ABNORMAL_UEVENT_TIMEOUT_SEC) {
		pr_debug("Thermal abnormal uevent rate limited for type:%s sensor:%s temp:%d",
			abnormality_type_str[type], sensor, temp);
		return 0;
	}

	snprintf(env_abnormality_type, sizeof(env_abnormality_type),
		"THERMAL_ABNORMAL_TYPE=%s", abnormality_type_str[type]);
	snprintf(env_abnormality_info, sizeof(env_abnormality_info),
		"THERMAL_ABNORMAL_INFO=name:%s,val:%d", sensor, temp);
	kobject_uevent_env(&mod_kobj, KOBJ_CHANGE, envp);
	last_abnormal_uevent_time[instance][type] = now;
	return 0;
}

/**
 * verify_temp_range() - Verify if temp is within valid range.
 * @instance: tr_handle for the index of tz in residency_stat_array.
 * @abnormality_type: pointer to abnormality_type to be set if any abnormality detected.
 *
 * verify if temperature within expected range, if not report thermal abnormal
 * uevent provided it's not same reading repeated.
 *
 * Return: 0 if temperature within range else 1
 */
static int verify_temp_range(tr_handle instance, enum abnormality_type *abnormality_type)
{
	struct temperature_residency_stats *stats = &residency_stat_array[instance];
	struct thermal_group_entry *thermal_group = stats->thermal_group;
	int temp = stats->prev.temp;

	read_lock(&thermal_group->lock);
	if (unlikely(thermal_group->temp_validity_config.min_temp > temp)) {
		*abnormality_type = EXTREME_LOW_TEMP;
		goto err;
	} else if (unlikely(thermal_group->temp_validity_config.max_temp < temp)) {
		*abnormality_type = EXTREME_HIGH_TEMP;
		goto err;
	}
	read_unlock(&thermal_group->lock);
	return 0;

err:
	read_unlock(&thermal_group->lock);
	if (stats->prev.repeat_ct == 1)
		report_thermal_abnormal_uevent(instance, *abnormality_type, temp);
	return 1;
}

/**
 * verify_temp_stuck() - Verify if sensor temp reading is stuck.
 * @instance: tr_handle for the index of tz in residency_stat_array.
 * @abnormality_type: pointer to abnormality_type to be set if any abnormality detected.
 *
 * Verify if sensor reporting same temp for temp_validity_config.min_repeat_count
 * consecutive times and for temp_validity_config.min_repeat_msecs duration.
 * If so:
 *  1) print abnormality info.
 *  2) report thermal abnormal uevent.
 *  3) Reset stats prev sample repeat count and start time to last update time to
 *     restart stuck verification.
 *
 * Return: 1 if sensor stuck else 0.
 */
static int verify_temp_stuck(tr_handle instance, enum abnormality_type *abnormality_type)
{
	struct temperature_residency_stats *stats = &residency_stat_array[instance];
	struct temperature_bucket_sample *temp_sample = &stats->prev;
	struct thermal_group_entry *thermal_group = stats->thermal_group;
	s64 repeat_msecs;

	read_lock(&thermal_group->lock);
	if (likely(temp_sample->repeat_ct < thermal_group->temp_validity_config.min_repeat_count))
		goto ret;
	repeat_msecs = ktime_to_ms(ktime_sub(temp_sample->update_time,
						temp_sample->start_time));
	if (repeat_msecs < thermal_group->temp_validity_config.min_repeat_msecs)
		goto ret;
	read_unlock(&thermal_group->lock);

	*abnormality_type = SENSOR_STUCK;
	pr_debug("Thermal Abnormality Detected: sensor:%s temp:%d stuck %d cnt %lld ms",
		stats->name, temp_sample->temp, temp_sample->repeat_ct, repeat_msecs);
	report_thermal_abnormal_uevent(instance, *abnormality_type, temp_sample->temp);

	temp_sample->start_time = temp_sample->update_time;
	temp_sample->repeat_ct = 1;
	return 1;

ret:
	read_unlock(&thermal_group->lock);
	return 0;
}

/*********************************************************************
 *                       EXTERNAL REFERENCE APIs                     *
 *********************************************************************/

tr_handle register_temp_residency_stats(const char *name, char *group_name)
{
	tr_handle instance;
	struct temperature_residency_stats *stats;
	struct thermal_group_entry *thermal_group;

	if (!name || name[0] == '\0')
		return -EINVAL;
	if (!group_name || group_name[0] == '\0')
		return -EINVAL;
	instance = get_next_available_handle();
	if (instance == -1)
		return -EINVAL;
	stats = &residency_stat_array[instance];
	spin_lock_init(&stats->lock);
	strncpy(stats->name, name, THERMAL_NAME_LENGTH);
	stats->num_thresholds = ARRAY_SIZE(default_thresholds);
	set_residency_thresholds(instance, default_thresholds);
	stats->started = false;
	stats->use_callback = false;
	stats->ops = (struct temp_residency_stats_callbacks){NULL, NULL, NULL, NULL};

	thermal_group = create_thermal_group(group_name);
	if (!thermal_group)
		return -ENOMEM;
	stats->thermal_group = thermal_group;

	return instance;
}
EXPORT_SYMBOL_GPL(register_temp_residency_stats);

int register_temp_residency_stats_callbacks(tr_handle instance,
					    struct temp_residency_stats_callbacks *ops)
{
	struct temperature_residency_stats *stats;

	if (!ops || (instance < 0) || (instance >= MAX_NUM_SUPPORTED_THERMAL_ZONES))
		return -EINVAL;

	stats = &residency_stat_array[instance];
	stats->use_callback = ops->get_stats && ops->get_thresholds &&
			      ops->set_thresholds && ops->reset_stats;
	if (!stats->use_callback)
		return -EINVAL;

	stats->ops = *ops;
	stats->ops.set_thresholds(instance, stats->threshold, stats->num_thresholds);
	return 0;
}
EXPORT_SYMBOL_GPL(register_temp_residency_stats_callbacks);

int unregister_temp_residency_stats(tr_handle instance)
{
	struct temperature_residency_stats *stats;

	if (instance < 0 || (instance >= MAX_NUM_SUPPORTED_THERMAL_ZONES))
		return -EINVAL;
	stats = &residency_stat_array[instance];
	strncpy(stats->name, "", THERMAL_NAME_LENGTH);
	set_residency_thresholds(instance, default_thresholds);
	stats->ops = (struct temp_residency_stats_callbacks){NULL, NULL, NULL, NULL};
	stats->use_callback = false;
	return 0;
}
EXPORT_SYMBOL_GPL(unregister_temp_residency_stats);

int temp_residency_stats_set_thresholds(tr_handle instance,
				const int *thresholds, int num_thresholds)
{
	struct temperature_residency_stats *stats;
	int index;

	if (instance < 0 || instance >= MAX_NUM_SUPPORTED_THERMAL_ZONES)
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

/**
 * temp_residency_stats_update() - Update thermal metrics & verify abnormality.
 * @instance: tr_handle for the index of tz in residency_stat_array.
 * @temp: temperature reading.
 *
 * For new temperature reading, update the residency stats array, daily min/max
 * temperature.
 * Verify if temperature is within valid range.
 * Verify if temperature stuck.
 *
 * Return:
 *  0 on success.
 *  -EINVAL if given instance doesn't exist.
 *  enum abnormality_type if any abnormality detected.
 */
int temp_residency_stats_update(tr_handle instance, int temp)
{
	int index, k, last_temp, curr_bucket, stride_len;
	int abnormality_detected = 0;
	ktime_t curr_time = ktime_get();
	s64 latency_ms;
	enum abnormality_type abnormality_type;
	struct temperature_residency_stats *stats;
	struct temperature_bucket_sample *prev_sample;

	if (instance < 0 || instance > MAX_NUM_SUPPORTED_THERMAL_ZONES)
		return -EINVAL;
	stats = &residency_stat_array[instance];

	/* Avoid using uninitialized stats */
	if (stats->name[0] == '\0')
		return -EINVAL;

	prev_sample = &stats->prev;
	curr_bucket = get_curr_bucket(instance, temp);
	if (!stats->started) {
		stats->started = true;
		set_temperature_sample(&stats->max_sample, temp);
		set_temperature_sample(&stats->min_sample, temp);
		goto end;
	}
	latency_ms = ktime_to_ms(ktime_sub(curr_time, prev_sample->update_time));
	if (curr_bucket == prev_sample->bucket) {
		atomic64_add(latency_ms, &(stats->time_in_state_ms[curr_bucket]));
		if (temp == prev_sample->temp) {
			prev_sample->update_time = curr_time;
			prev_sample->repeat_ct++;
			abnormality_detected = verify_temp_stuck(instance, &abnormality_type);
			if (abnormality_detected)
				return abnormality_type;
			goto outlier_check;
		}
		goto end;
	}

	k = mult_frac(1000, latency_ms, temp - prev_sample->temp);
	last_temp = prev_sample->temp;
	index = prev_sample->bucket;

	//stride_len is set to -1 if temperature decrease.
	stride_len = (index < curr_bucket) ? 1 : -1;

	while (index != curr_bucket) {
		if (stride_len == 1) {
			atomic64_add(
				mult_frac(k,
				stats->threshold[index] - last_temp, 1000),
				&(stats->time_in_state_ms[index]));
			last_temp = stats->threshold[index];
		} else {
			atomic64_add(
				mult_frac(k,
				stats->threshold[index - 1] - last_temp, 1000),
				&(stats->time_in_state_ms[index]));
			last_temp = stats->threshold[index - 1];
		}
		index = index + stride_len;
	}
	//for the last bucket
	atomic64_add(mult_frac(k, temp - last_temp, 1000),
		&(stats->time_in_state_ms[index]));

end:
	prev_sample->update_time = prev_sample->start_time = curr_time;
	prev_sample->temp = temp;
	prev_sample->bucket = curr_bucket;
	prev_sample->repeat_ct = 1;
	if (temp > stats->max_sample.temp)
		set_temperature_sample(&stats->max_sample, temp);
	if (temp < stats->min_sample.temp)
		set_temperature_sample(&stats->min_sample, temp);
outlier_check:
	abnormality_detected = verify_temp_range(instance, &abnormality_type);
	if (abnormality_detected)
		return abnormality_type;
	return 0;
}
EXPORT_SYMBOL_GPL(temp_residency_stats_update);

/*******************************************************************
 *				SYSFS							   *
 *******************************************************************/

static ssize_t temp_residency_all_stats_show(struct kobject *kobj,
					 struct kobj_attribute *attr,
					 char *buf)
{
	struct temperature_residency_stats *stats;
	int instance;
	int index, ret;
	int len = 0;

	for (instance = 0 ; instance < MAX_NUM_SUPPORTED_THERMAL_ZONES ; instance++) {
		stats = &residency_stat_array[instance];
		if (stats->name[0] == '\0' || kobj != stats->thermal_group->kobj)
			continue;

		if (stats->use_callback) {
			ret = get_tz_cb_threshold(instance, stats);
			if (ret) {
				pr_err("thresholds read failed: %s\n", stats->name);
				goto end_err;
			}
			ret = get_tz_cb_stats(instance, stats);
			if (ret) {
				pr_err("stats read failed: %s\n", stats->name);
				goto end_err;
			}
		} else {
			temp_residency_stats_update(instance, stats->prev.temp);
		}

		len += sysfs_emit_at(buf, len, "THERMAL ZONE: %s\n", stats->name);
		len += sysfs_emit_at(buf, len, "MAX_TEMP: %d\n", stats->max_sample.temp);
		len += sysfs_emit_at(buf, len, "MAX_TEMP_TIMESTAMP: %llds\n",
			stats->max_sample.timestamp);
		len += sysfs_emit_at(buf, len, "MIN_TEMP: %d\n", stats->min_sample.temp);
		len += sysfs_emit_at(buf, len, "MIN_TEMP_TIMESTAMP: %llds\n",
			stats->min_sample.timestamp);
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
	return len;
end_err:
	return ret;
}

static ssize_t temp_residency_all_stats_reset_store(struct kobject *kobj,
					  struct kobj_attribute *attr,
					  const char *buf,
					  size_t count)
{
	int instance;

	for (instance = 0 ; instance < MAX_NUM_SUPPORTED_THERMAL_ZONES ; instance++) {
		if (residency_stat_array[instance].name[0] != '\0' &&
		    kobj == residency_stat_array[instance].thermal_group->kobj)
			reset_residency_stats(instance);
	}
	return count;
}

static ssize_t temp_residency_name_stats_show(struct kobject *kobj,
					 struct kobj_attribute *attr,
					 char *buf)
{
	struct temperature_residency_stats *designated_stats;
	int index, ret;
	int len = 0;

	if (designated_handle == -1) {
		len += sysfs_emit_at(buf, len, "NO DESIGNATED THERMAL ZONE\n");
		return len;
	}
	designated_stats = &residency_stat_array[designated_handle];
	if (designated_stats->use_callback) {
		ret = get_tz_cb_threshold(designated_handle, designated_stats);
		if (ret) {
			pr_err("thresholds read failed: %s\n", designated_stats->name);
			goto end_err;
		}
		ret = get_tz_cb_stats(designated_handle, designated_stats);
		if (ret) {
			pr_err("stats read failed: %s\n", designated_stats->name);
			goto end_err;
		}
	} else {
		temp_residency_stats_update(designated_handle, designated_stats->prev.temp);
	}
	len += sysfs_emit_at(buf, len, "THERMAL ZONE: %s\n", designated_stats->name);
	len += sysfs_emit_at(buf, len, "MAX_TEMP: %d\n", designated_stats->max_sample.temp);
	len += sysfs_emit_at(buf, len, "MAX_TEMP_TIMESTAMP: %llds\n",
			designated_stats->max_sample.timestamp);
	len += sysfs_emit_at(buf, len, "MIN_TEMP: %d\n", designated_stats->min_sample.temp);
	len += sysfs_emit_at(buf, len, "MIN_TEMP_TIMESTAMP: %llds\n",
			designated_stats->min_sample.timestamp);
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
end_err:
	return ret;
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
	int instance, ret;
	char name[THERMAL_NAME_LENGTH];
	struct temperature_residency_stats *stats;

	ret = sscanf(buf, "%s", name);
	if (!ret)
		return -EINVAL;
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
	int index, ret;
	int len = 0;

	for (instance = 0 ; instance < MAX_NUM_SUPPORTED_THERMAL_ZONES ; instance++) {
		stats = &residency_stat_array[instance];
		if (stats->name[0] == '\0' || kobj != stats->thermal_group->kobj)
			continue;

		if (stats->use_callback) {
			ret = get_tz_cb_threshold(instance, stats);
			if (ret) {
				pr_err("thresholds read failed: %s\n", stats->name);
				return ret;
			}
		}
		len += sysfs_emit_at(buf, len, "THERMAL ZONE: %s\n", stats->name);
		for (index = 0; index < stats->num_thresholds; index++)
			len += sysfs_emit_at(buf, len, "%d ",
					stats->threshold[index]);
		len += sysfs_emit_at(buf, len, "\n");
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
	if ((ret == 0) || (ret > MAX_SUPPORTED_THRESHOLDS))
		return -EINVAL;
	/* check if threshold arr is sorted for binary search*/
	for (index = 0; index < ret - 1; index++)
		if (threshold[index] >= threshold[index + 1])
			return -EINVAL;

	for (instance = 0 ; instance < MAX_NUM_SUPPORTED_THERMAL_ZONES ; instance++) {
		stats = &residency_stat_array[instance];
		if (stats->name[0] != '\0' && kobj == stats->thermal_group->kobj) {
			stats->num_thresholds = ret;
			set_residency_thresholds(instance, threshold);
		}
	}
	return count;
}

static ssize_t abnormal_temp_max_threshold_show(struct kobject *kobj,
					 struct kobj_attribute *attr,
					 char *buf)
{
	int ret = 0;
	struct thermal_group_entry *thermal_group = get_thermal_group_by_kobj(kobj);

	if (!thermal_group)
		return -EINVAL;

	read_lock(&thermal_group->lock);
	ret = sysfs_emit(buf, "%d\n", thermal_group->temp_validity_config.max_temp);
	read_unlock(&thermal_group->lock);
	return ret;
}

static ssize_t abnormal_temp_max_threshold_store(struct kobject *kobj,
					  struct kobj_attribute *attr,
					  const char *buf,
					  size_t count)
{
	struct thermal_group_entry *thermal_group;
	int ret, max_temp;

	ret = kstrtoint(buf, 0, &max_temp);
	if (ret)
		return -EINVAL;

	thermal_group = get_thermal_group_by_kobj(kobj);
	if (!thermal_group)
		return -EINVAL;

	write_lock(&thermal_group->lock);
	if (thermal_group->temp_validity_config.min_temp > max_temp) {
		write_unlock(&thermal_group->lock);
		return -EINVAL;
	}
	thermal_group->temp_validity_config.max_temp = max_temp;
	write_unlock(&thermal_group->lock);
	return count;
}

static ssize_t abnormal_temp_min_threshold_show(struct kobject *kobj,
					 struct kobj_attribute *attr,
					 char *buf)
{
	int ret = 0;
	struct thermal_group_entry *thermal_group = get_thermal_group_by_kobj(kobj);

	if (!thermal_group)
		return -EINVAL;

	read_lock(&thermal_group->lock);
	ret = sysfs_emit(buf, "%d\n", thermal_group->temp_validity_config.min_temp);
	read_unlock(&thermal_group->lock);
	return ret;
}

static ssize_t abnormal_temp_min_threshold_store(struct kobject *kobj,
					  struct kobj_attribute *attr,
					  const char *buf,
					  size_t count)
{
	struct thermal_group_entry *thermal_group;
	int ret, min_temp;

	ret = kstrtoint(buf, 0, &min_temp);
	if (ret)
		return -EINVAL;

	thermal_group = get_thermal_group_by_kobj(kobj);
	if (!thermal_group)
		return -EINVAL;

	write_lock(&thermal_group->lock);
	if (thermal_group->temp_validity_config.max_temp < min_temp) {
		write_unlock(&thermal_group->lock);
		return -EINVAL;
	}
	thermal_group->temp_validity_config.min_temp = min_temp;
	write_unlock(&thermal_group->lock);
	return count;
}

static ssize_t abnormal_temp_stuck_count_show(struct kobject *kobj,
					 struct kobj_attribute *attr,
					 char *buf)
{
	int ret = 0;
	struct thermal_group_entry *thermal_group = get_thermal_group_by_kobj(kobj);

	if (!thermal_group)
		return -EINVAL;

	read_lock(&thermal_group->lock);
	ret = sysfs_emit(buf, "%d\n", thermal_group->temp_validity_config.min_repeat_count);
	read_unlock(&thermal_group->lock);
	return ret;
}

static ssize_t abnormal_temp_stuck_count_store(struct kobject *kobj,
					  struct kobj_attribute *attr,
					  const char *buf,
					  size_t count)
{
	struct thermal_group_entry *thermal_group;
	int ret;
	u8 repeat_count;

	ret = kstrtou8(buf, 0, &repeat_count);
	if (ret || !repeat_count)
		return -EINVAL;

	thermal_group = get_thermal_group_by_kobj(kobj);
	if (!thermal_group)
		return -EINVAL;

	write_lock(&thermal_group->lock);
	thermal_group->temp_validity_config.min_repeat_count = repeat_count;
	write_unlock(&thermal_group->lock);
	return count;
}

static ssize_t abnormal_temp_stuck_duration_show(struct kobject *kobj,
					 struct kobj_attribute *attr,
					 char *buf)
{
	int ret = 0;
	struct thermal_group_entry *thermal_group = get_thermal_group_by_kobj(kobj);

	if (!thermal_group)
		return -EINVAL;

	read_lock(&thermal_group->lock);
	ret = sysfs_emit(buf, "%d\n", thermal_group->temp_validity_config.min_repeat_msecs);
	read_unlock(&thermal_group->lock);
	return ret;
}

static ssize_t abnormal_temp_stuck_duration_store(struct kobject *kobj,
					  struct kobj_attribute *attr,
					  const char *buf,
					  size_t count)
{
	struct thermal_group_entry *thermal_group;
	int ret, repeat_msecs;

	ret = kstrtoint(buf, 0, &repeat_msecs);
	if (ret || repeat_msecs <= 0)
		return -EINVAL;

	thermal_group = get_thermal_group_by_kobj(kobj);
	if (!thermal_group)
		return -EINVAL;

	write_lock(&thermal_group->lock);
	thermal_group->temp_validity_config.min_repeat_msecs = repeat_msecs;
	write_unlock(&thermal_group->lock);
	return count;
}

static ssize_t temp_residency_name_thresholds_show(struct kobject *kobj,
					 struct kobj_attribute *attr,
					 char *buf)
{
	struct temperature_residency_stats *designated_stats;
	int index, ret;
	int len = 0;

	if (designated_handle == -1) {
		len += sysfs_emit_at(buf, len, "NO DESIGNATED THERMAL ZONE\n");
		return len;
	}
	designated_stats = &residency_stat_array[designated_handle];
	if (designated_stats->use_callback) {
		ret = get_tz_cb_threshold(designated_handle, designated_stats);
		if (ret) {
			pr_err("thresholds read failed: %s\n", designated_stats->name);
			return ret;
		}
	}
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
	if ((ret == 0) || (ret > MAX_SUPPORTED_THRESHOLDS))
		return -EINVAL;
	/* check if threshold arr is sorted for binary search*/
	for (index = 0; index < ret - 1; index++)
		if (threshold[index] >= threshold[index + 1])
			return -EINVAL;

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
	len += sysfs_emit_at(buf, len, "\n");
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

static struct kobj_attribute abnormal_min_temp_threshold_attr = __ATTR(
							min_temp_threshold,
							0664,
							abnormal_temp_min_threshold_show,
							abnormal_temp_min_threshold_store);

static struct kobj_attribute abnormal_max_temp_threshold_attr = __ATTR(
							max_temp_threshold,
							0664,
							abnormal_temp_max_threshold_show,
							abnormal_temp_max_threshold_store);

static struct kobj_attribute abnormal_temp_stuck_count_attr = __ATTR(
							temp_stuck_repeat_cnt,
							0664,
							abnormal_temp_stuck_count_show,
							abnormal_temp_stuck_count_store);

static struct kobj_attribute abnormal_temp_stuck_duration_attr = __ATTR(
							temp_stuck_repeat_ms,
							0664,
							abnormal_temp_stuck_duration_show,
							abnormal_temp_stuck_duration_store);

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

static struct attribute *temp_abnormality_all_attrs[] = {
	&abnormal_max_temp_threshold_attr.attr,
	&abnormal_min_temp_threshold_attr.attr,
	&abnormal_temp_stuck_count_attr.attr,
	&abnormal_temp_stuck_duration_attr.attr,
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

static const struct attribute_group temp_residency_name_attr_group = {
	.attrs = temp_residency_name_attrs,
	.name = "tr_by_name"
};

/*********************************************************************
 *				INITIALIZE DRIVER                        *
 *********************************************************************/

int thermal_metrics_init(struct kobject *metrics_kobj)
{
	struct kobject *secondary_sysfs_folder;
	int err = 0;

	designated_handle = -1;
	if (!metrics_kobj) {
		pr_err("metrics_kobj is not initialized\n");
		return -EINVAL;
	}
	secondary_sysfs_folder = kobject_create_and_add("thermal", metrics_kobj);
	if (!secondary_sysfs_folder) {
		pr_err("Failed to create secondary sysfs folder!\n");
		return -ENOMEM;
	}
	tr_by_group_kobj = kobject_create_and_add("tr_by_group", secondary_sysfs_folder);
	if (!tr_by_group_kobj) {
		pr_err("Failed to create tr_by_group_kobj!\n");
		return -ENOMEM;
	}
	temp_residency_all_attr_group = (struct attribute_group) {
		.attrs = temp_residency_all_attrs
	};
	temp_abnormality_all_attr_group = (struct attribute_group) {
		.attrs = temp_abnormality_all_attrs,
		.name = "abnormality"
	};
	err = sysfs_create_group(secondary_sysfs_folder, &temp_residency_name_attr_group);
	if (err) {
		pr_err("failed to create temp_residency_name folder\n");
		return err;
	}
	pr_info("thermal_metrics driver initialized! :D\n");
	return err;
}
