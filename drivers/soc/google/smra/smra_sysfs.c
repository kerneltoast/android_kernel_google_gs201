// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2024 Google LLC
 */

#define pr_fmt(fmt) "smra_sysfs: " fmt

#include <linux/kobject.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/sysfs.h>

#include "smra_core.h"
#include "smra_sysfs.h"

extern struct kobject *vendor_mm_kobj;
static struct kobject *smra_parent_kobj;
static struct kobject smra_kobj;

extern atomic64_t smra_buffer_overflow_cnt;

DEFINE_MUTEX(smra_sysfs_lock);
struct smra_config smra_config = {
	.recording_on = false,
	.buffer_has_trace = false,
	.merge_threshold = SMRA_DEFAULT_MERGE_THRESHOLD,
	.buffer_size = SMRA_DEFAULT_BUFFER_SIZE,
	.nr_targets = 0,
	.target_pids = { [0 ... SMRA_MAX_TARGET_CNT - 1] = -1 },
};

/*
 * There are three possible states after user interaction:
 * 1. The init/reset state: recording_on = false, buffer_has_trace = false
 * 2. User starts recording: recording_on = true, buffer_has_trace = false
 * 3. User stops recording: recording_on = false, buffer_has_trace = true
 * After finished recording, users should reset to the init/reset state by
 * echo 1 > /sys/kernel/vendor_mm/smra/reset
 */
static inline bool smra_is_reset(void)
{
	return !smra_config.recording_on && !smra_config.buffer_has_trace;
}

#define SMRA_ATTR_RO(_name) \
	static struct kobj_attribute _name##_attr = __ATTR_RO(_name)

#define SMRA_ATTR_WO(_name) \
	static struct kobj_attribute _name##_attr = __ATTR_WO(_name)

#define SMRA_ATTR_RW(_name) \
	static struct kobj_attribute _name##_attr = __ATTR_RW(_name)

static ssize_t buffer_overflow_cnt_show(struct kobject *kobj,
					struct kobj_attribute *attr,
					char *buf)
{
	return sysfs_emit(buf, "%llu\n",
			  atomic64_read(&smra_buffer_overflow_cnt));
}
SMRA_ATTR_RO(buffer_overflow_cnt);

static ssize_t buffer_size_show(struct kobject *kobj,
				struct kobj_attribute *attr,
				char *buf)
{
	int val;

	mutex_lock(&smra_sysfs_lock);
	val = smra_config.buffer_size;
	mutex_unlock(&smra_sysfs_lock);

	return sysfs_emit(buf, "%d\n", val);
}

static ssize_t buffer_size_store(struct kobject *kobj,
				 struct kobj_attribute *attr,
				 const char *buf, size_t len)
{
	int size;

	mutex_lock(&smra_sysfs_lock);
	if (!smra_is_reset()) {
		pr_warn("Cannot change buffer_size when busy, please make sure "
			"recording is finished and then reset first\n");
		mutex_unlock(&smra_sysfs_lock);
		return -EINVAL;
	}

	if (kstrtoint(buf, 10, &size)) {
		pr_warn("Receive invalid buffer size\n");
		mutex_unlock(&smra_sysfs_lock);
		return -EINVAL;
	}

	smra_config.buffer_size = size;
	mutex_unlock(&smra_sysfs_lock);

	return len;
}
SMRA_ATTR_RW(buffer_size);

static ssize_t merge_threshold_show(struct kobject *kobj,
				    struct kobj_attribute *attr,
				    char *buf)
{
	s64 val;

	mutex_lock(&smra_sysfs_lock);
	val = smra_config.merge_threshold;
	mutex_unlock(&smra_sysfs_lock);

	return sysfs_emit(buf, "%lld\n", val);
}

static ssize_t merge_threshold_store(struct kobject *kobj,
				     struct kobj_attribute *attr,
				     const char *buf, size_t len)
{

	s64 threshold;

	mutex_lock(&smra_sysfs_lock);
	if (!smra_is_reset()) {
		pr_warn("Cannot change merge_threshold when busy, please make "
			"sure recording is finished and then reset first\n");
		mutex_unlock(&smra_sysfs_lock);
		return -EINVAL;
	}

	if (kstrtos64(buf, 10, &threshold)) {
		pr_warn("Receive invalid merge threshold");
		mutex_unlock(&smra_sysfs_lock);
		return -EINVAL;
	}

	smra_config.merge_threshold = threshold;
	mutex_unlock(&smra_sysfs_lock);

	return len;
}
SMRA_ATTR_RW(merge_threshold);

static ssize_t target_pids_show(struct kobject *kobj,
			       struct kobj_attribute *attr,
			       char *buf)
{
	char pid_buf[MAX_PID_LEN];
	char pid_str[MAX_PID_LEN * SMRA_MAX_TARGET_CNT] = "";
	int i;

	mutex_unlock(&smra_sysfs_lock);
	for (i = 0; i < smra_config.nr_targets; i++) {
		snprintf(pid_buf, MAX_PID_LEN, "(%d) ",
			 smra_config.target_pids[i]);
		strncat(pid_str, pid_buf, 10);
	}
	mutex_unlock(&smra_sysfs_lock);

	return sysfs_emit(buf, "%s\n", pid_str);
}

static ssize_t target_pids_store(struct kobject *kobj,
				struct kobj_attribute *attr,
				const char *buf, size_t len)
{
	char *tmp, *token;
	pid_t pids[] = { [0 ... SMRA_MAX_TARGET_CNT - 1] = -1 };
	int i = 0;

	mutex_lock(&smra_sysfs_lock);
	if (!smra_is_reset()) {
		pr_warn("Cannot change target_pids when busy, please make sure "
			"recording is finished and then reset first\n");
		mutex_unlock(&smra_sysfs_lock);
		return -EINVAL;
	}

	tmp = kmalloc(len, GFP_KERNEL);
	if (!tmp) {
		mutex_unlock(&smra_sysfs_lock);
		return -ENOMEM;
	}
	memcpy(tmp, buf, len);

	while ((token = strsep(&tmp, " "))) {
		if (i >= SMRA_MAX_TARGET_CNT ||
		    kstrtoint(token, 10, &pids[i++])) {
			kfree(tmp);
			pr_warn("Receive invalid targets %s\n", buf);
			mutex_unlock(&smra_sysfs_lock);
			return -EINVAL;
		}
	}

	kfree(tmp);
	smra_config.nr_targets = i;
	memcpy(smra_config.target_pids, pids, sizeof(pids));
	mutex_unlock(&smra_sysfs_lock);

	return len;
}
SMRA_ATTR_RW(target_pids);

static ssize_t reset_store(struct kobject *kobj,
			   struct kobj_attribute *attr,
			   const char *buf, size_t len)
{
	mutex_lock(&smra_sysfs_lock);
	if (smra_config.recording_on) {
		pr_warn("Cannot reset when busy, please make sure "
			"recording is finished\n");
		mutex_unlock(&smra_sysfs_lock);
		return -EINVAL;
	}

	smra_config.buffer_size = SMRA_DEFAULT_BUFFER_SIZE;
	smra_config.merge_threshold = SMRA_DEFAULT_MERGE_THRESHOLD;
	smra_config.nr_targets = 0;
	if (smra_config.buffer_has_trace) {
		smra_reset();
		smra_config.buffer_has_trace = false;
	}
	mutex_unlock(&smra_sysfs_lock);

	return len;
}
SMRA_ATTR_WO(reset);

static ssize_t recording_on_show(struct kobject *kobj,
				 struct kobj_attribute *attr,
				 char *buf)
{
	bool enable;

	mutex_lock(&smra_sysfs_lock);
	enable = smra_config.recording_on;
	mutex_unlock(&smra_sysfs_lock);

	return sysfs_emit(buf, "%d\n", enable);
}

static ssize_t recording_on_store(struct kobject *kobj,
				  struct kobj_attribute *attr,
				  const char *buf, size_t len)
{
	int err, recording_on;

	mutex_lock(&smra_sysfs_lock);
	if (kstrtoint(buf, 10, &recording_on) ||
	    (recording_on != 0 && recording_on != 1)) {
		mutex_unlock(&smra_sysfs_lock);
		return -EINVAL;
	}

	if (recording_on) {
		if (!smra_is_reset()) {
			pr_warn("Please reset before recording\n");
			mutex_unlock(&smra_sysfs_lock);
			return -EINVAL;
		}
		if (smra_config.target_pids[0] == -1 || smra_config.buffer_size <= 0) {
			pr_warn("Invalid target pids and buffer size\n");
			mutex_unlock(&smra_sysfs_lock);
			return -EINVAL;
		}
		err = smra_setup(smra_config.target_pids, smra_config.nr_targets,
				 smra_config.buffer_size);
		if (err) {
			pr_warn("Setup error: %d\n", err);
			mutex_unlock(&smra_sysfs_lock);
			return err;
		}
		smra_start();
		smra_config.recording_on = true;
	} else {
		if (!smra_config.recording_on) {
			mutex_unlock(&smra_sysfs_lock);
			return -EINVAL;
		}
		smra_stop();
		smra_config.recording_on = false;
		smra_config.buffer_has_trace = true;
	}
	mutex_unlock(&smra_sysfs_lock);

	return len;
}
SMRA_ATTR_RW(recording_on);

static void smra_kobj_release(struct kobject *obj)
{
	/* Nothing needs to be done */
}

static struct attribute *smra_attrs[] = {
	&buffer_overflow_cnt_attr.attr,
	&buffer_size_attr.attr,
	&merge_threshold_attr.attr,
	&target_pids_attr.attr,
	&reset_attr.attr,
	&recording_on_attr.attr,
	NULL,
};

static const struct attribute_group smra_attr_group = {
	.attrs = smra_attrs,
};

static const struct attribute_group *smra_attr_groups[] = {
	&smra_attr_group,
	NULL,
};

static struct kobj_type smra_ktype = {
	.release = smra_kobj_release,
	.sysfs_ops = &kobj_sysfs_ops,
	.default_groups = smra_attr_groups,
};

int __init smra_sysfs_init(void)
{
#if IS_ENABLED(CONFIG_VH_MM)
	smra_parent_kobj = vendor_mm_kobj;
#else
	smra_parent_kobj = kobject_create_and_add("smra", kernel_kobj);
#endif
	return kobject_init_and_add(&smra_kobj, &smra_ktype,
				    smra_parent_kobj, "smra");
}
