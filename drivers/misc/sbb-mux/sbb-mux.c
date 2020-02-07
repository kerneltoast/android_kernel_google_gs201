// SPDX-License-Identifier: GPL-2.0
/*
 * SideBand Bit Multiplexer (SBBM).
 *
 * Copyright (C) 2020 Google, Inc.
 */

#include <linux/atomic.h>
#include <linux/kobject.h>
#include <linux/module.h>

#include "sbb-mux.h"

static struct kobject *primary_sysfs_folder;
static struct kobject *gpios_sysfs_folder;
static struct kobject *signals_sysfs_folder;

static struct sbb_signal_tracker signal_trackers[SBB_SIG_NUM_SIGNALS] = { 0 };

static ssize_t sbb_signal_id_show(struct kobject *kobj,
				  struct kobj_attribute *attr, char *buf)
{
	enum sbbm_signal_id signal_id = ((struct ext_kobj_attribute *)attr)->id;
	return scnprintf(buf, PAGE_SIZE, "%d", signal_id);
}

static ssize_t sbb_signal_type_show(struct kobject *kobj,
				    struct kobj_attribute *attr, char *buf)
{
	const char *signal_type_strings[SBB_SIG_NUM_SIGNALS] = { "constant",
								 "kernel",
								 "userland" };
	int signal_id = ((struct ext_kobj_attribute *)attr)->id;
	int signal_type = signals[signal_id].type;

	return scnprintf(buf, PAGE_SIZE, signal_type_strings[signal_type]);
}

static ssize_t sbb_signal_value_show(struct kobject *kobj,
				     struct kobj_attribute *attr, char *buf)
{
	enum sbbm_signal_id signal_id = ((struct ext_kobj_attribute *)attr)->id;
	return scnprintf(buf, PAGE_SIZE, "%d",
			 atomic_read(&signal_trackers[signal_id].value));
}

int sbbm_signal_update(enum sbbm_signal_id signal_id, int value)
{
	if (signal_id < 0 || signal_id >= SBB_SIG_NUM_SIGNALS)
		return -EINVAL;

	return sbb_signal_set_value(signal_id, !!value);
}

static int sbb_signal_set_value(enum sbbm_signal_id signal_id, int value)
{
	struct sbb_signal_tracker *tracker = &signal_trackers[signal_id];

	/*
	 * TODO: add messages to trace. Should *NOT* call pr_*, as this could be
	 * called from an interrupt context down the line.
	 */

	if (atomic_xchg(&tracker->value, value) == value)
		return -1;

	atomic_long_inc(&tracker->toggle_count);

	return 0;
}

static ssize_t sbb_signal_value_store(struct kobject *kobj,
				      struct kobj_attribute *attr,
				      const char *buf, size_t buf_len)
{
	enum sbbm_signal_id signal_id = ((struct ext_kobj_attribute *)attr)->id;

	if (buf_len < 1) {
		pr_err("sbb-mux: Invalid store size of %d for signal %s!",
		       buf_len, signals[signal_id].name);
		return -1;
	}

	if (sbb_signal_set_value(signal_id, buf[0] == '1')) {
		pr_err("sbb-mux: Signal %s is already set to %d!",
		       signals[signal_id].name, buf[0] == '1');
		return -1;
	}

	return buf_len;
}

static ssize_t sbb_signal_toggle_count_show(struct kobject *kobj,
					    struct kobj_attribute *attr,
					    char *buf)
{
	enum sbbm_signal_id signal_id = ((struct ext_kobj_attribute *)attr)->id;
	return scnprintf(
		buf, PAGE_SIZE, "%d",
		atomic_long_read(&signal_trackers[signal_id].toggle_count));
}

static int sbb_mux_init_sysfs_file(
	struct sysfs_file *file, struct kobject *parent, enum sbbm_signal_id id,
	const char *file_name, int mode,
	ssize_t (*show)(struct kobject *, struct kobj_attribute *, char *),
	ssize_t (*store)(struct kobject *, struct kobj_attribute *,
			 const char *, size_t))
{
	if (file->exists) {
		pr_err("File %s already exists for parent %p!", file_name,
		       parent);
		return -1;
	}

	file->extended_attribute.base_attribute.attr.name = file_name;
	file->extended_attribute.base_attribute.attr.mode = mode;
	file->extended_attribute.base_attribute.show = show;
	file->extended_attribute.base_attribute.store = store;

	file->extended_attribute.id = id;

	file->exists = !sysfs_create_file(
		parent, &file->extended_attribute.base_attribute.attr);

	return !file->exists;
}

static void sbb_mux_cleanup_sysfs_file(struct kobject *parent,
				       struct sysfs_file *file)
{
	if (!file->exists)
		return;

	sysfs_remove_file(parent,
			  &file->extended_attribute.base_attribute.attr);
	file->exists = false;
}

static int sbb_mux_initialize_tracker(struct sbb_signal_tracker *tracker,
				      enum sbbm_signal_id signal_id)
{
	pr_info("sbb-mux: Initializing signal [%d, '%s']...", signal_id,
		signals[signal_id].name);

	tracker->signal_id = signal_id;

	tracker->sysfs_folder = kobject_create_and_add(signals[signal_id].name,
						       signals_sysfs_folder);
	if (!tracker->sysfs_folder) {
		pr_err("sbb-mux: Failed to create sysfs subfolder for "
		       "signal %s!",
		       signals[signal_id].name);
		return -1;
	}

	if (sbb_mux_init_sysfs_file(&tracker->id_file, tracker->sysfs_folder,
				    signal_id, "id", 0444, sbb_signal_id_show,
				    NULL)) {
		pr_err("Could not create 'id' sysfs file for signal %s!",
		       signals[signal_id].name);
		return -1;
	}

	if (sbb_mux_init_sysfs_file(&tracker->type_file, tracker->sysfs_folder,
				    signal_id, "type", 0444,
				    sbb_signal_type_show, NULL)) {
		pr_err("Could not create 'type' sysfs file for signal %s!",
		       signals[signal_id].name);
		return -1;
	}

	if (sbb_mux_init_sysfs_file(
		    &tracker->value_file, tracker->sysfs_folder, signal_id,
		    "value",
		    signals[signal_id].type == USERLAND_DRIVEN ? 0666 : 0444,
		    sbb_signal_value_show,
		    signals[signal_id].type == USERLAND_DRIVEN ?
			    sbb_signal_value_store :
			    NULL)) {
		pr_err("Could not create 'value' sysfs file for signal %s!",
		       signals[signal_id].name);
		return -1;
	}

	if (sbb_mux_init_sysfs_file(&tracker->toggle_count_file,
				    tracker->sysfs_folder, signal_id,
				    "toggle_count", 0444,
				    sbb_signal_toggle_count_show, NULL)) {
		pr_err("Could not create 'toggle_count' sysfs file for "
		       "signal %s!",
		       signals[signal_id].name);
		return -1;
	}

	return 0;
}

static void sbb_mux_cleanup_tracker(struct sbb_signal_tracker *tracker)
{
	if (!tracker->sysfs_folder)
		return;

	sbb_mux_cleanup_sysfs_file(tracker->sysfs_folder, &tracker->id_file);
	sbb_mux_cleanup_sysfs_file(tracker->sysfs_folder, &tracker->type_file);
	sbb_mux_cleanup_sysfs_file(tracker->sysfs_folder, &tracker->value_file);
	sbb_mux_cleanup_sysfs_file(tracker->sysfs_folder,
				   &tracker->toggle_count_file);

	kobject_put(tracker->sysfs_folder);
	tracker->sysfs_folder = NULL;
}

static int sbb_mux_initialize_sysfs_nodes(void)
{
	int i;

	if (primary_sysfs_folder) {
		pr_err("sbb-mux sysfs nodes already initialized!");
		return -1;
	}

	primary_sysfs_folder = kobject_create_and_add("sbb-mux", kernel_kobj);
	if (!primary_sysfs_folder) {
		pr_err("sbb-mux: Failed to create primary sysfs folder!");
		return -1;
	}

	gpios_sysfs_folder =
		kobject_create_and_add("gpios", primary_sysfs_folder);
	if (!gpios_sysfs_folder) {
		pr_err("sbb-mux: Failed to create gpios sysfs folder!");
		return -1;
	}

	signals_sysfs_folder =
		kobject_create_and_add("signals", primary_sysfs_folder);
	if (!signals_sysfs_folder) {
		pr_err("sbb-mux: Failed to create signals sysfs folder!");
		return -1;
	}

	for (i = 0; i < SBB_SIG_NUM_SIGNALS; i++) {
		if (sbb_mux_initialize_tracker(&signal_trackers[i], i))
			return -1;
	}

	return 0;
}

static void sbb_mux_clean_up_sysfs_nodes(void)
{
	int i;

	if (!primary_sysfs_folder)
		return;

	kobject_put(primary_sysfs_folder);
	primary_sysfs_folder = NULL;

	if (!signals_sysfs_folder)
		return;

	kobject_put(signals_sysfs_folder);
	signals_sysfs_folder = NULL;

	for (i = 0; i < SBB_SIG_NUM_SIGNALS; i++) {
		sbb_mux_cleanup_tracker(&signal_trackers[i]);
	}
}

static int __init sbb_mux_init(void)
{
	atomic_set(&signal_trackers[SBB_SIG_ONE].value, 1);

	if (sbb_mux_initialize_sysfs_nodes()) {
		pr_err("sbb-mux: Issue when initializing signal trackers, "
		       "could not load!\n");
		sbb_mux_clean_up_sysfs_nodes();
		return -1;
	}

	pr_info("sbb-mux: Registered! :D\n");

	return 0;
}

static void __exit sbb_mux_exit(void)
{
	sbb_mux_clean_up_sysfs_nodes();
	pr_info("sbb-mux: Unregistered! :(\n");
}

module_init(sbb_mux_init);
module_exit(sbb_mux_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Vincent Palomares");
MODULE_DESCRIPTION("SideBand Bit Multiplexer Driver");
