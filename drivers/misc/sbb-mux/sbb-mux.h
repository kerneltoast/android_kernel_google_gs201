/* SPDX-License-Identifier: GPL-2.0 */
/*
 * SideBand Bit Multiplexer (SBBM).
 *
 * This module exposes different signals to Kernel and User land, and updates
 * platform-specific GPIOs to track user-decided signals.
 *
 * Kernel code <through a function call to this driver> or userland <through
 * a sysfs node> updates a binary signal. If signal is currently tracked by a
 * GPIO, then update the GPIO accordingly.
 *
 * Signal tracking selection is done through a sysfs node.
 *
 * The driver's sysfs files can be found in /sys/kernel/sbb-mux.
 *
 * Copyright (C) 2020 Google, Inc.
 */

#ifndef __SBB_MUX_H__
#define __SBB_MUX_H__

#include <misc/sbbm.h>

/*
 * The different signal types. Signal types may impact who is (or is not)
 * allowed to modify them.
 */
enum sbb_signal_type {
	/*
	 * The signal represents a constant value, and may only be initialized
	 * by the sbb-mux driver.
	 */
	CONSTANT = 0,
	/*
	 * The signal has to come from the kernel, and can only be modified
	 * through the sbb-mux kernel API.
	 */
	KERNEL_DRIVEN,
	/*
	 * The signal has to come from userland, and can only be modified
	 * through sysfs nodes.
	 */
	USERLAND_DRIVEN,
};

/*
 * Signal definition structure.
 */
struct sbb_signal {
	enum sbbm_signal_id id;
	const char *name; /* Used as signal identifier in sysfs nodes. */
	const enum sbb_signal_type type;
};

/*
 * Extra information about signals supported by the sbb-mux driver.
 * This array has to remain perfectly mapped with the sbbm_signal_id enum from
 * include/misc/sbbm.h.
 */
const struct sbb_signal signals[SBB_SIG_NUM_SIGNALS] = {
	{ SBB_SIG_ZERO, "zero", CONSTANT },
	{ SBB_SIG_ONE, "one", CONSTANT },
	{ SBB_SIG_KERNEL_TEST, "kernel_test", KERNEL_DRIVEN },
	{ SBB_SIG_USERLAND_TEST, "userland_test", USERLAND_DRIVEN },
};

/*
 * Extended kobj_attribute structure.
 * Pointers to kobj_attribute objects are the only driver-customizable
 * component that get given back to us as part of the show/store callbacks
 * when users interact with our sysfs files. 'Overload' the structure to
 * add an 'id' field, that can be used by callbacks to identify which the
 * signal or GPIO file is being accessed.
 */
struct ext_kobj_attribute {
	struct kobj_attribute base_attribute;
	int id;
};

/*
 * Tracks a sysfs file's existence and attributes.
 */
struct sysfs_file {
	struct ext_kobj_attribute extended_attribute;
	int exists;
};

/*
 * Run-time information on signals.
 */
struct sbb_signal_tracker {
	/*
	 * Is set with its index in the signal_trackers array.
	 */
	enum sbbm_signal_id signal_id;

	/*
	 * Current value for the signal.
	 */
	atomic_t value;
	/*
	 * The number of times the signal was successfully toggled (i.e. the
	 * number of times the value changed).
	 */
	atomic_long_t toggle_count;

	/*
	 * The sysfs folder for the signal. Contains all the files listed
	 * below.
	 */
	struct kobject *sysfs_folder;
	/*
	 * The sysfs file exposing the signal's id to userland.
	 */
	struct sysfs_file id_file;
	/*
	 * The sysfs file exposing the signal's type to userland.
	 */
	struct sysfs_file type_file;
	/*
	 * The sysfs file exposing the signal's value to userland.
	 */
	struct sysfs_file value_file;

	/*
	 * The sysfs file exposing the signal's toggle_count to userland.
	 */
	struct sysfs_file toggle_count_file;
};

/*
 * Interface for updating Kernel signals.
 * 'value' has to be 0 or 1.
 * Returns 0 on success, -1 or -EINVAL on error.
 * Error cases are:
 * - the signal_id is invalid (-EINVAL)
 * - the signal's value was already set to the target value (-1).
 */
int sbbm_signal_update(enum sbbm_signal_id signal_id, int value);
EXPORT_SYMBOL(sbbm_signal_update);

/*
 * Setter for signal values.
 * Returns 0 on success, -1 on error.
 * Note: this is an internal function. The API function should first check
 * that the target signal is within bounds, and is indeed KERNEL_DRIVEN.
 */
static int sbb_signal_set_value(enum sbbm_signal_id signal_id, int value);

/*
 * Callback for reads to a signal's "id" file.
 * Will output the id as an base-10 ASCII string.
 */
static ssize_t sbb_signal_id_show(struct kobject *kobj,
				  struct kobj_attribute *attr, char *buf);

/*
 * Callback for reads to a signal's "type" file.
 * Will always returns one of [ "constant", "userland", "kernel" ].
 */
static ssize_t sbb_signal_type_show(struct kobject *kobj,
				    struct kobj_attribute *attr, char *buf);

/*
 * Callback for reads to a signal's "value" file.
 * Will return "0" or "1" when read.
 */
static ssize_t sbb_signal_value_show(struct kobject *kobj,
				     struct kobj_attribute *attr, char *buf);

/*
 * Callback for writes to a signal's "value" file.
 * Note that only USERLAND_DRIVEN files may be written to. Expected inputs are
 * "0" and "1". Will return -1 if the signal could not be writtne to (e.g. due
 * due trying to write the signal's current value).
 */
static ssize_t sbb_signal_value_store(struct kobject *kobj,
				      struct kobj_attribute *attr,
				      const char *buf, size_t buf_len);

/*
 * Callback for reads to a signal's "toggle_count" file.
 * Will output the toggle_count as an base-10 ASCII string.
 */
static ssize_t sbb_signal_toggle_count_show(struct kobject *kobj,
					    struct kobj_attribute *attr,
					    char *buf);

/*
 * Creates a new sysfs file in the target 'parent' sysfs folder.
 * Returns 0 on success, -1 on error.
 */
static int sbb_mux_init_sysfs_file(
	struct sysfs_file *file, struct kobject *parent, enum sbbm_signal_id id,
	const char *file_name, int mode,
	ssize_t (*show)(struct kobject *, struct kobj_attribute *, char *),
	ssize_t (*store)(struct kobject *, struct kobj_attribute *,
			 const char *, size_t));

/*
 * Cleans up (i.e. remove and resets) the target sysfs file.
 */
static void sbb_mux_cleanup_sysfs_file(struct kobject *parent,
				       struct sysfs_file *file);

/*
 * Initializes the target signal tracker.
 * A sysfs folder in sbb-mux's "signals" folder will be created as part of
 * this, as well as new sysfs files exposing the tracker's properties.
 */
static int sbb_mux_initialize_tracker(struct sbb_signal_tracker *tracker,
				      enum sbbm_signal_id signal_id);

/*
 * Cleans up the target signal tracker, deallocating resources as necessary.
 */
static void sbb_mux_cleanup_tracker(struct sbb_signal_tracker *tracker);

/*
 * Initializes all sbb-mux's sysfs nodes (sysfs folder hierarchy, and signal
 * tracker files).
 */
static int sbb_mux_initialize_sysfs_nodes(void);

/*
 * Cleans up all sbb-mux's sysfs nodes (undoing
 * sbb_mux_initialize_sysfs_nodes).
 */
static void sbb_mux_clean_up_sysfs_nodes(void);

/*
 * Driver entry point.
 */
static int __init sbb_mux_init(void);

/*
 * Driver exit point.
 */
static void __exit sbb_mux_exit(void);

#endif /* __SBB_MUX_H__ */
