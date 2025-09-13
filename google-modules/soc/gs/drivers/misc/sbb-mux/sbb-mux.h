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

#include <linux/mutex.h>
#include <linux/spinlock_types.h>
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
	{ SBB_SIG_ZERO, "0", CONSTANT },
	{ SBB_SIG_ONE, "1", CONSTANT },
	{ SBB_SIG_KERNEL_TEST, "kernel_test", KERNEL_DRIVEN },
	{ SBB_SIG_USERLAND_TEST, "userland_test", USERLAND_DRIVEN },
	{ SBB_SIG_USERLAND_GP0, "gp_region_0", USERLAND_DRIVEN },
	{ SBB_SIG_USERLAND_GP1, "gp_region_1", USERLAND_DRIVEN },
	{ SBB_SIG_USERLAND_GP2, "gp_region_2", USERLAND_DRIVEN },
	{ SBB_SIG_USERLAND_GP3, "gp_region_3", USERLAND_DRIVEN },
	{ SBB_SIG_KERNEL_GP0, "k_gp_region_0", KERNEL_DRIVEN },
	{ SBB_SIG_KERNEL_GP1, "k_gp_region_1", KERNEL_DRIVEN },
	{ SBB_SIG_KERNEL_GP2, "k_gp_region_2", KERNEL_DRIVEN },
	{ SBB_SIG_KERNEL_GP3, "k_gp_region_3", KERNEL_DRIVEN },
	{ SBB_SIG_APP_MISC0, "app_misc0", USERLAND_DRIVEN },
	{ SBB_SIG_APP_MISC1, "app_misc1", USERLAND_DRIVEN },
	{ SBB_SIG_APP_MISC2, "app_misc2", USERLAND_DRIVEN },
	{ SBB_SIG_APP_MISC3, "app_misc3", USERLAND_DRIVEN },
	{ SBB_SIG_APP_MISC4, "app_misc4", USERLAND_DRIVEN },
	{ SBB_SIG_APP_MISC5, "app_misc5", USERLAND_DRIVEN },
	{ SBB_SIG_APP_MISC6, "app_misc6", USERLAND_DRIVEN },
	{ SBB_SIG_APP_MISC7, "app_misc7", USERLAND_DRIVEN },
	{ SBB_SIG_APP_MISC8, "app_misc8", USERLAND_DRIVEN },
	{ SBB_SIG_APP_MISC9, "app_misc9", USERLAND_DRIVEN },
	{ SBB_SIG_CAMERA_MISC0, "cam_misc0", USERLAND_DRIVEN },
	{ SBB_SIG_CAMERA_MISC1, "cam_misc1", USERLAND_DRIVEN },
	{ SBB_SIG_CAMERA_MISC2, "cam_misc2", USERLAND_DRIVEN },
	{ SBB_SIG_CAMERA_MISC3, "cam_misc3", USERLAND_DRIVEN },
	{ SBB_SIG_CAMERA_MISC4, "cam_misc4", USERLAND_DRIVEN },
	{ SBB_SIG_CAMERA_MISC5, "cam_misc5", USERLAND_DRIVEN },
	{ SBB_SIG_CAMERA_MISC6, "cam_misc6", USERLAND_DRIVEN },
	{ SBB_SIG_CAMERA_MISC7, "cam_misc7", USERLAND_DRIVEN },
	{ SBB_SIG_CAMERA_MISC8, "cam_misc8", USERLAND_DRIVEN },
	{ SBB_SIG_CAMERA_MISC9, "cam_misc9", USERLAND_DRIVEN },
	{ SBB_SIG_UFS_ACTIVE_IDLE, "ufs_active_idle", KERNEL_DRIVEN },
	{ SBB_SIG_UFS_POWER_ON, "ufs_power_on", KERNEL_DRIVEN },
	{ SBB_SIG_UFS_IO_OUTSTANDING, "ufs_io_outstanding", KERNEL_DRIVEN },
	{ SBB_SIG_MODEM_CP2AP_WAKE_ISR, "cp2ap_wake_isr", KERNEL_DRIVEN },
	{ SBB_SIG_PCIE_LINK_STATE, "pcie_link_state", KERNEL_DRIVEN }
};

/*
 * Extended kobj_attribute structure.
 * Pointers to kobj_attribute objects are the only driver-customizable
 * component that get given back to us as part of the show/store callbacks
 * when users interact with our sysfs files. 'Overload' the structure to
 * add an 'id' field, that can be used by callbacks to identify which
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
	int value;
	/*
	 * The number of times the signal was successfully toggled (i.e. the
	 * number of times the value changed).
	 */
	unsigned long toggle_count;
	/*
	 * Mask containing the index bits for all GPIOs tracking the signal.
	 * E.g. 0x5 means that GPIOs #2 and #0 are tracking the signal, and
	 * need to be updated when the signal value changes.
	 */
	unsigned long assigned_gpios_mask;
	/*
	 * Lock protecting value, toggle_count and assigned_gpios_mask.
	 */
	spinlock_t lock;

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

	/*
	 * The sysfs file exposing the signal's assigned GPIOs to userland.
	 */
	struct sysfs_file assigned_gpios_file;
};

/*
 * Interface for updating Kernel signals.
 * 'value' has to be 0 or 1.
 * Returns 0 on success, -EINVAL on error.
 * Error cases are:
 * - the signal_id is invalid
 * - the signal's value was already set to the target value.
 */
int sbbm_signal_update(enum sbbm_signal_id signal_id, bool value);
EXPORT_SYMBOL(sbbm_signal_update);

/*
 * Finds the signal id for the specified signal name.
 * Returns -EINVAL if no match could be found.
 */
static int sbb_signal_find(const char *name);

/*
 * Setter for signal values. Input values can be 0, 1, or SBB_REFRESH_VALUE.
 * On SBB_REFRESH_VALUE, any GPIOs tracking the signal will have their value
 * refreshed. This is only useful for initialization purposes.
 * Always returns 0 to indicate successful operation (this function is not
 * designed to fail).
 * Note: this is an internal function, which does not check that the target
 * signal is within bounds or that it is indeed KERNEL_DRIVEN.
 */
static int sbb_signal_set_value(enum sbbm_signal_id signal_id, int value);
#define SBB_REFRESH_VALUE -1

/*
 * Refreshes GPIO values for all signals. Only needed right after initializing
 * the GPIO trackers array.
 */
static void sbb_gpio_refresh_all(void);

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
 * "0" and "1". Will return -EINVAL if the signal could not be written to (e.g.
 * due to trying to write a value other than 0 or 1).
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
 * Callback for reads to a signal's "assigned_gpios" file.
 * Will output the names of all GPIOs tracking the signal (each name on
 * a new line).
 */
static ssize_t sbb_signal_assigned_gpios_show(struct kobject *kobj,
					      struct kobj_attribute *attr,
					      char *buf);

/*
 * Creates a new sysfs file in the target 'parent' sysfs folder.
 * Returns 0 on success, -EEXIST or -EINVAL on error.
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
 * Run-time information on GPIOs.
 */
struct sbb_gpio_tracker {
	/*
	 * Is set with its index in the DTS 'gpios' array (which is also its
	 * index in the 'gpio_trackers' array).
	 */
	int id;
	/*
	 * The name of the GPIO, as specified in the DTS.
	 */
	const char *name;
	/*
	 * The GPIO id returned when calling get_gpio.
	 */
	int system_id;
	/*
	 * The GPIO's descriptor.
	 */
	struct gpio_desc *gd;

	/*
	 * The SBBM signal tracked by the GPIO.
	 */
	enum sbbm_signal_id tracked_signal;
	/*
	 * Lock protecting tracked_signal.
	 * This lock is *NOT* needed when updating the GPIO's value.
	 */
	struct mutex lock;

	/*
	 * The sysfs folder for the GPIO. Contains all the files listed
	 * below.
	 */
	struct kobject *sysfs_folder;
	/*
	 * The sysfs file exposing the GPIO's tracked signal to userland.
	 */
	struct sysfs_file tracked_signal_file;
	/*
	 * The sysfs file exposing the GPIO's value to userland.
	 */
	struct sysfs_file value_file;
};

/*
 * Initializes the target GPIO tracker.
 * A sysfs folder in sbb-mux's "gpios" folder will be created as part of
 * this, as well as new sysfs files exposing the GPIO's properties.
 * Returns -EINVAL in case of irrecoverable error, -EPROBE_DEFER otherwise.
 */
static int sbb_mux_initialize_gpio_tracker(struct sbb_gpio_tracker *tracker,
					   int gpio_id);

/*
 * Cleans up the target GPIO tracker, deallocating resources as necessary.
 */
static void sbb_mux_cleanup_gpio_tracker(struct sbb_gpio_tracker *tracker);

/*
 * Driver probe point: find allocated GPIOs and initialize GPIO trackers.
 */
static int sbb_mux_drv_probe(struct platform_device *dev);

/*
 * Undo any initialization work done in probe.
 */
static void sbb_mux_drv_undo_probe(struct sbb_gpio_tracker **gpio_trackers_ptr);

/*
 * Driver remove point: free up resources allocated during probe.
 */
static int sbb_mux_drv_remove(struct platform_device *dev);

/*
 * Driver entry point.
 */
static int __init sbb_mux_init(void);

/*
 * Driver exit point.
 */
static void __exit sbb_mux_exit(void);

#endif /* __SBB_MUX_H__ */
