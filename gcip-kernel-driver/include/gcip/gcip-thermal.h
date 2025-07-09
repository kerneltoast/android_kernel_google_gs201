/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Thermal management support for GCIP devices.
 *
 * Copyright (C) 2023 Google LLC
 */

#ifndef __GCIP_THERMAL_H__
#define __GCIP_THERMAL_H__

#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/thermal.h>

#define GCIP_THERMAL_TABLE_SIZE_NAME "gcip-dvfs-table-size"
#define GCIP_THERMAL_TABLE_NAME "gcip-dvfs-table"
#define GCIP_THERMAL_MAX_NUM_STATES 16

enum gcip_thermal_voter {
	GCIP_THERMAL_COOLING_DEVICE,
	GCIP_THERMAL_SYSFS,
	GCIP_THERMAL_NOTIFIER_BLOCK,

	/* Keeps as the last entry for the total number of voters. */
	GCIP_THERMAL_MAX_NUM_VOTERS,
};

struct gcip_thermal {
	struct device *dev;
	struct thermal_cooling_device *cdev;
	struct notifier_block nb;
	struct dentry *dentry;
	struct gcip_pm *pm;

	/*
	 * Lock to protect the struct members listed below.
	 *
	 * Note that since the request of thermal state adjusting might happen during power state
	 * transitions (i.e., another thread calling gcip_thermal_restore_on_powering() with pm lock
	 * held), one must either use the non-blocking gcip_pm_get_if_powered() or make sure there
	 * won't be any new power transition after holding this thermal lock to prevent deadlock.
	 */
	struct mutex lock;
	unsigned long num_states;
	unsigned long state;
	unsigned long vote[GCIP_THERMAL_MAX_NUM_VOTERS];
	bool device_suspended;
	bool enabled;

	/* Private data. See struct gcip_thermal_args.*/
	void *data;

	/* Callbacks. See struct gcip_thermal_args. */
	int (*get_rate)(void *data, unsigned long *rate);
	int (*set_rate)(void *data, unsigned long rate);
	int (*control)(void *data, bool enable);
};

/* Arguments for devm_gcip_thermal_create. */
struct gcip_thermal_args {
	/* Device struct of GCIP device. */
	struct device *dev;
	/* GCIP power management. */
	struct gcip_pm *pm;
	/* Top-level debugfs directory for the device. */
	struct dentry *dentry;
	/* Name of the thermal cooling-device node in device tree. */
	const char *node_name;
	/* Thermal cooling device type for thermal_of_cooling_device_register() . */
	const char *type;
	/* Private data for callbacks listed below. */
	void *data;
	/*
	 * Callbacks listed below are called only if the device is powered and with the guarantee
	 * that there won't be any new power transition during the call (i.e., after
	 * gcip_pm_get_if_powered() succeeds or during the power up triggered by gcip_pm_get())
	 * to prevent deadlock since they are called with thermal lock held. See the note about
	 * thermal lock in struct gcip_thermal.
	 */
	/* Callback to get the device clock rate. */
	int (*get_rate)(void *data, unsigned long *rate);
	/*
	 * Callback to set the device clock rate.
	 * Might be called with pm lock held in gcip_thermal_restore_on_powering().
	 */
	int (*set_rate)(void *data, unsigned long rate);
	/*
	 * Callback to enable/disable the thermal control.
	 * Might be called with pm lock held in gcip_thermal_restore_on_powering().
	 */
	int (*control)(void *data, bool enable);
};

/* Gets the notifier_block struct for thermal throttling requests. */
struct notifier_block *gcip_thermal_get_notifier_block(struct gcip_thermal *thermal);
/* Allocates and initializes GCIP thermal struct. */
struct gcip_thermal *gcip_thermal_create(const struct gcip_thermal_args *args);
/* Destroys and frees GCIP thermal struct. */
void gcip_thermal_destroy(struct gcip_thermal *thermal);
/* Suspends the device due to thermal request. */
int gcip_thermal_suspend_device(struct gcip_thermal *thermal);
/* Resumes the device and restores previous thermal state. */
int gcip_thermal_resume_device(struct gcip_thermal *thermal);
/*
 * Checks whether the device is suspended by thermal.
 * Note that it's checked without thermal lock and state might change subsequently.
 */
bool gcip_thermal_is_device_suspended(struct gcip_thermal *thermal);
/*
 * Restores the previous thermal state.
 *
 * This function is designed to restore the thermal state during firmware load and thus it assumes
 * the caller will kept the block powered.
 */
int gcip_thermal_restore_on_powering(struct gcip_thermal *thermal);

#endif /* __GCIP_THERMAL_H__ */
