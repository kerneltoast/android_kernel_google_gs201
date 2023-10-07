/* SPDX-License-Identifier: GPL-2.0 */
/*
 * EdgeTPU thermal driver header.
 *
 * Copyright (C) 2020 Google, Inc.
 */
#ifndef __EDGETPU_THERMAL_H__
#define __EDGETPU_THERMAL_H__

#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/thermal.h>

#include "edgetpu-internal.h"

#define EDGETPU_COOLING_NAME "tpu_cooling"

struct edgetpu_thermal {
	struct device *dev;
	struct dentry *cooling_root;
	struct thermal_cooling_device *cdev;
	struct mutex lock;
	void *op_data;
	unsigned long cooling_state;
	unsigned long sysfs_req;
	unsigned int tpu_num_states;
	struct edgetpu_dev *etdev;
	bool thermal_suspended; /* TPU thermal suspended state */
};

struct edgetpu_state_pwr {
	unsigned long state;
	u32 power;
};

/*
 * Creates a managed edgetpu_thermal object.
 *
 * Returns -errno on error.
 */
struct edgetpu_thermal *devm_tpu_thermal_create(struct device *dev,
						struct edgetpu_dev *etdev);

/*
 * Marks the TPU is suspended and informs TPU device if it's powered.
 *
 * Returns 0 on success.
 */
int edgetpu_thermal_suspend(struct device *dev);
/*
 * Resumes the TPU from the suspend state and informs TPU CPU if it's powered.
 *
 * Returns 0 on success.
 */
int edgetpu_thermal_resume(struct device *dev);

/*
 * Checks whether device is thermal suspended.
 * Returns false if the thermal management is not supported.
 */
static inline bool edgetpu_thermal_is_suspended(struct edgetpu_thermal *thermal)
{
	if (!IS_ERR_OR_NULL(thermal))
		return thermal->thermal_suspended;
	return false;
}

#endif /* __EDGETPU_THERMAL_H__ */
