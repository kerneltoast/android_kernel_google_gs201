/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Platform thermal driver for GXP.
 *
 * Copyright (C) 2021 Google LLC
 */
#ifndef __GXP_THERMAL_H__
#define __GXP_THERMAL_H__

#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/thermal.h>

#include "gxp-internal.h"
#include "gxp-pm.h"

#define GXP_COOLING_NAME "gxp-cooling"

struct gxp_thermal_manager {
	struct dentry *cooling_root;
	struct thermal_cooling_device *cdev;
	struct mutex lock;
	void *op_data;
	unsigned long cooling_state;
	unsigned long sysfs_req;
	unsigned int gxp_num_states;
	struct gxp_dev *gxp;
	bool thermal_suspended; /* GXP thermal suspended state */
};

/*
 * Internal structure to do the state/pwr mapping
 * state: kHz that AUR is running
 * power: mW that the state consume
 */
struct gxp_state_pwr {
	unsigned long state;
	u32 power;
};

struct gxp_thermal_manager *gxp_thermal_init(struct gxp_dev *gxp);

#endif /* __GXP_THERMAL_H__ */
