/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Sysfs APIs for Google Pixel devices.
 *
 * Copyright 2022 Google LLC.
 */

#ifndef _TOUCH_APIS_H_
#define _TOUCH_APIS_H_

#if IS_ENABLED(CONFIG_TOUCHSCREEN_MOTION_FILTER)
#include "touch_mf_mode.h"
#endif
#if IS_ENABLED(CONFIG_GOOG_TOUCH_INTERFACE)
#include <goog_touch_interface.h>
#endif

enum scan_mode : u8 {
	SCAN_MODE_AUTO = 0,
	SCAN_MODE_NORMAL_ACTIVE,
	SCAN_MODE_NORMAL_IDLE,
	SCAN_MODE_LOW_POWER_ACTIVE,
	SCAN_MODE_LOW_POWER_IDLE,
	SCAN_MODE_MAX,
};

enum reset_result {
	RESET_RESULT_SUCCESS = 0,
	RESET_RESULT_FAIL = -1,
	RESET_RESULT_NOT_READY = -2,
	RESET_RESULT_NOT_SUPPORT = -3,
};

struct touch_apis_data {
	int reset_result;
	int scan_mode;
#if IS_ENABLED(CONFIG_TOUCHSCREEN_MOTION_FILTER)
	struct touch_mf *tmf;
	enum touch_mf_mode mf_mode;
#endif

	int (*get_fw_version)(struct device *dev, char *buf, size_t buf_size);
	int (*get_irq_enabled)(struct device *dev);
	int (*set_irq_enabled)(struct device *dev, bool enabled);
	bool (*is_scan_mode_supported)(struct device *dev, enum scan_mode mode);
	int (*ping)(struct device *dev);
	int (*hardware_reset)(struct device *dev);
	int (*software_reset)(struct device *dev);
	int (*set_scan_mode)(struct device *dev, enum scan_mode mode);
	int (*set_sensing_enabled)(struct device *dev, bool enabled);
#if IS_ENABLED(CONFIG_GOOG_TOUCH_INTERFACE) && IS_ENABLED(CONFIG_GTI_PM)
	bool (*get_wake_lock_state)(
		struct device *dev, enum gti_pm_wakelock_type type);
	int (*set_wake_lock_state)(
		struct device *dev, enum gti_pm_wakelock_type type, bool locked);
#endif
};

extern int touch_apis_init(struct device *dev, struct touch_apis_data *data);
extern void touch_apis_deinit(struct device *dev);

#endif
