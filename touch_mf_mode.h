/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Sysfs APIs for Google Pixel devices.
 *
 * Copyright 2022 Google LLC.
 */

#ifndef _TOUCH_MF_MODE_H_
#define _TOUCH_MF_MODE_H_

#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/time.h>

enum touch_mf_state {
	TOUCH_MF_STATE_UNFILTERED = 0,
	TOUCH_MF_STATE_FILTERED,
	TOUCH_MF_STATE_LOCKED,
};

enum touch_mf_mode {
	TOUCH_MF_MODE_UNFILTERED = 0,
	TOUCH_MF_MODE_DYNAMIC,
	TOUCH_MF_MODE_FILTERED,
	TOUCH_MF_MODE_AUTO_REPORT,
};

struct touch_mf {
	struct platform_device *pdev;
	enum touch_mf_mode mode;
	enum touch_mf_state state;
	struct mutex update_mutex;
	ktime_t downtime;
	int touches;

	int (*set_continuously_report_enabled)(
		struct device *dev, bool enabled);
};

extern int touch_mf_init(struct touch_mf *tmf);
extern int touch_mf_set_mode(struct touch_mf *tmf, enum touch_mf_mode mode);
extern int touch_mf_update_state(struct touch_mf *tmf, u8 touches);

#endif
