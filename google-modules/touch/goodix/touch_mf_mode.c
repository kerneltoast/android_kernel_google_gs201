// SPDX-License-Identifier: GPL-2.0
/*
 * Sysfs APIs for Google Pixel devices.
 *
 * Copyright 2022 Google LLC.
 */

#include "touch_mf_mode.h"

/* Update a state machine used to toggle control of the touch IC's motion
 * filter.
 */
int touch_mf_update_state(struct touch_mf *tmf, u8 touches)
{
	/* Motion filter timeout, in milliseconds */
	const u32 mf_timeout_ms = 500;
	u8 next_state = TOUCH_MF_STATE_UNFILTERED;

	mutex_lock(&tmf->update_mutex);

	tmf->touches = touches;

	if (tmf->mode == TOUCH_MF_MODE_UNFILTERED) {
		next_state = TOUCH_MF_STATE_UNFILTERED;
	} else if (tmf->mode == TOUCH_MF_MODE_DYNAMIC) {
		/* Determine the next filter state. The motion filter is enabled
		 * by default and it is disabled while a single finger is
		 * touching the screen. If another finger is touched down or if
		 * a timeout expires, the motion filter is reenabled and remains
		 * enabled until all fingers are lifted.
		 */
		next_state = tmf->state;
		switch (tmf->state) {
		case TOUCH_MF_STATE_FILTERED:
			if (touches == 1) {
				next_state = TOUCH_MF_STATE_UNFILTERED;
				tmf->downtime = ktime_get();
			}
			break;
		case TOUCH_MF_STATE_UNFILTERED:
			if (touches == 0) {
				next_state = TOUCH_MF_STATE_FILTERED;
			} else if (touches > 1 ||
				   ktime_after(ktime_get(),
					   ktime_add_ms(tmf->downtime,
						   mf_timeout_ms))) {
				next_state = TOUCH_MF_STATE_LOCKED;
			}
			break;
		case TOUCH_MF_STATE_LOCKED:
			if (touches == 0) {
				next_state = TOUCH_MF_STATE_FILTERED;
			}
			break;
		}
	} else if (tmf->mode == TOUCH_MF_MODE_FILTERED) {
		next_state = TOUCH_MF_STATE_FILTERED;
	} else if (tmf->mode == TOUCH_MF_MODE_AUTO_REPORT) {
		next_state = TOUCH_MF_STATE_UNFILTERED;
	}

	/* Update continuously report switch if needed */
	if ((next_state == TOUCH_MF_STATE_UNFILTERED) !=
		(tmf->state == TOUCH_MF_STATE_UNFILTERED)) {
		if (tmf->set_continuously_report_enabled != NULL) {
			tmf->set_continuously_report_enabled(&tmf->pdev->dev,
				next_state == TOUCH_MF_STATE_UNFILTERED);
		}
	}

	tmf->state = next_state;

	mutex_unlock(&tmf->update_mutex);

	return 0;
}

int touch_mf_set_mode(struct touch_mf *tmf, enum touch_mf_mode mode)
{
	int ret = 0;
	if ((mode < TOUCH_MF_MODE_UNFILTERED) ||
		(mode > TOUCH_MF_MODE_AUTO_REPORT)) {
		ret = -EINVAL;
	} else {
		tmf->mode = mode;
		touch_mf_update_state(tmf, tmf->touches);
	}
	return ret;
}

int touch_mf_init(struct touch_mf *tmf)
{
	/* init motion filter mode */
	tmf->mode = TOUCH_MF_MODE_DYNAMIC;
	tmf->touches = 0;
	mutex_init(&tmf->update_mutex);
	return 0;
}
