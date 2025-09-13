// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2023, Google LLC
 *
 * Type-C port cooling device.
 */

#ifndef GOOGLE_USB_THERMAL_VOTER_H_
#define GOOGLE_USB_THERMAL_VOTER_H_


#define USB_THROTTLE_VOTABLE "USB_THROTTLE"

enum {
	USB_RESUMED,
	USB_SUSPENDED
};

#endif  // GOOGLE_USB_THERMAL_VOTER_H_
