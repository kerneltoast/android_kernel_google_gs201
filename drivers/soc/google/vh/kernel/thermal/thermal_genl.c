// SPDX-License-Identifier: GPL-2.0-only
/* thermal_genl.c
 *
 * Android Vendor Hook Support
 *
 * Copyright 2021 Google LLC
 */

#include <linux/kernel.h>
#include <uapi/linux/thermal.h>

#define NUM_EVENT 1
static const int thermal_genl_enable[NUM_EVENT] = {
	THERMAL_GENL_EVENT_TZ_TRIP_UP
};

void vh_enable_thermal_genl_check(void *data, int event, int *enable_thermal_genl)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(thermal_genl_enable); i++) {
		if (thermal_genl_enable[i] == event) {
			*enable_thermal_genl = 1;
			return;
		}
	}
	*enable_thermal_genl = 0;
}
