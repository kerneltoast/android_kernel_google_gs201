// SPDX-License-Identifier: GPL-2.0-only
/* thermal_genl.c
 *
 * Android Vendor Hook Support
 *
 * Copyright 2021 Google LLC
 */

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/thermal.h>
#include <uapi/linux/thermal.h>

#define MAX_NUM_TZ 100
/* List for the thermal_zone which is not IRQ capable */
static const char *non_wakeable_tz_list[MAX_NUM_TZ] = {
	"soc",
	"usbc-therm-adc",
	"battery_cycle"
};

void vh_thermal_pm_notify_suspend(void *data, struct thermal_zone_device *tz, int *irq_wakeable)
{
	int i;

	*irq_wakeable = 1;
	for (i = 0; i < ARRAY_SIZE(non_wakeable_tz_list); i++ ) {
		if (non_wakeable_tz_list[i] == NULL)
			return;

		if (!strncmp(tz->type, non_wakeable_tz_list[i], strlen(non_wakeable_tz_list[i]))) {
			*irq_wakeable = 0;
			return;
		}
	}
}
