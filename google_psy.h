/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2018 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __GOOGLE_PSY_H_
#define __GOOGLE_PSY_H_

#include <linux/printk.h>
#include "gbms_power_supply.h"

static inline int gpsy_set_prop(struct power_supply *psy,
				      enum power_supply_property psp,
				      union power_supply_propval val,
				      const char *prop_name)
{
	int ret = 0;

	if (!psy)
		return -EINVAL;

	pr_debug("set %s for '%s' to %d\n", prop_name, psy->desc->name,
		 val.intval);

	ret = power_supply_set_property(psy, psp, &val);
	if (ret < 0) {
		pr_err("failed to set %s for '%s', ret=%d\n",
		       prop_name, psy->desc->name, ret);
		return ret;
	}

	return 0;
}

#define GPSY_SET_PROP(psy, psp, val) \
	gpsy_set_prop(psy, (enum power_supply_property)(psp), (union power_supply_propval) \
		{ .intval = (val) }, #psp)

static inline int gpsy_get_prop(struct power_supply *psy,
			       enum power_supply_property psp,
			       const char *prop_name,
			       int *err)
{
	union power_supply_propval val;
	int ret = 0;

	ret = (psy) ? power_supply_get_property(psy, psp, &val) : -EINVAL;
	if (err)
		*err = ret;
	if (ret < 0) {
		pr_err("failed to get %s from '%s', ret=%d\n",
		       prop_name, psy->desc->name, ret);
		return ret;
	}

	pr_debug("get %s for '%s' => %d\n", prop_name, psy->desc->name,
		 val.intval);

	return val.intval;
}

#define GPSY_GET_PROP(psy, psp) \
		gpsy_get_prop(psy, (enum power_supply_property)(psp), #psp, 0)
/* use GPSY_GET_INT_PROP() for properties that can be negative */
#define GPSY_GET_INT_PROP(psy, psp, err) \
		gpsy_get_prop(psy, (enum power_supply_property)(psp), #psp, err)

#endif	/* __GOOGLE_PSY_H_ */
