/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Google Battery Management System
 *
 * Copyright 2020 Google, LLC
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

#ifndef MAX1720X_BATTERY_H_
#define MAX1720X_BATTERY_H_

#include <linux/device.h>
#include <linux/regmap.h>
#include <linux/math64.h>

#include "maxfg_common.h"

#define EEPROM_SN	0
#define MAX1720X_SN	1

struct max17x0x_cache_data {
	struct maxfg_reg atom;
	u16 *cache_data;
};

int max1720x_get_capacity(struct i2c_client *client, int *iic_raw);
int max1720x_get_voltage_now(struct i2c_client *client, int *iic_raw);
int max17x0x_sw_reset(struct i2c_client *client);

/* */
#ifdef CONFIG_MAX1720X_REGLOG_LOG
static inline void max17x0x_reglog_log(struct maxfg_reglog *reglog,
				       unsigned int reg, u16 data, int rtn)
{
	if (!reglog)
		return;

	reglog->count[reg] += 1;
	if (rtn != 0) {
		reglog->errors[reg] += 1;
	} else {
		__set_bit(reg, reglog->valid);
		reglog->data[reg] = data;
	}

}

#else
static inline void max17x0x_reglog_log(struct maxfg_reglog *reglog,
				       unsigned int reg, u16 data, int rtn)
{

}
#endif

enum max1720x_drift_algo_version {
	MAX1720X_DA_VER_NONE = -1,	/* MW RC2 */
	MAX1720X_DA_VER_ORIG = 0,	/* MW A0, max1720x */
	MAX1720X_DA_VER_MWA1 = 1,	/* MW A1 RC1 */
	MAX1720X_DA_VER_MWA2 = 2,	/* MW A2 RC1 */
};

#define max1720x_check_drift_enabled(dd) \
		((dd)->algo_ver >= MAX1720X_DA_VER_ORIG)
#define max1720x_check_drift_on_soc(dd) \
		((dd)->algo_ver == MAX1720X_DA_VER_MWA1)
#define max1720x_check_drift_delay(dd) \
		((dd)->algo_ver == MAX1720X_DA_VER_MWA1 ? 351 : 0)

/* fix to capacity estimation */
struct max1720x_drift_data {
	u16 rsense;
	enum max1720x_drift_algo_version algo_ver;

	u16 design_capacity;
	int cycle_band;
	int cycle_fade;
	int cycle_stable;
	int ini_rcomp0;
	int ini_tempco;
	int ini_filtercfg;
};

struct max1720x_dyn_filtercfg {
	s32 temp;
	s32 hysteresis;
	u16 curr_val;
	u16 default_val;
	u16 adjust_val;
	struct mutex lock;
	bool disable_dynamic_filtercfg;
};

extern int max1720x_fixup_comp(struct max1720x_drift_data *ddata,
			       struct maxfg_regmap *map,
			       int plugged);
extern int max1720x_fixup_dxacc(struct max1720x_drift_data *ddata,
				struct maxfg_regmap *map,
				int cycle_count,
				int plugged,
				int lsb);

#endif
