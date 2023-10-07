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

#define MAX1720X_GAUGE_TYPE	0
#define MAX1730X_GAUGE_TYPE	1
#define MAX_M5_GAUGE_TYPE	2

#define EEPROM_SN	0
#define MAX1720X_SN	1

/* multiply by 2 when task period = 351 ms */
static inline int reg_to_micro_amp_h(s16 val, u16 rsense, int lsb)
{
	/* LSB: 5.0μVh/RSENSE ; Rsense LSB is 10μΩ */
	return div_s64((s64) val * 500000, rsense) * lsb;
}

/* divide by 2 when task period = 351 ms */
static inline s16 micro_amp_h_to_reg(int val, u16 rsense, int lsb)
{
	/* LSB: 5.0μVh/RSENSE ; Rsense LSB is 10μΩ */
	return div_s64((s64)(val / lsb) * rsense, 500000);
}

static inline int reg_to_micro_volt(u16 val)
{
	/* LSB: 0.078125mV */
	return div_u64((u64) val * 78125, 1000);
}

enum max17x0x_reg_tags {
	MAX17X0X_TAG_avgc,
	MAX17X0X_TAG_cnfg,
	MAX17X0X_TAG_mmdv,
	MAX17X0X_TAG_vcel,
	MAX17X0X_TAG_temp,
	MAX17X0X_TAG_curr,
	MAX17X0X_TAG_mcap,
	MAX17X0X_TAG_avgr,
	MAX17X0X_TAG_vfsoc,
	MAX17X0X_TAG_vfocv,

	MAX17X0X_TAG_BCNT,
	MAX17X0X_TAG_SNUM,
	MAX17X0X_TAG_HSTY,
	MAX17X0X_TAG_BCEA,
	MAX17X0X_TAG_rset,
	MAX17X0X_TAG_BRES,
};

enum max17x0x_reg_types {
	GBMS_ATOM_TYPE_MAP = 0,
	GBMS_ATOM_TYPE_REG = 1,
	GBMS_ATOM_TYPE_ZONE = 2,
	GBMS_ATOM_TYPE_SET = 3,
};

/* this is a map for u16 registers */
#define ATOM_INIT_MAP(...)			\
	.type = GBMS_ATOM_TYPE_MAP,		\
	.size = 2 * sizeof((u8[]){__VA_ARGS__}),\
	.map = (u8[]){__VA_ARGS__}

#define ATOM_INIT_REG16(r)		\
	.type = GBMS_ATOM_TYPE_REG,	\
	.size = 2,			\
	.reg = r

#define ATOM_INIT_ZONE(start, sz)	\
	.type = GBMS_ATOM_TYPE_ZONE,	\
	.size = sz,			\
	.base = start

/* a set has no storage and cannot be used in load/store */
#define ATOM_INIT_SET(...)		\
	.type = GBMS_ATOM_TYPE_SET,	\
	.size = 0,			\
	.map = (u8[]){__VA_ARGS__}

#define ATOM_INIT_SET16(...)		\
	.type = GBMS_ATOM_TYPE_SET,	\
	.size = 0,			\
	.map16 = (u16[]){__VA_ARGS__}

struct max17x0x_reg {
	int type;
	int size;
	union {
		unsigned int base;
		unsigned int reg;
		const u16 *map16;
		const u8 *map;
	};
};

struct max17x0x_cache_data {
	struct max17x0x_reg atom;
	u16 *cache_data;
};

#define NB_REGMAP_MAX 256

struct max17x0x_reglog {
	u16 data[NB_REGMAP_MAX];
	DECLARE_BITMAP(valid, NB_REGMAP_MAX);
	int errors[NB_REGMAP_MAX];
	int count[NB_REGMAP_MAX];
};

struct max17x0x_regtags {
	const struct max17x0x_reg *map;
	unsigned int max;
};

struct max17x0x_regmap {
	struct regmap *regmap;
	struct max17x0x_regtags regtags;
	struct max17x0x_reglog *reglog;
};

int max1720x_get_capacity(struct i2c_client *client, int *iic_raw);
int max1720x_get_voltage_now(struct i2c_client *client, int *iic_raw);
int max17x0x_sw_reset(struct i2c_client *client);

/* */
#ifdef CONFIG_MAX1720X_REGLOG_LOG
static inline void max17x0x_reglog_log(struct max17x0x_reglog *reglog,
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
static inline void max17x0x_reglog_log(struct max17x0x_reglog *reglog,
				       unsigned int reg, u16 data, int rtn)
{

}
#endif

static inline int max17x0x_regmap_read(const struct max17x0x_regmap *map,
				       unsigned int reg,
				       u16 *val,
				       const char *name)
{
	int rtn;
	unsigned int tmp;

	if (!map->regmap) {
		pr_err("Failed to read %s, no regmap\n", name);
		return -EIO;
	}

	rtn = regmap_read(map->regmap, reg, &tmp);
	if (rtn)
		pr_err("Failed to read %s\n", name);
	else
		*val = tmp;

	return rtn;
}

#define REGMAP_READ(regmap, what, dst) \
	max17x0x_regmap_read(regmap, what, dst, #what)

static inline int max17x0x_regmap_write(const struct max17x0x_regmap *map,
				       unsigned int reg,
				       u16 data,
				       const char *name)
{
	int rtn;

	if (!map->regmap) {
		pr_err("Failed to write %s, no regmap\n", name);
		return -EIO;
	}

	rtn = regmap_write(map->regmap, reg, data);
	if (rtn)
		pr_err("Failed to write %s\n", name);

	max17x0x_reglog_log(map->reglog, reg, data, rtn);

	return rtn;
}

#define REGMAP_WRITE(regmap, what, value) \
	max17x0x_regmap_write(regmap, what, value, #what)

#define WAIT_VERIFY	(10 * USEC_PER_MSEC) /* 10 msec */
static inline int max1720x_regmap_writeverify(const struct max17x0x_regmap *map,
					unsigned int reg,
					u16 data,
					const char *name)
{
	int tmp, ret, retries;

	if (!map->regmap) {
		pr_err("Failed to write %s, no regmap\n", name);
		return -EINVAL;
	}

	for (retries = 3; retries > 0; retries--) {
		ret = regmap_write(map->regmap, reg, data);
		if (ret < 0)
			continue;

		usleep_range(WAIT_VERIFY, WAIT_VERIFY + 100);

		ret = regmap_read(map->regmap, reg, &tmp);
		if (ret < 0)
			continue;

		if (tmp == data)
			return 0;
	}

	return -EIO;
}

#define REGMAP_WRITE_VERIFY(regmap, what, value) \
	max1720x_regmap_writeverify(regmap, what, value, #what)

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
			       struct max17x0x_regmap *map,
			       int plugged);
extern int max1720x_fixup_dxacc(struct max1720x_drift_data *ddata,
				struct max17x0x_regmap *map,
				int cycle_count,
				int plugged,
				int lsb);

#endif
