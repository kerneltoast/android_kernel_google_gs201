/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2019-2022 Google LLC
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

#define pr_fmt(fmt) KBUILD_MODNAME ": %s " fmt, __func__

#include <linux/ctype.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/regmap.h>
#include <misc/gvotable.h>
#include "gbms_power_supply.h"

#ifdef CONFIG_DEBUG_FS
# include <linux/debugfs.h>
# include <linux/seq_file.h>
#endif

struct max77729_chgr_data {
	struct device *dev;
	struct power_supply *psy;
	struct regmap *regmap;
	int irq_gpio;

	struct gvotable_election *mode_votable;
	struct gvotable_election *dc_suspend_votable;
	struct gvotable_election *dc_icl_votable;

	/* input_uv is the input voltage limit */
	int input_uv;

	bool input_suspend;
	bool online;

#ifdef CONFIG_DEBUG_FS
	struct dentry *de;
	u64 irq_count;
	u64 irq_seen;
#endif

	struct mutex io_lock;
};

enum max77729_chg_register {
	MAX77729_CHG_INT        = 0xB0,
	MAX77729_CHG_INT_MASK   = 0xB1,
	MAX77729_CHG_INT_OK     = 0xB2,
	MAX77729_CHG_DETAILS_00 = 0xB3,
	MAX77729_CHG_DETAILS_01 = 0xB4,
	MAX77729_CHG_DETAILS_02 = 0xB5,
	MAX77729_CHG_CNFG_00    = 0xB7,
	MAX77729_CHG_CNFG_01    = 0xB8,
	MAX77729_CHG_CNFG_02    = 0xB9,
	MAX77729_CHG_CNFG_03    = 0xBA,
	MAX77729_CHG_CNFG_04    = 0xBB,
	MAX77729_CHG_CNFG_05    = 0xBC,
	MAX77729_CHG_CNFG_06    = 0xBD,
	MAX77729_CHG_CNFG_07    = 0xBE,
	MAX77729_CHG_CNFG_08    = 0xBF,
	MAX77729_CHG_CNFG_09    = 0xC0,
	MAX77729_CHG_CNFG_10    = 0xC1,
	MAX77729_CHG_CNFG_11    = 0xC2,
	MAX77729_CHG_CNFG_12    = 0xC3,
};

/*
 * function to return bits:
 *   name: name used for function
 *   h: high value of bit range
 *   l: low value of bit range
 *   for single bit h == l
 *
 *   given BIT_RANGE_FUNCS(test, 6, 5) generates:
 *        _test_get(uint8_t r) and _test_set(uint8_t r, uint8_t v)
 */
#define BIT_RANGE_FUNCS(name, h, l) \
	static inline uint8_t _ ## name ## _set(uint8_t r, uint8_t v) \
	{ \
		return ((r & ~GENMASK(h, l)) | v << l); \
	} \
	\
	static inline uint8_t _ ## name ## _get(uint8_t r) \
	{ \
		return ((r & GENMASK(h, l)) >> l); \
	}

BIT_RANGE_FUNCS(chg_int_ok_aicl_ok,         7, 7)
BIT_RANGE_FUNCS(chg_int_ok_chgin_ok,        6, 6)
BIT_RANGE_FUNCS(chg_int_ok_wcin_ok,         5, 5)
BIT_RANGE_FUNCS(chg_int_ok_chg_ok,          4, 4)
BIT_RANGE_FUNCS(chg_int_ok_bat_ok,          3, 3)
BIT_RANGE_FUNCS(chg_int_ok_batp_ok,         2, 2)
BIT_RANGE_FUNCS(chg_int_ok_disqbat_ok,      1, 1)
BIT_RANGE_FUNCS(chg_int_ok_byp_ok,          0, 0)

BIT_RANGE_FUNCS(details_00_no_autoibus,     7, 7)
BIT_RANGE_FUNCS(details_00_chgin_dtls,      6, 5)
BIT_RANGE_FUNCS(details_00_wcin_dtls,       4, 3)
BIT_RANGE_FUNCS(details_00_spsn_dtls,       2, 1)
BIT_RANGE_FUNCS(details_00_nobat,           0, 0)

BIT_RANGE_FUNCS(details_01_treg,            7, 7)
BIT_RANGE_FUNCS(details_01_batt_dtls,       6, 4)
BIT_RANGE_FUNCS(details_01_chrg_dtls,       3, 0)

BIT_RANGE_FUNCS(details_02_rsrv,            7, 4)
BIT_RANGE_FUNCS(details_02_byp_dtls,        3, 0)

BIT_RANGE_FUNCS(cnfg_00_spsn_det_en,        7, 7)
BIT_RANGE_FUNCS(cnfg_00_disibs,             6, 6)
BIT_RANGE_FUNCS(cnfg_00_spr,                5, 5)
BIT_RANGE_FUNCS(cnfg_00_wdten,              4, 4)
BIT_RANGE_FUNCS(cnfg_00_mode,               3, 0)

BIT_RANGE_FUNCS(cnfg_01_pqen,               7, 7)
BIT_RANGE_FUNCS(cnfg_01_lsel,               6, 6)
BIT_RANGE_FUNCS(cnfg_01_rstrt,              5, 4)
BIT_RANGE_FUNCS(cnfg_01_recycle_en,         3, 3)
BIT_RANGE_FUNCS(cnfg_01_fchgtime,           2, 0)

BIT_RANGE_FUNCS(cnfg_02_otg_ilim,           7, 6)
BIT_RANGE_FUNCS(cnfg_02_chrgcc,             5, 0)

BIT_RANGE_FUNCS(cnfg_03_sys_track_dis,      7, 7)
BIT_RANGE_FUNCS(cnfg_03_auto_fship_mode_en, 6, 6)
BIT_RANGE_FUNCS(cnfg_03_to_time,            5, 3)
BIT_RANGE_FUNCS(cnfg_03_to_ith,             2, 0)

BIT_RANGE_FUNCS(cnfg_04_chg_cv_prm,         5, 0)

BIT_RANGE_FUNCS(cnfg_05_dis_ir_ctrl,        6, 4)
BIT_RANGE_FUNCS(cnfg_05_uno_ilim,           6, 4)
BIT_RANGE_FUNCS(cnfg_05_b2sovrc,            3, 0)

BIT_RANGE_FUNCS(cnfg_06_b2sovrc_dtc,        7, 7)
BIT_RANGE_FUNCS(cnfg_06_slowlx,             6, 5)
BIT_RANGE_FUNCS(cnfg_06_dis_aicl,           4, 4)
BIT_RANGE_FUNCS(cnfg_06_chgprot,            3, 2)
BIT_RANGE_FUNCS(cnfg_06_wdtclr,             1, 0)

BIT_RANGE_FUNCS(cnfg_07_wd_qbatoff,         7, 7)
BIT_RANGE_FUNCS(cnfg_07_regtemp,            6, 3)
BIT_RANGE_FUNCS(cnfg_07_fmbst,              2, 2)
BIT_RANGE_FUNCS(cnfg_07_fgsrc,              1, 1)
BIT_RANGE_FUNCS(cnfg_07_fship_mode,         0, 0)

BIT_RANGE_FUNCS(cnfg_08_rsvd,               7, 2)
BIT_RANGE_FUNCS(cnfg_08_fsw,                1, 0)

BIT_RANGE_FUNCS(cnfg_09_chg_en,             7, 7)
BIT_RANGE_FUNCS(cnfg_09_chgin_ilim,         6, 0)

BIT_RANGE_FUNCS(cnfg_10_inlim_clk,          7, 6)
BIT_RANGE_FUNCS(cnfg_10_wcin_ilim,          5, 0)

BIT_RANGE_FUNCS(cnfg_11_en_fg_ilim_ctrl,    7, 7)
BIT_RANGE_FUNCS(cnfg_11_vbypset,            6, 0)

BIT_RANGE_FUNCS(cnfg_12_spr,                7, 7)
BIT_RANGE_FUNCS(cnfg_12_wcinsel,            6, 6)
BIT_RANGE_FUNCS(cnfg_12_chginsel,           5, 5)
BIT_RANGE_FUNCS(cnfg_12_vchgin_reg,         4, 3)
BIT_RANGE_FUNCS(cnfg_12_wcin_reg,           2, 1)
BIT_RANGE_FUNCS(cnfg_12_diskip,             0, 0)

/* CHG_DETAILS_00:CHGIN_DTLS */
#define CHGIN_DTLS_VBUS_INVALID_VCHGIN_RISING       0x00
#define CHGIN_DTLS_VBUS_INVALID_VCHGIN_FALLING      0x01
#define CHGIN_DTLS_VBUS_INVALID_VCHGIN_OVLO         0x02
#define CHGIN_DTLS_VBUS_VALID                       0x04

/* CHG_DETAILS_01:CHG_DTLS */
#define CHGR_DTLS_DEAD_BATTERY_MODE                 0x00
#define CHGR_DTLS_FAST_CHARGE_CONST_CURRENT_MODE    0x01
#define CHGR_DTLS_FAST_CHARGE_CONST_VOLTAGE_MODE    0x02
#define CHGR_DTLS_TOP_OFF_MODE                      0x03
#define CHGR_DTLS_DONE_MODE                         0x04
#define CHGR_DTLS_TIMER_FAULT_MODE                  0x06
#define CHGR_DTLS_DETBAT_HIGH_SUSPEND_MODE          0x07
#define CHGR_DTLS_OFF_MODE                          0x08
#define CHGR_DTLS_OFF_HIGH_TEMP_MODE                0x0a
#define CHGR_DTLS_OFF_WATCHDOG_MODE                 0x0b

/* CHG_CNFG_00:MODE */
#define CHGR_MODE_ALL_OFF                           0x00
#define CHGR_MODE_ALL_OFF_1                         0x01
#define CHGR_MODE_ALL_OFF_2                         0x02
#define CHGR_MODE_ALL_OFF_3                         0x03
#define CHGR_MODE_BUCK_ON                           0x04
#define CHGR_MODE_CHGR_BUCK_ON                      0x05
#define CHGR_MODE_CHGR_BUCK_ON_6                    0x06
#define CHGR_MODE_CHGR_BUCK_ON_7                    0x07
#define CHGR_MODE_BOOST_UNO_ON                      0x08
#define CHGR_MODE_BOOST_ON                          0x09
#define CHGR_MODE_OTG_BOOST_ON                      0x0a
#define CHGR_MODE_RESERVED_B                        0x0b
#define CHGR_MODE_BUCK_BOOST_UNO_ON                 0x0c
#define CHGR_MODE_CHGR_BUCK_BOOST_UNO_ON            0x0d
#define CHGR_MODE_OTG_BUCK_BOOST_ON                 0x0e
#define CHGR_MODE_CHGR_OTG_BUCK_BOOST_ON            0x0f

/* CHG_CNFG_02 */
#define CHGCC_50_RANGE_MIN_STEP                     0x02
#define CHGCC_50_RANGE_MIN_UA                     100000
#define CHGCC_50_RANGE_INC_UA                      50000
#define CHGCC_50_RANGE_MAX_STEP                     0x3f
#define CHGCC_50_RANGE_MAX_UA                    3150000

/* CHG_CNFG_04 */
#define CHG_CV_PRM_CONT_100_RANGE_MIN_STEP          0x38
#define CHG_CV_PRM_CONT_100_RANGE_MIN_UV         3800000
#define CHG_CV_PRM_CONT_100_RANGE_MAX_STEP          0x39
#define CHG_CV_PRM_CONT_100_RANGE_MAX_UV         3900000
#define CHG_CV_PRM_CONT_100_RANGE_INC_UV          100000

#define CHG_CV_PRM_CONT_50_RANGE_MIN_STEP           0x00
#define CHG_CV_PRM_CONT_50_RANGE_MIN_UV          4000000
#define CHG_CV_PRM_CONT_50_RANGE_MAX_STEP           0x04
#define CHG_CV_PRM_CONT_50_RANGE_MAX_UV          4200000
#define CHG_CV_PRM_CONT_50_RANGE_INC_UV            50000

#define CHG_CV_PRM_CONT_10_RANGE_MIN_STEP           0x05
#define CHG_CV_PRM_CONT_10_RANGE_MIN_UV          4200000
#define CHG_CV_PRM_CONT_10_RANGE_MAX_STEP           0x23
#define CHG_CV_PRM_CONT_10_RANGE_MAX_UV          4500000
#define CHG_CV_PRM_CONT_10_RANGE_INC_UV            10000

/* CHG_CNFG_09:CHGIN_ILIM */
#define CHGIN_ILIM_25_RANGE_MIN_STEP                0x03
#define CHGIN_ILIM_25_RANGE_MIN_UA                100000
#define CHGIN_ILIM_25_RANGE_MAX_STEP                0x7F
#define CHGIN_ILIM_25_RANGE_INC_UA                 25000
#define CHGIN_ILIM_25_RANGE_MAX_UA               3200000

static inline int max77729_reg_read(struct max77729_chgr_data *data,
		uint8_t reg, uint8_t *val)
{
	int ret;
	int ival;
	struct regmap *regmap = data->regmap;

	ret = regmap_read(regmap, reg, &ival);
	if (!ret)
		*val = 0xFF & ival;

	return ret;
}

static inline int max77729_reg_write(struct max77729_chgr_data *data,
		uint8_t reg, uint8_t val)
{
	int ret;
	struct regmap *regmap = data->regmap;

	ret = regmap_write(regmap, reg, val);

	return ret;
}

static inline int max77729_reg_update(struct max77729_chgr_data *data,
		uint8_t reg, uint8_t msk, uint8_t val)
{
	int ret;
	unsigned tmp;
	struct regmap *regmap = data->regmap;

	mutex_lock(&data->io_lock);
	ret = regmap_read(regmap, reg, &tmp);
	if (!ret) {
		tmp &= ~msk;
		tmp |= val;
		ret = regmap_write(regmap, reg, tmp);
	}
	mutex_unlock(&data->io_lock);

	return ret;
}

static bool max77729_chg_is_reg(struct device *dev, unsigned int reg)
{
	return (reg >= 0xB0) && (reg <= 0xC3);
}

static const struct regmap_config max77729_chg_regmap_cfg = {
	.name = "max77729_charger",
	.reg_bits = 8,
	.val_bits = 8,
	.val_format_endian = REGMAP_ENDIAN_NATIVE,
	.max_register = 0xC3,
	.readable_reg = max77729_chg_is_reg,
	.volatile_reg = max77729_chg_is_reg,

};

static inline int chgr_x2y(int x, int min_x, int min_y, int slope)
{
	return x < min_x ? min_y : ((x - min_x) * slope + min_y);
}

static inline int chgr_y2x(int y, int min_x, int min_y, int m)
{
	return y < min_y ? min_x : ((y - min_y) / m + min_x);
}

/* return not online (no error) when not present */
static int max77729_is_online(struct max77729_chgr_data *data, int *online)
{
	uint8_t reg;

	*online = (max77729_reg_read(data, MAX77729_CHG_INT_OK, &reg) == 0) &&
		  (_chg_int_ok_wcin_ok_get(reg) ||
		  _chg_int_ok_chgin_ok_get(reg));

	return 0;
}


static int max77729_get_status(struct max77729_chgr_data *data, int *status)
{
	uint8_t val;
	int online, ret;

	ret = max77729_is_online(data, &online);
	if (ret < 0 || !online) {
		*status = POWER_SUPPLY_STATUS_DISCHARGING;
		return 0;
	}

	ret = max77729_reg_read(data, MAX77729_CHG_DETAILS_01, &val);
	if (ret < 0)
		return ret;

	switch (_details_01_chrg_dtls_get(val)) {
	case CHGR_DTLS_DEAD_BATTERY_MODE:
	case CHGR_DTLS_FAST_CHARGE_CONST_CURRENT_MODE:
	case CHGR_DTLS_FAST_CHARGE_CONST_VOLTAGE_MODE:
		*status = POWER_SUPPLY_STATUS_CHARGING;
		break;
	case CHGR_DTLS_TOP_OFF_MODE:
	case CHGR_DTLS_DONE_MODE:
		/* same as POWER_SUPPLY_PROP_CHARGE_DONE */
		*status = POWER_SUPPLY_STATUS_FULL;
		break;
	case CHGR_DTLS_TIMER_FAULT_MODE:
	case CHGR_DTLS_DETBAT_HIGH_SUSPEND_MODE:
	case CHGR_DTLS_OFF_MODE:
	case CHGR_DTLS_OFF_HIGH_TEMP_MODE:
	case CHGR_DTLS_OFF_WATCHDOG_MODE:
		*status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	default:
		*status = POWER_SUPPLY_STATUS_UNKNOWN;
		break;
	}

	return ret;
}

/*
 * right now only accept CHGR_MODE_
 * TODO: use a comparator that add/remove capabilities, might require some
 * changes to the votables implementation since the current vote will be
 * created based on the actual votes instead of being one of the vote.
 */
static int max77729_mode_comparator(void *l, void *r)
{
	const int a = *((int *) &l);
	const int b = *((int *) &r);
	int ret;

	if (a == CHGR_MODE_ALL_OFF) {
		ret = (b == CHGR_MODE_ALL_OFF) ? 0 : -1;
	} else if (b == CHGR_MODE_ALL_OFF) {
		ret = 1;
	} else {
		ret = gvotable_comparator_most_recent(l, r);
	}

	return ret;
}

static int max77729_reset_charger_state(struct max77729_chgr_data *data)
{
	uint8_t reg, reg_new;
	int ret;

	mutex_lock(&data->io_lock);
	ret = max77729_reg_read(data, MAX77729_CHG_CNFG_00, &reg);
	if (ret < 0) {
		dev_err(data->dev, "cannot read CNFG_00 (%d)\n", ret);
		goto unlock_done;
	}

	reg_new = _cnfg_00_mode_set(reg, CHGR_MODE_BUCK_ON);
	ret = max77729_reg_write(data, MAX77729_CHG_CNFG_00, reg_new);
	if (ret < 0) {
		dev_err(data->dev, "cannot disable charging (%d)\n", ret);
		goto unlock_done;
	}

	ret = max77729_reg_write(data, MAX77729_CHG_CNFG_00, reg);
	if (ret < 0) {
		dev_err(data->dev, "cannot restore charging (%d)\n", ret);
		goto unlock_done;
	}

unlock_done:
	mutex_unlock(&data->io_lock);
	return ret;
}

static int max77729_mode_callback(struct gvotable_election *el,
				  const char *reason, void *value)
{
	struct max77729_chgr_data *data = gvotable_get_data(el);
	int mode = (long)value;
	uint8_t reg, reg_new;
	int ret;

	switch (mode) {
	case CHGR_MODE_ALL_OFF:
	case CHGR_MODE_BUCK_ON:
	case CHGR_MODE_CHGR_BUCK_ON:
	case CHGR_MODE_BOOST_UNO_ON:
	case CHGR_MODE_BOOST_ON:
	case CHGR_MODE_OTG_BOOST_ON:
	case CHGR_MODE_BUCK_BOOST_UNO_ON:
	case CHGR_MODE_CHGR_BUCK_BOOST_UNO_ON:
	case CHGR_MODE_OTG_BUCK_BOOST_ON:
	case CHGR_MODE_CHGR_OTG_BUCK_BOOST_ON:
		break;
	default:
		dev_err(data->dev, "mode %d not supported\n", mode);
		return 0;
	}

	dev_info(data->dev, "Vote CHARGER_MODE %d reason=%s\n", mode,
		 reason ? reason : "");

	mutex_lock(&data->io_lock);
	ret = max77729_reg_read(data, MAX77729_CHG_CNFG_00, &reg);
	if (ret < 0) {
		dev_err(data->dev, "cannot read CNFG_00 (%d)\n", ret);
		goto unlock_done;
	}

	/*
	 * the test is somewhat redundant since the callaback is called only
	 * when the vote change
	 */
	reg_new = _cnfg_00_mode_set(reg, mode);
	if (reg_new != reg)
		ret = max77729_reg_write(data, MAX77729_CHG_CNFG_00, reg_new);

	if (ret < 0)
		dev_err(data->dev, "cannot set CNFG_00 (%d)\n", ret);

unlock_done:
	mutex_unlock(&data->io_lock);
	return 0;
}


static int max77729_dc_suspend_vote_callback(struct gvotable_election *el,
					     const char *reason, void *value)
{
	struct max77729_chgr_data *data = gvotable_get_data(el);
	int result = (long)value;

	/* TODO: disable WCIN */

	dev_info(data->dev, "DC_SUSPEND reason=%s, value=%d suspend=%d\n",
		reason ? reason : "", result, result >= 0);

	return 0;
}

static int max77729_dcicl_callback(struct gvotable_election *el,
				   const char *reason, void *value)
{
	struct max77729_chgr_data *data = gvotable_get_data(el);
	int dc_icl = (long)value;
	const bool suspend = dc_icl == 0;
	int ret;

	ret = gvotable_cast_long_vote(data->dc_suspend_votable,
				      "DC_ICL", 1, suspend);
	if (ret < 0)
		dev_err(data->dev, "cannot set DC_SUSPEND=%d (%d)\n",
			suspend, ret);

	return 0;
}

static int max77729_set_charge_enabled(struct max77729_chgr_data *data,
				       int enabled, const char *reason)
{
	return gvotable_cast_long_vote(data->mode_votable, reason,
				       CHGR_MODE_CHGR_BUCK_ON, enabled);
}

static int max77729_input_suspend(struct max77729_chgr_data *data,
				  bool enabled, const char *reason)
{
	data->input_suspend = enabled;
	return gvotable_cast_long_vote(data->mode_votable, reason,
				       CHGR_MODE_ALL_OFF, enabled);
}


static int max77729_set_ilim_max_ua(struct max77729_chgr_data *data, int ua)
{
	const bool suspend = ua == 0;
	uint8_t reg, reg_new;
	int ret, steps;

	if (ua < 0)
		return 0;

	ret = max77729_reg_read(data, MAX77729_CHG_CNFG_09, &reg);
	if (ret < 0)
		return ret;

	ua = min(ua, CHGIN_ILIM_25_RANGE_MAX_UA);

	steps = chgr_y2x(ua, CHGIN_ILIM_25_RANGE_MIN_STEP,
				CHGIN_ILIM_25_RANGE_MIN_UA,
				CHGIN_ILIM_25_RANGE_INC_UA);

	/* do not cache the values*/
	reg_new = _cnfg_09_chgin_ilim_set(reg, steps);
	ret = max77729_reg_write(data, MAX77729_CHG_CNFG_09, reg_new);
	if (ret == 0)
		ret = max77729_input_suspend(data, suspend, "ILIM");

	return ret;
}


static int max77729_get_ilim_max_ua(struct max77729_chgr_data *data, int *ua)
{
	uint8_t reg;
	int ret, steps;

	ret = max77729_reg_read(data, MAX77729_CHG_CNFG_09, &reg);
	if (ret < 0)
		return ret;

	steps = _cnfg_09_chgin_ilim_get(reg);
	*ua = chgr_x2y(steps, CHGIN_ILIM_25_RANGE_MIN_STEP,
		       CHGIN_ILIM_25_RANGE_MIN_UA, CHGIN_ILIM_25_RANGE_INC_UA);

	if (data->input_suspend)
		*ua = 0;

	return 0;
}

/*
 * TODO: now return the limit set in 09 since we only use this for irdrop
 * compensation and debug. We should really read this from the FG in the same
 * way the bootloader does.
 */
static int max77729_get_current_now_ua(struct max77729_chgr_data *data, int *ua)
{
	return max77729_get_ilim_max_ua(data, ua);
}

static int max77729_get_charge_voltage_max_uv(struct max77729_chgr_data *data, int *uv)
{
	uint8_t reg;
	int ret, steps;

	ret = max77729_reg_read(data, MAX77729_CHG_CNFG_04, &reg);
	if (ret < 0)
		return ret;

	steps = _cnfg_04_chg_cv_prm_get(reg);
	if (steps >= CHG_CV_PRM_CONT_100_RANGE_MIN_STEP) {
		*uv = chgr_x2y(steps, CHG_CV_PRM_CONT_100_RANGE_MIN_STEP,
				CHG_CV_PRM_CONT_100_RANGE_MIN_UV,
				CHG_CV_PRM_CONT_100_RANGE_INC_UV);
	} else if (steps >= CHG_CV_PRM_CONT_10_RANGE_MIN_STEP) {
		*uv = chgr_x2y(steps, CHG_CV_PRM_CONT_10_RANGE_MIN_STEP,
				CHG_CV_PRM_CONT_10_RANGE_MIN_UV,
				CHG_CV_PRM_CONT_10_RANGE_INC_UV);
	} else {
		*uv = chgr_x2y(steps, CHG_CV_PRM_CONT_50_RANGE_MIN_STEP,
				CHG_CV_PRM_CONT_50_RANGE_MIN_UV,
				CHG_CV_PRM_CONT_50_RANGE_INC_UV);
	}

	return ret;
}

static int max77729_set_charge_voltage_max_uv(struct max77729_chgr_data *data, int uv)
{
	uint8_t reg, reg_new;
	int ret, steps;

	if (uv < 0)
		return 0;

	mutex_lock(&data->io_lock);
	ret = max77729_reg_read(data, MAX77729_CHG_CNFG_04, &reg);
	if (ret < 0) {
		mutex_unlock(&data->io_lock);
		return ret;
	}

	if (uv < CHG_CV_PRM_CONT_50_RANGE_MIN_UV) {
		steps = chgr_y2x(uv, CHG_CV_PRM_CONT_100_RANGE_MIN_STEP,
				 CHG_CV_PRM_CONT_100_RANGE_MIN_UV,
				 CHG_CV_PRM_CONT_100_RANGE_INC_UV);
	} else if (uv < CHG_CV_PRM_CONT_10_RANGE_MIN_UV) {
		steps = chgr_y2x(uv, CHG_CV_PRM_CONT_50_RANGE_MIN_STEP,
				 CHG_CV_PRM_CONT_50_RANGE_MIN_UV,
				 CHG_CV_PRM_CONT_50_RANGE_INC_UV);
	} else {
		uv = min(uv, CHG_CV_PRM_CONT_10_RANGE_MAX_UV);

		steps = chgr_y2x(uv, CHG_CV_PRM_CONT_10_RANGE_MIN_STEP,
				 CHG_CV_PRM_CONT_10_RANGE_MIN_UV,
				 CHG_CV_PRM_CONT_10_RANGE_INC_UV);
	}

	/* do not cache the values*/
	reg_new = _cnfg_04_chg_cv_prm_set(reg, steps);
	ret = max77729_reg_write(data, MAX77729_CHG_CNFG_04, reg_new);

	mutex_unlock(&data->io_lock);
	return ret;
}

static int max77729_get_charge_enabled(struct max77729_chgr_data *data,
		int *enabled)
{
	int ret;
	const void *vote = (const void *)0;

	ret = gvotable_get_current_vote(data->mode_votable, &vote);
	if (ret < 0)
		return ret;

	switch ((uintptr_t)vote) {
	case CHGR_MODE_CHGR_BUCK_ON:
	case CHGR_MODE_CHGR_BUCK_BOOST_UNO_ON:
	case CHGR_MODE_CHGR_OTG_BUCK_BOOST_ON:
		*enabled = 1;
		break;
	default:
		*enabled = 0;
		break;
	}

	return ret;
}

/* 0 ua suspend charging using mode */
static int max77729_set_charge_current_max_ua(struct max77729_chgr_data *data, int ua)
{
	const int enabled = ua != 0;
	uint8_t reg;
	int ret;

	if (ua < 0)
		return 0;

	mutex_lock(&data->io_lock);
	/* CHG_EN (1<<7) is set in the BL */
	ret = max77729_reg_read(data, MAX77729_CHG_CNFG_02, &reg);
	if (ret == 0) {
		uint8_t reg_new;
		int steps;

		ua = min(ua, CHGCC_50_RANGE_MAX_UA);
		steps = chgr_y2x(ua, CHGCC_50_RANGE_MIN_STEP,
				 CHGCC_50_RANGE_MIN_UA,
				 CHGCC_50_RANGE_INC_UA);

		/* do not cache the values*/
		reg_new = _cnfg_02_chrgcc_set(reg, steps);
		ret = max77729_reg_write(data, MAX77729_CHG_CNFG_02, reg_new);
	}

	mutex_unlock(&data->io_lock);
	if (ret == 0)
		max77729_set_charge_enabled(data, enabled, "CC_MAX");

	return ret;
}

static int max77729_get_charge_current_max_ua(struct max77729_chgr_data *data,
					      int *ua)
{
	uint8_t reg;
	int ret;

	ret = max77729_reg_read(data, MAX77729_CHG_CNFG_02, &reg);
	if (ret < 0)
		return ret;

	if (reg == 0) {
		*ua = 0;
	} else {
		const int steps = _cnfg_02_chrgcc_get(reg);

		*ua = chgr_x2y(steps, CHGCC_50_RANGE_MIN_STEP,
			       CHGCC_50_RANGE_MIN_UA, CHGCC_50_RANGE_INC_UA);

	}

	return ret;
}


static int max77729_get_charge_type(struct max77729_chgr_data *data, int *type)
{
	int ret;
	uint8_t reg;

	ret = max77729_reg_read(data, MAX77729_CHG_DETAILS_01, &reg);
	if (ret < 0)
		return ret;

	switch(_details_01_chrg_dtls_get(reg)) {
	case CHGR_DTLS_DEAD_BATTERY_MODE:
		*type = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
		break;
	case CHGR_DTLS_FAST_CHARGE_CONST_CURRENT_MODE:
		*type = POWER_SUPPLY_CHARGE_TYPE_FAST;
		break;
	case CHGR_DTLS_FAST_CHARGE_CONST_VOLTAGE_MODE:
		*type = POWER_SUPPLY_CHARGE_TYPE_TAPER_EXT;
		break;
	/* This is really DONE, */
	case CHGR_DTLS_TOP_OFF_MODE:
		*type = POWER_SUPPLY_CHARGE_TYPE_TAPER_EXT;
		break;
	case CHGR_DTLS_DONE_MODE:
		*type = POWER_SUPPLY_CHARGE_TYPE_NONE;
		fallthrough;
	case CHGR_DTLS_TIMER_FAULT_MODE:
	case CHGR_DTLS_DETBAT_HIGH_SUSPEND_MODE:
	case CHGR_DTLS_OFF_MODE:
	case CHGR_DTLS_OFF_HIGH_TEMP_MODE:
	case CHGR_DTLS_OFF_WATCHDOG_MODE:
		*type = POWER_SUPPLY_CHARGE_TYPE_NONE;
		break;
	default:
		*type = POWER_SUPPLY_CHARGE_TYPE_NONE;
		ret = -EINVAL;
		break;
	}

	return ret;
}

/* TODO: return if AICL is running  */
static int max77729_get_current_limit(struct max77729_chgr_data *data,
		int *limited)
{
	*limited = 0;
	return 0;
}

static int max77729_get_present(struct max77729_chgr_data *data, int *present)
{
	*present = 1;
	return 0;
}

static enum power_supply_property max77729_psy_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_STATUS,
};

static int max77729_psy_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *pval)
{
	struct max77729_chgr_data *data = power_supply_get_drvdata(psy);
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		ret = max77729_is_online(data, &pval->intval);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		ret = max77729_get_present(data, &pval->intval);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = max77729_get_current_now_ua(data, &pval->intval);
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		ret = max77729_get_ilim_max_ua(data, &pval->intval);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		ret = max77729_get_charge_current_max_ua(data,
							 &pval->intval);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		pval->intval = data->input_uv;
		ret = 0;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		ret = max77729_get_charge_voltage_max_uv(data,
							 &pval->intval);
		break;
	case POWER_SUPPLY_PROP_STATUS:
		ret = max77729_get_status(data, &pval->intval);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		ret = max77729_get_charge_type(data, &pval->intval);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = max77729_get_charge_voltage_max_uv(data,
							 &pval->intval);
		break;

	default:
		dev_err(data->dev, "property (%d) unsupported.\n", psp);
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int max77729_psy_set_property(struct power_supply *psy,
				     enum power_supply_property psp,
				     const union power_supply_propval *pval)
{
	struct max77729_chgr_data *data =
		(struct max77729_chgr_data *)power_supply_get_drvdata(psy);
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		data->online = pval->intval;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		ret = max77729_set_ilim_max_ua(data, pval->intval);
		pr_info("ilim=%d (%d)\n", pval->intval, ret);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		ret = max77729_set_charge_current_max_ua(data,
							 pval->intval);
		pr_info("charge_current=%d (%d)\n", pval->intval, ret);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		data->input_uv = pval->intval;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		ret = max77729_set_charge_voltage_max_uv(data,
							 pval->intval);
		pr_info("charge_voltage=%d (%d)\n", pval->intval, ret);
		break;
	default:
		dev_err(data->dev, "unsupported property: %d\n", psp);
		ret = -EINVAL;
		break;
	};

	return ret;
}

static int max77729_psy_property_is_writable(struct power_supply *psy,
		enum power_supply_property psp)
{
	int writeable = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
	case POWER_SUPPLY_PROP_CURRENT_MAX:	/* ILIM */
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:	/* input voltage limit */
	case POWER_SUPPLY_PROP_ONLINE:
		writeable = 1;
		break;
	default:
		break;
	}

	return writeable;
}

static int max77729_gbms_psy_get_property(struct power_supply *psy,
					  enum gbms_property psp,
					  union gbms_propval *pval)
{
	struct max77729_chgr_data *data = power_supply_get_drvdata(psy);
	int enabled;
	int ret;

	switch (psp) {
	case GBMS_PROP_CHARGING_ENABLED:
		ret = max77729_get_charge_enabled(data, &pval->prop.intval);
		break;
	case GBMS_PROP_CHARGE_DISABLE:
		ret = max77729_get_charge_enabled(data, &enabled);
		if (ret == 0)
			pval->prop.intval = !enabled;
		break;
	case GBMS_PROP_INPUT_CURRENT_LIMITED:
		ret = max77729_get_current_limit(data, &pval->prop.intval);
		break;
	/* TODO: implement charger state, fix *_PROP_VOLTAGE_MAX */
	case GBMS_PROP_CHARGE_CHARGER_STATE:
		ret = -EINVAL;
		break;

	case GBMS_PROP_TAPER_CONTROL:
		ret = 0;
		break;
	default:
		pr_debug("%s: route to max77729_psy_get_property, psp:%d\n", __func__, psp);
		ret = -ENODATA;
		break;
	}

	return ret;
}

static int max77729_gbms_psy_set_property(struct power_supply *psy,
					  enum gbms_property psp,
					  const union gbms_propval *pval)
{
	struct max77729_chgr_data *data =
		(struct max77729_chgr_data *)power_supply_get_drvdata(psy);
	int ret = 0;

	switch (psp) {
	case GBMS_PROP_CHARGING_ENABLED:
		ret = max77729_set_charge_enabled(data, pval->prop.intval,
						  "USER");
		break;
	case GBMS_PROP_CHARGE_DISABLE:  /* ext */
		ret = max77729_set_charge_enabled(data, !pval->prop.intval,
						  "USER");
		break;
	case GBMS_PROP_TAPER_CONTROL:
		break;
	default:
		pr_debug("%s: route max77729_psy_set_property, psp:%d\n", __func__, psp);
		ret = -ENODATA;
		break;
	};

	return ret;
}

static int max77729_gbms_psy_property_is_writable(struct power_supply *psy,
						  enum gbms_property psp)
{
	int writeable = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
	case POWER_SUPPLY_PROP_CURRENT_MAX:	/* ILIM */
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:	/* input voltage limit */
	case GBMS_PROP_CHARGE_DISABLE:  /* ext */
	case GBMS_PROP_TAPER_CONTROL:
	case POWER_SUPPLY_PROP_ONLINE:
		writeable = 1;
		break;
	default:
		break;
	}

	return writeable;
}

static struct gbms_desc max77729_psy_desc = {
	.psy_dsc.name = "max77729-charger",
	.psy_dsc.type = POWER_SUPPLY_TYPE_UNKNOWN,
	.psy_dsc.properties = max77729_psy_props,
	.psy_dsc.num_properties = ARRAY_SIZE(max77729_psy_props),
	.psy_dsc.get_property = max77729_psy_get_property,
	.psy_dsc.set_property = max77729_psy_set_property,
	.psy_dsc.property_is_writeable = max77729_psy_property_is_writable,
	.get_property = max77729_gbms_psy_get_property,
	.set_property = max77729_gbms_psy_set_property,
	.property_is_writeable = max77729_gbms_psy_property_is_writable,
	.forward = true,
};


#ifdef CONFIG_DEBUG_FS

static int max77729_dbg_reset_charger_state(void *d, u64 val)
{
	struct max77729_chgr_data *data = d;
	max77729_reset_charger_state(data);
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(max77729_debug_reset_charger_state_fops,
			NULL, max77729_dbg_reset_charger_state, "%llu\n");

static int dbg_init_fs(struct max77729_chgr_data *data)
{
	data->de = debugfs_create_dir("max77729_chg", 0);
	if (!data->de)
		return -EINVAL;

	debugfs_create_u64("irq_count", 0444, data->de, &data->irq_count);
	debugfs_create_u64("irq_seen", 0444, data->de, &data->irq_seen);
	debugfs_create_file("chg_reset", 0200, data->de, data,
			    &max77729_debug_reset_charger_state_fops);

	return 0;
}

static void dbg_cleanup_fs(struct max77729_chgr_data *data)
{
	if (data->de)
		debugfs_remove_recursive(data->de);
	data->de = NULL;
}

#define DBG_INC(count) count++
#else /* CONFIG_DEBUG_FS */
static inline int dbg_init_fs(struct max77729_chgr_data *data)
{
	return 0;
}
static inline void dbg_remove_fs(struct max77729_chgr_data *data) { }
#define DBG_INC(count)
#endif

static irqreturn_t max77729_chgr_irq(int irq, void *d)
{
	struct max77729_chgr_data *data = (struct max77729_chgr_data *)d;
	uint8_t reg = 0;
	int ret;

	DBG_INC(data->irq_seen);

	/* reading the interrupt clears it */
	ret = max77729_reg_read(data, MAX77729_CHG_INT, &reg);
	pr_debug("IRQ reg=%x (%d)\n", reg, ret);
	if (ret < 0)
		dev_err_ratelimited(data->dev, "failed reading CHG_INT ret=%d\n",
				   ret);
	if (!reg)
		return IRQ_NONE;

	DBG_INC(data->irq_count);

	power_supply_changed(data->psy);
	return IRQ_HANDLED;
}

static int max77729_init_irq(struct i2c_client *client)
{
	struct max77729_chgr_data *data = i2c_get_clientdata(client);
	struct device *dev = &client->dev;
	int ret = 0;

	data->irq_gpio =
		of_get_named_gpio(dev->of_node, "max77729,irq-gpio", 0);
	if (data->irq_gpio < 0) {
		dev_err(dev, "failed get irq_gpio\n");
		return -EINVAL;
	}

	client->irq = gpio_to_irq(data->irq_gpio);
	ret = devm_request_threaded_irq(data->dev, client->irq, NULL,
					max77729_chgr_irq,
					IRQF_TRIGGER_LOW |
					IRQF_SHARED |
					IRQF_ONESHOT,
					dev_name(data->dev), data);
	if (ret == 0)
		enable_irq_wake(client->irq);

	return ret;
}

static int max77729_setup_votables(struct max77729_chgr_data *data)
{
	int ret;

	/* DC_SUSPEND votes on CHARGER_MODE */
	data->mode_votable =
		gvotable_create_int_election(NULL, max77729_mode_comparator,
					     max77729_mode_callback,
					     data);
	if (IS_ERR_OR_NULL(data->mode_votable)) {
		dev_err(data->dev, "Failed to initialize mode votable\n");
		ret = PTR_ERR(data->mode_votable);
		data->mode_votable = NULL;
		return ret;
	}

	gvotable_set_vote2str(data->mode_votable, gvotable_v2s_uint);
	gvotable_set_default(data->mode_votable, (void *)CHGR_MODE_BUCK_ON);
	gvotable_election_set_name(data->mode_votable, "CHARGER_MODE");

	/* DC_ICL votes on DC_SUSPEND */
	data->dc_suspend_votable =
		gvotable_create_bool_election(NULL,
					      max77729_dc_suspend_vote_callback,
					      data);
	if (IS_ERR_OR_NULL(data->dc_suspend_votable)) {
		ret = PTR_ERR(data->dc_suspend_votable);
		dev_err(data->dev, "no dc_suspend votable (%d)\n", ret);
		return ret;
	}

	gvotable_set_vote2str(data->dc_suspend_votable, gvotable_v2s_int);
	gvotable_election_set_name(data->dc_suspend_votable, "DC_SUSPEND");
	gvotable_use_default(data->dc_suspend_votable, true);

	data->dc_icl_votable =
		gvotable_create_int_election(NULL, gvotable_comparator_int_min,
					     max77729_dcicl_callback,
					     data);
	if (IS_ERR_OR_NULL(data->dc_icl_votable)) {
		ret = PTR_ERR(data->dc_icl_votable);
		dev_err(data->dev, "no dc_icl votable (%d)\n", ret);
		return ret;
	}

	gvotable_set_vote2str(data->dc_icl_votable, gvotable_v2s_uint);
	gvotable_set_default(data->dc_icl_votable, (void *)700000);
	gvotable_election_set_name(data->dc_icl_votable, "DC_ICL");
	gvotable_use_default(data->dc_icl_votable, true);

	return 0;
}

static int max77729_charger_probe(struct i2c_client *client,
				  const struct i2c_device_id *id)
{
	struct max77729_chgr_data *data;
	struct device *dev = &client->dev;
	struct power_supply_config chgr_psy_cfg = { };
	const char *psy_name;
	int ret = 0;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->dev = dev;
	i2c_set_clientdata(client, data);
	mutex_init(&data->io_lock);

	data->regmap = devm_regmap_init_i2c(client, &max77729_chg_regmap_cfg);
	if (IS_ERR(data->regmap)) {
		dev_err(dev, "Failed to initialize regmap\n");
		return -EINVAL;
	}

	ret = of_property_read_string(dev->of_node, "max77729,psy-name",
				      &psy_name);
	if (ret == 0)
		max77729_psy_desc.psy_dsc.name = devm_kstrdup(dev, psy_name,
						      GFP_KERNEL);

	chgr_psy_cfg.drv_data = data;
	chgr_psy_cfg.supplied_to = NULL;
	chgr_psy_cfg.num_supplicants = 0;
	data->psy = devm_power_supply_register(dev, &max77729_psy_desc.psy_dsc,
		&chgr_psy_cfg);
	if (IS_ERR(data->psy)) {
		dev_err(dev, "Failed to register psy rc = %ld\n",
			PTR_ERR(data->psy));
		goto exit;
	}

	ret = max77729_setup_votables(data);
	if (ret < 0)
		return -EINVAL;

	ret = max77729_reg_write(data, MAX77729_CHG_INT_MASK, 0x00);
	if (ret) {
		dev_err(dev, "failed to set intmask\n");
		goto exit;
	}

	ret = dbg_init_fs(data);
	if (ret)
		dev_err(dev, "Failed to initialize debug fs\n");

	if (max77729_init_irq(client) < 0) {
		dev_err(dev, "failed to initialize irq\n");
		goto exit;
	}

	dev_info(dev, "registered as %s\n", max77729_psy_desc.psy_dsc.name);

exit:
	return ret;
}

static void max77729_charger_remove(struct i2c_client *client)
{
	struct max77729_chgr_data *data = i2c_get_clientdata(client);

	gvotable_destroy_election(data->mode_votable);

	if (data->psy)
		power_supply_unregister(data->psy);

	dbg_cleanup_fs(data);
}

static const struct of_device_id max77729_charger_of_match_table[] = {
	{ .compatible = "maxim,max77729chrg"},
	{},
};
MODULE_DEVICE_TABLE(of, max77729_charger_of_match_table);

static const struct i2c_device_id max77729_id[] = {
	{"max77729_charger", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, max77729_id);

#if defined CONFIG_PM
static int max77729_charger_pm_suspend(struct device *dev)
{
	/* TODO: is there anything to do here? */
	return 0;
}

static int max77729_charger_pm_resume(struct device *dev)
{
	/* TODO: is there anything to do here? */
	return 0;
}
#endif

static const struct dev_pm_ops max77729_charger_pm_ops = {
	SET_LATE_SYSTEM_SLEEP_PM_OPS(
		max77729_charger_pm_suspend,
		max77729_charger_pm_resume)
};

static struct i2c_driver max77729_charger_i2c_driver = {
	.driver = {
		.name = "max77729-charger",
		.owner = THIS_MODULE,
		.of_match_table = max77729_charger_of_match_table,
#ifdef CONFIG_PM
		.pm = &max77729_charger_pm_ops,
#endif
	},
	.id_table = max77729_id,
	.probe    = max77729_charger_probe,
	.remove   = max77729_charger_remove,
};

module_i2c_driver(max77729_charger_i2c_driver);

MODULE_DESCRIPTION("Maxim 77729 Charger Driver");
MODULE_AUTHOR("Jim Wylder jwylder@google.com");
MODULE_AUTHOR("AleX Pelosi apelosi@google.com");
MODULE_LICENSE("GPL");
