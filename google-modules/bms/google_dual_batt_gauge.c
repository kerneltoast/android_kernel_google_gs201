/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2021 Google, LLC
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

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/kernel.h>
#include <linux/printk.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pm_runtime.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/debugfs.h>
#include <misc/gvotable.h>
#include "gbms_power_supply.h"
#include "google_bms.h"
#include "google_psy.h"

#define MAX(x, y)	((x) < (y) ? (y) : (x))
#define DUAL_FG_DELAY_INIT_MS	500
#define DUAL_FG_WORK_PERIOD_MS	10000
#define DUAL_BATT_TEMP_VOTER	"daul_batt_temp"

#define DUAL_BATT_BALANCE_VOTER	"dual_batt_balance"
#define DUAL_BATT_BALANCE_CC_ADJUST_STEP	100000  /* 100mA */
#define DUAL_BATT_BALANCE_FV_ADJUST_STEP	10000   /* 10mV */

#define DUAL_BATT_VSEC_OFFSET		50000
#define DUAL_BATT_VSEC_OFFSET_IDX	0
#define DUAL_BATT_VSEC_DEFAULT_RATIO	50

#define MONITOR_SOC_DIFF	5

#define REPORT_DEFAULT	0
#define REPORT_BASE	1
#define REPORT_SEC	2

static int debug_printk_prlog = LOGLEVEL_INFO;
#define logbuffer_prlog(p, level, fmt, ...)	\
	gbms_logbuffer_prlog(p->log, level, 0, debug_printk_prlog, fmt, ##__VA_ARGS__)

struct seq_soc_drop_wa {
	int seq_fg_delta_limit;
	int seq_fg_soc_limit;
	int report_soc_type;
};

struct dual_fg_drv {
	struct device *device;
	struct power_supply *psy;
	struct power_supply *batt_psy;

	const char *first_fg_psy_name;
	const char *second_fg_psy_name;

	struct power_supply *first_fg_psy;
	struct power_supply *second_fg_psy;

	struct mutex stats_lock;

	struct delayed_work init_work;
	struct delayed_work gdbatt_work;
	struct gvotable_election *fcc_votable;
	struct gvotable_election *fv_votable;

	struct gbms_chg_profile chg_profile;
	struct gbms_chg_profile base_profile;
	struct gbms_chg_profile sec_profile;

	struct logbuffer *log;

	struct notifier_block fg_nb;

	u32 battery_capacity;
	u32 base_capacity;
	u32 sec_capacity;
	int cc_max;
	int cc_balance_offset;
	int cc_balance_ratio;
	int fv_balance_offset;

	int base_charge_full;
	int sec_charge_full;

	bool init_complete;
	bool resume_complete;
	bool cable_in;

	u32 vsec_offset;
	u32 vsec_offset_max_idx;
	u32 vsec_ratio;
	u32 vbatt_ov_allowed_idx;

	int base_soc;
	int sec_soc;

	struct gbms_ce_tier_stats base_batt_stats;
	struct gbms_ce_tier_stats sec_batt_stats;
	union gbms_charger_state chg_state;
	ktime_t last_update;

	struct seq_soc_drop_wa seq_wa;
};

static int gdbatt_resume_check(struct dual_fg_drv *dual_fg_drv) {
	int ret = 0;

	pm_runtime_get_sync(dual_fg_drv->device);
	if (!dual_fg_drv->init_complete || !dual_fg_drv->resume_complete)
		ret = -EAGAIN;
	pm_runtime_put_sync(dual_fg_drv->device);

	return ret;
}

static enum power_supply_property gdbatt_fg_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CAPACITY,		/* replace with _RAW */
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,	/* used from gbattery */
	POWER_SUPPLY_PROP_CURRENT_AVG,		/* candidate for tier switch */
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_VOLTAGE_AVG,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_SERIAL_NUMBER,
};

static int gdbatt_get_temp(struct power_supply *fg_psy, int *temp)
{
	int err = 0;
	union power_supply_propval val;

	if (!fg_psy)
		return -EINVAL;

	err = power_supply_get_property(fg_psy,
					POWER_SUPPLY_PROP_TEMP, &val);
	if (err == 0)
		*temp = val.intval;

	return err;
}

static int gdbatt_select_temp_idx(struct gbms_chg_profile *profile, int temp)
{
	if (temp < profile->temp_limits[0] ||
	    temp > profile->temp_limits[profile->temp_nb_limits - 1])
		return -1;
	else
		return gbms_msc_temp_idx(profile, temp);
}

static int gdbatt_select_voltage_idx(struct gbms_chg_profile *profile, int vbatt)
{
	int vbatt_idx = 0;

	while (vbatt_idx < profile->volt_nb_limits - 1 &&
	       vbatt > profile->volt_limits[vbatt_idx])
		vbatt_idx++;

	return vbatt_idx;
}

static int gdbatt_oc_cc_offset(int ibase, int isec, int cc_base, int cc_sec, int now_offset, int step)
{
	int cc_offset_base = 0, cc_offset_sec = 0;

	if (cc_base < 0 || cc_sec < 0 || step <= 0) {
		pr_err("%s: invalid params, %d, %d, %d\n", __func__, cc_base, cc_sec, step);
		return 0;
	}

	if (ibase > cc_base)
		cc_offset_base = ((ibase - cc_base + step - 1) / step) * step;

	if (isec > cc_sec)
		cc_offset_sec = ((isec - cc_sec + step - 1) / step) * step;


	now_offset = ((cc_offset_base + cc_offset_sec) <= now_offset) ? (now_offset) :
									(now_offset + step);

	pr_debug("%s: %d, %d, %d", __func__, now_offset, cc_offset_base, cc_offset_sec);

	return now_offset;
}

static void gdbatt_check_current(struct dual_fg_drv *dual_fg_drv, int temp_idx, int vbat_idx)
{
	int ibase, isec, cc_base, cc_sec, cc_offset, next_cc_max, cc_lowerbd;
	int next_vbat_idx = vbat_idx + 1;
	struct gbms_chg_profile *profile = &dual_fg_drv->chg_profile;
	struct gbms_chg_profile *base_profile = &dual_fg_drv->base_profile;
	struct gbms_chg_profile *sec_profile = &dual_fg_drv->sec_profile;

	if (!dual_fg_drv->cable_in) {
		dual_fg_drv->cc_balance_offset = 0;
		dual_fg_drv->fv_balance_offset = 0;
		gvotable_cast_int_vote(dual_fg_drv->fcc_votable, DUAL_BATT_BALANCE_VOTER,
				       0, false);
		gvotable_cast_int_vote(dual_fg_drv->fv_votable, DUAL_BATT_BALANCE_VOTER,
				       0, false);
		return;
	}

	/* check for the last tier */
	if (next_vbat_idx >= profile->volt_nb_limits)
		next_vbat_idx = vbat_idx;

	next_cc_max = GBMS_CCCM_LIMITS(profile, temp_idx, next_vbat_idx);
	if (next_cc_max >= dual_fg_drv->cc_max)
		cc_lowerbd = (next_cc_max * dual_fg_drv->cc_balance_ratio) / 100;
	else
		cc_lowerbd = next_cc_max;

	cc_base = GBMS_CCCM_LIMITS(base_profile, temp_idx, vbat_idx);
	cc_sec= GBMS_CCCM_LIMITS(sec_profile, temp_idx, vbat_idx);

	ibase = GPSY_GET_PROP(dual_fg_drv->first_fg_psy, POWER_SUPPLY_PROP_CURRENT_AVG) * -1;
	isec = GPSY_GET_PROP(dual_fg_drv->second_fg_psy, POWER_SUPPLY_PROP_CURRENT_AVG) * -1;

	if ((ibase > cc_base) || (isec > cc_sec)) {
		const int cc_offset_max = dual_fg_drv->cc_max - cc_lowerbd;
		cc_offset = gdbatt_oc_cc_offset(ibase, isec, cc_base, cc_sec,
			dual_fg_drv->cc_balance_offset, DUAL_BATT_BALANCE_CC_ADJUST_STEP);
		if (cc_offset > cc_offset_max)
			cc_offset = cc_offset_max;

		gvotable_cast_int_vote(dual_fg_drv->fcc_votable, DUAL_BATT_BALANCE_VOTER,
				       dual_fg_drv->cc_max - cc_offset, true);

		logbuffer_prlog(dual_fg_drv, LOGLEVEL_DEBUG,
				"%s: battery OC base:%d/%d sec:%d/%d cc_offset:%d->%d cc_max:%d (%d/%d)",
				__func__, ibase, cc_base, isec, cc_sec,
				dual_fg_drv->cc_balance_offset, cc_offset,
				dual_fg_drv->cc_max - cc_offset, next_cc_max, cc_lowerbd);

		dual_fg_drv->cc_balance_offset = cc_offset;
		power_supply_changed(dual_fg_drv->psy);
	}
}

static void gdbatt_ov_last_tier(struct dual_fg_drv *dual_fg_drv)
{
	if (!dual_fg_drv->fv_votable)
		dual_fg_drv->fv_votable = gvotable_election_get_handle(VOTABLE_MSC_FV);
	if (dual_fg_drv->fv_votable) {
		struct gbms_chg_profile *profile = &dual_fg_drv->chg_profile;
		const int fv = profile->volt_limits[profile->volt_nb_limits - 1];
		int fv_offset = dual_fg_drv->fv_balance_offset + DUAL_BATT_BALANCE_FV_ADJUST_STEP;

		gvotable_cast_int_vote(dual_fg_drv->fv_votable, DUAL_BATT_BALANCE_VOTER,
				       fv - fv_offset, true);
		logbuffer_prlog(dual_fg_drv, LOGLEVEL_DEBUG, "%s: battery over max fv:%d->%d",
				__func__, fv, fv - fv_offset);
		dual_fg_drv->fv_balance_offset = fv_offset;
	}
}

static void gdbatt_ov_handler(struct dual_fg_drv *dual_fg_drv, int vbatt_idx, int temp_idx)
{
	struct gbms_chg_profile *profile = &dual_fg_drv->chg_profile;
	const int cc_max = dual_fg_drv->cc_max;
	int next_cc_max;
	int next_vbatt_idx = vbatt_idx + 1;

	if (next_vbatt_idx >= profile->volt_nb_limits)
		next_vbatt_idx = vbatt_idx;

	next_cc_max = GBMS_CCCM_LIMITS(profile, temp_idx, next_vbatt_idx);
	if (next_cc_max == cc_max) {
		pr_debug("%s: skip ov for tier %d/%d", __func__, vbatt_idx, next_vbatt_idx);
		return;
	}

	if (!dual_fg_drv->fcc_votable)
		dual_fg_drv->fcc_votable = gvotable_election_get_handle(VOTABLE_MSC_FCC);
	if (dual_fg_drv->fcc_votable) {
		const int cc_offset_max = cc_max - next_cc_max;
		int cc_offset = dual_fg_drv->cc_balance_offset + DUAL_BATT_BALANCE_CC_ADJUST_STEP;

		if (cc_offset > cc_offset_max)
			cc_offset = cc_offset_max;

		gvotable_cast_int_vote(dual_fg_drv->fcc_votable, DUAL_BATT_BALANCE_VOTER,
				       cc_max - cc_offset, true);
		logbuffer_prlog(dual_fg_drv, LOGLEVEL_DEBUG,"%s: battery OV cc_max:%d->%d (%d)",
				__func__, cc_max, cc_max - cc_offset, next_cc_max);

		dual_fg_drv->cc_balance_offset = cc_offset;
		power_supply_changed(dual_fg_drv->psy);
	}
}

/*
 * use the max of base and flip vbatt as battery voltage to solve the problem of
 * OV triggered by battery DCR(Direct Current Resistanc) unbalance.
 */
static int gdbatt_get_dual_vbatt(struct dual_fg_drv *dual_fg_drv,
				 int base_vbatt, int sec_vbatt)
{
	const int ov_allowed_idx = dual_fg_drv->vbatt_ov_allowed_idx;
	int dual_vbatt;

	if (ov_allowed_idx < 0) {
		const int sec_ratio = dual_fg_drv->vsec_ratio;
		const int base_ratio = 100 - sec_ratio;

		/* get the battery voltage based on the ratio of base and sec vbatt */
		dual_vbatt = (base_vbatt * base_ratio + sec_vbatt * sec_ratio) / 100;
	}
	else {
		/* use the max of base and flip vbatt as battery voltage */
		dual_vbatt = MAX(base_vbatt, sec_vbatt);
	}

	return dual_vbatt;
}

static bool gdbatt_ov_handler_allowed(struct dual_fg_drv *dual_fg_drv, int vbatt_idx)
{
	const int ov_allowed_idx = dual_fg_drv->vbatt_ov_allowed_idx;

	return ov_allowed_idx < 0 ? false : vbatt_idx >= ov_allowed_idx;
}

static void gdbatt_select_cc_max(struct dual_fg_drv *dual_fg_drv)
{
	struct gbms_chg_profile *profile = &dual_fg_drv->chg_profile;
	int base_temp, sec_temp, base_vbatt, sec_vbatt, dual_vbatt;
	int base_temp_idx, sec_temp_idx, base_vbatt_idx, sec_vbatt_idx, temp_idx, vbatt_idx;
	int base_cc_max, sec_cc_max, cc_max;
	struct power_supply *base_psy = dual_fg_drv->first_fg_psy;
	struct power_supply *sec_psy = dual_fg_drv->second_fg_psy;
	int ret = 0;
	bool check_current = false;

	if (!dual_fg_drv->cable_in)
		goto check_done;

	ret = gdbatt_get_temp(base_psy, &base_temp);
	if (ret < 0)
		goto check_done;

	ret = gdbatt_get_temp(sec_psy, &sec_temp);
	if (ret < 0)
		goto check_done;

	base_vbatt = GPSY_GET_PROP(base_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW);
	if (base_vbatt < 0)
		goto check_done;

	sec_vbatt = GPSY_GET_PROP(sec_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW);
	if (sec_vbatt < 0)
		goto check_done;

	dual_vbatt = gdbatt_get_dual_vbatt(dual_fg_drv, base_vbatt, sec_vbatt);

	base_temp_idx = gdbatt_select_temp_idx(profile, base_temp);
	sec_temp_idx = gdbatt_select_temp_idx(profile, sec_temp);

	vbatt_idx = gdbatt_select_voltage_idx(profile, dual_vbatt);
	if (base_vbatt > profile->volt_limits[profile->volt_nb_limits - 1])
		base_vbatt_idx = profile->volt_nb_limits;
	else
		base_vbatt_idx = gdbatt_select_voltage_idx(profile, base_vbatt);

	if (sec_vbatt > profile->volt_limits[profile->volt_nb_limits - 1]) {
		sec_vbatt_idx = profile->volt_nb_limits;
	} else {
		const int  sec_vbatt_offset = sec_vbatt - dual_fg_drv->vsec_offset;

		sec_vbatt_idx = gdbatt_select_voltage_idx(profile, sec_vbatt_offset);
		/* only apply offset in allowed idx */
		if (sec_vbatt_idx > dual_fg_drv->vsec_offset_max_idx)
			sec_vbatt_idx = gdbatt_select_voltage_idx(profile, sec_vbatt);
	}

	base_cc_max = GBMS_CCCM_LIMITS(profile, base_temp_idx, vbatt_idx);
	sec_cc_max = GBMS_CCCM_LIMITS(profile, sec_temp_idx, vbatt_idx);
	if (base_cc_max <= sec_cc_max) {
		cc_max = base_cc_max;
		temp_idx = base_temp_idx;
	} else {
		cc_max = sec_cc_max;
		temp_idx = sec_temp_idx;
	}

	if (cc_max == dual_fg_drv->cc_max) {
		if ((base_vbatt_idx > vbatt_idx) || (sec_vbatt_idx > vbatt_idx) ||
		    /*
		     * trigger ov_handler in allowed idx if use max of
		     * base and flip vbatt as dual vbatt
		     */
		    gdbatt_ov_handler_allowed(dual_fg_drv, vbatt_idx)) {
			logbuffer_prlog(dual_fg_drv, LOGLEVEL_DEBUG,
					"%s: battery OV v_base:%d, v_sec:%d",
					__func__, base_vbatt, sec_vbatt);
			if (vbatt_idx >= profile->volt_nb_limits)
				gdbatt_ov_last_tier(dual_fg_drv);
			else
				gdbatt_ov_handler(dual_fg_drv, vbatt_idx, temp_idx);
		} else {
			check_current = true;
		}
		goto check_done;
	}

	if (!dual_fg_drv->fcc_votable)
		dual_fg_drv->fcc_votable =
			gvotable_election_get_handle(VOTABLE_MSC_FCC);
	if (dual_fg_drv->fcc_votable) {
		/* reset balance offset */
		dual_fg_drv->cc_balance_offset = 0;
		gvotable_cast_int_vote(dual_fg_drv->fcc_votable, DUAL_BATT_BALANCE_VOTER, 0, false);

		/* set new cc_max by temp */
		pr_info("%s: temp:%d/%d(%d/%d), vbatt:%d/%d(%d/%d), cc_max:%d/%d(%d)\n", __func__,
			base_temp, sec_temp, base_temp_idx, sec_temp_idx, base_vbatt,
			sec_vbatt, base_vbatt_idx, sec_vbatt_idx, base_cc_max,
			sec_cc_max, cc_max);
		gvotable_cast_int_vote(dual_fg_drv->fcc_votable,
				       DUAL_BATT_TEMP_VOTER, cc_max, true);
		dual_fg_drv->cc_max = cc_max;
		power_supply_changed(dual_fg_drv->psy);
	}

check_done:
	if (check_current || !dual_fg_drv->cable_in)
		gdbatt_check_current(dual_fg_drv, temp_idx, vbatt_idx);
	pr_debug("check done. cable_in=%d (%d)\n", dual_fg_drv->cable_in, ret);
}

static int update_charge_full(struct dual_fg_drv *dual_fg_drv)
{
	struct power_supply *base_psy = dual_fg_drv->first_fg_psy;
	struct power_supply *sec_psy = dual_fg_drv->second_fg_psy;
	int base_data = 0, sec_data = 0;

	if (base_psy)
		base_data = GPSY_GET_PROP(base_psy, POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN);

	if (sec_psy)
		sec_data = GPSY_GET_PROP(sec_psy, POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN);

	if (base_data < 0 || sec_data < 0)
		return -EAGAIN;

	dev_info(dual_fg_drv->device, "update base_charge_full:%d->%d, sec_charge_full:%d->%d\n",
		 dual_fg_drv->base_charge_full, base_data, dual_fg_drv->sec_charge_full, sec_data);

	dual_fg_drv->base_charge_full = base_data;
	dual_fg_drv->sec_charge_full = sec_data;

	return 0;
}

static int gdbatt_get_capacity(struct dual_fg_drv *dual_fg_drv, int base_soc, int sec_soc)
{
	const int base_full = dual_fg_drv->base_charge_full / 1000;
	const int sec_full = dual_fg_drv->sec_charge_full / 1000;
	const int full_sum = base_full + sec_full;

	if (dual_fg_drv->seq_wa.report_soc_type == REPORT_BASE)
		return base_soc;

	if (dual_fg_drv->seq_wa.report_soc_type == REPORT_SEC)
		return sec_soc;

	if (!base_full || !sec_full)
		return (base_soc + sec_soc) / 2;

	return (base_soc * base_full + sec_soc * sec_full) / full_sum;
}

static int seq_soc_drop_check(const struct seq_soc_drop_wa *wa, int seq_soc, int sec_soc)
{
	const int soc_delta = sec_soc - seq_soc;
	int report_soc_type = REPORT_DEFAULT;

	if (!wa->seq_fg_soc_limit || !wa->seq_fg_delta_limit)
		return report_soc_type;

	if (!seq_soc || !sec_soc)
		report_soc_type = seq_soc ? REPORT_BASE : REPORT_SEC;
	else if (soc_delta > 0 && seq_soc < wa->seq_fg_soc_limit)
		report_soc_type = REPORT_SEC;

	return report_soc_type;
}

static void gdbatt_fg_logging(struct dual_fg_drv *dual_fg_drv, int base_soc_raw, int sec_soc_raw)
{
	const int base_soc = qnum_toint(qnum_from_q8_8(base_soc_raw));
	const int sec_soc = qnum_toint(qnum_from_q8_8(sec_soc_raw));
	const int soc_delta = sec_soc - base_soc;
	const int soc_logging_delta = dual_fg_drv->seq_wa.seq_fg_delta_limit ?
				      dual_fg_drv->seq_wa.seq_fg_delta_limit : MONITOR_SOC_DIFF;
	int report_soc_type;

	if (dual_fg_drv->base_soc == base_soc && dual_fg_drv->sec_soc == sec_soc)
		return;

	if (!dual_fg_drv->base_soc && !dual_fg_drv->sec_soc)
		dev_info(dual_fg_drv->device, "initial base_soc:%d(%#x), sec_soc:%d(%#x)\n",
			 base_soc, base_soc_raw, sec_soc, sec_soc_raw);

	if (abs(soc_delta) < soc_logging_delta) {
		/* Dump registers */
		GPSY_SET_PROP(dual_fg_drv->first_fg_psy, GBMS_PROP_FG_REG_LOGGING, true);
		GPSY_SET_PROP(dual_fg_drv->second_fg_psy, GBMS_PROP_FG_REG_LOGGING, true);
	}

	dual_fg_drv->base_soc = base_soc;
	dual_fg_drv->sec_soc = sec_soc;
	report_soc_type = seq_soc_drop_check(&dual_fg_drv->seq_wa, base_soc, sec_soc);
	if (dual_fg_drv->seq_wa.report_soc_type != report_soc_type) {
		dev_info(dual_fg_drv->device, "report_soc_type:%d -> %d, seq_soc:%d, sec_soc:%d\n",
			 dual_fg_drv->seq_wa.report_soc_type, report_soc_type, base_soc, sec_soc);
		dual_fg_drv->seq_wa.report_soc_type = report_soc_type;
	}
}

static void gdbatt_stats_init(struct dual_fg_drv *dual_fg_drv)
{
	const ktime_t now = get_boot_sec();

	mutex_lock(&dual_fg_drv->stats_lock);

	memset(&dual_fg_drv->base_batt_stats, 0, sizeof(dual_fg_drv->base_batt_stats));
	memset(&dual_fg_drv->sec_batt_stats, 0, sizeof(dual_fg_drv->sec_batt_stats));

	gbms_tier_stats_init(&dual_fg_drv->base_batt_stats, GBMS_STATS_BASE_BATT);
	gbms_tier_stats_init(&dual_fg_drv->sec_batt_stats, GBMS_STATS_SEC_BATT);

	dual_fg_drv->last_update = now;

	mutex_unlock(&dual_fg_drv->stats_lock);
}

/* call holding stats_lock */
static int gbatt_update_batt_stats(struct power_supply *psy, ktime_t elap,
				   struct gbms_ce_tier_stats *tier,
				   struct gbms_chg_profile *profile,
				   union gbms_charger_state *chg_state)
{
	int ret, temp, temp_idx, ibatt_ma, cc, soc_in;

	ret = gdbatt_get_temp(psy, &temp);
	if (ret < 0)
		return ret;

	temp_idx = gbms_msc_temp_idx(profile, temp);

	ibatt_ma = GPSY_GET_INT_PROP(psy, POWER_SUPPLY_PROP_CURRENT_NOW, &ret);
	if (ret < 0)
		return ret;

	ibatt_ma /= 1000;

	cc = GPSY_GET_INT_PROP(psy, POWER_SUPPLY_PROP_CHARGE_COUNTER, &ret);
	if (ret < 0)
		return ret;

	soc_in = GPSY_GET_PROP(psy, GBMS_PROP_CAPACITY_RAW);
	if (soc_in < 0) {
		pr_info("MSC_STAT dual cannot read soc_in=%d\n", soc_in);
		/* We still want to update initialized tiers. */
		soc_in = -1;
	}

	gbms_stats_update_tier(temp_idx, ibatt_ma, temp, elap, cc, chg_state, -1, soc_in, tier);

	return 0;
}

static void gbatt_update_stats(struct dual_fg_drv *dual_fg_drv)
{
	const ktime_t now = get_boot_sec();
	ktime_t elap;
	int ret = 0;

	ret = gdbatt_resume_check(dual_fg_drv);
	if (ret < 0)
		return;

	if (!dual_fg_drv->cable_in)
		return;

	if (!dual_fg_drv->first_fg_psy && !dual_fg_drv->second_fg_psy)
		return;

	if (!dual_fg_drv->batt_psy)
		dual_fg_drv->batt_psy = power_supply_get_by_name("battery");

	if (dual_fg_drv->batt_psy)
		dual_fg_drv->chg_state.v = GPSY_GET_INT64_PROP(dual_fg_drv->batt_psy,
							       GBMS_PROP_CHARGE_CHARGER_STATE,
							       &ret);
	if (ret < 0) {
		pr_info("fail to get charge state from battery (%d)\n", ret);
		return;
	}

	mutex_lock(&dual_fg_drv->stats_lock);
	elap = now - dual_fg_drv->last_update;
	dual_fg_drv->last_update = now;

	if (dual_fg_drv->first_fg_psy)
		ret = gbatt_update_batt_stats(dual_fg_drv->first_fg_psy, elap,
					      &dual_fg_drv->base_batt_stats,
					      &dual_fg_drv->base_profile,
					      &dual_fg_drv->chg_state);
	if (ret < 0)
		pr_info("fail to update base battery stats (%d)\n", ret);

	if (dual_fg_drv->second_fg_psy)
		ret = gbatt_update_batt_stats(dual_fg_drv->second_fg_psy, elap,
					      &dual_fg_drv->sec_batt_stats,
					      &dual_fg_drv->sec_profile,
					      &dual_fg_drv->chg_state);

	if (ret < 0)
		pr_info("fail to update sec battery stats (%d)\n", ret);

	mutex_unlock(&dual_fg_drv->stats_lock);
}

static void google_dual_batt_work(struct work_struct *work)
{
	struct dual_fg_drv *dual_fg_drv = container_of(work, struct dual_fg_drv,
						 gdbatt_work.work);
	struct power_supply *base_psy = dual_fg_drv->first_fg_psy;
	struct power_supply *sec_psy = dual_fg_drv->second_fg_psy;

	if (!dual_fg_drv->init_complete)
		goto done;

	gbatt_update_stats(dual_fg_drv);

	if (!base_psy || !sec_psy)
		goto done;

	gdbatt_select_cc_max(dual_fg_drv);

done:
	mod_delayed_work(system_wq, &dual_fg_drv->gdbatt_work,
			 msecs_to_jiffies(DUAL_FG_WORK_PERIOD_MS));
}

static int gdbatt_get_property(struct power_supply *psy,
				 enum power_supply_property psp,
				 union power_supply_propval *val)
{
	struct dual_fg_drv *dual_fg_drv = (struct dual_fg_drv *)
					power_supply_get_drvdata(psy);
	int err = 0;
	union power_supply_propval fg_1;
	union power_supply_propval fg_2;

	err = gdbatt_resume_check(dual_fg_drv);
	if (err != 0)
		return err;

	if (!dual_fg_drv->first_fg_psy && !dual_fg_drv->second_fg_psy)
		return -EAGAIN;

	if (!dual_fg_drv->first_fg_psy || !dual_fg_drv->second_fg_psy)
		goto single_fg;

	err = power_supply_get_property(dual_fg_drv->first_fg_psy, psp, &fg_1);
	if (err != 0) {
		pr_debug("error %d reading first fg prop %d\n", err, psp);
		return err;
	}

	err = power_supply_get_property(dual_fg_drv->second_fg_psy, psp, &fg_2);
	if (err != 0) {
		pr_debug("error %d reading second fg prop %d\n", err, psp);
		return err;
	}

	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
	case POWER_SUPPLY_PROP_CHARGE_FULL:
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
	case POWER_SUPPLY_PROP_CURRENT_AVG:
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = fg_1.intval + fg_2.intval;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = MAX(fg_1.intval, fg_2.intval);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
	case POWER_SUPPLY_PROP_TIME_TO_FULL_AVG:
		val->intval = MAX(fg_1.intval, fg_2.intval);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = gdbatt_get_dual_vbatt(dual_fg_drv, fg_1.intval, fg_2.intval);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_AVG:
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
	case POWER_SUPPLY_PROP_VOLTAGE_OCV:
		val->intval = gdbatt_get_dual_vbatt(dual_fg_drv, fg_1.intval, fg_2.intval);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = gdbatt_get_capacity(dual_fg_drv, fg_1.intval, fg_2.intval);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		/* larger one is bad. TODO: confirm its priority */
		val->intval = MAX(fg_1.intval, fg_2.intval);
		break;
	case POWER_SUPPLY_PROP_STATUS:
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = fg_1.intval;
		if (fg_1.intval != fg_2.intval)
			pr_debug("case %d not align: %d/%d",
				psp, fg_1.intval, fg_2.intval);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = fg_1.intval && fg_2.intval;
		if (fg_1.intval != fg_2.intval)
			pr_debug("PRESENT different: %d/%d", fg_1.intval, fg_2.intval);
		break;
	case POWER_SUPPLY_PROP_SERIAL_NUMBER:
		/* TODO: need hash SN */
		val->strval = fg_1.strval;
		break;
	default:
		pr_debug("getting unsupported property: %d\n", psp);
		break;
	}

	return 0;

single_fg:
	if (dual_fg_drv->first_fg_psy)
		err = power_supply_get_property(dual_fg_drv->first_fg_psy, psp, val);
	else if (dual_fg_drv->second_fg_psy)
		err = power_supply_get_property(dual_fg_drv->second_fg_psy, psp, val);

	if (err < 0)
		pr_debug("error %d reading single prop %d\n", err, psp);

	return 0;
}

static int gdbatt_set_property(struct power_supply *psy,
				 enum power_supply_property psp,
				 const union power_supply_propval *val)
{
	/* move gbms psp to gdbatt_gbms_set_property */
	return 0;
}

static int gdbatt_property_is_writeable(struct power_supply *psy,
					  enum power_supply_property psp)
{
	/* move gbms psp to gdbatt_gbms_property_is_writeable */
	return 0;
}

static int gdbatt_gbms_get_property(struct power_supply *psy,
				    enum gbms_property psp,
				    union gbms_propval *val)
{
	struct dual_fg_drv *dual_fg_drv = (struct dual_fg_drv *)
					power_supply_get_drvdata(psy);
	int err = 0;
	union gbms_propval fg_1;
	union gbms_propval fg_2;

	err = gdbatt_resume_check(dual_fg_drv);
	if (err != 0)
		return err;

	if (!dual_fg_drv->first_fg_psy && !dual_fg_drv->second_fg_psy)
		return -EAGAIN;

	if (!dual_fg_drv->first_fg_psy || !dual_fg_drv->second_fg_psy)
		goto single_fg;

	fg_1.prop.intval = GPSY_GET_INT_PROP(dual_fg_drv->first_fg_psy, psp, &err);
	if (err != 0) {
		pr_debug("error %d reading first fg prop %d\n", err, psp);
		return err;
	}

	fg_2.prop.intval = GPSY_GET_INT_PROP(dual_fg_drv->second_fg_psy, psp, &err);
	if (err != 0) {
		pr_debug("error %d reading second fg prop %d\n", err, psp);
		return err;
	}

	switch (psp) {
	case GBMS_PROP_CAPACITY_RAW:
		gdbatt_fg_logging(dual_fg_drv, fg_1.prop.intval, fg_2.prop.intval);
		val->prop.intval = gdbatt_get_capacity(dual_fg_drv, fg_1.prop.intval, fg_2.prop.intval);
		break;
	/* support bhi */
	case GBMS_PROP_HEALTH_ACT_IMPEDANCE:
	case GBMS_PROP_HEALTH_IMPEDANCE:
	case GBMS_PROP_RESISTANCE:
	case GBMS_PROP_RESISTANCE_RAW:
	case GBMS_PROP_RESISTANCE_AVG:
	case GBMS_PROP_BATTERY_AGE:
	case GBMS_PROP_CHARGE_FULL_ESTIMATE:
	case GBMS_PROP_CAPACITY_FADE_RATE:
	case GBMS_PROP_CAPACITY_FADE_RATE_FCR:
	case GBMS_PROP_BATT_ID:
	case GBMS_PROP_AAFV:
		val->prop.intval = fg_1.prop.intval;
		break;
	case GBMS_PROP_RECAL_FG:
		/* TODO: under porting */
		break;
	default:
		pr_debug("%s: route to gdbatt_get_property, psp:%d\n", __func__, psp);
		return -ENODATA;
	}

	return 0;

single_fg:
	if (dual_fg_drv->first_fg_psy)
		val->prop.intval = GPSY_GET_INT_PROP(dual_fg_drv->first_fg_psy, psp, &err);
	else if (dual_fg_drv->second_fg_psy)
		val->prop.intval = GPSY_GET_INT_PROP(dual_fg_drv->second_fg_psy, psp, &err);

	if (err < 0)
		pr_debug("error %d reading single prop %d\n", err, psp);

	return 0;
}

static int gdbatt_gbms_set_property(struct power_supply *psy,
				    enum gbms_property psp,
				    const union gbms_propval *val)
{
	struct dual_fg_drv *dual_fg_drv = (struct dual_fg_drv *)
					power_supply_get_drvdata(psy);
	int ret = 0;

	ret = gdbatt_resume_check(dual_fg_drv);
	if (ret != 0)
		return ret;

	switch (psp) {
	case GBMS_PROP_BATT_CE_CTRL:
		if (dual_fg_drv->first_fg_psy) {
			ret = GPSY_SET_PROP(dual_fg_drv->first_fg_psy, psp, val->prop.intval);
			if (ret < 0)
				pr_err("Cannot set the first BATT_CE_CTRL, ret=%d\n", ret);
		}
		if (dual_fg_drv->second_fg_psy) {
			ret = GPSY_SET_PROP(dual_fg_drv->second_fg_psy, psp, val->prop.intval);
			if (ret < 0)
				pr_err("Cannot set the second BATT_CE_CTRL, ret=%d\n", ret);
		}
		mutex_lock(&dual_fg_drv->stats_lock);
		dual_fg_drv->cable_in = !!val->prop.intval;
		if (dual_fg_drv->cable_in)
			dual_fg_drv->last_update = get_boot_sec();
		mutex_unlock(&dual_fg_drv->stats_lock);
		mod_delayed_work(system_wq, &dual_fg_drv->gdbatt_work, 0);
		break;
	case GBMS_PROP_HEALTH_ACT_IMPEDANCE:
		/* TODO: discuss with BattEng to decide save data */
		/* if (dual_fg_drv->first_fg_psy) {
			ret = GPSY_SET_PROP(dual_fg_drv->first_fg_psy, psp, val->prop.intval);
			if (ret < 0)
				pr_err("Cannot set the first HEALTH_ACT_IMPEDANCE, ret=%d\n", ret);
		} */
		break;
	case GBMS_PROP_RECAL_FG:
		/* TODO: under porting */
		break;
	case GBMS_PROP_AAFV:
		if (dual_fg_drv->first_fg_psy) {
			ret = GPSY_SET_PROP(dual_fg_drv->first_fg_psy, psp, val->prop.intval);
			if (ret < 0)
				pr_err("Cannot set aafv to the first FG, ret=%d\n", ret);
		}
		if (dual_fg_drv->second_fg_psy) {
			ret = GPSY_SET_PROP(dual_fg_drv->second_fg_psy, psp, val->prop.intval);
			if (ret < 0)
				pr_err("Cannot set aafv to the second FG, ret=%d\n", ret);
		}
		break;
	default:
		pr_debug("%s: route to gdbatt_set_property, psp:%d\n", __func__, psp);
		return -ENODATA;
	}

	if (ret < 0) {
		pr_debug("gdbatt: set_prop cannot write psp=%d\n", psp);
		return ret;
	}

	return 0;
}

static int gdbatt_gbms_property_is_writeable(struct power_supply *psy,
					     enum gbms_property psp)
{
	switch (psp) {
	case GBMS_PROP_BATT_CE_CTRL:
	case GBMS_PROP_AAFV:
		return 1;
	default:
		break;
	}

	return 0;
}

static struct gbms_desc gdbatt_psy_desc = {
	.psy_dsc.name = "dualbatt",
	.psy_dsc.type = POWER_SUPPLY_TYPE_BATTERY,
	.psy_dsc.get_property = gdbatt_get_property,
	.psy_dsc.set_property = gdbatt_set_property,
	.psy_dsc.property_is_writeable = gdbatt_property_is_writeable,
	.get_property = gdbatt_gbms_get_property,
	.set_property = gdbatt_gbms_set_property,
	.property_is_writeable = gdbatt_gbms_property_is_writeable,
	.psy_dsc.properties = gdbatt_fg_props,
	.psy_dsc.num_properties = ARRAY_SIZE(gdbatt_fg_props),
	.forward = true,
};

/* ------------------------------------------------------------------------ */

static int gdbatt_init_pack_chg_profile(struct gbms_chg_profile *pack_profile,
					struct device_node *node,
					const struct gbms_chg_profile *profile,
					u32 capacity_ma)
{
	const u32 table_size = (profile->temp_nb_limits - 1) * profile->volt_nb_limits;
	u32 ccm;
	int vi, ti, ret;

	/* copy profile to pack_profile */
	memcpy(pack_profile, profile, sizeof(*pack_profile));

	/* update C rates into pack_profile->cccm_limits */
	pack_profile->cccm_limits = kzalloc(sizeof(s32) * table_size, GFP_KERNEL);
	if (!pack_profile->cccm_limits)
		return -ENOMEM;

	ret = of_property_read_u32_array(node, "google,chg-pack-cc-limits",
					 pack_profile->cccm_limits,
					 table_size);
	if (ret < 0) {
		pr_err("cannot read chg-pack-cc-limits table, ret=%d\n", ret);
		kfree(pack_profile->cccm_limits);
		pack_profile->cccm_limits = NULL;
		return -EINVAL;
	}

	/* chg-battery-capacity is in mAh, chg-cc-limits relative to 100 */
	for (ti = 0; ti < pack_profile->temp_nb_limits - 1; ti++) {
		for (vi = 0; vi < pack_profile->volt_nb_limits; vi++) {
			ccm = GBMS_CCCM_LIMITS_GET(pack_profile, ti, vi);
			ccm *= capacity_ma * 10;

			GBMS_CCCM_LIMITS_SET(pack_profile, ti, vi) = ccm;
		}
	}

	return ret;
}


static int gdbatt_init_chg_profile(struct dual_fg_drv *dual_fg_drv)
{
	struct device_node *node = of_find_node_by_name(NULL, "google,battery");
	struct device_node *dual_fg_node = dual_fg_drv->device->of_node;
	struct gbms_chg_profile *profile = &dual_fg_drv->chg_profile;
	int ret = 0;

	if (profile->cccm_limits)
		return 0;

	ret = gbms_init_chg_profile(profile, node);
	if (ret < 0)
		return -EINVAL;

	ret = of_property_read_u32(node, "google,chg-battery-capacity",
				   &dual_fg_drv->battery_capacity);
	if (ret < 0)
		pr_warn("battery not present, no default capacity, zero charge table\n");

	ret = of_property_read_u32(dual_fg_node, "google,chg-base-battery-capacity",
				   &dual_fg_drv->base_capacity);
	if (ret < 0)
		pr_warn("base battery not present, no default capacity, zero charge table\n");

	ret = of_property_read_u32(dual_fg_node, "google,chg-sec-battery-capacity",
				   &dual_fg_drv->sec_capacity);
	if (ret < 0)
		pr_warn("secondary battery not present, no default capacity, zero charge table\n");

	gbms_init_chg_table(profile, node, dual_fg_drv->battery_capacity);
	ret = gdbatt_init_pack_chg_profile(&dual_fg_drv->base_profile, dual_fg_node, profile,
					   dual_fg_drv->base_capacity);
	if (ret < 0)
		return ret;
	ret = gdbatt_init_pack_chg_profile(&dual_fg_drv->sec_profile, dual_fg_node, profile,
					   dual_fg_drv->sec_capacity);
	if (ret < 0)
		return ret;

	return ret;
}

static int psy_changed(struct notifier_block *nb,
		       unsigned long action, void *data)
{
	struct power_supply *psy = data;
	struct dual_fg_drv *dual_fg_drv = container_of(nb, struct dual_fg_drv, fg_nb);

	if ((action != PSY_EVENT_PROP_CHANGED) || (psy == NULL) || (psy->desc == NULL) ||
	    (psy->desc->name == NULL))
		return NOTIFY_OK;

	pr_debug("name=%s evt=%lu\n", psy->desc->name, action);

	if (action == PSY_EVENT_PROP_CHANGED) {
		bool is_first_fg = (dual_fg_drv->first_fg_psy_name != NULL) &&
				   !strcmp(psy->desc->name, dual_fg_drv->first_fg_psy_name);
		bool is_second_fg = (dual_fg_drv->second_fg_psy_name != NULL) &&
				    !strcmp(psy->desc->name, dual_fg_drv->second_fg_psy_name);

		if (is_first_fg || is_second_fg)
			power_supply_changed(dual_fg_drv->psy);
	}

	return NOTIFY_OK;
}

static void google_dual_batt_gauge_init_work(struct work_struct *work)
{
	struct dual_fg_drv *dual_fg_drv = container_of(work, struct dual_fg_drv,
						 init_work.work);
	struct power_supply *first_fg_psy = dual_fg_drv->first_fg_psy;
	struct power_supply *second_fg_psy = dual_fg_drv->second_fg_psy;
	union power_supply_propval val;
	int err = 0;

	if (!dual_fg_drv->first_fg_psy && dual_fg_drv->first_fg_psy_name) {
		first_fg_psy = power_supply_get_by_name(dual_fg_drv->first_fg_psy_name);
		if (!first_fg_psy) {
			dev_info(dual_fg_drv->device,
				"failed to get \"%s\" power supply, retrying...\n",
				dual_fg_drv->first_fg_psy_name);
			goto retry_init_work;
		}

		dual_fg_drv->first_fg_psy = first_fg_psy;

		/* Don't use it if battery not present */
		err = power_supply_get_property(first_fg_psy,
						POWER_SUPPLY_PROP_PRESENT, &val);
		if (err == -EAGAIN)
			goto retry_init_work;
		if (err == 0 && val.intval == 0) {
			dev_info(dual_fg_drv->device, "First battery not PRESENT\n");
			dual_fg_drv->first_fg_psy_name = NULL;
			dual_fg_drv->first_fg_psy = NULL;
		}
	}

	if (!dual_fg_drv->second_fg_psy && dual_fg_drv->second_fg_psy_name) {
		second_fg_psy = power_supply_get_by_name(dual_fg_drv->second_fg_psy_name);
		if (!second_fg_psy) {
			pr_info("failed to get \"%s\" power supply, retrying...\n",
				dual_fg_drv->second_fg_psy_name);
			goto retry_init_work;
		}

		dual_fg_drv->second_fg_psy = second_fg_psy;

		/* Don't use it if battery not present */
		err = power_supply_get_property(second_fg_psy,
						POWER_SUPPLY_PROP_PRESENT, &val);
		if (err == -EAGAIN)
			goto retry_init_work;
		if (err == 0 && val.intval == 0) {
			dev_info(dual_fg_drv->device, "Second battery not PRESENT\n");
			dual_fg_drv->second_fg_psy_name = NULL;
			dual_fg_drv->second_fg_psy = NULL;
		}
	}

	err = update_charge_full(dual_fg_drv);
	if (err == -EAGAIN)
		goto retry_init_work;

	dual_fg_drv->cc_max = -1;
	err = gdbatt_init_chg_profile(dual_fg_drv);
	if (err < 0)
		dev_info(dual_fg_drv->device,"fail to init chg profile (%d)\n", err);

	dual_fg_drv->fg_nb.notifier_call = psy_changed;
	err = power_supply_reg_notifier(&dual_fg_drv->fg_nb);
	if (err < 0)
		pr_err("cannot register power supply notifer (%d)\n", err);

	gdbatt_stats_init(dual_fg_drv);

	dual_fg_drv->init_complete = true;
	dual_fg_drv->resume_complete = true;
	mod_delayed_work(system_wq, &dual_fg_drv->gdbatt_work, 0);
	dev_info(dual_fg_drv->device, "google_dual_batt_gauge_init_work done\n");

	return;

retry_init_work:
	schedule_delayed_work(&dual_fg_drv->init_work,
			      msecs_to_jiffies(DUAL_FG_DELAY_INIT_MS));
}

static ssize_t dbatt_stats_store(struct device *dev, struct device_attribute *attr,
				     const char *buf, size_t count) {
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct dual_fg_drv *dual_fg_drv = power_supply_get_drvdata(psy);

	if (count < 1)
		return -ENODATA;

	if (buf[0] == '0')
		gdbatt_stats_init(dual_fg_drv);

	return count;
}

static ssize_t dbatt_stats_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct dual_fg_drv *dual_fg_drv = power_supply_get_drvdata(psy);
	struct gbms_ce_tier_stats *b_stats = &dual_fg_drv->base_batt_stats;
	struct gbms_ce_tier_stats *s_stats = &dual_fg_drv->sec_batt_stats;
	uint32_t time_sum;
	ssize_t len = 0;

	mutex_lock(&dual_fg_drv->stats_lock);

	time_sum = b_stats->time_fast + b_stats->time_other + b_stats->time_taper;
	if (time_sum > 0)
		len = gbms_tier_stats_cstr(&buf[len], PAGE_SIZE, b_stats, false);

	time_sum = s_stats->time_fast + s_stats->time_other + s_stats->time_taper;
	if (time_sum > 0)
		len += gbms_tier_stats_cstr(&buf[len], PAGE_SIZE, s_stats, false);

	mutex_unlock(&dual_fg_drv->stats_lock);

	return len;
}

static const DEVICE_ATTR_RW(dbatt_stats);

static int google_dual_batt_init_sysfs(struct dual_fg_drv *dual_fg_drv)
{
	struct seq_soc_drop_wa *wa = &dual_fg_drv->seq_wa;
	struct dentry *de;
	int ret;

	/* stats */
	ret = device_create_file(&dual_fg_drv->psy->dev, &dev_attr_dbatt_stats);
	if (ret)
		dev_err(&dual_fg_drv->psy->dev, "Failed to create dbatt_stats\n");

	/* debugfs */
	de = debugfs_create_dir("google_dual_batt", 0);
	if (IS_ERR_OR_NULL(de)) {
		dev_err(dual_fg_drv->device, "Couldn't create debugfs, (%ld)\n", PTR_ERR(de));
	} else {
		debugfs_create_u32("debug_level", 0644, de, &debug_printk_prlog);
		debugfs_create_u32("seq_report_soc_type", 0644, de, &wa->report_soc_type);
		debugfs_create_u32("seq_soc_limit", 0644, de, &wa->seq_fg_soc_limit);
		debugfs_create_u32("seq_delta_limit", 0644, de, &wa->seq_fg_delta_limit);
	}

	return 0;
}

static int google_dual_batt_gauge_probe(struct platform_device *pdev)
{
	const char *first_fg_psy_name;
	const char *second_fg_psy_name;
	struct dual_fg_drv *dual_fg_drv;
	struct power_supply_config psy_cfg = {};
	int ret;

	dual_fg_drv = devm_kzalloc(&pdev->dev, sizeof(*dual_fg_drv), GFP_KERNEL);
	if (!dual_fg_drv)
		return -ENOMEM;

	dual_fg_drv->device = &pdev->dev;

	ret = of_property_read_string(pdev->dev.of_node, "google,first-fg-psy-name",
				      &first_fg_psy_name);
	if (ret == 0) {
		pr_info("google,first-fg-psy-name=%s\n", first_fg_psy_name);
		dual_fg_drv->first_fg_psy_name = devm_kstrdup(&pdev->dev,
							first_fg_psy_name, GFP_KERNEL);
		if (!dual_fg_drv->first_fg_psy_name)
			return -ENOMEM;
	}

	ret = of_property_read_string(pdev->dev.of_node, "google,second-fg-psy-name",
				      &second_fg_psy_name);
	if (ret == 0) {
		pr_info("google,second-fg-psy-name=%s\n", second_fg_psy_name);
		dual_fg_drv->second_fg_psy_name = devm_kstrdup(&pdev->dev, second_fg_psy_name,
							GFP_KERNEL);
		if (!dual_fg_drv->second_fg_psy_name)
			return -ENOMEM;
	}

	if (!dual_fg_drv->first_fg_psy_name && !dual_fg_drv->second_fg_psy_name) {
		pr_err("no dual gauge setting\n");
		return -EINVAL;
	}

	ret = of_property_read_u32(pdev->dev.of_node, "google,vsec-offset",
				   &dual_fg_drv->vsec_offset);
	if (ret < 0) {
		pr_debug("Couldn't set vsec_offset (%d)\n", ret);
		dual_fg_drv->vsec_offset = DUAL_BATT_VSEC_OFFSET;
	}

	INIT_DELAYED_WORK(&dual_fg_drv->init_work, google_dual_batt_gauge_init_work);
	INIT_DELAYED_WORK(&dual_fg_drv->gdbatt_work, google_dual_batt_work);
	mutex_init(&dual_fg_drv->stats_lock);
	platform_set_drvdata(pdev, dual_fg_drv);

	psy_cfg.drv_data = dual_fg_drv;
	psy_cfg.of_node = pdev->dev.of_node;

	if (of_property_read_bool(pdev->dev.of_node, "google,psy-type-unknown"))
		gdbatt_psy_desc.psy_dsc.type = POWER_SUPPLY_TYPE_UNKNOWN;

	dual_fg_drv->psy = devm_power_supply_register(dual_fg_drv->device,
						   &gdbatt_psy_desc.psy_dsc, &psy_cfg);
	if (IS_ERR(dual_fg_drv->psy)) {
		ret = PTR_ERR(dual_fg_drv->psy);
		if (ret == -EPROBE_DEFER)
			return -EPROBE_DEFER;

		/* TODO: fail with -ENODEV */
		dev_err(dual_fg_drv->device,
			"Couldn't register as power supply, ret=%d\n", ret);
	}

	ret = of_property_read_u32(pdev->dev.of_node, "google,cc-balance-ratio",
				   &dual_fg_drv->cc_balance_ratio);
	if (ret < 0)
		dual_fg_drv->cc_balance_ratio = 100;

	ret = of_property_read_u32(pdev->dev.of_node, "google,vfloat-offset-max-idx",
				   &dual_fg_drv->vsec_offset_max_idx);
	if (ret < 0)
		dual_fg_drv->vsec_offset_max_idx = DUAL_BATT_VSEC_OFFSET_IDX;

	ret = of_property_read_u32(pdev->dev.of_node, "google,sec-vbatt-ratio",
				   &dual_fg_drv->vsec_ratio);
	if (ret < 0)
		dual_fg_drv->vsec_ratio = DUAL_BATT_VSEC_DEFAULT_RATIO;

	ret = of_property_read_u32(pdev->dev.of_node, "google,vbatt-ov-allowed-idx",
				   &dual_fg_drv->vbatt_ov_allowed_idx);
	if (ret < 0)
		dual_fg_drv->vbatt_ov_allowed_idx = -1;

	ret = of_property_read_u32(pdev->dev.of_node, "google,seq-fg-soc-limit",
				   &dual_fg_drv->seq_wa.seq_fg_soc_limit);
	if (ret < 0)
		dual_fg_drv->seq_wa.seq_fg_soc_limit = 0;

	ret = of_property_read_u32(pdev->dev.of_node, "google,seq-fg-delta-limit",
				   &dual_fg_drv->seq_wa.seq_fg_delta_limit);
	if (ret < 0)
		dual_fg_drv->seq_wa.seq_fg_delta_limit = 0;

	dual_fg_drv->log = logbuffer_register("dual_batt");
	if (IS_ERR(dual_fg_drv->log)) {
		dev_err(dual_fg_drv->device, "Couldn't register logbuffer, (%ld)\n",
			PTR_ERR(dual_fg_drv->log));
		dual_fg_drv->log = NULL;
	}

	google_dual_batt_init_sysfs(dual_fg_drv);

	schedule_delayed_work(&dual_fg_drv->init_work,
					msecs_to_jiffies(DUAL_FG_DELAY_INIT_MS));

	pr_info("google_dual_batt_gauge_probe done\n");

	return 0;
}

static int google_dual_batt_gauge_remove(struct platform_device *pdev)
{
	struct dual_fg_drv *dual_fg_drv = platform_get_drvdata(pdev);

	if (!dual_fg_drv)
		return 0;

	power_supply_unreg_notifier(&dual_fg_drv->fg_nb);
	gbms_free_chg_profile(&dual_fg_drv->chg_profile);
	kfree(dual_fg_drv->base_profile.cccm_limits);
	kfree(dual_fg_drv->sec_profile.cccm_limits);

	if (dual_fg_drv->log)
		logbuffer_unregister(dual_fg_drv->log);

	return 0;
}

static void google_dual_batt_gauge_shutdown(struct platform_device *pdev)
{
	struct dual_fg_drv *dual_fg_drv = platform_get_drvdata(pdev);

	if (!dual_fg_drv)
		return;

	power_supply_unreg_notifier(&dual_fg_drv->fg_nb);
}

static int __maybe_unused google_dual_batt_pm_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct dual_fg_drv *dual_fg_drv = platform_get_drvdata(pdev);

	pm_runtime_get_sync(dual_fg_drv->device);
	dual_fg_drv->resume_complete = false;
	pm_runtime_put_sync(dual_fg_drv->device);

	return 0;
}

static int __maybe_unused google_dual_batt_pm_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct dual_fg_drv *dual_fg_drv = platform_get_drvdata(pdev);

	pm_runtime_get_sync(dual_fg_drv->device);
	dual_fg_drv->resume_complete = true;
	pm_runtime_put_sync(dual_fg_drv->device);

	return 0;
}

static const struct dev_pm_ops google_dual_batt_pm_ops = {
	SET_NOIRQ_SYSTEM_SLEEP_PM_OPS(google_dual_batt_pm_suspend, google_dual_batt_pm_resume)
};

static const struct of_device_id google_dual_batt_gauge_of_match[] = {
	{.compatible = "google,dual_batt_gauge"},
	{},
};
MODULE_DEVICE_TABLE(of, google_dual_batt_gauge_of_match);

static struct platform_driver google_dual_batt_gauge_driver = {
	.driver = {
		   .name = "google,dual_batt_gauge",
		   .owner = THIS_MODULE,
		   .of_match_table = google_dual_batt_gauge_of_match,
		   .probe_type = PROBE_PREFER_ASYNCHRONOUS,
		   .pm = &google_dual_batt_pm_ops,
		   },
	.probe = google_dual_batt_gauge_probe,
	.remove = google_dual_batt_gauge_remove,
	.shutdown = google_dual_batt_gauge_shutdown,
};

module_platform_driver(google_dual_batt_gauge_driver);

MODULE_DESCRIPTION("Google Dual Gauge Driver");
MODULE_AUTHOR("Jenny Ho <hsiufangho@google.com>");
MODULE_LICENSE("GPL");
