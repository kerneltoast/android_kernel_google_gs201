/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2020-2022 Google LLC
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

#ifdef CONFIG_PM_SLEEP
#define SUPPORT_PM_SLEEP 1
#endif

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/printk.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/pm_runtime.h>
#include <linux/platform_device.h>
#include <linux/thermal.h>
#include <linux/slab.h>
#include "gbms_power_supply.h"
#include "google_bms.h"
#include "google_dc_pps.h"
#include "google_psy.h"

#include <linux/debugfs.h>

/* Non DC Charger is the default */
#define GCPM_DEFAULT_CHARGER	0
/* TODO: handle capabilities based on index number */
#define GCPM_INDEX_DC_DISABLE	-1
#define GCPM_INDEX_DC_ENABLE	1
#define GCPM_MAX_CHARGERS	4

/* tier based, disabled now */
#define GCPM_DEFAULT_DC_LIMIT_DEMAND	0
/* thermal will change this */
#define GCPM_DEFAULT_DC_LIMIT_CC_MIN		1000000
#define GCPM_DEFAULT_DC_LIMIT_CC_MIN_WLC	2000000

/* voltage based */
#define GCPM_DEFAULT_DC_LIMIT_VBATT_MIN		3600000
#define GCPM_DEFAULT_DC_LIMIT_DELTA_LOW		200000

/* demand based limits */
#define GCPM_DEFAULT_DC_LIMIT_VBATT_MAX		4450000
#define GCPM_DEFAULT_DC_LIMIT_DELTA_HIGH	200000

/* SOC debounce */
#define GCPM_DEFAULT_DC_LIMIT_SOC_HIGH		100

/* behavior in taper */
#define GCPM_TAPER_STEP_FV_MARGIN	0
#define GCPM_TAPER_STEP_CC_STEP		0
#define GCPM_TAPER_STEP_COUNT		0
#define GCPM_TAPER_STEP_GRACE		1
#define GCPM_TAPER_STEP_VOLTAGE		0
#define GCPM_TAPER_STEP_CURRENT		0
/* enough time for the charger to settle to a new limit */
#define GCPM_TAPER_STEP_INTERVAL_S	120

/* TODO: move to configuration */
#define DC_TA_VMAX_MV		9800000
/* TODO: move to configuration */
#define DC_TA_VMIN_MV		8000000
/* TODO: move to configuration */
#define DC_VBATT_HEADROOM_MV	500000

static const int GCPM_FCC_RETRIES = 200;
static const int GCPM_FCC_RETRY_INTERVAL = 1000;

enum gcpm_dc_state_t {
	DC_DISABLED = -1,
	DC_IDLE = 0,
	DC_ENABLE,
	DC_RUNNING,
	DC_ENABLE_PASSTHROUGH,
	DC_PASSTHROUGH,
};

/* DC_ERROR_RETRY_MS <= DC_RUN_DELAY_MS */
#define DC_ENABLE_DELAY_MS	500
#define DC_RUN_DELAY_MS		9000
#define DC_ERROR_RETRY_MS	PPS_ERROR_RETRY_MS

#define PPS_PROG_TIMEOUT_S	10
#define PPS_PROG_RETRY_MS	2000
#define PPS_ACTIVE_RETRY_MS	1500
#define PPS_ACTIVE_USB_TIMEOUT_S	25
#define PPS_ACTIVE_WLC_TIMEOUT_S	500
#define PPS_READY_DELTA_TIMEOUT_S	10

#define PPS_ERROR_RETRY_MS	1000

enum {
	PPS_INDEX_NOT_SUPP = -1,
	PPS_INDEX_TCPM = 1,
	PPS_INDEX_WLC = 2,
	PPS_INDEX_MAX = 2,
};

#define MDIS_OF_CDEV_NAME "google,mdis_charger"
#define MDIS_CDEV_NAME "chg_mdis"
#define MDIS_IN_MAX	4
#define MDIS_OUT_MAX	GCPM_MAX_CHARGERS

#define MDIS_REASON_SETUP "SETUP"

#define COP_WARN_BACKOFF_MS 9000
#define COP_WARN_DEFAULT_OFFSET_MA 100
#define COP_WARN_DEFAULT_TRIGGER_COUNT 3

struct mdis_thermal_device
{
	struct gcpm_drv *gcpm;
	struct mutex tdev_lock;

	struct thermal_cooling_device *tcd;
	u32 *thermal_mitigation;
	int thermal_levels;
	int current_level;
	int therm_fan_alarm_level;
};

struct gcpm_drv  {
	struct device *device;
	struct platform_device *pdev;
	struct power_supply *psy;
	struct delayed_work init_work;

	/* charge limit for wireless DC (legacy) */
	struct gvotable_election *dc_fcc_votable;

	bool cp_fcc_hold;	/* debounces CP */
	int cp_fcc_hold_limit;	/* limit to re-enter CP */

	/* MDIS: wired and wireless via main charger */
	struct gvotable_election *fcc_votable;
	struct gvotable_election *dc_icl_votable;
	struct gvotable_election *tx_icl_votable;
	/* MDIS: wired and wireless via DC charger */
	struct gvotable_election *cp_votable;
	/* MDIS: configuration */
	struct power_supply *mdis_in[MDIS_IN_MAX];
	int mdis_in_count;
	struct power_supply *mdis_out[MDIS_OUT_MAX];
	int mdis_out_count;
	u32 *mdis_out_limits[MDIS_OUT_MAX];
	u32 mdis_out_sel[MDIS_OUT_MAX];
	/* MDIS: device and current budget */
	struct mdis_thermal_device thermal_device;
	struct gvotable_election *mdis_votable;
	struct gvotable_election *fan_level_votable;

	/* CSI */
	struct gvotable_election *csi_status_votable;

	/* combine PPS, route to the active PPS source */
	struct power_supply *pps_psy;

	/* basically the same as mdis_out */
	int chg_psy_retries;
	struct power_supply *chg_psy_avail[GCPM_MAX_CHARGERS];
	const char *chg_psy_names[GCPM_MAX_CHARGERS];
	struct gvotable_election *dc_chg_avail_votable;
	struct mutex chg_psy_lock;
	int chg_psy_active;
	int chg_psy_count;

	/* wakelock */
	struct wakeup_source *gcpm_ws;

	/* force a charger, this might have side effects */
	int force_active;

	struct logbuffer *log;

	/* TCPM state for wired PPS charging */
	const char *tcpm_psy_name;
	struct power_supply *tcpm_psy;
	struct pd_pps_data tcpm_pps_data;
	int log_psy_ratelimit;
	u32 tcpm_phandle;

	/* TCPM state for wireless PPS charging */
	const char *wlc_dc_name;
	struct power_supply *wlc_dc_psy;
	struct pd_pps_data wlc_pps_data;
	u32 wlc_phandle;
	bool no_init_wlc_ta_vol;
	struct delayed_work select_work;

	/* set to force PPS negotiation */
	bool force_pps;
	/* pps state and detect */
	struct delayed_work pps_work;
	/* request of output ua, */
	int out_ua;
	int out_uv;

	int dcen_gpio;
	bool dcen_gpio_default;

	/* >0 when enabled, pps charger to use */
	int pps_index;
	/* >0 when enabled, dc_charger */
	int dc_index;
	/* dc_charging state */
	int dc_state;

	ktime_t dc_start_time;

	/* Disable DC control */
	int dc_ctl;

	/* force check of the DC limit again (debug) */
	bool new_dc_limit;

	/* taper off of current at tier, voltage */
	u32 taper_step_interval;	/* countdown interval in seconds */
	u32 taper_step_voltage;		/* voltage before countdown */
	u32 taper_step_current;		/* current before countdown */
	u32 taper_step_grace;		/* steps from voltage before countdown */
	u32 taper_step_count;		/* countdown steps before dc_done */
	u32 taper_step_fv_margin;		/* countdown steps before dc_done */
	u32 taper_step_cc_step;		/* countdown steps before dc_done */
	int taper_step;			/* actual countdown */
	bool taper_step_used;		/* taper step was actually used */

	/* policy: soc% based limits for DC charging */
	u32 dc_limit_soc_high;		/* DC will not start over high */
	/* policy: power demand limit for DC charging */
	u32 dc_limit_vbatt_low;		/* DC will not stop until low */
	u32 wlc_dc_limit_vbatt_low;	/* WLC DC will not stop until low */
	u32 dc_limit_vbatt_min;		/* DC will start at min */
	u32 wlc_dc_limit_vbatt_min;	/* WLC DC will start at min */
	u32 dc_limit_vbatt_high;	/* DC will not start over high */
	u32 dc_limit_vbatt_max;		/* DC stop at max */
	u32 dc_limit_demand;

	/* TODO: keep TCPM/DC state in a structure add there */
	u32 dc_limit_cc_min;		/* PPS_DC stop if CC_MAX is under this */
	u32 dc_limit_cc_min_wlc;	/* WLC_DC stop if CC_MAX is under this */

	/* cc_max and fv_uv are the demand from google_charger */
	int cc_max;
	int fv_uv;

	bool dc_init_complete;
	bool init_complete;
	bool resume_complete;
	struct notifier_block chg_nb;

	/* Charge Overcurrent Protection */
	bool cop_supported;
	int cop_warn_count;
	int cop_warn_trigger;
	int cop_current_offset;
	int cop_saved_offset;
	struct delayed_work cop_warn_work;

	/* tie up to charger mode */
	struct gvotable_election *gbms_mode;

	/* debug fs */
	struct dentry *debug_entry;

	int wlc_dc_fcc;

	/* fcc retry */
	int fcc_retries;
	int fcc_retry_limit;
	struct delayed_work fcc_retry_work;

};

#define gcpm_psy_name(psy) \
	((psy) && (psy)->desc && (psy)->desc->name ? (psy)->desc->name : "???")

/* TODO: rename to "can_dc" and handle capabilities based on index number */
#define gcpm_is_dc(gcpm, index) \
	((index) >= GCPM_INDEX_DC_ENABLE)

/* Logging ----------------------------------------------------------------- */

int debug_printk_prlog = LOGLEVEL_INFO;

/* ------------------------------------------------------------------------- */

static struct gvotable_election *gcpm_get_cp_votable(struct gcpm_drv *gcpm)
{
	if (!gcpm->cp_votable) {
		struct gvotable_election *v;

		v = gvotable_election_get_handle("GCPM_FCC");
		if (!IS_ERR_OR_NULL(v))
			gcpm->cp_votable = v;
	}

	return gcpm->cp_votable;
}

static struct gvotable_election *gcpm_get_dc_icl_votable(struct gcpm_drv *gcpm)
{
	if (!gcpm->dc_icl_votable) {
		struct gvotable_election *v;

		v = gvotable_election_get_handle("DC_ICL");
		if (!IS_ERR_OR_NULL(v))
			gcpm->dc_icl_votable = v;
	}

	return gcpm->dc_icl_votable;
}

static struct gvotable_election *gcpm_get_fcc_votable(struct gcpm_drv *gcpm)
{
	if (!gcpm->fcc_votable) {
		struct gvotable_election *v;

		v = gvotable_election_get_handle("MSC_FCC");
		if (!IS_ERR_OR_NULL(v))
			gcpm->fcc_votable = v;
	}

	return gcpm->fcc_votable;
}

/* will kick gcpm_fcc_callback(), needs mutex_unlock(&gcpm->chg_psy_lock); */
static int gcpm_update_gcpm_fcc(struct gcpm_drv *gcpm, const char *reason,
				int limit, bool enable)
{
	struct gvotable_election *el;
	int ret = -ENODEV;

	el = gcpm_get_cp_votable(gcpm);
	if (el)
		ret = gvotable_cast_int_vote(el, reason, limit, enable);

	return ret;
}

/* current limit for DC charging */
static int gcpm_get_gcpm_fcc(struct gcpm_drv *gcpm)
{
	struct gvotable_election *el;
	int dc_iin = -1;

	el = gcpm_get_cp_votable(gcpm);
	if (el)
		dc_iin = gvotable_get_current_int_vote(el);
	if (dc_iin < 0)
		dc_iin = gcpm->cc_max;

	return dc_iin;
}

/* ------------------------------------------------------------------------- */

static struct power_supply *gcpm_chg_get_charger(const struct gcpm_drv *gcpm, int index)
{
	return (index < 0 || index >= gcpm->chg_psy_count) ? NULL : gcpm->chg_psy_avail[index];
}

static struct power_supply *gcpm_chg_get_default(const struct gcpm_drv *gcpm)
{
	return gcpm_chg_get_charger(gcpm, GCPM_DEFAULT_CHARGER);
}

/* TODO: place a lock around the operation? */
static struct power_supply *gcpm_chg_get_active(const struct gcpm_drv *gcpm)
{
	return gcpm_chg_get_charger(gcpm, gcpm->chg_psy_active);
}

static bool gcpm_chg_is_cp_active(const struct gcpm_drv *gcpm)
{
	return gcpm_is_dc(gcpm, gcpm->chg_psy_active);
}

/* !=NULL if the adapter is not CP */
static struct power_supply *gcpm_chg_get_active_cp(const struct gcpm_drv *gcpm)
{
	struct power_supply *psy = NULL;

	if (gcpm_chg_is_cp_active(gcpm))
		psy = gcpm_chg_get_charger(gcpm, gcpm->chg_psy_active);

	return psy;
}


static int gcpm_chg_ping(struct gcpm_drv *gcpm, int index, bool online)
{
	struct power_supply *chg_psy;
	int ret;

	chg_psy = gcpm->chg_psy_avail[index];
	if (!chg_psy)
		return 0;

	ret = GPSY_SET_PROP(chg_psy, POWER_SUPPLY_PROP_ONLINE, 0);
	if (ret < 0)
		pr_debug("adapter %d cannot ping (%d)", index, ret);

	return 0;
}

/* use the charger one when avalaible or fallback to the generated one */
static uint64_t gcpm_get_charger_state(const struct gcpm_drv *gcpm,
				       struct power_supply *chg_psy)
{
	union gbms_charger_state chg_state;
	int rc;

	rc = gbms_read_charger_state(&chg_state, chg_psy);
	if (rc < 0)
		return 0;

	return chg_state.v;
}

static int gcpm_resume_check(struct gcpm_drv *gcpm)
{
	int ret = 0;

	pm_runtime_get_sync(gcpm->device);
	if (!gcpm->init_complete || !gcpm->resume_complete)
		ret = -EAGAIN;
	pm_runtime_put_sync(gcpm->device);

	return ret;
}

/*
 * chg_psy_active==-1 if index was active
 * NOTE: GBMS_PROP_CHARGING_ENABLED will be pinged later on
 */
static int gcpm_chg_offline(struct gcpm_drv *gcpm, int index)
{
	const int active_index = gcpm->chg_psy_active;
	struct power_supply *chg_psy;
	int ret;

	ret = gcpm_update_gcpm_fcc(gcpm, "CC_MAX", gcpm->cc_max, false);
	if (ret < 0)
		pr_debug("PPS_DC: offline cannot update cp_fcc (%d)\n", ret);


	chg_psy = gcpm_chg_get_charger(gcpm, index);
	if (!chg_psy)
		return 0;

	/* OFFLINE should stop charging */
	ret = GPSY_SET_PROP(chg_psy, GBMS_PROP_CHARGING_ENABLED, 0);
	if (ret == 0)
		ret = GPSY_SET_PROP(chg_psy, POWER_SUPPLY_PROP_ONLINE, 0);
	if (ret == 0 && gcpm->chg_psy_active == index)
		gcpm->chg_psy_active = -1;

	pr_info("%s: %s active=%d->%d offline_ok=%d\n", __func__,
		 pps_name(chg_psy), active_index, gcpm->chg_psy_active, ret == 0);

	return ret;
}

/* preset charging parameters */
static int gcpm_chg_preset(struct power_supply *chg_psy, int fv_uv, int cc_max)
{
	const char *name = gcpm_psy_name(chg_psy);
	int ret;

	pr_debug("%s: %s fv_uv=%d cc_max=%d\n", __func__, name, fv_uv, cc_max);

	ret = GPSY_SET_PROP(chg_psy, POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX,
			    fv_uv);
	if (ret < 0) {
		pr_err("%s: %s no fv_uv (%d)\n", __func__, name, ret);
		return ret;
	}

	ret = GPSY_SET_PROP(chg_psy, POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
			    cc_max);
	if (ret < 0)
		pr_err("%s: %s no cc_max (%d)\n", __func__, name, ret);

	return ret;
}

/* setting online might start charging (if ENABLE is set) */
static int gcpm_chg_online(struct power_supply *chg_psy, int fv_uv, int cc_max)
{
	const char *name = gcpm_psy_name(chg_psy);
	bool preset_ok = true;
	int ret;

	if (!chg_psy) {
		pr_err("%s: invalid charger\n", __func__);
		return -EINVAL;
	}

	/* preset the new charger */
	ret = gcpm_chg_preset(chg_psy, fv_uv, cc_max);
	if (ret < 0)
		preset_ok = false;

	/* online (but so we can enable it) */
	ret = GPSY_SET_PROP(chg_psy, POWER_SUPPLY_PROP_ONLINE, 1);
	if (ret < 0) {
		pr_debug("%s: %s online failed (%d)\n", __func__, name, ret);
		return ret;
	}

	/* retry preset if failed */
	if (!preset_ok)
		ret = gcpm_chg_preset(chg_psy, fv_uv, cc_max);
	if (ret < 0) {
		int rc;

		pr_err("%s: %s preset failed (%d)\n", __func__, name, ret);

		rc = GPSY_SET_PROP(chg_psy, POWER_SUPPLY_PROP_ONLINE, 0);
		if (rc < 0)
			pr_err("%s: %s offline failed (%d)\n", __func__, name, rc);
	}

	return ret;
}

/*
 * gcpm->chg_psy_active == gcpm->dc_index on success.
 * NOTE: call with a lock around gcpm->chg_psy_lock
 */
static int gcpm_chg_start(struct gcpm_drv *gcpm, int index, int fv_uv, int cc_max)
{
	const int active_index = gcpm->chg_psy_active;
	struct power_supply *chg_psy;
	int ret = -EINVAL;

	if (index == active_index)
		return 0;

	if (active_index != -1)
		pr_err("%s: %d->%d not idle\n", __func__, active_index, index);

	/* validate the index before switch */
	chg_psy = gcpm_chg_get_charger(gcpm, index);
	if (chg_psy)
		ret = gcpm_chg_online(chg_psy, fv_uv, cc_max);
	if (ret < 0) {
		/* TODO: force active_index if != -1 */
		pr_debug("%s: index=%d not online (%d)\n",
			 __func__, index, ret);
		return ret;
	}

	pr_debug("%s: active=%d->%d\n", __func__, active_index, index);

	gcpm->chg_psy_active = index;
	return ret;
}

/*
 * Enable DirectCharge mode, PPS and DC charger must be already initialized
 * NOTE: disable might restart the default charger with stale settings
 */
static int gcpm_dc_enable(struct gcpm_drv *gcpm, bool enabled)
{
	if (gcpm->dcen_gpio >= 0 && !gcpm->dcen_gpio_default)
		gpio_set_value(gcpm->dcen_gpio, enabled);

	if (!gcpm->gbms_mode) {
		struct gvotable_election *v;

		v = gvotable_election_get_handle(GBMS_MODE_VOTABLE);
		if (IS_ERR_OR_NULL(v))
			return -ENODEV;
		gcpm->gbms_mode = v;
	}

	return gvotable_cast_long_vote(gcpm->gbms_mode, "GCPM",
				       GBMS_CHGR_MODE_CHGR_DC, enabled);
}

/*
 * disable DC and switch back to the default charger. Final DC statate is
 * DC_IDLE (i.e. this can be used to reset dc_state from DC_DISABLED).
 * NOTE: call with a lock around gcpm->chg_psy_lock
 * NOTE: I could pass in and return dc_state instead of changing gcpm
 * must  hold a lock on mutex_lock(&gcpm->chg_psy_lock);
 */
static int gcpm_dc_stop(struct gcpm_drv *gcpm, int index)
{
	int dc_state = gcpm->dc_state;
	int ret = 0;

	if (!gcpm_is_dc(gcpm, index))
		dc_state = DC_ENABLE_PASSTHROUGH;

	switch (dc_state) {
	case DC_RUNNING:
	case DC_PASSTHROUGH:
		ret = gcpm_chg_offline(gcpm, index);
		if (ret < 0)
			pr_warn("DC_PPS: Cannot offline DC index=%d (%d)",
				index, ret);
		else
			gcpm->dc_state = DC_ENABLE;
		fallthrough;
	case DC_ENABLE:
	case DC_ENABLE_PASSTHROUGH:
		ret = gcpm_dc_enable(gcpm, false);
		if (ret < 0) {
			pr_err("DC_PPS: Cannot disable DC (%d)", ret);
			break;
		}
		fallthrough;
	default:
		gcpm->dc_state = DC_DISABLED;
		break;
	}

	return ret;
}

/*
 * route the dc_limit to MSC_FCC for wireless charing.
 * @return <0 on error, 0 on limit not applied, 1 on limit applied (and positive)
 * call holding a lock on mutex_lock(&gcpm->chg_psy_lock);
 */
static int gcpm_dc_fcc_update(struct gcpm_drv *gcpm, int value)
{
	struct gvotable_election *msc_fcc;
	int limit = value;
	int ret = -ENODEV;

	msc_fcc = gcpm_get_fcc_votable(gcpm);
	if (!msc_fcc)
		goto error_exit;

	/* apply/enable DC_FCC only when a WLC_DC source is selected */
	if ((gcpm->pps_index != PPS_INDEX_WLC) ||
	    (gcpm->dc_index <= GCPM_DEFAULT_CHARGER) || limit < 0)
		limit = -1;

	/*
	 * The thermal voter for FCC wired must be disabled to allow higher
	 * charger rates for DC_FCC than for the wired case.
	 */
	ret = gvotable_cast_int_vote(msc_fcc, "DC_FCC", limit, limit >= 0);
	if (ret < 0)
		pr_err("%s: vote %d on MSC_FCC failed (%d)\n",  __func__,
		       limit, ret);
	else
		ret = limit >= 0;

error_exit:
	dev_dbg(gcpm->device, "%s: DC_FCC->MSC_FCC pps_index=%d value=%d limit=%d applied=%d\n",
		__func__, gcpm->pps_index, value, limit, ret);

	return ret;
}

/*
 * route the dc_limit to MSC_FCC for wireless charging and adjust the MDIS
 * limits when switching between CP and non CP charging.
 * NOTE: when in MDIS is at level = 0 the cooling zone disable the MDIS votes
 * on MSC_FCC, DC_ICL and GCPM_FCC.
 * NOTE: called with negative cp_limit when switching from WLC_CP to WLC and
 * with the HOLD limit when re-starting PPS_DC and WLC_DC.
 * @return <0 on error, 0 on limit not applied, 1 on limit applied and positive
 * call holding a lock on mutex_lock(&gcpm->chg_psy_lock);
 */
static int gcpm_update_votes(struct gcpm_drv *gcpm, int cp_limit)
{
	const bool enable = gcpm->thermal_device.current_level > 0;
	struct gvotable_election *el;
	int ret;

	pr_debug("%s: cp_limit=%d\n", __func__, cp_limit);

	/* update DC_FCC limit before disabling the others */
	if (cp_limit > 0)
		ret = gcpm_dc_fcc_update(gcpm, enable ? cp_limit : -1);

	/* vote on DC_ICL */
	el = gcpm_get_dc_icl_votable(gcpm);
	if (el)
		gvotable_recast_ballot(el, "MDIS", enable && cp_limit == 0);

	/* vote on MSC_FCC: applied only when CP is not enabled */
	el = gcpm_get_fcc_votable(gcpm);
	if (el)
		gvotable_recast_ballot(el, "MDIS", enable && cp_limit == 0);

	/* vote on GCPM_FCC: valid only on cp */
	el = gcpm_get_cp_votable(gcpm);
	if (el)
		gvotable_recast_ballot(el, "MDIS", enable && cp_limit);

	/* update DC_FCC limit after enabling the others */
	if (cp_limit <= 0)
		ret = gcpm_dc_fcc_update(gcpm, enable ? cp_limit : -1);

	return ret;
}


/* NOTE: call with a lock around gcpm->chg_psy_lock */
static int gcpm_dc_start(struct gcpm_drv *gcpm, int index)
{
	const int dc_iin = gcpm_get_gcpm_fcc(gcpm);
	struct power_supply *dc_psy;
	int ret;

	pr_info("PPS_DC: index=%d dc_iin=%d hold=%d\n",
		index, dc_iin, gcpm->cp_fcc_hold_limit);

	/* ENABLE will be called by the dc_pps workloop */
	ret = gcpm_chg_start(gcpm, index, gcpm->fv_uv, dc_iin);
	if (ret < 0) {
		pr_err("PPS_DC: index=%d not started (%d)\n", index, ret);
		return ret;
	}

	/*
	 * Restoring the DC_FCC limit might change charging current and cause
	 * demand to fall under dc_limit_demand. The possible resulting loop
	 * (enable/disable) is solved in gcpm_chg_select_work().
	 * NOTE: ->cp_fcc_hold_limit cannot be 0
	 */
	ret = gcpm_update_votes(gcpm, gcpm->cp_fcc_hold_limit);
	if (ret < 0)
		pr_debug("PPS_DC: start cannot update votes (%d)\n", ret);

	ret = gcpm_update_gcpm_fcc(gcpm, "CC_MAX", gcpm->cc_max, true);
	if (ret < 0)
		pr_debug("PPS_DC: start cannot update cp_fcc (%d)\n", ret);

	if (gcpm->pps_index == PPS_INDEX_WLC) {
		gcpm_update_gcpm_fcc(gcpm, "WLC_FCC", gcpm->wlc_dc_fcc, gcpm->wlc_dc_fcc > 0);
		pr_info("PPS_DC: index=%d vote gcpm_fcc to %d\n",
			 gcpm->pps_index, gcpm->wlc_dc_fcc);
	}

	/* this is the CP */
	dc_psy = gcpm_chg_get_active_cp(gcpm);
	if (!dc_psy) {
		pr_err("PPS_DC: gcpm->dc_state == DC_READY, no adapter\n");
		return -ENODEV;
	}

	/* set IIN_CFG (might not need) */
	ret = GPSY_SET_PROP(dc_psy, POWER_SUPPLY_PROP_CURRENT_MAX,
			    gcpm->out_ua);
	if (ret < 0) {
		pr_err("PPS_DC: no IIN (%d)\n", ret);
		return ret;
	}

	/* vote on MODE */
	ret = gcpm_dc_enable(gcpm, true);
	if (ret < 0) {
		pr_err("PPS_DC: dc_ready failed=%d\n", ret);
		return ret;
	}

	pr_debug("PPS_DC: dc_ready ok state=%d fv_uv=%d cc_max=%d, out_ua=%d\n",
		gcpm->dc_state, gcpm->fv_uv, gcpm->cc_max, gcpm->out_ua);

	return 0;
}

/*
 * Select the DC charger using the thermal policy.
 * DC charging is enabled when demand is over dc_limit (default 0) and
 * vbatt > vbatt_min (default or device tree). DC is not disabled when
 * vbatt is over vbat low.
 * DC is stopped when vbatt is over vbatt_max (default or DT) and not started
 * when vbatt is over vbatt_high (some default 200mV under vbatt_max).
 * NOTE: program target before enabling chaging.
 */
enum gcpm_dc_ctl_t {
	GCPM_DC_CTL_DEFAULT = 0,
	GCPM_DC_CTL_DISABLE_WIRED,
	GCPM_DC_CTL_DISABLE_WIRELESS,
	GCPM_DC_CTL_DISABLE_BOTH,
};

/*
 * the current source as index in mdis_in[].
 * < 0 error, the index in mdis_in[] if the source is in PPS mode
 */
static int gcpm_mdis_match_cp_source(struct gcpm_drv *gcpm, int *online)
{
	union power_supply_propval pval;
	int i, ret;

	for (i = 0; i < MDIS_IN_MAX; i++) {
		if (!gcpm->mdis_in[i])
			continue;

		ret = power_supply_get_property(gcpm->mdis_in[i],
						POWER_SUPPLY_PROP_ONLINE,
						&pval);
		if (ret || !pval.intval)
			continue;

		*online = pval.intval;
		return i;
	}

	return -EINVAL;
}

static int gcpm_mdis_in_is_wireless(struct gcpm_drv *gcpm, int index)
{
	return index == 1; /* TODO: query at startup using type==WIRELESS */
}

/* return the PPS_CP or the WLC_CP limit */
static int gcpm_chg_select_check_cp_limit(struct gcpm_drv *gcpm)
{
	int online, cp_min = -1, in_idx;

	in_idx = gcpm_mdis_match_cp_source(gcpm, &online);
	if (in_idx < 0 || gcpm_mdis_in_is_wireless(gcpm, in_idx)) {
		if (gcpm->dc_limit_cc_min_wlc >= 0)
			cp_min = gcpm->dc_limit_cc_min_wlc;
	} else if (gcpm->dc_limit_cc_min >= 0) {
		cp_min = gcpm->dc_limit_cc_min;
	}

	/* wlc might use a different (higher) CP limit than wired */
	dev_dbg(gcpm->device, "%s: in_idx=%d cp_min=%d\n", __func__, in_idx, cp_min);
	return cp_min;
}

/* call holding mutex_lock(&gcpm->chg_psy_lock) */
static int gcpm_chg_select_by_demand(struct gcpm_drv *gcpm)
{
	int cc_max = gcpm->cc_max; /* from google_charger */
	int index = GCPM_DEFAULT_CHARGER;
	int batt_demand = -1;
	int cp_min;

	/*
	 * ->cc_max is lowered from the main-charger thermal limit and might
	 * prevent this code from selecting the CP charger again when thermals
	 * caused this code to switch from CP to the main charger.
	 *
	 * NOTE: Need to check the value directly because source selection is
	 * done holding a lock on &gcpm->chg_psy_lock (cc_max will become the
	 * same as gcpm->cp_fcc_hold_limit on exit).
	 */
	if (gcpm->cp_fcc_hold && gcpm->cp_fcc_hold_limit >= 0 && cc_max != 0) {
		/*
		 * ->cp_fcc_hold is set when a thermal limit caused the switch
		 * from CP to main-charger. In this case ->cp_fcc_hold_limit
		 * keeps the current (alternate) CP limit that should be used
		 * to determine whether to go back to the Charge Pump.
		 * NOTE: ->cp_fcc_hold_limit is changed when the DC_FCC changes
		 * (wireless CP) AND when MDIS changes but is not changed when
		 * the MSC_FCC limit changes. This means that without MDIS
		 * CP will restart on PD wired only when the actual charging
		 * current exceeds the cp_min limit.
		 */
		dev_dbg(gcpm->device, "%s: change due to hold cc_max=%d->%d\n",
			__func__, cc_max, gcpm->cp_fcc_hold_limit);
		cc_max = gcpm->cp_fcc_hold_limit;
	}

	/* keeps on default charger until we have valid charging parameters */
	if (cc_max <= 0 || gcpm->fv_uv <= 0)
		goto exit_done; /* index == GCPM_DEFAULT_CHARGER; */

	/*
	 * power demand comes from charging tier or thermal limit: leave
	 * dc_limit_demand to 0 to switch only on cp_min.
	 * TODO: handle capabilities based on index number
	 */
	batt_demand = (cc_max / 1000) * (gcpm->fv_uv / 1000);
	if (batt_demand > gcpm->dc_limit_demand)
		index = GCPM_INDEX_DC_ENABLE;

	dev_dbg(gcpm->device, "%s: index=%d cc_max=%d gcpm->fv_uv=%d demand=%d, dc_limit=%d\n",
		__func__, index, cc_max / 1000, gcpm->fv_uv / 1000,
		batt_demand, gcpm->dc_limit_demand);

	/*
	 * the limit for DC depends on the source that is active with the
	 * complication that using the DC_ICL disables the THERMAL limit
	 * on MSC_FCC and will cause an immediate reselection of CP.
	 * The code settig ->hold and ->cp_fcc_hold_limit needs to make sure
	 * that the limit is appropriate.
	 */
	cp_min = gcpm_chg_select_check_cp_limit(gcpm);
	if (cp_min == -1 || cc_max <= cp_min) {
		const bool cp_active = gcpm_chg_is_cp_active(gcpm);

		/* current demand less than min demand for CP */
		dev_dbg(gcpm->device,
			"%s: cc_max=%d under cp_min=%d, ->hold=%d->%d index:%d->%d\n",
			__func__, cc_max, cp_min, gcpm->cp_fcc_hold,
			gcpm->cp_fcc_hold ? gcpm->cp_fcc_hold : cp_active,
			index, GCPM_DEFAULT_CHARGER);

		/*
		 * Switch to the default charger and hold it.
		 * NOTE: ->cp_fcc_hold is reset in gcpm_dc_fcc_callback()
		 * and when the MDIS thermal limit changes. This piece is
		 * only for legacy dc_fcc since the mdis code handle this.
		 */
		if (!gcpm->cp_fcc_hold)
			gcpm->cp_fcc_hold = cp_active;

		index = GCPM_DEFAULT_CHARGER;
	}

exit_done:
	dev_dbg(gcpm->device,
		"by_d: index:%d->%d demand=%d,limit=%d cc_max=%d,cp_min=%d, hold=%d",
		gcpm->dc_index, index, batt_demand, gcpm->dc_limit_demand,
		cc_max, cp_min, gcpm->cp_fcc_hold);
	return index;
}

/*
 * called only before enabling DC to debounce HIGH SOC.
 * call holding mutex_lock(&gcpm->chg_psy_lock)
 */
static int gcpm_chg_select_by_soc(struct power_supply *psy,
				  const struct gcpm_drv *gcpm)
{
	union power_supply_propval pval = { };
	int index = gcpm->dc_index; /* debounce it */
	int ret;

	ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_CAPACITY, &pval);
	if (ret < 0 || pval.intval < gcpm->dc_limit_soc_high)
		index = GCPM_INDEX_DC_ENABLE;

	pr_debug("%s: index=%d->%d ret=%d soc=%d limit=%d\n", __func__,
		 gcpm->dc_index, index, ret, pval.intval,
		 gcpm->dc_limit_soc_high);

	return index;
}

/* call holding mutex_lock(&gcpm->chg_psy_lock) */
static int gcpm_chg_select_by_voltage(struct power_supply *psy,
				      struct gcpm_drv *gcpm)
{
	int vbatt_min = gcpm->dc_limit_vbatt_min;
	const int vbatt_max = gcpm->dc_limit_vbatt_max;
	const int vbatt_high = gcpm->dc_limit_vbatt_high;
	int vbatt_low = gcpm->dc_limit_vbatt_low;
	int index = GCPM_DEFAULT_CHARGER;
	int vbatt = -1;
	int online = 0;
	int in_idx;

	in_idx = gcpm_mdis_match_cp_source(gcpm, &online);
	if (gcpm_mdis_in_is_wireless(gcpm, in_idx)) {
		vbatt_min = gcpm->wlc_dc_limit_vbatt_min;
		vbatt_low = gcpm->wlc_dc_limit_vbatt_low;
	}

	if (!vbatt_min && !vbatt_max)
		return GCPM_INDEX_DC_ENABLE;

	vbatt = GPSY_GET_PROP(psy, POWER_SUPPLY_PROP_VOLTAGE_NOW);
	if (vbatt < 0) {
		pr_err("CHG_CHK cannot read vbatt %d\n", vbatt);
		goto exit_done; /* index == GCPM_DEFAULT_CHARGER; */
	}

	/* vbatt_max is the hard limit */
	if (vbatt_max && vbatt > vbatt_max)
		goto exit_done; /* index == GCPM_DEFAULT_CHARGER; */

	/*
	 * Need to keep checking the voltage when vbatt is under low
	 * to make sure that DC starts at vbatt_min. Also keeps the
	 * same ->dc_index to avoid instability but keep checking
	 * demand.
	 */
	if (vbatt_low && vbatt < vbatt_low) {
		index = gcpm->dc_index == GCPM_DEFAULT_CHARGER ?
			-EAGAIN : gcpm->dc_index; /* debounce */
	} else if (vbatt_min && vbatt < vbatt_min) {
		index = gcpm->dc_index == GCPM_DEFAULT_CHARGER ?
			-EAGAIN : gcpm->dc_index; /* debounce */
	} else if (vbatt_high && vbatt > vbatt_high) {
		index = gcpm->dc_index; /* debounce */
	} else {
		/* vbatt_min <= vbatt <= vbatt_high */
		index = GCPM_INDEX_DC_ENABLE;
	}

exit_done:
	if (gcpm->dc_index == GCPM_INDEX_DC_ENABLE && index == GCPM_DEFAULT_CHARGER)
		pr_info("%s: index=%d->%d vbatt=%d: low=%d min=%d high=%d max=%d\n",
			__func__, gcpm->dc_index, index, vbatt, vbatt_low, vbatt_min,
			vbatt_high, vbatt_max);
	else
		pr_debug("%s: index=%d->%d vbatt=%d: low=%d min=%d high=%d max=%d\n",
			 __func__, gcpm->dc_index, index, vbatt, vbatt_low, vbatt_min,
			 vbatt_high, vbatt_max);

	return index;
}

/* call holding mutex_lock(&gcpm->chg_psy_lock) */
static int gcpm_chg_select(struct gcpm_drv *gcpm)
{
	int index = GCPM_DEFAULT_CHARGER;
	int ret;

	if (!gcpm->dc_init_complete)
		goto exit_done; /* index == GCPM_DEFAULT_CHARGER; */

	/* overrides cp_fcc_hold, might trigger taper_control */
	if (gcpm->force_active >= 0)
		return gcpm->force_active;

	/* kill switch */
	if (gcpm->dc_ctl == GCPM_DC_CTL_DISABLE_BOTH)
		goto exit_done; /* index == GCPM_DEFAULT_CHARGER; */

	ret = gvotable_get_current_int_vote(gcpm->dc_chg_avail_votable);
	dev_dbg(gcpm->device, "%s: dc_chg_avail vote: %d\n", __func__, ret);
	if (ret <= 0)
		goto exit_done;

	/*
	 * check demand first to react to thermal engine, then voltage to
	 * make sure that we are over min and that we don't start over high
	 * and we stop at max, finally use SOC to not restart if over a
	 * SOC%
	 */
	index = gcpm_chg_select_by_demand(gcpm);
	if (index == GCPM_INDEX_DC_ENABLE) {
		struct power_supply *chg_psy;

		/* checking the current charger, should check battery? */
		chg_psy = gcpm_chg_get_default(gcpm);
		if (chg_psy) {
			index = gcpm_chg_select_by_voltage(chg_psy, gcpm);
			if (index == GCPM_INDEX_DC_ENABLE)
				index = gcpm_chg_select_by_soc(chg_psy, gcpm);
		}
	}

exit_done:

	/* consistency check */
	if (index >= gcpm->chg_psy_count) {
		pr_err("CHG_CHK index=%d out of bounds %d\n", index,
		       gcpm->chg_psy_count);
		index = GCPM_DEFAULT_CHARGER;
	}

	return index;
}

static bool gcpm_chg_dc_check_source(const struct gcpm_drv *gcpm, int index)
{
	/* Will run detection only the first time */
	if (gcpm->tcpm_pps_data.stage == PPS_NOTSUPP &&
	    gcpm->wlc_pps_data.stage == PPS_NOTSUPP )
		return false;

	return gcpm_is_dc(gcpm, index);
}

/* reset gcpm pps state */
static void gcpm_pps_online(struct gcpm_drv *gcpm)
{
	/* reset setpoint */
	gcpm->out_ua = -1;
	gcpm->out_uv = -1;

	/* reset detection */
	if (gcpm->tcpm_pps_data.pps_psy) {
		pps_init_state(&gcpm->tcpm_pps_data);
		if (gcpm->dc_ctl & GCPM_DC_CTL_DISABLE_WIRED)
			gcpm->tcpm_pps_data.stage = PPS_NOTSUPP;
	}
	if (gcpm->wlc_pps_data.pps_psy) {
		pps_init_state(&gcpm->wlc_pps_data);
		if (gcpm->dc_ctl & GCPM_DC_CTL_DISABLE_WIRELESS)
			gcpm->wlc_pps_data.stage = PPS_NOTSUPP;
	}
	gcpm->pps_index = 0;
}

static struct pd_pps_data *gcpm_pps_data(struct gcpm_drv *gcpm)
{
	struct pd_pps_data *pps_data = NULL;

	if (gcpm->pps_index == PPS_INDEX_TCPM)
		pps_data = &gcpm->tcpm_pps_data;
	else if (gcpm->pps_index == PPS_INDEX_WLC)
		pps_data = &gcpm->wlc_pps_data;

	return pps_data;
}

/* Wait for a source to become ready for the handoff */
static int gcpm_pps_wait_for_ready(struct gcpm_drv *gcpm)
{
	struct pd_pps_data *pps_data = gcpm_pps_data(gcpm);
	int pps_ui = 0, vout = -1, iout = -1;
	bool pwr_ok = false;

	if (!pps_data)
		return -ENODEV;

	/* determine the limit/levels if needed */
	if (gcpm->pps_index == PPS_INDEX_WLC && !gcpm->no_init_wlc_ta_vol) {
		struct power_supply *chg_psy = gcpm_chg_get_active(gcpm);
		int vbatt = -1;

		if (chg_psy)
			vbatt = GPSY_GET_PROP(chg_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW);
		if (vbatt > 0)
			vout = vbatt * 4;
	} else {
		pwr_ok = true;
	}

	/* always need to ping */
	if (gcpm->pps_index != PPS_INDEX_WLC || !gcpm->no_init_wlc_ta_vol)
		pps_ui = pps_update_adapter(pps_data, vout, iout, pps_data->pps_psy);
	if (pps_ui < 0) {
		pr_err("PPS_Work: pps update, dc_state=%d (%d)\n",
			gcpm->dc_state, pps_ui);
		return pps_ui;
	}

	/* wait until adapter is at or over request */
	pwr_ok |= (vout <=0 || pps_data->out_uv >= vout) &&
		  (iout <=0 || pps_data->op_ua >= iout );

	pr_info("PPS_Work: pwr_ok=%d pps_ui=%d vout=%d out_uv=%d iout=%d op_ua=%d\n",
		pwr_ok, pps_ui, vout, pps_data->out_uv, iout, pps_data->op_ua);

	return pwr_ok ? pps_ui : -EAGAIN;
}

/* return true when  pd_online=PROG_ONLINE and stage=ACTIVE */
static int gcpm_pps_check_active(struct pd_pps_data *pps_data)
{
	int pps_ui;

	/* not supported for this stage */
	if (pps_data->stage == PPS_NOTSUPP)
		return 0;

	/* <0 for error, 0 for done, or the next polling interval */
	pps_ui = pps_work(pps_data, pps_data->pps_psy);
	if (pps_data->pd_online < PPS_PSY_PROG_ONLINE)
		pr_debug("PPS_Work: TCPM Wait %s pps_ui=%d online=%d, stage=%d\n",
			pps_name(pps_data->pps_psy), pps_ui, pps_data->pd_online,
			pps_data->stage);

	return pps_ui >= 0 && pps_data->stage == PPS_ACTIVE;
}

/*
 * Debounce online, enable PROG on each of the sources (ping) source that
 * transition to PPS_ACTIVE and set the pps_index.
 *
 * ->stage ==
 *	DISABLED => NONE -> AVAILABLE -> ACTIVE -> DISABLED
 *		 -> DISABLED
 *		 -> NOTSUPP
 *
 * return -EAGAIN if none of the sources is online, -ENODEV none of the sources
 * supports PPS, 0 if one of the sources is ONLINE an
 */
static int gcpm_pps_work(struct gcpm_drv *gcpm)
{
	int rc, ret = 0, online = 0, pps_index = 0;

	/*
	 * rc>0 when source has .pd_online==PROG_ONLINE and .stage==ACTIVE
	 * ,stage is active when the source has provided the source caps.
	 */
	rc =  gcpm_pps_check_active(&gcpm->tcpm_pps_data);
	if (rc <= 0) {
		rc =  gcpm_pps_check_active(&gcpm->wlc_pps_data);
		if (rc > 0)
			pps_index = PPS_INDEX_WLC;
		else
			online |= gcpm->wlc_pps_data.pd_online != 0;

		online |= gcpm->tcpm_pps_data.pd_online != 0;
	} else {
		pps_index = PPS_INDEX_TCPM;
	}

	if (gcpm->pps_index != pps_index)
		logbuffer_log(gcpm->log, "PPS_Work: pps_index %d->%d\n",
			      gcpm->pps_index, pps_index);

	/* handles PPS source offline or NOT active anymore */
	if (pps_index == 0 && gcpm->pps_index)
		ret = -ENODEV;
	else if (pps_index == 0 && !online)
		ret = -EAGAIN;

	pr_debug("PPS_Work: tcpm[online=%d, stage=%d] wlc[online=%d, stage=%d] ol=%d ret=%d pps_index=%d->%d\n",
		gcpm->tcpm_pps_data.pd_online, gcpm->tcpm_pps_data.stage,
		gcpm->wlc_pps_data.pd_online, gcpm->wlc_pps_data.stage,
		online, ret, gcpm->pps_index, pps_index);

	gcpm->pps_index = pps_index;
	return ret;
}

static int gcpm_pps_timeout(struct gcpm_drv *gcpm)
{
	struct pd_pps_data *wlc_pps_data = &gcpm->wlc_pps_data;

	return (wlc_pps_data->stage != PPS_NOTSUPP && wlc_pps_data->pd_online)
		? PPS_ACTIVE_WLC_TIMEOUT_S : PPS_ACTIVE_USB_TIMEOUT_S;
}

static int gcpm_update_mdis_charge_cntl_limit(struct mdis_thermal_device *tdev,
					      unsigned long lvl);


static int gcpm_pps_offline(struct gcpm_drv *gcpm)
{
	int ret;
	struct mdis_thermal_device *tdev = &gcpm->thermal_device;

	/* TODO: migh be a no-op when pps_index == 0 */

	if (gcpm->tcpm_pps_data.pps_psy) {
		ret = pps_prog_offline(&gcpm->tcpm_pps_data,
				       gcpm->tcpm_pps_data.pps_psy);
		if (ret < 0)
			pr_err("PPS_DC: fail tcpm offline (%d)\n", ret);
	}

	if (gcpm->wlc_pps_data.pps_psy) {
		ret = pps_prog_offline(&gcpm->wlc_pps_data,
				       gcpm->wlc_pps_data.pps_psy);
		if (ret < 0)
			pr_err("PPS_DC: fail wlc offline (%d)\n", ret);
	}

	/* force the callback set dc -> main thermal limits */
	if (tdev && gcpm->mdis_votable)
		gvotable_run_election(gcpm->mdis_votable, true);

	gcpm->pps_index = 0;
	return 0;
}

/* <=0 to disable, > 0 to enable "n" counts */
static bool gcpm_taper_ctl(struct gcpm_drv *gcpm, int count)
{
	bool changed = false;

	if (count <= 0) {
		changed = gcpm->taper_step != 0;
		gcpm->taper_step = 0;
	} else if (gcpm->taper_step == 0) {
		gcpm->taper_step = count;
		changed = true;
	}

	return changed;
}

/*
 * taper off charging current to ease the transition out of CP charging.
 * NOTE: this writes directly to the charging current.
 */
static bool gcpm_taper_step(const struct gcpm_drv *gcpm,
			   int dc_iin, int taper_step)
{
	const int delta = gcpm->taper_step_count - taper_step;
	int fv_uv = gcpm->fv_uv, cc_max = dc_iin;
	struct power_supply *dc_psy;

	if (taper_step <= 0)
		return true;
	/*
	 * TODO: on a race between TAPER and select, active might not
	 * be a DC source. Force done to prevent voltage spikes.
	 */
	dc_psy = gcpm_chg_get_active_cp(gcpm);
	if (!dc_psy)
		return true;

	/* Optional dc voltage limit */
	if (gcpm->taper_step_voltage) {
		int vbatt;

		vbatt = GPSY_GET_PROP(dc_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW);
		if (vbatt < 0)
			pr_err("%s: cannot read voltage (%d)", __func__, vbatt);
		else if (vbatt < gcpm->taper_step_voltage)
			return false;
	}

	/* Optional dc current limit */
	if (gcpm->taper_step_current) {
		int ret, ibatt;

		/* TODO: use current average if available */
		ibatt = GPSY_GET_INT_PROP(dc_psy, POWER_SUPPLY_PROP_CURRENT_NOW,
					  &ret);
		if (ret < 0)
			pr_err("%s: cannot read current (%d)", __func__, ret);
		else if (ibatt > gcpm->taper_step_current)
			return false;
	}

	/* delta < 0 during the grace period, which will increase cc_max */
	fv_uv -= gcpm->taper_step_fv_margin;
	if (gcpm->taper_step_cc_step) {
		cc_max -= delta * gcpm->taper_step_cc_step;
		if (cc_max < dc_iin / 2)
			cc_max = dc_iin / 2;
	}

	/* increase of cc_max due to delta < 0 are ignored */
	if (cc_max < dc_iin) {
		int ret;

		/* failure to preset stop taper and revert to main */
		ret = gcpm_chg_preset(dc_psy, fv_uv, cc_max);
		pr_info("CHG_CHK: taper_step=%d fv_uv=%d->%d, dc_iin=%d->%d\n",
			 taper_step, gcpm->fv_uv, fv_uv, dc_iin, cc_max);
		if (ret < 0) {
			pr_err("CHG_CHK: taper_step=%d failed, revert (%d)\n",
			       taper_step, ret);
			return true;
		}

	} else {
		pr_debug("CHG_CHK: grace taper_step=%d fv_uv=%d, dc_iin=%d\n",
			 taper_step, gcpm->fv_uv, dc_iin);
	}

	logbuffer_log(gcpm->log, "taper_step=%d delta=%d fv_uv=%d->%d, dc_iin=%d->%d",
		      taper_step, delta, gcpm->fv_uv, fv_uv, dc_iin, cc_max);


	/* not done */
	return false;
}

/* needs mutex_lock(&gcpm->chg_psy_lock); */
static int gcpm_chg_select_logic(struct gcpm_drv *gcpm)
{
	int index, schedule_pps_interval = -1;
	bool dc_done = false, dc_ena;

	dev_dbg(gcpm->device, "%s: init_ok=%d dc_state=%d dc_index=%d\n", __func__,
		 gcpm->dc_init_complete, gcpm->dc_state, gcpm->dc_index);

	if (!gcpm->dc_init_complete)
		return -EAGAIN;

	index = gcpm_chg_select(gcpm);
	if (index < 0) {
		pr_debug("%s: index=%d dc_state=%d dc_index=%d\n",
			 __func__, index, gcpm->dc_state, gcpm->dc_index);
		return -EAGAIN;
	}

	/* will not try to enable if the source cannot do PPS */
	dc_ena = gcpm_chg_dc_check_source(gcpm, index);

	/*
	 * taper control reduces cc_max every gcpm->taper_step_interval seconds
	 * by a fixed amount for gcpm->taper_step_count seconds. fv_uv might
	 * also be lowered by a fixed amount.
	 */
	if (dc_ena && gcpm->taper_step > 0) {
		const int interval = msecs_to_jiffies(gcpm->taper_step_interval * 1000);
		int dc_iin = gcpm->cc_max;

		dc_done = gcpm_taper_step(gcpm, dc_iin, gcpm->taper_step - 1);
		if (!dc_done) {
			mod_delayed_work(system_wq, &gcpm->select_work, interval);
			gcpm->taper_step -= 1;
			gcpm->taper_step_used = true;
		}

		if (dc_done)
			pr_info("%s: taper_step=%d done=%d\n", __func__,
				gcpm->taper_step, dc_done);
		else
			pr_debug("%s: taper_step=%d done=%d\n", __func__,
				 gcpm->taper_step, dc_done);
	} else if (gcpm->taper_step_used && gcpm->taper_step != 0) {
		const int vbatt_high = gcpm->dc_limit_vbatt_high;

		/* reset dc_state after taper step */
		gcpm_taper_ctl(gcpm, 0);
		if (gcpm->fv_uv < vbatt_high && gcpm->dc_state == DC_DISABLED)
			gcpm->dc_state = DC_IDLE;
	}

	pr_debug("%s: DC dc_ena=%d dc_state=%d dc_index=%d->%d taper_step=%d\n",
		 __func__, dc_ena, gcpm->dc_state, gcpm->dc_index, index,
		 gcpm->taper_step);

	/*
	 * NOTE: disabling DC might need to transition to charger mode 0
	 * same might apply when switching between WLC-DC and PPS-DC.
	 * Figure out a way to do this if needed.
	 */
	if (!dc_ena || dc_done) {

		if (gcpm->dc_state > DC_IDLE && gcpm->dc_index > 0) {
			pr_info("CHG_CHK: dc_ena=%d dc_done=%d stop PPS_Work for dc_index=%d\n",
				dc_ena, dc_done, gcpm->dc_index);

			/*
			 * dc_done will prevent DC to restart until disconnect
			 * or voltage goes over _high.
			 */
			gcpm->dc_index = dc_done ? GCPM_INDEX_DC_DISABLE :
					 GCPM_DEFAULT_CHARGER;
			gcpm_taper_ctl(gcpm, 0);
			schedule_pps_interval = 0;
		}
	} else if (gcpm->dc_state == DC_DISABLED) {
		/*
		 * dc is disabled when we are done OR when the source doesn't
		 * support PPS or failed the authentication.
		 */
		pr_debug("%s: PPS_Work disabled for the session\n", __func__);
	} else if (gcpm->dc_state == DC_IDLE) {
		const ktime_t dc_start_time = get_boot_sec();

		pr_info("CHG_CHK: start PPS_Work for dc_index=%d at %lld\n",
			 index, dc_start_time);

		/* reset pps state to re-enable detection */
		gcpm_pps_online(gcpm);

		/* TODO: DC_ENABLE or DC_PASSTHROUGH depending on index */
		gcpm->dc_state = DC_ENABLE_PASSTHROUGH;
		gcpm->dc_index = index;

		/* grace period of 500ms, PPS Work not called during grace */
		gcpm->dc_start_time = dc_start_time;
		schedule_pps_interval = DC_ENABLE_DELAY_MS;

		__pm_stay_awake(gcpm->gcpm_ws);
		pr_debug("%s: pm gcpm stay awake\n", __func__);
	}

	if (schedule_pps_interval >= 0) {
		pr_debug("%s: DC schedule pps_work in %ds\n", __func__,
			 schedule_pps_interval / 1000);

		mod_delayed_work(system_wq, &gcpm->pps_work,
				 msecs_to_jiffies(schedule_pps_interval));
	}

	return 0;
}

/*
 * triggered on every FV_UV and in DC_PASSTHROUGH
 * will keep polling if in -EAGAIN
 */
static void gcpm_chg_select_work(struct work_struct *work)
{
	struct gcpm_drv *gcpm =
		container_of(work, struct gcpm_drv, select_work.work);
	int ret;

	mutex_lock(&gcpm->chg_psy_lock);

	pr_debug("%s: on=%d dc_state=%d dc_index=%d\n", __func__,
		 gcpm->dc_init_complete, gcpm->dc_state, gcpm->dc_index);

	ret = gcpm_chg_select_logic(gcpm);
	if (ret == -EAGAIN) {
		const int interval = 5; /* 5 seconds */

		mod_delayed_work(system_wq, &gcpm->select_work,
				 msecs_to_jiffies(interval * 1000));
	}

	mutex_unlock(&gcpm->chg_psy_lock);
}

static int gcpm_enable_default(struct gcpm_drv *gcpm)
{
	struct power_supply *chg_psy = gcpm_chg_get_default(gcpm);
	int ret;

	/* gcpm_chg_offline set GBMS_PROP_CHARGING_ENABLED = 0 */
	ret = GPSY_SET_PROP(chg_psy, GBMS_PROP_CHARGING_ENABLED, 1);
	if (ret < 0) {
		pr_debug("%s: failed 2 enable charging (%d)\n", __func__, ret);
		return ret;
	}

	/* (re) online and start the default charger */
	ret = gcpm_chg_start(gcpm, GCPM_DEFAULT_CHARGER, gcpm->fv_uv, gcpm->cc_max);
	if (ret < 0) {
		pr_debug("%s: failed 2 start (%d)\n", __func__, ret);
		return ret;
	}

	return 0;
}

/* online the default charger (do not change active, nor enable) */
static int gcpm_online_default(struct gcpm_drv *gcpm)
{
	return gcpm_chg_online(gcpm_chg_get_default(gcpm), gcpm->fv_uv, gcpm->cc_max);
}

/*
 * restart the default charger after DC or while trying to start it.
 * Can come here during DC_ENABLE_PASSTHROUGH, with PPS enabled and
 * after a failure to start DC or on a failure to disable the default
 * charger.
 *
 * NOTE: the caller needs to reset gcpm->dc_index
 */
static int gcpm_pps_wlc_dc_restart_default(struct gcpm_drv *gcpm)
{
	const int active_index = gcpm->chg_psy_active; /* will change */
	const int dc_state = gcpm->dc_state; /* will change */
	int pps_done, ret;

	/* DC_FCC limit might be enabled as soon as we enter WLC_DC */
	ret = gcpm_update_votes(gcpm, 0);
	if (ret < 0)
		pr_err("PPS_DC: wlc_dc_rd cannot update votes (%d)\n", ret);

	pr_debug("PPS_DC: gcpm_update_gcpm_fcc unvote\n");
	gcpm_update_gcpm_fcc(gcpm, "WLC_FCC", 0, false);

	/* Clear taper count if not complete */
	gcpm_taper_ctl(gcpm, 0);

	/*
	 * in dc_state=DC_ENABLE_PASSTHROUGH it might  be able to take
	 * the current charger offline BUT might fail to start DC.
	 */
	if (dc_state <= DC_IDLE)
		return 0;

	/* online the default charger (do not change active, nor enable)
	 * TODO: possibly do nothing if the current charger is not DC.
	 */
	ret = gcpm_online_default(gcpm);
	if (ret < 0)
		pr_warn("%s: Cannot online default (%d)", __func__, ret);

	/*
	 * dc_state=DC_DISABLED, chg_psy_active==-1 a DC charger was active.
	 * in DC_ENABLE_PASSTHROUGH, gcpm_dc_stop() will vote on charger mode.
	 */
	ret = gcpm_dc_stop(gcpm, active_index);
	if (ret < 0) {
		pr_debug("%s: retry disable, dc_state=%d->%d (%d)\n",
			 __func__, dc_state, gcpm->dc_state, ret);
		return -EAGAIN;
	}

	/*
	 * Calling pps_offline is not really needed becasuse the adapter will
	 * revert to fixed once ping stops (pps state is re-initialized on
	 * DC start). I clear it to keep things neat and tidy.
	 *
	 * NOTE: Make sure that pps_prog_offline only changes from PROG to
	 * FIXED and not from OFFLINE to FIXED. Setting WLC from OFFLINE
	 * to FIXED (online) at the wrong time might interfere with
	 * the usecases that need to disable charging explicitly.
	 */
	pps_done = gcpm_pps_offline(gcpm);
	if (pps_done < 0)
		pr_debug("%s: fail 2 offline pps, dc_state=%d (%d)\n",
			__func__, gcpm->dc_state, pps_done);

	ret = gcpm_enable_default(gcpm);
	if (ret < 0) {
		pr_err("%s: fail 2 restart default, dc_state=%d pps_done=%d (%d)\n",
		       __func__, gcpm->dc_state, pps_done >= 0 ? : pps_done, ret);
		return -EAGAIN;
	}

	return 0;
}

/*
 * pps_data->stage:
 *  PPS_NONE -> PPS_AVAILABLE -> PPS_ACTIVE
 * 	     -> PPS_DISABLED  -> PPS_DISABLED
 * acquires mutex_lock(&gcpm->chg_psy_lock);
 */
static void gcpm_pps_wlc_dc_work(struct work_struct *work)
{
	struct gcpm_drv *gcpm =
		container_of(work, struct gcpm_drv, pps_work.work);
	struct pd_pps_data *pps_data;
	int ret, pps_ui = -ENODEV;
	ktime_t elap;

	/* spurious during init */
	mutex_lock(&gcpm->chg_psy_lock);

	elap = gcpm->dc_start_time <= 0 ? 0 : get_boot_sec() - gcpm->dc_start_time;

	pr_debug("%s: ok=%d dc_index=%d dc_state=%d dc_start_time=%lld\n",
		 __func__, gcpm->resume_complete && gcpm->init_complete,
		 gcpm->dc_index, gcpm->dc_state, gcpm->dc_start_time);

	if (!gcpm->resume_complete || !gcpm->init_complete) {
		pps_ui = DC_ERROR_RETRY_MS;
		goto pps_dc_reschedule;
	}

	/* disconnect, gcpm_chg_check() and most errors reset ->dc_index */
	if (gcpm->dc_index <= 0) {
		const int active_index = gcpm->chg_psy_active; /* will change */
		const bool dc_disable = gcpm->dc_index == GCPM_INDEX_DC_DISABLE;

		pr_debug("%s: stop for gcpm->dc_index=%d\n", __func__, gcpm->dc_index);

		/* will leave gcpm->dc_state in DC_DISABLED */
		ret = gcpm_pps_wlc_dc_restart_default(gcpm);
		if (ret < 0) {
			pr_warn("PPS_Work: retry restart elap=%lld dc_state=%d %d->%d (%d)\n",
				elap, gcpm->dc_state, active_index,
				gcpm->chg_psy_active, ret);

			pps_ui = DC_ERROR_RETRY_MS;
			goto pps_dc_reschedule;
		}

		/* Re-enable DC if just switching to the default charger */
		if (!dc_disable)
			gcpm->dc_state = DC_IDLE;

		gcpm->dc_start_time = 0;

		gbms_logbuffer_prlog(gcpm->log, LOGLEVEL_INFO, 0, debug_printk_prlog,
				     "PPS_Work: done%selap=%lld dc_state=%d %d->%d\n",
				     dc_disable ? "for the session " : " ",
				     elap, gcpm->dc_state, active_index,
				     gcpm->chg_psy_active);

		pr_debug("%s: pm gcpm relax\n", __func__);
		__pm_relax(gcpm->gcpm_ws);

		/* TODO: send a ps event? */
		goto pps_dc_done;
	}

	/* PPS was handed over to the DC driver, just monitor it... */
	if (gcpm->dc_state == DC_PASSTHROUGH) {
		struct power_supply *dc_psy;
		bool prog_online = false;
		int index;

		/* the dc driver needs to keep the source online */
		pps_data = gcpm_pps_data(gcpm);
		if (pps_data)
			prog_online = pps_check_prog_online(pps_data);
		if (!prog_online) {
			pr_err("PPS_Work: PPS offline, elap=%lld dc_index:%d->0\n",
			       elap, gcpm->dc_index);

			gcpm->dc_index = GCPM_DEFAULT_CHARGER;
			pps_ui = DC_ERROR_RETRY_MS;
			goto pps_dc_reschedule;
		}

		/* likely changed from debug, bail */
		dc_psy = gcpm_chg_get_active(gcpm);
		if (!dc_psy) {
			pr_err("PPS_Work: No adapter, elap=%lld in PASSTHROUGH\n",
			       elap);

			pps_ui = DC_ERROR_RETRY_MS;
			goto pps_dc_reschedule;
		}

		/* something is changed: kick the revert to default */
		index = gcpm_chg_select(gcpm);
		if (index != gcpm->dc_index)
			mod_delayed_work(system_wq, &gcpm->select_work, 0);

		/* ->pps_index valid: set/ping source to DC, ping watchdog */
		ret = GPSY_SET_PROP(dc_psy, GBMS_PROP_CHARGING_ENABLED,
				    gcpm->pps_index);
		if (ret == 0) {
			ret = gcpm_chg_ping(gcpm, GCPM_DEFAULT_CHARGER, 0);
			if (ret < 0)
				pr_err("PPS_Work: ping failed, elap=%lld with %d\n",
				       elap, ret);

			/* keep running to ping the adapters */
			pps_ui = DC_RUN_DELAY_MS;
		} else if (ret == -EBUSY || ret == -EAGAIN) {
			pps_ui = DC_ERROR_RETRY_MS;
		} else {
			pr_err("PPS_Work: ping DC failed, elap=%lld (%d)\n", elap, ret);
			ret = gcpm_chg_offline(gcpm, gcpm->dc_index);
			if (ret == 0)
				ret = gcpm_enable_default(gcpm);
			if (ret < 0) {
				pr_err("PPS_Work: cannot online default %d\n", ret);
				pps_ui = DC_ERROR_RETRY_MS;
			 } else {
				pr_err("PPS_Work: dc offline\n");
				pps_ui = 0;

				pr_debug("%s: pm gcpm relax\n", __func__);
				__pm_relax(gcpm->gcpm_ws);
			}
		}

		goto pps_dc_reschedule;
	}

	/*
	 * Wait until one of the sources becomes online AND switch to prog
	 * mode. gcpm_pps_work will return <0 when PPS is not supported from
	 * ANY source. Deadline to PPS_PROG_TIMEOUT_S.
	 */
	ret = gcpm_pps_work(gcpm);
	if (ret < 0) {
		if (elap < PPS_PROG_TIMEOUT_S) {
			pr_debug("PPS_Work: PROG elap=%lld ret=%d retry\n", elap, ret);

			/* retry for the session  */
			pps_ui = PPS_PROG_RETRY_MS;
			gcpm_pps_online(gcpm);
		} else {
			pr_err("PPS_Work: PROG timeout, elap=%lld dc_state=%d (%d)\n",
			       elap, gcpm->dc_state, ret);

			/* abort for the session  */
			gcpm->dc_index = GCPM_INDEX_DC_DISABLE;
			pps_ui = PPS_ERROR_RETRY_MS;
		}

		goto pps_dc_reschedule;
	}

	/*
	 * DC runs only when PPS is active (ie. online=PROG_ONLINE and
	 * ->stage=PPS_ACTIVE). Abort for the session if a source
	 * went PROG_ONLINE but is not active.
	 */
	pps_data = gcpm_pps_data(gcpm);
	if (!pps_data) {
		int timeout_s = gcpm_pps_timeout(gcpm);

		if (elap < timeout_s) {
			pr_debug("PPS_Work: ACTIVE elap=%lld ret=%d retry\n", elap, ret);

			/* WLC + Auth might require a very long time */
			pps_ui = PPS_ACTIVE_RETRY_MS;
		} else {
			pr_err("PPS_Work: ACTIVE timeout=%d, start=%lld elap=%lld dc_state=%d (%d)\n",
			       timeout_s, elap, gcpm->dc_start_time, gcpm->dc_state, ret);

			/* abort for the session (until disconnect) */
			gcpm->dc_index = GCPM_INDEX_DC_DISABLE;
			pps_ui = PPS_ERROR_RETRY_MS;
		}

		goto pps_dc_reschedule;
	}

	if (gcpm->dc_state == DC_ENABLE_PASSTHROUGH) {
		int timeout_s = gcpm_pps_timeout(gcpm) + PPS_READY_DELTA_TIMEOUT_S;
		int index;
		struct mdis_thermal_device *tdev = &gcpm->thermal_device;

		/* Also ping the source */
		pps_ui = gcpm_pps_wait_for_ready(gcpm);
		if (pps_ui < 0) {
			pr_info("PPS_Work: wait for source timeout=%d elap=%lld, dc_state=%d (%d)\n",
				timeout_s, elap, gcpm->dc_state, pps_ui);
			if (pps_ui != -EAGAIN)
				gcpm->dc_index = GCPM_DEFAULT_CHARGER;
			if (elap > timeout_s)
				gcpm->dc_index = GCPM_DEFAULT_CHARGER;

			/* error retry */
			pps_ui = PPS_ERROR_RETRY_MS;
			goto pps_dc_reschedule;
		}

		/*
		 * source selection might have changed demand and disabled DC
		 * (WLC_DC has a different mincurrent). Revert the input
		 * selection, retry when ->cp_fcc_hold_limit changes.
		 */
		index = gcpm_chg_select(gcpm);
		if (!gcpm_is_dc(gcpm, index)) {
			pr_info("PPS_Work: selection changed index=%d\n", index);

			gcpm->dc_index = GCPM_DEFAULT_CHARGER;
			pps_ui = PPS_ERROR_RETRY_MS;
			goto pps_dc_reschedule;
		}

		/* force the callback set main -> dc thermal limits */
		if (tdev && gcpm->mdis_votable)
			gvotable_run_election(gcpm->mdis_votable, true);

		/*
		 * offine current adapter and start new. Charging is enabled
		 * in DC_PASSTHROUGH setting GBMS_PROP_CHARGING_ENABLED to
		 * the PPS source.
		 * TODO: preset the DC charger before handoff
		 * NOTE: There are a bunch of interesting recovery scenarios.
		 */
		ret = gcpm_chg_offline(gcpm, gcpm->chg_psy_active);
		if (ret == 0)
			ret = gcpm_dc_start(gcpm, gcpm->dc_index);
		if (ret == 0) {
			gcpm->dc_state = DC_PASSTHROUGH;
			pps_ui = DC_ENABLE_DELAY_MS;
		} else if (pps_ui > DC_ERROR_RETRY_MS) {
			pps_ui = DC_ERROR_RETRY_MS;
		}

		pr_debug("%s: pm gcpm relax\n", __func__);
		__pm_relax(gcpm->gcpm_ws);
	} else {
		struct power_supply *pps_psy = pps_data->pps_psy;

		/* steady on PPS, if DC state is DC_ENABLE or DC_RUNNING */
		pps_ui = pps_update_adapter(pps_data, -1, -1, pps_psy);

		pr_info("PPS_Work: STEADY pd_online=%d pps_ui=%d dc_ena=%d dc_state=%d\n",
			pps_data->pd_online, pps_ui, gcpm->dc_index,
			gcpm->dc_state);
		if (pps_ui < 0)
			pps_ui = PPS_ERROR_RETRY_MS;
	}

pps_dc_reschedule:
	if (pps_ui <= 0) {
		pr_debug("PPS_Work: pps_ui=%d dc_index=%d dc_state=%d",
			 pps_ui, gcpm->dc_index, gcpm->dc_state);
	} else {
		pr_debug("PPS_Work: reschedule in %d dc_index=%d dc_state=%d (%d:%d)",
			 pps_ui, gcpm->dc_index, gcpm->dc_state, gcpm->out_uv, gcpm->out_ua);

		schedule_delayed_work(&gcpm->pps_work, msecs_to_jiffies(pps_ui));
	}

pps_dc_done:
	mutex_unlock(&gcpm->chg_psy_lock);
}

/*
 * coming in though the old dc_fcc votable.
 *
 * TODO: reimplement in terms of MDIS level remapping the limit
 */
static int gcpm_dc_fcc_callback(struct gvotable_election *el,
				const char *reason,
				void *value)
{
	struct gcpm_drv *gcpm = gvotable_get_data(el);
	const int limit = (long)value;
	int changed = gcpm->cp_fcc_hold_limit != limit;
	int applied;

	mutex_lock(&gcpm->chg_psy_lock);

	/*
	 * ->cp_fcc_hold_limit is updated from MDIS and from the DC_FCC code.
	 *
	 * applied=1 here when the dc_limit has been applied to MSC_FCC and we
	 * need to re-run the selection. Here the limit is applied ONLY when
	 * using WLC_CP.
	 *
	 * NOTE: the thermal engine needs to vote on mdis_chg OR on dc_fcc,
	 * dc_icl and msc_fcc.
	 */
	applied = gcpm_dc_fcc_update(gcpm, limit);
	if (applied < 0)
		pr_err("%s: cannot enforce DC_FCC limit applied=%d\n",
			__func__, applied);
	else if (applied)
		gcpm->cp_fcc_hold_limit = limit;

	/*
	 * ->cp_fcc_hold will be set in gcpm_chg_select_by_demand() for WLC_DC
	 * sessions when cc_max has fallen under the limit for WLC_DC charging
	 * (->dc_limit_cc_min_wlc). The hold keeps the device charging with
	 * the default charger (ie. non CP ie WLC) until released.
	 *
	 * Clearing the ->cp_fcc_hold when the thermal limit changes and is
	 * NON zero allows gcpm_chg_select_by_demand() to re-evaluate
	 * returning to CP charging.
	 */
	if (limit != 0 && gcpm->cp_fcc_hold) {
		gcpm->cp_fcc_hold = false;
		changed += 1;
	}

	pr_debug("%s: CPM_THERM_DC_FCC limit=%d hold=%d applied=%d changed=%d\n",
		 __func__, limit, gcpm->cp_fcc_hold, applied, changed);

	/*
	 * ->cp_fcc_hold force the selection of GCPM_DEFAULT_CHARGER in
	 * gcpm_chg_select_by_demand().
	 */
	if (applied || changed)
		mod_delayed_work(system_wq, &gcpm->select_work,
				 msecs_to_jiffies(DC_ENABLE_DELAY_MS));

	mutex_unlock(&gcpm->chg_psy_lock);
	return 0;
}

static int gcpm_dc_chg_avail_callback(struct gvotable_election *el,
				      const char *reason, void *value)
{
	struct gcpm_drv *gcpm = gvotable_get_data(el);
	const int dc_chg_avail = GVOTABLE_PTR_TO_INT(value);

	if (!gcpm->init_complete)
		return 0;

	mod_delayed_work(system_wq, &gcpm->select_work, 0);
	pr_debug("DC_CHG_AVAIL: dc_avail=%d, reason=%s\n", dc_chg_avail, reason);

	return 0;
}

static int gcpm_route_to_main_charger(struct gcpm_drv *gcpm, enum power_supply_property psp,
				      const union power_supply_propval *pval)
{
	struct power_supply *main_chg_psy = NULL;
	int ret;

	/* Route to main charger */
	main_chg_psy = gcpm_chg_get_default(gcpm);
	if (!main_chg_psy) {
		pr_err("invalid default charger for psp=%d\n", psp);
		return -EIO;
	}

	ret = power_supply_set_property(main_chg_psy, psp, pval);
	if (ret < 0 && ret != -EAGAIN)
		pr_err("cannot route prop:%d to default:%s (%d)\n",
		       psp, gcpm_psy_name(main_chg_psy), ret);

	return ret;
}

static int gcpm_reset_dc(struct gcpm_drv *gcpm)
{
	int i;

	/* send reset event to other dc chargers on disconnect */
	for (i = 0; i < gcpm->chg_psy_count; i++) {
		if (!gcpm->chg_psy_avail[i] || !gcpm_is_dc(gcpm, i))
			continue;
		GPSY_SET_PROP(gcpm->chg_psy_avail[i], GBMS_PROP_CHARGE_DISABLE, 1);
	}

	return 0;
}
/* --------------------------------------------------------------------- */

static int gcpm_set_active_charger(struct gcpm_drv *gcpm,
				   enum power_supply_property psp,
				   const union power_supply_propval *pval,
				   bool ta_check, bool route, bool cc_max_changed)
{
	struct power_supply *chg_psy = NULL;
	int ret = 0;

	/* used only for debug */
	if (gcpm->new_dc_limit) {
		gcpm->new_dc_limit = false;
		ta_check = true;
	}

	/*
	 * ta_check is set when the charging parameters change (cc_max, fv_uv)
	 * when changing the online state, in taper control and when charging
	 * is disabled. this code triggers the logic that selects DC charging
	 * or that causes charging to switch back the main charger.
	 */
	if (gcpm->dc_init_complete && ta_check) {
		const bool was_dc = gcpm_is_dc(gcpm, gcpm->dc_index);
		int rc;

		/*
		 * Synchronous! might kick off gcpm_pps_wlc_dc_work to negotiate
		 * DC charging. -EAGAIN will cause this code to be called again.
		 * NOTE: gcpm_chg_select_logic() might change gcpm->dc_index
		 */
		rc = gcpm_chg_select_logic(gcpm);
		if (rc == -EAGAIN) {
			const int interval = 5; /* seconds */

			/* let the setting go through but */
			mod_delayed_work(system_wq, &gcpm->select_work,
					msecs_to_jiffies(interval * 1000));
		}

		 /*
		  * Do not route while switching from DC to non DC because
		  * the DC charger might get the wrong limits.
		  * NOTE: gcpm_pps_wlc_dc_work() will configure the new charger
		  * on start (or on stop/timeout)
		  */
		if (was_dc && !gcpm_is_dc(gcpm, gcpm->dc_index))
			route = false;
	}

	/*  route to active charger only when needed */
	if (!route)
		goto done;

	chg_psy = gcpm_chg_get_active(gcpm);
	if (chg_psy) {
		/* replace the pval with dc_iin limit when DC is selected */
		ret = GPSY_SET_PROP(chg_psy, psp, pval->intval);
		if (ret < 0 && ret != -EAGAIN) {
			pr_err("cannot route prop=%d to %d:%s (%d)\n", psp,
				gcpm->chg_psy_active, gcpm_psy_name(chg_psy),
				ret);
		}
	} else {
		pr_err("invalid active charger = %d for prop=%d\n",
			gcpm->chg_psy_active, psp);
	}

	/* Write USB ICL and Voltage to main charger also to lift CHGIN_SUSP */
	if (psp == POWER_SUPPLY_PROP_CURRENT_MAX || psp == POWER_SUPPLY_PROP_VOLTAGE_MAX) {
		chg_psy = gcpm_chg_get_default(gcpm);
		ret = power_supply_set_property(chg_psy, psp, pval);
		if (ret < 0 && ret != -EAGAIN) {
			pr_err("cannot route prop=%d to %d:%s (%d)\n", psp,
				gcpm->chg_psy_active, gcpm_psy_name(chg_psy),
				ret);
		}
	}

done:
	/*
	 * route==false when using CP and when transitioning OUT of it.
	 * Will disable CC_MAX vote on GCPM_FCC when/if the limit is routed
	 * to the main-charger.
	 */
	if (psp == POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX) {
		int cc_max = gcpm->cc_max + gcpm->cop_current_offset;

		if (gcpm->cop_current_offset && cc_max_changed && (cc_max >= 0)) {
			pr_info("COP warn throttling cc_max=%d->%d\n",
				gcpm->cc_max + gcpm->cop_saved_offset , cc_max);
		} else {
			if (cc_max < 0)
				pr_err("COP error applying throttling, cur_offset:%d cc_max:%u\n",
					gcpm->cop_current_offset,
					gcpm->cc_max);

			cc_max = gcpm->cc_max;
		}

		if (cc_max_changed && gcpm->cop_supported)
			gcpm_route_to_main_charger(gcpm, psp, pval);
		gcpm_update_gcpm_fcc(gcpm, "CC_MAX", cc_max, !route);

		gcpm->cop_saved_offset = gcpm->cop_current_offset;
	} else if (psp == POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT) {
		ret = gcpm_route_to_main_charger(gcpm, psp, pval);
	}

	return ret;
}


static int gcpm_psy_set_property(struct power_supply *psy,
				 enum power_supply_property psp,
				 const union power_supply_propval *pval)
{
	struct gcpm_drv *gcpm = power_supply_get_drvdata(psy);
	bool ta_check = false;
	bool route = true;
	int ret = 0;
	bool cc_max_changed = false;

	if (gcpm_resume_check(gcpm))
		return -EAGAIN;

	mutex_lock(&gcpm->chg_psy_lock);

	switch (psp) {
	/* do not route to the active charger */
	case POWER_SUPPLY_PROP_ONLINE:
		pr_info("%s: ONLINE value=%d dc_index=%d dc_state=%d\n",
			__func__, pval->intval, gcpm->dc_index,
			gcpm->dc_state);
		ta_check = true;
		break;

	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		route = !gcpm_chg_is_cp_active(gcpm);
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		ta_check = gcpm->fv_uv != pval->intval;
		gcpm->fv_uv = pval->intval;
		break;

	/*
	 * from google_charger (usually) with demand adjusted by classic
	 * thermal engine and/or special charging profiles.
	 * The MDIS vote on MSC_FCC is disabled by the thermal
	 *
	 * Used by the select logic to determine the best charging strategy and
	 * either routed to the main-charger directly or voted on GCPM_FCC.
	 */
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		ta_check = gcpm->cc_max != pval->intval;
		route = !gcpm_chg_is_cp_active(gcpm);

		if (!pval->intval)
			gcpm->cop_current_offset = 0;

		cc_max_changed = ta_check || (gcpm->cop_current_offset != gcpm->cop_saved_offset);
		if (cc_max_changed)
			pr_debug("%s: route=%d ta_check=%d cc_max=%d->%d dc_index=%d"
				 " cop_offset:%d->%d\n",
				 __func__, route, ta_check, gcpm->cc_max, pval->intval,
				 gcpm->dc_index, gcpm->cop_saved_offset, gcpm->cop_current_offset);
		gcpm->cc_max = pval->intval;
		break;

	/* just route to the active charger */
	default:
		break;
	}

	ret = gcpm_set_active_charger(gcpm, psp, pval, ta_check, route, cc_max_changed);

	mutex_unlock(&gcpm->chg_psy_lock);

	/* the charger should not call into gcpm: this can change though */
	return ret;
}

static int gcpm_psy_get_property(struct power_supply *psy,
				 enum power_supply_property psp,
				 union power_supply_propval *pval)
{
	struct gcpm_drv *gcpm = power_supply_get_drvdata(psy);
	struct power_supply *chg_psy;
	bool route = false;
	int ret = 0;

	if (gcpm_resume_check(gcpm))
		return -EAGAIN;

	mutex_lock(&gcpm->chg_psy_lock);
	chg_psy = gcpm_chg_get_active(gcpm);
	if (!chg_psy) {
		pr_err("invalid active charger = %d for prop=%d\n",
			gcpm->chg_psy_active, psp);
		mutex_unlock(&gcpm->chg_psy_lock);
		return -ENODEV;
	}

	switch (psp) {
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		pval->intval = gcpm->cc_max;
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		pval->intval = gcpm->fv_uv;
		break;

	/* route to the active charger */
	default:
		route = true;
		break;
	}

	if (route)
		ret = power_supply_get_property(chg_psy, psp, pval);

	mutex_unlock(&gcpm->chg_psy_lock);
	return ret;
}

static int gcpm_psy_is_writeable(struct power_supply *psy,
				 enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		return 1;
	default:
		break;
	}

	return 0;
}

static int gcpm_gbms_psy_set_property(struct power_supply *psy,
				      enum gbms_property psp,
				      const union gbms_propval *pval)
{
	struct gcpm_drv *gcpm = power_supply_get_drvdata(psy);
	bool ta_check = false;
	bool route = true;
	int ret = 0;
	bool cc_max_changed = false;

	if (gcpm_resume_check(gcpm))
		return -EAGAIN;

	mutex_lock(&gcpm->chg_psy_lock);
	switch (psp) {
	/* do not route to the active charger */
	case GBMS_PROP_TAPER_CONTROL: {
		int count = 0;

		if (pval->prop.intval != GBMS_TAPER_CONTROL_OFF) {
			count = gcpm->taper_step_count + gcpm->taper_step_grace;
			pr_info("%s: TaperControl value=%d\n", __func__, count);
		}

		/* ta_check is set when taper control changes value */
		ta_check = gcpm_taper_ctl(gcpm, count);
		route = false;
	} break;

	/* route to the active charger in most cases */
	case GBMS_PROP_CHARGE_DISABLE:

		/* google_charger send this on disconnect and input_suspend. */
		pr_info("%s: ChargeDisable value=%d dc_index=%d dc_state=%d\n",
			__func__, pval->prop.intval, gcpm->dc_index, gcpm->dc_state);

		if (pval->prop.intval) {
			/*
			 * more or less the same as gcpm_pps_wlc_dc_work() when
			 * dc_index <= 0. But the default charger must not be
			 * restarted in this case though.
			 * TODO: factor the code with gcpm_pps_wlc_dc_work().
			 */

			/*
			 * No op if the current source is not DC (uncluding
			 * stop while in DC_ENABLE_), ->dc_state
			 * will be DC_DISABLED if this was actually disabled.
			 */
			ret = gcpm_dc_stop(gcpm,  gcpm->chg_psy_active);
			if (ret == -EAGAIN) {
				pr_debug("%s: cannot disable, try again\n", __func__);
				mutex_unlock(&gcpm->chg_psy_lock);
				return -EAGAIN;
			}

			ret = gcpm_pps_offline(gcpm);
			if (ret < 0)
				pr_debug("%s: fail 2 offline pps, dc_state=%d (%d)\n",
					__func__, gcpm->dc_state, ret);

			gcpm_reset_dc(gcpm);

			/* reset to the default charger, and clear taper */
			gcpm->dc_index = GCPM_DEFAULT_CHARGER;
			gcpm_taper_ctl(gcpm, 0);
			gcpm->taper_step_used = false;

			/*
			 * no-op if dc was NOT running, set online the charger
			 * but do not start it otherwise.
			 */
			ret = gcpm_chg_start(gcpm, GCPM_DEFAULT_CHARGER,
					     gcpm->fv_uv, gcpm->cc_max);
			if (ret < 0)
				pr_err("%s: cannot start default (%d)\n",
				       __func__, ret);

			pr_info("%s: ChargeDisable value=%d dc_index=%d dc_state=%d"
				" cop_offset=%d->0\n",
				__func__, pval->prop.intval, gcpm->dc_index, gcpm->dc_state,
				gcpm->cop_saved_offset);

			/*
			 * route = true so active will get the property.
			 * No need to re-check the TA selection on disable.
			 */
			ta_check = false;

			/* Cancel COP work on disconnect */
			cancel_delayed_work(&gcpm->cop_warn_work);
			gcpm->cop_warn_count = 0;
			gcpm->cop_current_offset = 0;
			gcpm->cop_saved_offset = 0;
		} else if (gcpm->dc_state <= DC_IDLE) {
			/*
			 * ->dc_state will be DC_DISABLED if DC was disabled
			 * via GBMS_PROP_CHARGE_DISABLE(1) of from other
			 * conditions such as taper control.
			 */
			if (gcpm->dc_state == DC_DISABLED)
				gcpm->dc_state = DC_IDLE;

			pr_info("%s: ChargeDisable value=%d dc_index=%d dc_state=%d\n",
				__func__, pval->prop.intval, gcpm->dc_index, gcpm->dc_state);

			gcpm_pps_online(gcpm);
			ta_check = true;
		}

		break;

	case POWER_SUPPLY_PROP_ONLINE:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		pr_debug("%s: route to gcpm_psy_set_property, psp:%d\n", __func__, psp);
		mutex_unlock(&gcpm->chg_psy_lock);
		return -ENODATA;

	/* just route to the active charger */
	default:
		break;
	}

	ret = gcpm_set_active_charger(gcpm, psp, &pval->prop, ta_check, route, cc_max_changed);

	mutex_unlock(&gcpm->chg_psy_lock);

	/* the charger should not call into gcpm: this can change though */
	return ret;
}

static int gcpm_gbms_psy_get_property(struct power_supply *psy,
				      enum gbms_property psp,
				      union gbms_propval *pval)
{
	struct gcpm_drv *gcpm = power_supply_get_drvdata(psy);
	union gbms_charger_state chg_state;
	struct power_supply *chg_psy;
	bool route = false;
	int ret = 0;

	if (gcpm_resume_check(gcpm))
		return -EAGAIN;

	mutex_lock(&gcpm->chg_psy_lock);
	chg_psy = gcpm_chg_get_active(gcpm);
	if (!chg_psy) {
		pr_err("invalid active charger = %d for prop=%d\n",
			gcpm->chg_psy_active, psp);
		mutex_unlock(&gcpm->chg_psy_lock);
		return -ENODEV;
	}

	switch (psp) {
	/* handle locally for now */
	case GBMS_PROP_CHARGE_CHARGER_STATE:
		chg_state.v = gcpm_get_charger_state(gcpm, chg_psy);
		pval->int64val = chg_state.v;
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		pr_debug("%s: route to gcpm_psy_get_property, psp:%d\n", __func__, psp);
		ret = -ENODATA;
		break;

	/* route to the active charger */
	default:
		route = true;
		break;
	}

	if (route)
		pval->prop.intval = GPSY_GET_INT_PROP(chg_psy, psp, &ret);

	mutex_unlock(&gcpm->chg_psy_lock);
	return ret;
}

static int gcpm_gbms_psy_is_writeable(struct power_supply *psy,
				      enum gbms_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
	case POWER_SUPPLY_PROP_CURRENT_MAX:
	case GBMS_PROP_CHARGING_ENABLED:
	case GBMS_PROP_CHARGE_DISABLE:
	case GBMS_PROP_TAPER_CONTROL:
		return 1;
	default:
		break;
	}

	return 0;
}

/*
 * TODO: POWER_SUPPLY_PROP_RERUN_AICL, POWER_SUPPLY_PROP_TEMP
 */
static enum power_supply_property gcpm_psy_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	/* pixel battery management subsystem */
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,	/* cc_max */
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX,	/* fv_uv */
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_CURRENT_MAX,	/* input current limit */
	POWER_SUPPLY_PROP_VOLTAGE_MAX,	/* input voltage limit*/
	POWER_SUPPLY_PROP_STATUS,
};

static struct gbms_desc gcpm_psy_desc = {
	.psy_dsc.name = "gcpm",
	.psy_dsc.type = POWER_SUPPLY_TYPE_UNKNOWN,
	.psy_dsc.get_property = gcpm_psy_get_property,
	.psy_dsc.set_property = gcpm_psy_set_property,
	.psy_dsc.property_is_writeable = gcpm_psy_is_writeable,
	.get_property = gcpm_gbms_psy_get_property,
	.set_property = gcpm_gbms_psy_set_property,
	.property_is_writeable = gcpm_gbms_psy_is_writeable,
	.psy_dsc.properties = gcpm_psy_properties,
	.psy_dsc.num_properties = ARRAY_SIZE(gcpm_psy_properties),
	.forward = true,
};

#define gcpm_psy_changed_tickle_pps(gcpm) \
	((gcpm)->dc_state == DC_PASSTHROUGH || (gcpm)->dc_state == DC_RUNNING)

static int gcpm_psy_changed(struct notifier_block *nb, unsigned long action,
			    void *data)
{
	struct gcpm_drv *gcpm = container_of(nb, struct gcpm_drv, chg_nb);
	const int index = gcpm->chg_psy_active;
	struct power_supply *psy = data;
	bool tickle_pps_work = false;

	if (index == -1)
		return NOTIFY_OK;

	if ((action != PSY_EVENT_PROP_CHANGED) ||
	    (psy == NULL) || (psy->desc == NULL) || (psy->desc->name == NULL))
		return NOTIFY_OK;

	if (strcmp(psy->desc->name, gcpm->chg_psy_names[index]) == 0) {
		/* route upstream when the charger active and found */
		if (gcpm->chg_psy_avail[index])
			power_supply_changed(gcpm->psy);

		tickle_pps_work = gcpm_psy_changed_tickle_pps(gcpm);
	} else if (strcmp(psy->desc->name, gcpm->chg_psy_names[0]) == 0) {
		/* possibly JEITA or other violation, check PPS */
		tickle_pps_work = gcpm_psy_changed_tickle_pps(gcpm);
	} else if (gcpm->tcpm_psy_name &&
		   !strcmp(psy->desc->name, gcpm->tcpm_psy_name)) {

		/* from tcpm source (even if not selected) */
		tickle_pps_work = gcpm_psy_changed_tickle_pps(gcpm);
	} else if (gcpm->wlc_dc_name &&
	      !strcmp(psy->desc->name, gcpm->wlc_dc_name)) {

		/* from wc source (even if not selected) */
		tickle_pps_work = gcpm_psy_changed_tickle_pps(gcpm);
	}

	/* should tickle the PPS loop only when is running */
	if (tickle_pps_work)
		mod_delayed_work(system_wq, &gcpm->pps_work, 0);

	return NOTIFY_OK;
}

static ssize_t dc_limit_demand_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct gcpm_drv *gcpm = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", gcpm->dc_limit_demand);
}
static ssize_t dc_limit_demand_store(struct device *dev,
                                 struct device_attribute *attr,
                                 const char *buf, size_t count)
{
	struct gcpm_drv *gcpm = dev_get_drvdata(dev);
	int ret = 0;
	u32 val;

	ret = kstrtou32(buf, 0, &val);
	if (ret < 0)
		return ret;

	mutex_lock(&gcpm->chg_psy_lock);
	if (gcpm->dc_limit_demand != val) {
		gcpm->dc_limit_demand = val;
		gcpm->new_dc_limit = true;
	}

	mutex_unlock(&gcpm->chg_psy_lock);

	return count;
}
static DEVICE_ATTR_RW(dc_limit_demand);

static ssize_t dc_limit_vbatt_max_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct gcpm_drv *gcpm = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", gcpm->dc_limit_vbatt_max);
}
static ssize_t dc_limit_vbatt_max_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct gcpm_drv *gcpm = dev_get_drvdata(dev);
	int ret = 0;
	u32 val;

	ret = kstrtou32(buf, 0, &val);
	if (ret < 0)
		return ret;

	gcpm->dc_limit_vbatt_max = val;

	return count;
}
static DEVICE_ATTR_RW(dc_limit_vbatt_max);

static ssize_t dc_limit_vbatt_min_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct gcpm_drv *gcpm = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", gcpm->dc_limit_vbatt_min);
}
static ssize_t dc_limit_vbatt_min_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct gcpm_drv *gcpm = dev_get_drvdata(dev);
	int ret = 0;
	u32 val;

	ret = kstrtou32(buf, 0, &val);
	if (ret < 0)
		return ret;

	gcpm->dc_limit_vbatt_min = val;

	return count;
}
static DEVICE_ATTR_RW(dc_limit_vbatt_min);

static ssize_t wlc_dc_limit_vbatt_min_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct gcpm_drv *gcpm = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", gcpm->wlc_dc_limit_vbatt_min);
}
static ssize_t wlc_dc_limit_vbatt_min_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct gcpm_drv *gcpm = dev_get_drvdata(dev);
	int ret = 0;
	u32 val;

	ret = kstrtou32(buf, 0, &val);
	if (ret < 0)
		return ret;

	gcpm->wlc_dc_limit_vbatt_min = val;

	return count;
}
static DEVICE_ATTR_RW(wlc_dc_limit_vbatt_min);

static ssize_t dc_ctl_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct gcpm_drv *gcpm = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", gcpm->dc_ctl);
}

static ssize_t dc_ctl_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct gcpm_drv *gcpm = dev_get_drvdata(dev);
	int ret = 0, val;

	ret = kstrtoint(buf, 0, &val);
	if (ret < 0)
		return ret;

	/*
	 * 0: enable both (Default)
	 * 1: disable wired-DC
	 * 2: disable wireless-DC
	 * 3: disable both
	 */
	switch (val) {
		case GCPM_DC_CTL_DEFAULT:
		case GCPM_DC_CTL_DISABLE_WIRED:
		case GCPM_DC_CTL_DISABLE_WIRELESS:
		case GCPM_DC_CTL_DISABLE_BOTH:
			gcpm->dc_ctl = val;
			break;
		default:
			return -EINVAL;
	};

	return count;
}
static DEVICE_ATTR_RW(dc_ctl);

static ssize_t thermal_mdis_fan_alarm_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct gcpm_drv *gcpm = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", gcpm->thermal_device.therm_fan_alarm_level);
}

static ssize_t thermal_mdis_fan_alarm_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct gcpm_drv *gcpm = dev_get_drvdata(dev);
	int ret = 0;
	u32 value;

	ret = kstrtou32(buf, 0, &value);
	if (ret < 0)
		return ret;

	if (value <= gcpm->thermal_device.thermal_levels)
		gcpm->thermal_device.therm_fan_alarm_level = value;

	return count;
}
static DEVICE_ATTR_RW(thermal_mdis_fan_alarm);

/* ------------------------------------------------------------------------ */

static int gcpm_get_max_charge_cntl_limit(struct thermal_cooling_device *tcd,
					  unsigned long *lvl)
{
	struct mdis_thermal_device *tdev = tcd->devdata;

	*lvl = tdev->thermal_levels;
	return 0;
}

static int gcpm_get_cur_charge_cntl_limit(struct thermal_cooling_device *tcd,
					  unsigned long *lvl)
{
	struct mdis_thermal_device *tdev = tcd->devdata;

	*lvl = tdev->current_level;
	return 0;
}

#define FAN_MDIS_ALARM_DEFAULT 3
static int fan_get_level(struct mdis_thermal_device *tdev)
{
	int fan_level = FAN_LVL_UNKNOWN;

	if (tdev->current_level <= 0)
		fan_level = FAN_LVL_NOT_CARE;
	else if (tdev->current_level >= tdev->therm_fan_alarm_level)
		fan_level = FAN_LVL_ALARM;
	else
		fan_level = FAN_LVL_MED;

	return fan_level;
}

static int gcpm_mdis_update_fan(struct gcpm_drv *gcpm)
{
	int ret = 0;

	if (!gcpm->fan_level_votable)
		gcpm->fan_level_votable = gvotable_election_get_handle(VOTABLE_FAN_LEVEL);

	if (gcpm->fan_level_votable) {
		const int level = fan_get_level(&gcpm->thermal_device);

		ret = gvotable_cast_int_vote(gcpm->fan_level_votable, "THERMAL_MDIS",
					     level, true);
		if (ret < 0)
			pr_err("%s: cannot update fan level (%d)", __func__, ret);
	}

	return ret;
}

static inline int mdis_cast_vote(struct gvotable_election *el, int vote, bool enabled)
{
	int ret = 0;

	if (enabled)
		ret = gvotable_cast_int_vote(el, "MDIS", vote, true);
	else if (vote >= 0)
		ret = gvotable_cast_int_vote(el, "MDIS", vote, false);
	else
		gvotable_recast_ballot(el, "MDIS", false);

	return ret;
}

static int mdis_set_wlc_online(struct gcpm_drv *gcpm)
{
	struct power_supply *wlc_psy = gcpm->wlc_pps_data.pps_psy;
	union power_supply_propval pval;
	int ret;

	if (!wlc_psy)
		return PPS_PSY_OFFLINE;

	ret = power_supply_get_property(wlc_psy, POWER_SUPPLY_PROP_ONLINE, &pval);
	if (ret < 0 || pval.intval == PPS_PSY_OFFLINE) {

		pval.intval = PPS_PSY_FIXED_ONLINE;
		ret = power_supply_set_property(wlc_psy, POWER_SUPPLY_PROP_ONLINE,
						&pval);
		if (ret < 0)
			return ret;
	}

	return pval.intval;
}

/*
 * A negative msc_fcc, dc_icl or cp_fcc disables the MDIS vote on the
 * corresponding source.
 * cp_fcc=0 re-enable the MDIS votes on MSC_FCC and DC_ICL and forces the
 * transition to MW charging when/if using the charge pump (in this case
 * charging will stop if MSC_FCC/DC_ICL are zero).
 *
 * needs mutex_unlock(&gcpm->chg_psy_lock);
 */
static int gcpm_mdis_update_limits(struct gcpm_drv *gcpm, int msc_fcc,
				   int dc_icl, int cp_fcc)
{
	struct gvotable_election *dc_icl_votable;
	struct gvotable_election *fcc_votable;
	struct gvotable_election *cp_votable;
	int ret;

	pr_info("MSC_MDIS msc_fcc=%d dc_icl=%d cp_fcc=%d\n",
		msc_fcc, dc_icl, cp_fcc);

	cp_votable = gcpm_get_cp_votable(gcpm);
	dc_icl_votable = gcpm_get_dc_icl_votable(gcpm);

	/*
	 * set (or reset) the MDIS limit for CP.
	 * The callback for GCPM_FCC needs to be locked.
	 */
	if (cp_fcc != 0 && cp_votable) {
		ret = mdis_cast_vote(cp_votable, cp_fcc, cp_fcc > 0);
		if (ret < 0)
			dev_err(gcpm->device, "MDIS: vote %d on CP failed (%d)\n",
				cp_fcc, ret);
	}

	/*
	 * set (or reset) the  MDIS limit for MSC_FCC.
	 * Turns off the main-charger from the charging loop in google_charger
	 * but will not be able to restart charging if/when the charging loop
	 * is not running (MSC_FCC might not have a callback that cause a
	 * respin of the usecase state machine)
	 * NOTE: this limit is enabled only when CP is not enabled
	 */
	fcc_votable = gcpm_get_fcc_votable(gcpm);
	if (fcc_votable) {
		ret = mdis_cast_vote(fcc_votable, msc_fcc, msc_fcc >= 0 && cp_fcc == 0);
		if (ret < 0)
			dev_err(gcpm->device, "MDIS: vote %d on MSC_FCC failed (%d)\n",
				msc_fcc, ret);
	}

	/*
	 * set (or reset) the  MDIS limit for DC_ICL.
	 * NOTE: Can vote on DC_ICL even when using CP.
	 */
	if (dc_icl != 0 && dc_icl_votable) {
		int wlc_state;

		/* need to set online WLC if not online */
		wlc_state = mdis_set_wlc_online(gcpm);
		if (wlc_state == PPS_PSY_OFFLINE)
			dev_err(gcpm->device, "MDIS: WLC offine\n");

		/* turning ON after critical level for WLC is complicated */
		ret = mdis_cast_vote(dc_icl_votable, dc_icl, dc_icl > 0);
		if (ret < 0)
			dev_err(gcpm->device, "MDIS: vote %d on DC_ICL failed (%d)\n",
				dc_icl, ret);
	}

	/* adjust limit for RTX */
	if (!gcpm->tx_icl_votable)
		gcpm->tx_icl_votable = gvotable_election_get_handle("TX_ICL");
	if (gcpm->tx_icl_votable)
		gvotable_cast_int_vote(gcpm->tx_icl_votable, "MDIS", 0, dc_icl == 0);

	/*
	 * turns off the CP and will revert to main.
	 * NOTE: The limit for main charger MSC_FCC is updated above.
	 */
	if (cp_fcc == 0 && cp_votable) {
		ret = mdis_cast_vote(cp_votable, 0, true);
		if (ret < 0)
			dev_err(gcpm->device, "MDIS: vote %d on CP failed (%d)\n",
				cp_fcc, ret);
	}

	/* turning off wireless charging equires disabling the wireless IC */
	if (dc_icl == 0 && dc_icl_votable) {
		ret = mdis_cast_vote(dc_icl_votable, 0, true);
		if (ret < 0)
			dev_err(gcpm->device, "vote %d on DC_ICL failed (%d)\n",
				dc_icl, ret);
	}

	/* one or more might fail, consider retries */
	return 0;
}

 /* max dissipation themal level: apply the limit  */
static int gcpm_set_mdis_charge_cntl_limit(struct thermal_cooling_device *tcd,
					   unsigned long lvl)
{
	struct mdis_thermal_device *tdev = tcd->devdata;
	struct gcpm_drv *gcpm = tdev->gcpm;
	struct gvotable_election* el = gcpm->mdis_votable;
	int ret;

	if (tdev->thermal_levels <= 0 || lvl < 0 || lvl > tdev->thermal_levels)
		return -EINVAL;

	mutex_lock(&gcpm->chg_psy_lock);
	ret = gvotable_cast_int_vote(el, REASON_MDIS, lvl, lvl > 0);
	mutex_unlock(&gcpm->chg_psy_lock);

	return ret;
}

static int gcpm_update_mdis_charge_cntl_limit(struct mdis_thermal_device *tdev,
					      unsigned long lvl)
{
	struct gcpm_drv *gcpm = tdev->gcpm;
	int online = 0, in_idx = -1;
	int msc_fcc, dc_icl, cp_fcc, ret;
	bool mdis_crit_lvl;

	if (tdev->thermal_levels <= 0 || lvl < 0 || lvl > tdev->thermal_levels)
		return -EINVAL;

	dev_dbg(gcpm->device, "MSC_THERM_MDIS lvl=%d->%d\n", tdev->current_level, (int)lvl);

	tdev->current_level = lvl;
	mdis_crit_lvl = lvl == tdev->thermal_levels || tdev->thermal_mitigation[lvl] == 0;
	if (mdis_crit_lvl) {
		msc_fcc = dc_icl = cp_fcc = 0;
		gcpm->cp_fcc_hold_limit = gcpm_chg_select_check_cp_limit(gcpm);
		gcpm->cp_fcc_hold = true;
	} else if (tdev->current_level == 0) {
		msc_fcc = dc_icl = cp_fcc = -1;
		/* mdis callback will clear hold and re-evaluate PPS */
		gcpm->cp_fcc_hold_limit = -1;
	} else {
		int cp_min = -1;

		/* 0 always is the main-charger */
		dc_icl = gcpm->mdis_out_limits[0][lvl + tdev->thermal_levels];
		msc_fcc = gcpm->mdis_out_limits[0][lvl];

		/*
		 * cp_fcc limit is routed to DC when DC is selected or ignored.
		 * the code in gcpm_psy_set_property() uses cp_fcc and cc_max
		 * to determine when to swich source.
		 */
		in_idx = gcpm_mdis_match_cp_source(gcpm, &online);
		if (in_idx < 0 || online != PPS_PSY_PROG_ONLINE) {
			/*
			 * this happens when none of the sources are online
			 * or when not using the CP. It CAN happen when we
			 * resume after the thermal engine has shut this down.
			 * Forces cp_fcc to 0 to apply dc_icl and msc_fcc.
			 */
			cp_fcc = 0;

			/* forces wlc-overrides-fcc when wireless charging */
			if (online && gcpm_mdis_in_is_wireless(gcpm, in_idx))
				msc_fcc = -1;
		} else if (gcpm_mdis_in_is_wireless(gcpm, in_idx)) {
			/* WLC_CP use the charge pump with wireless charging */
			cp_fcc = gcpm->mdis_out_limits[1][lvl + tdev->thermal_levels];

			if (gcpm->dc_limit_cc_min_wlc >= 0)
				cp_min = gcpm->dc_limit_cc_min_wlc;
			else if (gcpm->dc_limit_cc_min >= 0)
				cp_min = gcpm->dc_limit_cc_min;

			/*
			 * forces wlc-overrides-fcc when wireless charging
			 * Reset only in PROG_ONLINE to allow transitioning
			 * OUT of WLC_DC when the charging current falls
			 * under the DC limit.
			 */
			msc_fcc = -1;
		} else {
			/* PPS_CP use the charge pump with TCPM */
			cp_fcc = gcpm->mdis_out_limits[1][lvl];
			if (gcpm->dc_limit_cc_min >= 0)
				cp_min = gcpm->dc_limit_cc_min;
		}

		/*
		 * validate the cp limit against cp_min and disable CP
		 * with hold if the new limit is under it.
		 * NOTE: there might be a corner case when the MSC_FCC or the
		 * DC_ICL limit doesn't change after re-enabling the vote.
		 */
		if (cp_min == -1) {
			pr_debug("MSC_MDIS cp_fcc_hold_limit:%d->-1 cp_fcc=%d cp_min=%d\n",
				gcpm->cp_fcc_hold_limit, cp_fcc, cp_min);
		} else if (cp_fcc > cp_min) {
			/* mdis callback will clear hold and re-evaluate PPS */
			gcpm->cp_fcc_hold_limit = -1;
			pr_debug("MSC_MDIS cp_fcc_hold_limit:%d->-1 cp_fcc=%d cp_min=%d\n",
				gcpm->cp_fcc_hold_limit, cp_fcc, cp_min);
		} else if (cp_fcc <= cp_min) {
			/*
			 * setting ->cp_fcc_hold_limit to 0 select the
			  main-charger in gcpm_chg_select_by_demand().
			 */
			gcpm->cp_fcc_hold_limit = gcpm_mdis_in_is_wireless(gcpm, in_idx) ?
						0 : cp_min;
			gcpm->cp_fcc_hold = true;

			pr_debug("MSC_MDIS cp_fcc:%d->0 hold_limit=%d cp_min=%d\n",
				cp_fcc, gcpm->cp_fcc_hold_limit, cp_min);
			cp_fcc = 0;
		}
	}

	dev_info(gcpm->device,
		"MSC_THERM_MDIS lvl=%lu in_idx=%d online=%d cp_fcc=%d hold=%d, hold_limit=%d\n",
		lvl, in_idx, online, cp_fcc, gcpm->cp_fcc_hold,
		gcpm->cp_fcc_hold_limit);

	ret = gvotable_cast_int_vote(gcpm->dc_chg_avail_votable, REASON_MDIS,
				     !mdis_crit_lvl, 1);
	if (ret < 0)
		dev_err(gcpm->device, "Unable to cast vote for DC Chg avail (%d)\n", ret);
	/*
	 * this might be in the callback for mdis_votable
	 * . cp_fcc == 0 will apply msc_fcc, dc_icl and must cause the
	*    transition from CP to MW
	 * . cp_fcc < 0 it only removes the MDIS limit on CP charging
	 * . msc_fcc = -1 when charging from dc_icl (wlc-overrides-fcc)
	 */
	ret = gcpm_mdis_update_limits(gcpm, msc_fcc, dc_icl, cp_fcc);
	if (ret < 0)
		pr_err("%s: cannot update limits (%d)", __func__, ret);

	return 0;
}

static ssize_t
state2power_table_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct thermal_cooling_device *tdev = to_cooling_device(dev);
	struct mdis_thermal_device *mdev = tdev->devdata;
	ssize_t count = 0;
	int i;

	for (i = 0; i < mdev->thermal_levels; i++) {
		const int budgetMw = mdev->thermal_mitigation[i] / 1000;

		count += sysfs_emit_at(buf, count, "%u ", budgetMw);
	}

	/* b/231599097 add the implicit 0 at the end of the table */
	count += sysfs_emit_at(buf, count, "0\n");

	return count;
}

static DEVICE_ATTR_RO(state2power_table);

static ssize_t
mdis_vote_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct gcpm_drv *gcpm = dev_get_drvdata(dev);
	struct gvotable_election* el = gcpm->mdis_votable;
	int lvl;

	lvl = gvotable_get_int_vote(el, MDIS_REASON_SETUP);

	return sprintf(buf, "%d\n", lvl);
}

static ssize_t
mdis_vote_store(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct gcpm_drv *gcpm = dev_get_drvdata(dev);
	struct gvotable_election* el = gcpm->mdis_votable;
	int ret;
	long lvl;

	ret = kstrtol(buf, 0, &lvl);
	if (ret)
		return ret;
	mutex_lock(&gcpm->chg_psy_lock);
	ret = gvotable_cast_int_vote(el, MDIS_REASON_SETUP, lvl, lvl > 0);
	mutex_unlock(&gcpm->chg_psy_lock);
	if (ret)
		return ret;

	return count;
}

static DEVICE_ATTR_RW(mdis_vote);

static ssize_t
mdis_out_table_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct thermal_cooling_device *tdev = to_cooling_device(dev);
	struct mdis_thermal_device *mdev = tdev->devdata;
	struct gcpm_drv *gcpm = mdev->gcpm;
	const int entries = mdev->thermal_levels * gcpm->mdis_in_count;
	ssize_t count = 0;
	int i, j;

	for (i = 0; i < gcpm->mdis_out_count; i++) {

		count += sysfs_emit_at(buf, count, "%d:", i);

		for (j = 0; j < entries; j++) {
			const int limit = gcpm->mdis_out_limits[i][j];

			count += sysfs_emit_at(buf, count, "%u ", limit);
		}

		count += sysfs_emit_at(buf, count, "\n");
	}

	return count;
}

static DEVICE_ATTR_RO(mdis_out_table);

static const struct thermal_cooling_device_ops chg_mdis_tcd_ops = {
	.get_max_state = gcpm_get_max_charge_cntl_limit,
	.get_cur_state = gcpm_get_cur_charge_cntl_limit,
	.set_cur_state = gcpm_set_mdis_charge_cntl_limit,
};

#ifdef CONFIG_DEBUG_FS

static ssize_t mdis_tm_store(struct file *filp, const char __user *user_buf,
			     size_t count, loff_t *ppos)
{
	struct gcpm_drv *gcpm = filp->private_data;
	const int thermal_levels = gcpm->thermal_device.thermal_levels;
	const int mem_size = count + 1;
	char *str, *tmp, *saved_ptr;
	unsigned long long value;
	int ret, i;

	tmp = kzalloc(mem_size, GFP_KERNEL);
	if (!tmp)
		return -ENOMEM;

	ret = simple_write_to_buffer(tmp, mem_size, ppos, user_buf, count);
	if (!ret)
		goto error_done;

	for (saved_ptr = tmp, i = 0; i < thermal_levels; i++) {
		str = strsep(&saved_ptr, " ");
		if (!str)
			goto error_done;

		ret = kstrtoull(str, 10, &value);
		if (ret < 0)
			goto error_done;

		gcpm->thermal_device.thermal_mitigation[i] = value * 1000;
	}

error_done:
	kfree(tmp);
	return count;
}

DEBUG_ATTRIBUTE_WO(mdis_tm);

static ssize_t mdis_out_store(struct file *filp, const char __user *user_buf,
			     size_t count, loff_t *ppos)
{
	struct gcpm_drv *gcpm = filp->private_data;
	const int levels = gcpm->thermal_device.thermal_levels;
	const int mem_size = count + 1;
	unsigned long long value, index;
	char *str, *tmp, *saved_ptr;
	int ret, i;

	tmp = kzalloc(mem_size, GFP_KERNEL);
	if (!tmp)
		return -ENOMEM;

	ret = simple_write_to_buffer(tmp, mem_size, ppos, user_buf, count);
	if (!ret)
		goto error_done;

	for (saved_ptr = tmp; true; ) {

		str = strsep(&saved_ptr, ":");
		if (!str)
			goto error_done;

		ret = kstrtoull(str, 10, &index);
		if (ret < 0)
			goto error_done;

		if (index < 0 || index >= levels)
			break;

		for (i = 0; i < levels * gcpm->mdis_in_count; i++) {
			str = strsep(&saved_ptr, " ");
			if (!str)
				goto error_done;

			ret = kstrtoull(str, 10, &value);
			if (ret < 0)
				goto error_done;

			gcpm->mdis_out_limits[index][i] = value;
		}
	}

error_done:
	kfree(tmp);
	return count;
}

DEBUG_ATTRIBUTE_WO(mdis_out);

static void chg_mdis_tdev_free(struct mdis_thermal_device *tdev,
			       struct gcpm_drv *gcpm)
{
	devm_kfree(gcpm->device, tdev->thermal_mitigation);
	tdev->thermal_mitigation = NULL;
}

static int mdis_tdev_register(const char *of_name, const char *tcd_name,
			      struct mdis_thermal_device *ctdev,
			      const struct thermal_cooling_device_ops *ops)
{
	struct device_node *cooling_node = NULL;
	int ret;

	cooling_node = of_find_node_by_name(NULL, of_name);
	if (!cooling_node) {
		pr_err("No %s OF node for cooling device\n", of_name);
		return -EINVAL;
	}

	ctdev->tcd = thermal_of_cooling_device_register(cooling_node,
							tcd_name,
							ctdev,
							ops);
	if (IS_ERR_OR_NULL(ctdev->tcd)) {
		const long err = PTR_ERR(ctdev->tcd);

		pr_err("error registering %s cooling device (%ld)\n", tcd_name, err);
		return err;
	}

	ret = device_create_file(&ctdev->tcd->device, &dev_attr_state2power_table);
	if (ret)
		dev_err(ctdev->gcpm->device, "cound not create state table *(%d)\n", ret);

	ret = device_create_file(&ctdev->tcd->device, &dev_attr_mdis_out_table);
	if (ret)
		dev_err(ctdev->gcpm->device, "cound not create out table *(%d)\n", ret);

	return 0;
}

static int mdis_size_show(void *data, u64 *val)
{
	struct gcpm_drv *gcpm = data;

	*val = gcpm->thermal_device.thermal_levels;
	return 0;
}

static int mdis_size_store(void *data, u64 val)
{
	struct gcpm_drv *gcpm = data;
	struct mdis_thermal_device *tdev = &gcpm->thermal_device;
	const int newsize = val;
	const int newsize_bytes = newsize * sizeof(u32);
	u32 *limits;
	int bytes, index, i;
	int ret;

	mutex_lock(&gcpm->chg_psy_lock);

	bytes = (newsize <= tdev->thermal_levels ? newsize : tdev->thermal_levels) * sizeof(u32);

	for (index = 0; index < gcpm->mdis_out_count; index++) {
		limits = devm_kzalloc(gcpm->device, newsize_bytes * gcpm->mdis_in_count, GFP_KERNEL);
		if (!limits) {
			ret = -ENOMEM;
			tdev->thermal_levels = 0;
			goto exit;
		}
		for (i = 0; i < gcpm->mdis_in_count; i++)
			memcpy(limits + (i * newsize), gcpm->mdis_out_limits[index] + (i * tdev->thermal_levels), bytes);
		devm_kfree(gcpm->device, gcpm->mdis_out_limits[index]);
		gcpm->mdis_out_limits[index] = limits;
	}

	limits = devm_kzalloc(gcpm->device, newsize_bytes, GFP_KERNEL);
	if (!limits) {
		ret = -ENOMEM;
		goto exit;
	}
	memcpy(limits, tdev->thermal_mitigation, bytes);

	devm_kfree(gcpm->device, tdev->thermal_mitigation);
	tdev->thermal_mitigation = limits;
	tdev->thermal_levels = newsize;
	ret = 0;

	/* Need to re-register cooling device because size is stored in the thermal framework */
	thermal_cooling_device_unregister(tdev->tcd);

	ret = mdis_tdev_register(MDIS_OF_CDEV_NAME, MDIS_CDEV_NAME,
				 tdev, &chg_mdis_tcd_ops);
	if (ret) {
		dev_err(gcpm->device,
			"Couldn't register %s rc=%d\n", MDIS_OF_CDEV_NAME, ret);

		/* Free the limits too! */
		chg_mdis_tdev_free(tdev, gcpm);
		ret = -EINVAL;
		goto exit;
	}

exit:
	mutex_unlock(&gcpm->chg_psy_lock);
	return ret;
}

DEFINE_SIMPLE_ATTRIBUTE(mdis_size_fops, mdis_size_show,
			mdis_size_store, "%lld\n");

static int wlc_cc_lim_show(void *data, u64 *val)
{
	struct gcpm_drv *gcpm = data;

	*val = gcpm->dc_limit_cc_min_wlc;
	return 0;
}

static int wlc_cc_lim_store(void *data, u64 val)
{
	struct gcpm_drv *gcpm = data;

	gcpm->dc_limit_cc_min_wlc = val;
	return 0;

}

DEFINE_SIMPLE_ATTRIBUTE(wlc_cc_lim_fops, wlc_cc_lim_show,
			wlc_cc_lim_store, "%lld\n");

static int dc_cc_lim_show(void *data, u64 *val)
{
	struct gcpm_drv *gcpm = data;

	*val = gcpm->dc_limit_cc_min;
	return 0;
}

static int dc_cc_lim_store(void *data, u64 val)
{
	struct gcpm_drv *gcpm = data;

	gcpm->dc_limit_cc_min = val;
	return 0;

}

DEFINE_SIMPLE_ATTRIBUTE(dc_cc_lim_fops, dc_cc_lim_show,
			dc_cc_lim_store, "%lld\n");


#endif // CONFIG_DEBUG_FS

/* ------------------------------------------------------------------------- */

static int mdis_out_init_sel_online(u32 *out_sel, int len, const struct gcpm_drv *gcpm)
{
	static const char *name = "google,mdis-out-sel-online";
	struct device_node *node = gcpm->device->of_node;
	int ret, count, byte_len;

	if (!of_find_property(node, name, &byte_len))
		return -ENOENT;

	count = byte_len / sizeof(u32);
	if (count != len)
		return -ERANGE;

	ret = of_property_read_u32_array(node, name, out_sel, count);
	if (ret < 0)
		return -EINVAL;

	return count;
}

/* return the array and the len. Pass len = 0 to avoid check on length */
static u32* gcpm_init_limits(const char *name, int *len, struct device *dev)
{
	struct device_node *node = dev->of_node;
	int ret, byte_len;
	u32 *limits;

	if (!of_find_property(node, name, &byte_len))
		return ERR_PTR(-ENOENT);

	if (*len && (byte_len / sizeof(u32)) > *len)
		return ERR_PTR(-EINVAL);

	limits = devm_kzalloc(dev, byte_len, GFP_KERNEL);
	if (!limits)
		return ERR_PTR(-ENOMEM);

	ret = of_property_read_u32_array(node, name, limits, byte_len / sizeof(u32));
	if (ret < 0) {
		devm_kfree(dev, limits);
		return ERR_PTR(-ERANGE);
	}

	*len = byte_len / sizeof(u32);
	return limits;
}

/* ls /dev/thermal/cdev-by-name/ */
static int gcpm_tdev_init(struct mdis_thermal_device *tdev, const char *name,
			  struct gcpm_drv *gcpm)
{
	int levels = 0;
	u32 *limits;

	mutex_init(&tdev->tdev_lock);

	limits = gcpm_init_limits(name, &levels, gcpm->device);
	if (IS_ERR_OR_NULL(limits)) {
		dev_err(gcpm->device, "Cannot create thermal device %s (%d)\n",
			name, (int)PTR_ERR(limits));
		return PTR_ERR(limits);
	}

	tdev->thermal_levels = levels;
	tdev->thermal_mitigation = limits;
	tdev->gcpm = gcpm;
	return 0;
}

/*
 * pick the adapter with the highest current under the budget
 * needs mutex_lock(&gcpm->chg_psy_lock);
 */
static int gcpm_mdis_callback(struct gvotable_election *el, const char *reason,
			      void *value)
{
	struct gcpm_drv *gcpm = gvotable_get_data(el);
	struct mdis_thermal_device *tdev = &gcpm->thermal_device;
	const int lvl = (long)value;
	bool trigger_select;

	gcpm_update_mdis_charge_cntl_limit(tdev, lvl);

	trigger_select = lvl != 0 && gcpm->cp_fcc_hold;

	pr_debug("MSC_MDIS callback cur_lvl=%d lvl=%d hold=%d cp_fcc_hold_limit=%d\n",
		 tdev->current_level, lvl, gcpm->cp_fcc_hold,
		 gcpm->cp_fcc_hold_limit);

	/*
	 * the limit is cleared when charging current is greater that the limit.
	 * NOTE: Clearing the hold allows re-enabling PPS again.
	 */
	if (gcpm->cp_fcc_hold_limit == -1)
		gcpm->cp_fcc_hold = false;

	/*
	 * gcpm->cp_fcc_hold is set when charging switched to main from DC
	 * due to the charging current falling under cc_mi      n limit: clear the
	 * hold and give PPS a change if the limit is cleared.
	 * NOTE: need to
	 */
	if (trigger_select) {
		int ret;

		ret = gcpm_chg_select_logic(gcpm);
		if (ret == -EAGAIN) {
			const int interval = 5; /* seconds */

			/* let the setting go through but */
			mod_delayed_work(system_wq, &gcpm->select_work,
					 msecs_to_jiffies(interval * 1000));
		}
	}

	gcpm_mdis_update_fan(gcpm);

	if (!gcpm->csi_status_votable) {
		gcpm->csi_status_votable = gvotable_election_get_handle(VOTABLE_CSI_STATUS);
		if (!gcpm->csi_status_votable)
			return 0;
	}

	/* this is a problem only when speed is affected */
	gvotable_cast_long_vote(gcpm->csi_status_votable, "CSI_STATUS_THERM_MDIS",
				CSI_STATUS_System_Thermals,
				tdev->current_level != 0);

	/* will trigger a power supply change now */
	power_supply_changed(gcpm->psy);
	return 0;
}

/*
 * Callback for GCPM_FCC votable which routes the CP limit to the DC charger.
 * The votable combines the CC_MAX limit from google_charger and the MDIS
 * limit from the dissipation based cooling zone.
 * NOTE: it might not benecessary if/when we route the MDIS vote to MSC_FCC
 * directly.
 * caller needs to hold	mutex_lock(&gcpm->chg_psy_lock);
 */
static int gcpm_fcc_callback(struct gvotable_election *el, const char *reason,
			     void *value)
{
	struct gcpm_drv *gcpm = gvotable_get_data(el);
	const int limit = GVOTABLE_PTR_TO_INT(value);
	struct power_supply *cp_psy;
	int cp_min, ret;

	/*
	 * the current limit is changed, validate it against the min
	 * NOTE: this is also used to trigger select_work when restarting
	 * charging when coming off thermal mitigation.
	 */
	cp_min = gcpm_chg_select_check_cp_limit(gcpm);
	if (cp_min != -1 && limit <= cp_min) {
		pr_debug("MSC_GCPM_FCC: limit=%d reason=%s cpmin=%d trigger select\n",
			 limit, reason, cp_min);
		mod_delayed_work(system_wq, &gcpm->select_work, 0);
		return 0;
	}

	/* route the vote to the CP when active */
	cp_psy = gcpm_chg_get_active_cp(gcpm);
	if (!cp_psy) {
		pr_debug("MSC_GCPM_FCC: not active limit=%d\n", limit);
		return 0;
	}

	ret = GPSY_SET_PROP(cp_psy, POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
			    limit);

	if (ret < 0)
		pr_err("MSC_GCPM_FCC: cannot apply cp_limit to cc_max=%d (%d)\n",
		       limit, ret);

	if (ret == -EAGAIN) {
		gcpm->fcc_retries = GCPM_FCC_RETRIES;
		gcpm->fcc_retry_limit = limit;
		mod_delayed_work(system_wq, &gcpm->fcc_retry_work, GCPM_FCC_RETRY_INTERVAL);
	}

	pr_debug("MSC_GCPM_FCC: applied new cp_limit=%d cp_min=%d ret=%d\n",
		 limit, cp_min, ret);

	return 0;
}

#define INIT_DELAY_MS 100
#define INIT_RETRY_DELAY_MS 1000
#define GCPM_TCPM_PSY_MAX 2

/* Dissipation Based Thermal Management */
static int gcpm_init_mdis(struct gcpm_drv *gcpm)
{
	struct mdis_thermal_device *tdev = &gcpm->thermal_device;
	int i, count, ret;

	ret = gcpm_tdev_init(tdev, "google,mdis-thermal-mitigation", gcpm);
	if (ret < 0 || !tdev->thermal_levels) {
		dev_err(gcpm->device, "No device (%d)\n", ret);
		return -ENODEV;
	}

	/*
	 * TODO: remove ->chg_psy_avail[] and ->chg_psy_count and rewrite to
	 * use ->mdis_out[].
	 * NOTE: gcpm_init_work() needs to read into ->mdis_out[].
	 */
	gcpm->mdis_out[0] = gcpm->chg_psy_avail[0];
	gcpm->mdis_out[1] = gcpm->chg_psy_avail[1];
	gcpm->mdis_out_count = gcpm->chg_psy_count;

	/* one for each out, call with #mdis out */
	count = mdis_out_init_sel_online(gcpm->mdis_out_sel, gcpm->chg_psy_count, gcpm);
	if (count < 0) {
		dev_err(gcpm->device, "mdis sel online (%d)\n", ret);
		return -ERANGE;
	}

	/* TODO: rewrite to parse handles in the device tree */
	gcpm->mdis_in[0] = gcpm->tcpm_psy;
	gcpm->mdis_in[1] = gcpm->wlc_dc_psy;
	gcpm->mdis_in_count = 2;

	/*
	 * max charging current for the each thermal level charger and
	 * mdis_out_sel mode.
	 */
	for (i = 0; i < gcpm->mdis_out_count; i++) {
		int len = tdev->thermal_levels * gcpm->mdis_in_count;
		char of_name[36];
		u32 *limits;

		scnprintf(of_name, sizeof(of_name), "google,mdis-out%d-limits", i);

		limits = gcpm_init_limits(of_name, &len, gcpm->device);
		if (IS_ERR_OR_NULL(limits))
			return PTR_ERR(limits);

		gcpm->mdis_out_limits[i] = limits;
	}

	ret = of_property_read_u32(gcpm->device->of_node, "google,mdis-fan-alarm-level",
				   &tdev->therm_fan_alarm_level);
	if (ret < 0)
		tdev->therm_fan_alarm_level = FAN_MDIS_ALARM_DEFAULT;

	/* mdis thermal engine uses this callback */
	gcpm->mdis_votable =
		gvotable_create_int_election(NULL, gvotable_comparator_int_max,
					     gcpm_mdis_callback, gcpm);
	if (IS_ERR_OR_NULL(gcpm->mdis_votable)) {
		ret = PTR_ERR(gcpm->mdis_votable);
		dev_err(gcpm->device, "no mdis votable (%d)\n", ret);
		return ret;
	}

	gvotable_set_default(gcpm->mdis_votable, (void *)0);
	gvotable_set_vote2str(gcpm->mdis_votable, gvotable_v2s_int);
	gvotable_election_set_name(gcpm->mdis_votable, VOTABLE_MDIS);

	/* race with above */
	ret = mdis_tdev_register(MDIS_OF_CDEV_NAME, MDIS_CDEV_NAME,
				 tdev, &chg_mdis_tcd_ops);
	if (ret) {
		dev_err(gcpm->device,
			"Couldn't register %s rc=%d\n", MDIS_OF_CDEV_NAME, ret);

		// Free the limits too!
		chg_mdis_tdev_free(tdev, gcpm);
		return -EINVAL;
	}

	if (!gcpm->debug_entry)
		return 0;

	debugfs_create_file("state2power_table", 0644,  gcpm->debug_entry,
			    gcpm, &mdis_tm_fops);
	debugfs_create_file("mdis_out_table", 0644,  gcpm->debug_entry,
			    gcpm, &mdis_out_fops);
	debugfs_create_file("mdis_size", 0644, gcpm->debug_entry, gcpm, &mdis_size_fops);
	debugfs_create_file("wlc_cc_lim", 0644, gcpm->debug_entry, gcpm, &wlc_cc_lim_fops);
	debugfs_create_file("dc_cc_lim", 0644, gcpm->debug_entry, gcpm, &dc_cc_lim_fops);

	return 0;
}

static void gcpm_cop_warn_work(struct work_struct *work)
{
	struct gcpm_drv *gcpm = container_of(work, struct gcpm_drv,
					     cop_warn_work.work);
	const int offset = gcpm->cop_current_offset;

	mutex_lock(&gcpm->chg_psy_lock);

	/*
	 * COP warn irq triggering calls this after X seconds wait
	 * If the number of times that COP has triggered is above the threshold
	 * gcpm->cop_warn_trigger, we throttle the cc_max by COP_WARN_DEFAULT_OFFSET_MA.
	 *
	 * Throttling continues until COP triggers less than gcpm->cop_warn_trigger in which case we
	 * remove the throttling altogether.
	 *
	 * If the throttle value has changed, we trigger a charge loop to set the cc_max
	 */

	if (gcpm->cop_warn_count < gcpm->cop_warn_trigger) {
		gcpm->cop_current_offset = 0;
	} else {
		gcpm->cop_current_offset -= COP_WARN_DEFAULT_OFFSET_MA;
		dev_dbg(gcpm->device, "%s: scheduling cop_warn_work\n", __func__);
		schedule_delayed_work(&gcpm->cop_warn_work,
				      msecs_to_jiffies(COP_WARN_BACKOFF_MS));
	}

	dev_dbg(gcpm->device, "COP warn count:%d offset:%d\n", gcpm->cop_warn_count,
		gcpm->cop_current_offset);

	gcpm->cop_warn_count = 0;
	if (offset != gcpm->cop_current_offset)
		power_supply_changed(gcpm->psy);
	mutex_unlock(&gcpm->chg_psy_lock);
}

static irqreturn_t google_cpm_cop_warn_irq_handler(int irq, void *ptr)
{
	struct gcpm_drv *gcpm = ptr;

	if (gcpm_resume_check(gcpm))
		return IRQ_NONE;

	dev_warn(gcpm->device, "COP Warn triggered cc_max:%u\n", gcpm->cc_max);
	/* Schedule work on the first instance of COP Warn */
	if (gcpm->cop_warn_count == 0) {
		dev_dbg(gcpm->device, "%s: scheduling cop_warn_work\n", __func__);
		schedule_delayed_work(&gcpm->cop_warn_work,
				      msecs_to_jiffies(COP_WARN_BACKOFF_MS));
	}
	gcpm->cop_warn_count++;

	return IRQ_HANDLED;
}

/* this can run */
static void gcpm_init_work(struct work_struct *work)
{
	struct gcpm_drv *gcpm = container_of(work, struct gcpm_drv,
					     init_work.work);
	int i, found = 0, ret = 0;
	bool dc_not_done;

	/* might run along set_property() */
	mutex_lock(&gcpm->chg_psy_lock);

	/*
	 * could call pps_init() in probe() and use lazy init for ->tcpm_psy
	 * when the device an APDO in the sink capabilities.
	 */
	if (gcpm->tcpm_phandle && !gcpm->tcpm_psy) {
		struct power_supply *tcpm_psy;

		tcpm_psy = pps_get_tcpm_psy(gcpm->device->of_node,
					    GCPM_TCPM_PSY_MAX);
		if (!IS_ERR_OR_NULL(tcpm_psy)) {
			const char *name = tcpm_psy->desc->name;

			gcpm->tcpm_psy_name = name;
			gcpm->tcpm_psy = tcpm_psy;

			/* PPS charging: needs an APDO */
			ret = pps_init(&gcpm->tcpm_pps_data, gcpm->device,
				       gcpm->tcpm_psy, "wired-pps");
			if (ret == 0 && gcpm->debug_entry)
				pps_init_fs(&gcpm->tcpm_pps_data, gcpm->debug_entry);
			if (ret < 0) {
				pr_err("PPS init failure for %s (%d)\n",
				       name, ret);
			} else {
				gcpm->tcpm_pps_data.port_data =
					power_supply_get_drvdata(tcpm_psy);
				pps_init_state(&gcpm->tcpm_pps_data);
				pps_set_logbuffer(&gcpm->tcpm_pps_data, gcpm->log);
				pps_log(&gcpm->tcpm_pps_data, "TCPM_PPS for %s", gcpm->tcpm_psy_name);
			}

		} else if (!tcpm_psy || !gcpm->log_psy_ratelimit) {
			/* abort on an error */
			pr_warn("PPS not available for tcpm\n");
			gcpm->tcpm_phandle = 0;
		} else {
			pr_warn("tcpm power supply not found, retrying... ret:%d\n",
				ret);
			gcpm->log_psy_ratelimit--;
		}

	}

	/* TODO: lookup by phandle as the dude above */
	if (gcpm->wlc_dc_name && !gcpm->wlc_dc_psy) {
		struct power_supply *wlc_dc_psy;

		wlc_dc_psy = power_supply_get_by_name(gcpm->wlc_dc_name);
		if (wlc_dc_psy) {
			const char *name = gcpm->wlc_dc_name;

			gcpm->wlc_dc_psy = wlc_dc_psy;

			/* PPS charging: needs an APDO */
			ret = pps_init(&gcpm->wlc_pps_data, gcpm->device,
					gcpm->wlc_dc_psy, "wireless-pps");
			if (ret == 0 && gcpm->debug_entry)
				pps_init_fs(&gcpm->wlc_pps_data, gcpm->debug_entry);
			if (ret < 0) {
				pr_err("PPS init failure for %s (%d)\n",
				       name, ret);
			} else {
				gcpm->wlc_pps_data.port_data = NULL;
				pps_init_state(&gcpm->wlc_pps_data);
				pps_set_logbuffer(&gcpm->wlc_pps_data, gcpm->log);
				pps_log(&gcpm->wlc_pps_data, "WLC_PPS for %s", gcpm->wlc_dc_name);
			}

		} else if (!gcpm->log_psy_ratelimit) {
			/* give up if wlc_dc_psy return an error */
			pr_warn("PPS not available for %s\n", gcpm->wlc_dc_name);
			gcpm->wlc_dc_name = NULL;
		} else {
			pr_warn("%s power supply not found, retrying... ret:%d\n",
				gcpm->wlc_dc_name, ret);
			gcpm->log_psy_ratelimit--;
		}
	}

	/* default is index 0 */
	for (i = 0; i < gcpm->chg_psy_count; i++) {
		if (!gcpm->chg_psy_avail[i]) {
			const char *name = gcpm->chg_psy_names[i];

			gcpm->chg_psy_avail[i] = power_supply_get_by_name(name);
			if (gcpm->chg_psy_avail[i])
				pr_info("init_work found %d:%s\n", i, name);
		}

		found += !!gcpm->chg_psy_avail[i];
	}

	/* sort of done when we have the primary, make it online */
	if (gcpm->chg_psy_avail[0] && !gcpm->init_complete) {
		struct power_supply *def_psy = gcpm->chg_psy_avail[0];

		gcpm->chg_nb.notifier_call = gcpm_psy_changed;
		ret = power_supply_reg_notifier(&gcpm->chg_nb);
		if (ret < 0)
			pr_err("%s: no ps notifier, ret=%d\n", __func__, ret);

		ret = gcpm_enable_default(gcpm);
		if (ret < 0)
			pr_err("%s: default %s not online, ret=%d\n", __func__,
			       gcpm_psy_name(def_psy), ret);

		/* this is the reason why we need a lock here */
		gcpm->resume_complete = true;
		gcpm->init_complete = true;
	}

	dc_not_done = (gcpm->tcpm_phandle && !gcpm->tcpm_psy) ||
		      (gcpm->wlc_dc_name && !gcpm->wlc_dc_psy);

	/* keep looking for late arrivals, TCPM and WLC if set */
	if (found == gcpm->chg_psy_count && !dc_not_done)
		gcpm->chg_psy_retries = 0;
	else if (gcpm->chg_psy_retries)
		gcpm->chg_psy_retries--;

	pr_warn("%s retries=%d dc_not_done=%d tcpm_ok=%d wlc_ok=%d\n",
		__func__, gcpm->chg_psy_retries, dc_not_done,
		(!gcpm->tcpm_phandle || gcpm->tcpm_psy),
		(!gcpm->wlc_dc_name || gcpm->wlc_dc_psy));

	if (gcpm->chg_psy_retries) {
		const unsigned long jif = msecs_to_jiffies(INIT_RETRY_DELAY_MS);

		schedule_delayed_work(&gcpm->init_work, jif);
		mutex_unlock(&gcpm->chg_psy_lock);
		return;
	}

	pr_info("google_cpm init_work done %d/%d pps=%d wlc_dc=%d\n",
		found, gcpm->chg_psy_count,
		!!gcpm->tcpm_psy, !!gcpm->wlc_dc_psy);

	ret = gcpm_init_mdis(gcpm);
	if (ret < 0)
		pr_info("google_cpm: no mdis engine (%d)\n", ret);

	gcpm->dc_init_complete = true;
	mutex_unlock(&gcpm->chg_psy_lock);

	/* might run along set_property() */
	mod_delayed_work(system_wq, &gcpm->select_work, 0);
}

static void gcpm_fcc_retry_work(struct work_struct *work)
{
	struct gcpm_drv *gcpm = container_of(work, struct gcpm_drv,
					     fcc_retry_work.work);
	int ret = 0;
	struct power_supply *cp_psy;

	mutex_lock(&gcpm->chg_psy_lock);

	if (gcpm->fcc_retries--) {
		cp_psy = gcpm_chg_get_active_cp(gcpm);
		if (!cp_psy) {
			pr_debug("MSC_GCPM_FCC_RETRY: not active limit=%d\n", gcpm->fcc_retry_limit);
			goto exit;
		}

		ret = GPSY_SET_PROP(cp_psy, POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
			gcpm->fcc_retry_limit);

		if (ret < 0)
			pr_err("MSC_GCPM_FCC_RETRY: cannot apply cp_limit to cc_max=%d (%d)\n",
			gcpm->fcc_retry_limit, ret);

		if (ret == -EAGAIN)
			schedule_delayed_work(&gcpm->fcc_retry_work, msecs_to_jiffies(GCPM_FCC_RETRY_INTERVAL));
	}

exit:
	mutex_unlock(&gcpm->chg_psy_lock);

}

/* ------------------------------------------------------------------------ */

static int gcpm_debug_get_active(void *data, u64 *val)
{
	struct gcpm_drv *gcpm = data;

	mutex_lock(&gcpm->chg_psy_lock);
	*val = gcpm->dc_index;
	mutex_unlock(&gcpm->chg_psy_lock);
	return 0;
}

static int gcpm_debug_set_active(void *data, u64 val)
{
	struct gcpm_drv *gcpm = data;
	int intval = (int)val;

	if (gcpm->force_active != -1 && val == gcpm->force_active)
		intval = -1;

	pr_info("%s: val=%llu val=%lld intval=%d\n", __func__, val, val, intval);

	if (intval != -1 && (intval < 0 || intval >= gcpm->chg_psy_count))
		return -ERANGE;
	if (intval != -1 && !gcpm_chg_get_charger(gcpm, intval))
		return -EINVAL;

	mutex_lock(&gcpm->chg_psy_lock);
	gcpm->force_active = intval;
	mod_delayed_work(system_wq, &gcpm->select_work, 0);
	mutex_unlock(&gcpm->chg_psy_lock);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(gcpm_debug_active_fops, gcpm_debug_get_active,
			gcpm_debug_set_active, "%lld\n");

static int gcpm_debug_dc_limit_demand_show(void *data, u64 *val)
{
	struct gcpm_drv *gcpm = data;

	*val = gcpm->dc_limit_demand;
	return 0;
}

static int gcpm_debug_dc_limit_demand_set(void *data, u64 val)
{
	struct gcpm_drv *gcpm = data;
	const int intval = val;

	mutex_lock(&gcpm->chg_psy_lock);
	if (gcpm->dc_limit_demand != intval) {
		gcpm->dc_limit_demand = intval;
		gcpm->new_dc_limit = true;
	}

	mutex_unlock(&gcpm->chg_psy_lock);
	return 0;
}


DEFINE_SIMPLE_ATTRIBUTE(gcpm_debug_dc_limit_demand_fops,
			gcpm_debug_dc_limit_demand_show,
                        gcpm_debug_dc_limit_demand_set,
			"%llu\n");


static int gcpm_debug_pps_stage_get(void *data, u64 *val)
{
	struct gcpm_drv *gcpm = data;
	struct pd_pps_data *pps_data;

	mutex_lock(&gcpm->chg_psy_lock);
	pps_data = gcpm_pps_data(gcpm);
	if (pps_data)
		*val = pps_data->stage;
	mutex_unlock(&gcpm->chg_psy_lock);
	return 0;
}

static int gcpm_debug_pps_stage_set(void *data, u64 val)
{
	struct gcpm_drv *gcpm = data;
	const int intval = (int)val;
	struct pd_pps_data *pps_data;

	if (intval < PPS_DISABLED || intval > PPS_ACTIVE)
		return -EINVAL;

	mutex_lock(&gcpm->chg_psy_lock);
	pps_data = gcpm_pps_data(gcpm);
	if (pps_data)
		pps_data->stage = intval;
	gcpm->force_pps = !pps_is_disabled(intval);
	mod_delayed_work(system_wq, &gcpm->pps_work, 0);
	mutex_unlock(&gcpm->chg_psy_lock);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(gcpm_debug_pps_stage_fops, gcpm_debug_pps_stage_get,
			gcpm_debug_pps_stage_set, "%llu\n");

static int gcpm_debug_dc_state_get(void *data, u64 *val)
{
	struct gcpm_drv *gcpm = data;

	mutex_lock(&gcpm->chg_psy_lock);
	*val = gcpm->dc_state;
	mutex_unlock(&gcpm->chg_psy_lock);
	return 0;
}

static int gcpm_debug_dc_state_set(void *data, u64 val)
{
	struct gcpm_drv *gcpm = data;
	const int intval = (int)val;

	if (intval < DC_DISABLED || intval > DC_PASSTHROUGH)
		return -EINVAL;

	mutex_lock(&gcpm->chg_psy_lock);
	gcpm->dc_state = intval;
	mod_delayed_work(system_wq, &gcpm->select_work, 0);
	mutex_unlock(&gcpm->chg_psy_lock);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(gcpm_debug_dc_state_fops, gcpm_debug_dc_state_get,
			gcpm_debug_dc_state_set, "%llu\n");


static int gcpm_debug_taper_ctl_get(void *data, u64 *val)
{
	struct gcpm_drv *gcpm = data;

	mutex_lock(&gcpm->chg_psy_lock);
	*val = gcpm->taper_step;
	mutex_unlock(&gcpm->chg_psy_lock);
	return 0;
}

static int gcpm_debug_taper_ctl_set(void *data, u64 val)
{
	struct gcpm_drv *gcpm = data;
	bool ta_check;

	mutex_lock(&gcpm->chg_psy_lock);

	/* ta_check set when taper control changes value */
	ta_check = gcpm_taper_ctl(gcpm, val);
	if (ta_check)
		mod_delayed_work(system_wq, &gcpm->select_work, 0);
	mutex_unlock(&gcpm->chg_psy_lock);
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(gcpm_debug_taper_ctl_fops, gcpm_debug_taper_ctl_get,
			gcpm_debug_taper_ctl_set, "%llu\n");

static int gcpm_debug_taper_step_fv_margin_get(void *data, u64 *val)
{
	struct gcpm_drv *gcpm = data;

	mutex_lock(&gcpm->chg_psy_lock);
	*val = gcpm->taper_step_fv_margin;
	mutex_unlock(&gcpm->chg_psy_lock);
	return 0;
}

static int gcpm_debug_taper_step_fv_margin_set(void *data, u64 val)
{
	struct gcpm_drv *gcpm = data;
	const int intval = (int)val;

	mutex_lock(&gcpm->chg_psy_lock);
	gcpm->taper_step_fv_margin = intval;
	mutex_unlock(&gcpm->chg_psy_lock);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(gcpm_debug_taper_step_fv_margin_fops, gcpm_debug_taper_step_fv_margin_get,
			gcpm_debug_taper_step_fv_margin_set, "%llu\n");

static int gcpm_debug_taper_step_cc_step_get(void *data, u64 *val)
{
	struct gcpm_drv *gcpm = data;

	mutex_lock(&gcpm->chg_psy_lock);
	*val = gcpm->taper_step_cc_step;
	mutex_unlock(&gcpm->chg_psy_lock);
	return 0;
}

static int gcpm_debug_taper_step_cc_step_set(void *data, u64 val)
{
	struct gcpm_drv *gcpm = data;
	const int intval = (int)val;

	mutex_lock(&gcpm->chg_psy_lock);
	gcpm->taper_step_cc_step = intval;
	mutex_unlock(&gcpm->chg_psy_lock);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(gcpm_debug_taper_step_cc_step_fops, gcpm_debug_taper_step_cc_step_get,
			gcpm_debug_taper_step_cc_step_set, "%llu\n");

static int gcpm_debug_taper_step_count_get(void *data, u64 *val)
{
	struct gcpm_drv *gcpm = data;

	mutex_lock(&gcpm->chg_psy_lock);
	*val = gcpm->taper_step_count;
	mutex_unlock(&gcpm->chg_psy_lock);
	return 0;
}

static int gcpm_debug_taper_step_count_set(void *data, u64 val)
{
	struct gcpm_drv *gcpm = data;
	const int intval = (int)val;

	mutex_lock(&gcpm->chg_psy_lock);
	gcpm->taper_step_count = intval;
	mutex_unlock(&gcpm->chg_psy_lock);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(gcpm_debug_taper_step_count_fops, gcpm_debug_taper_step_count_get,
			gcpm_debug_taper_step_count_set, "%llu\n");

static int gcpm_debug_taper_step_grace_get(void *data, u64 *val)
{
	struct gcpm_drv *gcpm = data;

	mutex_lock(&gcpm->chg_psy_lock);
	*val = gcpm->taper_step_grace;
	mutex_unlock(&gcpm->chg_psy_lock);
	return 0;
}

static int gcpm_debug_taper_step_grace_set(void *data, u64 val)
{
	struct gcpm_drv *gcpm = data;
	const int intval = (int)val;

	mutex_lock(&gcpm->chg_psy_lock);
	gcpm->taper_step_grace = intval;
	mutex_unlock(&gcpm->chg_psy_lock);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(gcpm_debug_taper_step_grace_fops, gcpm_debug_taper_step_grace_get,
			gcpm_debug_taper_step_grace_set, "%llu\n");

static int gcpm_debug_taper_step_voltage_get(void *data, u64 *val)
{
	struct gcpm_drv *gcpm = data;

	mutex_lock(&gcpm->chg_psy_lock);
	*val = gcpm->taper_step_voltage;
	mutex_unlock(&gcpm->chg_psy_lock);
	return 0;
}

static int gcpm_debug_taper_step_voltage_set(void *data, u64 val)
{
	struct gcpm_drv *gcpm = data;
	const int intval = (int)val;

	mutex_lock(&gcpm->chg_psy_lock);
	gcpm->taper_step_voltage = intval;
	mutex_unlock(&gcpm->chg_psy_lock);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(gcpm_debug_taper_step_voltage_fops, gcpm_debug_taper_step_voltage_get,
			gcpm_debug_taper_step_voltage_set, "%llu\n");

static int gcpm_debug_taper_step_current_get(void *data, u64 *val)
{
	struct gcpm_drv *gcpm = data;

	mutex_lock(&gcpm->chg_psy_lock);
	*val = gcpm->taper_step_current;
	mutex_unlock(&gcpm->chg_psy_lock);
	return 0;
}

static int gcpm_debug_taper_step_current_set(void *data, u64 val)
{
	struct gcpm_drv *gcpm = data;
	const int intval = (int)val;

	mutex_lock(&gcpm->chg_psy_lock);
	gcpm->taper_step_current = intval;
	mutex_unlock(&gcpm->chg_psy_lock);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(gcpm_debug_taper_step_current_fops, gcpm_debug_taper_step_current_get,
			gcpm_debug_taper_step_current_set, "%llu\n");

static int gcpm_debug_taper_step_interval_get(void *data, u64 *val)
{
	struct gcpm_drv *gcpm = data;

	mutex_lock(&gcpm->chg_psy_lock);
	*val = gcpm->taper_step_interval;
	mutex_unlock(&gcpm->chg_psy_lock);
	return 0;
}

static int gcpm_debug_taper_step_interval_set(void *data, u64 val)
{
	struct gcpm_drv *gcpm = data;
	const int intval = (int)val;

	mutex_lock(&gcpm->chg_psy_lock);
	gcpm->taper_step_interval = intval;
	mutex_unlock(&gcpm->chg_psy_lock);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(gcpm_debug_taper_step_interval_fops, gcpm_debug_taper_step_interval_get,
			gcpm_debug_taper_step_interval_set, "%llu\n");

static struct dentry *gcpm_init_fs(struct gcpm_drv *gcpm)
{
	struct dentry *de;

	de = debugfs_create_dir("google_cpm", 0);
	if (IS_ERR_OR_NULL(de))
		return NULL;

	debugfs_create_file("dc_state", 0644, de, gcpm, &gcpm_debug_dc_state_fops);
	debugfs_create_file("active", 0644, de, gcpm, &gcpm_debug_active_fops);
	debugfs_create_file("dc_limit_demand", 0644, de, gcpm,
			    &gcpm_debug_dc_limit_demand_fops);

	debugfs_create_u32("dc_limit_soc_high", 0644, de, &gcpm->dc_limit_soc_high);

	debugfs_create_file("pps_stage", 0644, de, gcpm, &gcpm_debug_pps_stage_fops);

	/* smooth exit from DC */
	debugfs_create_file("taper_ctl", 0644, de, gcpm, &gcpm_debug_taper_ctl_fops);
	debugfs_create_file("taper_step_fv_margin", 0644, de, gcpm, &gcpm_debug_taper_step_fv_margin_fops);
	debugfs_create_file("taper_step_cc_step", 0644, de, gcpm, &gcpm_debug_taper_step_cc_step_fops);
	debugfs_create_file("taper_step_count", 0644, de, gcpm, &gcpm_debug_taper_step_count_fops);
	debugfs_create_file("taper_step_grace", 0644, de, gcpm, &gcpm_debug_taper_step_grace_fops);
	debugfs_create_file("taper_step_voltage", 0644, de, gcpm, &gcpm_debug_taper_step_voltage_fops);
	debugfs_create_file("taper_step_current", 0644, de, gcpm, &gcpm_debug_taper_step_current_fops);
	debugfs_create_file("taper_step_interval", 0644, de, gcpm, &gcpm_debug_taper_step_interval_fops);

	if (gcpm->cop_supported)
		debugfs_create_u32("cop_warn_trigger", 0644, de, &gcpm->cop_warn_trigger);

	return de;
}

/* ------------------------------------------------------------------------ */

static int gcpm_probe_psy_names(struct gcpm_drv *gcpm)
{
	struct device *dev = gcpm->device;
	int i, count, ret;

	if (!gcpm->device)
		return -EINVAL;

	count = of_property_count_strings(dev->of_node,
					  "google,chg-power-supplies");
	if (count <= 0 || count > GCPM_MAX_CHARGERS)
		return -ERANGE;

	ret = of_property_read_string_array(dev->of_node,
					    "google,chg-power-supplies",
					    (const char**)&gcpm->chg_psy_names,
					    count);
	if (ret != count)
		return -ERANGE;

	for (i = 0; i < count; i++)
		dev_info(gcpm->device, "%d:%s\n", i, gcpm->chg_psy_names[i]);

	return count;
}

/* -------------------------------------------------------------------------
 *  Use to abstract the PPS adapter if needed.
 */

static int gcpm_pps_psy_set_property(struct power_supply *psy,
				    enum power_supply_property prop,
				    const union power_supply_propval *val)
{
	struct gcpm_drv *gcpm = power_supply_get_drvdata(psy);
	struct pd_pps_data *pps_data;
	int ret = 0;

	mutex_lock(&gcpm->chg_psy_lock);

	pps_data = gcpm_pps_data(gcpm);
	if (!pps_data || !pps_data->pps_psy) {
		pr_debug("%s: no target prop=%d ret=%d\n", __func__, prop, ret);
		mutex_unlock(&gcpm->chg_psy_lock);
		return -EAGAIN;
	}

	switch (prop) {
	default:
		ret = power_supply_set_property(pps_data->pps_psy, prop, val);
		break;
	}

	mutex_unlock(&gcpm->chg_psy_lock);
	pr_debug("%s: prop=%d val=%d ret=%d\n", __func__,
		 prop, val->intval, ret);

	return ret;
}

static int gcpm_pps_psy_get_property(struct power_supply *psy,
				    enum power_supply_property prop,
				    union power_supply_propval *val)
{
	struct gcpm_drv *gcpm = power_supply_get_drvdata(psy);
	struct pd_pps_data *pps_data;
	int ret = 0;

	mutex_lock(&gcpm->chg_psy_lock);

	pps_data = gcpm_pps_data(gcpm);
	if (pps_data && pps_data->pps_psy) {
		ret = power_supply_get_property(pps_data->pps_psy, prop, val);
		pr_debug("%s: prop=%d val=%d ret=%d\n", __func__,
			 prop, val->intval, ret);
		goto done;
	}

	switch (prop) {
	case POWER_SUPPLY_PROP_USB_TYPE:
		val->intval = POWER_SUPPLY_USB_TYPE_UNKNOWN;
		break;
	default:
		val->intval = 0;
		break;
	}

done:
	mutex_unlock(&gcpm->chg_psy_lock);
	return ret;
}

/* check pps_is_avail(), pps_prog_online() and pps_check_type() */
static enum power_supply_property gcpm_pps_psy_properties[] = {
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MIN,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,	/* 17 */
	POWER_SUPPLY_PROP_ONLINE,	/* 4 */
	POWER_SUPPLY_PROP_PRESENT,	/* 3 */
	POWER_SUPPLY_PROP_TYPE,		/* */
	POWER_SUPPLY_PROP_USB_TYPE,	/* */
	POWER_SUPPLY_PROP_VOLTAGE_NOW,	/* */
};

static int gcpm_pps_psy_is_writeable(struct power_supply *psy,
				   enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
	case POWER_SUPPLY_PROP_ONLINE:
	case POWER_SUPPLY_PROP_CURRENT_NOW:
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		return 1;
	default:
		break;
	}

	return 0;
}

static enum power_supply_usb_type gcpm_pps_usb_types[] = {
	POWER_SUPPLY_USB_TYPE_UNKNOWN,
	POWER_SUPPLY_USB_TYPE_PD_PPS
};

static const struct power_supply_desc gcpm_pps_psy_desc = {
	.name		= "gcpm_pps",
	.type		= POWER_SUPPLY_TYPE_UNKNOWN,
	.get_property	= gcpm_pps_psy_get_property,
	.set_property 	= gcpm_pps_psy_set_property,
	.properties	= gcpm_pps_psy_properties,
	.property_is_writeable = gcpm_pps_psy_is_writeable,
	.num_properties	= ARRAY_SIZE(gcpm_pps_psy_properties),

	/* POWER_SUPPLY_PROP_USB_TYPE requires an array of these */
	.usb_types	= gcpm_pps_usb_types,
	.num_usb_types	= ARRAY_SIZE(gcpm_pps_usb_types),
};

/* ------------------------------------------------------------------------- */

#define LOG_PSY_RATELIMIT_CNT	200

static int google_cpm_probe(struct platform_device *pdev)
{
	struct power_supply_config gcpm_pps_psy_cfg = { 0 };
	struct power_supply_config psy_cfg = { 0 };
	const char *tmp_name = NULL;
	struct gcpm_drv *gcpm;
	int ret;

	gcpm = devm_kzalloc(&pdev->dev, sizeof(*gcpm), GFP_KERNEL);
	if (!gcpm)
		return -ENOMEM;

	gcpm->pdev = pdev;
	gcpm->device = &pdev->dev;
	gcpm->force_active = -1;
	gcpm->dc_ctl = GCPM_DC_CTL_DEFAULT;
	gcpm->log_psy_ratelimit = LOG_PSY_RATELIMIT_CNT;
	gcpm->chg_psy_retries = 10; /* chg_psy_retries *  INIT_RETRY_DELAY_MS */
	gcpm->out_uv = -1;
	gcpm->out_ua = -1;
	gcpm->cp_fcc_hold_limit = -1;

	INIT_DELAYED_WORK(&gcpm->pps_work, gcpm_pps_wlc_dc_work);
	INIT_DELAYED_WORK(&gcpm->select_work, gcpm_chg_select_work);
	INIT_DELAYED_WORK(&gcpm->init_work, gcpm_init_work);
	INIT_DELAYED_WORK(&gcpm->fcc_retry_work, gcpm_fcc_retry_work);
	INIT_DELAYED_WORK(&gcpm->cop_warn_work, gcpm_cop_warn_work);

	mutex_init(&gcpm->chg_psy_lock);

	gcpm->gcpm_ws = wakeup_source_register(NULL, "google-cpm");
	if (!gcpm->gcpm_ws) {
		dev_err(gcpm->device, "Failed to register wakeup source\n");
		return -ENODEV;
	}

	/* this is my name */
	ret = of_property_read_string(pdev->dev.of_node, "google,psy-name",
				      &tmp_name);
	if (ret == 0) {
		gcpm_psy_desc.psy_dsc.name = devm_kstrdup(&pdev->dev, tmp_name,
						  GFP_KERNEL);
		if (!gcpm_psy_desc.psy_dsc.name)
			return -ENOMEM;
	}

	/* subs power supply names */
	gcpm->chg_psy_count = gcpm_probe_psy_names(gcpm);
	if (gcpm->chg_psy_count <= 0)
		return -ENODEV;

	/* DC/PPS needs at least one power supply of this type */
	ret = of_property_read_u32(pdev->dev.of_node,
				   "google,tcpm-power-supply",
				   &gcpm->tcpm_phandle);
	if (ret < 0)
		pr_warn("google,tcpm-power-supply not defined\n");

	ret = of_property_read_string(pdev->dev.of_node,
				      "google,wlc_dc-power-supply",
				      &tmp_name);
	if (ret == 0) {
		gcpm->wlc_dc_name = devm_kstrdup(&pdev->dev, tmp_name,
						     GFP_KERNEL);
		if (!gcpm->wlc_dc_name)
			return -ENOMEM;
	}

	/* GCPM might need a gpio to enable/disable DC/PPS */
	gcpm->dcen_gpio = of_get_named_gpio(pdev->dev.of_node, "google,dc-en", 0);
	if (gcpm->dcen_gpio >= 0) {
		unsigned long init_flags = GPIOF_OUT_INIT_LOW;

		gcpm->dcen_gpio_default = of_property_read_bool(pdev->dev.of_node,
								"google,dc-en-value");
		if (gcpm->dcen_gpio_default)
			init_flags = GPIOF_OUT_INIT_HIGH;

		ret = devm_gpio_request_one(&pdev->dev, gcpm->dcen_gpio,
					    init_flags, "dc_pu_pin");
		pr_info("google,dc-en value =%d ret=%d\n",
			gcpm->dcen_gpio_default, ret);
	}

	/* Triggers to enable dc charging */
	ret = of_property_read_u32(pdev->dev.of_node, "google,dc_limit-demand",
				   &gcpm->dc_limit_demand);
	if (ret < 0)
		gcpm->dc_limit_demand = GCPM_DEFAULT_DC_LIMIT_DEMAND;

	ret = of_property_read_u32(pdev->dev.of_node, "google,dc_limit-cc_min",
				   &gcpm->dc_limit_cc_min);
	if (ret < 0)
		gcpm->dc_limit_cc_min = GCPM_DEFAULT_DC_LIMIT_CC_MIN;

	ret = of_property_read_u32(pdev->dev.of_node, "google,dc_limit-cc_min_wlc",
				   &gcpm->dc_limit_cc_min_wlc);
	if (ret < 0)
		gcpm->dc_limit_cc_min_wlc = GCPM_DEFAULT_DC_LIMIT_CC_MIN_WLC;


	/* voltage lower bound */
	ret = of_property_read_u32(pdev->dev.of_node, "google,dc_limit-vbatt_min",
				   &gcpm->dc_limit_vbatt_min);
	if (ret < 0)
		gcpm->dc_limit_vbatt_min = GCPM_DEFAULT_DC_LIMIT_VBATT_MIN;
	ret = of_property_read_u32(pdev->dev.of_node, "google,dc_limit-vbatt_low",
				   &gcpm->dc_limit_vbatt_low);
	if (ret < 0)
		gcpm->dc_limit_vbatt_low = gcpm->dc_limit_vbatt_min -
					   GCPM_DEFAULT_DC_LIMIT_DELTA_LOW;
	if (gcpm->dc_limit_vbatt_low > gcpm->dc_limit_vbatt_min)
		gcpm->dc_limit_vbatt_low = gcpm->dc_limit_vbatt_min;

	/* WLC voltage lower bound */
	ret = of_property_read_u32(pdev->dev.of_node, "google,wlc_dc_limit-vbatt_min",
				   &gcpm->wlc_dc_limit_vbatt_min);
	if (ret < 0)
		gcpm->wlc_dc_limit_vbatt_min = gcpm->dc_limit_vbatt_min;
	ret = of_property_read_u32(pdev->dev.of_node, "google,wlc_dc_limit-vbatt_low",
				   &gcpm->wlc_dc_limit_vbatt_low);
	if (ret < 0)
		gcpm->wlc_dc_limit_vbatt_low = gcpm->wlc_dc_limit_vbatt_min -
					   GCPM_DEFAULT_DC_LIMIT_DELTA_LOW;
	if (gcpm->wlc_dc_limit_vbatt_low > gcpm->wlc_dc_limit_vbatt_min)
		gcpm->wlc_dc_limit_vbatt_low = gcpm->wlc_dc_limit_vbatt_min;

	/* voltage upper bound */
	ret = of_property_read_u32(pdev->dev.of_node, "google,dc_limit-vbatt_max",
				   &gcpm->dc_limit_vbatt_max);
	if (ret < 0)
		gcpm->dc_limit_vbatt_max = GCPM_DEFAULT_DC_LIMIT_VBATT_MAX;
	ret = of_property_read_u32(pdev->dev.of_node, "google,dc_limit-vbatt_high",
				   &gcpm->dc_limit_vbatt_high);
	if (ret < 0)
		gcpm->dc_limit_vbatt_high = gcpm->dc_limit_vbatt_max -
					    GCPM_DEFAULT_DC_LIMIT_DELTA_HIGH;
	if (gcpm->dc_limit_vbatt_high > gcpm->dc_limit_vbatt_max)
		gcpm->dc_limit_vbatt_high = gcpm->dc_limit_vbatt_max;

	/* state of charge based limits */
	ret = of_property_read_u32(pdev->dev.of_node, "google,dc_limit-soc_high",
				   &gcpm->dc_limit_soc_high);
	if (ret < 0)
		gcpm->dc_limit_soc_high = GCPM_DEFAULT_DC_LIMIT_SOC_HIGH;

	/* taper control */
	gcpm->taper_step = -1;
	ret = of_property_read_u32(pdev->dev.of_node, "google,taper_step-fv-margin",
				   &gcpm->taper_step_fv_margin);
	if (ret < 0)
		gcpm->taper_step_fv_margin = GCPM_TAPER_STEP_FV_MARGIN;
	ret = of_property_read_u32(pdev->dev.of_node, "google,taper_step-cc-step",
				   &gcpm->taper_step_cc_step);
	if (ret < 0)
		gcpm->taper_step_cc_step = GCPM_TAPER_STEP_CC_STEP;
	ret = of_property_read_u32(pdev->dev.of_node, "google,taper_step-interval",
				   &gcpm->taper_step_interval);
	if (ret < 0)
		gcpm->taper_step_interval = GCPM_TAPER_STEP_INTERVAL_S;

	ret = of_property_read_u32(pdev->dev.of_node, "google,taper_step-count",
				   &gcpm->taper_step_count);
	if (ret < 0)
		gcpm->taper_step_count = GCPM_TAPER_STEP_COUNT;
	ret = of_property_read_u32(pdev->dev.of_node, "google,taper_step-grace",
				   &gcpm->taper_step_grace);
	if (ret < 0)
		gcpm->taper_step_grace = GCPM_TAPER_STEP_GRACE;
	ret = of_property_read_u32(pdev->dev.of_node, "google,taper_step-voltage",
				   &gcpm->taper_step_voltage);
	if (ret < 0)
		gcpm->taper_step_voltage = GCPM_TAPER_STEP_VOLTAGE;
	ret = of_property_read_u32(pdev->dev.of_node, "google,taper_step-current",
				   &gcpm->taper_step_current);
	if (ret < 0)
		gcpm->taper_step_current = GCPM_TAPER_STEP_CURRENT;
	ret = of_property_read_u32(pdev->dev.of_node, "google,wlc-dc-fcc-ua",
				   &gcpm->wlc_dc_fcc);
	if (ret < 0)
		gcpm->wlc_dc_fcc = 0;

	gcpm->no_init_wlc_ta_vol = of_property_read_bool(pdev->dev.of_node,
							"google,no-init-wlc-ta-vol");

	gcpm->cop_supported = of_property_read_bool(pdev->dev.of_node, "google,cop-supported");
	gcpm->cop_warn_trigger = COP_WARN_DEFAULT_TRIGGER_COUNT;

	if (gcpm->cop_supported) {
		int irq_in = platform_get_irq(gcpm->pdev, 0);

		if (irq_in < 0) {
			if (irq_in == -EPROBE_DEFER) {
				dev_warn(gcpm->device, "IRQ wait, deferring probe.\n");
				return irq_in;
			}

			dev_err(gcpm->device, "%s failed to get irq ret = %d\n", __func__, irq_in);
		} else {
			ret = devm_request_threaded_irq(gcpm->device, irq_in, NULL,
							google_cpm_cop_warn_irq_handler,
							IRQF_TRIGGER_LOW |
							IRQF_SHARED |
							IRQF_ONESHOT,
							"google_cpm_cop_warn",
							gcpm);
			if (ret < 0)
				dev_err(gcpm->device, "Error setting up cop warn irq\n");
		}
	}

	dev_info(gcpm->device, "taper ts_m=%d ts_ccs=%d ts_i=%d ts_cnt=%d ts_g=%d ts_v=%d ts_c=%d\n",
		 gcpm->taper_step_fv_margin, gcpm->taper_step_cc_step,
		 gcpm->taper_step_interval, gcpm->taper_step_count,
		 gcpm->taper_step_grace, gcpm->taper_step_voltage,
		 gcpm->taper_step_current);

	/* GCPM_FCC has the current cc_max for the selected charger */
	gcpm->cp_votable =
		gvotable_create_int_election(NULL, gvotable_comparator_int_min,
					     gcpm_fcc_callback, gcpm);
	if (IS_ERR_OR_NULL(gcpm->cp_votable)) {
		ret = PTR_ERR(gcpm->cp_votable);
		dev_err(gcpm->device, "no GCPM_FCC votable (%d)\n", ret);
		return ret;
	}

	gvotable_set_default(gcpm->cp_votable, (void *)-1);
	gvotable_set_vote2str(gcpm->cp_votable, gvotable_v2s_int);
	gvotable_election_set_name(gcpm->cp_votable, "GCPM_FCC");

	/*
	 * WirelessDC and Wireless charging use different thermal limit.
	 * The limit (level) comes from the thermal cooling zone msc_fcc and is
	 * mutually exclusive with the vote on dc_icl
	 */
	gcpm->dc_fcc_votable =
		gvotable_create_int_election(NULL, gvotable_comparator_int_min,
					     gcpm_dc_fcc_callback, gcpm);
	if (IS_ERR_OR_NULL(gcpm->dc_fcc_votable)) {
		ret = PTR_ERR(gcpm->dc_fcc_votable);
		dev_err(gcpm->device, "no dc_fcc votable (%d)\n", ret);
		return ret;
	}

	gvotable_set_default(gcpm->dc_fcc_votable, (void *)-1);
	gvotable_set_vote2str(gcpm->dc_fcc_votable, gvotable_v2s_int);
	gvotable_election_set_name(gcpm->dc_fcc_votable, "DC_FCC");

	gcpm->dc_chg_avail_votable = gvotable_create_int_election(
			NULL, gvotable_comparator_int_min,
			gcpm_dc_chg_avail_callback, gcpm);

	if (IS_ERR_OR_NULL(gcpm->dc_chg_avail_votable)) {
		ret = PTR_ERR(gcpm->dc_chg_avail_votable);
		dev_err(gcpm->device, "no DC chg avail votable %d\n", ret);
		return ret;
	}

	gvotable_set_default(gcpm->dc_chg_avail_votable, (void *)1);
	gvotable_set_vote2str(gcpm->dc_chg_avail_votable, gvotable_v2s_int);
	gvotable_election_set_name(gcpm->dc_chg_avail_votable, VOTABLE_DC_CHG_AVAIL);
	gvotable_use_default(gcpm->dc_chg_avail_votable, true);

	/* sysfs & debug */
	gcpm->debug_entry = gcpm_init_fs(gcpm);
	if (!gcpm->debug_entry)
		pr_warn("No debug control\n");

	platform_set_drvdata(pdev, gcpm);

	psy_cfg.drv_data = gcpm;
	psy_cfg.of_node = pdev->dev.of_node;
	gcpm->psy = devm_power_supply_register(gcpm->device,
					       &gcpm_psy_desc.psy_dsc,
					       &psy_cfg);
	if (IS_ERR(gcpm->psy)) {
		ret = PTR_ERR(gcpm->psy);
		if (ret == -EPROBE_DEFER)
			return -EPROBE_DEFER;

		dev_err(gcpm->device, "Couldn't register gcpm, (%d)\n", ret);
		return -ENODEV;
	}

	gcpm->log = logbuffer_register("cpm");
	if (IS_ERR(gcpm->log)) {
		dev_err(gcpm->device, "Couldn't register logbuffer, (%ld)\n",
			PTR_ERR(gcpm->log));
		gcpm->log = NULL;
	}

	/* gcpm_pps_psy_cfg.of_node is used to find out the snk_pdos */
	gcpm_pps_psy_cfg.drv_data = gcpm;
	gcpm_pps_psy_cfg.of_node = pdev->dev.of_node;
	gcpm->pps_psy = devm_power_supply_register(gcpm->device,
						   &gcpm_pps_psy_desc,
						   &gcpm_pps_psy_cfg);
	if (IS_ERR(gcpm->pps_psy)) {
		ret = PTR_ERR(gcpm->pps_psy);
		if (ret == -EPROBE_DEFER)
			return -EPROBE_DEFER;

		dev_err(gcpm->device, "Couldn't register gcpm_pps (%d)\n", ret);
		return -ENODEV;
	}

	ret = device_create_file(gcpm->device, &dev_attr_dc_limit_demand);
	if (ret)
		dev_err(gcpm->device, "Failed to create dc_limit_demand\n");

	ret = device_create_file(gcpm->device, &dev_attr_dc_limit_vbatt_max);
	if (ret)
		dev_err(gcpm->device, "Failed to create dc_limit_vbatt_max\n");

	ret = device_create_file(gcpm->device, &dev_attr_dc_limit_vbatt_min);
	if (ret)
		dev_err(gcpm->device, "Failed to create dc_limit_vbatt_min\n");

	ret = device_create_file(gcpm->device, &dev_attr_wlc_dc_limit_vbatt_min);
	if (ret)
		dev_warn(gcpm->device, "Failed to create wlc_dc_limit_vbatt_min\n");

	ret = device_create_file(gcpm->device, &dev_attr_dc_ctl);
	if (ret)
		dev_err(gcpm->device, "Failed to create dc_crl\n");

	ret = device_create_file(gcpm->device, &dev_attr_thermal_mdis_fan_alarm);
	if (ret)
		dev_err(gcpm->device, "Failed to create thermal_mdis_fan_alarm\n");

	ret = device_create_file(gcpm->device, &dev_attr_mdis_vote);
	if (ret)
		dev_err(gcpm->device, "Failed to create mdis_vote\n");

	/* give time to fg driver to start */
	schedule_delayed_work(&gcpm->init_work,
			      msecs_to_jiffies(INIT_DELAY_MS));

	return 0;
}

static int google_cpm_remove(struct platform_device *pdev)
{
	struct gcpm_drv *gcpm = platform_get_drvdata(pdev);
	int i;

	if (!gcpm)
		return 0;

	power_supply_unreg_notifier(&gcpm->chg_nb);

	gvotable_destroy_election(gcpm->dc_fcc_votable);

	for (i = 0; i < gcpm->chg_psy_count; i++) {
		if (!gcpm->chg_psy_avail[i])
			continue;

		power_supply_put(gcpm->chg_psy_avail[i]);
		gcpm->chg_psy_avail[i] = NULL;
	}

	pps_free(&gcpm->wlc_pps_data);
	pps_free(&gcpm->tcpm_pps_data);

	wakeup_source_unregister(gcpm->gcpm_ws);

	if (gcpm->wlc_dc_psy)
		power_supply_put(gcpm->wlc_dc_psy);
	if (gcpm->log)
		logbuffer_unregister(gcpm->log);

	return 0;
}

static void google_cpm_shutdown(struct platform_device *pdev)
{
	struct gcpm_drv *gcpm = platform_get_drvdata(pdev);

	if (!gcpm)
		return;

	power_supply_unreg_notifier(&gcpm->chg_nb);
}

static int __maybe_unused gcpm_pm_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gcpm_drv *gcpm = platform_get_drvdata(pdev);

	if (gcpm->init_complete) {
		pm_runtime_get_sync(gcpm->device);
		gcpm->resume_complete = false;
		pm_runtime_put_sync(gcpm->device);
	}

	return 0;
}

static int __maybe_unused gcpm_pm_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gcpm_drv *gcpm = platform_get_drvdata(pdev);

	if (gcpm->init_complete) {
		pm_runtime_get_sync(gcpm->device);
		gcpm->resume_complete = true;
		pm_runtime_put_sync(gcpm->device);
	}

	return 0;
}

static SIMPLE_DEV_PM_OPS(gcpm_pm_ops,
			 gcpm_pm_suspend,
			 gcpm_pm_resume);

static const struct of_device_id google_cpm_of_match[] = {
	{.compatible = "google,cpm"},
	{},
};
MODULE_DEVICE_TABLE(of, google_cpm_of_match);


static struct platform_driver google_cpm_driver = {
	.driver = {
		   .name = "google_cpm",
		   .owner = THIS_MODULE,
		   .of_match_table = google_cpm_of_match,
		   .probe_type = PROBE_PREFER_ASYNCHRONOUS,
		   .pm = &gcpm_pm_ops,
		   },
	.probe = google_cpm_probe,
	.remove = google_cpm_remove,
	.shutdown = google_cpm_shutdown,
};

module_platform_driver(google_cpm_driver);

MODULE_DESCRIPTION("Google Charging Policy Manager");
MODULE_AUTHOR("AleX Pelosi <apelosi@google.com>");
MODULE_LICENSE("GPL");

#if 0

/* NOTE: call with a lock around gcpm->chg_psy_lock */
static int gcpm_dc_charging(struct gcpm_drv *gcpm)
{
	struct power_supply *dc_psy;
	int vchg, ichg, status;

	dc_psy = gcpm_chg_get_active(gcpm);
	if (!dc_psy) {
		pr_err("DC_CHG: invalid charger\n");
		return -ENODEV;
	}

	vchg = GPSY_GET_PROP(dc_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW);
	ichg = GPSY_GET_PROP(dc_psy, POWER_SUPPLY_PROP_CURRENT_NOW);
	status = GPSY_GET_PROP(dc_psy, POWER_SUPPLY_PROP_STATUS);

	pr_err("DC_CHG: vchg=%d, ichg=%d status=%d\n",
	       vchg, ichg, status);

	return 0;
}

static void gcpm_pps_dc_charging(struct gcpm_drv *gcpm)
{
	struct pd_pps_data *pps_data = &gcpm->pps_data;
	struct power_supply *pps_psy = gcpm->tcpm_psy;
	const int pre_out_ua = pps_data->op_ua;
	const int pre_out_uv = pps_data->out_uv;
	int ret, pps_ui = -ENODEV;

	if (gcpm->dc_state == DC_ENABLE) {
		struct pd_pps_data *pps_data = &gcpm->pps_data;
		bool pwr_ok;

		/* must run at the end of PPS negotiation */
		if (gcpm->out_ua == -1)
			gcpm->out_ua = min(gcpm->cc_max, pps_data->max_ua);
		if (gcpm->out_uv == -1) {
			struct power_supply *chg_psy =
						gcpm_chg_get_active(gcpm);
			unsigned long ta_max_v, value;
			int vbatt = -1;

			ta_max_v = pps_data->max_ua * pps_data->max_uv;
			ta_max_v /= gcpm->out_ua;
			if (ta_max_v > DC_TA_VMAX_MV)
				ta_max_v = DC_TA_VMAX_MV;

			if (chg_psy)
				vbatt = GPSY_GET_PROP(chg_psy,
						POWER_SUPPLY_PROP_VOLTAGE_NOW);
			if (vbatt < 0)
				vbatt = gcpm->fv_uv;
			if (vbatt < 0)
				vbatt = 0;

			/* good for pca9468 */
			value = 2 * vbatt + DC_VBATT_HEADROOM_MV;
			if (value < DC_TA_VMIN_MV)
				value = DC_TA_VMIN_MV;

			/* PPS voltage in 20mV steps */
			gcpm->out_uv = value - value % 20000;
		}

		pr_info("CHG_CHK: max_uv=%d,max_ua=%d  out_uv=%d,out_ua=%d\n",
			pps_data->max_uv, pps_data->max_ua,
			gcpm->out_uv, gcpm->out_ua);

		pps_ui = pps_update_adapter(pps_data, gcpm->out_uv,
					    gcpm->out_ua, pps_psy);
		if (pps_ui < 0)
			pps_ui = PPS_ERROR_RETRY_MS;

		/* wait until adapter is at or over request */
		pwr_ok = pps_data->out_uv == gcpm->out_uv &&
				pps_data->op_ua == gcpm->out_ua;
		if (pwr_ok) {
			ret = gcpm_chg_offline(gcpm);
			if (ret == 0)
				ret = gcpm_dc_start(gcpm, gcpm->dc_index);
			if (ret == 0) {
				gcpm->dc_state = DC_RUNNING;
				pps_ui = DC_RUN_DELAY_MS;
			}  else if (pps_ui > DC_ERROR_RETRY_MS) {
				pps_ui = DC_ERROR_RETRY_MS;
			}
		}

		/*
			* TODO: add retries and switch to DC_ENABLE again or to
			* DC_DISABLED on timeout.
			*/

		pr_info("PPS_DC: dc_state=%d out_uv=%d %d->%d, out_ua=%d %d->%d\n",
			gcpm->dc_state,
			pps_data->out_uv, pre_out_uv, gcpm->out_uv,
			pps_data->op_ua, pre_out_ua, gcpm->out_ua);
	} else if (gcpm->dc_state == DC_RUNNING)  {

		ret = gcpm_chg_ping(gcpm, 0, 0);
		if (ret < 0)
			pr_err("PPS_DC: ping failed with %d\n", ret);

		/* update gcpm->out_uv, gcpm->out_ua */
		pr_info("PPS_DC: dc_state=%d out_uv=%d %d->%d out_ua=%d %d->%d\n",
			gcpm->dc_state,
			pps_data->out_uv, pre_out_uv, gcpm->out_uv,
			pps_data->op_ua, pre_out_ua, gcpm->out_ua);

		ret = gcpm_dc_charging(gcpm);
		if (ret < 0)
			pps_ui = DC_ERROR_RETRY_MS;

		ret = pps_update_adapter(&gcpm->pps_data,
						gcpm->out_uv, gcpm->out_ua,
						pps_psy);
		if (ret < 0)
			pps_ui = PPS_ERROR_RETRY_MS;
	}

	return pps_ui;
}
#endif
