/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2018-2022 Google LLC
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
#include <linux/printk.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/thermal.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/usb/pd.h>
#include <linux/usb/tcpm.h>
#include <linux/alarmtimer.h>
#include "gbms_power_supply.h"
#include "google_bms.h"
#include "google_dc_pps.h"
#include "google_psy.h"

#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#endif

#define CHG_DELAY_INIT_MS		250
#define CHG_DELAY_INIT_DETECT_MS	1000

#define DEFAULT_CHARGE_STOP_LEVEL 100
#define DEFAULT_CHARGE_START_LEVEL 0

#define CHG_DRV_EAGAIN_RETRIES		3
#define CHG_WORK_ERROR_RETRY_MS		1000
#define CHG_WORK_EAGAIN_RETRY_MS	5000
#define CHG_WORK_BD_TRIGGERED_MS	(5 * 60 * 1000)

#define CHG_DRV_CC_HW_TOLERANCE_MAX	250

#define CHG_DRV_MODE_NOIRDROP	1
#define CHG_DRV_MODE_DISABLED	2
#define CHG_DRV_MODE_NOOP	3

#define DRV_DEFAULT_CC0UPDATE_INTERVAL	0
#define DRV_DEFAULTCC_UPDATE_INTERVAL	30000
#define DRV_DEFAULTCV_UPDATE_INTERVAL	2000

#define MAX_VOTER			"MAX_VOTER"
#define THERMAL_DAEMON_VOTER		"THERMAL_DAEMON_VOTER"
#define THERMAL_DAEMON_DC_VOTER		"THERMAL_DAEMON_DCVOTER"
#define USER_VOTER			"USER_VOTER"	/* same as QCOM */
#define MSC_CHG_VOTER			"msc_chg"
#define MSC_CHG_FULL_VOTER		"msc_chg_full"
#define MSC_USER_VOTER			"msc_user"
#define MSC_USER_CHG_LEVEL_VOTER	"msc_user_chg_level"
#define MSC_CHG_TERM_VOTER		"msc_chg_term"
#define MSC_PWR_VOTER			"msc_pwr_disable"
#define MSC_HDA_VOTER			"msc_hda"
#define TEMP_DRYRUN_VOTER		"TEMP_DRYRUN_VOTER"

#define CHG_TERM_LONG_DELAY_MS		300000	/* 5 min */
#define CHG_TERM_SHORT_DELAY_MS		60000	/* 1 min */
#define CHG_TERM_RETRY_MS		2000	/* 2 sec */
#define CHG_TERM_RETRY_CNT		5

#define MAX_BD_VOLTAGE	4300000
#define MIN_BD_VOLTAGE	3800000
#define MAX_BD_SOC	100
#define MIN_BD_SOC	0
#define MAX_BD_TEMP	500
#define MIN_BD_TEMP	0

#define FCC_OF_CDEV_NAME "google,charger"
#define FCC_CDEV_NAME "fcc"
#define WLC_OF_CDEV_NAME "google,wlc_charger"
#define WLC_CDEV_NAME "dc_icl"
#define WLC_FCC_OF_CDEV_NAME "google,wlc_fcc_charger"
#define WLC_FCC_CDEV_NAME "wlc_fcc"

#define PPS_CC_TOLERANCE_PCT_DEFAULT	5
#define PPS_CC_TOLERANCE_PCT_MAX	10

#define PDO_FIXED_FLAGS \
	(PDO_FIXED_DUAL_ROLE | PDO_FIXED_DATA_SWAP | PDO_FIXED_USB_COMM)
#define PD_SNK_MAX_MA			3000
#define PD_SNK_MAX_MA_9V		2200
#define OP_SNK_MW			7600 /* see b/159863291 */

/* type detection */
#define EXT1_DETECT_THRESHOLD_UV	(10500000)
#define EXT2_DETECT_THRESHOLD_UV	(5000000)

#define PSY_RETRY_LIMIT	120

#define usb_pd_is_high_volt(ad) \
	(((ad)->ad_type == CHG_EV_ADAPTER_TYPE_USB_PD || \
	(ad)->ad_type == CHG_EV_ADAPTER_TYPE_USB_PD_PPS) && \
	(ad)->ad_voltage * 100 > PD_SNK_MIN_MV)

/*
 * We can only log up to 10 levels - we reduce this as much as possible in the
 * DT by mapping the levels with google,thermal-stats-lvl-map.
 */
#define STATS_THERMAL_LEVELS_MAX (10)

/* keep the highest power (V/I) rating, as it may drop near EOC  */
#define UPDATE_ADAPTER_POWER(ad, voltage_max, amperage_max) \
	do { \
		voltage_max /= 100000; \
		amperage_max /= 100000; \
		if ((ad->ad_voltage * ad->ad_amperage) < (voltage_max * amperage_max)) { \
			ad->ad_voltage = voltage_max; \
			ad->ad_amperage = amperage_max; \
		} \
	} while (0)

struct chg_drv;

enum chg_thermal_devices {
	CHG_TERMAL_DEVICE_FCC = 0,
	CHG_TERMAL_DEVICE_DC_IN,
	CHG_TERMAL_DEVICE_WLC_FCC,

	CHG_TERMAL_DEVICES_COUNT,
};

enum dock_defend_state {
       DOCK_DEFEND_DISABLED = -1,
       DOCK_DEFEND_ENABLED = 0,
       DOCK_DEFEND_ACTIVE,
};

enum dock_defend_settings {
       DOCK_DEFEND_USER_DISABLED = -1,
       DOCK_DEFEND_USER_CLEARED = 0,
       DOCK_DEFEND_USER_ENABLED,
};

enum {
	ADAPTER_CAP_PDO,
	ADAPTER_CAP_APDO,
};

struct chg_thermal_device {
	struct chg_drv *chg_drv;

	struct thermal_cooling_device *tcd;
	int *thermal_mitigation;
	int *thermal_budgets;
	int thermal_levels;
	int current_level;
	int therm_fan_alarm_level;
};

struct chg_termination {
	bool enable;
	bool alarm_start;
	struct work_struct work;
	struct alarm alarm;
	int cc_full_ref;
	int retry_cnt;
	int usb_5v;
};

enum chg_input_suspend_state {
	CHG_INPUT_SUSPEND_DISABLED = 0,
	CHG_INPUT_SUSPEND_ENABLED_DISCHARGING,
	CHG_INPUT_SUSPEND_ENABLED_CHARGER_STATUS,
};

/* re-evaluate FCC when switching power supplies */
static int chg_therm_update_fcc(struct chg_drv *chg_drv);
static void chg_reset_termination_data(struct chg_drv *chg_drv);
static int chg_vote_input_suspend(struct chg_drv *chg_drv,
				  char *voter, bool suspend);

struct bd_data {
	u32 bd_trigger_voltage;	/* also recharge upper bound */
	u32 bd_trigger_temp;	/* single reading */
	u32 bd_trigger_time;	/* debounce window after trigger*/
	u32 bd_drainto_soc;
	u32 bd_recharge_voltage;
	u32 bd_recharge_soc;

	u32 bd_resume_abs_temp;	/* at any time after trigger */

	u32 bd_resume_soc;	/* any time after disconnect */
	u32 bd_resume_time;	/* debounce window for resume temp */
	u32 bd_resume_temp;	/* check resume_time after disconnect */

	long long temp_sum;
	ktime_t time_sum;

	int last_voltage;
	int last_temp;
	ktime_t last_update;

	ktime_t disconnect_time;
	u32 triggered;		/* (d) */
	u32 enabled;		/* (d) */
	u32 bd_temp_enable;	/* for UI setting interface */

	bool lowerbd_reached;
	bool bd_temp_dry_run;

	/* dock_defend */
	u32 dd_triggered;
	u32 dd_enabled;
	int dd_state;
	int dd_settings;
	int dd_charge_stop_level;
	int dd_charge_start_level;
	int dd_trigger_time;
	ktime_t dd_last_update;

	struct logbuffer *bd_log;

	/* charge stats */
	struct gbms_ce_tier_stats bd_pretrigger_stats;
	ktime_t bd_pretrigger_stats_last_update;
	struct gbms_ce_tier_stats bd_resume_stats;
	ktime_t bd_resume_stats_last_update;
};

struct chg_drv {
	struct device *device;

	struct power_supply *chg_psy;
	const char *chg_psy_name;
	struct power_supply *wlc_psy;
	const char *wlc_psy_name;
	struct power_supply *ext_psy;
	const char *ext_psy_name;
	struct power_supply *bat_psy;
	const char *bat_psy_name;
	struct power_supply *tcpm_psy;
	const char *tcpm_psy_name;
	int psy_retry_count;

	struct notifier_block psy_nb;
	struct delayed_work init_work;
	struct delayed_work chg_work;
	struct work_struct chg_psy_work;
	struct wakeup_source *chg_ws;
	struct alarm chg_wakeup_alarm;
	u32 tcpm_phandle;

	struct power_supply *usb_psy;
	const char *usb_psy_name;
	bool usb_skip_probe;

	/* thermal devices */
	struct chg_thermal_device thermal_devices[CHG_TERMAL_DEVICES_COUNT];
	bool therm_wlc_override_fcc;

	/* */
	u32 cv_update_interval;
	u32 cc_update_interval;
	union gbms_ce_adapter_details adapter_details;

	struct gvotable_election *msc_interval_votable;
	struct gvotable_election *msc_fv_votable;
	struct gvotable_election *msc_fcc_votable;
	struct gvotable_election *msc_chg_disable_votable;
	struct gvotable_election *msc_pwr_disable_votable;
	struct gvotable_election *msc_temp_dry_run_votable;
	struct gvotable_election *usb_icl_votable;
	struct gvotable_election *dc_suspend_votable;
	struct gvotable_election *dc_icl_votable;
	struct gvotable_election *dc_fcc_votable;
	struct gvotable_election *fan_level_votable;
	struct gvotable_election *dead_battery_votable;
	struct gvotable_election *tx_icl_votable;
	struct gvotable_election *msc_last_votable;
	struct gvotable_election *chg_mdis;
	struct gvotable_election *force_5v_votable;

	bool init_done;
	bool batt_present;
	bool dead_battery;
	int batt_profile_fcc_ua;	/* max/default fcc */
	int batt_profile_fv_uv;		/* max/default fv_uv */
	int fv_uv;
	int cc_max;
	int topoff;
	int chg_cc_tolerance;
	int chg_mode;			/* debug */
	int stop_charging;		/* no power source */
	int egain_retries;
	bool taper_last_tier;		/* set taper on last tier entry */

	/* retail & battery defender */
	struct delayed_work bd_work;

	struct mutex bd_lock;
	struct bd_data bd_state;
	struct wakeup_source *bd_ws;

	bool overheat;			/* battery reports overheat */
	bool freeze_soc;		/* battery soc freeze & thaw */

	int disable_charging;		/* retail or bd */
	int disable_pwrsrc;		/* retail or bd  */
	bool lowerdb_reached;		/* track recharge */

	int charge_stop_level;		/* retail, userspace bd config */
	int charge_start_level;		/* retail, userspace bd config */

	/* pps charging */
	bool pps_enable;
	struct pd_pps_data pps_data;
	unsigned int pps_cc_tolerance_pct;
	union gbms_charger_state chg_state;

	/* override voltage and current */
	bool enable_user_fcc_fv;
	int user_fv_uv;
	int user_cc_max;
	int user_interval;

	/* prevent overcharge */
	struct chg_termination	chg_term;

	/* CSI */
	struct gvotable_election *csi_status_votable;
	struct gvotable_election *csi_type_votable;
	struct gvotable_election *thermal_level_votable;

	/* debug */
	struct dentry *debug_entry;
	uint8_t debug_input_suspend;

	/* dock_defend */
	struct delayed_work bd_dd_work;
	bool ext_volt_complete;

	/* charge stats */
	struct mutex stats_lock;
	struct gbms_ce_tier_stats dd_stats;
	struct gbms_ce_tier_stats thermal_stats[STATS_THERMAL_LEVELS_MAX];
	ktime_t dd_stats_last_update;
	ktime_t thermal_stats_last_update;
	int thermal_stats_last_level;
	int *thermal_stats_mdis_levels;
	int thermal_levels_count;
	u32 adapter_capabilities[2];	/* 0: PDO max pwr, 1: APDO max pwr */

	/* charging policy */
	struct gvotable_election *charging_policy_votable;
	int charging_policy;

	int online;
	int present;
};

static void reschedule_chg_work(struct chg_drv *chg_drv)
{
	mod_delayed_work(system_wq, &chg_drv->chg_work, 0);
	pr_debug("%s: rescheduling\n", __func__);
}

static enum alarmtimer_restart
google_chg_alarm_handler(struct alarm *alarm, ktime_t time)
{
	struct chg_drv *chg_drv =
	    container_of(alarm, struct chg_drv, chg_wakeup_alarm);

	__pm_stay_awake(chg_drv->chg_ws);

	reschedule_chg_work(chg_drv);

	return ALARMTIMER_NORESTART;
}

static void chg_psy_work(struct work_struct *work)
{
	struct chg_drv *chg_drv =
		container_of(work, struct chg_drv, chg_psy_work);

	reschedule_chg_work(chg_drv);
}

/* cannot block: run in atomic context when called from chg_psy_changed() */
static int chg_psy_changed(struct notifier_block *nb,
		       unsigned long action, void *data)
{
	struct power_supply *psy = data;
	struct chg_drv *chg_drv = container_of(nb, struct chg_drv, psy_nb);

	dev_info_ratelimited(chg_drv->device, "%s name=%s evt=%lu\n", __func__,
			     psy->desc->name, action);

	if ((action != PSY_EVENT_PROP_CHANGED) ||
	    (psy == NULL) || (psy->desc == NULL) || (psy->desc->name == NULL))
		return NOTIFY_OK;

	if (action == PSY_EVENT_PROP_CHANGED &&
	    (!strcmp(psy->desc->name, chg_drv->chg_psy_name) ||
	     !strcmp(psy->desc->name, chg_drv->bat_psy_name) ||
	     (chg_drv->usb_psy_name &&
	      !strcmp(psy->desc->name, chg_drv->usb_psy_name)) ||
	     (chg_drv->tcpm_psy_name &&
	      !strcmp(psy->desc->name, chg_drv->tcpm_psy_name)) ||
	     (chg_drv->ext_psy_name &&
	      !strcmp(psy->desc->name, chg_drv->ext_psy_name)) ||
	     (chg_drv->wlc_psy_name &&
	      !strcmp(psy->desc->name, chg_drv->wlc_psy_name)))) {
		schedule_work(&chg_drv->chg_psy_work);
	}
	return NOTIFY_OK;
}

#if 0
static char *psy_usb_type_str[] = {
	"Unknown", "Battery", "UPS", "Mains", "USB",
	"USB_DCP", "USB_CDP", "USB_ACA", "USB_C",
	"USB_PD", "USB_PD_DRP", "BrickID",
	"USB_HVDCP", "USB_HVDCP_3", "Wireless", "USB_FLOAT",
	"BMS", "Parallel", "Main", "Wipower", "USB_C_UFP", "USB_C_DFP",
};
#endif

static char *psy_usb_type_str[] = {
	[POWER_SUPPLY_USB_TYPE_UNKNOWN] = "Unknown",
	[POWER_SUPPLY_USB_TYPE_SDP] = "USB",
	[POWER_SUPPLY_USB_TYPE_DCP] = "USB_DCP",
	[POWER_SUPPLY_USB_TYPE_CDP] = "USB_CDP",
	[POWER_SUPPLY_USB_TYPE_ACA] = "USB_ACA",
	[POWER_SUPPLY_USB_TYPE_C] =  "USB_C",
	[POWER_SUPPLY_USB_TYPE_PD] = "USB_PD",
	[POWER_SUPPLY_USB_TYPE_PD_DRP] = "USB_PD_DRP",
	[POWER_SUPPLY_USB_TYPE_PD_PPS] = "PPS",
	[POWER_SUPPLY_USB_TYPE_APPLE_BRICK_ID] = "BrickID",
};

static char *psy_usbc_type_str[] = {
	"Unknown", "SDP", "DCP", "CDP", "ACA", "C",
	"PD", "PD_DRP", "PD_PPS", "BrickID"
};

static int chg_work_read_soc(struct power_supply *bat_psy, int *soc);

static void chg_stats_init(struct gbms_ce_tier_stats *tier, int8_t idx)
{
	memset(tier, 0, sizeof(*tier)); /* Reset all of the previous data */
	gbms_tier_stats_init(tier, idx);
}

static void bd_dd_stats_init(struct chg_drv *chg_drv)
{
	ktime_t now = get_boot_sec();

	mutex_lock(&chg_drv->stats_lock);
	/* Dock Defend */
	chg_stats_init(&chg_drv->dd_stats, GBMS_STATS_BD_TI_DOCK);
	chg_drv->dd_stats_last_update = now;
	/* Temp Defend pretrigger */
	chg_stats_init(&chg_drv->bd_state.bd_pretrigger_stats, GBMS_STATS_BD_TI_TEMP_PRETRIGGER);
	chg_drv->bd_state.bd_pretrigger_stats_last_update = now;
	/* Temp Defend resume */
	chg_stats_init(&chg_drv->bd_state.bd_resume_stats, GBMS_STATS_BD_TI_TEMP_RESUME);
	chg_drv->bd_state.bd_resume_stats_last_update = 0;
	mutex_unlock(&chg_drv->stats_lock);
}

static int thermal_stats_lvl_to_vtier(int thermal_level)
{
	return thermal_level + GBMS_STATS_TH_LVL0;
}

static void thermal_stats_init(struct chg_drv *chg_drv)
{
	int i;

	mutex_lock(&chg_drv->stats_lock);
	for (i = 0; i < STATS_THERMAL_LEVELS_MAX; i++) {
		int8_t idx = thermal_stats_lvl_to_vtier(i);

		chg_stats_init(&chg_drv->thermal_stats[i], idx);
	}

	chg_drv->thermal_stats_last_update = get_boot_sec();
	chg_drv->thermal_stats_last_level = 0;
	mutex_unlock(&chg_drv->stats_lock);
}

static void chg_stats_update(struct chg_drv *chg_drv, struct gbms_ce_tier_stats *tier,
			     ktime_t *last_update)
{
	int ibatt_ma, temp;
	int cc, soc_in;
	ktime_t elap, now = get_boot_sec();
	int ioerr;

	mutex_lock(&chg_drv->stats_lock);
	if (tier->soc_in == -1)
		elap = 0;
	else
		elap = now - *last_update;
	*last_update = now;

	ibatt_ma = GPSY_GET_INT_PROP(chg_drv->bat_psy, POWER_SUPPLY_PROP_CURRENT_NOW, &ioerr);
	if (ioerr < 0) {
		pr_err("%s: read ibatt_ma=%d, ioerr=%d\n", __func__, ibatt_ma, ioerr);
		goto stats_update_unlock;
	}
	ibatt_ma /= 1000;

	temp = GPSY_GET_INT_PROP(chg_drv->bat_psy, POWER_SUPPLY_PROP_TEMP, &ioerr);
	if (ioerr < 0)
		goto stats_update_unlock;

	cc = GPSY_GET_INT_PROP(chg_drv->bat_psy, POWER_SUPPLY_PROP_CHARGE_COUNTER, &ioerr);
	if (ioerr < 0)
		goto stats_update_unlock;

	cc /= 1000;

	/* Ignore error in soc_in read */
	ioerr = chg_work_read_soc(chg_drv->bat_psy, &soc_in);
	if (ioerr != 0)
		soc_in = -1; /* Still allows for tier updates */

	gbms_stats_update_tier(0, ibatt_ma, temp, elap, cc, &chg_drv->chg_state, -1,
				soc_in << 8, tier);
stats_update_unlock:
	mutex_unlock(&chg_drv->stats_lock);
}

static void dd_stats_update(struct chg_drv *chg_drv)
{
	if (!chg_drv->bd_state.dd_triggered) {
		chg_drv->dd_stats_last_update = get_boot_sec();
		return;
	}

	chg_stats_update(chg_drv, &chg_drv->dd_stats, &chg_drv->dd_stats_last_update);
}

/* called on google_charger_init_work() and on every disconnect */
static inline void chg_init_state(struct chg_drv *chg_drv)
{
	/* reset retail state */
	chg_drv->disable_charging = -1;
	chg_drv->disable_pwrsrc = -1;
	chg_drv->lowerdb_reached = true;

	/* reset charging parameters */
	chg_drv->fv_uv = -1;
	chg_drv->cc_max = -1;
	chg_drv->chg_state.v = 0;
	gvotable_cast_int_vote(chg_drv->msc_fv_votable,
			       MSC_CHG_VOTER, chg_drv->fv_uv, false);
	gvotable_cast_int_vote(chg_drv->msc_fcc_votable,
			       MSC_CHG_VOTER, chg_drv->cc_max, false);
	chg_drv->egain_retries = 0;

	/* reset and re-enable PPS detection */
	pps_init_state(&chg_drv->pps_data);
	if (!chg_drv->pps_enable)
		chg_drv->pps_data.stage = PPS_DISABLED;
	else if (chg_drv->pps_data.nr_snk_pdo)
		chg_drv->pps_data.stage = PPS_NONE;
}

/*
 * called from google_charger direcly on a tcpm_psy.
 * NOTE: Do not call on anything else!
 */
static int chg_update_capability(struct power_supply *tcpm_psy, unsigned int nr_pdo,
				 u32 pps_cap)
{
	struct tcpm_port *port;
	const u32 pdo[] = {PDO_FIXED(5000, PD_SNK_MAX_MA, PDO_FIXED_FLAGS),
			   PDO_FIXED(PD_SNK_MAX_MV, PD_SNK_MAX_MA_9V, 0),
			   pps_cap};
	int ret = -ENODEV;

	if (!tcpm_psy || !nr_pdo || nr_pdo > PDO_MAX_SUPP)
		return -EINVAL;

	port = chg_get_tcpm_port(tcpm_psy);
	if (port)
		ret = tcpm_update_sink_capabilities(port, pdo, nr_pdo, OP_SNK_MW);

	return ret;
}

/* NOTE: doesn't reset chg_drv->adapter_details.v = 0 see chg_work() */
static inline int chg_reset_state(struct chg_drv *chg_drv)
{
	int ret = 0;

	chg_init_state(chg_drv);

	if (chg_drv->chg_term.enable)
		chg_reset_termination_data(chg_drv);

	if (!pps_is_disabled(chg_drv->pps_data.stage) || chg_drv->chg_term.usb_5v == 1) {
		unsigned int nr_pdo = chg_drv->pps_data.default_pps_pdo ?
				      PDO_PPS : PDO_FIXED_HIGH_VOLTAGE;

		chg_update_capability(chg_drv->tcpm_psy, nr_pdo,
				      chg_drv->pps_data.default_pps_pdo);
	}

	if (chg_drv->chg_term.usb_5v == 1)
		chg_drv->chg_term.usb_5v = 0;

	/* TODO: handle interaction with PPS code */
	gvotable_cast_int_vote(chg_drv->msc_interval_votable,
			       CHG_PPS_VOTER, 0, false);
	/* when/if enabled */
	GPSY_SET_PROP(chg_drv->chg_psy, GBMS_PROP_TAPER_CONTROL,
		      GBMS_TAPER_CONTROL_OFF);
	/* make sure the battery knows that it's disconnected */
	ret = GPSY_SET_INT64_PROP(chg_drv->bat_psy,
				  GBMS_PROP_CHARGE_CHARGER_STATE,
				  chg_drv->chg_state.v);
	/* handle google_battery not resume case */
	if (ret == -EAGAIN) {
		pr_debug("MSC_CHG: reset charger state failed %d", ret);
		return ret;
	}

	return 0;
}

/* definitions from FOREACH_CHG_EV_ADAPTER() in google_bms.h */
static int info_usb_ad_type(int usb_type, int usbc_type)
{
	switch (usb_type) {
	case POWER_SUPPLY_USB_TYPE_SDP:
		return (usbc_type == POWER_SUPPLY_USB_TYPE_PD_PPS) ?
			CHG_EV_ADAPTER_TYPE_USB_PD_PPS :
			CHG_EV_ADAPTER_TYPE_USB_SDP;
	case POWER_SUPPLY_USB_TYPE_CDP:
		return CHG_EV_ADAPTER_TYPE_USB_CDP;
	case POWER_SUPPLY_USB_TYPE_DCP:
		if (usbc_type == POWER_SUPPLY_USB_TYPE_PD)
			return CHG_EV_ADAPTER_TYPE_USB_PD;
		else if (usbc_type == POWER_SUPPLY_USB_TYPE_PD_PPS)
			return CHG_EV_ADAPTER_TYPE_USB_PD_PPS;
		else
			return CHG_EV_ADAPTER_TYPE_USB_DCP;
	case POWER_SUPPLY_USB_TYPE_PD:
		return (usbc_type == POWER_SUPPLY_USB_TYPE_PD_PPS) ?
			CHG_EV_ADAPTER_TYPE_USB_PD_PPS :
			CHG_EV_ADAPTER_TYPE_USB_PD;
	/* TODO: handle POWER_SUPPLY_TYPE_USB_FLOAT if available*/
	default:
		return CHG_EV_ADAPTER_TYPE_USB;
	}
}

static int chg_hda_tz_vote(int voltage_max, int amperage_max)
{
	struct gvotable_election *hda_tz_votable =
				gvotable_election_get_handle(VOTABLE_HDA_TZ);
	int power = (voltage_max * amperage_max) / 1000;
	int vote = HDA_TZ_NONE;

	if (!hda_tz_votable)
		return -1;

	if (power < 5000)
		vote = HDA_TZ_USB_SUB5W;
	else if (power == 5000)
		vote = HDA_TZ_USB_5W;
	else if (power <= 7500)
		vote = HDA_TZ_USB_7P5W;
	else if (power <= 15000)
		vote = HDA_TZ_USB_15W;
	else if (power <= 18000)
		vote = HDA_TZ_USB_18W;
	else if (power > 18000)
		vote = HDA_TZ_USB_GT18W;

	pr_debug("%s: voltage_max: %d, amp_max: %d, vote: %d\n",
		__func__, voltage_max, amperage_max, vote);

	gvotable_cast_int_vote(hda_tz_votable, MSC_HDA_VOTER, vote, true);
	return 0;
}

static int info_usb_state(union gbms_ce_adapter_details *ad,
			  struct chg_drv *chg_drv,
			  struct power_supply *usb_psy,
			  struct power_supply *tcpm_psy)
{
	const char *usb_type_str = psy_usb_type_str[0];
	int usb_type, voltage_max = -1, amperage_max = -1;
	int ad_type, usbc_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;

	if (usb_psy) {
		int voltage_now, current_now;

		/* TODO: handle POWER_SUPPLY_PROP_REAL_TYPE in qc-compat */
		usb_type = PSY_GET_PROP(usb_psy, POWER_SUPPLY_PROP_USB_TYPE);
		if (tcpm_psy) {
			usbc_type = PSY_GET_PROP(tcpm_psy,
						 POWER_SUPPLY_PROP_USB_TYPE);

			if(pps_get_src_cap(&chg_drv->pps_data, tcpm_psy) > 0) {
				pps_get_max_power(&chg_drv->pps_data,
						  &chg_drv->adapter_capabilities[ADAPTER_CAP_PDO],
						  true);
				pps_get_max_power(&chg_drv->pps_data,
						  &chg_drv->adapter_capabilities[ADAPTER_CAP_APDO],
						  false);
			}
		}

		voltage_max = PSY_GET_PROP(usb_psy,
					   POWER_SUPPLY_PROP_VOLTAGE_MAX);
		amperage_max = PSY_GET_PROP(usb_psy,
					    POWER_SUPPLY_PROP_CURRENT_MAX);
		voltage_now = PSY_GET_PROP(usb_psy,
					   POWER_SUPPLY_PROP_VOLTAGE_NOW);
		current_now = PSY_GET_PROP(usb_psy,
					   POWER_SUPPLY_PROP_CURRENT_NOW);

		if (usb_type < ARRAY_SIZE(psy_usb_type_str))
			usb_type_str = psy_usb_type_str[usb_type];

		pr_info("usbchg=%s typec=%s usbv=%d usbc=%d usbMv=%d usbMc=%d\n",
			usb_type_str,
			tcpm_psy ? psy_usbc_type_str[usbc_type] : "null",
			voltage_now < 0 ? voltage_now : voltage_now / 1000,
			current_now / 1000,
			voltage_max < 0 ? voltage_max : voltage_max / 1000,
			amperage_max < 0 ? amperage_max : amperage_max / 1000);

		chg_hda_tz_vote(voltage_max/1000, amperage_max/1000);
	}

	if (!ad)
		return 0;

	/* if data is not allowed, keep previous value */
	if (voltage_max < 0 || amperage_max < 0) {
		if (ad->ad_type == CHG_EV_ADAPTER_TYPE_UNKNOWN)
			ad->ad_type = CHG_EV_ADAPTER_TYPE_USB_UNKNOWN;
		return -EINVAL;
	}

	ad_type = info_usb_ad_type(usb_type, usbc_type);
	/* detect unknown when disconnect, ingnore this case */
	if (ad->ad_type > ad_type && ad->ad_type != CHG_EV_ADAPTER_TYPE_USB)
		return 0;

	ad->ad_type = ad_type;
	UPDATE_ADAPTER_POWER(ad, voltage_max, amperage_max);

	return 0;
}

static int info_wlc_state(union gbms_ce_adapter_details *ad,
			  struct power_supply *wlc_psy)
{
	int voltage_max, amperage_max;

	voltage_max = GPSY_GET_PROP(wlc_psy, POWER_SUPPLY_PROP_VOLTAGE_MAX);
	amperage_max = GPSY_GET_PROP(wlc_psy, POWER_SUPPLY_PROP_CURRENT_MAX);

	pr_info("wlcv=%d wlcc=%d wlcMv=%d wlcMc=%d wlct=%d vrect=%d opfreq=%d, vcpout=%d\n",
		GPSY_GET_PROP(wlc_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW) / 1000,
		GPSY_GET_PROP(wlc_psy, POWER_SUPPLY_PROP_CURRENT_NOW) / 1000,
		voltage_max < 0 ? voltage_max : voltage_max / 1000,
		amperage_max < 0 ? amperage_max : amperage_max / 1000,
		GPSY_GET_PROP(wlc_psy, POWER_SUPPLY_PROP_TEMP),
		GPSY_GET_PROP(wlc_psy, GBMS_PROP_WLC_VRECT) / 1000,
		GPSY_GET_PROP(wlc_psy, GBMS_PROP_WLC_OP_FREQ) / 1000,
		GPSY_GET_PROP(wlc_psy, GBMS_PROP_WLC_VCPOUT) / 1000);

	if (!ad)
		return 0;

	/* if data is not allowed, keep previous value */
	if (voltage_max < 0 || amperage_max < 0) {
		if (ad->ad_type == CHG_EV_ADAPTER_TYPE_UNKNOWN)
			ad->ad_type = CHG_EV_ADAPTER_TYPE_WLC_UNKNOWN;
		return -EINVAL;
	}

	/* keep original (dream defend might change its type) */
	if (ad->ad_voltage > voltage_max / 100000)
		return 0;

	ad->ad_type = CHG_EV_ADAPTER_TYPE_WLC;
	if (voltage_max >= WLC_EPP_THRESHOLD_UV) {
		ad->ad_type = CHG_EV_ADAPTER_TYPE_WLC_EPP;
	} else if (voltage_max >= WLC_BPP_THRESHOLD_UV) {
		ad->ad_type = CHG_EV_ADAPTER_TYPE_WLC_SPP;
	}

	UPDATE_ADAPTER_POWER(ad, voltage_max, amperage_max);

	return 0;
}

static int info_ext_state(union gbms_ce_adapter_details *ad,
			  struct power_supply *ext_psy)
{
	int voltage_max, amperage_max;

	voltage_max = PSY_GET_PROP(ext_psy, POWER_SUPPLY_PROP_VOLTAGE_MAX);
	amperage_max = PSY_GET_PROP(ext_psy, POWER_SUPPLY_PROP_CURRENT_MAX);

	pr_info("extv=%d extcc=%d extMv=%d extMc=%d\n",
		PSY_GET_PROP(ext_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW) / 1000,
		PSY_GET_PROP(ext_psy, POWER_SUPPLY_PROP_CURRENT_NOW) / 1000,
		voltage_max < 0 ? voltage_max : voltage_max / 1000,
		amperage_max < 0 ? amperage_max : amperage_max / 1000);

	if (!ad)
		return 0;

	/* if data is not allowed, keep previous value */
	if (voltage_max < 0 || amperage_max < 0) {
		if (ad->ad_type == CHG_EV_ADAPTER_TYPE_UNKNOWN)
			ad->ad_type = CHG_EV_ADAPTER_TYPE_EXT_UNKNOWN;
		return -EINVAL;
	}

	if (voltage_max > EXT1_DETECT_THRESHOLD_UV)
		ad->ad_type = CHG_EV_ADAPTER_TYPE_EXT1;
	else if (voltage_max > EXT2_DETECT_THRESHOLD_UV)
		ad->ad_type = CHG_EV_ADAPTER_TYPE_EXT2;
	else
		ad->ad_type = CHG_EV_ADAPTER_TYPE_EXT;

	UPDATE_ADAPTER_POWER(ad, voltage_max, amperage_max);

	return 0;
}

/* NOTE: do not call this directly */
static int chg_set_charger(struct chg_drv *chg_drv, int fv_uv, int cc_max, int topoff)
{
	struct power_supply *chg_psy = chg_drv->chg_psy;
	union power_supply_propval pval;
	int rc;

	/*
	 *   when cc_max <= chg_drv->cc_max, set current, voltage (if needed)
	 *   when cc_max > chg_drv->cc_max, set voltage (if needed), current
	 */
	if (cc_max <= chg_drv->cc_max) {
		pval.intval = cc_max;

		rc = power_supply_set_property(chg_psy, POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
					       &pval);
		if (rc == -EAGAIN)
			return rc;
		if (rc != 0) {
			pr_err("MSC_CHG cannot set charging current rc=%d\n", rc);
			return -EIO;
		}
	}

	if (fv_uv != chg_drv->fv_uv) {
		pval.intval = fv_uv;
		rc = power_supply_set_property(chg_psy, POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX, &pval);
		if (rc == -EAGAIN)
			return rc;
		if (rc != 0) {
			pr_err("MSC_CHG cannot set float voltage rc=%d\n", rc);
			return -EIO;
		}
	}

	if (cc_max > chg_drv->cc_max) {
		pval.intval = cc_max;

		rc = power_supply_set_property(chg_psy, POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
					       &pval);
		if (rc == -EAGAIN)
			return rc;
		if (rc != 0) {
			pr_err("MSC_CHG cannot set charging current rc=%d\n", rc);
			return -EIO;
		}
	}

	if (chg_drv->topoff != topoff) {
		pval.intval = topoff;

		rc = power_supply_set_property(chg_psy, POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT,
					       &pval);
		if (rc == -EAGAIN)
			return rc;
		if (rc != 0) {
			pr_err("MSC_CHG cannot set topoff current rc=%d\n", rc);
			return -EIO;
		}
	}

	return rc;
}

static int chg_update_charger(struct chg_drv *chg_drv, int fv_uv, int cc_max, int topoff)
{
	const int chg_cc_tolerance = chg_drv->chg_cc_tolerance;
	const bool fv_changed = chg_drv->fv_uv != fv_uv;
	const bool cc_max_changed = chg_drv->cc_max != cc_max;
	const bool topoff_changed = chg_drv->topoff != topoff;
	struct power_supply *chg_psy = chg_drv->chg_psy;
	int fcc = cc_max;
	int rc = 0;

	if (!chg_psy)
		return 0;

	if (fv_changed && !chg_drv->taper_last_tier) {
		const int taper_limit = chg_drv->batt_profile_fv_uv >= 0 ?
				chg_drv->batt_profile_fv_uv : -1;
		int taper_ctl = GBMS_TAPER_CONTROL_OFF;

		if (taper_limit > 0 && fv_uv >= taper_limit)
			taper_ctl = GBMS_TAPER_CONTROL_ON;

		/* GBMS_PROP_TAPER_CONTROL is optional */
		rc = GPSY_SET_PROP(chg_psy, GBMS_PROP_TAPER_CONTROL, taper_ctl);
		if (rc < 0)
			pr_debug("MSC_CHG cannot set taper control rc=%d\n", rc);
	}

	/* when set cc_tolerance needs to be applied to everything */
	if (chg_drv->chg_cc_tolerance)
		fcc = (cc_max / 1000) * (1000 - chg_cc_tolerance);

	rc = chg_set_charger(chg_drv, fv_uv, fcc, topoff);
	if (rc == 0 && (fv_changed || cc_max_changed || topoff_changed)) {
		pr_info("MSC_CHG fv_uv=%d->%d cc_max=%d->%d topoff=%d->%d rc=%d\n",
			chg_drv->fv_uv, fv_uv,
			chg_drv->cc_max, cc_max,
			chg_drv->topoff, topoff,
			rc);

		chg_drv->cc_max = cc_max;
		chg_drv->fv_uv = fv_uv;
		chg_drv->topoff = topoff;
	}

	return rc;
}

/* b/117985113 */
static int chg_usb_online(struct power_supply *usb_psy)
{
	int online, rc;

	if (!usb_psy)
		return 1;

#ifdef CONFIG_USB_ONLINE_IS_TYPEC_MODE
	online = PSY_GET_INT_PROP(usb_psy, POWER_SUPPLY_PROP_TYPEC_MODE, &rc);
	if (rc < 0)
		return rc;

	switch (online) {
	case POWER_SUPPLY_TYPEC_SOURCE_DEFAULT:
	case POWER_SUPPLY_TYPEC_SOURCE_MEDIUM:
	case POWER_SUPPLY_TYPEC_SOURCE_HIGH:
	case POWER_SUPPLY_TYPEC_DAM_MEDIUM:
		online = 1;
		break;
	default:
		online = 0;
		break;
	}
#else
	online = PSY_GET_INT_PROP(usb_psy, POWER_SUPPLY_PROP_ONLINE, &rc);
	if (rc < 0)
		return rc;
#endif

	return online;
}


static bool chg_is_custom_enabled(int upperbd, int lowerbd)
{
	/* disabled */
	if ((upperbd == DEFAULT_CHARGE_STOP_LEVEL) &&
	    (lowerbd == DEFAULT_CHARGE_START_LEVEL))
		return false;

	/* invalid */
	if ((upperbd < lowerbd) ||
	    (upperbd > DEFAULT_CHARGE_STOP_LEVEL) ||
	    (lowerbd < DEFAULT_CHARGE_START_LEVEL))
		return false;

	return true;
}

/* returns 1 if charging should be disabled given the current battery capacity
 * given in percent, return 0 if charging should happen
 */
static int chg_work_is_charging_disabled(struct chg_drv *chg_drv, int capacity)
{
	const int upperbd = chg_drv->charge_stop_level;
	const int lowerbd = chg_drv->charge_start_level;
	int disable_charging = 0;

	if (!chg_is_custom_enabled(upperbd, lowerbd))
		return 0;

	if (chg_drv->lowerdb_reached && upperbd <= capacity) {
		pr_info("MSC_CHG lowerbd=%d, upperbd=%d, capacity=%d, lowerdb_reached=1->0, charging off\n",
			lowerbd, upperbd, capacity);
		disable_charging = 1;
		chg_drv->lowerdb_reached = false;
	} else if (!chg_drv->lowerdb_reached && lowerbd < capacity) {
		pr_info("MSC_CHG lowerbd=%d, upperbd=%d, capacity=%d, charging off\n",
			lowerbd, upperbd, capacity);
		disable_charging = 1;
	} else if (!chg_drv->lowerdb_reached && capacity <= lowerbd) {
		pr_info("MSC_CHG lowerbd=%d, upperbd=%d, capacity=%d, lowerdb_reached=0->1, charging on\n",
			lowerbd, upperbd, capacity);
		chg_drv->lowerdb_reached = true;
	} else {
		pr_info("MSC_CHG lowerbd=%d, upperbd=%d, capacity=%d, charging on\n",
			lowerbd, upperbd, capacity);
	}

	return disable_charging;
}

static void chg_termination_work(struct work_struct *work)
{
	struct chg_termination *chg_term =
			container_of(work, struct chg_termination, work);
	struct chg_drv *chg_drv =
			container_of(chg_term, struct chg_drv, chg_term);
	struct power_supply *bat_psy = chg_drv->bat_psy;
	int rc, cc, full, delay = CHG_TERM_LONG_DELAY_MS;

	cc = GPSY_GET_INT_PROP(bat_psy, POWER_SUPPLY_PROP_CHARGE_COUNTER, &rc);
	if (rc == -EAGAIN) {
		chg_term->retry_cnt++;

		if (chg_term->retry_cnt <= CHG_TERM_RETRY_CNT) {
			pr_info("Get CHARGE_COUNTER fail, try_cnt=%d, rc=%d\n",
				chg_term->retry_cnt, rc);
			/* try again and keep the pm_stay_awake */
			alarm_start_relative(&chg_term->alarm,
					     ms_to_ktime(CHG_TERM_RETRY_MS));
			return;
		} else {
			goto error;
		}
	} else if (rc < 0) {
		goto error;
	}

	/* Reset count if read successfully */
	chg_term->retry_cnt = 0;

	if (chg_term->cc_full_ref == 0)
		chg_term->cc_full_ref = cc;

	/*
	 * Suspend/Unsuspend USB input to keep cc_soc within the 0.5% to 0.75%
	 * overshoot range of the cc_soc value at termination, to prevent
	 * overcharging.
	 */
	full = chg_term->cc_full_ref;
	if ((long)cc < DIV_ROUND_CLOSEST((long)full * 10050, 10000)) {
		chg_vote_input_suspend(chg_drv, MSC_CHG_TERM_VOTER, false);
		delay = CHG_TERM_LONG_DELAY_MS;
	} else if ((long)cc > DIV_ROUND_CLOSEST((long)full * 10075, 10000)) {
		chg_vote_input_suspend(chg_drv, MSC_CHG_TERM_VOTER, true);
		delay = CHG_TERM_SHORT_DELAY_MS;
	}

	pr_info("Prevent overcharge data: cc: %d, cc_full_ref: %d, delay: %d\n",
		cc, chg_term->cc_full_ref, delay);

	alarm_start_relative(&chg_term->alarm, ms_to_ktime(delay));

	pm_relax(chg_drv->device);
	return;

error:
	pr_info("Get CHARGE_COUNTER fail, rc=%d\n", rc);
	chg_reset_termination_data(chg_drv);
}

static enum alarmtimer_restart chg_termination_alarm_cb(struct alarm *alarm,
							ktime_t now)
{
	struct chg_termination *chg_term =
			container_of(alarm, struct chg_termination, alarm);
	struct chg_drv *chg_drv =
			container_of(chg_term, struct chg_drv, chg_term);

	pr_info("Prevent overcharge alarm triggered %lld\n",
		ktime_to_ms(now));

	pm_stay_awake(chg_drv->device);
	schedule_work(&chg_term->work);

	return ALARMTIMER_NORESTART;
}

static void chg_reset_termination_data(struct chg_drv *chg_drv)
{
	if (!chg_drv->chg_term.alarm_start)
		return;

	chg_drv->chg_term.alarm_start = false;
	alarm_cancel(&chg_drv->chg_term.alarm);
	cancel_work_sync(&chg_drv->chg_term.work);
	chg_vote_input_suspend(chg_drv, MSC_CHG_TERM_VOTER, false);
	chg_drv->chg_term.cc_full_ref = 0;
	chg_drv->chg_term.retry_cnt = 0;
	pm_relax(chg_drv->device);
}

static void chg_eval_chg_termination(struct chg_termination *chg_term)
{
	if (chg_term->alarm_start)
		return;

	/*
	 * Post charge termination, switch to BSM mode triggers the risk of
	 * over charging as BATFET opening may take some time post the necessity
	 * of staying in supplemental mode, leading to unintended charging of
	 * battery. Trigger the function once charging is completed
	 * to prevent overcharing.
	 */
	alarm_start_relative(&chg_term->alarm,
			     ms_to_ktime(CHG_TERM_LONG_DELAY_MS));
	chg_term->alarm_start = true;
	chg_term->cc_full_ref = 0;
	chg_term->retry_cnt = 0;
}

static int chg_work_batt_roundtrip(const union gbms_charger_state *chg_state,
				   struct power_supply *bat_psy,
				   int *fv_uv, int *cc_max)
{
	int rc;

	rc = GPSY_SET_INT64_PROP(bat_psy,
				 GBMS_PROP_CHARGE_CHARGER_STATE,
				 chg_state->v);
	if (rc == -EAGAIN) {
		return -EAGAIN;
	} else if (rc < 0) {
		pr_err("MSC_CHG error cannot set CHARGE_CHARGER_STATE rc=%d\n",
		       rc);
		return -EINVAL;
	}

	/* battery can return negative values for cc_max and fv_uv. */
	*cc_max = GPSY_GET_INT_PROP(bat_psy,
				    POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
				    &rc);
	if (rc < 0) {
		pr_err("MSC_CHG error reading cc_max (%d)\n", rc);
		return -EIO;
	}

	/* ASSERT: (chg_state.f.flags&GBMS_CS_FLAG_DONE) && cc_max == 0 */

	*fv_uv = GPSY_GET_INT_PROP(bat_psy,
				   POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
				   &rc);
	if (rc < 0) {
		pr_err("MSC_CHG error reading fv_uv (%d)\n", rc);
		return -EIO;
	}

	return 0;
}

/* 0 stop charging, positive keep going */
static int chg_work_next_interval(const struct chg_drv *chg_drv,
				  union gbms_charger_state *chg_state)
{
	int update_interval = chg_drv->cc_update_interval;

	switch (chg_state->f.chg_status) {
	case POWER_SUPPLY_STATUS_FULL:
		update_interval = 0;
		break;
	case POWER_SUPPLY_STATUS_CHARGING:
		break;
	case POWER_SUPPLY_STATUS_NOT_CHARGING:
		update_interval = chg_drv->cv_update_interval;
		break;
	case POWER_SUPPLY_STATUS_DISCHARGING:
		/* DISCHARGING only when not connected -> stop charging */
		update_interval = 0;
		break;
	case POWER_SUPPLY_STATUS_UNKNOWN:
		update_interval = chg_drv->cv_update_interval;
		break;
	default:
		pr_err("invalid charging status %d\n", chg_state->f.chg_status);
		update_interval = chg_drv->cv_update_interval;
		break;
	}

	return update_interval;
}

static void chg_work_adapter_details(union gbms_ce_adapter_details *ad,
				     int usb_online, int wlc_online,
				     int ext_online,
				     struct chg_drv *chg_drv)
{
	/* print adapter details, route after at the end */
	if (wlc_online)
		(void)info_wlc_state(ad, chg_drv->wlc_psy);
	if (ext_online)
		(void)info_ext_state(ad, chg_drv->ext_psy);
	if (usb_online)
		(void)info_usb_state(ad, chg_drv, chg_drv->usb_psy, chg_drv->tcpm_psy);
}

static bool chg_work_check_wlc_state(struct power_supply *wlc_psy)
{
	int wlc_online, wlc_present;

	if (!wlc_psy)
		return false;

	wlc_online = GPSY_GET_PROP(wlc_psy, POWER_SUPPLY_PROP_ONLINE);
	wlc_present = GPSY_GET_PROP(wlc_psy, POWER_SUPPLY_PROP_PRESENT);

	return wlc_online == 1 || wlc_present == 1;
}

static bool chg_work_check_usb_state(struct chg_drv *chg_drv)
{
	struct power_supply *usb_psy = chg_drv->tcpm_psy ? chg_drv->tcpm_psy : chg_drv->usb_psy;
	int usb_online = 0, usb_present = 0;

	usb_online = chg_usb_online(usb_psy);

	if (chg_drv->usb_psy)
		usb_present = PSY_GET_PROP(chg_drv->usb_psy, POWER_SUPPLY_PROP_PRESENT);

	return usb_online > 0 || usb_present == 1;
}

static bool chg_work_check_ext_state(struct power_supply *ext_psy)
{
	int ext_online, ext_present;

	if (!ext_psy)
		return false;

	ext_online = PSY_GET_PROP(ext_psy, POWER_SUPPLY_PROP_ONLINE);
	ext_present = PSY_GET_PROP(ext_psy, POWER_SUPPLY_PROP_PRESENT);

	return ext_online == 1 || ext_present == 1;
}

/* not executed when battery is NOT present */
static int chg_work_roundtrip(struct chg_drv *chg_drv,
			      union gbms_charger_state *chg_state)
{
	struct power_supply *chg_psy = chg_drv->chg_psy;
	struct power_supply *wlc_psy = chg_drv->wlc_psy;
	struct power_supply *ext_psy = chg_drv->ext_psy;
	union gbms_charger_state batt_chg_state;
	const int upperbd = chg_drv->charge_stop_level;
	const int lowerbd = chg_drv->charge_start_level;
	int fv_uv = -1, cc_max = -1;
	int update_interval, rc;
	bool wlc_on, usb_on, ext_on;

	rc = gbms_read_charger_state(chg_state, chg_psy);
	if (rc < 0)
		return rc;

	/* NOIRDROP is default for remote sensing */
	if (chg_drv->chg_mode == CHG_DRV_MODE_NOIRDROP)
		chg_state->f.vchrg = 0;

	if (chg_is_custom_enabled(upperbd, lowerbd))
		chg_state->f.flags |= GBMS_CS_FLAG_CCLVL;

	if (chg_drv->debug_input_suspend) {
		chg_state->f.flags |= GBMS_CS_FLAG_INPUT_SUSPEND;

		if (chg_drv->debug_input_suspend == CHG_INPUT_SUSPEND_ENABLED_DISCHARGING) {
			chg_state->f.flags &= ~GBMS_CS_FLAG_BUCK_EN;
			chg_state->f.chg_status = POWER_SUPPLY_STATUS_DISCHARGING;
		}
	}

	/*
	 * The trigger of DREAM-DEFEND might look like a short disconnect.
	 * NOTE: This logic needs to be moved out of google_charger and fixed
	 * when we take care of b/202216343. google_charger should not be made
	 * aware of DD or other device-dependent low level tricks.
	 */
	batt_chg_state.v = chg_state->v;

	/*
	 * Sending _NOT_CHARGING down to the battery (with buck_en=0) while on
	 * WLC will keep dream defend stats in the same charging session.
	 * Add usb_state to prevent disconnection false positives, which may
	 * log data incorrectly
	 */
	wlc_on = chg_work_check_wlc_state(wlc_psy);
	usb_on = chg_work_check_usb_state(chg_drv);
	ext_on = chg_work_check_ext_state(ext_psy);
	if ((wlc_on || usb_on || ext_on) &&
	    batt_chg_state.f.chg_status == POWER_SUPPLY_STATUS_DISCHARGING) {
		if (!chg_drv->debug_input_suspend)
			batt_chg_state.f.chg_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		batt_chg_state.f.flags = gbms_gen_chg_flags(chg_state->f.chg_status,
							    chg_state->f.chg_type);
	}

	pr_debug("%s: wlc_on=%d usb_on=%d chg_state=%llx batt_chg_state=%llx\n", __func__,
		 wlc_on, usb_on, chg_state->v, batt_chg_state.v);

	/* might return negative values in fv_uv and cc_max */
	rc = chg_work_batt_roundtrip(&batt_chg_state, chg_drv->bat_psy,
				     &fv_uv, &cc_max);
	if (rc < 0)
		return rc;

	/*
	 * on fv_uv < 0 (eg JEITA tripped) in the middle of charging keep
	 * charging voltage steady. fv_uv will get the max value (if set in
	 * device tree) if routtrip return a negative value on connect.
	 * Sanity check on current make sure that cc_max doesn't jump to max.
	 */
	if (fv_uv < 0)
		fv_uv = chg_drv->fv_uv;
	if  (cc_max < 0)
		cc_max = chg_drv->cc_max;

	/*
	 * battery has already voted on these with MSC_LOGIC
	 * TODO: could use fv_uv<0 to enable/disable a safe charge voltage
	 * TODO: could use cc_max<0 to enable/disable a safe charge current
	 */
	gvotable_cast_int_vote(chg_drv->msc_fv_votable,
			       MSC_CHG_VOTER, fv_uv,
			       (chg_drv->user_fv_uv == -1) && (fv_uv > 0));
	gvotable_cast_int_vote(chg_drv->msc_fcc_votable,
			       MSC_CHG_VOTER, cc_max,
			       (chg_drv->user_cc_max == -1) && (cc_max >= 0));

	/*
	 * determine next undate interval only looking at the charger state
	 * and adjust using the new charging parameters from the battery.
	 * NOTE: chg_drv->cc_max and chg_drv->fv_uv are from the PREVIOUS
	 * update. cc_max and fv_uv come from the current roundtrip to
	 * the battery.
	 */
	update_interval = chg_work_next_interval(chg_drv, chg_state);

	pr_debug("%s: chg_drv->cc_max=%d cc_max=%d, update_interval=%d\n",
		 __func__, chg_drv->cc_max, cc_max, update_interval);

	/* no need to poll on cc_max==0 since, will be NOT charging */
	if (cc_max == 0) {
		pr_debug("%s: update_interval=%d->%d\n", __func__,
			 update_interval, DRV_DEFAULT_CC0UPDATE_INTERVAL);
		update_interval = DRV_DEFAULT_CC0UPDATE_INTERVAL;
	}

	/* update_interval <= 0 means stop charging */
	if (update_interval <= 0)
		return update_interval;

	/* no ping will cause timeout when pps_is_disabled() */
	if (chg_drv->tcpm_psy && !pps_is_disabled(chg_drv->pps_data.stage)) {
		int pps_ui;

		pps_ui = pps_work(&chg_drv->pps_data, chg_drv->tcpm_psy);
		if (pps_ui < 0)
			pps_ui = MSEC_PER_SEC;

		gvotable_cast_int_vote(chg_drv->msc_interval_votable,
				       CHG_PPS_VOTER, pps_ui, (pps_ui != 0));
	}

	/*
	 * update_interval <= 0 means stop charging
	 * NOTE: pps code moved to msc_update_charger_cb()
	 */
	return update_interval;
}

/* true if still in dead battery */
#define DEAD_BATTERY_DEADLINE_SEC	(45 * 60)

static bool chg_update_dead_battery(struct chg_drv *chg_drv)
{
	int dead = 0;
	const ktime_t uptime = get_boot_sec();

	/* will clear after deadline no matter what */
	if (uptime < DEAD_BATTERY_DEADLINE_SEC)
		dead = GPSY_GET_PROP(chg_drv->bat_psy, GBMS_PROP_DEAD_BATTERY);

	/* TODO: will stop looking for the votable after DEAD_BATTERY_DEADLINE_SEC */
	if (dead)
		return true;

	if (!chg_drv->dead_battery_votable)
		chg_drv->dead_battery_votable =
			gvotable_election_get_handle(VOTABLE_DEAD_BATTERY);
	if (chg_drv->dead_battery_votable) {
		gvotable_cast_int_vote(chg_drv->dead_battery_votable,
				       "MSC_BATT", 0, true);
		pr_info("dead battery cleared uptime=%lld\n", uptime);
	} else {
		pr_warn("dead battery cleared but no votable, uptime=%lld\n", uptime);
	}

	return false;
}


/* check for retail, battery defender */
static void chg_update_charging_state(struct chg_drv *chg_drv,
				      int disable_charging,
				      int disable_pwrsrc)
{
	/* disable charging is set in retail mode */
	if (disable_charging != chg_drv->disable_charging) {
		pr_info("MSC_CHG disable_charging %d -> %d",
			chg_drv->disable_charging, disable_charging);

		/* voted but not applied since msc_interval_votable <= 0 */
		gvotable_cast_int_vote(chg_drv->msc_fcc_votable,
				       MSC_USER_CHG_LEVEL_VOTER,
				       0, disable_charging != 0);
	}
	chg_drv->disable_charging = disable_charging;

	/* when disable_pwrsrc is set, disable_charging is set also */
	if (disable_pwrsrc != chg_drv->disable_pwrsrc) {
		pr_info("MSC_CHG disable_pwrsrc %d -> %d",
			chg_drv->disable_pwrsrc, disable_pwrsrc);

		/* applied right away */
		gvotable_cast_bool_vote(chg_drv->msc_pwr_disable_votable,
					MSC_USER_CHG_LEVEL_VOTER,
					disable_pwrsrc != 0);

		/* take a wakelock while discharging */
		if (disable_pwrsrc)
			__pm_stay_awake(chg_drv->bd_ws);
		else
			__pm_relax(chg_drv->bd_ws);

	}
	chg_drv->disable_pwrsrc = disable_pwrsrc;
}

/*
 * called on init and to reset the trigger
 * TEMP-DEFEND needs to have triggers for voltage, time and temperature, a
 * recharge voltage and some way to resume normal behavior. Resume can happen
 * based on at least ONE of the following criteria:
 * - when battery temperature fall under a limit
 * - at disconnect
 * - after disconnect if the SOC drops under limit
 * - some time after disconnect (optional if temperature under limit)
 */
static void bd_reset(struct bd_data *bd_state)
{
	bool can_resume = bd_state->bd_resume_abs_temp ||
			  bd_state->bd_resume_time ||
			  (bd_state->bd_resume_time == 0 &&
			  bd_state->bd_resume_soc == 0);

	bd_state->time_sum = 0;
	bd_state->temp_sum = 0;
	bd_state->last_update = 0;
	bd_state->last_voltage = 0;
	bd_state->last_temp = 0;
	bd_state->triggered = 0;
	bd_state->dd_triggered = 0;
	bd_state->dd_last_update = 0;

	/* also disabled when externally triggered, resume_temp is optional */
	bd_state->enabled = ((bd_state->bd_trigger_voltage &&
				bd_state->bd_recharge_voltage) ||
				(bd_state->bd_drainto_soc &&
				bd_state->bd_recharge_soc)) &&
			    bd_state->bd_trigger_time &&
			    bd_state->bd_trigger_temp &&
			    bd_state->bd_temp_enable &&
			    can_resume;
}

/* Defender */
static void bd_init(struct bd_data *bd_state, struct device *dev)
{
	int ret;

	ret = of_property_read_u32(dev->of_node, "google,bd-trigger-voltage",
				   &bd_state->bd_trigger_voltage);
	if (ret < 0)
		bd_state->bd_trigger_voltage = 0;

	ret = of_property_read_u32(dev->of_node, "google,bd-drainto-soc",
				   &bd_state->bd_drainto_soc);
	if (ret < 0)
		bd_state->bd_drainto_soc = 0;

	ret = of_property_read_u32(dev->of_node, "google,bd-trigger-temp",
				   &bd_state->bd_trigger_temp);
	if (ret < 0)
		bd_state->bd_trigger_temp = 0;

	ret = of_property_read_u32(dev->of_node, "google,bd-trigger-time",
				   &bd_state->bd_trigger_time);
	if (ret < 0)
		bd_state->bd_trigger_time = 0; /* hours */

	ret = of_property_read_u32(dev->of_node, "google,bd-recharge-voltage",
				   &bd_state->bd_recharge_voltage);
	if (ret < 0)
		bd_state->bd_recharge_voltage = 0;

	ret = of_property_read_u32(dev->of_node, "google,bd-recharge-soc",
				   &bd_state->bd_recharge_soc);
	if (ret < 0)
		bd_state->bd_recharge_soc = 0;

	ret = of_property_read_u32(dev->of_node, "google,bd-resume-abs-temp",
				   &bd_state->bd_resume_abs_temp);
	if (ret < 0)
		bd_state->bd_resume_abs_temp = 0;

	ret = of_property_read_u32(dev->of_node, "google,bd-resume-soc",
				   &bd_state->bd_resume_soc);
	if (ret < 0)
		bd_state->bd_resume_soc = 0;

	ret = of_property_read_u32(dev->of_node, "google,bd-resume-temp",
				   &bd_state->bd_resume_temp);
	if (ret < 0)
		bd_state->bd_resume_temp = 0;

	ret = of_property_read_u32(dev->of_node, "google,bd-resume-time",
				   &bd_state->bd_resume_time);
	if (ret < 0)
		bd_state->bd_resume_time = 0;

	bd_state->bd_temp_dry_run =
		 of_property_read_bool(dev->of_node, "google,bd-temp-dry-run");

	bd_state->bd_temp_enable =
		 of_property_read_bool(dev->of_node, "google,bd-temp-enable");

	/* also call to resume charging */
	bd_reset(bd_state);
	if (!bd_state->enabled)
		dev_warn(dev, "TEMP-DEFEND not enabled\n");

	bd_state->bd_log = logbuffer_register("bd");
	if (IS_ERR(bd_state->bd_log)) {
		ret = PTR_ERR(bd_state->bd_log);
		dev_err(dev, "failed to obtain logbuffer, ret=%d\n", ret);
		bd_state->bd_log = NULL;
	}

	gbms_logbuffer_prlog(bd_state->bd_log, LOGLEVEL_INFO, 0, LOGLEVEL_INFO,
		"MSC_BD: trig volt=%d,%d temp=%d,time=%d drainto=%d,%d resume=%d,%d %d,%d",
		bd_state->bd_trigger_voltage, bd_state->bd_recharge_voltage,
		bd_state->bd_trigger_temp, bd_state->bd_trigger_time,
		bd_state->bd_drainto_soc, bd_state->bd_recharge_soc,
		bd_state->bd_resume_abs_temp, bd_state->bd_resume_soc,
		bd_state->bd_resume_temp, bd_state->bd_resume_time);
}

static void bd_fan_vote(struct chg_drv *chg_drv, bool enable, int level)
{
	if (!chg_drv->fan_level_votable)
		chg_drv->fan_level_votable =
			gvotable_election_get_handle("FAN_LEVEL");
	if (chg_drv->fan_level_votable)
		gvotable_cast_int_vote(chg_drv->fan_level_votable,
					"MSC_BD", level, enable);
}

#define FAN_BD_LIMIT_ALARM	75
#define FAN_BD_LIMIT_HIGH	50
#define FAN_BD_LIMIT_MED	25
static int bd_fan_calculate_level(struct bd_data *bd_state)
{
	const u32 t = bd_state->bd_trigger_time;
	const ktime_t bd_fan_alarm = t * FAN_BD_LIMIT_ALARM / 100;
	const ktime_t bd_fan_high = t * FAN_BD_LIMIT_HIGH / 100;
	const ktime_t bd_fan_med = t * FAN_BD_LIMIT_MED / 100;
	long long temp_avg = 0;
	int bd_fan_level = FAN_LVL_NOT_CARE;

	if (bd_state->bd_temp_dry_run)
		return FAN_LVL_NOT_CARE;

	if (bd_state->time_sum)
		temp_avg = div64_s64(bd_state->temp_sum, bd_state->time_sum);

	if (temp_avg < bd_state->bd_trigger_temp)
		bd_fan_level = FAN_LVL_NOT_CARE;
	else if (bd_state->time_sum >= bd_fan_alarm)
		bd_fan_level = FAN_LVL_ALARM;
	else if (bd_state->time_sum >= bd_fan_high)
		bd_fan_level = FAN_LVL_HIGH;
	else if (bd_state->time_sum >= bd_fan_med)
		bd_fan_level = FAN_LVL_MED;

	pr_debug("bd_fan_level:%d, time_sum:%lld, temp_avg:%lld\n",
		 bd_fan_level, bd_state->time_sum, temp_avg);

	return bd_fan_level;
}

static void thermal_stats_update(struct chg_drv *chg_drv)
{
	int i;
	int thermal_level;

	if (chg_drv->thermal_levels_count <= 0 &&
	    chg_drv->thermal_devices[CHG_TERMAL_DEVICE_FCC].thermal_levels <= 0)
		return; /* Don't log any stats if there is nothing in the DT */

	if (!chg_drv->chg_mdis)
		chg_drv->chg_mdis = gvotable_election_get_handle(VOTABLE_MDIS);
	if (chg_drv->chg_mdis)
		thermal_level = gvotable_get_current_int_vote(chg_drv->chg_mdis);
	else
		thermal_level = chg_drv->thermal_devices[CHG_TERMAL_DEVICE_FCC].current_level;

	if (chg_drv->thermal_levels_count) {
		/* Translate the thermal tier to a stats tier */
		for (i = 0; i < chg_drv->thermal_levels_count; i++)
			if (thermal_level <= chg_drv->thermal_stats_mdis_levels[i])
				break;
	} else {
		/* Traditional thermal setting */
		i = thermal_level;
	}

	/* Note; we do not report level 0 (eg. mdis_level == 0) */
	if (i >= STATS_THERMAL_LEVELS_MAX)
		return;

	/* If current level and last level are both 0, skip vote */
	if (i == 0 && chg_drv->thermal_stats_last_level == 0) {
		mutex_lock(&chg_drv->stats_lock);
		chg_drv->thermal_stats_last_update = get_boot_sec();
		mutex_unlock(&chg_drv->stats_lock);
		return;
	}

	gvotable_cast_int_vote(chg_drv->thermal_level_votable, "THERMAL_UPDATE",
			       i, i > 0 ? true : false);
}

static int thermal_tier_stats_update(struct gvotable_election *el,
				     const char *reason, void *vote)
{
	struct chg_drv *chg_drv = gvotable_get_data(el);
	int thermal_level = GVOTABLE_PTR_TO_INT(vote);
	int thermal_tier_idx;

	mutex_lock(&chg_drv->stats_lock);
	thermal_tier_idx = thermal_level ? thermal_level : chg_drv->thermal_stats_last_level;
	chg_drv->thermal_stats_last_level = thermal_level;
	mutex_unlock(&chg_drv->stats_lock);

	chg_stats_update(chg_drv, &chg_drv->thermal_stats[thermal_tier_idx],
			 &chg_drv->thermal_stats_last_update);

	return 0;
}

static int msc_temp_defend_dryrun_cb(struct gvotable_election *el,
				     const char *reason, void *vote)
{
	struct chg_drv *chg_drv = gvotable_get_data(el);
	int is_dry_run = GVOTABLE_PTR_TO_INT(vote);

	chg_drv->bd_state.bd_temp_dry_run = !!is_dry_run;

	return 0;
}

static void bd_resume(struct chg_drv *chg_drv)
{
	struct bd_data *bd_state = &chg_drv->bd_state;

	bd_reset(bd_state);
	chg_stats_update(chg_drv, &bd_state->bd_resume_stats,
			 &bd_state->bd_resume_stats_last_update);
}

/* bd_state->triggered = 1 when charging needs to be disabled */
static int bd_update_stats(struct chg_drv *chg_drv)
{
	struct bd_data *bd_state = &chg_drv->bd_state;
	struct power_supply *bat_psy = chg_drv->bat_psy;
	const bool triggered = bd_state->triggered;
	const ktime_t now = get_boot_sec();
	int ret, vbatt, temp;
	long long temp_avg;
	unsigned long long elap;

	if (!bd_state->enabled)
		return 0;

	vbatt = GPSY_GET_INT_PROP(bat_psy, POWER_SUPPLY_PROP_VOLTAGE_AVG, &ret);
	if (ret < 0)
		return ret;

	/* not over vbat and !triggered, nothing to see here */
	if (vbatt < bd_state->bd_trigger_voltage && !triggered)
		return 0;

	/* it needs to keep averaging after trigger */
	temp = GPSY_GET_INT_PROP(bat_psy, POWER_SUPPLY_PROP_TEMP, &ret);
	if (ret < 0)
		return ret;

	/* it needs to keep averaging if triggered */
	if (bd_state->last_update == 0)
		bd_state->last_update = now;

	/*
	 * b/294978951 time_sum is abnormally large and triggered the TEMP-DEFEND
	 * add log here if elapse exceed 2 times of schedule time
	 */
	elap = now - bd_state->last_update;
	if (temp >= bd_state->bd_trigger_temp) {
		if (elap > (CHG_WORK_BD_TRIGGERED_MS / 1000 * 2))
			gbms_logbuffer_prlog(bd_state->bd_log, LOGLEVEL_INFO, 0, LOGLEVEL_INFO,
				"MSC_BD: longer elap %llu (%llu - %llu), temp=%d, time_sum=%llu, temp_sum=%llu",
				elap, now, bd_state->last_update, temp, bd_state->time_sum,
				bd_state->temp_sum);
		bd_state->time_sum += elap;
		bd_state->temp_sum += temp * elap;
	}

	bd_state->last_voltage = vbatt;
	bd_state->last_temp = temp;
	bd_state->last_update = now;

	if (bd_state->time_sum != 0 && !triggered)
		chg_stats_update(chg_drv, &chg_drv->bd_state.bd_pretrigger_stats,
				 &chg_drv->bd_state.bd_pretrigger_stats_last_update);

	/* wait until we have at least bd_trigger_time */
	if (bd_state->time_sum < bd_state->bd_trigger_time)
		return 0;

	/* exit and entry criteria on temperature while connected */
	temp_avg = div64_s64(bd_state->temp_sum, bd_state->time_sum);
	if (triggered && temp <= bd_state->bd_resume_abs_temp) {
		gbms_logbuffer_prlog(bd_state->bd_log, LOGLEVEL_INFO, 0, LOGLEVEL_INFO,
			"MSC_BD: resume time_sum=%lld, temp_sum=%lld, temp_avg=%lld",
			bd_state->time_sum, bd_state->temp_sum, temp_avg);
		bd_resume(chg_drv);
	} else if (!triggered && temp_avg >= bd_state->bd_trigger_temp) {
		gbms_logbuffer_prlog(bd_state->bd_log, LOGLEVEL_INFO, 0, LOGLEVEL_INFO,
			"MSC_BD: trigger time_sum=%lld, temp_sum=%lld, temp_avg=%lld",
			bd_state->time_sum, bd_state->temp_sum, temp_avg);
		bd_state->triggered = 1;
	}

	return 0;
}

/* @return true if BD needs to be triggered */
static int bd_recharge_logic(struct bd_data *bd_state, int val)
{
	int lowerbd, upperbd;
	int disable_charging = 0;

	if (!bd_state->triggered && bd_state->dd_triggered) {
		lowerbd = bd_state->dd_charge_start_level;
		upperbd = bd_state->dd_charge_stop_level;
		goto recharge_logic;
	}

	if (bd_state->bd_drainto_soc && bd_state->bd_recharge_soc) {
		lowerbd = bd_state->bd_recharge_soc;
		upperbd = bd_state->bd_drainto_soc;
	} else if (bd_state->bd_recharge_voltage &&
			bd_state->bd_trigger_voltage) {
		lowerbd = bd_state->bd_recharge_voltage;
		upperbd = bd_state->bd_trigger_voltage;
	} else {
		return 0;
	}

	if (bd_state->bd_temp_dry_run)
		return 0;

recharge_logic:
	if (!bd_state->triggered && !bd_state->dd_triggered)
		return 0;

	/* recharge logic between bd_recharge_voltage and bd_trigger_voltage */
	if (bd_state->lowerbd_reached && val >= upperbd) {
		pr_info("MSC_BD lowerbd=%d, upperbd=%d, val=%d, lowerbd_reached=1->0, charging off\n",
			lowerbd, upperbd, val);
		bd_state->lowerbd_reached = false;
		disable_charging = 1;
	} else if (!bd_state->lowerbd_reached && val > lowerbd) {
		pr_info("MSC_BD lowerbd=%d, upperbd=%d, val=%d, charging off\n",
			lowerbd, upperbd, val);
		disable_charging = 1;
	} else if (!bd_state->lowerbd_reached && val <= lowerbd) {
		pr_info("MSC_BD lowerbd=%d, upperbd=%d, val=%d, lowerbd_reached=0->1, charging on\n",
			lowerbd, upperbd, val);
		bd_state->lowerbd_reached = true;
	} else {
		pr_info("MSC_BD lowerbd=%d, upperbd=%d, val=%d, charging on\n",
			lowerbd, upperbd, val);
	}

	return disable_charging;
}

/* ignore the failure to set CAPACITY: it might not be implemented */
static int bd_batt_set_soc(struct chg_drv *chg_drv , int soc)
{
	const bool freeze = soc >= 0;
	int rc;

	rc = GPSY_SET_PROP(chg_drv->bat_psy, POWER_SUPPLY_PROP_CAPACITY, soc);

	pr_debug("MSC_BD soc=%d %s (%d)\n", soc,
		 soc == -1 ? "THAW" : "FREEZE", rc);

	chg_drv->freeze_soc = freeze;
	return 0;
}

static int bd_batt_set_overheat(struct chg_drv *chg_drv , bool hot)
{
	const int health = hot ? POWER_SUPPLY_HEALTH_OVERHEAT :
				 POWER_SUPPLY_HEALTH_UNKNOWN;
	int ret;

	ret = GPSY_SET_PROP(chg_drv->bat_psy, POWER_SUPPLY_PROP_HEALTH, health);
	if (ret == 0)
		chg_drv->overheat = hot;

	pr_debug("MSC_BD OVERHEAT hot=%d (%d)\n", hot, ret);

	return ret;
}

static int bd_batt_set_state(struct chg_drv *chg_drv, bool hot, int soc)
{
	const bool lock_soc = false; /* b/173141442 */
	const bool freeze = soc != -1;
	int ret = 0; /* LOOK! */

	/*
	 * OVERHEAT changes handling of writes to POWER_SUPPLY_PROP_CAPACITY.
	 * Locking/Unlocking SOC in OVERHEAT causes google_battery to adjust
	 * the curves for the BD logic (if needed).
	 */
	if (hot && chg_drv->overheat != hot) {

		ret = bd_batt_set_overheat(chg_drv, hot);
		if (ret == 0 && freeze != chg_drv->freeze_soc && lock_soc)
			ret = bd_batt_set_soc(chg_drv, soc);

	} else if (!hot && chg_drv->overheat != hot) {

		if (freeze != chg_drv->freeze_soc && lock_soc)
			ret = bd_batt_set_soc(chg_drv, soc);
		if (ret == 0)
			ret = bd_batt_set_overheat(chg_drv, hot);
	}

	return ret;
}

static int chg_work_read_soc(struct power_supply *bat_psy, int *soc)
{
	union power_supply_propval val;
	int ret = 0;

	ret = power_supply_get_property(bat_psy, POWER_SUPPLY_PROP_CAPACITY,
					&val);
	if (ret == 0)
		*soc = val.intval;

	return ret;
}

/*
 * Run in background after disconnect to reset the trigger.
 * The UI% is not frozen here: only battery health state (might) remain set to
 * POWER_SUPPLY_HEALTH_OVERHEAT until the condition clears.
 *
 * NOTE: this is only used to clear POWER_SUPPLY_HEALTH_OVERHEAT after
 * disconnect and (possibly to) adjust the UI.
 * NOTE: Not needed when we clear on disconnect e.g when configured as such:
 *	bd_state->bd_resume_soc == 0 && bd_state->bd_resume_time == 0
 *
 */
static void bd_work(struct work_struct *work)
{
	struct chg_drv *chg_drv =
		container_of(work, struct chg_drv, bd_work.work);
	struct bd_data *bd_state = &chg_drv->bd_state;
	const ktime_t now = get_boot_sec();
	const long long delta_time = now - bd_state->disconnect_time;
	int interval_ms = CHG_WORK_BD_TRIGGERED_MS;
	int ret, soc = -1;

	__pm_stay_awake(chg_drv->bd_ws);

	mutex_lock(&chg_drv->bd_lock);

	pr_debug("MSC_BD_WORK: triggered=%d dsc_time=%lld delta=%lld\n",
		bd_state->triggered, bd_state->disconnect_time, delta_time);

	/* stop running if battery defender or retail mode are triggered */
	if (!bd_state->triggered || !bd_state->disconnect_time)
		goto bd_done;

	ret = chg_work_read_soc(chg_drv->bat_psy, &soc);
	if (ret < 0) {
		pr_err("MSC_BD_WORK: error reading soc (%d)\n", ret);
		interval_ms = 1000;
		goto bd_rerun;
	}

	/* soc after disconnect (SSOC must not be locked) */
	if (bd_state->bd_resume_soc &&
	    soc < bd_state->bd_resume_soc) {
		gbms_logbuffer_prlog(bd_state->bd_log, LOGLEVEL_INFO, 0, LOGLEVEL_INFO,
			"MSC_BD_WORK: done soc=%d limit=%d", soc, bd_state->bd_resume_soc);
		bd_resume(chg_drv);
		goto bd_rerun;
	}

	/* set on time and temperature, reset on abs temperature */
	ret = bd_update_stats(chg_drv);
	if (ret < 0) {
		pr_err("MSC_BD_WORK: update stats: %d\n", ret);
		interval_ms = 1000;
		goto bd_rerun;
	}
	bd_fan_vote(chg_drv, bd_state->triggered, FAN_LVL_HIGH);

	/* reset on time since disconnect & optional temperature reading */
	if (bd_state->bd_resume_time && delta_time > bd_state->bd_resume_time) {
		const int temp = bd_state->last_temp; /* or use avg */
		int triggered;

		/* single reading < of resume temp, or total average temp */
		triggered = bd_state->bd_resume_temp &&
			    temp > bd_state->bd_resume_temp;
		if (!triggered) {
			gbms_logbuffer_prlog(bd_state->bd_log, LOGLEVEL_INFO, 0, LOGLEVEL_INFO,
				"MSC_BD_WORK: done time=%lld limit=%d, temp=%d limit=%d",
				delta_time, bd_state->bd_resume_time,
				temp, bd_state->bd_resume_temp);
			bd_resume(chg_drv);
			goto bd_rerun;
		}
	}

	pr_debug("MSC_BD_WORK: trig=%d soc=%d time=%lld limit=%d temp=%d limit=%d avg=%lld\n",
		bd_state->triggered,
		soc,
		delta_time, bd_state->bd_resume_time,
		bd_state->last_temp, bd_state->bd_resume_abs_temp,
		div64_s64(bd_state->temp_sum, bd_state->time_sum));

bd_rerun:
	if (!bd_state->triggered) {
		/* disable the overheat flag, race with DWELL-DEFEND */
		bd_batt_set_overheat(chg_drv, false);
		chg_update_charging_state(chg_drv, false, false);
	} else {
		schedule_delayed_work(&chg_drv->bd_work,
				      msecs_to_jiffies(interval_ms));
	}

bd_done:
	mutex_unlock(&chg_drv->bd_lock);

	__pm_relax(chg_drv->bd_ws);
}

static void chg_stop_bd_work(struct chg_drv *chg_drv)
{
	/* stop bd_work() when usb is back */
	mutex_lock(&chg_drv->bd_lock);
	if (chg_drv->bd_state.disconnect_time) {
		chg_drv->bd_state.disconnect_time = 0;

		/* might have never been scheduled */
		cancel_delayed_work(&chg_drv->bd_work);
	}
	mutex_unlock(&chg_drv->bd_lock);
}

/*
 * If triggered, clear overheat and thaw soc on disconnect.
 * NOTE: Do not clear the defender state: will re-evaluate on next connect.
 * NOTE: bd_batt_set_state() needs to come before the chg disable
 */
static int chg_start_bd_work(struct chg_drv *chg_drv)
{
	struct bd_data *bd_state = &chg_drv->bd_state;
	const bool bd_ena = bd_state->bd_resume_soc || bd_state->bd_resume_time;

	mutex_lock(&chg_drv->bd_lock);

	/* always track disconnect time */
	if (!bd_state->disconnect_time)
		bd_state->disconnect_time = get_boot_sec();

	/* caller might reset bd_state */
	if (!bd_state->triggered || !bd_ena) {
		mutex_unlock(&chg_drv->bd_lock);
		return 0;
	}

	/* bd_work will keep track of time */
	mod_delayed_work(system_wq, &chg_drv->bd_work, 0);
	mutex_unlock(&chg_drv->bd_lock);

	return 0;
}

/* dock_defend */
static void bd_dd_init(struct chg_drv *chg_drv)
{
	struct bd_data *bd_state = &chg_drv->bd_state;
	int ret;

	ret = of_property_read_u32(chg_drv->device->of_node, "google,dd-charge-stop-level",
				   &bd_state->dd_charge_stop_level);
	if (ret < 0)
		bd_state->dd_charge_stop_level = DEFAULT_CHARGE_STOP_LEVEL;

	ret = of_property_read_u32(chg_drv->device->of_node, "google,dd-charge-start-level",
				   &bd_state->dd_charge_start_level);
	if (ret < 0)
		bd_state->dd_charge_start_level = DEFAULT_CHARGE_START_LEVEL;

	ret = of_property_read_s32(chg_drv->device->of_node, "google,dd-state",
				   &bd_state->dd_state);
	if (ret < 0)
		bd_state->dd_state = DOCK_DEFEND_DISABLED;

	ret = of_property_read_s32(chg_drv->device->of_node, "google,dd-settings",
				   &bd_state->dd_settings);
	if (ret < 0)
		bd_state->dd_settings = DOCK_DEFEND_USER_DISABLED;

	ret = of_property_read_u32(chg_drv->device->of_node, "google,dd-trigger-time",
				   &bd_state->dd_trigger_time);
	if (ret < 0)
		bd_state->dd_trigger_time = 0;

	gbms_logbuffer_prlog(bd_state->bd_log, LOGLEVEL_INFO, 0, LOGLEVEL_INFO,
		"MSC_BD: dock_defend stop_level=%d start_level=%d state=%d settings=%d time=%d",
		bd_state->dd_charge_stop_level, bd_state->dd_charge_start_level,
		bd_state->dd_state, bd_state->dd_settings, bd_state->dd_trigger_time);
}

static int bd_dd_state_update(const int dd_state, const bool dd_triggered, const bool change)
{
	int new_state = dd_state;

	switch (new_state) {
	case DOCK_DEFEND_ENABLED:
		if (dd_triggered && change)
			new_state = DOCK_DEFEND_ACTIVE;
		break;
	case DOCK_DEFEND_ACTIVE:
		if (!dd_triggered)
			new_state = DOCK_DEFEND_ENABLED;
		break;
	default:
		break;
	}

	return new_state;
}

static void bd_dd_set_enabled(struct chg_drv *chg_drv, const int ext_present, const int ext_online)
{
	struct bd_data *bd_state = &chg_drv->bd_state;

	if (!ext_present) {
		bd_state->dd_enabled = 0;
	} else if (ext_present && ext_online && !bd_state->dd_enabled) {
		const ktime_t now = get_boot_sec();
		ktime_t time;

		/* dd_last_update will be cleared in bd_reset() */
		if (bd_state->dd_last_update == 0)
			bd_state->dd_last_update = now;

		time = now - bd_state->dd_last_update;
		if (bd_state->dd_trigger_time && time >= bd_state->dd_trigger_time) {
			bd_state->dd_enabled = 1;
			bd_state->lowerbd_reached = true;
		}
	}
}

#define dd_is_enabled(bd_state) \
	((bd_state)->dd_state != DOCK_DEFEND_DISABLED && \
	(bd_state)->dd_settings == DOCK_DEFEND_USER_ENABLED)
static void bd_dd_run_defender(struct chg_drv *chg_drv, int soc, int *disable_charging, int *disable_pwrsrc)
{
	struct bd_data *bd_state = &chg_drv->bd_state;
	const bool was_triggered = bd_state->dd_triggered;
	const int upperbd = bd_state->dd_charge_stop_level;
	const int lowerbd = bd_state->dd_charge_start_level;

	bd_state->dd_triggered = dd_is_enabled(bd_state) ?
				 chg_is_custom_enabled(upperbd, lowerbd) : false;

	if (bd_state->dd_triggered)
		*disable_charging = bd_recharge_logic(bd_state, soc);
	if (*disable_charging)
		*disable_pwrsrc = soc > bd_state->dd_charge_stop_level;

	/* update dd_state to user space */
	bd_state->dd_state = bd_dd_state_update(bd_state->dd_state,
						bd_state->dd_triggered,
						(soc >= upperbd));

	/* Start DD stats */
	dd_stats_update(chg_drv);

	/* need icl_ramp_work when disable_pwrsrc 1 -> 0 */
	if (!*disable_pwrsrc && chg_drv->disable_pwrsrc) {
		struct power_supply *dc_psy;

		dc_psy = power_supply_get_by_name("dc");
		if (dc_psy)
			power_supply_changed(dc_psy);
	}

	if (bd_state->dd_triggered != was_triggered)
		gbms_logbuffer_prlog(bd_state->bd_log, LOGLEVEL_INFO, 0, LOGLEVEL_INFO,
			"MSC_BD dd_triggered %d->%d", was_triggered, bd_state->dd_triggered);
}

static int chg_run_defender(struct chg_drv *chg_drv)
{
	const int upperbd = chg_drv->charge_stop_level;
	const int lowerbd = chg_drv->charge_start_level;
	int soc, disable_charging = 0, disable_pwrsrc = 0;
	struct power_supply *bat_psy = chg_drv->bat_psy;
	struct bd_data *bd_state = &chg_drv->bd_state;
	int bd_fan_level = FAN_LVL_UNKNOWN;
	int rc, ret;

	/*
	 * retry for max CHG_DRV_EGAIN_RETRIES * CHG_WORK_ERROR_RETRY_MS on
	 * -EGAIN (race on wake). Retry is silent until we exceed the
	 * threshold, ->egain_retries is reset on every wakeup.
	 * NOTE: -EGAIN after this should be flagged
	 */
	ret = chg_work_read_soc(bat_psy, &soc);
	if (ret == -EAGAIN) {
		chg_drv->egain_retries += 1;
		if (chg_drv->egain_retries < CHG_DRV_EAGAIN_RETRIES)
			return ret;
	}

	/* update_interval = -1, will reschedule */
	if (ret < 0)
		return ret;

	chg_drv->egain_retries = 0;

	mutex_lock(&chg_drv->bd_lock);

	/* DWELL-DEFEND and Retail case */
	disable_charging = chg_work_is_charging_disabled(chg_drv, soc);
	if (disable_charging && soc > chg_drv->charge_stop_level)
		disable_pwrsrc = 1;

	if (chg_is_custom_enabled(upperbd, lowerbd)) {
		/*
		 * This mode can be enabled from DWELL-DEFEND when in "idle",
		 * while TEMP-DEFEND is triggered or from Retail Mode.
		 * Clear the TEMP-DEFEND status (thaw SOC and clear HEALTH) and
		 * have usespace to report HEALTH status.
		 * NOTE: if here, bd_work() was already stopped.
		 */
		if (chg_drv->bd_state.triggered) {
			rc = bd_batt_set_state(chg_drv, false, -1);
			if (rc < 0)
				pr_err("MSC_BD resume (%d)\n", rc);

			bd_resume(chg_drv);
		}

		/* force TEMP-DEFEND off */
		chg_drv->bd_state.enabled = 0;

		/* set dd_state to inactive state (DOCK_DEFEND_ENABLED) */
		if (chg_drv->bd_state.dd_enabled)
			chg_drv->bd_state.dd_state = bd_dd_state_update(chg_drv->bd_state.dd_state,
									false, false);

	} else if (chg_drv->bd_state.enabled) {
		const bool was_triggered = bd_state->triggered;

		/* bd_work() is not running here */
		rc = bd_update_stats(chg_drv);
		if (rc < 0)
			pr_debug("MSC_DB BD update stats: %d\n", rc);

		bd_fan_level = bd_fan_calculate_level(bd_state);

		/* thaw SOC, clear overheat */
		if (!bd_state->triggered && was_triggered) {
			rc = bd_batt_set_state(chg_drv, false, -1);
			gbms_logbuffer_prlog(bd_state->bd_log, LOGLEVEL_INFO, 0, LOGLEVEL_INFO,
					     "MSC_BD resume (%d)", rc);
		} else if (bd_state->triggered) {
			const int lock_soc = soc; /* or set to 100 */

			/* recharge logic */
			if (bd_state->bd_drainto_soc) {
				disable_charging =
					bd_recharge_logic(bd_state, soc);
				if (disable_charging)
					disable_pwrsrc =
						soc > bd_state->bd_drainto_soc;
			} else {
				disable_charging = bd_recharge_logic(bd_state,
							bd_state->last_voltage);
				if (disable_charging)
					disable_pwrsrc =
						bd_state->last_voltage >
						bd_state->bd_trigger_voltage;
			}

			/* overheat and freeze, on trigger and reconnect */
			if (!was_triggered || chg_drv->stop_charging) {
				rc = bd_batt_set_state(chg_drv, true, lock_soc);

				gbms_logbuffer_prlog(bd_state->bd_log,
					LOGLEVEL_INFO, 0, LOGLEVEL_INFO,
					"MSC_BD triggered was=%d stop=%d lock_soc=%d",
					was_triggered, chg_drv->stop_charging, lock_soc);
			}

			/* set dd_state to inactive state (DOCK_DEFEND_ENABLED) */
			if (bd_state->dd_enabled)
				bd_state->dd_state = bd_dd_state_update(bd_state->dd_state,
									false, false);
		}
		/* run dock_defend */
		if (!bd_state->triggered && bd_state->dd_enabled)
			bd_dd_run_defender(chg_drv, soc, &disable_charging, &disable_pwrsrc);

		if (chg_drv->debug_input_suspend)
			bd_reset(&chg_drv->bd_state);
	} else if (chg_drv->bd_state.dd_enabled) {
		bd_dd_run_defender(chg_drv, soc, &disable_charging, &disable_pwrsrc);
	} else if (chg_drv->disable_charging) {
		/*
		 * chg_drv->disable_charging && !disable_charging
		 * re-enable TEMP-DEFEND
		 */
		bd_reset(&chg_drv->bd_state);

		/* DWELL-DEFEND handles OVERHEAT status */
	}

	bd_fan_vote(chg_drv, bd_fan_level != FAN_LVL_UNKNOWN, bd_fan_level);

	/* state in chg_drv->disable_charging, chg_drv->disable_pwrsrc */
	chg_update_charging_state(chg_drv, disable_charging, disable_pwrsrc);
	mutex_unlock(&chg_drv->bd_lock);

	return 0;
}

/* ------------------------------------------------------------------------ */

/*
 * Note: Some adapters have several PPS profiles providing different voltage
 * ranges and different maximal currents. If the device demands more power from
 * the adapter but reached the maximum it can get in the current profile,
 * search if there exists another profile providing more power. If it demands
 * less power, search if there exists another profile providing enough power
 * with higher current.
 *
 * return negative on errors or no suitable profile
 * return 0 on successful profile switch
 */
int chg_switch_profile(struct pd_pps_data *pps, struct power_supply *tcpm_psy,
		       bool more_pwr)
{
	u32 max_mv, max_ma, max_mw;
	u32 current_mw, current_ma;
	int i, ret = -ENODATA;
	u32 pdo;

	if (!pps || !tcpm_psy || pps->nr_src_cap < 2)
		return -EINVAL;

	current_ma = pps->op_ua / 1000;
	current_mw = (pps->out_uv / 1000) * current_ma / 1000;

	for (i = 1; i < pps->nr_src_cap; i++) {
		pdo = pps->src_caps[i];

		if (pdo_type(pdo) != PDO_TYPE_APDO)
			continue;

		/* TODO: use pps_data sink capabilities */
		max_mv = min_t(u32, PD_SNK_MAX_MV,
			       pdo_pps_apdo_max_voltage(pdo));
		/* TODO: use pps_data sink capabilities */
		max_ma = min_t(u32, PD_SNK_MAX_MA,
			       pdo_pps_apdo_max_current(pdo));
		max_mw = max_mv * max_ma / 1000;

		if (more_pwr && max_mw > current_mw) {
			/* export maximal capability, TODO: use sink cap */
			pdo = PDO_PPS_APDO(PD_SNK_MIN_MV,
					   PD_SNK_MAX_MV,
					   PD_SNK_MAX_MA);
			ret = chg_update_capability(tcpm_psy, PDO_PPS, pdo);
			if (ret < 0)
				pps_log(pps, "Failed to update sink caps, ret %d",
					ret);
			break;
		} else if (!more_pwr && max_mw >= current_mw &&
			   max_ma > current_ma) {

			/* TODO: tune the max_mv, fix this */
			pdo = PDO_PPS_APDO(PD_SNK_MIN_MV, 6000, PD_SNK_MAX_MA);
			ret = chg_update_capability(tcpm_psy, PDO_PPS, pdo);
			if (ret < 0)
				pps_log(pps, "Failed to update sink caps, ret %d",
					ret);
			break;
		}
	}

	if (ret == 0) {
		pps->keep_alive_cnt = 0;
		pps->stage = PPS_NONE;
	}

	return ret;
}


static void chg_update_csi(struct chg_drv *chg_drv)
{
	const bool is_policy = chg_drv->charging_policy == CHARGING_POLICY_VOTE_LONGLIFE;
	const bool is_dwell = !is_policy &&
			     chg_is_custom_enabled(chg_drv->charge_stop_level,
						   chg_drv->charge_start_level);
	const bool is_disconnected = chg_state_is_disconnected(&chg_drv->chg_state);
	const bool is_full = (chg_drv->chg_state.f.flags & GBMS_CS_FLAG_DONE) != 0;
	const bool is_dock = chg_drv->bd_state.dd_state == DOCK_DEFEND_ACTIVE;
	const bool is_temp = chg_drv->bd_state.triggered;

	if (!chg_drv->csi_status_votable)
		chg_drv->csi_status_votable =
			gvotable_election_get_handle(VOTABLE_CSI_STATUS);

	if (!chg_drv->csi_type_votable)
		chg_drv->csi_type_votable =
			gvotable_election_get_handle(VOTABLE_CSI_TYPE);

	if (!chg_drv->csi_status_votable || !chg_drv->csi_type_votable)
		return;

	/* full is set only on charger */
	gvotable_cast_long_vote(chg_drv->csi_status_votable, "CSI_STATUS_FULL",
				CSI_STATUS_UNKNOWN,
				!is_disconnected && is_full);

	/* Charging Status Defender_Dock */
	gvotable_cast_long_vote(chg_drv->csi_status_votable, "CSI_STATUS_DEFEND_DOCK",
				CSI_STATUS_Defender_Dock,
				is_dock);

	/* Battery defenders (but also retail mode) */
	gvotable_cast_long_vote(chg_drv->csi_status_votable, "CSI_STATUS_DEFEND_TEMP",
				CSI_STATUS_Defender_Temp,
				is_temp);
	gvotable_cast_long_vote(chg_drv->csi_status_votable, "CSI_STATUS_DEFEND_DWELL",
				CSI_STATUS_Defender_Dwell,
				is_dwell);

	/* Longlife is set on TEMP, DWELL and TRICKLE */
	gvotable_cast_long_vote(chg_drv->csi_type_votable, "CSI_TYPE_DEFEND",
				CSI_TYPE_LongLife,
				is_temp || is_dwell || is_dock || is_policy);

	/* Set to normal if the device docked */
	if (is_dock)
		gvotable_cast_long_vote(chg_drv->csi_type_votable, "CSI_TYPE_CONNECTED",
					CSI_TYPE_Normal, true);

	/* Charging Status Normal */
}

/* ------------------------------------------------------------------------ */

/* No op on battery not present */
static void chg_work(struct work_struct *work)
{
	struct chg_drv *chg_drv =
		container_of(work, struct chg_drv, chg_work.work);
	struct power_supply *bat_psy = chg_drv->bat_psy;
	struct power_supply *wlc_psy = chg_drv->wlc_psy;
	struct power_supply *ext_psy = chg_drv->ext_psy;
	struct power_supply *usb_psy = chg_drv->tcpm_psy ? chg_drv->tcpm_psy :
				       chg_drv->usb_psy;
	union gbms_ce_adapter_details ad = chg_drv->adapter_details;
	int wlc_online = 0, wlc_present = 0;
	int ext_online = 0, ext_present = 0;
	int usb_online, usb_present = 0;
	int present, online;
	int soc = -1, update_interval = -1;
	bool chg_done = false, online_changed = false;
	int success, rc = 0;
	int force_5v;

	__pm_stay_awake(chg_drv->chg_ws);

	if (!chg_drv->init_done) {
		pr_debug("battery charging work item, init pending\n");
		goto exit_skip;
	}

	pr_debug("battery charging work item\n");

	if (!chg_drv->batt_present) {
		/* -EGAIN = NOT ready, <0 don't know yet */
		rc = GPSY_GET_PROP(bat_psy, POWER_SUPPLY_PROP_PRESENT);
		if (rc < 0)
			goto rerun_error;

		chg_drv->batt_present = (rc > 0);
		if (!chg_drv->batt_present)
			goto exit_chg_work;

		pr_info("MSC_CHG battery present\n");
	}

	if (chg_drv->dead_battery)
		chg_drv->dead_battery = chg_update_dead_battery(chg_drv);

	if (chg_drv->chg_mode == CHG_DRV_MODE_DISABLED)
		goto exit_skip;

	/* cause msc_update_charger_cb to ignore updates */
	gvotable_cast_int_vote(chg_drv->msc_interval_votable,
			       MSC_CHG_VOTER, 0, true);

	/* NOTE: return online when usb is not defined */
	usb_online = chg_usb_online(usb_psy);
	/* NOTE: ask usb for present (when/if defined) */
	if (chg_drv->usb_psy)
		usb_present = PSY_GET_PROP(chg_drv->usb_psy, POWER_SUPPLY_PROP_PRESENT);
	else
		usb_present = usb_online;

	if (wlc_psy) {
		wlc_online = PSY_GET_PROP(wlc_psy, POWER_SUPPLY_PROP_ONLINE);
		wlc_present = PSY_GET_PROP(wlc_psy, POWER_SUPPLY_PROP_PRESENT);
	}

	/* TODO: b/186905611 validate Defender with EXT */
	if (ext_psy) {
		ext_online = PSY_GET_PROP(ext_psy, POWER_SUPPLY_PROP_ONLINE);
		ext_present = PSY_GET_PROP(ext_psy, POWER_SUPPLY_PROP_PRESENT);

		/* set dd_enabled for dock_defend */
		bd_dd_set_enabled(chg_drv, ext_present, ext_online);
	}

	/* ICL=0 on discharge will (might) cause usb online to go to 0 */
	present = usb_present || wlc_present || ext_present;
	online = usb_online || wlc_online || ext_online;

	/* Logging */
	if (chg_drv->online != online || chg_drv->present != present) {
		struct bd_data *bd_state = &chg_drv->bd_state;

		gbms_logbuffer_devlog(bd_state->bd_log, chg_drv->device,
				      LOGLEVEL_INFO, 0, LOGLEVEL_INFO,
				      "online:%d->%d [USB:%d/WLC:%d/EXT:%d], "
				      "present:%d->%d [USB:%d/WLC:%d/EXT:%d] (stop_charging:%d)",
				      chg_drv->online, online, usb_online, wlc_online,  ext_online,
				      chg_drv->present, present, usb_present, wlc_present,
				      ext_present, chg_drv->stop_charging);
		chg_drv->online = online;
		chg_drv->present = present;
		online_changed = true;
	}

	if (usb_online  < 0 || wlc_online < 0 || ext_online < 0) {
		pr_err("MSC_CHG error reading usb=%d wlc=%d ext=%d\n",
		       usb_online, wlc_online, ext_online);

		/* TODO: maybe disable charging when this happens? */
		goto rerun_error;
	} else if (!online && !present) {
		const bool stop_charging = chg_drv->stop_charging != 1;
		const int upperbd = chg_drv->charge_stop_level;
		const int lowerbd = chg_drv->charge_start_level;

		/*
		 * Update DD stats last time if DD is active.
		 * NOTE: *** Ensure this is done before disconnect indication to google_battery
		 */
		dd_stats_update(chg_drv);

		/* reset dock_defend */
		if (chg_drv->bd_state.dd_triggered) {
			chg_update_charging_state(chg_drv, false, false);
			chg_drv->bd_state.dd_triggered = 0;
		}
		if (chg_drv->bd_state.dd_settings == DOCK_DEFEND_USER_CLEARED)
			chg_drv->bd_state.dd_settings = DOCK_DEFEND_USER_ENABLED;
		if (chg_drv->bd_state.dd_state == DOCK_DEFEND_ACTIVE)
			chg_drv->bd_state.dd_state = DOCK_DEFEND_ENABLED;

		rc = chg_start_bd_work(chg_drv);
		if (rc < 0)
			goto rerun_error;

		if (stop_charging || online_changed) {
			int ret;
			struct gvotable_election *hda_tz_votable =
						gvotable_election_get_handle(VOTABLE_HDA_TZ);

			ad.v = 0;
			pr_info("MSC_CHG no power source, disabling charging\n");

			ret = GPSY_SET_PROP(chg_drv->chg_psy, GBMS_PROP_CHARGING_ENABLED, 0);
			if (ret < 0) {
				pr_err("MSC_CHG failed to set CHARGING_ENABLED to 0 (%d)\n", ret);
				if (ret == -EAGAIN)
					goto rerun_error;
			}

			gvotable_cast_bool_vote(chg_drv->msc_chg_disable_votable,
						MSC_CHG_VOTER, true);

			gvotable_cast_int_vote(chg_drv->msc_last_votable, "BATT", 0, 1);

			if (hda_tz_votable)
				gvotable_cast_int_vote(hda_tz_votable,
						       MSC_HDA_VOTER, HDA_TZ_NONE, false);

			if (!chg_drv->bd_state.triggered) {
				mutex_lock(&chg_drv->bd_lock);
				bd_reset(&chg_drv->bd_state);
				mutex_unlock(&chg_drv->bd_lock);
				bd_fan_vote(chg_drv, false, FAN_LVL_NOT_CARE);
			}

			rc = chg_reset_state(chg_drv);
			if (rc == -EAGAIN)
				schedule_delayed_work(&chg_drv->chg_work,
						      msecs_to_jiffies(100));
			else
				chg_drv->stop_charging = 1;
		}

		if (chg_is_custom_enabled(upperbd, lowerbd) && chg_drv->disable_pwrsrc)
			chg_run_defender(chg_drv);

		/* allow sleep (if disconnected) while draining */
		if (chg_drv->disable_pwrsrc)
			__pm_relax(chg_drv->bd_ws);

		goto exit_chg_work;
	} else {
		/* Run thermal stats when connected to power (preset || online) */
		thermal_stats_update(chg_drv);

		if (chg_drv->stop_charging != 0 && present) {
			const bool restore_fcc =
				chg_drv->therm_wlc_override_fcc;

			/* Stop BD work after disconnect */
			chg_stop_bd_work(chg_drv);

			/* will re-enable charging after setting FCC,CC_MAX */
			if (restore_fcc)
				(void)chg_therm_update_fcc(chg_drv);
	 	}
	}

	/* updates chg_drv->disable_pwrsrc and chg_drv->disable_charging */
	rc = chg_run_defender(chg_drv);
	if (rc == -EAGAIN)
		goto rerun_error;
	if (rc < 0) {
		pr_err("MSC_BD cannot run defender (%d)\n", rc);
		goto update_charger;
	}

	/* device might fall off the charger when disable_pwrsrc is set. */
	if (!chg_drv->disable_pwrsrc)
		chg_work_adapter_details(&ad, usb_online, wlc_online,
					 ext_online, chg_drv);

	rc = chg_work_roundtrip(chg_drv, &chg_drv->chg_state);
	if (rc == -EAGAIN)
		goto rerun_error;

	update_interval = rc;
	if (update_interval >= 0) {
		chg_done = (chg_drv->chg_state.f.flags &
			    GBMS_CS_FLAG_DONE) != 0;
		/* clear rc for exit_chg_work: update correct data */
		rc = 0;
	}

	/* Book dd stats to correct charging type if DD active */
	dd_stats_update(chg_drv);

	/*
	 * chg_drv->disable_pwrsrc -> chg_drv->disable_charging
	 * update_interval = 0 will reschedule if TEMP-DEFEND is enabled
	 */
	if (chg_drv->disable_charging)
		update_interval = 0;

	/* update_interval=0 when disconnected or on EOC (check for races)
	 * update_interval=-1 on an error (from roundtrip or reading soc)
	 * NOTE: might have cc_max==0 from the roundtrip on JEITA
	 */
update_charger:
	pr_debug("MSC_CHG disable_charging=%d, update_interval=%d\n",
		 chg_drv->disable_charging, update_interval);

	if (!chg_drv->disable_charging && update_interval > 0) {

		/* msc_update_charger_cb will write to charger and reschedule */
		gvotable_cast_int_vote(chg_drv->msc_interval_votable,
				       MSC_CHG_VOTER, update_interval, true);

		/* chg_drv->stop_charging set on disconnect, reset on connect */
		if (chg_drv->stop_charging != 0) {
			int ret;

			pr_info("MSC_CHG power source usb=%d wlc=%d, ext=%d enabling charging\n",
				usb_online, wlc_online, ext_online);

			ret = GPSY_SET_PROP(chg_drv->chg_psy, GBMS_PROP_CHARGING_ENABLED, 1);
			if (ret < 0) {
				pr_err("MSC_CHG failed to set CHARGING_ENABLED to 1 (%d)\n", ret);
				if (ret == -EAGAIN)
					goto rerun_error;
			}

			gvotable_cast_bool_vote(chg_drv->msc_chg_disable_votable,
						MSC_CHG_VOTER, false);
			chg_drv->stop_charging = 0;
		}
	} else {
		int res;

		/* needs to disable_charging, will reschedule */
		res = chg_update_charger(chg_drv, chg_drv->fv_uv, 0, chg_drv->topoff);
		if (res < 0 && res != -EAGAIN)
			pr_err("MSC_CHG cannot update charger (%d)\n", res);

		pr_debug("MSC_CHG charging disabled res=%d rc=%d ui=%d\n",
			 res, rc, update_interval);

		if (res < 0 || rc < 0 || update_interval < 0)
			goto rerun_error;
	}

	rc = chg_work_read_soc(bat_psy, &soc);
	if (rc < 0)
		pr_err("MSC_CHG error reading soc (%d)\n", rc);
	if (soc != 100)
		chg_done = false;

	force_5v = gvotable_get_current_int_vote(chg_drv->force_5v_votable);

	if (chg_done)
		pr_debug("MSC_CHG charge full force 5V: %d\n", force_5v);

	/* tied to the charger: could tie to battery @ 100% instead */
	if (!chg_drv->chg_term.usb_5v && chg_done && usb_pd_is_high_volt(&ad) && force_5v) {
		pr_info("MSC_CHG switch to 5V on full\n");
		chg_update_capability(chg_drv->tcpm_psy, PDO_FIXED_5V, 0);
		chg_drv->chg_term.usb_5v = 1;
	} else if (((chg_drv->pps_data.stage == PPS_ACTIVE) || !force_5v) && chg_done) {
		pr_info("MSC_CHG switch to Fixed Profile on full\n");
		chg_drv->pps_data.stage = PPS_DISABLED;
		chg_update_capability(chg_drv->tcpm_psy, PDO_FIXED_HIGH_VOLTAGE,
				      0);
	}

	/* WAR: battery overcharge on a weak adapter */
	if (chg_drv->chg_term.enable && chg_done)
		chg_eval_chg_termination(&chg_drv->chg_term);

	/* BD needs to keep checking the temperature after EOC */
	if (chg_drv->bd_state.enabled) {
		unsigned long jif = msecs_to_jiffies(CHG_WORK_BD_TRIGGERED_MS);

		pr_debug("MSC_BD reschedule in %d ms\n", CHG_WORK_BD_TRIGGERED_MS);
		schedule_delayed_work(&chg_drv->chg_work, jif);
	}

	goto exit_chg_work;

rerun_error:
	success = schedule_delayed_work(&chg_drv->chg_work,
				msecs_to_jiffies(CHG_WORK_ERROR_RETRY_MS));

	/*
	 * no need to reschedule the pending after an error
	 * NOTE: rc is the return code from battery properties
	 */
	if (rc != -EAGAIN)
		pr_err("MSC_CHG error rerun=%d in %d ms (%d)\n",
			success, CHG_WORK_ERROR_RETRY_MS, rc);

	/*
	 * If stay_awake is false, we are safe to ping the adapter
	 * NOTE: this probably needs to be changed to pps_keep_alive()
	 */
	if (!chg_drv->pps_data.stay_awake &&
	    chg_drv->pps_data.stage == PPS_ACTIVE)
		pps_ping(&chg_drv->pps_data, chg_drv->tcpm_psy);

	pr_debug("chg_work reschedule\n");
	return;

exit_chg_work:
	/* Route adapter details after the roundtrip since google_battery
	 * might overwrite the value when it starts a new cycle.
	 * NOTE: chg_reset_state() must not set chg_drv->adapter_details.v
	 * to zero. Fix the odd dependency when handling failure in setting
	 * GBMS_PROP_ADAPTER_DETAILS.
	 */
	if (rc == 0 && ad.v != chg_drv->adapter_details.v) {

		rc = GPSY_SET_PROP(chg_drv->bat_psy,
				   GBMS_PROP_ADAPTER_DETAILS,
				   (int)ad.v);

		/* TODO: handle failure rescheduling chg_work */
		if (rc < 0)
			pr_err("MSC_CHG no adapter details (%d)\n", rc);
		else
			chg_drv->adapter_details.v = ad.v;
	}

	chg_update_csi(chg_drv);

exit_skip:
	pr_debug("chg_work done\n");
	__pm_relax(chg_drv->chg_ws);
}

/* ------------------------------------------------------------------------ */

/* return negative when using ng charging */
static int chg_init_chg_profile(struct chg_drv *chg_drv)
{
	struct device *dev = chg_drv->device;
	struct device_node *node = dev->of_node;
	u32 temp;
	int ret;

	/* chg_work will use the minimum between all votess */
	ret = of_property_read_u32(node, "google,cv-update-interval",
				   &chg_drv->cv_update_interval);
	if (ret < 0 || chg_drv->cv_update_interval == 0)
		chg_drv->cv_update_interval = DRV_DEFAULTCV_UPDATE_INTERVAL;

	ret = of_property_read_u32(node, "google,cc-update-interval",
				   &chg_drv->cc_update_interval);
	if (ret < 0 || chg_drv->cc_update_interval == 0)
		chg_drv->cc_update_interval = DRV_DEFAULTCC_UPDATE_INTERVAL;

	/* when set will reduce cc_max by
	 * 	cc_max = cc_max * (1000 - chg_cc_tolerance) / 1000;
	 *
	 * this adds a "safety" margin for C rates if the charger doesn't do it.
	 */
	ret = of_property_read_u32(node, "google,chg-cc-tolerance", &temp);
	if (ret < 0)
		chg_drv->chg_cc_tolerance = 0;
	else if (temp > CHG_DRV_CC_HW_TOLERANCE_MAX)
		chg_drv->chg_cc_tolerance = CHG_DRV_CC_HW_TOLERANCE_MAX;
	else
		chg_drv->chg_cc_tolerance = temp;

	/* max charging current. This will be programmed to the charger when
	 * there is no charge table.
	 */
	ret = of_property_read_u32(node, "google,fcc-max-ua", &temp);
	if (ret < 0)
		chg_drv->batt_profile_fcc_ua = -EINVAL;
	else
		chg_drv->batt_profile_fcc_ua = temp;

	/* max and default charging voltage: this is what will be programmed to
	 * the charger when fv_uv is invalid.
	 */
	ret = of_property_read_u32(node, "google,fv-max-uv", &temp);
	if (ret < 0)
		chg_drv->batt_profile_fv_uv = -EINVAL;
	else
		chg_drv->batt_profile_fv_uv = temp;

	chg_drv->chg_term.enable =
		of_property_read_bool(node, "google,chg-termination-enable");

	/* fallback to 5V on charge termination */
	chg_drv->chg_term.usb_5v =
		of_property_read_bool(node, "google,chg-termination-5v");
	if (!chg_drv->chg_term.usb_5v) {
		chg_drv->chg_term.usb_5v = -1;
	} else {
		pr_info("renegotiate on full\n");
		chg_drv->chg_term.usb_5v = 0;
	}

	chg_drv->taper_last_tier = of_property_read_bool(node, "google,chg-taper-last-tier");
	if (chg_drv->taper_last_tier)
		pr_info("taper on last tier entry\n");

	pr_info("charging profile in the battery\n");
	return 0;
}

static ssize_t show_charge_stop_level(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct chg_drv *chg_drv = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", chg_drv->charge_stop_level);
}

static ssize_t set_charge_stop_level(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct chg_drv *chg_drv = dev_get_drvdata(dev);
	int ret = 0, val;

	ret = kstrtoint(buf, 0, &val);
	if (ret < 0)
		return ret;

	if (!chg_drv->bat_psy) {
		pr_err("chg_drv->bat_psy is not ready");
		return -ENODATA;
	}

	if ((val == chg_drv->charge_stop_level) ||
	    (val <= chg_drv->charge_start_level) ||
	    (val > DEFAULT_CHARGE_STOP_LEVEL))
		return -EINVAL;

	pr_info("%s: %d -> %d\n", __func__, chg_drv->charge_stop_level, val);
	chg_drv->charge_stop_level = val;

	/* Force update charging state vote */
	chg_run_defender(chg_drv);

	if (chg_drv->bat_psy)
		power_supply_changed(chg_drv->bat_psy);

	return count;
}

static DEVICE_ATTR(charge_stop_level, 0660, show_charge_stop_level,
					    set_charge_stop_level);

static ssize_t
show_charge_start_level(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct chg_drv *chg_drv = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", chg_drv->charge_start_level);
}

static ssize_t set_charge_start_level(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct chg_drv *chg_drv = dev_get_drvdata(dev);
	int ret = 0, val;

	ret = kstrtoint(buf, 0, &val);
	if (ret < 0)
		return ret;

	if (!chg_drv->bat_psy) {
		pr_err("chg_drv->bat_psy is not ready");
		return -ENODATA;
	}

	if ((val == chg_drv->charge_start_level) ||
	    (val >= chg_drv->charge_stop_level) ||
	    (val < DEFAULT_CHARGE_START_LEVEL))
		return -EINVAL;

	pr_info("%s: %d -> %d\n", __func__, chg_drv->charge_start_level, val);
	chg_drv->charge_start_level = val;

	/* Force update charging state vote */
	chg_run_defender(chg_drv);

	if (chg_drv->bat_psy)
		power_supply_changed(chg_drv->bat_psy);

	return count;
}

static DEVICE_ATTR(charge_start_level, 0660,
		   show_charge_start_level, set_charge_start_level);

static ssize_t
show_bd_temp_enable(struct device *dev,
		    struct device_attribute *attr, char *buf)
{
	struct chg_drv *chg_drv = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			chg_drv->bd_state.bd_temp_enable);
}

static ssize_t set_bd_temp_enable(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct chg_drv *chg_drv = dev_get_drvdata(dev);
	int ret = 0, val;

	ret = kstrtoint(buf, 0, &val);
	if (ret < 0)
		return ret;

	if (chg_drv->bd_state.bd_temp_enable == val)
		return count;

	mutex_lock(&chg_drv->bd_lock);

	chg_drv->bd_state.bd_temp_enable = val;

	bd_reset(&chg_drv->bd_state);

	mutex_unlock(&chg_drv->bd_lock);

	if (chg_drv->bat_psy)
		power_supply_changed(chg_drv->bat_psy);


	return count;
}
static DEVICE_ATTR(bd_temp_enable, 0660,
		   show_bd_temp_enable, set_bd_temp_enable);

static ssize_t
show_bd_trigger_voltage(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct chg_drv *chg_drv = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			chg_drv->bd_state.bd_trigger_voltage);
}

static ssize_t set_bd_trigger_voltage(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct chg_drv *chg_drv = dev_get_drvdata(dev);
	int ret = 0, val;

	ret = kstrtoint(buf, 0, &val);
	if (ret < 0)
		return ret;

	if (val > MAX_BD_VOLTAGE || val < MIN_BD_VOLTAGE)
		return -EINVAL;

	chg_drv->bd_state.bd_trigger_voltage = val;

	return count;
}

static DEVICE_ATTR(bd_trigger_voltage, 0660,
		   show_bd_trigger_voltage, set_bd_trigger_voltage);

static ssize_t
show_bd_drainto_soc(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct chg_drv *chg_drv = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 chg_drv->bd_state.bd_drainto_soc);
}

static ssize_t set_bd_drainto_soc(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct chg_drv *chg_drv = dev_get_drvdata(dev);
	int ret = 0, val;

	ret = kstrtoint(buf, 0, &val);
	if (ret < 0)
		return ret;

	if (val > MAX_BD_SOC || val < MIN_BD_SOC)
		return -EINVAL;

	chg_drv->bd_state.bd_drainto_soc = val;

	return count;
}

static DEVICE_ATTR(bd_drainto_soc, 0660,
		   show_bd_drainto_soc, set_bd_drainto_soc);

static ssize_t
show_bd_trigger_temp(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct chg_drv *chg_drv = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			chg_drv->bd_state.bd_trigger_temp);
}

static ssize_t set_bd_trigger_temp(struct device *dev, struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct chg_drv *chg_drv = dev_get_drvdata(dev);
	int ret = 0, val;

	ret = kstrtoint(buf, 0, &val);
	if (ret < 0)
		return ret;

	if (val > MAX_BD_TEMP || val < MIN_BD_TEMP)
		return -EINVAL;

	chg_drv->bd_state.bd_trigger_temp = val;

	return count;
}

static DEVICE_ATTR(bd_trigger_temp, 0660,
		   show_bd_trigger_temp, set_bd_trigger_temp);

static ssize_t
show_bd_trigger_time(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct chg_drv *chg_drv = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			chg_drv->bd_state.bd_trigger_time);
}

static ssize_t set_bd_trigger_time(struct device *dev, struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct chg_drv *chg_drv = dev_get_drvdata(dev);
	int ret = 0, val;

	ret = kstrtoint(buf, 0, &val);
	if (ret < 0)
		return ret;

	if (val < 0)
		return -EINVAL;

	chg_drv->bd_state.bd_trigger_time = val;

	return count;
}

static DEVICE_ATTR(bd_trigger_time, 0660,
		   show_bd_trigger_time, set_bd_trigger_time);

static ssize_t
show_bd_recharge_voltage(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct chg_drv *chg_drv = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			chg_drv->bd_state.bd_recharge_voltage);
}

static ssize_t set_bd_recharge_voltage(struct device *dev, struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct chg_drv *chg_drv = dev_get_drvdata(dev);
	int ret = 0, val;

	ret = kstrtoint(buf, 0, &val);
	if (ret < 0)
		return ret;

	if (val > MAX_BD_VOLTAGE || val < MIN_BD_VOLTAGE)
		return -EINVAL;

	chg_drv->bd_state.bd_recharge_voltage = val;

	return count;
}

static DEVICE_ATTR(bd_recharge_voltage, 0660,
		   show_bd_recharge_voltage, set_bd_recharge_voltage);

static ssize_t
show_bd_recharge_soc(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct chg_drv *chg_drv = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 chg_drv->bd_state.bd_recharge_soc);
}

static ssize_t set_bd_recharge_soc(struct device *dev, struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct chg_drv *chg_drv = dev_get_drvdata(dev);
	int ret = 0, val;

	ret = kstrtoint(buf, 0, &val);
	if (ret < 0)
		return ret;

	if (val > MAX_BD_SOC || val < MIN_BD_SOC)
		return -EINVAL;

	chg_drv->bd_state.bd_recharge_soc = val;

	return count;
}

static DEVICE_ATTR(bd_recharge_soc, 0660,
			show_bd_recharge_soc, set_bd_recharge_soc);

static ssize_t
show_bd_resume_abs_temp(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct chg_drv *chg_drv = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			chg_drv->bd_state.bd_resume_abs_temp);
}

static ssize_t set_bd_resume_abs_temp(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct chg_drv *chg_drv = dev_get_drvdata(dev);
	int ret = 0, val;

	ret = kstrtoint(buf, 0, &val);
	if (ret < 0)
		return ret;

	if (val > MAX_BD_TEMP || val < MIN_BD_TEMP)
		return -EINVAL;

	chg_drv->bd_state.bd_resume_abs_temp = val;

	return count;
}

static DEVICE_ATTR(bd_resume_abs_temp, 0660,
		   show_bd_resume_abs_temp, set_bd_resume_abs_temp);

static ssize_t
show_bd_resume_time(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct chg_drv *chg_drv = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			chg_drv->bd_state.bd_resume_time);
}

static ssize_t set_bd_resume_time(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct chg_drv *chg_drv = dev_get_drvdata(dev);
	int ret = 0, val;

	ret = kstrtoint(buf, 0, &val);
	if (ret < 0)
		return ret;

	if (val < 0)
		return -EINVAL;

	chg_drv->bd_state.bd_resume_time = val;

	return count;
}

static DEVICE_ATTR(bd_resume_time, 0660,
		   show_bd_resume_time, set_bd_resume_time);

static ssize_t
show_bd_resume_temp(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct chg_drv *chg_drv = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			chg_drv->bd_state.bd_resume_temp);
}

static ssize_t set_bd_resume_temp(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct chg_drv *chg_drv = dev_get_drvdata(dev);
	int ret = 0, val;

	ret = kstrtoint(buf, 0, &val);
	if (ret < 0)
		return ret;

	if (val > MAX_BD_TEMP || val < MIN_BD_TEMP)
		return -EINVAL;

	chg_drv->bd_state.bd_resume_temp = val;

	return count;
}

static DEVICE_ATTR(bd_resume_temp, 0660,
		   show_bd_resume_temp, set_bd_resume_temp);

static ssize_t
show_bd_resume_soc(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct chg_drv *chg_drv = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			chg_drv->bd_state.bd_resume_soc);
}

static ssize_t set_bd_resume_soc(struct device *dev, struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct chg_drv *chg_drv = dev_get_drvdata(dev);
	int ret = 0, val;

	ret = kstrtoint(buf, 0, &val);
	if (ret < 0)
		return ret;

	if (val > MAX_BD_SOC || val < MIN_BD_SOC)
		return -EINVAL;

	chg_drv->bd_state.bd_resume_soc = val;

	return count;
}

static DEVICE_ATTR(bd_resume_soc, 0660,
		   show_bd_resume_soc, set_bd_resume_soc);

static ssize_t
show_bd_temp_dry_run(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct chg_drv *chg_drv = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			chg_drv->bd_state.bd_temp_dry_run);
}

static ssize_t set_bd_temp_dry_run(struct device *dev, struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct chg_drv *chg_drv = dev_get_drvdata(dev);
	bool dry_run = chg_drv->bd_state.bd_temp_dry_run;
	int ret = 0, val;

	ret = kstrtoint(buf, 0, &val);
	if (ret < 0)
		return ret;

	if (val > 0 && !dry_run) {
		ret = gvotable_cast_bool_vote(chg_drv->msc_temp_dry_run_votable,
					      TEMP_DRYRUN_VOTER, true);
		if (ret < 0)
			dev_err(chg_drv->device, "Couldn't vote true"
				" to bd_temp_dry_run ret=%d\n", ret);
	} else if (val <= 0 && dry_run) {
		ret = gvotable_cast_bool_vote(chg_drv->msc_temp_dry_run_votable,
					      TEMP_DRYRUN_VOTER, false);
		if (ret < 0)
			dev_err(chg_drv->device, "Couldn't disable "
				"bd_temp_dry_run ret=%d\n", ret);
		if (chg_drv->bd_state.triggered) {
			mutex_lock(&chg_drv->bd_lock);
			bd_reset(&chg_drv->bd_state);
			mutex_unlock(&chg_drv->bd_lock);
		}
	}

	return count;
}

static DEVICE_ATTR(bd_temp_dry_run, 0660,
		   show_bd_temp_dry_run, set_bd_temp_dry_run);

static ssize_t bd_clear_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct chg_drv *chg_drv = dev_get_drvdata(dev);
	int ret = 0, val = 0;

	ret = kstrtoint(buf, 0, &val);
	if (ret < 0)
		return ret;

	if (!val)
		return ret;

	mutex_lock(&chg_drv->bd_lock);

	bd_reset(&chg_drv->bd_state);

	ret = bd_batt_set_state(chg_drv, false, -1);
	if (ret < 0)
		pr_err("MSC_BD set_batt_state (%d)\n", ret);

	mutex_unlock(&chg_drv->bd_lock);

	return count;
}

static DEVICE_ATTR_WO(bd_clear);

static ssize_t
bd_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct chg_drv *chg_drv = dev_get_drvdata(dev);
	struct bd_data *bd_state = &chg_drv->bd_state;
	long long temp_avg;
	ssize_t len;

	mutex_lock(&chg_drv->bd_lock);

	temp_avg = div64_s64(bd_state->temp_sum, bd_state->time_sum);
	len = scnprintf(buf, PAGE_SIZE,
		       "t_sum=%lld, time_sum=%lld t_avg=%lld lst_v=%d lst_t=%d lst_u=%lld, dt=%lld, t=%d e=%d\n",
		       bd_state->temp_sum, bd_state->time_sum, temp_avg,
		       bd_state->last_voltage, bd_state->last_temp,
		       bd_state->last_update, bd_state->disconnect_time,
		       bd_state->triggered, bd_state->enabled);

	mutex_unlock(&chg_drv->bd_lock);

	return len;
}

static DEVICE_ATTR_RO(bd_state);

static ssize_t show_dd_state(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct chg_drv *chg_drv = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", chg_drv->bd_state.dd_state);
}

static ssize_t set_dd_state(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	struct chg_drv *chg_drv = dev_get_drvdata(dev);
	int ret = 0, val;

	ret = kstrtoint(buf, 0, &val);
	if (ret < 0)
		return ret;

	if (val > DOCK_DEFEND_ENABLED || val < DOCK_DEFEND_DISABLED)
		return -EINVAL;

	if (chg_drv->bd_state.dd_state != val) {
		chg_drv->bd_state.dd_state = val;
		if (chg_drv->bat_psy)
			power_supply_changed(chg_drv->bat_psy);
	}

	return count;
}

static DEVICE_ATTR(dd_state, 0660,
		   show_dd_state, set_dd_state);

static ssize_t show_dd_settings(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct chg_drv *chg_drv = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", chg_drv->bd_state.dd_settings);
}

static ssize_t set_dd_settings(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	struct chg_drv *chg_drv = dev_get_drvdata(dev);
	int ret = 0, val;

	ret = kstrtoint(buf, 0, &val);
	if (ret < 0)
		return ret;

	if (val > DOCK_DEFEND_USER_ENABLED || val < DOCK_DEFEND_USER_DISABLED)
		return -EINVAL;

	if (chg_drv->bd_state.dd_settings != val) {
		chg_drv->bd_state.dd_settings = val;

		/* Update DD stats tier. Book stats till now to DOCK tier */
		if (DOCK_DEFEND_USER_CLEARED == chg_drv->bd_state.dd_settings)
			dd_stats_update(chg_drv);

		if (chg_drv->bat_psy)
			power_supply_changed(chg_drv->bat_psy);
	}

	return count;
}

static DEVICE_ATTR(dd_settings, 0660,
		   show_dd_settings, set_dd_settings);

static ssize_t show_dd_charge_stop_level(struct device *dev, struct device_attribute *attr,
					 char *buf)
{
	struct chg_drv *chg_drv = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", chg_drv->bd_state.dd_charge_stop_level);
}

static ssize_t set_dd_charge_stop_level(struct device *dev, struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct chg_drv *chg_drv = dev_get_drvdata(dev);
	int ret = 0, val;

	ret = kstrtoint(buf, 0, &val);
	if (ret < 0)
		return ret;

	if (!chg_drv->bat_psy) {
		pr_err("chg_drv->bat_psy is not ready");
		return -ENODATA;
	}

	if (val == chg_drv->bd_state.dd_charge_stop_level)
		return count;

	if ((val <= chg_drv->bd_state.dd_charge_start_level) ||
	    (val > DEFAULT_CHARGE_STOP_LEVEL))
		return -EINVAL;

	chg_drv->bd_state.dd_charge_stop_level = val;
	if (chg_drv->bat_psy)
		power_supply_changed(chg_drv->bat_psy);

	return count;
}

static DEVICE_ATTR(dd_charge_stop_level, 0660,
		   show_dd_charge_stop_level, set_dd_charge_stop_level);

static ssize_t show_dd_charge_start_level(struct device *dev, struct device_attribute *attr,
					  char *buf)
{
	struct chg_drv *chg_drv = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", chg_drv->bd_state.dd_charge_start_level);
}

static ssize_t set_dd_charge_start_level(struct device *dev, struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct chg_drv *chg_drv = dev_get_drvdata(dev);
	int ret = 0, val;

	ret = kstrtoint(buf, 0, &val);
	if (ret < 0)
		return ret;

	if (!chg_drv->bat_psy) {
		pr_err("chg_drv->bat_psy is not ready");
		return -ENODATA;
	}

	if (val == chg_drv->bd_state.dd_charge_start_level)
		return count;

	if ((val >= chg_drv->bd_state.dd_charge_stop_level) ||
	    (val < DEFAULT_CHARGE_START_LEVEL))
		return -EINVAL;

	chg_drv->bd_state.dd_charge_start_level = val;
	if (chg_drv->bat_psy)
		power_supply_changed(chg_drv->bat_psy);

	return count;
}

static DEVICE_ATTR(dd_charge_start_level, 0660,
		   show_dd_charge_start_level, set_dd_charge_start_level);

/* TODO: now created in qcom code, create in chg_create_votables() */
static int chg_find_votables(struct chg_drv *chg_drv)
{
	if (!chg_drv->usb_icl_votable)
		chg_drv->usb_icl_votable = gvotable_election_get_handle("USB_ICL");
	if (!chg_drv->dc_suspend_votable)
		chg_drv->dc_suspend_votable = gvotable_election_get_handle("DC_SUSPEND");

	return (!chg_drv->usb_icl_votable || !chg_drv->dc_suspend_votable)
		? -EINVAL : 0;
}

/* input suspend votes 0 ICL and call suspend on DC_ICL */
static int chg_vote_input_suspend(struct chg_drv *chg_drv,
				  char *voter, bool suspend)
{
	int rc;

	if (chg_find_votables(chg_drv) < 0)
		return -EINVAL;

	rc = gvotable_cast_int_vote(chg_drv->usb_icl_votable,
				    voter, 0, suspend);
	if (rc < 0)
		dev_err(chg_drv->device, "Couldn't vote to %s USB rc=%d\n",
			suspend ? "suspend" : "resume", rc);

	rc = gvotable_cast_bool_vote(chg_drv->dc_suspend_votable, voter, suspend);
	if (rc < 0)
		dev_err(chg_drv->device, "Couldn't vote to %s DC rc=%d\n",
			suspend ? "suspend" : "resume", rc);

	rc = gvotable_cast_bool_vote(chg_drv->msc_chg_disable_votable,
				     voter, suspend);
	if (rc < 0)
		dev_err(chg_drv->device, "Couldn't vote to %s USB rc=%d\n",
			suspend ? "suspend" : "resume", rc);

	return 0;
}

#ifdef CONFIG_DEBUG_FS

static int chg_get_input_suspend(void *data, u64 *val)
{
	struct chg_drv *chg_drv = (struct chg_drv *)data;
	int usb_icl, dc_suspend;

	if (chg_find_votables(chg_drv) < 0)
		return -EINVAL;

	usb_icl = gvotable_get_int_vote(chg_drv->usb_icl_votable, USER_VOTER);
	dc_suspend = gvotable_get_int_vote(chg_drv->dc_suspend_votable,
					   USER_VOTER);
	*val = (usb_icl == 0) && dc_suspend;

	return 0;
}

static int chg_set_input_suspend(void *data, u64 val)
{
	struct chg_drv *chg_drv = (struct chg_drv *)data;
	int rc;

	if (chg_find_votables(chg_drv) < 0)
		return -EINVAL;

	/* will set suspend on DC and vote 0 on ICL */
	rc = chg_vote_input_suspend(chg_drv, USER_VOTER, val != 0);
	if (rc < 0)
		return rc;

	if (val > CHG_INPUT_SUSPEND_ENABLED_CHARGER_STATUS) {
		pr_err("Error! Invalid input suspend value\n");
		return -EINVAL;
	}

	chg_drv->debug_input_suspend = val;

	if (chg_drv->chg_psy)
		power_supply_changed(chg_drv->chg_psy);

	return rc;
}
DEFINE_SIMPLE_ATTRIBUTE(chg_is_fops, chg_get_input_suspend,
				     chg_set_input_suspend, "%llu\n");


static int chg_get_chg_suspend(void *data, u64 *val)
{
	struct chg_drv *chg_drv = (struct chg_drv *)data;

	if (!chg_drv->msc_fcc_votable)
		return -EINVAL;

	/* can also set GBMS_PROP_CHARGE_DISABLE to charger */
	*val = gvotable_get_int_vote(chg_drv->msc_fcc_votable, USER_VOTER) == 0;

	return 0;
}

static int chg_set_chg_suspend(void *data, u64 val)
{
	struct chg_drv *chg_drv = (struct chg_drv *)data;
	int rc;

	if (!chg_drv->msc_fcc_votable)
		return -EINVAL;

	/* can also set GBMS_PROP_CHARGE_DISABLE to charger */
	rc = gvotable_cast_int_vote(chg_drv->msc_fcc_votable,
				    USER_VOTER, 0, val != 0);
	if (rc < 0) {
		dev_err(chg_drv->device,
			"Couldn't vote %s to chg_suspend rc=%d\n",
			val ? "suspend" : "resume", rc);
		return rc;
	}

	if (chg_drv->chg_psy)
		power_supply_changed(chg_drv->chg_psy);

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(chg_cs_fops, chg_get_chg_suspend,
				     chg_set_chg_suspend, "%llu\n");


static int chg_get_update_interval(void *data, u64 *val)
{
	struct chg_drv *chg_drv = (struct chg_drv *)data;

	if (!chg_drv->msc_interval_votable)
		return -EINVAL;

	/* can also set GBMS_PROP_CHARGE_DISABLE to charger */
	*val = gvotable_get_int_vote(chg_drv->msc_interval_votable,
				     USER_VOTER) == 0;

	return 0;
}

static int chg_set_update_interval(void *data, u64 val)
{
	struct chg_drv *chg_drv = (struct chg_drv *)data;
	int rc;

	if (val < 0)
		return -ERANGE;
	if (!chg_drv->msc_interval_votable)
		return -EINVAL;

	/* can also set GBMS_PROP_CHARGE_DISABLE to charger */
	rc = gvotable_cast_int_vote(chg_drv->msc_interval_votable,
				    USER_VOTER, 0, val);
	if (rc < 0) {
		dev_err(chg_drv->device,
			"Couldn't vote %lld to update_interval rc=%d\n",
			val, rc);
		return rc;
	}

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(chg_ui_fops, chg_get_update_interval,
				     chg_set_update_interval, "%llu\n");


/* use qcom VS maxim fg and more... */
static int get_chg_mode(void *data, u64 *val)
{
	struct chg_drv *chg_drv = (struct chg_drv *)data;

	*val = chg_drv->chg_mode;
	return 0;
}

static int set_chg_mode(void *data, u64 val)
{
	struct chg_drv *chg_drv = (struct chg_drv *)data;

	chg_drv->chg_mode = val;
	reschedule_chg_work(chg_drv);
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(chg_mode_fops, get_chg_mode, set_chg_mode, "%llu\n");

static int chg_get_fv_uv(void *data, u64 *val)
{
	struct chg_drv *chg_drv = (struct chg_drv *)data;

	*val = chg_drv->user_fv_uv;
	return 0;
}

static int chg_set_fv_uv(void *data, u64 val)
{
	struct chg_drv *chg_drv = (struct chg_drv *)data;

	if ((((int)val < -1)) || ((chg_drv->batt_profile_fv_uv > 0) &&
			   (val > chg_drv->batt_profile_fv_uv)))
		return -ERANGE;
	if (chg_drv->user_fv_uv == val)
		return 0;

	gvotable_cast_int_vote(chg_drv->msc_fv_votable,
			       MSC_USER_VOTER, val, (val > 0));
	chg_drv->user_fv_uv = val;

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(fv_uv_fops, chg_get_fv_uv,
				     chg_set_fv_uv, "%llu\n");

static int chg_get_cc_max(void *data, u64 *val)
{
	struct chg_drv *chg_drv = (struct chg_drv *)data;

	*val = chg_drv->user_cc_max;
	return 0;
}

static int chg_set_cc_max(void *data, u64 val)
{
	struct chg_drv *chg_drv = (struct chg_drv *)data;

	if ((((int)val < -1)) || ((chg_drv->batt_profile_fcc_ua > 0) &&
			   (val > chg_drv->batt_profile_fcc_ua)))
		return -ERANGE;
	if (chg_drv->user_cc_max == val)
		return 0;

	gvotable_cast_int_vote(chg_drv->msc_fcc_votable,
			       MSC_USER_VOTER, val, (val >= 0));
	chg_drv->user_cc_max = val;

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(cc_max_fops, chg_get_cc_max,
				     chg_set_cc_max, "%llu\n");


static int chg_get_interval(void *data, u64 *val)
{
	struct chg_drv *chg_drv = (struct chg_drv *)data;

	*val = chg_drv->user_interval;
	return 0;
}

static int chg_set_interval(void *data, u64 val)
{
	struct chg_drv *chg_drv = (struct chg_drv *)data;

	if (chg_drv->user_interval == val)
		return 0;

	gvotable_cast_int_vote(chg_drv->msc_interval_votable,
			       MSC_USER_VOTER, val, (val >= 0));
	chg_drv->user_interval = val;

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(chg_interval_fops,
				chg_get_interval,
				chg_set_interval, "%llu\n");


static int chg_reschedule_work(void *data, u64 val)
{
	struct chg_drv *chg_drv = (struct chg_drv *)data;

	reschedule_chg_work(chg_drv);
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(chg_reschedule_work_fops,
					NULL, chg_reschedule_work, "%llu\n");

static int bd_enabled_get(void *data, u64 *val)
{
	struct chg_drv *chg_drv = (struct chg_drv *)data;

	mutex_lock(&chg_drv->bd_lock);
	*val = chg_drv->bd_state.enabled;
	mutex_unlock(&chg_drv->bd_lock);

	return 0;
}

static int bd_enabled_set(void *data, u64 val)
{
	struct chg_drv *chg_drv = (struct chg_drv *)data;

	mutex_lock(&chg_drv->bd_lock);

	bd_reset(&chg_drv->bd_state);
	if (val == 0)
		chg_drv->bd_state.enabled = 0;

	mutex_unlock(&chg_drv->bd_lock);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(bd_enabled_fops, bd_enabled_get,
			bd_enabled_set, "%lld\n");


#endif

static int debug_get_pps_cc_tolerance(void *data, u64 *val)
{
	struct chg_drv *chg_drv = data;

	*val = chg_drv->pps_cc_tolerance_pct;
	return 0;
}

static int debug_set_pps_cc_tolerance(void *data, u64 val)
{
	struct chg_drv *chg_drv = data;

	chg_drv->pps_cc_tolerance_pct = val;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_pps_cc_tolerance_fops,
					debug_get_pps_cc_tolerance,
					debug_set_pps_cc_tolerance, "%llu\n");

static ssize_t
charging_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct chg_drv *chg_drv = dev_get_drvdata(dev);
	int value = CSI_STATUS_UNKNOWN;

	if (chg_drv->csi_status_votable)
		value = gvotable_get_current_int_vote(chg_drv->csi_status_votable);

	return scnprintf(buf, PAGE_SIZE, "%d\n", value);
}

static DEVICE_ATTR_RO(charging_status);

static ssize_t
charging_type_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct chg_drv *chg_drv = dev_get_drvdata(dev);
	int value = CSI_TYPE_UNKNOWN;

	if (chg_drv->csi_type_votable)
		value = gvotable_get_current_int_vote(chg_drv->csi_type_votable);

	return scnprintf(buf, PAGE_SIZE, "%d\n", value);
}

static DEVICE_ATTR_RO(charging_type);

static ssize_t
thermal_stats_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct chg_drv *chg_drv = dev_get_drvdata(dev);
	ssize_t len = 0;
	int size = PAGE_SIZE;
	int i;

	mutex_lock(&chg_drv->stats_lock);
	for (i = 0; i < STATS_THERMAL_LEVELS_MAX; i++) {
		if (chg_drv->thermal_stats[i].soc_in != -1)
			len += gbms_tier_stats_cstr(&buf[len],
						    size - len,
						    &chg_drv->thermal_stats[i],
						    false);
	}
	mutex_unlock(&chg_drv->stats_lock);

	return len;
}

static ssize_t thermal_stats_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct chg_drv *chg_drv = dev_get_drvdata(dev);

	if (count < 1)
		return -ENODATA;

	if (buf[0] == '0')
		thermal_stats_init(chg_drv);

	return count;
}

static DEVICE_ATTR_RW(thermal_stats);

static ssize_t
thermal_dc_fan_alarm_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct chg_drv *chg_drv = dev_get_drvdata(dev);
	struct chg_thermal_device *ctdev_dcin = &chg_drv->thermal_devices[CHG_TERMAL_DEVICE_DC_IN];
	int value = ctdev_dcin->therm_fan_alarm_level;

	return scnprintf(buf, PAGE_SIZE, "%d\n", value);
}

static ssize_t thermal_dc_fan_alarm_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	struct chg_drv *chg_drv = dev_get_drvdata(dev);
	struct chg_thermal_device *ctdev_dcin = &chg_drv->thermal_devices[CHG_TERMAL_DEVICE_DC_IN];
	int ret = 0;
	u32 value;

	ret = kstrtou32(buf, 0, &value);
	if (ret < 0)
		return ret;

	if (value <= ctdev_dcin->thermal_levels)
		ctdev_dcin->therm_fan_alarm_level = value;

	return count;
}

static DEVICE_ATTR_RW(thermal_dc_fan_alarm);

static ssize_t
charge_stats_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct chg_drv *chg_drv = dev_get_drvdata(dev);
	struct gbms_ce_tier_stats *dd_stats = &chg_drv->dd_stats;
	struct gbms_ce_tier_stats *bd_p_stats = &chg_drv->bd_state.bd_pretrigger_stats;
	struct gbms_ce_tier_stats *bd_r_stats = &chg_drv->bd_state.bd_resume_stats;
	uint32_t time_sum;
	ssize_t len = 0;

	mutex_lock(&chg_drv->stats_lock);
	time_sum = dd_stats->time_fast + dd_stats->time_other + dd_stats->time_taper;
	if (time_sum > 0)
		len = gbms_tier_stats_cstr(&buf[len], PAGE_SIZE, dd_stats, false);
	time_sum = bd_p_stats->time_fast + bd_p_stats->time_other + bd_p_stats->time_taper;
	if (time_sum > 0)
		len += gbms_tier_stats_cstr(&buf[len], PAGE_SIZE, bd_p_stats, false);
	if (chg_drv->bd_state.bd_resume_stats_last_update > 0)
		len += gbms_tier_stats_cstr(&buf[len], PAGE_SIZE, bd_r_stats, false);
	len += scnprintf(&buf[len], PAGE_SIZE - len, "\nD:0x0,%#x,0x0,0x0,0x0,0x0,%#x\n",
			 chg_drv->adapter_capabilities[ADAPTER_CAP_APDO],
			 chg_drv->adapter_capabilities[ADAPTER_CAP_PDO]);
	mutex_unlock(&chg_drv->stats_lock);

	return len;
}

static ssize_t charge_stats_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct chg_drv *chg_drv = dev_get_drvdata(dev);

	if (count < 1)
		return -ENODATA;

	if (buf[0] == '0') {
		bd_dd_stats_init(chg_drv);
		chg_drv->adapter_capabilities[ADAPTER_CAP_PDO] = 0;
		chg_drv->adapter_capabilities[ADAPTER_CAP_APDO] = 0;
	}

	return count;
}

static DEVICE_ATTR_RW(charge_stats);

static int chg_init_fs(struct chg_drv *chg_drv)
{
	int ret;

	ret = device_create_file(chg_drv->device, &dev_attr_charge_stop_level);
	if (ret != 0) {
		pr_err("Failed to create charge_stop_level files, ret=%d\n", ret);
		return ret;
	}

	ret = device_create_file(chg_drv->device, &dev_attr_charge_start_level);
	if (ret != 0) {
		pr_err("Failed to create charge_start_level files, ret=%d\n", ret);
		return ret;
	}

	ret = device_create_file(chg_drv->device, &dev_attr_bd_temp_enable);
	if (ret != 0) {
		pr_err("Failed to create bd_temp_enable files, ret=%d\n", ret);
		return ret;
	}

	ret = device_create_file(chg_drv->device, &dev_attr_bd_trigger_voltage);
	if (ret != 0) {
		pr_err("Failed to create bd_trigger_voltage files, ret=%d\n", ret);
		return ret;
	}

	ret = device_create_file(chg_drv->device, &dev_attr_bd_drainto_soc);
	if (ret != 0) {
		pr_err("Failed to create bd_drainto_soc files, ret=%d\n", ret);
		return ret;
	}

	ret = device_create_file(chg_drv->device, &dev_attr_bd_trigger_temp);
	if (ret != 0) {
		pr_err("Failed to create bd_trigger_temp files, ret=%d\n", ret);
		return ret;
	}

	ret = device_create_file(chg_drv->device, &dev_attr_bd_trigger_time);
	if (ret != 0) {
		pr_err("Failed to create bd_trigger_time files, ret=%d\n", ret);
		return ret;
	}

	ret = device_create_file(chg_drv->device,
				&dev_attr_bd_recharge_voltage);
	if (ret != 0) {
		pr_err("Failed to create bd_recharge_voltage files, ret=%d\n", ret);
		return ret;
	}

	ret = device_create_file(chg_drv->device, &dev_attr_bd_recharge_soc);
	if (ret != 0) {
		pr_err("Failed to create bd_recharge_soc files, ret=%d\n", ret);
		return ret;
	}

	ret = device_create_file(chg_drv->device, &dev_attr_bd_resume_abs_temp);
	if (ret != 0) {
		pr_err("Failed to create bd_resume_abs_temp files, ret=%d\n",
		       ret);
		return ret;
	}

	ret = device_create_file(chg_drv->device, &dev_attr_bd_resume_time);
	if (ret != 0) {
		pr_err("Failed to create bd_resume_time files, ret=%d\n",
		       ret);
		return ret;
	}

	ret = device_create_file(chg_drv->device, &dev_attr_bd_resume_temp);
	if (ret != 0) {
		pr_err("Failed to create bd_resume_temp files, ret=%d\n", ret);
		return ret;
	}

	ret = device_create_file(chg_drv->device, &dev_attr_bd_resume_soc);
	if (ret != 0) {
		pr_err("Failed to create bd_resume_soc files, ret=%d\n", ret);
		return ret;
	}

	ret = device_create_file(chg_drv->device, &dev_attr_bd_temp_dry_run);
	if (ret != 0) {
		pr_err("Failed to create bd_temp_dry_run files, ret=%d\n", ret);
		return ret;
	}

	ret = device_create_file(chg_drv->device, &dev_attr_bd_clear);
	if (ret != 0) {
		pr_err("Failed to create bd_clear files, ret=%d\n", ret);
		return ret;
	}

	ret = device_create_file(chg_drv->device, &dev_attr_bd_state);
	if (ret != 0) {
		pr_err("Failed to create bd_state files, ret=%d\n", ret);
		return ret;
	}

	ret = device_create_file(chg_drv->device, &dev_attr_charging_status);
	if (ret != 0) {
		pr_err("Failed to create charging_status, ret=%d\n", ret);
		return ret;
	}

	ret = device_create_file(chg_drv->device, &dev_attr_charging_type);
	if (ret != 0) {
		pr_err("Failed to create charging_type, ret=%d\n", ret);
		return ret;
	}

	ret = device_create_file(chg_drv->device, &dev_attr_thermal_stats);
	if (ret != 0) {
		pr_err("Failed to create thermal_stats, ret=%d\n", ret);
		return ret;
	}

	ret = device_create_file(chg_drv->device, &dev_attr_thermal_dc_fan_alarm);
	if (ret != 0) {
		pr_err("Failed to create thermal_dc_fan_alarm, ret=%d\n", ret);
		return ret;
	}

	ret = device_create_file(chg_drv->device, &dev_attr_charge_stats);
	if (ret != 0) {
		pr_err("Failed to create charge_stats files, ret=%d\n", ret);
		return ret;
	}

	/* dock_defend */
	if (chg_drv->ext_psy_name) {
		ret = device_create_file(chg_drv->device, &dev_attr_dd_state);
		if (ret != 0) {
			pr_err("Failed to create dd_state files, ret=%d\n", ret);
			return ret;
		}

		ret = device_create_file(chg_drv->device, &dev_attr_dd_settings);
		if (ret != 0) {
			pr_err("Failed to create dd_settings files, ret=%d\n", ret);
			return ret;
		}

		ret = device_create_file(chg_drv->device, &dev_attr_dd_charge_stop_level);
		if (ret != 0) {
			pr_err("Failed to create dd_charge_stop_level files, ret=%d\n", ret);
			return ret;
		}

		ret = device_create_file(chg_drv->device, &dev_attr_dd_charge_start_level);
		if (ret != 0) {
			pr_err("Failed to create dd_charge_start_level files, ret=%d\n", ret);
			return ret;
		}
	}


	chg_drv->debug_entry = debugfs_create_dir("google_charger", 0);
	if (IS_ERR_OR_NULL(chg_drv->debug_entry)) {
		chg_drv->debug_entry = NULL;
		return 0;
	}

	debugfs_create_file("chg_mode", 0644, chg_drv->debug_entry,
			    chg_drv, &chg_mode_fops);
	debugfs_create_file("input_suspend", 0644, chg_drv->debug_entry,
			    chg_drv, &chg_is_fops);
	debugfs_create_file("chg_suspend", 0644, chg_drv->debug_entry,
			    chg_drv, &chg_cs_fops);
	debugfs_create_file("update_interval", 0644, chg_drv->debug_entry,
			    chg_drv, &chg_ui_fops);
	debugfs_create_file("force_reschedule", 0600, chg_drv->debug_entry,
			    chg_drv, &chg_reschedule_work_fops);

	debugfs_create_bool("usb_skip_probe", 0600, chg_drv->debug_entry,
			    &chg_drv->usb_skip_probe);

	debugfs_create_file("pps_cc_tolerance", 0644, chg_drv->debug_entry,
			    chg_drv, &debug_pps_cc_tolerance_fops);

	debugfs_create_u32("bd_triggered", S_IRUGO | S_IWUSR, chg_drv->debug_entry,
			   &chg_drv->bd_state.triggered);
	debugfs_create_file("bd_enabled", 0600, chg_drv->debug_entry,
			    chg_drv, &bd_enabled_fops);

	if (chg_drv->enable_user_fcc_fv) {
		debugfs_create_file("fv_uv", 0644, chg_drv->debug_entry,
					chg_drv, &fv_uv_fops);
		debugfs_create_file("cc_max", 0644,
					chg_drv->debug_entry,
					chg_drv, &cc_max_fops);
		debugfs_create_file("interval", 0644,
					chg_drv->debug_entry,
					chg_drv, &chg_interval_fops);
	}

	/* dock_defend */
	if (chg_drv->ext_psy_name)
		debugfs_create_u32("dd_trigger_time", 0644, chg_drv->debug_entry,
				   &chg_drv->bd_state.dd_trigger_time);

	return 0;
}


/*
 * barebone initial pps policy.
 * Works only on output voltage keeping output current to the max.
 * Extend to hande CC
 */
static int pps_policy(struct chg_drv *chg_drv, int fv_uv, int cc_max,
		      struct power_supply *tcpm_psy,
		      struct power_supply *bat_psy)
{
	struct pd_pps_data *pps_data = &chg_drv->pps_data;
	const int ratio = 100 - chg_drv->pps_cc_tolerance_pct;
	const uint8_t flags = chg_drv->chg_state.f.flags;
	int ibatt, vbatt, ioerr;
	uint64_t exp_mw;
	int ret = 0;

	/* TODO: policy for negative/invalid targets? */
	if (cc_max <= 0) {
		logbuffer_log(pps_data->log, "negative cc_max=%d", cc_max);
		return 0;
	}

	/*
	 * TODO: Now we only need to adjust the pps in CC state.
	 * Consider CV state in the future.
	 */
	if (!(flags & GBMS_CS_FLAG_CC)) {
		logbuffer_log(pps_data->log, "waiting for CC flags=%x", flags);
		return 0;
	}

	ibatt = GPSY_GET_INT_PROP(bat_psy, POWER_SUPPLY_PROP_CURRENT_NOW,
				  &ioerr);
	vbatt = GPSY_GET_PROP(bat_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW);

	if (ioerr < 0 || vbatt < 0) {
		logbuffer_log(pps_data->log, "Failed to get ibatt (%d) or vbatt=%d",
			      ioerr, vbatt);
		return -EIO;
	}

	/* TODO: should we compensate for the round down here? */
	exp_mw = div64_u64(((uint64_t)vbatt * (uint64_t)cc_max) * 11,
		 10000000000);

	logbuffer_log(pps_data->log,
		"ibatt %d, vbatt %d, vbatt*cc_max*1.1 %lu mw, adapter %ld, keep_alive_cnt %d",
		ibatt, vbatt, (unsigned long)exp_mw,
		(long)pps_data->out_uv * (long)pps_data->op_ua / 1000000000,
		pps_data->keep_alive_cnt);

	/* negative current is discharging, should chase the load instead? */
	if (ibatt <= 0)
		return 0;

	/* always maximize the input current first */
	/* TODO: b/134799977 adjust the max current */
	if (pps_data->op_ua < pps_data->max_ua) {
		pps_data->op_ua = pps_data->max_ua;
		return 0;
	}

	/* demand more power */
	if ((ibatt < (cc_max * ratio) / 100) || flags & GBMS_CS_FLAG_ILIM) {

		if (pps_data->out_uv == pps_data->max_uv) {
			ret = chg_switch_profile(pps_data, tcpm_psy, true);
			return (ret == 0) ? -ECANCELED : 0;
		}

		pps_adjust_volt(pps_data, 100000);

		/* TODO: b/134799977 adjust the max current */

	/* input power is enough */
	} else {
		/* everything is fine; try to lower the Vout if
		 * battery is satisfied for a period of time */
		if (pps_data->keep_alive_cnt < PPS_KEEP_ALIVE_MAX)
			return 0;

		ret = chg_switch_profile(pps_data, tcpm_psy, false);
		if (ret == 0)
			return -ECANCELED;

		pps_adjust_volt(pps_data, -100000);

		/* TODO: b/134799977 adjust the max current */

	}

	return ret;
}

/*
 * NOTE: when PPS is active we need to ping the adapter or it will revert
 * to fixed profile.
 */
static int msc_update_pps(struct chg_drv *chg_drv, int fv_uv, int cc_max)
{
	struct pd_pps_data *pps_data = &chg_drv->pps_data;
	int rc;

	/* the policy determines the adapter output voltage and current */
	rc = pps_policy(chg_drv, fv_uv, cc_max,
			chg_drv->tcpm_psy, chg_drv->bat_psy);
	if (rc == -ECANCELED)
		return -ECANCELED;

	/* adjust PPS out for target demand */
	rc = pps_update_adapter(pps_data, pps_data->out_uv, pps_data->op_ua,
				chg_drv->tcpm_psy);
	if (rc == -EAGAIN)
		rc = PPS_ERROR_RETRY_MS;

	return rc;
}

/*
 * NOTE: chg_work() vote 0 at the beginning of each loop to gate the updates
 * to the charger
 */
static int msc_update_charger_cb(struct gvotable_election *el,
				 const char *reason, void *vote)
{
	int update_interval, rc = -EINVAL, fv_uv = -1, cc_max = -1, topoff = -1;
	struct chg_drv *chg_drv = gvotable_get_data(el);

	__pm_stay_awake(chg_drv->chg_ws);

	update_interval =
		gvotable_get_current_int_vote(chg_drv->msc_interval_votable);
	if (update_interval <= 0)
		goto msc_done;
	if (chg_drv->chg_mode == CHG_DRV_MODE_NOOP)
		goto msc_reschedule;

	/*
	 * = fv_uv will be at last charger tier in RL, last tier before JEITA
	 *   when JEITA hits, the default (max) value when temperature fall
	 *   outside JEITA limits on connect or negative when a value is not
	 *   specified in device tree. Setting max on JEITA violation is not
	 *   a problem because HW wil set the charge current to 0, the charger
	 *   driver does sanity checking as well (but we should not rely on it)
	 * = cc_max should not be negative here and is 0 on RL and on JEITA.
	 */
	fv_uv = gvotable_get_current_int_vote(chg_drv->msc_fv_votable);
	cc_max = gvotable_get_current_int_vote(chg_drv->msc_fcc_votable);
	if (chg_drv->bat_psy)
		topoff = GPSY_GET_PROP(chg_drv->bat_psy, POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT);

	/* invalid values will cause the adapter to exit PPS */
	if (cc_max < 0 || fv_uv < 0) {
		update_interval = CHG_WORK_ERROR_RETRY_MS;
		goto msc_reschedule;
	}

	if (chg_drv->pps_data.stage == PPS_ACTIVE) {
		int pps_ui;

		pps_ui = msc_update_pps(chg_drv, fv_uv, cc_max);
		/* I am not sure that is the case */
		if (pps_ui == -ECANCELED)
			goto msc_done;
		/* force ping */
		if (pps_ui >= 0 && pps_ui < update_interval)
			update_interval = pps_ui;
	}

	/* adjust charger for target demand */
	rc = chg_update_charger(chg_drv, fv_uv, cc_max, topoff);
	if (rc == -EAGAIN)
		update_interval = CHG_WORK_EAGAIN_RETRY_MS;
	else if (rc < 0)
		update_interval = CHG_WORK_ERROR_RETRY_MS;

msc_reschedule:
	alarm_try_to_cancel(&chg_drv->chg_wakeup_alarm);
	alarm_start_relative(&chg_drv->chg_wakeup_alarm,
			     ms_to_ktime(update_interval));

	pr_debug("MSC_CHG fv_uv=%d, cc_max=%d, rerun in %d ms (%d)\n",
		 fv_uv, cc_max, update_interval, rc);

msc_done:
	__pm_relax(chg_drv->chg_ws);
	return 0;
}

/*
 * NOTE: we need a single source of truth. Charging can be disabled via the
 * votable and directy setting the property.
 */
static int msc_chg_disable_cb(struct gvotable_election *el,
			      const char *reason, void *vote)
{
	struct chg_drv *chg_drv = gvotable_get_data(el);
	int chg_disable = GVOTABLE_PTR_TO_INT(vote);
	int rc;

	if (!chg_drv->chg_psy)
		return 0;

	rc = GPSY_SET_PROP(chg_drv->chg_psy, GBMS_PROP_CHARGE_DISABLE, chg_disable);
	if (rc < 0)
		dev_err(chg_drv->device, "Couldn't %s charging rc=%d\n",
				chg_disable ? "disable" : "enable", rc);

	return 0;
}

static int msc_pwr_disable_cb(struct gvotable_election *el,
			      const char *reason, void *vote)
{
	struct chg_drv *chg_drv = gvotable_get_data(el);
	int pwr_disable = GVOTABLE_PTR_TO_INT(vote);

	if (!chg_drv->chg_psy)
		return 0;

	chg_vote_input_suspend(chg_drv, MSC_PWR_VOTER, pwr_disable);

	return 0;
}

static int msc_last_cb(struct gvotable_election *el, const char *reason, void *vote)
{
	struct chg_drv *chg_drv = gvotable_get_data(el);
	int last_tier = GVOTABLE_PTR_TO_INT(vote);
	int taper_ctl = last_tier ? GBMS_TAPER_CONTROL_ON : GBMS_TAPER_CONTROL_OFF;
	struct power_supply *chg_psy = chg_drv->chg_psy;
	int rc;

	if (!chg_psy || !chg_drv->taper_last_tier)
		return 0;

	/* GBMS_PROP_TAPER_CONTROL is optional */
	rc = GPSY_SET_PROP(chg_psy, GBMS_PROP_TAPER_CONTROL, taper_ctl);
	if (rc < 0)
		pr_debug("MSC_CHG cannot set taper control rc=%d\n", rc);

	return rc;

}

static void chg_update_charging_policy(struct chg_drv *chg_drv, const int value)
{
	/* set custom upper and lower bound for long_life charging policy */
	if (value == CHARGING_POLICY_VOTE_LONGLIFE &&
	    chg_drv->charging_policy != CHARGING_POLICY_VOTE_LONGLIFE) {
		chg_drv->charge_stop_level = LONGLIFE_CHARGE_STOP_LEVEL;
		chg_drv->charge_start_level = LONGLIFE_CHARGE_START_LEVEL;
	} else if (value != CHARGING_POLICY_VOTE_LONGLIFE &&
		   chg_drv->charging_policy == CHARGING_POLICY_VOTE_LONGLIFE) {
		chg_drv->charge_stop_level = DEFAULT_CHARGE_STOP_LEVEL;
		chg_drv->charge_start_level = DEFAULT_CHARGE_START_LEVEL;
	}

	chg_drv->charging_policy = value;
}

static int charging_policy_cb(struct gvotable_election *el,
			      const char *reason, void *vote)
{
	struct chg_drv *chg_drv = gvotable_get_data(el);
	int charging_policy = GVOTABLE_PTR_TO_INT(vote);

	if (!chg_drv || chg_drv->charging_policy == charging_policy)
		return 0;

	chg_update_charging_policy(chg_drv, charging_policy);

	if (chg_drv->bat_psy)
		power_supply_changed(chg_drv->bat_psy);

	return 0;
}

static int chg_disable_std_votables(struct chg_drv *chg_drv)
{
	struct gvotable_election *qc_votable;
	bool std_votables;

	std_votables = of_property_read_bool(chg_drv->device->of_node,
					     "google,has-std-votables");
	if (!std_votables)
		return 0;

	qc_votable = gvotable_election_get_handle("FV");
	if (!qc_votable)
		return -EPROBE_DEFER;

	gvotable_cast_long_vote(qc_votable, MSC_CHG_VOTER, -1, true);

	qc_votable = gvotable_election_get_handle("FCC");
	if (!qc_votable)
		return -EPROBE_DEFER;

	gvotable_cast_long_vote(qc_votable, MSC_CHG_VOTER, -1, true);

	return 0;
}

static int chg_force_5v_cb(struct gvotable_election *el,
			  const char *reason, void *vote)
{
	struct chg_drv *chg_drv = gvotable_get_data(el);

	if (chg_drv->chg_psy)
		power_supply_changed(chg_drv->chg_psy);

	return 0;
}

static void chg_destroy_votables(struct chg_drv *chg_drv)
{
	gvotable_destroy_election(chg_drv->msc_fv_votable);
	gvotable_destroy_election(chg_drv->msc_fcc_votable);
	gvotable_destroy_election(chg_drv->msc_interval_votable);
	gvotable_destroy_election(chg_drv->msc_chg_disable_votable);
	gvotable_destroy_election(chg_drv->msc_pwr_disable_votable);
	gvotable_destroy_election(chg_drv->msc_temp_dry_run_votable);
	gvotable_destroy_election(chg_drv->msc_last_votable);
	gvotable_destroy_election(chg_drv->charging_policy_votable);
	gvotable_destroy_election(chg_drv->thermal_level_votable);
	gvotable_destroy_election(chg_drv->force_5v_votable);

	chg_drv->msc_fv_votable = NULL;
	chg_drv->msc_fcc_votable = NULL;
	chg_drv->msc_interval_votable = NULL;
	chg_drv->msc_chg_disable_votable = NULL;
	chg_drv->msc_pwr_disable_votable = NULL;
	chg_drv->msc_temp_dry_run_votable = NULL;
	chg_drv->csi_status_votable = NULL;
	chg_drv->csi_type_votable = NULL;
	chg_drv->msc_last_votable = NULL;
	chg_drv->charging_policy_votable = NULL;
	chg_drv->thermal_level_votable = NULL;
	chg_drv->force_5v_votable = NULL;
}

/* TODO: qcom/battery.c mostly handles PL charging: we don't need it.
 *  In order to remove and keep using QCOM code, create "USB_ICL",
 *  "PL_DISABLE", "PL_AWAKE" and "PL_ENABLE_INDIRECT" in a new function called
 *  qcom_batt_init(). Also might need to change the names of our votables for
 *  FCC, FV to match QCOM.
 * NOTE: Battery also register "qcom-battery" class so it might not be too
 * straightforward to remove all dependencies.
 */
static int chg_create_votables(struct chg_drv *chg_drv)
{
	int ret;

	chg_drv->msc_fv_votable =
		gvotable_create_int_election(NULL, gvotable_comparator_int_min,
					     NULL, chg_drv);
	if (IS_ERR_OR_NULL(chg_drv->msc_fv_votable)) {
		ret = PTR_ERR(chg_drv->msc_fv_votable);
		chg_drv->msc_fv_votable = NULL;
		goto error_exit;
	}

	gvotable_set_vote2str(chg_drv->msc_fv_votable, gvotable_v2s_int);
	gvotable_disable_force_int_entry(chg_drv->msc_fv_votable);
	gvotable_election_set_name(chg_drv->msc_fv_votable, VOTABLE_MSC_FV);

	chg_drv->msc_fcc_votable =
		gvotable_create_int_election(NULL, gvotable_comparator_int_min,
					     NULL, chg_drv);
	if (IS_ERR_OR_NULL(chg_drv->msc_fcc_votable)) {
		ret = PTR_ERR(chg_drv->msc_fcc_votable);
		chg_drv->msc_fcc_votable = NULL;
		goto error_exit;
	}

	gvotable_set_vote2str(chg_drv->msc_fcc_votable, gvotable_v2s_int);
	gvotable_disable_force_int_entry(chg_drv->msc_fcc_votable);
	gvotable_election_set_name(chg_drv->msc_fcc_votable, VOTABLE_MSC_FCC);

	chg_drv->msc_interval_votable =
		gvotable_create_int_election(NULL, gvotable_comparator_int_min,
					     msc_update_charger_cb, chg_drv);
	if (IS_ERR_OR_NULL(chg_drv->msc_interval_votable)) {
		ret = PTR_ERR(chg_drv->msc_interval_votable);
		chg_drv->msc_interval_votable = NULL;
		goto error_exit;
	}

	gvotable_set_vote2str(chg_drv->msc_interval_votable, gvotable_v2s_int);
	gvotable_election_set_name(chg_drv->msc_interval_votable,
				   VOTABLE_MSC_INTERVAL);

	chg_drv->msc_chg_disable_votable =
		gvotable_create_bool_election(NULL, msc_chg_disable_cb,
					      chg_drv);
	if (IS_ERR_OR_NULL(chg_drv->msc_chg_disable_votable)) {
		ret = PTR_ERR(chg_drv->msc_chg_disable_votable);
		chg_drv->msc_chg_disable_votable = NULL;
		goto error_exit;
	}

	gvotable_set_vote2str(chg_drv->msc_chg_disable_votable,
			      gvotable_v2s_int);
	gvotable_election_set_name(chg_drv->msc_chg_disable_votable,
				   VOTABLE_MSC_CHG_DISABLE);

	chg_drv->msc_pwr_disable_votable =
		gvotable_create_bool_election(NULL, msc_pwr_disable_cb,
					      chg_drv);
	if (IS_ERR_OR_NULL(chg_drv->msc_pwr_disable_votable)) {
		ret = PTR_ERR(chg_drv->msc_pwr_disable_votable);
		chg_drv->msc_pwr_disable_votable = NULL;
		goto error_exit;
	}

	gvotable_set_vote2str(chg_drv->msc_pwr_disable_votable,
			      gvotable_v2s_int);
	gvotable_election_set_name(chg_drv->msc_pwr_disable_votable,
				   VOTABLE_MSC_PWR_DISABLE);

	chg_drv->msc_temp_dry_run_votable =
		gvotable_create_bool_election(NULL, msc_temp_defend_dryrun_cb,
					      chg_drv);
	if (IS_ERR_OR_NULL(chg_drv->msc_temp_dry_run_votable)) {
		ret = PTR_ERR(chg_drv->msc_temp_dry_run_votable);
		chg_drv->msc_temp_dry_run_votable = NULL;
		goto error_exit;
	}

	gvotable_set_vote2str(chg_drv->msc_temp_dry_run_votable,
			      gvotable_v2s_int);
	gvotable_election_set_name(chg_drv->msc_temp_dry_run_votable,
				   VOTABLE_TEMP_DRYRUN);

	chg_drv->msc_last_votable =
		gvotable_create_int_election(NULL, gvotable_comparator_int_min, msc_last_cb,
					     chg_drv);
	if (IS_ERR_OR_NULL(chg_drv->msc_last_votable)) {
		ret = PTR_ERR(chg_drv->msc_last_votable);
		chg_drv->msc_last_votable = NULL;
		goto error_exit;
	}

	gvotable_set_default(chg_drv->msc_last_votable, (void *)0);
	gvotable_set_vote2str(chg_drv->msc_last_votable, gvotable_v2s_int);
	gvotable_election_set_name(chg_drv->msc_last_votable, VOTABLE_MSC_LAST);
	gvotable_use_default(chg_drv->msc_last_votable, true);

	chg_drv->charging_policy_votable =
		gvotable_create_int_election(NULL, gvotable_comparator_int_max,
					     charging_policy_cb, chg_drv);
	if (IS_ERR_OR_NULL(chg_drv->charging_policy_votable)) {
		ret = PTR_ERR(chg_drv->charging_policy_votable);
		chg_drv->charging_policy_votable = NULL;
		goto error_exit;
	}

	gvotable_set_vote2str(chg_drv->charging_policy_votable, gvotable_v2s_int);
	gvotable_election_set_name(chg_drv->charging_policy_votable,
				   VOTABLE_CHARGING_POLICY);
	gvotable_cast_long_vote(chg_drv->charging_policy_votable,
				"DEFAULT", CHARGING_POLICY_DEFAULT, true);

	chg_drv->thermal_level_votable =
		gvotable_create_int_election(NULL, gvotable_comparator_least_recent,
					     thermal_tier_stats_update, chg_drv);
	if (IS_ERR_OR_NULL(chg_drv->thermal_level_votable)) {
		ret = PTR_ERR(chg_drv->thermal_level_votable);
		chg_drv->thermal_level_votable = NULL;
		goto error_exit;
	}

	gvotable_set_default(chg_drv->thermal_level_votable, (void *)0);
	gvotable_set_vote2str(chg_drv->thermal_level_votable, gvotable_v2s_int);
	gvotable_election_set_name(chg_drv->thermal_level_votable, VOTABLE_THERMAL_LVL);

	chg_drv->force_5v_votable =
		gvotable_create_int_election(NULL, gvotable_comparator_int_min,
					     chg_force_5v_cb, chg_drv);
	if (IS_ERR_OR_NULL(chg_drv->force_5v_votable)) {
		ret = PTR_ERR(chg_drv->force_5v_votable);
		chg_drv->force_5v_votable = NULL;
		goto error_exit;
	}

	gvotable_set_default(chg_drv->force_5v_votable, (void *)1);
	gvotable_set_vote2str(chg_drv->force_5v_votable, gvotable_v2s_int);
	gvotable_election_set_name(chg_drv->force_5v_votable, VOTABLE_FORCE_5V);
	gvotable_use_default(chg_drv->force_5v_votable, true);

	return 0;

error_exit:
	chg_destroy_votables(chg_drv);

	return ret;
}

static void chg_init_votables(struct chg_drv *chg_drv)
{
	/* prevent all changes until the first roundtrip with real state */
	gvotable_cast_int_vote(chg_drv->msc_interval_votable,
			       MSC_CHG_VOTER, 0, true);

	/* will not be applied until we vote non-zero msc_interval */
	gvotable_cast_int_vote(chg_drv->msc_fv_votable, MAX_VOTER,
			       chg_drv->batt_profile_fv_uv,
			       chg_drv->batt_profile_fv_uv > 0);
	gvotable_cast_int_vote(chg_drv->msc_fcc_votable, MAX_VOTER,
			       chg_drv->batt_profile_fcc_ua,
			       chg_drv->batt_profile_fcc_ua > 0);

	/* update temp dry run votable if bd_temp_dry_run is set from DT */
	if (chg_drv->msc_temp_dry_run_votable && chg_drv->bd_state.bd_temp_dry_run)
		gvotable_cast_bool_vote(chg_drv->msc_temp_dry_run_votable, TEMP_DRYRUN_VOTER,
					chg_drv->bd_state.bd_temp_dry_run);
}

static int fan_get_level(struct chg_thermal_device *tdev)
{
	int level = FAN_LVL_UNKNOWN;
	int alarm_level = tdev->therm_fan_alarm_level;

	if (tdev->current_level == 0)
		level = FAN_LVL_NOT_CARE;
	else if (tdev->current_level >= alarm_level)
		level = FAN_LVL_ALARM;
	else
		level = FAN_LVL_MED;

	return level;
}

#define FAN_VOTER_DC_IN		"THERMAL_DC_IN"
#define FAN_VOTER_WLC_FCC	"THERMAL_WLC_FCC"

/*
 * We might have differnt fan hints for the DCIN and for FCC_IN.
 * Check the online state of WLC to figure out which one to apply.
 */
static int fan_vote_level(struct chg_drv *chg_drv, const char *reason, int hint)
{
	int ret;

	if (!chg_drv->fan_level_votable) {
		chg_drv->fan_level_votable =
			gvotable_election_get_handle("FAN_LEVEL");
		if (!chg_drv->fan_level_votable)
			return 0;
	}

	ret = gvotable_cast_int_vote(chg_drv->fan_level_votable,
				     reason, hint, hint != -1);

	pr_debug("MSC_THERM_FAN reason=%s, level=%d ret=%d\n",
		 reason, hint, ret);

	return ret;
}

static int chg_get_max_charge_cntl_limit(struct thermal_cooling_device *tcd,
					 unsigned long *lvl)
{
	struct chg_thermal_device *tdev =
		(struct chg_thermal_device *)tcd->devdata;
	*lvl = tdev->thermal_levels;
	return 0;
}

static int chg_get_cur_charge_cntl_limit(struct thermal_cooling_device *tcd,
					 unsigned long *lvl)
{
	struct chg_thermal_device *tdev =
		(struct chg_thermal_device *)tcd->devdata;
	*lvl = tdev->current_level;
	return 0;
}

/*
 * SW need to override fcc on WLC when therm_wlc_override_fcc is true or when
 * WLC is in PROG mode with a wlc-fcc-thermal-mitigation table. No override
 * when we are offline.
 */
static bool chg_therm_override_fcc(struct chg_drv *chg_drv)
{
	struct chg_thermal_device *ctdev_dcin =
			&chg_drv->thermal_devices[CHG_TERMAL_DEVICE_DC_IN];
	struct chg_thermal_device *ctdev_wlcfcc =
			&chg_drv->thermal_devices[CHG_TERMAL_DEVICE_WLC_FCC];
	bool override = chg_drv->therm_wlc_override_fcc;
	int wlc_online;

	if (!chg_drv->wlc_psy)
		return false;

	wlc_online = GPSY_GET_PROP(chg_drv->wlc_psy, POWER_SUPPLY_PROP_ONLINE);
	override = wlc_online > 0;

	pr_debug("%s: MSC_THERM_FCC wlc_online=%d override=%d, wlcfcc_lvl=%d, dcin_lvl=%d\n",
		__func__,  wlc_online, override, ctdev_wlcfcc->current_level,
		ctdev_dcin->current_level);

	return override;
}

/*
 * Wireless and wired limits are linked when therm_wlc_override_fcc is true
 * and when using WLC_DC. This code disables the the thermal vote on MSC_FCC
 * (b/128350180) when charging from WLC and the override flag is set or
 * when WLC is in PROG_ONLINE and a wlc-fcc-thermal-mitigation table is
 * defined.
 * @return true if applied, false if overriden, negative if error
 */
static int chg_therm_update_fcc(struct chg_drv *chg_drv)
{
	struct chg_thermal_device *tdev =
			&chg_drv->thermal_devices[CHG_TERMAL_DEVICE_FCC];
	bool override_fcc = false;
	int ret, fcc = -1;

	/* restore the thermal vote FCC level (if enabled) */
	override_fcc = chg_therm_override_fcc(chg_drv);
	if (!override_fcc && tdev->current_level > 0) {
		if (tdev->current_level < tdev->thermal_levels)
			fcc = tdev->thermal_mitigation[tdev->current_level];
		else
			fcc = 0;
	}

	/* !override_fcc will restore the fcc thermal limit when set */
	ret = gvotable_cast_int_vote(chg_drv->msc_fcc_votable,
				     THERMAL_DAEMON_VOTER, fcc, fcc != -1);
	if (ret < 0)
		pr_err("%s: MSC_THERM_FCC vote fcc=%d failed ret=%d\n",
		       __func__, fcc, ret);

	pr_debug("%s: MSC_THERM_FCC wlc %sfcc=%d fcc_level=%d ret=%d\n",
		 __func__, override_fcc ? "OVERRIDE " : "", fcc,
		 tdev->current_level, ret);

	return ret < 0 ? ret : !override_fcc;
}

static int chg_set_fcc_charge_cntl_limit(struct thermal_cooling_device *tcd,
					 unsigned long lvl)
{
	struct chg_thermal_device *tdev = (struct chg_thermal_device *)tcd->devdata;
	const bool changed = tdev->current_level != lvl;
	struct chg_drv *chg_drv = tdev->chg_drv;
	int fcc = 0, ret;

	if (lvl < 0 || tdev->thermal_levels <= 0 || lvl > tdev->thermal_levels)
		return -EINVAL;

	tdev->current_level = lvl;
	if (tdev->current_level < tdev->thermal_levels)
		fcc = tdev->thermal_mitigation[tdev->current_level];

	/* NOTE: ret <=0 not changed, ret > 0 changed */
	ret = chg_therm_update_fcc(chg_drv);
	if (changed && ret > 0) {
		const bool chg_disable = fcc == 0;

		pr_info("MSC_THERM_FCC lvl=%d ret=%d fcc=%d disable=%d\n",
			 tdev->current_level, ret, fcc, chg_disable);

		/* apply immediately */
		reschedule_chg_work(chg_drv);
	}

	if (chg_drv->csi_status_votable)
		gvotable_cast_long_vote(chg_drv->csi_status_votable,
					"CSI_STATUS_THERM_FCC",
					CSI_STATUS_System_Thermals,
					tdev->current_level != 0);

	return 0;
}


/*
 * set WLC to PPS_PSY_FIXED_ONLINE from OFFLINE IF the DC_ICL limit is nonzero.
 * TODO: implement with a votable of type ANY called wlc_offline and negative
 * logic.
 *
 * returns the wlc online state or error
 */
static int chg_therm_set_wlc_online(struct chg_drv *chg_drv)
{
	struct power_supply *wlc_psy = chg_drv->wlc_psy;
	union power_supply_propval pval;
	int ret;

	if (!wlc_psy)
		return PPS_PSY_OFFLINE;

	/* use wlc_present due to wlc_online report 1 during wlc_spoof */
	ret = power_supply_get_property(wlc_psy, POWER_SUPPLY_PROP_PRESENT, &pval);
	if (ret < 0 || pval.intval == PPS_PSY_OFFLINE) {
		int dc_icl;

		/* OFFLINE goes to online if dc_icl allows or votable not available */
		dc_icl = gvotable_get_current_int_vote(chg_drv->dc_icl_votable);
		if (dc_icl != 0)
			pval.intval = PPS_PSY_FIXED_ONLINE;

		/* will reset offline just in case */
		ret = power_supply_set_property(wlc_psy, POWER_SUPPLY_PROP_ONLINE,
						&pval);

		pr_debug("%s: pval.intval=%d, dc_icl=%d ret=%d\n", __func__,
			 pval.intval, dc_icl, ret);

		if (ret < 0)
			return ret;
	}

	return pval.intval;
}

/*
 * ->wlc_psy go offline only when the online state matches from_state, thhis
 * extends b/119501863 to the WLC_FCC case.
 * Disabling from WLC_FCC due to thermal reason can follow 2 paths:
 * 1) turn off DC charging and switch to DC_ICL if charging is enabled
 * 2) transition from DC_ICL to disabled UNLESS
 *
 * TODO: implement with a votable of type ANY called wlc_offline and negative
 * logic.
 *
 * returns the wlc online state or error
 */
static int chg_therm_set_wlc_offline(struct chg_drv *chg_drv, int from_state)
{
	struct power_supply *wlc_psy = chg_drv->wlc_psy;
	union power_supply_propval pval;
	int ret;

	if (!wlc_psy)
		return PPS_PSY_OFFLINE;

	ret = power_supply_get_property(wlc_psy, POWER_SUPPLY_PROP_ONLINE, &pval);
	if (ret < 0 || pval.intval == from_state) {
		int dc_icl = -1;

		/*
		 * PROG needs to go to PPS_PSY_FIXED_ONLINE to restart charging
		 * when coming out of DC.
		 */
		pval.intval = PPS_PSY_OFFLINE;
		if (from_state == PPS_PSY_PROG_ONLINE) {
			dc_icl = gvotable_get_current_int_vote(
					chg_drv->dc_icl_votable);
			if (dc_icl != 0)
				pval.intval = PPS_PSY_FIXED_ONLINE;
		}

		ret = power_supply_set_property(wlc_psy, POWER_SUPPLY_PROP_ONLINE,
						&pval);
		pr_debug("%s: pval.intval=%d, dc_icl=%d ret=%d \n", __func__,
			 pval.intval, dc_icl, ret);
		if (ret < 0)
			return ret;
	}

	return pval.intval;
}

/* BPP/HPP/GPP case */
static int chg_set_dc_in_charge_cntl_limit(struct thermal_cooling_device *tcd,
					   unsigned long lvl)
{
	struct chg_thermal_device *tdev =
			(struct chg_thermal_device *)tcd->devdata;
	const bool changed = tdev->current_level != lvl;
	struct chg_drv *chg_drv = tdev->chg_drv;
	int fan_hint = -1, dc_icl = -1;
	int wlc_state, ret = 0;

	if (lvl < 0 || tdev->thermal_levels <= 0 || lvl > tdev->thermal_levels)
		return -EINVAL;

	if (!chg_drv->dc_icl_votable)
		chg_drv->dc_icl_votable =
			gvotable_election_get_handle("DC_ICL");
	if (!chg_drv->tx_icl_votable)
		chg_drv->tx_icl_votable =
			gvotable_election_get_handle("TX_ICL");

	/* dc_icl == -1 on level 0 */
	tdev->current_level = lvl;
	if (tdev->current_level == tdev->thermal_levels)
		dc_icl = 0;
	else if (tdev->current_level != 0)
		dc_icl = tdev->thermal_mitigation[tdev->current_level];

	/* b/119501863 set the wireless charger offline if in FIXED mode */
	if (dc_icl == 0) {
		wlc_state = chg_therm_set_wlc_offline(chg_drv, PPS_PSY_FIXED_ONLINE);
		if (wlc_state < 0)
			pr_err("MSC_THERM_DC cannot offline ret=%d\n", wlc_state);

		if (chg_drv->tx_icl_votable)
			gvotable_cast_int_vote(chg_drv->tx_icl_votable,
					       THERMAL_DAEMON_VOTER, 0, true);

		pr_info("MSC_THERM_DC lvl=%ld, dc disable wlc_state=%d\n",
			lvl, wlc_state);
	}

	/* set the IF-PMIC before re-enable wlc */
	if (chg_drv->dc_icl_votable) {
		ret = gvotable_cast_int_vote(chg_drv->dc_icl_votable,
					     THERMAL_DAEMON_VOTER,
					     dc_icl, dc_icl >= 0);
		if (ret < 0 || changed)
			pr_info("MSC_THERM_DC lvl=%ld dc_icl=%d (%d)\n",
				lvl, dc_icl, ret);
	}

	/* set the wireless to PPS_PSY_FIXED_ONLINE if needed */
	if (dc_icl != 0) {
		wlc_state = chg_therm_set_wlc_online(chg_drv);
		if (wlc_state < 0)
			pr_err("MSC_THERM_DC cannot online ret=%d\n", wlc_state);
		if (chg_drv->tx_icl_votable)
			gvotable_cast_int_vote(chg_drv->tx_icl_votable,
					       THERMAL_DAEMON_VOTER, 0, false);
	}

	/* online/offline or vote might change the selection */
	if (ret == 0)
		chg_therm_update_fcc(chg_drv);

	/* TODO: use a cooling-device dependent logic for fan level */
	if (wlc_state == PPS_PSY_FIXED_ONLINE)
		fan_hint = fan_get_level(tdev);

	ret = fan_vote_level(chg_drv, FAN_VOTER_DC_IN, fan_hint);
	if (ret < 0)
		pr_err("MSC_THERM_DC %s cannot vote on fan_level %d\n",
		       FAN_VOTER_DC_IN, ret);

	if (chg_drv->csi_status_votable)
		gvotable_cast_long_vote(chg_drv->csi_status_votable,
					"CSI_STATUS_THERM_DC_ICL",
					CSI_STATUS_System_Thermals,
					tdev->current_level != 0);

	/* force to apply immediately */
	reschedule_chg_work(chg_drv);
	return 0;
}

/* DC case */
static int chg_set_wlc_fcc_charge_cntl_limit(struct thermal_cooling_device *tcd,
					     unsigned long lvl)
{
	struct chg_thermal_device *tdev =
			(struct chg_thermal_device *)tcd->devdata;
	const bool changed = tdev->current_level != lvl;
	struct chg_drv *chg_drv = tdev->chg_drv;
	int fan_hint = -1, dc_fcc = -1;
	int wlc_state, ret = 0;

	if (lvl < 0 || tdev->thermal_levels <= 0 || lvl > tdev->thermal_levels)
		return -EINVAL;

	if (!chg_drv->dc_fcc_votable) {
		chg_drv->dc_fcc_votable =
			gvotable_election_get_handle("DC_FCC");

		/* HACK: fallback to FCC */
		if (!chg_drv->dc_fcc_votable) {
			chg_drv->dc_fcc_votable = chg_drv->msc_fcc_votable;
			pr_warn("%s: DC_FCC uses msc_fcc votable\n", __func__);
		}
	}

	/* dc_fcc == -1 on level 0 */
	tdev->current_level = lvl;
	if (tdev->current_level == tdev->thermal_levels)
		dc_fcc = 0;
	else if (tdev->current_level != 0)
		dc_fcc = tdev->thermal_mitigation[tdev->current_level];

	/*
	 * Vote before setting the source offline to re-run the selection logic
	 * before taking the WLC to FIXED_ONLINE.
	 */
	if (chg_drv->dc_fcc_votable) {
		ret = gvotable_cast_int_vote(chg_drv->dc_fcc_votable,
					     THERMAL_DAEMON_VOTER,
					     dc_fcc, dc_fcc >= 0);
		if (ret < 0 || changed)
			pr_info("MSC_THERM_DC_FCC lvl=%ld dc_fcc=%d (%d)\n",
				lvl, dc_fcc, ret);
	}

	/*
	 * dc_fcc==0 set WLC to PPS_PSY_FIXED_ONLINE if in PROG mode; dc_fcc!=0
	 * reset WLC to PPS_PSY_FIXED_ONLINE if in OFFLINE unless DC_ICL==0.
	 */
	if (dc_fcc == 0) {
		wlc_state = chg_therm_set_wlc_offline(chg_drv, PPS_PSY_PROG_ONLINE);
		if (wlc_state < 0)
			pr_err("MSC_THERM_DC_FCC cannot offline ret=%d\n", wlc_state);

		pr_info("MSC_THERM_DC_FCC lvl=%ld, dc disable wlc_state=%d\n",
			lvl, wlc_state);
	} else {
		wlc_state = chg_therm_set_wlc_online(chg_drv);
		if (wlc_state < 0)
			pr_err("MSC_THERM_DC_FCC cannot online ret=%d\n", wlc_state);
	}

	/* setting "offline" or online or vote might change the selection */
	if (ret == 0)
		chg_therm_update_fcc(chg_drv);

	/* TODO: use a cooling-device dependent logic for fan level */
	if (wlc_state == PPS_PSY_PROG_ONLINE)
		fan_hint = fan_get_level(tdev);

	ret = fan_vote_level(chg_drv, FAN_VOTER_WLC_FCC, fan_hint);
	if (ret < 0)
		pr_err("MSC_THERM_DC %s cannot vote on fan_level %d\n",
		       FAN_VOTER_WLC_FCC, ret);

	if (chg_drv->csi_status_votable)
		gvotable_cast_long_vote(chg_drv->csi_status_votable,
					"CSI_STATUS_THERM_DC_FCC",
					CSI_STATUS_System_Thermals,
					tdev->current_level != 0);

	/* force to apply immediately */
	reschedule_chg_work(chg_drv);
	return 0;
}

static ssize_t
state2power_table_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct thermal_cooling_device *tdev = to_cooling_device(dev);
	struct chg_thermal_device *mdev = tdev->devdata;
	ssize_t count = 0;
	int i;

	for (i = 0; i < mdev->thermal_levels; i++) {
		const int budgetMw = mdev->thermal_budgets[i] / 1000;

		count += sysfs_emit_at(buf, count, "%u ", budgetMw);
	}

	/* b/231599097 add the implicit 0 at the end of the table */
	count += sysfs_emit_at(buf, count, "0\n");

	return count;
}

static DEVICE_ATTR_RO(state2power_table);

#ifdef CONFIG_DEBUG_FS

static ssize_t tm_store(struct chg_thermal_device *tdev,
			const char __user *user_buf,
			size_t count, loff_t *ppos)
{
	const int thermal_levels = tdev->thermal_levels;
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

		tdev->thermal_budgets[i] = value * 1000;
	}

error_done:
	kfree(tmp);
	return count;
}

static ssize_t fcc_tm_store(struct file *filp, const char __user *user_buf,
			    size_t count, loff_t *ppos)
{
	struct chg_drv *chg_drv = filp->private_data;
	struct chg_thermal_device *tdev =
			&chg_drv->thermal_devices[CHG_TERMAL_DEVICE_FCC];
	int ret;

	ret = tm_store(tdev, user_buf, count, ppos);
	if (ret < 0)
		count = ret;

	return count;
}

DEBUG_ATTRIBUTE_WO(fcc_tm);

static ssize_t dc_tm_store(struct file *filp, const char __user *user_buf,
			   size_t count, loff_t *ppos)
{
	struct chg_drv *chg_drv = filp->private_data;
	struct chg_thermal_device *tdev =
			&chg_drv->thermal_devices[CHG_TERMAL_DEVICE_DC_IN];
	int ret;

	ret = tm_store(tdev, user_buf, count, ppos);
	if (ret < 0)
		count = ret;

	return count;
}

DEBUG_ATTRIBUTE_WO(dc_tm);

#endif // CONFIG_DEBUG_FS

static int chg_init_mdis_stats_map(struct chg_drv *chg_drv, const char *name) {
	int rc, byte_len;

	if (!of_find_property(chg_drv->device->of_node, name, &byte_len)) {
		dev_err(chg_drv->device, "No thermal stats map for %s\n", name);
		return -ENOENT;
	}

	chg_drv->thermal_stats_mdis_levels = devm_kzalloc(chg_drv->device,
							  byte_len,
							  GFP_KERNEL);
	if (!chg_drv->thermal_stats_mdis_levels)
		return -ENOMEM;

	chg_drv->thermal_levels_count = byte_len / sizeof(u32);

	rc = of_property_read_u32_array(chg_drv->device->of_node,
					name,
					chg_drv->thermal_stats_mdis_levels,
					chg_drv->thermal_levels_count);
	if (rc < 0) {
		dev_err(chg_drv->device,
			"Couldn't read limits for %s rc = %d\n", name, rc);
		devm_kfree(chg_drv->device, chg_drv->thermal_stats_mdis_levels);
		chg_drv->thermal_stats_mdis_levels = NULL;
		return -ENODATA;
	}

	return 0;
}

static int chg_tdev_init(struct chg_thermal_device *tdev, const char *name,
			 struct chg_drv *chg_drv)
{
	int rc, byte_len;

	if (!of_find_property(chg_drv->device->of_node, name, &byte_len)) {
		dev_err(chg_drv->device, "No cooling device for %s\n", name);
		return -ENOENT;
	}

	tdev->thermal_mitigation = devm_kzalloc(chg_drv->device, byte_len,
							GFP_KERNEL);
	if (!tdev->thermal_mitigation)
		return -ENOMEM;

	tdev->thermal_levels = byte_len / sizeof(u32);

	rc = of_property_read_u32_array(chg_drv->device->of_node,
			name,
			tdev->thermal_mitigation,
			tdev->thermal_levels);
	if (rc < 0) {
		dev_err(chg_drv->device,
			"Couldn't read limits for %s rc = %d\n", name, rc);
		devm_kfree(chg_drv->device, tdev->thermal_mitigation);
		tdev->thermal_mitigation = NULL;
		return -ENODATA;
	}

	rc = of_property_read_u32(chg_drv->device->of_node,
				   "google,wlc-thermal-dc-fan-alarm",
				   &tdev->therm_fan_alarm_level);
	if (rc < 0)
		tdev->therm_fan_alarm_level = tdev->thermal_levels;

	tdev->chg_drv = chg_drv;

	return 0;
}

static int chg_tdev_budgets_init(struct chg_thermal_device *tdev, const char *name,
				 struct chg_drv *chg_drv)
{
	int rc, byte_len, thermal_levels;

	if (!of_find_property(chg_drv->device->of_node, name, &byte_len)) {
		dev_err(chg_drv->device, "No budgets table for %s\n", name);
		return -ENOENT;
	}

	thermal_levels = byte_len / sizeof(u32);
	if (tdev->thermal_levels != thermal_levels) {
		dev_err(chg_drv->device, "Length of budgets table is incorrect\n");
		return -ENOENT;
	}

	tdev->thermal_budgets = devm_kzalloc(chg_drv->device, byte_len,
						GFP_KERNEL);
	if (!tdev->thermal_budgets)
		return -ENOMEM;

	rc = of_property_read_u32_array(chg_drv->device->of_node,
			name,
			tdev->thermal_budgets,
			tdev->thermal_levels);
	if (rc < 0) {
		dev_err(chg_drv->device,
			"Couldn't read limits for %s rc = %d\n", name, rc);
		devm_kfree(chg_drv->device, tdev->thermal_budgets);
		tdev->thermal_budgets = NULL;
		return -ENODATA;
	}

	tdev->chg_drv = chg_drv;

	return 0;
}

static const struct thermal_cooling_device_ops chg_fcc_tcd_ops = {
	.get_max_state = chg_get_max_charge_cntl_limit,
	.get_cur_state = chg_get_cur_charge_cntl_limit,
	.set_cur_state = chg_set_fcc_charge_cntl_limit,
};

static const struct thermal_cooling_device_ops chg_dc_icl_tcd_ops = {
	.get_max_state = chg_get_max_charge_cntl_limit,
	.get_cur_state = chg_get_cur_charge_cntl_limit,
	.set_cur_state = chg_set_dc_in_charge_cntl_limit,
};

static const struct thermal_cooling_device_ops chg_wlc_fcc_tcd_ops = {
	.get_max_state = chg_get_max_charge_cntl_limit,
	.get_cur_state = chg_get_cur_charge_cntl_limit,
	.set_cur_state = chg_set_wlc_fcc_charge_cntl_limit,
};

static int
chg_thermal_device_register(const char *of_name,
			    const char *tcd_name,
			    struct chg_thermal_device *ctdev,
			    const struct thermal_cooling_device_ops *ops)
{
	struct device_node *cooling_node = NULL;

	cooling_node = of_find_node_by_name(NULL, of_name);
	if (!cooling_node) {
		pr_err("No %s OF node for cooling device\n",
		       of_name);
		return -EINVAL;
	}

	ctdev->tcd = thermal_of_cooling_device_register(cooling_node,
							tcd_name,
							ctdev,
							ops);

	if (IS_ERR_OR_NULL(ctdev->tcd)) {
		pr_err("error registering %s cooling device (%ld)\n",
		       tcd_name, PTR_ERR(ctdev->tcd));
		return -EINVAL;
	}

	return 0;
}

static int chg_thermal_state2power(struct chg_thermal_device *tdev,
				   struct chg_drv *chg_drv,
				   enum chg_thermal_devices device)
{
	int ret;

	/* state and debug */
	ret = device_create_file(&tdev->tcd->device, &dev_attr_state2power_table);
	if (ret)
		pr_info("cound not create state table *(%d)\n", ret);

	if (!chg_drv->debug_entry)
		return 0;

	if (device == CHG_TERMAL_DEVICE_FCC)
		debugfs_create_file("fcc_state2power_table", 0644,
				    chg_drv->debug_entry, chg_drv,
				    &fcc_tm_fops);
	if (device == CHG_TERMAL_DEVICE_DC_IN)
		debugfs_create_file("dc_state2power_table", 0644,
				    chg_drv->debug_entry, chg_drv,
				    &dc_tm_fops);

	return 0;
}

/* ls /dev/thermal/cdev-by-name/ */
static int chg_thermal_device_init(struct chg_drv *chg_drv)
{
	struct chg_thermal_device *ctdev_fcc, *ctdev_dc, *ctdev_wlcfcc;
	int rfcc, rdc, rwfcc;
	bool no_wlc;

	no_wlc = of_property_read_bool(chg_drv->device->of_node, "no-wlc");

	ctdev_fcc = &chg_drv->thermal_devices[CHG_TERMAL_DEVICE_FCC];
	rfcc = chg_tdev_init(ctdev_fcc, "google,thermal-mitigation", chg_drv);
	if (rfcc == 0) {
		int ret;

		ret = chg_tdev_budgets_init(ctdev_fcc,
					    "google,thermal-mitigation-budgets",
					    chg_drv);
		rfcc = chg_thermal_device_register(FCC_OF_CDEV_NAME,
						   FCC_CDEV_NAME,
						   ctdev_fcc,
						   &chg_fcc_tcd_ops);
		if (rfcc) {
			devm_kfree(chg_drv->device,
				   ctdev_fcc->thermal_mitigation);
			ctdev_fcc->thermal_mitigation = NULL;
			if (ret == 0) {
				devm_kfree(chg_drv->device,
					   ctdev_fcc->thermal_budgets);
				ctdev_fcc->thermal_budgets = NULL;
			}
		}

		if (ctdev_fcc->thermal_budgets)
			chg_thermal_state2power(ctdev_fcc, chg_drv,
						CHG_TERMAL_DEVICE_FCC);
	}

	ctdev_dc = &chg_drv->thermal_devices[CHG_TERMAL_DEVICE_DC_IN];

	if (no_wlc) {
		dev_err(chg_drv->device, "No cooling device for wlc\n");
		rdc = -ENOENT;
	} else {
		rdc = chg_tdev_init(ctdev_dc, "google,wlc-thermal-mitigation", chg_drv);
	}
	if (rdc == 0) {
		int ret;

		ret = chg_tdev_budgets_init(ctdev_dc,
					    "google,wlc-thermal-mitigation-budgets",
					    chg_drv);
		rdc = chg_thermal_device_register(WLC_OF_CDEV_NAME,
						  WLC_CDEV_NAME,
						  ctdev_dc,
						  &chg_dc_icl_tcd_ops);
		if (rdc) {
			devm_kfree(chg_drv->device,
				   ctdev_dc->thermal_mitigation);
			ctdev_dc->thermal_mitigation = NULL;
			if (ret == 0) {
				devm_kfree(chg_drv->device,
					   ctdev_dc->thermal_budgets);
				ctdev_dc->thermal_budgets = NULL;
			}
		}

		if (ctdev_dc->thermal_budgets)
			chg_thermal_state2power(ctdev_dc, chg_drv,
						CHG_TERMAL_DEVICE_DC_IN);
	}

	ctdev_wlcfcc = &chg_drv->thermal_devices[CHG_TERMAL_DEVICE_WLC_FCC];
	rwfcc = chg_tdev_init(ctdev_wlcfcc, "google,wlc-fcc-thermal-mitigation",
			      chg_drv);
	if (rwfcc == 0) {
		rwfcc = chg_thermal_device_register(WLC_FCC_OF_CDEV_NAME,
						    WLC_FCC_CDEV_NAME,
						    ctdev_wlcfcc,
						    &chg_wlc_fcc_tcd_ops);
		if (rwfcc) {
			devm_kfree(chg_drv->device,
				   ctdev_wlcfcc->thermal_mitigation);
			ctdev_wlcfcc->thermal_mitigation = NULL;
		}
	}

	chg_drv->therm_wlc_override_fcc =
		of_property_read_bool(chg_drv->device->of_node,
					"google,therm-wlc-overrides-fcc");

	chg_init_mdis_stats_map(chg_drv, "google,thermal-stats-lvl-map");

	pr_info("wlc-overrides-fcc=%d thermal-mitigation=%d "
		"wlc-thermal-mitigation=%d wlc-fcc-thermal-mitigation=%d\n",
		chg_drv->therm_wlc_override_fcc,
		!!ctdev_fcc->thermal_mitigation,
		!!ctdev_dc->thermal_mitigation,
		!!ctdev_wlcfcc->thermal_mitigation);

	return (rfcc | rdc | rwfcc);
}

static struct power_supply *psy_get_by_name(struct chg_drv *chg_drv,
					    const char *name)
{
	struct power_supply *psy;

	psy = power_supply_get_by_name(name);
	if (!psy)
		dev_dbg_ratelimited(chg_drv->device,
				    "failed to get \"%s\" power supply, retrying...\n",
				    name);

	return psy;
}

/* TODO: refactor to use pps_get_tcpm_psy() */
static struct power_supply *get_tcpm_psy(struct chg_drv *chg_drv)
{
	struct power_supply *psy[2];
	int i, ret;
	struct power_supply *tcpm_psy = NULL;

	ret = power_supply_get_by_phandle_array(chg_drv->device->of_node,
						"google,tcpm-power-supply", psy,
						ARRAY_SIZE(psy));
	if (ret < 0 && !chg_drv->usb_skip_probe) {
		dev_dbg_ratelimited(chg_drv->device,
				    "failed to get tcpm power supply, retrying... ret:%d\n",
				    ret);

		return ERR_PTR(ret);
	} else if (ret > 0) {
		for (i = 0; i < ret; i++) {
			if (psy[i]->desc && psy[i]->desc->name && !strncmp(
					psy[i]->desc->name, "tcpm", strlen(
							"tcpm")))
				tcpm_psy = psy[i];
			else
				power_supply_put(psy[i]);
		}

		if (tcpm_psy) {
			chg_drv->tcpm_psy_name = tcpm_psy->desc->name;
			dev_dbg(chg_drv->device, "tcpm psy_name: %s\n", chg_drv->tcpm_psy_name);
		} else {
			dev_dbg_ratelimited(chg_drv->device,
					    "tcpm power supply invalid. retrying ...\n");
		}
	}

	return tcpm_psy;
}

static int chg_get_psy(struct chg_drv *chg_drv, const char *psy_name, struct power_supply **psy)
{
	*psy = NULL;
	if (psy_name && chg_drv->psy_retry_count) {
		*psy = psy_get_by_name(chg_drv, psy_name);
		if (!*psy) {
			chg_drv->psy_retry_count--;
			return -EAGAIN;
		}
	}

	if (!psy || !*psy)
		return -ENODEV;
	return 0;
}

static void google_charger_init_work(struct work_struct *work)
{
	struct chg_drv *chg_drv = container_of(work, struct chg_drv,
					       init_work.work);
	struct power_supply *chg_psy = NULL, *usb_psy = NULL;
	struct power_supply *wlc_psy = NULL, *bat_psy = NULL;
	struct power_supply *ext_psy = NULL, *tcpm_psy = NULL;
	int ret = 0, ret_usb = 0, ret_wlc = 0, ret_ext = 0, ret_tcpm = 0;

	if (!chg_drv->chg_psy && chg_get_psy(chg_drv, chg_drv->chg_psy_name, &chg_psy))
		goto retry_init_work;
	if (!chg_drv->chg_psy)
		chg_drv->chg_psy = chg_psy;

	if (!chg_drv->bat_psy && chg_get_psy(chg_drv, chg_drv->bat_psy_name, &bat_psy))
		goto retry_init_work;
	if (!chg_drv->bat_psy)
		chg_drv->bat_psy = bat_psy;

	if (!chg_drv->usb_psy && chg_drv->usb_psy_name)	/* usb_psy_name is optional */
		ret_usb = chg_get_psy(chg_drv, chg_drv->usb_psy_name, &usb_psy);
	if (!ret_usb && !chg_drv->usb_psy)
		chg_drv->usb_psy = usb_psy;

	if (!chg_drv->wlc_psy && chg_drv->wlc_psy_name) /* wlc_psy_name is optional */
		ret_wlc = chg_get_psy(chg_drv, chg_drv->wlc_psy_name, &wlc_psy);
	if (!ret_wlc && !chg_drv->wlc_psy)
		chg_drv->wlc_psy = wlc_psy;

	if (!chg_drv->ext_psy && chg_drv->ext_psy_name) /* ext_psy_name is optional */
		ret_ext = chg_get_psy(chg_drv, chg_drv->ext_psy_name, &ext_psy);
	if (!ret_ext && !chg_drv->ext_psy)
		chg_drv->ext_psy = ext_psy;

	if (!chg_drv->tcpm_psy && chg_drv->tcpm_phandle && chg_drv->psy_retry_count) {
		tcpm_psy = get_tcpm_psy(chg_drv);
		if (IS_ERR_OR_NULL(tcpm_psy)) {
			tcpm_psy = NULL;
			chg_drv->psy_retry_count--;
			ret_tcpm = -EAGAIN;
		}
	}

	if (!ret_tcpm && !chg_drv->tcpm_psy)
		chg_drv->tcpm_psy = tcpm_psy;

	if (ret_usb == -EAGAIN || ret_wlc == -EAGAIN || ret_ext == -EAGAIN || ret_tcpm == -EAGAIN)
		goto retry_init_work;

	/* TODO: make this optional since we don't need to do this anymore */
	ret = chg_disable_std_votables(chg_drv);
	if (ret == -EPROBE_DEFER)
		goto retry_init_work;

	/* pass logbuffer_bd addr to google_battery to log AACP */
	if (chg_drv->bd_state.bd_log) {
		ret = GPSY_SET_INT64_PROP(chg_drv->bat_psy,
					 GBMS_PROP_LOGBUFFER_BD,
					 chg_drv->bd_state.bd_log);
		if (ret == -EAGAIN)
			goto retry_init_work;
		if (ret < 0)
			pr_info("send logbuffer_bd addr failed %d", ret);
	}

	/* PPS negotiation handled in google_charger */
	if (!chg_drv->tcpm_psy) {
		pr_info("PPS not available\n");
	} else {
		const char *name = chg_drv->tcpm_psy->desc->name;
		const bool pps_enable = of_property_read_bool(chg_drv->device->of_node,
							      "google,pps-enable");

		ret = pps_init(&chg_drv->pps_data, chg_drv->device, chg_drv->tcpm_psy, "gcharger-pps");
		if (ret < 0) {
			pr_err("PPS init failure for %s (%d)\n", name, ret);
		} else if (pps_enable) {
			if (chg_drv->debug_entry)
				pps_init_fs(&chg_drv->pps_data, chg_drv->debug_entry);
			chg_drv->pps_enable = true;
			pr_info("PPS available for %s\n", name);
		} else {
			chg_drv->pps_data.stage = PPS_DISABLED;
			pr_info("PPS not enabled\n");
		}
	}

	ret = chg_thermal_device_init(chg_drv);
	if (ret < 0)
		pr_err("Cannot register thermal devices, ret=%d\n", ret);

	chg_drv->dead_battery = chg_update_dead_battery(chg_drv);
	if (chg_drv->dead_battery)
		pr_info("dead battery mode\n");

	chg_init_state(chg_drv);
	chg_drv->stop_charging = -1;
	chg_drv->charge_stop_level = DEFAULT_CHARGE_STOP_LEVEL;
	chg_drv->charge_start_level = DEFAULT_CHARGE_START_LEVEL;
	chg_drv->charging_policy = CHARGING_POLICY_DEFAULT;
	mutex_init(&chg_drv->stats_lock);
	thermal_stats_init(chg_drv);

	/* reset override charging parameters */
	chg_drv->user_fv_uv = -1;
	chg_drv->user_cc_max = -1;

	/* dock_defend */
	if (chg_drv->ext_psy)
		bd_dd_init(chg_drv);
	/* Initialize stats no matter which device, as soc_in needs to be -1 */
	bd_dd_stats_init(chg_drv);

	chg_drv->psy_nb.notifier_call = chg_psy_changed;
	ret = power_supply_reg_notifier(&chg_drv->psy_nb);
	if (ret < 0)
		pr_err("Cannot register power supply notifer, ret=%d\n", ret);

	chg_drv->init_done = true;
	pr_info("google_charger chg=%d bat=%d wlc=%d usb=%d ext=%d tcpm=%d init_work done\n",
		!!chg_drv->chg_psy, !!chg_drv->bat_psy, !!chg_drv->wlc_psy,
		!!chg_drv->usb_psy, !!chg_drv->ext_psy, !!chg_drv->tcpm_psy);

	/* catch state changes that happened before registering the notifier */
	schedule_delayed_work(&chg_drv->chg_work,
		msecs_to_jiffies(CHG_DELAY_INIT_DETECT_MS));
	return;

retry_init_work:
	schedule_delayed_work(&chg_drv->init_work,
			      msecs_to_jiffies(CHG_DELAY_INIT_MS));
}

/*
 * Requires a charger and a battery. WLC, USB and TCPM are optional.
 */
static int google_charger_probe(struct platform_device *pdev)
{
	const char *chg_psy_name, *bat_psy_name;
	const char *usb_psy_name = NULL, *wlc_psy_name = NULL;
	const char *ext_psy_name = NULL;
	struct chg_drv *chg_drv;
	int ret;

	chg_drv = devm_kzalloc(&pdev->dev, sizeof(*chg_drv), GFP_KERNEL);
	if (!chg_drv)
		return -ENOMEM;

	chg_drv->device = &pdev->dev;

	ret = of_property_read_string(pdev->dev.of_node,
				      "google,chg-power-supply",
				      &chg_psy_name);
	if (ret != 0) {
		pr_err("cannot read google,chg-power-supply, ret=%d\n", ret);
		return -EINVAL;
	}
	chg_drv->chg_psy_name =
	    devm_kstrdup(&pdev->dev, chg_psy_name, GFP_KERNEL);
	if (!chg_drv->chg_psy_name)
		return -ENOMEM;

	ret = of_property_read_string(pdev->dev.of_node,
				      "google,bat-power-supply",
				      &bat_psy_name);
	if (ret != 0) {
		pr_err("cannot read google,bat-power-supply, ret=%d\n", ret);
		return -EINVAL;
	}
	chg_drv->bat_psy_name =
	    devm_kstrdup(&pdev->dev, bat_psy_name, GFP_KERNEL);
	if (!chg_drv->bat_psy_name)
		return -ENOMEM;

	ret = of_property_read_string(pdev->dev.of_node,
				      "google,wlc-power-supply",
				      &wlc_psy_name);
	if (ret != 0)
		pr_warn("google,wlc-power-supply not defined\n");
	if (wlc_psy_name) {
		chg_drv->wlc_psy_name =
		    devm_kstrdup(&pdev->dev, wlc_psy_name, GFP_KERNEL);
		if (!chg_drv->wlc_psy_name)
			return -ENOMEM;
	}

	ret = of_property_read_string(pdev->dev.of_node,
				      "google,ext-power-supply",
				      &ext_psy_name);
	if (ret != 0)
		pr_warn("google,ext-power-supply not defined\n");
	if (ext_psy_name) {
		chg_drv->ext_psy_name =
		    devm_kstrdup(&pdev->dev, ext_psy_name, GFP_KERNEL);
		if (!chg_drv->ext_psy_name)
			return -ENOMEM;
	}

	ret = of_property_read_string(pdev->dev.of_node,
				      "google,usb-power-supply",
				      &usb_psy_name);
	if (ret != 0)
		pr_warn("google,usb-power-supply not defined\n");
	if (usb_psy_name) {
		chg_drv->usb_psy_name =
		    devm_kstrdup(&pdev->dev, usb_psy_name, GFP_KERNEL);
		if (!chg_drv->usb_psy_name)
			return -ENOMEM;
	}

	ret = of_property_read_u32(pdev->dev.of_node,
				   "google,tcpm-power-supply",
				   &chg_drv->tcpm_phandle);
	if (ret < 0)
		pr_warn("google,tcpm-power-supply not defined\n");

	ret = of_property_read_u32(pdev->dev.of_node,
				   "google,psy-retry-count",
				   &chg_drv->psy_retry_count);
	if (ret < 0)
		chg_drv->psy_retry_count = PSY_RETRY_LIMIT;
	else
		pr_info("google,psy-retry-count is %d\n", chg_drv->psy_retry_count);

	/*
	 * when set will reduce the comparison value for ibatt by
	 *         cc_max * (100 - pps_cc_tolerance_pct) / 100
	 *  only used from the PPS policy
	 */
	ret = of_property_read_u32(pdev->dev.of_node,
				   "google,pps-cc-tolerance-pct",
				   &chg_drv->pps_cc_tolerance_pct);
	if (ret < 0)
		chg_drv->pps_cc_tolerance_pct = PPS_CC_TOLERANCE_PCT_DEFAULT;
	else if (chg_drv->pps_cc_tolerance_pct > PPS_CC_TOLERANCE_PCT_MAX)
		chg_drv->pps_cc_tolerance_pct = PPS_CC_TOLERANCE_PCT_MAX;

	/* user fcc, fv uv are bound by battery votes.
	 * set google,disable_votes in battery node to disable battery votes.
	 */
	chg_drv->enable_user_fcc_fv =
		of_property_read_bool(pdev->dev.of_node,
				      "google,enable-user-fcc-fv");
	if (chg_drv->enable_user_fcc_fv)
		pr_info("User can override FCC and FV\n");

	/* NOTE: newgen charging is configured in google_battery */
	ret = chg_init_chg_profile(chg_drv);
	if (ret < 0) {
		pr_err("cannot read charging profile from dt, ret=%d\n", ret);
		return ret;
	}

	if (chg_drv->chg_term.enable) {
		INIT_WORK(&chg_drv->chg_term.work,
			  chg_termination_work);

		if (alarmtimer_get_rtcdev()) {
			alarm_init(&chg_drv->chg_term.alarm,
				   ALARM_BOOTTIME,
				   chg_termination_alarm_cb);
		} else {
			/* keep the driver init even if get alarmtimer fail */
			chg_drv->chg_term.enable = false;
			cancel_work_sync(&chg_drv->chg_term.work);
			pr_err("Couldn't get rtc device\n");
		}
	}

	mutex_init(&chg_drv->bd_lock);
	chg_drv->bd_ws = wakeup_source_register(NULL, "defender");
	if (!chg_drv->bd_ws) {
		pr_err("Failed to register wakeup source\n");
		return -ENODEV;
	}
	INIT_DELAYED_WORK(&chg_drv->bd_work, bd_work);
	bd_init(&chg_drv->bd_state, chg_drv->device);

	INIT_DELAYED_WORK(&chg_drv->init_work, google_charger_init_work);
	INIT_DELAYED_WORK(&chg_drv->chg_work, chg_work);
	INIT_WORK(&chg_drv->chg_psy_work, chg_psy_work);
	platform_set_drvdata(pdev, chg_drv);

	alarm_init(&chg_drv->chg_wakeup_alarm, ALARM_BOOTTIME,
		   google_chg_alarm_handler);

	/* votables and chg_work need a wakeup source */
	chg_drv->chg_ws = wakeup_source_register(NULL, "google-charger");
	if (!chg_drv->chg_ws) {
		pr_err("Failed to register wakeup source\n");
		return -ENODEV;
	}

	/* create the votables before talking to google_battery */
	ret = chg_create_votables(chg_drv);
	if (ret < 0)
		pr_err("Failed to create votables, ret=%d\n", ret);
	else
		chg_init_votables(chg_drv);

	/* sysfs & debug */
	chg_init_fs(chg_drv);

	schedule_delayed_work(&chg_drv->init_work,
			      msecs_to_jiffies(CHG_DELAY_INIT_MS));

	dev_info(chg_drv->device, "probe work done");
	return 0;
}

static int google_charger_remove(struct platform_device *pdev)
{
	struct chg_drv *chg_drv = (struct chg_drv *)platform_get_drvdata(pdev);

	if (chg_drv) {
		power_supply_unreg_notifier(&chg_drv->psy_nb);

		if (chg_drv->chg_term.enable) {
			alarm_cancel(&chg_drv->chg_term.alarm);
			cancel_work_sync(&chg_drv->chg_term.work);
		}

		chg_destroy_votables(chg_drv);

		if (chg_drv->chg_psy)
			power_supply_put(chg_drv->chg_psy);
		if (chg_drv->bat_psy)
			power_supply_put(chg_drv->bat_psy);
		if (chg_drv->usb_psy)
			power_supply_put(chg_drv->usb_psy);
		if (chg_drv->wlc_psy)
			power_supply_put(chg_drv->wlc_psy);
		if (chg_drv->tcpm_psy)
			power_supply_put(chg_drv->tcpm_psy);

		wakeup_source_unregister(chg_drv->bd_ws);
		wakeup_source_unregister(chg_drv->chg_ws);
		wakeup_source_unregister(chg_drv->pps_data.pps_ws);

		alarm_try_to_cancel(&chg_drv->chg_wakeup_alarm);

		if (chg_drv->pps_data.log)
			logbuffer_unregister(chg_drv->pps_data.log);

		if (chg_drv->bd_state.bd_log)
			logbuffer_unregister(chg_drv->bd_state.bd_log);
	}

	return 0;
}

static void google_charger_shutdown(struct platform_device *pdev)
{
	struct chg_drv *chg_drv = (struct chg_drv *)platform_get_drvdata(pdev);

	if (chg_drv)
		power_supply_unreg_notifier(&chg_drv->psy_nb);
}

#ifdef SUPPORT_PM_SLEEP
static int chg_pm_suspend(struct device *dev)
{

	return 0;
}

static int chg_pm_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct chg_drv *chg_drv = platform_get_drvdata(pdev);

	chg_drv->egain_retries = 0;
	reschedule_chg_work(chg_drv);

	return 0;
}

static const struct dev_pm_ops chg_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(chg_pm_suspend, chg_pm_resume)
};
#endif


static const struct of_device_id match_table[] = {
	{.compatible = "google,charger"},
	{},
};
MODULE_DEVICE_TABLE(of, match_table);

static struct platform_driver google_charger_driver = {
	.driver = {
		   .name = "google,charger",
		   .owner = THIS_MODULE,
		   .of_match_table = match_table,
		   .probe_type = PROBE_PREFER_ASYNCHRONOUS,
#ifdef SUPPORT_PM_SLEEP
		   .pm = &chg_pm_ops,
#endif
		   },
	.probe = google_charger_probe,
	.remove = google_charger_remove,
	.shutdown = google_charger_shutdown,
};

module_platform_driver(google_charger_driver);

MODULE_DESCRIPTION("Multi-step battery charger driver");
MODULE_AUTHOR("Thierry Strudel <tstrudel@google.com>");
MODULE_AUTHOR("AleX Pelosi <apelosi@google.com>");
MODULE_LICENSE("GPL");
