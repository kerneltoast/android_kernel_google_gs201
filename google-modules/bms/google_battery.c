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
#include <linux/pm_runtime.h>
#include <linux/platform_device.h>
#include <linux/thermal.h>
#include <linux/slab.h>
#include <linux/rtc.h>
#include "gbms_power_supply.h"
#include "google_bms.h"
#include "google_psy.h"
#include "qmath.h"
#include <crypto/hash.h>

#include <linux/debugfs.h>

#define BATT_DELAY_INIT_MS		250
#define BATT_WORK_FAST_RETRY_CNT	30
#define BATT_WORK_FAST_RETRY_MS		1000
#define BATT_WORK_DEBOUNCE_RETRY_MS	3000
#define BATT_WORK_ERROR_RETRY_MS	1000

#define DEFAULT_BATT_UPDATE_INTERVAL		47000
#define DEFAULT_BATT_DRV_RL_SOC_THRESHOLD	95
#define DEFAULT_BD_TRICKLE_RL_SOC_THRESHOLD	90
#define DEFAULT_BD_TRICKLE_RESET_SEC		(5 * 60)
#define DEFAULT_HIGH_TEMP_UPDATE_THRESHOLD	550
#define DEFAULT_CV_MAX_TEMPERATURE		0

#define DEFAULT_HEALTH_SAFETY_MARGIN	(30 * 60)

#define MSC_ERROR_UPDATE_INTERVAL		5000
#define MSC_DEFAULT_UPDATE_INTERVAL		30000


/* AACR default slope is disabled by default */
#define AACR_START_CYCLE_DEFAULT	400
#define AACR_MAX_CYCLE_DEFAULT		0 /* disabled */

/* AAFV default slope is disabled by default */
#define AAFV_APPLY_MAX_DEFAULT		0 /* disabled */
#define AAFV_MAX_OFFSET_DEFAULT		100
#define AAFV_CLIFF_CYCLE_DEFAULT	-1
#define AAFV_CLIFF_OFFSET_DEFAULT	100

/* AACP */
#define AACP_OPT_OUT_CUT_OFF_DISABLED		-1 /* no cutoff */
#define AACP_OPT_OUT_CUT_OFF_CYCLES_DEFAULT	AACP_OPT_OUT_CUT_OFF_DISABLED

/* qual time is 0 minutes of charge or 0% increase in SOC */
#define DEFAULT_CHG_STATS_MIN_QUAL_TIME		0
#define DEFAULT_CHG_STATS_MIN_DELTA_SOC		0

/* Voters */
#define MSC_LOGIC_VOTER	"msc_logic"
#define SW_JEITA_VOTER	"sw_jeita"
#define RL_STATE_VOTER	"rl_state"
#define MSC_HEALTH_VOTER "chg_health"
#define BPST_DETECT_VOTER "bpst_detect"
#define FCRU_CHARGE_VOTER "fcr_update"

#define UICURVE_MAX	3

/* sync from google/logbuffer.c */
#define LOG_BUFFER_ENTRY_SIZE	256

/* Initial data of history cycle count */
#define HCC_DEFAULT_DELTA_CYCLE_CNT	10

#define BHI_HEALTH_INDEX_DEFAULT	100
#define BHI_MARGINAL_THRESHOLD_DEFAULT	80
#define BHI_NEED_REP_THRESHOLD_DEFAULT	70
#define BHI_CCBIN_INDEX_LIMIT		90
#define BHI_ALGO_FULL_HEALTH		10000
#define BHI_ALGO_ROUND_INDEX		50
#define BHI_CC_MARGINAL_THRESHOLD_DEFAULT	800
#define BHI_CC_NEED_REP_THRESHOLD_DEFAULT	1000
#define BHI_CAPACITY_MIN 		0
#define BHI_CAPACITY_MAX 		0xFFFF
#define BHI_CYCLE_GRACE_DEFAULT		200

#define BHI_ROUND_INDEX(index) 		\
	(((index) + BHI_ALGO_ROUND_INDEX) / 100)

#define DEFAULT_FORCE_FCR_UPDATE_CYCLE	10
#define DEFAULT_FORCE_FRC_UPDATE_FCR_DELTA	0

/* TODO: this is for Adaptive charging, rename */
enum batt_health_ui {
	/* Internal value used when health is cleared via dialog */
	CHG_DEADLINE_DIALOG = -3,
	/* Internal value used when health is settings disabled while running */
	CHG_DEADLINE_SETTING_STOP = -2,
	/* Internal value used when health is settings disabled */
	CHG_DEADLINE_SETTING = -1,
	/* Internal value used when health is cleared via alarms/re-plug */
	CHG_DEADLINE_CLEARED = 0,
};

#if (GBMS_CCBIN_BUCKET_COUNT < 1) || (GBMS_CCBIN_BUCKET_COUNT > 100)
#error "GBMS_CCBIN_BUCKET_COUNT needs to be a value from 1-100"
#endif

struct ssoc_uicurve {
	qnum_t real;
	qnum_t ui;
};

enum batt_rl_status {
	BATT_RL_STATUS_NONE = 0,
	BATT_RL_STATUS_DISCHARGE = -1,
	BATT_RL_STATUS_RECHARGE = 1,
};

#define RL_DELTA_SOC_MAX	8

struct batt_ssoc_rl_state {
	/* rate limiter state */
	qnum_t rl_ssoc_target;
	ktime_t rl_ssoc_last_update;

	/* rate limiter flags */
	bool rl_no_zero;
	int rl_fast_track;
	int rl_track_target;
	/* rate limiter config */
	int rl_delta_max_time;
	qnum_t rl_delta_max_soc;

	int rl_delta_soc_ratio[RL_DELTA_SOC_MAX];
	qnum_t rl_delta_soc_limit[RL_DELTA_SOC_MAX];
	int rl_delta_soc_cnt;

	qnum_t rl_ft_low_limit;
	qnum_t rl_ft_delta_limit;
};

#define SSOC_STATE_BUF_SZ 128
#define SSOC_DELTA 3

struct batt_ssoc_state {
	/* output of gauge data filter */
	qnum_t ssoc_gdf;
	/*  UI Curves */
	int ssoc_curve_type;    /*<0 dsg, >0 chg, 0? */
	struct ssoc_uicurve ssoc_curve[UICURVE_MAX];
	qnum_t ssoc_uic;
	/* output of rate limiter */
	qnum_t ssoc_rl;
	struct batt_ssoc_rl_state ssoc_rl_state;
	int ssoc_delta;

	/* output of rate limiter */
	int rl_rate;
	int rl_last_ssoc;
	ktime_t rl_last_update;

	/* connected or disconnected */
	ktime_t disconnect_time;
	int buck_enabled;

	/* recharge logic */
	int rl_soc_threshold;
	enum batt_rl_status rl_status;

	/* trickle defender */
	bool bd_trickle_enable;
	int bd_trickle_recharge_soc;
	int bd_trickle_cnt;
	bool bd_trickle_dry_run;
	bool bd_trickle_full;
	bool bd_trickle_eoc;
	u32 bd_trickle_reset_sec;

	/* buff */
	char ssoc_state_cstr[SSOC_STATE_BUF_SZ];

	/* Save/Restore fake capacity */
	bool save_soc_available;
	u16 save_soc;

	/* adjust SOC */
	int point_full_ui_soc;
};

struct gbatt_ccbin_data {
	u16 count[GBMS_CCBIN_BUCKET_COUNT];
	char cyc_ctr_cstr[GBMS_CCBIN_CSTR_SIZE];
	struct mutex lock;
	int prev_soc;
};

#define DEFAULT_RES_TEMP_LOW	350
#define DEFAULT_RES_TEMP_HIGH	390
#define DEFAULT_RAVG_SOC_LOW	5
#define DEFAULT_RAVG_SOC_HIGH	75
#define DEFAULT_RES_FILT_LEN	10

struct batt_res {
	bool estimate_requested;

	/* samples */
	int sample_accumulator;
	int sample_count;

	/* registers */
	int filter_count;
	int resistance_avg;

	/* configuration */
	int estimate_filter;
	int ravg_soc_low;
	int ravg_soc_high;
	int res_temp_low;
	int res_temp_high;
};

/* TODO: move single cell disconnect to bhi_data */
enum bpst_batt_status {
	BPST_BATT_UNKNOWN = 0,
	BPST_BATT_CONNECT = 1,
	BPST_BATT_DISCONNECT = 2,
	BPST_BATT_CELL_FAULT = 3,
};

struct batt_bpst {
	struct mutex lock;
	bool bpst_enable;
	bool bpst_detect_disable;
	bool bpst_cell_fault;
	/* single battery disconnect status */
	int bpst_sbd_status;
	int bpst_count_threshold;
	int bpst_chg_rate;
	u8 bpst_count;
};

#define DEV_SN_LENGTH 20
#define PAIRING_RETRIES 20
enum batt_paired_state {
	BATT_PAIRING_WRITE_ERROR = -4,
	BATT_PAIRING_READ_ERROR = -3,
	BATT_PAIRING_MISMATCH = -2,
	BATT_PAIRING_DISABLED = -1,
	BATT_PAIRING_ENABLED = 0,
	BATT_PAIRING_PAIRED = 1,
	BATT_PAIRING_RESET = 2,
};

enum batt_lfcollect_status {
	BATT_LFCOLLECT_NOT_AVAILABLE = 0,
	BATT_LFCOLLECT_ENABLED = 1,
	BATT_LFCOLLECT_COLLECT = 2,
};

enum batt_aacp_state {
	BATT_AA_STATE_UNAVAILABLE	= -1,
	BATT_AA_STATE_DISABLED		= 0,
	BATT_AA_STATE_ENABLED		= 1,
};

enum batt_aacr_state {
	BATT_AACR_UNKNOWN = -1,
	BATT_AACR_DISABLED = 0,
	BATT_AACR_ENABLED = 1,
	BATT_AACR_MAX,
};

enum batt_aacr_algo {
	BATT_AACR_ALGO_UP_B  = 2, 	  /* reference is the lower limit */
	BATT_AACR_ALGO_REFERENCE_CAP = 3, /* reference is the charge capacity */
	BATT_AACR_ALGO_LOW_B = 4,	  /* reference is the upper limit */
	BATT_AACR_ALGO_DEFAULT = BATT_AACR_ALGO_UP_B,
};

enum batt_aacr_phase {
	BATT_AACR_INVALID_CAP = -3,
	BATT_AACR_UNDER_CYCLES = -2,
};

enum batt_aafv_state {
	BATT_AAFV_UNKNOWN = -1,
	BATT_AAFV_DISABLED = 0,
	BATT_AAFV_ENABLED = 1,
	BATT_AAFV_MAX,
};

enum batt_aact_state {
	BATT_AACT_UNKNOWN = -1,
	BATT_AACT_DISABLED = 0,
	BATT_AACT_ENABLED = 1,
	BATT_AACT_MAX,
};

enum batt_aacp_opt_out {
	BATT_AACP_OPT_OUT_DISABLED = 0,
	BATT_AACP_OPT_OUT_ENABLED = 1,
	BATT_AACP_OPT_OUT_MAX,
};

enum batt_fcr_update_ops {
	BATT_FCR_UPDATE_DISABLE = 0,
	BATT_FCR_UPDATE_FULLCHARGE_DELTA = 1,
};

#define BATT_TEMP_RECORD_THR 3
/* discharge saved after charge */
#define SD_CHG_START 0
#define SD_DISCHG_START BATT_TEMP_RECORD_THR
#define BATT_SD_SAVE_SIZE (BATT_TEMP_RECORD_THR * 2)
#define BATT_SD_MAX_HOURS 15120 /* 90 weeks */

/* TODO: move swelling_data to bhi_data  */
struct swelling_data {
	/* Time in different temperature */
	bool is_enable;
	u32 temp_thr[BATT_TEMP_RECORD_THR];
	u32 soc_thr[BATT_TEMP_RECORD_THR];
	/*
	 * cumulative time array format:
	 * | saved  | 0                | 1                | 2                |
	 * |--------| Charge           | Charge           | Charge           |
	 * | Content| > 30degC & > 90% | > 35degC & > 90% | > 40degC & > 95% |
	 *
	 * | saved  | 3                | 4                | 5                |
	 * |--------| Discharge        | Discharge        | Discharge        |
	 * | Content| > 30degC & > 90% | > 35degC & > 90% | > 40degC & > 95% |
	 */
	u16 saved[BATT_SD_SAVE_SIZE];
	ktime_t chg[BATT_TEMP_RECORD_THR];
	ktime_t dischg[BATT_TEMP_RECORD_THR];
	ktime_t last_update;
};


struct bhi_weight bhi_w[] = {
	[BHI_ALGO_ACHI] = {100, 0, 0},
	[BHI_ALGO_ACHI_B] = {100, 0, 0},
	[BHI_ALGO_ACHI_RECAL] = {100, 0, 0},
	[BHI_ALGO_ACHI_RAVG_B] = {100, 0, 0},
	[BHI_ALGO_MIX_N_MATCH] = {90, 0, 10},
	[BHI_ALGO_ACHI_FCR] = {100, 0, 0},
	[BHI_ALGO_ACHI_CARETAKER] = {100, 0, 0},
};

struct bm_date {
	u8 bm_y;
	u8 bm_m;
	u8 bm_d;
	u8 reserve;
};

#define BHI_TREND_POINTS_SIZE 10
struct bhi_capacity_bound
{
	u16 limit[BHI_TREND_POINTS_SIZE];
	u16 trigger[BHI_TREND_POINTS_SIZE];
};

struct bhi_data
{
	/* context */
	int cycle_count;		/* from the FG */
	int battery_age;		/* from the FG, time in field */

	/* capacity metrics */
	int pack_capacity;		/* mAh, from the FG or from charge table */
	int capacity_fade;		/* calculated from battery history fullcapnom */
	int capacity_fade_fcr;		/* calculated from battery history fullcaprep */

	/* impedance */
	u32 act_impedance;		/* resistance, qualified */
	u32 cur_impedance;		/* resistance, qualified */
	struct batt_res res_state;	/* google_resistance */

	/* swell probability */
	int swell_cumulative;		/* from swell data */
	int ccbin_index;		/* from SOC residency */

	/* battery manufacture and activation date */
	struct bm_date bm_date;		/* from eeprom SN */
	u8 act_date[BATT_EEPROM_TAG_XYMD_LEN];
	int first_usage_date;

	/* set trend points and boundaries */
	u16 trend[BHI_TREND_POINTS_SIZE];
	struct bhi_capacity_bound lower_bound;
	struct bhi_capacity_bound upper_bound;
};

struct health_data
{
	/* current algorithm */
	int bhi_algo;
	int bhi_w_ci;
	int bhi_w_pi;
	int bhi_w_sd;
	/* current health index and status */
	int bhi_index;
	enum bhi_status bhi_status;
	int marginal_threshold;
	int need_rep_threshold;
	/* cycle count threshold */
	int cycle_count_marginal_threshold;
	int cycle_count_need_rep_threshold;
	/* current health metrics */
	int bhi_cap_index;
	int bhi_imp_index;
	int bhi_sd_index;
	/* debug health metrics */
	int bhi_debug_cycle_count;
	int bhi_debug_cap_index;
	int bhi_debug_imp_index;
	int bhi_debug_sd_index;
	int bhi_debug_health_index;
	int bhi_debug_health_status;
	/* algo BHI_ALGO_INDI capacity threshold */
	int bhi_indi_cap;
	/* algo BHI_ALGO_ACHI_B bounds check */
	int bhi_cycle_grace;

	/* current battery state */
	struct bhi_data bhi_data;

	/* recalibration */
	u8 cal_mode;
	u8 cal_state;
	int cal_target;

	/* algo BHI_ALGO_ACHI_CARETAKER status */
	enum bhi_status caretaker_status;
};

#define POWER_METRICS_MAX_DATA	50

struct power_metrics_data {
	unsigned long charge_count;
	unsigned long voltage;
	ktime_t time;
};

struct power_metrics {
	unsigned int polling_rate;
	unsigned int interval;
	unsigned int idx;
	struct power_metrics_data data[POWER_METRICS_MAX_DATA];
	struct delayed_work work;
};

#define CSI_THERMAL_SEVERITY_MAX 5
struct csi_stats {
	int ssoc;

	int csi_speed_min;
	int csi_speed_max;

	int csi_current_status;
	int csi_current_type;

	ktime_t csi_time_sum;
	int speed_sum;

	ktime_t last_update;

	uint8_t ad_type;
	uint8_t ad_voltage;
	uint8_t ad_amperage;
	uint16_t ssoc_in;
	uint16_t ssoc_out;
	ktime_t time_sum;
	ktime_t time_effective;
	ktime_t time_stat_last_update;
	uint16_t aggregate_status;
	uint16_t aggregate_type;
	int8_t temp_min;
	int8_t temp_max;
	uint16_t vol_in;
	uint16_t vol_out;
	uint16_t cc_in;
	uint16_t cc_out;
	ktime_t thermal_severity[CSI_THERMAL_SEVERITY_MAX];
	int thermal_lvl_max;
	int thermal_lvl_min;
};

#define TEMP_SAMPLE_SIZE 5
struct batt_temp_filter {
	struct delayed_work work;
	struct mutex lock;
	bool enable;
	bool force_update;
	bool resume_delay;
	int sample[TEMP_SAMPLE_SIZE];
	int default_interval;
	int fast_interval;
	int resume_delay_time;
	int last_idx;
};
#define NB_FAN_BT_LIMITS 4
/* battery driver state */
struct batt_drv {
	struct device *device;
	struct power_supply *psy;

	const char *fg_psy_name;
	struct power_supply *fg_psy;
	struct notifier_block fg_nb;

	struct delayed_work init_work;
	struct delayed_work batt_work;

	struct wakeup_source *msc_ws;
	struct wakeup_source *batt_ws;
	struct wakeup_source *taper_ws;
	struct wakeup_source *poll_ws;
	bool hold_taper_ws;

	/* TODO: b/111407333, will likely need to adjust SOC% on wakeup */
	bool init_complete;
	bool resume_complete;
	bool batt_present;
	u32 fake_battery_present;

	struct mutex batt_lock;
	struct mutex chg_lock;
	struct mutex hda_tz_lock;

	/* battery work */
	int fg_status;
	int batt_fast_update_cnt;
	u32 batt_update_interval;
	/* update high temperature in time */
	int batt_temp;
	u32 batt_update_high_temp_threshold;
	/* max temperature for cv mode before stopping charging*/
	u32 cv_max_temp;
	/* fake battery temp for thermal testing */
	int fake_temp;
	/* triger for recharge logic next update from charger */
	bool batt_full;
	struct batt_ssoc_state ssoc_state;
	/* bin count */
	struct gbatt_ccbin_data cc_data;
	/* fg cycle count */
	int cycle_count;
	/* for testing */
	int fake_aacp_cc;

	/* props */
	int soh;
	int fake_capacity;
	int batt_health;	/* health of battery, triggers defender UI */
	int report_health;	/* log health changes for debug */
	bool dead_battery;
	int capacity_level;
	bool chg_done;
	int vbatt_crit_deadline_sec;

	/* temp outside the charge table */
	int jeita_stop_charging;
	/* health based charging */
	struct batt_chg_health chg_health;

	/* MSC charging */
	u32 battery_capacity;	/* in mAh */
	struct gbms_chg_profile chg_profile;
	union gbms_charger_state chg_state;

	int temp_idx;
	int vbatt_idx;
	int profile_vbatt_idx;
	int checked_cv_cnt;
	int checked_ov_cnt;
	int checked_tier_switch_cnt;
	int last_log_cnt;

	int fv_uv;
	int cc_max;
	int topoff;
	int msc_update_interval;
	int cc_max_pullback;

	bool disable_votes;
	struct gvotable_election *msc_interval_votable;
	struct gvotable_election *fcc_votable;
	struct gvotable_election *fv_votable;
	struct gvotable_election *temp_dryrun_votable;
	struct gvotable_election *msc_last_votable;
	struct gvotable_election *point_full_ui_soc_votable;

	/* FAN level */
	struct gvotable_election *fan_level_votable;

	/* stats */
	int msc_state;
	int msc_irdrop_state;
	struct mutex stats_lock;
	struct gbms_charging_event ce_data;
	struct gbms_charging_event ce_qual;
	uint32_t chg_sts_qual_time;
	uint32_t chg_sts_delta_soc;

	/* health charge margin time */
	int health_safety_margin;

	/* time to full */
	struct batt_ttf_stats ttf_stats;
	bool ttf_debounce;
	ktime_t ttf_est;

	/* logging */
	struct logbuffer *ssoc_log;

	/* thermal */
	struct thermal_zone_device *tz_dev;

	/* battery virtual sensor */
	struct thermal_zone_device *batt_vs_tz;
	struct thermal_zone_device *batt_vs_mp_tz;
	struct thermal_zone_device *batt_vs_hda_tz;
	struct gvotable_election *hda_tz_votable;
	int hda_tz_limit;
	int hda_tz_vote;
	int batt_vs_w;
	int soc_mp_limit_low;
	int soc_mp_limit_high;
	int therm_mp_limit;
	int max_ratio_mp_limit;
	bool mp_debounce;
	bool need_mp;

	/* used to detect battery replacements and reset statistics */
	enum batt_paired_state pairing_state;
	char dev_sn[DEV_SN_LENGTH];
	uint8_t pairing_state_retry_cnt;

	/* collect battery history/lifetime data (history) */
	enum batt_lfcollect_status blf_state;
	u32 blf_collect_now;
	int hist_delta_cycle_cnt;
	int hist_data_max_cnt;
	int hist_data_saved_cnt;
	void *hist_data;

	/* Battery device info */
	u8 dev_info_check[GBMS_DINF_LEN];

	/* History Device */
	struct gbms_storage_device *history;

	/* Fan control */
	int fan_bt_limits[NB_FAN_BT_LIMITS];

	/* AACR: Aged Adjusted Charging Rate */
	enum batt_aacr_state aacr_state;
	enum batt_aacr_phase aacr_phase;
	int aacr_cycle_grace;
	int aacr_cycle_max;
	int aacr_algo;
	int aacr_min_capacity_rate;
	int aacr_cliff_capacity_rate;

	/* AAFV: Aged Adjusted Float Voltage */
	enum batt_aafv_state aafv_state;
	int aafv_apply_max;
	int aafv_max_offset;
	int aafv_cliff_cycle;
	int aafv_cliff_offset;
	struct logbuffer *bd_log;

	/* AACT: Aged Adjusted Charge Table */
	enum batt_aact_state aact_state;

	/* AACP: Configuration Version for AAFV, AACR and AACT for the device */
	int aacp_version;
	/* this is used for opt_cout cutoff */
	enum batt_aacp_opt_out aacp_opt_out;
	struct mutex aacp_state_lock;
	int aacp_opt_out_cut_off_cycles;
	int aacp_health_over_cliff;
	int aacp_health_over_cutoff;
	int aacp_health_last_entry;

	/* AACC: Age adjusted cycle count */
	int aacc;

	/* BHI: updated on disconnect, EOC */
	struct health_data health_data;
	struct swelling_data sd;

	/* CSI: charging speed */
	struct csi_stats csi_stats;
	struct gvotable_election *csi_status_votable;
	int csi_current_status;
	struct gvotable_election *csi_type_votable;
	int csi_current_type;
	int csi_current_speed;
	int fake_charging_speed;
	struct gvotable_election *thermal_level_votable;

	/* battery power metrics */
	struct power_metrics power_metrics;

	/* battery pack status */
	struct batt_bpst bpst_state;

	/* irdrop for DC */
	bool dc_irdrop;

	bool pullback_current;
	bool allow_higher_fv;

	/* shutdown flag */
	int boot_to_os_attempts;

	/* battery critical level */
	int batt_critical_voltage;

	/* battery temperature filter */
	struct batt_temp_filter temp_filter;

	/* charging policy */
	struct gvotable_election *charging_policy_votable;
	int charging_policy;

	int batt_id;

	/* for testing drain battery not shutdown */
	int restrict_level_critical;

	/* force charge to 100% even with charge limit to update FCR */
	int force_fcr_update_cycle;
	int last_full_charge;
	bool vote_force_full_charge;

	int force_fcr_update_ops;
};

static int gbatt_get_temp(struct batt_drv *batt_drv, int *temp);

static int gbatt_get_capacity(struct batt_drv *batt_drv);
static int ssoc_get_capacity(const struct batt_ssoc_state *ssoc);
static int batt_ttf_estimate(ktime_t *res, struct batt_drv *batt_drv);

static int gbatt_restore_capacity(struct batt_drv *batt_drv);
static bool aafv_update_float_voltage(struct batt_drv *batt_drv, int *fv_uv);

static bool aafv_enabled(const int state, const int opt_out)
{
	switch(state) {
		case BATT_AAFV_UNKNOWN:
			return false;
		case BATT_AAFV_ENABLED:
			return true;
		case BATT_AAFV_DISABLED:
			return !opt_out;
		default:
			return false;
	}
}

static bool aact_enabled(const int state, const int opt_out)
{
	switch(state) {
		case BATT_AACT_UNKNOWN:
			return false;
		case BATT_AACT_ENABLED:
			return true;
		case BATT_AACT_DISABLED:
			return !opt_out;
		default:
			return false;
	}
}

static bool aacr_enabled(const int state, const int opt_out)
{
	switch(state) {
		case BATT_AACR_UNKNOWN:
			return false;
		case BATT_AACR_ENABLED:
			return true;
		case BATT_AACR_DISABLED:
			return !opt_out;
		default:
			return false;
	}
}

static int batt_get_filter_temp(struct batt_temp_filter *temp_filter)
{
	int sum = 0, max, min, i;

	mutex_lock(&temp_filter->lock);
	max = min = temp_filter->sample[0];
	for (i = 0; i < TEMP_SAMPLE_SIZE; i++) {
		if (temp_filter->sample[i] > max)
			max = temp_filter->sample[i];
		if (temp_filter->sample[i] < min)
			min = temp_filter->sample[i];
		sum += temp_filter->sample[i];
	}
	mutex_unlock(&temp_filter->lock);

	return (sum - max - min) / (TEMP_SAMPLE_SIZE - 2);
}

static int gbatt_get_raw_temp(struct batt_drv *batt_drv, int *temp)
{
	int err = 0;
	union power_supply_propval val;

	if (batt_drv->temp_filter.enable) {
		*temp = batt_get_filter_temp(&batt_drv->temp_filter);
		return err;
	}

	if (!batt_drv->fg_psy)
		return -EINVAL;

	err = power_supply_get_property(batt_drv->fg_psy, POWER_SUPPLY_PROP_TEMP, &val);
	if (err == 0)
		*temp = val.intval;

	return err;
}

static inline void batt_update_cycle_count(struct batt_drv *batt_drv)
{
	const int ret = GPSY_GET_PROP(batt_drv->fg_psy, POWER_SUPPLY_PROP_CYCLE_COUNT);

	if (ret >= 0)
		batt_drv->cycle_count = ret;
	else
		dev_warn(batt_drv->device, "Failed to get cycle count (%d)\n", ret);
}

static int google_battery_tz_get_cycle_count(struct thermal_zone_device *tz, int *cycle_count)
{
	struct batt_drv *batt_drv = (struct batt_drv *)tz->devdata;

	if (!cycle_count) {
		pr_err("Cycle Count NULL");
		return -EINVAL;
	}

	if (batt_drv->cycle_count < 0)
		return batt_drv->cycle_count;

	*cycle_count = batt_drv->cycle_count;

	return 0;
}

static int batt_vs_tz_get(struct thermal_zone_device *tzd, int *batt_vs)
{
	struct batt_drv *batt_drv = tzd->devdata;
	int temp, rc;
	unsigned int ibat;
	unsigned long vs_tmp;

	if (!batt_vs)
		return -EINVAL;

	rc = gbatt_get_raw_temp(batt_drv, &temp);
	if (rc)
		return -EINVAL;

	temp = temp * 100;

	ibat = abs(GPSY_GET_INT_PROP(batt_drv->fg_psy, POWER_SUPPLY_PROP_CURRENT_AVG, &rc));
	if (rc)
		return -EINVAL;

	vs_tmp = div64_u64(mul_u32_u32(ibat, ibat) * batt_drv->batt_vs_w, 1000000000000);

	*batt_vs = temp - vs_tmp;

	return 0;
}

static struct thermal_zone_device_ops batt_vs_tz_ops = {
	.get_temp = batt_vs_tz_get,
};

#define SOC_MP_LIMIT_LOW	(50)
#define SOC_MP_LIMIT_HIGH	(80)
#define THERM_MP_LIMIT		(0)
#define MAX_RATIO_MP_LIMIT	(70)

static int batt_get_thermal_level(struct batt_drv *batt_drv)
{
	int thermal_level = -1;
	if (!batt_drv->thermal_level_votable)
		batt_drv->thermal_level_votable = gvotable_election_get_handle(VOTABLE_THERMAL_LVL);
	if (batt_drv->thermal_level_votable)
		thermal_level = gvotable_get_current_int_vote(batt_drv->thermal_level_votable);
	return thermal_level;
}

static bool batt_mp_adapter_qual(struct batt_drv *batt_drv)
{
	union gbms_ce_adapter_details ad;
	int demand = (batt_drv->cc_max * 75) / 100;
	int adapter_cap;

	/* can adapter provide 75% of demand */
	ad.v = batt_drv->ce_data.adapter_details.v;
	adapter_cap = ad.ad_amperage * 100000;
	if (adapter_cap <= demand) {
		dev_dbg(batt_drv->device, "%s: adapter power insuff: capability: %d, demand: %d\n",
			__func__, adapter_cap, demand);
		return false;
	}
	return true;
}

static bool batt_mp_ttf_qual(struct batt_drv *batt_drv)
{
	ktime_t res = 0;
	const int max_ratio = batt_ttf_estimate(&res, batt_drv);

	if (max_ratio < MAX_RATIO_MP_LIMIT) {
		dev_dbg(batt_drv->device, "%s: max_ratio under limit: max_ratio: %d, limit: %d\n",
			__func__, max_ratio, MAX_RATIO_MP_LIMIT);
		return false;
	}

	return true;
}

/* returns true if need more power allocation for batt charging */
static bool batt_needs_more_power(struct batt_drv *batt_drv, int ibatt)
{
	int capacity, thermal_level;

	dev_dbg(batt_drv->device, "%s: Start. \n", __func__);
	if (batt_drv->chg_state.f.chg_status != POWER_SUPPLY_STATUS_CHARGING) {
		dev_dbg(batt_drv->device, "%s: Status not = CHARGING %d\n", __func__,
			batt_drv->chg_state.f.chg_status);
		goto done;
	}

	if (batt_drv->chg_health.rest_state != CHG_HEALTH_DISABLED) {
		dev_dbg(batt_drv->device, "%s: rest state not _DISABLED %d\n", __func__,
			batt_drv->chg_health.rest_state);
		goto done;
	}

	if (ibatt > batt_drv->cc_max / 10) {
		dev_dbg(batt_drv->device, "%s: current less than 10 percent demand ibatt: %d, cc_max: %d\n",
			__func__, ibatt, batt_drv->cc_max);
		goto done;
	}

	thermal_level = batt_get_thermal_level(batt_drv);
	if (thermal_level > batt_drv->therm_mp_limit) {
		dev_dbg(batt_drv->device, "%s: thermal level under limit lvl: %d, limit: %d\n",
			__func__, thermal_level, batt_drv->therm_mp_limit);
		goto done;
	}

	capacity = gbatt_get_capacity(batt_drv);
	if (capacity >= batt_drv->soc_mp_limit_high)
		batt_drv->mp_debounce = true;
	else if (capacity <= batt_drv->soc_mp_limit_low)
		batt_drv->mp_debounce = false;
	if (batt_drv->mp_debounce) {
		dev_dbg(batt_drv->device, "%s: in capacity debounce capacity[now:%d, low:%d, high:%d]\n",
			__func__, capacity, batt_drv->soc_mp_limit_low, batt_drv->soc_mp_limit_high);
		goto done;
	}

	if (!batt_mp_adapter_qual(batt_drv))
		goto done;

	if (!batt_mp_ttf_qual(batt_drv))
		goto done;

	dev_dbg(batt_drv->device, "%s: Need more power\n", __func__);
	return true;
done:
	dev_dbg(batt_drv->device, "%s: Don't need more power\n", __func__);
	return false;
}

static int batt_vs_mp_tz_get(struct thermal_zone_device *tzd, int *batt_vs)
{
	struct batt_drv *batt_drv = tzd->devdata;

	if (!batt_vs)
		return -EINVAL;

	*batt_vs = batt_drv->need_mp;
	return 0;
}

static struct thermal_zone_device_ops batt_vs_mp_tz_ops = {
	.get_temp = batt_vs_mp_tz_get,
};

static int hda_tz_cb(struct gvotable_election *el,
			      const char *reason, void *vote)
{
	struct batt_drv *batt_drv = gvotable_get_data(el);

	mutex_lock(&batt_drv->hda_tz_lock);
	batt_drv->hda_tz_vote = GVOTABLE_PTR_TO_INT(vote);
	dev_dbg(batt_drv->device, "%s reason: %s, vote: %d\n", __func__,
		reason, batt_drv->hda_tz_vote);
	mutex_unlock(&batt_drv->hda_tz_lock);
	return 0;
}

static int batt_vs_hda_tz_get(struct thermal_zone_device *tzd, int *batt_vs)
{
	struct batt_drv *batt_drv = tzd->devdata;

	if (!batt_vs)
		return -EINVAL;

	mutex_lock(&batt_drv->hda_tz_lock);
	if (!batt_drv->hda_tz_limit)
		*batt_vs = batt_drv->hda_tz_vote;
	else
		*batt_vs = batt_drv->hda_tz_vote >= batt_drv->hda_tz_limit ? 1 : 0;
	mutex_unlock(&batt_drv->hda_tz_lock);
	return 0;
}

static struct thermal_zone_device_ops batt_vs_hda_tz_ops = {
	.get_temp = batt_vs_hda_tz_get,
};

static int psy_changed(struct notifier_block *nb,
		       unsigned long action, void *data)
{
	struct power_supply *psy = data;
	struct batt_drv *batt_drv = container_of(nb, struct batt_drv, fg_nb);

	pr_debug("name=%s evt=%lu\n", psy->desc->name, action);

	if ((action != PSY_EVENT_PROP_CHANGED) ||
	    (psy == NULL) || (psy->desc == NULL) || (psy->desc->name == NULL))
		return NOTIFY_OK;

	if (action == PSY_EVENT_PROP_CHANGED &&
	    (!strcmp(psy->desc->name, batt_drv->fg_psy_name))) {
		mod_delayed_work(system_wq, &batt_drv->batt_work, 0);
	}

	return NOTIFY_OK;
}

/* ------------------------------------------------------------------------- */


#define BATT_PRLOG_DEBUG  0
#define BATT_PRLOG_ALWAYS 1
#define BATT_PRLOG_LAST_LOG_COUNT 10

static int debug_printk_prlog = LOGLEVEL_INFO;

static inline int batt_prlog_level(bool level)
{
	return level ? BATT_PRLOG_ALWAYS : BATT_PRLOG_DEBUG;
}

__printf(2,3)
static void batt_prlog__(int info_level, const char *format, ...)
{
	const int level = info_level == BATT_PRLOG_ALWAYS ? LOGLEVEL_INFO : LOGLEVEL_DEBUG;

	if (level <= debug_printk_prlog) {
		va_list args;

		va_start(args, format);
			vprintk(format, args);
		va_end(args);
	}

}

#define batt_prlog(l, fmt, ...) batt_prlog__(l, pr_fmt(fmt), ##__VA_ARGS__)

/* ------------------------------------------------------------------------- */

#define SSOC_TRUE 15
#define SSOC_SPOOF 95
#define SSOC_FULL 100
#define UICURVE_BUF_SZ	(UICURVE_MAX * 15 + 1)
#define SSOC_HIGH_SOC 90

enum ssoc_uic_type {
	SSOC_UIC_TYPE_DSG  = -1,
	SSOC_UIC_TYPE_NONE = 0,
	SSOC_UIC_TYPE_CHG  = 1,
};

const qnum_t ssoc_point_true = qnum_rconst(SSOC_TRUE);
const qnum_t ssoc_point_spoof = qnum_rconst(SSOC_SPOOF);
const qnum_t ssoc_point_full = qnum_rconst(SSOC_FULL);

static struct ssoc_uicurve chg_curve[UICURVE_MAX] = {
	{ ssoc_point_true, ssoc_point_true },
	{ ssoc_point_spoof, ssoc_point_spoof },
	{ ssoc_point_full, ssoc_point_full },
};

static struct ssoc_uicurve dsg_curve[UICURVE_MAX] = {
	{ ssoc_point_true, ssoc_point_true },
	{ ssoc_point_spoof, ssoc_point_full },
	{ ssoc_point_full, ssoc_point_full },
};

static char *ssoc_uicurve_cstr(char *buff, size_t size,
			       struct ssoc_uicurve *curve)
{
	int i, len = 0;

	for (i = 0; i < UICURVE_MAX ; i++) {
		len += scnprintf(&buff[len], size - len,
				"[" QNUM_CSTR_FMT " " QNUM_CSTR_FMT "]",
				qnum_toint(curve[i].real),
				qnum_fracdgt(curve[i].real),
				qnum_toint(curve[i].ui),
				qnum_fracdgt(curve[i].ui));
		if (len >= size)
			break;
	}

	buff[len] = 0;
	return buff;
}

/* NOTE: no bounds checks on this one */
static int ssoc_uicurve_find(qnum_t real, struct ssoc_uicurve *curve)
{
	int i;

	for (i = 1; i < UICURVE_MAX ; i++) {
		if (real == curve[i].real)
			return i;
		if (real > curve[i].real)
			continue;
		break;
	}

	return i-1;
}

static qnum_t ssoc_uicurve_map(qnum_t real, struct ssoc_uicurve *curve)
{
	qnum_t slope = 0, delta_ui, delta_re;
	int i;

	if (real < curve[0].real)
		return real;
	if (real >= curve[UICURVE_MAX - 1].ui)
		return curve[UICURVE_MAX - 1].ui;

	i = ssoc_uicurve_find(real, curve);
	if (curve[i].real == real)
		return curve[i].ui;

	delta_ui = curve[i + 1].ui - curve[i].ui;
	delta_re =  curve[i + 1].real - curve[i].real;
	if (delta_re)
		slope = qnum_div(delta_ui, delta_re);

	return curve[i].ui + qnum_mul(slope, (real - curve[i].real));
}

/* "optimized" to work on 3 element curves */
static void ssoc_uicurve_splice(struct ssoc_uicurve *curve, qnum_t real,
				qnum_t ui)
{
	if (real < curve[0].real || real > curve[2].real)
		return;

#if UICURVE_MAX != 3
#error ssoc_uicurve_splice() only support UICURVE_MAX == 3
#endif

	/* splice only when real is within the curve range */
	curve[1].real = real;
	curve[1].ui = ui;

	if (curve[1].real > curve[UICURVE_MAX - 1].real)
		curve[UICURVE_MAX - 1].real = ssoc_point_full;
}

static void ssoc_uicurve_dup(struct ssoc_uicurve *dst,
			     struct ssoc_uicurve *curve)
{
	if (dst != curve)
		memcpy(dst, curve, sizeof(*dst)*UICURVE_MAX);
}

/* "optimized" to work on 3 element curves */
static void ssoc_uicurve_splice_full(struct ssoc_uicurve *curve,
				     qnum_t real,qnum_t ui)
{
	/*
	 * for case: curve:[15.00 15.00][99.00 99.00][98.00 100.00]
	 * the calculation in ssoc_uicurve_map causes minus value
	 */
	if (curve[1].real > real)
		return;

	curve[UICURVE_MAX - 1].real = real;
	curve[UICURVE_MAX - 1].ui = ui;
}



/* ------------------------------------------------------------------------- */

/* could also use the rate of change for this */
static qnum_t ssoc_rl_max_delta(const struct batt_ssoc_rl_state *rls,
				int bucken, ktime_t delta_time)
{
	int i;
	qnum_t max_delta;

	if (delta_time > rls->rl_delta_max_time &&
		((qnum_toint(rls->rl_delta_max_soc) * delta_time) / rls->rl_delta_max_time) > 100) {
		return qnum_fromint(100);
	}

	max_delta = div_s64(((qnumd_t)rls->rl_delta_max_soc * delta_time),
				  (rls->rl_delta_max_time ? rls->rl_delta_max_time : 1));

	if (rls->rl_fast_track)
		return max_delta;

	/* might have one table for charging and one for discharging */
	for (i = 0; i < rls->rl_delta_soc_cnt; i++) {
		if (rls->rl_delta_soc_limit[i] == 0)
			break;

		if (rls->rl_ssoc_target < rls->rl_delta_soc_limit[i])
			return div_s64(((qnumd_t)max_delta * 10),
				rls->rl_delta_soc_ratio[i]);
	}

	return max_delta;
}

static qnum_t ssoc_apply_rl(struct batt_ssoc_state *ssoc)
{
	const ktime_t now = get_boot_sec();
	struct batt_ssoc_rl_state *rls = &ssoc->ssoc_rl_state;
	qnum_t rl_val;

	/* track ssoc_uic when buck is enabled or the minimum value of uic */
	if (ssoc->buck_enabled ||
	    (!ssoc->buck_enabled && ssoc->ssoc_uic < rls->rl_ssoc_target))
		rls->rl_ssoc_target = ssoc->ssoc_uic;

	/* sanity on the target */
	if (rls->rl_ssoc_target > qnum_fromint(100))
		rls->rl_ssoc_target = qnum_fromint(100);
	if (rls->rl_ssoc_target < qnum_fromint(0))
		rls->rl_ssoc_target = qnum_fromint(0);

	/* closely track target */
	if (rls->rl_track_target) {
		rl_val = rls->rl_ssoc_target;
	} else {
		qnum_t step;
		const ktime_t delta_time = now - rls->rl_ssoc_last_update;
		const qnum_t max_delta = ssoc_rl_max_delta(rls,
							   ssoc->buck_enabled,
							   delta_time);

		/* apply the rate limiter, delta_soc to target */
		step = rls->rl_ssoc_target - ssoc->ssoc_rl;
		if (step < -max_delta)
			step = -max_delta;
		else if (step > max_delta)
			step = max_delta;

		rl_val = ssoc->ssoc_rl + step;
	}

	/* do not increase when not connected */
	if (!ssoc->buck_enabled && rl_val > ssoc->ssoc_rl)
		rl_val = ssoc->ssoc_rl;

	/* will report 0% when rl_no_zero clears */
	if (rls->rl_no_zero && rl_val <= qnum_fromint(1))
		rl_val = qnum_fromint(1);

	rls->rl_ssoc_last_update = now;
	return rl_val;
}

/* ------------------------------------------------------------------------- */

static int ssoc_get_real_raw(const struct batt_ssoc_state *ssoc)
{
	return ssoc->ssoc_gdf;
}

/* a statement :-) */
static qnum_t ssoc_get_capacity_raw(const struct batt_ssoc_state *ssoc)
{
	return ssoc->ssoc_rl;
}

static int ssoc_get_real(const struct batt_ssoc_state *ssoc)
{
	const qnum_t real_raw = ssoc_get_real_raw(ssoc);

	return qnum_toint(real_raw);
}

#define SOC_ROUND_BASE	0.5

/* reported to userspace: call while holding batt_lock */
static int ssoc_get_capacity(const struct batt_ssoc_state *ssoc)
{
	const qnum_t raw = ssoc_get_capacity_raw(ssoc);

	return qnum_roundint(raw, SOC_ROUND_BASE);
}

/* ------------------------------------------------------------------------- */

static void dump_ssoc_state(struct batt_ssoc_state *ssoc_state,
			    struct logbuffer *log)
{
	char buff[UICURVE_BUF_SZ] = { 0 };

	scnprintf(ssoc_state->ssoc_state_cstr,
		  sizeof(ssoc_state->ssoc_state_cstr),
		  "SSOC: l=%d%% gdf=%d.%02d uic=%d.%02d rl=%d.%02d ct=%d curve:%s rls=%d bd_cnt=%d",
		  ssoc_get_capacity(ssoc_state),
		  qnum_toint(ssoc_state->ssoc_gdf),
		  qnum_fracdgt(ssoc_state->ssoc_gdf),
		  qnum_toint(ssoc_state->ssoc_uic),
		  qnum_fracdgt(ssoc_state->ssoc_uic),
		  qnum_toint(ssoc_state->ssoc_rl),
		  qnum_fracdgt(ssoc_state->ssoc_rl),
		  ssoc_state->ssoc_curve_type,
		  ssoc_uicurve_cstr(buff, sizeof(buff), ssoc_state->ssoc_curve),
		  ssoc_state->rl_status,
		  ssoc_state->bd_trickle_cnt);

	logbuffer_log(log, "%s", ssoc_state->ssoc_state_cstr);
	pr_debug("%s\n", ssoc_state->ssoc_state_cstr);
}

/* ------------------------------------------------------------------------- */

/* call while holding batt_lock */
static void ssoc_update(struct batt_ssoc_state *ssoc, qnum_t soc)
{
	struct batt_ssoc_rl_state *rls =  &ssoc->ssoc_rl_state;
	qnum_t delta;

	/* low pass filter */
	ssoc->ssoc_gdf = soc;
	/* spoof UI @ EOC */
	ssoc->ssoc_uic = ssoc_uicurve_map(ssoc->ssoc_gdf, ssoc->ssoc_curve);

	/* first target is current UIC */
	if (rls->rl_ssoc_target == -1) {
		rls->rl_ssoc_target = ssoc->ssoc_uic;
		ssoc->ssoc_rl = ssoc->ssoc_uic;
	}

	/* enable fast track when target under configured limit */
	rls->rl_fast_track |= rls->rl_ssoc_target < rls->rl_ft_low_limit;

	/*
	 * delta fast tracking during charge
	 * NOTE: might use the stats from TTF to determine the maximum rate
	 */
	delta = rls->rl_ssoc_target - ssoc->ssoc_rl;
	if (rls->rl_ft_delta_limit && ssoc->buck_enabled && delta > 0) {
		/* only when SOC increase */
		rls->rl_fast_track |= delta > rls->rl_ft_delta_limit;
	} else if (rls->rl_ft_delta_limit && !ssoc->buck_enabled && delta < 0) {
		/* enable fast track when target under configured limit */
		rls->rl_fast_track |= -delta > rls->rl_ft_delta_limit;
	}

	/*
	 * Right now a simple test on target metric falling under 0.5%
	 * TODO: add a filter that decrements no_zero when a specific
	 * condition is met (ex rl_ssoc_target < 1%).
	 */
	if (rls->rl_no_zero)
		rls->rl_no_zero = rls->rl_ssoc_target > qnum_from_q8_8(128);

	/*  monotonicity and rate of change */
	ssoc->ssoc_rl = ssoc_apply_rl(ssoc);
}

/*
 * Maxim could need:
 *	1fh AvCap, 10h FullCap. 23h FullCapNom
 * QC could need:
 *	QG_CC_SOC, QG_Raw_SOC, QG_Bat_SOC, QG_Sys_SOC, QG_Mon_SOC
 */
#define DISABLE_POINT_FULL_UI_SOC (-1)
static int ssoc_work(struct batt_ssoc_state *ssoc_state,
		     struct power_supply *fg_psy)
{
	int soc_q8_8;
	qnum_t soc_raw;

	/*
	 * TODO: GBMS_PROP_CAPACITY_RAW should return a qnum_t
	 * TODO: add an array here configured in DT with the properties
	 * to query and their weights, make soc_raw come from fusion.
	 */
	soc_q8_8 = GPSY_GET_PROP(fg_psy, GBMS_PROP_CAPACITY_RAW);
	if (soc_q8_8 < 0)
		return -EINVAL;

	/*
	 * soc_raw can come from fusion:
	 *    soc_raw = m1 * w1 + m2 * w2 + ...
	 *
	 * where m1, m2 are gauge metrics, w1,w1 are weights that change
	 * with temperature, state of charge, battery health etc.
	 */
	soc_raw = qnum_from_q8_8(soc_q8_8);

	ssoc_update(ssoc_state, soc_raw);
	return 0;
}

static void ssoc_change_curve_at_gdf(struct batt_ssoc_state *ssoc_state,
				     qnum_t gdf, qnum_t capacity,
				     enum ssoc_uic_type type)
{
	struct ssoc_uicurve *new_curve;

	new_curve = (type == SSOC_UIC_TYPE_DSG) ? dsg_curve : chg_curve;
	ssoc_uicurve_dup(ssoc_state->ssoc_curve, new_curve);
	ssoc_state->ssoc_curve_type = type;

	/* splice at (->ssoc_gdf,->ssoc_rl) because past spoof */
	ssoc_uicurve_splice(ssoc_state->ssoc_curve, gdf, capacity);
}

/*
 * Called on connect and disconnect to adjust the UI curve. On disconnect
 * splice at GDF less a fixed delta while UI is at 100% (i.e. in RL) to
 * avoid showing 100% for "too long" after disconnect.
 */
static void ssoc_change_curve(struct batt_ssoc_state *ssoc_state, qnum_t delta,
			      enum ssoc_uic_type type)
{
	qnum_t ssoc_level = ssoc_get_capacity(ssoc_state);
	qnum_t gdf = ssoc_state->ssoc_gdf; /* actual battery level */

	/* force dsg curve when connect/disconnect with battery at 100% */
	if (ssoc_level >= SSOC_FULL) {
		const qnum_t rlt = qnum_fromint(ssoc_state->rl_soc_threshold);

		/* bounds GDF - DELTA to prevent SSOC/GDF from diverging significantly */
		gdf = gdf > rlt ? gdf : rlt;

		type = SSOC_UIC_TYPE_DSG;
		gdf -=  delta;
	}

	/* adjust gdf to update curve[1].real in ssoc_uicurve_splice() */
	if (gdf > ssoc_point_full)
		gdf = ssoc_point_full;

	ssoc_change_curve_at_gdf(ssoc_state, gdf,
				 ssoc_get_capacity_raw(ssoc_state), type);
}

/* Fan levels limits from battery temperature */
#define FAN_BT_LIMIT_NOT_CARE	320
#define FAN_BT_LIMIT_LOW	420
#define FAN_BT_LIMIT_MED	460
#define FAN_BT_LIMIT_HIGH	480
/* Fan levels limits from charge rate */
#define FAN_CHG_LIMIT_NOT_CARE	10
#define FAN_CHG_LIMIT_LOW	50
#define FAN_CHG_LIMIT_MED	70

static int fan_bt_calculate_level(struct batt_drv *batt_drv)
{
	int level, temp, ret;

	ret = gbatt_get_temp(batt_drv, &temp);
	if (ret < 0) {

		if (batt_drv->temp_idx < 2)
			level = FAN_LVL_NOT_CARE;
		else if (batt_drv->temp_idx == 3)
			level = FAN_LVL_MED;
		else
			level = FAN_LVL_HIGH;

		pr_warn("FAN_LEVEL: level=%d from temp_idx=%d (%d)\n",
			level, batt_drv->temp_idx, ret);
		return level;
	}

	if (temp <= batt_drv->fan_bt_limits[0])
		level = FAN_LVL_NOT_CARE;
	else if (temp <= batt_drv->fan_bt_limits[1])
		level = FAN_LVL_LOW;
	else if (temp <= batt_drv->fan_bt_limits[2])
		level = FAN_LVL_MED;
	else if (temp <= batt_drv->fan_bt_limits[3])
		level = FAN_LVL_HIGH;
	else
		level = FAN_LVL_ALARM;

	return level;
}

static int fan_calculate_level(struct batt_drv *batt_drv)
{
	int charging_rate, fan_level, chg_fan_level, cc_max;

	if (batt_drv->jeita_stop_charging == 1)
		return FAN_LVL_ALARM;

	/* defender limits from google_charger */
	fan_level = fan_bt_calculate_level(batt_drv);

	cc_max = gvotable_get_current_int_vote(batt_drv->fcc_votable);
	if (cc_max <= 0 || batt_drv->battery_capacity == 0)
		return fan_level;

	/* cc_max is -1 when disconnected */
	charging_rate = cc_max / batt_drv->battery_capacity / 10;
	if (charging_rate < FAN_CHG_LIMIT_NOT_CARE)
		chg_fan_level = FAN_LVL_NOT_CARE;
	else if (charging_rate <= FAN_CHG_LIMIT_LOW)
		chg_fan_level = FAN_LVL_LOW;
	else if (charging_rate <= FAN_CHG_LIMIT_MED)
		chg_fan_level = FAN_LVL_MED;
	else
		chg_fan_level = FAN_LVL_HIGH;

	/* Charge rate can increase the level */
	if (chg_fan_level > fan_level)
		fan_level = chg_fan_level;

	return fan_level;
}

static bool vote_fan_level(struct gvotable_election *fan_level_votable, int level, bool enable)
{
	int ret;

	if (!fan_level_votable)
		fan_level_votable = gvotable_election_get_handle(VOTABLE_FAN_LEVEL);

	if (!fan_level_votable)
		return false;

	ret = gvotable_cast_int_vote(fan_level_votable, "MSC_BATT", level, enable);
	if (ret < 0) {
		pr_err("MSC_FAN_LVL: enable:%d, level=%d ret=%d\n", enable, level, ret);
		return false;
	}

	return true;
}

static void fan_level_reset(struct batt_drv *batt_drv)
{
	vote_fan_level(batt_drv->fan_level_votable, 0, false);
}

/* ------------------------------------------------------------------------- */

/*
 * enter recharge logic in BATT_RL_STATUS_DISCHARGE on charger_DONE,
 * enter BATT_RL_STATUS_RECHARGE on Fuel Gauge FULL
 * NOTE: batt_rl_update_status() doesn't call this, it flip from DISCHARGE
 * to recharge on its own.
 * NOTE: call holding chg_lock
 * FIX: BatteryDefenderUI different rules when battery defender is enabled
 * @pre rl_status != BATT_RL_STATUS_NONE
 */
static bool batt_rl_enter(struct batt_ssoc_state *ssoc_state,
			  enum batt_rl_status rl_status)
{
	const int rl_current = ssoc_state->rl_status;
	const bool enable = ssoc_state->bd_trickle_enable;
	const bool dry_run = ssoc_state->bd_trickle_dry_run;

	/*
	 * NOTE: NO_OP when RL=DISCHARGE since batt_rl_update_status() flip
	 * between BATT_RL_STATUS_DISCHARGE and BATT_RL_STATUS_RECHARGE
	 * directly.
	 */
	if (rl_current == rl_status || rl_current == BATT_RL_STATUS_DISCHARGE)
		return false;

	/*
	 * NOTE: rl_status transition from *->DISCHARGE on charger FULL (during
	 * charge or at the end of recharge) and transition from
	 * NONE->RECHARGE when battery is full (SOC==100%) before charger is.
	 */
	if (rl_status == BATT_RL_STATUS_DISCHARGE) {
		if (enable && !dry_run && ssoc_state->bd_trickle_cnt > 0) {
			ssoc_change_curve(ssoc_state, 0, SSOC_UIC_TYPE_DSG);
		} else {
			ssoc_uicurve_dup(ssoc_state->ssoc_curve, dsg_curve);
			ssoc_state->ssoc_curve_type = SSOC_UIC_TYPE_DSG;
		}
	}

	ssoc_update(ssoc_state, ssoc_state->ssoc_gdf);
	ssoc_state->rl_status = rl_status;

	return true;
}

static int ssoc_rl_read_dt(struct batt_ssoc_rl_state *rls,
			   struct device_node *node)
{
	u32 tmp, delta_soc[RL_DELTA_SOC_MAX];
	int ret, i;

	ret = of_property_read_u32(node, "google,rl_delta-max-soc", &tmp);
	if (ret == 0)
		rls->rl_delta_max_soc = qnum_fromint(tmp);

	ret = of_property_read_u32(node, "google,rl_delta-max-time", &tmp);
	if (ret == 0)
		rls->rl_delta_max_time = tmp;

	if (!rls->rl_delta_max_soc || !rls->rl_delta_max_time)
		return -EINVAL;

	rls->rl_no_zero = of_property_read_bool(node, "google,rl_no-zero");
	rls->rl_track_target = of_property_read_bool(node,
						     "google,rl_track-target");

	ret = of_property_read_u32(node, "google,rl_ft-low-limit", &tmp);
	if (ret == 0)
		rls->rl_ft_low_limit = qnum_fromint(tmp);

	ret = of_property_read_u32(node, "google,rl_ft-delta-limit", &tmp);
	if (ret == 0)
		rls->rl_ft_delta_limit = qnum_fromint(tmp);

	rls->rl_delta_soc_cnt = of_property_count_elems_of_size(node,
					      "google,rl_soc-limits",
					      sizeof(u32));
	tmp = of_property_count_elems_of_size(node, "google,rl_soc-rates",
					      sizeof(u32));
	if (rls->rl_delta_soc_cnt != tmp || tmp == 0) {
		rls->rl_delta_soc_cnt = 0;
		goto done;
	}

	if (rls->rl_delta_soc_cnt > RL_DELTA_SOC_MAX)
		return -EINVAL;

	ret = of_property_read_u32_array(node, "google,rl_soc-limits",
					 delta_soc,
					 rls->rl_delta_soc_cnt);
	if (ret < 0)
		return ret;

	for (i = 0; i < rls->rl_delta_soc_cnt; i++)
		rls->rl_delta_soc_limit[i] = qnum_fromint(delta_soc[i]);

	ret = of_property_read_u32_array(node, "google,rl_soc-rates",
					 delta_soc,
					 rls->rl_delta_soc_cnt);
	if (ret < 0)
		return ret;

	for (i = 0; i < rls->rl_delta_soc_cnt; i++)
		rls->rl_delta_soc_ratio[i] = delta_soc[i];

done:
	return 0;
}


/*
 * NOTE: might need to use SOC from bootloader as starting point to avoid UI
 * SSOC jumping around or taking long time to coverge. Could technically read
 * charger voltage and estimate SOC% based on empty and full voltage.
 */
static int ssoc_init(struct batt_drv *batt_drv)
{
	struct batt_ssoc_state *ssoc_state = &batt_drv->ssoc_state;
	int ret, capacity;

	ret = ssoc_rl_read_dt(&ssoc_state->ssoc_rl_state, batt_drv->device->of_node);
	if (ret < 0)
		ssoc_state->ssoc_rl_state.rl_track_target = 1;
	ssoc_state->ssoc_rl_state.rl_ssoc_target = -1;

	/*
	 * ssoc_work() needs a curve: start with the charge curve to prevent
	 * SSOC% from increasing after a reboot. Curve type must be NONE until
	 * battery knows the charger BUCK_EN state.
	 */
	ssoc_uicurve_dup(ssoc_state->ssoc_curve, chg_curve);
	ssoc_state->ssoc_curve_type = SSOC_UIC_TYPE_NONE;

	ret = ssoc_work(ssoc_state, batt_drv->fg_psy);
	if (ret < 0)
		return -EIO;

	capacity = ssoc_get_capacity(ssoc_state);
	if (capacity >= SSOC_FULL) {
		/* consistent behavior when booting without adapter */
		ssoc_uicurve_dup(ssoc_state->ssoc_curve, dsg_curve);
	} else if (capacity < SSOC_TRUE) {
		/* no split */
	} else if (capacity < SSOC_SPOOF) {
		/* mark the initial point if under spoof */
		ssoc_uicurve_splice(ssoc_state->ssoc_curve,
						ssoc_state->ssoc_gdf,
						ssoc_state->ssoc_rl);

	}

	dump_ssoc_state(&batt_drv->ssoc_state, batt_drv->ssoc_log);

	ret = gbatt_restore_capacity(batt_drv);
	if (ret < 0)
		dev_warn(batt_drv->device, "unable to restore capacity, ret=%d\n", ret);
	else
		ssoc_state->save_soc_available = true;

	return 0;
}

/* ------------------------------------------------------------------------- */

/*
 * just reset state, no PS notifications no changes in the UI curve. This is
 * called on startup and on disconnect when the charge driver state is reset
 * NOTE: call holding chg_lock
 */
static void batt_rl_reset(struct batt_drv *batt_drv)
{
	batt_drv->ssoc_state.rl_status = BATT_RL_STATUS_NONE;
}

/*
 * RL recharge: call after SSOC work, restart charging when gdf hit the
 * recharge threshold.
 * NOTE: call holding chg_lock
 */
static void batt_rl_update_status(struct batt_drv *batt_drv)
{
	struct batt_ssoc_state *ssoc_state = &batt_drv->ssoc_state;
	const bool bd_dry_run = ssoc_state->bd_trickle_dry_run;
	const int bd_cnt = ssoc_state->bd_trickle_cnt;
	int soc, rl_soc_threshold;

	/* already in _RECHARGE or _NONE, done */
	if (ssoc_state->rl_status != BATT_RL_STATUS_DISCHARGE)
		return;
	/* no threashold (why I am here???) */
	if (!ssoc_state->rl_soc_threshold)
		return;
	/* recharge logic work on real soc */
	soc = ssoc_get_real(ssoc_state);

	if (ssoc_state->bd_trickle_enable)
		rl_soc_threshold = ((bd_cnt > 0) && !bd_dry_run) ?
			ssoc_state->bd_trickle_recharge_soc :
			ssoc_state->rl_soc_threshold;
	else
		rl_soc_threshold = ssoc_state->rl_soc_threshold;

	if (soc > rl_soc_threshold)
		return;

	/* change state (will restart charge) on trigger */
	ssoc_state->rl_status = BATT_RL_STATUS_RECHARGE;
	if (batt_drv->psy)
		power_supply_changed(batt_drv->psy);

	if (ssoc_state->bd_trickle_full && ssoc_state->bd_trickle_eoc) {
		ssoc_state->bd_trickle_cnt++;
		ssoc_state->bd_trickle_full = false;
		ssoc_state->bd_trickle_eoc = false;
	}

	dump_ssoc_state(&batt_drv->ssoc_state, batt_drv->ssoc_log);
}

/* ------------------------------------------------------------------------- */

static void bat_log_ttf_change(ktime_t estimate, int max_ratio, struct batt_drv *batt_drv)
{
	const struct gbms_charging_event *ce_data = &batt_drv->ce_data;
	char buff[LOG_BUFFER_ENTRY_SIZE];
	long elap, ibatt_avg, icl_avg;
	int i, len = 0;

	len += scnprintf(&buff[len], sizeof(buff) - len,
			 "MSC_TTF: est:%lld(%lldmin), max_ratio:%d ",
			 estimate, ktime_divns(estimate, 60), max_ratio);

	for (i = 0; i < GBMS_STATS_TIER_COUNT; i++) {
		elap = ce_data->tier_stats[i].time_fast +
				  ce_data->tier_stats[i].time_taper +
				  ce_data->tier_stats[i].time_other;
		if (elap) {
			ibatt_avg = div64_s64(ce_data->tier_stats[i].ibatt_sum, elap);
			icl_avg = div64_s64(ce_data->tier_stats[i].icl_sum, elap);
		} else {
			ibatt_avg = 0;
			icl_avg = 0;
		}

		len += scnprintf(&buff[len], sizeof(buff) - len,
				 "[%d:%ld,%ld,%ld]", i, elap / 60, ibatt_avg, icl_avg);
	}

	pr_info("%s", buff);

	batt_drv->ttf_est = estimate;
}

static bool batt_charge_to_limit_enable(const struct batt_chg_health *chg_health)
{
	return chg_health->rest_rate == 0 && chg_health->always_on_soc > 0;
}

/* batt_drv->batt_lock is acquired by a caller */
static void batt_force_fcr_update_charging_policy(struct batt_drv *batt_drv, bool update)
{
	if (batt_drv->vote_force_full_charge == update)
		return;

	gvotable_cast_long_vote(batt_drv->charging_policy_votable, FCRU_CHARGE_VOTER,
				CHARGING_POLICY_VOTE_FORCE_FULL_CHARGE, update);
	batt_drv->vote_force_full_charge = update;
}

static bool batt_need_force_fcr_update(struct batt_drv *batt_drv)
{
	switch (batt_drv->force_fcr_update_ops) {
	case BATT_FCR_UPDATE_DISABLE:
		return false;
	case BATT_FCR_UPDATE_FULLCHARGE_DELTA:
		return batt_drv->hist_data_saved_cnt - batt_drv->last_full_charge >=
		       batt_drv->force_fcr_update_cycle;
	default:
		return true;
	}
}
/*
 * msc_logic_health() sync ce_data->ce_health to batt_drv->chg_health
 * . return -EINVAL when the device is not connected to power -ERANGE when
 *   ttf_soc_estimate() returns a negative value (invalid parameters, or
 *   corrupted internal data)
 * . the estimate is 0 when the device is at 100%.
 * . the estimate is negative during debounce, when in overheat, when
 *   custom charge levels are active.
 */
#define MIN_DELTA_FOR_LOG_S 60
static int batt_ttf_estimate(ktime_t *res, struct batt_drv *batt_drv)
{
	qnum_t soc_raw = ssoc_get_real_raw(&batt_drv->ssoc_state);
	ktime_t estimate = 0;
	int rc = 0, max_ratio = 0, ssoc_full = SSOC_FULL;
	const bool need_fcr_update = batt_need_force_fcr_update(batt_drv);
	qnum_t raw_full;

	if (batt_drv->ssoc_state.buck_enabled != 1)
		return -EINVAL;

	if (batt_drv->ttf_stats.ttf_fake != -1) {
		estimate = batt_drv->ttf_stats.ttf_fake;
		goto done;
	}

	if (!need_fcr_update) {
		if (batt_drv->charging_policy == CHARGING_POLICY_VOTE_LONGLIFE)
			ssoc_full = LONGLIFE_CHARGE_STOP_LEVEL;
		else if (batt_charge_to_limit_enable(&batt_drv->chg_health))
			ssoc_full = batt_drv->chg_health.always_on_soc;
	}

	if (batt_drv->charging_policy == CHARGING_POLICY_VOTE_LONGLIFE)
		batt_force_fcr_update_charging_policy(batt_drv, need_fcr_update);


	raw_full = qnum_fromint(ssoc_full) - qnum_rconst(SOC_ROUND_BASE);

	/* TTF is 0 when over ssoc_full */
	if (ssoc_get_capacity(&batt_drv->ssoc_state) >= ssoc_full) {
		estimate = 0;
		goto done;
	}

	/* no estimates during debounce or with special profiles */
	if (batt_drv->ttf_debounce ||
	    batt_drv->batt_health == POWER_SUPPLY_HEALTH_OVERHEAT ||
	    ((batt_drv->chg_state.f.flags & GBMS_CS_FLAG_CCLVL) &&
	    (batt_drv->charging_policy != CHARGING_POLICY_VOTE_LONGLIFE))) {
		estimate = -1;
		goto done;
	}

	/*
	 * Handle rounding (removing it from the end)
	 * example: 96.64% with SOC_ROUND_BASE = 0.5 -> UI = 97
	 *    ttf = elap[96] * 0.36 + elap[97] + elap[98] +
	 * 	    elap[99] * (1 - 0.5)
	 *
	 * negative return value (usually) means data corruption
	 */
	rc = ttf_soc_estimate(&estimate, &batt_drv->ttf_stats,
			      &batt_drv->ce_data, soc_raw, raw_full);
	if (rc < 0)
		estimate = -1;
	else
		max_ratio = rc;

	/* Log data when changed over 1 min */
	if (abs(batt_drv->ttf_est - estimate) > MIN_DELTA_FOR_LOG_S)
		bat_log_ttf_change(estimate, max_ratio, batt_drv);

done:
	*res = estimate;
	return max_ratio;
}

/* ------------------------------------------------------------------------- */
/* CEV = Charging EVent */
static void cev_stats_init(struct gbms_charging_event *ce_data,
			   const struct gbms_chg_profile *profile)
{
	int i;

	memset(ce_data, 0, sizeof(*ce_data));

	ce_data->chg_profile = profile;
	ce_data->charging_stats.voltage_in = -1;
	ce_data->charging_stats.ssoc_in = -1;
	ce_data->charging_stats.voltage_out = -1;
	ce_data->charging_stats.ssoc_out = -1;

	ttf_soc_init(&ce_data->soc_stats);
	ce_data->last_soc = -1;

	for (i = 0; i < GBMS_STATS_TIER_COUNT ; i++)
		gbms_tier_stats_init(&ce_data->tier_stats[i], i);

	/* batt_chg_health_stats_close() will fix this */
	gbms_tier_stats_init(&ce_data->health_stats, GBMS_STATS_AC_TI_INVALID);
	gbms_tier_stats_init(&ce_data->health_pause_stats, GBMS_STATS_AC_TI_PAUSE);
	gbms_tier_stats_init(&ce_data->health_dryrun_stats, GBMS_STATS_AC_TI_V2_PREDICT);

	gbms_tier_stats_init(&ce_data->full_charge_stats, GBMS_STATS_AC_TI_FULL_CHARGE);
	gbms_tier_stats_init(&ce_data->high_soc_stats, GBMS_STATS_AC_TI_HIGH_SOC);
	gbms_tier_stats_init(&ce_data->overheat_stats, GBMS_STATS_BD_TI_OVERHEAT_TEMP);
	gbms_tier_stats_init(&ce_data->cc_lvl_stats, GBMS_STATS_BD_TI_CUSTOM_LEVELS);
	gbms_tier_stats_init(&ce_data->trickle_stats, GBMS_STATS_BD_TI_TRICKLE_CLEARED);
	gbms_tier_stats_init(&ce_data->temp_filter_stats, GBMS_STATS_TEMP_FILTER);
	gbms_tier_stats_init(&ce_data->policy_longlife_stats, GBMS_STATS_BD_TI_POLICY_LONGLIFE);
	gbms_tier_stats_init(&ce_data->policy_force_full_stats, GBMS_STATS_BD_TI_POLICY_FORCE_TO_FULL);
}

static void batt_chg_stats_start(struct batt_drv *batt_drv)
{
	union gbms_ce_adapter_details ad;
	struct gbms_charging_event *ce_data = &batt_drv->ce_data;
	const ktime_t now = get_boot_sec();
	int vin, cc_in;

	mutex_lock(&batt_drv->stats_lock);
	ad.v = batt_drv->ce_data.adapter_details.v;
	cev_stats_init(ce_data, &batt_drv->chg_profile);
	batt_drv->ce_data.adapter_details.v = ad.v;

	vin = GPSY_GET_PROP(batt_drv->fg_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW);
	ce_data->charging_stats.voltage_in = (vin < 0) ? -1 : vin / 1000;
	ce_data->charging_stats.ssoc_in =
				ssoc_get_capacity(&batt_drv->ssoc_state);
	cc_in = GPSY_GET_PROP(batt_drv->fg_psy,
				POWER_SUPPLY_PROP_CHARGE_COUNTER);
	ce_data->charging_stats.cc_in = (cc_in < 0) ? -1 : cc_in / 1000;

	ce_data->charging_stats.ssoc_out = -1;
	ce_data->charging_stats.voltage_out = -1;

	ce_data->first_update = now;
	ce_data->last_update = now;

	mutex_unlock(&batt_drv->stats_lock);
}

/* call holding stats_lock */
static bool batt_chg_stats_qual(const struct batt_drv *batt_drv)
{
	const struct gbms_charging_event *ce_data = &batt_drv->ce_data;
	const long elap = ce_data->last_update - ce_data->first_update;
	const long ssoc_delta = ce_data->charging_stats.ssoc_out -
				ce_data->charging_stats.ssoc_in;

	return elap >= batt_drv->chg_sts_qual_time ||
	    ssoc_delta >= batt_drv->chg_sts_delta_soc;
}

/* call holding stats_lock */
static void batt_chg_stats_soc_update(struct gbms_charging_event *ce_data,
				      qnum_t soc, ktime_t elap, int tier_index,
				      int cc)
{
	int index;
	const int last_soc = ce_data->last_soc;

	index = qnum_toint(soc);
	if (index < 0)
		index = 0;
	if (index > 100)
		index = 100;
	if (index < last_soc)
		return;

	if (ce_data->soc_stats.elap[index] == 0) {
		ce_data->soc_stats.ti[index] = tier_index;
		ce_data->soc_stats.cc[index] = cc;
	}

	if (last_soc != -1)
		ce_data->soc_stats.elap[last_soc] += elap;

	ce_data->last_soc = index;
}

/* call holding stats_lock */
static void batt_chg_stats_update(struct batt_drv *batt_drv, int temp_idx,
				  int tier_idx, int ibatt_ma, int temp,
				  ktime_t elap)
{
	const int soc_real = ssoc_get_real(&batt_drv->ssoc_state);
	const int msc_state = batt_drv->msc_state; /* last msc_state */
	struct gbms_charging_event *ce_data = &batt_drv->ce_data;
	struct gbms_ce_tier_stats *tier = NULL;
	struct gbms_chg_profile *profile = &batt_drv->chg_profile;
	const int last_vbatt_idx = profile->volt_nb_limits - 1;
	int cc, soc_in;

	if (elap == 0)
		return;

	/* TODO: read at start of tier and update cc_total of previous */
	cc = GPSY_GET_PROP(batt_drv->fg_psy, POWER_SUPPLY_PROP_CHARGE_COUNTER);
	if (cc < 0) {
		pr_debug("MSC_STAT cannot read cc=%d\n", cc);
		return;
	}
	cc = cc / 1000;

	soc_in = GPSY_GET_PROP(batt_drv->fg_psy, GBMS_PROP_CAPACITY_RAW);
	if (soc_in < 0) {
		pr_info("MSC_STAT cannot read soc_in=%d\n", soc_in);
		/* We still want to update initialized tiers. */
		soc_in = -1;
	}

	/* Log CSI info into chg_stats */
	ce_data->csi_aggregate_status = batt_drv->csi_stats.aggregate_status;
	ce_data->csi_aggregate_type = batt_drv->csi_stats.aggregate_type;

	/* Log aacp_version, aacc and aafv into chg_stats */
	ce_data->aacp_version = batt_drv->aacp_version;
	ce_data->aacc = batt_drv->aacc;
	ce_data->aafv = profile->volt_limits[last_vbatt_idx] / 1000;

	/* Note: To log new voltage tiers, add to list in go/pixel-vtier-defs */
	/* ---  Log tiers in PARALLEL below ---  */

	if (soc_real >= SSOC_HIGH_SOC)
		gbms_stats_update_tier(temp_idx, ibatt_ma, temp, elap, cc,
				       &batt_drv->chg_state, msc_state, soc_in,
				       &ce_data->high_soc_stats);

	if (batt_drv->chg_health.dry_run_deadline > 0)
		gbms_stats_update_tier(temp_idx, ibatt_ma, temp, elap, cc,
				       &batt_drv->chg_state, msc_state, soc_in,
				       &ce_data->health_dryrun_stats);

	/* --- Log tiers in SERIES below --- */
	if (batt_drv->batt_full) {

		/* Override regular charge tiers when fully charged */
		gbms_stats_update_tier(temp_idx, ibatt_ma, temp, elap, cc,
				       &batt_drv->chg_state, msc_state, soc_in,
				       &ce_data->full_charge_stats);

	} else if (msc_state == MSC_HEALTH_PAUSE) {

		/*
		 * We log the pause tier in different AC tier groups so that we
		 * can capture pause time separately.
		 */
		gbms_stats_update_tier(temp_idx, ibatt_ma, temp, elap, cc,
				       &batt_drv->chg_state, msc_state, soc_in,
				       &ce_data->health_pause_stats);

	} else if (msc_state == MSC_HEALTH || msc_state == MSC_HEALTH_ALWAYS_ON) {
		/*
		 * It works because msc_logic call BEFORE updating msc_state.
		 * NOTE: that OVERHEAT and CCLVL disable AC, I should not be
		 * here if either of them are set.
		 * NOTE: We currently only log time when AC is ACTIVE.
		 * Thus, when disconnecting in ENABLED state, we will log a
		 * GBMS_STATS_AC_TI_ENABLED tier with no time, and the regular
		 * charge time is accumulated in normal charge tiers.
		 * Similarly, once we reach 100%, we stop counting time in the
		 * health tier and we rely on the full_charge_stats.
		 */

		/* tier used for TTF during HC, check msc_logic_health() */
		gbms_stats_update_tier(temp_idx, ibatt_ma, temp, elap, cc,
				       &batt_drv->chg_state, msc_state, soc_in,
				       &ce_data->health_stats);

	} else if (batt_drv->temp_filter.enable) {
		struct batt_temp_filter *temp_filter = &batt_drv->temp_filter;
		int no_filter_temp;

		mutex_lock(&temp_filter->lock);
		no_filter_temp = temp_filter->sample[temp_filter->last_idx];
		mutex_unlock(&temp_filter->lock);

		gbms_stats_update_tier(temp_idx, ibatt_ma, no_filter_temp, elap, cc,
				       &batt_drv->chg_state, msc_state, soc_in,
				       &ce_data->temp_filter_stats);
	} else {
		const qnum_t soc = ssoc_get_capacity_raw(&batt_drv->ssoc_state);

		/* book to previous soc unless discharging */
		if (msc_state != MSC_DSG) {
			/* TODO: should I use ssoc instead? */
			batt_chg_stats_soc_update(ce_data, soc, elap,
						  tier_idx, cc);
		}

		/*
		 * ce_data.tier_stats[tier_idx] are used for time to full.
		 * Do not book to them if we are in overheat or LVL
		 */
		tier = &ce_data->tier_stats[tier_idx];
	}

	/* --- Log tiers in PARALLEL that MUST NULL normal tiers below --- */

	/* batt_drv->batt_health is protected with chg_lock, */
	if (batt_drv->batt_health == POWER_SUPPLY_HEALTH_OVERHEAT) {
		gbms_stats_update_tier(temp_idx, ibatt_ma, temp, elap, cc,
				       &batt_drv->chg_state, msc_state, soc_in,
				       &ce_data->overheat_stats);
		tier = NULL;
	}

	/* custom charge levels (DWELL-DEFEND or RETAIL or LONGLIFE) */
	if (batt_drv->vote_force_full_charge) {
		gbms_stats_update_tier(temp_idx, ibatt_ma, temp, elap, cc,
				       &batt_drv->chg_state, msc_state, soc_in,
				       &ce_data->policy_force_full_stats);
		tier = NULL;
	} else if (batt_drv->charging_policy == CHARGING_POLICY_VOTE_LONGLIFE) {
		gbms_stats_update_tier(temp_idx, ibatt_ma, temp, elap, cc,
				       &batt_drv->chg_state, msc_state, soc_in,
				       &ce_data->policy_longlife_stats);
		tier = NULL;
	} else if (batt_drv->chg_state.f.flags & GBMS_CS_FLAG_CCLVL) {
		gbms_stats_update_tier(temp_idx, ibatt_ma, temp, elap, cc,
				       &batt_drv->chg_state, msc_state, soc_in,
				       &ce_data->cc_lvl_stats);
		tier = NULL;
	}

	/*
	 * Time/current spent in OVERHEAT or at CustomLevel should not
	 * be booked to ce_data.tier_stats[tier_idx]
	 */
	if (!tier)
		return;

	gbms_stats_update_tier(temp_idx, ibatt_ma, temp, elap, cc,
			       &batt_drv->chg_state, msc_state, soc_in, tier);
}

static int batt_chg_health_vti(const struct batt_chg_health *chg_health)
{
	enum chg_health_state rest_state = chg_health->rest_state;
	ktime_t rest_deadline = chg_health->rest_deadline;
	int tier_idx = GBMS_STATS_AC_TI_INVALID;
	bool aon_enabled = chg_health->always_on_soc != -1;

	switch (rest_state) {
	/* battery defender did it */
	case CHG_HEALTH_BD_DISABLED:
	case CHG_HEALTH_CCLVL_DISABLED:
		tier_idx = GBMS_STATS_AC_TI_DEFENDER;
		break;
	/* user disabled with deadline */
	case CHG_HEALTH_USER_DISABLED:
		if (rest_deadline == CHG_DEADLINE_SETTING)
			tier_idx = GBMS_STATS_AC_TI_DISABLE_SETTING;
		else if (rest_deadline == CHG_DEADLINE_SETTING_STOP)
			tier_idx = GBMS_STATS_AC_TI_DISABLE_SETTING_STOP;
		else if (rest_deadline == CHG_DEADLINE_DIALOG)
			tier_idx = GBMS_STATS_AC_TI_DISABLE_DIALOG;
		else
			tier_idx = GBMS_STATS_AC_TI_DISABLE_MISC;
		break;
	/* missed the deadline, TODO: log the deadline */
	case CHG_HEALTH_DISABLED:
		tier_idx = GBMS_STATS_AC_TI_DISABLED;
		break;
	/* disconnected in active mode, TODO: log the deadline */
	case CHG_HEALTH_ACTIVE:
	case CHG_HEALTH_PAUSE:
		if (aon_enabled)
			tier_idx = GBMS_STATS_AC_TI_ACTIVE_AON;
		else
			tier_idx = GBMS_STATS_AC_TI_ACTIVE;
		break;
	/* never became active */
	case CHG_HEALTH_ENABLED:
		if (aon_enabled)
			tier_idx = GBMS_STATS_AC_TI_ENABLED_AON;
		else
			tier_idx = GBMS_STATS_AC_TI_ENABLED;
		break;
	/* active, worked */
	case CHG_HEALTH_DONE:
		if (aon_enabled)
			tier_idx = GBMS_STATS_AC_TI_DONE_AON;
		else
			tier_idx = GBMS_STATS_AC_TI_VALID;
		break;
	default:
		break;
	}

	return tier_idx;
}

static int batt_chg_vbat2tier(const int vbatt_idx)
{
	return vbatt_idx < GBMS_STATS_TIER_COUNT ?
		vbatt_idx : GBMS_STATS_TIER_COUNT - 1;
}

static int batt_bpst_stats_update(struct batt_drv *batt_drv)
{
	struct batt_bpst *bpst_state = &batt_drv->bpst_state;

	if (!bpst_state->bpst_enable)
		return BPST_BATT_UNKNOWN;

	if (bpst_state->bpst_cell_fault)
		return BPST_BATT_CELL_FAULT;

	if (bpst_state->bpst_sbd_status)
		return BPST_BATT_DISCONNECT;

	return BPST_BATT_CONNECT;
}

/* Only the qualified copy gets the timestamp and the exit voltage. */
static bool batt_chg_stats_close(struct batt_drv *batt_drv,
				 char *reason,
				 bool force)
{
	bool publish;
	const int vout = GPSY_GET_PROP(batt_drv->fg_psy,
				POWER_SUPPLY_PROP_VOLTAGE_NOW);
	const int cc_out = GPSY_GET_PROP(batt_drv->fg_psy,
				POWER_SUPPLY_PROP_CHARGE_COUNTER);
	const ktime_t now = get_boot_sec();
	const ktime_t dry_run_deadline = batt_drv->chg_health.dry_run_deadline;

	/* book last period to the current tier
	 * NOTE: profile_vbatt_idx != -1 -> temp_idx != -1
	 */
	if (batt_drv->profile_vbatt_idx != -1 && batt_drv->temp_idx != -1) {
		const ktime_t elap = now - batt_drv->ce_data.last_update;
		const int tier_idx = batt_chg_vbat2tier(batt_drv->profile_vbatt_idx);
		int ibatt, temp, rc = 0;

		/* use default value to close charging session when read fail */
		ibatt = GPSY_GET_INT_PROP(batt_drv->fg_psy, POWER_SUPPLY_PROP_CURRENT_NOW, &rc);
		if (rc != 0)
			ibatt = 0;

		rc = gbatt_get_raw_temp(batt_drv, &temp);
		if (rc != 0)
			temp = 250;

		batt_chg_stats_update(batt_drv,
				      batt_drv->temp_idx, tier_idx,
				      ibatt / 1000, temp, elap);
		batt_drv->ce_data.last_update = now;
	}

	/* record the closing in data (and qual) */
	batt_drv->ce_data.charging_stats.voltage_out =
				(vout < 0) ? -1 : vout / 1000;
	batt_drv->ce_data.charging_stats.ssoc_out =
				ssoc_get_capacity(&batt_drv->ssoc_state);
	batt_drv->ce_data.charging_stats.cc_out =
				(cc_out < 0) ? -1 : cc_out / 1000;

	/* close/fix heath charge data (if enabled) */
	memcpy(&batt_drv->ce_data.ce_health, &batt_drv->chg_health,
	       sizeof(batt_drv->ce_data.ce_health));
	batt_drv->ce_data.health_stats.vtier_idx =
				batt_chg_health_vti(&batt_drv->chg_health);
	batt_drv->ce_data.health_dryrun_stats.vtier_idx =
		(now > dry_run_deadline) ? GBMS_STATS_AC_TI_V2_PREDICT_SUCCESS :
						 GBMS_STATS_AC_TI_V2_PREDICT;

	/* TODO: add a field to ce_data to qual weird charge sessions */
	publish = force || batt_chg_stats_qual(batt_drv);
	if (publish) {
		struct gbms_charging_event *ce_qual = &batt_drv->ce_qual;

		/* all charge tiers including health */
		memcpy(ce_qual, &batt_drv->ce_data, sizeof(*ce_qual));

		pr_info("MSC_STAT %s: elap=%lld ssoc=%d->%d v=%d->%d c=%d->%d hdl=%lld hrs=%d"
			" hti=%d/%d csi=%d/%d\n",
			reason,
			ce_qual->last_update - ce_qual->first_update,
			ce_qual->charging_stats.ssoc_in,
			ce_qual->charging_stats.ssoc_out,
			ce_qual->charging_stats.voltage_in,
			ce_qual->charging_stats.voltage_out,
			ce_qual->charging_stats.cc_in,
			ce_qual->charging_stats.cc_out,
			ce_qual->ce_health.rest_deadline,
			ce_qual->ce_health.rest_state,
			ce_qual->health_stats.vtier_idx,
			ce_qual->health_pause_stats.vtier_idx,
			ce_qual->csi_aggregate_status,
			ce_qual->csi_aggregate_type);
	}

	return publish;
}

static int batt_chg_stats_soc_next(const struct gbms_charging_event *ce_data,
				   int i)
{
	int soc_next;

	if (i == GBMS_STATS_TIER_COUNT -1)
		return ce_data->last_soc;

	soc_next = ce_data->tier_stats[i + 1].soc_in >> 8;
	if (soc_next <= 0)
		return ce_data->last_soc;

	return soc_next;
}

static void bat_log_chg_stats(struct logbuffer *log,
			      const struct gbms_charging_event *ce_data)
{
	const char *adapter_name =
		gbms_chg_ev_adapter_s(ce_data->adapter_details.ad_type);
	int i;

	logbuffer_log(log, "A: %s,%d,%d,%d",
			adapter_name,
			ce_data->adapter_details.ad_type,
			ce_data->adapter_details.ad_voltage * 100,
			ce_data->adapter_details.ad_amperage * 100);

	logbuffer_log(log, "S: %hu,%hu, %hu,%hu %hu,%hu %lld,%lld, %u",
			ce_data->charging_stats.ssoc_in,
			ce_data->charging_stats.voltage_in,
			ce_data->charging_stats.ssoc_out,
			ce_data->charging_stats.voltage_out,
			ce_data->charging_stats.cc_in,
			ce_data->charging_stats.cc_out,
			ce_data->first_update,
			ce_data->last_update,
			ce_data->chg_profile->capacity_ma);

	for (i = 0; i < GBMS_STATS_TIER_COUNT; i++) {
		const int soc_next = batt_chg_stats_soc_next(ce_data, i);
		const int soc_in = ce_data->tier_stats[i].soc_in >> 8;
		const long elap = ce_data->tier_stats[i].time_fast +
				  ce_data->tier_stats[i].time_taper +
				  ce_data->tier_stats[i].time_other;
		/* retrun len in below functions sometimes more than 256 */
		char buff[LOG_BUFFER_ENTRY_SIZE * 2] = {0};
		int len = 0;

		/* Do not output tiers without time */
		if (!elap)
			continue;

		len = gbms_tier_stats_cstr(buff, sizeof(buff),
						&ce_data->tier_stats[i], true);
		gbms_log_cstr_handler(log, buff, len);

		if (soc_next) {
			len = ttf_soc_cstr(buff, sizeof(buff),
					   &ce_data->soc_stats,
					   soc_in, soc_next);
			gbms_log_cstr_handler(log, buff, len);
		}
	}
}

/* End of charging: close stats, qualify event publish data */
static void batt_chg_stats_pub(struct batt_drv *batt_drv, char *reason,
			       bool force, bool skip_uevent)
{
	bool publish;

	mutex_lock(&batt_drv->stats_lock);
	publish = batt_chg_stats_close(batt_drv, reason, force);
	if (publish) {
		ttf_stats_update(&batt_drv->ttf_stats,
				 &batt_drv->ce_qual, false);

		if (skip_uevent == false)
			kobject_uevent(&batt_drv->device->kobj, KOBJ_CHANGE);
	}

	bat_log_chg_stats(batt_drv->ttf_stats.ttf_log, &batt_drv->ce_data);
	mutex_unlock(&batt_drv->stats_lock);
}

/* health_stats->tier_index is set on stats_close() */
static int batt_health_stats_cstr(char *buff, int size,
				  const struct gbms_charging_event *ce_data,
				  bool verbose)
{
	const struct gbms_ce_tier_stats *health_stats = &ce_data->health_stats;
	const int vti = batt_chg_health_vti(&ce_data->ce_health);
	int len = 0;

	len += scnprintf(&buff[len], size - len, "\nH: %d %d %lld %d\n",
			 ce_data->ce_health.rest_state, vti,
			 ce_data->ce_health.rest_deadline,
			 ce_data->ce_health.always_on_soc);

	/* no additional tier stats when vti is invalid */
	if (vti == GBMS_STATS_AC_TI_INVALID)
		return len;

	len += gbms_tier_stats_cstr(&buff[len], size - len,
				    health_stats, verbose);

	/* Only add pause tier logging if there is pause time */
	if (ce_data->health_pause_stats.soc_in != -1)
		len += gbms_tier_stats_cstr(&buff[len], size - len,
					    &ce_data->health_pause_stats,
					    verbose);

	return len;
}

/* doesn't output hc stats */
static int batt_chg_stats_cstr(char *buff, int size,
			       const struct gbms_charging_event *ce_data,
			       bool verbose, int state_capacity)
{
	int i, len = 0;

	if (verbose) {
		const char *adapter_name =
			gbms_chg_ev_adapter_s(ce_data->adapter_details.ad_type);

		len += scnprintf(&buff[len], size - len, "A: %s,",
				adapter_name);
	}

	len += scnprintf(&buff[len], size - len, "%d,%d,%d",
				ce_data->adapter_details.ad_type,
				ce_data->adapter_details.ad_voltage * 100,
				ce_data->adapter_details.ad_amperage * 100);

	len += scnprintf(&buff[len], size - len, "%s%hu,%hu, %hu,%hu %d %hu,%hu, %d,%d,%d,%d",
				(verbose) ?  "\nS: " : ", ",
				ce_data->charging_stats.ssoc_in,
				ce_data->charging_stats.voltage_in,
				ce_data->charging_stats.ssoc_out,
				ce_data->charging_stats.voltage_out,
				state_capacity,
				ce_data->csi_aggregate_status,
				ce_data->csi_aggregate_type,
				ce_data->aacp_version,
				ce_data->aacc,
				ce_data->aafv,
				ce_data->max_charge_voltage / 1000);


	if (verbose) {
		len += scnprintf(&buff[len], size - len, " %hu,%hu",
				ce_data->charging_stats.cc_in,
				ce_data->charging_stats.cc_out);

		len += scnprintf(&buff[len], size - len, " %lld,%lld",
				ce_data->first_update,
				ce_data->last_update);
	}

	for (i = 0; i < GBMS_STATS_TIER_COUNT; i++) {
		const int soc_next = batt_chg_stats_soc_next(ce_data, i);
		const int soc_in = ce_data->tier_stats[i].soc_in >> 8;
		const long elap = ce_data->tier_stats[i].time_fast +
				  ce_data->tier_stats[i].time_taper +
				  ce_data->tier_stats[i].time_other;

		/* Do not output tiers without time */
		if (!elap)
			continue;

		len += gbms_tier_stats_cstr(&buff[len], size - len,
					    &ce_data->tier_stats[i],
					    verbose);

		if (soc_next)
			len += ttf_soc_cstr(&buff[len], size - len,
					    &ce_data->soc_stats,
					    soc_in, soc_next);
	}

	/* Does not currently check MSC_HEALTH */
	if (ce_data->health_dryrun_stats.soc_in != -1)
		len += gbms_tier_stats_cstr(&buff[len], size - len,
					    &ce_data->health_dryrun_stats,
					    verbose);

	if (ce_data->full_charge_stats.soc_in != -1)
		len += gbms_tier_stats_cstr(&buff[len], size - len,
					    &ce_data->full_charge_stats,
					    verbose);

	if (ce_data->high_soc_stats.soc_in != -1)
		len += gbms_tier_stats_cstr(&buff[len], size - len,
					    &ce_data->high_soc_stats,
					    verbose);

	if (ce_data->overheat_stats.soc_in != -1)
		len += gbms_tier_stats_cstr(&buff[len], size - len,
					    &ce_data->overheat_stats,
					    verbose);

	if (ce_data->cc_lvl_stats.soc_in != -1)
		len += gbms_tier_stats_cstr(&buff[len], size - len,
					    &ce_data->cc_lvl_stats,
					    verbose);

	if (ce_data->temp_filter_stats.soc_in != -1)
		len += gbms_tier_stats_cstr(&buff[len], size - len,
					    &ce_data->temp_filter_stats,
					    verbose);

	if (ce_data->policy_longlife_stats.soc_in != -1)
		len += gbms_tier_stats_cstr(&buff[len], size - len,
					    &ce_data->policy_longlife_stats,
					    verbose);

	if (ce_data->policy_force_full_stats.soc_in != -1)
		len += gbms_tier_stats_cstr(&buff[len], size - len,
					    &ce_data->policy_force_full_stats,
					    verbose);

	/* If bd_clear triggers, we need to know about it even if trickle hasn't
	 * triggered
	 */
	if (ce_data->trickle_stats.soc_in != -1 || ce_data->bd_clear_trickle)
		len += gbms_tier_stats_cstr(&buff[len], size - len,
					    &ce_data->trickle_stats,
					    verbose);

	return len;
}

/* ------------------------------------------------------------------------- */

static int batt_ravg_value(const struct batt_res *rstate)
{
	return rstate->resistance_avg * 100;
}

static void batt_res_dump_logs(const struct batt_res *rstate)
{
	pr_info("RAVG: req:%d, sample:%d[%d], filt_cnt:%d, res_avg:%d\n",
		rstate->estimate_requested, rstate->sample_accumulator,
		rstate->sample_count, rstate->filter_count,
		rstate->resistance_avg);
}

static void batt_res_state_set(struct batt_res *rstate, bool breq)
{
	rstate->estimate_requested = breq;
	rstate->sample_accumulator = 0;
	rstate->sample_count = 0;
}

static int batt_ravg_write(int resistance_avg, int filter_count)
{
	const u16 ravg = (resistance_avg > 0xffff ) ?  0xffff : resistance_avg;
	const u16 rfcn = filter_count & 0xffff;
	int ret;

	ret = gbms_storage_write(GBMS_TAG_RAVG, &ravg, sizeof(ravg));
	if (ret < 0) {
		pr_debug("RAVG: failed to write RAVG (%d)\n", ret);
		return -EIO;
	}

	/*
	 * filter_count <= estimate_filter
	 * TODO: we might not need this...
	 * TODO: check the error path (ravg saved but filter count not saved)
	 */
	ret = gbms_storage_write(GBMS_TAG_RFCN, &rfcn, sizeof(rfcn));
	if (ret < 0) {
		pr_debug("RAVG: failed to write RFCN (%d)\n", ret);
		return -EIO;
	}

	return 0;
}

static void batt_res_update(struct batt_res *rstate)
{
	int filter_estimate = 0;
	int total_estimate = 0;
	long new_estimate = 0;

	/* accumulator is scaled */
	new_estimate = rstate->sample_accumulator / rstate->sample_count;
	filter_estimate = rstate->resistance_avg * rstate->filter_count;

	rstate->filter_count++;
	if (rstate->filter_count > rstate->estimate_filter) {
		rstate->filter_count = rstate->estimate_filter;
		filter_estimate -= rstate->resistance_avg;
	}

	total_estimate = filter_estimate + new_estimate;
	rstate->resistance_avg = total_estimate / rstate->filter_count;
}

static int batt_res_load_data(struct batt_res *rstate,
			      struct power_supply *fg_psy)
{
	u16 resistance_avg = 0, filter_count = 0;
	int ret;

	ret = gbms_storage_read(GBMS_TAG_RAVG, &resistance_avg,
				sizeof(resistance_avg));
	if (ret < 0) {
		pr_err("failed to get resistance_avg(%d)\n", ret);
		goto error_done;
	}

	ret = gbms_storage_read(GBMS_TAG_RFCN, &filter_count,
				sizeof(filter_count));
	if (ret < 0) {
		pr_err("failed to get resistance filt_count(%d)\n", ret);
		goto error_done;
	}

	/* no value in storage: start now (or start over) */
	if (resistance_avg == 0xffff || filter_count == 0xffff) {
		resistance_avg = 0;
		filter_count = 0;
	}

error_done:
	rstate->resistance_avg = resistance_avg;
	rstate->filter_count = filter_count;
	return 0;
}

/*
 * accumulate and resistance when SOC is between ravg_soc_low and ravg_soc_high
 * and temperature is in the right range. Discard the new sample if the device
 * is disconencted.
 * hold mutex_unlock(&batt_drv->chg_lock);
 */
static void batt_res_work(struct batt_drv *batt_drv)
{
	struct batt_res *rstate = &batt_drv->health_data.bhi_data.res_state;
	const int soc = ssoc_get_real(&batt_drv->ssoc_state);
	struct power_supply *fg_psy = batt_drv->fg_psy;
	int ret, temp, resistance;

	if (soc >= rstate->ravg_soc_high) {

		/* done: recalculate resistance_avg and save it */
		if (rstate->sample_count > 0) {
			batt_res_update(rstate);

			ret = batt_ravg_write(rstate->resistance_avg,
					      rstate->filter_count);
			if (ret == 0)
				batt_res_dump_logs(rstate);
		}

		/* loose the new data when it cannot save */
		batt_res_state_set(rstate, false);
		return;
	}

	/* wait for it */
	if (soc < rstate->ravg_soc_low)
		return;

	/* do not collect samples when temperature is outside the range */
	ret = gbatt_get_raw_temp(batt_drv, &temp);
	if (ret < 0 || temp < rstate->res_temp_low || temp > rstate->res_temp_high)
		return;

	/* resistance in mOhm, skip read errors */
	resistance = GPSY_GET_INT_PROP(fg_psy, GBMS_PROP_RESISTANCE, &ret);
	if (ret < 0)
		return;

	/* accumulate samples if temperature and SOC are within range */
	rstate->sample_accumulator += resistance / 100;
	rstate->sample_count++;
	pr_debug("RAVG: sample:%d[%d], filt_cnt:%d\n",
		 rstate->sample_accumulator, rstate->sample_count,
		 rstate->filter_count);
}

/* ------------------------------------------------------------------------- */

static int batt_csi_status_mask(void *data, const char *reason, void *vote)
{
	uint16_t *aggregate_status = data;
	int status = (long)vote;
	uint16_t status_mask = 0;

	switch (status) {
	case CSI_STATUS_UNKNOWN:
		status_mask = CSI_STATUS_MASK_UNKNOWN;
		break;
	case CSI_STATUS_Health_Cold:
		status_mask = CSI_STATUS_MASK_HEALTH_COLD;
		break;
	case CSI_STATUS_Health_Hot:
		status_mask = CSI_STATUS_MASK_HEALTH_HOT;
		break;
	case CSI_STATUS_System_Thermals:
		status_mask = CSI_STATUS_MASK_SYS_THERMALS;
		break;
	case CSI_STATUS_System_Load:
		status_mask = CSI_STATUS_MASK_SYS_LOAD;
		break;
	case CSI_STATUS_Adapter_Auth:
		status_mask = CSI_STATUS_MASK_ADA_AUTH;
		break;
	case CSI_STATUS_Adapter_Power:
		status_mask = CSI_STATUS_MASK_ADA_POWER;
		break;
	case CSI_STATUS_Adapter_Quality:
		status_mask = CSI_STATUS_MASK_ADA_QUALITY;
		break;
	case CSI_STATUS_Defender_Temp:
		status_mask = CSI_STATUS_MASK_DEFEND_TEMP;
		break;
	case CSI_STATUS_Defender_Dwell:
		status_mask = CSI_STATUS_MASK_DEFEND_DWELL;
		break;
	case CSI_STATUS_Defender_Trickle:
		status_mask = CSI_STATUS_MASK_DEFEND_TRICLE;
		break;
	case CSI_STATUS_Defender_Dock:
		status_mask = CSI_STATUS_MASK_DEFEND_DOCK;
		break;
	case CSI_STATUS_NotCharging:
		status_mask = CSI_STATUS_MASK_NOTCHARGING;
		break;
	case CSI_STATUS_Charging:
		status_mask = CSI_STATUS_MASK_CHARGING;
		break;
	case CSI_STATUS_Defender_Limit:
		status_mask = CSI_STATUS_MASK_DEFEND_LIMIT;
		break;
	default:
		break;
	}

	*aggregate_status |= status_mask;

	return 0;
}

static int batt_csi_type_mask(void *data, const char *reason, void *vote)
{
	uint16_t *aggregate_type = data;
	int type = (long)vote;
	uint16_t type_mask = 0;

	switch (type) {
	case CSI_TYPE_UNKNOWN:
		type_mask = CSI_TYPE_MASK_UNKNOWN;
		break;
	case CSI_TYPE_None:
		type_mask = CSI_TYPE_MASK_NONE;
		break;
	case CSI_TYPE_Fault:
		type_mask = CSI_TYPE_MASK_FAULT;
		break;
	case CSI_TYPE_JEITA:
		type_mask = CSI_TYPE_MASK_JEITA;
		break;
	case CSI_TYPE_LongLife:
		type_mask = CSI_TYPE_MASK_LONGLIFE;
		break;
	case CSI_TYPE_Adaptive:
		type_mask = CSI_TYPE_MASK_ADAPTIVE;
		break;
	case CSI_TYPE_Normal:
		type_mask = CSI_TYPE_MASK_NORMAL;
		break;
	default:
		break;
	}

	*aggregate_type |= type_mask;

	return 0;
}

static void batt_init_csi_stat(struct batt_drv *batt_drv)
{
	struct csi_stats *csi_stats = &batt_drv->csi_stats;

	csi_stats->vol_in = 0;
	csi_stats->cc_in = 0;
	csi_stats->ssoc_in = 0;
	csi_stats->cc_out = 0;
	csi_stats->vol_out = 0;
	csi_stats->ssoc_out = 0;
	csi_stats->temp_min = 0;
	csi_stats->temp_max = 0;
	csi_stats->time_sum = 0;
	csi_stats->time_effective = 0;
	csi_stats->time_stat_last_update = 0;
	csi_stats->aggregate_type = 0;
	csi_stats->aggregate_status = 0;
	memset(csi_stats->thermal_severity, 0,
	       sizeof(csi_stats->thermal_severity));


}

static void batt_update_csi_stat(struct batt_drv *batt_drv)
{
	const union gbms_ce_adapter_details *ad = &batt_drv->ce_data.adapter_details;
	const int ssoc = ssoc_get_capacity(&batt_drv->ssoc_state);
	struct csi_stats *csi_stats = &batt_drv->csi_stats;
	struct power_supply *fg_psy = batt_drv->fg_psy;
	const int8_t batt_temp = batt_drv->batt_temp / 10;
	const ktime_t now = get_boot_sec();
	int thermal_level = 0;
	ktime_t elap;

	if (chg_state_is_disconnected(&batt_drv->chg_state)) {
		/* Only update CSI stats when plugged and charging */
		if(csi_stats->time_stat_last_update == 0)
			return;

		/* update disconnected data */
		if(csi_stats->vol_in != 0) {
			const int vol_out = GPSY_GET_PROP(fg_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW);
			const int cc_out = GPSY_GET_PROP(fg_psy, POWER_SUPPLY_PROP_CHARGE_COUNTER);

			if (vol_out < 0 || cc_out < 0)
				return;

			csi_stats->vol_out = vol_out / 1000;
			csi_stats->cc_out = cc_out / 1000;
			csi_stats->ssoc_out = ssoc;

			logbuffer_log(batt_drv->ttf_stats.ttf_log,
				"csi_stats: %s,%d,%d,%d,%d,%lld,%d,%d,%lld,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
				 gbms_chg_ev_adapter_s(csi_stats->ad_type), csi_stats->ad_voltage * 100,
				 csi_stats->ad_amperage * 100, csi_stats->ssoc_in, csi_stats->ssoc_out,
				 csi_stats->time_sum / 60, csi_stats->aggregate_type, csi_stats->aggregate_status,
				 csi_stats->time_effective / 60, csi_stats->temp_min, csi_stats->temp_max,
				 csi_stats->vol_in, csi_stats->vol_out, csi_stats->cc_in, csi_stats->cc_out,
				 (int)(csi_stats->thermal_severity[0] * 100 / csi_stats->time_sum),
				 (int)(csi_stats->thermal_severity[1] * 100 / csi_stats->time_sum),
				 (int)(csi_stats->thermal_severity[2] * 100 / csi_stats->time_sum),
				 (int)(csi_stats->thermal_severity[3] * 100 / csi_stats->time_sum),
				 (int)(csi_stats->thermal_severity[4] * 100 / csi_stats->time_sum));
			return;
		}
	}

	/* initial connected data */
	if (csi_stats->time_stat_last_update == 0) {
		const int vol_in = GPSY_GET_PROP(fg_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW);
		const int cc_in = GPSY_GET_PROP(fg_psy, POWER_SUPPLY_PROP_CHARGE_COUNTER);

		if (vol_in < 0 || cc_in < 0)
			return;

		csi_stats->ssoc_in = ssoc;
		csi_stats->temp_min = batt_temp;
		csi_stats->temp_max = batt_temp;
		csi_stats->vol_in =  vol_in / 1000;
		csi_stats->cc_in = cc_in / 1000;
		csi_stats->time_stat_last_update = now;
	}

	gvotable_election_for_each(batt_drv->csi_type_votable, batt_csi_type_mask,
				   &csi_stats->aggregate_type);
	gvotable_election_for_each(batt_drv->csi_status_votable, batt_csi_status_mask,
				   &csi_stats->aggregate_status);
	elap = now - csi_stats->time_stat_last_update;
	csi_stats->time_sum += elap;
	csi_stats->ad_type = ad->ad_type;
	csi_stats->ad_voltage = ad->ad_voltage;
	csi_stats->ad_amperage = ad->ad_amperage;
	if (batt_drv->csi_current_type == CSI_TYPE_Normal && ssoc != 100)
		csi_stats->time_effective += elap;

	if (batt_temp < csi_stats->temp_min)
		csi_stats->temp_min = batt_temp;
	if (batt_temp > csi_stats->temp_max)
		csi_stats->temp_max = batt_temp;

	thermal_level = batt_get_thermal_level(batt_drv);
	if (thermal_level >= CSI_THERMAL_SEVERITY_MAX)
		thermal_level = CSI_THERMAL_SEVERITY_MAX - 1;
	if (thermal_level < 0)
		thermal_level = 0;

	csi_stats->thermal_severity[thermal_level] += elap;
	csi_stats->time_stat_last_update = now;
}

static void batt_log_csi_ttf_info(struct batt_drv *batt_drv)
{
	struct csi_stats *csi_stats = &batt_drv->csi_stats;
	const bool same_type_and_status =
		csi_stats->csi_current_type == batt_drv->csi_current_type &&
		csi_stats->csi_current_status == batt_drv->csi_current_status;
	int current_speed = batt_drv->csi_current_speed;
	int min_speed = csi_stats->csi_speed_min;
	int max_speed = csi_stats->csi_speed_max;
	const ktime_t right_now = get_boot_sec();
	int ssoc = -1;

	if (!batt_drv->init_complete)
		return;

	/* if record disconnected data, wait clear for next session */
	if (csi_stats->vol_out == 0)
		batt_update_csi_stat(batt_drv);

	if (chg_state_is_disconnected(&batt_drv->chg_state))
		goto log_and_done;

	ssoc = ssoc_get_capacity(&batt_drv->ssoc_state);
	if (ssoc == csi_stats->ssoc && same_type_and_status) {
		const ktime_t elap = right_now - csi_stats->last_update;

		csi_stats->last_update = right_now;

		/* accumulate only positive*/
		if (current_speed < 0)
			return;

		if (min_speed == max_speed && max_speed == 0)
			min_speed = max_speed = current_speed;
		else if (current_speed < min_speed)
			min_speed = current_speed;
		else if (current_speed > max_speed)
			max_speed = current_speed;

		csi_stats->csi_speed_min = min_speed;
		csi_stats->csi_speed_max = max_speed;
		csi_stats->speed_sum += current_speed * elap;
		csi_stats->csi_time_sum += elap;
		return;
	}

log_and_done:
	csi_stats->csi_current_status = batt_drv->csi_current_status;
	csi_stats->csi_current_type = batt_drv->csi_current_type;

	if (csi_stats->ssoc != -1) {
		const int csi_speed_avg = csi_stats->csi_time_sum == 0 ?
					  ((csi_stats->csi_speed_min + csi_stats->csi_speed_max) / 2) :
					  (csi_stats->speed_sum / csi_stats->csi_time_sum);
		const int cc = GPSY_GET_PROP(batt_drv->fg_psy, POWER_SUPPLY_PROP_CHARGE_COUNTER);
		ktime_t res = 0;
		const int max_ratio = batt_ttf_estimate(&res, batt_drv);
		u64 hours = 0;
		u32 remaining_sec = 0;

		if (max_ratio < 0)
			res = 0;
		if (res > 0)
			hours = div_u64_rem(res, 3600, &remaining_sec);

		gbms_logbuffer_prlog(batt_drv->ttf_stats.ttf_log, LOGLEVEL_INFO, 0, LOGLEVEL_DEBUG,
				     "ssoc=%d temp=%d CSI[speed=%d,%d,%d type=%d status=%d lvl=%d,%d"
				     " TTF[cc=%d time=%lld %lld:%d:%d (est=%lld max_ratio=%d)]",
				     csi_stats->ssoc, batt_drv->batt_temp, csi_speed_avg,
				     csi_stats->csi_speed_min, csi_stats->csi_speed_max,
				     csi_stats->csi_current_type, csi_stats->csi_current_status,
				     csi_stats->thermal_lvl_min, csi_stats->thermal_lvl_max,
				     cc / 1000, right_now, hours, remaining_sec / 60,
				     remaining_sec % 60, res, max_ratio);

	}

	/* ssoc == -1 on disconnect */
	if (ssoc == -1 || current_speed < 0)
		current_speed = 0;

	if (ssoc == -1 || ssoc != csi_stats->ssoc) {
		csi_stats->thermal_lvl_min = 0;
		csi_stats->thermal_lvl_max = 0;
	}

	csi_stats->ssoc = ssoc;
	csi_stats->csi_speed_min = current_speed;
	csi_stats->csi_speed_max = current_speed;

	/* ssoc == -1 on disconnect */
	if (ssoc == -1) {
		csi_stats->thermal_lvl_min = 0;
		csi_stats->thermal_lvl_max = 0;
	}

	csi_stats->csi_time_sum = 0;
	csi_stats->speed_sum = 0;
	csi_stats->last_update = right_now;
}

static int csi_status_cb(struct gvotable_election *el, const char *reason,
			 void *value)
{
	struct batt_drv *batt_drv = gvotable_get_data(el);
	int status = GVOTABLE_PTR_TO_INT(value);

	if (!batt_drv || batt_drv->csi_current_status == status)
		return 0;

	batt_drv->csi_current_status = status;
	batt_log_csi_ttf_info(batt_drv);

	if (batt_drv->psy)
		power_supply_changed(batt_drv->psy);

	return 0;
}

static int csi_type_cb(struct gvotable_election *el, const char *reason,
		       void *value)
{
	struct batt_drv *batt_drv = gvotable_get_data(el);
	int type = GVOTABLE_PTR_TO_INT(value);

	if (!batt_drv || batt_drv->csi_current_type == type)
		return 0;

	batt_drv->csi_current_type = type;
	batt_log_csi_ttf_info(batt_drv);

	if (batt_drv->psy)
		power_supply_changed(batt_drv->psy);

	return 0;
}
static bool batt_is_trickle(struct batt_ssoc_state *ssoc_state)
{
	return ssoc_state->bd_trickle_cnt > 0 &&
		ssoc_state->bd_trickle_enable &&
		!ssoc_state->bd_trickle_dry_run;
}

static bool batt_csi_status_is_dock(const struct batt_drv *batt_drv)
{
	int dock_status;

	if (!batt_drv->csi_status_votable)
		return false;

	dock_status = gvotable_get_int_vote(batt_drv->csi_status_votable, "CSI_STATUS_DEFEND_DOCK");
	return dock_status == CSI_STATUS_Defender_Dock;
}

/* all reset on disconnect */
static void batt_update_csi_type(struct batt_drv *batt_drv)
{
	const bool is_disconnected = chg_state_is_disconnected(&batt_drv->chg_state);
	const bool is_trickle = batt_is_trickle(&batt_drv->ssoc_state);
	const bool is_ac = batt_drv->msc_state == MSC_HEALTH ||
			batt_drv->msc_state == MSC_HEALTH_PAUSE ||
			batt_drv->msc_state == MSC_HEALTH_ALWAYS_ON;
	const bool is_dock = batt_csi_status_is_dock(batt_drv);

	if (!batt_drv->csi_type_votable) {
		batt_drv->csi_type_votable =
			gvotable_election_get_handle(VOTABLE_CSI_TYPE);
		if (!batt_drv->csi_type_votable)
			return;
	}

	/* normal or full if connected, nothing otherwise */
	gvotable_cast_long_vote(batt_drv->csi_type_votable, "CSI_TYPE_CONNECTED",
				(is_disconnected && !is_dock) ? CSI_TYPE_None : CSI_TYPE_Normal,
				true);

	/* SW JEITA */
	gvotable_cast_long_vote(batt_drv->csi_type_votable, "CSI_TYPE_JEITA",
				CSI_TYPE_JEITA,
				!is_disconnected && batt_drv->jeita_stop_charging == 1);

	/* Longlife is set on TEMP, DWELL and TRICKLE */
	gvotable_cast_long_vote(batt_drv->csi_type_votable, "CSI_TYPE_TRICKLE",
				CSI_TYPE_LongLife,
				!is_disconnected && is_trickle);

	/* Adaptive charging, individually */
	gvotable_cast_long_vote(batt_drv->csi_type_votable, "CSI_TYPE_AC",
				CSI_TYPE_Adaptive,
				!is_disconnected && is_ac);

	/* CSI_TYPE_Fault is permanent TODO: check single cell disconnect */
	gvotable_cast_long_vote(batt_drv->csi_type_votable, "CSI_TYPE_SINGLE_CELL",
				CSI_TYPE_Fault, false);
}

static bool batt_csi_check_ad_qual(const struct batt_drv *chg_drv)
{
	return false; /* TODO */
}

/*
 * these are absolute values: an underpowered adapter is a problem when
 * charging speed falls under 80%.
 */
static bool batt_csi_check_ad_power(const union gbms_ce_adapter_details *ad)
{
	const unsigned int ad_mw = (ad->ad_voltage * ad->ad_amperage) * 10000;
	unsigned int limit_mw = 9000 * 2000;	/* 18 Watts: it changes with the device */

	switch (ad->ad_type) {
	case CHG_EV_ADAPTER_TYPE_USB:
	case CHG_EV_ADAPTER_TYPE_USB_SDP:
	case CHG_EV_ADAPTER_TYPE_USB_CDP:
	case CHG_EV_ADAPTER_TYPE_USB_ACA:
	case CHG_EV_ADAPTER_TYPE_USB_C:
	case CHG_EV_ADAPTER_TYPE_USB_PD:
	case CHG_EV_ADAPTER_TYPE_USB_PD_DRP:
	case CHG_EV_ADAPTER_TYPE_USB_PD_PPS:
	case CHG_EV_ADAPTER_TYPE_USB_BRICKID:
	case CHG_EV_ADAPTER_TYPE_USB_HVDCP:
	case CHG_EV_ADAPTER_TYPE_USB_HVDCP3:
		break;
	case CHG_EV_ADAPTER_TYPE_WLC:
	case CHG_EV_ADAPTER_TYPE_WLC_EPP:
	case CHG_EV_ADAPTER_TYPE_WLC_SPP:
		limit_mw = 7500000;
		break;
	case CHG_EV_ADAPTER_TYPE_EXT:
	case CHG_EV_ADAPTER_TYPE_EXT1:
	case CHG_EV_ADAPTER_TYPE_EXT2:
	case CHG_EV_ADAPTER_TYPE_EXT_UNKNOWN:
		limit_mw = 10500 * 1250;
		break;
	default:
		break;
	}

	return ad_mw < limit_mw;
}

/*
 * COLD and HOT are only at the limits, we might want to flag anything that is
 * not the reference tier instead.
 */
static void batt_update_csi_status(struct batt_drv *batt_drv)
{
	const struct gbms_chg_profile *profile = &batt_drv->chg_profile;
	const int temp_hot_idx = profile->temp_nb_limits - 1;
	const bool is_cold = batt_drv->batt_temp < profile->temp_limits[0];
	const bool is_hot = batt_drv->batt_temp >= profile->temp_limits[temp_hot_idx];
	const bool is_trickle = batt_is_trickle(&batt_drv->ssoc_state);
	const bool is_disconnected = chg_state_is_disconnected(&batt_drv->chg_state);
	const union gbms_ce_adapter_details *ad = &batt_drv->ce_data.adapter_details;

	if (!batt_drv->csi_status_votable) {
		batt_drv->csi_status_votable =
			gvotable_election_get_handle(VOTABLE_CSI_STATUS);
		if (!batt_drv->csi_status_votable)
			return;
	}

	/*
	 * discharging when the battery current is negative. There will likely
	 * be a more specific reason (e.g System_* or Adapter_* or one of
	 * Defender_*).
	 */

	/* Charging Status Health_Cold */
	gvotable_cast_long_vote(batt_drv->csi_status_votable, "CSI_STATUS_COLD",
				CSI_STATUS_Health_Cold,
				!is_disconnected && is_cold);

	/* Charging Status Health_Hot */
	gvotable_cast_long_vote(batt_drv->csi_status_votable, "CSI_STATUS_HOT",
				CSI_STATUS_Health_Hot,
				!is_disconnected && is_hot);

	/* looks at absolute power, it could look also look at golden adapter */
	gvotable_cast_long_vote(batt_drv->csi_status_votable, "CSI_STATUS_ADA_POWR",
				CSI_STATUS_Adapter_Power,
				!is_disconnected && !batt_drv->chg_done &&
				batt_csi_check_ad_power(ad));

	/* Adapter quality looks at input voltage and current */
	gvotable_cast_long_vote(batt_drv->csi_status_votable, "CSI_STATUS_ADA_QUAL",
				CSI_STATUS_Adapter_Quality,
				!is_disconnected && batt_csi_check_ad_qual(batt_drv));

	/* Charging Status Defender_Trickle */
	gvotable_cast_long_vote(batt_drv->csi_status_votable, "CSI_STATUS_DEFEND_TRICKLE",
				CSI_STATUS_Defender_Trickle,
				!is_disconnected && is_trickle);

	gvotable_cast_long_vote(batt_drv->csi_status_votable, "CSI_STATUS_DSG",
				CSI_STATUS_NotCharging,
				!is_disconnected && batt_drv->msc_state == MSC_DSG &&
				!batt_drv->chg_done);

	gvotable_cast_long_vote(batt_drv->csi_status_votable, "CSI_STATUS_100",
				CSI_STATUS_Charging,
				!is_disconnected && batt_drv->batt_full && !batt_drv->chg_done);

	gvotable_cast_long_vote(batt_drv->csi_status_votable, "CSI_STATUS_CHG",
				CSI_STATUS_Charging, !is_disconnected);
}

#define CSI_CHG_SPEED_MAX 100
#define CSI_CHG_SPEED_MIN 0

/*
 * slowing down due to batt_drv->temp_idx != from reference is reported
 * in status as CSI_STATUS_COLD or CSI_STATUS_HOT.
 *
 *	cc_max = GBMS_CCCM_LIMITS(profile, batt_drv->temp_idx, batt_drv->vbatt_idx);
 *	if (cc_max && cc_max < nominal_demand)
 *		nominal_demand = cc_max;
 */
static int batt_calc_charging_speed(struct batt_drv *batt_drv)
{
	const struct gbms_chg_profile *profile = &batt_drv->chg_profile;
	const int soc = ssoc_get_capacity(&batt_drv->ssoc_state);
	const int chg_type = batt_drv->chg_state.f.chg_type;
	int cc_max, vbatt_idx, ibatt, nominal_demand;
	int chg_speed = -1;

	if (chg_state_is_disconnected(&batt_drv->chg_state))
		return -1;

	if (batt_drv->fake_charging_speed)
		return batt_drv->fake_charging_speed;

	/* if the battery is the limit, speed is 100% */
	if (chg_type == POWER_SUPPLY_CHARGE_TYPE_TAPER_EXT)
		return 100;

	/* Get average current via tiers. */
	vbatt_idx = ttf_pwr_vtier_idx(&batt_drv->ttf_stats, soc);

	/* Wait 1 min to get avg_ibat */
	ibatt = ttf_pwr_ibatt(&batt_drv->ce_data.tier_stats[vbatt_idx]);
	if (ibatt == 0)
		return -1;

	/* Get nominal demand current via ttf table */
	nominal_demand = ttf_ref_cc(&batt_drv->ttf_stats, soc);
	if (nominal_demand <= 0)
		return -1;

	/*
	 * Adjust demand to battery temperature when batt_drv->temp_idx is
	 * different from the reference. Here status will either be *_COLD
	 * or *_HOT.
	 */
	cc_max = GBMS_CCCM_LIMITS(profile, batt_drv->temp_idx, batt_drv->vbatt_idx) / 1000;
	if (cc_max && cc_max < nominal_demand)
		nominal_demand = cc_max;

	chg_speed = ibatt * 100 / nominal_demand;

	pr_debug("chg_speed=%d ibatt=%d nominal_demand=%d cc_max=%d",
		 chg_speed, ibatt, nominal_demand, cc_max);

	/* bound in [0,100] */
	if (chg_speed > CSI_CHG_SPEED_MAX)
		chg_speed = CSI_CHG_SPEED_MAX;
	else if (chg_speed < CSI_CHG_SPEED_MIN)
		chg_speed = CSI_CHG_SPEED_MIN;

	return chg_speed;
}

static void batt_update_thermal_lvl(struct batt_drv *batt_drv)
{
	struct csi_stats *csi_stats = &batt_drv->csi_stats;
	int thermal_level = 0;

	if (chg_state_is_disconnected(&batt_drv->chg_state))
		return;

	if (!batt_drv->thermal_level_votable)
		batt_drv->thermal_level_votable = gvotable_election_get_handle(VOTABLE_THERMAL_LVL);
	if (batt_drv->thermal_level_votable)
		thermal_level = gvotable_get_current_int_vote(batt_drv->thermal_level_votable);

	if (thermal_level < 0)
		return;

	if (csi_stats->thermal_lvl_max == 0 && csi_stats->thermal_lvl_min == 0)
		csi_stats->thermal_lvl_max = csi_stats->thermal_lvl_min = thermal_level;
	else if (thermal_level > csi_stats->thermal_lvl_max)
		csi_stats->thermal_lvl_max = thermal_level;
	else if (thermal_level < csi_stats->thermal_lvl_min)
		csi_stats->thermal_lvl_min = thermal_level;
}

static void batt_update_csi_info(struct batt_drv *batt_drv)
{
	int charging_speed;

	batt_update_csi_type(batt_drv);
	batt_update_csi_status(batt_drv);
	batt_update_thermal_lvl(batt_drv);

	charging_speed = batt_calc_charging_speed(batt_drv);
	if (batt_drv->csi_current_speed != charging_speed) {
		batt_drv->csi_current_speed = charging_speed;
		batt_log_csi_ttf_info(batt_drv);
	}
}

/* ------------------------------------------------------------------------- */

/* NOTE: should not reset always_on_soc */
static inline void batt_reset_rest_state(struct batt_chg_health *chg_health)
{
	chg_health->rest_cc_max = -1;
	chg_health->rest_fv_uv = -1;

	/* Keep negative deadlines (they mean user has disabled via settings)
	 * NOTE: CHG_DEADLINE_DIALOG needs to be applied only for the current
	 * session. Therefore, it should be cleared on disconnect.
	 */
	if (chg_health->rest_deadline < 0 &&
	    chg_health->rest_deadline != CHG_DEADLINE_DIALOG) {
		chg_health->rest_state = CHG_HEALTH_USER_DISABLED;
	} else {
		chg_health->rest_state = CHG_HEALTH_INACTIVE;
		chg_health->rest_deadline = 0;
	}

	chg_health->dry_run_deadline = 0;
	chg_health->active_time = 0;
}

/* should not reset rl state */
static inline void batt_reset_chg_drv_state(struct batt_drv *batt_drv)
{
	/* the wake assertion will be released on disconnect and on SW JEITA */
	if (batt_drv->hold_taper_ws) {
		batt_drv->hold_taper_ws = false;
		__pm_relax(batt_drv->taper_ws);
	}

	/* polling */
	batt_drv->batt_fast_update_cnt = 0;
	batt_drv->ttf_debounce = 1;
	batt_drv->fg_status = POWER_SUPPLY_STATUS_UNKNOWN;
	batt_drv->chg_done = false;
	batt_drv->ssoc_state.bd_trickle_full = false;
	batt_drv->ssoc_state.bd_trickle_eoc = false;
	/* algo */
	batt_drv->temp_idx = -1;
	batt_drv->vbatt_idx = -1;
	batt_drv->profile_vbatt_idx = -1;
	batt_drv->fv_uv = -1;
	batt_drv->cc_max = -1;
	batt_drv->cc_max_pullback = -1;
	batt_drv->msc_update_interval = -1;
	batt_drv->jeita_stop_charging = -1;
	/* timers */
	batt_drv->checked_cv_cnt = 0;
	batt_drv->checked_ov_cnt = 0;
	batt_drv->checked_tier_switch_cnt = 0;
	/* stats and logs */
	batt_drv->msc_state = -1;
	batt_drv->last_log_cnt = 0;
	/* health */
	batt_reset_rest_state(&batt_drv->chg_health);
	/* fan level */
	fan_level_reset(batt_drv);
}

/*
 * software JEITA, disable charging when outside the charge table.
 * NOTE: ->jeita_stop_charging is either -1 (init or reset), 1 (disable) or 0
 * TODO: need to be able to disable (leave to HW)
 */
static bool msc_logic_soft_jeita(struct batt_drv *batt_drv, int temp)
{
	const struct gbms_chg_profile *profile = &batt_drv->chg_profile;

	if (temp < profile->temp_limits[0] ||
	    temp >= profile->temp_limits[profile->temp_nb_limits - 1]) {
		if (batt_drv->jeita_stop_charging < 0) {
			batt_drv->jeita_stop_charging = 1;
			batt_prlog(BATT_PRLOG_ALWAYS,
				   "MSC_JEITA temp=%d off limits, do not enable charging\n",
				   temp);
		} else if (batt_drv->jeita_stop_charging == 0) {
			batt_prlog(BATT_PRLOG_ALWAYS,
				   "MSC_JEITA temp=%d off limits, disabling charging\n",
				    temp);
		}

		return true;
	}

	return false;
}

/* TODO: only change batt_drv->checked_ov_cnt, an */
static int msc_logic_irdrop(struct batt_drv *batt_drv,
			    int vbatt, int ibatt, int temp_idx,
			    int *vbatt_idx, int *fv_uv,
			    int *update_interval, int *p_cc_max_pullback)
{
	const struct gbms_chg_profile *profile = &batt_drv->chg_profile;
	const int vtier = profile->volt_limits[*vbatt_idx];
	const int chg_type = batt_drv->chg_state.f.chg_type;
	const int utv_margin = profile->cv_range_accuracy;
	const int otv_margin = profile->cv_otv_margin;
	const int switch_cnt = profile->cv_tier_switch_cnt;
	int vchg = batt_drv->chg_state.f.vchrg;
	int msc_state = MSC_NONE;
	bool match_enable;
	bool no_back_down = false;

	if (batt_drv->chg_state.f.flags & GBMS_CS_FLAG_DIRECT_CHG) {
		if (batt_drv->dc_irdrop)
			no_back_down = true;
		else
			vchg = 0;
	}
	match_enable = vchg != 0;

	if ((vbatt - vtier) > otv_margin) {
		/* OVER: vbatt over vtier for more than margin */
		const int cc_max = GBMS_CCCM_LIMITS(profile, temp_idx,
						    *vbatt_idx);

		/*
		 * pullback when over tier voltage, fast poll, penalty
		 * on TAPER_RAISE and no cv debounce (so will consider
		 * switching voltage tiers if the current is right).
		 * NOTE: lowering voltage might cause a small drop in
		 * current (we should remain  under next tier)
		 * TODO: the fv_uv_resolution might be different in
		 * main charger and CP (should separate them)
		 */
		*fv_uv = gbms_msc_round_fv_uv(profile, vtier,
					      *fv_uv - profile->fv_uv_resolution,
					      no_back_down ? cc_max : 0,
					      batt_drv->allow_higher_fv);
		if (*fv_uv < vtier)
			*fv_uv = vtier;

		*update_interval = profile->cv_update_interval;
		batt_drv->checked_ov_cnt = profile->cv_tier_ov_cnt;
		batt_drv->checked_cv_cnt = 0;

		if (batt_drv->checked_tier_switch_cnt > 0 || !match_enable) {
			/* no pullback, next tier if already counting */
			msc_state = MSC_VSWITCH;
			*vbatt_idx = batt_drv->vbatt_idx + 1;

			batt_prlog(BATT_PRLOG_ALWAYS,
				   "MSC_VSWITCH vt=%d vb=%d ibatt=%d me=%d\n",
				   vtier, vbatt, ibatt, match_enable);
		} else if (-ibatt == cc_max) {
			/* pullback, double penalty if at full current */
			msc_state = MSC_VOVER;
			batt_drv->checked_ov_cnt *= 2;

			batt_prlog(BATT_PRLOG_ALWAYS,
				   "MSC_VOVER vt=%d  vb=%d ibatt=%d fv_uv=%d->%d\n",
				   vtier, vbatt, ibatt,
				   batt_drv->fv_uv, *fv_uv);
		} else {
			/* simple pullback */
			msc_state = MSC_PULLBACK;
			if (no_back_down) {
				*fv_uv = batt_drv->fv_uv;
				if (batt_drv->pullback_current)
					*p_cc_max_pullback = -ibatt;
			}
			batt_prlog(BATT_PRLOG_ALWAYS,
				  "MSC_PULLBACK vt=%d vb=%d ibatt=%d fv_uv=%d->%d no_back=%d\n",
				  vtier, vbatt, ibatt,
				  batt_drv->fv_uv, *fv_uv, no_back_down);
		}

		/*
		 * might get here after windup because algo will track the
		 * voltage drop caused from load as IRDROP.
		 * TODO: make sure that being current limited clear
		 * the taper condition.
		 */

	} else if (chg_type == POWER_SUPPLY_CHARGE_TYPE_FAST) {
		/*
		 * FAST: usual compensation (vchrg is vqcom)
		 * NOTE: there is a race in reading from charger and
		 * data might not be consistent (b/110318684)
		 * NOTE: could add PID loop for management of thermals
		 */
		const int vchrg_uv = vchg * 1000;
		const int pre_fv = *fv_uv;

		msc_state = MSC_FAST;

		/* invalid or 0 vchg disable IDROP compensation */
		if (vchrg_uv <= 0) {
			/* could keep it steady instead */
			*fv_uv = vtier;
		} else if (vchrg_uv > vbatt) {
			const int cc_max = GBMS_CCCM_LIMITS(profile, temp_idx, *vbatt_idx);

			*fv_uv = gbms_msc_round_fv_uv(profile, vtier, vtier + (vchrg_uv - vbatt),
						      no_back_down ? cc_max : 0,
						      batt_drv->allow_higher_fv);
		}

		/* not allow to reduce fv in DC to avoid the VSWITCH */
		if (no_back_down && (pre_fv > *fv_uv))
			*fv_uv = pre_fv;

		/* no tier switch in fast charge (TODO unless close to tier) */
		if (batt_drv->checked_cv_cnt == 0)
			batt_drv->checked_cv_cnt = 1;

		batt_prlog(BATT_PRLOG_ALWAYS,
			   "MSC_FAST vt=%d vb=%d ib=%d fv_uv=%d->%d vchrg=%d cv_cnt=%d no_back=%d\n",
			   vtier, vbatt, ibatt, batt_drv->fv_uv, *fv_uv,
			   batt_drv->chg_state.f.vchrg,
			   batt_drv->checked_cv_cnt, no_back_down);

	} else if (chg_type == POWER_SUPPLY_CHARGE_TYPE_TRICKLE) {
		/*
		 * Precharge: charging current/voltage are limited in
		 * hardware, no point in applying irdrop compensation.
		 * Just wait for battery voltage to raise over the
		 * precharge to fast charge threshold.
		 */
		msc_state = MSC_TYPE;

		/* no tier switching in trickle */
		if (batt_drv->checked_cv_cnt == 0)
			batt_drv->checked_cv_cnt = 1;

		batt_prlog(BATT_PRLOG_ALWAYS, "MSC_PRE vt=%d vb=%d fv_uv=%d chg_type=%d\n",
			   vtier, vbatt, *fv_uv, chg_type);
	} else if (chg_type != POWER_SUPPLY_CHARGE_TYPE_TAPER_EXT) {
		const int type_margin = utv_margin;

		/*
		 * Not fast, taper or precharge: in *_UNKNOWN and *_NONE.
		 * Set checked_cv_cnt=0 when voltage is withing utv_margin of
		 * vtier (tune marging) to force checking current and avoid
		 * early termination for lack of headroom. Carry on at the
		 * same update_interval otherwise.
		 */
		msc_state = MSC_TYPE;
		if (vbatt > (vtier - type_margin)) {
			*update_interval = profile->cv_update_interval;
			batt_drv->checked_cv_cnt = 0;
		} else {
			batt_drv->checked_cv_cnt = 1;
		}

		batt_prlog(BATT_PRLOG_ALWAYS,
			   "MSC_TYPE vt=%d margin=%d cv_cnt=%d vb=%d fv_uv=%d chg_type=%d\n",
			   vtier, type_margin, batt_drv->checked_cv_cnt, vbatt,
			   *fv_uv, chg_type);

	} else if (batt_drv->checked_ov_cnt) {
		/*
		 * TAPER_DLY: countdown to raise fv_uv and/or check
		 * for tier switch, will keep steady...
		 */
		batt_prlog(BATT_PRLOG_ALWAYS,
			   "MSC_DLY vt=%d vb=%d fv_uv=%d margin=%d cv_cnt=%d, ov_cnt=%d\n",
			   vtier, vbatt, *fv_uv, profile->cv_range_accuracy,
			   batt_drv->checked_cv_cnt,
			   batt_drv->checked_ov_cnt);

		msc_state = MSC_DLY;
		batt_drv->checked_ov_cnt -= 1;
		*update_interval = profile->cv_update_interval;

	} else if ((vtier - vbatt) < utv_margin) {
		const bool log_level = batt_drv->msc_state != MSC_STEADY &&
				      batt_drv->msc_state != MSC_RSTC;

		/* TAPER_STEADY: close enough to tier */

		msc_state = MSC_STEADY;
		*update_interval = profile->cv_update_interval;

		batt_prlog(batt_prlog_level(log_level),
			   "MSC_STEADY vt=%d vb=%d fv_uv=%d margin=%d\n",
			   vtier, vbatt, *fv_uv,
			   profile->cv_range_accuracy);
	} else if (batt_drv->checked_tier_switch_cnt >= (switch_cnt - 1)) {
		/*
		 * TAPER_TIERCNTING: prepare to switch to next tier
		 * so not allow to raise vfloat to prevent battery
		 * voltage over than tier
		 */
		msc_state = MSC_TIERCNTING;
		*update_interval = profile->cv_update_interval;

		batt_prlog(BATT_PRLOG_ALWAYS,
			   "MSC_TIERCNTING vt=%d vb=%d fv_uv=%d margin=%d\n",
			   vtier, vbatt, *fv_uv,
			   profile->cv_range_accuracy);
	} else if (match_enable) {
		/*
		 * TAPER_RAISE: under tier vlim, raise one click &
		 * debounce taper (see above handling of STEADY)
		 */
		const int cc_max = GBMS_CCCM_LIMITS(profile, temp_idx, *vbatt_idx);

		msc_state = MSC_RAISE;
		*fv_uv = gbms_msc_round_fv_uv(profile, vtier,
					      *fv_uv + profile->fv_uv_resolution,
					      no_back_down ? cc_max : 0,
					      batt_drv->allow_higher_fv);
		*update_interval = profile->cv_update_interval;

		/* debounce next taper voltage adjustment */
		batt_drv->checked_cv_cnt = profile->cv_debounce_cnt;

		batt_prlog(BATT_PRLOG_ALWAYS, "MSC_RAISE vt=%d vb=%d fv_uv=%d->%d\n",
			   vtier, vbatt, batt_drv->fv_uv, *fv_uv);
	} else {
		msc_state = MSC_STEADY;
		batt_prlog(BATT_PRLOG_DEBUG, "MSC_DISB vt=%d vb=%d fv_uv=%d->%d\n",
			vtier, vbatt, batt_drv->fv_uv, *fv_uv);
	}

	return msc_state;
}

/* battery health based charging on SOC */
static enum chg_health_state msc_health_active(const struct batt_drv *batt_drv)
{
	int ssoc, ssoc_threshold = -1;

	ssoc_threshold = CHG_HEALTH_REST_SOC(&batt_drv->chg_health);
	if (ssoc_threshold < 0)
		return CHG_HEALTH_INACTIVE;

	ssoc = ssoc_get_capacity(&batt_drv->ssoc_state);
	if (ssoc >= ssoc_threshold)
		return CHG_HEALTH_ACTIVE;

	return CHG_HEALTH_ENABLED;
}

#define HEALTH_PAUSE_DEBOUNCE 180
#define HEALTH_PAUSE_MAX_SSOC 95
#define HEALTH_PAUSE_TIME 3
static bool msc_health_pause(struct batt_drv *batt_drv, const ktime_t ttf,
			      const ktime_t now,
			      const enum chg_health_state rest_state) {
	const struct gbms_charging_event *ce_data = &batt_drv->ce_data;
	const struct gbms_ce_tier_stats	*h = &ce_data->health_stats;
	struct batt_chg_health *rest = &batt_drv->chg_health;
	const ktime_t deadline = rest->rest_deadline;
	const ktime_t safety_margin = (ktime_t)batt_drv->health_safety_margin;
	/* Note: We only capture ACTIVE time in health stats */
	const ktime_t elap_h = h->time_fast + h->time_taper + h->time_other;
	const int ssoc = ssoc_get_capacity(&batt_drv->ssoc_state);

	/*
	 * the safety marging cannot be less than 0 (it would subtract time from TTF and would
         * cause AC to never meet 100% in time). Use 0<= to disable PAUSE.
	 */
	if (safety_margin <= 0)
		return false;

	/*
	 * Expected behavior:
	 * 1. ACTIVE: small current run a while for ttf
	 * 2. PAUSE: when time is enough to pause
	 * 3. ACTIVE: when time out and back to ACTIVE charge
	 */
	if (rest_state != CHG_HEALTH_ACTIVE && rest_state != CHG_HEALTH_PAUSE)
		return false;

	/*
	 * ssoc: transfer in high soc impact charge full condition, disable pause
	 * behavior in high soc
	 */
	if (ssoc > HEALTH_PAUSE_MAX_SSOC)
		return false;

	/*
	 * elap_h: running active for a while wait status and current stable
	 * need to re-check before re-enter pause, so we need to minus previous
	 * health active time (rest->active_time) for next HEALTH_PAUSE_DEBOUNCE
	 */
	if (elap_h - rest->active_time < HEALTH_PAUSE_DEBOUNCE)
		return false;

	/* prevent enter <---> leave PAUSE too many times */
	if (rest->active_time > (HEALTH_PAUSE_TIME * HEALTH_PAUSE_DEBOUNCE))
		return false;

	/* check if time meets the PAUSE condition or not */
	if (ttf > 0 && deadline > now + ttf + safety_margin)
		return true;

	/* record time for next pause check */
	rest->active_time = elap_h;

	return false;
}

/*
 * for logging, userspace should use
 *   deadline == 0 on fast replug (leave initial deadline ok)
 *   deadline == -1 when the feature is disabled
 *       if charge health was active/enabled, set to -2
 *   deadline == absolute requested deadline (if always_on is set)
 * return true if there was a change
 */
static bool batt_health_set_chg_deadline(struct batt_chg_health *chg_health,
					 long long ttf_with_margin,
					 long long deadline_s)
{
	enum chg_health_state rest_state = chg_health->rest_state;
	bool new_deadline;

	/* disabled in settings */
	if (deadline_s < 0) {
		new_deadline = chg_health->rest_deadline != deadline_s;
		chg_health->rest_state = CHG_HEALTH_USER_DISABLED;

		/* disabled with notification; assumes that the dialog exists
		 * only if there is a >0 deadline.
		 */
		if (deadline_s == CHG_DEADLINE_DIALOG)
			chg_health->rest_deadline = CHG_DEADLINE_DIALOG;
		else if (chg_health->rest_deadline > 0) /* was active */
			chg_health->rest_deadline = CHG_DEADLINE_SETTING_STOP;
		else
			chg_health->rest_deadline = CHG_DEADLINE_SETTING;

	/* disabled with replug */
	} else if (deadline_s == 0) {
		new_deadline = chg_health->rest_deadline != deadline_s;
		/* ->rest_deadline will be reset to 0 on disconnect */

		/* Don't disable A/C if already done */
		if (chg_health->rest_state != CHG_HEALTH_DONE)
			chg_health->rest_state = CHG_HEALTH_USER_DISABLED;

	} else { /* enabled from any previous state */
		const ktime_t rest_deadline = get_boot_sec() + deadline_s;
		enum chg_health_state new_rest_state = CHG_HEALTH_ENABLED;

		/* ->always_on SOC overrides the deadline */
		new_deadline = chg_health->rest_deadline != rest_deadline;

		if (rest_deadline < ttf_with_margin)
			new_rest_state = CHG_HEALTH_DISABLED;

		chg_health->rest_state = new_rest_state;
		chg_health->rest_deadline = rest_deadline;
	}

	return new_deadline || rest_state != chg_health->rest_state;
}

#define HEALTH_CHG_RATE_BEFORE_TRIGGER 80
/* health based charging trade charging speed for battery cycle life. */
static bool msc_logic_health(struct batt_drv *batt_drv)
{
	const struct gbms_chg_profile *profile = &batt_drv->chg_profile;
	struct batt_chg_health *rest = &batt_drv->chg_health;
	const ktime_t deadline = rest->rest_deadline;
	enum chg_health_state rest_state = rest->rest_state;
	const bool aon_enabled = rest->always_on_soc != -1;
	const int capacity_ma = batt_drv->battery_capacity;
	const ktime_t now = get_boot_sec();
	int fv_uv = -1, cc_max = -1;
	bool changed = false;
	ktime_t ttf = 0, safety_margin = 0;
	int ret;

	/* move to ENABLED if INACTIVE when aon_enabled is set */
	if (aon_enabled && rest_state == CHG_HEALTH_INACTIVE)
		rest_state = CHG_HEALTH_ENABLED;

	/*
	 * on disconnect batt_reset_rest_state() will set rest_state to
	 * CHG_HEALTH_USER_DISABLED if the deadline is negative.
	 */
	if (rest_state == CHG_HEALTH_CCLVL_DISABLED ||
	    rest_state == CHG_HEALTH_BD_DISABLED ||
	    rest_state == CHG_HEALTH_USER_DISABLED ||
	    rest_state == CHG_HEALTH_DISABLED ||
	    rest_state == CHG_HEALTH_INACTIVE)
		goto done_no_op;

	/* Keeps AC enabled after DONE */
	if (rest_state == CHG_HEALTH_DONE)
		goto done_exit;

	/* disable AC because we are running custom charging levels */
	if (batt_drv->chg_state.f.flags & GBMS_CS_FLAG_CCLVL) {
		rest_state = CHG_HEALTH_CCLVL_DISABLED;
		goto done_exit;
	}

	/* disable AC because BD-TEMP triggered */
	if (batt_drv->batt_health == POWER_SUPPLY_HEALTH_OVERHEAT) {
		rest_state = CHG_HEALTH_BD_DISABLED;
		goto done_exit;
	}

	/*
	 * ret < 0 right after plug-in or when the device is discharging due
	 * to a large sysload or an underpowered adapter (or both). Current
	 * strategy leaves everything as is (hoping) that the load is temporary.
	 * The estimate will be negative when BD is triggered and during the
	 * debounce period.
	 */
	ret = batt_ttf_estimate(&ttf, batt_drv);
	if (ret < 0)
		return false;

	/* estimate is 0 at 100%: set to done and keep AC enabled in RL */
	if (ttf == 0) {
		rest_state = CHG_HEALTH_DONE;
		goto done_exit;
	}

	if (batt_drv->health_safety_margin > 0)
		safety_margin = batt_drv->health_safety_margin;

	/*
	 * rest_state here is either ENABLED or ACTIVE, transition to DISABLED
	 * when the deadline cannot be met with the current rate. set a new
	 * deadline or reset always_on_soc to re-enable AC for this session.
	 * NOTE: A device with AON enabled might (will) receive a deadline if
	 * plugged in within the AC window: ignore it.
	 * NOTE: cannot have a negative deadline with rest_state different
	 * from CHG_HEALTH_USER_DISABLED.
	 * TODO: consider adding a margin or debounce it.
	 */
	if (aon_enabled == false &&
	    (rest_state == CHG_HEALTH_ACTIVE || rest_state == CHG_HEALTH_ENABLED) &&
	    deadline > 0 && ttf != -1 && now + ttf + safety_margin > deadline) {
		rest_state = CHG_HEALTH_DISABLED;
		goto done_exit;
	}

	/* Decide enter PAUSE state or not by time if not set ACA */
	if (aon_enabled == false &&
	    msc_health_pause(batt_drv, ttf, now, rest_state)) {
		rest_state = CHG_HEALTH_PAUSE;
		goto done_exit;
	}

	/*
	 * rest_state here is either ENABLED or ACTIVE,
	 * NOTE: State might transition from _ACTIVE to _ENABLED after a
	 * discharge cycle that makes the battery fall under the threshold.
	 * State will transition back to _ENABLED after some time unless
	 * the deadline is met.
	 */
	rest_state = msc_health_active(batt_drv);

done_exit:
	if (rest_state == CHG_HEALTH_ACTIVE || rest_state == CHG_HEALTH_DONE) {
		int last_voltage_idx = gbms_msc_get_last_voltage_idx(profile, batt_drv->temp_idx);

		/* cc_max in ua: capacity in mAh, rest_rate in deciPct */
		cc_max = capacity_ma * rest->rest_rate * 10;

		/*
		 * default FV_UV to the last charge tier since fv_uv will be
		 * set to that on _DONE.
		 * NOTE this might need to be adjusted for the actual charge
		 * tiers that have nonzero charging current
		 */
		fv_uv = profile->volt_limits[last_voltage_idx];

		/* TODO: make sure that we wakeup when we are close to ttf */
	} else if (rest_state == CHG_HEALTH_ENABLED) {
		cc_max = capacity_ma * rest->rest_rate_before_trigger * 10;
	} else if (rest_state == CHG_HEALTH_PAUSE) {
		/*
		 * pause charging behavior when the the deadline is longer than
		 * expected charge time. return back to CHG_HEALTH_ACTIVE and
		 * start health charge when now + ttf + margine close to deadline
		*/
		cc_max = 0;
	}

done_no_op:
	/* send a power supply event when rest_state changes */
	changed = rest->rest_state != rest_state ||
		  rest->rest_cc_max != cc_max || rest->rest_fv_uv != fv_uv;

	if (!changed)
		return false;

	/* msc_logic_* will vote on cc_max and fv_uv. */
	rest->rest_cc_max = cc_max;
	if (fv_uv > 0)
		aafv_update_float_voltage(batt_drv, &fv_uv);

	rest->rest_fv_uv = fv_uv;
	gbms_logbuffer_prlog(batt_drv->ttf_stats.ttf_log, LOGLEVEL_INFO, 0, LOGLEVEL_DEBUG,
			     "MSC_HEALTH: now=%lld deadline=%lld aon_soc=%d ttf=%lld state=%d->%d fv_uv=%d, cc_max=%d safety_margin=%d active_time:%lld",
			     now, rest->rest_deadline, rest->always_on_soc, ttf, rest->rest_state,
			     rest_state, fv_uv, cc_max, batt_drv->health_safety_margin,
			     rest->active_time);

	rest->rest_state = rest_state;
	memcpy(&batt_drv->ce_data.ce_health, &batt_drv->chg_health,
			sizeof(batt_drv->ce_data.ce_health));
	return true;
}

static int msc_pm_hold(int msc_state)
{
	int pm_state = -1;

	switch (msc_state) {
	case MSC_RAISE:
	case MSC_VOVER:
	case MSC_PULLBACK:
		pm_state = 1; /* __pm_stay_awake */
		break;
	case MSC_SEED:
	case MSC_DSG:
	case MSC_VSWITCH:
	case MSC_NEXT:
	case MSC_LAST:
	case MSC_RSTC:
	case MSC_HEALTH:
	case MSC_HEALTH_PAUSE:
	case MSC_HEALTH_ALWAYS_ON:
	case MSC_WAIT:
	case MSC_FAST:
	case MSC_NYET:
	case MSC_STEADY:
	case MSC_DONE:
		pm_state = 0;  /* pm_relax */
		break;
	default:
		pr_debug("hold not defined for msc_state=%d\n", msc_state);
		pm_state = 0;  /* pm_relax */
		break;
	}

	return pm_state;
}

/* same as design when under the grace period */
static u32 aacr_get_reference_capacity(const struct batt_drv *batt_drv, int cycle_count)
{
	const int design_capacity = batt_drv->battery_capacity;
	const int aacr_cycle_grace = batt_drv->aacr_cycle_grace;
	const int aacr_cycle_max = batt_drv->aacr_cycle_max;
	int fade10;

	fade10 = gbms_aacr_fade10(&batt_drv->chg_profile, cycle_count);
	if (fade10 >= 0) {
		/* use interpolation between known points */
	} else if (aacr_cycle_max && (cycle_count > aacr_cycle_grace)) {
		/* or use slope from ->aacr_cycle_grace for 20% @ ->aacr_cycle_max */
		fade10 = (200 * (cycle_count -  aacr_cycle_grace)) /
			 (aacr_cycle_max - aacr_cycle_grace);

		pr_debug("%s: aacr_cycle_max=%d, cycle_count=%d fade10=%d\n",
			 __func__, aacr_cycle_max, cycle_count, fade10);
	} else {
		fade10 = 0;
	}

	return design_capacity - (design_capacity * fade10 / 1000);
}

/* AACP ------------------------------------------------------------------- */

static int aacp_get_cc(const struct batt_drv *batt_drv)
{
	return batt_drv->fake_aacp_cc ? batt_drv->fake_aacp_cc : batt_drv->aacc;
}

/* must be called holding aacp_state_lock */
static void aacp_update_opt_out_cutoff(struct batt_drv *batt_drv)
{
	const int cycle_count = aacp_get_cc(batt_drv);

	if (batt_drv->aacp_opt_out_cut_off_cycles == AACP_OPT_OUT_CUT_OFF_DISABLED)
		return;

	if (cycle_count >= batt_drv->aacp_opt_out_cut_off_cycles)
		batt_drv->aacp_opt_out = BATT_AACP_OPT_OUT_DISABLED;
}

/* AACR ------------------------------------------------------------------- */

/* 80% of design_capacity min, design_capacity in grace, aacr or negative */
/* requires mutex_lock(&batt_drv->aacp_state_lock); */
static int aacr_get_capacity_for_algo(const struct batt_drv *batt_drv, int cycle_count,
				      int aacr_algo)
{
	const int design_capacity = batt_drv->battery_capacity; /* mAh */
	const int min_cap_rate = batt_drv->aacr_min_capacity_rate;
	const int min_capacity = (batt_drv->battery_capacity * min_cap_rate) / 100;
	const int cliff_cap_rate = batt_drv->aacr_cliff_capacity_rate;
	const int cliff_capacity = (batt_drv->battery_capacity * cliff_cap_rate) / 100;
	int reference_capacity, full_cap_nom, full_capacity;
	struct power_supply *fg_psy = batt_drv->fg_psy;
	int aacr_capacity;

	/* peg at 80% of design when over limit (if set) */
	if (batt_drv->aacr_cycle_max && (cycle_count >= batt_drv->aacr_cycle_max))
		return cliff_capacity;

	reference_capacity = aacr_get_reference_capacity(batt_drv, cycle_count);
	if (reference_capacity <= 0)
		return design_capacity;

	if (aacr_algo == BATT_AACR_ALGO_REFERENCE_CAP)
		return reference_capacity;

	/* full_cap_nom in uAh, need to scale to mAh */
	full_cap_nom = GPSY_GET_PROP(fg_psy, POWER_SUPPLY_PROP_CHARGE_FULL);
	if (full_cap_nom < 0)
		return full_cap_nom;
	full_cap_nom /= 1000;

	if (aacr_algo == BATT_AACR_ALGO_LOW_B)
		full_capacity = min(min(full_cap_nom, design_capacity), reference_capacity);
	else
		full_capacity = max(min(full_cap_nom, design_capacity), reference_capacity);

	aacr_capacity = max(full_capacity, min_capacity);
	aacr_capacity = (aacr_capacity / 50) * 50; /* 50mAh, ~1% capacity */

	pr_debug("%s: design=%d reference=%d full_cap_nom=%d full=%d aacr=%d algo=%d\n",
		 __func__, design_capacity, reference_capacity, full_cap_nom,
		 full_capacity, aacr_capacity, aacr_algo);

	return aacr_capacity;
}

/* requires mutex_lock(&batt_drv->aacp_state_lock); */
static int aacr_get_capacity_at_cycle(const struct batt_drv *batt_drv, int cycle_count)
{
	const int design_capacity = batt_drv->battery_capacity; /* mAh */

	/* batt_drv->cycle_count might be negative */
	if (cycle_count <= batt_drv->aacr_cycle_grace)
		return design_capacity;

	return aacr_get_capacity_for_algo(batt_drv, cycle_count, batt_drv->aacr_algo);
}

/* design_capacity when not enabled, never a negative value */
/* requires mutex_lock(&batt_drv->aacp_state_lock); */
static u32 aacr_get_capacity_locked(struct batt_drv *batt_drv)
{
	int capacity = batt_drv->battery_capacity;
	int cycle_count;

	if (aacr_enabled(batt_drv->aacr_state, batt_drv->aacp_opt_out) == false)
		goto exit_done;

	cycle_count = aacp_get_cc(batt_drv);
	if (cycle_count <= batt_drv->aacr_cycle_grace) {
		batt_drv->aacr_phase = BATT_AACR_UNDER_CYCLES;
	} else {
		int aacr_capacity;

		aacr_capacity = aacr_get_capacity_at_cycle(batt_drv, cycle_count);
		if (aacr_capacity < 0) {
			batt_drv->aacr_phase = BATT_AACR_INVALID_CAP;
		} else {
			batt_drv->aacr_phase = BATT_AACR_ENABLED;
			capacity = aacr_capacity;
		}
	}

exit_done:
	return (u32)capacity;
}

static u32 aacr_filtered_capacity(struct batt_drv *batt_drv, struct gbms_charging_event *ce)
{
	const bool enabled = aacr_enabled(batt_drv->aacr_state, batt_drv->aacp_opt_out);

	return enabled ? ce->chg_profile->capacity_ma : batt_drv->aacr_state;
}

/* BHI -------------------------------------------------------------------- */

/*
 * Format of XYMD:
 *   XYMD[0]: YEAR (1 CHARACTER, NUMERIC; LAST DIGIT OF YEAR)
 *   XYMD[1]: Month(1 Character ASCII,Alphanumeric;1-9 For Han-Sept,A=Oct,B=Nov,C=Dec)
 *   XYMD[2]: Day(1 Character ASCII,Alphanumeric;1-9 For 1st-9th, A=10th,B=11th,...X=31st;
 *            skip the letter “I” and the letter”O”)
 */
static inline int date_to_xymd(u8 val)
{
	if (val >= 23)
		return (val - 23) + 0x50; /* 0x50 = 'P', 23th */
	else if (val >= 18)
		return (val - 18) + 0x4a; /* 0x4a = 'J', 18th*/
	else if (val >= 10)
		return (val - 10) + 0x41; /* 0x41 = 'A', 10th*/
	else
		return (val + 0x30);
}

static inline int xymd_to_date(u8 val)
{
	if (val >= 0x50)
		return (val - 0x50) + 23; /* 0x50 = 'P', 23th */
	else if (val >= 0x4a)
		return (val - 0x4a) + 18; /* 0x4a = 'J', 18th*/
	else if (val >= 0x41)
		return (val - 0x41) + 10; /* 0x41 = 'A', 10th*/
	else
		return (val - 0x30);
}

static int batt_get_manufacture_date(struct bhi_data *bhi_data)
{
	struct bm_date *date = &bhi_data->bm_date;
	u8 data[BATT_EEPROM_TAG_XYMD_LEN];
	int ret = 0;

	ret = gbms_storage_read(GBMS_TAG_MYMD, data, sizeof(data));
	if (ret < 0)
		return ret;

	/* format: YYMMDD */
	date->bm_y = xymd_to_date(data[0]) + 20;
	date->bm_m = xymd_to_date(data[1]);
	date->bm_d = xymd_to_date(data[2]);
	pr_debug("%s: battery manufacture date: 20%d-%d-%d\n",
		 __func__, date->bm_y, date->bm_m, date->bm_d);

	return 0;
}

static int batt_mdate_to_epoch(struct bhi_data *bhi_data)
{

	struct bm_date *date = &bhi_data->bm_date;
	struct rtc_time tm;

	tm.tm_year = date->bm_y + 100;	// base is 1900
	tm.tm_mon = date->bm_m - 1;	// 0 is Jan ... 11 is Dec
	tm.tm_mday = date->bm_d;	// 1st ... 31th

	return rtc_tm_to_time64(&tm);
}

static int batt_get_activation_date(struct bhi_data *bhi_data)
{
	int ret;

	ret = gbms_storage_read(GBMS_TAG_AYMD, &bhi_data->act_date,
				sizeof(bhi_data->act_date));
	if (ret < 0)
		return ret;

	if (bhi_data->act_date[0] == 0xff) {
		/*
		 * TODO: set a default value
		 * might be set by first_usage_date_show() from user space
		 */
		bhi_data->act_date[0] = 0x30; /* 0x30 = '0', 2020 */
		bhi_data->act_date[1] = 0x43; /* 0x43 = 'C', 12th */
		bhi_data->act_date[2] = 0x31; /* 0x31 = '1', 1st */
	}

	return 0;
}

static int get_activation_date(struct health_data *health_data, struct rtc_time *tm)
{
	struct bhi_data *bhi_data = &health_data->bhi_data;
	struct bm_date date;

	/* read activation date when data is not successfully read in probe */
	if (bhi_data->act_date[0] == 0) {
		int ret;

		ret = batt_get_activation_date(bhi_data);
		if (ret < 0)
			return ret;
	}

	/*
	 * convert date to epoch
	 * for example:
	 * act_date: ASCII 2/2/3 -> bm_y/bm_m/bm_d: 22/02/03
	 *                       -> tm_year/tm_mon/tm_mday: 122/01/03
	 *                       -> epoch: 164384640
	 */
	date.bm_y = xymd_to_date(bhi_data->act_date[0]) + 20;
	date.bm_m = xymd_to_date(bhi_data->act_date[1]);
	date.bm_d = xymd_to_date(bhi_data->act_date[2]);

	tm->tm_year = date.bm_y + 100; /* base is 1900 */
	tm->tm_mon = date.bm_m - 1;    /* 0 is Jan ... 11 is Dec */
	tm->tm_mday = date.bm_d;       /* 1st ... 31th */

	return 0;
}

#define ONE_HR_SEC		3600
#define ONE_YEAR_HRS		(24 * 365)
#define BHI_INDI_CAP_DEFAULT	85
static int bhi_individual_conditions_index(struct health_data *health_data)
{
	const struct bhi_data *bhi_data = &health_data->bhi_data;
	const int cur_capacity_pct = 100 - bhi_data->capacity_fade;
	const int bhi_indi_cap = health_data->bhi_indi_cap;
	struct rtc_time tm;
	int battery_age, ret;

	ret = get_activation_date(health_data, &tm);
	if (ret == 0) {
		struct timespec64 ts;
		unsigned long long date_in_epoch;

		/* get by local time */
		ktime_get_real_ts64(&ts);
		date_in_epoch = ts.tv_sec - (sys_tz.tz_minuteswest * 60);

		battery_age = (date_in_epoch - rtc_tm_to_time64(&tm)) / ONE_HR_SEC;
		pr_debug("%s: age: act_date:%d timerh:%d\n", __func__, battery_age,
			 health_data->bhi_data.battery_age);
	} else {
		battery_age = health_data->bhi_data.battery_age;
	}

	/* Removed impedance condition due to validation required */
	if (battery_age >= ONE_YEAR_HRS || cur_capacity_pct <= bhi_indi_cap)
		return health_data->need_rep_threshold * 100;

	return BHI_ALGO_FULL_HEALTH;
}

/* GBMS_PROP_CAPACITY_FADE_RATE access this via GBMS_TAG_HCNT */
static int hist_get_index(int cycle_count, const struct batt_drv *batt_drv)
{
	/* wait for history to be initialized */
	if (batt_drv->hist_data_max_cnt <= 0 || cycle_count < 0)
		return -ENODATA;

	return cycle_count / batt_drv->hist_delta_cycle_cnt;
}

static int bhi_cap_data_update(struct bhi_data *bhi_data, struct batt_drv *batt_drv)
{
	struct power_supply *fg_psy = batt_drv->fg_psy;
	int rc, rc_fcr, tmp_cap_uah;
	const int fade_rate = GPSY_GET_INT_PROP(fg_psy, GBMS_PROP_CAPACITY_FADE_RATE, &rc);
	const int fade_rate_fcr =
			GPSY_GET_INT_PROP(fg_psy, GBMS_PROP_CAPACITY_FADE_RATE_FCR, &rc_fcr);
	const int designcap_uah = GPSY_GET_PROP(fg_psy, POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN);
	const int full_capacity = 100;
	const int pack_capacity_uah = bhi_data->pack_capacity * 1000;

	/* GBMS_PROP_CAPACITY_FADE_RATE is in percent */
	if (designcap_uah < 0)
		return -ENODATA;
	if (bhi_data->pack_capacity <= 0)
		return -EINVAL;
	if (rc || rc_fcr)
		return -EINVAL;

	tmp_cap_uah = (full_capacity - fade_rate) * designcap_uah;
	bhi_data->capacity_fade = full_capacity - (tmp_cap_uah / pack_capacity_uah);

	tmp_cap_uah = (full_capacity - fade_rate_fcr) * designcap_uah;
	bhi_data->capacity_fade_fcr = full_capacity - (tmp_cap_uah / pack_capacity_uah);

	pr_debug("%s: cap_fade=%d, cap_fade_fcr=%d, cycle_count=%d\n", __func__,
		 bhi_data->capacity_fade, bhi_data->capacity_fade_fcr, bhi_data->cycle_count);

	return 0;
}


/*
 * NOTE: make sure that the FG and this code use the same reference value for
 * capacity. Also GBMS_PROP_CAPACITY_FADE_RATE is in percent.
 */
static int bhi_health_get_capacity(int algo, const struct bhi_data *bhi_data)
{
	const int fade_rate = (algo == BHI_ALGO_ACHI_FCR) ? bhi_data->capacity_fade_fcr
							  : bhi_data->capacity_fade;

	return bhi_data->pack_capacity * (100 - fade_rate) / 100;
}

static bool bhi_bound_validity_check(const u16 *capacity, int lower, int upper)
{
	int cycles = 0;

	/* data validity */
	for (cycles = 0; cycles < BHI_TREND_POINTS_SIZE; cycles++)
		if (capacity[cycles] < lower || capacity[cycles] > upper)
			return false;

	return true;
}

static int bhi_get_capacity_bound(int cycle_count, const u16 *cap_bound)
{
	int index, ca_upper, ca_under;

	if (cycle_count <= 100)
		return cap_bound[0];

	index = cycle_count / 100;

	if (index >= BHI_TREND_POINTS_SIZE)
		index = BHI_TREND_POINTS_SIZE - 1;

	ca_upper = cap_bound[index];
	ca_under = cap_bound[index - 1];

	return ca_under + (cycle_count - (index * 100)) * (ca_upper - ca_under) / 100;
}

static bool bhi_algo_has_bounds(int algo)
{
	return algo == BHI_ALGO_ACHI_B || algo == BHI_ALGO_ACHI_RECAL ||
	       algo == BHI_ALGO_ACHI_RAVG_B || algo == BHI_ALGO_ACHI_CARETAKER;
}

static int bhi_algo_apply_bounds(int algo, int capacity_health, int cycle_count,
				 const struct bhi_data *bhi_data)
{
	int cap, l_bound, u_bound;

	l_bound = bhi_get_capacity_bound(cycle_count, &bhi_data->lower_bound.limit[0]);
	u_bound = bhi_get_capacity_bound(cycle_count, &bhi_data->upper_bound.limit[0]);

	cap = max(capacity_health, l_bound);
	cap = min(capacity_health, u_bound);

	pr_debug("%s: algo=%d l_bound=%d u_bound=%d\n", __func__, algo, l_bound, u_bound);

	return cap;
}

/* The limit for capacity is 80% of design */
static int bhi_calc_cap_index(int algo, struct batt_drv *batt_drv)
{
	const struct health_data *health_data = &batt_drv->health_data;
	const struct bhi_data *bhi_data = &health_data->bhi_data;
	int capacity_health, index;

	if (algo == BHI_ALGO_DISABLED)
		return BHI_ALGO_FULL_HEALTH;

	if (health_data->bhi_debug_cap_index)
		return health_data->bhi_debug_cap_index;

	if (!bhi_data->pack_capacity)
		return -ENODATA;

	capacity_health = bhi_health_get_capacity(algo, bhi_data);

	if (bhi_algo_has_bounds(algo)) {
		const int cycle_count = aacp_get_cc(batt_drv);

		capacity_health = bhi_algo_apply_bounds(algo, capacity_health, cycle_count,
							bhi_data);
	}

	/*
	 * TODO: compare to google_capacity?
	 * ret = gbms_storage_read(GBMS_TAG_GCFE, &gcap sizeof(gcap));
	 */

	index = (capacity_health * BHI_ALGO_FULL_HEALTH) / bhi_data->pack_capacity;

	pr_debug("%s: algo=%d index=%d ch=%d, pc=%d, fr=%d, fr_fcr=%d\n", __func__, algo, index,
		 capacity_health, bhi_data->pack_capacity, bhi_data->capacity_fade,
		 bhi_data->capacity_fade_fcr);

	return index;
}

static int bhi_cap_index_bound(int bhi_algo, int index)
{
	if (index < 0)
		return BHI_ALGO_FULL_HEALTH;

	if (index > BHI_ALGO_FULL_HEALTH && bhi_algo > BHI_ALGO_ACHI)
		return BHI_ALGO_FULL_HEALTH;

	return index;
}

/* read and qualify the battery initial impedance */
static int bhi_imp_read_ai(struct bhi_data *bhi_data, struct power_supply *fg_psy)
{
	int act_impedance;


	/* use ravg if the filter is full? */
	act_impedance = batt_ravg_value(&bhi_data->res_state);

	/* TODO: qualify with filter length */

	return act_impedance;
}

/* hold mutex_unlock(&batt_drv->chg_lock); */
static int bhi_imp_data_update(struct bhi_data *bhi_data, struct power_supply *fg_psy)
{
	const int use_ravg = true;
	int act_impedance = bhi_data->act_impedance;
	int cur_impedance;

	if (!act_impedance) {
		act_impedance = GPSY_GET_PROP(fg_psy, GBMS_PROP_HEALTH_ACT_IMPEDANCE);
		if (act_impedance == -EINVAL) {
			int ret;

			act_impedance = use_ravg ? bhi_imp_read_ai(bhi_data, fg_psy) :
					GPSY_GET_PROP(fg_psy, GBMS_PROP_HEALTH_IMPEDANCE);
			if (act_impedance <= 0)
				goto exit_done;

			ret = GPSY_SET_PROP(fg_psy, GBMS_PROP_HEALTH_ACT_IMPEDANCE,
					    act_impedance);
			if (ret < 0)
				goto exit_done;
		}

		if (act_impedance < 0)
			goto exit_done;

		/* primed, saved */
		bhi_data->act_impedance = act_impedance;
		return 0;
	}


	cur_impedance = batt_ravg_value(&bhi_data->res_state);

	/*
	 *  Can delegate to the FG with:
	 * 	cur_impedance = GPSY_GET_PROP(fg_psy, GBMS_PROP_HEALTH_IMPEDANCE);
	 * if (cur_impedance < 0)
	 * 	goto exit_done;
	 */

	 /* max in this session. Use average maybe? */
	 if (cur_impedance > bhi_data->cur_impedance)
		bhi_data->cur_impedance = cur_impedance;

exit_done:
	pr_debug("%s: cur_impedance=%d, act_impedance=%d\n", __func__,
		 cur_impedance, act_impedance);
	return 0;
}
/* pick the impedance from the algo */
static int bhi_health_get_impedance(int algo, const struct bhi_data *bhi_data)
{
	u32 cur_impedance;

	switch (algo) {
	case BHI_ALGO_DISABLED:
	case BHI_ALGO_CYCLE_COUNT:
	case BHI_ALGO_ACHI_RECAL:
	case BHI_ALGO_ACHI_RAVG_B:
	case BHI_ALGO_MIX_N_MATCH:
		cur_impedance = batt_ravg_value(&bhi_data->res_state);
		break;
	default:
		return 0;
	}

	if (cur_impedance <= 0)
		cur_impedance = bhi_data->act_impedance;

	return cur_impedance;
}

static int bhi_calc_imp_index(int algo, const struct health_data *health_data)
{
	const struct bhi_data *bhi_data = &health_data->bhi_data;
	u32 cur_impedance;
	int imp_index;

	if (!bhi_data->act_impedance)
		return BHI_ALGO_FULL_HEALTH;

	if (health_data->bhi_debug_imp_index)
		return health_data->bhi_debug_imp_index;

	cur_impedance = bhi_health_get_impedance(algo, bhi_data);
	if (cur_impedance == 0)
		return BHI_ALGO_FULL_HEALTH;

	/*
	 * TODO: on algo==*_B bounds check cur_impedance against res10
	 * before calculating the impedance index.
	 */

	/* The limit is 2x of activation. */
	imp_index = (2 * bhi_data->act_impedance - cur_impedance) * BHI_ALGO_FULL_HEALTH /
		    bhi_data->act_impedance;

	pr_debug("%s: algo=%d index=%d current=%d, activation=%d\n", __func__,
		 algo, imp_index, cur_impedance, bhi_data->act_impedance);

	return imp_index;
}

static int bhi_calc_sd_total(const struct swelling_data *sd)
{
	int i, swell_total = 0;
	ktime_t time_at;

	for (i = 0; i < BATT_TEMP_RECORD_THR ; i++) {
		time_at = sd->chg[i] / 3600;
		time_at += sd->dischg[i] / 3600;
		// TODO: use weights for temperature and soc
		// eg. sd->temp_thr[i]/10, sd->soc_thr[i],
		swell_total += time_at;
	}

	return swell_total;
}

static int bhi_calc_sd_index(int algo, const struct health_data *health_data)
{
	const struct bhi_data *bhi_data = &health_data->bhi_data;

	if (health_data->bhi_debug_sd_index)
		return health_data->bhi_debug_sd_index;

	pr_debug("%s: algo=%d index=%d\n", __func__, algo, bhi_data->ccbin_index);
	return bhi_data->ccbin_index;
}

static int bhi_cycle_count_index(const struct health_data *health_data)
{
	const int cc = health_data->bhi_data.cycle_count;
	int cc_mt = health_data->cycle_count_marginal_threshold;
	const int cc_nrt = health_data->cycle_count_need_rep_threshold;
	int h_mt = health_data->marginal_threshold;
	const int h_nrt = health_data->need_rep_threshold;
	int cc_index;

	/* threshold should be reasonable */
	if (h_mt < h_nrt || cc_nrt <= cc_mt)
		return BHI_ALGO_FULL_HEALTH;

	/* remove marginal_threshold */
	if (h_mt == h_nrt) {
		h_mt = 100;
		cc_mt = 0;
	}

	/* use interpolation to get index via cycle count/health threshold */
	cc_index = (h_mt - h_nrt) * (cc - cc_nrt) / (cc_mt - cc_nrt) + h_nrt;
	cc_index = cc_index * 100; /* for BHI_ROUND_INDEX*/

	if (cc_index > BHI_ALGO_FULL_HEALTH)
		cc_index = BHI_ALGO_FULL_HEALTH;

	if (cc_index < 0)
		cc_index = 0;

	return cc_index;
}

static int bhi_calc_health_index(int algo, struct health_data *health_data,
				 int cap_index, int imp_index, int sd_index)
{
	int ratio, index;
	int w_ci = 0;
	int w_ii = 0;
	int w_sd = 0;

	if (health_data->bhi_debug_health_index)
		return health_data->bhi_debug_health_index;

	switch (algo) {
	case BHI_ALGO_DISABLED:
		return BHI_ALGO_FULL_HEALTH;
	case BHI_ALGO_CYCLE_COUNT:
		return bhi_cycle_count_index(health_data);
	case BHI_ALGO_ACHI:
	case BHI_ALGO_ACHI_B:
	case BHI_ALGO_ACHI_RECAL:
	case BHI_ALGO_ACHI_RAVG_B:
	case BHI_ALGO_MIX_N_MATCH:
	case BHI_ALGO_ACHI_FCR:
	case BHI_ALGO_ACHI_CARETAKER:
		w_ci = bhi_w[algo].w_ci;
		w_ii = bhi_w[algo].w_ii;
		w_sd = bhi_w[algo].w_sd;
		break;
	case BHI_ALGO_DEBUG:
		w_ci = health_data->bhi_w_ci;
		w_ii = health_data->bhi_w_pi;
		w_sd = health_data->bhi_w_sd;
		break;
	case BHI_ALGO_INDI:
		return bhi_individual_conditions_index(health_data);
	default:
		return -EINVAL;
	}

	if (cap_index < 0)
		w_ci = 0;
	if (imp_index < 0)
		w_ii = 0;
	if (sd_index < 0)
		w_sd = 0;

	/* TODO: check single cell disconnect */

	ratio = w_ci + w_ii + w_sd;
	if (ratio > 100)
		return -ERANGE;
	if (!ratio)
		return 100;

	index = (cap_index * w_ci + imp_index * w_ii + sd_index * w_sd) / ratio;
	pr_debug("%s: algo=%d index=%d cap_index=%d/%d  imp_index=%d/%d sd_index=%d/%d\n",
		 __func__, algo, index, cap_index, w_ci, imp_index, w_ii, sd_index, w_sd);

	return index;
}

static bool bhi_algo_has_grace(int algo)
{
	return algo == BHI_ALGO_ACHI_B || algo == BHI_ALGO_ACHI_RAVG_B ||
	       algo == BHI_ALGO_ACHI_CARETAKER;
}

static enum bhi_status bhi_calc_health_status(int algo, int health_index,
					      const struct health_data *data)
{
	enum bhi_status health_status;

	if (data->bhi_debug_health_status)
		return data->bhi_debug_health_status;

	if (algo == BHI_ALGO_DISABLED)
		return BH_UNKNOWN;

	if (bhi_algo_has_grace(algo)) {
		const int cycle_count = data->bhi_data.cycle_count;
		const int l_bound = bhi_get_capacity_bound(cycle_count,
							   &data->bhi_data.lower_bound.limit[0]);

		if (l_bound == 0 && data->bhi_cycle_grace && cycle_count < data->bhi_cycle_grace)
			return BH_NOT_AVAILABLE;
	}

	if (data->cal_state == REC_STATE_SCHEDULED)
		return BH_INCONSISTENT;

	if (health_index < 0)
		health_status = BH_UNKNOWN;
	else if (health_index <= data->need_rep_threshold)
		health_status = BH_NEEDS_REPLACEMENT;
	else if (health_index <= data->marginal_threshold)
		health_status = BH_MARGINAL;
	else
		health_status = BH_NOMINAL;

	/* take the worse status between health_status and caretaker_status */
	if (algo == BHI_ALGO_ACHI_CARETAKER)
		health_status = max(health_status, data->caretaker_status);

	return health_status;
}

static enum bhi_status bhi_get_caretaker_status(struct batt_drv *batt_drv)
{
	const int cycle_count = aacp_get_cc(batt_drv);
	const int last_aafv_entry = gbms_aafv_get_last_entry(&batt_drv->chg_profile);
	enum bhi_status health_status = BH_NOMINAL;

	if (last_aafv_entry && cycle_count >= last_aafv_entry)
		health_status = batt_drv->aacp_health_last_entry;

	if (batt_drv->aacp_opt_out_cut_off_cycles != AACP_OPT_OUT_CUT_OFF_DISABLED
		&& cycle_count >= batt_drv->aacp_opt_out_cut_off_cycles)
		health_status = max(health_status, batt_drv->aacp_health_over_cutoff);

	if (batt_drv->aafv_cliff_cycle >= 0 && cycle_count >= batt_drv->aafv_cliff_cycle)
		health_status = max(health_status, batt_drv->aacp_health_over_cliff);

	return health_status;
}

static int batt_bhi_data_save(struct batt_drv *batt_drv)
{
	/* TODO: load save health status, index, cap index, imp index */

	/* TODO: save current impedance if not using RAVG */

	return 0;
}

static int batt_bhi_data_load(struct batt_drv *batt_drv)
{
	/* TODO: load last health status, index, cap index, imp index */
	/* TODO: prime current impedance if not using RAVG */

	return 0;
}

static int batt_bhi_map_algo(int algo, const struct health_data *health_data)
{
	if (algo != BHI_ALGO_DTOOL)
		return algo;

	/* diagnostic tool use BHI_ALGO_ACHI_B as default when bhi_algo is disabled */
	return health_data->bhi_algo != BHI_ALGO_DISABLED ?
	       health_data->bhi_algo : BHI_ALGO_ACHI_B;
}

/* call holding mutex_lock(&batt_drv->chg_lock)  */
static int batt_bhi_stats_update(struct batt_drv *batt_drv)
{
	struct health_data *health_data = &batt_drv->health_data;
	struct power_supply *fg_psy = batt_drv->fg_psy;
	const int bhi_algo = health_data->bhi_algo;
	enum bhi_status status;
	bool changed = false;
	int age, index;

	/* age (and cycle count* might be used in the calc */
	age = GPSY_GET_PROP(fg_psy, GBMS_PROP_BATTERY_AGE);
	if (age < 0)
		return -EIO;
	health_data->bhi_data.battery_age = age;

	/* cycle count is cached */
	if (health_data->bhi_debug_cycle_count != 0)
		health_data->bhi_data.cycle_count = health_data->bhi_debug_cycle_count;
	else
		health_data->bhi_data.cycle_count = batt_drv->cycle_count;

	index = bhi_calc_cap_index(bhi_algo, batt_drv);
	index = bhi_cap_index_bound(bhi_algo, index);
	changed |= health_data->bhi_cap_index != index;
	health_data->bhi_cap_index = index;

	index = bhi_calc_imp_index(bhi_algo, health_data);
	if (index < 0)
		index = 0;
	changed |= health_data->bhi_imp_index != index;
	health_data->bhi_imp_index = index;

	index = bhi_calc_sd_index(bhi_algo, health_data);
	if (index < 0)
		index = BHI_ALGO_FULL_HEALTH;
	changed |= health_data->bhi_sd_index != index;
	health_data->bhi_sd_index = index;

	index = bhi_calc_health_index(bhi_algo, health_data,
				      health_data->bhi_cap_index,
				      health_data->bhi_imp_index,
				      health_data->bhi_sd_index);
	if (index < 0)
		index = BHI_ALGO_FULL_HEALTH;

	changed |= health_data->bhi_index != index;
	health_data->bhi_index = index;

	health_data->caretaker_status = bhi_get_caretaker_status(batt_drv);
	status = bhi_calc_health_status(bhi_algo, BHI_ROUND_INDEX(index), health_data);
	changed |= health_data->bhi_status != status;
	health_data->bhi_status = status;

	pr_debug("%s: algo=%d status=%d bhi=%d cap_index=%d, imp_index=%d sd_index=%d (%d)\n", __func__,
		 bhi_algo, health_data->bhi_status, health_data->bhi_index,
		 health_data->bhi_cap_index, health_data->bhi_imp_index,
		 health_data->bhi_sd_index,  changed);


	if (changed) {
		int ret;

		/* TODO: send a power supply event? */

		ret = batt_bhi_data_save(batt_drv);
		if (ret < 0)
			pr_err("BHI: cannot save data (%d)\n", ret);
	}

	return changed;
}

/*
 * calculate the ratio of the time spent at under the soc_limit vs the time
 * spent over the soc_limit in percent.
 * call holding mutex_lock(&batt_drv->chg_lock);
 */
static int bhi_cycle_count_residency(struct gbatt_ccbin_data *ccd , int soc_limit)
{
	int i, under = 0, over = 0;

	for (i = 0; i < GBMS_CCBIN_BUCKET_COUNT; i++) {
		if (ccd->count[i] == 0xFFFF)
			continue;
		if ((i * 10) < soc_limit)
			under += ccd->count[i];
		else
			over += ccd->count[i];
	}

	pr_debug("%s: under=%d, over=%d limit=%d\n", __func__, under, over, soc_limit);
	return (under * BHI_ALGO_FULL_HEALTH) / (under + over);
}

static bool batt_bhi_need_recalibration(struct batt_drv *batt_drv)
{
	int ret, full_cap_nom, cycle_count, l_trigger, u_trigger;

	/* already on-going */
	if (batt_drv->health_data.cal_state != REC_STATE_OK ||
	    batt_drv->health_data.cal_mode != REC_MODE_RESET)
		return false;

	/* recalibration enabled in algorithm 4 */
	if (batt_drv->health_data.bhi_algo != BHI_ALGO_ACHI_RECAL)
		return false;

	/* read full capacity value */
	ret = GPSY_GET_PROP(batt_drv->fg_psy, POWER_SUPPLY_PROP_HEALTH);
	if (ret == POWER_SUPPLY_HEALTH_CALIBRATION_REQUIRED)
		return true;

	/* read full capacity value */
	ret = GPSY_GET_PROP(batt_drv->fg_psy, POWER_SUPPLY_PROP_CHARGE_FULL);
	if (ret < 0)
		return false;

	full_cap_nom = ret / 1000;

	/*
	 * compare with design value, allow to reset FG if conditions match
	 * and wait for appropriate time to execute
	 */
	cycle_count = batt_drv->cycle_count;
	l_trigger = bhi_get_capacity_bound(cycle_count,
					   &batt_drv->health_data.bhi_data.lower_bound.trigger[0]);
	u_trigger = bhi_get_capacity_bound(cycle_count,
					   &batt_drv->health_data.bhi_data.upper_bound.trigger[0]);

	return (full_cap_nom < l_trigger ||  full_cap_nom > u_trigger);
}

/*
 * initial or done   detect abnormal      FG reset/learning    FG learning done
 * STATE_OK    --->  STATE_SCHEDULED ---> STATE_SCHEDULED ---> STATE_OK
 * MODE_RESET  --->  MODE_BEST_TIME  ---> MODE_RESTART    ---> MODE_RESET
 *                  (MODE_IMMEDIATE)
 */
static int batt_bhi_update_recalibration_status(struct batt_drv *batt_drv)
{
	u8 cal_state = batt_drv->health_data.cal_state;
	u8 cal_mode = batt_drv->health_data.cal_mode;
	int recal_state, ret = 0;
	bool is_best_time = false;

	recal_state = GPSY_GET_PROP(batt_drv->fg_psy, GBMS_PROP_RECAL_FG);
	if (recal_state < 0)
		return recal_state;
	if (recal_state != 0) {
		cal_state = REC_STATE_SCHEDULED;
		cal_mode = REC_MODE_RESTART;
		goto done;
	}

	if (cal_mode == REC_MODE_BEST_TIME) {
		if (batt_drv->msc_state == MSC_HEALTH_PAUSE)
			is_best_time = true;
		if (batt_drv->chg_done == true)
			is_best_time = true;
	}

	if (cal_mode == REC_MODE_IMMEDIATE)
		is_best_time = true;

	if (is_best_time) {
		ret = GPSY_SET_PROP(batt_drv->fg_psy, GBMS_PROP_RECAL_FG,
				    batt_drv->health_data.cal_target);
		if (ret == 0) {
			cal_state = REC_STATE_SCHEDULED;
			cal_mode = REC_MODE_RESTART;
		}
		goto done;
	}

	if (cal_mode == REC_MODE_RESTART && recal_state == 0) {
		cal_state = REC_STATE_OK;
		cal_mode = REC_MODE_RESET;
		goto done;
	}

	if (batt_bhi_need_recalibration(batt_drv)) {
		cal_state = REC_STATE_SCHEDULED;
		cal_mode = REC_MODE_BEST_TIME;
	}

done:
	if (batt_drv->health_data.cal_state != cal_state ||
	    batt_drv->health_data.cal_mode != cal_mode) {
		gbms_logbuffer_prlog(batt_drv->ttf_stats.ttf_log, LOGLEVEL_INFO, 0, LOGLEVEL_DEBUG,
				     "RE_CAL: cal_state: %d -> %d, cal_mode:%d -> %d\n",
				     batt_drv->health_data.cal_state, cal_state,
				     batt_drv->health_data.cal_mode, cal_mode);
		batt_drv->health_data.cal_state = cal_state;
		batt_drv->health_data.cal_mode = cal_mode;
	}

	return ret;
}

/* call holding mutex_lock(&batt_drv->chg_lock)  */
static int batt_bhi_stats_update_all(struct batt_drv *batt_drv)
{
	struct health_data *health_data = &batt_drv->health_data;
	int ret;

	/* swell probability: cc residecy needs ccd->lock */
	batt_drv->health_data.bhi_data.ccbin_index =
		bhi_cycle_count_residency(&batt_drv->cc_data, BHI_CCBIN_INDEX_LIMIT);
	/* swell cumulative needs a new lock */
	batt_drv->health_data.bhi_data.swell_cumulative =
		bhi_calc_sd_total(&batt_drv->sd);

	pr_debug("BHI: limit=%d%% ccbin_index=%d swell_total=%d\n",
			BHI_CCBIN_INDEX_LIMIT,
			batt_drv->health_data.bhi_data.ccbin_index,
			batt_drv->health_data.bhi_data.swell_cumulative);

	/* impedance should be pretty recent */
	ret = bhi_imp_data_update(&health_data->bhi_data, batt_drv->fg_psy);
	if (ret < 0)
		pr_err("bhi imp data not available (%d)\n", ret);

	/* bhi_capacity_index on disconnect */
	ret = bhi_cap_data_update(&batt_drv->health_data.bhi_data, batt_drv);
	if (ret < 0)
		pr_err("bhi cap data not available (%d)\n", ret);

	batt_bhi_stats_update(batt_drv);

	return 0;
}

/* AAFV ------------------------------------------------------------------- */

static int batt_init_aafv_profile(struct batt_drv *batt_drv)
{
	struct gbms_chg_profile *profile = &batt_drv->chg_profile;
	struct device_node *node = batt_drv->device->of_node;
	int ret;

	ret = gbms_read_aafv_limits(profile, gbms_batt_id_node(node));
	dev_info(batt_drv->device, "AAFV: schedule supported: %s",
		ret ? "not detected" : "detected");

	/* NOTE: might need to be BRID specific */
	ret = of_property_read_u32(node, "google,aafv-config",
				   &batt_drv->aafv_state);
	if (ret < 0)
		batt_drv->aafv_state = profile->aafv_nb_limits ?
			BATT_AAFV_DISABLED : BATT_AAFV_UNKNOWN;

	ret = of_property_read_u32(node, "google,aafv-max-offset", &batt_drv->aafv_max_offset);
	if (ret < 0)
		batt_drv->aafv_max_offset = AAFV_MAX_OFFSET_DEFAULT;

	ret = of_property_read_u32(node, "google,aafv-cliff-cycle", &batt_drv->aafv_cliff_cycle);
	if (ret < 0)
		batt_drv->aafv_cliff_cycle = AAFV_CLIFF_CYCLE_DEFAULT;

	ret = of_property_read_u32(node, "google,aafv-cliff-offset", &batt_drv->aafv_cliff_offset);
	if (ret < 0)
		batt_drv->aafv_cliff_offset = AAFV_CLIFF_OFFSET_DEFAULT;

	return 0;
}

/* requires mutex_lock(&batt_drv->aacp_state_lock); */
static u32 aafv_get_offset_by_cycles(const struct batt_drv *batt_drv, int cycle_count)
{
	int aafv_offset;

	if (aafv_enabled(batt_drv->aafv_state, batt_drv->aacp_opt_out) == false)
		return 0;

	if (batt_drv->aafv_apply_max)
		aafv_offset = batt_drv->aafv_max_offset;
	else if (batt_drv->aafv_cliff_cycle >= 0 && cycle_count >= batt_drv->aafv_cliff_cycle)
		aafv_offset = batt_drv->aafv_cliff_offset;
	else
		aafv_offset = gbms_aafv_get_offset(&batt_drv->chg_profile, cycle_count);

	return (u32)(aafv_offset * GBMS_AAFV_VOLTAGE_OFFSET_SCALE);
}

/*
 * executed at connect, when opt_out changes or when server side changes
 * the state.
 * requires mutex_lock(&batt_drv->aacp_state_lock);
 */
static void aafv_update_offset(struct batt_drv *batt_drv)
{
	const int cycle_count = aacp_get_cc(batt_drv);
	u32 aafv_offset;
	int fg_ret;

	/*  aafv_offset==0 when aafv is disabled */
	aafv_offset = aafv_get_offset_by_cycles(batt_drv, cycle_count);
	if (batt_drv->chg_profile.aafv_offset == aafv_offset)
		return;

	fg_ret = GPSY_SET_PROP(batt_drv->fg_psy, GBMS_PROP_AAFV, aafv_offset);

	gbms_logbuffer_prlog(batt_drv->bd_log, LOGLEVEL_INFO, 0, LOGLEVEL_INFO,
		"AAFV: cc:%d, st:%d, oo:%d, of:%d->%d (fg_ret=%d)",
		cycle_count, batt_drv->aafv_state, batt_drv->aacp_opt_out,
		batt_drv->chg_profile.aafv_offset, aafv_offset, fg_ret);

	/* will try again on next connect if setting the fg failed */
	if (fg_ret == 0)
		batt_drv->chg_profile.aafv_offset = aafv_offset;
}

/* requires mutex_lock(&batt_drv->aacp_state_lock); */
static bool aafv_update_float_voltage(struct batt_drv *batt_drv, int *fv_uv)
{
	struct gbms_chg_profile *profile = &batt_drv->chg_profile;
	const int last_vbatt_idx = profile->volt_nb_limits - 1;
	const u32 last_fv = profile->volt_limits[last_vbatt_idx];
	const u32 penultimate_fv = (last_vbatt_idx >= 1) ?
			profile->volt_limits[last_vbatt_idx - 1] : 0;
	bool is_last = *fv_uv == last_fv;
	u32 aafv_fv;

	/*
	 * only lower last charge tier if higher than the adjusted
	 * NOTE: assumes that ->chg_profile.aafv_offset is correct
	 * for the cycle count. aafv_offset is updated when the device
	 * start charging or when server side changes the configuration
	 */
	aafv_fv = last_fv - batt_drv->chg_profile.aafv_offset;
	if (aafv_fv > penultimate_fv && aafv_fv < last_fv)
		*fv_uv = (int)aafv_fv;

	return is_last;
}

/* AACC ------------------------------------------------------------------- */

static void aacc_update_cycle_count(struct batt_drv *batt_drv)
{
	int cycle_count = batt_drv->cycle_count;

	/* TODO: AACC is TBD */
	batt_drv->aacc = cycle_count;
}

/* ------------------------------------------------------------------------ */

/* TODO: factor msc_logic_irdop from the logic about tier switch */
static int msc_logic(struct batt_drv *batt_drv)
{
	bool sw_jeita;
	int msc_state = MSC_NONE;
	struct power_supply *fg_psy = batt_drv->fg_psy;
	struct gbms_chg_profile *profile = &batt_drv->chg_profile;
	int vbatt_idx = batt_drv->vbatt_idx, fv_uv = batt_drv->fv_uv, temp_idx;
	int temp, ibatt, vbatt, ioerr, profile_vbatt_idx;
	int update_interval = MSC_DEFAULT_UPDATE_INTERVAL;
	const ktime_t now = get_boot_sec();
	ktime_t elap = now - batt_drv->ce_data.last_update;
	bool changed;

	ioerr = gbatt_get_raw_temp(batt_drv, &temp);
	if (ioerr < 0)
		return -EIO;

	/*
	 * driver state is (was) reset when we hit the SW jeita limit.
	 * NOTE: resetting driver state will release the wake assertion
	 */
	sw_jeita = msc_logic_soft_jeita(batt_drv, temp);
	if (sw_jeita) {
		/* reset batt_drv->jeita_stop_charging to -1 */
		if (batt_drv->jeita_stop_charging == 0)
			batt_reset_chg_drv_state(batt_drv);

		return 0;
	} else if (batt_drv->jeita_stop_charging) {
		batt_prlog(BATT_PRLOG_ALWAYS,
			   "MSC_JEITA temp=%d ok, enabling charging\n",
			   temp);
		batt_drv->jeita_stop_charging = 0;
		batt_drv->ce_data.max_charge_voltage = 0;
	}

	ibatt = GPSY_GET_INT_PROP(fg_psy, POWER_SUPPLY_PROP_CURRENT_NOW,
					  &ioerr);
	if (ioerr < 0)
		return -EIO;

	vbatt = GPSY_GET_PROP(fg_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW);
	if (vbatt < 0)
		return -EIO;

	/* save the maximum battery voltage */
	batt_drv->ce_data.max_charge_voltage = max(batt_drv->ce_data.max_charge_voltage, vbatt);

	/*
	 * Multi Step Charging with IRDROP compensation when vchrg is != 0
	 * vbatt_idx = batt_drv->vbatt_idx, fv_uv = batt_drv->fv_uv
	 */
	temp_idx = gbms_msc_temp_idx(profile, temp);
	if (temp_idx != batt_drv->temp_idx || batt_drv->fv_uv == -1 ||
		batt_drv->vbatt_idx == -1) {

		msc_state = MSC_SEED;

		/* seed voltage and charging table on connect or temp_idx change, book 0 time */
		if (batt_drv->vbatt_idx == -1 || temp_idx != batt_drv->temp_idx)
			vbatt_idx = gbms_msc_voltage_idx_merge_tiers(profile, vbatt, temp_idx);

		batt_prlog(BATT_PRLOG_ALWAYS,
			   "MSC_SEED temp=%d vb=%d temp_idx:%d->%d, vbatt_idx:%d->%d\n",
			   temp, vbatt, batt_drv->temp_idx, temp_idx,
			   batt_drv->vbatt_idx, vbatt_idx);

		/* Debounce tier switch only when not already switching */
		if (batt_drv->checked_tier_switch_cnt == 0)
			batt_drv->checked_cv_cnt = profile->cv_debounce_cnt;
	} else if (batt_drv->msc_state == MSC_DONE) {
		const int vtier = profile->volt_limits[vbatt_idx];

		/* 4300 - 200 = 4100 */
		if (vbatt < (vtier - 200000)) {
			batt_prlog(BATT_PRLOG_ALWAYS, "MSC_DONE restart vbatt=%d margin=%d\n",
			           vbatt, 200000);
			msc_state = MSC_DSG;
		} else {
			msc_state = MSC_DONE;
			batt_prlog(BATT_PRLOG_ALWAYS, "MSC_DONE propagate vbatt=%d\n", vbatt);
		}
	} else if (ibatt > 0) {
		const int vtier = profile->volt_limits[vbatt_idx];
		const bool log_level = batt_drv->msc_state != MSC_DSG ||
				       batt_drv->cc_max != 0;

		/*
		 * Track battery voltage if discharging is due to system load,
		 * low ILIM or lack of headroom; stop charging work and reset
		 * batt_drv state() when discharging is due to disconnect.
		 * NOTE: POWER_SUPPLY_PROP_STATUS return *_DISCHARGING only on
		 * disconnect.
		 * NOTE: same vbat_idx will not change fv_uv
		 */
		msc_state = MSC_DSG;
		vbatt_idx = gbms_msc_voltage_idx_merge_tiers(profile, vbatt, temp_idx);

		batt_prlog(batt_prlog_level(log_level),
			   "MSC_DSG vbatt_idx:%d->%d vt=%d fv_uv=%d vb=%d ib=%d cv_cnt=%d ov_cnt=%d\n",
			   batt_drv->vbatt_idx, vbatt_idx, vtier, fv_uv, vbatt, ibatt,
			   batt_drv->checked_cv_cnt, batt_drv->checked_ov_cnt);

	} else if (batt_drv->vbatt_idx == gbms_msc_get_last_voltage_idx(profile, temp_idx)) {
		const int chg_type = batt_drv->chg_state.f.chg_type;
		const int vtier = profile->volt_limits[vbatt_idx];
		int log_level;

		/*
		 * will not adjust charger voltage only in the configured
		 * last tier.
		 * NOTE: might not be the "real" last tier since can I have
		 * tiers with max charge current == 0.
		 * NOTE: should I use a voltage limit instead?
		 */

		if (chg_type == POWER_SUPPLY_CHARGE_TYPE_FAST)
			msc_state = MSC_FAST;
		else if (chg_type != POWER_SUPPLY_CHARGE_TYPE_TAPER_EXT)
			msc_state = MSC_TYPE;
		else
			msc_state = MSC_LAST;

		log_level = batt_prlog_level(batt_drv->msc_state != msc_state);
		if (log_level != BATT_PRLOG_ALWAYS && msc_state == MSC_LAST) {

			if (batt_drv->last_log_cnt > 0)
				batt_drv->last_log_cnt--;
			if (batt_drv->last_log_cnt == 0) {
				batt_drv->last_log_cnt = BATT_PRLOG_LAST_LOG_COUNT;
				log_level = batt_prlog_level(true);
			}
		}

		batt_prlog(log_level, "MSC_LAST vt=%d fv_uv=%d vb=%d ib=%d\n",
			   vtier, fv_uv, vbatt, ibatt);

	} else {
		const int tier_idx = batt_chg_vbat2tier(batt_drv->vbatt_idx);
		const int vtier = profile->volt_limits[vbatt_idx];
		const int switch_cnt = profile->cv_tier_switch_cnt;
		const int cc_next_max = GBMS_CCCM_LIMITS(profile, temp_idx,
							vbatt_idx + 1);
		const int chg_type = batt_drv->chg_state.f.chg_type;

		/* book elapsed time to previous tier & msc_irdrop_state */
		msc_state = msc_logic_irdrop(batt_drv,
					     vbatt, ibatt, temp_idx,
					     &vbatt_idx, &fv_uv,
					     &update_interval,
					     &batt_drv->cc_max_pullback);

		if (msc_pm_hold(msc_state) == 1 && !batt_drv->hold_taper_ws) {
			__pm_stay_awake(batt_drv->taper_ws);
			batt_drv->hold_taper_ws = true;
		}

		mutex_lock(&batt_drv->stats_lock);
		gbms_chg_stats_tier(&batt_drv->ce_data.tier_stats[tier_idx],
				    batt_drv->msc_irdrop_state, elap);
		batt_drv->msc_irdrop_state = msc_state;
		mutex_unlock(&batt_drv->stats_lock);

		/*
		 * Basic multi step charging: switch to next tier when ibatt
		 * is under next tier cc_max.
		 */
		if (batt_drv->checked_cv_cnt > 0) {
			/* debounce period on tier switch */
			batt_drv->checked_cv_cnt -= 1;

			batt_prlog(batt_prlog_level(msc_state != MSC_FAST),
				   "MSC_WAIT s:%d->%d vt=%d fv_uv=%d vb=%d ib=%d cv_cnt=%d ov_cnt=%d t_cnt=%d\n",
				   msc_state, MSC_WAIT, vtier, fv_uv, vbatt, ibatt,
				   batt_drv->checked_cv_cnt, batt_drv->checked_ov_cnt,
				   batt_drv->checked_tier_switch_cnt);

			if (-ibatt > cc_next_max)
				batt_drv->checked_tier_switch_cnt = 0;

			msc_state = MSC_WAIT;
		} else if (cc_next_max && -ibatt > cc_next_max) {

			/* current over next tier, reset tier switch count */
			batt_prlog(BATT_PRLOG_ALWAYS,
				   "MSC_RSTC s:%d->%d vt=%d fv_uv=%d vb=%d ib=%d cc_next_max=%d t_cnt=%d->0\n",
				   msc_state, MSC_RSTC, vtier, fv_uv, vbatt, ibatt, cc_next_max,
				   batt_drv->checked_tier_switch_cnt);

			batt_drv->checked_tier_switch_cnt = 0;
			msc_state = MSC_RSTC;
		} else if ((cc_next_max == 0) &&
				   (chg_type == POWER_SUPPLY_CHARGE_TYPE_TAPER_EXT) &&
				   (temp >= batt_drv->cv_max_temp)) {

			vbatt_idx = batt_drv->vbatt_idx + 1;

			batt_prlog(BATT_PRLOG_ALWAYS,
				   "MSC_DONE s:%d->%d tier vb=%d ib=%d vbatt_idx=%d->%d\n",
				   msc_state, MSC_DONE, vbatt, ibatt,
				   batt_drv->vbatt_idx, vbatt_idx);
			msc_state = MSC_DONE;
		} else if (batt_drv->checked_tier_switch_cnt >= switch_cnt) {
			/* next tier, fv_uv detemined at MSC_SET */
			vbatt_idx = batt_drv->vbatt_idx + 1;

			batt_prlog(BATT_PRLOG_ALWAYS,
				   "MSC_NEXT s:%d->%d tier vb=%d ib=%d vbatt_idx=%d->%d\n",
				   msc_state, MSC_NEXT, vbatt, ibatt,
				   batt_drv->vbatt_idx, vbatt_idx);

			msc_state = MSC_NEXT;
		} else {
			/* current under next tier, +1 on tier switch count */
			batt_drv->checked_tier_switch_cnt++;

			batt_prlog(BATT_PRLOG_ALWAYS,
				   "MSC_NYET s:%d->%d vt=%d vb=%d ib=%d cc_next_max=%d t_cnt=%d\n",
				   msc_state, MSC_NYET, vtier, vbatt, ibatt, cc_next_max,
				   batt_drv->checked_tier_switch_cnt);

			msc_state = MSC_NYET;
		}

	}

	if (msc_pm_hold(msc_state) == 0 && batt_drv->hold_taper_ws) {
		batt_drv->hold_taper_ws = false;
		__pm_relax(batt_drv->taper_ws);
	}

	/* need a new fv_uv only on a new voltage tier.  */
	if (vbatt_idx != batt_drv->vbatt_idx || temp_idx != batt_drv->temp_idx) {
		vbatt_idx = gbms_msc_voltage_idx_merge_tiers(profile, vbatt, temp_idx);
		fv_uv = profile->volt_limits[vbatt_idx];
		batt_drv->checked_tier_switch_cnt = 0;
		batt_drv->checked_ov_cnt = 0;
	}

	if (!batt_drv->msc_last_votable)
		batt_drv->msc_last_votable = gvotable_election_get_handle(VOTABLE_MSC_LAST);

	if (batt_drv->msc_last_votable)
		gvotable_cast_int_vote(batt_drv->msc_last_votable, "BATT",
				       vbatt_idx == gbms_msc_get_last_voltage_idx(profile, temp_idx), 1);
	/*
	 * book elapsed time to previous tier & msc_state
	 * NOTE: temp_idx != -1 but batt_drv->msc_state could be -1
	 */
	mutex_lock(&batt_drv->stats_lock);
	profile_vbatt_idx = gbms_msc_voltage_idx(profile, vbatt);
	if (profile_vbatt_idx != -1 && profile_vbatt_idx < profile->volt_nb_limits) {
		int tier_idx = batt_chg_vbat2tier(profile_vbatt_idx);

		/* this is the seed after the connect */
		if (batt_drv->profile_vbatt_idx == -1)
			elap = 0;

		batt_chg_stats_update(batt_drv, temp_idx, tier_idx,
				      ibatt / 1000, temp,
				      elap);

	}

	batt_drv->msc_state = msc_state;
	batt_drv->ce_data.last_update = now;
	mutex_unlock(&batt_drv->stats_lock);

	batt_drv->need_mp = batt_needs_more_power(batt_drv, ibatt);

	changed = batt_drv->temp_idx != temp_idx ||
		  batt_drv->vbatt_idx != vbatt_idx ||
		  batt_drv->fv_uv != fv_uv ||
		  batt_drv->cc_max_pullback != batt_drv->cc_max;

	/* next update */
	batt_drv->msc_update_interval = update_interval;
	/* if tier change, reset cc_max from chg table, otherwise use pullback value */
	if (!batt_drv->pullback_current || vbatt_idx != batt_drv->vbatt_idx ||
	    temp_idx != batt_drv->temp_idx) {
		batt_drv->cc_max = GBMS_CCCM_LIMITS(profile, temp_idx, vbatt_idx);
		batt_drv->cc_max_pullback = 0;
	} else if (batt_drv->cc_max_pullback > 0) {
		batt_drv->cc_max = batt_drv->cc_max_pullback;
	} else {
		batt_drv->cc_max = GBMS_CCCM_LIMITS(profile, temp_idx, vbatt_idx);
		batt_drv->cc_max_pullback = 0;
	}

	/* adjust last fv_uv for AAFV */
	if (vbatt_idx != batt_drv->vbatt_idx || temp_idx != batt_drv->temp_idx) {
		if (vbatt_idx == profile->volt_nb_limits - 1 && batt_drv->cc_max > 0)
			aafv_update_float_voltage(batt_drv, &fv_uv);
	}

	batt_prlog(batt_prlog_level(changed),
		   "MSC_LOGIC temp_idx:%d->%d, vbatt_idx:%d->%d, fv=%d->%d, cc_max=%d, ui=%d cv_cnt=%d ov_cnt=%d\n",
		   batt_drv->temp_idx, temp_idx, batt_drv->vbatt_idx, vbatt_idx,
		   batt_drv->fv_uv, fv_uv, batt_drv->cc_max, update_interval,
		   batt_drv->checked_cv_cnt, batt_drv->checked_ov_cnt);

	batt_drv->vbatt_idx = vbatt_idx;
	batt_drv->profile_vbatt_idx = profile_vbatt_idx;
	batt_drv->temp_idx = temp_idx;
	batt_drv->topoff = profile->topoff_limits[temp_idx];
	batt_drv->fv_uv = fv_uv;

	return 0;
}

/* no ssoc_delta when in overheat */
static int ssoc_get_delta(struct batt_drv *batt_drv)
{
	const bool overheat = batt_drv->batt_health ==
			      POWER_SUPPLY_HEALTH_OVERHEAT;

	return overheat ? 0 : qnum_fromint(batt_drv->ssoc_state.ssoc_delta);
}

/* TODO: handle the whole state buck_enable state */
static void ssoc_change_state(struct batt_ssoc_state *ssoc_state, bool ben)
{
	const ktime_t now = get_boot_sec();

	if (!ben) {
		ssoc_state->disconnect_time = now;
	} else if (ssoc_state->disconnect_time) {
		const u32 trickle_reset = ssoc_state->bd_trickle_reset_sec;
		const long long elap = now - ssoc_state->disconnect_time;

		if (trickle_reset && elap > trickle_reset)
			ssoc_state->bd_trickle_cnt = 0;

		pr_debug("MSC_BD: bd_trickle_cnt=%d dsc_time=%lld elap=%lld\n",
			 ssoc_state->bd_trickle_cnt,
			 ssoc_state->disconnect_time,
			 elap);

		ssoc_state->disconnect_time = 0;
	}

	ssoc_state->buck_enabled = ben;
}

static void bd_trickle_reset(struct batt_ssoc_state *ssoc_state,
			     struct gbms_charging_event *ce_data)
{
	ssoc_state->bd_trickle_cnt = 0;
	ssoc_state->disconnect_time = 0;
	ssoc_state->bd_trickle_full = false;
	ssoc_state->bd_trickle_eoc = false;

	/* Set to false in cev_stats_init */
	ce_data->bd_clear_trickle = true;
}

static void batt_prlog_din(union gbms_charger_state *chg_state, int log_level)
{
	batt_prlog(log_level,
		   "MSC_DIN chg_state=%lx f=0x%x chg_s=%s chg_t=%s vchg=%d icl=%d\n",
		   (unsigned long)chg_state->v,
		   chg_state->f.flags,
		   gbms_chg_status_s(chg_state->f.chg_status),
		   gbms_chg_type_s(chg_state->f.chg_type),
		   chg_state->f.vchrg,
		   chg_state->f.icl);
}

static void google_battery_dump_profile(const struct gbms_chg_profile *profile)
{
	char *buff;

	buff = kzalloc(GBMS_CHG_ALG_BUF_SZ, GFP_KERNEL);
	if (buff) {
		gbms_dump_chg_profile(buff, GBMS_CHG_ALG_BUF_SZ, profile);
		pr_info("%s", buff);
		kfree(buff);
	}
}

/* requires mutex_lock(&batt_drv->aacp_state_lock); */
static void aacr_update_chg_table(struct batt_drv *batt_drv)
{
	u32 capacity;

	capacity = aacr_get_capacity_locked(batt_drv);
	if (capacity == batt_drv->chg_profile.capacity_ma)
		return;

	gbms_logbuffer_prlog(batt_drv->bd_log, LOGLEVEL_INFO, 0, LOGLEVEL_INFO,
		"AACR: cc:%d, st:%d, oo=%d, capacity:%d->%d, algo:%d, grace:%d, cmax:%d, min_cr:%d, cliff_cr:%d",
		aacp_get_cc(batt_drv), batt_drv->aacr_state, batt_drv->aacp_opt_out,
		batt_drv->chg_profile.capacity_ma, capacity,
		batt_drv->aacr_algo, batt_drv->aacr_cycle_grace,
		batt_drv->aacr_cycle_max, batt_drv->aacr_min_capacity_rate,
		batt_drv->aacr_cliff_capacity_rate);
	gbms_init_chg_table(&batt_drv->chg_profile, batt_drv->device->of_node, capacity);
	google_battery_dump_profile(&batt_drv->chg_profile);
}

/* cell fault: disconnect of one of the battery cells */
static bool batt_cell_fault_detect(struct batt_bpst *bpst_state)
{
	int bpst_sbd_status;

	/*
	 * fake bpst_sbd_status by "echo 1 > /d/bpst/bpst_sbd_status"
	 * TODO: will implement the code from the algo in b/203019566
	 */
	bpst_sbd_status = bpst_state->bpst_sbd_status;

	return !!bpst_sbd_status && !bpst_state->bpst_detect_disable;
}

static int batt_bpst_detect_begin(struct batt_bpst *bpst_state)
{
	const int bpst_count_threshold = bpst_state->bpst_count_threshold;
	u8 data;
	int ret;

	if (bpst_state->bpst_detect_disable)
		return 0;

	ret = gbms_storage_read(GBMS_TAG_BPST, &data, sizeof(data));
	if (ret < 0)
		return -EINVAL;

	if (data == 0xff) {
		data = 0;
		ret = gbms_storage_write(GBMS_TAG_BPST, &data, sizeof(data));
		if (ret < 0)
			return -EINVAL;
	}
	bpst_state->bpst_count = data;
	if (bpst_count_threshold > 0) {
		bpst_state->bpst_cell_fault = (data >= (u8)bpst_count_threshold);
		pr_debug("%s: MSC_BPST: single battery disconnect %d\n",
			 __func__, bpst_state->bpst_cell_fault);
	}

	/* reset detection status */
	bpst_state->bpst_sbd_status = 0;

	pr_debug("%s: MSC_BPST: %d in connected\n", __func__, data);
	return 0;
}

static int batt_bpst_detect_update(struct batt_drv *batt_drv)
{
	struct batt_bpst *bpst_state = &batt_drv->bpst_state;
	const u8 data = bpst_state->bpst_count + 1;
	int ret;

	if (data < 0xff) {
		ret = gbms_storage_write(GBMS_TAG_BPST, &data, sizeof(data));
		if (ret < 0)
			return -EINVAL;
	}

	pr_debug("%s: MSC_BPST: %d in disconnected\n", __func__, data);
	return 0;
}

static int batt_bpst_reset(struct batt_bpst *bpst_state)
{
	if (bpst_state->bpst_enable) {
		u8 data = 0;

		return gbms_storage_write(GBMS_TAG_BPST, &data, sizeof(data));
	}

	return 0;
}

#define BATT_BPST_DEFAULT_CHG_RATE 100
static int batt_init_bpst_profile(struct batt_drv *batt_drv)
{
	struct batt_bpst *bpst_state = &batt_drv->bpst_state;
	struct device_node *node = batt_drv->device->of_node;
	int ret;

	/* set cell_fault initial status */
	bpst_state->bpst_cell_fault = false;

	bpst_state->bpst_enable = of_property_read_bool(node, "google,bpst-enable");
	if (!bpst_state->bpst_enable)
		return 0;

	ret = of_property_read_u32(node, "google,bpst-chg-rate", &bpst_state->bpst_chg_rate);
	if (ret < 0)
		bpst_state->bpst_chg_rate = BATT_BPST_DEFAULT_CHG_RATE;

	dev_info(batt_drv->device, "bpst profile enabled, rate=%d, ret=%d\n",
		 bpst_state->bpst_chg_rate, ret);

	return 0;
}

/* AACT ------------------------------------------------------------------- */

/* call holding mutex_lock(&batt_drv->aacp_state_lock); */
static void aact_reset(struct gbms_chg_profile *profile)
{
	profile->aact_nb_limits = 0;
	profile->aact_idx = 0;
	profile->aact_cccm_limits = 0;
}

/* call holding mutex_lock(&batt_drv->aacp_state_lock); */
static int aact_get_index(const struct batt_drv *batt_drv)
{
	const int cycle_count = aacp_get_cc(batt_drv);

	return gbms_aact_get_index(&batt_drv->chg_profile, cycle_count);
}

/* call holding mutex_lock(&batt_drv->aacp_state_lock); */
static int aact_update_chg_table(struct batt_drv *batt_drv)
{
	struct gbms_chg_profile *profile = &batt_drv->chg_profile;
	struct device_node *node = batt_drv->device->of_node;
	const bool enabled = aact_enabled(batt_drv->aact_state, batt_drv->aacp_opt_out);
	int ret;

	if (!profile->aact_init_profile && enabled) {
		/* init AACT charge table */
		ret = gbms_init_aact_profile(profile, node);
		if (ret < 0)
			return ret;
		ret = gbms_update_chg_profile_from_aact(profile);
		if (ret < 0)
			return ret;
		gbms_init_chg_table(profile, node, batt_drv->battery_capacity);
	} else if (profile->aact_init_profile && !enabled) {
		/* reset AACT */
		aact_reset(profile);

		/* init default charge table */
		ret = gbms_init_chg_profile(profile, node);
		if (ret < 0)
			return ret;

		gbms_init_chg_table(profile, node, batt_drv->battery_capacity);
	}

	if (enabled)
		profile->aact_idx = aact_get_index(batt_drv);

	return 0;
}

/* ------------------------------------------------------------------------- */

/* call holding mutex_lock(&batt_drv->chg_lock); */
static int batt_chg_logic(struct batt_drv *batt_drv)
{
	int rc, level, err = 0;
	bool jeita_stop;
	bool changed = false;
	const bool disable_votes = batt_drv->disable_votes;
	const int ssoc = ssoc_get_capacity(&batt_drv->ssoc_state);
	union gbms_charger_state *chg_state = &batt_drv->chg_state;
	int log_vote_level = BATT_PRLOG_DEBUG;

	if (!batt_drv->chg_profile.cccm_limits)
		return -EINVAL;

	__pm_stay_awake(batt_drv->msc_ws);

	batt_prlog_din(chg_state, BATT_PRLOG_ALWAYS);

	/* disconnect! */
	if (chg_state_is_disconnected(chg_state)) {
		const qnum_t ssoc_delta = ssoc_get_delta(batt_drv);

		if (batt_drv->ssoc_state.buck_enabled == 0)
			goto msc_logic_exit;

		/* update bpst */
		mutex_lock(&batt_drv->bpst_state.lock);
		if (batt_drv->bpst_state.bpst_enable) {
			bool cell_fault_detect = batt_cell_fault_detect(&batt_drv->bpst_state);

			if (cell_fault_detect) {
				rc = batt_bpst_detect_update(batt_drv);
				pr_info("MSC_BPST: cell_fault_detect in disconnected(%d)\n", rc);
			}
		}
		mutex_unlock(&batt_drv->bpst_state.lock);

		/* here on: disconnect */
		batt_log_csi_ttf_info(batt_drv);
		batt_chg_stats_pub(batt_drv, "disconnect", false, false);

		/* change curve before changing the state. */
		ssoc_change_curve(&batt_drv->ssoc_state, ssoc_delta,
				  SSOC_UIC_TYPE_DSG);

		batt_drv->chg_health.rest_deadline = 0;
		batt_reset_chg_drv_state(batt_drv);
		batt_update_cycle_count(batt_drv);
		batt_rl_reset(batt_drv);

		/* aacc: cycle count */
		aacc_update_cycle_count(batt_drv);

		/* charging_policy: vote AC false when disconnected */
		gvotable_cast_long_vote(batt_drv->charging_policy_votable, "MSC_AC",
					CHARGING_POLICY_VOTE_ADAPTIVE_AC, false);

		/* trigger google_capacity learning. */
		err = GPSY_SET_PROP(batt_drv->fg_psy,
				    GBMS_PROP_BATT_CE_CTRL,
				    false);
		if (err < 0)
			pr_err("Cannot set the BATT_CE_CTRL.\n");

		/* TODO: move earlier and include the change to the curve */
		ssoc_change_state(&batt_drv->ssoc_state, 0);
		changed = true;

		/* google_resistance: update and stop accumulation. */
		batt_res_work(batt_drv);
		batt_res_state_set(&batt_drv->health_data.bhi_data.res_state, false);

		batt_bhi_stats_update_all(batt_drv);
		goto msc_logic_done;
	}

	/* executed when going from disconnected to connected */
	if (batt_drv->ssoc_state.buck_enabled <= 0) {
		struct bhi_data *bhi_data = &batt_drv->health_data.bhi_data;
		const qnum_t ssoc_delta = ssoc_get_delta(batt_drv);

		/*
		 * FIX: BatteryDefenderUI needs use a different curve because
		 * bd->bd_voltage_trigger needs now to be 100%. In alternative
		 * we use the regular charge curve and show that charging stop
		 * BEFORE reaching 100%. This is similar to what we do if BD
		 * trigger over bd->bd_voltage_trigger BUT under SSOC=100%
		 */
		ssoc_change_curve(&batt_drv->ssoc_state, ssoc_delta,
				  SSOC_UIC_TYPE_CHG);

		/* google_resistance is calculated while charging */
		if (bhi_data->res_state.estimate_filter)
			batt_res_state_set(&bhi_data->res_state, true);

		/* AgeAdjustedChargingProfiles */
		mutex_lock(&batt_drv->aacp_state_lock);
		aacp_update_opt_out_cutoff(batt_drv);

		err = aact_update_chg_table(batt_drv);
		if (err < 0) {
			struct gbms_chg_profile *profile = &batt_drv->chg_profile;
			struct device_node *node = batt_drv->device->of_node;
			int rc;

			/* reset AACT */
			aact_reset(profile);

			/* set state to unknown and init default charge table */
			batt_drv->aact_state = BATT_AACT_UNKNOWN;
			rc = gbms_init_chg_profile(profile, node);
			if (rc == 0) {
				gbms_init_chg_table(profile, node,
						    aacr_get_capacity_locked(batt_drv));
			}

			pr_err("Cannot update aact charge table (%d)\n", err);
		}

		aafv_update_offset(batt_drv);
		aacr_update_chg_table(batt_drv);

		mutex_unlock(&batt_drv->aacp_state_lock);

		batt_chg_stats_start(batt_drv);

		err = GPSY_SET_PROP(batt_drv->fg_psy, GBMS_PROP_BATT_CE_CTRL, true);
		if (err < 0)
			pr_err("Cannot set the BATT_CE_CTRL (%d)\n", err);

		/* released in battery_work() */
		__pm_stay_awake(batt_drv->poll_ws);
		batt_drv->batt_fast_update_cnt = BATT_WORK_FAST_RETRY_CNT;
		mod_delayed_work(system_wq, &batt_drv->batt_work,
				 msecs_to_jiffies(BATT_WORK_FAST_RETRY_MS));

		/* TODO: move earlier and include the change to the curve */
		ssoc_change_state(&batt_drv->ssoc_state, 1);
		changed = true;

		/* start bpst detect */
		mutex_lock(&batt_drv->bpst_state.lock);
		if (batt_drv->bpst_state.bpst_enable) {
			rc = batt_bpst_detect_begin(&batt_drv->bpst_state);
			if (rc < 0)
				pr_err("MSC_BPST: Cannot start bpst detect\n");
		}
		mutex_unlock(&batt_drv->bpst_state.lock);

		/* reset ttf tier */
		ttf_tier_reset(&batt_drv->ttf_stats);

		batt_log_csi_ttf_info(batt_drv);
	}

	/*
	 * enter RL in DISCHARGE on charger DONE and enter RL in RECHARGE on
	 * battery FULL (i.e. SSOC==100%). charger DONE forces the discharge
	 * curve while RECHARGE will not modify the current curve.
	 */
	if ((batt_drv->chg_state.f.flags & GBMS_CS_FLAG_DONE) != 0) {
		changed = batt_rl_enter(&batt_drv->ssoc_state,
					BATT_RL_STATUS_DISCHARGE);

		batt_drv->chg_done = true;
		batt_drv->ssoc_state.bd_trickle_eoc = true;
	} else if (batt_drv->batt_full) {
		changed = batt_rl_enter(&batt_drv->ssoc_state,
					BATT_RL_STATUS_RECHARGE);
		batt_drv->ssoc_state.bd_trickle_full = true;
	}

	/* will acquire mutex_lock(&batt_drv->aacp_state_lock); */
	err = msc_logic(batt_drv);
	if (err < 0) {
		/* NOTE: google charger will poll again. */
		batt_drv->msc_update_interval = -1;

		batt_prlog(BATT_PRLOG_ALWAYS,
			   "MSC_DOUT ERROR=%d fv_uv=%d cc_max=%d update_interval=%d\n",
			   err, batt_drv->fv_uv, batt_drv->cc_max,
			   batt_drv->msc_update_interval);

		goto msc_logic_exit;
	}

	/*
	 * TODO: might need to behave in a different way when health based
	 * charging is active
	 */
	changed |= msc_logic_health(batt_drv);
	if (CHG_HEALTH_REST_IS_AON(&batt_drv->chg_health, ssoc))
		batt_drv->msc_state = MSC_HEALTH_ALWAYS_ON;
	else if (CHG_HEALTH_REST_IS_ACTIVE(&batt_drv->chg_health))
		batt_drv->msc_state = MSC_HEALTH;
	else if (CHG_HEALTH_REST_IS_PAUSE(&batt_drv->chg_health))
		batt_drv->msc_state = MSC_HEALTH_PAUSE;

msc_logic_done:

	/* set ->cc_max = 0 on RL and SW_JEITA, no vote on interval in RL_DSG */
	if (batt_drv->ssoc_state.rl_status == BATT_RL_STATUS_DISCHARGE) {
		log_vote_level = batt_prlog_level(batt_drv->cc_max != 0);
		batt_drv->msc_update_interval = -1;
		batt_drv->cc_max = 0;
	}

	jeita_stop = batt_drv->jeita_stop_charging == 1;
	if (jeita_stop) {
		log_vote_level = batt_prlog_level(batt_drv->cc_max != 0);
		batt_drv->cc_max = 0;
	}

	if (changed)
		log_vote_level = BATT_PRLOG_ALWAYS;
	batt_prlog(log_vote_level,
		   "%s msc_state=%d cv_cnt=%d ov_cnt=%d rl_sts=%d temp_idx:%d, vbatt_idx:%d  fv_uv=%d cc_max=%d update_interval=%d\n",
		   (disable_votes) ? "MSC_DOUT" : "MSC_VOTE",
		   batt_drv->msc_state,
		   batt_drv->checked_cv_cnt, batt_drv->checked_ov_cnt,
		   batt_drv->ssoc_state.rl_status,
		   batt_drv->temp_idx, batt_drv->vbatt_idx,
		   batt_drv->fv_uv, batt_drv->cc_max,
		   batt_drv->msc_update_interval);

	 /*
	  * google_charger has voted(<=0) on msc_interval_votable and the
	  * votes on fcc and fv_uv will not be applied until google_charger
	  * votes a non-zero value.
	  *
	  * SW_JEITA: ->jeita_stop_charging != 0
	  * . ->msc_update_interval = -1 , fv_uv = -1 and ->cc_max = 0
	  * . vote(0) on ->fcc_votable with SW_JEITA_VOTER
	  * BATT_RL: rl_status == BATT_RL_STATUS_DISCHARGE
	  * . ->msc_update_interval = -1 , fv_uv = -1 and ->cc_max = 0
	  * . vote(0) on ->fcc_votable with SW_JEITA_VOTER
	  *
	  * Votes for MSC_LOGIC_VOTER will be all disabled.
	  */
	if (!batt_drv->fv_votable)
		batt_drv->fv_votable =
			gvotable_election_get_handle(VOTABLE_MSC_FV);
	if (batt_drv->fv_votable) {
		/* make sure using rest_fv_uv when HEALTH_ACTIVE */
		const int rest_fv_uv = batt_drv->chg_health.rest_fv_uv;
		const int fv_uv = rest_fv_uv > 0 ? 0 : batt_drv->fv_uv;

		gvotable_cast_int_vote(batt_drv->fv_votable,
				       MSC_LOGIC_VOTER, fv_uv,
				       !disable_votes && (fv_uv > 0));

		gvotable_cast_int_vote(batt_drv->fv_votable,
				       MSC_HEALTH_VOTER, rest_fv_uv,
				       !disable_votes && (rest_fv_uv > 0));
	}

	if (!batt_drv->fcc_votable)
		batt_drv->fcc_votable =
			gvotable_election_get_handle(VOTABLE_MSC_FCC);
	if (batt_drv->fcc_votable) {
		enum batt_rl_status rl_status = batt_drv->ssoc_state.rl_status;
		const int rest_cc_max = batt_drv->chg_health.rest_cc_max;
		struct batt_bpst *bpst_state = &batt_drv->bpst_state;

		/* while in RL => ->cc_max != -1 && ->fv_uv != -1 */
		gvotable_cast_int_vote(batt_drv->fcc_votable, RL_STATE_VOTER, 0,
				       !disable_votes &&
				       (rl_status == BATT_RL_STATUS_DISCHARGE));

		/* jeita_stop_charging != 0 => ->fv_uv = -1 && cc_max == -1 */
		gvotable_cast_int_vote(batt_drv->fcc_votable, SW_JEITA_VOTER, 0,
				       !disable_votes && jeita_stop);

		/* health based charging */
		gvotable_cast_int_vote(batt_drv->fcc_votable,
				       MSC_HEALTH_VOTER, rest_cc_max,
				       !disable_votes && (rest_cc_max != -1));

		gvotable_cast_int_vote(batt_drv->fcc_votable,
				       MSC_LOGIC_VOTER, batt_drv->cc_max,
				       !disable_votes &&
				       (batt_drv->cc_max != -1));

		/* bpst detection */
		if (bpst_state->bpst_detect_disable || bpst_state->bpst_cell_fault) {
			const int chg_rate = batt_drv->bpst_state.bpst_chg_rate;
			const int bpst_cc_max = (batt_drv->cc_max == -1) ? batt_drv->cc_max
							: ((batt_drv->cc_max * chg_rate) / 100);

			gvotable_cast_int_vote(batt_drv->fcc_votable,
					       BPST_DETECT_VOTER, bpst_cc_max,
					       !disable_votes &&
					       (bpst_cc_max != -1));
		}
	}

	/* Fan level can be updated only during power transfer */
	level = fan_calculate_level(batt_drv);
	vote_fan_level(batt_drv->fan_level_votable, level, true);
	pr_debug("MSC_FAN_LVL: level=%d\n", level);

	if (!batt_drv->msc_interval_votable)
		batt_drv->msc_interval_votable =
			gvotable_election_get_handle(VOTABLE_MSC_INTERVAL);
	if (batt_drv->msc_interval_votable)
		gvotable_cast_int_vote(batt_drv->msc_interval_votable,
				       MSC_LOGIC_VOTER,
				       batt_drv->msc_update_interval,
				       !disable_votes &&
				       (batt_drv->msc_update_interval != -1));

	batt_update_csi_info(batt_drv);

msc_logic_exit:

	if (changed) {
		dump_ssoc_state(&batt_drv->ssoc_state, batt_drv->ssoc_log);
		if (batt_drv->psy)
			power_supply_changed(batt_drv->psy);
	}

	__pm_relax(batt_drv->msc_ws);
	return err;
}

/*
 * called at boot and when switching profiles from debugfs().
 * NOTE: aacr_get_capacity_locked() requires mutex_lock(&batt_drv->aacp_state_lock)
 */
static int batt_init_chg_profile(struct batt_drv *batt_drv, struct device_node *node)
{
	struct gbms_chg_profile *profile = &batt_drv->chg_profile;
	int ret = 0;

	/* handle retry */
	if (!profile->cccm_limits) {
		ret = gbms_init_chg_profile(profile, node);
		if (ret < 0)
			return -EINVAL;
	}

	/* this is in mAh */
	ret = of_property_read_u32(gbms_batt_id_node(node),
				   "google,chg-battery-capacity",
				    &batt_drv->battery_capacity);
	/* google,chg-battery-capacity does not exist in the child_node */
	if (ret < 0)
		ret = of_property_read_u32(node, "google,chg-battery-capacity",
					   &batt_drv->battery_capacity);
	if (ret < 0)
		pr_warn("read chg-battery-capacity from gauge\n");

	/*
	 * use battery FULL design when is not specified in DT. When battery is
	 * not present use default capacity from DT (if present) or disable
	 * charging altogether.
	 */
	if (batt_drv->battery_capacity == 0) {
		u32 fc = 0;
		struct power_supply *fg_psy = batt_drv->fg_psy;

		if (batt_drv->batt_present) {
			fc = GPSY_GET_PROP(fg_psy, POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN);
			if (fc == -EAGAIN)
				return -EPROBE_DEFER;
			if (fc > 0) {
				pr_info("successfully read charging profile:\n");
				/* convert uA to mAh*/
				batt_drv->battery_capacity = fc / 1000;
			}

		}

		if (batt_drv->battery_capacity == 0) {
			ret = of_property_read_u32(node,
					"google,chg-battery-default-capacity",
						&batt_drv->battery_capacity);
			if (ret < 0)
				pr_warn("battery not present, no default capacity, zero charge table\n");
			else
				pr_warn("battery not present, using default capacity\n");
		}
	}

	/*
	 * TODO: b/403865140 move AACR init code to batt_init_aact_profile()
	 * and call it from google_battery_init_work() after BRID is available.
	 */

	ret = gbms_read_aacr_limits(profile, gbms_batt_id_node(node));
	if (ret == 0)
		pr_info("AACR: supported\n");

	/*
	 * AACR tables (profile->aacr_nb_limits) in DT enable AACR classic by
	 * default UNLESS the feature is disabled via "google,aacr-disable".
	 *
	 *   ret = of_property_read_bool(node, "google,aacr-disable");
	 *   if (!ret && profile->aacr_nb_limits)
	 * 	  batt_drv->aacr_state = BATT_AACR_ENABLED;
	 *   else
	 *	  batt_drv->aacr_state = BATT_AACR_DISABLED;
	 *
	 * With AACR+H the state depends from the configuration. Default state
	 * on a valid configuration is enabled.
	 */

	ret = of_property_read_u32(node, "google,aacr-config",
				   &batt_drv->aacr_state);
	if (ret < 0)
		batt_drv->aacr_state = profile->aacr_nb_limits ?
			BATT_AACR_DISABLED : BATT_AACR_UNKNOWN;

	ret = of_property_read_u32(node, "google,aacr-algo", &batt_drv->aacr_algo);
	if (ret < 0)
		batt_drv->aacr_algo = BATT_AACR_ALGO_DEFAULT;

	ret = of_property_read_u32(node, "google,aacr-min-capacity-rate",
				   &batt_drv->aacr_min_capacity_rate);
	if (ret < 0)
		batt_drv->aacr_min_capacity_rate = 80; /* default rate */

	ret = of_property_read_u32(node, "google,aacr-cliff-capacity-rate",
				   &batt_drv->aacr_cliff_capacity_rate);
	if (ret < 0)
		batt_drv->aacr_cliff_capacity_rate = 50; /* default rate */

	ret = of_property_read_u32(node, "google,aacr-cycle-max", &batt_drv->aacr_cycle_max);
	if (ret < 0)
		batt_drv->aacr_cycle_max = AACR_MAX_CYCLE_DEFAULT; /* default rate */

	ret = of_property_read_u32(node, "google,aacr-cycle-grace", &batt_drv->aacr_cycle_grace);
	if (ret < 0)
		batt_drv->aacr_cycle_grace = AACR_START_CYCLE_DEFAULT; /* default rate */

	/* NOTE: with NG charger tolerance is applied from "charger" */
	gbms_init_chg_table(profile, node, aacr_get_capacity_locked(batt_drv));

	return 0;
}

/* ------------------------------------------------------------------------- */

/* call holding mutex_unlock(&ccd->lock); */
static int batt_cycle_count_store(struct gbatt_ccbin_data *ccd)
{
	int ret;

	ret = gbms_storage_write(GBMS_TAG_BCNT, ccd->count, sizeof(ccd->count));
	if (ret < 0 && ret != -ENOENT) {
		pr_err("failed to set bin_counts ret=%d\n", ret);
		return ret;
	}

	return 0;
}

/* call holding mutex_unlock(&ccd->lock); */
static int batt_cycle_count_load(struct gbatt_ccbin_data *ccd)
{
	int ret, i;

	ret = gbms_storage_read(GBMS_TAG_BCNT, ccd->count, sizeof(ccd->count));
	if (ret < 0 && ret != -ENOENT) {
		pr_err("failed to get bin_counts ret=%d\n", ret);
		return ret;
	}

	for (i = 0; i < GBMS_CCBIN_BUCKET_COUNT; i++)
		if (ccd->count[i] == 0xFFFF)
			ccd->count[i] = 0;

	ccd->prev_soc = -1;
	return 0;
}

/* update only when SSOC is increasing, not need to check charging */
static void batt_cycle_count_update(struct batt_drv *batt_drv, int soc)
{
	struct gbatt_ccbin_data *ccd = &batt_drv->cc_data;

	if (soc < 0 || soc > 100)
		return;

	mutex_lock(&ccd->lock);

	if (ccd->prev_soc != -1 && soc > ccd->prev_soc) {
		int bucket, cnt;

		for (cnt = soc ; cnt > ccd->prev_soc ; cnt--) {
			/* cnt decremented by 1 for bucket symmetry */
			bucket = (cnt - 1) * GBMS_CCBIN_BUCKET_COUNT / 100;
			ccd->count[bucket]++;
		}

		/* NOTE: could store on FULL or disconnect instead */
		(void)batt_cycle_count_store(ccd);
	}

	ccd->prev_soc = soc;

	mutex_unlock(&ccd->lock);
}

/* ------------------------------------------------------------------------- */

static ssize_t cycle_counts_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	int ret;

	mutex_lock(&batt_drv->cc_data.lock);

	ret = gbms_cycle_count_sscan(batt_drv->cc_data.count, buf);
	if (ret == 0) {
		ret = batt_cycle_count_store(&batt_drv->cc_data);
		if (ret < 0)
			pr_err("cannot store bin count ret=%d\n", ret);
	}

	if (ret == 0)
		ret = count;

	mutex_unlock(&batt_drv->cc_data.lock);

	return ret;
}

static ssize_t cycle_counts_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buff)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	int len;

	mutex_lock(&batt_drv->cc_data.lock);
	len = gbms_cycle_count_cstr(buff, PAGE_SIZE, batt_drv->cc_data.count);
	mutex_unlock(&batt_drv->cc_data.lock);

	return len;
}

static DEVICE_ATTR_RW(cycle_counts);

static ssize_t resistance_show(struct device *dev, struct device_attribute *attr, char *buff)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	int value = -1;

	if (batt_drv->fg_psy)
		value = GPSY_GET_PROP(batt_drv->fg_psy, GBMS_PROP_RESISTANCE);

	return scnprintf(buff, PAGE_SIZE, "%d\n", value);
}

static DEVICE_ATTR_RO(resistance);

static ssize_t resistance_avg_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buff)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);

	/* resistance_avg is scaled */
	return scnprintf(buff, PAGE_SIZE, "%d\n",
			 batt_ravg_value(&batt_drv->health_data.bhi_data.res_state));
}

static DEVICE_ATTR_RO(resistance_avg);

static ssize_t charge_full_estimate_show(struct device *dev, struct device_attribute *attr,
					 char *buff)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	int value = -1;

	if (batt_drv->fg_psy)
		value = GPSY_GET_PROP(batt_drv->fg_psy, GBMS_PROP_CHARGE_FULL_ESTIMATE);

	return scnprintf(buff, PAGE_SIZE, "%d\n", value);
}

static DEVICE_ATTR_RO(charge_full_estimate);

#ifdef CONFIG_DEBUG_FS

static int cycle_count_bins_store(void *data, u64 val)
{
	struct batt_drv *batt_drv = (struct batt_drv *)data;
	int ret;

	mutex_lock(&batt_drv->cc_data.lock);
	ret = batt_cycle_count_store(&batt_drv->cc_data);
	if (ret < 0)
		pr_err("cannot store bin count ret=%d\n", ret);
	mutex_unlock(&batt_drv->cc_data.lock);

	return ret;
}

static int cycle_count_bins_reload(void *data, u64 *val)
{
	struct batt_drv *batt_drv = (struct batt_drv *)data;
	int ret;

	mutex_lock(&batt_drv->cc_data.lock);
	ret = batt_cycle_count_load(&batt_drv->cc_data);
	if (ret < 0)
		pr_err("cannot restore bin count ret=%d\n", ret);
	mutex_unlock(&batt_drv->cc_data.lock);
	*val = ret;

	return ret;
}

DEFINE_SIMPLE_ATTRIBUTE(cycle_count_bins_sync_fops,
				cycle_count_bins_reload,
				cycle_count_bins_store, "%llu\n");


static int debug_get_ssoc_gdf(void *data, u64 *val)
{
	struct batt_drv *batt_drv = (struct batt_drv *)data;
	*val = batt_drv->ssoc_state.ssoc_gdf;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_ssoc_gdf_fops, debug_get_ssoc_gdf, NULL, "%llu\n");


static int debug_get_ssoc_uic(void *data, u64 *val)
{
	struct batt_drv *batt_drv = (struct batt_drv *)data;
	*val = batt_drv->ssoc_state.ssoc_uic;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_ssoc_uic_fops, debug_get_ssoc_uic, NULL, "%llu\n");

static int debug_get_ssoc_rls(void *data, u64 *val)
{
	struct batt_drv *batt_drv = (struct batt_drv *)data;

	mutex_lock(&batt_drv->chg_lock);
	*val = batt_drv->ssoc_state.rl_status;
	mutex_unlock(&batt_drv->chg_lock);

	return 0;
}

static int debug_set_ssoc_rls(void *data, u64 val)
{
	struct batt_drv *batt_drv = (struct batt_drv *)data;

	if (val < 0 || val > 2)
		return -EINVAL;

	mutex_lock(&batt_drv->chg_lock);
	batt_drv->ssoc_state.rl_status = val;
	if (!batt_drv->fcc_votable)
		batt_drv->fcc_votable =
			gvotable_election_get_handle(VOTABLE_MSC_FCC);
	if (batt_drv->fcc_votable)
		gvotable_cast_int_vote(batt_drv->fcc_votable, RL_STATE_VOTER, 0,
				       batt_drv->ssoc_state.rl_status ==
				       BATT_RL_STATUS_DISCHARGE);
	mutex_unlock(&batt_drv->chg_lock);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_ssoc_rls_fops,
				debug_get_ssoc_rls, debug_set_ssoc_rls, "%llu\n");

static int debug_get_fv_dc_ratio(void *data, u64 *val)
{
	struct batt_drv *batt_drv = (struct batt_drv *)data;

	*val = batt_drv->chg_profile.fv_dc_ratio;

	return 0;
}

static int debug_set_fv_dc_ratio(void *data, u64 val)
{
	struct batt_drv *batt_drv = (struct batt_drv *)data;

	if (val < 0)
		return -EINVAL;

	mutex_lock(&batt_drv->chg_lock);
	batt_drv->chg_profile.fv_dc_ratio = val;
	mutex_unlock(&batt_drv->chg_lock);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_fv_dc_ratio_fops,
			debug_get_fv_dc_ratio, debug_set_fv_dc_ratio, "%llu\n");

static int debug_get_mp_tz(void *data, u64 *val)
{
	struct batt_drv *batt_drv = (struct batt_drv *)data;

	*val = batt_drv->need_mp;
	return 0;
}

static int debug_set_mp_tz(void *data, u64 val)
{
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_mp_tz_fops, debug_get_mp_tz, debug_set_mp_tz, "%llu\n");

static ssize_t debug_get_ssoc_uicurve(struct file *filp,
					   char __user *buf,
					   size_t count, loff_t *ppos)
{
	struct batt_drv *batt_drv = (struct batt_drv *)filp->private_data;
	char tmp[UICURVE_BUF_SZ] = { 0 };

	if (*ppos)
		return 0;

	mutex_lock(&batt_drv->chg_lock);
	ssoc_uicurve_cstr(tmp, sizeof(tmp), batt_drv->ssoc_state.ssoc_curve);
	mutex_unlock(&batt_drv->chg_lock);

	return simple_read_from_buffer(buf, count, ppos, tmp, strlen(tmp));
}

static ssize_t debug_set_ssoc_uicurve(struct file *filp,
					 const char __user *user_buf,
					 size_t count, loff_t *ppos)
{
	struct batt_drv *batt_drv = (struct batt_drv *)filp->private_data;
	int ret, curve_type;
	char buf[8] = {0};

	ret = simple_write_to_buffer(buf, sizeof(buf), ppos, user_buf, count);
	if (!ret)
		return -EFAULT;

	mutex_lock(&batt_drv->chg_lock);

	/* FIX: BatteryDefenderUI doesn't really handle this yet */
	curve_type = (int)simple_strtoull(buf, NULL, 10);
	if (curve_type >= -1 && curve_type <= 1)
		ssoc_change_curve(&batt_drv->ssoc_state, 0, curve_type);
	else
		ret = -EINVAL;

	mutex_unlock(&batt_drv->chg_lock);

	if (ret == 0)
		ret = count;

	return 0;
}

BATTERY_DEBUG_ATTRIBUTE(debug_ssoc_uicurve_cstr_fops,
					debug_get_ssoc_uicurve,
					debug_set_ssoc_uicurve);

static int debug_force_psy_update(void *data, u64 val)
{
	struct batt_drv *batt_drv = (struct batt_drv *)data;

	if (!batt_drv->psy)
		return -EINVAL;

	power_supply_changed(batt_drv->psy);
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_force_psy_update_fops,
				NULL, debug_force_psy_update, "%llu\n");

/* Adaptive Charging */
static int debug_chg_health_rest_rate_read(void *data, u64 *val)
{
	struct batt_drv *batt_drv = data;

	if (!batt_drv->psy)
		return -EINVAL;

	*val = batt_drv->chg_health.rest_rate;
	return 0;
}

/* Adaptive Charging */
static int debug_chg_health_rest_rate_write(void *data, u64 val)
{
	struct batt_drv *batt_drv = data;

	if (!batt_drv->psy)
		return -EINVAL;

	batt_drv->chg_health.rest_rate = val;
	return 0;
}

/* Adaptive Charging */
DEFINE_SIMPLE_ATTRIBUTE(debug_chg_health_rest_rate_fops,
			debug_chg_health_rest_rate_read,
			debug_chg_health_rest_rate_write, "%llu\n");


/* Adaptive Charging */
static int debug_chg_health_rest_rate_before_trigger_read(void *data, u64 *val)
{
	struct batt_drv *batt_drv = data;

	if (!batt_drv->psy)
		return -EINVAL;

	*val = batt_drv->chg_health.rest_rate_before_trigger;
	return 0;
}

/* Adaptive Charging */
static int debug_chg_health_rest_rate_before_trigger_write(void *data, u64 val)
{
	struct batt_drv *batt_drv = data;

	if (!batt_drv->psy)
		return -EINVAL;

	batt_drv->chg_health.rest_rate_before_trigger = val;
	return 0;
}

/* Adaptive Charging */
DEFINE_SIMPLE_ATTRIBUTE(debug_chg_health_rest_rate_before_trigger_fops,
			debug_chg_health_rest_rate_before_trigger_read,
			debug_chg_health_rest_rate_before_trigger_write, "%llu\n");

/* Adaptive Charging */
static int debug_chg_health_thr_soc_read(void *data, u64 *val)
{
	struct batt_drv *batt_drv = (struct batt_drv *)data;

	if (!batt_drv->psy)
		return -EINVAL;

	*val = batt_drv->chg_health.rest_soc;
	return 0;
}

/* Adaptive Charging */
static int debug_chg_health_thr_soc_write(void *data, u64 val)
{
	struct batt_drv *batt_drv = (struct batt_drv *)data;

	if (!batt_drv->psy)
		return -EINVAL;

	batt_drv->chg_health.rest_soc = val;
	return 0;
}

/* Adaptive Charging */
DEFINE_SIMPLE_ATTRIBUTE(debug_chg_health_thr_soc_fops,
			debug_chg_health_thr_soc_read,
			debug_chg_health_thr_soc_write, "%llu\n");

/* Adaptive Charging */
static int debug_chg_health_set_stage(void *data, u64 val)
{
	struct batt_drv *batt_drv = (struct batt_drv *)data;

	if (!batt_drv->psy)
		return -EINVAL;

	switch (val) {
	case CHG_HEALTH_DISABLED:
	case CHG_HEALTH_INACTIVE:
	case CHG_HEALTH_ENABLED:
	case CHG_HEALTH_ACTIVE:
	case CHG_HEALTH_DONE:
		break;
	default:
		return -EINVAL;
	}

	batt_drv->chg_health.rest_state = val;
	return 0;
}

/* Adaptive Charging */
DEFINE_SIMPLE_ATTRIBUTE(debug_chg_health_stage_fops, NULL,
			debug_chg_health_set_stage, "%llu\n");

/* debug variable */
static int raw_profile_cycles;

static ssize_t debug_get_chg_raw_profile(struct file *filp,
					 char __user *buf,
					 size_t count, loff_t *ppos)
{
	struct batt_drv *batt_drv = (struct batt_drv *)filp->private_data;
	char *tmp;
	int len;

	if (*ppos)
		return 0;

	tmp = kzalloc(PAGE_SIZE, GFP_KERNEL);
	if (!tmp)
		return -ENOMEM;

	mutex_lock(&batt_drv->aacp_state_lock);
	if (raw_profile_cycles) {
		struct gbms_chg_profile profile;
		int count;
		const bool enabled = aact_enabled(batt_drv->aact_state, batt_drv->aacp_opt_out);

		len = gbms_init_chg_profile(&profile, batt_drv->device->of_node);
		if (len < 0)
			goto exit_done;

		if (enabled) {
			len = gbms_init_aact_profile(&profile, batt_drv->device->of_node);
			if (len < 0)
				goto exit_done;
			len = gbms_update_chg_profile_from_aact(&profile);
			if (len < 0)
				goto exit_done;
			profile.aact_idx = aact_get_index(batt_drv);
		}

		/* len is the capacity */
		len = aacr_get_capacity_at_cycle(batt_drv, raw_profile_cycles);
		if (len <= 0) {
			gbms_free_chg_profile(&profile);
			goto exit_done;
		}

		count = scnprintf(tmp, PAGE_SIZE, "AACR Profile at %d cycles\n",
				  raw_profile_cycles);
		gbms_init_chg_table(&profile, batt_drv->device->of_node, len);
		gbms_dump_chg_profile(&tmp[count], PAGE_SIZE - count, &profile);
		gbms_free_chg_profile(&profile);
	} else {
		gbms_dump_chg_profile(tmp, PAGE_SIZE, &batt_drv->chg_profile);
	}

	len = simple_read_from_buffer(buf, count, ppos, tmp, strlen(tmp));

exit_done:
	mutex_unlock(&batt_drv->aacp_state_lock);
	kfree(tmp);
	return len;
}

static ssize_t debug_set_chg_raw_profile(struct file *filp,
					 const char __user *user_buf,
					 size_t count, loff_t *ppos)
{
	int ret = 0, val;
	char buf[8];

	ret = simple_write_to_buffer(buf, sizeof(buf), ppos, user_buf, count);
	if (!ret)
		return -EFAULT;

	buf[ret] = '\0';
	ret = kstrtoint(buf, 0, &val);
	if (ret < 0)
		return ret;

	raw_profile_cycles = val;
	return count;
}

BATTERY_DEBUG_ATTRIBUTE(debug_chg_raw_profile_fops,
			debug_get_chg_raw_profile,
			debug_set_chg_raw_profile);

static ssize_t debug_get_power_metrics(struct file *filp, char __user *buf,
				       size_t count, loff_t *ppos)
{
	struct batt_drv *batt_drv = (struct batt_drv *)filp->private_data;
	char *tmp;
	int idx, len = 0;

	if (*ppos)
		return 0;

	tmp = kzalloc(PAGE_SIZE, GFP_KERNEL);
	if (!tmp)
		return -ENOMEM;

	for(idx = 0; idx < POWER_METRICS_MAX_DATA; idx++) {
		len += scnprintf(&tmp[len], PAGE_SIZE - len, "%2d: %8ld/%8ld - %5lld\n", idx,
				 batt_drv->power_metrics.data[idx].voltage,
				 batt_drv->power_metrics.data[idx].charge_count,
				 batt_drv->power_metrics.data[idx].time);
	}

	len = simple_read_from_buffer(buf, count, ppos, tmp, strlen(tmp));
	kfree(tmp);

	return len;
}

BATTERY_DEBUG_ATTRIBUTE(debug_power_metrics_fops, debug_get_power_metrics, NULL);

static int debug_bpst_sbd_status_read(void *data, u64 *val)
{
	struct batt_drv *batt_drv = (struct batt_drv *)data;

	*val = batt_drv->bpst_state.bpst_sbd_status;
	return 0;
}

static int debug_bpst_sbd_status_write(void *data, u64 val)
{
	struct batt_drv *batt_drv = (struct batt_drv *)data;

	if (val < 0 || val > 1)
		return -EINVAL;

	mutex_lock(&batt_drv->bpst_state.lock);
	batt_drv->bpst_state.bpst_sbd_status = val;
	mutex_unlock(&batt_drv->bpst_state.lock);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_bpst_sbd_status_fops,
			debug_bpst_sbd_status_read,
			debug_bpst_sbd_status_write, "%llu\n");

static int debug_ravg_fops_write(void *data, u64 val)
{
	struct batt_drv *batt_drv = (struct batt_drv *)data;
	struct batt_res *res_state = &batt_drv->health_data.bhi_data.res_state;
	int resistance_avg = val / 100, filter_count = 1;
	int ret;

	mutex_lock(&batt_drv->chg_lock);

	batt_res_state_set(res_state, false);
	res_state->resistance_avg = resistance_avg;
	res_state->filter_count = filter_count;

	/* reset storage to defaults */
	if (val == 0) {
		resistance_avg = 0xffff;
		filter_count = 0xffff;
	}

	ret = batt_ravg_write(resistance_avg, filter_count);
	pr_info("RAVG: update val=%d, resistance_avg=%x filter_count=%x (%d)\n",
		(int)val, resistance_avg, filter_count, ret);
	mutex_unlock(&batt_drv->chg_lock);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_ravg_fops, NULL, debug_ravg_fops_write, "%llu\n");

#endif

/* ------------------------------------------------------------------------- */

static ssize_t debug_get_fake_temp(struct file *filp,
					   char __user *buf,
					   size_t count, loff_t *ppos)
{
	struct batt_drv *batt_drv = (struct batt_drv *)filp->private_data;
	char tmp[8];

	if (*ppos)
		return 0;

	mutex_lock(&batt_drv->chg_lock);
	scnprintf(tmp, sizeof(tmp), "%d\n", batt_drv->fake_temp);
	mutex_unlock(&batt_drv->chg_lock);

	return simple_read_from_buffer(buf, count, ppos, tmp, strlen(tmp));
}

static ssize_t debug_set_fake_temp(struct file *filp,
					 const char __user *user_buf,
					 size_t count, loff_t *ppos)
{
	struct batt_drv *batt_drv = (struct batt_drv *)filp->private_data;
	int ret = 0, val;
	char buf[8];

	ret = simple_write_to_buffer(buf, sizeof(buf), ppos, user_buf, count);
	if (!ret)
		return -EFAULT;

	buf[ret] = '\0';
	ret = kstrtoint(buf, 0, &val);
	if (ret < 0)
		return ret;

	mutex_lock(&batt_drv->chg_lock);
	batt_drv->fake_temp = val;
	mutex_unlock(&batt_drv->chg_lock);

	return count;
}

BATTERY_DEBUG_ATTRIBUTE(debug_fake_temp_fops, debug_get_fake_temp,
			debug_set_fake_temp);


static enum batt_paired_state
batt_reset_pairing_state(const struct batt_drv *batt_drv)
{
	char dev_info[GBMS_DINF_LEN];
	int ret = 0;

	memset(dev_info, 0xff, sizeof(dev_info));
	ret = gbms_storage_write(GBMS_TAG_DINF, dev_info, sizeof(dev_info));
	if (ret < 0)
		return -EIO;

	return 0;
}

static ssize_t debug_set_pairing_state(struct file *filp,
					 const char __user *user_buf,
					 size_t count, loff_t *ppos)
{
	struct batt_drv *batt_drv = (struct batt_drv *)filp->private_data;
	int ret = 0, val;
	char buf[8];

	ret = simple_write_to_buffer(buf, sizeof(buf), ppos, user_buf, count);
	if (ret <= 0)
		return ret;

	buf[ret] = '\0';
	ret = kstrtoint(buf, 0, &val);
	if (ret < 0)
		return ret;

	mutex_lock(&batt_drv->chg_lock);

	if (val == BATT_PAIRING_ENABLED) {
		batt_drv->pairing_state = BATT_PAIRING_ENABLED;
		mod_delayed_work(system_wq, &batt_drv->batt_work, 0);
	} else if (val == BATT_PAIRING_RESET) {

		/* send a paring enable to re-pair OR reboot */
		ret = batt_reset_pairing_state(batt_drv);
		if (ret == 0)
			batt_drv->pairing_state = BATT_PAIRING_DISABLED;
		else
			count = -EIO;
	} else {
		count = -EINVAL;
	}
	mutex_unlock(&batt_drv->chg_lock);

	return count;
}

BATTERY_DEBUG_ATTRIBUTE(debug_pairing_fops, 0, debug_set_pairing_state);

/* TODO: add write to stop/start collection, erase history etc. */
static int debug_get_blf_state(void *data, u64 *val)
{
	struct batt_drv *batt_drv = (struct batt_drv *)data;

	mutex_lock(&batt_drv->chg_lock);
	*val = batt_drv->blf_state;
	mutex_unlock(&batt_drv->chg_lock);

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(debug_blf_state_fops, debug_get_blf_state, NULL, "%llu\n");

static ssize_t debug_get_bhi_status(struct file *filp, char __user *buf,
				   size_t count, loff_t *ppos)
{
	struct batt_drv *batt_drv = (struct batt_drv *)filp->private_data;
	struct health_data *health_data = &batt_drv->health_data;
	const int cap_idx = health_data->bhi_debug_cap_index;
	const int imp_idx = health_data->bhi_debug_imp_index;
	const int sd_idx = health_data->bhi_debug_sd_index;
	const int algo = BHI_ALGO_DEBUG;
	int health_idx = health_data->bhi_debug_health_index;
	int health_status, len;
	char *tmp;

	tmp = kzalloc(PAGE_SIZE, GFP_KERNEL);
	if (!tmp)
		return -ENOMEM;

	if (health_idx == 0)
		health_idx = bhi_calc_health_index(algo, health_data, cap_idx, imp_idx, sd_idx);

	health_status = bhi_calc_health_status(algo, BHI_ROUND_INDEX(health_idx), health_data);

	if (health_data->bhi_debug_health_index != 0)
		scnprintf(tmp, PAGE_SIZE, "%d, %d\n", health_status, health_idx);
	else
		scnprintf(tmp, PAGE_SIZE, "%d, %d [%d/%d %d/%d %d/%d]\n", health_status,
			  health_idx, cap_idx, health_data->bhi_w_ci, imp_idx,
			  health_data->bhi_w_pi, sd_idx, health_data->bhi_w_sd);

	len = simple_read_from_buffer(buf, count, ppos, tmp, strlen(tmp));
	kfree(tmp);

	return len;
}
BATTERY_DEBUG_ATTRIBUTE(debug_bhi_status_fops, debug_get_bhi_status, 0);

static ssize_t debug_set_first_usage_date(struct file *filp,
					  const char __user *user_buf,
					  size_t count, loff_t *ppos)
{
	struct batt_drv *batt_drv = (struct batt_drv *)filp->private_data;
	struct bhi_data *bhi_data = &batt_drv->health_data.bhi_data;
	int ret = 0, val;
	char buf[8];

	ret = simple_write_to_buffer(buf, sizeof(buf), ppos, user_buf, count);
	if (!ret)
		return -EFAULT;

	buf[ret] = '\0';
	ret = kstrtoint(buf, 0, &val);
	if (ret < 0)
		return ret;

	/* reset device activation date */
	if (val == 1) {
		u8 act_date[BATT_EEPROM_TAG_XYMD_LEN];

		memset(act_date, 0xff, sizeof(act_date));
		ret = gbms_storage_write(GBMS_TAG_AYMD, act_date, sizeof(act_date));
		if (ret < 0)
			return -EINVAL;

		/* set a default value */
		bhi_data->act_date[0] = 0x30; /* 0x30 = '0', 2020 */
		bhi_data->act_date[1] = 0x43; /* 0x43 = 'C', 12th */
		bhi_data->act_date[2] = 0x31; /* 0x31 = '1', 1st */
	}

	return count;
}

BATTERY_DEBUG_ATTRIBUTE(debug_first_usage_date_fops, 0, debug_set_first_usage_date);

static ssize_t chg_profile_switch_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	int len;

	len = scnprintf(buf, PAGE_SIZE, "%d\n", batt_drv->chg_profile.debug_chg_profile);

	return len;
}

static ssize_t chg_profile_switch_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv =(struct batt_drv *) power_supply_get_drvdata(psy);
	struct device_node *node;
	int val, ret;

	if (!batt_drv->chg_profile.enable_switch_chg_profile)
		return count;

	ret = kstrtoint(buf, 0, &val);
	if (ret < 0)
		return ret;

	if (batt_drv->chg_profile.debug_chg_profile == !!val)
		return count;

	if (val)
		node = of_find_node_by_name(batt_drv->device->of_node,
					    "google_debug_chg_profile");
	else
		node = batt_drv->device->of_node;

	if (!node)
		return count;

	batt_drv->chg_profile.cccm_limits = 0;
	ret = batt_init_chg_profile(batt_drv, node);
	if (ret < 0)
		return ret;

	pr_info("update debug_chg_profile:%d -> %d\n",
		batt_drv->chg_profile.debug_chg_profile, (int)val);

	batt_drv->chg_profile.debug_chg_profile = val;

	return count;
}

static DEVICE_ATTR_RW(chg_profile_switch);


/* TODO: add writes to restart pairing (i.e. provide key) */
static ssize_t pairing_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	int len;

	mutex_lock(&batt_drv->chg_lock);
	len = scnprintf(buf, PAGE_SIZE, "%d\n", batt_drv->pairing_state);
	mutex_unlock(&batt_drv->chg_lock);
	return len;
}

static DEVICE_ATTR_RO(pairing_state);

static ssize_t charge_stats_actual_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv =(struct batt_drv *)
					power_supply_get_drvdata(psy);

	if (count < 1)
		return -ENODATA;

	switch (buf[0]) {
	case 'p': /* publish data to qual */
	case 'P': /* force publish data to qual */
		batt_chg_stats_pub(batt_drv, "debug cmd", buf[0] == 'P', false);
		break;
	default:
		count = -EINVAL;
		break;
	}

	return count;
}

static ssize_t charge_stats_actual_show(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv =(struct batt_drv *)
					power_supply_get_drvdata(psy);
	int len;

	mutex_lock(&batt_drv->stats_lock);
	len = batt_chg_stats_cstr(buf, PAGE_SIZE, &batt_drv->ce_data, false,
			aacr_filtered_capacity(batt_drv, &batt_drv->ce_data));
	mutex_unlock(&batt_drv->stats_lock);

	return len;
}

static DEVICE_ATTR_RW(charge_stats_actual);

static ssize_t charge_stats_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv =(struct batt_drv *)
					power_supply_get_drvdata(psy);

	if (count < 1)
		return -ENODATA;

	mutex_lock(&batt_drv->stats_lock);
	switch (buf[0]) {
	case 0:
	case '0': /* invalidate current qual */
		cev_stats_init(&batt_drv->ce_qual, &batt_drv->chg_profile);
		break;
	}
	mutex_unlock(&batt_drv->stats_lock);

	return count;
}

/* regular and health stats */
static ssize_t batt_chg_qual_stats_cstr(char *buff, int size,
					struct gbms_charging_event *ce_qual,
					bool verbose, int state_capacity)
{
	ssize_t len = 0;

	len += batt_chg_stats_cstr(&buff[len], size - len, ce_qual, verbose, state_capacity);
	if (ce_qual->ce_health.rest_state != CHG_HEALTH_INACTIVE)
		len += batt_health_stats_cstr(&buff[len], size - len,
					      ce_qual, verbose);
	return len;
}

static ssize_t charge_stats_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv =(struct batt_drv *)
					power_supply_get_drvdata(psy);
	struct gbms_charging_event *ce_qual = &batt_drv->ce_qual;
	int len = -ENODATA;

	mutex_lock(&batt_drv->stats_lock);
	if (ce_qual->last_update - ce_qual->first_update)
		len = batt_chg_qual_stats_cstr(buf, PAGE_SIZE, ce_qual, false,
					aacr_filtered_capacity(batt_drv, ce_qual));
	mutex_unlock(&batt_drv->stats_lock);

	return len;
}

static DEVICE_ATTR_RW(charge_stats);

/* show current/active and qual data */
static ssize_t charge_details_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv =(struct batt_drv *)
					power_supply_get_drvdata(psy);
	struct gbms_charging_event *ce_data = &batt_drv->ce_data;
	const bool qual_valid = (batt_drv->ce_qual.last_update -
				batt_drv->ce_qual.first_update) != 0;
	int len = 0;

	mutex_lock(&batt_drv->stats_lock);

	/* this is the current one */
	len += batt_chg_stats_cstr(&buf[len], PAGE_SIZE - len, ce_data, true,
				   aacr_filtered_capacity(batt_drv, ce_data));

	/*
	 * stats are accumulated in ce_data->health_stats, rest_* fields
	 * are set on stats_close()
	 */
	if (batt_drv->chg_health.rest_state != CHG_HEALTH_INACTIVE) {
		const struct gbms_ce_tier_stats *h = &batt_drv->ce_data.health_stats;
		const struct gbms_ce_tier_stats *p = &batt_drv->ce_data.health_pause_stats;
		const long elap_h = h->time_fast + h->time_taper + h->time_other;
		const long elap_p = p->time_fast + p->time_taper + p->time_other;
		const ktime_t now = get_boot_sec();
		int vti;

		vti = batt_chg_health_vti(&batt_drv->chg_health);
		len += scnprintf(&buf[len], PAGE_SIZE - len,
				"\nH: %d %d %ld %ld %lld %lld %d",
				batt_drv->chg_health.rest_state,
				vti, elap_h, elap_p, now,
				batt_drv->chg_health.rest_deadline,
				batt_drv->chg_health.always_on_soc);

		/* NOTE: vtier_idx is -1, can also check elap  */
		if (h->soc_in != -1)
			len += gbms_tier_stats_cstr(&buf[len],
						    PAGE_SIZE - len, h, !!elap_h);
		if (p->soc_in != -1)
			len += gbms_tier_stats_cstr(&buf[len],
						    PAGE_SIZE - len, p, !!elap_p);
	}

	len += scnprintf(&buf[len], PAGE_SIZE - len, "\n");

	/* this was the last one (if present) */
	if (qual_valid) {
		len += batt_chg_qual_stats_cstr(
			&buf[len], PAGE_SIZE - len, &batt_drv->ce_qual, true,
			aacr_filtered_capacity(batt_drv, &batt_drv->ce_qual));
		len += scnprintf(&buf[len], PAGE_SIZE - len, "\n");
	}

	mutex_unlock(&batt_drv->stats_lock);

	return len;
}

static DEVICE_ATTR_RO(charge_details);

/* tier and soc details */
static ssize_t ttf_details_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = (struct batt_drv *)
					power_supply_get_drvdata(psy);
	struct batt_ttf_stats *ttf_stats;
	int len;

	if (!batt_drv->ssoc_state.buck_enabled)
		return -ENODATA;

	ttf_stats = kzalloc(sizeof(*ttf_stats), GFP_KERNEL);
	if (!ttf_stats)
		return -ENOMEM;

	mutex_lock(&batt_drv->stats_lock);
	/* update a private copy of ttf stats */
	ttf_stats_update(ttf_stats_dup(ttf_stats, &batt_drv->ttf_stats),
			 &batt_drv->ce_data, false);
	mutex_unlock(&batt_drv->stats_lock);

	len = ttf_dump_details(buf, PAGE_SIZE, ttf_stats,
			       batt_drv->ce_data.last_soc);
	kfree(ttf_stats);

	return len;
}

static DEVICE_ATTR_RO(ttf_details);

/* house stats */
static ssize_t ttf_stats_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv =(struct batt_drv *)
					power_supply_get_drvdata(psy);
	const int verbose = true;
	int i, len = 0;

	mutex_lock(&batt_drv->stats_lock);

	for (i = 0; i < GBMS_STATS_TIER_COUNT; i++)
		len += ttf_tier_cstr(&buf[len], PAGE_SIZE,
				     &batt_drv->ttf_stats.tier_stats[i]);

	len += scnprintf(&buf[len], PAGE_SIZE - len, "\n");

	if (verbose)
		len += ttf_soc_cstr_combine(&buf[len], PAGE_SIZE - len,
					    &batt_drv->ttf_stats.soc_ref,
					    &batt_drv->ttf_stats.soc_stats);

	mutex_unlock(&batt_drv->stats_lock);

	return len;
}

/* userspace restore the TTF data with this */
static ssize_t ttf_stats_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	int res;
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv =(struct batt_drv *)
					power_supply_get_drvdata(psy);

	if (count < 1)
		return -ENODATA;
	if (!batt_drv->ssoc_state.buck_enabled)
		return -ENODATA;

	mutex_lock(&batt_drv->stats_lock);
	switch (buf[0]) {
	case 'u':
	case 'U': /* force update */
		ttf_stats_update(&batt_drv->ttf_stats, &batt_drv->ce_data,
				 (buf[0] == 'U'));
		break;
	default:
		/* TODO: userspace restore of the data */
		res = ttf_stats_sscan(&batt_drv->ttf_stats, buf, count);
		if (res < 0)
			count = res;
		break;
	}
	mutex_unlock(&batt_drv->stats_lock);

	return count;
}

static DEVICE_ATTR_RW(ttf_stats);

/* ------------------------------------------------------------------------- */

static ssize_t charge_stage_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = (struct batt_drv *)
					power_supply_get_drvdata(psy);
	const char *s = "Inactive";

	mutex_lock(&batt_drv->chg_lock);
	switch (batt_drv->chg_health.rest_state) {
	case CHG_HEALTH_DISABLED:
		s = "Disabled";
		break;
	case CHG_HEALTH_ENABLED:
		s = "Enabled";
		break;
	case CHG_HEALTH_PAUSE:
	case CHG_HEALTH_ACTIVE:
		s = "Active";
		break;
	case CHG_HEALTH_DONE:
		s = "Done";
		break;
	default:
		break;
	}
	mutex_unlock(&batt_drv->chg_lock);

	return scnprintf(buf, PAGE_SIZE, "%s\n", s);
}

static DEVICE_ATTR_RO(charge_stage);

static ssize_t charge_limit_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv =(struct batt_drv *)
					power_supply_get_drvdata(psy);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 batt_drv->chg_health.always_on_soc);
}

/* setting disable (deadline = -1) or replug (deadline == 0) will disable */
static void batt_set_health_charge_limit(struct batt_drv *batt_drv,
					 const int always_on_soc)
{
	enum chg_health_state rest_state;

	mutex_lock(&batt_drv->chg_lock);

	/*
	 * There are interesting overlaps with the AC standard behavior since
	 * the aon limit can be set at any time (and while AC limit is active)
	 * TODO: fully document the state machine
	 */
	rest_state = batt_drv->chg_health.rest_state;

	if (always_on_soc != -1) {
		switch (rest_state) {
		case CHG_HEALTH_DISABLED: /* didn't meet deadline */
		case CHG_HEALTH_INACTIVE: /* deadline was not provided */
			rest_state = CHG_HEALTH_ENABLED;
			break;
		default:
			/* _DONE, _ENABLED, _ACTIVE, _USER_DISABLED */
			break;
		}
	} else if (batt_drv->chg_health.always_on_soc != -1) {

		switch (rest_state) {
		case CHG_HEALTH_ENABLED: /* waiting for always_on_soc */
		case CHG_HEALTH_ACTIVE: /* activated at always_on_soc */
			if (batt_drv->chg_health.rest_deadline > 0)
				rest_state = CHG_HEALTH_ENABLED;
			else
				rest_state = CHG_HEALTH_INACTIVE;
			break;
		default:
			/* _DONE, _DISABLED, _USER_DISABLED */
			break;
		}
	}

	batt_drv->chg_health.always_on_soc = always_on_soc;
	batt_drv->chg_health.rest_state = rest_state;

	mutex_unlock(&batt_drv->chg_lock);
}

static ssize_t charge_limit_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv =(struct batt_drv *)
					power_supply_get_drvdata(psy);
	const int always_on_soc = simple_strtol(buf, NULL, 10);

	/* Always enable AC when SOC is over the trigger point. */
	if (always_on_soc < -1 || always_on_soc > 99)
		return -EINVAL;

	batt_set_health_charge_limit(batt_drv, always_on_soc);
	power_supply_changed(batt_drv->psy);
	return count;
}

static DEVICE_ATTR_RW(charge_limit);

static void batt_init_chg_health(struct batt_drv *batt_drv)
{
	int ret;

	ret = of_property_read_u32(batt_drv->device->of_node,
				   "google,chg-rest-soc",
				   &batt_drv->chg_health.rest_soc);
	if (ret < 0)
		batt_drv->chg_health.rest_soc = -1;

	ret = of_property_read_u32(batt_drv->device->of_node,
				   "google,chg-rest-rate",
				   &batt_drv->chg_health.rest_rate);
	if (ret < 0)
		batt_drv->chg_health.rest_rate = 0;

	ret = of_property_read_u32(batt_drv->device->of_node,
				   "google,chg-rest-rate-before-trigger",
				   &batt_drv->chg_health.rest_rate_before_trigger);
	if (ret < 0)
		batt_drv->chg_health.rest_rate_before_trigger = HEALTH_CHG_RATE_BEFORE_TRIGGER;

	batt_set_health_charge_limit(batt_drv, -1);

	gbms_logbuffer_prlog(batt_drv->bd_log, LOGLEVEL_INFO, 0, LOGLEVEL_DEBUG,
			     "MSC_HEALTH: %s: rest_soc=%d, aon_soc=%d, rest_rate/before=%d/%d",
			     __func__, batt_drv->chg_health.rest_soc,
			     batt_drv->chg_health.always_on_soc,
			     batt_drv->chg_health.rest_rate,
			     batt_drv->chg_health.rest_rate_before_trigger);
}

static ssize_t charge_deadline_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv =(struct batt_drv *)
					power_supply_get_drvdata(psy);
	const struct batt_chg_health *rest = &batt_drv->chg_health;
	const bool aon_enabled = rest->always_on_soc != -1;
	const ktime_t now = get_boot_sec();
	long long deadline = 0;
	ktime_t ttf = 0;
	int ret = 0;

	mutex_lock(&batt_drv->chg_lock);

	/*
	 * = (rest_deadline <= 0) means state is either Inactive or Disabled
	 * = (rest_deadline < now) means state is either Done or Disabled
	 *
	 * State becomes Disabled from Enabled or Active when/if msc_logic()
	 * determines that the device cannot reach full before the deadline.
	 *
	 * UI checks for:
	 *   (stage == 'Active' || stage == 'Enabled') && deadline > 0
	 */
	deadline = batt_drv->chg_health.rest_deadline;

	/* ACA: show time to full when ACA triggered */
	if (aon_enabled && rest->rest_state == CHG_HEALTH_ACTIVE) {
		ret = batt_ttf_estimate(&ttf, batt_drv);
		if (ret < 0)
			pr_debug("unable to get ttf (%d)\n", ret);
		else
			deadline = now + ttf;
	}

	if (deadline > 0 && deadline > now)
		deadline -= now;
	else if (deadline > 0)
		deadline = 0;

	mutex_unlock(&batt_drv->chg_lock);

	/*
	 * deadline < 0 feature disabled. deadline = 0 expired or disabled for
	 * this session, deadline > 0 time to deadline otherwise.
	 */
	return scnprintf(buf, PAGE_SIZE, "%lld\n", (long long)deadline);
}

/* userspace restore the TTF data with this */
static ssize_t charge_deadline_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv =(struct batt_drv *)
					power_supply_get_drvdata(psy);
	ktime_t ttf_with_margin = 0;
	long long deadline_s;
	bool changed;

	/* API works in seconds */
	deadline_s = simple_strtoll(buf, NULL, 10);

	mutex_lock(&batt_drv->chg_lock);
	/* Let deadline < 0 pass to set stats */
	if (!batt_drv->ssoc_state.buck_enabled && deadline_s >= 0) {
		mutex_unlock(&batt_drv->chg_lock);
		return -EINVAL;
	}

	if (batt_ttf_estimate(&ttf_with_margin, batt_drv) < 0)
		ttf_with_margin = LLONG_MAX;
	else if (batt_drv->health_safety_margin > 0)
		ttf_with_margin += batt_drv->health_safety_margin;

	changed = batt_health_set_chg_deadline(&batt_drv->chg_health, (long long)ttf_with_margin,
					       deadline_s);
	mutex_unlock(&batt_drv->chg_lock);

	if (changed) {
		/* charging_policy: vote AC */
		gvotable_cast_long_vote(batt_drv->charging_policy_votable, "MSC_AC",
					CHARGING_POLICY_VOTE_ADAPTIVE_AC,
					batt_drv->chg_health.rest_deadline > 0);

		power_supply_changed(batt_drv->psy);
	}

	gbms_logbuffer_prlog(batt_drv->bd_log, LOGLEVEL_INFO, 0, LOGLEVEL_DEBUG,
			     "MSC_HEALTH: deadline_s=%lld deadline at %lld",
			     deadline_s, batt_drv->chg_health.rest_deadline);

	return count;
}

static DEVICE_ATTR_RW(charge_deadline);

static ssize_t charge_deadline_dryrun_store(struct device *dev,
					    struct device_attribute *attr,
					    const char *buf, size_t count)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv =
		(struct batt_drv *)power_supply_get_drvdata(psy);
	long long deadline_s;

	/* API works in seconds */
	deadline_s = simple_strtoll(buf, NULL, 10);

	mutex_lock(&batt_drv->chg_lock);
	if (!batt_drv->ssoc_state.buck_enabled || deadline_s < 0) {
		mutex_unlock(&batt_drv->chg_lock);
		return -EINVAL;
	}
	batt_drv->chg_health.dry_run_deadline = get_boot_sec() + deadline_s;
	mutex_unlock(&batt_drv->chg_lock);

	return count;
}

static DEVICE_ATTR_WO(charge_deadline_dryrun);

enum batt_ssoc_status {
	BATT_SSOC_STATUS_UNKNOWN = 0,
	BATT_SSOC_STATUS_CONNECTED = 1,
	BATT_SSOC_STATUS_DISCONNECTED = 2,
	BATT_SSOC_STATUS_FULL = 3,
};

static ssize_t ssoc_details_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	struct batt_ssoc_state *ssoc_state = &batt_drv->ssoc_state;
	int len = 0;
	enum batt_ssoc_status status = BATT_SSOC_STATUS_UNKNOWN;
	char buff[UICURVE_BUF_SZ] = { 0 };

	mutex_lock(&batt_drv->chg_lock);

	if (ssoc_state->buck_enabled == 0) {
		status = BATT_SSOC_STATUS_DISCONNECTED;
	} else if (ssoc_state->buck_enabled == 1) {
		if (batt_drv->batt_full)
			status = BATT_SSOC_STATUS_FULL;
		else
			status = BATT_SSOC_STATUS_CONNECTED;
	}

	len = scnprintf(
		buf, sizeof(ssoc_state->ssoc_state_cstr),
		"soc: l=%d%% gdf=%d.%02d uic=%d.%02d rl=%d.%02d\n"
		"curve:%s\n"
		"status: ct=%d rl=%d s=%d\n",
		ssoc_get_capacity(ssoc_state), qnum_toint(ssoc_state->ssoc_gdf),
		qnum_fracdgt(ssoc_state->ssoc_gdf),
		qnum_toint(ssoc_state->ssoc_uic),
		qnum_fracdgt(ssoc_state->ssoc_uic),
		qnum_toint(ssoc_state->ssoc_rl),
		qnum_fracdgt(ssoc_state->ssoc_rl),
		ssoc_uicurve_cstr(buff, sizeof(buff), ssoc_state->ssoc_curve),
		ssoc_state->ssoc_curve_type, ssoc_state->rl_status, status);

	mutex_unlock(&batt_drv->chg_lock);

	return len;
}

static DEVICE_ATTR_RO(ssoc_details);

static ssize_t bd_trickle_enable_show(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 batt_drv->ssoc_state.bd_trickle_enable);
}

static ssize_t bd_trickle_enable_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	int ret = 0, val;

	ret = kstrtoint(buf, 0, &val);
	if (ret < 0)
		return ret;

	batt_drv->ssoc_state.bd_trickle_enable = !!val;

	return count;
}

static DEVICE_ATTR_RW(bd_trickle_enable);

static ssize_t bd_trickle_cnt_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 batt_drv->ssoc_state.bd_trickle_cnt);
}

static ssize_t bd_trickle_cnt_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	int ret = 0, val;

	ret = kstrtoint(buf, 0, &val);
	if (ret < 0)
		return ret;

	batt_drv->ssoc_state.bd_trickle_cnt = val;

	return count;
}

static DEVICE_ATTR_RW(bd_trickle_cnt);

static ssize_t bd_trickle_recharge_soc_show(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 batt_drv->ssoc_state.bd_trickle_recharge_soc);
}

#define BD_RL_SOC_FULL		100
#define BD_RL_SOC_LOW		50
static ssize_t bd_trickle_recharge_soc_store(struct device *dev,
					     struct device_attribute *attr,
					     const char *buf, size_t count)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	int ret = 0, val;

	ret = kstrtoint(buf, 0, &val);
	if (ret < 0)
		return ret;

	if ((val >= BD_RL_SOC_FULL) || (val < BD_RL_SOC_LOW))
		return count;

	batt_drv->ssoc_state.bd_trickle_recharge_soc = val;

	return count;
}

static DEVICE_ATTR_RW(bd_trickle_recharge_soc);

static ssize_t bd_trickle_dry_run_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 batt_drv->ssoc_state.bd_trickle_dry_run);
}

static ssize_t bd_trickle_dry_run_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	int ret = 0, val;

	ret = kstrtoint(buf, 0, &val);
	if (ret < 0)
		return ret;

	batt_drv->ssoc_state.bd_trickle_dry_run = val ? true : false;

	return count;
}

static DEVICE_ATTR_RW(bd_trickle_dry_run);

static ssize_t bd_trickle_reset_sec_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 batt_drv->ssoc_state.bd_trickle_reset_sec);
}

static ssize_t bd_trickle_reset_sec_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	unsigned int val;
	int ret = 0;

	ret = kstrtouint(buf, 0, &val);
	if (ret < 0)
		return ret;

	batt_drv->ssoc_state.bd_trickle_reset_sec = val;

	return count;
}

static DEVICE_ATTR_RW(bd_trickle_reset_sec);

static ssize_t bd_clear_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	int ret = 0, val = 0;

	ret = kstrtoint(buf, 0, &val);
	if (ret < 0)
		return ret;

	if (val)
		bd_trickle_reset(&batt_drv->ssoc_state, &batt_drv->ce_data);

	return count;
}

static DEVICE_ATTR_WO(bd_clear);

static ssize_t time_to_ac_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv =(struct batt_drv *)
					power_supply_get_drvdata(psy);
	const int soc = CHG_HEALTH_REST_SOC(&batt_drv->chg_health);
	qnum_t soc_raw = ssoc_get_capacity_raw(&batt_drv->ssoc_state);
	qnum_t soc_health = qnum_fromint(soc);
	ktime_t estimate;
	int rc;

	rc = ttf_soc_estimate(&estimate, &batt_drv->ttf_stats,
			      &batt_drv->ce_data, soc_raw,
			      soc_health - qnum_rconst(SOC_ROUND_BASE));
	if (rc < 0)
		estimate = -1;

	if (estimate == -1)
		return -ERANGE;

	return scnprintf(buf, PAGE_SIZE, "%lld\n", (long long)estimate);
}

static DEVICE_ATTR_RO(time_to_ac);

static ssize_t ac_soc_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv =(struct batt_drv *)
					power_supply_get_drvdata(psy);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 CHG_HEALTH_REST_SOC(&batt_drv->chg_health));
}

static DEVICE_ATTR_RO(ac_soc);

static ssize_t charge_to_limit_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count) {
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	bool is_enable = batt_drv->chg_health.rest_rate == 0 &&
			 batt_drv->chg_health.always_on_soc > 0;
	int charge_to_limit;
	int ret = 0;

	ret = kstrtoint(buf, 0, &charge_to_limit);
	if (ret < 0)
		return ret;

	/* disable charge to limit, restore original setting */
	if (is_enable && charge_to_limit == 0) {
		batt_init_chg_health(batt_drv);
		return count;
	}

	/* invalid value, do nothing */
	if (charge_to_limit <= 0 || charge_to_limit > 99)
		return count;


	if (batt_drv->chg_health.rest_rate == 0 &&
	    batt_drv->chg_health.always_on_soc == charge_to_limit)
	    	return count;

	gbms_logbuffer_prlog(batt_drv->ttf_stats.ttf_log, LOGLEVEL_INFO, 0, LOGLEVEL_DEBUG,
			     "MSC_HEALTH: %s: set aon_soc=%d->%d", __func__,
			     batt_drv->chg_health.always_on_soc, charge_to_limit);

	/* will set cc_max = 0 */
	batt_drv->chg_health.rest_rate = 0;
	batt_set_health_charge_limit(batt_drv, charge_to_limit);

	return count;
}

static ssize_t charge_to_limit_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	const bool is_enable = batt_charge_to_limit_enable(&batt_drv->chg_health);
	int result = 0;

	if (is_enable)
		result = batt_drv->chg_health.always_on_soc;
	else
		result = 0;

	return sysfs_emit(buf, "%d\n", result);
}

static DEVICE_ATTR_RW(charge_to_limit);

static ssize_t charger_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);

	return scnprintf(buf, PAGE_SIZE, "0x%llx\n", batt_drv->chg_state.v);
}

static DEVICE_ATTR_RO(charger_state);


static ssize_t charge_type_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);

	return scnprintf(buf, PAGE_SIZE, "%d\n", batt_drv->chg_state.f.chg_type);
}

static DEVICE_ATTR_RO(charge_type);

static ssize_t constant_charge_current_show(struct device *dev,
					    struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);

	return scnprintf(buf, PAGE_SIZE, "%d\n", batt_drv->cc_max);
}

static DEVICE_ATTR_RO(constant_charge_current);


static ssize_t constant_charge_voltage_show(struct device *dev,
					    struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);

	return scnprintf(buf, PAGE_SIZE, "%d\n", batt_drv->fv_uv);
}

static DEVICE_ATTR_RO(constant_charge_voltage);

static ssize_t health_safety_margin_show(struct device *dev,
					 struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 batt_drv->health_safety_margin);
}

static ssize_t health_safety_margin_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	int ret = 0, val;

	ret = kstrtoint(buf, 0, &val);
	if (ret < 0)
		return ret;

	/*
	 * less than 0 is not accaptable: we will not reach full in time.
	 * set to 0 to disable PAUSE but keep AC charge
	 */
	if (val < 0)
		val = 0;

	batt_drv->health_safety_margin = val;

	return count;
}

static DEVICE_ATTR_RW(health_safety_margin);

/* BPST ------------------------------------------------------------------- */

static ssize_t bpst_reset_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	struct batt_bpst *bpst_state = &batt_drv->bpst_state;
	int ret = 0, val = 0;

	ret = kstrtoint(buf, 0, &val);
	if (ret < 0)
		return ret;

	if (val) {
		ret = batt_bpst_reset(bpst_state);
		if (ret < 0)
			pr_err("%s: MSC_BPST: Cannot reset GBMS_TAG_BPST (%d)\n", __func__, ret);
	}

	return count;
}

static DEVICE_ATTR_WO(bpst_reset);

static ssize_t show_bpst_detect_disable(struct device *dev,
				        struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 batt_drv->bpst_state.bpst_detect_disable);
}

static ssize_t set_bpst_detect_disable(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	int ret = 0, val;

	ret = kstrtoint(buf, 0, &val);
	if (ret < 0)
		return ret;

	mutex_lock(&batt_drv->bpst_state.lock);
	batt_drv->bpst_state.bpst_detect_disable = !!val;
	mutex_unlock(&batt_drv->bpst_state.lock);
	if (batt_drv->psy)
		power_supply_changed(batt_drv->psy);

	return count;
}

static DEVICE_ATTR(bpst_detect_disable, 0660,
		   show_bpst_detect_disable, set_bpst_detect_disable);

/* AACR ------------------------------------------------------------------- */

static ssize_t aacr_state_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count) {
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	enum batt_aacr_state state = batt_drv->aacr_state;
	int val, algo, ret = 0;

	ret = kstrtoint(buf, 0, &val);
	if (ret < 0)
		return ret;

	if (val < BATT_AACR_UNKNOWN)
		return -ERANGE;

	mutex_lock(&batt_drv->aacp_state_lock);

	switch (val) {
	case BATT_AACR_UNKNOWN:
		state = BATT_AACR_UNKNOWN;
		break;
	case BATT_AACR_DISABLED:
		state = BATT_AACR_DISABLED;
		break;
	/* configuration needs to be valid */
	case BATT_AACR_ENABLED:
		state = BATT_AACR_ENABLED;
		algo = BATT_AACR_ALGO_DEFAULT;
		break;

	/* change of algo and state if unknown */
	case BATT_AACR_ALGO_LOW_B:
		algo = BATT_AACR_ALGO_LOW_B;
		if (batt_drv->aacr_state == BATT_AACR_UNKNOWN)
			state = BATT_AACR_DISABLED;
		break;
	case BATT_AACR_ALGO_UP_B:
		algo = BATT_AACR_ALGO_UP_B;
		if (batt_drv->aacr_state == BATT_AACR_UNKNOWN)
			state = BATT_AACR_DISABLED;
		break;
	case BATT_AACR_ALGO_REFERENCE_CAP:
		algo = BATT_AACR_ALGO_REFERENCE_CAP;
		if (batt_drv->aacr_state == BATT_AACR_UNKNOWN)
			state = BATT_AACR_DISABLED;
		break;
	default:
		return -ERANGE;
	}

	if (batt_drv->aacr_state == state && batt_drv->aacr_algo == algo) {
		mutex_unlock(&batt_drv->aacp_state_lock);
		return count;
	}

	pr_info("aacr_state: %d -> %d, aacr_algo: %d -> %d\n",
		batt_drv->aacr_state, state, batt_drv->aacr_algo, algo);

	batt_drv->aacr_state = state;
	batt_drv->aacr_algo = algo;

	aacr_update_chg_table(batt_drv);
	mutex_unlock(&batt_drv->aacp_state_lock);

	return count;
}

static ssize_t aacr_state_show(struct device *dev,  struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
		aact_enabled(batt_drv->aacr_state, batt_drv->aacp_opt_out));
}

static DEVICE_ATTR_RW(aacr_state);

static ssize_t aacr_config_show(struct device *dev,  struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	struct device_node *node = batt_drv->device->of_node;
	u32 aaxx_config;
	int ret;

	ret = of_property_read_u32(node, "google,aacr-config",  &aaxx_config);
	if (ret < 0)
		aaxx_config = batt_drv->chg_profile.aacr_nb_limits ?
			BATT_AACR_DISABLED : BATT_AACR_UNKNOWN;;


	return scnprintf(buf, PAGE_SIZE, "%d\n", aaxx_config);
}

static DEVICE_ATTR_RO(aacr_config);

static ssize_t aacr_cycle_grace_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	int value, ret = 0;

	ret = kstrtoint(buf, 0, &value);
	if (ret < 0)
		return ret;

	if (value < 0)
		return -ERANGE;

	mutex_lock(&batt_drv->aacp_state_lock);
	batt_drv->aacr_cycle_grace = value;
	mutex_unlock(&batt_drv->aacp_state_lock);

	return count;
}

static ssize_t aacr_cycle_grace_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);

	return scnprintf(buf, PAGE_SIZE, "%d\n", batt_drv->aacr_cycle_grace);
}

static DEVICE_ATTR_RW(aacr_cycle_grace);

static ssize_t aacr_cycle_max_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	int value, ret = 0;

	ret = kstrtoint(buf, 0, &value);
	if (ret < 0)
		return ret;

	if (value < 0 || value > 3000) /* unexpected cycles */
		return -ERANGE;

	mutex_lock(&batt_drv->aacp_state_lock);
	batt_drv->aacr_cycle_max = value;
	mutex_unlock(&batt_drv->aacp_state_lock);

	return count;
}

static ssize_t aacr_cycle_max_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);

	return scnprintf(buf, PAGE_SIZE, "%d\n", batt_drv->aacr_cycle_max);
}

static DEVICE_ATTR_RW(aacr_cycle_max);

static ssize_t aacr_algo_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);

	return scnprintf(buf, PAGE_SIZE, "%d\n", batt_drv->aacr_algo);
}

static DEVICE_ATTR_RO(aacr_algo);

static ssize_t aacr_min_capacity_rate_store(struct device *dev,
					    struct device_attribute *attr,
					    const char *buf, size_t count)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	int rate, ret = 0;

	ret = kstrtoint(buf, 0, &rate);
	if (ret < 0 || rate > 100 || rate <= 0)
		return ret;

	mutex_lock(&batt_drv->aacp_state_lock);
	batt_drv->aacr_min_capacity_rate = rate;
	mutex_unlock(&batt_drv->aacp_state_lock);

	return count;
}

static ssize_t aacr_min_capacity_rate_show(struct device *dev,
					   struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);

	return scnprintf(buf, PAGE_SIZE, "%d\n", batt_drv->aacr_min_capacity_rate);
}

static DEVICE_ATTR_RW(aacr_min_capacity_rate);

static ssize_t aacr_cliff_capacity_rate_store(struct device *dev,
					      struct device_attribute *attr,
					      const char *buf, size_t count)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	int rate, ret = 0;

	ret = kstrtoint(buf, 0, &rate);
	if (ret < 0)
		return ret;

	if(rate > 100 || rate <= 0)
		return -ERANGE;

	mutex_lock(&batt_drv->aacp_state_lock);
	batt_drv->aacr_cliff_capacity_rate = rate;
	mutex_unlock(&batt_drv->aacp_state_lock);

	return count;
}

static ssize_t aacr_cliff_capacity_rate_show(struct device *dev,
					     struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);

	return scnprintf(buf, PAGE_SIZE, "%d\n", batt_drv->aacr_cliff_capacity_rate);
}

static DEVICE_ATTR_RW(aacr_cliff_capacity_rate);

static ssize_t aacr_profile_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	struct gbms_chg_profile *profile = &batt_drv->chg_profile;
	u32 cc[GBMS_AACR_DATA_MAX] = { 0 };
	u32 fd[GBMS_AACR_DATA_MAX] = { 0 };
	int cnt = 0, batt_id;

	cnt = sscanf(buf, "%d,%u:%u,%u:%u,%u:%u,%u:%u,%u:%u,%u:%u,%u:%u,%u:%u,%u:%u,%u:%u",
		     &batt_id, &cc[0], &fd[0], &cc[1], &fd[1], &cc[2], &fd[2], &cc[3], &fd[3],
		     &cc[4], &fd[4], &cc[5], &fd[5], &cc[6], &fd[6], &cc[7], &fd[7],
		     &cc[8], &fd[8], &cc[9], &fd[9]);

	/* The number entered must be an odd number (include batt_id) */
	if (cnt % 2 == 0 || cnt < 3)
		return -ERANGE;

	/* validity check */
	for (cnt = 0; cnt < GBMS_AACR_DATA_MAX - 1; cnt++) {
		if (cc[cnt + 1] == 0)
			break;

		if (cc[cnt] > cc[cnt + 1] || fd[cnt] > fd[cnt + 1])
			return -ERANGE;
	}

	/* 0 means force setting the profile */
	if (batt_id == batt_drv->batt_id || batt_id == 0) {
		mutex_lock(&batt_drv->aacp_state_lock);
		memcpy(&profile->aacr_reference_cycles, cc, sizeof(cc));
		memcpy(&profile->aacr_reference_fade10, fd, sizeof(fd));
		profile->aacr_nb_limits = (u32)(cnt + 1);
		mutex_unlock(&batt_drv->aacp_state_lock);
	}

	return count;
}

static ssize_t aacr_profile_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	struct gbms_chg_profile *profile = &batt_drv->chg_profile;
	ssize_t cnt = 0;
	int i;

	if (profile->aacr_nb_limits == 0)
		return cnt;

	cnt += sysfs_emit_at(buf, cnt, "%d", batt_drv->batt_id);

	for (i = 0; i < profile->aacr_nb_limits; i++)
		cnt += sysfs_emit_at(buf, cnt, ",%u:%u",
				     profile->aacr_reference_cycles[i],
				     profile->aacr_reference_fade10[i]);

	cnt += sysfs_emit_at(buf, cnt, "\n");

	return cnt;
}

static DEVICE_ATTR_RW(aacr_profile);

/* AAFV ------------------------------------------------------------------- */

static ssize_t aafv_state_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count) {
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	int val, ret = 0;

	ret = kstrtoint(buf, 0, &val);
	if (ret < 0)
		return ret;

	mutex_lock(&batt_drv->aacp_state_lock);

	dev_info(batt_drv->device, "AAFV: aafv_state: %d -> %d\n", batt_drv->aafv_state, val);

	if (batt_drv->aafv_state == val)
		goto done;

	batt_drv->aafv_state = val;
	/* the offset is applied immediately */
	aafv_update_offset(batt_drv);

done:
	mutex_unlock(&batt_drv->aacp_state_lock);

	return count;
}

static ssize_t aafv_state_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
		aafv_enabled(batt_drv->aafv_state, batt_drv->aacp_opt_out));
}

static DEVICE_ATTR_RW(aafv_state);

static ssize_t aafv_config_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	struct device_node *node = batt_drv->device->of_node;
	u32 aaxx_config;
	int ret;

	/* NOTE: see batt_init_aafv_profile */
	ret = of_property_read_u32(node, "google,aafv-config",  &aaxx_config);
	if (ret < 0)
		aaxx_config = batt_drv->chg_profile.aafv_nb_limits ?
			BATT_AAFV_DISABLED : BATT_AAFV_UNKNOWN;;

	return scnprintf(buf, PAGE_SIZE, "%d\n", aaxx_config);
}

static DEVICE_ATTR_RO(aafv_config);


static ssize_t aafv_apply_max_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	int value, ret = 0;

	ret = kstrtoint(buf, 0, &value);
	if (ret < 0)
		return ret;

	mutex_lock(&batt_drv->aacp_state_lock);
	if (value >= 0)
		batt_drv->aafv_apply_max = value;
	mutex_unlock(&batt_drv->aacp_state_lock);

	return count;
}

static ssize_t aafv_apply_max_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);

	return scnprintf(buf, PAGE_SIZE, "%d\n", batt_drv->aafv_apply_max);
}

static DEVICE_ATTR_RW(aafv_apply_max);

static ssize_t aafv_max_offset_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	int value, ret = 0;

	ret = kstrtoint(buf, 0, &value);
	if (ret < 0)
		return ret;

	mutex_lock(&batt_drv->aacp_state_lock);
	if (value >= 0)
		batt_drv->aafv_max_offset = value;
	mutex_unlock(&batt_drv->aacp_state_lock);

	return count;
}

static ssize_t aafv_max_offset_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);

	return scnprintf(buf, PAGE_SIZE, "%d\n", batt_drv->aafv_max_offset);
}

static DEVICE_ATTR_RW(aafv_max_offset);

static ssize_t aafv_cliff_cycle_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	int value, ret = 0;

	ret = kstrtoint(buf, 0, &value);
	if (ret < 0)
		return ret;

	mutex_lock(&batt_drv->aacp_state_lock);
	if (value >= 0)
		batt_drv->aafv_cliff_cycle = value;
	mutex_unlock(&batt_drv->aacp_state_lock);

	return count;
}

static ssize_t aafv_cliff_cycle_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);

	return scnprintf(buf, PAGE_SIZE, "%d\n", batt_drv->aafv_cliff_cycle);
}

static DEVICE_ATTR_RW(aafv_cliff_cycle);

static ssize_t aafv_cliff_offset_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	struct gbms_chg_profile *profile = &batt_drv->chg_profile;
	int value, ret = 0;
	bool is_valid;

	ret = kstrtoint(buf, 0, &value);
	if (ret < 0)
		return ret;

	mutex_lock(&batt_drv->aacp_state_lock);
	is_valid = gbms_aafv_offset_is_valid(profile, value, profile->aafv_nb_limits);
	if (is_valid)
		batt_drv->aafv_cliff_offset = value;
	mutex_unlock(&batt_drv->aacp_state_lock);

	return count;
}

static ssize_t aafv_cliff_offset_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);

	return scnprintf(buf, PAGE_SIZE, "%d\n", batt_drv->aafv_cliff_offset);
}

static DEVICE_ATTR_RW(aafv_cliff_offset);

static ssize_t aafv_profile_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	struct gbms_chg_profile *profile = &batt_drv->chg_profile;
	u32 cc[GBMS_AAFV_DATA_MAX] = { 0 };
	u32 of[GBMS_AAFV_DATA_MAX] = { 0 };
	int cnt = 0, batt_id, nb_limits, idx;
	bool is_valid;

	/* This profile might change due to AACR and AACT */
	cnt = sscanf(buf, "%d,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u, \
		     %u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u", &batt_id,
		     &cc[0], &of[0], &cc[1], &of[1], &cc[2], &of[2], &cc[3], &of[3],
		     &cc[4], &of[4], &cc[5], &of[5], &cc[6], &of[6], &cc[7], &of[7],
		     &cc[8], &of[8], &cc[9], &of[9], &cc[10], &of[10], &cc[11], &of[11],
		     &cc[12], &of[12], &cc[13], &of[13], &cc[14], &of[14], &cc[15], &of[15]);

	/* The number entered must be an odd number (include batt_id) */
	if (cnt % 2 == 0 || cnt < 3)
		return -ERANGE;

	/* Check if cc[] and of[] are sorted from small to large */
	nb_limits = cnt / 2;
	if (nb_limits >= 2)
		for (idx = 1; idx < nb_limits; idx++)
			if (cc[idx] < cc[idx - 1] || of[idx] < of[idx - 1])
				goto done;

	is_valid = gbms_aafv_offset_is_valid(profile, of[nb_limits - 1], (u32)nb_limits);

	/* support id 0 as a common profile for all batteries */
	if ((batt_id == 0 || batt_id == batt_drv->batt_id) && is_valid) {
		mutex_lock(&batt_drv->aacp_state_lock);
		memcpy(&profile->aafv_cycles, cc, sizeof(cc));
		memcpy(&profile->aafv_offsets, of, sizeof(of));
		profile->aafv_nb_limits = (u32)nb_limits;
		mutex_unlock(&batt_drv->aacp_state_lock);
	}

done:
	return count;
}

static ssize_t aafv_profile_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	struct gbms_chg_profile *profile = &batt_drv->chg_profile;
	ssize_t count = 0;
	int i;

	if (profile->aafv_nb_limits == 0)
		return count;

	count += sysfs_emit_at(buf, count, "%d ", batt_drv->batt_id);

	for (i = 0; i < profile->aafv_nb_limits ; i++) {
		const int cycle = profile->aafv_cycles[i];
		const int offset = profile->aafv_offsets[i];

		if (i == profile->aafv_nb_limits - 1)
			count += sysfs_emit_at(buf, count, "<%u>:<%u>\n", cycle, offset);
		else
			count += sysfs_emit_at(buf, count, "<%u>:<%u>,", cycle, offset);
	}

	return count;
}

static DEVICE_ATTR_RW(aafv_profile);


static ssize_t aafv_offset_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);

	return scnprintf(buf, PAGE_SIZE, "%d\n", batt_drv->chg_profile.aafv_offset);
}

static DEVICE_ATTR_RO(aafv_offset);

/* AACT ------------------------------------------------------------------- */

static ssize_t aact_state_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	int val, ret = 0;

	ret = kstrtoint(buf, 0, &val);
	if (ret < 0)
		return ret;

	mutex_lock(&batt_drv->aacp_state_lock);

	if (batt_drv->aact_state == val)
		goto done;

	dev_info(batt_drv->device, "AACT: aact_state: %d -> %d\n", batt_drv->aact_state, val);
	batt_drv->aact_state = val;

	ret = aact_update_chg_table(batt_drv);
	if (ret < 0) {
		struct gbms_chg_profile *profile = &batt_drv->chg_profile;
		struct device_node *node = batt_drv->device->of_node;
		int err;

		/* reset AACT */
		aact_reset(profile);

		/* set state to unknown and init default charge table */
		batt_drv->aact_state = BATT_AACT_UNKNOWN;
		err = gbms_init_chg_profile(profile, node);
		if (err == 0)
			gbms_init_chg_table(profile, node, aacr_get_capacity_locked(batt_drv));

		mutex_unlock(&batt_drv->aacp_state_lock);
		return ret;
	}

done:
	mutex_unlock(&batt_drv->aacp_state_lock);
	return count;
}

static ssize_t aact_state_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			aact_enabled(batt_drv->aact_state, batt_drv->aacp_opt_out));
}

static DEVICE_ATTR_RW(aact_state);

static ssize_t aact_config_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	struct device_node *node = batt_drv->device->of_node;
	u32 aaxx_config;
	int ret;

	/* FIXME: b/403865140 split state from configuration */
	ret = of_property_read_u32(node, "google,aact-config",  &aaxx_config);
	if (ret < 0)
		aaxx_config = BATT_AACT_UNKNOWN;

	return scnprintf(buf, PAGE_SIZE, "%d\n", aaxx_config);
}

static DEVICE_ATTR_RO(aact_config);

static int aact_load_profile(struct batt_drv *batt_drv)
{
	struct gbms_chg_profile *profile = &batt_drv->chg_profile;
	struct device_node *node = batt_drv->device->of_node;

	/* No need to reload if aact profile exist */
	if (profile->aact_cccm_limits)
		return 0;

	return gbms_init_aact_profile(profile, node);
}

static ssize_t aact_cv_limits_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	struct gbms_chg_profile *profile = &batt_drv->chg_profile;
	u32 tmp[GBMS_CHG_VOLT_NB_LIMITS_MAX] = { 0 };
	int cnt = 0, ret;

	/* can only be updated when AACT is disabled */
	if (batt_drv->aact_state != BATT_AACT_DISABLED)
		return count;

	/* init aact profile */
	ret = aact_load_profile(batt_drv);
	if (ret < 0)
		return ret;

	cnt = sscanf(buf, "%u,%u,%u,%u,%u,%u,%u,%u,%u,%u",
		     &tmp[0], &tmp[1], &tmp[2], &tmp[3], &tmp[4], &tmp[5],
		     &tmp[6], &tmp[7], &tmp[8], &tmp[9]);
	memcpy(profile->aact_volt_limits, tmp, sizeof(tmp));
	profile->aact_volt_nb_limits = (u32)cnt;

	gbms_logbuffer_prlog(batt_drv->bd_log, LOGLEVEL_INFO, 0, LOGLEVEL_INFO,
			     "AACT: update aact_cv_limits, cnt=%d", cnt);

	return count;
}

static ssize_t aact_cv_limits_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	struct gbms_chg_profile *profile = &batt_drv->chg_profile;
	ssize_t count = 0;
	int ret, i;

	if (batt_drv->aact_state < 0)
		return count;

	/* init aact profile */
	ret = aact_load_profile(batt_drv);
	if (ret < 0)
		return ret;

	for (i = 0; i < profile->aact_volt_nb_limits ; i++) {
		const int cccm_limit = profile->aact_volt_limits[i];

		if (i == profile->aact_volt_nb_limits - 1)
			count += sysfs_emit_at(buf, count, "%u\n", cccm_limit);
		else
			count += sysfs_emit_at(buf, count, "%u, ", cccm_limit);
	}

	return count;
}

static DEVICE_ATTR_RW(aact_cv_limits);

static ssize_t aact_temp_limits_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	struct gbms_chg_profile *profile = &batt_drv->chg_profile;
	u32 tmp[GBMS_CHG_TEMP_NB_LIMITS_MAX] = { 0 };
	int cnt = 0, ret;

	/* can only be updated when AACT is disabled */
	if (batt_drv->aact_state != BATT_AACT_DISABLED)
		return count;

	/* init aact profile */
	ret = aact_load_profile(batt_drv);
	if (ret < 0)
		return ret;

	cnt = sscanf(buf, "%u,%u,%u,%u,%u,%u,%u,%u,%u,%u",
		     &tmp[0], &tmp[1], &tmp[2], &tmp[3], &tmp[4], &tmp[5],
		     &tmp[6], &tmp[7], &tmp[8], &tmp[9]);
	memcpy(profile->aact_temp_limits, tmp, sizeof(tmp));
	profile->aact_temp_nb_limits = (u32)cnt;

	gbms_logbuffer_prlog(batt_drv->bd_log, LOGLEVEL_INFO, 0, LOGLEVEL_INFO,
			     "AACT: update aact_temp_limits, cnt=%d", cnt);

	return count;
}

static ssize_t aact_temp_limits_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	struct gbms_chg_profile *profile = &batt_drv->chg_profile;
	ssize_t count = 0;
	int ret, i;

	if (batt_drv->aact_state < 0)
		return count;

	/* init aact profile */
	ret = aact_load_profile(batt_drv);
	if (ret < 0)
		return ret;

	for (i = 0; i < profile->aact_temp_nb_limits ; i++) {
		const int cccm_limit = profile->aact_temp_limits[i];

		if (i == profile->aact_temp_nb_limits - 1)
			count += sysfs_emit_at(buf, count, "%u\n", cccm_limit);
		else
			count += sysfs_emit_at(buf, count, "%u, ", cccm_limit);
	}

	return count;
}

static DEVICE_ATTR_RW(aact_temp_limits);

static ssize_t aact_chg_ecc_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	struct gbms_chg_profile *profile = &batt_drv->chg_profile;
	u32 tmp[GBMS_AACT_NB_LIMITS_MAX] = { 0 };
	int cnt = 0, ret;

	/* can only be updated when AACT is disabled */
	if (batt_drv->aact_state != BATT_AACT_DISABLED)
		return count;

	/* init aact profile */
	ret = aact_load_profile(batt_drv);
	if (ret < 0)
		return ret;

	cnt = sscanf(buf, "%u,%u,%u,%u,%u,%u,%u,%u,%u,%u",
		     &tmp[0], &tmp[1], &tmp[2], &tmp[3], &tmp[4], &tmp[5],
		     &tmp[6], &tmp[7], &tmp[8], &tmp[9]);
	memcpy(profile->aact_limits, tmp, sizeof(tmp));
	profile->aact_nb_limits = (u32)cnt;

	gbms_logbuffer_prlog(batt_drv->bd_log, LOGLEVEL_INFO, 0, LOGLEVEL_INFO,
			     "AACT: update aact_chg_ecc, cnt=%d", cnt);

	return count;
}

static ssize_t aact_chg_ecc_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	struct gbms_chg_profile *profile = &batt_drv->chg_profile;
	ssize_t count = 0;
	int ret, i;

	if (batt_drv->aact_state < 0)
		return count;

	/* init aact profile */
	ret = aact_load_profile(batt_drv);
	if (ret < 0)
		return ret;

	for (i = 0; i < profile->aact_nb_limits ; i++) {
		const int cccm_limit = profile->aact_limits[i];

		if (i == profile->aact_nb_limits - 1)
			count += sysfs_emit_at(buf, count, "%u\n", cccm_limit);
		else
			count += sysfs_emit_at(buf, count, "%u, ", cccm_limit);
	}

	return count;
}

static DEVICE_ATTR_RW(aact_chg_ecc);

static ssize_t aact_profile_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	struct gbms_chg_profile *profile = &batt_drv->chg_profile;
	u32 pf[GBMS_AACT_PROFILE_MAX] = { 0 };
	int cnt = 0, ret;
	u32 cccm_array_size;

	/* can only be updated when AACT is disabled */
	if (batt_drv->aact_state != BATT_AACT_DISABLED)
		return count;

	/* init aact profile */
	ret = aact_load_profile(batt_drv);
	if (ret < 0)
		return ret;

	cccm_array_size = (profile->aact_temp_nb_limits - 1)
			  * profile->aact_volt_nb_limits
			  * profile->aact_nb_limits;

	cnt = sscanf(buf, "%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u, \
		     %u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u, \
		     %u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u, \
		     %u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u, \
		     %u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u, \
		     %u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u, \
		     %u,%u,%u,%u",
		     &pf[0], &pf[1], &pf[2], &pf[3], &pf[4], &pf[5], &pf[6], &pf[7],
		     &pf[8], &pf[9], &pf[10], &pf[11], &pf[12], &pf[13], &pf[14], &pf[15],
		     &pf[16], &pf[17], &pf[18], &pf[19], &pf[20], &pf[21], &pf[22], &pf[23],
		     &pf[24], &pf[25], &pf[26], &pf[27], &pf[28], &pf[29], &pf[30], &pf[31],
		     &pf[32], &pf[33], &pf[34], &pf[35], &pf[36], &pf[37], &pf[38], &pf[39],
		     &pf[40], &pf[41], &pf[42], &pf[43], &pf[44], &pf[45], &pf[46], &pf[47],
		     &pf[48], &pf[49], &pf[50], &pf[51], &pf[52], &pf[53], &pf[54], &pf[55],
		     &pf[56], &pf[57], &pf[58], &pf[59], &pf[60], &pf[61], &pf[62], &pf[63],
		     &pf[64], &pf[65], &pf[66], &pf[67], &pf[68], &pf[69], &pf[70], &pf[71],
		     &pf[72], &pf[73], &pf[74], &pf[75], &pf[76], &pf[77], &pf[78], &pf[79],
		     &pf[80], &pf[81], &pf[82], &pf[83], &pf[84], &pf[85], &pf[86], &pf[87],
		     &pf[88], &pf[89], &pf[90], &pf[91], &pf[92], &pf[93], &pf[94], &pf[95],
		     &pf[96], &pf[97], &pf[98], &pf[99]);

	if (cnt != cccm_array_size)
		return -ERANGE;

	memcpy(profile->aact_cccm_limits, pf, sizeof(pf));
	profile->aact_update_profile = true;

	gbms_logbuffer_prlog(batt_drv->bd_log, LOGLEVEL_INFO, 0, LOGLEVEL_INFO,
			     "AACT: update aact_profile, cnt=%d", cnt);

	return count;
}

static ssize_t aact_profile_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	struct gbms_chg_profile *profile = &batt_drv->chg_profile;
	ssize_t count = 0;
	u32 cccm_array_size;
	int ret, i;

	if (batt_drv->aact_state < 0)
		return count;

	/* init aact profile */
	ret = aact_load_profile(batt_drv);
	if (ret < 0)
		return ret;

	cccm_array_size = (profile->aact_temp_nb_limits - 1)
			  * profile->aact_volt_nb_limits
			  * profile->aact_nb_limits;

	for (i = 0; i < cccm_array_size ; i++) {
		const int cccm_limit = profile->aact_cccm_limits[i];

		if (i == cccm_array_size - 1)
			count += sysfs_emit_at(buf, count, "%u\n", cccm_limit);
		else
			count += sysfs_emit_at(buf, count, "%u, ", cccm_limit);
	}

	return count;
}

static DEVICE_ATTR_RW(aact_profile);

/* AACP ------------------------------------------------------------------- */

static ssize_t aacp_version_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	int value, ret = 0;

	ret = kstrtoint(buf, 0, &value);
	if (ret < 0)
		return ret;

	if (value >= 0)
		batt_drv->aacp_version = value;

	return count;
}

static ssize_t aacp_version_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);

	return scnprintf(buf, PAGE_SIZE, "%d\n", batt_drv->aacp_version);
}

static DEVICE_ATTR_RW(aacp_version);

static ssize_t aacp_opt_out_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	int value, ret = 0;

	ret = kstrtoint(buf, 0, &value);
	if (ret < 0)
		return ret;

	if (value < 0 || value > 1)
		return count;

	mutex_lock(&batt_drv->aacp_state_lock);

	batt_drv->aacp_opt_out = value;
	/* offset is applied immediately */
	aafv_update_offset(batt_drv);
	/* cutoff on next connect */
	aacp_update_opt_out_cutoff(batt_drv);

	mutex_unlock(&batt_drv->aacp_state_lock);

	return count;
}

static ssize_t aacp_opt_out_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);

	return scnprintf(buf, PAGE_SIZE, "%d\n", batt_drv->aacp_opt_out);
}

static DEVICE_ATTR_RW(aacp_opt_out);

/* AACC ------------------------------------------------------------------- */

static ssize_t aacc_show(struct device *dev,
			 struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);

	return scnprintf(buf, PAGE_SIZE, "%d\n", batt_drv->aacc);
}

static DEVICE_ATTR_RO(aacc);

/* Swelling  --------------------------------------------------------------- */

static ssize_t swelling_data_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	struct swelling_data *sd = &batt_drv->sd;
	int len = 0, i;

	len += scnprintf(&buf[len], PAGE_SIZE - len,
			 "temp/soc\tcharge(s)\tdischarge(s)\n");
	for (i = 0; i < BATT_TEMP_RECORD_THR ; i++) {
		len += scnprintf(&buf[len], PAGE_SIZE - len,
				 "%d/%d\t%llu\t%llu\n",
				 sd->temp_thr[i]/10, sd->soc_thr[i],
				 sd->chg[i], sd->dischg[i]);
	}

	return len;
}

static DEVICE_ATTR_RO(swelling_data);

/* BHI --------------------------------------------------------------------- */

static ssize_t health_index_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 BHI_ROUND_INDEX(batt_drv->health_data.bhi_index));
}

static DEVICE_ATTR_RO(health_index);

static ssize_t health_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);

	return scnprintf(buf, PAGE_SIZE, "%d\n", batt_drv->health_data.bhi_status);
}

static DEVICE_ATTR_RO(health_status);

static ssize_t health_impedance_index_show(struct device *dev,
					   struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 BHI_ROUND_INDEX(batt_drv->health_data.bhi_imp_index));
}

static DEVICE_ATTR_RO(health_impedance_index);

static ssize_t health_capacity_index_show(struct device *dev,
					  struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 BHI_ROUND_INDEX(batt_drv->health_data.bhi_cap_index));
}

static DEVICE_ATTR_RO(health_capacity_index);

static ssize_t health_index_stats_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	struct bhi_data *bhi_data = &batt_drv->health_data.bhi_data;
	struct health_data *health_data = &batt_drv->health_data;
	int len = 0, i;

	/* might be POR and FG not ready */
	if (bhi_data->cycle_count <= 0)
		return len;

	mutex_lock(&batt_drv->chg_lock);

	for (i = BHI_ALGO_DISABLED; i < BHI_ALGO_MAX; i++) {
		int health_index, health_status, cap_index, imp_index, sd_index;
		const int use_algo = batt_bhi_map_algo(i, health_data);

		cap_index = bhi_calc_cap_index(use_algo, batt_drv);
		cap_index = bhi_cap_index_bound(use_algo, cap_index);
		imp_index = bhi_calc_imp_index(use_algo, health_data);
		sd_index = bhi_calc_sd_index(use_algo, health_data);

		health_index = bhi_calc_health_index(use_algo, health_data, cap_index,
						     imp_index, sd_index);
		health_status = bhi_calc_health_status(use_algo, BHI_ROUND_INDEX(health_index),
						       health_data);

		if (health_index < 0)
			continue;

		pr_debug("bhi: %d: %d, %d,%d,%d %d,%d,%d %d,%d\n", i,
			 health_status, health_index, cap_index, imp_index,
			 bhi_data->swell_cumulative,
			 bhi_health_get_capacity(use_algo, bhi_data),
			 bhi_health_get_impedance(use_algo, bhi_data),
			 bhi_data->battery_age,
			 bhi_data->cycle_count);


		len += scnprintf(&buf[len], PAGE_SIZE - len,
				 "%d: %d, %d,%d,%d %d,%d,%d %d,%d, %d\n",
				 i, health_status,
				 BHI_ROUND_INDEX(health_index),
				 BHI_ROUND_INDEX(cap_index),
				 BHI_ROUND_INDEX(imp_index),
				 bhi_data->swell_cumulative,
				 bhi_health_get_capacity(use_algo, bhi_data),
				 bhi_health_get_impedance(use_algo, bhi_data),
				 bhi_data->battery_age,
				 bhi_data->cycle_count,
				 batt_bpst_stats_update(batt_drv));
	}

	mutex_unlock(&batt_drv->chg_lock);

	return len;
}

static DEVICE_ATTR_RO(health_index_stats);

static ssize_t health_algo_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	int value, ret;

	ret = kstrtoint(buf, 0, &value);
	if (ret < 0)
		return ret;

	if (value < BHI_ALGO_DISABLED || value >= BHI_ALGO_MAX || value == BHI_ALGO_DTOOL)
		return -EINVAL;

	mutex_lock(&batt_drv->chg_lock);
	batt_drv->health_data.bhi_algo = value;
	ret = batt_bhi_stats_update_all(batt_drv);
	mutex_unlock(&batt_drv->chg_lock);

	if (ret < 0)
		return ret;

	return count;
}

static ssize_t health_algo_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);

	return scnprintf(buf, PAGE_SIZE, "%d\n", batt_drv->health_data.bhi_algo);
}

static DEVICE_ATTR_RW(health_algo);

static ssize_t health_indi_cap_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	int value, ret;

	ret = kstrtoint(buf, 0, &value);
	if (ret < 0)
		return ret;

	if (value > 100 || value < 0)
		return count;

	batt_drv->health_data.bhi_indi_cap = value;

	return count;
}

static ssize_t health_indi_cap_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);

	return scnprintf(buf, PAGE_SIZE, "%d\n", batt_drv->health_data.bhi_indi_cap);
}

static DEVICE_ATTR_RW(health_indi_cap);

static ssize_t manufacturing_date_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	struct bm_date *date = &batt_drv->health_data.bhi_data.bm_date;
	struct rtc_time tm;

	/* read manufacturing date when data is not loaded yet */
	if (date->bm_y == 0) {
		int ret;

		ret = batt_get_manufacture_date(&batt_drv->health_data.bhi_data);
		if (ret < 0)
			return scnprintf(buf, PAGE_SIZE, "%d\n", ret);
	}

	tm.tm_year = date->bm_y + 100;	// base is 1900
	tm.tm_mon = date->bm_m - 1;	// 0 is Jan ... 11 is Dec
	tm.tm_mday = date->bm_d;	// 1st ... 31th

	return scnprintf(buf, PAGE_SIZE, "%lld\n", rtc_tm_to_time64(&tm));
}

static DEVICE_ATTR_RO(manufacturing_date);

#define FIRST_USAGE_DATE_DEFAULT	1606780800 //2020-12-01
#define FIRST_USAGE_DATE_MAX		2147483647 //2038-01-19

static ssize_t first_usage_date_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	struct bhi_data *bhi_data = &batt_drv->health_data.bhi_data;
	int value, ret;

	ret = kstrtoint(buf, 0, &value);
	if (ret < 0)
		return ret;

	/* return if the device tree is set */
	if (bhi_data->first_usage_date >= 0)
		return count > 0 ? count : 0;

	/*
	 * set: epoch
	 * when value is 0, set by local time; otherwise, set by system call.
	 */
	if (value == 0 ||
	    (value >= FIRST_USAGE_DATE_DEFAULT && value <= FIRST_USAGE_DATE_MAX)) {
		u8 act_date[BATT_EEPROM_TAG_XYMD_LEN];
		unsigned long long date_in_epoch = value;
		struct rtc_time tm;

		ret = gbms_storage_read(GBMS_TAG_AYMD, act_date, sizeof(act_date));
		if (ret < 0)
			return -EINVAL;

		if (act_date[0] != 0xff || act_date[1] != 0xff || act_date[2] != 0xff)
			return count > 0 ? count : 0;

		if (date_in_epoch == 0) {
			struct timespec64 ts;

			/* set by local time */
			ktime_get_real_ts64(&ts);
			date_in_epoch = ts.tv_sec - (sys_tz.tz_minuteswest * 60);
		}

		/* read manufacturing date when data is not loaded yet */
		if (bhi_data->bm_date.bm_y == 0) {
			ret = batt_get_manufacture_date(bhi_data);
			if (ret < 0)
				pr_warn("cannot get battery manufacture date, ret=%d\n", ret);
		}

		if (bhi_data->bm_date.bm_y) {
			unsigned long long mdate_in_epoch = batt_mdate_to_epoch(bhi_data);

			/* not sooner then manufacture date */
			if (date_in_epoch < mdate_in_epoch)
				date_in_epoch = mdate_in_epoch;
		}
		rtc_time64_to_tm(date_in_epoch, &tm);

		/* convert epoch to date
		 * for example:
		 * epoch: 1643846400 -> tm_year/tm_mon/tm_mday: 122/01/03
		 *                   -> date: 2/02/03 (LAST DIGIT OF YEAR)
		 *                   -> act_date: ASCII 2/2/3
		 */
		act_date[0] = date_to_xymd(tm.tm_year - 100 - 20);	// base is 1900
		act_date[1] = date_to_xymd(tm.tm_mon + 1);		// 0 is Jan ... 11 is Dec
		act_date[2] = date_to_xymd(tm.tm_mday);			// 1st ... 31th

		ret = gbms_storage_write(GBMS_TAG_AYMD, act_date, sizeof(act_date));
		if (ret < 0)
			return -EINVAL;

		/* update bhi_data->act_date */
		memcpy(&bhi_data->act_date, act_date, sizeof(act_date));
	} else {
		pr_warn("%s: input value is invalid %d\n", __func__, value);
	}

	return count > 0 ? count : 0;
}

static ssize_t first_usage_date_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	struct bhi_data *bhi_data = &batt_drv->health_data.bhi_data;
	struct rtc_time tm;
	int ret;

	/* return if the device tree is set */
	if (bhi_data->first_usage_date >= 0)
		return scnprintf(buf, PAGE_SIZE, "%d\n", bhi_data->first_usage_date);

	ret = get_activation_date(&batt_drv->health_data, &tm);
	if (ret < 0)
		return scnprintf(buf, PAGE_SIZE, "%d\n", -EIO);

	return scnprintf(buf, PAGE_SIZE, "%lld\n", rtc_tm_to_time64(&tm));
}

static DEVICE_ATTR_RW(first_usage_date);

static int batt_get_charging_state(const struct batt_drv *batt_drv)
{
	int ret = BATTERY_STATUS_UNKNOWN;
	int type, status;

	/* wait for csi_type updated */
	if (!batt_drv->csi_type_votable)
		return ret;

	type = gvotable_get_current_int_vote(batt_drv->csi_type_votable);
	switch (type) {
	case CSI_TYPE_Normal:
	case CSI_TYPE_None:
		ret = BATTERY_STATUS_NORMAL;
		break;
	case CSI_TYPE_JEITA:
		/* wait for csi_status updated */
		if (!batt_drv->csi_status_votable)
			break;

		status = gvotable_get_current_int_vote(batt_drv->csi_status_votable);
		ret = (status == CSI_STATUS_Health_Cold) ?
		      BATTERY_STATUS_TOO_COLD : BATTERY_STATUS_TOO_HOT;
		break;
	case CSI_TYPE_LongLife:
		ret = BATTERY_STATUS_LONGLIFE;
		break;
	case CSI_TYPE_Adaptive:
		ret = BATTERY_STATUS_ADAPTIVE;
		break;
	default:
		break;
	}

	return ret;
}

static ssize_t charging_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	int charging_state = batt_get_charging_state(batt_drv);

	return scnprintf(buf, PAGE_SIZE, "%d\n", charging_state);
}

static DEVICE_ATTR_RO(charging_state);

static void batt_update_charging_policy(struct batt_drv *batt_drv)
{
	int value, ret;

	value = gvotable_get_current_int_vote(batt_drv->charging_policy_votable);
	if (value == batt_drv->charging_policy)
		return;

	/* update adaptive charging */
	if (value == CHARGING_POLICY_VOTE_ADAPTIVE_AON)
		batt_set_health_charge_limit(batt_drv, ADAPTIVE_ALWAYS_ON_SOC);
	else if (value != CHARGING_POLICY_VOTE_ADAPTIVE_AON &&
		   batt_drv->charging_policy == CHARGING_POLICY_VOTE_ADAPTIVE_AON)
		batt_set_health_charge_limit(batt_drv, -1);

	gbms_logbuffer_devlog(batt_drv->ttf_stats.ttf_log, batt_drv->device,
			      LOGLEVEL_INFO, 0, LOGLEVEL_DEBUG,
			      "update charging_policy: %d -> %d\n",
			      batt_drv->charging_policy, value);
	batt_drv->charging_policy = value;

	gvotable_cast_long_vote(batt_drv->csi_status_votable, "CSI_STATUS_DEFEND_LIMIT",
				CSI_STATUS_Defender_Limit,
				value == CHARGING_POLICY_VOTE_LONGLIFE);

	if (value != CHARGING_POLICY_VOTE_LONGLIFE)
		return;

	/* for LONGLIFE: make full charge happens XXX cycles from the current cycle */
	batt_drv->last_full_charge = batt_drv->hist_data_saved_cnt;
	ret = gbms_storage_write(GBMS_TAG_FCRU, &batt_drv->last_full_charge,
				 GBMS_FCRU_LEN);
	if (ret < 0)
		pr_err("failed to store FCNU (%d)\n", ret);
}

static int charging_policy_translate(int value)
{
	int ret = CHARGING_POLICY_VOTE_DEFAULT;

	switch (value) {
	case CHARGING_POLICY_DEFAULT:
		ret = CHARGING_POLICY_VOTE_DEFAULT;
		break;
	case CHARGING_POLICY_LONGLIFE:
		ret = CHARGING_POLICY_VOTE_LONGLIFE;
		break;
	case CHARGING_POLICY_ADAPTIVE:
		ret = CHARGING_POLICY_VOTE_ADAPTIVE_AON;
		break;
	default:
		break;
	}

	return ret;
}

static ssize_t charging_policy_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	int value, ret;

	ret = kstrtoint(buf, 0, &value);
	if (ret < 0)
		return ret;

	if (value > CHARGING_POLICY_ADAPTIVE || value < CHARGING_POLICY_DEFAULT)
		return count;

	if (!batt_drv->charging_policy_votable) {
		batt_drv->charging_policy_votable =
			gvotable_election_get_handle(VOTABLE_CHARGING_POLICY);
		if (!batt_drv->charging_policy_votable)
			return count;
	}

	gvotable_cast_long_vote(batt_drv->charging_policy_votable, "MSC_USER",
				charging_policy_translate(value), true);
	batt_update_charging_policy(batt_drv);

	return count;
}

static ssize_t charging_policy_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	int value = CHARGING_POLICY_VOTE_UNKNOWN;

	if (!batt_drv->charging_policy_votable)
		batt_drv->charging_policy_votable =
			gvotable_election_get_handle(VOTABLE_CHARGING_POLICY);
	if (batt_drv->charging_policy_votable)
		value = gvotable_get_current_int_vote(batt_drv->charging_policy_votable);

	return scnprintf(buf, PAGE_SIZE, "%d\n", value);
}

static DEVICE_ATTR_RW(charging_policy);

static ssize_t health_set_cal_mode_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = (struct batt_drv *)power_supply_get_drvdata(psy);
	int value, ret;

	ret = kstrtoint(buf, 0, &value);
	if (ret < 0)
		return ret;

	mutex_lock(&batt_drv->chg_lock);
	if (batt_drv->health_data.cal_state == REC_STATE_SCHEDULED)
		goto exit_done;

	gbms_logbuffer_prlog(batt_drv->ttf_stats.ttf_log, LOGLEVEL_INFO, 0, LOGLEVEL_DEBUG,
			     "RE_CAL: cal_state: %d, cal_mode:%d -> %d\n",
			     batt_drv->health_data.cal_state,
			     batt_drv->health_data.cal_mode, value);
	batt_drv->health_data.cal_mode = value;
	ret = batt_bhi_update_recalibration_status(batt_drv);
	if (ret < 0)
		count = ret;

exit_done:
	mutex_unlock(&batt_drv->chg_lock);
	return count;
}

static ssize_t health_set_cal_mode_show(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = (struct batt_drv *)power_supply_get_drvdata(psy);

	return scnprintf(buf, PAGE_SIZE, "%d\n", batt_drv->health_data.cal_mode);
}

static DEVICE_ATTR_RW(health_set_cal_mode);

static ssize_t health_get_cal_state_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);

	return scnprintf(buf, PAGE_SIZE, "%d\n", batt_drv->health_data.cal_state);
}

static DEVICE_ATTR_RO(health_get_cal_state);

static ssize_t health_set_low_boundary_store(struct device *dev,
					     struct device_attribute *attr,
					     const char *buf, size_t count)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	struct bhi_data *bhi_data = &batt_drv->health_data.bhi_data;
	char tp_type;
	u16 *cpb, capacity[BHI_TREND_POINTS_SIZE];
	const int buf_len = strlen(buf);
	int cnt = 0, len = 0, pos=0, cap_min, cap_max;
	u16 batt_id;

	/* TODO: b/309695456 input data validity check */
	do {
		cap_min = BHI_CAPACITY_MIN;
		cap_max = BHI_CAPACITY_MAX;

		sscanf(&buf[len], "%c:%n", &tp_type, &pos);

		/* if type is not given(pos!=0), using default type - GBMS_TP_TRENDPOINTS */
		if (pos > 0)
			len += pos;
		else
			tp_type = GBMS_TP_TRENDPOINTS;

		switch (tp_type) {
		case GBMS_TP_TRENDPOINTS:
			cpb = &bhi_data->trend[0];
			break;
		case GBMS_TP_LOWER_BOUND:
			cpb = &bhi_data->lower_bound.limit[0];
			break;
		case GBMS_TP_UPPER_BOUND:
			cpb = &bhi_data->upper_bound.limit[0];
			break;
		case GBMS_TP_LOWER_TRIGGER:
			cpb = &bhi_data->lower_bound.trigger[0];
			break;
		case GBMS_TP_UPPER_TRIGGER:
			cpb = &bhi_data->upper_bound.trigger[0];
			break;
		default:
			dev_err(&batt_drv->psy->dev, "incorrect boundary type:%c\n", tp_type);
			return -ERANGE;
		}

		cnt = sscanf(&buf[len], "%hu,%hu,%hu,%hu,%hu,%hu,%hu,%hu,%hu,%hu,%hu%n", &batt_id,
			     &capacity[0], &capacity[1], &capacity[2], &capacity[3], &capacity[4],
			     &capacity[5], &capacity[6], &capacity[7], &capacity[8], &capacity[9],
			     &pos);

		if (cnt != BHI_TREND_POINTS_SIZE + 1)
			return -ERANGE;

		if ((int)batt_id == batt_drv->batt_id &&
		    bhi_bound_validity_check(capacity, cap_min, cap_max))
			memcpy(&cpb[0], capacity, sizeof(capacity));

		len += pos;

		while (buf[len] != '\n' && len < buf_len)
			len++;
	} while (len++ < buf_len);

	return count;
}

static inline int trend_points_to_buffer(char *buf, size_t len, char type, u16 *points)
{
	return scnprintf(buf,len, "%c:%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n", type,
			 points[0], points[1], points[2], points[3], points[4], points[5],
			 points[6], points[7], points[8], points[9]);
}

static ssize_t health_set_low_boundary_show(struct device *dev, struct device_attribute *attr,
					    char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	struct bhi_data *bhi_data = &batt_drv->health_data.bhi_data;
	int written;
	size_t pos = 0;

	written = trend_points_to_buffer(buf + pos, PAGE_SIZE - pos, GBMS_TP_TRENDPOINTS,
					 bhi_data->trend);
	if (!written)
		return -ENOSPC;
	pos += written;

	written = trend_points_to_buffer(buf + pos, PAGE_SIZE - pos, GBMS_TP_LOWER_BOUND,
					 bhi_data->lower_bound.limit);
	if (!written)
		return -ENOSPC;
	pos += written;

	written = trend_points_to_buffer(buf + pos, PAGE_SIZE - pos, GBMS_TP_UPPER_BOUND,
					 bhi_data->upper_bound.limit);
	if (!written)
		return -ENOSPC;
	pos += written;

	written = trend_points_to_buffer(buf + pos, PAGE_SIZE - pos, GBMS_TP_LOWER_TRIGGER,
					 bhi_data->lower_bound.trigger);
	if (!written)
		return -ENOSPC;
	pos += written;

	written = trend_points_to_buffer(buf + pos, PAGE_SIZE - pos, GBMS_TP_UPPER_TRIGGER,
					 bhi_data->upper_bound.trigger);
	if (!written)
		return -ENOSPC;
	pos += written;

	return pos;
}

static DEVICE_ATTR_RW(health_set_low_boundary);

static int debug_bhi_cycle_grace_write(void *data, u64 val)
{
	struct batt_drv *batt_drv = (struct batt_drv *)data;

	if (!batt_drv->psy)
		return -EINVAL;

	batt_drv->health_data.bhi_cycle_grace = (int)val;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_bhi_cycle_grace_fops, NULL, debug_bhi_cycle_grace_write, "%llu\n");

/* CSI --------------------------------------------------------------------- */

static ssize_t charging_speed_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	int value, ret;

	ret = kstrtoint(buf, 0, &value);
	if (ret < 0)
		return ret;

	pr_info("fake_charging_speed: %d -> %d\n", batt_drv->fake_charging_speed, value);
	batt_drv->fake_charging_speed = value;

	return count;
}

static ssize_t charging_speed_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);

	return scnprintf(buf, PAGE_SIZE, "%d\n", batt_drv->csi_current_speed);
}

static DEVICE_ATTR_RW(charging_speed);

static ssize_t csi_stats_store(struct device *dev, struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);

	if (count < 1)
		return -ENODATA;

	if (buf[0] == '0')
		batt_init_csi_stat(batt_drv);

	return count;
}

static ssize_t csi_stats_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	struct csi_stats *stats = &batt_drv->csi_stats;
	int ver = 0;

	if (stats->time_stat_last_update == 0 || stats->time_sum == 0)
		return 0;

	return scnprintf(buf, PAGE_SIZE,
			"%d,%s,%d,%d,%d,%d,%lld,%d,%d,%lld,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
			 ver, gbms_chg_ev_adapter_s(stats->ad_type), stats->ad_voltage * 100,
			 stats->ad_amperage * 100, stats->ssoc_in, stats->ssoc_out,
			 stats->time_sum / 60, stats->aggregate_type, stats->aggregate_status,
			 stats->time_effective / 60, stats->temp_min, stats->temp_max,
			 stats->vol_in, stats->vol_out, stats->cc_in, stats->cc_out,
			 (int)(stats->thermal_severity[0] * 100 / stats->time_sum),
			 (int)(stats->thermal_severity[1] * 100 / stats->time_sum),
			 (int)(stats->thermal_severity[2] * 100 / stats->time_sum),
			 (int)(stats->thermal_severity[3] * 100 / stats->time_sum),
			 (int)(stats->thermal_severity[4] * 100 / stats->time_sum));
}

static DEVICE_ATTR_RW(csi_stats);

static ssize_t power_metrics_polling_rate_store(struct device *dev,
						struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	unsigned int value, ret = 0;

	ret = kstrtouint(buf, 0, &value);
	if (ret < 0)
		return ret;
	if (value <= 0)
		return -EINVAL;

	batt_drv->power_metrics.polling_rate = value;
	return count;
}

static ssize_t power_metrics_polling_rate_show(struct device *dev,
					       struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);

	return scnprintf(buf, PAGE_SIZE, "%d\n", batt_drv->power_metrics.polling_rate);
}

static DEVICE_ATTR_RW(power_metrics_polling_rate);

static ssize_t power_metrics_interval_store(struct device *dev,
					    struct device_attribute *attr,
					    const char *buf, size_t count)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	unsigned int value, ret = 0;
	const int polling_rate = batt_drv->power_metrics.polling_rate;

	ret = kstrtouint(buf, 0, &value);
	if (ret < 0)
		return ret;
	if ((value >= polling_rate * POWER_METRICS_MAX_DATA) || value < polling_rate)
		return -EINVAL;

	batt_drv->power_metrics.interval = value;
	return count;
}

static ssize_t power_metrics_interval_show(struct device *dev,
					   struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);

	return scnprintf(buf, PAGE_SIZE, "%d\n", batt_drv->power_metrics.interval);
}

static DEVICE_ATTR_RW(power_metrics_interval);

static long power_metrics_delta_cc(struct batt_drv *batt_drv, int idx1, int idx2)
{
	return batt_drv->power_metrics.data[idx1].charge_count -
	    batt_drv->power_metrics.data[idx2].charge_count;
}

static long power_metrics_avg_vbat(struct batt_drv *batt_drv, int idx1, int idx2)
{
	unsigned long v1 = batt_drv->power_metrics.data[idx1].voltage;
	unsigned long v2 = batt_drv->power_metrics.data[idx2].voltage;

	return (v1 + v2) / 2;
}

static long power_metrics_delta_time(struct batt_drv *batt_drv, int idx1, int idx2)
{
	return batt_drv->power_metrics.data[idx1].time -
	    batt_drv->power_metrics.data[idx2].time;
}

static ssize_t power_metrics_power_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	const unsigned int polling_rate = batt_drv->power_metrics.polling_rate;
	const unsigned int interval = batt_drv->power_metrics.interval;
	const unsigned int pm_idx = batt_drv->power_metrics.idx;
	unsigned int step, idx, idx_prev, i;
	long cc, vbat, time, time_prev;
	long power_avg = 0;

	step = interval / polling_rate;
	if (step == 0)
		return scnprintf(buf, PAGE_SIZE, "Error interval.\n");

	idx_prev = pm_idx;
	for (i = 1; i <= step; i++) {
		if (i > pm_idx)
			idx = pm_idx + POWER_METRICS_MAX_DATA - i;
		else
			idx = pm_idx - i;

		if (batt_drv->power_metrics.data[idx].voltage == 0)
			return scnprintf(buf, PAGE_SIZE, "Not enough data.\n");
		if (power_metrics_delta_time(batt_drv, idx_prev, idx) <= 0)
			return scnprintf(buf, PAGE_SIZE, "Time stamp error.\n");

		time = power_metrics_delta_time(batt_drv, pm_idx, idx);
		if (time < interval) {
			/* P += (dCC * V / dT) * (dT / interval) */
			cc = power_metrics_delta_cc(batt_drv, idx_prev, idx);
			vbat = power_metrics_avg_vbat(batt_drv, idx_prev, idx);
			power_avg += (cc * vbat) / interval;
			idx_prev = idx;
			continue;
		}

		if (i == 1) {
			/* P = dCC * V / dT */
			cc = power_metrics_delta_cc(batt_drv, pm_idx, idx);
			vbat = power_metrics_avg_vbat(batt_drv, pm_idx, idx);
			power_avg = cc * vbat / time;
		} else {
			/* P += (dCC * V / dT) * (the left time to interval / interval) */
			cc = power_metrics_delta_cc(batt_drv, idx_prev, idx);
			vbat = power_metrics_avg_vbat(batt_drv, idx_prev, idx);
			time = power_metrics_delta_time(batt_drv, idx_prev, idx);
			time_prev = power_metrics_delta_time(batt_drv, pm_idx, idx_prev);
			power_avg += (cc * vbat * (interval - time_prev) / time ) / interval;
		}

		break;
	}

	return scnprintf(buf, PAGE_SIZE, "%ld\n", power_avg / 1000000);
}

static DEVICE_ATTR_RO(power_metrics_power);

static ssize_t power_metrics_current_show(struct device *dev,
					  struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	const unsigned int polling_rate = batt_drv->power_metrics.polling_rate;
	const unsigned int interval = batt_drv->power_metrics.interval;
	const unsigned int pm_idx = batt_drv->power_metrics.idx;
	unsigned int step, idx, idx_prev, i;
	long cc, time, time_prev;
	long current_avg = 0;

	step = interval / polling_rate;
	if (step == 0)
		return scnprintf(buf, PAGE_SIZE, "Error interval.\n");

	idx_prev = pm_idx;
	for (i = 1; i <= step; i++) {
		if (i > pm_idx)
			idx = pm_idx + POWER_METRICS_MAX_DATA - i;
		else
			idx = pm_idx - i;

		if (batt_drv->power_metrics.data[idx].voltage == 0)
			return scnprintf(buf, PAGE_SIZE, "Not enough data.\n");
		if (power_metrics_delta_time(batt_drv, idx_prev, idx) <= 0)
			return scnprintf(buf, PAGE_SIZE, "Time stamp error.\n");

		time = power_metrics_delta_time(batt_drv, pm_idx, idx);
		if (time < interval) {
			/* I += (dCC / dT) * (dT / interval) */
			cc = power_metrics_delta_cc(batt_drv, idx_prev, idx);
			current_avg += cc / interval;
			idx_prev = idx;
			continue;
		}

		if (i == 1) {
			/* I = dCC / dT */
			cc = power_metrics_delta_cc(batt_drv, pm_idx, idx);
			current_avg = cc / time;
		} else {
			 /* I += (dCC / dT) * (the left time to interval / interval) */
			cc = power_metrics_delta_cc(batt_drv, idx_prev, idx);
			time = power_metrics_delta_time(batt_drv, idx_prev, idx);
			time_prev = power_metrics_delta_time(batt_drv, pm_idx, idx_prev);
			current_avg += (cc * (interval - time_prev) / time) / interval;
		}

		break;
	}

	return scnprintf(buf, PAGE_SIZE, "%ld\n", current_avg);
}

static DEVICE_ATTR_RO(power_metrics_current);

static ssize_t dev_sn_store(struct device *dev,
			    struct device_attribute *attr,
			    const char *buf, size_t count)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	const size_t max_len = sizeof(batt_drv->dev_sn);

	if (strlcpy(batt_drv->dev_sn, buf, max_len) >= max_len)
		pr_warn("Paired data out of bounds\n");

	return count;
}

static ssize_t dev_sn_show(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);

	return scnprintf(buf, PAGE_SIZE, "%s\n", batt_drv->dev_sn);
}

static DEVICE_ATTR_RW(dev_sn);

static ssize_t temp_filter_enable_store(struct device *dev,
			    struct device_attribute *attr,
			    const char *buf, size_t count)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	struct batt_temp_filter *temp_filter = &batt_drv->temp_filter;
	int val, ret;
	bool enable;

	ret = kstrtoint(buf, 0, &val);
	if (ret < 0)
		return ret;

	enable = val != 0;

	if (temp_filter->enable != enable) {
		temp_filter->enable = enable;
		temp_filter->force_update = true;
		mod_delayed_work(system_wq, &temp_filter->work, 0);
	}

	return count;
}

static ssize_t temp_filter_enable_show(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);

	return scnprintf(buf, PAGE_SIZE, "%d\n", batt_drv->temp_filter.enable);
}

static DEVICE_ATTR_RW(temp_filter_enable);

static ssize_t force_fcr_update_ops_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	int val, ret;

	ret = kstrtoint(buf, 0, &val);
	if (ret < 0)
		return ret;

	/* chg_lock protect msc_logic */
	mutex_lock(&batt_drv->batt_lock);

	batt_drv->force_fcr_update_ops = val;

	/* if force full charge was active */
	if (batt_drv->force_fcr_update_ops == BATT_FCR_UPDATE_DISABLE)
		batt_force_fcr_update_charging_policy(batt_drv, false);

	mutex_unlock(&batt_drv->batt_lock);

	return count;
}

static ssize_t force_fcr_update_ops_show(struct device *dev,
					 struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);

	return scnprintf(buf, PAGE_SIZE, "%d\n", batt_drv->force_fcr_update_ops);
}

static DEVICE_ATTR_RW(force_fcr_update_ops);

/* ------------------------------------------------------------------------- */

static struct attribute *batt_attrs[] = {
	&dev_attr_charge_stats.attr,
	&dev_attr_charge_stats_actual.attr,
	&dev_attr_charge_details.attr,
	&dev_attr_ssoc_details.attr,
	&dev_attr_charge_deadline.attr,
	&dev_attr_charge_stage.attr,
	&dev_attr_charge_limit.attr,
	&dev_attr_time_to_ac.attr,
	&dev_attr_ac_soc.attr,
	&dev_attr_charge_deadline_dryrun.attr,
	&dev_attr_charge_to_limit.attr,
	&dev_attr_ttf_stats.attr,
	&dev_attr_ttf_details.attr,
	&dev_attr_bd_trickle_enable.attr,
	&dev_attr_bd_trickle_cnt.attr,
	&dev_attr_bd_trickle_recharge_soc.attr,
	&dev_attr_bd_trickle_dry_run.attr,
	&dev_attr_bd_trickle_reset_sec.attr,
	&dev_attr_bd_clear.attr,
	&dev_attr_pairing_state.attr,
	&dev_attr_cycle_counts.attr,
	&dev_attr_charge_full_estimate.attr,
	&dev_attr_resistance_avg.attr,
	&dev_attr_resistance.attr,
	&dev_attr_charger_state.attr,
	&dev_attr_charge_type.attr,
	&dev_attr_constant_charge_current.attr,
	&dev_attr_constant_charge_voltage.attr,
	&dev_attr_health_safety_margin.attr,
	&dev_attr_aacr_state.attr,
	&dev_attr_aacr_config.attr,
	&dev_attr_aacr_cycle_grace.attr,
	&dev_attr_aacr_cycle_max.attr,
	&dev_attr_aacr_algo.attr,
	&dev_attr_aacr_min_capacity_rate.attr,
	&dev_attr_aacr_cliff_capacity_rate.attr,
	&dev_attr_aacr_profile.attr,
	&dev_attr_aafv_state.attr,
	&dev_attr_aafv_config.attr,
	&dev_attr_aafv_apply_max.attr,
	&dev_attr_aafv_max_offset.attr,
	&dev_attr_aafv_cliff_cycle.attr,
	&dev_attr_aafv_cliff_offset.attr,
	&dev_attr_aafv_profile.attr,
	&dev_attr_aafv_offset.attr,
	&dev_attr_aact_state.attr,
	&dev_attr_aact_config.attr,
	&dev_attr_aact_cv_limits.attr,
	&dev_attr_aact_temp_limits.attr,
	&dev_attr_aact_chg_ecc.attr,
	&dev_attr_aact_profile.attr,
	&dev_attr_aacp_version.attr,
	&dev_attr_aacp_opt_out.attr,
	&dev_attr_aacc.attr,
	&dev_attr_swelling_data.attr,
	&dev_attr_health_index.attr,
	&dev_attr_health_status.attr,
	&dev_attr_health_capacity_index.attr,
	&dev_attr_health_index_stats.attr,
	&dev_attr_health_impedance_index.attr,
	&dev_attr_health_algo.attr,
	&dev_attr_health_indi_cap.attr,
	&dev_attr_manufacturing_date.attr,
	&dev_attr_first_usage_date.attr,
	&dev_attr_charging_state.attr,
	&dev_attr_charging_policy.attr,
	&dev_attr_health_set_cal_mode.attr,
	&dev_attr_health_get_cal_state.attr,
	&dev_attr_health_set_low_boundary.attr,
	&dev_attr_charging_speed.attr,
	&dev_attr_csi_stats.attr,
	&dev_attr_power_metrics_polling_rate.attr,
	&dev_attr_power_metrics_interval.attr,
	&dev_attr_power_metrics_power.attr,
	&dev_attr_power_metrics_current.attr,
	&dev_attr_dev_sn.attr,
	&dev_attr_temp_filter_enable.attr,
	&dev_attr_chg_profile_switch.attr,
	&dev_attr_force_fcr_update_ops.attr,
	NULL,
};

static const struct attribute_group batt_attr_grp = {
	.attrs = batt_attrs,
};

static int batt_init_fs(struct batt_drv *batt_drv)
{
	int ret;

	ret = sysfs_create_group(&batt_drv->psy->dev.kobj, &batt_attr_grp);
	if (ret)
		dev_err(&batt_drv->psy->dev, "Failed to create sysfs group\n");

	return 0;

}

static int batt_init_debugfs(struct batt_drv *batt_drv)
{
	struct dentry *de = NULL;

	de = debugfs_create_dir("google_battery", 0);
	if (IS_ERR_OR_NULL(de))
		return 0;

	debugfs_create_u32("debug_level", 0644, de, &debug_printk_prlog);
	debugfs_create_file("cycle_count_sync", 0600, de, batt_drv,
			    &cycle_count_bins_sync_fops);
	debugfs_create_file("ssoc_gdf", 0644, de, batt_drv, &debug_ssoc_gdf_fops);
	debugfs_create_file("ssoc_uic", 0644, de, batt_drv, &debug_ssoc_uic_fops);
	debugfs_create_file("ssoc_rls", 0444, de, batt_drv, &debug_ssoc_rls_fops);
	debugfs_create_file("fv_dc_ratio", 0644, de, batt_drv, &debug_fv_dc_ratio_fops);
	debugfs_create_file("mp_tz", 0644, de, batt_drv, &debug_mp_tz_fops);
	debugfs_create_u32("mp_soc_limit_low", 0600, de, &batt_drv->soc_mp_limit_low);
	debugfs_create_u32("mp_soc_limit_high", 0600, de, &batt_drv->soc_mp_limit_high);
	debugfs_create_u32("mp_therm_limit", 0600, de, &batt_drv->therm_mp_limit);
	debugfs_create_u32("mp_max_ratio_limit", 0600, de, &batt_drv->max_ratio_mp_limit);
	debugfs_create_u32("hda_tz_limit", 0600, de, &batt_drv->hda_tz_limit);
	debugfs_create_file("ssoc_uicurve", 0644, de, batt_drv,
			    &debug_ssoc_uicurve_cstr_fops);
	debugfs_create_file("force_psy_update", 0400, de, batt_drv,
			    &debug_force_psy_update_fops);
	debugfs_create_file("pairing_state", 0200, de, batt_drv, &debug_pairing_fops);
	debugfs_create_file("temp", 0400, de, batt_drv, &debug_fake_temp_fops);
	debugfs_create_u32("battery_present", 0600, de,
			   &batt_drv->fake_battery_present);

	/* history */
	debugfs_create_file("blf_state", 0400, de, batt_drv, &debug_blf_state_fops);
	debugfs_create_u32("blf_collect_now", 0600, de, &batt_drv->blf_collect_now);

	/* defender */
	debugfs_create_u32("fake_capacity", 0600, de, &batt_drv->fake_capacity);

	/* aacp test */
	debugfs_create_u32("fake_aacp_cc", 0600, de, &batt_drv->fake_aacp_cc);

	/* health charging (adaptive charging) */
	debugfs_create_file("chg_health_thr_soc", 0600, de, batt_drv,
			    &debug_chg_health_thr_soc_fops);
	debugfs_create_file("chg_health_rest_rate", 0600, de, batt_drv,
			    &debug_chg_health_rest_rate_fops);
	debugfs_create_file("chg_health_rest_rate_before_trigger", 0600, de, batt_drv,
			    &debug_chg_health_rest_rate_before_trigger_fops);
	debugfs_create_file("chg_health_stage", 0600, de, batt_drv,
			    &debug_chg_health_stage_fops);

	/* charging table */
	debugfs_create_file("chg_raw_profile", 0644, de, batt_drv,
			    &debug_chg_raw_profile_fops);

	/* battery virtual sensor*/
	debugfs_create_u32("batt_vs_w", 0600, de, &batt_drv->batt_vs_w);

	/* power metrics */
	debugfs_create_file("power_metrics", 0400, de, batt_drv, &debug_power_metrics_fops);

	/* bhi fullcapnom count */
	debugfs_create_u32("bhi_w_ci", 0644, de, &batt_drv->health_data.bhi_w_ci);
	debugfs_create_u32("bhi_w_pi", 0644, de, &batt_drv->health_data.bhi_w_pi);
	debugfs_create_u32("bhi_w_sd", 0644, de, &batt_drv->health_data.bhi_w_sd);
	debugfs_create_u32("act_impedance", 0644, de,
			   &batt_drv->health_data.bhi_data.act_impedance);
	debugfs_create_u32("bhi_debug_cycle_count", 0644, de,
			   &batt_drv->health_data.bhi_debug_cycle_count);
	debugfs_create_u32("bhi_debug_cap_idx", 0644, de,
			   &batt_drv->health_data.bhi_debug_cap_index);
	debugfs_create_u32("bhi_debug_imp_idx", 0644, de,
			   &batt_drv->health_data.bhi_debug_imp_index);
	debugfs_create_u32("bhi_debug_sd_idx", 0644, de,
			   &batt_drv->health_data.bhi_debug_sd_index);
	debugfs_create_u32("bhi_debug_health_idx", 0644, de,
			   &batt_drv->health_data.bhi_debug_health_index);
	debugfs_create_u32("bhi_debug_health_status", 0644, de,
			   &batt_drv->health_data.bhi_debug_health_status);
	debugfs_create_file("bhi_debug_status", 0644, de, batt_drv,
			   &debug_bhi_status_fops);
	debugfs_create_file("bhi_debug_cycle_grace", 0644, de, batt_drv,
			    &debug_bhi_cycle_grace_fops);
	debugfs_create_file("reset_first_usage_date", 0644, de, batt_drv,
			    &debug_first_usage_date_fops);

	/* google_resistance, tuning */
	debugfs_create_u32("ravg_temp_low", 0644, de,
			   &batt_drv->health_data.bhi_data.res_state.res_temp_low);
	debugfs_create_u32("ravg_temp_high", 0644, de,
			   &batt_drv->health_data.bhi_data.res_state.res_temp_high);
	debugfs_create_u32("ravg_soc_low", 0644, de,
			   &batt_drv->health_data.bhi_data.res_state.ravg_soc_low);
	debugfs_create_u32("ravg_soc_high", 0644, de,
			   &batt_drv->health_data.bhi_data.res_state.ravg_soc_high);
	debugfs_create_file("ravg", 0400, de,  batt_drv, &debug_ravg_fops);

	/* battery temperature filter */
	debugfs_create_u32("temp_filter_default_interval", 0644, de,
			   &batt_drv->temp_filter.default_interval);
	debugfs_create_u32("temp_filter_fast_interval", 0644, de,
			   &batt_drv->temp_filter.fast_interval);
	debugfs_create_u32("temp_filter_resume_delay_interval", 0644, de,
			   &batt_drv->temp_filter.resume_delay_time);

	/* shutdown flag */
	debugfs_create_u32("boot_to_os_attempts", 0660, de, &batt_drv->boot_to_os_attempts);

	/* drain test */
	debugfs_create_u32("restrict_level_critical", 0644, de, &batt_drv->restrict_level_critical);

	return 0;
}

/* bpst detection */
static int batt_bpst_init_fs(struct batt_drv *batt_drv)
{
	int ret;

	if (!batt_drv->bpst_state.bpst_enable)
		return 0;

	ret = device_create_file(&batt_drv->psy->dev, &dev_attr_bpst_reset);
	if (ret)
		dev_err(&batt_drv->psy->dev, "Failed to create bpst_reset\n");
	ret = device_create_file(&batt_drv->psy->dev, &dev_attr_bpst_detect_disable);
	if (ret)
		dev_err(&batt_drv->psy->dev, "Failed to create bpst_detect_disable\n");

	return 0;

}

static int batt_bpst_init_debugfs(struct batt_drv *batt_drv)
{
	struct dentry *de = NULL;

	de = debugfs_create_dir("bpst", 0);
	if (IS_ERR_OR_NULL(de))
		return 0;

	debugfs_create_file("bpst_sbd_status", 0600, de, batt_drv,
			    &debug_bpst_sbd_status_fops);
	debugfs_create_u32("bpst_count_threshold", 0600, de,
			    &batt_drv->bpst_state.bpst_count_threshold);
	debugfs_create_u32("bpst_chg_rate", 0600, de,
			    &batt_drv->bpst_state.bpst_chg_rate);

	return 0;
}

/* ------------------------------------------------------------------------- */

/* could also use battery temperature, age */
static bool gbatt_check_dead_battery(const struct batt_drv *batt_drv)
{
	return ssoc_get_capacity(&batt_drv->ssoc_state) == 0;
}

#define VBATT_CRITICAL_LEVEL		3300000
#define VBATT_CRITICAL_DEADLINE_SEC	40

static bool gbatt_check_critical_level(const struct batt_drv *batt_drv,
				       int fg_status)
{
	const struct batt_ssoc_state *ssoc_state = &batt_drv->ssoc_state;
	const int soc = ssoc_get_real(ssoc_state);

	if (fg_status == POWER_SUPPLY_STATUS_UNKNOWN)
		return true;

	if (batt_drv->restrict_level_critical || soc != 0)
		return false;

	/* debounce with battery voltage (if set) for VBATT_CRITICAL_DEADLINE_SEC at boot */
	if (ssoc_state->buck_enabled == 1 &&
	    (fg_status == POWER_SUPPLY_STATUS_DISCHARGING ||
	     fg_status == POWER_SUPPLY_STATUS_NOT_CHARGING)) {
		const ktime_t now = get_boot_sec();
		int vbatt;

		if (!batt_drv->fg_psy)
			return false;

		vbatt = PSY_GET_PROP(batt_drv->fg_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW);
		if (vbatt <= -EAGAIN)
			return false;

		/* disable the check */
		if (now > batt_drv->vbatt_crit_deadline_sec ||
		    batt_drv->batt_critical_voltage == 0)
			goto exit_done;

		if (vbatt >= batt_drv->batt_critical_voltage)
			return false;
exit_done:
		/* dump log for tuning parameters */
		pr_info("%s: vbatt: %d, v_th:%d, fg_status: %d, now: %lld\n",
			__func__, vbatt, batt_drv->batt_critical_voltage,
			fg_status, now);

		return true;
	}

	/* here soc == 0, shutdown if not connected or if state is not charging  */
	return ssoc_state->buck_enabled == 0 || fg_status != POWER_SUPPLY_STATUS_CHARGING;
}

#define SSOC_LEVEL_FULL		SSOC_SPOOF
#define SSOC_LEVEL_HIGH		80
#define SSOC_LEVEL_NORMAL	30
#define SSOC_LEVEL_LOW		0

/*
 * could also use battery temperature, age.
 * NOTE: this implementation looks at the SOC% but it might be looking to
 * other quantities or flags.
 * NOTE: CRITICAL_LEVEL implies BATTERY_DEAD but BATTERY_DEAD doesn't imply
 * CRITICAL.
 */
static int gbatt_get_capacity_level(const struct batt_drv *batt_drv,
				    int fg_status)
{
	const struct batt_ssoc_state *ssoc_state = &batt_drv->ssoc_state;
	const int soc = ssoc_get_real(ssoc_state);
	int capacity_level;

	if (soc >= SSOC_LEVEL_FULL) {
		capacity_level = POWER_SUPPLY_CAPACITY_LEVEL_FULL;
	} else if (soc > SSOC_LEVEL_HIGH) {
		capacity_level = POWER_SUPPLY_CAPACITY_LEVEL_HIGH;
	} else if (soc > SSOC_LEVEL_NORMAL) {
		capacity_level = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
	} else if (soc > SSOC_LEVEL_LOW) {
		capacity_level = POWER_SUPPLY_CAPACITY_LEVEL_LOW;
	} else if (ssoc_state->buck_enabled == -1) {
		/* only at startup, this should not happen */
		capacity_level = POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN;
	} else if (gbatt_check_critical_level(batt_drv, fg_status)) {
		capacity_level = POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
	} else {
		capacity_level = POWER_SUPPLY_CAPACITY_LEVEL_LOW;
	}

	return capacity_level;
}

static int gbatt_get_temp(struct batt_drv *batt_drv, int *temp)
{
	int err = 0;

	if (batt_drv->fake_temp)
		*temp = batt_drv->fake_temp;
	else
		err = gbatt_get_raw_temp(batt_drv, temp);

	return err;
}

static int batt_do_md5(const u8 *data, unsigned int len, u8 *result)
{
	struct crypto_shash *tfm;
	struct shash_desc *shash;
	int size, ret = 0;

	tfm = crypto_alloc_shash("md5", 0, 0);
	if (IS_ERR(tfm)) {
		pr_err("Error MD5 transform: %ld\n", PTR_ERR(tfm));
		return PTR_ERR(tfm);
	}

	size = sizeof(struct shash_desc) + crypto_shash_descsize(tfm);
	shash = kmalloc(size, GFP_KERNEL);
	if (!shash)
		return -ENOMEM;

	shash->tfm = tfm;
	ret = crypto_shash_digest(shash, data, len, result);
	kfree(shash);
	crypto_free_shash(tfm);

	return ret;
}

/* called with a lock on ->chg_lock */
static enum batt_paired_state batt_check_pairing_state(struct batt_drv *batt_drv)
{
	char dev_info[GBMS_DINF_LEN];
	char mfg_info[GBMS_MINF_LEN];
	u8 *dev_info_check = batt_drv->dev_info_check;
	int ret, len;

	len = strlen(batt_drv->dev_sn);

	/* No dev_sn, return current state */
	if (len == 0)
		return batt_drv->pairing_state;

	ret = gbms_storage_read(GBMS_TAG_DINF, dev_info, GBMS_DINF_LEN);
	if (ret < 0) {
		pr_err("Read device pairing info failed, ret=%d\n", ret);
		return BATT_PAIRING_READ_ERROR;
	}

	if (batt_drv->dev_info_check[0] == 0) {
		char data[DEV_SN_LENGTH + GBMS_MINF_LEN];

		ret = gbms_storage_read(GBMS_TAG_MINF, mfg_info, GBMS_MINF_LEN);
		if (ret < 0) {
			pr_err("read mfg info. fail, ret=%d\n", ret);
			return BATT_PAIRING_READ_ERROR;
		}

		memcpy(data, batt_drv->dev_sn, len);
		memcpy(&data[len], mfg_info, GBMS_MINF_LEN);

		ret = batt_do_md5(data, len + GBMS_MINF_LEN, dev_info_check);
		if (ret < 0) {
			pr_err("execute batt_do_md5 fail, ret=%d\n", ret);
			return BATT_PAIRING_MISMATCH;
		}
	}

	/* new battery: pair the battery to this device */
	if (dev_info[0] == 0xFF) {

		ret = gbms_storage_write(GBMS_TAG_DINF, dev_info_check, GBMS_DINF_LEN);
		if (ret < 0) {
			pr_err("Pairing to this device failed, ret=%d\n", ret);
			return BATT_PAIRING_WRITE_ERROR;
		}

	/* recycled battery */
	} else if (strncmp(dev_info, dev_info_check, strlen(dev_info_check))) {
		pr_warn("Battery paired to a different device\n");

		return BATT_PAIRING_MISMATCH;
	}

	return BATT_PAIRING_PAIRED;
}

/*  TODO: handle history collection, use storage */
static int batt_hist_data_collect(void *h, int idx)
{
	int cnt;

	cnt = gbms_storage_read(GBMS_TAG_CLHI, h, 0);
	if (cnt > 0)
		cnt = gbms_storage_write_data(GBMS_TAG_HIST, h, cnt, idx);

	return cnt;
}

/* TODO: handle history collection, use storage */
static void batt_hist_free_data(void *p)
{
	if (p)
		kfree(p);
}

/* save data in hours */
#define SAVE_UNIT 3600
static void gbatt_save_sd(struct swelling_data *sd)
{
	u16 sd_saved[BATT_SD_SAVE_SIZE];
	bool update_save_data = false;
	int i, j, ret = 0;

	/* Change seconds to hours */
	for (i = 0; i < BATT_TEMP_RECORD_THR; i++) {
		j = i + SD_DISCHG_START;

		sd_saved[i] = ktime_divns(sd->chg[i], SAVE_UNIT) < BATT_SD_MAX_HOURS ?
			      ktime_divns(sd->chg[i], SAVE_UNIT) : BATT_SD_MAX_HOURS;

		sd_saved[j] = ktime_divns(sd->dischg[i], SAVE_UNIT) < BATT_SD_MAX_HOURS ?
			      ktime_divns(sd->dischg[i], SAVE_UNIT) : BATT_SD_MAX_HOURS;

		if (sd_saved[i] > sd->saved[i] ||
		    sd_saved[j] > sd->saved[j]) {
			sd->saved[i] = sd_saved[i];
			sd->saved[j] = sd_saved[j];
			update_save_data = true;
		}
	}

	if (!update_save_data)
		return;

	ret = gbms_storage_write(GBMS_TAG_STRD, &sd->saved, sizeof(sd->saved));
	if (ret < 0)
		pr_warn("Failed to save swelling data, ret=%d\n", ret);
}

static void gbatt_record_over_temp(struct batt_drv *batt_drv)
{
	struct swelling_data *sd = &batt_drv->sd;
	struct batt_ssoc_state *ssoc_state = &batt_drv->ssoc_state;
	const bool charge = batt_drv->fg_status == POWER_SUPPLY_STATUS_CHARGING ||
			    (batt_drv->fg_status == POWER_SUPPLY_STATUS_FULL &&
			    !batt_drv->chg_done);
	const int temp = batt_drv->batt_temp;
	const int soc = ssoc_get_real(ssoc_state);
	const ktime_t now = get_boot_sec();
	const ktime_t elap = now - sd->last_update;
	bool update_data = false;
	int i;

	for (i = 0; i < BATT_TEMP_RECORD_THR; i++) {
		/*
		 *  thresholds table:
		 *  | i        | 0      | 1      | 2      |
		 *  |----------|--------|--------|--------|
		 *  | temp_thr | 30degC | 35degC | 40degC |
		 *  | soc_thr  | 90%    | 90%    | 95%    |
		 */
		if (temp < sd->temp_thr[i] || soc < sd->soc_thr[i])
			continue;

		if (charge)
			sd->chg[i] += elap;
		else
			sd->dischg[i] += elap;

		update_data = true;
	}

	if (update_data)
		gbatt_save_sd(&batt_drv->sd);

	sd->last_update = now;
}

static int gbatt_save_capacity(struct batt_ssoc_state *ssoc_state)
{
	const int ui_soc = ssoc_get_capacity(ssoc_state);
	int ret = 0;

	if (!ssoc_state->save_soc_available)
		return ret;

	if (ssoc_state->save_soc != (u16)ui_soc) {
		ssoc_state->save_soc = (u16)ui_soc;
		ret = gbms_storage_write(GBMS_TAG_RSOC, &ssoc_state->save_soc,
					 sizeof(ssoc_state->save_soc));
	}

	return ret;
}

/* battery history data collection */
static int batt_history_data_work(struct batt_drv *batt_drv)
{
	int cycle_cnt, idx, ret;

	/* TODO: google_battery caches cycle count, should use that */
	cycle_cnt = GPSY_GET_PROP(batt_drv->fg_psy,
				  POWER_SUPPLY_PROP_CYCLE_COUNT);

	/* not update history if cycle count is not ready */
	if (cycle_cnt <= 0)
		return -EIO;

	/* update hist_data_saved_cnt when cycle count has been fixed */
	if (cycle_cnt < batt_drv->cycle_count) {
		batt_drv->cycle_count = cycle_cnt;
		batt_drv->hist_data_saved_cnt = cycle_cnt - 1;
	}

	if (batt_drv->blf_collect_now) {
		pr_info("MSC_HIST cycle_cnt:%d->%d saved_cnt=%d\n",
			cycle_cnt, batt_drv->blf_collect_now,
			batt_drv->hist_data_saved_cnt);

		cycle_cnt = batt_drv->blf_collect_now;
		batt_drv->hist_data_saved_cnt = cycle_cnt - 1;
		batt_drv->blf_collect_now = 0;
	}

	if (cycle_cnt <= batt_drv->hist_data_saved_cnt)
		return 0;

	idx = cycle_cnt / batt_drv->hist_delta_cycle_cnt;

	/* save in last when over max cycles */
	if (idx >= batt_drv->hist_data_max_cnt)
		idx = batt_drv->hist_data_max_cnt - 1;

	ret = batt_hist_data_collect(batt_drv->hist_data, idx);
	if (ret < 0)
		return ret;

	batt_drv->hist_data_saved_cnt = cycle_cnt;

	pr_info("MSC_HIST Update data with cnt:%d\n", cycle_cnt);

	return 0;
}

/* TODO: read from the HIST tag */
#define BATT_ONE_HIST_LEN	12

static int google_battery_init_hist_work(struct batt_drv *batt_drv )
{
	const int one_hist_len = BATT_ONE_HIST_LEN; /* TODO: read from the tag */
	int cnt;

	/*
	 * Determine the max number of history entries
	 * NOTE: gbms_storage will return -EPROBE_DEFER during init
	 */
	cnt = gbms_storage_read_data(GBMS_TAG_HIST, NULL, 0, 0);
	if (cnt == -EPROBE_DEFER)
		return -EAGAIN;

	if (cnt <= 0) {
		pr_err("MSC_HIST collect history data not available (%d)\n", cnt);
		batt_drv->blf_state = BATT_LFCOLLECT_NOT_AVAILABLE;
		return -ENODATA;
	}

	batt_drv->hist_data = kzalloc(one_hist_len, GFP_KERNEL);
	if (!batt_drv->hist_data) {
		pr_err("MSC_HIST cannot allocate buffer of size=%d\n",
		       one_hist_len);
		batt_drv->blf_state = BATT_LFCOLLECT_NOT_AVAILABLE;
	} else {
		batt_drv->blf_state = BATT_LFCOLLECT_COLLECT;
		batt_drv->hist_data_max_cnt = cnt;
		batt_drv->hist_data_saved_cnt = -1;
	}

	pr_info("MSC_HIST init_hist_work done, state:%d, cnt:%d",
		 batt_drv->blf_state, cnt);

	return 0;
}

#define TEMP_FILTER_DEFAULT_INTERVAL_MS 30000
#define TEMP_FILTER_FAST_INTERVAL_MS 3000
#define TEMP_FILTER_RESUME_DELAY_MS 1500
#define TEMP_FILTER_LOG_DIFF 50
static void batt_init_temp_filter(struct batt_drv *batt_drv)
{
	struct batt_temp_filter *temp_filter = &batt_drv->temp_filter;
	const struct device_node *node = batt_drv->device->of_node;
	u32 tmp;
	int ret;

	mutex_init(&batt_drv->temp_filter.lock);

	ret = of_property_read_u32(node, "google,temp-filter-default-interval", &tmp);
	if (ret == 0)
		temp_filter->default_interval = tmp;
	else
		temp_filter->default_interval = TEMP_FILTER_DEFAULT_INTERVAL_MS;

	ret = of_property_read_u32(node, "google,temp-filter-fast-interval", &tmp);
	if (ret == 0)
		temp_filter->fast_interval = tmp;
	else
		temp_filter->fast_interval = TEMP_FILTER_FAST_INTERVAL_MS;

	ret = of_property_read_u32(node, "google,temp-filter-resume-delay", &tmp);
	if (ret == 0)
		temp_filter->resume_delay_time = tmp;
	else
		temp_filter->resume_delay_time = TEMP_FILTER_RESUME_DELAY_MS;

	/* initial temperature value in first read data */
	temp_filter->force_update = true;
	mod_delayed_work(system_wq, &temp_filter->work, 0);

	pr_info("temperature filter: default:%ds, fast:%ds, resume:%dms\n",
		temp_filter->default_interval / 1000, temp_filter->fast_interval / 1000,
		temp_filter->resume_delay_time);
}

static void google_battery_temp_filter_work(struct work_struct *work)
{
	struct batt_drv *batt_drv = container_of(work, struct batt_drv, temp_filter.work.work);
	const union gbms_ce_adapter_details *ad = &batt_drv->ce_data.adapter_details;
	struct batt_temp_filter *temp_filter = &batt_drv->temp_filter;
	int interval = temp_filter->default_interval;
	union power_supply_propval val;
	int err = 0, i;

	if (!temp_filter->enable || interval == 0)
		return;

	if (!batt_drv->fg_psy)
		goto done;

	if (temp_filter->resume_delay) {
		interval = temp_filter->resume_delay_time; /* i2c might busy when resume */
		temp_filter->resume_delay = false;
		temp_filter->force_update = true;
		goto done;
	}

	if (ad->ad_type == CHG_EV_ADAPTER_TYPE_WLC ||
	    ad->ad_type == CHG_EV_ADAPTER_TYPE_WLC_EPP ||
	    ad->ad_type == CHG_EV_ADAPTER_TYPE_WLC_SPP)
		interval = temp_filter->fast_interval;

	err = power_supply_get_property(batt_drv->fg_psy, POWER_SUPPLY_PROP_TEMP, &val);
	if (err != 0)
		goto done;

	/* logging if big difference */
	if (abs(val.intval - temp_filter->sample[temp_filter->last_idx]) > TEMP_FILTER_LOG_DIFF)
		pr_info("temperature filter: [%d, %d, %d, %d, %d] val:%d idx:%d interval=%dms\n",
			temp_filter->sample[0], temp_filter->sample[1], temp_filter->sample[2],
			temp_filter->sample[3], temp_filter->sample[4], val.intval,
			temp_filter->last_idx, interval);

	mutex_lock(&temp_filter->lock);
	if (temp_filter->force_update) {
		temp_filter->force_update = false;
		for (i = 0; i < TEMP_SAMPLE_SIZE; i++)
			temp_filter->sample[i] = val.intval;
	} else {
		temp_filter->last_idx = (temp_filter->last_idx + 1) % TEMP_SAMPLE_SIZE;
		temp_filter->sample[temp_filter->last_idx] = val.intval;
	}
	mutex_unlock(&temp_filter->lock);

done:
	pr_debug("temperature filter: [%d, %d, %d, %d, %d] interval=%dms\n",
		 temp_filter->sample[0], temp_filter->sample[1], temp_filter->sample[2],
		 temp_filter->sample[3], temp_filter->sample[4], interval);
	mod_delayed_work(system_wq, &temp_filter->work, msecs_to_jiffies(interval));
}

#define BOOT_TO_OS_ATTEMPTS 3

static int batt_init_shutdown_flag(struct batt_drv *batt_drv)
{
	struct device_node *node = batt_drv->device->of_node;
	u8 data;
	int ret;

	if (of_property_read_bool(node, "google,shutdown-flag-disable"))
		return 0;

	ret = gbms_storage_read(GBMS_TAG_SUFG, &data, sizeof(data));
	if (ret < 0)
		return -EIO;

	batt_drv->boot_to_os_attempts = data;

	/* reset battery shutdown flag */
	data = 0;
	ret = gbms_storage_write(GBMS_TAG_SUFG, &data, sizeof(data));

	return (ret < 0) ? -EIO : 0;
}

static int batt_set_shutdown_flag(struct batt_drv *batt_drv)
{
	u8 data = batt_drv->boot_to_os_attempts;
	int ret;

	if (data == 0)
		data = BOOT_TO_OS_ATTEMPTS;

	ret = gbms_storage_write(GBMS_TAG_SUFG, &data, sizeof(data));

	return (ret < 0) ? -EIO : 0;
}

#define BOOT_TO_OS_ATTEMPTS_CLR_SOC 5

static void batt_clear_shutdown_flag(struct batt_drv *batt_drv)
{
	const struct batt_ssoc_state *ssoc_state = &batt_drv->ssoc_state;
	const int soc = ssoc_get_real(ssoc_state);

	if (ssoc_state->buck_enabled == 1 && soc >= BOOT_TO_OS_ATTEMPTS_CLR_SOC)
		batt_drv->boot_to_os_attempts = 0;
}

static int point_full_ui_soc_cb(struct gvotable_election *el,
			      const char *reason, void *vote)
{
	struct batt_drv *batt_drv = gvotable_get_data(el);
	struct batt_ssoc_state *ssoc_state = &batt_drv->ssoc_state;
	int ssoc = ssoc_get_capacity(ssoc_state);
	int soc = GVOTABLE_PTR_TO_INT(vote);

	if (ssoc_state->point_full_ui_soc == soc)
		return 0;

	dev_info(batt_drv->device, "update point_full_ui_soc: %d -> %d\n",
		 ssoc_state->point_full_ui_soc, soc);

	ssoc_state->point_full_ui_soc = soc;

	if (ssoc_state->point_full_ui_soc != DISABLE_POINT_FULL_UI_SOC &&
	    ssoc < SSOC_FULL && ssoc_state->buck_enabled == 1) {
		struct ssoc_uicurve *curve = ssoc_state->ssoc_curve;
		const qnum_t full = qnum_fromint(ssoc_state->point_full_ui_soc);

		ssoc_uicurve_splice_full(curve, full, ssoc_point_full);
		dump_ssoc_state(&batt_drv->ssoc_state, batt_drv->ssoc_log);
		ssoc_state->point_full_ui_soc = DISABLE_POINT_FULL_UI_SOC;
	}

	return 0;
}

static int batt_update_hist_work(struct batt_drv *batt_drv)
{
	int ret = 0;

	if (batt_drv->blf_state == BATT_LFCOLLECT_ENABLED) {

		ret = google_battery_init_hist_work(batt_drv);

		if (batt_drv->blf_state == BATT_LFCOLLECT_COLLECT) {
			ret = batt_history_data_work(batt_drv);
			if (ret < 0) {
				pr_err("BHI: cannot prime history (%d)\n", ret);
			} else {
				mutex_lock(&batt_drv->chg_lock);
				ret = batt_bhi_stats_update_all(batt_drv);
				if (ret < 0)
					pr_err("BHI: cannot init stats (%d)\n", ret);
				mutex_unlock(&batt_drv->chg_lock);
			}
		}
	}

	if (batt_drv->blf_state == BATT_LFCOLLECT_COLLECT) {
		ret = batt_history_data_work(batt_drv);
		if (ret == -ENOENT) {
			batt_drv->blf_state = BATT_LFCOLLECT_NOT_AVAILABLE;
			pr_info("MSC_HIST Battery data collection disabled\n");
		}  else if (ret < 0) {
			pr_debug("MSC_HIST cannot collect battery data %d\n", ret);
		}
	}

	return ret;
}

/*
 * poll the battery, run SOC%, dead battery, critical.
 * scheduled from psy_changed and from timer
 */

#define UPDATE_INTERVAL_AT_FULL_FACTOR	4

static void google_battery_work(struct work_struct *work)
{
	struct batt_drv *batt_drv =
	    container_of(work, struct batt_drv, batt_work.work);
	struct power_supply *fg_psy = batt_drv->fg_psy;
	struct batt_ssoc_state *ssoc_state = &batt_drv->ssoc_state;
	int update_interval = batt_drv->batt_update_interval;
	const int prev_ssoc = ssoc_get_capacity(ssoc_state);
	int present, fg_status, batt_temp, ret;
	bool notify_psy_changed = false;
	bool update_last_full_charge = false;

	pr_debug("battery work item\n");

	pm_runtime_get_sync(batt_drv->device);
	if (!batt_drv->resume_complete) {
		schedule_delayed_work(&batt_drv->batt_work, msecs_to_jiffies(100));
		pm_runtime_put_sync(batt_drv->device);
		return;
	}
	pm_runtime_put_sync(batt_drv->device);

	__pm_stay_awake(batt_drv->batt_ws);

	/* chg_lock protect msc_logic */
	mutex_lock(&batt_drv->chg_lock);

	present = GPSY_GET_PROP(fg_psy, POWER_SUPPLY_PROP_PRESENT);
	if (present && !batt_drv->batt_present) {
		pr_debug("%s: change of battery state %d->%d\n",
			 __func__, batt_drv->batt_present, present);

		batt_drv->batt_present = true;
		notify_psy_changed = true;
	} else if (!present && batt_drv->batt_present) {
		pr_debug("%s: change of battery state %d->%d\n",
			 __func__, batt_drv->batt_present, present);

		batt_drv->batt_present = false;

		/* add debounce? */
		notify_psy_changed = true;
		mutex_unlock(&batt_drv->chg_lock);
		goto reschedule;
	}

	fg_status = GPSY_GET_INT_PROP(fg_psy, POWER_SUPPLY_PROP_STATUS, &ret);
	if (ret < 0) {
		mutex_unlock(&batt_drv->chg_lock);
		goto reschedule;
	}

	/* batt_lock protect SSOC code etc. */
	mutex_lock(&batt_drv->batt_lock);

	/* TODO: poll rate should be min between ->batt_update_interval and
	 * whatever ssoc_work() decides (typically rls->rl_delta_max_time)
	 */
	ret = ssoc_work(ssoc_state, fg_psy);
	if (ret < 0) {
		update_interval = BATT_WORK_ERROR_RETRY_MS;
	} else {
		bool full;
		int ssoc, level;

		/* handle charge/recharge */
		batt_rl_update_status(batt_drv);

		ssoc = ssoc_get_capacity(ssoc_state);
		if (prev_ssoc != ssoc) {
			pr_debug("%s: change of ssoc %d->%d\n", __func__,
				 prev_ssoc, ssoc);

			dump_ssoc_state(ssoc_state, batt_drv->ssoc_log);
			batt_log_csi_ttf_info(batt_drv);
			notify_psy_changed = true;
		}

		/* TODO(b/138860602): clear ->chg_done to enforce the
		 * same behavior during the transition 99 -> 100 -> Full
		 */

		level = gbatt_get_capacity_level(batt_drv, fg_status);
		if (level != batt_drv->capacity_level) {
			pr_debug("%s: change of capacity level %d->%d\n",
				 __func__, batt_drv->capacity_level,
				 level);

			/* set battery critical shutdown */
			if (level == POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL) {
				ret = batt_set_shutdown_flag(batt_drv);
				if (ret < 0)
					pr_warn("failed to write shutdown flag, ret=%d\n", ret);
			}

			batt_drv->capacity_level = level;
			notify_psy_changed = true;
		}

		if (batt_drv->boot_to_os_attempts > 0)
			batt_clear_shutdown_flag(batt_drv);

		if (batt_drv->dead_battery) {
			batt_drv->dead_battery = gbatt_check_dead_battery(batt_drv);
			if (!batt_drv->dead_battery) {
				pr_debug("%s: dead_battery 1->0\n", __func__);
				notify_psy_changed = true;
			}
		}

		/* fuel gauge triggered recharge logic. */
		full = (ssoc == SSOC_FULL);
		if (full && !batt_drv->batt_full) {
			batt_log_csi_ttf_info(batt_drv);
			batt_chg_stats_pub(batt_drv, "100%", false, true);
		}
		batt_drv->batt_full = full;

		/* update resistance all the time and capacity on disconnect */
		ret = bhi_imp_data_update(&batt_drv->health_data.bhi_data, fg_psy);
		if (ret < 0 && ret != -ENODATA)
			pr_warn("cannot update perf index ret=%d\n", ret);

		/* restore SSOC after reboot */
		ret = gbatt_save_capacity(&batt_drv->ssoc_state);
		if (ret < 0)
			pr_warn("write save_soc fail, ret=%d\n", ret);

		/* debounce fg_status changes at 100% */
		if (fg_status != batt_drv->fg_status) {

			pr_debug("%s: ssoc=%d full=%d change of fg_status %d->%d\n",
				 __func__, ssoc, full, batt_drv->fg_status, fg_status);
			if (!full)
				notify_psy_changed = true;
		}

		/* slow down the updates at full */
		if (full && batt_drv->chg_done) {
			update_interval *= UPDATE_INTERVAL_AT_FULL_FACTOR;
			update_last_full_charge = true;
		}
	}

	/* notifications for this are debounced  */
	batt_drv->fg_status = fg_status;

	/* TODO: poll other data here if needed */

	ret = gbatt_get_temp(batt_drv, &batt_temp);
	if (ret == 0 && batt_temp != batt_drv->batt_temp) {
		const int limit = batt_drv->batt_update_high_temp_threshold;

		batt_drv->batt_temp = batt_temp;
		if (batt_drv->batt_temp > limit) {
			pr_debug("%s: temperature over limit %d > %d\n",
				 __func__, batt_temp, limit);
			notify_psy_changed = true;
		}
	}

	if (batt_drv->sd.is_enable)
		gbatt_record_over_temp(batt_drv);

	mutex_unlock(&batt_drv->batt_lock);

	/*
	 * wait for timeout or state equal to CHARGING, FULL or UNKNOWN
	 * (which will likely not happen) even on ssoc error. msc_logic
	 * hold poll_ws wakelock during this time.
	 * Delay the estimates for time to full for BATT_WORK_DEBOUNCE_RETRY_MS
	 * after the device start charging.
	 */
	if (batt_drv->batt_fast_update_cnt) {

		if (fg_status != POWER_SUPPLY_STATUS_DISCHARGING &&
		    fg_status != POWER_SUPPLY_STATUS_NOT_CHARGING) {
			batt_drv->batt_fast_update_cnt = 0;
			update_interval = BATT_WORK_DEBOUNCE_RETRY_MS;
		} else {
			update_interval = BATT_WORK_FAST_RETRY_MS;
			batt_drv->batt_fast_update_cnt -= 1;
		}
	} else if (batt_drv->ttf_debounce) {
		batt_drv->ttf_debounce = 0;
		batt_log_csi_ttf_info(batt_drv);
	}

	/* acquired in msc_logic */
	if (batt_drv->batt_fast_update_cnt == 0)
		__pm_relax(batt_drv->poll_ws);

	/* set a connect */
	if (batt_drv->health_data.bhi_data.res_state.estimate_requested)
		batt_res_work(batt_drv);

	/* check only once and when/if the pairing state is reset */
	if (batt_drv->pairing_state == BATT_PAIRING_ENABLED) {
		enum batt_paired_state state;

		state = batt_check_pairing_state(batt_drv);
		switch (state) {
		/* somethig is wrong with eeprom comms, HW problem? */
		case BATT_PAIRING_READ_ERROR:
		case BATT_PAIRING_WRITE_ERROR:
			if (batt_drv->pairing_state_retry_cnt > 0)
				batt_drv->pairing_state_retry_cnt -= 1;
			else
				batt_drv->pairing_state = state;
			break;
		default:
			batt_drv->pairing_state = state;
			break;
		}
	}

	/* check recalibration conditions */
	ret = batt_bhi_update_recalibration_status(batt_drv);
	if (ret < 0)
		pr_err("bhi update recalibration not available (%d)\n", ret);

	mutex_unlock(&batt_drv->chg_lock);

	/* TODO: we might not need to do this all the time */
	batt_cycle_count_update(batt_drv, ssoc_get_real(ssoc_state));

	if (update_last_full_charge) {
		batt_drv->last_full_charge = batt_drv->hist_data_saved_cnt;
		ret = gbms_storage_write(GBMS_TAG_FCRU, &batt_drv->last_full_charge,
					 GBMS_FCRU_LEN);
		if (ret < 0)
			pr_err("failed to store FCNU (%d)\n", ret);

		dev_info(batt_drv->device, "force full charged at cycle %d\n",
			 batt_drv->last_full_charge);

		batt_force_fcr_update_charging_policy(batt_drv, false);
	}

reschedule:

	if (notify_psy_changed)
		power_supply_changed(batt_drv->psy);

	if (batt_drv->blf_state != BATT_LFCOLLECT_NOT_AVAILABLE) {
		ret = batt_update_hist_work(batt_drv);
		if (ret == -EAGAIN)
			update_interval = BATT_WORK_DEBOUNCE_RETRY_MS;
	}

	if (update_interval) {
		pr_debug("rerun battery work in %d ms\n", update_interval);
		schedule_delayed_work(&batt_drv->batt_work,
				      msecs_to_jiffies(update_interval));
	}


	__pm_relax(batt_drv->batt_ws);
}

static void power_metrics_data_work(struct work_struct *work)
{
	struct batt_drv *batt_drv = container_of(work, struct batt_drv,
						 power_metrics.work.work);
	const unsigned int idx = batt_drv->power_metrics.idx;
	unsigned long cc, vbat;
	unsigned int next_work = batt_drv->power_metrics.polling_rate * 1000;
	ktime_t now = get_boot_sec();

	if (!batt_drv->fg_psy)
		goto error;

	pm_runtime_get_sync(batt_drv->device);
	if (!batt_drv->resume_complete) {
		next_work = 100;
		pm_runtime_put_sync(batt_drv->device);
		goto error;
	}
	pm_runtime_put_sync(batt_drv->device);

	cc = GPSY_GET_PROP(batt_drv->fg_psy, POWER_SUPPLY_PROP_CHARGE_COUNTER);
	vbat = GPSY_GET_PROP(batt_drv->fg_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW);

	if ((cc < 0) || (vbat < 0)) {
		if ((cc == -EAGAIN) || (vbat == -EAGAIN))
			next_work = 100;
		goto error;
	}

	if ((idx == 0) && (batt_drv->power_metrics.data[idx].voltage == 0))
		batt_drv->power_metrics.idx = 0;
	else
		batt_drv->power_metrics.idx++;
	if (batt_drv->power_metrics.idx >= POWER_METRICS_MAX_DATA)
		batt_drv->power_metrics.idx = 0;

	batt_drv->power_metrics.data[batt_drv->power_metrics.idx].charge_count = cc;
	batt_drv->power_metrics.data[batt_drv->power_metrics.idx].voltage = vbat;
	batt_drv->power_metrics.data[batt_drv->power_metrics.idx].time = now;

error:
	schedule_delayed_work(&batt_drv->power_metrics.work, msecs_to_jiffies(next_work));
}

/* ------------------------------------------------------------------------- */

/*
 * Keep the number of properties under UEVENT_NUM_ENVP (minus # of
 * standard uevent variables) i.e 26.
 *
 * Removed the following from sysnodes
 * GBMS_PROP_ADAPTER_DETAILS			gbms
 * POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT	gbms
 * POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE	gbms
 *
 * POWER_SUPPLY_PROP_CHARGE_TYPE,
 * POWER_SUPPLY_PROP_CURRENT_AVG,
 * POWER_SUPPLY_PROP_VOLTAGE_AVG,
 * POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
 * POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
 * POWER_SUPPLY_PROP_VOLTAGE_OCV,
 */

static enum power_supply_property gbatt_battery_props[] = {
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_SERIAL_NUMBER,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,	/* No need for this? */
	POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,		/* 23 */
	POWER_SUPPLY_PROP_VOLTAGE_OCV,

	/*  hard limit to 26 */
};

static bool temp_defend_dry_run(struct gvotable_election *temp_dryrun_votable)
{
	bool dry_run = 1;

	if (!temp_dryrun_votable)
		temp_dryrun_votable =
			gvotable_election_get_handle(VOTABLE_TEMP_DRYRUN);
	if (temp_dryrun_votable)
		dry_run = !!gvotable_get_current_int_vote(temp_dryrun_votable);

	return dry_run;
}

/*
 * status is:
 * . _UNKNOWN during init
 * . _DISCHARGING when not connected
 * when connected to a power supply status is
 * . _FULL (until disconnect) after the charger flags DONE if SSOC=100%
 * . _CHARGING if FG reports _FULL but SSOC < 100% (should not happen)
 * . _CHARGING if FG reports _NOT_CHARGING
 * . _NOT_CHARGING if FG report _DISCHARGING
 * . same as FG state otherwise
 */
static int gbatt_get_status(struct batt_drv *batt_drv,
			    union power_supply_propval *val)
{
	int err, ssoc;

	if (batt_drv->ssoc_state.buck_enabled == 0) {
		if (batt_drv->chg_state.f.chg_status == POWER_SUPPLY_STATUS_NOT_CHARGING)
			val->intval = batt_drv->chg_state.f.chg_status;
		else
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		return 0;
	}

	if (batt_drv->ssoc_state.buck_enabled == -1) {
		val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
		return 0;
	}

	/* ->buck_enabled = 1, from here ownward device is connected */

	if (batt_drv->batt_health == POWER_SUPPLY_HEALTH_OVERHEAT &&
	    !temp_defend_dry_run(batt_drv->temp_dryrun_votable)) {
		val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		return 0;
	}

	if (batt_drv->batt_fast_update_cnt) {
		val->intval = POWER_SUPPLY_STATUS_CHARGING;
		return 0;
	}

	if (batt_drv->msc_state == MSC_HEALTH_PAUSE) {
		/* Expect AC to discharge in PAUSE. However, UI must persist */
		val->intval = POWER_SUPPLY_STATUS_CHARGING;
		return 0;
	}

	if (!batt_drv->fg_psy)
		return -EINVAL;

	ssoc = ssoc_get_capacity(&batt_drv->ssoc_state);

	/* FULL when the charger said so and SSOC == 100% */
	if (batt_drv->chg_done && ssoc == SSOC_FULL) {
		val->intval = POWER_SUPPLY_STATUS_FULL;
		return 0;
	}

	err = power_supply_get_property(batt_drv->fg_psy,
					POWER_SUPPLY_PROP_STATUS,
					val);
	if (err != 0)
		return err;

	if (val->intval == POWER_SUPPLY_STATUS_FULL) {

		/* not full unless the charger says so */
		if (!batt_drv->chg_done)
			val->intval = POWER_SUPPLY_STATUS_CHARGING;

		/* NOTE: FG driver could flag FULL before GDF is at 100% when
		 * gauge is not tuned or when capacity estimates are wrong.
		 */
		if (ssoc != SSOC_FULL)
			val->intval = POWER_SUPPLY_STATUS_CHARGING;

	} else if (val->intval == POWER_SUPPLY_STATUS_NOT_CHARGING) {
		/* smooth transition between charging and full */
		val->intval = POWER_SUPPLY_STATUS_CHARGING;
	} else if (val->intval == POWER_SUPPLY_STATUS_DISCHARGING) {
		/* connected and discharging is NOT charging */
		val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
	}

	return 0;
}

/* lock batt_drv->batt_lock */
static int gbatt_get_capacity(struct batt_drv *batt_drv)
{
	struct batt_ssoc_state *ssoc_state = &batt_drv->ssoc_state;
	int capacity;

	if (batt_drv->fake_capacity >= 0 && batt_drv->fake_capacity <= 100)
		capacity = batt_drv->fake_capacity;
	else
		capacity = ssoc_get_capacity(ssoc_state);

	return capacity;
}


static void gbatt_reset_curve(struct batt_drv *batt_drv, int ssoc_cap)
{
	struct batt_ssoc_state *ssoc_state = &batt_drv->ssoc_state;
	const qnum_t cap = qnum_fromint(ssoc_cap);
	const qnum_t gdf = ssoc_state->ssoc_gdf;
	enum ssoc_uic_type type;

	if (gdf < cap) {
		type = SSOC_UIC_TYPE_DSG;
	} else {
		type = SSOC_UIC_TYPE_CHG;
	}

	pr_info("reset curve at gdf=%d.%d cap=%d.%d type=%d\n",
		qnum_toint(gdf), qnum_fracdgt(gdf),
		qnum_toint(cap), qnum_fracdgt(cap),
		type);

	/* current is the drop point on the discharge curve */
	ssoc_change_curve_at_gdf(ssoc_state, gdf, cap, type);
	ssoc_work(ssoc_state, batt_drv->fg_psy);
	dump_ssoc_state(ssoc_state, batt_drv->ssoc_log);
}

/* splice the curve at point when the SSOC is removed */
static void gbatt_set_capacity(struct batt_drv *batt_drv, int capacity)
{
	if (capacity < 0)
		capacity = -EINVAL;

	if (batt_drv->batt_health != POWER_SUPPLY_HEALTH_OVERHEAT) {
		/* just set the value if not in overheat  */
	} else if (capacity < 0 && batt_drv->fake_capacity >= 0) {
		gbatt_reset_curve(batt_drv, batt_drv->fake_capacity);
	} else if (capacity > 0) {
		/* TODO: convergence to the new capacity? */
	}

	batt_drv->fake_capacity = capacity;
}

static int gbatt_set_health(struct batt_drv *batt_drv, int health)
{
	if (health > POWER_SUPPLY_HEALTH_HOT ||
	    health < POWER_SUPPLY_HEALTH_UNKNOWN)
		return -EINVAL;

	batt_drv->batt_health = health;

	/* disable health charging if in overheat */
	if (health == POWER_SUPPLY_HEALTH_OVERHEAT)
		msc_logic_health(batt_drv);

	return 0;
}

#define RESTORE_SOC_LOWER_THRESHOLD	2
#define RESTORE_SOC_UPPER_THRESHOLD	5
static int gbatt_restore_capacity(struct batt_drv *batt_drv)
{
	struct batt_ssoc_state *ssoc_state = &batt_drv->ssoc_state;
	int ret = 0, save_soc, gdf_soc, delta;

	ret = gbms_storage_read(GBMS_TAG_RSOC, &ssoc_state->save_soc,
						sizeof(ssoc_state->save_soc));
	if (ret < 0)
		return ret;

	save_soc = (int)ssoc_state->save_soc;
	gdf_soc = qnum_toint(ssoc_state->ssoc_gdf);
	delta = save_soc - gdf_soc;

	dev_info(batt_drv->device, "save_soc:%d, gdf:%d\n", save_soc, gdf_soc);

	if (delta >= RESTORE_SOC_LOWER_THRESHOLD && delta <= RESTORE_SOC_UPPER_THRESHOLD)
		gbatt_reset_curve(batt_drv, save_soc);

	return ret;
}

static int gbatt_get_health(struct batt_drv *batt_drv)
{
	int charging_state = batt_get_charging_state(batt_drv);
	int health = POWER_SUPPLY_HEALTH_UNKNOWN;

	switch (charging_state) {
	case BATTERY_STATUS_NORMAL:
	case BATTERY_STATUS_LONGLIFE:
	case BATTERY_STATUS_ADAPTIVE:
		health = POWER_SUPPLY_HEALTH_GOOD;
		break;
	case BATTERY_STATUS_TOO_COLD:
		health = POWER_SUPPLY_HEALTH_COLD;
		break;
	case BATTERY_STATUS_TOO_HOT:
		health = POWER_SUPPLY_HEALTH_HOT;
		break;
	default:
		break;
	}

	return health;
}

static int gbatt_get_ttf(struct batt_drv *batt_drv)
{
	const int report_ratio = batt_drv->ttf_stats.report_max_ratio;
	union power_supply_propval val;
	int max_ratio, rc;
	ktime_t res;

	/* report deadline when AC enable */
	if (batt_drv->chg_health.rest_deadline > 0) {
		const ktime_t now = get_boot_sec();
		const ktime_t ac_end = batt_drv->chg_health.rest_deadline - now;

		return ac_end > 0 ? ac_end : 0;
	}

	max_ratio = batt_ttf_estimate(&res, batt_drv);
	/* always report when report_max_ratio is 0 */
	if (report_ratio && max_ratio >= report_ratio)
		return 0;

	if (max_ratio >= 0)
		return res < 0 ? 0 : res;

	if (!batt_drv->fg_psy)
		return -1;

	rc = power_supply_get_property(batt_drv->fg_psy, POWER_SUPPLY_PROP_TIME_TO_FULL_NOW, &val);
	return rc < 0 ? -1 : val.intval;
}

static int gbatt_get_property(struct power_supply *psy,
				 enum power_supply_property psp,
				 union power_supply_propval *val)
{
	struct batt_drv *batt_drv = (struct batt_drv *)
					power_supply_get_drvdata(psy);
	int rc, err = 0;

	pm_runtime_get_sync(batt_drv->device);
	if (!batt_drv->init_complete || !batt_drv->resume_complete) {
		pm_runtime_put_sync(batt_drv->device);
		return -EAGAIN;
	}
	pm_runtime_put_sync(batt_drv->device);

	switch (psp) {
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		if (batt_drv->cycle_count < 0)
			err = batt_drv->cycle_count;
		else
			val->intval = batt_drv->cycle_count;
		break;

	case POWER_SUPPLY_PROP_CAPACITY:
		mutex_lock(&batt_drv->batt_lock);
		val->intval = gbatt_get_capacity(batt_drv);
		mutex_unlock(&batt_drv->batt_lock);
		break;

	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		if (batt_drv->fake_capacity >= 0 &&
				batt_drv->fake_capacity <= 100)
			val->intval = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
		else
			val->intval = batt_drv->capacity_level;
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		mutex_lock(&batt_drv->chg_lock);
		val->intval = batt_drv->cc_max;
		mutex_unlock(&batt_drv->chg_lock);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		mutex_lock(&batt_drv->chg_lock);
		val->intval = batt_drv->fv_uv;
		mutex_unlock(&batt_drv->chg_lock);
		break;

	/*
	 * POWER_SUPPLY_PROP_CHARGE_DONE comes from the charger BUT battery
	 * has also an idea about it.
	 *	mutex_lock(&batt_drv->chg_lock);
	 *	val->intval = batt_drv->chg_done;
	 *	mutex_unlock(&batt_drv->chg_lock);
	 */

	/*
	 * compat: POWER_SUPPLY_PROP_CHARGE_TYPE comes from the charger so
	 * using the last value reported from the CHARGER. This (of course)
	 * means that NG charging needs to be enabled.
	 */
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		mutex_lock(&batt_drv->chg_lock);
		val->intval = batt_drv->chg_state.f.chg_type;
		mutex_unlock(&batt_drv->chg_lock);
		break;

	case POWER_SUPPLY_PROP_STATUS:
		err = gbatt_get_status(batt_drv, val);
		break;

	/* health */
	case POWER_SUPPLY_PROP_HEALTH:
		if (batt_drv->batt_health == POWER_SUPPLY_HEALTH_OVERHEAT &&
		    temp_defend_dry_run(batt_drv->temp_dryrun_votable)) {
			val->intval = POWER_SUPPLY_HEALTH_GOOD;
		} else if (batt_drv->batt_health != POWER_SUPPLY_HEALTH_UNKNOWN) {
			val->intval = batt_drv->batt_health;
		} else if (batt_drv->health_data.cal_state == REC_STATE_SCHEDULED) {
			val->intval = POWER_SUPPLY_HEALTH_CALIBRATION_REQUIRED;
		} else if (!batt_drv->fg_psy) {
			val->intval = POWER_SUPPLY_HEALTH_UNKNOWN;
		} else {
			rc = gbatt_get_health(batt_drv);
			val->intval = rc < 0 ? POWER_SUPPLY_HEALTH_UNKNOWN : rc;
			batt_drv->soh = val->intval;
		}
		if (batt_drv->report_health != val->intval) {
			/* Log health change for debug */
			logbuffer_log(batt_drv->ttf_stats.ttf_log,
				      "h:%d->%d batt_health:%d dry_run:%d soh:%d cal_state:%d",
				      batt_drv->report_health, val->intval, batt_drv->batt_health,
				      temp_defend_dry_run(batt_drv->temp_dryrun_votable),
				      batt_drv->soh, batt_drv->health_data.cal_state);
			batt_drv->report_health = val->intval;
		}
		break;

	/* cannot set err, negative estimate will revert to HAL */
	case POWER_SUPPLY_PROP_TIME_TO_FULL_NOW:
		val->intval = gbatt_get_ttf(batt_drv);
		break;

	case POWER_SUPPLY_PROP_TEMP:
		err = gbatt_get_temp(batt_drv, &val->intval);
		break;

	case POWER_SUPPLY_PROP_CURRENT_AVG:
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		if (!batt_drv->fg_psy)
			return -EINVAL;
		err = power_supply_get_property(batt_drv->fg_psy, psp, val);
		if (err == 0)
			val->intval = -val->intval;
		break;

	/* Can force the state here */
	case POWER_SUPPLY_PROP_PRESENT:
		if (batt_drv->fake_battery_present != -1) {
			val->intval = batt_drv->fake_battery_present;
		} else if (batt_drv->fg_psy) {

			/* TODO: use the cached value? */
			rc = power_supply_get_property(batt_drv->fg_psy,
						       psp, val);
			if (rc < 0)
				val->intval = 0;
		} else {
			err = -EINVAL;
		}
		break;

	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		if (batt_drv->topoff)
			val->intval = batt_drv->topoff;
		else
			val->intval = -1;
		break;

	default:
		if (!batt_drv->fg_psy)
			return -EINVAL;
		err = power_supply_get_property(batt_drv->fg_psy, psp, val);
		break;
	}

	if (err < 0) {
		pr_debug("gbatt: get_prop cannot read psp=%d\n", psp);
		return err;
	}

	return 0;
}

static int gbatt_set_property(struct power_supply *psy,
				 enum power_supply_property psp,
				 const union power_supply_propval *val)
{
	struct batt_drv *batt_drv = (struct batt_drv *)
					power_supply_get_drvdata(psy);
	int ret = 0;

	pm_runtime_get_sync(batt_drv->device);
	if (!batt_drv->init_complete || !batt_drv->resume_complete) {
		pm_runtime_put_sync(batt_drv->device);
		return -EAGAIN;
	}
	pm_runtime_put_sync(batt_drv->device);


	switch (psp) {
	case POWER_SUPPLY_PROP_CAPACITY:
		mutex_lock(&batt_drv->chg_lock);
		if (val->intval != batt_drv->fake_capacity) {
			gbatt_set_capacity(batt_drv, val->intval);
			if (batt_drv->psy)
				power_supply_changed(batt_drv->psy);
		}
		mutex_unlock(&batt_drv->chg_lock);
		break;

	case POWER_SUPPLY_PROP_TIME_TO_FULL_NOW:
		if (val->intval <= 0)
			batt_drv->ttf_stats.ttf_fake = -1;
		else
			batt_drv->ttf_stats.ttf_fake = val->intval;
		pr_info("time_to_full = %lld\n", batt_drv->ttf_stats.ttf_fake);
		if (batt_drv->psy)
			power_supply_changed(batt_drv->psy);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		mutex_lock(&batt_drv->chg_lock);
		if (batt_drv->batt_health != val->intval) {
			ret = gbatt_set_health(batt_drv, val->intval);
			if (ret == 0 && batt_drv->psy)
				power_supply_changed(batt_drv->psy);
		}
		mutex_unlock(&batt_drv->chg_lock);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	if (ret < 0) {
		pr_debug("gbatt: get_prop cannot write psp=%d\n", psp);
		return ret;
	}


	return 0;
}

static int gbatt_property_is_writeable(struct power_supply *psy,
					  enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_CAPACITY:
	case POWER_SUPPLY_PROP_TIME_TO_FULL_NOW:
	case POWER_SUPPLY_PROP_HEALTH:
		return 1;
	default:
		break;
	}

	return 0;
}

static int gbatt_gbms_get_property(struct power_supply *psy,
				   enum gbms_property psp,
				   union gbms_propval *val)
{
	struct batt_drv *batt_drv = (struct batt_drv *)
					power_supply_get_drvdata(psy);
	int err = 0;

	pm_runtime_get_sync(batt_drv->device);
	if (!batt_drv->init_complete || !batt_drv->resume_complete) {
		pm_runtime_put_sync(batt_drv->device);
		return -EAGAIN;
	}
	pm_runtime_put_sync(batt_drv->device);

	switch (psp) {
	case GBMS_PROP_ADAPTER_DETAILS:
		val->prop.intval = batt_drv->ce_data.adapter_details.v;
		break;

	case GBMS_PROP_DEAD_BATTERY:
		val->prop.intval = batt_drv->dead_battery;
		break;
	/*
	 * ng charging:
	 * 1) write to GBMS_PROP_CHARGE_CHARGER_STATE,
	 * 2) read POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT and
	 *    POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE
	 */
	case GBMS_PROP_CHARGE_CHARGER_STATE:
		val->int64val = batt_drv->chg_state.v;
		break;

	default:
		if (!batt_drv->fg_psy)
			return -EINVAL;
		if (psp >= GBMS_PROP_LOCAL_EXTENSIONS) {
			val->prop.intval = GPSY_GET_INT_PROP(batt_drv->fg_psy, psp, &err);
		} else {
			pr_debug("%s: route to gbatt_get_property, psp:%d\n", __func__, psp);
			return -ENODATA;
		}
		break;
	}

	if (err < 0) {
		pr_debug("gbatt: get_prop cannot read psp=%d\n", psp);
		return err;
	}

	return 0;
}

static int gbatt_gbms_set_property(struct power_supply *psy,
				   enum gbms_property psp,
				   const union gbms_propval *val)
{
	struct batt_drv *batt_drv = (struct batt_drv *)
					power_supply_get_drvdata(psy);
	int ret = 0;

	pm_runtime_get_sync(batt_drv->device);
	if (!batt_drv->init_complete || !batt_drv->resume_complete) {
		pm_runtime_put_sync(batt_drv->device);
		return -EAGAIN;
	}
	pm_runtime_put_sync(batt_drv->device);

	switch (psp) {
	case GBMS_PROP_ADAPTER_DETAILS:
		mutex_lock(&batt_drv->stats_lock);
		batt_drv->ce_data.adapter_details.v = val->prop.intval;
		mutex_unlock(&batt_drv->stats_lock);
	break;

	/* NG Charging, where it all begins */
	case GBMS_PROP_CHARGE_CHARGER_STATE:
		mutex_lock(&batt_drv->chg_lock);
		batt_drv->chg_state.v = val->int64val;
		ret = batt_chg_logic(batt_drv);
		mutex_unlock(&batt_drv->chg_lock);
		break;

	case GBMS_PROP_LOGBUFFER_BD:
		batt_drv->bd_log = (struct logbuffer *)val->int64val;
		gbms_logbuffer_prlog(batt_drv->bd_log, LOGLEVEL_INFO, 0, LOGLEVEL_INFO,
				     "AACP: set logbuffer_bd addr");
		break;

	default:
		pr_debug("%s: route to gbatt_set_property, psp:%d\n", __func__, psp);
		return -ENODATA;
	}

	if (ret < 0) {
		pr_debug("gbatt: get_prop cannot write psp=%d\n", psp);
		return ret;
	}

	return 0;
}

static int gbatt_gbms_property_is_writeable(struct power_supply *psy,
					    enum gbms_property psp)
{
	switch (psp) {
	case GBMS_PROP_CHARGE_CHARGER_STATE:
	case POWER_SUPPLY_PROP_CAPACITY:
	case GBMS_PROP_ADAPTER_DETAILS:
	case POWER_SUPPLY_PROP_TIME_TO_FULL_NOW:
	case POWER_SUPPLY_PROP_HEALTH:
	case GBMS_PROP_LOGBUFFER_BD:
		return 1;
	default:
		break;
	}

	return 0;
}

static struct gbms_desc gbatt_psy_desc = {
	.psy_dsc.name = "battery",
	.psy_dsc.type = POWER_SUPPLY_TYPE_BATTERY,
	.psy_dsc.get_property = gbatt_get_property,
	.psy_dsc.set_property = gbatt_set_property,
	.psy_dsc.property_is_writeable = gbatt_property_is_writeable,
	.get_property = gbatt_gbms_get_property,
	.set_property = gbatt_gbms_set_property,
	.property_is_writeable = gbatt_gbms_property_is_writeable,
	.psy_dsc.properties = gbatt_battery_props,
	.psy_dsc.num_properties = ARRAY_SIZE(gbatt_battery_props),
	.forward = true,
};

/* ------------------------------------------------------------------------ */

static int batt_init_sd(struct swelling_data *sd)
{
	int ret, i, j;

	if (!sd->is_enable)
		return 0;

	ret = gbms_storage_read(GBMS_TAG_STRD, &sd->saved,
				sizeof(sd->saved));
	if (ret < 0)
		return ret;

	if (sd->saved[SD_CHG_START] == 0xFFFF) {
		/* Empty EEPROM, initial sd_saved */
		for (i = 0; i < BATT_SD_SAVE_SIZE; i++)
			sd->saved[i] = 0;
	} else {
		/* Available data, restore */
		for (i = 0; i < BATT_TEMP_RECORD_THR; i++) {
			j = i + SD_DISCHG_START;
			sd->chg[i] = sd->saved[i] * SAVE_UNIT;
			sd->dischg[i] = sd->saved[j] * SAVE_UNIT;
		}
	}

	return ret;
}

/* bhi_init */
static int batt_bhi_init(struct batt_drv *batt_drv)
{
	struct health_data *health_data = &batt_drv->health_data;
	struct bhi_data *bhi_data = &health_data->bhi_data;
	u16 capacity_boundary[BHI_TREND_POINTS_SIZE];
	int ret, i;

	/* set upper_bound value to BHI_CAPACITY_MAX(0xFFFF) */
	memset(bhi_data->upper_bound.limit, 0xFF, sizeof(bhi_data->upper_bound.limit));
	memset(bhi_data->upper_bound.trigger, 0xFF, sizeof(bhi_data->upper_bound.trigger));

	/* see enum bhi_algo */
	ret = of_property_read_u32(batt_drv->device->of_node, "google,bhi-algo-ver",
				   &health_data->bhi_algo);
	if (ret < 0)
		health_data->bhi_algo = BHI_ALGO_DISABLED;
	/* default weights */
	ret = of_property_read_u32(batt_drv->device->of_node, "google,bhi-w_ci",
				   &health_data->bhi_w_ci);
	if (ret < 0)
		health_data->bhi_w_ci = 100;
	ret = of_property_read_u32(batt_drv->device->of_node, "google,bhi-w_pi",
				   &health_data->bhi_w_pi);
	if (ret < 0)
		health_data->bhi_w_pi = 0;
	ret = of_property_read_u32(batt_drv->device->of_node, "google,bhi-w_sd",
				   &health_data->bhi_w_sd);
	if (ret < 0)
		health_data->bhi_w_sd = 0;
	/* default thresholds */
	ret = of_property_read_u32(batt_drv->device->of_node, "google,bhi-status-marginal",
				   &health_data->marginal_threshold);
	if (ret < 0)
		health_data->marginal_threshold = BHI_MARGINAL_THRESHOLD_DEFAULT;

	ret = of_property_read_u32(batt_drv->device->of_node, "google,bhi-status-need-rep",
				   &health_data->need_rep_threshold);
	if (ret < 0)
		health_data->need_rep_threshold = BHI_NEED_REP_THRESHOLD_DEFAULT;
	/* cycle count thresholds */
	ret = of_property_read_u32(batt_drv->device->of_node, "google,bhi-cycle-count-marginal",
				   &health_data->cycle_count_marginal_threshold);
	if (ret < 0)
		health_data->cycle_count_marginal_threshold = BHI_CC_MARGINAL_THRESHOLD_DEFAULT;

	ret = of_property_read_u32(batt_drv->device->of_node, "google,bhi-cycle-count-need-rep",
				   &health_data->cycle_count_need_rep_threshold);
	if (ret < 0)
		health_data->cycle_count_need_rep_threshold = BHI_CC_NEED_REP_THRESHOLD_DEFAULT;

	/* algorithm BHI_ALGO_INDI capacity threshold */
	ret = of_property_read_u32(batt_drv->device->of_node, "google,bhi-indi-cap",
				   &health_data->bhi_indi_cap);
	if (ret < 0)
		health_data->bhi_indi_cap = BHI_INDI_CAP_DEFAULT;

	/* algorithm BHI_ALGO_ACHI_B bounds check */
	ret = of_property_read_u32(batt_drv->device->of_node, "google,bhi-cycle-grace",
				   &health_data->bhi_cycle_grace);
	if (ret < 0)
		health_data->bhi_cycle_grace = BHI_CYCLE_GRACE_DEFAULT;

	/* design is the value used to build the charge table */
	bhi_data->pack_capacity = batt_drv->battery_capacity;

	/* need battery id to get right trend points */
	batt_drv->batt_id = GPSY_GET_PROP(batt_drv->fg_psy, GBMS_PROP_BATT_ID);

	ret = of_property_read_u16_array(gbms_batt_id_node(batt_drv->device->of_node),
					 "google,bhi-l-bound", &capacity_boundary[0],
					 BHI_TREND_POINTS_SIZE);
	if (ret == 0 && bhi_bound_validity_check(capacity_boundary, 0,
						 batt_drv->battery_capacity)) {
		memcpy(&bhi_data->lower_bound.limit[0], capacity_boundary,
		       sizeof(capacity_boundary));
	} else {
		for (i = 0; i < BHI_TREND_POINTS_SIZE; i++)
			bhi_data->lower_bound.limit[i] = bhi_data->pack_capacity * 60 / 100;
	}
	dev_info(batt_drv->device, "bhi_l_bound [%d, %d, %d, %d, %d, %d, %d, %d, %d, %d]\n",
		 bhi_data->lower_bound.limit[0], bhi_data->lower_bound.limit[1],
		 bhi_data->lower_bound.limit[2], bhi_data->lower_bound.limit[3],
		 bhi_data->lower_bound.limit[4], bhi_data->lower_bound.limit[5],
		 bhi_data->lower_bound.limit[6], bhi_data->lower_bound.limit[7],
		 bhi_data->lower_bound.limit[8], bhi_data->lower_bound.limit[9]);

	ret = of_property_read_u16_array(gbms_batt_id_node(batt_drv->device->of_node),
					 "google,bhi-u-bound", &capacity_boundary[0],
					 BHI_TREND_POINTS_SIZE);
	if (ret == 0 && bhi_bound_validity_check(capacity_boundary, batt_drv->battery_capacity,
						 BHI_CAPACITY_MAX)) {
		memcpy(&bhi_data->upper_bound.limit[0], capacity_boundary,
		       sizeof(capacity_boundary));
	} else {
		for (i = 0; i < BHI_TREND_POINTS_SIZE; i++)
			bhi_data->upper_bound.limit[i] = bhi_data->pack_capacity;
	}
	dev_info(batt_drv->device, "bhi_u_bound [%d, %d, %d, %d, %d, %d, %d, %d, %d, %d]\n",
		 bhi_data->upper_bound.limit[0], bhi_data->upper_bound.limit[1],
		 bhi_data->upper_bound.limit[2], bhi_data->upper_bound.limit[3],
		 bhi_data->upper_bound.limit[4], bhi_data->upper_bound.limit[5],
		 bhi_data->upper_bound.limit[6], bhi_data->upper_bound.limit[7],
		 bhi_data->upper_bound.limit[8], bhi_data->upper_bound.limit[9]);

	ret = of_property_read_u16_array(gbms_batt_id_node(batt_drv->device->of_node),
					 "google,bhi-l-trigger", &capacity_boundary[0],
					 BHI_TREND_POINTS_SIZE);
	if (ret == 0 && bhi_bound_validity_check(capacity_boundary, BHI_CAPACITY_MIN,
						 BHI_CAPACITY_MAX)) {
		memcpy(&bhi_data->lower_bound.trigger[0], capacity_boundary,
		       sizeof(capacity_boundary));
		dev_info(batt_drv->device,
			 "bhi_l_trigger [%d, %d, %d, %d, %d, %d, %d, %d, %d, %d]\n",
			 bhi_data->lower_bound.trigger[0], bhi_data->lower_bound.trigger[1],
			 bhi_data->lower_bound.trigger[2], bhi_data->lower_bound.trigger[3],
			 bhi_data->lower_bound.trigger[4], bhi_data->lower_bound.trigger[5],
			 bhi_data->lower_bound.trigger[6], bhi_data->lower_bound.trigger[7],
			 bhi_data->lower_bound.trigger[8], bhi_data->lower_bound.trigger[9]);
	}

	ret = of_property_read_u16_array(gbms_batt_id_node(batt_drv->device->of_node),
					 "google,bhi-u-trigger", &capacity_boundary[0],
					 BHI_TREND_POINTS_SIZE);
	if (ret == 0 && bhi_bound_validity_check(capacity_boundary, BHI_CAPACITY_MIN,
						 BHI_CAPACITY_MAX)) {
		memcpy(&bhi_data->upper_bound.trigger[0], capacity_boundary,
		       sizeof(capacity_boundary));
		dev_info(batt_drv->device,
			 "bhi_u_trigger [%d, %d, %d, %d, %d, %d, %d, %d, %d, %d]\n",
			 bhi_data->upper_bound.trigger[0], bhi_data->upper_bound.trigger[1],
			 bhi_data->upper_bound.trigger[2], bhi_data->upper_bound.trigger[3],
			 bhi_data->upper_bound.trigger[4], bhi_data->upper_bound.trigger[5],
			 bhi_data->upper_bound.trigger[6], bhi_data->upper_bound.trigger[7],
			 bhi_data->upper_bound.trigger[8], bhi_data->upper_bound.trigger[9]);
	}

	/* debug data initialization */
	health_data->bhi_debug_cycle_count = 0;
	health_data->bhi_debug_cap_index = 0;
	health_data->bhi_debug_imp_index = 0;
	health_data->bhi_debug_sd_index = 0;
	health_data->bhi_debug_health_index = 0;
	health_data->bhi_debug_health_status = 0;
	/* TODO: restore cal_state/cal_mode if reboot */
	health_data->cal_state = REC_STATE_OK;
	health_data->cal_mode = REC_MODE_RESET;

	return 0;
}

static void batt_fan_bt_init(struct batt_drv *batt_drv) {
	int nb_fan_bt, ret;

	nb_fan_bt = of_property_count_elems_of_size(batt_drv->device->of_node,
						    "google,fan-bt-limits", sizeof(u32));
	if (nb_fan_bt == NB_FAN_BT_LIMITS) {
		ret = of_property_read_u32_array(batt_drv->device->of_node,
						 "google,fan-bt-limits",
						 batt_drv->fan_bt_limits,
						 nb_fan_bt);
		if (ret == 0) {
			int i;

			pr_info("FAN_BT_LIMITS: ");
			for (i = 0; i < nb_fan_bt; i++)
				pr_info("%d ", batt_drv->fan_bt_limits[i]);

			return;
		} else {
			pr_err("Fail to read google,fan-bt-limits from dtsi, ret=%d\n", ret);
		}
	}
	batt_drv->fan_bt_limits[0] = FAN_BT_LIMIT_NOT_CARE;
	batt_drv->fan_bt_limits[1] = FAN_BT_LIMIT_LOW;
	batt_drv->fan_bt_limits[2] = FAN_BT_LIMIT_MED;
	batt_drv->fan_bt_limits[3] = FAN_BT_LIMIT_HIGH;

	pr_info("Use default FAN_BT_LIMITS: %d %d %d %d\n", batt_drv->fan_bt_limits[0],
							    batt_drv->fan_bt_limits[1],
							    batt_drv->fan_bt_limits[2],
							    batt_drv->fan_bt_limits[3]);
}

static int batt_prop_iter(int index, gbms_tag_t *tag, void *ptr)
{
	static gbms_tag_t keys[] = {GBMS_TAG_HCNT};
	const int count = ARRAY_SIZE(keys);

	if (index >= 0 && index < count) {
		*tag = keys[index];
		return 0;
	}

	return -ENOENT;
}

static int batt_prop_read(gbms_tag_t tag, void *buff, size_t size, void *ptr)
{
	struct batt_drv *batt_drv = ptr;
	int index, ret = 0;

	switch (tag) {
	case GBMS_TAG_HCNT:
		if (size != sizeof(u16))
			return -ERANGE;
		/* history needs to be enabled for this */
		index = hist_get_index(batt_drv->hist_data_saved_cnt, batt_drv);
		if (index < 0)
			return index;
		*(u16 *)buff = index;
		break;
	default:
		ret = -ENOENT;
		break;
	}

	return ret;
}

static struct gbms_storage_desc batt_prop_dsc = {
	.iter = batt_prop_iter,
	.read = batt_prop_read,
};

static void google_battery_init_work(struct work_struct *work)
{
	struct batt_drv *batt_drv = container_of(work, struct batt_drv,
						 init_work.work);
	struct device_node *node = batt_drv->device->of_node;
	struct power_supply *fg_psy = batt_drv->fg_psy;
	const char *batt_vs_tz_name = NULL;
	int init_delay_ms, ret = 0;

	batt_rl_reset(batt_drv);
	batt_drv->dead_battery = true; /* clear in batt_work() */
	batt_drv->capacity_level = POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN;
	batt_drv->ssoc_state.buck_enabled = -1;
	batt_drv->hold_taper_ws = false;
	batt_drv->fake_temp = 0;
	batt_drv->fake_battery_present = -1;
	batt_drv->boot_to_os_attempts = 0;
	batt_drv->charging_policy = CHARGING_POLICY_DEFAULT;
	batt_reset_chg_drv_state(batt_drv);

	mutex_init(&batt_drv->chg_lock);
	mutex_init(&batt_drv->batt_lock);
	mutex_init(&batt_drv->stats_lock);
	mutex_init(&batt_drv->cc_data.lock);
	mutex_init(&batt_drv->bpst_state.lock);
	mutex_init(&batt_drv->hda_tz_lock);
	mutex_init(&batt_drv->aacp_state_lock);

	ret = of_property_read_u32(node, "google,batt-init-delay", &init_delay_ms);
	if (ret < 0)
		init_delay_ms = BATT_DELAY_INIT_MS;

	if (!batt_drv->fg_psy) {

		fg_psy = power_supply_get_by_name(batt_drv->fg_psy_name);
		if (!fg_psy) {
			pr_debug("failed to get \"%s\" power supply, retrying...\n",
				 batt_drv->fg_psy_name);
			goto retry_init_work;
		}

		batt_drv->fg_psy = fg_psy;
	}

	if (!batt_drv->batt_present) {
		ret = GPSY_GET_PROP(fg_psy, POWER_SUPPLY_PROP_PRESENT);
		if (ret == -EAGAIN)
			goto retry_init_work;

		batt_drv->batt_present = (ret > 0);
		if (!batt_drv->batt_present)
			pr_warn("battery not present (ret=%d)\n", ret);
	}

	ret = of_property_read_u32(node, "google,recharge-soc-threshold",
				   &batt_drv->ssoc_state.rl_soc_threshold);
	if (ret < 0)
		batt_drv->ssoc_state.rl_soc_threshold =
				DEFAULT_BATT_DRV_RL_SOC_THRESHOLD;

	ret = of_property_read_u32(node, "google,bd-trickle-recharge-soc",
				   &batt_drv->ssoc_state.bd_trickle_recharge_soc);
	if (ret < 0)
		batt_drv->ssoc_state.bd_trickle_recharge_soc =
				DEFAULT_BD_TRICKLE_RL_SOC_THRESHOLD;

	batt_drv->ssoc_state.bd_trickle_dry_run = false;

	ret = of_property_read_u32(node, "google,bd-trickle-reset-sec",
				   &batt_drv->ssoc_state.bd_trickle_reset_sec);
	if (ret < 0)
		batt_drv->ssoc_state.bd_trickle_reset_sec =
				DEFAULT_BD_TRICKLE_RESET_SEC;

	batt_drv->ssoc_state.bd_trickle_enable =
		of_property_read_bool(node, "google,bd-trickle-enable");

	ret = of_property_read_u32(node, "google,ssoc-delta",
				   &batt_drv->ssoc_state.ssoc_delta);
	if (ret < 0)
		batt_drv->ssoc_state.ssoc_delta = SSOC_DELTA;

	ret = of_property_read_u32(node, "google,health-safety-margin",
				   &batt_drv->health_safety_margin);
	if (ret < 0)
		batt_drv->health_safety_margin = DEFAULT_HEALTH_SAFETY_MARGIN;

	ret = of_property_read_u32_array(node, "google,temp-record-thr",
					 batt_drv->sd.temp_thr,
					 BATT_TEMP_RECORD_THR);
	if (ret == 0) {
		ret = of_property_read_u32_array(node, "google,soc-record-thr",
						 batt_drv->sd.soc_thr,
						 BATT_TEMP_RECORD_THR);
		if (ret == 0)
			batt_drv->sd.is_enable = true;
	}

	ret = batt_init_sd(&batt_drv->sd);
	if (ret < 0) {
		pr_err("Unable to read swelling data, ret=%d\n", ret);
		batt_drv->sd.is_enable = false;
	}

	ret = of_property_read_u32(node, "google,cv-max-temp", &batt_drv->cv_max_temp);
	if (ret < 0)
		batt_drv->cv_max_temp = DEFAULT_CV_MAX_TEMPERATURE;

	/* init bpst setting */
	ret = batt_init_bpst_profile(batt_drv);
	if (ret < 0)
		pr_err("bpst profile disabled, ret=%d\n", ret);

	/* init shutdown flag */
	ret = batt_init_shutdown_flag(batt_drv);
	if (ret < 0)
		pr_err("failed to init shutdown flag, ret=%d\n", ret);

	/* cycle count is cached: read here bc SSOC, chg_profile might use it */
	batt_update_cycle_count(batt_drv);

	ret = ssoc_init(batt_drv);
	if (ret < 0 && batt_drv->batt_present)
		goto retry_init_work;

	/* could read EEPROM and history here */

	batt_init_aafv_profile(batt_drv);

	/* chg_profile will use cycle_count when aacr is enabled */
	ret = batt_init_chg_profile(batt_drv, node);
	if (ret == -EPROBE_DEFER)
		goto retry_init_work;

	if (ret < 0) {
		pr_err("charging profile disabled, ret=%d\n", ret);
	} else if (batt_drv->battery_capacity) {
		google_battery_dump_profile(&batt_drv->chg_profile);
	}

	batt_drv->temp_filter.enable = of_property_read_bool(node, "google,temp-filter-enable");
	if (batt_drv->temp_filter.enable)
		batt_init_temp_filter(batt_drv);

	cev_stats_init(&batt_drv->ce_data, &batt_drv->chg_profile);
	cev_stats_init(&batt_drv->ce_qual, &batt_drv->chg_profile);
	batt_init_csi_stat(batt_drv);

	batt_drv->fg_nb.notifier_call = psy_changed;
	ret = power_supply_reg_notifier(&batt_drv->fg_nb);
	if (ret < 0)
		pr_err("cannot register power supply notifer, ret=%d\n",
			ret);

	batt_drv->batt_ws = wakeup_source_register(NULL, "google-battery");
	batt_drv->taper_ws = wakeup_source_register(NULL, "Taper");
	batt_drv->poll_ws = wakeup_source_register(NULL, "Poll");
	batt_drv->msc_ws = wakeup_source_register(NULL, "MSC");
	if (!batt_drv->batt_ws || !batt_drv->taper_ws ||
			!batt_drv->poll_ws || !batt_drv->msc_ws)
		pr_err("failed to register wakeup sources\n");

	mutex_lock(&batt_drv->cc_data.lock);
	ret = batt_cycle_count_load(&batt_drv->cc_data);
	if (ret < 0)
		pr_err("cannot restore bin count ret=%d\n", ret);
	mutex_unlock(&batt_drv->cc_data.lock);

	batt_drv->fake_capacity = (batt_drv->batt_present) ? -EINVAL
						: DEFAULT_BATT_FAKE_CAPACITY;

	/* charging configuration */
	ret = of_property_read_u32(node, "google,update-interval",
				   &batt_drv->batt_update_interval);
	if (ret < 0)
		batt_drv->batt_update_interval = DEFAULT_BATT_UPDATE_INTERVAL;

	/* high temperature notify configuration */
	ret = of_property_read_u32(batt_drv->device->of_node,
				   "google,update-high-temp-threshold",
				   &batt_drv->batt_update_high_temp_threshold);
	if (ret < 0)
		batt_drv->batt_update_high_temp_threshold =
					DEFAULT_HIGH_TEMP_UPDATE_THRESHOLD;
	/* charge statistics */
	ret = of_property_read_u32(node, "google,chg-stats-qual-time",
				   &batt_drv->chg_sts_qual_time);
	if (ret < 0)
		batt_drv->chg_sts_qual_time =
					DEFAULT_CHG_STATS_MIN_QUAL_TIME;

	ret = of_property_read_u32(node, "google,chg-stats-delta-soc",
				   &batt_drv->chg_sts_delta_soc);
	if (ret < 0)
		batt_drv->chg_sts_delta_soc =
					DEFAULT_CHG_STATS_MIN_DELTA_SOC;

	/* time to full */
	ret = ttf_stats_init(&batt_drv->ttf_stats, batt_drv->device->of_node,
			     batt_drv->battery_capacity);
	if (ret < 0)
		pr_info("time to full not available\n");

	/* TTF log is used report more things nowadays */
	batt_drv->ttf_stats.ttf_log = logbuffer_register("ttf");
	if (IS_ERR(batt_drv->ttf_stats.ttf_log)) {
		ret = PTR_ERR(batt_drv->ttf_stats.ttf_log);
		dev_err(batt_drv->device, "failed to create ttf_log, ret=%d\n", ret);

		batt_drv->ttf_stats.ttf_log = NULL;
	}

	/* RAVG: google_resistance  */
	ret = batt_res_load_data(&batt_drv->health_data.bhi_data.res_state,
				 batt_drv->fg_psy);
	if (ret < 0)
		dev_warn(batt_drv->device, "RAVG not available (%d)\n", ret);
	batt_res_dump_logs(&batt_drv->health_data.bhi_data.res_state);

	/* health based charging, triggers */
	batt_init_chg_health(batt_drv);

	/* override setting google,battery-roundtrip = 0 in device tree */
	batt_drv->disable_votes =
		of_property_read_bool(node, "google,disable-votes");
	if (batt_drv->disable_votes)
		pr_info("battery votes disabled\n");

	/* pairing battery vs. device */
	if (of_property_read_bool(node, "google,eeprom-pairing")) {
		batt_drv->pairing_state = BATT_PAIRING_ENABLED;
		batt_drv->pairing_state_retry_cnt = PAIRING_RETRIES;
	} else {
		batt_drv->pairing_state = BATT_PAIRING_DISABLED;
	}

	/* use delta cycle count to adjust collecting period */
	ret = of_property_read_u32(batt_drv->device->of_node,
					"google,history-delta-cycle-count",
					&batt_drv->hist_delta_cycle_cnt);
	if (ret < 0)
		batt_drv->hist_delta_cycle_cnt = HCC_DEFAULT_DELTA_CYCLE_CNT;

	ret = of_property_read_u32(batt_drv->device->of_node, "google,batt-voltage-critical",
				   &batt_drv->batt_critical_voltage);
	if (ret < 0)
		batt_drv->batt_critical_voltage = VBATT_CRITICAL_LEVEL;

	/* battery virtual sensor */
	ret = of_property_read_string(batt_drv->device->of_node,
				      "google,batt-vs-tz-name",
				      &batt_vs_tz_name);
	if (ret == 0) {
		batt_drv->batt_vs_tz =
		    thermal_zone_device_register(batt_vs_tz_name, 0, 0,
						 batt_drv, &batt_vs_tz_ops, NULL, 0, 0);
		if (IS_ERR(batt_drv->batt_vs_tz)) {
			pr_err("batt_vs tz register failed. err:%ld\n",
			       PTR_ERR(batt_drv->batt_vs_tz));
			batt_drv->batt_vs_tz = NULL;
		} else {
			thermal_zone_device_update(batt_drv->batt_vs_tz, THERMAL_DEVICE_UP);
		}
		batt_drv->batt_vs_w = 88;

		pr_info("google,batt-vs-tz-name is %s\n", batt_vs_tz_name);
	}

	/* battery virtual sensor for more power */
	batt_drv->batt_vs_mp_tz = thermal_zone_device_register("mdis_morepower", 0, 0,
								batt_drv, &batt_vs_mp_tz_ops,
								NULL, 0, 0);
	if (IS_ERR(batt_drv->batt_vs_mp_tz)) {
		pr_err("batt_vs_mp tz register failed. err: %ld\n",
			PTR_ERR(batt_drv->batt_vs_mp_tz));
		batt_drv->batt_vs_mp_tz = NULL;
	} else {
		thermal_zone_device_update(batt_drv->batt_vs_mp_tz, THERMAL_DEVICE_UP);
	}

	batt_drv->batt_vs_hda_tz = thermal_zone_device_register("thb_hda", 0, 0,
								batt_drv, &batt_vs_hda_tz_ops,
								NULL, 0, 0);
	if (IS_ERR(batt_drv->batt_vs_hda_tz)) {
		pr_err("batt_vs_hda_tz register failed. err: %ld\n",
			PTR_ERR(batt_drv->batt_vs_hda_tz));
	} else {
		thermal_zone_device_update(batt_drv->batt_vs_hda_tz, THERMAL_DEVICE_UP);
	}

	ret = of_property_read_u32(batt_drv->device->of_node, "google,morepower-soc-limit-low",
				   &batt_drv->soc_mp_limit_low);
	if (ret < 0)
		batt_drv->soc_mp_limit_low = SOC_MP_LIMIT_LOW;

	ret = of_property_read_u32(batt_drv->device->of_node, "google,morepower-soc-limit-high",
				   &batt_drv->soc_mp_limit_high);
	if (ret < 0)
		batt_drv->soc_mp_limit_high = SOC_MP_LIMIT_HIGH;

	ret = of_property_read_u32(batt_drv->device->of_node, "google,morepower_therm_limit",
				   &batt_drv->therm_mp_limit);
	if (ret < 0)
		batt_drv->therm_mp_limit = THERM_MP_LIMIT;

	ret = of_property_read_u32(batt_drv->device->of_node, "google,morepower_max_ratio_limit",
				   &batt_drv->max_ratio_mp_limit);
	if (ret < 0)
		batt_drv->max_ratio_mp_limit = MAX_RATIO_MP_LIMIT;

	batt_drv->dc_irdrop = of_property_read_bool(node, "google,dc-irdrop");
	if (batt_drv->dc_irdrop)
		pr_info("dc irdrop is enabled\n");

	batt_drv->pullback_current = of_property_read_bool(gbms_batt_id_node(node),
							   "google,pullback-current");
	if (batt_drv->pullback_current)
		pr_info("pullback current is enabled\n");

	batt_drv->allow_higher_fv = of_property_read_bool(gbms_batt_id_node(node),
							   "google,allow-higher-fv");
	if (batt_drv->allow_higher_fv)
		pr_info("allow higher fv is enabled\n");

	ret = of_property_read_u32(node, "google,first-usage-date",
				   &batt_drv->health_data.bhi_data.first_usage_date);
	if (ret < 0)
		batt_drv->health_data.bhi_data.first_usage_date = -1;

	/* AACP opt-out */
	ret = of_property_read_u32(node, "google,aacp-opt-out-cut-off-cycles",
				   &batt_drv->aacp_opt_out_cut_off_cycles);
	if (ret < 0)
		batt_drv->aacp_opt_out_cut_off_cycles = AACP_OPT_OUT_CUT_OFF_CYCLES_DEFAULT;

	/* AACP health status */
	ret = of_property_read_u32(node, "google,aacp-health-over-cliff",
				   &batt_drv->aacp_health_over_cliff);
	if (ret < 0)
		batt_drv->aacp_health_over_cliff = BH_NEEDS_REPLACEMENT;

	ret = of_property_read_u32(node, "google,aacp-health-over-cutoff",
				   &batt_drv->aacp_health_over_cutoff);
	if (ret < 0)
		batt_drv->aacp_health_over_cutoff = BH_MARGINAL;

	ret = of_property_read_u32(node, "google,aacp-health-last-entry",
				   &batt_drv->aacp_health_last_entry);
	if (ret < 0)
		batt_drv->aacp_health_last_entry = BH_MARGINAL;

	/* single battery disconnect */
	(void)batt_bpst_init_debugfs(batt_drv);

	/* these don't require nvm storage */
	ret = gbms_storage_register(&batt_prop_dsc, "battery", batt_drv);
	if (ret == -EBUSY)
		ret = 0;

	/* google_battery expose history via a standard device */
	batt_drv->history = gbms_storage_create_device("battery_history",
						       GBMS_TAG_HIST);
	if (!batt_drv->history)
		pr_err("history not available\n");

	/* BHI: might need RAVG and battery history */
	ret = batt_bhi_init(batt_drv);
	if (ret < 0) {
		dev_warn(batt_drv->device, "BHI: not supported (%d)\n", ret);
	} else {
		/* reload the last estimates,  */
		ret = batt_bhi_data_load(batt_drv);
		if (ret < 0)
			dev_err(batt_drv->device, "BHI: invalid data, starting fresh (%d)\n", ret);
	}

	/* use delta cycle count != 0 to enable collecting history */
	if (batt_drv->hist_delta_cycle_cnt) {
		batt_drv->blf_state = BATT_LFCOLLECT_ENABLED;
		batt_update_hist_work(batt_drv);
	}

	/* power metrics */
	schedule_delayed_work(&batt_drv->power_metrics.work,
			      msecs_to_jiffies(batt_drv->power_metrics.polling_rate * 1000));

	/* configure force full charge when charge limit is enable */
	ret = of_property_read_s32(node, "google,force-fcn-update-cycle",
				   &batt_drv->force_fcr_update_cycle);
	if (ret != 0)
		batt_drv->force_fcr_update_cycle = DEFAULT_FORCE_FCR_UPDATE_CYCLE;

	ret = gbms_storage_read(GBMS_TAG_FCRU, &batt_drv->last_full_charge, GBMS_FCRU_LEN);
	if (ret < 0)
		dev_err(batt_drv->device, "failed to read GBMS_TAG_FCRU (%d)\n", ret);

	/* if force_fcr_update_cycle has default value of eeprom */
	if (batt_drv->last_full_charge == 0xFFFF)
		batt_drv->last_full_charge = 0;

	/* configure force full charge mode */
	ret = of_property_read_s32(node, "google,force-fcn-update-ops",
				   &batt_drv->force_fcr_update_ops);
	if (ret != 0)
		batt_drv->force_fcr_update_ops = BATT_FCR_UPDATE_FULLCHARGE_DELTA;



	pr_info("google_battery init_work done\n");

	batt_drv->init_complete = true;
	batt_drv->resume_complete = true;

	schedule_delayed_work(&batt_drv->batt_work, 0);

	return;

retry_init_work:
	schedule_delayed_work(&batt_drv->init_work,
			      msecs_to_jiffies(init_delay_ms));
}

static struct thermal_zone_device_ops google_battery_tz_ops = {
	.get_temp = google_battery_tz_get_cycle_count,
};

static int batt_ravg_init(struct batt_res *res_state, struct device_node *node)
{
	int ret;

	if (of_property_read_bool(node, "google,no-ravg"))
		return -ENOENT;

	/* Resistance Estimation configuration */
	ret = of_property_read_u32(node, "google,res-temp-hi",
				   &res_state->res_temp_high);
	if (ret < 0)
		res_state->res_temp_high = DEFAULT_RES_TEMP_HIGH;

	ret = of_property_read_u32(node, "google,res-temp-lo",
				   &res_state->res_temp_low);
	if (ret < 0)
		res_state->res_temp_low = DEFAULT_RES_TEMP_LOW;

	ret = of_property_read_u32(node, "google,res-soc-thresh",
				   &res_state->ravg_soc_high);
	if (ret < 0)
		res_state->ravg_soc_high = DEFAULT_RAVG_SOC_HIGH;
	ret = of_property_read_u32(node, "google,ravg-soc-low",
				   &res_state->ravg_soc_low);
	if (ret < 0)
		res_state->ravg_soc_low = DEFAULT_RAVG_SOC_LOW;

	ret = of_property_read_u32(node, "google,res-filt-length",
				   &res_state->estimate_filter);
	if (ret < 0)
		res_state->estimate_filter = DEFAULT_RES_FILT_LEN;

	return 0;
}

static int google_battery_probe(struct platform_device *pdev)
{
	const char *fg_psy_name, *psy_name = NULL;
	struct batt_drv *batt_drv;
	int ret;
	struct power_supply_config psy_cfg = {};

	batt_drv = devm_kzalloc(&pdev->dev, sizeof(*batt_drv), GFP_KERNEL);
	if (!batt_drv)
		return -ENOMEM;

	batt_drv->device = &pdev->dev;

	ret = of_property_read_string(pdev->dev.of_node, "google,fg-psy-name",
				      &fg_psy_name);
	if (ret != 0) {
		pr_err("cannot read google,fg-psy-name, ret=%d\n", ret);
		return -EINVAL;
	}

	batt_drv->fg_psy_name = devm_kstrdup(&pdev->dev, fg_psy_name,
					     GFP_KERNEL);
	if (!batt_drv->fg_psy_name)
		return -ENOMEM;

	/* change name and type for debug/test */
	if (of_property_read_bool(pdev->dev.of_node, "google,psy-type-unknown"))
		gbatt_psy_desc.psy_dsc.type = POWER_SUPPLY_TYPE_UNKNOWN;

	ret = of_property_read_string(pdev->dev.of_node,
				      "google,psy-name", &psy_name);
	if (ret == 0) {
		gbatt_psy_desc.psy_dsc.name =
		    devm_kstrdup(&pdev->dev, psy_name, GFP_KERNEL);
	}

	ret = of_property_read_s32(pdev->dev.of_node, "google,vbatt-crit-deadline-sec",
				   &batt_drv->vbatt_crit_deadline_sec);
	if (ret != 0) {
		batt_drv->vbatt_crit_deadline_sec = VBATT_CRITICAL_DEADLINE_SEC;
	}

	INIT_DELAYED_WORK(&batt_drv->init_work, google_battery_init_work);
	INIT_DELAYED_WORK(&batt_drv->batt_work, google_battery_work);
	INIT_DELAYED_WORK(&batt_drv->power_metrics.work, power_metrics_data_work);
	INIT_DELAYED_WORK(&batt_drv->temp_filter.work, google_battery_temp_filter_work);
	platform_set_drvdata(pdev, batt_drv);

	psy_cfg.drv_data = batt_drv;
	psy_cfg.of_node = pdev->dev.of_node;

	batt_drv->psy = devm_power_supply_register(batt_drv->device,
						   &gbatt_psy_desc.psy_dsc, &psy_cfg);
	if (IS_ERR(batt_drv->psy)) {
		ret = PTR_ERR(batt_drv->psy);
		if (ret == -EPROBE_DEFER)
			return -EPROBE_DEFER;

		/* TODO: fail with -ENODEV */
		dev_err(batt_drv->device,
			"Couldn't register as power supply, ret=%d\n", ret);
	}

	batt_drv->ssoc_log = logbuffer_register("ssoc");
	if (IS_ERR(batt_drv->ssoc_log)) {
		ret = PTR_ERR(batt_drv->ssoc_log);
		dev_err(batt_drv->device,
			"failed to create ssoc_log, ret=%d\n", ret);
		batt_drv->ssoc_log = NULL;
	}

	/* RAVG: google_resistance */
	ret = batt_ravg_init(&batt_drv->health_data.bhi_data.res_state,
			     pdev->dev.of_node);
	if (ret < 0)
		dev_info(batt_drv->device, "RAVG: not available\n");

	batt_drv->tz_dev = devm_thermal_of_zone_register(batt_drv->device, 0,
							 batt_drv,
							 &google_battery_tz_ops);
	if (IS_ERR(batt_drv->tz_dev)) {
		pr_err("battery tz register failed. err:%ld\n",
			PTR_ERR(batt_drv->tz_dev));
		ret = PTR_ERR(batt_drv->tz_dev);
		batt_drv->tz_dev = NULL;
	} else {
		thermal_zone_device_update(batt_drv->tz_dev, THERMAL_DEVICE_UP);
	}

	/* Fan levels limits from battery temperature */
	batt_fan_bt_init(batt_drv);

	/* charge speed interface: status and type */
	batt_drv->csi_status_votable =
		gvotable_create_int_election(NULL, gvotable_comparator_int_min,
					     csi_status_cb, batt_drv);
	if (IS_ERR_OR_NULL(batt_drv->csi_status_votable)) {
		ret = PTR_ERR(batt_drv->csi_status_votable);
		batt_drv->csi_status_votable = NULL;
	}

	gvotable_set_default(batt_drv->csi_status_votable, (void *)CSI_STATUS_UNKNOWN);
	gvotable_set_vote2str(batt_drv->csi_status_votable, gvotable_v2s_int);
	gvotable_election_set_name(batt_drv->csi_status_votable, VOTABLE_CSI_STATUS);

	batt_drv->csi_type_votable =
		gvotable_create_int_election(NULL, gvotable_comparator_int_min,
					     csi_type_cb, batt_drv);
	if (IS_ERR_OR_NULL(batt_drv->csi_type_votable)) {
		ret = PTR_ERR(batt_drv->csi_type_votable);
		batt_drv->csi_type_votable = NULL;
	}

	gvotable_set_default(batt_drv->csi_type_votable, (void *)CSI_TYPE_UNKNOWN);
	gvotable_set_vote2str(batt_drv->csi_type_votable, gvotable_v2s_int);
	gvotable_election_set_name(batt_drv->csi_type_votable, VOTABLE_CSI_TYPE);

	batt_drv->point_full_ui_soc_votable =
		gvotable_create_int_election(NULL, gvotable_comparator_int_min,
					     point_full_ui_soc_cb, batt_drv);
	if (IS_ERR_OR_NULL(batt_drv->point_full_ui_soc_votable)) {
		ret = PTR_ERR(batt_drv->point_full_ui_soc_votable);
		dev_err(batt_drv->device, "Fail to create point_full_ui_soc_votable\n");
		batt_drv->point_full_ui_soc_votable = NULL;
	} else {
		gvotable_set_vote2str(batt_drv->point_full_ui_soc_votable, gvotable_v2s_int);
		gvotable_set_default(batt_drv->point_full_ui_soc_votable, (void *)DISABLE_POINT_FULL_UI_SOC);
		gvotable_election_set_name(batt_drv->point_full_ui_soc_votable, VOTABLE_CHARGING_UISOC);
	}

	batt_drv->hda_tz_votable =
		gvotable_create_int_election(NULL, gvotable_comparator_int_max, hda_tz_cb, batt_drv);
	if (IS_ERR_OR_NULL(batt_drv->hda_tz_votable)) {
		ret = PTR_ERR(batt_drv->hda_tz_votable);
		dev_err(batt_drv->device, "Fail to create hda_tz_votable (%d)\n", ret);
		batt_drv->hda_tz_votable = NULL;
	} else {
		gvotable_set_vote2str(batt_drv->hda_tz_votable, gvotable_v2s_int);
		gvotable_set_default(batt_drv->hda_tz_votable, (void *)0);
		gvotable_election_set_name(batt_drv->hda_tz_votable, VOTABLE_HDA_TZ);
	}

	ret = of_property_read_u32(pdev->dev.of_node, "google,hda-tz-limit",
				   &batt_drv->hda_tz_limit);

	/* TODO: b/403865140 move these configurations before the votables */

	/* AAFV configuration */
	batt_drv->aafv_state = BATT_AAFV_UNKNOWN;
	batt_drv->aafv_apply_max = AAFV_APPLY_MAX_DEFAULT;
	batt_drv->aafv_max_offset = AAFV_MAX_OFFSET_DEFAULT;
	batt_drv->aafv_cliff_cycle = AAFV_CLIFF_CYCLE_DEFAULT;
	batt_drv->aafv_cliff_offset = AAFV_CLIFF_OFFSET_DEFAULT;

	/* TODO: b/403865140 refactor AACT to batt_init_aact_profile() */
	/* NOTE: might need to have a device and batteryID configs */
	ret = of_property_read_u32(pdev->dev.of_node, "google,aact-config",
				   &batt_drv->aact_state);
	if (ret < 0)
		batt_drv->aact_state = BATT_AACT_UNKNOWN;

	/*
	 * AA** features are opted opted out until the state is confirmed by
	 * a system property during boot. features that are enabled by default
	 * cannot be opted out.
	 */
	batt_drv->aacp_opt_out = BATT_AACP_OPT_OUT_ENABLED;

	/* AACP version will be updated from server side when parameters updated */
	batt_drv->aacp_version = 1;

	/* create the sysfs node */
	batt_init_fs(batt_drv);
	batt_bpst_init_fs(batt_drv);

	/* debugfs */
	(void)batt_init_debugfs(batt_drv);

	/* give time to fg driver to start */
	schedule_delayed_work(&batt_drv->init_work,
					msecs_to_jiffies(BATT_DELAY_INIT_MS));

	/* power metrics */
	batt_drv->power_metrics.polling_rate = 30;
	batt_drv->power_metrics.interval = 120;

	/* Date of manufacturing of the battery */
	ret = batt_get_manufacture_date(&batt_drv->health_data.bhi_data);
	if (ret < 0)
		pr_warn("cannot get battery manufacture date, ret=%d\n", ret);

	/* Date of first use of the battery */
	if (!batt_drv->health_data.bhi_data.act_date[0]) {
		ret = batt_get_activation_date(&batt_drv->health_data.bhi_data);
		if (ret < 0)
			pr_warn("cannot get battery activation date, ret=%d\n", ret);
	}

	return 0;
}

static int google_battery_remove(struct platform_device *pdev)
{
	struct batt_drv *batt_drv = platform_get_drvdata(pdev);

	if (!batt_drv)
		return 0;

	power_supply_unreg_notifier(&batt_drv->fg_nb);

	if (batt_drv->ssoc_log)
		logbuffer_unregister(batt_drv->ssoc_log);
	if (batt_drv->ttf_stats.ttf_log)
		logbuffer_unregister(batt_drv->ttf_stats.ttf_log);
	if (batt_drv->history)
		gbms_storage_cleanup_device(batt_drv->history);

	if (batt_drv->fg_psy)
		power_supply_put(batt_drv->fg_psy);

	batt_hist_free_data(batt_drv->hist_data);

	gbms_free_chg_profile(&batt_drv->chg_profile);

	wakeup_source_unregister(batt_drv->msc_ws);
	wakeup_source_unregister(batt_drv->batt_ws);
	wakeup_source_unregister(batt_drv->taper_ws);
	wakeup_source_unregister(batt_drv->poll_ws);

	gvotable_destroy_election(batt_drv->fan_level_votable);
	gvotable_destroy_election(batt_drv->csi_status_votable);
	gvotable_destroy_election(batt_drv->csi_type_votable);
	gvotable_destroy_election(batt_drv->point_full_ui_soc_votable);

	batt_drv->fan_level_votable = NULL;
	batt_drv->csi_status_votable = NULL;
	batt_drv->csi_type_votable = NULL;
	batt_drv->charging_policy_votable = NULL;
	batt_drv->point_full_ui_soc_votable = NULL;

	return 0;
}

static void google_battery_shutdown(struct platform_device *pdev)
{
	struct batt_drv *batt_drv = platform_get_drvdata(pdev);

	if (!batt_drv)
		return;

	power_supply_unreg_notifier(&batt_drv->fg_nb);
}

#ifdef SUPPORT_PM_SLEEP
static int gbatt_pm_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct batt_drv *batt_drv = platform_get_drvdata(pdev);

	pm_runtime_get_sync(batt_drv->device);
	batt_drv->resume_complete = false;
	pm_runtime_put_sync(batt_drv->device);

	return 0;
}

static int gbatt_pm_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct batt_drv *batt_drv = platform_get_drvdata(pdev);

	pm_runtime_get_sync(batt_drv->device);
	batt_drv->resume_complete = true;
	batt_drv->temp_filter.resume_delay = true;
	pm_runtime_put_sync(batt_drv->device);

	mod_delayed_work(system_wq, &batt_drv->batt_work, 0);

	return 0;
}

static const struct dev_pm_ops gbatt_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(gbatt_pm_suspend, gbatt_pm_resume)
};
#endif


static const struct of_device_id google_charger_of_match[] = {
	{.compatible = "google,battery"},
	{},
};
MODULE_DEVICE_TABLE(of, google_charger_of_match);


static struct platform_driver google_battery_driver = {
	.driver = {
		   .name = "google,battery",
		   .owner = THIS_MODULE,
		   .of_match_table = google_charger_of_match,
		   .probe_type = PROBE_PREFER_ASYNCHRONOUS,
#ifdef SUPPORT_PM_SLEEP
		   .pm = &gbatt_pm_ops,
#endif
		   },
	.probe = google_battery_probe,
	.remove = google_battery_remove,
	.shutdown = google_battery_shutdown,
};

module_platform_driver(google_battery_driver);

MODULE_DESCRIPTION("Google Battery Driver");
MODULE_AUTHOR("AleX Pelosi <apelosi@google.com>");
MODULE_LICENSE("GPL");
