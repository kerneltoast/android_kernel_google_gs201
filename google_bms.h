/*
 * Google Battery Management System
 *
 * Copyright (C) 2018 Google Inc.
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

#ifndef __GOOGLE_BMS_H_
#define __GOOGLE_BMS_H_

#include <linux/minmax.h>
#include <linux/types.h>
#include <linux/usb/pd.h>
#include <misc/logbuffer.h>
#include "gbms_power_supply.h"
#include "qmath.h"
#include "gbms_storage.h"

struct device_node;

#define GBMS_CHG_TEMP_NB_LIMITS_MAX 10
#define GBMS_CHG_VOLT_NB_LIMITS_MAX 5
#define GBMS_CHG_ALG_BUF_SZ 500
#define GBMS_CHG_TOPOFF_NB_LIMITS_MAX 6
#define GBMS_AACR_DATA_MAX 10

struct gbms_chg_profile {
	const char *owner_name;

	int temp_nb_limits;
	s32 temp_limits[GBMS_CHG_TEMP_NB_LIMITS_MAX];
	int volt_nb_limits;
	s32 volt_limits[GBMS_CHG_VOLT_NB_LIMITS_MAX];
	int topoff_nb_limits;
	s32 topoff_limits[GBMS_CHG_TOPOFF_NB_LIMITS_MAX];
	/* Array of constant current limits */
	u32 *cccm_limits;
	/* used to fill table  */
	u32 capacity_ma;

	/* behavior */
	u32 fv_uv_margin_dpct;
	u32 fv_dc_ratio;
	u32 cv_range_accuracy;
	u32 cv_debounce_cnt;
	u32 cv_update_interval;
	u32 cv_tier_ov_cnt;
	u32 cv_tier_switch_cnt;
	/* taper step */
	u32 fv_uv_resolution;
	/* experimental */
	u32 cv_otv_margin;

	/* AACR feature */
	u32 reference_cycles[GBMS_AACR_DATA_MAX];
	u32 reference_fade10[GBMS_AACR_DATA_MAX];
	u32 aacr_nb_limits;
};

#define WLC_BPP_THRESHOLD_UV	7000000
#define WLC_EPP_THRESHOLD_UV	11000000

#define FOREACH_CHG_EV_ADAPTER(S)		\
	S(UNKNOWN), 	\
	S(USB),		\
	S(USB_SDP),	\
	S(USB_DCP),	\
	S(USB_CDP),	\
	S(USB_ACA),	\
	S(USB_C),	\
	S(USB_PD),	\
	S(USB_PD_DRP),	\
	S(USB_PD_PPS),	\
	S(USB_BRICKID),	\
	S(USB_HVDCP),	\
	S(USB_HVDCP3),	\
	S(FLOAT),	\
	S(WLC),		\
	S(WLC_EPP),	\
	S(WLC_SPP),	\
	S(GPP),		\
	S(10W),		\
	S(L7),		\
	S(DL),		\
	S(WPC_EPP),	\
	S(WPC_GPP),	\
	S(WPC_10W),	\
	S(WPC_BPP),	\
	S(WPC_L7),	\
	S(EXT),	\
	S(EXT1),	\
	S(EXT2),	\
	S(EXT_UNKNOWN), \

#define CHG_EV_ADAPTER_STRING(s)	#s
#define _CHG_EV_ADAPTER_PRIMITIVE_CAT(a, ...) a ## __VA_ARGS__

#define BATTERY_DEBUG_ATTRIBUTE(name, fn_read, fn_write) \
static const struct file_operations name = {	\
	.open	= simple_open,			\
	.llseek	= no_llseek,			\
	.read	= fn_read,			\
	.write	= fn_write,			\
}

/* Enums will start with CHG_EV_ADAPTER_TYPE_ */
#define CHG_EV_ADAPTER_ENUM(e)	\
			_CHG_EV_ADAPTER_PRIMITIVE_CAT(CHG_EV_ADAPTER_TYPE_,e)

enum chg_ev_adapter_type_t {
	FOREACH_CHG_EV_ADAPTER(CHG_EV_ADAPTER_ENUM)
};

enum gbms_msc_states_t {
	MSC_NONE = 0,
	MSC_SEED,
	MSC_DSG,
	MSC_LAST,
	MSC_VSWITCH,
	MSC_VOVER,
	MSC_PULLBACK,
	MSC_FAST,
	MSC_TYPE,
	MSC_DLY,	/* in taper */
	MSC_STEADY,	/* in taper */
	MSC_TIERCNTING, /* in taper */
	MSC_RAISE,	/* in taper */
	MSC_WAIT,	/* in taper */
	MSC_RSTC,	/* in taper */
	MSC_NEXT,	/* in taper */
	MSC_NYET,	/* in taper */
	MSC_HEALTH,
	MSC_HEALTH_PAUSE,
	MSC_HEALTH_ALWAYS_ON,
	MSC_STATES_COUNT,
};

union gbms_ce_adapter_details {
	uint32_t	v;
	struct {
		uint8_t		ad_type;
		uint8_t		pad;
		uint8_t 	ad_voltage;
		uint8_t 	ad_amperage;
	};
};

struct gbms_ce_stats {
	uint16_t 	voltage_in;
	uint16_t	ssoc_in;
	uint16_t	cc_in;
	uint16_t 	voltage_out;
	uint16_t 	ssoc_out;
	uint16_t	cc_out;
};

struct ttf_tier_stat {
	int16_t soc_in;
	int	cc_in;
	int	cc_total;
	ktime_t	avg_time;
};

struct gbms_ce_tier_stats {
	int8_t		temp_idx;
	int8_t		vtier_idx;

	int16_t		soc_in;		/* 8.8 */
	uint16_t	cc_in;
	uint16_t	cc_total;

	uint32_t	time_fast;
	uint32_t	time_taper;
	uint32_t	time_other;

	int16_t		temp_in;
	int16_t		temp_min;
	int16_t		temp_max;

	int16_t		ibatt_min;
	int16_t		ibatt_max;

	uint16_t	icl_min;
	uint16_t	icl_max;

	int64_t		icl_sum;
	int64_t		temp_sum;
	int64_t		ibatt_sum;
	uint32_t 	sample_count;

	uint16_t 	msc_cnt[MSC_STATES_COUNT];
	uint32_t 	msc_elap[MSC_STATES_COUNT];
};

#define GBMS_STATS_TIER_COUNT	3
#define GBMS_SOC_STATS_LEN	101

/* time to full */

/* collected in charging event */
struct ttf_soc_stats {
	int ti[GBMS_SOC_STATS_LEN];		/* charge tier at each soc */
	int cc[GBMS_SOC_STATS_LEN];		/* coulomb count at each soc */
	ktime_t elap[GBMS_SOC_STATS_LEN];	/* time spent at soc */
};

/* reference data for soc estimation  */
struct ttf_adapter_stats {
	u32 *soc_table;
	u32 *elap_table;
	int table_count;
};

/* updated when the device publish the charge stats
 * NOTE: soc_stats and tier_stats are only valid for the given chg_profile
 * since tier, coulumb count and elap time spent at each SOC depends on the
 * maximum amout of current that can be pushed to the battery.
 */
struct batt_ttf_stats {
	ktime_t ttf_fake;

	struct ttf_soc_stats soc_ref;	/* gold: soc->elap,cc */
	int ref_temp_idx;
	int ref_watts;

	struct ttf_soc_stats soc_stats; /* rolling */
	struct ttf_tier_stat tier_stats[GBMS_STATS_TIER_COUNT];

	struct logbuffer *ttf_log;
};

/*
 * health based charging can be enabled from userspace with a deadline
 *
 * initial state:
 * 	deadline = 0, rest_state = CHG_HEALTH_INACTIVE
 *
 * deadline = -1 from userspace
 *	CHG_HEALTH_* -> CHG_HEALTH_USER_DISABLED (settings disabled)
 * on deadline = 0 from userspace
 *	CHG_HEALTH_* -> CHG_HEALTH_USER_DISABLED (alarm, plug or misc. disabled)
 * on deadline > 0 from userspace
 * 	CHG_HEALTH_* -> CHG_HEALTH_ENABLED
 *
 *  from CHG_HEALTH_ENABLED, msc_logic_health() can change the state to
 * 	CHG_HEALTH_ENABLED  <-> CHG_HEALTH_ACTIVE
 * 	CHG_HEALTH_ENABLED  -> CHG_HEALTH_DISABLED
 *
 * from CHG_HEALTH_ACTIVE, msc_logic_health() can change the state to
 * 	CHG_HEALTH_ACTIVE   <-> CHG_HEALTH_ENABLED
 * 	CHG_HEALTH_ACTIVE   -> CHG_HEALTH_DISABLED
 * 	CHG_HEALTH_ACTIVE   -> CHG_HEALTH_DONE
 */
enum chg_health_state {
	CHG_HEALTH_CCLVL_DISABLED = -6,
	CHG_HEALTH_BD_DISABLED = -5,
	CHG_HEALTH_USER_DISABLED = -3,
	CHG_HEALTH_DISABLED = -2,
	CHG_HEALTH_DONE = -1,
	CHG_HEALTH_INACTIVE = 0,
	CHG_HEALTH_ENABLED,
	CHG_HEALTH_ACTIVE,
	CHG_HEALTH_PAUSE,
};

#define STATS_TH_SIZE 10
/* tier index used to log the session */
enum gbms_stats_tier_idx_t {
	GBMS_STATS_AC_TI_DISABLE_DIALOG = -6,
	GBMS_STATS_AC_TI_DEFENDER = -5,
	GBMS_STATS_AC_TI_DISABLE_SETTING_STOP = -4,
	GBMS_STATS_AC_TI_DISABLE_MISC = -3,
	GBMS_STATS_AC_TI_DISABLE_SETTING = -2,
	GBMS_STATS_AC_TI_INVALID = -1,

	/* Regular charge tiers 0 -> 9 */
	GBMS_STATS_AC_TI_VALID = 10,
	GBMS_STATS_AC_TI_DISABLED = 11,
	GBMS_STATS_AC_TI_ENABLED = 12,
	GBMS_STATS_AC_TI_ACTIVE = 13,
	GBMS_STATS_AC_TI_ENABLED_AON = 14,
	GBMS_STATS_AC_TI_ACTIVE_AON = 15,
	GBMS_STATS_AC_TI_PAUSE = 16,
	GBMS_STATS_AC_TI_PAUSE_AON = 17,
	GBMS_STATS_AC_TI_V2_PREDICT = 18,
	GBMS_STATS_AC_TI_V2_PREDICT_SUCCESS = 19,
	GBMS_STATS_AC_TI_DONE_AON = 20,

	/* Thermal stats, reported from google_charger - reserved 50-59 */
	GBMS_STATS_TH_LVL0 = 50,
	GBMS_STATS_TH_LVL1 = 51,
	GBMS_STATS_TH_LVL2 = 52,
	GBMS_STATS_TH_LVL3 = 53,
	GBMS_STATS_TH_LVL4 = 54,
	GBMS_STATS_TH_LVL5 = 55,
	GBMS_STATS_TH_LVL6 = 56,
	GBMS_STATS_TH_LVL7 = 57,
	GBMS_STATS_TH_LVL8 = 58,
	GBMS_STATS_TH_LVL9 = 59,

	/* TODO: rename, these are not really related to AC */
	GBMS_STATS_AC_TI_FULL_CHARGE = 100,
	GBMS_STATS_AC_TI_HIGH_SOC = 101,

	/* Defender TEMP or DWELL */
	GBMS_STATS_BD_TI_OVERHEAT_TEMP = 110,
	GBMS_STATS_BD_TI_CUSTOM_LEVELS = 111,
	GBMS_STATS_BD_TI_TRICKLE = 112,
	GBMS_STATS_BD_TI_DOCK = 113,
	GBMS_STATS_BD_TI_TEMP_PRETRIGGER = 114,
	GBMS_STATS_BD_TI_TEMP_RESUME = 115,

	GBMS_STATS_BD_TI_TRICKLE_CLEARED = 122,
	GBMS_STATS_BD_TI_DOCK_CLEARED = 123,
	GBMS_STATS_TEMP_FILTER = 124,
};

/* health state */
struct batt_chg_health {
	int rest_soc;		/* entry criteria */
	int rest_voltage;	/* entry criteria */
	int always_on_soc;	/* entry criteria */

	ktime_t rest_deadline;	/* full by this in seconds */
	ktime_t dry_run_deadline; /* full by this in seconds (prediction) */
	int rest_rate;		/* centirate once enter */
	int rest_rate_before_trigger;

	enum chg_health_state rest_state;
	int rest_cc_max;
	int rest_fv_uv;
	ktime_t active_time;
};

#define CHG_HEALTH_REST_IS_ACTIVE(rest) \
	((rest)->rest_state == CHG_HEALTH_ACTIVE)

#define CHG_HEALTH_REST_IS_PAUSE(rest) \
	((rest)->rest_state == CHG_HEALTH_PAUSE)

#define CHG_HEALTH_REST_IS_AON(rest, ssoc) \
	(((rest)->rest_state == CHG_HEALTH_ACTIVE) ? \
	(((rest)->always_on_soc != -1) ? \
	(ssoc >= (rest)->always_on_soc) : 0) : 0)

#define CHG_HEALTH_REST_SOC(rest) (((rest)->always_on_soc != -1) ? \
			(rest)->always_on_soc : (rest)->rest_soc)

/* reset on every charge session */
struct gbms_charging_event {
	union gbms_ce_adapter_details	adapter_details;

	/* profile used for this charge event */
	const struct gbms_chg_profile *chg_profile;
	/* charge event and tier tracking */
	struct gbms_ce_stats		charging_stats;
	struct gbms_ce_tier_stats	tier_stats[GBMS_STATS_TIER_COUNT];

	/* soc tracking for time to full */
	struct ttf_soc_stats soc_stats;
	int last_soc;

	ktime_t first_update;
	ktime_t last_update;
	bool bd_clear_trickle;

	/* health based charging */
	struct batt_chg_health		ce_health;	/* updated on close */
	struct gbms_ce_tier_stats	health_stats;	/* updated in HC */
	struct gbms_ce_tier_stats	health_pause_stats;	/* updated in HCP */
	/* updated on sysfs write */
	struct gbms_ce_tier_stats health_dryrun_stats;

	/* other stats */
	struct gbms_ce_tier_stats full_charge_stats;
	struct gbms_ce_tier_stats high_soc_stats;

	struct gbms_ce_tier_stats overheat_stats;
	struct gbms_ce_tier_stats cc_lvl_stats;
	struct gbms_ce_tier_stats trickle_stats;
	struct gbms_ce_tier_stats temp_filter_stats;
};

#define GBMS_CCCM_LIMITS_SET(profile, ti, vi) \
	profile->cccm_limits[((ti) * profile->volt_nb_limits) + (vi)]

#define GBMS_CCCM_LIMITS(profile, ti, vi) \
	(((ti) >= 0 && (vi) >= 0) ? profile->cccm_limits[((ti) * profile->volt_nb_limits) + (vi)] : 0)

/* newgen charging */
#define GBMS_CS_FLAG_BUCK_EN	BIT(0)
#define GBMS_CS_FLAG_DONE	BIT(1)
#define GBMS_CS_FLAG_CC		BIT(2)
#define GBMS_CS_FLAG_CV		BIT(3)
#define GBMS_CS_FLAG_ILIM	BIT(4)
#define GBMS_CS_FLAG_CCLVL	BIT(5)
#define GBMS_CS_FLAG_DIRECT_CHG	BIT(6)

union gbms_charger_state {
	uint64_t v;
	struct {
		uint8_t flags;
		uint8_t pad;
		uint8_t chg_status;
		uint8_t chg_type;
		uint16_t vchrg;
		uint16_t icl;
	} f;
};

int gbms_init_chg_profile_internal(struct gbms_chg_profile *profile,
			  struct device_node *node, const char *owner_name);
#define gbms_init_chg_profile(p, n) \
	gbms_init_chg_profile_internal(p, n, KBUILD_MODNAME)

void gbms_init_chg_table(struct gbms_chg_profile *profile,
			 struct device_node *node, u32 capacity);

void gbms_free_chg_profile(struct gbms_chg_profile *profile);

void gbms_dump_raw_profile(char *buff, size_t len, const struct gbms_chg_profile *profile, int scale);
#define gbms_dump_chg_profile(buff, len, profile) gbms_dump_raw_profile(buff, len, profile, 1000)

/* newgen charging: charge profile */
int gbms_msc_temp_idx(const struct gbms_chg_profile *profile, int temp);
int gbms_msc_voltage_idx(const struct gbms_chg_profile *profile, int vbatt);
int gbms_msc_round_fv_uv(const struct gbms_chg_profile *profile,
			   int vtier, int fv_uv, int cc_ua);

/* newgen charging: charger flags  */
uint8_t gbms_gen_chg_flags(int chg_status, int chg_type);
/* newgen charging: read/gen charger state  */
int gbms_read_charger_state(union gbms_charger_state *chg_state,
			    struct power_supply *chg_psy);
/* calculate aacr reference capacity */
int gbms_aacr_fade10(const struct gbms_chg_profile *profile, int cycles);

/* logbuffer and syslog */
__printf(5,6)
void gbms_logbuffer_prlog(struct logbuffer *log, int level, int debug_no_logbuffer,
			  int debug_printk_prlog, const char *f, ...);

/* debug/print */
const char *gbms_chg_type_s(int chg_type);
const char *gbms_chg_status_s(int chg_status);
const char *gbms_chg_ev_adapter_s(int adapter);

/* Votables */
#define VOTABLE_MSC_CHG_DISABLE	"MSC_CHG_DISABLE"
#define VOTABLE_MSC_PWR_DISABLE	"MSC_PWR_DISABLE"
#define VOTABLE_MSC_INTERVAL	"MSC_INTERVAL"
#define VOTABLE_MSC_FCC		"MSC_FCC"
#define VOTABLE_MSC_FV		"MSC_FV"
#define VOTABLE_FAN_LEVEL	"FAN_LEVEL"
#define VOTABLE_DEAD_BATTERY	"DEAD_BATTERY"
#define VOTABLE_TEMP_DRYRUN	"MSC_TEMP_DRYRUN"
#define VOTABLE_MDIS		"CHG_MDIS"
#define VOTABLE_THERMAL_LVL	"CHG_THERM_LVL"

#define VOTABLE_CSI_STATUS	"CSI_STATUS"
#define VOTABLE_CSI_TYPE	"CSI_TYPE"

#define VOTABLE_CHARGING_POLICY	"CHARGING_POLICY"
#define VOTABLE_CHARGING_UISOC	"CHARGING_UISOC"

#define VOTABLE_DC_CHG_AVAIL	"DC_AVAIL"
#define REASON_DC_DRV		"DC_DRV"
#define REASON_MDIS		"MDIS"

#define FAN_LVL_UNKNOWN		-1
#define FAN_LVL_NOT_CARE	0
#define FAN_LVL_LOW		1
#define FAN_LVL_MED		2
#define FAN_LVL_HIGH		3
#define FAN_LVL_ALARM		4

/* Binned cycle count */
#define GBMS_CCBIN_CSTR_SIZE	(GBMS_CCBIN_BUCKET_COUNT * 6 + 2)

int gbms_cycle_count_sscan_bc(u16 *ccount, int bcnt, const char *buff);
int gbms_cycle_count_cstr_bc(char *buff, size_t size,
					const u16 *ccount, int bcnt);

#define gbms_cycle_count_sscan(cc, buff) \
	gbms_cycle_count_sscan_bc(cc, GBMS_CCBIN_BUCKET_COUNT, buff)

#define gbms_cycle_count_cstr(buff, size, cc)	\
	gbms_cycle_count_cstr_bc(buff, size, cc, GBMS_CCBIN_BUCKET_COUNT)


/* Time to full */
int ttf_soc_cstr(char *buff, int size, const struct ttf_soc_stats *soc_stats,
		 int start, int end);

int ttf_soc_estimate(ktime_t *res,
		     const struct batt_ttf_stats *stats,
		     const struct gbms_charging_event *ce_data,
		     qnum_t soc, qnum_t last);

void ttf_soc_init(struct ttf_soc_stats *dst);

int ttf_tier_cstr(char *buff, int size, const struct ttf_tier_stat *t_stat);

int ttf_tier_estimate(ktime_t *res,
		      const struct batt_ttf_stats *ttf_stats,
		      int temp_idx, int vbatt_idx,
		      int capacity, int full_capacity);

int ttf_stats_init(struct batt_ttf_stats *stats,
		   struct device *device,
		   int capacity_ma);

void ttf_stats_update(struct batt_ttf_stats *stats,
	 	      struct gbms_charging_event *ce_data,
		      bool force);

int ttf_stats_cstr(char *buff, int size, const struct batt_ttf_stats *stats,
		   bool verbose);

int ttf_stats_sscan(struct batt_ttf_stats *stats,
		    const char *buff, size_t size);

struct batt_ttf_stats *ttf_stats_dup(struct batt_ttf_stats *dst,
				     const struct batt_ttf_stats *src);

void ttf_log(const struct batt_ttf_stats *stats, const char *fmt, ...);

ssize_t ttf_dump_details(char *buf, int max_size,
			 const struct batt_ttf_stats *ttf_stats,
			 int last_soc);

int ttf_pwr_vtier_idx(const struct batt_ttf_stats *stats, int soc);

int ttf_ref_cc(const struct batt_ttf_stats *stats, int soc);

int ttf_pwr_ibatt(const struct gbms_ce_tier_stats *ts);

void ttf_tier_reset(struct batt_ttf_stats *stats);

int gbms_read_aacr_limits(struct gbms_chg_profile *profile,
			  struct device_node *node);

bool chg_state_is_disconnected(const union gbms_charger_state *chg_state);

/* Voltage tier stats */
void gbms_tier_stats_init(struct gbms_ce_tier_stats *stats, int8_t idx);

void gbms_chg_stats_tier(struct gbms_ce_tier_stats *tier,
			 int msc_state, ktime_t elap);

void gbms_stats_update_tier(int temp_idx, int ibatt_ma, int temp, ktime_t elap,
			    int cc, union gbms_charger_state *chg_state,
			    enum gbms_msc_states_t msc_state, int soc_in,
			    struct gbms_ce_tier_stats *tier);

int gbms_tier_stats_cstr(char *buff, int size,
			 const struct gbms_ce_tier_stats *tier_stat,
			 bool verbose);

void gbms_log_cstr_handler(struct logbuffer *log, char *buf, int len);




/*
 * Charger modes
 *
 */

enum gbms_charger_modes {
	GBMS_CHGR_MODE_CHGR_DC	= 0x20,

	GBMS_USB_BUCK_ON	= 0x30,
	GBMS_USB_OTG_ON 	= 0x31,
	GBMS_USB_OTG_FRS_ON	= 0x32,

	GBMS_CHGR_MODE_WLC_TX	= 0x40,

	GBMS_POGO_VIN		= 0x50,
	GBMS_POGO_VOUT		= 0x51,
};

#define GBMS_MODE_VOTABLE "CHARGER_MODE"

/* Battery Health */
enum bhi_algo {
	BHI_ALGO_DISABLED = 0,

	BHI_ALGO_CYCLE_COUNT	= 1, /* bare, just use cycle count */
	BHI_ALGO_ACHI		= 2, /* cap avg from history, no resistance */
	BHI_ALGO_ACHI_B		= 3, /* same as ACHI + bounds check */
	BHI_ALGO_ACHI_RAVG	= 4, /* same as ACHI and google_resistance */
	BHI_ALGO_ACHI_RAVG_B	= 5, /* same as ACHI_RAVG + bounds check */

	/* TODO:
	 * BHI_ALGO_ACHI_QRES	 = 4,  cap avg from history, qual resistance
	 * BHI_ALGO_ACHI_QRES_B	= 21,  same ACHI_QRES + bounds check
	 * BHI_ALGO_GCAP_RAVG	= 40,  google_capacity, google_resistance
	 * BHI_ALGO_GCAP_RAVG_B	= 41,  same as GCAP_RAVG + bounds check
	 */

	BHI_ALGO_MIX_N_MATCH 	= 6,
	BHI_ALGO_DEBUG		= 7,
	BHI_ALGO_INDI		= 8, /* individual conditions check */
	BHI_ALGO_MAX,
};

/*
 * Report battery health from health status (for health hal aidl v2)
 * BH_NOMINAL		: BATTERY_HEALTH_GOOD
 * BH_MARGINAL		: BATTERY_HEALTH_FAIR
 * BH_NEEDS_REPLACEMENT	: BATTERY_HEALTH_DEAD
 * BH_FAILED		: BATTERY_HEALTH_UNSPECIFIED_FAILURE
 * BH_NOT_AVAILABLE	: BATTERY_HEALTH_NOT_AVAILABLE
 */
enum bhi_status {
	BH_UNKNOWN = -1,
	BH_NOMINAL,
	BH_MARGINAL,
	BH_NEEDS_REPLACEMENT,
	BH_FAILED,
	BH_NOT_AVAILABLE,
};

struct bhi_weight {
	int w_ci;
	int w_ii;
	int w_sd;
};

/* Charging Speed */
enum csi_type {
	CSI_TYPE_UNKNOWN = -1,

	CSI_TYPE_None = 0,		// Disconnected
	CSI_TYPE_Fault = 1,		// Internal Failures
	CSI_TYPE_JEITA = 2,		// HW limits (will have HOT or COLD)
	CSI_TYPE_LongLife = 3, 		// DefenderConditions
	CSI_TYPE_Adaptive = 4,		// AdaptiveCharging
	CSI_TYPE_Normal = 5,		// All Good
};

enum csi_status {
	CSI_STATUS_UNKNOWN = -1,

	CSI_STATUS_Health_Cold = 10,	// battery temperature not nominal
	CSI_STATUS_Health_Hot = 11,	// battery temperature not nominal
	CSI_STATUS_System_Thermals = 20,// Thermal engine
	CSI_STATUS_System_Load = 21,	// Load might eventually become thermals
	CSI_STATUS_Adapter_Auth = 30,	// During authentication
	CSI_STATUS_Adapter_Power = 31,	// Low power adapter
	CSI_STATUS_Adapter_Quality = 32,// Adapter or cable (low input voltage)
	CSI_STATUS_Defender_Temp = 40,	// TEMP Defend
	CSI_STATUS_Defender_Dwell = 41,	// DWELL Defend
	CSI_STATUS_Defender_Trickle = 42,
	CSI_STATUS_Defender_Dock = 43,	// Dock Defend
	CSI_STATUS_NotCharging = 100,	// There will be a more specific reason
	CSI_STATUS_Charging = 200,	// All good
};

#define CSI_TYPE_MASK_UNKNOWN		(1 << 0)
#define CSI_TYPE_MASK_NONE		(1 << 1)
#define CSI_TYPE_MASK_FAULT		(1 << 2)
#define CSI_TYPE_MASK_JEITA		(1 << 3)
#define CSI_TYPE_MASK_LONGLIFE		(1 << 4)
#define CSI_TYPE_MASK_ADAPTIVE		(1 << 5)
#define CSI_TYPE_MASK_NORMAL		(1 << 6)

#define CSI_STATUS_MASK_UNKNOWN		(1 << 0)
#define CSI_STATUS_MASK_HEALTH_COLD	(1 << 1)
#define CSI_STATUS_MASK_HEALTH_HOT	(1 << 2)
#define CSI_STATUS_MASK_SYS_THERMALS	(1 << 3)
#define CSI_STATUS_MASK_SYS_LOAD	(1 << 4)
#define CSI_STATUS_MASK_ADA_AUTH	(1 << 5)
#define CSI_STATUS_MASK_ADA_POWER	(1 << 6)
#define CSI_STATUS_MASK_ADA_QUALITY	(1 << 7)
#define CSI_STATUS_MASK_DEFEND_TEMP	(1 << 8)
#define CSI_STATUS_MASK_DEFEND_DWELL	(1 << 9)
#define CSI_STATUS_MASK_DEFEND_TRICLE	(1 << 10)
#define CSI_STATUS_MASK_DEFEND_DOCK	(1 << 11)
#define CSI_STATUS_MASK_NOTCHARGING	(1 << 12)
#define CSI_STATUS_MASK_CHARGING	(1 << 13)

enum charging_state {
       BATTERY_STATUS_UNKNOWN = -1,

       BATTERY_STATUS_NORMAL = 1,
       BATTERY_STATUS_TOO_COLD = 2,
       BATTERY_STATUS_TOO_HOT = 3,
       BATTERY_STATUS_LONGLIFE = 4,
       BATTERY_STATUS_ADAPTIVE = 5,
};

#define LONGLIFE_CHARGE_STOP_LEVEL 80
#define LONGLIFE_CHARGE_START_LEVEL 70
#define ADAPTIVE_ALWAYS_ON_SOC 80

enum charging_policy {
       CHARGING_POLICY_UNKNOWN = -1,

       CHARGING_POLICY_DEFAULT = 1,
       CHARGING_POLICY_LONGLIFE = 2,
       CHARGING_POLICY_ADAPTIVE = 3,
};

/*
 * LONGLIFE takes precedence over AC or AON limits,
 * and AC also must take precedence over the AON limit.
 */
enum charging_policy_vote {
       CHARGING_POLICY_VOTE_UNKNOWN = -1,

       CHARGING_POLICY_VOTE_DEFAULT = 1,
       CHARGING_POLICY_VOTE_ADAPTIVE_AON = 2,
       CHARGING_POLICY_VOTE_ADAPTIVE_AC = 3,
       CHARGING_POLICY_VOTE_LONGLIFE = 4,
};

#define to_cooling_device(_dev)	\
	container_of(_dev, struct thermal_cooling_device, device)

#define DEBUG_ATTRIBUTE_WO(name) \
static const struct file_operations name ## _fops = {	\
	.open	= simple_open,			\
	.llseek	= no_llseek,			\
	.write	= name ## _store,			\
}


#endif  /* __GOOGLE_BMS_H_ */
