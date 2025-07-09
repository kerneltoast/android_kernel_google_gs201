/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Platform data for the LN8411 battery charger driver.
 */

#ifndef _LN8411_CHARGER_H_
#define _LN8411_CHARGER_H_

#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/thermal.h>
#include <linux/pm_runtime.h>
#include <linux/kernel.h>
#include <linux/gpio/driver.h>

/* Google integration */
#include "gbms_power_supply.h"
#include "google_bms.h"
#include "google_dc_pps.h"

enum ln8411_modes {
	LN8411_FWD4TO1 = 0,
	LN8411_FWD2TO1,
	LN8411_FWD1TO1,
	LN8411_REV1TO4 = 4,
	LN8411_REV1TO2,
	LN8411_REV1TO1,
};

enum ln8411_error {
	LN8411_ERROR_NONE = 0,
	LN8411_ERROR_UCP,
	LN8411_ERROR_NOT_ACTIVE,
	LN8411_ERROR_APDO,
	LN8411_ERROR_LOW_VBATT,
};

struct ln8411_platform_data {
	s32		irq_gpio;	/* GPIO pin that's connected to INT# */
	u32		iin_cfg;	/* Input Current Limit - uA unit */
	u32		iin_cfg_max;	/* from config/dt */
	u32		iin_topoff;	/* Input Topoff current -uV unit */
	s32		iin_max_offset;
	s32		iin_cc_comp_offset;
	u32		ta_max_vol_2_1;
	u32		ta_max_vol_4_1;
	bool		si_fet_ovp_drive;

#if IS_ENABLED(CONFIG_THERMAL)
	const char *usb_tz_name;
#endif
};

/* - PPS Integration Shared definitions ---------------------------------- */

/* AC[0] */
#define LN8411_CHGS_VER		1
#define LN8411_CHGS_VER_MASK	0xff
/* AC[1] APDO */
/* RS[0] */
#define LN8411_CHGS_FLAG_SHIFT	0
#define LN8411_CHGS_FLAG_MASK	0xff
#define LN8411_CHGS_F_STBY	BIT(0)
#define LN8411_CHGS_F_SHDN	BIT(1)
#define LN8411_CHGS_F_DONE	BIT(2)
#define LN8411_CHGS_PRE_SHIFT	8
#define LN8411_CHGS_PRE_MASK	(0xff << LN8411_CHGS_PRE_SHIFT)
#define LN8411_CHGS_RCPC_SHIFT	16
#define LN8411_CHGS_RCPC_MASK	(0xff << LN8411_CHGS_RCPC_SHIFT)
#define LN8411_CHGS_NC_SHIFT	24
#define LN8411_CHGS_NC_MASK	(0xff << LN8411_CHGS_NC_SHIFT)
/* RS[1] */
#define LN8411_CHGS_OVCC_SHIFT	0
#define LN8411_CHGS_OVCC_MASK	(0xffff << LN8411_CHGS_OVCC_SHIFT)
#define LN8411_CHGS_ADJ_SHIFT	16
#define LN8411_CHGS_ADJ_MASK	(0xffff << LN8411_CHGS_ADJ_MASK)
/* RS[2] */
#define LN8411_CHGS_CC_SHIFT	0
#define LN8411_CHGS_CC_MASK	(0xffff << LN8411_CHGS_CC_SHIFT)
#define LN8411_CHGS_CV_SHIFT	16
#define LN8411_CHGS_CV_MASK	(0xffff << LN8411_CHGS_CV_SHIFT)
/* RS[3] */
#define LN8411_CHGS_CA_SHIFT	0
#define LN8411_CHGS_CA_MASK	(0xff << LN8411_CHGS_CA_SHIFT)


struct ln8411_chg_stats {
	u32 adapter_capabilities[2];
	u32 receiver_state[5];

	u8 valid;
	u32 ovc_count;
	u32 ovc_max_ibatt;
	u32 ovc_max_delta;

	u32 rcp_count;
	u32 nc_count;
	u32 pre_count;
	u32 ca_count;
	u32 cc_count;
	u32 cv_count;
	u32 adj_count;
	u32 stby_count;
};

struct ln8411_chip_info {
	u8 device_id;
	u8 chip_rev;
};

#define ln8411_chg_stats_valid(chg_data) ((chg_data)->valid)

static inline void ln8411_chg_stats_update_flags(struct ln8411_chg_stats *chg_data, u8 flags)
{
	chg_data->receiver_state[0] |= flags << LN8411_CHGS_FLAG_SHIFT;
}

static inline void ln8411_chg_stats_set_flags(struct ln8411_chg_stats *chg_data, u8 flags)
{
	chg_data->receiver_state[0] &= ~LN8411_CHGS_FLAG_MASK;
	ln8411_chg_stats_update_flags(chg_data, flags);
}

static inline void ln8411_chg_stats_inc_ovcf(struct ln8411_chg_stats *chg_data,
					    s32 ibatt, s32 cc_max)
{
	const s32 delta = ibatt - cc_max;

	chg_data->ovc_count++;
	if (delta > chg_data->ovc_max_delta) {
		chg_data->ovc_max_ibatt = ibatt;
		chg_data->ovc_max_delta = delta;
	}
}

/**
 * struct ln8411_charger - ln8411 charger instance
 * @monitor_wake_lock: lock to enter the suspend mode
 * @lock: protects concurrent access to online variables
 * @dev: pointer to device
 * @regmap: pointer to driver regmap
 * @mains: power_supply instance for AC/DC power
 * @dc_wq: work queue for the algorithm and monitor timer
 * @timer_work: timer work for charging
 * @timer_id: timer id for timer_work
 * @timer_period: timer period for timer_work
 * @last_update_time: last update time after sleep
 * @pps_index: psy index
 * @tcpm_psy_name: name of TCPM power supply
 * @tcpm_phandle: lookup for tcpm power supply
 * @pps_work: pps work for PPS periodic time
 * @pps_data: internal data for dc_pps
 * @log: logbuffer
 * @pd: phandle for qualcomm PMI usbpd-phy
 * @wlc_psy_name: power supply for wlc DC
 * @wlc_psy: wlc DC ps
 * @mains_online: is AC/DC input connected
 * @charging_state: direct charging state
 * @ret_state: return direct charging state after DC_STATE_ADJUST_TAVOL is done
 * @iin_cc: input current for the direct charging in cc mode, uA
 * @ta_cur: AC/DC(TA) current, uA
 * @ta_vol: AC/DC(TA) voltage, uV
 * @ta_objpos: AC/DC(TA) PDO object position
 * @ta_max_cur: TA maximum current of APDO, uA
 * @ta_max_vol: TA maximum voltage for the direct charging, uV
 * @ta_max_pwr: TA maximum power, uW
 * @prev_iin: Previous IIN ADC of LN8411, uA
 * @prev_inc: Previous TA voltage or current increment factor
 * @fv_uv: requested float voltage
 * @cc_max: requested charge current max
 * @new_iin: New request input current limit, uA
 * @new_vfloat: Request for new vfloat
 * @adc_comp_gain: adc gain for compensation
 * @retry_cnt: retry counter for re-starting charging if charging stop happens
 * @ta_type: TA type for the direct charging, USBPD TA or Wireless Charger.
 * @chg_mode: supported DC charging mode 2:1 or 4:1 mode
 * @pdata: pointer to platform data
 * @usb_tzd: device for thermal zone
 * @debug_root: debug entry
 * @debug_address: debug register address
 * @debug_adc_channel: ADC channel to read
 * @init_done: true when initialization is complete
 * @dc_start_time: start time (sec since boot) of the DC session
 */
struct ln8411_charger {
	struct wakeup_source	*monitor_wake_lock;
	struct mutex		lock;
	struct device		*dev;
	struct regmap		*regmap;
	struct power_supply	*mains;

	struct workqueue_struct	*dc_wq;
	struct delayed_work	timer_work;
	u32		timer_id;
	unsigned long		timer_period;
	unsigned long		last_update_time;

	bool			mains_online;
	u32 			charging_state;
	u32			ret_state;

	u32			iin_cc;

	u32			ta_cur;
	u32			ta_vol;
	u32			ta_objpos;

	/* need for APDO switch test */
	unsigned int		prev_ta_cur;
	unsigned int		prev_ta_vol;

	/* same as pps_data */
	u32			ta_max_cur;
	u32			ta_max_vol;
	unsigned long		ta_max_pwr;

	u32			prev_iin;
	u32			prev_inc;

	u32			new_iin;
	s32 			new_vfloat;

	s32			adc_comp_gain;

	enum ln8411_error	error;
	s32			retry_cnt;
	s32			ibus_ucp_retry_cnt;
	u8			ibus_ucp_debounce_cnt;
	s32			low_batt_retry_cnt;

	struct ln8411_platform_data *pdata;

	/* Google Integration Start */
	s32 			pps_index;		/* 0=disabled, 1=tcpm, 2=wireless */
	bool			init_done;
	bool			hw_init_done;

	/* PPS_wireless */
	const char 		*wlc_psy_name;
	struct power_supply 	*wlc_psy;
	/*  PPS_wired with TCPM */
	u32			tcpm_phandle;
	const char 		*tcpm_psy_name;
	struct power_supply 	*pd;
	struct delayed_work	pps_work;
	struct pd_pps_data	pps_data;
	struct logbuffer	*log;

#if IS_ENABLED(CONFIG_THERMAL)
	struct thermal_zone_device *usb_tzd;
#endif

	/* WIRELESS or WIRED */
	s32			ta_type;
	/*
	 *	0 - No direct charging
	 *	1 - 2:1 charging mode
	 *	2 - 4:1 charging mode
	 */
	s32			chg_mode;

	/* requested charging current and voltage */
	s32			fv_uv;
	s32			cc_max;
	ktime_t			dc_start_time;

	/* monitoring */
	struct power_supply	*batt_psy;

	/* debug */
	struct dentry		*debug_root;
	u32			debug_address;
	s32			debug_adc_channel;


	bool 			wlc_ramp_out_iin;
	u32 			wlc_ramp_out_delay;
	u32 			wlc_ramp_out_vout_target;

	struct ln8411_chg_stats	chg_data;
	struct gvotable_election *dc_avail;

	u32 			debug_count;
	struct i2c_client 	*client;
	struct ln8411_chip_info chip_info;
	struct attribute_group 	attrs;    /* SysFS attributes */
	struct delayed_work 	init_hw_work;

#if IS_ENABLED(CONFIG_GPIOLIB)
	struct gpio_chip gpio;
#endif
	/* Google Integration END */

	u32			iin_reg;
	u32			vfloat_reg;
	bool			ftm_mode; /* factory test, will ignore usb pps */
};

/* Direct Charging State */
enum {
	DC_STATE_NO_CHARGING,	/* No charging */
	DC_STATE_CHECK_VBAT,	/* Check min battery level */
	DC_STATE_PRESET_DC, 	/* Preset TA voltage/current for DC */
	DC_STATE_CHECK_ACTIVE,	/* Check active status before Adjust CC mode */
	DC_STATE_ADJUST_CC,	/* Adjust CC mode */
	DC_STATE_CC_MODE,	/* Check CC mode status */
	DC_STATE_START_CV,	/* Start CV mode */
	DC_STATE_CV_MODE,	/* Check CV mode status */
	DC_STATE_CHARGING_DONE,	/* Charging Done */
	DC_STATE_ADJUST_TAVOL,	/* Adjust TA voltage, new TA current < 1000mA */
	DC_STATE_ADJUST_TACUR,	/* Adjust TA current, new TA current < 1000mA */
	DC_STATE_ERROR,		/* Error encountered, no charging */
	DC_STATE_MAX,
};

/* PD Message Type */
enum {
	PD_MSG_REQUEST_APDO,
	MSG_REQUEST_FIXED_PDO,
	WCRX_REQUEST_VOLTAGE,
};

/* TA Type for the direct charging */
enum {
	TA_TYPE_UNKNOWN,
	TA_TYPE_USBPD,
	TA_TYPE_WIRELESS,
};

/* Direct Charging Mode for the direct charging */
enum {
	CHG_NO_DC_MODE,
	CHG_2TO1_DC_MODE,
	CHG_4TO1_DC_MODE,
	CHG_1TO2_DC_MODE,
};

/* PPS timers */
#define LN8411_PDMSG_WAIT_T		250	/* 250ms */
#define LN8411_PDMSG_RETRY_T		1000	/* 1000ms */
#define LN8411_PDMSG_WLC_WAIT_T	2000	/* 2000ms */
#define LN8411_PPS_PERIODIC_T		10000	/* 10000ms */
#define LN8411_TA_CONFIG_WAIT_T		(4 * LN8411_PDMSG_WAIT_T)

/* - Core driver  ---------------------------- */

s32 ln8411_input_current_limit(struct ln8411_charger *ln8411);

/* - PPS Integration (move to a separate file) ---------------------------- */

/* */
enum {
	PPS_INDEX_DISABLED = 0,
	PPS_INDEX_TCPM = 1,
	PPS_INDEX_WLC,
	PPS_INDEX_MAX,
};

s32 ln8411_probe_pps(struct ln8411_charger *ln8411_chg);

s32 ln8411_request_pdo(struct ln8411_charger *ln8411);
s32 ln8411_usbpd_setup(struct ln8411_charger *ln8411);
s32 ln8411_send_pd_message(struct ln8411_charger *ln8411, u32 msg_type);
s32 ln8411_get_apdo_max_power(struct ln8411_charger *ln8411,
			    u32 ta_max_vol, u32 ta_max_cur);
int ln8411_get_apdo_index(struct ln8411_charger *ln8411, unsigned int *ta_max_vol,
			  unsigned int *ta_max_cur, unsigned int *ta_objpos);
s32 ln8411_send_rx_voltage(struct ln8411_charger *ln8411, u32 msg_type);
s32 ln8411_get_rx_max_power(struct ln8411_charger *ln8411);
s32 ln8411_set_ta_type(struct ln8411_charger *ln8411, s32 pps_index);

/* GBMS integration */
struct power_supply *ln8411_get_rx_psy(struct ln8411_charger *ln8411);
s32 ln8411_get_chg_chgr_state(struct ln8411_charger *ln8411,
			    union gbms_charger_state *chg_state);
s32 ln8411_is_present(struct ln8411_charger *ln8411);
s32 ln8411_get_status(struct ln8411_charger *ln8411);
s32 ln8411_get_charge_type(struct ln8411_charger *ln8411);
int ln8411_read_adc(struct ln8411_charger *ln8411,
			  const enum ln8411_adc_chan chan);
int get_chip_info(struct ln8411_charger *chg);
int ln8411_check_active(struct ln8411_charger *ln8411);

extern s32 debug_printk_prlog;
extern s32 debug_no_logbuffer;

#define logbuffer_prlog(p, level, fmt, ...)	\
    gbms_logbuffer_prlog(p->log, level, debug_no_logbuffer, debug_printk_prlog, fmt, ##__VA_ARGS__)

/* charge stats */
void ln8411_chg_stats_init(struct ln8411_chg_stats *chg_data);
s32 ln8411_chg_stats_update(struct ln8411_chg_stats *chg_data,
			  const struct ln8411_charger *ln8411);
s32 ln8411_chg_stats_done(struct ln8411_chg_stats *chg_data,
			const struct ln8411_charger *ln8411);
void ln8411_chg_stats_dump(const struct ln8411_charger *ln8411);

#endif
