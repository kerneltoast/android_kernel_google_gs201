/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Platform data for the HL7132 battery charger driver.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _HL7132_CHARGER_H_
#define _HL7132_CHARGER_H_

#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/thermal.h>
#include <linux/pm_runtime.h>
#include <linux/kernel.h>

/* Google integration */
#include "gbms_power_supply.h"
#include "google_bms.h"
#include "google_dc_pps.h"

struct hl7132_platform_data {
	int	irq_gpio;		/* GPIO pin that's connected to INT# */
	unsigned int	iin_cfg;	/* Input Current Limit - uA unit */
	unsigned int	iin_cfg_max;	/* from config/dt */
	unsigned int	vbat_reg;	/* vbat_reg Voltage - uV unit */
	unsigned int	vbat_reg_dt;	/* from config/dt */
	unsigned int	iin_topoff;	/* Input Topoff current -uV unit */
	/* Switching frequency: 0 - 500khz, ... , 11 - 1600kHz */
	unsigned int	fsw_cfg;

	/* NTC voltage threshold : 0~2.4V - uV unit */
	unsigned int	ntc_th;

	unsigned int max_init_retry;

	/* Temperature threshold, used as an enable / disable */
	unsigned int ts0_th;
	unsigned int ts1_th;

	int		iin_max_offset;
	int		iin_cc_comp_offset;

	unsigned int	ta_max_vol;
	unsigned int	ta_max_vol_cp;

#if IS_ENABLED(CONFIG_THERMAL)
	const char *usb_tz_name;
#endif

	/* Adjustable TA voltage offset as per HW integration guide section 5.4.1 */
	int ta_vol_offset;
	unsigned int init_sleep;
};

/* - PPS Integration Shared definitions ---------------------------------- */

/* AC[0] */
#define HL7132_CHGS_VER		1
#define HL7132_CHGS_VER_MASK	0xff
/* AC[1] APDO */
/* RS[0] */
#define HL7132_CHGS_FLAG_SHIFT	0
#define HL7132_CHGS_FLAG_MASK	0xff
#define HL7132_CHGS_F_STBY	(1 << 0)
#define HL7132_CHGS_F_SHDN	(1 << 1)
#define HL7132_CHGS_F_DONE	(1 << 2)
#define HL7132_CHGS_PRE_SHIFT	8
#define HL7132_CHGS_PRE_MASK	(0xff << HL7132_CHGS_PRE_SHIFT)
#define HL7132_CHGS_RCPC_SHIFT	16
#define HL7132_CHGS_RCPC_MASK	(0xff << HL7132_CHGS_RCPC_SHIFT)
#define HL7132_CHGS_NC_SHIFT	24
#define HL7132_CHGS_NC_MASK	(0xff << HL7132_CHGS_NC_SHIFT)
/* RS[1] */
#define HL7132_CHGS_OVCC_SHIFT	0
#define HL7132_CHGS_OVCC_MASK	(0xffff << HL7132_CHGS_OVCC_SHIFT)
#define HL7132_CHGS_ADJ_SHIFT	16
#define HL7132_CHGS_ADJ_MASK	(0xffff << HL7132_CHGS_ADJ_MASK)
/* RS[2] */
#define HL7132_CHGS_CC_SHIFT	0
#define HL7132_CHGS_CC_MASK	(0xffff << HL7132_CHGS_CC_SHIFT)
#define HL7132_CHGS_CV_SHIFT	16
#define HL7132_CHGS_CV_MASK	(0xffff << HL7132_CHGS_CV_SHIFT)
/* RS[3] */
#define HL7132_CHGS_CA_SHIFT	0
#define HL7132_CHGS_CA_MASK	(0xff << HL7132_CHGS_CA_SHIFT)


struct hl7132_chg_stats {
	u32 adapter_capabilities[2];
	u32 receiver_state[5];

	bool valid;
	unsigned int ovc_count;
	unsigned int ovc_max_ibatt;
	unsigned int ovc_max_delta;

	unsigned int rcp_count;
	unsigned int nc_count;
	unsigned int pre_count;
	unsigned int ca_count;
	unsigned int cc_count;
	unsigned int cv_count;
	unsigned int adj_count;
	unsigned int stby_count;
	unsigned int iin_loop_count;
};

#define hl7132_chg_stats_valid(chg_data) ((chg_data)->valid)

static inline void hl7132_chg_stats_update_flags(struct hl7132_chg_stats *chg_data, u8 flags)
{
	chg_data->receiver_state[0] |= flags << HL7132_CHGS_FLAG_SHIFT;
}

static inline void hl7132_chg_stats_set_flags(struct hl7132_chg_stats *chg_data, u8 flags)
{
	chg_data->receiver_state[0] &= ~HL7132_CHGS_FLAG_MASK;
	hl7132_chg_stats_update_flags(chg_data, flags);
}

static inline void hl7132_chg_stats_inc_ovcf(struct hl7132_chg_stats *chg_data,
					    int ibatt, int cc_max)
{
	const int delta = ibatt - cc_max;

	chg_data->ovc_count++;
	if (delta > chg_data->ovc_max_delta) {
		chg_data->ovc_max_ibatt = ibatt;
		chg_data->ovc_max_delta = delta;
	}
}

// TODO update docstring
/**
 * struct hl7132_charger - hl7132 charger instance
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
 * @prev_iin: Previous IIN ADC of HL7132, uA
 * @prev_inc: Previous TA voltage or current increment factor
 * @vbat_reg_uv: requested vbat_reg
 * @cc_max: requested charge current max
 * @new_iin: New request input current limit, uA
 * @new_vbat_reg: Request for new vbat_reg
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
struct hl7132_charger {
	struct wakeup_source	*monitor_wake_lock;
	struct mutex		lock;
	struct device		*dev;
	struct regmap		*regmap;
	struct power_supply	*mains;

	struct workqueue_struct	*dc_wq;
	struct delayed_work	timer_work;
	unsigned int		timer_id;
	unsigned long		timer_period;
	unsigned long		last_update_time;

	bool			mains_online;
	unsigned int		charging_state;
	unsigned int		ret_state;

	unsigned int		iin_cc;

	unsigned int		ta_cur;
	unsigned int		ta_vol;
	unsigned int		ta_objpos;

	/* need for APDO switch test */
	unsigned int		prev_ta_cur;
	unsigned int		prev_ta_vol;

	/* same as pps_data */
	unsigned int		ta_max_cur;
	unsigned int		ta_max_vol;
	unsigned long		ta_max_pwr;

	unsigned int		prev_iin;
	unsigned int		prev_inc;

	unsigned int		new_iin;
	int			new_vfloat;

	int			adc_comp_gain;

	int			retry_cnt;

	struct hl7132_platform_data *pdata;

	/* Google Integration Start */
	int pps_index;		/* 0=disabled, 1=tcpm*/
	bool			init_done;
	bool hw_init_done;
	unsigned int hw_init_retry_cnt;

	/*  PPS_wired with TCPM */
	u32			tcpm_phandle;
	const char		*tcpm_psy_name;
	struct power_supply	*pd;
	struct delayed_work	pps_work;
	struct pd_pps_data	pps_data;
	struct logbuffer	*log;

#if IS_ENABLED(CONFIG_THERMAL)
	struct thermal_zone_device *usb_tzd;
#endif

	/* WIRELESS or WIRED */
	int	ta_type;
	/*
	 *	0 - No direct charging
	 *	1 - 2:1 charging mode
	 *	2 - 4:1 charging mode
	 */
	int	chg_mode;

	/* requested charging current and voltage */
	int fv_uv; /* vbat_reg for hl7132 */
	int	cc_max;
	ktime_t	dc_start_time;

	/* monitoring */
	struct power_supply	*batt_psy;

	/* debug */
	struct dentry		*debug_root;
	u32			debug_address;
	int			debug_adc_channel;


	bool wlc_ramp_out_iin;
	u32 wlc_ramp_out_delay;
	u32 wlc_ramp_out_vout_target;

	struct hl7132_chg_stats	chg_data;

	struct gvotable_election *dc_avail;
/* Google Integration END */
	bool			ftm_mode; /* factory test, will ignore usb pps */
};

/* Direct Charging State */
enum {
	DC_STATE_NO_CHARGING,	/* No charging */
	DC_STATE_CHECK_VBAT,	/* Check min battery level */
	DC_STATE_PRESET_DC,	/* Preset TA voltage/current for DC */
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
};

/* PPS timers */
#define HL7132_PDMSG_WAIT_T		250	/* 250ms */
#define HL7132_PDMSG_RETRY_T		1000	/* 1000ms */
#define HL7132_PDMSG_WLC_WAIT_T	5000	/* 5000ms */
#define HL7132_PPS_PERIODIC_T		10000	/* 10000ms */

/* - Core driver  ---------------------------- */

int hl7132_read_adc(struct hl7132_charger *hl7132, u8 adc_ch);
int hl7132_input_current_limit(struct hl7132_charger *hl7132);

/* - PPS Integration (move to a separate file) ---------------------------- */

/* */
enum {
	PPS_INDEX_DISABLED = 0,
	PPS_INDEX_TCPM = 1,
	PPS_INDEX_WLC,
	PPS_INDEX_MAX,
};

int hl7132_probe_pps(struct hl7132_charger *hl7132_chg);

int hl7132_request_pdo(struct hl7132_charger *hl7132);
int hl7132_usbpd_setup(struct hl7132_charger *hl7132);
int hl7132_send_pd_message(struct hl7132_charger *hl7132, unsigned int msg_type);
int hl7132_get_apdo_max_power(struct hl7132_charger *hl7132,
			       unsigned int ta_max_vol, unsigned int ta_max_cur);
int hl7132_get_apdo_index(struct hl7132_charger *hl7132, unsigned int *ta_max_vol,
			   unsigned int *ta_max_cur, unsigned int *ta_objpos);
int hl7132_send_rx_voltage(struct hl7132_charger *hl7132, unsigned int msg_type);
int hl7132_get_rx_max_power(struct hl7132_charger *hl7132);
int hl7132_set_ta_type(struct hl7132_charger *hl7132, int pps_index);

/* GBMS integration */
struct power_supply *hl7132_get_rx_psy(struct hl7132_charger *hl7132);
int hl7132_get_chg_chgr_state(struct hl7132_charger *hl7132,
			       union gbms_charger_state *chg_state);
int hl7132_is_present(struct hl7132_charger *hl7132);
int hl7132_get_status(struct hl7132_charger *hl7132);
int hl7132_get_charge_type(struct hl7132_charger *hl7132);

extern int debug_printk_prlog;
extern int debug_no_logbuffer;

#define logbuffer_prlog(p, level, fmt, ...)	\
	gbms_logbuffer_prlog(p->log, level, debug_no_logbuffer, debug_printk_prlog, \
				fmt, ##__VA_ARGS__)

/* charge stats */
void hl7132_chg_stats_init(struct hl7132_chg_stats *chg_data);
int hl7132_chg_stats_update(struct hl7132_chg_stats *chg_data,
			   const struct hl7132_charger *hl7132);
int hl7132_chg_stats_done(struct hl7132_chg_stats *chg_data,
			 const struct hl7132_charger *hl7132);
void hl7132_chg_stats_dump(const struct hl7132_charger *hl7132);

#endif
