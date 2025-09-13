/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Platform data for the NXP PCA9468 battery charger driver.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _PCA9468_CHARGER_H_
#define _PCA9468_CHARGER_H_

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

struct pca9468_platform_data {
	int	irq_gpio;		/* GPIO pin that's connected to INT# */
	unsigned int	iin_cfg;	/* Input Current Limit - uA unit */
	unsigned int	iin_cfg_max;	/* from config/dt */
	unsigned int	v_float;	/* V_Float Voltage - uV unit */
	unsigned int	v_float_dt;	/* from config/dt */
	unsigned int 	iin_topoff;	/* Input Topoff current -uV unit */
	/* Switching frequency: 0 - 833kHz, ... , 3 - 980kHz */
	unsigned int 	fsw_cfg;
	/* NTC voltage threshold : 0~2.4V - uV unit */
	unsigned int	ntc_th;

	int		iin_max_offset;
	int		iin_cc_comp_offset;

	unsigned int	ta_max_vol;
	unsigned int	ta_max_vol_cp;

	/* Spread Spectrum settings */
	unsigned int	sc_clk_dither_rate;
	unsigned int	sc_clk_dither_limit;
	bool		sc_clk_dither_en;
	int		ta_max_cur_mult;

#ifdef CONFIG_THERMAL
	const char *usb_tz_name;
#endif
};

/* - PPS Integration Shared definitions ---------------------------------- */

/* AC[0] */
#define P9468_CHGS_VER		1
#define P9468_CHGS_VER_MASK	0xff
/* AC[1] APDO */
/* RS[0] */
#define P9468_CHGS_FLAG_SHIFT	0
#define P9468_CHGS_FLAG_MASK	0xff
#define P9468_CHGS_F_STBY	(1 << 0)
#define P9468_CHGS_F_SHDN	(1 << 1)
#define P9468_CHGS_F_DONE	(1 << 2)
#define P9468_CHGS_PRE_SHIFT	8
#define P9468_CHGS_PRE_MASK	(0xff << P9468_CHGS_PRE_SHIFT)
#define P9468_CHGS_RCPC_SHIFT	16
#define P9468_CHGS_RCPC_MASK	(0xff << P9468_CHGS_RCPC_SHIFT)
#define P9468_CHGS_NC_SHIFT	24
#define P9468_CHGS_NC_MASK	(0xff << P9468_CHGS_NC_SHIFT)
/* RS[1] */
#define P9468_CHGS_OVCC_SHIFT	0
#define P9468_CHGS_OVCC_MASK	(0xffff << P9468_CHGS_OVCC_SHIFT)
#define P9468_CHGS_ADJ_SHIFT	16
#define P9468_CHGS_ADJ_MASK	(0xffff << P9468_CHGS_ADJ_MASK)
/* RS[2] */
#define P9468_CHGS_CC_SHIFT	0
#define P9468_CHGS_CC_MASK	(0xffff << P9468_CHGS_CC_SHIFT)
#define P9468_CHGS_CV_SHIFT	16
#define P9468_CHGS_CV_MASK	(0xffff << P9468_CHGS_CV_SHIFT)
/* RS[3] */
#define P9468_CHGS_CA_SHIFT	0
#define P9468_CHGS_CA_MASK	(0xff << P9468_CHGS_CA_SHIFT)


struct p9468_chg_stats {
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

#define p9468_chg_stats_valid(chg_data) ((chg_data)->valid)

static inline void p9468_chg_stats_update_flags(struct p9468_chg_stats *chg_data, u8 flags)
{
	chg_data->receiver_state[0] |= flags << P9468_CHGS_FLAG_SHIFT;
}

static inline void p9468_chg_stats_set_flags(struct p9468_chg_stats *chg_data, u8 flags)
{
	chg_data->receiver_state[0] &= ~P9468_CHGS_FLAG_MASK;
	p9468_chg_stats_update_flags(chg_data, flags);
}

static inline void p9468_chg_stats_inc_ovcf(struct p9468_chg_stats *chg_data,
					    int ibatt, int cc_max)
{
	const int delta = ibatt - cc_max;

	chg_data->ovc_count++;
	if (delta > chg_data->ovc_max_delta) {
		chg_data->ovc_max_ibatt = ibatt;
		chg_data->ovc_max_delta = delta;
	}
}

/**
 * struct pca9468_charger - pca9468 charger instance
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
 * @prev_iin: Previous IIN ADC of PCA9468, uA
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
struct pca9468_charger {
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
	unsigned int 		charging_state;
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
	int 			new_vfloat;

	int			adc_comp_gain;

	int			retry_cnt;

	struct pca9468_platform_data *pdata;

/* Google Integration Start */
	int pps_index;		/* 0=disabled, 1=tcpm, 2=wireless */
	bool			init_done;

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

#ifdef CONFIG_THERMAL
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
	int	fv_uv;
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

	struct p9468_chg_stats	chg_data;

	struct gvotable_election *dc_avail;
/* Google Integration END */
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
};

/* PPS timers */
#define PCA9468_PDMSG_WAIT_T		250	/* 250ms */
#define PCA9468_PDMSG_RETRY_T		1000	/* 1000ms */
#define PCA9468_PDMSG_WLC_WAIT_T	5000	/* 5000ms */
#define PCA9468_PPS_PERIODIC_T		10000	/* 10000ms */

/* - Core driver  ---------------------------- */

int pca9468_read_adc(const struct pca9468_charger *pca9468, u8 adc_ch);
int pca9468_input_current_limit(struct pca9468_charger *pca9468);

/* - PPS Integration (move to a separate file) ---------------------------- */

/* */
enum {
	PPS_INDEX_DISABLED = 0,
	PPS_INDEX_TCPM = 1,
	PPS_INDEX_WLC,
	PPS_INDEX_MAX,
};

int pca9468_probe_pps(struct pca9468_charger *pca9468_chg);

int pca9468_request_pdo(struct pca9468_charger *pca9468);
int pca9468_usbpd_setup(struct pca9468_charger *pca9468);
int pca9468_send_pd_message(struct pca9468_charger *pca9468, unsigned int msg_type);
int pca9468_get_apdo_max_power(struct pca9468_charger *pca9468,
			       unsigned int ta_max_vol, unsigned int ta_max_cur);
int pca9468_get_apdo_index(struct pca9468_charger *pca9468, unsigned int *ta_max_vol,
			   unsigned int *ta_max_cur, unsigned int *ta_objpos);
int pca9468_send_rx_voltage(struct pca9468_charger *pca9468, unsigned int msg_type);
int pca9468_get_rx_max_power(struct pca9468_charger *pca9468);
int pca9468_set_ta_type(struct pca9468_charger *pca9468, int pps_index);

/* GBMS integration */
struct power_supply *pca9468_get_rx_psy(struct pca9468_charger *pca9468);
int pca9468_get_chg_chgr_state(struct pca9468_charger *pca9468,
			       union gbms_charger_state *chg_state);
int pca9468_is_present(struct pca9468_charger *pca9468);
int pca9468_get_status(struct pca9468_charger *pca9468);
int pca9468_get_charge_type(struct pca9468_charger *pca9468);

extern int debug_printk_prlog;
extern int debug_no_logbuffer;

#define logbuffer_prlog(p, level, fmt, ...)	\
	gbms_logbuffer_prlog(p->log, level, debug_no_logbuffer, debug_printk_prlog, fmt, ##__VA_ARGS__)

/* charge stats */
void p9468_chg_stats_init(struct p9468_chg_stats *chg_data);
int p9468_chg_stats_update(struct p9468_chg_stats *chg_data,
			   const struct pca9468_charger *pca9468);
int p9468_chg_stats_done(struct p9468_chg_stats *chg_data,
			 const struct pca9468_charger *pca9468);
void p9468_chg_stats_dump(const struct pca9468_charger *pca9468);

#endif
