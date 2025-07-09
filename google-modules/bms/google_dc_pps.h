/* SPDX-License-Identifier: GPL-2.0 */
/*
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

#ifndef __GOOGLE_DC_PPS_H_
#define __GOOGLE_DC_PPS_H_

#include <linux/usb/pd.h>
#include <linux/device.h>
#include <misc/logbuffer.h>

#define PD_T_PPS_TIMEOUT		9000	/* Maximum of 10 seconds */
#define PD_T_PPS_DEADLINE_S		7

#define PPS_KEEP_ALIVE_MAX	3
#define PPS_ERROR_RETRY_MS 	1000
#define CHG_PPS_VOTER		"pps_chg"

/*
 * There is a similar one in tcpm.c
 * NOTE: we don't really need to replicate the values in tcpm.c in the
 * internal state, we just need to know to set 2 to the TCPM power supply
 * to enable PPS.
 */
enum pps_psy_online_states {
	PPS_PSY_OFFLINE = 0,
	PPS_PSY_FIXED_ONLINE,
	PPS_PSY_PROG_ONLINE,
};

enum pd_pps_stage {
	PPS_NOTSUPP = -1,	/* tried and failed or disconnected */
	PPS_DISABLED = 0,	/* default state, never tried */
	PPS_NONE,		/* try to enable */
	PPS_AVAILABLE,
	PPS_ACTIVE,
};

enum pd_nr_pdo {
	PDO_FIXED_5V = 1,
	PDO_FIXED_HIGH_VOLTAGE,
	PDO_PPS,

	PDO_MAX_SUPP = PDO_PPS,
	PDO_MAX = PDO_MAX_OBJECTS,	/* 7 */
};

struct pd_pps_data {
	struct power_supply *pps_psy;
	void *port_data;

	struct wakeup_source *pps_ws;
	bool stay_awake;

	int nr_src_cap;
	u32 *src_caps;
	u32 snk_pdo[PDO_MAX_OBJECTS];
	unsigned int nr_snk_pdo;
	u32 default_pps_pdo;

	int pd_online;
	enum pd_pps_stage stage;
	unsigned int keep_alive_cnt;
	ktime_t last_update;

	/* from TA */
	int min_uv;
	int max_uv;
	int max_ua;

	/* to TA */
	int out_uv;
	int op_ua;

	/* logging client */
	struct logbuffer *log;
};

/* TODO: device dependent */
#define PD_SNK_MAX_MV			9000
/* TODO: device dependent */
#define PD_SNK_MIN_MV			5000

struct tcpm_port;
struct tcpm_port *chg_get_tcpm_port(struct power_supply *tcpm_psy);


/* */
#define pps_is_disabled(x) (((x) == PPS_NOTSUPP) || ((x) == PPS_DISABLED))

#define pps_name(pps_psy) \
	((pps_psy) && (pps_psy)->desc && (pps_psy)->desc->name ? \
		(pps_psy)->desc->name : "<>")


struct dentry;
int pps_init(struct pd_pps_data *pps_data, struct device *dev,
	     struct power_supply *pps_psy, const char *pps_ws_name);
int pps_init_fs(struct pd_pps_data *pps_data, struct dentry *de);
/* reset state and leave in DISABLED  */
void pps_init_state(struct pd_pps_data *pps_data);
/* free resources  */
void pps_free(struct pd_pps_data *pps_data);

/* Run the PPS state machine   */
int pps_work(struct pd_pps_data *pps, struct power_supply *tcpm_psy);

/* rougly equivalent */
int pps_ping(struct pd_pps_data *pps, struct power_supply *tcpm_psy);
int pps_keep_alive(struct pd_pps_data *pps, struct power_supply *tcpm_psy);

/* update the PPS adapter */
int pps_update_adapter(struct pd_pps_data *pps_data,
		       int pending_uv, int pending_ua,
		       struct power_supply *tcpm_psy);
int pps_check_adapter(struct pd_pps_data *pps,
		      int pending_uv, int pending_ua,
		      struct power_supply *tcpm_psy);

/* */
int pps_prog_offline(struct pd_pps_data *pps, struct power_supply *tcpm_psy);

void pps_adjust_volt(struct pd_pps_data *pps, int mod);

int chg_switch_profile(struct pd_pps_data *pps, struct power_supply *tcpm_psy,
		       bool more_pwr);

int pps_get_apdo_max_power(struct pd_pps_data *pps, unsigned int *ta_idx,
			   unsigned int *ta_max_vol, unsigned int *ta_max_cur,
			   unsigned long *ta_max_pwr);
int pps_get_max_power(struct pd_pps_data *pps_data, unsigned int *ta_max_pwr, bool pd);

bool pps_check_prog_online(struct pd_pps_data *pps_data);
bool pps_prog_check_online(struct pd_pps_data *pps_data,
			   struct power_supply *tcpm_psy);

int pps_get_src_cap(struct pd_pps_data *pps, struct power_supply *tcpm_psy);

void pps_set_logbuffer(struct pd_pps_data *pps_data, struct logbuffer *log);
void pps_log(struct pd_pps_data *pps, const char *fmt, ...);

/* probe */
struct power_supply *pps_get_tcpm_psy(struct device_node *node, size_t size);

int pps_request_pdo(struct pd_pps_data *pps_data, unsigned int ta_idx,
		    unsigned int ta_max_vol, unsigned int ta_max_cur);



#endif /* __GOOGLE_DC_PPS_H_ */
