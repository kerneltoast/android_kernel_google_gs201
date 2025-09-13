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

#include <linux/kernel.h>
#include <linux/printk.h>
#include <linux/of.h>
#include <linux/time.h>
#include <linux/usb/pd.h>
#include <linux/usb/tcpm.h>
#include <linux/alarmtimer.h>
#include "google_bms.h"
#include "google_psy.h"
#include "google_dc_pps.h"
#include <linux/usb/max77759_export.h>

#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#endif

#define PPS_UPDATE_DELAY_MS		2000

void pps_log(struct pd_pps_data *pps, const char *fmt, ...)
{
	va_list args;

	if (!pps || !pps->log)
		return;

	va_start(args, fmt);
	logbuffer_vlog(pps->log, fmt, args);
	va_end(args);
}
// EXPORT_SYMBOL_GPL(pps_log);

/*
 * State is initialized to PPS_DISABLED and SW will enable detect setting
 * ->stage to PPS_NONE and calling pps_work() until the state becomes
 * PPS_ACTIVE or PPS_NOTSUPP.
 */
void pps_init_state(struct pd_pps_data *pps_data)
{
	/* pps_data->src_caps can be NULL */
	if (pps_data->src_caps) {
		if (!pps_data->nr_src_cap)
			pr_err("%s: %s non zero src_caps, zero nr_src_cap\n",
			       __func__, pps_name(pps_data->pps_psy));
		tcpm_put_partner_src_caps(&pps_data->src_caps);
	}

	pps_data->pd_online = PPS_PSY_OFFLINE;
	pps_data->stage = PPS_DISABLED;
	pps_data->keep_alive_cnt = 0;
	pps_data->nr_src_cap = 0;
	pps_data->src_caps = NULL;

	/* not reference counted */
	if (pps_data->stay_awake)
		__pm_relax(pps_data->pps_ws);

}
// EXPORT_SYMBOL_GPL(pps_init_state);

/*
 * pps_psy can be tcpm, wireless or gcpm_pps.
 * NOTE: gcpm_pps route the props to the underlying psy
 */
static int pps_check_type(struct pd_pps_data *pps_data,
			  struct power_supply *pps_psy)
{
	union power_supply_propval pval;
	int ret;

	if (!pps_psy->desc)
		return -EINVAL;

	/* TODO: add POWER_SUPPLY_TYPE_PPS_PORT? */
	pr_debug("%s: name=%s type=%d\n", __func__, pps_name(pps_psy),
		 pps_psy->desc->type);
	if (pps_psy->desc->type == POWER_SUPPLY_TYPE_WIRELESS)
		return true;

	/* NOTE: this keep trying with "slow" adapters */
	ret = power_supply_get_property(pps_psy, POWER_SUPPLY_PROP_USB_TYPE, &pval);
	pr_debug("%s: name=%s type=%d ret=%d\n", __func__, pps_name(pps_psy),
		 pval.intval, ret);
	if (ret == 0 && pval.intval == POWER_SUPPLY_USB_TYPE_PD_PPS)
		return true;
	if (ret == 0 && pval.intval == POWER_SUPPLY_USB_TYPE_PD)
		return true;
	if (ret == 0 && pval.intval == POWER_SUPPLY_USB_TYPE_C)
		return true;

	return false;
}

/* always call with a tcpm_psy */
struct tcpm_port *chg_get_tcpm_port(struct power_supply *tcpm_psy)
{
	union power_supply_propval pval;
	void *port = NULL;
	int ret;

	/* port is valid for a tcpm power supply but not for gcpm */
	ret = power_supply_get_property(tcpm_psy, POWER_SUPPLY_PROP_USB_TYPE, &pval);
	if (ret == 0 && (pval.intval == POWER_SUPPLY_USB_TYPE_PD_PPS ||
			 pval.intval == POWER_SUPPLY_USB_TYPE_PD ||
			 pval.intval == POWER_SUPPLY_USB_TYPE_C))
		port = power_supply_get_drvdata(tcpm_psy);

	/* TODO: make sure that tcpm_psy is really a tcpm source */

	return (struct tcpm_port *)port;
}

/* false when not present or error (either way don't run) */
static enum pd_pps_stage pps_is_avail(struct pd_pps_data *pps,
				      struct power_supply *tcpm_psy)
{
	/* TODO: make silent, check return value and value */
	pps->max_uv = PSY_GET_PROP(tcpm_psy, POWER_SUPPLY_PROP_VOLTAGE_MAX);
	pps->min_uv = PSY_GET_PROP(tcpm_psy, POWER_SUPPLY_PROP_VOLTAGE_MIN);
	pps->max_ua = PSY_GET_PROP(tcpm_psy, POWER_SUPPLY_PROP_CURRENT_MAX);
	pps->out_uv = PSY_GET_PROP(tcpm_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW);
	pps->op_ua = PSY_GET_PROP(tcpm_psy, POWER_SUPPLY_PROP_CURRENT_NOW);
	if (pps->max_uv < 0 || pps->min_uv < 0 || pps->max_ua < 0 ||
		pps->out_uv < 0 || pps->op_ua < 0)
		return PPS_NONE;

	/* TODO: lower the loglevel after the development stage */
	pps_log(pps, "max_v %d, min_v %d, max_c %d, out_v %d, op_c %d",
		pps->max_uv, pps->min_uv, pps->max_ua, pps->out_uv,
		pps->op_ua);

	/* FIXME: set interval to PD_T_PPS_TIMEOUT here may cause timeout */
	return PPS_AVAILABLE;
}

/* make sure that the adapter doesn't revert back to FIXED PDO */
int pps_ping(struct pd_pps_data *pps, struct power_supply *tcpm_psy)
{
	int rc;

	if (!tcpm_psy)
		return -ENODEV;

	/* NOTE: the adapter should already be in PROG_ONLINE */
	rc = PSY_SET_PROP(tcpm_psy, POWER_SUPPLY_PROP_ONLINE, PPS_PSY_PROG_ONLINE);
	if (rc == 0)
		pps->pd_online = PPS_PSY_PROG_ONLINE;
	else if (rc != -EAGAIN && rc != -EOPNOTSUPP)
		pps_log(pps, "failed to ping, ret = %d", rc);

	return rc;
}
// EXPORT_SYMBOL_GPL(pps_ping);

/* */
int pps_get_src_cap(struct pd_pps_data *pps, struct power_supply *tcpm_psy)
{
	struct tcpm_port *port;

	if (!pps || !tcpm_psy)
		return -EINVAL;

	if (pps->nr_src_cap && pps->src_caps) {
		pr_debug("%s: %s using cached nr_src_cap=%d\n", __func__,
			 pps_name(tcpm_psy), pps->nr_src_cap);
		return pps->nr_src_cap;
	}

	port = chg_get_tcpm_port(tcpm_psy);
	if (port) {
		int nr_src_cap;

		if (pps->src_caps)
			pr_debug("%s: %s warning src_caps!=0, nr_src_cap=%d\n",
				 __func__, pps_name(tcpm_psy), pps->nr_src_cap);

		nr_src_cap = tcpm_get_partner_src_caps(port, &pps->src_caps);
		if (nr_src_cap < 0)
			return nr_src_cap;

		pr_debug("%s: %s found nr_src_cap=%d\n", __func__,
			 pps_name(tcpm_psy), nr_src_cap);
		pps->nr_src_cap = nr_src_cap;
	}

	return pps->nr_src_cap;
}
// EXPORT_SYMBOL_GPL(pps_get_src_cap);

bool pps_check_prog_online(struct pd_pps_data *pps_data)
{
	if (!pps_data || !pps_data->pps_psy)
		return false;

	return PSY_GET_PROP(pps_data->pps_psy, POWER_SUPPLY_PROP_ONLINE) ==
	       PPS_PSY_PROG_ONLINE;
}

/*
 * bail if not online and PROG, query source caps and advance to ACTIVE
 * if not there.
 */
bool pps_prog_check_online(struct pd_pps_data *pps_data,
			  struct power_supply *tcpm_psy)
{
	int pd_online = 0;

	if (!pps_data || !tcpm_psy)
		return -ENODEV;

	pd_online = PSY_GET_PROP(tcpm_psy, POWER_SUPPLY_PROP_ONLINE);
	if (pd_online == 0) {
		pps_init_state(pps_data);
		return false;
	}

	if (pd_online != PPS_PSY_PROG_ONLINE)
		goto not_supp;

	/* make the transition to active */
	if (pps_data->stage != PPS_ACTIVE) {
		int rc;

		pps_data->stage = pps_is_avail(pps_data, tcpm_psy);
		if (pps_data->stage != PPS_AVAILABLE) {
			pr_debug("%s: not available\n", __func__);
			goto not_supp;
		}

		rc = pps_ping(pps_data, tcpm_psy);
		if (rc < 0) {
			pr_debug("%s: ping failed %d\n", __func__, rc);
			goto not_supp;
		}

		pps_data->last_update = get_boot_sec();
		rc = pps_get_src_cap(pps_data, tcpm_psy);
		if (rc < 0) {
			pr_debug("%s: no source caps %d\n", __func__, rc);
			goto not_supp;
		}

		pr_debug("%s: online & active nr_src_cap=%d\n",
			 __func__, pps_data->nr_src_cap);

		pps_data->pd_online = PPS_PSY_PROG_ONLINE;
		pps_data->stage = PPS_ACTIVE;
	}

	return true;

not_supp:
	pps_init_state(pps_data);
	pps_data->stage = PPS_NOTSUPP;
	return false;
}
// EXPORT_SYMBOL_GPL(pps_prog_check_online);

/*
 * enable PPS prog mode (Internal), also start the negotiation.
 * <0 when not supported, 0 if supported (and update state)
 */
static int pps_prog_online(struct pd_pps_data *pps,
			   struct power_supply *tcpm_psy)
{
	union power_supply_propval pval = { .intval = PPS_PSY_PROG_ONLINE, };
	int ret;

	ret = power_supply_set_property(tcpm_psy, POWER_SUPPLY_PROP_ONLINE, &pval);
	pr_debug("%s: name=%s ret=%d\n", __func__, pps_name(tcpm_psy), ret);

	if (ret == -EOPNOTSUPP) {
		pps->stage = PPS_NOTSUPP;
	} else if (ret == 0) {
		pps->pd_online = PPS_PSY_PROG_ONLINE;
		pps->stage = PPS_NONE;
	}

	pps->last_update = get_boot_sec();
	return ret;
}

/* Disable PPS prog mode, will end up in PPS_NOTSUP */
int pps_prog_offline(struct pd_pps_data *pps, struct power_supply *tcpm_psy)
{
	union power_supply_propval pval;
	int ret;

	if (!pps || !tcpm_psy)
		return -ENODEV;

	ret = power_supply_get_property(tcpm_psy, POWER_SUPPLY_PROP_ONLINE, &pval);
	if (ret < 0 || pval.intval != PPS_PSY_PROG_ONLINE)
		goto exit_done;

	pval.intval = PPS_PSY_FIXED_ONLINE;
	ret = power_supply_set_property(tcpm_psy, POWER_SUPPLY_PROP_ONLINE, &pval);

exit_done:
	pps_init_state(pps);
	return ret;
}
// EXPORT_SYMBOL_GPL(pps_prog_offline);

void pps_adjust_volt(struct pd_pps_data *pps, int mod)
{
	if (!pps)
		return;

	if (mod > 0) {
		pps->out_uv = (pps->out_uv + mod) < pps->max_uv ?
			      (pps->out_uv + mod) : pps->max_uv;
	} else if (mod < 0) {
		pps->out_uv = (pps->out_uv + mod) > pps->min_uv ?
			      (pps->out_uv + mod) : pps->min_uv;
	}
}


/* ------------------------------------------------------------------------ */

static int debug_get_pps_out_uv(void *data, u64 *val)
{
	struct pd_pps_data *pps_data = data;

	*val = pps_data->out_uv;
	return 0;
}

static int debug_set_pps_out_uv(void *data, u64 val)
{
	struct pd_pps_data *pps_data = data;

	/* TODO: use votable */
	pps_data->out_uv = val;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_pps_out_uv_fops,
				debug_get_pps_out_uv,
				debug_set_pps_out_uv, "%llu\n");

static int debug_get_pps_op_ua(void *data, u64 *val)
{
	struct pd_pps_data *pps_data = data;

	*val = pps_data->op_ua;
	return 0;
}

static int debug_set_pps_op_ua(void *data, u64 val)
{
	struct pd_pps_data *pps_data = data;

	/* TODO: use votable */
	pps_data->op_ua = val;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_pps_op_ua_fops,
					debug_get_pps_op_ua,
					debug_set_pps_op_ua, "%llu\n");

int pps_init_fs(struct pd_pps_data *pps_data, struct dentry *de)
{

	if (!de || !pps_data)
		return -EINVAL;

	debugfs_create_file("pps_out_uv", 0600, de, pps_data,
			    &debug_pps_out_uv_fops);
	debugfs_create_file("pps_out_ua", 0600, de, pps_data,
			    &debug_pps_op_ua_fops);
	debugfs_create_file("pps_op_ua", 0600, de, pps_data,
			    &debug_pps_op_ua_fops);

	return 0;
}

void pps_set_logbuffer(struct pd_pps_data *pps_data, struct logbuffer *log)
{
	pps_data->log = IS_ERR(log) ? NULL : log;
}

/* ------------------------------------------------------------------------ */

static int pps_get_sink_pdo(u32 *snk_pdo, int max, struct device_node *node)
{
	struct device_node *dn, *conn;
	unsigned int nr_snk_pdo;
	const __be32 *prop;
	int length, ret;

	prop = of_get_property(node, "google,usbc-connector", NULL);
	if (!prop)
		prop = of_get_property(node, "google,usb-c-connector", NULL);
	if (!prop)
		prop = of_get_property(node, "google,tcpm-power-supply", NULL);
	if (!prop)
		return -ENOENT;

	dn = of_find_node_by_phandle(be32_to_cpup(prop));
	if (!dn) {
		pr_err("Couldn't find usb_con node\n");
		return -ENOENT;
	}

	conn = of_get_child_by_name(dn, "connector");
	if (conn) {
		of_node_put(dn);
		dn = conn;
	}

	prop = of_get_property(dn, "sink-pdos", &length);
	if (!prop) {
		pr_err("Couldn't find sink-pdos property\n");
		of_node_put(dn);
		return -ENOENT;
	}

	nr_snk_pdo = length / sizeof(u32);
	if (nr_snk_pdo == 0 || nr_snk_pdo > max) {
		pr_err("Invalid length of sink-pdos\n");
		of_node_put(dn);
		return -EINVAL;
	}

	ret = of_property_read_u32_array(dn, "sink-pdos", snk_pdo, nr_snk_pdo);
	of_node_put(dn);
	if (ret < 0)
		return ret;

	return nr_snk_pdo;
}

static int pps_find_apdo(struct pd_pps_data *pps_data, int nr_snk_pdo)
{
	int i;

	for (i = 0; i < nr_snk_pdo; i++) {
		u32 pdo = pps_data->snk_pdo[i];

		if (pdo_type(pdo) != PDO_TYPE_FIXED)
			pr_debug("%s %d type=%d", __func__, i, pdo_type(pdo));
		else
			pr_debug("%s %d FIXED v=%d c=%d", __func__, i,
				 pdo_fixed_voltage(pdo),
				 pdo_max_current(pdo));

		if (pdo_type(pdo) == PDO_TYPE_APDO) {
			/* TODO: check APDO_TYPE_PPS */
			pps_data->default_pps_pdo = pdo;
			return 0;
		}
	}

	return -ENODATA;
}

static int pps_init_snk(struct pd_pps_data *pps_data,
			 struct device_node *node)
{
	int ret, nr_snk_pdo = 0;

	memset(pps_data, 0, sizeof(*pps_data));

	nr_snk_pdo = pps_get_sink_pdo(pps_data->snk_pdo, PDO_MAX_OBJECTS, node);
	if (nr_snk_pdo < 0) {
		pr_err("Couldn't find connector property (%d)\n", nr_snk_pdo);
		nr_snk_pdo = 0;
	} else {
		ret = pps_find_apdo(pps_data, nr_snk_pdo);
		if (ret < 0) {
			pr_err("nr_sink_pdo=%d sink APDO not found ret=%d\n",
				nr_snk_pdo, ret);
			return -ENOENT;
		}
	}

	pps_data->nr_snk_pdo = nr_snk_pdo;
	return 0;
}

/* Look for the connector and retrieve source capabilities.
 * pps_data->nr_snk_pdo == 0 means no PPS
 */
int pps_init(struct pd_pps_data *pps_data, struct device *dev,
	     struct power_supply *pps_psy, const char *pps_ws_name)
{
	int ret;

	if (!pps_data || !pps_psy || !dev)
		return -EINVAL;

	ret = pps_init_snk(pps_data, dev->of_node);
	if (ret < 0)
		return ret;

	/* TODO: look in the power supply device node ? maybe? */
	if (!pps_data->nr_snk_pdo)
		dev_warn(dev, "%s has nr_sink_pdo=0\n", pps_name(pps_psy));

	/*
	 * The port needs to ping or update the PPS adapter every 10 seconds
	 * (maximum). However, Qualcomm PD phy returns error when system is
	 * waking up. To prevent the timeout when system is resumed from
	 * suspend, hold a wakelock while PPS is active.
	 *
	 * Remove this wakeup source once we fix the Qualcomm PD phy issue.
	 */
	pps_data->stay_awake = of_property_read_bool(dev->of_node,
						     "google,pps-awake");
	if (pps_data->stay_awake) {
		pps_data->pps_ws = wakeup_source_register(NULL, pps_ws_name);
		if (!pps_data->pps_ws) {
			dev_err(dev, "Failed to register wakeup source\n");
			return -ENODEV;
		}
	}

	pps_data->pps_psy = pps_psy;
	return 0;
}

void pps_free(struct pd_pps_data *pps_data)
{
	if (!pps_data || !pps_data->pps_psy)
		return;
	if (pps_data->pps_ws)
		wakeup_source_unregister(pps_data->pps_ws);
	pps_data->pps_psy = NULL;
}


/* ------------------------------------------------------------------------- */

/*
 * This is the first part of the DC/PPS charging state machine.
 * Detect and configure the PPS adapter for the PPS profile.
 * pps->stage is updated to PPS_NONE, PPS_AVAILABLE, PPS_ACTIVE or
 * PPS_NOTSUPP.
 *
 * Returns:
 * . 0 to disable the PPS update interval voter
 * . <0 for error
 * . the max update interval pps should request
 */
int pps_work(struct pd_pps_data *pps, struct power_supply *pps_psy)
{
	union power_supply_propval pval;
	int ret, type_ok, pd_online;
	enum pd_pps_stage stage;

	if (!pps)
		return -EINVAL;
	if (!pps_psy) {
		pps->stage = PPS_NOTSUPP;
		return -EINVAL;
	}

	/*
	 * POWER_SUPPLY_PROP_PRESENT must reports cable (or field)
	 * NOTE: TCPM doesn't implement this, WLC and GCPM pps do.
	 */
	ret = power_supply_get_property(pps_psy, POWER_SUPPLY_PROP_PRESENT,
					&pval);
	if (ret == 0 && pval.intval == 0) {
		pr_debug("%s: %s pval.intval=%d ret=%d\n", __func__,
			 pps_name(pps_psy), pval.intval, ret);
		pps->stage = PPS_NOTSUPP;
	}

	/* detection is done for this cycle */
	if (pps->stage == PPS_NOTSUPP)
		return 0;

	/*
	 * 2) pps->pd_online == PPS_PSY_PROG_ONLINE && stage == PPS_NONE
	 *  If the source really support PPS (set in 1): set stage to
	 *  PPS_AVAILABLE and reschedule after PD_T_PPS_TIMEOUT
	 */
	if (pps->stage == PPS_NONE && pps->pd_online == PPS_PSY_PROG_ONLINE) {
		int rc;

		pps->stage = pps_is_avail(pps, pps_psy);
		if (pps->stage == PPS_AVAILABLE) {
			rc = pps_ping(pps, pps_psy);
			if (rc < 0) {
				pps->pd_online = PPS_PSY_FIXED_ONLINE;
				return 0;
			}

			if (pps->stay_awake)
				__pm_stay_awake(pps->pps_ws);

			pps->last_update = get_boot_sec();
			rc = pps_get_src_cap(pps, pps_psy);
			if (rc < 0)
				pps_log(pps, "Cannot get partner src caps");
		}

		return PD_T_PPS_TIMEOUT;
	}

	/*
	 * no need to retry (error) when I cannot read POWER_SUPPLY_PROP_ONLINE.
	 * The prop is set to PPS_PSY_PROG_ONLINE (from PPS_PSY_FIXED_ONLINE)
	 * when usbc_type is POWER_SUPPLY_USB_TYPE_PD_PPS.
	 */
	pd_online = PSY_GET_PROP(pps_psy, POWER_SUPPLY_PROP_ONLINE);
	if (pd_online < 0)
		return 0;

	/*
	 * 3) pd_online == PPS_PSY_PROG_ONLINE == pps->pd_online
	 * pps is active now, we are done here. pd_online will change to
	 * if pd_online is !PPS_PSY_PROG_ONLINE go back to 1) OR exit.
	 */
	stage = (pd_online == pps->pd_online) &&
		     (pd_online == PPS_PSY_PROG_ONLINE) &&
		     (pps->stage == PPS_AVAILABLE || pps->stage == PPS_ACTIVE) ?
		     PPS_ACTIVE : PPS_NONE;
	if (stage != pps->stage) {
		pps_log(pps, "work: pd_online %d->%d stage %d->%d",
			pps->pd_online, pd_online, pps->stage,
			stage);
		pps->stage = stage;
	}

	if (pps->stage == PPS_ACTIVE)
		return 0;

	/*
	 * 1) stage == PPS_NONE && pps->pd_online!=PPS_PSY_PROG_ONLINE
	 *  If usbc_type is ok and pd_online is PPS_PSY_FIXED_ONLINE,
	 *  set POWER_SUPPLY_PROP_ONLINE to PPS_PSY_PROG_ONLINE (enable PPS)
	 *  and reschedule in PD_T_PPS_TIMEOUT.
	 */
	type_ok = pps_check_type(pps, pps_psy);
	if (!type_ok)
		pr_debug("%s: %s type not ok\n", __func__, pps_name(pps_psy));

	if (type_ok && pd_online == PPS_PSY_FIXED_ONLINE) {
		int rc;

		/* 0 = when in PROG */
		rc = pps_prog_online(pps, pps_psy);
		switch (rc) {
		case 0:
			return PD_T_PPS_TIMEOUT;
		case -EAGAIN:
			pps_log(pps, "work: not in SNK_READY, rerun");
			pps->pd_online = pd_online;
			return rc;
		case -EOPNOTSUPP:
			pps->stage = PPS_NOTSUPP;
			break;
		default:
			pps_log(pps, "work: PROP_ONLINE (%d)", rc);
			break;
		}
	}

	/* reset PPS_NOTSUPP with pps_init_state() */
	if (pps->stage == PPS_NOTSUPP) {
		if (pps->stay_awake)
			__pm_relax(pps->pps_ws);
		pps_log(pps, "work: PPS not supported");
	}

	pps->pd_online = pd_online;
	return 0;
}
// EXPORT_SYMBOL_GPL(pps_work);

int pps_keep_alive(struct pd_pps_data *pps, struct power_supply *tcpm_psy)
{
	int ret;

	if (!pps || !tcpm_psy)
		return -EINVAL;

	ret = pps_ping(pps, tcpm_psy);
	if (ret < 0) {
		pps->pd_online = PPS_PSY_FIXED_ONLINE;
		pps->keep_alive_cnt = 0;
		return ret;
	}

	pps->keep_alive_cnt += (pps->keep_alive_cnt < UINT_MAX);
	pps->last_update = get_boot_sec();
	return 0;
}
// EXPORT_SYMBOL_GPL(pps_keep_alive);

int pps_check_adapter(struct pd_pps_data *pps,
		      int pending_uv, int pending_ua,
		      struct power_supply *tcpm_psy)
{
	const int scale = 50000; /* HACK */
	int out_uv, op_ua;

	if (!pps || !tcpm_psy)
		return -EINVAL;

	out_uv = PSY_GET_PROP(tcpm_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW);
	op_ua = PSY_GET_PROP(tcpm_psy, POWER_SUPPLY_PROP_CURRENT_NOW);

	pr_debug("%s: mv=%d->%d ua=%d,%d\n", __func__,
		out_uv, pending_uv, op_ua, pending_ua);

	if (out_uv < 0 || op_ua < 0)
		return -EIO;

	return (out_uv / scale) >= (pending_uv /scale) &&
	       (op_ua / scale) >= (pending_ua / scale);
}

static int pps_set_prop(struct pd_pps_data *pps,
			enum power_supply_property prop, int val,
			struct power_supply *tcpm_psy)
{
	int ret;

	ret = PSY_SET_PROP(tcpm_psy, prop, val);
	if (ret == 0) {
		pps->keep_alive_cnt = 0;
	} else if (ret == -EOPNOTSUPP) {
		pps->pd_online = PPS_PSY_FIXED_ONLINE;
		pps->keep_alive_cnt = 0;
		if (pps->stay_awake)
			__pm_relax(pps->pps_ws);
	}

	return ret;
}

static void pps_log_keepalive(struct pd_pps_data *pps)
{
	pps_log(pps, "%d KEEP ALIVE", pps->keep_alive_cnt);
	pps->keep_alive_cnt = 0;
}

/*
 * return negative values on errors
 * return PD_T_PPS_TIMEOUT after successful updates or pings
 * return PPS_UPDATE_DELAY_MS when the update interval is less than
 *	  PPS_UPDATE_DELAY_MS
 * return the delta to the nex ping deadline otherwise
 */
int pps_update_adapter(struct pd_pps_data *pps,
		       int pending_uv, int pending_ua,
		       struct power_supply *tcpm_psy)
{
	int interval = get_boot_sec() - pps->last_update;
	int ret;

	if (!tcpm_psy)
		return -EINVAL;

	pps->out_uv = PSY_GET_PROP(tcpm_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW);
	pps->op_ua = PSY_GET_PROP(tcpm_psy, POWER_SUPPLY_PROP_CURRENT_NOW);
	if (pps->out_uv < 0 || pps->op_ua < 0) {
		pr_debug("%s: %s error out_uv=%d op_ua=%d\n", __func__,
			 pps_name(tcpm_psy), pps->out_uv, pps->op_ua);
		return -EIO;
	}

	if (pending_uv < 0)
		pending_uv = pps->out_uv;
	if (pending_ua < 0)
		pending_ua = pps->op_ua;

	/*
	 * TCPM accepts one change per power negotiation cycle.
	 * TODO: change voltage, current or current, voltage depending on
	 *       the values.
	 */
	if (pps->op_ua != pending_ua) {

		if (pps->keep_alive_cnt)
			pps_log_keepalive(pps);

		ret = pps_set_prop(pps, POWER_SUPPLY_PROP_CURRENT_NOW,
				   pending_ua, tcpm_psy);
		pr_debug("%s: %s SET_UA out_ua %d->%d, ret=%d", __func__,
			pps_name(tcpm_psy), pps->op_ua, pending_ua, ret);

		pps_log(pps, "SET_UA out_ua %d->%d, ret=%d",
			pps->op_ua, pending_ua, ret);

		if (ret == 0) {
			pps->last_update = get_boot_sec();
			pps->op_ua = pending_ua;

			/* voltage is pending too */
			if (pps->out_uv != pending_uv)
				return 0;

			return PD_T_PPS_TIMEOUT;
		}

		if (ret != -EAGAIN && ret != -EOPNOTSUPP)
			pps_log(pps, "failed to set CURRENT_NOW, ret = %d",
				ret);
	} else if (pps->out_uv != pending_uv) {
		ret = pps_set_prop(pps, POWER_SUPPLY_PROP_VOLTAGE_NOW,
				   pending_uv,  tcpm_psy);

		if (pps->keep_alive_cnt)
			pps_log_keepalive(pps);

		pr_debug("%s: %s SET_UV out_v %d->%d, ret=%d\n", __func__,
			pps_name(tcpm_psy), pps->out_uv, pending_uv, ret);
		pps_log(pps, "SET_UV out_v %d->%d, ret=%d",
			pps->out_uv, pending_uv, ret);

		if (ret == 0) {
			pps->last_update = get_boot_sec();
			pps->out_uv = pending_uv;
			return PD_T_PPS_TIMEOUT;
		}

		if (ret != -EAGAIN && ret != -EOPNOTSUPP)
			pps_log(pps, "failed to set VOLTAGE_NOW, ret = %d",
				ret);

	} else if (interval < PD_T_PPS_DEADLINE_S) {
		pr_debug("%s: %s mv=%d->%d ua=%d->%d interval=%d\n", __func__,
			pps_name(tcpm_psy), pps->out_uv, pending_uv,
			pps->op_ua, pending_ua, interval);
		/* TODO: tune this, now assume that PD_T_PPS_TIMEOUT >= 7s */
		return PD_T_PPS_TIMEOUT - (interval * MSEC_PER_SEC);
	} else {
		ret = pps_keep_alive(pps, tcpm_psy);
		if (ret < 0) {
			if (pps->keep_alive_cnt)
				pps_log_keepalive(pps);
			pps_log(pps, "KEEP ALIVE out_v %d, op_c %d (%d)",
				pps->out_uv, pps->op_ua, ret);
		} else {
			pps->keep_alive_cnt += 1;
		}

		pr_debug("%s: %s KEEP ALIVE out_v %d, op_c %d (%d)", __func__,
			pps_name(tcpm_psy), pps->out_uv, pps->op_ua, ret);

		if (ret == 0)
			return PD_T_PPS_TIMEOUT;
	}

	if (ret == -EOPNOTSUPP)
		pps_log(pps, "PPS deactivated while updating");

	return ret;
}
// EXPORT_SYMBOL_GPL(pps_update_adapter);

/* just a wrapper for power_supply_get_by_phandle_array() */
struct power_supply *pps_get_tcpm_psy(struct device_node *node, size_t size)
{
	const char *propname = "google,tcpm-power-supply";
	struct power_supply *tcpm_psy = NULL;
	struct power_supply *psy[2];
	int i, ret;

	if (!node)
		return ERR_PTR(-EINVAL);

	ret = power_supply_get_by_phandle_array(node, propname, psy,
						ARRAY_SIZE(psy));
	if (ret < 0)
		return ERR_PTR(-EAGAIN);

	for (i = 0; i < ret; i++) {
		const char *name = psy[i]->desc ? psy[i]->desc->name : NULL;

		if (!tcpm_psy && name && !strncmp(name, "tcpm", 4))
			tcpm_psy = psy[i];
		else
			power_supply_put(psy[i]);
	}

	return tcpm_psy;
}
// EXPORT_SYMBOL_GPL(pps_get_tcpm_psy);

/* TODO:  */
int pps_request_pdo(struct pd_pps_data *pps_data, unsigned int ta_idx,
		    unsigned int ta_max_vol, unsigned int ta_max_cur)
{
	const unsigned int max_mw = ta_max_vol * ta_max_cur;
	struct tcpm_port *port;
	int ret = -ENODEV;

	if (!pps_data || !pps_data->pps_psy || ta_idx > PDO_MAX_OBJECTS)
		return -EINVAL;

	/* tcpm pport */
	port = chg_get_tcpm_port(pps_data->pps_psy);
	if (port)
		ret = tcpm_update_sink_capabilities(port, pps_data->snk_pdo,
						    ta_idx, max_mw);


	return ret;
}
// EXPORT_SYMBOL_GPL(pps_request_pdo);


/* ------------------------------------------------------------------------- */

/*
 * return the first APDO from the TCPM source which voltage greater or equal
 * to *ta_max_vol and current greater or equal to *ta_max_cur.
 * NOTE: 0 in ta_max_vol and ta_max_cur will select the first APDO.
 */
int pps_get_apdo_max_power(struct pd_pps_data *pps_data, unsigned int *ta_idx,
			   unsigned int *ta_max_vol, unsigned int *ta_max_cur,
			   unsigned long *ta_max_pwr)
{
	int max_current, max_voltage, max_power;
	const int ta_max_vol_mv = *ta_max_vol / 1000;
	const int ta_max_cur_mv = *ta_max_cur / 1000;
	int i;

	if (!pps_data)
		return -EINVAL;

	if (pps_data->nr_src_cap <= 0)
		return -ENOENT;

	/* already done that */
	if (*ta_idx != 0)
		return -ENOTSUPP;

	/* find the new max_uv, max_ua, and max_pwr */
	for (i = 0; i < pps_data->nr_src_cap; i++) {
		const u32 pdo = pps_data->src_caps[i];

		if (pdo_type(pdo) != PDO_TYPE_FIXED)
			continue;

		max_voltage = pdo_fixed_voltage(pdo); /* mV */
		max_current = pdo_max_current(pdo); /* mA */
		max_power = pdo_max_power(pdo); /* mW */
		*ta_max_pwr = max_power * 1000; /* uW */
	}

	/* Get the first APDO that that exceeds the limits */
	for (i = 0; i < pps_data->nr_src_cap; i++) {
		const u32 pdo = pps_data->src_caps[i];
		bool voltage_ok, current_ok;

		if (pdo_type(pdo) != PDO_TYPE_APDO)
			continue;

		max_current = pdo_pps_apdo_max_current(pdo); /* mA */
		max_voltage = pdo_pps_apdo_max_voltage(pdo); /* mV */

		/* stop on first match */
		voltage_ok = max_voltage >= ta_max_vol_mv;
		current_ok = max_current >= ta_max_cur_mv;
		if (voltage_ok && current_ok) {
			*ta_max_vol = max_voltage * 1000;	/* uV */
			*ta_max_cur = max_current * 1000;	/* uA */
			*ta_idx = i + 1;
			return 0;
		}

	}

	pr_debug("%s: max_uv (%u) and max_ua (%u) out of APDO src caps\n",
		 __func__, *ta_max_vol, *ta_max_cur);
	return -EINVAL;
}
// EXPORT_SYMBOL_GPL(pps_get_apdo_max_power);

int pps_get_max_power(struct pd_pps_data *pps_data, unsigned int *ta_max_pwr, bool pd)
{
	int max_current, max_voltage;
	unsigned int max_power;
	int i;

	if (pps_data->nr_src_cap <= 0)
		return -ENOENT;

	*ta_max_pwr = 0;

	/* find the new max_uv, max_ua, and max_pwr */
	for (i = 0; i < pps_data->nr_src_cap; i++) {
		const u32 pdo = pps_data->src_caps[i];

		if (pd && pdo_type(pdo) == PDO_TYPE_FIXED) {
			max_voltage = pdo_fixed_voltage(pdo); /* mV */
			max_current = pdo_max_current(pdo); /* mA */
		} else if (!pd && pdo_type(pdo) == PDO_TYPE_APDO) {
			max_voltage = pdo_pps_apdo_max_voltage(pdo); /* mV */
			max_current = pdo_pps_apdo_max_current(pdo); /* mA */
		}

		max_power = max_voltage * max_current / 1000;

		if (max_power > *ta_max_pwr)
			*ta_max_pwr = max_power; /* uW */

	}

	pr_debug("%s: max_power: %u\n", __func__, *ta_max_pwr);
	return 0;
}
