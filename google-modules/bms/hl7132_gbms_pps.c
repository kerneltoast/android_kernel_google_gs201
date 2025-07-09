// SPDX-License-Identifier: GPL-2.0
/*
 * HL7132 Direct Charger PPS Integration
 *
 * Copyright (C) 2021 Google, LLC
 *
 */


#include <linux/err.h>
#include <linux/init.h>
#include <linux/version.h>
#include <linux/delay.h>
#include <linux/dev_printk.h>
#include <linux/of_device.h>
#include <linux/regmap.h>

#include "hl7132_regs.h"
#include "hl7132_charger.h"

/* Logging ----------------------------------------------------------------- */

int debug_printk_prlog = LOGLEVEL_INFO;
int debug_no_logbuffer;

/* DC PPS integration ------------------------------------------------------ */

static void hl7132_chg_stats_set_apdo(struct hl7132_chg_stats *chg_data, u32 apdo);

static struct device_node *hl7132_find_config(struct device_node *node)
{
	struct device_node *temp;

	if (!node)
		return node;
	temp = of_parse_phandle(node, "hl7132,google_cpm", 0);
	if (temp)
		node = temp;
	return node;
}

int hl7132_probe_pps(struct hl7132_charger *hl7132_chg)
{
	bool pps_available = false;
	struct device_node *node;
	int ret;

	node = hl7132_find_config(hl7132_chg->dev->of_node);
	if (!node)
		return -ENODEV;

	ret = of_property_read_u32(node, "google,tcpm-power-supply",
				   &hl7132_chg->tcpm_phandle);
	if (ret < 0)
		dev_warn(hl7132_chg->dev,
			"hl7132: google,tcpm-power-supply not defined\n");
	else
		pps_available |= true;

	return pps_available ? 0 : -ENODEV;
}

/* ------------------------------------------------------------------------ */

/* switch PDO if needed */
int hl7132_request_pdo(struct hl7132_charger *hl7132)
{
	int ret = 0;

	dev_dbg(hl7132->dev, "%s: ta_objpos=%u, ta_vol=%u, ta_cur=%u\n",
		__func__, hl7132->ta_objpos, hl7132->ta_vol, hl7132->ta_cur);

	/*
	 * the reference implementation call pps_request_pdo() twice with a
	 * 100 ms delay between the calls when the function returns -EBUSY:
	 *
	 *	ret = pps_request_pdo(&hl7132->pps_data, hl7132->ta_objpos,
	 *				hl7132->ta_vol, hl7132->ta_cur,
	 *				hl7132->pd);
	 *
	 * The wrapper in google_dc_pps route the calls to the tcpm engine
	 * via tcpm_update_sink_capabilities(). The sync capabilities are
	 * in pps_data, ->ta_objpos select the (A)PDO index, ->ta_vol and
	 * ->ta_cur are the desired TA voltage and current.
	 *
	 * this is now handled by pps_update_adapter()
	 *
	 * TODO: verify the timing and make sure that there are no races that
	 * cause the targets
	 */

	return ret;
}

int hl7132_usbpd_setup(struct hl7132_charger *hl7132)
{
	struct power_supply *tcpm_psy;
	bool online;
	int ret = 0;

	if (hl7132->pd != NULL)
		goto check_online;

	if (hl7132->tcpm_psy_name) {
		tcpm_psy = power_supply_get_by_name(hl7132->tcpm_psy_name);
		if (!tcpm_psy)
			return -ENODEV;

		hl7132->pd = tcpm_psy;
	} else if (hl7132->tcpm_phandle) {
		struct device_node *node;

		node = hl7132_find_config(hl7132->dev->of_node);
		if (!node)
			return -ENODEV;

		tcpm_psy = pps_get_tcpm_psy(node, 2);
		if (IS_ERR(tcpm_psy))
			return PTR_ERR(tcpm_psy);
		if (!tcpm_psy) {
			hl7132->tcpm_phandle = 0;
			return -ENODEV;
		}

		dev_err(hl7132->dev, "%s: TCPM name is %s\n", __func__,
		       pps_name(tcpm_psy));
		hl7132->tcpm_psy_name = tcpm_psy->desc->name;
		hl7132->pd = tcpm_psy;
	} else {
		dev_err(hl7132->dev, "%s: TCPM DC not defined\n", __func__);
		return -ENODEV;
	}

	/* not needed if tcpm-power-supply is not there */
	ret = pps_init(&hl7132->pps_data, hl7132->dev, tcpm_psy, "hl-pps");
	if (ret == 0) {
		pps_set_logbuffer(&hl7132->pps_data, hl7132->log);
		pps_init_state(&hl7132->pps_data);
	}

check_online:
	online = pps_prog_check_online(&hl7132->pps_data, hl7132->pd);
	if (!online)
		return -ENODEV;

	return ret;
}

/* call holding mutex_unlock(&hl7132->lock); */
int hl7132_send_pd_message(struct hl7132_charger *hl7132,
				   unsigned int msg_type)
{
	struct pd_pps_data *pps_data = &hl7132->pps_data;
	struct power_supply *tcpm_psy = hl7132->pd;
	bool online;
	int pps_ui;
	int ret;

	if (!tcpm_psy || (hl7132->charging_state == DC_STATE_NO_CHARGING &&
	    msg_type == PD_MSG_REQUEST_APDO) || !hl7132->mains_online) {
		dev_dbg(hl7132->dev,
			"%s: failure tcpm_psy_ok=%d charging_state=%u online=%d\n",
			__func__,  tcpm_psy != 0, hl7132->charging_state,
			hl7132->mains_online);
		return -EINVAL;
	}

	/* false when offline (0) or not in prog (1) mode */
	online = pps_prog_check_online(&hl7132->pps_data, tcpm_psy);
	if (!online) {
		dev_dbg(hl7132->dev, "%s: not online\n", __func__);
		return -EINVAL;
	}

	/* turn off PPS/PROG, revert to PD */
	if (msg_type == MSG_REQUEST_FIXED_PDO) {
		ret = pps_prog_offline(&hl7132->pps_data, tcpm_psy);
		dev_dbg(hl7132->dev, "%s: requesting offline ret=%d\n",
			__func__, ret);
		/* TODO: reset state? */
		return ret;
	}

	dev_dbg(hl7132->dev,
		"%s: tcpm_psy_ok=%d pd_online=%d pps_stage=%d charging_state=%u\n",
		__func__,  tcpm_psy != 0,  pps_data->pd_online,
		pps_data->stage, hl7132->charging_state);

	if (hl7132->pps_data.stage == PPS_ACTIVE) {

		/* not sure I need to do this */
		ret = hl7132_request_pdo(hl7132);
		if (ret == 0) {
			const int pre_out_uv = pps_data->out_uv;
			const int pre_out_ua = pps_data->op_ua;

			dev_dbg(hl7132->dev,
				"%s: ta_vol=%u, ta_cur=%u, ta_objpos=%u\n",
				__func__, hl7132->ta_vol, hl7132->ta_cur,
				hl7132->ta_objpos);

			pps_ui = pps_update_adapter(&hl7132->pps_data,
						    hl7132->ta_vol,
						    hl7132->ta_cur,
						    tcpm_psy);
			dev_dbg(hl7132->dev,
				"%s: out_uv=%d %d->%d, out_ua=%d %d->%d (%d)\n",
				 __func__,
				 pps_data->out_uv, pre_out_uv, hl7132->ta_vol,
				 pps_data->op_ua, pre_out_ua, hl7132->ta_cur,
				 pps_ui);

			if (pps_ui == 0)
				pps_ui = HL7132_PDMSG_WAIT_T;
			if (pps_ui < 0)
				pps_ui = HL7132_PDMSG_RETRY_T;
		} else {
			dev_dbg(hl7132->dev, "%s: request_pdo failed ret=%d\n",
				 __func__, ret);
			pps_ui = HL7132_PDMSG_RETRY_T;
		}

	} else {
		ret = pps_keep_alive(pps_data, tcpm_psy);
		if (ret == 0)
			pps_ui = PD_T_PPS_TIMEOUT;

		dev_dbg(hl7132->dev, "%s: keep alive ret=%d\n", __func__, ret);
	}

	if (((hl7132->charging_state == DC_STATE_NO_CHARGING) &&
		(msg_type == PD_MSG_REQUEST_APDO)) ||
		(hl7132->mains_online == false)) {

		/*
		 *  Vbus reset might occour even when PD comms is successful.
		 * Check again.
		 */
		pps_ui = -EINVAL;
	}

	/* PPS_Work: will reschedule */
	dev_dbg(hl7132->dev, "%s: pps_ui = %d\n", __func__, pps_ui);
	if (pps_ui > 0)
		mod_delayed_work(system_wq, &hl7132->pps_work,
				 msecs_to_jiffies(pps_ui));

	return pps_ui;
}

/*
 * Get the max current/voltage/power of APDO from the CC/PD driver.
 *
 * Initialize &hl7132->ta_max_vol, &hl7132->ta_max_cur, &hl7132->ta_max_pwr
 * initialize hl7132->pps_data and &hl7132->ta_objpos also
 *
 * call holding mutex_unlock(&hl7132->lock);
 */
int hl7132_get_apdo_max_power(struct hl7132_charger *hl7132,
			       unsigned int ta_max_vol,
			       unsigned int ta_max_cur)
{
	int ret;

	/* limits */
	hl7132->ta_objpos = 0; /* if !=0 will return the ca */
	hl7132->ta_max_vol = ta_max_vol;
	hl7132->ta_max_cur = ta_max_cur;

	/* check the phandle */
	ret = hl7132_usbpd_setup(hl7132);
	if (ret != 0) {
		dev_err(hl7132->dev, "cannot find TCPM %d\n", ret);
		hl7132->pd = NULL;
		return ret;
	}

	/* technically already in pda_data since check online does it */
	ret = pps_get_src_cap(&hl7132->pps_data, hl7132->pd);
	if (ret < 0)
		return ret;

	ret = pps_get_apdo_max_power(&hl7132->pps_data, &hl7132->ta_objpos,
				     &hl7132->ta_max_vol, &hl7132->ta_max_cur,
				     &hl7132->ta_max_pwr);
	if (ret < 0) {
		dev_err(hl7132->dev,
			"cannot determine the apdo max power ret = %d\n", ret);
		return ret;
	}

	dev_dbg(hl7132->dev, "%s: APDO pos=%u max_v=%u max_c=%u max_pwr=%lu\n",
		__func__, hl7132->ta_objpos, hl7132->ta_max_vol,
		hl7132->ta_max_cur, hl7132->ta_max_pwr);

	hl7132_chg_stats_set_apdo(&hl7132->chg_data,
				 hl7132->pps_data.src_caps[hl7132->ta_objpos - 1]);

	return 0;
}

/*
 * Get the first APDO satisfying give ta_vol, ta_cur from the CC/PD driver.
 *
 * call holding mutex_unlock(&hl7132->lock);
 */
int hl7132_get_apdo_index(struct hl7132_charger *hl7132,
			   unsigned int *ta_max_vol,
			   unsigned int *ta_max_cur,
			   unsigned int *ta_objpos)
{
	int ret;
	unsigned long ta_max_pwr;

	/* limits */
	*ta_objpos = 0; /* if !=0 will return the ca */

	/* check the phandle */
	ret = hl7132_usbpd_setup(hl7132);
	if (ret != 0) {
		dev_err(hl7132->dev, "cannot find TCPM %d\n", ret);
		hl7132->pd = NULL;
		return ret;
	}

	/* technically already in pda_data since check online does it */
	ret = pps_get_src_cap(&hl7132->pps_data, hl7132->pd);
	if (ret < 0)
		return ret;

	ret = pps_get_apdo_max_power(&hl7132->pps_data, ta_objpos,
				     ta_max_vol, ta_max_cur,
				     &ta_max_pwr);
	if (ret < 0) {
		dev_err(hl7132->dev,
			"cannot determine the apdo index ret = %d\n", ret);
		return ret;
	}

	dev_dbg(hl7132->dev, "%s: APDO pos=%u max_v=%u max_c=%u max_pwr=%lu\n",
		__func__, *ta_objpos, *ta_max_vol, *ta_max_cur, ta_max_pwr);

	return 0;
}

/* called from start_direct_charging(), negative will abort */
int hl7132_set_ta_type(struct hl7132_charger *hl7132, int pps_index)
{
	if (pps_index == PPS_INDEX_TCPM) {
		int ret;

		ret = hl7132_usbpd_setup(hl7132);
		if (ret != 0) {
			dev_err(hl7132->dev, "Cannot find the TA %d\n", ret);
			return ret;
		}

		hl7132->ta_type = TA_TYPE_USBPD;
		hl7132->chg_mode = CHG_2TO1_DC_MODE;
	} else {
		hl7132->ta_type = TA_TYPE_UNKNOWN;
		hl7132->chg_mode = 0;
		return -EINVAL;
	}

	return 0;
}


/* GBMS integration ------------------------------------------------------ */

int hl7132_get_charge_type(struct hl7132_charger *hl7132)
{
	int ret, sts;

	if (!hl7132->mains_online)
		return POWER_SUPPLY_CHARGE_TYPE_NONE;

	/* Report regulation state */
	ret = regmap_read(hl7132->regmap, HL7132_REG_INT_STS_A, &sts);
	if (ret < 0)
		return ret;

	dev_dbg(hl7132->dev,
		"%s: int_sts_a=%0x2 VBAT_REG=%d IIN_REG=%d charging_state=%d\n",
		__func__, sts, !!(sts & HL7132_BIT_REG_STS_VBAT_REG),
		 !!(sts & HL7132_BIT_REG_STS_IIN_REG),
		 hl7132->charging_state);

	/* Use SW state for now */
	switch (hl7132->charging_state) {
	case DC_STATE_ADJUST_CC:
	case DC_STATE_CC_MODE:
	case DC_STATE_ADJUST_TAVOL:
	case DC_STATE_ADJUST_TACUR:
		return POWER_SUPPLY_CHARGE_TYPE_FAST;
	case DC_STATE_START_CV:
	case DC_STATE_CV_MODE:
		return POWER_SUPPLY_CHARGE_TYPE_TAPER_EXT;
	case DC_STATE_CHECK_ACTIVE: /* in preset */
	case DC_STATE_CHARGING_DONE:
		break;
	}

	return POWER_SUPPLY_CHARGE_TYPE_NONE;
}

int hl7132_get_status(struct hl7132_charger *hl7132)
{
	int ret, sts, chg_state;

	ret = regmap_read(hl7132->regmap, HL7132_REG_INT_STS_A, &sts);
	if (ret < 0)
		return ret;

	chg_state = sts >> 6;

	dev_dbg(hl7132->dev, "%s: int_sts_a=%x, chg_state=%d\n",
		 __func__, sts, chg_state);

	if (chg_state == STATE_CHG_STS_STANDBY) {
		const bool online = hl7132->mains_online;

		/* no disconnect during charger transition */
		return online ? POWER_SUPPLY_STATUS_NOT_CHARGING :
		       POWER_SUPPLY_STATUS_DISCHARGING;
	}

	/* Use SW state (for now) */
	switch (hl7132->charging_state) {
	case DC_STATE_NO_CHARGING:
	case DC_STATE_ERROR:
	case DC_STATE_CHECK_VBAT:
	case DC_STATE_PRESET_DC:
	case DC_STATE_CHECK_ACTIVE:
		return POWER_SUPPLY_STATUS_NOT_CHARGING;
	case DC_STATE_ADJUST_CC:
	case DC_STATE_CC_MODE:
	case DC_STATE_START_CV:
	case DC_STATE_CV_MODE:
		return POWER_SUPPLY_STATUS_CHARGING;
	/* cpm will need to stop it */
	case DC_STATE_CHARGING_DONE:
		return POWER_SUPPLY_STATUS_CHARGING;
	default:
		break;
	}

	return POWER_SUPPLY_STATUS_UNKNOWN;
}

int hl7132_is_present(struct hl7132_charger *hl7132)
{
	int sts = 0;
	int ret = 0;
	int chg_state;

	ret = regmap_read(hl7132->regmap, HL7132_REG_INT_STS_A, &sts);
	if (ret < 0)
		return ret;

	chg_state = sts >> 6;

	if (chg_state == STATE_CHG_STS_ACTIVE || chg_state == STATE_CHG_STS_STANDBY)
		return 1;

	return 0;
}

int hl7132_get_chg_chgr_state(struct hl7132_charger *hl7132,
				      union gbms_charger_state *chg_state)
{
	int vchrg;

	chg_state->v = 0;
	chg_state->f.chg_status = hl7132_get_status(hl7132);
	chg_state->f.chg_type = hl7132_get_charge_type(hl7132);
	chg_state->f.flags = gbms_gen_chg_flags(chg_state->f.chg_status,
						chg_state->f.chg_type);
	chg_state->f.flags |= GBMS_CS_FLAG_DIRECT_CHG;

	vchrg = hl7132_read_adc(hl7132, ADCCH_VBAT);
	if (vchrg > 0)
		chg_state->f.vchrg = vchrg / 1000;

	if (chg_state->f.chg_status != POWER_SUPPLY_STATUS_DISCHARGING) {
		int rc;

		rc = hl7132_input_current_limit(hl7132);
		if (rc > 0)
			chg_state->f.icl = rc / 1000;
	}

	return 0;
}

/* ------------------------------------------------------------------------ */

/* call holding (&hl7132->lock); */
void hl7132_chg_stats_init(struct hl7132_chg_stats *chg_data)
{
	memset(chg_data, 0, sizeof(*chg_data));
	chg_data->adapter_capabilities[0] |= HL7132_CHGS_VER;
}

static void hl7132_chg_stats_set_apdo(struct hl7132_chg_stats *chg_data, u32 apdo)
{
	chg_data->adapter_capabilities[1] = apdo;
}

/* call holding (&hl7132->lock); */
int hl7132_chg_stats_update(struct hl7132_chg_stats *chg_data,
			   const struct hl7132_charger *hl7132)
{
	switch (hl7132->charging_state) {
	case DC_STATE_NO_CHARGING:
		chg_data->nc_count++;
		break;
	case DC_STATE_CHECK_VBAT:
	case DC_STATE_PRESET_DC:
		chg_data->pre_count++;
		break;
	case DC_STATE_CHECK_ACTIVE:
		chg_data->ca_count++;
		break;
	case DC_STATE_ADJUST_CC:
	case DC_STATE_CC_MODE:
		chg_data->cc_count++;
		break;
	case DC_STATE_START_CV:
	case DC_STATE_CV_MODE:
		chg_data->cv_count++;
		break;
	case DC_STATE_ADJUST_TAVOL:
	case DC_STATE_ADJUST_TACUR:
		chg_data->adj_count++;
		break;
	case DC_STATE_CHARGING_DONE:
		chg_data->receiver_state[0] |= HL7132_CHGS_F_DONE;
		break;
	default:
		break;
	}

	return 0;
}

void hl7132_chg_stats_dump(const struct hl7132_charger *hl7132)
{
	const struct hl7132_chg_stats *chg_data = &hl7132->chg_data;

	logbuffer_prlog(hl7132, LOGLEVEL_INFO,
			"N: ovc=%d,ovc_ibatt=%d,ovc_delta=%d rcp=%d,stby=%d,iin_loop=%d",
			chg_data->ovc_count,
			chg_data->ovc_max_ibatt, chg_data->ovc_max_delta,
			chg_data->rcp_count, chg_data->stby_count, chg_data->iin_loop_count);
	logbuffer_prlog(hl7132, LOGLEVEL_INFO,
			"C: nc=%d,pre=%d,ca=%d,cc=%d,cv=%d,adj=%d\n",
			chg_data->nc_count, chg_data->pre_count,
			chg_data->ca_count, chg_data->cc_count,
			chg_data->cv_count, chg_data->adj_count);
}

int hl7132_chg_stats_done(struct hl7132_chg_stats *chg_data,
			 const struct hl7132_charger *hl7132)
{
	/* AC[0] version */
	/* AC[1] is APDO */
	/* RS[0][0:8] flags */
	if (chg_data->stby_count)
		hl7132_chg_stats_update_flags(chg_data, HL7132_CHGS_F_STBY);
	chg_data->receiver_state[0] = (chg_data->pre_count & 0xff) <<
				      HL7132_CHGS_PRE_SHIFT;
	chg_data->receiver_state[0] |= (chg_data->rcp_count & 0xff) <<
				       HL7132_CHGS_RCPC_SHIFT;
	chg_data->receiver_state[0] |= (chg_data->nc_count & 0xff) <<
				       HL7132_CHGS_NC_SHIFT;
	/* RS[1] counters */
	chg_data->receiver_state[1] = (chg_data->ovc_count & 0xffff) <<
				      HL7132_CHGS_OVCC_SHIFT;
	chg_data->receiver_state[1] |= (chg_data->adj_count & 0xffff) <<
				       HL7132_CHGS_ADJ_SHIFT;
	/* RS[2] counters */
	chg_data->receiver_state[2] = (chg_data->adj_count & 0xffff) <<
				      HL7132_CHGS_ADJ_SHIFT;
	chg_data->receiver_state[2] |= (chg_data->adj_count & 0xffff) <<
				       HL7132_CHGS_ADJ_SHIFT;
	/* RS[3] counters */
	chg_data->receiver_state[3] = (chg_data->cc_count & 0xffff) <<
				      HL7132_CHGS_CC_SHIFT;
	chg_data->receiver_state[3] |= (chg_data->cv_count & 0xffff) <<
				       HL7132_CHGS_CV_SHIFT;
	/* RS[4] counters */
	chg_data->receiver_state[1] = (chg_data->ca_count & 0xff) <<
				      HL7132_CHGS_CA_SHIFT;

	chg_data->valid = true;

	return 0;
}
