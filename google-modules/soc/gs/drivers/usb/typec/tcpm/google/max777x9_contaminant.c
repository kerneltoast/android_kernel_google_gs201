// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2023, Google LLC
 *
 * MAX777x9 contaminant detection glue layer
 */

#include "max777x9_contaminant.h"
#include "max77759_contaminant.h"
#include "max77779_contaminant.h"
#include "max77759_helper.h"
#include "tcpci_max77759.h"
#include "tcpci_max77759_vendor_reg.h"

struct max777x9_contaminant *max777x9_contaminant_init(struct max77759_plat *plat, bool enable,
						       bool is_max77779)
{
	struct max777x9_contaminant *contaminant;
	struct device *dev = plat->dev;

	contaminant = devm_kzalloc(dev, sizeof(*contaminant), GFP_KERNEL);
	if (!contaminant)
		return ERR_PTR(-ENOMEM);

	contaminant->chip = plat;
	contaminant->is_max77779 = is_max77779;

	/*
	 * Do not enable in *.ATTACHED state as it would cause an unncessary
	 * disconnect.
	 */
	if (enable) {
		is_max77779 ? max77779_enable_contaminant_detection(plat) :
			      max77759_enable_contaminant_detection(plat);
	}

	return contaminant;
}
EXPORT_SYMBOL_GPL(max777x9_contaminant_init);

static int max777x9_read_comparators(struct max777x9_contaminant *contaminant,
				     u8 *vendor_cc_status2_cc1, u8 *vendor_cc_status2_cc2)
{
	if (!contaminant)
		return -EAGAIN;

	return contaminant->is_max77779 ?
		max77779_read_comparators(contaminant, vendor_cc_status2_cc1,
					  vendor_cc_status2_cc2) :
		max77759_read_comparators(contaminant, vendor_cc_status2_cc1,
					  vendor_cc_status2_cc2);
}

static int max777x9_read_resistance_kohm(struct max777x9_contaminant *contaminant,
					 enum adc_select channel, int sleep_msec,
					 bool raw)
{
	if (!contaminant)
		return -EAGAIN;

	return contaminant->is_max77779 ?
		max77779_read_resistance_kohm(contaminant, channel, sleep_msec, raw) :
		max77759_read_resistance_kohm(contaminant, channel, sleep_msec, raw);
}

int max777x9_enable_contaminant_detection(struct max77759_plat *chip, bool maxq)
{
	struct max777x9_contaminant *contaminant = chip->contaminant;

	if (!contaminant)
		return -EAGAIN;

	contaminant->contaminant_detect_maxq = maxq;

	return contaminant->is_max77779 ?
		max77779_enable_contaminant_detection(chip) :
		max77759_enable_contaminant_detection(chip);
}
EXPORT_SYMBOL_GPL(max777x9_enable_contaminant_detection);

int max777x9_disable_contaminant_detection(struct max77759_plat *chip)
{
	struct max777x9_contaminant *contaminant = chip->contaminant;

	/* TODO: Evaluate to make sense to return EAGAIN */
	if (!contaminant)
		return 0;

	return contaminant->is_max77779 ?
		max77779_disable_contaminant_detection(chip) :
		max77759_disable_contaminant_detection(chip);
}
EXPORT_SYMBOL_GPL(max777x9_disable_contaminant_detection);

static int max777x9_enable_dry_detection(struct max777x9_contaminant *contaminant)
{
	return contaminant->is_max77779 ?
		max77779_enable_dry_detection(contaminant) :
		max77759_enable_dry_detection(contaminant);
}

static int max777x9_detect_contaminant(struct max777x9_contaminant *contaminant)
{
	return contaminant->is_max77779 ?
		max77779_detect_contaminant(contaminant) :
		max77759_detect_contaminant(contaminant);
}

static int max777x9_maxq_detect_contaminant(struct max777x9_contaminant *contaminant, u8 cc_status)
{
	int cc1_raw = 0, cc2_raw = 0, sbu1_raw = 0, sbu2_raw = 0;
	u8 vendor_cc_status2_cc1 = 0, vendor_cc_status2_cc2 = 0, cc1_vufp_rd0p5 = 0;
	u8 cc2_vufp_rd0p5 = 0, maxq_detect_type, role_ctrl = 0, role_ctrl_backup = 0;
	int ret;
	struct max77759_plat *chip = contaminant->chip;
	struct regmap *regmap = contaminant->chip->data.regmap;
	u8 response[5];

	ret = max77759_read8(regmap, TCPC_ROLE_CTRL, &role_ctrl);
	if (ret < 0)
		return -EIO;
	role_ctrl_backup = role_ctrl;
	role_ctrl = 0x0F;
	ret = max77759_write8(regmap, TCPC_ROLE_CTRL, role_ctrl);
	if (ret < 0)
		return -EIO;

	logbuffer_log(chip->log, "Contaminant: Query Maxq");
	if (contaminant->state == NOT_DETECTED) {
		cc1_raw = max777x9_read_resistance_kohm(contaminant, CC1_SCALE2, READ1_SLEEP_MS,
							true);
		cc2_raw = max777x9_read_resistance_kohm(contaminant, CC2_SCALE2, READ2_SLEEP_MS,
							true);
	}

	sbu1_raw = max777x9_read_resistance_kohm(contaminant, SBU1, READ1_SLEEP_MS, true);
	sbu2_raw = max777x9_read_resistance_kohm(contaminant, SBU2, READ2_SLEEP_MS, true);

	if (contaminant->state == NOT_DETECTED) {
		max777x9_read_comparators(contaminant, &vendor_cc_status2_cc1,
					  &vendor_cc_status2_cc2);
		logbuffer_log(chip->log, "Contaminant: Query Maxq vcc2_1:%u vcc2_2:%u",
			      vendor_cc_status2_cc1, vendor_cc_status2_cc2);

		cc1_vufp_rd0p5 = vendor_cc_status2_cc1 & CC1_VUFP_RD0P5 ? 1 : 0;
		cc2_vufp_rd0p5 = vendor_cc_status2_cc2 & CC2_VUFP_RD0P5 ? 1 : 0;
	}
	maxq_detect_type = contaminant->state == NOT_DETECTED ? MAXQ_DETECT_TYPE_CC_AND_SBU :
		MAXQ_DETECT_TYPE_SBU_ONLY;

	ret = maxq_query_contaminant(cc1_raw, cc2_raw, sbu1_raw, sbu2_raw, cc1_vufp_rd0p5,
				     cc2_vufp_rd0p5, maxq_detect_type, 0, response, 5);

	/* Upon errors, falling back to NOT_DETECTED state. */
	if (ret < 0) {
		logbuffer_log(chip->log, "Contaminant: Maxq errors");
		return NOT_DETECTED;
	}

	ret = response[2];
	logbuffer_log(chip->log, "Contaminant: Result opcode:%u present:%u cc_thr:%u, sbu_thr:%u",
		      response[0], response[2], response[3], response[4]);

	if (ret == NOT_DETECTED)
		ret = max77759_write8(regmap, TCPC_ROLE_CTRL, role_ctrl_backup);
	else
		ret = max77759_write8(regmap, TCPC_ROLE_CTRL, (TCPC_ROLE_CTRL_DRP | 0xA));

	if (ret < 0)
		return -EIO;

	return ret;
}

static bool is_cc_open(u8 cc_status)
{
	return status_check(cc_status, TCPC_CC_STATUS_CC1_MASK << TCPC_CC_STATUS_CC1_SHIFT,
			    TCPC_CC_STATE_SRC_OPEN) && status_check(cc_status,
								    TCPC_CC_STATUS_CC2_MASK <<
								    TCPC_CC_STATUS_CC2_SHIFT,
								    TCPC_CC_STATE_SRC_OPEN);
}

static void max777x9_update_contaminant_state(struct max777x9_contaminant *contaminant,
					      enum contamiant_state state)
{
	struct max77759_plat *chip = contaminant->chip;

	if (contaminant->state == state)
		return;

	contaminant->state = state;
	kobject_uevent(&chip->dev->kobj, KOBJ_CHANGE);
}

/*
 * Don't want to be in workqueue as this is time critical for the state machine
 * to forward progress.
 */
int max777x9_process_contaminant_alert(struct max777x9_contaminant *contaminant, bool debounce_path,
				       bool tcpm_toggling, bool *cc_update_handled,
				       bool *port_clean)
{
	u8 cc_status, pwr_cntl;
	struct regmap *regmap = contaminant->chip->data.regmap;
	enum contamiant_state state;
	struct max77759_plat *chip = contaminant->chip;
	int ret;

	/*
	 * Contaminant alert should only be processed when ALERT.CC_STAT is set.
	 * Caller i.e. the top level interrupt handler can check this to
	 * prevent redundant reads.
	 */
	ret = max77759_read8(regmap, TCPC_CC_STATUS, &cc_status);
	if (ret < 0)
		return -EIO;
	logbuffer_log(chip->log, "Contaminant: CC_STATUS: %#x", cc_status);

	ret = max77759_read8(regmap, TCPC_POWER_CTRL, &pwr_cntl);
	if (ret < 0)
		return -EIO;
	logbuffer_log(chip->log, "Contaminant: POWER_CONTROL: %#x", pwr_cntl);

	/* Exit if still LookingForConnection. */
	if (cc_status & TCPC_CC_STATUS_TOGGLING) {
		logbuffer_log(chip->log, "Contaminant: Looking for connection");
		/* Restart toggling before returning in debounce path */
		if (debounce_path && (contaminant->state == NOT_DETECTED ||
				      contaminant->state == SINK)) {
			ret = max777x9_enable_contaminant_detection(contaminant->chip,
								    contaminant->contaminant_detect_maxq);
			if (ret == -EIO)
				return ret;
		}
		if (contaminant->state == DETECTED) {
			*cc_update_handled = true;
			*port_clean = false;
		} else {
			*cc_update_handled = false;
			*port_clean = true;
		}
		return 0;
	}

	if (contaminant->state == NOT_DETECTED || contaminant->state == SINK ||
	    contaminant->state == FLOATING_CABLE) {
		/* ConnectResult = 0b -> Rp */
		if ((status_check(cc_status, TCPC_CC_STATUS_TERM, TCPC_CC_STATUS_TERM_RP)) &&
		    ((status_check(cc_status, TCPC_CC_STATUS_CC1_MASK << TCPC_CC_STATUS_CC1_SHIFT,
				   TCPC_CC_STATE_WTRSEL << TCPC_CC_STATUS_CC1_SHIFT)) ||
		    (status_check(cc_status, TCPC_CC_STATUS_CC2_MASK << TCPC_CC_STATUS_CC2_SHIFT,
				  TCPC_CC_STATE_WTRSEL << TCPC_CC_STATUS_CC2_SHIFT))) &&
		    (status_check(cc_status, TCPC_CC_STATUS_TOGGLING, 0))) {
			logbuffer_log(chip->log, "Contaminant: Check if wet: CC 0x3");
			ret = contaminant->contaminant_detect_maxq ?
				max777x9_maxq_detect_contaminant(contaminant, cc_status)
				: max777x9_detect_contaminant(contaminant);
			if (ret == -EIO)
				return ret;
			state = ret;
			max777x9_update_contaminant_state(contaminant, state);

			if (state == DETECTED) {
				ret = max777x9_enable_dry_detection(contaminant);
				if (ret == -EIO)
					return ret;
				*cc_update_handled = true;
				return 0;
			}

			/* Sink or Not detected */
			ret = max777x9_enable_contaminant_detection(contaminant->chip,
								    contaminant->contaminant_detect_maxq);
			if (ret == -EIO)
				return ret;
			*cc_update_handled = true;
			*port_clean = true;
			return 0;
		} else {
			/* Need to check again after tCCDebounce */
			if (((cc_status & TCPC_CC_STATUS_TOGGLING) == 0)  &&
			    (debounce_path || (tcpm_toggling && is_cc_open(cc_status)))) {
				/*
				 * Stage 3
				 */
				if (!debounce_path) {
					logbuffer_log(chip->log,
						      "Contaminant: Not debounce path sleep 100ms");
					msleep(100);
				}

				ret = max77759_read8(regmap, TCPC_CC_STATUS, &cc_status);
				if (ret < 0)
					return ret;
				logbuffer_log(chip->log,
					      "Contaminant: CC_STATUS check stage 3 sw WAR: %#x",
					      cc_status);
				if (is_cc_open(cc_status)) {
					u8 role_ctrl, role_ctrl_backup;

					ret = max77759_read8(regmap, TCPC_ROLE_CTRL, &role_ctrl);
					if (ret < 0)
						return ret;
					role_ctrl_backup = role_ctrl;
					role_ctrl |= 0x0F;
					role_ctrl &= ~(TCPC_ROLE_CTRL_DRP);
					ret = max77759_write8(regmap, TCPC_ROLE_CTRL, role_ctrl);
					if (ret < 0)
						return ret;

					logbuffer_log(chip->log,
						      "Contaminant: Check if wet (stage 3)");
					ret = contaminant->contaminant_detect_maxq ?
						max777x9_maxq_detect_contaminant(contaminant,
										 cc_status)
						: max777x9_detect_contaminant(contaminant);
					if (ret == -EIO)
						return ret;
					state = ret;
					max777x9_update_contaminant_state(contaminant, state);

					ret = max77759_write8(regmap, TCPC_ROLE_CTRL,
							      role_ctrl_backup);
					if (ret < 0)
						return ret;
					if (state == DETECTED) {
						ret = max777x9_enable_dry_detection(contaminant);
						if (ret == -EIO)
							return ret;
						*cc_update_handled = true;
						return 0;
					}
					/* Sink or Not detected */
					ret = max777x9_enable_contaminant_detection(
							contaminant->chip,
							contaminant->contaminant_detect_maxq);
					if (ret == -EIO)
						return ret;
					*port_clean = true;
				}
			}
		}

		/* Restart toggling before returning in debounce path */
		if (debounce_path) {
			ret = max777x9_enable_contaminant_detection(contaminant->chip,
								    contaminant->contaminant_detect_maxq);
			if (ret == -EIO)
				return ret;
			*port_clean = true;
		}
		*cc_update_handled = false;
		return 0;
	} else if (contaminant->state == DETECTED) {
		if (status_check(cc_status, TCPC_CC_STATUS_TOGGLING, 0)) {
			logbuffer_log(chip->log, "Contaminant: Check if dry");
			state = contaminant->contaminant_detect_maxq ?
				max777x9_maxq_detect_contaminant(contaminant, cc_status)
				: max777x9_detect_contaminant(contaminant);
			max777x9_update_contaminant_state(contaminant, state);

			if (state == DETECTED) {
				ret = max777x9_enable_dry_detection(contaminant);
				if (ret == -EIO)
					return ret;
				*cc_update_handled = true;
				return 0;
			}

			/*
			 * Re-enable contaminant detection, hence toggling and
			 * auto_ultra_low_power_mode as well.
			 */
			max777x9_disable_auto_ultra_low_power_mode(chip, false);
			ret = max777x9_enable_contaminant_detection(contaminant->chip,
							contaminant->contaminant_detect_maxq);
			if (ret == -EIO)
				return ret;
			*cc_update_handled = true;
			*port_clean = true;
			return 0;
		}
		/* TCPM does not manage ports in dry detection phase. */
		*cc_update_handled = true;
		return 0;
	}

	*cc_update_handled = false;
	return 0;
}
EXPORT_SYMBOL_GPL(max777x9_process_contaminant_alert);

bool max777x9_is_contaminant_detected(struct max77759_plat *chip)
{
	if (chip)
		return chip->contaminant->state == DETECTED;

	return false;
}
EXPORT_SYMBOL_GPL(max777x9_is_contaminant_detected);

bool max777x9_is_floating_cable_or_sink_detected(struct max77759_plat *chip)
{
	if (chip)
		return chip->contaminant->state == FLOATING_CABLE ||
			chip->contaminant->state == SINK;

	return false;
}
EXPORT_SYMBOL_GPL(max777x9_is_floating_cable_or_sink_detected);

void max777x9_disable_auto_ultra_low_power_mode(struct max77759_plat *chip, bool disable)
{
	if (!chip || !chip->contaminant)
		return;

	chip->contaminant->is_max77779 ?
		max77779_disable_auto_ultra_low_power_mode(chip, disable) :
		max77759_disable_auto_ultra_low_power_mode(chip, disable);
}
EXPORT_SYMBOL_GPL(max777x9_disable_auto_ultra_low_power_mode);
MODULE_AUTHOR("Badhri Jagan Sridharan <badhri@google.com>");
MODULE_DESCRIPTION("MAX777x9 contaminant detection glue layer");
MODULE_LICENSE("GPL");
