// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright 2020 Google LLC
 *
 * USB contaminant detection
 */

#include <linux/device.h>
#include <linux/irqreturn.h>
#include <linux/module.h>
#include <linux/usb/tcpm.h>
#include <linux/usb/typec.h>
#include <misc/logbuffer.h>

#include "max77759_helper.h"
#include "tcpci.h"
#include "tcpci_max77759.h"
#include "tcpci_max77759_vendor_reg.h"

enum contamiant_state {
	NOT_DETECTED,
	DETECTED,
	FLOATING_CABLE,
	SINK,
	DISABLED,
};

/* To be kept in sync with TCPC_VENDOR_ADC_CTRL1.ADCINSEL */
enum fladc_select {
	CC1_SCALE1 = 1,
	CC1_SCALE2,
	CC2_SCALE1,
	CC2_SCALE2,
	SBU1,
	SBU2,
};

/* Updated in MDR2 slide. */
#define FLADC_1uA_LSB_MV		25
/* High range CC */
#define FLADC_CC_HIGH_RANGE_LSB_MV	208
/* Low range CC */
#define FLADC_CC_LOW_RANGE_LSB_MV      126

/* 1uA current source */
#define FLADC_CC_SCALE1			1
/* 5 uA current source */
#define FLADC_CC_SCALE2			5

#define FLADC_1uA_CC_OFFSET_MV		300
#define FLADC_CC_HIGH_RANGE_OFFSET_MV	624
#define FLADC_CC_LOW_RANGE_OFFSET_MV	378

/* Actually translates to 18.7K */
#define ACCESSORY_THRESHOLD_CC_K	25
#define CONTAMINANT_THRESHOLD_SBU_K	1000
#define	CONTAMINANT_THRESHOLD_CC_K	1000

#define READ1_SLEEP_MS			10
#define READ2_SLEEP_MS			5

static bool contaminant_detect_maxq;

struct max77759_contaminant {
	struct max77759_plat *chip;
	enum contamiant_state state;
};

static int adc_to_mv(struct max77759_contaminant *contaminant, enum fladc_select channel,
		     bool ua_src, u8 fladc)
{
	/* SBU channels only have 1 scale with 1uA. */
	if ((ua_src && (channel == CC1_SCALE2 || channel == CC2_SCALE2 || channel == SBU1 ||
			channel == SBU2)))
		/* Mean of range */
		return FLADC_1uA_CC_OFFSET_MV + (fladc * FLADC_1uA_LSB_MV);
	else if (!ua_src && (channel == CC1_SCALE1 || channel == CC2_SCALE1))
		return FLADC_CC_HIGH_RANGE_OFFSET_MV + (fladc * FLADC_CC_HIGH_RANGE_LSB_MV);
	else if (!ua_src && (channel == CC1_SCALE2 || channel == CC2_SCALE2))
		return FLADC_CC_LOW_RANGE_OFFSET_MV + (fladc * FLADC_CC_LOW_RANGE_LSB_MV);
	else
		logbuffer_log(contaminant->chip->log, "ADC ERROR: SCALE UNKNOWN");

	return fladc;
}

static inline bool status_check(u8 reg, u8 mask, u8 val)
{
	return ((reg) & (mask)) == (val);
}

static int read_adc_mv(struct max77759_contaminant *contaminant,
		       enum fladc_select channel, int sleep_msec, bool raw,
		       bool ua_src)
{
	struct regmap *regmap = contaminant->chip->data.regmap;
	u8 fladc;
	struct logbuffer *log = contaminant->chip->log;
	int ret;

	/* Channel & scale select */
	ret = max77759_update_bits8(regmap, TCPC_VENDOR_ADC_CTRL1, ADCINSEL_MASK,
				    channel << ADC_CHANNEL_OFFSET);
	if (ret < 0)
		return ret;

	/* Enable ADC */
	ret = max77759_update_bits8(regmap, TCPC_VENDOR_ADC_CTRL1, ADCEN, ADCEN);
	if (ret < 0)
		return ret;

	MAX77759_LOG_REGISTER(regmap, TCPC_VENDOR_ADC_CTRL1, log);

	usleep_range(sleep_msec * 1000, (sleep_msec + 1) * 1000);
	ret = max77759_read8(regmap, TCPC_VENDOR_FLADC_STATUS, &fladc);
	if (ret < 0)
		return ret;
	logbuffer_log(log, "Contaminant: ADC %u", fladc);

	/* Disable ADC */
	ret = max77759_update_bits8(regmap, TCPC_VENDOR_ADC_CTRL1, ADCEN, 0);
	if (ret < 0)
		return ret;

	ret = max77759_update_bits8(regmap, TCPC_VENDOR_ADC_CTRL1, ADCINSEL_MASK, 0);
	if (ret < 0)
		return ret;

	if (!raw)
		return adc_to_mv(contaminant, channel, ua_src, fladc);
	else
		return fladc;
}

static int read_resistance_kohm(struct max77759_contaminant *contaminant,
				enum fladc_select channel, int sleep_msec,
				bool raw)
{
	struct regmap *regmap = contaminant->chip->data.regmap;
	struct logbuffer *log = contaminant->chip->log;
	int mv;
	u8 switch_setting;
	int ret;

	if (channel == CC1_SCALE1 || channel == CC2_SCALE1 || channel == CC1_SCALE2 ||
	    channel == CC2_SCALE2) {
		/* Enable 1uA current source */
		ret = max77759_update_bits8(regmap, TCPC_VENDOR_CC_CTRL2, CCLPMODESEL_MASK,
					    ULTRA_LOW_POWER_MODE);
		if (ret < 0)
			return ret;
		/*
		 * CC resistive ladder is automatically disabled when
		 * 1uA source is ON and Flash ADC channel is not CC scale1.
		 * 1uA soruce is default on here.
		 *
		 * REMOVED IN MDR2.0 V2.0
		 */

		/* Enable 1uA current source */
		ret = max77759_update_bits8(regmap, TCPC_VENDOR_CC_CTRL2, CCRPCTRL_MASK, UA_1_SRC);
		if (ret < 0)
			return ret;

		/* OVP disable */
		ret = max77759_update_bits8(regmap, TCPC_VENDOR_CC_CTRL2, CCOVPDIS, CCOVPDIS);
		if (ret < 0)
			return ret;

		MAX77759_LOG_REGISTER(regmap, TCPC_VENDOR_CC_CTRL2, log);

		mv = read_adc_mv(contaminant, channel, sleep_msec, raw, true);
		/* OVP enable */
		ret = max77759_update_bits8(regmap, TCPC_VENDOR_CC_CTRL2, CCOVPDIS, 0);
		if (ret < 0)
			return ret;
		/* returns KOhm as 1uA source is used. */
		return mv;
	}

	logbuffer_log(log, "Contaminant: SBU read");
	/*
	 * SBU measurement
	 * OVP disable
	 */
	ret = max77759_update_bits8(regmap, TCPC_VENDOR_CC_CTRL2, CCLPMODESEL_MASK,
				    ULTRA_LOW_POWER_MODE);
	if (ret < 0)
		return ret;

	ret = max77759_update_bits8(regmap, TCPC_VENDOR_CC_CTRL2, SBUOVPDIS, SBUOVPDIS);
	if (ret < 0)
		return ret;

	/* Cache switch setting */
	ret = max77759_read8(regmap, TCPC_VENDOR_SBUSW_CTRL, &switch_setting);
	if (ret < 0)
		return ret;
	MAX77759_LOG_REGISTER(regmap, TCPC_VENDOR_SBUSW_CTRL, log);

	/* SBU switches auto configure when channel is selected. */
	/* Enable 1ua current source */
	ret = max77759_update_bits8(regmap, TCPC_VENDOR_CC_CTRL2, SBURPCTRL, SBURPCTRL);
	if (ret < 0)
		return ret;
	MAX77759_LOG_REGISTER(regmap, TCPC_VENDOR_CC_CTRL2, log);

	mv = read_adc_mv(contaminant, channel, sleep_msec, raw, true);
	/* Disable current source */
	ret = max77759_update_bits8(regmap, TCPC_VENDOR_CC_CTRL2, SBURPCTRL, 0);
	if (ret < 0)
		return ret;
	/* Set switch to original setting */
	ret = max77759_write8(regmap, TCPC_VENDOR_SBUSW_CTRL, switch_setting);
	if (ret < 0)
		return ret;

	/* OVP disable */
	ret = max77759_update_bits8(regmap, TCPC_VENDOR_CC_CTRL2, SBUOVPDIS, 0);
	if (ret < 0)
		return ret;

	/*
	 * 1ua current source on sbu;
	 * return KOhm
	 */
	logbuffer_log(contaminant->chip->log, "Contaminant: SBU read %#x", mv);
	return mv;
}

static void read_comparators(struct max77759_contaminant *contaminant,
			     u8 *vendor_cc_status2_cc1,
			     u8 *vendor_cc_status2_cc2)
{
	struct regmap *regmap = contaminant->chip->data.regmap;
	struct logbuffer *log = contaminant->chip->log;
	int ret;

	logbuffer_log(log, "Contaminant: enable comparators");

	/* Enable 80uA source */
	ret = max77759_update_bits8(regmap, TCPC_VENDOR_CC_CTRL2, CCRPCTRL_MASK, UA_80_SRC);
	if (ret < 0)
		return;

	/* Enable comparators */
	ret = max77759_update_bits8(regmap, TCPC_VENDOR_CC_CTRL1, CCCOMPEN, CCCOMPEN);
	if (ret < 0)
		return;

	MAX77759_LOG_REGISTER(regmap, TCPC_VENDOR_CC_CTRL1, log);

	/* Disable low power mode */
	ret = max77759_update_bits8(regmap, TCPC_VENDOR_CC_CTRL2, CCLPMODESEL_MASK,
				    LOW_POWER_MODE_DISABLE);
	if (ret < 0)
		return;
	MAX77759_LOG_REGISTER(regmap, TCPC_VENDOR_CC_CTRL2, log);

	/* Sleep to allow comparators settle */
	usleep_range(5000, 6000);
	ret = max77759_update_bits8(regmap, TCPC_TCPC_CTRL, TCPC_TCPC_CTRL_ORIENTATION,
				    PLUG_ORNT_CC1);
	if (ret < 0)
		return;
	MAX77759_LOG_REGISTER(regmap, TCPC_TCPC_CTRL, log);

	usleep_range(5000, 6000);
	ret = max77759_read8(regmap, VENDOR_CC_STATUS2, vendor_cc_status2_cc1);
	if (ret < 0)
		return;
	logbuffer_log(log, "Contaminant: VENDOR_CC_STATUS2: %u", *vendor_cc_status2_cc1);

	ret = max77759_update_bits8(regmap, TCPC_TCPC_CTRL, TCPC_TCPC_CTRL_ORIENTATION,
				    PLUG_ORNT_CC2);
	if (ret < 0)
		return;
	MAX77759_LOG_REGISTER(regmap, TCPC_TCPC_CTRL, log);

	usleep_range(5000, 6000);
	ret = max77759_read8(regmap, VENDOR_CC_STATUS2, vendor_cc_status2_cc2);
	if (ret < 0)
		return;
	logbuffer_log(contaminant->chip->log, "Contaminant: VENDOR_CC_STATUS2: %u",
		      *vendor_cc_status2_cc2);

	ret = max77759_update_bits8(regmap, TCPC_VENDOR_CC_CTRL1, CCCOMPEN, 0);
	if (ret < 0)
		return;
	max77759_update_bits8(regmap, TCPC_VENDOR_CC_CTRL2, CCRPCTRL_MASK, 0);
}

static int detect_contaminant(struct max77759_contaminant *contaminant)
{
	int cc1_k, cc2_k, sbu1_k, sbu2_k;
	u8 vendor_cc_status2_cc1 = 0xff, vendor_cc_status2_cc2 = 0xff;
	u8 role_ctrl = 0, role_ctrl_backup = 0;
	struct max77759_plat *chip = contaminant->chip;
	int inferred_state = NOT_DETECTED;
	struct regmap *regmap = contaminant->chip->data.regmap;

	max77759_read8(regmap, TCPC_ROLE_CTRL, &role_ctrl);
	role_ctrl_backup = role_ctrl;
	role_ctrl = 0x0F;
	max77759_write8(regmap, TCPC_ROLE_CTRL, role_ctrl);

	/* CCLPMODESEL_AUTO_LOW_POWER in use. */
	cc1_k = read_resistance_kohm(contaminant, CC1_SCALE2, READ1_SLEEP_MS, false);
	cc2_k = read_resistance_kohm(contaminant, CC2_SCALE2, READ2_SLEEP_MS, false);
	logbuffer_log(chip->log, "Contaminant: cc1_k:%u cc2_k:%u", cc1_k, cc2_k);

	sbu1_k = read_resistance_kohm(contaminant, SBU1, READ1_SLEEP_MS, false);
	sbu2_k = read_resistance_kohm(contaminant, SBU2, READ2_SLEEP_MS, false);
	logbuffer_log(chip->log, "Contaminant: sbu1_k:%u sbu2_k:%u", sbu1_k, sbu2_k);
	read_comparators(contaminant, &vendor_cc_status2_cc1, &vendor_cc_status2_cc2);
	logbuffer_log(chip->log, "Contaminant: vcc2_cc1:%u vcc2_cc2:%u", vendor_cc_status2_cc1,
		      vendor_cc_status2_cc2);

	if ((!(CC1_VUFP_RD0P5 & vendor_cc_status2_cc1) ||
	     !(CC2_VUFP_RD0P5 & vendor_cc_status2_cc2)) &&
	    !(CC1_VUFP_RD0P5 & vendor_cc_status2_cc1 && CC2_VUFP_RD0P5 & vendor_cc_status2_cc2)) {
		logbuffer_log(chip->log, "Contaminant: AP SINK detected");
		inferred_state = SINK;
	} else if (cc1_k < CONTAMINANT_THRESHOLD_CC_K || cc2_k < CONTAMINANT_THRESHOLD_CC_K) {
		if (sbu1_k < CONTAMINANT_THRESHOLD_SBU_K || sbu2_k < CONTAMINANT_THRESHOLD_SBU_K) {
			logbuffer_log(chip->log, "Contaminant: AP contaminant detected");
			inferred_state = DETECTED;
		} else {
			logbuffer_log(chip->log, "Contaminant: AP floating cable detected");
			/*
			 * Consider floating cable as sink as well to allow
			 * TotalPhase analyzer to work as it presents ~600k in
			 * one of the CC pins.
			 */
			inferred_state = SINK;
		}
	}

	if (inferred_state == NOT_DETECTED)
		max77759_write8(regmap, TCPC_ROLE_CTRL, role_ctrl_backup);
	else
		max77759_write8(regmap, TCPC_ROLE_CTRL, (TCPC_ROLE_CTRL_DRP | 0xA));

	return inferred_state;
}

static int enable_dry_detection(struct max77759_contaminant *contaminant)
{
	struct regmap *regmap = contaminant->chip->data.regmap;
	struct max77759_plat *chip = contaminant->chip;
	u8 temp;
	int ret;

	/*
	 * tunable: 1ms water detection debounce
	 * tunable: 1000mV/1000K thershold for water detection
	 * tunable: 4.8s water cycle
	 */
	ret = max77759_update_bits8(regmap, TCPC_VENDOR_CC_CTRL3, CCWTRDEB_MASK | CCWTRSEL_MASK
				    | WTRCYCLE_MASK, CCWTRDEB_1MS << CCWTRDEB_SHIFT |
				    CCWTRSEL_1V << CCWTRSEL_SHIFT | WTRCYCLE_4_8_S <<
				    WTRCYCLE_SHIFT);
	if (ret < 0)
		return ret;

	ret = max77759_update_bits8(regmap, TCPC_ROLE_CTRL, TCPC_ROLE_CTRL_DRP,
				    TCPC_ROLE_CTRL_DRP);
	if (ret < 0)
		return ret;

	/* tunable: 1ua / Ultra low power mode enabled. */
	ret = max77759_update_bits8(regmap, TCPC_VENDOR_CC_CTRL1, CCCONNDRY, CCCONNDRY);
	if (ret < 0)
		return ret;
	ret = max77759_read8(regmap, TCPC_VENDOR_CC_CTRL1, &temp);
	if (ret < 0)
		return ret;
	logbuffer_log(chip->log, "Contaminant: TCPC_VENDOR_CC_CTRL1 %u", temp);

	ret = max77759_update_bits8(regmap, TCPC_VENDOR_CC_CTRL2, CCLPMODESEL_MASK,
				    ULTRA_LOW_POWER_MODE);
	if (ret < 0)
		return ret;
	ret = max77759_read8(regmap, TCPC_VENDOR_CC_CTRL2, &temp);
	if (ret < 0)
		return ret;
	logbuffer_log(chip->log, "Contaminant: TCPC_VENDOR_CC_CTRL2 %u", temp);

	/* Enable Look4Connection before sending the command */
	ret = max77759_update_bits8(regmap, TCPC_TCPC_CTRL, TCPC_TCPC_CTRL_EN_LK4CONN_ALRT,
				    TCPC_TCPC_CTRL_EN_LK4CONN_ALRT);
	if (ret < 0)
		return ret;

	ret = max77759_write8(regmap, TCPC_COMMAND, TCPC_CMD_LOOK4CONNECTION);
	if (ret < 0)
		return ret;
	logbuffer_log(chip->log, "Contaminant: Dry detecion enabled");
	return 0;
}

static int maxq_detect_contaminant(struct max77759_contaminant *contaminant, u8 cc_status)
{
	int cc1_raw = 0, cc2_raw = 0, sbu1_raw = 0, sbu2_raw = 0;
	u8 vendor_cc_status2_cc1 = 0, vendor_cc_status2_cc2 = 0, cc1_vufp_rd0p5 = 0;
	u8 cc2_vufp_rd0p5 = 0, maxq_detect_type, role_ctrl = 0, role_ctrl_backup = 0;
	int ret;
	struct max77759_plat *chip = contaminant->chip;
	struct regmap *regmap = contaminant->chip->data.regmap;
	u8 response[5];

	max77759_read8(regmap, TCPC_ROLE_CTRL, &role_ctrl);
	role_ctrl_backup = role_ctrl;
	role_ctrl = 0x0F;
	max77759_write8(regmap, TCPC_ROLE_CTRL, role_ctrl);

	logbuffer_log(chip->log, "Contaminant: Query Maxq");
	if (contaminant->state == NOT_DETECTED) {
		cc1_raw = read_resistance_kohm(contaminant, CC1_SCALE2, READ1_SLEEP_MS, true);
		cc2_raw = read_resistance_kohm(contaminant, CC2_SCALE2, READ2_SLEEP_MS, true);
	}

	sbu1_raw = read_resistance_kohm(contaminant, SBU1, READ1_SLEEP_MS, true);
	sbu2_raw = read_resistance_kohm(contaminant, SBU2, READ2_SLEEP_MS, true);

	if (contaminant->state == NOT_DETECTED) {
		read_comparators(contaminant, &vendor_cc_status2_cc1, &vendor_cc_status2_cc2);
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
		max77759_write8(regmap, TCPC_ROLE_CTRL, role_ctrl_backup);
	else
		max77759_write8(regmap, TCPC_ROLE_CTRL, (TCPC_ROLE_CTRL_DRP | 0xA));

	return ret;
}

static bool is_cc_open(cc_status)
{
	return status_check(cc_status, TCPC_CC_STATUS_CC1_MASK << TCPC_CC_STATUS_CC1_SHIFT,
			    TCPC_CC_STATE_SRC_OPEN) && status_check(cc_status,
								    TCPC_CC_STATUS_CC2_MASK <<
								    TCPC_CC_STATUS_CC2_SHIFT,
								    TCPC_CC_STATE_SRC_OPEN);
}

static void update_contaminant_state(struct max77759_contaminant *contaminant,
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
bool process_contaminant_alert(struct max77759_contaminant *contaminant, bool debounce_path,
			       bool tcpm_toggling)
{
	u8 cc_status, pwr_cntl;
	struct regmap *regmap = contaminant->chip->data.regmap;
	enum contamiant_state state;
	struct max77759_plat *chip = contaminant->chip;

	/*
	 * Contaminant alert should only be processed when ALERT.CC_STAT is set.
	 * Caller i.e. the top level interrupt handler can check this to
	 * prevent redundant reads.
	 */
	max77759_read8(regmap, TCPC_CC_STATUS, &cc_status);
	logbuffer_log(chip->log, "Contaminant: CC_STATUS: %#x", cc_status);

	max77759_read8(regmap, TCPC_POWER_CTRL, &pwr_cntl);
	logbuffer_log(chip->log, "Contaminant: POWER_CONTROL: %#x", pwr_cntl);

	/* Exit if still LookingForConnection. */
	if (cc_status & TCPC_CC_STATUS_TOGGLING) {
		logbuffer_log(chip->log, "Contaminant: Looking for connection");
		/* Restart toggling before returning in debounce path */
		if (debounce_path && (contaminant->state == NOT_DETECTED ||
				      contaminant->state == SINK))
			enable_contaminant_detection(contaminant->chip, contaminant_detect_maxq);
		return false;
	}

	if (contaminant->state == NOT_DETECTED || contaminant->state == SINK) {
		/* ConnectResult = 0b -> Rp */
		if ((status_check(cc_status, TCPC_CC_STATUS_TERM, TCPC_CC_STATUS_TERM_RP)) &&
		    ((status_check(cc_status, TCPC_CC_STATUS_CC1_MASK << TCPC_CC_STATUS_CC1_SHIFT,
				   TCPC_CC_STATE_WTRSEL << TCPC_CC_STATUS_CC1_SHIFT)) ||
		    (status_check(cc_status, TCPC_CC_STATUS_CC2_MASK << TCPC_CC_STATUS_CC2_SHIFT,
				  TCPC_CC_STATE_WTRSEL << TCPC_CC_STATUS_CC2_SHIFT))) &&
		    (status_check(cc_status, TCPC_CC_STATUS_TOGGLING, 0))) {
			logbuffer_log(chip->log, "Contaminant: Check if wet: CC 0x3");
			state = contaminant_detect_maxq ?
				maxq_detect_contaminant(contaminant, cc_status)
				: detect_contaminant(contaminant);
			update_contaminant_state(contaminant, state);

			if (state == DETECTED || state == FLOATING_CABLE) {
				enable_dry_detection(contaminant);
				return true;
			}

			/* Sink or Not detected */
			enable_contaminant_detection(contaminant->chip, contaminant_detect_maxq);
			return true;
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

				max77759_read8(regmap, TCPC_CC_STATUS, &cc_status);
				logbuffer_log(chip->log,
					      "Contaminant: CC_STATUS check stage 3 sw WAR: %#x",
					      cc_status);
				if (is_cc_open(cc_status)) {
					u8 role_ctrl, role_ctrl_backup;

					max77759_read8(regmap, TCPC_ROLE_CTRL, &role_ctrl);
					role_ctrl_backup = role_ctrl;
					role_ctrl |= 0x0F;
					role_ctrl &= ~(TCPC_ROLE_CTRL_DRP);
					max77759_write8(regmap, TCPC_ROLE_CTRL, role_ctrl);

					logbuffer_log(chip->log,
						      "Contaminant: Check if wet (stage 3)");
					state = contaminant_detect_maxq ?
						maxq_detect_contaminant(contaminant, cc_status)
						: detect_contaminant(contaminant);
					update_contaminant_state(contaminant, state);

					max77759_write8(regmap, TCPC_ROLE_CTRL, role_ctrl_backup);
					if (state == DETECTED || state == FLOATING_CABLE) {
						enable_dry_detection(contaminant);
						return true;
					}
					/* Sink or Not detected */
					enable_contaminant_detection(contaminant->chip,
								     contaminant_detect_maxq);
				}
			}
		}

		/* Restart toggling before returning in debounce path */
		if (debounce_path)
			enable_contaminant_detection(contaminant->chip, contaminant_detect_maxq);
		return false;
	} else if (contaminant->state == DETECTED || contaminant->state ==
		   FLOATING_CABLE) {
		if (status_check(cc_status, TCPC_CC_STATUS_TOGGLING, 0)) {
			logbuffer_log(chip->log, "Contaminant: Check if dry");
			state = contaminant_detect_maxq ?
				maxq_detect_contaminant(contaminant, cc_status)
				: detect_contaminant(contaminant);
			update_contaminant_state(contaminant, state);

			if (state == DETECTED || state == FLOATING_CABLE) {
				enable_dry_detection(contaminant);
				return true;
			}

			/* Re-enable contaminant detection, hence toggling as well. */
			enable_contaminant_detection(contaminant->chip, contaminant_detect_maxq);
			return true;
		}
		/* TCPM does not manage ports in dry detection phase. */
		return true;
	}

	return false;
}
EXPORT_SYMBOL_GPL(process_contaminant_alert);

void disable_contaminant_detection(struct max77759_plat *chip)
{
	struct regmap *regmap = chip->data.regmap;
	struct max77759_contaminant *contaminant = chip->contaminant;
	int ret;

	if (!contaminant)
		return;

	ret = max77759_update_bits8(regmap, TCPC_VENDOR_CC_CTRL2, CCLPMODESEL_MASK, 0);
	if (ret < 0)
		return;

	ret = max77759_write8(regmap, TCPC_ROLE_CTRL, TCPC_ROLE_CTRL_DRP |
			      (TCPC_ROLE_CTRL_CC_RD << TCPC_ROLE_CTRL_CC1_SHIFT) |
			      (TCPC_ROLE_CTRL_CC_RD << TCPC_ROLE_CTRL_CC2_SHIFT));
	if (ret < 0)
		return;

	ret = max77759_update_bits8(regmap, TCPC_VENDOR_CC_CTRL2, CCLPMODESEL_MASK,
				    LOW_POWER_MODE_DISABLE);
	if (ret < 0)
		return;

	ret = max77759_update_bits8(regmap, TCPC_TCPC_CTRL, TCPC_TCPC_CTRL_EN_LK4CONN_ALRT,
				    TCPC_TCPC_CTRL_EN_LK4CONN_ALRT);
	if (ret < 0)
		return;

	ret = max77759_update_bits8(regmap, TCPC_VENDOR_CC_CTRL1, CCCONNDRY, 0);
	if (ret < 0)
		return;

	ret = max77759_write8(regmap, TCPC_COMMAND, TCPC_CMD_LOOK4CONNECTION);
	if (ret < 0)
		return;

	/* Reset state before disabling detection */
	if (contaminant->state != NOT_DETECTED && contaminant->state != SINK)
		contaminant->state = NOT_DETECTED;

	logbuffer_log(chip->log, "Contaminant: Contaminant detection disabled");
}
EXPORT_SYMBOL_GPL(disable_contaminant_detection);

int enable_contaminant_detection(struct max77759_plat *chip, bool maxq)
{
	struct regmap *regmap = chip->data.regmap;
	struct max77759_contaminant *contaminant = chip->contaminant;
	u8 vcc2, pwr_ctrl;
	int ret;

	if (!contaminant)
		return -EAGAIN;

	contaminant_detect_maxq = maxq;
	/*
	 * tunable: 1ms water detection debounce
	 * tunable: 1000mV/1000K thershold for water detection
	 * tunable: 4.8s water cycle
	 */
	ret = max77759_update_bits8(regmap, TCPC_VENDOR_CC_CTRL3, CCWTRDEB_MASK | CCWTRSEL_MASK
				    | WTRCYCLE_MASK, CCWTRDEB_1MS << CCWTRDEB_SHIFT |
				    CCWTRSEL_1V << CCWTRSEL_SHIFT | WTRCYCLE_4_8_S <<
				    WTRCYCLE_SHIFT);
	if (ret < 0)
		return ret;

	/* Contaminant detection mode: contaminant detection */
	ret = max77759_update_bits8(regmap, TCPC_VENDOR_CC_CTRL1, CCCONNDRY, 0);
	if (ret < 0)
		return ret;

	ret = max77759_read8(regmap, TCPC_VENDOR_CC_CTRL2, &vcc2);
	if (ret < 0)
		return ret;

	/* tunable: Periodic contaminant detection */
	ret = max77759_update_bits8(regmap, TCPC_VENDOR_CC_CTRL2, CCLPMODESEL_MASK,
				    AUTO_ULTRA_LOW_POWER_MODE);
	if (ret < 0)
		return ret;

	ret = max77759_read8(regmap, TCPC_VENDOR_CC_CTRL2, &vcc2);
	if (ret < 0)
		return ret;

	/* Mask flash adc interrupt */
	ret = max77759_update_bits8(regmap, TCPC_VENDOR_ALERT_MASK2, MSK_FLASH_ADCINT, 0);
	if (ret < 0)
		return ret;

	/* Disable Auto disacharge before enabling toggling */
	ret = max77759_read8(regmap, TCPC_POWER_CTRL, &pwr_ctrl);
	logbuffer_log(chip->log, "TCPC_POWER_CTRL:0x%x ret:%d", pwr_ctrl, ret);
	if (pwr_ctrl & TCPC_POWER_CTRL_AUTO_DISCHARGE) {
		logbuffer_log(chip->log, "TCPC_POWER_CTRL_AUTO_DISCHARGE not cleared");
		ret = regmap_update_bits(regmap, TCPC_POWER_CTRL, TCPC_POWER_CTRL_AUTO_DISCHARGE,
					 0);
		if (ret < 0)
			logbuffer_log(chip->log, "[%s]: Disabling auto discharge failed", __func__);
	}

	ret = max77759_write8(regmap, TCPC_ROLE_CTRL, TCPC_ROLE_CTRL_DRP |
			      (TCPC_ROLE_CTRL_CC_RD <<
			       TCPC_ROLE_CTRL_CC1_SHIFT) |
			      (TCPC_ROLE_CTRL_CC_RD <<
			       TCPC_ROLE_CTRL_CC2_SHIFT));
	if (ret < 0) {
		logbuffer_log(chip->log, "[%s]: Enabling DRP failed ret:%d", __func__,
			      ret);
		return ret;
	}

	/* Enable Look4Connection before sending the command */
	ret = max77759_update_bits8(regmap, TCPC_TCPC_CTRL, TCPC_TCPC_CTRL_EN_LK4CONN_ALRT,
				    TCPC_TCPC_CTRL_EN_LK4CONN_ALRT);
	if (ret < 0) {
		logbuffer_log(chip->log, "[%s]: Enabling looking for connection failed ret:%d",
			      __func__, ret);
		return ret;
	}

	ret = max77759_write8(regmap, TCPC_COMMAND, TCPC_CMD_LOOK4CONNECTION);
	if (ret < 0)
		return ret;

	/* Reset state before enabling detection */
	if (contaminant->state != NOT_DETECTED && contaminant->state != SINK)
		contaminant->state = NOT_DETECTED;

	logbuffer_log(chip->log, "Contaminant: Contaminant detection enabled");

	return 0;
}
EXPORT_SYMBOL_GPL(enable_contaminant_detection);

bool is_contaminant_detected(struct max77759_plat *chip)
{
	if (chip)
		return chip->contaminant->state == DETECTED;

	return false;
}
EXPORT_SYMBOL_GPL(is_contaminant_detected);

struct max77759_contaminant *max77759_contaminant_init(struct max77759_plat
							 *plat, bool enable)
{
	struct max77759_contaminant *contaminant;
	struct device *dev = plat->dev;

	contaminant = devm_kzalloc(dev, sizeof(*contaminant), GFP_KERNEL);
	if (!contaminant)
		return ERR_PTR(-ENOMEM);

	contaminant->chip = plat;

	/*
	 * Do not enable in *.ATTACHED state as it would cause an unncessary
	 * disconnect.
	 */
	if (enable)
		enable_contaminant_detection(plat, contaminant_detect_maxq);

	return contaminant;
}
EXPORT_SYMBOL_GPL(max77759_contaminant_init);

MODULE_DESCRIPTION("MAX77759_CONTAMINANT Module");
MODULE_AUTHOR("Badhri Jagan Sridharan <badhri@google.com>");
MODULE_LICENSE("GPL");
