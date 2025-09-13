// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright 2023 Google LLC
 *
 * MAX77779 USB contaminant detection
 */

#include <linux/device.h>
#include <linux/irqreturn.h>
#include <linux/module.h>
#include <linux/usb/tcpm.h>
#include <linux/usb/typec.h>
#include <misc/logbuffer.h>

#include "max77759_helper.h"
#include "max777x9_contaminant.h"
#include "tcpci_max77759.h"
#include "tcpci_max77779_vendor_reg.h"
#include "google_tcpci_shim.h"

#define SARADC_1uA_LSB_UV		4900
/* TODO: High range CC */
#define SARADC_CC_HIGH_RANGE_LSB_MV	208
/* TODO: Low range CC */
#define SARADC_CC_LOW_RANGE_LSB_MV      126

/* 1uA current source */
#define SARADC_CC_SCALE1			1
/* 5 uA current source */
#define SARADC_CC_SCALE2			5

#define SARADC_1uA_CC_OFFSET_MV		0
#define SARADC_CC_HIGH_RANGE_OFFSET_MV	624
#define SARADC_CC_LOW_RANGE_OFFSET_MV	378

/* Actually translates to 18.7K */
#define ACCESSORY_THRESHOLD_CC_K	25
#define CONTAMINANT_THRESHOLD_SBU_K	1000
#define CONTAMINANT_THRESHOLD_CC_K	1000

static u32 adc_to_uv(struct max777x9_contaminant *contaminant, enum adc_select channel,
		     bool ua_src, u8 saradc_status)
{
	u32 adc_uv = saradc_status;

	/* SBU channels only have 1 scale with 1uA. */
	if ((ua_src && (channel == CC1_SCALE2 || channel == CC2_SCALE2 || channel == SBU1 ||
			channel == SBU2)))
		/* Mean of range */
		adc_uv = SARADC_1uA_CC_OFFSET_MV + (saradc_status * SARADC_1uA_LSB_UV);
	else if (!ua_src && (channel == CC1_SCALE1 || channel == CC2_SCALE1))
		adc_uv = SARADC_CC_HIGH_RANGE_OFFSET_MV +
			 (saradc_status * SARADC_CC_HIGH_RANGE_LSB_MV);
	else if (!ua_src && (channel == CC1_SCALE2 || channel == CC2_SCALE2))
		adc_uv = SARADC_CC_LOW_RANGE_OFFSET_MV +
			(saradc_status * SARADC_CC_LOW_RANGE_LSB_MV);
	else
		logbuffer_log(contaminant->chip->log, "ADC ERROR: SCALE UNKNOWN");

	return adc_uv;
}

static int read_adc_uv(struct max777x9_contaminant *contaminant,
		       enum adc_select channel, int sleep_msec, bool raw,
		       bool ua_src)
{
	struct regmap *regmap = contaminant->chip->data.regmap;
	u8 saradc_status;
	struct logbuffer *log = contaminant->chip->log;
	int ret;

	/* Set VBUS_VOLT_MON = 1 for ADC measurement */
	ret = max77759_update_bits8(regmap, TCPC_POWER_CTRL, TCPC_POWER_CTRL_VBUS_VOLT_MON,
				    TCPC_POWER_CTRL_VBUS_VOLT_MON);
	if (ret < 0)
		return -EIO;

	/* Channel & scale select */
	ret = max77759_update_bits8(regmap, TCPC_VENDOR_ADC_CTRL1, ADCINSEL_MASK,
				    channel << ADC_CHANNEL_OFFSET);
	if (ret < 0)
		return -EIO;

	/* Enable ADC */
	ret = max77759_update_bits8(regmap, TCPC_VENDOR_ADC_CTRL1, ADCEN, ADCEN);
	if (ret < 0)
		return -EIO;

	MAX77759_LOG_REGISTER(regmap, TCPC_VENDOR_ADC_CTRL1, log);
	/* SAR_ADC_STS when set indicated  valid data in ADC */
	MAX77759_LOG_REGISTER(regmap, VENDOR_CC_STATUS1, log);

	usleep_range(sleep_msec * 1000, (sleep_msec + 1) * 1000);
	ret = max77759_read8(regmap, TCPC_VENDOR_SARADC_STATUS, &saradc_status);
	if (ret < 0)
		return -EIO;
	logbuffer_log(log, "Contaminant: ADC %u", saradc_status);
	/* SAR_ADC_STS when set indicated  valid data in ADC */
	MAX77759_LOG_REGISTER(regmap, VENDOR_CC_STATUS1, log);

	/* Disable ADC */
	ret = max77759_update_bits8(regmap, TCPC_VENDOR_ADC_CTRL1, ADCEN, 0);
	if (ret < 0)
		return -EIO;

	ret = max77759_update_bits8(regmap, TCPC_VENDOR_ADC_CTRL1, ADCINSEL_MASK, 0);
	if (ret < 0)
		return -EIO;

	ret = max77759_update_bits8(regmap, TCPC_POWER_CTRL, TCPC_POWER_CTRL_VBUS_VOLT_MON, 0);
	if (ret < 0)
		return -EIO;

	if (!raw)
		return adc_to_uv(contaminant, channel, ua_src, saradc_status);
	else
		return saradc_status;
}

int max77779_read_resistance_kohm(struct max777x9_contaminant *contaminant,
				  enum adc_select channel, int sleep_msec,
				  bool raw)
{
	struct regmap *regmap = contaminant->chip->data.regmap;
	struct logbuffer *log = contaminant->chip->log;
	int uv;
	u8 switch_setting;
	int ret;

	if (channel == CC1_SCALE1 || channel == CC2_SCALE1 || channel == CC1_SCALE2 ||
	    channel == CC2_SCALE2) {
		/* Enable 1uA current source */
		ret = max77759_update_bits8(regmap, TCPC_VENDOR_CC_CTRL2, CCLPMODESEL_MASK,
					    ULTRA_LOW_POWER_MODE);
		if (ret < 0)
			return -EIO;

		/* Enable 1uA current source */
		ret = max77759_update_bits8(regmap, TCPC_VENDOR_CC_CTRL2, CCRPCTRL_MASK, UA_1_SRC);
		if (ret < 0)
			return -EIO;

		/* OVP disable */
		ret = max77759_update_bits8(regmap, TCPC_VENDOR_CC_CTRL2, CCOVPDIS, CCOVPDIS);
		if (ret < 0)
			return -EIO;

		MAX77759_LOG_REGISTER(regmap, TCPC_VENDOR_CC_CTRL2, log);

		uv = read_adc_uv(contaminant, channel, sleep_msec, raw, true);
		/* OVP enable */
		ret = max77759_update_bits8(regmap, TCPC_VENDOR_CC_CTRL2, CCOVPDIS, 0);
		if (ret < 0)
			return -EIO;
		/* returns KOhm as 1uA source is used. */
		return uv / 1000;
	}

	logbuffer_log(log, "Contaminant: SBU read");
	/*
	 * SBU measurement
	 * OVP disable
	 */

	ret = max77759_update_bits8(regmap, TCPC_VENDOR_SBU_CTRL1, SBUOVPDIS, SBUOVPDIS);
	if (ret < 0)
		return -EIO;

	/* Cache switch setting */
	ret = max77759_read8(regmap, TCPC_VENDOR_SBUSW_CTRL, &switch_setting);
	if (ret < 0)
		return -EIO;
	MAX77759_LOG_REGISTER(regmap, TCPC_VENDOR_SBUSW_CTRL, log);

	/* 1ua current source enable */
	ret = max77759_update_bits8(regmap, TCPC_VENDOR_SBU_CTRL1, SBUULPSRCSEL | SBURPCTRL_ULP_EN,
				    SBUULPSRC_1UA | SBURPCTRL_ULP_EN);
	if (ret < 0)
		return -EIO;

	MAX77759_LOG_REGISTER(regmap, TCPC_VENDOR_SBU_CTRL1, log);

	uv = read_adc_uv(contaminant, channel, sleep_msec, raw, true);
	/* Disable current source */
	ret = max77759_update_bits8(regmap, TCPC_VENDOR_SBU_CTRL1, SBUULPSRCSEL | SBURPCTRL_ULP_EN,
				    0);
	if (ret < 0)
		return -EIO;
	/* Set switch to original setting */
	ret = max77759_write8(regmap, TCPC_VENDOR_SBUSW_CTRL, switch_setting);
	if (ret < 0)
		return -EIO;

	/* OVP enable */
	ret = max77759_update_bits8(regmap, TCPC_VENDOR_SBU_CTRL1, SBUOVPDIS, 0);
	if (ret < 0)
		return -EIO;

	/*
	 * 1ua current source on sbu;
	 * return KOhm
	 */
	logbuffer_log(contaminant->chip->log, "Contaminant: SBU read %#x", uv);
	return uv / 1000;
}
EXPORT_SYMBOL_GPL(max77779_read_resistance_kohm);

int max77779_read_comparators(struct max777x9_contaminant *contaminant, u8 *vendor_cc_status2_cc1,
			      u8 *vendor_cc_status2_cc2)
{
	struct regmap *regmap = contaminant->chip->data.regmap;
	struct logbuffer *log = contaminant->chip->log;
	int ret;

	logbuffer_log(log, "Contaminant: enable comparators");

	/* Enable 80uA source */
	ret = max77759_update_bits8(regmap, TCPC_VENDOR_CC_CTRL2, CCRPCTRL_MASK, UA_80_SRC);
	if (ret < 0)
		return -EIO;

	/* Enable comparators */
	ret = max77759_update_bits8(regmap, TCPC_VENDOR_CC_CTRL1, CCCOMPEN, CCCOMPEN);
	if (ret < 0)
		return -EIO;

	MAX77759_LOG_REGISTER(regmap, TCPC_VENDOR_CC_CTRL1, log);

	/* Disable low power mode */
	ret = max77759_update_bits8(regmap, TCPC_VENDOR_CC_CTRL2, CCLPMODESEL_MASK,
				    LOW_POWER_MODE_DISABLE);
	if (ret < 0)
		return -EIO;
	MAX77759_LOG_REGISTER(regmap, TCPC_VENDOR_CC_CTRL2, log);

	/* Sleep to allow comparators settle */
	usleep_range(5000, 6000);
	ret = max77759_update_bits8(regmap, TCPC_TCPC_CTRL, TCPC_TCPC_CTRL_ORIENTATION,
				    PLUG_ORNT_CC1);
	if (ret < 0)
		return -EIO;
	MAX77759_LOG_REGISTER(regmap, TCPC_TCPC_CTRL, log);

	usleep_range(5000, 6000);
	ret = max77759_read8(regmap, VENDOR_CC_STATUS2, vendor_cc_status2_cc1);
	if (ret < 0)
		return -EIO;
	logbuffer_log(log, "Contaminant: VENDOR_CC_STATUS2: %u", *vendor_cc_status2_cc1);

	ret = max77759_update_bits8(regmap, TCPC_TCPC_CTRL, TCPC_TCPC_CTRL_ORIENTATION,
				    PLUG_ORNT_CC2);
	if (ret < 0)
		return -EIO;
	MAX77759_LOG_REGISTER(regmap, TCPC_TCPC_CTRL, log);

	usleep_range(5000, 6000);
	ret = max77759_read8(regmap, VENDOR_CC_STATUS2, vendor_cc_status2_cc2);
	if (ret < 0)
		return -EIO;
	logbuffer_log(contaminant->chip->log, "Contaminant: VENDOR_CC_STATUS2: %u",
		      *vendor_cc_status2_cc2);

	ret = max77759_update_bits8(regmap, TCPC_VENDOR_CC_CTRL1, CCCOMPEN, 0);
	if (ret < 0)
		return -EIO;
	ret = max77759_update_bits8(regmap, TCPC_VENDOR_CC_CTRL2, CCRPCTRL_MASK, 0);
	if (ret < 0)
		return -EIO;

	return 0;
}
EXPORT_SYMBOL_GPL(max77779_read_comparators);

int max77779_detect_contaminant(struct max777x9_contaminant *contaminant)
{
	int cc1_k, cc2_k, sbu1_k, sbu2_k, ret;
	u8 vendor_cc_status2_cc1 = 0xff, vendor_cc_status2_cc2 = 0xff;
	u8 role_ctrl = 0, role_ctrl_backup = 0;
	struct max77759_plat *chip = contaminant->chip;
	int inferred_state = NOT_DETECTED;
	struct regmap *regmap = contaminant->chip->data.regmap;

	ret = max77759_read8(regmap, TCPC_ROLE_CTRL, &role_ctrl);
	if (ret < 0)
		return -EIO;
	role_ctrl_backup = role_ctrl;
	role_ctrl = 0x0F;
	ret = max77759_write8(regmap, TCPC_ROLE_CTRL, role_ctrl);
	if (ret < 0)
		return -EIO;

	/* CCLPMODESEL_AUTO_LOW_POWER in use. */
	cc1_k = max77779_read_resistance_kohm(contaminant, CC1_SCALE2, READ1_SLEEP_MS, false);
	cc2_k = max77779_read_resistance_kohm(contaminant, CC2_SCALE2, READ2_SLEEP_MS, false);
	logbuffer_log(chip->log, "Contaminant: cc1_k:%u cc2_k:%u", cc1_k, cc2_k);

	sbu1_k = max77779_read_resistance_kohm(contaminant, SBU1, READ1_SLEEP_MS, false);
	sbu2_k = max77779_read_resistance_kohm(contaminant, SBU2, READ2_SLEEP_MS, false);
	logbuffer_log(chip->log, "Contaminant: sbu1_k:%u sbu2_k:%u", sbu1_k, sbu2_k);
	ret = max77779_read_comparators(contaminant, &vendor_cc_status2_cc1,
					&vendor_cc_status2_cc2);
	if (ret == -EIO)
		return ret;
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
			 * Do not enable dry detection for floating cable to allow
			 * TotalPhase analyzer to work as it presents ~600k in
			 * one of the CC pins.
			 */
			inferred_state = FLOATING_CABLE;
		}
	}

	if (inferred_state == NOT_DETECTED)
		ret = max77759_write8(regmap, TCPC_ROLE_CTRL, role_ctrl_backup);
	else
		ret = max77759_write8(regmap, TCPC_ROLE_CTRL, (TCPC_ROLE_CTRL_DRP | 0xA));

	if (ret < 0)
		return -EIO;

	return inferred_state;
}
EXPORT_SYMBOL_GPL(max77779_detect_contaminant);

int max77779_enable_dry_detection(struct max777x9_contaminant *contaminant)
{
	struct regmap *regmap = contaminant->chip->data.regmap;
	struct max77759_plat *chip = contaminant->chip;
	u8 temp;
	int ret;

	/*
	 * tunable: 1ms water detection debounce
	 * tunable: 1000mV/1000K threshold for water detection
	 * tunable: 4.8s water cycle
	 */
	ret = max77759_update_bits8(regmap, TCPC_VENDOR_CC_CTRL3, CCWTRDEB_MASK | CCWTRSEL_MASK
				    | WTRCYCLE_MASK | SBU_DET_EN, CCWTRDEB_1MS << CCWTRDEB_SHIFT |
				    CCWTRSEL_1V << CCWTRSEL_SHIFT | WTRCYCLE_4_8_S <<
				    WTRCYCLE_SHIFT);
	if (ret < 0)
		return -EIO;

	ret = max77759_update_bits8(regmap, TCPC_ROLE_CTRL, TCPC_ROLE_CTRL_DRP,
				    TCPC_ROLE_CTRL_DRP);
	if (ret < 0)
		return -EIO;

	/* tunable: 1ua / Ultra low power mode enabled. */
	ret = max77759_update_bits8(regmap, TCPC_VENDOR_CC_CTRL1, CCCONNDRY, CCCONNDRY);
	if (ret < 0)
		return -EIO;
	ret = max77759_read8(regmap, TCPC_VENDOR_CC_CTRL1, &temp);
	if (ret < 0)
		return -EIO;
	logbuffer_log(chip->log, "Contaminant: TCPC_VENDOR_CC_CTRL1 %u", temp);

	ret = max77759_update_bits8(regmap, TCPC_VENDOR_CC_CTRL2, CCLPMODESEL_MASK,
				    ULTRA_LOW_POWER_MODE);
	if (ret < 0)
		return -EIO;
	ret = max77759_read8(regmap, TCPC_VENDOR_CC_CTRL2, &temp);
	if (ret < 0)
		return -EIO;
	logbuffer_log(chip->log, "Contaminant: TCPC_VENDOR_CC_CTRL2 %u", temp);

	/* Enable Look4Connection before sending the command */
	ret = max77759_update_bits8(regmap, TCPC_TCPC_CTRL, TCPC_TCPC_CTRL_EN_LK4CONN_ALRT,
				    TCPC_TCPC_CTRL_EN_LK4CONN_ALRT);
	if (ret < 0)
		return -EIO;

	ret = max77759_write8(regmap, TCPC_COMMAND, TCPC_CMD_LOOK4CONNECTION);
	if (ret < 0)
		return -EIO;

	MAX77759_LOG_REGISTER(regmap, TCPC_VENDOR_CC_CTRL1, chip->log);
	MAX77759_LOG_REGISTER(regmap, TCPC_VENDOR_CC_CTRL2, chip->log);
	MAX77759_LOG_REGISTER(regmap, TCPC_VENDOR_CC_CTRL3, chip->log);

	logbuffer_log(chip->log, "Contaminant: Dry detection enabled");
	return 0;
}
EXPORT_SYMBOL_GPL(max77779_enable_dry_detection);

int max77779_disable_contaminant_detection(struct max77759_plat *chip)
{
	struct regmap *regmap = chip->data.regmap;
	struct max777x9_contaminant *contaminant = chip->contaminant;
	int ret;

	if (!contaminant)
		return 0;

	ret = max77759_write8(regmap, TCPC_ROLE_CTRL, TCPC_ROLE_CTRL_DRP |
			      (TCPC_ROLE_CTRL_CC_RD << TCPC_ROLE_CTRL_CC1_SHIFT) |
			      (TCPC_ROLE_CTRL_CC_RD << TCPC_ROLE_CTRL_CC2_SHIFT));
	if (ret < 0)
		return -EIO;

	ret = max77759_update_bits8(regmap, TCPC_VENDOR_CC_CTRL2, CCLPMODESEL_MASK,
				    LOW_POWER_MODE_DISABLE);
	if (ret < 0)
		return -EIO;

	ret = max77759_update_bits8(regmap, TCPC_TCPC_CTRL, TCPC_TCPC_CTRL_EN_LK4CONN_ALRT,
				    TCPC_TCPC_CTRL_EN_LK4CONN_ALRT);
	if (ret < 0)
		return -EIO;

	ret = max77759_update_bits8(regmap, TCPC_VENDOR_CC_CTRL1, CCCONNDRY, 0);
	if (ret < 0)
		return -EIO;

	ret = max77759_write8(regmap, TCPC_COMMAND, TCPC_CMD_LOOK4CONNECTION);
	if (ret < 0)
		return -EIO;

	/* Reset state before disabling detection */
	if (contaminant->state != NOT_DETECTED && contaminant->state != SINK)
		contaminant->state = NOT_DETECTED;

	MAX77759_LOG_REGISTER(regmap, TCPC_VENDOR_CC_CTRL1, chip->log);
	MAX77759_LOG_REGISTER(regmap, TCPC_VENDOR_CC_CTRL2, chip->log);
	MAX77759_LOG_REGISTER(regmap, TCPC_VENDOR_CC_CTRL3, chip->log);

	logbuffer_log(chip->log, "Contaminant: Contaminant detection disabled");

	return 0;
}
EXPORT_SYMBOL_GPL(max77779_disable_contaminant_detection);

int max77779_enable_contaminant_detection(struct max77759_plat *chip)
{
	struct regmap *regmap = chip->data.regmap;
	struct max777x9_contaminant *contaminant = chip->contaminant;
	u8 pwr_ctrl;
	int ret;

	if (!contaminant)
		return -EAGAIN;

	/*
	 * tunable: 1ms water detection debounce
	 * tunable: 1000mV/1000K threshold for water detection
	 * tunable: SBU detection disable
	 * tunable: 4.8s water cycle
	 */
	ret = max77759_update_bits8(regmap, TCPC_VENDOR_CC_CTRL3, CCWTRDEB_MASK | CCWTRSEL_MASK
				    | WTRCYCLE_MASK | SBU_DET_EN, CCWTRDEB_1MS << CCWTRDEB_SHIFT |
				    CCWTRSEL_1V << CCWTRSEL_SHIFT | WTRCYCLE_4_8_S <<
				    WTRCYCLE_SHIFT);
	if (ret < 0)
		return -EIO;

	/* Contaminant detection mode: contaminant detection */
	ret = max77759_update_bits8(regmap, TCPC_VENDOR_CC_CTRL1, CCCONNDRY, 0);
	if (ret < 0)
		return -EIO;

	if (!contaminant->auto_ultra_low_power_mode_disabled) {
		/* tunable: Periodic contaminant detection */
		ret = max77759_update_bits8(regmap, TCPC_VENDOR_CC_CTRL2, CCLPMODESEL_MASK,
					    AUTO_ULTRA_LOW_POWER_MODE);
		if (ret < 0)
			return -EIO;
	}

	MAX77759_LOG_REGISTER(regmap, TCPC_VENDOR_CC_CTRL1, chip->log);
	MAX77759_LOG_REGISTER(regmap, TCPC_VENDOR_CC_CTRL2, chip->log);
	MAX77759_LOG_REGISTER(regmap, TCPC_VENDOR_CC_CTRL3, chip->log);

	/* Mask flash adc interrupt */
	ret = max77759_update_bits8(regmap, TCPC_VENDOR_ALERT_MASK2, MSK_FLASH_ADCINT, 0);
	if (ret < 0)
		return -EIO;

	/* Disable Auto disacharge before enabling toggling */
	ret = max77759_read8(regmap, TCPC_POWER_CTRL, &pwr_ctrl);
	if (ret < 0)
		return -EIO;
	logbuffer_log(chip->log, "TCPC_POWER_CTRL:0x%x ret:%d", pwr_ctrl, ret);
	if (pwr_ctrl & TCPC_POWER_CTRL_AUTO_DISCHARGE) {
		logbuffer_log(chip->log, "TCPC_POWER_CTRL_AUTO_DISCHARGE not cleared");
		ret = regmap_update_bits(regmap, TCPC_POWER_CTRL, TCPC_POWER_CTRL_AUTO_DISCHARGE,
					 0);
		if (ret < 0) {
			logbuffer_log(chip->log, "[%s]: Disabling auto discharge failed", __func__);
			return -EIO;
		}
	}

	ret = max77759_write8(regmap, TCPC_ROLE_CTRL, TCPC_ROLE_CTRL_DRP |
			      (TCPC_ROLE_CTRL_CC_RD <<
			       TCPC_ROLE_CTRL_CC1_SHIFT) |
			      (TCPC_ROLE_CTRL_CC_RD <<
			       TCPC_ROLE_CTRL_CC2_SHIFT));
	if (ret < 0) {
		logbuffer_log(chip->log, "[%s]: Enabling DRP failed ret:%d", __func__,
			      ret);
		return -EIO;
	}

	/* Enable Look4Connection before sending the command */
	ret = max77759_update_bits8(regmap, TCPC_TCPC_CTRL, TCPC_TCPC_CTRL_EN_LK4CONN_ALRT,
				    TCPC_TCPC_CTRL_EN_LK4CONN_ALRT);
	if (ret < 0) {
		logbuffer_log(chip->log, "[%s]: Enabling looking for connection failed ret:%d",
			      __func__, ret);
		return -EIO;
	}

	ret = max77759_write8(regmap, TCPC_COMMAND, TCPC_CMD_LOOK4CONNECTION);
	if (ret < 0)
		return -EIO;

	logbuffer_log(chip->log, "Contaminant: Contaminant detection enabled");

	return 0;
}
EXPORT_SYMBOL_GPL(max77779_enable_contaminant_detection);

void max77779_disable_auto_ultra_low_power_mode(struct max77759_plat *chip, bool disable)
{
	struct max777x9_contaminant *contaminant = chip->contaminant;
	int ret;

	if (!chip || !chip->contaminant)
		return;

	if (contaminant->auto_ultra_low_power_mode_disabled == disable) {
		logbuffer_log(chip->log, "Auto ultra low power mode already %s",
			      disable ? "disable" : "enable");
		return;
	}

	ret = max77759_update_bits8(chip->data.regmap, TCPC_VENDOR_CC_CTRL2, CCLPMODESEL_MASK,
				    disable ? LOW_POWER_MODE_DISABLE : AUTO_ULTRA_LOW_POWER_MODE);

	logbuffer_log(chip->log, "Contaminant: Auto ultra low power mode %s ret:%d",
		      disable ? "disable" : "enable", ret);
	if (!ret)
		contaminant->auto_ultra_low_power_mode_disabled = disable;
}
EXPORT_SYMBOL_GPL(max77779_disable_auto_ultra_low_power_mode);

MODULE_DESCRIPTION("MAX77759_CONTAMINANT Module");
MODULE_AUTHOR("Badhri Jagan Sridharan <badhri@google.com>");
MODULE_LICENSE("GPL");
