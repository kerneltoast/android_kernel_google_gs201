// SPDX-License-Identifier: GPL-2.0-only
/*
 * Driver for the HL7132 battery charger.
 *
 * Copyright (C) 2024 Google, LLC.
 */

#include <linux/err.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/of_irq.h>
#include <linux/of_device.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/rtc.h>
#include <misc/gvotable.h>

#include "hl7132_regs.h"
#include "hl7132_charger.h"

#if defined(CONFIG_OF)
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#endif /* CONFIG_OF */

/* Timer definition */
#define HL7132_VBATMIN_CHECK_T	1000	/* 1000ms */
#define HL7132_CCMODE_CHECK1_T	5000	/* 10000ms -> 500ms */
#define HL7132_CCMODE_CHECK2_T	5000	/* 5000ms */
#define HL7132_CVMODE_CHECK_T	10000	/* 10000ms */
#define HL7132_ENABLE_DELAY_T	150	/* 150ms */
#define HL7132_CVMODE_CHECK2_T	1000	/* 1000ms */

/* Battery Threshold */
#define HL7132_DC_VBAT_MIN		3400000 /* uV */
/* Input Current Limit default value */
#define HL7132_IIN_CFG_DFT		3000000 /* uA*/
/* Charging vbat_reg default value */
#define HL7132_VBAT_REG_DFT		4350000	/* uV */
/* Charging vbat_reg max voltage value */
#define HL7132_COMP_VFLOAT_MAX		4600000	/* uV */

/* Sense Resistance default value */
#define HL7132_SENSE_R_DFT		1	/* 10mOhm */
/* Switching Frequency default value */
#define HL7132_FSW_CFG_DFT		3	/* 980KHz */
/* NTC threshold voltage default value */
#define HL7132_NTC_TH_DFT		0	/* uV*/

/* Charging Done Condition */
#define HL7132_IIN_DONE_DFT	500000		/* uA */
/* parallel charging done condition */
#define HL7132_IIN_P_DONE	1000000		/* uA */
/* Parallel charging default threshold */
#define HL7132_IIN_P_TH_DFT	4000000		/* uA */
/* Single charging default threshold */
#define HL7132_IIN_S_TH_DFT	10000000	/* uA */

/* Maximum TA voltage threshold */
#define HL7132_TA_MAX_VOL		9800000 /* uV */
/* Maximum TA current threshold, set to max(cc_max) / 2 */
#define HL7132_TA_MAX_CUR		2600000	 /* uA */
/* Minimum TA current threshold */
#define HL7132_TA_MIN_CUR		1000000	/* uA - PPS minimum current */

/* Minimum TA voltage threshold in Preset mode */
#define HL7132_TA_MIN_VOL_PRESET	8000000	/* uV */
/* TA voltage threshold starting Adjust CC mode */
#define HL7132_TA_MIN_VOL_CCADJ	8500000	/* 8000000uV --> 8500000uV */

#define HL7132_TA_VOL_PRE_OFFSET	500000	 /* uV */
/* Adjust CC mode TA voltage step */
#define HL7132_TA_VOL_STEP_ADJ_CC	40000	/* uV */
/* Pre CV mode TA voltage step */
#define HL7132_TA_VOL_STEP_PRE_CV	20000	/* uV */

/* IIN_CC adc offset for accuracy */
#define HL7132_IIN_ADC_OFFSET		20000	/* uA */
/* IIN_CC compensation offset */
#define HL7132_IIN_CC_COMP_OFFSET	25000	/* uA */
/* IIN_CC compensation offset in Power Limit Mode(Constant Power) TA */
#define HL7132_IIN_CC_COMP_OFFSET_CP	20000	/* uA */
/* TA maximum voltage that can support CC in Constant Power Mode */
#define HL7132_TA_MAX_VOL_CP		9800000	/* 9760000uV --> 9800000uV */
/* Offset for cc_max / 2 */
#define HL7132_IIN_MAX_OFFSET		0
/* Offset for TA max current */
#define HL7132_TA_CUR_MAX_OFFSET	200000 /* uA */

/* 0.6V - above this the chip is enabled, below it is disabled via a
 * "thermal shutdown"
 */
#define HL7132_TS_ENABLE_THRESHOLD 600000


/* maximum retry counter for restarting charging or initializing */
#define HL7132_MAX_RETRY_CNT		3	/* retries */
/* TA IIN tolerance */
#define HL7132_TA_IIN_OFFSET		100000	/* uA */
/* IIN_CC upper protection offset in Power Limit Mode TA */
#define HL7132_IIN_CC_UPPER_OFFSET	50000	/* 50mA */

/* PD Message Voltage and Current Step */
#define PD_MSG_TA_VOL_STEP		20000	/* uV */
#define PD_MSG_TA_CUR_STEP		50000	/* uA */

#define HL7132_OTV_MARGIN		12000	/* uV */

#define HL7132_TIER_SWITCH_DELTA	25000	/* uV */

/* INT1 Register Buffer */
enum {
	REG_INT1,
	REG_INT1_MSK,
	REG_INT1_STS,
	REG_INT1_MAX
};

/* STS Register Buffer */
enum {
	REG_STS_A,
	REG_STS_B,
	REG_STS_C,
	REG_STS_D,
	REG_STS_MAX
};

/* Status */
enum {
	STS_MODE_CHG_LOOP,	/* TODO: There is no such thing */
	STS_MODE_VFLT_LOOP,
	STS_MODE_IIN_LOOP,
	STS_MODE_IBAT_LOOP,
	STS_MODE_LOOP_INACTIVE,
	STS_MODE_TEMP_REG,
	STS_MODE_CHG_DONE,
	STS_MODE_VIN_UVLO,
	STS_MODE_UNKNOWN
};

/* Timer ID */
enum {
	TIMER_ID_NONE,
	TIMER_VBATMIN_CHECK,
	TIMER_PRESET_DC,
	TIMER_PRESET_CONFIG,
	TIMER_CHECK_ACTIVE,
	TIMER_ADJUST_CCMODE,
	TIMER_CHECK_CCMODE,
	TIMER_ENTER_CVMODE,
	TIMER_CHECK_CVMODE, /* 8 */
	TIMER_PDMSG_SEND,   /* 9 */
	TIMER_ADJUST_TAVOL,
	TIMER_ADJUST_TACUR,
};


/* TA increment Type */
enum {
	INC_NONE,	/* No increment */
	INC_TA_VOL,	/* TA voltage increment */
	INC_TA_CUR,	/* TA current increment */
};

/* BATT info Type */
enum {
	BATT_CURRENT,
	BATT_VOLTAGE,
};

/* ------------------------------------------------------------------------ */

static int hl7132_hw_ping(struct hl7132_charger *hl7132)
{
	unsigned int val = 0;
	int ret;

	/* Read Device info register to check the incomplete I2C operation */
	ret = regmap_read(hl7132->regmap, HL7132_REG_DEVICE_ID, &val);
	val = val & HL7132_BIT_DEV_ID;
	if ((ret < 0) || (val != HL7132_DEVICE_ID)) {
		ret = regmap_read(hl7132->regmap, HL7132_REG_DEVICE_ID, &val);
		val = val & HL7132_BIT_DEV_ID;
	}
	if ((ret < 0) || (val != HL7132_DEVICE_ID)) {
		dev_err(hl7132->dev,
			"reading DEVICE_ID failed, val=%#x ret=%d\n",
			val, ret);
		return -EINVAL;
	}

	return 0;
}

/* HW integration guide section 4
 * call holding mutex_lock(&hl7132->lock)
 */
static int hl7132_hw_init(struct hl7132_charger *hl7132)
{
	int ret = 0;

	unsigned int reg_value;
	unsigned int reg_ctrl_0, reg_ctrl_1, track_ov_uv, ctrl_0;
	unsigned int vbat_ovp_th, iin_ocp_th, iin_ucp_th;
	unsigned int track_ov, track_uv;

	/* HW integration guide section 4.1.1 */
	dev_info(hl7132->dev, "%s: Triggering soft reset\n", __func__);
	regmap_update_bits(hl7132->regmap, HL7132_REG_CTRL_2,
			   HL7132_BITS_SFT_RST,
			   HL7132_SFT_RESET << MASK2SHIFT(HL7132_BITS_SFT_RST));
	/* regmap_update_bits will always report a failure after soft reset,
	 * so confirm that it succeeded by making sure HL7132_REG_CTRL_2 is back
	 * to default after waiting for soft reset to complete - chip holds I2C
	 * BUS for ~6ms after reset is triggered. Wait 100ms as per HW
	 * integration guide.
	 */
	msleep(hl7132->pdata->init_sleep);

	ret = regmap_read(hl7132->regmap, HL7132_REG_CTRL_2, &reg_value);
	msleep(20);
	ret = regmap_read(hl7132->regmap, HL7132_REG_CTRL_2, &reg_value);
	if (ret < 0) {
		dev_err(hl7132->dev, "%s: Failed to read after soft reset\n",
			__func__);
		return ret;
	}
	if (reg_value != HL7132_CTRL_2_DFT) {
		dev_err(hl7132->dev, "%s: Failed to perform soft reset\n",
			__func__);
		return ret;
	}

	/* HW integration guide section 4.2.1 - check device ID */
	if (hl7132_hw_ping(hl7132))
		return ret;

	/* HW integration guide section 4.2.2 - Set TSBAT_EN_PIN - enable TS
	 * protection and set thresholds
	 */
	/* (used for HW integration guide section 5.6 - TBAT error control) */
	ret = regmap_update_bits(hl7132->regmap, HL7132_REG_CTRL_1,
				 HL7132_BIT_TS_PROT_EN, HL7132_BIT_TS_PROT_EN);
	if (ret < 0)
		return ret;
	ret = regmap_write(hl7132->regmap, HL7132_REG_TS0_TH_0,
			   HL7132_TS0_TH_0_INIT_DFT);
	if (ret < 0)
		return ret;
	ret = regmap_write(hl7132->regmap, HL7132_REG_TS0_TH_1,
			   HL7132_TS0_TH_1_INIT_DFT);
	if (ret < 0)
		return ret;

	/* TODO discussing with HW
	 * HW integration guide section 4.2.7 - set VIN_UV_SEL to 1
	 */
	//ret = regmap_update_bits(hl7132->regmap, HL7132_REG_CTRL_1,
	//			 HL7132_BIT_VIN_UV_SEL, HL7132_BIT_VIN_UV_SEL);
	//if (ret < 0)
	//	return ret;

	/* HW integration guide section 4.2.3 - Disable IBAT OCP */
	ret = regmap_update_bits(hl7132->regmap, HL7132_REG_IBAT_REG,
				 HL7132_BIT_IBAT_OCP_DIS, 1);
	if (ret < 0)
		return ret;

	/* HW integration guide section 4.2.4 - Confirm default protection thresholds */
	ret = regmap_read(hl7132->regmap, HL7132_REG_REG_CTRL_0, &reg_ctrl_0);
	if (ret < 0) {
		dev_err(hl7132->dev, "%s: Failed to read REG_CTRL_0, ret=%d\n",
			__func__, ret);
		return ret;
	}

	ret = regmap_read(hl7132->regmap, HL7132_REG_REG_CTRL_1, &reg_ctrl_1);
	if (ret < 0) {
		dev_err(hl7132->dev, "%s: Failed to read REG_CTRL_1, ret=%d\n",
			__func__, ret);
		return ret;
	}

	ret = regmap_read(hl7132->regmap, HL7132_REG_TRACK_OV_UV, &track_ov_uv);
	if (ret < 0) {
		dev_err(hl7132->dev, "%s: Failed to read TRACK_OV, ret=%d\n",
			__func__, ret);
		return ret;
	}

	ret = regmap_read(hl7132->regmap, HL7132_REG_CTRL_0, &ctrl_0);
	if (ret < 0) {
		dev_err(hl7132->dev, "%s: Failed to read CTRL_0, ret=%d\n",
			__func__, ret);
		return ret;
	}

	vbat_ovp_th = (reg_ctrl_1 & HL7132_BITS_VBAT_OVP_TH) >> MASK2SHIFT(HL7132_BITS_VBAT_OVP_TH);
	if ((vbat_ovp_th) != HL7132_VBAT_OVP_TH_DFT) {
		dev_warn(hl7132->dev,
			 "%s: Unexpected VBAT_OVP_TH value (0x%02x, expected 0x%02x)\n",
			 __func__, vbat_ovp_th, HL7132_VBAT_OVP_TH_DFT);
		return -EINVAL;
	}

	iin_ocp_th = (reg_ctrl_0 & HL7132_BITS_IIN_OCP_TH) >> MASK2SHIFT(HL7132_BITS_IIN_OCP_TH);
	if (iin_ocp_th != HL7132_IIN_OCP_TH_DFT) {
		dev_warn(hl7132->dev,
			 "%s: Unexpected IIN_OCP_TH value (0x%02x, expected 0x%02x)\n",
			 __func__, iin_ocp_th, HL7132_IIN_OCP_TH_DFT);
		return -EINVAL;
	}

	iin_ucp_th = (ctrl_0 & HL7132_BITS_IIN_UCP_TH) >> MASK2SHIFT(HL7132_BITS_IIN_UCP_TH);
	if (iin_ucp_th != HL7132_IIN_UCP_TH_DFT) {
		dev_warn(hl7132->dev,
			 "%s: Unexpected IIN_UCP_TH value (0x%02x, expected 0x%02x)\n",
			 __func__, iin_ucp_th, HL7132_IIN_UCP_TH_DFT);

		return -EINVAL;
	}

	track_ov = (track_ov_uv & HL7132_BITS_TRACK_OV) >> MASK2SHIFT(HL7132_BITS_TRACK_OV);
	if (track_ov != HL7132_TRACK_OV_DFT) {
		dev_warn(hl7132->dev,
			 "%s: Unexpected TRACK_OV value (0x%02x, expected 0x%02x)\n",
			 __func__, track_ov, HL7132_TRACK_OV_DFT);
		return -EINVAL;
	}

	track_uv = (track_ov_uv & HL7132_BITS_TRACK_UV) >> MASK2SHIFT(HL7132_BITS_TRACK_UV);
	if (track_uv != HL7132_TRACK_UV_DFT) {
		dev_warn(hl7132->dev,
			 "%s: Unexpected TRACK_UV value (0x%02x, expected 0x%02x)\n",
			 __func__, track_uv, HL7132_TRACK_UV_DFT);
		return -EINVAL;
	}

	/* HW integration guide section 4.2.5 */
	/* Unmask TS_TEMP interrupt */
	ret = regmap_update_bits(hl7132->regmap, HL7132_REG_INT_MSK,
				 HL7132_BIT_TS_TEMP_M, 0);
	if (ret < 0)
		return ret;

	/* Clear interrupt flags (read to clear) */
	ret = regmap_read(hl7132->regmap, HL7132_REG_INT, &reg_value);
	if (ret < 0)
		return ret;

	/* HW integration guide section 4.2.6 */
	/* Disable unused ADC channels */
	ret = regmap_write(hl7132->regmap, HL7132_REG_ADC_CTRL_1,
			   HL7132_ADC_CTRL_1_INIT_DFT);
	if (ret < 0)
		return ret;

	/* Enable ADC */
	ret = regmap_update_bits(hl7132->regmap, HL7132_REG_ADC_CTRL_0,
				 HL7132_BIT_ADC_EN, HL7132_BIT_ADC_EN);

	if (ret < 0)
		return ret;

	return 0;
}

/* HW integration guide section 5.2 Check SC Initialize Condition */
static int hl7132_check_init(struct hl7132_charger *hl7132)
{
	int ret;
	unsigned int reg_value;

	/* HW integration guide section 5.2.1 */
	ret = regmap_read(hl7132->regmap, HL7132_REG_IBAT_REG, &reg_value);
	if (ret < 0 || reg_value != HL7132_IBAT_REG_DFT) {
		dev_err(hl7132->dev,
			"%s: IBAT_REG not configured correctly (0x%02x, expected 0x%02x)\n",
			__func__, reg_value, HL7132_IBAT_REG_DFT);
		return -EINVAL;
	}

	/* HW integration guide section 5.2.2 */
	ret = regmap_read(hl7132->regmap, HL7132_REG_CTRL_1, &reg_value);
	if (ret < 0 || reg_value != HL7132_REG_CTRL_1_INIT_DFT) {
		dev_err(hl7132->dev,
			"%s: CTRL_1 not configured correctly (0x%02x, expected 0x%02x)\n",
			__func__, reg_value, HL7132_REG_CTRL_1_INIT_DFT);
		return -EINVAL;
	}

	ret = regmap_read(hl7132->regmap, HL7132_REG_TS0_TH_0, &reg_value);
	if (ret < 0 || reg_value != HL7132_TS0_TH_0_INIT_DFT) {
		dev_err(hl7132->dev,
			"%s: TS0_TH_0 not configured correctly (0x%02x, expected 0x%02x)\n",
			__func__, reg_value, HL7132_TS0_TH_0_INIT_DFT);
		return -EINVAL;
	}

	ret = regmap_read(hl7132->regmap, HL7132_REG_TS0_TH_1, &reg_value);
	if (ret < 0 || reg_value != HL7132_TS0_TH_1_INIT_DFT) {
		dev_err(hl7132->dev,
			"%s: TS0_TH_1 not configured correctly (0x%02x, expected 0x%02x)\n",
			__func__, reg_value, HL7132_TS0_TH_1_INIT_DFT);
		return -EINVAL;
	}

	/* HW integration guide section 5.2.3 */
	ret = regmap_read(hl7132->regmap, HL7132_REG_ADC_CTRL_0, &reg_value);
	if (ret < 0 || reg_value != HL7132_BIT_ADC_EN) {
		dev_err(hl7132->dev,
			"%s: ADC_CTRL_0 not configured correctly (0x%02x, expected 0x%02lx)\n",
			__func__, reg_value, HL7132_BIT_ADC_EN);
		return -EINVAL;
	}

	ret = regmap_read(hl7132->regmap, HL7132_REG_ADC_CTRL_1, &reg_value);
	if (ret < 0 || reg_value != HL7132_ADC_CTRL_1_INIT_DFT) {
		dev_err(hl7132->dev,
			"%s: ADC_CTRL_1 not configured correctly (0x%02x, expected 0x%02lx)\n",
			__func__, reg_value, HL7132_ADC_CTRL_1_INIT_DFT);
		return -EINVAL;
	}

	/* Section 5.3.1 - do here to consolidate initialization checks since
	 * VIN_OVP isn't being changed between 5.2 and 5.3
	 */
	ret = regmap_read(hl7132->regmap, HL7132_REG_VIN_OVP, &reg_value);
	if (ret < 0 || reg_value != HL7132_VIN_OVP_DFT) {
		dev_err(hl7132->dev,
			"%s: REG_VIN_OVP not configured correctly (0x%02x, expected 0x%02x)\n",
			__func__, reg_value, HL7132_VIN_OVP_DFT);
		return -EINVAL;
	}

	/* Section 5.3.2 - do here to consolidate initialization checks since
	 * REG_CTRL_0 / IIN_OCP_TH isn't being changed
	 */
	ret = regmap_read(hl7132->regmap, HL7132_REG_REG_CTRL_0, &reg_value);
	reg_value = (reg_value & HL7132_BITS_IIN_OCP_TH) >> MASK2SHIFT(HL7132_BITS_IIN_OCP_TH);
	if (ret < 0 || reg_value != HL7132_IIN_OCP_TH_DFT) {
		dev_err(hl7132->dev,
			"%s: REG_OCP_TH not configured correctly (0x%02x, expected 0x%02x)\n",
			__func__, reg_value, HL7132_IIN_OCP_TH_DFT);
		return -EINVAL;
	}

	return 0;
}

/* HW integration guide section 5.4.5 read ADC values */
/* ADC Read function, return uV or uA */
int hl7132_read_adc(struct hl7132_charger *hl7132, u8 adc_ch)
{
	u8 reg_data[2];
	u16 raw_adc = 0;
	int conv_adc = -1;
	int ret;

	switch (adc_ch) {
	case ADCCH_VIN:
		ret = regmap_bulk_read(hl7132->regmap, HL7132_REG_ADC_VIN_0, reg_data, 2);
		if (ret < 0)
			goto error;
		raw_adc = ((reg_data[0] << 2) | (reg_data[1] & HL7132_BITS_ADC_VIN_LSB));
		conv_adc = raw_adc * HL7132_VIN_STEP;
		break;

	case ADCCH_IIN:
		ret = regmap_bulk_read(hl7132->regmap, HL7132_REG_ADC_IIN_0, reg_data, 2);
		if (ret < 0)
			goto error;
		raw_adc = ((reg_data[0] << 2) | (reg_data[1] & HL7132_BITS_ADC_IIN_LSB));
		conv_adc = raw_adc * HL7132_IIN_STEP;
		break;

	case ADCCH_VBAT:
		ret = regmap_bulk_read(hl7132->regmap, HL7132_REG_ADC_VBAT_0, reg_data, 2);
		if (ret < 0)
			goto error;
		raw_adc = ((reg_data[0] << 2) | (reg_data[1] & HL7132_BITS_ADC_VBAT_LSB));
		conv_adc = raw_adc * HL7132_VBAT_STEP;
		break;

	/* case ADCCH_IBAT: not needed */

	case ADCCH_VTS:
		ret = regmap_bulk_read(hl7132->regmap, HL7132_REG_ADC_VTS_0, reg_data, 2);
		if (ret < 0)
			goto error;

		raw_adc = ((reg_data[0] << 2) | (reg_data[1] & HL7132_BITS_ADC_VTS_LSB));
		conv_adc = raw_adc * HL7132_VTS_STEP;
		break;

	case ADCCH_VOUT:
		ret = regmap_bulk_read(hl7132->regmap, HL7132_REG_ADC_VOUT_0, reg_data, 2);
		if (ret < 0)
			goto error;

		raw_adc = ((reg_data[0] << 2) | (reg_data[1] & HL7132_BITS_ADC_VOUT_LSB));
		conv_adc = raw_adc * HL7132_VOUT_STEP;
		break;

	case ADCCH_TDIE:
		ret = regmap_bulk_read(hl7132->regmap, HL7132_REG_ADC_TDIE_0, reg_data, 2);
		if (ret < 0)
			goto error;

		raw_adc = ((reg_data[0] << 2) | (reg_data[1] & HL7132_BITS_ADC_TDIE_LSB));
		conv_adc = raw_adc * HL7132_TDIE_STEP / HL7132_TDIE_DENOM;
		break;

	default:
		conv_adc = -EINVAL;
		break;
	}

error:
	/* if disabled a channel, re-enable it in -> HL7132_REG_ADC_CTRL_1 */

	dev_dbg(hl7132->dev, "%s: adc_ch=%u, raw_adc=%x convert_val=%d\n",
		__func__, adc_ch, raw_adc, conv_adc);

	return conv_adc;
}

/* vbat_reg voltage (10 mV) resolution */
static int hl7132_set_vbat_reg(struct hl7132_charger *hl7132,
			      unsigned int vbat_reg)
{
	const int val = HL7132_VBAT_REG(vbat_reg);
	int ret;

	ret = regmap_write(hl7132->regmap, HL7132_REG_VBAT_REG, val);

	dev_info(hl7132->dev, "%s: vbat_reg=%u (%d)\n", __func__, vbat_reg, ret);

	return ret;
}

static int hl7132_set_input_current(struct hl7132_charger *hl7132,
				    unsigned int iin)
{
	int ret, val;
	unsigned int iin_cfg;

	/* round-up and increase one step */
	iin = iin + PD_MSG_TA_CUR_STEP;
	val = HL7132_IIN_CFG(iin);

	/* Set IIN_CFG to one step higher */
	val = val + 1;

	/* If value goes above max, HW clamps to 3.5A */
	iin_cfg = HL7132_IIN_CFG_MIN + val * HL7132_IIN_CFG_STEP;
	if (iin_cfg > HL7132_IIN_CFG_MAX)
		iin_cfg = HL7132_IIN_CFG_MAX;

	ret = regmap_update_bits(hl7132->regmap, HL7132_REG_IIN_REG,
				 HL7132_BITS_IIN_REG_TH, val);

	dev_info(hl7132->dev, "%s: iin=%d real iin_cfg=%d (%d)\n", __func__,
		 iin, iin_cfg, ret);

	return ret;
}

static inline bool hl7132_can_inc_ta_cur(struct hl7132_charger *hl7132)
{
	return hl7132->ta_cur + PD_MSG_TA_CUR_STEP < min(hl7132->ta_max_cur,
		hl7132->iin_cc + HL7132_TA_CUR_MAX_OFFSET);
}

/* Returns the enable or disable value. into 1 or 0. */
static int hl7132_get_charging_enabled(struct hl7132_charger *hl7132)
{
	int sts = 0;
	int ret = 0;
	int chg_state;

	ret = regmap_read(hl7132->regmap, HL7132_REG_INT_STS_A, &sts);
	if (ret < 0)
		return ret;

	chg_state = sts >> 6;

	if (chg_state == STATE_CHG_STS_ACTIVE)
		return 1;

	return 0;
}

/* HW integration guide section 5.5 - Disable charging */
/* call holding mutex_lock(&hl7132->lock); */
static int hl7132_set_charging(struct hl7132_charger *hl7132, bool enable)
{
	int ret = 0;
	int value = 0;
	unsigned int r_val = 0;

	if (enable)
		value = HL7132_BIT_CHG_EN;

	ret = regmap_update_bits(hl7132->regmap, HL7132_REG_CTRL_0,
				 HL7132_BIT_CHG_EN, value);
	if (ret < 0)
		goto Err;

	ret = regmap_read(hl7132->regmap, HL7132_REG_CTRL_0, &r_val);
Err:
	dev_dbg(hl7132->dev, "%s: End, ret=%d, r_val=[%x]\n", __func__, ret, r_val);
	return ret;
}

static int hl7132_check_state(u8 val[8], struct hl7132_charger *hl7132, int loglevel)
{
	int ret, vin, vout, vbat, iin, tdie, ts;

	/* Dump registers from INT to STATUS_C */
	ret = regmap_bulk_read(hl7132->regmap, HL7132_REG_INT,
			       &val[HL7132_REG_INT], 7);
	if (ret < 0)
		return ret;

	logbuffer_prlog(hl7132, loglevel,
			"%s: INT: %#02x, INT_MSK: %#02x, INT_STS_A: %#02x, INT_STS_B: %#02x, STATUS_A: %#02x, STATUS_B: %#02x, STATUS_C: %#02x\n",
			__func__, val[HL7132_REG_INT], val[HL7132_REG_INT_MSK],
			val[HL7132_REG_INT_STS_A], val[HL7132_REG_INT_STS_B],
			val[HL7132_REG_STATUS_A], val[HL7132_REG_STATUS_B],
			val[HL7132_REG_STATUS_C]);

	vin = hl7132_read_adc(hl7132, ADCCH_VIN);
	vout = hl7132_read_adc(hl7132, ADCCH_VOUT);
	vbat = hl7132_read_adc(hl7132, ADCCH_VBAT);
	iin = hl7132_read_adc(hl7132, ADCCH_IIN);
	tdie = hl7132_read_adc(hl7132, ADCCH_TDIE);
	ts = hl7132_read_adc(hl7132, ADCCH_VTS);

	logbuffer_prlog(hl7132, loglevel,
			"%s: vin: %d, vout: %d, vbat: %d, iin: %d, tdie: %d, ts: %d\n",
			__func__, vin, vout, vbat, iin, tdie, ts);


	return 0;
}

/* HW integration guide section 5.4.4.b report failure information */
/* HL7132 is not active state  - standby or shutdown */
/* Stop charging in timer_work */
/* return 0 when no error is detected */
static int hl7132_check_not_active(struct hl7132_charger *hl7132)
{
	u8 val[8]; /* INT to STATUS_C */
	int ret;
	unsigned int masked_reg;

	ret = hl7132_check_state(val, hl7132, LOGLEVEL_WARNING);
	if (ret < 0) {
		dev_err(hl7132->dev, "%s: cannot read state\n", __func__);
		return ret;
	}

	if ((val[HL7132_REG_INT_STS_B] & HL7132_BIT_V_NOT_OK_STS) == HL7132_BIT_V_NOT_OK_STS) {
		logbuffer_prlog(hl7132, LOGLEVEL_ERR, "%s: VOK is invalid", __func__);

		if (val[HL7132_REG_STATUS_A] & HL7132_BIT_VIN_OVP_STS)
			logbuffer_prlog(hl7132, LOGLEVEL_ERR, "%s: VIN OVP", __func__);
		else if (val[HL7132_REG_STATUS_A] & HL7132_BIT_VIN_UVLO_STS)
			logbuffer_prlog(hl7132, LOGLEVEL_ERR, "%s: VIN UVLO", __func__);
		else if (val[HL7132_REG_STATUS_A] & HL7132_BIT_TRACK_OV_STS)
			logbuffer_prlog(hl7132, LOGLEVEL_ERR, "%s: TRACK OVP", __func__);
		else if (val[HL7132_REG_STATUS_A] & HL7132_BIT_TRACK_UV_STS)
			logbuffer_prlog(hl7132, LOGLEVEL_ERR, "%s: TRACK UVP", __func__);
		else if (val[HL7132_REG_STATUS_A] & HL7132_BIT_VBAT_OVP_STS)
			logbuffer_prlog(hl7132, LOGLEVEL_ERR, "%s: VBAT OVP", __func__);
		else if (val[HL7132_REG_STATUS_A] & HL7132_BIT_PMID_QUAL_STS)
			logbuffer_prlog(hl7132, LOGLEVEL_ERR, "%s: PMID_QUAL", __func__);
		else if (val[HL7132_REG_STATUS_A] & HL7132_BIT_VOUT_OVP_STS)
			logbuffer_prlog(hl7132, LOGLEVEL_ERR, "%s: VOUT OVP", __func__);

		return -EINVAL;
	}

	if (val[HL7132_REG_INT] & HL7132_BIT_TS_TEMP_I) {
		int ntc_adc, ntc_th_upper, ntc_th_lower; /* NTC protection */
		u8 reg_data[2];				 /* NTC threshold */

		ret = regmap_bulk_read(hl7132->regmap, HL7132_REG_TS0_TH_0,
					reg_data, sizeof(reg_data));
		if (ret < 0)
			return -EIO;

		masked_reg = reg_data[1] & HL7132_BITS_TS0_TH_MSB;
		ntc_th_upper = (masked_reg << 8) | reg_data[0]; /* uV unit */

		ret = regmap_bulk_read(hl7132->regmap, HL7132_REG_TS1_TH_0,
					reg_data, sizeof(reg_data));
		if (ret < 0)
			return -EIO;

		masked_reg = reg_data[1] & HL7132_BITS_TS1_TH_1_MSB;
		ntc_th_lower = (masked_reg << 8) | reg_data[0]; /* uV unit */

		/* Read NTC ADC */
		ntc_adc = hl7132_read_adc(hl7132, ADCCH_VTS); /* uV unit */
		logbuffer_prlog(hl7132, LOGLEVEL_ERR,
				"%s: NTC Protection, NTC_TH_UPPER=%d(uV), NTC_TH_LOWER=%d(uV) NTC_ADC=%d(uV)",
				__func__, ntc_th_upper, ntc_th_lower, ntc_adc);

		return -EINVAL;
	}

	if (val[HL7132_REG_INT_STS_B] & HL7132_BIT_CUR_STS) {
		if (val[HL7132_REG_STATUS_B] & HL7132_BIT_IIN_OCP_STS)
			logbuffer_prlog(hl7132, LOGLEVEL_ERR, "%s: IIN is over OCP", __func__);
		else if (val[HL7132_REG_STATUS_B] & HL7132_BIT_IBAT_OCP_STS)
			logbuffer_prlog(hl7132, LOGLEVEL_ERR, "%s: IBAT is over OCP", __func__);
		else if (val[HL7132_REG_STATUS_B] & HL7132_BIT_IIN_UCP_STS)
			logbuffer_prlog(hl7132, LOGLEVEL_ERR, "%s: IIN is under UCP", __func__);

		return -EINVAL;
	}

	if (val[HL7132_REG_INT_STS_B] & HL7132_BIT_SHORT_STS) {
		if (val[HL7132_REG_STATUS_B] & HL7132_BIT_FET_SHORT_STS)
			logbuffer_prlog(hl7132, LOGLEVEL_ERR,
					"%s: FET short is detected", __func__);
		else if (val[HL7132_REG_STATUS_B] & HL7132_BIT_CFLY_SHORT_STS)
			logbuffer_prlog(hl7132, LOGLEVEL_ERR,
					"%s: CFLY short is detected", __func__);

		return -EINVAL;
	}

	if (val[HL7132_REG_INT_STS_B] & HL7132_BIT_WDOG_STS) {
		/* Watchdog timer is expired */
		logbuffer_prlog(hl7132, LOGLEVEL_ERR,
				"%s: Watchdog timer is expired", __func__);
		return -EINVAL;
	}

	if (val[HL7132_REG_STATUS_B] & HL7132_BIT_THSD_STS) {
		/* Over temperature protection */
		logbuffer_prlog(hl7132, LOGLEVEL_ERR,
				"%s: Device is in thermal shutdown", __func__);
		return -EINVAL;
	}

	return 0;
}

/* Keep the current charging state, check INT_STS_B again */
/* return 0 if VIN is still present, -EAGAIN if needs to retry, -EINVAL oth */
static int hl7132_check_standby(struct hl7132_charger *hl7132)
{
	unsigned int reg_val, masked_reg;
	int ret, chg_state;
	u8 val[8];
	bool is_active;

	/* Re-read the state register */
	ret = regmap_read(hl7132->regmap, HL7132_REG_INT_STS_B, &reg_val);
	if (ret < 0)
		return -EIO;

	dev_dbg(hl7132->dev, "%s: RCP check, INT_STS_B=%#x\n", __func__,
		reg_val);

	/* Re-read and dump state */
	hl7132_check_state(val, hl7132, LOGLEVEL_INFO);
	masked_reg = val[HL7132_REG_INT_STS_A] & HL7132_BIT_STATE_CHG_STS;
	chg_state = masked_reg >> MASK2SHIFT(HL7132_BIT_STATE_CHG_STS);

	/* RCP condition, but VIN is valid and the HL7132 is active */
	/* V_NOT_OK_STS should be low for valid VIN */
	if ((reg_val & HL7132_BIT_V_NOT_OK_STS) && chg_state == STATE_CHG_STS_ACTIVE) {
		const int charging_state = hl7132->charging_state;

		/*
		 * Try again when called from hl7132_check_active_state().
		 * If VIN is increased, input current will increase over
		 * IIN_LOW level.
		 */
		is_active = charging_state == DC_STATE_CHECK_ACTIVE;
		logbuffer_prlog(hl7132,
				is_active ? LOGLEVEL_WARNING : LOGLEVEL_ERR,
				"%s: RCP triggered but VIN is valid, state=%d",
				__func__, charging_state);

		    hl7132->chg_data.rcp_count++;
		return -EAGAIN;
	}

	/* Not in RCP state, retry only when DC is starting */
	if (chg_state == STATE_CHG_STS_STANDBY) {
		logbuffer_prlog(hl7132, LOGLEVEL_WARNING, "%s: device in standby", __func__);
		hl7132->chg_data.stby_count++;
		ret = -EAGAIN;
	} else { /* chg_state == STATE_CHG_STS_SHUTDOWN - active checked above */
		logbuffer_prlog(hl7132, LOGLEVEL_WARNING, "%s: device in shutdown", __func__);
		ret = -EINVAL;
	}

	return ret;
}

/*
 * HW integration guide section 5.4.4.a check status of charging
 *
 * Check Active status, 0 is active (or in RCP), <0 indicates a problem.
 * The function is called from different contexts/functions, errors are fatal
 * (i.e. stop charging) from all contexts except when this is called from
 * hl7132_check_active_state().
 *
 * Other contexts:
 * . hl7132_charge_adjust_ccmode
 * . hl7132_charge_ccmode
 * . hl7132_charge_start_cvmode
 * . hl7132_charge_cvmode
 * . hl7132_adjust_ta_voltage
 * . hl7132_adjust_ta_current
 * call holding mutex_lock(&hl7132->lock)
 */
static int hl7132_check_error(struct hl7132_charger *hl7132)
{
	unsigned int reg_val, masked_reg;
	int ret, chg_state;
	int vbatt;

	ret = regmap_read(hl7132->regmap, HL7132_REG_INT_STS_A, &reg_val);
	if (ret < 0) {
		dev_err(hl7132->dev,
			"%s: Failed to read INT_STS_A, ret=%d\n", __func__, ret);
		goto error;
	}

	chg_state = reg_val >> 6;

	if (chg_state == STATE_CHG_STS_ACTIVE) {

		/* HL7132 is charging */

		/* Check whether the battery voltage is over the minimum */
		vbatt = hl7132_read_adc(hl7132, ADCCH_VBAT);
		if (vbatt > HL7132_DC_VBAT_MIN) {
			/* Normal charging battery level */
			/* Check temperature regulation loop */
			masked_reg = reg_val & HL7132_BIT_REG_STS;
			if ((masked_reg >> MASK2SHIFT(HL7132_BIT_REG_STS)) == REG_STS_TEMP) {
				/* Over temperature protection */
				dev_err(hl7132->dev,
					"%s: Device is in temperature regulation\n",
					__func__);
				ret = -EINVAL;
			}
		} else {
			/* Abnormal battery level */
			dev_err(hl7132->dev,
				"%s: Error abnormal battery voltage=%d\n",
				__func__, vbatt);
			ret = -EINVAL;
		}

		dev_dbg(hl7132->dev, "%s: Active Status ok=%d (ret=%d)\n",
			__func__, ret == 0, ret);
		return ret;
	}

	/* not in error but in standby or shutdown */

	ret = hl7132_check_not_active(hl7132);
	if (ret < 0) {
		/* There was an error, done... */
	} else if (chg_state == STATE_CHG_STS_SHUTDOWN) {
		/* HL7132 is in shutdown state */
		dev_err(hl7132->dev, "%s: HL7132 is in shutdown\n", __func__);
		ret = -EINVAL;
	} else if (hl7132->charging_state == DC_STATE_NO_CHARGING) {
		/*
		 * Sometimes battery driver might call set_property function
		 * to stop charging during msleep. At this case, charging
		 * state would change DC_STATE_NO_CHARGING. HL7132 should
		 * stop checking RCP condition and exit timer_work
		 */
		dev_err(hl7132->dev, "%s: other driver forced stop\n", __func__);
		ret = -EINVAL;
	} else {

		/* Check the RCP condition, T_REVI_DET is 300ms */
		msleep(200);

		/*
		 * return 0 if VIN is still present, -EAGAIN if needs to retry,
		 * -EINVAL on error.
		 */
		ret = hl7132_check_standby(hl7132);
	}

error:
	dev_dbg(hl7132->dev, "%s: Not Active Status=%d\n", __func__, ret);
	return ret;
}

static int hl7132_get_iin(struct hl7132_charger *hl7132, int *iin)
{
	const int offset = 0;
	int temp;

	temp = hl7132_read_adc(hl7132, ADCCH_IIN);
	if (temp < 0)
		return temp;

	if (temp < offset)
		temp = offset;

	*iin = (temp - offset) * 2; /* 2:1 */
	return 0;
}

static int hl7132_get_batt_info(struct hl7132_charger *hl7132, int info_type, int *info)
{
	union power_supply_propval val;
	enum power_supply_property psp;
	int ret;

	if (!hl7132->batt_psy)
		hl7132->batt_psy = power_supply_get_by_name("battery");
	if (!hl7132->batt_psy)
		return -EINVAL;

	if (info_type == BATT_CURRENT)
		psp = POWER_SUPPLY_PROP_CURRENT_NOW;
	else
		psp = POWER_SUPPLY_PROP_VOLTAGE_NOW;

	ret = power_supply_get_property(hl7132->batt_psy, psp, &val);
	if (ret == 0)
		*info = val.intval;

	return ret;
}

static int hl7132_get_ibatt(struct hl7132_charger *hl7132, int *info)
{
	return hl7132_get_batt_info(hl7132, BATT_CURRENT, info);
}

static void hl7132_prlog_state(struct hl7132_charger *hl7132, const char *fn)
{
	int rc, ibat, icn = -EINVAL, iin = -EINVAL;
	bool ovc_flag;

	rc = hl7132_get_ibatt(hl7132, &ibat);
	if (rc == 0)
		rc = hl7132_get_iin(hl7132, &icn);
	if (rc == 0)
		iin = hl7132_read_adc(hl7132, ADCCH_IIN);
	ovc_flag = ibat > hl7132->cc_max;
	if (ovc_flag)
		hl7132_chg_stats_inc_ovcf(&hl7132->chg_data, ibat, hl7132->cc_max);

	logbuffer_prlog(hl7132, ovc_flag ? LOGLEVEL_WARNING : LOGLEVEL_DEBUG,
			"%s: iin=%d, iin_cc=%d, icn=%d ibat=%d, cc_max=%d rc=%d",
			fn, iin, hl7132->iin_cc, icn, ibat, hl7132->cc_max, rc);
}

static int hl7132_read_status(struct hl7132_charger *hl7132)
{
	unsigned int reg_val;
	int ret;

	ret = regmap_read(hl7132->regmap, HL7132_REG_STATUS_A, &reg_val);
	if (ret < 0)
		return ret;

	if (reg_val & HL7132_BIT_VIN_UVLO_STS)
		return STS_MODE_VIN_UVLO;

	ret = regmap_read(hl7132->regmap, HL7132_REG_INT_STS_A, &reg_val);
	if (ret < 0)
		return ret;

	reg_val = (reg_val & HL7132_BIT_REG_STS) >> MASK2SHIFT(HL7132_BIT_REG_STS);

	switch (reg_val) {
	case REG_STS_NONE:
		return STS_MODE_LOOP_INACTIVE; /* No regulation loop active */
	case REG_STS_VBAT:
		return STS_MODE_VFLT_LOOP; /* Battery voltage regulation */
	case REG_STS_IIN:
		return STS_MODE_IIN_LOOP; /* Input current regulation */
	case REG_STS_IBAT:
		return STS_MODE_IBAT_LOOP; /* Battery current regulation (if applicable) */
	case REG_STS_TEMP:
		return STS_MODE_TEMP_REG; /* Temperature regulation */
	default:
		return STS_MODE_UNKNOWN; /* Unknown or reserved state */
	}
}

static int hl7132_const_charge_voltage(struct hl7132_charger *hl7132);

static int hl7132_check_status(struct hl7132_charger *hl7132)
{
	int icn = -EINVAL, ibat = -EINVAL, vbat = -EINVAL;
	int rc, status;

	status = hl7132_read_status(hl7132);
	if (status < 0)
		goto error;

	rc = hl7132_get_iin(hl7132, &icn);
	if (rc == 0)
		rc = hl7132_get_batt_info(hl7132, BATT_CURRENT, &ibat);
	if (rc == 0)
		rc = hl7132_get_batt_info(hl7132, BATT_VOLTAGE, &vbat);

error:
	dev_dbg(hl7132->dev,
		"%s: status=%d icn:%d ibat:%d delta_c=%d, vbat:%d, fv:%d, cc_max:%d\n",
		 __func__, status, icn, ibat, icn - ibat, vbat,
		 hl7132->fv_uv, hl7132->cc_max);

	return status;
}

/* hold mutex_lock(&hl7132->lock); */
static int hl7132_recover_ta(struct hl7132_charger *hl7132)
{
	int ret;

	dev_dbg(hl7132->dev, "%s: recovering TA\n", __func__);

	if (hl7132->ftm_mode)
		return 0;

	/* TODO: recover TA to value before handoff, or use DT */
	hl7132->ta_vol = 9000000;
	hl7132->ta_cur = 2200000;
	hl7132->ta_objpos = 1; /* PDO1 - fixed 5V */
	ret = hl7132_send_pd_message(hl7132, MSG_REQUEST_FIXED_PDO);

	/* will not be able to recover if TA is offline */
	if (ret < 0)
		dev_dbg(hl7132->dev, "%s: cannot recover TA (%d)\n", __func__,
			ret);

	return 0;
}

/* Stop Charging */
static int hl7132_stop_charging(struct hl7132_charger *hl7132)
{
	int ret = 0;

	/* mark the end with \n in logbuffer */
	logbuffer_prlog(hl7132, LOGLEVEL_DEBUG,
			"%s: hl7132->charging_state=%d ret=%d\n",
			__func__, hl7132->charging_state, ret);

	mutex_lock(&hl7132->lock);

	/* Check the current state */
	if (hl7132->charging_state == DC_STATE_NO_CHARGING)
		goto done;

	/* Stop Direct charging  */
	cancel_delayed_work(&hl7132->timer_work);
	cancel_delayed_work(&hl7132->pps_work);
	hl7132->timer_id = TIMER_ID_NONE;
	hl7132->timer_period = 0;

	/* Clear parameter */
	if (hl7132->charging_state != DC_STATE_ERROR)
		hl7132->charging_state = DC_STATE_NO_CHARGING;
	hl7132->ret_state = DC_STATE_NO_CHARGING;
	hl7132->prev_iin = 0;
	hl7132->prev_inc = INC_NONE;
	hl7132->chg_mode = CHG_NO_DC_MODE;

	/* restore to config */
	hl7132->pdata->iin_cfg = hl7132->pdata->iin_cfg_max;
	hl7132->pdata->vbat_reg = hl7132->pdata->vbat_reg_dt;

	/*
	 * Clear charging configuration
	 * TODO: use defaults when these are negative or zero at startup
	 * NOTE: cc_max is twice of IIN + headroom
	 */
	hl7132->cc_max = -1;
	hl7132->fv_uv = -1;

	/* Clear requests for new vbat_reg and new IIN */
	hl7132->new_vfloat = 0;
	hl7132->new_iin = 0;

	/* used to start DC and during errors */
	hl7132->retry_cnt = 0;

	hl7132->prev_ta_cur = 0;
	hl7132->prev_ta_vol = 0;
	/* close stats */
	hl7132_chg_stats_done(&hl7132->chg_data, hl7132);
	hl7132_chg_stats_dump(hl7132);

	/* TODO: something here to prep TA for the switch */

	/* HW integration guide section 5.5 - Disable charging */
	ret = hl7132_set_charging(hl7132, false);
	if (ret < 0) {
		dev_err(hl7132->dev, "%s: Error-set_charging(main)\n", __func__);
		goto error;
	}

	/* stop charging and recover TA voltage */
	if (hl7132->mains_online == true)
		hl7132_recover_ta(hl7132);

	power_supply_changed(hl7132->mains);

	/* HW integration guide sections 4.1, 5.5 - Soft reset and reinitialize */
	hl7132->hw_init_done = false;
	if (hl7132_hw_init(hl7132) == 0)
		hl7132->hw_init_done = true;

done:
error:
	mutex_unlock(&hl7132->lock);
	__pm_relax(hl7132->monitor_wake_lock);
	dev_dbg(hl7132->dev, "%s: END, ret=%d\n", __func__, ret);
	return ret;
}

#define FCC_TOLERANCE_RATIO		99
#define FCC_POWER_INCREASE_THRESHOLD	99

/*
 * Compensate TA current for the target input current called from
 * hl7132_charge_ccmode() when loop becomes not active.
 *
 * hl7132_charge_ccmode() ->
 *	-> hl7132_set_ta_voltage_comp()
 *	-> hl7132_set_ta_current_comp2()
 *
 * NOTE: call holding mutex_lock(&hl7132->lock);
 */
static int hl7132_set_ta_current_comp(struct hl7132_charger *hl7132)
{
	const int iin_high = hl7132->iin_cc + hl7132->pdata->iin_cc_comp_offset;
	const int iin_low = hl7132->iin_cc - hl7132->pdata->iin_cc_comp_offset;
	int rc, ibat, icn = -EINVAL, iin = -EINVAL;
	bool ovc_flag;

	/* IIN = IBAT+SYSLOAD */
	rc = hl7132_get_ibatt(hl7132, &ibat);
	if (rc == 0)
		rc = hl7132_get_iin(hl7132, &icn);
	if (rc == 0)
		iin = hl7132_read_adc(hl7132, ADCCH_IIN);

	ovc_flag = ibat > hl7132->cc_max;
	if (ovc_flag)
		hl7132_chg_stats_inc_ovcf(&hl7132->chg_data, ibat, hl7132->cc_max);

	logbuffer_prlog(hl7132, ovc_flag ? LOGLEVEL_WARNING : LOGLEVEL_DEBUG,
			"%s: iin=%d, iin_cc=[%d,%d,%d], icn=%d ibat=%d, cc_max=%d rc=%d prev_iin=%d ta_cur=%d, ta_vol=%d",
			__func__, iin, iin_low, hl7132->iin_cc, iin_high,
			icn, ibat, hl7132->cc_max, rc,
			hl7132->prev_iin,  hl7132->ta_cur, hl7132->ta_vol);

	if (iin < 0)
		return iin;

	/* Compare IIN ADC with target input current */
	if (iin > iin_high) {

		/* TA current is higher than the target input current */
		if (hl7132->ta_cur > hl7132->iin_cc) {
			/* TA current is over than IIN_CC */
			/* Decrease TA current (50mA) */
			hl7132->ta_cur = hl7132->ta_cur - PD_MSG_TA_CUR_STEP;
			logbuffer_prlog(hl7132, LOGLEVEL_DEBUG, "Cont1: ta_cur=%u",
					hl7132->ta_cur);

		/* TA current is already less than IIN_CC */
		/* Compara IIN_ADC with the previous IIN_ADC */
		} else if (iin < (hl7132->prev_iin - HL7132_IIN_ADC_OFFSET)) {
			/* Assume that TA operation mode is CV mode */
			/* Decrease TA voltage (20mV) */
			hl7132->ta_vol = hl7132->ta_vol - PD_MSG_TA_VOL_STEP;
			logbuffer_prlog(hl7132, LOGLEVEL_DEBUG, "Cont2-1: ta_vol=%u",
					hl7132->ta_vol);
		} else {
			/* Assume TA operation mode is CL mode */
			/* Decrease TA current (50mA) */
			hl7132->ta_cur = hl7132->ta_cur - PD_MSG_TA_CUR_STEP;
			logbuffer_prlog(hl7132, LOGLEVEL_DEBUG, "Cont2-2: ta_cur=%u",
					hl7132->ta_cur);
		}

		/* Send PD Message */
		hl7132->timer_id = TIMER_PDMSG_SEND;
		hl7132->timer_period = 0;

	} else if (iin < iin_low) {

		/* compare IIN ADC with previous IIN ADC + 20mA */
		if (iin > (hl7132->prev_iin + HL7132_IIN_ADC_OFFSET)) {
			/*
			 * TA voltage is not enough to supply the operating
			 * current of RDO: increase TA voltage
			 */

			/* Compare TA max voltage */
			if (hl7132->ta_vol == hl7132->ta_max_vol) {
				/* TA voltage is already the maximum voltage */
				/* Compare TA max current */
				if (!hl7132_can_inc_ta_cur(hl7132)) {
					/* TA voltage and current are at max */
					logbuffer_prlog(hl7132, LOGLEVEL_DEBUG,
							"End1: ta_vol=%u, ta_cur=%u",
							hl7132->ta_vol, hl7132->ta_cur);

					/* Set timer */
					hl7132->timer_id = TIMER_CHECK_CCMODE;
					hl7132->timer_period = HL7132_CCMODE_CHECK1_T;
				} else {
					/* Increase TA current (50mA) */
					hl7132->ta_cur = hl7132->ta_cur + PD_MSG_TA_CUR_STEP;

					logbuffer_prlog(hl7132, LOGLEVEL_DEBUG,
							"Cont3: ta_cur=%u",
							hl7132->ta_cur);

					/* Send PD Message */
					hl7132->timer_id = TIMER_PDMSG_SEND;
					hl7132->timer_period = 0;

					/* Set TA increment flag */
					hl7132->prev_inc = INC_TA_CUR;
				}
			} else {
				/* Increase TA voltage (20mV) */
				hl7132->ta_vol = hl7132->ta_vol + PD_MSG_TA_VOL_STEP;
				logbuffer_prlog(hl7132, LOGLEVEL_DEBUG,
						"Cont4: ta_vol=%u", hl7132->ta_vol);

				/* Send PD Message */
				hl7132->timer_id = TIMER_PDMSG_SEND;
				hl7132->timer_period = 0;

				/* Set TA increment flag */
				hl7132->prev_inc = INC_TA_VOL;
			}

		/* TA current is lower than the target input current */
		/* Check the previous TA increment */
		} else if (hl7132->prev_inc == INC_TA_VOL) {
			/*
			 * The previous increment is TA voltage, but
			 * input current does not increase.
			 */

			/* Try to increase TA current */
			/* Compare TA max current */
			if (!hl7132_can_inc_ta_cur(hl7132)) {

				/* TA current is already the maximum current */
				/* Compare TA max voltage */
				if (hl7132->ta_vol == hl7132->ta_max_vol) {
					/*
					 * TA voltage and current are already
					 * the maximum values
					 */
					logbuffer_prlog(hl7132, LOGLEVEL_DEBUG,
							"End2: ta_vol=%u, ta_cur=%u",
							hl7132->ta_vol, hl7132->ta_cur);

					hl7132->timer_id = TIMER_CHECK_CCMODE;
					hl7132->timer_period = HL7132_CCMODE_CHECK1_T;
				} else {
					/* Increase TA voltage (20mV) */
					hl7132->ta_vol = hl7132->ta_vol + PD_MSG_TA_VOL_STEP;
					logbuffer_prlog(hl7132, LOGLEVEL_DEBUG,
							"Cont5: ta_vol=%u",
							hl7132->ta_vol);

					/* Send PD Message */
					hl7132->timer_id = TIMER_PDMSG_SEND;
					hl7132->timer_period = 0;

					/* Set TA increment flag */
					hl7132->prev_inc = INC_TA_VOL;
				}
			} else {
				const unsigned int ta_cur = hl7132->ta_cur +
							    PD_MSG_TA_CUR_STEP;

				/* Increase TA current (50mA) */
				logbuffer_prlog(hl7132, LOGLEVEL_DEBUG,
						"Cont6: ta_cur=%u->%u",
						hl7132->ta_cur, ta_cur);

				hl7132->ta_cur = hl7132->ta_cur + PD_MSG_TA_CUR_STEP;
				hl7132->timer_id = TIMER_PDMSG_SEND;
				hl7132->timer_period = 0;

				hl7132->prev_inc = INC_TA_CUR;
			}

		/*
		 * The previous increment was TA current, but input current
		 * did not increase. Try to increase TA voltage.
		 */
		} else if (hl7132->ta_vol == hl7132->ta_max_vol) {
			/* TA voltage is already the maximum voltage */

			/* Compare TA maximum current */
			if (!hl7132_can_inc_ta_cur(hl7132)) {
				/*
				 * TA voltage and current are already at the
				 * maximum values
				 */
				logbuffer_prlog(hl7132, LOGLEVEL_DEBUG,
						"End3: ta_vol=%u, ta_cur=%u",
						 hl7132->ta_vol, hl7132->ta_cur);

				hl7132->timer_id = TIMER_CHECK_CCMODE;
				hl7132->timer_period = HL7132_CCMODE_CHECK1_T;
			} else {
				/* Increase TA current (50mA) */
				hl7132->ta_cur = hl7132->ta_cur + PD_MSG_TA_CUR_STEP;
				logbuffer_prlog(hl7132, LOGLEVEL_DEBUG,
						"Cont7: ta_cur=%u", hl7132->ta_cur);

				/* Send PD Message */
				hl7132->timer_id = TIMER_PDMSG_SEND;
				hl7132->timer_period = 0;

				/* Set TA increment flag */
				hl7132->prev_inc = INC_TA_CUR;
			}
		} else {
			/* Increase TA voltage (20mV) */
			logbuffer_prlog(hl7132, LOGLEVEL_DEBUG,
					"Comp. Cont8: ta_vol=%u->%u",
					hl7132->ta_vol, hl7132->ta_vol + PD_MSG_TA_VOL_STEP);

			hl7132->ta_vol += PD_MSG_TA_VOL_STEP;

			/* Send PD Message */
			hl7132->timer_id = TIMER_PDMSG_SEND;
			hl7132->timer_period = 0;

			/* Set TA increment flag */
			hl7132->prev_inc = INC_TA_VOL;
		}

	} else {
		/* IIN ADC is in valid range */
		/* IIN_CC - 50mA < IIN ADC < IIN_CC + 50mA  */
		logbuffer_prlog(hl7132, LOGLEVEL_DEBUG,
				"Comp. End4(valid): ta_vol=%u, ta_cur=%u",
				hl7132->ta_vol, hl7132->ta_cur);
		/* Set timer */
		hl7132->timer_id = TIMER_CHECK_CCMODE;
		hl7132->timer_period = HL7132_CCMODE_CHECK1_T;

		/* b/186969924: reset increment state on valid */
		hl7132->prev_inc = INC_NONE;
	}

	/* Save previous iin adc */
	hl7132->prev_iin = iin;
	return 0;
}

/*
 * max iin for 2:1 mode given cc_max and iin_cfg.
 * TODO: maybe use pdata->iin_cfg if cc_max is zero or negative.
 */
static int hl7132_get_iin_max(const struct hl7132_charger *hl7132, int cc_max)
{
	const int cc_limit = hl7132->pdata->iin_max_offset + cc_max / 2;
	int iin_max;

	iin_max = min_t(unsigned int, hl7132->pdata->iin_cfg_max, cc_limit);

	dev_dbg(hl7132->dev,
		"%s: iin_max=%d iin_cfg=%u iin_cfg_max=%d cc_max=%d cc_limit=%d\n",
		 __func__, iin_max, hl7132->pdata->iin_cfg,
		 hl7132->pdata->iin_cfg_max, cc_max, cc_limit);

	return iin_max;
}

/* Compensate TA current for constant power mode */
/* hold mutex_lock(&hl7132->lock), schedule on return 0 */
static int hl7132_set_ta_current_comp2(struct hl7132_charger *hl7132)
{
	int rc, ibat, icn = -EINVAL, iin = -EINVAL;
	bool ovc_flag;

	/* IIN = IBAT+SYSLOAD */
	rc = hl7132_get_ibatt(hl7132, &ibat);
	if (rc == 0)
		rc = hl7132_get_iin(hl7132, &icn);
	if (rc == 0)
		iin = hl7132_read_adc(hl7132, ADCCH_IIN);

	ovc_flag = ibat > hl7132->cc_max;
	if (ovc_flag)
		hl7132_chg_stats_inc_ovcf(&hl7132->chg_data, ibat, hl7132->cc_max);

	logbuffer_prlog(hl7132, ovc_flag ? LOGLEVEL_WARNING : LOGLEVEL_DEBUG,
			"%s: iin=%d, iin_cc=[%d,%d,%d], iin_cfg=%d icn=%d ibat=%d, cc_max=%d rc=%d",
			__func__, iin,
			hl7132->iin_cc - HL7132_IIN_CC_COMP_OFFSET_CP,
			hl7132->iin_cc,
			hl7132->iin_cc + HL7132_IIN_CC_COMP_OFFSET_CP,
			hl7132->pdata->iin_cfg,
			icn, ibat, hl7132->cc_max, rc);
	if (iin < 0)
		return iin;

	/* Compare IIN ADC with target input current */
	if (iin > (hl7132->pdata->iin_cfg + hl7132->pdata->iin_cc_comp_offset)) {
		/* TA current is higher than the target input current limit */
		hl7132->ta_cur = hl7132->ta_cur - PD_MSG_TA_CUR_STEP;

		hl7132->timer_id = TIMER_PDMSG_SEND;
		hl7132->timer_period = 0;
	} else if (iin < (hl7132->iin_cc - HL7132_IIN_CC_COMP_OFFSET_CP)) {

		/* TA current is lower than the target input current */
		/* IIN_ADC < IIN_CC -20mA */
		if (hl7132->ta_vol == hl7132->ta_max_vol) {
			const int iin_cc_lb = hl7132->iin_cc -
				      hl7132->pdata->iin_cc_comp_offset;

			/* Check IIN_ADC < IIN_CC - 50mA */
			if (iin < iin_cc_lb) {
				const unsigned int ta_max_vol =
				    hl7132->pdata->ta_max_vol * hl7132->chg_mode;
				unsigned int iin_apdo;
				unsigned int val;

				/* Set new IIN_CC to IIN_CC - 50mA */
				hl7132->iin_cc = hl7132->iin_cc -
					  hl7132->pdata->iin_cc_comp_offset;

				/* Set new TA_MAX_VOL to TA_MAX_PWR/IIN_CC */
				/* Adjust new IIN_CC with APDO resolution */
				iin_apdo = hl7132->iin_cc / PD_MSG_TA_CUR_STEP;
				iin_apdo = iin_apdo * PD_MSG_TA_CUR_STEP;
				/* in mV */
				val = hl7132->ta_max_pwr / (iin_apdo / hl7132->chg_mode / 1000);
				/* Adjust values with APDO resolution(20mV) */
				val = val * 1000 / PD_MSG_TA_VOL_STEP;
				val = val * PD_MSG_TA_VOL_STEP; /* uV */

				/* Set new TA_MAX_VOL */
				hl7132->ta_max_vol = min(val, ta_max_vol);

				/* Increase TA voltage(40mV) */
				hl7132->ta_vol = hl7132->ta_vol + PD_MSG_TA_VOL_STEP * 2;

				logbuffer_prlog(hl7132, LOGLEVEL_DEBUG,
						"Cont1: ta_vol=%u",
						hl7132->ta_vol);

				/* Send PD Message */
				hl7132->timer_id = TIMER_PDMSG_SEND;
				hl7132->timer_period = 0;
			} else {
				/* Wait for next current step compensation */
				/* IIN_CC - 50mA < IIN ADC < IIN_CC - 20mA */
				logbuffer_prlog(hl7132, LOGLEVEL_DEBUG,
						"Comp.(wait): ta_vol=%u",
						hl7132->ta_vol);

				/* Set timer */
				hl7132->timer_id = TIMER_CHECK_CCMODE;
				hl7132->timer_period = HL7132_CCMODE_CHECK2_T;
			}
		} else {
			/* Increase TA voltage(40mV) */
			hl7132->ta_vol = hl7132->ta_vol + PD_MSG_TA_VOL_STEP * 2;
			if (hl7132->ta_vol > hl7132->ta_max_vol)
				hl7132->ta_vol = hl7132->ta_max_vol;

			logbuffer_prlog(hl7132, LOGLEVEL_DEBUG, "Cont2: ta_vol=%u",
					hl7132->ta_vol);

			/* Send PD Message */
			hl7132->timer_id = TIMER_PDMSG_SEND;
			hl7132->timer_period = 0;
		}
	} else {
		/* IIN ADC is in valid range */
		/* IIN_CC - 50mA < IIN ADC < IIN_CFG + 50mA */
		dev_dbg(hl7132->dev, "End(valid): ta_vol=%u\n", hl7132->ta_vol);

		hl7132->timer_id = TIMER_CHECK_CCMODE;
		hl7132->timer_period = HL7132_CCMODE_CHECK2_T;

		/* b/186969924: reset increment state on valid */
		hl7132->prev_inc = INC_NONE;
	}

	/* Save previous iin adc */
	hl7132->prev_iin = iin;
	return 0;
}

/* Compensate TA voltage for the target input current */
/* hold mutex_lock(&hl7132->lock), schedule on return 0 */
static int hl7132_set_ta_voltage_comp(struct hl7132_charger *hl7132)
{
	const int iin_high = hl7132->iin_cc + hl7132->pdata->iin_cc_comp_offset;
	const int iin_low = hl7132->iin_cc - hl7132->pdata->iin_cc_comp_offset;
	const int ibat_limit = (hl7132->cc_max * FCC_POWER_INCREASE_THRESHOLD) / 100;
	int rc, ibat, icn = -EINVAL, iin = -EINVAL;
	bool ovc_flag;

	dev_dbg(hl7132->dev, "%s: ======START=======\n", __func__);
	dev_dbg(hl7132->dev, "%s: = charging_state=%u ==\n", __func__,
		 hl7132->charging_state);

	/* IIN = IBAT+SYSLOAD */
	rc = hl7132_get_ibatt(hl7132, &ibat);
	if (rc == 0)
		rc = hl7132_get_iin(hl7132, &icn);
	if (rc == 0)
		iin = hl7132_read_adc(hl7132, ADCCH_IIN);

	ovc_flag = ibat > hl7132->cc_max;
	if (ovc_flag)
		hl7132_chg_stats_inc_ovcf(&hl7132->chg_data, ibat, hl7132->cc_max);

	logbuffer_prlog(hl7132, ovc_flag ? LOGLEVEL_WARNING : LOGLEVEL_DEBUG,
			"%s: iin=%d, iin_cc=[%d,%d,%d], icn=%d ibat=%d, cc_max=%d rc=%d ta_cur=%d, ta_vol=%d",
			__func__, iin, iin_low, hl7132->iin_cc, iin_high,
			icn, ibat, hl7132->cc_max, rc, hl7132->ta_cur, hl7132->ta_vol);

	if (iin < 0)
		return iin;

	/* Compare IIN ADC with target input current */
	if (iin > iin_high) {
		/* TA current is higher than the target input current */
		/* Decrease TA voltage (20mV) */
		hl7132->ta_vol = hl7132->ta_vol - PD_MSG_TA_VOL_STEP;
		logbuffer_prlog(hl7132, LOGLEVEL_DEBUG, "Cont1: ta_vol=%u",
				hl7132->ta_vol);

		/* Send PD Message */
		hl7132->timer_id = TIMER_PDMSG_SEND;
		hl7132->timer_period = 0;

	} else if (iin < hl7132->iin_cc - hl7132->pdata->iin_cc_comp_offset) {

		/* TA current is lower than the target input current */
		/* Compare TA max voltage */
		if (hl7132->ta_vol == hl7132->ta_max_vol) {
			/* TA is already at maximum voltage */
			logbuffer_prlog(hl7132, LOGLEVEL_DEBUG, "End1(max TA vol): ta_vol=%u",
					hl7132->ta_vol);

			/* Set timer */
			/* Check the current charging state */
			if (hl7132->charging_state == DC_STATE_CC_MODE) {
				/* CC mode */
				hl7132->timer_id = TIMER_CHECK_CCMODE;
				hl7132->timer_period = HL7132_CCMODE_CHECK1_T;
			} else {
				/* CV mode */
				hl7132->timer_id = TIMER_CHECK_CVMODE;
				hl7132->timer_period = HL7132_CVMODE_CHECK_T;
			}
		} else {
			const unsigned int ta_vol = hl7132->ta_vol;

			/* Increase TA voltage (20mV) */
			hl7132->ta_vol = hl7132->ta_vol + PD_MSG_TA_VOL_STEP;
			logbuffer_prlog(hl7132, LOGLEVEL_DEBUG, "Cont2: ta_vol:%u->%u",
					ta_vol, hl7132->ta_vol);

			/* Send PD Message */
			hl7132->timer_id = TIMER_PDMSG_SEND;
			hl7132->timer_period = 0;
		}
	} else {
		/* IIN ADC is in valid range */
		/* IIN_CC - 50mA < IIN ADC < IIN_CC + 50mA  */
		logbuffer_prlog(hl7132, LOGLEVEL_DEBUG,
				"End(valid): ta_vol=%u low_ibat=%d\n",
				hl7132->ta_vol, ibat < ibat_limit);

		/* Check the current charging state */
		if (hl7132->charging_state == DC_STATE_CC_MODE) {
			hl7132->timer_id = TIMER_CHECK_CCMODE;
			hl7132->timer_period = HL7132_CCMODE_CHECK1_T;
		} else {
			hl7132->timer_id = TIMER_CHECK_CVMODE;
			hl7132->timer_period = HL7132_CVMODE_CHECK_T;
		}
	}

	return 0;
}

/*
 * iin limit for 2:1 for the adapter and chg_mode
 * Minimum between the configuration, cc_max (scaled with offset) and the
 * adapter capabilities.
 */
static int hl7132_get_iin_limit(const struct hl7132_charger *hl7132)
{
	int iin_cc;

	iin_cc = hl7132_get_iin_max(hl7132, hl7132->cc_max);
	if (hl7132->ta_max_cur * hl7132->chg_mode < iin_cc)
		iin_cc = hl7132->ta_max_cur * hl7132->chg_mode;

	dev_dbg(hl7132->dev, "%s: iin_cc=%d ta_max_cur=%u, chg_mode=%d\n",
		__func__, iin_cc, hl7132->ta_max_cur, hl7132->chg_mode);

	return iin_cc;
}

/* recalculate ->ta_vol and ->ta_cur looking at demand (cc_max) */
static int hl7132_set_wired_dc(struct hl7132_charger *hl7132, int vbat)
{
	const unsigned long ta_max_vol = hl7132->pdata->ta_max_vol * hl7132->chg_mode;
	unsigned long val;
	int iin_cc;

	hl7132->iin_cc = hl7132_get_iin_limit(hl7132);

	/* Calculate new TA max voltage, current */
	val = hl7132->iin_cc / PD_MSG_TA_CUR_STEP;
	iin_cc = val * PD_MSG_TA_CUR_STEP;

	val = hl7132->ta_max_pwr / (iin_cc / hl7132->chg_mode  / 1000); /* mV */

	/* Adjust values with APDO resolution(20mV) */
	val = val * 1000 / PD_MSG_TA_VOL_STEP;
	val = val * PD_MSG_TA_VOL_STEP; /* uV */
	hl7132->ta_max_vol = min(val, ta_max_vol);

	/* HW integration guide section 5.4.1 - request TA voltage
	 * MAX[8000mV * chg_mode, 2 * VBAT_ADC * chg_mode + 500 mV]
	 */
	hl7132->ta_vol = max(HL7132_TA_MIN_VOL_PRESET * hl7132->chg_mode,
			      2 * vbat * hl7132->chg_mode + hl7132->pdata->ta_vol_offset);

	/* PPS voltage resolution is 20mV */
	val = hl7132->ta_vol / PD_MSG_TA_VOL_STEP;
	hl7132->ta_vol = val * PD_MSG_TA_VOL_STEP;
	hl7132->ta_vol = min(hl7132->ta_vol, hl7132->ta_max_vol);
	/* Set TA current to IIN_CC */
	hl7132->ta_cur = iin_cc / hl7132->chg_mode;

	logbuffer_prlog(hl7132, LOGLEVEL_DEBUG,
			"%s: iin_cc=%d, ta_vol=%d ta_cur=%d ta_max_vol=%d",
			__func__, hl7132->iin_cc, hl7132->ta_vol, hl7132->ta_cur,
			hl7132->ta_max_vol);

	return 0;
}

/*
 * like hl7132_preset_dcmode() but will not query the TA.
 * Called from timer:
 * [hl7132_charge_ccmode | hl7132_charge_cvmode] ->
 *	hl7132_apply_new_iin() ->
 *		hl7132_adjust_ta_current() ->
 *			hl7132_reset_dcmode()
 *	hl7132_apply_new_vfloat() ->
 *		hl7132_reset_dcmode()
 *
 * NOTE: caller holds mutex_lock(&hl7132->lock);
 */
static int hl7132_reset_dcmode(struct hl7132_charger *hl7132)
{
	int ret = -EINVAL, vbat;

	dev_dbg(hl7132->dev, "%s: ======START=======\n", __func__);
	dev_dbg(hl7132->dev, "%s: = charging_state=%u ==\n", __func__,
		 hl7132->charging_state);

	if (hl7132->cc_max < 0) {
		dev_err(hl7132->dev, "%s: invalid cc_max=%d\n", __func__,
			hl7132->cc_max);
		goto error;
	}

	/*
	 * VBAT is over threshold but it might be "bouncy" due to transitory
	 * used to determine ta_vout.
	 */
	vbat = hl7132_read_adc(hl7132, ADCCH_VBAT);
	if (vbat < 0)
		return vbat;

	ret = hl7132_set_wired_dc(hl7132, vbat);

	/* Clear previous IIN ADC, TA increment flag */
	hl7132->prev_inc = INC_NONE;
	hl7132->prev_iin = 0;
error:
	dev_dbg(hl7132->dev, "%s: End, ret=%d\n", __func__, ret);
	return ret;
}

/*
 * The caller was triggered from hl7132_apply_new_iin(), return to the
 * calling CC or CV loop.
 * call holding mutex_unlock(&hl7132->lock);
 */
static void hl7132_return_to_loop(struct hl7132_charger *hl7132)
{
	switch (hl7132->ret_state) {
	case DC_STATE_CC_MODE:
		hl7132->timer_id = TIMER_CHECK_CCMODE;
		break;
	case DC_STATE_CV_MODE:
		hl7132->timer_id = TIMER_CHECK_CVMODE;
		break;
	default:
		dev_err(hl7132->dev, "%s: invalid ret_state=%u\n",
			__func__, hl7132->ret_state);
		return;
	}

	dev_info(hl7132->dev, "%s: charging_state=%u->%u\n", __func__,
		 hl7132->charging_state, hl7132->ret_state);

	hl7132->charging_state = hl7132->ret_state;
	hl7132->timer_period = 1000;
	hl7132->ret_state = 0;
	hl7132->new_iin = 0;
}

/*
 * Kicked from hl7132_apply_new_iin() when hl7132->new_iin!=0 and completed
 * off the timer.
 * NOTE: Will return to the calling loop in ->ret_state
 */
static int hl7132_adjust_ta_current(struct hl7132_charger *hl7132)
{
	const int ta_limit = hl7132->iin_cc / hl7132->chg_mode;
	int rc, ibat, icn = -EINVAL, iin = -EINVAL;
	bool ovc_flag;
	int ret = 0;

	rc = hl7132_check_error(hl7132);
	if (rc != 0)
		return rc;

	rc = hl7132_get_ibatt(hl7132, &ibat);
	if (rc == 0)
		rc = hl7132_get_iin(hl7132, &icn);
	if (rc == 0)
		iin = hl7132_read_adc(hl7132, ADCCH_IIN);

	ovc_flag = ibat > hl7132->cc_max;
	if (ovc_flag)
		hl7132_chg_stats_inc_ovcf(&hl7132->chg_data, ibat, hl7132->cc_max);

	logbuffer_prlog(hl7132, ovc_flag ? LOGLEVEL_WARNING : LOGLEVEL_DEBUG,
			"%s: iin=%d, iin_cc=%d ta_limit=%d, iin_cfg=%d icn=%d ibat=%d, cc_max=%d rc=%d",
			__func__, iin, hl7132->iin_cc, ta_limit, hl7132->pdata->iin_cfg,
			icn, ibat, hl7132->cc_max, rc);

	if (hl7132->charging_state != DC_STATE_ADJUST_TACUR)
		dev_info(hl7132->dev, "%s: charging_state=%u->%u\n", __func__,
			 hl7132->charging_state, DC_STATE_ADJUST_TACUR);

	hl7132->charging_state = DC_STATE_ADJUST_TACUR;

	if (hl7132->ta_cur == ta_limit) {

		logbuffer_prlog(hl7132, LOGLEVEL_DEBUG,
				"adj. End, ta_cur=%u, ta_vol=%u, iin_cc=%u, chg_mode=%u",
				hl7132->ta_cur, hl7132->ta_vol,
				hl7132->iin_cc, hl7132->chg_mode);

		/* "Recover" IIN_CC to the original value (new_iin) */
		hl7132->iin_cc = hl7132->new_iin;
		hl7132_return_to_loop(hl7132);

	} else if (hl7132->iin_cc > hl7132->pdata->iin_cfg) {
		const int old_iin_cfg = hl7132->pdata->iin_cfg;

		/* Raise iin_cfg to the new iin_cc value (why????) */
		hl7132->pdata->iin_cfg = hl7132->iin_cc;

		ret = hl7132_set_input_current(hl7132, hl7132->iin_cc);
		if (ret == 0)
			ret = hl7132_reset_dcmode(hl7132);
		if (ret < 0)
			goto error;

		logbuffer_prlog(hl7132, LOGLEVEL_DEBUG,
				"New IIN, ta_max_vol=%u, ta_max_cur=%u, ta_max_pwr=%lu, iin_cc=%u, iin_cfg=%d->%d chg_mode=%u",
				hl7132->ta_max_vol, hl7132->ta_max_cur,
				hl7132->ta_max_pwr, hl7132->iin_cc,
				old_iin_cfg, hl7132->iin_cc,
				hl7132->chg_mode);

		hl7132->new_iin = 0;

		dev_info(hl7132->dev, "%s: charging_state=%u->%u\n", __func__,
			 hl7132->charging_state, DC_STATE_ADJUST_CC);

		/* Send PD Message and go to Adjust CC mode */
		hl7132->charging_state = DC_STATE_ADJUST_CC;
		hl7132->timer_id = TIMER_PDMSG_SEND;
		hl7132->timer_period = 0;
	} else {
		unsigned int val;

		/*
		 * Adjust IIN_CC with APDO resolution(50mA)
		 * hl7132->iin_cc will be reset to hl7132->new_iin when
		 * ->ta_cur reaches the ta_limit at the beginning of the
		 * function
		 */
		val = hl7132->iin_cc / PD_MSG_TA_CUR_STEP;
		hl7132->iin_cc = val * PD_MSG_TA_CUR_STEP;
		hl7132->ta_cur = hl7132->iin_cc / hl7132->chg_mode;

		logbuffer_prlog(hl7132, LOGLEVEL_DEBUG, "adjust iin=%u ta_cur=%d chg_mode=%d",
				hl7132->iin_cc, hl7132->ta_cur, hl7132->chg_mode);

		/* Send PD Message */
		hl7132->timer_id = TIMER_PDMSG_SEND;
		hl7132->timer_period = 0;
	}

	/* reschedule on ret == 0 */
error:
	return ret;
}

/* Kicked from apply_new_iin() then run off the timer
 * call holding mutex_lock(&hl7132->lock);
 */
static int hl7132_adjust_ta_voltage(struct hl7132_charger *hl7132)
{
	int rc, ibat, icn = -EINVAL, iin = -EINVAL;
	bool ovc_flag;

	if (hl7132->charging_state != DC_STATE_ADJUST_TAVOL)
		dev_info(hl7132->dev, "%s: charging_state=%u->%u\n", __func__,
			 hl7132->charging_state, DC_STATE_ADJUST_TAVOL);

	hl7132->charging_state = DC_STATE_ADJUST_TAVOL;

	rc = hl7132_check_error(hl7132);
	if (rc != 0)
		return rc;

	rc = hl7132_get_ibatt(hl7132, &ibat);
	if (rc == 0)
		rc = hl7132_get_iin(hl7132, &icn);
	if (rc == 0)
		iin = hl7132_read_adc(hl7132, ADCCH_IIN);

	ovc_flag = ibat > hl7132->cc_max;
	if (ovc_flag)
		hl7132_chg_stats_inc_ovcf(&hl7132->chg_data, ibat, hl7132->cc_max);

	logbuffer_prlog(hl7132, ovc_flag ? LOGLEVEL_WARNING : LOGLEVEL_DEBUG,
			"%s: iin=%d, iin_cc=[%d,%d,%d], icn=%d ibat=%d, cc_max=%d rc=%d",
			__func__, iin, hl7132->iin_cc - PD_MSG_TA_CUR_STEP,
			hl7132->iin_cc, hl7132->iin_cc + PD_MSG_TA_CUR_STEP,
			icn, ibat, hl7132->cc_max, rc);

	if (iin < 0)
		return iin;


	/* Compare IIN ADC with targer input current */
	if (iin > (hl7132->iin_cc + PD_MSG_TA_CUR_STEP)) {
		/* TA current is higher than the target input current */
		/* Decrease TA voltage (20mV) */
		hl7132->ta_vol = hl7132->ta_vol - PD_MSG_TA_VOL_STEP;

		logbuffer_prlog(hl7132, LOGLEVEL_DEBUG, "Cont1, ta_vol=%u",
				hl7132->ta_vol);

		/* Send PD Message */
		hl7132->timer_id = TIMER_PDMSG_SEND;
		hl7132->timer_period = 0;
	} else if (iin < (hl7132->iin_cc - PD_MSG_TA_CUR_STEP)) {
		/* TA current is lower than the target input current */

		if (hl7132_check_status(hl7132) == STS_MODE_VFLT_LOOP) {
			/* IIN current may not able to increase in CV */

			logbuffer_prlog(hl7132, LOGLEVEL_DEBUG,
					"End1-1, skip adjust for cv, ta_cur=%u, ta_vol=%u, iin_cc=%u, chg_mode=%u",
					hl7132->ta_cur, hl7132->ta_vol,
					hl7132->iin_cc, hl7132->chg_mode);

			hl7132_return_to_loop(hl7132);
		} else if (hl7132->ta_vol == hl7132->ta_max_vol) {
			/* TA TA voltage is already at the maximum voltage */

			logbuffer_prlog(hl7132, LOGLEVEL_DEBUG,
					"End1, ta_cur=%u, ta_vol=%u, iin_cc=%u, chg_mode=%u",
					hl7132->ta_cur, hl7132->ta_vol,
					hl7132->iin_cc, hl7132->chg_mode);

			hl7132_return_to_loop(hl7132);
		} else {
			/* Increase TA voltage (20mV) */
			hl7132->ta_vol = hl7132->ta_vol + PD_MSG_TA_VOL_STEP;

			logbuffer_prlog(hl7132, LOGLEVEL_DEBUG, "Cont2, ta_vol=%u",
					hl7132->ta_vol);

			/* Send PD Message */
			hl7132->timer_id = TIMER_PDMSG_SEND;
			hl7132->timer_period = 0;
		}
	} else {
		/* IIN ADC is in valid range */
		/* IIN_CC - 50mA < IIN ADC < IIN_CC + 50mA  */

		logbuffer_prlog(hl7132, LOGLEVEL_DEBUG,
				"End2, ta_cur=%u, ta_vol=%u, iin_cc=%u, chg_mode=%u",
				hl7132->ta_cur, hl7132->ta_vol,
				hl7132->iin_cc, hl7132->chg_mode);

		hl7132_return_to_loop(hl7132);
	}

	return 0;
}

/*
 * Called from CC and CV loops to set a new IIN (i.e. a new cc_max charging
 * current). Should also change the iin_cfg to avoid overcurrents.
 * NOTE: caller must hold mutex_lock(&hl7132->lock)
 */
static int hl7132_apply_new_iin(struct hl7132_charger *hl7132)
{
	int ret;

	logbuffer_prlog(hl7132, LOGLEVEL_INFO,
			"new_iin=%d (cc_max=%d), ta_type=%d charging_state=%d",
			hl7132->new_iin, hl7132->cc_max,
			hl7132->ta_type, hl7132->charging_state);

	/* iin_cfg is adjusted UP in hl7132_set_input_current() */
	ret = hl7132_set_input_current(hl7132, hl7132->new_iin);
	if (ret < 0)
		return ret;
	hl7132->pdata->iin_cfg = hl7132->new_iin;

	/*
	 * ->ret_state is used to go back to the loop (CC or CV) that called
	 * this function.
	 */
	hl7132->ret_state = hl7132->charging_state;

	/*
	 * new_iin is used to trigger the process which might span one or more
	 * timer ticks the new_iin . The flag will be cleared once the target
	 * is reached.
	 */
	hl7132->iin_cc = hl7132->new_iin;
	if (hl7132->iin_cc < (HL7132_TA_MIN_CUR * hl7132->chg_mode)) {
		/* TA current = HL7132_TA_MIN_CUR(1.0A) */
		hl7132->ta_cur = HL7132_TA_MIN_CUR;
		ret = hl7132_adjust_ta_voltage(hl7132);
	} else {
		ret = hl7132_adjust_ta_current(hl7132);
	}

	/* need reschedule on ret != 0 */

	dev_dbg(hl7132->dev, "%s: ret=%d\n", __func__, ret);
	return ret;
}

/*
 * also called from hl7132_set_new_cc_max()
 * call holding mutex_unlock(&hl7132->lock);
 */
static int hl7132_set_new_iin(struct hl7132_charger *hl7132, int iin)
{
	int ret = 0;

	if (iin < 0) {
		dev_dbg(hl7132->dev, "%s: ignore negative iin=%d\n", __func__,
			iin);
		return 0;
	}

	/* same as previous request nevermind */
	if (iin == hl7132->new_iin)
		return 0;

	dev_dbg(hl7132->dev, "%s: new_iin=%d->%d state=%d\n", __func__,
		 hl7132->new_iin, iin, hl7132->charging_state);

	/* apply iin_cc in hl7132_preset_config() at start */
	if (hl7132->charging_state == DC_STATE_NO_CHARGING ||
	    hl7132->charging_state == DC_STATE_CHECK_VBAT) {

		/* used on start vs the ->iin_cfg one */
		hl7132->pdata->iin_cfg = iin;
		hl7132->iin_cc = iin;
	} else if (hl7132->ret_state == 0) {
		/*
		 * hl7132_apply_new_iin() has not picked out the value yet
		 * and the value can be changed safely.
		 */
		hl7132->new_iin = iin;

		/* might want to tickle the loop now */
	} else {
		/* the caller must retry */
		ret = -EAGAIN;
	}

	dev_dbg(hl7132->dev, "%s: ret=%d\n", __func__, ret);
	return ret;
}

/*
 * The is no CC loop in this part: current must be controlled on TA side
 * adjusting output power. cc_max (the charging current) is scaled to iin
 *
 */
static int hl7132_set_new_cc_max(struct hl7132_charger *hl7132, int cc_max)
{
	const int prev_cc_max = hl7132->cc_max;
	int iin_max, ret = 0;

	if (cc_max < 0) {
		dev_dbg(hl7132->dev, "%s: ignore negative cc_max=%d\n", __func__,
			cc_max);
		return 0;
	}

	mutex_lock(&hl7132->lock);

	/* same as previous request nevermind */
	if (cc_max == hl7132->cc_max)
		goto done;

	/* iin will be capped by the adapter capabilities in reset_dcmode() */
	iin_max = hl7132_get_iin_max(hl7132, cc_max);
	if (iin_max <= 0) {
		dev_dbg(hl7132->dev, "%s: ignore negative iin_max=%d\n",
			__func__, iin_max);
		goto done;
	}

	if (hl7132->ta_max_cur && hl7132->ta_max_cur < iin_max)
		iin_max = hl7132->ta_max_cur;

	ret = hl7132_set_new_iin(hl7132, iin_max);
	if (ret == 0)
		hl7132->cc_max = cc_max;

	logbuffer_prlog(hl7132, LOGLEVEL_INFO,
			"%s: charging_state=%d cc_max=%d->%d iin_max=%d, ret=%d",
			__func__, hl7132->charging_state, prev_cc_max,
			cc_max, iin_max, ret);

done:
	dev_dbg(hl7132->dev, "%s: ret=%d\n", __func__, ret);
	mutex_unlock(&hl7132->lock);
	return ret;
}

/*
 * Apply hl7132->new_vfloat to the charging voltage.
 * Called from CC and CV loops, needs mutex_lock(&hl7132->lock)
 */
static int hl7132_apply_new_vfloat(struct hl7132_charger *hl7132)
{
	int ret = 0;

	if (hl7132->fv_uv == hl7132->new_vfloat)
		goto error_done;

	/* actually change the hardware */
	ret = hl7132_set_vbat_reg(hl7132, hl7132->new_vfloat);
	if (ret < 0)
		goto error_done;

	/* Restart the process if tier switch happened (either direction) */
	if (hl7132->charging_state == DC_STATE_CV_MODE &&
	    abs(hl7132->new_vfloat - hl7132->fv_uv) > HL7132_TIER_SWITCH_DELTA) {
		ret = hl7132_reset_dcmode(hl7132);
		if (ret < 0) {
			dev_err(hl7132->dev, "%s: cannot reset dcmode (%d)\n",
				__func__, ret);
		} else {
			dev_info(hl7132->dev, "%s: charging_state=%u->%u\n",
				__func__, hl7132->charging_state,
				DC_STATE_ADJUST_CC);

			hl7132->charging_state = DC_STATE_ADJUST_CC;
			hl7132->timer_id = TIMER_PDMSG_SEND;
			hl7132->timer_period = 0;
		}
	}

	hl7132->fv_uv = hl7132->new_vfloat;

error_done:
	logbuffer_prlog(hl7132, LOGLEVEL_INFO,
			"%s: new_vfloat=%d, ret=%d", __func__,
			hl7132->new_vfloat, ret);

	if (ret == 0)
		hl7132->new_vfloat = 0;

	return ret;
}

static int hl7132_set_new_vfloat(struct hl7132_charger *hl7132, int vfloat)
{
	int ret = 0;

	if (vfloat < 0) {
		dev_dbg(hl7132->dev, "%s: ignore negative vfloat %d\n",
			__func__, vfloat);
		return 0;
	}

	mutex_lock(&hl7132->lock);
	if (hl7132->new_vfloat == vfloat)
		goto done;

	/* use fv_uv at start in hl7132_preset_config() */
	if (hl7132->charging_state == DC_STATE_NO_CHARGING ||
	    hl7132->charging_state == DC_STATE_CHECK_VBAT) {
		hl7132->fv_uv = vfloat;
	} else {
		/* applied in hl7132_apply_new_vfloat() from CC or in CV loop */
		hl7132->new_vfloat = vfloat;
		dev_dbg(hl7132->dev, "%s: new_vfloat=%d\n", __func__,
			hl7132->new_vfloat);

		/* might want to tickle the cycle */
	}

done:
	mutex_unlock(&hl7132->lock);
	return ret;
}

/* called on loop inactive */
static void hl7132_adjust_ccmode_wired(struct hl7132_charger *hl7132, int iin)
{

	/* USBPD TA is connected */
	if (iin > (hl7132->iin_cc - HL7132_IIN_ADC_OFFSET)) {
		/* IIN_ADC > IIN_CC -20mA ? */
		/* Input current is already over IIN_CC */
		/* End TA voltage and current adjustment */

		dev_info(hl7132->dev, "%s: charging_state=%u->%u\n", __func__,
			 hl7132->charging_state, DC_STATE_CC_MODE);

		/* change charging state to CC mode */
		hl7132->charging_state = DC_STATE_CC_MODE;

		logbuffer_prlog(hl7132, LOGLEVEL_DEBUG,
				"End1: IIN_ADC=%d, ta_vol=%u, ta_cur=%u",
				iin, hl7132->ta_vol, hl7132->ta_cur);

		/* Clear TA increment flag */
		hl7132->prev_inc = INC_NONE;
		/* Go to CC mode */
		hl7132->timer_id = TIMER_CHECK_CCMODE;
		hl7132->timer_period = 0;

	/* Check TA voltage */
	} else if (hl7132->ta_vol == hl7132->ta_max_vol) {
		/* TA voltage is already max value */
		logbuffer_prlog(hl7132, LOGLEVEL_DEBUG,
				"End2: MAX value, ta_vol=%u, ta_cur=%u",
				hl7132->ta_vol, hl7132->ta_cur);

		/* Clear TA increment flag */
		hl7132->prev_inc = INC_NONE;
		/* Go to CC mode */
		hl7132->timer_id = TIMER_CHECK_CCMODE;
		hl7132->timer_period = 0;

		/* Check TA tolerance
		 * The current input current compares the final input
		 * current(IIN_CC) with 100mA offset PPS current tolerance
		 * has +/-150mA, so offset defined 100mA(tolerance +50mA)
		 */
	} else if (iin < (hl7132->iin_cc - HL7132_TA_IIN_OFFSET)) {
		/*
		 * TA voltage too low to enter TA CC mode, so we
		 * should increase TA voltage
		 */
		hl7132->ta_vol = hl7132->ta_vol + HL7132_TA_VOL_STEP_ADJ_CC *
					hl7132->chg_mode;

		if (hl7132->ta_vol > hl7132->ta_max_vol)
			hl7132->ta_vol = hl7132->ta_max_vol;

		logbuffer_prlog(hl7132, LOGLEVEL_DEBUG, "Cont1: ta_vol=%u",
				hl7132->ta_vol);

		/* Set TA increment flag */
		hl7132->prev_inc = INC_TA_VOL;
		/* Send PD Message */
		hl7132->timer_id = TIMER_PDMSG_SEND;
		hl7132->timer_period = 0;

	/* compare IIN ADC with previous IIN ADC + 20mA */
	} else if (iin > (hl7132->prev_iin + HL7132_IIN_ADC_OFFSET)) {
		/* TA can supply more current if TA voltage is high */
		/* TA voltage too low for TA CC mode: increase it */
		hl7132->ta_vol = hl7132->ta_vol +
					HL7132_TA_VOL_STEP_ADJ_CC *
					hl7132->chg_mode;
		if (hl7132->ta_vol > hl7132->ta_max_vol)
			hl7132->ta_vol = hl7132->ta_max_vol;

		logbuffer_prlog(hl7132, LOGLEVEL_DEBUG, "Cont2: ta_vol=%u",
				hl7132->ta_vol);
		/* Set TA increment flag */
		hl7132->prev_inc = INC_TA_VOL;

		/* Send PD Message */
		hl7132->timer_id = TIMER_PDMSG_SEND;
		hl7132->timer_period = 0;

	/* Check the previous increment */
	} else if (hl7132->prev_inc == INC_TA_CUR) {
		/*
		 * The previous increment is TA current, but input
		 * current does not increase. Try with voltage.
		 */

		hl7132->ta_vol = hl7132->ta_vol +
					HL7132_TA_VOL_STEP_ADJ_CC *
					hl7132->chg_mode;
		if (hl7132->ta_vol > hl7132->ta_max_vol)
			hl7132->ta_vol = hl7132->ta_max_vol;

		logbuffer_prlog(hl7132, LOGLEVEL_DEBUG, "Cont3: ta_vol=%u",
				hl7132->ta_vol);

		hl7132->prev_inc = INC_TA_VOL;
		hl7132->timer_id = TIMER_PDMSG_SEND;
		hl7132->timer_period = 0;

		/*
		 * The previous increment is TA voltage, but input
		 * current does not increase
		 */

		/* Try to increase TA current */
		/* Check APDO max current */
	} else if (!hl7132_can_inc_ta_cur(hl7132)) {
		/* TA current is maximum current */

		logbuffer_prlog(hl7132, LOGLEVEL_DEBUG,
				"End(MAX_CUR): IIN_ADC=%d, ta_vol=%u, ta_cur=%u",
				iin, hl7132->ta_vol, hl7132->ta_cur);

		hl7132->prev_inc = INC_NONE;

		/* Go to CC mode */
		hl7132->timer_id = TIMER_CHECK_CCMODE;
		hl7132->timer_period = 0;
	} else {
		/* TA has tolerance and compensate it as real current */
		/* Increase TA current(50mA) */
		hl7132->ta_cur = hl7132->ta_cur + PD_MSG_TA_CUR_STEP;
		if (hl7132->ta_cur > hl7132->ta_max_cur)
			hl7132->ta_cur = hl7132->ta_max_cur;

		logbuffer_prlog(hl7132, LOGLEVEL_DEBUG, "Cont4: ta_cur=%u",
				hl7132->ta_cur);

		hl7132->prev_inc = INC_TA_CUR;
		hl7132->timer_id = TIMER_PDMSG_SEND;
		hl7132->timer_period = 0;
	}
}

static int hl7132_vote_dc_avail(struct hl7132_charger *hl7132, int vote, int enable)
{
	int ret = 0;

	if (!hl7132->dc_avail)
		hl7132->dc_avail = gvotable_election_get_handle(VOTABLE_DC_CHG_AVAIL);

	if (hl7132->dc_avail) {
		ret = gvotable_cast_int_vote(hl7132->dc_avail, REASON_DC_DRV, vote, enable);
		if (ret < 0)
			dev_err(hl7132->dev,
				"Unable to cast vote for DC Chg avail (%d)\n",
				ret);
	}

	logbuffer_prlog(hl7132, hl7132->charging_state == DC_STATE_ERROR ?
			LOGLEVEL_INFO : LOGLEVEL_DEBUG,
			"%s: Voting dc_avail when in error state", __func__);

	return ret;
}

/* 2:1 Direct Charging Adjust CC MODE control
 * called at the beginnig of CC mode charging. Will be followed by
 * hl7132_charge_ccmode with which shares some of the adjustments.
 */
static int hl7132_charge_adjust_ccmode(struct hl7132_charger *hl7132)
{
	int iin, ccmode, vbatt, vin_vol;

	int ret = 0;

	mutex_lock(&hl7132->lock);

	dev_dbg(hl7132->dev, "%s: ======START=======\n", __func__);
	hl7132_prlog_state(hl7132, __func__);

	if (hl7132->charging_state != DC_STATE_ADJUST_CC)
		dev_info(hl7132->dev, "%s: charging_state=%u->%u\n", __func__,
			 hl7132->charging_state, DC_STATE_ADJUST_CC);

	hl7132->charging_state = DC_STATE_ADJUST_CC;

	ret = hl7132_check_error(hl7132);
	if (ret != 0)
		goto error; // This is not active mode.

	ccmode = hl7132_check_status(hl7132);
	if (ccmode < 0) {
		ret = ccmode;
		goto error;
	}

	switch (ccmode) {
	case STS_MODE_IIN_LOOP:
		hl7132->chg_data.iin_loop_count++;
		fallthrough;
	case STS_MODE_CHG_LOOP:	/* CHG_LOOP does't exist */
		if (hl7132->ta_cur > HL7132_TA_MIN_CUR) {
			/* TA current is higher than 1.0A */
			/* Decrease TA current (50mA) */
			hl7132->ta_cur = hl7132->ta_cur - PD_MSG_TA_CUR_STEP;

			logbuffer_prlog(hl7132, LOGLEVEL_DEBUG, "End2: ta_cur=%u, ta_vol=%u",
					hl7132->ta_cur, hl7132->ta_vol);
		} else {
			/* Decrease TA voltage (20mV) */
			hl7132->ta_vol = hl7132->ta_vol - PD_MSG_TA_VOL_STEP;

			logbuffer_prlog(hl7132, LOGLEVEL_DEBUG, "End3: ta_cur=%u, ta_vol=%u",
					hl7132->ta_cur, hl7132->ta_vol);
		}

		hl7132->prev_inc = INC_NONE;

		dev_info(hl7132->dev, "%s: charging_state=%u->%u\n", __func__,
			 hl7132->charging_state, DC_STATE_CC_MODE);

		/* Send PD Message and then go to CC mode */
		hl7132->charging_state = DC_STATE_CC_MODE;
		hl7132->timer_id = TIMER_PDMSG_SEND;
		hl7132->timer_period = 0;
		break;

	case STS_MODE_VFLT_LOOP:
		vbatt = hl7132_read_adc(hl7132, ADCCH_VBAT);

		logbuffer_prlog(hl7132, LOGLEVEL_DEBUG, "End4: vbatt=%d, ta_vol=%u",
				vbatt, hl7132->ta_vol);

		/* Clear TA increment flag */
		hl7132->prev_inc = INC_NONE;
		/* Go to Pre-CV mode */
		hl7132->timer_id = TIMER_ENTER_CVMODE;
		hl7132->timer_period = 0;
		break;

	case STS_MODE_LOOP_INACTIVE:
		iin = hl7132_read_adc(hl7132, ADCCH_IIN);
		logbuffer_prlog(hl7132, LOGLEVEL_DEBUG,
				"Inactive: iin=%d, iin_cc=%d, cc_max=%d",
				iin, hl7132->iin_cc, hl7132->cc_max);
		if (iin < 0)
			break;

		hl7132_adjust_ccmode_wired(hl7132, iin);
		hl7132->prev_iin = iin;

		break;

	case STS_MODE_VIN_UVLO:
		/* VIN UVLO - just notification , it works by hardware */
		vin_vol = hl7132_read_adc(hl7132, ADCCH_VIN);

		logbuffer_prlog(hl7132, LOGLEVEL_DEBUG, "VIN_UVLO: ta_vol=%u, vin_vol=%d",
				hl7132->ta_cur, vin_vol);

		/* Check VIN after 1sec */
		hl7132->timer_id = TIMER_ADJUST_CCMODE;
		hl7132->timer_period = 1000;
		break;

	default:
		goto error;
	}

	mod_delayed_work(hl7132->dc_wq, &hl7132->timer_work,
			 msecs_to_jiffies(hl7132->timer_period));
error:
	mutex_unlock(&hl7132->lock);
	dev_dbg(hl7132->dev, "%s: End, ret=%d, ccmode=%d\n", __func__, ret,
		ccmode);
	return ret;
}

/* <0 error, 0 no new limits, >0 new limits */
static int hl7132_apply_new_limits(struct hl7132_charger *hl7132)
{
	int ret = 0;

	if (hl7132->new_iin && hl7132->new_iin < hl7132->iin_cc) {
		ret = hl7132_apply_new_iin(hl7132);
		if (ret == 0)
			ret = 1;
	} else if (hl7132->new_vfloat) {
		ret = hl7132_apply_new_vfloat(hl7132);
		if (ret == 0)
			ret = 1;
	} else if (hl7132->new_iin) {
		ret = hl7132_apply_new_iin(hl7132);
		if (ret == 0)
			ret = 1;
	} else {
		return 0;
	}

	return ret;
}

/* 2:1 Direct Charging CC MODE control */
static int hl7132_charge_ccmode(struct hl7132_charger *hl7132)
{
	int ccmode = -1, vin_vol, iin, ret = 0;

	dev_dbg(hl7132->dev, "%s: ======START=======\n", __func__);

	mutex_lock(&hl7132->lock);

	if (hl7132->charging_state != DC_STATE_CC_MODE)
		dev_info(hl7132->dev, "%s: charging_state=%u->%u\n", __func__,
			 hl7132->charging_state, DC_STATE_CC_MODE);

	hl7132->charging_state = DC_STATE_CC_MODE;

	hl7132_prlog_state(hl7132, __func__);

	ret = hl7132_check_error(hl7132);
	if (ret != 0)
		goto error_exit;

	/*
	 * A change in VFLOAT here means that we have busted the tier, a
	 * change in iin means that the thermal engine had changed cc_max.
	 * hl7132_apply_new_limits() changes hl7132->charging_state to
	 * DC_STATE_ADJUST_TAVOL or DC_STATE_ADJUST_TACUR when new limits
	 * need to be applied.
	 */
	ret = hl7132_apply_new_limits(hl7132);
	if (ret < 0)
		goto error_exit;
	if (ret > 0)
		goto done;

	ccmode = hl7132_check_status(hl7132);
	if (ccmode < 0) {
		ret = ccmode;
		goto error_exit;
	}

	switch (ccmode) {
	case STS_MODE_LOOP_INACTIVE:
	{
		/* Set input current compensation */
		const int ta_max_vol = hl7132->ta_max_vol;

		/* Check TA current with TA_MIN_CUR */
		if (hl7132->ta_cur <= HL7132_TA_MIN_CUR) {
			hl7132->ta_cur = HL7132_TA_MIN_CUR;

			ret = hl7132_set_ta_voltage_comp(hl7132);
		} else if (ta_max_vol >= hl7132->pdata->ta_max_vol_cp) {
			ret = hl7132_set_ta_current_comp(hl7132);
		} else {
			/* constant power mode */
			ret = hl7132_set_ta_current_comp2(hl7132);
		}

		logbuffer_prlog(hl7132, LOGLEVEL_DEBUG,
				"INACTIVE2: ta_cur=%u, ta_vol=%u",
				hl7132->ta_cur,
				hl7132->ta_vol);
		break;
	}
	case STS_MODE_VFLT_LOOP:
		/* TODO: adjust fv_uv here based on real vbatt */

		iin = hl7132_read_adc(hl7132, ADCCH_IIN);
		logbuffer_prlog(hl7132, LOGLEVEL_DEBUG, "CC VFLOAT: iin=%d", iin);

		/* go to Pre-CV mode */
		hl7132->timer_id = TIMER_ENTER_CVMODE;
		hl7132->timer_period = 0;
		break;

	case STS_MODE_IIN_LOOP:
		hl7132->chg_data.iin_loop_count++;
		fallthrough;
	case STS_MODE_CHG_LOOP:
		iin = hl7132_read_adc(hl7132, ADCCH_IIN);
		if (iin < 0)
			break;

		if (hl7132->ta_cur <= HL7132_TA_MIN_CUR) {
			/* Decrease TA voltage (20mV) */
			hl7132->ta_vol = hl7132->ta_vol - PD_MSG_TA_VOL_STEP;
			logbuffer_prlog(hl7132, LOGLEVEL_DEBUG,
					"IIN_LOOP2: iin=%d, next_ta_vol=%u",
					iin, hl7132->ta_vol);
		} else {
			/* Decrease TA current (50mA) */
			hl7132->ta_cur = hl7132->ta_cur - PD_MSG_TA_CUR_STEP;
			logbuffer_prlog(hl7132, LOGLEVEL_DEBUG,
					"IIN_LOOP3: iin=%d, next_ta_cur=%u",
					iin, hl7132->ta_cur);
		}

		/* Send PD Message */
		hl7132->timer_id = TIMER_PDMSG_SEND;
		hl7132->timer_period = 0;
		break;

	case STS_MODE_VIN_UVLO:
		/* VIN UVLO - just notification, it works by hardware */
		vin_vol = hl7132_read_adc(hl7132, ADCCH_VIN);

		logbuffer_prlog(hl7132, LOGLEVEL_DEBUG,
				"VIN_UVLO: ta_cur=%u ta_vol=%u, vin_vol=%d",
				hl7132->ta_cur, hl7132->ta_vol, vin_vol);

		/* Check VIN after 1sec */
		hl7132->timer_id = TIMER_CHECK_CCMODE;
		hl7132->timer_period = 1000;
		break;

	default:
		break;
	}

done:
	mod_delayed_work(hl7132->dc_wq, &hl7132->timer_work,
			 msecs_to_jiffies(hl7132->timer_period));

error_exit:
	mutex_unlock(&hl7132->lock);
	dev_dbg(hl7132->dev,
		"%s: End, ccmode=%d timer_id=%d, timer_period=%lu ret=%d\n",
		 __func__, ccmode, hl7132->timer_id, hl7132->timer_period,
		 ret);
	return ret;
}


/* 2:1 Direct Charging Start CV MODE control - Pre CV MODE */
static int hl7132_charge_start_cvmode(struct hl7132_charger *hl7132)
{
	int ret = 0;
	int cvmode;
	int vin_vol;

	dev_dbg(hl7132->dev, "%s: ======START=======\n", __func__);

	mutex_lock(&hl7132->lock);

	if (hl7132->charging_state != DC_STATE_START_CV)
		dev_info(hl7132->dev, "%s: charging_state=%u->%u\n", __func__,
			 hl7132->charging_state, DC_STATE_START_CV);

	hl7132->charging_state = DC_STATE_START_CV;

	/* Check the charging type */
	ret = hl7132_check_error(hl7132);
	if (ret != 0)
		goto error_exit;

	/* Check the status */
	cvmode = hl7132_check_status(hl7132);
	if (cvmode < 0) {
		ret = cvmode;
		goto error_exit;
	}

	switch (cvmode) {
	case STS_MODE_IIN_LOOP:
		hl7132->chg_data.iin_loop_count++;
		fallthrough;
	case STS_MODE_CHG_LOOP:

		/* Check TA current */
		if (hl7132->ta_cur > HL7132_TA_MIN_CUR) {
			/* TA current is higher than 1.0A */

			/* Decrease TA current (50mA) */
			hl7132->ta_cur = hl7132->ta_cur - PD_MSG_TA_CUR_STEP;
			logbuffer_prlog(hl7132, LOGLEVEL_DEBUG,
					"%s: PreCV IIN_LOOP: ta_cur=%u",
					__func__, hl7132->ta_cur);
		} else {
			/* TA current is less than 1.0A */
			/* Decrease TA voltage (20mV) */
			hl7132->ta_vol = hl7132->ta_vol - PD_MSG_TA_VOL_STEP;
			logbuffer_prlog(hl7132, LOGLEVEL_DEBUG,
					"%s: PreCV IIN_LOOP: ta_vol=%u",
					__func__, hl7132->ta_vol);
		}

		/* Send PD Message */
		hl7132->timer_id = TIMER_PDMSG_SEND;
		hl7132->timer_period = 0;
		break;

	case STS_MODE_VFLT_LOOP:
		/* Decrease TA voltage (20mV) */
		hl7132->ta_vol = hl7132->ta_vol -
					HL7132_TA_VOL_STEP_PRE_CV *
					hl7132->chg_mode;
		logbuffer_prlog(hl7132, LOGLEVEL_DEBUG,
				"%s: PreCV VF Cont: ta_vol=%u",
				__func__, hl7132->ta_vol);

		/* Send PD Message */
		hl7132->timer_id = TIMER_PDMSG_SEND;
		hl7132->timer_period = 0;
		break;

	case STS_MODE_LOOP_INACTIVE:
		/* Exit Pre CV mode */
		logbuffer_prlog(hl7132, LOGLEVEL_DEBUG,
				"%s: PreCV End: ta_vol=%u, ta_cur=%u",
				__func__, hl7132->ta_vol, hl7132->ta_cur);

		/* Need to implement notification to other driver */
		/* To do here */

		/* Go to CV mode */
		hl7132->timer_id = TIMER_CHECK_CVMODE;
		hl7132->timer_period = 0;
		break;

	case STS_MODE_VIN_UVLO:
		/* VIN UVLO - just notification , it works by hardware */
		vin_vol = hl7132_read_adc(hl7132, ADCCH_VIN);

		logbuffer_prlog(hl7132, LOGLEVEL_DEBUG,
				"%s: PreCV VIN_UVLO: ta_vol=%u, vin_vol=%u",
				__func__, hl7132->ta_cur, vin_vol);

		/* Check VIN after 1sec */
		hl7132->timer_id = TIMER_ENTER_CVMODE;
		hl7132->timer_period = 1000;
		break;

	default:
		break;
	}

	mod_delayed_work(hl7132->dc_wq, &hl7132->timer_work,
			 msecs_to_jiffies(hl7132->timer_period));
error_exit:
	mutex_unlock(&hl7132->lock);
	dev_dbg(hl7132->dev, "%s: End, cvmode=%d, ret=%d\n", __func__, cvmode,
		ret);
	return ret;
}

static int hl7132_check_eoc(struct hl7132_charger *hl7132)
{
	const int eoc_tolerance = 25000; /* 25mV under max vbat_reg */
	const int vlimit = HL7132_COMP_VFLOAT_MAX - eoc_tolerance;
	int iin, vbat;

	iin = hl7132_read_adc(hl7132, ADCCH_IIN);
	if (iin < 0) {
		dev_err(hl7132->dev, "%s: iin=%d\n", __func__, iin);
		return iin;
	}

	vbat = hl7132_read_adc(hl7132, ADCCH_VBAT);
	if (vbat < 0) {
		dev_err(hl7132->dev, "%s: vbat=%d\n", __func__, vbat);
		return vbat;
	}

	dev_dbg(hl7132->dev, "%s: iin=%d, topoff=%u, vbat=%d vlimit=%d\n",
		__func__, iin, hl7132->pdata->iin_topoff, vbat, vlimit);

	return iin < hl7132->pdata->iin_topoff && vbat >= vlimit;
}

/* 2:1 Direct Charging CV MODE control */
static int hl7132_charge_cvmode(struct hl7132_charger *hl7132)
{
	int ret = 0;
	int cvmode;
	int vin_vol;

	dev_dbg(hl7132->dev, "%s: ======START=======\n", __func__);

	mutex_lock(&hl7132->lock);

	if (hl7132->charging_state != DC_STATE_CV_MODE)
		dev_info(hl7132->dev, "%s: charging_state=%u->%u\n", __func__,
			 hl7132->charging_state, DC_STATE_CV_MODE);

	hl7132->charging_state = DC_STATE_CV_MODE;

	ret = hl7132_check_error(hl7132);
	if (ret != 0)
		goto error_exit;

	/*
	 * A change in vbat_reg and cc_max here is a normal tier transition, a
	 * change in iin  means that the thermal engine has changed cc_max.
	 */
	ret = hl7132_apply_new_limits(hl7132);
	if (ret < 0)
		goto error_exit;
	if (ret > 0)
		goto done;

	cvmode = hl7132_check_status(hl7132);
	if (cvmode < 0) {
		ret = cvmode;
		goto error_exit;
	}

	if (cvmode == STS_MODE_LOOP_INACTIVE) {
		ret = hl7132_check_eoc(hl7132);
		if (ret < 0)
			goto error_exit;
		if (ret)
			cvmode = STS_MODE_CHG_DONE;
	}

	switch (cvmode) {
	case STS_MODE_CHG_DONE: {
		const bool done_already = hl7132->charging_state ==
					  DC_STATE_CHARGING_DONE;

		if (!done_already)
			dev_info(hl7132->dev, "%s: charging_state=%u->%u\n",
				 __func__, hl7132->charging_state,
				 DC_STATE_CHARGING_DONE);


		/* Keep CV mode until driver send stop charging */
		hl7132->charging_state = DC_STATE_CHARGING_DONE;
		power_supply_changed(hl7132->mains);

		/* _cpm already came in */
		if (hl7132->charging_state == DC_STATE_NO_CHARGING) {
			dev_dbg(hl7132->dev, "%s: Already stop DC\n", __func__);
			break;
		}

		logbuffer_prlog(hl7132, LOGLEVEL_DEBUG,
				"%s: done_already=%d charge Done\n", __func__,
				done_already);

		hl7132->timer_id = TIMER_CHECK_CVMODE;
		hl7132->timer_period = HL7132_CVMODE_CHECK_T;
	} break;

	case STS_MODE_IIN_LOOP:
		hl7132->chg_data.iin_loop_count++;
		fallthrough;
	case STS_MODE_CHG_LOOP:
		/* Check TA current */
		if (hl7132->ta_cur > HL7132_TA_MIN_CUR) {
			/* TA current is higher than (1.0A*chg_mode) */
			/* Decrease TA current (50mA) */
			hl7132->ta_cur = hl7132->ta_cur -
						PD_MSG_TA_CUR_STEP;
			logbuffer_prlog(hl7132, LOGLEVEL_DEBUG,
					"%s: CV LOOP, Cont: ta_cur=%u",
					__func__, hl7132->ta_cur);
		} else {
			/* TA current is less than (1.0A*chg_mode) */
			/* Decrease TA Voltage (20mV) */
			hl7132->ta_vol = hl7132->ta_vol -
						PD_MSG_TA_VOL_STEP;
			logbuffer_prlog(hl7132, LOGLEVEL_DEBUG,
					"%s: CV LOOP, Cont: ta_vol=%u",
					__func__, hl7132->ta_vol);
		}

		/* Send PD Message */
		hl7132->timer_id = TIMER_PDMSG_SEND;
		hl7132->timer_period = 0;
		break;

	case STS_MODE_VFLT_LOOP:
		/* Decrease TA voltage */
		hl7132->ta_vol = hl7132->ta_vol -
					PD_MSG_TA_VOL_STEP;
		logbuffer_prlog(hl7132, LOGLEVEL_DEBUG,
				"%s: CV VFLOAT, Cont: ta_vol=%u",
				__func__, hl7132->ta_vol);

		/* Send PD Message */
		hl7132->timer_id = TIMER_PDMSG_SEND;
		hl7132->timer_period = 0;
		break;

	case STS_MODE_LOOP_INACTIVE:
		hl7132->timer_id = TIMER_CHECK_CVMODE;
		hl7132->timer_period = HL7132_CVMODE_CHECK_T;
		break;

	case STS_MODE_VIN_UVLO:
		/* VIN UVLO - just notification, it works by hardware */
		vin_vol = hl7132_read_adc(hl7132, ADCCH_VIN);
		logbuffer_prlog(hl7132, LOGLEVEL_DEBUG,
				"%s: CC VIN_UVLO: ta_cur=%u ta_vol=%u, vin_vol=%d",
				__func__, hl7132->ta_cur, hl7132->ta_vol,
				vin_vol);

		/* Check VIN after 1sec */
		hl7132->timer_id = TIMER_CHECK_CVMODE;
		hl7132->timer_period = 1000;
		break;

	default:
		break;
	}

done:
	dev_dbg(hl7132->dev,
		"%s: reschedule next id=%d period=%ld chg_state=%d\n",
		 __func__, hl7132->timer_id, hl7132->timer_period,
		hl7132->charging_state);

	mod_delayed_work(hl7132->dc_wq, &hl7132->timer_work,
			 msecs_to_jiffies(hl7132->timer_period));
error_exit:
	mutex_unlock(&hl7132->lock);
	dev_dbg(hl7132->dev, "%s: End, ret=%d cvmode=%d next\n", __func__, ret,
		cvmode);
	return ret;
}

/*
 * Preset TA voltage and current for Direct Charging Mode using
 * the configured cc_max and fv_uv limits. Used only on start
 */
static int hl7132_preset_dcmode(struct hl7132_charger *hl7132)
{
	int vbat;
	int ret = 0;
	unsigned int ta_max_vol;

	dev_dbg(hl7132->dev, "%s: ======START=======\n", __func__);
	dev_dbg(hl7132->dev, "%s: = charging_state=%u ==\n", __func__,
		 hl7132->charging_state);

	/* gcpm set ->cc_max and ->fv_uv before starting */
	if (hl7132->cc_max < 0 || hl7132->fv_uv < 0) {
		dev_err(hl7132->dev, "%s: cc_max=%d fv_uv=%d invalid\n",
			__func__, hl7132->cc_max, hl7132->fv_uv);
		return -EINVAL;
	}

	if (hl7132->charging_state != DC_STATE_PRESET_DC)
		dev_info(hl7132->dev, "%s: charging_state=%u->%u\n", __func__,
			 hl7132->charging_state, DC_STATE_PRESET_DC);

	hl7132->charging_state = DC_STATE_PRESET_DC;

	/* VBAT is over threshold but it might be "bouncy" due to transitory */
	vbat = hl7132_read_adc(hl7132, ADCCH_VBAT);
	if (vbat < 0) {
		ret = vbat;
		goto error;
	}

	/* vbat_reg is set on start from GCPM */
	if (vbat > hl7132->fv_uv) {
		dev_err(hl7132->dev,
			"%s: vbat adc=%d is higher than VFLOAT=%d\n",
			__func__, vbat, hl7132->fv_uv);
		ret = -EINVAL;
		goto error;
	}

	/* determined by ->cfg_iin and cc_max */
	hl7132->ta_max_cur = hl7132_get_iin_max(hl7132, hl7132->cc_max);
	dev_dbg(hl7132->dev,
		"%s: ta_max_cur=%u, iin_cfg=%u, hl7132->ta_type=%d\n",
		 __func__, hl7132->ta_max_cur, hl7132->pdata->iin_cfg,
		 hl7132->ta_type);

	/* HW integration guide Section 5.2 - check initialization */
	if (hl7132_check_init(hl7132)) {
		if (hl7132->hw_init_retry_cnt >= hl7132->pdata->max_init_retry) {
			dev_err(hl7132->dev,
				"%s: initialization retries failed (%d/%d) - exit 2:1 mode\n",
				 __func__, hl7132->hw_init_retry_cnt,
				 hl7132->pdata->max_init_retry);

			hl7132->charging_state = DC_STATE_ERROR;
			hl7132_vote_dc_avail(hl7132, 0, 1);
			ret = -EINVAL;
			goto error;
		} else {
			hl7132->hw_init_retry_cnt++;
			dev_warn(hl7132->dev,
				"%s: initialization not valid, retry (%d/%d)\n",
				__func__, hl7132->hw_init_retry_cnt,
				hl7132->pdata->max_init_retry);
			ret = -EAGAIN;
			goto error;
		}
	}
	hl7132->hw_init_retry_cnt = 0;

	ta_max_vol = hl7132->pdata->ta_max_vol * hl7132->chg_mode;

	if (hl7132->ftm_mode)
		goto skip_pps;
	/*
	 * Get the APDO max for 2:1 mode.
	 * Returns ->ta_max_vol, ->ta_max_cur, ->ta_max_pwr and
	 * ->ta_objpos for the given ta_max_vol and ta_max_cur.
	 */

	ret = hl7132_get_apdo_max_power(hl7132, ta_max_vol, HL7132_TA_MAX_CUR);
	if (ret < 0) {
		dev_warn(hl7132->dev, "%s: No APDO to support 2:1 for %d\n",
			__func__, HL7132_TA_MAX_CUR);
		ret = hl7132_get_apdo_max_power(hl7132, ta_max_vol, 0);
	}

	if (ret < 0) {
		dev_err(hl7132->dev, "%s: No APDO to support 2:1\n", __func__);
		hl7132->charging_state = DC_STATE_ERROR;
		hl7132_vote_dc_avail(hl7132, 0, 1);
		goto error;
	}

skip_pps:
	/*
	 * ->ta_max_cur is too high for startup, needs to target
	 * CC before hitting max current AND work to ta_max_cur
	 * from there.
	 */
	ret = hl7132_set_wired_dc(hl7132, vbat);
	if (ret < 0) {
		dev_err(hl7132->dev, "%s: set wired failed (%d)\n", __func__,
			ret);
		hl7132->chg_mode = CHG_NO_DC_MODE;
		goto error;
	}

	logbuffer_prlog(hl7132, LOGLEVEL_INFO,
			"Preset DC, objpos=%d ta_max_vol=%u, ta_max_cur=%u, ta_max_pwr=%lu, iin_cc=%u, chg_mode=%u",
			hl7132->ta_objpos, hl7132->ta_max_vol, hl7132->ta_max_cur,
			hl7132->ta_max_pwr, hl7132->iin_cc, hl7132->chg_mode);

error:
	dev_dbg(hl7132->dev, "%s: End, ret=%d\n", __func__, ret);
	return ret;
}

/* Preset direct charging configuration and start charging */
static int hl7132_preset_config(struct hl7132_charger *hl7132)
{
	int ret = 0;

	dev_dbg(hl7132->dev, "%s: ======START=======\n", __func__);

	mutex_lock(&hl7132->lock);

	dev_info(hl7132->dev, "%s: charging_state=%u->%u\n", __func__,
		 hl7132->charging_state, DC_STATE_PRESET_DC);

	hl7132->charging_state = DC_STATE_PRESET_DC;

	/* HW integration guide section 5.4.3.a - set CC mode current */
	/* ->iin_cc and ->fv_uv are configured externally */
	ret = hl7132_set_input_current(hl7132, hl7132->pdata->iin_cfg);
	if (ret < 0)
		goto error;

	/* HW integration guide section 5.4.3.b - set CV mode voltage */
	ret = hl7132_set_vbat_reg(hl7132, hl7132->fv_uv);
	if (ret < 0)
		goto error;

	/* HW integration guide section 5.4.3.c - enable QRB FET & CP */
	/* Enable HL7132 unless aready enabled */
	ret = hl7132_set_charging(hl7132, true);
	if (ret < 0)
		goto error;

	/* Clear previous iin adc */
	hl7132->prev_iin = 0;
	hl7132->prev_inc = INC_NONE;

	/* Go to CHECK_ACTIVE state after 150ms */
	hl7132->timer_id = TIMER_CHECK_ACTIVE;
	hl7132->timer_period = HL7132_ENABLE_DELAY_T;
	mod_delayed_work(hl7132->dc_wq, &hl7132->timer_work,
			 msecs_to_jiffies(hl7132->timer_period));
error:
	mutex_unlock(&hl7132->lock);
	dev_dbg(hl7132->dev, "%s: End, ret=%d\n", __func__, ret);
	return ret;
}

/*
 * Check the charging status at start before entering the adjust cc mode or
 * from hl7132_send_message() after a failure.
 */
static int hl7132_check_active_state(struct hl7132_charger *hl7132)
{
	int ret = 0;

	dev_dbg(hl7132->dev, "%s: ======START=======\n", __func__);
	dev_dbg(hl7132->dev, "%s: = charging_state=%u ==\n", __func__,
		hl7132->charging_state);

	mutex_lock(&hl7132->lock);

	if (hl7132->charging_state != DC_STATE_CHECK_ACTIVE)
		dev_info(hl7132->dev, "%s: charging_state=%u->%u\n", __func__,
			 hl7132->charging_state, DC_STATE_CHECK_ACTIVE);

	hl7132->charging_state = DC_STATE_CHECK_ACTIVE;

	ret = hl7132_check_error(hl7132);
	if (ret == 0) {
		/* HL7132 is active state */
		hl7132->retry_cnt = 0;
		hl7132->timer_id = TIMER_ADJUST_CCMODE;
		hl7132->timer_period = 0;
	} else if (ret == -EAGAIN) {
		/* try restarting only */
		if (hl7132->retry_cnt >= HL7132_MAX_RETRY_CNT) {
			dev_err(hl7132->dev, "%s: retry failed\n", __func__);
			hl7132->charging_state = DC_STATE_ERROR;
			hl7132_vote_dc_avail(hl7132, 0, 1);
			ret = -EINVAL;
			goto exit_done;
		}

		/*
		 * HW integration guide section 5.5 - Disable charging
		 *
		 * Disable charging to retry enabling it later, return 0 here
		 * and the timer loop will figure out that there is something
		 * wrong and will retry.
		 */
		ret = hl7132_set_charging(hl7132, false);
		dev_err(hl7132->dev, "%s: retry cnt=%d, (%d)\n", __func__,
		       hl7132->retry_cnt, ret);
		if (ret == 0) {
			hl7132->timer_id = TIMER_PRESET_DC;
			hl7132->timer_period = 0;
			hl7132->retry_cnt++;
		}
	} else {
		dev_err(hl7132->dev, "%s: Error! disabling hl7132: ret(%d)\n",
			__func__, ret);
		hl7132->charging_state = DC_STATE_ERROR;
		hl7132_vote_dc_avail(hl7132, 0, 1);
	}

exit_done:

	/* Implement error handler function if it is needed */
	if (ret < 0) {
		logbuffer_prlog(hl7132, LOGLEVEL_ERR,
				"%s: charging_state=%d, not active or error (%d)",
				__func__, hl7132->charging_state, ret);
		hl7132->timer_id = TIMER_ID_NONE;
		hl7132->timer_period = 0;
	}

	if (!hl7132->ftm_mode)
		mod_delayed_work(hl7132->dc_wq, &hl7132->timer_work,
				 msecs_to_jiffies(hl7132->timer_period));
	mutex_unlock(&hl7132->lock);
	return ret;
}

/* Enter direct charging algorithm */
static int hl7132_start_direct_charging(struct hl7132_charger *hl7132)
{
	struct hl7132_chg_stats *chg_data = &hl7132->chg_data;
	int ret = 0;

	mutex_lock(&hl7132->lock);

	/* configure DC charging type for the requested index */
	if (!hl7132->ftm_mode)
		ret = hl7132_set_ta_type(hl7132, hl7132->pps_index);

	dev_info(hl7132->dev, "%s: Current ta_type=%d, chg_mode=%d\n", __func__,
		 hl7132->ta_type, hl7132->chg_mode);

	if (ret < 0)
		goto error_done;

	/* wake lock */
	__pm_stay_awake(hl7132->monitor_wake_lock);

	/* Preset charging configuration and TA condition */
	ret = hl7132_preset_dcmode(hl7132);
	if (ret == 0) {
		/* Configure the TA  and start charging */
		hl7132->timer_id = TIMER_PDMSG_SEND;
		hl7132->timer_period = 0;

		mod_delayed_work(hl7132->dc_wq, &hl7132->timer_work,
				 msecs_to_jiffies(hl7132->timer_period));
	}

error_done:
	dev_dbg(hl7132->dev, "%s: End, ret=%d\n", __func__, ret);

	hl7132_chg_stats_update(chg_data, hl7132);
	mutex_unlock(&hl7132->lock);
	return ret;
}

/* Check Vbat minimum level to start direct charging */
static int hl7132_check_vbatmin(struct hl7132_charger *hl7132)
{
	int ret = 0, vbat;
	unsigned int val = 0;

	mutex_lock(&hl7132->lock);

	if (hl7132->charging_state != DC_STATE_CHECK_VBAT)
		dev_info(hl7132->dev, "%s: charging_state=%u->%u\n", __func__,
			 hl7132->charging_state, DC_STATE_CHECK_VBAT);

	hl7132->charging_state = DC_STATE_CHECK_VBAT;

	/* HW integration guide section 5.4.2 */
	ret = regmap_read(hl7132->regmap, HL7132_REG_STATUS_A, &val);
	if (ret < 0) {
		dev_err(hl7132->dev, "%s: Failed to read STATUS_A\n", __func__);
		goto error;
	}

	if (val & HL7132_BIT_VIN_UVLO_STS) {
		dev_err(hl7132->dev,
			"%s: USB not inserted, not starting charging\n",
			__func__);
		ret = -EINVAL;
		goto error;
	}

	ret = regmap_read(hl7132->regmap, HL7132_REG_INT_STS_A, &val);
	if (ret < 0) {
		dev_err(hl7132->dev, "%s: Failed to read INT_STS_A\n", __func__);
		goto error;
	}

	val = (val & HL7132_BIT_STATE_CHG_STS) >> MASK2SHIFT(HL7132_BIT_STATE_CHG_STS);

	switch (val) {
	case STATE_CHG_STS_RESET:
		dev_err(hl7132->dev,
			"%s: Chip in RESET mode, not starting charging\n",
			__func__);
		ret = -EINVAL;
		break;
	case STATE_CHG_STS_SHUTDOWN:
		/* Exit 2:1 mode */
		hl7132->charging_state = DC_STATE_NO_CHARGING;
		hl7132_vote_dc_avail(hl7132, 0, 1);
		ret = -EINVAL;
		break;
	case STATE_CHG_STS_STANDBY:
		/* Ready to charge, nothing needed here */
		break;
	case STATE_CHG_STS_ACTIVE:
		ret = regmap_read(hl7132->regmap, HL7132_REG_CTRL_0, &val);
		if (ret < 0) {
			dev_err(hl7132->dev, "%s: Failed to read REG_CTRL_0\n",
				__func__);
			goto error;
		}

		if ((val & HL7132_BIT_CHG_EN) == 0) {
			dev_err(hl7132->dev,
				"%s: Chip reports ACTIVE but CHG_EN=0, reinitializing\n",
				__func__);

			hl7132->hw_init_done = false;
			ret = -EAGAIN;
			if (hl7132_hw_init(hl7132) == 0)
				hl7132->hw_init_done = true;
			else
				ret = -EINVAL;

			goto error;
		}

		/* No errors, continue charging */
		break;
	default:
		/* This should never be hit, the above 4 states enumerate all
		 * possible states reported by 2 bits
		 */
		dev_err(hl7132->dev, "%s: Unexpected STATE_CHG_STS %d\n",
			__func__, val);
		ret = -EINVAL;
		goto error;
	}

	if (hl7132->hw_init_done == false)
		if (!hl7132_hw_init(hl7132))
			hl7132->hw_init_done = true;

	vbat = hl7132_read_adc(hl7132, ADCCH_VBAT);
	if (vbat < 0) {
		ret = vbat;
		goto error;
	}

	/* wait for CPM to send in the params */
	if (hl7132->cc_max < 0 || hl7132->fv_uv < 0 || !hl7132->hw_init_done) {
		dev_dbg(hl7132->dev,
			"%s: not yet fv_uv=%d, cc_max=%d vbat=%d, hw_init_done=%d\n",
			__func__, hl7132->fv_uv, hl7132->cc_max, vbat,
			hl7132->hw_init_done);

		/* retry again after 1sec */
		hl7132->timer_id = TIMER_VBATMIN_CHECK;
		hl7132->timer_period = HL7132_VBATMIN_CHECK_T;
		hl7132->retry_cnt += 1;
	} else {
		logbuffer_prlog(hl7132, LOGLEVEL_INFO,
				"%s: starts at fv_uv=%d, cc_max=%d vbat=%d (min=%d)",
				__func__, hl7132->fv_uv, hl7132->cc_max, vbat,
				HL7132_DC_VBAT_MIN);

		hl7132->timer_id = TIMER_PRESET_DC;
		hl7132->timer_period = 0;
		hl7132->retry_cnt = 0; /* start charging */
	}

	/* timeout for VBATMIN or charging parameters */
	if (hl7132->retry_cnt > HL7132_MAX_RETRY_CNT) {
		logbuffer_prlog(hl7132, LOGLEVEL_DEBUG,
				"%s: TIMEOUT fv_uv=%d, cc_max=%d vbat=%d limit=%d",
				__func__, hl7132->fv_uv, hl7132->cc_max, vbat,
				HL7132_DC_VBAT_MIN);
		ret = -ETIMEDOUT;
	} else {
		mod_delayed_work(hl7132->dc_wq, &hl7132->timer_work,
				 msecs_to_jiffies(hl7132->timer_period));
	}


error:
	mutex_unlock(&hl7132->lock);
	dev_dbg(hl7132->dev, "%s: End, ret=%d\n", __func__, ret);
	return ret;
}

/*
 * return 1 if apdo switch, 0 if no switch. < 0 on err.
 * TODO : lower input current per TCPC spec
 */
static int hl7132_check_apdo_switch(struct hl7132_charger *hl7132)
{
	unsigned int ta_max_vol, ta_max_cur, ta_objpos;
	unsigned int new_ta_cur, new_ta_max_cur, val;
	int ret;

	dev_dbg(hl7132->dev,
		"%s:START: ta_vol: %d, prev_ta_vol: %d, ta_cur: %d, prev_ta_cur: %d\n",
		__func__, hl7132->ta_vol, hl7132->prev_ta_vol, hl7132->ta_cur,
		hl7132->prev_ta_cur);

	ta_max_vol = hl7132->ta_vol;
	ta_max_cur = hl7132->ta_cur;

	ret = hl7132_get_apdo_index(hl7132, &ta_max_vol, &ta_max_cur, &ta_objpos);
	if (ret) {
		dev_dbg(hl7132->dev, "%s: error getting apdo index (%d)\n",
			__func__, ret);
		return ret;
	}

	if (hl7132->prev_ta_cur == hl7132->ta_cur && hl7132->prev_ta_vol == hl7132->ta_vol) {
		hl7132->ta_objpos = ta_objpos;
		return 0;
	}

	if (ta_objpos == hl7132->ta_objpos) {
		dev_dbg(hl7132->dev, "%s: stay at apdo %d\n", __func__,
			ta_objpos);
		return 0;
	}

	if (hl7132->prev_ta_cur < hl7132->ta_cur) {
		/* Should never happen as we limit to ta_max_cur */
		dev_err(hl7132->dev,
			"%s: ta_cur: %d > ta_max_cur %d causing APDO switch\n",
			__func__, hl7132->ta_cur, hl7132->ta_max_cur);
		hl7132->ta_cur = hl7132->ta_max_cur;
		ret = -EINVAL;
	} else if (hl7132->prev_ta_vol != hl7132->ta_vol) {
		const long power = (hl7132->prev_ta_cur / 1000) * (hl7132->prev_ta_vol / 1000);

		new_ta_cur = (power / (hl7132->ta_vol / 1000)) * 1000;
		new_ta_max_cur = new_ta_cur;
		ta_max_vol = hl7132->ta_vol;
		dev_dbg(hl7132->dev,
			"%s: find new ta_cur: ta_vol: %d, ta_cur: %d\n",
			__func__, hl7132->ta_vol, new_ta_cur);
		ret = hl7132_get_apdo_index(hl7132, &ta_max_vol, &new_ta_max_cur, &ta_objpos);
		if (ret) {
			new_ta_max_cur = 0;
			ta_max_vol = hl7132->ta_vol;
			ret = hl7132_get_apdo_index(hl7132, &ta_max_vol, &new_ta_max_cur,
							&ta_objpos);
			if (ret) {
				dev_err(hl7132->dev,
					"No available APDO to switch to (%d)\n",
					ret);
			} else {
				/* APDO can't provide needed ta_cur, so limit to max */
				val = new_ta_max_cur / PD_MSG_TA_CUR_STEP;
				hl7132->ta_cur = val * PD_MSG_TA_CUR_STEP;
				hl7132->ta_max_cur = hl7132->ta_cur;
			}
		} else {
			val = new_ta_cur / PD_MSG_TA_CUR_STEP;
			hl7132->ta_cur = val * PD_MSG_TA_CUR_STEP;
			hl7132->ta_max_cur = new_ta_max_cur;
		}
	}

	if (!ret && ta_objpos != hl7132->ta_objpos) {
		const int temp_cur = hl7132->ta_cur;

		dev_info(hl7132->dev,
			 "ta_vol: %d->%d, ta_cur: %d->%d, ta_pos: %d->%d\n",
			 hl7132->prev_ta_vol, hl7132->ta_vol, hl7132->prev_ta_cur,
			 hl7132->ta_cur, hl7132->ta_objpos, ta_objpos);

		hl7132->ta_objpos = ta_objpos;

		/* Send one message immediately */
		/* force only voltage change */
		hl7132->ta_cur = hl7132->prev_ta_cur;
		ret = hl7132_send_pd_message(hl7132, PD_MSG_REQUEST_APDO);
		if (ret < 0)
			return ret;
		hl7132->ta_cur = temp_cur;
		ret = 1;
	}

	return ret;
}

static int hl7132_send_message(struct hl7132_charger *hl7132)
{
	int val, ret;
	const int timer_id = hl7132->timer_id;

	/* Go to the next state */
	mutex_lock(&hl7132->lock);

	dev_dbg(hl7132->dev, "%s: ====== START =======\n", __func__);

	if (hl7132->ftm_mode)
		goto skip_pps;

	/* Adjust TA current and voltage step */
	/* PPS voltage resolution is 20mV */
	val = hl7132->ta_vol / PD_MSG_TA_VOL_STEP;
	hl7132->ta_vol = val * PD_MSG_TA_VOL_STEP;
	/* PPS current resolution is 50mA */
	val = hl7132->ta_cur / PD_MSG_TA_CUR_STEP;
	hl7132->ta_cur = val * PD_MSG_TA_CUR_STEP;
	/* PPS minimum current is 1000mA */
	if (hl7132->ta_cur < HL7132_TA_MIN_CUR)
		hl7132->ta_cur = HL7132_TA_MIN_CUR;

	dev_dbg(hl7132->dev, "%s: ta_type=%d, ta_vol=%d ta_cur=%d\n", __func__,
		hl7132->ta_type, hl7132->ta_vol, hl7132->ta_cur);

	if (!hl7132->prev_ta_cur)
		hl7132->prev_ta_cur = hl7132->ta_cur;
	if (!hl7132->prev_ta_vol)
		hl7132->prev_ta_vol = hl7132->ta_vol;

	hl7132_check_apdo_switch(hl7132);
	/* Send PD Message */
	ret = hl7132_send_pd_message(hl7132, PD_MSG_REQUEST_APDO);
	if (ret >= 0) {
		hl7132->prev_ta_cur = hl7132->ta_cur;
		hl7132->prev_ta_vol = hl7132->ta_vol;
	}

skip_pps:
	switch (hl7132->charging_state) {
	case DC_STATE_PRESET_DC:
		hl7132->timer_id = TIMER_PRESET_CONFIG;
		break;
	case DC_STATE_ADJUST_CC:
		hl7132->timer_id = TIMER_ADJUST_CCMODE;
		break;
	case DC_STATE_CC_MODE:
		hl7132->timer_id = TIMER_CHECK_CCMODE;
		break;
	case DC_STATE_START_CV:
		hl7132->timer_id = TIMER_ENTER_CVMODE;
		break;
	case DC_STATE_CV_MODE:
		hl7132->timer_id = TIMER_CHECK_CVMODE;
		break;
	case DC_STATE_ADJUST_TAVOL:
		hl7132->timer_id = TIMER_ADJUST_TAVOL;
		break;
	case DC_STATE_ADJUST_TACUR:
		hl7132->timer_id = TIMER_ADJUST_TACUR;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	if (ret < 0) {
		dev_err(hl7132->dev, "%s: Error-send_pd_message to %d (%d)\n",
			__func__, hl7132->ta_type, ret);
		hl7132->timer_id = TIMER_CHECK_ACTIVE;
	}

	if (hl7132->ftm_mode)
		hl7132->timer_period = 0;
	else
		hl7132->timer_period = HL7132_PDMSG_WAIT_T;

	logbuffer_prlog(hl7132, LOGLEVEL_DEBUG,
			"%s: charging_state=%u timer_id:%d->%d ret=%d",
			__func__, hl7132->charging_state,
			timer_id, hl7132->timer_id, ret);

	mod_delayed_work(hl7132->dc_wq, &hl7132->timer_work,
			 msecs_to_jiffies(hl7132->timer_period));

	dev_dbg(hl7132->dev, "%s: End: timer_id=%d timer_period=%lu\n", __func__,
		hl7132->timer_id, hl7132->timer_period);

	mutex_unlock(&hl7132->lock);
	return ret;
}

/* delayed work function for charging timer */
static void hl7132_timer_work(struct work_struct *work)
{
	struct hl7132_charger *hl7132 =
		container_of(work, struct hl7132_charger, timer_work.work);
	unsigned int charging_state;
	int timer_id;
	int ret = 0;

	dev_dbg(hl7132->dev, "%s: ========= START =========\n", __func__);

	/* TODO: remove locks from the calls and run all of this locked */
	mutex_lock(&hl7132->lock);

	hl7132_chg_stats_update(&hl7132->chg_data, hl7132);
	charging_state = hl7132->charging_state;
	timer_id = hl7132->timer_id;

	dev_dbg(hl7132->dev, "%s: timer id=%d, charging_state=%u\n", __func__,
		 hl7132->timer_id, charging_state);

	mutex_unlock(&hl7132->lock);

	switch (timer_id) {

	/* charging_state <- DC_STATE_CHECK_VBAT */
	case TIMER_VBATMIN_CHECK:
		ret = hl7132_check_vbatmin(hl7132);
		if (ret < 0)
			goto error;
		break;

	/* charging_state <- DC_STATE_PRESET_DC */
	case TIMER_PRESET_DC:
		ret = hl7132_start_direct_charging(hl7132);
		if (ret < 0)
			goto error;
		break;

	/*
	 * charging_state <- DC_STATE_PRESET_DC
	 *	preset configuration, start charging
	 */
	case TIMER_PRESET_CONFIG:
		ret = hl7132_preset_config(hl7132);
		if (ret < 0)
			goto error;
		break;

	/*
	 * charging_state <- DC_STATE_PRESET_DC
	 *	150 ms after preset_config
	 */
	case TIMER_CHECK_ACTIVE:
		ret = hl7132_check_active_state(hl7132);
		if (ret < 0)
			goto error;
		break;

	case TIMER_ADJUST_CCMODE:
		ret = hl7132_charge_adjust_ccmode(hl7132);
		if (ret < 0)
			goto error;
		break;

	case TIMER_CHECK_CCMODE:
		ret = hl7132_charge_ccmode(hl7132);
		if (ret < 0)
			goto error;
		break;

	case TIMER_ENTER_CVMODE:
		/* Enter Pre-CV mode */
		ret = hl7132_charge_start_cvmode(hl7132);
		if (ret < 0)
			goto error;
		break;

	case TIMER_CHECK_CVMODE:
		ret = hl7132_charge_cvmode(hl7132);
		if (ret < 0)
			goto error;
		break;

	case TIMER_PDMSG_SEND:
		ret = hl7132_send_message(hl7132);
		if (ret < 0)
			goto error;
		break;

	/* called from 2 contexts */
	case TIMER_ADJUST_TAVOL:
		mutex_lock(&hl7132->lock);

		ret = hl7132_adjust_ta_voltage(hl7132);
		if (ret < 0) {
			mutex_unlock(&hl7132->lock);
			goto error;
		}

		mod_delayed_work(hl7132->dc_wq, &hl7132->timer_work,
				 msecs_to_jiffies(hl7132->timer_period));
		mutex_unlock(&hl7132->lock);
		break;

	/* called from 2 contexts */
	case TIMER_ADJUST_TACUR:
		mutex_lock(&hl7132->lock);
		ret = hl7132_adjust_ta_current(hl7132);
		if (ret < 0) {
			mutex_unlock(&hl7132->lock);
			goto error;
		}

		mod_delayed_work(hl7132->dc_wq, &hl7132->timer_work,
				 msecs_to_jiffies(hl7132->timer_period));
		mutex_unlock(&hl7132->lock);
		break;

	case TIMER_ID_NONE:
		ret = hl7132_stop_charging(hl7132);
		if (ret < 0)
			goto error;
		break;

	default:
		break;
	}

	/* Check the charging state again */
	if (hl7132->charging_state == DC_STATE_NO_CHARGING) {
		cancel_delayed_work(&hl7132->timer_work);
		cancel_delayed_work(&hl7132->pps_work);
	}

	dev_dbg(hl7132->dev,
		"%s: timer_id=%d->%d, charging_state=%u->%u, period=%ld\n",
		 __func__, timer_id, hl7132->timer_id, charging_state,
		 hl7132->charging_state, hl7132->timer_period);

	return;

error:
	dev_dbg(hl7132->dev, "%s: ========= ERROR =========\n", __func__);
	logbuffer_prlog(hl7132, LOGLEVEL_ERR,
			"%s: timer_id=%d->%d, charging_state=%u->%u, period=%ld ret=%d",
			__func__, timer_id, hl7132->timer_id, charging_state,
			hl7132->charging_state, hl7132->timer_period, ret);

	hl7132_stop_charging(hl7132);
}

/* delayed work function for pps periodic timer */
static void hl7132_pps_request_work(struct work_struct *work)
{
	struct hl7132_charger *hl7132 = container_of(work,
					struct hl7132_charger, pps_work.work);
	int ret = 0;

	dev_dbg(hl7132->dev, "%s: = charging_state=%u ==\n", __func__,
		 hl7132->charging_state);

	if (!hl7132->ftm_mode)
		ret = hl7132_send_pd_message(hl7132, PD_MSG_REQUEST_APDO);
	if (ret < 0)
		dev_err(hl7132->dev, "%s: Error-send_pd_message\n", __func__);

	/* TODO: do other background stuff */

	dev_dbg(hl7132->dev, "%s: ret=%d\n", __func__, ret);
}

enum {
	ENUM_INT,
	ENUM_INT_MASK,
	ENUM_INT_STS_A,
	ENUM_INT_STS_B,
	ENUM_INT_MAX,
};

static irqreturn_t hl7132_interrupt_handler(int irg, void *data)
{
	struct hl7132_charger *hl7132 = data;
	u8 r_buf[ENUM_INT_MAX];
	u8 masked;
	int ret = 0;
	bool handled = false;

	ret = regmap_bulk_read(hl7132->regmap, HL7132_REG_INT, r_buf, 4);
	if (ret < 0) {
		dev_warn(hl7132->dev, "Failed to read interrupt registers\n");
		return IRQ_NONE;
	}

	masked = r_buf[ENUM_INT] & ~r_buf[ENUM_INT_MASK];

	dev_dbg(hl7132->dev,
		"INT=0x%2x, INT_MSK=0x%2x, STS_A=0x%2x, STS_B=0x%2x, masked=0x%2x\n",
		r_buf[ENUM_INT], r_buf[ENUM_INT_MASK], r_buf[ENUM_INT_STS_A],
		r_buf[ENUM_INT_STS_B], masked);

	if (masked & HL7132_BIT_STATE_CHG_I) {
		dev_info(hl7132->dev, "[%s] dev_state changed!\n", __func__);
		handled = true;
	}

	if (masked & HL7132_BIT_REG_I) {
		dev_info(hl7132->dev, "[%s] regulation status changed!\n", __func__);
		handled = true;
	}

	if (masked & HL7132_BIT_TS_TEMP_I) {
		dev_info(hl7132->dev, "[%s] TS threshold crossed, disabling chip!\n", __func__);
		handled = true;
	}

	if (masked & HL7132_BIT_V_OK_I) {
		dev_info(hl7132->dev, "[%s] V_OK_I changed!\n", __func__);
		handled = true;
	}

	if (masked & HL7132_BIT_CUR_I) {
		dev_info(hl7132->dev, "[%s] CUR_I changed!\n", __func__);
		handled = true;
	}

	if (masked & HL7132_BIT_SHORT_I) {
		dev_info(hl7132->dev, "[%s] SHORT_I changed!\n", __func__);
		handled = true;
	}

	if (masked & HL7132_BIT_WDOG_I) {
		dev_info(hl7132->dev, "[%s] WDOG_I changed!\n", __func__);
		handled = true;
	}

	return handled ? IRQ_HANDLED : IRQ_NONE;
}

static int hl7132_irq_init(struct hl7132_charger *hl7132,
			   struct i2c_client *client)
{
	const struct hl7132_platform_data *pdata = hl7132->pdata;
	int ret, irq;

	irq = gpio_to_irq(pdata->irq_gpio);

	ret = devm_gpio_request_one(hl7132->dev, pdata->irq_gpio, GPIOF_IN,
				    client->name);
	if (ret < 0)
		goto fail;

	ret = devm_request_threaded_irq(hl7132->dev, irq, NULL,
					hl7132_interrupt_handler,
					IRQF_TRIGGER_LOW | IRQF_ONESHOT,
					client->name, hl7132);
	if (ret < 0)
		goto fail_gpio;

	/* All interrupts are masked by default. hl7132_hw_init unmasks
	 * the TS interrupt.
	 */
	client->irq = irq;
	return 0;

fail_gpio:
	gpio_free(pdata->irq_gpio);
fail:
	free_irq(irq, hl7132);
	client->irq = 0;
	return ret;
}

/* Returns the input current limit programmed into the charger in uA. */
int hl7132_input_current_limit(struct hl7132_charger *hl7132)
{
	int ret, intval;
	unsigned int val;

	if (!hl7132->mains_online)
		return -ENODATA;

	ret = regmap_read(hl7132->regmap, HL7132_REG_IIN_REG, &val);
	if (ret < 0)
		return ret;

	/* 50 mA/step * 1000 uA/mA = 50000 uA/step). */
	intval = (val & HL7132_BITS_IIN_REG_TH) * 50000;

	if (intval < 1000000) /* HL7132 min is 1A */
		intval = 1000000;

	return intval;
}

/* Returns the constant charge current requested from GCPM */
static int get_const_charge_current(struct hl7132_charger *hl7132)
{
	/* Charging current cannot be controlled directly */
	return hl7132->cc_max;
}

/* Return the constant charge voltage programmed into the charger in uV. */
static int hl7132_const_charge_voltage(struct hl7132_charger *hl7132)
{
	unsigned int reg_val;
	int ret;

	if (!hl7132->mains_online)
		return -ENODATA;

	/* Read VBAT_REG register */
	ret = regmap_read(hl7132->regmap, HL7132_REG_VBAT_REG, &reg_val);
	if (ret < 0)
		return ret;

	/* Extract VBAT_REG_TH bits and convert to uV */
	reg_val = (reg_val & HL7132_BITS_VBAT_REG_TH);
	return (reg_val * HL7132_VBAT_REG_STEP) + HL7132_VBAT_REG_OFFSET;
}

/* index is the PPS source to use */
static int hl7132_set_charging_enabled(struct hl7132_charger *hl7132, int index)
{
	int ret = 0;

	if (index < 0 || index >= PPS_INDEX_MAX)
		return -EINVAL;

	mutex_lock(&hl7132->lock);

	/* Done is detected in CV when iin goes UNDER topoff. */
	if (hl7132->charging_state == DC_STATE_CHARGING_DONE)
		index = 0;

	if (index == 0) {

		logbuffer_prlog(hl7132, LOGLEVEL_DEBUG,
				"%s: stop pps_idx=%d->%d charging_state=%d timer_id=%d",
				__func__, hl7132->pps_index, index,
				hl7132->charging_state,
				hl7132->timer_id);

		/* this is the same as stop charging */
		hl7132->pps_index = 0;

		cancel_delayed_work(&hl7132->timer_work);
		cancel_delayed_work(&hl7132->pps_work);

		/* will call hl7132_stop_charging() in timer_work() */
		hl7132->timer_id = TIMER_ID_NONE;
		hl7132->timer_period = 0;
		mod_delayed_work(hl7132->dc_wq, &hl7132->timer_work,
				 msecs_to_jiffies(hl7132->timer_period));
	} else if (hl7132->charging_state == DC_STATE_NO_CHARGING) {
		logbuffer_prlog(hl7132, LOGLEVEL_DEBUG,
				"%s: start pps_idx=%d->%d charging_state=%d timer_id=%d",
				__func__, hl7132->pps_index, index,
				hl7132->charging_state,
				hl7132->timer_id);

		/* Start Direct Charging on Index */
		hl7132->dc_start_time = get_boot_sec();
		hl7132_chg_stats_init(&hl7132->chg_data);
		hl7132->pps_index = index;

		dev_info(hl7132->dev, "%s: charging_state=%u->%u\n", __func__,
			 hl7132->charging_state, DC_STATE_CHECK_VBAT);

		/* PD is already in PE_SNK_STATE */
		hl7132->charging_state = DC_STATE_CHECK_VBAT;
		hl7132->timer_id = TIMER_VBATMIN_CHECK;
		hl7132->timer_period = 0;
		mod_delayed_work(hl7132->dc_wq, &hl7132->timer_work,
				 msecs_to_jiffies(hl7132->timer_period));

		/* Set the initial charging step */
		power_supply_changed(hl7132->mains);
	} else if (hl7132->charging_state == DC_STATE_ERROR) {
		logbuffer_prlog(hl7132, LOGLEVEL_DEBUG,
				"%s: error pps_idx=%d->%d charging_state=%d timer_id=%d",
				__func__, hl7132->pps_index, index,
				hl7132->charging_state,
				hl7132->timer_id);
		ret = -EINVAL;
	}

	mutex_unlock(&hl7132->lock);

	return ret;
}

static int hl7132_mains_set_property(struct power_supply *psy,
				      enum power_supply_property prop,
				      const union power_supply_propval *val)
{
	struct hl7132_charger *hl7132 = power_supply_get_drvdata(psy);
	int ret = 0;

	dev_dbg(hl7132->dev, "%s: =========START=========\n", __func__);
	dev_dbg(hl7132->dev, "%s: prop=%d, val=%d\n", __func__, prop,
		val->intval);

	if (!hl7132->init_done)
		return -EAGAIN;

	switch (prop) {

	case POWER_SUPPLY_PROP_ONLINE:
		if (val->intval == 0) {
			ret = hl7132_stop_charging(hl7132);
			if (ret < 0)
				dev_err(hl7132->dev,
					"%s: cannot stop charging (%d)\n",
				       __func__, ret);

			hl7132->mains_online = false;
		} else if (hl7132->mains_online == false) {
			hl7132->mains_online = true;
		}

		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		ret = hl7132_set_new_vfloat(hl7132, val->intval);
		break;

	/*
	 * HL7132 cannot control charging current directly so need to control
	 * current on TA side resolving cc_max for TA_VOL*TA_CUT on vbat.
	 * NOTE: iin should be equivalent to iin = cc_max /2
	 */
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = hl7132_set_new_cc_max(hl7132, val->intval);
		break;

	/* CURRENT MAX, same as IIN is really only set by the algo */
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		dev_dbg(hl7132->dev, "%s: set iin %d, ignore\n", __func__,
			val->intval);
		break;

	/* allow direct setting, not used */
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		mutex_lock(&hl7132->lock);
		ret = hl7132_set_new_iin(hl7132, val->intval);
		mutex_unlock(&hl7132->lock);
		break;

	default:
		ret = -EINVAL;
		break;
	}

	dev_dbg(hl7132->dev, "%s: End, ret=%d\n", __func__, ret);
	return ret;
}

static int hl7132_mains_get_property(struct power_supply *psy,
				     enum power_supply_property prop,
				     union power_supply_propval *val)
{
	struct hl7132_charger *hl7132 = power_supply_get_drvdata(psy);
	int intval, rc, ret = 0;

	dev_dbg(hl7132->dev, "%s: prop=%d, val=%d\n", __func__, prop,
		val->intval);

	if (!hl7132->init_done)
		return -EAGAIN;

	switch (prop) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = hl7132->mains_online;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = hl7132_is_present(hl7132);
		if (val->intval < 0)
			val->intval = 0;
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		ret = hl7132_const_charge_voltage(hl7132);
		if (ret < 0)
			return ret;
		val->intval = ret;
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		ret = get_const_charge_current(hl7132);
		if (ret < 0)
			return ret;
		val->intval = ret;
		break;

	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = hl7132_input_current_limit(hl7132);
		if (ret < 0)
			return ret;
		val->intval = ret;
		break;

	case POWER_SUPPLY_PROP_CURRENT_NOW:
		/* return the output current - uA unit */
		rc = hl7132_get_iin(hl7132, &val->intval);
		if (rc < 0)
			dev_err(hl7132->dev, "Invalid IIN ADC (%d)\n", rc);
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		mutex_lock(&hl7132->lock);
		intval = hl7132_read_adc(hl7132, ADCCH_VOUT);
		mutex_unlock(&hl7132->lock);
		if (intval < 0)
			return intval;
		val->intval = intval;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		mutex_lock(&hl7132->lock);
		intval = hl7132_read_adc(hl7132, ADCCH_VBAT);
		mutex_unlock(&hl7132->lock);
		if (intval < 0)
			return intval;
		val->intval = intval;
		break;

	/* TODO: read NTC temperature? */
	case POWER_SUPPLY_PROP_TEMP:
		mutex_lock(&hl7132->lock);
		val->intval = hl7132_read_adc(hl7132, ADCCH_TDIE);
		mutex_unlock(&hl7132->lock);
		break;

	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = hl7132_get_charge_type(hl7132);
		break;

	case POWER_SUPPLY_PROP_STATUS:
		val->intval = hl7132_get_status(hl7132);
		break;

	case POWER_SUPPLY_PROP_CURRENT_MAX:
		ret = hl7132_input_current_limit(hl7132);
		if (ret < 0)
			return ret;
		val->intval = ret;
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

/*
 * GBMS not visible
 * POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
 * POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
 * POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
 * POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX,
 */
static enum power_supply_property hl7132_mains_properties[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_TEMP,
	/* same as POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT */
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_STATUS,
};

static int hl7132_mains_is_writeable(struct power_supply *psy,
				      enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		return 1;
	default:
		break;
	}

	return 0;
}

static int hl7132_gbms_mains_set_property(struct power_supply *psy,
					   enum gbms_property prop,
					   const union gbms_propval *val)
{
	struct hl7132_charger *hl7132 = power_supply_get_drvdata(psy);
	int ret = 0;

	dev_dbg(hl7132->dev, "%s: prop=%d, val=%d\n", __func__, prop,
		val->prop.intval);
	if (!hl7132->init_done)
		return -EAGAIN;

	switch (prop) {

	case GBMS_PROP_CHARGING_ENABLED:
		dev_dbg(hl7132->dev, "%s: ChargeEnable %d, chg_state:%d\n",
			__func__, val->prop.intval, hl7132->charging_state);
		ret = hl7132_set_charging_enabled(hl7132, val->prop.intval);
		break;

	case GBMS_PROP_CHARGE_DISABLE:
		dev_dbg(hl7132->dev, "%s: ChargeDisable %d, chg_state:%d\n",
			__func__, val->prop.intval, hl7132->charging_state);
		if (val->prop.intval) {
			if (hl7132->charging_state == DC_STATE_ERROR)
				hl7132->charging_state = DC_STATE_NO_CHARGING;
			hl7132_vote_dc_avail(hl7132, 1, 1);
		}
		break;

	default:
		dev_dbg(hl7132->dev,
			"%s: route to hl7132_mains_set_property, psp:%d\n",
			__func__, prop);
		return -ENODATA;
	}

	dev_dbg(hl7132->dev, "%s: End, ret=%d\n", __func__, ret);
	return ret;
}

static int hl7132_gbms_mains_get_property(struct power_supply *psy,
					   enum gbms_property prop,
					   union gbms_propval *val)
{
	struct hl7132_charger *hl7132 = power_supply_get_drvdata(psy);
	union gbms_charger_state chg_state;
	int ret = 0;

	dev_dbg(hl7132->dev, "%s: prop=%d, val=%d\n", __func__, prop,
		val->prop.intval);
	if (!hl7132->init_done)
		return -EAGAIN;

	switch (prop) {
	case GBMS_PROP_CHARGE_DISABLE:
		ret = hl7132_get_charging_enabled(hl7132);
		if (ret < 0)
			return ret;
		val->prop.intval = !ret;
		break;

	case GBMS_PROP_CHARGING_ENABLED:
		ret = hl7132_get_charging_enabled(hl7132);
		if (ret < 0)
			return ret;
		val->prop.intval = ret;
		break;

	case GBMS_PROP_CHARGE_CHARGER_STATE:
		ret = hl7132_get_chg_chgr_state(hl7132, &chg_state);
		if (ret < 0)
			return ret;
		val->int64val = chg_state.v;
		break;

	default:
		dev_dbg(hl7132->dev,
			"%s: route to hl7132_mains_get_property, psp:%d\n",
			__func__, prop);
		return -ENODATA;
	}

	return 0;
}

static int hl7132_gbms_mains_is_writeable(struct power_supply *psy,
					   enum gbms_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
	case POWER_SUPPLY_PROP_CURRENT_MAX:
	case GBMS_PROP_CHARGING_ENABLED:
	case GBMS_PROP_CHARGE_DISABLE:
		return 1;
	default:
		break;
	}

	return 0;
}

static bool hl7132_is_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case HL7132_REG_DEVICE_ID ... HL7132_REG_ADC_TDIE_1:
		return true;
	default:
		break;
	}

	return false;
}

static struct regmap_config hl7132_regmap = {
	.name		= "dc-mains",
	.reg_bits	= 8,
	.val_bits	= 8,
	.max_register	= HL7132_MAX_REGISTER,
	.readable_reg = hl7132_is_reg,
	.volatile_reg = hl7132_is_reg,
};

static struct gbms_desc hl7132_mains_desc = {
	.psy_dsc.name		= "hl7132-mains",
	/* b/179246019 will not look online to Android */
	.psy_dsc.type		= POWER_SUPPLY_TYPE_UNKNOWN,
	.psy_dsc.get_property	= hl7132_mains_get_property,
	.psy_dsc.set_property	= hl7132_mains_set_property,
	.psy_dsc.property_is_writeable = hl7132_mains_is_writeable,
	.get_property		= hl7132_gbms_mains_get_property,
	.set_property		= hl7132_gbms_mains_set_property,
	.property_is_writeable = hl7132_gbms_mains_is_writeable,
	.psy_dsc.properties	= hl7132_mains_properties,
	.psy_dsc.num_properties	= ARRAY_SIZE(hl7132_mains_properties),
	.forward		= true,
};

#if defined(CONFIG_OF)
static int of_hl7132_dt(struct device *dev,
			 struct hl7132_platform_data *pdata)
{
	struct device_node *np_hl7132 = dev->of_node;
	int ret;

	if (!np_hl7132)
		return -EINVAL;

	/* irq gpio */
	pdata->irq_gpio = of_get_named_gpio(np_hl7132, "hl7132,irq-gpio", 0);
	dev_info(dev, "%s: irq-gpio: %d\n", __func__, pdata->irq_gpio);

	/* input current limit */
	ret = of_property_read_u32(np_hl7132, "hl7132,input-current-limit",
				   &pdata->iin_cfg_max);
	if (ret) {
		dev_warn(dev, "%s: hl7132,input-current-limit is Empty\n",
			 __func__);
		pdata->iin_cfg_max = HL7132_IIN_CFG_DFT;
	}
	pdata->iin_cfg = pdata->iin_cfg_max;
	dev_info(dev, "%s: hl7132,iin_cfg is %u\n", __func__, pdata->iin_cfg);

	/* TA max voltage limit */
	ret = of_property_read_u32(np_hl7132, "hl7132,ta-max-vol",
				   &pdata->ta_max_vol);
	if (ret) {
		dev_warn(dev, "%s: hl7132,ta-max-vol is Empty\n",
			 __func__);
		pdata->ta_max_vol = HL7132_TA_MAX_VOL;
	}
	ret = of_property_read_u32(np_hl7132, "hl7132,ta-max-vol-cp",
				   &pdata->ta_max_vol_cp);
	if (ret) {
		dev_warn(dev, "%s: hl7132,ta-max-vol-cp is Empty\n",
			 __func__);
		pdata->ta_max_vol_cp = pdata->ta_max_vol;
	}

	/* charging float voltage */
	ret = of_property_read_u32(np_hl7132, "hl7132,vbat_reg-voltage",
				   &pdata->vbat_reg_dt);
	if (ret) {
		dev_warn(dev, "%s: hl7132,vbat_reg-voltage is Empty\n",
			 __func__);
		pdata->vbat_reg_dt = HL7132_VBAT_REG_DFT;
	}
	pdata->vbat_reg = pdata->vbat_reg_dt;
	dev_info(dev, "%s: hl7132,vbat_reg is %u\n", __func__, pdata->vbat_reg);

	/* input topoff current */
	ret = of_property_read_u32(np_hl7132, "hl7132,input-itopoff",
				   &pdata->iin_topoff);
	if (ret) {
		dev_warn(dev, "%s: hl7132,input-itopoff is Empty\n",
			 __func__);
		pdata->iin_topoff = HL7132_IIN_DONE_DFT;
	}
	dev_info(dev, "%s: hl7132,iin_topoff is %u\n", __func__,
		pdata->iin_topoff);

	/* switching frequency */
	ret = of_property_read_u32(np_hl7132, "hl7132,switching-frequency",
				   &pdata->fsw_cfg);
	if (ret) {
		dev_warn(dev, "%s: hl7132,switching frequency is Empty\n",
			 __func__);
		pdata->fsw_cfg = HL7132_FSW_CFG_DFT;
	}
	dev_info(dev, "%s: hl7132,fsw_cfg is %u\n", __func__, pdata->fsw_cfg);

	/* NTC threshold voltage */
	ret = of_property_read_u32(np_hl7132, "hl7132,ntc-threshold",
				   &pdata->ntc_th);
	if (ret) {
		pr_warn("%s: hl7132,ntc threshold voltage is Empty\n",
			__func__);
		pdata->ntc_th = HL7132_NTC_TH_DFT;
	}
	dev_info(dev, "%s: hl7132,ntc_th is %u\n", __func__, pdata->ntc_th);

	/* iin offsets */
	ret = of_property_read_u32(np_hl7132, "hl7132,iin-max-offset",
				   &pdata->iin_max_offset);
	if (ret)
		pdata->iin_max_offset = HL7132_IIN_MAX_OFFSET;
	dev_info(dev, "%s: hl7132,iin_max_offset is %u\n", __func__,
		 pdata->iin_max_offset);

	ret = of_property_read_u32(np_hl7132, "hl7132,iin-cc_comp-offset",
				   &pdata->iin_cc_comp_offset);
	if (ret)
		pdata->iin_cc_comp_offset = HL7132_IIN_CC_COMP_OFFSET;
	dev_info(dev, "%s: hl7132,iin_cc_comp_offset is %u\n", __func__,
		 pdata->iin_cc_comp_offset);

	ret = of_property_read_u32(np_hl7132, "hl7132,ta-vol-offset",
				   &pdata->ta_vol_offset);
	if (ret)
		pdata->ta_vol_offset = HL7132_TA_VOL_PRE_OFFSET;
	dev_info(dev, "%s: hl7132,ta-vol-offset is %u\n", __func__,
		 pdata->ta_vol_offset);


#if IS_ENABLED(CONFIG_THERMAL)
	/* USBC thermal zone */
	ret = of_property_read_string(np_hl7132, "google,usb-port-tz-name",
				      &pdata->usb_tz_name);
	if (ret) {
		dev_info(dev, "%s: google,usb-port-tz-name is Empty\n",
			 __func__);
		pdata->usb_tz_name = NULL;
	} else {
		dev_info(dev, "%s: google,usb-port-tz-name is %s\n", __func__,
			pdata->usb_tz_name);
	}
#endif

	ret = of_property_read_u32(np_hl7132, "hl7132,max-init-retry",
				   &pdata->max_init_retry);
	if (ret)
		pdata->max_init_retry = HL7132_MAX_RETRY_CNT;
	dev_info(dev, "%s: hl7132,max-init-retry is %u\n", __func__,
		 pdata->max_init_retry);

	return 0;
}
#else
static int of_hl7132_dt(struct device *dev,
			 struct hl7132_platform_data *pdata)
{
	return 0;
}
#endif /* CONFIG_OF */

#ifdef CONFIG_THERMAL
static int hl7132_usb_tz_read_temp(struct thermal_zone_device *tzd, int *temp)
{
	struct hl7132_charger *hl7132 = tzd->devdata;

	if (!hl7132)
		return -ENODEV;

	*temp = hl7132_read_adc(hl7132, ADCCH_TDIE);

	return 0;
}

static struct thermal_zone_device_ops hl7132_usb_tzd_ops = {
	.get_temp = hl7132_usb_tz_read_temp,
};
#endif

static int read_reg(void *data, u64 *val)
{
	struct hl7132_charger *chip = data;
	int rc;
	unsigned int temp;

	rc = regmap_read(chip->regmap, chip->debug_address, &temp);
	if (rc) {
		dev_err(chip->dev, "Couldn't read reg %x rc = %d\n",
			chip->debug_address, rc);
		return -EAGAIN;
	}
	*val = temp;
	return 0;
}

static int write_reg(void *data, u64 val)
{
	struct hl7132_charger *chip = data;
	int rc;
	u8 temp;

	temp = (u8) val;
	rc = regmap_write(chip->regmap, chip->debug_address, temp);
	if (rc) {
		dev_err(chip->dev, "Couldn't write 0x%02x to 0x%02x rc = %d\n",
			temp, chip->debug_address, rc);
		return -EAGAIN;
	}
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(register_debug_ops, read_reg, write_reg, "0x%02llx\n");

static int debug_apply_offsets(void *data, u64 val)
{
	struct hl7132_charger *chip = data;
	int ret;

	ret = hl7132_set_new_cc_max(chip, chip->cc_max);
	dev_info(chip->dev,
		 "Apply offsets iin_max_o=%d iin_cc_comp_o=%d ret=%d\n",
		 chip->pdata->iin_max_offset, chip->pdata->iin_cc_comp_offset,
		 ret);

	return ret;
}
DEFINE_SIMPLE_ATTRIBUTE(apply_offsets_debug_ops, NULL, debug_apply_offsets, "0x%02llx\n");

static int debug_soft_reset_1(void *data, u64 val)
{
	int ret, reg_value;
	struct hl7132_charger *hl7132 = data;

	dev_info(hl7132->dev,
		"%s: Triggering soft reset then sleeping %llu ms before 1 read\n",
		__func__, val);

	regmap_write(hl7132->regmap, HL7132_REG_CTRL_2, 0xC0);
	/* regmap_update_bits will always report a failure after soft reset,
	 * so confirm that it succeeded by making sure HL7132_REG_CTRL_2 is back
	 * to default after waiting for soft reset to complete - chip holds I2C
	 * BUS for ~6ms after reset is triggered. Wait 100ms as per HW
	 * integration guide.
	 */
	msleep(val);

	ret = regmap_read(hl7132->regmap, HL7132_REG_CTRL_2, &reg_value);
	if (ret < 0) {
		dev_err(hl7132->dev, "%s: Failed to read after soft reset\n",
			__func__);
		return ret;
	}
	if (reg_value != HL7132_CTRL_2_DFT) {
		dev_err(hl7132->dev, "%s: Failed to perform soft reset\n",
			__func__);
		return ret;
	}

	dev_info(hl7132->dev, "%s: Soft reset complete\n", __func__);

	return ret;
}
DEFINE_SIMPLE_ATTRIBUTE(soft_reset_1_debug_ops, NULL, debug_soft_reset_1, "0x%02llx\n");

static int debug_soft_reset_2(void *data, u64 val)
{
	int ret, reg_value;
	struct hl7132_charger *hl7132 = data;

	dev_info(hl7132->dev,
		"%s: Triggering soft reset then sleeping %llu ms before 2 reads\n",
		__func__, val);

	regmap_write(hl7132->regmap, HL7132_REG_CTRL_2, 0xC0);
	/* regmap_update_bits will always report a failure after soft reset,
	 * so confirm that it succeeded by making sure HL7132_REG_CTRL_2 is back
	 * to default after waiting for soft reset to complete - chip holds I2C
	 * BUS for ~6ms after reset is triggered. Wait 100ms as per HW
	 * integration guide.
	 */
	msleep(val);

	ret = regmap_read(hl7132->regmap, HL7132_REG_CTRL_2, &reg_value);
	msleep(20);
	ret = regmap_read(hl7132->regmap, HL7132_REG_CTRL_2, &reg_value);
	if (ret < 0) {
		dev_err(hl7132->dev, "%s: Failed to read after soft reset\n",
			__func__);
		return ret;
	}
	if (reg_value != HL7132_CTRL_2_DFT) {
		dev_err(hl7132->dev, "%s: Failed to perform soft reset\n",
			__func__);
		return ret;
	}

	dev_info(hl7132->dev, "%s: Soft reset complete\n", __func__);

	return ret;
}
DEFINE_SIMPLE_ATTRIBUTE(soft_reset_2_debug_ops, NULL, debug_soft_reset_2, "0x%02llx\n");

static int debug_adc_chan_get(void *data, u64 *val)
{
	struct hl7132_charger *hl7132 = data;

	*val = hl7132_read_adc(data, hl7132->debug_adc_channel);
	return 0;
}

static int debug_adc_chan_set(void *data, u64 val)
{
	struct hl7132_charger *hl7132 = data;

	if (val < ADCCH_VIN || val >= ADCCH_MAX)
		return -EINVAL;
	hl7132->debug_adc_channel = val;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_adc_chan_ops, debug_adc_chan_get,
			debug_adc_chan_set, "%llu\n");

static int debug_ftm_mode_get(void *data, u64 *val)
{
	struct hl7132_charger *hl7132 = data;
	*val = hl7132->ftm_mode;
	return 0;
}

static int debug_ftm_mode_set(void *data, u64 val)
{
	struct hl7132_charger *hl7132 = data;

	if (val) {
		hl7132->ftm_mode = true;
		hl7132->ta_type = TA_TYPE_USBPD;
		hl7132->chg_mode = CHG_2TO1_DC_MODE;
	} else {
		hl7132->ftm_mode = false;
	}

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(debug_ftm_mode_ops, debug_ftm_mode_get, debug_ftm_mode_set, "%llu\n");

static int debug_pps_index_get(void *data, u64 *val)
{
	struct hl7132_charger *hl7132 = data;

	*val = hl7132->pps_index;
	return 0;
}

static int debug_pps_index_set(void *data, u64 val)
{
	struct hl7132_charger *hl7132 = data;

	return hl7132_set_charging_enabled(hl7132, (int)val);
}

DEFINE_SIMPLE_ATTRIBUTE(debug_pps_index_ops, debug_pps_index_get,
			debug_pps_index_set, "%llu\n");

static int debug_ta_max_vol_set(void *data, u64 val)
{
	struct hl7132_charger *hl7132 = data;

	hl7132->pdata->ta_max_vol = val;
	hl7132->pdata->ta_max_vol_cp = val;

	hl7132->ta_max_vol = val * hl7132->chg_mode;

	return 0;
}

static int debug_ta_max_vol_get(void *data, u64 *val)
{
	struct hl7132_charger *hl7132 = data;

	*val = hl7132->pdata->ta_max_vol;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(debug_ta_max_vol_ops, debug_ta_max_vol_get,
			debug_ta_max_vol_set, "%llu\n");

static ssize_t sts_ab_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct hl7132_charger *hl7132 = dev_get_drvdata(dev);
	u8 tmp[2];
	int ret;

	ret = regmap_bulk_read(hl7132->regmap, HL7132_REG_INT_STS_A, &tmp, sizeof(tmp));
	if (ret < 0)
		return ret;

	return scnprintf(buf, PAGE_SIZE, "%02x%02x\n", tmp[0], tmp[1]);
}

static DEVICE_ATTR_RO(sts_ab);


static ssize_t chg_stats_show(struct device *dev, struct device_attribute *attr,
				    char *buff)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct hl7132_charger *hl7132 = i2c_get_clientdata(client);
	struct hl7132_chg_stats *chg_data = &hl7132->chg_data;
	const int max_size = PAGE_SIZE;
	int len = -ENODATA;

	mutex_lock(&hl7132->lock);

	if (!hl7132_chg_stats_valid(chg_data))
		goto exit_done;

	len = scnprintf(buff, max_size,
			"D:%#x,%#x %#x,%#x,%#x,%#x,%#x\n",
			chg_data->adapter_capabilities[0],
			chg_data->adapter_capabilities[1],
			chg_data->receiver_state[0],
			chg_data->receiver_state[1],
			chg_data->receiver_state[2],
			chg_data->receiver_state[3],
			chg_data->receiver_state[4]);
	len += scnprintf(&buff[len], max_size - len,
			"N: ovc=%d,ovc_ibatt=%d,ovc_delta=%d rcp=%d,stby=%d, iin_loop=%d\n",
			chg_data->ovc_count, chg_data->ovc_max_ibatt, chg_data->ovc_max_delta,
			chg_data->rcp_count,
			chg_data->stby_count,
			chg_data->iin_loop_count);
	len += scnprintf(&buff[len], max_size - len,
			"C: nc=%d,pre=%d,ca=%d,cc=%d,cv=%d,adj=%d\n",
			chg_data->nc_count,
			chg_data->pre_count,
			chg_data->ca_count,
			chg_data->cc_count,
			chg_data->cv_count,
			chg_data->adj_count);

exit_done:
	mutex_unlock(&hl7132->lock);
	return len;
}

static ssize_t chg_stats_store(struct device *dev, struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct hl7132_charger *hl7132 = i2c_get_clientdata(client);

	mutex_lock(&hl7132->lock);
	hl7132_chg_stats_init(&hl7132->chg_data);
	mutex_unlock(&hl7132->lock);

	return count;
}

static DEVICE_ATTR_RW(chg_stats);

static ssize_t dump_reg_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct hl7132_charger *hl7132 = dev_get_drvdata(dev);
	u8 tmp[HL7132_MAX_REGISTER + 1];
	int ret, i;
	int len = 0;

	ret = regmap_bulk_read(hl7132->regmap, HL7132_REG_DEVICE_ID, &tmp,
			       HL7132_MAX_REGISTER + 1);
	if (ret < 0)
		return ret;

	for (i = 0; i <= HL7132_MAX_REGISTER; i++)
		len += scnprintf(&buf[len], PAGE_SIZE - len, "%02x: %02x\n", i, tmp[i]);

	return len;
}

static DEVICE_ATTR_RO(dump_reg);

static int hl7132_create_fs_entries(struct hl7132_charger *chip)
{

	device_create_file(chip->dev, &dev_attr_sts_ab);
	device_create_file(chip->dev, &dev_attr_chg_stats);
	device_create_file(chip->dev, &dev_attr_dump_reg);

	chip->debug_root = debugfs_create_dir("charger-hl7132", NULL);
	if (IS_ERR_OR_NULL(chip->debug_root)) {
		dev_err(chip->dev, "Couldn't create debug dir\n");
		return -ENOENT;
	}

	debugfs_create_u32("debug_level", 0644, chip->debug_root,
			   &debug_printk_prlog);
	debugfs_create_u32("no_logbuffer", 0644, chip->debug_root,
			   &debug_no_logbuffer);

	// TODO testing
	debugfs_create_u32("init_sleep", 0644, chip->debug_root,
			   &chip->pdata->init_sleep);

	debugfs_create_file("data", 0644, chip->debug_root, chip, &register_debug_ops);
	debugfs_create_x32("address", 0644, chip->debug_root, &chip->debug_address);

	debugfs_create_u32("iin_max_offset", 0644, chip->debug_root,
			   &chip->pdata->iin_max_offset);
	debugfs_create_u32("iin_cc_comp_offset", 0644, chip->debug_root,
			   &chip->pdata->iin_cc_comp_offset);
	debugfs_create_file("apply_offsets", 0644, chip->debug_root, chip,
			    &apply_offsets_debug_ops);

	// TODO testing
	debugfs_create_file("soft_reset1", 0644, chip->debug_root, chip,
			    &soft_reset_1_debug_ops);
		debugfs_create_file("soft_reset2", 0644, chip->debug_root, chip,
			    &soft_reset_2_debug_ops);

	debugfs_create_file("ta_vol_max", 0644, chip->debug_root, chip,
			   &debug_ta_max_vol_ops);

	chip->debug_adc_channel = ADCCH_VOUT;
	debugfs_create_file("adc_chan", 0644, chip->debug_root, chip,
			    &debug_adc_chan_ops);
	debugfs_create_file("pps_index", 0644, chip->debug_root, chip,
			    &debug_pps_index_ops);
	debugfs_create_file("ftm_mode", 0644, chip->debug_root, chip,
			    &debug_ftm_mode_ops);

	return 0;
}


static int hl7132_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	static char *battery[] = { "hl7132-battery" };
	struct power_supply_config mains_cfg = {};
	struct hl7132_platform_data *pdata;
	struct hl7132_charger *hl7132_chg;
	struct device *dev = &client->dev;
	const char *psy_name = NULL;
	int ret;

	dev_info(dev, "starting hl7132 probe\n");

	hl7132_chg = devm_kzalloc(dev, sizeof(*hl7132_chg), GFP_KERNEL);
	if (!hl7132_chg)
		return -ENOMEM;

#if defined(CONFIG_OF)
	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
				     sizeof(struct hl7132_platform_data),
				     GFP_KERNEL);
		if (!pdata) {
			dev_err(dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}

		ret = of_hl7132_dt(dev, pdata);
		if (ret < 0) {
			dev_err(dev, "Failed to get device of_node\n");
			return -ENOMEM;
		}

		client->dev.platform_data = pdata;
	} else {
		pdata = client->dev.platform_data;
	}
#else
	pdata = dev->platform_data;
#endif
	if (!pdata)
		return -EINVAL;

	i2c_set_clientdata(client, hl7132_chg);

	mutex_init(&hl7132_chg->lock);
	hl7132_chg->dev = &client->dev;
	hl7132_chg->pdata = pdata;
	hl7132_chg->charging_state = DC_STATE_NO_CHARGING;

	/* Create a work queue for the direct charger */
	hl7132_chg->dc_wq = alloc_ordered_workqueue("hl7132_dc_wq", WQ_MEM_RECLAIM);
	if (hl7132_chg->dc_wq == NULL) {
		dev_err(dev, "failed to create work queue\n");
		return -ENOMEM;
	}

	hl7132_chg->monitor_wake_lock =
		wakeup_source_register(NULL, "hl7132-charger-monitor");
	if (!hl7132_chg->monitor_wake_lock) {
		dev_err(dev, "Failed to register wakeup source\n");
		return -ENODEV;
	}

	/* initialize work */
	INIT_DELAYED_WORK(&hl7132_chg->timer_work, hl7132_timer_work);
	hl7132_chg->timer_id = TIMER_ID_NONE;
	hl7132_chg->timer_period = 0;

	INIT_DELAYED_WORK(&hl7132_chg->pps_work, hl7132_pps_request_work);

	ret = of_property_read_string(dev->of_node,
				      "hl7132,psy_name", &psy_name);
	if ((ret == 0) && (strlen(psy_name) > 0))
		hl7132_regmap.name = hl7132_mains_desc.psy_dsc.name =
		    devm_kstrdup(dev, psy_name, GFP_KERNEL);

	hl7132_chg->regmap = devm_regmap_init_i2c(client, &hl7132_regmap);
	if (IS_ERR(hl7132_chg->regmap)) {
		ret = -EINVAL;
		dev_err(dev, "regmap init failed, err = %d\n", ret);
		goto error;
	}
	ret = hl7132_probe_pps(hl7132_chg);
	if (ret < 0) {
		dev_warn(hl7132_chg->dev, "hl7132: PPS not available (%d)\n",
			 ret);
	} else {
		const char *logname = "pca9468"; /* TODO b/360866957*/

		hl7132_chg->log = logbuffer_register(logname);
		if (IS_ERR(hl7132_chg->log)) {
			dev_err(hl7132_chg->dev, "%s: no logbuffer (%ld)\n",
				 __func__, PTR_ERR(hl7132_chg->log));
			hl7132_chg->log = NULL;
		}
	}

	// TODO testing
	pdata->init_sleep = 1000;

	ret = hl7132_hw_ping(hl7132_chg);
	if (ret)
		goto error;

	/* TODO: only enable ADC if usb_tz_name is defined */
	hl7132_chg->hw_init_done = false;
	ret = hl7132_hw_init(hl7132_chg);
	if (ret == 0)
		hl7132_chg->hw_init_done = true;
	else
		goto error;

	mains_cfg.supplied_to = battery;
	mains_cfg.num_supplicants = ARRAY_SIZE(battery);
	mains_cfg.drv_data = hl7132_chg;
	hl7132_chg->mains = devm_power_supply_register(dev,
							&hl7132_mains_desc.psy_dsc,
							&mains_cfg);
	if (IS_ERR(hl7132_chg->mains)) {
		ret = -ENODEV;
		goto error;
	}

	/* Interrupt pin is optional. */
	if (pdata->irq_gpio >= 0) {
		ret = hl7132_irq_init(hl7132_chg, client);
		if (ret < 0)
			dev_warn(dev, "failed to initialize IRQ: %d\n", ret);
	}

	ret = hl7132_create_fs_entries(hl7132_chg);
	if (ret < 0)
		dev_err(dev, "error while registering debugfs %d\n", ret);

#if IS_ENABLED(CONFIG_THERMAL)
	if (pdata->usb_tz_name) {
		hl7132_chg->usb_tzd =
			thermal_zone_device_register(pdata->usb_tz_name, 0, 0,
						     hl7132_chg,
						     &hl7132_usb_tzd_ops,
						     NULL, 0, 0);
		if (IS_ERR(hl7132_chg->usb_tzd)) {
			hl7132_chg->usb_tzd = NULL;
			ret = PTR_ERR(hl7132_chg->usb_tzd);
			dev_err(dev,
				"Couldn't register usb connector thermal zone ret=%d\n",
				ret);
		}
	}
#endif

	hl7132_chg->dc_avail = NULL;

	hl7132_chg->init_done = true;
	dev_info(dev, "hl7132: probe_done\n");
	dev_dbg(dev, "%s: =========END=========\n", __func__);
	return 0;

error:
	destroy_workqueue(hl7132_chg->dc_wq);
	debugfs_remove(hl7132_chg->debug_root);
	if (hl7132_chg->log)
		logbuffer_unregister(hl7132_chg->log);
	mutex_destroy(&hl7132_chg->lock);
	wakeup_source_unregister(hl7132_chg->monitor_wake_lock);
	return ret;
}

static void hl7132_remove(struct i2c_client *client)
{
	struct hl7132_charger *hl7132_chg = i2c_get_clientdata(client);

	/* stop charging if it is active */
	hl7132_stop_charging(hl7132_chg);

	destroy_workqueue(hl7132_chg->dc_wq);
	debugfs_remove(hl7132_chg->debug_root);
	mutex_destroy(&hl7132_chg->lock);
	wakeup_source_unregister(hl7132_chg->monitor_wake_lock);

	if (client->irq) {
		free_irq(client->irq, hl7132_chg);
		gpio_free(hl7132_chg->pdata->irq_gpio);
	}

#if IS_ENABLED(CONFIG_THERMAL)
	if (hl7132_chg->usb_tzd)
		thermal_zone_device_unregister(hl7132_chg->usb_tzd);
#endif
	if (hl7132_chg->log)
		logbuffer_unregister(hl7132_chg->log);

	pps_free(&hl7132_chg->pps_data);
}

static const struct i2c_device_id hl7132_id[] = {
	{ "hl7132", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, hl7132_id);

#if defined(CONFIG_OF)
static struct of_device_id hl7132_i2c_dt_ids[] = {
	{ .compatible = "hl,hl7132" },
	{ },
};
MODULE_DEVICE_TABLE(of, hl7132_i2c_dt_ids);
#endif /* CONFIG_OF */

#if defined(CONFIG_PM)
#ifdef CONFIG_RTC_HCTOSYS
static int get_current_time(unsigned long *now_tm_sec)
{
	struct rtc_time tm;
	struct rtc_device *rtc;
	int rc;

	rtc = rtc_class_open(CONFIG_RTC_HCTOSYS_DEVICE);
	if (rtc == NULL) {
		pr_err("%s: unable to open rtc device (%s)\n", __FILE__,
		       CONFIG_RTC_HCTOSYS_DEVICE);
		return -EINVAL;
	}

	rc = rtc_read_time(rtc, &tm);
	if (rc) {
		pr_err("Error reading rtc device (%s) : %d\n",
		       CONFIG_RTC_HCTOSYS_DEVICE, rc);
		goto close_time;
	}

	rc = rtc_valid_tm(&tm);
	if (rc) {
		pr_err("Invalid RTC time (%s): %d\n",
		       CONFIG_RTC_HCTOSYS_DEVICE, rc);
		goto close_time;
	}

	*now_tm_sec = rtc_tm_to_time64(&tm);

close_time:
	rtc_class_close(rtc);
	return rc;
}

static void
hl7132_check_and_update_charging_timer(struct hl7132_charger *hl7132)
{
	unsigned long current_time = 0, next_update_time, time_left;

	get_current_time(&current_time);

	if (hl7132->timer_id != TIMER_ID_NONE) {
		next_update_time = hl7132->last_update_time +
				   (hl7132->timer_period / 1000); /* seconds */

		dev_dbg(hl7132->dev,
			"%s: current_time=%ld, next_update_time=%ld\n",
			__func__, current_time, next_update_time);

		if (next_update_time > current_time)
			time_left = next_update_time - current_time;
		else
			time_left = 0;

		mutex_lock(&hl7132->lock);
		hl7132->timer_period = time_left * 1000; /* ms unit */
		mutex_unlock(&hl7132->lock);
		schedule_delayed_work(&hl7132->timer_work,
				      msecs_to_jiffies(hl7132->timer_period));

		dev_dbg(hl7132->dev, "%s: timer_id=%d, time_period=%ld\n",
			__func__, hl7132->timer_id, hl7132->timer_period);
	}
	hl7132->last_update_time = current_time;
}
#endif

static int hl7132_suspend(struct device *dev)
{
	struct hl7132_charger *hl7132 = dev_get_drvdata(dev);

	dev_dbg(hl7132->dev, "%s: cancel delayed work\n", __func__);

	/* cancel delayed_work */
	cancel_delayed_work(&hl7132->timer_work);

	return 0;
}

static int hl7132_resume(struct device *dev)
{
	struct hl7132_charger *hl7132 = dev_get_drvdata(dev);

	dev_dbg(hl7132->dev, "%s: update_timer\n", __func__);

	/* Update the current timer */
#ifdef CONFIG_RTC_HCTOSYS
	hl7132_check_and_update_charging_timer(hl7132);
#else
	if (hl7132->timer_id != TIMER_ID_NONE) {
		mutex_lock(&hl7132->lock);
		hl7132->timer_period = 0; /* ms unit */
		mutex_unlock(&hl7132->lock);
		schedule_delayed_work(&hl7132->timer_work,
				      msecs_to_jiffies(hl7132->timer_period));
	}
#endif
	return 0;
}
#else
#define hl7132_suspend		NULL
#define hl7132_resume		NULL
#endif

const struct dev_pm_ops hl7132_pm_ops = {
	.suspend = hl7132_suspend,
	.resume = hl7132_resume,
};

static struct i2c_driver hl7132_driver = {
	.driver = {
		.name = "hl7132",
#if defined(CONFIG_OF)
		.of_match_table = hl7132_i2c_dt_ids,
#endif /* CONFIG_OF */
#if defined(CONFIG_PM)
		.pm = &hl7132_pm_ops,
#endif
	},
	.probe        = hl7132_probe,
	.remove       = hl7132_remove,
	.id_table     = hl7132_id,
};

module_i2c_driver(hl7132_driver);

MODULE_AUTHOR("Baltazar Ortiz <baltazarortiz@google.com>");
MODULE_DESCRIPTION("HL7132 gcharger driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("3.7.0");
