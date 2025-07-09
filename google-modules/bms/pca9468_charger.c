/*
 * Driver for the NXP PCA9468 battery charger.
 *
 * Copyright (C) 2018 NXP Semiconductor.
 * Copyright 2020 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
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

#include "pca9468_regs.h"
#include "pca9468_charger.h"

#if defined (CONFIG_OF)
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#endif /* CONFIG_OF */

/* adc_gain bit[7:4] of reg 0x31 - 2's complement */
static int adc_gain[16] = { 0,  1,  2,  3,  4,  5,  6,  7,
			   -8, -7, -6, -5, -4, -3, -2, -1};

/* Timer definition */
#define PCA9468_VBATMIN_CHECK_T	1000	/* 1000ms */
#define PCA9468_CCMODE_CHECK1_T	5000	/* 10000ms -> 500ms */
#define PCA9468_CCMODE_CHECK2_T	5000	/* 5000ms */
#define PCA9468_CVMODE_CHECK_T 	10000	/* 10000ms */
#define PCA9468_ENABLE_DELAY_T	150	/* 150ms */
#define PCA9468_CVMODE_CHECK2_T	1000	/* 1000ms */
#define PCA9468_ENABLE_WLC_DELAY_T	300	/* 300ms */

/* Battery Threshold */
#define PCA9468_DC_VBAT_MIN		3400000 /* uV */
/* Input Current Limit default value */
#define PCA9468_IIN_CFG_DFT		2500000 /* uA*/
/* Charging Float Voltage default value */
#define PCA9468_VFLOAT_DFT		4350000	/* uV */
/* Charging Sub Float Voltage default value */
#define PCA9468_VFLOAT_SUB_DFT		5000000	/* 5000000uV */
/* Charging Float Voltage max voltage for comp */
#define PCA9468_COMP_VFLOAT_MAX		4450000	/* uV */

/* Sense Resistance default value */
#define PCA9468_SENSE_R_DFT		1	/* 10mOhm */
/* Switching Frequency default value */
#define PCA9468_FSW_CFG_DFT		3	/* 980KHz */
/* NTC threshold voltage default value */
#define PCA9468_NTC_TH_DFT		0	/* uV*/

/* Charging Done Condition */
#define PCA9468_IIN_DONE_DFT	500000		/* uA */
/* parallel charging done conditoin */
#define PCA9468_IIN_P_DONE	1000000		/* uA */
/* Parallel charging default threshold */
#define PCA9468_IIN_P_TH_DFT	4000000		/* uA */
/* Single charging default threshold */
#define PCA9468_IIN_S_TH_DFT	10000000	/* uA */

/* Maximum TA voltage threshold */
#define PCA9468_TA_MAX_VOL		9800000 /* uV */
/* Maximum TA current threshold, set to max(cc_max) / 2 */
#define PCA9468_TA_MAX_CUR		2600000	 /* uA */
/* Minimum TA current threshold */
#define PCA9468_TA_MIN_CUR		1000000	/* uA - PPS minimum current */

/* Minimum TA voltage threshold in Preset mode */
#define PCA9468_TA_MIN_VOL_PRESET	8000000	/* uV */
/* TA voltage threshold starting Adjust CC mode */
#define PCA9468_TA_MIN_VOL_CCADJ	8500000	/* 8000000uV --> 8500000uV */

#define PCA9468_TA_VOL_PRE_OFFSET	500000	 /* uV */
/* Adjust CC mode TA voltage step */
#define PCA9468_TA_VOL_STEP_ADJ_CC	40000	/* uV */
/* Pre CV mode TA voltage step */
#define PCA9468_TA_VOL_STEP_PRE_CV	20000	/* uV */

/* IIN_CC adc offset for accuracy */
#define PCA9468_IIN_ADC_OFFSET		20000	/* uA */
/* IIN_CC compensation offset */
#define PCA9468_IIN_CC_COMP_OFFSET	25000	/* uA */
/* IIN_CC compensation offset in Power Limit Mode(Constant Power) TA */
#define PCA9468_IIN_CC_COMP_OFFSET_CP	20000	/* uA */
/* TA maximum voltage that can support CC in Constant Power Mode */
#define PCA9468_TA_MAX_VOL_CP		9800000	/* 9760000uV --> 9800000uV */
/* Offset for cc_max / 2 */
#define PCA9468_IIN_MAX_OFFSET		0
/* Offset for TA max current */
#define PCA9468_TA_CUR_MAX_OFFSET	200000 /* uA */


/* maximum retry counter for restarting charging */
#define PCA9468_MAX_RETRY_CNT		3	/* retries */
/* TA IIN tolerance */
#define PCA9468_TA_IIN_OFFSET		100000	/* uA */
/* IIN_CC upper protection offset in Power Limit Mode TA */
#define PCA9468_IIN_CC_UPPER_OFFSET	50000	/* 50mA */

/* PD Message Voltage and Current Step */
#define PD_MSG_TA_VOL_STEP		20000	/* uV */
#define PD_MSG_TA_CUR_STEP		50000	/* uA */

/* Maximum WCRX voltage threshold */
#define PCA9468_WCRX_MAX_VOL		9750000 /* uV */
/* WCRX voltage Step */
#define WCRX_VOL_STEP			100000	/* uV */

#define PCA9468_OTV_MARGIN		12000	/* uV */

/* Spread Spectrum default settings */
#define PCA9468_SC_CLK_DITHER_RATE_DEF	0	/* 25kHz */
#define PCA9468_SC_CLK_DITHER_LIMIT_DEF	0xF	/* 10% */

#define PCA9468_TIER_SWITCH_DELTA	25000	/* uV */
#define PCA9468_TA_MAX_CUR_MULT_DEF	1	/* No WLC Cap Div */

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
	STS_MODE_LOOP_INACTIVE,
	STS_MODE_CHG_DONE,
	STS_MODE_VIN_UVLO,
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

/* IIN offset as the switching frequency in uA*/
static int iin_fsw_cfg[16] = { 9990, 10540, 11010, 11520, 12000, 12520, 12990,
			      13470, 5460, 6050, 6580, 7150, 7670, 8230, 8720,
			      9260};


/* ------------------------------------------------------------------------ */

/* ADC Read function, return uV or uA */
int pca9468_read_adc(const struct pca9468_charger *pca9468, u8 adc_ch)
{
	u8 reg_data[2];
	u16 raw_adc = 0;
	int conv_adc = -1;
	int ret;

	switch (adc_ch) {
	case ADCCH_VOUT:
		/* ~PCA9468_BIT_CH1_EN, PCA9468_REG_ADC_CFG, udelay(120) us */

		ret = regmap_bulk_read(pca9468->regmap, PCA9468_REG_STS_ADC_4,
				       reg_data, 2);
		if (ret < 0) {
			conv_adc = ret;
			goto error;
		}

		raw_adc = ((reg_data[1] & PCA9468_BIT_ADC_VOUT9_2) << 2) |
			  ((reg_data[0] & PCA9468_BIT_ADC_VOUT1_0) >> 6);
		conv_adc = raw_adc * VOUT_STEP;	/* unit - uV */
		break;

	case ADCCH_VIN:
		/* ~PCA9468_BIT_CH2_EN, PCA9468_REG_ADC_CFG, udelay(120) us */

		ret = regmap_bulk_read(pca9468->regmap, PCA9468_REG_STS_ADC_3,
				       reg_data, 2);
		if (ret < 0) {
			conv_adc = ret;
			goto error;
		}

		raw_adc = ((reg_data[1] & PCA9468_BIT_ADC_VIN9_4) << 4) |
			  ((reg_data[0] & PCA9468_BIT_ADC_VIN3_0) >> 4);
		conv_adc = raw_adc * VIN_STEP;	/* unit - uV */
		break;

	case ADCCH_VBAT:
		/* ~PCA9468_BIT_CH3_EN, PCA9468_REG_ADC_CFG, udelay(120) us */

		ret = regmap_bulk_read(pca9468->regmap, PCA9468_REG_STS_ADC_6,
				       reg_data, 2);
		if (ret < 0) {
			conv_adc = ret;
			goto error;
		}

		raw_adc = ((reg_data[1] & PCA9468_BIT_ADC_VBAT9_8) << 8) |
			  ((reg_data[0] & PCA9468_BIT_ADC_VBAT7_0) >> 0);
		conv_adc = raw_adc * VBAT_STEP; /* unit - uV */
		break;

	case ADCCH_IIN:
		/* ~PCA9468_BIT_CH5_EN, PCA9468_REG_ADC_CFG, udelay(120) us */

		ret = regmap_bulk_read(pca9468->regmap, PCA9468_REG_STS_ADC_1,
				       reg_data, 2);
		if (ret < 0) {
			conv_adc = ret;
			goto error;
		}

		raw_adc = ((reg_data[1] & PCA9468_BIT_ADC_IIN9_8) << 8) |
			  ((reg_data[0] & PCA9468_BIT_ADC_IIN7_0) >> 0);

		/*
		 * iin = rawadc*4.89 + (rawadc*4.89 - 900) *
		 * 	 adc_comp_gain/100
		 */
		conv_adc = raw_adc * IIN_STEP + (raw_adc * IIN_STEP -
			   ADC_IIN_OFFSET) * pca9468->adc_comp_gain /
			   100; /* unit - uA */
		/*
		 * If ADC raw value is 0, convert value will be minus value
		 * because of compensation gain, so in this case conv_adc
		 * is 0
		 */
		if (conv_adc < 0)
			conv_adc = 0;
		break;

	case ADCCH_DIETEMP:
		/* ~PCA9468_BIT_CH6_EN, PCA9468_REG_ADC_CFG, udelay(120) us */

		ret = regmap_bulk_read(pca9468->regmap, PCA9468_REG_STS_ADC_7,
				       reg_data, 2);
		if (ret < 0) {
			conv_adc = ret;
			goto error;
		}

		raw_adc = ((reg_data[1] & PCA9468_BIT_ADC_DIETEMP9_6) << 6) |
			  ((reg_data[0] & PCA9468_BIT_ADC_DIETEMP5_0) >> 2);

		/* Temp = (935-rawadc)*0.435, unit - C */
		conv_adc = (935 - raw_adc) * DIETEMP_STEP / DIETEMP_DENOM;
		if (conv_adc > DIETEMP_MAX)
			conv_adc = DIETEMP_MAX;
		else if (conv_adc < DIETEMP_MIN)
			conv_adc = DIETEMP_MIN;
		break;

	case ADCCH_NTC:
		/* ~PCA9468_BIT_CH7_EN, PCA9468_REG_ADC_CFG, udelay(120) us */

		ret = regmap_bulk_read(pca9468->regmap, PCA9468_REG_STS_ADC_8,
				       reg_data, 2);
		if (ret < 0) {
			conv_adc = ret;
			goto error;
		}

		raw_adc = ((reg_data[1] & PCA9468_BIT_ADC_NTCV9_4) << 4) |
			  ((reg_data[0] & PCA9468_BIT_ADC_NTCV3_0) >> 4);

		/* Temp = (rawadc < 185)? (960-rawadc/4) : (730-rawadc/8) */
		/* unit: 0.1 degree C */
		if (raw_adc < NTC_CURVE_THRESHOLD)
			conv_adc = NTC_CURVE_1_BASE - ((raw_adc * 10) >> NTC_CURVE_1_SHIFT);
		else
			conv_adc = NTC_CURVE_2_BASE - ((raw_adc * 10) >> NTC_CURVE_2_SHIFT);
		break;

	default:
		conv_adc = -EINVAL;
		break;
	}

error:
	/* if disabled a channel, re-enable it in -> PCA9468_REG_ADC_CFG */

	pr_debug("%s: adc_ch=%u, raw_adc=%x convert_val=%d\n", __func__,
		 adc_ch, raw_adc, conv_adc);

	return conv_adc;
}

/* v float voltage (5 mV) resolution */
static int pca9468_set_vfloat(struct pca9468_charger *pca9468,
			      unsigned int v_float)
{
	const int val = PCA9468_V_FLOAT(v_float);
	int ret;

	ret = regmap_write(pca9468->regmap, PCA9468_REG_V_FLOAT, val);

	dev_info(pca9468->dev, "%s: v_float=%u (%d)\n", __func__, v_float, ret);

	return ret;
}

static int pca9468_set_input_current(struct pca9468_charger *pca9468,
				     unsigned int iin)
{
	int ret, val;

	/* round-up and increase one step */
	iin = iin + PD_MSG_TA_CUR_STEP;
	val = PCA9468_IIN_CFG(iin);

	/* Set IIN_CFG to one step higher */
	val = val + 1;
	if (val > 0x32)
		val = 0x32; /* maximum value is 5A */

	ret = regmap_update_bits(pca9468->regmap, PCA9468_REG_IIN_CTRL,
				 PCA9468_BIT_IIN_CFG, val);

	dev_info(pca9468->dev, "%s: iin=%d real iin_cfg=%d (%d)\n", __func__,
		 iin, val * PCA9468_IIN_CFG_STEP, ret);

	return ret;
}

static inline bool pca9468_can_inc_ta_cur(struct pca9468_charger *pca9468)
{
	return pca9468->ta_cur + PD_MSG_TA_CUR_STEP < min(pca9468->ta_max_cur,
		pca9468->iin_cc + PCA9468_TA_CUR_MAX_OFFSET);
}

/* Returns the enable or disable value. into 1 or 0. */
static int pca9468_get_charging_enabled(struct pca9468_charger *pca9468)
{
	int ret, intval;
	unsigned int val;

	ret = regmap_read(pca9468->regmap, PCA9468_REG_START_CTRL, &val);
	if (ret < 0)
		return ret;

	intval = (val & PCA9468_BIT_STANDBY_EN) ? 0 : 1;

	return intval;
}


/* b/194346461 ramp down IIN */
static int pca9468_wlc_ramp_down_iin(struct pca9468_charger *pca9468,
				     struct power_supply *wlc_psy)
{
	const int ramp_down_step = PCA9468_IIN_CFG_STEP;
	int ret = 0, iin;

	if (!pca9468->wlc_ramp_out_iin)
		return 0;

	iin = pca9468_input_current_limit(pca9468);
	for ( ; iin >= PCA9468_IIN_CFG_MIN; iin -= ramp_down_step) {
		int iin_adc, wlc_iout = -1;

		iin_adc = pca9468_read_adc(pca9468, ADCCH_IIN);
		if (wlc_psy) {
			union power_supply_propval pro_val;

			ret = power_supply_get_property(wlc_psy,
					POWER_SUPPLY_PROP_ONLINE,
					&pro_val);
			if (ret < 0 || pro_val.intval != PPS_PSY_PROG_ONLINE)
				break;

			ret = power_supply_get_property(wlc_psy,
					POWER_SUPPLY_PROP_CURRENT_NOW,
					&pro_val);
			if (ret == 0)
				wlc_iout = pro_val.intval;
		}

		ret = pca9468_set_input_current(pca9468, iin);
		if (ret < 0) {
			pr_err("%s: ramp down iin=%d (%d)\n", __func__,
				iin, ret);
			break;
		}

		pr_debug("%s: iin_adc=%d, wlc_iout-%d ramp down iin=%d\n",
				__func__, iin_adc, wlc_iout, iin);
		msleep(pca9468->wlc_ramp_out_delay);
	}

	return ret;
}

/* b/194346461 ramp down VOUT */
#define WLC_VOUT_CFG_STEP	40000

/* the caller will set to vbatt * 4 */
static int pca9468_wlc_ramp_down_vout(struct pca9468_charger *pca9468,
				      struct power_supply *wlc_psy)
{
	const int ramp_down_step = WLC_VOUT_CFG_STEP;
	union power_supply_propval pro_val;
	int vout = 0, vout_target = pca9468->wlc_ramp_out_vout_target;
	int ret, vbatt;

	while (true) {
		vbatt = pca9468_read_adc(pca9468, ADCCH_VBAT);
		if (vbatt <= 0) {
			pr_err("%s: invalid vbatt %d\n", __func__, vbatt);
			break;
		}

		ret = power_supply_get_property(wlc_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW,
					        &pro_val);
		if (ret < 0) {
			pr_err("%s: invalid vout %d\n", __func__, ret);
			break;
		}

		if (!pca9468->wlc_ramp_out_vout_target)
			vout_target = vbatt * 4;

		if (!vout)
			vout = pro_val.intval;
		if (vout < vout_target) {
			pr_debug("%s: underflow vout=%d, vbatt=%d (target=%d)\n", __func__,
			         vout, vbatt, vout_target);
			return 0;
		}

		pro_val.intval = vout - ramp_down_step;

		pr_debug("%s: vbatt=%d, wlc_vout=%d->%d\n", __func__, vbatt,
			 vout, pro_val.intval);

		ret = power_supply_set_property(wlc_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW,
						&pro_val);
		if (ret < 0) {
			pr_err("%s: cannot set vout %d\n", __func__, ret);
			break;
		}

		msleep(pca9468->wlc_ramp_out_delay);
		vout = pro_val.intval;
	}

	return -EIO;
}

/* call holding mutex_lock(&pca9468->lock); */
static int pca9468_set_charging(struct pca9468_charger *pca9468, bool enable)
{
	const int ntc_protection_en = 0; /* TODO: DT option? */
	int ret, val;

	pr_debug("%s: enable=%d ta_type=%d\n", __func__,  enable, pca9468->ta_type);

	if (enable && pca9468_get_charging_enabled(pca9468) == enable) {
		pr_debug("%s: no op, already enabled\n", __func__);
		return 0;
	}

	/* might needs to disable NTC_PROTECTION_EN in all cases */

	if (enable) {
		/* Improve adc */
		val = 0x5B;
		ret = regmap_write(pca9468->regmap, PCA9468_REG_ADC_ACCESS, val);
		if (ret < 0)
			goto error;
		ret = regmap_update_bits(pca9468->regmap, PCA9468_REG_ADC_IMPROVE,
					 PCA9468_BIT_ADC_IIN_IMP, 0);
		if (ret < 0)
			goto error;

		/* For fixing input current error */
		/* Overwrite 0x00 in 0x41 register */
		val = 0x00;
		ret = regmap_write(pca9468->regmap, 0x41, val);
		if (ret < 0)
			goto error;
		/* Overwrite 0x01 in 0x43 register */
		val = 0x01;
		ret = regmap_write(pca9468->regmap, 0x43, val);
		if (ret < 0)
			goto error;
		/* Overwrite 0x00 in 0x4B register */
		val = 0x00;
		ret = regmap_write(pca9468->regmap, 0x4B, val);
		if (ret < 0)
			goto error;
		/* End for fixing input current error */

	} else {
		/* Disable NTC_PROTECTION_EN */
		ret = regmap_update_bits(pca9468->regmap, PCA9468_REG_TEMP_CTRL,
					 PCA9468_BIT_NTC_PROTECTION_EN, 0);
	}

	if (enable) {
		/* ENABLE PCA9468 */
		ret = regmap_update_bits(pca9468->regmap, PCA9468_REG_START_CTRL,
					 PCA9468_BIT_STANDBY_EN,
					 PCA9468_STANDBY_DONOT);
		if (ret < 0)
			goto error;


		/* Wait 50ms, first to keep the start-up sequence */
		mdelay(50);
		/* Wait 150ms */
		msleep(150);

		/* Improve ADC */
		ret = regmap_update_bits(pca9468->regmap,
					 PCA9468_REG_ADC_IMPROVE,
					 PCA9468_BIT_ADC_IIN_IMP,
					 PCA9468_BIT_ADC_IIN_IMP);
		if (ret  < 0)
			goto error;

		val = 0x00;
		ret = regmap_write(pca9468->regmap, PCA9468_REG_ADC_ACCESS,
				   val);

		/* Restore NTC_PROTECTION_EN */
		ret = regmap_update_bits(pca9468->regmap, PCA9468_REG_TEMP_CTRL,
					 PCA9468_BIT_NTC_PROTECTION_EN,
					 ntc_protection_en);
	} else {

		if (pca9468->ta_type == TA_TYPE_WIRELESS) {
			struct power_supply *wlc_psy;
			int ret;

			wlc_psy = pca9468_get_rx_psy(pca9468);
			if (wlc_psy) {
				ret = pca9468_wlc_ramp_down_iin(pca9468, wlc_psy);
				if (ret < 0)
					dev_err(pca9468->dev, "cannot ramp out iin (%d)\n", ret);

				ret = pca9468_wlc_ramp_down_vout(pca9468, wlc_psy);
				if (ret < 0)
					dev_err(pca9468->dev, "cannot ramp out vout (%d)\n", ret);
			}
		}

		/* turn off the PCA */
		ret = regmap_update_bits(pca9468->regmap, PCA9468_REG_START_CTRL,
					 PCA9468_BIT_STANDBY_EN,
					 PCA9468_STANDBY_FORCED);
		if (ret < 0)
			goto error;

		/* Wait 5ms to keep the shutdown sequence */
		mdelay(5);
	}

error:
	pr_debug("%s: End, ret=%d\n", __func__, ret);
	return ret;
}

static int pca9468_check_state(u8 val[8], struct pca9468_charger *pca9468, int loglevel)
{
	int ret;

	/* Dump register */
	ret = regmap_bulk_read(pca9468->regmap, PCA9468_REG_INT1,
			       &val[PCA9468_REG_INT1], 7);
	if (ret < 0)
		return ret;

	logbuffer_prlog(pca9468, loglevel,
			"%s: Error reg[1]=%#x,[2]=%#x,[3]=%#x,[4]=%#x,[5]=%#x,[6]=%#x,[7]=%#x",
			__func__, val[1], val[2], val[3], val[4], val[5], val[6], val[7]);

	return 0;
}

static void pca9468_dump_test_debug(const struct pca9468_charger *pca9468,
				    int loglevel)
{
	u8 test_val[16];
	int ret, vin, vout, vbat;

	/* Read test register for debugging */
	ret = regmap_bulk_read(pca9468->regmap, 0x40, test_val, 16);
	if (ret < 0) {
		logbuffer_prlog(pca9468, LOGLEVEL_ERR,
				"%s: cannot read test registers (%d)\n",
				__func__, ret);
	} else {
		logbuffer_prlog(pca9468, loglevel,
				"%s: Error reg[0x40]=%#x,[0x41]=%#x,[0x42]=%#x,[0x43]=%#x,[0x44]=%#x,[0x45]=%#x,[0x46]=%#x,[0x47]=%#x",
				__func__, test_val[0], test_val[1], test_val[2], test_val[3],
				test_val[4], test_val[5], test_val[6], test_val[7]);
		logbuffer_prlog(pca9468, loglevel,
				"%s: Error reg[0x48]=%#x,[0x49]=%#x,[0x4A]=%#x,[0x4B]=%#x,[0x4C]=%#x,[0x4D]=%#x,[0x4E]=%#x,[0x4F]=%#x",
				__func__, test_val[8], test_val[9], test_val[10], test_val[11],
				test_val[12], test_val[13], test_val[14], test_val[15]);
	}

	vin = pca9468_read_adc(pca9468, ADCCH_VIN);
	vout = pca9468_read_adc(pca9468, ADCCH_VOUT);
	vbat = pca9468_read_adc(pca9468, ADCCH_VBAT);
	logbuffer_prlog(pca9468, loglevel, "%s: vin: %d, vout: %d, vbat: %d\n",
			__func__, vin, vout, vbat);
}

static void pca9468_dump_config(const struct pca9468_charger *pca9468,
				int loglevel)
{
	u8 val[10];
	int ret;

	ret = regmap_bulk_read(pca9468->regmap, PCA9468_REG_IIN_CTRL,
			       val, sizeof(val));
	if (ret < 0) {
		logbuffer_prlog(pca9468, LOGLEVEL_ERR,
				"%s: cannot read config (%d)\n", __func__, ret);
	} else {
		logbuffer_prlog(pca9468, loglevel,
				"%s: Config reg[0x21]=%#x,[0x22]=%#x,[0x23]=%#x,[0x24]=%#x,[0x25]=%#x,[0x26]=%#x,[0x27]=%#x,[0x28]=%#x,[0x29]=%#x,[0x2A]=%#x",
				__func__, val[0], val[1], val[2], val[3], val[4],
				val[5], val[6], val[7], val[8], val[9]);
	}
}

/* PCA9468 is not active state  - standby or shutdown */
/* Stop charging in timer_work */
/* return 0 when no error is detected */
static int pca9468_check_not_active(struct pca9468_charger *pca9468)
{
	u8 val[8];
	int ret;

	ret = pca9468_check_state(val, pca9468, LOGLEVEL_WARNING);
	if (ret < 0) {
		pr_err("%s: cannot read state\n", __func__);
		return ret;
	}

	pca9468_dump_test_debug(pca9468, LOGLEVEL_ERR);

	/* Check INT1_STS first */
	if ((val[PCA9468_REG_INT1_STS] & PCA9468_BIT_V_OK_STS) != PCA9468_BIT_V_OK_STS) {
		/* VBUS is invalid */
		logbuffer_prlog(pca9468, LOGLEVEL_ERR,
				"%s: VOK is invalid", __func__);

		/* Check STS_A. NOTE: V_OV_TRACKING is with VIN OV */
		if (val[PCA9468_REG_STS_A] & PCA9468_BIT_CFLY_SHORT_STS)
			logbuffer_prlog(pca9468, LOGLEVEL_ERR,
					"%s: Flying Cap is shorted to GND", __func__);
		else if (val[PCA9468_REG_STS_A] & PCA9468_BIT_VOUT_UV_STS)
			logbuffer_prlog(pca9468, LOGLEVEL_ERR,
					"%s: VOUT UV", __func__); /* VOUT < VOUT_OK */
		else if (val[PCA9468_REG_STS_A] & PCA9468_BIT_VBAT_OV_STS)
			logbuffer_prlog(pca9468, LOGLEVEL_ERR,
					"%s: VBAT OV", __func__); /* VBAT > VBAT_OV */
		else if (val[PCA9468_REG_STS_A] & PCA9468_BIT_VIN_OV_STS)
			logbuffer_prlog(pca9468, LOGLEVEL_ERR,
					"%s: VIN OV", __func__); /* VIN > V_OV_FIXED */
		else if (val[PCA9468_REG_STS_A] & PCA9468_BIT_VIN_UV_STS)
			logbuffer_prlog(pca9468, LOGLEVEL_ERR,
					"%s: VIN UV", __func__); /* VIN < V_UVTH */
		else
			logbuffer_prlog(pca9468, LOGLEVEL_ERR,
					"%s: Invalid VIN or VOUT", __func__);

		return  -EINVAL;
	}

	if (val[PCA9468_REG_INT1_STS] & PCA9468_BIT_NTC_TEMP_STS) {
		int ntc_adc, ntc_th; /* NTC protection */
		u8 reg_data[2]; /* NTC threshold */

		ret = regmap_bulk_read(pca9468->regmap, PCA9468_REG_NTC_TH_1,
				       reg_data, sizeof(reg_data));
		if (ret < 0)
			return -EIO;

		ntc_th = ((reg_data[1] & PCA9468_BIT_NTC_THRESHOLD9_8) << 8) |
			 reg_data[0];	/* uV unit */

		/* Read NTC ADC */
		ntc_adc = pca9468_read_adc(pca9468, ADCCH_NTC);	/* uV unit */
		logbuffer_prlog(pca9468, LOGLEVEL_ERR,
				"%s: NTC Protection, NTC_TH=%d(uV), NTC_ADC=%d(uV)",
				__func__, ntc_th, ntc_adc);

		return -EINVAL;
	}

	if (val[PCA9468_REG_INT1_STS] & PCA9468_BIT_CTRL_LIMIT_STS) {
		/* OCP event happens */

		if (val[PCA9468_REG_STS_B] & PCA9468_BIT_OCP_FAST_STS)
			logbuffer_prlog(pca9468, LOGLEVEL_ERR,
					"%s: IIN is over OCP_FAST", __func__);
		else if (val[PCA9468_REG_STS_B] & PCA9468_BIT_OCP_AVG_STS)
			logbuffer_prlog(pca9468, LOGLEVEL_ERR,
					"%s: IIN is over OCP_AVG", __func__);
		else
			logbuffer_prlog(pca9468, LOGLEVEL_ERR,
					"%s: No Loop active", __func__);

		return -EINVAL;
	}

	if (val[PCA9468_REG_INT1_STS] & PCA9468_BIT_TEMP_REG_STS) {
		/* Over temperature protection */
		logbuffer_prlog(pca9468, LOGLEVEL_ERR,
				"%s: Device is in temperature regulation", __func__);
		return -EINVAL;
	}

	if (val[PCA9468_REG_INT1_STS] & PCA9468_BIT_TIMER_STS) {
		const u8 sts_b = val[PCA9468_REG_STS_B];

		if (sts_b & PCA9468_BIT_CHARGE_TIMER_STS)
			logbuffer_prlog(pca9468, LOGLEVEL_ERR,
					"%s: Charger timer is expired", __func__);
		else if (sts_b & PCA9468_BIT_WATCHDOG_TIMER_STS)
			logbuffer_prlog(pca9468, LOGLEVEL_ERR,
				"%s: Watchdog timer is expired", __func__);
		else
			logbuffer_prlog(pca9468, LOGLEVEL_ERR,
					"%s: Timer INT, but no timer STS", __func__);

		return -EINVAL;
	}

	if (val[PCA9468_REG_STS_A] & PCA9468_BIT_CFLY_SHORT_STS) {
		logbuffer_prlog(pca9468, LOGLEVEL_ERR,
				"%s: Flying Cap is shorted to GND", __func__);
		return -EINVAL;
	}

	return 0;
}

/* Keep the current charging state, check STS_B again */
/* return 0 if VIN is still present, -EAGAIN if needs to retry, -EINVAL oth */
static int pca9468_check_standby(struct pca9468_charger *pca9468)
{
	unsigned int reg_val;
	int ret;
	u8 val[8];

	/* re-read the state register */
	ret = regmap_read(pca9468->regmap, PCA9468_REG_STS_B, &reg_val);
	if (ret < 0)
		return -EIO;

	pr_debug("%s: RCP check, STS_B=%#x\n",	__func__, reg_val);

	/* RCP condition, but VIN is valid and the PCA is active */
	if (reg_val & PCA9468_BIT_ACTIVE_STATE_STS) {
		const int charging_state = pca9468->charging_state;

		/*
		 * Try again when called from pca9468_check_active_state().
		 * If VIN is increased, input current will increase over
		 * IIN_LOW level.
		 */
		logbuffer_prlog(pca9468, charging_state == DC_STATE_CHECK_ACTIVE ?
				LOGLEVEL_WARNING : LOGLEVEL_ERR,
				"%s: RCP triggered but VIN is valid, state=%d",
				__func__, charging_state);

		pca9468->chg_data.rcp_count++;
		return -EAGAIN;
	}

	/* re-read and dump state, debug registers */
	pca9468_check_state(val, pca9468, LOGLEVEL_INFO);
	ret = regmap_bulk_read(pca9468->regmap, 0x48, val, 3);
	logbuffer_prlog(pca9468, LOGLEVEL_ERR,
			"%s: Error reg[0x48]=%#x,[0x49]=%#x,[0x4a]=%#x",
			__func__, val[0], val[1], val[2]);
	pca9468_dump_config(pca9468, LOGLEVEL_INFO);

	/* Not in RCP state, retry only when DC is starting */
	if (reg_val & PCA9468_BIT_STANDBY_STATE_STS) {
		logbuffer_prlog(pca9468, LOGLEVEL_WARNING, "%s: device in standby", __func__);
		pca9468->chg_data.stby_count++;
		ret = -EAGAIN;
	}  else {
		logbuffer_prlog(pca9468, LOGLEVEL_ERR, "%s: device in shutdown", __func__);
		p9468_chg_stats_update_flags(&pca9468->chg_data, P9468_CHGS_F_SHDN);
		ret = -EINVAL;
	}

	return ret;
}

/*
 * Check Active status, 0 is active (or in RCP), <0 indicates a problem.
 * The function is called from different contexts/functions, errors are fatal
 * (i.e. stop charging) from all contexts except when this is called from
 * pca9468_check_active_state().
 *
 * Other contexts:
 * . pca9468_charge_adjust_ccmode
 * . pca9468_charge_ccmode
 * . pca9468_charge_start_cvmode
 * . pca9468_charge_cvmode
 * . pca9468_adjust_ta_voltage
 * . pca9468_adjust_rx_voltage
 * . pca9468_adjust_ta_current
 * call holding mutex_lock(&pca9468->lock)
 */
static int pca9468_check_error(struct pca9468_charger *pca9468)
{
	unsigned int reg_val;
	int ret;

	ret = regmap_read(pca9468->regmap, PCA9468_REG_STS_B, &reg_val);
	if (ret < 0)
		goto error;

	/* PCA9468 is active state */
	if (reg_val & PCA9468_BIT_ACTIVE_STATE_STS) {
		int vbatt;

		/* PCA9468 is charging */

		/* Check whether the battery voltage is over the minimum */
		vbatt = pca9468_read_adc(pca9468, ADCCH_VBAT);
		if (vbatt > PCA9468_DC_VBAT_MIN) {
			/* Normal charging battery level */
			/* Check temperature regulation loop */
			/* Read INT1_STS register */
			ret = regmap_read(pca9468->regmap, PCA9468_REG_INT1_STS,
					  &reg_val);
			if (ret < 0) {
				pr_err("%s: cannot read status (%d)\n", __func__, ret);
			} else if (reg_val & PCA9468_BIT_TEMP_REG_STS) {
				/* Over temperature protection */
				pr_err("%s: Device is in temperature regulation\n",
					__func__);
				ret = -EINVAL;
			}
		} else {
			/* Abnormal battery level */
			pr_err("%s: Error abnormal battery voltage=%d\n",
				__func__, vbatt);
			ret = -EINVAL;
		}

		pr_debug("%s: Active Status ok=%d (ret=%d)\n", __func__,
			 ret == 0, ret);
		return ret;
	}

	/* not in error but in standby or shutdown */

	ret = pca9468_check_not_active(pca9468);
	if (ret < 0) {
		/* There was an error, done... */
	} else if ((reg_val & PCA9468_BIT_STANDBY_STATE_STS) == 0) {
		/* PCA9468 is in shutdown state */
		pr_err("%s: PCA9468 is in shutdown\n", __func__);
		ret = -EINVAL;
	} else if (pca9468->charging_state == DC_STATE_NO_CHARGING) {
		/*
		 * Sometimes battery driver might call set_property function
		 * to stop charging during msleep. At this case, charging
		 * state would change DC_STATE_NO_CHARGING. PCA9468 should
		 * stop checking RCP condition and exit timer_work
		 */
		pr_err("%s: other driver forced stop\n", __func__);
		ret = -EINVAL;
	} else {

		/* Check the RCP condition, T_REVI_DET is 300ms */
		msleep(200);

		/*
		 * return 0 if VIN is still present, -EAGAIN if needs to retry,
		 * -EINVAL on error.
		 */
		ret = pca9468_check_standby(pca9468);
	}

error:
	pr_debug("%s: Not Active Status=%d\n", __func__, ret);
	return ret;
}

static int pca9468_get_iin(struct pca9468_charger *pca9468, int *iin)
{
	const int offset = iin_fsw_cfg[pca9468->pdata->fsw_cfg];
	int temp;

	temp = pca9468_read_adc(pca9468, ADCCH_IIN);
	if (temp < 0)
		return temp;

	if (temp < offset)
		temp = offset;

	*iin = (temp - offset) * 2;
	return 0;
}

static int pca9468_get_batt_info(struct pca9468_charger *pca9468, int info_type, int *info)
{
	union power_supply_propval val;
	enum power_supply_property psp;
	int ret;

	if (!pca9468->batt_psy)
		pca9468->batt_psy = power_supply_get_by_name("battery");
	if (!pca9468->batt_psy)
		return -EINVAL;

	if (info_type == BATT_CURRENT)
		psp = POWER_SUPPLY_PROP_CURRENT_NOW;
	else
		psp = POWER_SUPPLY_PROP_VOLTAGE_NOW;

	ret = power_supply_get_property(pca9468->batt_psy, psp, &val);
	if (ret == 0)
		*info = val.intval;

	return ret;
}

static int pca9468_get_ibatt(struct pca9468_charger *pca9468, int *info)
{
	return pca9468_get_batt_info(pca9468, BATT_CURRENT, info);
}

static void pca9468_prlog_state(struct pca9468_charger *pca9468, const char *fn)
{
	int rc, ibat, icn = -EINVAL, iin = -EINVAL;
	bool ovc_flag;

	rc = pca9468_get_ibatt(pca9468, &ibat);
	if (rc == 0)
		rc = pca9468_get_iin(pca9468, &icn);
	if (rc == 0)
		iin = pca9468_read_adc(pca9468, ADCCH_IIN);
	ovc_flag = ibat > pca9468->cc_max;
	if (ovc_flag)
		p9468_chg_stats_inc_ovcf(&pca9468->chg_data, ibat, pca9468->cc_max);;

	logbuffer_prlog(pca9468, ovc_flag ? LOGLEVEL_WARNING : LOGLEVEL_DEBUG,
			"%s: iin=%d, iin_cc=%d, icn=%d ibat=%d, cc_max=%d rc=%d",
			fn, iin, pca9468->iin_cc, icn, ibat, pca9468->cc_max, rc);
}

static int pca9468_read_status(struct pca9468_charger *pca9468)
{
	unsigned int reg_val;
	int ret;

	/* Read STS_A */
	ret = regmap_read(pca9468->regmap, PCA9468_REG_STS_A, &reg_val);
	if (ret < 0)
		return ret;

	if (reg_val & PCA9468_BIT_VIN_UV_STS) {
		ret = STS_MODE_VIN_UVLO;
	} else if (reg_val & PCA9468_BIT_IIN_LOOP_STS) {
		ret = STS_MODE_IIN_LOOP;
	} else if (reg_val & PCA9468_BIT_CHG_LOOP_STS) {
		ret = STS_MODE_CHG_LOOP; /* never */
	} else if (reg_val & PCA9468_BIT_VFLT_LOOP_STS) {
		ret = STS_MODE_VFLT_LOOP;
	} else {
		ret = STS_MODE_LOOP_INACTIVE; /* lower IIN or TA to enter CC? */
	}

	return ret;
}

static int pca9468_const_charge_voltage(struct pca9468_charger *pca9468);

static int pca9468_check_status(struct pca9468_charger *pca9468)
{
	int icn = -EINVAL, ibat = -EINVAL, vbat = -EINVAL;
	int rc, status;

	status = pca9468_read_status(pca9468);
	if (status < 0)
		goto error;

	rc = pca9468_get_iin(pca9468, &icn);
	if (rc == 0)
		rc = pca9468_get_batt_info(pca9468, BATT_CURRENT, &ibat);
	if (rc == 0)
		rc = pca9468_get_batt_info(pca9468, BATT_VOLTAGE, &vbat);

error:
	pr_debug("%s: status=%d icn:%d ibat:%d delta_c=%d, vbat:%d, fv:%d, cc_max:%d\n",
		 __func__, status, icn, ibat, icn - ibat, vbat,
		 pca9468->fv_uv, pca9468->cc_max);

	return status;
}

/* hold mutex_lock(&pca9468->lock); */
static int pca9468_recover_ta(struct pca9468_charger *pca9468)
{
	int ret;

	if (pca9468->ftm_mode)
		return 0;

	if (pca9468->ta_type == TA_TYPE_WIRELESS) {
		pca9468->ta_vol = 0; /* set to a value to change rx vol */
		ret = pca9468_send_rx_voltage(pca9468, MSG_REQUEST_FIXED_PDO);
	} else {
		/* TODO: recover TA to value before handoff, or use DT */
		pca9468->ta_vol = 9000000;
		pca9468->ta_cur = 2200000;
		pca9468->ta_objpos = 1; /* PDO1 - fixed 5V */
		ret = pca9468_send_pd_message(pca9468, MSG_REQUEST_FIXED_PDO);
	}

	/* will not be able to recover if TA is offline */
	if (ret < 0)
		pr_debug("%s: cannot recover TA (%d)\n", __func__, ret);

	return 0;
}

/* Stop Charging */
static int pca9468_stop_charging(struct pca9468_charger *pca9468)
{
	int ret = 0;

	/* mark the end with \n in logbuffer */
	logbuffer_prlog(pca9468, LOGLEVEL_DEBUG,
			"%s: pca9468->charging_state=%d ret=%d\n",
			__func__, pca9468->charging_state, ret);

	mutex_lock(&pca9468->lock);

	/* Check the current state */
	if (pca9468->charging_state == DC_STATE_NO_CHARGING)
		goto done;

	/* Stop Direct charging  */
	cancel_delayed_work(&pca9468->timer_work);
	cancel_delayed_work(&pca9468->pps_work);
	pca9468->timer_id = TIMER_ID_NONE;
	pca9468->timer_period = 0;

	/* Clear parameter */
	if (pca9468->charging_state != DC_STATE_ERROR)
		pca9468->charging_state = DC_STATE_NO_CHARGING;
	pca9468->ret_state = DC_STATE_NO_CHARGING;
	pca9468->prev_iin = 0;
	pca9468->prev_inc = INC_NONE;
	pca9468->chg_mode = CHG_NO_DC_MODE;

	/* restore to config */
	pca9468->pdata->iin_cfg = pca9468->pdata->iin_cfg_max;
	pca9468->pdata->v_float = pca9468->pdata->v_float_dt;

	/*
	 * Clear charging configuration
	 * TODO: use defaults when these are negative or zero at startup
	 * NOTE: cc_max is twice of IIN + headroom
	 */
	pca9468->cc_max = -1;
	pca9468->fv_uv = -1;

	/* Clear requests for new Vfloat and new IIN */
	pca9468->new_vfloat = 0;
	pca9468->new_iin = 0;

	/* used to start DC and during errors */
	pca9468->retry_cnt = 0;

	pca9468->prev_ta_cur = 0;
	pca9468->prev_ta_vol = 0;
	/* close stats */
	p9468_chg_stats_done(&pca9468->chg_data, pca9468);
	p9468_chg_stats_dump(pca9468);

	/* TODO: something here to prep TA for the switch */

	ret = pca9468_set_charging(pca9468, false);
	if (ret < 0) {
		pr_err("%s: Error-set_charging(main)\n", __func__);
		goto error;
	}

	/* stop charging and recover TA voltage */
	if (pca9468->mains_online == true)
		pca9468_recover_ta(pca9468);

	power_supply_changed(pca9468->mains);

done:
error:
	mutex_unlock(&pca9468->lock);
	__pm_relax(pca9468->monitor_wake_lock);
	pr_debug("%s: END, ret=%d\n", __func__, ret);
	return ret;
}

#define FCC_TOLERANCE_RATIO		99
#define FCC_POWER_INCREASE_THRESHOLD	99

/*
 * Compensate TA current for the target input current called from
 * pca9468_charge_ccmode() when loop becomes not active.
 *
 * pca9468_charge_ccmode() ->
 * 	-> pca9468_set_rx_voltage_comp()
 * 	-> pca9468_set_ta_voltage_comp()
 * 	-> pca9468_set_ta_current_comp2()
 *
 * NOTE: call holding mutex_lock(&pca9468->lock);
 */
static int pca9468_set_ta_current_comp(struct pca9468_charger *pca9468)
{
	const int iin_high = pca9468->iin_cc + pca9468->pdata->iin_cc_comp_offset;
	const int iin_low = pca9468->iin_cc - pca9468->pdata->iin_cc_comp_offset;
	int rc, ibat, icn = -EINVAL, iin = -EINVAL;
	bool ovc_flag;

	/* IIN = IBAT+SYSLOAD */
	rc = pca9468_get_ibatt(pca9468, &ibat);
	if (rc == 0)
		rc = pca9468_get_iin(pca9468, &icn);
	if (rc == 0)
		iin = pca9468_read_adc(pca9468, ADCCH_IIN);

	ovc_flag = ibat > pca9468->cc_max;
	if (ovc_flag)
		p9468_chg_stats_inc_ovcf(&pca9468->chg_data, ibat, pca9468->cc_max);

	logbuffer_prlog(pca9468, ovc_flag ? LOGLEVEL_WARNING : LOGLEVEL_DEBUG,
			"%s: iin=%d, iin_cc=[%d,%d,%d], icn=%d ibat=%d, cc_max=%d rc=%d prev_iin=%d",
			__func__, iin, iin_low, pca9468->iin_cc, iin_high,
			icn, ibat, pca9468->cc_max, rc,
			pca9468->prev_iin);
	if (iin < 0)
		return iin;

	/* Compare IIN ADC with target input current */
	if (iin > iin_high) {

		/* TA current is higher than the target input current */
		if (pca9468->ta_cur > pca9468->iin_cc) {
			/* TA current is over than IIN_CC */
			/* Decrease TA current (50mA) */
			pca9468->ta_cur = pca9468->ta_cur - PD_MSG_TA_CUR_STEP;
			logbuffer_prlog(pca9468, LOGLEVEL_DEBUG, "Cont1: ta_cur=%u",
					pca9468->ta_cur);

		/* TA current is already less than IIN_CC */
		/* Compara IIN_ADC with the previous IIN_ADC */
		} else if (iin < (pca9468->prev_iin - PCA9468_IIN_ADC_OFFSET)) {
			/* Assume that TA operation mode is CV mode */
			/* Decrease TA voltage (20mV) */
			pca9468->ta_vol = pca9468->ta_vol - PD_MSG_TA_VOL_STEP;
			logbuffer_prlog(pca9468, LOGLEVEL_DEBUG, "Cont2-1: ta_vol=%u",
					pca9468->ta_vol);
		} else {
			/* Assume TA operation mode is CL mode */
			/* Decrease TA current (50mA) */
			pca9468->ta_cur = pca9468->ta_cur - PD_MSG_TA_CUR_STEP;
			logbuffer_prlog(pca9468, LOGLEVEL_DEBUG, "Cont2-2: ta_cur=%u",
					pca9468->ta_cur);
		}

		/* Send PD Message */
		pca9468->timer_id = TIMER_PDMSG_SEND;
		pca9468->timer_period = 0;

	} else if (iin < iin_low) {

		/* compare IIN ADC with previous IIN ADC + 20mA */
		if (iin > (pca9468->prev_iin + PCA9468_IIN_ADC_OFFSET)) {
			/*
			 * TA voltage is not enough to supply the operating
			 * current of RDO: increase TA voltage
			 */

			/* Compare TA max voltage */
			if (pca9468->ta_vol == pca9468->ta_max_vol) {
				/* TA voltage is already the maximum voltage */
				/* Compare TA max current */
				if (!pca9468_can_inc_ta_cur(pca9468)) {
					/* TA voltage and current are at max */
					logbuffer_prlog(pca9468, LOGLEVEL_DEBUG,
							"End1: ta_vol=%u, ta_cur=%u",
							pca9468->ta_vol, pca9468->ta_cur);

					/* Set timer */
					pca9468->timer_id = TIMER_CHECK_CCMODE;
					pca9468->timer_period = PCA9468_CCMODE_CHECK1_T;
				} else {
					/* Increase TA current (50mA) */
					pca9468->ta_cur = pca9468->ta_cur + PD_MSG_TA_CUR_STEP;

					logbuffer_prlog(pca9468, LOGLEVEL_DEBUG,
							"Cont3: ta_cur=%u",
							pca9468->ta_cur);

					/* Send PD Message */
					pca9468->timer_id = TIMER_PDMSG_SEND;
					pca9468->timer_period = 0;

					/* Set TA increment flag */
					pca9468->prev_inc = INC_TA_CUR;
				}
			} else {
				/* Increase TA voltage (20mV) */
				pca9468->ta_vol = pca9468->ta_vol + PD_MSG_TA_VOL_STEP;
				logbuffer_prlog(pca9468, LOGLEVEL_DEBUG,
						"Cont4: ta_vol=%u", pca9468->ta_vol);

				/* Send PD Message */
				pca9468->timer_id = TIMER_PDMSG_SEND;
				pca9468->timer_period = 0;

				/* Set TA increment flag */
				pca9468->prev_inc = INC_TA_VOL;
			}

		/* TA current is lower than the target input current */
		/* Check the previous TA increment */
		} else if (pca9468->prev_inc == INC_TA_VOL) {
			/*
			 * The previous increment is TA voltage, but
			 * input current does not increase.
			 */

			/* Try to increase TA current */
			/* Compare TA max current */
			if (!pca9468_can_inc_ta_cur(pca9468)) {

				/* TA current is already the maximum current */
				/* Compare TA max voltage */
				if (pca9468->ta_vol == pca9468->ta_max_vol) {
					/*
					 * TA voltage and current are already
					 * the maximum values
					 */
					logbuffer_prlog(pca9468, LOGLEVEL_DEBUG,
							"End2: ta_vol=%u, ta_cur=%u",
							pca9468->ta_vol, pca9468->ta_cur);

					pca9468->timer_id = TIMER_CHECK_CCMODE;
					pca9468->timer_period = PCA9468_CCMODE_CHECK1_T;
				} else {
					/* Increase TA voltage (20mV) */
					pca9468->ta_vol = pca9468->ta_vol + PD_MSG_TA_VOL_STEP;
					logbuffer_prlog(pca9468, LOGLEVEL_DEBUG,
							"Cont5: ta_vol=%u",
							pca9468->ta_vol);

					/* Send PD Message */
					pca9468->timer_id = TIMER_PDMSG_SEND;
					pca9468->timer_period = 0;

					/* Set TA increment flag */
					pca9468->prev_inc = INC_TA_VOL;
				}
			} else {
				const unsigned int ta_cur = pca9468->ta_cur +
							    PD_MSG_TA_CUR_STEP;

				/* Increase TA current (50mA) */
				logbuffer_prlog(pca9468, LOGLEVEL_DEBUG,
						"Cont6: ta_cur=%u->%u",
						pca9468->ta_cur, ta_cur);

				pca9468->ta_cur = pca9468->ta_cur + PD_MSG_TA_CUR_STEP;
				pca9468->timer_id = TIMER_PDMSG_SEND;
				pca9468->timer_period = 0;

				pca9468->prev_inc = INC_TA_CUR;
			}

		/*
		 * The previous increment was TA current, but input current
		 * did not increase. Try to increase TA voltage.
		 */
		} else if (pca9468->ta_vol == pca9468->ta_max_vol) {
			/* TA voltage is already the maximum voltage */

			/* Compare TA maximum current */
			if (!pca9468_can_inc_ta_cur(pca9468)) {
				/*
				* TA voltage and current are already at the
				 * maximum values
				 */
				logbuffer_prlog(pca9468, LOGLEVEL_DEBUG,
						"End3: ta_vol=%u, ta_cur=%u",
						 pca9468->ta_vol, pca9468->ta_cur);

				pca9468->timer_id = TIMER_CHECK_CCMODE;
				pca9468->timer_period = PCA9468_CCMODE_CHECK1_T;
			} else {
				/* Increase TA current (50mA) */
				pca9468->ta_cur = pca9468->ta_cur + PD_MSG_TA_CUR_STEP;
				logbuffer_prlog(pca9468, LOGLEVEL_DEBUG,
						"Cont7: ta_cur=%u", pca9468->ta_cur);

				/* Send PD Message */
				pca9468->timer_id = TIMER_PDMSG_SEND;
				pca9468->timer_period = 0;

				/* Set TA increment flag */
				pca9468->prev_inc = INC_TA_CUR;
			}
		} else {
			/* Increase TA voltage (20mV) */
			logbuffer_prlog(pca9468, LOGLEVEL_DEBUG,
					"Comp. Cont8: ta_vol=%u->%u",
					pca9468->ta_vol, pca9468->ta_vol + PD_MSG_TA_VOL_STEP);

			pca9468->ta_vol += PD_MSG_TA_VOL_STEP;

			/* Send PD Message */
			pca9468->timer_id = TIMER_PDMSG_SEND;
			pca9468->timer_period = 0;

			/* Set TA increment flag */
			pca9468->prev_inc = INC_TA_VOL;
		}

	} else {
		/* IIN ADC is in valid range */
		/* IIN_CC - 50mA < IIN ADC < IIN_CC + 50mA  */
		logbuffer_prlog(pca9468, LOGLEVEL_DEBUG,
				"Comp. End4(valid): ta_vol=%u, ta_cur=%u",
				pca9468->ta_vol, pca9468->ta_cur);
		/* Set timer */
		pca9468->timer_id = TIMER_CHECK_CCMODE;
		pca9468->timer_period = PCA9468_CCMODE_CHECK1_T;

		/* b/186969924: reset increment state on valid */
		pca9468->prev_inc = INC_NONE;
	}

	/* Save previous iin adc */
	pca9468->prev_iin = iin;
	return 0;
}

/*
 * max iin for 2:1 mode given cc_max and iin_cfg.
 * TODO: maybe use pdata->iin_cfg if cc_max is zero or negative.
 */
static int pca9468_get_iin_max(const struct pca9468_charger *pca9468, int cc_max)
{
	const int cc_limit = pca9468->pdata->iin_max_offset + cc_max / 2;
	int iin_max;

	iin_max = min(pca9468->pdata->iin_cfg_max, (unsigned int)cc_limit);

	pr_debug("%s: iin_max=%d iin_cfg=%u iin_cfg_max=%d cc_max=%d cc_limit=%d\n",
		 __func__, iin_max, pca9468->pdata->iin_cfg,
		 pca9468->pdata->iin_cfg_max, cc_max, cc_limit);

	return iin_max;
}

/* Compensate TA current for constant power mode */
/* hold mutex_lock(&pca9468->lock), schedule on return 0 */
static int pca9468_set_ta_current_comp2(struct pca9468_charger *pca9468)
{
	int rc, ibat, icn = -EINVAL, iin = -EINVAL;
	bool ovc_flag;

	/* IIN = IBAT+SYSLOAD */
	rc = pca9468_get_ibatt(pca9468, &ibat);
	if (rc == 0)
		rc = pca9468_get_iin(pca9468, &icn);
	if (rc == 0)
		iin = pca9468_read_adc(pca9468, ADCCH_IIN);

	ovc_flag = ibat > pca9468->cc_max;
	if (ovc_flag)
		p9468_chg_stats_inc_ovcf(&pca9468->chg_data, ibat, pca9468->cc_max);;

	logbuffer_prlog(pca9468, ovc_flag ? LOGLEVEL_WARNING : LOGLEVEL_DEBUG,
			"%s: iin=%d, iin_cc=[%d,%d,%d], iin_cfg=%d icn=%d ibat=%d, cc_max=%d rc=%d",
			__func__, iin,
			pca9468->iin_cc - PCA9468_IIN_CC_COMP_OFFSET_CP,
			pca9468->iin_cc,
			pca9468->iin_cc + PCA9468_IIN_CC_COMP_OFFSET_CP,
			pca9468->pdata->iin_cfg,
			icn, ibat, pca9468->cc_max, rc);
	if (iin < 0)
		return iin;

	/* Compare IIN ADC with target input current */
	if (iin > (pca9468->pdata->iin_cfg + pca9468->pdata->iin_cc_comp_offset)) {
		/* TA current is higher than the target input current limit */
		pca9468->ta_cur = pca9468->ta_cur - PD_MSG_TA_CUR_STEP;

		pca9468->timer_id = TIMER_PDMSG_SEND;
		pca9468->timer_period = 0;
	} else if (iin < (pca9468->iin_cc - PCA9468_IIN_CC_COMP_OFFSET_CP)) {

		/* TA current is lower than the target input current */
		/* IIN_ADC < IIN_CC -20mA */
		if (pca9468->ta_vol == pca9468->ta_max_vol) {
			const int iin_cc_lb = pca9468->iin_cc -
				      pca9468->pdata->iin_cc_comp_offset;

			/* Check IIN_ADC < IIN_CC - 50mA */
			if (iin < iin_cc_lb) {
				const unsigned int ta_max_vol =
				    pca9468->pdata->ta_max_vol * pca9468->chg_mode;
				unsigned int iin_apdo;
				unsigned int val;

				/* Set new IIN_CC to IIN_CC - 50mA */
				pca9468->iin_cc = pca9468->iin_cc -
					  pca9468->pdata->iin_cc_comp_offset;

				/* Set new TA_MAX_VOL to TA_MAX_PWR/IIN_CC */
				/* Adjust new IIN_CC with APDO resolution */
				iin_apdo = pca9468->iin_cc / PD_MSG_TA_CUR_STEP;
				iin_apdo = iin_apdo * PD_MSG_TA_CUR_STEP;
				/* in mV */
				val = pca9468->ta_max_pwr / (iin_apdo / pca9468->chg_mode / 1000);
				/* Adjust values with APDO resolution(20mV) */
				val = val * 1000 / PD_MSG_TA_VOL_STEP;
				val = val * PD_MSG_TA_VOL_STEP; /* uV */

				/* Set new TA_MAX_VOL */
				pca9468->ta_max_vol = min(val, ta_max_vol);

				/* Increase TA voltage(40mV) */
				pca9468->ta_vol = pca9468->ta_vol + PD_MSG_TA_VOL_STEP * 2;

				logbuffer_prlog(pca9468, LOGLEVEL_DEBUG,
						"Cont1: ta_vol=%u",
						pca9468->ta_vol);

				/* Send PD Message */
				pca9468->timer_id = TIMER_PDMSG_SEND;
				pca9468->timer_period = 0;
			} else {
				/* Wait for next current step compensation */
				/* IIN_CC - 50mA < IIN ADC < IIN_CC - 20mA */
				logbuffer_prlog(pca9468, LOGLEVEL_DEBUG,
						"Comp.(wait): ta_vol=%u",
						pca9468->ta_vol);

				/* Set timer */
				pca9468->timer_id = TIMER_CHECK_CCMODE;
				pca9468->timer_period = PCA9468_CCMODE_CHECK2_T;
			}
		} else {
			/* Increase TA voltage(40mV) */
			pca9468->ta_vol = pca9468->ta_vol + PD_MSG_TA_VOL_STEP * 2;
			if (pca9468->ta_vol > pca9468->ta_max_vol)
				pca9468->ta_vol = pca9468->ta_max_vol;

			logbuffer_prlog(pca9468, LOGLEVEL_DEBUG, "Cont2: ta_vol=%u",
					pca9468->ta_vol);

			/* Send PD Message */
			pca9468->timer_id = TIMER_PDMSG_SEND;
			pca9468->timer_period = 0;
		}
	} else {
		/* IIN ADC is in valid range */
		/* IIN_CC - 50mA < IIN ADC < IIN_CFG + 50mA */
		pr_debug("End(valid): ta_vol=%u\n", pca9468->ta_vol);

		pca9468->timer_id = TIMER_CHECK_CCMODE;
		pca9468->timer_period = PCA9468_CCMODE_CHECK2_T;

		/* b/186969924: reset increment state on valid */
		pca9468->prev_inc = INC_NONE;
	}

	/* Save previous iin adc */
	pca9468->prev_iin = iin;
	return 0;
}

/* Compensate TA voltage for the target input current */
/* hold mutex_lock(&pca9468->lock), schedule on return 0 */
static int pca9468_set_ta_voltage_comp(struct pca9468_charger *pca9468)
{
	const int iin_high = pca9468->iin_cc + pca9468->pdata->iin_cc_comp_offset;
	const int iin_low = pca9468->iin_cc - pca9468->pdata->iin_cc_comp_offset;
	const int ibat_limit = (pca9468->cc_max * FCC_POWER_INCREASE_THRESHOLD) / 100;
	int rc, ibat, icn = -EINVAL, iin = -EINVAL;
	bool ovc_flag;

	pr_debug("%s: ======START=======\n", __func__);
	pr_debug("%s: = charging_state=%u == \n", __func__,
		 pca9468->charging_state);

	/* IIN = IBAT+SYSLOAD */
	rc = pca9468_get_ibatt(pca9468, &ibat);
	if (rc == 0)
		rc = pca9468_get_iin(pca9468, &icn);
	if (rc == 0)
		iin = pca9468_read_adc(pca9468, ADCCH_IIN);

	ovc_flag = ibat > pca9468->cc_max;
	if (ovc_flag)
		p9468_chg_stats_inc_ovcf(&pca9468->chg_data, ibat, pca9468->cc_max);;

	logbuffer_prlog(pca9468, ovc_flag ? LOGLEVEL_WARNING : LOGLEVEL_DEBUG,
			"%s: iin=%d, iin_cc=[%d,%d,%d], icn=%d ibat=%d, cc_max=%d rc=%d",
			__func__, iin, iin_low, pca9468->iin_cc, iin_high,
			icn, ibat, pca9468->cc_max, rc);

	if (iin < 0)
		return iin;

	/* Compare IIN ADC with target input current */
	if (iin > iin_high) {
		/* TA current is higher than the target input current */
		/* Decrease TA voltage (20mV) */
		pca9468->ta_vol = pca9468->ta_vol - PD_MSG_TA_VOL_STEP;
		logbuffer_prlog(pca9468, LOGLEVEL_DEBUG, "Cont1: ta_vol=%u",
				pca9468->ta_vol);

		/* Send PD Message */
		pca9468->timer_id = TIMER_PDMSG_SEND;
		pca9468->timer_period = 0;

	} else if (iin < pca9468->iin_cc - pca9468->pdata->iin_cc_comp_offset) {

		/* TA current is lower than the target input current */
		/* Compare TA max voltage */
		if (pca9468->ta_vol == pca9468->ta_max_vol) {
			/* TA is already at maximum voltage */
			logbuffer_prlog(pca9468, LOGLEVEL_DEBUG,"End1(max TA vol): ta_vol=%u",
					pca9468->ta_vol);

			/* Set timer */
			/* Check the current charging state */
			if (pca9468->charging_state == DC_STATE_CC_MODE) {
				/* CC mode */
				pca9468->timer_id = TIMER_CHECK_CCMODE;
				pca9468->timer_period = PCA9468_CCMODE_CHECK1_T;
			} else {
				/* CV mode */
				pca9468->timer_id = TIMER_CHECK_CVMODE;
				pca9468->timer_period = PCA9468_CVMODE_CHECK_T;
			}
		} else {
			const unsigned ta_vol = pca9468->ta_vol;

			/* Increase TA voltage (20mV) */
			pca9468->ta_vol = pca9468->ta_vol + PD_MSG_TA_VOL_STEP;
			logbuffer_prlog(pca9468, LOGLEVEL_DEBUG, "Cont2: ta_vol:%u->%u",
					ta_vol, pca9468->ta_vol);

			/* Send PD Message */
			pca9468->timer_id = TIMER_PDMSG_SEND;
			pca9468->timer_period = 0;
		}
	} else {
		/* IIN ADC is in valid range */
		/* IIN_CC - 50mA < IIN ADC < IIN_CC + 50mA  */
		logbuffer_prlog(pca9468, LOGLEVEL_DEBUG,
				"End(valid): ta_vol=%u low_ibat=%d\n",
				pca9468->ta_vol, ibat < ibat_limit);

		/* Check the current charging state */
		if (pca9468->charging_state == DC_STATE_CC_MODE) {
			pca9468->timer_id = TIMER_CHECK_CCMODE;
			pca9468->timer_period = PCA9468_CCMODE_CHECK1_T;
		} else {
			pca9468->timer_id = TIMER_CHECK_CVMODE;
			pca9468->timer_period = PCA9468_CVMODE_CHECK_T;
		}
	}

	return 0;
}

/* hold mutex_lock(&pca9468->lock), schedule on return 0 */
static int pca9468_set_rx_voltage_comp(struct pca9468_charger *pca9468)
{
	int rc, ibat, icn = -EINVAL, iin = -EINVAL;
	bool ovc_flag;

	pr_debug("%s: ======START=======\n", __func__);

	rc = pca9468_get_ibatt(pca9468, &ibat);
	if (rc == 0)
		rc = pca9468_get_iin(pca9468, &icn);
	if (rc == 0)
		iin = pca9468_read_adc(pca9468, ADCCH_IIN);

	ovc_flag = ibat > pca9468->cc_max;
	if (ovc_flag)
		p9468_chg_stats_inc_ovcf(&pca9468->chg_data, ibat, pca9468->cc_max);;

	logbuffer_prlog(pca9468, ovc_flag ? LOGLEVEL_WARNING : LOGLEVEL_DEBUG,
			"%s: iin=%d, iin_cc=[%d,%d,%d], icn=%d ibat=%d, cc_max=%d rc=%d",
			__func__, iin,
			pca9468->iin_cc - pca9468->pdata->iin_cc_comp_offset,
			pca9468->iin_cc,
			pca9468->iin_cc + pca9468->pdata->iin_cc_comp_offset,
			icn, ibat, pca9468->cc_max, rc);
	if (iin < 0)
		return iin;

	/* Compare IIN ADC with target input current */
	if (iin > (pca9468->iin_cc + pca9468->pdata->iin_cc_comp_offset)) {

		/* RX current is higher than the target input current */
		pca9468->ta_vol = pca9468->ta_vol - WCRX_VOL_STEP;
		logbuffer_prlog(pca9468, LOGLEVEL_DEBUG, "Cont1: rx_vol=%u",
				pca9468->ta_vol);

		/* Set RX Voltage */
		pca9468->timer_id = TIMER_PDMSG_SEND;
		pca9468->timer_period = 0;

	} else if (iin < (pca9468->iin_cc - pca9468->pdata->iin_cc_comp_offset)) {

		/* RX current is lower than the target input current */
		/* Compare RX max voltage */
		if (pca9468->ta_vol == pca9468->ta_max_vol) {

			/* TA current is already the maximum voltage */
			logbuffer_prlog(pca9468, LOGLEVEL_DEBUG,
					"End1(max RX vol): rx_vol=%u",
					pca9468->ta_vol);

			/* Check the current charging state */
			if (pca9468->charging_state == DC_STATE_CC_MODE) {
				/* CC mode */
				pca9468->timer_id = TIMER_CHECK_CCMODE;
				pca9468->timer_period = PCA9468_CCMODE_CHECK1_T;
			} else {
				/* CV mode */
				pca9468->timer_id = TIMER_CHECK_CVMODE;
				pca9468->timer_period = PCA9468_CVMODE_CHECK_T;
			}
		} else {
			/* Increase RX voltage (100mV) */
			pca9468->ta_vol = pca9468->ta_vol + WCRX_VOL_STEP;
			logbuffer_prlog(pca9468, LOGLEVEL_DEBUG, "Cont2: rx_vol=%u",
					pca9468->ta_vol);

			/* Set RX Voltage */
			pca9468->timer_id = TIMER_PDMSG_SEND;
			pca9468->timer_period = 0;
		}
	} else {
		/* IIN ADC is in valid range */
		/* IIN_CC - 50mA < IIN ADC < IIN_CC + 50mA  */
		logbuffer_prlog(pca9468, LOGLEVEL_DEBUG, "End(valid): rx_vol=%u",
				pca9468->ta_vol);

		if (pca9468->charging_state == DC_STATE_CC_MODE) {
			pca9468->timer_id = TIMER_CHECK_CCMODE;
			pca9468->timer_period = PCA9468_CCMODE_CHECK1_T;
		} else {
			pca9468->timer_id = TIMER_CHECK_CVMODE;
			pca9468->timer_period = PCA9468_CVMODE_CHECK_T;
		}
	}

	return 0;
}

/*
 * iin limit for 2:1 for the adapter and chg_mode
 * Minimum between the confguration, cc_max (scaled with offset) and the
 * adapter capabilities.
 */
static int pca9468_get_iin_limit(const struct pca9468_charger *pca9468)
{
	int iin_cc;

	iin_cc = pca9468_get_iin_max(pca9468, pca9468->cc_max);
	if (pca9468->ta_max_cur * pca9468->chg_mode < iin_cc)
		iin_cc = pca9468->ta_max_cur * pca9468->chg_mode;

	pr_debug("%s: iin_cc=%d ta_max_cur=%u, chg_mode=%d\n", __func__,
		 iin_cc, pca9468->ta_max_cur, pca9468->chg_mode);

	return iin_cc;
}

/* recalculate ->ta_vol looking at demand (cc_max) */
static int pca9468_set_wireless_dc(struct pca9468_charger *pca9468, int vbat)
{
	unsigned long val;

	pca9468->iin_cc = pca9468_get_iin_limit(pca9468);

	/* RX_vol = MAX[(2*VBAT_ADC*CHG_mode + 500mV), 8.0V*CHG_mode] */
	pca9468->ta_vol = max(PCA9468_TA_MIN_VOL_PRESET * pca9468->chg_mode,
				(2 * vbat *  pca9468->chg_mode +
				PCA9468_TA_VOL_PRE_OFFSET));

	/* RX voltage resolution is 100mV */
	val = pca9468->ta_vol / WCRX_VOL_STEP;
	pca9468->ta_vol = val * WCRX_VOL_STEP;
	/* Set RX voltage to MIN[RX voltage, RX_MAX_VOL*chg_mode] */
	pca9468->ta_vol = min(pca9468->ta_vol, pca9468->ta_max_vol);

	/* ta_cur is ignored */
	logbuffer_prlog(pca9468, LOGLEVEL_DEBUG,
			"%s: iin_cc=%d, ta_vol=%d ta_max_vol=%d", __func__,
			pca9468->iin_cc, pca9468->ta_vol, pca9468->ta_max_vol);

	return 0;
}

/* recalculate ->ta_vol and ->ta_cur looking at demand (cc_max) */
static int pca9468_set_wired_dc(struct pca9468_charger *pca9468, int vbat)
{
	const unsigned long ta_max_vol = pca9468->pdata->ta_max_vol * pca9468->chg_mode;
	unsigned long val;
	int iin_cc;

	pca9468->iin_cc = pca9468_get_iin_limit(pca9468);

	/* Calculate new TA max voltage, current */
	val = pca9468->iin_cc / PD_MSG_TA_CUR_STEP;
	iin_cc = val * PD_MSG_TA_CUR_STEP;

	val = pca9468->ta_max_pwr / (iin_cc / pca9468->chg_mode  / 1000); /* mV */

	/* Adjust values with APDO resolution(20mV) */
	val = val * 1000 / PD_MSG_TA_VOL_STEP;
	val = val * PD_MSG_TA_VOL_STEP; /* uV */
	pca9468->ta_max_vol = min(val, ta_max_vol);

	/* MAX[8000mV * chg_mode, 2 * VBAT_ADC * chg_mode + 500 mV] */
	pca9468->ta_vol = max(PCA9468_TA_MIN_VOL_PRESET * pca9468->chg_mode,
			      2 * vbat * pca9468->chg_mode + PCA9468_TA_VOL_PRE_OFFSET);

	/* PPS voltage resolution is 20mV */
	val = pca9468->ta_vol / PD_MSG_TA_VOL_STEP;
	pca9468->ta_vol = val * PD_MSG_TA_VOL_STEP;
	pca9468->ta_vol = min(pca9468->ta_vol, pca9468->ta_max_vol);
	/* Set TA current to IIN_CC */
	pca9468->ta_cur = iin_cc / pca9468->chg_mode;

	logbuffer_prlog(pca9468, LOGLEVEL_DEBUG,
			"%s: iin_cc=%d, ta_vol=%d ta_cur=%d ta_max_vol=%d",
			__func__, pca9468->iin_cc, pca9468->ta_vol, pca9468->ta_cur,
			pca9468->ta_max_vol);

	return 0;
}

/*
 * like pca9468_preset_dcmode() but will not query the TA.
 * Called from timer:
 * [pca9468_charge_ccmode | pca9468_charge_cvmode] ->
 * 	pca9468_apply_new_iin() ->
 * 		pca9468_adjust_ta_current() ->
 * 			pca9468_reset_dcmode()
 * 	pca9468_apply_new_vfloat() ->
 * 		pca9468_reset_dcmode()
 *
 * NOTE: caller holds mutex_lock(&pca9468->lock);
 */
static int pca9468_reset_dcmode(struct pca9468_charger *pca9468)
{
	int ret = -EINVAL, vbat;

	pr_debug("%s: ======START=======\n", __func__);
	pr_debug("%s: = charging_state=%u == \n", __func__,
		 pca9468->charging_state);

	if (pca9468->cc_max < 0) {
		pr_err("%s: invalid cc_max=%d\n", __func__, pca9468->cc_max);
		goto error;
	}

	/*
	 * VBAT is over threshold but it might be "bouncy" due to transitory
	 * used to determine ta_vout.
	 */
	vbat = pca9468_read_adc(pca9468, ADCCH_VBAT);
	if (vbat < 0)
		return vbat;

	/* Check the TA type and set the charging mode */
	if (pca9468->ta_type == TA_TYPE_WIRELESS) {
		ret = pca9468_set_wireless_dc(pca9468, vbat);
	} else {
		ret = pca9468_set_wired_dc(pca9468, vbat);
	}

	/* Clear previous IIN ADC, TA increment flag */
	pca9468->prev_inc = INC_NONE;
	pca9468->prev_iin = 0;
error:
	pr_debug("%s: End, ret=%d\n", __func__, ret);
	return ret;
}

/*
 * The caller was triggered from pca9468_apply_new_iin(), return to the
 * calling CC or CV loop.
 * call holding mutex_unlock(&pca9468->lock);
 */
static void pca9468_return_to_loop(struct pca9468_charger *pca9468)
{
	switch (pca9468->ret_state) {
	case DC_STATE_CC_MODE:
		pca9468->timer_id = TIMER_CHECK_CCMODE;
		break;
	case DC_STATE_CV_MODE:
		pca9468->timer_id = TIMER_CHECK_CVMODE;
		break;
	default:
		dev_err(pca9468->dev, "%s: invalid ret_state=%u\n",
			__func__, pca9468->ret_state);
		return;
	}

	dev_info(pca9468->dev, "%s: charging_state=%u->%u\n", __func__,
		 pca9468->charging_state, pca9468->ret_state);

	pca9468->charging_state = pca9468->ret_state;
	pca9468->timer_period = 1000;
	pca9468->ret_state = 0;
	pca9468->new_iin = 0;
}

/*
 * Kicked from pca9468_apply_new_iin() when pca9468->new_iin!=0 and completed
 * off the timer. Never called on WLC_DC.
 * NOTE: Will return to the calling loop in ->ret_state
 */
static int pca9468_adjust_ta_current(struct pca9468_charger *pca9468)
{
	const int ta_limit = pca9468->iin_cc / pca9468->chg_mode;
	int rc, ibat, icn = -EINVAL, iin = -EINVAL;
	bool ovc_flag;
	int ret = 0;

	rc = pca9468_check_error(pca9468);
	if (rc != 0)
		return rc;

	rc = pca9468_get_ibatt(pca9468, &ibat);
	if (rc == 0)
		rc = pca9468_get_iin(pca9468, &icn);
	if (rc == 0)
		iin = pca9468_read_adc(pca9468, ADCCH_IIN);

	ovc_flag = ibat > pca9468->cc_max;
	if (ovc_flag)
		p9468_chg_stats_inc_ovcf(&pca9468->chg_data, ibat, pca9468->cc_max);;

	logbuffer_prlog(pca9468, ovc_flag ? LOGLEVEL_WARNING : LOGLEVEL_DEBUG,
			"%s: iin=%d, iin_cc=%d ta_limit=%d, iin_cfg=%d icn=%d ibat=%d, cc_max=%d rc=%d",
			__func__, iin, pca9468->iin_cc, ta_limit, pca9468->pdata->iin_cfg,
			icn, ibat, pca9468->cc_max, rc);

	if (pca9468->charging_state != DC_STATE_ADJUST_TACUR)
		dev_info(pca9468->dev, "%s: charging_state=%u->%u\n", __func__,
			 pca9468->charging_state, DC_STATE_ADJUST_TACUR);

	pca9468->charging_state = DC_STATE_ADJUST_TACUR;

	if (pca9468->ta_cur == ta_limit) {

		logbuffer_prlog(pca9468, LOGLEVEL_DEBUG,
				"adj. End, ta_cur=%u, ta_vol=%u, iin_cc=%u, chg_mode=%u",
				pca9468->ta_cur, pca9468->ta_vol,
				pca9468->iin_cc, pca9468->chg_mode);

		/* "Recover" IIN_CC to the original value (new_iin) */
		pca9468->iin_cc = pca9468->new_iin;
		pca9468_return_to_loop(pca9468);

	} else if (pca9468->iin_cc > pca9468->pdata->iin_cfg) {
		const int old_iin_cfg = pca9468->pdata->iin_cfg;

		/* Raise iin_cfg to the new iin_cc value (why??!?!?) */
		pca9468->pdata->iin_cfg = pca9468->iin_cc;

		ret = pca9468_set_input_current(pca9468, pca9468->iin_cc);
		if (ret == 0)
			ret = pca9468_reset_dcmode(pca9468);
		if (ret < 0)
			goto error;

		logbuffer_prlog(pca9468, LOGLEVEL_DEBUG,
				"New IIN, ta_max_vol=%u, ta_max_cur=%u, ta_max_pwr=%lu, iin_cc=%u, iin_cfg=%d->%d chg_mode=%u",
				pca9468->ta_max_vol, pca9468->ta_max_cur,
				pca9468->ta_max_pwr, pca9468->iin_cc,
				old_iin_cfg, pca9468->iin_cc,
				pca9468->chg_mode);

		pca9468->new_iin = 0;

		dev_info(pca9468->dev, "%s: charging_state=%u->%u\n", __func__,
			 pca9468->charging_state, DC_STATE_ADJUST_CC);

		/* Send PD Message and go to Adjust CC mode */
		pca9468->charging_state = DC_STATE_ADJUST_CC;
		pca9468->timer_id = TIMER_PDMSG_SEND;
		pca9468->timer_period = 0;
	} else {
		unsigned int val;

		/*
		 * Adjust IIN_CC with APDO resolution(50mA)
		 * pca9468->iin_cc will be reset to pca9468->new_iin when
		 * ->ta_cur reaches the ta_limit at the beginning of the
		 * function
		 */
		val = pca9468->iin_cc / PD_MSG_TA_CUR_STEP;
		pca9468->iin_cc = val * PD_MSG_TA_CUR_STEP;
		pca9468->ta_cur = pca9468->iin_cc / pca9468->chg_mode;

		logbuffer_prlog(pca9468, LOGLEVEL_DEBUG, "adjust iin=%u ta_cur=%d chg_mode=%d",
				pca9468->iin_cc, pca9468->ta_cur, pca9468->chg_mode);

		/* Send PD Message */
		pca9468->timer_id = TIMER_PDMSG_SEND;
		pca9468->timer_period = 0;
	}

	/* reschedule on ret == 0 */
error:
	return ret;
}

/* Kicked from apply_new_iin() then run off the timer
 * call holding mutex_lock(&pca9468->lock);
 */
static int pca9468_adjust_ta_voltage(struct pca9468_charger *pca9468)
{
	int rc, ibat, icn = -EINVAL, iin = -EINVAL;
	bool ovc_flag;

	if (pca9468->charging_state != DC_STATE_ADJUST_TAVOL)
		dev_info(pca9468->dev, "%s: charging_state=%u->%u\n", __func__,
			 pca9468->charging_state, DC_STATE_ADJUST_TAVOL);

	pca9468->charging_state = DC_STATE_ADJUST_TAVOL;

	rc = pca9468_check_error(pca9468);
	if (rc != 0)
		return rc;

	rc = pca9468_get_ibatt(pca9468, &ibat);
	if (rc == 0)
		rc = pca9468_get_iin(pca9468, &icn);
	if (rc == 0)
		iin = pca9468_read_adc(pca9468, ADCCH_IIN);

	ovc_flag = ibat > pca9468->cc_max;
	if (ovc_flag)
		p9468_chg_stats_inc_ovcf(&pca9468->chg_data, ibat, pca9468->cc_max);;

	logbuffer_prlog(pca9468, ovc_flag ? LOGLEVEL_WARNING : LOGLEVEL_DEBUG,
			"%s: iin=%d, iin_cc=[%d,%d,%d], icn=%d ibat=%d, cc_max=%d rc=%d",
			__func__, iin, pca9468->iin_cc - PD_MSG_TA_CUR_STEP,
			pca9468->iin_cc, pca9468->iin_cc + PD_MSG_TA_CUR_STEP,
			icn, ibat, pca9468->cc_max, rc);

	if (iin < 0)
		return iin;


	/* Compare IIN ADC with targer input current */
	if (iin > (pca9468->iin_cc + PD_MSG_TA_CUR_STEP)) {
		/* TA current is higher than the target input current */
		/* Decrease TA voltage (20mV) */
		pca9468->ta_vol = pca9468->ta_vol - PD_MSG_TA_VOL_STEP;

		logbuffer_prlog(pca9468, LOGLEVEL_DEBUG, "Cont1, ta_vol=%u",
				pca9468->ta_vol);

		/* Send PD Message */
		pca9468->timer_id = TIMER_PDMSG_SEND;
		pca9468->timer_period = 0;
	} else if (iin < (pca9468->iin_cc - PD_MSG_TA_CUR_STEP)) {
		/* TA current is lower than the target input current */

		if (pca9468_check_status(pca9468) == STS_MODE_VFLT_LOOP) {
			/* IIN current may not able to increase in CV */

			logbuffer_prlog(pca9468, LOGLEVEL_DEBUG,
					"End1-1, skip adjust for cv, ta_cur=%u, ta_vol=%u, iin_cc=%u, chg_mode=%u",
					pca9468->ta_cur, pca9468->ta_vol,
					pca9468->iin_cc, pca9468->chg_mode);

			pca9468_return_to_loop(pca9468);
		} else if (pca9468->ta_vol == pca9468->ta_max_vol) {
			/* TA TA voltage is already at the maximum voltage */

			logbuffer_prlog(pca9468, LOGLEVEL_DEBUG,
					"End1, ta_cur=%u, ta_vol=%u, iin_cc=%u, chg_mode=%u",
					pca9468->ta_cur, pca9468->ta_vol,
					pca9468->iin_cc, pca9468->chg_mode);

			pca9468_return_to_loop(pca9468);
		} else {
			/* Increase TA voltage (20mV) */
			pca9468->ta_vol = pca9468->ta_vol + PD_MSG_TA_VOL_STEP;

			logbuffer_prlog(pca9468, LOGLEVEL_DEBUG, "Cont2, ta_vol=%u",
					pca9468->ta_vol);

			/* Send PD Message */
			pca9468->timer_id = TIMER_PDMSG_SEND;
			pca9468->timer_period = 0;
		}
	} else {
		/* IIN ADC is in valid range */
		/* IIN_CC - 50mA < IIN ADC < IIN_CC + 50mA  */

		logbuffer_prlog(pca9468, LOGLEVEL_DEBUG,
				"End2, ta_cur=%u, ta_vol=%u, iin_cc=%u, chg_mode=%u",
				pca9468->ta_cur, pca9468->ta_vol,
			pca9468->iin_cc, pca9468->chg_mode);

		pca9468_return_to_loop(pca9468);
	}

	return 0;
}

/*
 * Kicked from apply_new_iin() then run off the timer
 * * NOTE: caller must hold mutex_lock(&pca9468->lock)
 */
static int pca9468_adjust_rx_voltage(struct pca9468_charger *pca9468)
{
	const int iin_high = pca9468->iin_cc + pca9468->pdata->iin_cc_comp_offset;
	const int iin_low = pca9468->iin_cc - pca9468->pdata->iin_cc_comp_offset;
	int rc, ibat, icn = -EINVAL, iin = -EINVAL;
	bool ovc_flag;

	if (pca9468->charging_state != DC_STATE_ADJUST_TAVOL)
		dev_info(pca9468->dev, "%s: charging_state=%u->%u\n", __func__,
			 pca9468->charging_state, DC_STATE_ADJUST_TAVOL);

	pca9468->charging_state = DC_STATE_ADJUST_TAVOL;

	rc = pca9468_check_error(pca9468);
	if (rc != 0)
		return rc;

	rc = pca9468_get_ibatt(pca9468, &ibat);
	if (rc == 0)
		rc = pca9468_get_iin(pca9468, &icn);
	if (rc == 0)
		iin = pca9468_read_adc(pca9468, ADCCH_IIN);

	ovc_flag = ibat > pca9468->cc_max;
	if (ovc_flag)
		p9468_chg_stats_inc_ovcf(&pca9468->chg_data, ibat, pca9468->cc_max);;

	logbuffer_prlog(pca9468, ovc_flag ? LOGLEVEL_WARNING : LOGLEVEL_DEBUG,
			"%s: iin=%d, iin_cc=[%d,%d,%d], icn=%d ibat=%d, cc_max=%d rc=%d",
			__func__, iin, iin_low, pca9468->iin_cc, iin_high,
			icn, ibat, pca9468->cc_max, rc);

	if (iin < 0)
		return iin;

	/* Compare IIN ADC with targer input current */
	if (iin > iin_high) {
		/* RX current is higher than the target input current */

		/* Decrease RX voltage (100mV) */
		pca9468->ta_vol = pca9468->ta_vol - WCRX_VOL_STEP;

		logbuffer_prlog(pca9468, LOGLEVEL_DEBUG, "Cont1, rx_vol=%u",
				pca9468->ta_vol);

		pca9468->timer_id = TIMER_PDMSG_SEND;
		pca9468->timer_period = 0;
	} else if (iin < iin_low) {
		/* RX current is lower than the target input current */

		if (pca9468_check_status(pca9468) == STS_MODE_VFLT_LOOP) {
			/* RX current may not able to increase in CV */
			logbuffer_prlog(pca9468, LOGLEVEL_DEBUG,
					"End1-1, skip adjust for cv, rx_vol=%u, iin_cc=%u",
					pca9468->ta_vol, pca9468->iin_cc);

			pca9468_return_to_loop(pca9468);
		} else if (pca9468->ta_vol == pca9468->ta_max_vol) {
			/* RX current is already the maximum voltage */
			logbuffer_prlog(pca9468, LOGLEVEL_DEBUG,
					"End1, rx_vol=%u, iin_cc=%u, chg_mode=%u",
					pca9468->ta_vol, pca9468->iin_cc,
					pca9468->chg_mode);

			/* Return charging state to the previous state */
			pca9468_return_to_loop(pca9468);
		} else {
			/* Increase RX voltage (100mV) */
			pca9468->ta_vol = pca9468->ta_vol + WCRX_VOL_STEP;

			logbuffer_prlog(pca9468, LOGLEVEL_DEBUG, "Cont2, rx_vol=%u",
					pca9468->ta_vol);

			/* Set RX voltage */
			pca9468->timer_id = TIMER_PDMSG_SEND;
			pca9468->timer_period = 0;
		}
	} else {
		/* IIN ADC is in valid range */

		logbuffer_prlog(pca9468, LOGLEVEL_DEBUG,
				"End2, rx_vol=%u, iin_cc=%u, chg_mode=%u",
				pca9468->ta_vol, pca9468->iin_cc,
				pca9468->chg_mode);

		/* Return charging state to the previous state */
		pca9468_return_to_loop(pca9468);
	}

	return 0;
}

/*
 * Called from CC and CV loops to set a new IIN (i.e. a new cc_max charging
 * current). Should also change the iin_cfg to avoid overcurrents.
 * NOTE: caller must hold mutex_lock(&pca9468->lock)
 */
static int pca9468_apply_new_iin(struct pca9468_charger *pca9468)
{
	int ret;

	logbuffer_prlog(pca9468, LOGLEVEL_INFO,
			"new_iin=%d (cc_max=%d), ta_type=%d charging_state=%d",
			pca9468->new_iin, pca9468->cc_max,
			pca9468->ta_type, pca9468->charging_state);

	/* iin_cfg is adjusted UP in pca9468_set_input_current() */
	ret = pca9468_set_input_current(pca9468, pca9468->new_iin);
	if (ret < 0)
		return ret;
	pca9468->pdata->iin_cfg = pca9468->new_iin;

	 /*
	  * ->ret_state is used to go back to the loop (CC or CV) that called
	  * this function.
	  */
	pca9468->ret_state = pca9468->charging_state;

	/*
	 * new_iin is used to trigger the process which might span one or more
	 * timer ticks the new_iin . The flag will be cleared once the target
	 * is reached.
	 */
	pca9468->iin_cc = pca9468->new_iin;
	if (pca9468->ta_type == TA_TYPE_WIRELESS) {
		ret = pca9468_adjust_rx_voltage(pca9468);
	} else if (pca9468->iin_cc < (PCA9468_TA_MIN_CUR * pca9468->chg_mode)) {
		/* TA current = PCA9468_TA_MIN_CUR(1.0A) */
		pca9468->ta_cur = PCA9468_TA_MIN_CUR;
		ret = pca9468_adjust_ta_voltage(pca9468);
	} else {
		ret = pca9468_adjust_ta_current(pca9468);
	}

	/* need reschedule on ret != 0 */

	pr_debug("%s: ret=%d\n", __func__, ret);
	return ret;
}

/*
 * also called from pca9468_set_new_cc_max()
 * call holding mutex_unlock(&pca9468->lock);
 */
static int pca9468_set_new_iin(struct pca9468_charger *pca9468, int iin)
{
	int ret = 0;

	if (iin < 0) {
		pr_debug("%s: ignore negative iin=%d\n", __func__, iin);
		return 0;
	}

	/* same as previous request nevermind */
	if (iin == pca9468->new_iin)
		return 0;

	pr_debug("%s: new_iin=%d->%d state=%d\n", __func__,
		 pca9468->new_iin, iin, pca9468->charging_state);

	/* apply iin_cc in pca9468_preset_config() at start */
	if (pca9468->charging_state == DC_STATE_NO_CHARGING ||
	    pca9468->charging_state == DC_STATE_CHECK_VBAT) {

		/* used on start vs the ->iin_cfg one */
		pca9468->pdata->iin_cfg = iin;
		pca9468->iin_cc = iin;
	} else if (pca9468->ret_state == 0) {
		/*
		 * pca9468_apply_new_iin() has not picked out the value yet
		 * and the value can be changed safely.
		 */
		pca9468->new_iin = iin;

		/* might want to tickle the loop now */
	} else {
		/* the caller must retry */
		ret = -EAGAIN;
	}

	pr_debug("%s: ret=%d\n", __func__, ret);
	return ret;
}

/*
 * The is no CC loop in this part: current must be controlled on TA side
 * adjusting output power. cc_max (the charging current) is scaled to iin
 *
 */
static int pca9468_set_new_cc_max(struct pca9468_charger *pca9468, int cc_max)
{
	const int prev_cc_max = pca9468->cc_max;
	int iin_max, ret = 0;

	if (cc_max < 0) {
		pr_debug("%s: ignore negative cc_max=%d\n", __func__, cc_max);
		return 0;
	}

	mutex_lock(&pca9468->lock);

	/* same as previous request nevermind */
	if (cc_max == pca9468->cc_max)
		goto done;

	/* iin will be capped by the adapter capabilities in reset_dcmode() */
	iin_max = pca9468_get_iin_max(pca9468, cc_max);
	if (iin_max <= 0) {
		pr_debug("%s: ignore negative iin_max=%d\n", __func__, iin_max);
		goto done;
	}

	if (pca9468->ta_max_cur && pca9468->ta_max_cur < iin_max)
		iin_max = pca9468->ta_max_cur;

	ret = pca9468_set_new_iin(pca9468, iin_max);
	if (ret == 0)
		pca9468->cc_max = cc_max;

	logbuffer_prlog(pca9468, LOGLEVEL_INFO,
			"%s: charging_state=%d cc_max=%d->%d iin_max=%d, ret=%d",
			__func__, pca9468->charging_state, prev_cc_max,
			cc_max, iin_max, ret);

done:
	pr_debug("%s: ret=%d\n", __func__, ret);
	mutex_unlock(&pca9468->lock);
	return ret;
}

/*
 * Apply pca9468->new_vfloat to the charging voltage.
 * Called from CC and CV loops, needs mutex_lock(&pca9468->lock)
 */
static int pca9468_apply_new_vfloat(struct pca9468_charger *pca9468)
{
	int ret = 0;

	if (pca9468->fv_uv == pca9468->new_vfloat)
		goto error_done;

	/* actually change the hardware */
	ret = pca9468_set_vfloat(pca9468, pca9468->new_vfloat);
	if (ret < 0)
		goto error_done;

	/* Restart the process if tier switch happened (either direction) */
	if (pca9468->charging_state == DC_STATE_CV_MODE &&
	    abs(pca9468->new_vfloat - pca9468->fv_uv) > PCA9468_TIER_SWITCH_DELTA) {
		ret = pca9468_reset_dcmode(pca9468);
		if (ret < 0) {
			pr_err("%s: cannot reset dcmode (%d)\n", __func__, ret);
		} else {
			dev_info(pca9468->dev, "%s: charging_state=%u->%u\n", __func__,
				pca9468->charging_state, DC_STATE_ADJUST_CC);

			pca9468->charging_state = DC_STATE_ADJUST_CC;
			pca9468->timer_id = TIMER_PDMSG_SEND;
			pca9468->timer_period = 0;
		}
	}

	pca9468->fv_uv = pca9468->new_vfloat;

error_done:
	logbuffer_prlog(pca9468, LOGLEVEL_INFO,
			"%s: new_vfloat=%d, ret=%d", __func__,
			pca9468->new_vfloat, ret);

	if (ret == 0)
		pca9468->new_vfloat = 0;

	return ret;
}

static int pca9468_set_new_vfloat(struct pca9468_charger *pca9468, int vfloat)
{
	int ret = 0;

	if (vfloat < 0) {
		pr_debug("%s: ignore negative vfloat %d\n", __func__, vfloat);
		return 0;
	}

	mutex_lock(&pca9468->lock);
	if (pca9468->new_vfloat == vfloat)
		goto done;

	/* use fv_uv at start in pca9468_preset_config() */
	if (pca9468->charging_state == DC_STATE_NO_CHARGING ||
	    pca9468->charging_state == DC_STATE_CHECK_VBAT) {
		pca9468->fv_uv = vfloat;
	} else {
		/* applied in pca9468_apply_new_vfloat() from CC or in CV loop */
		pca9468->new_vfloat = vfloat;
		pr_debug("%s: new_vfloat=%d\n", __func__, pca9468->new_vfloat);

		/* might want to tickle the cycle */
	}

done:
	mutex_unlock(&pca9468->lock);
	return ret;
}

/* called on loop inactive */
static void pca9468_adjust_ccmode_wireless(struct pca9468_charger *pca9468, int iin)
{
	/* IIN_ADC > IIN_CC -20mA ? */
	if (iin > (pca9468->iin_cc - PCA9468_IIN_ADC_OFFSET)) {
		/* Input current is already over IIN_CC */
		/* End RX voltage adjustment */

		dev_info(pca9468->dev, "%s: charging_state=%u->%u\n", __func__,
			 pca9468->charging_state, DC_STATE_CC_MODE);

		/* change charging state to CC mode */
		pca9468->charging_state = DC_STATE_CC_MODE;

		logbuffer_prlog(pca9468, LOGLEVEL_DEBUG, "End1: IIN_ADC=%d, rx_vol=%u",
				iin, pca9468->ta_vol);

		/* Clear TA increment flag */
		pca9468->prev_inc = INC_NONE;
		/* Go to CC mode */
		pca9468->timer_id = TIMER_CHECK_CCMODE;
		pca9468->timer_period = 0;

	/* Check RX voltage */
	} else if (pca9468->ta_vol == pca9468->ta_max_vol) {
		/* RX voltage is already max value */
		logbuffer_prlog(pca9468, LOGLEVEL_DEBUG,"End2: MAX value, rx_vol=%u max=%d",
				pca9468->ta_vol, pca9468->ta_max_vol);

		/* Clear TA increment flag */
		pca9468->prev_inc = INC_NONE;
		/* Go to CC mode */
		pca9468->timer_id = TIMER_CHECK_CCMODE;
		pca9468->timer_period = 0;
	} else {
		/* Try to increase RX voltage(100mV) */
		pca9468->ta_vol = pca9468->ta_vol + WCRX_VOL_STEP;
		if (pca9468->ta_vol > pca9468->ta_max_vol)
			pca9468->ta_vol = pca9468->ta_max_vol;

		logbuffer_prlog(pca9468, LOGLEVEL_DEBUG, "Cont: rx_vol=%u",
				pca9468->ta_vol);
		/* Set RX voltage */
		pca9468->timer_id = TIMER_PDMSG_SEND;
		pca9468->timer_period = 0;
	}
}

/* called on loop inactive */
static void pca9468_adjust_ccmode_wired(struct pca9468_charger *pca9468, int iin)
{

	/* USBPD TA is connected */
	if (iin > (pca9468->iin_cc - PCA9468_IIN_ADC_OFFSET)) {
		/* IIN_ADC > IIN_CC -20mA ? */
		/* Input current is already over IIN_CC */
		/* End TA voltage and current adjustment */

		dev_info(pca9468->dev, "%s: charging_state=%u->%u\n", __func__,
			 pca9468->charging_state, DC_STATE_CC_MODE);

		/* change charging state to CC mode */
		pca9468->charging_state = DC_STATE_CC_MODE;

		logbuffer_prlog(pca9468, LOGLEVEL_DEBUG,
				"End1: IIN_ADC=%d, ta_vol=%u, ta_cur=%u",
				iin, pca9468->ta_vol, pca9468->ta_cur);

		/* Clear TA increment flag */
		pca9468->prev_inc = INC_NONE;
		/* Go to CC mode */
		pca9468->timer_id = TIMER_CHECK_CCMODE;
		pca9468->timer_period = 0;

	/* Check TA voltage */
	} else if (pca9468->ta_vol == pca9468->ta_max_vol) {
		/* TA voltage is already max value */
		logbuffer_prlog(pca9468, LOGLEVEL_DEBUG,
				"End2: MAX value, ta_vol=%u, ta_cur=%u",
				pca9468->ta_vol, pca9468->ta_cur);

		/* Clear TA increment flag */
		pca9468->prev_inc = INC_NONE;
		/* Go to CC mode */
		pca9468->timer_id = TIMER_CHECK_CCMODE;
		pca9468->timer_period = 0;

		/* Check TA tolerance
		 * The current input current compares the final input
		 * current(IIN_CC) with 100mA offset PPS current tolerance
		 * has +/-150mA, so offset defined 100mA(tolerance +50mA)
		 */
	} else if (iin < (pca9468->iin_cc - PCA9468_TA_IIN_OFFSET)) {
		/*
		 * TA voltage too low to enter TA CC mode, so we
		 * should increase TA voltage
		 */
		pca9468->ta_vol = pca9468->ta_vol + PCA9468_TA_VOL_STEP_ADJ_CC *
					pca9468->chg_mode;

		if (pca9468->ta_vol > pca9468->ta_max_vol)
			pca9468->ta_vol = pca9468->ta_max_vol;

		logbuffer_prlog(pca9468, LOGLEVEL_DEBUG, "Cont1: ta_vol=%u",
				pca9468->ta_vol);

		/* Set TA increment flag */
		pca9468->prev_inc = INC_TA_VOL;
		/* Send PD Message */
		pca9468->timer_id = TIMER_PDMSG_SEND;
		pca9468->timer_period = 0;

	/* compare IIN ADC with previous IIN ADC + 20mA */
	} else if (iin > (pca9468->prev_iin + PCA9468_IIN_ADC_OFFSET)) {
		/* TA can supply more current if TA voltage is high */
		/* TA voltage too low for TA CC mode: increase it */
		pca9468->ta_vol = pca9468->ta_vol +
					PCA9468_TA_VOL_STEP_ADJ_CC *
					pca9468->chg_mode;
		if (pca9468->ta_vol > pca9468->ta_max_vol)
			pca9468->ta_vol = pca9468->ta_max_vol;

		logbuffer_prlog(pca9468, LOGLEVEL_DEBUG, "Cont2: ta_vol=%u",
				pca9468->ta_vol);
		/* Set TA increment flag */
		pca9468->prev_inc = INC_TA_VOL;

		/* Send PD Message */
		pca9468->timer_id = TIMER_PDMSG_SEND;
		pca9468->timer_period = 0;

	/* Check the previous increment */
	} else if (pca9468->prev_inc == INC_TA_CUR) {
		/*
		 * The previous increment is TA current, but input
		 * current does not increase. Try with voltage.
		 */

		pca9468->ta_vol = pca9468->ta_vol +
					PCA9468_TA_VOL_STEP_ADJ_CC *
					pca9468->chg_mode;
		if (pca9468->ta_vol > pca9468->ta_max_vol)
			pca9468->ta_vol = pca9468->ta_max_vol;

		logbuffer_prlog(pca9468, LOGLEVEL_DEBUG, "Cont3: ta_vol=%u",
				pca9468->ta_vol);

		pca9468->prev_inc = INC_TA_VOL;
		pca9468->timer_id = TIMER_PDMSG_SEND;
		pca9468->timer_period = 0;

		/*
		 * The previous increment is TA voltage, but input
		 * current does not increase
		 */

		/* Try to increase TA current */
		/* Check APDO max current */
	} else if (!pca9468_can_inc_ta_cur(pca9468)) {
		/* TA current is maximum current */

		logbuffer_prlog(pca9468, LOGLEVEL_DEBUG,
				"End(MAX_CUR): IIN_ADC=%d, ta_vol=%u, ta_cur=%u",
				iin, pca9468->ta_vol, pca9468->ta_cur);

		pca9468->prev_inc = INC_NONE;

		/* Go to CC mode */
		pca9468->timer_id = TIMER_CHECK_CCMODE;
		pca9468->timer_period = 0;
	} else {
		/* TA has tolerance and compensate it as real current */
		/* Increase TA current(50mA) */
		pca9468->ta_cur = pca9468->ta_cur + PD_MSG_TA_CUR_STEP;
		if (pca9468->ta_cur > pca9468->ta_max_cur)
			pca9468->ta_cur = pca9468->ta_max_cur;

		logbuffer_prlog(pca9468, LOGLEVEL_DEBUG, "Cont4: ta_cur=%u",
				pca9468->ta_cur);

		pca9468->prev_inc = INC_TA_CUR;
		pca9468->timer_id = TIMER_PDMSG_SEND;
		pca9468->timer_period = 0;
	}
}

static int pca9468_vote_dc_avail(struct pca9468_charger *pca9468, int vote, int enable)
{
	int ret = 0;

	if (!pca9468->dc_avail)
		pca9468->dc_avail = gvotable_election_get_handle(VOTABLE_DC_CHG_AVAIL);

	if (pca9468->dc_avail) {
		ret = gvotable_cast_int_vote(pca9468->dc_avail, REASON_DC_DRV, vote, enable);
		if (ret < 0)
			dev_err(pca9468->dev, "Unable to cast vote for DC Chg avail (%d)\n", ret);
	}

	logbuffer_prlog(pca9468, pca9468->charging_state == DC_STATE_ERROR ?
			LOGLEVEL_INFO : LOGLEVEL_DEBUG,
			"%s: Voting dc_avail when in error state", __func__);

	return ret;
}

/* 2:1 Direct Charging Adjust CC MODE control
 * called at the beginnig of CC mode charging. Will be followed by
 * pca9468_charge_ccmode with wich share some of the adjustments.
 */
static int pca9468_charge_adjust_ccmode(struct pca9468_charger *pca9468)
{
	int  iin, ccmode, vbatt, vin_vol;
	int ret = 0;

	mutex_lock(&pca9468->lock);

	pr_debug("%s: ======START=======\n", __func__);
	pca9468_prlog_state(pca9468, __func__);

	if (pca9468->charging_state != DC_STATE_ADJUST_CC)
		dev_info(pca9468->dev, "%s: charging_state=%u->%u\n", __func__,
			 pca9468->charging_state, DC_STATE_ADJUST_CC);

	pca9468->charging_state = DC_STATE_ADJUST_CC;

	ret = pca9468_check_error(pca9468);
	if (ret != 0)
		goto error; // This is not active mode.

	ccmode = pca9468_check_status(pca9468);
	if (ccmode < 0) {
		ret = ccmode;
		goto error;
	}

	switch(ccmode) {
	case STS_MODE_IIN_LOOP:
		pca9468->chg_data.iin_loop_count++;
		fallthrough;
	case STS_MODE_CHG_LOOP:	/* CHG_LOOP does't exist */
		if (pca9468->ta_type == TA_TYPE_WIRELESS) {
			/* Decrease RX voltage (100mV) */
			pca9468->ta_vol = pca9468->ta_vol - WCRX_VOL_STEP;
			logbuffer_prlog(pca9468, LOGLEVEL_DEBUG, "End1: rx_vol=%u",
					 pca9468->ta_vol);
		} else if (pca9468->ta_cur > PCA9468_TA_MIN_CUR) {
			/* TA current is higher than 1.0A */
			/* Decrease TA current (50mA) */
			pca9468->ta_cur = pca9468->ta_cur - PD_MSG_TA_CUR_STEP;

			logbuffer_prlog(pca9468, LOGLEVEL_DEBUG, "End2: ta_cur=%u, ta_vol=%u",
					pca9468->ta_cur, pca9468->ta_vol);
		} else {
			/* Decrease TA voltage (20mV) */
			pca9468->ta_vol = pca9468->ta_vol - PD_MSG_TA_VOL_STEP;

			logbuffer_prlog(pca9468, LOGLEVEL_DEBUG, "End3: ta_cur=%u, ta_vol=%u",
					pca9468->ta_cur, pca9468->ta_vol);
		}

		pca9468->prev_inc = INC_NONE;

		dev_info(pca9468->dev, "%s: charging_state=%u->%u\n", __func__,
			 pca9468->charging_state, DC_STATE_CC_MODE);

		/* Send PD Message and then go to CC mode */
		pca9468->charging_state = DC_STATE_CC_MODE;
		pca9468->timer_id = TIMER_PDMSG_SEND;
		pca9468->timer_period = 0;
		break;

	case STS_MODE_VFLT_LOOP:
		vbatt = pca9468_read_adc(pca9468, ADCCH_VBAT);

		logbuffer_prlog(pca9468, LOGLEVEL_DEBUG, "End4: vbatt=%d, ta_vol=%u",
				vbatt, pca9468->ta_vol);

		/* Clear TA increment flag */
		pca9468->prev_inc = INC_NONE;
		/* Go to Pre-CV mode */
		pca9468->timer_id = TIMER_ENTER_CVMODE;
		pca9468->timer_period = 0;
		break;

	case STS_MODE_LOOP_INACTIVE:

		iin = pca9468_read_adc(pca9468, ADCCH_IIN);
		logbuffer_prlog(pca9468, LOGLEVEL_DEBUG,
				"Inactive: iin=%d, iin_cc=%d, cc_max=%d",
				iin, pca9468->iin_cc, pca9468->cc_max);
		if (iin < 0)
			break;

		if (pca9468->ta_type == TA_TYPE_WIRELESS) {
			pca9468_adjust_ccmode_wireless(pca9468, iin);
		} else {
			pca9468_adjust_ccmode_wired(pca9468, iin);
		}

		pca9468->prev_iin = iin;
		break;

	case STS_MODE_VIN_UVLO:
		/* VIN UVLO - just notification , it works by hardware */
		vin_vol = pca9468_read_adc(pca9468, ADCCH_VIN);

		logbuffer_prlog(pca9468, LOGLEVEL_DEBUG, "VIN_UVLO: ta_vol=%u, vin_vol=%d",
				pca9468->ta_cur, vin_vol);

		/* Check VIN after 1sec */
		pca9468->timer_id = TIMER_ADJUST_CCMODE;
		pca9468->timer_period = 1000;
		break;

	default:
		goto error;
	}

	mod_delayed_work(pca9468->dc_wq, &pca9468->timer_work,
			 msecs_to_jiffies(pca9468->timer_period));
error:
	mutex_unlock(&pca9468->lock);
	pr_debug("%s: End, ret=%d\n", __func__, ret);
	return ret;
}

/* <0 error, 0 no new limits, >0 new limits */
static int pca9468_apply_new_limits(struct pca9468_charger *pca9468)
{
	int ret = 0;

	if (pca9468->new_iin && pca9468->new_iin < pca9468->iin_cc) {
		ret = pca9468_apply_new_iin(pca9468);
		if (ret == 0)
			ret = 1;
	} else if (pca9468->new_vfloat) {
		ret = pca9468_apply_new_vfloat(pca9468);
		if (ret == 0)
			ret = 1;
	} else if (pca9468->new_iin) {
		ret = pca9468_apply_new_iin(pca9468);
		if (ret == 0)
			ret = 1;
	} else {
		return 0;
	}

	return ret;
}

/* 2:1 Direct Charging CC MODE control */
static int pca9468_charge_ccmode(struct pca9468_charger *pca9468)
{
	int ccmode = -1, vin_vol, iin, ret = 0;

	pr_debug("%s: ======START======= \n", __func__);

	mutex_lock(&pca9468->lock);

	if (pca9468->charging_state != DC_STATE_CC_MODE)
		dev_info(pca9468->dev, "%s: charging_state=%u->%u\n", __func__,
			 pca9468->charging_state, DC_STATE_CC_MODE);

	pca9468->charging_state = DC_STATE_CC_MODE;

	pca9468_prlog_state(pca9468, __func__);

	ret = pca9468_check_error(pca9468);
	if (ret != 0)
		goto error_exit;

	/*
	 * A change in VFLOAT here means that we have busted the tier, a
	 * change in iin means that the thermal engine had changed cc_max.
	 * pca9468_apply_new_limits() changes pca9468->charging_state to
	 * DC_STATE_ADJUST_TAVOL or DC_STATE_ADJUST_TACUR when new limits
	 * need to be applied.
	 */
	ret = pca9468_apply_new_limits(pca9468);
	if (ret < 0)
		goto error_exit;
	if (ret > 0)
		goto done;

	ccmode = pca9468_check_status(pca9468);
	if (ccmode < 0) {
		ret = ccmode;
		goto error_exit;
	}

	switch(ccmode) {
	case STS_MODE_LOOP_INACTIVE:

		/* Set input current compensation */
		if (pca9468->ta_type == TA_TYPE_WIRELESS) {
			/* Need RX voltage compensation */
			ret = pca9468_set_rx_voltage_comp(pca9468);

			logbuffer_prlog(pca9468, LOGLEVEL_DEBUG, "INACTIVE1: rx_vol=%u",
					pca9468->ta_vol);
		} else {
			const int ta_max_vol = pca9468->ta_max_vol;

			/* Check TA current with TA_MIN_CUR */
			if (pca9468->ta_cur <= PCA9468_TA_MIN_CUR) {
				pca9468->ta_cur = PCA9468_TA_MIN_CUR;

				ret = pca9468_set_ta_voltage_comp(pca9468);
			} else if (ta_max_vol >= pca9468->pdata->ta_max_vol_cp) {
				ret = pca9468_set_ta_current_comp(pca9468);
			} else {
				/* constant power mode */
				ret = pca9468_set_ta_current_comp2(pca9468);
			}

			logbuffer_prlog(pca9468, LOGLEVEL_DEBUG,
					"INACTIVE2: ta_cur=%u, ta_vol=%u",
					pca9468->ta_cur,
					pca9468->ta_vol);
		}
		break;

	case STS_MODE_VFLT_LOOP:
		/* TODO: adjust fv_uv here based on real vbatt */

		iin = pca9468_read_adc(pca9468, ADCCH_IIN);
		logbuffer_prlog(pca9468, LOGLEVEL_DEBUG, "CC VFLOAT: iin=%d", iin);

		/* go to Pre-CV mode */
		pca9468->timer_id = TIMER_ENTER_CVMODE;
		pca9468->timer_period = 0;
		break;

	case STS_MODE_IIN_LOOP:
		pca9468->chg_data.iin_loop_count++;
		fallthrough;
	case STS_MODE_CHG_LOOP:
		iin = pca9468_read_adc(pca9468, ADCCH_IIN);
		if (iin < 0)
			break;

		if (pca9468->ta_type == TA_TYPE_WIRELESS) {
			/* Decrease RX voltage (100mV) */
			pca9468->ta_vol = pca9468->ta_vol - WCRX_VOL_STEP;
			logbuffer_prlog(pca9468, LOGLEVEL_DEBUG,
					"IIN_LOOP1: iin=%d, next_rx_vol=%u",
					iin, pca9468->ta_vol);
		} else if (pca9468->ta_cur <= PCA9468_TA_MIN_CUR) {
			/* Decrease TA voltage (20mV) */
			pca9468->ta_vol = pca9468->ta_vol - PD_MSG_TA_VOL_STEP;
			logbuffer_prlog(pca9468, LOGLEVEL_DEBUG,
					"IIN_LOOP2: iin=%d, next_ta_vol=%u",
					iin, pca9468->ta_vol);
		} else {
			/* Decrease TA current (50mA) */
			pca9468->ta_cur = pca9468->ta_cur - PD_MSG_TA_CUR_STEP;
			logbuffer_prlog(pca9468, LOGLEVEL_DEBUG,
					"IIN_LOOP3: iin=%d, next_ta_cur=%u",
					iin, pca9468->ta_cur);
		}

		/* Send PD Message */
		pca9468->timer_id = TIMER_PDMSG_SEND;
		pca9468->timer_period = 0;
		break;

	case STS_MODE_VIN_UVLO:
		/* VIN UVLO - just notification, it works by hardware */
		vin_vol = pca9468_read_adc(pca9468, ADCCH_VIN);

		logbuffer_prlog(pca9468, LOGLEVEL_DEBUG,
				"VIN_UVLO: ta_cur=%u ta_vol=%u, vin_vol=%d",
				pca9468->ta_cur, pca9468->ta_vol, vin_vol);

		/* Check VIN after 1sec */
		pca9468->timer_id = TIMER_CHECK_CCMODE;
		pca9468->timer_period = 1000;
		break;

	default:
		break;
	}

done:
	mod_delayed_work(pca9468->dc_wq, &pca9468->timer_work,
			 msecs_to_jiffies(pca9468->timer_period));

error_exit:
	mutex_unlock(&pca9468->lock);
	pr_debug("%s: End, ccmode=%d timer_id=%d, timer_period=%lu ret=%d\n",
		 __func__, ccmode, pca9468->timer_id, pca9468->timer_period,
		 ret);
	return ret;
}


/* 2:1 Direct Charging Start CV MODE control - Pre CV MODE */
static int pca9468_charge_start_cvmode(struct pca9468_charger *pca9468)
{
	int ret = 0;
	int cvmode;
	int vin_vol;

	pr_debug("%s: ======START=======\n", __func__);

	mutex_lock(&pca9468->lock);

	if (pca9468->charging_state != DC_STATE_START_CV)
		dev_info(pca9468->dev, "%s: charging_state=%u->%u\n", __func__,
			 pca9468->charging_state, DC_STATE_START_CV);

	pca9468->charging_state = DC_STATE_START_CV;

	/* Check the charging type */
	ret = pca9468_check_error(pca9468);
	if (ret != 0)
		goto error_exit;

	/* Check the status */
	cvmode = pca9468_check_status(pca9468);
	if (cvmode < 0) {
		ret = cvmode;
		goto error_exit;
	}

	switch(cvmode) {
	case STS_MODE_IIN_LOOP:
		pca9468->chg_data.iin_loop_count++;
		fallthrough;
	case STS_MODE_CHG_LOOP:

		if (pca9468->ta_type == TA_TYPE_WIRELESS) {
			/* Decrease RX voltage (100mV) */
			pca9468->ta_vol = pca9468->ta_vol - WCRX_VOL_STEP;
			logbuffer_prlog(pca9468, LOGLEVEL_DEBUG,
					"%s: PreCV IIN_LOOP: rx_vol=%u",
				 __func__, pca9468->ta_vol);
		} else {
			/* Check TA current */
			if (pca9468->ta_cur > PCA9468_TA_MIN_CUR) {
				/* TA current is higher than 1.0A */

				/* Decrease TA current (50mA) */
				pca9468->ta_cur = pca9468->ta_cur - PD_MSG_TA_CUR_STEP;
				logbuffer_prlog(pca9468, LOGLEVEL_DEBUG,
						"%s: PreCV IIN_LOOP: ta_cur=%u",
						__func__, pca9468->ta_cur);
			} else {
				/* TA current is less than 1.0A */
				/* Decrease TA voltage (20mV) */
				pca9468->ta_vol = pca9468->ta_vol - PD_MSG_TA_VOL_STEP;
				logbuffer_prlog(pca9468, LOGLEVEL_DEBUG,
						"%s: PreCV IIN_LOOP: ta_vol=%u",
						__func__, pca9468->ta_vol);
			}
		}

		/* Send PD Message */
		pca9468->timer_id = TIMER_PDMSG_SEND;
		pca9468->timer_period = 0;
		break;

	case STS_MODE_VFLT_LOOP:
		/* Check the TA type */
		if (pca9468->ta_type == TA_TYPE_WIRELESS) {
			/* Decrease RX voltage (100mV) */
			pca9468->ta_vol = pca9468->ta_vol - WCRX_VOL_STEP;
			logbuffer_prlog(pca9468, LOGLEVEL_DEBUG,
					"%s: PreCV VF Cont: rx_vol=%u",
					__func__, pca9468->ta_vol);
		} else {
			/* Decrease TA voltage (20mV) */
			pca9468->ta_vol = pca9468->ta_vol -
					  PCA9468_TA_VOL_STEP_PRE_CV *
					  pca9468->chg_mode;
			logbuffer_prlog(pca9468, LOGLEVEL_DEBUG,
					"%s: PreCV VF Cont: ta_vol=%u",
					__func__, pca9468->ta_vol);
		}

		/* Send PD Message */
		pca9468->timer_id = TIMER_PDMSG_SEND;
		pca9468->timer_period = 0;
		break;

	case STS_MODE_LOOP_INACTIVE:
		/* Exit Pre CV mode */
		logbuffer_prlog(pca9468, LOGLEVEL_DEBUG,
				"%s: PreCV End: ta_vol=%u, ta_cur=%u",
				__func__, pca9468->ta_vol, pca9468->ta_cur);

		/* Need to implement notification to other driver */
		/* To do here */

		/* Go to CV mode */
		pca9468->timer_id = TIMER_CHECK_CVMODE;
		pca9468->timer_period = 0;
		break;

	case STS_MODE_VIN_UVLO:
		/* VIN UVLO - just notification , it works by hardware */
		vin_vol = pca9468_read_adc(pca9468, ADCCH_VIN);

		logbuffer_prlog(pca9468, LOGLEVEL_DEBUG,
				"%s: PreCV VIN_UVLO: ta_vol=%u, vin_vol=%u",
				__func__, pca9468->ta_cur, vin_vol);

		/* Check VIN after 1sec */
		pca9468->timer_id = TIMER_ENTER_CVMODE;
		pca9468->timer_period = 1000;
		break;

	default:
		break;
	}

	mod_delayed_work(pca9468->dc_wq, &pca9468->timer_work,
			 msecs_to_jiffies(pca9468->timer_period));
error_exit:
	mutex_unlock(&pca9468->lock);
	pr_debug("%s: End, ret=%d\n", __func__, ret);
	return ret;
}

static int pca9468_check_eoc(struct pca9468_charger *pca9468)
{
	const int eoc_tolerance = 25000; /* 25mV under max float voltage */
	const int vlimit = PCA9468_COMP_VFLOAT_MAX - eoc_tolerance;
	int iin, vbat;

	iin = pca9468_read_adc(pca9468, ADCCH_IIN);
	if (iin < 0) {
		pr_err("%s: iin=%d\n", __func__, iin);
		return iin;
	}

	vbat = pca9468_read_adc(pca9468, ADCCH_VBAT);
	if (vbat < 0) {
		pr_err("%s: vbat=%d\n", __func__, vbat);
		return vbat;
	}

	pr_debug("%s: iin=%d, topoff=%u, vbat=%d vlimit=%d\n", __func__,
		 iin, pca9468->pdata->iin_topoff,
		 vbat, vlimit);

	return iin < pca9468->pdata->iin_topoff && vbat >= vlimit;
}

/* 2:1 Direct Charging CV MODE control */
static int pca9468_charge_cvmode(struct pca9468_charger *pca9468)
{
	int ret = 0;
	int cvmode;
	int vin_vol;

	pr_debug("%s: ======START=======\n", __func__);

	mutex_lock(&pca9468->lock);

	if (pca9468->charging_state != DC_STATE_CV_MODE)
		dev_info(pca9468->dev, "%s: charging_state=%u->%u\n", __func__,
			 pca9468->charging_state, DC_STATE_CV_MODE);

	pca9468->charging_state = DC_STATE_CV_MODE;

	ret = pca9468_check_error(pca9468);
	if (ret != 0)
		goto error_exit;

	/*
	 * A change in vfloat and cc_max here is a normal tier transition, a
	 * change in iin  means that the thermal engine has changed cc_max.
	 */
	ret = pca9468_apply_new_limits(pca9468);
	if (ret < 0)
		goto error_exit;
	if (ret > 0)
		goto done;

	cvmode = pca9468_check_status(pca9468);
	if (cvmode < 0) {
		ret = cvmode;
		goto error_exit;
	}

	if (cvmode == STS_MODE_LOOP_INACTIVE) {
		ret = pca9468_check_eoc(pca9468);
		if (ret < 0)
			goto error_exit;
		if (ret)
			cvmode = STS_MODE_CHG_DONE;
	}

	switch(cvmode) {
	case STS_MODE_CHG_DONE: {
		const bool done_already = pca9468->charging_state ==
					  DC_STATE_CHARGING_DONE;

		if (!done_already)
			dev_info(pca9468->dev, "%s: charging_state=%u->%u\n",
				 __func__, pca9468->charging_state,
				 DC_STATE_CHARGING_DONE);


		/* Keep CV mode until driver send stop charging */
		pca9468->charging_state = DC_STATE_CHARGING_DONE;
		power_supply_changed(pca9468->mains);

		/* _cpm already came in */
		if (pca9468->charging_state == DC_STATE_NO_CHARGING) {
			pr_debug("%s: Already stop DC\n", __func__);
			break;
		}

		logbuffer_prlog(pca9468, LOGLEVEL_DEBUG,
				"%s: done_already=%d charge Done\n", __func__,
				done_already);

		pca9468->timer_id = TIMER_CHECK_CVMODE;
		pca9468->timer_period = PCA9468_CVMODE_CHECK_T;
	} break;

	case STS_MODE_IIN_LOOP:
		pca9468->chg_data.iin_loop_count++;
		fallthrough;
	case STS_MODE_CHG_LOOP:
		/* Check the TA type */
		if (pca9468->ta_type == TA_TYPE_WIRELESS) {
			/* Decrease RX Voltage (100mV) */
			pca9468->ta_vol = pca9468->ta_vol -
						WCRX_VOL_STEP;
			logbuffer_prlog(pca9468, LOGLEVEL_DEBUG,
					"%s: CV LOOP, Cont: rx_vol=%u",
					__func__, pca9468->ta_vol);

		/* Check TA current */
		} else if (pca9468->ta_cur > PCA9468_TA_MIN_CUR) {
			/* TA current is higher than (1.0A*chg_mode) */
			/* Decrease TA current (50mA) */
			pca9468->ta_cur = pca9468->ta_cur -
						PD_MSG_TA_CUR_STEP;
			logbuffer_prlog(pca9468, LOGLEVEL_DEBUG,
					"%s: CV LOOP, Cont: ta_cur=%u",
					__func__, pca9468->ta_cur);
		} else {
			/* TA current is less than (1.0A*chg_mode) */
			/* Decrease TA Voltage (20mV) */
			pca9468->ta_vol = pca9468->ta_vol -
						PD_MSG_TA_VOL_STEP;
			logbuffer_prlog(pca9468, LOGLEVEL_DEBUG,
					"%s: CV LOOP, Cont: ta_vol=%u",
					__func__, pca9468->ta_vol);
		}

		/* Send PD Message */
		pca9468->timer_id = TIMER_PDMSG_SEND;
		pca9468->timer_period = 0;
		break;

	case STS_MODE_VFLT_LOOP:
		/* Check the TA type */
		if (pca9468->ta_type == TA_TYPE_WIRELESS) {
			/* Decrease RX voltage */
			pca9468->ta_vol = pca9468->ta_vol -
						WCRX_VOL_STEP;
			logbuffer_prlog(pca9468, LOGLEVEL_DEBUG,
					"%s: CV VFLOAT, Cont: rx_vol=%u",
					__func__, pca9468->ta_vol);
		} else {
			/* Decrease TA voltage */
			pca9468->ta_vol = pca9468->ta_vol -
						PD_MSG_TA_VOL_STEP;
			logbuffer_prlog(pca9468, LOGLEVEL_DEBUG,
					"%s: CV VFLOAT, Cont: ta_vol=%u",
					__func__, pca9468->ta_vol);
		}

		/* Send PD Message */
		pca9468->timer_id = TIMER_PDMSG_SEND;
		pca9468->timer_period = 0;
		break;

	case STS_MODE_LOOP_INACTIVE:
		pca9468->timer_id = TIMER_CHECK_CVMODE;
		pca9468->timer_period = PCA9468_CVMODE_CHECK_T;
		break;

	case STS_MODE_VIN_UVLO:
		/* VIN UVLO - just notification, it works by hardware */
		vin_vol = pca9468_read_adc(pca9468, ADCCH_VIN);
		logbuffer_prlog(pca9468, LOGLEVEL_DEBUG,
				"%s: CC VIN_UVLO: ta_cur=%u ta_vol=%u, vin_vol=%d",
				__func__, pca9468->ta_cur, pca9468->ta_vol,
				vin_vol);

		/* Check VIN after 1sec */
		pca9468->timer_id = TIMER_CHECK_CVMODE;
		pca9468->timer_period = 1000;
		break;

	default:
		break;
	}

done:
	pr_debug("%s: reschedule next id=%d period=%ld chg_state=%d\n",
		 __func__, pca9468->timer_id, pca9468->timer_period,
		pca9468->charging_state);

	mod_delayed_work(pca9468->dc_wq, &pca9468->timer_work,
			 msecs_to_jiffies(pca9468->timer_period));
error_exit:
	mutex_unlock(&pca9468->lock);
	pr_debug("%s: End, ret=%d next\n", __func__, ret);
	return ret;
}

/*
 * Preset TA voltage and current for Direct Charging Mode using
 * the configured cc_max and fv_uv limits. Used only on start
 */
static int pca9468_preset_dcmode(struct pca9468_charger *pca9468)
{
	int vbat;
	int ret = 0;

	pr_debug("%s: ======START=======\n", __func__);
	pr_debug("%s: = charging_state=%u == \n", __func__,
		 pca9468->charging_state);

	/* gcpm set ->cc_max and ->fv_uv before starting */
	if (pca9468->cc_max < 0 || pca9468->fv_uv < 0) {
		pr_err("%s: cc_max=%d fv_uv=%d invalid\n", __func__,
		       pca9468->cc_max, pca9468->fv_uv);
		return -EINVAL;
	}

	if (pca9468->charging_state != DC_STATE_PRESET_DC)
		dev_info(pca9468->dev, "%s: charging_state=%u->%u\n", __func__,
			 pca9468->charging_state, DC_STATE_PRESET_DC);

	pca9468->charging_state = DC_STATE_PRESET_DC;

	/* VBAT is over threshold but it might be "bouncy" due to transitory */
	vbat = pca9468_read_adc(pca9468, ADCCH_VBAT);
	if (vbat < 0) {
		ret = vbat;
		goto error;
	}

	/* v_float is set on start from GCPM */
	if (vbat > pca9468->fv_uv) {
		pr_err("%s: vbat adc=%d is higher than VFLOAT=%d\n", __func__,
			vbat, pca9468->fv_uv);
		ret = -EINVAL;
		goto error;
	}

	/* determined by ->cfg_iin and cc_max */
	pca9468->ta_max_cur = pca9468_get_iin_max(pca9468, pca9468->cc_max);
	pr_debug("%s: ta_max_cur=%u, iin_cfg=%u, pca9468->ta_type=%d\n",
		 __func__, pca9468->ta_max_cur, pca9468->pdata->iin_cfg,
		 pca9468->ta_type);

	/* Check the TA type and set the charging mode */
	if (pca9468->ta_type == TA_TYPE_WIRELESS) {
		if (pca9468->ftm_mode)
			goto error;
		/*
		 * Set the RX max voltage to enough high value to find RX
		 * maximum voltage initially
		 */
		pca9468->ta_max_vol = PCA9468_WCRX_MAX_VOL * pca9468->chg_mode;

		/* Get the RX max current/voltage(RX_MAX_CUR/VOL) */
		ret = pca9468_get_rx_max_power(pca9468);
		if (ret < 0) {
			pr_err("%s: no RX voltage to support 4:1 (%d)\n", __func__, ret);
			pca9468->chg_mode = CHG_NO_DC_MODE;
			goto error;
		}

		ret = pca9468_set_wireless_dc(pca9468, vbat);
		if (ret < 0) {
			pr_err("%s: set wired failed (%d)\n", __func__, ret);
			pca9468->chg_mode = CHG_NO_DC_MODE;
			goto error;
		}

		logbuffer_prlog(pca9468, LOGLEVEL_INFO,
				"Preset DC, rx_max_vol=%u, rx_max_cur=%u, rx_max_pwr=%lu, iin_cc=%u, chg_mode=%u",
				pca9468->ta_max_vol, pca9468->ta_max_cur, pca9468->ta_max_pwr,
				pca9468->iin_cc, pca9468->chg_mode);
	} else {
		const unsigned int ta_max_vol = pca9468->pdata->ta_max_vol * pca9468->chg_mode;

		if (pca9468->ftm_mode)
			goto skip_pps;
		/*
		 * Get the APDO max for 2:1 mode.
		 * Returns ->ta_max_vol, ->ta_max_cur, ->ta_max_pwr and
		 * ->ta_objpos for the given ta_max_vol and ta_max_cur.
		 */
		ret = pca9468_get_apdo_max_power(pca9468, ta_max_vol, PCA9468_TA_MAX_CUR);
		if (ret < 0) {
			pr_warn("%s: No APDO to support 2:1 for %d\n", __func__,
				PCA9468_TA_MAX_CUR);
			ret = pca9468_get_apdo_max_power(pca9468, ta_max_vol, 0);
		}

		if (ret < 0) {
			pr_err("%s: No APDO to support 2:1\n", __func__);
			pca9468->charging_state = DC_STATE_ERROR;
			pca9468_vote_dc_avail(pca9468, 0, 1);
			goto error;
		}

skip_pps:
		/*
		 * ->ta_max_cur is too high for startup, needs to target
		 * CC before hitting max current AND work to ta_max_cur
		 * from there.
		 */
		ret = pca9468_set_wired_dc(pca9468, vbat);
		if (ret < 0) {
			pr_err("%s: set wired failed (%d)\n", __func__, ret);
			pca9468->chg_mode = CHG_NO_DC_MODE;
			goto error;
		}

		logbuffer_prlog(pca9468, LOGLEVEL_INFO,
				"Preset DC, objpos=%d ta_max_vol=%u, ta_max_cur=%u, ta_max_pwr=%lu, iin_cc=%u, chg_mode=%u",
				pca9468->ta_objpos, pca9468->ta_max_vol, pca9468->ta_max_cur,
				pca9468->ta_max_pwr, pca9468->iin_cc, pca9468->chg_mode);

	}

error:
	pr_debug("%s: End, ret=%d\n", __func__, ret);
	return ret;
}

/* Preset direct charging configuration and start charging */
static int pca9468_preset_config(struct pca9468_charger *pca9468)
{
	int ret = 0;

	pr_debug("%s: ======START=======\n", __func__);

	mutex_lock(&pca9468->lock);

	dev_info(pca9468->dev, "%s: charging_state=%u->%u\n", __func__,
			pca9468->charging_state, DC_STATE_PRESET_DC);

	pca9468->charging_state = DC_STATE_PRESET_DC;

	/* ->iin_cc and ->fv_uv are configured externally */
	ret = pca9468_set_input_current(pca9468, pca9468->pdata->iin_cfg);
	if (ret < 0)
		goto error;

	ret = pca9468_set_vfloat(pca9468, pca9468->fv_uv);
	if (ret < 0)
		goto error;

	/* Enable PCA9468 unless aready enabled */
	ret = pca9468_set_charging(pca9468, true);
	if (ret < 0)
		goto error;

	/* Clear previous iin adc */
	pca9468->prev_iin = 0;
	pca9468->prev_inc = INC_NONE;

	/* Go to CHECK_ACTIVE state after 150ms, 300ms for wireless */
	pca9468->timer_id = TIMER_CHECK_ACTIVE;
	if (pca9468->ta_type == TA_TYPE_WIRELESS)
		pca9468->timer_period = PCA9468_ENABLE_WLC_DELAY_T;
	else
		pca9468->timer_period = PCA9468_ENABLE_DELAY_T;
	mod_delayed_work(pca9468->dc_wq, &pca9468->timer_work,
			   msecs_to_jiffies(pca9468->timer_period));
error:
	mutex_unlock(&pca9468->lock);
	pr_debug("%s: End, ret=%d\n", __func__, ret);
	return ret;
}

/*
 * Check the charging status at start before entering the adjust cc mode or
 * from pca9468_send_message() after a failure.
 */
static int pca9468_check_active_state(struct pca9468_charger *pca9468)
{
	int ret = 0;

	pr_debug("%s: ======START=======\n", __func__);
	pr_debug("%s: = charging_state=%u == \n", __func__,
		 pca9468->charging_state);

	mutex_lock(&pca9468->lock);

	if (pca9468->charging_state != DC_STATE_CHECK_ACTIVE)
		dev_info(pca9468->dev, "%s: charging_state=%u->%u\n", __func__,
			 pca9468->charging_state, DC_STATE_CHECK_ACTIVE);

	pca9468->charging_state = DC_STATE_CHECK_ACTIVE;

	ret = pca9468_check_error(pca9468);
	if (ret == 0) {
		/* PCA9468 is active state */
		pca9468->retry_cnt = 0;
		pca9468->timer_id = TIMER_ADJUST_CCMODE;
		pca9468->timer_period = 0;
	} else if (ret == -EAGAIN) {
		/* try restarting only */
		if (pca9468->retry_cnt >= PCA9468_MAX_RETRY_CNT) {
			pr_err("%s: retry failed\n", __func__);
			pca9468->charging_state = DC_STATE_ERROR;
			pca9468_vote_dc_avail(pca9468, 0, 1);
			ret = -EINVAL;
			goto exit_done;
		}

		/*
		 * Disable charging to retry enabling it later, return 0 here
		 * and the timer loop will figure out that there is something
		 * wrong and will retry.
		 */
		ret = pca9468_set_charging(pca9468, false);
		pr_err("%s: retry cnt=%d, (%d)\n", __func__,
		       pca9468->retry_cnt, ret);
		if (ret == 0) {
			pca9468->timer_id = TIMER_PRESET_DC;
			pca9468->timer_period = 0;
			pca9468->retry_cnt++;
		}
	} else {
		pr_err("%s: Error! disabling pca9468: ret(%d)\n", __func__, ret);
		pca9468->charging_state = DC_STATE_ERROR;
		pca9468_vote_dc_avail(pca9468, 0, 1);
	}

exit_done:

	/* Implement error handler function if it is needed */
	if (ret < 0) {
		logbuffer_prlog(pca9468, LOGLEVEL_ERR,
				"%s: charging_state=%d, not active or error (%d)",
				__func__, pca9468->charging_state, ret);
		pca9468->timer_id = TIMER_ID_NONE;
		pca9468->timer_period = 0;
	}

	if (!pca9468->ftm_mode)
		mod_delayed_work(pca9468->dc_wq, &pca9468->timer_work,
				 msecs_to_jiffies(pca9468->timer_period));
	mutex_unlock(&pca9468->lock);
	return ret;
}

/* Enter direct charging algorithm */
static int pca9468_start_direct_charging(struct pca9468_charger *pca9468)
{
	struct p9468_chg_stats *chg_data = &pca9468->chg_data;
	unsigned int val;
	int ret;

	pr_debug("%s: =========START=========\n", __func__);
	mutex_lock(&pca9468->lock);

	/* Set OV_DELTA to 40% */
	val = OV_DELTA_40P << MASK2SHIFT(PCA9468_BIT_OV_DELTA);
	ret = regmap_update_bits(pca9468->regmap, PCA9468_REG_SAFETY_CTRL,
				 PCA9468_BIT_OV_DELTA, val);
	if (ret < 0)
		goto error_done;

	/* Set Switching Frequency */
	val = pca9468->pdata->fsw_cfg;
	ret = regmap_update_bits(pca9468->regmap, PCA9468_REG_START_CTRL,
				 PCA9468_BIT_FSW_CFG, val);
	if (ret < 0)
		goto error_done;

	/* Set NTC voltage threshold */
	val = pca9468->pdata->ntc_th / PCA9468_NTC_TH_STEP;
	ret = regmap_write(pca9468->regmap, PCA9468_REG_NTC_TH_1, (val & 0xFF));
	if (ret < 0)
		goto error_done;
	ret = regmap_update_bits(pca9468->regmap, PCA9468_REG_NTC_TH_2,
				 PCA9468_BIT_NTC_THRESHOLD9_8, (val >> 8));
	if (ret < 0)
		goto error_done;

	/* configure DC charging type for the requested index */
	if (!pca9468->ftm_mode)
		ret = pca9468_set_ta_type(pca9468, pca9468->pps_index);
	pr_info("%s: Current ta_type=%d, chg_mode=%d\n", __func__,
		pca9468->ta_type, pca9468->chg_mode);
	if (ret < 0)
		goto error_done;

	/* wake lock */
	__pm_stay_awake(pca9468->monitor_wake_lock);

	/* Preset charging configuration and TA condition */
	ret = pca9468_preset_dcmode(pca9468);
	if (ret == 0) {
		/* Configure the TA  and start charging */
		pca9468->timer_id = TIMER_PDMSG_SEND;
		pca9468->timer_period = 0;

		mod_delayed_work(pca9468->dc_wq, &pca9468->timer_work,
				 msecs_to_jiffies(pca9468->timer_period));
	}

error_done:
	pr_debug("%s: End, ret=%d\n", __func__, ret);

	p9468_chg_stats_update(chg_data, pca9468);
	mutex_unlock(&pca9468->lock);
	return ret;
}

/* Check Vbat minimum level to start direct charging */
static int pca9468_check_vbatmin(struct pca9468_charger *pca9468)
{
	int ret = 0, vbat;

	pr_debug("%s: =========START=========\n", __func__);

	mutex_lock(&pca9468->lock);

	if (pca9468->charging_state != DC_STATE_CHECK_VBAT)
		dev_info(pca9468->dev, "%s: charging_state=%u->%u\n", __func__,
			 pca9468->charging_state, DC_STATE_CHECK_VBAT);

	pca9468->charging_state = DC_STATE_CHECK_VBAT;

	vbat = pca9468_read_adc(pca9468, ADCCH_VBAT);
	if (vbat < 0) {
		ret = vbat;
		goto error;
	}

	/* wait for CPM to send in the params */
	if (pca9468->cc_max < 0 || pca9468->fv_uv < 0) {
		pr_debug("%s: not yet fv_uv=%d, cc_max=%d vbat=%d\n", __func__,
			pca9468->fv_uv, pca9468->cc_max, vbat);

		/* retry again after 1sec */
		pca9468->timer_id = TIMER_VBATMIN_CHECK;
		pca9468->timer_period = PCA9468_VBATMIN_CHECK_T;
		pca9468->retry_cnt += 1;
	} else {
		logbuffer_prlog(pca9468, LOGLEVEL_INFO,
				"%s: starts at fv_uv=%d, cc_max=%d vbat=%d (min=%d)",
				__func__, pca9468->fv_uv, pca9468->cc_max, vbat,
				PCA9468_DC_VBAT_MIN);

		pca9468->timer_id = TIMER_PRESET_DC;
		pca9468->timer_period = 0;
		pca9468->retry_cnt = 0; /* start charging */
	}

	/* timeout for VBATMIN or charging parameters */
	if (pca9468->retry_cnt > PCA9468_MAX_RETRY_CNT) {
		logbuffer_prlog(pca9468, LOGLEVEL_DEBUG,
				"%s: TIMEOUT fv_uv=%d, cc_max=%d vbat=%d limit=%d",
				__func__, pca9468->fv_uv, pca9468->cc_max, vbat,
				PCA9468_DC_VBAT_MIN);
		ret = -ETIMEDOUT;
	} else {
		mod_delayed_work(pca9468->dc_wq, &pca9468->timer_work,
				 msecs_to_jiffies(pca9468->timer_period));
	}


error:
	mutex_unlock(&pca9468->lock);
	pr_debug("%s: End, ret=%d\n", __func__, ret);
	return ret;
}

/*
 * return 1 if apdo switch, 0 if no switch. < 0 on err.
 * TODO : lower input current per TCPC spec
 */
static int pca9468_check_apdo_switch(struct pca9468_charger *pca9468)
{
	unsigned int ta_max_vol, ta_max_cur, ta_objpos;
	unsigned int new_ta_cur, new_ta_max_cur, val;
	int ret;

	dev_dbg(pca9468->dev, "%s:START: ta_vol: %d, prev_ta_vol: %d, ta_cur: %d, prev_ta_cur: %d\n",
		__func__, pca9468->ta_vol, pca9468->prev_ta_vol, pca9468->ta_cur, pca9468->prev_ta_cur);

	ta_max_vol = pca9468->ta_vol;
	ta_max_cur = pca9468->ta_cur;

	ret = pca9468_get_apdo_index(pca9468, &ta_max_vol, &ta_max_cur, &ta_objpos);
	if (ret) {
		dev_dbg(pca9468->dev, "%s: error getting apdo index (%d)\n", __func__, ret);
		return ret;
	}

	if (pca9468->prev_ta_cur == pca9468->ta_cur && pca9468->prev_ta_vol == pca9468->ta_vol) {
		pca9468->ta_objpos = ta_objpos;
		return 0;
	}

	if (ta_objpos == pca9468->ta_objpos) {
		dev_dbg(pca9468->dev, "%s: stay at apdo %d\n", __func__, ta_objpos);
		return 0;
	}

	if (pca9468->prev_ta_cur < pca9468->ta_cur) {
		/* Should never happen as we limit to ta_max_cur */
		dev_err(pca9468->dev, "%s: ta_cur: %d > ta_max_cur %d causing APDO switch\n",
				__func__, pca9468->ta_cur, pca9468->ta_max_cur);
		pca9468->ta_cur = pca9468->ta_max_cur;
		ret = -EINVAL;
	} else if (pca9468->prev_ta_vol != pca9468->ta_vol) {
		const long power = (pca9468->prev_ta_cur / 1000) * (pca9468->prev_ta_vol / 1000);

		new_ta_cur = (power / (pca9468->ta_vol / 1000)) * 1000;
		new_ta_max_cur = new_ta_cur;
		ta_max_vol = pca9468->ta_vol;
		dev_dbg(pca9468->dev, "%s: find new ta_cur: ta_vol: %d, ta_cur: %d\n",
			__func__, pca9468->ta_vol, new_ta_cur);
		ret = pca9468_get_apdo_index(pca9468, &ta_max_vol, &new_ta_max_cur, &ta_objpos);
		if (ret) {
			new_ta_max_cur = 0;
			ta_max_vol = pca9468->ta_vol;
			ret = pca9468_get_apdo_index(pca9468, &ta_max_vol, &new_ta_max_cur, &ta_objpos);
			if (ret) {
				dev_err(pca9468->dev, "No available APDO to switch to (%d)\n", ret);
			} else {
				/* APDO can't provide needed ta_cur, so limit to max */
				val = new_ta_max_cur / PD_MSG_TA_CUR_STEP;
				pca9468->ta_cur = val * PD_MSG_TA_CUR_STEP;
				pca9468->ta_max_cur = pca9468->ta_cur;
			}
		} else {
			val = new_ta_cur / PD_MSG_TA_CUR_STEP;
			pca9468->ta_cur = val * PD_MSG_TA_CUR_STEP;
			pca9468->ta_max_cur = new_ta_max_cur;
		}
	}

	if (!ret && ta_objpos != pca9468->ta_objpos) {
		const int temp_cur = pca9468->ta_cur;

		dev_info(pca9468->dev, "ta_vol: %d->%d, ta_cur: %d->%d, ta_pos: %d->%d\n",
			pca9468->prev_ta_vol, pca9468->ta_vol, pca9468->prev_ta_cur, pca9468->ta_cur,
			pca9468->ta_objpos, ta_objpos);

		pca9468->ta_objpos = ta_objpos;

		/* Send one message immediately */
		/* force only voltage change */
		pca9468->ta_cur = pca9468->prev_ta_cur;
		ret = pca9468_send_pd_message(pca9468, PD_MSG_REQUEST_APDO);
		if (ret < 0)
			return ret;
		pca9468->ta_cur = temp_cur;
		ret = 1;
	}

	return ret;
}

static int pca9468_send_message(struct pca9468_charger *pca9468)
{
	int val, ret;
	const int timer_id = pca9468->timer_id;

	/* Go to the next state */
	mutex_lock(&pca9468->lock);

	pr_debug("%s: ====== START ======= \n", __func__);

	if (pca9468->ftm_mode)
		goto skip_pps;

	/* Adjust TA current and voltage step */
	if (pca9468->ta_type == TA_TYPE_WIRELESS) {
		/* RX voltage resolution is 100mV */
		val = pca9468->ta_vol / WCRX_VOL_STEP;
		pca9468->ta_vol = val * WCRX_VOL_STEP;

		/* Set RX voltage */
		pr_debug("%s: ta_type=%d, ta_vol=%d\n", __func__,
			 pca9468->ta_type, pca9468->ta_vol);
		ret = pca9468_send_rx_voltage(pca9468, WCRX_REQUEST_VOLTAGE);
	} else {
		/* PPS voltage resolution is 20mV */
		val = pca9468->ta_vol / PD_MSG_TA_VOL_STEP;
		pca9468->ta_vol = val * PD_MSG_TA_VOL_STEP;
		/* PPS current resolution is 50mA */
		val = pca9468->ta_cur / PD_MSG_TA_CUR_STEP;
		pca9468->ta_cur = val * PD_MSG_TA_CUR_STEP;
		/* PPS minimum current is 1000mA */
		if (pca9468->ta_cur < PCA9468_TA_MIN_CUR)
			pca9468->ta_cur = PCA9468_TA_MIN_CUR;

		pr_debug("%s: ta_type=%d, ta_vol=%d ta_cur=%d\n", __func__,
			 pca9468->ta_type, pca9468->ta_vol, pca9468->ta_cur);

		if (!pca9468->prev_ta_cur)
			pca9468->prev_ta_cur = pca9468->ta_cur;
		if (!pca9468->prev_ta_vol)
			pca9468->prev_ta_vol = pca9468->ta_vol;

		pca9468_check_apdo_switch(pca9468);
		/* Send PD Message */
		ret = pca9468_send_pd_message(pca9468, PD_MSG_REQUEST_APDO);
		if (ret >= 0) {
			pca9468->prev_ta_cur = pca9468->ta_cur;
			pca9468->prev_ta_vol = pca9468->ta_vol;
		}
	}

skip_pps:
	switch (pca9468->charging_state) {
	case DC_STATE_PRESET_DC:
		pca9468->timer_id = TIMER_PRESET_CONFIG;
		break;
	case DC_STATE_ADJUST_CC:
		pca9468->timer_id = TIMER_ADJUST_CCMODE;
		break;
	case DC_STATE_CC_MODE:
		pca9468->timer_id = TIMER_CHECK_CCMODE;
		break;
	case DC_STATE_START_CV:
		pca9468->timer_id = TIMER_ENTER_CVMODE;
		break;
	case DC_STATE_CV_MODE:
		pca9468->timer_id = TIMER_CHECK_CVMODE;
		break;
	case DC_STATE_ADJUST_TAVOL:
		pca9468->timer_id = TIMER_ADJUST_TAVOL;
		break;
	case DC_STATE_ADJUST_TACUR:
		pca9468->timer_id = TIMER_ADJUST_TACUR;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	if (ret < 0) {
		pr_err("%s: Error-send_pd_message to %d (%d)\n",
			__func__, pca9468->ta_type, ret);
		pca9468->timer_id = TIMER_CHECK_ACTIVE;
	}

	if (pca9468->ftm_mode)
		pca9468->timer_period = 0;
	else if (pca9468->ta_type == TA_TYPE_WIRELESS)
		pca9468->timer_period = PCA9468_PDMSG_WLC_WAIT_T;
	else
		pca9468->timer_period = PCA9468_PDMSG_WAIT_T;

	logbuffer_prlog(pca9468, LOGLEVEL_DEBUG,
			"%s: charging_state=%u timer_id:%d->%d ret=%d",
			__func__, pca9468->charging_state,
			timer_id, pca9468->timer_id, ret);

	mod_delayed_work(pca9468->dc_wq, &pca9468->timer_work,
			 msecs_to_jiffies(pca9468->timer_period));

	pr_debug("%s: End: timer_id=%d timer_period=%lu\n", __func__,
		 pca9468->timer_id, pca9468->timer_period);

	mutex_unlock(&pca9468->lock);
	return ret;
}

/* delayed work function for charging timer */
static void pca9468_timer_work(struct work_struct *work)
{
	struct pca9468_charger *pca9468 =
		container_of(work, struct pca9468_charger, timer_work.work);
	unsigned int charging_state;
	int timer_id;
	int ret = 0;

	pr_debug("%s: ========= START =========\n", __func__);

	/* TODO: remove locks from the calls and run all of this locked */
	mutex_lock(&pca9468->lock);

	p9468_chg_stats_update(&pca9468->chg_data, pca9468);
	charging_state = pca9468->charging_state;
	timer_id = pca9468->timer_id;

	pr_debug("%s: timer id=%d, charging_state=%u\n", __func__,
		 pca9468->timer_id, charging_state);

	mutex_unlock(&pca9468->lock);

	switch (timer_id) {

	/* charging_state <- DC_STATE_CHECK_VBAT */
	case TIMER_VBATMIN_CHECK:
		ret = pca9468_check_vbatmin(pca9468);
		if (ret < 0)
			goto error;
		break;

	/* charging_state <- DC_STATE_PRESET_DC */
	case TIMER_PRESET_DC:
		ret = pca9468_start_direct_charging(pca9468);
		if (ret < 0)
			goto error;
		break;

	/*
	 * charging_state <- DC_STATE_PRESET_DC
	 *	preset configuration, start charging
	 */
	case TIMER_PRESET_CONFIG:
		ret = pca9468_preset_config(pca9468);
		if (ret < 0)
			goto error;
		break;

	/*
	 * charging_state <- DC_STATE_PRESET_DC
	 *	150 ms after preset_config
	 */
	case TIMER_CHECK_ACTIVE:
		ret = pca9468_check_active_state(pca9468);
		if (ret < 0)
			goto error;
		break;

	case TIMER_ADJUST_CCMODE:
		ret = pca9468_charge_adjust_ccmode(pca9468);
		if (ret < 0)
			goto error;
		break;

	case TIMER_CHECK_CCMODE:
		ret = pca9468_charge_ccmode(pca9468);
		if (ret < 0)
			goto error;
		break;

	case TIMER_ENTER_CVMODE:
		/* Enter Pre-CV mode */
		ret = pca9468_charge_start_cvmode(pca9468);
		if (ret < 0)
			goto error;
		break;

	case TIMER_CHECK_CVMODE:
		ret = pca9468_charge_cvmode(pca9468);
		if (ret < 0)
			goto error;
		break;

	case TIMER_PDMSG_SEND:
		ret = pca9468_send_message(pca9468);
		if (ret < 0)
			goto error;
		break;

	/* called from 2 contexts */
	case TIMER_ADJUST_TAVOL:
		mutex_lock(&pca9468->lock);

		if (pca9468->ta_type == TA_TYPE_WIRELESS)
			ret = pca9468_adjust_rx_voltage(pca9468);
		else
			ret = pca9468_adjust_ta_voltage(pca9468);
		if (ret < 0) {
			mutex_unlock(&pca9468->lock);
			goto error;
		}

		mod_delayed_work(pca9468->dc_wq, &pca9468->timer_work,
				 msecs_to_jiffies(pca9468->timer_period));
		mutex_unlock(&pca9468->lock);
		break;

	/* called from 2 contexts */
	case TIMER_ADJUST_TACUR:
		mutex_lock(&pca9468->lock);
		ret = pca9468_adjust_ta_current(pca9468);
		if (ret < 0) {
			mutex_unlock(&pca9468->lock);
			goto error;
		}

		mod_delayed_work(pca9468->dc_wq, &pca9468->timer_work,
				 msecs_to_jiffies(pca9468->timer_period));
		mutex_unlock(&pca9468->lock);
		break;

	case TIMER_ID_NONE:
		ret = pca9468_stop_charging(pca9468);
		if (ret < 0)
			goto error;
		break;

	default:
		break;
	}

	/* Check the charging state again */
	if (pca9468->charging_state == DC_STATE_NO_CHARGING) {
		cancel_delayed_work(&pca9468->timer_work);
		cancel_delayed_work(&pca9468->pps_work);
	}

	pr_debug("%s: timer_id=%d->%d, charging_state=%u->%u, period=%ld\n",
		 __func__, timer_id, pca9468->timer_id, charging_state,
		 pca9468->charging_state, pca9468->timer_period);

	return;

error:
	pr_debug("%s: ========= ERROR =========\n", __func__);
	logbuffer_prlog(pca9468, LOGLEVEL_ERR,
			"%s: timer_id=%d->%d, charging_state=%u->%u, period=%ld ret=%d",
			__func__, timer_id, pca9468->timer_id, charging_state,
			pca9468->charging_state, pca9468->timer_period, ret);

	pca9468_stop_charging(pca9468);
}

/* delayed work function for pps periodic timer */
static void pca9468_pps_request_work(struct work_struct *work)
{
	struct pca9468_charger *pca9468 = container_of(work,
					struct pca9468_charger, pps_work.work);
	int ret = 0;

	pr_debug("%s: =========START=========\n", __func__);
	pr_debug("%s: = charging_state=%u == \n", __func__,
		 pca9468->charging_state);

	if (!pca9468->ftm_mode)
		ret = pca9468_send_pd_message(pca9468, PD_MSG_REQUEST_APDO);
	if (ret < 0)
		pr_err("%s: Error-send_pd_message\n", __func__);

	/* TODO: do other background stuff */

	pr_debug("%s: ret=%d\n", __func__, ret);
}

static int pca9468_hw_ping(struct pca9468_charger *pca9468)
{
	unsigned int val = 0;
	int ret;

	/* Read Device info register to check the incomplete I2C operation */
	ret = regmap_read(pca9468->regmap, PCA9468_REG_DEVICE_INFO, &val);
	if ((ret < 0) || (val != PCA9468_DEVICE_ID))
		ret = regmap_read(pca9468->regmap, PCA9468_REG_DEVICE_INFO, &val);
	if ((ret < 0) || (val != PCA9468_DEVICE_ID)) {
		dev_err(pca9468->dev, "reading DEVICE_INFO failed, val=%#x ret=%d\n",
			val, ret);
		return -EINVAL;
	}

	return 0;
}

/* one and done in probe */
static int pca9468_hw_init(struct pca9468_charger *pca9468)
{
	unsigned int val;
	int ret;

	/* Set OV_DELTA to 40% */
	val = OV_DELTA_40P << MASK2SHIFT(PCA9468_BIT_OV_DELTA);
	ret = regmap_update_bits(pca9468->regmap, PCA9468_REG_SAFETY_CTRL,
				 PCA9468_BIT_OV_DELTA, val);
	if (ret < 0)
		return ret;

	/* Set Switching Frequency */
	val = pca9468->pdata->fsw_cfg << MASK2SHIFT(PCA9468_BIT_FSW_CFG);
	ret = regmap_update_bits(pca9468->regmap, PCA9468_REG_START_CTRL,
				 PCA9468_BIT_FSW_CFG, val);
	if (ret < 0)
		return ret;

	/* Enable Reverse Current Detection, Active mode High, Force standby */
	val = PCA9468_BIT_REV_IIN_DET | PCA9468_EN_ACTIVE_H |
	      PCA9468_STANDBY_FORCED;
	ret = regmap_update_bits(pca9468->regmap, PCA9468_REG_START_CTRL,
				 (PCA9468_BIT_REV_IIN_DET |
				  PCA9468_BIT_EN_CFG |
				  PCA9468_BIT_STANDBY_EN),
				 val);
	if (ret < 0)
		return ret;

	/* clear LIMIT_INCREMENT_EN */
	val = 0;
	ret = regmap_update_bits(pca9468->regmap, PCA9468_REG_IIN_CTRL,
				 PCA9468_BIT_LIMIT_INCREMENT_EN, val);
	if (ret < 0)
		return ret;

	/* Set the ADC channels, NTC is invalid if Bias is not enabled */
	val = PCA9468_BIT_CH6_EN |	/* DIETEMP ADC */
	      PCA9468_BIT_CH5_EN |	/* IIN ADC */
	      PCA9468_BIT_CH3_EN |	/* VBAT ADC */
	      PCA9468_BIT_CH2_EN |	/* VIN ADC */
	      PCA9468_BIT_CH1_EN;	/* VOUT ADC */

	ret = regmap_write(pca9468->regmap, PCA9468_REG_ADC_CFG, val);
	if (ret < 0)
		return ret;

	/* ADC Mode change */
	val = 0x5B;
	ret = regmap_write(pca9468->regmap, PCA9468_REG_ADC_ACCESS, val);
	if (ret < 0)
		return ret;
	val = 0x10;
	ret = regmap_write(pca9468->regmap, PCA9468_REG_ADC_MODE, val);
	if (ret < 0)
		return ret;
	val = 0x00;
	ret = regmap_write(pca9468->regmap, PCA9468_REG_ADC_ACCESS, val);
	if (ret < 0)
		return ret;

	/* Read ADC compensation gain */
	ret = regmap_read(pca9468->regmap, PCA9468_REG_ADC_ADJUST, &val);
	if (ret < 0)
		return ret;
	pca9468->adc_comp_gain = adc_gain[val >> MASK2SHIFT(PCA9468_BIT_ADC_GAIN)];

	/* input current - uA*/
	ret = pca9468_set_input_current(pca9468, pca9468->pdata->iin_cfg);
	if (ret < 0)
		return ret;

	/* v float voltage */
	ret = pca9468_set_vfloat(pca9468, pca9468->pdata->v_float);
	if (ret < 0)
		return ret;

	/* Spread Spectrum settings */
	ret = regmap_update_bits(pca9468->regmap, PCA9468_REG_ADC_CTRL,
				 PCA9468_BIT_SC_CLK_DITHER_RATE,
				 pca9468->pdata->sc_clk_dither_rate);
	if (ret < 0)
		return ret;
	ret = regmap_update_bits(pca9468->regmap, PCA9468_REG_NTC_TH_2,
				 PCA9468_SC_CLK_DITHER_LIMIT,
				 pca9468->pdata->sc_clk_dither_limit << 4);
	if (ret < 0)
		return ret;
	val = pca9468->pdata->sc_clk_dither_en ? PCA9468_BIT_SC_CLK_DITHER_EN : 0;
	ret = regmap_update_bits(pca9468->regmap, PCA9468_REG_TEMP_CTRL,
				 PCA9468_BIT_SC_CLK_DITHER_EN, val);
	if (ret < 0)
		return ret;

	return ret;
}

static irqreturn_t pca9468_interrupt_handler(int irq, void *data)
{
	struct pca9468_charger *pca9468 = data;
	/* INT1, INT1_MSK, INT1_STS, STS_A, B, C, D */
	u8 int1[REG_INT1_MAX], sts[REG_STS_MAX];
	u8 masked_int;	/* masked int */
	bool handled = false;
	int ret;

	/* Read INT1, INT1_MSK, INT1_STS */
	ret = regmap_bulk_read(pca9468->regmap, PCA9468_REG_INT1, int1, 3);
	if (ret < 0) {
		dev_warn(pca9468->dev, "reading INT1_X failed\n");
		return IRQ_NONE;
	}

	/* Read STS_A, B, C, D */
	ret = regmap_bulk_read(pca9468->regmap, PCA9468_REG_STS_A, sts, 4);
	if (ret < 0) {
		dev_warn(pca9468->dev, "reading STS_X failed\n");
		return IRQ_NONE;
	}

	pr_debug("%s: int1=0x%2x, int1_sts=0x%2x, sts_a=0x%2x\n", __func__,
			int1[REG_INT1], int1[REG_INT1_STS], sts[REG_STS_A]);

	/* Check Interrupt */
	masked_int = int1[REG_INT1] & !int1[REG_INT1_MSK];
	if (masked_int & PCA9468_BIT_V_OK_INT) {
		/* V_OK interrupt happened */
		mutex_lock(&pca9468->lock);
		pca9468->mains_online = !!(int1[REG_INT1_STS] &
					PCA9468_BIT_V_OK_STS);

		/* TODO: alex perform a clean shutdown */
		mutex_unlock(&pca9468->lock);
		power_supply_changed(pca9468->mains);
		handled = true;
	}

	if (masked_int & PCA9468_BIT_NTC_TEMP_INT) {
		/* NTC_TEMP interrupt happened */
		if (int1[REG_INT1_STS] & PCA9468_BIT_NTC_TEMP_STS) {
			/* above NTC_THRESHOLD */
			dev_err(pca9468->dev, "charging stopped due to NTC threshold voltage\n");
		}
		handled = true;
	}

	if (masked_int & PCA9468_BIT_CHG_PHASE_INT) {
		/* CHG_PHASE interrupt happened */
		if (int1[REG_INT1_STS] & PCA9468_BIT_CHG_PHASE_STS) {
			/* Any of loops is active*/
			if (sts[REG_STS_A] & PCA9468_BIT_VFLT_LOOP_STS)	{
				/* V_FLOAT loop is in regulation */
				pr_debug("%s: V_FLOAT loop interrupt\n",
					__func__);
				/* Disable CHG_PHASE_M */
				ret = regmap_update_bits(pca9468->regmap,
						PCA9468_REG_INT1_MSK,
						PCA9468_BIT_CHG_PHASE_M,
						PCA9468_BIT_CHG_PHASE_M);
				if (ret < 0) {
					handled = false;
					return handled;
				}

				/* Go to Pre CV Mode */
				pca9468->timer_id = TIMER_ENTER_CVMODE;
				pca9468->timer_period = 10;
				mod_delayed_work(pca9468->dc_wq, &pca9468->timer_work,
					msecs_to_jiffies(pca9468->timer_period));

			} else if (sts[REG_STS_A] & PCA9468_BIT_IIN_LOOP_STS) {
				/* IIN loop or ICHG loop is in regulation */
				pr_debug("%s: IIN loop interrupt\n", __func__);
			} else if (sts[REG_STS_A] & PCA9468_BIT_CHG_LOOP_STS) {
				/* ICHG loop is in regulation */
				pr_debug("%s: ICHG loop interrupt\n", __func__);
			}
		}
		handled = true;
	}

	if (masked_int & PCA9468_BIT_CTRL_LIMIT_INT) {
		/* CTRL_LIMIT interrupt happened */
		if (int1[REG_INT1_STS] & PCA9468_BIT_CTRL_LIMIT_STS) {
			/* No Loop is active or OCP */
			if (sts[REG_STS_B] & PCA9468_BIT_OCP_FAST_STS) {
				/* Input fast over current */
				dev_err(pca9468->dev, "IIN > 50A instantaneously\n");
			}
			if (sts[REG_STS_B] & PCA9468_BIT_OCP_AVG_STS) {
				/* Input average over current */
				dev_err(pca9468->dev, "IIN > IIN_CFG*150percent\n");
			}
		}
		handled = true;
	}

	if (masked_int & PCA9468_BIT_TEMP_REG_INT) {
		/* TEMP_REG interrupt happened */
		if (int1[REG_INT1_STS] & PCA9468_BIT_TEMP_REG_STS) {
			/* Device is in temperature regulation */
			dev_err(pca9468->dev, "Device is in temperature regulation\n");
		}
		handled = true;
	}

	if (masked_int & PCA9468_BIT_ADC_DONE_INT) {
		/* ADC complete interrupt happened */
		dev_dbg(pca9468->dev, "ADC has been completed\n");
		handled = true;
	}

	if (masked_int & PCA9468_BIT_TIMER_INT) {
		/* Timer falut interrupt happened */
		if (int1[REG_INT1_STS] & PCA9468_BIT_TIMER_STS) {
			if (sts[REG_STS_B] & PCA9468_BIT_CHARGE_TIMER_STS) {
				/* Charger timer is expired */
				dev_err(pca9468->dev, "Charger timer is expired\n");
			}
			if (sts[REG_STS_B] & PCA9468_BIT_WATCHDOG_TIMER_STS) {
				/* Watchdog timer is expired */
				dev_err(pca9468->dev, "Watchdog timer is expired\n");
			}
		}
		handled = true;
	}

	return handled ? IRQ_HANDLED : IRQ_NONE;
}

static int pca9468_irq_init(struct pca9468_charger *pca9468,
			    struct i2c_client *client)
{
	const struct pca9468_platform_data *pdata = pca9468->pdata;
	int ret, msk, irq;

	irq = gpio_to_irq(pdata->irq_gpio);

	ret = gpio_request_one(pdata->irq_gpio, GPIOF_IN, client->name);
	if (ret < 0)
		goto fail;

	ret = request_threaded_irq(irq, NULL, pca9468_interrupt_handler,
				   IRQF_TRIGGER_LOW | IRQF_ONESHOT,
				   client->name, pca9468);
	if (ret < 0)
		goto fail_gpio;

	/* disable all interrupts by default. */
	msk = PCA9468_BIT_V_OK_M |
	      PCA9468_BIT_NTC_TEMP_M |
	      PCA9468_BIT_CHG_PHASE_M |
	      PCA9468_BIT_RESERVED_M |
	      PCA9468_BIT_CTRL_LIMIT_M |
	      PCA9468_BIT_TEMP_REG_M |
	      PCA9468_BIT_ADC_DONE_M |
	      PCA9468_BIT_TIMER_M;
	ret = regmap_write(pca9468->regmap, PCA9468_REG_INT1_MSK, msk);
	if (ret < 0)
		goto fail_write;

	client->irq = irq;
	return 0;

fail_write:
	free_irq(irq, pca9468);
fail_gpio:
	gpio_free(pdata->irq_gpio);
fail:
	client->irq = 0;
	return ret;
}

/* Returns the input current limit programmed into the charger in uA. */
int pca9468_input_current_limit(struct pca9468_charger *pca9468)
{
	int ret, intval;
	unsigned int val;

	if (!pca9468->mains_online)
		return -ENODATA;

	ret = regmap_read(pca9468->regmap, PCA9468_REG_IIN_CTRL, &val);
	if (ret < 0)
		return ret;

	intval = (val & PCA9468_BIT_IIN_CFG) * 100000;

	if (intval < 500000)
		intval = 500000;

	return intval;
}

/* Returns the constant charge current requested from GCPM */
static int get_const_charge_current(struct pca9468_charger *pca9468)
{
	/* Charging current cannot be controlled directly */
	return pca9468->cc_max;
}

/* Return the constant charge voltage programmed into the charger in uV. */
static int pca9468_const_charge_voltage(struct pca9468_charger *pca9468)
{
	unsigned int val;
	int ret;

	if (!pca9468->mains_online)
		return -ENODATA;

	ret = regmap_read(pca9468->regmap, PCA9468_REG_V_FLOAT, &val);
	if (ret < 0)
		return ret;

	return (val * 5 + 3725) * 1000;
}

/* index is the PPS source to use */
static int pca9468_set_charging_enabled(struct pca9468_charger *pca9468, int index)
{
	int ret = 0;
	if (index < 0 || index >= PPS_INDEX_MAX)
		return -EINVAL;

	mutex_lock(&pca9468->lock);

	/* Done is detected in CV when iin goes UNDER topoff. */
	if (pca9468->charging_state == DC_STATE_CHARGING_DONE)
		index = 0;

	if (index == 0) {

		logbuffer_prlog(pca9468, LOGLEVEL_DEBUG,
				"%s: stop pps_idx=%d->%d charging_state=%d timer_id=%d",
				__func__, pca9468->pps_index, index,
				pca9468->charging_state,
				pca9468->timer_id);

		/* this is the same as stop charging */
		pca9468->pps_index = 0;

		cancel_delayed_work(&pca9468->timer_work);
		cancel_delayed_work(&pca9468->pps_work);

		/* will call pca9468_stop_charging() in timer_work() */
		pca9468->timer_id = TIMER_ID_NONE;
		pca9468->timer_period = 0;
		mod_delayed_work(pca9468->dc_wq, &pca9468->timer_work,
				 msecs_to_jiffies(pca9468->timer_period));
	} else if (pca9468->charging_state == DC_STATE_NO_CHARGING) {
		logbuffer_prlog(pca9468, LOGLEVEL_DEBUG,
				"%s: start pps_idx=%d->%d charging_state=%d timer_id=%d",
				__func__, pca9468->pps_index, index,
				pca9468->charging_state,
				pca9468->timer_id);

		/* Start Direct Charging on Index */
		pca9468->dc_start_time = get_boot_sec();
		p9468_chg_stats_init(&pca9468->chg_data);
		pca9468->pps_index = index;

		dev_info(pca9468->dev, "%s: charging_state=%u->%u\n", __func__,
			 pca9468->charging_state, DC_STATE_CHECK_VBAT);

		/* PD is already in PE_SNK_STATE */
		pca9468->charging_state = DC_STATE_CHECK_VBAT;
		pca9468->timer_id = TIMER_VBATMIN_CHECK;
		pca9468->timer_period = 0;
		mod_delayed_work(pca9468->dc_wq, &pca9468->timer_work,
				 msecs_to_jiffies(pca9468->timer_period));

		/* Set the initial charging step */
		power_supply_changed(pca9468->mains);
	} else if (pca9468->charging_state == DC_STATE_ERROR) {
		logbuffer_prlog(pca9468, LOGLEVEL_DEBUG,
				"%s: error pps_idx=%d->%d charging_state=%d timer_id=%d",
				__func__, pca9468->pps_index, index,
				pca9468->charging_state,
				pca9468->timer_id);
		ret = -EINVAL;
	}

	mutex_unlock(&pca9468->lock);

	return ret;
}

static int pca9468_mains_set_property(struct power_supply *psy,
				      enum power_supply_property prop,
				      const union power_supply_propval *val)
{
	struct pca9468_charger *pca9468 = power_supply_get_drvdata(psy);
	int ret = 0;

	pr_debug("%s: =========START=========\n", __func__);
	pr_debug("%s: prop=%d, val=%d\n", __func__, prop, val->intval);
	if (!pca9468->init_done)
		return -EAGAIN;

	switch (prop) {

	case POWER_SUPPLY_PROP_ONLINE:
		if (val->intval == 0) {
			ret = pca9468_stop_charging(pca9468);
			if (ret < 0)
				pr_err("%s: cannot stop charging (%d)\n",
				       __func__, ret);

			pca9468->mains_online = false;
		} else if (pca9468->mains_online == false) {
			pca9468->mains_online = true;
		}

		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		ret = pca9468_set_new_vfloat(pca9468, val->intval);
		break;

	/*
	 * pcaA9468 cannot control charging current directly so need to control
	 * current on TA side resolving cc_max for TA_VOL*TA_CUT on vbat.
	 * NOTE: iin should be equivalent to iin = cc_max /2
	 */
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = pca9468_set_new_cc_max(pca9468, val->intval);
		break;

	/* CURRENT MAX, same as IIN is really only set by the algo */
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		pr_debug("%s: set iin %d, ignore\n", __func__, val->intval);
		break;

	/* allow direct setting, not used */
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		mutex_lock(&pca9468->lock);
		ret = pca9468_set_new_iin(pca9468, val->intval);
		mutex_unlock(&pca9468->lock);
		break;

	default:
		ret = -EINVAL;
		break;
	}

	pr_debug("%s: End, ret=%d\n", __func__, ret);
	return ret;
}

static int pca9468_mains_get_property(struct power_supply *psy,
				     enum power_supply_property prop,
				     union power_supply_propval *val)
{
	struct pca9468_charger *pca9468 = power_supply_get_drvdata(psy);
	int intval, rc, ret = 0;

	if (!pca9468->init_done)
		return -EAGAIN;

	switch (prop) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = pca9468->mains_online;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = pca9468_is_present(pca9468);
		if (val->intval < 0)
			val->intval = 0;
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		ret = pca9468_const_charge_voltage(pca9468);
		if (ret < 0)
			return ret;
		val->intval = ret;
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		ret = get_const_charge_current(pca9468);
		if (ret < 0)
			return ret;
		val->intval = ret;
		break;

	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = pca9468_input_current_limit(pca9468);
		if (ret < 0)
			return ret;
		val->intval = ret;
		break;

	case POWER_SUPPLY_PROP_CURRENT_NOW:
		/* return the output current - uA unit */
		rc = pca9468_get_iin(pca9468, &val->intval);
		if (rc < 0)
			dev_err(pca9468->dev, "Invalid IIN ADC (%d)\n", rc);
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		intval = pca9468_read_adc(pca9468, ADCCH_VOUT);
		if (intval < 0)
			return intval;
		val->intval = intval;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		intval = pca9468_read_adc(pca9468, ADCCH_VBAT);
		if (intval < 0)
			return intval;
		val->intval = intval;
		break;

	/* TODO: read NTC temperature? */
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = pca9468_read_adc(pca9468, ADCCH_DIETEMP);
		break;

	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = pca9468_get_charge_type(pca9468);
		break;

	case POWER_SUPPLY_PROP_STATUS:
		val->intval = pca9468_get_status(pca9468);
		break;

	case POWER_SUPPLY_PROP_CURRENT_MAX:
		ret = pca9468_input_current_limit(pca9468);
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
static enum power_supply_property pca9468_mains_properties[] = {
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

static int pca9468_mains_is_writeable(struct power_supply *psy,
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

static int pca9468_gbms_mains_set_property(struct power_supply *psy,
					   enum gbms_property prop,
					   const union gbms_propval *val)
{
	struct pca9468_charger *pca9468 = power_supply_get_drvdata(psy);
	int ret = 0;

	pr_debug("%s: =========START=========\n", __func__);
	pr_debug("%s: prop=%d, val=%d\n", __func__, prop, val->prop.intval);
	if (!pca9468->init_done)
		return -EAGAIN;

	switch (prop) {

	case GBMS_PROP_CHARGING_ENABLED:
		ret = pca9468_set_charging_enabled(pca9468, val->prop.intval);
		break;

	case GBMS_PROP_CHARGE_DISABLE:
		dev_dbg(pca9468->dev, "%s: ChargeDisable %d, chg_state:%d\n", __func__,
			val->prop.intval, pca9468->charging_state);
		if (val->prop.intval) {
			if (pca9468->charging_state == DC_STATE_ERROR)
				pca9468->charging_state = DC_STATE_NO_CHARGING;
			pca9468_vote_dc_avail(pca9468, 1, 1);
		}
		break;

	default:
		pr_debug("%s: route to pca9468_mains_set_property, psp:%d\n", __func__, prop);
		return -ENODATA;
	}

	pr_debug("%s: End, ret=%d\n", __func__, ret);
	return ret;
}

static int pca9468_gbms_mains_get_property(struct power_supply *psy,
					   enum gbms_property prop,
					   union gbms_propval *val)
{
	struct pca9468_charger *pca9468 = power_supply_get_drvdata(psy);
	union gbms_charger_state chg_state;
	int ret = 0;

	if (!pca9468->init_done)
		return -EAGAIN;

	switch (prop) {
	case GBMS_PROP_CHARGE_DISABLE:
		ret = pca9468_get_charging_enabled(pca9468);
		if (ret < 0)
			return ret;
		val->prop.intval = !ret;
		break;

	case GBMS_PROP_CHARGING_ENABLED:
		ret = pca9468_get_charging_enabled(pca9468);
		if (ret < 0)
			return ret;
		val->prop.intval = ret;
		break;

	case GBMS_PROP_CHARGE_CHARGER_STATE:
		ret = pca9468_get_chg_chgr_state(pca9468, &chg_state);
		if (ret < 0)
			return ret;
		val->int64val = chg_state.v;
		break;

	default:
		pr_debug("%s: route to pca9468_mains_get_property, psp:%d\n", __func__, prop);
		return -ENODATA;
	}

	return 0;
}

static int pca9468_gbms_mains_is_writeable(struct power_supply *psy,
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

static bool pca9468_is_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case PCA9468_REG_DEVICE_INFO ... PCA9468_REG_STS_ADC_9:
	case PCA9468_REG_IIN_CTRL ... PCA9468_REG_NTC_TH_2:
	case PCA9468_REG_ADC_ACCESS:
	case PCA9468_REG_ADC_ADJUST:
	case PCA9468_REG_ADC_IMPROVE:
	case PCA9468_REG_ADC_MODE:
	case 0x40 ... 0x4f: /* debug */
		return true;
	default:
		break;
	}

	return false;
}

static struct regmap_config pca9468_regmap = {
	.name		= "dc-mains",
	.reg_bits	= 8,
	.val_bits	= 8,
	.max_register	= PCA9468_MAX_REGISTER,
	.readable_reg = pca9468_is_reg,
	.volatile_reg = pca9468_is_reg,
};

static struct gbms_desc pca9468_mains_desc = {
	.psy_dsc.name		= "pca9468-mains",
	/* b/179246019 will not look online to Android */
	.psy_dsc.type		= POWER_SUPPLY_TYPE_UNKNOWN,
	.psy_dsc.get_property	= pca9468_mains_get_property,
	.psy_dsc.set_property 	= pca9468_mains_set_property,
	.psy_dsc.property_is_writeable = pca9468_mains_is_writeable,
	.get_property		= pca9468_gbms_mains_get_property,
	.set_property 		= pca9468_gbms_mains_set_property,
	.property_is_writeable = pca9468_gbms_mains_is_writeable,
	.psy_dsc.properties	= pca9468_mains_properties,
	.psy_dsc.num_properties	= ARRAY_SIZE(pca9468_mains_properties),
	.forward		= true,
};

#if defined(CONFIG_OF)
static int of_pca9468_dt(struct device *dev,
			 struct pca9468_platform_data *pdata)
{
	struct device_node *np_pca9468 = dev->of_node;
	int ret;
	if(!np_pca9468)
		return -EINVAL;

	/* irq gpio */
	pdata->irq_gpio = of_get_named_gpio(np_pca9468, "pca9468,irq-gpio", 0);
	pr_info("%s: irq-gpio: %d \n", __func__, pdata->irq_gpio);

	/* input current limit */
	ret = of_property_read_u32(np_pca9468, "pca9468,input-current-limit",
				   &pdata->iin_cfg_max);
	if (ret) {
		pr_warn("%s: pca9468,input-current-limit is Empty\n", __func__);
		pdata->iin_cfg_max = PCA9468_IIN_CFG_DFT;
	}
	pdata->iin_cfg = pdata->iin_cfg_max;
	pr_info("%s: pca9468,iin_cfg is %u\n", __func__, pdata->iin_cfg);

	/* TA max voltage limit */
	ret = of_property_read_u32(np_pca9468, "pca9468,ta-max-vol",
				   &pdata->ta_max_vol);
	if (ret) {
		pr_warn("%s: pca9468,ta-max-vol is Empty\n", __func__);
		pdata->ta_max_vol = PCA9468_TA_MAX_VOL;
	}
	ret = of_property_read_u32(np_pca9468, "pca9468,ta-max-vol-cp",
				   &pdata->ta_max_vol_cp);
	if (ret) {
		pr_warn("%s: pca9468,ta-max-vol-cp is Empty\n", __func__);
		pdata->ta_max_vol_cp = pdata->ta_max_vol;
	}

	/* charging float voltage */
	ret = of_property_read_u32(np_pca9468, "pca9468,float-voltage",
				   &pdata->v_float_dt);
	if (ret) {
		pr_warn("%s: pca9468,float-voltage is Empty\n", __func__);
		pdata->v_float_dt = PCA9468_VFLOAT_DFT;
	}
	pdata->v_float = pdata->v_float_dt;
	pr_info("%s: pca9468,v_float is %u\n", __func__, pdata->v_float);

	/* input topoff current */
	ret = of_property_read_u32(np_pca9468, "pca9468,input-itopoff",
				   &pdata->iin_topoff);
	if (ret) {
		pr_warn("%s: pca9468,input-itopoff is Empty\n", __func__);
		pdata->iin_topoff = PCA9468_IIN_DONE_DFT;
	}
	pr_info("%s: pca9468,iin_topoff is %u\n", __func__, pdata->iin_topoff);

	/* switching frequency */
	ret = of_property_read_u32(np_pca9468, "pca9468,switching-frequency",
				   &pdata->fsw_cfg);
	if (ret) {
		pr_warn("%s: pca9468,switching frequency is Empty\n", __func__);
		pdata->fsw_cfg = PCA9468_FSW_CFG_DFT;
	}
	pr_info("%s: pca9468,fsw_cfg is %u\n", __func__, pdata->fsw_cfg);

	/* NTC threshold voltage */
	ret = of_property_read_u32(np_pca9468, "pca9468,ntc-threshold",
				   &pdata->ntc_th);
	if (ret) {
		pr_warn("%s: pca9468,ntc threshold voltage is Empty\n",
			__func__);
		pdata->ntc_th = PCA9468_NTC_TH_DFT;
	}
	pr_info("%s: pca9468,ntc_th is %u\n", __func__, pdata->ntc_th);

	/* iin offsets */
	ret = of_property_read_u32(np_pca9468, "pca9468,iin-max-offset",
				   &pdata->iin_max_offset);
	if (ret)
		pdata->iin_max_offset = PCA9468_IIN_MAX_OFFSET;
	pr_info("%s: pca9468,iin_max_offset is %u\n", __func__, pdata->iin_max_offset);

	ret = of_property_read_u32(np_pca9468, "pca9468,iin-cc_comp-offset",
				   &pdata->iin_cc_comp_offset);
	if (ret)
		pdata->iin_cc_comp_offset = PCA9468_IIN_CC_COMP_OFFSET;
	pr_info("%s: pca9468,iin_cc_comp_offset is %u\n", __func__, pdata->iin_cc_comp_offset);

	/* Spread Spectrum settings */
	ret = of_property_read_u32(np_pca9468, "pca9468,sc-clk-dither-rate",
				   &pdata->sc_clk_dither_rate);
	if (ret)
		pdata->sc_clk_dither_rate = PCA9468_SC_CLK_DITHER_RATE_DEF;
	else
		pr_info("%s: pca9468,sc-clk-dither-rate is %u\n", __func__,
			pdata->sc_clk_dither_rate);
	ret = of_property_read_u32(np_pca9468, "pca9468,sc-clk-dither-limit",
				   &pdata->sc_clk_dither_limit);
	if (ret)
		pdata->sc_clk_dither_limit = PCA9468_SC_CLK_DITHER_LIMIT_DEF;
	else
		pr_info("%s: pca9468,sc-clk-dither-limit is %u\n", __func__,
			pdata->sc_clk_dither_limit);
	pdata->sc_clk_dither_en = of_property_read_bool(np_pca9468, "pca9468,spread-spectrum");
	pr_info("%s: pca9468,spread-spectrum is %u\n", __func__, pdata->sc_clk_dither_en);

	ret = of_property_read_u32(np_pca9468, "pca9468,ta-max-cur-mult", &pdata->ta_max_cur_mult);
	if (ret)
		pdata->ta_max_cur_mult = PCA9468_TA_MAX_CUR_MULT_DEF;
	else
		pr_info("%s: pca9468,ta-max-cur-mult is %d\n", __func__, pdata->ta_max_cur_mult);

#ifdef CONFIG_THERMAL
	/* USBC thermal zone */
	ret = of_property_read_string(np_pca9468, "google,usb-port-tz-name",
				      &pdata->usb_tz_name);
	if (ret) {
		pr_info("%s: google,usb-port-tz-name is Empty\n", __func__);
		pdata->usb_tz_name = NULL;
	} else {
		pr_info("%s: google,usb-port-tz-name is %s\n", __func__,
			pdata->usb_tz_name);
	}
#endif

	return 0;
}
#else
static int of_pca9468_dt(struct device *dev,
			 struct pca9468_platform_data *pdata)
{
	return 0;
}
#endif /* CONFIG_OF */

#ifdef CONFIG_THERMAL
static int pca9468_usb_tz_read_temp(struct thermal_zone_device *tzd, int *temp)
{
	struct pca9468_charger *pca9468 = tzd->devdata;

	if (!pca9468)
		return -ENODEV;
	*temp = pca9468_read_adc(pca9468, ADCCH_NTC);

	return 0;
}

static struct thermal_zone_device_ops pca9468_usb_tzd_ops = {
	.get_temp = pca9468_usb_tz_read_temp,
};
#endif

static int read_reg(void *data, u64 *val)
{
	struct pca9468_charger *chip = data;
	int rc;
	unsigned int temp;

	rc = regmap_read(chip->regmap, chip->debug_address, &temp);
	if (rc) {
		pr_err("Couldn't read reg %x rc = %d\n",
			chip->debug_address, rc);
		return -EAGAIN;
	}
	*val = temp;
	return 0;
}

static int write_reg(void *data, u64 val)
{
	struct pca9468_charger *chip = data;
	int rc;
	u8 temp;

	temp = (u8) val;
	rc = regmap_write(chip->regmap, chip->debug_address, temp);
	if (rc) {
		pr_err("Couldn't write 0x%02x to 0x%02x rc = %d\n",
			temp, chip->debug_address, rc);
		return -EAGAIN;
	}
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(register_debug_ops, read_reg, write_reg, "0x%02llx\n");

static int debug_apply_offsets(void *data, u64 val)
{
	struct pca9468_charger *chip = data;
	int ret;

	ret = pca9468_set_new_cc_max(chip, chip->cc_max);
	pr_info("Apply offsets iin_max_o=%d iin_cc_comp_o=%d ret=%d\n",
		chip->pdata->iin_max_offset, chip->pdata->iin_cc_comp_offset,
		ret);

	return ret;
}
DEFINE_SIMPLE_ATTRIBUTE(apply_offsets_debug_ops, NULL, debug_apply_offsets, "0x%02llx\n");


static int debug_adc_chan_get(void *data, u64 *val)
{
	struct pca9468_charger *pca9468 = data;

	*val = pca9468_read_adc(data, pca9468->debug_adc_channel);
	return 0;
}

static int debug_adc_chan_set(void *data, u64 val)
{
	struct pca9468_charger *pca9468 = data;

	if (val < ADCCH_VOUT || val >= ADCCH_MAX)
		return -EINVAL;
	pca9468->debug_adc_channel = val;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_adc_chan_ops, debug_adc_chan_get,
			debug_adc_chan_set, "%llu\n");

static int debug_ftm_mode_get(void *data, u64 *val)
{
	struct pca9468_charger *pca9468 = data;
	*val = pca9468->ftm_mode;
	return 0;
}

static int debug_ftm_mode_set(void *data, u64 val)
{
	struct pca9468_charger *pca9468 = data;

	if (val) {
		pca9468->ftm_mode = true;
		pca9468->ta_type = TA_TYPE_USBPD;
		pca9468->chg_mode = CHG_2TO1_DC_MODE;
	} else {
		pca9468->ftm_mode = false;
	}

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(debug_ftm_mode_ops, debug_ftm_mode_get, debug_ftm_mode_set, "%llu\n");

static int debug_pps_index_get(void *data, u64 *val)
{
	struct pca9468_charger *pca9468 = data;

	*val = pca9468->pps_index;
	return 0;
}

static int debug_pps_index_set(void *data, u64 val)
{
	struct pca9468_charger *pca9468 = data;

	return pca9468_set_charging_enabled(pca9468, (int)val);
}

DEFINE_SIMPLE_ATTRIBUTE(debug_pps_index_ops, debug_pps_index_get,
			debug_pps_index_set, "%llu\n");

static int debug_ta_max_vol_set(void *data, u64 val)
{
	struct pca9468_charger *pca9468 = data;

	pca9468->pdata->ta_max_vol = val;
	pca9468->pdata->ta_max_vol_cp = val;

	pca9468->ta_max_vol = val * pca9468->chg_mode;

	return 0;
}

static int debug_ta_max_vol_get(void *data, u64 *val)
{
	struct pca9468_charger *pca9468 = data;

	*val = pca9468->pdata->ta_max_vol;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(debug_ta_max_vol_ops, debug_ta_max_vol_get,
			debug_ta_max_vol_set, "%llu\n");

static ssize_t show_sts_ab(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct pca9468_charger *pca9468 = dev_get_drvdata(dev);
	u8 tmp[2];
	int ret;

	ret = regmap_bulk_read(pca9468->regmap, PCA9468_REG_STS_A, &tmp, sizeof(tmp));
	if (ret < 0)
		return ret;

	return scnprintf(buf, PAGE_SIZE, "%02x%02x\n", tmp[0], tmp[1]);
}

static DEVICE_ATTR(sts_ab, 0444, show_sts_ab, NULL);


static ssize_t p9468_show_chg_stats(struct device *dev, struct device_attribute *attr,
				    char *buff)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pca9468_charger *pca9468 = i2c_get_clientdata(client);
	struct p9468_chg_stats *chg_data = &pca9468->chg_data;
	const int max_size = PAGE_SIZE;
	int len = -ENODATA;

	mutex_lock(&pca9468->lock);

	if (!p9468_chg_stats_valid(chg_data))
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
	mutex_unlock(&pca9468->lock);
	return len;
}

static ssize_t p9468_set_chg_stats(struct device *dev, struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pca9468_charger *pca9468 = i2c_get_clientdata(client);

	mutex_lock(&pca9468->lock);
	p9468_chg_stats_init(&pca9468->chg_data);
	mutex_unlock(&pca9468->lock);

	return count;
}

static DEVICE_ATTR(chg_stats, 0644, p9468_show_chg_stats, p9468_set_chg_stats);

static ssize_t show_dump_reg(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct pca9468_charger *pca9468 = dev_get_drvdata(dev);
	u8 tmp[PCA9468_MAX_REGISTER + 1];
	int ret, i;
	int len = 0;

	ret = regmap_bulk_read(pca9468->regmap, PCA9468_REG_DEVICE_INFO, &tmp,
			       PCA9468_MAX_REGISTER + 1);
	if (ret < 0)
		return ret;

	for (i = 0; i <= PCA9468_MAX_REGISTER; i++)
		len += scnprintf(&buf[len], PAGE_SIZE - len, "%02x: %02x\n", i, tmp[i]);

	return len;
}

static DEVICE_ATTR(registers_dump, 0444, show_dump_reg, NULL);

static int pca9468_create_fs_entries(struct pca9468_charger *chip)
{

	device_create_file(chip->dev, &dev_attr_sts_ab);
	device_create_file(chip->dev, &dev_attr_chg_stats);
	device_create_file(chip->dev, &dev_attr_registers_dump);

	chip->debug_root = debugfs_create_dir("charger-pca9468", NULL);
	if (IS_ERR_OR_NULL(chip->debug_root)) {
		dev_err(chip->dev, "Couldn't create debug dir\n");
		return -ENOENT;
	}

	debugfs_create_bool("wlc_rampout_iin", 0644, chip->debug_root,
			     &chip->wlc_ramp_out_iin);
	debugfs_create_u32("wlc_rampout_delay", 0644, chip->debug_root,
			   &chip->wlc_ramp_out_delay);
	debugfs_create_u32("wlc_rampout_vout_target", 0644, chip->debug_root,
			   &chip->wlc_ramp_out_vout_target);


	debugfs_create_u32("debug_level", 0644, chip->debug_root,
			   &debug_printk_prlog);
	debugfs_create_u32("no_logbuffer", 0644, chip->debug_root,
			   &debug_no_logbuffer);

	debugfs_create_file("data", 0644, chip->debug_root, chip, &register_debug_ops);
	debugfs_create_x32("address", 0644, chip->debug_root, &chip->debug_address);

	debugfs_create_u32("iin_max_offset", 0644, chip->debug_root,
			   &chip->pdata->iin_max_offset);
	debugfs_create_u32("iin_cc_comp_offset", 0644, chip->debug_root,
			   &chip->pdata->iin_cc_comp_offset);
	debugfs_create_file("apply_offsets", 0644, chip->debug_root, chip,
			    &apply_offsets_debug_ops);

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


static int pca9468_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	static char *battery[] = { "pca9468-battery" };
	struct power_supply_config mains_cfg = {};
	struct pca9468_platform_data *pdata;
	struct pca9468_charger *pca9468_chg;
	struct device *dev = &client->dev;
	const char *psy_name = NULL;
	int ret;

	pr_debug("%s: =========START=========\n", __func__);

	pca9468_chg = devm_kzalloc(dev, sizeof(*pca9468_chg), GFP_KERNEL);
	if (!pca9468_chg)
		return -ENOMEM;

#if defined(CONFIG_OF)
	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
				     sizeof(struct pca9468_platform_data),
				     GFP_KERNEL);
		if (!pdata) {
			dev_err(&client->dev, "Failed to allocate memory \n");
			return -ENOMEM;
		}

		ret = of_pca9468_dt(&client->dev, pdata);
		if (ret < 0){
			dev_err(&client->dev, "Failed to get device of_node \n");
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

	i2c_set_clientdata(client, pca9468_chg);

	mutex_init(&pca9468_chg->lock);
	pca9468_chg->dev = &client->dev;
	pca9468_chg->pdata = pdata;
	pca9468_chg->charging_state = DC_STATE_NO_CHARGING;
	pca9468_chg->wlc_ramp_out_iin = true;
	pca9468_chg->wlc_ramp_out_vout_target = 0; /* use Vbatt*4 as default */
	pca9468_chg->wlc_ramp_out_delay = 250; /* 250 ms default */

	/* Create a work queue for the direct charger */
	pca9468_chg->dc_wq = alloc_ordered_workqueue("pca9468_dc_wq", WQ_MEM_RECLAIM);
	if (pca9468_chg->dc_wq == NULL) {
		dev_err(pca9468_chg->dev, "failed to create work queue\n");
		return -ENOMEM;
	}

	pca9468_chg->monitor_wake_lock =
		wakeup_source_register(NULL, "pca9468-charger-monitor");
	if (!pca9468_chg->monitor_wake_lock) {
		pr_err("Failed to register wakeup source\n");
		return -ENODEV;
	}

	/* initialize work */
	INIT_DELAYED_WORK(&pca9468_chg->timer_work, pca9468_timer_work);
	pca9468_chg->timer_id = TIMER_ID_NONE;
	pca9468_chg->timer_period = 0;

	INIT_DELAYED_WORK(&pca9468_chg->pps_work, pca9468_pps_request_work);

	ret = of_property_read_string(dev->of_node,
				      "pca9468,psy_name", &psy_name);
	if ((ret == 0) && (strlen(psy_name) > 0))
		pca9468_regmap.name = pca9468_mains_desc.psy_dsc.name =
		    devm_kstrdup(dev, psy_name, GFP_KERNEL);

	pca9468_chg->regmap = devm_regmap_init_i2c(client, &pca9468_regmap);
	if (IS_ERR(pca9468_chg->regmap)) {
		ret = -EINVAL;
		goto error;
	}

	ret = pca9468_probe_pps(pca9468_chg);
	if (ret < 0) {
		pr_warn("pca9468: PPS not available (%d)\n", ret);
	} else {
		const char *logname = "pca9468";

		pca9468_chg->log = logbuffer_register(logname);
		if (IS_ERR(pca9468_chg->log)) {
			pr_err("%s: no logbuffer (%ld)\n", __func__,
			       PTR_ERR(pca9468_chg->log));
			pca9468_chg->log = NULL;
		}
	}

	ret = pca9468_hw_ping(pca9468_chg);
	if (ret < 0)
		goto error;

	/* TODO: only enable ADC if usb_tz_name is defined */
	ret = pca9468_hw_init(pca9468_chg);
	if (ret < 0)
		goto error;

	mains_cfg.supplied_to = battery;
	mains_cfg.num_supplicants = ARRAY_SIZE(battery);
	mains_cfg.drv_data = pca9468_chg;
	pca9468_chg->mains = devm_power_supply_register(dev,
							&pca9468_mains_desc.psy_dsc,
							&mains_cfg);
	if (IS_ERR(pca9468_chg->mains)) {
		ret = -ENODEV;
		goto error;
	}

	/* Interrupt pin is optional. */
	if (pdata->irq_gpio >= 0) {
		ret = pca9468_irq_init(pca9468_chg, client);
		if (ret < 0) {
			dev_warn(dev, "failed to initialize IRQ: %d\n", ret);
			dev_warn(dev, "disabling IRQ support\n");
		}

		/* disable interrupt */
		disable_irq(client->irq);
	}

	ret = pca9468_create_fs_entries(pca9468_chg);
	if (ret < 0)
		dev_err(dev, "error while registering debugfs %d\n", ret);

#ifdef CONFIG_THERMAL
	if (pdata->usb_tz_name) {
		pca9468_chg->usb_tzd =
			thermal_zone_device_register(pdata->usb_tz_name, 0, 0,
						     pca9468_chg,
						     &pca9468_usb_tzd_ops,
						     NULL, 0, 0);
		if (IS_ERR(pca9468_chg->usb_tzd)) {
			pca9468_chg->usb_tzd = NULL;
			ret = PTR_ERR(pca9468_chg->usb_tzd);
			dev_err(dev, "Couldn't register usb connector thermal zone ret=%d\n",
				ret);
		}
	}
#endif

	pca9468_chg->dc_avail = NULL;

	pca9468_chg->init_done = true;
	pr_info("pca9468: probe_done\n");
	pr_debug("%s: =========END=========\n", __func__);
	return 0;

error:
	destroy_workqueue(pca9468_chg->dc_wq);
	mutex_destroy(&pca9468_chg->lock);
	wakeup_source_unregister(pca9468_chg->monitor_wake_lock);
	return ret;
}

static void pca9468_remove(struct i2c_client *client)
{
	struct pca9468_charger *pca9468_chg = i2c_get_clientdata(client);

	/* stop charging if it is active */
	pca9468_stop_charging(pca9468_chg);

	if (client->irq) {
		free_irq(client->irq, pca9468_chg);
		gpio_free(pca9468_chg->pdata->irq_gpio);
	}

	destroy_workqueue(pca9468_chg->dc_wq);

	wakeup_source_unregister(pca9468_chg->monitor_wake_lock);

#ifdef CONFIG_THERMAL
	if (pca9468_chg->usb_tzd)
		thermal_zone_device_unregister(pca9468_chg->usb_tzd);
#endif
	if (pca9468_chg->log)
		logbuffer_unregister(pca9468_chg->log);
	pps_free(&pca9468_chg->pps_data);
}

static const struct i2c_device_id pca9468_id[] = {
	{ "pca9468", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, pca9468_id);

#if defined(CONFIG_OF)
static struct of_device_id pca9468_i2c_dt_ids[] = {
	{ .compatible = "nxp,pca9468" },
	{ },
};
MODULE_DEVICE_TABLE(of, pca9468_i2c_dt_ids);
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
		pr_err("%s: unable to open rtc device (%s)\n",
			__FILE__, CONFIG_RTC_HCTOSYS_DEVICE);
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
pca9468_check_and_update_charging_timer(struct pca9468_charger *pca9468)
{
	unsigned long current_time = 0, next_update_time, time_left;

	get_current_time(&current_time);

	if (pca9468->timer_id != TIMER_ID_NONE)	{
		next_update_time = pca9468->last_update_time +
				(pca9468->timer_period / 1000); /* seconds */

		pr_debug("%s: current_time=%ld, next_update_time=%ld\n",
			__func__, current_time, next_update_time);

		if (next_update_time > current_time)
			time_left = next_update_time - current_time;
		else
			time_left = 0;

		mutex_lock(&pca9468->lock);
		pca9468->timer_period = time_left * 1000; /* ms unit */
		mutex_unlock(&pca9468->lock);
		schedule_delayed_work(&pca9468->timer_work,
				msecs_to_jiffies(pca9468->timer_period));

		pr_debug("%s: timer_id=%d, time_period=%ld\n", __func__,
			 pca9468->timer_id, pca9468->timer_period);
	}
	pca9468->last_update_time = current_time;
}
#endif

static int pca9468_suspend(struct device *dev)
{
	struct pca9468_charger *pca9468 = dev_get_drvdata(dev);

	pr_debug("%s: cancel delayed work\n", __func__);

	/* cancel delayed_work */
	cancel_delayed_work(&pca9468->timer_work);
	return 0;
}

static int pca9468_resume(struct device *dev)
{
	struct pca9468_charger *pca9468 = dev_get_drvdata(dev);

	pr_debug("%s: update_timer\n", __func__);

	/* Update the current timer */
#ifdef CONFIG_RTC_HCTOSYS
	pca9468_check_and_update_charging_timer(pca9468);
#else
	if (pca9468->timer_id != TIMER_ID_NONE) {
		mutex_lock(&pca9468->lock);
		pca9468->timer_period = 0;	/* ms unit */
		mutex_unlock(&pca9468->lock);
		schedule_delayed_work(&pca9468->timer_work,
				      msecs_to_jiffies(pca9468->timer_period));
	}
#endif
	return 0;
}
#else
#define pca9468_suspend		NULL
#define pca9468_resume		NULL
#endif

const struct dev_pm_ops pca9468_pm_ops = {
	.suspend = pca9468_suspend,
	.resume = pca9468_resume,
};

static struct i2c_driver pca9468_driver = {
	.driver = {
		.name = "pca9468",
#if defined(CONFIG_OF)
		.of_match_table = pca9468_i2c_dt_ids,
#endif /* CONFIG_OF */
#if defined(CONFIG_PM)
		.pm = &pca9468_pm_ops,
#endif
	},
	.probe        = pca9468_probe,
	.remove       = pca9468_remove,
	.id_table     = pca9468_id,
};

module_i2c_driver(pca9468_driver);

MODULE_AUTHOR("Clark Kim <clark.kim@nxp.com>");
MODULE_AUTHOR("AleX Pelosi <apelosi@google.com>");
MODULE_AUTHOR("Wasb Liu <wasbliu@google.com>");
MODULE_DESCRIPTION("PCA9468 gcharger driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("3.7.0");
