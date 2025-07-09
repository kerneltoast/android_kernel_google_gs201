/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Driver for LN8411 Direct charger
 * Based on PCA9468 driver
 */

#ifndef _LN8411_H_
#define _LN8411_H_

#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/regmap.h>
#include "ln8411_A1.h"

#define WC_DRV_VERSION			"1.0" /* driver version string format */
/* FW register address */
#define FWREG_CHIP_ID_REG		0xA400

#define FWREG_MAX_REGISTER		0xbe

/* error code */
#define E_BUS_R				0x80000001
#define E_BUS_W				0x80000002
#define E_BUS_WR			0x80000003

#define MAX_CMD_SIZE			200

#define LN8411_IIN_CFG_MIN		150000
/* input current step, unit - uA */
#define LN8411_IIN_CFG_STEP		100000
/* input current, unit - uA */
#define LN8411_IIN_CFG(input_curr)	((input_curr) / LN8411_IIN_CFG_STEP)

enum ln8411_keys {
	LN8411_LION_CTRL_UNLOCK = 0x5b,
	LN8411_LION_CTRL_LOCK = 0x0,
	LN8411_LION_CTRL_EN_RESET = 0xc6,
	LN8411_LION_CTRL_EN_SW_OVERRIDE = 0xaa,
};

enum ln8411_adc_chan {
	LN8411_ADC_CHAN_IBUS = 0,
	LN8411_ADC_CHAN_VBUS,
	LN8411_ADC_CHAN_VUSB,
	LN8411_ADC_CHAN_VWPC,
	LN8411_ADC_CHAN_VOUT,
	LN8411_ADC_CHAN_VBAT,
	LN8411_ADC_CHAN_IBAT,
	LN8411_ADC_CHAN_TSBAT,
	LN8411_ADC_CHAN_TDIE,
};

/* ADC Channel */
enum {
	ADCCH_VIN = LN8411_ADC_CHAN_VBUS,
	ADCCH_VOUT = LN8411_ADC_CHAN_VOUT,
	ADCCH_VBAT = LN8411_ADC_CHAN_VBAT,
	ADCCH_IIN = LN8411_ADC_CHAN_IBUS,
	ADCCH_DIETEMP = LN8411_ADC_CHAN_TDIE,
	ADCCH_MAX
};

/* ADC step */
#define VBAT_STEP	LN8411_VBAT_ADC_STEP_UV
#define IIN_STEP	LN8411_IBUS_ADC_STEP_UA
#define DIETEMP_STEP  	(LN8411_TDIE_STEP_DECIC * 100)
#define VFLOAT_STEP	25000
#define CBUS_UCP_STEP	25000
#define SWCAP_OVP_STEP	25000

#endif /* LN8411_H */

