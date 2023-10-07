/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2020-2022 Google LLC
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

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/ctype.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/regmap.h>
#include <linux/thermal.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <misc/gvotable.h>
#include <linux/platform_device.h>
#include "gbms_power_supply.h"
#include "google_bms.h"
#include "max_m5.h"
#include "max77759.h"
#include "max77759_charger.h"

#define BATOILO_DET_30US 0x4
#define MAX77759_DEFAULT_MODE	MAX77759_CHGR_MODE_ALL_OFF
#define CHG_TERM_VOLT_DEBOUNCE	200
#define MAX77759_OTG_5000_MV 5000
#define GS101_OTG_DEFAULT_MV MAX77759_OTG_5000_MV

/* CHG_DETAILS_01:CHG_DTLS */
#define CHGR_DTLS_DEAD_BATTERY_MODE			0x00
#define CHGR_DTLS_FAST_CHARGE_CONST_CURRENT_MODE	0x01
#define CHGR_DTLS_FAST_CHARGE_CONST_VOLTAGE_MODE	0x02
#define CHGR_DTLS_TOP_OFF_MODE				0x03
#define CHGR_DTLS_DONE_MODE				0x04
#define CHGR_DTLS_TIMER_FAULT_MODE			0x06
#define CHGR_DTLS_DETBAT_HIGH_SUSPEND_MODE		0x07
#define CHGR_DTLS_OFF_MODE				0x08
#define CHGR_DTLS_OFF_HIGH_TEMP_MODE			0x0a
#define CHGR_DTLS_OFF_WATCHDOG_MODE			0x0b
#define CHGR_DTLS_OFF_JEITA				0x0c
#define CHGR_DTLS_OFF_TEMP				0x0d

#define CHGR_CHG_CNFG_12_VREG_4P6V			0x1
#define CHGR_CHG_CNFG_12_VREG_4P7V			0x2

static inline int max77759_reg_read(struct regmap *regmap, uint8_t reg,
				    uint8_t *val)
{
	int ret, ival;

	ret = regmap_read(regmap, reg, &ival);
	if (ret == 0)
		*val = 0xFF & ival;

	return ret;
}

static inline int max77759_reg_write(struct regmap *regmap, uint8_t reg,
				     uint8_t val)
{
	return regmap_write(regmap, reg, val);
}

static inline int max77759_readn(struct regmap *regmap, uint8_t reg,
				 uint8_t *val, int count)
{
	return regmap_bulk_read(regmap, reg, val, count);
}

static inline int max77759_writen(struct regmap *regmap, uint8_t reg,
				  const uint8_t *val, int count)
{
	return regmap_bulk_write(regmap, reg, val, count);
}

static inline int max77759_reg_update(struct max77759_chgr_data *data,
				      uint8_t reg, uint8_t msk, uint8_t val)
{
	int ret;
	unsigned tmp;

	mutex_lock(&data->io_lock);
	ret = regmap_read(data->regmap, reg, &tmp);
	if (!ret) {
		tmp &= ~msk;
		tmp |= val;
		ret = regmap_write(data->regmap, reg, tmp);
	}
	mutex_unlock(&data->io_lock);

	return ret;
}

/* ----------------------------------------------------------------------- */

static int max77759_resume_check(struct max77759_chgr_data *data)
{
	int ret = 0;

	pm_runtime_get_sync(data->dev);
	if (!data->init_complete || !data->resume_complete)
		ret = -EAGAIN;
	pm_runtime_put_sync(data->dev);

	return ret;
}

static int max77759_get_vdroop_ok(struct i2c_client *client, bool *state)
{
	struct max77759_chgr_data *data;
	u8 val;

	if (!client)
		return -ENODEV;

	data = i2c_get_clientdata(client);
	if (!data || !data->regmap)
		return -ENODEV;

	if (max77759_resume_check(data))
		return -EAGAIN;

	if (max77759_reg_read(data->regmap, MAX77759_CHG_DETAILS_01, &val) < 0)
		return -EINVAL;

	*state = _chg_details_01_vdroop2_ok_get(val);;
	return 0;
}

static int max77759_get_batoilo_lvl(struct i2c_client *client, unsigned int *lvl)
{
	struct max77759_chgr_data *data;
	u8 val;

	if (!client)
		return -ENODEV;

	data = i2c_get_clientdata(client);
	if (!data || !data->regmap)
		return -ENODEV;

	if (max77759_resume_check(data))
		return -EAGAIN;

	if (max77759_reg_read(data->regmap, MAX77759_CHG_CNFG_14, &val) < 0)
		return -EINVAL;
	*lvl = (BO_STEP * ((val & MAX77759_CHG_CNFG_14_BAT_OILO_MASK)
			>> MAX77759_CHG_CNFG_14_BAT_OILO_SHIFT) + BO_LOWER_LIMIT);
	return 0;
}

static int max77759_set_batoilo_lvl(struct i2c_client *client, unsigned int lvl)
{
	struct max77759_chgr_data *data;
	u8 val;

	if (!client)
		return -ENODEV;

	data = i2c_get_clientdata(client);
	if (!data || !data->regmap)
		return -ENODEV;

	if (max77759_resume_check(data))
		return -EAGAIN;

	/* TODO: use rmw */
	if (max77759_reg_read(data->regmap, MAX77759_CHG_CNFG_14, &val) < 0)
		return -EINVAL;
	val &= ~MAX77759_CHG_CNFG_14_BAT_OILO_MASK;
	val |= (lvl - BO_LOWER_LIMIT) / 200;
	if (max77759_reg_write(data->regmap, MAX77759_CHG_CNFG_14, val) < 0)
		return -EIO;
	return 0;
}

static int max77759_get_uvlo_lvl(struct i2c_client *client, uint8_t mode, unsigned int *lvl)
{
	struct max77759_chgr_data *data;
	u8 val;
	u8 reg;

	if (!client)
		return -ENODEV;

	data = i2c_get_clientdata(client);
	if (!data || !data->regmap)
		return -ENODEV;

	if (max77759_resume_check(data))
		return -EAGAIN;

	reg = (mode == 1) ? MAX77759_CHG_CNFG_15 : MAX77759_CHG_CNFG_16;

	if (max77759_reg_read(data->regmap, reg, &val) < 0)
		return -EINVAL;
	*lvl = VD_STEP * ((val & MAX77759_CHG_CNFG_15_SYS_UVLO1_MASK)
			  >> MAX77759_CHG_CNFG_15_SYS_UVLO1_SHIFT) + VD_LOWER_LIMIT;
	return 0;
}

static int max77759_set_uvlo_lvl(struct i2c_client *client, uint8_t mode, unsigned int lvl)
{
	struct max77759_chgr_data *data;
	u8 val;
	u8 reg;

	if (!client)
		return -ENODEV;

	data = i2c_get_clientdata(client);
	if (!data || !data->regmap)
		return -ENODEV;

	if (max77759_resume_check(data))
		return -EAGAIN;

	reg = (mode == UVLO1) ? MAX77759_CHG_CNFG_15 : MAX77759_CHG_CNFG_16;

	if (max77759_reg_read(data->regmap, reg, &val) < 0)
		return -EINVAL;
	val &= ~MAX77759_CHG_CNFG_15_SYS_UVLO1_MASK;
	val |= (lvl - VD_LOWER_LIMIT) / 50;
	if (max77759_reg_write(data->regmap, reg, val) < 0)
		return -EIO;
	return 0;
}

static const struct bcl_ifpmic_ops bcl_ifpmic_ops = {
	max77759_get_vdroop_ok,
	max77759_set_uvlo_lvl,
	max77759_get_uvlo_lvl,
	max77759_set_batoilo_lvl,
	max77759_get_batoilo_lvl,
};

/* ----------------------------------------------------------------------- */

int max77759_chg_reg_write(struct i2c_client *client, u8 reg, u8 value)
{
	struct max77759_chgr_data *data;

	if (!client)
		return -ENODEV;

	data = i2c_get_clientdata(client);
	if (!data || !data->regmap)
		return -ENODEV;

	return max77759_reg_write(data->regmap, reg, value);
}
EXPORT_SYMBOL_GPL(max77759_chg_reg_write);

int max77759_chg_reg_read(struct i2c_client *client, u8 reg, u8 *value)
{
	struct max77759_chgr_data *data;

	if (!client)
		return -ENODEV;

	data = i2c_get_clientdata(client);
	if (!data || !data->regmap)
		return -ENODEV;

	return max77759_reg_read(data->regmap, reg, value);
}
EXPORT_SYMBOL_GPL(max77759_chg_reg_read);

int max77759_chg_reg_update(struct i2c_client *client,
			    u8 reg, u8 mask, u8 value)
{
	struct max77759_chgr_data *data;

	if (!client)
		return -ENODEV;

	data = i2c_get_clientdata(client);
	if (!data || !data->regmap)
		return -ENODEV;

	return regmap_write_bits(data->regmap, reg, mask, value);
}
EXPORT_SYMBOL_GPL(max77759_chg_reg_update);

int max77759_chg_mode_write(struct i2c_client *client,
			    enum max77759_charger_modes mode)
{
	struct max77759_chgr_data *data;

	if (!client)
		return -ENODEV;

	data = i2c_get_clientdata(client);
	if (!data || !data->regmap)
		return -ENODEV;

	return regmap_write_bits(data->regmap, MAX77759_CHG_CNFG_00,
				 MAX77759_CHG_CNFG_00_MODE_MASK,
				 mode);
}
EXPORT_SYMBOL_GPL(max77759_chg_mode_write);

/* 1 if changed, 0 if not changed, or < 0 on error */
static int max77759_chg_prot(struct regmap *regmap, bool enable)
{
	u8 value = enable ? 0 : MAX77759_CHG_CNFG_06_CHGPROT_MASK;
	u8 prot;
	int ret;

	ret = max77759_reg_read(regmap, MAX77759_CHG_CNFG_06, &prot);
	if (ret < 0)
		return -EIO;

	if ((prot & MAX77759_CHG_CNFG_06_CHGPROT_MASK) == value)
		return 0;

	ret = regmap_write_bits(regmap, MAX77759_CHG_CNFG_06,
				MAX77759_CHG_CNFG_06_CHGPROT_MASK,
				value);
	if (ret < 0)
		return -EIO;

	return 1;
}

int max77759_chg_insel_write(struct i2c_client *client, u8 mask, u8 value)
{
	struct max77759_chgr_data *data;
	int ret, prot;

	if (!client)
		return -ENODEV;

	data = i2c_get_clientdata(client);
	if (!data || !data->regmap)
		return -ENODEV;

	prot = max77759_chg_prot(data->regmap, false);
	if (prot < 0)
		return -EIO;

	/* changing [CHGIN|WCIN]_INSEL: works when protection is disabled  */
	ret = regmap_write_bits(data->regmap, MAX77759_CHG_CNFG_12, mask, value);
	if (ret < 0 || prot == 0)
		return ret;

	prot = max77759_chg_prot(data->regmap, true);
	if (prot < 0) {
		pr_err("%s: cannot restore protection bits (%d)\n",
		       __func__, prot);
		return prot;
	};

	return ret;
}
EXPORT_SYMBOL_GPL(max77759_chg_insel_write);

int max77759_chg_insel_read(struct i2c_client *client, u8 *value)
{
	struct max77759_chgr_data *data;

	if (!client)
		return -ENODEV;

	data = i2c_get_clientdata(client);
	if (!data || !data->regmap)
		return -ENODEV;

	return  max77759_reg_read(data->regmap, MAX77759_CHG_CNFG_12, value);
}
EXPORT_SYMBOL_GPL(max77759_chg_insel_read);

/* ----------------------------------------------------------------------- */

static int max77759_find_pmic(struct max77759_chgr_data *data)
{
	struct device_node *dn;

	if (data->pmic_i2c_client)
		return 0;

	dn = of_parse_phandle(data->dev->of_node, "max77759,pmic", 0);
	if (!dn)
		return -ENXIO;

	data->pmic_i2c_client = of_find_i2c_device_by_node(dn);
	if (!data->pmic_i2c_client)
		return -EAGAIN;

	return 0;
}

static int max77759_find_fg(struct max77759_chgr_data *data)
{
	struct device_node *dn;

	if (data->fg_i2c_client)
		return 0;

	dn = of_parse_phandle(data->dev->of_node, "max77759,max_m5", 0);
	if (!dn)
		return -ENXIO;

	data->fg_i2c_client = of_find_i2c_device_by_node(dn);
	if (!data->fg_i2c_client)
		return -EAGAIN;

	return 0;
}

static int max77759_read_vbatt(struct max77759_chgr_data *data, int *vbatt)
{
	int ret;

	ret = max77759_find_fg(data);
	if (ret == 0)
		ret = max1720x_get_voltage_now(data->fg_i2c_client, vbatt);

	return ret;
}

static int max77759_read_vbyp(struct max77759_chgr_data *data, int *vbyp)
{
	int ret;

	ret = max77759_find_fg(data);
	if (ret == 0)
		ret = max_m5_read_vbypass(data->fg_i2c_client, vbyp);

	return ret;
}

/* ----------------------------------------------------------------------- */

/* set WDTEN in CHG_CNFG_18 (0xCB), tWD = 80s */
static int max77759_wdt_enable(struct max77759_chgr_data *data, bool enable)
{
	int ret;
	u8 reg;

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_CNFG_18, &reg);
	if (ret < 0)
		return -EIO;

	if ((!!_chg_cnfg_18_wdten_get(reg)) == enable)
		return 0;

	/* this register is protected, read back to check if it worked */
	reg = _chg_cnfg_18_wdten_set(reg, enable);
	ret = max77759_reg_write(data->regmap, MAX77759_CHG_CNFG_18, reg);
	if (ret < 0)
		return -EIO;

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_CNFG_18, &reg);
	if (ret < 0)
		return -EIO;

	return (ret == 0 && (!!_chg_cnfg_18_wdten_get(reg)) == enable) ?
		0 : -EINVAL;
}

/* First step to convert votes to a usecase and a setting for mode */
static int max77759_foreach_callback(void *data, const char *reason,
				     void *vote)
{
	struct max77759_foreach_cb_data *cb_data = data;
	int mode = (long)vote; /* max77759_mode is an int election */

	switch (mode) {
	/* Direct raw modes last come fist served */
	case MAX77759_CHGR_MODE_ALL_OFF:
	case MAX77759_CHGR_MODE_BUCK_ON:
	case MAX77759_CHGR_MODE_CHGR_BUCK_ON:
	case MAX77759_CHGR_MODE_BOOST_UNO_ON:
	case MAX77759_CHGR_MODE_BOOST_ON:
	case MAX77759_CHGR_MODE_OTG_BOOST_ON:
	case MAX77759_CHGR_MODE_BUCK_BOOST_UNO_ON:
	case MAX77759_CHGR_MODE_CHGR_BUCK_BOOST_UNO_ON:
	case MAX77759_CHGR_MODE_OTG_BUCK_BOOST_ON:
	case MAX77759_CHGR_MODE_CHGR_OTG_BUCK_BOOST_ON:
		pr_debug("%s: RAW vote=0x%x\n", __func__, mode);
		if (cb_data->use_raw)
			break;
		cb_data->raw_value = mode;
		cb_data->reason = reason;
		cb_data->use_raw = true;
		break;

	/* temporary, can be used to program the WLC chip, remove */
	case GBMS_CHGR_MODE_BOOST_UNO_ON:
		if (!cb_data->boost_on || !cb_data->uno_on)
			cb_data->reason = reason;
		pr_debug("%s: BOOST_UNO vote=0x%x\n", __func__, mode);
		cb_data->boost_on += 1;
		cb_data->uno_on += 1;
		break;

	/* SYSTEM modes can add complex transactions */

	/* MAX77759: on disconnect */
	case GBMS_CHGR_MODE_STBY_ON:
		if (!cb_data->stby_on)
			cb_data->reason = reason;
		pr_debug("%s: STBY_ON %s vote=0x%x\n",
			 __func__, reason ? reason : "<>", mode);
		cb_data->stby_on += 1;
		break;
	/* USB+WLCIN, factory only */
	case GBMS_CHGR_MODE_USB_WLC_RX:
		pr_debug("%s: USB_WLC_RX %s vote=0x%x\n",
			 __func__, reason ? reason : "<>", mode);
		if (!cb_data->usb_wlc)
			cb_data->reason = reason;
		cb_data->usb_wlc += 1;
		break;

	/* input_suspend => 0 ilim */
	case GBMS_CHGR_MODE_CHGIN_OFF:
		if (!cb_data->chgin_off)
			cb_data->reason = reason;
		pr_debug("%s: CHGIN_OFF %s vote=0x%x\n", __func__,
			 reason ? reason : "<>", mode);
		cb_data->chgin_off += 1;
		break;
	/* input_suspend => DC_SUSPEND */
	case GBMS_CHGR_MODE_WLCIN_OFF:
		if (!cb_data->wlcin_off)
			cb_data->reason = reason;
		pr_debug("%s: WLCIN_OFF %s vote=0x%x\n", __func__,
			 reason ? reason : "<>", mode);
		cb_data->wlcin_off += 1;
		break;
	/* MAX77759: charging on via CC_MAX (needs inflow, buck_on on) */
	case GBMS_CHGR_MODE_CHGR_BUCK_ON:
		if (!cb_data->chgr_on)
			cb_data->reason = reason;
		pr_debug("%s: CHGR_BUCK_ON %s vote=0x%x\n", __func__,
			 reason ? reason : "<>", mode);
		cb_data->chgr_on += 1;
		break;

	/* USB: present, charging controlled via GBMS_CHGR_MODE_CHGR_BUCK_ON */
	case GBMS_USB_BUCK_ON:
		if (!cb_data->buck_on)
			cb_data->reason = reason;
		pr_debug("%s: BUCK_ON %s vote=0x%x\n", __func__,
			 reason ? reason : "<>", mode);
		cb_data->buck_on += 1;
		break;
	/* USB: OTG, source, fast role swap case */
	case GBMS_USB_OTG_FRS_ON:
		if (!cb_data->frs_on)
			cb_data->reason = reason;
		pr_debug("%s: FRS_ON vote=0x%x\n", __func__, mode);
		cb_data->frs_on += 1;
		break;
	/* USB: boost mode, source, normally external boost */
	case GBMS_USB_OTG_ON:
		if (!cb_data->otg_on)
			cb_data->reason = reason;
		pr_debug("%s: OTG_ON %s vote=0x%x\n", __func__,
			 reason ? reason : "<>", mode);
		cb_data->otg_on += 1;
		break;
	/* DC Charging: mode=0, set CP_EN */
	case GBMS_CHGR_MODE_CHGR_DC:
		if (!cb_data->dc_on)
			cb_data->reason = reason;
		pr_debug("%s: DC_ON vote=0x%x\n", __func__, mode);
		cb_data->dc_on += 1;
		break;
	/* WLC Tx */
	case GBMS_CHGR_MODE_WLC_TX:
		if (!cb_data->wlc_tx)
			cb_data->reason = reason;
		pr_debug("%s: WLC_TX vote=%x\n", __func__, mode);
		cb_data->wlc_tx += 1;
		break;
	/* pogo vin */
	case GBMS_POGO_VIN:
		if (!cb_data->pogo_vin)
			cb_data->reason = reason;
		pr_debug("%s: POGO VIN vote=%x\n", __func__, mode);
		cb_data->pogo_vin += 1;
		break;
	/* pogo vout */
	case GBMS_POGO_VOUT:
		if (!cb_data->pogo_vout)
			cb_data->reason = reason;
		pr_debug("%s: POGO VOUT vote=%x\n", __func__, mode);
		cb_data->pogo_vout += 1;
		break;

	default:
		pr_err("mode=%x not supported\n", mode);
		break;
	}

	return 0;
}

#define cb_data_is_inflow_off(cb_data) \
	((cb_data)->chgin_off && (cb_data)->wlcin_off)

/*
 * It could use cb_data->charge_done to turn off charging.
 * TODO: change chgr_on=>2 to (cc_max && chgr_ena)
 */
static bool cb_data_is_chgr_on(const struct max77759_foreach_cb_data *cb_data)
{
	return cb_data->stby_on ? 0 : (cb_data->chgr_on >= 2);
}

/*
 * Case	USB_chg USB_otg	WLC_chg	WLC_TX	PMIC_Charger	Ext_B	LSxx	Name
 * -------------------------------------------------------------------------------------
 * 4-1	0	1	10	0	IF-PMIC-WCIN	1	1/0	USB_OTG_WLC_RX
 * 4-2	0	1	01	0	DC WCIN		1	1/0	USB_OTG_WLC_DC
 * 5-1	0	1	0	0	0		1	1/0	USB_OTG
 * 5-2	0	1	0	0	OTG_5V		0	0/0	USB_OTG_FRS
 * 7-2	0	1	0	1	OTG_5V		2	0/1	USB_OTG_WLC_TX
 * -------------------------------------------------------------------------------------
 * Ext_Boost = 0 off, 1 = OTG 5V, 2 = WTX 7.5
 * WLC_chg = 0 off, 1 = on, 2 = PPS
 *
 * NOTE: do not call with (cb_data->wlc_rx && cb_data->wlc_tx)
 */
static int max77759_get_otg_usecase(struct max77759_foreach_cb_data *cb_data)
{
	const int chgr_on = cb_data_is_chgr_on(cb_data);
	bool dc_on = cb_data->dc_on; /* && !cb_data->charge_done */
	int usecase;
	u8 mode;

	/* invalid, cannot do OTG stuff with USB power */
	if (cb_data->buck_on) {
		pr_err("%s: buck_on with OTG\n", __func__);
		return -EINVAL;
	}

	/* pure OTG defaults to ext boost */
	if (cb_data->pogo_vout) {
		usecase = GSU_MODE_USB_OTG_POGO_VOUT;
		mode = MAX77759_CHGR_MODE_OTG_BOOST_ON;
	} else if (!cb_data->wlc_rx && !cb_data->wlc_tx) {
		/* 5-1: USB_OTG or  5-2: USB_OTG_FRS */

		if (cb_data->frs_on) {
			usecase = GSU_MODE_USB_OTG_FRS;
			mode = MAX77759_CHGR_MODE_OTG_BOOST_ON;
		} else {
			usecase = GSU_MODE_USB_OTG;
			mode = MAX77759_CHGR_MODE_ALL_OFF;
		}

		/* b/188730136  OTG cases with DC on */
		if (dc_on)
			pr_err("%s: TODO enable pps+OTG\n", __func__);
	} else if (cb_data->wlc_tx) {
		/* 7-2: WLC_TX -> WLC_TX + OTG */
		usecase = GSU_MODE_USB_OTG_WLC_TX;
		mode = MAX77759_CHGR_MODE_OTG_BOOST_ON;
	} else if (cb_data->wlc_rx) {
		usecase = GSU_MODE_USB_OTG_WLC_RX;
		if (chgr_on)
			mode = MAX77759_CHGR_MODE_CHGR_BUCK_ON;
		else
			mode = MAX77759_CHGR_MODE_BUCK_ON;
	} else if (dc_on) {
		return -EINVAL;
	} else {
		return -EINVAL;
	}

	cb_data->reg = _chg_cnfg_00_cp_en_set(cb_data->reg, dc_on);
	cb_data->reg = _chg_cnfg_00_mode_set(cb_data->reg, mode);
	return usecase;
}

/*
 * Determines the use case to switch to. This is device/system dependent and
 * will likely be factored to a separate file (compile module).
 */
static int max77759_get_usecase(struct max77759_foreach_cb_data *cb_data,
				struct max77759_usecase_data *uc_data)
{
	const int buck_on = cb_data->chgin_off ? 0 : cb_data->buck_on;
	const int chgr_on = cb_data_is_chgr_on(cb_data);
	bool wlc_tx = cb_data->wlc_tx != 0;
	bool wlc_rx = cb_data->wlc_rx != 0;
	bool dc_on = cb_data->dc_on; /* && !cb_data->charge_done */
	int usecase;
	u8 mode;

	/* consistency check, TOD: add more */
	if (wlc_tx) {
		if (wlc_rx) {
			pr_err("%s: wlc_tx and wlc_rx\n", __func__);
			return -EINVAL;
		}

		if (uc_data->ext_otg_only && cb_data->otg_on) {
			pr_warn("%s: no wlc_tx with otg_on for now\n", __func__);
			wlc_tx = 0;
			cb_data->wlc_tx = 0;
		}
	}

	/* TODO: GSU_MODE_USB_OTG_WLC_DC */
	if (dc_on && cb_data->wlc_rx)
		cb_data->otg_on = 0;

	/* OTG modes override the others, might need to move under usb_wlc */
	if (cb_data->otg_on || cb_data->frs_on)
		return max77759_get_otg_usecase(cb_data);

	/* USB will disable wlc_rx */
	if (cb_data->buck_on && !uc_data->dcin_is_dock)
		wlc_rx = false;

	/* buck_on is wired, wlc_rx is wireless, might still need rTX */
	if (cb_data->usb_wlc) {
		/* USB+WLC for factory and testing */
		usecase = GSU_MODE_USB_WLC_RX;
		mode = MAX77759_CHGR_MODE_CHGR_BUCK_ON;
	} else if (cb_data->pogo_vout) {

		if (!buck_on) {
			mode = MAX77759_CHGR_MODE_ALL_OFF;
			usecase = GSU_MODE_POGO_VOUT;
		} else if (chgr_on) {
			mode = MAX77759_CHGR_MODE_CHGR_BUCK_ON;
			usecase = GSU_MODE_USB_CHG_POGO_VOUT;
		} else {
			mode = MAX77759_CHGR_MODE_BUCK_ON;
			usecase = GSU_MODE_USB_CHG_POGO_VOUT;
		}

	} else if (!buck_on && !wlc_rx) {
		mode = MAX77759_CHGR_MODE_ALL_OFF;

		/* Rtx using the internal battery */
		usecase = GSU_MODE_STANDBY;
		if (wlc_tx)
			usecase = GSU_MODE_WLC_TX;

		/* here also on WLC_DC->WLC_DC+USB */
		dc_on = false;
	} else if (wlc_tx) {

		/* Disable DC when Rtx is on will handle by dc_avail_votable */
		if (!buck_on) {
			mode = MAX77759_CHGR_MODE_ALL_OFF;
			usecase = GSU_MODE_WLC_TX;
		} else if (chgr_on) {
			mode = MAX77759_CHGR_MODE_CHGR_BUCK_ON;
			usecase = GSU_MODE_USB_CHG_WLC_TX;
		} else {
			mode = MAX77759_CHGR_MODE_BUCK_ON;
			usecase = GSU_MODE_USB_CHG_WLC_TX;
		}

	} else if (wlc_rx) {

		/* will be in mode 4 if in stby unless dc is enabled */
		if (chgr_on) {
			mode = MAX77759_CHGR_MODE_CHGR_BUCK_ON;
			usecase = GSU_MODE_WLC_RX;
		} else {
			mode = MAX77759_CHGR_MODE_BUCK_ON;
			usecase = GSU_MODE_WLC_RX;
		}

		/* wired input should be disabled here */
		if (dc_on) {
			mode = MAX77759_CHGR_MODE_ALL_OFF;
			usecase = GSU_MODE_WLC_DC;
		}

		if (uc_data->dcin_is_dock)
			usecase = GSU_MODE_DOCK;

	} else {

		/* MODE_BUCK_ON is inflow */
		if (chgr_on) {
			mode = MAX77759_CHGR_MODE_CHGR_BUCK_ON;
			usecase = GSU_MODE_USB_CHG;
		} else {
			mode = MAX77759_CHGR_MODE_BUCK_ON;
			usecase = GSU_MODE_USB_CHG;
		}

		/*
		 * NOTE: OTG cases handled in max77759_get_otg_usecase()
		 * NOTE: usecases with !(buck|wlc)_on same as.
		 * NOTE: mode=0 if standby, mode=5 if charging, mode=0xa on otg
		 * TODO: handle rTx + DC and some more.
		 */
		if (dc_on && cb_data->wlc_rx) {
			/* WLC_DC->WLC_DC+USB -> ignore dc_on */
		} else if (dc_on) {
			mode = MAX77759_CHGR_MODE_ALL_OFF;
			usecase = GSU_MODE_USB_DC;
		} else if (cb_data->stby_on && !chgr_on) {
			mode = MAX77759_CHGR_MODE_ALL_OFF;
			usecase = GSU_MODE_STANDBY;
		}

	}

	if (!cb_data->dc_avail_votable)
		cb_data->dc_avail_votable = gvotable_election_get_handle(VOTABLE_DC_CHG_AVAIL);
	if (cb_data->dc_avail_votable)
		gvotable_cast_int_vote(cb_data->dc_avail_votable,
				       "WLC_TX", wlc_tx? 0 : 1, wlc_tx);

	/* reg might be ignored later */
	cb_data->reg = _chg_cnfg_00_cp_en_set(cb_data->reg, dc_on);
	cb_data->reg = _chg_cnfg_00_mode_set(cb_data->reg, mode);

	return usecase;
}

static void max77759_set_pogo_ovp_en(struct max77759_usecase_data *uc_data,
				     const int enabled)
{
	const int gpio_en = gpio_get_value_cansleep(uc_data->pogo_ovp_en);
	const bool pogo_ovp_en = uc_data->pogo_ovp_en_act_low ?
				 (gpio_en == 0) : (gpio_en == 1);

	/* return if pogo_ovp_en has been set */
	if ((enabled && pogo_ovp_en) || (!enabled && !pogo_ovp_en))
		return;

	/* turn on/off pogo_ovp_en */
	gpio_set_value_cansleep(uc_data->pogo_ovp_en, enabled ?
				!uc_data->pogo_ovp_en_act_low :
				uc_data->pogo_ovp_en_act_low);
}

/*
 * adjust *INSEL (only one source can be enabled at a given time)
 * NOTE: providing compatibility with input_suspend makes this more complex
 * that it needs to be.
 */
static int max77759_set_insel(struct max77759_chgr_data *data,
			      struct max77759_usecase_data *uc_data,
			      const struct max77759_foreach_cb_data *cb_data,
			      int from_uc, int use_case)
{
	const u8 insel_mask = MAX77759_CHG_CNFG_12_CHGINSEL_MASK |
			      MAX77759_CHG_CNFG_12_WCINSEL_MASK;
	int wlc_on = cb_data->wlc_tx && !cb_data->dc_on;
	bool force_wlc = false;
	u8 insel_value = 0;
	int ret;

	if (cb_data->usb_wlc) {
		insel_value |= MAX77759_CHG_CNFG_12_WCINSEL;
		force_wlc = true;
	} else if (cb_data_is_inflow_off(cb_data)) {
		/*
		 * input_suspend masks both inputs but must still allow
		 * TODO: use a separate use case for usb + wlc
		 */
		 force_wlc = true;
	} else if (cb_data->buck_on && !cb_data->chgin_off) {
		insel_value |= MAX77759_CHG_CNFG_12_CHGINSEL;
	} else if (cb_data->wlc_rx && !cb_data->wlcin_off) {

		/* always disable WLC when USB is present */
		if (!cb_data->buck_on)
			insel_value |= MAX77759_CHG_CNFG_12_WCINSEL;
		else
			force_wlc = true;

	} else if (cb_data->otg_on) {
		/* all OTG cases MUST mask CHGIN */
		insel_value |= MAX77759_CHG_CNFG_12_WCINSEL;
	} else {
		/* disconnected, do not enable chgin if in input_suspend */
		if (!cb_data->chgin_off)
			insel_value |= MAX77759_CHG_CNFG_12_CHGINSEL;

		/* disconnected, do not enable wlc_in if in input_suspend */
		if (!cb_data->buck_on && (!cb_data->wlcin_off || cb_data->wlc_tx))
			insel_value |= MAX77759_CHG_CNFG_12_WCINSEL;

		force_wlc = true;
	}

	if (cb_data->pogo_vout) {
		/* always disable WCIN when pogo power out */
		insel_value &= ~MAX77759_CHG_CNFG_12_WCINSEL;
		/* turn off pogo_ovp */
		if (uc_data->pogo_ovp_en > 0)
			gpio_set_value_cansleep(uc_data->pogo_ovp_en, uc_data->pogo_ovp_en_act_low);
	} else if (cb_data->pogo_vin && !cb_data->wlcin_off) {
		/* always disable USB when Dock is present */
		insel_value &= ~MAX77759_CHG_CNFG_12_CHGINSEL;
		insel_value |= MAX77759_CHG_CNFG_12_WCINSEL;
	}

	if (from_uc != use_case || force_wlc || wlc_on) {
		wlc_on = wlc_on || (insel_value & MAX77759_CHG_CNFG_12_WCINSEL) != 0;

		/* b/182973431 disable WLC_IC while CHGIN, rtx will enable WLC later */
		ret = gs101_wlc_en(uc_data, wlc_on);
		if (ret < 0)
			pr_err("%s: error wlc_en=%d ret:%d\n", __func__,
			       wlc_on, ret);
	} else {
		u8 value = 0;

		wlc_on = max77759_chg_insel_read(uc_data->client, &value);
		if (wlc_on == 0)
			wlc_on = (value & MAX77759_CHG_CNFG_12_WCINSEL) != 0;
	}

	/* changing [CHGIN|WCIN]_INSEL: works when protection is disabled  */
	ret = max77759_chg_insel_write(uc_data->client, insel_mask, insel_value);

	if (uc_data->pogo_ovp_en > 0)
		max77759_set_pogo_ovp_en(uc_data, cb_data->pogo_vin);

	pr_debug("%s: usecase=%d->%d mask=%x insel=%x wlc_on=%d force_wlc=%d (%d)\n",
		 __func__, from_uc, use_case, insel_mask, insel_value, wlc_on,
		 force_wlc, ret);

	return ret;
}

/* switch to a use case, handle the transitions */
static int max77759_set_usecase(struct max77759_chgr_data *data,
				const struct max77759_foreach_cb_data *cb_data,
				int use_case)
{
	struct max77759_usecase_data *uc_data = &data->uc_data;
	const int from_uc = uc_data->use_case;
	int ret;

	if (uc_data->is_a1 == -1) {

		ret = max77759_find_pmic(data);
		if (ret == 0) {
			u8 id, rev;

			ret = max777x9_pmic_get_id(data->pmic_i2c_client, &id, &rev);
			if (ret == 0)
				uc_data->is_a1 = id == MAX77759_PMIC_PMIC_ID_MW &&
						 rev >= MAX77759_PMIC_REV_A0;
		}
	}

	/* Need this only for for usecases that control the switches */
	if (!uc_data->init_done) {
		uc_data->init_done = gs101_setup_usecases(uc_data, data->dev->of_node);

		dev_info(data->dev, "bst_on:%d, bst_sel:%d, ext_bst_ctl:%d lsw1_o:%d lsw1_c:%d\n",
			 uc_data->bst_on, uc_data->bst_sel, uc_data->ext_bst_ctl,
			 uc_data->lsw1_is_open, uc_data->lsw1_is_closed);
	}

	/* always fix/adjust insel (solves multiple input_suspend) */
	ret = max77759_set_insel(data, uc_data, cb_data, from_uc, use_case);
	if (ret < 0) {
		dev_err(data->dev, "use_case=%d->%d set_insel failed ret:%d\n",
			from_uc, use_case, ret);
		return ret;
	}

	/* usbchg+wlctx will call _set_insel() multiple times. */
	if (from_uc == use_case)
		goto exit_done;

	/* transition to STBY if requested from the use case. */
	ret = gs101_to_standby(uc_data, use_case);
	if (ret < 0) {
		dev_err(data->dev, "use_case=%d->%d to_stby failed ret:%d\n",
			from_uc, use_case, ret);
		return ret;
	}

	/* transition from data->use_case to use_case */
	ret = gs101_to_usecase(uc_data, use_case);
	if (ret < 0) {
		dev_err(data->dev, "use_case=%d->%d to_usecase failed ret:%d\n",
			from_uc, use_case, ret);
		return ret;
	}

exit_done:

	/* finally set mode register */
	ret = max77759_chg_reg_write(uc_data->client, MAX77759_CHG_CNFG_00,
				     cb_data->reg);
	pr_debug("%s: CHARGER_MODE=%x ret:%x\n", __func__, cb_data->reg, ret);
	if (ret < 0) {
		dev_err(data->dev,  "use_case=%d->%d CNFG_00=%x failed ret:%d\n",
			from_uc, use_case, cb_data->reg, ret);
		return ret;
	}

	return ret;
}

static int max77759_wcin_is_online(struct max77759_chgr_data *data);

/*
 * I am using a the comparator_none, need scan all the votes to determine
 * the actual.
 */
static int max77759_mode_callback(struct gvotable_election *el,
				  const char *trigger, void *value)
{
	struct max77759_chgr_data *data = gvotable_get_data(el);
	const int from_use_case = data->uc_data.use_case;
	struct max77759_foreach_cb_data cb_data = { 0 };
	const char *reason;
	int use_case, ret;
	bool nope, rerun = false;
	u8 reg = 0;

	__pm_stay_awake(data->usecase_wake_lock);
	mutex_lock(&data->io_lock);

	reason = trigger;
	use_case = data->uc_data.use_case;

	if (max77759_resume_check(data)) {
		schedule_delayed_work(&data->mode_rerun_work, msecs_to_jiffies(50));
		rerun = true;
		goto unlock_done;
	}

	/* no caching */
	ret = max77759_reg_read(data->regmap, MAX77759_CHG_CNFG_00, &reg);
	if (ret < 0) {
		dev_err(data->dev, "cannot read CNFG_00 (%d)\n", ret);
		goto unlock_done;
	}

	/* Need to switch to MW (turn off dc_on) and enforce no charging  */
	cb_data.charge_done = data->charge_done;

	/* this is the last vote of the election */
	cb_data.reg = reg;	/* current */
	cb_data.el = el;	/* election */

	/* read directly instead of using the vote */
	cb_data.wlc_rx = max77759_wcin_is_online(data) &&
			 !data->wcin_input_suspend;
	cb_data.wlcin_off = !!data->wcin_input_suspend;

	pr_debug("%s: wcin_is_online=%d data->wcin_input_suspend=%d\n", __func__,
		  max77759_wcin_is_online(data), data->wcin_input_suspend);

	/* now scan all the reasons, accumulate in cb_data */
	gvotable_election_for_each(el, max77759_foreach_callback, &cb_data);

	nope = !cb_data.use_raw && !cb_data.stby_on && !cb_data.dc_on &&
	       !cb_data.chgr_on && !cb_data.buck_on && ! cb_data.boost_on &&
	       !cb_data.otg_on && !cb_data.uno_on && !cb_data.wlc_tx &&
	       !cb_data.wlc_rx && !cb_data.wlcin_off && !cb_data.chgin_off &&
	       !cb_data.usb_wlc && !cb_data.pogo_vout && !cb_data.pogo_vin;
	if (nope) {
		pr_debug("%s: nope callback\n", __func__);
		goto unlock_done;
	}

	dev_info(data->dev, "%s:%s full=%d raw=%d stby_on=%d, dc_on=%d, chgr_on=%d, buck_on=%d,"
		" boost_on=%d, otg_on=%d, uno_on=%d wlc_tx=%d wlc_rx=%d usb_wlc=%d"
		" chgin_off=%d wlcin_off=%d frs_on=%d pogo_vout=%d pogo_vin=%d\n",
		__func__, trigger ? trigger : "<>",
		data->charge_done, cb_data.use_raw, cb_data.stby_on, cb_data.dc_on,
		cb_data.chgr_on, cb_data.buck_on, cb_data.boost_on, cb_data.otg_on,
		cb_data.uno_on, cb_data.wlc_tx, cb_data.wlc_rx, cb_data.usb_wlc,
		cb_data.chgin_off, cb_data.wlcin_off, cb_data.frs_on, cb_data.pogo_vout,
		cb_data.pogo_vin);

	/* just use raw "as is", no changes to switches etc */
	if (cb_data.use_raw) {
		cb_data.reg = cb_data.raw_value;
		use_case = GSU_RAW_MODE;
	} else {
		struct max77759_usecase_data *uc_data = &data->uc_data;
		bool use_internal_bst;

		/* insel needs it, otg usecases needs it */
		if (!uc_data->init_done) {
			uc_data->init_done = gs101_setup_usecases(uc_data,
						data->dev->of_node);
			gs101_dump_usecasase_config(uc_data);
		}

		/*
		 * force FRS if ext boost or NBC is not enabled
		 * TODO: move to setup_usecase
		 */
		use_internal_bst = uc_data->vin_is_valid < 0 &&
				   uc_data->bst_on < 0;
		if (cb_data.otg_on && use_internal_bst)
			cb_data.frs_on = cb_data.otg_on;

		/* figure out next use case if not in raw mode */
		use_case = max77759_get_usecase(&cb_data, uc_data);
		if (use_case < 0) {
			dev_err(data->dev, "no valid use case %d\n", use_case);
			goto unlock_done;
		}

		dev_dbg(data->dev, "fccm_reset=%d data->otg_changed=%d cb_data.otg_on=%d\n",
			data->otg_fccm_reset, data->otg_changed, cb_data.otg_on);

		/* re-set FCCM mode for OTG */
		if (data->otg_fccm_reset && data->otg_changed != cb_data.otg_on) {
			data->otg_changed = cb_data.otg_on;
			/*
			 * cb_data.otg_on is disabled in max77759_get_usecase
			 * for GSU_MODE_USB_OTG_WLC_DC.
			 */
			if (cb_data.otg_on) {
				schedule_delayed_work(&data->otg_fccm_worker,
						      msecs_to_jiffies(0));
			} else {
				cancel_delayed_work_sync(&data->otg_fccm_worker);

				/* Force to reset the FCCM mode to disable */
				if (data->uc_data.ext_bst_mode > 0)
					gpio_set_value_cansleep(data->uc_data.ext_bst_mode, 0);
				__pm_relax(data->otg_fccm_wake_lock);
			}
		}
	}

	/* state machine that handle transition between states */
	ret = max77759_set_usecase(data, &cb_data, use_case);
	if (ret < 0) {
		struct max77759_usecase_data *uc_data = &data->uc_data;

		ret = gs101_force_standby(uc_data);
		if (ret < 0) {
			dev_err(data->dev, "use_case=%d->%d force_stby failed ret:%d\n",
				data->uc_data.use_case, use_case, ret);
			goto unlock_done;
		}

		cb_data.reg = MAX77759_CHGR_MODE_ALL_OFF;
		cb_data.reason = "error";
		use_case = GSU_MODE_STANDBY;
	}

	/* the election is an int election */
	if (cb_data.reason)
		reason = cb_data.reason;
	if (!reason)
		reason = "<>";

	/* this changes the trigger */
	ret = gvotable_election_set_result(el, reason, (void*)(uintptr_t)cb_data.reg);
	if (ret < 0) {
		dev_err(data->dev, "cannot update election %d\n", ret);
		goto unlock_done;
	}

	/* mode */
	data->uc_data.use_case = use_case;

unlock_done:
	if (!rerun)
		dev_info(data->dev, "%s:%s use_case=%d->%d CHG_CNFG_00=%x->%x\n",
			 __func__, trigger ? trigger : "<>",
			 from_use_case, use_case,
			 reg, cb_data.reg);
	else
		dev_info(data->dev, "%s:%s vote before resume complete\n",
			 __func__, trigger ? trigger : "<>");
	mutex_unlock(&data->io_lock);
	__pm_relax(data->usecase_wake_lock);
	return 0;
}

#define MODE_RERUN	"RERUN"
static void max77759_mode_rerun_work(struct work_struct *work)
{
	struct max77759_chgr_data *data = container_of(work, struct max77759_chgr_data,
						       mode_rerun_work.work);

	/* TODO: add rerun election API for this b/223089247 */
	max77759_mode_callback(data->mode_votable, MODE_RERUN, NULL);

	return;
}

static int max77759_get_charge_enabled(struct max77759_chgr_data *data,
				       int *enabled)
{
	int ret;
	const void *vote = (const void *)0;

	ret = gvotable_get_current_vote(data->mode_votable, &vote);
	if (ret < 0)
		return ret;

	switch ((uintptr_t)vote) {
	case MAX77759_CHGR_MODE_CHGR_BUCK_ON:
	case MAX77759_CHGR_MODE_CHGR_BUCK_BOOST_UNO_ON:
	case MAX77759_CHGR_MODE_CHGR_OTG_BUCK_BOOST_ON:
		*enabled = 1;
		break;
	default:
		*enabled = 0;
		break;
	}

	return ret;
}

/* reset charge_done if needed on cc_max!=0 and on charge_disable(false) */
static int max77759_enable_sw_recharge(struct max77759_chgr_data *data,
				       bool force)
{
	struct max77759_usecase_data *uc_data = &data->uc_data;
	const bool charge_done = data->charge_done;
	bool needs_restart = force || data->charge_done;
	uint8_t reg;
	int ret;

	if(max77759_resume_check(data))
		return -EAGAIN;

	if (!needs_restart) {
		ret = max77759_reg_read(data->regmap, MAX77759_CHG_DETAILS_01, &reg);
		needs_restart = (ret < 0) ||
				_chg_details_01_chg_dtls_get(reg) == CHGR_DTLS_DONE_MODE;
		if (!needs_restart)
			return 0;
	}

	/* This: will not trigger the usecase state machine */
	ret = max77759_reg_read(data->regmap, MAX77759_CHG_CNFG_00, &reg);
	if (ret == 0)
		ret = max77759_chg_mode_write(uc_data->client, MAX77759_CHGR_MODE_ALL_OFF);
	if (ret == 0)
		ret = max77759_chg_mode_write(uc_data->client, reg);

	data->charge_done = false;

	pr_debug("%s charge_done=%d->0, reg=%hhx (%d)\n", __func__,
		 charge_done, reg, ret);

	return ret;
}

/* called from gcpm and for CC_MAX == 0 */
static int max77759_set_charge_enabled(struct max77759_chgr_data *data,
				       int enabled, const char *reason)
{
	/* ->charge_done is reset in max77759_enable_sw_recharge() */
	pr_debug("%s %s enabled=%d\n", __func__, reason, enabled);

	return gvotable_cast_long_vote(data->mode_votable, reason,
				       GBMS_CHGR_MODE_CHGR_BUCK_ON, enabled);
}

/* google_charger on disconnect */
static int max77759_set_charge_disable(struct max77759_chgr_data *data,
				       int enabled, const char *reason)
{
	/* make sure charging is restarted on enable */
	if (enabled) {
		int ret;

		ret = max77759_enable_sw_recharge(data, false);
		if (ret < 0)
			dev_err(data->dev, "%s cannot re-enable charging (%d)\n",
				__func__, ret);
	}

	return gvotable_cast_long_vote(data->mode_votable, reason,
				       GBMS_CHGR_MODE_STBY_ON, enabled);
}

static int max77759_chgin_input_suspend(struct max77759_chgr_data *data,
					bool enabled, const char *reason)
{
	const int old_value = data->chgin_input_suspend;
	int ret;

	pr_debug("%s enabled=%d->%d reason=%s\n", __func__,
		 data->wcin_input_suspend, enabled, reason);

	data->chgin_input_suspend = enabled; /* the callback might use this */
	ret = gvotable_cast_long_vote(data->mode_votable, "CHGIN_SUSP",
				      GBMS_CHGR_MODE_CHGIN_OFF, enabled);
	if (ret < 0)
		data->chgin_input_suspend = old_value; /* restored */

	return ret;
}

static int max77759_wcin_input_suspend(struct max77759_chgr_data *data,
				       bool enabled, const char *reason)
{
	const int old_value = data->wcin_input_suspend;
	int ret;

	pr_debug("%s enabled=%d->%d reason=%s\n", __func__,
		 data->wcin_input_suspend, enabled, reason);

	data->wcin_input_suspend = enabled; /* the callback uses this!  */
	ret = gvotable_cast_long_vote(data->mode_votable, reason,
				      GBMS_CHGR_MODE_WLCIN_OFF, enabled);
	if (ret < 0)
		data->wcin_input_suspend = old_value; /* restore */

	return ret;
}

static int max77759_set_regulation_voltage(struct max77759_chgr_data *data,
					   int voltage_uv)
{
	u8 value;

	if (voltage_uv >= 4500000)
		value = 0x32;
	else if (voltage_uv < 4000000)
		value = 0x38 + (voltage_uv - 3800000) / 100000;
	else
		value = (voltage_uv - 4000000) / 10000;

	value = VALUE2FIELD(MAX77759_CHG_CNFG_04_CHG_CV_PRM, value);
	return max77759_reg_update(data, MAX77759_CHG_CNFG_04,
				   MAX77759_CHG_CNFG_04_CHG_CV_PRM_MASK,
				   value);
}

static int max77759_get_regulation_voltage_uv(struct max77759_chgr_data *data,
					      int *voltage_uv)
{
	u8 value;
	int ret;

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_CNFG_04, &value);
	if (ret < 0)
		return ret;

	if  (value < 0x33)
		*voltage_uv = (4000 + value * 10) * 1000;
	else if (value == 0x38)
		*voltage_uv = 3800 * 1000;
	else if (value == 0x39)
		*voltage_uv = 3900 * 1000;
	else
		return -EINVAL;

	return 0;
}

/* set charging current to 0 to disable charging (MODE=0) */
static int max77759_set_charger_current_max_ua(struct max77759_chgr_data *data,
					       int current_ua)
{
	const int disabled = current_ua == 0;
	u8 value;
	int ret;

	if (current_ua < 0)
		return 0;

	/* ilim=0 -> switch to mode 0 and suspend charging */
	if  (current_ua == 0)
		value = 0x0;
	else if (current_ua <= 200000)
		value = 0x03;
	else if (current_ua >= 4000000)
		value = 0x3c;
	else
		value = 0x3 + (current_ua - 200000) / 66670;

	/*
	 * cc_max > 0 might need to restart charging: the usecase state machine
	 * will be triggered in max77759_set_charge_enabled()
	 */
	if (current_ua) {
		ret = max77759_enable_sw_recharge(data, false);
		if (ret < 0)
			dev_err(data->dev, "cannot re-enable charging (%d)\n", ret);
	}

	value = VALUE2FIELD(MAX77759_CHG_CNFG_02_CHGCC, value);
	ret = max77759_reg_update(data, MAX77759_CHG_CNFG_02,
				   MAX77759_CHG_CNFG_02_CHGCC_MASK,
				   value);
	if (ret == 0)
		ret = max77759_set_charge_enabled(data, !disabled, "CC_MAX");

	return ret;
}

static int max77759_get_charger_current_max_ua(struct max77759_chgr_data *data,
					       int *current_ua)
{
	u8 value;
	int ret;

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_CNFG_02,
				&value);
	if (ret < 0)
		return ret;

	/* TODO: fix the rounding */
	value = VALUE2FIELD(MAX77759_CHG_CNFG_02_CHGCC, value);

	/* ilim=0 -> mode 0 with charging suspended */
	if (value == 0)
		*current_ua = 0;
	else if (value < 3)
		*current_ua = 133 * 1000;
	else if (value >= 0x3C)
		*current_ua = 4000 * 1000;
	else
		*current_ua = 133000 + (value - 2) * 66670;

	return 0;
}

/* enable autoibus and charger mode */
static int max77759_chgin_set_ilim_max_ua(struct max77759_chgr_data *data,
					  int ilim_ua)
{
	const bool suspend = ilim_ua == 0;
	u8 value;
	int ret;

	/* TODO: disable charging */
	if (ilim_ua < 0)
		return 0;

	if (ilim_ua == 0)
		value = 0x00;
	else if (ilim_ua > 3200000)
		value = 0x7f;
	else
		value = 0x04 + (ilim_ua - 125000) / 25000;

	value = VALUE2FIELD(MAX77759_CHG_CNFG_09_NO_AUTOIBUS, 1) |
		VALUE2FIELD(MAX77759_CHG_CNFG_09_CHGIN_ILIM, value);
	ret = max77759_reg_update(data, MAX77759_CHG_CNFG_09,
					MAX77759_CHG_CNFG_09_NO_AUTOIBUS |
					MAX77759_CHG_CNFG_09_CHGIN_ILIM_MASK,
					value);
	if (ret == 0)
		ret = max77759_chgin_input_suspend(data, suspend, "ILIM");

	return ret;
}

static int max77759_chgin_get_ilim_max_ua(struct max77759_chgr_data *data,
					  int *ilim_ua)
{
	int icl, ret;
	u8 value;

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_CNFG_09, &value);
	if (ret < 0)
		return ret;

	value = FIELD2VALUE(MAX77759_CHG_CNFG_09_CHGIN_ILIM, value);
	if (value == 0)
		icl = 0;
	else if (value > 3)
		icl = 100 + (value - 3) * 25;
	else
		icl = 100;

	*ilim_ua = icl * 1000;

	if (data->chgin_input_suspend)
		*ilim_ua = 0;

	return 0;
}

static int max77759_set_topoff_current_max_ma(struct max77759_chgr_data *data,
					       int current_ma)
{
	u8 value;
	int ret;

	if (current_ma < 0)
		return 0;

	if (current_ma <= 150)
		value = 0x0;
	else if (current_ma >= 500)
		value = 0x7;
	else
		value = (current_ma - 150) / 50;

	value = VALUE2FIELD(MAX77759_CHG_CNFG_03_TO_ITH, value);
	ret = max77759_reg_update(data, MAX77759_CHG_CNFG_03,
				   MAX77759_CHG_CNFG_03_TO_ITH_MASK,
				   value);

	return ret;
}

static int max77759_wcin_set_ilim_max_ua(struct max77759_chgr_data *data,
					 int ilim_ua)
{
	u8 value;
	int ret;

	if (ilim_ua < 0)
		return -EINVAL;

	if (ilim_ua == 0)
		value = 0x00;
	else if (ilim_ua <= 125000)
		value = 0x01;
	else
		value = 0x3 + (ilim_ua - 125000) / 31250;

	value = VALUE2FIELD(MAX77759_CHG_CNFG_10_WCIN_ILIM, value);
	ret = max77759_reg_update(data, MAX77759_CHG_CNFG_10,
					MAX77759_CHG_CNFG_10_WCIN_ILIM_MASK,
					value);

	/* Legacy: DC_ICL doesn't suspend on ilim_ua == 0 (it should) */

	return ret;
}

static int max77759_wcin_get_ilim_max_ua(struct max77759_chgr_data *data,
					 int *ilim_ua)
{
	int ret;
	u8 value;

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_CNFG_10, &value);
	if (ret < 0)
		return ret;

	value = FIELD2VALUE(MAX77759_CHG_CNFG_10_WCIN_ILIM, value);
	if (value == 0)
		*ilim_ua = 0;
	else if (value < 4)
		*ilim_ua = 125000;
	else
		*ilim_ua = 125000 + (value - 3) * 31250;

	if (data->wcin_input_suspend)
		*ilim_ua = 0;

	return 0;
}

/* default is no suspend, any valid vote will suspend  */
static int max77759_dc_suspend_vote_callback(struct gvotable_election *el,
					     const char *reason, void *value)
{
	struct max77759_chgr_data *data = gvotable_get_data(el);
	int ret, suspend = (long)value > 0;

	/* will trigger a CHARGER_MODE callback */
	ret = max77759_wcin_input_suspend(data, suspend, "DC_SUSPEND");
	if (ret < 0)
		return 0;

	pr_debug("%s: DC_SUSPEND reason=%s, value=%ld suspend=%d (%d)\n",
		 __func__, reason ? reason : "", (long)value, suspend, ret);

	return 0;
}

static int max77759_dcicl_callback(struct gvotable_election *el,
				   const char *reason,
				   void *value)
{
	struct max77759_chgr_data *data = gvotable_get_data(el);
	int dc_icl = (long)value;
	const bool suspend = dc_icl == 0;
	int ret;

	pr_debug("%s: DC_ICL reason=%s, value=%ld suspend=%d\n",
		 __func__, reason ? reason : "", (long)value, suspend);

	/* doesn't trigger a CHARGER_MODE */
	ret = max77759_wcin_set_ilim_max_ua(data, dc_icl);
	if (ret < 0)
		dev_err(data->dev, "cannot set dc_icl=%d (%d)\n",
			dc_icl, ret);

	/* will trigger a CHARGER_MODE callback */
	ret = max77759_wcin_input_suspend(data, suspend, "DC_ICL");
	if (ret < 0)
		dev_err(data->dev, "cannot set suspend=%d (%d)\n",
			suspend, ret);

	return 0;
}

/*************************
 * WCIN PSY REGISTRATION   *
 *************************/
static enum power_supply_property max77759_wcin_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
};

static int max77759_wcin_is_valid(struct max77759_chgr_data *data)
{
	uint8_t int_ok;
	int ret;

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_INT_OK, &int_ok);
	return (ret == 0) && _chg_int_ok_wcin_ok_get(int_ok);
}

static int max77759_wcin_is_online(struct max77759_chgr_data *data)
{
	uint8_t val;
	int ret;

	ret = max77759_wcin_is_valid(data);
	if (ret <= 0)
		return ret;

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_DETAILS_02, &val);
	return (ret == 0) && _chg_details_02_wcin_sts_get(val);
}

/* TODO: make this configurable */
static struct power_supply* max77759_get_wlc_psy(struct max77759_chgr_data *chg)
{
	if (!chg->wlc_psy)
		chg->wlc_psy = power_supply_get_by_name("wireless");
	return chg->wlc_psy;
}

static int max77759_wcin_voltage_max(struct max77759_chgr_data *chg,
				     union power_supply_propval *val)
{
	struct power_supply *wlc_psy;
	int rc;

	if (!max77759_wcin_is_valid(chg)) {
		val->intval = 0;
		return 0;
	}

	wlc_psy = max77759_get_wlc_psy(chg);
	if (!wlc_psy)
		return max77759_get_regulation_voltage_uv(chg, &val->intval);

	rc = power_supply_get_property(wlc_psy, POWER_SUPPLY_PROP_VOLTAGE_MAX, val);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't get VOLTAGE_MAX, rc=%d\n", rc);
		return rc;
	}

	return rc;
}

static int max77759_wcin_voltage_now(struct max77759_chgr_data *chg,
				     union power_supply_propval *val)
{
	struct power_supply *wlc_psy;
	int rc;

	if (!max77759_wcin_is_valid(chg)) {
		val->intval = 0;
		return 0;
	}

	wlc_psy = max77759_get_wlc_psy(chg);
	if (!wlc_psy)
		return max77759_read_vbyp(chg, &val->intval);

	rc = power_supply_get_property(wlc_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, val);
	if (rc < 0)
		dev_err(chg->dev, "Couldn't get VOLTAGE_NOW, rc=%d\n", rc);

	return rc;
}

#define MAX77759_WCIN_RAW_TO_UA	156

/* current is valid only when charger mode is one of the following */
static bool max77759_current_check_mode(struct max77759_chgr_data *data)
{
	int ret;
	u8 reg;

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_CNFG_00, &reg);
	if (ret < 0)
		return false;

	return reg == 5 || reg == 6 || reg == 7 || reg == 0xe || reg == 0xf;
}

/* only valid in mode 5, 6, 7, e, f */
static int max77759_wcin_current_now(struct max77759_chgr_data *data, int *iic)
{
	int ret, iic_raw;

	ret = max77759_find_fg(data);
	if (ret < 0)
		return ret;

	ret = max_m5_read_actual_input_current_ua(data->fg_i2c_client, &iic_raw);
	if (ret < 0)
		return ret;

	*iic = iic_raw * MAX77759_WCIN_RAW_TO_UA;
	return 0;
}

static int max77759_wcin_get_prop(struct power_supply *psy,
				  enum power_supply_property psp,
				  union power_supply_propval *val)
{
	struct max77759_chgr_data *chgr = power_supply_get_drvdata(psy);
	int rc = 0;

	if (max77759_resume_check(chgr))
		return -EAGAIN;

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = max77759_wcin_is_valid(chgr);
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = max77759_wcin_is_online(chgr);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		rc = max77759_wcin_voltage_now(chgr, val);
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		rc = max77759_wcin_get_ilim_max_ua(chgr, &val->intval);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		rc = max77759_wcin_voltage_max(chgr, val);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = 0;
		if (max77759_wcin_is_online(chgr))
			rc = max77759_wcin_current_now(chgr, &val->intval);
		break;
	default:
		return -EINVAL;
	}
	if (rc < 0) {
		pr_debug("Couldn't get prop %d rc = %d\n", psp, rc);
		return -ENODATA;
	}
	return 0;
}

static int max77759_wcin_set_prop(struct power_supply *psy,
				  enum power_supply_property psp,
				  const union power_supply_propval *val)
{
	struct max77759_chgr_data *chgr = power_supply_get_drvdata(psy);
	int rc = 0;

	if (max77759_resume_check(chgr))
		return -EAGAIN;

	switch (psp) {
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		rc = max77759_wcin_set_ilim_max_ua(chgr, val->intval);
		pr_debug("%s: DC_ICL=%d (%d)\n", __func__, val->intval, rc);
		break;
	/* called from google_cpm when switching chargers */
	case GBMS_PROP_CHARGING_ENABLED:
		rc = max77759_set_charge_enabled(chgr, val->intval > 0,
						 "DC_PSP_ENABLED");
		pr_debug("%s: charging_enabled=%d (%d)\n",
			__func__, val->intval > 0, rc);
		break;
	default:
		return -EINVAL;
	}

	return rc;
}

static int max77759_wcin_prop_is_writeable(struct power_supply *psy,
					   enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_CURRENT_MAX:
	case GBMS_PROP_CHARGING_ENABLED:
		return 1;
	default:
		break;
	}

	return 0;
}

static struct power_supply_desc max77759_wcin_psy_desc = {
	.name = "dc",
	.type = POWER_SUPPLY_TYPE_UNKNOWN,
	.properties = max77759_wcin_props,
	.num_properties = ARRAY_SIZE(max77759_wcin_props),
	.get_property = max77759_wcin_get_prop,
	.set_property = max77759_wcin_set_prop,
	.property_is_writeable = max77759_wcin_prop_is_writeable,
};

static int max77759_init_wcin_psy(struct max77759_chgr_data *data)
{
	struct power_supply_config wcin_cfg = {};
	struct device *dev = data->dev;
	const char *name;
	int ret;

	wcin_cfg.drv_data = data;
	wcin_cfg.of_node = dev->of_node;

	if (of_property_read_bool(dev->of_node, "max77759,dc-psy-type-wireless"))
		max77759_wcin_psy_desc.type = POWER_SUPPLY_TYPE_WIRELESS;

	ret = of_property_read_string(dev->of_node, "max77759,dc-psy-name", &name);
	if (ret == 0) {
		max77759_wcin_psy_desc.name = devm_kstrdup(dev, name, GFP_KERNEL);
		if (!max77759_wcin_psy_desc.name)
			return -ENOMEM;
	}

	data->wcin_psy = devm_power_supply_register(data->dev,
					&max77759_wcin_psy_desc, &wcin_cfg);
	if (IS_ERR(data->wcin_psy))
		return PTR_ERR(data->wcin_psy);

	return 0;
}

static int max77759_higher_headroom_enable(struct max77759_chgr_data *data, bool flag)
{
	int ret = 0;
	u8 reg, reg_rd, val = flag ? CHGR_CHG_CNFG_12_VREG_4P7V : CHGR_CHG_CNFG_12_VREG_4P6V;

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_CNFG_12, &reg);
	if (ret < 0)
		return ret;

	reg_rd = reg;
	ret = max77759_chg_prot(data->regmap, false);
	if (ret < 0)
		return ret;

	reg = _chg_cnfg_12_vchgin_reg_set(reg, val);
	ret = max77759_reg_write(data->regmap, MAX77759_CHG_CNFG_12, reg);
	if (ret)
		goto done;;

	dev_dbg(data->dev, "%s: val: %#02x, reg: %#02x -> %#02x\n", __func__, val, reg_rd, reg);

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_CNFG_12, &reg);
	if (ret)
		goto done;

done:
	ret = max77759_chg_prot(data->regmap, true);
	if (ret < 0)
		dev_err(data->dev, "%s: error enabling prot (%d)\n", __func__, ret);
	return ret < 0 ? ret : 0;
}

static int max77759_chg_is_valid(struct max77759_chgr_data *data)
{
	uint8_t int_ok;
	int ret;

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_INT_OK, &int_ok);
	return (ret == 0) && _chg_int_ok_chgin_ok_get(int_ok);
}

static int max77759_chgin_is_online(struct max77759_chgr_data *data)
{
	uint8_t val;
	int ret;

	ret = max77759_chg_is_valid(data);
	if (ret <= 0)
		return ret;

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_DETAILS_02, &val);
	return (ret == 0) && _chg_details_02_chgin_sts_get(val);
}

/*
 * NOTE: could also check aicl to determine whether the adapter is, in fact,
 * at fault. Possibly qualify this with battery voltage as subpar adapters
 * are likely to flag AICL when the battery is at high voltage.
 */
static int max77759_is_limited(struct max77759_chgr_data *data)
{
	int ret;
	u8 value;

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_INT_OK, &value);
	return (ret == 0) && _chg_int_ok_inlim_ok_get(value) == 0;
}

static int max77759_is_valid(struct max77759_chgr_data *data)
{
	uint8_t int_ok;
	int ret;

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_INT_OK, &int_ok);
	return (ret == 0) && (_chg_int_ok_chgin_ok_get(int_ok) ||
	       _chg_int_ok_wcin_ok_get(int_ok));
}

/* WCIN || CHGIN present, valid  && CHGIN FET is closed */
static int max77759_is_online(struct max77759_chgr_data *data)
{
	uint8_t val;
	int ret;

	ret = max77759_is_valid(data);
	if (ret <= 0)
		return 0;

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_DETAILS_02, &val);
	return (ret == 0) && (_chg_details_02_chgin_sts_get(val) ||
	       _chg_details_02_wcin_sts_get(val));
}

static int max77759_get_charge_type(struct max77759_chgr_data *data)
{
	int ret;
	uint8_t reg;

	if (!max77759_is_online(data))
		return POWER_SUPPLY_CHARGE_TYPE_NONE;

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_DETAILS_01, &reg);
	if (ret < 0)
		return POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;

	switch(_chg_details_01_chg_dtls_get(reg)) {
	case CHGR_DTLS_DEAD_BATTERY_MODE:
		return POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
	case CHGR_DTLS_FAST_CHARGE_CONST_CURRENT_MODE:
		return POWER_SUPPLY_CHARGE_TYPE_FAST;
	case CHGR_DTLS_FAST_CHARGE_CONST_VOLTAGE_MODE:
	case CHGR_DTLS_TOP_OFF_MODE:
		return POWER_SUPPLY_CHARGE_TYPE_TAPER;

	case CHGR_DTLS_DONE_MODE:
	case CHGR_DTLS_TIMER_FAULT_MODE:
	case CHGR_DTLS_DETBAT_HIGH_SUSPEND_MODE:
	case CHGR_DTLS_OFF_MODE:
	case CHGR_DTLS_OFF_HIGH_TEMP_MODE:
	case CHGR_DTLS_OFF_WATCHDOG_MODE:
		return POWER_SUPPLY_CHARGE_TYPE_NONE;
	default:
		break;
	}

	return POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
}

static bool max77759_is_full(struct max77759_chgr_data *data)
{
	int vlimit = data->chg_term_voltage;
	int ret, vbatt = 0;

	/*
	 * Set voltage level to leave CHARGER_DONE (BATT_RL_STATUS_DISCHARGE)
	 * and enter BATT_RL_STATUS_RECHARGE. It sets STATUS_DISCHARGE again
	 * once CHARGER_DONE flag set (return true here)
	 */
	ret = max77759_read_vbatt(data, &vbatt);
	if (ret == 0)
		vbatt = vbatt / 1000;

	if (data->charge_done)
		vlimit -= data->chg_term_volt_debounce;

	/* true when chg_term_voltage==0, false if read error (vbatt==0) */
	return vbatt >= vlimit;
}

static int max77759_get_status(struct max77759_chgr_data *data)
{
	uint8_t val;
	int ret;

	if (!max77759_is_online(data))
		return POWER_SUPPLY_STATUS_DISCHARGING;

	/*
	 * EOC can be made sticky returning POWER_SUPPLY_STATUS_FULL on
	 * ->charge_done. Also need a check on max77759_is_full() or
	 * google_charger will fail to restart charging.
	 */
	ret = max77759_reg_read(data->regmap, MAX77759_CHG_DETAILS_01, &val);
	if (ret < 0)
		return POWER_SUPPLY_STATUS_UNKNOWN;

	switch (_chg_details_01_chg_dtls_get(val)) {
		case CHGR_DTLS_DEAD_BATTERY_MODE:
		case CHGR_DTLS_FAST_CHARGE_CONST_CURRENT_MODE:
		case CHGR_DTLS_FAST_CHARGE_CONST_VOLTAGE_MODE:
		case CHGR_DTLS_TOP_OFF_MODE:
			return POWER_SUPPLY_STATUS_CHARGING;
		case CHGR_DTLS_DONE_MODE:
			/* same as POWER_SUPPLY_PROP_CHARGE_DONE */
			if (!max77759_is_full(data))
				data->charge_done = false;
			if (data->charge_done)
				return POWER_SUPPLY_STATUS_FULL;
			return POWER_SUPPLY_STATUS_NOT_CHARGING;
		case CHGR_DTLS_TIMER_FAULT_MODE:
		case CHGR_DTLS_DETBAT_HIGH_SUSPEND_MODE:
		case CHGR_DTLS_OFF_MODE:
		case CHGR_DTLS_OFF_HIGH_TEMP_MODE:
		case CHGR_DTLS_OFF_WATCHDOG_MODE:
			return POWER_SUPPLY_STATUS_NOT_CHARGING;
		default:
			break;
	}

	return POWER_SUPPLY_STATUS_UNKNOWN;
}

static int max77759_get_chg_chgr_state(struct max77759_chgr_data *data,
				       union gbms_charger_state *chg_state)
{
	int usb_present, usb_valid, dc_present, dc_valid;
	const char *source = "";
	uint8_t int_ok, dtls;
	int vbatt, icl = 0;
	int rc;

	chg_state->v = 0;
	chg_state->f.chg_status = max77759_get_status(data);
	chg_state->f.chg_type = max77759_get_charge_type(data);
	chg_state->f.flags = gbms_gen_chg_flags(chg_state->f.chg_status,
						chg_state->f.chg_type);

	rc = max77759_reg_read(data->regmap, MAX77759_CHG_INT_OK, &int_ok);
	if (rc == 0)
		rc = max77759_reg_read(data->regmap, MAX77759_CHG_DETAILS_02,
					&dtls);

	/* present when connected, valid when FET is closed */
	usb_present = (rc == 0) && _chg_int_ok_chgin_ok_get(int_ok);
	usb_valid = usb_present && _chg_details_02_chgin_sts_get(dtls);

	/* present if in field, valid when FET is closed */
	dc_present = (rc == 0) && _chg_int_ok_wcin_ok_get(int_ok);
	dc_valid = dc_present && _chg_details_02_wcin_sts_get(dtls);

	rc = max77759_read_vbatt(data, &vbatt);
	if (rc == 0)
		chg_state->f.vchrg = vbatt / 1000;

	if (chg_state->f.chg_status == POWER_SUPPLY_STATUS_DISCHARGING)
		goto exit_done;

	rc = max77759_is_limited(data);
	if (rc > 0)
		chg_state->f.flags |= GBMS_CS_FLAG_ILIM;

	/* TODO: b/ handle input MUX corner cases */
	if (usb_valid) {
		max77759_chgin_get_ilim_max_ua(data, &icl);
		/* TODO: 'u' only when in sink */
		if (!dc_present)
			source = "U";
		 else if (dc_valid)
			source = "UW";
		 else
			source = "Uw";

	} else if (dc_valid) {
		max77759_wcin_get_ilim_max_ua(data, &icl);

		/* TODO: 'u' only when in sink */
		source = usb_present ? "uW" : "W";
	} else if (usb_present && dc_present) {
		source = "uw";
	} else if (usb_present) {
		source = "u";
	} else if (dc_present) {
		source = "w";
	}

	chg_state->f.icl = icl / 1000;

exit_done:
	pr_debug("MSC_PCS chg_state=%lx [0x%x:%d:%d:%d:%d] chg=%s\n",
		 (unsigned long)chg_state->v,
		 chg_state->f.flags,
		 chg_state->f.chg_type,
		 chg_state->f.chg_status,
		 chg_state->f.vchrg,
		 chg_state->f.icl,
		 source);

	return 0;
}

#define MAX77759_CHGIN_RAW_TO_UA	125

/* only valid in mode 5, 6, 7, e, f */
static int max77759_chgin_current_now(struct max77759_chgr_data *data, int *iic)
{
	int ret, iic_raw;

	ret = max77759_find_fg(data);
	if (ret < 0)
		return ret;

	ret = max_m5_read_actual_input_current_ua(data->fg_i2c_client, &iic_raw);
	if (ret < 0)
		return ret;

	*iic = iic_raw * MAX77759_CHGIN_RAW_TO_UA;
	return 0;
}

static int max77759_wd_tickle(struct max77759_chgr_data *data)
{
	int ret;
	u8 reg, reg_new;

	mutex_lock(&data->io_lock);
	ret = max77759_reg_read(data->regmap, MAX77759_CHG_CNFG_00, &reg);
	if (ret == 0) {
		reg_new  = _chg_cnfg_00_wdtclr_set(reg, 0x1);
		ret = max77759_reg_write(data->regmap, MAX77759_CHG_CNFG_00,
					 reg_new);
	}

	if (ret < 0)
		pr_err("WD Tickle failed %d\n", ret);

	mutex_unlock(&data->io_lock);
	return ret;
}

/* online is used from DC charging to tickle the watchdog (if enabled) */
static int max77759_set_online(struct max77759_chgr_data *data, bool online)
{
	int ret = 0;

	if (data->wden) {
		ret = max77759_wd_tickle(data);
		if (ret < 0)
			pr_err("cannot tickle the watchdog\n");
	}

	if (data->online != online) {
		ret = gvotable_cast_long_vote(data->mode_votable, "OFFLINE",
					      GBMS_CHGR_MODE_STBY_ON, !online);
		data->online = online;
	}

	return ret;
}

static int max77759_psy_set_property(struct power_supply *psy,
				     enum power_supply_property psp,
				     const union power_supply_propval *pval)
{
	struct max77759_chgr_data *data = power_supply_get_drvdata(psy);
	int ret = -EINVAL;

	if (max77759_resume_check(data))
		return -EAGAIN;

	switch (psp) {
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		ret = max77759_chgin_set_ilim_max_ua(data, pval->intval);
		pr_debug("%s: icl=%d (%d)\n", __func__, pval->intval, ret);
		break;
	/* Charge current is set to 0 to EOC */
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		ret = max77759_set_charger_current_max_ua(data, pval->intval);
		pr_debug("%s: charge_current=%d (%d)\n",
			__func__, pval->intval, ret);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		ret = max77759_set_regulation_voltage(data, pval->intval);
		pr_debug("%s: charge_voltage=%d (%d)\n",
			__func__, pval->intval, ret);
		if (ret)
			break;
		if (max77759_is_online(data) && pval->intval >= data->chg_term_voltage * 1000)
			ret = max77759_higher_headroom_enable(data, true);
		break;
	/* called from google_cpm when switching chargers */
	case GBMS_PROP_CHARGING_ENABLED:
		ret = max77759_set_charge_enabled(data, pval->intval,
						  "PSP_ENABLED");
		pr_debug("%s: charging_enabled=%d (%d)\n",
			__func__, pval->intval, ret);
		break;
	/* called from google_charger on disconnect */
	case GBMS_PROP_CHARGE_DISABLE:
		ret = max77759_set_charge_disable(data, pval->intval,
						  "PSP_DISABLE");
		pr_debug("%s: charge_disable=%d (%d)\n",
			__func__, pval->intval, ret);
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		ret = max77759_set_online(data, pval->intval != 0);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		ret = max77759_set_topoff_current_max_ma(data, pval->intval);
		pr_debug("%s: topoff_current=%d (%d)\n",
			__func__, pval->intval, ret);
		break;
	case GBMS_PROP_TAPER_CONTROL:
		ret = 0;
		break;
	default:
		break;
	}

	if (ret == 0 && data->wden)
		max77759_wd_tickle(data);


	return ret;
}

static int max77759_psy_get_property(struct power_supply *psy,
				     enum power_supply_property psp,
				     union power_supply_propval *pval)
{
	struct max77759_chgr_data *data = power_supply_get_drvdata(psy);
	union gbms_charger_state chg_state;
	int rc, ret = 0;

	if (max77759_resume_check(data))
		return -EAGAIN;

	switch (psp) {
	case GBMS_PROP_CHARGE_DISABLE:
		rc = max77759_get_charge_enabled(data, &pval->intval);
		if (rc == 0)
			pval->intval = !pval->intval;
		else
			pval->intval = rc;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		pval->intval = max77759_get_charge_type(data);
		break;
	case GBMS_PROP_CHARGING_ENABLED:
		ret = max77759_get_charge_enabled(data, &pval->intval);
		break;
	case GBMS_PROP_CHARGE_CHARGER_STATE:
		rc = max77759_get_chg_chgr_state(data, &chg_state);
		if (rc == 0)
			gbms_propval_int64val(pval) = chg_state.v;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		ret = max77759_get_charger_current_max_ua(data, &pval->intval);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		ret = max77759_get_regulation_voltage_uv(data, &pval->intval);
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		pval->intval = max77759_is_online(data);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		pval->intval = max77759_is_valid(data);
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		ret = max77759_chgin_get_ilim_max_ua(data, &pval->intval);
		break;
	case GBMS_PROP_INPUT_CURRENT_LIMITED:
		pval->intval = max77759_is_limited(data);
		break;
	case POWER_SUPPLY_PROP_STATUS:
		pval->intval = max77759_get_status(data);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		rc = max77759_read_vbatt(data, &pval->intval);
		if (rc < 0)
			pval->intval = -1;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		if (!max77759_current_check_mode(data)) {
			pval->intval = 0;
			break;
		}

		if (max77759_wcin_is_online(data))
			rc = max77759_wcin_current_now(data, &pval->intval);
		else if (max77759_chgin_is_online(data))
			rc = max77759_chgin_current_now(data, &pval->intval);
		else
			rc = -EINVAL;

		if (rc < 0)
			pval->intval = rc;
		break;
	case GBMS_PROP_TAPER_CONTROL:
		ret = 0;
		break;
	default:
		pr_debug("property (%d) unsupported.\n", psp);
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int max77759_psy_is_writeable(struct power_supply *psy,
				 enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
	case POWER_SUPPLY_PROP_VOLTAGE_MAX: /* compat, same the next */
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
	case POWER_SUPPLY_PROP_CURRENT_MAX:
	case GBMS_PROP_CHARGE_DISABLE:
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
	case GBMS_PROP_TAPER_CONTROL:
		return 1;
	default:
		break;
	}

	return 0;
}

/*
 * TODO: POWER_SUPPLY_PROP_RERUN_AICL, POWER_SUPPLY_PROP_TEMP
 * POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX
 * POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX
 */
static enum power_supply_property max77759_psy_props[] = {
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,		/* compat */
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_STATUS,
};

static struct power_supply_desc max77759_psy_desc = {
	.name = "max77759-charger",
	.type = POWER_SUPPLY_TYPE_UNKNOWN,
	.properties = max77759_psy_props,
	.num_properties = ARRAY_SIZE(max77759_psy_props),
	.get_property = max77759_psy_get_property,
	.set_property = max77759_psy_set_property,
	.property_is_writeable = max77759_psy_is_writeable,
};

static ssize_t show_fship_dtls(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct max77759_chgr_data *data = dev_get_drvdata(dev);
	static char *fship_reason[] = {"None", "PWRONB1", "PWRONB1", "PWR"};
	u8 pmic_rd;
	int ret;

	if (data->fship_dtls != -1)
		goto exit_done;

	if(max77759_resume_check(data))
		return -EAGAIN;

	ret = max77759_find_pmic(data);
	if (ret < 0)
		return ret;

	ret = max777x9_pmic_reg_read(data->pmic_i2c_client,
				     MAX77759_FSHIP_EXIT_DTLS,
				     &pmic_rd, 1);
	if (ret < 0)
		return -EIO;

	if (pmic_rd & MAX77759_FSHIP_EXIT_DTLS_RD) {
		u8 fship_dtls;

		ret = max77759_reg_read(data->regmap, MAX77759_CHG_DETAILS_03,
					&fship_dtls);
		if (ret < 0)
			return -EIO;

		data->fship_dtls =
			_chg_details_03_fship_exit_dtls_get(fship_dtls);

		pmic_rd &= ~MAX77759_FSHIP_EXIT_DTLS_RD;
		ret = max777x9_pmic_reg_write(data->pmic_i2c_client,
					      MAX77759_FSHIP_EXIT_DTLS,
					      &pmic_rd, 1);
		if (ret < 0)
			pr_err("FSHIP: cannot update RD (%d)\n", ret);

	} else {
		data->fship_dtls = 0;
	}

exit_done:
	return scnprintf(buf, PAGE_SIZE, "%d %s\n", data->fship_dtls,
			 fship_reason[data->fship_dtls]);
}

static DEVICE_ATTR(fship_dtls, 0444, show_fship_dtls, NULL);

/* -- BCL ------------------------------------------------------------------ */

static int vdroop2_ok_get(void *d, u64 *val)
{
	struct max77759_chgr_data *data = d;
	int ret = 0;
	u8 chg_dtls1;

	if(max77759_resume_check(data))
		return -EAGAIN;

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_DETAILS_01, &chg_dtls1);
	if (ret < 0)
		return -ENODEV;

	*val = _chg_details_01_vdroop2_ok_get(chg_dtls1);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(vdroop2_ok_fops, vdroop2_ok_get, NULL, "%llu\n");

static int vdp1_stp_bst_get(void *d, u64 *val)
{
	struct max77759_chgr_data *data = d;
	int ret = 0;
	u8 chg_cnfg18;

	if(max77759_resume_check(data))
		return -EAGAIN;

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_CNFG_18, &chg_cnfg18);
	if (ret < 0)
		return -ENODEV;

	*val = _chg_cnfg_18_vdp1_stp_bst_get(chg_cnfg18);
	return 0;
}

static int vdp1_stp_bst_set(void *d, u64 val)
{
	struct max77759_chgr_data *data = d;
	int ret = 0;
	u8 chg_cnfg18;
	const u8 vdp1_stp_bst = (val > 0)? 0x1 : 0x0;

	if(max77759_resume_check(data))
		return -EAGAIN;

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_CNFG_18, &chg_cnfg18);
	if (ret < 0)
		return -ENODEV;

	chg_cnfg18 = _chg_cnfg_18_vdp1_stp_bst_set(chg_cnfg18, vdp1_stp_bst);
	ret = max77759_reg_write(data->regmap, MAX77759_CHG_CNFG_18, chg_cnfg18);

	return ret;
}

DEFINE_SIMPLE_ATTRIBUTE(vdp1_stp_bst_fops, vdp1_stp_bst_get, vdp1_stp_bst_set, "%llu\n");

static int vdp2_stp_bst_get(void *d, u64 *val)
{
	struct max77759_chgr_data *data = d;
	int ret = 0;
	u8 chg_cnfg18;

	if(max77759_resume_check(data))
		return -EAGAIN;

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_CNFG_18, &chg_cnfg18);
	if (ret < 0)
		return -ENODEV;

	*val = _chg_cnfg_18_vdp2_stp_bst_get(chg_cnfg18);
	return 0;
}

static int vdp2_stp_bst_set(void *d, u64 val)
{
	struct max77759_chgr_data *data = d;
	int ret = 0;
	u8 chg_cnfg18;
	const u8 vdp2_stp_bst = (val > 0)? 0x1 : 0x0;

	if(max77759_resume_check(data))
		return -EAGAIN;

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_CNFG_18, &chg_cnfg18);
	if (ret < 0)
		return -ENODEV;

	chg_cnfg18 = _chg_cnfg_18_vdp2_stp_bst_set(chg_cnfg18, vdp2_stp_bst);
	ret = max77759_reg_write(data->regmap, MAX77759_CHG_CNFG_18, chg_cnfg18);

	return ret;
}

DEFINE_SIMPLE_ATTRIBUTE(vdp2_stp_bst_fops, vdp2_stp_bst_get, vdp2_stp_bst_set, "%llu\n");

/* -- input protection ----------------------------------------------------- */

/* write to INPUT_MASK_CLR in to re-enable detection */
static int max77759_chgr_input_mask_clear(struct max77759_chgr_data *data)
{
	u8 value;
	int ret;

	if(max77759_resume_check(data))
		return -EAGAIN;

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_CNFG_10, &value);
	if (ret < 0)
		return -ENODEV;

	ret = max77759_reg_write(data->regmap, MAX77759_CHG_CNFG_10,
				 _chg_cnfg_10_input_mask_clr_set(value, 1));
	if (ret < 0)
		pr_err("%s: cannot clear input_mask ret=%d\n", __func__, ret);

	ret = max77759_reg_write(data->regmap, MAX77759_CHG_CNFG_10,
				 _chg_cnfg_10_input_mask_clr_set(value, 0));
	if (ret < 0)
		pr_err("%s: cannot reset input_mask ret=%d\n", __func__, ret);

	return ret;
}


static int input_mask_clear_set(void *d, u64 val)
{
	struct max77759_chgr_data *data = d;

	return max77759_chgr_input_mask_clear(data);
}

DEFINE_SIMPLE_ATTRIBUTE(input_mask_clear_fops, NULL, input_mask_clear_set, "%llu\n");

/* -- charge control ------------------------------------------------------ */

static int charger_restart_set(void *d, u64 val)
{
	struct max77759_chgr_data *data = d;
	int ret;

	ret = max77759_enable_sw_recharge(data, !!val);
	dev_info(data->dev, "triggered recharge(force=%d) %d\n", !!val, ret);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(charger_restart_fops, NULL, charger_restart_set, "%llu\n");

/* -- debug --------------------------------------------------------------- */

static int max77759_chg_debug_reg_read(void *d, u64 *val)
{
	struct max77759_chgr_data *data = d;
	u8 reg = 0;
	int ret;

	if(max77759_resume_check(data))
		return -EAGAIN;

	ret = max77759_reg_read(data->regmap, data->debug_reg_address, &reg);
	if (ret)
		return ret;

	*val = reg;
	return 0;
}

static int max77759_chg_debug_reg_write(void *d, u64 val)
{
	struct max77759_chgr_data *data = d;
	u8 reg = (u8) val;

	if(max77759_resume_check(data))
		return -EAGAIN;

	pr_warn("debug write reg 0x%x, 0x%x", data->debug_reg_address, reg);
	return max77759_reg_write(data->regmap, data->debug_reg_address, reg);
}
DEFINE_SIMPLE_ATTRIBUTE(debug_reg_rw_fops, max77759_chg_debug_reg_read,
			max77759_chg_debug_reg_write, "%02llx\n");


static ssize_t max77759_chg_show_reg_all(struct file *filp, char __user *buf,
					size_t count, loff_t *ppos)
{
	struct max77759_chgr_data *data = (struct max77759_chgr_data *)filp->private_data;
	u32 reg_address;
	u8 reg = 0;
	char *tmp;
	int ret = 0, len = 0;

	if (!data->regmap) {
		pr_err("Failed to read, no regmap\n");
		return -EIO;
	}

	tmp = kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (!tmp)
		return -ENOMEM;

	for (reg_address = 0xB0; reg_address <= 0xCC; reg_address++) {
		ret = max77759_reg_read(data->regmap, reg_address, &reg);
		if (ret < 0)
			continue;

		len += scnprintf(tmp + len, PAGE_SIZE - len, "%02x: %02x\n", reg_address, reg);
	}

	if (len > 0)
		len = simple_read_from_buffer(buf, count,  ppos, tmp, strlen(tmp));

	kfree(tmp);

	return len;
}

BATTERY_DEBUG_ATTRIBUTE(debug_all_reg_fops, max77759_chg_show_reg_all, NULL);

static int dbg_init_fs(struct max77759_chgr_data *data)
{
	int ret;

	ret = device_create_file(data->dev, &dev_attr_fship_dtls);
	if (ret != 0)
		pr_err("Failed to create fship_dtls, ret=%d\n", ret);

	data->de = debugfs_create_dir("max77759_chg", 0);
	if (IS_ERR_OR_NULL(data->de))
		return -EINVAL;

	debugfs_create_atomic_t("insel_cnt", 0644, data->de, &data->insel_cnt);
	debugfs_create_bool("insel_clear", 0644, data->de, &data->insel_clear);

	debugfs_create_atomic_t("early_topoff_cnt", 0644, data->de,
				&data->early_topoff_cnt);

	/* BCL */
	debugfs_create_file("vdroop2_ok", 0400, data->de, data,
			    &vdroop2_ok_fops);
	debugfs_create_file("vdp1_stp_bst", 0600, data->de, data,
			    &vdp1_stp_bst_fops);
	debugfs_create_file("vdp2_stp_bst", 0600, data->de, data,
			    &vdp2_stp_bst_fops);

	debugfs_create_file("input_mask_clear", 0600, data->de, data,
			    &input_mask_clear_fops);
	debugfs_create_file("chg_restart", 0600, data->de, data,
			    &charger_restart_fops);

	debugfs_create_u32("address", 0600, data->de, &data->debug_reg_address);
	debugfs_create_file("data", 0600, data->de, data, &debug_reg_rw_fops);
	/* dump all registers */
	debugfs_create_file("registers", 0444, data->de, data, &debug_all_reg_fops);
	return 0;
}

static bool max77759_chg_is_reg(struct device *dev, unsigned int reg)
{
	return (reg >= MAX77759_CHG_INT) && (reg <= MAX77759_CHG_CNFG_19);
}

static const struct regmap_config max77759_chg_regmap_cfg = {
	.name = "max77759_charger",
	.reg_bits = 8,
	.val_bits = 8,
	.val_format_endian = REGMAP_ENDIAN_NATIVE,
	.max_register = MAX77759_CHG_CNFG_19,
	.readable_reg = max77759_chg_is_reg,
	.volatile_reg = max77759_chg_is_reg,

};

static void max77759_aicl_changed(struct max77759_chgr_data *data)
{
	uint8_t int_ok;
	int ret;
	bool aicl_active;

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_INT_OK, &int_ok);
	if (ret) {
		dev_err(data->dev,  "%s:Failed to read MAX77759_CHG_INT_OK.AICL_OK ret:%d\n", __func__, ret);
		return;
	}

	aicl_active = !(int_ok & MAX77759_CHG_INT_OK_AICL_OK);

	if (IS_ERR_OR_NULL(data->aicl_active_el))
		data->aicl_active_el =
				gvotable_election_get_handle("AICL_ACTIVE_EL");

	if (IS_ERR_OR_NULL(data->aicl_active_el)) {
		dev_err(data->dev,  "AICL_ACTIVE_EL get failed %ld\n",
			PTR_ERR(data->aicl_active_el));
		return;
	}

	gvotable_cast_long_vote(data->aicl_active_el, "BMS_VOTER", aicl_active, aicl_active);
}

/*
 * int[0]
 *  CHG_INT_AICL_I	(0x1 << 7)
 *  CHG_INT_CHGIN_I	(0x1 << 6)
 *  CHG_INT_WCIN_I	(0x1 << 5)
 *  CHG_INT_CHG_I	(0x1 << 4)
 *  CHG_INT_BAT_I	(0x1 << 3)
 *  CHG_INT_INLIM_I	(0x1 << 2)
 *  CHG_INT_THM2_I	(0x1 << 1)
 *  CHG_INT_BYP_I	(0x1 << 0)
 *
 * int[1]
 *  CHG_INT2_INSEL_I		(0x1 << 7)
 *  CHG_INT2_SYS_UVLO1_I	(0x1 << 6)
 *  CHG_INT2_SYS_UVLO2_I	(0x1 << 5)
 *  CHG_INT2_BAT_OILO_I		(0x1 << 4)
 *  CHG_INT2_CHG_STA_CC_I	(0x1 << 3)
 *  CHG_INT2_CHG_STA_CV_I	(0x1 << 2)
 *  CHG_INT2_CHG_STA_TO_I	(0x1 << 1)
 *  CHG_INT2_CHG_STA_DONE_I	(0x1 << 0)
 *
 * these 3 cause un-necessary chatter at EOC due to the interaction between
 * the CV and the IIN loop:
 *   MAX77759_CHG_INT2_MASK_CHG_STA_CC_M |
 *   MAX77759_CHG_INT2_MASK_CHG_STA_CV_M |
 *   MAX77759_CHG_INT_MASK_CHG_M
 *
 * TODO: MAX77759_CHG_INT_MASK_AICL_M needs to be throttled
 * if system keeps going in and out of AICL regulation.
 * No evidenvce of this happening so far.
 */
static u8 max77759_int_mask[MAX77759_CHG_INT_COUNT] = {
	(u8)~(MAX77759_CHG_INT_MASK_AICL_M |
	  MAX77759_CHG_INT_MASK_CHGIN_M |
	  MAX77759_CHG_INT_MASK_WCIN_M |
	  MAX77759_CHG_INT_MASK_BAT_M |
	  MAX77759_CHG_INT_MASK_THM2_M_MASK),
	(u8)~(MAX77759_CHG_INT2_MASK_INSEL_M |
	  MAX77759_CHG_INT2_MASK_SYS_UVLO1_M |
	  MAX77759_CHG_INT2_MASK_SYS_UVLO2_M |
	  MAX77759_CHG_INT2_MASK_CHG_STA_TO_M |
	  MAX77759_CHG_INT2_MASK_CHG_STA_DONE_M),
};

static irqreturn_t max77759_chgr_irq(int irq, void *client)
{
	struct max77759_chgr_data *data = client;
	u8 chg_int[MAX77759_CHG_INT_COUNT];
	bool broadcast;
	int ret;

	if (max77759_resume_check(data)) {
		if (data->init_complete && !data->irq_disabled) {
			data->irq_disabled = true;
			disable_irq_nosync(data->irq_int);
		}
		return IRQ_HANDLED;
	}

	ret = max77759_readn(data->regmap, MAX77759_CHG_INT, chg_int,
			     sizeof(chg_int));
	if (ret < 0)
		return IRQ_NONE;

	if ((chg_int[0] & ~max77759_int_mask[0]) == 0 &&
	    (chg_int[1] & ~max77759_int_mask[1]) == 0)
		return IRQ_NONE;

	ret = max77759_writen(data->regmap, MAX77759_CHG_INT, chg_int,
			      sizeof(chg_int));
	if (ret < 0)
		return IRQ_NONE;

	pr_debug("INT : %02x %02x\n", chg_int[0], chg_int[1]);

	/* always broadcast battery events */
	broadcast = chg_int[0] & MAX77759_CHG_INT_MASK_BAT_M;

	if (chg_int[1] & MAX77759_CHG_INT2_MASK_INSEL_M) {

		if (data->insel_clear)
			ret = max77759_chgr_input_mask_clear(data);

		pr_debug("%s: INSEL insel_auto_clear=%d (%d)\n", __func__,
			 data->insel_clear, data->insel_clear ? ret : 0);
		atomic_inc(&data->insel_cnt);

		ret = max77759_higher_headroom_enable(data, false); /* reset on plug/unplug */
		if (ret)
			return IRQ_NONE;
	}

#if IS_ENABLED(CONFIG_GOOGLE_BCL)
	/* TODO: make this an interrupt controller */
	if (chg_int[1] & MAX77759_CHG_INT2_SYS_UVLO1_I) {
		pr_debug("%s: SYS_UVLO1\n", __func__);

		if (data->bcl_dev)
			google_bcl_irq_changed(data->bcl_dev, UVLO1);
	}

	if (chg_int[1] & MAX77759_CHG_INT2_SYS_UVLO2_I) {
		pr_debug("%s: SYS_UVLO2\n", __func__);

		if (data->bcl_dev)
			google_bcl_irq_changed(data->bcl_dev, UVLO2);
	}

	if (chg_int[1] & MAX77759_CHG_INT2_BAT_OILO_I) {
		pr_debug("%s: BAT_OILO\n", __func__);

		if (data->bcl_dev)
			google_bcl_irq_changed(data->bcl_dev, BATOILO);
	}
#endif

	if (chg_int[1] & MAX77759_CHG_INT2_MASK_CHG_STA_TO_M) {
		pr_debug("%s: TOP_OFF\n", __func__);

		if (!max77759_is_full(data)) {
			/*
			 * on small adapter  might enter top-off far from the
			 * last charge tier due to system load.
			 * TODO: check inlim (maybe) and rewrite fv_uv
			 */
			atomic_inc(&data->early_topoff_cnt);
		}

	}

	if (chg_int[1] & MAX77759_CHG_INT2_MASK_CHG_STA_CC_M)
		pr_debug("%s: CC_MODE\n", __func__);

	if (chg_int[1] & MAX77759_CHG_INT2_MASK_CHG_STA_CV_M)
		pr_debug("%s: CV_MODE\n", __func__);

	if (chg_int[1] & MAX77759_CHG_INT2_MASK_CHG_STA_DONE_M) {
		const bool charge_done = data->charge_done;

		/* reset on disconnect or toggles of enable/disable */
		if (max77759_is_full(data))
			data->charge_done = true;
		broadcast = true;

		pr_debug("%s: CHARGE DONE charge_done=%d->%d\n", __func__,
			 charge_done, data->charge_done);
	}

	if (chg_int[0] & MAX77759_CHG_INT_AICL_I) {
		pr_debug("%s: AICL state change\n", __func__);
		max77759_aicl_changed(data);
	}

	/* wired input is changed */
	if (chg_int[0] & MAX77759_CHG_INT_MASK_CHGIN_M) {
		pr_debug("%s: CHGIN charge_done=%d\n", __func__, data->charge_done);

		data->charge_done = false;
		broadcast = true;

		if (data->chgin_psy)
			power_supply_changed(data->chgin_psy);
	}

	/* wireless input is changed */
	if (chg_int[0] & MAX77759_CHG_INT_MASK_WCIN_M) {
		pr_debug("%s: WCIN charge_done=%d\n", __func__, data->charge_done);

		data->charge_done = false;
		broadcast = true;

		if (data->wcin_psy)
			power_supply_changed(data->wcin_psy);
	}

	/* THM2 is changed */
	if (chg_int[0] & MAX77759_CHG_INT_MASK_THM2_M_MASK) {
		uint8_t int_ok;
		bool thm2_sts;

		ret = max77759_reg_read(data->regmap, MAX77759_CHG_INT_OK, &int_ok);
		if (ret == 0) {
			thm2_sts = (_chg_int_ok_thm2_ok_get(int_ok))? false : true;

			if (thm2_sts != data->thm2_sts) {
				pr_info("%s: THM2 %d->%d\n", __func__, data->thm2_sts, thm2_sts);
				if (!thm2_sts) {
					pr_info("%s: THM2 run recover...\n", __func__);
					ret = regmap_write_bits(data->regmap, MAX77759_CHG_CNFG_13,
							MAX77759_CHG_CNFG_13_THM2_HW_CTRL, 0);
					if (ret == 0)
						ret = regmap_write_bits(data->regmap,
								MAX77759_CHG_CNFG_13,
								MAX77759_CHG_CNFG_13_THM2_HW_CTRL,
								MAX77759_CHG_CNFG_13_THM2_HW_CTRL);
				}
				data->thm2_sts = thm2_sts;
			}
		}
	}

	/* someting is changed */
	if (data->psy && broadcast)
		power_supply_changed(data->psy);

	return IRQ_HANDLED;
}

static int max77759_setup_votables(struct max77759_chgr_data *data)
{
	int ret;

	/* votes might change mode */
	data->mode_votable = gvotable_create_int_election(NULL, NULL,
					max77759_mode_callback,
					data);
	if (IS_ERR_OR_NULL(data->mode_votable)) {
		ret = PTR_ERR(data->mode_votable);
		dev_err(data->dev, "no mode votable (%d)\n", ret);
		return ret;
	}

	gvotable_set_vote2str(data->mode_votable, gvotable_v2s_uint);
	/* will use gvotable_get_default() when available */
	gvotable_set_default(data->mode_votable, (void *)GBMS_CHGR_MODE_STBY_ON);
	gvotable_election_set_name(data->mode_votable, GBMS_MODE_VOTABLE);

	/* Wireless charging, DC name is for compat */
	data->dc_suspend_votable =
		gvotable_create_bool_election(NULL,
					     max77759_dc_suspend_vote_callback,
					     data);
	if (IS_ERR_OR_NULL(data->dc_suspend_votable)) {
		ret = PTR_ERR(data->dc_suspend_votable);
		dev_err(data->dev, "no dc_suspend votable (%d)\n", ret);
		return ret;
	}

	gvotable_set_vote2str(data->dc_suspend_votable, gvotable_v2s_int);
	gvotable_election_set_name(data->dc_suspend_votable, "DC_SUSPEND");

	data->dc_icl_votable =
		gvotable_create_int_election(NULL, gvotable_comparator_int_min,
					     max77759_dcicl_callback,
					     data);
	if (IS_ERR_OR_NULL(data->dc_icl_votable)) {
		ret = PTR_ERR(data->dc_icl_votable);
		dev_err(data->dev, "no dc_icl votable (%d)\n", ret);
		return ret;
	}

	gvotable_set_vote2str(data->dc_icl_votable, gvotable_v2s_uint);
	gvotable_set_default(data->dc_icl_votable, (void *)700000);
	gvotable_election_set_name(data->dc_icl_votable, "DC_ICL");
	gvotable_use_default(data->dc_icl_votable, true);

	return 0;
}

#if IS_ENABLED(CONFIG_GOOGLE_BCL)
static void max77759_register_bcl(struct work_struct *work)
{
	struct max77759_chgr_data *data = container_of(work, struct max77759_chgr_data,
						       init_bcl.work);
	struct bcl_device *bcl_dev;

	bcl_dev = google_retrieve_bcl_handle();
	if (!bcl_dev)
		goto retry_init_work;

	if (google_bcl_register_ifpmic(bcl_dev, &bcl_ifpmic_ops) == 0)
		data->bcl_dev = bcl_dev;

	return;
retry_init_work:
	schedule_delayed_work(&data->init_bcl, msecs_to_jiffies(1000));
}

static int max77759_init_bcl(struct max77759_chgr_data *data)
{
	u8 regdata;
	int ret;

	/* TODO: use RMW */
	ret = max77759_reg_read(data->regmap, MAX77759_CHG_CNFG_17, &regdata);
	if (ret == 0)
		ret = max77759_reg_write(data->regmap, MAX77759_CHG_CNFG_17,
					 regdata | BATOILO_DET_30US);
	if (ret < 0)
		return -EIO;

	INIT_DELAYED_WORK(&data->init_bcl, max77759_register_bcl);
	schedule_delayed_work(&data->init_bcl, msecs_to_jiffies(1000));

	return ret;
}
#endif

static void max77759_otg_fccm_worker(struct work_struct *work)
{
	struct max77759_chgr_data *data = container_of(work,
						struct max77759_chgr_data,
						otg_fccm_worker.work);
	int gpio_en, vbatt, ret;

	__pm_stay_awake(data->otg_fccm_wake_lock);

	if (data->uc_data.ext_bst_mode <= 0)
		goto done_relax;

	ret = max77759_read_vbatt(data, &vbatt);
	if (ret < 0) {
		schedule_delayed_work(&data->otg_fccm_worker, msecs_to_jiffies(50));
		goto done_relax;
        }

	vbatt = vbatt / 1000;
	gpio_en = gpio_get_value_cansleep(data->uc_data.ext_bst_mode);

	dev_dbg(data->dev, "fccm: vbatt=%d, gpio_en=%d\n", vbatt, gpio_en);

	if (vbatt > data->otg_fccm_vbatt_upperbd && !gpio_en) {
		dev_info(data->dev, "enable fccm mode.\n");
		gpio_set_value_cansleep(data->uc_data.ext_bst_mode, 1);
	} else if (vbatt < data->otg_fccm_vbatt_lowerbd && gpio_en) {
		dev_info(data->dev, "disable fccm mode.\n");
		gpio_set_value_cansleep(data->uc_data.ext_bst_mode, 0);
	}

	schedule_delayed_work(&data->otg_fccm_worker, msecs_to_jiffies(30000));
done_relax:
	__pm_relax(data->otg_fccm_wake_lock);
}

#define MAX77759_FCCM_UPPERBD_VOL 4600
#define MAX77759_FCCM_LOWERBD_VOL 3600
static int max77759_charger_probe(struct i2c_client *client,
				  const struct i2c_device_id *id)
{
	struct power_supply_config chgr_psy_cfg = { 0 };
	struct device *dev = &client->dev;
	struct max77759_chgr_data *data;
	struct regmap *regmap;
	const char *tmp;
	u32 usb_otg_mv;
	int ret = 0;
	u8 ping;

	regmap = devm_regmap_init_i2c(client, &max77759_chg_regmap_cfg);
	if (IS_ERR(regmap)) {
		dev_err(dev, "Failed to initialize regmap\n");
		return -EINVAL;
	}

	ret = max77759_reg_read(regmap, MAX77759_CHG_CNFG_00, &ping);
	if (ret < 0)
		return -ENODEV;

	/* TODO: PING or read HW version from PMIC */

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->dev = dev;
	data->regmap = regmap;
	data->fship_dtls = -1;
	data->wden = false; /* TODO: read from DT */
	mutex_init(&data->io_lock);
	atomic_set(&data->insel_cnt, 0);
	atomic_set(&data->early_topoff_cnt, 0);
	i2c_set_clientdata(client, data);

#if IS_ENABLED(CONFIG_GOOGLE_BCL)
	/*
	 * NOTE: this should possibly be retried later. Startip seems to be
	 * a little "racey".
	 */
	ret = max77759_init_bcl(data);
	if (ret < 0)
	        pr_err("Failed to register BCL callback %d\n", ret);
#endif

	data->usecase_wake_lock = wakeup_source_register(NULL, "max77759-usecase");
	if (!data->usecase_wake_lock) {
		pr_err("Failed to register wakeup source\n");
		return -ENODEV;
	}

	/* NOTE: only one instance */
	ret = of_property_read_string(dev->of_node, "max77759,psy-name", &tmp);
	if (ret == 0)
		max77759_psy_desc.name = devm_kstrdup(dev, tmp, GFP_KERNEL);

	chgr_psy_cfg.drv_data = data;
	chgr_psy_cfg.supplied_to = NULL;
	chgr_psy_cfg.num_supplicants = 0;
	data->psy = devm_power_supply_register(dev, &max77759_psy_desc,
		&chgr_psy_cfg);
	if (IS_ERR(data->psy)) {
		dev_err(dev, "Failed to register psy rc = %ld\n",
			PTR_ERR(data->psy));
		return -EINVAL;
	}

	/* CHARGER_MODE needs this (initialized to -EPROBE_DEFER) */
	gs101_setup_usecases(&data->uc_data, NULL);
	data->uc_data.client = client;

	INIT_DELAYED_WORK(&data->mode_rerun_work, max77759_mode_rerun_work);

	data->irq_gpio = of_get_named_gpio(dev->of_node, "max77759,irq-gpio", 0);
	if (data->irq_gpio < 0) {
		dev_err(dev, "failed get irq_gpio\n");
	} else {
		client->irq = gpio_to_irq(data->irq_gpio);

		ret = devm_request_threaded_irq(data->dev, client->irq, NULL,
						max77759_chgr_irq,
						IRQF_TRIGGER_LOW |
						IRQF_SHARED |
						IRQF_ONESHOT,
						"max77759_charger",
						data);
		if (ret == 0) {
			enable_irq_wake(client->irq);

			/* might cause the isr to be called */
			max77759_chgr_irq(-1, data);
			ret = max77759_writen(regmap, MAX77759_CHG_INT_MASK,
					      max77759_int_mask,
					      sizeof(max77759_int_mask));
			if (ret < 0)
				dev_err(dev, "cannot set irq_mask (%d)\n", ret);

			data->irq_disabled = false;
			data->irq_int = client->irq;
		}
	}

	ret = dbg_init_fs(data);
	if (ret < 0)
		dev_err(dev, "Failed to initialize debug fs\n");

	mutex_lock(&data->io_lock);
	ret = max77759_wdt_enable(data, data->wden);
	if (ret < 0)
		dev_err(dev, "wd enable=%d failed %d\n", data->wden, ret);

	/* disable fast charge safety timer */
	max77759_chg_reg_update(data->uc_data.client, MAX77759_CHG_CNFG_01,
				MAX77759_CHG_CNFG_01_FCHGTIME_MASK,
				MAX77759_CHG_CNFG_01_FCHGTIME_CLEAR);

	if (of_property_read_bool(dev->of_node, "google,max77759-thm2-monitor")) {
		/* enable THM2 monitor at 60 degreeC */
		max77759_chg_reg_update(data->uc_data.client, MAX77759_CHG_CNFG_13,
				MAX77759_CHG_CNFG_13_THM2_HW_CTRL |
				MAX77759_CHG_CNFG_13_USB_TEMP_MASK,
				0xA);
	} else if (!of_property_read_bool(dev->of_node, "max77759,usb-mon")) {
		/* b/193355117 disable THM2 monitoring */
		max77759_chg_reg_update(data->uc_data.client, MAX77759_CHG_CNFG_13,
					MAX77759_CHG_CNFG_13_THM2_HW_CTRL |
					MAX77759_CHG_CNFG_13_USB_TEMP_MASK,
					0);
	}

	mutex_unlock(&data->io_lock);

	data->otg_fccm_wake_lock = wakeup_source_register(NULL, "max77759-otg_fccm");
	if (!data->otg_fccm_wake_lock)
		pr_err("Failed to register otg_fccm wakeup source\n");

	ret = of_property_read_u32(dev->of_node,
				   "max77759,otg-fccm-vbatt-upperbd",
				   &data->otg_fccm_vbatt_upperbd);
	if (ret == 0)
		ret = of_property_read_u32(dev->of_node,
					   "max77759,otg-fccm-vbatt-lowerbd",
					   &data->otg_fccm_vbatt_lowerbd);
	data->otg_changed = false;
	data->otg_fccm_reset = ret == 0 &&
		data->otg_fccm_vbatt_upperbd > data->otg_fccm_vbatt_lowerbd &&
		data->otg_fccm_vbatt_upperbd < MAX77759_FCCM_UPPERBD_VOL &&
		data->otg_fccm_vbatt_lowerbd > MAX77759_FCCM_LOWERBD_VOL;
	if (data->otg_fccm_reset)
		dev_info(dev, "fccm_reset enabled lo=%dmV hi=%dmV\n",
			 data->otg_fccm_vbatt_lowerbd,
			 data->otg_fccm_vbatt_upperbd);

	INIT_DELAYED_WORK(&data->otg_fccm_worker, max77759_otg_fccm_worker);

	ret = of_property_read_u32(dev->of_node, "max77759,chg-term-voltage",
				   &data->chg_term_voltage);
	if (ret < 0)
		data->chg_term_voltage = 0;

	ret = of_property_read_u32(dev->of_node, "max77759,chg-term-volt-debounce",
				   &data->chg_term_volt_debounce);
	if (ret < 0)
		data->chg_term_volt_debounce = CHG_TERM_VOLT_DEBOUNCE;
	if (data->chg_term_voltage == 0)
		data->chg_term_volt_debounce = 0;

	ret = of_property_read_u32(dev->of_node, "max77759,usb-otg-mv", &usb_otg_mv);
	if (ret)
		dev_err(dev, "usb-otg-mv not found, using default\n");

	ret = max77759_otg_vbyp_mv_to_code(&data->uc_data.otg_value, ret ?
					   GS101_OTG_DEFAULT_MV : usb_otg_mv);
	if (ret < 0) {
		dev_err(dev, "Invalid value of USB OTG voltage, set to 5000\n");
		data->uc_data.otg_value = MAX77759_CHG_CNFG_11_OTG_VBYP_5000MV;
	}

	if (of_property_read_bool(dev->of_node, "max77759,dcin-is-dock"))
		data->uc_data.dcin_is_dock = true;
	else
		data->uc_data.dcin_is_dock = false;

	data->init_complete = 1;
	data->resume_complete = 1;

	/* other drivers (ex tcpci) need this. */
	ret = max77759_setup_votables(data);
	if (ret < 0)
		return ret;

	ret = max77759_init_wcin_psy(data);
	if (ret < 0)
		pr_err("Couldn't register dc power supply (%d)\n", ret);

	dev_info(dev, "registered as %s\n", max77759_psy_desc.name);
	return 0;
}

static int max77759_charger_remove(struct i2c_client *client)
{
	struct max77759_chgr_data *data = i2c_get_clientdata(client);

	if (data->de)
		debugfs_remove(data->de);
	wakeup_source_unregister(data->usecase_wake_lock);
	wakeup_source_unregister(data->otg_fccm_wake_lock);

	return 0;
}


static const struct of_device_id max77759_charger_of_match_table[] = {
	{ .compatible = "maxim,max77759chrg"},
	{},
};
MODULE_DEVICE_TABLE(of, max77759_charger_of_match_table);

static const struct i2c_device_id max77759_id[] = {
	{"max77759_charger", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, max77759_id);

#if defined CONFIG_PM
static int max77759_charger_pm_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct max77759_chgr_data *data = platform_get_drvdata(pdev);

	pm_runtime_get_sync(data->dev);
	data->resume_complete = false;
	pm_runtime_put_sync(data->dev);

	return 0;
}

static int max77759_charger_pm_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct max77759_chgr_data *data = platform_get_drvdata(pdev);

	pm_runtime_get_sync(data->dev);
	data->resume_complete = true;
	if (data->irq_disabled) {
		enable_irq(data->irq_int);
		data->irq_disabled = false;
	}

	pm_runtime_put_sync(data->dev);

	if ((data->otg_fccm_reset) && (data->otg_changed))
		mod_delayed_work(system_wq, &data->otg_fccm_worker, 0);

	return 0;
}
#endif

static const struct dev_pm_ops max77759_charger_pm_ops = {
	SET_LATE_SYSTEM_SLEEP_PM_OPS(
		max77759_charger_pm_suspend,
		max77759_charger_pm_resume)
};

static struct i2c_driver max77759_charger_i2c_driver = {
	.driver = {
		.name = "max77759-charger",
		.owner = THIS_MODULE,
		.of_match_table = max77759_charger_of_match_table,
#ifdef CONFIG_PM
		.pm = &max77759_charger_pm_ops,
#endif
	},
	.id_table = max77759_id,
	.probe    = max77759_charger_probe,
	.remove   = max77759_charger_remove,
};

module_i2c_driver(max77759_charger_i2c_driver);

MODULE_DESCRIPTION("Maxim 77759 Charger Driver");
MODULE_AUTHOR("AleX Pelosi <apelosi@google.com>");
MODULE_LICENSE("GPL");
