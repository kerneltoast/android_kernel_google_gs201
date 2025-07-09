/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2021 Google, LLC
 *
 */


#include <linux/kernel.h>
#include <linux/printk.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/regmap.h>
#include "max77759.h"
#include "max77759_charger.h"

/* ----------------------------------------------------------------------- */
static void gs101_ext_bst_mode(struct max77759_usecase_data *uc_data, int mode);
static int max77759_chgr_reg_write(struct i2c_client *client, u8 reg, u8 value)
{
	struct max77759_chgr_data *data;

	if (!client)
		return -ENODEV;

	data = i2c_get_clientdata(client);
	if (!data || !data->regmap)
		return -ENODEV;

	return regmap_write(data->regmap, reg, value);
}

static int max77759_chgr_reg_read(struct i2c_client *client, u8 reg, u8 *value)
{
	struct max77759_chgr_data *data;
	int ret, ival;

	if (!client)
		return -ENODEV;

	data = i2c_get_clientdata(client);
	if (!data || !data->regmap)
		return -ENODEV;

	ret = regmap_read(data->regmap, reg, &ival);
	if (ret == 0)
		*value = 0xFF & ival;

	return ret;
}

static int max77759_chgr_reg_update(struct i2c_client *client,
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

static int max77759_chgr_mode_write(struct i2c_client *client,
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


/* 1 if changed, 0 if not changed, or < 0 on error */
static int max77759_chgr_prot(struct regmap *regmap, bool enable)
{
	u8 value = enable ? 0 : MAX77759_CHG_CNFG_06_CHGPROT_MASK;
	u8 prot;
	int ret, ival;

	ret = regmap_read(regmap, MAX77759_CHG_CNFG_06, &ival);
	if (ret < 0)
		return -EIO;
	prot = 0xFF & ival;

	if ((prot & MAX77759_CHG_CNFG_06_CHGPROT_MASK) == value)
		return 0;

	ret = regmap_write_bits(regmap, MAX77759_CHG_CNFG_06,
				MAX77759_CHG_CNFG_06_CHGPROT_MASK,
				value);
	if (ret < 0)
		return -EIO;

	return 1;
}

static int max77759_chgr_insel_write(struct i2c_client *client, u8 mask, u8 value)
{
	struct max77759_chgr_data *data;
	int ret, prot;

	if (!client)
		return -ENODEV;

	data = i2c_get_clientdata(client);
	if (!data || !data->regmap)
		return -ENODEV;

	prot = max77759_chgr_prot(data->regmap, false);
	if (prot < 0)
		return -EIO;

	/* changing [CHGIN|WCIN]_INSEL: works when protection is disabled  */
	ret = regmap_write_bits(data->regmap, MAX77759_CHG_CNFG_12, mask, value);
	if (ret < 0 || prot == 0)
		return ret;

	prot = max77759_chgr_prot(data->regmap, true);
	if (prot < 0) {
		pr_err("%s: cannot restore protection bits (%d)\n",
		       __func__, prot);
		return prot;
	};

	return ret;
}

/* ----------------------------------------------------------------------- */

/* control VENDOR_EXTBST_CTRL (from TCPCI module) */
static int gs101_ls_mode(struct max77759_usecase_data *uc_data, int mode)
{
	int ret;

	pr_debug("%s: mode=%d ext_bst_ctl=%d lsw1_c=%d lsw1_o=%d\n", __func__, mode,
		uc_data->ext_bst_ctl, uc_data->lsw1_is_closed,
		uc_data->lsw1_is_open);

	if (uc_data->ext_bst_ctl < 0)
		return 0;

	/* VENDOR_EXTBST_CTRL control LSW1, the read will check the state */
	gpio_set_value_cansleep(uc_data->ext_bst_ctl, mode);

	/* b/182953320 load switch is optional */
	if (uc_data->lsw1_is_open < 0 || uc_data->lsw1_is_closed < 0)
		return 0;

	/* ret <= 0 if *_is* is not true and > 1 if true */
	switch (mode) {
	case 0:
		/* the OVP open right away */
		ret = gpio_get_value_cansleep(uc_data->lsw1_is_open);
		if (ret <= 0 && uc_data->ls1_en > 0) {
			const int max_count = 3;
			int loops;

			/*  do it manually and re-read after 20ms */
			for (loops = 0; loops < max_count; loops++) {
				gpio_set_value_cansleep(uc_data->ls1_en, 0);
				usleep_range(20 * USEC_PER_MSEC, 20 * USEC_PER_MSEC + 100);

				ret = gpio_get_value_cansleep(uc_data->lsw1_is_open);
				pr_debug("%s: open lsw1 attempt %d/%d ret=%d\n",
					 __func__, loops, max_count, ret);
				if (ret > 0)
					break;
			}
		}
		break;
	case 1:
		/* it takes 11 ms to turn on the OVP */
		usleep_range(11 * USEC_PER_MSEC, 11 * USEC_PER_MSEC + 100);
		ret = gpio_get_value_cansleep(uc_data->lsw1_is_closed);
		break;
	default:
		return -EINVAL;
	}

	return (ret <= 0) ? -EIO : 0;
}

/* OVP LS2 */
#define OVP_LS2_MODE_OFF	0
#define OVP_LS2_MODE_ON		1
static int gs101_ls2_mode(struct max77759_usecase_data *uc_data, int mode)
{
	pr_debug("%s: ls2_en=%d mode=%d\n", __func__, uc_data->ls2_en, mode);

	if (uc_data->ls2_en >= 0)
		gpio_set_value_cansleep(uc_data->ls2_en, !!mode);

	return 0;
}

/* control external boost mode
 * can be done controlling ls1, ls2
 */
#define EXT_MODE_OFF		0
#define EXT_MODE_OTG_5_0V	1
#define EXT_MODE_OTG_7_5V	2

/*
 * bst_on=GPIO5 on Max77759 on canopy and on all whitefins,
 * bst_sel=Granville
 */
static int gs101_ext_mode(struct max77759_usecase_data *uc_data, int mode)
{
	int ret = 0;

	pr_debug("%s: mode=%d on=%d sel=%d\n", __func__, mode,
		 uc_data->bst_on, uc_data->bst_sel);

	if (uc_data->bst_on < 0)
		return 0;

	switch (mode) {
	case EXT_MODE_OFF:
		gpio_set_value_cansleep(uc_data->bst_on, 0);
		break;
	case EXT_MODE_OTG_5_0V:
		if (uc_data->bst_sel > 0) {
			gpio_set_value_cansleep(uc_data->bst_sel, 0);
			usleep_range(100 * USEC_PER_MSEC, 100 * USEC_PER_MSEC + 100);
		}
		gpio_set_value_cansleep(uc_data->bst_on, 1);
		break;
	case EXT_MODE_OTG_7_5V: /* TODO: verify this */
		if (uc_data->bst_sel > 0) {
			gpio_set_value_cansleep(uc_data->bst_sel, 1);
			usleep_range(100 * USEC_PER_MSEC, 100 * USEC_PER_MSEC + 100);
		}
		gpio_set_value_cansleep(uc_data->bst_on, 1);
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

int gs101_wlc_en(struct max77759_usecase_data *uc_data, enum wlc_state_t state)
{
	int ret = 0;
	int wlc_on = 0;

	if (state == WLC_ENABLED)
		wlc_on = 1;

	pr_debug("%s: cpout_en=%d wlc_en=%d wlc_vbus_en=%d wlc_on=%d wlc_state=%d\n", __func__,
		 uc_data->cpout_en, uc_data->wlc_en, uc_data->wlc_vbus_en, wlc_on, state);

	if (uc_data->cpout_en >= 0) {
		if (state == WLC_SPOOFED && uc_data->wlc_spoof_gpio)
			gpio_set_value_cansleep(uc_data->wlc_spoof_gpio, 1);
		gpio_set_value_cansleep(uc_data->cpout_en, wlc_on);
	} else if (!wlc_on) {
		/*
		 * when
		 *   uc_data->cpout_en != -EPROBE_DEFER && uc_data->wlc_en
		 * could use uc_data->wlc_en with:
		 *   gpio_set_value_cansleep(uc_data->wlc_en, !!wlc_on);
		 *
		 * BUT need to resolve the race on start since toggling
		 * ->wlc_en might not be undone by using ->cpout_en
		 */
	}

	/* b/202526678 */
	if (uc_data->wlc_vbus_en >= 0)
		gpio_set_value_cansleep(uc_data->wlc_vbus_en, wlc_on);

	return ret;
}
EXPORT_SYMBOL_GPL(gs101_wlc_en);

/* RTX reverse wireless charging */
static int gs101_wlc_tx_enable(struct max77759_usecase_data *uc_data,
			       bool enable)
{
	int ret = 0;

	if (enable) {

		if (!uc_data->wlctx_bst_en_first)
			ret = gs101_ls2_mode(uc_data, OVP_LS2_MODE_ON);
		if (ret == 0)
			ret = gs101_ext_mode(uc_data, EXT_MODE_OTG_7_5V);
		if (ret == 0 && uc_data->wlctx_bst_en_first) {
			usleep_range(20 * USEC_PER_MSEC, 20 * USEC_PER_MSEC + 100);
			ret = gs101_ls2_mode(uc_data, OVP_LS2_MODE_ON);
		}
		if (ret < 0)
			return ret;

		usleep_range(100 * USEC_PER_MSEC, 100 * USEC_PER_MSEC + 100);

		/* p9412 will not be in RX when powered from EXT */
		ret = gs101_wlc_en(uc_data, WLC_ENABLED);
		if (ret < 0)
			return ret;

		if (uc_data->cpout21_en >= 0)
			gpio_set_value_cansleep(uc_data->cpout21_en, 0);
	} else {
		/* p9412 is already off from insel */
		ret = gs101_wlc_en(uc_data, WLC_DISABLED);
		if (ret < 0)
			return ret;

		/* NOTE: turn off WLC, no need to reset cpout */
		if (!uc_data->wlctx_bst_en_first)
			ret = gs101_ext_mode(uc_data, EXT_MODE_OFF);
		if (ret == 0)
			ret = gs101_ls2_mode(uc_data, OVP_LS2_MODE_OFF);
		if (ret == 0 && uc_data->wlctx_bst_en_first)
			ret = gs101_ext_mode(uc_data, EXT_MODE_OFF);

		/* STBY will re-enable WLC */
	}

	return ret;
}

/* change p9412 CPOUT and adjust WCIN_REG */
#define GS101_WLCRX_CPOUT_DFLT	0
#define GS101_WLCRX_CPOUT_5_2V	1

static int gs101_cpout_mode(struct max77759_usecase_data *uc_data, int mode)
{
	int ret;

	/* do not change MW unless p9412 can be changed as well */
	if (uc_data->cpout_ctl < 0)
		return 0;

	if (mode == GS101_WLCRX_CPOUT_5_2V) {
		/* p9412: set CPOUT==5.2 only if on BPP */
		gpio_set_value_cansleep(uc_data->cpout_ctl, 1);

		/* NOTE: no DC_IN to MW when WCIN_REG==4_85 unless CPOUT==5.2 */
		ret =  max77759_chgr_reg_update(uc_data->client, MAX77759_CHG_CNFG_12,
					       MAX77759_CHG_CNFG_12_WCIN_REG_MASK,
					       MAX77759_CHG_CNFG_12_WCIN_REG_4_85);
	} else {
		/* p9412: reset CPOUT to default */
		gpio_set_value_cansleep(uc_data->cpout_ctl, 0);

		ret =  max77759_chgr_reg_update(uc_data->client, MAX77759_CHG_CNFG_12,
					       MAX77759_CHG_CNFG_12_WCIN_REG_MASK,
					       MAX77759_CHG_CNFG_12_WCIN_REG_4_5);
	}

	return ret;
}

static int gs101_otg_bypass_enable(struct max77759_usecase_data *uc_data, u8 vbyp)
{
	int ret;

	ret = max77759_chgr_reg_write(uc_data->client, MAX77759_CHG_CNFG_11, vbyp);
	if (ret < 0) {
		pr_err("%s: cannot set vbypset (%d)\n", __func__, ret);
		return ret;
	}

	return max77759_chgr_reg_update(uc_data->client, MAX77759_CHG_CNFG_18,
				       MAX77759_CHG_CNFG_18_OTG_V_PGM,
				       MAX77759_CHG_CNFG_18_OTG_V_PGM);
}

static int gs101_otg_bypass_disable(struct max77759_usecase_data *uc_data)
{
    return max77759_chgr_reg_update(uc_data->client, MAX77759_CHG_CNFG_18,
				   MAX77759_CHG_CNFG_18_OTG_V_PGM, 0);
}

static int gs101_otg_update_ilim(struct max77759_usecase_data *uc_data, int enable)
{
	u8 ilim;

	if (uc_data->otg_orig == uc_data->otg_ilim)
		return 0;

	if (enable) {
		int rc;

		rc = max77759_chgr_reg_read(uc_data->client, MAX77759_CHG_CNFG_05,
					   &uc_data->otg_orig);
		if (rc < 0) {
			pr_err("%s: cannot read otg_ilim (%d), use default\n",
			       __func__, rc);
			uc_data->otg_orig = MAX77759_CHG_CNFG_05_OTG_ILIM_1500MA;
		} else {
			uc_data->otg_orig &= MAX77759_CHG_CNFG_05_OTG_ILIM_MASK;
		}

		ilim = uc_data->otg_ilim;
	} else {
		ilim = uc_data->otg_orig;
	}

	return max77759_chgr_reg_update(uc_data->client, MAX77759_CHG_CNFG_05,
				      MAX77759_CHG_CNFG_05_OTG_ILIM_MASK,
				      ilim);
}

static int gs101_otg_frs(struct max77759_usecase_data *uc_data, int enable)
{
	int ret;

	ret = max77759_chgr_mode_write(uc_data->client, MAX77759_CHGR_MODE_OTG_BOOST_ON);
	if (ret < 0) {
		pr_err("%s: cannot set CNFG_00 to 0xa ret:%d\n", __func__, ret);
		return ret;
	}

	ret = gs101_otg_update_ilim(uc_data, enable);
	if (ret < 0)
		pr_err("%s: cannot update otg_ilim: %d\n", __func__, ret);

	return ret;
}

static int gs101_pogo_vout_enable(struct max77759_usecase_data *uc_data,
			       bool enable)
{
	int ret;

	pr_debug("%s: pogo_vout_en (%d)\n", __func__, enable);

	ret = gs101_ext_mode(uc_data, enable ? EXT_MODE_OTG_5_0V : EXT_MODE_OFF);
	if (ret == 0 && uc_data->pogo_vout_en > 0)
		gpio_set_value_cansleep(uc_data->pogo_vout_en, enable);

	return ret;
}

/*
 * Transition to standby (if needed) at the beginning of the sequences
 * @return <0 on error, 0 on success. ->use_case becomes GSU_MODE_STANDBY
 * if the transition is necessary (and successful).
 */
int gs101_to_standby(struct max77759_usecase_data *uc_data, int use_case)
{
	const int from_uc = uc_data->use_case;
	bool need_stby = false;
	bool from_otg = false;
	int ret;

	switch (from_uc) {
	case GSU_MODE_USB_CHG:
		need_stby = use_case != GSU_MODE_USB_CHG_WLC_TX &&
			    use_case != GSU_MODE_WLC_RX &&
			    use_case != GSU_MODE_DOCK &&
			    use_case != GSU_MODE_USB_DC &&
			    use_case != GSU_MODE_USB_OTG_FRS &&
			    use_case != GSU_MODE_USB_CHG_POGO_VOUT;
		break;
	case GSU_MODE_WLC_RX:
		need_stby = use_case != GSU_MODE_USB_OTG_WLC_RX &&
			    use_case != GSU_MODE_WLC_DC;
		break;
	case GSU_MODE_WLC_TX:
		need_stby = use_case != GSU_MODE_USB_OTG_WLC_TX &&
			    use_case != GSU_MODE_USB_CHG_WLC_TX &&
			    use_case != GSU_MODE_USB_DC_WLC_TX &&
			    use_case != GSU_MODE_USB_OTG_FRS;
		break;
	case GSU_MODE_USB_CHG_WLC_TX:
		need_stby = use_case != GSU_MODE_USB_CHG &&
			    use_case != GSU_MODE_USB_OTG_WLC_TX &&
			    use_case != GSU_MODE_USB_DC;
		break;

	case GSU_MODE_USB_OTG:
		from_otg = true;
		if (use_case == GSU_MODE_USB_OTG_FRS)
			break;
		if (use_case == GSU_MODE_USB_OTG_WLC_TX)
			break;
		if (use_case == GSU_MODE_USB_OTG_WLC_RX)
			break;
		if (use_case == GSU_MODE_USB_OTG_WLC_DC)
			break;
		if (use_case == GSU_MODE_USB_OTG_POGO_VOUT)
			break;

		/* From 5. USB OTG to anything else, go to stby */
		gs101_ext_bst_mode(uc_data, 0);

		ret = gs101_ls_mode(uc_data, 0);
		if (ret == 0)
			ret = gs101_ext_mode(uc_data, 0);
		if (ret < 0)
			return -EIO;

		/* TODO:Discharge IN/OUT with AO37 is done in TCPM */
		msleep(100);

		need_stby = true;
		break;

	case GSU_MODE_USB_OTG_WLC_RX:
		from_otg = true;
		need_stby = use_case != GSU_MODE_WLC_RX &&
			    use_case != GSU_MODE_DOCK &&
			    use_case != GSU_MODE_USB_OTG;
		break;
	case GSU_MODE_USB_DC:
		need_stby = use_case != GSU_MODE_USB_DC;
		break;
	case GSU_MODE_WLC_DC:
		need_stby = use_case != GSU_MODE_WLC_DC;
		break;

	case GSU_MODE_USB_OTG_FRS:
		from_otg = true;
		need_stby = use_case != GSU_MODE_USB_OTG_FRS &&
			    use_case != GSU_MODE_USB_OTG_WLC_TX &&
			    use_case != GSU_MODE_USB_OTG_POGO_VOUT;
		break;
		/*
		 *  if (use_case == GSU_MODE_USB_OTG)
		 * 	break;
		 */
	case GSU_RAW_MODE:
		need_stby = true;
		break;
	case GSU_MODE_USB_OTG_WLC_TX:
	case GSU_MODE_USB_OTG_POGO_VOUT:
		from_otg = true;
		need_stby = false;
		break;
	case GSU_MODE_POGO_VOUT:
		need_stby = use_case != GSU_MODE_USB_OTG_POGO_VOUT &&
			    use_case != GSU_MODE_USB_CHG_POGO_VOUT &&
			    use_case != GSU_MODE_USB_OTG_FRS;
		break;
	case GSU_MODE_USB_CHG_POGO_VOUT:
		need_stby = use_case != GSU_MODE_USB_CHG;
		break;
	case GSU_MODE_STANDBY:
	default:
		need_stby = false;
		break;
	}

	if (use_case == GSU_MODE_USB_WLC_RX || use_case == GSU_RAW_MODE)
		need_stby = true;

	pr_info("%s: use_case=%d->%d from_otg=%d need_stby=%d\n", __func__,
		 from_uc, use_case, from_otg, need_stby);

	if (!need_stby)
		return 0;

	/* there are no ways out of OTG FRS */
	if (from_uc == GSU_MODE_USB_OTG_FRS) {
		ret = gs101_otg_frs(uc_data, true);
		if (ret < 0) {
			pr_err("%s: cannot turn off OTG_FRS (%d)\n", __func__, ret);
			return ret;
		}
	}

	/* from WLC_TX to STBY */
	if (from_uc == GSU_MODE_WLC_TX) {
		ret = gs101_wlc_tx_enable(uc_data, false);
		if (ret < 0) {
			pr_err("%s: cannot tun off wlc_tx (%d)\n", __func__, ret);
			return ret;
		}

		/* re-enable wlc IC if disabled */
		ret = gs101_wlc_en(uc_data, WLC_ENABLED);
		if (ret < 0)
			pr_err("%s: cannot enable WLC (%d)\n", __func__, ret);
	}

	/* from WLC-DC to STBY */
	if (from_uc == GSU_MODE_WLC_DC) {
		if (uc_data->dc_sw_gpio > 0)
			gpio_set_value_cansleep(uc_data->dc_sw_gpio, 0);
		ret = gs101_ext_mode(uc_data, EXT_MODE_OFF);
		if (ret < 0) {
			pr_debug("%s: cannot change extmode ret:%d\n",
				 __func__, ret);
			return ret;
		}
	}

	/*
	 * There is no direct transition to STBY from BPP_RX+OTG  but we might
	 * get here on error and when forcing raw values. This makes sure that
	 * CPOUT is set to default.
	 */
	if (from_uc == GSU_MODE_USB_OTG_WLC_RX) {
		ret = gs101_cpout_mode(uc_data, GS101_WLCRX_CPOUT_DFLT);
		if (ret < 0)
			pr_err("%s: cannot reset cpout (%d)\n", __func__, ret);

		/* Reset the extbst mode */
		if (uc_data->wlc_otg_extbst_en)
			gs101_ext_bst_mode(uc_data, 0);
	}

	/* from POGO_VOUT to STBY */
	if (from_uc == GSU_MODE_POGO_VOUT) {
		ret = gs101_pogo_vout_enable(uc_data, false);
		if (ret < 0)
			pr_err("%s: cannot tun off pogo_vout (%d)\n", __func__, ret);
	}

	/* b/178458456 exit from all OTG cases need to reset the limit */
	if (uc_data->otg_enable > 0)
		gpio_set_value_cansleep(uc_data->otg_enable, 0);

	/* transition to STBY (might need to be up) */
	ret = max77759_chgr_mode_write(uc_data->client, MAX77759_CHGR_MODE_ALL_OFF);
	if (ret < 0)
		return -EIO;

	uc_data->use_case = GSU_MODE_STANDBY;
	return ret;
}
EXPORT_SYMBOL_GPL(gs101_to_standby);

/* enable/disable soft-start. No need soft start from OTG->OTG_FRS */
static int gs101_ramp_bypass(struct max77759_usecase_data *uc_data, bool enable)
{
	const u8 value = enable ? MAX77759_CHG_CNFG_00_BYPV_RAMP_BYPASS : 0;

	if (uc_data->is_a1 <= 0)
		return 0;

	return max77759_chgr_reg_update(uc_data->client, MAX77759_CHG_CNFG_00,
				       MAX77759_CHG_CNFG_00_BYPV_RAMP_BYPASS,
				       value);
}

/* cleanup from every usecase */
int gs101_force_standby(struct max77759_usecase_data *uc_data)
{
	const u8 insel_mask = MAX77759_CHG_CNFG_12_CHGINSEL_MASK |
			      MAX77759_CHG_CNFG_12_WCINSEL_MASK;
	const u8 insel_value = MAX77759_CHG_CNFG_12_CHGINSEL |
			       MAX77759_CHG_CNFG_12_WCINSEL;
	int ret;

	pr_debug("%s: recovery\n", __func__);

	ret = gs101_ls_mode(uc_data, 0);
	if (ret < 0)
		pr_err("%s: cannot change ls_mode (%d)\n",
			__func__, ret);

	ret = gs101_ext_mode(uc_data, 0);
	if (ret < 0)
		pr_err("%s: cannot change ext mode (%d)\n",
			__func__, ret);

	ret = gs101_ramp_bypass(uc_data, false);
	if (ret < 0)
		pr_err("%s: cannot reset ramp_bypass (%d)\n",
			__func__, ret);

	ret = gs101_pogo_vout_enable(uc_data, false);
	if (ret < 0)
		pr_err("%s: cannot tun off pogo_vout (%d)\n",
			__func__, ret);

	ret = max77759_chgr_mode_write(uc_data->client, MAX77759_CHGR_MODE_ALL_OFF);
	if (ret < 0)
		pr_err("%s: cannot reset mode register (%d)\n",
			__func__, ret);

	ret = max77759_chgr_insel_write(uc_data->client, insel_mask, insel_value);
	if (ret < 0)
		pr_err("%s: cannot reset insel (%d)\n",
			__func__, ret);

	gs101_ext_bst_mode(uc_data, 0);

	return 0;
}
EXPORT_SYMBOL_GPL(gs101_force_standby);

/* b/188488966 */
static int gs101_frs_to_otg(struct max77759_usecase_data *uc_data)
{
	int closed, ret;

	ret = gs101_ext_mode(uc_data, EXT_MODE_OTG_5_0V);
	if (ret < 0)
		goto exit_done;

	msleep(100);

	if (uc_data->ls1_en > 0)
		gpio_set_value_cansleep(uc_data->ls1_en, 1);

	msleep(100);

	if (uc_data->lsw1_is_closed >= 0)
		closed = gpio_get_value_cansleep(uc_data->lsw1_is_closed);

exit_done:
	pr_debug("%s: ls1_en=%d lsw1_is_closed=%d closed=%d ret=%d\n",
		 __func__, uc_data->ls1_en, uc_data->lsw1_is_closed,
		 closed, ret);
	return ret;
}

/* From OTG <-> OTG_FRS */
static int gs101_otg_mode(struct max77759_usecase_data *uc_data, int to)
{
	int ret = -EINVAL;

	pr_debug("%s: to=%d\n", __func__, to);

	/* no transition needed if only use external boost OTG */
	if (uc_data->ext_otg_only > 0)
		return 0;

	if (to == GSU_MODE_USB_OTG) {

		ret = gs101_ext_mode(uc_data, EXT_MODE_OTG_5_0V);
		if (ret < 0)
			return ret;

		usleep_range(5 * USEC_PER_MSEC, 5 * USEC_PER_MSEC + 100);

		ret = max77759_chgr_mode_write(uc_data->client,
					      MAX77759_CHGR_MODE_ALL_OFF);

	} else if (to == GSU_MODE_USB_OTG_FRS) {
		int rc;

		ret = gs101_ramp_bypass(uc_data, true);
		if (ret == 0)
			ret = max77759_chgr_mode_write(uc_data->client,
						      MAX77759_CHGR_MODE_OTG_BOOST_ON);
		if (ret < 0)
			return ret;

		usleep_range(5 * USEC_PER_MSEC, 5 * USEC_PER_MSEC + 100);

		rc =  gs101_ramp_bypass(uc_data, false);
		if (rc < 0)
			pr_err("%s: cannot clear bypass rc:%d\n",  __func__, rc);

		/* b/192986752 make sure that LSW1 is open before going to FRS */
		rc = gs101_ls_mode(uc_data, 0);
		if (rc < 0)
			pr_err("%s: cannot clear lsw1 rc:%d\n",  __func__, rc);

		ret = gs101_ext_mode(uc_data, EXT_MODE_OFF);
		if (ret < 0)
			return ret;
	}

	return ret;
}

/*
 * This must follow different paths depending on the platforms.
 *
 * When vin_is_valid is implemented the code uses the NBC workaround for MW
 * and the OVP. The code defaults to just setting the MODE register (at the
 * end of the use case) and toggling bst_on/bst_sel and setting ext_bst_ctl
 * otherwise.
 *
 * NOTE: the USB stack expects VBUS to be on after voting for the usecase.
 */
static int gs101_otg_enable(struct max77759_usecase_data *uc_data, int mode)
{
	int ret, retn;

	/* the code default to write to the MODE register */
	if (uc_data->vin_is_valid >= 0) {

		/* b/178458456 */
		if (uc_data->otg_enable > 0)
			gpio_set_value_cansleep(uc_data->otg_enable, 1);

		/* NBC workaround */
		ret = gs101_ls_mode(uc_data, 1);
		if (ret < 0) {
			pr_debug("%s: cannot close load switch (%d)\n",
				 __func__, ret);
			return ret;
		}

		ret = gs101_otg_bypass_enable(uc_data, uc_data->otg_value);
		if (ret < 0) {
			pr_debug("%s: cannot set otg voltage (%d)\n", __func__, ret);
			return ret;
		}

		ret = max77759_chgr_mode_write(uc_data->client,
					      MAX77759_CHGR_MODE_OTG_BOOST_ON);
		if (ret < 0) {
			pr_debug("%s: cannot set CNFG_00 to 0xa ret:%d\n",
				 __func__, ret);
			return ret;
		}

		ret = gpio_get_value_cansleep(uc_data->vin_is_valid);
		if (ret == 0) {
			pr_debug("%s: VIN not VALID\n",  __func__);
			ret = -EIO;
			goto reset_otg_voltage;
		}

		ret = gs101_ext_mode(uc_data, mode);
		if (ret < 0)
			pr_debug("%s: cannot change extmode ret:%d\n",
				 __func__, ret);

	} else {
		ret = gs101_otg_bypass_enable(uc_data, uc_data->otg_value);
		if (ret < 0) {
			pr_debug("%s: cannot set otg voltage (%d)\n", __func__, ret);
			return ret;
		}

		/* ext mode is defined when ext boost is avalaible */
		ret = gs101_ext_mode(uc_data, mode);
		if (ret < 0) {
			pr_debug("%s: cannot change extmode ret:%d\n",
				 __func__, ret);
			goto reset_otg_voltage;
		}

		usleep_range(5 * USEC_PER_MSEC, 5 * USEC_PER_MSEC + 100);

		/* load switch from MW */
		ret = gs101_ls_mode(uc_data, 1);
		if (ret < 0) {
			pr_debug("%s: cannot close load switch (%d)\n",
				 __func__, ret);
			goto reset_otg_voltage;
		}

		/* time for VBUS to be on (could check PWRSTAT in MW) */
		msleep(30);

		/* b/178458456 */
		if (uc_data->otg_enable > 0)
			gpio_set_value_cansleep(uc_data->otg_enable, 0);
	}

	goto exit;

reset_otg_voltage:
	retn = gs101_otg_bypass_disable(uc_data);
	if (retn < 0)
		pr_debug("%s: cannot reset otg voltage (%d)\n", __func__, retn);
exit:
	return ret;
}

/* configure ilim wlctx */
static int gs101_wlctx_otg_en(struct max77759_usecase_data *uc_data, bool enable)
{
	int ret;

	if (enable) {
		/* this should be already set */
		if (uc_data->sw_en >= 0)
			gpio_set_value_cansleep(uc_data->sw_en, 1);

		ret = gs101_otg_update_ilim(uc_data, true);
		if (ret < 0)
			pr_err("%s: cannot update otg_ilim (%d)\n", __func__, ret);

		ret = gs101_otg_bypass_enable(uc_data, uc_data->otg_vbyp);
		if (ret < 0)
			pr_err("%s: cannot set otg_vbyp (%d)\n", __func__, ret);
	} else {
		/* TODO: Discharge IN/OUT nodes with AO37 should be done in TCPM */

		msleep(100);

		/* "bad things will happen (tm)" if you force off ->sw_en */

		ret = gs101_otg_update_ilim(uc_data, false);
		if (ret < 0)
			pr_err("%s: cannot restore otg_ilim (%d)\n", __func__, ret);

		/* TODO: restore initial value on !MAX77759_CHG_CNFG_11 */
		ret = gs101_otg_bypass_disable(uc_data);
		if (ret < 0)
			pr_err("%s: cannot reset otg_v_pgm (%d)\n", __func__, ret);

		ret = 0;
        }

	return ret;
}

/*
 * Control the extbst mode enable/disable by the platform:
 * otg_fccm_reset = 1, ignore the ext_bst_mode control
 */
static void gs101_ext_bst_mode(struct max77759_usecase_data *uc_data, int mode)
{
	struct max77759_chgr_data *data;

	pr_debug("%s: ext_bst_mode=%d mode=%d\n", __func__, uc_data->ext_bst_mode, mode);

	if (uc_data->ext_bst_mode <= 0)
		return;

	if (!uc_data->client)
		goto write_bst_mode;

	data = i2c_get_clientdata(uc_data->client);
	if (data && data->otg_fccm_reset) {
		pr_info("Set FCCM on 77759's callback\n");
		return;
	}

write_bst_mode:
	gpio_set_value_cansleep(uc_data->ext_bst_mode, mode);
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
 * WLC_chg = 0 off, 1 = on, 2 = PPS
 * Ext_Boost = 0 off, 1 = OTG 5V, 2 = WTX 7.5
 *
 * 5-1: mode=0x0 in MW, EXT_B=1, LS1=1, LS2=0, IDLE <-> OTG (ext)
 * 5-2: mode=0xa in MW, EXT_B=0, LS1=0, LS2=0, IDLE <-> OTG_FRS
 * 7-2: mode=0xa in MW, EXT_B=2, LS1=0, LS2=1
 *
 * AO37 + GPIO5 MW (canopy 3, whitev2p2)
 * . AO_ls1 <&max20339_gpio 0 GPIO_ACTIVE_HIGH> - bit0
 * . AO_ls2 <&max20339_gpio 1 GPIO_ACTIVE_HIGH> - bit4
 *
 * ls1 can be controlled poking the AO37 OR using a MW_GPIO _> EXT_BST_EN
 *
 * max77759,bst_on = <&max777x9_gpio 4 GPIO_ACTIVE_HIGH>
 * max77759,bst-sel = <&gpp27 3 GPIO_ACTIVE_HIGH>
 * max77759,bst_on=0, max77759,bst_sel=x => OFF
 * max77759,bst_on=1, max77759,bst_sel=0 => 5V
 * max77759,bst_on=1, max77759,bst_sel=1 => 7.5V
 *
 * Ext_Boost = 0 off
 * 	MW_gpio5	: Ext_B = 0, MW_gpio5 -> LOW
 * 	AO_ls1/ls2	: 0/0
 *
 * Ext_Boost = 1 = OTG 5V
 * 	MW_gpio5	: Ext_B = 1, MW_gpio5 -> HIGH
 * 	AO_ls1/ls2	: 1/0
 *
 * Ext_Boost = 2 WTX 7.5
 * 	MW_gpio5	: Ext_B = 2, MW_gpio5 -> HIGH
 * 	AO_ls1/ls2	: 0/1
 *
 * NOTE: do not call with (cb_data->wlc_rx && cb_data->wlc_tx)
 */

static int gs101_standby_to_otg(struct max77759_usecase_data *uc_data, int use_case)
{
	int ret;
	const int mode = (uc_data->ext_otg_only || use_case != GSU_MODE_USB_OTG_FRS) ?
			 EXT_MODE_OTG_5_0V : EXT_MODE_OFF;

	ret = gs101_otg_enable(uc_data, mode);
	if (ret == 0 && uc_data->ext_otg_only && !uc_data->wlc_otg_extbst_en)
		gs101_ext_bst_mode(uc_data, 1);

	if (ret == 0)
		usleep_range(5 * USEC_PER_MSEC, 5 * USEC_PER_MSEC + 100);
	/*
	 * Assumption: gs101_to_usecase() will write back cached values to
	 * CHG_CNFG_00.Mode. At the moment, the cached value at
	 * max77759_mode_callback is 0. If the cached value changes to something
	 * other than 0, then, the code has to be revisited.
	 */

	return ret;
}

/* was b/179816224 WLC_RX -> WLC_RX + OTG (Transition #10) */
static int gs101_wlcrx_to_wlcrx_otg(struct max77759_usecase_data *uc_data)
{
	pr_warn("%s: disabled\n", __func__);
	return 0;
}

static int gs101_to_otg_usecase(struct max77759_usecase_data *uc_data, int use_case)
{
	const int from_uc = uc_data->use_case;
	int ret = 0;

	switch (from_uc) {
	/* 5-1: #3: stby to USB OTG, mode = 1 */
	/* 5-2: #3: stby to USB OTG_FRS, mode = 0 */
	case GSU_MODE_STANDBY:
		ret = gs101_standby_to_otg(uc_data, use_case);
		if (ret < 0) {
			pr_err("%s: cannot enable OTG ret:%d\n",  __func__, ret);
			return ret;
		}
	break;

	/* b/186535439 : USB_CHG->USB_OTG_FRS*/
	case GSU_MODE_USB_CHG:
	case GSU_MODE_USB_CHG_WLC_TX:
		/* need to go through stby out of this */
		if (use_case != GSU_MODE_USB_OTG_FRS && use_case != GSU_MODE_USB_OTG_WLC_TX)
			return -EINVAL;

		ret = gs101_otg_frs(uc_data, true);
	break;


	case GSU_MODE_WLC_TX:
		/* b/179820595: WLC_TX -> WLC_TX + OTG */
		if (use_case == GSU_MODE_USB_OTG_WLC_TX) {
			ret = max77759_chgr_mode_write(uc_data->client, MAX77759_CHGR_MODE_OTG_BOOST_ON);
			if (ret < 0) {
				pr_err("%s: cannot set CNFG_00 to 0xa ret:%d\n",  __func__, ret);
				return ret;
			}
		}
	break;

	case GSU_MODE_WLC_RX:
	case GSU_MODE_DOCK:
		if (use_case == GSU_MODE_USB_OTG_WLC_RX) {
			/* OTG+WLC set extbst mode to high */
			if (uc_data->wlc_otg_extbst_en)
				gs101_ext_bst_mode(uc_data, 1);

			if (uc_data->rx_otg_en) {
				ret = gs101_cpout_mode(uc_data, GS101_WLCRX_CPOUT_5_2V);
				if (ret == 0)
					ret = gs101_standby_to_otg(uc_data, use_case);
			} else {
				ret = gs101_wlcrx_to_wlcrx_otg(uc_data);
			}
		}
	break;

	case GSU_MODE_USB_OTG:
		/* b/179820595: OTG -> WLC_TX + OTG (see b/181371696) */
		if (use_case == GSU_MODE_USB_OTG_WLC_TX) {
			ret = gs101_otg_mode(uc_data, GSU_MODE_USB_OTG_FRS);
			if (ret == 0)
				ret = gs101_wlc_tx_enable(uc_data, true);
		}
		/* b/179816224: OTG -> WLC_RX + OTG */
		if (use_case == GSU_MODE_USB_OTG_WLC_RX) {
			/* OTG+WLC set extbst mode to high */
			if (uc_data->wlc_otg_extbst_en) {
				gs101_ext_bst_mode(uc_data, 1);

				ret = gs101_cpout_mode(uc_data,
						       GS101_WLCRX_CPOUT_5_2V);
			} else {
				ret = gs101_cpout_mode(uc_data,
						       GS101_WLCRX_CPOUT_5_2V);
				if (!ret)
					gs101_ext_bst_mode(uc_data, 1);
			}
		}
		if (use_case == GSU_MODE_USB_OTG_POGO_VOUT) {
			ret = gs101_otg_mode(uc_data, GSU_MODE_USB_OTG_FRS);
			if (ret == 0)
				ret = gs101_pogo_vout_enable(uc_data, true);
		}
	break;
	case GSU_MODE_USB_OTG_WLC_TX:
		/*  b/179820595: WLC_TX + OTG -> OTG */
		if (use_case == GSU_MODE_USB_OTG) {
			ret = gs101_wlc_tx_enable(uc_data, false);
			if (ret == 0)
				ret = gs101_frs_to_otg(uc_data);
		}
	break;
	case GSU_MODE_USB_OTG_WLC_RX:
		/* b/179816224: WLC_RX + OTG -> OTG */
		if (use_case == GSU_MODE_USB_OTG) {
			/* it's in STBY, no need to reset gs101_otg_mode()  */
			if (!uc_data->ext_otg_only) {
				gs101_ext_bst_mode(uc_data, 0);

				ret = gs101_cpout_mode(uc_data,
						       GS101_WLCRX_CPOUT_DFLT);
			} else {
				/* OTG only, set extbst mode to low */
				ret = gs101_cpout_mode(uc_data,
						       GS101_WLCRX_CPOUT_DFLT);
				if (uc_data->wlc_otg_extbst_en)
					gs101_ext_bst_mode(uc_data, 0);
			}
		}
	break;
	/* TODO: */
	case GSU_MODE_USB_OTG_FRS: {
		if (use_case == GSU_MODE_USB_OTG_WLC_TX) {
			ret = gs101_wlc_tx_enable(uc_data, true);
			break;
		}
		if (use_case == GSU_MODE_USB_OTG_POGO_VOUT) {
			ret = gs101_pogo_vout_enable(uc_data, true);
			break;
		}
		/*
		 * OTG source handover: OTG_FRS -> OTG
		 * from EXT_BST (Regular OTG) to IF-PMIC OTG (FRS OTG)
		 */
		if (use_case != GSU_MODE_USB_OTG)
			return -EINVAL;

		/* TODO: */
	} break;

	case GSU_MODE_POGO_VOUT:
		if (use_case == GSU_MODE_USB_OTG_POGO_VOUT) {
			ret = max77759_chgr_mode_write(uc_data->client, MAX77759_CHGR_MODE_OTG_BOOST_ON);
			if (ret < 0) {
				pr_err("%s: cannot set CNFG_00 to 0xa ret:%d\n",  __func__, ret);
				return ret;
			}
		}
	break;
	case GSU_MODE_USB_OTG_POGO_VOUT:
		if (use_case == GSU_MODE_USB_OTG) {
			ret = gs101_pogo_vout_enable(uc_data, false);
			if (ret == 0)
				ret = gs101_frs_to_otg(uc_data);
		}
	break;

	default:
		return -ENOTSUPP;
	}

	return ret;
}

/* handles the transition data->use_case ==> use_case */
int gs101_to_usecase(struct max77759_usecase_data *uc_data, int use_case)
{
	const int from_uc = uc_data->use_case;
	int ret = 0;

	switch (use_case) {
	case GSU_MODE_USB_OTG:
	case GSU_MODE_USB_OTG_FRS:
	case GSU_MODE_USB_OTG_WLC_RX:
	case GSU_MODE_USB_OTG_WLC_DC:
	case GSU_MODE_USB_OTG_WLC_TX:
	case GSU_MODE_USB_OTG_POGO_VOUT:
		ret = gs101_to_otg_usecase(uc_data, use_case);
		break;
	case GSU_MODE_WLC_TX:
	case GSU_MODE_USB_CHG_WLC_TX:
		/* Coex Case #4, WLC_TX + OTG -> WLC_TX */
		if (from_uc == GSU_MODE_USB_OTG_WLC_TX) {
			ret = max77759_chgr_mode_write(uc_data->client,
						      MAX77759_CHGR_MODE_ALL_OFF);
			if (ret == 0)
				ret = gs101_wlctx_otg_en(uc_data, false);
		} else {
			ret = gs101_wlc_tx_enable(uc_data, true);
		}

		break;
	case GSU_MODE_WLC_RX:
	case GSU_MODE_DOCK:
		if (from_uc == GSU_MODE_USB_OTG_WLC_RX) {
			/* to_stby brought to stby */
			gs101_ext_bst_mode(uc_data, 0);
			if (uc_data->ext_otg_only) {
				if (ret == 0)
					ret = gs101_ls_mode(uc_data, 0);
				if (ret == 0)
					ret = gs101_ext_mode(uc_data, 0);
				if (ret == 0)
					ret = gs101_cpout_mode(uc_data, GS101_WLCRX_CPOUT_DFLT);
			} else {
				if (ret == 0)
					ret = gs101_cpout_mode(uc_data, GS101_WLCRX_CPOUT_DFLT);
				if (ret == 0)
					ret = gs101_otg_mode(uc_data, GSU_MODE_USB_OTG);
			}
		}
		if (from_uc == GSU_MODE_WLC_DC) {
			ret = gs101_ext_mode(uc_data, EXT_MODE_OFF);
			if (ret < 0) {
				pr_debug("%s: cannot change extmode ret:%d\n",
					 __func__, ret);
				return ret;
			}
		}
		break;
	case GSU_MODE_USB_CHG:
	case GSU_MODE_USB_DC:
		if (from_uc == GSU_MODE_WLC_TX || from_uc == GSU_MODE_USB_CHG_WLC_TX)
			ret = gs101_wlc_tx_enable(uc_data, false);

		if (from_uc == GSU_MODE_USB_CHG_POGO_VOUT)
			ret = gs101_pogo_vout_enable(uc_data, false);
		/* b/232723240: charge over USB-C
		 *              set to 0 for POGO_OVP_EN
		 *              set to 1 for POGO_OVP_EN_L
		 */
		if (uc_data->pogo_ovp_en > 0)
			gpio_set_value_cansleep(uc_data->pogo_ovp_en, uc_data->pogo_ovp_en_act_low);
		break;
	case GSU_MODE_USB_WLC_RX:
	case GSU_RAW_MODE:
		/* just write the value to the register (it's in stby) */
		break;
	case GSU_MODE_WLC_DC:
		if (uc_data->dc_sw_gpio > 0)
			gpio_set_value_cansleep(uc_data->dc_sw_gpio, 1);
		ret = gs101_ext_mode(uc_data, EXT_MODE_OTG_5_0V);
		if (ret < 0) {
			pr_debug("%s: cannot change extmode ret:%d\n",
				 __func__, ret);
			return ret;
		}
		break;
	case GSU_MODE_POGO_VOUT:
	case GSU_MODE_USB_CHG_POGO_VOUT:
		if (from_uc == GSU_MODE_USB_OTG_POGO_VOUT) {
			ret = max77759_chgr_mode_write(uc_data->client,
						      MAX77759_CHGR_MODE_ALL_OFF);
		} else {
			ret = gs101_pogo_vout_enable(uc_data, true);
		}

		break;
	default:
		break;
	}

	return ret;
}
EXPORT_SYMBOL_GPL(gs101_to_usecase);

static int max77759_otg_ilim_ma_to_code(u8 *code, int otg_ilim)
{
	if (otg_ilim == 0)
		*code = 0;
	else if (otg_ilim >= 500 && otg_ilim <= 1500)
		*code = 1 + (otg_ilim - 500) / 100;
	else
		return -EINVAL;

	return 0;
}

int max77759_otg_vbyp_mv_to_code(u8 *code, int vbyp)
{
	if (vbyp >= 12000)
		*code = 0x8c;
	else if (vbyp >= 5000)
		*code = (vbyp - 5000) / 50;
	else
		return -EINVAL;

	return 0;
}
EXPORT_SYMBOL_GPL(max77759_otg_vbyp_mv_to_code);

#define GS101_OTG_ILIM_DEFAULT_MA	1500
#define GS101_OTG_VBYPASS_DEFAULT_MV	5100

/* lazy init on the switches */


static bool gs101_setup_usecases_done(struct max77759_usecase_data *uc_data)
{
	return (uc_data->cpout_en != -EPROBE_DEFER) &&
	       (uc_data->cpout_ctl != -EPROBE_DEFER) &&
	       (uc_data->wlc_vbus_en != -EPROBE_DEFER) &&
	       (uc_data->ext_bst_ctl != -EPROBE_DEFER) &&
	       (uc_data->bst_sel != -EPROBE_DEFER);

	/* TODO: handle platform specific differences..
	       uc_data->ls2_en != -EPROBE_DEFER &&
	       uc_data->lsw1_is_closed != -EPROBE_DEFER &&
	       uc_data->lsw1_is_open != -EPROBE_DEFER &&
	       uc_data->vin_is_valid != -EPROBE_DEFER &&
	       uc_data->cpout_ctl != -EPROBE_DEFER &&
	       uc_data->cpout_en != -EPROBE_DEFER &&
	       uc_data->cpout21_en != -EPROBE_DEFER
	       uc_data->bst_on != -EPROBE_DEFER &&
	       uc_data->bst_sel != -EPROBE_DEFER &&
	       uc_data->ext_bst_ctl != -EPROBE_DEFER;
	*/
}

static void gs101_setup_default_usecase(struct max77759_usecase_data *uc_data)
{
	int ret;

	uc_data->is_a1 = -1;

	uc_data->bst_on = -EPROBE_DEFER;
	uc_data->bst_sel = -EPROBE_DEFER;
	uc_data->ext_bst_ctl = -EPROBE_DEFER;
	uc_data->pogo_ovp_en = -EPROBE_DEFER;
	uc_data->pogo_vout_en = -EPROBE_DEFER;

	uc_data->ls1_en = -EPROBE_DEFER;
	uc_data->ls2_en = -EPROBE_DEFER;
	uc_data->sw_en = -EPROBE_DEFER;

	uc_data->vin_is_valid = -EPROBE_DEFER;
	uc_data->lsw1_is_closed = -EPROBE_DEFER;
	uc_data->lsw1_is_open = -EPROBE_DEFER;

	uc_data->otg_enable = -EPROBE_DEFER;

	uc_data->wlc_en = -EPROBE_DEFER;
	uc_data->wlc_vbus_en = -EPROBE_DEFER;
	uc_data->cpout_en = -EPROBE_DEFER;
	uc_data->wlc_spoof_gpio = -EPROBE_DEFER;
	uc_data->cpout_ctl = -EPROBE_DEFER;
	uc_data->cpout21_en = -EPROBE_DEFER;

	uc_data->ext_bst_mode = -EPROBE_DEFER;
	uc_data->dc_sw_gpio = -EPROBE_DEFER;

	uc_data->init_done = false;

	/* TODO: override in bootloader and remove */
	ret = max77759_otg_ilim_ma_to_code(&uc_data->otg_ilim,
					   GS101_OTG_ILIM_DEFAULT_MA);
	if (ret < 0)
		uc_data->otg_ilim = MAX77759_CHG_CNFG_05_OTG_ILIM_1500MA;
	ret = max77759_chgr_reg_read(uc_data->client, MAX77759_CHG_CNFG_05,
				    &uc_data->otg_orig);
	if (ret == 0) {
		uc_data->otg_orig &= MAX77759_CHG_CNFG_05_OTG_ILIM_MASK;
	} else {
		uc_data->otg_orig = uc_data->otg_ilim;
	}

	ret = max77759_otg_vbyp_mv_to_code(&uc_data->otg_vbyp,
					   GS101_OTG_VBYPASS_DEFAULT_MV);
	if (ret < 0)
		uc_data->otg_vbyp = MAX77759_CHG_CNFG_11_OTG_VBYP_5100MV;
}
bool gs101_setup_usecases(struct max77759_usecase_data *uc_data,
			  struct device_node *node)
{
	enum of_gpio_flags flags = 0;

	if (!node) {
		gs101_setup_default_usecase(uc_data);
		return false;
	}

	/* control external boost if present */
	if (uc_data->bst_on == -EPROBE_DEFER)
		uc_data->bst_on = of_get_named_gpio(node, "max77759,bst-on", 0);
	if (uc_data->bst_sel == -EPROBE_DEFER)
		uc_data->bst_sel = of_get_named_gpio(node, "max77759,bst-sel", 0);
	if (uc_data->ext_bst_ctl == -EPROBE_DEFER)
		uc_data->ext_bst_ctl = of_get_named_gpio(node, "max77759,extbst-ctl", 0);

	/* for enabling charging over pogo */
	if (uc_data->pogo_ovp_en == -EPROBE_DEFER) {
		uc_data->pogo_ovp_en = of_get_named_gpio_flags(node, "max77759,pogo-ovp-en", 0,
							       &flags);
		if (uc_data->pogo_ovp_en >= 0)
			uc_data->pogo_ovp_en_act_low = (flags & OF_GPIO_ACTIVE_LOW) ? 1 : 0;
	}
	if (uc_data->pogo_vout_en == -EPROBE_DEFER) {
		uc_data->pogo_vout_en = of_get_named_gpio(node, "max77759,pogo-vout-sw-en", 0);

		if (uc_data->pogo_vout_en >= 0)
			gpio_set_value_cansleep(uc_data->pogo_vout_en, 0);
	}

	/* NBC workaround */
	if (uc_data->vin_is_valid == -EPROBE_DEFER)
		uc_data->vin_is_valid = of_get_named_gpio(node, "max77759,vin-is_valid", 0);
	if (uc_data->lsw1_is_closed == -EPROBE_DEFER)
		uc_data->lsw1_is_closed = of_get_named_gpio(node, "max77759,lsw1-is_closed", 0);
	if (uc_data->lsw1_is_open == -EPROBE_DEFER)
		uc_data->lsw1_is_open = of_get_named_gpio(node, "max77759,lsw1-is_open", 0);

	/* all OTG cases, change INOVLO */
	if (uc_data->otg_enable == -EPROBE_DEFER)
		uc_data->otg_enable = of_get_named_gpio(node, "max77759,otg-enable", 0);

	/*  wlc_rx: disable when chgin, CPOUT is safe */
	if (uc_data->wlc_en == -EPROBE_DEFER)
		uc_data->wlc_en = of_get_named_gpio(node, "max77759,wlc-en", 0);
	if (uc_data->wlc_vbus_en == -EPROBE_DEFER)
		uc_data->wlc_vbus_en = of_get_named_gpio(node, "max77759,wlc-vbus_en", 0);
	/*  wlc_rx -> wlc_rx+otg disable cpout */
	if (uc_data->cpout_en == -EPROBE_DEFER)
		uc_data->cpout_en = of_get_named_gpio(node, "max77759,cpout-en", 0);
	/*  wlc_rx thermal throttle -> spoof online */
	if (uc_data->wlc_spoof_gpio == -EPROBE_DEFER)
	    uc_data->wlc_spoof_gpio = of_get_named_gpio(node, "max77759,wlc-spoof", 0);
	/* to 5.2V in p9412 */
	if (uc_data->cpout_ctl == -EPROBE_DEFER)
		uc_data->cpout_ctl = of_get_named_gpio(node, "max77759,cpout-ctl", 0);
	/* ->wlc_tx disable 2:1 cpout */
	if (uc_data->cpout21_en == -EPROBE_DEFER)
		uc_data->cpout21_en = of_get_named_gpio(node, "max77759,cpout_21-en", 0);

	if (uc_data->ls1_en == -EPROBE_DEFER)
		uc_data->ls1_en = of_get_named_gpio(node, "max77759,ls1-en", 0);
	if (uc_data->ls2_en == -EPROBE_DEFER)
		uc_data->ls2_en = of_get_named_gpio(node, "max77759,ls2-en", 0);
	/* OTG+RTXL: IN-OUT switch of AO37 (forced always) */
	if (uc_data->sw_en == -EPROBE_DEFER)
		uc_data->sw_en = of_get_named_gpio(node, "max77759,sw-en", 0);
	/* OPTIONAL: only in P1.1+ (TPS61372) */
	if (uc_data->ext_bst_mode == -EPROBE_DEFER)
		uc_data->ext_bst_mode = of_get_named_gpio(node, "max77759,extbst-mode", 0);

	/* OPTIONAL: support wlc_rx -> wlc_rx+otg */
	uc_data->rx_otg_en = of_property_read_bool(node, "max77759,rx-to-rx-otg-en");
	/* OPTIONAL: support external boost OTG only */
	uc_data->ext_otg_only = of_property_read_bool(node, "max77759,ext-otg-only");
	/* OPTIONAL: use bst_on first on/off sequence */
	uc_data->wlctx_bst_en_first = of_property_read_bool(node, "max77759,bst-lsw-sequence");

	/* OPTIONAL: only extbst-enable when wlc+otg */
	uc_data->wlc_otg_extbst_en = of_property_read_bool(node, "max77759,wlc-otg-extbst-en");

	if (uc_data->dc_sw_gpio == -EPROBE_DEFER)
		uc_data->dc_sw_gpio = of_get_named_gpio(node, "max77759,gpio_dc_switch", 0);

	return gs101_setup_usecases_done(uc_data);
}
EXPORT_SYMBOL_GPL(gs101_setup_usecases);

void gs101_dump_usecasase_config(struct max77759_usecase_data *uc_data)
{
	pr_info("bst_on:%d, bst_sel:%d, ext_bst_ctl:%d\n",
		 uc_data->bst_on, uc_data->bst_sel, uc_data->ext_bst_ctl);
	pr_info("vin_valid:%d lsw1_o:%d lsw1_c:%d\n", uc_data->vin_is_valid,
		 uc_data->lsw1_is_open, uc_data->lsw1_is_closed);
	pr_info("wlc_en:%d wlc_vbus_en:%d cpout_en:%d cpout_ctl:%d cpout21_en=%d\n",
		uc_data->wlc_en, uc_data->wlc_vbus_en,
		uc_data->cpout_en, uc_data->cpout_ctl, uc_data->cpout21_en);
	pr_info("ls2_en:%d sw_en:%d ext_bst_mode:%d dc_sw_en:%d\n",
		uc_data->ls2_en, uc_data->sw_en, uc_data->ext_bst_mode, uc_data->dc_sw_gpio);
	pr_info("rx_to_rx_otg:%d ext_otg_only:%d\n",
		uc_data->rx_otg_en, uc_data->ext_otg_only);
}
EXPORT_SYMBOL_GPL(gs101_dump_usecasase_config);

