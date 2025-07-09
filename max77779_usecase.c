/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2023 Google, LLC
 *
 */


#include <linux/kernel.h>
#include <linux/printk.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/regmap.h>
#include "google_bms.h"
#include "max77779.h"
#include "max77779_charger.h"

static int gs201_otg_enable(struct max77779_usecase_data *uc_data, bool enable);
/* ----------------------------------------------------------------------- */

int gs201_wlc_en(struct max77779_usecase_data *uc_data, enum wlc_state_t state)
{
	const int wlc_on = state == WLC_ENABLED;
	int ret;

	pr_debug("%s: wlc_en=%d wlc_on=%d wlc_state=%d\n", __func__,
		 uc_data->wlc_en, wlc_on, state);

	if (uc_data->wlc_en < 0)
		return 0;

	if (state == WLC_SPOOFED && uc_data->wlc_spoof_vbyp > 0) {
		ret = max77779_external_chg_reg_write(uc_data->dev, MAX77779_CHG_CNFG_11,
					     uc_data->wlc_spoof_vbyp);
		pr_debug("%s: MAX77779_CHG_CNFG_11 write to %02x (ret = %d)\n",
			 __func__, uc_data->wlc_spoof_vbyp, ret);
	}

	gpio_set_value_cansleep(uc_data->wlc_spoof_gpio,
				state == WLC_SPOOFED && uc_data->wlc_spoof_gpio);
	gpio_set_value_cansleep(uc_data->wlc_en, wlc_on);

	return 0;
}

/* RTX reverse wireless charging */
static int gs201_wlc_tx_enable(struct max77779_usecase_data *uc_data, int use_case,
			       bool enable)
{
	int ret = 0;

	pr_debug("%s: use_case:%d enable:%d\n", __func__, use_case, enable);

	if (!enable) {
		ret = max77779_external_chg_reg_write(uc_data->dev, MAX77779_CHG_CNFG_11, 0x0);
		if (ret < 0)
			pr_err("%s: fail to reset MAX77779_CHG_REVERSE_BOOST_VOUT\n",
				__func__);

		ret = gs201_wlc_en(uc_data, WLC_DISABLED);
		if (ret < 0)
			pr_err("%s: cannot disable WLC (%d)\n", __func__, ret);

		return ret;
	}

	ret = gs201_wlc_en(uc_data, WLC_ENABLED);
	if (ret < 0)
		pr_err("%s: cannot enable WLC (%d)\n", __func__, ret);

	if (uc_data->rtx_ready >= 0)
		gpio_set_value_cansleep(uc_data->rtx_ready, 1);

	return ret;
}
static int gs201_wlc_tx_config(struct max77779_usecase_data *uc_data, int use_case)
{
	u8 val;
	int ret = 0;

	/* We need to configure max77779 */
	if (use_case == GSU_MODE_WLC_TX) {
		ret = max77779_external_chg_reg_write(uc_data->dev,
							MAX77779_CHG_CNFG_11,
							MAX77779_CHG_REVERSE_BOOST_VOUT_7V);
		if (ret < 0)
			pr_err("fail to configure MAX77779_CHG_REVERSE_BOOST_VOUT\n");
	} else {
		ret = max77779_external_chg_reg_write(uc_data->dev,
							MAX77779_CHG_CNFG_11,
							0x0);
		if (ret < 0)
			pr_err("fail to reset MAX77779_CHG_REVERSE_BOOST_VOUT\n");
	}
	/* Set WCSM to 1.4A */
	ret = max77779_external_chg_reg_read(uc_data->dev, MAX77779_CHG_CNFG_05, &val);
	if (ret < 0)
		pr_err("%s: fail to read MAX77779_CHG_CNFG_05 ret:%d\n", __func__, ret);

	ret = max77779_external_chg_reg_write(uc_data->dev, MAX77779_CHG_CNFG_05,
		_max77779_chg_cnfg_05_wcsm_ilim_set(val,
				MAX77779_CHG_CNFG_05_WCSM_ILIM_1400_MA));
	if (ret < 0) {
		pr_err("%s: fail to write MAX77779_CHG_CNFG_05 ret:%d\n", __func__, ret);
		return ret;
	}

	return ret;
}

static int gs201_otg_update_ilim(struct max77779_usecase_data *uc_data, int enable)
{
	u8 ilim;

	if (uc_data->otg_orig == uc_data->otg_ilim)
		return 0;

	if (enable) {
		int rc;

		rc = max77779_external_chg_reg_read(uc_data->dev, MAX77779_CHG_CNFG_05,
					   	    &uc_data->otg_orig);
		if (rc < 0) {
			pr_err("%s: cannot read otg_ilim (%d), use default\n",
			       __func__, rc);
			uc_data->otg_orig = MAX77779_CHG_CNFG_05_OTG_ILIM_1500MA;
		} else {
			uc_data->otg_orig &= MAX77779_CHG_CNFG_05_OTG_ILIM_MASK;
		}

		ilim = uc_data->otg_ilim;
	} else {
		ilim = uc_data->otg_orig;
	}

	return max77779_external_chg_reg_update(uc_data->dev, MAX77779_CHG_CNFG_05,
						MAX77779_CHG_CNFG_05_OTG_ILIM_MASK,
						ilim);
}

static int gs201_pogo_vout_enable(struct max77779_usecase_data *uc_data,
				  bool enable, bool otg)
{
	pr_debug("%s: enable: %d, otg: %d\n", __func__, enable, otg);

	if (enable && otg) {
		if (uc_data->pogo_vout_en >= 0)
			gpio_set_value_cansleep(uc_data->pogo_vout_en, 0);

		if (uc_data->ext_bst_ctl >= 0)
			gpio_set_value_cansleep(uc_data->ext_bst_ctl, 1);

		return 0;
	}
	if (!otg && uc_data->ext_bst_ctl >= 0)
		gpio_set_value_cansleep(uc_data->ext_bst_ctl, 0);

	if (enable) {
		if (uc_data->bst_on >= 0)
			gpio_set_value_cansleep(uc_data->bst_on, true);

		if (uc_data->pogo_vout_en >= 0)
			gpio_set_value_cansleep(uc_data->pogo_vout_en, true);
	} else {
		if (uc_data->pogo_vout_en >= 0)
			gpio_set_value_cansleep(uc_data->pogo_vout_en, false);

		if (uc_data->bst_on >= 0)
			gpio_set_value_cansleep(uc_data->bst_on, false);
	}

	return 0;
}

/*
 * Transition to standby (if needed) at the beginning of the sequences
 * @return <0 on error, 0 on success. ->use_case becomes GSU_MODE_STANDBY
 * if the transition is necessary (and successful).
 */
int gs201_to_standby(struct max77779_usecase_data *uc_data, int use_case)
{
	const int from_uc = uc_data->use_case;
	bool need_stby = false;
	bool from_otg = false;
	int ret;

	switch (from_uc) {
	case GSU_MODE_USB_CHG:
		if (use_case == GSU_MODE_USB_OTG) {
			need_stby = uc_data->ext_bst_ctl >= 0;
			break;
		}

		need_stby = use_case != GSU_MODE_DOCK &&
			    use_case != GSU_MODE_USB_DC &&
			    use_case != GSU_MODE_USB_OTG_FRS &&
			    use_case != GSU_MODE_USB_CHG_POGO_VOUT;
		break;
	case GSU_MODE_WLC_RX:
		/* HPP supported by device handled by wlc driver */
		need_stby = use_case != GSU_MODE_USB_OTG_WLC_RX &&
			    use_case != GSU_MODE_WLC_DC;
		break;
	case GSU_MODE_WLC_TX:
		need_stby = true;
		if (uc_data->rtx_ready >= 0)
			gpio_set_value_cansleep(uc_data->rtx_ready, 0);
		break;
	case GSU_MODE_USB_OTG:
		from_otg = true;
		if (use_case == GSU_MODE_USB_OTG_WLC_RX)
			break;
		if (use_case == GSU_MODE_USB_OTG_POGO_VOUT)
			break;

		gs201_otg_enable(uc_data, false);

		need_stby = true;
		break;
	case GSU_MODE_USB_OTG_FRS:
		from_otg = true;
		if (use_case == GSU_MODE_USB_OTG_WLC_RX) {
			need_stby = uc_data->ext_bst_ctl >= 0;
			break;
		}

		need_stby = use_case != GSU_MODE_USB_CHG;
		break;
	case GSU_MODE_USB_OTG_WLC_RX:
		from_otg = true;
		if (use_case == GSU_MODE_USB_OTG_FRS) {
			need_stby = uc_data->ext_bst_ctl >= 0;
			break;
		}

		need_stby = use_case != GSU_MODE_WLC_RX &&
			    use_case != GSU_MODE_DOCK &&
			    use_case != GSU_MODE_USB_OTG;
		break;
	case GSU_MODE_USB_DC:
		need_stby = use_case != GSU_MODE_USB_CHG;
		break;
	case GSU_MODE_WLC_DC:
		if (!uc_data->reverse12_en)
			return -EINVAL;
		need_stby = use_case != GSU_MODE_WLC_DC;
		break;
	case GSU_RAW_MODE:
		need_stby = true;
		break;
	case GSU_MODE_USB_OTG_POGO_VOUT:
		from_otg = true;
		need_stby = use_case != GSU_MODE_POGO_VOUT &&
			    use_case != GSU_MODE_USB_OTG;
		break;
	case GSU_MODE_POGO_VOUT:
		need_stby = use_case != GSU_MODE_USB_CHG_POGO_VOUT &&
			    use_case != GSU_MODE_USB_OTG_POGO_VOUT;
		break;
	case GSU_MODE_USB_CHG_POGO_VOUT:
		if (use_case == GSU_MODE_USB_CHG) {
			ret = gs201_pogo_vout_enable(uc_data, false, false);
			if (ret < 0)
				pr_err("%s: cannot tun off pogo_vout (%d)\n", __func__, ret);
			break;
		}
		need_stby = use_case != GSU_MODE_POGO_VOUT;
		break;
	case GSU_MODE_STANDBY:
	case GSU_MODE_STANDBY_BUCK_ON:
	default:
		need_stby = false;
		break;
	}

	if (use_case == GSU_RAW_MODE)
		need_stby = true;
	else if (use_case == from_uc)
		need_stby = false;

	pr_info("%s: use_case=%d->%d from_otg=%d need_stby=%d\n", __func__,
		from_uc, use_case, from_otg, need_stby);

	if (!need_stby)
		return 0;

	/* from POGO_VOUT to STBY */
	if (from_uc == GSU_MODE_POGO_VOUT ||
	    from_uc == GSU_MODE_USB_OTG_POGO_VOUT ||
	    from_uc == GSU_MODE_USB_CHG_POGO_VOUT) {
		ret = gs201_pogo_vout_enable(uc_data, false, false);
		if (ret < 0)
			pr_err("%s: cannot tun off pogo_vout (%d)\n", __func__, ret);
	}

	/* transition to STBY (might need to be up) */
	ret = max77779_external_chg_mode_write(uc_data->dev, MAX77779_CHGR_MODE_ALL_OFF);
	if (ret < 0)
		return -EIO;

	if (uc_data->rtx_available >= 0)
		gpio_set_value_cansleep(uc_data->rtx_available, 1);

	uc_data->use_case = GSU_MODE_STANDBY;
	return ret;
}

/* enable/disable soft-start */
static int gs201_ramp_bypass(struct max77779_usecase_data *uc_data, bool enable)
{
	const u8 value = enable ? MAX77779_CHG_CNFG_00_BYPV_RAMP_BYPASS_MASK : 0;

	return max77779_external_chg_reg_update(uc_data->dev, MAX77779_CHG_CNFG_00,
						MAX77779_CHG_CNFG_00_BYPV_RAMP_BYPASS_MASK,
						value);
}

/* cleanup from every usecase */
int gs201_force_standby(struct max77779_usecase_data *uc_data)
{
	const u8 insel_mask = MAX77779_CHG_CNFG_12_CHGINSEL_MASK |
			      MAX77779_CHG_CNFG_12_WCINSEL_MASK;
	const u8 insel_value = MAX77779_CHG_CNFG_12_CHGINSEL |
			       MAX77779_CHG_CNFG_12_WCINSEL;
	int ret;

	pr_debug("%s: recovery\n", __func__);

	ret = gs201_ramp_bypass(uc_data, false);
	if (ret < 0)
		pr_err("%s: cannot reset ramp_bypass (%d)\n",
			__func__, ret);

	ret = gs201_pogo_vout_enable(uc_data, false, false);
	if (ret < 0)
		pr_err("%s: cannot tun off pogo_vout (%d)\n", __func__, ret);

	ret = max77779_external_chg_mode_write(uc_data->dev, MAX77779_CHGR_MODE_ALL_OFF);
	if (ret < 0)
		pr_err("%s: cannot reset mode register (%d)\n",
			__func__, ret);

	ret = max77779_external_chg_insel_write(uc_data->dev, insel_mask, insel_value);
	if (ret < 0)
		pr_err("%s: cannot reset insel (%d)\n",
			__func__, ret);

	gs201_otg_enable(uc_data, false);

	if (uc_data->rtx_ready >= 0)
		gpio_set_value_cansleep(uc_data->rtx_ready, 0);

	return 0;
}

static int gs201_otg_mode(struct max77779_usecase_data *uc_data, int to)
{
	int ret = -EINVAL;

	pr_debug("%s: to=%d\n", __func__, to);

	if (to == GSU_MODE_USB_OTG) {
		ret = max77779_external_chg_mode_write(uc_data->dev,
						       MAX77779_CHGR_MODE_ALL_OFF);
	}

	return ret;
}

/*
 * This must follow different paths depending on the platforms.
 *
 * NOTE: the USB stack expects VBUS to be on after voting for the usecase.
 */
static int gs201_otg_enable_frs(struct max77779_usecase_data *uc_data)
{
	int ret;

	ret = gs201_otg_update_ilim(uc_data, true);
	if (ret < 0) {
		pr_debug("%s: cannot update otg ilim ret:%d\n", __func__, ret);
		return ret;
	}

	/* the code default to write to the MODE register */

	ret = max77779_external_chg_mode_write(uc_data->dev,
					       MAX77779_CHGR_MODE_OTG_BOOST_ON);
	if (ret < 0) {
		pr_debug("%s: cannot set CNFG_00 to 0xa ret:%d\n", __func__, ret);
		return ret;
	}

	return ret;
}

static int gs201_otg_enable(struct max77779_usecase_data *uc_data, bool enable)
{
	pr_debug("%s: enable:%d\n", __func__, enable);

	if (enable) {
		if (uc_data->bst_on >= 0)
			gpio_set_value_cansleep(uc_data->bst_on, 1);

		usleep_range(5 * USEC_PER_MSEC, 5 * USEC_PER_MSEC + 100);

		if (uc_data->ext_bst_ctl >= 0)
			gpio_set_value_cansleep(uc_data->ext_bst_ctl, 1);

	} else {
		if (uc_data->ext_bst_ctl >= 0)
			gpio_set_value_cansleep(uc_data->ext_bst_ctl, 0);

		usleep_range(5 * USEC_PER_MSEC, 5 * USEC_PER_MSEC + 100);

		if (uc_data->bst_on >= 0)
			gpio_set_value_cansleep(uc_data->bst_on, 0);
	}

	return 0;
}

/*
 * Case	USB_chg USB_otg	WLC_chg	WLC_TX	PMIC_Charger	Name
 * -------------------------------------------------------------------------------------
 * 7	0	1	1	0	IF-PMIC-WCIN	USB_OTG_WLC_RX
 * 9	0	1	0	0	0		USB_OTG / USB_OTG_FRS
 * -------------------------------------------------------------------------------------
 * WLC_chg = 0 off, 1 = on, 2 = PPS
 *
 * NOTE: do not call with (cb_data->wlc_rx && cb_data->wlc_tx)
 */

static int gs201_standby_to_otg(struct max77779_usecase_data *uc_data, int use_case)
{
	int ret;

	ret = gs201_otg_enable(uc_data, true);

	if (ret == 0)
		usleep_range(5 * USEC_PER_MSEC, 5 * USEC_PER_MSEC + 100);
	/*
	 * Assumption: gs201_to_usecase() will write back cached values to
	 * CHG_CNFG_00.Mode. At the moment, the cached value at
	 * max77779_mode_callback is 0. If the cached value changes to something
	 * other than 0, then, the code has to be revisited.
	 */

	return ret;
}

/* was b/179816224 WLC_RX -> WLC_RX + OTG (Transition #10) */
static int gs201_wlcrx_to_wlcrx_otg(struct max77779_usecase_data *uc_data)
{
	pr_warn("%s: disabled\n", __func__);
	return 0;
}

static int gs201_to_otg_usecase(struct max77779_usecase_data *uc_data, int use_case)
{
	const int from_uc = uc_data->use_case;
	int ret = 0;

	switch (from_uc) {
	/* 9: stby to USB OTG */
	/* 10: stby to USB_OTG_FRS */
	case GSU_MODE_STANDBY:
		ret = gs201_standby_to_otg(uc_data, use_case);
		if (ret < 0) {
			pr_err("%s: cannot enable OTG ret:%d\n",  __func__, ret);
			return ret;
		}
	break;

	case GSU_MODE_USB_CHG:
		/* need to go through stby out of this */
		if (use_case != GSU_MODE_USB_OTG && use_case != GSU_MODE_USB_OTG_FRS)
			return -EINVAL;
		else if (use_case == GSU_MODE_USB_OTG)
			ret = gs201_otg_enable(uc_data, true);
		else
			ret = gs201_otg_enable_frs(uc_data);
	break;


	case GSU_MODE_WLC_TX:
	break;

	case GSU_MODE_WLC_RX:
	case GSU_MODE_DOCK:
		if (use_case == GSU_MODE_USB_OTG_WLC_RX) {
			if (uc_data->rx_otg_en) {
				ret = gs201_standby_to_otg(uc_data, use_case);
			} else {
				ret = gs201_wlcrx_to_wlcrx_otg(uc_data);
			}
		}
	break;

	case GSU_MODE_USB_OTG:
		/* b/179816224: OTG -> WLC_RX + OTG */
		if (use_case == GSU_MODE_USB_OTG_WLC_RX) {
			/* pvp TODO: Is it just WLC Rx enable? */
		}
	break;
	case GSU_MODE_USB_OTG_WLC_RX:
		if (use_case == GSU_MODE_USB_OTG_FRS)
			return -EINVAL;
	break;
	case GSU_MODE_USB_OTG_FRS:
		if (use_case == GSU_MODE_USB_OTG_WLC_RX)
			return -EINVAL;
	break;
	case GSU_MODE_POGO_VOUT:
		if (use_case == GSU_MODE_USB_OTG_POGO_VOUT) {
			ret = max77779_external_chg_reg_write(uc_data->dev,
							      MAX77779_CHG_CNFG_00,
							      MAX77779_CHGR_MODE_BOOST_UNO_ON);

			msleep(40);
			ret = gs201_pogo_vout_enable(uc_data, true, true);
		}
	break;
	case GSU_MODE_USB_OTG_POGO_VOUT:
	break;
	default:
		return -ENOTSUPP;
	}

	return ret;
}

/* handles the transition data->use_case ==> use_case */
int gs201_to_usecase(struct max77779_usecase_data *uc_data, int use_case)
{
	const int from_uc = uc_data->use_case;
	bool rtx_avail = false;
	int ret = 0;

	switch (use_case) {
	case GSU_MODE_USB_OTG:
	case GSU_MODE_USB_OTG_FRS:
	case GSU_MODE_USB_OTG_WLC_RX:
	case GSU_MODE_USB_OTG_POGO_VOUT:
		ret = gs201_to_otg_usecase(uc_data, use_case);
		break;
	case GSU_MODE_WLC_TX:
		rtx_avail = true;
		ret = gs201_wlc_tx_config(uc_data, use_case);
		break;
	case GSU_MODE_WLC_RX:
	case GSU_MODE_DOCK:
		if (from_uc == GSU_MODE_USB_OTG_WLC_RX) {
			if (uc_data->ext_otg_only)
				ret = gs201_otg_enable(uc_data, false);
			else
				ret = gs201_otg_mode(uc_data, GSU_MODE_USB_OTG);
		}
		break;
	case GSU_MODE_USB_CHG:
	case GSU_MODE_USB_DC:
		rtx_avail = false;
		break;
	case GSU_MODE_STANDBY_BUCK_ON:
		break;
	case GSU_MODE_STANDBY:
	case GSU_RAW_MODE:
		/* just write the value to the register (it's in stby) */
		rtx_avail = true;
		break;
	case GSU_MODE_USB_WLC_RX:
	case GSU_MODE_WLC_DC:
		break;
	case GSU_MODE_POGO_VOUT:
		ret = gs201_pogo_vout_enable(uc_data, true, false);

		/* wait for ext boost ready */
		if (ret == 0 && from_uc == GSU_MODE_USB_OTG_POGO_VOUT)
			msleep(4);
		break;
	case GSU_MODE_USB_CHG_POGO_VOUT:
		ret = gs201_pogo_vout_enable(uc_data, true, false);
		break;
	default:
		break;
	}

	if (uc_data->rtx_available >= 0)
		gpio_set_value_cansleep(uc_data->rtx_available, rtx_avail);

	return ret;
}

/* finish usecase configuration after max77779 mode register is set */
int gs201_finish_usecase(struct max77779_usecase_data *uc_data, int use_case)
{
	const int from_uc = uc_data->use_case;
	int ret = 0;

	switch (use_case) {
	case GSU_MODE_WLC_TX:
		/* p9412 will not be in RX when powered from EXT */
		ret = gs201_wlc_tx_enable(uc_data, use_case, true);
		if (ret < 0)
			return ret;
		break;
	default:
		if (from_uc == GSU_MODE_WLC_TX) {
			/* p9412 is already off from insel */
			ret = gs201_wlc_tx_enable(uc_data, use_case, false);
			if (ret < 0)
				return ret;

			ret = gs201_wlc_en(uc_data, WLC_ENABLED); /* re-enable wlc in case of rx */
			if (ret < 0)
				return ret;
		}
		break;
	}

	return ret;
}

static int max77779_otg_ilim_ma_to_code(u8 *code, int otg_ilim)
{
	if (otg_ilim == 0)
		*code = 0;
	else if (otg_ilim >= 500 && otg_ilim <= 1500)
		*code = 1 + (otg_ilim - 500) / 100;
	else
		return -EINVAL;

	return 0;
}

int max77779_otg_vbyp_mv_to_code(u8 *code, int vbyp)
{
	if (vbyp >= 12000)
		*code = 0x8c;
	else if (vbyp >= 5000)
		*code = (vbyp - 5000) / 50;
	else
		return -EINVAL;

	return 0;
}

#define GS201_OTG_ILIM_DEFAULT_MA	1500
#define GS201_OTG_VBYPASS_DEFAULT_MV	5100

/* lazy init on the switches */


static bool gs201_setup_usecases_done(struct max77779_usecase_data *uc_data)
{
	return (uc_data->wlc_en != -EPROBE_DEFER) &&
	       (uc_data->bst_on != -EPROBE_DEFER) &&
	       (uc_data->ext_bst_mode != -EPROBE_DEFER) &&
	       (uc_data->ext_bst_ctl != -EPROBE_DEFER) &&
	       (uc_data->rtx_ready != -EPROBE_DEFER) &&
	       (uc_data->wlc_spoof_gpio != -EPROBE_DEFER) &&
	       (uc_data->rtx_available != -EPROBE_DEFER);

	/* TODO: handle platform specific differences.. */
}

static void gs201_setup_default_usecase(struct max77779_usecase_data *uc_data)
{
	int ret;

	/* external boost */
	uc_data->bst_on = -EPROBE_DEFER;
	uc_data->ext_bst_ctl = -EPROBE_DEFER;
	uc_data->ext_bst_mode = -EPROBE_DEFER;
	uc_data->pogo_vout_en = -EPROBE_DEFER;

	uc_data->otg_enable = -EPROBE_DEFER;

	uc_data->wlc_en = -EPROBE_DEFER;
	uc_data->rtx_ready = -EPROBE_DEFER;
	uc_data->rtx_available = -EPROBE_DEFER;

	uc_data->wlc_spoof_gpio = -EPROBE_DEFER;

	uc_data->wlc_spoof_vbyp = 0;
	uc_data->init_done = false;

	/* TODO: override in bootloader and remove */
	ret = max77779_otg_ilim_ma_to_code(&uc_data->otg_ilim,
					   GS201_OTG_ILIM_DEFAULT_MA);
	if (ret < 0)
		uc_data->otg_ilim = MAX77779_CHG_CNFG_05_OTG_ILIM_1500MA;
	ret = max77779_external_chg_reg_read(uc_data->dev, MAX77779_CHG_CNFG_05,
					     &uc_data->otg_orig);
	if (ret == 0) {
		uc_data->otg_orig &= MAX77779_CHG_CNFG_05_OTG_ILIM_MASK;
	} else {
		uc_data->otg_orig = uc_data->otg_ilim;
	}

	ret = max77779_otg_vbyp_mv_to_code(&uc_data->otg_vbyp,
					   GS201_OTG_VBYPASS_DEFAULT_MV);
	if (ret < 0)
		uc_data->otg_vbyp = MAX77779_CHG_CNFG_11_OTG_VBYP_5100MV;
}

bool gs201_setup_usecases(struct max77779_usecase_data *uc_data,
			  struct device_node *node)
{
	u32 data;
	int ret;

	if (!node) {
		gs201_setup_default_usecase(uc_data);
		return false;
	}

	/* control external boost if present */
	if (uc_data->bst_on == -EPROBE_DEFER) {
		uc_data->bst_on = of_get_named_gpio(node, "max77779,bst-on", 0);
		if (uc_data->bst_on >= 0)
			gpio_direction_output(uc_data->bst_on, 0);
	}
	if (uc_data->ext_bst_ctl == -EPROBE_DEFER)
		uc_data->ext_bst_ctl = of_get_named_gpio(node, "max77779,extbst-ctl", 0);
	if (uc_data->ext_bst_mode == -EPROBE_DEFER) {
		uc_data->ext_bst_mode = of_get_named_gpio(node, "max77779,extbst-mode", 0);
		if (uc_data->ext_bst_mode >= 0)
			gpio_set_value_cansleep(uc_data->ext_bst_mode, 0);
	}

	/*  wlc_rx: disable when chgin, CPOUT is safe */
	if (uc_data->wlc_en == -EPROBE_DEFER)
		uc_data->wlc_en = of_get_named_gpio(node, "max77779,wlc-en", 0);

	/*  wlc_rx thermal throttle -> spoof online */
	if (uc_data->wlc_spoof_gpio == -EPROBE_DEFER)
		uc_data->wlc_spoof_gpio = of_get_named_gpio(node, "max77779,wlc-spoof", 0);

	/* OPTIONAL: wlc-spoof-vol */
	ret = of_property_read_u32(node, "max77779,wlc-spoof-vbyp", &data);
	if (ret < 0)
		uc_data->wlc_spoof_vbyp = 0;
	else
		uc_data->wlc_spoof_vbyp = data;

	/* OPTIONAL: support wlc_rx -> wlc_rx+otg */
	uc_data->rx_otg_en = of_property_read_bool(node, "max77779,rx-to-rx-otg-en");

	/* OPTIONAL: support external boost OTG only */
	uc_data->ext_otg_only = of_property_read_bool(node, "max77779,ext-otg-only");

	/* OPTIONAL: support reverse 1:2 mode for RTx */
	uc_data->reverse12_en = of_property_read_bool(node, "max77779,reverse_12-en");

	if (uc_data->rtx_ready == -EPROBE_DEFER)
		uc_data->rtx_ready = of_get_named_gpio(node, "max77779,rtx-ready", 0);

	if (uc_data->rtx_available == -EPROBE_DEFER)
		uc_data->rtx_available = of_get_named_gpio(node, "max77779,rtx-available", 0);

	if (uc_data->pogo_vout_en == -EPROBE_DEFER) {
		uc_data->pogo_vout_en = of_get_named_gpio(node, "max77779,pogo-vout-sw-en", 0);

		if (uc_data->pogo_vout_en >= 0)
			gpio_direction_output(uc_data->pogo_vout_en, 0);
	}

	return gs201_setup_usecases_done(uc_data);
}

void gs201_dump_usecasase_config(struct max77779_usecase_data *uc_data)
{
	pr_info("bst_on:%d, ext_bst_ctl: %d, ext_bst_mode:%d, pogo_vout_en:%d\n",
		 uc_data->bst_on, uc_data->ext_bst_ctl, uc_data->ext_bst_mode,
		 uc_data->pogo_vout_en);
	pr_info("wlc_en:%d, reverse12_en:%d rtx_ready:%d\n",
		uc_data->wlc_en, uc_data->reverse12_en, uc_data->rtx_ready);
	pr_info("rtx_available:%d, rx_to_rx_otg:%d ext_otg_only:%d wlc_spoof_gpio:%d\n",
		uc_data->rtx_available, uc_data->rx_otg_en, uc_data->ext_otg_only,
		uc_data->wlc_spoof_gpio);
}

