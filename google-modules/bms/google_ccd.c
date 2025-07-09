/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2021 Google, LLC
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

#include <linux/debugfs.h>
#include <linux/kernel.h>
#include <linux/printk.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include "gbms_power_supply.h"
#include "google_bms.h"
#include "google_psy.h"

#define PD_VOLTAGE_MAX_MV	9000	/* 9V */
#define PD_CURRENT_MAX_UA	3000000	/* 3A */

#define GCCD_MAIN_CHARGE_CURRENT_MAX	4000000		/* 4A */
#define GCCD_BUCK_CHARGE_CURRENT_MAX	900000		/* 0.9A */
#define GCCD_BUCK_CHARGE_PWR_THRESHOLD	27000000	/* 27W */
#define GCCD_MAIN_CHGIN_ILIM		2200000		/* 2.2A */

struct gccd_drv {
	struct device *device;
	struct power_supply *psy;
	const char *main_chg_psy_name;
	const char *buck_chg_psy_name;
	struct power_supply *main_chg_psy;
	struct power_supply *buck_chg_psy;
	struct mutex gccd_lock;
	struct delayed_work init_work;
	struct gvotable_election *fcc_votable;
	struct gvotable_election *fv_votable;
	bool init_complete;
	int voltage_max;
	int current_max;
	int buck_chg_en;
	bool ftm_mode; /* factory test, force enable buck charger */

};

/* ------------------------------------------------------------------------- */

static int gccd_get_charge_current_max(struct gccd_drv *gccd);
static int gccd_set_charge_current_max(struct gccd_drv *gccd, int chg_current, bool pwr_changed);

/* this function is only for debug */
static int gccd_set_buck_active(struct gccd_drv *gccd, int enabled)
{
	int cc_max, ret;

	/* convert type to boolean */
	gccd->ftm_mode = !!enabled;
	cc_max = gccd_get_charge_current_max(gccd);
	pr_info("%s: charge_current=%d (0)\n", __func__, cc_max);
	ret = gccd_set_charge_current_max(gccd, cc_max, false);

	return ret;
}

static int debug_buck_active_get(void *data, u64 *val)
{
	struct gccd_drv *gccd = (struct gccd_drv *)data;

	*val = gccd->ftm_mode;
	return 0;
}

static int debug_buck_active_set(void *data, u64 val)
{
	struct gccd_drv *gccd = (struct gccd_drv *)data;
	int ret;

	if (val < 0 || val > 1)
		return -EINVAL;

	ret = gccd_set_buck_active(gccd, val);
	if (ret)
		pr_info("%s: Failed to set buck active: %d\n", __func__, ret);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_buck_active_fops, debug_buck_active_get,
			debug_buck_active_set, "%llu\n");

static int gccd_init_fs(struct gccd_drv *gccd)
{
	/* TODO: ... */
	return 0;
}

static int gccd_init_debugfs(struct gccd_drv *gccd)
{
	struct dentry *de = NULL;

	de = debugfs_create_dir("google_ccd", 0);
	if (IS_ERR_OR_NULL(de))
		return 0;

	/* buck_active */
	debugfs_create_file("buck_active", 0600, de, gccd,
			    &debug_buck_active_fops);

	return 0;
}

/* ------------------------------------------------------------------------- */

static bool gccd_get_chg_psy(struct gccd_drv *gccd)
{
	if (!gccd->main_chg_psy && gccd->main_chg_psy_name) {
		gccd->main_chg_psy = power_supply_get_by_name(gccd->main_chg_psy_name);
		if (!gccd->main_chg_psy)
			return false;
	}

	if (!gccd->buck_chg_psy && gccd->buck_chg_psy_name) {
		gccd->buck_chg_psy = power_supply_get_by_name(gccd->buck_chg_psy_name);
		if (!gccd->buck_chg_psy)
			return false;
	}

	return true;
}

static int gccd_has_chg_in(struct gccd_drv *gccd)
{
	int ret;

	if (!gccd_get_chg_psy(gccd))
		return -EINVAL;

	ret = PSY_GET_PROP(gccd->main_chg_psy, POWER_SUPPLY_PROP_PRESENT);
	if (ret < 0) {
		dev_err(gccd->device, "Error getting charging status: %d\n", ret);
		return -EINVAL;
	}

	return ret != 0;
}

static bool gccd_find_votable(struct gccd_drv *gccd)
{
	if (!gccd->fcc_votable) {
		gccd->fcc_votable = gvotable_election_get_handle("MSC_FCC");
		if (!gccd->fcc_votable) {
			dev_err(gccd->device, "Could not get votable: MSC_FCC\n");
			return false;
		}
	}

	if (!gccd->fv_votable) {
		gccd->fv_votable = gvotable_election_get_handle("MSC_FV");
		if (!gccd->fv_votable) {
			dev_err(gccd->device, "Could not get votable: MSC_FV\n");
			return false;
		}
	}

	return true;
}

static int gccd_get_charge_current_max(struct gccd_drv *gccd)
{
	int cc_max = -1;

	if (!gccd_find_votable(gccd))
		return cc_max;

	cc_max = gvotable_get_current_int_vote(gccd->fcc_votable);

	return cc_max;
}

static int gccd_get_charge_voltage_max(struct gccd_drv *gccd)
{
	int fv_uv = -1;

	if (!gccd_find_votable(gccd))
		return fv_uv;

	fv_uv = gvotable_get_current_int_vote(gccd->fv_votable);

	return fv_uv;
}

static int gccd_set_charge_current_max(struct gccd_drv *gccd,
				       int chg_current, bool pwr_changed)
{
	int main_chg_current = chg_current, buck_chg_current = 0;
	int watt = gccd->voltage_max * gccd->current_max;
	int fv_uv = gccd_get_charge_voltage_max(gccd);
	struct power_supply *main_psy = gccd->main_chg_psy;
	bool pwr_ok = false;
	int ret;

	if (gccd->ftm_mode) {
		/* set CHGIN_ILIM (CHG_CNFG_09) to 2200mA in ftm_mode */
		ret = PSY_SET_PROP(main_psy, POWER_SUPPLY_PROP_CURRENT_MAX,
				   GCCD_MAIN_CHGIN_ILIM);

		/* force enable buck_chg*/
		if (ret == 0)
			pwr_ok = true;

		goto set_current_max;
	}

	pwr_ok = watt >= GCCD_BUCK_CHARGE_PWR_THRESHOLD &&
		 main_chg_current > GCCD_MAIN_CHARGE_CURRENT_MAX;

	if (pwr_changed)
		pr_info("%s: pwr_ok=%d (%d, %d, %d)\n",
			__func__, pwr_ok, watt, main_chg_current, fv_uv);

	/* Don't enable buck charging */
	if (pwr_changed && !pwr_ok)
		return 0;

set_current_max:
	if (pwr_ok) {
		/*
		 * Sequoia has a solution for mechanical heat dissipation,
		 * set SQ: 4A, buck: (fcc - 4A)
		 */
		main_chg_current = GCCD_MAIN_CHARGE_CURRENT_MAX;
		buck_chg_current = (chg_current - GCCD_MAIN_CHARGE_CURRENT_MAX);
		if (buck_chg_current > GCCD_BUCK_CHARGE_CURRENT_MAX)
			buck_chg_current = GCCD_BUCK_CHARGE_CURRENT_MAX;
	}

	pr_info("%s: charge_current=%d, main=%d, buck=%d, v_max=%d, c_max=%d\n",
		__func__, chg_current, main_chg_current, buck_chg_current,
		gccd->voltage_max, gccd->current_max);

	ret = PSY_SET_PROP(main_psy, POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
			   main_chg_current);

	/*
	 * enable buck charging by pull charging gpio high active when
	 * buck_chg_current is non-zero
	 */
	if (ret == 0 && gccd->buck_chg_en >= 0) {
		struct power_supply *buck_psy = gccd->buck_chg_psy;
		int en = (buck_chg_current > 0);

		pr_info("%s: buck_charger enable=%d\n", __func__, en);

		ret = PSY_SET_PROP(buck_psy, POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
				   buck_chg_current);
		if (ret == 0)
			gpio_direction_output(gccd->buck_chg_en, en);
	}

	return ret;
}

/* ------------------------------------------------------------------------ */

static int gccd_gpio_init(struct device *dev, struct gccd_drv *gccd)
{
	int ret = 0;
	struct device_node *node = dev->of_node;

	/* BUCK_CHG_EN */
	ret = of_get_named_gpio(node, "google,buck_chg_en", 0);
	gccd->buck_chg_en = ret;
	if (ret < 0)
		dev_warn(dev, "unable to read google,buck_chg_en from dt: %d\n",
			 ret);
	else
		dev_info(dev, "BUCK_CHG_EN gpio:%d", gccd->buck_chg_en);

	return (ret < 0) ? ret : 0;
}

/* use the charger one when avalaible or fallback to the generated one */
static uint64_t gccd_get_charger_state(const struct gccd_drv *gccd,
				       struct power_supply *chg_psy)
{
	union gbms_charger_state chg_state;
	int rc;

	rc = gbms_read_charger_state(&chg_state, chg_psy);
	if (rc < 0)
		return 0;

	return chg_state.v;
}

static enum power_supply_property gccd_psy_properties[] = {
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,		/* compat */
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_STATUS,
};

static int gccd_psy_get_property(struct power_supply *psy,
				 enum power_supply_property psp,
				 union power_supply_propval *pval)
{
	struct gccd_drv *gccd = (struct gccd_drv *)power_supply_get_drvdata(psy);
	int ret = 0;

	if (!gccd->init_complete)
		return -EAGAIN;

	if (!gccd_get_chg_psy(gccd))
		return -EAGAIN;

	mutex_lock(&gccd->gccd_lock);

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		pval->intval = gccd_has_chg_in(gccd);
		if (pval->intval < 0)
			pval->intval = 0;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
	case POWER_SUPPLY_PROP_ONLINE:
	case POWER_SUPPLY_PROP_CURRENT_MAX:
	case POWER_SUPPLY_PROP_STATUS:
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		pval->intval = PSY_GET_INT_PROP(gccd->main_chg_psy, psp, &ret);
		break;
	default:
		pr_debug("%s: property (%d) unsupported.\n", __func__, psp);
		ret = -EINVAL;
		break;
	}

	mutex_unlock(&gccd->gccd_lock);

	return ret;
}

static int gccd_psy_set_property(struct power_supply *psy,
				 enum power_supply_property psp,
				 const union power_supply_propval *pval)
{
	struct gccd_drv *gccd = (struct gccd_drv *)power_supply_get_drvdata(psy);
	int ret = 0;
	int current_max, voltage_max;
	bool changed = false;

	if (!gccd->init_complete)
		return -EAGAIN;

	if (!gccd_get_chg_psy(gccd))
		return -EAGAIN;

	mutex_lock(&gccd->gccd_lock);

	switch (psp) {
	case POWER_SUPPLY_PROP_CURRENT_MAX: {
		bool is_adapter_9v3a = gccd->voltage_max == PD_VOLTAGE_MAX_MV &&
				       pval->intval == PD_CURRENT_MAX_UA;

		/* set CHGIN_ILIM (CHG_CNFG_09) to 2200mA for 9V/3A adapter */
		if (is_adapter_9v3a || gccd->ftm_mode)
			ret = PSY_SET_PROP(gccd->main_chg_psy, psp, GCCD_MAIN_CHGIN_ILIM);
		else
			ret = PSY_SET_PROP(gccd->main_chg_psy, psp, pval->intval);
		if (ret)
			break;

		current_max = pval->intval / 1000;
		if (gccd->current_max != current_max) {
			dev_dbg(gccd->device, "current_max: %d->%d\n", gccd->current_max, current_max);
			changed = true;
			gccd->current_max = current_max;
		}
		break;
	}
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		ret = PSY_SET_PROP(gccd->main_chg_psy, psp, pval->intval);
		if (ret)
			break;

		voltage_max = pval->intval / 1000;
		if (gccd->voltage_max != voltage_max) {
			dev_dbg(gccd->device, "voltage_max: %d->%d\n", gccd->voltage_max, voltage_max);
			changed = true;
			gccd->voltage_max = voltage_max;
		}
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		pr_debug("%s: charge_current=%d (0)\n", __func__, pval->intval);
		ret = gccd_set_charge_current_max(gccd, pval->intval, false);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		pr_debug("%s: charge_voltage=%d \n", __func__, pval->intval);
		ret = PSY_SET_PROP(gccd->main_chg_psy, psp, pval->intval);
		if (!ret)
			ret = PSY_SET_PROP(gccd->buck_chg_psy, psp, pval->intval);
		break;
	case POWER_SUPPLY_PROP_ONLINE:
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		ret = PSY_SET_PROP(gccd->main_chg_psy, psp, pval->intval);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	if (changed) {
		int cc_max;

		cc_max = gccd_get_charge_current_max(gccd);
		pr_info("%s: charge_current=%d (1)\n", __func__, cc_max);
		ret = gccd_set_charge_current_max(gccd, cc_max, true);
	}

	mutex_unlock(&gccd->gccd_lock);

	return ret;
}

static int gccd_psy_is_writeable(struct power_supply *psy,
				 enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
	case POWER_SUPPLY_PROP_VOLTAGE_MAX: /* compat, same the next */
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
	case POWER_SUPPLY_PROP_CURRENT_MAX:
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		return 1;
	default:
		break;
	}

	return 0;
}

static int gccd_gbms_psy_get_property(struct power_supply *psy,
				      enum gbms_property psp,
				      union gbms_propval *pval)
{
	struct gccd_drv *gccd = (struct gccd_drv *)power_supply_get_drvdata(psy);
	union gbms_charger_state chg_state;
	int ret = 0;

	if (!gccd->init_complete)
		return -EAGAIN;

	if (!gccd_get_chg_psy(gccd))
		return -EAGAIN;

	mutex_lock(&gccd->gccd_lock);

	switch (psp) {
	case GBMS_PROP_CHARGE_CHARGER_STATE:
		chg_state.v = gccd_get_charger_state(gccd, gccd->main_chg_psy);
		pval->int64val = chg_state.v;
		break;
	case GBMS_PROP_CHARGE_DISABLE:
	case GBMS_PROP_CHARGING_ENABLED:
	case GBMS_PROP_INPUT_CURRENT_LIMITED:
	case GBMS_PROP_TAPER_CONTROL:
		pval->prop.intval = GPSY_GET_INT_PROP(gccd->main_chg_psy, psp, &ret);
		break;
	default:
		pr_debug("%s: route to gccd_psy_get_property, psp:%d\n", __func__, psp);
		ret = -ENODATA;
		break;
	}

	mutex_unlock(&gccd->gccd_lock);

	return ret;
}

static int gccd_gbms_psy_set_property(struct power_supply *psy,
				      enum gbms_property psp,
				      const union gbms_propval *pval)
{
	struct gccd_drv *gccd = (struct gccd_drv *)power_supply_get_drvdata(psy);
	int ret = 0;

	if (!gccd->init_complete)
		return -EAGAIN;

	if (!gccd_get_chg_psy(gccd))
		return -EAGAIN;

	mutex_lock(&gccd->gccd_lock);

	switch (psp) {
	case GBMS_PROP_CHARGING_ENABLED:
	case GBMS_PROP_CHARGE_DISABLE:
		ret = GPSY_SET_PROP(gccd->main_chg_psy, psp, pval->prop.intval);
		break;
	case GBMS_PROP_TAPER_CONTROL:
		if (pval->prop.intval == GBMS_TAPER_CONTROL_ON)
			gpio_direction_output(gccd->buck_chg_en, 0);
		break;
	default:
		pr_debug("%s: route to gccd_psy_set_property, psp:%d\n", __func__, psp);
		ret = -ENODATA;
		break;
	}

	mutex_unlock(&gccd->gccd_lock);

	return ret;
}

static int gccd_gbms_psy_is_writeable(struct power_supply *psy,
				      enum gbms_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
	case POWER_SUPPLY_PROP_VOLTAGE_MAX: /* compat, same the next */
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
	case POWER_SUPPLY_PROP_CURRENT_MAX:
	case GBMS_PROP_CHARGING_ENABLED:
	case GBMS_PROP_CHARGE_DISABLE:
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
	case GBMS_PROP_TAPER_CONTROL:
		return 1;
	default:
		break;
	}

	return 0;
}

static struct gbms_desc gccd_psy_desc = {
	.psy_dsc.name = "gccd",
	.psy_dsc.type = POWER_SUPPLY_TYPE_UNKNOWN,
	.psy_dsc.get_property = gccd_psy_get_property,
	.psy_dsc.set_property = gccd_psy_set_property,
	.psy_dsc.property_is_writeable = gccd_psy_is_writeable,
	.psy_dsc.properties = gccd_psy_properties,
	.psy_dsc.num_properties = ARRAY_SIZE(gccd_psy_properties),
	.get_property = gccd_gbms_psy_get_property,
	.set_property = gccd_gbms_psy_set_property,
	.property_is_writeable = gccd_gbms_psy_is_writeable,
	.forward = true,
};

/* ------------------------------------------------------------------------ */

#define GCCD_DELAY_INIT_MS 500

static void gccd_init_work(struct work_struct *work)
{
	struct gccd_drv *gccd = container_of(work, struct gccd_drv,
					     init_work.work);

	if (!gccd_get_chg_psy(gccd))
		goto retry_init_work;

	if (gccd_gpio_init(gccd->device, gccd) < 0)
		goto retry_init_work;

	(void)gccd_init_fs(gccd);
	(void)gccd_init_debugfs(gccd);

	gccd->init_complete = true;
	dev_info(gccd->device, "gccd_init_work done\n");

	return;

retry_init_work:
	schedule_delayed_work(&gccd->init_work,
			      msecs_to_jiffies(GCCD_DELAY_INIT_MS));
}

static int google_ccd_probe(struct platform_device *pdev)
{
	const char *main_chg_psy_name;
	const char *buck_chg_psy_name;
	struct gccd_drv *gccd;
	int ret;
	struct power_supply_config psy_cfg = { };

	gccd = devm_kzalloc(&pdev->dev, sizeof(*gccd), GFP_KERNEL);
	if (!gccd)
		return -ENOMEM;

	gccd->device = &pdev->dev;

	/* main charger */
	ret = of_property_read_string(pdev->dev.of_node, "google,main-chg-psy-name",
				      &main_chg_psy_name);
	if (ret < 0)
		return -ENODEV;

	/* buck charger */
	ret = of_property_read_string(pdev->dev.of_node, "google,buck-chg-psy-name",
				      &buck_chg_psy_name);

	if (ret < 0)
		return -ENODEV;

	dev_info(gccd->device, "google,main-chg-psy-name=%s\n", main_chg_psy_name);
	gccd->main_chg_psy_name = devm_kstrdup(&pdev->dev,
					       main_chg_psy_name, GFP_KERNEL);
	if (!gccd->main_chg_psy_name) {
		devm_kfree(&pdev->dev, gccd);
		return -ENOMEM;
	}

	dev_info(gccd->device, "google,buck-chg-psy-name=%s\n", buck_chg_psy_name);
	gccd->buck_chg_psy_name = devm_kstrdup(&pdev->dev,
					       buck_chg_psy_name, GFP_KERNEL);
	if (!gccd->buck_chg_psy_name) {
		devm_kfree(&pdev->dev, gccd);
		return -ENOMEM;
	}

	mutex_init(&gccd->gccd_lock);
	INIT_DELAYED_WORK(&gccd->init_work, gccd_init_work);

	platform_set_drvdata(pdev, gccd);

	psy_cfg.drv_data = gccd;
	psy_cfg.of_node = pdev->dev.of_node;

	gccd->psy = devm_power_supply_register(gccd->device,
					       &gccd_psy_desc.psy_dsc, &psy_cfg);
	if (IS_ERR(gccd->psy)) {
		ret = PTR_ERR(gccd->psy);
		dev_err(gccd->device, "Couldn't register as power supply, ret=%d\n", ret);
		devm_kfree(&pdev->dev, gccd);
		return ret;
	}

	schedule_delayed_work(&gccd->init_work, 0);

	dev_info(gccd->device, "google_ccd_probe done\n");

	return 0;
}

static int google_ccd_remove(struct platform_device *pdev)
{
	struct gccd_drv *gccd = platform_get_drvdata(pdev);

	cancel_delayed_work(&gccd->init_work);

	if (gccd->main_chg_psy)
		power_supply_put(gccd->main_chg_psy);

	if (gccd->buck_chg_psy)
		power_supply_put(gccd->buck_chg_psy);

	return 0;
}

static const struct of_device_id google_ccd_of_match[] = {
	{.compatible = "google,ccd"},
	{},
};
MODULE_DEVICE_TABLE(of, google_ccd_of_match);

static struct platform_driver google_ccd_driver = {
	.driver = {
		   .name = "google_ccd",
		   .owner = THIS_MODULE,
		   .of_match_table = google_ccd_of_match,
		   .probe_type = PROBE_PREFER_ASYNCHRONOUS,
		   },
	.probe = google_ccd_probe,
	.remove = google_ccd_remove,
};

module_platform_driver(google_ccd_driver);

MODULE_DESCRIPTION("Google Charger Combine Driver");
MODULE_AUTHOR("Jack Wu <wjack@google.com>");
MODULE_LICENSE("GPL");
