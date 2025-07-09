// SPDX-License-Identifier: GPL-2.0-only
/*
 * google_bcl_votable.c Google bcl votable driver
 *
 * Copyright (c) 2023, Google LLC. All rights reserved.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <misc/gvotable.h>
#include "bcl.h"
#include "soc/google/debug-snapshot.h"

#if IS_ENABLED(CONFIG_REGULATOR_S2MPG14)
#include <dt-bindings/interrupt-controller/zuma.h>
#include <linux/mfd/samsung/rtc-s2mpg14.h>
#include <linux/mfd/samsung/s2mpg14-register.h>
#include <linux/mfd/samsung/s2mpg15-register.h>
#include <max77779_regs.h>
#elif IS_ENABLED(CONFIG_REGULATOR_S2MPG12)
#include <dt-bindings/interrupt-controller/gs201.h>
#include <linux/mfd/samsung/rtc-s2mpg12.h>
#include <linux/mfd/samsung/s2mpg12-register.h>
#include <linux/mfd/samsung/s2mpg13-register.h>
#elif IS_ENABLED(CONFIG_REGULATOR_S2MPG10)
#include <dt-bindings/interrupt-controller/gs101.h>
#include <linux/mfd/samsung/rtc-s2mpg10.h>
#include <linux/mfd/samsung/s2mpg10-register.h>
#include <linux/mfd/samsung/s2mpg11-register.h>
#endif

#define BCL_WLC "BCL_WLC"
#define BCL_USB "BCL_USB"
#define BCL_USB_OTG "BCL_USB_OTG"

enum {
	WLC_ENABLED_TX,
	WLC_DISABLED_TX,
};

enum {
	USB_PLUGGED,
	USB_UNPLUGGED,
};

static int google_bcl_wlc_votable_callback(struct gvotable_election *el,
					   const char *reason, void *value)
{
	struct bcl_device *bcl_dev = gvotable_get_data(el);
	int ret;
	u8 wlc_tx_enable = (long)value ? WLC_ENABLED_TX : WLC_DISABLED_TX;

	if (!smp_load_acquire(&bcl_dev->enabled))
		return -EINVAL;
#if IS_ENABLED(CONFIG_SOC_ZUMAPRO)
	ret = max77779_adjust_batoilo_lvl(bcl_dev, wlc_tx_enable,
   					  bcl_dev->batt_irq_conf1.batoilo_wlc_trig_lvl,
   					  bcl_dev->batt_irq_conf2.batoilo_wlc_trig_lvl);
#else
	ret = max77759_adjust_batoilo_lvl(bcl_dev, wlc_tx_enable,
   					  bcl_dev->batt_irq_conf1.batoilo_wlc_trig_lvl);
#endif
	if (ret < 0) {
		dev_err(bcl_dev->device, "BATOILO cannot be adjusted\n");
		return ret;
	}

	/* b/335695535 outlines max77759 configuration */

	return 0;
}

static int google_bcl_usb_votable_callback(struct gvotable_election *el,
					   const char *reason, void *value)
{
	int ret = 0, err = 0;
	struct bcl_device *bcl_dev = gvotable_get_data(el);
	u8 usb_enable = (long)value ? USB_PLUGGED: USB_UNPLUGGED;
	union power_supply_propval prop = { };

	if (!smp_load_acquire(&bcl_dev->enabled))
		return -EINVAL;
	if (bcl_dev->usb_otg_conf && bcl_dev->otg_psy)
		err = power_supply_get_property(bcl_dev->otg_psy,
						POWER_SUPPLY_PROP_STATUS, &prop);
	else
		err = power_supply_get_property(bcl_dev->batt_psy,
						POWER_SUPPLY_PROP_STATUS, &prop);

	if (prop.intval != POWER_SUPPLY_STATUS_DISCHARGING) {
#if IS_ENABLED(CONFIG_SOC_ZUMAPRO)
		ret = max77779_adjust_batoilo_lvl(bcl_dev, usb_enable,
						  bcl_dev->batt_irq_conf1.batoilo_usb_trig_lvl,
						  bcl_dev->batt_irq_conf2.batoilo_usb_trig_lvl);
#else
		ret = max77759_adjust_batoilo_lvl(bcl_dev, usb_enable,
						  bcl_dev->batt_irq_conf1.batoilo_usb_trig_lvl);
#endif
		if (ret < 0)
			dev_err(bcl_dev->device, "USB: BATOILO cannot be adjusted\n");
		return ret;
	}

#if IS_ENABLED(CONFIG_REGULATOR_S2MPG14)
	pmic_write(CORE_PMIC_MAIN_RTC, bcl_dev, S2MPG14_RTC_SCRATCH1, usb_enable);
#elif IS_ENABLED(CONFIG_REGULATOR_S2MPG12)
	pmic_write(CORE_PMIC_MAIN_RTC, bcl_dev, S2MPG12_RTC_SCRATCH1, usb_enable);
#endif

	if (bcl_dev->usb_otg_conf) {
#if IS_ENABLED(CONFIG_SOC_ZUMAPRO)
		ret = max77779_adjust_batoilo_lvl(bcl_dev, usb_enable,
						  bcl_dev->batt_irq_conf1.batoilo_otg_trig_lvl,
						  bcl_dev->batt_irq_conf2.batoilo_otg_trig_lvl);
#else
		ret = max77759_adjust_batoilo_lvl(bcl_dev, usb_enable,
						  bcl_dev->batt_irq_conf1.batoilo_otg_trig_lvl);
#endif
		if (ret < 0)
			dev_err(bcl_dev->device, "USB: BATOILO cannot be adjusted\n");

#if IS_ENABLED(CONFIG_SOC_ZUMAPRO)
		ret = max77779_adjust_bat_open_to(bcl_dev, usb_enable);
		if (ret < 0)
			dev_err(bcl_dev->device, "USB: BAT OPEN cannot be adjusted\n");
#endif
	}

	return ret;
}

int google_bcl_setup_votable(struct bcl_device *bcl_dev)
{
	int ret;

	bcl_dev->toggle_wlc = gvotable_create_bool_election(NULL, google_bcl_wlc_votable_callback,
							    bcl_dev);
	if (IS_ERR_OR_NULL(bcl_dev->toggle_wlc)) {
		ret = PTR_ERR(bcl_dev->toggle_wlc);
		dev_err(bcl_dev->device, "no toggle_wlc votable (%d)\n", ret);
		return ret;
	}
	gvotable_set_vote2str(bcl_dev->toggle_wlc, gvotable_v2s_int);
	gvotable_election_set_name(bcl_dev->toggle_wlc, BCL_WLC);

	bcl_dev->toggle_usb = gvotable_create_bool_election(NULL, google_bcl_usb_votable_callback,
							    bcl_dev);
	if (IS_ERR_OR_NULL(bcl_dev->toggle_usb)) {
		ret = PTR_ERR(bcl_dev->toggle_usb);
		gvotable_destroy_election(bcl_dev->toggle_wlc);
		dev_err(bcl_dev->device, "no toggle_usb votable (%d)\n", ret);
		return ret;
	}
	gvotable_set_vote2str(bcl_dev->toggle_usb, gvotable_v2s_int);
	gvotable_election_set_name(bcl_dev->toggle_usb, BCL_USB);

	return 0;
}

void google_bcl_remove_votable(struct bcl_device *bcl_dev)
{
	if (!IS_ERR_OR_NULL(bcl_dev->toggle_wlc))
		gvotable_destroy_election(bcl_dev->toggle_wlc);
	if (!IS_ERR_OR_NULL(bcl_dev->toggle_usb))
		gvotable_destroy_election(bcl_dev->toggle_usb);
}
