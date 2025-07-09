// SPDX-License-Identifier: GPL-2.0-only
/*
 * Platform thermal driver for GXP.
 *
 * Copyright (C) 2021-2023 Google LLC
 */

#include <linux/device.h>
#include <linux/minmax.h>

#include <gcip/gcip-pm.h>
#include <gcip/gcip-thermal.h>

#include "gxp-config.h"
#include "gxp-internal.h"
#include "gxp-pm.h"
#include "gxp-thermal.h"
#if GXP_HAS_MCU
#include "gxp-kci.h"
#include "gxp-mcu.h"
#endif /* GXP_HAS_MCU */
#include "mobile-soc.h"

static int gxp_thermal_get_rate(void *data, unsigned long *rate)
{
	struct gxp_dev *gxp = data;

	*rate = gxp_soc_pm_get_rate(gxp, AUR_DVFS_DOMAIN, 0);
	return 0;
}

static int gxp_thermal_set_rate(void *data, unsigned long rate)
{
	struct gxp_dev *gxp = data;
	int ret = 0;

	dev_warn(gxp->dev, "Received thermal throttling requests %lu.\n", rate);
	if (!gxp_is_direct_mode(gxp)) {
#if GXP_HAS_MCU
		struct gxp_mcu *mcu = gxp_mcu_of(gxp);

		ret = gxp_kci_notify_throttling(&mcu->kci, rate);
		if (ret > 0) {
			dev_err(gxp->dev,
				"Received GCIP_KCI_CODE_NOTIFY_THROTTLING error code: %u.", ret);
			ret = gcip_kci_error_to_errno(gxp->dev, ret);
		}
#endif /* GXP_HAS_MCU */
	} else {
		rate = max_t(unsigned long, rate, aur_power_state2rate[AUR_UUD]);
		ret = gxp_pm_blk_set_rate(gxp, rate);
	}

	if (ret) {
		dev_err(gxp->dev, "error setting gxp cooling state: %d\n", ret);
		return ret;
	}

	gxp_pm_set_thermal_limit(gxp, rate);

	return 0;
}

static int gxp_thermal_control(void *data, bool enable)
{
	int ret = -EOPNOTSUPP;
#if GXP_HAS_MCU
	struct gxp_dev *gxp = data;
	struct gxp_mcu *mcu = gxp_mcu_of(gxp);

	dev_warn(gxp->dev, "Received request to %s thermal throttling.\n",
		 enable ? "enable" : "disable");
	ret = gxp_kci_thermal_control(&mcu->kci, enable);
	if (ret) {
		if (ret > 0) {
			dev_err(gxp->dev, "Received GCIP_KCI_CODE_THERMAL_CONTROL error code: %u.",
				ret);
			ret = gcip_kci_error_to_errno(gxp->dev, ret);
		}
		dev_err(gxp->dev, "Error on %s thermal throttling: %d.\n",
			enable ? "enabling" : "disabling", ret);
	}
#endif /* GXP_HAS_MCU */
	return ret;
}

int gxp_thermal_init(struct gxp_dev *gxp)
{
	const struct gcip_thermal_args args = {
		.dev = gxp->dev,
		.pm = gxp->power_mgr->pm,
		.dentry = gxp->d_entry,
		.node_name = GXP_COOLING_NAME,
		.type = GXP_COOLING_NAME,
		.data = gxp,
		.get_rate = gxp_thermal_get_rate,
		.set_rate = gxp_thermal_set_rate,
		.control = gxp_thermal_control,
	};
	struct gcip_thermal *thermal;

	if (gxp->thermal)
		return -EEXIST;

	thermal = gcip_thermal_create(&args);
	if (IS_ERR(thermal))
		return PTR_ERR(thermal);

	gxp->thermal = thermal;

	return 0;
}

void gxp_thermal_exit(struct gxp_dev *gxp)
{
	gcip_thermal_destroy(gxp->thermal);
	gxp->thermal = NULL;
}
