// SPDX-License-Identifier: GPL-2.0
/*
 * Platform thermal driver for GXP.
 *
 * Copyright (C) 2021-2023 Google LLC
 */

#include <linux/acpm_dvfs.h>
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

static int gxp_thermal_get_rate(void *data, unsigned long *rate)
{
	*rate = exynos_acpm_get_rate(AUR_DVFS_DOMAIN, 0);

	return 0;
}

static int gxp_thermal_set_rate(void *data, unsigned long rate)
{
	struct gxp_dev *gxp = data;
	int ret = 0;

	if (!gxp_is_direct_mode(gxp)) {
#if GXP_HAS_MCU
		struct gxp_mcu *mcu = gxp_mcu_of(gxp);

		ret = gxp_kci_notify_throttling(&mcu->kci, rate);
#endif /* GXP_HAS_MCU */
	} else {
		rate = max_t(unsigned long, rate,
			     aur_power_state2rate[AUR_UUD]);
		ret = gxp_pm_blk_set_rate_acpm(gxp, rate);
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
	return -EOPNOTSUPP;
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
