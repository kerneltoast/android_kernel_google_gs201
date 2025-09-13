// SPDX-License-Identifier: GPL-2.0-only
/*
 * Amalthea power management implementations.
 *
 * Copyright (C) 2023-2024 Google LLC
 */

#include "gxp-internal.h"
#include "gxp-lpm.h"
#include "gxp-pm.h"

static int amalthea_before_blk_power_down(struct gxp_dev *gxp)
{
	/* Need to put TOP LPM into active state before blk off. */
	if (!gxp_lpm_wait_state_eq(gxp, LPM_PSM_TOP, LPM_ACTIVE_STATE)) {
		dev_err(gxp->dev,
			"failed to force TOP LPM to PS0 during blk down\n");
		return -EAGAIN;
	}

	return 0;
}

static const struct gxp_pm_ops gxp_pm_ops = {
	.before_blk_power_down = amalthea_before_blk_power_down,
};

void gxp_pm_chip_set_ops(struct gxp_power_manager *mgr)
{
	mgr->ops = &gxp_pm_ops;
}

void gxp_pm_chip_init(struct gxp_dev *gxp)
{
}

void gxp_pm_chip_exit(struct gxp_dev *gxp)
{
}
