/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * GXP SSMT driver.
 *
 * Copyright (C) 2022 Google LLC
 */

#ifndef __GXP_SSMT_H__
#define __GXP_SSMT_H__

#include "gxp-internal.h"

#define SSMT_CFG_OFFSET (0x0004)
#define SSMT_MODE_CLAMPED (0x0u)
#define SSMT_MODE_CLIENT (0x1u)

#define SSMT_CLAMP_MODE_BYPASS (1u << 31)
#define MAX_NUM_CONTEXTS 8

struct gxp_ssmt {
	struct gxp_dev *gxp;
	void __iomem *idma_ssmt_base;
	void __iomem *inst_data_ssmt_base;
};

/*
 * Initializes @ssmt structure.
 *
 * Resources allocated in this function are all device-managed.
 *
 * Returns 0 on success, -errno otherwise.
 */
int gxp_ssmt_init(struct gxp_dev *gxp, struct gxp_ssmt *ssmt);

/*
 * Programs SSMT to have @core (0 ~ GXP_NUM_CORES - 1) issue transactions
 * with VID = @vid.
 */
void gxp_ssmt_set_core_vid(struct gxp_ssmt *ssmt, uint core, uint vid);

/*
 * Programs SSMT to always use SCIDs as VIDs.
 * Supports both client-driven and clamp mode.
 */
void gxp_ssmt_set_bypass(struct gxp_ssmt *ssmt);

#endif /* __GXP_SSMT_H__ */
