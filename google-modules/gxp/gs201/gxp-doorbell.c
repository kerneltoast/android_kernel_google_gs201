// SPDX-License-Identifier: GPL-2.0
/*
 * GXP doorbell interface.
 *
 * Copyright (C) 2020 Google LLC
 */

#include <linux/bitops.h>

#include "gxp-doorbell.h"
#include "gxp-internal.h"

#define GXP_DOORBELL_STRIDE (GXP_REG_DOORBELL_1_STATUS \
			     - GXP_REG_DOORBELL_0_STATUS)

void gxp_doorbell_enable_for_core(struct gxp_dev *gxp, u32 doorbell_num,
				  uint core)
{
	u32 val;

	/* Enable DOORBELL_NUM on requested core */
	val = gxp_read_32_core(gxp, core, GXP_REG_COMMON_INT_MASK_0);
	val |= BIT(doorbell_num);
	gxp_write_32_core(gxp, core, GXP_REG_COMMON_INT_MASK_0, val);
}

void gxp_doorbell_set(struct gxp_dev *gxp, u32 doorbell_num)
{
	uint offset = GXP_REG_DOORBELL_0_SET
		      + (GXP_DOORBELL_STRIDE * doorbell_num);

	gxp_write_32(gxp, offset, GXP_REG_DOORBELLS_SET_WRITEMASK);
}

void gxp_doorbell_clear(struct gxp_dev *gxp, u32 doorbell_num)
{
	uint offset = GXP_REG_DOORBELL_0_CLEAR
		      + (GXP_DOORBELL_STRIDE * doorbell_num);

	gxp_write_32(gxp, offset, GXP_REG_DOORBELLS_CLEAR_WRITEMASK);
}

u32 gxp_doorbell_status(struct gxp_dev *gxp, u32 doorbell_num)
{
	uint offset = GXP_REG_DOORBELL_0_STATUS
		      + (GXP_DOORBELL_STRIDE * doorbell_num);

	return gxp_read_32(gxp, offset);
}
