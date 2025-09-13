// SPDX-License-Identifier: GPL-2.0-only
/*
 * GXP doorbell interface.
 *
 * Copyright (C) 2020 Google LLC
 */

#include <linux/bitops.h>

#include "gxp-doorbell.h"
#include "gxp-internal.h"

void gxp_doorbell_enable_for_core(struct gxp_dev *gxp, u32 doorbell_num,
				  uint core)
{
	u32 val;

	/* Enable DOORBELL_NUM on requested core */
	val = gxp_read_32(gxp, GXP_REG_CORE_COMMON_INT_MASK_0(core));
	val |= BIT(doorbell_num);
	gxp_write_32(gxp, GXP_REG_CORE_COMMON_INT_MASK_0(core), val);
}

void gxp_doorbell_set(struct gxp_dev *gxp, u32 doorbell_num)
{
	uint offset = GXP_REG_DOORBELL_SET(doorbell_num);

	gxp_write_32(gxp, offset, GXP_REG_DOORBELL_SET_WRITEMASK);
}

void gxp_doorbell_clear(struct gxp_dev *gxp, u32 doorbell_num)
{
	uint offset = GXP_REG_DOORBELL_CLEAR(doorbell_num);

	gxp_write_32(gxp, offset, GXP_REG_DOORBELL_CLEAR_WRITEMASK);
}

u32 gxp_doorbell_status(struct gxp_dev *gxp, u32 doorbell_num)
{
	uint offset = GXP_REG_DOORBELL_STATUS(doorbell_num);

	return gxp_read_32(gxp, offset);
}
