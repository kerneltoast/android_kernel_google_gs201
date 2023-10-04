// SPDX-License-Identifier: GPL-2.0
/*
 * GXP bus performance monitor interface.
 *
 * Copyright (C) 2021 Google LLC
 */

#include "gxp-bpm.h"
#include "gxp-config.h"

#define BPM_EVENT_TYPE_BIT	2
#define BPM_EVENT_TYPE_MASK	0x1F

#define BPM_START_BIT	10
#define BPM_STOP_BIT	11

#define BPM_CONFIG_OFFSET		0x00
#define BPM_CNTR_CONFIG_OFFSET		0x18
#define BPM_SNAPSHOT_CNTR_OFFSET	0x98

#define BPM_DISABLE	0x0
#define BPM_ENABLE	0x1

void gxp_bpm_configure(struct gxp_dev *gxp, u8 core, u32 bpm_offset, u32 event)
{
	u32 val = ((event & BPM_EVENT_TYPE_MASK) << BPM_EVENT_TYPE_BIT) |
		  BPM_ENABLE;
	u32 bpm_base = GXP_REG_INST_BPM + bpm_offset;

	/* Configure event */
	gxp_write_32_core(gxp, core, bpm_base + BPM_CNTR_CONFIG_OFFSET, val);
	/* Arm counter */
	gxp_write_32_core(gxp, core, bpm_base + BPM_CONFIG_OFFSET, BPM_ENABLE);
}

void gxp_bpm_start(struct gxp_dev *gxp, u8 core)
{
	gxp_write_32_core(gxp, core, GXP_REG_PROFILING_CONDITION,
			  BPM_ENABLE << BPM_START_BIT);
}

void gxp_bpm_stop(struct gxp_dev *gxp, u8 core)
{
	gxp_write_32_core(gxp, core, GXP_REG_PROFILING_CONDITION,
			  BPM_ENABLE << BPM_STOP_BIT);
}

u32 gxp_bpm_read_counter(struct gxp_dev *gxp, u8 core, u32 bpm_offset)
{
	u32 bpm_base = GXP_REG_INST_BPM + bpm_offset;

	/* Disarm counter */
	gxp_write_32_core(gxp, core, bpm_base + BPM_CONFIG_OFFSET, BPM_DISABLE);
	/* Read final counter value */
	return gxp_read_32_core(gxp, core, bpm_base + BPM_SNAPSHOT_CNTR_OFFSET);
}
