// SPDX-License-Identifier: GPL-2.0-only
/*
 * GXP bus performance monitor interface.
 *
 * Copyright (C) 2021 Google LLC
 */

#include "gxp-bpm.h"
#include "gxp-config.h"

void gxp_bpm_configure(struct gxp_dev *gxp, u8 core, u32 bpm_offset, u32 event)
{
	u32 val = ((event & BPM_EVENT_TYPE_MASK) << BPM_EVENT_TYPE_BIT) |
		  BPM_ENABLE;
	u32 bpm_base = GXP_REG_CORE_INST_BPM(core) + bpm_offset;

	/* Configure event */
	gxp_write_32(gxp, bpm_base + BPM_CNTR_CONFIG_OFFSET, val);
	/* Arm counter */
	gxp_write_32(gxp, bpm_base + BPM_CONFIG_OFFSET, BPM_ENABLE);
}

void gxp_bpm_start(struct gxp_dev *gxp, u8 core)
{
	gxp_write_32(gxp, GXP_REG_CORE_PROFILING_CONDITION(core), BPM_ENABLE << BPM_START_BIT);
}

void gxp_bpm_stop(struct gxp_dev *gxp, u8 core)
{
	gxp_write_32(gxp, GXP_REG_CORE_PROFILING_CONDITION(core), BPM_ENABLE << BPM_STOP_BIT);
}

u32 gxp_bpm_read_counter(struct gxp_dev *gxp, u8 core, u32 bpm_offset)
{
	u32 bpm_base = GXP_REG_CORE_INST_BPM(core) + bpm_offset;

	/* Disarm counter */
	gxp_write_32(gxp, bpm_base + BPM_CONFIG_OFFSET, BPM_DISABLE);
	/* Read final counter value */
	return gxp_read_32(gxp, bpm_base + BPM_SNAPSHOT_CNTR_OFFSET);
}
