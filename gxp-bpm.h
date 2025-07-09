/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * GXP bus performance monitor interface.
 *
 * Copyright (C) 2021 Google LLC
 */
#ifndef __GXP_BPM_H__
#define __GXP_BPM_H__

#include "gxp-internal.h"

/*
 * Available BPMs
 *
 * Passed to gxp_bpm_configure() and gxp_bpm_read_counter() to specify
 * which BPM to use: instruction, data, or DMA.
 */
#define INST_BPM_OFFSET	0x0000
#define DATA_BPM_OFFSET	0x1000
#define IDMA_BPM_OFFSET	0x2000

/*
 * Available BPM Events
 *
 * Passed to gxp_bpm_configure() to specify the type of event being counted.
 */
#define BPM_EVENT_ADDR_READ	0x0
#define BPM_EVENT_ADDR_WRITE	0x1
#define BPM_EVENT_WRITE_XFER	0x2
#define BPM_EVENT_READ_XFER	0x3

#define BPM_EVENT_TYPE_BIT	2
#define BPM_EVENT_TYPE_MASK	0x1F

#define BPM_START_BIT	10
#define BPM_STOP_BIT	11

#define BPM_CONFIG_OFFSET		0x00
#define BPM_CNTR_CONFIG_OFFSET		0x18
#define BPM_SNAPSHOT_CNTR_OFFSET	0x98

#define BPM_DISABLE	0x0
#define BPM_ENABLE	0x1

void gxp_bpm_configure(struct gxp_dev *gxp, u8 core, u32 bpm_offset, u32 event);
void gxp_bpm_start(struct gxp_dev *gxp, u8 core);
void gxp_bpm_stop(struct gxp_dev *gxp, u8 core);
u32 gxp_bpm_read_counter(struct gxp_dev *gxp, u8 core, u32 bpm_offset);

#endif /* __GXP_BPM_H__ */
