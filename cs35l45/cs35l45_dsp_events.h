/* SPDX-License-Identifier: GPL-2.0 */
/*
 * cs35l45_dsp_events.h -- DSP-generated event definitions for CS35L45
 *
 * Copyright (c) 2020 Cirrus Logic Inc.
 *
 */

#ifndef __CS35L45_DSP_EVENTS_H
#define __CS35L45_DSP_EVENTS_H

#define CS35L45_MBOX3_CMD_MASK		0xFF
#define CS35L45_MBOX3_CMD_SHIFT		0
#define CS35L45_MBOX3_DATA_MASK		0xFFFFFF00
#define CS35L45_MBOX3_DATA_SHIFT	8

enum mbox3_events {
	EVENT_SPEAKER_OPEN_SHORT_STATUS = 0x66,
};

enum speaker_status {
	SPK_STATUS_ALL_CLEAR = 1,
	SPK_STATUS_OPEN_CIRCUIT = 2,
	SPK_STATUS_SHORT_CIRCUIT = 4,
};

#endif /* __CS35L45_DSP_EVENTS_H */
