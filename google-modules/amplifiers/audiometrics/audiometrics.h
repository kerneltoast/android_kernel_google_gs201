/* SPDX-License-Identifier: GPL-2.0 OR Apache-2.0 */
/*
 * Google Whitechapel Audio Metrics Driver
 *
 * Copyright (c) 2021 Google LLC
 *
 */

#ifndef _AUDIOMETRICS_H
#define _AUDIOMETRICS_H

#include "uapi/audiometrics_api.h"

#define MIC_DEGRADE_SHIFT_BITS 3
enum mic_break_stat_mask {
	MIC_BREAK_STAT_OK = 0,
	MIC_BREAK_STAT_MIC1_BREAK = 0x1,
	MIC_BREAK_STAT_MIC2_BREAK = 0x2,
	MIC_BREAK_STAT_MIC3_BREAK = 0x4,
	MIC_BREAK_STAT_MIC_BREAK_MASK = 0x07,
	MIC_BREAK_STAT_MIC1_DEGRADE = 0x8,
	MIC_BREAK_STAT_MIC2_DEGRADE = 0x10,
	MIC_BREAK_STAT_MIC3_DEGRADE = 0x20,
	MIC_BREAK_STAT_MIC_DEGRADE_MASK = 0x38,
	MIC_BREAK_STAT_MAX = 0x3F,
};

enum {
	CODEC_STATE_UNKNOWN = -99,
	CODEC_STATE_ONLINE = 0,
};

enum {
	WDSP_STAT_CRASH = 0,
	WDSP_STAT_DOWN,
	WDSP_STAT_UP,
};

typedef int (*pdm_callback)(void* priv, int index);
#if IS_ENABLED(CONFIG_AUDIOMETRICS)
extern void pdm_callback_register(pdm_callback callback, int pdm_number, void* pdm_priv);
#else
static inline
void pdm_callback_register(pdm_callback callback, int pdm_number, void* pdm_priv)
{
}
#endif
#endif /* _AUDIOMETRICS_H */
