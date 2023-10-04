/* SPDX-License-Identifier: GPL-2.0 */
/*
 * CS40L20/CS40L25/CS40L25A/CS40L25B Wavetable Definitions
 *
 *
 * Copyright (C) 2020-2021 Cirrus Logic, Inc. and
 *                         Cirrus Logic International Semiconductor Ltd.
 */

#ifndef CS40L25_WAVETABLE_H
#define CS40L25_WAVETABLE_H

#define WT_WAVELEN_MAX		0x3FFFFF
#define WT_WAVELEN_INDEFINITE	0x400000
#define WT_WAVELEN_CALCULATED	0x800000

#define WT_REPEAT_LOOP_MARKER	0xFF

#define WT_MAX_SECTIONS		255

#define WT_T10_FLAG_DURATION	0x8

struct wt_type10_comp_section {
	u8 amplitude;
	u8 index;
	u8 repeat;
	u8 flags;
	u16 delay;
	u16 duration;
};

struct wt_type10_comp {
	u32 wlength;
	u8 nsections;
	u8 repeat;

	struct wt_type10_comp_section sections[WT_MAX_SECTIONS];
};

#define WT_T12_FLAG_CHIRP	BIT(7)
#define WT_T12_FLAG_BRAKE	BIT(6)
#define WT_T12_FLAG_AMP_REG	BIT(5)
#define WT_T12_FLAG_EXT_FREQ	BIT(4)

struct wt_type12_pwle_section {
	u16 time;
	u16 level;
	u16 frequency;
	u8 flags;
	u32 vbtarget;
};

struct wt_type12_pwle {
	u32 wlength;
	u8 repeat;
	u16 wait;
	u8 nsections;

	struct wt_type12_pwle_section sections[WT_MAX_SECTIONS];
};

#endif
