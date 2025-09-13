/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * RT4539 Backlight Driver
 *
 * Copyright (C) 2021 Google LLC.
 */

#ifndef _RT4539_H
#define _RT4539_H

enum rt4539_chip_id {
	RT4539,
};

/**
 * struct rt4539_platform_data
 * @name : Backlight driver name. If it is not defined, default name is set.
 * @bit_selection : indicate the brightness resolution
 * @dimming_mode : value of dimming mode selection field
 * @boost_switch_freq: value of boost switching frequency
 * @current_max : value of MAX_CURRENT register
 * @brightness_control: value of advanced brightness control field
 * @fade_in_out_time_control: value of fade in/out time
 * @slope_time_control: transition time of brightness value
 * @slope_time_filter: how smoothness of the slope time
 * @enabled_leds : value of LED enabled bits
 * @initial_brightness : initial value of backlight brightness
 * @boost_ovp_selection : value of boost output over voltage protection
 * @led_short_protection : enable LED short protection
 * @exponential_mapping : value of mapping mode bit
 * @led_unused_check : enable LED unused check
 * @pfm_enable : enable PFM function
 */
struct rt4539_platform_data {
	const char *name;
	u8 led_headroom;
	u8 bit_selection;
	u8 dimming_mode;
	u8 boost_switch_freq;
	u8 current_max;
	u8 brightness_control;
	u8 fade_in_out_time_control;
	u8 slope_time_control;
	u8 slope_time_filter;
	u8 enabled_leds;
	u16 initial_brightness;
	u8 boost_ovp_selection;
	bool led_short_protection;
	bool exponential_mapping;
	bool led_unused_check;
	bool pfm_enable;
};

#endif
