// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google Whitechapel AoC ALSA Driver - Mixer controls
 *
 * Copyright (c) 2019 Google LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include "aoc_alsa.h"

/* Volume maximum and minimum */
#define CTRL_VOL_MIN 0
#define CTRL_VOL_MAX 1000
#define AOC_MIC_RECORD_GAIN_IN_DB_MIN -40
#define AOC_MIC_RECORD_GAIN_IN_DB_MAX 30

#define MMAP_MIC_RECORD_GAIN_IN_DB_MIN -300
#define MMAP_MIC_RECORD_GAIN_IN_DB_MAX 0

#define COMPRE_OFFLOAD_GAIN_MIN 0
#define COMPRE_OFFLOAD_GAIN_MAX 8388608 /* 2^23 = 8388608 */

#define AOC_CHIRP_INTERVAL_KEY 0
#define AOC_CHIRP_ENABLE_KEY 1
#define AOC_CHIRP_MODE_KEY 2

/*
 * Redefined the macro from soc.h so that the control value can be negative.
 * In orginal definition, xmin can be a negative value,  but the min control
 * value is always zero.
 */
#define SOC_SINGLE_RANGE_EXT_TLV_modified(xname, xreg, xshift, xmin, xmax, count, xhandler_get,    \
					  xhandler_put, tlv_array)                                 \
	{                                                                                          \
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname),                              \
		.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ | SNDRV_CTL_ELEM_ACCESS_READWRITE,        \
		.tlv.p = (tlv_array), .info = snd_soc_info_volsw_range_modified,                   \
		.get = xhandler_get, .put = xhandler_put,                                          \
		.private_value = (uintptr_t) & (struct soc_mixer_control)                          \
		{                                                                                  \
			.reg = xreg, .rreg = xreg, .shift = xshift, .rshift = count, .min = xmin,  \
			.max = xmax, .platform_max = xmax, .invert = 0                             \
		}                                                                                  \
	}

static int snd_soc_info_volsw_range_modified(struct snd_kcontrol *kcontrol,
					     struct snd_ctl_elem_info *uinfo)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;

	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = (mc->rshift <= 1) ? 1 : mc->rshift;
	uinfo->value.integer.min = mc->min;
	uinfo->value.integer.max = mc->max;

	return 0;
}

static int snd_aoc_ctl_info(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_info *uinfo)
{
	if (kcontrol->private_value == PCM_PLAYBACK_VOLUME) {
		uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
		uinfo->count = 1;
		uinfo->value.integer.min = CTRL_VOL_MIN;
		uinfo->value.integer.max = CTRL_VOL_MAX;
	} else if (kcontrol->private_value == PCM_PLAYBACK_MUTE) {
		uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
		uinfo->count = 1;
		uinfo->value.integer.min = 0;
		uinfo->value.integer.max = 1;
	} else if (kcontrol->private_value == BUILDIN_MIC_POWER_STATE) {
		uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
		uinfo->count = NUM_OF_BUILTIN_MIC;
		uinfo->value.integer.min = 0;
		uinfo->value.integer.max = 1;
	} else if (kcontrol->private_value == BUILDIN_MIC_CAPTURE_LIST) {
		uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
		uinfo->count = NUM_OF_BUILTIN_MIC;
		uinfo->value.integer.min = -1;
		uinfo->value.integer.max = NUM_OF_BUILTIN_MIC - 1;
	} else if (kcontrol->private_value == BUILDIN_US_MIC_CAPTURE_LIST) {
		uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
		uinfo->count = NUM_OF_BUILTIN_MIC;
		uinfo->value.integer.min = -1;
		uinfo->value.integer.max = NUM_OF_BUILTIN_MIC - 1;
	} else if (kcontrol->private_value == A2DP_ENCODER_PARAMETERS) {
		uinfo->type = SNDRV_CTL_ELEM_TYPE_BYTES;
		uinfo->count = sizeof(struct AUDIO_OUTPUT_BT_A2DP_ENC_CFG);
	}
	return 0;
}

static int snd_aoc_ctl_get(struct snd_kcontrol *kcontrol,
			   struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	BUG_ON(!chip && !(chip->avail_substreams & AVAIL_SUBSTREAMS_MASK));

	if (kcontrol->private_value == PCM_PLAYBACK_VOLUME)
		ucontrol->value.integer.value[0] = chip2alsa(chip->volume);
	else if (kcontrol->private_value == PCM_PLAYBACK_MUTE)
		ucontrol->value.integer.value[0] = chip->mute;

	mutex_unlock(&chip->audio_mutex);
	return 0;
}

static int
snd_aoc_buildin_mic_power_ctl_get(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	int i;
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	for (i = 0; i < NUM_OF_BUILTIN_MIC; i++)
		ucontrol->value.integer.value[i] =
			aoc_get_builtin_mic_power_state(chip, i);

	mutex_unlock(&chip->audio_mutex);
	return 0;
}

static int
snd_aoc_buildin_mic_power_ctl_put(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	int i;
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	for (i = 0; i < NUM_OF_BUILTIN_MIC; i++)
		aoc_set_builtin_mic_power_state(
			chip, i, ucontrol->value.integer.value[i]);

	mutex_unlock(&chip->audio_mutex);
	return 0;
}

static int
snd_aoc_buildin_mic_capture_list_ctl_get(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
{
	int i;
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	for (i = 0; i < NUM_OF_BUILTIN_MIC; i++)
		ucontrol->value.integer.value[i] =
			chip->buildin_mic_id_list[i]; // geting power state;

	mutex_unlock(&chip->audio_mutex);
	return 0;
}

static int
snd_aoc_buildin_mic_capture_list_ctl_put(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
{
	int i;
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	for (i = 0; i < NUM_OF_BUILTIN_MIC; i++)
		chip->buildin_mic_id_list[i] =
			ucontrol->value.integer.value[i]; // geting power state;

	mutex_unlock(&chip->audio_mutex);
	return 0;
}

static int
snd_aoc_buildin_us_mic_capture_list_ctl_get(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
{
	int i;
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	for (i = 0; i < NUM_OF_BUILTIN_MIC; i++)
		ucontrol->value.integer.value[i] =
			chip->buildin_us_mic_id_list[i];

	mutex_unlock(&chip->audio_mutex);
	return 0;
}

static int
snd_aoc_buildin_us_mic_capture_list_ctl_put(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
{
	int i;
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	for (i = 0; i < NUM_OF_BUILTIN_MIC; i++)
		chip->buildin_us_mic_id_list[i] =
			ucontrol->value.integer.value[i];

	mutex_unlock(&chip->audio_mutex);
	return 0;
}

static int incall_capture_enable_ctl_get(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);
	struct soc_enum *mc = (struct soc_enum *)kcontrol->private_value;
	int stream = mc->shift_l;
	int err = 0;

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	err = aoc_incall_capture_enable_get(chip, stream, &ucontrol->value.integer.value[0]);
	if (err < 0)
		pr_err("ERR:%d get incall capture stream %d fail\n", err, stream);

	mutex_unlock(&chip->audio_mutex);
	return err;
}

static int incall_capture_enable_ctl_set(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);
	struct soc_enum *mc = (struct soc_enum *)kcontrol->private_value;
	int stream = mc->shift_l;
	int err = 0;

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	err = aoc_incall_capture_enable_set(chip, stream, ucontrol->value.integer.value[0]);
	if (err < 0)
		pr_err("ERR:%d set voice call capture stream %d fail\n", err, stream);

	mutex_unlock(&chip->audio_mutex);
	return err;
}

static int incall_playback_enable_ctl_get(struct snd_kcontrol *kcontrol,
					  struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);
	struct soc_mixer_control *mc = (struct soc_mixer_control *)kcontrol->private_value;
	int playback_stream = mc->shift;
	int err = 0;

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	err = aoc_incall_playback_enable_get(chip, playback_stream,
					     &ucontrol->value.integer.value[0]);
	if (err < 0)
		pr_err("ERR:%d get incall playback stream %d fail\n", err, playback_stream);

	mutex_unlock(&chip->audio_mutex);
	return err;
}

static int incall_playback_enable_ctl_set(struct snd_kcontrol *kcontrol,
					  struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);
	struct soc_mixer_control *mc = (struct soc_mixer_control *)kcontrol->private_value;
	int playback_stream = mc->shift;
	int err = 0;

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	err = aoc_incall_playback_enable_set(chip, playback_stream,
					     ucontrol->value.integer.value[0]);
	if (err < 0)
		pr_err("ERR:%d set voice call playback stream %d fail\n", err, playback_stream);

	mutex_unlock(&chip->audio_mutex);
	return err;
}

static int mic_power_ctl_get(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	u32 mic_idx = (u32)mc->shift;

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	/* geting power statef from AoC ; */
	ucontrol->value.integer.value[0] =
		aoc_get_builtin_mic_power_state(chip, mic_idx);

	mutex_unlock(&chip->audio_mutex);
	return 0;
}

static int mic_power_ctl_set(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	u32 mic_idx = (u32)mc->shift;

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	aoc_set_builtin_mic_power_state(chip, mic_idx,
					ucontrol->value.integer.value[0]);

	mutex_unlock(&chip->audio_mutex);
	return 0;
}

static int mic_clock_rate_get(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	ucontrol->value.integer.value[0] = aoc_mic_clock_rate_get(chip);

	mutex_unlock(&chip->audio_mutex);
	return 0;
}

static int mic_hw_gain_get(struct snd_kcontrol *kcontrol,
			   struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);
	struct soc_mixer_control *mc = (struct soc_mixer_control *)kcontrol->private_value;
	u32 state = (u32)mc->shift;

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	ucontrol->value.integer.value[0] = aoc_mic_hw_gain_get(chip, state);

	mutex_unlock(&chip->audio_mutex);
	return 0;
}

static int mic_hw_gain_set(struct snd_kcontrol *kcontrol,
			   struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	u32 state = (u32)mc->shift;

	if (ucontrol->value.integer.value[0] < MIC_HW_GAIN_IN_CB_MIN ||
	    ucontrol->value.integer.value[0] > MIC_HW_GAIN_IN_CB_MAX)
		return -EINVAL;

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	aoc_mic_hw_gain_set(chip, state, ucontrol->value.integer.value[0]);

	mutex_unlock(&chip->audio_mutex);
	return 0;
}

static int sidetone_enable_ctl_get(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	ucontrol->value.integer.value[0] = chip->sidetone_enable;

	mutex_unlock(&chip->audio_mutex);
	return 0;
}

static int incall_mic_sink_mute_ctl_set(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);
	struct soc_mixer_control *mc = (struct soc_mixer_control *)kcontrol->private_value;
	int param = mc->shift;
	int val, err = 0;

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	val = ucontrol->value.integer.value[0];
	err = aoc_incall_mic_sink_mute_set(chip, param, val);
	if (err < 0)
		pr_err("ERR:%d incall %s mute set to %d fail\n", err, (param == 0) ? "mic" : "sink",
		       val);

	mutex_unlock(&chip->audio_mutex);
	return err;
}

static int incall_mic_sink_mute_ctl_get(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);
	struct soc_mixer_control *mc = (struct soc_mixer_control *)kcontrol->private_value;
	int param = mc->shift;
	int err = 0;

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	err = aoc_incall_mic_sink_mute_get(chip, param, &ucontrol->value.integer.value[0]);
	if (err < 0)
		pr_err("ERR:%d incall %s mute get fail\n", err, (param == 0) ? "mic" : "sink");

	mutex_unlock(&chip->audio_mutex);
	return err;
}

static int incall_playback_mic_channel_ctl_set(struct snd_kcontrol *kcontrol,
					       struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);
	struct soc_mixer_control *mc = (struct soc_mixer_control *)kcontrol->private_value;
	int stream = mc->shift;
	int val, err = 0;

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	val = ucontrol->value.integer.value[0];
	err = aoc_incall_playback_mic_channel_set(chip, stream, val);
	if (err < 0)
		pr_err("ERR:%d incall playback mic source set fail for ring %d: mic %d\n", err,
		       stream, val);

	mutex_unlock(&chip->audio_mutex);
	return err;
}

static int incall_playback_mic_channel_ctl_get(struct snd_kcontrol *kcontrol,
					       struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);
	struct soc_mixer_control *mc = (struct soc_mixer_control *)kcontrol->private_value;
	int stream = mc->shift;
	int err = 0;

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	err = aoc_incall_playback_mic_channel_get(chip, stream, &ucontrol->value.integer.value[0]);
	if (err < 0)
		pr_err("ERR:%d incall playback mic source get fail for ring %d\n", err, stream);

	mutex_unlock(&chip->audio_mutex);
	return err;
}

static int mic_record_gain_ctl_set(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);
	int val, err = 0;

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	val = ucontrol->value.integer.value[0];
	err = aoc_mic_record_gain_set(chip, val);
	if (err < 0)
		pr_err("ERR:%d mic record gain set to %d fail\n", err, val);

	mutex_unlock(&chip->audio_mutex);
	return err;
}

static int mic_record_gain_ctl_get(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);
	int err = 0;

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	err = aoc_mic_record_gain_get(chip, &ucontrol->value.integer.value[0]);
	if (err < 0)
		pr_err("ERR:%d mic record gain get fail\n", err);

	mutex_unlock(&chip->audio_mutex);
	return err;
}

static int mmap_record_gain_ctl_set(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);
	int val, err = 0;

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	val = ucontrol->value.integer.value[0];
	err = aoc_mmap_record_gain_set(chip, val);
	if (err < 0)
		pr_err("ERR:%d mmap record gain set to %d fail\n", err, val);

	mutex_unlock(&chip->audio_mutex);
	return err;
}

static int mmap_record_gain_ctl_get(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);
	int err = 0;

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	err = aoc_mmap_record_gain_get(chip, &ucontrol->value.integer.value[0]);
	if (err < 0)
		pr_err("ERR:%d mmap record gain get fail\n", err);

	mutex_unlock(&chip->audio_mutex);
	return err;
}

static int lvm_enable_ctl_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);
	int val, err = 0;

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	val = ucontrol->value.integer.value[0];
	err = aoc_lvm_enable_set(chip, val);
	if (err < 0)
		pr_err("ERR:%d lvm %s fail\n", err, val ? "Enable" : "Disable");

	mutex_unlock(&chip->audio_mutex);
	return err;
}

static int lvm_enable_ctl_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);
	int err = 0;

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	err = aoc_lvm_enable_get(chip, &ucontrol->value.integer.value[0]);
	if (err < 0)
		pr_err("ERR:%d lvm enable get fail\n", err);

	mutex_unlock(&chip->audio_mutex);

	return err;
}

static int decoder_ref_enable_ctl_set(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);
	int val, err = 0;

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	val = ucontrol->value.integer.value[0];
	err = aoc_decoder_ref_enable_set(chip, val);
	if (err < 0)
		pr_err("ERR:%d lvm %s fail\n", err, val ? "Enable" : "Disable");

	mutex_unlock(&chip->audio_mutex);
	return err;
}

static int decoder_ref_enable_ctl_get(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);
	int err = 0;

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	err = aoc_decoder_ref_enable_get(chip, &ucontrol->value.integer.value[0]);
	if (err < 0)
		pr_err("ERR:%d lvm enable get fail\n", err);

	mutex_unlock(&chip->audio_mutex);

	return err;
}

static int audio_capture_eraser_enable_ctl_get(struct snd_kcontrol *kcontrol,
					       struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	ucontrol->value.integer.value[0] = chip->capture_eraser_enable;

	mutex_unlock(&chip->audio_mutex);

	return 0;
}

static int audio_capture_eraser_enable_ctl_set(struct snd_kcontrol *kcontrol,
					       struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);
	int err = 0;

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	chip->capture_eraser_enable = ucontrol->value.integer.value[0];
	err = aoc_audio_capture_eraser_enable(chip, chip->capture_eraser_enable);
	if (err < 0)
		pr_err("ERR:%d capture eraser %s fail\n", err,
		       (chip->capture_eraser_enable) ? "Enable" : "Disable");

	mutex_unlock(&chip->audio_mutex);
	return err;
}

#if ! IS_ENABLED(CONFIG_SOC_GS101)
static int hotword_tap_enable_ctl_get(struct snd_kcontrol *kcontrol,
					       struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	ucontrol->value.integer.value[0] = chip->hotword_tap_enable;

	mutex_unlock(&chip->audio_mutex);

	return 0;
}

static int hotword_tap_enable_ctl_set(struct snd_kcontrol *kcontrol,
					       struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);
	int err = 0;

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	chip->hotword_tap_enable = ucontrol->value.integer.value[0];
	err = aoc_hotword_tap_enable(chip, chip->hotword_tap_enable);
	if (err < 0)
		pr_err("ERR:%d hotword_tap %s fail\n", err,
		       (chip->hotword_tap_enable) ? "Enable" : "Disable");

	mutex_unlock(&chip->audio_mutex);
	return err;
}
#endif

static int audio_cca_module_load_ctl_get(struct snd_kcontrol *kcontrol,
					       struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	ucontrol->value.integer.value[0] = chip->cca_module_loaded;

	mutex_unlock(&chip->audio_mutex);

	return 0;
}

static int audio_cca_module_load_ctl_set(struct snd_kcontrol *kcontrol,
					       struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);
	int err = 0;

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	chip->cca_module_loaded = ucontrol->value.integer.value[0];
	err = aoc_load_cca_module(chip, chip->cca_module_loaded);
	if (err < 0)
		pr_err("ERR:%d %s CCA fail\n", err,
		       (chip->cca_module_loaded) ? "Load" : "Unload");

	mutex_unlock(&chip->audio_mutex);
	return err;
}

static int audio_gapless_offload_ctl_get(struct snd_kcontrol *kcontrol,
					       struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	ucontrol->value.integer.value[0] = chip->gapless_offload_enable;

	mutex_unlock(&chip->audio_mutex);

	return 0;
}

static int audio_gapless_offload_ctl_set(struct snd_kcontrol *kcontrol,
					       struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);
	int err = 0;

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	chip->gapless_offload_enable = ucontrol->value.integer.value[0];

	mutex_unlock(&chip->audio_mutex);
	return err;
}

static int sidetone_enable_ctl_set(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);
	int err = 0;

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	chip->sidetone_enable = ucontrol->value.integer.value[0];
	err = aoc_sidetone_enable(chip, chip->sidetone_enable);
	if (err < 0)
		pr_err("ERR:%d sidetone %s fail\n", err,
		       (chip->sidetone_enable) ? "Enable" : "Disable");

	mutex_unlock(&chip->audio_mutex);
	return err;
}

static int aoc_sidetone_cfg_ctl_get(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);
	struct soc_mixer_control *mc = (struct soc_mixer_control *)kcontrol->private_value;
	u32 param = (u32)mc->shift;
	int err = 0;

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	err = aoc_sidetone_cfg_get(chip, param, &ucontrol->value.integer.value[0]);
	if (err < 0)
		pr_err("ERR:%d sidetone config fail to get param %d\n", err, param);

	mutex_unlock(&chip->audio_mutex);
	return err;
}

static int aoc_sidetone_cfg_ctl_set(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);
	struct soc_mixer_control *mc = (struct soc_mixer_control *)kcontrol->private_value;
	u32 param = (u32)mc->shift;
	int err = 0;

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	err = aoc_sidetone_cfg_set(chip, param, ucontrol->value.integer.value[0]);
	if (err < 0)
		pr_err("ERR:%d sidetone config fail to set param %d\n", err, param);

	mutex_unlock(&chip->audio_mutex);
	return err;
}

static int aoc_sidetone_eq_ctl_get(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);
	struct soc_mixer_control *mc = (struct soc_mixer_control *)kcontrol->private_value;
	u32 biquad_idx = (u32)mc->shift;
	int err = 0;

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	err = aoc_sidetone_eq_get(chip, biquad_idx, ucontrol->value.integer.value);
	if (err < 0)
		pr_err("ERR:%d sidetone EQ biquad %d fail to get parameter\n", err, biquad_idx);

	mutex_unlock(&chip->audio_mutex);
	return err;
}

static int aoc_sidetone_eq_ctl_set(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);
	struct soc_mixer_control *mc = (struct soc_mixer_control *)kcontrol->private_value;
	u32 biquad_idx = (u32)mc->shift;
	int err = 0;

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	err = aoc_sidetone_eq_set(chip, biquad_idx, ucontrol->value.integer.value);
	if (err < 0)
		pr_err("ERR:%d sidetone EQ biquad %d fail to set parameter\n", err, biquad_idx);

	mutex_unlock(&chip->audio_mutex);
	return err;
}

static int mic_dc_blocker_get(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	ucontrol->value.integer.value[0] = aoc_mic_dc_blocker_get(chip);

	mutex_unlock(&chip->audio_mutex);
	return 0;
}

static int mic_dc_blocker_set(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	aoc_mic_dc_blocker_set(chip, ucontrol->value.integer.value[0]);

	mutex_unlock(&chip->audio_mutex);
	return 0;
}

static int compr_offload_volume_get(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);

	if(chip->compr_offload_volume<0 || chip->compr_offload_volume>1000)
		return -EINVAL;

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	ucontrol->value.integer.value[0] = chip->compr_offload_volume;
	mutex_unlock(&chip->audio_mutex);
	return 0;
}

static int compr_offload_volume_set(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	chip->compr_offload_volume = ucontrol->value.integer.value[0];

	/* temporary solution */
	aoc_audio_volume_set(chip, chip->compr_offload_volume, OFF_LOAD, ASNK_SPEAKER);
	aoc_audio_volume_set(chip, chip->compr_offload_volume, OFF_LOAD, ASNK_USB);
	aoc_audio_volume_set(chip, chip->compr_offload_volume, OFF_LOAD, ASNK_BT);

	mutex_unlock(&chip->audio_mutex);
	return 0;
}

static int aoc_compr_offload_gain_ctl_get(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);
	int err = 0;

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	err = aoc_compr_offload_linear_gain_get(chip, &ucontrol->value.integer.value[0]);
	if (err < 0)
		pr_err("ERR:%d compr offload linear gain get fail\n", err);

	mutex_unlock(&chip->audio_mutex);
	return 0;
}

static int aoc_compr_offload_gain_ctl_set(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);
	int err = 0;

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	err = aoc_compr_offload_linear_gain_set(chip, &ucontrol->value.integer.value[0]);
	if (err < 0)
		pr_err("ERR:%d compr offload linear gain set fail\n", err);

	mutex_unlock(&chip->audio_mutex);
	return err;
}

static int pcm_wait_time_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	ucontrol->value.integer.value[0] = chip->pcm_wait_time_in_ms;

	mutex_unlock(&chip->audio_mutex);
	return 0;
}

static int pcm_wait_time_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);
	int val = ucontrol->value.integer.value[0];

	if (val < 0 || val > DEFAULT_PCM_WAIT_TIME_IN_MSECS)
		return -EINVAL;

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	chip->pcm_wait_time_in_ms = val;

	mutex_unlock(&chip->audio_mutex);
	return 0;
}

static int voice_pcm_wait_time_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	ucontrol->value.integer.value[0] = chip->voice_pcm_wait_time_in_ms;

	mutex_unlock(&chip->audio_mutex);
	return 0;
}

static int voice_pcm_wait_time_set(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);
	int val = ucontrol->value.integer.value[0];

	if (val < 0 || val > DEFAULT_PCM_WAIT_TIME_IN_MSECS)
		return -EINVAL;

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	chip->voice_pcm_wait_time_in_ms = val;

	mutex_unlock(&chip->audio_mutex);
	return 0;
}

static int audio_capture_mic_source_get(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	ucontrol->value.integer.value[0] = chip->audio_capture_mic_source;

	mutex_unlock(&chip->audio_mutex);
	return 0;
}

static int audio_capture_mic_source_set(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);
	int err = 0;

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	chip->audio_capture_mic_source = ucontrol->value.integer.value[0];
	if (chip->audio_capture_mic_source == DEFAULT_MIC)
		chip->audio_capture_mic_source = BUILTIN_MIC;

	pr_info("mic source set: %d\n", chip->audio_capture_mic_source);

	if (err < 0)
		pr_err("ERR:%d in audio capture mic source set\n", err);

	mutex_unlock(&chip->audio_mutex);

	return err;
}

static int eraser_aec_ref_source_get(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	ucontrol->value.integer.value[0] = chip->eraser_aec_ref_source;

	mutex_unlock(&chip->audio_mutex);
	return 0;
}

static int eraser_aec_ref_source_set(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);
	int err;

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	if (ucontrol->value.integer.value[0] < NUM_AEC_REF_SOURCE) {
		chip->eraser_aec_ref_source = ucontrol->value.integer.value[0];
		err = aoc_eraser_aec_reference_set(chip, chip->eraser_aec_ref_source);
		if (err < 0)
			pr_err("ERR:%d in Eraser aec ref source set\n", err);
	} else {
		err = -EINVAL;
		pr_err("ERR:%d invalid ft aec ref source\n", err);
	}

	mutex_unlock(&chip->audio_mutex);
	return err;
}

static int ft_aec_ref_source_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	ucontrol->value.integer.value[0] = chip->ft_aec_ref_source;

	mutex_unlock(&chip->audio_mutex);
	return 0;
}

static int ft_aec_ref_source_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);
	int err = 0;

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	if (ucontrol->value.integer.value[0] < NUM_AEC_REF_SOURCE)
		chip->ft_aec_ref_source = ucontrol->value.integer.value[0];
	else {
		err = -EINVAL;
		pr_err("ERR:%d invalid ft aec ref source\n", err);
	}

	mutex_unlock(&chip->audio_mutex);
	return err;
}

static int voice_call_mic_source_get(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	ucontrol->value.integer.value[0] = chip->voice_call_mic_source;

	mutex_unlock(&chip->audio_mutex);
	return 0;
}

static int voice_call_mic_source_set(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	chip->voice_call_mic_source = ucontrol->value.integer.value[0];

	mutex_unlock(&chip->audio_mutex);
	return 0;
}

static int voice_call_mic_mute_get(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	ucontrol->value.integer.value[0] = chip->voice_call_mic_mute;

	mutex_unlock(&chip->audio_mutex);
	return 0;
}

static int voice_call_mic_mute_set(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	if (chip->voice_call_mic_mute != ucontrol->value.integer.value[0]) {
		chip->voice_call_mic_mute = ucontrol->value.integer.value[0];
		aoc_voice_call_mic_mute(chip, ucontrol->value.integer.value[0]);
	}

	mutex_unlock(&chip->audio_mutex);
	return 0;
}

static int voice_call_audio_enable_get(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	ucontrol->value.integer.value[0] = chip->voice_call_audio_enable;

	mutex_unlock(&chip->audio_mutex);
	return 0;
}

static int voice_call_audio_enable_set(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	if (chip->voice_call_audio_enable != ucontrol->value.integer.value[0]) {
		chip->voice_call_audio_enable = ucontrol->value.integer.value[0];
	}

	mutex_unlock(&chip->audio_mutex);
	return 0;
}

static int mic_spatial_module_enable_get(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	ucontrol->value.integer.value[0] = chip->mic_spatial_module_enable;

	mutex_unlock(&chip->audio_mutex);
	return 0;
}

static int mic_spatial_module_enable_set(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	chip->mic_spatial_module_enable = ucontrol->value.integer.value[0];

	if (chip->mic_spatial_module_enable)
		aoc_spatial_module_start(chip);
	else
		aoc_spatial_module_stop(chip);

	mutex_unlock(&chip->audio_mutex);
	return 0;
}

static const char *dsp_state_texts[] = { "Idle", "Playback", "Telephony" };

static int aoc_dsp_state_ctl_info(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_info *uinfo)
{
	return snd_ctl_enum_info(uinfo, 1, ARRAY_SIZE(dsp_state_texts),
				 dsp_state_texts);
}

static int aoc_asp_mode_ctl_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);
	struct soc_enum *mc = (struct soc_enum *)kcontrol->private_value;

	u32 block = (u32)mc->shift_l;
	u32 component = (u32)(0x00ff & mc->reg);
	u32 key = (u32)(0xff00 & mc->reg) >> 8;

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	ucontrol->value.enumerated.item[0] =
		aoc_get_asp_mode(chip, block, component, key);

	mutex_unlock(&chip->audio_mutex);
	return 0;
}

static int aoc_asp_mode_ctl_set(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);
	struct soc_enum *mc = (struct soc_enum *)kcontrol->private_value;
	u32 block = (u32)mc->shift_l;
	u32 component = (u32)(0x00ff & mc->reg);
	u32 key = (u32)(0xff00 & mc->reg) >> 8;

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	aoc_set_asp_mode(chip, block, component, key,
			 ucontrol->value.enumerated.item[0]);
	pr_debug("asp mode set: block %d component %d - %d\n", block, component,
		 ucontrol->value.enumerated.item[0]);

	mutex_unlock(&chip->audio_mutex);
	return 0;
}

static int aoc_audio_dsp_mode_ctl_get(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	aoc_get_audio_dsp_mode(chip, (long *)&(ucontrol->value.enumerated.item[0]));

	mutex_unlock(&chip->audio_mutex);
	return 0;
}

static int aoc_audio_dsp_mode_ctl_set(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	aoc_set_audio_dsp_mode(chip, ucontrol->value.enumerated.item[0]);

	pr_debug("Audio dsp mode set: %d", ucontrol->value.enumerated.item[0]);

	mutex_unlock(&chip->audio_mutex);
	return 0;
}

static int
aoc_builtin_mic_process_mode_ctl_get(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	ucontrol->value.enumerated.item[0] =
		aoc_get_builtin_mic_process_mode(chip);

	mutex_unlock(&chip->audio_mutex);
	return 0;
}

static int aoc_builtin_mic_process_mode_ctl_set(struct snd_kcontrol *kcontrol,
						struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);
	int mode, err = 0;

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	mode = ucontrol->value.enumerated.item[0];
	err = aoc_set_builtin_mic_process_mode(chip, mode);
	if (err < 0)
		pr_err("ERR:%d builtin mic process mode set to %d fail", err, mode);

	mutex_unlock(&chip->audio_mutex);
	return 0;
}

static int aoc_sink_mode_ctl_get(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);
	struct soc_enum *mc = (struct soc_enum *)kcontrol->private_value;
	u32 sink_idx = (u32)mc->shift_l;

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	ucontrol->value.enumerated.item[0] = aoc_get_sink_mode(chip, sink_idx);

	mutex_unlock(&chip->audio_mutex);
	return 0;
}

static int aoc_sink_mode_ctl_set(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);
	struct soc_enum *mc = (struct soc_enum *)kcontrol->private_value;
	u32 sink_idx = (u32)mc->shift_l;

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	aoc_set_sink_mode(chip, sink_idx, ucontrol->value.enumerated.item[0]);
	pr_debug("sink mode set: %d - %d\n", sink_idx,
		 ucontrol->value.enumerated.item[0]);

	mutex_unlock(&chip->audio_mutex);
	return 0;
}

static int usb_cfg_ctl_get(struct snd_kcontrol *kcontrol,
			   struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	u32 idx = (u32)mc->shift;
	int val;

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	switch (idx) {
	case USB_DEV_ID:
		val = chip->usb_sink_cfg.dev_id;
		break;
	case USB_TX_EP_ID:
		val = chip->usb_sink_cfg.tx_ep_num;
		break;
	case USB_TX_SR:
		val = chip->usb_sink_cfg.tx_sr;
		break;
	case USB_TX_CH:
		val = chip->usb_sink_cfg.tx_ch;
		break;
	case USB_TX_BW:
		val = chip->usb_sink_cfg.tx_width;
		break;
	case USB_RX_EP_ID:
		val = chip->usb_sink_cfg.rx_ep_num;
		break;
	case USB_RX_SR:
		val = chip->usb_sink_cfg.rx_sr;
		break;
	case USB_RX_CH:
		val = chip->usb_sink_cfg.rx_ch;
		break;
	case USB_RX_BW:
		val = chip->usb_sink_cfg.rx_width;
		break;
	case USB_CFG_TO_AOC:
		val = 0;
		break;
	default:
		val = -1;
		pr_err("ERR: incorrect index for USB config in %s\n", __func__);
	}

	ucontrol->value.enumerated.item[0] = val;

	mutex_unlock(&chip->audio_mutex);
	return 0;
}

static int usb_cfg_ctl_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);
	struct soc_mixer_control *mc = (struct soc_mixer_control *)kcontrol->private_value;
	u32 idx = (u32)mc->shift;
	int err = 0;
	int val = ucontrol->value.enumerated.item[0];

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	switch (idx) {
	case USB_DEV_ID:
		chip->usb_sink_cfg.dev_id = val;
		break;
	case USB_TX_EP_ID:
		chip->usb_sink_cfg.tx_ep_num = val;
		break;
	case USB_TX_SR:
		chip->usb_sink_cfg.tx_sr = val;
		break;
	case USB_TX_CH:
		chip->usb_sink_cfg.tx_ch = val;
		break;
	case USB_TX_BW:
		chip->usb_sink_cfg.tx_width = val;
		break;
	case USB_RX_EP_ID:
		chip->usb_sink_cfg.rx_ep_num = val;
		break;
	case USB_RX_SR:
		chip->usb_sink_cfg.rx_sr = val;
		break;
	case USB_RX_CH:
		chip->usb_sink_cfg.rx_ch = val;
		break;
	case USB_RX_BW:
		chip->usb_sink_cfg.rx_width = val;
		break;
	case USB_CFG_TO_AOC:
		err = aoc_set_usb_config(chip);
		if (err < 0)
			pr_err("ERR:%d fail to update aoc usb config!\n", err);
		break;
	default:
		err = -EINVAL;
		pr_err("ERR: %d incorrect index for USB config\n", err);
	}

	mutex_unlock(&chip->audio_mutex);
	return err;
}

static int usb_cfg_v2_ctl_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	u32 idx = (u32)mc->shift;
	int val;

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	switch (idx) {
	case USB_BUS_ID:
		val = chip->usb_sink_cfg_v2.bus_id;
		break;
	case USB_DEV_ID:
		val = chip->usb_sink_cfg_v2.dev_num;
		break;
	case USB_TX_EP_ID:
		val = chip->usb_sink_cfg_v2.tx_ep_num;
		break;
	case USB_TX_FORMAT:
		val = chip->usb_sink_cfg_v2.tx_format;
		break;
	case USB_TX_SR:
		val = chip->usb_sink_cfg_v2.tx_sr;
		break;
	case USB_TX_CH:
		val = chip->usb_sink_cfg_v2.tx_ch;
		break;
	case USB_TX_BW:
		val = chip->usb_sink_cfg_v2.tx_width;
		break;
	case USB_RX_EP_ID:
		val = chip->usb_sink_cfg_v2.rx_ep_num;
		break;
	case USB_RX_FORMAT:
		val = chip->usb_sink_cfg_v2.rx_format;
		break;
	case USB_RX_SR:
		val = chip->usb_sink_cfg_v2.rx_sr;
		break;
	case USB_RX_CH:
		val = chip->usb_sink_cfg_v2.rx_ch;
		break;
	case USB_RX_BW:
		val = chip->usb_sink_cfg_v2.rx_width;
		break;
	case USB_CFG_TO_AOC:
		val = 0;
		break;
	default:
		val = -1;
		pr_err("ERR: incorrect index for USB config v2 in %s\n", __func__);
	}

	ucontrol->value.enumerated.item[0] = val;

	mutex_unlock(&chip->audio_mutex);
	return 0;
}

static int usb_cfg_v2_ctl_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);
	struct soc_mixer_control *mc = (struct soc_mixer_control *)kcontrol->private_value;
	u32 idx = (u32)mc->shift;
	int err = 0;
	int val = ucontrol->value.enumerated.item[0];

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	switch (idx) {
	case USB_BUS_ID:
		chip->usb_sink_cfg_v2.bus_id = val;
		break;
	case USB_DEV_ID:
		chip->usb_sink_cfg_v2.dev_num = val;
		break;
	case USB_TX_EP_ID:
		chip->usb_sink_cfg_v2.tx_ep_num = val;
		break;
	case USB_TX_FORMAT:
		chip->usb_sink_cfg_v2.tx_format = val;
		break;
	case USB_TX_SR:
		chip->usb_sink_cfg_v2.tx_sr = val;
		break;
	case USB_TX_CH:
		chip->usb_sink_cfg_v2.tx_ch = val;
		break;
	case USB_TX_BW:
		chip->usb_sink_cfg_v2.tx_width = val;
		break;
	case USB_RX_EP_ID:
		chip->usb_sink_cfg_v2.rx_ep_num = val;
		break;
	case USB_RX_FORMAT:
		chip->usb_sink_cfg_v2.rx_format = val;
		break;
	case USB_RX_SR:
		chip->usb_sink_cfg_v2.rx_sr = val;
		break;
	case USB_RX_CH:
		chip->usb_sink_cfg_v2.rx_ch = val;
		break;
	case USB_RX_BW:
		chip->usb_sink_cfg_v2.rx_width = val;
		break;
	case USB_CFG_TO_AOC:
		err = aoc_set_usb_config_v2(chip);
		if (err < 0)
			pr_err("ERR:%d fail to update aoc usb config v2!\n", err);
		break;
	default:
		err = -EINVAL;
		pr_err("ERR: %d incorrect index for USB config v2\n", err);
	}

	mutex_unlock(&chip->audio_mutex);
	return err;
}

static int aoc_dsp_state_ctl_get(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	ucontrol->value.enumerated.item[0] = aoc_get_dsp_state(chip);

	mutex_unlock(&chip->audio_mutex);
	return 0;
}

static int aoc_sink_state_ctl_get(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);
	struct soc_enum *mc = (struct soc_enum *)kcontrol->private_value;
	u32 sink_idx = (u32)mc->shift_l;

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	ucontrol->value.enumerated.item[0] = aoc_get_sink_state(chip, sink_idx);
	pr_debug("sink %d - %d\n", sink_idx,
		 ucontrol->value.enumerated.item[0]);

	mutex_unlock(&chip->audio_mutex);
	return 0;
}

static int aoc_sink_channel_bitmap_ctl_get(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	u32 sink_idx = (u32)mc->shift;

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	ucontrol->value.enumerated.item[0] =
		aoc_get_sink_channel_bitmap(chip, sink_idx);
	pr_debug("sink %d channel bitmap - %d\n", sink_idx,
		 ucontrol->value.enumerated.item[0]);

	mutex_unlock(&chip->audio_mutex);
	return 0;
}

static int aoc_audio_chirp_enable_set(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);
	int err = 0;

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	chip->chirp_enable = ucontrol->value.integer.value[0];
	err = aoc_audio_set_chirp_parameter(chip, AOC_CHIRP_ENABLE_KEY, chip->chirp_enable);

	mutex_unlock(&chip->audio_mutex);
	return err;
}

static int aoc_audio_chirp_enable_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	ucontrol->value.integer.value[0] = chip->chirp_enable;

	mutex_unlock(&chip->audio_mutex);
	return 0;
}

static int aoc_audio_chirp_interval_set(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);
	int err = 0;

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	chip->chirp_interval = ucontrol->value.integer.value[0];
	err = aoc_audio_set_chirp_parameter(chip, AOC_CHIRP_INTERVAL_KEY, chip->chirp_interval);

	mutex_unlock(&chip->audio_mutex);
	return err;
}

static int aoc_audio_chirp_interval_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	ucontrol->value.integer.value[0] = chip->chirp_interval;

	mutex_unlock(&chip->audio_mutex);
	return 0;
}

static int aoc_audio_chirp_mode_set(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);
	int err = 0;

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	chip->chirp_mode = ucontrol->value.integer.value[0];
	err = aoc_audio_set_chirp_parameter(chip, AOC_CHIRP_MODE_KEY, chip->chirp_mode);

	mutex_unlock(&chip->audio_mutex);
	return err;
}

static int aoc_audio_chirp_mode_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	ucontrol->value.integer.value[0] = chip->chirp_mode;

	mutex_unlock(&chip->audio_mutex);
	return 0;
}

static int aoc_audio_chre_src_gain_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol) {
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);
	struct soc_mixer_control *mc = (struct soc_mixer_control *)kcontrol->private_value;
	int chre_path = mc->shift;

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	ucontrol->value.integer.value[0] = chip->chre_src_gain[chre_path];

	mutex_unlock(&chip->audio_mutex);
	return 0;
}

static int aoc_audio_chre_src_gain_set(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);
	struct soc_mixer_control *mc = (struct soc_mixer_control *)kcontrol->private_value;
	int err = 0;

	int chre_path = mc->shift;

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	chip->chre_src_gain[chre_path] = ucontrol->value.integer.value[0];
	switch (chre_path) {
		case CHRE_GAIN_PATH_PDM:
			err = aoc_audio_set_chre_src_pdm_gain(chip, chip->chre_src_gain[chre_path]);
			break;
		case CHRE_GAIN_PATH_AEC:
			err = aoc_audio_set_chre_src_aec_gain(chip, chip->chre_src_gain[chre_path]);
			break;
		default:
			pr_err("Unknown CHRE gain path: %d", chre_path);
			break;
	}

	mutex_unlock(&chip->audio_mutex);
	return err;
}

static int aoc_audio_chre_src_aec_timeout_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol) {
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	ucontrol->value.integer.value[0] = chip->chre_src_aec_timeout;

	mutex_unlock(&chip->audio_mutex);
	return 0;
}

static int aoc_audio_chre_src_aec_timeout_set(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);
	int err = 0;


	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	chip->chre_src_aec_timeout = ucontrol->value.integer.value[0];
	err = aoc_audio_set_chre_src_aec_timeout(chip, chip->chre_src_aec_timeout);

	mutex_unlock(&chip->audio_mutex);
	return err;
}

static int a2dp_encoder_parameters_put(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);
	int para_size = sizeof(chip->a2dp_encoder_cfg);

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	memcpy(&chip->a2dp_encoder_cfg, ucontrol->value.bytes.data, para_size);

	aoc_a2dp_set_enc_param(chip, &chip->a2dp_encoder_cfg);

	mutex_unlock(&chip->audio_mutex);
	return 0;
}

static int a2dp_encoder_parameters_get(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	memcpy(ucontrol->value.bytes.data, &chip->a2dp_encoder_cfg,
		sizeof(chip->a2dp_encoder_cfg));

	mutex_unlock(&chip->audio_mutex);
	return 0;
}

static int us_record_ctl_get(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	return 0;
}

static int us_record_ctl_set(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct aoc_chip *chip = snd_kcontrol_chip(kcontrol);

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	aoc_audio_us_record(chip, ucontrol->value.integer.value[0]);
	mutex_unlock(&chip->audio_mutex);
	return 0;
}

/* TODO: this has to be consistent to enum APMicProcessIndex in aoc-interface.h */
static const char *builtin_mic_process_mode_texts[] = { "Raw", "Spatial" };
static SOC_ENUM_SINGLE_DECL(builtin_mic_process_mode_enum, 1, 0,
			    builtin_mic_process_mode_texts);

/* TODO: this has to be consistent to BT/USB Mode enum in aoc_alsa.h */
static const char *bt_mode_texts[] = { "Unconfigured", "SCO",
				       "ESCO",	       "ESCO_SWB",
				       "A2DP_ENC_SBC", "A2DP_ENC_AAC",
				       "A2DP_ENC_LC3", "BLE_ENC_LC3",
				       "BLE_CONVERSATION", "A2DP_ENC_OPUS",
				       "A2DP_RAW",     "ESCO_LC3" };
static SOC_ENUM_SINGLE_DECL(bt_mode_enum, 1, SINK_BT, bt_mode_texts);

/* TODO: seek better way to create a series of controls  */
static const char *block_asp_mode_texts[] = { "ASP_OFF", "ASP_ON", "ASP_BYPASS",
					      "ASP_GROUND" };
static SOC_ENUM_SINGLE_DECL(block_16_state_enum, 2, 16, block_asp_mode_texts);
static SOC_ENUM_SINGLE_DECL(block_17_state_enum, 2, 17, block_asp_mode_texts);
static SOC_ENUM_SINGLE_DECL(block_18_state_enum, 2, 18, block_asp_mode_texts);
/* ASR mode: block 19, module 15, key 0 for FT*/
static SOC_ENUM_SINGLE_DECL(block_19_state_enum, 15, 19, block_asp_mode_texts);
/* ASR mode: block 19, module 15, key 1 for ASRC*/
static SOC_ENUM_SINGLE_DECL(block_19_15_1_state_enum, (15|(1<<8)), 19, block_asp_mode_texts);
static SOC_ENUM_SINGLE_DECL(block_20_state_enum, 2, 20, block_asp_mode_texts);

/* TODO: seek better way to create a series of controls  */
static const char *sink_processing_state_texts[] = { "Idle", "Active",
						     "Bypass" };
static SOC_ENUM_SINGLE_DECL(sink_0_state_enum, 1, 0,
			    sink_processing_state_texts);
static SOC_ENUM_SINGLE_DECL(sink_1_state_enum, 1, 1,
			    sink_processing_state_texts);
static SOC_ENUM_SINGLE_DECL(sink_2_state_enum, 1, 2,
			    sink_processing_state_texts);
static SOC_ENUM_SINGLE_DECL(sink_3_state_enum, 1, 3,
			    sink_processing_state_texts);
static SOC_ENUM_SINGLE_DECL(sink_4_state_enum, 1, 4,
			    sink_processing_state_texts);

/* audio dsp state switch */
static const char *audio_dsp_state_switch_texts[] = { "Ambient", "Record", "Telephony"};
static SOC_ENUM_SINGLE_DECL(audio_dsp_state_switch_enum, 1, 0, audio_dsp_state_switch_texts);

/* incall capture stream state */
static const char *incall_capture_stream_texts[] = { "Off", "UL", "DL", "UL_DL" };
static SOC_ENUM_SINGLE_DECL(incall_capture_stream0_enum, 1, 0, incall_capture_stream_texts);
static SOC_ENUM_SINGLE_DECL(incall_capture_stream1_enum, 1, 1, incall_capture_stream_texts);
static SOC_ENUM_SINGLE_DECL(incall_capture_stream2_enum, 1, 2, incall_capture_stream_texts);

/* audio capture mic source */
static const char *audio_capture_mic_source_texts[] = { "Default", "Builtin_MIC", "USB_MIC",
						       "BT_MIC", "No MIC", "ERASER" };
static SOC_ENUM_SINGLE_DECL(audio_capture_mic_source_enum, 1, 0, audio_capture_mic_source_texts);

/* Voice call mic source */
static const char *voice_call_mic_source_texts[] = { "Default", "Builtin_MIC", "USB_MIC", "BT_MIC",
						     "IN_CALL_MUSIC" };
static SOC_ENUM_SINGLE_DECL(voice_call_mic_source_enum, 1, 0,
			    voice_call_mic_source_texts);

/* AEC reference source */
static const char *eraser_aec_ref_source_texts[NUM_AEC_REF_SOURCE] = { "Default", "SPEAKER", "USB",
								       "BT" };
static SOC_ENUM_SINGLE_DECL(eraser_aec_ref_source_enum, 1, 0, eraser_aec_ref_source_texts);

static const char *ft_aec_ref_source_texts[NUM_AEC_REF_SOURCE] = { "Default", "SPEAKER", "USB",
								   "BT" };
static SOC_ENUM_SINGLE_DECL(ft_aec_ref_source_enum, 1, 0, ft_aec_ref_source_texts);

static struct snd_kcontrol_new snd_aoc_ctl[] = {
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "PCM Playback Volume",
		.index = 0,
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE |
			  SNDRV_CTL_ELEM_ACCESS_TLV_READ,
		.private_value = PCM_PLAYBACK_VOLUME,
		.info = snd_aoc_ctl_info,
		.get = snd_aoc_ctl_get,
		.put = NULL,
		.count = 1,
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "PCM Playback Switch",
		.index = 0,
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.private_value = PCM_PLAYBACK_MUTE,
		.info = snd_aoc_ctl_info,
		.get = snd_aoc_ctl_get,
		.put = NULL,
		.count = 1,
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "BUILDIN MIC POWER STATE",
		.index = 0,
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.private_value = BUILDIN_MIC_POWER_STATE,
		.info = snd_aoc_ctl_info,
		.get = snd_aoc_buildin_mic_power_ctl_get,
		.put = snd_aoc_buildin_mic_power_ctl_put,
		.count = 1,
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "BUILDIN MIC ID CAPTURE LIST",
		.index = 0,
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.private_value = BUILDIN_MIC_CAPTURE_LIST,
		.info = snd_aoc_ctl_info,
		.get = snd_aoc_buildin_mic_capture_list_ctl_get,
		.put = snd_aoc_buildin_mic_capture_list_ctl_put,
		.count = 1,
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "BUILDIN US MIC ID CAPTURE LIST",
		.index = 0,
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.private_value = BUILDIN_US_MIC_CAPTURE_LIST,
		.info = snd_aoc_ctl_info,
		.get = snd_aoc_buildin_us_mic_capture_list_ctl_get,
		.put = snd_aoc_buildin_us_mic_capture_list_ctl_put,
		.count = 1,
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "Audio DSP State",
		.index = 0,
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info = aoc_dsp_state_ctl_info,
		.get = aoc_dsp_state_ctl_get,
		.count = 1,
	},

	SOC_ENUM_EXT("AoC Speaker Mixer ASP Mode", block_16_state_enum,
		     aoc_asp_mode_ctl_get, aoc_asp_mode_ctl_set),
	SOC_ENUM_EXT("AoC Headphone Mixer ASP Mode", block_17_state_enum,
		     aoc_asp_mode_ctl_get, aoc_asp_mode_ctl_set),
	SOC_ENUM_EXT("AoC BT Mixer ASP Mode", block_18_state_enum,
		     aoc_asp_mode_ctl_get, aoc_asp_mode_ctl_set),
	SOC_ENUM_EXT("AoC Modem Mixer ASP Mode", block_19_state_enum,
		     aoc_asp_mode_ctl_get, aoc_asp_mode_ctl_set),
	SOC_ENUM_EXT("AoC USB Mixer ASP Mode", block_20_state_enum,
		     aoc_asp_mode_ctl_get, aoc_asp_mode_ctl_set),
	SOC_ENUM_EXT("AoC Modem Downlink ASRC Mode", block_19_15_1_state_enum,
		     aoc_asp_mode_ctl_get, aoc_asp_mode_ctl_set),

	SOC_ENUM_EXT("Audio DSP Mode", audio_dsp_state_switch_enum,
		     aoc_audio_dsp_mode_ctl_get, aoc_audio_dsp_mode_ctl_set),


	SOC_ENUM_EXT("BUILTIN MIC Process Mode", builtin_mic_process_mode_enum,
		     aoc_builtin_mic_process_mode_ctl_get, aoc_builtin_mic_process_mode_ctl_set),

	SOC_ENUM_EXT("BT Mode", bt_mode_enum, aoc_sink_mode_ctl_get,
		     aoc_sink_mode_ctl_set),

	SOC_SINGLE_EXT("USB Dev ID", SND_SOC_NOPM, USB_DEV_ID, 100, 0,
		       usb_cfg_ctl_get, usb_cfg_ctl_set),
	SOC_SINGLE_EXT("USB Playback EP ID", SND_SOC_NOPM, USB_TX_EP_ID, 100, 0,
		       usb_cfg_ctl_get, usb_cfg_ctl_set),
	SOC_SINGLE_EXT("USB Playback SR", SND_SOC_NOPM, USB_TX_SR, 48000, 0,
		       usb_cfg_ctl_get, usb_cfg_ctl_set),
	SOC_SINGLE_EXT("USB Playback CH", SND_SOC_NOPM, USB_TX_CH, 2, 0,
		       usb_cfg_ctl_get, usb_cfg_ctl_set),
	SOC_SINGLE_EXT("USB Playback BW", SND_SOC_NOPM, USB_TX_BW, 32, 0,
		       usb_cfg_ctl_get, usb_cfg_ctl_set),
	SOC_SINGLE_EXT("USB Capture EP ID", SND_SOC_NOPM, USB_RX_EP_ID, 100, 0,
		       usb_cfg_ctl_get, usb_cfg_ctl_set),
	SOC_SINGLE_EXT("USB Capture SR", SND_SOC_NOPM, USB_RX_SR, 48000, 0,
		       usb_cfg_ctl_get, usb_cfg_ctl_set),
	SOC_SINGLE_EXT("USB Capture CH", SND_SOC_NOPM, USB_RX_CH, 2, 0,
		       usb_cfg_ctl_get, usb_cfg_ctl_set),
	SOC_SINGLE_EXT("USB Capture BW", SND_SOC_NOPM, USB_RX_BW, 32, 0,
		       usb_cfg_ctl_get, usb_cfg_ctl_set),
	SOC_SINGLE_EXT("USB Config To AoC", SND_SOC_NOPM, USB_CFG_TO_AOC, 1, 0,
		       usb_cfg_ctl_get, usb_cfg_ctl_set),

	SOC_SINGLE_EXT("USB Bus ID v2", SND_SOC_NOPM, USB_BUS_ID, 100, 0,
		       usb_cfg_v2_ctl_get, usb_cfg_v2_ctl_set),
	SOC_SINGLE_EXT("USB Dev ID v2", SND_SOC_NOPM, USB_DEV_ID, 100, 0,
		       usb_cfg_v2_ctl_get, usb_cfg_v2_ctl_set),
	SOC_SINGLE_EXT("USB Playback EP ID v2", SND_SOC_NOPM, USB_TX_EP_ID, 100, 0,
		       usb_cfg_v2_ctl_get, usb_cfg_v2_ctl_set),
	SOC_SINGLE_EXT("USB Playback FORMAT v2", SND_SOC_NOPM, USB_TX_FORMAT, 100, 0,
		       usb_cfg_v2_ctl_get, usb_cfg_v2_ctl_set),
	SOC_SINGLE_EXT("USB Playback SR v2", SND_SOC_NOPM, USB_TX_SR, 48000, 0,
		       usb_cfg_v2_ctl_get, usb_cfg_v2_ctl_set),
	SOC_SINGLE_EXT("USB Playback CH v2", SND_SOC_NOPM, USB_TX_CH, 2, 0,
		       usb_cfg_v2_ctl_get, usb_cfg_v2_ctl_set),
	SOC_SINGLE_EXT("USB Playback BW v2", SND_SOC_NOPM, USB_TX_BW, 32, 0,
		       usb_cfg_v2_ctl_get, usb_cfg_v2_ctl_set),
	SOC_SINGLE_EXT("USB Capture EP ID v2", SND_SOC_NOPM, USB_RX_EP_ID, 100, 0,
		       usb_cfg_v2_ctl_get, usb_cfg_v2_ctl_set),
	SOC_SINGLE_EXT("USB Capture FORMAT v2", SND_SOC_NOPM, USB_RX_FORMAT, 100, 0,
		       usb_cfg_v2_ctl_get, usb_cfg_v2_ctl_set),
	SOC_SINGLE_EXT("USB Capture SR v2", SND_SOC_NOPM, USB_RX_SR, 48000, 0,
		       usb_cfg_v2_ctl_get, usb_cfg_v2_ctl_set),
	SOC_SINGLE_EXT("USB Capture CH v2", SND_SOC_NOPM, USB_RX_CH, 2, 0,
		       usb_cfg_v2_ctl_get, usb_cfg_v2_ctl_set),
	SOC_SINGLE_EXT("USB Capture BW v2", SND_SOC_NOPM, USB_RX_BW, 32, 0,
		       usb_cfg_v2_ctl_get, usb_cfg_v2_ctl_set),
	SOC_SINGLE_EXT("USB Config To AoC v2", SND_SOC_NOPM, USB_CFG_TO_AOC, 1, 0,
		       usb_cfg_v2_ctl_get, usb_cfg_v2_ctl_set),

	SOC_ENUM_EXT("Audio Sink 0 Processing State", sink_0_state_enum,
		     aoc_sink_state_ctl_get, NULL),
	SOC_ENUM_EXT("Audio Sink 1 Processing State", sink_1_state_enum,
		     aoc_sink_state_ctl_get, NULL),
	SOC_ENUM_EXT("Audio Sink 2 Processing State", sink_2_state_enum,
		     aoc_sink_state_ctl_get, NULL),
	SOC_ENUM_EXT("Audio Sink 3 Processing State", sink_3_state_enum,
		     aoc_sink_state_ctl_get, NULL),
	SOC_ENUM_EXT("Audio Sink 4 Processing State", sink_4_state_enum,
		     aoc_sink_state_ctl_get, NULL),

	/* 16 bit for each sink */
	SOC_SINGLE_EXT("AoC Speaker Sink Channel Bitmap", SND_SOC_NOPM, 0,
		       0x00ffff, 0, aoc_sink_channel_bitmap_ctl_get, NULL),
	SOC_SINGLE_EXT("AoC Headphone Sink Channel Bitmap", SND_SOC_NOPM, 1,
		       0x00ffff, 0, aoc_sink_channel_bitmap_ctl_get, NULL),
	SOC_SINGLE_EXT("AoC BT Sink Channel Bitmap", SND_SOC_NOPM, 2, 0x00ffff,
		       0, aoc_sink_channel_bitmap_ctl_get, NULL),
	SOC_SINGLE_EXT("AoC Modem Sink Channel Bitmap", SND_SOC_NOPM, 3,
		       0x00ffff, 0, aoc_sink_channel_bitmap_ctl_get, NULL),
	SOC_SINGLE_EXT("AoC USB Sink Channel Bitmap", SND_SOC_NOPM, 4, 0x00ffff,
		       0, aoc_sink_channel_bitmap_ctl_get, NULL),

	SOC_SINGLE_EXT("Audio Capture Eraser Enable", SND_SOC_NOPM, 0, 1, 0,
		       audio_capture_eraser_enable_ctl_get, audio_capture_eraser_enable_ctl_set),

#if ! IS_ENABLED(CONFIG_SOC_GS101)
	SOC_SINGLE_EXT("Hotword Tap Enable", SND_SOC_NOPM, 0, 1, 0,
		       hotword_tap_enable_ctl_get, hotword_tap_enable_ctl_set),
#endif

	SOC_ENUM_EXT("Audio Capture Mic Source", audio_capture_mic_source_enum,
		     audio_capture_mic_source_get, audio_capture_mic_source_set),

	SOC_ENUM_EXT("Eraser AEC Reference Source", eraser_aec_ref_source_enum,
		     eraser_aec_ref_source_get, eraser_aec_ref_source_set),

	SOC_ENUM_EXT("FT AEC Reference Source", ft_aec_ref_source_enum,
		     ft_aec_ref_source_get, ft_aec_ref_source_set),

	SOC_ENUM_EXT("Voice Call Mic Source", voice_call_mic_source_enum,
		     voice_call_mic_source_get, voice_call_mic_source_set),
	SOC_SINGLE_EXT("Voice Call Mic Mute", SND_SOC_NOPM, 0, 1, 0,
		       voice_call_mic_mute_get, voice_call_mic_mute_set),
	SOC_SINGLE_EXT("Voice Call Audio Enable", SND_SOC_NOPM, 0, 1, 0,
		       voice_call_audio_enable_get,
		       voice_call_audio_enable_set),
	SOC_SINGLE_EXT("Mic Spatial Module Enable", SND_SOC_NOPM, 0, 1, 0,
		       mic_spatial_module_enable_get,
		       mic_spatial_module_enable_set),

	SOC_ENUM_EXT("Incall Capture Stream0", incall_capture_stream0_enum,
		     incall_capture_enable_ctl_get, incall_capture_enable_ctl_set),
	SOC_ENUM_EXT("Incall Capture Stream1", incall_capture_stream1_enum,
		     incall_capture_enable_ctl_get, incall_capture_enable_ctl_set),
	SOC_ENUM_EXT("Incall Capture Stream2", incall_capture_stream2_enum,
		     incall_capture_enable_ctl_get, incall_capture_enable_ctl_set),

	SOC_SINGLE_EXT("Incall Playback Stream0", SND_SOC_NOPM, 0, 1, 0,
		       incall_playback_enable_ctl_get, incall_playback_enable_ctl_set),
	SOC_SINGLE_EXT("Incall Playback Stream1", SND_SOC_NOPM, 1, 1, 0,
		       incall_playback_enable_ctl_get, incall_playback_enable_ctl_set),

	SOC_SINGLE_EXT("MIC0", SND_SOC_NOPM, BUILTIN_MIC0, 1, 0,
		       mic_power_ctl_get, mic_power_ctl_set),
	SOC_SINGLE_EXT("MIC1", SND_SOC_NOPM, BUILTIN_MIC1, 1, 0,
		       mic_power_ctl_get, mic_power_ctl_set),
	SOC_SINGLE_EXT("MIC2", SND_SOC_NOPM, BUILTIN_MIC2, 1, 0,
		       mic_power_ctl_get, mic_power_ctl_set),
	SOC_SINGLE_EXT("MIC3", SND_SOC_NOPM, BUILTIN_MIC3, 1, 0,
		       mic_power_ctl_get, mic_power_ctl_set),

	SOC_SINGLE_EXT("MIC Clock Rate", SND_SOC_NOPM, 0, 20000000, 0,
		       mic_clock_rate_get, NULL),
	SOC_SINGLE_EXT("MIC DC Blocker", SND_SOC_NOPM, 0, 1, 0,
		       mic_dc_blocker_get, mic_dc_blocker_set),

	SOC_SINGLE_RANGE_EXT_TLV_modified(
		"MIC HW Gain At Lower Power Mode (cB)", SND_SOC_NOPM,
		MIC_LOW_POWER_GAIN, MIC_HW_GAIN_IN_CB_MIN,
		MIC_HW_GAIN_IN_CB_MAX, 0, mic_hw_gain_get, mic_hw_gain_set,
		NULL),
	SOC_SINGLE_RANGE_EXT_TLV_modified(
		"MIC HW Gain At High Power Mode (cB)", SND_SOC_NOPM,
		MIC_HIGH_POWER_GAIN, MIC_HW_GAIN_IN_CB_MIN,
		MIC_HW_GAIN_IN_CB_MAX, 0, mic_hw_gain_get, mic_hw_gain_set,
		NULL),
	SOC_SINGLE_RANGE_EXT_TLV_modified(
		"MIC HW Gain (cB)", SND_SOC_NOPM, MIC_CURRENT_GAIN,
		MIC_HW_GAIN_IN_CB_MIN, MIC_HW_GAIN_IN_CB_MAX, 0,
		mic_hw_gain_get, NULL, NULL),

	/* Incall mic and sink mute */
	SOC_SINGLE_EXT("Incall Mic Mute", SND_SOC_NOPM, 0, 1, 0, incall_mic_sink_mute_ctl_get,
		       incall_mic_sink_mute_ctl_set),
	SOC_SINGLE_EXT("Incall Sink Mute", SND_SOC_NOPM, 1, 1, 0, incall_mic_sink_mute_ctl_get,
		       incall_mic_sink_mute_ctl_set),

	/* Incall playback0 and playback1 mic source choice */
	SOC_SINGLE_EXT("Incall Playback0 Mic Channel", SND_SOC_NOPM, 0, 2, 0, incall_playback_mic_channel_ctl_get,
		       incall_playback_mic_channel_ctl_set),
	SOC_SINGLE_EXT("Incall Playback1 Mic Channel", SND_SOC_NOPM, 1, 2, 0, incall_playback_mic_channel_ctl_get,
		       incall_playback_mic_channel_ctl_set),

	/* UltraSonic Record enable */
	SOC_SINGLE_EXT("US Record Enable", SND_SOC_NOPM, 0, 1, 0,
		       us_record_ctl_get, us_record_ctl_set),


	/* LVM enable 1/0 for comp offload */
	SOC_SINGLE_EXT("LVM Enable", SND_SOC_NOPM, 0, 1, 0,
		       lvm_enable_ctl_get, lvm_enable_ctl_set),
	SOC_SINGLE_EXT("Decoder Reference Enable", SND_SOC_NOPM, 0, 1, 0,
		       decoder_ref_enable_ctl_get, decoder_ref_enable_ctl_set),

	/* Sidetone switch and tuning parameters */
	SOC_SINGLE_EXT("Sidetone Enable", SND_SOC_NOPM, 0, 1, 0,
		       sidetone_enable_ctl_get, sidetone_enable_ctl_set),

	SOC_SINGLE_RANGE_EXT_TLV_modified("Sidetone Volume", SND_SOC_NOPM, SIDETONE_CFG_VOL,
					  SIDETONE_VOL_MIN, SIDETONE_VOL_MAX, 0,
					  aoc_sidetone_cfg_ctl_get, aoc_sidetone_cfg_ctl_set, NULL),
	SOC_SINGLE_RANGE_EXT_TLV_modified(
		"Sidetone Selected Mic", SND_SOC_NOPM, SIDETONE_CFG_MIC_ID, SIDETONE_MIC_ID_MIN,
		SIDETONE_MIC_ID_MAX, 0, aoc_sidetone_cfg_ctl_get, aoc_sidetone_cfg_ctl_set, NULL),

	SOC_SINGLE_RANGE_EXT_TLV_modified("Sidetone EQ Stage Number", SND_SOC_NOPM,
					  SIDETONE_CFG_STAGE_NUM, SIDETONE_EQ_STAGE_NUM_MIN,
					  SIDETONE_EQ_STAGE_NUM_MAX, 0, aoc_sidetone_cfg_ctl_get,
					  aoc_sidetone_cfg_ctl_set, NULL),

	SOC_SINGLE_RANGE_EXT_TLV_modified("Sidetone Biquad0", SND_SOC_NOPM, BIQUAD0,
					  SIDETONE_BIQUAD_PARAM_MIN, SIDETONE_BIQUAD_PARAM_MAX,
					  SIDETONE_BIQUAD_PARAM_NUM, aoc_sidetone_eq_ctl_get,
					  aoc_sidetone_eq_ctl_set, NULL),
	SOC_SINGLE_RANGE_EXT_TLV_modified("Sidetone Biquad1", SND_SOC_NOPM, BIQUAD1,
					  SIDETONE_BIQUAD_PARAM_MIN, SIDETONE_BIQUAD_PARAM_MAX,
					  SIDETONE_BIQUAD_PARAM_NUM, aoc_sidetone_eq_ctl_get,
					  aoc_sidetone_eq_ctl_set, NULL),
	SOC_SINGLE_RANGE_EXT_TLV_modified("Sidetone Biquad2", SND_SOC_NOPM, BIQUAD2,
					  SIDETONE_BIQUAD_PARAM_MIN, SIDETONE_BIQUAD_PARAM_MAX,
					  SIDETONE_BIQUAD_PARAM_NUM, aoc_sidetone_eq_ctl_get,
					  aoc_sidetone_eq_ctl_set, NULL),
	SOC_SINGLE_RANGE_EXT_TLV_modified("Sidetone Biquad3", SND_SOC_NOPM, BIQUAD3,
					  SIDETONE_BIQUAD_PARAM_MIN, SIDETONE_BIQUAD_PARAM_MAX,
					  SIDETONE_BIQUAD_PARAM_NUM, aoc_sidetone_eq_ctl_get,
					  aoc_sidetone_eq_ctl_set, NULL),
	SOC_SINGLE_RANGE_EXT_TLV_modified("Sidetone Biquad4", SND_SOC_NOPM, BIQUAD4,
					  SIDETONE_BIQUAD_PARAM_MIN, SIDETONE_BIQUAD_PARAM_MAX,
					  SIDETONE_BIQUAD_PARAM_NUM, aoc_sidetone_eq_ctl_get,
					  aoc_sidetone_eq_ctl_set, NULL),

	SOC_SINGLE_RANGE_EXT_TLV_modified("MIC Record Soft Gain (dB)", SND_SOC_NOPM, 0,
					  AOC_MIC_RECORD_GAIN_IN_DB_MIN,
					  AOC_MIC_RECORD_GAIN_IN_DB_MAX, 1, mic_record_gain_ctl_get,
					  mic_record_gain_ctl_set, NULL),

	SOC_SINGLE_RANGE_EXT_TLV_modified("Mmap Record Gain", SND_SOC_NOPM, 0,
					  MMAP_MIC_RECORD_GAIN_IN_DB_MIN,
					  MMAP_MIC_RECORD_GAIN_IN_DB_MAX, 1,
					  mmap_record_gain_ctl_get,
					  mmap_record_gain_ctl_set, NULL),

	SOC_SINGLE_EXT("Compress Offload Volume", SND_SOC_NOPM, 0, 100, 0, compr_offload_volume_get,
		       compr_offload_volume_set),

	SOC_SINGLE_RANGE_EXT_TLV_modified("Compress Offload Gain (L and R)", SND_SOC_NOPM, 0,
					  COMPRE_OFFLOAD_GAIN_MIN, COMPRE_OFFLOAD_GAIN_MAX, 2,
					  aoc_compr_offload_gain_ctl_get,
					  aoc_compr_offload_gain_ctl_set, NULL),

	SOC_SINGLE_EXT("Voice Call Rx Volume", SND_SOC_NOPM, 0, 100, 0, NULL,
		       NULL),
	SOC_SINGLE_EXT("VOIP Rx Volume", SND_SOC_NOPM, 0, 100, 0, NULL, NULL),

	SOC_SINGLE_EXT("PCM Stream Wait Time in MSec", SND_SOC_NOPM, 0, 1000000, 0, pcm_wait_time_get,
		       pcm_wait_time_set),

	SOC_SINGLE_EXT("CCA Module Load", SND_SOC_NOPM, 0, 1, 0,
		       audio_cca_module_load_ctl_get, audio_cca_module_load_ctl_set),

	SOC_SINGLE_EXT("Gapless Offload Enable", SND_SOC_NOPM, 0, 1, 0,
		       audio_gapless_offload_ctl_get, audio_gapless_offload_ctl_set),

	SOC_SINGLE_EXT("Voice PCM Stream Wait Time in MSec", SND_SOC_NOPM, 0, 10000, 0,
		voice_pcm_wait_time_get, voice_pcm_wait_time_set),

	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "A2DP Encoder Parameters",
		.index = 0,
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.private_value = A2DP_ENCODER_PARAMETERS,
		.info = snd_aoc_ctl_info,
		.get = a2dp_encoder_parameters_get,
		.put = a2dp_encoder_parameters_put,
		.count = 1,
	},
	SOC_SINGLE_EXT("AoC Chirp Enable", SND_SOC_NOPM, 0, 1, 0,
		       aoc_audio_chirp_enable_get, aoc_audio_chirp_enable_set),
	SOC_SINGLE_RANGE_EXT_TLV_modified("AoC Chirp Interval", SND_SOC_NOPM,
		0, 20, 200, 0, aoc_audio_chirp_interval_get, aoc_audio_chirp_interval_set, NULL),
	SOC_SINGLE_RANGE_EXT_TLV_modified("AoC Chirp Mode", SND_SOC_NOPM,
		0, 0, 10, 0, aoc_audio_chirp_mode_get, aoc_audio_chirp_mode_set, NULL),
	SOC_SINGLE_RANGE_EXT_TLV_modified("CHRE SRC PDM Gain (cB)", SND_SOC_NOPM,
		CHRE_GAIN_PATH_PDM, -1280, 1280, 0, aoc_audio_chre_src_gain_get, aoc_audio_chre_src_gain_set, NULL),
	SOC_SINGLE_RANGE_EXT_TLV_modified("CHRE SRC AEC Gain (cB)", SND_SOC_NOPM,
		CHRE_GAIN_PATH_AEC, -1280, 1280, 0, aoc_audio_chre_src_gain_get, aoc_audio_chre_src_gain_set, NULL),
	SOC_SINGLE_RANGE_EXT_TLV_modified("CHRE SRC AEC Timeout in MSec", SND_SOC_NOPM,
		0, 0, 60000, 0, aoc_audio_chre_src_aec_timeout_get, aoc_audio_chre_src_aec_timeout_set, NULL),
};

int snd_aoc_new_ctl(struct aoc_chip *chip)
{
	int err;
	unsigned int idx;

	strcpy(chip->card->mixername, "Aoc Mixer");
	for (idx = 0; idx < ARRAY_SIZE(snd_aoc_ctl); idx++) {
		err = snd_ctl_add(chip->card, snd_ctl_new1(&snd_aoc_ctl[idx], chip));
		if (err < 0)
			return err;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(snd_aoc_new_ctl);
