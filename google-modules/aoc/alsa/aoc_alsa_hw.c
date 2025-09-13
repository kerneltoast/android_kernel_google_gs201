// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google Whitechapel AoC ALSA Driver on  AoC Audio Control
 *
 * Copyright (c) 2019 Google LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <soc/google/modem_notifier.h>

#include "aoc_alsa.h"
#include "aoc_alsa_drv.h"
#include "aoc_alsa_path.h"

#ifndef ALSA_AOC_CMD_LOG_DISABLE
static int cmd_count;
#endif

#define DEFAULT_TELEPHONY_MIC PORT_INCALL_TX

#define AOC_CHIRP_BLOCK 9
#define AOC_ATRIGGER_BLOCK 23

extern struct be_path_cache port_array[PORT_MAX];

/*
 * TODO: TDM/I2S will be removed from port naming and will be replaced
 * by sink-associated devices such as spker, headphone, bt, usb, mode
 */
static int aoc_audio_sink[] = {
#if IS_ENABLED(CONFIG_SOC_GS101)
	[PORT_I2S_0_RX] = SINK_HEADPHONE, [PORT_I2S_0_TX] = -1,
#else
	[PORT_I2S_0_RX] = SINK_UNUSED, [PORT_I2S_0_TX] = -1,
#endif
	[PORT_I2S_1_RX] = SINK_BT,        [PORT_I2S_1_TX] = -1,
	[PORT_I2S_2_RX] = SINK_USB,       [PORT_I2S_2_TX] = -1,
	[PORT_TDM_0_RX] = SINK_SPEAKER,   [PORT_TDM_0_TX] = -1,
	[PORT_TDM_1_RX] = SINK_MODEM,     [PORT_TDM_1_TX] = -1,
	[PORT_USB_RX] = SINK_USB,         [PORT_USB_TX] = -1,
	[PORT_BT_RX] = SINK_BT,           [PORT_BT_TX] = -1,
	[PORT_INCALL_RX] = -1,            [PORT_INCALL_TX] = -1,
	[PORT_INTERNAL_MIC] = -1,	  [PORT_HAPTIC_RX] = SINK_SPEAKER,
	[PORT_INTERNAL_MIC_US] = -1,      [PORT_DP_DMA_RX] = SINK_USB,
};

static int hw_id_to_sink(int hw_idx)
{
	if (hw_idx < 0 || hw_idx >= ARRAY_SIZE(aoc_audio_sink))
		return -1;

	return aoc_audio_sink[hw_idx];
}

static int ep_id_to_source(int ep_idx)
{
	/* TODO: refactor needed. Haptics pcm dev id: 7, its entrypoint is 10(HAPTICS) */
	switch (ep_idx) {
	case IDX_EP8:
		return HAPTICS;
	case IDX_HIFI:
		return USB_HIFI;
	case IDX_US:
		return SPEAKER_US;
	case IDX_IMSV:
		return IMMERSIVE;
	default:
		return ep_idx;
	}
}

static bool ap_filter_capture_stream(struct aoc_alsa_stream *alsa_stream)
{
	switch (alsa_stream->idx) {
	case UC_AUDIO_RECORD:
	case UC_MMAP_RECORD:
	case UC_LOW_LATENCY_AUDIO_RECORD:
		return true;
	default:
		return false;
	}
}

static struct aoc_alsa_stream *find_alsa_stream_by_ep_id(struct aoc_chip *chip, int ep_id)
{
	int i;
	struct snd_soc_pcm_runtime *rtd;

	for (i = 0; i < ARRAY_SIZE(chip->alsa_stream); i++) {
		if (!chip->alsa_stream[i])
			continue;

		if (!chip->alsa_stream[i]->substream)
			continue;

		rtd = chip->alsa_stream[i]->substream->private_data;
		if (rtd && rtd->dai_link->id == ep_id)
			return chip->alsa_stream[i];
	}

	return NULL;
}

static struct aoc_alsa_stream *find_alsa_stream_by_device_idx(struct aoc_chip *chip, int idx)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(chip->alsa_stream); i++) {
		if (!chip->alsa_stream[i])
			continue;

		if (chip->alsa_stream[i]->idx == idx)
			return chip->alsa_stream[i];
	}

	return NULL;
}

static struct aoc_alsa_stream *find_alsa_stream_by_capture_stream(struct aoc_chip *chip)
{
	int i;

	if (aoc_audio_capture_active_stream_num(chip) == 0)
		return NULL;

	for (i = 0; i < ARRAY_SIZE(chip->alsa_stream); i++) {
		struct aoc_alsa_stream *alsa_stream = chip->alsa_stream[i];
		if (!alsa_stream)
			continue;

		if (!alsa_stream->substream)
			continue;

		if (alsa_stream->substream->stream == SNDRV_PCM_STREAM_CAPTURE)
			return chip->alsa_stream[i];
	}

	return NULL;
}

static int hw_id_to_phone_mic_source(int hw_id)
{
	int mic_input_source;

	/* Use user-specified mic for voice call independently of the playback sink */
	switch (hw_id) {
	case PORT_USB_TX:
		mic_input_source = MODEM_USB_INPUT_INDEX;
		break;
	case PORT_BT_TX:
		mic_input_source = MODEM_BT_INPUT_INDEX;
		break;
	case PORT_INCALL_TX:
		mic_input_source = MODEM_INCALL_INPUT_INDEX;
		break;
	default:
		pr_err("ERR in mic input source for voice call, mic source=%d\n",
			hw_id);
		fallthrough;
	case PORT_INTERNAL_MIC:
		mic_input_source = MODEM_MIC_INPUT_INDEX;
		break;
	}
	return mic_input_source;
}

/* temp usage */
static int aoc_audio_stream_type[] = {
	[0] = MMAPED,  [1] = NORMAL,   [2] = NORMAL,	   [3] = NORMAL,  [4] = NORMAL,
	[5] = NORMAL,  [6] = COMPRESS, [7] = NORMAL,	   [8] = NORMAL,  [9] = MMAPED,
	[10] = RAW,    [11] = NORMAL,  [12] = NORMAL,	   [13] = NORMAL, [14] = NORMAL,
	[15] = NORMAL, [16] = NORMAL,  [17] = NORMAL,	   [18] = INCALL, [19] = INCALL,
	[20] = INCALL, [21] = INCALL,  [22] = INCALL,	   [23] = MMAPED, [24] = NORMAL,
	[25] = HIFI,   [26] = HIFI,    [27] = ANDROID_AEC, [28] = MMAPED, [29] = INCALL,
	[30] = NORMAL, [31] = CAP_INJ, [32] = HOTWORD_TAP, [54] = INCALL,
};

int aoc_pcm_device_to_stream_type(int device)
{
	if (device < 0 || device >= ARRAY_SIZE(aoc_audio_stream_type))
		return -1;

	return aoc_audio_stream_type[device];
}

static bool aoc_pcm_is_mmap_raw(struct aoc_alsa_stream *alsa_stream)
{
	return (alsa_stream->stream_type == MMAPED || alsa_stream->stream_type == RAW);
}
/*
 * Sending commands to AoC for setting parameters and start/stop the streams
 */
static int aoc_audio_control(const char *cmd_channel, const uint8_t *cmd,
			     size_t cmd_size, uint8_t *response,
			     struct aoc_chip *chip)
{
	struct aoc_service_dev *dev;
	uint8_t *buffer;
	int buffer_size = 1024;
	struct timespec64 tv0, tv1;
	int err, count;
	unsigned long time_expired;

	if (!cmd_channel || !cmd)
		return -EINVAL;

	if (mutex_lock_interruptible(&chip->audio_cmd_chan_mutex))
		return -EINTR;

	/* Get the aoc audio control channel at runtime */
	err = alloc_aoc_audio_service(cmd_channel, &dev, NULL, NULL);
	if (err < 0) {
		mutex_unlock(&chip->audio_cmd_chan_mutex);
		return err;
	}

#ifndef ALSA_AOC_CMD_LOG_DISABLE
	cmd_count++;
	pr_notice_ratelimited(ALSA_AOC_CMD
			      " cmd [%s] id %#06x, size %zu, cntr %d\n",
			      CMD_CHANNEL(dev), ((struct CMD_HDR *)cmd)->id,
			      cmd_size, cmd_count);
#endif

	buffer = kmalloc_array(buffer_size, sizeof(*buffer), GFP_ATOMIC);
	if (!buffer) {
		err = -ENOMEM;
		pr_err("ERR: no memory!\n");
		goto exit;
	}

	/*
	 * TODO: assuming only one user for the audio control channel.
	 * clear all the response messages from previous commands
	 */
	count = 0;
	while (((err = aoc_service_read(dev, buffer, buffer_size,
					NONBLOCKING)) >= 1)) {
		count++;
	}
	if (count > 0)
		pr_debug("%d messages read for previous commands\n", count);

	/* Sending cmd to AoC */
	if ((aoc_service_write(dev, cmd, cmd_size, NONBLOCKING)) != cmd_size) {
		pr_err(ALSA_AOC_CMD " ERR: ring full - cmd id %#06x\n",
		       ((struct CMD_HDR *)cmd)->id);
		err = -EAGAIN;
		goto exit;
	}

#ifdef AOC_CMD_DEBUG_ENABLE
	ktime_get_real_ts64(&tv0);
#endif /* AOC_CMD_DEBUG_ENABLE */

	/* Getting responses from Aoc for the command just sent */
	count = 0;
	time_expired = jiffies + msecs_to_jiffies(WAITING_TIME_MS);
	while (((err = aoc_service_read(dev, buffer, buffer_size,
					NONBLOCKING)) < 1) &&
	       time_is_after_jiffies(time_expired)) {
		count++;
	}

#ifdef AOC_CMD_DEBUG_ENABLE
	ktime_get_real_ts64(&tv1);
	pr_debug("Elapsed: %lld (usecs)\n",
		 (tv1.tv_sec - tv0.tv_sec) * USEC_PER_SEC +
			 (tv1.tv_nsec - tv0.tv_nsec) / NSEC_PER_USEC);

	if (count > 0)
		pr_debug("%d times tried for response\n", count);
#endif /* AOC_CMD_DEBUG_ENABLE */

	if (err < 1) {
		uint16_t cmd_id = ((struct CMD_HDR *)cmd)->id;
		char reset_reason[40];

		scnprintf(reset_reason, sizeof(reset_reason), "ALSA command timeout %#06x",
			cmd_id);
		pr_err(ALSA_AOC_CMD " ERR:timeout - cmd [%s] id %#06x\n",
		       CMD_CHANNEL(dev), cmd_id);
		print_hex_dump(KERN_ERR, ALSA_AOC_CMD " :mem ",
			       DUMP_PREFIX_OFFSET, 16, 1, cmd, cmd_size, false);
		aoc_trigger_watchdog(reset_reason);
	} else if (err == 4) {
		pr_err(ALSA_AOC_CMD " ERR:%#x - cmd [%s] id %#06x\n",
		       *(uint32_t *)buffer, CMD_CHANNEL(dev),
		       ((struct CMD_HDR *)cmd)->id);
		print_hex_dump(KERN_ERR, ALSA_AOC_CMD " :mem ",
			       DUMP_PREFIX_OFFSET, 16, 1, cmd, cmd_size, false);
	} else {
		pr_debug(ALSA_AOC_CMD
			 " cmd [%s] id %#06x, reply mesg size %d\n",
			 CMD_CHANNEL(dev), ((struct CMD_HDR *)cmd)->id, err);
	}

	if (response != NULL)
		memcpy(response, buffer, cmd_size);

exit:
	kfree(buffer);
	free_aoc_audio_service(cmd_channel, dev);

	mutex_unlock(&chip->audio_cmd_chan_mutex);

	return err < 1 ? -EAGAIN : 0;
}

static int aoc_audio_control_simple_cmd(const char *cmd_channel, int cmd_id, struct aoc_chip *chip)
{
	int err = 0;
	struct CMD_HDR cmd;

	AocCmdHdrSet(&cmd, cmd_id, sizeof(cmd));

	err = aoc_audio_control(cmd_channel, (uint8_t *)&cmd, sizeof(cmd), NULL, chip);
	if (err < 0) {
		pr_err("ERR:%d in audio control simple cmd:%d sent\n", err, cmd_id);
		return err;
	}

	return 0;
}

int aoc_audio_volume_set(struct aoc_chip *chip, uint32_t volume, int src,
			 int dst)
{
	int err;
	struct CMD_AUDIO_OUTPUT_SET_PARAMETER cmd;

	/* No volume control for capturing */
	/* Haptics in AoC does not have adjustable volume */
	if (src == HAPTICS)
		return 0;

	AocCmdHdrSet(&(cmd.parent), CMD_AUDIO_OUTPUT_SET_PARAMETER_ID,
		     sizeof(cmd));

	/* Sink 0-4 with block ID from 16-20 */
	cmd.block = dst + AOC_AUDIO_SINK_BLOCK_ID_BASE;
	cmd.component = 0;
	cmd.key = src;
	cmd.val = volume;
	pr_debug("volume changed to: %d\n", volume);

	/* Send cmd to AOC */
	err = aoc_audio_control(CMD_OUTPUT_CHANNEL, (uint8_t *)&cmd,
				sizeof(cmd), NULL, chip);
	if (err < 0)
		pr_err("ERR:%d in volume set\n", err);

	return err;
}

static int aoc_audio_capture_mic_input(struct aoc_chip *chip,
				       int input_cmd, int mic_input_source)
{
	int err;
	struct CMD_AUDIO_INPUT_AP_INPUT_START cmd1;

	if (input_cmd == START) {
		AocCmdHdrSet(&(cmd1.parent), CMD_AUDIO_INPUT_AP_INPUT_START_ID,
			     sizeof(cmd1));

		cmd1.mic_input_source = mic_input_source;
		err = aoc_audio_control(CMD_INPUT_CHANNEL, (uint8_t *)&cmd1, sizeof(cmd1),
			NULL, chip);
		if (err < 0)
			pr_err("ERR:%d audio capture mic input start fail!\n", err);

	} else {
		err = aoc_audio_control_simple_cmd(CMD_INPUT_CHANNEL,
			CMD_AUDIO_INPUT_AP_INPUT_STOP_ID, chip);
		if (err < 0)
			pr_err("ERR:%d audio capture mic input stop fail!\n", err);
	}

	return err;
}

static int aoc_audio_us_capture_mic_input(struct aoc_chip *chip, int input_cmd)
{
	int err;

	if (input_cmd == START) {
		err = aoc_audio_control_simple_cmd(
			CMD_INPUT_CHANNEL, CMD_AUDIO_INPUT_ULTRASONIC_CAPTURE_START_ID,
			chip);
		if (err < 0)
			pr_err("ERR:%d audio us capture mic input start fail!\n", err);

	} else {
		err = aoc_audio_control_simple_cmd(CMD_INPUT_CHANNEL,
			CMD_AUDIO_INPUT_ULTRASONIC_CAPTURE_STOP_ID, chip);
		if (err < 0)
			pr_err("ERR:%d audio us capture mic input stop fail!\n", err);
	}

	return err;
}

int aoc_audio_capture_mic_prepare(struct aoc_chip *chip)
{
	int err = 0;
	int mic_input_source;

	switch (chip->audio_capture_mic_source) {
	case BUILTIN_MIC:
		mic_input_source = AP_INPUT_PROCESSOR_MIC_INPUT_INDEX;
		break;
	case USB_MIC:
		mic_input_source = AP_INPUT_PROCESSOR_USB_INPUT_INDEX;
		break;
	case BT_MIC:
		mic_input_source = AP_INPUT_PROCESSOR_BT_INPUT_INDEX;
		break;
	case ERASER:
		mic_input_source = AP_INPUT_PROCESSOR_ERASER_INPUT_INDEX;
		break;
	default:
		pr_err("ERR in mic input source for audio capture mic source=%d\n",
		       chip->audio_capture_mic_source);
		err = EINVAL;
		goto exit;
	}

	/* Update mask before start capture */
	if (mic_input_source == AP_INPUT_PROCESSOR_MIC_INPUT_INDEX)
		aoc_audio_mic_mask_set(chip, false);

	// CMD_AUDIO_INPUT_AP_INPUT_START_ID with mic_input_source
	pr_info("mic_input_source = %d\n", mic_input_source);

	err = aoc_audio_capture_mic_input(chip, START, mic_input_source);
	if (err < 0)
		pr_err("ERR:%d in audio capture mic input setup start\n", err);

exit:
	return err;
}

int aoc_audio_capture_mic_close(struct aoc_chip *chip)
{
	int err = 0;

	err = aoc_audio_capture_mic_input(chip, STOP, 0);
	if (err < 0) {
		pr_err("ERR:%d in audio capture mic input stop\n", err);
		return err;
	}

	return 0;
}

static int aoc_audio_us_capture_mic_prepare(struct aoc_chip *chip)
{
	int err = 0;

	err = aoc_audio_us_capture_mic_input(chip, START);
	if (err < 0)
		pr_err("ERR:%d in audio us capture mic input setup start\n", err);

	return err;
}

static int aoc_audio_us_capture_mic_close(struct aoc_chip *chip)
{
	int err = 0;

	err = aoc_audio_us_capture_mic_input(chip, STOP);
	if (err < 0) {
		pr_err("ERR:%d in audio us capture mic input stop\n", err);
		return err;
	}

	return 0;
}

int aoc_audio_capture_active_stream_num(struct aoc_chip *chip)
{
	return (int)hweight64((uint64_t)(chip->opened & AOC_AUDIO_CAPUTRE_DEVICE_MASK));
}

static int aoc_record_param_configured_num(struct aoc_chip *chip)
{
	return (int)hweight64((uint64_t)(chip->capture_param_set & AOC_AUDIO_CAPUTRE_DEVICE_MASK));
}

static int aoc_us_record_param_configured_num(struct aoc_chip *chip)
{
	return (int)hweight64((uint64_t)(chip->capture_param_set & AOC_ULTRASONIC_CAPUTRE_DEVICE_MASK));
}

int aoc_set_builtin_mic_power_state(struct aoc_chip *chip, int iMic, int state)
{
	int err;
	struct CMD_AUDIO_INPUT_MIC_POWER_ON cmd_on;
	struct CMD_AUDIO_INPUT_MIC_POWER_OFF cmd_off;

	if (state == 1) {
		AocCmdHdrSet(&(cmd_on.parent), CMD_AUDIO_INPUT_MIC_POWER_ON_ID,
			     sizeof(cmd_on));
		cmd_on.mic_index = iMic;
		err = aoc_audio_control(CMD_INPUT_CHANNEL, (uint8_t *)&cmd_on,
					sizeof(cmd_on), NULL, chip);
	} else {
		AocCmdHdrSet(&(cmd_off.parent),
			     CMD_AUDIO_INPUT_MIC_POWER_OFF_ID, sizeof(cmd_off));
		cmd_off.mic_index = iMic;
		err = aoc_audio_control(CMD_INPUT_CHANNEL, (uint8_t *)&cmd_off,
					sizeof(cmd_off), NULL, chip);
	}

	if (err < 0)
		pr_err("ERR:%d in set mic state\n", err);

	return err < 0 ? err : 0;
}

int aoc_get_builtin_mic_power_state(struct aoc_chip *chip, int iMic)
{
	int err;
	struct CMD_AUDIO_INPUT_MIC_GET_POWER_STATE cmd;

	AocCmdHdrSet(&(cmd.parent), CMD_AUDIO_INPUT_MIC_GET_POWER_STATE_ID,
		     sizeof(cmd));

	cmd.mic_index = iMic;

	err = aoc_audio_control(CMD_INPUT_CHANNEL, (uint8_t *)&cmd, sizeof(cmd),
				(uint8_t *)&cmd, chip);
	if (err < 0)
		pr_err("ERR:%d in get mic state\n", err);

	return err < 0 ? err : cmd.power_state;
}

int aoc_mic_clock_rate_get(struct aoc_chip *chip)
{
	int err;
	struct CMD_AUDIO_INPUT_GET_MIC_CLOCK_FREQUENCY cmd;

	AocCmdHdrSet(&(cmd.parent), CMD_AUDIO_INPUT_GET_MIC_CLOCK_FREQUENCY_ID,
		     sizeof(cmd));

	err = aoc_audio_control(CMD_INPUT_CHANNEL, (uint8_t *)&cmd, sizeof(cmd),
				(uint8_t *)&cmd, chip);
	if (err < 0) {
		pr_err("ERR:%d in get mic clock frequency\n", err);
		return err;
	}

	return cmd.mic_clock_frequency_hz;
}

int aoc_mic_hw_gain_get(struct aoc_chip *chip, int state)
{
	int err;
	/* TODO: the cmd structures for 3 states differ only in names */
	struct CMD_AUDIO_INPUT_GET_MIC_CURRENT_HW_GAIN cmd;
	int cmd_id;

	switch (state) {
	case MIC_LOW_POWER_GAIN:
		cmd_id = CMD_AUDIO_INPUT_GET_MIC_LOW_POWER_HW_GAIN_ID;
		break;
	case MIC_HIGH_POWER_GAIN:
		cmd_id = CMD_AUDIO_INPUT_GET_MIC_HIGH_POWER_HW_GAIN_ID;
		break;
	default:
		cmd_id = CMD_AUDIO_INPUT_GET_MIC_CURRENT_HW_GAIN_ID;
	}

	AocCmdHdrSet(&(cmd.parent), cmd_id, sizeof(cmd));

	err = aoc_audio_control(CMD_INPUT_CHANNEL, (uint8_t *)&cmd, sizeof(cmd),
				(uint8_t *)&cmd, chip);
	if (err < 0) {
		pr_err("ERR:%d in get current mic hw gain\n", err);
		return err;
	}

	return cmd.mic_hw_gain_cb;
}

int aoc_mic_hw_gain_set(struct aoc_chip *chip, int state, int gain)
{
	int err;
	/* TODO: the cmd structures for 3 states differ only in names */
	struct CMD_AUDIO_INPUT_SET_MIC_LOW_POWER_HW_GAIN cmd;
	int cmd_id;

	switch (state) {
	case MIC_LOW_POWER_GAIN:
		cmd_id = CMD_AUDIO_INPUT_SET_MIC_LOW_POWER_HW_GAIN_ID;
		break;
	case MIC_HIGH_POWER_GAIN:
		cmd_id = CMD_AUDIO_INPUT_SET_MIC_HIGH_POWER_HW_GAIN_ID;
		break;
	default:
		cmd_id = CMD_AUDIO_INPUT_SET_MIC_LOW_POWER_HW_GAIN_ID;
	}

	AocCmdHdrSet(&(cmd.parent), cmd_id, sizeof(cmd));
	cmd.mic_hw_gain_cb = gain;
	pr_debug("power state =%d, gain = %d\n", state, cmd.mic_hw_gain_cb);

	err = aoc_audio_control(CMD_INPUT_CHANNEL, (uint8_t *)&cmd, sizeof(cmd),
				(uint8_t *)&cmd, chip);
	if (err < 0) {
		pr_err("ERR:%d in set mic hw gain\n", err);
		return err;
	}

	return 0;
}

int aoc_mic_dc_blocker_get(struct aoc_chip *chip)
{
	int err;
	struct CMD_AUDIO_INPUT_GET_MIC_DC_BLOCKER cmd;

	AocCmdHdrSet(&(cmd.parent), CMD_AUDIO_INPUT_GET_MIC_DC_BLOCKER_ID,
		     sizeof(cmd));

	err = aoc_audio_control(CMD_INPUT_CHANNEL, (uint8_t *)&cmd, sizeof(cmd),
				(uint8_t *)&cmd, chip);
	if (err < 0) {
		pr_err("ERR:%d in get mic dc blocker state\n", err);
		return err;
	}

	return cmd.dc_blocker_enabled;
}

int aoc_mic_dc_blocker_set(struct aoc_chip *chip, int enable)
{
	int err;
	struct CMD_AUDIO_INPUT_SET_MIC_DC_BLOCKER cmd;

	AocCmdHdrSet(&(cmd.parent), CMD_AUDIO_INPUT_SET_MIC_DC_BLOCKER_ID,
		     sizeof(cmd));
	cmd.dc_blocker_enabled = enable;

	err = aoc_audio_control(CMD_INPUT_CHANNEL, (uint8_t *)&cmd, sizeof(cmd),
				NULL, chip);
	if (err < 0)
		pr_err("ERR:%d in set mic dc blocker state as %d\n", err,
		       enable);

	return err;
}

int aoc_sidetone_cfg_get(struct aoc_chip *chip, int param, long *val)
{
	int err = 0;
	struct CMD_AUDIO_OUTPUT_GET_SIDETONE cmd;

	AocCmdHdrSet(&(cmd.parent), CMD_AUDIO_OUTPUT_GET_SIDETONE_ID, sizeof(cmd));

	err = aoc_audio_control(CMD_OUTPUT_CHANNEL, (uint8_t *)&cmd, sizeof(cmd), (uint8_t *)&cmd,
				chip);
	if (err < 0) {
		pr_err("ERR:%d in get mic dc blocker state\n", err);
		return err;
	}

	pr_info("%s, sidetone- vol:%d, Eq num stage: %d, mic:%d\n", __func__, cmd.volume,
		cmd.eq_num_stages, cmd.mic_select);

	chip->sidetone_cfg = cmd;

	if (val) {
		switch (param) {
		case SIDETONE_CFG_VOL:
			*val = cmd.volume;
			break;
		case SIDETONE_CFG_STAGE_NUM:
			*val = cmd.eq_num_stages;
			break;
		case SIDETONE_CFG_MIC_ID:
			*val = cmd.mic_select;
			break;
		default:
			err = -EINVAL;
			pr_err("ERR:%d invalid param for sidetone cfg\n", err);
		}
	}

	return err;
}

int aoc_sidetone_cfg_set(struct aoc_chip *chip, int param, long val)
{
	int err = 0;
	struct CMD_AUDIO_OUTPUT_SET_SIDETONE cmd;

	aoc_sidetone_cfg_get(chip, 0, NULL); //get the updated data from AoC before setting
	cmd.eq_num_stages = chip->sidetone_cfg.eq_num_stages;
	cmd.mic_select = chip->sidetone_cfg.mic_select;
	cmd.volume = chip->sidetone_cfg.volume;

	switch (param) {
	case SIDETONE_CFG_VOL:
		cmd.volume = val;
		break;
	case SIDETONE_CFG_STAGE_NUM:
		cmd.eq_num_stages = val;
		break;
	case SIDETONE_CFG_MIC_ID:
		cmd.mic_select = val;
		break;
	default:
		return -EINVAL;
	}

	pr_info("%s, side tone- vol:%d, Eq num stage: %d, mic:%d\n", __func__, cmd.volume,
		cmd.eq_num_stages, cmd.mic_select);

	AocCmdHdrSet(&(cmd.parent), CMD_AUDIO_OUTPUT_SET_SIDETONE_ID, sizeof(cmd));
	err = aoc_audio_control(CMD_OUTPUT_CHANNEL, (uint8_t *)&cmd, sizeof(cmd), (uint8_t *)&cmd,
				chip);
	if (err < 0) {
		pr_err("ERR:%d in set sidetone params\n", err);
		return err;
	}

	return err;
}

int aoc_sidetone_eq_get(struct aoc_chip *chip, int biquad_idx, long *val)
{
	int i, err = 0;
	struct CMD_AUDIO_OUTPUT_GET_SIDETONE_EQ cmd;
	int n_params = ARRAY_SIZE(cmd.coeffs);

	pr_info("%s: sidetone eq %d - %d params\n", __func__, biquad_idx, n_params);

	AocCmdHdrSet(&(cmd.parent), CMD_AUDIO_OUTPUT_GET_SIDETONE_EQ_ID, sizeof(cmd));

	cmd.stage_num = biquad_idx;
	err = aoc_audio_control(CMD_OUTPUT_CHANNEL, (uint8_t *)&cmd, sizeof(cmd), (uint8_t *)&cmd,
				chip);
	if (err < 0) {
		pr_err("ERR:%d in get sidetone eq param\n", err);
		return err;
	}

	if (val) {
		for (i = 0; i < n_params; i++)
			val[i] = *(uint32_t *)&cmd.coeffs[i];
	}

	return 0;
}

int aoc_sidetone_eq_set(struct aoc_chip *chip, int biquad_idx, long *val)
{
	int i, err = 0;
	uint32_t tmp;
	struct CMD_AUDIO_OUTPUT_SET_SIDETONE_EQ cmd;
	int n_params = ARRAY_SIZE(cmd.coeffs);

	pr_info("%s: sidetone eq %d - %d params\n", __func__, biquad_idx, n_params);

	AocCmdHdrSet(&(cmd.parent), CMD_AUDIO_OUTPUT_SET_SIDETONE_EQ_ID, sizeof(cmd));
	cmd.stage_num = biquad_idx;
	for (i = 0; i < n_params; i++) {
		tmp = (uint32_t)val[i];
		cmd.coeffs[i] = *(float *)(&tmp);
	}
	err = aoc_audio_control(CMD_OUTPUT_CHANNEL, (uint8_t *)&cmd, sizeof(cmd), (uint8_t *)&cmd,
				chip);
	if (err < 0) {
		pr_err("ERR:%d in set sidetone eq param\n", err);
		return err;
	}

	return 0;
}

int aoc_incall_capture_enable_get(struct aoc_chip *chip, int stream, long *val)
{
	int err;
	struct CMD_AUDIO_OUTPUT_TELE_CAPT cmd;

#if IS_ENABLED(CONFIG_SOC_GS101) || IS_ENABLED(CONFIG_SOC_GS201)
	if (stream == 3) {
		*val = chip->incall_capture_state[stream];
		return 0;
	}
#endif

	AocCmdHdrSet(&(cmd.parent), CMD_AUDIO_OUTPUT_GET_TELE_CAPT_ID, sizeof(cmd));

	cmd.ring = stream;
	err = aoc_audio_control(CMD_OUTPUT_CHANNEL, (uint8_t *)&cmd, sizeof(cmd), (uint8_t *)&cmd,
				chip);
	if (err < 0) {
		pr_err("ERR:%d in get voice capture stream %d state\n", err, stream);
		return err;
	}

	if (val)
		*val = cmd.mode;

	chip->incall_capture_state[stream] = cmd.mode;
	pr_info("%s: get voice call capture stream %d state %d\n", __func__, stream, cmd.mode);

	return 0;
}

int aoc_incall_capture_enable_set(struct aoc_chip *chip, int stream, long val)
{
	int err;
	struct CMD_AUDIO_OUTPUT_TELE_CAPT cmd;

#if IS_ENABLED(CONFIG_SOC_GS101) || IS_ENABLED(CONFIG_SOC_GS201)
	if (stream == 3) {
		chip->incall_capture_state[stream] = val;
		pr_info("%s: Not support stream 3\n", __func__);
		return 0;
	}
#endif

	AocCmdHdrSet(&(cmd.parent), CMD_AUDIO_OUTPUT_SET_TELE_CAPT_ID, sizeof(cmd));
	cmd.ring = stream;
	cmd.mode = val;

	err = aoc_audio_control(CMD_OUTPUT_CHANNEL, (uint8_t *)&cmd, sizeof(cmd), (uint8_t *)&cmd,
				chip);
	if (err < 0) {
		pr_err("ERR:%d in set incall capture stream %d state as %d\n", err, stream,
		       cmd.mode);
		return err;
	}

	chip->incall_capture_state[stream] = val;
	pr_info("%s: set incall capture stream %d state as %d\n", __func__, stream, cmd.mode);

	return 0;
}

int aoc_incall_playback_enable_get(struct aoc_chip *chip, int stream, long *val)
{
	int err;
	struct CMD_AUDIO_OUTPUT_INCALL_MUSIC cmd;

	AocCmdHdrSet(&(cmd.parent), CMD_AUDIO_OUTPUT_GET_INCALL_MUSIC_ID, sizeof(cmd));

	cmd.ring = stream;
	err = aoc_audio_control(CMD_OUTPUT_CHANNEL, (uint8_t *)&cmd, sizeof(cmd), (uint8_t *)&cmd,
				chip);
	if (err < 0) {
		pr_err("ERR:%d in get voice playback stream %d state\n", err, stream);
		return err;
	}

	if (val)
		*val = cmd.enable;

	pr_info("%s: get incall playback stream %d state %d\n", __func__, stream, cmd.enable);

	return 0;
}

int aoc_incall_playback_enable_set(struct aoc_chip *chip, int stream, long val)
{
	int err;
	struct CMD_AUDIO_OUTPUT_INCALL_MUSIC cmd;

	AocCmdHdrSet(&(cmd.parent), CMD_AUDIO_OUTPUT_SET_INCALL_MUSIC_ID, sizeof(cmd));
	cmd.ring = stream;
	cmd.enable = val;

	err = aoc_audio_control(CMD_OUTPUT_CHANNEL, (uint8_t *)&cmd, sizeof(cmd), (uint8_t *)&cmd,
				chip);
	if (err < 0) {
		pr_err("ERR:%d in set incall playback stream %d state as %d\n", err, stream,
		       cmd.enable);
		return err;
	}

	pr_info("%s: set incall playback stream %d state as %d\n", __func__, stream, cmd.enable);

	return 0;
}

int aoc_incall_playback_mic_channel_get(struct aoc_chip *chip, int stream, long *val)
{
	int err;
	struct CMD_AUDIO_OUTPUT_INCALL_MUSIC_CHAN cmd;

	AocCmdHdrSet(&(cmd.parent), CMD_AUDIO_OUTPUT_GET_INCALL_MUSIC_CHAN_ID, sizeof(cmd));

	cmd.ring = stream;
	err = aoc_audio_control(CMD_OUTPUT_CHANNEL, (uint8_t *)&cmd, sizeof(cmd), (uint8_t *)&cmd,
				chip);
	if (err < 0) {
		pr_err("ERR:%d in get incall music stream %d mic channel\n", err, stream);
		return err;
	}

	if (val)
		*val = cmd.channel;

	pr_info("incall playback stream %d: mic channel get :%d\n", stream, cmd.channel);

	return 0;
}

int aoc_incall_playback_mic_channel_set(struct aoc_chip *chip, int stream, long val)
{
	int err;
	struct CMD_AUDIO_OUTPUT_INCALL_MUSIC_CHAN cmd;

	AocCmdHdrSet(&(cmd.parent), CMD_AUDIO_OUTPUT_SET_INCALL_MUSIC_CHAN_ID, sizeof(cmd));
	cmd.ring = stream;
	cmd.channel = val;

	err = aoc_audio_control(CMD_OUTPUT_CHANNEL, (uint8_t *)&cmd, sizeof(cmd), (uint8_t *)&cmd,
				chip);
	if (err < 0) {
		pr_err("ERR:%d in incall playback stream %d: mic channel set as %d\n", err, stream,
		       cmd.channel);
		return err;
	}

	pr_info("incall playback stream %d: mic channel :%d set\n", stream, cmd.channel);

	return 0;
}

/* TODO: temporary solution for mic muting, has to be revised using DSP modules instead of mixer */
int aoc_voice_call_mic_mute(struct aoc_chip *chip, int mute)
{
	int err;
	int gain = (mute == 1) ? -700 : chip->default_mic_hw_gain;

	pr_debug("voice call mic mute: %d\n", mute);
	if ((err = aoc_mic_hw_gain_set(chip, MIC_HIGH_POWER_GAIN, gain))) {
		pr_err("ERR: fail in muting mic in voice call\n");
		return err;
	}

	return 0;
}

int aoc_get_builtin_mic_process_mode(struct aoc_chip *chip)
{
	int err;
	struct CMD_AUDIO_INPUT_GET_AP_MIC_INDEX cmd;

	AocCmdHdrSet(&(cmd.parent), CMD_AUDIO_INPUT_GET_AP_MIC_INDEX_ID,
		     sizeof(cmd));

	err = aoc_audio_control(CMD_INPUT_CHANNEL, (uint8_t *)&cmd,
				sizeof(cmd), (uint8_t *)&cmd, chip);
	if (err < 0)
		pr_err("Error in get builtin_mic_process_mode!\n");

	return err < 0 ? err : cmd.mic_process_index;
}

int aoc_set_builtin_mic_process_mode(struct aoc_chip *chip, long mode)
{
	int err;
	struct CMD_AUDIO_INPUT_GET_AP_MIC_INDEX cmd;

	AocCmdHdrSet(&(cmd.parent), CMD_AUDIO_INPUT_SET_AP_MIC_INDEX_ID, sizeof(cmd));

	switch (mode) {
	case 0:
		cmd.mic_process_index = AP_MIC_PROCESS_RAW;
		break;
	case 1:
		cmd.mic_process_index = AP_MIC_PROCESS_SPATIAL;
		break;
	default:
		cmd.mic_process_index = AP_MIC_PROCESS_RAW;
	}

	err = aoc_audio_control(CMD_INPUT_CHANNEL, (uint8_t *)&cmd, sizeof(cmd), NULL, chip);
	if (err < 0) {
		pr_err("ERR: %d in set builtin_mic_process_mode!\n", err);
		return err;
	}

	return 0;
}

int aoc_get_dsp_state(struct aoc_chip *chip)
{
	int err;
	struct CMD_AUDIO_OUTPUT_GET_DSP_STATE cmd;

	AocCmdHdrSet(&(cmd.parent), CMD_AUDIO_OUTPUT_GET_DSP_STATE_ID,
		     sizeof(cmd));

	err = aoc_audio_control(CMD_OUTPUT_CHANNEL, (uint8_t *)&cmd,
				sizeof(cmd), (uint8_t *)&cmd, chip);
	if (err < 0)
		pr_err("Error in get aoc dsp state !\n");

	return err < 0 ? err : cmd.mode;
}

int aoc_get_asp_mode(struct aoc_chip *chip, int block, int component, int key)
{
	int err;
	struct CMD_AUDIO_OUTPUT_GET_PARAMETER cmd;

	AocCmdHdrSet(&(cmd.parent), CMD_AUDIO_OUTPUT_GET_PARAMETER_ID,
		     sizeof(cmd));

	/* Sink 0-4 with block ID from 16-20 */
	cmd.block = block;
	cmd.component = component;
	cmd.key = key;

	pr_info("get asp mode: block=%d, component=%d, key=%d\n", block, component, key);

	/* Send cmd to AOC */
	err = aoc_audio_control(CMD_OUTPUT_CHANNEL, (uint8_t *)&cmd,
				sizeof(cmd), (uint8_t *)&cmd, chip);
	if (err < 0) {
		pr_err("ERR:%d in getting dsp mode, block=%d, component=%d, key=%d\n",
		       err, block, component, key);
		return err;
	}

	return cmd.val;
}

int aoc_set_asp_mode(struct aoc_chip *chip, int block, int component, int key,
		     int val)
{
	int err;
	struct CMD_AUDIO_OUTPUT_SET_PARAMETER cmd;

	AocCmdHdrSet(&(cmd.parent), CMD_AUDIO_OUTPUT_SET_PARAMETER_ID,
		     sizeof(cmd));

	/* Sink 0-4 with block ID from 16-20 */
	cmd.block = block;
	cmd.component = component;
	cmd.key = key;
	cmd.val = val;

	pr_info("set asp mode: block=%d, component=%d, key=%d, val=%d\n", block,
		component, key, val);

	/* Send cmd to AOC */
	err = aoc_audio_control(CMD_OUTPUT_CHANNEL, (uint8_t *)&cmd,
				sizeof(cmd), NULL, chip);
	if (err < 0)
		pr_err("ERR:%d in dsp mode, block=%d, component=%d, key=%d, val=%d\n",
		       err, block, component, key, val);

	return err;
}

int aoc_get_audio_dsp_mode(struct aoc_chip *chip, long *val)
{
	int err;
	struct CMD_AUDIO_OUTPUT_DSP_MODE cmd;

	AocCmdHdrSet(&(cmd.parent), CMD_AUDIO_OUTPUT_DSP_MODE_GET_ID, sizeof(cmd));

	pr_debug("Get audio asp mode\n");

	/* Send cmd to AOC */
	err = aoc_audio_control(CMD_OUTPUT_CHANNEL, (uint8_t *)&cmd, sizeof(cmd), (uint8_t *)&cmd,
				chip);
	if (err < 0) {
		pr_err("ERR:%d in getting audio dsp mode", err);
		return err;
	}

	if (val)
		*val = cmd.mode;

	return 0;
}

int aoc_set_audio_dsp_mode(struct aoc_chip *chip, long val)
{
	int err;
	struct CMD_AUDIO_OUTPUT_DSP_MODE cmd;

	AocCmdHdrSet(&(cmd.parent), CMD_AUDIO_OUTPUT_DSP_MODE_SET_ID, sizeof(cmd));

	cmd.mode = val;

	pr_info("Set audio dsp mode: %ld\n", val);

	/* Send cmd to AOC */
	err = aoc_audio_control(CMD_OUTPUT_CHANNEL, (uint8_t *)&cmd, sizeof(cmd), NULL, chip);
	if (err < 0) {
		pr_err("ERR:%d in audio dsp mode set:  val=%ld\n", err, val);
		return err;
	}

	return 0;
}

int aoc_get_sink_channel_bitmap(struct aoc_chip *chip, int sink)
{
	int err;
	struct CMD_AUDIO_OUTPUT_GET_SINKS_BITMAPS cmd;

	if (sink >= AUDIO_OUTPUT_SINKS) {
		pr_err("Err: sink id %d not exists!\n", sink);
		return -EINVAL;
	}

	AocCmdHdrSet(&(cmd.parent), CMD_AUDIO_OUTPUT_GET_SINKS_BITMAPS_ID, sizeof(cmd));

	err = aoc_audio_control(CMD_OUTPUT_CHANNEL, (uint8_t *)&cmd, sizeof(cmd), (uint8_t *)&cmd,
				chip);
	if (err < 0) {
		pr_err("Err:%d in get aoc sink %d channel bitmap!\n", err, sink);
		return err;
	}

	return cmd.bitmap[sink];
}

int aoc_get_sink_mode(struct aoc_chip *chip, int sink)
{
	return chip->sink_mode[sink];
}

int aoc_set_sink_mode(struct aoc_chip *chip, int sink, int mode)
{
	int err;
	struct CMD_AUDIO_OUTPUT_SINK2 cmd;
	AocCmdHdrSet(&(cmd.parent), CMD_AUDIO_OUTPUT_SINK2_ID, sizeof(cmd));

	cmd.sink = sink;
	cmd.mode = mode;
	err = aoc_audio_control(CMD_OUTPUT_CHANNEL, (uint8_t *)&cmd,
				sizeof(cmd), (uint8_t *)&cmd, chip);
	if (err < 0) {
		pr_err("Error in get aoc sink processing state !\n");
		return err;
	}

	chip->sink_mode[sink] = mode;
	pr_info("sink state set :%d - %d\n", sink, cmd.mode);

	return 0;
}

int aoc_get_sink_state(struct aoc_chip *chip, int sink)
{
	int err;
	struct CMD_AUDIO_OUTPUT_GET_SINK_PROCESSING_STATE cmd;
	AocCmdHdrSet(&(cmd.parent),
		     CMD_AUDIO_OUTPUT_GET_SINK_PROCESSING_STATE_ID,
		     sizeof(cmd));

	cmd.sink = sink;
	err = aoc_audio_control(CMD_OUTPUT_CHANNEL, (uint8_t *)&cmd,
				sizeof(cmd), (uint8_t *)&cmd, chip);
	if (err < 0)
		pr_err("Error in get aoc sink processing state !\n");

	pr_info("sink_state:%d - %d\n", sink, cmd.mode);

	return err < 0 ? err : cmd.mode;
}

/* TODO: usb cfg may be devided into three commands, dev/ep-ids, rx, tx */
int aoc_set_usb_config(struct aoc_chip *chip)
{
	int err;
	struct CMD_AUDIO_OUTPUT_USB_CONFIG cmd = chip->usb_sink_cfg;

	AocCmdHdrSet(&(cmd.parent), CMD_AUDIO_OUTPUT_USB_CONFIG_ID, sizeof(cmd));

	cmd.rx_enable = true;
	cmd.tx_enable = true;
	err = aoc_audio_control(CMD_OUTPUT_CHANNEL, (uint8_t *)&cmd, sizeof(cmd), NULL, chip);
	if (err < 0)
		pr_err("Err:%d in aoc set usb config!\n", err);

	return err;
}

int aoc_set_usb_config_v2(struct aoc_chip *chip)
{
	int err;
	struct CMD_AUDIO_OUTPUT_USB_CONFIG_V2 cmd = chip->usb_sink_cfg_v2;

	AocCmdHdrSet(&(cmd.parent), CMD_AUDIO_OUTPUT_USB_CONFIG_V2_ID, sizeof(cmd));

	cmd.rx_enable = true;
	cmd.tx_enable = true;
	err = aoc_audio_control(CMD_OUTPUT_CHANNEL, (uint8_t *)&cmd, sizeof(cmd), NULL, chip);
	if (err < 0)
		pr_err("Err:%d in aoc set usb config v2!\n", err);

	return err;
}

int aoc_set_usb_feedback_endpoint(struct aoc_chip *chip, struct usb_device *udev,
			struct usb_host_endpoint *ep)
{
	struct usb_endpoint_descriptor *ep_desc = &ep->desc;
	struct CMD_USB_CONTROL_SEND_FEEDBACK_EP_INFO cmd;
	int err = 0;

	AocCmdHdrSet(&(cmd.parent), CMD_USB_CONTROL_SEND_FEEDBACK_EP_INFO_ID, sizeof(cmd));

	cmd.enabled = true;
	cmd.bus_id = udev->bus->busnum;
	cmd.dev_num = udev->devnum;
	cmd.slot_id = udev->slot_id;
	cmd.ep_num = usb_endpoint_num(ep_desc);
	cmd.max_packet = ep_desc->wMaxPacketSize;
	cmd.binterval = ep_desc->bInterval;
	cmd.brefresh = ep_desc->bRefresh;

	err = aoc_audio_control(CMD_OUTPUT_CHANNEL, (uint8_t *)&cmd, sizeof(cmd), (uint8_t *)&cmd,
		chip);
	if (err < 0) {
		pr_err("ERR:%d in aoc set usb feedback endpoint\n", err);
	}

	return err;
}

int aoc_set_usb_offload_state(struct aoc_chip *chip, bool offload_enable)
{
	struct CMD_USB_CONTROL_SET_OFFLOAD_STATE cmd;
	int err = 0;

	AocCmdHdrSet(&(cmd.parent), CMD_USB_CONTROL_SET_OFFLOAD_STATE_ID, sizeof(cmd));

	cmd.offloading = offload_enable;

	err = aoc_audio_control(CMD_OUTPUT_CHANNEL, (uint8_t *)&cmd, sizeof(cmd), (uint8_t *)&cmd,
		chip);
	if (err < 0) {
		pr_err("ERR:%d in aoc set usb offload fail\n", err);
	}

	return err;
}

static int
aoc_audio_playback_trigger_source(struct aoc_alsa_stream *alsa_stream, int cmd,
				  int src)
{
	int err;
	struct CMD_AUDIO_OUTPUT_SOURCE source;

	AocCmdHdrSet(&(source.parent), CMD_AUDIO_OUTPUT_SOURCE_ID,
		     sizeof(source));

	/* source on/off */
	source.source = src;
	switch (cmd) {
	case START:
		source.on = 1;
		break;
	case STOP:
		source.on = 0;
		break;
	default:
		pr_err("Invalid source operation (only On/Off allowed)\n");
		return -EINVAL;
	}

	err = aoc_audio_control(CMD_OUTPUT_CHANNEL, (uint8_t *)&source,
				sizeof(source), NULL, alsa_stream->chip);

	pr_debug("Source %d %s !\n", alsa_stream->idx,
		 cmd == START ? "on" : "off");

	return err;
}

static int aoc_audio_playback_source_on(struct aoc_chip *chip, int cmd, int src)
{
	int err;
	struct CMD_AUDIO_OUTPUT_SOURCE source;

	AocCmdHdrSet(&(source.parent), CMD_AUDIO_OUTPUT_SOURCE_ID, sizeof(source));

	/* source on/off */
	source.source = src;
	switch (cmd) {
	case START:
		source.on = 1;
		break;
	case STOP:
		source.on = 0;
		break;
	default:
		pr_err("Invalid source operation (only On/Off allowed)\n");
		return -EINVAL;
	}

	err = aoc_audio_control(CMD_OUTPUT_CHANNEL, (uint8_t *)&source, sizeof(source), NULL, chip);
	if (err < 0) {
		pr_err("ERR:%d in playback source %s\n", err, (cmd == START) ? "on" : "off");
		return err;
	}

	pr_debug("Source %d %s !\n", src, cmd == START ? "on" : "off");

	return 0;
}

/* Bind/unbind the source and dest */
static int aoc_audio_path_bind(int src, int dst, int cmd, struct aoc_chip *chip)
{
	int err;
	struct CMD_AUDIO_OUTPUT_BIND bind;

	if (dst < 0)
		return 0;

	pr_info("%s: src:%d - sink:%d!\n", cmd == START ? "bind" : "unbind", src, dst);

	AocCmdHdrSet(&(bind.parent), CMD_AUDIO_OUTPUT_BIND2_ID, sizeof(bind));
	bind.bind = (cmd == START) ? 1 : 0;
	bind.src = src;
	bind.dst = dst;
	err = aoc_audio_control(CMD_OUTPUT_CHANNEL, (uint8_t *)&bind, sizeof(bind), NULL, chip);

	if (err < 0)
		pr_err("ERR:%d %s: src:%d - sink:%d!\n", err, (cmd == START) ? "bind" : "unbind",
		       src, dst);

	return err;
}

static int aoc_mmap_capture_trigger(struct aoc_alsa_stream *alsa_stream, int record_cmd)
{
	int err = 0;
	int cmd_id;
	struct CMD_AUDIO_INPUT_MMAP_ENABLE_2 cmd;
	struct aoc_chip *chip = alsa_stream->chip;

	if (alsa_stream->channels > 2) {
		pr_err("ERR: mmap capture channel number %d (only mono or stereo allowed)\n",
		       alsa_stream->channels);
		return -EINVAL;
	}

	cmd_id = (record_cmd == START) ? CMD_AUDIO_INPUT_MMAP_ENABLE_2_ID :
					       CMD_AUDIO_INPUT_MIC_MMAP_DISABLE_ID;

	if (record_cmd == START) {
		AocCmdHdrSet(&(cmd.parent), cmd_id, sizeof(cmd));
		cmd.chan = alsa_stream->channels;
		err = aoc_audio_control(CMD_INPUT_CHANNEL, (uint8_t *)&cmd, sizeof(cmd), NULL, chip);
	} else
		err = aoc_audio_control_simple_cmd(CMD_INPUT_CHANNEL, cmd_id, chip);

	if (err < 0) {
		pr_err("ERR:%d in audio mmap capture %s\n", err,
		       (record_cmd == START) ? "start" : "stop");
		return err;
	}

	return 0;
}

static int aoc_raw_capture_trigger(struct aoc_alsa_stream *alsa_stream, int record_cmd)
{
	int err = 0;
	int cmd_id;
	struct CMD_AUDIO_INPUT_RAW_ENABLE_2 cmd;
	struct aoc_chip *chip = alsa_stream->chip;

	if (alsa_stream->channels > 2) {
		pr_err("ERR: raw capture channel number %d (only mono or stereo allowed)\n",
		       alsa_stream->channels);
		return -EINVAL;
	}

	cmd_id = (record_cmd == START) ? CMD_AUDIO_INPUT_RAW_ENABLE_2_ID :
					       CMD_AUDIO_INPUT_MIC_RAW_DISABLE_ID;

	if (record_cmd == START) {
		AocCmdHdrSet(&(cmd.parent), cmd_id, sizeof(cmd));
		cmd.chan = alsa_stream->channels;
		err = aoc_audio_control(CMD_INPUT_CHANNEL, (uint8_t *)&cmd, sizeof(cmd), NULL, chip);
	} else
		err = aoc_audio_control_simple_cmd(CMD_INPUT_CHANNEL, cmd_id, chip);

	if (err < 0) {
		pr_err("ERR:%d in audio raw capture %s\n", err,
		       (record_cmd == START) ? "start" : "stop");
		return err;
	}

	return 0;
}

static int aoc_audio_capture_mic_process_mode(struct aoc_chip *chip,
					      struct aoc_alsa_stream *alsa_stream)
{
	int err;
	struct CMD_AUDIO_INPUT_GET_AP_MIC_INDEX cmd;

	if (alsa_stream->idx == UC_ULTRASONIC_RECORD)
		return 0;

	AocCmdHdrSet(&(cmd.parent), CMD_AUDIO_INPUT_SET_AP_MIC_INDEX_ID, sizeof(cmd));

	if (chip->mic_spatial_module_enable)
		cmd.mic_process_index = AP_MIC_PROCESS_SPATIAL;
	else
		cmd.mic_process_index = AP_MIC_PROCESS_RAW;

	err = aoc_audio_control(CMD_INPUT_CHANNEL, (uint8_t *)&cmd, sizeof(cmd), NULL, chip);
	if (err < 0) {
		pr_err("ERR: %d in set builtin_mic_process_mode!\n", err);
		return err;
	}

	return 0;
}

static int aoc_audio_capture_runtime_control(struct aoc_alsa_stream *alsa_stream, int dst, bool on,
					     bool be_on)
{
	int err;
	struct aoc_chip *chip = alsa_stream->chip;

	if (!be_on)
		return 0;

	if (on) {
		err = aoc_audio_capture_mic_process_mode(chip, alsa_stream);
		if (err < 0)
			return err;

		err = ap_data_control_trigger(chip, alsa_stream, START);
		if (err < 0)
			return err;
	} else {
		err = ap_data_control_trigger(chip, alsa_stream, STOP);
		if (err < 0)
			return err;
	}
	return 0;
}

static int aoc_raw_capture_runtime_control(struct aoc_alsa_stream *alsa_stream,
	 int dst, bool on, bool be_on)
{
	int err;
	if (on) {
		if (!be_on)
			return 0;

		err = aoc_raw_capture_trigger(alsa_stream, START);
		if (err < 0)
			return err;
	} else {
		err = aoc_raw_capture_trigger(alsa_stream, STOP);
		if (err < 0)
			return err;
	}
	return 0;
}

static int aoc_mmap_capture_runtime_control(struct aoc_alsa_stream *alsa_stream,
	 int dst, bool on, bool be_on)
{
	int err;
	if (on) {
		if (!be_on)
			return 0;

		err = aoc_mmap_capture_trigger(alsa_stream, START);
		if (err < 0)
			return err;
	} else {
		err = aoc_mmap_capture_trigger(alsa_stream, STOP);
		if (err < 0)
			return err;
	}
	return 0;
}

int ap_data_control_trigger(struct aoc_chip *chip, struct aoc_alsa_stream *alsa_stream,
			    int record_cmd)
{
	int err = 0;
	struct CMD_HDR cmd;
	int cmd_id;

	/* We don't support the stop_data yet */
	if (record_cmd != START)
		return 0;

	if (alsa_stream->idx == UC_ULTRASONIC_RECORD)
		cmd_id = CMD_AUDIO_INPUT_ULTRASONIC_AP_START_ID;
	else
		cmd_id = CMD_AUDIO_INPUT_MIC_RECORD_AP_START_DATA_ID;

	AocCmdHdrSet(&cmd, cmd_id, sizeof(cmd));

	err = aoc_audio_control(CMD_INPUT_CHANNEL, (uint8_t *)&cmd, sizeof(cmd), NULL, chip);
	if (err < 0) {
		pr_err("ERR:%d in audio input mic record start/stop\n", err);
	}
	return err;
}

int ap_record_stop(struct aoc_chip *chip, struct aoc_alsa_stream *alsa_stream)
{
	int err = 0;
	int cmd_id;

	if (alsa_stream->idx == UC_ULTRASONIC_RECORD)
		cmd_id = CMD_AUDIO_INPUT_ULTRASONIC_CAPTURE_STOP_ID;
	else
		cmd_id = CMD_AUDIO_INPUT_AP_INPUT_STOP_ID;

	err = aoc_audio_control_simple_cmd(CMD_INPUT_CHANNEL, cmd_id, chip);

	if (err < 0) {
		pr_err("ERR:%d in audio input mic record start/stop\n", err);
	}
	return err;
}

static int aoc_record_filter_runtime_control(struct aoc_chip *chip, uint32_t port_id, bool on)
{
	int err;

	if (aoc_record_param_configured_num(chip) == 0)
		return 0;

	pr_info("%s: port 0x%x runtime %d", __func__, port_id, on);

	if (on) {
		err = aoc_audio_capture_mic_prepare(chip);
	} else {
		err = aoc_audio_capture_mic_close(chip);
	}
	return err;
}

static int aoc_us_record_filter_runtime_control(struct aoc_chip *chip, uint32_t port_id, bool on)
{
	int err;

	if (aoc_us_record_param_configured_num(chip) == 0)
		return 0;

	pr_info("%s: port 0x%x runtime %d", __func__, port_id, on);
	if (on) {
		err = aoc_audio_us_capture_mic_prepare(chip);
	} else {
		err = aoc_audio_us_capture_mic_close(chip);
	}

	return err;
}

int aoc_capture_filter_runtime_control(struct aoc_chip *chip, uint32_t port_id, bool on)
{
	switch (port_id) {
	case INTERNAL_MIC_US_TX:
		return aoc_us_record_filter_runtime_control(chip, port_id, on);
	default:
		return aoc_record_filter_runtime_control(chip, port_id, on);
	}
}

int aoc_audio_capture_runtime_trigger(struct aoc_chip *chip, int ep_id, int dst, bool on)
{
	struct aoc_alsa_stream *alsa_stream;

	alsa_stream = find_alsa_stream_by_ep_id(chip, ep_id);
	if (!alsa_stream)
		return 0;

	if (!alsa_stream->running)
		return 0;

	pr_info("%s: on %d idx %d", __func__, on, alsa_stream->idx);

	switch (alsa_stream->idx) {
	case UC_ULTRASONIC_RECORD:
	case UC_AUDIO_RECORD:
		return aoc_audio_capture_runtime_control(alsa_stream, dst, on, true);
	case UC_MMAP_RECORD:
		return aoc_mmap_capture_runtime_control(alsa_stream, dst, on, true);
	case UC_LOW_LATENCY_AUDIO_RECORD:
		return aoc_raw_capture_runtime_control(alsa_stream, dst, on, true);
	default:
		return 0;
	}
}

static int aoc_audio_capture_path_bind(struct aoc_chip *chip, int ep_id,
	 int dst, bool on, bool be_on)
{
	struct aoc_alsa_stream *alsa_stream;

	alsa_stream = find_alsa_stream_by_ep_id(chip, ep_id);
	if (!alsa_stream)
		return 0;

	if (!alsa_stream->running)
		return 0;

	switch (alsa_stream->idx) {
	case UC_ULTRASONIC_RECORD:
	case UC_AUDIO_RECORD:
		return aoc_audio_capture_runtime_control(alsa_stream, dst, on, be_on);
	case UC_MMAP_RECORD:
		return aoc_mmap_capture_runtime_control(alsa_stream, dst, on, be_on);
	case UC_LOW_LATENCY_AUDIO_RECORD:
		return aoc_raw_capture_runtime_control(alsa_stream, dst, on, be_on);
	default:
		return 0;
	}
}

int aoc_audio_path_open(struct aoc_chip *chip, int src, int dest, bool be_on)
{
	uint32_t src_idx, dest_idx;
	bool src_for_capture;

	/* ignore nohost */
	if (src & AOC_NOHOST)
		return 0;

	src_for_capture = src & AOC_TX;
	src_idx = AOC_ID_TO_INDEX(src);
	dest_idx = AOC_ID_TO_INDEX(dest);

	/* voice call capture or playback */
	if ((src_idx == 3 && src_for_capture) || (src_idx == 4 && !src_for_capture))
		return aoc_phonecall_path_open(chip, src_idx, dest_idx, dest & AOC_TX);

	if (src_idx == IDX_VOIP)
		return aoc_voipcall_path_open(chip, src_idx, dest_idx, dest & AOC_TX);

	if (src_for_capture)
		return aoc_audio_capture_path_bind(chip, src, dest, true, be_on);
	else
		return aoc_audio_path_bind(ep_id_to_source(src_idx),
			 hw_id_to_sink(dest_idx), START, chip);
}

int aoc_audio_path_close(struct aoc_chip *chip, int src, int dest, bool be_on)
{
	uint32_t src_idx, dest_idx;
	bool src_for_capture;

	/* ignore nohost */
	if (src & AOC_NOHOST)
		return 0;

	src_for_capture = src & AOC_TX;
	src_idx = AOC_ID_TO_INDEX(src);
	dest_idx = AOC_ID_TO_INDEX(dest);

	/* voice call capture or playback */
	if ((src_idx == 3 && src_for_capture) || (src_idx == 4 && !src_for_capture))
		return aoc_phonecall_path_close(chip, src_idx, dest_idx, dest & AOC_TX);

	if (src_idx == IDX_VOIP)
		return aoc_voipcall_path_close(chip, src_idx, dest_idx, dest & AOC_TX);

	if (src_for_capture)
		return aoc_audio_capture_path_bind(chip, src, dest, false, be_on);
	else
		return aoc_audio_path_bind(ep_id_to_source(src_idx),
			 hw_id_to_sink(dest_idx), STOP, chip);
}

#if !IS_ENABLED(CONFIG_SOC_GS101)
static int aoc_audio_playback_set_params2(struct aoc_alsa_stream *alsa_stream, int source_mode)
{
	struct CMD_AUDIO_OUTPUT_EP_SETUP2 cmd;

	AocCmdHdrSet(&(cmd.parent), CMD_AUDIO_OUTPUT_EP_SETUP2_ID, sizeof(cmd));
	cmd.channel = alsa_stream->entry_point_idx;

	cmd.buffer_size = alsa_stream->buffer_size;
	cmd.period_size = alsa_stream->period_size;

	cmd.mode = source_mode;

	pr_debug("audio set param2:idx %d, buffer_size=%d, period_size=%d\n", cmd.channel,
				cmd.buffer_size, cmd.period_size);
	return aoc_audio_control(CMD_OUTPUT_CHANNEL, (uint8_t *)&cmd,
				sizeof(cmd), NULL, alsa_stream->chip);
}

int aoc_hifi_incall_set_params(struct aoc_alsa_stream *alsa_stream)
{
	int source_mode;

	if (alsa_stream->stream_type == INCALL)
		source_mode = ENTRYPOINT_MODE_INCALL_SCREEN;
	else
		source_mode = ENTRYPOINT_MODE_HIFI;

	return aoc_audio_playback_set_params2(alsa_stream, source_mode);
}

int aoc_voip_set_params(struct aoc_alsa_stream *alsa_stream)
{
	int source_mode;

	source_mode = ENTRYPOINT_MODE_VOIP;
	return aoc_audio_playback_set_params2(alsa_stream, source_mode);
}
#endif

static int aoc_audio_playback_set_params(struct aoc_alsa_stream *alsa_stream,
					 uint32_t channels, uint32_t samplerate,
					 uint32_t bps, bool pcm_float_fmt,
					 int source_mode)
{
	int err;
	struct CMD_AUDIO_OUTPUT_EP_SETUP cmd;

	AocCmdHdrSet(&(cmd.parent), CMD_AUDIO_OUTPUT_EP_SETUP_ID, sizeof(cmd));
	cmd.d.channel = alsa_stream->entry_point_idx;
	cmd.d.watermark = PLAYBACK_WATERMARK_DEFAULT;
	cmd.d.length = 0;
	cmd.d.address = 0;
	cmd.d.wraparound = true;
	cmd.d.metadata.offset = 0;

	switch (bps) {
	case 32:
		cmd.d.metadata.bits = WIDTH_32_BIT;
		break;
	case 24:
		cmd.d.metadata.bits = WIDTH_24_BIT;
		break;
	case 16:
		cmd.d.metadata.bits = WIDTH_16_BIT;
		break;
	case 8:
		cmd.d.metadata.bits = WIDTH_8_BIT;
		break;
	default:
		cmd.d.metadata.bits = WIDTH_32_BIT;
	}

	cmd.d.metadata.format =
		(pcm_float_fmt) ? FRMT_FLOATING_POINT : FRMT_FIXED_POINT;
	cmd.d.metadata.chan = channels;

	switch (samplerate) {
	case 48000:
		cmd.d.metadata.sr = SR_48KHZ;
		break;
	case 44100:
		cmd.d.metadata.sr = SR_44K1HZ;
		break;
	case 16000:
		cmd.d.metadata.sr = SR_16KHZ;
		break;
	case 8000:
		cmd.d.metadata.sr = SR_8KHZ;
		break;
	default:
		cmd.d.metadata.sr = SR_48KHZ;
	}

	pr_debug("chan =%d, sr=%d, bits=%d\n", cmd.d.metadata.chan,
		 cmd.d.metadata.sr, cmd.d.metadata.bits);

	switch (source_mode) {
	case PLAYBACK_MODE:
		cmd.mode = ENTRYPOINT_MODE_PLAYBACK;
		break;
	case HAPTICS_MODE:
		cmd.mode = ENTRYPOINT_MODE_HAPTICS;
		break;
	case OFFLOAD_MODE:
		cmd.mode = ENTRYPOINT_MODE_DECODE_OFFLOAD;
		break;
	default:
		cmd.mode = ENTRYPOINT_MODE_PLAYBACK;
	}

	err = aoc_audio_control(CMD_OUTPUT_CHANNEL, (uint8_t *)&cmd,
				sizeof(cmd), NULL, alsa_stream->chip);
	if (err < 0) {
		pr_err("ERR:%d in playback set parameters\n", err);
		goto exit;
	}

#if !IS_ENABLED(CONFIG_SOC_GS101)
	err = aoc_audio_playback_set_params2(alsa_stream, cmd.mode);
	if (err < 0)
		pr_err("ERR:%d in playback set parameters2\n", err);
#endif

exit:
	return err;
}

#if !IS_ENABLED(CONFIG_SOC_GS101)
static int aoc_audio_capture_set_params2(struct aoc_alsa_stream *alsa_stream) {
	struct CMD_AUDIO_INPUT_MIC_RECORD_AP_SET_PARAMS2 cmd;
	int cmd_id;

	cmd.channel = alsa_stream->idx;
	cmd.buffer_size = alsa_stream->buffer_size;
	cmd.period_size = alsa_stream->period_size;

	if (alsa_stream->idx == UC_ULTRASONIC_RECORD)
		cmd_id = CMD_AUDIO_INPUT_ULTRASONIC_AP_SET_PARAMS2_ID;
	else
		cmd_id = CMD_AUDIO_INPUT_MIC_RECORD_AP_SET_PARAMS2_ID;
	AocCmdHdrSet(&(cmd.parent), cmd_id, sizeof(cmd));

	pr_debug("audio set param2:idx %d, buffer_size=%d, period_size=%d\n", cmd.channel,
				cmd.buffer_size, cmd.period_size);
	return aoc_audio_control(CMD_INPUT_CHANNEL, (uint8_t *)&cmd,
				sizeof(cmd), NULL, alsa_stream->chip);
}
#endif

static int aoc_audio_capture_set_params(struct aoc_alsa_stream *alsa_stream, uint32_t channels,
					uint32_t samplerate, uint32_t bps, bool pcm_float_fmt)
{
	int err = 0;
	struct CMD_AUDIO_INPUT_MIC_RECORD_AP_SET_PARAMS cmd;
	int cmd_id;
	struct aoc_chip *chip = alsa_stream->chip;
	int i, mic_id;

	if (alsa_stream->idx < 0 || alsa_stream->idx >= sizeof(chip->capture_param_set) * 8) {
		pr_err("ERR: idx %d over capture_param_set maximum\n", alsa_stream->idx);
		return -EINVAL;
	}

	/* Regular audio capture should be the primary setting of the single ap filter */
	if ((alsa_stream->idx != UC_ULTRASONIC_RECORD) &&
	    (chip->capture_param_set & (1 << UC_AUDIO_RECORD))) {
		pr_info("%s: ignore capture set param 0x%llu", __func__, chip->capture_param_set);
		chip->capture_param_set |= (1 << alsa_stream->idx);

		if (!aoc_ring_flush_read_data(alsa_stream->dev->service, AOC_UP, 0)) {
			pr_err("ERR: capture ring buffer flush fail\n");
			err = -EINVAL;
		}
		return err;
	}

	if (alsa_stream->idx == UC_ULTRASONIC_RECORD)
		cmd_id = CMD_AUDIO_INPUT_ULTRASONIC_AP_SET_PARAMS_ID;
	else
		cmd_id = CMD_AUDIO_INPUT_MIC_RECORD_AP_SET_PARAMS_ID;
	AocCmdHdrSet(&(cmd.parent), cmd_id, sizeof(cmd));

	/* TODO: the output of spatial module is stereo */
	if (channels < 1 || channels > NUM_OF_BUILTIN_MIC) {
		pr_err("ERR: wrong channel number %u for capture\n", channels);
		err = -EINVAL;
		goto exit;
	}

	/* TODO: update this after DSP supports the runtime configuration */
	cmd.pdm_mask = 0;
	if (alsa_stream->idx == UC_ULTRASONIC_RECORD) {
		for (i = 0; i < ARRAY_SIZE(chip->buildin_us_mic_id_list); i++) {
			cmd.interleaving[i] = 0xff;
			mic_id = chip->buildin_us_mic_id_list[i];
			if (mic_id != -1) {
				cmd.interleaving[i] = mic_id;
				cmd.pdm_mask = cmd.pdm_mask | (1 << mic_id);
			}
		}
	} else {
		for (i = 0; i < ARRAY_SIZE(chip->buildin_mic_id_list); i++) {
			cmd.interleaving[i] = 0xff;
			mic_id = chip->buildin_mic_id_list[i];
			if (mic_id != -1) {
				cmd.interleaving[i] = mic_id;
				cmd.pdm_mask = cmd.pdm_mask | (1 << mic_id);
			}
		}
	}

	cmd.period_ms = 10; /*TODO: how to make it configuratable*/
	cmd.num_periods = 4; /*TODO: how to make it configuratable*/

	switch (samplerate) {
	case 96000:
		cmd.sample_rate = SR_96KHZ;
		break;
	case 48000:
		cmd.sample_rate = SR_48KHZ;
		break;
	case 44100:
		cmd.sample_rate = SR_44K1HZ;
		break;
	case 16000:
		cmd.sample_rate = SR_16KHZ;
		break;
	case 8000:
		cmd.sample_rate = SR_8KHZ;
		break;
	default:
		cmd.sample_rate = SR_48KHZ;
	}

	switch (bps) {
	case 32:
		cmd.requested_format.bits = WIDTH_32_BIT;
		break;
	case 24:
		/* TODO: tinycap limitation */
		cmd.requested_format.bits = WIDTH_32_BIT;
		break;
	case 16:
		cmd.requested_format.bits = WIDTH_16_BIT;
		break;
	case 8:
		cmd.requested_format.bits = WIDTH_8_BIT;
		break;
	default:
		cmd.requested_format.bits = WIDTH_32_BIT;
	}
	cmd.requested_format.sr = cmd.sample_rate; /* TODO: double check format*/
	cmd.requested_format.format = (pcm_float_fmt) ? FRMT_FLOATING_POINT : FRMT_FIXED_POINT;

	cmd.requested_format.chan = channels;

	if (chip->mic_spatial_module_enable && !aoc_pcm_is_mmap_raw(alsa_stream))
		cmd.mic_process_index = AP_MIC_PROCESS_SPATIAL;
	else
		cmd.mic_process_index = AP_MIC_PROCESS_RAW;

	err = aoc_audio_control(CMD_INPUT_CHANNEL, (uint8_t *)&cmd, sizeof(cmd), NULL, chip);
	if (err < 0) {
		pr_err("ERR:%d in capture parameter setup\n", err);
		goto exit;
	}

#if !IS_ENABLED(CONFIG_SOC_GS101)
	err = aoc_audio_capture_set_params2(alsa_stream);
	if (err < 0) {
		pr_err("ERR:%d in capture parameter2 setup\n", err);
		goto exit;
	}
#endif

	chip->capture_param_set |= (1 << alsa_stream->idx);

	if (alsa_stream->idx == UC_ULTRASONIC_RECORD) {
		/* Start the ultrasound mic */
		err = aoc_audio_us_capture_mic_prepare(chip);
		if (err < 0)
			pr_err("ERR:%d in audio us capture mic prepare\n", err);
	} else {
		/* Start the pdm/usb/bt mic */
		err = aoc_audio_capture_mic_prepare(chip);
		if (err < 0)
			pr_err("ERR:%d in audio capture mic prepare\n", err);
	}

	pr_debug("Flush aoc ring buffer\n");
	if (!aoc_ring_flush_read_data(alsa_stream->dev->service, AOC_UP, 0)) {
		pr_err("ERR: capture ring buffer flush fail\n");
		err = -EINVAL;
	}

	/* Resume the mmap and raw */
	if (alsa_stream->idx == UC_AUDIO_RECORD) {
		struct aoc_alsa_stream *target_alsa_stream;

		if (chip->capture_param_set & (1 << UC_MMAP_RECORD)) {
			target_alsa_stream = find_alsa_stream_by_device_idx(chip, UC_MMAP_RECORD);
			if (target_alsa_stream && target_alsa_stream->running) {
				aoc_mmap_capture_trigger(alsa_stream, START);
			}
		}

		if (chip->capture_param_set & (1 << UC_LOW_LATENCY_AUDIO_RECORD)) {
			target_alsa_stream =
				find_alsa_stream_by_device_idx(chip, UC_LOW_LATENCY_AUDIO_RECORD);
			if (target_alsa_stream && target_alsa_stream->running) {
				aoc_raw_capture_trigger(alsa_stream, START);
			}
		}
	}

exit:
	return err;
}

static int aoc_audio_capture_spatial_module_trigger(struct aoc_chip *chip,
						    int record_cmd)
{
	int err = 0;
	struct CMD_HDR cmd;
	pr_info("%s: %d", __func__, record_cmd);
	AocCmdHdrSet(&cmd,
		     (record_cmd == START) ? CMD_AUDIO_INPUT_SPATIAL_START_ID :
						   CMD_AUDIO_INPUT_SPATIAL_STOP_ID,
		     sizeof(cmd));

	err = aoc_audio_control(CMD_INPUT_CHANNEL, (uint8_t *)&cmd, sizeof(cmd),
				NULL, chip);
	if (err < 0)
		pr_err("ERR:%d in spatial module trigger\n", err);

	return err;
}

int aoc_spatial_module_start(struct aoc_chip *chip)
{
	return aoc_audio_capture_spatial_module_trigger(chip, START);
}

int aoc_spatial_module_stop(struct aoc_chip *chip)
{
	return aoc_audio_capture_spatial_module_trigger(chip, STOP);
}

/* Start or stop the stream */
static int aoc_audio_capture_trigger(struct aoc_alsa_stream *alsa_stream, int record_cmd)
{
	int err = 0;
	struct aoc_chip *chip = alsa_stream->chip;

	pr_info("%s: %d", __func__, record_cmd);

	/* Update Buildin MIC broken state */
	if (chip->audio_capture_mic_source == BUILTIN_MIC && record_cmd == STOP) {
		chip->broken_detect_count %= NUM_OF_MIC_BROKEN_RECORD;
		aoc_buildin_mic_broken_get(
			chip, &chip->buildin_mic_broken_detect[chip->broken_detect_count++]);
	}

	if (alsa_stream->stream_type == NORMAL) {
		err = ap_data_control_trigger(chip, alsa_stream, record_cmd);
		if (err < 0)
			goto exit;
	}

	/* For mmap capture */
	if (alsa_stream->stream_type == MMAPED) {
		err = aoc_mmap_capture_trigger(alsa_stream, record_cmd);
		if (err < 0)
			pr_err("ERR:%d in aoc mmap capture start/stop\n", err);
	}

	/* For raw capture */
	if (alsa_stream->stream_type == RAW) {
		err = aoc_raw_capture_trigger(alsa_stream, record_cmd);
		if (err < 0)
			pr_err("ERR:%d in aoc raw capture start/stop\n", err);
	}

exit:
	if (err < 0)
		pr_err("ERR:%d in capture trigger\n", err);

	return err;
}

static int aoc_audio_get_parameters(int cmd_id, int block, int component, int key, int *val,
				    struct aoc_chip *chip)
{
	int err;
	struct CMD_AUDIO_INPUT_GET_PARAMETER cmd0;
	struct CMD_AUDIO_OUTPUT_GET_PARAMETER cmd1;

	if (cmd_id == CMD_AUDIO_INPUT_GET_PARAMETER_ID) /* Input channel */
	{
		AocCmdHdrSet(&(cmd0.parent), CMD_AUDIO_INPUT_GET_PARAMETER_ID, sizeof(cmd0));
		cmd0.block = block;
		cmd0.component = component;
		cmd0.key = key;
		err = aoc_audio_control(CMD_INPUT_CHANNEL, (uint8_t *)&cmd0, sizeof(cmd0),
					(uint8_t *)&cmd0, chip);

		if (!err && val)
			*val = cmd0.val;
	} else /*Output channel */
	{
		AocCmdHdrSet(&(cmd1.parent), CMD_AUDIO_OUTPUT_GET_PARAMETER_ID, sizeof(cmd1));
		cmd1.block = block;
		cmd1.component = component;
		cmd1.key = key;
		err = aoc_audio_control(CMD_OUTPUT_CHANNEL, (uint8_t *)&cmd1, sizeof(cmd1),
					(uint8_t *)&cmd1, chip);

		if (!err && val)
			*val = cmd1.val;
	}

	if (err < 0) {
		pr_err("ERR:%d in audio parameter get- block:%d,component:%d,key:%d\n", err, block,
		       component, key);
		return err;
	}

	return 0;
}

static int aoc_audio_set_parameters(int cmd_id, int block, int component, int key, int val,
				    struct aoc_chip *chip)
{
	int err;
	struct CMD_AUDIO_INPUT_SET_PARAMETER cmd0;
	struct CMD_AUDIO_OUTPUT_SET_PARAMETER cmd1;

	if (cmd_id == CMD_AUDIO_INPUT_SET_PARAMETER_ID) /* Input channel */
	{
		AocCmdHdrSet(&(cmd0.parent), CMD_AUDIO_INPUT_SET_PARAMETER_ID, sizeof(cmd0));
		cmd0.block = block;
		cmd0.component = component;
		cmd0.key = key;
		cmd0.val = val;
		err = aoc_audio_control(CMD_INPUT_CHANNEL, (uint8_t *)&cmd0, sizeof(cmd0), NULL,
					chip);
	} else /*Output channel */
	{
		AocCmdHdrSet(&(cmd1.parent), CMD_AUDIO_OUTPUT_SET_PARAMETER_ID, sizeof(cmd1));
		cmd1.block = block;
		cmd1.component = component;
		cmd1.key = key;
		cmd1.val = val;
		err = aoc_audio_control(CMD_OUTPUT_CHANNEL, (uint8_t *)&cmd1, sizeof(cmd1), NULL,
					chip);
	}

	if (err < 0) {
		pr_err("ERR:%d in audio parameter set- block:%d,component:%d,key:%d,val:%d\n", err,
		       block, component, key, val);
		return err;
	}

	return 0;
}

int aoc_incall_mic_sink_mute_get(struct aoc_chip *chip, int param, long *mute)
{
	int err;
	int cmd_id, block, component, key, value;

	if (param == INCALL_MIC_ID) /* Up link (mic) */
	{
		cmd_id = CMD_AUDIO_OUTPUT_GET_PARAMETER_ID;
		block = 19;
		component = 0;
		key = 16;
	} else /* Download link (sink) */
	{
		cmd_id = CMD_AUDIO_OUTPUT_GET_PARAMETER_ID;
		block = 19;
		component = 30;
		key = 16;
	}

	/* Send cmd to AOC */
	err = aoc_audio_get_parameters(cmd_id, block, component, key, &value, chip);
	if (err < 0) {
		pr_err("ERR:%d in incall mute get\n", err);
		return err;
	}

	if (mute)
		*mute = (value <= MUTE_DB) ? INCALL_MUTE : INCALL_UNMUTE;

	return 0;
}

int aoc_incall_mic_gain_set(struct aoc_chip *chip, int param, long gain)
{
	int err;
	int cmd_id, block, component, key;

	cmd_id = CMD_AUDIO_OUTPUT_SET_PARAMETER_ID;
	block = 19;
	key = 16;

	if (param == INCALL_MIC_ID) /* Up link (mic) */
		component = 0;
	else /* Download link (sink) */
		component = 30;

	/* Send cmd to AOC */
	err = aoc_audio_set_parameters(cmd_id, block, component, key, (int) gain, chip);
	if (err < 0) {
		pr_err("ERR:%d in incall mic gain set\n", err);
		return err;
	}

	return 0;
}

/* can we get dB gain directly */
int aoc_mic_record_gain_get(struct aoc_chip *chip, long *val)
{
	int err;
	int cmd_id, block, component, key, value;

	cmd_id = CMD_AUDIO_INPUT_GET_PARAMETER_ID;
	block = 136;
	component = 30;
	key = 16; /* for dB */

	/* Send cmd to AOC */
	err = aoc_audio_get_parameters(cmd_id, block, component, key, &value, chip);
	if (err < 0) {
		pr_err("ERR:%d in in mic record gain get\n", err);
		return err;
	}

	if (val)
		*val = value;

	return 0;
}

int aoc_mic_record_gain_set(struct aoc_chip *chip, long val)
{
	int err;
	int cmd_id, block, component, key, value;

	cmd_id = CMD_AUDIO_INPUT_SET_PARAMETER_ID;
	block = 136;
	component = 30;
	key = 16; /* for dB */
	value = val; /* TODO: db value in float? */

	/* Send cmd to AOC */
	err = aoc_audio_set_parameters(cmd_id, block, component, key, value, chip);
	if (err < 0) {
		pr_err("ERR:%d in mic record gain set\n", err);
		return err;
	}

	return 0;
}

int aoc_mmap_record_gain_get(struct aoc_chip *chip, long *val)
{
	int err;
	int cmd_id, block, component, key, value;

	cmd_id = CMD_AUDIO_INPUT_GET_PARAMETER_ID;
	block = 136;
	component = 0;
	key = 16; /* for dB */

	/* Send cmd to AOC */
	err = aoc_audio_get_parameters(cmd_id, block, component, key, &value, chip);
	if (err < 0) {
		pr_err("ERR:%d mmap mic record gain get\n", err);
		return err;
	}

	if (val)
		*val = value;

	return 0;
}

int aoc_mmap_record_gain_set(struct aoc_chip *chip, long val)
{
	int err;
	int cmd_id, block, component, key, value;

	cmd_id = CMD_AUDIO_INPUT_SET_PARAMETER_ID;
	block = 136;
	component = 0;
	key = 16; /* for dB */
	value = val;

	/* Send cmd to AOC */
	err = aoc_audio_set_parameters(cmd_id, block, component, key, value, chip);
	if (err < 0) {
		pr_err("ERR:%d mmap record gain set\n", err);
		return err;
	}

	return 0;
}

int aoc_buildin_mic_broken_get(struct aoc_chip *chip, int *val)
{
	int err;
	int cmd_id, block, component, key, value;

	cmd_id = CMD_AUDIO_INPUT_GET_PARAMETER_ID;
	block = 139; /* ABLOCK_INPUT_PDM_MIC */
	component = 0;
	key = 16;

	/* Send cmd to AOC */
	err = aoc_audio_get_parameters(cmd_id, block, component, key, &value, chip);
	if (err < 0) {
		pr_err("ERR:%d %s\n", err, __func__);
		return err;
	}

	if (val)
		*val = value;
	return 0;
}

int aoc_audio_capture_eraser_enable(struct aoc_chip *chip, long enable)
{
	int cmd_id, err = 0;

	cmd_id = (enable == 1) ? CMD_AUDIO_INPUT_MIC_RECORD_AP_ENABLE_AEC_ID :
				       CMD_AUDIO_INPUT_MIC_RECORD_AP_DISABLE_AEC_ID;
	err = aoc_audio_control_simple_cmd(CMD_INPUT_CHANNEL, cmd_id, chip);
	if (err < 0) {
		pr_err("ERR:%d in audio capture eraser %s\n", err, (enable) ? "enable" : "disable");
		return err;
	}

	return 0;
}

int aoc_hotword_tap_enable(struct aoc_chip *chip, long enable)
{
	if (chip->hotword_supported) {
		int cmd_id, err = 0;

		cmd_id = (enable == 1) ? CMD_AUDIO_INPUT_HOTWORD_ENABLE_HOTWORD_TAP_ID :
						CMD_AUDIO_INPUT_HOTWORD_DISABLE_HOTWORD_TAP_ID;
		err = aoc_audio_control_simple_cmd(CMD_INPUT_CHANNEL, cmd_id, chip);
		if (err < 0) {
			pr_err("ERR:%d in hotword tap %s\n", err, (enable) ? "enable" : "disable");
			return err;
		}

		return 0;
	} else {
		pr_err("WARN:hotword is not supported on this device\n");
		return 0;
	}
}

int aoc_load_cca_module(struct aoc_chip *chip, long load)
{
	int cmd_id, err = 0;

	cmd_id = (load == 1) ? CMD_AUDIO_OUTPUT_VOICE_CCA_START_ID :
				       CMD_AUDIO_OUTPUT_VOICE_CCA_STOP_ID;
	err = aoc_audio_control_simple_cmd(CMD_OUTPUT_CHANNEL, cmd_id, chip);

	return err;
}

int aoc_enable_cca_on_voip(struct aoc_chip *chip, long enable)
{
	int cmd_id, err = 0;

	cmd_id = (enable == 1) ?
			CMD_AUDIO_OUTPUT_VOICE_ENABLE_CCA_ON_VOIP_ID :
			CMD_AUDIO_OUTPUT_VOICE_DISABLE_CCA_ON_VOIP_ID;
	err = aoc_audio_control_simple_cmd(CMD_OUTPUT_CHANNEL, cmd_id, chip);

	return err;
}

int aoc_lvm_enable_get(struct aoc_chip *chip, long *enable)
{
	int err;
	struct CMD_AUDIO_OUTPUT_GET_PARAMETER cmd;

	AocCmdHdrSet(&(cmd.parent), CMD_AUDIO_OUTPUT_GET_PARAMETER_ID, sizeof(cmd));

	cmd.block = 14; /* ABLOCK_DCDOFF */
	cmd.component = 0; /* LVM */
	cmd.key = 16; /* ASP_LVM_PARAM_ENABLE */

	/* Send cmd to AOC */
	err = aoc_audio_control(CMD_OUTPUT_CHANNEL, (uint8_t *)&cmd, sizeof(cmd), (uint8_t *)&cmd,
		chip);
	if (err < 0) {
		pr_err("ERR:%d in lvm get\n", err);
		return err;
	}

	pr_debug("lvm: %s\n", cmd.val ? "enabled" : "disabled");
	if (enable)
		*enable = cmd.val;

	return 0;
}

int aoc_lvm_enable_set(struct aoc_chip *chip, long enable)
{
	int err;
	struct CMD_AUDIO_OUTPUT_SET_PARAMETER cmd;

	AocCmdHdrSet(&(cmd.parent), CMD_AUDIO_OUTPUT_SET_PARAMETER_ID, sizeof(cmd));

	cmd.block = 14; /* ABLOCK_DCDOFF */
	cmd.component = 0;
	cmd.key = 16;
	cmd.val = enable;
	pr_debug("lvm: %s\n", enable ? "enabled" : "disabled");

	/* Send cmd to AOC */
	err = aoc_audio_control(CMD_OUTPUT_CHANNEL, (uint8_t *)&cmd, sizeof(cmd), NULL, chip);
	if (err < 0) {
		pr_err("ERR:%d in lvm set\n", err);
		return err;
	}

	return 0;
}

int aoc_decoder_ref_enable_get(struct aoc_chip *chip, long *enable)
{
	int err;
	struct CMD_AUDIO_OUTPUT_GET_PARAMETER cmd;

	AocCmdHdrSet(&(cmd.parent), CMD_AUDIO_OUTPUT_GET_PARAMETER_ID, sizeof(cmd));

	cmd.block = 14;
	cmd.component = 1;
	cmd.key = 0;

	/* Send cmd to AOC */
	err = aoc_audio_control(CMD_OUTPUT_CHANNEL, (uint8_t *)&cmd, sizeof(cmd), (uint8_t *)&cmd,
		chip);
	if (err < 0) {
		pr_err("ERR:%d in decoder ref get\n", err);
		return err;
	}

	pr_debug("decoder ref: %s\n", cmd.val ? "enabled" : "disabled");
	if (enable)
		*enable = cmd.val;

	return 0;
}

int aoc_decoder_ref_enable_set(struct aoc_chip *chip, long enable)
{
	int err;
	struct CMD_AUDIO_OUTPUT_SET_PARAMETER cmd;

	AocCmdHdrSet(&(cmd.parent), CMD_AUDIO_OUTPUT_SET_PARAMETER_ID, sizeof(cmd));

	cmd.block = 14;
	cmd.component = 1;
	cmd.key = 0;
	cmd.val = enable;
	pr_debug("decoder ref: %s\n", enable ? "enabled" : "disabled");

	/* Send cmd to AOC */
	err = aoc_audio_control(CMD_OUTPUT_CHANNEL, (uint8_t *)&cmd, sizeof(cmd), NULL, chip);
	if (err < 0) {
		pr_err("ERR:%d in decoder ref set\n", err);
		return err;
	}

	return 0;
}

int aoc_compr_offload_linear_gain_get(struct aoc_chip *chip, long *val)
{
	int err;
	struct CMD_AUDIO_OUTPUT_DEC_CH_GAIN cmd;

	AocCmdHdrSet(&(cmd.parent), CMD_AUDIO_OUTPUT_DEC_CH_GAIN_GET_ID, sizeof(cmd));

	cmd.ch_bit_mask = 0x03;

	/* Send cmd to AOC */
	err = aoc_audio_control(CMD_OUTPUT_CHANNEL, (uint8_t *)&cmd, sizeof(cmd), (uint8_t *)&cmd,
				chip);

	if (err < 0) {
		pr_err("ERR:%d in decoder ref get\n", err);
		return err;
	}

	if (val) {
		val[0] = cmd.ch_gains[0];
		val[1] = cmd.ch_gains[1];
	}

	return 0;
}

int aoc_compr_offload_linear_gain_set(struct aoc_chip *chip, long *val)
{
	int err;
	struct CMD_AUDIO_OUTPUT_DEC_CH_GAIN cmd;

	AocCmdHdrSet(&(cmd.parent), CMD_AUDIO_OUTPUT_DEC_CH_GAIN_SET_ID, sizeof(cmd));

	cmd.ch_bit_mask = 0x03;

	cmd.ch_gains[0] = val[0];
	cmd.ch_gains[1] = val[1];

	if (val[0] == 0 && val[1] == 0) {
		err = aoc_audio_control_simple_cmd(CMD_OUTPUT_CHANNEL,
						CMD_AUDIO_OUTPUT_DEC_RESET_CURRENT_GAIN_ID,
						chip);
		if (err < 0) {
			pr_err("ERR:%d in aoc compr reset gain\n", err);
			return err;
		}
		/* To allow the reset finished in aoc */
		msleep(100);
	}

	/* Send cmd to AOC */
	err = aoc_audio_control(CMD_OUTPUT_CHANNEL, (uint8_t *)&cmd, sizeof(cmd), NULL, chip);
	if (err < 0) {
		pr_err("ERR:%d in compr offload linear gain set\n", err);
		return err;
	}

	return 0;
}

#if IS_ENABLED(CONFIG_SOC_ZUMA)
int aoc_mel_enable(struct aoc_chip *chip, int enable)
{
	int err = 0;
	struct CMD_AUDIO_OUTPUT_MEL_STATE cmd;

	AocCmdHdrSet(&(cmd.parent), CMD_AUDIO_OUTPUT_MEL_STATE_ID, sizeof(cmd));

	cmd.enable = enable ? true : false;

	err = aoc_audio_control(CMD_OUTPUT_CHANNEL, (uint8_t *)&cmd, sizeof(cmd), (uint8_t *)&cmd,
		chip);
	if (err < 0)
		pr_err("ERR:%d in mel enable\n", err);

	return err;
}

int aoc_mel_rs2_set(struct aoc_chip *chip, long *rs2)
{
	int err = 0;
	struct CMD_AUDIO_OUTPUT_MEL_RS2 cmd;
	uint32_t tmp;

	AocCmdHdrSet(&(cmd.parent), CMD_AUDIO_OUTPUT_MEL_SET_RS2_ID, sizeof(cmd));

	tmp = (uint32_t)rs2[0];
	cmd.rs2_value = *(float *)(&tmp);

	err = aoc_audio_control(CMD_OUTPUT_CHANNEL, (uint8_t *)&cmd, sizeof(cmd), (uint8_t *)&cmd,
		chip);
	if (err < 0)
		pr_err("ERR:%d in mel rs2 set\n", err);

	return err;
}

int aoc_mel_rs2_get(struct aoc_chip *chip, long *rs2)
{
	int err = 0;
	struct CMD_AUDIO_OUTPUT_MEL_RS2 cmd;

	AocCmdHdrSet(&(cmd.parent), CMD_AUDIO_OUTPUT_MEL_GET_RS2_ID, sizeof(cmd));

	err = aoc_audio_control(CMD_OUTPUT_CHANNEL, (uint8_t *)&cmd, sizeof(cmd), (uint8_t *)&cmd,
		chip);
	if (err < 0)
		pr_err("ERR:%d in mel rs2 get\n", err);
	else
		*rs2 = *(uint32_t *)&cmd.rs2_value;

	return err;
}
#endif

#if !(IS_ENABLED(CONFIG_SOC_GS101) || IS_ENABLED(CONFIG_SOC_GS201))
int aoc_compr_offload_playback_rate_set(struct aoc_chip *chip, long *val)
{
	int err;
	struct CMD_AUDIO_OUTPUT_DECODER_CFG_SPEED cmd;
	uint32_t tmp;

	AocCmdHdrSet(&(cmd.parent), CMD_AUDIO_OUTPUT_DECODER_CFG_SPEED_ID, sizeof(cmd));

	tmp = (uint32_t)val[0];
	cmd.speed = *(float *)(&tmp);
	tmp = (uint32_t)val[1];
	cmd.pitch = *(float *)(&tmp);
	cmd.stretch_mode = (int32_t)val[2];
	cmd.fallback_mode = (int32_t)val[3];

	tmp = *(uint32_t *)&cmd.speed;
	pr_info("speed = %x vs %lx\n", tmp, val[0]);
	tmp = *(uint32_t *)&cmd.pitch;
	pr_info("pitch = %x vs %lx\n", tmp, val[1]);
	pr_info("stretch_mode = %x vs %lx\n", cmd.stretch_mode, val[2]);
	pr_info("fallback_mode = %x vs %lx\n", cmd.fallback_mode, val[3]);

	chip->decoder_cfg_speed = cmd;

	/* Send cmd to AOC */
	err = aoc_audio_control(CMD_OUTPUT_CHANNEL, (uint8_t *)&cmd, sizeof(cmd), NULL, chip);
	if (err < 0) {
		pr_err("ERR:%d in compr offload playback rate set\n", err);
		return err;
	}

	return 0;
}
#endif

int aoc_sidetone_enable(struct aoc_chip *chip, int enable)
{
	int err = 0;
	int cmd = (enable) ? START : STOP;
	int src, dest;

	src = SIDETONE;
	dest = SINK_SPEAKER;

	pr_info("sidetone: %s \n", (enable) ? "Enabled" : "Disabled");

	/* Source on/off */
	err = aoc_audio_playback_source_on(chip, cmd, src);
	if (err < 0) {
		pr_err("ERR=%d, sidetone %s fail in source(%d) on/off\n", err,
		       (enable) ? "START" : "STOP", src);
		goto exit;
	}

	/* Bind/ubind - source 11 and sink 0 */
	err = aoc_audio_path_bind(src, dest, cmd, chip);
	if (err < 0)
		pr_err("ERR=%d, sidestone %s fail in binding src:%d - dest:%d\n", err,
		       (enable) ? "START" : "STOP", src, dest);

exit:
	return err;
}

int aoc_mic_loopback(struct aoc_chip *chip, int enable)
{
	int err;
	struct CMD_AUDIO_INPUT_ENABLE_MIC_LOOPBACK cmd;

	AocCmdHdrSet(&(cmd.parent),
		     ((enable == 1) ? CMD_AUDIO_INPUT_MIC_LOOPBACK_START_ID :
				      CMD_AUDIO_INPUT_MIC_LOOPBACK_STOP_ID),
		     sizeof(cmd));
	cmd.sample_rate = SR_48KHZ;

	err = aoc_audio_control(CMD_INPUT_CHANNEL, (uint8_t *)&cmd, sizeof(cmd),
				NULL, chip);
	if (err < 0)
		pr_err("ERR:%d in mic loopback\n", err);

	return err;
}

/*
 * For capture: start recording
 * for playback: source on and bind source/sinks
 *
 * TODO: Capturing from four mics on board differ from other sources
 * (BT, USB,... I2S interface from headphone)
 */
int aoc_audio_start(struct aoc_alsa_stream *alsa_stream)
{
	int err = 0;
	int src;

	/* TODO: compress offload may not use START for source on, CHECK! */
	if (alsa_stream->cstream ||
	    (alsa_stream->substream->stream == SNDRV_PCM_STREAM_PLAYBACK)) {
		src = alsa_stream->entry_point_idx;
		err = aoc_audio_playback_trigger_source(alsa_stream, START,
							src);
		if (err < 0)
			pr_err("ERR:%d in source on\n", err);
	} else {
		err = aoc_audio_capture_trigger(alsa_stream, START);
		if (err < 0)
			pr_err("ERR:%d in capture start\n", err);
	}

	return err;
}

/*
 * For capture: stop recording
 * For playback: source off and unbind source/sinks
 */
int aoc_audio_stop(struct aoc_alsa_stream *alsa_stream)
{
	int err = 0;
	int src;

	if (alsa_stream->cstream ||
	    (alsa_stream->substream->stream == SNDRV_PCM_STREAM_PLAYBACK)) {
		src = alsa_stream->entry_point_idx;
		err = aoc_audio_playback_trigger_source(alsa_stream, STOP, src);
		if (err < 0)
			pr_err("ERR:%d in source off\n", err);
	} else {
		err = aoc_audio_capture_trigger(alsa_stream, STOP);
		if (err < 0)
			pr_err("ERR:%d in capture stop\n", err);
	}

	return err;
}

static int aoc_audio_hifi_start(struct aoc_alsa_stream *alsa_stream)
{
	int cmd_id, err = 0;
	struct aoc_chip *chip = alsa_stream->chip;

	if (alsa_stream->substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		cmd_id = CMD_AUDIO_OUTPUT_USB_HIFI_PLAYBACK_START_ID;
		err = aoc_audio_control_simple_cmd(CMD_OUTPUT_CHANNEL, cmd_id, chip);
		if (err < 0)
			pr_err("ERR:%d in usb hifi playback start on\n", err);
	} else {
		cmd_id = CMD_AUDIO_INPUT_USB_HIFI_CAPTURE_START_ID;
		err = aoc_audio_control_simple_cmd(CMD_INPUT_CHANNEL, cmd_id, chip);
		if (err < 0)
			pr_err("ERR:%d in usb hifi capture start on\n", err);
	}

	return err;
}

static int aoc_audio_hifi_stop(struct aoc_alsa_stream *alsa_stream)
{
	int cmd_id, err = 0;
	struct aoc_chip *chip = alsa_stream->chip;

	if (alsa_stream->substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		cmd_id = CMD_AUDIO_OUTPUT_USB_HIFI_PLAYBACK_STOP_ID;
		err = aoc_audio_control_simple_cmd(CMD_OUTPUT_CHANNEL, cmd_id, chip);
		if (err < 0)
			pr_err("ERR:%d in usb hifi playback stop on\n", err);
	} else {
		cmd_id = CMD_AUDIO_INPUT_USB_HIFI_CAPTURE_STOP_ID;
		err = aoc_audio_control_simple_cmd(CMD_INPUT_CHANNEL, cmd_id, chip);
		if (err < 0)
			pr_err("ERR:%d in usb hifi capture stop on\n", err);
	}

	return err;
}

static int aoc_audio_android_aec_start(struct aoc_alsa_stream *alsa_stream)
{
	int cmd_id, err = 0;
	struct aoc_chip *chip = alsa_stream->chip;

	cmd_id = CMD_AUDIO_OUTPUT_SPKR_ANDROID_AEC_START_ID;
	err = aoc_audio_control_simple_cmd(CMD_OUTPUT_CHANNEL, cmd_id, chip);
	if (err < 0) {
		pr_err("ERR:%d in android aec capture start\n", err);
		return err;
	}

	return 0;
}

static int aoc_audio_android_aec_stop(struct aoc_alsa_stream *alsa_stream)
{
	int cmd_id, err = 0;
	struct aoc_chip *chip = alsa_stream->chip;

	cmd_id = CMD_AUDIO_OUTPUT_SPKR_ANDROID_AEC_STOP_ID;
	err = aoc_audio_control_simple_cmd(CMD_OUTPUT_CHANNEL, cmd_id, chip);
	if (err < 0) {
		pr_err("ERR:%d in android aec capture stop\n", err);
		return err;
	}

	return 0;
}

static int aoc_audio_capture_inject_params_check(struct aoc_alsa_stream *alsa_stream)
{
	int err = 0;
	int active_mic = 0;
	int i;
	struct aoc_alsa_stream *active_capture_stream;
	if (alsa_stream->stream_type == CAP_INJ) {
		active_capture_stream = find_alsa_stream_by_capture_stream(alsa_stream->chip);
		if (active_capture_stream == NULL) {
			pr_warn("No active capture stream.\n");
		}
		/* Capture injection is replacing the PDM mic data NOT the recording data,
		 * So it need to compare with the PDM mic format */
		/* checking if format is match */
		/*
		if (alsa_stream->params_rate != active_capture_stream->params_rate) {
			pr_err("[CAP_INJ] sample rate mismatch %d vs %d\n",
				alsa_stream->params_rate, active_capture_stream->params_rate);
			err = -EINVAL;
		}
		if (alsa_stream->pcm_format_width != active_capture_stream->pcm_format_width) {
			pr_err("[CAP_INJ] width mismatch %d vs %d\n",
				alsa_stream->pcm_format_width,
				active_capture_stream->pcm_format_width);
			err = -EINVAL;
		}
		*/
		for (i = 0; i < ARRAY_SIZE(alsa_stream->chip->buildin_mic_id_list); i++) {
			if (alsa_stream->chip->buildin_mic_id_list[i] != -1)
				active_mic ++;
		}
		if (alsa_stream->channels != active_mic) {
			pr_warn("[CAP_INJ] channels and active mic mismatch %d vs %d\n",
				alsa_stream->channels, active_mic);
		}
	} else {
		pr_err("[CAP_INJ] incorrect stream type %d\n", alsa_stream->stream_type);
		err = -EINVAL;
	}
	return err;
}

static int aoc_audio_capture_inject_start(struct aoc_alsa_stream *alsa_stream)
{
	int err = 0;
	struct aoc_chip *chip = alsa_stream->chip;

	err = aoc_audio_capture_inject_params_check(alsa_stream);
	if (err < 0)
		return err;

	err = aoc_audio_control_simple_cmd(CMD_INPUT_CHANNEL,
			CMD_AUDIO_INPUT_CAPTURE_INJECTION_START_ID, chip);
	if (err < 0) {
		pr_err("ERR:%d in audio capture inject start\n", err);
		return err;
	}

	return 0;
}

static int aoc_audio_capture_inject_stop(struct aoc_alsa_stream *alsa_stream)
{
	int err = 0;
	struct aoc_chip *chip = alsa_stream->chip;

	err = aoc_audio_control_simple_cmd(CMD_INPUT_CHANNEL,
			CMD_AUDIO_INPUT_CAPTURE_INJECTION_STOP_ID, chip);
	if (err < 0) {
		pr_err("ERR:%d in audio capture inject stop\n", err);
		return err;
	}

	return 0;
}

int aoc_audio_incall_start(struct aoc_alsa_stream *alsa_stream)
{
	int stream, err = 0;
	struct aoc_chip *chip = alsa_stream->chip;
	struct snd_soc_pcm_runtime *rtd = alsa_stream->substream->private_data;

	if (alsa_stream->stream_type == CAP_INJ)
		return aoc_audio_capture_inject_start(alsa_stream);

	if (alsa_stream->stream_type == HIFI)
		return aoc_audio_hifi_start(alsa_stream);

	if (alsa_stream->stream_type == ANDROID_AEC)
		return aoc_audio_android_aec_start(alsa_stream);

	if (alsa_stream->stream_type == HOTWORD_TAP)
		return 0;

	if (!rtd) {
		pr_err("ERR: invalid pcm runtime structure\n");
		return 0;
	}

	if (alsa_stream->substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		/* Mapping the incall playback to specific stream ring.*/
		switch (rtd->dai_link->id) {
			case IDX_INCALL_PB0_RX:
				stream = 0;
				break;
			case IDX_INCALL_PB1_RX:
				stream = 1;
				break;
			case IDX_INCALL_PB2_RX:
				stream = 2;
				break;
			default:
			/* TODO: Keep original logic, but should be removed if it is useless.*/
				stream = 2;
				break;
		}
		err = aoc_incall_playback_enable_set(chip, stream, 1);
		if (err < 0)
			pr_err("ERR:%d in incall playback start on\n", err);
	} else {
		/* Mapping the incall playback to specific stream ring.*/
		switch (rtd->dai_link->id) {
			case IDX_INCALL_CAP0_TX:
				stream = 0;
				break;
			case IDX_INCALL_CAP1_TX:
				stream = 1;
				break;
			case IDX_INCALL_CAP2_TX:
				stream = 2;
				break;
			case IDX_INCALL_CAP3_TX:
				stream = 3;
				break;
			default:
				pr_err("ERR: dai_link id %d for incall_cap", rtd->dai_link->id);
				return err;
		}
		err = aoc_incall_capture_enable_set(chip, stream,
						    chip->incall_capture_state[stream]);
		if (err < 0)
			pr_err("ERR:%d in incall capture start on\n", err);
	}

	return err;
}

int aoc_audio_incall_stop(struct aoc_alsa_stream *alsa_stream)
{
	int stream, err = 0;
	struct aoc_chip *chip = alsa_stream->chip;
	struct snd_soc_pcm_runtime *rtd = alsa_stream->substream->private_data;

	if (alsa_stream->stream_type == CAP_INJ)
		return aoc_audio_capture_inject_stop(alsa_stream);

	if (alsa_stream->stream_type == HIFI)
		return aoc_audio_hifi_stop(alsa_stream);

	if (alsa_stream->stream_type == ANDROID_AEC)
		return aoc_audio_android_aec_stop(alsa_stream);

	if (alsa_stream->stream_type == HOTWORD_TAP)
		return 0;

	if (!rtd) {
		pr_err("ERR: invalid pcm runtime structure\n");
		return 0;
	}

	if (alsa_stream->substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		/* Mapping the incall playback to specific stream ring.*/
		switch (rtd->dai_link->id) {
			case IDX_INCALL_PB0_RX:
				stream = 0;
				break;
			case IDX_INCALL_PB1_RX:
				stream = 1;
				break;
			case IDX_INCALL_PB2_RX:
				stream = 2;
				break;
			default:
			/* TODO: Keep original logic, but should be removed if it is useless.*/
				stream = 2;
				break;
		}
		err = aoc_incall_playback_enable_set(chip, stream, 0);
		if (err < 0)
			pr_err("ERR:%d in incall playback start on\n", err);
	} else {
		/* Mapping the incall playback to specific stream ring.*/
		switch (rtd->dai_link->id) {
			case IDX_INCALL_CAP0_TX:
				stream = 0;
				break;
			case IDX_INCALL_CAP1_TX:
				stream = 1;
				break;
			case IDX_INCALL_CAP2_TX:
				stream = 2;
				break;
			case IDX_INCALL_CAP3_TX:
				stream = 3;
				break;
			default:
				pr_err("ERR: dai_link id %d for incall_cap", rtd->dai_link->id);
				return err;
		}
		err = aoc_incall_capture_enable_set(chip, stream, 0);
		if (err < 0)
			pr_err("ERR:%d in incall capture start on\n", err);
	}

	return err;
}

int aoc_audio_voip_start(struct aoc_alsa_stream *alsa_stream)
{
	return 0;
}

int aoc_audio_voip_stop(struct aoc_alsa_stream *alsa_stream)
{
	return 0;
}

/* TODO: this function is modified to deal with the issue where ALSA appl_ptr
 * and the reader pointer in AoC ringer buffer are out-of-sync due to overflow
 */
int aoc_audio_read(struct aoc_alsa_stream *alsa_stream, void *dest,
		   uint32_t count)
{
	int err = 0;
	void *tmp;
	struct aoc_service_dev *dev = alsa_stream->dev;
	uint32_t avail;

	tmp = (void *)(alsa_stream->substream->runtime->dma_area);

	memset(tmp, 0, count);

	avail = aoc_ring_bytes_available_to_read(dev->service, AOC_UP);

	if (unlikely(avail < count)) {
		pr_err("ERR: overrun in audio capture. avail = %d, toread = %d\n",
		       avail, count);
	}

	/* Only read bytes available in the ring buffer */
	avail = min(avail, count);
	if (avail) {
		err = aoc_service_read(dev, (void *)tmp, avail, NONBLOCKING);
		if (unlikely(err != avail)) {
			pr_err("ERR: %d bytes not read from ring buffer\n",
			       count - err);
			err = -EFAULT;
			goto out;
		}
	}

	/* If AoC is not ready, force read data to zero */
	if (!aoc_online_state(dev))
		memset(tmp, 0, count);

	err = copy_to_user(dest, tmp, count);
	if (err != 0) {
		pr_err("ERR: %d bytes not copied to user space\n", err);
		err = -EFAULT;
	}

out:
	return err < 0 ? err : 0;
}

int aoc_audio_write(struct aoc_alsa_stream *alsa_stream, void *src,
		    uint32_t count)
{
	int err = 0;
	struct aoc_service_dev *dev = alsa_stream->dev;
	void *tmp;
	int avail;
	uint32_t block_size;

	avail = aoc_ring_bytes_available_to_write(dev->service, AOC_DOWN);
	if (unlikely(avail < count)) {
		pr_err("ERR: inconsistent write/read pointers, avail = %d, towrite = %u\n",
		       avail, count);
		err = -EFAULT;
		goto out;
	}

	if (alsa_stream->substream) {
		tmp = alsa_stream->substream->runtime->dma_area;
		block_size = count;
	} else {
		tmp = alsa_stream->cstream->runtime->buffer;
		block_size = alsa_stream->offload_temp_data_buf_size;
	}

	while (count > 0) {
		if (count < block_size)
			block_size = count;

		if (alsa_stream->cstream)
			pr_debug("compr offload, count: %d, blocksize: %d\n", count, block_size);

		err = copy_from_user(tmp, src, block_size);
		if (err != 0) {
			pr_err("ERR: %d bytes not read from user space\n", err);
			err = -EFAULT;
			goto out;
		}

		err = aoc_service_write(dev, tmp, block_size, NONBLOCKING);
		if (err != block_size) {
			pr_err("ERR: unwritten data - %d bytes\n", block_size - err);
			err = -EFAULT;
		}

		count -= block_size;
		src += block_size;
	}

out:
	return err < 0 ? err : 0;
}

int aoc_displayport_service_alloc(struct aoc_chip *chip)
{
	struct aoc_service_dev *dev;
	int err = 0;
	if (!chip)
		return -ENODEV;
	if (mutex_lock_interruptible(&chip->audio_cmd_chan_mutex))
		return -EINTR;

	err = alloc_aoc_audio_service(AOC_DISPLAYPORT_SERVICE, &dev, NULL, NULL);
	if (err < 0)
		goto error;

	chip->dp_starting = 0;
	chip->dp_dev = dev;
error:
	mutex_unlock(&chip->audio_cmd_chan_mutex);
	return err;
}

int aoc_displayport_service_free(struct aoc_chip *chip)
{
	struct aoc_service_dev *dev;
	if (!chip)
		return -ENODEV;
	if (mutex_lock_interruptible(&chip->audio_cmd_chan_mutex))
		return -EINTR;

	chip->dp_starting = 0;
	dev = chip->dp_dev;
	chip->dp_dev = NULL;
	if (dev)
		free_aoc_audio_service(AOC_DISPLAYPORT_SERVICE, dev);
	mutex_unlock(&chip->audio_cmd_chan_mutex);
	return 0;
}

int aoc_displayport_flush(struct aoc_chip *chip)
{
	struct aoc_service_dev *dev;
	int err = 0;

	if (!chip)
		return -ENODEV;

	dev = chip->dp_dev;

	if (!dev)
		return -EINVAL;

	if (!aoc_ring_flush_read_data(dev->service, AOC_UP, 0)) {
		dev_err(&dev->dev, "flush dp data failed\n");
	}

	return err;
}

int aoc_displayport_read(struct aoc_chip *chip, void *dest, size_t buf_size)
{
	struct aoc_service_dev *dev;
	int err = 0;
	size_t avail;

	if (!chip || !dest)
		return -ENODEV;

	dev = chip->dp_dev;

	if (!dev)
		return -EINVAL;

	memset(dest, 0, buf_size);

	avail = aoc_ring_bytes_available_to_read(dev->service, AOC_UP);

	if (avail == 0) {
		dev_err(&dev->dev, "ERR: no data in diaplayport read\n");
		err = -EINVAL;
		goto done;
	}
	if (chip->dp_starting == 0) {
		if (chip->dp_start_threshold == 0) {
			dev_warn(&dev->dev, "use default start threshold\n");
			chip->dp_start_threshold = buf_size * 2;
		}
		if (avail < chip->dp_start_threshold) {
			dev_warn(&dev->dev,
				"Wait more dp buffer to start. avail = %zu, threshold = %zu\n",
				avail, chip->dp_start_threshold);
			err = -EAGAIN;
			goto done;
		}
		chip->dp_starting = 1;
	}

	if (unlikely(avail < buf_size)) {
		dev_err(&dev->dev, "ERR: overrun in displayport read. avail = %zu, toread = %zu\n",
		       avail, buf_size);
		err = -EAGAIN;
		goto done;
	}

	/* Only read bytes available in the ring buffer */
	avail = min(avail, buf_size);
	if (!avail)
		goto done;

	err = aoc_service_read(dev, (void *)dest, avail, NONBLOCKING);
	if (unlikely(err != avail)) {
		dev_err(&dev->dev, "ERR: %zu bytes not read from ring buffer\n",
		       avail - err);
		err = -EFAULT;
	}

done:
	return err;
}

/* PCM channel setup ??? */
static int aoc_audio_set_ctls_chan(struct aoc_alsa_stream *alsa_stream,
				   struct aoc_chip *chip)
{
	int err = 0;
	int src, dst, i;

	pr_debug(" Setting ALSA  volume(%d)\n", chip->volume);

	if (alsa_stream->substream &&
	    alsa_stream->substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		return 0;

	src = alsa_stream->entry_point_idx;
	for (i = 0; i < MAX_NUM_OF_SINKS_PER_STREAM; i++) {
		dst = alsa_stream->chip->sink_id_list[i];
		if (dst == -1)
			continue;

		err = aoc_audio_volume_set(chip, chip->volume, src, dst);
		if (err < 0) {
			pr_err("ERR:%d in volume setting\n", err);
			goto out;
		}
	}

out:
	return err;
}

int aoc_audio_set_ctls(struct aoc_chip *chip)
{
	int i;
	int err = 0;

	/* change ctls for all substreams */
	for (i = 0; i < MAX_NUM_OF_SUBSTREAMS; i++) {
		if (chip->avail_substreams & (1 << i)) {
			pr_debug(" Setting %llu stream i =%d\n",
				 chip->avail_substreams, i);

			if (!chip->alsa_stream[i]) {
				pr_debug(
					" No ALSA stream available?! %i:%p (%llu)\n",
					i, chip->alsa_stream[i],
					chip->avail_substreams);
				err = 0;
			} else if (aoc_audio_set_ctls_chan(chip->alsa_stream[i],
							   chip) != 0) {
				pr_err("ERR: couldn't set controls for stream %d\n",
				       i);
				err = -EINVAL;
			} else
				pr_debug("controls set for stream %d\n", i);
		}
	}
	return err;
}

int aoc_audio_set_params(struct aoc_alsa_stream *alsa_stream, uint32_t channels,
			 uint32_t samplerate, uint32_t bps, bool pcm_float_fmt, int source_mode)
{
	int err = 0;
	pr_debug("setting channels(%u), samplerate(%u), bits-per-sample(%u)\n", channels,
		 samplerate, bps);

	if (alsa_stream->cstream || alsa_stream->substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		err = aoc_audio_playback_set_params(alsa_stream, channels, samplerate, bps,
						    pcm_float_fmt, source_mode);
		if (err < 0)
			pr_err("ERR:%d playback audio param set fails\n", err);
	} else {
		err = aoc_audio_capture_set_params(alsa_stream, channels, samplerate, bps,
						   pcm_float_fmt);
		if (err < 0) {
			pr_err("ERR:%d capture audio param set fails\n", err);
			goto exit;
		}
	}

exit:
	return err;
}

int aoc_eraser_aec_reference_set(struct aoc_chip *chip, long aec_input_source)
{
	int err;
	struct CMD_AUDIO_INPUT_FEEDBACK_SRC_SELECT_REF cmd;

	AocCmdHdrSet(&(cmd.parent), CMD_AUDIO_INPUT_HOTWORD_SELECT_AEC_REF_ID, sizeof(cmd));

	cmd.aec_ref_index = 0;

	switch (aec_input_source) {
	case DEFAULT_PLAYBACK:
	case SPEAKER_PLAYBACK:
		cmd.aec_ref_index = FEEDBACK_SRC_AEC_SPKR_INPUT_INDEX;
		break;

	case USB_PLAYBACK:
		cmd.aec_ref_index = FEEDBACK_SRC_AEC_USB_INPUT_INDEX;
		break;

	case BT_PLAYBACK:
		cmd.aec_ref_index = FEEDBACK_SRC_AEC_BT_INPUT_INDEX;
		break;

	default:
		pr_err("ERR: Eraser AEC ref source wrong %ld, fall back to default!",
		       aec_input_source);
		cmd.aec_ref_index = FEEDBACK_SRC_AEC_SPKR_INPUT_INDEX;
	}

	pr_notice("Eraser AEC ref source set as %ld\n", aec_input_source);

	err = aoc_audio_control(CMD_INPUT_CHANNEL, (uint8_t *)&cmd, sizeof(cmd), NULL, chip);
	if (err < 0)
		pr_err("ERR:%d in eraser aec ref source set!\n", err);

	return err;
}

static int aoc_audio_modem_mic_input(struct aoc_chip *chip, int input_cmd, int mic_input_source)
{
	int err;
	struct CMD_HDR cmd0; /* For modem mic input STOP */
	struct CMD_AUDIO_INPUT_MODEM_INPUT_START_2 cmd1;

	if (input_cmd == START) {
		AocCmdHdrSet(&(cmd1.parent), CMD_AUDIO_INPUT_MODEM_INPUT_START_2_ID, sizeof(cmd1));

		cmd1.mic_input_source = mic_input_source;
		switch (chip->ft_aec_ref_source) {
		case DEFAULT_PLAYBACK:
			cmd1.ref_input_source = mic_input_source;
			break;

		case SPEAKER_PLAYBACK:
			cmd1.ref_input_source = MODEM_MIC_INPUT_INDEX;
			break;

		case USB_PLAYBACK:
			cmd1.ref_input_source = MODEM_USB_INPUT_INDEX;
			break;

		case BT_PLAYBACK:
			cmd1.ref_input_source = MODEM_BT_INPUT_INDEX;
			break;

		default:
			pr_err("ERR: AEC ref source wrong %d!", chip->ft_aec_ref_source);
			cmd1.ref_input_source = mic_input_source;
		}

		pr_notice("Modem mic input source: %d, aec ref source: %d\n", cmd1.mic_input_source,
			  cmd1.ref_input_source);

		err = aoc_audio_control(CMD_INPUT_CHANNEL, (uint8_t *)&cmd1, sizeof(cmd1), NULL,
					chip);
		if (err < 0)
			pr_err("ERR:%d modem input start fail!\n", err);

	} else {
		AocCmdHdrSet(&cmd0, CMD_AUDIO_INPUT_MODEM_INPUT_STOP_ID, sizeof(cmd0));

		err = aoc_audio_control(CMD_INPUT_CHANNEL, (uint8_t *)&cmd0, sizeof(cmd0), NULL,
					chip);
		if (err < 0)
			pr_err("ERR:%d modem input stop fail!\n", err);
	}

	return err;
}

static int aoc_telephony_mic_close(struct aoc_chip *chip, int mic)
{
	int err;
	int mic_input_source;

	if (chip->telephony_expect_mic != mic) {
		pr_info("%s: expect_mic %d and mic %d is not matched\n", __func__,
			chip->telephony_expect_mic, mic);
		return 0;
	}

	chip->telephony_expect_mic = NULL_PATH;

	if (chip->telephony_curr_mic != NULL_PATH) {
		pr_info("close telephony mic - %d\n", mic);
		/* Audio capture disabled for modem input */
		err = aoc_audio_modem_mic_input(chip, STOP, 0);
		if (err < 0)
			pr_err("ERR:%d modem input stop fail\n", err);

		chip->telephony_curr_mic = NULL_PATH;
	}

	/* Since the telehpony sink is still active, use the default mic source */
	if (chip->telephony_curr_sink != NULL_PATH) {
		pr_info("%s: telephony_curr_sink - %d\n", __func__, chip->telephony_curr_sink);
		mic_input_source = hw_id_to_phone_mic_source(DEFAULT_TELEPHONY_MIC);
		err = aoc_audio_modem_mic_input(chip, START, mic_input_source);
		if (err < 0)
			pr_err("ERR:%d modem input start fail\n", err);
		chip->telephony_curr_mic = DEFAULT_TELEPHONY_MIC;
	}
	return err;
}

static int aoc_telephony_mic_open(struct aoc_chip *chip, int mic)
{
	int err;
	int mic_input_source;

	chip->telephony_expect_mic = mic;

	/* The same mic source, ignore update */
	if (chip->telephony_curr_mic == mic) {
		pr_info("%s: same mic source %d\n", __func__, mic);
		return 0;
	}

	/* Different mic source is active, tear down the old one */
	if (chip->telephony_curr_mic != NULL_PATH) {
		pr_info("%s: disable mic - %d\n", __func__, chip->telephony_curr_mic);
		err = aoc_audio_modem_mic_input(chip, STOP, 0);
		if (err < 0)
			pr_err("ERR:%d modem input stop fail\n", err);

		chip->telephony_curr_mic = NULL_PATH;
	}

	mic_input_source = hw_id_to_phone_mic_source(mic);

	/* Update mask before start capture */
	if (mic_input_source == MODEM_MIC_INPUT_INDEX ||
		mic_input_source == MODEM_INCALL_INPUT_INDEX)
		aoc_audio_mic_mask_set(chip, true);

	pr_info("open telephony mic: %d - %d\n", mic_input_source, mic);
	if (mic_input_source != NULL_PATH) {
		err = aoc_audio_modem_mic_input(chip, START, mic_input_source);
		if (err < 0)
			pr_err("ERR:%d modem input start fail\n", err);
	}

	chip->telephony_curr_mic = mic;

	return err;
}

static int aoc_telephony_sink_close(struct aoc_chip *chip, int sink)
{
	int err;
	int sink_id;

	if (chip->telephony_expect_sink != sink) {
		pr_info("%s: expect_sink %d and sink %d is not matched\n", __func__,
			chip->telephony_expect_sink, sink);
		return 0;
	}

	chip->telephony_expect_sink = NULL_PATH;

	if (chip->telephony_curr_sink != NULL_PATH) {
		sink_id = hw_id_to_sink(sink);
		pr_info("close telephony sink: %d - %d\n", sink_id, sink);
		if (sink_id != NULL_PATH) {
			/* Audio playback disabled for modem output */
			err = aoc_audio_path_bind(8, sink_id, STOP, chip);
			if (err < 0)
				pr_err("ERR:%d Telephony Downlink unbind fail\n", err);
		}
		chip->telephony_curr_sink = NULL_PATH;
	}

	/* No mic active requirement, disable the default mic source */
	if (chip->telephony_curr_mic != NULL_PATH &&
		chip->telephony_expect_mic == NULL_PATH) {
		pr_info("%s: disable mic - %d\n", __func__, chip->telephony_curr_mic);
		err = aoc_audio_modem_mic_input(chip, STOP, 0);
		if (err < 0)
			pr_err("ERR:%d modem input stop fail\n", err);

		chip->telephony_curr_mic = NULL_PATH;
	}
	return err;
}

static int aoc_telephony_sink_open(struct aoc_chip *chip, int sink)
{
	int err;
	int sink_id;

	chip->telephony_expect_sink = sink;

	/* If there's sink active already on then, disable it first */
	if (chip->telephony_curr_sink != NULL_PATH) {
		sink_id = hw_id_to_sink(chip->telephony_curr_sink);
		pr_info("%s: reset previous sink id %d", __func__, sink_id);
		if (sink_id != NULL_PATH) {
			/* Audio playback disabled for modem output */
			err = aoc_audio_path_bind(8, sink_id, STOP, chip);
			if (err < 0)
				pr_err("ERR:%d Telephony Downlink unbind fail\n", err);
		}
		chip->telephony_curr_sink = NULL_PATH;
	}


	/* Since DSP requires mic source active before the sink is active,
	 * use the default mic source if no mic source is active on then
	 */
	if (chip->telephony_curr_mic == NULL_PATH) {
		int mic_input_source;

		mic_input_source = hw_id_to_phone_mic_source(DEFAULT_TELEPHONY_MIC);
		pr_info("%s: use default mic source: %d - %d\n", __func__,
			mic_input_source, DEFAULT_TELEPHONY_MIC);
		err = aoc_audio_modem_mic_input(chip, START, mic_input_source);
		if (err < 0)
			pr_err("ERR:%d modem input start fail\n", err);
		chip->telephony_curr_mic = DEFAULT_TELEPHONY_MIC;
	}

	sink_id = hw_id_to_sink(sink);
	pr_info("open telephony sink id: %d - %d\n", sink_id, sink);
	if (sink_id != NULL_PATH) {
		/* Audio playback enabled for momdem output */
		err = aoc_audio_path_bind(8, sink_id, START, chip);
		if (err < 0)
			pr_err("ERR:%d Telephony Downlink bind fail\n", err);
	}

	chip->telephony_curr_sink = sink;

	return err;
}

int aoc_phonecall_path_open(struct aoc_chip *chip, int src, int dst, bool capture)
{
	int err;

	pr_info("Open phone call path - src:%d, dst:%d\n", src, dst);

	if (!chip->voice_call_audio_enable) {
		pr_info("phone call audio NOT enabled\n");
		return 0;
	}

	if (dst < 0 || dst >= ARRAY_SIZE(chip->voice_path_vote)) {
		pr_warn("%s: invalid dst %d\n", __func__, dst);
		return -EINVAL;
	}

	if (chip->voice_path_vote[dst])
		return 0;

	chip->voice_path_vote[dst] = true;

	if (chip->voip_path_vote[dst]) {
		pr_info("%s: voip already votes %d\n", __func__, dst);
		return 0;
	}

	if (capture) {
		pr_info("voice call mic input: %d\n", dst);
		err = aoc_telephony_mic_open(chip, dst);
		if (err < 0)
			pr_err("ERR:%d modem input start fail\n", err);
	} else {
		pr_info("voice call sink: %d\n", dst);
		/* Audio playback enabled for momdem output */
		err = aoc_telephony_sink_open(chip, dst);
		if (err < 0)
			pr_err("ERR:%d Telephony Downlink bind fail\n", err);
	}

	return err;
}

int aoc_phonecall_path_close(struct aoc_chip *chip, int src, int dst, bool capture)
{
	int err;

	pr_info("close phone call path - src:%d, dst:%d\n", src, dst);

	if (!chip->voice_call_audio_enable) {
		pr_info("phone call audio NOT enabled\n");
		return 0;
	}

	if (dst < 0 || dst >= ARRAY_SIZE(chip->voice_path_vote)) {
		pr_warn("%s: invalid dst %d\n", __func__, dst);
		return -EINVAL;
	}

	if (!chip->voice_path_vote[dst])
		return 0;

	chip->voice_path_vote[dst] = false;

	if (chip->voip_path_vote[dst]) {
		pr_info("%s: voip still needs %d\n", __func__, dst);
		return 0;
	}

	if (capture) {
		/* Audio capture disabled for modem input */
		err = aoc_telephony_mic_close(chip, dst);
		if (err < 0) {
			pr_err("ERR:%d modem input stop fail\n", err);
			goto exit;
		}
	} else {
		/* Audio playback disabled for modem output */
		err = aoc_telephony_sink_close(chip, dst);
		if (err < 0)
			pr_err("ERR:%d Telephony Downlink unbind fail\n", err);
	}
exit:
	return err;
}

int aoc_voipcall_path_open(struct aoc_chip *chip, int src, int dst, bool capture)
{
	int err;

	pr_info("Open voip call path - src:%d, dst:%d\n", src, dst);

	if (!chip->voice_call_audio_enable) {
		pr_info("phone call audio NOT enabled\n");
		return 0;
	}

	if (dst < 0 || dst >= ARRAY_SIZE(chip->voip_path_vote)) {
		pr_warn("%s: invalid dst %d\n", __func__, dst);
		return -EINVAL;
	}

	if (chip->voip_path_vote[dst])
		return 0;

	chip->voip_path_vote[dst] = true;
	if (chip->voice_path_vote[dst]) {
		pr_info("%s: voice already votes %d\n", __func__, dst);
		return 0;
	}

	if (capture) {
		pr_info("voip call mic input: %d\n", dst);
		err = aoc_telephony_mic_open(chip, dst);
		if (err < 0)
			pr_err("ERR:%d modem input start fail\n", err);
	} else {
		pr_info("voice call sink: %d\n", dst);
		/* Audio playback enabled for momdem output */
		err = aoc_telephony_sink_open(chip, dst);
		if (err < 0) {
			pr_err("ERR:%d Telephony Downlink bind fail\n", err);
			goto exit;
		}
	}
exit:
	return err;
}

int aoc_voipcall_path_close(struct aoc_chip *chip, int src, int dst, bool capture)
{
	int err;

	pr_info("close voip call path - src:%d, dst:%d\n", src, dst);

	if (!chip->voice_call_audio_enable) {
		pr_info("phone call audio NOT enabled\n");
		return 0;
	}

	if (dst < 0 || dst >= ARRAY_SIZE(chip->voip_path_vote)) {
		pr_warn("%s: invalid dst %d\n", __func__, dst);
		return -EINVAL;
	}

	if (!chip->voip_path_vote[dst])
		return 0;

	chip->voip_path_vote[dst] = false;

	if (chip->voice_path_vote[dst]) {
		pr_info("%s: voice still needs %d\n", __func__, dst);
		return 0;
	}

	if (capture) {
		/* Audio capture disabled for modem input */
		err = aoc_telephony_mic_close(chip, dst);
		if (err < 0) {
			pr_err("ERR:%d modem input stop fail\n", err);
		}
	} else {
		/* Audio playback disabled for modem output */
		err = aoc_telephony_sink_close(chip, dst);
		if (err < 0)
			pr_err("ERR:%d Telephony Downlink unbind fail\n", err);
	}

	return err;
}

static int aoc_modem_voip_control(struct aoc_chip *chip, int cmd_id)
{
	int err;

	err = aoc_audio_control_simple_cmd(CMD_OUTPUT_CHANNEL, cmd_id, chip);
	if (err < 0)
		pr_err("ERR:%d modem/voip start or stop fail!\n", err);

	return err;
}

static int aoc_modem_start(struct aoc_chip *chip)
{
	int cmd_id = CMD_AUDIO_OUTPUT_TELEPHONY_MODEM_START_ID;

	return aoc_modem_voip_control(chip, cmd_id);
}

static int aoc_modem_stop(struct aoc_chip *chip)
{
	int cmd_id = CMD_AUDIO_OUTPUT_TELEPHONY_MODEM_STOP_ID;

	return aoc_modem_voip_control(chip, cmd_id);
}

static int aoc_voip_start(struct aoc_chip *chip)
{
	int cmd_id = CMD_AUDIO_OUTPUT_TELEPHONY_VOIP_START_ID;

	return aoc_modem_voip_control(chip, cmd_id);
}


static int aoc_voip_stop(struct aoc_chip *chip)
{
	int cmd_id = CMD_AUDIO_OUTPUT_TELEPHONY_VOIP_STOP_ID;

	return aoc_modem_voip_control(chip, cmd_id);
}

/* TODO: entry point idx and sink id should be specified  in the alsa_stream */
int prepare_phonecall(struct aoc_alsa_stream *alsa_stream)
{
	int err;
	int src = alsa_stream->entry_point_idx;

	/* TODO: check ptrs */
	if (!alsa_stream->chip->voice_call_audio_enable) {
		pr_info("phone call audio NOT enabled\n");
		return 0;
	}

	pr_debug("prepare phone call - dev %d\n", alsa_stream->entry_point_idx);
	if (src != 4)
		return 0;

	/* Binding modem to start audio flow */
	err = aoc_modem_start(alsa_stream->chip);
	if (err < 0)
		pr_err("ERR:%d Telephony modem start fail\n", err);

#if IS_ENABLED(CONFIG_EXYNOS_MODEM_IF)
	modem_voice_call_notify_event(MODEM_VOICE_CALL_ON, NULL);
#endif

	return err;
}

int teardown_phonecall(struct aoc_alsa_stream *alsa_stream)
{
	int err = 0;
	int src = alsa_stream->entry_point_idx;

	if (!alsa_stream->chip->voice_call_audio_enable)
		return 0;

	pr_info("stop phone call - dev %d\n", alsa_stream->entry_point_idx);
	if (src != 4)
		return 0;

	/* Unbinding modem to stop audio flow */
	err = aoc_modem_stop(alsa_stream->chip);
	if (err < 0)
		pr_err("ERR:%d Telephony modem stop fail\n", err);

#if IS_ENABLED(CONFIG_EXYNOS_MODEM_IF)
	modem_voice_call_notify_event(MODEM_VOICE_CALL_OFF, NULL);
#endif

	return err;
}

int prepare_voipcall(struct aoc_alsa_stream *alsa_stream)
{
	int err;
	struct aoc_chip *chip = alsa_stream->chip;

	/* TODO: check ptrs */
	if (!chip->voice_call_audio_enable) {
		pr_info("voip call audio NOT enabled\n");
		return 0;
	}

	if (alsa_stream->substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		chip->voip_tx_prepared = true;
		if (chip->voip_rx_prepared)
			return 0;
	} else {
		chip->voip_rx_prepared = true;
		if (chip->voip_tx_prepared)
			return 0;
	}

	pr_debug("prepare voip call - dev %d\n", alsa_stream->entry_point_idx);

	err = aoc_voip_start(chip);
	if (err < 0)
		pr_err("ERR:%d Telephony voip start fail\n", err);

	return err;
}

int teardown_voipcall(struct aoc_alsa_stream *alsa_stream)
{
	int err = 0;
	struct aoc_chip *chip = alsa_stream->chip;

	if (!chip->voice_call_audio_enable)
		return 0;

	if (alsa_stream->substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		chip->voip_tx_prepared = false;
		if (chip->voip_rx_prepared)
			return 0;
	} else {
		chip->voip_rx_prepared = false;
		if (chip->voip_tx_prepared)
			return 0;
	}

	pr_info("stop voip call - dev %d\n", alsa_stream->entry_point_idx);

	err = aoc_voip_stop(chip);
	if (err < 0)
		pr_err("ERR:%d Telephony voip stop fail\n", err);

	return err;
}

int aoc_compr_offload_setup(struct aoc_alsa_stream *alsa_stream, int type)
{
	int err;
	struct CMD_AUDIO_OUTPUT_DECODER_CFG cmd;

	/* TODO: refactor may be needed for passing codec info from HAL to AoC */
	AocCmdHdrSet(&(cmd.parent),
		alsa_stream->gapless_offload_enable?
			CMD_AUDIO_OUTPUT_DECODER_CFG_GAPLESS_ID:
			CMD_AUDIO_OUTPUT_DECODER_CFG_ID,
		sizeof(cmd));

	/* TODO: HAL only passes MP3 or AAC, need to consider/test other AAC options */
	cmd.cfg.format = type;
	cmd.cfg.samplerate = alsa_stream->params_rate;
	cmd.cfg.channels = alsa_stream->channels;
	cmd.address = 0;
	cmd.size = 0;

	memcpy(cmd.cfg.options, alsa_stream->compr_offload_codec_options,
		sizeof(cmd.cfg.options));

	pr_info("%s type=%d format=%d sr=%d chan=%d\n", __func__, type, cmd.cfg.format,
		cmd.cfg.samplerate, cmd.cfg.channels);

	err = aoc_audio_control(CMD_OUTPUT_CHANNEL, (uint8_t *)&cmd, sizeof(cmd), (uint8_t *)&cmd,
				alsa_stream->chip);
	if (err < 0) {
		pr_err("ERR:%d in set compress offload codec\n", err);
		return err;
	}

	err = cmd.parent.reply;
	if (err < 0) {
		pr_err("ERR:%d in set compress offload codec cmd reply\n", err);
		return err;
	}
	return 0;
}

int aoc_compr_offload_send_metadata(struct aoc_alsa_stream *alsa_stream)
{
	int err;
	struct CMD_AUDIO_OUTPUT_DECODER_GAPLESS_METADATA cmd;

	if (!alsa_stream->gapless_offload_enable)
		return 0;

	if (!alsa_stream->send_metadata)
		return 0;

	if (alsa_stream->compr_padding == COMPR_INVALID_METADATA ||
	    alsa_stream->compr_delay == COMPR_INVALID_METADATA) {
		pr_warn("invalid metadata\n");
		return 0;
	}

	AocCmdHdrSet(&(cmd.parent), CMD_AUDIO_OUTPUT_DECODER_GAPLESS_METADATA_ID, sizeof(cmd));
	cmd.curr_track_padding_frames = alsa_stream->compr_padding;
	cmd.curr_track_delay_frames = alsa_stream->compr_delay;

	pr_info("send metadata, padding %d, delay %d\n", cmd.curr_track_padding_frames,
		cmd.curr_track_delay_frames);

	err = aoc_audio_control(CMD_OUTPUT_CHANNEL, (uint8_t *)&cmd, sizeof(cmd), NULL,
				alsa_stream->chip);

	if (err < 0) {
		pr_err("ERR:%d in set aoc compress offload in partial drain state\n", err);
		return err;
	}

	alsa_stream->send_metadata = 0;
	return 0;
}

int aoc_compr_offload_partial_drain(struct aoc_alsa_stream *alsa_stream)
{
	int err;

	if (!alsa_stream->gapless_offload_enable)
		return 0;

	pr_info("compress offload partial drain\n");

	err = aoc_audio_control_simple_cmd(CMD_OUTPUT_CHANNEL,
		CMD_AUDIO_OUTPUT_DECODER_GAPLESS_PARTIAL_DRAIN_ID, alsa_stream->chip);
	if (err < 0)
		pr_err("ERR:%d compress offload partial drain!\n", err);

	return err;
}

int aoc_compr_offload_close(struct aoc_alsa_stream *alsa_stream)
{
	int err;

	if (!alsa_stream->gapless_offload_enable)
		return 0;

	pr_info("compress offload decoder de-inited\n");

	err = aoc_audio_control_simple_cmd(CMD_OUTPUT_CHANNEL,
		CMD_AUDIO_OUTPUT_DECODER_GAPLESS_DEINIT_ID, alsa_stream->chip);
	if (err < 0) {
		pr_err("ERR:%d in compress offload close\n", err);
		return err;
	}

	return 0;
}

int aoc_compr_offload_get_io_samples(struct aoc_alsa_stream *alsa_stream, uint64_t *sample)
{
	int err;
	struct CMD_AUDIO_OUTPUT_GET_EP_SAMPLES cmd;

	AocCmdHdrSet(&(cmd.parent), CMD_AUDIO_OUTPUT_GET_EP_TOT_SAMPLES_ID,
		     sizeof(cmd));
	cmd.source = alsa_stream->entry_point_idx;

	err = aoc_audio_control(CMD_OUTPUT_CHANNEL, (uint8_t *)&cmd,
				sizeof(cmd), (uint8_t *)&cmd,
				alsa_stream->chip);
	if (err < 0)
		pr_err("ERR:%d in getting compress offload io-sample number\n",
		       err);
	else
		*sample = cmd.samples;

	return err < 0 ? err : 0;
}

int aoc_compr_offload_flush_buffer(struct aoc_alsa_stream *alsa_stream)
{
	int err;

	err = aoc_audio_control_simple_cmd(CMD_OUTPUT_CHANNEL,
		CMD_AUDIO_OUTPUT_DECODE_FLUSH_RB_ID, alsa_stream->chip);

	if (err < 0)
		pr_err("ERR:%d flush compress offload buffer fail!\n", err);

	return err;
}

static int aoc_compr_reset_gain_and_delay(struct aoc_alsa_stream *alsa_stream)
{
	int err = 0;
	int cmd_id = CMD_AUDIO_OUTPUT_DEC_RESET_CURRENT_GAIN_ID;
	struct aoc_chip *chip = alsa_stream->chip;

	err = aoc_audio_control_simple_cmd(CMD_OUTPUT_CHANNEL, cmd_id, chip);
	if (err < 0) {
		pr_err("ERR:%d in aoc compr reset gain\n", err);
		return err;
	}

	/* To allow the reset finished in aoc */
	msleep(COMPR_OFFLOAD_GAIN_RESET_TIME_DELAY_IN_MSECS);

	return 0;
}

int aoc_compr_pause(struct aoc_alsa_stream *alsa_stream)
{
	int err;

	/* reset the gain in aoc for compr offload playback and then wait for 10 ms */
	err = aoc_compr_reset_gain_and_delay(alsa_stream);
	if (err < 0)
		pr_err("ERR:%d aoc compr reset gain ramp fail\n", err);

	err = aoc_audio_stop(alsa_stream);
	if (err < 0)
		pr_err("ERR:%d aoc_compr_pause fail\n", err);

	return 0;
}

int aoc_compr_resume(struct aoc_alsa_stream *alsa_stream)
{
	int err;

	err = aoc_audio_start(alsa_stream);
	if (err < 0)
		pr_err("ERR:%d aoc_compr_resume fail\n", err);

	return 0;
}

int aoc_audio_setup(struct aoc_alsa_stream *alsa_stream)
{
	return 0;
}

int aoc_audio_open(struct aoc_alsa_stream *alsa_stream)
{
	return 0;
}

int aoc_audio_close(struct aoc_alsa_stream *alsa_stream)
{
	struct aoc_chip *chip = alsa_stream->chip;
	struct snd_pcm_substream *substream = alsa_stream->substream;

	/* To deal with recording with spatial module enabled */
	if (substream && substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		if (alsa_stream->idx == UC_ULTRASONIC_RECORD)
			ap_record_stop(chip, alsa_stream);
		else if (ap_filter_capture_stream(alsa_stream)) {
			/* Stop the capturing mic*/
			if (aoc_audio_capture_active_stream_num(chip) == 0) {
				pr_info("%s: record stop\n", __func__);
				ap_record_stop(chip, alsa_stream);
			}
		}
	}
	return 0;
}

static void print_enc_param(struct AUDIO_OUTPUT_BT_A2DP_ENC_CFG *enc_cfg)
{
	int i;

	pr_info("codecType = %x\n", enc_cfg->codecType);
	pr_info("bitrate = %x\n", enc_cfg->bitrate);
	pr_info("peerMTU = %x\n", enc_cfg->peerMTU);
	for (i = 0; i < 6; i++)
		pr_info("  params[%d] = %x\n", i, enc_cfg->params[i]);
}

int aoc_a2dp_set_enc_param(struct aoc_chip *chip, struct AUDIO_OUTPUT_BT_A2DP_ENC_CFG *cfg)
{
	int err = 0;
	struct CMD_AUDIO_OUTPUT_BT_A2DP_ENC_CFG cmd;

	AocCmdHdrSet(&(cmd.parent), CMD_AUDIO_OUTPUT_BT_A2DP_ENC_CFG_ID, sizeof(cmd));
	memcpy(&cmd.bt_a2dp_enc_cfg, cfg, sizeof(*cfg));

	print_enc_param(&cmd.bt_a2dp_enc_cfg);

	err = aoc_audio_control(CMD_OUTPUT_CHANNEL, (uint8_t *)&cmd, sizeof(cmd), (uint8_t *)&cmd,
				chip);

	if (err < 0)
		pr_err("ERR:%d set enc parameter failed\n", err);

	return err;
}

int aoc_pdm_mic_power_cfg_init(struct aoc_chip *chip, uint32_t *cfg, int count)
{
	int i, err = 0;
	const int cmd_id = CMD_AUDIO_INPUT_SET_PARAMETER_ID;
	const int block = 139; /* ABLOCK_INPUT_PDM_MIC */
	const int component = ASP_ID_NONE;
	int key_base = 2; /* PDM_POWER */

	/* Send cmd to AOC */
	for (i = 0; i < count; i++) {
		err = aoc_audio_set_parameters(cmd_id, block, component, key_base + i,
			(int) cfg[i], chip);
		if (err < 0) {
			pr_err("ERR:%s %d\n", __func__, err);
			return err;
		}
	}
	return err;
}

int aoc_pdm_mic_power_cfg_get(struct aoc_chip *chip, uint32_t *cfg, int count)
{
	int i, err = 0;
	const int cmd_id = CMD_AUDIO_INPUT_GET_PARAMETER_ID;
	const int block = 139; /* ABLOCK_INPUT_PDM_MIC */
	const int component = ASP_ID_NONE;
	int key_base = 2; /* PDM_POWER */

	/* Send cmd to AOC */
	for (i = 0; i < count; i++) {
		err = aoc_audio_get_parameters(cmd_id, block, component, key_base + i,
						&cfg[i], chip);
		if (err < 0) {
			pr_err("ERR:%s %d\n", __func__, err);
			return err;
		}
	}
	return err;
}

int aoc_audio_us_record(struct aoc_chip *chip, bool enable)
{
	int cmd_id, err = 0;

	cmd_id = enable ? CMD_AUDIO_INPUT_DIRECT_ULTRASONIC_CAPTURE_ENABLE_ID :
			  CMD_AUDIO_INPUT_DIRECT_ULTRASONIC_CAPTURE_DISABLE_ID;

	err = aoc_audio_control_simple_cmd(CMD_INPUT_CHANNEL, cmd_id, chip);
	if (err < 0)
		pr_err("ERR:%d in ultra sonic record %s control\n", err, enable ? "start" : "stop");

	return err;
}

int aoc_audio_set_chirp_parameter(struct aoc_chip *chip, int key, int value)
{
	int err;
	struct CMD_AUDIO_OUTPUT_SET_PARAMETER cmd;

	AocCmdHdrSet(&cmd.parent, CMD_AUDIO_OUTPUT_SET_PARAMETER_ID,
		     sizeof(cmd));
	cmd.block = AOC_CHIRP_BLOCK;
	cmd.key = key;
	cmd.val = value;

	err = aoc_audio_control(CMD_OUTPUT_CHANNEL, (uint8_t *)&cmd,
				sizeof(cmd), (uint8_t *)&cmd, chip);
	if (err < 0)
		pr_err("ERR:%d in AoC Set Chirp Parameter, key: %d\n", err, key);

	return err < 0 ? err : 0;
}

int aoc_audio_set_two_one(struct aoc_chip *chip, int enable)
{
	int err;
	struct CMD_AUDIO_OUTPUT_SET_PARAMETER cmd;

	AocCmdHdrSet(&cmd.parent, CMD_AUDIO_OUTPUT_SET_PARAMETER_ID,
		     sizeof(cmd));
	cmd.block = AOC_ATRIGGER_BLOCK;
	cmd.key = 0;
	cmd.val = enable;

	err = aoc_audio_control(CMD_OUTPUT_CHANNEL, (uint8_t *)&cmd,
				sizeof(cmd), (uint8_t *)&cmd, chip);
	if (err < 0)
		pr_err("ERR:%d setting 2.1 %d\n", err, enable);

	return err < 0 ? err : 0;
}

int aoc_audio_set_chre_src_pdm_gain(struct aoc_chip *chip, int gain)
{
	if (chip->chre_supported) {
		int err;
		struct CMD_AUDIO_INPUT_SET_CHRE_SRC_PDM_GAIN cmd;

		AocCmdHdrSet(&cmd.parent, CMD_AUDIO_INPUT_SET_CHRE_SRC_PDM_GAIN_ID,
				sizeof(cmd));
		cmd.gain_centibel = gain;

		err = aoc_audio_control(CMD_INPUT_CHANNEL, (uint8_t *)&cmd,
					sizeof(cmd), (uint8_t *)&cmd, chip);
		if (err < 0)
			pr_err("ERR:%d in AoC Set CHRE PDM gain\n", err);

		return err < 0 ? err : 0;
	} else {
		pr_err("WARN: setting CHRE PDM gain is not supported\n");
		return 0;
	}
}

int aoc_audio_set_chre_src_aec_gain(struct aoc_chip *chip, int gain)
{
	if (chip->chre_supported) {
		int err;
		struct CMD_AUDIO_INPUT_SET_CHRE_SRC_AEC_GAIN cmd;

		AocCmdHdrSet(&cmd.parent, CMD_AUDIO_INPUT_SET_CHRE_SRC_AEC_GAIN_ID,
				sizeof(cmd));
		cmd.gain_centibel = gain;

		err = aoc_audio_control(CMD_INPUT_CHANNEL, (uint8_t *)&cmd,
					sizeof(cmd), (uint8_t *)&cmd, chip);
		if (err < 0)
			pr_err("ERR:%d in AoC Set CHRE AEC gain\n", err);

		return err < 0 ? err : 0;
	} else {
		pr_err("WARN: setting CHRE AEC gain is not supported\n");
		return 0;
	}
}

int aoc_audio_set_chre_src_aec_timeout(struct aoc_chip *chip, int timeout)
{
	if (chip->chre_supported) {
		int err;
		struct CMD_AUDIO_INPUT_SET_CHRE_SRC_AEC_TIMEOUT cmd;

		AocCmdHdrSet(&cmd.parent, CMD_AUDIO_INPUT_SET_CHRE_SRC_AEC_TIMEOUT_ID,
				sizeof(cmd));
		cmd.timeout_ms = timeout;

		err = aoc_audio_control(CMD_INPUT_CHANNEL, (uint8_t *)&cmd,
					sizeof(cmd), (uint8_t *)&cmd, chip);
		if (err < 0)
			pr_err("ERR:%d in AoC Set CHRE timeout\n", err);

		return err < 0 ? err : 0;
	} else {
		pr_err("WARN: setting CHRE AEC gain is not supported\n");
		return 0;
	}
}

#if !(IS_ENABLED(CONFIG_SOC_GS101) || IS_ENABLED(CONFIG_SOC_GS201))
int aoc_audio_set_hdmic_gain(struct aoc_chip *chip, int gain)
{
	int err;
	struct CMD_AUDIO_INPUT_SET_HDMIC_GAIN cmd;

	AocCmdHdrSet(&cmd.parent, CMD_AUDIO_INPUT_SET_HDMIC_GAIN_ID,
			sizeof(cmd));
	cmd.gain_centibel = gain;

	err = aoc_audio_control(CMD_INPUT_CHANNEL, (uint8_t *)&cmd,
				sizeof(cmd), (uint8_t *)&cmd, chip);
	if (err < 0)
		pr_err("ERR:%d in AoC set HDMIC gain\n", err);

	return err < 0 ? err : 0;
}
#endif

/* Update PDM mic mask */
int aoc_audio_mic_mask_set(struct aoc_chip *chip, bool is_voice)
{
	uint32_t value = UINT_MAX;
	uint8_t *mask = (uint8_t *)(&value);
	int key = is_voice, i;
	const int cmd_id = CMD_AUDIO_INPUT_SET_PARAMETER_ID;
	const int block = 139; /* ABLOCK_INPUT_PDM_MIC */
	const int component = ASP_ID_NONE;
	const int total_lists = min(NUM_OF_BUILTIN_MIC, (int)sizeof(uint32_t));

	for (i = 0; i < total_lists; i++)
		mask[i] = chip->buildin_mic_id_list[i];

	return aoc_audio_set_parameters(cmd_id, block, component, key, value, chip);
}

int aoc_multichannel_processor_switch_set(struct aoc_chip *chip, int value)
{
	const int cmd_id = CMD_AUDIO_OUTPUT_SET_PARAMETER_ID;
	const int block = 22; /* ABLOCK_MCPROC */
	const int component = 43; /* ASP_ID_MC_PP */
	const int paramid = 0; /* parameter ID for the active module */

	return aoc_audio_set_parameters(cmd_id, block, component, paramid, value, chip);
}
