// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google Whitechapel AoC ALSA Driver on Audio Path
 *
 * Copyright (c) 2019 Google LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/string.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/soc-dai.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>

#include "aoc_alsa.h"
#include "aoc_alsa_drv.h"
#include "aoc_alsa_path.h"
#include "google-aoc-enum.h"

struct be_path_cache port_array[PORT_MAX] = {
	[0 ... PORT_MAX - 1] = {
		.fe_put_mask = {
			[0 ... BE_MAP_SZ(fe_put_mask) - 1] = 0,
		},
		.on = false,
	},
};

static const struct snd_soc_dai_ops be_dai_ops;
static struct mutex path_mutex;

static int aoc_compress_new(struct snd_soc_pcm_runtime *rtd, int num);

static const uint32_t rx_ep_list[] = {
	IDX_EP2_RX,           /* low-latency-playback */
	IDX_EP3_RX,           /* haptic-audio */
	IDX_EP5_RX,           /* voice-call */
	IDX_EP6_RX,           /* deep-buffer-playback */
	IDX_EP7_RX,           /* compress-offload-playback */
	IDX_VOIP_RX,          /* voip-playback */
	IDX_RAW_RX,           /* raw-playback */
	IDX_EP1_RX,           /* mmap-playback */
	IDX_HIFI_RX,          /* hifi-playback */
	IDX_NOHOST1_RX,       /* hostless rx */
	IDX_HAPTIC_NoHOST_RX, /* haptic hostless rx */
	IDX_IMSV_RX,           /* immersive-playback */
	IDX_EP4_RX,           /* reserved */
	IDX_EP8_RX,           /* reserved */
	IDX_DP_DMA_NoHOST_RX, /* dp dma hostless rx */
};

static const uint32_t tx_ep_list[] = {
	IDX_EP1_TX,     /* audio-record */
	IDX_EP4_TX,     /* voice-call */
	IDX_VOIP_TX,    /* voip-record */
	IDX_EP3_TX,     /* low-latency-record */
	IDX_RAW_TX,     /* raw-record */
	IDX_EP2_TX,     /* mmap-record */
	IDX_HIFI_TX,    /* hifi tx */
	IDX_NOHOST1_TX, /* hostless tx */
	IDX_EP5_TX,     /* reserved */
	IDX_EP6_TX,     /* reserved */
	IDX_EP7_TX,     /* reserved */
};

static struct snd_soc_dai_driver aoc_dai_drv[] = {
	/* FE dai */
	{
		.playback = {
			.stream_name = "EP1 Playback",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
					SNDRV_PCM_FMTBIT_S24_LE |
					SNDRV_PCM_FMTBIT_S24_3LE |
					SNDRV_PCM_FMTBIT_FLOAT_LE |
					SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 2,
		},
		.name = "EP1 PB",
		.id = IDX_EP1_RX,
	},

	{
		.playback = {
			.stream_name = "EP2 Playback",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
					SNDRV_PCM_FMTBIT_S24_LE |
					SNDRV_PCM_FMTBIT_S24_3LE |
					SNDRV_PCM_FMTBIT_FLOAT_LE |
					SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 2,
		},
		.name = "EP2 PB",
		.id = IDX_EP2_RX,
	},

	{
		.playback = {
			.stream_name = "EP3 Playback",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
					SNDRV_PCM_FMTBIT_S24_LE |
					SNDRV_PCM_FMTBIT_S24_3LE |
					SNDRV_PCM_FMTBIT_FLOAT_LE |
					SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 2,
		},
		.name = "EP3 PB",
		.id = IDX_EP3_RX,
	},

	{
		.playback = {
			.stream_name = "EP4 Playback",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
					SNDRV_PCM_FMTBIT_S24_LE |
					SNDRV_PCM_FMTBIT_S24_3LE |
					SNDRV_PCM_FMTBIT_FLOAT_LE |
					SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 6,
		},
		.name = "EP4 PB",
		.id = IDX_EP4_RX,
	},

	{
		.playback = {
			.stream_name = "EP5 Playback",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
					SNDRV_PCM_FMTBIT_S24_LE |
					SNDRV_PCM_FMTBIT_S24_3LE |
					SNDRV_PCM_FMTBIT_FLOAT_LE |
					SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 2,
		},
		.name = "EP5 PB",
		.id = IDX_EP5_RX,
	},

	{
		.playback = {
			.stream_name = "EP6 Playback",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
					SNDRV_PCM_FMTBIT_S24_LE |
					SNDRV_PCM_FMTBIT_S24_3LE |
					SNDRV_PCM_FMTBIT_FLOAT_LE |
					SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 2,
		},
		.name = "EP6 PB",
		.id = IDX_EP6_RX,
	},

	{
		.playback = {
			.stream_name = "EP7 Playback",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
					SNDRV_PCM_FMTBIT_S24_LE |
					SNDRV_PCM_FMTBIT_S24_3LE |
					SNDRV_PCM_FMTBIT_FLOAT_LE |
					SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 2,
		},
		.compress_new = aoc_compress_new,
		.name = "EP7 PB",
		.id = IDX_EP7_RX,
	},

	{
		.playback = {
			.stream_name = "EP8 Playback",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
					SNDRV_PCM_FMTBIT_S24_LE |
					SNDRV_PCM_FMTBIT_S24_3LE |
					SNDRV_PCM_FMTBIT_FLOAT_LE |
					SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 2,
		},
		.name = "EP8 PB",
		.id = IDX_EP8_RX,
	},

	{
		.playback = {
			.stream_name = "NoHost1 Playback",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
					SNDRV_PCM_FMTBIT_S24_LE |
					SNDRV_PCM_FMTBIT_S24_3LE |
					SNDRV_PCM_FMTBIT_FLOAT_LE |
					SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 2,
		},
		.name = "NoHost1 PB",
		.id = IDX_NOHOST1_RX,
	},

	{
		.playback = {
			.stream_name = "audio_voip_rx",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
					SNDRV_PCM_FMTBIT_S24_LE |
					SNDRV_PCM_FMTBIT_S24_3LE |
					SNDRV_PCM_FMTBIT_FLOAT_LE |
					SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 2,
		},
		.name = "audio_voip_rx",
		.id = IDX_VOIP_RX,
	},

	{
		.playback = {
			.stream_name = "audio_incall_pb_0",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
					SNDRV_PCM_FMTBIT_S24_LE |
					SNDRV_PCM_FMTBIT_S24_3LE |
					SNDRV_PCM_FMTBIT_FLOAT_LE |
					SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 2,
		},
		.name = "audio_incall_pb_0",
		.id = IDX_INCALL_PB0_RX,
	},

	{
		.playback = {
			.stream_name = "audio_incall_pb_1",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
					SNDRV_PCM_FMTBIT_S24_LE |
					SNDRV_PCM_FMTBIT_S24_3LE |
					SNDRV_PCM_FMTBIT_FLOAT_LE |
					SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 2,
		},
		.name = "audio_incall_pb_1",
		.id = IDX_INCALL_PB1_RX,
	},

	{
		.playback = {
			.stream_name = "audio_incall_pb_2",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
					SNDRV_PCM_FMTBIT_S24_LE |
					SNDRV_PCM_FMTBIT_S24_3LE |
					SNDRV_PCM_FMTBIT_FLOAT_LE |
					SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 4,
		},
		.name = "audio_incall_pb_2",
		.id = IDX_INCALL_PB2_RX,
	},

	{
		.playback = {
			.stream_name = "audio_raw",
			.rates = SNDRV_PCM_RATE_48000,
			.formats = SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 2,
#if IS_ENABLED(CONFIG_SOC_ZUMA)
			.channels_max = 2,
#else
			.channels_max = 4,
#endif
		},
		.name = "audio_raw",
		.id = IDX_RAW_RX,
	},

	{
		.playback = {
			.stream_name = "HAPTIC NoHost Playback",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
					SNDRV_PCM_FMTBIT_S24_LE |
					SNDRV_PCM_FMTBIT_S24_3LE |
					SNDRV_PCM_FMTBIT_FLOAT_LE |
					SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 4,
		},
		.name = "HAPTIC NoHost PB",
		.id = IDX_HAPTIC_NoHOST_RX,
	},

	{
		.playback = {
			.stream_name = "audio_hifiout",
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
					SNDRV_PCM_FMTBIT_S24_LE |
					SNDRV_PCM_FMTBIT_S24_3LE |
					SNDRV_PCM_FMTBIT_FLOAT_LE |
					SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 8,
		},
		.name = "audio_hifiout",
		.id = IDX_HIFI_RX,
	},

	{
		.playback = {
			.stream_name = "audio_ultrasonic",
			.rates = SNDRV_PCM_RATE_96000,
			.formats = SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 2,
#if IS_ENABLED(CONFIG_SOC_ZUMA)
			.channels_max = 2,
#else
			.channels_max = 4,
#endif
		},
		.name = "audio_ultrasonic",
		.id = IDX_US_RX,
	},

	{
		.playback = {
			.stream_name = "audio_immersive",
			.rates = SNDRV_PCM_RATE_48000,
			.formats = SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 2,
			.channels_max = 2,
		},
		.name = "audio_immersive",
		.id = IDX_IMSV_RX,
	},

	{
		.playback = {
			.stream_name = "audio_capture_inject",
			.rates = SNDRV_PCM_RATE_8000_96000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
					SNDRV_PCM_FMTBIT_S24_LE |
					SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 4,
		},
		.name = "audio_capture_inject",
		.id = IDX_CAP_INJ_RX,
	},

	{
		.capture = {
			.stream_name = "EP1 Capture",
			.rates = SNDRV_PCM_RATE_8000_96000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
					SNDRV_PCM_FMTBIT_S24_LE |
					SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 4,
		},
		.name = "EP1 CAP",
		.id = IDX_EP1_TX,
	},

	{
		.capture = {
			.stream_name = "EP2 Capture",
			.rates = SNDRV_PCM_RATE_8000_96000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
					SNDRV_PCM_FMTBIT_S24_LE |
					SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 4,
		},
		.name = "EP2 CAP",
		.id = IDX_EP2_TX,
	},

	{
		.capture = {
			.stream_name = "EP3 Capture",
			.rates = SNDRV_PCM_RATE_8000_96000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
					SNDRV_PCM_FMTBIT_S24_LE |
					SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 4,
		},
		.name = "EP3 CAP",
		.id = IDX_EP3_TX,
	},

	{
		.capture = {
			.stream_name = "EP4 Capture",
			.rates = SNDRV_PCM_RATE_8000_96000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
					SNDRV_PCM_FMTBIT_S24_LE |
					SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 4,
		},
		.name = "EP4 CAP",
		.id = IDX_EP4_TX,
	},

	{
		.capture = {
			.stream_name = "EP5 Capture",
			.rates = SNDRV_PCM_RATE_8000_96000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
					SNDRV_PCM_FMTBIT_S24_LE |
					SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 4,
		},
		.name = "EP5 CAP",
		.id = IDX_EP5_TX,
	},

	{
		.capture = {
			.stream_name = "EP6 Capture",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
					SNDRV_PCM_FMTBIT_S24_LE |
					SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 4,
		},
		.name = "EP6 CAP",
		.id = IDX_EP6_TX,
	},

	{
		.capture = {
			.stream_name = "NoHost1 Capture",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
					SNDRV_PCM_FMTBIT_S24_LE |
					SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 4,
		},
		.name = "NoHost1 CAP",
		.id = IDX_NOHOST1_TX,
	},

	{
		.capture = {
			.stream_name = "audio_voip_tx",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
					SNDRV_PCM_FMTBIT_S24_LE |
					SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 4,
		},
		.name = "audio_voip_tx",
		.id = IDX_VOIP_TX,
	},

	{
		.capture = {
			.stream_name = "audio_incall_cap_0",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
					SNDRV_PCM_FMTBIT_S24_LE |
					SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 8,
		},
		.name = "audio_incall_cap_0",
		.id = IDX_INCALL_CAP0_TX,
	},

	{
		.capture = {
			.stream_name = "audio_incall_cap_1",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
					SNDRV_PCM_FMTBIT_S24_LE |
					SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 8,
		},
		.name = "audio_incall_cap_1",
		.id = IDX_INCALL_CAP1_TX,
	},

	{
		.capture = {
			.stream_name = "audio_incall_cap_2",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
					SNDRV_PCM_FMTBIT_S24_LE |
					SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 8,
		},
		.name = "audio_incall_cap_2",
		.id = IDX_INCALL_CAP2_TX,
	},

	{
		.capture = {
			.stream_name = "audio_incall_cap_3",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
					SNDRV_PCM_FMTBIT_S24_LE |
					SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 8,
		},
		.name = "audio_incall_cap_3",
		.id = IDX_INCALL_CAP3_TX,
	},

	{
		.capture = {
			.stream_name = "audio_hifiin",
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
					SNDRV_PCM_FMTBIT_S24_LE |
					SNDRV_PCM_FMTBIT_S24_3LE |
					SNDRV_PCM_FMTBIT_FLOAT_LE |
					SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 8,
		},
		.name = "audio_hifiin",
		.id = IDX_HIFI_TX,
	},

	{
		.capture = {
			.stream_name = "audio_android_aec",
			.rates = SNDRV_PCM_RATE_8000_96000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
					SNDRV_PCM_FMTBIT_S24_LE |
					SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 2,
		},
		.name = "audio_android_aec",
		.id = IDX_ANDROID_AEC_TX,
	},

	{
		.capture = {
			.stream_name = "audio_hotword_tap",
			.rates = SNDRV_PCM_RATE_16000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
			.channels_min = 1,
			.channels_max = 2,
		},
		.name = "audio_hotword_tap",
		.id = IDX_HOTWORD_TAP_TX,
	},

	/* BE dai */
	{
		.playback = {
			.stream_name = "I2S_0_RX Playback",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
					SNDRV_PCM_FMTBIT_S24_LE |
					SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 4,
		},
		.ops = &be_dai_ops,
		.name = "I2S_0_RX",
		.id = I2S_0_RX,
	},

	{
		.capture = {
			.stream_name = "I2S_0_TX Capture",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
					SNDRV_PCM_FMTBIT_S24_LE |
					SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 4,
		},
		.ops = &be_dai_ops,
		.name = "I2S_0_TX",
		.id = I2S_0_TX,
	},

	{
		.playback = {
			.stream_name = "I2S_1_RX Playback",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
					SNDRV_PCM_FMTBIT_S24_LE |
					SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 4,
		},
		.ops = &be_dai_ops,
		.name = "I2S_1_RX",
		.id = I2S_1_RX,
	},

	{
		.capture = {
			.stream_name = "I2S_1_TX Capture",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
					SNDRV_PCM_FMTBIT_S24_LE |
					SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 4,
		},
		.ops = &be_dai_ops,
		.name = "I2S_1_TX",
		.id = I2S_1_TX,
	},

	{
		.playback = {
			.stream_name = "I2S_2_RX Playback",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
					SNDRV_PCM_FMTBIT_S24_LE |
					SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 4,
		},
		.ops = &be_dai_ops,
		.name = "I2S_2_RX",
		.id = I2S_2_RX,
	},

	{
		.capture = {
			.stream_name = "I2S_2_TX Capture",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
					SNDRV_PCM_FMTBIT_S24_LE |
					SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 4,
		},
		.ops = &be_dai_ops,
		.name = "I2S_2_TX",
		.id = I2S_2_TX,
	},

	{
		.playback = {
			.stream_name = "TDM_0_RX Playback",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
					SNDRV_PCM_FMTBIT_S24_LE |
					SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 4,
		},
		.ops = &be_dai_ops,
		.name = "TDM_0_RX",
		.id = TDM_0_RX,
	},

	{
		.capture = {
			.stream_name = "TDM_0_TX Capture",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
					SNDRV_PCM_FMTBIT_S24_LE |
					SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 4,
		},
		.ops = &be_dai_ops,
		.name = "TDM_0_TX",
		.id = TDM_0_TX,
	},

	{
		.playback = {
			.stream_name = "TDM_1_RX Playback",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
					SNDRV_PCM_FMTBIT_S24_LE |
					SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 4,
		},
		.ops = &be_dai_ops,
		.name = "TDM_1_RX",
		.id = TDM_1_RX,
	},

	{
		.capture = {
			.stream_name = "TDM_1_TX Capture",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
					SNDRV_PCM_FMTBIT_S24_LE |
					SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 4,
		},
		.ops = &be_dai_ops,
		.name = "TDM_1_TX",
		.id = TDM_1_TX,
	},

	{
		.capture = {
			.stream_name = "INTERNAL_MIC_TX Capture",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
					SNDRV_PCM_FMTBIT_S24_LE |
					SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 4,
		},
		.ops = &be_dai_ops,
		.name = "INTERNAL_MIC_TX",
		.id = INTERNAL_MIC_TX,
	},

	{
		.capture = {
			.stream_name = "INTERNAL_MIC_US_TX Capture",
			.rates = SNDRV_PCM_RATE_8000_96000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
					SNDRV_PCM_FMTBIT_S24_LE |
					SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 4,
		},
		.ops = &be_dai_ops,
		.name = "INTERNAL_MIC_US_TX",
		.id = INTERNAL_MIC_US_TX,
	},

	{
		.capture = {
			.stream_name = "ERASER_TX Capture",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
					SNDRV_PCM_FMTBIT_S24_LE |
					SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 4,
		},
		.ops = &be_dai_ops,
		.name = "ERASER_TX",
		.id = ERASER_TX,
	},

	{
		.playback = {
			.stream_name = "BT_RX Playback",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
					SNDRV_PCM_FMTBIT_S24_LE |
					SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 4,
		},
		.ops = &be_dai_ops,
		.name = "BT_RX",
		.id = BT_RX,
	},

	{
		.capture = {
			.stream_name = "BT_TX Capture",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
					SNDRV_PCM_FMTBIT_S24_LE |
					SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 4,
		},
		.ops = &be_dai_ops,
		.name = "BT_TX",
		.id = BT_TX,
	},

	{
		.playback = {
			.stream_name = "USB_RX Playback",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
					SNDRV_PCM_FMTBIT_S24_LE |
					SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 4,
		},
		.ops = &be_dai_ops,
		.name = "USB_RX",
		.id = USB_RX,
	},

	{
		.capture = {
			.stream_name = "USB_TX Capture",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
					SNDRV_PCM_FMTBIT_S24_LE |
					SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 4,
		},
		.ops = &be_dai_ops,
		.name = "USB_TX",
		.id = USB_TX,
	},

	{
		.playback = {
			.stream_name = "INCALL_RX Playback",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
					SNDRV_PCM_FMTBIT_S24_LE |
					SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 4,
		},
		.name = "INCALL_RX",
		.id = INCALL_RX,
	},

	{
		.capture = {
			.stream_name = "INCALL_TX Capture",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
					SNDRV_PCM_FMTBIT_S24_LE |
					SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 4,
		},
		.name = "INCALL_TX",
		.id = INCALL_TX,
	},

	{
		.playback = {
			.stream_name = "HAPTIC_RX Playback",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
					SNDRV_PCM_FMTBIT_S24_LE |
					SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 4,
		},
		.name = "HAPTIC_RX",
		.id = HAPTIC_RX,
	},

	{
		.playback = {
			.stream_name = "DP_DMA NoHost Playback",
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
					SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 2,
			.channels_max = 8,
		},
		.name = "DP_DMA NoHost PB",
		.id = IDX_DP_DMA_NoHOST_RX,
	},

	{
		.playback = {
			.stream_name = "DP_DMA_RX Playback",
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
					SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 2,
			.channels_max = 8,
		},
		.ops = &be_dai_ops,
		.name = "DP_DMA_RX",
		.id = DP_DMA_RX,
	},

};

static int aoc_compress_new(struct snd_soc_pcm_runtime *rtd, int num)
{
	int ret = snd_soc_new_compress(rtd, num);
	if (ret >= 0) {
		rtd->pcm->nonatomic = true;
	}
	return ret;
}

static int be_startup(struct snd_pcm_substream *stream, struct snd_soc_dai *dai)
{
	pr_debug("%s: dai %s id 0x%x", __func__, dai->name, dai->id);
	return 0;
}

static int be_hw_params(struct snd_pcm_substream *stream,
			struct snd_pcm_hw_params *params,
			struct snd_soc_dai *dai)
{
	pr_debug("%s: dai %s id 0x%x", __func__, dai->name, dai->id);
	pr_debug("%s: ch %d rate %d bit %d", __func__, params_channels(params),
		params_rate(params),
		snd_pcm_format_width(params_format(params)));
	return 0;
}

static int aoc_capture_eps_trigger(struct aoc_chip *chip, int hw_id, bool on)
{
	int bit;
	uint32_t hw_idx;
	uint32_t ep_id;

	hw_idx = AOC_ID_TO_INDEX(hw_id);
	if (hw_idx >= ARRAY_SIZE(port_array)) {
		pr_err("%s: invalid idx hw_idx 0x%x", __func__,
		       hw_id);
		return -EINVAL;
	}

	for_each_set_bit(bit, port_array[hw_idx].fe_put_mask, IDX_FE_MAX) {
		ep_id = (AOC_FE|AOC_TX|bit);
		aoc_audio_capture_runtime_trigger(chip, ep_id, hw_id, on);
	}
	return 0;
}

static int be_prepare(struct snd_pcm_substream *stream, struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = stream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct aoc_chip *chip = snd_soc_card_get_drvdata(card);
	uint32_t hw_idx;

	hw_idx = AOC_ID_TO_INDEX(dai->id);

	if (hw_idx >= ARRAY_SIZE(port_array)) {
		pr_err("%s: invalid idx hw_idx 0x%x", __func__,
		       dai->id);
		return -EINVAL;
	}

	pr_info("%s: dai %s id 0x%x", __func__, dai->name, dai->id);

	mutex_lock(&path_mutex);
	switch (dai->id) {
	case INTERNAL_MIC_TX:
	case INTERNAL_MIC_US_TX:
	case ERASER_TX:
	case BT_TX:
	case USB_TX:
		mutex_lock(&chip->audio_mutex);
		aoc_capture_filter_runtime_control(chip, dai->id, true);
		aoc_capture_eps_trigger(chip, dai->id, true);
		mutex_unlock(&chip->audio_mutex);
		break;
	default:
		break;
	}
	port_array[hw_idx].on = true;
	mutex_unlock(&path_mutex);
	return 0;
}

static void be_shutdown(struct snd_pcm_substream *stream,
			struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = stream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct aoc_chip *chip = snd_soc_card_get_drvdata(card);
	uint32_t hw_idx;

	hw_idx = AOC_ID_TO_INDEX(dai->id);

	if (hw_idx >= ARRAY_SIZE(port_array)) {
		pr_err("%s: invalid idx hw_idx 0x%x", __func__,
		       dai->id);
		return;
	}

	pr_info("%s: dai %s id 0x%x", __func__, dai->name, dai->id);
	mutex_lock(&path_mutex);
	switch (dai->id) {
	case INTERNAL_MIC_TX:
	case INTERNAL_MIC_US_TX:
	case ERASER_TX:
	case BT_TX:
	case USB_TX:
		mutex_lock(&chip->audio_mutex);
		aoc_capture_filter_runtime_control(chip, dai->id, false);
		mutex_unlock(&chip->audio_mutex);
		break;
	default:
		break;
	}
	port_array[hw_idx].on = false;
	mutex_unlock(&path_mutex);
}

static const struct snd_soc_dai_ops be_dai_ops = {
	.startup = be_startup,
	.shutdown = be_shutdown,
	.hw_params = be_hw_params,
	.prepare = be_prepare,
};

static int aoc_mic_loopback_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
		(struct snd_soc_component *)snd_kcontrol_chip(kcontrol);
	struct aoc_chip *chip =
		(struct aoc_chip *)snd_soc_card_get_drvdata(component->card);

	int enable = 0;

	mutex_lock(&chip->audio_mutex);
	enable = chip->mic_loopback_enabled;
	mutex_unlock(&chip->audio_mutex);
	ucontrol->value.integer.value[0] = enable;

	return 0;
}

static int aoc_mic_loopback_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	int enable = ucontrol->value.integer.value[0];

	struct snd_soc_component *component =
		(struct snd_soc_component *)snd_kcontrol_chip(kcontrol);
	struct aoc_chip *chip =
		(struct aoc_chip *)snd_soc_card_get_drvdata(component->card);

	mutex_lock(&chip->audio_mutex);
	if (chip->mic_loopback_enabled != enable) {
		chip->mic_loopback_enabled = enable;
		aoc_mic_loopback(chip, enable);
	}
	mutex_unlock(&chip->audio_mutex);
	return 0;
}

static int aoc_default_sink_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
		(struct snd_soc_component *)snd_kcontrol_chip(kcontrol);
	struct aoc_chip *chip =
		(struct aoc_chip *)snd_soc_card_get_drvdata(component->card);
	int sink;

	mutex_lock(&chip->audio_mutex);
	sink = chip->default_sink_id;
	mutex_unlock(&chip->audio_mutex);
	ucontrol->value.integer.value[0] = sink;
	return 0;
}

static int aoc_default_sink_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	int sink = ucontrol->value.integer.value[0];
	struct snd_soc_component *component =
		(struct snd_soc_component *)snd_kcontrol_chip(kcontrol);
	struct aoc_chip *chip =
		(struct aoc_chip *)snd_soc_card_get_drvdata(component->card);

	mutex_lock(&chip->audio_mutex);
	if (chip->default_sink_id != sink) {
		chip->default_sink_id = sink;
		/* for a new default sink value, the sink list will be reset */
		chip->sink_id_list[0] = sink;
		chip->sink_id_list[1] = -1; /*TODO: refactor needed */
		pr_notice("Default sink: %d", sink);
	}
	mutex_unlock(&chip->audio_mutex);
	return 0;
}

static int aoc_default_mic_get(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
		(struct snd_soc_component *)snd_kcontrol_chip(kcontrol);
	struct aoc_chip *chip =
		(struct aoc_chip *)snd_soc_card_get_drvdata(component->card);
	int mic;

	mutex_lock(&chip->audio_mutex);
	mic = chip->default_mic_id;
	mutex_unlock(&chip->audio_mutex);
	ucontrol->value.integer.value[0] = mic;
	return 0;
}

static int aoc_default_mic_put(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	int i;
	int mic = ucontrol->value.integer.value[0];
	struct snd_soc_component *component =
		(struct snd_soc_component *)snd_kcontrol_chip(kcontrol);
	struct aoc_chip *chip =
		(struct aoc_chip *)snd_soc_card_get_drvdata(component->card);

	mutex_lock(&chip->audio_mutex);

	if (chip->default_mic_id != mic) {
		chip->default_mic_id = mic;
		chip->buildin_mic_id_list[0] = mic;
		for (i = 1; i < NUM_OF_BUILTIN_MIC; i++)
			chip->buildin_mic_id_list[i] = -1;
	}

	mutex_unlock(&chip->audio_mutex);
	return 0;
}

static int aoc_sink_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
		(struct snd_soc_component *)snd_kcontrol_chip(kcontrol);
	struct aoc_chip *chip =
		(struct aoc_chip *)snd_soc_card_get_drvdata(component->card);

	mutex_lock(&chip->audio_mutex);

	ucontrol->value.integer.value[0] =
		chip->sink_id_list[0]; /*TODO: refactor needed */
	ucontrol->value.integer.value[1] = chip->sink_id_list[1];

	mutex_unlock(&chip->audio_mutex);
	return 0;
}

static int aoc_sink_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
		(struct snd_soc_component *)snd_kcontrol_chip(kcontrol);
	struct aoc_chip *chip =
		(struct aoc_chip *)snd_soc_card_get_drvdata(component->card);

	mutex_lock(&chip->audio_mutex);
	chip->sink_id_list[0] =
		ucontrol->value.integer.value[0]; /*TODO: refactor needed*/
	chip->sink_id_list[1] = ucontrol->value.integer.value[1];
	mutex_unlock(&chip->audio_mutex);
	return 0;
}

const struct snd_kcontrol_new runtime_ctrls[] = {
	SOC_SINGLE_EXT("MIC LOOPBACK", IDX_EP1_RX, I2S_0_RX, 1, 0,
		       aoc_mic_loopback_get, aoc_mic_loopback_put),
	SOC_SINGLE_EXT("DEFAULT_SINK_ID", IDX_EP2_RX, I2S_0_RX, 4, 0,
		       aoc_default_sink_get, aoc_default_sink_put),
	SOC_SINGLE_EXT("DEFAULT_MIC_ID", IDX_EP2_RX, I2S_0_RX, 3, 0,
		       aoc_default_mic_get, aoc_default_mic_put),
	SOC_DOUBLE_EXT("SINK_IDS", IDX_EP2_RX, I2S_0_RX, I2S_0_TX, 4, 0,
		       aoc_sink_get, aoc_sink_put),
};

static int aoc_path_get(uint32_t ep_idx, uint32_t hw_idx,
			struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	int enable = 0;

	ep_idx = AOC_ID_TO_INDEX(ep_idx);
	hw_idx = AOC_ID_TO_INDEX(hw_idx);

	if (hw_idx >= PORT_MAX || ep_idx >= IDX_FE_MAX) {
		pr_err("%s: invalid idx hw_idx 0x%x ep_idx %x", __func__,
		       hw_idx, ep_idx);
		return -EINVAL;
	}

	mutex_lock(&path_mutex);
	enable = test_bit(ep_idx, port_array[hw_idx].fe_put_mask) ? 1 : 0;
	mutex_unlock(&path_mutex);

	ucontrol->value.integer.value[0] = enable;

	pr_debug("%s: get ep %u hw_id 0x%x enable %d", __func__, ep_idx, hw_idx,
		enable);
	return 0;
}

static int aoc_path_put(uint32_t ep_id, uint32_t hw_id,
			struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_widget *widget =
		snd_soc_dapm_kcontrol_widget(kcontrol);
	struct snd_soc_component *component =
		snd_soc_dapm_to_component(widget->dapm);
	struct aoc_chip *chip =
		(struct aoc_chip *)snd_soc_card_get_drvdata(component->card);

	uint32_t ep_idx, hw_idx;

	int enable = ucontrol->value.integer.value[0];
	int ret;

	ep_idx = AOC_ID_TO_INDEX(ep_id);
	hw_idx = AOC_ID_TO_INDEX(hw_id);

	if (hw_idx >= PORT_MAX || ep_idx >= IDX_FE_MAX) {
		pr_err("%s: invalid idx hw_idx 0x%x ep_idx %x", __func__,
		       hw_idx, ep_idx);
		return -EINVAL;
	}

	pr_info("%s: set ep %u hw_id 0x%x enable %d chip %p", __func__, ep_idx,
		hw_idx, enable, chip);

	mutex_lock(&path_mutex);

	if (enable) {
		set_bit(ep_idx, port_array[hw_idx].fe_put_mask);
		mutex_lock(&chip->audio_mutex);
		aoc_audio_path_open(chip, ep_id, hw_id, port_array[hw_idx].on);
		mutex_unlock(&chip->audio_mutex);
	} else {
		clear_bit(ep_idx, port_array[hw_idx].fe_put_mask);
		mutex_lock(&chip->audio_mutex);
		aoc_audio_path_close(chip, ep_id, hw_id, port_array[hw_idx].on);
		mutex_unlock(&chip->audio_mutex);
	}

	mutex_unlock(&path_mutex);

	/* Notify AoC driver here if necessary */
	ret = snd_soc_dapm_mixer_update_power(widget->dapm, kcontrol, enable,
					      NULL);
	if (ret < 0) {
		pr_warn("%s: ret %d fail to set mixer for ep %u hw 0x%x enable %d",
			__func__, ret, ep_idx, hw_idx, enable);
	}
	return 0;
}

bool aoc_alsa_usb_capture_enabled(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(tx_ep_list); i++) {
		struct snd_ctl_elem_value ucontrol;
		int ret;

		ret = aoc_path_get(tx_ep_list[i], USB_TX, NULL, &ucontrol);
		if (ret) {
			pr_err("%s failed ret %d\n", __func__, ret);
			return false;
		}

		if (ucontrol.value.integer.value[0])
			return true;
	}

	return false;
}
EXPORT_SYMBOL_GPL(aoc_alsa_usb_capture_enabled);

bool aoc_alsa_usb_playback_enabled(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(rx_ep_list); i++) {
		struct snd_ctl_elem_value ucontrol;
		int ret;

		ret = aoc_path_get(rx_ep_list[i], USB_RX, NULL, &ucontrol);
		if (ret) {
			pr_err("%s failed ret %d\n", __func__, ret);
			return false;
		}

		if (ucontrol.value.integer.value[0])
			return true;
	}

	return false;
}
EXPORT_SYMBOL_GPL(aoc_alsa_usb_playback_enabled);

bool aoc_alsa_dp_playback_enabled(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(rx_ep_list); i++) {
		struct snd_ctl_elem_value ucontrol;
		int ret;

		ret = aoc_path_get(rx_ep_list[i], DP_DMA_RX, NULL, &ucontrol);
		if (ret) {
			pr_err("%s failed ret %d\n", __func__, ret);
			return false;
		}

		if (ucontrol.value.integer.value[0])
			return true;
	}

	return false;
}

static int i2s_0_rx_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;

	u32 ep_idx = mc->shift;

	return aoc_path_put(ep_idx, I2S_0_RX, kcontrol, ucontrol);
}

static int i2s_0_rx_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	u32 ep_idx = mc->shift;

	return aoc_path_get(ep_idx, I2S_0_RX, kcontrol, ucontrol);
}

const struct snd_kcontrol_new i2s_0_rx_ctrl[] = {
	SOC_SINGLE_EXT("EP1", SND_SOC_NOPM, IDX_EP1_RX, 1, 0, i2s_0_rx_get, i2s_0_rx_put),
	SOC_SINGLE_EXT("EP2", SND_SOC_NOPM, IDX_EP2_RX, 1, 0, i2s_0_rx_get, i2s_0_rx_put),
	SOC_SINGLE_EXT("EP3", SND_SOC_NOPM, IDX_EP3_RX, 1, 0, i2s_0_rx_get, i2s_0_rx_put),
	SOC_SINGLE_EXT("EP4", SND_SOC_NOPM, IDX_EP4_RX, 1, 0, i2s_0_rx_get, i2s_0_rx_put),
	SOC_SINGLE_EXT("EP5", SND_SOC_NOPM, IDX_EP5_RX, 1, 0, i2s_0_rx_get, i2s_0_rx_put),
	SOC_SINGLE_EXT("EP6", SND_SOC_NOPM, IDX_EP6_RX, 1, 0, i2s_0_rx_get, i2s_0_rx_put),
	SOC_SINGLE_EXT("EP7", SND_SOC_NOPM, IDX_EP7_RX, 1, 0, i2s_0_rx_get, i2s_0_rx_put),
	SOC_SINGLE_EXT("VOIP", SND_SOC_NOPM, IDX_VOIP_RX, 1, 0, i2s_0_rx_get, i2s_0_rx_put),
	SOC_SINGLE_EXT("RAW", SND_SOC_NOPM, IDX_RAW_RX, 1, 0, i2s_0_rx_get, i2s_0_rx_put),
	SOC_SINGLE_EXT("NoHost1", SND_SOC_NOPM, IDX_NOHOST1_RX, 1, 0, i2s_0_rx_get, i2s_0_rx_put),
	SOC_SINGLE_EXT("US", SND_SOC_NOPM, IDX_US_RX, 1, 0, i2s_0_rx_get, i2s_0_rx_put),
	SOC_SINGLE_EXT("IMSV", SND_SOC_NOPM, IDX_IMSV_RX, 1, 0, i2s_0_rx_get, i2s_0_rx_put),
};

static int i2s_1_rx_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;

	u32 ep_idx = mc->shift;

	return aoc_path_put(ep_idx, I2S_1_RX, kcontrol, ucontrol);
}

static int i2s_1_rx_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	u32 ep_idx = mc->shift;

	return aoc_path_get(ep_idx, I2S_1_RX, kcontrol, ucontrol);
}

const struct snd_kcontrol_new i2s_1_rx_ctrl[] = {
	SOC_SINGLE_EXT("EP1", SND_SOC_NOPM, IDX_EP1_RX, 1, 0, i2s_1_rx_get, i2s_1_rx_put),
	SOC_SINGLE_EXT("EP2", SND_SOC_NOPM, IDX_EP2_RX, 1, 0, i2s_1_rx_get, i2s_1_rx_put),
	SOC_SINGLE_EXT("EP3", SND_SOC_NOPM, IDX_EP3_RX, 1, 0, i2s_1_rx_get, i2s_1_rx_put),
	SOC_SINGLE_EXT("EP4", SND_SOC_NOPM, IDX_EP4_RX, 1, 0, i2s_1_rx_get, i2s_1_rx_put),
	SOC_SINGLE_EXT("EP5", SND_SOC_NOPM, IDX_EP5_RX, 1, 0, i2s_1_rx_get, i2s_1_rx_put),
	SOC_SINGLE_EXT("EP6", SND_SOC_NOPM, IDX_EP6_RX, 1, 0, i2s_1_rx_get, i2s_1_rx_put),
	SOC_SINGLE_EXT("EP7", SND_SOC_NOPM, IDX_EP7_RX, 1, 0, i2s_1_rx_get, i2s_1_rx_put),
	SOC_SINGLE_EXT("VOIP", SND_SOC_NOPM, IDX_VOIP_RX, 1, 0, i2s_1_rx_get, i2s_1_rx_put),
	SOC_SINGLE_EXT("RAW", SND_SOC_NOPM, IDX_RAW_RX, 1, 0, i2s_1_rx_get, i2s_1_rx_put),
	SOC_SINGLE_EXT("NoHost1", SND_SOC_NOPM, IDX_NOHOST1_RX, 1, 0, i2s_1_rx_get, i2s_1_rx_put),
	SOC_SINGLE_EXT("US", SND_SOC_NOPM, IDX_US_RX, 1, 0, i2s_1_rx_get, i2s_1_rx_put),
	SOC_SINGLE_EXT("IMSV", SND_SOC_NOPM, IDX_IMSV_RX, 1, 0, i2s_1_rx_get, i2s_1_rx_put),
};

static int i2s_2_rx_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;

	u32 ep_idx = mc->shift;

	return aoc_path_put(ep_idx, I2S_2_RX, kcontrol, ucontrol);
}

static int i2s_2_rx_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	u32 ep_idx = mc->shift;

	return aoc_path_get(ep_idx, I2S_2_RX, kcontrol, ucontrol);
}

const struct snd_kcontrol_new i2s_2_rx_ctrl[] = {
	SOC_SINGLE_EXT("EP1", SND_SOC_NOPM, IDX_EP1_RX, 1, 0, i2s_2_rx_get, i2s_2_rx_put),
	SOC_SINGLE_EXT("EP2", SND_SOC_NOPM, IDX_EP2_RX, 1, 0, i2s_2_rx_get, i2s_2_rx_put),
	SOC_SINGLE_EXT("EP3", SND_SOC_NOPM, IDX_EP3_RX, 1, 0, i2s_2_rx_get, i2s_2_rx_put),
	SOC_SINGLE_EXT("EP4", SND_SOC_NOPM, IDX_EP4_RX, 1, 0, i2s_2_rx_get, i2s_2_rx_put),
	SOC_SINGLE_EXT("EP5", SND_SOC_NOPM, IDX_EP5_RX, 1, 0, i2s_2_rx_get, i2s_2_rx_put),
	SOC_SINGLE_EXT("EP6", SND_SOC_NOPM, IDX_EP6_RX, 1, 0, i2s_2_rx_get, i2s_2_rx_put),
	SOC_SINGLE_EXT("EP7", SND_SOC_NOPM, IDX_EP7_RX, 1, 0, i2s_2_rx_get, i2s_2_rx_put),
	SOC_SINGLE_EXT("VOIP", SND_SOC_NOPM, IDX_VOIP_RX, 1, 0, i2s_2_rx_get, i2s_2_rx_put),
	SOC_SINGLE_EXT("RAW", SND_SOC_NOPM, IDX_RAW_RX, 1, 0, i2s_2_rx_get, i2s_2_rx_put),
	SOC_SINGLE_EXT("NoHost1", SND_SOC_NOPM, IDX_NOHOST1_RX, 1, 0, i2s_2_rx_get, i2s_2_rx_put),
	SOC_SINGLE_EXT("US", SND_SOC_NOPM, IDX_US_RX, 1, 0, i2s_2_rx_get, i2s_2_rx_put),
	SOC_SINGLE_EXT("IMSV", SND_SOC_NOPM, IDX_IMSV_RX, 1, 0, i2s_2_rx_get, i2s_2_rx_put),
};

static int tdm_0_rx_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;

	u32 ep_idx = mc->shift;

	return aoc_path_put(ep_idx, TDM_0_RX, kcontrol, ucontrol);
}

static int tdm_0_rx_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	u32 ep_idx = mc->shift;

	return aoc_path_get(ep_idx, TDM_0_RX, kcontrol, ucontrol);
}

const struct snd_kcontrol_new tdm_0_rx_ctrl[] = {
	SOC_SINGLE_EXT("EP1", SND_SOC_NOPM, IDX_EP1_RX, 1, 0, tdm_0_rx_get, tdm_0_rx_put),
	SOC_SINGLE_EXT("EP2", SND_SOC_NOPM, IDX_EP2_RX, 1, 0, tdm_0_rx_get, tdm_0_rx_put),
	SOC_SINGLE_EXT("EP3", SND_SOC_NOPM, IDX_EP3_RX, 1, 0, tdm_0_rx_get, tdm_0_rx_put),
	SOC_SINGLE_EXT("EP4", SND_SOC_NOPM, IDX_EP4_RX, 1, 0, tdm_0_rx_get, tdm_0_rx_put),
	SOC_SINGLE_EXT("EP5", SND_SOC_NOPM, IDX_EP5_RX, 1, 0, tdm_0_rx_get, tdm_0_rx_put),
	SOC_SINGLE_EXT("EP6", SND_SOC_NOPM, IDX_EP6_RX, 1, 0, tdm_0_rx_get, tdm_0_rx_put),
	SOC_SINGLE_EXT("EP7", SND_SOC_NOPM, IDX_EP7_RX, 1, 0, tdm_0_rx_get, tdm_0_rx_put),
	SOC_SINGLE_EXT("EP8", SND_SOC_NOPM, IDX_EP8_RX, 1, 0, tdm_0_rx_get, tdm_0_rx_put),
	SOC_SINGLE_EXT("VOIP", SND_SOC_NOPM, IDX_VOIP_RX, 1, 0, tdm_0_rx_get, tdm_0_rx_put),
	SOC_SINGLE_EXT("RAW", SND_SOC_NOPM, IDX_RAW_RX, 1, 0, tdm_0_rx_get, tdm_0_rx_put),
	SOC_SINGLE_EXT("NoHost1", SND_SOC_NOPM, IDX_NOHOST1_RX, 1, 0, tdm_0_rx_get, tdm_0_rx_put),
	SOC_SINGLE_EXT("US", SND_SOC_NOPM, IDX_US_RX, 1, 0, tdm_0_rx_get, tdm_0_rx_put),
	SOC_SINGLE_EXT("IMSV", SND_SOC_NOPM, IDX_IMSV_RX, 1, 0, tdm_0_rx_get, tdm_0_rx_put),
};

static int tdm_1_rx_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;

	u32 ep_idx = mc->shift;

	return aoc_path_put(ep_idx, TDM_1_RX, kcontrol, ucontrol);
}

static int tdm_1_rx_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	u32 ep_idx = mc->shift;

	return aoc_path_get(ep_idx, TDM_1_RX, kcontrol, ucontrol);
}

const struct snd_kcontrol_new tdm_1_rx_ctrl[] = {
	SOC_SINGLE_EXT("EP1", SND_SOC_NOPM, IDX_EP1_RX, 1, 0, tdm_1_rx_get, tdm_1_rx_put),
	SOC_SINGLE_EXT("EP2", SND_SOC_NOPM, IDX_EP2_RX, 1, 0, tdm_1_rx_get, tdm_1_rx_put),
	SOC_SINGLE_EXT("EP3", SND_SOC_NOPM, IDX_EP3_RX, 1, 0, tdm_1_rx_get, tdm_1_rx_put),
	SOC_SINGLE_EXT("EP4", SND_SOC_NOPM, IDX_EP4_RX, 1, 0, tdm_1_rx_get, tdm_1_rx_put),
	SOC_SINGLE_EXT("EP5", SND_SOC_NOPM, IDX_EP5_RX, 1, 0, tdm_1_rx_get, tdm_1_rx_put),
	SOC_SINGLE_EXT("EP6", SND_SOC_NOPM, IDX_EP6_RX, 1, 0, tdm_1_rx_get, tdm_1_rx_put),
	SOC_SINGLE_EXT("EP7", SND_SOC_NOPM, IDX_EP7_RX, 1, 0, tdm_1_rx_get, tdm_1_rx_put),
	SOC_SINGLE_EXT("VOIP", SND_SOC_NOPM, IDX_VOIP_RX, 1, 0, tdm_1_rx_get, tdm_1_rx_put),
	SOC_SINGLE_EXT("RAW", SND_SOC_NOPM, IDX_RAW_RX, 1, 0, tdm_1_rx_get, tdm_1_rx_put),
	SOC_SINGLE_EXT("NoHost1", SND_SOC_NOPM, IDX_NOHOST1_RX, 1, 0,
		tdm_1_rx_get, tdm_1_rx_put),
	SOC_SINGLE_EXT("US", SND_SOC_NOPM, IDX_US_RX, 1, 0, tdm_1_rx_get, tdm_1_rx_put),
	SOC_SINGLE_EXT("IMSV", SND_SOC_NOPM, IDX_IMSV_RX, 1, 0, tdm_1_rx_get, tdm_1_rx_put),
};

static int bt_rx_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;

	u32 ep_idx = mc->shift;

	return aoc_path_put(ep_idx, BT_RX, kcontrol, ucontrol);
}

static int bt_rx_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	u32 ep_idx = mc->shift;

	return aoc_path_get(ep_idx, BT_RX, kcontrol, ucontrol);
}

const struct snd_kcontrol_new bt_rx_ctrl[] = {
	SOC_SINGLE_EXT("EP1", SND_SOC_NOPM, IDX_EP1_RX, 1, 0, bt_rx_get, bt_rx_put),
	SOC_SINGLE_EXT("EP2", SND_SOC_NOPM, IDX_EP2_RX, 1, 0, bt_rx_get, bt_rx_put),
	SOC_SINGLE_EXT("EP3", SND_SOC_NOPM, IDX_EP3_RX, 1, 0, bt_rx_get, bt_rx_put),
	SOC_SINGLE_EXT("EP4", SND_SOC_NOPM, IDX_EP4_RX, 1, 0, bt_rx_get, bt_rx_put),
	SOC_SINGLE_EXT("EP5", SND_SOC_NOPM, IDX_EP5_RX, 1, 0, bt_rx_get, bt_rx_put),
	SOC_SINGLE_EXT("EP6", SND_SOC_NOPM, IDX_EP6_RX, 1, 0, bt_rx_get, bt_rx_put),
	SOC_SINGLE_EXT("EP7", SND_SOC_NOPM, IDX_EP7_RX, 1, 0, bt_rx_get, bt_rx_put),
	SOC_SINGLE_EXT("VOIP", SND_SOC_NOPM, IDX_VOIP_RX, 1, 0, bt_rx_get, bt_rx_put),
	SOC_SINGLE_EXT("RAW", SND_SOC_NOPM, IDX_RAW_RX, 1, 0, bt_rx_get, bt_rx_put),
	SOC_SINGLE_EXT("NoHost1", SND_SOC_NOPM, IDX_NOHOST1_RX, 1, 0, bt_rx_get, bt_rx_put),
	SOC_SINGLE_EXT("IMSV", SND_SOC_NOPM, IDX_IMSV_RX, 1, 0, bt_rx_get, bt_rx_put),
};

static int usb_rx_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;

	u32 ep_idx = mc->shift;

	return aoc_path_put(ep_idx, USB_RX, kcontrol, ucontrol);
}

static int usb_rx_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	u32 ep_idx = mc->shift;

	return aoc_path_get(ep_idx, USB_RX, kcontrol, ucontrol);
}

const struct snd_kcontrol_new usb_rx_ctrl[] = {
	SOC_SINGLE_EXT("EP1", SND_SOC_NOPM, IDX_EP1_RX, 1, 0, usb_rx_get, usb_rx_put),
	SOC_SINGLE_EXT("EP2", SND_SOC_NOPM, IDX_EP2_RX, 1, 0, usb_rx_get, usb_rx_put),
	SOC_SINGLE_EXT("EP3", SND_SOC_NOPM, IDX_EP3_RX, 1, 0, usb_rx_get, usb_rx_put),
	SOC_SINGLE_EXT("EP4", SND_SOC_NOPM, IDX_EP4_RX, 1, 0, usb_rx_get, usb_rx_put),
	SOC_SINGLE_EXT("EP5", SND_SOC_NOPM, IDX_EP5_RX, 1, 0, usb_rx_get, usb_rx_put),
	SOC_SINGLE_EXT("EP6", SND_SOC_NOPM, IDX_EP6_RX, 1, 0, usb_rx_get, usb_rx_put),
	SOC_SINGLE_EXT("EP7", SND_SOC_NOPM, IDX_EP7_RX, 1, 0, usb_rx_get, usb_rx_put),
	SOC_SINGLE_EXT("VOIP", SND_SOC_NOPM, IDX_VOIP_RX, 1, 0, usb_rx_get, usb_rx_put),
	SOC_SINGLE_EXT("HIFI", SND_SOC_NOPM, IDX_HIFI_RX, 1, 0, usb_rx_get, usb_rx_put),
	SOC_SINGLE_EXT("RAW", SND_SOC_NOPM, IDX_RAW_RX, 1, 0, usb_rx_get, usb_rx_put),
	SOC_SINGLE_EXT("NoHost1", SND_SOC_NOPM, IDX_NOHOST1_RX, 1, 0, usb_rx_get, usb_rx_put),
	SOC_SINGLE_EXT("IMSV", SND_SOC_NOPM, IDX_IMSV_RX, 1, 0, usb_rx_get, usb_rx_put),
};

static int incall_rx_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;

	u32 ep_idx = mc->shift;

	return aoc_path_put(ep_idx, INCALL_RX, kcontrol, ucontrol);
}

static int incall_rx_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	u32 ep_idx = mc->shift;

	return aoc_path_get(ep_idx, INCALL_RX, kcontrol, ucontrol);
}

const struct snd_kcontrol_new incall_rx_ctrl[] = {
	SOC_SINGLE_EXT("EP1", SND_SOC_NOPM, IDX_EP1_RX, 1, 0, incall_rx_get, incall_rx_put),
	SOC_SINGLE_EXT("EP2", SND_SOC_NOPM, IDX_EP2_RX, 1, 0, incall_rx_get, incall_rx_put),
	SOC_SINGLE_EXT("EP3", SND_SOC_NOPM, IDX_EP3_RX, 1, 0, incall_rx_get, incall_rx_put),
	SOC_SINGLE_EXT("EP4", SND_SOC_NOPM, IDX_EP4_RX, 1, 0, incall_rx_get, incall_rx_put),
	SOC_SINGLE_EXT("EP5", SND_SOC_NOPM, IDX_EP5_RX, 1, 0, incall_rx_get, incall_rx_put),
	SOC_SINGLE_EXT("EP6", SND_SOC_NOPM, IDX_EP6_RX, 1, 0, incall_rx_get, incall_rx_put),
	SOC_SINGLE_EXT("EP7", SND_SOC_NOPM, IDX_EP7_RX, 1, 0, incall_rx_get, incall_rx_put),
	SOC_SINGLE_EXT("RAW", SND_SOC_NOPM, IDX_RAW_RX, 1, 0, incall_rx_get, incall_rx_put),
	SOC_SINGLE_EXT("NoHost1", SND_SOC_NOPM, IDX_NOHOST1_RX, 1, 0, incall_rx_get, incall_rx_put),
	SOC_SINGLE_EXT("IMSV", SND_SOC_NOPM, IDX_IMSV_RX, 1, 0, incall_rx_get, incall_rx_put),
};

static int haptic_rx_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;

	u32 ep_idx = mc->shift;

	return aoc_path_put(ep_idx, HAPTIC_RX, kcontrol, ucontrol);
}

static int haptic_rx_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	u32 ep_idx = mc->shift;

	return aoc_path_get(ep_idx, HAPTIC_RX, kcontrol, ucontrol);
}

const struct snd_kcontrol_new haptic_rx_ctrl[] = {
	SOC_SINGLE_EXT("EP8", SND_SOC_NOPM, IDX_EP8_RX, 1, 0, haptic_rx_get, haptic_rx_put),
};

static int ep1_tx_put(struct snd_kcontrol *kcontrol,
		      struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;

	u32 hw_idx = mc->shift;

	return aoc_path_put(IDX_EP1_TX, hw_idx, kcontrol, ucontrol);
}

static int ep1_tx_get(struct snd_kcontrol *kcontrol,
		      struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	u32 hw_idx = mc->shift;

	return aoc_path_get(IDX_EP1_TX, hw_idx, kcontrol, ucontrol);
}

const struct snd_kcontrol_new ep1_tx_ctrl[] = {
	SOC_SINGLE_EXT("I2S_0_TX", SND_SOC_NOPM, I2S_0_TX, 1, 0, ep1_tx_get, ep1_tx_put),
	SOC_SINGLE_EXT("I2S_1_TX", SND_SOC_NOPM, I2S_1_TX, 1, 0, ep1_tx_get, ep1_tx_put),
	SOC_SINGLE_EXT("I2S_2_TX", SND_SOC_NOPM, I2S_2_TX, 1, 0, ep1_tx_get, ep1_tx_put),
	SOC_SINGLE_EXT("TDM_0_TX", SND_SOC_NOPM, TDM_0_TX, 1, 0, ep1_tx_get, ep1_tx_put),
	SOC_SINGLE_EXT("TDM_1_TX", SND_SOC_NOPM, TDM_1_TX, 1, 0, ep1_tx_get, ep1_tx_put),
	SOC_SINGLE_EXT("INTERNAL_MIC_TX", SND_SOC_NOPM, INTERNAL_MIC_TX, 1, 0,
		ep1_tx_get, ep1_tx_put),
	SOC_SINGLE_EXT("ERASER_TX", SND_SOC_NOPM, ERASER_TX, 1, 0,
		ep1_tx_get, ep1_tx_put),
	SOC_SINGLE_EXT("BT_TX", SND_SOC_NOPM, BT_TX, 1, 0, ep1_tx_get, ep1_tx_put),
	SOC_SINGLE_EXT("USB_TX", SND_SOC_NOPM, USB_TX, 1, 0, ep1_tx_get, ep1_tx_put),
	SOC_SINGLE_EXT("INCALL_TX", SND_SOC_NOPM, INCALL_TX, 1, 0, ep1_tx_get, ep1_tx_put),
};

static int ep2_tx_put(struct snd_kcontrol *kcontrol,
		      struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;

	u32 hw_idx = mc->shift;

	return aoc_path_put(IDX_EP2_TX, hw_idx, kcontrol, ucontrol);
}

static int ep2_tx_get(struct snd_kcontrol *kcontrol,
		      struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	u32 hw_idx = mc->shift;

	return aoc_path_get(IDX_EP2_TX, hw_idx, kcontrol, ucontrol);
}

const struct snd_kcontrol_new ep2_tx_ctrl[] = {
	SOC_SINGLE_EXT("I2S_0_TX", SND_SOC_NOPM, I2S_0_TX, 1, 0, ep2_tx_get, ep2_tx_put),
	SOC_SINGLE_EXT("I2S_1_TX", SND_SOC_NOPM, I2S_1_TX, 1, 0, ep2_tx_get, ep2_tx_put),
	SOC_SINGLE_EXT("I2S_2_TX", SND_SOC_NOPM, I2S_2_TX, 1, 0, ep2_tx_get, ep2_tx_put),
	SOC_SINGLE_EXT("TDM_0_TX", SND_SOC_NOPM, TDM_0_TX, 1, 0, ep2_tx_get, ep2_tx_put),
	SOC_SINGLE_EXT("TDM_1_TX", SND_SOC_NOPM, TDM_1_TX, 1, 0, ep2_tx_get, ep2_tx_put),
	SOC_SINGLE_EXT("INTERNAL_MIC_TX", SND_SOC_NOPM, INTERNAL_MIC_TX, 1, 0,
		ep2_tx_get, ep2_tx_put),
	SOC_SINGLE_EXT("ERASER_TX", SND_SOC_NOPM, ERASER_TX, 1, 0,
		ep2_tx_get, ep2_tx_put),
	SOC_SINGLE_EXT("BT_TX", SND_SOC_NOPM, BT_TX, 1, 0, ep2_tx_get, ep2_tx_put),
	SOC_SINGLE_EXT("USB_TX", SND_SOC_NOPM, USB_TX, 1, 0, ep2_tx_get, ep2_tx_put),
	SOC_SINGLE_EXT("INCALL_TX", SND_SOC_NOPM, INCALL_TX, 1, 0, ep2_tx_get, ep2_tx_put),
};

static int ep3_tx_put(struct snd_kcontrol *kcontrol,
		      struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;

	u32 hw_idx = mc->shift;

	return aoc_path_put(IDX_EP3_TX, hw_idx, kcontrol, ucontrol);
}

static int ep3_tx_get(struct snd_kcontrol *kcontrol,
		      struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	u32 hw_idx = mc->shift;

	return aoc_path_get(IDX_EP3_TX, hw_idx, kcontrol, ucontrol);
}

const struct snd_kcontrol_new ep3_tx_ctrl[] = {
	SOC_SINGLE_EXT("I2S_0_TX", SND_SOC_NOPM, I2S_0_TX, 1, 0, ep3_tx_get, ep3_tx_put),
	SOC_SINGLE_EXT("I2S_1_TX", SND_SOC_NOPM, I2S_1_TX, 1, 0, ep3_tx_get, ep3_tx_put),
	SOC_SINGLE_EXT("I2S_2_TX", SND_SOC_NOPM, I2S_2_TX, 1, 0, ep3_tx_get, ep3_tx_put),
	SOC_SINGLE_EXT("TDM_0_TX", SND_SOC_NOPM, TDM_0_TX, 1, 0, ep3_tx_get, ep3_tx_put),
	SOC_SINGLE_EXT("TDM_1_TX", SND_SOC_NOPM, TDM_1_TX, 1, 0, ep3_tx_get, ep3_tx_put),
	SOC_SINGLE_EXT("INTERNAL_MIC_TX", SND_SOC_NOPM, INTERNAL_MIC_TX, 1, 0,
		ep3_tx_get, ep3_tx_put),
	SOC_SINGLE_EXT("ERASER_TX", SND_SOC_NOPM, ERASER_TX, 1, 0,
		ep3_tx_get, ep3_tx_put),
	SOC_SINGLE_EXT("BT_TX", SND_SOC_NOPM, BT_TX, 1, 0, ep3_tx_get, ep3_tx_put),
	SOC_SINGLE_EXT("USB_TX", SND_SOC_NOPM, USB_TX, 1, 0, ep3_tx_get, ep3_tx_put),
	SOC_SINGLE_EXT("INCALL_TX", SND_SOC_NOPM, INCALL_TX, 1, 0, ep3_tx_get, ep3_tx_put),
};

static int ep4_tx_put(struct snd_kcontrol *kcontrol,
		      struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;

	u32 hw_idx = mc->shift;

	return aoc_path_put(IDX_EP4_TX, hw_idx, kcontrol, ucontrol);
}

static int ep4_tx_get(struct snd_kcontrol *kcontrol,
		      struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	u32 hw_idx = mc->shift;

	return aoc_path_get(IDX_EP4_TX, hw_idx, kcontrol, ucontrol);
}

const struct snd_kcontrol_new ep4_tx_ctrl[] = {
	SOC_SINGLE_EXT("I2S_0_TX", SND_SOC_NOPM, I2S_0_TX, 1, 0, ep4_tx_get, ep4_tx_put),
	SOC_SINGLE_EXT("I2S_1_TX", SND_SOC_NOPM, I2S_1_TX, 1, 0, ep4_tx_get, ep4_tx_put),
	SOC_SINGLE_EXT("I2S_2_TX", SND_SOC_NOPM, I2S_2_TX, 1, 0, ep4_tx_get, ep4_tx_put),
	SOC_SINGLE_EXT("TDM_0_TX", SND_SOC_NOPM, TDM_0_TX, 1, 0, ep4_tx_get, ep4_tx_put),
	SOC_SINGLE_EXT("TDM_1_TX", SND_SOC_NOPM, TDM_1_TX, 1, 0, ep4_tx_get, ep4_tx_put),
	SOC_SINGLE_EXT("INTERNAL_MIC_TX", SND_SOC_NOPM, INTERNAL_MIC_TX, 1, 0,
		ep4_tx_get, ep4_tx_put),
	SOC_SINGLE_EXT("ERASER_TX", SND_SOC_NOPM, ERASER_TX, 1, 0,
		ep4_tx_get, ep4_tx_put),
	SOC_SINGLE_EXT("BT_TX", SND_SOC_NOPM, BT_TX, 1, 0, ep4_tx_get, ep4_tx_put),
	SOC_SINGLE_EXT("USB_TX", SND_SOC_NOPM, USB_TX, 1, 0, ep4_tx_get, ep4_tx_put),
	SOC_SINGLE_EXT("INCALL_TX", SND_SOC_NOPM, INCALL_TX, 1, 0, ep4_tx_get, ep4_tx_put),
};

static int ep5_tx_put(struct snd_kcontrol *kcontrol,
		      struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;

	u32 hw_idx = mc->shift;

	return aoc_path_put(IDX_EP5_TX, hw_idx, kcontrol, ucontrol);
}

static int ep5_tx_get(struct snd_kcontrol *kcontrol,
		      struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	u32 hw_idx = mc->shift;

	return aoc_path_get(IDX_EP5_TX, hw_idx, kcontrol, ucontrol);
}

const struct snd_kcontrol_new ep5_tx_ctrl[] = {
	SOC_SINGLE_EXT("I2S_0_TX", SND_SOC_NOPM, I2S_0_TX, 1, 0, ep5_tx_get, ep5_tx_put),
	SOC_SINGLE_EXT("I2S_1_TX", SND_SOC_NOPM, I2S_1_TX, 1, 0, ep5_tx_get, ep5_tx_put),
	SOC_SINGLE_EXT("I2S_2_TX", SND_SOC_NOPM, I2S_2_TX, 1, 0, ep5_tx_get, ep5_tx_put),
	SOC_SINGLE_EXT("TDM_0_TX", SND_SOC_NOPM, TDM_0_TX, 1, 0, ep5_tx_get, ep5_tx_put),
	SOC_SINGLE_EXT("TDM_1_TX", SND_SOC_NOPM, TDM_1_TX, 1, 0, ep5_tx_get, ep5_tx_put),
	SOC_SINGLE_EXT("INTERNAL_MIC_TX", SND_SOC_NOPM, INTERNAL_MIC_TX, 1, 0,
		ep5_tx_get, ep5_tx_put),
	SOC_SINGLE_EXT("INTERNAL_MIC_US_TX", SND_SOC_NOPM, INTERNAL_MIC_US_TX, 1, 0,
		ep5_tx_get, ep5_tx_put),
	SOC_SINGLE_EXT("ERASER_TX", SND_SOC_NOPM, ERASER_TX, 1, 0,
		ep5_tx_get, ep5_tx_put),
	SOC_SINGLE_EXT("BT_TX", SND_SOC_NOPM, BT_TX, 1, 0, ep5_tx_get, ep5_tx_put),
	SOC_SINGLE_EXT("USB_TX", SND_SOC_NOPM, USB_TX, 1, 0, ep5_tx_get, ep5_tx_put),
	SOC_SINGLE_EXT("INCALL_TX", SND_SOC_NOPM, INCALL_TX, 1, 0, ep5_tx_get, ep5_tx_put),
};

static int ep6_tx_put(struct snd_kcontrol *kcontrol,
		      struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;

	u32 hw_idx = mc->shift;

	return aoc_path_put(IDX_EP6_TX, hw_idx, kcontrol, ucontrol);
}

static int ep6_tx_get(struct snd_kcontrol *kcontrol,
		      struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	u32 hw_idx = mc->shift;

	return aoc_path_get(IDX_EP6_TX, hw_idx, kcontrol, ucontrol);
}

const struct snd_kcontrol_new ep6_tx_ctrl[] = {
	SOC_SINGLE_EXT("I2S_0_TX", SND_SOC_NOPM, I2S_0_TX, 1, 0, ep6_tx_get, ep6_tx_put),
	SOC_SINGLE_EXT("I2S_1_TX", SND_SOC_NOPM, I2S_1_TX, 1, 0, ep6_tx_get, ep6_tx_put),
	SOC_SINGLE_EXT("I2S_2_TX", SND_SOC_NOPM, I2S_2_TX, 1, 0, ep6_tx_get, ep6_tx_put),
	SOC_SINGLE_EXT("TDM_0_TX", SND_SOC_NOPM, TDM_0_TX, 1, 0, ep6_tx_get, ep6_tx_put),
	SOC_SINGLE_EXT("TDM_1_TX", SND_SOC_NOPM, TDM_1_TX, 1, 0, ep6_tx_get, ep6_tx_put),
	SOC_SINGLE_EXT("INTERNAL_MIC_TX", SND_SOC_NOPM, INTERNAL_MIC_TX, 1, 0,
		ep6_tx_get, ep6_tx_put),
	SOC_SINGLE_EXT("ERASER_TX", SND_SOC_NOPM, ERASER_TX, 1, 0,
		ep6_tx_get, ep6_tx_put),
	SOC_SINGLE_EXT("BT_TX", SND_SOC_NOPM, BT_TX, 1, 0, ep6_tx_get, ep6_tx_put),
	SOC_SINGLE_EXT("USB_TX", SND_SOC_NOPM, USB_TX, 1, 0, ep6_tx_get, ep6_tx_put),
	SOC_SINGLE_EXT("INCALL_TX", SND_SOC_NOPM, INCALL_TX, 1, 0, ep6_tx_get, ep6_tx_put),
};

static int voip_tx_put(struct snd_kcontrol *kcontrol,
		      struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	u32 hw_idx = mc->shift;

	return aoc_path_put(IDX_VOIP_TX, hw_idx, kcontrol, ucontrol);
}

static int voip_tx_get(struct snd_kcontrol *kcontrol,
		      struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	u32 hw_idx = mc->shift;

	return aoc_path_get(IDX_VOIP_TX, hw_idx, kcontrol, ucontrol);
}

const struct snd_kcontrol_new voip_tx_ctrl[] = {
	SOC_SINGLE_EXT("I2S_0_TX", SND_SOC_NOPM, I2S_0_TX, 1, 0, voip_tx_get, voip_tx_put),
	SOC_SINGLE_EXT("I2S_1_TX", SND_SOC_NOPM, I2S_1_TX, 1, 0, voip_tx_get, voip_tx_put),
	SOC_SINGLE_EXT("I2S_2_TX", SND_SOC_NOPM, I2S_2_TX, 1, 0, voip_tx_get, voip_tx_put),
	SOC_SINGLE_EXT("TDM_0_TX", SND_SOC_NOPM, TDM_0_TX, 1, 0, voip_tx_get, voip_tx_put),
	SOC_SINGLE_EXT("TDM_1_TX", SND_SOC_NOPM, TDM_1_TX, 1, 0, voip_tx_get, voip_tx_put),
	SOC_SINGLE_EXT("INTERNAL_MIC_TX", SND_SOC_NOPM, INTERNAL_MIC_TX, 1, 0,
		voip_tx_get, voip_tx_put),
	SOC_SINGLE_EXT("ERASER_TX", SND_SOC_NOPM, ERASER_TX, 1, 0,
		voip_tx_get, voip_tx_put),
	SOC_SINGLE_EXT("BT_TX", SND_SOC_NOPM, BT_TX, 1, 0, voip_tx_get, voip_tx_put),
	SOC_SINGLE_EXT("USB_TX", SND_SOC_NOPM, USB_TX, 1, 0, voip_tx_get, voip_tx_put),
};

static int nohost1_tx_put(struct snd_kcontrol *kcontrol,
		      struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	u32 hw_idx = mc->shift;

	return aoc_path_put(IDX_NOHOST1_TX, hw_idx, kcontrol, ucontrol);
}

static int nohost1_tx_get(struct snd_kcontrol *kcontrol,
		      struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	u32 hw_idx = mc->shift;

	return aoc_path_get(IDX_NOHOST1_TX, hw_idx, kcontrol, ucontrol);
}

const struct snd_kcontrol_new nohost1_tx_ctrl[] = {
	SOC_SINGLE_EXT("I2S_0_TX", SND_SOC_NOPM, I2S_0_TX, 1, 0, nohost1_tx_get, nohost1_tx_put),
	SOC_SINGLE_EXT("I2S_1_TX", SND_SOC_NOPM, I2S_1_TX, 1, 0, nohost1_tx_get, nohost1_tx_put),
	SOC_SINGLE_EXT("I2S_2_TX", SND_SOC_NOPM, I2S_2_TX, 1, 0, nohost1_tx_get, nohost1_tx_put),
	SOC_SINGLE_EXT("TDM_0_TX", SND_SOC_NOPM, TDM_0_TX, 1, 0, nohost1_tx_get, nohost1_tx_put),
	SOC_SINGLE_EXT("TDM_1_TX", SND_SOC_NOPM, TDM_1_TX, 1, 0, nohost1_tx_get, nohost1_tx_put),
	SOC_SINGLE_EXT("INTERNAL_MIC_TX", SND_SOC_NOPM, INTERNAL_MIC_TX, 1, 0,
		nohost1_tx_get, nohost1_tx_put),
	SOC_SINGLE_EXT("ERASER_TX", SND_SOC_NOPM, ERASER_TX, 1, 0,
		nohost1_tx_get, nohost1_tx_put),
	SOC_SINGLE_EXT("BT_TX", SND_SOC_NOPM, BT_TX, 1, 0, nohost1_tx_get, nohost1_tx_put),
	SOC_SINGLE_EXT("USB_TX", SND_SOC_NOPM, USB_TX, 1, 0, nohost1_tx_get, nohost1_tx_put),
	SOC_SINGLE_EXT("INCALL_TX", SND_SOC_NOPM, INCALL_TX, 1, 0, nohost1_tx_get, nohost1_tx_put),
};

const struct snd_soc_dapm_widget aoc_widget[] = {

	/* FE */
	/* Audio playback */
	SND_SOC_DAPM_AIF_IN("EP1_RX", "EP1 Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("EP2_RX", "EP2 Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("EP3_RX", "EP3 Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("EP4_RX", "EP4 Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("EP5_RX", "EP5 Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("EP6_RX", "EP6 Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("EP7_RX", "EP7 Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("EP8_RX", "EP8 Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("VOIP_RX", "audio_voip_rx", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("HIFI_RX", "audio_hifiout", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("RAW_RX", "audio_raw", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("US_RX", "audio_ultrasonic", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("IMSV_RX", "audio_immersive", 0, SND_SOC_NOPM, 0, 0),

	/* Audio record */
	SND_SOC_DAPM_AIF_OUT("EP1_TX", "EP1 Capture", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("EP2_TX", "EP2 Capture", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("EP3_TX", "EP3 Capture", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("EP4_TX", "EP4 Capture", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("EP5_TX", "EP5 Capture", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("EP6_TX", "EP6 Capture", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("VOIP_TX", "audio_voip_tx", 0, SND_SOC_NOPM, 0, 0),

	/* NoHost FE */
	/* RX */
	SND_SOC_DAPM_AIF_IN("NoHost1_RX", "NoHost1 Playback", 0,
		SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("HAPTIC_NoHost_RX", "HAPTIC NoHost Playback", 0,
		SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("DP_DMA_NoHost_RX", "DP_DMA NoHost Playback", 0,
		SND_SOC_NOPM, 0, 0),
	/* TX */
	SND_SOC_DAPM_AIF_OUT("NoHost1_TX", "NoHost1 Capture", 0,
		SND_SOC_NOPM, 0, 0),

	/* BE */
	SND_SOC_DAPM_AIF_OUT("I2S_0_RX", "I2S_0_RX", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("I2S_1_RX", "I2S_1_RX", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("I2S_2_RX", "I2S_1_RX", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("TDM_0_RX", "TDM_0_RX", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("TDM_1_RX", "TDM_1_RX", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("BT_RX", "BT_RX", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("USB_RX", "USB_RX", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("INCALL_RX", "INCALL_RX", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("HAPTIC_RX", "HAPTIC_RX", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("DP_DMA_RX", "DP_DMA_RX", 0, SND_SOC_NOPM, 0, 0),

	SND_SOC_DAPM_AIF_IN("I2S_0_TX", "I2S_0_TX", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("I2S_1_TX", "I2S_1_TX", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("I2S_2_TX", "I2S_2_TX", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("TDM_0_TX", "TDM_0_TX", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("TDM_1_TX", "TDM_1_TX", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("INTERNAL_MIC_TX", "INTERNAL_MIC_TX",
		0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("INTERNAL_MIC_US_TX", "INTERNAL_MIC_US_TX",
		0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("ERASER_TX", "ERASER_TX",
		0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("BT_TX", "BT_TX", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("USB_TX", "USB_TX", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("INCALL_TX", "INCALL_TX", 0, SND_SOC_NOPM, 0, 0),

	SND_SOC_DAPM_OUTPUT("HW_SINK"),
	SND_SOC_DAPM_INPUT("HW_SOURCE"),

	/* Playback path */
	SND_SOC_DAPM_MIXER("I2S_0_RX Mixer", SND_SOC_NOPM, 0, 0, i2s_0_rx_ctrl,
			   ARRAY_SIZE(i2s_0_rx_ctrl)),

	SND_SOC_DAPM_MIXER("I2S_1_RX Mixer", SND_SOC_NOPM, 0, 0, i2s_1_rx_ctrl,
			   ARRAY_SIZE(i2s_1_rx_ctrl)),

	SND_SOC_DAPM_MIXER("I2S_2_RX Mixer", SND_SOC_NOPM, 0, 0, i2s_2_rx_ctrl,
			   ARRAY_SIZE(i2s_2_rx_ctrl)),

	SND_SOC_DAPM_MIXER("TDM_0_RX Mixer", SND_SOC_NOPM, 0, 0, tdm_0_rx_ctrl,
			   ARRAY_SIZE(tdm_0_rx_ctrl)),

	SND_SOC_DAPM_MIXER("TDM_1_RX Mixer", SND_SOC_NOPM, 0, 0, tdm_1_rx_ctrl,
			   ARRAY_SIZE(tdm_1_rx_ctrl)),

	SND_SOC_DAPM_MIXER("BT_RX Mixer", SND_SOC_NOPM, 0, 0, bt_rx_ctrl,
			   ARRAY_SIZE(bt_rx_ctrl)),

	SND_SOC_DAPM_MIXER("USB_RX Mixer", SND_SOC_NOPM, 0, 0, usb_rx_ctrl,
			   ARRAY_SIZE(usb_rx_ctrl)),

	SND_SOC_DAPM_MIXER("INCALL_RX Mixer", SND_SOC_NOPM, 0, 0, incall_rx_ctrl,
			   ARRAY_SIZE(incall_rx_ctrl)),

	SND_SOC_DAPM_MIXER("HAPTIC_RX Mixer", SND_SOC_NOPM, 0, 0, haptic_rx_ctrl,
			   ARRAY_SIZE(haptic_rx_ctrl)),

	/* Record path */
	SND_SOC_DAPM_MIXER("EP1 TX Mixer", SND_SOC_NOPM, 0, 0, ep1_tx_ctrl,
			   ARRAY_SIZE(ep1_tx_ctrl)),

	SND_SOC_DAPM_MIXER("EP2 TX Mixer", SND_SOC_NOPM, 0, 0, ep2_tx_ctrl,
			   ARRAY_SIZE(ep2_tx_ctrl)),

	SND_SOC_DAPM_MIXER("EP3 TX Mixer", SND_SOC_NOPM, 0, 0, ep3_tx_ctrl,
			   ARRAY_SIZE(ep3_tx_ctrl)),

	SND_SOC_DAPM_MIXER("EP4 TX Mixer", SND_SOC_NOPM, 0, 0, ep4_tx_ctrl,
			   ARRAY_SIZE(ep4_tx_ctrl)),

	SND_SOC_DAPM_MIXER("EP5 TX Mixer", SND_SOC_NOPM, 0, 0, ep5_tx_ctrl,
			   ARRAY_SIZE(ep5_tx_ctrl)),

	SND_SOC_DAPM_MIXER("EP6 TX Mixer", SND_SOC_NOPM, 0, 0, ep6_tx_ctrl,
			   ARRAY_SIZE(ep6_tx_ctrl)),

	/* NoHost TX path */
	SND_SOC_DAPM_MIXER("VOIP TX Mixer", SND_SOC_NOPM, 0, 0,
		voip_tx_ctrl, ARRAY_SIZE(voip_tx_ctrl)),

	/* NoHost TX path */
	SND_SOC_DAPM_MIXER("NoHost1 TX Mixer", SND_SOC_NOPM, 0, 0,
		nohost1_tx_ctrl, ARRAY_SIZE(nohost1_tx_ctrl)),
};

static const struct snd_soc_dapm_route aoc_routes[] = {
	{ "I2S_0_RX Mixer", "EP1", "EP1_RX" },
	{ "I2S_0_RX Mixer", "EP2", "EP2_RX" },
	{ "I2S_0_RX Mixer", "EP3", "EP3_RX" },
	{ "I2S_0_RX Mixer", "EP4", "EP4_RX" },
	{ "I2S_0_RX Mixer", "EP5", "EP5_RX" },
	{ "I2S_0_RX Mixer", "EP6", "EP6_RX" },
	{ "I2S_0_RX Mixer", "EP7", "EP7_RX" },
	{ "I2S_0_RX Mixer", "VOIP", "VOIP_RX" },
	{ "I2S_0_RX Mixer", "RAW", "RAW_RX" },
	{ "I2S_0_RX Mixer", "NoHost1", "NoHost1_RX" },
	{ "I2S_0_RX Mixer", "US", "US_RX" },
	{ "I2S_0_RX Mixer", "IMSV", "IMSV_RX" },
	{ "I2S_0_RX", NULL, "I2S_0_RX Mixer" },
	{ "HW_SINK", NULL, "I2S_0_RX" },

	{ "I2S_1_RX Mixer", "EP1", "EP1_RX" },
	{ "I2S_1_RX Mixer", "EP2", "EP2_RX" },
	{ "I2S_1_RX Mixer", "EP3", "EP3_RX" },
	{ "I2S_1_RX Mixer", "EP4", "EP4_RX" },
	{ "I2S_1_RX Mixer", "EP5", "EP5_RX" },
	{ "I2S_1_RX Mixer", "EP6", "EP6_RX" },
	{ "I2S_1_RX Mixer", "EP7", "EP7_RX" },
	{ "I2S_1_RX Mixer", "VOIP", "VOIP_RX" },
	{ "I2S_1_RX Mixer", "RAW", "RAW_RX" },
	{ "I2S_1_RX Mixer", "NoHost1", "NoHost1_RX" },
	{ "I2S_1_RX Mixer", "US", "US_RX" },
	{ "I2S_1_RX Mixer", "IMSV", "IMSV_RX" },
	{ "I2S_1_RX", NULL, "I2S_1_RX Mixer" },
	{ "HW_SINK", NULL, "I2S_1_RX" },

	{ "I2S_2_RX Mixer", "EP1", "EP1_RX" },
	{ "I2S_2_RX Mixer", "EP2", "EP2_RX" },
	{ "I2S_2_RX Mixer", "EP3", "EP3_RX" },
	{ "I2S_2_RX Mixer", "EP4", "EP4_RX" },
	{ "I2S_2_RX Mixer", "EP5", "EP5_RX" },
	{ "I2S_2_RX Mixer", "EP6", "EP6_RX" },
	{ "I2S_2_RX Mixer", "EP7", "EP7_RX" },
	{ "I2S_2_RX Mixer", "VOIP", "VOIP_RX" },
	{ "I2S_2_RX Mixer", "RAW", "RAW_RX" },
	{ "I2S_2_RX Mixer", "NoHost1", "NoHost1_RX" },
	{ "I2S_2_RX Mixer", "US", "US_RX" },
	{ "I2S_2_RX Mixer", "IMSV", "IMSV_RX" },
	{ "I2S_2_RX", NULL, "I2S_2_RX Mixer" },
	{ "HW_SINK", NULL, "I2S_2_RX" },

	{ "TDM_0_RX Mixer", "EP1", "EP1_RX" },
	{ "TDM_0_RX Mixer", "EP2", "EP2_RX" },
	{ "TDM_0_RX Mixer", "EP3", "EP3_RX" },
	{ "TDM_0_RX Mixer", "EP4", "EP4_RX" },
	{ "TDM_0_RX Mixer", "EP5", "EP5_RX" },
	{ "TDM_0_RX Mixer", "EP6", "EP6_RX" },
	{ "TDM_0_RX Mixer", "EP7", "EP7_RX" },
	{ "TDM_0_RX Mixer", "EP8", "EP8_RX" },
	{ "TDM_0_RX Mixer", "VOIP", "VOIP_RX" }, //why we have EP8 here?
	{ "TDM_0_RX Mixer", "RAW", "RAW_RX" },
	{ "TDM_0_RX Mixer", "NoHost1", "NoHost1_RX" },
	{ "TDM_0_RX Mixer", "US", "US_RX" },
	{ "TDM_0_RX Mixer", "IMSV", "IMSV_RX" },
	{ "TDM_0_RX", NULL, "TDM_0_RX Mixer" },
	{ "HW_SINK", NULL, "TDM_0_RX" },

	{ "TDM_1_RX Mixer", "EP1", "EP1_RX" },
	{ "TDM_1_RX Mixer", "EP2", "EP2_RX" },
	{ "TDM_1_RX Mixer", "EP3", "EP3_RX" },
	{ "TDM_1_RX Mixer", "EP4", "EP4_RX" },
	{ "TDM_1_RX Mixer", "EP5", "EP5_RX" },
	{ "TDM_1_RX Mixer", "EP6", "EP6_RX" },
	{ "TDM_1_RX Mixer", "EP7", "EP7_RX" },
	{ "TDM_1_RX Mixer", "VOIP", "VOIP_RX" },
	{ "TDM_1_RX Mixer", "RAW", "RAW_RX" },
	{ "TDM_1_RX Mixer", "NoHost1", "NoHost1_RX" },
	{ "TDM_1_RX Mixer", "US", "US_RX" },
	{ "TDM_1_RX Mixer", "IMSV", "IMSV_RX" },
	{ "TDM_1_RX", NULL, "TDM_1_RX Mixer" },
	{ "HW_SINK", NULL, "TDM_1_RX" },

	{ "BT_RX Mixer", "EP1", "EP1_RX" },
	{ "BT_RX Mixer", "EP2", "EP2_RX" },
	{ "BT_RX Mixer", "EP3", "EP3_RX" },
	{ "BT_RX Mixer", "EP4", "EP4_RX" },
	{ "BT_RX Mixer", "EP5", "EP5_RX" },
	{ "BT_RX Mixer", "EP6", "EP6_RX" },
	{ "BT_RX Mixer", "EP7", "EP7_RX" },
	{ "BT_RX Mixer", "VOIP", "VOIP_RX" },
	{ "BT_RX Mixer", "RAW", "RAW_RX" },
	{ "BT_RX Mixer", "NoHost1", "NoHost1_RX" },
	{ "BT_RX Mixer", "IMSV", "IMSV_RX" },
	{ "BT_RX", NULL, "BT_RX Mixer" },
	{ "HW_SINK", NULL, "BT_RX" },

	{ "USB_RX Mixer", "EP1", "EP1_RX" },
	{ "USB_RX Mixer", "EP2", "EP2_RX" },
	{ "USB_RX Mixer", "EP3", "EP3_RX" },
	{ "USB_RX Mixer", "EP4", "EP4_RX" },
	{ "USB_RX Mixer", "EP5", "EP5_RX" },
	{ "USB_RX Mixer", "EP6", "EP6_RX" },
	{ "USB_RX Mixer", "EP7", "EP7_RX" },
	{ "USB_RX Mixer", "VOIP", "VOIP_RX" },
	{ "USB_RX Mixer", "HIFI", "HIFI_RX" },
	{ "USB_RX Mixer", "RAW", "RAW_RX" },
	{ "USB_RX Mixer", "NoHost1", "NoHost1_RX" },
	{ "USB_RX Mixer", "IMSV", "IMSV_RX" },
	{ "USB_RX", NULL, "USB_RX Mixer" },
	{ "HW_SINK", NULL, "USB_RX" },

	{ "INCALL_RX Mixer", "EP1", "EP1_RX" },
	{ "INCALL_RX Mixer", "EP2", "EP2_RX" },
	{ "INCALL_RX Mixer", "EP3", "EP3_RX" },
	{ "INCALL_RX Mixer", "EP4", "EP4_RX" },
	{ "INCALL_RX Mixer", "EP5", "EP5_RX" },
	{ "INCALL_RX Mixer", "EP6", "EP6_RX" },
	{ "INCALL_RX Mixer", "EP7", "EP7_RX" },
	{ "INCALL_RX Mixer", "RAW", "RAW_RX" },
	{ "INCALL_RX Mixer", "NoHost1", "NoHost1_RX" },
	{ "INCALL_RX Mixer", "IMSV", "IMSV_RX" },
	{ "INCALL_RX", NULL, "INCALL_RX Mixer" },
	{ "HW_SINK", NULL, "INCALL_RX" },

	{ "HAPTIC_RX Mixer", "EP8", "EP8_RX" },
	{ "HAPTIC_RX", NULL, "HAPTIC_RX Mixer" },

	{ "HAPTIC_RX", NULL, "HAPTIC_NoHost_RX" },
	{ "HW_SINK", NULL, "HAPTIC_RX" },

	{ "DP_DMA_RX", NULL, "DP_DMA_NoHost_RX" },
	{ "HW_SINK", NULL, "DP_DMA_RX" },

	{ "EP1_TX", NULL, "EP1 TX Mixer" },
	{ "EP1 TX Mixer", "I2S_0_TX", "I2S_0_TX" },
	{ "EP1 TX Mixer", "I2S_1_TX", "I2S_1_TX" },
	{ "EP1 TX Mixer", "I2S_2_TX", "I2S_2_TX" },
	{ "EP1 TX Mixer", "TDM_0_TX", "TDM_0_TX" },
	{ "EP1 TX Mixer", "TDM_1_TX", "TDM_1_TX" },
	{ "EP1 TX Mixer", "INTERNAL_MIC_TX", "INTERNAL_MIC_TX" },
	{ "EP1 TX Mixer", "ERASER_TX", "ERASER_TX" },
	{ "EP1 TX Mixer", "BT_TX", "BT_TX" },
	{ "EP1 TX Mixer", "USB_TX", "USB_TX" },
	{ "EP1 TX Mixer", "INCALL_TX", "INCALL_TX" },

	{ "EP2_TX", NULL, "EP2 TX Mixer" },
	{ "EP2 TX Mixer", "I2S_0_TX", "I2S_0_TX" },
	{ "EP2 TX Mixer", "I2S_1_TX", "I2S_1_TX" },
	{ "EP2 TX Mixer", "I2S_2_TX", "I2S_2_TX" },
	{ "EP2 TX Mixer", "TDM_0_TX", "TDM_0_TX" },
	{ "EP2 TX Mixer", "TDM_1_TX", "TDM_1_TX" },
	{ "EP2 TX Mixer", "INTERNAL_MIC_TX", "INTERNAL_MIC_TX" },
	{ "EP2 TX Mixer", "ERASER_TX", "ERASER_TX" },
	{ "EP2 TX Mixer", "BT_TX", "BT_TX" },
	{ "EP2 TX Mixer", "USB_TX", "USB_TX" },
	{ "EP2 TX Mixer", "INCALL_TX", "INCALL_TX" },

	{ "EP3_TX", NULL, "EP3 TX Mixer" },
	{ "EP3 TX Mixer", "I2S_0_TX", "I2S_0_TX" },
	{ "EP3 TX Mixer", "I2S_1_TX", "I2S_1_TX" },
	{ "EP3 TX Mixer", "I2S_2_TX", "I2S_2_TX" },
	{ "EP3 TX Mixer", "TDM_0_TX", "TDM_0_TX" },
	{ "EP3 TX Mixer", "TDM_1_TX", "TDM_1_TX" },
	{ "EP3 TX Mixer", "INTERNAL_MIC_TX", "INTERNAL_MIC_TX" },
	{ "EP3 TX Mixer", "ERASER_TX", "ERASER_TX" },
	{ "EP3 TX Mixer", "BT_TX", "BT_TX" },
	{ "EP3 TX Mixer", "USB_TX", "USB_TX" },
	{ "EP3 TX Mixer", "INCALL_TX", "INCALL_TX" },

	{ "EP4_TX", NULL, "EP4 TX Mixer" },
	{ "EP4 TX Mixer", "I2S_0_TX", "I2S_0_TX" },
	{ "EP4 TX Mixer", "I2S_1_TX", "I2S_1_TX" },
	{ "EP4 TX Mixer", "I2S_2_TX", "I2S_2_TX" },
	{ "EP4 TX Mixer", "TDM_0_TX", "TDM_0_TX" },
	{ "EP4 TX Mixer", "TDM_1_TX", "TDM_1_TX" },
	{ "EP4 TX Mixer", "INTERNAL_MIC_TX", "INTERNAL_MIC_TX" },
	{ "EP4 TX Mixer", "ERASER_TX", "ERASER_TX" },
	{ "EP4 TX Mixer", "BT_TX", "BT_TX" },
	{ "EP4 TX Mixer", "USB_TX", "USB_TX" },
	{ "EP4 TX Mixer", "INCALL_TX", "INCALL_TX" },

	{ "EP5_TX", NULL, "EP5 TX Mixer" },
	{ "EP5 TX Mixer", "I2S_0_TX", "I2S_0_TX" },
	{ "EP5 TX Mixer", "I2S_1_TX", "I2S_1_TX" },
	{ "EP5 TX Mixer", "I2S_2_TX", "I2S_2_TX" },
	{ "EP5 TX Mixer", "TDM_0_TX", "TDM_0_TX" },
	{ "EP5 TX Mixer", "TDM_1_TX", "TDM_1_TX" },
	{ "EP5 TX Mixer", "INTERNAL_MIC_TX", "INTERNAL_MIC_TX" },
	{ "EP5 TX Mixer", "INTERNAL_MIC_US_TX", "INTERNAL_MIC_US_TX" },
	{ "EP5 TX Mixer", "ERASER_TX", "ERASER_TX" },
	{ "EP5 TX Mixer", "BT_TX", "BT_TX" },
	{ "EP5 TX Mixer", "USB_TX", "USB_TX" },
	{ "EP5 TX Mixer", "INCALL_TX", "INCALL_TX" },

	{ "EP6_TX", NULL, "EP6 TX Mixer" },
	{ "EP6 TX Mixer", "I2S_0_TX", "I2S_0_TX" },
	{ "EP6 TX Mixer", "I2S_1_TX", "I2S_1_TX" },
	{ "EP6 TX Mixer", "I2S_2_TX", "I2S_2_TX" },
	{ "EP6 TX Mixer", "TDM_0_TX", "TDM_0_TX" },
	{ "EP6 TX Mixer", "TDM_1_TX", "TDM_1_TX" },
	{ "EP6 TX Mixer", "INTERNAL_MIC_TX", "INTERNAL_MIC_TX" },
	{ "EP6 TX Mixer", "ERASER_TX", "ERASER_TX" },
	{ "EP6 TX Mixer", "BT_TX", "BT_TX" },
	{ "EP6 TX Mixer", "USB_TX", "USB_TX" },
	{ "EP6 TX Mixer", "INCALL_TX", "INCALL_TX" },

	{ "VOIP_TX", NULL, "VOIP TX Mixer" },
	{ "VOIP TX Mixer", "I2S_0_TX", "I2S_0_TX" },
	{ "VOIP TX Mixer", "I2S_1_TX", "I2S_1_TX" },
	{ "VOIP TX Mixer", "I2S_2_TX", "I2S_2_TX" },
	{ "VOIP TX Mixer", "TDM_0_TX", "TDM_0_TX" },
	{ "VOIP TX Mixer", "TDM_1_TX", "TDM_1_TX" },
	{ "VOIP TX Mixer", "INTERNAL_MIC_TX", "INTERNAL_MIC_TX" },
	{ "VOIP TX Mixer", "ERASER_TX", "ERASER_TX" },
	{ "VOIP TX Mixer", "BT_TX", "BT_TX" },
	{ "VOIP TX Mixer", "USB_TX", "USB_TX" },

	{ "NoHost1_TX", NULL, "NoHost1 TX Mixer" },
	{ "NoHost1 TX Mixer", "I2S_0_TX", "I2S_0_TX" },
	{ "NoHost1 TX Mixer", "I2S_1_TX", "I2S_1_TX" },
	{ "NoHost1 TX Mixer", "I2S_2_TX", "I2S_2_TX" },
	{ "NoHost1 TX Mixer", "TDM_0_TX", "TDM_0_TX" },
	{ "NoHost1 TX Mixer", "TDM_1_TX", "TDM_1_TX" },
	{ "NoHost1 TX Mixer", "INTERNAL_MIC_TX", "INTERNAL_MIC_TX" },
	{ "NoHost1 TX Mixer", "ERASER_TX", "ERASER_TX" },
	{ "NoHost1 TX Mixer", "BT_TX", "BT_TX" },
	{ "NoHost1 TX Mixer", "USB_TX", "USB_TX" },
	{ "NoHost1 TX Mixer", "INCALL_TX", "INCALL_TX" },

	{ "TDM_0_TX", NULL, "HW_SOURCE" },
	{ "TDM_1_TX", NULL, "HW_SOURCE" },
	{ "I2S_0_TX", NULL, "HW_SOURCE" },
	{ "I2S_1_TX", NULL, "HW_SOURCE" },
	{ "I2S_2_TX", NULL, "HW_SOURCE" },
	{ "INTERNAL_MIC_TX", NULL, "HW_SOURCE" },
	{ "INTERNAL_MIC_US_TX", NULL, "HW_SOURCE" },
	{ "ERASER_TX", NULL, "HW_SOURCE" },
	{ "BT_TX", NULL, "HW_SOURCE" },
	{ "USB_TX", NULL, "HW_SOURCE" },
	{ "INCALL_TX", NULL, "HW_SOURCE" },

	/* Link path to BE */
	/* Playback */
	{ "I2S_0_RX Playback", NULL, "I2S_0_RX" },
	{ "I2S_1_RX Playback", NULL, "I2S_1_RX" },
	{ "I2S_2_RX Playback", NULL, "I2S_2_RX" },
	{ "TDM_0_RX Playback", NULL, "TDM_0_RX" },
	{ "TDM_1_RX Playback", NULL, "TDM_1_RX" },
	{ "BT_RX Playback", NULL, "BT_RX" },
	{ "USB_RX Playback", NULL, "USB_RX" },
	{ "INCALL_RX Playback", NULL, "INCALL_RX" },
	{ "HAPTIC_RX Playback", NULL, "HAPTIC_RX" },
	{ "DP_DMA_RX Playback", NULL, "DP_DMA_RX" },

	/* Capture */
	{ "I2S_0_TX", NULL, "I2S_0_TX Capture" },
	{ "I2S_1_TX", NULL, "I2S_1_TX Capture" },
	{ "I2S_2_TX", NULL, "I2S_2_TX Capture" },
	{ "TDM_0_TX", NULL, "TDM_0_TX Capture" },
	{ "TDM_1_TX", NULL, "TDM_1_TX Capture" },
	{ "INTERNAL_MIC_TX", NULL, "INTERNAL_MIC_TX Capture" },
	{ "INTERNAL_MIC_US_TX", NULL, "INTERNAL_MIC_US_TX Capture" },
	{ "ERASER_TX", NULL, "ERASER_TX Capture" },
	{ "BT_TX", NULL, "BT_TX Capture" },
	{ "USB_TX", NULL, "USB_TX Capture" },
	{ "INCALL_TX", NULL, "INCALL_TX Capture" },
};

static int aoc_of_xlate_dai_name(struct snd_soc_component *component,
				 const struct of_phandle_args *args,
				 const char **dai_name)
{
	int i, ret = -EINVAL, head, next;
	uint32_t id;

	if (args->args_count != 1) {
		pr_err("%s: invalid arg count %d", __func__, args->args_count);
		return -EINVAL;
	}

	id = args->args[0];
	if (id & AOC_BE) {
		/* reverse scan */
		head = ARRAY_SIZE(aoc_dai_drv) - 1;
		next = -1;
	} else {
		head = 0;
		next = 1;
	}

	mutex_lock(&path_mutex);
	for (i = 0; i < ARRAY_SIZE(aoc_dai_drv); i++, head += next) {
		if (id == aoc_dai_drv[head].id) {
			*dai_name = aoc_dai_drv[head].name;
			ret = 0;
			pr_debug("%s: find dai %s for id 0x%x", __func__,
				*dai_name, args->args[0]);
			break;
		}
	}
	mutex_unlock(&path_mutex);

	if (ret)
		pr_err("fail to xlate 0x%x", args->args[0]);

	return ret;
}

static int aoc_cmp_probe(struct snd_soc_component *comp)
{
	struct snd_soc_dapm_context *dapm = snd_soc_component_get_dapm(comp);
	int ret;

	ret = snd_soc_dapm_new_controls(dapm, aoc_widget,
					ARRAY_SIZE(aoc_widget));
	if (ret < 0) {
		pr_err("%s: fail to reg new ctrls %d", __func__, ret);
		goto err;
	}

	ret = snd_soc_dapm_add_routes(dapm, aoc_routes, ARRAY_SIZE(aoc_routes));
	if (ret < 0) {
		pr_err("%s: fail to reg routes %d", __func__, ret);
		goto err;
	}

	ret = snd_soc_add_component_controls(comp, runtime_ctrls,
					     ARRAY_SIZE(runtime_ctrls));
	if (ret < 0) {
		pr_err("%s: fail to add ctrls %d", __func__, ret);
		goto err;
	}

	snd_soc_dapm_ignore_suspend(dapm, "HW_SINK");
	snd_soc_dapm_ignore_suspend(dapm, "HW_SOURCE");
err:
	return 0;
}

static const struct snd_soc_component_driver aoc_component = {
	.name = "AoC Path",
	.of_xlate_dai_name = aoc_of_xlate_dai_name,
	.probe = aoc_cmp_probe,
};

static int aoc_path_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	int ret = 0;

	pr_info("%s", __func__);
	if (!np)
		return -EINVAL;

	ret = devm_snd_soc_register_component(dev, &aoc_component, aoc_dai_drv,
					      ARRAY_SIZE(aoc_dai_drv));
	if (ret) {
		pr_err("%s: fail to reigster aoc path compon %d", __func__,
		       ret);
	}

	return ret;
}

static const struct of_device_id aoc_path_of_match[] = {
	{
		.compatible = "google-aoc-path",
	},
	{},
};
MODULE_DEVICE_TABLE(of, aoc_path_of_match);

static struct platform_driver aoc_path_drv = {
	.driver = {
		.name = "google-aoc-path",
		.of_match_table = aoc_path_of_match,
	},
	.probe = aoc_path_probe,
};

int aoc_path_init(void)
{
	int err;

	pr_info("%s", __func__);
	mutex_init(&path_mutex);
	err = platform_driver_register(&aoc_path_drv);
	if (err) {
		pr_err("Error registering aoc path %d .\n", err);
		return err;
	}

	return 0;
}

void aoc_path_exit(void)
{
	platform_driver_unregister(&aoc_path_drv);
	mutex_destroy(&path_mutex);
}
