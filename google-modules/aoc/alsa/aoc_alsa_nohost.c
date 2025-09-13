// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google Whitechapel AoC ALSA Driver on PCM
 * Copyright (c) 2020 Google LLC
 */
#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/version.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/control.h>
#include <sound/soc.h>
#include "aoc_alsa.h"
static struct snd_pcm_hardware snd_aoc_nohost_hw = {
	.info = (SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER |
		 SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID),
	.formats = SNDRV_PCM_FMTBIT_S8 | SNDRV_PCM_FMTBIT_U8 |
		   SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_3LE |
		   SNDRV_PCM_FMTBIT_S32_LE | SNDRV_PCM_FMTBIT_FLOAT_LE,
	.rates = SNDRV_PCM_RATE_CONTINUOUS | SNDRV_PCM_RATE_8000_48000,
	.rate_min = 8000,
	.rate_max = 48000,
	.channels_min = 1,
	.channels_max = 2,
	.buffer_bytes_max = PAGE_SIZE,
	.period_bytes_min = 16,
	.period_bytes_max = 2048,
	.periods_min = 2,
	.periods_max = 4,
};

static int snd_aoc_nohost_open(struct snd_soc_component *component,
	struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;

	runtime->hw = snd_aoc_nohost_hw;
	return 0;
}

static int snd_aoc_nohost_close(struct snd_soc_component *component,
	struct snd_pcm_substream *substream)
{
	return 0;
}

static int snd_aoc_nohost_hw_params(struct snd_soc_component *component,
	struct snd_pcm_substream *substream, struct snd_pcm_hw_params *params)
{
	int err;

	/*
	 * The ap side won't write/read any data on no host mode.
	 * Always allocate one page in case ALSA core driver will
	 * access the dma of substream.
	 */
	err = snd_pcm_lib_malloc_pages(substream, PAGE_SIZE);
	if (err < 0)
		pr_err("pcm_lib_malloc failed to allocated pages for buffers\n");

	return err;
}

/* PCM hw_free callback */
static int snd_aoc_nohost_hw_free(struct snd_soc_component *component,
	struct snd_pcm_substream *substream)
{
	return snd_pcm_lib_free_pages(substream);
}

/* PCM prepare callback */
static int snd_aoc_nohost_prepare(struct snd_soc_component *component,
	struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;

	/*
	 * Adjust the stop_threshold and boundary to pass
	 * the trigger_start test condition
	 */
	runtime->stop_threshold = runtime->boundary = runtime->buffer_size;
	return 0;
}

static int aoc_nohost_new(struct snd_soc_component *component,
	struct snd_soc_pcm_runtime *rtd)
{
	struct snd_pcm_substream *substream = NULL;

	dma_set_mask_and_coherent(component->dev, DMA_BIT_MASK(64));

	/* Allocate DMA memory */
	if (rtd->dai_link->dpcm_playback) {
		substream =
			rtd->pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream;
		snd_pcm_lib_preallocate_pages(
			substream, SNDRV_DMA_TYPE_CONTINUOUS,
			component->dev,
			snd_aoc_nohost_hw.buffer_bytes_max,
			snd_aoc_nohost_hw.buffer_bytes_max);
	}
	if (rtd->dai_link->dpcm_capture) {
		substream =
			rtd->pcm->streams[SNDRV_PCM_STREAM_CAPTURE].substream;
		snd_pcm_lib_preallocate_pages(
			substream, SNDRV_DMA_TYPE_CONTINUOUS,
			component->dev,
			snd_aoc_nohost_hw.buffer_bytes_max,
			snd_aoc_nohost_hw.buffer_bytes_max);
	}
	rtd->pcm->nonatomic = true;
	return 0;
}

static const struct snd_soc_component_driver aoc_nohost_component = {
	.name = "AoC NoHost",
	.open = snd_aoc_nohost_open,
	.close = snd_aoc_nohost_close,
	.hw_params = snd_aoc_nohost_hw_params,
	.hw_free = snd_aoc_nohost_hw_free,
	.prepare = snd_aoc_nohost_prepare,
	.pcm_construct = aoc_nohost_new,
};

static int aoc_nohost_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	int err = 0;

	pr_debug("%s", __func__);
	if (!np)
		return -EINVAL;

	err = devm_snd_soc_register_component(dev, &aoc_nohost_component, NULL, 0);
	if (err)
		pr_err("%s: fail to register aoc pcm comp %d", __func__, err);

	return err;
}

static const struct of_device_id aoc_nohost_of_match[] = {
	{
		.compatible = "google-aoc-snd-nohost",
	},
	{},
};
MODULE_DEVICE_TABLE(of, aoc_nohost_of_match);

static struct platform_driver aoc_nohost_drv = {
	.driver =
	{
		.name = "google-aoc-snd-nohost",
		.of_match_table = aoc_nohost_of_match,
	},
	.probe = aoc_nohost_probe,
};

int aoc_nohost_init(void)
{
	int err;

	pr_debug("%s", __func__);
	err = platform_driver_register(&aoc_nohost_drv);
	if (err) {
		pr_err("error registering aoc nohost drv %d\n", err);
		return err;
	}
	return 0;
}

void aoc_nohost_exit(void)
{
	platform_driver_unregister(&aoc_nohost_drv);
}
