// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google Whitechapel AoC ALSA  Driver on PCM
 *
 * Copyright (c) 2019 Google LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/version.h>

#include "aoc_alsa.h"
#include "aoc_alsa_drv.h"

/* Timer interrupt to read the ring buffer reader/writer positions */

/* Hardware definition
 * TODO: different pcm may have different hardware setup,
 * considering deep buffer and compressed offload buffer
 */
static struct snd_pcm_hardware snd_aoc_playback_hw = {
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
	.buffer_bytes_max = 15360,
	.period_bytes_min = 16,
	.period_bytes_max = 7680,
	.periods_min = 2,
	.periods_max = 4,
};

static enum hrtimer_restart aoc_pcm_hrtimer_irq_handler(struct hrtimer *timer)
{
	return HRTIMER_RESTART;
}

/* Timer interrupt handler to update the ring buffer reader/writer positions
 * during playback/capturing
 */
static void aoc_pcm_timer_irq_handler(struct timer_list *timer)
{
	return ;
}

static void snd_aoc_pcm_free(struct snd_pcm_runtime *runtime)
{
	pr_debug("Freeing up alsa stream here ..\n");
	pr_debug("%s:", __func__);

	kfree(runtime->private_data);
	runtime->private_data = NULL;
}

/* PCM open callback */
static int snd_aoc_pcm_open(struct snd_soc_component *component,
			    struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd =
		(struct snd_soc_pcm_runtime *)substream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct aoc_chip *chip =
		(struct aoc_chip *)snd_soc_card_get_drvdata(card);
	struct snd_pcm_runtime *runtime = substream->runtime;

	struct aoc_alsa_stream *alsa_stream = NULL;
	int idx;
	int err;

	pr_debug("stream (%d)\n", substream->number); /* Playback or capture */
	if (mutex_lock_interruptible(&chip->audio_mutex)) {
		pr_err("ERR: interrupted whilst waiting for lock\n");
		return -EINTR;
	}

	idx = substream->pcm->device;
	pr_debug("pcm device open (%d)\n", idx);
	pr_debug("chip open (%llu)\n", chip->opened);

	alsa_stream = kzalloc(sizeof(struct aoc_alsa_stream), GFP_KERNEL);
	if (alsa_stream == NULL) {
		err = -ENOMEM;
		pr_err("ERR: no memory for %s", rtd->dai_link->name);
		goto out;
	}

	/* Initialise alsa_stream */
	alsa_stream->chip = chip;
	alsa_stream->substream = substream;
	alsa_stream->cstream = NULL;
	alsa_stream->dev = NULL;
	alsa_stream->idx = idx;

	/* Ring buffer will be flushed at prepare() before playback/capture */
	alsa_stream->hw_ptr_base = 0;
	alsa_stream->prev_consumed = alsa_stream->hw_ptr_base;
	alsa_stream->n_overflow = 0;

	err = aoc_audio_open(alsa_stream);
	if (err != 0) {
		pr_err("ERR: fail to audio open  %s", rtd->dai_link->name);
		goto out;
	}
	runtime->private_data = alsa_stream;
	runtime->private_free = snd_aoc_pcm_free;
	runtime->hw = snd_aoc_playback_hw;
	chip->alsa_stream[idx] = alsa_stream;
	chip->opened |= (1 << idx);
	alsa_stream->open = 1;
	alsa_stream->draining = 1;

	alsa_stream->timer_interval_ns = PCM_TIMER_INTERVAL_NANOSECS;
	timer_setup(&(alsa_stream->timer), aoc_pcm_timer_irq_handler, 0);
	hrtimer_init( &(alsa_stream->hr_timer), CLOCK_MONOTONIC, HRTIMER_MODE_REL );
	alsa_stream->hr_timer.function = &aoc_pcm_hrtimer_irq_handler;

	chip->default_mic_hw_gain =
		aoc_mic_hw_gain_get(chip, MIC_HIGH_POWER_GAIN);

	aoc_voice_call_mic_mute(chip, chip->voice_call_mic_mute);

	alsa_stream->entry_point_idx = substream->pcm->device;

	mutex_unlock(&chip->audio_mutex);

	return 0;
out:
	kfree(alsa_stream);
	mutex_unlock(&chip->audio_mutex);

	pr_debug("pcm open err=%d\n", err);
	return err;
}


/* Close callback */
static int snd_aoc_pcm_close(struct snd_soc_component *component,
			     struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct aoc_alsa_stream *alsa_stream = runtime->private_data;
	struct aoc_chip *chip = alsa_stream->chip;
	int err;

	pr_debug("%s: name %s substream %p", __func__, rtd->dai_link->name,
		 substream);
	aoc_timer_stop_sync(alsa_stream);

	if (mutex_lock_interruptible(&chip->audio_mutex)) {
		pr_err("ERR: interrupted while waiting for lock\n");
		return -EINTR;
	}

	/* Stop phone call (Refactor needed) */
	pr_notice("Stop voice call\n");
	err = teardown_phonecall(alsa_stream);
	if (err < 0)
		pr_err("ERR: fail in phone call tearing down\n");

	aoc_mic_hw_gain_set(chip, MIC_HIGH_POWER_GAIN,
			    chip->default_mic_hw_gain);

	runtime = substream->runtime;
	alsa_stream = runtime->private_data;

	pr_debug("alsa pcm close\n");
	/*
	 * Call stop if it's still running. This happens when app
	 * is force killed and we don't get a stop trigger.
	 */
	if (alsa_stream->running) {
		err = aoc_audio_stop(alsa_stream);
		alsa_stream->running = 0;
		if (err != 0)
			pr_err("ERR: fail to stop alsa stream\n");
	}

	alsa_stream->period_size = 0;
	alsa_stream->buffer_size = 0;

	if (alsa_stream->open) {
		alsa_stream->open = 0;
		aoc_audio_close(alsa_stream);
	}
	if (alsa_stream->chip)
		alsa_stream->chip->alsa_stream[alsa_stream->idx] = NULL;
	/*
	 * Do not free up alsa_stream here, it will be freed up by
	 * runtime->private_free callback we registered in *_open above
	 */

	chip->opened &= ~(1 << alsa_stream->idx);

	mutex_unlock(&chip->audio_mutex);

	return 0;
}

/* PCM hw_params callback */
static int snd_aoc_pcm_hw_params(struct snd_soc_component *component,
				 struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct aoc_alsa_stream *alsa_stream = runtime->private_data;
	int err;

	err = snd_pcm_lib_malloc_pages(substream, params_buffer_bytes(params));
	if (err < 0) {
		pr_err("ERR: %d fail in pcm buffer allocation\n", err);
		return err;
	}

	alsa_stream->channels = params_channels(params);
	alsa_stream->params_rate = params_rate(params);
	alsa_stream->pcm_format_width =
		snd_pcm_format_width(params_format(params));

	alsa_stream->pcm_float_fmt =
		(params_format(params) == SNDRV_PCM_FORMAT_FLOAT_LE);

	pr_debug("alsa_stream->pcm_format_width = %d\n",
		 alsa_stream->pcm_format_width);
	return err;
}

/* PCM hw_free callback */
static int snd_aoc_pcm_hw_free(struct snd_soc_component *component,
			       struct snd_pcm_substream *substream)
{
	return snd_pcm_lib_free_pages(substream);
}

/* PCM prepare callback */
static int snd_aoc_pcm_prepare(struct snd_soc_component *component,
			       struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct aoc_alsa_stream *alsa_stream = runtime->private_data;
	struct aoc_chip *chip = alsa_stream->chip;
	int channels, err;

	aoc_timer_stop_sync(alsa_stream);

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	channels = alsa_stream->channels;

	pr_debug("channels = %d, rate = %d, bits = %d, float-fmt = %d\n",
		 channels, alsa_stream->params_rate,
		 alsa_stream->pcm_format_width, alsa_stream->pcm_float_fmt);

	aoc_audio_setup(alsa_stream);

	/* in preparation of the stream */
	/* aoc_audio_set_ctls(alsa_stream->chip); */
	alsa_stream->buffer_size = snd_pcm_lib_buffer_bytes(substream);
	alsa_stream->period_size = snd_pcm_lib_period_bytes(substream);
	alsa_stream->pos = 0;
	alsa_stream->hw_ptr_base = 0;
	alsa_stream->prev_consumed = alsa_stream->hw_ptr_base;
	alsa_stream->n_overflow = 0;

	pr_debug("buffer_size=%d, period_size=%d pos=%d frame_bits=%d\n",
		 alsa_stream->buffer_size, alsa_stream->period_size,
		 alsa_stream->pos, runtime->frame_bits);

	/* Prepare phone call (Refactor needed) */
	pr_notice("Start voice call\n");
	err = prepare_phonecall(alsa_stream);
	if (err < 0)
		pr_err("ERR:%d in preparing phonecall\n", err);

	mutex_unlock(&chip->audio_mutex);

	runtime->stop_threshold = runtime->boundary = runtime->buffer_size;

	return err;
}

static int aoc_pcm_new(struct snd_soc_component *component, struct snd_soc_pcm_runtime *rtd)
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
			snd_aoc_playback_hw.buffer_bytes_max,
			snd_aoc_playback_hw.buffer_bytes_max);
	}

	if (rtd->dai_link->dpcm_capture) {
		substream =
			rtd->pcm->streams[SNDRV_PCM_STREAM_CAPTURE].substream;
		snd_pcm_lib_preallocate_pages(
			substream, SNDRV_DMA_TYPE_CONTINUOUS,
			component->dev,
			snd_aoc_playback_hw.buffer_bytes_max,
			snd_aoc_playback_hw.buffer_bytes_max);
	}

	rtd->pcm->nonatomic = true;
	return 0;
}

static const struct snd_soc_component_driver aoc_pcm_component = {
	.name = "AoC VOICE",
	.open = snd_aoc_pcm_open,
	.close = snd_aoc_pcm_close,
	.hw_params = snd_aoc_pcm_hw_params,
	.hw_free = snd_aoc_pcm_hw_free,
	.prepare = snd_aoc_pcm_prepare,
	.pcm_construct = aoc_pcm_new,
};

static int aoc_pcm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	int err = 0;

	pr_debug("%s", __func__);
	if (!np)
		return -EINVAL;

	err = devm_snd_soc_register_component(dev, &aoc_pcm_component, NULL, 0);
	if (err)
		pr_err("ERR: %d fail to reigster aoc pcm comp\n", err);

	return err;
}

static const struct of_device_id aoc_voice_of_match[] = {
	{
		.compatible = "google-aoc-snd-voice",
	},
	{},
};
MODULE_DEVICE_TABLE(of, aoc_voice_of_match);

static struct platform_driver aoc_pcm_drv = {
    .driver =
        {
            .name = "google-aoc-snd-voice",
            .of_match_table = aoc_voice_of_match,
        },
    .probe = aoc_pcm_probe,
};

int aoc_voice_init(void)
{
	int err;

	pr_debug("%s", __func__);
	err = platform_driver_register(&aoc_pcm_drv);
	if (err) {
		pr_err("ERR: %d in registering aoc voice drv\n", err);
		return err;
	}

	return 0;
}

void aoc_voice_exit(void)
{
	platform_driver_unregister(&aoc_pcm_drv);
}
