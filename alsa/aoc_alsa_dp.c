// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google Whitechapel AoC ALSA Driver on PCM for dp_dma
 * Copyright (c) 2023 Google LLC
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/version.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/control.h>
#include <sound/soc.h>
#include "aoc_alsa.h"
#include "dp_audio.h"

static const struct snd_pcm_hardware snd_aoc_dp_hw = {
	.info = (SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER |
		 SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID),
	.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S32_LE,
	.rates = SNDRV_PCM_RATE_8000_192000,
	.rate_min = 8000,
	.rate_max = 192000,
	.channels_min = 2,
	.channels_max = 8,
	.buffer_bytes_max = SZ_1M,
	.period_bytes_min = 16,
	.period_bytes_max = 16384,
	.periods_min = 1,
	.periods_max = 64,
};

static int snd_aoc_dp_open(struct snd_soc_component *component,
	struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct aoc_chip *chip = snd_soc_card_get_drvdata(card);
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct device *dev = component->dev;

	dev_dbg(dev, "substream=%pK\n", substream);
	runtime->hw = snd_aoc_dp_hw;
	if (aoc_displayport_service_alloc(chip) < 0) {
		dev_err(dev, "fail to allocate audio_displayport service\n");
		return -EINVAL;
	}
	aoc_displayport_flush(chip);
	return 0;
}

static int snd_aoc_dp_close(struct snd_soc_component *component,
	struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct aoc_chip *chip = snd_soc_card_get_drvdata(card);
	struct device *dev = component->dev;

	dev_dbg(dev, "substream=%pK\n", substream);
	aoc_displayport_service_free(chip);
	return 0;
}

static int snd_aoc_dp_fill_buffer(struct snd_pcm_substream *substream,
	unsigned int offset, unsigned int bytes)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct aoc_chip *chip = snd_soc_card_get_drvdata(card);
	struct snd_pcm_runtime *runtime = substream->runtime;
	char *dst = runtime->dma_area + offset;
	struct device *dev = card->dev;

	dev_dbg(dev, "dst = 0x%pK =(0x%pK + %#x) size = %u\n",
		dst, runtime->dma_area, offset, bytes);

	return aoc_displayport_read(chip, dst, bytes);
}

static int snd_aoc_dp_hw_params(struct snd_soc_component *component,
	struct snd_pcm_substream *substream, struct snd_pcm_hw_params *params)
{
	struct device *dev = component->dev;
	dev_dbg(dev, "substream=%pK\n", substream);
	dp_dma_register_fill_buffer_cb(substream, snd_aoc_dp_fill_buffer);
	return 0;
}

/* PCM hw_free callback */
static int snd_aoc_dp_hw_free(struct snd_soc_component *component,
	struct snd_pcm_substream *substream)
{
	struct device *dev = component->dev;
	dev_dbg(dev, "substream=%pK\n", substream);
	dp_dma_register_fill_buffer_cb(substream, NULL);
	return 0;
}

/* Trigger callback */
static int snd_aoc_dp_trigger(struct snd_soc_component *component,
	struct snd_pcm_substream *substream, int cmd)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct device *dev = component->dev;
	dev_dbg(dev, "(%pK) private_data  = %pK\n", substream, runtime->private_data);

	return 0;
}

/* Pointer callback */
static snd_pcm_uframes_t snd_aoc_dp_pointer(struct snd_soc_component *component,
	struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct device *dev = component->dev;
	dev_dbg(dev, "%s:(%pK) private_data  = %pK\n", __func__,
		substream, runtime->private_data);
	return 0;
}

/* PCM prepare callback */
static int snd_aoc_dp_prepare(struct snd_soc_component *component,
	struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct device *dev = component->dev;
	dev_dbg(dev, "dma_addr=%llu dma_bytes=%x dma_area=0x%pK\n",
		runtime->dma_addr, (int)runtime->dma_bytes, runtime->dma_area);

	/*
	 * Adjust the stop_threshold and boundary to pass
	 * the trigger_start test condition
	 */
	runtime->stop_threshold = runtime->boundary = runtime->buffer_size;
	return 0;
}

static int aoc_dp_new(struct snd_soc_component *component,
	struct snd_soc_pcm_runtime *rtd)
{
	return 0;
}

static const struct snd_soc_component_driver aoc_dp_component = {
	.name = "AoC DP",
	.open = snd_aoc_dp_open,
	.close = snd_aoc_dp_close,
	.hw_params = snd_aoc_dp_hw_params,
	.hw_free = snd_aoc_dp_hw_free,
	.trigger = snd_aoc_dp_trigger,
	.pointer = snd_aoc_dp_pointer,
	.prepare = snd_aoc_dp_prepare,
	.pcm_construct = aoc_dp_new,
};

static int aoc_dp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	int err = 0;

	if (!np)
		return -EINVAL;

	err = devm_snd_soc_register_component(dev, &aoc_dp_component, NULL, 0);
	if (err)
		dev_err(dev, "fail to register aoc pcm comp %d", err);

	return err;
}

static const struct of_device_id aoc_dp_of_match[] = {
	{
		.compatible = "google-aoc-snd-dp",
	},
	{},
};
MODULE_DEVICE_TABLE(of, aoc_dp_of_match);

static struct platform_driver aoc_dp_drv = {
	.driver =
	{
		.name = "google-aoc-snd-dp",
		.of_match_table = aoc_dp_of_match,
	},
	.probe = aoc_dp_probe,
};

int aoc_dp_init(void)
{
	int err;

	pr_debug("%s", __func__);
	err = platform_driver_register(&aoc_dp_drv);
	if (err) {
		pr_err("error registering aoc dp drv %d\n", err);
		return err;
	}
	return 0;
}

void aoc_dp_exit(void)
{
	platform_driver_unregister(&aoc_dp_drv);
}
