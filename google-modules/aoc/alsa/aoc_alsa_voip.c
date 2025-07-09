// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google Whitechapel AoC ALSA Driver on PCM
 *
 * Copyright (c) 2019 Google LLC
 *
 */

#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/version.h>

#include "aoc_alsa.h"
#include "aoc_alsa_drv.h"

#include <linux/hrtimer.h>
#include <linux/ktime.h>

/* Hardware definition
 * TODO: different pcm may have different hardware setup,
 * considering deep buffer and compressed offload buffer
 */
static struct snd_pcm_hardware snd_aoc_playback_hw = {
	.info = (SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER | SNDRV_PCM_INFO_MMAP |
		 SNDRV_PCM_INFO_MMAP_VALID),
	.formats = SNDRV_PCM_FMTBIT_S8 | SNDRV_PCM_FMTBIT_U8 | SNDRV_PCM_FMTBIT_S16_LE |
		   SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE | SNDRV_PCM_FMTBIT_FLOAT_LE,
	.rates = SNDRV_PCM_RATE_CONTINUOUS | SNDRV_PCM_RATE_8000_48000,
	.rate_min = 8000,
	.rate_max = 48000,
	.channels_min = 1,
	.channels_max = 4,
	.buffer_bytes_max = 16384,
	.period_bytes_min = 16,
	.period_bytes_max = 7680,
	.periods_min = 2,
	.periods_max = 128,
};

static bool aoc_voip_support_interrupt(uint8_t mbox_index)
{
	return (mbox_index == VOIP_CHANNEL);
}

static enum hrtimer_restart aoc_voip_irq_process(struct aoc_alsa_stream *alsa_stream)
{
	struct aoc_service_dev *dev;
	unsigned long consumed; /* TODO: uint64_t? */

	/* The number of bytes read/writtien should be the bytes in the buffer
	 * already played out in the case of playback. But this may not be true
	 * in the AoC ring buffer implementation, since the reader pointer in
	 * the playback case represents what has been read from the buffer,
	 * not what already played out .
	*/
	if (alsa_stream->dev == NULL ||
		 alsa_stream->substream->runtime->status->state != SNDRV_PCM_STATE_RUNNING)
		return HRTIMER_RESTART;

	dev = alsa_stream->dev;
	consumed = ((alsa_stream->substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ?
				  aoc_ring_bytes_read(dev->service, AOC_DOWN) :
				  aoc_ring_bytes_written(dev->service, AOC_UP));

	dev_dbg(&(dev->dev), "consumed = %ld , hw_ptr_base =%ld\n", consumed,
		alsa_stream->hw_ptr_base);

	/* TODO: To do more on no pointer update? */
	if (consumed == alsa_stream->prev_consumed)
		return HRTIMER_RESTART;

	/* To deal with overlfow in Tx or Rx in int32_t */
	if (consumed < alsa_stream->prev_consumed) {
		alsa_stream->n_overflow++;
		dev_notice(&(dev->dev), "overflow in Tx/Rx: %ld - %ld - %d times\n", consumed,
			   alsa_stream->prev_consumed, alsa_stream->n_overflow);
	}
	alsa_stream->prev_consumed = consumed;

	if (!aoc_pcm_update_pos(alsa_stream, consumed))
		return HRTIMER_RESTART;

	alsa_stream->prev_pos = alsa_stream->pos;

	/* Do not queue a work if the cancel_work is active */
	if (atomic_read(&alsa_stream->cancel_work_active) > 0
			|| alsa_stream->voip_period_wq == NULL)
		return HRTIMER_RESTART;

	if (!queue_work(alsa_stream->voip_period_wq, &alsa_stream->pcm_period_work)) {
		wake_up(&alsa_stream->substream->runtime->sleep);
		wake_up(&alsa_stream->substream->runtime->tsleep);
		alsa_stream->wq_busy_count++;

		if (!(alsa_stream->wq_busy_count % 5))
			pr_warn("voip period work busy count = %d\n", alsa_stream->wq_busy_count);
	} else
		alsa_stream->wq_busy_count = 0;

	return HRTIMER_RESTART;
}

static enum hrtimer_restart aoc_voip_hrtimer_irq_handler(struct hrtimer *timer)
{
	struct aoc_alsa_stream *alsa_stream;

	WARN_ON(!timer);
	alsa_stream = container_of(timer, struct aoc_alsa_stream, hr_timer);

	WARN_ON(!alsa_stream || !alsa_stream->substream);

	/* Start the timer immediately for next period */
	/* aoc_timer_start(alsa_stream); */
	aoc_timer_restart(alsa_stream);

	return aoc_voip_irq_process(alsa_stream);
}

void aoc_voip_isr(struct aoc_service_dev *dev)
{
	struct aoc_alsa_stream *alsa_stream;

	if (!dev) {
		pr_err("ERR: NULL aoc service pointer\n");
		return;
	}

	alsa_stream = dev->prvdata;

	if (alsa_stream == NULL)
		return;

	if (alsa_stream->substream == NULL) {
		pr_err("ERR: NULL alsa_stream->substream pointer\n");
		return;
	}

	aoc_voip_irq_process(alsa_stream);
}

static void snd_aoc_pcm_free(struct snd_pcm_runtime *runtime)
{
	pr_debug("Freeing up alsa stream here ..\n");

	kfree(runtime->private_data);
	runtime->private_data = NULL;
}

/* PCM open callback */
static int snd_aoc_pcm_open(struct snd_soc_component *component,
			    struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct aoc_chip *chip = snd_soc_card_get_drvdata(card);
	struct snd_pcm_runtime *runtime = substream->runtime;

	struct aoc_alsa_stream *alsa_stream = NULL;
	struct aoc_service_dev *dev = NULL;
	int idx;
	int err;

	dev_dbg(component->dev, "stream (%d)\n", substream->number); /* Playback or capture */
	if (mutex_lock_interruptible(&chip->audio_mutex)) {
		dev_err(component->dev, "ERR: interrupted whilst waiting for lock\n");
		return -EINTR;
	}

	idx = substream->pcm->device;
	dev_dbg(component->dev, "pcm device open (%d)\n", idx);
	dev_dbg(component->dev, "chip open (%llu)\n", chip->opened);

	/* Find the corresponding aoc audio service */
	err = alloc_aoc_audio_service(rtd->dai_link->name, &dev, NULL, NULL);
	if (err < 0) {
		dev_err(component->dev, "ERR:%d fail to alloc service for %s", err,
			rtd->dai_link->name);
		goto out;
	}

	alsa_stream = kzalloc(sizeof(struct aoc_alsa_stream), GFP_KERNEL);
	if (alsa_stream == NULL) {
		err = -ENOMEM;
		dev_err(component->dev, "ERR: fail to alloc alsa_stream for %s",
			rtd->dai_link->name);
		goto out;
	}

	/* Initialise alsa_stream */
	alsa_stream->chip = chip;
	alsa_stream->substream = substream;
	alsa_stream->cstream = NULL;
	alsa_stream->dev = dev;
	alsa_stream->idx = idx;
	alsa_stream->wq_busy_count = 0;
	atomic_set(&alsa_stream->cancel_work_active, 0);

	INIT_WORK(&alsa_stream->pcm_period_work, aoc_pcm_period_work_handler);
	alsa_stream->voip_period_wq = alloc_ordered_workqueue("alsa_voip_period_work", WQ_HIGHPRI);
	if (!alsa_stream->voip_period_wq) {
		err = -ENOMEM;
		pr_err("ERR: fail to alloc workqueue for %s", rtd->dai_link->name);
		goto out;
	}

	/* Ring buffer will be flushed at prepare() before playback/capture */
	alsa_stream->hw_ptr_base = (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ?
						 aoc_ring_bytes_read(dev->service, AOC_DOWN) :
						 aoc_ring_bytes_written(dev->service, AOC_UP);
	alsa_stream->prev_consumed = alsa_stream->hw_ptr_base;
	alsa_stream->n_overflow = 0;
	alsa_stream->prev_buffer_cnt = 0;

	err = aoc_audio_open(alsa_stream);
	if (err != 0) {
		pr_err("ERR: fail to audio open for %s", rtd->dai_link->name);
		goto out;
	}
	runtime->private_data = alsa_stream;
	runtime->private_free = snd_aoc_pcm_free;
	runtime->hw = snd_aoc_playback_hw;
	chip->alsa_stream[idx] = alsa_stream;
	chip->opened |= (1 << idx);
	alsa_stream->open = 1;
	alsa_stream->draining = 1;

	if (aoc_voip_support_interrupt(alsa_stream->dev->mbox_index)) {
		dev->prvdata = alsa_stream;
		alsa_stream->isr_type = INTR;
	} else {
		alsa_stream->timer_interval_ns = PCM_TIMER_INTERVAL_NANOSECS;
		hrtimer_init(&(alsa_stream->hr_timer), CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		alsa_stream->hr_timer.function = &aoc_voip_hrtimer_irq_handler;
		alsa_stream->isr_type = TIMER;
	}

	/* TODO: refactor needed on mapping between device number and entrypoint */
	alsa_stream->entry_point_idx = (idx == 7) ? HAPTICS : idx;
	mutex_unlock(&chip->audio_mutex);

	return 0;
out:
	if (alsa_stream) {
		if (alsa_stream->voip_period_wq) {
			flush_workqueue(alsa_stream->voip_period_wq);
			destroy_workqueue(alsa_stream->voip_period_wq);
			alsa_stream->voip_period_wq = NULL;
		}
		kfree(alsa_stream);
	}

	if (dev) {
		free_aoc_audio_service(rtd->dai_link->name, dev);
		dev = NULL;
	}
	mutex_unlock(&chip->audio_mutex);

	dev_dbg(component->dev, "pcm open err=%d\n", err);
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

	dev_dbg(component->dev, "name %s substream %pK", rtd->dai_link->name, substream);
	aoc_timer_stop_sync(alsa_stream);
	atomic_set(&alsa_stream->cancel_work_active, 1);
	audio_free_isr(alsa_stream->dev);
	if (alsa_stream->voip_period_wq) {
		flush_workqueue(alsa_stream->voip_period_wq);
		destroy_workqueue(alsa_stream->voip_period_wq);
		alsa_stream->voip_period_wq = NULL;
	}
	atomic_set(&alsa_stream->cancel_work_active, 0);

	if (mutex_lock_interruptible(&chip->audio_mutex)) {
		dev_err(component->dev, "ERR: interrupted while waiting for lock\n");
		return -EINTR;
	}

	/* Stop voip call (Refactor needed) */
	pr_notice("Stop voip call\n");
	err = teardown_voipcall(alsa_stream);
	if (err < 0)
		pr_err("ERR: fail in voip call tearing down\n");

	runtime = substream->runtime;
	alsa_stream = runtime->private_data;

	dev_dbg(component->dev, "alsa pcm close\n");
	free_aoc_audio_service(rtd->dai_link->name, alsa_stream->dev);
	/*
	* Call stop if it's still running. This happens when app
	* is force killed and we don't get a stop trigger.
	*/
	if (alsa_stream->running) {
		err = aoc_audio_voip_stop(alsa_stream);
		alsa_stream->running = 0;
		if (err != 0)
			dev_err(component->dev, "ERR: fail to stop alsa stream\n");
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
	struct aoc_chip *chip = alsa_stream->chip;
	int err;

	err = snd_pcm_lib_malloc_pages(substream, params_buffer_bytes(params));
	if (err < 0) {
		dev_err(component->dev, "ERR:%d fail in pcm buffer allocation\n", err);
		return err;
	}

	substream->wait_time = msecs_to_jiffies(chip->voice_pcm_wait_time_in_ms);

	alsa_stream->channels = params_channels(params);
	alsa_stream->params_rate = params_rate(params);
	alsa_stream->pcm_format_width = snd_pcm_format_width(params_format(params));

	alsa_stream->pcm_float_fmt = (params_format(params) == SNDRV_PCM_FORMAT_FLOAT_LE);

	dev_dbg(component->dev, "alsa_stream->pcm_format_width = %d\n",
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
	struct aoc_service_dev *dev = alsa_stream->dev;
	struct aoc_chip *chip = alsa_stream->chip;
	int channels, err;

	aoc_timer_stop_sync(alsa_stream);

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;

	alsa_stream->buffer_size = snd_pcm_lib_buffer_bytes(substream);
	alsa_stream->period_size = snd_pcm_lib_period_bytes(substream);

#if !IS_ENABLED(CONFIG_SOC_GS101)
	/* Set the audio formats and flush the DRAM buffer */
	err = aoc_voip_set_params(alsa_stream);
	if (err < 0)
		pr_notice("Failed to set %d VoIP Rx params\n", err);
#endif

	channels = alsa_stream->channels;

	pr_debug("channels = %d, rate = %d, bits = %d, float-fmt = %d\n",
		 channels, alsa_stream->params_rate,
		 alsa_stream->pcm_format_width, alsa_stream->pcm_float_fmt);

	aoc_audio_setup(alsa_stream);

	pr_debug("Flush aoc ring buffer\n");
	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		if (!aoc_ring_flush_read_data(alsa_stream->dev->service, AOC_UP, 0)) {
			pr_err("ERR: capture ring buffer flush fail\n");
			err = -EINVAL;
		}
	}

	/* in preparation of the stream */
	/* aoc_audio_set_ctls(alsa_stream->chip); */
	alsa_stream->buffer_size = snd_pcm_lib_buffer_bytes(substream);
	alsa_stream->period_size = snd_pcm_lib_period_bytes(substream);
	alsa_stream->pos = 0;
	alsa_stream->prev_pos = 0;
	alsa_stream->pos_delta = 0;

	alsa_stream->hw_ptr_base = (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ?
						 aoc_ring_bytes_written(dev->service, AOC_DOWN) :
						 aoc_ring_bytes_written(dev->service, AOC_UP);
	alsa_stream->prev_consumed = alsa_stream->hw_ptr_base;
	alsa_stream->n_overflow = 0;
	alsa_stream->prev_buffer_cnt = 0;

	pr_notice("Start voip call\n");
	err = prepare_voipcall(alsa_stream);
	if (err < 0)
		dev_err(component->dev, "ERR:%d in preparing voip call\n", err);

	dev_dbg(component->dev, "pcm prepare: hw_ptr_base = %lu\n", alsa_stream->hw_ptr_base);

	dev_dbg(component->dev, "buffer_size=%d, period_size=%d pos=%d frame_bits=%d\n",
		alsa_stream->buffer_size, alsa_stream->period_size, alsa_stream->pos,
		runtime->frame_bits);

	mutex_unlock(&chip->audio_mutex);
	return err;
}

/* Trigger callback */
static int snd_aoc_pcm_trigger(struct snd_soc_component *component,
			       struct snd_pcm_substream *substream, int cmd)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct aoc_alsa_stream *alsa_stream = runtime->private_data;
	int err = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		dev_dbg(component->dev, "aoc_AUDIO_TRIGGER_START running=%d\n",
			alsa_stream->running);
		if (!alsa_stream->running) {
			/* start timer first to avoid underrun/overrun */
			aoc_timer_start(alsa_stream);

			err = aoc_audio_voip_start(alsa_stream);
			if (err == 0) {
				alsa_stream->running = 1;
				alsa_stream->draining = 1;
			} else {
				dev_err(component->dev, "ERR:%d fail to START stream\n", err);
			}
		}
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		dev_dbg(component->dev, "aoc_AUDIO_TRIGGER_STOP running=%d draining=%d\n",
			alsa_stream->running, runtime->status->state == SNDRV_PCM_STATE_DRAINING);

		if (runtime->status->state == SNDRV_PCM_STATE_DRAINING) {
			dev_dbg(component->dev, "DRAINING\n");
			alsa_stream->draining = 1;
		} else {
			dev_dbg(component->dev, "DROPPING\n");
			alsa_stream->draining = 0;
		}
		if (alsa_stream->running) {
			err = aoc_audio_voip_stop(alsa_stream);
			if (err != 0)
				dev_err(component->dev, "ERR:%d fail to STOP stream\n", err);
			alsa_stream->running = 0;
		}
		break;
	default:
		err = -EINVAL;
	}

	return err;
}

/* Copy data from user space to hardware buffer  */
static int snd_aoc_pcm_playback_copy_user(struct snd_soc_component *component,
					  struct snd_pcm_substream *substream, int channel,
					  unsigned long pos, void __user *buf, unsigned long count)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct aoc_alsa_stream *alsa_stream = runtime->private_data;
	int err = 0;

	err = aoc_audio_write(alsa_stream, buf, count);
	if (err)
		dev_err(component->dev, "ERR:%d fail to send audio to aoc\n", err);

	return err;
}

/* Copy data from hardware buffer to user space */
static int snd_aoc_pcm_capture_copy_user(struct snd_soc_component *component,
					 struct snd_pcm_substream *substream, int channel,
					 unsigned long pos, void __user *buf, unsigned long count)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct aoc_alsa_stream *alsa_stream = runtime->private_data;
	int err = 0;

	err = aoc_audio_read(alsa_stream, buf, count);
	if (err)
		dev_err(component->dev, "ERR:%d fail to get audio from aoc\n", err);

	return err;
}

/* Copy data between hardware buffer and user space */
static int snd_aoc_pcm_copy_user(struct snd_soc_component *component,
				 struct snd_pcm_substream *substream, int channel,
				 unsigned long pos, void __user *buf, unsigned long count)
{
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		return snd_aoc_pcm_playback_copy_user(component, substream, channel, pos, buf,
						      count);
	} else { /* Capture */
		return snd_aoc_pcm_capture_copy_user(component, substream, channel, pos, buf,
						     count);
	}
}

/* Pointer callback */
static snd_pcm_uframes_t snd_aoc_pcm_pointer(struct snd_soc_component *component,
					     struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct aoc_alsa_stream *alsa_stream = runtime->private_data;
	int pointer;

	dev_dbg(component->dev, "pcm_pointer... (%d) hwptr=%ld appl=%ld pos=%d\n", 0,
		frames_to_bytes(runtime, runtime->status->hw_ptr),
		frames_to_bytes(runtime, runtime->control->appl_ptr), alsa_stream->pos);

	pointer = bytes_to_frames(substream->runtime, alsa_stream->pos);

	dev_dbg(component->dev, "pcm pointer  = %d\n", pointer);

	return pointer;
}

static int snd_aoc_pcm_lib_ioctl(struct snd_soc_component *component,
				 struct snd_pcm_substream *substream, unsigned int cmd, void *arg)
{
	int err = snd_pcm_lib_ioctl(substream, cmd, arg);

	dev_dbg(component->dev, " .. substream=%pK, cmd=%d, arg=%pK (%x) err=%d\n", substream, cmd,
		arg, arg ? *(unsigned int *)arg : 0, err);
	return err;
}

static int aoc_pcm_new(struct snd_soc_component *component, struct snd_soc_pcm_runtime *rtd)
{
	struct snd_pcm_substream *substream = NULL;

	dma_set_mask_and_coherent(component->dev, DMA_BIT_MASK(64));

	/* Allocate DMA memory */
	if (rtd->dai_link->dpcm_playback) {
		substream = rtd->pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream;
		snd_pcm_lib_preallocate_pages(substream, SNDRV_DMA_TYPE_CONTINUOUS,
					      component->dev,
					      snd_aoc_playback_hw.buffer_bytes_max,
					      snd_aoc_playback_hw.buffer_bytes_max);
	}

	if (rtd->dai_link->dpcm_capture) {
		substream = rtd->pcm->streams[SNDRV_PCM_STREAM_CAPTURE].substream;
		snd_pcm_lib_preallocate_pages(substream, SNDRV_DMA_TYPE_CONTINUOUS,
					      component->dev,
					      snd_aoc_playback_hw.buffer_bytes_max,
					      snd_aoc_playback_hw.buffer_bytes_max);
	}

	rtd->pcm->nonatomic = true;
	return 0;
}

static const struct snd_soc_component_driver aoc_pcm_component = {
	.name = "AoC PCM",
	.open = snd_aoc_pcm_open,
	.close = snd_aoc_pcm_close,
	.ioctl = snd_aoc_pcm_lib_ioctl,
	.hw_params = snd_aoc_pcm_hw_params,
	.hw_free = snd_aoc_pcm_hw_free,
	.copy_user = snd_aoc_pcm_copy_user,
	.prepare = snd_aoc_pcm_prepare,
	.trigger = snd_aoc_pcm_trigger,
	.pointer = snd_aoc_pcm_pointer,
	.pcm_construct = aoc_pcm_new,
};

static int aoc_pcm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	int err = 0;

	if (!np)
		return -EINVAL;
	err = devm_snd_soc_register_component(dev, &aoc_pcm_component, NULL, 0);
	if (err) {
		dev_err(dev, "ERR:%d fail to reigster aoc pcm comp\n", err);
		return err;
	}

	return 0;
}

static const struct of_device_id aoc_voip_of_match[] = {
	{
		.compatible = "google-aoc-snd-voip",
	},
	{},
};
MODULE_DEVICE_TABLE(of, aoc_voip_of_match);

static struct platform_driver aoc_pcm_drv = {
    .driver =
        {
            .name = "google-aoc-snd-voip",
            .of_match_table = aoc_voip_of_match,
        },
    .probe = aoc_pcm_probe,
};

int aoc_voip_init(void)
{
	int err;

	err = platform_driver_register(&aoc_pcm_drv);
	if (err) {
		pr_err("ERR:%d in registering aoc pcm drv\n", err);
		return err;
	}

	return 0;
}

void aoc_voip_exit(void)
{
	platform_driver_unregister(&aoc_pcm_drv);
}
