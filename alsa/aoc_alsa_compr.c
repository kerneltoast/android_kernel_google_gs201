// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google Whitechapel AoC ALSA  Driver on Compr offload
 *
 * Copyright (c) 2019 Google LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
//#define DEBUG

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/version.h>

#include "aoc_alsa.h"
#include "aoc_alsa_drv.h"

static void aoc_stop_work_handler(struct work_struct *work);

static void aoc_compr_reset_handler(aoc_aud_service_event_t evnt, void *cookies)
{
	struct aoc_alsa_stream *alsa_stream = (struct aoc_alsa_stream *)cookies;

	if (!alsa_stream || !alsa_stream->cstream) {
		pr_err("%s: no active compress offload stream pointer\n", __func__);
		return;
	}

	if (evnt == AOC_SERVICE_EVENT_DOWN) {
		pr_debug("%s: AoC service is removed, wakeup sleep thread\n", __func__);
		/*
		 * We don't hold stream lock here before change stream's state because of avoiding
		 * deadlock on stream lock and chip->audio_lock. The tasks without checking xrun
		 * still return error since AOC isn't ready. Most of alsa stream tasks from
		 * user-space will be returned by xrun state.
		 */
		alsa_stream->cstream->runtime->state = SNDRV_PCM_STATE_XRUN;
		wake_up(&alsa_stream->cstream->runtime->sleep);

		schedule_work(&alsa_stream->free_aoc_service_work);
		return;
	}
}

static void aoc_compr_reset_pointer(struct aoc_alsa_stream *alsa_stream)
{
	struct snd_compr_stream *cstream = alsa_stream->cstream;
	struct aoc_service_dev *dev = alsa_stream->dev;

	alsa_stream->hw_ptr_base = (cstream->direction == SND_COMPRESS_PLAYBACK) ?
						 aoc_ring_bytes_read(dev->service, AOC_DOWN) :
						 aoc_ring_bytes_written(dev->service, AOC_UP);
	alsa_stream->prev_consumed = alsa_stream->hw_ptr_base;
	alsa_stream->n_overflow = 0;
	cstream->runtime->total_bytes_available = 0;

	pr_debug("%s hw_ptr_base = %lu compr_pcm_io_sample_base = %llu\n",
		__func__, alsa_stream->hw_ptr_base, alsa_stream->compr_pcm_io_sample_base);

}

int aoc_compr_offload_reset_io_sample_base(struct aoc_alsa_stream *alsa_stream)
{
	int err = 0;

	err = aoc_compr_offload_get_io_samples(alsa_stream,
		&alsa_stream->compr_pcm_io_sample_base);
	if (err < 0) {
		pr_err("ERR: fail to get audio playback samples, err = %d\n", err);
		return err;
	}
	pr_info("%s: compr_pcm_io_sample_base = %llu\n",
		__func__, alsa_stream->compr_pcm_io_sample_base);
	return err;
}

void aoc_compr_offload_isr(struct aoc_service_dev *dev)
{
	struct aoc_alsa_stream *alsa_stream;
	unsigned long consumed, n;

	if (!dev) {
		pr_err("ERR: NULL compress offload aoc service pointer\n");
		return;
	}

	alsa_stream = dev->prvdata;
	if (!alsa_stream || !alsa_stream->cstream) {
		pr_err("ERR: NULL compress offload stream pointer\n");
		return;
	}

	pm_wakeup_ws_event(alsa_stream->chip->wakelock, 1000, true);

	/*
	 * The number of bytes read/writtien should be the bytes in the buffer
	 * already played out in the case of playback. But this may not be true
	 * in the AoC ring buffer implementation, since the reader pointer in
	 * the playback case represents what has been read from the buffer,
	 * not what already played out .
	 */

	/* Check EOF (n>0), and then flush the buffer for next EOF */
	n = aoc_ring_bytes_available_to_read(alsa_stream->dev_eof->service, AOC_UP);
	if (n > 0) {
		if (!aoc_ring_flush_read_data(alsa_stream->dev_eof->service, AOC_UP, 0))
			pr_err("ERR: decoder_eof ring buffer flush fail\n");

		if (alsa_stream->draining == 1) {
			pr_info("compress offload ring buffer is depleted\n");
			snd_compr_drain_notify(alsa_stream->cstream);
			alsa_stream->eof_reach = 1;
			alsa_stream->draining = 0;
			return;
		}
	}

	consumed = aoc_ring_bytes_read(dev->service, AOC_DOWN);

	/* TODO: To do more on no pointer update? */
	if (consumed == alsa_stream->prev_consumed)
		return;

	pr_debug("compr offload consumed = %lu, hw_ptr_base = %lu\n", consumed,
		 alsa_stream->hw_ptr_base);

	/* To deal with overlfow in Tx or Rx in int32_t */
	if (consumed < alsa_stream->prev_consumed) {
		alsa_stream->n_overflow++;
		pr_notice("overflow in Tx/Rx: %lu - %lu - %d times\n", consumed,
			  alsa_stream->prev_consumed, alsa_stream->n_overflow);
	}
	alsa_stream->prev_consumed = consumed;

	/* Update the pcm pointer */
	if (unlikely(alsa_stream->n_overflow)) {
		alsa_stream->pos = (consumed + 0x100000000 * alsa_stream->n_overflow -
				    alsa_stream->hw_ptr_base) %
				   alsa_stream->buffer_size;
	} else {
		alsa_stream->pos = (consumed - alsa_stream->hw_ptr_base) % alsa_stream->buffer_size;
	}

	/* Wake up the sleeping thread */
	if (alsa_stream->cstream)
		snd_compr_fragment_elapsed(alsa_stream->cstream);

	return;
}

/* TODO: the handler has to be changed based on the compress offload */
/*  the pointer should be modified based on the interrupt from AoC */
static enum hrtimer_restart aoc_compr_hrtimer_irq_handler(struct hrtimer *timer)
{
#ifdef AOC_COMPR_HRTIMER_IRQ_HANDLER_BYPASS
	return HRTIMER_NORESTART;
#else
	struct aoc_alsa_stream *alsa_stream;
	struct aoc_service_dev *dev;
	unsigned long consumed;

	if (!timer) {
		pr_err("ERR: NULL timer pointer\n");
		return HRTIMER_NORESTART;
	}

	alsa_stream = container_of(timer, struct aoc_alsa_stream, hr_timer);

	if (!alsa_stream || !alsa_stream->cstream) {
		pr_err("ERR: NULL compress offload stream pointer\n");
		return HRTIMER_NORESTART;
	}

	/* Start the hr timer immediately for next period */
	aoc_timer_restart(alsa_stream);

	/*
	 * The number of bytes read/writtien should be the bytes in the buffer
	 * already played out in the case of playback. But this may not be true
	 * in the AoC ring buffer implementation, since the reader pointer in
	 * the playback case represents what has been read from the buffer,
	 * not what already played out .
	 */
	dev = alsa_stream->dev;

	if (aoc_ring_bytes_available_to_read(dev->service, AOC_DOWN) == 0) {
		pr_info("compress offload ring buffer is depleted\n");
		snd_compr_drain_notify(alsa_stream->cstream);
		return HRTIMER_RESTART;
	}

	consumed = aoc_ring_bytes_read(dev->service, AOC_DOWN);

	/* TODO: To do more on no pointer update? */
	if (consumed == alsa_stream->prev_consumed)
		return HRTIMER_RESTART;

	pr_debug("consumed = %lu, hw_ptr_base = %lu\n", consumed,
		 alsa_stream->hw_ptr_base);

	/* To deal with overlfow in Tx or Rx in int32_t */
	if (consumed < alsa_stream->prev_consumed) {
		alsa_stream->n_overflow++;
		pr_notice("overflow in Tx/Rx: %lu - %lu - %d times\n", consumed,
			  alsa_stream->prev_consumed, alsa_stream->n_overflow);
	}
	alsa_stream->prev_consumed = consumed;

	/* Update the pcm pointer */
	if (unlikely(alsa_stream->n_overflow)) {
		alsa_stream->pos =
			(consumed + 0x100000000 * alsa_stream->n_overflow -
			 alsa_stream->hw_ptr_base) % alsa_stream->buffer_size;
	} else {
		alsa_stream->pos = (consumed - alsa_stream->hw_ptr_base) %
				   alsa_stream->buffer_size;
	}

	/* Wake up the sleeping thread */
	if (alsa_stream->cstream)
		snd_compr_fragment_elapsed(alsa_stream->cstream);

	return HRTIMER_RESTART;
#endif
}

static int aoc_compr_prepare(struct aoc_alsa_stream *alsa_stream)
{
	int err;
	struct snd_compr_stream *cstream = alsa_stream->cstream;
	struct aoc_service_dev *dev = alsa_stream->dev;

	if (cstream->direction == SND_COMPRESS_PLAYBACK) {
		/*
		 * Reset write pointer to ensure writing to the base of the ring.
		 * AoC requires this to allow DMA transfers to be aligned correctly.
		 */
		aoc_ring_reset_write_pointer(dev->service, AOC_DOWN);
	}

	/* No prepare() for compress offload, so do buffer flushing here */
	err = aoc_compr_offload_flush_buffer(alsa_stream);
	if (err != 0) {
		pr_err("ERR: fail to flush compress offload buffer\n");
		return -EFAULT;
	}

	alsa_stream->hw_ptr_base = (cstream->direction == SND_COMPRESS_PLAYBACK) ?
						 aoc_ring_bytes_read(dev->service, AOC_DOWN) :
						 aoc_ring_bytes_written(dev->service, AOC_UP);
	alsa_stream->prev_consumed = alsa_stream->hw_ptr_base;
	alsa_stream->n_overflow = 0;

	pr_debug("compress offload hw_ptr_base =%lu\n", alsa_stream->hw_ptr_base);

	return 0;
}

static void aoc_stop_work_handler(struct work_struct *work)
{
	struct aoc_alsa_stream *alsa_stream =
		container_of(work, struct aoc_alsa_stream, free_aoc_service_work);
	struct aoc_chip *chip;
	int err;

	if (!alsa_stream)
		return;

	chip = alsa_stream->chip;
	if (mutex_lock_interruptible(&chip->audio_mutex)) {
		pr_err("ERR: interrupted while waiting for lock\n");
		return;
	}

	if (alsa_stream->running) {
		err = aoc_audio_stop(alsa_stream);
		if (err != 0) {
			pr_err("failed to STOP alsa device (%d)\n",
			       err);
		}
		alsa_stream->running = 0;
	}

	aoc_timer_stop(alsa_stream);
	aoc_compr_prepare(alsa_stream);
	mutex_unlock(&chip->audio_mutex);
	return;
}

static int aoc_compr_playback_open(struct snd_compr_stream *cstream)
{
	struct snd_soc_pcm_runtime *rtd =
		(struct snd_soc_pcm_runtime *)cstream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct aoc_chip *chip =
		(struct aoc_chip *)snd_soc_card_get_drvdata(card);

	struct snd_compr_runtime *runtime = cstream->runtime;

	struct aoc_alsa_stream *alsa_stream = NULL;
	struct aoc_service_dev *dev = NULL;
	struct aoc_service_dev *dev_eof = NULL;
	int idx;
	int err;

	if (mutex_lock_interruptible(&chip->audio_mutex)) {
		pr_err("ERR: interrupted whilst waiting for lock\n");
		return -EINTR;
	}

	idx = cstream->device->device;
	pr_notice("alsa compr offload open (%d)\n", idx);
	pr_debug("chip open (%llu)\n", chip->opened);

	alsa_stream = kzalloc(sizeof(struct aoc_alsa_stream), GFP_KERNEL);
	if (alsa_stream == NULL) {
		err = -ENOMEM;
		pr_err("ERR: no memory for %s", rtd->dai_link->name);
		goto out;
	}

	INIT_WORK(&alsa_stream->free_aoc_service_work, aoc_stop_work_handler);

	/* Find the corresponding aoc audio service */
	err = alloc_aoc_audio_service(rtd->dai_link->name, &dev, aoc_compr_reset_handler,
			alsa_stream);
	if (err < 0) {
		pr_err("ERR: fail to alloc service for %s",
		       rtd->dai_link->name);
		goto out;
	}

	err = alloc_aoc_audio_service(AOC_COMPR_OFFLOAD_EOF_SERVICE, &dev_eof, NULL, NULL);
	if (err < 0) {
		pr_err("ERR: fail to alloc service for " AOC_COMPR_OFFLOAD_EOF_SERVICE);
		goto out;
	}

	/* Initialise alsa_stream */
	alsa_stream->chip = chip;
	alsa_stream->cstream = cstream;
	alsa_stream->substream = NULL;
	alsa_stream->dev = dev;
	alsa_stream->dev_eof = dev_eof;
	alsa_stream->idx = idx;
	alsa_stream->compr_padding = COMPR_INVALID_METADATA;
	alsa_stream->compr_delay = COMPR_INVALID_METADATA;
	alsa_stream->send_metadata = 1;
	alsa_stream->eof_reach = 0;
	alsa_stream->gapless_offload_enable = chip->gapless_offload_enable;
	snd_compr_use_pause_in_draining(cstream);

	err = aoc_audio_open(alsa_stream);
	if (err != 0) {
		pr_err("fail to audio open for %s", rtd->dai_link->name);
		goto out;
	}
	runtime->private_data = alsa_stream;
	chip->alsa_stream[idx] = alsa_stream;
	chip->opened |= (1 << idx);
	alsa_stream->open = 1;
	alsa_stream->draining = 0; // set to 1 when partial drain

	alsa_stream->timer_interval_ns = COMPR_OFFLOAD_TIMER_INTERVAL_NANOSECS;
	hrtimer_init(&(alsa_stream->hr_timer), CLOCK_MONOTONIC,
		     HRTIMER_MODE_REL);
	alsa_stream->hr_timer.function = &aoc_compr_hrtimer_irq_handler;

	dev->prvdata = alsa_stream; /* For interrupt-driven playback */

	alsa_stream->entry_point_idx = idx;

	/* No prepare() for compress offload, so do buffer flushing here */
	err = aoc_compr_prepare(alsa_stream);
	if (err != 0) {
		pr_err("fail to prepare compress offload buffer: %s", rtd->dai_link->name);
		goto out;
	}

	/* TODO: temporary compr offload volume set to protect speaker*/
	aoc_audio_volume_set(chip, chip->compr_offload_volume, idx, 0);

	chip->compr_offload_stream = alsa_stream;
	mutex_unlock(&chip->audio_mutex);

	return 0;
out:
	kfree(alsa_stream);
	if (dev) {
		free_aoc_audio_service(rtd->dai_link->name, dev);
		free_aoc_audio_service(AOC_COMPR_OFFLOAD_EOF_SERVICE, dev_eof);
		dev = NULL;
		dev_eof = NULL;
	}
	chip->alsa_stream[idx] = NULL;
	chip->opened &= ~(1 << idx);
	mutex_unlock(&chip->audio_mutex);

	pr_err("pcm open err=%d\n", err);
	return err;
}

static int aoc_compr_playback_free(struct snd_compr_stream *cstream)
{
	struct snd_soc_pcm_runtime *rtd =
		(struct snd_soc_pcm_runtime *)cstream->private_data;
	struct snd_compr_runtime *runtime = cstream->runtime;

	struct aoc_alsa_stream *alsa_stream = runtime->private_data;
	struct aoc_chip *chip = alsa_stream->chip;
	int err;

	pr_debug("dai name %s, cstream %pK\n", rtd->dai_link->name, cstream);
	aoc_timer_stop_sync(alsa_stream);

	cancel_work_sync(&alsa_stream->free_aoc_service_work);
	if (mutex_lock_interruptible(&chip->audio_mutex)) {
		pr_err("ERR: interrupted while waiting for lock\n");
		return -EINTR;
	}
	chip->compr_offload_stream = NULL;

	pr_notice("alsa compr offload close\n");
	free_aoc_audio_service(rtd->dai_link->name, alsa_stream->dev);
	free_aoc_audio_service(AOC_COMPR_OFFLOAD_EOF_SERVICE, alsa_stream->dev_eof);

	/*
	 * Call stop if it's still running. This happens when app
	 * is force killed and we don't get a stop trigger.
	 */
	if (alsa_stream->running) {
		err = aoc_audio_stop(alsa_stream);
		alsa_stream->running = 0;
		if (err != 0)
			pr_err("ERR: failed to stop the stream\n");
		aoc_compr_offload_close(alsa_stream);
	}

	if (alsa_stream->open) {
		alsa_stream->open = 0;
		aoc_audio_close(alsa_stream);
	}
	if (alsa_stream->chip)
		alsa_stream->chip->alsa_stream[alsa_stream->idx] = NULL;
	/*
	 * Do not free up alsa_stream here, it will be freed up by
	 * runtime->private_free callback we registered in *_open above
	 * TODO: no such operation for compress offload
	 */
	chip->opened &= ~(1 << alsa_stream->idx);
	kfree(alsa_stream);

	mutex_unlock(&chip->audio_mutex);

	return 0;
}

static int aoc_compr_open(struct snd_soc_component *component, struct snd_compr_stream *cstream)
{
	int ret = 0;
	if (cstream->direction == SND_COMPRESS_PLAYBACK)
		ret = aoc_compr_playback_open(cstream);

	return ret;
}

static int aoc_compr_free(struct snd_soc_component *component, struct snd_compr_stream *cstream)
{
	int ret = 0;
	if (cstream->direction == SND_COMPRESS_PLAYBACK)
		ret = aoc_compr_playback_free(cstream);

	return ret;
}

static int aoc_compr_trigger(struct snd_soc_component *component, struct snd_compr_stream *cstream,
			     int cmd)
{
	struct snd_compr_runtime *runtime = cstream->runtime;
	struct aoc_alsa_stream *alsa_stream = runtime->private_data;
	int err = 0;

	pr_debug("%s: cmd = %d\n", __func__, cmd);
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		pr_debug("%s: SNDRV_PCM_TRIGGER_START\n", __func__);
		if (alsa_stream->running)
			break;

		/* start timer first to avoid underrun/overrun */
		pr_debug("%s: start timer\n", __func__);
		aoc_timer_start(alsa_stream);

		/* Decoder type: SND_AUDIOCODEC_MP3 or SND_AUDIOCODEC_AAC */
		err = aoc_compr_offload_setup(alsa_stream, alsa_stream->compr_offload_codec);
		if (err < 0) {
			pr_err("ERR:%d decoder setup fail\n", err);
			goto out;
		}

		err = aoc_audio_start(alsa_stream);
		if (err == 0) {
			alsa_stream->running = 1;
		} else {
			pr_err(" Failed to START alsa device (%d)\n", err);
		}
		break;

	case SNDRV_PCM_TRIGGER_STOP:
		pr_debug("%s: SNDRV_PCM_TRIGGER_STOP\n", __func__);
		if (alsa_stream->running) {
			err = aoc_audio_stop(alsa_stream);
			if (err != 0) {
				pr_err("failed to STOP alsa device (%d)\n",
				       err);
				/* return 0 to wake up sleep thread to unblock compress_poll() */
				err = 0;
			}
			alsa_stream->running = 0;
		}

		aoc_timer_stop(alsa_stream);
		aoc_compr_offload_close(alsa_stream);
		aoc_compr_prepare(alsa_stream);
		break;

	case SND_COMPR_TRIGGER_DRAIN:
		pr_debug("%s: SNDRV_PCM_TRIGGER_DRAIN\n", __func__);
		alsa_stream->draining = 1;
		break;

	case SND_COMPR_TRIGGER_PARTIAL_DRAIN:
		pr_debug("%s: SNDRV_PCM_TRIGGER_PARTIAL_DRAIN\n", __func__);
		aoc_compr_offload_partial_drain(alsa_stream);
		alsa_stream->draining = 1;
		break;

	case SND_COMPR_TRIGGER_NEXT_TRACK:
		pr_debug("%s: SND_COMPR_TRIGGER_NEXT_TRACK\n", __func__);
		alsa_stream->compr_padding = COMPR_INVALID_METADATA;
		alsa_stream->compr_delay = COMPR_INVALID_METADATA;
		alsa_stream->send_metadata = 1;
		break;

	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		pr_debug("%s: SNDRV_PCM_TRIGGER_PAUSE_PUSH\n", __func__);
		if (alsa_stream->running) {
			err = aoc_compr_pause(alsa_stream);
			if (err != 0)
				pr_err("failed to pause alsa device (%d)\n",
				       err);
			cstream->runtime->state = SNDRV_PCM_STATE_PAUSED;
			wake_up(&cstream->runtime->sleep);
		}
		break;

	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		pr_debug("%s: SNDRV_PCM_TRIGGER_PAUSE_RELEASE\n", __func__);
		if (alsa_stream->running) {
			err = aoc_compr_resume(alsa_stream);
			if (err != 0)
				pr_err("failed to resume alsa device (%d)\n",
				       err);
		}
		break;

	default:
		err = -EINVAL;
	}
out:
	return err;
}

int aoc_compr_get_position(struct aoc_alsa_stream *alsa_stream, uint64_t *position)
{
	uint64_t current_sample = 0;

	if (position == NULL) {
		pr_err("%s: invalid position\n", __func__);
		return -EINVAL;
	}

	if (aoc_compr_offload_get_io_samples(alsa_stream, &current_sample) < 0) {
		pr_err("%s: failed to get playback samples\n", __func__);
		return -EINVAL;
	}

	*position = (current_sample - alsa_stream->compr_pcm_io_sample_base) *
		    (long)alsa_stream->params_rate / AOC_COMPR_OFFLOAD_DEFAULT_SR;

	pr_debug("%s: current_sample=%llu base=%llu\n", __func__,
			current_sample, alsa_stream->compr_pcm_io_sample_base);
	return 0;
}

static int aoc_compr_pointer(struct snd_soc_component *component, struct snd_compr_stream *cstream,
			     struct snd_compr_tstamp *arg)
{
	struct snd_compr_runtime *runtime = cstream->runtime;
	struct aoc_alsa_stream *alsa_stream = runtime->private_data;
	uint64_t current_sample = 0;

	//pr_debug("%s, %pK, %pK\n", __func__, runtime, arg);

	arg->byte_offset = alsa_stream->pos;
	arg->copied_total = alsa_stream->prev_consumed - alsa_stream->hw_ptr_base;

	/* TODO: overflow in samples, HAL only uses pcm_io_samples for timestamps */
	arg->sampling_rate = alsa_stream->params_rate;

	if (aoc_compr_offload_get_io_samples(alsa_stream, &current_sample) < 0) {
		pr_err("%s: failed to get playback samples\n", __func__);
		return 0;
	}
	arg->pcm_io_frames = (current_sample - alsa_stream->compr_pcm_io_sample_base) *
			     (long)arg->sampling_rate / AOC_COMPR_OFFLOAD_DEFAULT_SR;

	pr_debug("compr ptr -total bytes: %llu copied: %u diff:%llu,sampes=%u,fs=%d,base=%llu\n",
		 runtime->total_bytes_available, arg->copied_total,
		 runtime->total_bytes_available - arg->copied_total, arg->pcm_io_frames,
		 arg->sampling_rate, alsa_stream->compr_pcm_io_sample_base);

	if ((runtime->total_bytes_available - arg->copied_total) == runtime->buffer_size)
		__pm_relax(alsa_stream->chip->wakelock);

	return 0;
}

static int aoc_compr_ack(struct snd_soc_component *component, struct snd_compr_stream *cstream,
			 size_t count)
{
	struct snd_compr_runtime *runtime = cstream->runtime;

	pr_debug("%s, %pK, %zu\n", __func__, runtime, count);

	return 0;
}

static int aoc_compr_playback_copy(struct snd_compr_stream *cstream,
				   char __user *buf, size_t count)
{
	struct snd_compr_runtime *runtime = cstream->runtime;
	struct aoc_alsa_stream *alsa_stream = runtime->private_data;
	int err = 0;

	err = aoc_compr_offload_send_metadata(alsa_stream);
	if (err < 0)
		pr_err("ERR: %d failed to send metadata\n", err);

	err = aoc_audio_write(alsa_stream, buf, count);
	if (err < 0) {
		pr_err("ERR:%d failed to write to buffer\n", err);
		return err;
	}

	pr_debug("%s: count %lu\n", __func__, count);
	return count;
}

static int aoc_compr_copy(struct snd_soc_component *component, struct snd_compr_stream *cstream,
			  char __user *buf, size_t count)
{
	int ret = 0;

	if (cstream->direction == SND_COMPRESS_PLAYBACK)
		ret = aoc_compr_playback_copy(cstream, buf, count);

	return ret;
}

static int aoc_compr_get_caps(struct snd_soc_component *component, struct snd_compr_stream *cstream,
			      struct snd_compr_caps *arg)
{
	struct snd_compr_runtime *runtime = cstream->runtime;
	int ret = 0;

	pr_debug("%s, %pK, %pK\n", __func__, runtime, arg);

	return ret;
}

static int aoc_compr_get_codec_caps(struct snd_soc_component *component,
				    struct snd_compr_stream *cstream,
				    struct snd_compr_codec_caps *codec)
{
	pr_debug("%s, %d\n", __func__, codec->codec);

	switch (codec->codec) {
#if !IS_ENABLED(CONFIG_SOC_GS101) && !IS_ENABLED(CONFIG_SOC_GS201)
	case SND_AUDIOCODEC_PCM:
		break;
#endif
	case SND_AUDIOCODEC_MP3:
		break;
	case SND_AUDIOCODEC_AAC:
		break;
	default:
		pr_err("%s: Unsupported audio codec %d\n", __func__,
		       codec->codec);
		return -EINVAL;
	}

	return 0;
}

static int aoc_compr_set_metadata(struct snd_soc_component *component,
				  struct snd_compr_stream *cstream,
				  struct snd_compr_metadata *metadata)
{
	struct snd_compr_runtime *runtime = cstream->runtime;
	struct aoc_alsa_stream *alsa_stream = runtime->private_data;
	int ret = 0;

	pr_debug("%s %pK, %pK\n", __func__, runtime, metadata);
	/* clear pointer for gapless offload playback, call sequence should be:
	 * receive EOF -> notify EOF to ap -> ap set_metadata for next track
	 * -> ap write buffer for next track -> write metadata to aoc ->
	 * write next track buffer to aoc.
	 * Reset buffer pointer when ap set_metadata for next track.
	 */
	if (alsa_stream->eof_reach) {
		alsa_stream->eof_reach = 0;
		aoc_compr_reset_pointer(alsa_stream);
	}

	if (metadata->key == SNDRV_COMPRESS_ENCODER_PADDING) {
		alsa_stream->compr_padding = metadata->value[0];
	} else if (metadata->key == SNDRV_COMPRESS_ENCODER_DELAY) {
		alsa_stream->compr_delay = metadata->value[0];
	} else {
		pr_warn("invalid key %d\n", metadata->key);
		ret = -EINVAL;
	}
	return ret;
}

static int aoc_compr_get_metadata(struct snd_soc_component *component,
				  struct snd_compr_stream *cstream,
				  struct snd_compr_metadata *metadata)
{
	struct snd_compr_runtime *runtime = cstream->runtime;
	struct aoc_alsa_stream *alsa_stream = runtime->private_data;
	int ret = 0;

	pr_debug("%s %pK, %pK\n", __func__, runtime, metadata);
	if (metadata->key == SNDRV_COMPRESS_ENCODER_PADDING) {
		metadata->value[0] = alsa_stream->compr_padding;
	} else if (metadata->key == SNDRV_COMPRESS_ENCODER_DELAY) {
		metadata->value[0] = alsa_stream->compr_delay;
	} else {
		pr_warn("invalid key %d\n", metadata->key);
		ret = -EINVAL;
	}

	return ret;
}

static int snd_audiocodec_to_aoc_decoder(int snd_type, int codec_param)
{
	pr_info("%s: snd_type=%x, codec_param=%x\n", __func__, snd_type, codec_param);
#if IS_ENABLED(CONFIG_SOC_ZUMA)
	if (((codec_param >> 16) & 0xFFFF) == AOC_CODEC_TAG) {
		int codec = codec_param & 0xFFFF;
		if (codec == AOC_CODEC_OPUS)
			return AUDIO_OUTPUT_DECODER_OPUS;
	}
#endif
	switch(snd_type) {
#if !IS_ENABLED(CONFIG_SOC_GS101) && !IS_ENABLED(CONFIG_SOC_GS201)
	case SND_AUDIOCODEC_PCM:
		return AUDIO_OUTPUT_DECODER_PCM;
#endif
	case SND_AUDIOCODEC_MP3:
		return AUDIO_OUTPUT_DECODER_MP3;
	case SND_AUDIOCODEC_AAC:
		return AUDIO_OUTPUT_DECODER_AAC_LC;
	default:
		return AUDIO_OUTPUT_DECODER_UNKNOWN;
	}
}

static int aoc_compr_set_params(struct snd_soc_component *component,
				struct snd_compr_stream *cstream, struct snd_compr_params *params)
{
	struct snd_compr_runtime *runtime = cstream->runtime;
	struct aoc_alsa_stream *alsa_stream = runtime->private_data;
	struct snd_codec *codec = &params->codec;

	uint8_t *temp_data_buf;
	int buffer_size;
#if !IS_ENABLED(CONFIG_SOC_GS101) && !IS_ENABLED(CONFIG_SOC_GS201)
	int i;
#endif

	pr_debug("%s, fragment size = %d, number of fragment = %d\n", __func__,
		 params->buffer.fragment_size, params->buffer.fragments);

	/* DRAM compr offload buffer size in runtime */
	buffer_size = params->buffer.fragment_size * params->buffer.fragments;

	pr_debug("%s buffer size: %d\n", __func__, buffer_size);

	alsa_stream->offload_temp_data_buf_size = COMPR_OFFLOAD_KERNEL_TMP_BUF_SIZE;
	temp_data_buf = kmalloc_array(alsa_stream->offload_temp_data_buf_size,
				      sizeof(*temp_data_buf), GFP_KERNEL);
	if (!temp_data_buf) {
		pr_err("ERR: not enough memory allocated for compress offload\n");
		return -ENOMEM;
	}

	runtime->buffer = temp_data_buf;
	alsa_stream->buffer_size = buffer_size;
	alsa_stream->period_size = params->buffer.fragment_size;
	alsa_stream->params_rate = params->codec.sample_rate;

	/* TODO: need to double check on the AoC decoder requirements */
	alsa_stream->channels  =  params->codec.ch_out;
	alsa_stream->compr_offload_codec =
		snd_audiocodec_to_aoc_decoder(params->codec.id, codec->reserved[0]);
	memset(alsa_stream->compr_offload_codec_options, 0,
		sizeof(alsa_stream->compr_offload_codec_options));
#if !IS_ENABLED(CONFIG_SOC_GS101) && !IS_ENABLED(CONFIG_SOC_GS201)
	if (alsa_stream->compr_offload_codec == AUDIO_OUTPUT_DECODER_PCM)
		for(i = 0;i < CODEC_RESERVED_SIZE;i ++)
			alsa_stream->compr_offload_codec_options[i] =
				(uint8_t)codec->reserved[i];
#endif
	if (alsa_stream->compr_offload_codec == AUDIO_OUTPUT_DECODER_UNKNOWN) {
		pr_err("ERR: unsupport codec %x\n", params->codec.id);
		return -EINVAL;
	}
	/* TODO: send the codec info to AoC ? */

	return 0;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 9, 0))
static const struct snd_compress_ops snd_aoc_compr_ops = {
#else
static const struct snd_compr_ops snd_aoc_compr_ops = {
#endif
	.open = aoc_compr_open,
	.free = aoc_compr_free,
	.set_params = aoc_compr_set_params,
	.set_metadata = aoc_compr_set_metadata,
	.get_metadata = aoc_compr_get_metadata,
	.trigger = aoc_compr_trigger,
	.pointer = aoc_compr_pointer,
	.copy = aoc_compr_copy,
	.ack = aoc_compr_ack,
	.get_caps = aoc_compr_get_caps,
	.get_codec_caps = aoc_compr_get_codec_caps,
};

static int aoc_compr_new(struct snd_soc_component *component, struct snd_soc_pcm_runtime *rtd)
{
	pr_debug("%s, %pK", __func__, rtd);

	return 0;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 9, 0))
static const struct snd_soc_component_driver aoc_compr_component = {
	.name = "AoC COMPR",
	.compress_ops = &snd_aoc_compr_ops,
	.pcm_construct = aoc_compr_new,
};
#elif (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 18, 0))
static const struct snd_soc_component_driver aoc_compr_component = {
	.name = "AoC COMPR",
	.compr_ops = &snd_aoc_compr_ops,
	.pcm_new = aoc_compr_new,
};
#else
static const struct snd_soc_platform_driver aoc_compr_platform = {
	.compr_ops = &snd_aoc_compr_ops,
	.pcm_new = aoc_compr_new,
};
#endif

static int aoc_compr_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	int err = 0;

	pr_debug("%s", __func__);

	if (!np)
		return -EINVAL;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 18, 0))
	err = devm_snd_soc_register_component(dev, &aoc_compr_component, NULL,
					      0);
	if (err)
		pr_err("ERR:%d fail to reigster aoc pcm comp\n", err);
#else
	err = devm_snd_soc_register_platform(dev, &aoc_compr_platform);
	if (err) {
		pr_err("ERR:%d fail to reigster aoc pcm platform %d", err);
	}
#endif
	return err;
}

static const struct of_device_id aoc_compr_of_match[] = {
	{
		.compatible = "google-aoc-snd-compr",
	},
	{},
};
MODULE_DEVICE_TABLE(of, aoc_compr_of_match);

static struct platform_driver aoc_compr_drv = {
    .driver =
        {
            .name = "google-aoc-snd-compr",
            .of_match_table = aoc_compr_of_match,
        },
    .probe = aoc_compr_probe,
};

int aoc_compr_init(void)
{
	int err;

	pr_debug("%s", __func__);
	err = platform_driver_register(&aoc_compr_drv);
	if (err) {
		pr_err("ERR:%d fail in registering aoc compr drv\n", err);
		return err;
	}

	return 0;
}

void aoc_compr_exit(void)
{
	platform_driver_unregister(&aoc_compr_drv);
}
