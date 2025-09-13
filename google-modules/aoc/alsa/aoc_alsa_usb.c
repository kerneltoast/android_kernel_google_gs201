// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google Whitechapel AoC ALSA  Driver on PCM
 *
 * Copyright (c) 2023 Google LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <sound/core.h>
#include <linux/usb.h>
#include <trace/hooks/audio_usboffload.h>

#include "aoc_alsa.h"
#include "card.h"
#include "usbaudio.h"

struct uaudio_dev {
	int card_num;
	struct snd_usb_audio *chip;
};

static struct uaudio_dev uadev[SNDRV_CARDS];

void (*cb_func)(struct usb_device*, struct usb_host_endpoint*);

static bool snd_usb_is_implicit_feedback(struct snd_usb_endpoint *ep)
{
	return  ep->implicit_fb_sync && usb_pipeout(ep->pipe);
}

static struct snd_usb_substream *find_substream(unsigned int card_num,
	unsigned int device, unsigned int direction)
{
	struct snd_usb_stream *as;
	struct snd_usb_substream *subs = NULL;
	struct snd_usb_audio *chip;

	chip = uadev[card_num].chip;

	if (!chip || atomic_read(&chip->shutdown))
		goto done;

	if (device >= chip->pcm_devs)
		goto done;

	if (direction > SNDRV_PCM_STREAM_CAPTURE)
		goto done;

	list_for_each_entry(as, &chip->pcm_list, list) {
		if (as->pcm_index == device) {
			subs = &as->substream[direction];
			goto done;
		}
	}

done:
	return subs;
}

void usb_audio_offload_connect(struct snd_usb_audio *chip)
{
	int card_num;

	if (!chip)
		return;

	card_num = chip->card->number;
	if (card_num >= SNDRV_CARDS)
		return;

	pr_info("%s card%d connected", __func__, card_num);
	mutex_lock(&chip->mutex);
	uadev[card_num].chip = chip;
	uadev[card_num].card_num = card_num;
	chip->quirk_flags &= ~QUIRK_FLAG_GENERIC_IMPLICIT_FB;
	chip->quirk_flags |= QUIRK_FLAG_SKIP_IMPLICIT_FB;
	mutex_unlock(&chip->mutex);
}

void usb_audio_offload_disconnect(struct snd_usb_audio *chip)
{
	int card_num;

	if (!chip)
		return;

	card_num = chip->card->number;
	if (card_num >= SNDRV_CARDS)
		return;

	mutex_lock(&chip->mutex);
	pr_info("%s card%d disconnected", __func__, card_num);
	uadev[card_num].chip = NULL;
	uadev[card_num].card_num = 0;
	mutex_unlock(&chip->mutex);
}

void usb_audio_offload_suspend(struct usb_interface *intf, pm_message_t message)
{
	pr_debug("%s", __func__);
}

int aoc_set_usb_mem_config(struct aoc_chip *achip)
{
	struct usb_host_endpoint *ep = NULL;
	struct usb_host_endpoint *fb_ep = NULL;
	struct snd_usb_substream *subs = NULL;
	struct snd_usb_audio *chip = NULL;
	unsigned int card_num;
	unsigned int device;
	unsigned int direction;
	bool implicit_fb = false;

	if (!achip)
		return -ENODEV;

	card_num = achip->usb_card;
	device = achip->usb_device;
	direction = achip->usb_direction;
	if (card_num < SNDRV_CARDS) {
		chip = uadev[card_num].chip;
	}
	if (!chip) {
		pr_err("%s no device connected (card %u device %u, direction %u)",
			__func__, card_num, device, direction);
		return -ENODEV;
	}
	pr_info("%s card %u device %u, direction %u", __func__,
			card_num, device, direction);
	if (direction <= SNDRV_PCM_STREAM_CAPTURE) {
		mutex_lock(&chip->mutex);
		subs = find_substream(card_num, device, direction);
		if (!subs) {
			pr_err("%s subs not found", __func__);
			mutex_unlock(&chip->mutex);
			return -ENODEV;
		}
		ep = usb_pipe_endpoint(subs->dev, subs->data_endpoint->pipe);
		implicit_fb = snd_usb_is_implicit_feedback(subs->data_endpoint);
		if (!ep) {
			pr_err("%s data ep # %d context is null\n",
					__func__, subs->data_endpoint->ep_num);
			mutex_unlock(&chip->mutex);
			return -ENODEV;
		}

		if (cb_func) {
			cb_func(subs->dev, ep);
		} else {
			pr_info("%s call aoc_alsa_usb_callback_register() first, skip", __func__);
		}

		if (subs->sync_endpoint && !implicit_fb) {
			fb_ep = usb_pipe_endpoint(subs->dev, subs->sync_endpoint->pipe);
			if (!fb_ep) {
				pr_info("%s sync ep # %d context is null\n",
						__func__, subs->sync_endpoint->ep_num);
			} else {
				if (cb_func) {
					cb_func(subs->dev, fb_ep);
				}
				aoc_set_usb_feedback_endpoint(achip, subs->dev, fb_ep);
			}
		}
		mutex_unlock(&chip->mutex);
	}

	aoc_set_usb_offload_state(achip, true);
	return 0;
}

bool aoc_alsa_usb_callback_register(
			void (*callback)(struct usb_device*, struct usb_host_endpoint*))
{
	pr_info("%s usb callback register", __func__);
	if (callback) {
		cb_func = callback;
	} else {
		pr_err("%s: cb_func register fail", __func__);
		return false;
	}
	return true;
}
EXPORT_SYMBOL_GPL(aoc_alsa_usb_callback_register);

void aoc_alsa_usb_callback_unregister(void)
{
	pr_info("%s usb callback unregister", __func__);
	if (cb_func) {
		cb_func = NULL;
	}
}
EXPORT_SYMBOL_GPL(aoc_alsa_usb_callback_unregister);

static void audio_usb_offload_connect(void *unused, struct usb_interface *intf,
			struct snd_usb_audio *chip)
{
	usb_audio_offload_connect(chip);
}

static void audio_usb_offload_disconnect(void *unused, struct usb_interface *intf)
{
	struct snd_usb_audio *chip = usb_get_intfdata(intf);

	usb_audio_offload_disconnect(chip);
}

int aoc_usb_init(void)
{
	int ret = 0;

	register_trace_android_vh_audio_usb_offload_connect(audio_usb_offload_connect, NULL);
	register_trace_android_rvh_audio_usb_offload_disconnect(audio_usb_offload_disconnect, NULL);

	return ret;
}

void aoc_usb_exit(void)
{
	unregister_trace_android_vh_audio_usb_offload_connect(audio_usb_offload_connect, NULL);
}

