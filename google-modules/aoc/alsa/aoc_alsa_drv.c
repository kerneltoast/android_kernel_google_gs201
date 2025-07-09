// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google Whitechapel AoC ALSA Driver
 *
 * Copyright (c) 2019 Google LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/string.h>
#include "aoc_alsa_drv.h"
#include "aoc_alsa.h"

#define AOC_ALSA_NAME "aoc_alsa"

/* Driver methods */
static int aoc_alsa_probe(struct aoc_service_dev *dev);
static int aoc_alsa_remove(struct aoc_service_dev *dev);

struct aoc_service_resource {
	const char *name;
	struct aoc_service_dev *dev;
	int ref;
	bool waiting;
	wait_queue_head_t wait_head;
	service_event_cb_t event_callback;
	void *prvdata;
};

/* TODO: audio_haptics should be added, capture1-3 needs to be determined */
static const char *const audio_service_names[] = {
	"audio_output_control",
	"audio_input_control",
	"audio_playback0",
	"audio_playback1",
	"audio_playback2",
	"audio_playback3",
	"audio_playback4",
	"audio_playback5",
	"audio_playback6",
	"audio_haptics",
	"audio_capture0",
	"audio_capture1",
	"audio_capture2",
	"audio_capture3",
	"ultrasonic_capture",
	"audio_voip_rx",
	"audio_voip_tx",
	"audio_incall_pb_0",
	"audio_incall_pb_1",
	"audio_incall_pb_2",
	"audio_incall_cap_0",
	"audio_incall_cap_1",
	"audio_incall_cap_2",
	"decoder_eof",
	"audio_raw",
	"audio_hifiin",
	"audio_hifiout",
	"audio_android_aec",
	"audio_ultrasonic",
	"audio_immersive",
	"audio_capture_inject",
#if IS_ENABLED(CONFIG_SOC_GS201) || IS_ENABLED(CONFIG_SOC_ZUMA)
	"audio_hotword_tap",
#endif
#if IS_ENABLED(CONFIG_SOC_ZUMA)
	"audio_displayport",
	"audio_incall_cap_3",
#endif
	NULL,
};

static struct aoc_service_resource
	service_lists[ARRAY_SIZE(audio_service_names) - 1];

static spinlock_t service_lock;
static int8_t n_services = 0;
static bool drv_registered = false;
static bool aoc_audio_online = false;
static wait_queue_head_t aoc_audio_state_wait_head;

static void compressed_offload_isr(struct aoc_service_dev *dev)
{
	aoc_compr_offload_isr(dev);
}

static void pcm_isr(struct aoc_service_dev *dev)
{
	aoc_pcm_isr(dev);
}

static void voip_isr(struct aoc_service_dev *dev)
{
	aoc_voip_isr(dev);
}

static void incall_hifi_isr(struct aoc_service_dev *dev)
{
	aoc_incall_hifi_isr(dev);
}

static void audio_set_isr(struct aoc_service_dev *dev)
{
	if (dev->mbox_index == PCM_CHANNEL) {
		dev->handler = pcm_isr;
		pr_notice("%s supports interrupt-driven\n", dev_name(&dev->dev));
	} else if (dev->mbox_index == INCALL_CHANNEL || dev->mbox_index == HIFI_CHANNEL) {
		dev->handler = incall_hifi_isr;
		pr_notice("%s supports interrupt-driven\n", dev_name(&dev->dev));
	} else if (dev->mbox_index == VOIP_CHANNEL) {
		dev->handler = voip_isr;
		pr_notice("%s supports interrupt-driven\n", dev_name(&dev->dev));
	}
}

void audio_free_isr(struct aoc_service_dev *dev)
{
	spin_lock(&service_lock);
	if (dev) {
		if (dev->mbox_index == PCM_CHANNEL || dev->mbox_index == INCALL_CHANNEL ||
		    dev->mbox_index == HIFI_CHANNEL || dev->mbox_index == VOIP_CHANNEL) {
			pr_notice("%s free interrupt handler\n", dev_name(&dev->dev));
			dev->handler = NULL;
		}
	}
	spin_unlock(&service_lock);
}
EXPORT_SYMBOL_GPL(audio_free_isr);

int8_t aoc_audio_service_num(void)
{
	return n_services;
}
EXPORT_SYMBOL_GPL(aoc_audio_service_num);

int alloc_aoc_audio_service(const char *name, struct aoc_service_dev **dev, service_event_cb_t cb,
		void *cookies)
{
	int i, err = -EINVAL;

	if (!name || !dev)
		return -EINVAL;

	*dev = NULL;

	spin_lock(&service_lock);

	if (!aoc_audio_online) {
		err = -EPROBE_DEFER;
		goto done;
	}

	for (i = 0; i < ARRAY_SIZE(service_lists); i++) {
		if (strcmp(name, service_lists[i].name) == 0)
			break;
	}

	if (i == ARRAY_SIZE(service_lists))
		goto done;

	/* the AoC is not up yet, retrun -EPROBE_DEFER */
	if (!service_lists[i].dev) {
		err = -EPROBE_DEFER;
		goto done;
	}

	/* Only can be allocated by one client? */
	if (service_lists[i].ref != 0) {
		pr_err("%s has been allocated %d\n", name,
		       service_lists[i].ref);
		goto done;
	}

	audio_set_isr(service_lists[i].dev);

	*dev = service_lists[i].dev;
	service_lists[i].event_callback = cb;
	service_lists[i].prvdata = cookies;
	service_lists[i].ref++;
	err = 0;

done:
	spin_unlock(&service_lock);

	return err;
}
EXPORT_SYMBOL_GPL(alloc_aoc_audio_service);

int free_aoc_audio_service(const char *name, struct aoc_service_dev *dev)
{
	int i, err = -EINVAL;

	if (!name || !dev)
		return -EINVAL;

	spin_lock(&service_lock);
	for (i = 0; i < ARRAY_SIZE(service_lists); i++) {
		/*
		 * In normal case, we can find the service by testing
		 * the address of dev, but in AoC crash case, we should
		 * check the service name as well since the dev might
		 * have been set to NULL
		 */
		if (service_lists[i].dev) {
			if (dev == service_lists[i].dev)
				break;
		} else if (strcmp(name, service_lists[i].name) == 0)
			break;
	}

	if (i == ARRAY_SIZE(service_lists))
		goto done;

	if (service_lists[i].ref <= 0) {
		pr_err("ERR: %s ref = %d abnormal\n", name,
		       service_lists[i].ref);
		goto done;
	}

	if (service_lists[i].dev) {
		service_lists[i].dev->prvdata = NULL;
	}

	service_lists[i].ref--;
	service_lists[i].event_callback = NULL;
	service_lists[i].prvdata = NULL;

	/* Wake up the remove thread if necessary */
	if (service_lists[i].ref == 0 &&
	    service_lists[i].waiting)
		wake_up(&service_lists[i].wait_head);

	err = 0;

done:
	spin_unlock(&service_lock);

	if (err != 0)
		pr_err("ERR: %s can't free audio service\n", name);

	return err;
}
EXPORT_SYMBOL_GPL(free_aoc_audio_service);

__poll_t aoc_audio_state_poll(struct file *f, poll_table *wait,
		struct aoc_state_client_t *client)
{
	if (!f || !wait || !client || !client->inuse)
		return POLLERR;

	poll_wait(f, &aoc_audio_state_wait_head, wait);

	if (client->exit ||
		client->online != aoc_audio_online)
		return EPOLLIN | EPOLLRDNORM;

	return 0;
}
EXPORT_SYMBOL_GPL(aoc_audio_state_poll);

bool aoc_audio_current_state(void)
{
    return aoc_audio_online;
}
EXPORT_SYMBOL_GPL(aoc_audio_current_state);

struct aoc_state_client_t *alloc_audio_state_client(void)
{
	struct aoc_state_client_t *client;

	client = kmalloc(sizeof(struct aoc_state_client_t), GFP_KERNEL);

	if (!client)
		return NULL;

	client->online = aoc_audio_online;
	client->exit = false;
	client->inuse = false;

	return client;
}
EXPORT_SYMBOL_GPL(alloc_audio_state_client);

void free_audio_state_client(struct aoc_state_client_t *client)
{
	kfree(client);
}
EXPORT_SYMBOL_GPL(free_audio_state_client);

static int snd_aoc_alsa_probe(void)
{
	int err;

	err = aoc_pcm_init();
	if (err) {
		pr_err("ERR:fail to init aoc pcm\n");
		goto out;
	}

	err = aoc_voice_init();
	if (err) {
		pr_err("ERR: fail to init aoc voice\n");
		goto out;
	}

	err = aoc_compr_init();
	if (err) {
		pr_err("ERR:%d failed to init aoc compress offload\n", err);
		goto out;
	}

	err = aoc_path_init();
	if (err) {
		pr_err("ERR: fail to init aoc path\n");
		goto out;
	}

	err = aoc_nohost_init();
	if (err) {
		pr_err("ERR: fail to init aoc nohost driver\n");
		goto out;
	}

	err = aoc_incall_init();
	if (err) {
		pr_err("ERR: fail to init aoc incall driver\n");
		goto out;
	}

	err = aoc_voip_init();
	if (err) {
		pr_err("ERR: fail to init aoc voip driver\n");
		goto out;
	}

	err = aoc_usb_init();
	if (err) {
		pr_err("ERR: fail to init aoc usb driver\n");
		goto out;
	}

	err = aoc_dp_init();
	if (err) {
		pr_err("ERR: fail to init aoc dp driver\n");
		goto out;
	}

	return 0;

out:
	return err;
}

static int snd_aoc_alsa_remove(void)
{
	aoc_dp_exit();
	aoc_voip_exit();
	aoc_incall_exit();
	aoc_nohost_exit();
	aoc_path_exit();
	aoc_compr_exit();
	aoc_voice_exit();
	aoc_pcm_exit();

	return 0;
}

static int aoc_alsa_probe(struct aoc_service_dev *adev)
{
	int i;
	int8_t nservices;
	struct device *dev = &adev->dev;

	spin_lock(&service_lock);

	/* Put the aoc service devices in order */
	for (i = 0; i < ARRAY_SIZE(service_lists); i++) {
		if (strcmp(service_lists[i].name, dev_name(dev)) == 0)
			break;
	}

	if (i == ARRAY_SIZE(service_lists)) {
		dev_err(dev, "invalid dev %s", dev_name(dev));
		spin_unlock(&service_lock);
		return -EINVAL;
	}

	service_lists[i].dev = adev;
	service_lists[i].ref = 0;
	service_lists[i].event_callback = NULL;
	service_lists[i].prvdata = NULL;
	service_lists[i].waiting = false;
	dev_notice(dev, "services %d: %s vs. %s\n", n_services,
		  service_lists[i].name, dev_name(dev));


	n_services++;
	nservices = n_services;

	dev_notice(dev, "services number %d vs. reg number %d\n",
		(int)ARRAY_SIZE(service_lists), nservices);

	if (nservices == ARRAY_SIZE(service_lists)) {
		if (!aoc_audio_online) {
			aoc_audio_online = true;
			wake_up(&aoc_audio_state_wait_head);
		}
	}
	spin_unlock(&service_lock);

	if (nservices == ARRAY_SIZE(service_lists) && !drv_registered) {
		drv_registered = true;
		dev_notice(dev, "alsa-aoc communication is ready!\n");
	}

	if (strcmp(dev_name(dev), AOC_COMPR_OFFLOAD_SERVICE) == 0)
		adev->handler = compressed_offload_isr;

	return 0;
}

static int aoc_alsa_remove(struct aoc_service_dev *adev)
{
	int i = 0;
	struct device *dev = &adev->dev;
	spin_lock(&service_lock);

	for (i = 0; i < ARRAY_SIZE(service_lists); i++) {
		if (strcmp(service_lists[i].name, dev_name(dev)) == 0)
			break;
	}

	if (i == ARRAY_SIZE(service_lists)) {
		dev_err(dev, "invalid dev %s\n", dev_name(dev));
		spin_unlock(&service_lock);
		return -EINVAL;
	}

	service_lists[i].dev = NULL;

	if (aoc_audio_online) {
		aoc_audio_online = false;
		wake_up(&aoc_audio_state_wait_head);
	}

	if (service_lists[i].event_callback)
		service_lists[i].event_callback(AOC_SERVICE_EVENT_DOWN, service_lists[i].prvdata);

	if (service_lists[i].ref < 0) {
		dev_warn(dev, "invalid ref %d for %s\n",
			service_lists[i].ref, dev_name(dev));
		service_lists[i].ref = 0;
	}

	/*
	 * All clients have released the resource
	 */
	if (service_lists[i].ref == 0)
		goto done;

	service_lists[i].waiting = true;
	spin_unlock(&service_lock);
	dev_info(dev, "alsa wait %s\n", dev_name(dev));

	/*
	 * We should block the remove function until the ref
	 * is 0, otherwise, it might cause "use after free".
	 */
	wait_event(service_lists[i].wait_head, service_lists[i].ref == 0);

	dev_info(dev, "alsa wait %s done\n", dev_name(dev));
	spin_lock(&service_lock);
	service_lists[i].waiting = false;

done:
	n_services--;

	spin_unlock(&service_lock);
	dev_notice(dev, "remove service %s\n", dev_name(dev));

	return 0;
}

/*TODO: ? */
static void cleanup_resources(void)
{
}

static struct aoc_driver aoc_alsa_driver = {
	.drv = {
		.name = AOC_ALSA_NAME,
	},
	.service_names = audio_service_names,
	.probe = aoc_alsa_probe,
	.remove = aoc_alsa_remove,
};

static int __init aoc_alsa_init(void)
{
	int i;

	pr_debug("aoc alsa driver init\n");

	aoc_audio_online = false;
	drv_registered = false;
	spin_lock_init(&service_lock);

	init_waitqueue_head(&aoc_audio_state_wait_head);

	for (i = 0; i < ARRAY_SIZE(service_lists); i++) {
		service_lists[i].name = audio_service_names[i];
		init_waitqueue_head(&service_lists[i].wait_head);
	}

	snd_aoc_alsa_probe();
	aoc_driver_register(&aoc_alsa_driver);

	pr_debug("aoc alsa driver init done\n");
	return 0;
}

static void __exit aoc_alsa_exit(void)
{
	pr_debug("aoc alsa driver exit\n");

	if (drv_registered) {
		snd_aoc_alsa_remove();
		drv_registered = false;
	}

	aoc_driver_unregister(&aoc_alsa_driver);
	cleanup_resources();
}

module_init(aoc_alsa_init);
module_exit(aoc_alsa_exit);

MODULE_DESCRIPTION("Whitechapel AoC ALSA Driver");
MODULE_AUTHOR("Xinhui Zhou and Carter Hsu (Google)");
MODULE_LICENSE("GPL v2");
