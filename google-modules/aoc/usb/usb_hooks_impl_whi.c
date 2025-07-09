// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2021 Google Corp.
 *
 * Author:
 *  Puma Hsu <pumahsu@google.com>
 */

#include <linux/usb.h>
#include <trace/hooks/sound.h>
#include <trace/hooks/usb.h>

#include "aoc_usb.h"

/*
 * 1: enable AP suspend support
 * 0: disable AP suspend support
 */
#define ap_suspend_enabled 1

/*
 * Return true when the platform supports AP suspend with USB awake.
 * Currently this will respond to Kernel alsa pcm driver.
 */
static void sound_vendor_support_suspend(void *unused, struct usb_device *udev,
					 int direction, bool *is_support)
{
	if (!udev)
		return;

	*is_support = true;
	return;
}

/*
 * Set bypass = 1 will skip USB suspend.
 */
static void usb_vendor_dev_suspend(void *unused, struct usb_device *udev,
				   pm_message_t msg, int *bypass)
{
	bool usb_playback = false;
	bool usb_capture = false;
	int usb_audio_count = xhci_get_usb_audio_count();

	*bypass = 0;

	/* If no USB audio device is connected, we won't skip suspend.*/
	if (!udev || usb_audio_count < 1)
		return;

	usb_playback = aoc_alsa_usb_playback_enabled();
	usb_capture = aoc_alsa_usb_capture_enabled();

	if (usb_playback || usb_capture) {
		dev_info(&udev->dev, "%s: skip suspend process (playback:%d,capture:%d)\n",
			 __func__, usb_playback, usb_capture);
		*bypass = 1;
	}

	return;
}

/*
 * Set bypass = 1 will skip USB resume.
 */
static void usb_vendor_dev_resume(void *unused, struct usb_device *udev,
				  pm_message_t msg, int *bypass)
{
	if (!udev) {
		*bypass = 0;
		return;
	}

	if (udev->port_is_suspended || udev->state == USB_STATE_SUSPENDED) {
		*bypass = 0;
	} else {
		dev_info(&udev->dev, "%s: skip resume process\n", __func__);
		*bypass = 1;
	}

	return;
}

int usb_vendor_helper_init(void)
{
	int ret = 0;

	if (ap_suspend_enabled) {
		pr_info("%s: AP suspend support is enabled\n", __func__);
	} else {
		pr_info("%s: AP suspend support is disabled\n", __func__);
		return ret;
	}

	ret = register_trace_android_vh_sound_usb_support_cpu_suspend(sound_vendor_support_suspend,
								      NULL);
	if (ret)
		pr_err("register_trace_android_vh_sound_usb_support_cpu_suspend failed, ret:%d\n",
		       ret);

	ret = register_trace_android_rvh_usb_dev_suspend(usb_vendor_dev_suspend, NULL);
	if (ret)
		pr_err("register_trace_android_rvh_usb_dev_suspend failed, ret:%d\n", ret);

	ret = register_trace_android_vh_usb_dev_resume(usb_vendor_dev_resume, NULL);
	if (ret)
		pr_err("register_trace_android_vh_usb_dev_resume failed, ret:%d\n", ret);

	return ret;
}
