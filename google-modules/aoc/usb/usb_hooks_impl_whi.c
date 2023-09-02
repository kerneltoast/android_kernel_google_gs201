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
 * This returns the enum of device/hub info.
 * Currently we only distinguish one layer usb device, so it only
 * has usb1~3, and usb1-1/usb2-1/usb3-1. For the other cases we
 * return UNDEFINED.
 * TODO(b/199883028): optimize this function to be flexible.
 */
static int get_usb_dev_hub_info(struct usb_device *udev)
{
	enum usb_dev_hub usb_dev_hub = UNDEFINED;

	if (!udev)
		return usb_dev_hub;

	if (!udev->parent) {
		/* hub */
		switch (udev->bus->busnum) {
		case 1:
			usb_dev_hub = USB1;
			break;
		case 2:
			usb_dev_hub = USB2;
			break;
		case 3:
			usb_dev_hub = USB3;
			break;
		default:
			usb_dev_hub = UNDEFINED;
			break;
		}
	} else {
		/* device */
		switch (udev->bus->busnum) {
		case 1:
			if (udev->portnum == 1)
				usb_dev_hub = USB1_1;
			break;
		case 2:
			if (udev->portnum == 1)
				usb_dev_hub = USB2_1;
			break;
		case 3:
			if (udev->portnum == 1)
				usb_dev_hub = USB3_1;
			break;
		default:
			usb_dev_hub = UNDEFINED;
			break;
		}
	}

	return usb_dev_hub;
}

/*
 * Set bypass = 1 will skip USB suspend.
 */
static void usb_vendor_dev_suspend(void *unused, struct usb_device *udev,
				   pm_message_t msg, int *bypass)
{
	struct xhci_hcd *xhci;
	int usb_audio_count = 0;
	bool usb_playback = false;
	bool usb_capture = false;
	enum usb_dev_hub usb_dev_hub;

	*bypass = 0;

	if (!udev)
		return;

	usb_dev_hub = get_usb_dev_hub_info(udev);

	/*
	 * We don't change dummy_hcd's behavior, the dummy_hcd is in bus 1.
	 * We don't change undefined device neither.
	 */
	if (usb_dev_hub >= USB1 && usb_dev_hub <= USB1_1)
		return;

	xhci = get_xhci_hcd_by_udev(udev);
	if (!xhci) {
		dev_err(&udev->dev, "%s: couldn't get xhci\n", __func__);
		return;
	}

	usb_audio_count = xhci_get_usb_audio_count(xhci);

	if (usb_audio_count > 0) {
		/*
		 * We query playback/capture states only when there is USB
		 * audio device connected.
		 */
		usb_playback = aoc_alsa_usb_playback_enabled();
		usb_capture = aoc_alsa_usb_capture_enabled();
	} else {
		/* If no USB audio device is connected, we won't skip suspend. */
		return;
	}

	/*
	 * Note: Currently we also allow the UNDEFINED case go to check
	 * playback/capture state. For example, the headset behind hub
	 * may be USB2-1.1
	 */
	if ((usb_dev_hub >= USB2 && usb_dev_hub <= USB3_1) || usb_dev_hub == UNDEFINED) {
		if (usb_playback || usb_capture) {
			dev_info(&udev->dev, "%s: skip suspend process (playback:%d,capture:%d)\n",
				 __func__, usb_playback, usb_capture);
			*bypass = 1;
		}
	}

	return;
}

/*
 * Set bypass = 1 will skip USB resume.
 */
static void usb_vendor_dev_resume(void *unused, struct usb_device *udev,
				  pm_message_t msg, int *bypass)
{
	enum usb_dev_hub usb_dev_hub;

	if (!udev) {
		*bypass = 0;
		return;
	}

	usb_dev_hub = get_usb_dev_hub_info(udev);

	/*
	 * We don't change dummy_hcd's behavior, the dummy_hcd is in bus 1.
	 * We don't change undefined device neither.
	 */
	if (usb_dev_hub <= USB1_1) {
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

	ret = register_trace_android_vh_usb_dev_suspend(usb_vendor_dev_suspend, NULL);
	if (ret)
		pr_err("register_trace_android_vh_usb_dev_suspend failed, ret:%d\n", ret);

	ret = register_trace_android_vh_usb_dev_resume(usb_vendor_dev_resume, NULL);
	if (ret)
		pr_err("register_trace_android_vh_usb_dev_resume failed, ret:%d\n", ret);

	return ret;
}
