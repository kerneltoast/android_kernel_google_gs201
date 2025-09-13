// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2022 Google Corp.
 *
 * Author:
 *  Howard.Yen <howardyen@google.com>
 */

#include <sound/pcm.h>
#include <uapi/sound/asound.h>
#include "usbaudio.h"
#include "card.h"
#include "xhci-plat.h"

#include "aoc_usb.h"

 static int snd_usb_audio_vendor_connect(struct usb_interface *intf)
{
	struct usb_device *udev;
	struct xhci_hcd *xhci;

	if (!intf) {
		pr_err("%s: Invalid parameter\n", __func__);
		return -EINVAL;
	}

	udev = interface_to_usbdev(intf);
	xhci = get_xhci_hcd_by_udev(udev);

	xhci_set_offload_state(xhci, true);

	return 0;
}

static void snd_usb_audio_vendor_disconnect(struct usb_interface *intf)
{
	struct usb_device *udev;
	struct xhci_hcd *xhci;

	if (!intf) {
		pr_err("%s: Invalid parameter\n", __func__);
		return;
	}

	udev = interface_to_usbdev(intf);
	xhci = get_xhci_hcd_by_udev(udev);

	xhci_set_offload_state(xhci, false);

	return;
}

static int snd_usb_audio_vendor_set_interface(struct usb_device *udev,
					      struct usb_host_interface *alts,
					      int iface, int alt)
{
	return 0;
}

static int snd_usb_audio_vendor_set_rate(struct usb_interface *intf, int iface, int rate,
					 int alt)
{
	return 0;
}

static int snd_usb_audio_vendor_set_pcm_buf(struct usb_device *udev, int iface)
{
	return 0;
}

static int snd_usb_audio_vendor_set_pcm_intf(struct usb_interface *intf, int iface, int alt,
					     int direction)
{
	struct usb_device *udev;
	struct xhci_hcd *xhci;
	struct xhci_vendor_data *vendor_data;

	if (!intf) {
		pr_err("%s: Invalid parameter\n", __func__);
		return 0;
	}

	udev = interface_to_usbdev(intf);
	xhci = get_xhci_hcd_by_udev(udev);
	vendor_data = xhci_to_priv(xhci)->vendor_data;

	if (vendor_data->offload_state) {
		xhci_dbg(xhci, "offloading is enabled\n");
		return 0;
	}

	xhci_set_offload_state(xhci, true);
	return 0;
}

static int snd_usb_audio_vendor_set_pcm_connection(struct usb_device *udev,
						   enum snd_vendor_pcm_open_close onoff,
						   int direction)
{
	return 0;
}

static int snd_usb_audio_vendor_set_pcm_binterval(struct audioformat *fp,
						  struct audioformat *found,
						  int *cur_attr, int *attr)
{
	return 0;
}

static int snd_usb_audio_vendor_usb_add_ctls(struct snd_usb_audio *chip)
{
	return 0;
}

static struct snd_usb_audio_vendor_ops snd_usb_ops = {
	.connect = snd_usb_audio_vendor_connect,
	.disconnect = snd_usb_audio_vendor_disconnect,
	.set_interface = snd_usb_audio_vendor_set_interface,
	.set_rate = snd_usb_audio_vendor_set_rate,
	.set_pcm_buf = snd_usb_audio_vendor_set_pcm_buf,
	.set_pcm_intf = snd_usb_audio_vendor_set_pcm_intf,
	.set_pcm_connection = snd_usb_audio_vendor_set_pcm_connection,
	.set_pcm_binterval = snd_usb_audio_vendor_set_pcm_binterval,
	.usb_add_ctls = snd_usb_audio_vendor_usb_add_ctls,
};

int snd_usb_audio_vendor_helper_init(void)
{
	return snd_vendor_set_ops(&snd_usb_ops);
}
