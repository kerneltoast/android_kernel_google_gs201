// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2015-2017 Samsung Electronics Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

 /* usb_power_notify.c */

#define pr_fmt(fmt) "usb_notify: " fmt

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/bitops.h>
#include <linux/err.h>
#include <linux/usb.h>
#include <linux/notifier.h>
#include <linux/version.h>
#include "usb_power_notify.h"
#include "../core/hub.h"
#include "core.h"
#include "otg.h"

int port_off_done;
EXPORT_SYMBOL_GPL(port_off_done);
u32 pp_set_delayed;
EXPORT_SYMBOL_GPL(pp_set_delayed);
u32 portsc_control_priority;
EXPORT_SYMBOL_GPL(portsc_control_priority);
spinlock_t xhcioff_lock; /* protect portsc off setting */
EXPORT_SYMBOL_GPL(xhcioff_lock);

enum usb_port_state {
	PORT_EMPTY = 0,		/* OTG only */
	PORT_USB2,		/* usb 2.0 device only */
	PORT_USB3,		/* usb 3.0 device only */
	PORT_HUB,		/* usb hub single */
	PORT_DP			/* DP device */
};

#define DIS_RX_DETECT		BIT(9)
#define USB_CLASS_BILLBOARD	0x11
#define PORT_POWER		BIT(9)

static enum usb_port_state port_state;
int is_otg_only;

void xhci_portsc_power_off(void __iomem *portsc, u32 on, u32 prt)
{
	u32 reg;

	spin_lock(&xhcioff_lock);

	pr_info("%s, on=%d portsc_control_priority=%d, prt=%d\n",
		__func__, on,  portsc_control_priority, prt);

	if (portsc_control_priority > prt) {
		spin_unlock(&xhcioff_lock);
		return;
	}

	portsc_control_priority = prt;

	if (on && !port_off_done) {
		pr_info("%s, Do not switch-on port\n", __func__);
		spin_unlock(&xhcioff_lock);
		return;
	}

	reg = readl(portsc);

	if (on)
		reg |= PORT_POWER;
	else
		reg &= ~PORT_POWER;

	writel(reg, portsc);
	reg = readl(portsc);

	pr_info("power %s portsc, reg = 0x%x addr = %8x\n",
		on ? "on" : "off", reg, (u64)portsc);

	reg = readl(phycon_base_addr + 0x70);
	if (on)
		reg &= ~DIS_RX_DETECT;
	else
		reg |= DIS_RX_DETECT;

	writel(reg, phycon_base_addr + 0x70);

	if (on)
		port_off_done = 0;
	else
		port_off_done = 1;

	pr_info("phycon ess_ctrl = 0x%x\n", readl(phycon_base_addr + 0x70));

	spin_unlock(&xhcioff_lock);
}
EXPORT_SYMBOL_GPL(xhci_portsc_power_off);

int xhci_portsc_set(u32 on)
{
	if (usb3_portsc && !on) {
		xhci_portsc_power_off(usb3_portsc, 0, 2);
		pp_set_delayed = 0;
		return 0;
	}

	if (!on)
		pp_set_delayed = 1;

	pr_info("%s, usb3_portsc is NULL\n", __func__);
	return -EIO;
}
EXPORT_SYMBOL(xhci_portsc_set);

int xhci_port_power_set(u32 on, u32 prt)
{
	if (usb3_portsc) {
		xhci_portsc_power_off(usb3_portsc, on, prt);
		return 0;
	}

	pr_info("%s, usb3_portsc is NULL\n", __func__);
	return -EIO;
}
EXPORT_SYMBOL_GPL(xhci_port_power_set);

static int check_usb3_hub(struct usb_device *dev, bool on)
{
	struct usb_device *hdev;
	struct usb_device *udev = dev;
	struct device *ddev = &udev->dev;
	enum usb_port_state pre_state;
	int usb3_hub_detect = 0;
	int usb2_detect = 0;
	int port;
	int bInterfaceClass = 0;

	if (udev->bus->root_hub == udev) {
		pr_info("this dev is a root hub\n");
		goto skip;
	}

	pre_state = port_state;

	/* Find root hub */
	hdev = udev->parent;
	if (!hdev)
		goto skip;

	hdev = dev->bus->root_hub;
	if (!hdev)
		goto skip;
	pr_info("root hub maxchild = %d\n", hdev->maxchild);

	/* check all ports */
	usb_hub_for_each_child(hdev, port, udev) {
		dev_dbg(ddev, "%s, class = %d, speed = %d\n",
			__func__, udev->descriptor.bDeviceClass,
						udev->speed);
		dev_dbg(ddev, "udev = 0x%8x, state = %d\n", udev, udev->state);
		if (udev && udev->state == USB_STATE_CONFIGURED) {
			if (!dev->config->interface[0])
				continue;

			bInterfaceClass	= udev->config->interface[0]
					->cur_altsetting->desc.bInterfaceClass;
			if (on) {
				if (bInterfaceClass == USB_CLASS_HID ||
				    bInterfaceClass == USB_CLASS_AUDIO) {
					udev->do_remote_wakeup =
						(udev->config->desc.bmAttributes &
							USB_CONFIG_ATT_WAKEUP) ? 1 : 0;
					if (udev->do_remote_wakeup == 1) {
						device_init_wakeup(ddev, 1);
						usb_enable_autosuspend(dev);
					}
					dev_dbg(ddev, "%s, remote_wakeup = %d\n",
						__func__, udev->do_remote_wakeup);
				}
			}
			if (bInterfaceClass == USB_CLASS_HUB) {
				port_state = PORT_HUB;
				usb3_hub_detect = 1;
				break;
			} else if (bInterfaceClass == USB_CLASS_BILLBOARD) {
				port_state = PORT_DP;
				usb3_hub_detect = 1;
				break;
			}

			if (udev->speed >= USB_SPEED_SUPER) {
				port_state = PORT_USB3;
				usb3_hub_detect = 1;
				break;
			}
			port_state = PORT_USB2;
			usb2_detect = 1;
		} else {
			pr_info("not configured, state = %d\n",
				udev->state);
			port_state = PORT_USB3;
			usb3_hub_detect = 1;
		}
	}

	if (!usb3_hub_detect && !usb2_detect)
		port_state = PORT_EMPTY;

	pr_info("%s %s state pre=%d now=%d\n", __func__,
		on ? "on" : "off", pre_state, port_state);

	return port_state;

skip:
	return -EINVAL;
}

static void set_usb3_port_power(struct usb_device *dev, bool on)
{
	//cancel_delayed_work_sync(&g_dwc->usb_qos_lock_delayed_work);

	switch (check_usb3_hub(dev, on)) {
	case PORT_EMPTY:
		pr_info("Port check empty\n");
		is_otg_only = 1;
		//usb_power_notify_control(0, 1);
		xhci_port_power_set(1, 1);
		//dwc3_otg_qos_lock(g_dwc, 1);
		break;
	case PORT_USB2:
		pr_info("Port check usb2\n");
		is_otg_only = 0;
		xhci_port_power_set(0, 1);
		//usb_power_notify_control(0, 0);
		//schedule_delayed_work(&g_dwc->usb_qos_lock_delayed_work,
		//		msecs_to_jiffies(USB_BUS_CLOCK_DELAY_MS));
		break;
	case PORT_USB3:
		/* xhci_port_power_set(1, 1); */
		is_otg_only = 0;
		pr_info("Port check usb3\n");
		break;
	case PORT_HUB:
		/*xhci_port_power_set(1, 1);*/
		pr_info("Port check hub\n");
		is_otg_only = 0;
		break;
	case PORT_DP:
		/*xhci_port_power_set(1, 1);*/
		pr_info("Port check DP\n");
		is_otg_only = 0;
		break;
	default:
		break;
	}
}

static int usb_power_notify(struct notifier_block *self,
			    unsigned long action, void *dev)
{
	switch (action) {
	case USB_DEVICE_ADD:
		set_usb3_port_power(dev, 1);
		break;
	case USB_DEVICE_REMOVE:
		set_usb3_port_power(dev, 0);
		break;
	}
	return NOTIFY_OK;
}

static struct notifier_block dev_nb = {
	.notifier_call = usb_power_notify,
};

void register_usb_power_notify(void)
{
	is_otg_only = 1;
	port_state = PORT_EMPTY;
	usb_register_notify(&dev_nb);
}
EXPORT_SYMBOL(register_usb_power_notify);

void unregister_usb_power_notify(void)
{
	usb_unregister_notify(&dev_nb);
}
EXPORT_SYMBOL(unregister_usb_power_notify);
