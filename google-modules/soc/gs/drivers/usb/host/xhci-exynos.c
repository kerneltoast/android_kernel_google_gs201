// SPDX-License-Identifier: GPL-2.0
/*
 * xhci-exynos.c - xHCI host controller driver platform Bus Glue.
 *
 * Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com
 * Author: Sebastian Andrzej Siewior <bigeasy@linutronix.de>
 *
 * A lot of code borrowed from the Linux xHCI driver.
 *
 * Copyright (c) 2021 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 */

#include <core/hub.h> /* $(srctree)/drivers/usb/core/hub.h */
#include <host/xhci.h> /* $(srctree)/drivers/usb/host/xhci.h */
#include <host/xhci-mvebu.h> /* $(srctree)/drivers/usb/host/xhci-mvebu.h */
#include <host/xhci-plat.h> /* $(srctree)/drivers/usb/host/xhci-plat.h */
#include <host/xhci-rcar.h> /* $(srctree)/drivers/usb/host/xhci-rcar.h */

#include <linux/acpi.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/pci.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/usb.h>
#include <linux/usb/dwc3-exynos.h>
#include <linux/usb/of.h>
#include <linux/usb/phy.h>

#include <soc/google/exynos-cpupm.h>
#include <trace/hooks/usb.h>

#include "xhci-exynos.h"

static struct hc_driver xhci_exynos_hc_driver;

/* Callback for bus suspend */
void (*bus_suspend_callback)(void *bus_suspend_payload, bool main_hcd, bool suspend);
void *bus_suspend_payload;

static int xhci_exynos_setup(struct usb_hcd *hcd);
static int xhci_exynos_start(struct usb_hcd *hcd);

static const struct xhci_driver_overrides xhci_exynos_overrides __initconst = {
	.extra_priv_size = sizeof(struct xhci_exynos_priv),
	.reset = xhci_exynos_setup,
	.start = xhci_exynos_start,
};

void register_bus_suspend_callback(void (*callback)(void *bus_suspend_payload, bool main_hcd,
						    bool suspend),
				   void *data)
{
	bus_suspend_callback = callback;
	bus_suspend_payload = data;
}
EXPORT_SYMBOL_GPL(register_bus_suspend_callback);

/*
 * list of VID:PID pair of udevs which are allowed to enter Suspend state regardless of the
 * remote_wakeup bit in their descriptors.
 * Both VID and PID are required in each entry.
 */
static const struct xhci_exynos_udev_ids autosuspend_accessories[] = {
	{
		.vendor = cpu_to_le16(0x18d1),
		.product = cpu_to_le16(0x9480),
	},
	{
		.vendor = cpu_to_le16(0x18d1),
		.product = cpu_to_le16(0x4f60),
	},
	{
		.vendor = cpu_to_le16(0x18d1),
		.product = cpu_to_le16(0x4fa3),
	},
	{ },
};

static bool xhci_exynos_match_udev(const struct xhci_exynos_udev_ids *list, const u16 vid,
				   const u16 pid)
{
	if (list) {
		while (list->vendor && list->product) {
			if (list->vendor == cpu_to_le16(vid) && list->product == cpu_to_le16(pid))
				return true;
			list++;
		}
	}
	return false;
}

static void xhci_exynos_early_stop_set(struct xhci_hcd_exynos *xhci_exynos, struct usb_hcd *hcd)
{
	struct usb_device *rhdev = hcd->self.root_hub;
	struct usb_hub *hub;
	struct usb_port *port_dev;

	if (!rhdev || !rhdev->actconfig || !rhdev->maxchild) {
		dev_info(xhci_exynos->dev, "no rhdev to set early_stop\n");
		return;
	}

	hub = usb_get_intfdata(rhdev->actconfig->interface[0]);

	if (!hub) {
		dev_info(xhci_exynos->dev, "can't get usb_hub\n");
		return;
	}

	port_dev = hub->ports[0];
	port_dev->early_stop = true;

	return;
}

static void xhci_priv_exynos_start(struct usb_hcd *hcd)
{
	struct xhci_exynos_priv *priv = hcd_to_xhci_exynos_priv(hcd);

	if (priv->plat_start)
		priv->plat_start(hcd);
}

static int xhci_priv_init_quirk(struct usb_hcd *hcd)
{
	struct xhci_exynos_priv *priv = hcd_to_xhci_exynos_priv(hcd);

	if (!priv->init_quirk)
		return 0;

	return priv->init_quirk(hcd);
}

static int xhci_priv_resume_quirk(struct usb_hcd *hcd)
{
	struct xhci_exynos_priv *priv = hcd_to_xhci_exynos_priv(hcd);

	if (!priv->resume_quirk)
		return 0;

	return priv->resume_quirk(hcd);
}

static void xhci_exynos_quirks(struct device *dev, struct xhci_hcd *xhci)
{
	struct xhci_exynos_priv *priv = xhci_to_exynos_priv(xhci);

	/*
	 * As of now platform drivers don't provide MSI support so we ensure
	 * here that the generic code does not try to make a pci_dev from our
	 * dev struct in order to setup MSI
	 */
	xhci->quirks |= XHCI_PLAT | priv->quirks;
}

/* called during probe() after chip reset completes */
static int xhci_exynos_setup(struct usb_hcd *hcd)
{
	int ret;

	ret = xhci_priv_init_quirk(hcd);
	if (ret)
		return ret;

	ret = xhci_gen_setup(hcd, xhci_exynos_quirks);

	return ret;
}

static int xhci_exynos_start(struct usb_hcd *hcd)
{
	xhci_priv_exynos_start(hcd);
	return xhci_run(hcd);
}

void xhci_exynos_portsc_power_off(struct xhci_hcd_exynos *exynos, u32 on, u32 prt)
{
	struct xhci_hcd_exynos	*xhci_exynos = exynos;
	void __iomem *portsc = xhci_exynos->usb3_portsc;

	u32 reg;

	spin_lock(&xhci_exynos->xhcioff_lock);

	if (xhci_exynos->portsc_control_priority > prt) {
		spin_unlock(&xhci_exynos->xhcioff_lock);
		return;
	}

	xhci_exynos->portsc_control_priority = prt;

	if (on && !xhci_exynos->port_off_done) {
		dev_info(xhci_exynos->dev, "%s, Do not switch-on port\n", __func__);
		spin_unlock(&xhci_exynos->xhcioff_lock);
		return;
	}

	reg = readl(portsc);

	if (on)
		reg |= PORT_POWER;
	else
		reg &= ~PORT_POWER;

	writel(reg, portsc);

	reg = readl(phycon_base_addr + 0x70);
	if (on)
		reg &= ~DIS_RX_DETECT;
	else
		reg |= DIS_RX_DETECT;

	writel(reg, phycon_base_addr + 0x70);

	if (on)
		xhci_exynos->port_off_done = 0;
	else
		xhci_exynos->port_off_done = 1;

	spin_unlock(&xhci_exynos->xhcioff_lock);
}

int xhci_exynos_port_power_set(struct xhci_hcd_exynos *exynos, u32 on, u32 prt)
{
	struct xhci_hcd_exynos	*xhci_exynos = exynos;

	if (xhci_exynos->usb3_portsc) {
		xhci_exynos_portsc_power_off(xhci_exynos, on, prt);
		return 0;
	}

	dev_info(xhci_exynos->dev, "%s, usb3_portsc is NULL\n", __func__);
	return -EIO;
}
EXPORT_SYMBOL_GPL(xhci_exynos_port_power_set);

/*
 * Query the suspend capability from the USB descriptors
 * @udev: the USB device to be checked
 */
static bool xhci_exynos_allow_suspend_by_descriptor(struct usb_device *udev)
{
	struct usb_interface_descriptor *desc;
	bool allow_suspend = false;
	int i;

	/* Allow suspend if it is a specific accessory */
	if (xhci_exynos_match_udev(autosuspend_accessories, le16_to_cpu(udev->descriptor.idVendor),
				   le16_to_cpu(udev->descriptor.idProduct)))
		return true;

	/* If @udev is a hub, directly check bmAttributes in the descriptor. */
	if (udev->descriptor.bDeviceClass == USB_CLASS_HUB)
		return (udev->config->desc.bmAttributes & USB_CONFIG_ATT_WAKEUP) ? true : false;

	/* Query if there is audio interface and check if it supports remote_wakeup */
	for (i = 0; i < udev->config->desc.bNumInterfaces; i++) {
		desc = &udev->config->intf_cache[i]->altsetting->desc;
		if (desc->bInterfaceClass == USB_CLASS_AUDIO) {
			udev->do_remote_wakeup = (udev->config->desc.bmAttributes &
						  USB_CONFIG_ATT_WAKEUP) ? true : false;
			dev_dbg(&udev->dev, "%s: remote_wakeup = %d\n", __func__,
				udev->do_remote_wakeup);
			if (udev->do_remote_wakeup)
				allow_suspend = true;
			break;
		} else {
			allow_suspend = false;
		}
	}

	return allow_suspend;
}

/*
 * Query the suspend capability of USB device @target_udev after the action taken from @action_udev
 * @target_udev: the USB device to be checked
 * @action_udev: the USB device who takes the action
 * @action: the action taken from @action_udev
 *
 * If @target_udev is a hub, this function doesn't recursively check the child devices under it.
 */
static bool xhci_exynos_allow_suspend_with_action(struct usb_device *target_udev,
						  struct usb_device *action_udev,
						  unsigned long action)
{
	bool allow_suspend = false;

	if (!action_udev || !target_udev)
		return true;

	/* Since @target_udev is being removed, it won't block the bus suspend. Return true.*/
	if (target_udev == action_udev && action == USB_DEVICE_REMOVE)
		return true;

	/*
	 * If @target_udev is a hub, query all child devices. Note that it won't check recursively
	 * if a child device is also a hub.
	 */
	if (target_udev->descriptor.bDeviceClass == USB_CLASS_HUB) {
		struct usb_device *child_udev;
		int port;

		/* Don't support suspend if the hub itself doesn't support remote_wakeup */
		allow_suspend = xhci_exynos_allow_suspend_by_descriptor(target_udev);
		if (!allow_suspend)
			return false;

		/*
		 * Visit all child devices to find any audio device and check remote_wakeup.
		 * If one of them doesn't support remote_wakeup, then we can't support auto-suspend.
		 */
		usb_hub_for_each_child(target_udev, port, child_udev) {
			/*
			 * We don't need to check the descriptor if the child_udev is exactly the
			 * same as @action_dev (the USB device which is doing @action) and it is
			 * removed, Skip to check the rest of ports to know if we can support
			 * suspend or not.
			 */
			if (child_udev && action == USB_DEVICE_REMOVE && action_udev == child_udev)
				continue;

			if (child_udev && child_udev->state == USB_STATE_CONFIGURED) {
				if (!child_udev->config->interface[0])
					return false;

				allow_suspend = xhci_exynos_allow_suspend_by_descriptor(child_udev);
			}

			/* Don't allow if one of the child devices doesn't allow suspend. */
			if (!allow_suspend)
				return false;
		}
	} else {
		allow_suspend = xhci_exynos_allow_suspend_by_descriptor(target_udev);
	}

	return allow_suspend;
}

/*
 * Scan the child devices of @root_hub. Set port_state accordingly and update the suspend flag.
 *
 * @root_hub: the root hub of main hcd or shared hcd
 * @action_udev: the USB device which is doing @action
 * @action: the action taken from @action_udev
 * @suspend: allow suspend or not
 *
 * Note: port_state: the state of the port directly under the root hub.
 */
static void xhci_exynos_scan_roothub(struct xhci_hcd_exynos *xhci_exynos,
				     struct usb_device *root_hub, struct usb_device *action_udev,
				     unsigned long action, bool *suspend)
{
	struct usb_device *child_udev;
	int port;

	usb_hub_for_each_child(root_hub, port, child_udev) {
		if (child_udev && child_udev->state == USB_STATE_CONFIGURED) {
			int bInterfaceClass;

			if (!child_udev->config->interface[0]) {
				*suspend = false;
				break;
			}

			bInterfaceClass = child_udev->config->interface[0]
					->cur_altsetting->desc.bInterfaceClass;

			if (bInterfaceClass == USB_CLASS_HUB)
				xhci_exynos->port_state = PORT_HUB;
			else if (bInterfaceClass == USB_CLASS_BILLBOARD)
				xhci_exynos->port_state = PORT_DP;
			else if (child_udev->speed >= USB_SPEED_SUPER)
				xhci_exynos->port_state = PORT_USB3;
			else
				xhci_exynos->port_state = PORT_USB2;

			/* If still allow suspend so far, check current child_udev */
			if (*suspend)
				*suspend &= xhci_exynos_allow_suspend_with_action(child_udev,
										  action_udev,
										  action);
		} else {
			dev_dbg(&child_udev->dev, "not configured, state: %d\n", child_udev->state);
		}
	}
}

/*
 * While @action_udev is doing @action, scan the USB Bus and set the port_state. Also check the
 * suspend capability of each port (2 levels under root hubs) to decide whether the xhci_exynos
 * wakelocks are going to be held or released.
 */
static int xhci_exynos_check_port(struct xhci_hcd_exynos *xhci_exynos,
				  struct usb_device *action_udev, unsigned long action)
{
	struct usb_device *roothub_main;
	struct usb_device *roothub_shared;
	enum usb_port_state pre_state;
	bool suspend = true;

	if (action_udev->bus->root_hub == action_udev) {
		dev_dbg(&action_udev->dev, "this dev is a root hub\n");
		goto skip;
	}

	pre_state = xhci_exynos->port_state;

	/* clear port_state; it might be set to others when checking the ports under root hubs */
	xhci_exynos->port_state = PORT_EMPTY;

	roothub_main = xhci_exynos->hcd->self.root_hub;
	roothub_shared = xhci_exynos->shared_hcd->self.root_hub;

	/* Check all ports from root hub of main_hcd */
	if (roothub_main)
		xhci_exynos_scan_roothub(xhci_exynos, roothub_main, action_udev, action, &suspend);

	/* Check all ports from root hub of shared_hcd; port_state would override if needed */
	if (roothub_shared)
		xhci_exynos_scan_roothub(xhci_exynos, roothub_shared, action_udev, action,
					 &suspend);

	dev_info(&action_udev->dev, "%s action=%lu state pre=%d now=%d suspend=%u\n", __func__,
		 action, pre_state, xhci_exynos->port_state, suspend);

	/* When @action_udev is added, enable autosuspend of @action_udev if it supports */
	if (action == USB_DEVICE_ADD) {
		if (xhci_exynos_allow_suspend_by_descriptor(action_udev)) {
			dev_dbg(&action_udev->dev, "enable autosuspend on device\n");
			device_init_wakeup(&action_udev->dev, 1);
			usb_enable_autosuspend(action_udev);
		} else if (!suspend) {
			/*
			 * wakelock might be released in previous action. Hold the wakelock if
			 * the system is not allowed to suspend due to *this* action.
			 */
			__pm_stay_awake(xhci_exynos->main_wakelock);
			__pm_stay_awake(xhci_exynos->shared_wakelock);
			dev_dbg(xhci_exynos->dev, "holding wakelock\n");
		}
	}

	/*
	 * Release the wakelock when all (2 levels under root hubs) ports allow to suspend,
	 * so the System is able to suspend too. Here we also check whether the sub-system
	 * supports or not.
	 */
	if (suspend) {
		__pm_relax(xhci_exynos->main_wakelock);
		__pm_relax(xhci_exynos->shared_wakelock);
		dev_dbg(xhci_exynos->dev, "wakelock released\n");
	}

	return xhci_exynos->port_state;

skip:
	return -EINVAL;
}

static void xhci_exynos_set_port(struct usb_device *udev, unsigned long action)
{
	struct usb_hcd *hcd = container_of(udev->bus, struct usb_hcd, self);
	struct xhci_exynos_priv *priv = hcd_to_xhci_exynos_priv(hcd);
	struct xhci_hcd_exynos *xhci_exynos = priv->xhci_exynos;
	struct device *dev = &udev->dev;
	int check_port;

	if (!xhci_exynos) {
		dev_err(dev, "Couldn't get exynos xhci!\n");
		return;
	} else
		udev->dev.platform_data  = xhci_exynos;

	check_port = xhci_exynos_check_port(xhci_exynos, udev, action);
	if (check_port < 0)
		return;

	switch (check_port) {
	case PORT_EMPTY:
		dev_dbg(dev, "Port check empty\n");
		xhci_exynos->is_otg_only = 1;
		if (xhci_exynos->port_ctrl_allowed)
			xhci_exynos_port_power_set(xhci_exynos, 1, 1);
		break;
	case PORT_USB2:
		dev_dbg(dev, "Port check usb2\n");
		xhci_exynos->is_otg_only = 0;
		if (xhci_exynos->port_ctrl_allowed)
			xhci_exynos_port_power_set(xhci_exynos, 0, 1);
		break;
	case PORT_USB3:
		xhci_exynos->is_otg_only = 0;
		dev_dbg(dev, "Port check usb3\n");
		break;
	case PORT_HUB:
		dev_dbg(dev, "Port check hub\n");
		xhci_exynos->is_otg_only = 0;
		break;
	case PORT_DP:
		dev_dbg(dev, "Port check DP\n");
		xhci_exynos->is_otg_only = 0;
		break;
	default:
		break;
	}
}

static int xhci_exynos_power_notify(struct notifier_block *self,
			    unsigned long action, void *dev)
{
	switch (action) {
	case USB_DEVICE_ADD:
		xhci_exynos_set_port(dev, action);
		break;
	case USB_DEVICE_REMOVE:
		xhci_exynos_set_port(dev, action);
		break;
	}
	return NOTIFY_OK;
}

static struct notifier_block dev_nb = {
	.notifier_call = xhci_exynos_power_notify,
};

void xhci_exynos_register_notify(void)
{
	usb_register_notify(&dev_nb);
}

void xhci_exynos_unregister_notify(void)
{
	usb_unregister_notify(&dev_nb);
}

static ssize_t
xhci_exynos_ss_compliance_show(struct device *dev,
			     struct device_attribute *attr,
			     char *buf)
{
	struct xhci_hcd_exynos *xhci_exynos = dev_get_drvdata(dev);
	struct usb_hcd	*hcd = xhci_exynos->hcd;

	u32			reg;
	void __iomem *reg_base;

	reg_base = hcd->regs;
	reg = readl(reg_base + PORTSC_OFFSET);

	return sysfs_emit(buf, "0x%x\n", reg);
}

static ssize_t
xhci_exynos_ss_compliance_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t n)
{
	struct xhci_hcd_exynos *xhci_exynos = dev_get_drvdata(dev);
	struct usb_hcd	*hcd = xhci_exynos->hcd;
	int		value;
	u32			reg;
	void __iomem *reg_base;

	if (kstrtoint(buf, 10, &value))
		return -EINVAL;

	reg_base = hcd->regs;

	if (value == 1) {
		/* PORTSC PLS is set to 10, LWS to 1 */
		reg = readl(reg_base + PORTSC_OFFSET);
		reg &= ~((0xF << 5) | (1 << 16));
		reg |= (10 << 5) | (1 << 16);
		writel(reg, reg_base + PORTSC_OFFSET);
	} else {
		dev_dbg(dev, "Only 1 is allowed for input value\n");
	}

	return n;
}

static DEVICE_ATTR_RW(xhci_exynos_ss_compliance);


static struct attribute *xhci_exynos_attrs[] = {
	&dev_attr_xhci_exynos_ss_compliance.attr,
	NULL
};
ATTRIBUTE_GROUPS(xhci_exynos);

#ifdef CONFIG_OF
static const struct of_device_id usb_xhci_of_match[] = {
	{
		.compatible = "generic-xhci",
	}, {
		.compatible = "xhci-platform",
	},
	{},
};
MODULE_DEVICE_TABLE(of, usb_xhci_of_match);
#endif

static void xhci_exynos_pm_runtime_init(struct device *dev)
{
	dev->power.runtime_status = RPM_SUSPENDED;
	dev->power.idle_notification = false;

	dev->power.disable_depth = 1;
	atomic_set(&dev->power.usage_count, 0);

	dev->power.runtime_error = 0;

	atomic_set(&dev->power.child_count, 0);
	pm_suspend_ignore_children(dev, false);
	dev->power.runtime_auto = true;

	dev->power.request_pending = false;
	dev->power.request = RPM_REQ_NONE;
	dev->power.deferred_resume = false;
	dev->power.accounting_timestamp = jiffies;

	dev->power.timer_expires = 0;
	init_waitqueue_head(&dev->power.wait_queue);
}

static struct xhci_exynos_ops *xhci_vendor_ops;

int xhci_exynos_register_offload_ops(struct xhci_exynos_ops *offload_ops)
{
	if (offload_ops == NULL)
		return -EINVAL;

	xhci_vendor_ops = offload_ops;

	return 0;
}
EXPORT_SYMBOL_GPL(xhci_exynos_register_offload_ops);

static int xhci_vendor_offload_init(struct device *dev, struct xhci_hcd *xhci)
{
	struct xhci_exynos_ops *ops = xhci_vendor_ops;

	if (ops && ops->offload_init)
		return ops->offload_init(xhci);

	dev_err(dev, "Offload hooks or init function is null!\n");
	return -EINVAL;
}

static void xhci_vendor_offload_cleanup(struct device *dev, struct xhci_hcd *xhci)
{
	struct xhci_exynos_ops *ops = xhci_vendor_ops;

	if (ops && ops->offload_cleanup)
		ops->offload_cleanup(xhci);
	else
		dev_err(dev, "Offload hooks or cleanup function is null!\n");
}

static int xhci_vendor_offload_setup(struct device *dev, struct xhci_hcd *xhci)
{
	struct xhci_exynos_ops *ops = xhci_vendor_ops;

	if (ops && ops->offload_setup)
		return ops->offload_setup(xhci);

	dev_err(dev, "Offload hooks or setup function is null!\n");
	return -EINVAL;
}

static int xhci_exynos_probe(struct platform_device *pdev)
{
	struct device		*parent = pdev->dev.parent;
	const struct hc_driver	*driver;
	struct device		*sysdev, *tmpdev;
	struct xhci_hcd		*xhci;
	struct xhci_hcd_exynos	*xhci_exynos;
	struct xhci_exynos_priv *priv;
	struct resource         *res;
	struct usb_hcd		*hcd;
	int			ret;
	int			irq;
	struct wakeup_source	*main_wakelock, *shared_wakelock;

	if (usb_disabled())
		return -ENODEV;

	driver = &xhci_exynos_hc_driver;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	/*
	 * sysdev must point to a device that is known to the system firmware
	 * or PCI hardware. We handle these three cases here:
	 * 1. xhci_exynos comes from firmware
	 * 2. xhci_exynos is child of a device from firmware (dwc3-exynos)
	 * 3. xhci_exynos is grandchild of a pci device (dwc3-pci)
	 */
	for (sysdev = &pdev->dev; sysdev; sysdev = sysdev->parent) {
		if (is_of_node(sysdev->fwnode) ||
			is_acpi_device_node(sysdev->fwnode))
			break;
	}

	if (!sysdev)
		sysdev = &pdev->dev;

	/* Try to set 64-bit DMA first */
	if (WARN_ON(!sysdev->dma_mask))
		/* Platform did not initialize dma_mask */
		ret = dma_coerce_mask_and_coherent(sysdev,
						   DMA_BIT_MASK(64));
	else
		ret = dma_set_mask_and_coherent(sysdev, DMA_BIT_MASK(64));

	/* If seting 64-bit DMA mask fails, fall back to 32-bit DMA mask */
	if (ret) {
		ret = dma_set_mask_and_coherent(sysdev, DMA_BIT_MASK(32));
		if (ret)
			return ret;
	}

	main_wakelock = wakeup_source_register(&pdev->dev, dev_name(&pdev->dev));
	__pm_stay_awake(main_wakelock);
	dev_dbg(&pdev->dev, "holding wakelock\n");

	/* Initialization shared wakelock for SS HCD */
	shared_wakelock = wakeup_source_register(&pdev->dev, dev_name(&pdev->dev));
	__pm_stay_awake(shared_wakelock);

	xhci_exynos_pm_runtime_init(&pdev->dev);

	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);
	pm_runtime_get_noresume(&pdev->dev);

	hcd = __usb_create_hcd(driver, sysdev, &pdev->dev,
			       dev_name(&pdev->dev), NULL);
	if (!hcd) {
		ret = -ENOMEM;
		goto disable_runtime;
	}
	hcd->skip_phy_initialization = 1;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	hcd->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(hcd->regs)) {
		ret = PTR_ERR(hcd->regs);
		goto put_hcd;
	}

	hcd->rsrc_start = res->start;
	hcd->rsrc_len = resource_size(res);

	xhci = hcd_to_xhci(hcd);
	xhci_exynos = devm_kzalloc(&pdev->dev, sizeof(struct xhci_hcd_exynos), GFP_KERNEL);
	xhci_exynos->dev = &pdev->dev;

	xhci_exynos->hcd = (struct usb_hcd *)platform_get_drvdata(pdev);
	platform_set_drvdata(pdev, xhci_exynos);

	spin_lock_init(&xhci_exynos->xhcioff_lock);

	xhci_exynos->usb3_portsc = hcd->regs + PORTSC_OFFSET;
	xhci_exynos->is_otg_only = 1;
	xhci_exynos->port_state = PORT_EMPTY;

	xhci_exynos_register_notify();

	/*
	 * Not all platforms have clks so it is not an error if the
	 * clock do not exist.
	 */
	xhci->reg_clk = devm_clk_get_optional(&pdev->dev, "reg");
	if (IS_ERR(xhci->reg_clk)) {
		ret = PTR_ERR(xhci->reg_clk);
		goto unregister_notify;
	}

	ret = clk_prepare_enable(xhci->reg_clk);
	if (ret)
		goto unregister_notify;

	xhci->clk = devm_clk_get_optional(&pdev->dev, NULL);
	if (IS_ERR(xhci->clk)) {
		ret = PTR_ERR(xhci->clk);
		goto disable_reg_clk;
	}

	ret = clk_prepare_enable(xhci->clk);
	if (ret)
		goto disable_reg_clk;

	priv = hcd_to_xhci_exynos_priv(hcd);
	priv->xhci_exynos = xhci_exynos;

	device_wakeup_enable(hcd->self.controller);

	xhci->main_hcd = hcd;
	xhci->shared_hcd = __usb_create_hcd(driver, sysdev, &pdev->dev,
			dev_name(&pdev->dev), hcd);
	if (!xhci->shared_hcd) {
		ret = -ENOMEM;
		goto disable_clk;
	}
	xhci->shared_hcd->skip_phy_initialization = 1;

	xhci_exynos->shared_hcd = xhci->shared_hcd;

	/* imod_interval is the interrupt moderation value in nanoseconds. */
	xhci->imod_interval = 40000;

	/* Iterate over all parent nodes for finding quirks */
	for (tmpdev = &pdev->dev; tmpdev; tmpdev = tmpdev->parent) {

		if (device_property_read_bool(tmpdev, "usb2-lpm-disable"))
			xhci->quirks |= XHCI_HW_LPM_DISABLE;

		if (device_property_read_bool(tmpdev, "usb3-lpm-capable"))
			xhci->quirks |= XHCI_LPM_SUPPORT;

		if (device_property_read_bool(tmpdev, "quirk-broken-port-ped"))
			xhci->quirks |= XHCI_BROKEN_PORT_PED;

		if (device_property_read_bool(tmpdev, "port_ctrl_allowed"))
			xhci_exynos->port_ctrl_allowed = true;

		device_property_read_u32(tmpdev, "imod-interval-ns",
					 &xhci->imod_interval);
	}

	hcd->usb_phy = devm_usb_get_phy_by_phandle(sysdev, "usb-phy", 0);
	if (IS_ERR(hcd->usb_phy)) {
		ret = PTR_ERR(hcd->usb_phy);
		if (ret == -EPROBE_DEFER)
			goto put_usb3_hcd;
		hcd->usb_phy = NULL;
	} else {
		ret = usb_phy_init(hcd->usb_phy);
		if (ret)
			goto put_usb3_hcd;
	}

	/* Get USB2.0 PHY for main hcd */
	if (parent) {
		xhci_exynos->phy_usb2 = devm_phy_get(parent, "usb2-phy");
		if (IS_ERR_OR_NULL(xhci_exynos->phy_usb2)) {
			xhci_exynos->phy_usb2 = NULL;
			dev_err(&pdev->dev,
				"%s: failed to get phy\n", __func__);
		}
	}

	ret = xhci_vendor_offload_init(&pdev->dev, xhci);
	if (ret)
		goto disable_usb_phy;

	xhci_exynos->main_wakelock = main_wakelock;
	xhci_exynos->shared_wakelock = shared_wakelock;

	hcd->tpl_support = of_usb_host_tpl_support(sysdev->of_node);
	xhci->shared_hcd->tpl_support = hcd->tpl_support;
	ret = usb_add_hcd(hcd, irq, IRQF_SHARED);
	if (ret)
		goto disable_usb_phy;

	if (HCC_MAX_PSA(xhci->hcc_params) >= 4)
		xhci->shared_hcd->can_do_streams = 1;

	ret = usb_add_hcd(xhci->shared_hcd, irq, IRQF_SHARED);
	if (ret)
		goto dealloc_usb2_hcd;

	xhci_exynos_early_stop_set(xhci_exynos, hcd);
	xhci_exynos_early_stop_set(xhci_exynos, xhci->shared_hcd);

	ret = xhci_vendor_offload_setup(&pdev->dev, xhci);
	if (ret)
		goto disable_usb_phy;

	device_enable_async_suspend(&pdev->dev);
	pm_runtime_put_noidle(&pdev->dev);

	device_set_wakeup_enable(&xhci->main_hcd->self.root_hub->dev, 1);
	device_set_wakeup_enable(&xhci->shared_hcd->self.root_hub->dev, 1);

	/*
	 * Prevent runtime pm from being on as default, users should enable
	 * runtime pm using power/control in sysfs.
	 */
	pm_runtime_forbid(&pdev->dev);

	return 0;


dealloc_usb2_hcd:
	usb_remove_hcd(hcd);

disable_usb_phy:
	usb_phy_shutdown(hcd->usb_phy);

put_usb3_hcd:
	usb_put_hcd(xhci->shared_hcd);

disable_clk:
	clk_disable_unprepare(xhci->clk);

disable_reg_clk:
	clk_disable_unprepare(xhci->reg_clk);

unregister_notify:
	xhci_exynos_unregister_notify();

put_hcd:
	usb_put_hcd(hcd);

disable_runtime:
	pm_runtime_put_noidle(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	wakeup_source_unregister(main_wakelock);
	wakeup_source_unregister(shared_wakelock);

	return ret;
}

static int xhci_exynos_remove(struct platform_device *dev)
{
	struct xhci_hcd_exynos *xhci_exynos = platform_get_drvdata(dev);
	struct usb_hcd	*hcd = xhci_exynos->hcd;
	struct xhci_hcd	*xhci = hcd_to_xhci(hcd);
	struct clk *clk = xhci->clk;
	struct clk *reg_clk = xhci->reg_clk;
	struct usb_hcd *shared_hcd = xhci->shared_hcd;
	struct usb_device *rhdev = hcd->self.root_hub;
	struct usb_device *srhdev = shared_hcd->self.root_hub;

	pm_runtime_get_sync(&dev->dev);
	xhci->xhc_state |= XHCI_STATE_REMOVING;

	xhci_exynos_unregister_notify();

	__pm_relax(xhci_exynos->main_wakelock);
	wakeup_source_unregister(xhci_exynos->main_wakelock);

	__pm_relax(xhci_exynos->shared_wakelock);
	wakeup_source_unregister(xhci_exynos->shared_wakelock);

	if (!rhdev || !srhdev)
		goto remove_hcd;

remove_hcd:
	usb_remove_hcd(shared_hcd);
	xhci->shared_hcd = NULL;
	usb_phy_shutdown(hcd->usb_phy);
	usb_remove_hcd(hcd);

	xhci_vendor_offload_cleanup(&dev->dev, xhci);

	devm_iounmap(&dev->dev, hcd->regs);
	usb_put_hcd(shared_hcd);

	clk_disable_unprepare(clk);
	clk_disable_unprepare(reg_clk);
	usb_put_hcd(hcd);

	pm_runtime_disable(&dev->dev);
	pm_runtime_put_noidle(&dev->dev);
	pm_runtime_set_suspended(&dev->dev);

	return 0;
}

static void xhci_exynos_shutdown(struct platform_device *dev)
{
	struct xhci_hcd_exynos *xhci_exynos = platform_get_drvdata(dev);
	struct usb_hcd	*hcd = xhci_exynos->hcd;

	platform_set_drvdata(dev, hcd);
	usb_hcd_platform_shutdown(dev);
	platform_set_drvdata(dev, xhci_exynos);
}

static int __maybe_unused xhci_exynos_suspend(struct device *dev)
{
	struct xhci_hcd_exynos *xhci_exynos = dev_get_drvdata(dev);
	struct usb_hcd	*hcd = xhci_exynos->hcd;
	struct xhci_hcd	*xhci = hcd_to_xhci(hcd);
	int ret = 0;
	int ret_phy = 0;
	int bypass = 0;
	struct usb_device *udev = hcd->self.root_hub;
	pm_message_t msg;

	msg.event = 0;
	trace_android_rvh_usb_dev_suspend(udev, msg, &bypass);
	if (bypass)
		return 0;

	if (xhci_exynos->port_state == PORT_USB2 || (xhci_exynos->port_state == PORT_HUB)) {
		ret_phy = exynos_usbdrd_phy_vendor_set(xhci_exynos->phy_usb2, 1, 0);
		if (ret_phy)
			dev_info(xhci_exynos->dev, "%s: phy vendor set fail, ret:%d\n",
				 __func__, ret_phy);
	}

	/*
	 * xhci_suspend() needs `do_wakeup` to know whether host is allowed
	 * to do wakeup during suspend. Since xhci_exynos_suspend is currently
	 * only designed for system suspend, device_may_wakeup() is enough
	 * to dertermine whether host is allowed to do wakeup. Need to
	 * reconsider this when xhci_exynos_suspend enlarges its scope, e.g.,
	 * also applies to runtime suspend.
	 */

	ret = xhci_suspend(xhci, device_may_wakeup(dev));
	if (ret)
		return ret;

	if (bus_suspend_callback)
		(*bus_suspend_callback)(bus_suspend_payload, hcd == xhci->main_hcd, true);

	return  ret;
}

static int __maybe_unused xhci_exynos_resume(struct device *dev)
{
	struct xhci_hcd_exynos *xhci_exynos = dev_get_drvdata(dev);
	struct usb_hcd	*hcd = xhci_exynos->hcd;
	struct xhci_hcd	*xhci = hcd_to_xhci(hcd);
	int ret = 0;
	int ret_phy = 0;
	int bypass = 0;
	struct usb_device *udev = hcd->self.root_hub;
	pm_message_t msg;

	msg.event = 0;
	trace_android_vh_usb_dev_resume(udev, msg, &bypass);
	if (bypass)
		return 0;

	ret = xhci_priv_resume_quirk(hcd);
	if (ret)
		return ret;

	ret = xhci_resume(xhci, 0);
	if (ret) {
		dev_err(xhci_exynos->dev, "%s: xhci resume failed, ret = %d\n", __func__, ret);
		return ret;
	}

	if (xhci_exynos->port_state == PORT_USB2 || (xhci_exynos->port_state == PORT_HUB)) {
		ret_phy = exynos_usbdrd_phy_vendor_set(xhci_exynos->phy_usb2, 1, 1);
		if (ret_phy)
			dev_info(xhci_exynos->dev, "%s: phy vendor set fail, ret:%d\n",
				 __func__, ret_phy);
	}

	if (bus_suspend_callback)
		(*bus_suspend_callback)(bus_suspend_payload, hcd == xhci->main_hcd, false);

	return ret;
}

static const struct dev_pm_ops xhci_exynos_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(xhci_exynos_suspend, xhci_exynos_resume)

};

static const struct acpi_device_id usb_xhci_acpi_match[] = {
	/* XHCI-compliant USB Controller */
	{ "PNP0D10", },
	{ }
};
MODULE_DEVICE_TABLE(acpi, usb_xhci_acpi_match);

static struct platform_driver usb_xhci_driver = {
	.probe	= xhci_exynos_probe,
	.remove	= xhci_exynos_remove,
	.shutdown = xhci_exynos_shutdown,
	.driver	= {
		.name = "xhci-hcd-exynos",
		.pm = &xhci_exynos_pm_ops,
		.of_match_table = of_match_ptr(usb_xhci_of_match),
		.acpi_match_table = ACPI_PTR(usb_xhci_acpi_match),
		.dev_groups = xhci_exynos_groups,
	},
};
MODULE_ALIAS("platform:xhci-hcd-exynos");

static int __init xhci_exynos_init(void)
{
	xhci_init_driver(&xhci_exynos_hc_driver, &xhci_exynos_overrides);
	return platform_driver_register(&usb_xhci_driver);
}
module_init(xhci_exynos_init);

static void __exit xhci_exynos_exit(void)
{
	platform_driver_unregister(&usb_xhci_driver);
}
module_exit(xhci_exynos_exit);

MODULE_DESCRIPTION("xHCI Exynos Platform Host Controller Driver");
MODULE_LICENSE("GPL");
