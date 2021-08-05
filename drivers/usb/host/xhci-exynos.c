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

#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/usb/phy.h>
#include <linux/slab.h>
#include <linux/phy/phy.h>
#include <linux/acpi.h>
#include <linux/usb/of.h>

#include "../core/phy.h"
#include "xhci.h"
#include "xhci-plat.h"
#include "xhci-mvebu.h"
#include "xhci-rcar.h"
#include "xhci-exynos.h"
#include "../dwc3/dwc3-exynos.h"
#include <soc/google/exynos-cpupm.h>

static struct hc_driver xhci_exynos_hc_driver;

static int xhci_exynos_setup(struct usb_hcd *hcd);
static int xhci_exynos_start(struct usb_hcd *hcd);

static const struct xhci_driver_overrides xhci_exynos_overrides __initconst = {
	.extra_priv_size = sizeof(struct xhci_exynos_priv),
	.reset = xhci_exynos_setup,
	.start = xhci_exynos_start,
	.address_device = xhci_exynos_address_device,
	.bus_suspend = xhci_exynos_bus_suspend,
	.bus_resume = xhci_exynos_bus_resume,
};

int xhci_exynos_address_device(struct usb_hcd *hcd, struct usb_device *udev)
{
	struct xhci_exynos_priv *priv = hcd_to_xhci_exynos_priv(hcd);
	struct xhci_hcd_exynos *xhci_exynos = priv->xhci_exynos;
	int ret;

	ret = xhci_address_device(hcd, udev);
	udev->dev.platform_data  = xhci_exynos;

	return ret;
}

int xhci_exynos_bus_suspend(struct usb_hcd *hcd)
{
	struct xhci_exynos_priv *priv = hcd_to_xhci_exynos_priv(hcd);
	struct xhci_hcd_exynos *xhci_exynos = priv->xhci_exynos;
	struct xhci_hcd *xhci = hcd_to_xhci(hcd);
	int ret, ret_phy, main_hcd;

	if (hcd == xhci->main_hcd)
		main_hcd = 1;
	else
		main_hcd = 0;

	ret = xhci_bus_suspend(hcd);

	if (hcd == xhci->main_hcd &&
	    xhci_exynos->port_state == PORT_USB2) {
		ret_phy = exynos_usbdrd_phy_vendor_set(xhci_exynos->phy_usb2, 1, 0);
		if (ret_phy)
			dev_info(xhci_exynos->dev, "phy vendor set fail\n");
	}

	xhci_exynos_wake_lock(xhci_exynos, main_hcd, 0);

	return ret;
}

int xhci_exynos_bus_resume(struct usb_hcd *hcd)
{
	struct xhci_exynos_priv *priv = hcd_to_xhci_exynos_priv(hcd);
	struct xhci_hcd_exynos *xhci_exynos = priv->xhci_exynos;
	struct xhci_hcd *xhci = hcd_to_xhci(hcd);
	int ret, main_hcd;

	if (hcd == xhci->main_hcd)
		main_hcd = 1;
	else
		main_hcd = 0;

	if (hcd == xhci->main_hcd &&
	    xhci_exynos->port_state == PORT_USB2) {
		ret = exynos_usbdrd_phy_vendor_set(xhci_exynos->phy_usb2, 1, 1);
		if (ret)
			dev_info(xhci_exynos->dev, "phy vendor set fail\n");
	}

	ret = xhci_bus_resume(hcd);

	xhci_exynos_wake_lock(xhci_exynos, main_hcd, 1);

	return ret;
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

static int xhci_exynos_check_port(struct xhci_hcd_exynos *exynos, struct usb_device *dev, bool on)
{
	struct usb_device *hdev;
	struct usb_device *udev = dev;
	struct usb_host_config *config;
	struct usb_interface_descriptor *desc;
	struct device *ddev = &udev->dev;
	struct xhci_hcd_exynos	*xhci_exynos = exynos;
	enum usb_port_state pre_state;
	int usb3_hub_detect = 0;
	int usb2_detect = 0;
	int port, i;
	int bInterfaceClass = 0;

	if (udev->bus->root_hub == udev) {
		dev_dbg(ddev, "this dev is a root hub\n");
		goto skip;
	}

	pre_state = xhci_exynos->port_state;

	/* Find root hub */
	hdev = udev->parent;
	if (!hdev)
		goto skip;

	hdev = dev->bus->root_hub;
	if (!hdev)
		goto skip;
	dev_dbg(ddev, "root hub maxchild = %d\n", hdev->maxchild);

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
				config = udev->config;
				for (i = 0; i < config->desc.bNumInterfaces; i++) {
					desc = &config->intf_cache[i]->altsetting->desc;
					if (desc->bInterfaceClass == USB_CLASS_AUDIO) {
						udev->do_remote_wakeup =
							(udev->config->desc.bmAttributes &
								USB_CONFIG_ATT_WAKEUP) ? 1 : 0;
						if (udev->do_remote_wakeup == 1) {
							device_init_wakeup(ddev, 1);
							usb_enable_autosuspend(dev);
						}
						dev_dbg(ddev, "%s, remote_wakeup = %d\n",
							__func__, udev->do_remote_wakeup);
						break;
					}
				}
			}
			if (bInterfaceClass == USB_CLASS_HUB) {
				xhci_exynos->port_state = PORT_HUB;
				usb3_hub_detect = 1;
				break;
			} else if (bInterfaceClass == USB_CLASS_BILLBOARD) {
				xhci_exynos->port_state = PORT_DP;
				usb3_hub_detect = 1;
				break;
			}

			if (udev->speed >= USB_SPEED_SUPER) {
				xhci_exynos->port_state = PORT_USB3;
				usb3_hub_detect = 1;
				break;
			} else {
				xhci_exynos->port_state = PORT_USB2;
				usb2_detect = 1;
			}
		} else {
			dev_dbg(ddev, "not configured, state = %d\n", udev->state);
		}
	}

	if (!usb3_hub_detect && !usb2_detect)
		xhci_exynos->port_state = PORT_EMPTY;

	dev_dbg(ddev, "%s %s state pre=%d now=%d\n", __func__,
		on ? "on" : "off", pre_state, xhci_exynos->port_state);

	return xhci_exynos->port_state;

skip:
	return -EINVAL;
}

static void xhci_exynos_set_port(struct usb_device *dev, bool on)
{
	struct xhci_hcd_exynos *xhci_exynos = dev_get_platdata(&dev->dev);
	struct device *ddev = &dev->dev;
	int check_port;

	check_port = xhci_exynos_check_port(xhci_exynos, dev, on);
	if (check_port < 0)
		return;

	switch (check_port) {
	case PORT_EMPTY:
		dev_dbg(ddev, "Port check empty\n");
		xhci_exynos->is_otg_only = 1;
		xhci_exynos_port_power_set(xhci_exynos, 1, 1);
		break;
	case PORT_USB2:
		dev_dbg(ddev, "Port check usb2\n");
		xhci_exynos->is_otg_only = 0;
		xhci_exynos_port_power_set(xhci_exynos, 0, 1);
		break;
	case PORT_USB3:
		xhci_exynos->is_otg_only = 0;
		dev_dbg(ddev, "Port check usb3\n");
		break;
	case PORT_HUB:
		dev_dbg(ddev, "Port check hub\n");
		xhci_exynos->is_otg_only = 0;
		break;
	case PORT_DP:
		dev_dbg(ddev, "Port check DP\n");
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
		xhci_exynos_set_port(dev, 1);
		break;
	case USB_DEVICE_REMOVE:
		xhci_exynos_set_port(dev, 0);
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

static struct xhci_plat_priv_overwrite xhci_plat_vendor_overwrite;

int xhci_exynos_register_vendor_ops(struct xhci_vendor_ops *vendor_ops)
{
	if (vendor_ops == NULL)
		return -EINVAL;

	xhci_plat_vendor_overwrite.vendor_ops = vendor_ops;

	return 0;
}
EXPORT_SYMBOL_GPL(xhci_exynos_register_vendor_ops);

static int xhci_vendor_init(struct xhci_hcd *xhci)
{
	struct xhci_vendor_ops *ops = NULL;

	if (xhci_plat_vendor_overwrite.vendor_ops)
		ops = xhci->vendor_ops = xhci_plat_vendor_overwrite.vendor_ops;

	if (ops && ops->vendor_init)
		return ops->vendor_init(xhci);

	return 0;
}

static int xhci_vendor_cleanup(struct xhci_hcd *xhci)
{
	struct xhci_vendor_ops *ops = xhci_vendor_get_ops(xhci);
	int ret = 0;

	if (ops && ops->vendor_cleanup)
		ops->vendor_cleanup(xhci);
	else
		ret = -EOPNOTSUPP;

	xhci->vendor_ops = NULL;
	return ret;
}

int xhci_exynos_wake_lock(struct xhci_hcd_exynos *xhci_exynos,
				   int is_main_hcd, int is_lock)
{
	struct usb_hcd	*hcd = xhci_exynos->hcd;
	int idle_ip_index;
	struct xhci_hcd *xhci = hcd_to_xhci(hcd);

	dev_dbg(xhci_exynos->dev, "%s\n", __func__);

	if (xhci->xhc_state & XHCI_STATE_REMOVING) {
		dev_info(xhci_exynos->dev, "%s - Host removing return!\n",
				__func__);
		return -ESHUTDOWN;
	}

	if (is_lock) {
		if (is_main_hcd) {
			dev_info(xhci_exynos->dev, "%s: Main HCD WAKE LOCK\n", __func__);
			__pm_stay_awake(xhci_exynos->main_wakelock);
		} else {
			dev_info(xhci_exynos->dev, "%s: Shared HCD WAKE LOCK\n", __func__);
			__pm_stay_awake(xhci_exynos->shared_wakelock);
		}
		/* Add a routine for disable IDLEIP (IP idle) */
		dev_info(xhci_exynos->dev, "IDLEIP(SICD) disable.\n");
		idle_ip_index = dwc3_otg_get_idle_ip_index();
		exynos_update_ip_idle_status(idle_ip_index, 0);
	} else {
		if (is_main_hcd) {
			dev_info(xhci_exynos->dev, "%s: Main HCD WAKE UNLOCK\n", __func__);
			__pm_relax(xhci_exynos->main_wakelock);
		} else {
			dev_info(xhci_exynos->dev, "%s: Shared HCD WAKE UNLOCK\n", __func__);
			__pm_relax(xhci_exynos->shared_wakelock);
		}

		/* Add a routine for enable IDLEIP (IP idle) */
		idle_ip_index = dwc3_otg_get_idle_ip_index();
		exynos_update_ip_idle_status(idle_ip_index, 1);
	}

	return 0;
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
		goto put_hcd;
	}

	ret = clk_prepare_enable(xhci->reg_clk);
	if (ret)
		goto put_hcd;

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

	ret = xhci_vendor_init(xhci);
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

put_hcd:
	usb_put_hcd(hcd);

disable_runtime:
	pm_runtime_put_noidle(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

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

	__pm_relax(xhci_exynos->main_wakelock);
	wakeup_source_unregister(xhci_exynos->main_wakelock);

	__pm_relax(xhci_exynos->shared_wakelock);
	wakeup_source_unregister(xhci_exynos->shared_wakelock);

	xhci_exynos_unregister_notify();

	if (!rhdev || !srhdev)
		goto remove_hcd;

remove_hcd:
	/*
	 * We jump to put_hcd here because we move usb_remove_hcd(shared_hcd)
	 * and the following 3 lines to our vendor hook implementation. This
	 * is to fix a race from our audio offload design. Please refer to the
	 * commit message for the detailed information.
	 */
	if (!xhci_vendor_cleanup(xhci))
		goto put_hcd;

	usb_remove_hcd(shared_hcd);
	xhci->shared_hcd = NULL;
	usb_phy_shutdown(hcd->usb_phy);
	usb_remove_hcd(hcd);

put_hcd:
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

extern u32 dwc3_otg_is_connect(void);
static int __maybe_unused xhci_exynos_suspend(struct device *dev)
{
	struct xhci_hcd_exynos *xhci_exynos = dev_get_drvdata(dev);
	struct usb_hcd	*hcd = xhci_exynos->hcd;
	struct xhci_hcd	*xhci = hcd_to_xhci(hcd);
	int ret;

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

	return  ret;
}

static int __maybe_unused xhci_exynos_resume(struct device *dev)
{
	struct xhci_hcd_exynos *xhci_exynos = dev_get_drvdata(dev);
	struct usb_hcd	*hcd = xhci_exynos->hcd;
	struct xhci_hcd	*xhci = hcd_to_xhci(hcd);
	int ret;

	ret = xhci_priv_resume_quirk(hcd);
	if (ret)
		return ret;

	return xhci_resume(xhci, 0);
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
