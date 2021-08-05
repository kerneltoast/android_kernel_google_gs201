/* SPDX-License-Identifier: GPL-2.0 */
/*
 * xhci-exynos.h - xHCI host controller driver platform Bus Glue.
 *
 * Copyright (c) 2021 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 */

#ifndef _XHCI_EXYNOS_H
#define _XHCI_EXYNOS_H

#include "xhci.h"	/* for hcd_to_xhci() */

#define PORTSC_OFFSET		0x430
#define DIS_RX_DETECT		BIT(9)
#define USB_CLASS_BILLBOARD	0x11

enum usb_port_state {
	PORT_EMPTY = 0,		/* OTG only */
	PORT_USB2,		/* usb 2.0 device only */
	PORT_USB3,		/* usb 3.0 device only */
	PORT_HUB,		/* usb hub single */
	PORT_DP			/* DP device */
};

struct xhci_hcd_exynos {
	struct device 		*dev;
	struct usb_hcd 		*hcd;
	struct usb_hcd 		*shared_hcd;
	struct phy		*phy_usb2;

	struct wakeup_source	*main_wakelock;
	struct wakeup_source	*shared_wakelock;

	void __iomem		*usb3_portsc;
	spinlock_t 		xhcioff_lock;
	int 			is_otg_only;
	int 			port_off_done;
	int 			port_set_delayed;
	u32 			portsc_control_priority;
	enum usb_port_state	port_state;
};

struct xhci_exynos_priv {
	const char *firmware_name;
	unsigned long long quirks;
	struct xhci_vendor_data *vendor_data;
	int (*plat_setup)(struct usb_hcd *);
	void (*plat_start)(struct usb_hcd *);
	int (*init_quirk)(struct usb_hcd *);
	int (*suspend_quirk)(struct usb_hcd *);
	int (*resume_quirk)(struct usb_hcd *);
	struct xhci_hcd_exynos *xhci_exynos;
};

#define hcd_to_xhci_exynos_priv(h) ((struct xhci_exynos_priv *)hcd_to_xhci(h)->priv)
#define xhci_to_exynos_priv(x) ((struct xhci_exynos_priv *)(x)->priv)

extern void __iomem *phycon_base_addr;
extern int exynos_usbdrd_phy_vendor_set(struct phy *phy, int is_enable,
					int is_cancel);
extern int dwc3_otg_get_idle_ip_index(void);
void xhci_exynos_register_notify(void);
void xhci_exynos_unregister_notify(void);
static int xhci_exynos_address_device(struct usb_hcd *hcd, struct usb_device *udev);
static int xhci_exynos_bus_suspend(struct usb_hcd *hcd);
static int xhci_exynos_bus_resume(struct usb_hcd *hcd);
static int xhci_exynos_wake_lock(struct xhci_hcd_exynos *xhci_exynos,
				 int is_main_hcd, int is_lock);

#endif	/* _XHCI_EXYNOS_H */
