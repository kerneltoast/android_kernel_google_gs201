/* SPDX-License-Identifier: GPL-2.0 */
/*
 * exynos-otg.h - Samsung EXYNOS OTG Header
 *
 * Copyright (c) 2021 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 */

#ifndef __LINUX_USB_DWC3_OTG_H
#define __LINUX_USB_DWC3_OTG_H
#include <linux/pm_qos.h>
#include <linux/pm_wakeup.h>
#include <linux/power_supply.h>
#include <linux/usb/role.h>

#include <misc/gvotable.h>
#include <soc/google/exynos_pm_qos.h>

/**
 * struct dwc3_otg: OTG driver data. Shared by HCD and DCD.
 * @dwc: pointer to our controller context structure.
 * @wakelock: prevents the system from entering suspend while
 *		host or peripheral mode is active.
 */
struct dwc3_otg {
	struct dwc3             *dwc;
	struct dwc3_exynos      *exynos;
	struct wakeup_source	*wakelock;

	bool			host_on;
	bool			device_on;
	bool			host_ready;
	enum usb_role		current_role;
	/* New data role that is updated before the data role change is executed */
	enum usb_role		desired_role;
	struct kernfs_node	*desired_role_kn;

	int			otg_connection;

	struct exynos_pm_qos_request	pm_qos_int_req;
	int				pm_qos_int_val;

	struct work_struct	work;

	struct notifier_block	pm_nb;
	struct completion	resume_cmpl;
	int			dwc3_suspended;
	int			in_shutdown;
	bool			usb_charged;

	struct mutex lock;
	struct mutex		role_lock;

	struct gvotable_election *ssphy_restart_votable;
	struct gvotable_election *usbdp_tca_votable;
};

int dwc3_exynos_otg_init(struct dwc3 *dwc, struct dwc3_exynos *exynos);
void dwc3_exynos_otg_exit(struct dwc3 *dwc, struct dwc3_exynos *exynos);
bool dwc3_otg_check_usb_suspend(struct dwc3_exynos *exynos);
int dwc3_otg_start_host(struct dwc3_otg *dotg, int on);
int dwc3_otg_start_gadget(struct dwc3_otg *dotg, int on);
enum usb_role dwc3_exynos_wait_role(struct dwc3_otg *dotg);
void dwc3_exynos_set_role(struct dwc3_otg *dotg);

extern void __iomem *phycon_base_addr;
extern int exynos_usbdrd_pipe3_enable(struct phy *phy);
extern int exynos_usbdrd_pipe3_disable(struct phy *phy);

#endif /* __LINUX_USB_DWC3_OTG_H */
