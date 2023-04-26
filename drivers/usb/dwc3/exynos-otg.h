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
#include <linux/pm_wakeup.h>
#include <linux/usb/otg-fsm.h>
#include <linux/pm_qos.h>
#include <misc/gvotable.h>
#include <soc/google/exynos_pm_qos.h>
#include <linux/power_supply.h>
#include "dwc3-exynos.h"


struct dwc3_ext_otg_ops {
	int	(*setup)(struct device *dev, struct otg_fsm *fsm);
	void	(*exit)(struct device *dev);
	int	(*start)(struct device *dev);
	void	(*stop)(struct device *dev);
};

/**
 * struct dwc3_otg: OTG driver data. Shared by HCD and DCD.
 * @otg: USB OTG Transceiver structure.
 * @fsm: OTG Final State Machine.
 * @dwc: pointer to our controller context structure.
 * @irq: IRQ number assigned for HSUSB controller.
 * @regs: ioremapped register base address.
 * @wakelock: prevents the system from entering suspend while
 *		host or peripheral mode is active.
 * @vbus_reg: Vbus regulator.
 * @ready: is one when OTG is ready for operation.
 * @ext_otg_ops: external OTG engine ops.
 */
struct dwc3_otg {
	struct usb_otg          otg;
	struct otg_fsm		fsm;
	struct dwc3             *dwc;
	struct dwc3_exynos      *exynos;
	int                     irq;
	void __iomem            *regs;
	struct wakeup_source	*wakelock;
	struct wakeup_source	*reconn_wakelock;

	unsigned		ready:1;
	int			otg_connection;

	struct regulator	*vbus_reg;

	struct exynos_pm_qos_request	pm_qos_int_req;
	int				pm_qos_int_val;

	struct dwc3_ext_otg_ops *ext_otg_ops;

	struct work_struct	recov_work;

	struct notifier_block	pm_nb;
	struct notifier_block	psy_notifier;
	struct completion	resume_cmpl;
	int			dwc3_suspended;
	int			fsm_reset;
	int			in_shutdown;
	bool			skip_retry;

	struct mutex lock;
	u16 combo_phy_control;
	u16 usb2_phy_control;

	struct gvotable_election *ssphy_restart_votable;
};

static inline int dwc3_ext_otg_setup(struct dwc3_otg *dotg)
{
	struct device *dev = dotg->exynos->dev;

	if (!dotg->ext_otg_ops->setup)
		return -EOPNOTSUPP;
	return dotg->ext_otg_ops->setup(dev, &dotg->fsm);
}

static inline int dwc3_ext_otg_exit(struct dwc3_otg *dotg)
{
	struct device *dev = dotg->exynos->dev;

	if (!dotg->ext_otg_ops->exit)
		return -EOPNOTSUPP;
	dotg->ext_otg_ops->exit(dev);
	return 0;
}

static inline int dwc3_ext_otg_start(struct dwc3_otg *dotg)
{
	struct device *dev = dotg->exynos->dev;

	pr_info("%s\n", __func__);

	if (!dotg->ext_otg_ops->start)
		return -EOPNOTSUPP;
	return dotg->ext_otg_ops->start(dev);
}

static inline int dwc3_ext_otg_stop(struct dwc3_otg *dotg)
{
	struct device *dev = dotg->exynos->dev;

	if (!dotg->ext_otg_ops->stop)
		return -EOPNOTSUPP;
	dotg->ext_otg_ops->stop(dev);
	return 0;
}

int dwc3_exynos_otg_init(struct dwc3 *dwc, struct dwc3_exynos *exynos);
void dwc3_exynos_otg_exit(struct dwc3 *dwc, struct dwc3_exynos *exynos);
int dwc3_otg_start(struct dwc3 *dwc, struct dwc3_exynos *exynos);
void dwc3_otg_stop(struct dwc3 *dwc, struct dwc3_exynos *exynos);
int dwc3_otg_usb_recovery_reconn(struct dwc3_exynos *exynos);
bool dwc3_otg_check_usb_suspend(struct dwc3_exynos *exynos);
bool dwc3_otg_check_usb_activity(struct dwc3_exynos *exynos);

extern void __iomem *phycon_base_addr;
extern int exynos_usbdrd_pipe3_enable(struct phy *phy);
extern int exynos_usbdrd_pipe3_disable(struct phy *phy);

#endif /* __LINUX_USB_DWC3_OTG_H */
