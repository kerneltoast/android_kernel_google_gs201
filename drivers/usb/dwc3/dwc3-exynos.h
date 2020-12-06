/* SPDX-License-Identifier: GPL-2.0 */
/**
 * dwc3-exynos.h - Samsung EXYNOS DWC3 Specific Glue layer header
 *
 * Copyright (c) 2012 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Author: Anton Tikhomirov <av.tikhomirov@samsung.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2  of
 * the License as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __LINUX_USB_DWC3_EXYNOS_H
#define __LINUX_USB_DWC3_EXYNOS_H

struct dwc3_exynos_rsw {
	struct otg_fsm		*fsm;
	struct work_struct	work;
};

struct dwc3_exynos {
	struct platform_device	*usb2_phy;
	struct platform_device	*usb3_phy;
	struct device		*dev;
	struct dwc3		*dwc;

	struct clk		**clocks;
	struct clk		*bus_clock;

	struct regulator	*vdd33;
	struct regulator	*vdd10;

	struct extcon_dev	*edev;
	struct notifier_block	vbus_nb;
	struct notifier_block	id_nb;

	bool			usb_data_enabled;

	int			idle_ip_index;
	unsigned long		bus_clock_rate;

	struct dwc3_exynos_rsw	rsw;
};

bool dwc3_exynos_rsw_available(struct device *dev);
int dwc3_exynos_rsw_setup(struct device *dev, struct otg_fsm *fsm);
void dwc3_exynos_rsw_exit(struct device *dev);
int dwc3_exynos_rsw_start(struct device *dev);
void dwc3_exynos_rsw_stop(struct device *dev);
int dwc3_exynos_id_event(struct device *dev, int state);
int dwc3_exynos_vbus_event(struct device *dev, bool vbus_active);
int dwc3_exynos_start_ldo(struct device *dev, bool on);
int dwc3_exynos_phy_enable(int owner, bool on);
int dwc3_exynos_get_idle_ip_index(struct device *dev);
extern int dwc3_exynos_set_bus_clock(struct device *dev, int clk_level);
unsigned int of_usb_get_suspend_clk_freq(struct device *dev);
int dwc3_probe(struct platform_device *pdev,
	       struct dwc3_exynos *exynos);
void dwc3_core_exit_mode(struct dwc3 *dwc);
void dwc3_free_event_buffers(struct dwc3 *dwc);
void dwc3_free_scratch_buffers(struct dwc3 *dwc);
int dwc3_otg_phy_enable(struct otg_fsm *fsm, int owner, bool on);
void dwc3_otg_run_sm(struct otg_fsm *fsm);

#endif /* __LINUX_USB_DWC3_EXYNOS_H */

