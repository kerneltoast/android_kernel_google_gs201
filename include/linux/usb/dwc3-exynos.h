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

#include <dwc3/core.h> /* $(srctree)/drivers/usb/dwc3/core.h */

/* Exynos Specific Register Definition */

/* LINK Registers */
#define DWC3_LU3LFPSRXTIM	0xd010
#define DWC3_LSKIPFREQ		0xd020
#define DWC3_LLUCTL		0xd024
#define DWC3_BU31RHBDBG		0xd800

/* Link Register - LLUCTL */
#define DWC3_PENDING_HP_TIMER_US(n)	((n) << 16)
#define DWC3_LLUCTL_LTSSM_TIMER_OVRRD	BIT(23)
#define DWC3_EN_US_HP_TIMER		BIT(15)
#define DWC3_LLUCTL_PIPE_RESET		BIT(7)
#define DWC3_FORCE_GEN1			BIT(10)
#define DWC3_LLUCTL_TX_TS1_CNT(n)	((n) << 0)
#define DWC3_LLUCTL_TX_TS1_CNT_MASK	DWC3_LLUCTL_TX_TS1_CNT(0x1f)

/* Link Register - LSKIPFREQ */
#define DWC3_PM_ENTRY_TIMER_US(n)	((n) << 20)
#define DWC3_PM_ENTRY_TIMER_US_MASK	DWC3_PM_ENTRY_TIMER_US(0xf)
#define DWC3_PM_LC_TIMER_US(n)		((n) << 24)
#define DWC3_PM_LC_TIMER_US_MASK	DWC3_PM_LC_TIMER_US(0x7)
#define DWC3_EN_PM_TIMER_US		BIT(27)

/* Global User Control 1 Register */
#define DWC3_GUCTL1_PARKMODE_DISABLE_SS	BIT(17)
#define DWC3_GUCTL1_TX_IPGAP_LINECHECK_DIS	BIT(28)
#define DWC3_GUCTL1_DEV_L1_EXIT_BY_HW	BIT(24)
#define DWC3_GUCTL1_IP_GAP_ADD_ON(n)		((n) << 21)
#define DWC3_GUCTL1_IP_GAP_ADD_ON_MASK	(DWC3_GUCTL1_IP_GAP_ADD_ON(0x7))

/* Debug Register */
#define  DWC3_BU31RHBDBG_TOUTCTL	(0x1 << 3)

#define DWC3_EXYNOS_IGNORE_CORE_OPS	0xff

#define USB_BUS_CLOCK_DELAY_MS 3000

#define DWC3_EXYNOS_MAX_WAIT_COUNT 250
#define DWC3_EXYNOS_DISCONNECT_COUNT 500

/* PHY Owner Bits */
#define DWC3_EXYNOS_PHY_OWNER_USB	BIT(0)
#define DWC3_EXYNOS_PHY_OWNER_DP	BIT(1)

struct dwc3_exynos_config { /* Exynos Specific Configuations */
	bool adj_sof_accuracy;
	bool is_not_vbus_pad;
	bool sparse_transfer_control;
	bool no_extra_delay;
	bool ux_exit_in_px_quirk;
	bool elastic_buf_mode_quirk;
	bool force_gen1;
	bool u1u2_exitfail_to_recov_quirk;
	u32 usb_host_device_timeout;
	u32 suspend_clk_freq;
};

struct dwc3_exynos {
	struct platform_device	*usb2_phy;
	struct platform_device	*usb3_phy;
	struct device		*dev;
	struct dwc3		*dwc;

	struct clk		**clocks;
	struct clk		*bus_clock;

	struct extcon_dev	*edev;
	struct notifier_block	device_nb;
	struct notifier_block	host_nb;

	struct mutex		dotg_lock;

	bool			usb_data_enabled;
	bool			extra_delay;
	bool			gadget_state;

	int			idle_ip_index;

	struct dwc3_otg		*dotg;

	struct dwc3_exynos_config config;

	int			force_speed;

	/* Flag for setting current_dr_role peripheral */
	int			need_dr_role;

	struct xhci_goog_dma_coherent_mem	**mem;

	/* bitmap of usb phy owners */
	int			phy_owner_bits;
};

static inline u32 dwc3_exynos_readl(void __iomem *base, u32 offset)
{
	return readl(base + offset - DWC3_GLOBALS_REGS_START);
}

static inline void dwc3_exynos_writel(void __iomem *base, u32 offset, u32 value)
{
	writel(value, base + offset - DWC3_GLOBALS_REGS_START);
}

int dwc3_exynos_host_event(struct device *dev, int action);
int dwc3_exynos_device_event(struct device *dev, bool action);
int dwc3_exynos_phy_enable(int owner, bool on);
extern int dwc3_exynos_set_bus_clock(struct device *dev, int clk_level);

int dwc3_exynos_core_init(struct dwc3 *dwc, struct dwc3_exynos *exynos);
int dwc3_exynos_host_init(struct dwc3_exynos *exynos);
void dwc3_exynos_host_exit(struct dwc3_exynos *exynos);
void dwc3_exynos_gadget_disconnect_proc(struct dwc3 *dwc);
int dwc3_core_susphy_set(struct dwc3 *dwc, int on);
#endif /* __LINUX_USB_DWC3_EXYNOS_H */

