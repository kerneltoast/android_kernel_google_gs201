/* SPDX-License-Identifier: GPL-2.0 */
/*
 * core.h - DesignWare USB3 DRD Core Header
 *
 * Copyright (C) 2010-2011 Texas Instruments Incorporated - http://www.ti.com
 *
 * Authors: Felipe Balbi <balbi@ti.com>,
 *	    Sebastian Andrzej Siewior <bigeasy@linutronix.de>
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

#ifndef __DRIVERS_USB_DWC3_CORE_EXYNOS_H
#define __DRIVERS_USB_DWC3_CORE_EXYNOS_H

#define DWC3_GDBGLSPMUX_HST	0xc170

/* LINK Registers */
#define DWC3_LSKIPFREQ		0xd020
#define DWC3_LLUCTL		0xd024

/* Bit fields */
/* Link Register - LLUCTL */
#define DWC3_PENDING_HP_TIMER_US(n)	((n) << 16)
#define DWC3_EN_US_HP_TIMER		BIT(15)
#define DWC3_FORCE_GEN1		BIT(10)

/* Link Register - LSKIPFREQ */
#define DWC3_PM_ENTRY_TIMER_US(n)	((n) << 20)
#define DWC3_PM_LC_TIMER_US(n)		((n) << 24)
#define DWC3_EN_PM_TIMER_US		BIT(27)

/* Global Debug Queue/FIFO Space Available Register */
#define DWC3_GSBUSCFG0_INCRBRSTEN	BIT(0)
#define DWC3_GSBUSCFG0_INCR4BRSTEN	BIT(1)
#define DWC3_GSBUSCFG0_INCR8BRSTEN	BIT(2)
#define DWC3_GSBUSCFG0_INCR16BRSTEN	BIT(3)
#define DWC3_GSBUSCFG0_INCR32BRSTEN	BIT(4)
#define DWC3_GSBUSCFG0_INCR64BRSTEN	BIT(5)
#define DWC3_GSBUSCFG0_INCR128BRSTEN	BIT(6)
#define DWC3_GSBUSCFG0_INCR256BRSTEN	BIT(7)
#define DWC3_GSBUSCFG0_DESWRREQINFO	(2 << 16)
#define DWC3_GSBUSCFG0_DATWRREQINFO	(2 << 20)
#define DWC3_GSBUSCFG0_DESRDREQINFO	(2 << 24)
#define DWC3_GSBUSCFG0_DATRDREQINFO	(2 << 28)

#define DWC3_GSBUSCFG1_BREQLIMIT(n)	((n) << 8)
#define DWC3_GSBUSCFG1_BREQLIMIT_SHIFT	8
#define DWC3_GSBUSCFG1_BREQLIMIT_MASK	(0xf << 8)
#define DWC3_GSBUSCFG1_EN1KPAGE		BIT(12)

#define DWC3_GRXTHRCFG_USBRXPKTCNTSEL		BIT(29)
#define DWC3_GRXTHRCFG_USBRXPKTCNT_MASK		(0xf << 24)
#define DWC3_GRXTHRCFG_USBRXPKTCNT_SHIFT	24
#define DWC3_GRXTHRCFG_USBRXPKTCNT(n)		((n) << 24)
#define DWC3_GRXTHRCFG_USBMAXRXBURSTSIZE_MASK	(0x1f << 19)
#define DWC3_GRXTHRCFG_USBMAXRXBURSTSIZE_SHIFT	19
#define DWC3_GRXTHRCFG_USBMAXRXBURSTSIZE(n)	((n) << 19)

#define DWC3_GCTL_PWRDNSCALE_MASK      DWC3_GCTL_PWRDNSCALE(0x1fff)

#define DWC3_GUCTL_REFCLKPER(n)                ((n) << 22)
#define DWC3_GUCTL_REFCLKPER_MASK      DWC3_GUCTL_REFCLKPER(0x3FF)
#define DWC3_GUCTL_USBHSTINAUTORETRYEN	BIT(14)
#define DWC3_GUCTL_SPRSCTRLTRANSEN	BIT(17)

/* Global USB3 PIPE Control Register */
#define DWC3_GUSB3PIPECTL_U1U2EXITFAIL_TO_RECOV	BIT(25)
#define DWC3_ELASTIC_BUFFER_MODE	BIT(0)

/* Global Frame Length Adjustment Register */
#define DWC3_GFLADJ_REFCLK_240MHZDECR_PLS1	BIT(31)
#define DWC3_GFLADJ_REFCLK_240MHZ_DECR(n)	((n) << 24)
#define DWC3_GFLADJ_REFCLK_240MHZ_DECR_MASK	DWC3_GFLADJ_REFCLK_240MHZ_DECR(0x7F)
#define DWC3_GFLADJ_REFCLK_LPM_SEL		BIT(23)
#define DWC3_GFLADJ_REFCLK_FLADJ(n)		((n) << 8)
#define DWC3_GFLADJ_REFCLK_FLADJ_MASK		DWC3_GFLADJ_REFCLK_FLADJ(0x3FFF)

#define DWC3_GFLADJ_REFCLK_240MHZDECR_PLS1	BIT(31)
#define DWC3_GFLADJ_REFCLK_240MHZ_DECR(n)	((n) << 24)
#define DWC3_GFLADJ_REFCLK_240MHZ_DECR_MASK	DWC3_GFLADJ_REFCLK_240MHZ_DECR(0x7F)
#define DWC3_GFLADJ_REFCLK_LPM_SEL		BIT(23)
#define DWC3_GFLADJ_REFCLK_FLADJ(n)		((n) << 8)
#define DWC3_GFLADJ_REFCLK_FLADJ_MASK		DWC3_GFLADJ_REFCLK_FLADJ(0x3FFF)

#define DWC3_DCFG_FULLSPEED1	(3 << 0)

#define DWC3_DEVTEN_U3L2_SUSPEN		BIT(6)

#define DWC3_DSTS_FULLSPEED1		(3 << 0)

/* OTG Control Register */
#define DWC3_OTG_OCTL_PERIMODE         BIT(6)

/* OTG Events Register */
#define DWC3_OEVT_CLEAR_ALL			(~DWC3_OEVT_DEVICEMODE)
#define DWC3_OEVTEN_OTGCONIDSTSCHNGEVNT		BIT(24)
#define DWC3_OEVTEN_OTGBDEVVBUSCHNGEVNT		BIT(8)

/* OTG Status Register */
#define DWC3_OTG_OSTS_BSESVALID		BIT(1)
#define DWC3_OTG_OSTS_CONIDSTS		BIT(0)

#define DWC3_TRB_MASK		(DWC3_TRB_NUM - 1)

int dwc3_exynos_core_init(struct dwc3 *dwc);
int exynos_usbdrd_phy_tune(struct phy *phy, int phy_state);
void exynos_usbdrd_phy_conn(struct phy *phy, int is_conn);
//int exynos_usbdrd_dp_ilbk(struct phy *phy);
//int exynos_usbdrd_phy_vendor_set(struct phy *phy, int is_enable,
//						int is_cancel);
int exynos_usbdrd_phy_set(struct phy *phy, int option, void *info);

#endif /* __DRIVERS_USB_DWC3_CORE_H */
