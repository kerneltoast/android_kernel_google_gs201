// SPDX-License-Identifier: GPL-2.0-only
/*
 * PCIe phy driver for zuma SoC
 *
 * Copyright (C) 2021 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 */


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/of_gpio.h>
#include <linux/pci.h>
#include <linux/delay.h>
#include <linux/exynos-pci-noti.h>
#include <linux/regmap.h>
#include "pcie-designware.h"
#include "pcie-exynos-common.h"
#include "pcie-exynos-rc.h"
#include <dt-bindings/pci/pci.h>
#include <misc/logbuffer.h>

#if IS_ENABLED(CONFIG_EXYNOS_OTP)
#include <linux/exynos_otp.h>
#endif

/* avoid checking rx elecidle when access DBI */
void exynos_pcie_rc_phy_check_rx_elecidle(void *phy_pcs_base_regs, int val, int ch_num)
{
	/*
	 * Todo: need guide
	 */
}

/* PHY all power down */
void exynos_pcie_rc_phy_all_pwrdn(struct exynos_pcie *exynos_pcie, int ch_num)
{
	void __iomem *phy_base_regs = exynos_pcie->phy_base;
	void __iomem *udbg_base_regs = exynos_pcie->udbg_base;
	void __iomem *phy_pcs_base_regs = exynos_pcie->phy_pcs_base;
	u32 val;
	u32 pcs_val, pcs_val2, phy_val, phy_val2;
	int cnt = 0;

	dev_dbg(exynos_pcie->pci->dev, "[CAL: %s]\n", __func__);

	if (exynos_pcie->ch_num == 1) { //PCIE GEN3 1 lane channel
		val = readl(phy_base_regs + 0x204) & ~(0x3 << 2);
		writel(val, phy_base_regs + 0x204);

		//XO CLK gating
		//val = readl(udbg_base_regs + 0xC828) | (0x1 << 17);
		//writel(val, udbg_base_regs + 0xC828);

		writel(0x2A, phy_base_regs + 0x1044);
		writel(0xAA, phy_base_regs + 0x1048);
		writel(0xA8, phy_base_regs + 0x104C);
		writel(0x80, phy_base_regs + 0x1050);
		writel(0x0A, phy_base_regs + 0x185C);
		udelay(1);

		writel(0xFF, phy_base_regs + 0x0208);
		udelay(1);

		writel(0x0A, phy_base_regs + 0x0580);
		writel(0xAA, phy_base_regs + 0x0928);

		//Common Bias, PLL off
		writel(0x0A, phy_base_regs + 0x00C);

		udelay(50);
		// External PLL gating
		val = readl(udbg_base_regs + 0xC700) & ~BIT(0);
		writel(val, udbg_base_regs + 0xC700);
		udelay(10);
	}
	else { //PCIe GEN3 2 lane channel
		val = readl(phy_base_regs + 0x204) & ~(0x3 << 2);
		writel(val, phy_base_regs + 0x204);

		//XO CLK gating
		//val = readl(udbg_base_regs + 0xC804) | (0x3 << 8);
		//writel(val, udbg_base_regs + 0xC804);

		/* PLL off, Bias on */
		writel(0x300D9, phy_pcs_base_regs + 0x150);
		udelay(5);
		pcs_val = readl(phy_pcs_base_regs + 0x150);

		writel(0x2A, phy_base_regs + 0x1044);
		writel(0xAA, phy_base_regs + 0x1048);
		writel(0xA8, phy_base_regs + 0x104C);
		writel(0x80, phy_base_regs + 0x1050);
		writel(0x0A, phy_base_regs + 0x185C);
		udelay(1);

		writel(0x2A, phy_base_regs + 0x2044);
		writel(0xAA, phy_base_regs + 0x2048);
		writel(0xA8, phy_base_regs + 0x204C);
		writel(0x80, phy_base_regs + 0x2050);
		writel(0x0A, phy_base_regs + 0x285C);
		udelay(1);

		writel(0xFF, phy_base_regs + 0x208);
		writel(0xFF, phy_base_regs + 0x228);
		udelay(1);

		writel(0x0A, phy_base_regs + 0x0580);
		writel(0xAA, phy_base_regs + 0x0928);

		writel(0x0A, phy_base_regs + 0x000C);

		/* Force PLL Lock Signal off */
		phy_val = readl(phy_base_regs + 0x600);
		phy_val2 = phy_val & ~(0x3 << 4);
		phy_val2 = phy_val | (0x2 << 4);
		writel(phy_val2, phy_base_regs + 0x600);
		dev_dbg(exynos_pcie->pci->dev, "pwrdn: pma+0x600: %#x->%#x\n", phy_val, phy_val2);

		/* PLL off, Bias off */
		writel(0x300DE, phy_pcs_base_regs + 0x150);
		udelay(5);
		pcs_val2 = readl(phy_pcs_base_regs + 0x150);
		dev_dbg(exynos_pcie->pci->dev, "pwrdn: pcs+0x150: %#x->%#x\n", pcs_val, pcs_val2);

		/* Check for PMA PLL to be disengaged */
		while (cnt < 100) {
			val = readl(phy_base_regs + 0x260);
			if ((val & 0x4) == 0x0)
				break;
			cnt++;
			udelay(1);
		}

		/* If PMA didn't disengage log the error but skip turning
		 * EXT PLL off.
		 */
		if (cnt == 100) {
			logbuffer_logk(exynos_pcie->log, LOGLEVEL_ERR,
				       "pwrdn: PMA PLL did not turn off\n");
			return;
		}

		// Turn off External PLL for PCIe PHY
		val = readl(udbg_base_regs + 0xC700) | (0x1 << 1);
		writel(val, udbg_base_regs + 0xC700);
		udelay(10);
	}
}

/* PHY all power down clear */
void exynos_pcie_rc_phy_all_pwrdn_clear(struct exynos_pcie *exynos_pcie, int ch_num)
{
	void __iomem *phy_base_regs = exynos_pcie->phy_base;
	void __iomem *udbg_base_regs = exynos_pcie->udbg_base;
	u32 val, val2;

	dev_dbg(exynos_pcie->pci->dev, "[CAL: %s]\n", __func__);

	if (exynos_pcie->ch_num == 1) { //PCIE GEN3 1 lane channel
		// External PLL gating
		val = readl(udbg_base_regs + 0xC700) | BIT(0);
		writel(val, udbg_base_regs + 0xC700);
		udelay(100);
		writel(0x00, phy_base_regs + 0x000C);

		writel(0x55, phy_base_regs + 0x0928);
		writel(0x02, phy_base_regs + 0x0580);

		writel(0x00, phy_base_regs + 0x0208);

		writel(0x00, phy_base_regs + 0x1044);
		writel(0x00, phy_base_regs + 0x1048);
		writel(0x00, phy_base_regs + 0x104C);
		writel(0x00, phy_base_regs + 0x1050);
		writel(0x00, phy_base_regs + 0x185C);

		//XO CLK gating
		//val = readl(udbg_base_regs + 0xC828) & ~(0x1 << 17);
		//writel(val, udbg_base_regs + 0xC828);
		udelay(10);
	}
	else { //PCIe GEN3 2 lane channel
		val = readl(udbg_base_regs + 0xC700) & ~(0x1 << 1);
		writel(val, udbg_base_regs + 0xC700);
		udelay(100);

		/* Restore PLL Lock signal Off */
		val = readl(phy_base_regs + 0x600);
		val2 = val & ~(0x3 << 4);
		writel(val2, phy_base_regs + 0x600);
		dev_dbg(exynos_pcie->pci->dev, "pwrdn_clr: pma+0x600: %#x->%#x\n", val, val2);

		writel(0x00, phy_base_regs + 0x000C);

		writel(0x55, phy_base_regs + 0x0928);
		writel(0x02, phy_base_regs + 0x0580);

		writel(0x00, phy_base_regs + 0x0208);
		writel(0x00, phy_base_regs + 0x0228);

		writel(0x00, phy_base_regs + 0x1044);
		writel(0x00, phy_base_regs + 0x1048);
		writel(0x00, phy_base_regs + 0x104C);
		writel(0x00, phy_base_regs + 0x1050);
		writel(0x00, phy_base_regs + 0x185C);

		writel(0x00, phy_base_regs + 0x2044);
		writel(0x00, phy_base_regs + 0x2048);
		writel(0x00, phy_base_regs + 0x204C);
		writel(0x00, phy_base_regs + 0x2050);
		writel(0x00, phy_base_regs + 0x285C);

		//XO CLK gating
		//val = readl(udbg_base_regs + 0xC804) & ~(0x3 << 8);
		//writel(val, udbg_base_regs + 0xC804);

		//val = readl(phy_base_regs + 0x204) & ~(0x3 << 2);
		//writel(val, phy_base_regs + 0x204);
		udelay(10);
	}
}

#if IS_ENABLED(CONFIG_EXYNOS_OTP)
void exynos_pcie_rc_pcie_phy_otp_config(void *phy_base_regs, int ch_num)
{
	/* To be updated */
}
#endif

#define NEW_DATA_AVERAGE_WEIGHT  10
static void link_stats_log_pll_lock(struct exynos_pcie *pcie, u32 pll_lock_time)
{
	pcie->link_stats.pll_lock_time_avg =
	    DIV_ROUND_CLOSEST(pcie->link_stats.pll_lock_time_avg *
			      (100 - NEW_DATA_AVERAGE_WEIGHT) +
			      pll_lock_time * NEW_DATA_AVERAGE_WEIGHT, 100);
}

#define LCPLL_REF_CLK_SEL	(0x3 << 4)

static int check_exynos_pcie_reg_status(struct exynos_pcie *exynos_pcie,
					void __iomem *base, int offset, int bit,
					int *cnt)
{
	int i;
	u32 status;
	ktime_t timestamp = ktime_get();

	for (i = 0; i < 10000; i++) {
		udelay(10);
		status = readl(base + offset) & BIT(bit);
		if (status != 0)
			break;
	}
	timestamp = ktime_sub(ktime_get(), timestamp);

	if (cnt)
		*cnt = i;
	dev_dbg(exynos_pcie->pci->dev, "elapsed time: %lld uS\n", ktime_to_us(timestamp));
	logbuffer_log(exynos_pcie->log, "elapsed time: %lld uS", ktime_to_us(timestamp));

	if (status == 0)
		logbuffer_logk(exynos_pcie->log, LOGLEVEL_ERR, "status 0x%x failed", offset);
	else {
		dev_dbg(exynos_pcie->pci->dev, "status 0x%x passed cnt=%d\n",
			offset, i);
		logbuffer_log(exynos_pcie->log, "status 0x%x passed cnt=%d", offset, i);
	}

	return status;
}

void exynos_pcie_rc_pcie_phy_config(struct exynos_pcie *exynos_pcie, int ch_num)
{
	void __iomem *elbi_base_regs = exynos_pcie->elbi_base;
	void __iomem *udbg_base_regs = exynos_pcie->udbg_base;
	void __iomem *soc_base_regs = exynos_pcie->soc_base;
	void __iomem *phy_base_regs = exynos_pcie->phy_base;
	void __iomem *phy_pcs_base_regs = exynos_pcie->phy_pcs_base;
	int num_lanes = exynos_pcie->num_lanes;
	int lock_cnt;
	u32 ext_pll_lock, pll_lock, cdr_lock, oc_done;
	u32 val;
	u32 i;

	dev_dbg(exynos_pcie->pci->dev, "PCIe CAL ver 0.2\n");

	if (exynos_pcie->ch_num == 1) {
		/* I/A will be disabled in PCIe poweroff
		   if (exynos_pcie->use_ia && exynos_pcie->info->set_ia_code) {
		//IA disable
		exynos_ia_write(exynos_pcie, 0x0, 0x60);
		exynos_ia_write(exynos_pcie, 0x0, 0x0);
		exynos_ia_write(exynos_pcie, 0xD0000004, 0x100);
		exynos_ia_write(exynos_pcie, 0xB0020188, 0x108);
		udelay(10);
		}
		 */

		if (exynos_pcie->customized_de_emphasis) {
			/* set customized de-emphasis level */
			val = readl(phy_pcs_base_regs + 0x4CC);
			val &= ~(0x1f << 12);
			val |= (exynos_pcie->de_emphasis_value << 12);

			writel(val, phy_pcs_base_regs +  0x4CC);
			writel(val, phy_pcs_base_regs +  0x4D0);
			writel(val, phy_pcs_base_regs +  0x4D4);
			writel(val, phy_pcs_base_regs +  0x4D8);
			writel(val, phy_pcs_base_regs +  0x4DC);
			writel(val, phy_pcs_base_regs +  0x4E0);
			writel(val, phy_pcs_base_regs +  0x4E4);
			writel(val, phy_pcs_base_regs +  0x4E8);
			writel(val, phy_pcs_base_regs +  0x4EC);
		}

		writel(0x3, phy_base_regs + 0x032C);    //PHY input clock un-gating

		val = readl(udbg_base_regs + 0xC700) | (0x1 << 0);
		writel(val, udbg_base_regs + 0xC700);        //Release External PLL init

		val = readl(udbg_base_regs + 0xC700) & ~(0x1 << 1);
		writel(val, udbg_base_regs + 0xC700);        //Override External PLL RESETB

		/* check external pll lock */
		ext_pll_lock = check_exynos_pcie_reg_status(exynos_pcie, udbg_base_regs,
							    0xC734, 2, &lock_cnt);
		logbuffer_log(exynos_pcie->log, "EXT_PLL_LOCK : 0x%x", ext_pll_lock);

		/* soc_ctrl setting */
		val = readl(soc_base_regs + 0x4004) & ~(1 << 2);
		writel(val, soc_base_regs + 0x4004);    //Select PHY input clock. 1b'1 = TCXO, 1b'0 = External PLL clock

		val = readl(soc_base_regs + 0x4004) | (0x3 << 4);
		writel(val, soc_base_regs + 0x4004);    //Select PHY input clock. 2b'11 = External PLL

		/* device type setting */
		writel(0x4, elbi_base_regs + 0x0080);

		/* soft_pwr_rst */
		writel(0xF, elbi_base_regs + 0x3A4);
		writel(0xD, elbi_base_regs + 0x3A4);
		udelay(10);
		writel(0xF, elbi_base_regs + 0x3A4);
		udelay(10);

		/* pma rst assert*/
		writel(0x1, elbi_base_regs + 0x1404);
		writel(0x1, elbi_base_regs + 0x1408);
		writel(0x1, elbi_base_regs + 0x1400);
		writel(0x0, elbi_base_regs + 0x1404);
		writel(0x0, elbi_base_regs + 0x1408);
		writel(0x0, elbi_base_regs + 0x1400);

		/* slv pend check */
		writel(0x1, elbi_base_regs + PCIE_SLV_PEND_SEL_NAK);

		/* pma_setting */

		//Common
		writel(0x88, phy_base_regs + 0x0000);
		writel(0x66, phy_base_regs + 0x001C);
		writel(0x00, phy_base_regs + 0x01F4);
		writel(0x59, phy_base_regs + 0x0514);
		writel(0x11, phy_base_regs + 0x051C);
		writel(0x0E, phy_base_regs + 0x062C);
		writel(0x22, phy_base_regs + 0x0644);
		writel(0x03, phy_base_regs + 0x0688);
		writel(0x28, phy_base_regs + 0x06D4);
		writel(0x64, phy_base_regs + 0x0788);
		writel(0x64, phy_base_regs + 0x078C);
		writel(0x50, phy_base_regs + 0x0790);
		writel(0x50, phy_base_regs + 0x0794);
		writel(0x05, phy_base_regs + 0x0944);
		writel(0x05, phy_base_regs + 0x0948);
		writel(0x05, phy_base_regs + 0x094C);
		writel(0x05, phy_base_regs + 0x0950);

		/* REFCLK 38.4Mhz to External PLL path */
		writel(0x02, phy_base_regs + 0x0590);
		writel(0xB0, phy_base_regs + 0x07F8);
		writel(0x08, phy_base_regs + 0x0730);

		//Delay@L1.2 = 7.68us
		writel(0xC0, phy_base_regs + 0x344);

		//test with no relatin of CLKREQ @L1.2
		writel(0x04, phy_base_regs + 0x0040);
		writel(0x03, phy_base_regs + 0x0204);
		writel(0x1F, phy_base_regs + 0x02D4);
		writel(0x10, phy_base_regs + 0x0358);

		//ERIO on
		writel(0x01, phy_base_regs + 0x0018);

		//lane
		writel(0x5B, phy_base_regs + 0x0514);
		writel(0x0C, phy_base_regs + 0x0608);
		writel(0x0F, phy_base_regs + 0x060C);
		writel(0x0F, phy_base_regs + 0x0610);
		writel(0x0F, phy_base_regs + 0x0614);
		writel(0x0F, phy_base_regs + 0x0618);
		writel(0x80, phy_base_regs + 0x0510);
		writel(0x03, phy_base_regs + 0x0688);
		writel(0x23, phy_base_regs + 0x0644);
		writel(0x11, phy_base_regs + 0x0624);

		//PLL margin issue setting for ERIO (GEN1 & GEN2)
		writel(0x0f, phy_base_regs + 0x0630);
		writel(0x53, phy_base_regs + 0x06D0);

		for (i = 0; i < num_lanes; i++) {
			phy_base_regs += (i * 0x1000);

			writel(0x04, phy_base_regs + 0x1140);
			writel(0x04, phy_base_regs + 0x1144);
			writel(0x04, phy_base_regs + 0x1148);
			writel(0x02, phy_base_regs + 0x114C);
			writel(0x00, phy_base_regs + 0x1150);
			writel(0x00, phy_base_regs + 0x1154);
			writel(0x00, phy_base_regs + 0x1158);
			writel(0x00, phy_base_regs + 0x115C);
			writel(0x1C, phy_base_regs + 0x12CC);
			writel(0x6C, phy_base_regs + 0x12DC);
			writel(0x29, phy_base_regs + 0x130C);
			writel(0x2F, phy_base_regs + 0x13B4);
			writel(0x05, phy_base_regs + 0x1A64);
			writel(0x05, phy_base_regs + 0x1A68);
			writel(0x05, phy_base_regs + 0x1A84);
			writel(0x05, phy_base_regs + 0x1A88);
			writel(0x00, phy_base_regs + 0x1A98);
			writel(0x00, phy_base_regs + 0x1A9C);
			writel(0x07, phy_base_regs + 0x1AA8);
			writel(0x00, phy_base_regs + 0x1AB8);
			writel(0x00, phy_base_regs + 0x1ABC);
			writel(0x90, phy_base_regs + 0x1AF8);
			writel(0x03, phy_base_regs + 0x1B34);
			writel(0x03, phy_base_regs + 0x1BB0);
			writel(0x03, phy_base_regs + 0x1BB4);
			writel(0x06, phy_base_regs + 0x1BC0);
			writel(0x06, phy_base_regs + 0x1BC4);
			writel(0x01, phy_base_regs + 0x1BE8);
			writel(0x04, phy_base_regs + 0x1BF8);
			writel(0x00, phy_base_regs + 0x1C98);
			writel(0x81, phy_base_regs + 0x1CA4);
		}

		if (exynos_pcie->ep_device_type == EP_BCM_WIFI) {
			/* L1SS exit link down issue in BCM */
			writel(0x02, phy_base_regs + 0x1B20);
			writel(0x23, phy_base_regs + 0x1340);
			writel(0x3C, phy_base_regs + 0x002C);
		} else if (exynos_pcie->ep_device_type == EP_QC_WIFI) {
			/* TX over/under shoot improvement in QC */
			writel(0x1, phy_base_regs + 0x1818);
			writel(0x8, phy_base_regs + 0x1808);

			/* L1SS exit link down issue in QC */
			writel(0x2, phy_base_regs + 0x1B1C);
		}

		val = readl(phy_base_regs + 0x1350);
		val |= 0x1;
		writel(val, phy_base_regs + 0x1350);

		/* PCS setting */
		writel(0x100B0604, phy_pcs_base_regs + 0x190);//New Guide

		//test with no relatin of CLKREQ @L1.2
		writel(0x000700D5, phy_pcs_base_regs + 0x154);

		writel(0x16400000, phy_pcs_base_regs + 0x100);
		writel(0x08600000, phy_pcs_base_regs + 0x104);
		writel(0x18500000, phy_pcs_base_regs + 0x114);
		writel(0x60700000, phy_pcs_base_regs + 0x124);
		writel(0x00000007, phy_pcs_base_regs + 0x174);
		writel(0x00000100, phy_pcs_base_regs + 0x178);

		if (exynos_pcie->ep_device_type == EP_BCM_WIFI) {
			writel(0x00000700, phy_pcs_base_regs + 0x17c);
		} else {
			writel(0x00000010, phy_pcs_base_regs + 0x17c);
		}

		/* Additional configuration for SAMSUNG WIFI */
		if (exynos_pcie->ep_device_type == EP_SAMSUNG_WIFI) {
			phy_base_regs = exynos_pcie->phy_base;

			//work around
			writel(0x3D, phy_pcs_base_regs + 0x8);// xo clock always on setting 1D -> 3D
			writel(0x02, phy_base_regs + 0x40);
			writel(0x02, phy_base_regs + 0x2D4);
			writel(0x02, phy_base_regs + 0x358);

			writel(0x02, phy_base_regs + 0x204);

			writel(0x700D5, phy_pcs_base_regs + 0x154);// always on clkref P1 CPM
			writel(0x16400000, phy_pcs_base_regs + 0x100);
			writel(0x08600000, phy_pcs_base_regs + 0x104);
			writel(0x18500000, phy_pcs_base_regs + 0x114);
			writel(0x60700000, phy_pcs_base_regs + 0x124);// P1cpm -> p1.2 timer3
			writel(0x5, phy_pcs_base_regs + 0x174);
			writel(0x100, phy_pcs_base_regs + 0x178);// timer 2 cnt
			writel(0x10, phy_pcs_base_regs + 0x17C);// timer 3 count
		}

		if (exynos_pcie->ep_device_type == EP_BCM_WIFI) {
			writel(0x01600202, phy_pcs_base_regs + 0x110);
		}

		/* pma rst release */
		writel(0x1, elbi_base_regs + 0x1404);
		udelay(10);
		writel(0x1, elbi_base_regs + 0x1408);
		writel(0x1, elbi_base_regs + 0x1400);

		/* check pll lock */
		pll_lock = check_exynos_pcie_reg_status(exynos_pcie, phy_base_regs,
							0x0A80, 0, &lock_cnt);
		/* check cdr lock */
		cdr_lock = check_exynos_pcie_reg_status(exynos_pcie, phy_base_regs,
							0x15C0, 4, &lock_cnt);
		if (cdr_lock)
			link_stats_log_pll_lock(exynos_pcie, lock_cnt);

		/* check offset calibration */
		oc_done = check_exynos_pcie_reg_status(exynos_pcie, phy_base_regs,
						       0x140C, 7, &lock_cnt);

		logbuffer_log(exynos_pcie->log,
			      "PLL_LOCK : 0x%x, CDR_LOCK : 0x%x, OC_DONE : 0x%x",
			      pll_lock, cdr_lock, oc_done);

		writel(0x0, phy_base_regs + 0x032C);    //PHY input clock un-gating

		//L1 exit off by DBI
		writel(0x1, elbi_base_regs + 0x1078);
	}
	else {
		writel(0x3, phy_base_regs + 0x032C);    //PHY input clock un-gating

		/* External PLL seting */
		val = readl(udbg_base_regs + 0xC710) & ~(0x1 << 1);
		writel(val, udbg_base_regs + 0xC710);        //External PLL initialization

		val = readl(udbg_base_regs + 0xC700) & ~(0x1 << 1);
		writel(val, udbg_base_regs + 0xC700);        //Override External PLL RESETB

		/* check external pll lock */
		ext_pll_lock = check_exynos_pcie_reg_status(exynos_pcie, udbg_base_regs,
							    0xC704, 3, &lock_cnt);
		logbuffer_log(exynos_pcie->log, "EXT_PLL_LOCK : 0x%x", ext_pll_lock);

		/* soc_ctrl setting */
		val = readl(soc_base_regs + 0x4004) & ~(1 << 2);
		writel(val, soc_base_regs + 0x4004);    //Select PHY input clock. 1b'1 = TCXO, 1b'0 = External PLL clock

		val = readl(soc_base_regs + 0x4004) | (0x3 << 4);
		writel(val, soc_base_regs + 0x4004);    //Select PHY input clock. 2b'11 = External PLL

		/* device type setting */
		writel(0x4, elbi_base_regs + 0x0080);

		/* soft_pwr_rst */
		writel(0xF, elbi_base_regs + 0x3A4);
		writel(0xD, elbi_base_regs + 0x3A4);
		udelay(10);
		writel(0xF, elbi_base_regs + 0x3A4);
		udelay(10);

		/* pma rst assert*/
		writel(0x1, elbi_base_regs + 0x1404);
		writel(0x1, elbi_base_regs + 0x1408);
		writel(0x1, elbi_base_regs + 0x1400);
		writel(0x0, elbi_base_regs + 0x1404);
		writel(0x0, elbi_base_regs + 0x1408);
		writel(0x0, elbi_base_regs + 0x1400);

                /* slv pend check */
                writel(0x1, elbi_base_regs + PCIE_SLV_PEND_SEL_NAK);

		/* pma_setting */

		//Common
		writel(0x88, phy_base_regs + 0x0000);
		writel(0x66, phy_base_regs + 0x001C);
		writel(0x00, phy_base_regs + 0x01F4);
		writel(0x81, phy_base_regs + 0x0510);
		writel(0x59, phy_base_regs + 0x0514);
		writel(0x11, phy_base_regs + 0x051C);
		writel(0x0E, phy_base_regs + 0x062C);
		writel(0x22, phy_base_regs + 0x0644);
		writel(0x03, phy_base_regs + 0x0688);
		writel(0x28, phy_base_regs + 0x06D4);
		writel(0x64, phy_base_regs + 0x0788);
		writel(0x64, phy_base_regs + 0x078C);
		writel(0x50, phy_base_regs + 0x0790);
		writel(0x50, phy_base_regs + 0x0794);
		writel(0x05, phy_base_regs + 0x0944);
		writel(0x05, phy_base_regs + 0x0948);
		writel(0x05, phy_base_regs + 0x094C);
		writel(0x05, phy_base_regs + 0x0950);

		/* REFCLK 38.4Mhz to External PLL path */
		writel(0x02, phy_base_regs + 0x0590);
		writel(0xB0, phy_base_regs + 0x07F8);
		writel(0x08, phy_base_regs + 0x0730);

		//Delay@L1.2 = 7.68us
		writel(0xC0, phy_base_regs + 0x344);

		//test with no relatin of CLKREQ @L1.2
		writel(0x04, phy_base_regs + 0x0040);
		writel(0x03, phy_base_regs + 0x0204);
		writel(0x1F, phy_base_regs + 0x02D4);
		writel(0x10, phy_base_regs + 0x0358);

		//ERIO on
		writel(0x01, phy_base_regs + 0x0018);

		//lane
		writel(0x5B, phy_base_regs + 0x0514);
		writel(0x0C, phy_base_regs + 0x0608);
		writel(0x0F, phy_base_regs + 0x060C);
		writel(0x0F, phy_base_regs + 0x0610);
		writel(0x0F, phy_base_regs + 0x0614);
		writel(0x0F, phy_base_regs + 0x0618);
		writel(0x80, phy_base_regs + 0x0510);
		writel(0x03, phy_base_regs + 0x0688);
		writel(0x23, phy_base_regs + 0x0644);
		writel(0x11, phy_base_regs + 0x0624);

		//PLL margin issue setting for ERIO (GEN1 & GEN2)
		writel(0xf, phy_base_regs + 0x0630);
		writel(0x53, phy_base_regs + 0x06D0);

		//lane
		for (i = 0; i < num_lanes; i++) {
			phy_base_regs += (i * 0x1000);

			writel(0x04, phy_base_regs + 0x1140);
			writel(0x04, phy_base_regs + 0x1144);
			writel(0x04, phy_base_regs + 0x1148);
			writel(0x02, phy_base_regs + 0x114C);
			writel(0x00, phy_base_regs + 0x1150);
			writel(0x00, phy_base_regs + 0x1154);
			writel(0x00, phy_base_regs + 0x1158);
			writel(0x00, phy_base_regs + 0x115C);
			writel(0x1C, phy_base_regs + 0x12CC);
			writel(0x6C, phy_base_regs + 0x12DC);
			writel(0x29, phy_base_regs + 0x130C);
			writel(0x2F, phy_base_regs + 0x13B4);
			writel(0x05, phy_base_regs + 0x1A64);
			writel(0x05, phy_base_regs + 0x1A68);
			writel(0x05, phy_base_regs + 0x1A84);
			writel(0x05, phy_base_regs + 0x1A88);
			writel(0x00, phy_base_regs + 0x1A98);
			writel(0x00, phy_base_regs + 0x1A9C);
			writel(0x07, phy_base_regs + 0x1AA8);
			writel(0x00, phy_base_regs + 0x1AB8);
			writel(0x00, phy_base_regs + 0x1ABC);
			writel(0x90, phy_base_regs + 0x1AF8);
			writel(0x03, phy_base_regs + 0x1B34);
			writel(0x03, phy_base_regs + 0x1BB0);
			writel(0x03, phy_base_regs + 0x1BB4);
			writel(0x06, phy_base_regs + 0x1BC0);
			writel(0x06, phy_base_regs + 0x1BC4);
			writel(0x01, phy_base_regs + 0x1BE8);
			writel(0x04, phy_base_regs + 0x1BF8);
			writel(0x00, phy_base_regs + 0x1C98);
			writel(0x81, phy_base_regs + 0x1CA4);
		}

		/* PCS setting */
		writel(0x100B0604, phy_pcs_base_regs + 0x190);//New Guide

		//test with no relation of CLKREQ @L1.2
		writel(0x000700D5, phy_pcs_base_regs + 0x154);

		writel(0x16400000, phy_pcs_base_regs + 0x100);
		writel(0x08600000, phy_pcs_base_regs + 0x104);
		writel(0x18500000, phy_pcs_base_regs + 0x114);
		writel(0x60700000, phy_pcs_base_regs + 0x124);
		writel(0x00000007, phy_pcs_base_regs + 0x174);
		writel(0x00000100, phy_pcs_base_regs + 0x178);
		writel(0x00000010, phy_pcs_base_regs + 0x17c);

		/* pma rst release */
		writel(0x1, elbi_base_regs + 0x1404);
		udelay(10);
		writel(0x1, elbi_base_regs + 0x1408);
		writel(0x1, elbi_base_regs + 0x1400);

		/* check pll lock */
		pll_lock = check_exynos_pcie_reg_status(exynos_pcie, phy_base_regs,
							0x0A80, 0, &lock_cnt);
		/* check cdr lock */
		cdr_lock = check_exynos_pcie_reg_status(exynos_pcie, phy_base_regs,
							0x15C0, 4, &lock_cnt);
		if (cdr_lock)
			link_stats_log_pll_lock(exynos_pcie, lock_cnt);

		/* check offset calibration */
		oc_done = check_exynos_pcie_reg_status(exynos_pcie, phy_base_regs,
						       0x140C, 7, &lock_cnt);

		logbuffer_log(exynos_pcie->log,
			      "PLL_LOCK : 0x%x, CDR_LOCK : 0x%x, OC_DONE : 0x%x",
			      pll_lock, cdr_lock, oc_done);

		/* udbg setting */
		//need to udbg base SFR
		val = (readl(udbg_base_regs + 0xC710) & ~(0x3 << 8)) | (0x3 << 8);
		writel(val, udbg_base_regs + 0xC710); //when entring L2, External PLL clock gating

		writel(0x0, phy_base_regs + 0x032C);    //PHY input clock un-gating

		//L1 exit off by DBI
		writel(0x1, elbi_base_regs + 0x1078);
	}
}
EXPORT_SYMBOL_GPL(exynos_pcie_rc_pcie_phy_config);

int exynos_pcie_rc_eom(struct device *dev, void *phy_base_regs)
{
	struct exynos_pcie *exynos_pcie = dev_get_drvdata(dev);
	struct device_node *np = dev->of_node;
	unsigned int val;
	unsigned int num_of_smpl;
	unsigned int lane_width = 1;
	unsigned int timeout;
	int i, ret;
	int test_cnt = 0;
	struct pcie_eom_result **eom_result;

	u32 phase_sweep = 0;
	u32 vref_sweep = 0;
	u32 err_cnt = 0;
	u32 err_cnt_13_8;
	u32 err_cnt_7_0;

	dev_info(dev, "Start EoM\n");

	ret = of_property_read_u32(np, "num-lanes", &lane_width);
	if (ret) {
		dev_err(dev, "failed to get num of lane\n");
		return -EINVAL;
	}
	dev_info(dev, "num-lanes : %d\n", lane_width);

	/* eom_result[lane_num][test_cnt] */
	eom_result = kzalloc(sizeof(struct pcie_eom_result *) * lane_width, GFP_KERNEL);
	if (!eom_result)
		return -ENOMEM;

	for (i = 0; i < lane_width; i++) {
		eom_result[i] = kzalloc(sizeof(struct pcie_eom_result) *
				EOM_PH_SEL_MAX * EOM_DEF_VREF_MAX, GFP_KERNEL);
		if (!eom_result[i]) {
			kfree(eom_result);
			return -ENOMEM;
		}
	}

	exynos_pcie->eom_result = eom_result;

	num_of_smpl = 0xf;

	for (i = 0; i < lane_width; i++) {
		writel(0x21, phy_base_regs + RX_SSLMS_ADAP_HOLD_PMAD);
		writel(0x0, phy_base_regs + RX_EFOM_MODE);
		writel(0x0, phy_base_regs + RX_FOM_EN);

		writel(0x2, phy_base_regs + RX_EFOM_SETTLE_TIME);
		writel(0xA, phy_base_regs + RX_EFOM_NUM_OF_SAMPLE);
		writel(0x3, phy_base_regs + RX_SSLMS_ADAP_COEF_SEL_7_0);

		writel(0x27, phy_base_regs + RX_SSLMS_ADAP_HOLD_PMAD);
		writel(0xE2, phy_base_regs + RX_EFOM_SETTLE_TIME);
		writel(0x50, phy_base_regs + RX_EFOM_BIT_WIDTH_SEL);
		writel(0x14, phy_base_regs + RX_EFOM_MODE);
		writel(0x2, phy_base_regs + RX_FOM_EN);

		for (phase_sweep = 0; phase_sweep < PHASE_MAX; phase_sweep++) {
			writel(phase_sweep, phy_base_regs + RX_EFOM_EOM_PH_SEL);

			for (vref_sweep = 0; vref_sweep < VREF_MAX; vref_sweep++) {
				writel(0x14, phy_base_regs + RX_EFOM_MODE);
				writel(0x2, phy_base_regs + RX_FOM_EN);

				writel(vref_sweep, phy_base_regs + RX_EFOM_DFE_VREF_CTRL);

				writel(0x3, phy_base_regs + RX_FOM_EN);

				timeout = 0;
				do {
					if (timeout == 100) {
						dev_err(dev, "timeout happened\n");
						return false;
					}

					udelay(1);
					val = readl(phy_base_regs + RX_EFOM_DONE) & (0x1);

					timeout++;
				} while (val != 0x1);

				err_cnt_13_8 = readl(phy_base_regs + MON_RX_EFOM_ERR_CNT_13_8) << 8;
				err_cnt_7_0 = readl(phy_base_regs + MON_RX_EFOM_ERR_CNT_7_0);
				err_cnt = err_cnt_13_8 | err_cnt_7_0;

				dev_dbg(dev, "%d,%d : %d %d %d\n",
					i, test_cnt, phase_sweep, vref_sweep,
					err_cnt);

				//save result
				eom_result[i][test_cnt].phase = phase_sweep;
				eom_result[i][test_cnt].vref = vref_sweep;
				eom_result[i][test_cnt].err_cnt = err_cnt;
				test_cnt++;
			}
		}
		writel(0x21, phy_base_regs + RX_SSLMS_ADAP_HOLD_PMAD);
		writel(0x0, phy_base_regs + RX_EFOM_MODE);
		writel(0x0, phy_base_regs + RX_FOM_EN);

		/* goto next lane */
		phy_base_regs += 0x1000;
		test_cnt = 0;
	}

	return 0;
}

void exynos_pcie_rc_phy_init(struct dw_pcie_rp *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct exynos_pcie *exynos_pcie = to_exynos_pcie(pci);

	dev_info(pci->dev, "Initialize PHY functions.\n");

	exynos_pcie->phy_ops.phy_check_rx_elecidle =
		exynos_pcie_rc_phy_check_rx_elecidle;
	exynos_pcie->phy_ops.phy_all_pwrdn = exynos_pcie_rc_phy_all_pwrdn;
	exynos_pcie->phy_ops.phy_all_pwrdn_clear =
					exynos_pcie_rc_phy_all_pwrdn_clear;
	exynos_pcie->phy_ops.phy_config = exynos_pcie_rc_pcie_phy_config;
	exynos_pcie->phy_ops.phy_eom = exynos_pcie_rc_eom;
}
EXPORT_SYMBOL_GPL(exynos_pcie_rc_phy_init);

static void exynos_pcie_quirks(struct pci_dev *dev)
{
	device_disable_async_suspend(&dev->dev);
	pr_info("[%s] async suspend disabled\n", __func__);
}
DECLARE_PCI_FIXUP_FINAL(PCI_ANY_ID, PCI_ANY_ID, exynos_pcie_quirks);

MODULE_AUTHOR("Hongseock Kim <hongpooh.kim@samsung.com>");
MODULE_DESCRIPTION("PCIe phy driver for zuma SoC");
MODULE_LICENSE("GPL v2");
