// SPDX-License-Identifier: GPL-2.0
/*
 * Samsung EXYNOS SoC series USB DRD PHY driver
 *
 * Author: Sung-Hyun Na <sunghyun.na@samsung.com>
 *
 * Chip Abstraction Layer for USB PHY
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include "phy-samsung-usb-cal.h"
#include "phy-exynos-usb3p1-reg.h"
#include "phy-exynos-usbdp-gen2-v4-reg.h"
#include "phy-exynos-usbdp-gen2-v4-reg-pcs.h"

void phy_exynos_usbdp_g2_v2_tune_each(struct exynos_usbphy_info
		*info, char *name, int val)
{
	void __iomem *regs_base = info->pma_base;
	void __iomem *pcs_base = info->pcs_base;
	u32 reg = 0;

	if (!name)
		return;

	if (val == -1)
		return;

	/*
	 * RX Squelch Detect Threshold Control
	 * SQTH
	 * Gen2
	 * 0x0DF8 [3:0]
	 * 0x1DF8 [3:0]
	 * Gen1
	 * 0x0DF8 [7:4]
	 * 0x1DF8 [7:4]
	 * 0000    0mV
	 * 0001	25mV
	 * 0010	40mV
	 * 0011    55mV
	 * 0100    70mV
	 * 0101    85mV
	 * 0110	100mV
	 * 0111	115mV
	 * 1000    130mV
	 * 1001    145mV
	 * 1010	160mV
	 * 1011	175mV
	 * 1100    190mV
	 * 1101    205mV
	 * 1110    220mV
	 * 1111    235mV
	 */
	if (!strcmp(name, "ssrx_sqhs_th_ssp")) {
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG037E);
		reg &= USBDP_TRSV_REG037E_LN0_RX_SQHS_TH_CTRL_SSP_CLR;
		reg |= USBDP_TRSV_REG037E_LN0_RX_SQHS_TH_CTRL_SSP_SET(val);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG037E);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG077E);
		reg &= USBDP_TRSV_REG077E_LN2_RX_SQHS_TH_CTRL_SSP_CLR;
		reg |= USBDP_TRSV_REG077E_LN2_RX_SQHS_TH_CTRL_SSP_SET(val);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG077E);
	} else if (!strcmp(name, "ssrx_sqhs_th_ss")) {
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG037E);
		reg &= USBDP_TRSV_REG037E_LN0_RX_SQHS_TH_CTRL_SP_CLR;
		reg |= USBDP_TRSV_REG037E_LN0_RX_SQHS_TH_CTRL_SP_SET(val);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG037E);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG077E);
		reg &= USBDP_TRSV_REG077E_LN2_RX_SQHS_TH_CTRL_SP_CLR;
		reg |= USBDP_TRSV_REG077E_LN2_RX_SQHS_TH_CTRL_SP_SET(val);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG077E);
	}

	/*
	 * LFPS Detect Threshold Control
	 * LFPS RX
	 * 0x0A0C [3:1]
	 * 0x1A0C [3:1]
	 *         000 60mV
	 *         001 90mV
	 *         010 130mV
	 *         011 160mV
	 *         111 280mV
	 */
	else if (!strcmp(name, "ssrx_lfps_th")) {
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0283);
		reg &= USBDP_TRSV_REG0283_LN0_ANA_RX_LFPS_TH_CTRL_CLR;
		reg |= USBDP_TRSV_REG0283_LN0_ANA_RX_LFPS_TH_CTRL_SET(val);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0283);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0683);
		reg &= USBDP_TRSV_REG0683_LN2_ANA_RX_LFPS_TH_CTRL_CLR;
		reg |= USBDP_TRSV_REG0683_LN2_ANA_RX_LFPS_TH_CTRL_SET(val);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0683);
	}

	/*
	 * Adapation logic off
	 * 0x0AD0=00
	 * 0x1AD0=00
	 *
	 * DFE 1tap Adapation
	 * 0x0AD0=03
	 * 0x1AD0=03
	 *
	 * DFE all-tap Adapation (Recommand)
	 * 0x0AD0=3F
	 * 0x1AD0=3F
	 *
	 * DFE,CTLE Adapation
	 * 0x0AD0=FF
	 * 0x1AD0=FF
	 */
	else if (!strcmp(name, "ssrx_adap_coef_sel")) {
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG02B4);
		reg &= USBDP_TRSV_REG02B4_LN0_RX_SSLMS_ADAP_COEF_SEL__7_0_CLR;
		reg |=
		USBDP_TRSV_REG02B4_LN0_RX_SSLMS_ADAP_COEF_SEL__7_0_SET(val);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG02B4);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG06B4);
		reg &= USBDP_TRSV_REG06B4_LN2_RX_SSLMS_ADAP_COEF_SEL__7_0_CLR;
		reg |=
		USBDP_TRSV_REG06B4_LN2_RX_SSLMS_ADAP_COEF_SEL__7_0_SET(val);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG06B4);
	}

	/*
	 * RX PEQ Control
	 * 0x0AD0[7] = 0 : Tune, 1 : adaptation
	 * 0x1AD0[7]
	 *     Gen1
	 *     0x0A74 [4:0]
	 *     0x1A74 [4:0]
	 *     Gen2
	 *     0x0A78 [4:0]
	 *     0x1A78 [4:0]
	 *             00000   0dB
	 *             ...
	 *             01100   5dB
	 *             ...
	 *             10000   6dB (Max)
	 */
	else if (!strcmp(name, "ssrx_mf_eq_psel_ctrl_ss")) {
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG029D);
		reg &= USBDP_TRSV_REG029D_LN0_RX_SSLMS_MF_INIT_SP_CLR;
		reg |= USBDP_TRSV_REG029D_LN0_RX_SSLMS_MF_INIT_SP_SET(val);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG029D);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG069D);
		reg &= USBDP_TRSV_REG069D_LN2_RX_SSLMS_MF_INIT_SP_CLR;
		reg |= USBDP_TRSV_REG069D_LN2_RX_SSLMS_MF_INIT_SP_SET(val);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG069D);
	} else if (!strcmp(name, "ssrx_mf_eq_psel_ctrl_ssp")) {
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG029E);
		reg &= USBDP_TRSV_REG029E_LN0_RX_SSLMS_MF_INIT_SSP_CLR;
		reg |= USBDP_TRSV_REG029E_LN0_RX_SSLMS_MF_INIT_SSP_SET(val);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG029E);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG069E);
		reg &= USBDP_TRSV_REG069E_LN2_RX_SSLMS_MF_INIT_SSP_CLR;
		reg |= USBDP_TRSV_REG069E_LN2_RX_SSLMS_MF_INIT_SSP_SET(val);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG069E);
	} else if (!strcmp(name, "ssrx_mf_eq_zsel_ctrl_ss")) {
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG025A);
		reg &= USBDP_TRSV_REG025A_LN0_RX_PEQ_Z_CTRL_SP_CLR;
		reg |= USBDP_TRSV_REG025A_LN0_RX_PEQ_Z_CTRL_SP_SET(val);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG025A);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG065A);
		reg &= USBDP_TRSV_REG065A_LN2_RX_PEQ_Z_CTRL_SP_CLR;
		reg |= USBDP_TRSV_REG065A_LN2_RX_PEQ_Z_CTRL_SP_SET(val);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG065A);
	} else if (!strcmp(name, "ssrx_mf_eq_zsel_ctrl_ssp")) {
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG025A);
		reg &= USBDP_TRSV_REG025A_LN0_RX_PEQ_Z_CTRL_SSP_CLR;
		reg |= USBDP_TRSV_REG025A_LN0_RX_PEQ_Z_CTRL_SSP_SET(val);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG025A);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG065A);
		reg &= USBDP_TRSV_REG065A_LN2_RX_PEQ_Z_CTRL_SSP_CLR;
		reg |= USBDP_TRSV_REG065A_LN2_RX_PEQ_Z_CTRL_SSP_SET(val);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG065A);
	}

	/*
	 * RX HF EQ Control
	 * 0x0AD0[6] = 0 : Tune, 1 : adaptation
	 * 0x1AD0[6]
	 *     Gen1
	 *     0x0A5C [4:0]
	 *     0x1A5C [4:0]
	 *     Gen2
	 *     0x0A60 [4:0]
	 *     0x1A60 [4:0]
	 *             00000   0dB
	 *             ...
	 *             01100   15dB
	 *             ...
	 *             10000   20dB (Max)
	 */
	else if (!strcmp(name, "ssrx_hf_eq_rsel_ctrl_ss")) {
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0297);
		reg &= USBDP_TRSV_REG0297_LN0_RX_SSLMS_HF_INIT_SP_CLR;
		reg |= USBDP_TRSV_REG0297_LN0_RX_SSLMS_HF_INIT_SP_SET(val);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0297);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0697);
		reg &= USBDP_TRSV_REG0697_LN2_RX_SSLMS_HF_INIT_SP_CLR;
		reg |= USBDP_TRSV_REG0697_LN2_RX_SSLMS_HF_INIT_SP_SET(val);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0697);
	} else if (!strcmp(name, "ssrx_hf_eq_rsel_ctrl_ssp")) {
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0298);
		reg &= USBDP_TRSV_REG0298_LN0_RX_SSLMS_HF_INIT_SSP_CLR;
		reg |= USBDP_TRSV_REG0298_LN0_RX_SSLMS_HF_INIT_SSP_SET(val);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0298);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0698);
		reg &= USBDP_TRSV_REG0698_LN2_RX_SSLMS_HF_INIT_SSP_CLR;
		reg |= USBDP_TRSV_REG0698_LN2_RX_SSLMS_HF_INIT_SSP_SET(val);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0698);
	} else if (!strcmp(name, "ssrx_hf_eq_csel_ctrl_ss")) {
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG024C);
		reg &= USBDP_TRSV_REG024C_LN0_RX_CTLE_HF_CS_CTRL_SP_CLR;
		reg |= USBDP_TRSV_REG024C_LN0_RX_CTLE_HF_CS_CTRL_SP_SET(val);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG024C);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG064C);
		reg &= USBDP_TRSV_REG064C_LN2_RX_CTLE_HF_CS_CTRL_SP_CLR;
		reg |= USBDP_TRSV_REG064C_LN2_RX_CTLE_HF_CS_CTRL_SP_SET(val);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG064C);
	} else if (!strcmp(name, "ssrx_hf_eq_csel_ctrl_ssp")) {
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG024D);
		reg &= USBDP_TRSV_REG024D_LN0_RX_CTLE_HF_CS_CTRL_SSP_CLR;
		reg |= USBDP_TRSV_REG024D_LN0_RX_CTLE_HF_CS_CTRL_SSP_SET(val);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG024D);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG064D);
		reg &= USBDP_TRSV_REG064D_LN2_RX_CTLE_HF_CS_CTRL_SSP_CLR;
		reg |= USBDP_TRSV_REG064D_LN2_RX_CTLE_HF_CS_CTRL_SSP_SET(val);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG064D);
	}

	/*
	 * DFE 1 tap  Control
	 * 0x0AD0[1] = 0 : Tune, 1 : adaptation
	 * 0x1AD0[1]
	 *     Gen1
	 *     0x0CD0 [6:0]
	 *     0x0D00 [6:0]
	 *     0x1CD0 [6:0]
	 *     0x1D00 [6:0]
	 *     Gen2
	 *     0x0CE8 [6:0]
	 *     0x0D18 [6:0]
	 *     0x1CE8 [6:0]
	 *     0x1D18 [6:0]
	 *             00  0mV
	 *             ...
	 *             04  8mV
	 *             ...
	 *             7F  254mV
	 */
	else if (!strcmp(name, "ssrx_dfe_1tap_ctrl_ss")) {
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0334);
		reg &= USBDP_TRSV_REG0334_LN0_RX_SSLMS_C1_E_INIT_SP_CLR;
		reg |= USBDP_TRSV_REG0334_LN0_RX_SSLMS_C1_E_INIT_SP_SET(val);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0334);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0340);
		reg &= USBDP_TRSV_REG0340_LN0_RX_SSLMS_C1_O_INIT_SP_CLR;
		reg |= USBDP_TRSV_REG0340_LN0_RX_SSLMS_C1_O_INIT_SP_SET(val);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0340);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0734);
		reg &= USBDP_TRSV_REG0734_LN2_RX_SSLMS_C1_E_INIT_SP_CLR;
		reg |= USBDP_TRSV_REG0734_LN2_RX_SSLMS_C1_E_INIT_SP_SET(val);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0734);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0740);
		reg &= USBDP_TRSV_REG0740_LN2_RX_SSLMS_C1_O_INIT_SP_CLR;
		reg |= USBDP_TRSV_REG0740_LN2_RX_SSLMS_C1_O_INIT_SP_SET(val);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0740);
	} else if (!strcmp(name, "ssrx_dfe_1tap_ctrl_ssp")) {
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG033A);
		reg &= USBDP_TRSV_REG033A_LN0_RX_SSLMS_C1_E_INIT_SSP_CLR;
		reg |= USBDP_TRSV_REG033A_LN0_RX_SSLMS_C1_E_INIT_SSP_SET(val);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG033A);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0346);
		reg &= USBDP_TRSV_REG0346_LN0_RX_SSLMS_C1_O_INIT_SSP_CLR;
		reg |= USBDP_TRSV_REG0346_LN0_RX_SSLMS_C1_O_INIT_SSP_SET(val);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0346);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG073A);
		reg &= USBDP_TRSV_REG073A_LN2_RX_SSLMS_C1_E_INIT_SSP_CLR;
		reg |= USBDP_TRSV_REG073A_LN2_RX_SSLMS_C1_E_INIT_SSP_SET(val);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG073A);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0746);
		reg &= USBDP_TRSV_REG0746_LN2_RX_SSLMS_C1_O_INIT_SSP_CLR;
		reg |= USBDP_TRSV_REG0746_LN2_RX_SSLMS_C1_O_INIT_SSP_SET(val);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0746);
	}

	/*
	 * DFE 2 tap  Control
	 * 0x0AD0[2] = 0 : Tune, 1 : adaptation
	 * 0x1AD0[2]
	 *     Gen1
	 *     0x0CD4 [6:0]
	 *     0x0D04 [6:0]
	 *     0x1CD4 [6:0]
	 *     0x1D04 [6:0]
	 *     Gen2
	 *     0x0CEC [6:0]
	 *     0x0D1C [6:0]
	 *     0x1CEC [6:0]
	 *     0x1D1C [6:0]
	 *             40  -240mV (2's complement)
	 *             7F  -3.75mV
	 *             00  0mV
	 *             01  3.75mV
	 *             3F  240mV
	 */
	else if (!strcmp(name, "ssrx_dfe_2tap_ctrl_ss")) {
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0335);
		reg &= USBDP_TRSV_REG0335_LN0_RX_SSLMS_C2_E_INIT_SP_CLR;
		reg |= USBDP_TRSV_REG0335_LN0_RX_SSLMS_C2_E_INIT_SP_SET(val);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0335);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0341);
		reg &= USBDP_TRSV_REG0341_LN0_RX_SSLMS_C2_O_INIT_SP_CLR;
		reg |= USBDP_TRSV_REG0341_LN0_RX_SSLMS_C2_O_INIT_SP_SET(val);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0341);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0735);
		reg &= USBDP_TRSV_REG0735_LN2_RX_SSLMS_C2_E_INIT_SP_CLR;
		reg |= USBDP_TRSV_REG0735_LN2_RX_SSLMS_C2_E_INIT_SP_SET(val);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0735);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0741);
		reg &= USBDP_TRSV_REG0741_LN2_RX_SSLMS_C2_O_INIT_SP_CLR;
		reg |= USBDP_TRSV_REG0741_LN2_RX_SSLMS_C2_O_INIT_SP_SET(val);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0741);
	} else if (!strcmp(name, "ssrx_dfe_2tap_ctrl_ssp")) {
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG033B);
		reg &= USBDP_TRSV_REG033B_LN0_RX_SSLMS_C2_E_INIT_SSP_CLR;
		reg |= USBDP_TRSV_REG033B_LN0_RX_SSLMS_C2_E_INIT_SSP_SET(val);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG033B);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0347);
		reg &= USBDP_TRSV_REG0347_LN0_RX_SSLMS_C2_O_INIT_SSP_CLR;
		reg |= USBDP_TRSV_REG0347_LN0_RX_SSLMS_C2_O_INIT_SSP_SET(val);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0347);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG073B);
		reg &= USBDP_TRSV_REG073B_LN2_RX_SSLMS_C2_E_INIT_SSP_CLR;
		reg |= USBDP_TRSV_REG073B_LN2_RX_SSLMS_C2_E_INIT_SSP_SET(val);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG073B);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0747);
		reg &= USBDP_TRSV_REG0747_LN2_RX_SSLMS_C2_O_INIT_SSP_CLR;
		reg |= USBDP_TRSV_REG0747_LN2_RX_SSLMS_C2_O_INIT_SSP_SET(val);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0747);
	}

	/*
	 * DFE 3 tap  Control
	 * 0x0AD0[3] = 0 : Tune, 1 : adapation
	 * 0x1AD0[3]
	 *     Gen1
	 *     0x0CD8 [6:0]
	 *     0x0D08 [6:0]
	 *     0x1CD8 [6:0]
	 *     0x1D08 [6:0]
	 *     Gen2
	 *     0x0CF0 [6:0]
	 *     0x0D20 [6:0]
	 *     0x1CF0 [6:0]
	 *     0x1D20 [6:0]
	 *             40  -240mV (2's complement)
	 *             7F  -3.75mV
	 *             00  0mV
	 *             01  3.75mV
	 *             3F  240mV
	 */
	else if (!strcmp(name, "ssrx_dfe_3tap_ctrl_ss")) {
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0336);
		reg &= USBDP_TRSV_REG0336_LN0_RX_SSLMS_C3_E_INIT_SP_CLR;
		reg |= USBDP_TRSV_REG0336_LN0_RX_SSLMS_C3_E_INIT_SP_SET(val);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0336);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0342);
		reg &= USBDP_TRSV_REG0342_LN0_RX_SSLMS_C3_O_INIT_SP_CLR;
		reg |= USBDP_TRSV_REG0342_LN0_RX_SSLMS_C3_O_INIT_SP_SET(val);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0342);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0736);
		reg &= USBDP_TRSV_REG0736_LN2_RX_SSLMS_C3_E_INIT_SP_CLR;
		reg |= USBDP_TRSV_REG0736_LN2_RX_SSLMS_C3_E_INIT_SP_SET(val);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0736);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0742);
		reg &= USBDP_TRSV_REG0742_LN2_RX_SSLMS_C3_O_INIT_SP_CLR;
		reg |= USBDP_TRSV_REG0742_LN2_RX_SSLMS_C3_O_INIT_SP_SET(val);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0742);
	} else if (!strcmp(name, "ssrx_dfe_3tap_ctrl_ssp")) {
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG033C);
		reg &= USBDP_TRSV_REG033C_LN0_RX_SSLMS_C3_E_INIT_SSP_CLR;
		reg |= USBDP_TRSV_REG033C_LN0_RX_SSLMS_C3_E_INIT_SSP_SET(val);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG033C);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0348);
		reg &= USBDP_TRSV_REG0348_LN0_RX_SSLMS_C3_O_INIT_SSP_CLR;
		reg |= USBDP_TRSV_REG0348_LN0_RX_SSLMS_C3_O_INIT_SSP_SET(val);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0348);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG073C);
		reg &= USBDP_TRSV_REG073C_LN2_RX_SSLMS_C3_E_INIT_SSP_CLR;
		reg |= USBDP_TRSV_REG073C_LN2_RX_SSLMS_C3_E_INIT_SSP_SET(val);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG073C);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0748);
		reg &= USBDP_TRSV_REG0748_LN2_RX_SSLMS_C3_O_INIT_SSP_CLR;
		reg |= USBDP_TRSV_REG0748_LN2_RX_SSLMS_C3_O_INIT_SSP_SET(val);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0748);
	}

	/*
	 * DFE 4 tap  Control
	 * 0x0AD0[4] = 0 : Tune, 1 : adapation
	 * 0x1AD0[4]
	 *     Gen1
	 *     0x0CDC [5:0]
	 *     0x0D0C [5:0]
	 *     0x1CDC [5:0]
	 *     0x1D0C [5:0]
	 *     Gen2
	 *     0x0CF4 [5:0]
	 *     0x0D24 [5:0]
	 *     0x1CF4 [5:0]
	 *     0x1D24 [5:0]
	 *             20  -120mV (2's complement)
	 *             3F  -3.75mV
	 *             00  0mV
	 *             01  3.75mV
	 *             1F  120mV
	 */
	else if (!strcmp(name, "ssrx_dfe_4tap_ctrl_ss")) {
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0337);
		reg &= USBDP_TRSV_REG0337_LN0_RX_SSLMS_C4_E_INIT_SP_CLR;
		reg |= USBDP_TRSV_REG0337_LN0_RX_SSLMS_C4_E_INIT_SP_SET(val);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0337);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0343);
		reg &= USBDP_TRSV_REG0343_LN0_RX_SSLMS_C4_O_INIT_SP_CLR;
		reg |= USBDP_TRSV_REG0343_LN0_RX_SSLMS_C4_O_INIT_SP_SET(val);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0343);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0737);
		reg &= USBDP_TRSV_REG0737_LN2_RX_SSLMS_C4_E_INIT_SP_CLR;
		reg |= USBDP_TRSV_REG0737_LN2_RX_SSLMS_C4_E_INIT_SP_SET(val);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0737);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0743);
		reg &= USBDP_TRSV_REG0743_LN2_RX_SSLMS_C4_O_INIT_SP_CLR;
		reg |= USBDP_TRSV_REG0743_LN2_RX_SSLMS_C4_O_INIT_SP_SET(val);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0743);
	} else if (!strcmp(name, "ssrx_dfe_4tap_ctrl_ssp")) {
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG033D);
		reg &= USBDP_TRSV_REG033D_LN0_RX_SSLMS_C4_E_INIT_SSP_CLR;
		reg |= USBDP_TRSV_REG033D_LN0_RX_SSLMS_C4_E_INIT_SSP_SET(val);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG033D);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0349);
		reg &= USBDP_TRSV_REG0349_LN0_RX_SSLMS_C4_O_INIT_SSP_CLR;
		reg |= USBDP_TRSV_REG0349_LN0_RX_SSLMS_C4_O_INIT_SSP_SET(val);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0349);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG073D);
		reg &= USBDP_TRSV_REG073D_LN2_RX_SSLMS_C4_E_INIT_SSP_CLR;
		reg |= USBDP_TRSV_REG073D_LN2_RX_SSLMS_C4_E_INIT_SSP_SET(val);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG073D);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0749);
		reg &= USBDP_TRSV_REG0749_LN2_RX_SSLMS_C4_O_INIT_SSP_CLR;
		reg |= USBDP_TRSV_REG0749_LN2_RX_SSLMS_C4_O_INIT_SSP_SET(val);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0749);
	}

	/*
	 * DFE 5 tap  Control
	 * 0x0AD0[5] = 0 : Tune, 1 : adaptation
	 * 0x1AD0[5]
	 *     Gen1
	 *     0x0CE0 [5:0]
	 *     0x0D10 [5:0]
	 *     0x1CE0 [5:0]
	 *     0x1D10 [5:0]
	 *     Gen2
	 *     0x0CF8 [5:0]
	 *     0x0D28 [7:2]
	 *     0x1CF8 [5:0]
	 *     0x1D28 [7:2]
	 *             20  -120mV (2's complement)
	 *             3F  -3.75mV
	 *             00  0mV
	 *             01  3.75mV
	 *             1F  120mV
	 */
	else if (!strcmp(name, "ssrx_dfe_5tap_ctrl_ss")) {
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0338);
		reg &= USBDP_TRSV_REG0338_LN0_RX_SSLMS_C5_E_INIT_SP_CLR;
		reg |= USBDP_TRSV_REG0338_LN0_RX_SSLMS_C5_E_INIT_SP_SET(val);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0338);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0344);
		reg &= USBDP_TRSV_REG0344_LN0_RX_SSLMS_C5_O_INIT_SP_CLR;
		reg |= USBDP_TRSV_REG0344_LN0_RX_SSLMS_C5_O_INIT_SP_SET(val);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0344);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0738);
		reg &= USBDP_TRSV_REG0738_LN2_RX_SSLMS_C5_E_INIT_SP_CLR;
		reg |= USBDP_TRSV_REG0738_LN2_RX_SSLMS_C5_E_INIT_SP_SET(val);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0738);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0744);
		reg &= USBDP_TRSV_REG0744_LN2_RX_SSLMS_C5_O_INIT_SP_CLR;
		reg |= USBDP_TRSV_REG0744_LN2_RX_SSLMS_C5_O_INIT_SP_SET(val);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0744);
	} else if (!strcmp(name, "ssrx_dfe_5tap_ctrl_ssp")) {
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG033E);
		reg &= USBDP_TRSV_REG033E_LN0_RX_SSLMS_C5_E_INIT_SSP_CLR;
		reg |= USBDP_TRSV_REG033E_LN0_RX_SSLMS_C5_E_INIT_SSP_SET(val);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG033E);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG034A);
		reg &= USBDP_TRSV_REG034A_LN0_RX_SSLMS_C5_O_INIT_SSP_CLR;
		reg |= USBDP_TRSV_REG034A_LN0_RX_SSLMS_C5_O_INIT_SSP_SET(val);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG034A);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG073E);
		reg &= USBDP_TRSV_REG073E_LN2_RX_SSLMS_C5_E_INIT_SSP_CLR;
		reg |= USBDP_TRSV_REG073E_LN2_RX_SSLMS_C5_E_INIT_SSP_SET(val);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG073E);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG074A);
		reg &= USBDP_TRSV_REG074A_LN2_RX_SSLMS_C5_O_INIT_SSP_CLR;
		reg |= USBDP_TRSV_REG074A_LN2_RX_SSLMS_C5_O_INIT_SSP_SET(val);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG074A);
	}

	/*
	 * CDR BW Control
	 * Gen1
	 * 0x0EB8 [5:0]: 0x0 : Min, 0x3F : Max
	 * 0x1EB8 [5:0]
	 * Gen2
	 * 0x0EBC [5:0]: 0x0 : Min, 0x3F : Max
	 * 0x1EBC [5:0]
	 */
	else if (!strcmp(name, "ssrx_cdr_fbb_fine_ctrl_sp")) {
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG03AE);
		reg &= USBDP_TRSV_REG03AE_LN0_RX_CDR_FBB_FINE_CTRL_SP_CLR;
		reg |= USBDP_TRSV_REG03AE_LN0_RX_CDR_FBB_FINE_CTRL_SP_SET(val);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG03AE);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG07AE);
		reg &= USBDP_TRSV_REG07AE_LN2_RX_CDR_FBB_FINE_CTRL_SP_CLR;
		reg |= USBDP_TRSV_REG07AE_LN2_RX_CDR_FBB_FINE_CTRL_SP_SET(val);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG07AE);
	} else if (!strcmp(name, "ssrx_cdr_fbb_fine_ctrl_ssp")) {
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG03AF);
		reg &= USBDP_TRSV_REG03AF_LN0_RX_CDR_FBB_FINE_CTRL_SSP_CLR;
		reg |= USBDP_TRSV_REG03AF_LN0_RX_CDR_FBB_FINE_CTRL_SSP_SET(val);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG03AF);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG07AF);
		reg &= USBDP_TRSV_REG07AF_LN2_RX_CDR_FBB_FINE_CTRL_SSP_CLR;
		reg |= USBDP_TRSV_REG07AF_LN2_RX_CDR_FBB_FINE_CTRL_SSP_SET(val);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG07AF);
	}

	/*
	 * RX Termination
	 * 0x0BB0 [1:0]: 1 : Tune, 0 : calibration
	 * 0x1BB0 [1:0]
	 *   0x0BB4 [7:4]
	 *   0x1BB4 [7:4]
	 *           0000    57 Ohm
	 *           0001    57 Ohm
	 *           0010    53 Ohm
	 *           0011    44 Ohm
	 *           1111    37 Ohm
	 */
	else if (!strcmp(name, "ssrx_term_cal")) {
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG02EC);
		reg &= USBDP_TRSV_REG02EC_LN0_RX_RCAL_OPT_CODE_CLR;
		reg |= USBDP_TRSV_REG02EC_LN0_RX_RCAL_OPT_CODE_SET(1);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG02EC);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG02ED);
		reg &= USBDP_TRSV_REG02ED_LN0_RX_RTERM_CTRL_CLR;
		reg |= USBDP_TRSV_REG02ED_LN0_RX_RTERM_CTRL_SET(val);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG02ED);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG06EC);
		reg &= USBDP_TRSV_REG06EC_LN2_RX_RCAL_OPT_CODE_CLR;
		reg |= USBDP_TRSV_REG06EC_LN2_RX_RCAL_OPT_CODE_SET(1);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG06EC);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG06ED);
		reg &= USBDP_TRSV_REG06ED_LN2_RX_RTERM_CTRL_CLR;
		reg |= USBDP_TRSV_REG06ED_LN2_RX_RTERM_CTRL_SET(val);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG06ED);
	}

	/*
	 * Gen1 Tx DRIVER pre-shoot, de-emphasis, level ctrl
	 * [17:12] Deemphasis
	 * [11:6] Level
	 * [5:0] Preshoot
	 *
	 */
	else if (!strcmp(name, "sstx_amp_ss")) {
		reg = readl(pcs_base + EXYNOS_USBDP_PCS_LEQ_HS_TX_COEF_MAP_0);
		reg &= ~(0x00FC0);
		reg |= val << 6;
		writel(reg, pcs_base + EXYNOS_USBDP_PCS_LEQ_HS_TX_COEF_MAP_0);
	}

	else if (!strcmp(name, "sstx_deemp_ss")) {
		reg = readl(pcs_base + EXYNOS_USBDP_PCS_LEQ_HS_TX_COEF_MAP_0);
		reg &= ~(0x3F000);
		reg |= val << 12;
		writel(reg, pcs_base + EXYNOS_USBDP_PCS_LEQ_HS_TX_COEF_MAP_0);
	}

	else if (!strcmp(name, "sstx_pre_shoot_ss")) {
		reg = readl(pcs_base + EXYNOS_USBDP_PCS_LEQ_HS_TX_COEF_MAP_0);
		reg &= ~(0x0003F);
		reg |= val;
		writel(reg, pcs_base + EXYNOS_USBDP_PCS_LEQ_HS_TX_COEF_MAP_0);
	}

	/*
	 * Gen2 Tx DRIVER level ctrl
	 * [17:12] Deemphasis
	 * [11:6] Level
	 * [5:0] Preshoot
	 *
	 */
	else if (!strcmp(name, "sstx_amp_ssp")) {
		reg = readl(pcs_base + EXYNOS_USBDP_PCS_LEQ_LOCAL_COEF);
		reg &= USBDP_PCS_LEQ_LOCAL_COEF_PMA_CENTER_COEF_CLR;
		reg |= USBDP_PCS_LEQ_LOCAL_COEF_PMA_CENTER_COEF_SET(val);
		writel(reg, pcs_base + EXYNOS_USBDP_PCS_LEQ_LOCAL_COEF);
	}

	/*
	 * TX IDRV UP
	 * 0x101C [7]: 1: Tune, 0 : From PnR logic
	 * 0x201C [7]
	 *   0x101C [6:4]
	 *   0x201C [6:4]
	 *           111 0.85V
	 *           110 1V
	 *           011 1.1V
	 *           ...
	 *           000 1.2V ( Max )
	 */
	else if (!strcmp(name, "sstx_idrv_up")) {
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0407);
		reg &= USBDP_TRSV_REG0407_OVRD_LN1_TX_DRV_IDRV_IUP_CTRL_CLR;
		reg |= USBDP_TRSV_REG0407_OVRD_LN1_TX_DRV_IDRV_IUP_CTRL_SET(1);
		reg &= USBDP_TRSV_REG0407_LN1_TX_DRV_IDRV_IUP_CTRL_CLR;
		reg |= USBDP_TRSV_REG0407_LN1_TX_DRV_IDRV_IUP_CTRL_SET(val);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0407);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0807);
		reg &= USBDP_TRSV_REG0807_OVRD_LN3_TX_DRV_IDRV_IUP_CTRL_CLR;
		reg |= USBDP_TRSV_REG0807_OVRD_LN3_TX_DRV_IDRV_IUP_CTRL_SET(1);
		reg &= USBDP_TRSV_REG0807_LN3_TX_DRV_IDRV_IUP_CTRL_CLR;
		reg |= USBDP_TRSV_REG0807_LN3_TX_DRV_IDRV_IUP_CTRL_SET(val);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0807);
	}

	/*
	 * TX IDRV UP for LFPS TX
	 * It is valid where
	 * 0x101C [7]: 0x201C [7] = 0 (Default setting)
	 *   0x0408 [6:4]
	 *           111 0.85V
	 *           110 1V
	 *           011 1.1V
	 *           ...
	 *           000 1.2V ( Max )
	 */
	else if (!strcmp(name, "sstx_lfps_idrv_up")) {
		reg = readl(regs_base + EXYNOS_USBDP_CMN_REG0102);
		reg &= USBDP_CMN_REG0102_TX_DRV_LFPS_MODE_IDRV_IUP_CTRL_CLR;
		reg |=
		USBDP_CMN_REG0102_TX_DRV_LFPS_MODE_IDRV_IUP_CTRL_SET(val);
		writel(reg, regs_base + EXYNOS_USBDP_CMN_REG0102);
	}

	/*
	 * TX UP Termination
	 * 0x10B0 [3:2]: 1 : Tune, 0 : calibration
	 * 0x20B0 [3:2]
	 *   0x10B4 [7:4]
	 *   0x20B4 [7:4]
	 *           0000    57 Ohm
	 *           0001    57 Ohm
	 *           0010    53 Ohm
	 *           0011    44 Ohm
	 *           1111    37 Ohm
	 */
	else if (!strcmp(name, "sstx_up_term")) {
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG042C);
		reg &= USBDP_TRSV_REG042C_LN1_TX_RCAL_UP_OPT_CODE_CLR;
		reg |= USBDP_TRSV_REG042C_LN1_TX_RCAL_UP_OPT_CODE_SET(1);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG042C);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG042D);
		reg &= USBDP_TRSV_REG042D_LN1_TX_RCAL_UP_CODE_CLR;
		reg |= USBDP_TRSV_REG042D_LN1_TX_RCAL_UP_CODE_SET(val);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG042D);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG082C);
		reg &= USBDP_TRSV_REG082C_LN3_TX_RCAL_UP_OPT_CODE_CLR;
		reg |= USBDP_TRSV_REG082C_LN3_TX_RCAL_UP_OPT_CODE_SET(1);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG082C);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG082D);
		reg &= USBDP_TRSV_REG082D_LN3_TX_RCAL_UP_CODE_CLR;
		reg |= USBDP_TRSV_REG082D_LN3_TX_RCAL_UP_CODE_SET(val);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG082D);
	}

	/*
	 * TX DN Termination
	 * 0x10B0 [1:0]: 1 : Tune, 0 : calibration
	 * 0x20B0 [1:0]
	 *   0x10B4 [3:0]
	 *   0x20B4 [3:0]
	 *           0000    57 Ohm
	 *           0001    57 Ohm
	 *           0010    53 Ohm
	 *           0011    44 Ohm
	 *           1111    37 Ohm
	 */
	else if (!strcmp(name, "sstx_dn_term")) {
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG042C);
		reg &= USBDP_TRSV_REG042C_LN1_TX_RCAL_DN_OPT_CODE_CLR;
		reg |= USBDP_TRSV_REG042C_LN1_TX_RCAL_DN_OPT_CODE_SET(1);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG042C);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG042D);
		reg &= USBDP_TRSV_REG042D_LN1_TX_RCAL_DN_CODE_CLR;
		reg |= USBDP_TRSV_REG042D_LN1_TX_RCAL_DN_CODE_SET(val);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG042D);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG082C);
		reg &= USBDP_TRSV_REG082C_LN3_TX_RCAL_DN_OPT_CODE_CLR;
		reg |= USBDP_TRSV_REG082C_LN3_TX_RCAL_DN_OPT_CODE_SET(1);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG082C);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG082D);
		reg &= USBDP_TRSV_REG082D_LN3_TX_RCAL_DN_CODE_CLR;
		reg |= USBDP_TRSV_REG082D_LN3_TX_RCAL_DN_CODE_SET(val);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG082D);
	}

	/* REXT ovrd */
	else if (!strcmp(name, "rext_ovrd")) {
		reg = readl(regs_base + EXYNOS_USBDP_CMN_REG0006);
		reg |= USBDP_CMN_REG0006_OVRD_BIAS_ICAL_CODE_SET(1);
		reg &= USBDP_CMN_REG0006_BIAS_ICAL_CODE_CLR;
		reg |= USBDP_CMN_REG0006_BIAS_ICAL_CODE_SET(val);
		writel(reg, regs_base + EXYNOS_USBDP_CMN_REG0006);
	}
}

static void
phy_exynos_usbdp_g2_v4_ctrl_pma_ready(struct exynos_usbphy_info
		*info)
{
	void __iomem *regs_base = info->ctrl_base;
	u32 reg;

	/* link pipe_clock selection to pclk of PMA */
	reg = readl(regs_base + EXYNOS_USBCON_CLKRST);
	reg |= CLKRST_LINK_PCLK_SEL;
	writel(reg, regs_base + EXYNOS_USBCON_CLKRST);

	reg = readl(regs_base + EXYNOS_USBCON_COMBO_PMA_CTRL);
	reg |= PMA_REF_FREQ_SEL_SET(1);
	/* SFR reset */
	reg |= PMA_LOW_PWR;
	reg |= PMA_APB_SW_RST;
	/* reference clock 26MHz path from XTAL */
	reg &= ~PMA_ROPLL_REF_CLK_SEL_MASK;
	reg &= ~PMA_LCPLL_REF_CLK_SEL_MASK;
	/* PMA_POWER_OFF */
	reg |= PMA_TRSV_SW_RST;
	reg |= PMA_CMN_SW_RST;
	reg |= PMA_INIT_SW_RST;
	writel(reg, regs_base + EXYNOS_USBCON_COMBO_PMA_CTRL);

	udelay(1);

	reg = readl(regs_base + EXYNOS_USBCON_COMBO_PMA_CTRL);
	reg &= ~PMA_LOW_PWR;
	writel(reg, regs_base + EXYNOS_USBCON_COMBO_PMA_CTRL);

	/* APB enable */
	reg = readl(regs_base + EXYNOS_USBCON_COMBO_PMA_CTRL);
	reg &= ~PMA_APB_SW_RST;
	writel(reg, regs_base + EXYNOS_USBCON_COMBO_PMA_CTRL);
}

static void
phy_exynos_usbdp_g2_v4_ctrl_pma_rst_release(struct exynos_usbphy_info
		*info)
{
	void __iomem *regs_base = info->ctrl_base;
	u32 reg;

	/* Reset release from port */
	reg = readl(regs_base + EXYNOS_USBCON_COMBO_PMA_CTRL);
	reg &= ~PMA_TRSV_SW_RST;
	reg &= ~PMA_CMN_SW_RST;
	reg &= ~PMA_INIT_SW_RST;
	writel(reg, regs_base + EXYNOS_USBCON_COMBO_PMA_CTRL);
}

static void
phy_exynos_usbdp_g2_v4_aux_force_off(struct exynos_usbphy_info
		*info)
{
	void __iomem *regs_base = info->pma_base;
	u32 reg;

	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG0008);
	reg &= USBDP_CMN_REG0008_OVRD_AUX_EN_CLR;
	reg |= USBDP_CMN_REG0008_OVRD_AUX_EN_SET(1);
	reg &= USBDP_CMN_REG0008_AUX_EN_CLR;
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG0008);
}

static inline void
phy_exynos_usbdp_g2_v4_pma_default_sfr_update(struct exynos_usbphy_info
		*info)
{
	void __iomem *regs_base = info->pma_base;
	u32 reg = 0;

	/* 190627: CDR data mode exit GEN1 ON / GEN2 OFF */
	writel(0xff, regs_base + 0x0c8c);
	writel(0xff, regs_base + 0x1c8c);
	writel(0x7d, regs_base + 0x0c9c);
	writel(0x7d, regs_base + 0x1c9c);

	/* 190708: Setting for improvement of EDS distribution */
	writel(0x06, regs_base + 0x0e7c);
	writel(0x00, regs_base + 0x09e0);
	writel(0x36, regs_base + 0x09e4);
	writel(0x06, regs_base + 0x1e7c);
	writel(0x00, regs_base + 0x1e90);
	writel(0x36, regs_base + 0x1e94);

	/* 190707: Improve LVCC */
	writel(0x30, regs_base + 0x08f0);
	writel(0x30, regs_base + 0x18f0);

	/* 190820: LFPS RX VIH shmoo hole */
	writel(0x0c, regs_base + 0x0a08);
	writel(0x0c, regs_base + 0x1a08);

	/* 190820: Remove unrelated option for v4 phy */
	writel(0x05, regs_base + 0x0a0c);
	writel(0x05, regs_base + 0x1a0c);

	/* 190903: Improve Gen2 LVCC */
	writel(0x1c, regs_base + 0x00f8);
	writel(0x54, regs_base + 0x00fc);

	/* 191128: Change Vth of RCV_DET because of
	 * TD 7.40 Polling Retry Test
	 */
	writel(0x07, regs_base + 0x104c);
	writel(0x07, regs_base + 0x204c);

	/*
	 * reduce Ux Exit time, Recovery.Active(TS1) n x REFCLK_PERIOD(38.4ns)
	 * 0x0CA8  0x00    ln0_rx_valid_rstn_delay_rise_sp__15_8=0
	 * 0x0CAC  0x04    ln0_rx_valid_rstn_delay_rise_sp__7_0=4
	 * 0x1CA8  0x00    ln2_rx_valid_rstn_delay_rise_sp__15_8=0
	 * 0x1CAC  0x04    ln2_rx_valid_rstn_delay_rise_sp__7_0=4
	 * 0x0CB8  0x00    ln0_rx_valid_rstn_delay_rise_ssp__15_8=0
	 * 0x0CBC  0x04    ln0_rx_valid_rstn_delay_rise_ssp__7_0=4
	 * 0x1CB8  0x00    ln2_rx_valid_rstn_delay_rise_ssp__15_8=0
	 * 0x1CBC  0x04    ln2_rx_valid_rstn_delay_rise_ssp__7_0=4
	 */
	/* Gen1 */
	writel(0x00, regs_base + 0x0ca8);
	writel(0x04, regs_base + 0x0cac);
	writel(0x00, regs_base + 0x1ca8);
	writel(0x04, regs_base + 0x1cac);
	/* Gen2 */
	writel(0x00, regs_base + 0x0cb8);
	writel(0x04, regs_base + 0x0cbc);
	writel(0x00, regs_base + 0x1cb8);
	writel(0x04, regs_base + 0x1cbc);

	/* RX impedance setting
	 * 0x0BB0  0x01    rx_rcal_opt_code=1
	 * 0x0BB4  0xE0    rx_rterm_ctrl=E
	 * 0x1BB0  0x01    rx_rcal_opt_code=1
	 * 0x1BB4  0xE0    rx_rterm_ctrl=E
	 */
	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG02EC);
	reg &= USBDP_TRSV_REG02EC_LN0_RX_RCAL_OPT_CODE_CLR;
	reg |= USBDP_TRSV_REG02EC_LN0_RX_RCAL_OPT_CODE_SET(1);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG02EC);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG02ED);
	reg &= USBDP_TRSV_REG02ED_LN0_RX_RTERM_CTRL_CLR;
	reg |= USBDP_TRSV_REG02ED_LN0_RX_RTERM_CTRL_SET(0xa);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG02ED);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG06EC);
	reg &= USBDP_TRSV_REG06EC_LN2_RX_RCAL_OPT_CODE_CLR;
	reg |= USBDP_TRSV_REG06EC_LN2_RX_RCAL_OPT_CODE_SET(1);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG06EC);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG06ED);
	reg &= USBDP_TRSV_REG06ED_LN2_RX_RTERM_CTRL_CLR;
	reg |= USBDP_TRSV_REG06ED_LN2_RX_RTERM_CTRL_SET(0xa);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG06ED);
}

static inline void phy_exynos_usbdp_g2_v4_pma_default_sfr_update_19_2Mhz(struct exynos_usbphy_info *info)
{
	void __iomem *regs_base = info->pma_base;

	/* 190627: CDR data mode exit GEN1 ON / GEN2 OFF */
	writel(0xff, regs_base + 0x0c8c);
	writel(0xff, regs_base + 0x1c8c);
	writel(0x7d, regs_base + 0x0c9c);
	writel(0x7d, regs_base + 0x1c9c);

	/* 190708: Setting for improvement of EDS distribution */
	writel(0x06, regs_base + 0x0e7c);
	writel(0x00, regs_base + 0x09e0);
	writel(0x35, regs_base + 0x09e4);
	writel(0x06, regs_base + 0x1e7c);
	writel(0x00, regs_base + 0x19e0);
	writel(0x35, regs_base + 0x19e4);

	/* 190707: Improve LVCC */
	writel(0x30, regs_base + 0x08f0);
	writel(0x30, regs_base + 0x18f0);

	/* 190820: LFPS RX VIH shmoo hole */
	writel(0x0c, regs_base + 0x0a08);
	writel(0x0c, regs_base + 0x1a08);

	/* 190820: Remove unrelated option for v4 phy */
	writel(0x05, regs_base + 0x0a0c);
	writel(0x05, regs_base + 0x1a0c);

	/* 210521: Improve USB Gen2 LVCC  */
	writel(0x1c, regs_base + 0x00f8);
	writel(0x54, regs_base + 0x00fc);

	/* 191128: Change Vth of RCV_DET because of TD 7.40 Polling Retry Test */
	writel(0x05, regs_base + 0x104c);
	writel(0x05, regs_base + 0x204c);

	/* Gen1 */
	writel(0x00, regs_base + 0x0ca8);
	writel(0x00, regs_base + 0x1ca8);
	writel(0x04, regs_base + 0x0cac);
	writel(0x04, regs_base + 0x1cac);

	/* Gen2 */
	writel(0x00, regs_base + 0x0cb8);
	writel(0x00, regs_base + 0x1cb8);
	writel(0x04, regs_base + 0x0cbc);
	writel(0x04, regs_base + 0x1cbc);

	/* CDR Lock Delay for JTOL Link Training */
	writel(0x03, regs_base + 0x0320);
	writel(0x23, regs_base + 0x0324);

	/* common reset for 4lane tx */
	writel(0x63, regs_base + 0x0858);
	writel(0x63, regs_base + 0x1858);
	writel(0x63, regs_base + 0x1058);
	writel(0x63, regs_base + 0x2058);

	/*
	* Change rx rterm to 90 ohm
	*/
	writel(0xA0, regs_base + 0x0BB4);
	writel(0xA0, regs_base + 0x1BB4);

	/*
	* added setting
	*/
	writel(0x0C, regs_base + 0x0A08);
	writel(0x0C, regs_base + 0x1A08);

	writel(0x2B, regs_base + 0x0DEC);
	writel(0x2B, regs_base + 0x1DEC);

	/* change rx_hf_cs_ctrl[7:6] */
	writel(0x7F, regs_base + 0x0934);
	writel(0x7F, regs_base + 0x1934);
	/* change rx_vga_rl_ctrl[5:3] */
	writel(0x32, regs_base + 0x0948);
	writel(0x32, regs_base + 0x1948);
	/* change vga_bin_ssp[5:1] */
	writel(0x00, regs_base + 0x0DF4);
	writel(0x00, regs_base + 0x1DF4);

	writel(0x1D, regs_base + 0x091C);
	writel(0x1D, regs_base + 0x191C);

	writel(0x0C, regs_base + 0x0928);
	writel(0x0C, regs_base + 0x1928);

	writel(0x3C, regs_base + 0x0E0C);
	writel(0x3C, regs_base + 0x1E0C);

	writel(0x04, regs_base + 0x0EBC);
	writel(0x04, regs_base + 0x1EBC);

	writel(0x10, regs_base + 0x0908);
	writel(0x10, regs_base + 0x1908);

}

static inline void phy_exynos_usbdp_g2_v4_pma_default_sfr_update_19_2Mhz_Gen2(struct exynos_usbphy_info *info)
{
	void __iomem *regs_base = info->pma_base;
	u32 reg;

	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG0101);
	reg &= USBDP_CMN_REG0101_TIME_CDR_WATCHDOG_WAIT_RESTART__7_0_CLR;
	reg |= USBDP_CMN_REG0101_TIME_CDR_WATCHDOG_WAIT_RESTART__7_0_SET(0xBB);
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG0101);

	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG012C);
	reg &= USBDP_CMN_REG012C_RX_LFPS_DET_FILT_TH3_SHORT_RISE_SP_CLR;
	reg |= USBDP_CMN_REG012C_RX_LFPS_DET_FILT_TH3_SHORT_RISE_SP_SET(0x03);
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG012C);

	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG012D);
	reg &= USBDP_CMN_REG012D_RX_LFPS_DET_FILT_TH3_SHORT_FALL_SP_CLR;
	reg |= USBDP_CMN_REG012D_RX_LFPS_DET_FILT_TH3_SHORT_FALL_SP_SET(0x3);
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG012D);

	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG0134);
	reg &= USBDP_CMN_REG0134_RX_LFPS_DET_FILT_TH3_SHORT_RISE_SSP_CLR;
	reg |= USBDP_CMN_REG0134_RX_LFPS_DET_FILT_TH3_SHORT_RISE_SSP_SET(0x3);
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG0134);

	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG0135);
	reg &= USBDP_CMN_REG0135_RX_LFPS_DET_FILT_TH3_SHORT_FALL_SSP_CLR;
	reg |= USBDP_CMN_REG0135_RX_LFPS_DET_FILT_TH3_SHORT_FALL_SSP_SET(0x3);
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG0135);

	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG013C);
	reg &= USBDP_CMN_REG013C_RX_LFPS_DET_FILT_TH3_LONG_RISE_SP_CLR;
	reg |= USBDP_CMN_REG013C_RX_LFPS_DET_FILT_TH3_LONG_RISE_SP_SET(0x0F);
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG013C);

	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG013D);
	reg &= USBDP_CMN_REG013D_RX_LFPS_DET_FILT_TH3_LONG_FALL_SP_CLR;
	reg |= USBDP_CMN_REG013D_RX_LFPS_DET_FILT_TH3_LONG_FALL_SP_SET(0x0F);
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG013D);

	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG0144);
	reg &= USBDP_CMN_REG0144_RX_LFPS_DET_FILT_TH3_LONG_RISE_SSP_CLR;
	reg |= USBDP_CMN_REG0144_RX_LFPS_DET_FILT_TH3_LONG_RISE_SSP_SET(0x0F);
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG0144);

	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG0145);
	reg &= USBDP_CMN_REG0145_RX_LFPS_DET_FILT_TH3_LONG_FALL_SSP_CLR;
	reg |= USBDP_CMN_REG0145_RX_LFPS_DET_FILT_TH3_LONG_FALL_SSP_SET(0x0F);
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG0145);

	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG016A);
	reg &= USBDP_CMN_REG016A_TX_LFPS_TP_GEN_T_PWM_CLR;
	reg |= USBDP_CMN_REG016A_TX_LFPS_TP_GEN_T_PWM_SET(0x2B);
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG016A);

	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG016B);
	reg &= USBDP_CMN_REG016B_TX_LFPS_TP_GEN_T_LFPS0_CLR;
	reg |= USBDP_CMN_REG016B_TX_LFPS_TP_GEN_T_LFPS0_SET(0x0F);
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG016B);

	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG016C);
	reg &= USBDP_CMN_REG016C_TX_LFPS_TP_GEN_T_LFPS1_CLR;
	reg |= USBDP_CMN_REG016C_TX_LFPS_TP_GEN_T_LFPS1_SET(0x1E);
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG016C);

	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG016F);
	reg &= USBDP_CMN_REG016F_BIAS_ICAL_AUTO_COMP_EN_DELAY_CLR;
	reg |= USBDP_CMN_REG016F_BIAS_ICAL_AUTO_COMP_EN_DELAY_SET(0x13);
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG016F);

	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG0170);
	reg &= USBDP_CMN_REG0170_BIAS_ICAL_AUTO_CODE_DELAY_CLR;
	reg |= USBDP_CMN_REG0170_BIAS_ICAL_AUTO_CODE_DELAY_SET(0x13);
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG0170);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG031A);
	reg &= USBDP_TRSV_REG031A_LN0_TG_RXD_COMP_DELAY_TIME__15_8_CLR;
	reg |= USBDP_TRSV_REG031A_LN0_TG_RXD_COMP_DELAY_TIME__15_8_SET(0x00);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG031A);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG031B);
	reg &= USBDP_TRSV_REG031B_LN0_TG_RXD_COMP_DELAY_TIME__7_0_CLR;
	reg |= USBDP_TRSV_REG031B_LN0_TG_RXD_COMP_DELAY_TIME__7_0_SET(0x03);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG031B);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0324);
	reg &= USBDP_TRSV_REG0324_LN0_RX_CDR_DATA_MODE_EXIT_EXTEND_SP_CLR;
	reg |= USBDP_TRSV_REG0324_LN0_RX_CDR_DATA_MODE_EXIT_EXTEND_SP_SET(0x03);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0324);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG032A);
	reg &= USBDP_TRSV_REG032A_LN0_RX_VALID_RSTN_DELAY_RISE_SP__15_8_CLR;
	reg |= USBDP_TRSV_REG032A_LN0_RX_VALID_RSTN_DELAY_RISE_SP__15_8_SET(0x03);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG032A);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG032B);
	reg &= USBDP_TRSV_REG032B_LN0_RX_VALID_RSTN_DELAY_RISE_SP__7_0_CLR;
	reg |= USBDP_TRSV_REG032B_LN0_RX_VALID_RSTN_DELAY_RISE_SP__7_0_SET(0x00);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG032B);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG032C);
	reg &= USBDP_TRSV_REG032C_LN0_RX_VALID_RSTN_DELAY_FALL_SP__15_8_CLR;
	reg |= USBDP_TRSV_REG032C_LN0_RX_VALID_RSTN_DELAY_FALL_SP__15_8_SET(0x00);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG032C);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG032D);
	reg &= USBDP_TRSV_REG032D_LN0_RX_VALID_RSTN_DELAY_FALL_SP__7_0_CLR;
	reg |= USBDP_TRSV_REG032D_LN0_RX_VALID_RSTN_DELAY_FALL_SP__7_0_SET(0x00);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG032D);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG032E);
	reg &= USBDP_TRSV_REG032E_LN0_RX_VALID_RSTN_DELAY_RISE_SSP__15_8_CLR;
	reg |= USBDP_TRSV_REG032E_LN0_RX_VALID_RSTN_DELAY_RISE_SSP__15_8_SET(0x03);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG032E);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG032F);
	reg &= USBDP_TRSV_REG032F_LN0_RX_VALID_RSTN_DELAY_RISE_SSP__7_0_CLR;
	reg |= USBDP_TRSV_REG032F_LN0_RX_VALID_RSTN_DELAY_RISE_SSP__7_0_SET(0x00);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG032F);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0330);
	reg &= USBDP_TRSV_REG0330_LN0_RX_VALID_RSTN_DELAY_FALL_SSP__15_8_CLR;
	reg |= USBDP_TRSV_REG0330_LN0_RX_VALID_RSTN_DELAY_FALL_SSP__15_8_SET(0x00);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0330);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0331);
	reg &= USBDP_TRSV_REG0331_LN0_RX_VALID_RSTN_DELAY_FALL_SSP__7_0_CLR;
	reg |= USBDP_TRSV_REG0331_LN0_RX_VALID_RSTN_DELAY_FALL_SSP__7_0_SET(0x00);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0331);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG071A);
	reg &= USBDP_TRSV_REG071A_LN2_TG_RXD_COMP_DELAY_TIME__15_8_CLR;
	reg |= USBDP_TRSV_REG071A_LN2_TG_RXD_COMP_DELAY_TIME__15_8_SET(0x00);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG071A);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG071B);
	reg &= USBDP_TRSV_REG071B_LN2_TG_RXD_COMP_DELAY_TIME__7_0_CLR;
	reg |= USBDP_TRSV_REG071B_LN2_TG_RXD_COMP_DELAY_TIME__7_0_SET(0x03);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG071B);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0724);
	reg &= USBDP_TRSV_REG0724_LN2_RX_CDR_DATA_MODE_EXIT_EXTEND_SP_CLR;
	reg |= USBDP_TRSV_REG0724_LN2_RX_CDR_DATA_MODE_EXIT_EXTEND_SP_SET(0x03);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0724);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG072A);
	reg &= USBDP_TRSV_REG072A_LN2_RX_VALID_RSTN_DELAY_RISE_SP__15_8_CLR;
	reg |= USBDP_TRSV_REG072A_LN2_RX_VALID_RSTN_DELAY_RISE_SP__15_8_SET(0x03);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG072A);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG072B);
	reg &= USBDP_TRSV_REG072B_LN2_RX_VALID_RSTN_DELAY_RISE_SP__7_0_CLR;
	reg |= USBDP_TRSV_REG072B_LN2_RX_VALID_RSTN_DELAY_RISE_SP__7_0_SET(0x00);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG072B);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG072C);
	reg &= USBDP_TRSV_REG072C_LN2_RX_VALID_RSTN_DELAY_FALL_SP__15_8_CLR;
	reg |= USBDP_TRSV_REG072C_LN2_RX_VALID_RSTN_DELAY_FALL_SP__15_8_SET(0x00);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG072C);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG072D);
	reg &= USBDP_TRSV_REG072D_LN2_RX_VALID_RSTN_DELAY_FALL_SP__7_0_CLR;
	reg |= USBDP_TRSV_REG072D_LN2_RX_VALID_RSTN_DELAY_FALL_SP__7_0_SET(0x00);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG072D);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG072E);
	reg &= USBDP_TRSV_REG072E_LN2_RX_VALID_RSTN_DELAY_RISE_SSP__15_8_CLR;
	reg |= USBDP_TRSV_REG072E_LN2_RX_VALID_RSTN_DELAY_RISE_SSP__15_8_SET(0x03);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG072E);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG072F);
	reg &= USBDP_TRSV_REG072F_LN2_RX_VALID_RSTN_DELAY_RISE_SSP__7_0_CLR;
	reg |= USBDP_TRSV_REG072F_LN2_RX_VALID_RSTN_DELAY_RISE_SSP__7_0_SET(0x00);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG072F);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0730);
	reg &= USBDP_TRSV_REG0730_LN2_RX_VALID_RSTN_DELAY_FALL_SSP__15_8_CLR;
	reg |= USBDP_TRSV_REG0730_LN2_RX_VALID_RSTN_DELAY_FALL_SSP__15_8_SET(0x00);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0730);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0731);
	reg &= USBDP_TRSV_REG0731_LN2_RX_VALID_RSTN_DELAY_FALL_SSP__7_0_CLR;
	reg |= USBDP_TRSV_REG0731_LN2_RX_VALID_RSTN_DELAY_FALL_SSP__7_0_SET(0x00);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0731);
}

static inline void phy_exynos_usbdp_g2_v4_pma_default_sfr_update_19_2Mhz_Gen1(struct exynos_usbphy_info *info)
{

	void __iomem *regs_base = info->pma_base;
	u32 reg;

	/* --- Common Block --- */

	/* ana_lcpll_pms_mdiv, mdiv_afc*/
	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG0027);
	reg &= USBDP_CMN_REG0027_ANA_LCPLL_PMS_MDIV_CLR;
	reg |= USBDP_CMN_REG0027_ANA_LCPLL_PMS_MDIV_SET(0x82);
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG0027);

	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG0028);
	reg &= USBDP_CMN_REG0028_ANA_LCPLL_PMS_MDIV_AFC_CLR;

	reg |= USBDP_CMN_REG0028_ANA_LCPLL_PMS_MDIV_AFC_SET(0x82);
	// Fixed: 20210514
	// reg |= USBDP_CMN_REG0028_ANA_LCPLL_PMS_MDIV_AFC_SET(0xD4);
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG0028);

	/*lcpll_pms_sdiv_sp, ssp*/
	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG002A);
	reg &= USBDP_CMN_REG002A_LCPLL_PMS_SDIV_SP_CLR;
	reg |= USBDP_CMN_REG002A_LCPLL_PMS_SDIV_SP_SET(0x3);
	reg &= USBDP_CMN_REG002A_LCPLL_PMS_SDIV_SSP_CLR;
	reg |= USBDP_CMN_REG002A_LCPLL_PMS_SDIV_SSP_SET(0x1);
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG002A);

	/* IQ Div for LC VCO and RO bypass */
	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG002D);
	reg &= USBDP_CMN_REG002D_ANA_LCPLL_IQDIV_BYPASS_CLR;
	reg |= USBDP_CMN_REG002D_ANA_LCPLL_IQDIV_BYPASS_SET(0x1);
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG002D);

	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG0074);
	reg &= USBDP_CMN_REG0074_ANA_ROPLL_IQDIV_BYPASS_CLR;
	reg |= USBDP_CMN_REG0074_ANA_ROPLL_IQDIV_BYPASS_SET(0x0);
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG0074);

	/* Numerator of SDM with i_lcpll_sdm_numerator_sign (-255~255) */
	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG0034);
	reg &= USBDP_CMN_REG0034_ANA_LCPLL_SDM_NUMERATOR_CLR;
	reg |= USBDP_CMN_REG0034_ANA_LCPLL_SDM_NUMERATOR_SET(0x10);
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG0034);

	/* Denominator of SDM (Max. 255) */
	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG0032);
	reg &= USBDP_CMN_REG0032_ANA_LCPLL_SDM_DENOMINATOR_CLR;
	reg |= USBDP_CMN_REG0032_ANA_LCPLL_SDM_DENOMINATOR_SET(0x27);
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG0032);

	/* LC PLL input clk phase, SDC divide-ratio  */
	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG0035);
	reg &= USBDP_CMN_REG0035_ANA_LCPLL_SDM_PH_NUM_SEL_CLR;
	reg |= USBDP_CMN_REG0035_ANA_LCPLL_SDM_PH_NUM_SEL_SET(0x0);
	reg &= USBDP_CMN_REG0035_ANA_LCPLL_SDC_N_CLR;
	reg |= USBDP_CMN_REG0035_ANA_LCPLL_SDC_N_SET(0x5);
	reg &= USBDP_CMN_REG0035_ANA_LCPLL_SDC_N2_CLR;
	reg |= USBDP_CMN_REG0035_ANA_LCPLL_SDC_N2_SET(0x1);
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG0035);

	/* RO PLL PI input clock phase number 0: 8-phase, 1: 4-phase */
	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG0086);
	reg &= USBDP_CMN_REG0086_ANA_ROPLL_SDM_PH_NUM_SEL_CLR;
	reg |= USBDP_CMN_REG0086_ANA_ROPLL_SDM_PH_NUM_SEL_SET(0x1);
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG0086);

	/* RO PLL SDC divide-ratio selection */
	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG0089);
	reg &= USBDP_CMN_REG0089_ANA_ROPLL_SDC_N2_CLR;
	reg |= USBDP_CMN_REG0089_ANA_ROPLL_SDC_N2_SET(0x1);
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG0089);

	/* RO PLL numerator of SDC (Max 65) */
	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG0036);
	reg &= USBDP_CMN_REG0036_ANA_LCPLL_SDC_NUMERATOR_CLR;
	reg |= USBDP_CMN_REG0036_ANA_LCPLL_SDC_NUMERATOR_SET(0x0);
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG0036);

	/* LC PLL denominator of SDC (Max 65) */
	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG0037);
	reg &= USBDP_CMN_REG0037_ANA_LCPLL_SDC_DENOMINATOR_CLR;
	reg |= USBDP_CMN_REG0037_ANA_LCPLL_SDC_DENOMINATOR_SET(0x0);
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG0037);

	/* LC PLL SSC modulation deviation control */
	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG0039);
	reg &= USBDP_CMN_REG0039_ANA_LCPLL_SSC_FM_DEVIATION_CLR;
	reg |= USBDP_CMN_REG0039_ANA_LCPLL_SSC_FM_DEVIATION_SET(0x32);
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG0039);

	/* LC PLL SSC modulation frequency control */
	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG003A);
	reg &= USBDP_CMN_REG003A_ANA_LCPLL_SSC_FM_FREQ_CLR;
	reg |= USBDP_CMN_REG003A_ANA_LCPLL_SSC_FM_FREQ_SET(0x5);
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG003A);

	/* --- TRSV Block --- */
	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG02B6);
	reg &= USBDP_TRSV_REG02B6_LN0_RX_CDR_PMS_M_SP__8_CLR;
	reg |= USBDP_TRSV_REG02B6_LN0_RX_CDR_PMS_M_SP__8_SET(0x0);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG02B6);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG02B7);
	reg &= USBDP_TRSV_REG02B7_LN0_RX_CDR_PMS_M_SP__7_0_CLR;
	// modified
	reg |= USBDP_TRSV_REG02B7_LN0_RX_CDR_PMS_M_SP__7_0_SET(0x82);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG02B7);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG02B8);
	reg &= USBDP_TRSV_REG02B8_LN0_RX_CDR_PMS_M_SSP__8_CLR;
	reg |= USBDP_TRSV_REG02B8_LN0_RX_CDR_PMS_M_SSP__8_SET(0x1);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG02B8);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG02B9);
	reg &= USBDP_TRSV_REG02B9_LN0_RX_CDR_PMS_M_SSP__7_0_CLR;
	reg |= USBDP_TRSV_REG02B9_LN0_RX_CDR_PMS_M_SSP__7_0_SET(0x4);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG02B9);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0386);
	reg &= USBDP_TRSV_REG0386_LN0_RX_CDR_AFC_PMS_M_SP__8_CLR;
	reg |= USBDP_TRSV_REG0386_LN0_RX_CDR_AFC_PMS_M_SP__8_SET(0x0);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0386);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0387);
	reg &= USBDP_TRSV_REG0387_LN0_RX_CDR_AFC_PMS_M_SP__7_0_CLR;
	reg |= USBDP_TRSV_REG0387_LN0_RX_CDR_AFC_PMS_M_SP__7_0_SET(0x82);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0387);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0388);
	reg &= USBDP_TRSV_REG0388_LN0_RX_CDR_AFC_PMS_M_SSP__8_CLR;
	reg |= USBDP_TRSV_REG0388_LN0_RX_CDR_AFC_PMS_M_SSP__8_SET(0x1);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0388);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0389);
	reg &= USBDP_TRSV_REG0389_LN0_RX_CDR_AFC_PMS_M_SSP__7_0_CLR;
	reg |= USBDP_TRSV_REG0389_LN0_RX_CDR_AFC_PMS_M_SSP__7_0_SET(0x4);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0389);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG06B6);
	reg &= USBDP_TRSV_REG06B6_LN2_RX_CDR_PMS_M_SP__8_CLR;
	reg |= USBDP_TRSV_REG06B6_LN2_RX_CDR_PMS_M_SP__8_SET(0x0);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG06B6);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG06B7);
	reg &= USBDP_TRSV_REG06B7_LN2_RX_CDR_PMS_M_SP__7_0_CLR;
	reg |= USBDP_TRSV_REG06B7_LN2_RX_CDR_PMS_M_SP__7_0_SET(0x82);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG06B7);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG06B8);
	reg &= USBDP_TRSV_REG06B8_LN2_RX_CDR_PMS_M_SSP__8_CLR;
	reg |= USBDP_TRSV_REG06B8_LN2_RX_CDR_PMS_M_SSP__8_SET(0x1);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG06B8);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG06B9);
	reg &= USBDP_TRSV_REG06B9_LN2_RX_CDR_PMS_M_SSP__7_0_CLR;
	reg |= USBDP_TRSV_REG06B9_LN2_RX_CDR_PMS_M_SSP__7_0_SET(0x4);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG06B9);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0786);
	reg &= USBDP_TRSV_REG0786_LN2_RX_CDR_AFC_PMS_M_SP__8_CLR;
	reg |= USBDP_TRSV_REG0786_LN2_RX_CDR_AFC_PMS_M_SP__8_SET(0x0);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0786);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0787);
	reg &= USBDP_TRSV_REG0787_LN2_RX_CDR_AFC_PMS_M_SP__7_0_CLR;
	reg |= USBDP_TRSV_REG0787_LN2_RX_CDR_AFC_PMS_M_SP__7_0_SET(0x82);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0787);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0788);
	reg &= USBDP_TRSV_REG0788_LN2_RX_CDR_AFC_PMS_M_SSP__8_CLR;
	reg |= USBDP_TRSV_REG0788_LN2_RX_CDR_AFC_PMS_M_SSP__8_SET(0x1);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0788);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0789);
	reg &= USBDP_TRSV_REG0789_LN2_RX_CDR_AFC_PMS_M_SSP__7_0_CLR;
	reg |= USBDP_TRSV_REG0789_LN2_RX_CDR_AFC_PMS_M_SSP__7_0_SET(0x4);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0789);
}

static void phy_exynos_usbdp_g2_v4_set_pcs(struct exynos_usbphy_info *info)
{
	void __iomem *regs_base = info->pcs_base;
	u32 reg;

	/* abnormal comman pattern mask */
	reg = readl(regs_base + EXYNOS_USBDP_PCS_RX_BACK_END_MODE_VEC);
	reg &= USBDP_PCS_RX_BACK_END_MODE_VEC_DISABLE_DATA_MASK_CLR;
	writel(reg, regs_base + EXYNOS_USBDP_PCS_RX_BACK_END_MODE_VEC);

	/* De-serializer enabled when U2 */
	reg = readl(regs_base + EXYNOS_USBDP_PCS_PM_OUT_VEC_2);
	reg &= USBDP_PCS_PM_OUT_VEC_2_B4_DYNAMIC_CLR;
	reg |= USBDP_PCS_PM_OUT_VEC_2_B4_SEL_OUT_SET(1);
	writel(reg, regs_base + EXYNOS_USBDP_PCS_PM_OUT_VEC_2);

	/* TX Keeper Disable, Squelch off when U3 */
	reg = readl(regs_base + EXYNOS_USBDP_PCS_PM_OUT_VEC_3);
	reg &= USBDP_PCS_PM_OUT_VEC_3_B7_DYNAMIC_CLR;
	reg |= USBDP_PCS_PM_OUT_VEC_3_B7_SEL_OUT_SET(1);
	reg &= USBDP_PCS_PM_OUT_VEC_3_B2_SEL_OUT_CLR;
	/* Squelch on when U3 */
	reg |= USBDP_PCS_PM_OUT_VEC_3_B2_SEL_OUT_SET(1);
	writel(reg, regs_base + EXYNOS_USBDP_PCS_PM_OUT_VEC_3);

	/* 20180822 PCS SFR setting : Noh M.W */
	writel(0x05700000, regs_base + EXYNOS_USBDP_PCS_PM_NS_VEC_PS1_N1);
	writel(0x01700202, regs_base + EXYNOS_USBDP_PCS_PM_NS_VEC_PS2_N0);
	writel(0x01700707, regs_base + EXYNOS_USBDP_PCS_PM_NS_VEC_PS3_N0);
	writel(0x0070, regs_base + EXYNOS_USBDP_PCS_PM_TIMEOUT_0);

	/* Block Aligner Type B */
	reg = readl(regs_base + EXYNOS_USBDP_PCS_RX_RX_CONTROL);
	reg &= USBDP_PCS_RX_RX_CONTROL_EN_BLOCK_ALIGNER_TYPE_B_CLR;
	reg |= USBDP_PCS_RX_RX_CONTROL_EN_BLOCK_ALIGNER_TYPE_B_SET(1);
	writel(reg, regs_base + EXYNOS_USBDP_PCS_RX_RX_CONTROL);

	/* Block align at TS1/TS2 for Gen2 stability (Gen2 only) */
	reg = readl(regs_base + EXYNOS_USBDP_PCS_RX_RX_CONTROL_DEBUG);
	reg &= USBDP_PCS_RX_RX_CONTROL_DEBUG_EN_TS_CHECK_CLR;
	reg |= USBDP_PCS_RX_RX_CONTROL_DEBUG_EN_TS_CHECK_SET(1);
	/*
	 * increse pcs ts1 adding packet-cnt 1 --> 4
	 * lnx_rx_valid_rstn_delay_rise_sp/ssp :
	 * 19.6us(0x200) -> 15.3us(0x4)
	 */
	reg &= USBDP_PCS_RX_RX_CONTROL_DEBUG_NUM_COM_FOUND_CLR;
	reg |= USBDP_PCS_RX_RX_CONTROL_DEBUG_NUM_COM_FOUND_SET(4);
	writel(reg, regs_base + EXYNOS_USBDP_PCS_RX_RX_CONTROL_DEBUG);

	/* Gen1 Tx DRIVER pre-shoot, de-emphasis, level ctrl
	 * [15:12] de-emphasis
	 * [9:6] level - 0xb (max)
	 * [3:0] pre-shoot - Gen1 does not support pre-shoot
	 */
	reg = readl(regs_base + EXYNOS_USBDP_PCS_LEQ_HS_TX_COEF_MAP_0);
	reg &= USBDP_PCS_LEQ_HS_TX_COEF_MAP_0_LEVEL_CLR;
	reg |= USBDP_PCS_LEQ_HS_TX_COEF_MAP_0_LEVEL_SET((8 << 12 |
			0xB << 6 | 0x0));
	writel(reg, regs_base + EXYNOS_USBDP_PCS_LEQ_HS_TX_COEF_MAP_0);

	/* Gen2 Tx DRIVER level ctrl */
	reg = readl(regs_base + EXYNOS_USBDP_PCS_LEQ_LOCAL_COEF);
	reg &= USBDP_PCS_LEQ_LOCAL_COEF_PMA_CENTER_COEF_CLR;
	reg |= USBDP_PCS_LEQ_LOCAL_COEF_PMA_CENTER_COEF_SET(0xB);
	writel(reg, regs_base + EXYNOS_USBDP_PCS_LEQ_LOCAL_COEF);

	/* Gen2 U1 exit LFPS duration : 900ns ~ 1.2us */
	writel(0x1000, regs_base + EXYNOS_USBDP_PCS_PM_TIMEOUT_3);

	/* set skp_remove_th 0x2 -> 0x7 for avoiding retry problem. */
	reg = readl(regs_base + EXYNOS_USBDP_PCS_RX_EBUF_PARAM);
	reg &= USBDP_PCS_RX_EBUF_PARAM_SKP_REMOVE_TH_EMPTY_MODE_CLR;
	reg |= USBDP_PCS_RX_EBUF_PARAM_SKP_REMOVE_TH_EMPTY_MODE_SET(0x7);
	writel(reg, regs_base + EXYNOS_USBDP_PCS_RX_EBUF_PARAM);
}

static inline void
phy_exynos_usbdp_g2_v4_pma_lane_mux_sel(struct exynos_usbphy_info
		*info)
{
	void __iomem *regs_base = info->pma_base;
	u32 reg;

	/* Lane configuration */
	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG00B8);
	reg &= USBDP_CMN_REG00B8_LANE_MUX_SEL_DP_CLR;

	if (info->used_phy_port == 0)
		reg |= USBDP_CMN_REG00B8_LANE_MUX_SEL_DP_SET(0xc);
	else
		reg |= USBDP_CMN_REG00B8_LANE_MUX_SEL_DP_SET(0x3);
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG00B8);

	/*
	 * ln1_tx_rxd_en = 0
	 * ln3_tx_rxd_en = 0
	 */
	if (info->used_phy_port == 0) {
		/* ln3_tx_rxd_en = 0 */
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0413);
		reg &= USBDP_TRSV_REG0413_OVRD_LN1_TX_RXD_COMP_EN_CLR;
		reg &= USBDP_TRSV_REG0413_OVRD_LN1_TX_RXD_EN_CLR;
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0413);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0813);
		reg &= USBDP_TRSV_REG0813_OVRD_LN3_TX_RXD_COMP_EN_CLR;
		reg |= USBDP_TRSV_REG0813_OVRD_LN3_TX_RXD_COMP_EN_SET(1);
		reg &= USBDP_TRSV_REG0813_OVRD_LN3_TX_RXD_EN_CLR;
		reg |= USBDP_TRSV_REG0813_OVRD_LN3_TX_RXD_EN_SET(1);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0813);
	} else {
		/* ln1_tx_rxd_en = 0 */
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0413);
		reg &= USBDP_TRSV_REG0413_OVRD_LN1_TX_RXD_COMP_EN_CLR;
		reg |= USBDP_TRSV_REG0413_OVRD_LN1_TX_RXD_COMP_EN_SET(1);
		reg &= USBDP_TRSV_REG0413_OVRD_LN1_TX_RXD_EN_CLR;
		reg |= USBDP_TRSV_REG0413_OVRD_LN1_TX_RXD_EN_SET(1);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0413);
	}
}

void phy_exynos_usbdp_g2_v4_tune(struct exynos_usbphy_info *info)
{
	u32 cnt = 0;

	if (!info)
		return;

	if (!info->tune_param)
		return;

	for (; info->tune_param[cnt].value != EXYNOS_USB_TUNE_LAST; cnt++) {
		char *para_name;
		int val;

		val = info->tune_param[cnt].value;
		if (val == -1)
			continue;

		para_name = info->tune_param[cnt].name;
		if (!para_name)
			break;

		/* TODO */
		phy_exynos_usbdp_g2_v2_tune_each(info, para_name, val);
	}
}

static void
phy_exynos_usbdp_g2_v4_tune_each_late(struct exynos_usbphy_info
		*info, char *name, int val)
{
	void __iomem *link_base = info->link_base;
	u32 reg;

	/* Gen2 Tx DRIVER pre-shoot, de-emphasis ctrl
	 * [17:12] Deemphasis
	 * [11:6] Level
	 * [5:0] Preshoot
	 */

	if (!strcmp(name, "sstx_deemp_ssp")) {
		reg = readl(link_base + USB31DRD_LINK_LCSR_TX_DEEMPH);
		reg &= ~(0x3F000);
		reg |= val << 12;
		writel(reg, link_base + USB31DRD_LINK_LCSR_TX_DEEMPH);
	}

	else if (!strcmp(name, "sstx_pre_shoot_ssp")) {
		reg = readl(link_base + USB31DRD_LINK_LCSR_TX_DEEMPH);
		reg &= ~(0x0003F);
		reg |= val;
		writel(reg, link_base + USB31DRD_LINK_LCSR_TX_DEEMPH);
	}
}

static void
phy_exynos_usbdp_g2_v4_tune_late(struct exynos_usbphy_info
		*info)
{
	u32 cnt = 0;
	void __iomem *link_base;
	void __iomem *pcs_base;
	u32 reg;

	if (!info)
		return;

	link_base = info->link_base;
	pcs_base = info->pcs_base;

	/* Gen2 Tx DRIVER pre-shoot, de-emphasis ctrl
	 * [17:12] Deemphasis
	 * [11:6] Level (not valid)
	 * [5:0] Preshoot
	 */
	/* normal operation, compliance pattern 15 */
	reg = readl(link_base + USB31DRD_LINK_LCSR_TX_DEEMPH);
	reg &= ~(0x3FFFF);
	reg |= 4 << 12 | 4;
	writel(reg, link_base + USB31DRD_LINK_LCSR_TX_DEEMPH);

	/* compliance pattern 13 */
	reg = readl(link_base + USB31DRD_LINK_LCSR_TX_DEEMPH_1);
	reg &= ~(0x3FFFF);
	reg |= 4;
	writel(reg, link_base + USB31DRD_LINK_LCSR_TX_DEEMPH_1);

	/* compliance pattern 14 */
	reg = readl(link_base + USB31DRD_LINK_LCSR_TX_DEEMPH_2);
	reg &= ~(0x3FFFF);
	reg |= 4 << 12;
	writel(reg, link_base + USB31DRD_LINK_LCSR_TX_DEEMPH_2);

	if (!info->tune_param)
		return;

	for (; info->tune_param[cnt].value != EXYNOS_USB_TUNE_LAST; cnt++) {
		char *para_name;
		int val;

		val = info->tune_param[cnt].value;
		if (val == -1)
			continue;
		para_name = info->tune_param[cnt].name;
		if (!para_name)
			break;
		/* TODO */
		phy_exynos_usbdp_g2_v4_tune_each_late(info, para_name, val);
	}

	/* Squelch off when U3 */
	reg = readl(pcs_base + EXYNOS_USBDP_PCS_PM_OUT_VEC_3);
	reg &= USBDP_PCS_PM_OUT_VEC_3_B2_SEL_OUT_CLR;   // Squelch off when U3
	writel(reg, pcs_base + EXYNOS_USBDP_PCS_PM_OUT_VEC_3);
}

static int
phy_exynos_usbdp_g2_v4_pma_check_offset_cal_code(struct exynos_usbphy_info
		*info)
{
	void __iomem *regs_base = info->pma_base;
	u32 reg;
	u32 code = 0;

	if (info->used_phy_port == 0) {
		/*
		 * 0x0F28  ln0_mon_rx_oc_dfe_adder_even
		 * 0x0F2C  ln0_mon_rx_oc_dfe_adder_odd
		 * 0x0F30  ln0_mon_rx_oc_dfe_dac_adder_even
		 * ln0_mon_rx_oc_dfe_dac_adder_odd
		 * 0x0F34  ln0_mon_rx_oc_dfe_sa_edge_even
		 * 0x0F38  ln0_mon_rx_oc_dfe_sa_edge_odd
		 * 0x0F3C  ln0_mon_rx_oc_dfe_dac_edge_odd
		 * ln0_mon_rx_oc_dfe_dac_edge_even
		 * 0x0F40  ln0_mon_rx_oc_dfe_sa_err_even
		 * 0x0F44  ln0_mon_rx_oc_dfe_sa_err_odd
		 * 0x0F48  ln0_mon_rx_oc_dfe_dac_err_even
		 * ln0_mon_rx_oc_dfe_dac_err_odd
		 * 0x0F4C  ln0_mon_rx_oc_ctle
		 * 0x072C  ln0_mon_rx_oc_init_vga_code
		 * 0x0730  ln0_mon_rx_oc_init_dac_vga_code
		 */
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG03CA);
		code =
		USBDP_TRSV_REG03CA_LN0_MON_RX_OC_DFE_ADDER_EVEN_GET(reg);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG03CB);
		code =
		USBDP_TRSV_REG03CB_LN0_MON_RX_OC_DFE_ADDER_ODD_GET(reg);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG03CC);
		code =
		USBDP_TRSV_REG03CC_LN0_MON_RX_OC_DFE_DAC_ADDER_EVEN_GET(reg);
		code =
		USBDP_TRSV_REG03CC_LN0_MON_RX_OC_DFE_DAC_ADDER_ODD_GET(reg);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG03CD);
		code =
		USBDP_TRSV_REG03CD_LN0_MON_RX_OC_DFE_SA_EDGE_EVEN_GET(reg);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG03CE);
		code =
		USBDP_TRSV_REG03CE_LN0_MON_RX_OC_DFE_SA_EDGE_ODD_GET(reg);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG03CF);
		code =
		USBDP_TRSV_REG03CF_LN0_MON_RX_OC_DFE_DAC_EDGE_ODD_GET(reg);
		code =
		USBDP_TRSV_REG03CF_LN0_MON_RX_OC_DFE_DAC_EDGE_EVEN_GET(reg);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG03D0);
		code =
		USBDP_TRSV_REG03D0_LN0_MON_RX_OC_DFE_SA_ERR_EVEN_GET(reg);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG03D1);
		code =
		USBDP_TRSV_REG03D1_LN0_MON_RX_OC_DFE_SA_ERR_ODD_GET(reg);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG03D2);
		code =
		USBDP_TRSV_REG03D2_LN0_MON_RX_OC_DFE_DAC_ERR_EVEN_GET(reg);
		code =
		USBDP_TRSV_REG03D2_LN0_MON_RX_OC_DFE_DAC_ERR_ODD_GET(reg);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG03D3);
		code =
		USBDP_TRSV_REG03D3_LN0_MON_RX_OC_CTLE_GET(reg);

		/* 0x0F5C  ln0_mon_rx_oc_fail__7_0 */
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG03D7);
		code =
		USBDP_TRSV_REG03D7_LN0_MON_RX_OC_FAIL__7_0_GET(reg);

		reg = readl(regs_base + EXYNOS_USBDP_CMN_REG01CB);
		code =
		USBDP_CMN_REG01CB_LN0_MON_RX_OC_INIT_VGA_CODE_GET(reg);

	} else {
		/*
		 * 0x0748  ln2_mon_rx_oc_init_vga_code
		 * 0x074C  ln2_mon_rx_oc_init_dac_vga_code
		 * 0x1F28  ln2_mon_rx_oc_dfe_adder_even
		 * 0x1F2C  ln2_mon_rx_oc_dfe_adder_odd
		 * 0x1F30  ln2_mon_rx_oc_dfe_dac_adder_even
		 * ln2_mon_rx_oc_dfe_dac_adder_odd
		 * 0x1F34  ln2_mon_rx_oc_dfe_sa_edge_even
		 * 0x1F38  ln2_mon_rx_oc_dfe_sa_edge_odd
		 * 0x1F3C  ln2_mon_rx_oc_dfe_dac_edge_odd
		 * ln2_mon_rx_oc_dfe_dac_edge_even
		 * 0x1F40  ln2_mon_rx_oc_dfe_sa_err_even
		 * 0x1F44  ln2_mon_rx_oc_dfe_sa_err_odd
		 * 0x1F48  ln2_mon_rx_oc_dfe_dac_err_even
		 * ln2_mon_rx_oc_dfe_dac_err_odd
		 * 0x1F4C  ln2_mon_rx_oc_ctle
		 */
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG07CA);
		code =
		USBDP_TRSV_REG07CA_LN2_MON_RX_OC_DFE_ADDER_EVEN_GET(reg);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG07CB);
		code =
		USBDP_TRSV_REG07CB_LN2_MON_RX_OC_DFE_ADDER_ODD_GET(reg);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG07CC);
		code =
		USBDP_TRSV_REG07CC_LN2_MON_RX_OC_DFE_DAC_ADDER_EVEN_GET(reg);
		code =
		USBDP_TRSV_REG07CC_LN2_MON_RX_OC_DFE_DAC_ADDER_ODD_GET(reg);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG07CD);
		code =
		USBDP_TRSV_REG07CD_LN2_MON_RX_OC_DFE_SA_EDGE_EVEN_GET(reg);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG07CE);
		code =
		USBDP_TRSV_REG07CE_LN2_MON_RX_OC_DFE_SA_EDGE_ODD_GET(reg);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG07CF);
		code =
		USBDP_TRSV_REG07CF_LN2_MON_RX_OC_DFE_DAC_EDGE_ODD_GET(reg);
		code =
		USBDP_TRSV_REG07CF_LN2_MON_RX_OC_DFE_DAC_EDGE_EVEN_GET(reg);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG07D0);
		code =
		USBDP_TRSV_REG07D0_LN2_MON_RX_OC_DFE_SA_ERR_EVEN_GET(reg);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG07D1);
		code =
		USBDP_TRSV_REG07D1_LN2_MON_RX_OC_DFE_SA_ERR_ODD_GET(reg);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG07D2);
		code =
		USBDP_TRSV_REG07D2_LN2_MON_RX_OC_DFE_DAC_ERR_EVEN_GET(reg);
		code =
		USBDP_TRSV_REG07D2_LN2_MON_RX_OC_DFE_DAC_ERR_ODD_GET(reg);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG07D3);
		code =
		USBDP_TRSV_REG07D3_LN2_MON_RX_OC_CTLE_GET(reg);

		/* 0x1F5C  ln2_mon_rx_oc_fail__7_0 */
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG07D7);
		code =
		USBDP_TRSV_REG07D7_LN2_MON_RX_OC_FAIL__7_0_GET(reg);

		reg = readl(regs_base + EXYNOS_USBDP_CMN_REG01D2);
		code =
		USBDP_CMN_REG01D2_LN2_MON_RX_OC_INIT_VGA_CODE_GET(reg);
	}

	if (code)
		return -1;
	else
		return 0;
}

static int
phy_exynos_usbdp_g2_v4_pma_check_pll_lock(struct exynos_usbphy_info
		*info)
{
	void __iomem *regs_base = info->pma_base;
	u32 reg;
	u32 cnt;

	for (cnt = 1000; cnt != 0; cnt--) {
		reg = readl(regs_base + EXYNOS_USBDP_CMN_REG01C0);
		if ((reg & (USBDP_CMN_REG01C0_ANA_LCPLL_LOCK_DONE_MSK
			| USBDP_CMN_REG01C0_ANA_LCPLL_AFC_DONE_MSK))) {
			break;
		}
		udelay(1);
	}

	if (!cnt)
		return -1;

	return 0;
}

static int
phy_exynos_usbdp_g2_v4_pma_check_cdr_lock(struct exynos_usbphy_info
		*info)
{
	void __iomem *regs_base = info->pma_base;
	u32 reg;
	u32 cnt;

	if (info->used_phy_port == 0) {
		for (cnt = 1000; cnt != 0; cnt--) {
			reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG03C3);
			if ((reg &
			(USBDP_TRSV_REG03C3_LN0_MON_RX_CDR_LOCK_DONE_MSK |
			USBDP_TRSV_REG03C3_LN0_MON_RX_CDR_FLD_PLL_MODE_DONE_MSK
			| USBDP_TRSV_REG03C3_LN0_MON_RX_CDR_CAL_DONE_MSK
			| USBDP_TRSV_REG03C3_LN0_MON_RX_CDR_AFC_DONE_MSK))) {
				break;
			}
			udelay(1);
		}
	} else {
		for (cnt = 1000; cnt != 0; cnt--) {
			reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG07C3);
			if ((reg &
			(USBDP_TRSV_REG07C3_LN2_MON_RX_CDR_LOCK_DONE_MSK |
			USBDP_TRSV_REG07C3_LN2_MON_RX_CDR_FLD_PLL_MODE_DONE_MSK
			| USBDP_TRSV_REG07C3_LN2_MON_RX_CDR_CAL_DONE_MSK
			| USBDP_TRSV_REG07C3_LN2_MON_RX_CDR_AFC_DONE_MSK))) {
				break;
			}
			udelay(1);
		}
	}

	if (!cnt)
		return -2;

	return 0;
}

static void phy_exynos_usbdp_g2_v4_pma_ovrd_enable(struct exynos_usbphy_info *info, u32 cmn_rate)
{
	void __iomem *regs_base = info->pma_base;
	u32 reg;

	/*
	 * pcs_pll_en = 0
	 * 0x0390 0x40
	 */
	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG00E4);
	reg &= USBDP_CMN_REG00E4_OVRD_PCS_PLL_EN_CLR;
	reg |= USBDP_CMN_REG00E4_OVRD_PCS_PLL_EN_SET(1);
	reg &= USBDP_CMN_REG00E4_PCS_PLL_EN_CLR;
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG00E4);

	/*
	 * pcs_bgr_en = 0
	 * pcs_bias_en = 0
	 * 0x0388 0xA0
	 */
	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG00E2);
	reg &= USBDP_CMN_REG00E2_OVRD_PCS_BGR_EN_CLR;
	reg |= USBDP_CMN_REG00E2_OVRD_PCS_BGR_EN_SET(1);
	reg &= USBDP_CMN_REG00E2_PCS_BGR_EN_CLR;
	reg &= USBDP_CMN_REG00E2_OVRD_PCS_BIAS_EN_CLR;
	reg |= USBDP_CMN_REG00E2_OVRD_PCS_BIAS_EN_SET(1);
	reg &= USBDP_CMN_REG00E2_PCS_BIAS_EN_CLR;
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG00E2);

	/*
	 * cmn_rate
	 * 0x0384 0x04 // Gen1
	 *        0x05 // Gen2
	 */
	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG00E1);
	reg &= USBDP_CMN_REG00E1_OVRD_PCS_RATE_CLR;
	reg |= USBDP_CMN_REG00E1_OVRD_PCS_RATE_SET(1);
	reg &= USBDP_CMN_REG00E1_PCS_RATE_CLR;
	reg |= USBDP_CMN_REG00E1_PCS_RATE_SET(cmn_rate);    /* 0: Gen1, 1: Gen2 */
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG00E1);

	/*
	 * lane_mux_sel_dp
	 * ln1_tx_rxd_en
	 * ln3_tx_rxd_en
	 * 0x02E0 0x30
	 * 0x104C 0x01
	 * 0x204C 0x01
	 */
	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG00B8);
	reg &= USBDP_CMN_REG00B8_LANE_MUX_SEL_DP_CLR;
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG00B8);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0413);
	reg &= USBDP_TRSV_REG0413_OVRD_LN1_TX_RXD_COMP_EN_CLR;
	reg &= USBDP_TRSV_REG0413_OVRD_LN1_TX_RXD_EN_CLR;
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0413);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0813);
	reg &= USBDP_TRSV_REG0813_OVRD_LN3_TX_RXD_COMP_EN_CLR;
	reg &= USBDP_TRSV_REG0813_OVRD_LN3_TX_RXD_EN_CLR;
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0813);

	/*
	 * pcs_bgr_en = 1
	 * pcs_bias_en = 1
	 * pcs_powerdown = 0
	 * pcs_des_en = 0
	 * pcs_cdr_en = 1
	 * pcs_pll_en = 1
	 * pcs_rx_ctle_en = 1
	 * pcs_rx_sqhs_en = 1
	 * pcs_rx_term_en = 1
	 * pcs_tx_drv_en =1
	 * pcs_tx_elecidle = 1
	 * pcs_tx_lfps_en = 0
	 * pcs_tx_rcv_det_en =1
	 * pcs_tx_ser_en =0
	 * 0x0388 0xFB
	 * 0x038C 0x20
	 * 0x0390 0x63
	 * 0x0394 0x03
	 * 0x0398 0xC3
	 * 0x03A4 0x03
	 * 0x03A8 0x8E
	 */
	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG00E2);
	reg &= USBDP_CMN_REG00E2_OVRD_PCS_BGR_EN_CLR;
	reg |= USBDP_CMN_REG00E2_OVRD_PCS_BGR_EN_SET(1);
	reg &= USBDP_CMN_REG00E2_PCS_BGR_EN_CLR;
	reg |= USBDP_CMN_REG00E2_PCS_BGR_EN_SET(1);
	reg &= USBDP_CMN_REG00E2_OVRD_PCS_BIAS_EN_CLR;
	reg |= USBDP_CMN_REG00E2_OVRD_PCS_BIAS_EN_SET(1);
	reg &= USBDP_CMN_REG00E2_PCS_BIAS_EN_CLR;
	reg |= USBDP_CMN_REG00E2_PCS_BIAS_EN_SET(1);
	reg &= USBDP_CMN_REG00E2_OVRD_PCS_POWERDOWN_CLR;
	reg |= USBDP_CMN_REG00E2_OVRD_PCS_POWERDOWN_SET(1);
	reg &= USBDP_CMN_REG00E2_PCS_POWERDOWN_CLR;
	reg &= USBDP_CMN_REG00E2_OVRD_PCS_CDR_EN_CLR;
	reg |= USBDP_CMN_REG00E2_OVRD_PCS_CDR_EN_SET(1);
	reg &= USBDP_CMN_REG00E2_PCS_CDR_EN_CLR;
	reg |= USBDP_CMN_REG00E2_PCS_CDR_EN_SET(1);
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG00E2);

	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG00E3);
	reg &= USBDP_CMN_REG00E3_OVRD_PCS_DES_EN_CLR;
	reg |= USBDP_CMN_REG00E3_OVRD_PCS_DES_EN_SET(1);
	reg &= USBDP_CMN_REG00E3_PCS_DES_EN_CLR;
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG00E3);

	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG00E4);
	reg &= USBDP_CMN_REG00E4_OVRD_PCS_PLL_EN_CLR;
	reg |= USBDP_CMN_REG00E4_OVRD_PCS_PLL_EN_SET(1);
	reg &= USBDP_CMN_REG00E4_PCS_PLL_EN_CLR;
	reg |= USBDP_CMN_REG00E4_PCS_PLL_EN_SET(1);
	reg &= USBDP_CMN_REG00E4_OVRD_PCS_RX_CTLE_EN_CLR;
	reg |= USBDP_CMN_REG00E4_OVRD_PCS_RX_CTLE_EN_SET(1);
	reg &= USBDP_CMN_REG00E4_PCS_RX_CTLE_EN_CLR;
	reg |= USBDP_CMN_REG00E4_PCS_RX_CTLE_EN_SET(1);
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG00E4);

	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG00E5);
	reg &= USBDP_CMN_REG00E5_OVRD_PCS_RX_SQHS_EN_CLR;
	reg |= USBDP_CMN_REG00E5_OVRD_PCS_RX_SQHS_EN_SET(1);
	reg &= USBDP_CMN_REG00E5_PCS_RX_SQHS_EN_CLR;
	reg |= USBDP_CMN_REG00E5_PCS_RX_SQHS_EN_SET(1);
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG00E5);

	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG00E6);
	reg &= USBDP_CMN_REG00E6_OVRD_PCS_RX_TERM_EN_CLR;
	reg |= USBDP_CMN_REG00E6_OVRD_PCS_RX_TERM_EN_SET(1);
	reg &= USBDP_CMN_REG00E6_PCS_RX_TERM_EN_CLR;
	reg |= USBDP_CMN_REG00E6_PCS_RX_TERM_EN_SET(1);
	reg &= USBDP_CMN_REG00E6_OVRD_PCS_TX_DRV_EN_CLR;
	reg |= USBDP_CMN_REG00E6_OVRD_PCS_TX_DRV_EN_SET(1);
	reg &= USBDP_CMN_REG00E6_PCS_TX_DRV_EN_CLR;
	reg |= USBDP_CMN_REG00E6_PCS_TX_DRV_EN_SET(1);
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG00E6);

	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG00E9);
	reg &= USBDP_CMN_REG00E9_OVRD_PCS_TX_ELECIDLE_CLR;
	reg |= USBDP_CMN_REG00E9_OVRD_PCS_TX_ELECIDLE_SET(1);
	reg &= USBDP_CMN_REG00E9_PCS_TX_ELECIDLE_CLR;
	reg |= USBDP_CMN_REG00E9_PCS_TX_ELECIDLE_SET(1);
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG00E9);

	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG00EA);
	reg &= USBDP_CMN_REG00EA_OVRD_PCS_TX_LFPS_EN_CLR;
	reg |= USBDP_CMN_REG00EA_OVRD_PCS_TX_LFPS_EN_SET(1);
	reg &= USBDP_CMN_REG00EA_PCS_TX_LFPS_EN_CLR;
	reg &= USBDP_CMN_REG00EA_OVRD_PCS_TX_RCV_DET_EN_CLR;
	reg |= USBDP_CMN_REG00EA_OVRD_PCS_TX_RCV_DET_EN_SET(1);
	reg &= USBDP_CMN_REG00EA_PCS_TX_RCV_DET_EN_CLR;
	reg |= USBDP_CMN_REG00EA_PCS_TX_RCV_DET_EN_SET(1);
	reg &= USBDP_CMN_REG00EA_OVRD_PCS_TX_SER_EN_CLR;
	reg |= USBDP_CMN_REG00EA_OVRD_PCS_TX_SER_EN_SET(1);
	reg &= USBDP_CMN_REG00EA_PCS_TX_SER_EN_CLR;
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG00EA);
}

static void phy_exynos_usbdp_g2_v4_pma_ovrd_pcs_rst_release(struct exynos_usbphy_info *info)
{
	void __iomem *regs_base = info->pma_base;
	u32 reg;

	/*
	 * dp_init/cmn_rstn = 1
	 * 0x038C 0xEF
	 */
	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG00E3);
	reg &= USBDP_CMN_REG00E3_OVRD_PCS_CMN_RSTN_CLR;
	reg |= USBDP_CMN_REG00E3_OVRD_PCS_CMN_RSTN_SET(1);
	reg &= USBDP_CMN_REG00E3_PCS_CMN_RSTN_CLR;
	reg |= USBDP_CMN_REG00E3_PCS_CMN_RSTN_SET(1);
	reg &= USBDP_CMN_REG00E3_OVRD_PCS_INIT_RSTN_CLR;
	reg |= USBDP_CMN_REG00E3_OVRD_PCS_INIT_RSTN_SET(1);
	reg &= USBDP_CMN_REG00E3_PCS_INIT_RSTN_CLR;
	reg |= USBDP_CMN_REG00E3_PCS_INIT_RSTN_SET(1);
	reg &= USBDP_CMN_REG00E3_OVRD_PCS_LANE_RSTN_CLR;
	reg |= USBDP_CMN_REG00E3_OVRD_PCS_LANE_RSTN_SET(1);
	reg &= USBDP_CMN_REG00E3_PCS_LANE_RSTN_CLR;
	reg |= USBDP_CMN_REG00E3_PCS_LANE_RSTN_SET(1);
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG00E3);

}

static void phy_exynos_usbdp_g2_v4_pma_ovrd_power_on(struct exynos_usbphy_info *info)
{
	void __iomem *regs_base = info->pma_base;
	u32 reg;

	/*
	 * pcs_des_en = 1
	 * pcs_tx_ser_en =1
	 * pcs_tx_elecidle = 0
	 * 0x038C 0xFF
	 * 0x03A4 0x02
	 * 0x03A8 0x83
	 */
	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG00E3);
	reg &= USBDP_CMN_REG00E3_OVRD_PCS_DES_EN_CLR;
	reg |= USBDP_CMN_REG00E3_OVRD_PCS_DES_EN_SET(1);
	reg &= USBDP_CMN_REG00E3_PCS_DES_EN_CLR;
	reg |= USBDP_CMN_REG00E3_PCS_DES_EN_SET(1);
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG00E3);

	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG00E9);
	reg &= USBDP_CMN_REG00E9_OVRD_PCS_TX_ELECIDLE_CLR;
	reg |= USBDP_CMN_REG00E9_OVRD_PCS_TX_ELECIDLE_SET(1);
	reg &= USBDP_CMN_REG00E9_PCS_TX_ELECIDLE_CLR;
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG00E9);

	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG00EA);
	reg &= USBDP_CMN_REG00EA_OVRD_PCS_TX_RCV_DET_EN_CLR;
	reg &= USBDP_CMN_REG00EA_PCS_TX_RCV_DET_EN_CLR;
	reg &= USBDP_CMN_REG00EA_OVRD_PCS_TX_SER_EN_CLR;
	reg |= USBDP_CMN_REG00EA_OVRD_PCS_TX_SER_EN_SET(1);
	reg &= USBDP_CMN_REG00EA_PCS_TX_SER_EN_CLR;
	reg |= USBDP_CMN_REG00EA_PCS_TX_SER_EN_SET(1);
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG00EA);
}

static int phy_exynos_usbdp_g2_v4_pma_bist_en(struct exynos_usbphy_info *info)
{
	void __iomem *regs_base = info->pma_base;
	u32 reg;
	int ret = 0;
	int temp;

	/* dp_lane_en = 0  */
	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG00B9);
	reg &= USBDP_CMN_REG00B9_DP_LANE_EN_CLR;
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG00B9);

	/* dp_bist_en = 0  */
	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG00D6);
	reg &= USBDP_CMN_REG00D6_DP_BIST_EN_CLR;
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG00D6);

	/*
	 * pcs_bgr_en, pcs_bias_en
	 * pcs_des_en, pcs_cdr_en
	 * pcs_pll_en pcs_rx_ctle_en
	 * pcs_rx_sqhs_en
	 * pcs_rx_term_en, pcs_tx_drv_en
	 * pcs_tx_elecidle
	 * pcs_tx_ser_en, pcs_tx_lfps_en
	 * ln0_ana_cdr_en
	 * ln2_ana_cdr_en
	 * 0x0388 0xFB
	 * 0x038C 0xFF
	 * 0x0390 0x60
	 * 0x0394 0x00
	 * 0x0398 0x00
	 * 0x03A4 0x03
	 * 0x03A8 0x83
	 * 0x08B0 0x00
	 * 0x18B0 0x00
	 */
	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG00E4);
	reg &= USBDP_CMN_REG00E4_OVRD_PCS_RX_CTLE_EN_CLR;
	reg &= USBDP_CMN_REG00E4_PCS_RX_CTLE_EN_CLR;
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG00E4);

	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG00E5);
	reg &= USBDP_CMN_REG00E5_OVRD_PCS_RX_SQHS_EN_CLR;
	reg &= USBDP_CMN_REG00E5_PCS_RX_SQHS_EN_CLR;
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG00E5);

	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG00E6);
	reg &= USBDP_CMN_REG00E6_OVRD_PCS_RX_TERM_EN_CLR;
	reg &= USBDP_CMN_REG00E6_PCS_RX_TERM_EN_CLR;
	reg &= USBDP_CMN_REG00E6_OVRD_PCS_TX_DRV_EN_CLR;
	reg &= USBDP_CMN_REG00E6_PCS_TX_DRV_EN_CLR;
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG00E6);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG022C);
	reg &= USBDP_TRSV_REG022C_OVRD_LN0_RX_CDR_EN_CLR;
	reg &= USBDP_TRSV_REG022C_LN0_RX_CDR_EN_CLR;
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG022C);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG062C);
	reg &= USBDP_TRSV_REG062C_OVRD_LN2_RX_CDR_EN_CLR;
	reg &= USBDP_TRSV_REG062C_LN2_RX_CDR_EN_CLR;
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG062C);

	/*
	 * ln0/2_bist_en, ln0/2_bist_tx_en
	 * ln0/2_bist_data_en
	 * 0x0C00 0xC4
	 * 0x1C00 0xC4
	 */
	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0300);
	reg &= USBDP_TRSV_REG0300_LN0_BIST_EN_CLR;
	reg |= USBDP_TRSV_REG0300_LN0_BIST_EN_SET(1);
	reg &= USBDP_TRSV_REG0300_LN0_BIST_DATA_EN_CLR;
	reg |= USBDP_TRSV_REG0300_LN0_BIST_DATA_EN_SET(1);
	reg &= USBDP_TRSV_REG0300_LN0_BIST_TX_EN_CLR;
	reg |= USBDP_TRSV_REG0300_LN0_BIST_TX_EN_SET(1);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0300);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0700);
	reg &= USBDP_TRSV_REG0700_LN2_BIST_EN_CLR;
	reg |= USBDP_TRSV_REG0700_LN2_BIST_EN_SET(1);
	reg &= USBDP_TRSV_REG0700_LN2_BIST_DATA_EN_CLR;
	reg |= USBDP_TRSV_REG0700_LN2_BIST_DATA_EN_SET(1);
	reg &= USBDP_TRSV_REG0700_LN2_BIST_TX_EN_CLR;
	reg |= USBDP_TRSV_REG0700_LN2_BIST_TX_EN_SET(1);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0700);

	/*
	 * ln0/2_retimedlb_en
	 * 0x0BF8 0x0A
	 * 0x1BF8 0x0A
	 */
	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG02FE);
	reg &= USBDP_TRSV_REG02FE_LN0_RETIMEDLB_EN_CLR;
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG02FE);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG06FE);
	reg &= USBDP_TRSV_REG06FE_LN2_RETIMEDLB_EN_CLR;
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG06FE);

	/*
	 * ln#_ana_tx_drv_accdrv_en
	 * ln#_ana_tx_slb_en = 1
	 * 0x081C 0x02
	 * 0x101C 0x02
	 * 0x181C 0x02
	 * 0x201C 0x02
	 * 0x0880 0x00
	 * 0x1080 0x02
	 * 0x1880 0x00
	 * 0x2080 0x02
	 */
	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0207);
	reg &= USBDP_TRSV_REG0207_LN0_ANA_TX_DRV_ACCDRV_EN_CLR;
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0207);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0407);
	reg &= USBDP_TRSV_REG0407_LN1_ANA_TX_DRV_ACCDRV_EN_CLR;
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0407);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0607);
	reg &= USBDP_TRSV_REG0607_LN2_ANA_TX_DRV_ACCDRV_EN_CLR;
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0607);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0807);
	reg &= USBDP_TRSV_REG0807_LN3_ANA_TX_DRV_ACCDRV_EN_CLR;
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0807);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0220);
	reg &= USBDP_TRSV_REG0220_LN0_ANA_TX_SLB_EN_CLR;
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0220);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0420);
	reg &= USBDP_TRSV_REG0420_LN1_ANA_TX_SLB_EN_CLR;
	reg |= USBDP_TRSV_REG0420_LN1_ANA_TX_SLB_EN_SET(1);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0420);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0620);
	reg &= USBDP_TRSV_REG0620_LN2_ANA_TX_SLB_EN_CLR;
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0620);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0820);
	reg &= USBDP_TRSV_REG0820_LN3_ANA_TX_SLB_EN_CLR;
	reg |= USBDP_TRSV_REG0820_LN3_ANA_TX_SLB_EN_SET(1);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0820);

	/*
	 * ln0/2_bist_auto_run
	 * 0x0BFC 0x80
	 * 0x1BFC 0x80
	 */
	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG02FF);
	reg &= USBDP_TRSV_REG02FF_LN0_BIST_AUTO_RUN_CLR;
	reg |= USBDP_TRSV_REG02FF_LN0_BIST_AUTO_RUN_SET(1);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG02FF);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG06FF);
	reg &= USBDP_TRSV_REG06FF_LN2_BIST_AUTO_RUN_CLR;
	reg |= USBDP_TRSV_REG06FF_LN2_BIST_AUTO_RUN_SET(1);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG06FF);

//	mdelay(5);   // TODO: it is necessary experimentally

	/*
	 * ln0/2_ana_rx_slb_d_lane_sel
	 * ln0/2_ana_rx_slb_en
	 * 0x0A10 0x38
	 * 0x1A10 0x38
	 */
	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0284);
	reg &= USBDP_TRSV_REG0284_LN0_ANA_RX_SLB_D_LANE_SEL_CLR;
	reg |= USBDP_TRSV_REG0284_LN0_ANA_RX_SLB_D_LANE_SEL_SET(1);
	reg &= USBDP_TRSV_REG0284_LN0_ANA_RX_SLB_EN_CLR;
	reg |= USBDP_TRSV_REG0284_LN0_ANA_RX_SLB_EN_SET(1);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0284);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0684);
	reg &= USBDP_TRSV_REG0684_LN2_ANA_RX_SLB_D_LANE_SEL_CLR;
	reg |= USBDP_TRSV_REG0684_LN2_ANA_RX_SLB_D_LANE_SEL_SET(1);
	reg &= USBDP_TRSV_REG0684_LN2_ANA_RX_SLB_EN_CLR;
	reg |= USBDP_TRSV_REG0684_LN2_ANA_RX_SLB_EN_SET(1);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0684);

	udelay(10);

	temp = info->used_phy_port;
	info->used_phy_port = 0;
	ret |= phy_exynos_usbdp_g2_v4_pma_check_cdr_lock(info);
	info->used_phy_port = 1;
	ret |= phy_exynos_usbdp_g2_v4_pma_check_cdr_lock(info);
	info->used_phy_port = temp;

	/*
	 * ln0/2_bist_rx_en
	 * 0x0C00 0xE4
	 * 0x1C00 0xE4
	 */
	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0300);
	reg &= USBDP_TRSV_REG0300_LN0_BIST_RX_EN_CLR;
	reg |= USBDP_TRSV_REG0300_LN0_BIST_RX_EN_SET(1);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0300);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0700);
	reg &= USBDP_TRSV_REG0700_LN2_BIST_RX_EN_CLR;
	reg |= USBDP_TRSV_REG0700_LN2_BIST_RX_EN_SET(1);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0700);

	if (ret)
		return -3;

	return 0;
}

static int phy_exynos_usbdp_g2_v4_pma_bist_result(struct exynos_usbphy_info *info)
{
	void __iomem *regs_base = info->pma_base;
	u32 reg;
	u32 pass_flag = 0, start_flag = 0;

	/*
	 * LN0 BIST pass/start flag
	 * LN2 BIST pass/start flag
	 * 0x0F20 (Read)
	 * 0x1F20 (Read)
	 */
	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG03C8);
	pass_flag = USBDP_TRSV_REG03C8_LN0_MON_BIST_COMP_TEST_GET(reg);
	start_flag = USBDP_TRSV_REG03C8_LN0_MON_BIST_COMP_START_GET(reg);

	if (!(pass_flag & start_flag))
		return -4;

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG07C8);
	pass_flag = USBDP_TRSV_REG07C8_LN2_MON_BIST_COMP_TEST_GET(reg);
	start_flag = USBDP_TRSV_REG07C8_LN2_MON_BIST_COMP_START_GET(reg);

	if (!(pass_flag & start_flag))
		return -5;

	return 0;
}

int phy_exynos_usbdp_g2_v4_internal_loopback(struct exynos_usbphy_info *info, u32 cmn_rate)
{
	int ret;
	int temp;

	phy_exynos_usbdp_g2_v4_ctrl_pma_ready(info);
	phy_exynos_usbdp_g2_v4_aux_force_off(info);
	phy_exynos_usbdp_g2_v4_pma_default_sfr_update(info);
	phy_exynos_usbdp_g2_v4_tune(info);

	phy_exynos_usbdp_g2_v4_pma_ovrd_enable(info, cmn_rate);
	phy_exynos_usbdp_g2_v4_ctrl_pma_rst_release(info);
	phy_exynos_usbdp_g2_v4_pma_ovrd_pcs_rst_release(info);

	do {
		ret = phy_exynos_usbdp_g2_v4_pma_check_pll_lock(info);
		if (ret)
			break;

		phy_exynos_usbdp_g2_v4_pma_ovrd_power_on(info);

		temp = info->used_phy_port;
		info->used_phy_port = 0;
		ret = phy_exynos_usbdp_g2_v4_pma_check_cdr_lock(info);
		info->used_phy_port = temp;
		if (ret)
			break;

		info->used_phy_port = 1;
		ret = phy_exynos_usbdp_g2_v4_pma_check_cdr_lock(info);
		info->used_phy_port = temp;
		if (ret)
			break;

		mdelay(10); /* it is necessary experimentally */

		ret = phy_exynos_usbdp_g2_v4_pma_bist_en(info);
		if (ret)
			break;

		udelay(100);

		ret = phy_exynos_usbdp_g2_v4_pma_bist_result(info);
		if (ret)
			break;
	} while (0);

	return ret;
}

void phy_exynos_usbdp_g2_v4_eom_init(struct exynos_usbphy_info *info, u32 cmn_rate)
{
	void __iomem *regs_base = info->pma_base;
	u32 reg;

	/* EOM Sample Number ( 2^10 )  ----> applied 2^(sample_number) */
	if (info->used_phy_port == 0) {
		/*
		 * 0x0B7C 0x00 ln0_rx_efom_num_of_sample__13_8 = 0
		 * 0x0B80 0x0A ln0_rx_efom_num_of_sample__7_0 = 0xa
		 */
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG02DF);
		reg &= USBDP_TRSV_REG02DF_LN0_RX_EFOM_NUM_OF_SAMPLE__13_8_CLR;
		reg |= USBDP_TRSV_REG02DF_LN0_RX_EFOM_NUM_OF_SAMPLE__13_8_SET(0);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG02DF);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG02E0);
		reg &= USBDP_TRSV_REG02E0_LN0_RX_EFOM_NUM_OF_SAMPLE__7_0_CLR;
		if (!cmn_rate)
			reg |= USBDP_TRSV_REG02E0_LN0_RX_EFOM_NUM_OF_SAMPLE__7_0_SET(0xa);
		else
			reg |= USBDP_TRSV_REG02E0_LN0_RX_EFOM_NUM_OF_SAMPLE__7_0_SET(0xE);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG02E0);
	} else {
		/*
		 * 0x1B7C 0x00 ln2_rx_efom_num_of_sample__13_8 = 0
		 * 0x1B80 0x0A ln2_rx_efom_num_of_sample__7_0 = 0xa
		 */
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG06DF);
		reg &= USBDP_TRSV_REG06DF_LN2_RX_EFOM_NUM_OF_SAMPLE__13_8_CLR;
		reg |= USBDP_TRSV_REG06DF_LN2_RX_EFOM_NUM_OF_SAMPLE__13_8_SET(0);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG06DF);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG06E0);
		reg &= USBDP_TRSV_REG06E0_LN2_RX_EFOM_NUM_OF_SAMPLE__7_0_CLR;
		if (!cmn_rate)
			reg |= USBDP_TRSV_REG06E0_LN2_RX_EFOM_NUM_OF_SAMPLE__7_0_SET(0xa);
		else
			reg |= USBDP_TRSV_REG06E0_LN2_RX_EFOM_NUM_OF_SAMPLE__7_0_SET(0xE);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG06E0);
	}

	/* ovrd adap en, eom_en */
	if (info->used_phy_port == 0) {
		/*
		 * 0x09D0 0x01 ovrd_ln0_rx_dfe_vref_odd_ctrl = 1
		 * 0x09D8 0x01 ovrd_ln0_rx_dfe_vref_even_ctrl = 1
		 */
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0274);
		reg |= USBDP_TRSV_REG0274_OVRD_LN0_RX_DFE_VREF_ODD_CTRL_SET(1);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0274);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0276);
		reg |= USBDP_TRSV_REG0276_OVRD_LN0_RX_DFE_VREF_EVEN_CTRL_SET(1);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0276);
		/*
		 * 0x099C 0x2F ovrd_ln0_rx_dfe_adap_en = 1
		 * ln0_rx_dfe_adap_en = 0
		 * ovrd_ln0_rx_dfe_eom_en = 1
		 * ln0_rx_dfe_eom_en = 1
		 */
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0267);
		reg |= USBDP_TRSV_REG0267_OVRD_LN0_RX_DFE_ADAP_EN_SET(1);
		reg &= USBDP_TRSV_REG0267_LN0_RX_DFE_ADAP_EN_CLR;
		reg |= USBDP_TRSV_REG0267_OVRD_LN0_RX_DFE_EOM_EN_SET(1);
		reg |= USBDP_TRSV_REG0267_LN0_RX_DFE_EOM_EN_SET(1);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0267);
	} else {
		/*
		 * 0x19D0 0x01 ovrd_ln2_rx_dfe_vref_odd_ctrl = 1
		 * 0x19D8 0x01 ovrd_ln2_rx_dfe_vref_even_ctrl = 1
		 */
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0674);
		reg |= USBDP_TRSV_REG0674_OVRD_LN2_RX_DFE_VREF_ODD_CTRL_SET(1);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0674);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0676);
		reg |= USBDP_TRSV_REG0676_OVRD_LN2_RX_DFE_VREF_EVEN_CTRL_SET(1);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0676);

		/*
		 * 0x199C 0x2F ovrd_ln2_rx_dfe_adap_en = 1
		 * ln2_rx_dfe_adap_en = 0
		 * ovrd_ln2_rx_dfe_eom_en = 1
		 * ln2_rx_dfe_eom_en = 1
		 */
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0667);
		reg |= USBDP_TRSV_REG0667_OVRD_LN2_RX_DFE_ADAP_EN_SET(1);
		reg &= USBDP_TRSV_REG0667_LN2_RX_DFE_ADAP_EN_CLR;
		reg |= USBDP_TRSV_REG0667_OVRD_LN2_RX_DFE_EOM_EN_SET(1);
		reg |= USBDP_TRSV_REG0667_LN2_RX_DFE_EOM_EN_SET(1);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0667);
	}

	/* PI STR ( 0 min, f:max ) */
	if (info->used_phy_port == 0) {
		/*
		 * 0x09AC 0x03 ln0_ana_rx_dfe_eom_pi_str_ctrl = 3
		 */
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG026B);
		reg &= USBDP_TRSV_REG026B_LN0_ANA_RX_DFE_EOM_PI_STR_CTRL_CLR;
		if (!cmn_rate)
			reg |= USBDP_TRSV_REG026B_LN0_ANA_RX_DFE_EOM_PI_STR_CTRL_SET(0x3);
		else
			reg |= USBDP_TRSV_REG026B_LN0_ANA_RX_DFE_EOM_PI_STR_CTRL_SET(0xF);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG026B);
	} else {
		/*
		 * 0x19AC 0x03 ln2_ana_rx_dfe_eom_pi_str_ctrl = 3
		 */
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG066B);
		reg &= USBDP_TRSV_REG066B_LN2_ANA_RX_DFE_EOM_PI_STR_CTRL_CLR;
		if (!cmn_rate)
			reg |= USBDP_TRSV_REG066B_LN2_ANA_RX_DFE_EOM_PI_STR_CTRL_SET(0x3);
		else
			reg |= USBDP_TRSV_REG066B_LN2_ANA_RX_DFE_EOM_PI_STR_CTRL_SET(0xF);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG066B);
	}


	/* EOM MODE */
	if (info->used_phy_port == 0) {
		/*
		 * 0x0B70 0x10 ln0_rx_efom_mode = 4
		 */
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG02DC);
		reg &= USBDP_TRSV_REG02DC_LN0_RX_EFOM_MODE_CLR;
		reg |= USBDP_TRSV_REG02DC_LN0_RX_EFOM_MODE_SET(0x4);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG02DC);
	} else {
		/*
		 * 0x1B70 0x10 ln2_rx_efom_mode = 4
		 */
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG06DC);
		reg &= USBDP_TRSV_REG06DC_LN2_RX_EFOM_MODE_CLR;
		reg |= USBDP_TRSV_REG06DC_LN2_RX_EFOM_MODE_SET(0x4);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG06DC);
	}

	/* EOM START SSM DISABLE */
	if (info->used_phy_port == 0) {
		/*
		 * 0x0B74 0x10 ln0_rx_efom_start_ssm_disable = 1
		 */
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG02DD);
		reg &= USBDP_TRSV_REG02DD_LN0_RX_EFOM_START_SSM_DISABLE_CLR;
		reg |= USBDP_TRSV_REG02DD_LN0_RX_EFOM_START_SSM_DISABLE_SET(1);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG02DD);
	} else {
		/*
		 * 0x1B74 0x10 ln2_rx_efom_start_ssm_disable = 1
		 */
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG06DD);
		reg &= USBDP_TRSV_REG06DD_LN2_RX_EFOM_START_SSM_DISABLE_CLR;
		reg |= USBDP_TRSV_REG06DD_LN2_RX_EFOM_START_SSM_DISABLE_SET(1);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG06DD);
	}

	/*
	 * PCS EOM INPUT
	 * 0x0394 0x0C ovrd_pcs_rx_fom_en = 1
	 * pcs_rx_fom_en = 1
	 */
	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG00E5);
	reg |= USBDP_CMN_REG00E5_OVRD_PCS_RX_FOM_EN_SET(1);
	reg |= USBDP_CMN_REG00E5_PCS_RX_FOM_EN_SET(1);
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG00E5);

	/* NON DATA, DE-SER EN */
	if (info->used_phy_port == 0) {
		/*
		 * 0x0978 0x2C ovrd_ln0_rx_des_non_data_sel = 1
		 * ln0_rx_des_non_data_sel = 0
		 * ovrd_ln0_rx_des_rstn = 1
		 * ln0_rx_des_rstn = 1
		 */
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG025E);
		reg |= USBDP_TRSV_REG025E_OVRD_LN0_RX_DES_NON_DATA_SEL_SET(1);
		reg |= USBDP_TRSV_REG025E_OVRD_LN0_RX_DES_RSTN_SET(1);
		reg |= USBDP_TRSV_REG025E_LN0_RX_DES_RSTN_SET(1);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG025E);
	} else {
		/*
		 * 0x1978 0x2C ovrd_ln2_rx_des_non_data_sel = 1
		 * ln2_rx_des_non_data_sel = 0
		 * ovrd_ln2_rx_des_rstn = 1
		 * ln2_rx_des_rstn = 1
		 */
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG065E);
		reg |= USBDP_TRSV_REG065E_OVRD_LN2_RX_DES_NON_DATA_SEL_SET(1);
		reg |= USBDP_TRSV_REG065E_OVRD_LN2_RX_DES_RSTN_SET(1);
		reg |= USBDP_TRSV_REG065E_LN2_RX_DES_RSTN_SET(1);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG065E);
	}

	/*
	 * EOM CLOCK DIV  <------ Need to Confirm about value, it just predicted value
	 * it was set at phy_exynos_usbdp_g2_v2_pma_default_sfr_update
	 * 0x09A0 0x24
	 * 0x19A0 0x24
	 */
	if (info->used_phy_port == 0) {
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0268);
		reg |= USBDP_TRSV_REG0268_LN0_RX_DFE_EOM_PI_DIV_SEL_SP_SET(4);
		if (!cmn_rate) {
			reg &= USBDP_TRSV_REG0268_LN0_RX_DFE_EOM_PI_DIV_SEL_SSP_CLR;
			reg |= USBDP_TRSV_REG0268_LN0_RX_DFE_EOM_PI_DIV_SEL_SSP_SET(4);
		} else {
			reg &= USBDP_TRSV_REG0268_LN0_RX_DFE_EOM_PI_DIV_SEL_SSP_CLR;
			reg |= USBDP_TRSV_REG0268_LN0_RX_DFE_EOM_PI_DIV_SEL_SSP_SET(2);
		}
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0268);
	} else {
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0668);
		reg |= USBDP_TRSV_REG0668_LN2_RX_DFE_EOM_PI_DIV_SEL_SP_SET(4);
		if (!cmn_rate) {
			reg &= USBDP_TRSV_REG0668_LN2_RX_DFE_EOM_PI_DIV_SEL_SSP_CLR;
			reg |= USBDP_TRSV_REG0668_LN2_RX_DFE_EOM_PI_DIV_SEL_SSP_SET(4);
		} else {
			reg &= USBDP_TRSV_REG0668_LN2_RX_DFE_EOM_PI_DIV_SEL_SSP_CLR;
			reg |= USBDP_TRSV_REG0668_LN2_RX_DFE_EOM_PI_DIV_SEL_SSP_SET(2);
		}
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0668);
	}

	/* EOM BIT WIDTH  <--- Need to Confirm about value, it just predicted value */
	if (info->used_phy_port == 0) {
		/*
		 * Gen2
		 * 0x0B78 0x47 ln0_rx_efom_settle_time = 4
		 * ln0_rx_efom_bit_width_sel = 1
		 *
		 * Gen1
		 * 0x0B78 0x4F ln0_rx_efom_settle_time = 4
		 * ln0_rx_efom_bit_width_sel = 3
		 */
		if (cmn_rate) {
			reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG02DE);
			reg &= USBDP_TRSV_REG02DE_LN0_RX_EFOM_SETTLE_TIME_CLR;
			reg |= USBDP_TRSV_REG02DE_LN0_RX_EFOM_SETTLE_TIME_SET(0x4);
			reg &= USBDP_TRSV_REG02DE_LN0_RX_EFOM_BIT_WIDTH_SEL_CLR;
			reg |= USBDP_TRSV_REG02DE_LN0_RX_EFOM_BIT_WIDTH_SEL_SET(0x1);
			writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG02DE);
		} else {
			reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG02DE);
			reg &= USBDP_TRSV_REG02DE_LN0_RX_EFOM_SETTLE_TIME_CLR;
			reg |= USBDP_TRSV_REG02DE_LN0_RX_EFOM_SETTLE_TIME_SET(0x4);
			reg &= USBDP_TRSV_REG02DE_LN0_RX_EFOM_BIT_WIDTH_SEL_CLR;
			reg |= USBDP_TRSV_REG02DE_LN0_RX_EFOM_BIT_WIDTH_SEL_SET(0x3);
			writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG02DE);
		}
	} else {
		/*
		 * Gen2
		 * 0x1B78 0x47 ln2_rx_efom_settle_time = 4
		 * ln2_rx_efom_bit_width_sel = 1
		 *
		 * Gen1
		 * 0x1B78 0x4F ln2_rx_efom_settle_time = 4
		 * ln2_rx_efom_bit_width_sel = 3
		 */
		if (cmn_rate) {
			reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG06DE);
			reg &= USBDP_TRSV_REG06DE_LN2_RX_EFOM_SETTLE_TIME_CLR;
			reg |= USBDP_TRSV_REG06DE_LN2_RX_EFOM_SETTLE_TIME_SET(0x4);
			reg &= USBDP_TRSV_REG06DE_LN2_RX_EFOM_BIT_WIDTH_SEL_CLR;
			reg |= USBDP_TRSV_REG06DE_LN2_RX_EFOM_BIT_WIDTH_SEL_SET(0x1);
			writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG06DE);
		} else {
			reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG06DE);
			reg &= USBDP_TRSV_REG06DE_LN2_RX_EFOM_SETTLE_TIME_CLR;
			reg |= USBDP_TRSV_REG06DE_LN2_RX_EFOM_SETTLE_TIME_SET(0x4);
			reg &= USBDP_TRSV_REG06DE_LN2_RX_EFOM_BIT_WIDTH_SEL_CLR;
			reg |= USBDP_TRSV_REG06DE_LN2_RX_EFOM_BIT_WIDTH_SEL_SET(0x3);
			writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG06DE);
		}
	}

	/*
	 * Switch to 6:New EOM, E:legacy EFOM mode
	 * 0x0450 0x06 efom_legacy_mode_en = 0
	 */
	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG0114);
	if (!cmn_rate)
		/* Gen1 : New EOM Mode */
		reg &= USBDP_CMN_REG0114_EFOM_LEGACY_MODE_EN_CLR;
	else
		/* Gen2 10Ghz_CCO : Legacy EOM Mode */
		reg |= USBDP_CMN_REG0114_EFOM_LEGACY_MODE_EN_SET(1);
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG0114);
}

static void phy_exynos_usbdp_g2_v4_eom_deinit(struct exynos_usbphy_info *info)
{
	void __iomem *regs_base = info->pma_base;
	u32 reg;

	/* EOM START ovrd clear */
	if (info->used_phy_port == 0) {
		/*
		 * 0x0B70 0x10 ln0_ovrd_rx_efom_start = 0
		 */
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG02DC);
		reg &= USBDP_TRSV_REG02DC_LN0_OVRD_RX_EFOM_START_CLR;
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG02DC);
	} else {
		/*
		 * 0x1B70 0x10 ln2_ovrd_rx_efom_start = 0
		 */
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG06DC);
		reg &= USBDP_TRSV_REG06DC_LN2_OVRD_RX_EFOM_START_CLR;
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG06DC);
	}


	/* EOM Sample Number clear */
	if (info->used_phy_port == 0) {
		/*
		 * 0x0B7C 0x00 ln0_rx_efom_num_of_sample__13_8 = 0
		 * 0x0B80 0x00 ln0_rx_efom_num_of_sample__7_0 = 0
		 */
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG02DF);
		reg &= USBDP_TRSV_REG02DF_LN0_RX_EFOM_NUM_OF_SAMPLE__13_8_CLR;
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG02DF);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG02E0);
		reg &= USBDP_TRSV_REG02E0_LN0_RX_EFOM_NUM_OF_SAMPLE__7_0_CLR;
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG02E0);
	} else {
		/*
		 * 0x1B7C 0x00 ln2_rx_efom_num_of_sample__13_8 = 0
		 * 0x1B80 0x00 ln2_rx_efom_num_of_sample__7_0 = 0
		 */
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG06DF);
		reg &= USBDP_TRSV_REG06DF_LN2_RX_EFOM_NUM_OF_SAMPLE__13_8_CLR;
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG06DF);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG06E0);
		reg &= USBDP_TRSV_REG06E0_LN2_RX_EFOM_NUM_OF_SAMPLE__7_0_CLR;
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG06E0);
	}

	/* ovrd adap en, eom_en */
	if (info->used_phy_port == 0) {
		/*
		 * 0x09D0 0x00 ovrd_ln0_rx_dfe_vref_odd_ctrl = 0
		 * 0x09D8 0x00 ovrd_ln0_rx_dfe_vref_even_ctrl = 0
		 */
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0274);
		reg &= USBDP_TRSV_REG0274_OVRD_LN0_RX_DFE_VREF_ODD_CTRL_CLR;
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0274);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0276);
		reg &= USBDP_TRSV_REG0276_OVRD_LN0_RX_DFE_VREF_EVEN_CTRL_CLR;
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0276);

		/*
		 * 0x099C 0x1C ovrd_ln0_rx_dfe_adap_en = 0
		 * ln0_rx_dfe_adap_en = 1
		 * ovrd_ln0_rx_dfe_eom_en = 0
		 * ln0_rx_dfe_eom_en = 0
		 */
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0267);
		reg &= USBDP_TRSV_REG0267_OVRD_LN0_RX_DFE_ADAP_EN_CLR;
		reg |= USBDP_TRSV_REG0267_LN0_RX_DFE_ADAP_EN_SET(1);
		reg &= USBDP_TRSV_REG0267_OVRD_LN0_RX_DFE_EOM_EN_CLR;
		reg &= USBDP_TRSV_REG0267_LN0_RX_DFE_EOM_EN_CLR;
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0267);
	} else {
		/*
		 * 0x19D0 0x00 ovrd_ln2_rx_dfe_vref_odd_ctrl = 0
		 * 0x19D8 0x00 ovrd_ln2_rx_dfe_vref_even_ctrl = 0
		 */
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0674);
		reg &= USBDP_TRSV_REG0674_OVRD_LN2_RX_DFE_VREF_ODD_CTRL_CLR;
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0674);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0676);
		reg &= USBDP_TRSV_REG0676_OVRD_LN2_RX_DFE_VREF_EVEN_CTRL_CLR;
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0676);

		/*
		 * 0x099C 0x1C ovrd_ln0_rx_dfe_adap_en = 0
		 * ln0_rx_dfe_adap_en = 1
		 * ovrd_ln0_rx_dfe_eom_en = 0
		 * ln0_rx_dfe_eom_en = 0
		 */
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0667);
		reg &= USBDP_TRSV_REG0667_OVRD_LN2_RX_DFE_ADAP_EN_CLR;
		reg |= USBDP_TRSV_REG0667_LN2_RX_DFE_ADAP_EN_SET(1);
		reg &= USBDP_TRSV_REG0667_OVRD_LN2_RX_DFE_EOM_EN_CLR;
		reg &= USBDP_TRSV_REG0667_LN2_RX_DFE_EOM_EN_CLR;
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0667);
	}

	/* PI STR ( 0 min, f:max ) */
	if (info->used_phy_port == 0) {
		/*
		 * 0x09AC 0x00 ln0_ana_rx_dfe_eom_pi_str_ctrl = 0
		 */
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG026B);
		reg &= USBDP_TRSV_REG026B_LN0_ANA_RX_DFE_EOM_PI_STR_CTRL_CLR;
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG026B);
	} else {
		/*
		 * 0x19AC 0x00 ln2_ana_rx_dfe_eom_pi_str_ctrl = 0
		 */
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG066B);
		reg &= USBDP_TRSV_REG066B_LN2_ANA_RX_DFE_EOM_PI_STR_CTRL_CLR;
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG066B);
	}

	/* EOM MODE */
	if (info->used_phy_port == 0) {
		/*
		 * 0x0B70 0x00 ln0_rx_efom_mode = 0
		 */
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG02DC);
		reg &= USBDP_TRSV_REG02DC_LN0_RX_EFOM_MODE_CLR;
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG02DC);
	} else {
		/*
		 * 0x1B70 0x00 ln2_rx_efom_mode = 0
		 */
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG06DC);
		reg &= USBDP_TRSV_REG06DC_LN2_RX_EFOM_MODE_CLR;
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG06DC);
	}

	/* EOM START SSM DISABLE */
	if (info->used_phy_port == 0) {
		/*
		 * 0x0B74 0x00 ln0_rx_efom_start_ssm_disable = 0
		 */
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG02DD);
		reg &= USBDP_TRSV_REG02DD_LN0_RX_EFOM_START_SSM_DISABLE_CLR;
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG02DD);
	} else {
		/*
		 * 0x1B74 0x00 ln2_rx_efom_start_ssm_disable = 0
		 */
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG06DD);
		reg &= USBDP_TRSV_REG06DD_LN2_RX_EFOM_START_SSM_DISABLE_CLR;
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG06DD);
	}

	/*
	 * PCS EOM INPUT
	 * 0x0394 0x00 ovrd_pcs_rx_fom_en = 0
	 * pcs_rx_fom_en = 0
	 */
	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG00E5);
	reg &= USBDP_CMN_REG00E5_OVRD_PCS_RX_FOM_EN_CLR;
	reg &= USBDP_CMN_REG00E5_PCS_RX_FOM_EN_CLR;
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG00E5);

	/* NON DATA, DE-SER EN */
	if (info->used_phy_port == 0) {
		/*
		 * 0x0978 0x00 ovrd_ln0_rx_des_non_data_sel = 0
		 * ovrd_ln0_rx_des_rstn = 0
		 * ln0_rx_des_rstn = 0
		 */
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG025E);
		reg &= USBDP_TRSV_REG025E_OVRD_LN0_RX_DES_NON_DATA_SEL_CLR;
		reg &= USBDP_TRSV_REG025E_OVRD_LN0_RX_DES_RSTN_CLR;
		reg &= USBDP_TRSV_REG025E_LN0_RX_DES_RSTN_CLR;
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG025E);
	} else {
		/*
		 * 0x1978 0x00 ovrd_ln2_rx_des_non_data_sel = 0
		 * ovrd_ln2_rx_des_rstn = 0
		 * ln2_rx_des_rstn = 0
		 */
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG065E);
		reg &= USBDP_TRSV_REG065E_OVRD_LN2_RX_DES_NON_DATA_SEL_CLR;
		reg &= USBDP_TRSV_REG065E_OVRD_LN2_RX_DES_RSTN_CLR;
		reg &= USBDP_TRSV_REG065E_LN2_RX_DES_RSTN_CLR;
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG065E);
	}

	/* EOM BIT WIDTH */
	if (info->used_phy_port == 0) {
		/*
		 * 0x0B78 0x07 ln0_rx_efom_settle_time = 0
		 * ln0_rx_efom_bit_width_sel = 1
		 */
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG02DE);
		reg &= USBDP_TRSV_REG02DE_LN0_RX_EFOM_SETTLE_TIME_CLR;
		reg &= USBDP_TRSV_REG02DE_LN0_RX_EFOM_BIT_WIDTH_SEL_CLR;
		reg |= USBDP_TRSV_REG02DE_LN0_RX_EFOM_BIT_WIDTH_SEL_SET(0x1);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG02DE);
	} else {
		/*
		 * 0x1B78 0x07 ln2_rx_efom_settle_time = 0
		 * ln2_rx_efom_bit_width_sel = 1
		 */
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG06DE);
		reg &= USBDP_TRSV_REG06DE_LN2_RX_EFOM_SETTLE_TIME_CLR;
		reg &= USBDP_TRSV_REG06DE_LN2_RX_EFOM_BIT_WIDTH_SEL_CLR;
		reg |= USBDP_TRSV_REG06DE_LN2_RX_EFOM_BIT_WIDTH_SEL_SET(0x1);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG06DE);
	}
}


void phy_exynos_usbdp_g2_v4_eom_start(struct exynos_usbphy_info *info, u32 ph_sel, u32 def_vref)
{
	void __iomem *regs_base = info->pma_base;
	u32 reg;

	/* EOM PHASE SETTING */
	if (info->used_phy_port == 0) {
		/*
		 * 0x0B94  ln0_rx_efom_eom_ph_sel
		 */
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG02E5);
		reg &= USBDP_TRSV_REG02E5_LN0_RX_EFOM_EOM_PH_SEL_CLR;
		reg |= USBDP_TRSV_REG02E5_LN0_RX_EFOM_EOM_PH_SEL_SET(ph_sel);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG02E5);
	} else {
		/*
		 * 0x1B94  ln2_rx_efom_eom_ph_sel
		 */
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG06E5);
		reg &= USBDP_TRSV_REG06E5_LN2_RX_EFOM_EOM_PH_SEL_CLR;
		reg |= USBDP_TRSV_REG06E5_LN2_RX_EFOM_EOM_PH_SEL_SET(ph_sel);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG06E5);
	}

	/* EOM VREF SETTING */
	if (info->used_phy_port == 0) {
		/*
		 * 0x09D4  ln0_rx_dfe_vref_odd_ctrl__7_0
		 * 0x09DC  ln0_rx_dfe_vref_even_ctrl__7_0
		 */
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0275);
		reg &= USBDP_TRSV_REG0275_LN0_RX_DFE_VREF_ODD_CTRL__7_0_CLR;
		reg |= USBDP_TRSV_REG0275_LN0_RX_DFE_VREF_ODD_CTRL__7_0_SET(def_vref);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0275);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0277);
		reg &= USBDP_TRSV_REG0277_LN0_RX_DFE_VREF_EVEN_CTRL__7_0_CLR;
		reg |= USBDP_TRSV_REG0277_LN0_RX_DFE_VREF_EVEN_CTRL__7_0_SET(def_vref);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0277);
	} else {
		/*
		 * 0x19D4  ln2_rx_dfe_vref_odd_ctrl__7_0
		 * 0x19DC  ln2_rx_dfe_vref_even_ctrl__7_0
		 */
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0675);
		reg &= USBDP_TRSV_REG0675_LN2_RX_DFE_VREF_ODD_CTRL__7_0_CLR;
		reg |= USBDP_TRSV_REG0675_LN2_RX_DFE_VREF_ODD_CTRL__7_0_SET(def_vref);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0675);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0677);
		reg &= USBDP_TRSV_REG0677_LN2_RX_DFE_VREF_EVEN_CTRL__7_0_CLR;
		reg |= USBDP_TRSV_REG0677_LN2_RX_DFE_VREF_EVEN_CTRL__7_0_SET(def_vref);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0677);
	}

	/* EOM START */
	if (info->used_phy_port == 0) {
		/*
		 * 0x0B70 0x13 ln0_ovrd_rx_efom_start = 1
		 * ln0_rx_efom_start = 1
		 */
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG02DC);
		reg |= USBDP_TRSV_REG02DC_LN0_OVRD_RX_EFOM_START_SET(1);
		reg |= USBDP_TRSV_REG02DC_LN0_RX_EFOM_START_SET(1);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG02DC);
	} else {
		/*
		 * 0x1B70 0x13 ln2_ovrd_rx_efom_start = 1
		 * ln2_rx_efom_start = 1
		 */
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG06DC);
		reg |= USBDP_TRSV_REG06DC_LN2_OVRD_RX_EFOM_START_SET(1);
		reg |= USBDP_TRSV_REG06DC_LN2_RX_EFOM_START_SET(1);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG06DC);
	}
}


int phy_exynos_usbdp_g2_v4_eom_get_done_status(struct exynos_usbphy_info *info)
{
	void __iomem *regs_base = info->pma_base;
	u32 reg;
	u32 eom_done;

	/* Check efom_done */
	if (info->used_phy_port == 0) {
		/*
		 * 0x0F98  ln0_mon_rx_efom_done
		 */
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG03E6);
		eom_done = USBDP_TRSV_REG03E6_LN0_MON_RX_EFOM_DONE_GET(reg);
	} else {
		/*
		 * 0x1F98  ln2_mon_rx_efom_done
		 */
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG07E6);
		eom_done = USBDP_TRSV_REG07E6_LN2_MON_RX_EFOM_DONE_GET(reg);
	}

	if (eom_done)
		return 0;

	return -1;
}


u64 phy_exynos_usbdp_g2_v4_eom_get_err_cnt(struct exynos_usbphy_info *info, u32 cmn_rate)
{
	void __iomem *regs_base = info->pma_base;
	u32 reg;
	u64 err_cnt_lo = 0;
	u64 err_cnt_hi = 0;

	/* Get error count */
	if (!cmn_rate) {
		if (info->used_phy_port == 0) {
			/*
			 * 0x077C  ln0_mon_rx_efom_err_cnt__7_0
			 * 0x0778  ln0_mon_rx_efom_err_cnt__15_8
			 * 0x0774  ln0_mon_rx_efom_err_cnt__23_16
			 * 0x0770  ln0_mon_rx_efom_err_cnt__31_24
			 * 0x076C  ln0_mon_rx_efom_err_cnt__39_32
			 * 0x0768  ln0_mon_rx_efom_err_cnt__47_40
			 * 0x0764  ln0_mon_rx_efom_err_cnt__52_48
			 */
			reg = readl(regs_base + EXYNOS_USBDP_CMN_REG01DF);
			err_cnt_lo |=
			USBDP_CMN_REG01DF_LN0_MON_RX_EFOM_ERR_CNT__7_0_GET(reg) << 0;
			reg = readl(regs_base + EXYNOS_USBDP_CMN_REG01DE);
			err_cnt_lo |=
			USBDP_CMN_REG01DE_LN0_MON_RX_EFOM_ERR_CNT__15_8_GET(reg) << 8;
			reg = readl(regs_base + EXYNOS_USBDP_CMN_REG01DD);
			err_cnt_lo |=
			USBDP_CMN_REG01DD_LN0_MON_RX_EFOM_ERR_CNT__23_16_GET(reg) << 16;
			reg = readl(regs_base + EXYNOS_USBDP_CMN_REG01DC);
			err_cnt_lo |=
			USBDP_CMN_REG01DC_LN0_MON_RX_EFOM_ERR_CNT__31_24_GET(reg) << 24;
			reg = readl(regs_base + EXYNOS_USBDP_CMN_REG01DB);
			err_cnt_hi |=
			USBDP_CMN_REG01DB_LN0_MON_RX_EFOM_ERR_CNT__39_32_GET(reg) << 0;
			reg = readl(regs_base + EXYNOS_USBDP_CMN_REG01DA);
			err_cnt_hi |=
			USBDP_CMN_REG01DA_LN0_MON_RX_EFOM_ERR_CNT__47_40_GET(reg) << 4;
			reg = readl(regs_base + EXYNOS_USBDP_CMN_REG01D9);
			err_cnt_hi |=
			USBDP_CMN_REG01D9_LN0_MON_RX_EFOM_ERR_CNT__52_48_GET(reg) << 8;
		} else {
			/*
			 * 0x0798  ln2_mon_rx_efom_err_cnt__7_0
			 * 0x0794  ln2_mon_rx_efom_err_cnt__15_8
			 * 0x0790  ln2_mon_rx_efom_err_cnt__23_16
			 * 0x078c  ln2_mon_rx_efom_err_cnt__31_24
			 * 0x0788  ln2_mon_rx_efom_err_cnt__39_32
			 * 0x0784  ln2_mon_rx_efom_err_cnt__47_40
			 * 0x0780  ln2_mon_rx_efom_err_cnt__52_48
			 */
			reg = readl(regs_base + EXYNOS_USBDP_CMN_REG01E6);
			err_cnt_lo |=
			USBDP_CMN_REG01E6_LN2_MON_RX_EFOM_ERR_CNT__7_0_GET(reg) << 0;
			reg = readl(regs_base + EXYNOS_USBDP_CMN_REG01E5);
			err_cnt_lo |=
			USBDP_CMN_REG01E5_LN2_MON_RX_EFOM_ERR_CNT__15_8_GET(reg) << 8;
			reg = readl(regs_base + EXYNOS_USBDP_CMN_REG01E4);
			err_cnt_lo |=
			USBDP_CMN_REG01E4_LN2_MON_RX_EFOM_ERR_CNT__23_16_GET(reg) << 16;
			reg = readl(regs_base + EXYNOS_USBDP_CMN_REG01E3);
			err_cnt_lo |=
			USBDP_CMN_REG01E3_LN2_MON_RX_EFOM_ERR_CNT__31_24_GET(reg) << 24;
			reg = readl(regs_base + EXYNOS_USBDP_CMN_REG01E2);
			err_cnt_hi |=
			USBDP_CMN_REG01E2_LN2_MON_RX_EFOM_ERR_CNT__39_32_SET(reg) << 0;
			reg = readl(regs_base + EXYNOS_USBDP_CMN_REG01E1);
			err_cnt_hi |=
			USBDP_CMN_REG01E1_LN2_MON_RX_EFOM_ERR_CNT__47_40_GET(reg) << 4;
			reg = readl(regs_base + EXYNOS_USBDP_CMN_REG01E0);
			err_cnt_hi |=
			USBDP_CMN_REG01E0_LN2_MON_RX_EFOM_ERR_CNT__52_48_SET(reg) << 8;
		}
		return err_cnt_hi << 32 | err_cnt_lo;
	}

	if (info->used_phy_port == 0) {
		err_cnt_lo = readl(regs_base + EXYNOS_USBDP_TRSV_REG03E8);
		err_cnt_hi = readl(regs_base + EXYNOS_USBDP_TRSV_REG03E7);
	} else {
		err_cnt_lo = readl(regs_base + EXYNOS_USBDP_TRSV_REG07E8);
		err_cnt_hi = readl(regs_base + EXYNOS_USBDP_TRSV_REG07E7);
	}

	return err_cnt_hi << 8 | err_cnt_lo;

}

void phy_exynos_usbdp_g2_v4_eom_stop(struct exynos_usbphy_info *info)
{
	void __iomem *regs_base = info->pma_base;
	u32 reg;

	/* EOM STOP */
	if (info->used_phy_port == 0) {
		/*
		 * 0x0B70 0x12 ln0_rx_efom_start = 0
		 */
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG02DC);
		reg &= USBDP_TRSV_REG02DC_LN0_RX_EFOM_START_CLR;
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG02DC);
	} else {
		/*
		 * 0x1B70 0x12 ln2_rx_efom_start = 0
		 */
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG06DC);
		reg &= USBDP_TRSV_REG06DC_LN2_RX_EFOM_START_CLR;
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG06DC);
	}
}

void phy_exynos_usbdp_g2_v4_eom(struct exynos_usbphy_info *info,
					struct usb_eom_result_s *eom_result, u32 cmn_rate)
{
	u32 ph_sel, def_vref = 0;
	u64 err_cnt = 0;
	u32 test_cnt = 0;
	u32 ui_cnt, ui_no;
	int timeout;

	pr_info("cmn_rate = %d\n", cmn_rate);

	phy_exynos_usbdp_g2_v4_eom_init(info, cmn_rate);    /* 0: Gen1, 1: Gen2 */

	if (!cmn_rate)
		ui_cnt = 1; /* Gen1 */
	else
		ui_cnt = 2; /* Gen2 */


	for (ui_no = 0; ui_no < ui_cnt; ui_no++) {
		for (ph_sel = 0; ph_sel < EOM_PH_SEL_MAX; ph_sel++) {
			for (def_vref = 0; def_vref < EOM_DEF_VREF_MAX; def_vref++) {
				phy_exynos_usbdp_g2_v4_eom_start(info, ph_sel, def_vref);
				timeout = 500;
				while (phy_exynos_usbdp_g2_v4_eom_get_done_status(info)) {
					usleep_range(10, 20);
					timeout--;
					if (!timeout) {
						dev_err(info->dev, "eom timeout error\n");
						break;
					}
				}

				if (!timeout) {
					dev_err(info->dev, "eom test fail - timeout\n");
					break;
				}
				err_cnt = phy_exynos_usbdp_g2_v4_eom_get_err_cnt(info, cmn_rate);
				phy_exynos_usbdp_g2_v4_eom_stop(info);

				/* Save result */
				eom_result[test_cnt].phase = ph_sel;
				eom_result[test_cnt].vref = def_vref;
				eom_result[test_cnt].err = err_cnt;
				test_cnt++;
			}
			if (!timeout) {
				dev_err(info->dev, "eom test fail - exit\n");
				break;
			}
		}
	}

	phy_exynos_usbdp_g2_v4_eom_deinit(info);
}

int phy_exynos_usbdp_g2_v4_enable(struct exynos_usbphy_info *info)
{
	int ret = 0;
	struct reg_set *reg_base =
		(struct reg_set *)info->pma_base;
	struct device *dev = info->dev;
	u32 phy_ref_clock = 0;

	phy_exynos_usbdp_g2_v4_ctrl_pma_ready(info);
	phy_exynos_usbdp_g2_v4_aux_force_off(info);

	if(dev) {
		ret = of_property_read_u32(dev->of_node, "phy_ref_clock", &phy_ref_clock);
		if (ret < 0) {
			dev_err(dev, "Couldn't read phy_ref_clock %s node, error = %d\n",
				dev->of_node->name, ret);
		}
	}

	if(phy_ref_clock == 19200000) {
		// pma default sfr updated for RefClk 19.2Mhz
		phy_exynos_usbdp_g2_v4_pma_default_sfr_update_19_2Mhz(info);
		// pma configuration for RefClk 19.2Mhz-Gen1
		phy_exynos_usbdp_g2_v4_pma_default_sfr_update_19_2Mhz_Gen1(info);
		// pma configuration for RefClk 19.2Mhz-Gen2
		phy_exynos_usbdp_g2_v4_pma_default_sfr_update_19_2Mhz_Gen2(info);
	} else {
		phy_exynos_usbdp_g2_v4_pma_default_sfr_update(info);
	}

	phy_exynos_usbdp_g2_v4_set_pcs(info);
	phy_exynos_usbdp_g2_v4_tune(info);
	phy_exynos_usbdp_g2_v4_pma_lane_mux_sel(info);
	phy_exynos_usbdp_g2_v4_ctrl_pma_rst_release(info);

	ret = phy_exynos_usbdp_g2_v4_pma_check_pll_lock(info);
	if (!ret)
		ret = phy_exynos_usbdp_g2_v4_pma_check_cdr_lock(info);

	mdelay(10);

	phy_exynos_usbdp_g2_v4_tune_late(info);
	phy_exynos_usbdp_g2_v4_pma_check_offset_cal_code(info);

	pr_info("%s: reg000:%p, reg0001:%p\n", __func__,
		&reg_base->reg0000, &reg_base->reg0001);

	return 0;
}

void phy_exynos_usbdp_g2_v4_disable(struct exynos_usbphy_info *info)
{
	void __iomem *regs_base = info->ctrl_base;
	u32 reg;

	// Change pipe pclk to suspend_clk
	reg = readl(regs_base + EXYNOS_USBCON_CLKRST);
	reg &= ~CLKRST_LINK_PCLK_SEL;
	writel(reg, regs_base + EXYNOS_USBCON_CLKRST);

	// powerdown and ropll/lcpll refclk off for reducing powerdown current
	reg = readl(regs_base + EXYNOS_USBCON_COMBO_PMA_CTRL);
	reg &= ~PMA_ROPLL_REF_CLK_SEL_MASK;
	reg |= PMA_ROPLL_REF_CLK_SEL_SET(1);
	reg &= ~PMA_LCPLL_REF_CLK_SEL_MASK;
	reg |= PMA_LCPLL_REF_CLK_SEL_SET(1);
	reg |= PMA_LOW_PWR;
	writel(reg, regs_base + EXYNOS_USBCON_COMBO_PMA_CTRL);
}
