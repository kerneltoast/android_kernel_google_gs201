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
#include <linux/io.h>
#include <linux/kernel.h>
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
		pr_info("[OC] adder e       = %02X\n", code);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG03CB);
		code =
		USBDP_TRSV_REG03CB_LN0_MON_RX_OC_DFE_ADDER_ODD_GET(reg);
		pr_info("[OC] adder o       = %02X\n", code);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG03CC);
		code =
		USBDP_TRSV_REG03CC_LN0_MON_RX_OC_DFE_DAC_ADDER_EVEN_GET(reg);
		pr_info("[OC] dac_adder e/o = %02X", code);
		code =
		USBDP_TRSV_REG03CC_LN0_MON_RX_OC_DFE_DAC_ADDER_ODD_GET(reg);
		pr_info("/%02X\n", code);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG03CD);
		code =
		USBDP_TRSV_REG03CD_LN0_MON_RX_OC_DFE_SA_EDGE_EVEN_GET(reg);
		pr_info("[OC] sa_edge e     = %02X\n", code);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG03CE);
		code =
		USBDP_TRSV_REG03CE_LN0_MON_RX_OC_DFE_SA_EDGE_ODD_GET(reg);
		pr_info("[OC] sa_edge o     = %02X\n", code);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG03CF);
		code =
		USBDP_TRSV_REG03CF_LN0_MON_RX_OC_DFE_DAC_EDGE_ODD_GET(reg);
		pr_info("[OC] dac_edge o/e  = %02X", code);
		code =
		USBDP_TRSV_REG03CF_LN0_MON_RX_OC_DFE_DAC_EDGE_EVEN_GET(reg);
		pr_info("/%02X\n", code);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG03D0);
		code =
		USBDP_TRSV_REG03D0_LN0_MON_RX_OC_DFE_SA_ERR_EVEN_GET(reg);
		pr_info("[OC] sa_err e      = %02X\n", code);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG03D1);
		code =
		USBDP_TRSV_REG03D1_LN0_MON_RX_OC_DFE_SA_ERR_ODD_GET(reg);
		pr_info("[OC] sa_err o      = %02X\n", code);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG03D2);
		code =
		USBDP_TRSV_REG03D2_LN0_MON_RX_OC_DFE_DAC_ERR_EVEN_GET(reg);
		pr_info("[OC] dac_err e/o   = %02X", code);
		code =
		USBDP_TRSV_REG03D2_LN0_MON_RX_OC_DFE_DAC_ERR_ODD_GET(reg);
		pr_info("/%02X\n", code);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG03D3);
		code =
		USBDP_TRSV_REG03D3_LN0_MON_RX_OC_CTLE_GET(reg);
		pr_info("[OC] ctle          = %02X\n", code);

		/* 0x0F5C  ln0_mon_rx_oc_fail__7_0 */
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG03D7);
		code =
		USBDP_TRSV_REG03D7_LN0_MON_RX_OC_FAIL__7_0_GET(reg);
		pr_info("[OC] oc_fail       = %02X\n", code);

		reg = readl(regs_base + EXYNOS_USBDP_CMN_REG01CB);
		code =
		USBDP_CMN_REG01CB_LN0_MON_RX_OC_INIT_VGA_CODE_GET(reg);
		pr_info("[OC] oc_vga        = %02X\n", code);

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
		pr_info("[OC] adder e       = %02X\n", code);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG07CB);
		code =
		USBDP_TRSV_REG07CB_LN2_MON_RX_OC_DFE_ADDER_ODD_GET(reg);
		pr_info("[OC] adder o       = %02X\n", code);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG07CC);
		code =
		USBDP_TRSV_REG07CC_LN2_MON_RX_OC_DFE_DAC_ADDER_EVEN_GET(reg);
		pr_info("[OC] dac_adder e/o = %02X", code);
		code =
		USBDP_TRSV_REG07CC_LN2_MON_RX_OC_DFE_DAC_ADDER_ODD_GET(reg);
		pr_info("/%02X\n", code);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG07CD);
		code =
		USBDP_TRSV_REG07CD_LN2_MON_RX_OC_DFE_SA_EDGE_EVEN_GET(reg);
		pr_info("[OC] sa_edge e     = %02X\n", code);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG07CE);
		code =
		USBDP_TRSV_REG07CE_LN2_MON_RX_OC_DFE_SA_EDGE_ODD_GET(reg);
		pr_info("[OC] sa_edge o     = %02X\n", code);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG07CF);
		code =
		USBDP_TRSV_REG07CF_LN2_MON_RX_OC_DFE_DAC_EDGE_ODD_GET(reg);
		pr_info("[OC] dac_edge o/e  = %02X", code);
		code =
		USBDP_TRSV_REG07CF_LN2_MON_RX_OC_DFE_DAC_EDGE_EVEN_GET(reg);
		pr_info("/%02X\n", code);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG07D0);
		code =
		USBDP_TRSV_REG07D0_LN2_MON_RX_OC_DFE_SA_ERR_EVEN_GET(reg);
		pr_info("[OC] sa_err e      = %02X\n", code);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG07D1);
		code =
		USBDP_TRSV_REG07D1_LN2_MON_RX_OC_DFE_SA_ERR_ODD_GET(reg);
		pr_info("[OC] sa_err o      = %02X\n", code);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG07D2);
		code =
		USBDP_TRSV_REG07D2_LN2_MON_RX_OC_DFE_DAC_ERR_EVEN_GET(reg);
		pr_info("[OC] dac_err e/o   = %02X", code);
		code =
		USBDP_TRSV_REG07D2_LN2_MON_RX_OC_DFE_DAC_ERR_ODD_GET(reg);
		pr_info("/%02X\n", code);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG07D3);
		code =
		USBDP_TRSV_REG07D3_LN2_MON_RX_OC_CTLE_GET(reg);
		pr_info("[OC] ctle          = %02X\n", code);

		/* 0x1F5C  ln2_mon_rx_oc_fail__7_0 */
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG07D7);
		code =
		USBDP_TRSV_REG07D7_LN2_MON_RX_OC_FAIL__7_0_GET(reg);
		pr_info("[OC] oc_fail       = %02X\n", code);

		reg = readl(regs_base + EXYNOS_USBDP_CMN_REG01D2);
		code =
		USBDP_CMN_REG01D2_LN2_MON_RX_OC_INIT_VGA_CODE_GET(reg);
		pr_info("[OC] oc_vga        = %02X\n", code);
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

int phy_exynos_usbdp_g2_v4_enable(struct exynos_usbphy_info *info)
{
	int ret = 0;
	struct reg_set *reg_base =
		(struct reg_set *)info->pma_base;

	phy_exynos_usbdp_g2_v4_ctrl_pma_ready(info);
	phy_exynos_usbdp_g2_v4_aux_force_off(info);
	phy_exynos_usbdp_g2_v4_pma_default_sfr_update(info);
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

	pr_info("reg000:%p, reg0001:%p\n",
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
