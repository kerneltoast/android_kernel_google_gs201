// SPDX-License-Identifier: GPL-2.0
/*
 * Samsung EXYNOS SoC series USB DRD PHY driver
 *
 * Copyright (C) 2022 Samsung Electronics Co., Ltd.
 */

#include <linux/types.h>
#include <linux/delay.h>
#include <linux/io.h>

#include "phy-samsung-usb-cal.h"

#include "eusb-con-reg.h"
#include "eusb-phy-reg.h"

void phy_exynos_eusb_tune(struct exynos_usbphy_info *info)
{
	void *base;
	u32 phy_cfg_tx, phy_cfg_rx, cnt;

	if (!info->tune_param) {
		return;
	}

	/* eusb phy tuning */
	base = info->regs_base;

	phy_cfg_tx = readl(base + EUSBCON_REG_TXTUNE);
	phy_cfg_rx = readl(base + EUSBCON_REG_RXTUNE);

	cnt = 0;
	for (; info->tune_param[cnt].value != EXYNOS_USB_TUNE_LAST; cnt++) {
		char *para_name;
		int val;

		val = info->tune_param[cnt].value;
		if (val == -1) {
			continue;
		}
		para_name = info->tune_param[cnt].name;
		if (!strcmp(para_name, "tx_fsls_slew_rate")) {
			((EUSBPHY_REG_TXTUNE_p) (&phy_cfg_tx))->b.fsls_slew_rate = val;
		} else if (!strcmp(para_name, "tx_fsls_vref_tune")) {
			((EUSBPHY_REG_TXTUNE_p) (&phy_cfg_tx))->b.fsls_vref_tune = val;
		} else if (!strcmp(para_name, "tx_fsls_vreg_bypass")) {
			((EUSBPHY_REG_TXTUNE_p) (&phy_cfg_tx))->b.fsls_vreg_bypass = val;
		} else if (!strcmp(para_name, "tx_hs_vref_tune")) {
			((EUSBPHY_REG_TXTUNE_p) (&phy_cfg_tx))->b.hs_vref_tune = val;
		} else if (!strcmp(para_name, "tx_hs_xv")) {
			((EUSBPHY_REG_TXTUNE_p) (&phy_cfg_tx))->b.hs_xv = val;
		} else if (!strcmp(para_name, "tx_preemp")) {
			((EUSBPHY_REG_TXTUNE_p) (&phy_cfg_tx))->b.preemp = val;
		} else if (!strcmp(para_name, "tx_res")) {
			((EUSBPHY_REG_TXTUNE_p) (&phy_cfg_tx))->b.res = val;
		} else if (!strcmp(para_name, "tx_rise")) {
			((EUSBPHY_REG_TXTUNE_p) (&phy_cfg_tx))->b.rise = val;
		} else if (!strcmp(para_name, "rx_eq_ctle")) {
			((EUSBPHY_REG_RXTUNE_p) (&phy_cfg_rx))->b.eq_ctle = val;
		} else if (!strcmp(para_name, "rx_hs_term_en")) {
			((EUSBPHY_REG_RXTUNE_p) (&phy_cfg_rx))->b.hs_term_en = val;
		} else if (!strcmp(para_name, "rx_hs_tune")) {
			((EUSBPHY_REG_RXTUNE_p) (&phy_cfg_rx))->b.hs_tune = val;
		} else if (!strcmp(para_name, "reg_direct") && (val != -1)) {
			phy_cfg_tx = val & 0xffff;
			phy_cfg_rx = (val >> 16) & 0xffff;
			break;
		}
	}
	writel(phy_cfg_tx, base + EUSBCON_REG_TXTUNE);
	writel(phy_cfg_rx, base + EUSBCON_REG_RXTUNE);
}

void phy_exynos_eusb_reset(struct exynos_usbphy_info *info)
{
	u32 reg;
	void *base;

	base = info->regs_base;


	reg = readl(base + EUSBCON_REG_RST_CTRL);
	((EUSBPHY_REG_RST_CTRL_p) (&reg))->b.phy_reset = 1;
	((EUSBPHY_REG_RST_CTRL_p) (&reg))->b.phy_reset_ovrd_en = 1;
	writel(reg, base + EUSBCON_REG_RST_CTRL);

	reg = readl(base + EUSBCON_REG_CMN_CTRL);
	((EUSBPHY_REG_CMN_CTRL_p) (&reg))->b.phy_enable = 0;
	writel(reg, base + EUSBCON_REG_CMN_CTRL);


}

void phy_exynos_eusb_initiate(struct exynos_usbphy_info *info)
{
	u32 reg, value;
	void *base;

	base = info->regs_base;



	/* Refer to the eUSB PHY databook 4.1.2.1 */
	/* 1. Set the phy_reset signal to 1'b1 to hold the eUSB 2.0 PHY
	 * in reset  */
	reg = readl(base + EUSBCON_REG_RST_CTRL);
	((EUSBPHY_REG_RST_CTRL_p) (&reg))->b.phy_reset = 1;
	((EUSBPHY_REG_RST_CTRL_p) (&reg))->b.phy_reset_ovrd_en = 1;
	((EUSBPHY_REG_RST_CTRL_p) (&reg))->b.utmi_port_reset = 1;
	((EUSBPHY_REG_RST_CTRL_p) (&reg))->b.utmi_port_reset_ovrd_en = 1;
	writel(reg, base + EUSBCON_REG_RST_CTRL);

	/* 2. Set all strapping options as described in Chapter 3,
	 *    “Signal Descriptions”. Strapping signals include:
	 * ❑ ref_freq_sel[2:0]
	 * ❑ phy_cfg_rcal_bypass
	 * ❑ phy_cfg_rcal_code[3:0]
	 * ❑ phy_cfg_rptr_mode
	 * ❑ phy_cfg_rx_hs_term_en
	 * ❑ “Parameter Override Signals”
	 */

	/* phy_cfg_rptr_mode */
	reg = readl(base + EUSBCON_REG_CMN_CTRL);
	((EUSBPHY_REG_CMN_CTRL_p) (&reg))->b.phy_cfg_rptr_mode = 1;

	if ((info->refclk == USBPHY_REFCLK_DIFF_19_2MHZ) ||
			(info->refclk == USBPHY_REFCLK_EXT_19P2MHZ)) {
		/* Reference Clock Frequency	: 19.2MHz
		 * ref_freq_sel[2:0] 			: 0
		 * phy_cfg_pll_fb_div[11:0] 	: 368
		 * phy_cfg_pll_ref_div[3:0]		: 0
		 */
		if ((EXYNOS_USBCON_VER_MINOR(info->version) == 0) ||
			(EXYNOS_USBCON_VER_MINOR(info->version) == 1)) { // 4nm eusb phy
			/* ref_freq_sel */
			((EUSBPHY_REG_CMN_CTRL_p) (&reg))->b.ref_freq_sel = 0;
			writel(reg, base + EUSBCON_REG_CMN_CTRL);
			/* phy_cfg_pll_fb_div[11:0] */
			reg = readl(base + EUSBCON_REG_PLLCFG0);
			((EUSBPHY_REG_PLLCFG0_p) (&reg))->b.pll_fb_div = 368;
			writel(reg, base + EUSBCON_REG_PLLCFG0);
			/* phy_cfg_pll_ref_div[3:0] */
			reg = readl(base + EUSBCON_REG_PLLCFG1);
			((EUSBPHY_REG_PLLCFG1_p) (&reg))->b.pll_ref_div = 0;
			writel(reg, base + EUSBCON_REG_PLLCFG1);
		} else if (EXYNOS_USBCON_VER_MINOR(info->version) == 2) { // 3nm eusb phy
			/* ref_freq_sel */
			((EUSBPHY_REG_CMN_CTRL_p) (&reg))->b.ref_freq_sel = 0;
			writel(reg, base + EUSBCON_REG_CMN_CTRL);
			/* phy_cfg_pll_fb_div[11:0] */
			reg = readl(base + EUSBCON_REG_PLLCFG0);
			((EUSBPHY_REG_PLLCFG0_p) (&reg))->b.pll_fb_div = 368;
			((EUSBPHY_REG_PLLCFG0_p) (&reg))->b.pll_int_cntrl = 4;
			((EUSBPHY_REG_PLLCFG0_p) (&reg))->b.pll_gmp_cntrl = 2;
			writel(reg, base + EUSBCON_REG_PLLCFG0);
			/* phy_cfg_pll_ref_div[3:0]	*/
			reg = readl(base + EUSBCON_REG_PLLCFG1);
			((EUSBPHY_REG_PLLCFG1_p) (&reg))->b.pll_ref_div = 0;
			((EUSBPHY_REG_PLLCFG1_p) (&reg))->b.pll_vref_tune = 0;
			((EUSBPHY_REG_PLLCFG1_p) (&reg))->b.pll_prop_cntrl = 8;
			writel(reg, base + EUSBCON_REG_PLLCFG1);
		}
		pr_info("%s: 19.2\n", __func__);
	} else if ((info->refclk == USBPHY_REFCLK_DIFF_20MHZ) ||
			(info->refclk == USBPHY_REFCLK_EXT_20MHZ)) {
		/* Reference Clock Frequency	: 20MHz
		 * ref_freq_sel[2:0] 			: 1
		 * phy_cfg_pll_fb_div[11:0] 	: 352
		 * phy_cfg_pll_ref_div[3:0]		: 0
		 */
		/* ref_freq_sel */
		((EUSBPHY_REG_CMN_CTRL_p) (&reg))->b.ref_freq_sel = 1;
		writel(reg, base + EUSBCON_REG_CMN_CTRL);
		/* phy_cfg_pll_fb_div[11:0] */
		reg = readl(base + EUSBCON_REG_PLLCFG0);
		((EUSBPHY_REG_PLLCFG0_p) (&reg))->b.pll_fb_div = 352;
		writel(reg, base + EUSBCON_REG_PLLCFG0);
		/* phy_cfg_pll_ref_div[3:0]	*/
		reg = readl(base + EUSBCON_REG_PLLCFG1);
		((EUSBPHY_REG_PLLCFG1_p) (&reg))->b.pll_ref_div = 0;
		writel(reg, base + EUSBCON_REG_PLLCFG1);
		pr_info("%s: 20\n", __func__);
	} else if ((info->refclk == USBPHY_REFCLK_DIFF_24MHZ) ||
			(info->refclk == USBPHY_REFCLK_EXT_24MHZ)) {
		/* Reference Clock Frequency	: 24MHz
		 * ref_freq_sel[2:0] 			: 2
		 * phy_cfg_pll_fb_div[11:0] 	: 288
		 * phy_cfg_pll_ref_div[3:0]		: 0
		 */
		/* ref_freq_sel */
		((EUSBPHY_REG_CMN_CTRL_p) (&reg))->b.ref_freq_sel = 2;
		writel(reg, base + EUSBCON_REG_CMN_CTRL);
		/* phy_cfg_pll_fb_div[11:0] */
		reg = readl(base + EUSBCON_REG_PLLCFG0);
		((EUSBPHY_REG_PLLCFG0_p) (&reg))->b.pll_fb_div = 288;
		writel(reg, base + EUSBCON_REG_PLLCFG0);
		/* phy_cfg_pll_ref_div[3:0]	*/
		reg = readl(base + EUSBCON_REG_PLLCFG1);
		((EUSBPHY_REG_PLLCFG1_p) (&reg))->b.pll_ref_div = 0;
		writel(reg, base + EUSBCON_REG_PLLCFG1);
		pr_info("%s: 24\n", __func__);
	} else if ((info->refclk == USBPHY_REFCLK_DIFF_26MHZ) ||
			(info->refclk == USBPHY_REFCLK_EXT_26MHZ)) {
		/* Reference Clock Frequency	: 26MHz
		 * ref_freq_sel[2:0] 			: 3
		 * phy_cfg_pll_fb_div[11:0] 	: 263
		 * phy_cfg_pll_ref_div[3:0]		: 0
		 */
		if ((EXYNOS_USBCON_VER_MINOR(info->version) == 0) ||
			(EXYNOS_USBCON_VER_MINOR(info->version) == 1)) { // 4nm eusb phy
			/* ref_freq_sel */
			((EUSBPHY_REG_CMN_CTRL_p) (&reg))->b.ref_freq_sel = 3;
			writel(reg, base + EUSBCON_REG_CMN_CTRL);
			/* phy_cfg_pll_fb_div[11:0] */
			reg = readl(base + EUSBCON_REG_PLLCFG0);
			((EUSBPHY_REG_PLLCFG0_p) (&reg))->b.pll_fb_div = 263;
			writel(reg, base + EUSBCON_REG_PLLCFG0);
			/* phy_cfg_pll_ref_div[3:0] */
			reg = readl(base + EUSBCON_REG_PLLCFG1);
			((EUSBPHY_REG_PLLCFG1_p) (&reg))->b.pll_ref_div = 0;
			writel(reg, base + EUSBCON_REG_PLLCFG1);
		} else if (EXYNOS_USBCON_VER_MINOR(info->version) == 2) { // 3nm eusb phy
			/* ref_freq_sel */
			((EUSBPHY_REG_CMN_CTRL_p) (&reg))->b.ref_freq_sel = 3;
			writel(reg, base + EUSBCON_REG_CMN_CTRL);
			/* phy_cfg_pll_fb_div[11:0] */
			reg = readl(base + EUSBCON_REG_PLLCFG0);
			((EUSBPHY_REG_PLLCFG0_p) (&reg))->b.pll_fb_div = 263;
			((EUSBPHY_REG_PLLCFG0_p) (&reg))->b.pll_int_cntrl = 4;
			((EUSBPHY_REG_PLLCFG0_p) (&reg))->b.pll_gmp_cntrl = 2;
			writel(reg, base + EUSBCON_REG_PLLCFG0);
			/* phy_cfg_pll_ref_div[3:0]	*/
			reg = readl(base + EUSBCON_REG_PLLCFG1);
			((EUSBPHY_REG_PLLCFG1_p) (&reg))->b.pll_ref_div = 0;
			((EUSBPHY_REG_PLLCFG1_p) (&reg))->b.pll_vref_tune = 0;
			((EUSBPHY_REG_PLLCFG1_p) (&reg))->b.pll_prop_cntrl = 7;
			writel(reg, base + EUSBCON_REG_PLLCFG1);
		}
		pr_info("%s: 26\n", __func__);
	} else if ((info->refclk == USBPHY_REFCLK_DIFF_48MHZ) ||
			(info->refclk == USBPHY_REFCLK_EXT_48MHZ)) {
		/* Reference Clock Frequency	: 48MHz
		 * ref_freq_sel[2:0] 			: 2
		 * phy_cfg_pll_fb_div[11:0] 	: 288
		 * phy_cfg_pll_ref_div[3:0]		: 1
		 */
		/* ref_freq_sel */
		((EUSBPHY_REG_CMN_CTRL_p) (&reg))->b.ref_freq_sel = 2;
		writel(reg, base + EUSBCON_REG_CMN_CTRL);
		/* phy_cfg_pll_fb_div[11:0] */
		reg = readl(base + EUSBCON_REG_PLLCFG0);
		((EUSBPHY_REG_PLLCFG0_p) (&reg))->b.pll_fb_div = 288;
		writel(reg, base + EUSBCON_REG_PLLCFG0);
		/* phy_cfg_pll_ref_div[3:0]	*/
		reg = readl(base + EUSBCON_REG_PLLCFG1);
		((EUSBPHY_REG_PLLCFG1_p) (&reg))->b.pll_ref_div = 1;
		writel(reg, base + EUSBCON_REG_PLLCFG1);
	} else {
		/* Not supported clock, so skip */
	}

	if (EXYNOS_USBCON_VER_MINOR(info->version) == 0) {
		/* use default value
			26MHz : 3 - pll_cpbias_cntrl
			19.2MHz : 1 - pll_cpbias_cntrl
		*/
	} else if (EXYNOS_USBCON_VER_MINOR(info->version) == 1) {
		reg = readl(base + EUSBCON_REG_PLLCFG0);
		((EUSBPHY_REG_PLLCFG0_p) (&reg))->b.pll_cpbias_cntrl = 0;
		writel(reg, base + EUSBCON_REG_PLLCFG0);
	} else if (EXYNOS_USBCON_VER_MINOR(info->version) == 2) {
		reg = readl(base + EUSBCON_REG_PLLCFG0);
		((EUSBPHY_REG_PLLCFG0_p) (&reg))->b.pll_cpbias_cntrl = 0;
		writel(reg, base + EUSBCON_REG_PLLCFG0);
	}

	reg = readl(base + EUSBCON_REG_TXTUNE);
	((EUSBPHY_REG_TXTUNE_p) (&reg))->b.fsls_vref_tune = 0;
	writel(reg, base + EUSBCON_REG_TXTUNE);
	phy_exynos_eusb_tune(info);

	/* 3. Set all inputs to a default state as necessary for
	 *    the eUSB 2.0 PHY application.
	 */
	reg = readl(base + EUSBCON_REG_TESTSE);
	((EUSBPHY_REG_TESTSE_p) (&reg))->b.test_iddq = 0;
	writel(reg, base + EUSBCON_REG_TESTSE);

	reg = readl(base + EUSBCON_REG_CMN_CTRL);
	((EUSBPHY_REG_CMN_CTRL_p) (&reg))->b.phy_enable = 0;
	writel(reg, base + EUSBCON_REG_CMN_CTRL);

	/* 5. If some registers programming is needed before
	 *    power-up sequence starts, keep phy_enable at 1’b0,
	 *    otherwise keep phy_enable at 1’b1 and skip step 7.
	 */

	/* After releasing test_iddq to 1'b0,
	 * phy_reset should be held at 1'b1 for at least an additional 10us.
	 */
	udelay(10);

	/* 6. Release phy_reset from 1'b1 to 1'b0 */
	reg = readl(base + EUSBCON_REG_RST_CTRL);
	((EUSBPHY_REG_RST_CTRL_p) (&reg))->b.phy_reset = 0;
	writel(reg, base + EUSBCON_REG_RST_CTRL);

	/* additional delay for REXT calibration */
	udelay(10);

	/* 7. Perform all necessary registers programming
	 * (configuration settings) and then set phy_enable at
	 * 1’b1.
	 */

	reg = readl(base + EUSBCON_REG_CMN_CTRL);
	((EUSBPHY_REG_CMN_CTRL_p) (&reg))->b.phy_enable = 1;
	writel(reg, base + EUSBCON_REG_CMN_CTRL);

	/* additional delay for REXT calibration */
	value = 1000;
	udelay(value);

	/* T4 indicates when the PHY analog and digital circuit has powered up and
	 * is ready to perform an eUSB protocol Port Reset (ESE1). The utmi_clk
	 * output clock starts toggling.
	 */
	udelay(28);

	/* T5 indicates when the PHY (eDSPr/eDSPn/eUSPr) has completed the
	 * transmission of Port Reset (ESE1) on eUSB lines.
	 */
	udelay(2500);

	reg = readl(base + EUSBCON_REG_RST_CTRL);
	((EUSBPHY_REG_RST_CTRL_p) (&reg))->b.utmi_port_reset = 0;
	((EUSBPHY_REG_RST_CTRL_p) (&reg))->b.utmi_port_reset_ovrd_en = 0;
	writel(reg, base + EUSBCON_REG_RST_CTRL);


}

void phy_exynos_eusb_terminate(struct exynos_usbphy_info *info)
{
	u32 reg;
	void *base;

	base = info->regs_base;

	udelay(2500);
	reg = readl(base + EUSBCON_REG_CMN_CTRL);
	((EUSBPHY_REG_CMN_CTRL_p) (&reg))->b.phy_enable = 0;
	writel(reg, base + EUSBCON_REG_CMN_CTRL);
	reg = readl(base + EUSBCON_REG_RST_CTRL);
	((EUSBPHY_REG_RST_CTRL_p) (&reg))->b.phy_reset = 1;
	writel(reg, base + EUSBCON_REG_RST_CTRL);
	reg = readl(base + EUSBCON_REG_TESTSE);
	((EUSBPHY_REG_TESTSE_p) (&reg))->b.test_iddq = 1;
	writel(reg, base + EUSBCON_REG_TESTSE);
}

u8 phy_exynos_eusb_get_eusb_state(struct exynos_usbphy_info *info)
{
	EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_EUSB_STATE_JMP_DBG_OUT_o reg;
	void *base;

	base = info->regs_base_2nd;
	reg.data = readl(base + EUSBPHY_REG_PHY_CR_XCVR0_DIG_CTL_EUSB_STATE_JMP_DBG_OUT) & 0xffff;
	return reg.b.eusb_state_jmp_dbg_out;
}
