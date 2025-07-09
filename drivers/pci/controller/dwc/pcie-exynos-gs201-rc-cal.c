// SPDX-License-Identifier: GPL-2.0-only
/*
 * PCIe phy driver for gs201 SoC
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

	dev_info(exynos_pcie->pci->dev, "[CAL: %s]\n", __func__);
	writel(0x20, phy_base_regs + 0x408);
	writel(0x0A, phy_base_regs + 0x40C);

	writel(0x0A, phy_base_regs + 0x800);
	writel(0xBF, phy_base_regs + 0x804);
	writel(0x02, phy_base_regs + 0xA40);
	writel(0x2A, phy_base_regs + 0xA44);
	writel(0xAA, phy_base_regs + 0xA48);
	writel(0xA8, phy_base_regs + 0xA4C);
	writel(0x80, phy_base_regs + 0xA50);

	writel(0x0A, phy_base_regs + 0x1000);
	writel(0xBF, phy_base_regs + 0x1004);
	writel(0x02, phy_base_regs + 0x1240);
	writel(0x2A, phy_base_regs + 0x1244);
	writel(0xAA, phy_base_regs + 0x1248);
	writel(0xA8, phy_base_regs + 0x124C);
	writel(0x80, phy_base_regs + 0x1250);
}

/* PHY all power down clear */
void exynos_pcie_rc_phy_all_pwrdn_clear(struct exynos_pcie *exynos_pcie, int ch_num)
{
	void __iomem *phy_base_regs = exynos_pcie->phy_base;

	dev_info(exynos_pcie->pci->dev, "[CAL: %s]\n", __func__);
	writel(0x28, phy_base_regs + 0xD8);
	mdelay(1);

	writel(0x00, phy_base_regs + 0x408);
	writel(0x00, phy_base_regs + 0x40C);

	writel(0x00, phy_base_regs + 0x800);
	writel(0x00, phy_base_regs + 0x804);
	writel(0x00, phy_base_regs + 0xA40);
	writel(0x00, phy_base_regs + 0xA44);
	writel(0x00, phy_base_regs + 0xA48);
	writel(0x00, phy_base_regs + 0xA4C);
	writel(0x00, phy_base_regs + 0xA50);

	writel(0x00, phy_base_regs + 0x1000);
	writel(0x00, phy_base_regs + 0x1004);
	writel(0x00, phy_base_regs + 0x1240);
	writel(0x00, phy_base_regs + 0x1244);
	writel(0x00, phy_base_regs + 0x1248);
	writel(0x00, phy_base_regs + 0x124C);
	writel(0x00, phy_base_regs + 0x1250);
}

#if IS_ENABLED(CONFIG_EXYNOS_OTP)
void exynos_pcie_rc_pcie_phy_otp_config(void *phy_base_regs, int ch_num)
{
	/* To be updated */
}
#endif

#define LCPLL_REF_CLK_SEL	(0x3 << 4)

void exynos_pcie_rc_pcie_phy_config(struct exynos_pcie *exynos_pcie, int ch_num)
{
	void __iomem *elbi_base_regs = exynos_pcie->elbi_base;
	void __iomem *phy_base_regs = exynos_pcie->phy_base;
	void __iomem *phy_pcs_base_regs = exynos_pcie->phy_pcs_base;
	int num_lanes = exynos_pcie->num_lanes;
	u32 val;
	u32 i;

	dev_info(exynos_pcie->pci->dev, "[CAL: %s] CAL ver 210802\n", __func__);

	/* init. input clk path */
	writel(0x28, phy_base_regs + 0xD8);

	/* PCS MUX glitch W/A */
	val = readl(exynos_pcie->phy_pcs_base + 0x008);
	val |= (1 << 7);
	writel(val, phy_pcs_base_regs + 0x008);
	if (num_lanes == 2) {
		val = readl(exynos_pcie->phy_pcs_base + 0x808);
		val |= (1 << 7);
		writel(val, phy_pcs_base_regs + 0x808);
	}

	/* PHY CMN_RST, INIT_RST, PORT_RST Assert */
	writel(0x1, elbi_base_regs + 0x1404);
	writel(0x1, elbi_base_regs + 0x1408);
	writel(0x1, elbi_base_regs + 0x1400);
	writel(0x0, elbi_base_regs + 0x1404);
	writel(0x0, elbi_base_regs + 0x1408);
	writel(0x0, elbi_base_regs + 0x1400);
	udelay(10);

	writel(0x1, elbi_base_regs + 0x1404);
	udelay(10);

	/* pma_setting */
	/* Common */
	writel(0x50, phy_base_regs + 0x018);
	writel(0x33, phy_base_regs + 0x048);
	writel(0x01, phy_base_regs + 0x068);
	writel(0x12, phy_base_regs + 0x070);
	writel(0x00, phy_base_regs + 0x08C);
	writel(0x21, phy_base_regs + 0x090);
	writel(0x14, phy_base_regs + 0x0B0);
	writel(0x50, phy_base_regs + 0x0B8);
	writel(0x51, phy_base_regs + 0x0E0);
	writel(0x00, phy_base_regs + 0x100);
	writel(0x80, phy_base_regs + 0x104);
	writel(0x38, phy_base_regs + 0x140);
	writel(0xA4, phy_base_regs + 0x180);
	writel(0x03, phy_base_regs + 0x188); /* LCPLL 100MHz no divide */
	writel(0x38, phy_base_regs + 0x2A8);
	writel(0x12, phy_base_regs + 0x2E4);
	/* PMA enable and aggregation mode(bifurcation mode: 0xC0) */
	writel(0x80, phy_base_regs + 0x400);
	writel(0x20, phy_base_regs + 0x408);
	writel(0x00, phy_base_regs + 0x550);
	writel(0x00, phy_base_regs + 0x5A8);
	writel(0xFF, phy_base_regs + 0x5EC);

	/* PCIe_TYPE: RC */
	writel(0x02, phy_base_regs + 0x458); /* 100MHz CLK on */
	writel(0x34, phy_base_regs + 0x5B0); /* diff,control REFCLK source */
	writel(0x20, phy_base_regs + 0x450); /* when entering L1.2, add delay */

	/* PHY INPUT CLK 38.4 */
	writel(0x34, phy_base_regs + 0xAC);
	writel(0x34, phy_base_regs + 0xB4);
	writel(0x50, phy_base_regs + 0xE0);
	writel(0x00, phy_base_regs + 0xF4);
	writel(0x08, phy_base_regs + 0xF8);
	writel(0xA0, phy_base_regs + 0x104);
	writel(0x19, phy_base_regs + 0x11C);
	writel(0x17, phy_base_regs + 0x124);
	writel(0x41, phy_base_regs + 0x220);

	for (i = 0; i < num_lanes; i++) {
		phy_base_regs += (i * 0x800);

		writel(0x08, phy_base_regs + 0x82C);
		writel(0x24, phy_base_regs + 0x830);
		writel(0x80, phy_base_regs + 0x878);
		writel(0x40, phy_base_regs + 0x894);
		writel(0x00, phy_base_regs + 0x8C0);
		writel(0x30, phy_base_regs + 0x8F4);
		writel(0x05, phy_base_regs + 0x908);
		writel(0xE0, phy_base_regs + 0x90C);
		writel(0xD4, phy_base_regs + 0x914);
		writel(0xD3, phy_base_regs + 0x91C);
		writel(0xCE, phy_base_regs + 0x920);
		writel(0x01, phy_base_regs + 0x924);
		writel(0x35, phy_base_regs + 0x928);
		writel(0xBA, phy_base_regs + 0x92C);
		writel(0x41, phy_base_regs + 0x930);
		writel(0x15, phy_base_regs + 0x934);
		writel(0x13, phy_base_regs + 0x938);
		writel(0x4E, phy_base_regs + 0x93C);
		writel(0x43, phy_base_regs + 0x948);
		writel(0xFC, phy_base_regs + 0x94C);
		writel(0x10, phy_base_regs + 0x954);
		writel(0x69, phy_base_regs + 0x958);
		writel(0x40, phy_base_regs + 0x964);
		writel(0xF6, phy_base_regs + 0x9B4);
		writel(0x2D, phy_base_regs + 0x9C0);
		writel(0xB7, phy_base_regs + 0x9C4);
		writel(0x3C, phy_base_regs + 0x9CC);
		writel(0x7E, phy_base_regs + 0x9DC);
		writel(0x02, phy_base_regs + 0xA40);
		writel(0x26, phy_base_regs + 0xA70);
		writel(0x00, phy_base_regs + 0xA74);
		writel(0x06, phy_base_regs + 0xB40);	/* hf_init */
		writel(0x06, phy_base_regs + 0xB44);
		writel(0x04, phy_base_regs + 0xB48);	/* value changed: 0x06 -> 0x04 */
		writel(0x03, phy_base_regs + 0xB4C);	/* value changed: 0x04 -> 0x03 */
		writel(0x03, phy_base_regs + 0xB50);	/* mf_init */
		writel(0x03, phy_base_regs + 0xB54);
		writel(0x03, phy_base_regs + 0xB58);
		writel(0x03, phy_base_regs + 0xB5C);
		writel(0x1B, phy_base_regs + 0xC10);
		writel(0x10, phy_base_regs + 0xC44);
		writel(0x10, phy_base_regs + 0xC48);
		writel(0x10, phy_base_regs + 0xC4C);
		writel(0x10, phy_base_regs + 0xC50);
		writel(0x02, phy_base_regs + 0xC54);
		writel(0x02, phy_base_regs + 0xC58);
		writel(0x02, phy_base_regs + 0xC5C);
		writel(0x02, phy_base_regs + 0xC60);
		writel(0x02, phy_base_regs + 0xC6C);
		writel(0x02, phy_base_regs + 0xC70);
		writel(0xE7, phy_base_regs + 0xCA8);
		writel(0x00, phy_base_regs + 0xCAC);
		writel(0x0E, phy_base_regs + 0xCB0);
		writel(0x1C, phy_base_regs + 0xCCC);
		writel(0x05, phy_base_regs + 0xCD4);
		writel(0x77, phy_base_regs + 0xCD8);
		writel(0x7A, phy_base_regs + 0xCDC);
		writel(0x2F, phy_base_regs + 0xDB4);

		/* RX tuning */
		writel(0x80, phy_base_regs + 0x8FC);
		writel(0xF4, phy_base_regs + 0x914);
		writel(0xD3, phy_base_regs + 0x91C);
		writel(0xCA, phy_base_regs + 0x920);
		writel(0x3D, phy_base_regs + 0x928);
		writel(0xB8, phy_base_regs + 0x92C);
		writel(0x41, phy_base_regs + 0x930);
		writel(0x17, phy_base_regs + 0x934);
		writel(0x4C, phy_base_regs + 0x93C);
		writel(0x73, phy_base_regs + 0x948);
		writel(0xFC, phy_base_regs + 0x94C);
		writel(0x55, phy_base_regs + 0x96C);
		writel(0x78, phy_base_regs + 0x988);
		writel(0x3B, phy_base_regs + 0x994);
		writel(0xF6, phy_base_regs + 0x9B4);
		writel(0xFF, phy_base_regs + 0x9C4);
		writel(0x20, phy_base_regs + 0x9C8);
		writel(0x2F, phy_base_regs + 0xA08);
		writel(0x3F, phy_base_regs + 0xB9C);

		/* for GEN4 TX termination resistor? */
		writel(0x00, phy_base_regs + 0x9CC);
		writel(0xFF, phy_base_regs + 0xCD8);
		writel(0x6E, phy_base_regs + 0xCDC);

		/* TX idrv for termination resistor */
		writel(0x0F, phy_base_regs + 0x82C);
		writel(0x60, phy_base_regs + 0x830);
		writel(0x7E, phy_base_regs + 0x834);

		/* Auto FBB */
		writel(0x00, phy_base_regs + 0xC08);
		writel(0x09, phy_base_regs + 0xC10);
		writel(0x04, phy_base_regs + 0xC40);
		writel(0x00, phy_base_regs + 0xC70);
		/* DFE */
		writel(0x76, phy_base_regs + 0x9B4);
	}

	phy_base_regs = exynos_pcie->phy_base;
	for (i = 0; i < 2; i++) {
		phy_base_regs += (i * 0x800);

		/* PHY INPUT CLK 38.4 */
		writel(0x41, phy_base_regs + 0xBCC);
		writel(0x41, phy_base_regs + 0xBD4);
		writel(0x68, phy_base_regs + 0xBDC);
		writel(0x00, phy_base_regs + 0xBE0);
		writel(0xD0, phy_base_regs + 0xBE4);
	}

	/* PCS setting: pcie_pcs_setting() in F/W code */

	/* aggregation mdoe(bifurcation disabled) */
	writel(0x00, phy_pcs_base_regs + 0x004);
	if (num_lanes == 2)
		writel(0x00, phy_pcs_base_regs + 0x804);

	/* if RC */
	writel(0x700D5, phy_pcs_base_regs + 0x154);
	if (num_lanes == 2)
		writel(0x700D5, phy_pcs_base_regs + 0x954);

	/* add for L2 entry and power */
	writel(0x300FF, phy_pcs_base_regs + 0x150);
	if (num_lanes == 2)
		writel(0x300FF, phy_pcs_base_regs + 0x950);

	/* add L2 power down delay */
	writel(0x40, phy_pcs_base_regs + 0x170);
	if (num_lanes == 2)
		writel(0x40, phy_pcs_base_regs + 0x970);

	/* when entring L1.2, ERIO CLK gating */
	val = readl(exynos_pcie->phy_pcs_base + 0x008);
	val &= ~((1 << 4) | (1 << 5));
	val |= (1 << 4);
	writel(val, phy_pcs_base_regs + 0x008);
	if (num_lanes == 2) {
		val = readl(exynos_pcie->phy_pcs_base + 0x808);
		val &= ~((1 << 4) | (1 << 5));
		val |= (1 << 4);
		writel(val, phy_pcs_base_regs + 0x808);
	}

	/* PLL & BIAS EN off delay	*/
	writel(0x100B0808, phy_pcs_base_regs + 0x190);

	/* PHY CMN_RST, PORT_RST Release */
	writel(0x1, elbi_base_regs + 0x1400);
	writel(0x1, elbi_base_regs + 0x1408);

	/* Additional PMA Configurations */
	phy_base_regs = exynos_pcie->phy_base;
	val = readl(phy_base_regs + 0x5D0);
	val |= (0x1 << 4);
	val &= ~(0x1 << 3);
	writel(val, phy_base_regs + 0x5D0);
	dev_info(exynos_pcie->pci->dev, "[%s] XO clock configuration : 0x%x\n",
			__func__, readl(phy_base_regs + 0x5D0));
}
EXPORT_SYMBOL_GPL(exynos_pcie_rc_pcie_phy_config);

int exynos_pcie_rc_eom(struct device *dev, void *phy_base_regs)
{
	struct exynos_pcie *exynos_pcie = dev_get_drvdata(dev);
	struct exynos_pcie_ops *pcie_ops = &exynos_pcie->exynos_pcie_ops;
	struct dw_pcie *pci = exynos_pcie->pci;
	struct dw_pcie_rp *pp = &pci->pp;
	struct device_node *np = dev->of_node;
	unsigned int val;
	unsigned int speed_rate, num_of_smpl;
	unsigned int lane_width = 1;
	int i, ret;
	int test_cnt = 0;
	struct pcie_eom_result **eom_result;

	u32 phase_sweep = 0;
	u32 phase_step = 1;
	u32 phase_loop = 1;
	u32 vref_sweep = 0;
	u32 vref_step = 1;
	u32 err_cnt = 0;
	u32 cdr_value = 0;
	u32 eom_done = 0;
	u32 err_cnt_13_8;
	u32 err_cnt_7_0;

	dev_info(dev, "[%s] START!\n", __func__);

	ret = of_property_read_u32(np, "num-lanes", &lane_width);
	if (ret) {
		dev_err(dev, "[%s] failed to get num of lane(lane width=0\n", __func__);
		lane_width = 0;
	} else {
		dev_info(dev, "[%s] num-lanes : %d\n", __func__, lane_width);
	}

	/* eom_result[lane_num][test_cnt] */
	eom_result = kcalloc(1, sizeof(struct pcie_eom_result *) * lane_width, GFP_KERNEL);
	for (i = 0; i < lane_width; i++) {
		eom_result[i] = devm_kzalloc(dev, sizeof(*eom_result[i]) *
				EOM_PH_SEL_MAX * EOM_DEF_VREF_MAX, GFP_KERNEL);
	}
	for (i = 0; i < lane_width; i++) {
		if (!eom_result[i]) {
			dev_err(dev, "[%s] failed to alloc 'eom_result[%d]\n",
				       __func__, i);
			return -ENOMEM;
		}
	}

	exynos_pcie->eom_result = eom_result;

	pcie_ops->rd_own_conf(pp, PCIE_LINK_CTRL_STAT, 4, &val);
	speed_rate = (val >> 16) & 0xf;

	if (speed_rate == 1 || speed_rate == 2) {
		dev_err(dev, "[%s] speed_rate(GEN%d) is not GEN3 or GEN4\n", __func__, speed_rate);
		/* memory free 'eom_result' */
		for (i = 0; i < lane_width; i++)
			devm_kfree(dev, eom_result[i]);

		return -EINVAL;
	}

	num_of_smpl = 13;

	for (i = 0; i < lane_width; i++) {
		writel(0xE7, phy_base_regs + RX_EFOM_BIT_WIDTH_SEL);

		val = readl(phy_base_regs + ANA_RX_DFE_EOM_PI_STR_CTRL);
		val |= 0xF;
		writel(val, phy_base_regs + ANA_RX_DFE_EOM_PI_STR_CTRL);

		val = readl(phy_base_regs + ANA_RX_DFE_EOM_PI_DIVSEL_G12);
		val |= (0x4 | 0x10);
		writel(val, phy_base_regs + ANA_RX_DFE_EOM_PI_DIVSEL_G12);

		val = readl(phy_base_regs + ANA_RX_DFE_EOM_PI_DIVSEL_G34);
		val |= (0x4 | 0x20);	/* target sfr  changed: ANA_RC_...DIVSEL_G32 -> G34 */
		writel(val, phy_base_regs + ANA_RX_DFE_EOM_PI_DIVSEL_G34);

		val = readl(phy_base_regs + RX_CDR_LOCK) >> 2;
		cdr_value = val & 0x1;
		eom_done = readl(phy_base_regs + RX_EFOM_DONE) & 0x1;
		dev_info(dev, "eom_done 0x%x , cdr_value : 0x%x\n", eom_done, cdr_value);

		writel(0x0, phy_base_regs + RX_EFOM_NUMOF_SMPL_13_8);
		writel(num_of_smpl, phy_base_regs + RX_EFOM_NUMOF_SMPL_7_0);

		for (phase_sweep = 0; phase_sweep <= 0x47 * phase_loop;
				phase_sweep = phase_sweep + phase_step) {
			val = (phase_sweep % 72) << 1;
			writel(val, phy_base_regs + RX_EFOM_EOM_PH_SEL);

			for (vref_sweep = 0; vref_sweep <= 255;
			     vref_sweep = vref_sweep + vref_step) {
				/* malfunction code: writel(0x12, phy_base_regs + RX_EFOM_MODE); */
				val = readl(phy_base_regs + RX_EFOM_MODE);
				val &= ~(0x1f);
				val |= (0x2 | 0x10);
				writel(val, phy_base_regs + RX_EFOM_MODE);

				writel(vref_sweep, phy_base_regs + RX_EFOM_DFE_VREF_CTRL);

				/* malfunction code:  writel(0x13, phy_base_regs + RX_EFOM_MODE); */
				val = readl(phy_base_regs + RX_EFOM_MODE);
				val |= 0x1;	/* value changed: 0x13 -> 0x1 */
				writel(val, phy_base_regs + RX_EFOM_MODE);

				val = readl(phy_base_regs + RX_EFOM_DONE) & 0x1;
				while (val != 0x1) {
					udelay(1);
					val = readl(phy_base_regs +
							RX_EFOM_DONE) & 0x1;
				}

				err_cnt_13_8 = readl(phy_base_regs +
						MON_RX_EFOM_ERR_CNT_13_8) << 8;
				err_cnt_7_0 = readl(phy_base_regs +
						MON_RX_EFOM_ERR_CNT_7_0);
				err_cnt = err_cnt_13_8 + err_cnt_7_0;

				if (vref_sweep == 128)
					dev_info(dev, "%d,%d : %d %d %d\n", i, test_cnt,
						 phase_sweep, vref_sweep, err_cnt);

				/* save result */
				eom_result[i][test_cnt].phase = phase_sweep;
				eom_result[i][test_cnt].vref = vref_sweep;
				eom_result[i][test_cnt].err_cnt = err_cnt;

				test_cnt++;
			}
		}
		writel(0x21, phy_base_regs + 0xBA0); /* 0xBA0 */
		writel(0x00, phy_base_regs + 0xCA0); /* RX_EFOM_MODE = 0xCA0 */

		/* goto next lane */
		phy_base_regs += 0x800;
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
MODULE_DESCRIPTION("PCIe phy driver for gs201 SoC");
MODULE_LICENSE("GPL v2");
