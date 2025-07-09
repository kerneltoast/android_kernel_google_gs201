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

#include "exynos-usb-blkcon-sfr.h"

static void ready_rewa(struct exynos_usbphy_info *cal_info);

void exynos_usbcon_init_link(struct exynos_usbphy_info *cal_info)
{
	void __iomem *regs_base;
	u32 reg;

	regs_base = cal_info->regs_base;

	/* LINKCTRL
	 * 1. Disable q-channel
	 * 2. Bypass debounce filter for vbus, bvalid and id  */
	reg = readl(regs_base + USBCON_REG_LINKCTRL);
	((USBCON_REG_LINKCTRL_p) (&reg))->b.dis_id0_qact = 1;
	((USBCON_REG_LINKCTRL_p) (&reg))->b.dis_bvalid_qact = 1;
	((USBCON_REG_LINKCTRL_p) (&reg))->b.dis_vbusvalid_qact = 1;
	((USBCON_REG_LINKCTRL_p) (&reg))->b.dis_linkgate_qact = 1;
	((USBCON_REG_LINKCTRL_p) (&reg))->b.force_qact = 0;
	udelay(500);
	writel(reg, regs_base + USBCON_REG_LINKCTRL);
	udelay(500);
	((USBCON_REG_LINKCTRL_p) (&reg))->b.force_qact = 1;
	((USBCON_REG_LINKCTRL_p) (&reg))->b.bus_filter_bypass = 0xf;
	writel(reg, regs_base + USBCON_REG_LINKCTRL);

	/* Reset Link */
	reg = readl(regs_base + USBCON_REG_LINK_CLKRST);
	((USBCON_REG_LINK_CLKRST_p) (&reg))->b.link_sw_rst = 1;
	writel(reg, regs_base + USBCON_REG_LINK_CLKRST);
	udelay(10);
	((USBCON_REG_LINK_CLKRST_p) (&reg))->b.link_sw_rst = 0;
	writel(reg, regs_base + USBCON_REG_LINK_CLKRST);

	/* UTMI_CTRL
	 * 1. Set high bvalid and vbus valid  */
	reg = readl(regs_base + USBCON_REG_UTMI_CTRL);
	((USBCON_REG_UTMI_CTRL_p) (&reg))->b.force_bvalid = 1;
	((USBCON_REG_UTMI_CTRL_p) (&reg))->b.force_vbusvalid = 1;
	writel(reg, regs_base + USBCON_REG_UTMI_CTRL);

	/* Initilze UTMI ReWA */
	if (cal_info->hs_rewa)
		ready_rewa(cal_info);

}

void exynos_usbcon_dp_pullup_en(struct exynos_usbphy_info *cal_info, int en)
{
	void __iomem *regs_base;
	u32 reg;

	regs_base = cal_info->regs_base;

	reg = readl(regs_base + USBCON_REG_UTMI_CTRL);
	((USBCON_REG_UTMI_CTRL_p) (&reg))->b.force_bvalid = en;
	((USBCON_REG_UTMI_CTRL_p) (&reg))->b.force_vbusvalid = en;
	writel(reg, regs_base + USBCON_REG_UTMI_CTRL);
}

void exynos_usbcon_detach_pipe3_phy(struct exynos_usbphy_info *cal_info)
{
	void __iomem *regs_base;
	u32 reg;

	regs_base = cal_info->regs_base;

	/* force pipe3 signal for link */
	reg = readl(regs_base + USBCON_REG_LINKCTRL);
	((USBCON_REG_LINKCTRL_p) (&reg))->b.force_pipe_en = 1;
	((USBCON_REG_LINKCTRL_p) (&reg))->b.force_phystatus = 0;
	((USBCON_REG_LINKCTRL_p) (&reg))->b.force_rxelecidle = 1;
	writel(reg, regs_base + USBCON_REG_LINKCTRL);
	pr_debug("%s %d USBCON_REG_LINKCTRL: %#08x\n", __func__,
		__LINE__, readl(regs_base + USBCON_REG_LINKCTRL));

	/* pclk to suspend clock */
	reg = readl(regs_base + USBCON_REG_LINK_CLKRST);
	((USBCON_REG_LINK_CLKRST_p) (&reg))->b.link_pclk_sel = 0;
	writel(reg, regs_base + USBCON_REG_LINK_CLKRST);
}

void exynos_usbcon_disable_pipe3_phy(struct exynos_usbphy_info *cal_info)
{
	void __iomem *regs_base;
	u32 reg;

	exynos_usbcon_detach_pipe3_phy(cal_info);

	regs_base = cal_info->regs_base;

	/* calibrate only eUSB Phy */
	reg = readl(regs_base + USBCON_REG_HSP_MISC);
	((USBCON_REG_HSP_MISC_p) (&reg))->b.sel_res_tune_mux = 2;
	((USBCON_REG_HSP_MISC_p) (&reg))->b.set_req_in2 = 1;
	((USBCON_REG_HSP_MISC_p) (&reg))->b.set_ack_in2 = 0;
	((USBCON_REG_HSP_MISC_p) (&reg))->b.set_req_in1 = 0;
	((USBCON_REG_HSP_MISC_p) (&reg))->b.set_ack_in1 = 0;
	writel(reg, regs_base + USBCON_REG_HSP_MISC);
}

void exynos_usbcon_ready_to_pipe3_phy(struct exynos_usbphy_info *cal_info)
{
	void __iomem *regs_base;
	u32 reg;

	regs_base = cal_info->regs_base;

	/* Disable forcing pipe interface */
	reg = readl(regs_base + USBCON_REG_LINKCTRL);
	((USBCON_REG_LINKCTRL_p) (&reg))->b.force_pipe_en = 0;
	writel(reg, regs_base + USBCON_REG_LINKCTRL);

	/* calibrate Sequence: Dual Phy */
	reg = readl(regs_base + USBCON_REG_HSP_MISC);
	((USBCON_REG_HSP_MISC_p) (&reg))->b.sel_res_tune_mux = 1;
	((USBCON_REG_HSP_MISC_p) (&reg))->b.set_req_in2 = 0;
	((USBCON_REG_HSP_MISC_p) (&reg))->b.set_ack_in2 = 0;
	((USBCON_REG_HSP_MISC_p) (&reg))->b.set_req_in1 = 0;
	((USBCON_REG_HSP_MISC_p) (&reg))->b.set_ack_in1 = 0;
	writel(reg, regs_base + USBCON_REG_HSP_MISC);

	/* Pclk to pipe_clk */
	reg = readl(regs_base + USBCON_REG_LINK_CLKRST);
	((USBCON_REG_LINK_CLKRST_p) (&reg))->b.link_pclk_sel = 1;
	writel(reg, regs_base + USBCON_REG_LINK_CLKRST);
}

u64 exynos_usbcon_get_logic_trace(struct exynos_usbphy_info *cal_info)
{
	void __iomem *regs_base;
	u64 ret;

	regs_base = cal_info->regs_base;

	ret = readl(regs_base + USBCON_REG_LINK_DEBUG_L);
	ret |= ((u64) readl(regs_base + USBCON_REG_LINK_DEBUG_H)) << 32;

	return ret;
}

void exynos_usbcon_set_fsv_out_en(struct exynos_usbphy_info *cal_info, u32 en)
{
	void __iomem *regs_base = cal_info->regs_base;
	u32 reg;

	reg = readl(regs_base + USBCON_REG_HSP_MISC);
	if (en) {
		((USBCON_REG_HSP_MISC_p) (&reg))->b.fsvp_out_en = 1;
		((USBCON_REG_HSP_MISC_p) (&reg))->b.fsvm_out_en = 1;
	} else {
		((USBCON_REG_HSP_MISC_p) (&reg))->b.fsvp_out_en = 0;
		((USBCON_REG_HSP_MISC_p) (&reg))->b.fsvm_out_en = 0;
	}
	writel(reg, regs_base + USBCON_REG_HSP_MISC);
}

u8 exynos_usbcon_get_fs_vplus_vminus(struct exynos_usbphy_info *cal_info)
{
	void __iomem *regs_base = cal_info->regs_base;
	USBCON_REG_HSP_MISC_o reg;

	reg.data = readl(regs_base + USBCON_REG_HSP_MISC);

	return ((reg.b.fsvminus & 0x1) << 1) | (reg.b.fsvplus & 0x1);
}

static void ready_rewa(struct exynos_usbphy_info *cal_info)
{
	u32 reg;
	void __iomem *regs_base = cal_info->regs_base;

	/* Disable ReWA */
	reg = readl(regs_base + USBCON_REG_REWA_CTL);
	((USBCON_REG_REWA_CTL_p) (&reg))->b.hsrewa_en = 0;
	writel(reg, regs_base + USBCON_REG_REWA_CTL);

	/* Config ReWA Operation */
	reg = readl(regs_base + USBCON_REG_HSREWA_CTL);
	/* Select line state check circuit
	 * 0 : FSVPLUS/FSMINUS
	 * 1 : LINE STATE
	 */
	((USBCON_REG_HSREWA_CTL_p) (&reg))->b.dpdm_mon_sel = 1;
	/* Select Drive K circuit
	 * 0 : Auto Resume in the PHY
	 * 1 : BYPASS mode by ReWA
	 */
	((USBCON_REG_HSREWA_CTL_p) (&reg))->b.dig_bypass_con_en = 0;
	writel(reg, regs_base + USBCON_REG_HSREWA_CTL);

	/* Set host K timeout from host K drive. */
	reg = 0xFF00; // value 0xFF00 means 200ms (Nominal host K drive time is 20ms)
	writel(reg, regs_base + USBCON_REG_HSREWA_REFTO);

	/* Set host K delay from device K*/
	reg = 0x1; // value 1 means 30.5us
	writel(reg, regs_base + USBCON_REG_HSREWA_HSTK);

	/* Mask wakeup_req and all inetrrupts */
	reg = readl(regs_base + USBCON_REG_HSREWA_INTR);
	((USBCON_REG_HSREWA_INTR_p) (&reg))->b.wakeup_req_mask = 1;
	((USBCON_REG_HSREWA_INTR_p) (&reg))->b.timeout_intr_mask = 1;
	((USBCON_REG_HSREWA_INTR_p) (&reg))->b.event_intr_mask = 1;
	((USBCON_REG_HSREWA_INTR_p) (&reg))->b.wakeup_intr_mask = 1;
	writel(reg, regs_base + USBCON_REG_HSREWA_INTR);

	/* Mask all INT events  */
	reg = readl(regs_base + USBCON_REG_HSREWA_INT1_EVNT_MSK);
	((USBCON_REG_HSREWA_INT1_EVNT_MSK_p) (&reg))->b.err_sus_mask = 1;
	((USBCON_REG_HSREWA_INT1_EVNT_MSK_p) (&reg))->b.err_dev_k_mask = 1;
	((USBCON_REG_HSREWA_INT1_EVNT_MSK_p) (&reg))->b.discon_mask = 1;
	((USBCON_REG_HSREWA_INT1_EVNT_MSK_p) (&reg))->b.bypass_dis_mask = 1;
	((USBCON_REG_HSREWA_INT1_EVNT_MSK_p) (&reg))->b.ret_dis_mask = 1;
	((USBCON_REG_HSREWA_INT1_EVNT_MSK_p) (&reg))->b.ret_en_mask = 1;
	writel(reg, regs_base + USBCON_REG_HSREWA_INT1_EVNT_MSK);
}

int exynos_usbcon_enable_rewa(struct exynos_usbphy_info *cal_info)
{
	void __iomem *regs_base = cal_info->regs_base;
	u32 reg;
	int cnt;

	/* Set the wakeup source mask */
	reg = readl(regs_base + USBCON_REG_HSREWA_INTR);
	((USBCON_REG_HSREWA_INTR_p) (&reg))->b.wakeup_req_mask = 1;
	((USBCON_REG_HSREWA_INTR_p) (&reg))->b.timeout_intr_mask = 0;
	((USBCON_REG_HSREWA_INTR_p) (&reg))->b.event_intr_mask = 0;
	((USBCON_REG_HSREWA_INTR_p) (&reg))->b.wakeup_intr_mask = 0;
	writel(reg, regs_base + USBCON_REG_HSREWA_INTR);

	/* Clear the link_ready and sys_valid flag */
	reg = readl(regs_base + USBCON_REG_HSREWA_CTL);
	((USBCON_REG_HSREWA_CTL_p) (&reg))->b.hs_link_ready = 0;
	((USBCON_REG_HSREWA_CTL_p) (&reg))->b.hs_sys_valid = 0;
	writel(reg, regs_base + USBCON_REG_HSREWA_CTL);

	/* Enable ReWA */
	reg = readl(regs_base + USBCON_REG_REWA_CTL);
	((USBCON_REG_REWA_CTL_p) (&reg))->b.hsrewa_en = 1;
	writel(reg, regs_base + USBCON_REG_REWA_CTL);

	/* Check Status : Wait ReWA Status is retention enabled */
	for (cnt = 10000; cnt != 0; cnt--) {
		reg = readl(regs_base + USBCON_REG_HSREWA_INT1_EVNT);
		/* non suspend status*/
		if (((USBCON_REG_HSREWA_INT1_EVNT_p) (&reg))->b.err_sus)
			return HS_REWA_EN_STS_NOT_SUSPEND;

		/* Disconnect Status */
		if (((USBCON_REG_HSREWA_INT1_EVNT_p) (&reg))->b.discon)
			return HS_REWA_EN_STS_DISCONNECT;

		/* Success ReWA Enable */
		if (((USBCON_REG_HSREWA_INT1_EVNT_p) (&reg))->b.ret_en)
			break;
		udelay(30);
	}
	if (cnt == 0)
		return HS_REWA_EN_STS_NOT_SUSPEND;

	/* Set the INT1 for detect K and Disconnect */
	reg = readl(regs_base + USBCON_REG_HSREWA_INT1_EVNT_MSK);
	((USBCON_REG_HSREWA_INT1_EVNT_MSK_p) (&reg))->b.err_sus_mask = 1;
	((USBCON_REG_HSREWA_INT1_EVNT_MSK_p) (&reg))->b.err_dev_k_mask = 0;
	((USBCON_REG_HSREWA_INT1_EVNT_MSK_p) (&reg))->b.discon_mask = 0;
	((USBCON_REG_HSREWA_INT1_EVNT_MSK_p) (&reg))->b.bypass_dis_mask = 1;
	((USBCON_REG_HSREWA_INT1_EVNT_MSK_p) (&reg))->b.ret_dis_mask = 1;
	((USBCON_REG_HSREWA_INT1_EVNT_MSK_p) (&reg))->b.ret_en_mask = 1;
	writel(reg, regs_base + USBCON_REG_HSREWA_INT1_EVNT_MSK);

	/* Clear the wakeup source mask */
	reg = readl(regs_base + USBCON_REG_HSREWA_INTR);
	((USBCON_REG_HSREWA_INTR_p) (&reg))->b.wakeup_req_mask = 0;
	writel(reg, regs_base + USBCON_REG_HSREWA_INTR);

	udelay(100);

	return HS_REWA_EN_STS_ENABLED;
}

int exynos_usbcon_rewa_req_sys_valid(struct exynos_usbphy_info *cal_info)
{
	void __iomem *regs_base = cal_info->regs_base;
	u32 reg;
	int cnt;

	/* Set interrupt mask for prevent additional interrupt */
	reg = readl(regs_base + USBCON_REG_HSREWA_INTR);
	((USBCON_REG_HSREWA_INTR_p) (&reg))->b.wakeup_req_mask = 1;
	((USBCON_REG_HSREWA_INTR_p) (&reg))->b.timeout_intr_mask = 1;
	((USBCON_REG_HSREWA_INTR_p) (&reg))->b.event_intr_mask = 1;
	((USBCON_REG_HSREWA_INTR_p) (&reg))->b.wakeup_intr_mask = 1;
	writel(reg, regs_base + USBCON_REG_HSREWA_INTR);

	/* Set the system valid flag after all System clock resumed */
	reg = readl(regs_base + USBCON_REG_HSREWA_CTL);
	((USBCON_REG_HSREWA_CTL_p) (&reg))->b.hs_sys_valid = 1;
	writel(reg, regs_base + USBCON_REG_HSREWA_CTL);

	/* Check Status : Wait ReWA Status is retention disabled */
	for (cnt = 10000; cnt != 0; cnt--) {

		reg = readl(regs_base + USBCON_REG_HSREWA_INT1_EVNT);

		/* Disconnect Status */
		if (((USBCON_REG_HSREWA_INT1_EVNT_p) (&reg))->b.discon)
			return HS_REWA_EN_STS_DISCONNECT;

		/* Success ReWA Enable */
		if (((USBCON_REG_HSREWA_INT1_EVNT_p) (&reg))->b.ret_dis)
			break;
		udelay(30);
	}

	return HS_REWA_EN_STS_DISABLED;
}

int exynos_usbcon_rewa_disable(struct exynos_usbphy_info *cal_info)
{
	void __iomem *regs_base = cal_info->regs_base;
	u32 reg;
	int cnt;

	/* Check ReWA already disable
	 * If ReWA was disabled states, disabled sequence is already done */
	reg = readl(regs_base + USBCON_REG_REWA_CTL);
	if (!(((USBCON_REG_REWA_CTL_p) (&reg))->b.hsrewa_en))
		return 0;

	/* Set the USB link_ready flag */
	reg = readl(regs_base + USBCON_REG_HSREWA_CTL);
	((USBCON_REG_HSREWA_CTL_p) (&reg))->b.hs_link_ready = 1;
	writel(reg, regs_base + USBCON_REG_HSREWA_CTL);

	/* Wait for Digital bypass control signals is disabled event */
	for (cnt = 10000; cnt != 0; cnt--) {
		reg = readl(regs_base + USBCON_REG_HSREWA_INT1_EVNT);
		if (((USBCON_REG_HSREWA_INT1_EVNT_p) (&reg))->b.bypass_dis)
			break;
		udelay(30);
	}

	if (!cnt)
		return -1;

	/* Wait for HS-ReWA done */
	for (cnt = 1000; cnt != 0; cnt--) {
		reg = readl(regs_base + USBCON_REG_HSREWA_CTL);
		if (((USBCON_REG_HSREWA_CTL_p) (&reg))->b.hs_rewa_done)
			break;
		udelay(30);
	}

	if (!cnt)
		return -1;

	reg = readl(regs_base + USBCON_REG_REWA_CTL);
	((USBCON_REG_REWA_CTL_p) (&reg))->b.hsrewa_en = 0;
	writel(reg, regs_base + USBCON_REG_REWA_CTL);

	return 0;
}

int exynos_usbcon_rewa_cancel(struct exynos_usbphy_info *cal_info)
{
	void __iomem *regs_base = cal_info->regs_base;
	u32 reg;

	/* Check ReWA already disable
	 * If ReWA was disabled states, disabled sequence is already done */
	reg = readl(regs_base + USBCON_REG_REWA_CTL);
	if (!(((USBCON_REG_REWA_CTL_p) (&reg))->b.hsrewa_en))
		return 0;

	reg = readl(regs_base + USBCON_REG_REWA_CTL);
	((USBCON_REG_REWA_CTL_p) (&reg))->b.hsrewa_en = 0;
	writel(reg, regs_base + USBCON_REG_REWA_CTL);

	return 0;
}

void exynos_usbcon_u3_rewa_enable(struct exynos_usbphy_info *cal_info, int lfps_overlap_mode)
{
	void __iomem *regs_base = cal_info->regs_base;
	u32 reg;
	u32 lfpsresp_limit_cnt = 0x39219;   // 9ms

	reg = readl(regs_base + USBCON_REG_U3REWA_CTRL);
	((USBCON_REG_U3REWA_CTRL_p) (&reg))->b.check_u3 = 0;
	((USBCON_REG_U3REWA_CTRL_p) (&reg))->b.u3rewa_blk_en = 1;

	if (!lfps_overlap_mode) {
		/* Disable overlap_lfps */
		((USBCON_REG_U3REWA_CTRL_p) (&reg))->b.overlap_lfps = 0;
	} else {
		/* Enable overlap_lfps */
		((USBCON_REG_U3REWA_CTRL_p) (&reg))->b.overlap_lfps = 1;

		/* Set lfpsresp_limit_cnt */
		writel(lfpsresp_limit_cnt, regs_base + USBCON_REG_U3REWA_LMT_CNT);
	}

	writel(reg, regs_base + USBCON_REG_U3REWA_CTRL);
}

void exynos_usbcon_u3_rewa_disable(struct exynos_usbphy_info *cal_info)
{
	void __iomem *regs_base = cal_info->regs_base;
	u32 reg;

	reg = readl(regs_base + USBCON_REG_U3REWA_CTRL);
	((USBCON_REG_U3REWA_CTRL_p) (&reg))->b.u3rewa_blk_en = 0;
	writel(reg, regs_base + USBCON_REG_U3REWA_CTRL);
}
