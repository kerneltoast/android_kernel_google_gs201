// SPDX-License-Identifier: GPL-2.0-only
//
// UFS Host Controller driver for Exynos specific extensions
//
// Copyright (C) 2013-2014 Samsung Electronics Co., Ltd.
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/clk.h>
#include <linux/smc.h>
#include <ufs/ufshcd.h>
#include <host/ufshcd-pltfrm.h>
#include <ufs/ufshci.h>
#include <ufs/unipro.h>
#include <ufs/ufs_quirks.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/spinlock.h>

#include "ufs-exynos-gs.h"
#include "ufs-pixel-crypto.h"
#include <soc/google/exynos-pmu-if.h>
#include <soc/google/exynos-cpupm.h>
#include <trace/hooks/ufshcd.h>

static unsigned int desired_power_mode_gear;
module_param(desired_power_mode_gear, uint, 0444);
MODULE_PARM_DESC(desired_power_mode_gear, "Target UFS HS/PWM Gear, "
					  "0 ( = not set) "
					  "1-4 ( = use specified gear)");
#define POWER_MODE_GEAR_VALID(x) ((x) && ((x) <= 4))

static unsigned int desired_power_mode_lane;
module_param(desired_power_mode_lane, uint, 0444);
MODULE_PARM_DESC(desired_power_mode_lane, "Target UFS Number of Lanes, "
					  "0 ( = not set) "
					  "1-2 ( = use specified num lanes)");
#define POWER_MODE_LANE_VALID(x) ((x) && ((x) <= 2))

static unsigned int desired_power_mode_pwr;
module_param(desired_power_mode_pwr, uint, 0444);
MODULE_PARM_DESC(desired_power_mode_pwr, "Target UFS PA Power Mode, "
					 "0 ( = not set) "
					 "1 ( = FAST_MODE) "
					 "2 ( = SLOW_MODE) "
					 "4 ( = FASTAUTO_MODE) "
					 "5 ( = SLOWAUTO_MODE)");
#define POWER_MODE_PWR_VALID(x) (((x) <= 5) && (0x36 & (1 << (x))))

static unsigned int desired_power_mode_hs_rate;
module_param(desired_power_mode_hs_rate, uint, 0444);
MODULE_PARM_DESC(desired_power_mode_hs_rate, "Target UFS HS Series, "
					     "0 ( = not set) "
					     "1 ( = HS Series A) "
					     "2 ( = HS Series B)");
#define POWER_MODE_HS_RATE_VALID(x) ((x) && ((x) <= 2))

#define IS_C_STATE_ON(h) ((h)->c_state == C_ON)
#define PRINT_STATES(h)						\
	dev_err((h)->dev, "%s: prev h_state %d, cur c_state %d\n",	\
				__func__, (h)->h_state, (h)->c_state)

/* phy context retention control */
#define UNIP_PA_DBG_OPTION_SUITE_1	0x39A8
#define DBG_SUITE1_ENABLE		0x90913C1C
#define DBG_SUITE1_DISABLE		0x98913C1C

#define UNIP_PA_DBG_OPTION_SUITE_2	0x39B4
#define DBG_SUITE2_ENABLE		0xE01C115F
#define DBG_SUITE2_DISABLE		0xE01C195F

#define H8T_GRANULARITY         100

static struct exynos_ufs *ufs_host_backup[1];
static int ufs_host_index;
static const char *res_token[2] = {
	"passes",
	"fails",
};

enum {
	UFS_S_TOKEN_FAIL,
	UFS_S_TOKEN_NUM,
};

static const char *ufs_s_str_token[UFS_S_TOKEN_NUM] = {
	"fail to",
};

static const char *ufs_pmu_token = "ufs-phy-iso";
static const char *ufs_ext_blks[EXT_BLK_MAX][2] = {
	{"samsung,sysreg-phandle", "ufs-iocc"},	/* sysreg */
};

static const int ufs_ext_ignore[EXT_BLK_MAX] = {0};

/*
 * This type makes 1st DW and another DW be logged.
 * The second one is the head of CDB for COMMAND UPIU and
 * the head of data for DATA UPIU.
 */
static const int __cport_log_type = 0x22;

struct pixel_ufs *to_pixel_ufs(struct ufs_hba *hba)
{
	return &to_exynos_ufs(hba)->pixel_ufs;
}

/* Functions to map registers or to something by other modules */
static void ufs_udelay(u32 n)
{
	udelay(n);
}

static inline void ufs_map_vs_regions(struct exynos_ufs *ufs)
{
	ufs->handle.hci = ufs->reg_hci;
	ufs->handle.ufsp = ufs->reg_ufsp;
	ufs->handle.unipro = ufs->reg_unipro;
	ufs->handle.pma = ufs->reg_phy;
	ufs->handle.cport = ufs->reg_cport;
	ufs->handle.udelay = ufs_udelay;
}

/* Helper for UFS CAL interface */
static int ufs_call_cal(struct exynos_ufs *ufs, int init, void *func)
{
	struct ufs_cal_param *p = &ufs->cal_param;
	struct ufs_vs_handle *handle = &ufs->handle;
	int ret;
	u32 reg;
	cal_if_func_init fn_init;
	cal_if_func fn;

	/* Enable MPHY APB */
	reg = hci_readl(handle, HCI_FORCE_HCS);
	reg &= ~(UNIPRO_MCLK_STOP_EN | MPHY_APBCLK_STOP_EN);
	hci_writel(handle, reg, HCI_FORCE_HCS);

	if (init) {
		fn_init = (cal_if_func_init)func;
		ret = fn_init(p, ufs_host_index);
	} else {
		fn = (cal_if_func)func;
		ret = fn(p);
	}
	if (ret != UFS_CAL_NO_ERROR) {
		dev_err(ufs->dev, "%s: %d\n", __func__, ret);
		ret = -EPERM;
	}

	/* Disable MPHY APB */
	reg = hci_readl(handle, HCI_FORCE_HCS);
	reg |= MPHY_APBCLK_STOP_EN;
	hci_writel(handle, reg, HCI_FORCE_HCS);

	return ret;

}

static inline void __pm_qos_ctrl(struct exynos_ufs *ufs, bool hold)
{
#if defined(CONFIG_EXYNOS_PM_QOS) || defined(CONFIG_EXYNOS_PM_QOS_MODULE)
	s32 val = hold ? ufs->pm_qos_int_value : 0;

	exynos_pm_qos_update_request(&ufs->pm_qos_int, val);
#endif
}

static inline void __sicd_ctrl(struct exynos_ufs *ufs, bool hold)
{
#if IS_ENABLED(CONFIG_EXYNOS_CPUPM)
	/*
	 * 0 : block to enter system idle state
	 * 1 : allow to use system idle state
	 */
	exynos_update_ip_idle_status(ufs->idle_ip_index, !hold);
#endif
}

static void exynos_ufs_update_active_lanes(struct ufs_hba *hba)
{
	struct exynos_ufs *ufs = to_exynos_ufs(hba);
	struct ufs_cal_param *p = &ufs->cal_param;
	struct ufs_vs_handle *handle = &ufs->handle;
	u32 active_tx_lane = 0;
	u32 active_rx_lane = 0;

	active_tx_lane = unipro_readl(handle, UNIP_PA_ACTIVETXDATALENS);
	active_rx_lane = unipro_readl(handle, UNIP_PA_ACTIVERXDATALENS);

	/*
	 * Exynos driver doesn't consider asymmetric lanes, e.g. rx=2, tx=1
	 * so, for the cases, panic occurs to detect when you face new hardware
	 */
	if (!active_rx_lane || !active_tx_lane || active_rx_lane != active_tx_lane) {
		dev_err(hba->dev, "%s: invalid active lanes. rx=%d, tx=%d\n",
			__func__, active_rx_lane, active_tx_lane);
		WARN_ON(1);
	}
	p->active_rx_lane = (u8)active_rx_lane;
	p->active_tx_lane = (u8)active_tx_lane;

	if (!hba->clk_gating.is_suspended)
		dev_info(ufs->dev, "PA_ActiveTxDataLanes(%d),"
				   "PA_ActiveRxDataLanes(%d)\n",
				    active_tx_lane, active_rx_lane);
}

static void exynos_ufs_get_caps_after_link(struct ufs_hba *hba)
{
	struct exynos_ufs *ufs = to_exynos_ufs(hba);
	struct uic_pwr_mode *pmd = &ufs->req_pmd_parm;
	struct ufs_cal_param *p = &ufs->cal_param;
	struct ufs_vs_handle *handle = &ufs->handle;
	u32 max_rx_hs_gear = 0;
	u32 connected_tx_lane = 0;
	u32 connected_rx_lane = 0;

	connected_tx_lane = unipro_readl(handle, UNIP_PA_CONNECTEDTXDATALANES);
	connected_rx_lane = unipro_readl(handle, UNIP_PA_CONNECTEDRXDATALANES);
	max_rx_hs_gear = unipro_readl(&ufs->handle, UNIP_PA_MAXRXHSGEAR);

	/* check connected lanes, not permit asymmetric situations */
	if (!connected_tx_lane || !connected_rx_lane ||
	    connected_tx_lane != connected_rx_lane)
		dev_err(hba->dev, "%s: asymmetric connected lanes. rx=%d, tx=%d\n",
			__func__, connected_rx_lane, connected_tx_lane);

	/* set variables for CAL */
	p->connected_tx_lane = (u8)connected_rx_lane;
	p->connected_rx_lane = (u8)connected_rx_lane;
	p->max_gear = min_t(u8, max_rx_hs_gear, pmd->gear);

	dev_info(ufs->dev, "PA_ActiveTxDataLanes(%d), PA_ActiveRxDataLanes(%d)\n",
		connected_tx_lane, connected_rx_lane);
	if (!hba->clk_gating.is_suspended)
		dev_info(ufs->dev, "max_gear(%d), PA_MaxRxHSGear(%d)\n",
			p->max_gear, max_rx_hs_gear);

	/* set for sysfs */
	ufs->params[UFS_SYSFS_EOM_SZ] =
			EOM_PH_SEL_MAX * EOM_DEF_VREF_MAX *
			ufs_s_eom_repeat[ufs->cal_param.max_gear];
}

static inline void exynos_ufs_ctrl_phy_pwr(struct exynos_ufs *ufs, bool en)
{
	struct ext_cxt *cxt = &ufs->cxt_phy_iso;

	exynos_pmu_update(cxt->offset, cxt->mask, (en ? 1 : 0) ? cxt->val : 0);

	if (en)
		pixel_update_power_event(ufs->hba, PE_SYSTEM_RESUME);
	else
		pixel_update_power_event(ufs->hba, PE_SYSTEM_SUSPEND);
}

static inline void __thaw_cport_logger(struct ufs_vs_handle *handle)
{
	hci_writel(handle, __cport_log_type, 0x114);
	hci_writel(handle, 1, 0x110);
}

static inline void __freeze_cport_logger(struct ufs_vs_handle *handle)
{
	hci_writel(handle, 0, 0x110);
}

/*
 * Exynos debugging main function
 */
static void exynos_ufs_dump_debug_info(struct ufs_hba *hba)
{
	struct exynos_ufs *ufs = to_exynos_ufs(hba);
	struct ufs_vs_handle *handle = &ufs->handle;
	unsigned long flags;

	spin_lock_irqsave(&ufs->dbg_lock, flags);
	if (ufs->under_dump == 0) {
		ufs->under_dump = 1;
	} else {
		spin_unlock_irqrestore(&ufs->dbg_lock, flags);
		goto out;
	}
	spin_unlock_irqrestore(&ufs->dbg_lock, flags);

	/* freeze cport logger */
	__freeze_cport_logger(handle);

	exynos_ufs_dump_info(handle, ufs->dev);

	/* thaw cport logger */
	__thaw_cport_logger(handle);

	spin_lock_irqsave(&ufs->dbg_lock, flags);
	ufs->under_dump = 0;
	spin_unlock_irqrestore(&ufs->dbg_lock, flags);
out:
	return;
}

inline void exynos_ufs_ctrl_auto_hci_clk(struct exynos_ufs *ufs, bool en)
{
	u32 reg = hci_readl(&ufs->handle, HCI_FORCE_HCS);

	if (en)
		hci_writel(&ufs->handle, reg | HCI_CORECLK_STOP_EN,
			   HCI_FORCE_HCS);
	else
		hci_writel(&ufs->handle, reg & ~HCI_CORECLK_STOP_EN,
			   HCI_FORCE_HCS);
}

static inline void exynos_ufs_ctrl_clk(struct exynos_ufs *ufs, bool en)
{
	u32 reg = hci_readl(&ufs->handle, HCI_FORCE_HCS);

	if (en)
		hci_writel(&ufs->handle, reg | CLK_STOP_CTRL_EN_ALL,
			   HCI_FORCE_HCS);
	else
		hci_writel(&ufs->handle, reg & ~CLK_STOP_CTRL_EN_ALL,
			   HCI_FORCE_HCS);
}

static inline void exynos_ufs_gate_clk(struct exynos_ufs *ufs, bool en)
{
	u32 reg = hci_readl(&ufs->handle, HCI_CLKSTOP_CTRL);

	if (en)
		hci_writel(&ufs->handle, reg | CLK_STOP_ALL,
			   HCI_CLKSTOP_CTRL);
	else
		hci_writel(&ufs->handle, reg & ~CLK_STOP_ALL,
			   HCI_CLKSTOP_CTRL);
}

static void exynos_ufs_set_unipro_mclk(struct exynos_ufs *ufs)
{
	ufs->mclk_rate = (u32)clk_get_rate(ufs->clk_unipro);
	if (!ufs->hba->clk_gating.is_suspended)
		dev_info(ufs->dev, "mclk: %u\n", ufs->mclk_rate);
}

static void exynos_ufs_fit_aggr_timeout(struct exynos_ufs *ufs)
{
	u32 cnt_val;
	unsigned long nVal;

	/* IA_TICK_SEL : 1(1us_TO_CNT_VAL) */
	nVal = hci_readl(&ufs->handle, HCI_UFSHCI_V2P1_CTRL);
	nVal |= IA_TICK_SEL;
	hci_writel(&ufs->handle, nVal, HCI_UFSHCI_V2P1_CTRL);

	cnt_val = ufs->mclk_rate / 1000000;
	hci_writel(&ufs->handle, cnt_val & CNT_VAL_1US_MASK,
		   HCI_1US_TO_CNT_VAL);
}

static void exynos_ufs_init_pmc_req(struct ufs_hba *hba,
				    struct ufs_pa_layer_attr *pwr_max,
				    struct ufs_pa_layer_attr *pwr_req)
{
	struct exynos_ufs *ufs = to_exynos_ufs(hba);
	struct uic_pwr_mode *req_pmd = &ufs->req_pmd_parm;
	struct uic_pwr_mode *act_pmd = &ufs->act_pmd_parm;

	/*
	 * Exynos driver doesn't consider asymmetric lanes, e.g. rx=2, tx=1
	 * so, for the cases, panic occurs to detect when you face new hardware.
	 * pwr_max comes from core driver, i.e. ufshcd. That keeps the number
	 * of connected lanes.
	 */
	if (pwr_max->lane_rx != pwr_max->lane_tx) {
		dev_err(hba->dev, "%s: invalid connected lanes. rx=%d, tx=%d\n",
			__func__, pwr_max->lane_rx, pwr_max->lane_tx);
		WARN_ON(1);
	}

	act_pmd->gear = min_t(u8, pwr_max->gear_rx, req_pmd->gear);
	pwr_req->gear_rx = act_pmd->gear;

	act_pmd->gear = min_t(u8, pwr_max->gear_tx, req_pmd->gear);
	pwr_req->gear_tx = act_pmd->gear;

	act_pmd->lane = min_t(u8, pwr_max->lane_rx, req_pmd->lane);
	pwr_req->lane_rx = act_pmd->lane;

	act_pmd->lane = min_t(u8, pwr_max->lane_tx, req_pmd->lane);
	pwr_req->lane_tx = act_pmd->lane;

	act_pmd->mode = req_pmd->mode;
	pwr_req->pwr_rx = act_pmd->mode;
	pwr_req->pwr_tx = act_pmd->mode;

	act_pmd->hs_series = req_pmd->hs_series;
	pwr_req->hs_rate = act_pmd->hs_series;
}

static void exynos_ufs_dev_hw_reset(struct ufs_hba *hba)
{
	struct exynos_ufs *ufs = to_exynos_ufs(hba);

	/* bit[1] for resetn */
	hci_writel(&ufs->handle, 0 << 0, HCI_GPIO_OUT);
	udelay(5);
	hci_writel(&ufs->handle, 1 << 0, HCI_GPIO_OUT);
}

static inline void exynos_enable_vendor_irq(struct exynos_ufs *ufs)
{
	struct ufs_vs_handle *handle = &ufs->handle;
	u32 reg;

	/* report to IS.UE reg when UIC error happens during AH8 */
	reg = hci_readl(handle, HCI_VENDOR_SPECIFIC_IE);
	reg |= AH8_ERR_REPORT_UE;
	hci_writel(handle, reg, HCI_VENDOR_SPECIFIC_IE);
}

static void exynos_ufs_config_host(struct exynos_ufs *ufs)
{
	u32 reg;

	/* internal clock control */
	exynos_ufs_ctrl_auto_hci_clk(ufs, false);
	exynos_ufs_set_unipro_mclk(ufs);

	/* period for interrupt aggregation */
	exynos_ufs_fit_aggr_timeout(ufs);

	/* misc HCI configurations */
	hci_writel(&ufs->handle, 0xA, HCI_DATA_REORDER);
	hci_writel(&ufs->handle, PRDT_PREFECT_EN | PRDT_SET_SIZE(12),
		   HCI_TXPRDT_ENTRY_SIZE);
	hci_writel(&ufs->handle, PRDT_SET_SIZE(12), HCI_RXPRDT_ENTRY_SIZE);
	hci_writel(&ufs->handle, 0xFFFFFFFF, HCI_UTRL_NEXUS_TYPE);
	hci_writel(&ufs->handle, 0xFFFFFFFF, HCI_UTMRL_NEXUS_TYPE);

	reg = hci_readl(&ufs->handle,
			HCI_AXIDMA_RWDATA_BURST_LEN) & ~BURST_LEN(0);
	hci_writel(&ufs->handle, WLU_EN | BURST_LEN(3),
		   HCI_AXIDMA_RWDATA_BURST_LEN);

	/*
	 * enable HWAGC control by IOP
	 *
	 * default value 1->0 at KC.
	 * always "0"(controlled by UFS_ACG_DISABLE)
	 */
	reg = hci_readl(&ufs->handle, HCI_IOP_ACG_DISABLE);
	hci_writel(&ufs->handle, reg & (~HCI_IOP_ACG_DISABLE_EN),
		   HCI_IOP_ACG_DISABLE);

	unipro_writel(&ufs->handle, DBG_SUITE1_ENABLE,
			UNIP_PA_DBG_OPTION_SUITE_1);
	unipro_writel(&ufs->handle, DBG_SUITE2_ENABLE,
			UNIP_PA_DBG_OPTION_SUITE_2);

	/*
	 * There are some cases that should be reported as AH8 error,
	 * i.e. IS.UHES or UHXS, but not. To cover the cases, I enable
	 * one vendor interrupt source that causes to raise IS.UE.
	 * I check an UIC error in the vendor hook function named
	 * __check_int_errors to report it as fatal.
	 */
	if (ufshcd_is_auto_hibern8_supported(ufs->hba) && ufs->ah8_ahit)
		exynos_enable_vendor_irq(ufs);
}

static int exynos_ufs_config_externals(struct exynos_ufs *ufs)
{
	int ret = 0;
	int i;
	struct regmap **p = NULL;
	struct ext_cxt *q = NULL;

	/* PHY isolation bypass */
	exynos_ufs_ctrl_phy_pwr(ufs, true);

	/* Set for UFS iocc */
	for (i = EXT_SYSREG, p = &ufs->regmap_sys, q = &ufs->cxt_iocc;
			i < EXT_BLK_MAX; i++, p++, q++) {
		if (IS_ERR_OR_NULL(*p)) {
			if (!ufs_ext_ignore[i])
				ret = -EINVAL;
			dev_err(ufs->dev, "Unable to control %s\n",
				ufs_ext_blks[i][1]);
			goto out;
		}
		regmap_update_bits(*p, q->offset, q->mask, q->val);
	}

out:
	return ret;
}

static int exynos_ufs_get_clks(struct ufs_hba *hba)
{
	struct exynos_ufs *ufs = to_exynos_ufs(hba);
	struct list_head *head = &hba->clk_list_head;
	struct ufs_clk_info *clki;
	int i = 0;

	if (!head || list_empty(head))
		goto out;

	list_for_each_entry(clki, head, list) {
		/*
		 * get clock with an order listed in device tree
		 *
		 * hci, unipro
		 */
		if (i == 0)
			ufs->clk_hci = clki->clk;
		else if (i == 1)
			ufs->clk_unipro = clki->clk;

		i++;
	}
out:
	if (!ufs->clk_hci || !ufs->clk_unipro)
		return -EINVAL;

	return 0;
}

static int exynos_ufs_check_ah8_fsm_state(struct ufs_hba *hba, u32 state)
{
	struct exynos_ufs *ufs = to_exynos_ufs(hba);
	struct ufs_vs_handle *handle = &ufs->handle;
	int retry = 50, ret = -EINVAL;
	u32 reg;

	while (retry--) {
		reg = hci_readl(handle, HCI_AH8_STATE);

		if (reg & HCI_AH8_STATE_ERROR)
			goto out;

		if (reg & state)
			break;

		usleep_range(1000, 1100);
	}

	if (!retry)
		goto out;

	ret = 0;
out:
	dev_info(hba->dev, "%s: cnt = %d, state = %08X, reg = %08X\n",
			__func__, retry, state, reg);
	WARN_ONCE(ret, "ret = %d\n", ret);

	return ret;
}

static void exynos_ufs_set_features(struct ufs_hba *hba)
{
	struct device_node *np = hba->dev->of_node;
	struct exynos_ufs *ufs = to_exynos_ufs(hba);

	/* caps */
	hba->caps = UFSHCD_CAP_CLK_GATING;
	if (ufs->ah8_ahit == 0)
		hba->caps |= UFSHCD_CAP_HIBERN8_WITH_CLK_GATING;

	/* quirks of common driver */
	hba->quirks = UFSHCD_QUIRK_PRDT_BYTE_GRAN |
			UFSHCI_QUIRK_SKIP_RESET_INTR_AGGR |
			UFSHCI_QUIRK_BROKEN_REQ_LIST_CLR |
			UFSHCD_QUIRK_BROKEN_OCS_FATAL_ERROR |
			UFSHCI_QUIRK_SKIP_MANUAL_WB_FLUSH_CTRL |
			UFSHCD_QUIRK_BROKEN_AUTO_HIBERN8 |
			UFSHCD_QUIRK_SKIP_DEF_UNIPRO_TIMEOUT_SETTING;

	if (of_find_property(np, "fixed-prdt-req_list-ocs", NULL))
		hba->quirks &= ~(UFSHCD_QUIRK_PRDT_BYTE_GRAN |
				UFSHCI_QUIRK_BROKEN_REQ_LIST_CLR |
				UFSHCD_QUIRK_BROKEN_OCS_FATAL_ERROR |
				UFSHCI_QUIRK_SKIP_RESET_INTR_AGGR);
}

/*
 * Exynos-specific callback functions
 */

static int exynos_ufs_init(struct ufs_hba *hba)
{
	struct exynos_ufs *ufs = to_exynos_ufs(hba);
	int ret;

	/* refer to hba */
	ufs->hba = hba;

	/* configure externals */
	ret = exynos_ufs_config_externals(ufs);
	if (ret)
		return ret;

	/* to read standard hci registers */
	ufs->handle.std = hba->mmio_base;

	/* get some clock sources and debug information structures */
	ret = exynos_ufs_get_clks(hba);
	if (ret)
		return ret;

	/* set features, such as caps or quirks */
	exynos_ufs_set_features(hba);

	return pixel_init(hba, ufs->dev, &exynos_crypto_ops);
}

static void exynos_ufs_init_host(struct ufs_hba *hba)
{
	struct exynos_ufs *ufs = to_exynos_ufs(hba);
	unsigned long timeout = jiffies + msecs_to_jiffies(1);

	/* host reset */
	hci_writel(&ufs->handle, UFS_SW_RST_MASK, HCI_SW_RST);

	do {
		if (!(hci_readl(&ufs->handle, HCI_SW_RST) & UFS_SW_RST_MASK))
			goto success;
	} while (time_before(jiffies, timeout));

	dev_err(ufs->dev, "timeout host sw-reset\n");

	exynos_ufs_dump_info(&ufs->handle, ufs->dev);

	goto out;

success:
	/* configure host */
	exynos_ufs_config_host(ufs);
out:
	return;
}

static int exynos_ufs_setup_clocks(struct ufs_hba *hba, bool on,
				   enum ufs_notify_change_status notify)
{
	struct exynos_ufs *ufs = to_exynos_ufs(hba);

	if (notify == PRE_CHANGE) {
		/* Clear for SICD */
		__sicd_ctrl(ufs, on);

		/* PM Qos hold for stability */
		__pm_qos_ctrl(ufs, on);
	} else {
		ufs->c_state = on ? C_ON : C_OFF;
	}

	return 0;
}

static int exynos_ufs_get_available_lane(struct ufs_hba *hba)
{
	struct exynos_ufs *ufs = to_exynos_ufs(hba);
	struct ufs_vs_handle *handle = &ufs->handle;
	int ret = -EINVAL;

	/* Get the available lane count */
	ufs->available_lane_tx = unipro_readl(handle, UNIP_PA_AVAILTXDATALENS);
	ufs->available_lane_rx = unipro_readl(handle, UNIP_PA_AVAILRXDATALENS);

	/*
	 * Exynos driver doesn't consider asymmetric lanes, e.g. rx=2, tx=1
	 * so, for the cases, panic occurs to detect when you face new hardware
	 */
	if (!ufs->available_lane_rx || !ufs->available_lane_tx ||
	    ufs->available_lane_rx != ufs->available_lane_tx) {
		dev_err(hba->dev, "%s: invalid host available lanes. rx=%d, tx=%d\n",
			__func__,
			ufs->available_lane_rx,
			ufs->available_lane_tx);
		WARN_ON(1);
		goto out;
	}
	ret = exynos_ufs_dbg_set_lanes(handle, ufs->dev,
				       ufs->available_lane_rx);
	if (ret)
		goto out;

	ufs->num_lanes = ufs->available_lane_rx;
	ret = 0;
out:
	return ret;
}

static void exynos_ufs_override_hba_params(struct ufs_hba *hba)
{
	hba->spm_lvl = UFS_PM_LVL_5;
}

static void exynos_ufs_update_clkgate_delay_ms(struct ufs_hba *hba)
{
	int timer, scale;

	if (!hba->ahit)
		return;

	timer = FIELD_GET(UFSHCI_AHIBERN8_TIMER_MASK, hba->ahit);
	scale = FIELD_GET(UFSHCI_AHIBERN8_SCALE_MASK, hba->ahit);
	for (; scale > 0; --scale)
		timer *= UFSHCI_AHIBERN8_SCALE_FACTOR;
	timer /= 1000;

	/* Let's use 2x delay_ms */
	if (timer)
		hba->clk_gating.delay_ms = timer << 1;
}

static int exynos_ufs_hce_enable_notify(struct ufs_hba *hba,
					enum ufs_notify_change_status notify)
{
	struct exynos_ufs *ufs = to_exynos_ufs(hba);
	int ret = 0;

	if (!IS_C_STATE_ON(ufs) ||
	    (ufs->h_state != H_DISABLED &&
	     ufs->h_state != H_SUSPEND))
		PRINT_STATES(ufs);

	switch (notify) {
	case PRE_CHANGE:
		/*
		 * The maximum segment size must be set after scsi_host_alloc()
		 * has been called and before LUN scanning starts
		 * (ufshcd_async_scan()). Note: this callback may also be called
		 * from other functions than ufshcd_init().
		 */
		hba->host->max_segment_size = 4096;

		/*
		 * This function is called in ufshcd_hba_enable,
		 * maybe boot, wake-up or link start-up failure cases.
		 * To start safely, reset of entire logics of host
		 * is required
		 */
		exynos_ufs_init_host(hba);

		/* device reset */
		exynos_ufs_dev_hw_reset(hba);

		/* deliver ah8 timer and counter values */
		hba->ahit = ufs->ah8_ahit;

		/* adjust clkgate_delay */
		exynos_ufs_update_clkgate_delay_ms(hba);
		break;
	case POST_CHANGE:
		exynos_ufs_ctrl_clk(ufs, true);
		exynos_ufs_gate_clk(ufs, false);

		ret = exynos_ufs_get_available_lane(hba);

		/* freeze cport logger */
		__thaw_cport_logger(&ufs->handle);

		ufs->h_state = H_RESET;
		break;
	default:
		break;
	}

	return ret;
}

static int exynos_ufs_link_startup_notify(struct ufs_hba *hba,
					  enum ufs_notify_change_status notify)
{
	struct exynos_ufs *ufs = to_exynos_ufs(hba);
	struct ufs_cal_param *p = &ufs->cal_param;
	struct ufs_vs_handle *handle = &ufs->handle;
	int ret = 0;

	switch (notify) {
	case PRE_CHANGE:
		exynos_ufs_override_hba_params(hba);

		if (!IS_C_STATE_ON(ufs) || ufs->h_state != H_RESET)
			PRINT_STATES(ufs);

		/* hci */
		hci_writel(&ufs->handle, DFES_ERR_EN | DFES_DEF_DL_ERRS,
			   HCI_ERROR_EN_DL_LAYER);
		hci_writel(&ufs->handle, DFES_ERR_EN | DFES_DEF_N_ERRS,
			   HCI_ERROR_EN_N_LAYER);
		hci_writel(&ufs->handle, DFES_ERR_EN | DFES_DEF_T_ERRS,
			   HCI_ERROR_EN_T_LAYER);

		/*
		 * unipro, keep phy context even if link startup fails
		 * and for unipro v1.8.
		 */
		unipro_writel(handle, DBG_SUITE1_ENABLE, UNIP_PA_DBG_OPTION_SUITE_1);
		unipro_writel(handle, DBG_SUITE2_ENABLE, UNIP_PA_DBG_OPTION_SUITE_2);

		/* cal */
		p->mclk_rate = ufs->mclk_rate;
		p->available_lane = ufs->num_lanes;
		ret = ufs_call_cal(ufs, 0, ufs_cal_pre_link);
		break;
	case POST_CHANGE:
		/*
		 * Get values updated through capability exchange.
		 * Those values could be used in CAL.
		 */
		exynos_ufs_get_caps_after_link(ufs->hba);

		/* cal */
		ret = ufs_call_cal(ufs, 0, ufs_cal_post_link);

		/* print link start-up result */
		if (!hba->clk_gating.is_suspended)
			dev_info(ufs->dev, "UFS link start-up %s\n",
				 (!ret) ? res_token[0] : res_token[1]);

		ufs->h_state = H_LINK_UP;
		break;
	default:
		break;
	}

	return ret;
}

static int exynos_ufs_pwr_change_notify(struct ufs_hba *hba,
					enum ufs_notify_change_status notify,
					struct ufs_pa_layer_attr *pwr_max,
					struct ufs_pa_layer_attr *pwr_req)
{
	struct exynos_ufs *ufs = to_exynos_ufs(hba);
	struct uic_pwr_mode *act_pmd = &ufs->act_pmd_parm;
	int ret = 0;

	switch (notify) {
	case PRE_CHANGE:
		/*
		 * we're here, that means the sequence up to fDeviceinit
		 * is doen successfully.
		 */
		if (!hba->clk_gating.is_suspended)
			dev_info(ufs->dev, "UFS device initialized\n");

		if (!IS_C_STATE_ON(ufs) || ufs->h_state != H_REQ_BUSY)
			PRINT_STATES(ufs);

		/* Set PMC parameters to be requested */
		exynos_ufs_init_pmc_req(hba, pwr_max, pwr_req);

		/* cal */
		ufs->cal_param.pmd = act_pmd;
		ret = ufs_call_cal(ufs, 0, ufs_cal_pre_pmc);

		break;
	case POST_CHANGE:
		/* update active lanes after pmc */
		exynos_ufs_update_active_lanes(hba);

		/* cal */
		ret = ufs_call_cal(ufs, 0, ufs_cal_post_pmc);

		if (!hba->clk_gating.is_suspended)
			dev_info(ufs->dev,
				 "Power mode change(%d): M(%d)G(%d)L(%d)"
				 "HS-series(%d)\n",
				 ret, act_pmd->mode, act_pmd->gear,
				 act_pmd->lane, act_pmd->hs_series);
		/*
		 * print gear change result.
		 * Exynos driver always considers gear change to
		 * HS-B and fast mode.
		 */
		if (ufs->req_pmd_parm.mode == FAST_MODE &&
		    ufs->req_pmd_parm.hs_series == PA_HS_MODE_B &&
		    !hba->clk_gating.is_suspended)
			dev_info(ufs->dev, "HS mode config %s\n",
				 (!ret) ? res_token[0] : res_token[1]);

		ufs->h_state = H_LINK_BOOST;

		break;
	default:
		break;
	}

	return ret;
}

static void exynos_ufs_set_nexus_t_xfer_req(struct ufs_hba *hba,
					    int tag, bool cmd)
{
	struct exynos_ufs *ufs = to_exynos_ufs(hba);
	u32 type;

	lockdep_assert_held(&hba->outstanding_lock);

	if (!IS_C_STATE_ON(ufs) ||
	    (ufs->h_state != H_LINK_UP &&
	     ufs->h_state != H_LINK_BOOST &&
	     ufs->h_state != H_REQ_BUSY))
		PRINT_STATES(ufs);

	type =  hci_readl(&ufs->handle, HCI_UTRL_NEXUS_TYPE);

	if (cmd)
		type |= (1 << tag);
	else
		type &= ~(1 << tag);

	hci_writel(&ufs->handle, type, HCI_UTRL_NEXUS_TYPE);

	ufs->h_state = H_REQ_BUSY;
}

static void exynos_ufs_set_nexus_t_task_mgmt(struct ufs_hba *hba,
					     int tag, u8 tm_func)
{
	struct exynos_ufs *ufs = to_exynos_ufs(hba);
	u32 type;

	lockdep_assert_held(hba->host->host_lock);

	if (!IS_C_STATE_ON(ufs) ||
	    (ufs->h_state != H_LINK_BOOST &&
	     ufs->h_state != H_TM_BUSY &&
	     ufs->h_state != H_REQ_BUSY))
		PRINT_STATES(ufs);

	type =  hci_readl(&ufs->handle, HCI_UTMRL_NEXUS_TYPE);

	switch (tm_func) {
	case UFS_ABORT_TASK:
		pixel_print_cmd_log(hba);
		fallthrough;
	case UFS_QUERY_TASK:
		type |= (1 << tag);
		break;
	case UFS_ABORT_TASK_SET:
	case UFS_CLEAR_TASK_SET:
	case UFS_LOGICAL_RESET:
	case UFS_QUERY_TASK_SET:
		type &= ~(1 << tag);
		break;
	}

	hci_writel(&ufs->handle, type, HCI_UTMRL_NEXUS_TYPE);

	ufs->h_state = H_TM_BUSY;
}

static void exynos_ufs_hibern8_notify(struct ufs_hba *hba,
				      enum uic_cmd_dme cmd,
				      enum ufs_notify_change_status notify)
{
	struct exynos_ufs *ufs = to_exynos_ufs(hba);
	int ret;

	if (notify == PRE_CHANGE && ufshcd_is_auto_hibern8_supported(hba) &&
	    ufs->ah8_ahit) {
		ufshcd_auto_hibern8_update(hba, 0);
		ret = exynos_ufs_check_ah8_fsm_state(hba, HCI_AH8_IDLE_STATE);
		if (ret)
			return;
	}

	if (cmd == UIC_CMD_DME_HIBER_ENTER) {
		if (!IS_C_STATE_ON(ufs) ||
		    (ufs->h_state != H_LINK_UP &&
		     ufs->h_state != H_REQ_BUSY &&
		     ufs->h_state != H_LINK_BOOST))
			PRINT_STATES(ufs);

		if (notify == PRE_CHANGE) {
			;
		} else {
			/* cal */
			ufs_call_cal(ufs, 0, ufs_cal_post_h8_enter);
			/* Internal clock off */
			exynos_ufs_gate_clk(ufs, true);

			ufs->h_state_prev = ufs->h_state;
			ufs->h_state = H_HIBERN8;

			pixel_ufs_record_hibern8(hba, 1);
		}
	} else {
		if (notify == PRE_CHANGE) {
			ufs->h_state = ufs->h_state_prev;

			/* Internal clock on */
			exynos_ufs_gate_clk(ufs, false);

			/* cal */
			ufs_call_cal(ufs, 0, ufs_cal_pre_h8_exit);
		} else {
			int h8_delay_ms_ovly =
				ufs->params[UFS_SYSFS_H8_D_MS];

			/* override h8 enter delay */
			if (h8_delay_ms_ovly)
				hba->clk_gating.delay_ms =
					(unsigned long)h8_delay_ms_ovly;

			pixel_ufs_record_hibern8(hba, 0);
		}
	}
}

static int __exynos_ufs_suspend(struct ufs_hba *hba, enum ufs_pm_op pm_op,
				enum ufs_notify_change_status status)
{
	struct exynos_ufs *ufs = to_exynos_ufs(hba);
	int ret = 0;

	if (status == PRE_CHANGE)
		return ret;

	if (!IS_C_STATE_ON(ufs) ||
	    ufs->h_state != H_HIBERN8)
		PRINT_STATES(ufs);

#if defined(CONFIG_EXYNOS_PM_QOS) || defined(CONFIG_EXYNOS_PM_QOS_MODULE)
	exynos_pm_qos_update_request(&ufs->pm_qos_int, 0);
#endif
	if (ufshcd_is_auto_hibern8_supported(hba) && ufs->ah8_ahit) {
		ret = exynos_ufs_check_ah8_fsm_state(hba, HCI_AH8_IDLE_STATE);
		if (ret)
			return ret;
	}

	hci_writel(&ufs->handle, 0 << 0, HCI_GPIO_OUT);

	exynos_ufs_ctrl_phy_pwr(ufs, false);

	ufs->h_state = H_SUSPEND;
	return ret;
}

static int __exynos_ufs_resume(struct ufs_hba *hba, enum ufs_pm_op pm_op)
{
	struct exynos_ufs *ufs = to_exynos_ufs(hba);
	int ret = 0;

	if (!IS_C_STATE_ON(ufs) ||
	    ufs->h_state != H_SUSPEND)
		PRINT_STATES(ufs);

	/* system init */
	ret = exynos_ufs_config_externals(ufs);
	if (ret)
		return ret;

	pixel_ufs_crypto_resume(hba);

	return 0;
}

static int __apply_dev_quirks(struct ufs_hba *hba)
{
	struct exynos_ufs *ufs = to_exynos_ufs(hba);
	struct ufs_vs_handle *handle = &ufs->handle;
	const u32 pa_h8_time_offset = 0x329C;
	u32 peer_hibern8time;
	struct ufs_cal_param *p = &ufs->cal_param;
	/* 50us is a heuristic value, so it could change later */
	u32 ref_gate_margin = (hba->dev_info.wspecversion >= 0x300) ?
		hba->dev_info.clk_gating_wait_us : 50;

	/*
	 * As for tActivate, device value is bigger than host value,
	 * while, as for tHibern8time, vice versa.
	 * For tActiavate, it's enabled by setting
	 * UFS_DEVICE_QUIRK_HOST_PA_TACTIVATE to one.
	 * In here, it's handled only for tHibern8.
	 */
	peer_hibern8time = unipro_readl(handle, pa_h8_time_offset);
	if (hba->dev_quirks & UFS_DEVICE_QUIRK_HOST_PA_TACTIVATE) {
		peer_hibern8time++;
		unipro_writel(handle, peer_hibern8time, pa_h8_time_offset);
	}

	/* TODO: confirm PA_Granularity (0x15AA) is set to 6 (100us) in case
	 * Reset Value changes
	 */
	p->ah8_thinern8_time = peer_hibern8time * H8T_GRANULARITY;
	p->ah8_brefclkgatingwaittime = ref_gate_margin;

	return 0;
}

static void __fixup_dev_quirks(struct ufs_hba *hba)
{
	hba->dev_quirks &= ~(UFS_DEVICE_QUIRK_RECOVERY_FROM_DL_NAC_ERRORS);
}

/* to make device state active */
static int __device_reset(struct ufs_hba *hba)
{
	return 0;
}

static struct ufs_hba_variant_ops exynos_ufs_ops = {
	.init = exynos_ufs_init,
	.setup_clocks = exynos_ufs_setup_clocks,
	.hce_enable_notify = exynos_ufs_hce_enable_notify,
	.link_startup_notify = exynos_ufs_link_startup_notify,
	.pwr_change_notify = exynos_ufs_pwr_change_notify,
	.setup_xfer_req = exynos_ufs_set_nexus_t_xfer_req,
	.setup_task_mgmt = exynos_ufs_set_nexus_t_task_mgmt,
	.hibern8_notify = exynos_ufs_hibern8_notify,
	.dbg_register_dump = exynos_ufs_dump_debug_info,
	.suspend = __exynos_ufs_suspend,
	.resume = __exynos_ufs_resume,
	.apply_dev_quirks = __apply_dev_quirks,
	.fixup_dev_quirks = __fixup_dev_quirks,
	.device_reset = __device_reset,
};

static void __check_int_errors(void *data, struct ufs_hba *hba,
			       bool queue_eh_work)
{
	struct exynos_ufs *ufs = to_exynos_ufs(hba);
	struct ufs_vs_handle *handle = &ufs->handle;
	u32 reg;

	if (!(hba->errors & UIC_ERROR) ||
	    !ufshcd_is_auto_hibern8_supported(hba))
		return;

	reg = hci_readl(handle, HCI_AH8_STATE);
	if (reg & HCI_AH8_STATE_ERROR)
		ufshcd_set_link_broken(hba);

	hci_writel(handle, AH8_ERR_REPORT_UE, HCI_VENDOR_SPECIFIC_IS);
}

static void exynos_ufs_register_vendor_hooks(void)
{
	register_trace_android_vh_ufs_check_int_errors(__check_int_errors,
						       NULL);
}

/*
 * This function is to define offset, mask and shift to access somewhere.
 */
static int __ufs_populate_dt_extern(struct device *dev,
				    const char *name, struct ext_cxt *cxt)
{
	struct device_node *np;
	int ret = -EINVAL;

	np = of_get_child_by_name(dev->of_node, name);
	if (!np) {
		dev_info(dev, "get node(%s) doesn't exist\n", name);
		goto out;
	}

	ret = of_property_read_u32(np, "offset", &cxt->offset);
	if (ret == 0) {
		ret = of_property_read_u32(np, "mask", &cxt->mask);
		if (ret == 0)
			ret = of_property_read_u32(np, "val", &cxt->val);
	}
	if (ret != 0) {
		dev_err(dev, "%s set cxt(%s) val\n",
			ufs_s_str_token[UFS_S_TOKEN_FAIL], name);
		goto out;
	}

	ret = 0;
out:
	return ret;
}

static int exynos_ufs_populate_dt_extern(struct device *dev,
					 struct exynos_ufs *ufs)
{
	struct device_node *np = dev->of_node;
	struct regmap **reg = NULL;
	struct ext_cxt *cxt;

	bool is_dma_coherent = !!of_find_property(dev->of_node,
						"dma-coherent", NULL);

	int i;
	int ret = -EINPROGRESS;

	/*
	 * pmu for phy isolation. for the pmu,
	 * we use api from outside, not regmap
	 */
	cxt = &ufs->cxt_phy_iso;
	ret = __ufs_populate_dt_extern(dev, ufs_pmu_token, cxt);
	if (ret) {
		dev_err(dev, "%s: %u: %s get %s\n", __func__, __LINE__,
			ufs_s_str_token[UFS_S_TOKEN_FAIL], ufs_pmu_token);
		goto out;
	}

	/* others */
	for (i = 0, reg = &ufs->regmap_sys, cxt = &ufs->cxt_iocc;
			i < EXT_BLK_MAX; i++, reg++, cxt++) {
		/* look up phandle for external regions */
		*reg = syscon_regmap_lookup_by_phandle(np, ufs_ext_blks[i][0]);
		if (IS_ERR(*reg)) {
			dev_err(dev, "%s: %u: %s find %s\n",
				__func__, __LINE__,
				ufs_s_str_token[UFS_S_TOKEN_FAIL],
				ufs_ext_blks[i][0]);
			if (ufs_ext_ignore[i])
				continue;
			else
				ret = PTR_ERR(*reg);
			goto out;
		}

		/* get and pars echild nodes for external regions in ufs node */
		ret = __ufs_populate_dt_extern(dev,
					       ufs_ext_blks[i][1], cxt);
		if (ret) {
			dev_err(dev, "%s: %u: %s get %s\n",
				__func__, __LINE__,
				ufs_s_str_token[UFS_S_TOKEN_FAIL],
				ufs_ext_blks[i][1]);
			if (ufs_ext_ignore[i]) {
				ret = 0;
				continue;
			}
			goto out;
		}

		dev_info(dev, "%s: offset 0x%x, mask 0x%x, value 0x%x\n",
			 ufs_ext_blks[i][1], cxt->offset, cxt->mask, cxt->val);
	}

	/*
	 * w/o 'dma-coherent' means the descriptors would be non-cacheable.
	 * so, iocc should be disabled.
	 */
	if (!is_dma_coherent) {
		ufs->cxt_iocc.val = 0;
		dev_info(dev, "no 'dma-coherent', ufs iocc disabled\n");
	}
out:
	return ret;
}

static int exynos_ufs_get_pwr_mode(struct device_node *np,
				   struct exynos_ufs *ufs)
{
	struct uic_pwr_mode *pmd = &ufs->req_pmd_parm;

	if (POWER_MODE_PWR_VALID(desired_power_mode_pwr))
		pmd->mode = desired_power_mode_pwr;
	else
		pmd->mode = FAST_MODE;

	if (POWER_MODE_LANE_VALID(desired_power_mode_lane))
		pmd->lane = desired_power_mode_lane;
	else if (of_property_read_u8(np, "ufs,pmd-attr-lane", &pmd->lane))
		pmd->lane = 1;

	if (POWER_MODE_GEAR_VALID(desired_power_mode_gear))
		pmd->gear = desired_power_mode_gear;
	else if (of_property_read_u8(np, "ufs,pmd-attr-gear", &pmd->gear))
		pmd->gear = 1;

	if (POWER_MODE_HS_RATE_VALID(desired_power_mode_hs_rate))
		pmd->hs_series = desired_power_mode_hs_rate;
	else
		pmd->hs_series = PA_HS_MODE_B;

	return 0;
}

static int exynos_ufs_populate_dt(struct device *dev,
				  struct exynos_ufs *ufs)
{
	struct device_node *np = dev->of_node;
	struct device_node *child_np;
	int ret;

	/* Regmap for external regions */
	ret = exynos_ufs_populate_dt_extern(dev, ufs);
	if (ret) {
		dev_err(dev, "%s populate dt-pmu\n",
			ufs_s_str_token[UFS_S_TOKEN_FAIL]);
		goto out;
	}

	/* Get exynos-evt version for featuring */
	if (of_property_read_u8(np, "evt-ver", &ufs->cal_param.evt_ver))
		ufs->cal_param.evt_ver = 0;

	/* PM QoS */
	child_np = of_get_child_by_name(np, "ufs-pm-qos");
	ufs->pm_qos_int_value = 0;
	if (!child_np)
		dev_info(dev, "No ufs-pm-qos node, not guarantee pm qos\n");
	else
		of_property_read_u32(child_np, "freq-int",
				     &ufs->pm_qos_int_value);

	/* UIC specifics */
	exynos_ufs_get_pwr_mode(np, ufs);

	ufs->cal_param.board = BRD_SMDK;
	of_property_read_u8(np, "brd-for-cal", &ufs->cal_param.board);

	/* Auto hibern8 */
	if (of_find_property(np, "samsung,support-ah8", NULL))
		ufs->ah8_ahit = FIELD_PREP(UFSHCI_AHIBERN8_TIMER_MASK, 10) |
			    FIELD_PREP(UFSHCI_AHIBERN8_SCALE_MASK, 2);
	else
		ufs->ah8_ahit = 0;

	if (ufs->ah8_ahit) {
		ufs->cal_param.support_ah8_cal = true;
		ufs->cal_param.ah8_thinern8_time = 3;
		ufs->cal_param.ah8_brefclkgatingwaittime = 1;
	} else {
		ufs->cal_param.support_ah8_cal = false;
	}
out:
	dev_info(dev, "evt version : %d, board: %d\n",
			ufs->cal_param.evt_ver, ufs->cal_param.board);
	return ret;
}

static int exynos_ufs_ioremap(struct exynos_ufs *ufs, struct platform_device *pdev)
{
	/* Indicators for logs */
	static const char *ufs_region_names[NUM_OF_UFS_MMIO_REGIONS + 1] = {
		"",			/* standard hci */
		"reg_hci",		/* exynos-specific hci */
		"reg_unipro",		/* unipro */
		"reg_ufsp",		/* ufs protector */
		"reg_phy",		/* phy */
		"reg_cport",		/* cport */
	};
	struct device *dev = &pdev->dev;
	struct resource *res;
	void **p = NULL;
	int i = 0;
	int ret = 0;

	for (i = 1, p = &ufs->reg_hci;
			i < NUM_OF_UFS_MMIO_REGIONS + 1; i++, p++) {
		res = platform_get_resource(pdev, IORESOURCE_MEM, i);
		if (!res) {
			ret = -ENOMEM;
			break;
		}
		*p = devm_ioremap_resource(dev, res);
		if (!*p) {
			ret = -ENOMEM;
			break;
		}
		dev_info(dev, "%-10s %pK\n", ufs_region_names[i], *p);
	}

	if (ret)
		dev_err(dev, "%s ioremap for %s\n",
			ufs_s_str_token[UFS_S_TOKEN_FAIL], ufs_region_names[i]);
	dev_info(dev, "\n");
	return ret;
}

static void exynos_ufs_iounmap(struct exynos_ufs *ufs)
{
	void **p;
	int i;

	for (i = 0, p = &ufs->reg_hci; i < NUM_OF_UFS_MMIO_REGIONS; i++, p++) {
		devm_iounmap(ufs->dev, *p);
		*p = NULL;
	}
}

/* sysfs to support utc, eom or whatever */
struct exynos_ufs_sysfs_attr {
	struct device_attribute dev_attr;
	int id;
};

#define UFS_SYSFS_ATTR(_name, _mode, _show, _store, _id)	\
static struct exynos_ufs_sysfs_attr ufs_attr_##_name = {	\
	.dev_attr = {						\
		.attr = { .name = #_name, .mode = _mode, },	\
		.show = _show,					\
		.store = _store,				\
	},							\
	.id = _id,						\
}

static ssize_t __sysfs_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev =
		container_of(dev, struct platform_device, dev);
	struct exynos_ufs *ufs = dev_get_platdata(&pdev->dev);
	struct exynos_ufs_sysfs_attr *ufs_attr =
		container_of(attr, struct exynos_ufs_sysfs_attr, dev_attr);

	return sysfs_emit(buf, "%u\n", ufs->params[ufs_attr->id]);
}

static ssize_t __sysfs_store(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct platform_device *pdev =
		container_of(dev, struct platform_device, dev);
	struct exynos_ufs *ufs = dev_get_platdata(&pdev->dev);
	struct exynos_ufs_sysfs_attr *ufs_attr =
		container_of(attr, struct exynos_ufs_sysfs_attr, dev_attr);

	u32 val;

	if (kstrtou32(buf, 10, &val))
		return -EINVAL;

	ufs->params[ufs_attr->id] = val;

	return count;
}

static ssize_t __sysfs_store_mon(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct platform_device *pdev =
		container_of(dev, struct platform_device, dev);
	struct exynos_ufs *ufs = dev_get_platdata(&pdev->dev);
	struct exynos_ufs_sysfs_attr *ufs_attr =
		container_of(attr, struct exynos_ufs_sysfs_attr, dev_attr);
	struct ufs_vs_handle *handle = &ufs->handle;
	u32 value;

	if (kstrtou32(buf, 10, &value))
		return -EINVAL;

	if (value & UFS_S_MON_LV1) {
		/* Trigger HCI error */
		dev_info(ufs->dev, "Interface error test\n");
		unipro_writel(handle, DBG_DL_RX_INFO_FORCE |
				DBG_DL_RX_INFO_TYPE | DBG_RX_BUFFER_OVERFLOW,
				UNIP_DBG_RX_INFO_CONTROL_DIRECT);
	} else if (value & UFS_S_MON_LV2) {
		/* Block all the interrupts */
		dev_info(ufs->dev, "Device error test\n");
		std_writel(handle, 0, REG_INTERRUPT_ENABLE);
	} else {
		dev_err(ufs->dev, "Undefined level\n");
		return -EINVAL;
	}

	ufs->params[ufs_attr->id] = value;

	return count;
}

static ssize_t __sysfs_show_h8_delay_ms(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev =
		container_of(dev, struct platform_device, dev);
	struct exynos_ufs *ufs = dev_get_platdata(&pdev->dev);

	return sysfs_emit(buf, "%lu\n", ufs->hba->clk_gating.delay_ms);
}

static ssize_t __sysfs_store_eom(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct platform_device *pdev =
		container_of(dev, struct platform_device, dev);
	struct exynos_ufs *ufs = dev_get_platdata(&pdev->dev);

	int ret;

	ret = ufs_call_cal(ufs, 0, ufs_cal_eom);
	if (ret) {
		dev_err(ufs->dev, "%s store eom data\n",
			ufs_s_str_token[UFS_S_TOKEN_FAIL]);
		return -EINVAL;
	}

	return count;
}

static ssize_t __sysfs_store_eom_res(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct platform_device *pdev =
		container_of(dev, struct platform_device, dev);
	struct exynos_ufs *ufs = dev_get_platdata(&pdev->dev);
	int value, offset;
	int ret;

	ret = sscanf(buf, "%d %d", &value, &offset);
	if (value >= ufs->num_lanes) {
		dev_err(ufs->dev, "Fail set lane to %u. Its max is %u\n", value,
				ufs->num_lanes);
		return -EINVAL;
	}

	ufs->params[UFS_SYSFS_EOM_LANE] = value;
	ufs->params[UFS_SYSFS_EOM_OFS] = offset * EOM_DEF_VREF_MAX;

	return count;
}

static ssize_t __sysfs_show_eom_res(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev =
		container_of(dev, struct platform_device, dev);
	struct exynos_ufs *ufs = dev_get_platdata(&pdev->dev);
	struct ufs_eom_result_s *p =
		ufs->cal_param.eom[ufs->params[UFS_SYSFS_EOM_LANE]] +
		ufs->params[UFS_SYSFS_EOM_OFS];
	int len = 0;
	int i;

	for (i = 0; i < EOM_DEF_VREF_MAX; i++) {
		len += snprintf(buf + len, PAGE_SIZE, "%u %u %u\n", p->v_phase,
				p->v_vref, p->v_err);
		p++;
	}

	return len;
}

static ssize_t hibern8_status_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev =
		container_of(dev, struct platform_device, dev);
	struct exynos_ufs *ufs = dev_get_platdata(&pdev->dev);
	struct ufs_vs_handle *handle = &ufs->handle;

	u32 reg = hci_readl(handle, HCI_AH8_STATE);

	return sprintf(buf, "%#x\n", reg);
}

UFS_SYSFS_ATTR(eom_version, 0444, __sysfs_show, NULL, UFS_SYSFS_EOM_VER);
UFS_SYSFS_ATTR(eom_size, 0444, __sysfs_show, NULL, UFS_SYSFS_EOM_SZ);
UFS_SYSFS_ATTR(h8_delay_ms, 0644, __sysfs_show_h8_delay_ms,
	       __sysfs_store, UFS_SYSFS_H8_D_MS);
UFS_SYSFS_ATTR(monitor, 0644, __sysfs_show, __sysfs_store_mon, UFS_SYSFS_MON);

UFS_SYSFS_ATTR(eom, 0200, NULL, __sysfs_store_eom, -1);
UFS_SYSFS_ATTR(eom_res, 0644, __sysfs_show_eom_res, __sysfs_store_eom_res, -1);
UFS_SYSFS_ATTR(hibern8_status, 0444, hibern8_status_show, NULL, -1);

static struct attribute *exynos_ufs_sysfs_attrs[] = {
	&ufs_attr_eom_version.dev_attr.attr,
	&ufs_attr_eom_size.dev_attr.attr,
	&ufs_attr_h8_delay_ms.dev_attr.attr,
	&ufs_attr_monitor.dev_attr.attr,

	/* not use a field named param below */
	&ufs_attr_eom.dev_attr.attr,
	&ufs_attr_eom_res.dev_attr.attr,
	&ufs_attr_hibern8_status.dev_attr.attr,
	NULL,
};

static const struct attribute_group exynos_ufs_group = {
	.name = "exynos-ufs",
	.attrs = exynos_ufs_sysfs_attrs,
};

static int exynos_ufs_sysfs_init(struct exynos_ufs *ufs)
{
	int error = -ENOMEM;
	int i;
	struct ufs_eom_result_s *p;

	/* allocate memory for eom per lane */
	for (i = 0; i < MAX_LANE; i++) {
		ufs->cal_param.eom[i] =
			devm_kcalloc(ufs->dev, EOM_MAX_SIZE,
					sizeof(struct ufs_eom_result_s), GFP_KERNEL);
		p = ufs->cal_param.eom[i];
		if (!p) {
			dev_err(ufs->dev, "%s allocate eom data\n",
					ufs_s_str_token[UFS_S_TOKEN_FAIL]);
			goto fail_mem;
		}
	}

	/*
	 * create an attributes group with a path of /sys/devices/platform/..
	 */
	error = sysfs_create_group(&ufs->dev->kobj, &exynos_ufs_group);
	if (error) {
		dev_err(ufs->dev, "%s create sysfs group: %d\n",
				ufs_s_str_token[UFS_S_TOKEN_FAIL], error);
		goto fail_mem;
	}

	/*
	 * Set sysfs params by default. The values could change or
	 * initial configuration could be done elsewhere in the future.
	 *
	 * As for eom_version, you have to move it to store a value
	 * from device tree when eom code is revised, even though I expect
	 * it's not gonna to happen.
	 */
	ufs->params[UFS_SYSFS_EOM_VER] = 0;
	ufs->params[UFS_SYSFS_MON] = 0;
	ufs->params[UFS_SYSFS_H8_D_MS] = 4;

	return 0;

fail_mem:
	for (i = 0; i < MAX_LANE; i++) {
		if (ufs->cal_param.eom[i])
			devm_kfree(ufs->dev, ufs->cal_param.eom[i]);
		ufs->cal_param.eom[i] = NULL;
	}
	return error;
}

static void exynos_ufs_sysfs_exit(struct exynos_ufs *ufs)
{
	int i;

	sysfs_remove_group(&ufs->dev->kobj, &exynos_ufs_group);
	for (i = 0; i < MAX_LANE; i++) {
		devm_kfree(ufs->dev, ufs->cal_param.eom[i]);
		ufs->cal_param.eom[i] = NULL;
	}
}

static u64 exynos_ufs_dma_mask = DMA_BIT_MASK(32);

static int exynos_ufs_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct exynos_ufs *ufs;
	int ret;

	dev_info(dev, "%s: start\n", __func__);
	dev_info(dev, "===============================\n");

	/* allocate memory */
	ufs = devm_kzalloc(dev, sizeof(*ufs), GFP_KERNEL);
	if (!ufs) {
		ret = -ENOMEM;
		goto out;
	}
	ufs->dev = dev;
	dev->platform_data = ufs;
	dev->dma_mask = &exynos_ufs_dma_mask;

	/* remap regions */
	ret = exynos_ufs_ioremap(ufs, pdev);
	if (ret)
		goto free_exynos_ufs;
	ufs_map_vs_regions(ufs);

	/* populate device tree nodes */
	ret = exynos_ufs_populate_dt(dev, ufs);
	if (ret) {
		dev_err(dev, "%s get dt info.\n",
			ufs_s_str_token[UFS_S_TOKEN_FAIL]);
		goto iounmap;
	}

	/* init cal */
	ufs->cal_param.handle = &ufs->handle;
	ret = ufs_call_cal(ufs, 1, ufs_cal_init);
	if (ret)
		goto iounmap;
	dev_info(dev, "===============================\n");

	/* idle ip nofification for SICD, disable by default */
#if IS_ENABLED(CONFIG_EXYNOS_CPUPM)
	ufs->idle_ip_index = exynos_get_idle_ip_index(dev_name(ufs->dev));
#endif
	__sicd_ctrl(ufs, true);

	/* register pm qos knobs */
#if defined(CONFIG_EXYNOS_PM_QOS) || defined(CONFIG_EXYNOS_PM_QOS_MODULE)
	exynos_pm_qos_add_request(&ufs->pm_qos_int,
				PM_QOS_DEVICE_THROUGHPUT, 0);
#endif

	/* init dbg */
	ret = exynos_ufs_init_dbg(&ufs->handle, dev);
	if (ret)
		goto remove_qos_request;
	spin_lock_init(&ufs->dbg_lock);

	/* store ufs host symbols to analyse later */
	ufs->id = ufs_host_index++;
	ufs_host_backup[ufs->id] = ufs;

	/* init sysfs */
	exynos_ufs_sysfs_init(ufs);

	/* init specific states */
	ufs->h_state = H_DISABLED;
	ufs->c_state = C_OFF;

	/* register vendor hooks */
	exynos_ufs_register_vendor_hooks();

	/* go to core driver through the glue driver */
	ret = ufshcd_pltfrm_init(pdev, &exynos_ufs_ops);
	if (ret)
		goto sysfs_exit;

out:
	return ret;

sysfs_exit:
	exynos_ufs_sysfs_exit(ufs);

remove_qos_request:
#if defined(CONFIG_EXYNOS_PM_QOS) || defined(CONFIG_EXYNOS_PM_QOS_MODULE)
	exynos_pm_qos_remove_request(&ufs->pm_qos_int);
#endif

iounmap:
	exynos_ufs_iounmap(ufs);

free_exynos_ufs:
	devm_kfree(dev, ufs);
	return ret;
}

static int exynos_ufs_remove(struct platform_device *pdev)
{
	struct exynos_ufs *ufs = dev_get_platdata(&pdev->dev);
	struct ufs_hba *hba =  platform_get_drvdata(pdev);

	ufs_host_index--;

	sysfs_remove_group(&ufs->dev->kobj, &exynos_ufs_group);
	pixel_exit(hba);

	disable_irq(hba->irq);
	ufshcd_remove(hba);

	exynos_pm_qos_remove_request(&ufs->pm_qos_int);

	exynos_ufs_ctrl_phy_pwr(ufs, false);

	return 0;
}

static void exynos_ufs_shutdown(struct platform_device *pdev)
{
	ufshcd_shutdown((struct ufs_hba *)platform_get_drvdata(pdev));
}

static const struct dev_pm_ops exynos_ufs_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(ufshcd_system_suspend, ufshcd_system_resume)
};

static const struct of_device_id exynos_ufs_match[] = {
	{ .compatible = "samsung,exynos-ufs", },
	{},
};
MODULE_DEVICE_TABLE(of, exynos_ufs_match);

static struct platform_driver exynos_ufs_driver = {
	.driver = {
		.name = "exynos-ufs",
		.owner = THIS_MODULE,
		.pm = &exynos_ufs_dev_pm_ops,
		.of_match_table = exynos_ufs_match,
		.suppress_bind_attrs = true,
	},
	.probe = exynos_ufs_probe,
	.remove = exynos_ufs_remove,
	.shutdown = exynos_ufs_shutdown,
};

module_platform_driver(exynos_ufs_driver);
MODULE_DESCRIPTION("Exynos Specific UFSHCI driver");
MODULE_AUTHOR("Seungwon Jeon <tgih.jun@samsung.com>");
MODULE_AUTHOR("Kiwoong Kim <kwmad.kim@samsung.com>");
MODULE_LICENSE("GPL");
