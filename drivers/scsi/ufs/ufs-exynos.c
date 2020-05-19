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
#include "ufshcd.h"
#include "ufshci.h"
#include "unipro.h"
#include "ufshcd-pltfrm.h"
#include "ufs_quirks.h"
#include "ufs-exynos.h"
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/spinlock.h>
#include <soc/samsung/exynos-pmu-if.h>

/* Performance */
#include "ufs-exynos-perf.h"

#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#endif

#define IS_C_STATE_ON(h) ((h)->c_state == C_ON)
#define PRINT_STATES(h)						\
	dev_err((h)->dev, "%s: prev h_state %d, cur c_state %d\n",	\
				__func__, (h)->h_state, (h)->c_state)

static struct exynos_ufs *ufs_host_backup[1];
static int ufs_host_index;
static const char *res_token[2] = {
	"passes",
	"fails",
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
static inline int ufs_init_cal(struct exynos_ufs *ufs, int idx,
			       struct platform_device *pdev)
{
	int ret = 0;
	struct ufs_cal_param *p = &ufs->cal_param;

	p->host = ufs;
	p->board = 0;	/* ken: need a dt node for board */
	ret = ufs_cal_init(p, idx);
	if (ret != UFS_CAL_NO_ERROR) {
		dev_err(ufs->dev, "ufs_init_cal = %d!!!\n", ret);
		return -EPERM;
	}

	return 0;
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
	if (!active_rx_lane || !active_tx_lane ||
	    active_rx_lane != active_tx_lane) {
		dev_err(hba->dev, "%s: invalid host active lanes. rx=%d, tx=%d\n",
			__func__, active_rx_lane, active_tx_lane);
		WARN_ON(1);
	}

	p->active_tx_lane = (u8)active_tx_lane;
	p->active_rx_lane = (u8)active_rx_lane;

	dev_info(ufs->dev, "PA_ActiveTxDataLanes(%d), PA_ActiveRxDataLanes(%d)\n",
		 active_tx_lane, active_rx_lane);
}

static void exynos_ufs_update_max_gear(struct ufs_hba *hba)
{
	struct exynos_ufs *ufs = to_exynos_ufs(hba);
	struct uic_pwr_mode *pmd = &ufs->req_pmd_parm;
	struct ufs_cal_param *p = &ufs->cal_param;
	u32 max_rx_hs_gear = 0;

	max_rx_hs_gear = unipro_readl(&ufs->handle, UNIP_PA_MAXRXHSGEAR);

	p->max_gear = min_t(u8, max_rx_hs_gear, pmd->gear);

	dev_info(ufs->dev, "max_gear(%d), PA_MaxRxHSGear(%d)\n",
		 p->max_gear, max_rx_hs_gear);
}

static inline int ufs_pre_link(struct exynos_ufs *ufs)
{
	int ret = 0;
	struct ufs_cal_param *p = &ufs->cal_param;

	p->mclk_rate = ufs->mclk_rate;
	/*
	 * assume that unipro clock is supposed to be unsigned
	 * and less than 1GHz.
	 */
	if (p->mclk_rate >= 1000000000U)
		WARN_ON(1);

	p->available_lane = ufs->num_rx_lanes;

	ret = ufs_cal_pre_link(p);
	if (ret != UFS_CAL_NO_ERROR) {
		dev_err(ufs->dev, "%s: %d!!!\n", __func__, ret);
		return -EPERM;
	}

	return 0;
}

static inline int ufs_post_link(struct exynos_ufs *ufs)
{
	int ret = 0;

	/* update max gear after link*/
	exynos_ufs_update_max_gear(ufs->hba);

	ret = ufs_cal_post_link(&ufs->cal_param);
	if (ret != UFS_CAL_NO_ERROR) {
		dev_err(ufs->dev, "%s: %d!!!\n", __func__, ret);
		return -EPERM;
	}

	return 0;
}

static inline int ufs_pre_gear_change(struct exynos_ufs *ufs,
				      struct uic_pwr_mode *pmd)
{
	struct ufs_cal_param *p = &ufs->cal_param;
	int ret = 0;

	p->pmd = pmd;
	ret = ufs_cal_pre_pmc(p);
	if (ret != UFS_CAL_NO_ERROR) {
		dev_err(ufs->dev, "%s: %d!!!\n", __func__, ret);
		return -EPERM;
	}

	return 0;
}

static inline int ufs_post_gear_change(struct exynos_ufs *ufs)
{
	int ret = 0;

	ret = ufs_cal_post_pmc(&ufs->cal_param);
	if (ret != UFS_CAL_NO_ERROR) {
		dev_err(ufs->dev, "%s: %d!!!\n", __func__, ret);
		return -EPERM;
	}

	return 0;
}

static inline int ufs_post_h8_enter(struct exynos_ufs *ufs)
{
	int ret = 0;

	ret = ufs_cal_post_h8_enter(&ufs->cal_param);
	if (ret != UFS_CAL_NO_ERROR) {
		dev_err(ufs->dev, "%s: %d!!!\n", __func__, ret);
		return -EPERM;
	}

	return 0;
}

static inline int ufs_pre_h8_exit(struct exynos_ufs *ufs)
{
	int ret = 0;

	ret = ufs_cal_pre_h8_exit(&ufs->cal_param);
	if (ret != UFS_CAL_NO_ERROR) {
		dev_err(ufs->dev, "%s: %d!!!\n", __func__, ret);
		return -EPERM;
	}

	return 0;
}

/* Adaptor for UFS CAL */
void ufs_lld_dme_set(void *h, u32 addr, u32 val)
{
	ufshcd_dme_set(((struct exynos_ufs *)h)->hba, addr, val);
}

void ufs_lld_dme_get(void *h, u32 addr, u32 *val)
{
	ufshcd_dme_get(((struct exynos_ufs *)h)->hba, addr, val);
}

void ufs_lld_dme_peer_set(void *h, u32 addr, u32 val)
{
	ufshcd_dme_peer_set(((struct exynos_ufs *)h)->hba, addr, val);
}

void ufs_lld_pma_write(void *h, u32 val, u32 addr)
{
	pma_writel(&((struct exynos_ufs *)h)->handle, val, addr);
}

u32 ufs_lld_pma_read(void *h, u32 addr)
{
	return pma_readl(&((struct exynos_ufs *)h)->handle, addr);
}

void ufs_lld_unipro_write(void *h, u32 val, u32 addr)
{
	unipro_writel(&((struct exynos_ufs *)h)->handle, val, addr);
}

void ufs_lld_udelay(u32 val)
{
	udelay(val);
}

void ufs_lld_usleep_delay(u32 min, u32 max)
{
	usleep_range(min, max);
}

unsigned long ufs_lld_get_time_count(unsigned long offset)
{
	return jiffies;
}

unsigned long ufs_lld_calc_timeout(const unsigned int ms)
{
	return msecs_to_jiffies(ms);
}

static inline void exynos_ufs_ctrl_phy_pwr(struct exynos_ufs *ufs, bool en)
{
	struct ext_cxt *cxt = &ufs->cxt_phy_iso;

	exynos_pmu_update(cxt->offset, cxt->mask, (en ? 1 : 0) ? cxt->val : 0);
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
	struct ufs_cal_param *p = &ufs->cal_param;

	/* update lane variable after link */
	ufs->num_rx_lanes = pwr_max->lane_rx;
	ufs->num_tx_lanes = pwr_max->lane_tx;

	p->connected_rx_lane = pwr_max->lane_rx;
	p->connected_tx_lane = pwr_max->lane_tx;

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

	/* performance */
	ufs_perf_reset(ufs->perf, true);
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

static void exynos_ufs_set_features(struct ufs_hba *hba)
{
	/* caps */
	hba->caps = UFSHCD_CAP_CLK_GATING |
			UFSHCD_CAP_HIBERN8_WITH_CLK_GATING |
			UFSHCD_CAP_INTR_AGGR;

	/* quirks of common driver */
	hba->quirks = UFSHCD_QUIRK_PRDT_BYTE_GRAN |
			UFSHCI_QUIRK_SKIP_RESET_INTR_AGGR |
			UFSHCI_QUIRK_BROKEN_REQ_LIST_CLR |
			UFSHCD_QUIRK_BROKEN_CRYPTO |
			UFSHCD_QUIRK_BROKEN_OCS_FATAL_ERROR;

	hba->dev_quirks &= ~(UFS_DEVICE_QUIRK_RECOVERY_FROM_DL_NAC_ERRORS);
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

	/* idle ip nofification for SICD, disable by default */
#if defined(CONFIG_ARM64_EXYNOS_CPUIDLE)
	ufs->idle_ip_index = exynos_get_idle_ip_index(dev_name(&pdev->dev));
	exynos_update_ip_idle_status(ufs->idle_ip_index, 0);
#endif

	/* to read standard hci registers */
	ufs->handle.std = hba->mmio_base;

	/* get some clock sources and debug information structures */
	ret = exynos_ufs_get_clks(hba);
	if (ret)
		return ret;

	/* set features, such as caps or quirks */
	exynos_ufs_set_features(hba);

	return 0;
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
	int ret = 0;

	if (on) {
		if (notify == PRE_CHANGE) {
			/* Clear for SICD */
#if defined(CONFIG_ARM64_EXYNOS_CPUIDLE)
			exynos_update_ip_idle_status(ufs->idle_ip_index, 0);
#endif
		} else {
			/* PM Qos hold for stability */
#ifdef PM_QOS_DEVICE_THROUGHPUT
			pm_qos_update_request(&ufs->pm_qos_int,
					      ufs->pm_qos_int_value);
#endif
			ufs->c_state = C_ON;
		}
	} else {
		if (notify == PRE_CHANGE) {
			ufs->c_state = C_OFF;

			/* reset perf context to start again */
			ufs_perf_reset(ufs->perf, false);
			/* PM Qos Release for stability */
#ifdef PM_QOS_DEVICE_THROUGHPUT
			pm_qos_update_request(&ufs->pm_qos_int, 0);
#endif
		} else {
			/* Set for SICD */
#if defined(CONFIG_ARM64_EXYNOS_CPUIDLE)
			exynos_update_ip_idle_status(ufs->idle_ip_index, 1);
#endif
		}
	}

	return ret;
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

	ufs->num_rx_lanes = ufs->available_lane_rx;
	ufs->num_tx_lanes = ufs->available_lane_tx;

	ret = 0;
out:
	return ret;
}

static void exynos_ufs_override_hba_params(struct ufs_hba *hba)
{
	hba->clk_gating.delay_ms = 12;
	hba->spm_lvl = UFS_PM_LVL_5;
}

static int exynos_ufs_hce_enable_notify(struct ufs_hba *hba,
					enum ufs_notify_change_status notify)
{
	struct exynos_ufs *ufs = to_exynos_ufs(hba);
	int ret = 0;

	PRINT_STATES(ufs);

	switch (notify) {
	case PRE_CHANGE:
		/* override some parameters from core driver */
		exynos_ufs_override_hba_params(hba);

		/*
		 * This function is called in ufshcd_hba_enable,
		 * maybe boot, wake-up or link start-up failure cases.
		 * To start safely, reset of entire logics of host
		 * is required
		 */
		exynos_ufs_init_host(hba);

		/* device reset */
		exynos_ufs_dev_hw_reset(hba);
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
	int ret = 0;

	switch (notify) {
	case PRE_CHANGE:
		if (!IS_C_STATE_ON(ufs) || ufs->h_state != H_RESET)
			PRINT_STATES(ufs);

		/* hci */
		hci_writel(&ufs->handle, DFES_ERR_EN | DFES_DEF_DL_ERRS,
			   HCI_ERROR_EN_DL_LAYER);
		hci_writel(&ufs->handle, DFES_ERR_EN | DFES_DEF_N_ERRS,
			   HCI_ERROR_EN_N_LAYER);
		hci_writel(&ufs->handle, DFES_ERR_EN | DFES_DEF_T_ERRS,
			   HCI_ERROR_EN_T_LAYER);

		ufs->mclk_rate = clk_get_rate(ufs->clk_unipro);

		ret = ufs_pre_link(ufs);
		break;
	case POST_CHANGE:
		/* UIC configuration table after link startup */
		ret = ufs_post_link(ufs);

		/* print link start-up result */
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
		dev_info(ufs->dev, "UFS device initialized\n");

		if (!IS_C_STATE_ON(ufs) || ufs->h_state != H_REQ_BUSY)
			PRINT_STATES(ufs);

		/* Set PMC parameters to be requested */
		exynos_ufs_init_pmc_req(hba, pwr_max, pwr_req);

		/* UIC configuration table before power mode change */
		ret = ufs_pre_gear_change(ufs, act_pmd);

		break;
	case POST_CHANGE:
		/* update active lanes after pmc */
		exynos_ufs_update_active_lanes(hba);

		/* UIC configuration table after power mode change */
		ret = ufs_post_gear_change(ufs);

		dev_info(ufs->dev,
			 "Power mode change(%d): M(%d)G(%d)L(%d)HS-series(%d)\n",
			 ret, act_pmd->mode, act_pmd->gear,
			 act_pmd->lane, act_pmd->hs_series);
		/*
		 * print gear change result.
		 * Exynos driver always considers gear change to
		 * HS-B and fast mode.
		 */
		if (ufs->req_pmd_parm.mode == FAST_MODE &&
		    ufs->req_pmd_parm.hs_series == PA_HS_MODE_B)
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

	if (!IS_C_STATE_ON(ufs) ||
	    (ufs->h_state != H_LINK_BOOST &&
	     ufs->h_state != H_TM_BUSY &&
	     ufs->h_state != H_REQ_BUSY))
		PRINT_STATES(ufs);

	type =  hci_readl(&ufs->handle, HCI_UTMRL_NEXUS_TYPE);

	switch (tm_func) {
	case UFS_ABORT_TASK:
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

	if (cmd == UIC_CMD_DME_HIBER_ENTER) {
		if (!IS_C_STATE_ON(ufs) ||
		    (ufs->h_state != H_LINK_UP &&
		     ufs->h_state != H_REQ_BUSY &&
		     ufs->h_state != H_LINK_BOOST))
			PRINT_STATES(ufs);

		if (notify == PRE_CHANGE) {
			;
		} else {
			/* BG/SQ off */
			ufs_post_h8_enter(ufs);
			/* Internal clock off */
			exynos_ufs_gate_clk(ufs, true);

			ufs->h_state_prev = ufs->h_state;
			ufs->h_state = H_HIBERN8;
		}
	} else {
		if (notify == PRE_CHANGE) {
			ufs->h_state = ufs->h_state_prev;

			/* Internal clock on */
			exynos_ufs_gate_clk(ufs, false);
			/* BG/SQ on */
			ufs_pre_h8_exit(ufs);
		} else {
			;
		}
	}
}

static int __exynos_ufs_suspend(struct ufs_hba *hba, enum ufs_pm_op pm_op)
{
	struct exynos_ufs *ufs = to_exynos_ufs(hba);

	if (!IS_C_STATE_ON(ufs) ||
	    ufs->h_state != H_HIBERN8)
		PRINT_STATES(ufs);

#ifdef PM_QOS_DEVICE_THROUGHPUT
	pm_qos_update_request(&ufs->pm_qos_int, 0);
#endif

	hci_writel(&ufs->handle, 0 << 0, HCI_GPIO_OUT);

	exynos_ufs_ctrl_phy_pwr(ufs, false);

	ufs->h_state = H_SUSPEND;
	return 0;
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
};

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
		dev_err(dev, "failed to set cxt(%s) val\n", name);
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
		dev_err(dev, "%s: %u: fail to get %s\n",
			__func__, __LINE__, ufs_pmu_token);
		goto out;
	}

	/* others */
	for (i = 0, reg = &ufs->regmap_sys, cxt = &ufs->cxt_iocc;
			i < EXT_BLK_MAX; i++, reg++, cxt++) {
		/* look up phandle for external regions */
		*reg = syscon_regmap_lookup_by_phandle(np, ufs_ext_blks[i][0]);
		if (IS_ERR(*reg)) {
			dev_err(dev, "%s: %u: fail to find %s\n",
				__func__, __LINE__, ufs_ext_blks[i][0]);
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
			dev_err(dev, "%s: %u: fail to get %s\n",
				__func__, __LINE__,
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

	pmd->mode = FAST_MODE;

	if (of_property_read_u8(np, "ufs,pmd-attr-lane", &pmd->lane))
		pmd->lane = 1;

	if (of_property_read_u8(np, "ufs,pmd-attr-gear", &pmd->gear))
		pmd->gear = 1;

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
		dev_err(dev, "failed to populate dt-pmu\n");
		goto out;
	}

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

	ufs->cal_param.board = 0;
	of_property_read_u8(np, "brd-for-cal", &ufs->cal_param.board);
out:
	return ret;
}

static u64 exynos_ufs_dma_mask = DMA_BIT_MASK(32);

/*
 * DEBUG FS FUNCTIONS
 *
 * These are to make UFS driver run differently to get some
 * information and let the driver behave in specific ways on purpose.
 *
 * Initially, it's used to check if ufshcd_err_handler and SCSI error
 * handler callbacks work properly.
 */
#ifdef CONFIG_DEBUG_FS
static int exynos_ufs_debug_show(struct seq_file *s, void *unused)
{
	struct exynos_ufs *ufs = s->private;

	dev_info(ufs->dev, "0x%016lx\n", ufs->debug.monitor);

	return 0;
}

static int exynos_ufs_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, exynos_ufs_debug_show, inode->i_private);
}

static ssize_t exynos_ufs_debug_write(struct file *file,
				      const char __user *ubuf,
				      size_t count, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct exynos_ufs *ufs = s->private;
	unsigned long value;
	char buf[32];
	struct ufs_vs_handle *handle = &ufs->handle;

	memset(buf, 0, sizeof(buf));
	if (copy_from_user(&buf, ubuf, min_t(size_t, sizeof(buf) - 1, count))) {
		count = 0;
		goto end;
	}

	if (kstrtoul(buf, 0, &value)) {
		count = 0;
		goto end;
	}

	if (value & UFSHCD_DEBUG_LEVEL1) {
		/* Trigger HCI error */
		dev_info(ufs->dev, "Interface error test\n");
		unipro_writel(handle, (0x1F << 24 | 0x14 << 18),
			      UNIP_DBG_RX_INFO_CONTROL_DIRECT);
	} else if (value & UFSHCD_DEBUG_LEVEL2) {
		/* Block all the interrupts */
		dev_info(ufs->dev, "Device error test\n");
		std_writel(handle, 0, REG_INTERRUPT_ENABLE);
	} else {
		dev_err(ufs->dev, "Undefined debugfs level\n");
	}

	ufs->debug.monitor = value;

end:
	return count;
}

static const struct file_operations exynos_ufs_debug_fops = {
	.open			= exynos_ufs_debug_open,
	.write			= exynos_ufs_debug_write,
	.read			= seq_read,
	.llseek			= seq_lseek,
	.release		= single_release,
};

static void exynos_ufs_debugfs_init(struct exynos_ufs *ufs)
{
	struct dentry *file;

	ufs->debug.root = debugfs_create_dir("ufs", NULL);
	if (IS_ERR(ufs->debug.root)) {
		dev_err(ufs->dev, "debugfs is not enabled\n");
		return;
	} else if (!ufs->debug.root) {
		dev_err(ufs->dev, "Can't create debugfs root\n");
		return;
	}

	file = debugfs_create_file("monitor", 0644,
				   ufs->debug.root, ufs,
				   &exynos_ufs_debug_fops);
	if (!file) {
		dev_dbg(ufs->dev, "Can't create debugfs monitor\n");
		debugfs_remove_recursive(ufs->debug.root);
		ufs->debug.root = NULL;
	}
}

static void exynos_ufs_debugfs_exit(struct exynos_ufs *ufs)
{
	debugfs_remove_recursive(ufs->debug.root);
	ufs->debug.root = NULL;
}
#else
static void exynos_ufs_debugfs_init(struct exynos_ufs *ufs)
{
}

static void exynos_ufs_debugfs_exit(struct exynos_ufs *ufs)
{
}
#endif

static int exynos_ufs_ioremap(struct exynos_ufs *ufs,
			      struct platform_device *pdev)
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
		dev_info(dev, "%-10s 0x%llx\n", ufs_region_names[i], *p);
	}

	if (ret)
		dev_err(dev, "fail to ioremap for %s, 0x%llx\n",
			ufs_region_names[i]);
	dev_info(dev, "\n");
	return ret;
}

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
		goto out;
	ufs_map_vs_regions(ufs);

	/* populate device tree nodes */
	ret = exynos_ufs_populate_dt(dev, ufs);
	if (ret) {
		dev_err(dev, "failed to get dt info.\n");
		return ret;
	}

	/* init io perf stat, need an identifier later */
	if (!ufs_perf_init(&ufs->perf, dev))
		dev_err(dev, "Not enable UFS performance mode\n");

	/* init cal */
	ret = ufs_init_cal(ufs, ufs_host_index, pdev);
	if (ret)
		return ret;
	dev_info(dev, "===============================\n");

	/* register pm qos knobs */
#ifdef PM_QOS_DEVICE_THROUGHPUT
	pm_qos_add_request(&ufs->pm_qos_int, PM_QOS_DEVICE_THROUGHPUT, 0);
#endif

	/* init dbg */
	ret = exynos_ufs_init_dbg(&ufs->handle);
	if (ret)
		return ret;
	spin_lock_init(&ufs->dbg_lock);

	/* init debug fs */
	exynos_ufs_debugfs_init(ufs);

	/* store ufs host symbols to analyse later */
	ufs->id = ufs_host_index++;
	ufs_host_backup[ufs->id] = ufs;

	/* init specific states */
	ufs->h_state = H_DISABLED;
	ufs->c_state = C_OFF;

	/* go to core driver through the glue driver */
	ret = ufshcd_pltfrm_init(pdev, &exynos_ufs_ops);
out:
	return ret;
}

static int exynos_ufs_remove(struct platform_device *pdev)
{
	struct exynos_ufs *ufs = dev_get_platdata(&pdev->dev);
	struct ufs_hba *hba =  platform_get_drvdata(pdev);

	ufs_host_index--;

	exynos_ufs_debugfs_exit(ufs);

	disable_irq(hba->irq);
	ufshcd_remove(hba);

#ifdef PM_QOS_DEVICE_THROUGHPUT
	pm_qos_remove_request(&ufs->pm_qos_int);
#endif

	/* performance */
	ufs_perf_exit(ufs->perf);

	exynos_ufs_ctrl_phy_pwr(ufs, false);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int exynos_ufs_suspend(struct device *dev)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);

	return ufshcd_system_suspend(hba);
}

static int exynos_ufs_resume(struct device *dev)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);

	return ufshcd_system_resume(hba);
}
#else
#define exynos_ufs_suspend	NULL
#define exynos_ufs_resume	NULL
#endif /* CONFIG_PM_SLEEP */

static void exynos_ufs_shutdown(struct platform_device *pdev)
{
	struct exynos_ufs *ufs = dev_get_platdata(&pdev->dev);

	ufshcd_shutdown((struct ufs_hba *)platform_get_drvdata(pdev));
	ufs_perf_exit(ufs->perf);
}

static const struct dev_pm_ops exynos_ufs_dev_pm_ops = {
	.suspend		= exynos_ufs_suspend,
	.resume			= exynos_ufs_resume,
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
