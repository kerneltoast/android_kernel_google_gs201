/*
 * drivers/media/platform/exynos/mfc/mfc_core_pm.c
 *
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/pm_runtime.h>
#include <linux/clk.h>
#include <linux/soc/samsung/exynos-smc.h>

#include "mfc_core_qos.h"
#include "mfc_core_pm.h"
#include "mfc_perf_measure.h"

#include "mfc_core_hw_reg_api.h"

#include "mfc_sync.h"

void mfc_core_pm_init(struct mfc_core *core)
{
	spin_lock_init(&core->pm.clklock);
	atomic_set(&core->pm.pwr_ref, 0);
	atomic_set(&core->pm.protect_ref, 0);
	atomic_set(&core->clk_ref, 0);

	core->pm.device = core->device;
	core->pm.clock_on_steps = 0;
	core->pm.clock_off_steps = 0;
	pm_runtime_enable(core->pm.device);
}

void mfc_core_pm_final(struct mfc_core *core)
{
	pm_runtime_disable(core->pm.device);
}

void mfc_core_protection_on(struct mfc_core *core)
{
#if IS_ENABLED(CONFIG_EXYNOS_CONTENT_PATH_PROTECTION)
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&core->pm.clklock, flags);
	mfc_core_debug(3, "Begin: enable protection\n");

	if (atomic_read(&core->pm.protect_ref)) {
		mfc_core_err("IP protection is already enabled\n");
		MFC_TRACE_CORE("already protected\n");
		spin_unlock_irqrestore(&core->pm.clklock, flags);
		return;
	}

	ret = exynos_smc(SMC_PROTECTION_SET, 0,
			core->id * PROT_MFC1, SMC_PROTECTION_ENABLE);
	if (ret != DRMDRV_OK) {
		snprintf(core->crash_info, MFC_CRASH_INFO_LEN,
			"Protection Enable failed! ret(%u)\n", ret);
		mfc_core_err("%s", core->crash_info);
		call_dop(core, dump_and_stop_debug_mode, core);
		spin_unlock_irqrestore(&core->pm.clklock, flags);
		return;
	}

	atomic_inc(&core->pm.protect_ref);
	MFC_TRACE_CORE("protection\n");
	mfc_core_debug(3, "End: enable protection\n");
	spin_unlock_irqrestore(&core->pm.clklock, flags);

	/* RISC_ON is required because the MFC was reset */
	if (core->dev->pdata->security_ctrl) {
		mfc_core_risc_on(core);
		mfc_core_debug(2, "Will now wait for completion of firmware transfer\n");
		if (mfc_wait_for_done_core(core, MFC_REG_R2H_CMD_FW_STATUS_RET)) {
			mfc_core_err("Failed to RISC_ON\n");
			mfc_core_clean_dev_int_flags(core);
			call_dop(core, dump_and_stop_always, core);
		}
	} else {
		mfc_core_set_risc_base_addr(core, MFCBUF_DRM);
	}
#endif
}

void mfc_core_protection_off(struct mfc_core *core)
{
#if IS_ENABLED(CONFIG_EXYNOS_CONTENT_PATH_PROTECTION)
	unsigned long flags;
	int ret = 0;
	/*
	 * After clock off the protection disable should be
	 * because the MFC core can continuously run during clock on
	 */
	mfc_core_debug(3, "Begin: disable protection\n");
	spin_lock_irqsave(&core->pm.clklock, flags);

	if (!atomic_read(&core->pm.protect_ref)) {
		mfc_core_err("IP protection is already disabled\n");
		MFC_TRACE_CORE("already un-protected\n");
		spin_unlock_irqrestore(&core->pm.clklock, flags);
		return;
	}

	ret = exynos_smc(SMC_PROTECTION_SET, 0,
			core->id * PROT_MFC1, SMC_PROTECTION_DISABLE);
	if (ret != DRMDRV_OK) {
		snprintf(core->crash_info, MFC_CRASH_INFO_LEN,
			"Protection Disable failed! ret(%u)\n", ret);
		mfc_core_err("%s", core->crash_info);
		call_dop(core, dump_and_stop_debug_mode, core);
		spin_unlock_irqrestore(&core->pm.clklock, flags);
		return;
	}

	atomic_dec(&core->pm.protect_ref);
	mfc_core_debug(3, "End: disable protection\n");
	MFC_TRACE_CORE("un-protection\n");
	spin_unlock_irqrestore(&core->pm.clklock, flags);

	/* Normal base address setting is required because the MFC was reset */
	mfc_core_set_risc_base_addr(core, MFCBUF_NORMAL);
#endif
}

int mfc_core_pm_clock_on(struct mfc_core *core)
{
	int ret = 0;
	int state;

	core->pm.clock_on_steps = 1;
	state = atomic_read(&core->clk_ref);

	/*
	 * When the clock is enabled, the MFC core can run immediately.
	 * So the base addr and protection should be applied before clock enable.
	 * The MFC and TZPC SFR are in APB clock domain and it is accessible
	 * through Q-CH even if clock off.
	 * The sequence for switching normal to drm is following
	 * cache flush (cmd 12) -> clock off -> set DRM base addr
	 * -> IP Protection enable -> clock on
	 */
	core->pm.clock_on_steps |= 0x1 << 1;
	if (core->pm.base_type != MFCBUF_INVALID) {
		/*
		 * There is no place to set core->pm.base_type,
		 * so it is always MFCBUF_INVALID now.
		 * When necessary later, you can set the bse_type.
		 */
		core->pm.clock_on_steps |= 0x1 << 2;
		ret = mfc_core_wait_pending(core);
		if (ret != 0) {
			snprintf(core->crash_info, MFC_CRASH_INFO_LEN,
				"pending wait failed (%d)\n", ret);
			mfc_core_err("%s", core->crash_info);
			call_dop(core, dump_and_stop_debug_mode, core);
			return ret;
		}
		core->pm.clock_on_steps |= 0x1 << 3;
		mfc_core_set_risc_base_addr(core, core->pm.base_type);
	}

	core->pm.clock_on_steps |= 0x1 << 4;
	if (!IS_ERR(core->pm.clock)) {
		ret = clk_enable(core->pm.clock);
		if (ret < 0)
			mfc_core_err("clk_enable failed (%d)\n", ret);
	}

	core->pm.clock_on_steps |= 0x1 << 5;
	atomic_inc_return(&core->clk_ref);

	core->pm.clock_on_steps |= 0x1 << 6;
	state = atomic_read(&core->clk_ref);
	mfc_core_debug(2, "+ %d\n", state);
	MFC_TRACE_LOG_CORE("c+%d", state);

	return 0;
}

void mfc_core_pm_clock_off(struct mfc_core *core)
{
	int state;

	core->pm.clock_off_steps = 1;
	atomic_dec_return(&core->clk_ref);

	core->pm.clock_off_steps |= 0x1 << 1;
	state = atomic_read(&core->clk_ref);
	if (state < 0) {
		mfc_core_err("Clock state is wrong(%d)\n", state);
		atomic_set(&core->clk_ref, 0);
		core->pm.clock_off_steps |= 0x1 << 2;
		MFC_TRACE_CORE("** clock_off wrong: ref state(%d)\n", atomic_read(&core->clk_ref));
	} else {
		core->pm.clock_off_steps |= 0x1 << 3;
		if (!IS_ERR(core->pm.clock)) {
			clk_disable(core->pm.clock);
			core->pm.clock_off_steps |= 0x1 << 4;
		}
	}

	core->pm.clock_off_steps |= 0x1 << 5;
	state = atomic_read(&core->clk_ref);
	mfc_core_debug(2, "- %d\n", state);
	MFC_TRACE_LOG_CORE("c-%d", state);
}

int mfc_core_pm_power_on(struct mfc_core *core)
{
#ifdef CONFIG_MFC_USE_BUS_DEVFREQ
	struct mfc_dev *dev = core->dev;
	struct mfc_platdata *pdata = dev->pdata;
#endif
	int ret;

	MFC_TRACE_CORE("++ Power on\n");
	ret = pm_runtime_get_sync(core->pm.device);
	if (ret < 0) {
		snprintf(core->crash_info, MFC_CRASH_INFO_LEN,
			"Failed to get power: ret(%d)\n", ret);
		mfc_core_err("%s", core->crash_info);
		call_dop(core, dump_and_stop_debug_mode, core);
		goto err_power_on;
	}

#ifdef CONFIG_MFC_USE_BUS_DEVFREQ
	if (pdata->idle_clk_ctrl) {
		mfc_core_debug(2, "request mfc idle clk OFF\n");
		exynos_pm_qos_add_request(&core->qos_req_mfc_noidle,
				PM_QOS_MFC_THROUGHPUT +
				(core->id * MFC_THROUGHPUT_OFFSET),
				pdata->mfc_freqs[0]);
	}
#endif

	core->pm.clock = clk_get(core->device, "aclk_mfc");
	if (IS_ERR(core->pm.clock)) {
		mfc_core_err("failed to get parent clock: ret(%d)\n", ret);
	} else {
		ret = clk_prepare(core->pm.clock);
		if (ret) {
			mfc_core_err("clk_prepare() failed: ret(%d)\n", ret);
			clk_put(core->pm.clock);
		}
	}

	atomic_inc(&core->pm.pwr_ref);

	MFC_TRACE_CORE("-- Power on: ret(%d)\n", ret);
	MFC_TRACE_LOG_CORE("p+%d", mfc_core_pm_get_pwr_ref_cnt(core));

	return 0;

err_power_on:
	return ret;
}

int mfc_core_pm_power_off(struct mfc_core *core)
{
#ifdef CONFIG_MFC_USE_BUS_DEVFREQ
	struct mfc_dev *dev = core->dev;
	struct mfc_platdata *pdata = dev->pdata;
#endif
	int ret;
	int state = atomic_read(&core->clk_ref);

	MFC_TRACE_CORE("++ Power off\n");

	while (state > 0) {
		mfc_core_info("MFC clock is still enabled (%d)\n", state);
		mfc_core_pm_clock_off(core);
		state = atomic_read(&core->clk_ref);
	}

	if (!IS_ERR(core->pm.clock)) {
		clk_unprepare(core->pm.clock);
		clk_put(core->pm.clock);
	}

#ifdef CONFIG_MFC_USE_BUS_DEVFREQ
	if (pdata->idle_clk_ctrl) {
		exynos_pm_qos_remove_request(&core->qos_req_mfc_noidle);
		mfc_core_debug(2, "request mfc idle clk ON\n");
	}
#endif

	ret = pm_runtime_put_sync(core->pm.device);
	if (ret < 0) {
		snprintf(core->crash_info, MFC_CRASH_INFO_LEN,
			"Failed to put power: ret(%d)\n", ret);
		mfc_core_err("%s", core->crash_info);
		call_dop(core, dump_and_stop_debug_mode, core);
		return ret;
	}

	atomic_dec(&core->pm.pwr_ref);

	MFC_TRACE_CORE("-- Power off: ret(%d)\n", ret);
	MFC_TRACE_LOG_CORE("p-%d", mfc_core_pm_get_pwr_ref_cnt(core));

	return ret;
}

void mfc_core_pm_idle_suspend(struct mfc_core *core)
{
	struct device_driver *drv = &mfc_core_driver.driver;
	int ret = 0;

	if ((core->sleep == 1) || !idle_suspend_enable)
		return;

	if (!mfc_core_pm_get_pwr_ref_cnt(core)) {
		mfc_core_info("MFC core idle suspend is skipped since mfc is power off");
		return;
	}

	mfc_perf_core_trace("sleep", 1);

	mfc_core_debug(2, "MFC core idle suspend is called\n");

	ret = drv->pm->suspend(core->pm.device);

	if (ret < 0) {
		mfc_core_err("MFC core failed to suspend: ret(%d)\n", ret);
		return;
	}

	ret = mfc_core_pm_power_off(core);

	if (ret < 0) {
		mfc_core_err("MFC core failed to power off: ret(%d)\n", ret);
		return;
	}

	mfc_perf_core_trace("sleep", 0);

	mfc_core_debug(2, "MFC core idle suspend is completed\n");

}

void mfc_core_pm_idle_resume(struct mfc_core *core)
{
	struct device_driver *drv = &mfc_core_driver.driver;
	int ret = 0;

	if (core->sleep == 0)
		return;

	mfc_perf_core_trace("wakeup", 1);

	mfc_core_debug(2, "MFC core idle resume is called\n");

	ret = mfc_core_pm_power_on(core);

	if (ret < 0) {
		mfc_core_err("MFC core failed to power on: ret(%d)\n", ret);
		return;
	}

	ret = drv->pm->resume(core->pm.device);

	if (ret < 0) {
		mfc_core_err("MFC core failed to resume: ret(%d)\n", ret);
		return;
	}

	mfc_perf_core_trace("wakeup", 0);

	mfc_core_debug(2, "MFC core idle resume is completed\n");

}
