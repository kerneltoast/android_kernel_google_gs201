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
#include <soc/samsung/exynos-smc.h>

#include "mfc_core_qos.h"
#include "mfc_core_pm.h"

#include "mfc_core_hw_reg_api.h"

void mfc_core_pm_init(struct mfc_core *core)
{
	spin_lock_init(&core->pm.clklock);
	atomic_set(&core->pm.pwr_ref, 0);
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
		core->pm.clock_on_steps |= 0x1 << 2;
		ret = mfc_core_wait_pending(core);
		if (ret != 0) {
			mfc_core_err("pending wait failed (%d)\n", ret);
			call_dop(core, dump_and_stop_debug_mode, core);
			return ret;
		}
		core->pm.clock_on_steps |= 0x1 << 3;
		mfc_core_set_risc_base_addr(core, core->pm.base_type);
	}

	core->pm.clock_on_steps |= 0x1 << 4;
#if IS_ENABLED(CONFIG_EXYNOS_CONTENT_PATH_PROTECTION)
	if (core->curr_core_ctx_is_drm) {
		unsigned long flags;

		spin_lock_irqsave(&core->pm.clklock, flags);
		mfc_core_debug(3, "Begin: enable protection\n");
		ret = exynos_smc(SMC_PROTECTION_SET, 0,
				core->id * PROT_MFC1, SMC_PROTECTION_ENABLE);
		core->pm.clock_on_steps |= 0x1 << 5;
		if (ret != DRMDRV_OK) {
			mfc_core_err("Protection Enable failed! ret(%u)\n", ret);
			call_dop(core, dump_and_stop_debug_mode, core);
			spin_unlock_irqrestore(&core->pm.clklock, flags);
			return -EACCES;
		}
		mfc_core_debug(3, "End: enable protection\n");
		spin_unlock_irqrestore(&core->pm.clklock, flags);
	}
#endif

	core->pm.clock_on_steps |= 0x1 << 6;
	if (!IS_ERR(core->pm.clock)) {
		ret = clk_enable(core->pm.clock);
		if (ret < 0)
			mfc_core_err("clk_enable failed (%d)\n", ret);
	}

	core->pm.clock_on_steps |= 0x1 << 7;
	atomic_inc_return(&core->clk_ref);

	core->pm.clock_on_steps |= 0x1 << 8;
	state = atomic_read(&core->clk_ref);
	mfc_core_debug(2, "+ %d\n", state);
	MFC_TRACE_LOG_CORE("c+%d", state);

	return 0;
}

/* Use only in functions that first instance is guaranteed, like mfc_init_hw() */
int mfc_core_pm_clock_on_with_base(struct mfc_core *core,
				enum mfc_buf_usage_type buf_type)
{
	int ret;
	core->pm.base_type = buf_type;
	ret = mfc_core_pm_clock_on(core);
	core->pm.base_type = MFCBUF_INVALID;

	return ret;
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
		if (!IS_ERR(core->pm.clock))
			clk_disable(core->pm.clock);

		core->pm.clock_off_steps |= 0x1 << 4;
#if IS_ENABLED(CONFIG_EXYNOS_CONTENT_PATH_PROTECTION)
		if (core->curr_core_ctx_is_drm) {
			unsigned long flags;
			int ret = 0;
			/*
			 * After clock off the protection disable should be
			 * because the MFC core can continuously run during clock on
			 */
			mfc_core_debug(3, "Begin: disable protection\n");
			spin_lock_irqsave(&core->pm.clklock, flags);
			core->pm.clock_off_steps |= 0x1 << 5;
			ret = exynos_smc(SMC_PROTECTION_SET, 0,
					core->id * PROT_MFC1, SMC_PROTECTION_DISABLE);
			if (ret != DRMDRV_OK) {
				mfc_core_err("Protection Disable failed! ret(%u)\n", ret);
				call_dop(core, dump_and_stop_debug_mode, core);
				spin_unlock_irqrestore(&core->pm.clklock, flags);
				return;
			}
			mfc_core_debug(3, "End: disable protection\n");
			core->pm.clock_off_steps |= 0x1 << 6;
			spin_unlock_irqrestore(&core->pm.clklock, flags);
		}
#endif
	}

	core->pm.clock_off_steps |= 0x1 << 7;
	state = atomic_read(&core->clk_ref);
	mfc_core_debug(2, "- %d\n", state);
	MFC_TRACE_LOG_CORE("c-%d", state);
}

int mfc_core_pm_power_on(struct mfc_core *core)
{
	struct mfc_dev *dev = core->dev;
	struct mfc_platdata *pdata = dev->pdata;
	int ret;

	MFC_TRACE_CORE("++ Power on\n");
	ret = pm_runtime_get_sync(core->pm.device);
	if (ret < 0) {
		mfc_core_err("Failed to get power: ret(%d)\n", ret);
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
	struct mfc_dev *dev = core->dev;
	struct mfc_platdata *pdata = dev->pdata;
	int ret;

	MFC_TRACE_CORE("++ Power off\n");

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
		mfc_core_err("Failed to put power: ret(%d)\n", ret);
		call_dop(core, dump_and_stop_debug_mode, core);
		return ret;
	}

	atomic_dec(&core->pm.pwr_ref);

	MFC_TRACE_CORE("-- Power off: ret(%d)\n", ret);
	MFC_TRACE_LOG_CORE("p-%d", mfc_core_pm_get_pwr_ref_cnt(core));

	return ret;
}
