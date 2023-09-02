// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2020 Google LLC.
 *
 * Author: Sidath Senanayake <sidaths@google.com>
 */

/* Linux includes */
#include <linux/of_device.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#endif

/* SOC includes */
#ifdef CONFIG_MALI_PIXEL_GPU_SECURE_RENDERING
#include <soc/samsung/exynos-smc.h>
#endif /* CONFIG_MALI_PIXEL_GPU_SECURE_RENDERING */

/* Mali core includes */
#include <mali_kbase.h>
#ifdef CONFIG_MALI_PIXEL_GPU_SECURE_RENDERING
#include <device/mali_kbase_device_internal.h>
#endif /* CONFIG_MALI_PIXEL_GPU_SECURE_RENDERING */
#if MALI_USE_CSF
#include <csf/mali_kbase_csf_firmware_cfg.h>
#endif

/* Pixel integration includes */
#include "mali_kbase_config_platform.h"
#include "pixel_gpu_control.h"
#include "pixel_gpu_sscd.h"

#define CREATE_TRACE_POINTS
#include "pixel_gpu_trace.h"

#ifdef CONFIG_MALI_PIXEL_GPU_SECURE_RENDERING
/**
 * GPU_SMC_TZPC_OK -  SMC CALL return value on success
 */
#define GPU_SMC_TZPC_OK 0

#ifdef CONFIG_MALI_HOST_CONTROLS_SC_RAILS
#define HOST_CONTROLS_SC_RAILS_CFG_ENTRY_NAME "Host controls SC rails"
#endif

/**
 * pixel_gpu_secure_mode_enable() - Enables secure mode for the GPU
 *
 * @pdev: Pointer to the &struct protected_mode_device associated with the GPU
 *
 * Context: The caller needs to hold the GPU HW access lock.
 *
 * Return: On success, returns 0. Otherwise returns a non-zero value to indicate
 * failure.
 */
static int pixel_gpu_secure_mode_enable(struct protected_mode_device *pdev)
{
	struct kbase_device *kbdev = pdev->data;
	struct pixel_context *pc = kbdev->platform_context;
	int ret = 0;

	lockdep_assert_held(&kbdev->hwaccess_lock);

	/* We expect to only be called when not already in protected mode */
	WARN_ON(kbdev->protected_mode);

	ret = kbase_pm_protected_mode_enable(kbdev);
	if (ret != 0)
		return ret;

	if (!pc->tz_protection_enabled) {
		ret = exynos_smc(SMC_PROTECTION_SET, 0, PROT_G3D,
				 SMC_PROTECTION_ENABLE);
		if (ret == GPU_SMC_TZPC_OK)
			pc->tz_protection_enabled = true;
		else
			dev_err(kbdev->dev,
				"%s: SMC_PROTECTION_SET (ENABLE) failed: %d\n",
				__func__, ret);
	}

	return ret;
}

/**
 * pixel_gpu_secure_mode_disable() - Disables secure mode for the GPU
 *
 * @pdev: Pointer to the &struct protected_mode_device associated with the GPU
 *
 * Context: The caller needs to hold the GPU PM access lock.
 *
 * Return: On success, returns 0. Otherwise returns a non-zero value to indicate
 * failure.
 */
static int pixel_gpu_secure_mode_disable(struct protected_mode_device *pdev)
{
	/* Turn off secure mode and reset GPU : TZPC */
	struct kbase_device *kbdev = pdev->data;
	struct pixel_context *pc = kbdev->platform_context;
	int ret = 0;

	lockdep_assert_held(&kbdev->pm.lock);

	/* This function is called whenever the GPU is reset, whether it was
	 * previously in protected mode or not. SMC returns an error if we try
	 * to disable protection when it wasn't enabled.
	 */
	if (pc->tz_protection_enabled) {
		ret = exynos_smc(SMC_PROTECTION_SET, 0, PROT_G3D,
				 SMC_PROTECTION_DISABLE);
		if (ret == GPU_SMC_TZPC_OK)
			pc->tz_protection_enabled = false;
		else
			dev_err(kbdev->dev,
				"%s: SMC_PROTECTION_SET (DISABLE) failed: %d\n",
				__func__, ret);
	}

	return kbase_pm_protected_mode_disable(kbdev);
}

struct protected_mode_ops pixel_protected_ops = {
	.protected_mode_enable = pixel_gpu_secure_mode_enable,
	.protected_mode_disable = pixel_gpu_secure_mode_disable
};

#endif /* CONFIG_MALI_PIXEL_GPU_SECURE_RENDERING */

#ifdef CONFIG_MALI_HOST_CONTROLS_SC_RAILS
/**
 * gpu_pixel_enable_host_ctrl_sc_rails() - Enable the config in FW to support host based
 *                                         control of SC power rails
 *
 * Look for the config entry that enables support in FW for the Host based
 * control of shader core power rails and set it before the initial boot
 * or reload of firmware.
 *
 * @kbdev:     Kbase device structure
 *
 * Return: 0 if successful, negative error code on failure
 */
static int gpu_pixel_enable_host_ctrl_sc_rails(struct kbase_device *kbdev)
{
	u32 addr;
	int ec = kbase_csf_firmware_cfg_find_config_address(
		kbdev, HOST_CONTROLS_SC_RAILS_CFG_ENTRY_NAME, &addr);

	if (!ec) {
		kbase_csf_update_firmware_memory(kbdev, addr, 1);
	}

	return ec;
}
#endif

static int gpu_fw_cfg_init(struct kbase_device *kbdev) {
	int ec = 0;

#if MALI_USE_CSF
#ifdef CONFIG_MALI_HOST_CONTROLS_SC_RAILS
	ec = gpu_pixel_enable_host_ctrl_sc_rails(kbdev);
	if (ec)
		dev_warn(kbdev->dev, "pixel: failed to enable SC rail host-control");
#endif
	if (gpu_sscd_fw_log_init(kbdev, 0)) {
		dev_warn(kbdev->dev, "pixel: failed to enable FW log");
	}
#endif

	return ec;
}

/**
 * gpu_pixel_init() - Initializes the Pixel integration for the Mali GPU.
 *
 * @kbdev: The &struct kbase_device for the GPU.
 *
 * Return: On success, returns 0. On failure an error code is returned.
 */
static int gpu_pixel_init(struct kbase_device *kbdev)
{
	int ret;

	struct pixel_context *pc;

	pc = kzalloc(sizeof(struct pixel_context), GFP_KERNEL);
	if (pc == NULL) {
		dev_err(kbdev->dev, "pixel: failed to alloc platform context struct\n");
		ret = -ENOMEM;
		goto done;
	}

	kbdev->platform_context = pc;
	pc->kbdev = kbdev;

	ret = gpu_pm_init(kbdev);
	if (ret)
		goto done;

#ifdef CONFIG_MALI_MIDGARD_DVFS
	ret = gpu_dvfs_init(kbdev);
	if (ret) {
		dev_err(kbdev->dev, "DVFS init failed\n");
		goto done;
	}
#endif /* CONFIG_MALI_MIDGARD_DVFS */

	ret = gpu_sysfs_init(kbdev);
	if (ret) {
		dev_err(kbdev->dev, "sysfs init failed\n");
		goto done;
	}

	ret = gpu_sscd_init(kbdev);
	if (ret) {
		dev_err(kbdev->dev, "SSCD init failed\n");
		goto done;
	}
	ret = 0;

done:
	return ret;
}

/**
 * gpu_pixel_term() - Terminates the Pixel integration for the Mali GPU.
 *
 * @kbdev: The &struct kbase_device for the GPU.
 */
static void gpu_pixel_term(struct kbase_device *kbdev)
{
	struct pixel_context *pc = kbdev->platform_context;

	gpu_sscd_term(kbdev);
	gpu_sysfs_term(kbdev);
	gpu_dvfs_term(kbdev);
	gpu_pm_term(kbdev);

	kbdev->platform_context = NULL;
	kfree(pc);
}

struct kbase_platform_funcs_conf platform_funcs = {
	.platform_init_func = &gpu_pixel_init,
	.platform_term_func = &gpu_pixel_term,
	.platform_handler_context_init_func = &gpu_dvfs_kctx_init,
	.platform_handler_context_term_func = &gpu_dvfs_kctx_term,
	.platform_handler_work_begin_func = &gpu_dvfs_metrics_work_begin,
	.platform_handler_work_end_func = &gpu_dvfs_metrics_work_end,
	.platform_fw_cfg_init_func = &gpu_fw_cfg_init,
	.platform_handler_core_dump_func = &gpu_sscd_dump,
};
