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

/* Mali core includes */
#include <mali_kbase.h>
#include <device/mali_kbase_device_internal.h>
#if MALI_USE_CSF
#include <csf/mali_kbase_csf_firmware_cfg.h>
#endif

/* We need this include due to the removal from mali_kbase.h */
#include <mali_kbase_hwaccess_pm.h>

/* Pixel integration includes */
#include "mali_kbase_config_platform.h"
#include "pixel_gpu_control.h"
#include "pixel_gpu_sscd.h"
#include "pixel_gpu_slc.h"

#define CREATE_TRACE_POINTS
#include "pixel_gpu_trace.h"

#include "pixel_gpu_uevent.h"

static int gpu_fw_cfg_init(struct kbase_device *kbdev) {
	int ec = 0;

#if MALI_USE_CSF
	if (gpu_sscd_fw_log_init(kbdev, 0)) {
		dev_warn(kbdev->dev, "pixel: failed to enable FW log");
	}
#endif

	return ec;
}

/**
 * gpu_pixel_kctx_init() - Called when a kernel context is created
 *
 * @kctx: The &struct kbase_context that is being initialized
 *
 * This function is called when the GPU driver is initializing a new kernel context.
 *
 * Return: Returns 0 on success, or an error code on failure.
 */
static int gpu_pixel_kctx_init(struct kbase_context *kctx)
{
	struct kbase_device* kbdev = kctx->kbdev;
	struct pixel_platform_data *platform_data;
	int err;

	kctx->platform_data = kzalloc(sizeof(struct pixel_platform_data), GFP_KERNEL);
	if (kctx->platform_data == NULL) {
		dev_err(kbdev->dev, "pixel: failed to alloc platform_data for kctx");
		err = -ENOMEM;
		goto done;
	}

	platform_data = kctx->platform_data;
	platform_data->kctx = kctx;

	err = gpu_dvfs_kctx_init(kctx);
	if (err) {
		dev_err(kbdev->dev, "pixel: DVFS kctx init failed\n");
		goto done;
	}

	err = gpu_slc_kctx_init(kctx);
	if (err) {
		dev_err(kbdev->dev, "pixel: SLC kctx init failed\n");
		goto done;
	}

done:
	return err;
}

/**
 * gpu_pixel_kctx_term() - Called when a kernel context is terminated
 *
 * @kctx: The &struct kbase_context that is being terminated
 */
static void gpu_pixel_kctx_term(struct kbase_context *kctx)
{
	gpu_slc_kctx_term(kctx);
	gpu_dvfs_kctx_term(kctx);

	kfree(kctx->platform_data);
	kctx->platform_data = NULL;
}

#ifdef CONFIG_MALI_PM_RUNTIME_S2MPU_CONTROL
/**
 * gpu_s2mpu_init - Initialize S2MPU for G3D
 *
 * @kbdev: The &struct kbase_device for the GPU.
 *
 * Return: On success, returns 0. On failure an error code is returned.
 */
static int gpu_s2mpu_init(struct kbase_device *kbdev)
{
	int ret = 0;
	struct device_node *np;
	struct platform_device *pdev;

	/*
	 * We expect "s2mpus" entry in device tree to point to gpu s2mpu device
	 */
	np = of_parse_phandle(kbdev->dev->of_node, "s2mpus", 0);
	if (!np) {
		dev_err(kbdev->dev, "No 's2mpus' entry found in the device tree\n");
		ret = -ENODEV;
		goto done;
	}

	pdev = of_find_device_by_node(np);
	of_node_put(np);
	if (!pdev) {
		dev_err(kbdev->dev, "No device specified in 's2mpus' device node\n");
		ret = -ENODEV;
		goto done;
	}

	kbdev->s2mpu_dev = &pdev->dev;
	dev_info(kbdev->dev, "s2mpu device %s successfully configured\n",
				dev_name(kbdev->s2mpu_dev));

done:
	return ret;
}
#endif /* CONFIG_MALI_PM_RUNTIME_S2MPU_CONTROL */

static const struct kbase_device_init dev_init[] = {
#ifdef CONFIG_MALI_PM_RUNTIME_S2MPU_CONTROL
	{ gpu_s2mpu_init, NULL, "S2MPU init failed" },
#endif /* CONFIG_MALI_PM_RUNTIME_S2MPU_CONTROL */
	{ gpu_pm_init, gpu_pm_term, "PM init failed" },
#ifdef CONFIG_MALI_MIDGARD_DVFS
	{ gpu_dvfs_init, gpu_dvfs_term, "DVFS init failed" },
#endif
	{ gpu_sysfs_init, gpu_sysfs_term, "sysfs init failed" },
	{ gpu_sscd_init, gpu_sscd_term, "SSCD init failed" },
	{ gpu_slc_init, gpu_slc_term, "SLC init failed" },
#if IS_ENABLED(CONFIG_EXYNOS_ITMON)
	{ gpu_itmon_init, gpu_itmon_term, "ITMON notifier init failed" },
#endif
	{ gpu_uevent_init, gpu_uevent_term, "GPU uevent init failed"},
};

static void gpu_pixel_term_partial(struct kbase_device *kbdev,
		unsigned int i)
{
	while (i-- > 0) {
		if (dev_init[i].term)
			dev_init[i].term(kbdev);
	}
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
	int ret = 0;
	unsigned int i;
	struct pixel_context *pc;

	pc = kzalloc(sizeof(struct pixel_context), GFP_KERNEL);
	if (pc == NULL) {
		dev_err(kbdev->dev, "pixel: failed to alloc platform context struct\n");
		ret = -ENOMEM;
		goto done;
	}

	kbdev->platform_context = pc;
	pc->kbdev = kbdev;

	for (i = 0; i < ARRAY_SIZE(dev_init); i++) {
		if (dev_init[i].init) {
			ret = dev_init[i].init(kbdev);
			if (ret) {
				dev_err(kbdev->dev, "%s error = %d\n",
					dev_init[i].err_mes, ret);
				break;
			}
		}
	}

	if (ret) {
		gpu_pixel_term_partial(kbdev, i);
		kbdev->platform_context = NULL;
		kfree(pc);
	}

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

	gpu_pixel_term_partial(kbdev, ARRAY_SIZE(dev_init));
	kbdev->platform_context = NULL;
	kfree(pc);
}

/**
 * gpu_pixel_late_init() - Verifies final state of features after init.
 *
 * @kbdev: The &struct kbase_device for the GPU.
 */
static int gpu_pixel_late_init(struct kbase_device *kbdev)
{
	if (kbase_is_large_pages_enabled())
		panic("b/407731257: kbase_is_large_pages_enabled() should return false");

	if (kbase_is_page_migration_enabled())
		panic("b/407731257: kbase_is_page_migration_enabled() should return false");

	return 0;
}

struct kbase_platform_funcs_conf platform_funcs = {
	.platform_init_func = &gpu_pixel_init,
	.platform_term_func = &gpu_pixel_term,
	.platform_late_init_func = &gpu_pixel_late_init,
#ifdef CONFIG_MALI_MIDGARD_DVFS
	.platform_handler_context_init_func = &gpu_pixel_kctx_init,
	.platform_handler_context_term_func = &gpu_pixel_kctx_term,
#endif /* CONFIG_MALI_MIDGARD_DVFS */
	.platform_handler_context_active = &gpu_slc_kctx_active,
	.platform_handler_context_idle = &gpu_slc_kctx_idle,
	.platform_handler_tick_tock = &gpu_slc_tick_tock,
	.platform_fw_cfg_init_func = &gpu_fw_cfg_init,
	.platform_handler_core_dump_func = &gpu_sscd_dump,
};
