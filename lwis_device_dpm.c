// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google LWIS Dynamic Power Managerment
 *
 * Copyright (c) 2020 Google, LLC
 */

#define pr_fmt(fmt) KBUILD_MODNAME "-dpm: " fmt

#include "lwis_device_dpm.h"

#include <linux/clk.h>
#include <linux/slab.h>

#include "lwis_commands.h"
#include "lwis_platform.h"

#define LWIS_DRIVER_NAME "lwis-dpm"

static struct lwis_device_subclass_operations dpm_vops = {
	.register_io = NULL,
	.register_io_barrier = NULL,
	.device_enable = NULL,
	.device_disable = NULL,
	.event_enable = NULL,
	.event_flags_updated = NULL,
	.close = NULL,
};

/*
 *  lwis_dpm_update_qos: update qos requirement for lwis device.
 */
int lwis_dpm_update_qos(struct lwis_device *lwis_dev, struct lwis_qos_setting_v3 *qos_setting)
{
	int ret = 0;
	struct lwis_device *target_dev = lwis_find_dev_by_id(qos_setting->device_id);

	if (!target_dev) {
		dev_err(lwis_dev->dev, "Can't find device by id: %d\n", qos_setting->device_id);
		return -ENOENT;
	}

	/* b/190270885 : We see some ramdump issues due to dpm qos updates
	 * when device is disabled. We might disallow to update qos on this case.
	 */
	if (target_dev->enabled == 0 && target_dev->type != DEVICE_TYPE_DPM) {
		dev_warn(target_dev->dev, "%s disabled, no need to update qos\n", target_dev->name);
		return -EPERM;
	}

	// TODO: Real qos_family_name implementation will be done in b/291854347
	// As for qos update, either of qos_family_name or clock_family need set.
	if (strlen(qos_setting->qos_family_name) > 0 ||
	    (qos_setting->clock_family != CLOCK_FAMILY_INVALID)) {
		ret = lwis_platform_dpm_update_qos(lwis_dev, target_dev, qos_setting);
	} else {
		dev_err(lwis_dev->dev, "Invalid clock family name and clock family %d\n",
			qos_setting->clock_family);
		ret = -EINVAL;
	}

	return ret;
}

/*
 *  lwis_dpm_update_clock: update specific clock settings to lwis device.
 */
int lwis_dpm_update_clock(struct lwis_device *lwis_dev, struct lwis_clk_setting *clk_settings,
			  size_t num_settings)
{
	int ret = 0, i, clk_index;
	uint32_t old_clk;

	if (!lwis_dev->clocks) {
		dev_err(lwis_dev->dev, "%s has no clocks\n", lwis_dev->name);
		ret = -EINVAL;
		goto out;
	}

	for (i = 0; i < num_settings; ++i) {
		clk_index = clk_settings[i].clk_index;
		if (clk_index < 0 || clk_index >= lwis_dev->clocks->count) {
			dev_err(lwis_dev->dev, "%s clk index %d is invalid\n", lwis_dev->name,
				clk_index);
			ret = -EINVAL;
			goto out;
		}

		if (IS_ERR_OR_NULL(lwis_dev->clocks->clk[clk_index].clk)) {
			dev_err(lwis_dev->dev, "%s clk is invalid\n", lwis_dev->name);
			ret = -EINVAL;
			goto out;
		}

		old_clk = clk_get_rate(lwis_dev->clocks->clk[clk_index].clk);
		if (old_clk == clk_settings[i].frequency)
			continue;

		ret = clk_set_rate(lwis_dev->clocks->clk[clk_index].clk, clk_settings[i].frequency);
		if (ret) {
			dev_err(lwis_dev->dev, "Error updating clock %s freq: %u\n",
				lwis_dev->clocks->clk[clk_index].name, clk_settings[i].frequency);
			goto out;
		}

		dev_info(lwis_dev->dev, "Update %s freq from %u to %u, clock read back: %lu\n",
			 lwis_dev->clocks->clk[clk_index].name, old_clk, clk_settings[i].frequency,
			 clk_get_rate(lwis_dev->clocks->clk[clk_index].clk));
	}
out:
	return ret;
}

static int lwis_dpm_device_probe(struct platform_device *plat_dev)
{
	int ret = 0;
	struct lwis_dpm_device *dpm_dev;
	struct device *dev = &plat_dev->dev;

	/* Allocate top device specific data construct */
	dpm_dev = devm_kzalloc(dev, sizeof(struct lwis_dpm_device), GFP_KERNEL);
	if (!dpm_dev)
		return -ENOMEM;

	dpm_dev->base_dev.type = DEVICE_TYPE_DPM;
	dpm_dev->base_dev.vops = dpm_vops;
	dpm_dev->base_dev.plat_dev = plat_dev;
	dpm_dev->base_dev.k_dev = &plat_dev->dev;

	/* Call the base device probe function */
	ret = lwis_base_probe(&dpm_dev->base_dev);
	if (ret)
		dev_err(dev, "Error in lwis base probe, ret: %d\n", ret);

	platform_set_drvdata(plat_dev, &dpm_dev->base_dev);

	return ret;
}

#ifdef CONFIG_OF
static const struct of_device_id lwis_id_match[] = {
	{ .compatible = LWIS_DPM_DEVICE_COMPAT },
	{},
};
// MODULE_DEVICE_TABLE(of, lwis_id_match);

static struct platform_driver lwis_driver = {
	.probe = lwis_dpm_device_probe,
	.driver = {
		.name = LWIS_DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = lwis_id_match,
	},
};

#else /* CONFIG_OF not defined */
static struct platform_device_id lwis_driver_id[] = {
	{
		.name = LWIS_DRIVER_NAME,
		.driver_data = 0,
	},
	{},
};
MODULE_DEVICE_TABLE(platform, lwis_driver_id);

static struct platform_driver lwis_driver = { .probe = lwis_dpm_device_probe,
					      .id_table = lwis_driver_id,
					      .driver = {
						      .name = LWIS_DRIVER_NAME,
						      .owner = THIS_MODULE,
					      } };
#endif /* CONFIG_OF */

/*
 *  lwis_dpm_device_init: Init function that will be called by the kernel
 *  initialization routines.
 */
int __init lwis_dpm_device_init(void)
{
	int ret = 0;

	pr_info("DPM device initialization\n");

	ret = platform_driver_register(&lwis_driver);
	if (ret)
		pr_err("platform_driver_register failed: %d\n", ret);

	return ret;
}

int lwis_dpm_device_deinit(void)
{
	platform_driver_unregister(&lwis_driver);
	return 0;
}

uint32_t lwis_dpm_read_clock(struct lwis_device *lwis_dev)
{
	uint32_t clock = 0;

	if (!lwis_dev->clocks) {
		dev_err(lwis_dev->dev, "%s clock not defined", lwis_dev->name);
		return -ENODEV;
	}
	clock = clk_get_rate(lwis_dev->clocks->clk[0].clk);
	return clock;
}
