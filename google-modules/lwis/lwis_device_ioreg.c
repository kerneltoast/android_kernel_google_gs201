/*
 * Google LWIS I/O Mapped Device Driver
 *
 * Copyright (c) 2018 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define pr_fmt(fmt) KBUILD_MODNAME "-ioreg-dev: " fmt

#include "lwis_device_ioreg.h"

#include <linux/device.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <uapi/linux/sched/types.h>

#include "lwis_init.h"
#include "lwis_interrupt.h"
#include "lwis_ioreg.h"
#include "lwis_periodic_io.h"
#include "lwis_util.h"

#ifdef CONFIG_OF
#include "lwis_dt.h"
#endif

#define LWIS_DRIVER_NAME "lwis-ioreg"

static int lwis_ioreg_device_enable(struct lwis_device *lwis_dev);
static int lwis_ioreg_device_disable(struct lwis_device *lwis_dev);
static int lwis_ioreg_register_io(struct lwis_device *lwis_dev, struct lwis_io_entry *entry,
				  int access_size);
static int lwis_ioreg_register_io_barrier(struct lwis_device *lwis_dev, bool read, bool write);

static struct lwis_device_subclass_operations ioreg_vops = {
	.register_io = lwis_ioreg_register_io,
	.register_io_barrier = lwis_ioreg_register_io_barrier,
	.device_enable = lwis_ioreg_device_enable,
	.device_disable = lwis_ioreg_device_disable,
	.event_enable = NULL,
	.event_flags_updated = NULL,
	.close = NULL,
};

static struct lwis_event_subscribe_operations ioreg_subscribe_ops = {
	.subscribe_event = NULL,
	.unsubscribe_event = NULL,
	.notify_event_subscriber = NULL,
	.release = NULL,
};

static int lwis_ioreg_device_enable(struct lwis_device *lwis_dev)
{
	return 0;
}

static int lwis_ioreg_device_disable(struct lwis_device *lwis_dev)
{
	return 0;
}

static int lwis_ioreg_register_io(struct lwis_device *lwis_dev, struct lwis_io_entry *entry,
				  int access_size)
{
	lwis_save_register_io_info(lwis_dev, entry, access_size);
	return lwis_ioreg_io_entry_rw((struct lwis_ioreg_device *)lwis_dev, entry, access_size);
}

static int lwis_ioreg_register_io_barrier(struct lwis_device *lwis_dev, bool use_read_barrier,
					  bool use_write_barrier)
{
	return lwis_ioreg_set_io_barrier((struct lwis_ioreg_device *)lwis_dev, use_read_barrier,
					 use_write_barrier);
}

static int lwis_ioreg_device_setup(struct lwis_ioreg_device *ioreg_dev)
{
	int ret = 0;

#ifdef CONFIG_OF
	/* Parse device tree for device configurations */
	ret = lwis_ioreg_device_parse_dt(ioreg_dev);
	if (ret) {
		dev_err(ioreg_dev->base_dev.dev, "Failed to parse device tree\n");
	}
#else
	/* Non-device-tree init: Save for future implementation */
	ret = -ENOSYS;
#endif

	return ret;
}

static int lwis_ioreg_device_probe(struct platform_device *plat_dev)
{
	int ret = 0;
	struct lwis_ioreg_device *ioreg_dev;

	/* Allocate IOREG device specific data construct */
	ioreg_dev = kzalloc(sizeof(struct lwis_ioreg_device), GFP_KERNEL);
	if (!ioreg_dev) {
		pr_err("Failed to allocate IOREG device structure\n");
		return -ENOMEM;
	}

	ioreg_dev->base_dev.type = DEVICE_TYPE_IOREG;
	ioreg_dev->base_dev.vops = ioreg_vops;
	ioreg_dev->base_dev.subscribe_ops = ioreg_subscribe_ops;

	/* Call the base device probe function */
	ret = lwis_base_probe(&ioreg_dev->base_dev, plat_dev);
	if (ret) {
		pr_err("Error in lwis base probe\n");
		goto error_probe;
	}

	/* Call IOREG device specific setup function */
	ret = lwis_ioreg_device_setup(ioreg_dev);
	if (ret) {
		dev_err(ioreg_dev->base_dev.dev, "Error in IOREG device initialization\n");
		lwis_base_unprobe(&ioreg_dev->base_dev);
		goto error_probe;
	}

	/* Create associated kworker threads */
	ret = lwis_create_kthread_workers(&ioreg_dev->base_dev);
	if (ret) {
		dev_err(ioreg_dev->base_dev.dev, "Failed to create lwis_ioreg_kthread");
		lwis_base_unprobe(&ioreg_dev->base_dev);
		goto error_probe;
	}

	if (ioreg_dev->base_dev.transaction_thread_priority != 0) {
		ret = lwis_set_kthread_priority(&ioreg_dev->base_dev,
						ioreg_dev->base_dev.transaction_worker_thread,
						ioreg_dev->base_dev.transaction_thread_priority);
		if (ret) {
			dev_err(ioreg_dev->base_dev.dev,
				"Failed to set LWIS IOREG transaction kthread priority (%d)", ret);
			lwis_base_unprobe(&ioreg_dev->base_dev);
			goto error_probe;
		}
	}

	dev_info(ioreg_dev->base_dev.dev, "IOREG Device Probe: Success\n");

	return 0;

error_probe:
	kfree(ioreg_dev);
	return ret;
}

#ifdef CONFIG_PM
static int lwis_ioreg_device_suspend(struct device *dev)
{
	struct lwis_device *lwis_dev = dev_get_drvdata(dev);

	if (lwis_dev->enabled != 0) {
		dev_warn(lwis_dev->dev, "Can't suspend because %s is in use!\n", lwis_dev->name);
		return -EBUSY;
	}

	return 0;
}

static int lwis_ioreg_device_resume(struct device *dev)
{
	return 0;
}

static SIMPLE_DEV_PM_OPS(lwis_ioreg_device_ops, lwis_ioreg_device_suspend,
			 lwis_ioreg_device_resume);
#endif

#ifdef CONFIG_OF
static const struct of_device_id lwis_id_match[] = {
	{ .compatible = LWIS_IOREG_DEVICE_COMPAT },
	{},
};
// MODULE_DEVICE_TABLE(of, lwis_id_match);

static struct platform_driver lwis_driver = {
	.probe = lwis_ioreg_device_probe,
	.driver =
		{
			.name = LWIS_DRIVER_NAME,
			.owner = THIS_MODULE,
			.of_match_table = lwis_id_match,
			.pm = &lwis_ioreg_device_ops,
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

static struct platform_driver lwis_driver = { .probe = lwis_ioreg_device_probe,
					      .id_table = lwis_driver_id,
					      .driver = {
						      .name = LWIS_DRIVER_NAME,
						      .owner = THIS_MODULE,
					      } };
#endif /* CONFIG_OF */

/*
 *  lwis_ioreg_device_init: Init function that will be called by the kernel
 *  initialization routines.
 */
int __init lwis_ioreg_device_init(void)
{
	int ret = 0;

	pr_info("IOREG device initialization\n");

	ret = platform_driver_register(&lwis_driver);
	if (ret) {
		pr_err("platform_driver_register failed: %d\n", ret);
	}

	return ret;
}

int lwis_ioreg_device_deinit(void)
{
	platform_driver_unregister(&lwis_driver);
	return 0;
}
