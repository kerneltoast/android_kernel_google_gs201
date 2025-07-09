// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google LWIS SPI Device Driver
 *
 * Copyright (c) 2023 Google, LLC
 */

#define pr_fmt(fmt) KBUILD_MODNAME "-spi-dev: " fmt

#include "lwis_device_spi.h"

#include <linux/device.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/sched.h>
#include <linux/sched/types.h>
#include <linux/slab.h>
#include <uapi/linux/sched/types.h>

#include "lwis_device.h"
#include "lwis_spi.h"
#include "lwis_util.h"
#include "lwis_trace.h"

#ifdef CONFIG_OF
#include "lwis_dt.h"
#endif

#define LWIS_DRIVER_NAME "lwis-spi-device"

static int lwis_spi_device_enable(struct lwis_device *lwis_dev);
static int lwis_spi_device_disable(struct lwis_device *lwis_dev);
static int lwis_spi_register_io(struct lwis_device *lwis_dev, struct lwis_io_entry *entry,
				int access_size);

static struct lwis_device_subclass_operations spi_vops = {
	.register_io = lwis_spi_register_io,
	.register_io_barrier = NULL,
	.device_enable = lwis_spi_device_enable,
	.device_disable = lwis_spi_device_disable,
	.event_enable = NULL,
	.event_flags_updated = NULL,
	.close = NULL,
};

static int lwis_spi_device_enable(struct lwis_device *lwis_dev)
{
	return 0;
}

static int lwis_spi_device_disable(struct lwis_device *lwis_dev)
{
	return 0;
}

static int lwis_spi_register_io(struct lwis_device *lwis_dev, struct lwis_io_entry *entry,
				int access_size)
{
	struct lwis_spi_device *spi_dev;

	spi_dev = container_of(lwis_dev, struct lwis_spi_device, base_dev);

	/* Running in interrupt context is not supported as spi driver might sleep */
	if (in_interrupt())
		return -EAGAIN;

	lwis_save_register_io_info(lwis_dev, entry, access_size);

	return lwis_spi_io_entry_rw(spi_dev, entry);
}

static int spi_device_setup(struct lwis_spi_device *spi_dev)
{
	int ret;

#ifdef CONFIG_OF
	/* Parse device tree for device configurations */
	ret = lwis_spi_device_parse_dt(spi_dev);
	if (ret) {
		dev_err(spi_dev->base_dev.dev, "Failed to parse device tree\n");
		return ret;
	}
#else
	/* Non-device-tree init: Save for future implementation */
	return -EINVAL;
#endif

	ret = spi_setup(spi_dev->spi);
	if (ret) {
		dev_err(spi_dev->base_dev.dev, "spi_setup fail (%d)\n", ret);
		return ret;
	}

	return 0;
}

static int lwis_spi_device_probe(struct spi_device *spi)
{
	int ret = 0;
	struct lwis_spi_device *spi_dev;
	struct device *dev = &spi->dev;

	/* Allocate SPI device specific data construct */
	spi_dev = devm_kzalloc(dev, sizeof(*spi_dev), GFP_KERNEL);
	if (!spi_dev)
		return -ENOMEM;

	spi_dev->base_dev.type = DEVICE_TYPE_SPI;
	spi_dev->base_dev.vops = spi_vops;
	spi_dev->base_dev.plat_dev = NULL;
	spi_dev->base_dev.k_dev = &spi->dev;
	spi_dev->spi = spi;

	/* Initialize mutex lock */
	mutex_init(&spi_dev->spi_lock);

	/* Call the base device probe function */
	ret = lwis_base_probe(&spi_dev->base_dev);
	if (ret) {
		dev_err(dev, "Error in lwis base probe\n");
		return ret;
	}
	spi_set_drvdata(spi, &spi_dev->base_dev);

	/* Call SPI device specific setup function */
	ret = spi_device_setup(spi_dev);
	if (ret) {
		dev_err(spi_dev->base_dev.dev, "Error in spi device initialization\n");
		lwis_base_unprobe(&spi_dev->base_dev);
		return ret;
	}

	/* Create associated kworker threads */
	ret = lwis_create_kthread_workers(&spi_dev->base_dev);
	if (ret) {
		dev_err(spi_dev->base_dev.dev, "Failed to create lwis_spi_kthread");
		lwis_base_unprobe(&spi_dev->base_dev);
		return ret;
	}

	if (spi_dev->base_dev.transaction_thread_priority != 0) {
		ret = lwis_set_kthread_priority(&spi_dev->base_dev,
						spi_dev->base_dev.transaction_worker_thread,
						spi_dev->base_dev.transaction_thread_priority);
		if (ret) {
			dev_err(spi_dev->base_dev.dev,
				"Failed to set LWIS SPI transaction kthread priority (%d)", ret);
			lwis_base_unprobe(&spi_dev->base_dev);
			return ret;
		}
	}

	dev_info(spi_dev->base_dev.dev, "SPI Device Probe: Success\n");

	return 0;
}

#ifdef CONFIG_PM
static int lwis_spi_device_suspend(struct device *dev)
{
	struct lwis_device *lwis_dev = dev_get_drvdata(dev);

	if (lwis_dev->pm_hibernation == 0)
		return 0;

	if (lwis_dev->enabled != 0) {
		dev_warn(lwis_dev->dev, "Can't suspend because %s is in use!\n", lwis_dev->name);
		return -EBUSY;
	}

	return 0;
}

static int lwis_spi_device_resume(struct device *dev)
{
	return 0;
}

static SIMPLE_DEV_PM_OPS(lwis_spi_device_ops, lwis_spi_device_suspend, lwis_spi_device_resume);
#endif

#ifdef CONFIG_OF
static const struct of_device_id lwis_id_match[] = {
	{ .compatible = LWIS_SPI_DEVICE_COMPAT },
	{},
};
// MODULE_DEVICE_TABLE(of, lwis_id_match);

static struct spi_driver lwis_driver = {
	.probe = lwis_spi_device_probe,
	.driver = {
		.name = LWIS_DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = lwis_id_match,
		.pm = &lwis_spi_device_ops,
	},
};
#else /* CONFIG_OF not defined */
static struct spi_device_id lwis_driver_id[] = {
	{
		.name = LWIS_DRIVER_NAME,
		.driver_data = 0,
	},
	{},
};
MODULE_DEVICE_TABLE(spi, lwis_driver_id);

static struct spi_driver lwis_driver = { .probe = lwis_spi_device_probe,
					 .id_table = lwis_driver_id,
					 .driver = {
						 .name = LWIS_DRIVER_NAME,
						 .owner = THIS_MODULE,
					 } };
#endif /* CONFIG_OF */

/*
 *  lwis_spi_device_init: Init function that will be called by the kernel
 *  initialization routines.
 */
int __init lwis_spi_device_init(void)
{
	int ret = 0;

	pr_info("SPI device initialization\n");

	ret = spi_register_driver(&lwis_driver);
	if (ret)
		pr_err("spi_register_driver failed: %d\n", ret);

	return ret;
}

int lwis_spi_device_deinit(void)
{
	spi_unregister_driver(&lwis_driver);
	return 0;
}
