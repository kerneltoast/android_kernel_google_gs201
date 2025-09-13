// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * BBD PPS GPIO core file
 *
 * Copyright (C) 2024 Google LLC
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio/consumer.h>
#include <linux/list.h>
#include <linux/property.h>
#include <linux/timer.h>

/* Info for each registered platform device */
struct bbd_pps_gpio_device_data {
	int irq; /* IRQ used as PPS source */
	struct gpio_desc *gpio_pin; /* GPIO port descriptors */
	__u32 assert_sequence; /* PPS assert event seq # */
	struct timespec64 g_assert_elapsed_tu; /* PPS elapsed rt assert seq # */
	bool assert_falling_edge;
};

static ssize_t pps_assert_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct bbd_pps_gpio_device_data *bbd_pps_gpio = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%lld.%09ld#%u\n",
		       (long long)bbd_pps_gpio->g_assert_elapsed_tu.tv_sec,
		       bbd_pps_gpio->g_assert_elapsed_tu.tv_nsec,
		       bbd_pps_gpio->assert_sequence);
}

static DEVICE_ATTR_RO(pps_assert);

/* Handle the PPS pulse in the interrupt handler */
static irqreturn_t bbd_pps_gpio_handler(int irq, void *data)
{
	struct bbd_pps_gpio_device_data *bbd_pps_gpio_device = data;
	struct timespec64 ts64 = { .tv_sec = 0, .tv_nsec = 0 };
	/*
	 * Calculate the monotonic clock from the timespec clock and stores the result
	 * in timespec64 format using ktime_get_boottime_ts64() for compatibility with
	 * the Android sensor system.
	 */
	ktime_get_boottime_ts64(&ts64);
	bbd_pps_gpio_device->g_assert_elapsed_tu = ts64;
	bbd_pps_gpio_device->assert_sequence++;

	return IRQ_HANDLED;
}

static int bbd_pps_gpio_setup(struct device *dev)
{
	struct bbd_pps_gpio_device_data *data = dev_get_drvdata(dev);

	data->gpio_pin = devm_gpiod_get(dev, NULL, GPIOD_IN);
	if (IS_ERR(data->gpio_pin))
		return dev_err_probe(dev, PTR_ERR(data->gpio_pin),
				     "failed to request PPS GPIO\n");

	data->assert_falling_edge =
		device_property_read_bool(dev, "assert-falling-edge");

	return 0;
}

/*
 * Different from the mainline PPS GPIO driver, only two things
 * 1. Probe the device-tree.
 * 2. Register the GPIO interrupt.
 */
static int bbd_pps_gpio_probe(struct platform_device *pdev)
{
	struct bbd_pps_gpio_device_data *data;
	struct device *dev = &pdev->dev;
	int ret;
	unsigned long int_flags;

	/* allocate space for device info */
	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	dev_set_drvdata(dev, data);

	/* GPIO setup */
	ret = bbd_pps_gpio_setup(dev);
	if (ret)
		return ret;

	/* IRQ setup */
	ret = gpiod_to_irq(data->gpio_pin);
	if (ret < 0) {
		dev_err(dev, "failed to map GPIO to IRQ: %d\n", ret);
		return -EINVAL;
	}
	data->irq = ret;

	/* register IRQ interrupt handler */
	int_flags = data->assert_falling_edge ? IRQF_TRIGGER_FALLING :
						IRQF_TRIGGER_RISING;
	ret = devm_request_irq(dev, data->irq, bbd_pps_gpio_handler, int_flags,
			       "BBD_GPIO", data);
	if (ret) {
		dev_err(dev, "failed to acquire IRQ %d\n", data->irq);
		return -EINVAL;
	}

	if (device_create_file(dev, &dev_attr_pps_assert)) {
		dev_err(dev, "failed to create device file pps_assert");
		return -EINVAL;
	}

	return 0;
}

static int bbd_pps_gpio_remove(struct platform_device *pdev)
{
	struct bbd_pps_gpio_device_data *data = platform_get_drvdata(pdev);
	disable_irq_nosync(data->irq);
	dev_dbg(&pdev->dev, "removed IRQ %d as PPS source\n", data->irq);
	device_remove_file(&pdev->dev, &dev_attr_pps_assert);
	return 0;
}

static int bbd_pps_gpio_suspend(struct device *dev)
{
	struct bbd_pps_gpio_device_data *data = dev_get_drvdata(dev);
	disable_irq_nosync(data->irq);
	return 0;
}

static int bbd_pps_gpio_resume(struct device *dev)
{
	struct bbd_pps_gpio_device_data *data = dev_get_drvdata(dev);
	enable_irq(data->irq);
	return 0;
}

static const struct dev_pm_ops bbd_pps_pm_ops = {
	.suspend = bbd_pps_gpio_suspend,
	.resume = bbd_pps_gpio_resume,
};

static const struct of_device_id pps_gpio_dt_ids[] = {
	{
		.compatible = "bbd-pps-gpio",
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, pps_gpio_dt_ids);

static struct platform_driver bbd_pps_gpio_driver = {
	.probe		= bbd_pps_gpio_probe,
	.remove		= bbd_pps_gpio_remove,
	.driver		= {
		.name		= "bbd_pps_gpio",
		.of_match_table	= pps_gpio_dt_ids,
		.pm		= &bbd_pps_pm_ops,
	},
};

/*
 * Module init stuff
 */
static int __init bbd_pps_gpio_init(void)
{
	return platform_driver_register(&bbd_pps_gpio_driver);
}

static void __exit bbd_pps_gpio_exit(void)
{
	platform_driver_unregister(&bbd_pps_gpio_driver);
}

module_init(bbd_pps_gpio_init);
module_exit(bbd_pps_gpio_exit);
MODULE_AUTHOR("Cheng Change <chengcha@google.com>");
MODULE_DESCRIPTION("Special PPS GPIO Handler");
MODULE_LICENSE("GPL");
