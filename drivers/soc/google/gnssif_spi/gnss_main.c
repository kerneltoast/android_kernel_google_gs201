// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2022 Samsung Electronics.
 *
 */
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/platform_data/sscoredump.h>
#include <linux/time.h>

#include "gnss_prj.h"
#include "gnss_utils.h"

#define DEVICE_NAME "gnss"

static void sscd_release(struct device *dev);

static struct sscd_platform_data sscd_pdata;

static struct platform_device sscd_dev = {
	.name            = DEVICE_NAME,
	.driver_override = SSCD_NAME,
	.id              = -1,
	.dev             = {
		.platform_data = &sscd_pdata,
		.release       = sscd_release,
		},
};

static void sscd_release(struct device *dev)
{
	gif_info("%s: enter\n", __FUNCTION__);
}

static ssize_t coredump_store(struct device *dev, struct device_attribute *attr, const char *buf,
			      size_t count)
{
	int symbol = (int)';';
	const char *next = NULL;
	char *coredump = NULL;
	char *reason = NULL;
	int length = 0;
	static time64_t last_trigger_time = 0;
	time64_t trigger_time = ktime_get_seconds();

	// Ignore the trigger time less than 30 seconds
	if (trigger_time - last_trigger_time < 30) {
		goto clean;
	}
	last_trigger_time = trigger_time;

	gif_err("Trigger Coredump string: %s\n", buf);
	next = strchr(buf, symbol);
	if (next) {
		length = (int)(next - buf);
		reason = kmalloc(length + 1, GFP_KERNEL);
		if (!reason) {
			gif_err("Allocate crash reason failed\n");
			goto clean;
		}
		memcpy(reason, buf, length);
		reason[length] = '\0';
		length = strlen(next + 1);
		coredump = kmalloc(length, GFP_KERNEL);
		if (!coredump) {
			gif_err("Allocate crash coredump failed\n");
			goto clean;
		}
		memcpy(coredump, next + 1, length);
		gnss_set_coredump(coredump, length, reason);
	}
clean:
	if (reason)
		kfree(reason);
	if (coredump)
		kfree(coredump);
	return count;
}

static DEVICE_ATTR_WO(coredump);

static int parse_dt_common_pdata(struct device_node *np, struct gnss_pdata *pdata)
{
	gif_dt_read_string(np, "device,name", pdata->name);
	gif_dt_read_string(np, "device_node_name", pdata->node_name);

	gif_info("device name: %s node name: %s\n", pdata->name, pdata->node_name);
	return 0;
}

static struct gnss_pdata *gnss_if_parse_dt_pdata(struct device *dev)
{
	struct gnss_pdata *pdata;
	int ret = 0;

	pdata = devm_kzalloc(dev, sizeof(struct gnss_pdata), GFP_KERNEL);
	if (!pdata) {
		gif_err("gnss_pdata: alloc fail\n");
		return ERR_PTR(-ENOMEM);
	}
	dev->platform_data = pdata;

	ret = parse_dt_common_pdata(dev->of_node, pdata);
	if (ret != 0) {
		gif_err("Failed to parse common pdata from dt\n");
		goto parse_dt_pdata_err;
	}

	gif_info("DT parse complete!\n");
	return pdata;

parse_dt_pdata_err:
	if (pdata)
		devm_kfree(dev, pdata);
	dev->platform_data = NULL;

	return ERR_PTR(-EINVAL);
}

static int gnss_remove(struct platform_device *pdev)
{
	struct gnss_ctl *gc = platform_get_drvdata(pdev);

	gif_disable_irq_sync(&gc->irq_gnss2ap_spi);

	gpio_set_value(gc->gpio_ap2gnss_spi.num, 0);

	free_irq(gc->irq_gnss2ap_spi.num, &gc);

	gpio_free(gc->gpio_ap2gnss_spi.num);

	gpio_free(gc->gpio_gnss2ap_spi.num);

	platform_device_unregister(&sscd_dev);

	device_remove_file(&pdev->dev, &dev_attr_coredump);

	return 0;
}

static int gnss_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct gnss_pdata *pdata = dev->platform_data;
	struct gnss_ctl *gc;
	struct io_device *iod;
	struct link_device *ld;

	gif_info("Exynos GNSS interface driver probe begins\n");

	gif_info("%s: +++\n", pdev->name);

	if (!dev->of_node) {
		gif_err("No DT data!\n");
		goto probe_fail;
	}

	pdata = gnss_if_parse_dt_pdata(dev);
	if (IS_ERR(pdata)) {
		gif_err("DT parse error!\n");
		return PTR_ERR(pdata);
	}

	gc = create_ctl_device(pdev);
	if (!gc) {
		gif_err("%s: Could not create gc\n", pdata->name);
		goto probe_fail;
	}

	ld = create_link_device(pdev);
	if (!ld) {
		gif_err("%s: Could not create ld\n", pdata->name);
		goto free_gc;
	}

	ld->gc = gc;

	iod = create_io_device(pdev, ld, gc, pdata);
	if (!iod) {
		gif_err("%s: Could not create iod\n", pdata->name);
		goto free_ld;
	}

	platform_set_drvdata(pdev, gc);

	platform_device_register(&sscd_dev);

	if (device_create_file(dev, &dev_attr_coredump))
		gif_err("Unable to create sysfs coredump entry");

	/* wa: to prevent wrong irq handling during probe */
	gif_enable_irq(&gc->irq_gnss2ap_spi);

	gif_info("%s: ---\n", pdev->name);

	return 0;

free_ld:
	devm_kfree(dev, ld);
free_gc:
	devm_kfree(dev, gc);
probe_fail:
	gif_err("%s: xxx\n", pdata->name);

	return -ENOMEM;
}

static const struct of_device_id gnss_dt_match[] = {
	{
		.compatible = "samsung,exynos-gnss",
	},
	{},
};
MODULE_DEVICE_TABLE(of, gnss_dt_match);

static struct platform_driver gnss_driver = {
		.probe = gnss_probe,
		.remove = gnss_remove,
		.driver = {
				.name = "gnss_interface",
				.owner = THIS_MODULE,
#if IS_ENABLED(CONFIG_OF)
				.of_match_table = of_match_ptr(gnss_dt_match),
#endif
		},
};

module_platform_driver(gnss_driver);

int gnss_set_coredump(const char *buf, int buf_len, const char *info)
{
	struct sscd_platform_data *pdata = dev_get_platdata(&sscd_dev.dev);
	struct sscd_segment seg;

	if (pdata->sscd_report) {
		memset(&seg, 0, sizeof(seg));
		seg.addr = (void *)buf;
		seg.size = buf_len;
		pdata->sscd_report(&sscd_dev, &seg, 1, 0, info);
	}
	return 0;
}

MODULE_DESCRIPTION("Exynos GNSS interface driver");
MODULE_LICENSE("GPL");
