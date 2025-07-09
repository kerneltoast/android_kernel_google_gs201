// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2022 Samsung Electronics.
 *
 */

#include <linux/init.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>

#include "include/gnss.h"
#include "gnss_prj.h"
#include "gnss_utils.h"

/* handling interrupt by BETP messages from kepler through SPI */
static irqreturn_t kepler_spi_irq_handler(int irq, void *data)
{
	struct gnss_ctl *gc = data;
	struct link_device *ld = gc->iod->ld;

	gif_disable_irq_nosync(&gc->irq_gnss2ap_spi);

	if (atomic_read(&gc->wait_rdy)) {
		atomic_set(&gc->wait_rdy, 0);
		complete_all(&gc->gnss_rdy_cmpl);
		return IRQ_HANDLED;
	}

	if (atomic_read(&gc->rx_in_progress) == 0) {
		queue_delayed_work(ld->rx_wq, &ld->rx_dwork, 0);
	}

	return IRQ_HANDLED;
}

static int kepler_spi_status(struct gnss_ctl *gc)
{
	return gpio_get_value(gc->gpio_gnss2ap_spi.num);
}

static void kepler_send_betp_int(struct gnss_ctl *gc, int value)
{
	gif_debug("About to Set ap2gnss_spi value to %d\n", value);
	gpio_set_value(gc->gpio_ap2gnss_spi.num, value);
	gif_debug("complete to set ap2gnss_spi value to %d\n", value);
}

static int kepler_betp_int_status(struct gnss_ctl *gc)
{
	return gpio_get_value(gc->gpio_ap2gnss_spi.num);
}

static void gnss_set_ops(struct gnss_ctl *gc)
{
	gc->ops.gnss_spi_status = kepler_spi_status;
	gc->ops.gnss_send_betp_int = kepler_send_betp_int;
	gc->ops.gnss_betp_int_status = kepler_betp_int_status;
}

static int init_ctl_device(struct gnss_ctl *gc, struct gnss_pdata *pdata)
{
	int ret;
	struct platform_device *pdev;
	struct device_node *np;

	gnss_set_ops(gc);

	dev_set_drvdata(gc->dev, gc);

	pdev = to_platform_device(gc->dev);
	np = pdev->dev.of_node;

	/* GNSS2AP SPI INT */
	gc->gpio_gnss2ap_spi.num = of_get_named_gpio(np, "gpio_gnss2ap_spi", 0);
	if (gc->gpio_gnss2ap_spi.num < 0) {
		gif_err("Can't get gpio_gnss2ap_spi!\n");
		return -ENODEV;
	}
	gc->gpio_gnss2ap_spi.label = "GNSS2AP_SPI";
	ret = gpio_request(gc->gpio_gnss2ap_spi.num, gc->gpio_gnss2ap_spi.label);
	if (ret) {
		gif_err("Request gpio fail - gpio_gnss2ap_spi.num failed(%d)\n", ret);
		return ret;
	}
	gc->irq_gnss2ap_spi.num = gpio_to_irq(gc->gpio_gnss2ap_spi.num);

	/* Disable the interrupt until gnss_main.c probe function finish */
	gif_configure_irq(&gc->irq_gnss2ap_spi, gc->irq_gnss2ap_spi.num,
			"kepler_spi_irq_handler", IRQF_TRIGGER_HIGH | IRQF_NO_AUTOEN);

	ret = gif_request_irq(&gc->irq_gnss2ap_spi, kepler_spi_irq_handler, gc);
	if (ret) {
		gif_err("Request irq fail - kepler_spi_irq_handler(%d)\n", ret);
		goto free_gpio_gnss2ap;
	}

	ret = enable_irq_wake(gc->irq_gnss2ap_spi.num);
	if (ret) {
		gif_err("Failed to enable wakeup_source - enable_irq_wake(%d)\n", ret);
		goto free_gpio_irq;
	}

	gc->gpio_ap2gnss_spi.num =
				of_get_named_gpio(np, "gpio_ap2gnss_spi", 0);
	if (gc->gpio_ap2gnss_spi.num < 0) {
		gif_err("Can't get gpio_ap2gnss_spi!\n");
		ret = -ENODEV;
		goto free_gpio_irq;
	}
	gc->gpio_ap2gnss_spi.label = "AP2GNSS_SPI";
	ret = gpio_request(gc->gpio_ap2gnss_spi.num, gc->gpio_ap2gnss_spi.label);
	if (ret) {
		gif_err("Request gpio fail - gpio_ap2gnss_spi.num failed(%d)\n", ret);
		goto free_gpio_irq;
	}

	ret = gpio_direction_output(gc->gpio_ap2gnss_spi.num, 0);
	if (ret) {
		gif_err("Set gpio direction fail - gpio_ap2gnss_spi.num failed(%d)\n", ret);
		goto free_gpio_ap2gnss;
	}

	/* initialize gpio values to all zero */
	gpio_set_value(gc->gpio_ap2gnss_spi.num, 0);

	init_completion(&gc->gnss_rdy_cmpl);

	atomic_set(&gc->wait_rdy, 0);
	atomic_set(&gc->tx_in_progress, 0);
	atomic_set(&gc->rx_in_progress, 0);

	return ret;

free_gpio_ap2gnss:
	gpio_free(gc->gpio_ap2gnss_spi.num);

free_gpio_irq:
	free_irq(gc->irq_gnss2ap_spi.num, &gc);

free_gpio_gnss2ap:
	gpio_free(gc->gpio_gnss2ap_spi.num);

	return ret;

}

struct gnss_ctl *create_ctl_device(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct gnss_pdata *pdata = pdev->dev.platform_data;
	struct gnss_ctl *gc;
	int ret;

	gc = devm_kzalloc(dev, sizeof(struct gnss_ctl), GFP_KERNEL);
	if (!gc)
		return NULL;
	gc->dev = dev;
	gc->pdata = pdata;
	gc->name = pdata->name;

	ret = init_ctl_device(gc, pdata);
	if (ret) {
		gif_err("%s: init_ctl_device fail (err %d)\n",
				pdata->name, ret);
		devm_kfree(dev, gc);
		return NULL;
	}

	return gc;
}
