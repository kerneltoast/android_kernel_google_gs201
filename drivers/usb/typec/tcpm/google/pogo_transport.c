// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2021, Google LLC
 *
 * Pogo management driver
 */

#include <linux/i2c.h>
#include <linux/extcon.h>
#include <linux/extcon-provider.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/usb/tcpm.h>

#include "../tcpci.h"
#include "tcpci_max77759.h"

#define POGO_TIMEOUT_MS 10000

struct pogo_event {
	struct kthread_work work;
	struct pogo_transport *pogo_transport;
	bool pogo_active;
};

struct pogo_transport {
	struct device *dev;
	struct max77759_plat *chip;
	struct logbuffer *log;
	int pogo_gpio;
	int pogo_irq;
	int pogo_data_mux_gpio;
	bool pogo_usb_active;
	bool pogo_active;
	struct kthread_worker *wq;
};

static void update_pogo_transport(struct kthread_work *work)
{
	struct pogo_event *event = container_of(work, struct pogo_event, work);
	struct pogo_transport *pogo_transport = event->pogo_transport;
	struct max77759_plat *chip = pogo_transport->chip;
	int ret;

	mutex_lock(&chip->data_path_lock);
	pogo_transport->pogo_active = event->pogo_active;
	logbuffer_log(chip->log, "%s pogo_usb:%d pogo_usb_active:%d data_active:%d", __func__,
		      pogo_transport->pogo_active ? 1 : 0, pogo_transport->pogo_usb_active ? 1 : 0,
		      chip->data_active ? 1 : 0);
	if (pogo_transport->pogo_active && !pogo_transport->pogo_usb_active) {
		data_alt_path_active(chip, true);
		if (chip->data_active) {
			ret = extcon_set_state_sync(chip->extcon,
						    chip->active_data_role == TYPEC_HOST ?
						    EXTCON_USB_HOST : EXTCON_USB, 0);

			logbuffer_log(chip->log, "%s turning off %s",
				      ret < 0 ? "Failed" : "Succeeded",
				      chip->active_data_role == TYPEC_HOST ? "Host" : "Device");
			chip->data_active = false;
		}
		gpio_set_value(pogo_transport->pogo_data_mux_gpio, 1);
		logbuffer_log(pogo_transport->log, "POGO: data-mux:%d",
			      gpio_get_value(pogo_transport->pogo_data_mux_gpio));
		ret = extcon_set_state_sync(chip->extcon, EXTCON_USB_HOST, 1);
		logbuffer_log(chip->log, "%s: %s turning on host for Pogo", __func__, ret < 0 ?
			      "Failed" : "Succeeded");
		pogo_transport->pogo_usb_active = true;
	}
	if (!pogo_transport->pogo_active && pogo_transport->pogo_usb_active) {
		ret = extcon_set_state_sync(chip->extcon, EXTCON_USB_HOST, 0);
		logbuffer_log(chip->log, "%s: %s turning off host for Pogo", __func__, ret < 0 ?
			      "Failed" : "Succeeded");
		pogo_transport->pogo_usb_active = false;
		gpio_set_value(pogo_transport->pogo_data_mux_gpio, 0);
		logbuffer_log(pogo_transport->log, "POGO: data-mux:%d",
			      gpio_get_value(pogo_transport->pogo_data_mux_gpio));
		data_alt_path_active(chip, false);
		enable_data_path_locked(chip);
	}
	mutex_unlock(&chip->data_path_lock);
	devm_kfree(pogo_transport->dev, event);
}

static irqreturn_t pogo_irq(int irq, void *dev_id)
{
	struct pogo_transport *pogo_transport = dev_id;
	struct pogo_event *evt;

	logbuffer_log(pogo_transport->log, "Pogo threaded irq running");

	evt = devm_kzalloc(pogo_transport->dev, sizeof(*evt), GFP_KERNEL);
	if (!evt) {
		logbuffer_log(pogo_transport->log, "POGO: Dropping event");
		return IRQ_HANDLED;
	}
	kthread_init_work(&evt->work, update_pogo_transport);
	evt->pogo_transport = pogo_transport;
	evt->pogo_active = !gpio_get_value(pogo_transport->pogo_gpio);
	kthread_queue_work(pogo_transport->wq, &evt->work);

	return IRQ_HANDLED;
}

static irqreturn_t pogo_isr(int irq, void *dev_id)
{
	struct pogo_transport *pogo_transport = dev_id;

	logbuffer_log(pogo_transport->log, "POGO IRQ triggered ");
	pm_wakeup_event(pogo_transport->dev, POGO_TIMEOUT_MS);

	return IRQ_WAKE_THREAD;
}

static int init_pogo_alert_gpio(struct pogo_transport *pogo_transport)
{
	int ret;

	pogo_transport->pogo_gpio = of_get_named_gpio(pogo_transport->dev->of_node,
						      "pogo-transport-status", 0);
	if (pogo_transport->pogo_gpio < 0) {
		dev_err(pogo_transport->dev, "Pogo status gpio not found ret:%d\n",
			pogo_transport->pogo_gpio);
		return pogo_transport->pogo_gpio;
	}

	ret = devm_gpio_request(pogo_transport->dev, pogo_transport->pogo_gpio,
				"pogo-transport-status");
	if (ret) {
		dev_err(pogo_transport->dev, "failed to request pogo-transport-status gpio, ret:%d",
			ret);
		return ret;
	}

	ret = gpio_direction_input(pogo_transport->pogo_gpio);
	if (ret) {
		dev_err(pogo_transport->dev, "failed set pogo-transport-status as input, ret:%d",
			ret);
		return ret;
	}

	pogo_transport->pogo_irq = gpio_to_irq(pogo_transport->pogo_gpio);
	if (pogo_transport->pogo_irq <= 0) {
		dev_err(pogo_transport->dev, "Pogo irq not found\n");
		return -ENODEV;
	}

	ret = devm_request_threaded_irq(pogo_transport->dev, pogo_transport->pogo_irq, pogo_isr,
					pogo_irq, (IRQF_SHARED | IRQF_ONESHOT |
						   IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING),
					dev_name(pogo_transport->dev), pogo_transport);
	if (ret < 0) {
		dev_err(pogo_transport->dev, "pogo-transport-status request irq failed ret:%d\n",
			ret);
		return ret;
	}

	ret = enable_irq_wake(pogo_transport->pogo_irq);
	if (ret) {
		dev_err(pogo_transport->dev, "Enable irq wake failed ret:%d\n");
		goto free_irq;
	}

	pogo_transport->pogo_data_mux_gpio = of_get_named_gpio(pogo_transport->dev->of_node,
							       "pogo-transport-sel", 0);
	if (pogo_transport->pogo_data_mux_gpio < 0) {
		dev_err(pogo_transport->dev, "Pogo sel gpio not found ret:%d\n",
			pogo_transport->pogo_gpio);
		ret = pogo_transport->pogo_data_mux_gpio;
		goto disable_irq;
	}

	ret = devm_gpio_request(pogo_transport->dev, pogo_transport->pogo_data_mux_gpio,
				"pogo-transport-sel");
	if (ret) {
		dev_err(pogo_transport->dev, "failed to request pogo-transport-sel gpio, ret:%d",
			ret);
		goto disable_irq;
	}

	ret = gpio_direction_output(pogo_transport->pogo_data_mux_gpio, 0);
	if (ret) {
		dev_err(pogo_transport->dev, "failed set pogo-transport-sel as output, ret:%d",
			ret);
		goto disable_irq;
	}

	return 0;

disable_irq:
	disable_irq_wake(pogo_transport->pogo_irq);
free_irq:
	devm_free_irq(pogo_transport->dev, pogo_transport->pogo_irq, pogo_transport);

	return ret;
}

static int pogo_transport_probe(struct platform_device *pdev)
{
	struct pogo_transport *pogo_transport;
	int ret = 0;
	struct device_node *data_np;
	struct i2c_client *data_client;
	struct max77759_plat *chip;

	data_np = of_parse_phandle(pdev->dev.of_node, "data-phandle", 0);
	if (!data_np) {
		dev_err(&pdev->dev, "Failed to find tcpci node\n");
		return -ENODEV;
	}

	data_client = of_find_i2c_device_by_node(data_np);
	if (!data_client) {
		dev_err(&pdev->dev, "Failed to find tcpci client\n");
		ret = -EPROBE_DEFER;
		goto free_np;
	}

	chip = i2c_get_clientdata(data_client);
	if (!chip) {
		dev_err(&pdev->dev, "Failed to find max77759_plat\n");
		ret = -EPROBE_DEFER;
		goto put_client;
	}

	pogo_transport = devm_kzalloc(&pdev->dev, sizeof(*pogo_transport), GFP_KERNEL);
	if (!pogo_transport) {
		ret = -ENOMEM;
		goto put_client;
	}

	pogo_transport->dev = &pdev->dev;
	pogo_transport->chip = chip;

	pogo_transport->log = logbuffer_register("pogo_transport");
	if (IS_ERR_OR_NULL(pogo_transport->log)) {
		dev_err(pogo_transport->dev, "logbuffer get failed\n");
		ret = -EPROBE_DEFER;
		goto put_client;
	}
	platform_set_drvdata(pdev, pogo_transport);

	pogo_transport->wq = kthread_create_worker(0, "wq-pogo-transport");
	if (IS_ERR_OR_NULL(pogo_transport->wq)) {
		ret = PTR_ERR(pogo_transport->wq);
		goto unreg_logbuffer;
	}

	ret = init_pogo_alert_gpio(pogo_transport);
	if (ret) {
		logbuffer_log(pogo_transport->log, "init_pogo_alert_gpio error:%d\n", ret);
		goto destroy_worker;
	}

	put_device(&data_client->dev);
	of_node_put(data_np);
	return 0;

destroy_worker:
	kthread_destroy_worker(pogo_transport->wq);
unreg_logbuffer:
	logbuffer_unregister(pogo_transport->log);
put_client:
	put_device(&data_client->dev);
free_np:
	of_node_put(data_np);
	return ret;
}

static int pogo_transport_remove(struct platform_device *pdev)
{
	struct pogo_transport *pogo_transport = platform_get_drvdata(pdev);

	disable_irq_wake(pogo_transport->pogo_irq);
	devm_free_irq(pogo_transport->dev, pogo_transport->pogo_irq, pogo_transport);
	kthread_destroy_worker(pogo_transport->wq);
	logbuffer_unregister(pogo_transport->log);

	return 0;
}

static const struct of_device_id pogo_transport_of_match[] = {
	{.compatible = "pogo-transport"},
	{},
};
MODULE_DEVICE_TABLE(of, pogo_transport_of_match);

static struct platform_driver pogo_transport_driver = {
	.driver = {
		   .name = "pogo-transport",
		   .owner = THIS_MODULE,
		   .of_match_table = pogo_transport_of_match,
		   },
	.probe = pogo_transport_probe,
	.remove = pogo_transport_remove,
};

module_platform_driver(pogo_transport_driver);

MODULE_DESCRIPTION("Pogo data management");
MODULE_AUTHOR("Badhri Jagan Sridharan <badhri@google.com>");
MODULE_LICENSE("GPL");
