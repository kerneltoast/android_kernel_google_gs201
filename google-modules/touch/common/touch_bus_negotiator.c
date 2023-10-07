// SPDX-License-Identifier: GPL-2.0
/*
 * Touch Bus Negotiator for Google Pixel devices.
 *
 * Copyright (C) 2021 Google, Inc.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/net.h>
#include <linux/printk.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include "touch_bus_negotiator.h"

#if IS_ENABLED(CONFIG_TOUCHSCREEN_TBN_AOC_CHANNEL_MODE)
#include <uapi/linux/sched/types.h>
#include "aoc_tbn_service_dev.h"
#endif

#define TBN_MODULE_NAME "touch_bus_negotiator"
#define TBN_AOC_CHANNEL_THREAD_NAME "tbn_aoc_channel"

#if IS_ENABLED(CONFIG_TOUCHSCREEN_TBN_AOC_CHANNEL_MODE)
static void handle_tbn_event_response(struct tbn_context *tbn,
	struct TbnEventResponse *response);
#endif

static struct tbn_context *tbn_context;

static irqreturn_t tbn_aoc2ap_irq_thread(int irq, void *ptr)
{
	struct tbn_context *tbn = ptr;

	dev_info(tbn->dev, "%s: bus_released:%d bus_requested:%d.\n", __func__,
		completion_done(&tbn->bus_released), completion_done(&tbn->bus_requested));

	if (completion_done(&tbn->bus_released) && completion_done(&tbn->bus_requested))
		return IRQ_HANDLED;

	/*
	 * For bus release, there two possibilities:
	 * 1. aoc2ap gpio value already changed to AOC
	 * 2. tbn_release_bus() with TBN_RELEASE_BUS_TIMEOUT_MS timeout
	 *    for complete_all(&tbn->bus_released);
	 */
	while (!completion_done(&tbn->bus_released)) {
		if (gpio_get_value(tbn->aoc2ap_gpio) == TBN_BUS_OWNER_AOC)
			complete_all(&tbn->bus_released);
		else
			usleep_range(10000, 10000);	/* wait 10 ms for gpio stablized */
	}

	/*
	 * For bus request, there two possibilities:
	 * 1. aoc2ap gpio value already changed to AP
	 * 2. tbn_request_bus() with TBN_REQUEST_BUS_TIMEOUT_MS timeout
	 *    for complete_all(&tbn->bus_requested);
	 */
	while (!completion_done(&tbn->bus_requested)) {
		if (gpio_get_value(tbn->aoc2ap_gpio) == TBN_BUS_OWNER_AP)
			complete_all(&tbn->bus_requested);
		else
			usleep_range(10000, 10000);	/* wait 10 ms for gpio stablized */
	}

	return IRQ_HANDLED;
}

#if IS_ENABLED(CONFIG_TOUCHSCREEN_TBN_AOC_CHANNEL_MODE)
static int aoc_channel_kthread(void *data)
{
	struct tbn_context *tbn = data;
	struct TbnEventResponse resp;
	ssize_t len;
	bool service_ready = false;

	while (!kthread_should_stop()) {
		if (service_ready != aoc_tbn_service_ready()) {
			service_ready = !service_ready;
			dev_info(tbn->dev, "%s: AOC TBN service is %s.\n",
				__func__, service_ready ? "ready" : "not ready");
		}

		if (!service_ready) {
			msleep(1000);
			continue;
		}

		len = aoc_tbn_service_read(&resp, sizeof(resp));
		if (len < 0) {
			dev_err(tbn->dev, "%s: failed to read message, err: %d\n",
				__func__, len);
			msleep(1000);
			continue;
		}

		if (kthread_should_stop()) {
			break;
		}

		if (len == sizeof(resp)) {
			handle_tbn_event_response(tbn, &resp);
		}
	}

	return 0;
}

static void handle_tbn_event_response(struct tbn_context *tbn,
	struct TbnEventResponse *response)
{
	mutex_lock(&tbn->event_lock);

	if (response->id != tbn->event.id) {
		dev_err(tbn->dev,
			"%s: receive wrong response, id: %d, expected id: %d, "
			"bus_released:%d bus_requested:%d.\n",
			__func__, response->id, tbn->event.id,
			completion_done(&tbn->bus_released),
			completion_done(&tbn->bus_requested));
		goto exit;
	}

	if (response->err != 0) {
		dev_err(tbn->dev, "%s: send tbn event failed, err %d!\n",
			__func__, response->err);
		tbn->event_resp.err = response->err;
	} else {
		tbn->event_resp.lptw_triggered = response->lptw_triggered;
	}

	if (response->operation == TBN_OPERATION_AP_REQUEST_BUS) {
		complete_all(&tbn->bus_requested);
	} else if (response->operation == TBN_OPERATION_AP_RELEASE_BUS) {
		complete_all(&tbn->bus_released);
	} else {
		dev_err(tbn->dev, "%s: response unknown operation, op: %d!\n",
			__func__, response->operation);
	}

exit:
	mutex_unlock(&tbn->event_lock);
}

static void send_tbn_event(struct tbn_context *tbn, enum TbnOperation operation)
{
	ssize_t len;
	int retry = 3;

	if (!aoc_tbn_service_ready()) {
		dev_err(tbn_context->dev, "%s: AOC TBN service is not ready.\n",
			__func__);
		return;
	}

	mutex_lock(&tbn->event_lock);

	tbn->event.operation = operation;
	tbn->event.id++;

	while (retry) {
		len = aoc_tbn_service_write(&tbn->event, sizeof(tbn->event));
		if (len == sizeof(tbn->event)) {
			break;
		}
		dev_err(tbn_context->dev, "%s: failed to send TBN event, retry: %d.\n",
			__func__, retry);
		retry--;
	}

	mutex_unlock(&tbn->event_lock);
}
#endif

int tbn_handshaking(struct tbn_context *tbn, enum TbnOperation operation)
{
	struct completion *wait_for_completion;
	enum tbn_bus_owner bus_owner;
	unsigned int irq_type;
	unsigned int timeout;
	const char *msg;
	int ret = 0;

	if (!tbn || tbn->registered_mask == 0) {
		dev_err(tbn_context->dev, "%s: tbn is not ready to serve.\n", __func__);
		return -EINVAL;
	}

	if (operation == TBN_OPERATION_AP_REQUEST_BUS) {
		wait_for_completion = &tbn->bus_requested;
		bus_owner = TBN_BUS_OWNER_AP;
		irq_type = IRQF_TRIGGER_FALLING;
		timeout = TBN_REQUEST_BUS_TIMEOUT_MS;
		msg = "request";
	} else if (operation == TBN_OPERATION_AP_RELEASE_BUS) {
		wait_for_completion = &tbn->bus_released;
		bus_owner = TBN_BUS_OWNER_AOC;
		irq_type = IRQF_TRIGGER_RISING;
		timeout = TBN_RELEASE_BUS_TIMEOUT_MS;
		msg = "release";
	} else {
		dev_err(tbn_context->dev, "%s: request unknown operation, op: %d.\n",
			__func__, operation);
		return -EINVAL;
	}

	if (tbn->mode == TBN_MODE_GPIO) {
		int ap2aoc_val_org = gpio_get_value(tbn->ap2aoc_gpio);
		int aoc2ap_val_org = gpio_get_value(tbn->aoc2ap_gpio);

		reinit_completion(wait_for_completion);

		irq_set_irq_type(tbn->aoc2ap_irq, irq_type);
		enable_irq(tbn->aoc2ap_irq);
		gpio_direction_output(tbn->ap2aoc_gpio, bus_owner);
		if (wait_for_completion_timeout(wait_for_completion,
			msecs_to_jiffies(timeout)) == 0) {
			int ap2aoc_val = gpio_get_value(tbn->ap2aoc_gpio);
			int aoc2ap_val = gpio_get_value(tbn->aoc2ap_gpio);

			complete_all(wait_for_completion);
			if (bus_owner == aoc2ap_val)
				ret = 0;
			else
				ret = -ETIMEDOUT;
			dev_err(tbn->dev, "AP %s bus ... timeout!, ap2aoc_gpio(B:%d,A:%d)"
				" aoc2ap_gpio(B:%d,A:%d), ret=%d\n",
				msg, ap2aoc_val_org, ap2aoc_val, aoc2ap_val_org,
				aoc2ap_val, ret);
		} else
			dev_info(tbn->dev, "AP %s bus ... SUCCESS!\n", msg);
		disable_irq_nosync(tbn->aoc2ap_irq);
#if IS_ENABLED(CONFIG_TOUCHSCREEN_TBN_AOC_CHANNEL_MODE)
	} else if (tbn->mode == TBN_MODE_AOC_CHANNEL) {
		tbn->event_resp.lptw_triggered = false;
		tbn->event_resp.err = 0;

		reinit_completion(wait_for_completion);

		send_tbn_event(tbn, operation);
		if (wait_for_completion_timeout(wait_for_completion,
			msecs_to_jiffies(timeout)) == 0) {
			dev_err(tbn->dev, "AP %s bus ... timeout!\n", msg);
			complete_all(wait_for_completion);
			ret = -ETIMEDOUT;
		} else {
			if (tbn->event_resp.err == 0) {
				dev_info(tbn->dev, "AP %s bus ... SUCCESS!\n", msg);
			} else {
				dev_info(tbn->dev, "AP %s bus ... failed!\n", msg);
				ret = -EBUSY;
			}
		}
#endif
	} else if (tbn->mode == TBN_MODE_MOCK) {
		dev_info(tbn->dev, "AP %s bus ... SUCCESS!\n", msg);
	} else {
		ret = -EINVAL;
	}

	return ret;
}

int tbn_request_bus_with_result(u32 dev_mask, bool *lptw_triggered)
{
	int ret = 0;

	if (!tbn_context)
		return -ENODEV;

	mutex_lock(&tbn_context->dev_mask_mutex);

	if ((dev_mask & tbn_context->registered_mask) == 0) {
		mutex_unlock(&tbn_context->dev_mask_mutex);
		dev_err(tbn_context->dev, "%s: dev_mask %#x is invalid.\n",
			__func__, dev_mask);
		return -EINVAL;
	}

	if (tbn_context->requested_dev_mask == 0) {
		ret = tbn_handshaking(tbn_context, TBN_OPERATION_AP_REQUEST_BUS);
		if ((ret == 0) && (lptw_triggered != NULL))
			*lptw_triggered = tbn_context->event_resp.lptw_triggered;
	} else {
		dev_dbg(tbn_context->dev,
			"%s: Bus already requested, requested_dev_mask %#x dev_mask %#x.\n",
			__func__, tbn_context->requested_dev_mask, dev_mask);
	}
	tbn_context->requested_dev_mask |= dev_mask;

	mutex_unlock(&tbn_context->dev_mask_mutex);

	return ret;
}
EXPORT_SYMBOL_GPL(tbn_request_bus_with_result);

int tbn_request_bus(u32 dev_mask)
{
	return tbn_request_bus_with_result(dev_mask, NULL);
}
EXPORT_SYMBOL_GPL(tbn_request_bus);

int tbn_release_bus(u32 dev_mask)
{
	int ret = 0;

	if (!tbn_context)
		return -ENODEV;

	mutex_lock(&tbn_context->dev_mask_mutex);

	if ((dev_mask & tbn_context->registered_mask) == 0) {
		mutex_unlock(&tbn_context->dev_mask_mutex);
		dev_err(tbn_context->dev, "%s: dev_mask %#x is invalid.\n",
			__func__, dev_mask);
		return -EINVAL;
	}

	if (tbn_context->requested_dev_mask == 0) {
		dev_warn(tbn_context->dev,
			 "%s: Bus already released, dev_mask %#x.\n",
			 __func__, dev_mask);
		mutex_unlock(&tbn_context->dev_mask_mutex);
		return 0;
	}

	/* Release the bus when the last requested_dev_mask bit releases. */
	if (tbn_context->requested_dev_mask == dev_mask) {
		ret = tbn_handshaking(tbn_context, TBN_OPERATION_AP_RELEASE_BUS);
	} else {
		dev_dbg(tbn_context->dev,
			 "%s: Bus is still in use, requested_dev_mask %#x dev_mask %#x.\n",
			 __func__, tbn_context->requested_dev_mask, dev_mask);
	}

	tbn_context->requested_dev_mask &= ~dev_mask;

	mutex_unlock(&tbn_context->dev_mask_mutex);

	return ret;
}
EXPORT_SYMBOL_GPL(tbn_release_bus);

int register_tbn(u32 *output)
{
	u32 i = 0;

	*output = 0;

	if (!tbn_context) {
		pr_warn("%s: tbn_context doesn't exist.", __func__);
		return 0;
	}

	mutex_lock(&tbn_context->dev_mask_mutex);
	for (i = 0; i < tbn_context->max_devices; i++) {
		if (tbn_context->registered_mask & BIT_MASK(i))
			continue;
		tbn_context->registered_mask |= BIT_MASK(i);
		/* Assume screen is on while registering tbn. */
		tbn_context->requested_dev_mask |= BIT_MASK(i);
		*output = BIT_MASK(i);
		break;
	}

	mutex_unlock(&tbn_context->dev_mask_mutex);

	return 0;
}
EXPORT_SYMBOL_GPL(register_tbn);

void unregister_tbn(u32 *output)
{
	if (!tbn_context)
		return ;

	mutex_lock(&tbn_context->dev_mask_mutex);
	tbn_context->registered_mask &= ~(*output);
	*output = 0;
	mutex_unlock(&tbn_context->dev_mask_mutex);
}
EXPORT_SYMBOL_GPL(unregister_tbn);

static int tbn_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct tbn_context *tbn = NULL;
	struct device_node *np = dev->of_node;
	int err = 0;
#if IS_ENABLED(CONFIG_TOUCHSCREEN_TBN_AOC_CHANNEL_MODE)
	struct sched_param param = {
		.sched_priority = 10,
	};
#endif


	tbn = devm_kzalloc(dev, sizeof(struct tbn_context), GFP_KERNEL);
	if (!tbn)
		goto failed;

	tbn->dev = dev;
	tbn->event_resp.lptw_triggered = false;
	tbn_context = tbn;
	dev_set_drvdata(tbn->dev, tbn);

	if (of_property_read_u32(np, "tbn,max_devices", &tbn->max_devices))
		tbn->max_devices = 1;

	err = of_property_read_u32(np, "tbn,mode", &tbn->mode);
	if (err)
		tbn->mode = TBN_MODE_GPIO;

	if (tbn->mode == TBN_MODE_GPIO) {
		tbn->ap2aoc_gpio = of_get_named_gpio(np, "tbn,ap2aoc_gpio", 0);
		if (gpio_is_valid(tbn->ap2aoc_gpio)) {
			err = devm_gpio_request_one(tbn->dev, tbn->ap2aoc_gpio,
				GPIOF_OUT_INIT_LOW, "tbn,ap2aoc_gpio");
			if (err) {
				dev_err(tbn->dev, "%s: Unable to request ap2aoc_gpio %d, err %d!\n",
					__func__, tbn->ap2aoc_gpio, err);
				goto failed;
			}
		} else {
			dev_err(tbn->dev, "%s: invalid ap2aoc_gpio %d!\n",
				__func__, tbn->ap2aoc_gpio);
			err = -EPROBE_DEFER;
			goto failed;
		}

		tbn->aoc2ap_gpio = of_get_named_gpio(np, "tbn,aoc2ap_gpio", 0);
		if (gpio_is_valid(tbn->aoc2ap_gpio)) {
			err = devm_gpio_request_one(tbn->dev, tbn->aoc2ap_gpio,
				GPIOF_DIR_IN, "tbn,aoc2ap_gpio");
			if (err) {
				dev_err(tbn->dev, "%s: Unable to request aoc2ap_gpio %d, err %d!\n",
					__func__, tbn->aoc2ap_gpio, err);
				goto failed;
			}
			tbn->aoc2ap_irq = gpio_to_irq(tbn->aoc2ap_gpio);
			err = devm_request_threaded_irq(tbn->dev,
				tbn->aoc2ap_irq, NULL,
				tbn_aoc2ap_irq_thread,
				IRQF_TRIGGER_RISING |
				IRQF_ONESHOT, "tbn", tbn);
			if (err) {
				dev_err(tbn->dev,
					"%s: Unable to request_threaded_irq, err %d!\n",
					__func__, err);
				goto failed;
			}
			disable_irq_nosync(tbn->aoc2ap_irq);
		} else {
			dev_err(tbn->dev, "%s: invalid aoc2ap_gpio %d!\n",
				__func__, tbn->aoc2ap_gpio);
			err = -EPROBE_DEFER;
			goto failed;
		}

		dev_info(tbn->dev,
			"%s: gpios(aoc2ap: %d ap2aoc: %d)\n",
			__func__, tbn->aoc2ap_gpio, tbn->ap2aoc_gpio);
#if IS_ENABLED(CONFIG_TOUCHSCREEN_TBN_AOC_CHANNEL_MODE)
	} else if (tbn->mode == TBN_MODE_AOC_CHANNEL) {
		mutex_init(&tbn->event_lock);

		tbn->aoc_channel_task = kthread_run(&aoc_channel_kthread, tbn,
			TBN_AOC_CHANNEL_THREAD_NAME);
		if (IS_ERR(tbn->aoc_channel_task)) {
			err = PTR_ERR(tbn->aoc_channel_task);
			goto failed;
		}

		err = sched_setscheduler(tbn->aoc_channel_task, SCHED_FIFO, &param);
		if (err != 0) {
			goto failed;
		}
#endif
	} else if (tbn->mode == TBN_MODE_MOCK) {
		err = 0;
	} else {
		dev_err(tbn->dev, "bus negotiator: invalid mode: %d\n", tbn->mode);
		err = -EINVAL;
		goto failed;
	}

	mutex_init(&tbn->dev_mask_mutex);

	init_completion(&tbn->bus_requested);
	init_completion(&tbn->bus_released);
	complete_all(&tbn->bus_requested);
	complete_all(&tbn->bus_released);

	dev_info(tbn->dev, "bus negotiator initialized: %pK, mode: %d\n", tbn, tbn->mode);

failed:
	if (err) {
		devm_kfree(dev, tbn);
		tbn_context = NULL;
	}

	return err;
}

static int tbn_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct tbn_context *tbn = dev_get_drvdata(dev);

	if (tbn->mode == TBN_MODE_GPIO) {
		free_irq(tbn->aoc2ap_irq, tbn);
		if (gpio_is_valid(tbn->aoc2ap_gpio))
			gpio_free(tbn->aoc2ap_gpio);
		if (gpio_is_valid(tbn->aoc2ap_gpio))
			gpio_free(tbn->aoc2ap_gpio);
#if IS_ENABLED(CONFIG_TOUCHSCREEN_TBN_AOC_CHANNEL_MODE)
	} else if (tbn->mode == TBN_MODE_AOC_CHANNEL) {
		kthread_stop(tbn->aoc_channel_task);
#endif
	}
	return 0;
}

static struct of_device_id tbn_of_match_table[] = {
	{
		.compatible = TBN_MODULE_NAME,
	},
	{},
};

static struct platform_driver tbn_driver = {
	.driver = {
		.name = TBN_MODULE_NAME,
		.of_match_table = tbn_of_match_table,
	},
	.probe = tbn_probe,
	.remove = tbn_remove,
};

static int __init tbn_init(void)
{
	return platform_driver_register(&tbn_driver);
}

static void __exit tbn_exit(void)
{
	platform_driver_unregister(&tbn_driver);
}
module_init(tbn_init);
module_exit(tbn_exit);

MODULE_SOFTDEP("pre: touch_offload");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Touch Bus Negotiator");
MODULE_AUTHOR("Google, Inc.");
