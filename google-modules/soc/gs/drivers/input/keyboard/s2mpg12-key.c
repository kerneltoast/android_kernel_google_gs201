// SPDX-License-Identifier: GPL-2.0+
/*
 * Driver for Power keys on s2mpg12 IC by PWRON rising, falling interrupts.
 *
 * s2mpg12-keys.c
 * S2MPG12 Keyboard Driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/of_irq.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/s2mpg12-key.h>
#include <linux/mfd/samsung/s2mpg12.h>
#include <linux/mfd/samsung/s2mpg12-register.h>

#define WAKELOCK_TIME		(HZ / 10)

static int force_key_irq_en;

struct power_button_data {
	struct power_keys_button *button;
	struct input_dev *input;
	struct work_struct work;
	struct delayed_work key_work;
	struct workqueue_struct *irq_wqueue;
	bool key_pressed;
	bool suspended;
};

struct power_keys_drvdata {
	struct device *dev;
	struct s2mpg12_platform_data *s2mpg12_pdata;
	const struct power_keys_platform_data *pdata;
	struct input_dev *input;

	struct i2c_client *pmm_i2c;
	int irq_pwronr;
	int irq_pwronf;
	struct power_button_data button_data[0];
};

static int power_keys_wake_lock_timeout(struct device *dev, long timeout)
{
	struct wakeup_source *ws = NULL;

	if (!dev->power.wakeup) {
		dev_err(dev, "Not register wakeup source\n");
		goto err;
	}

	ws = dev->power.wakeup;
	__pm_wakeup_event(ws, jiffies_to_msecs(timeout));

	return 0;
err:
	return -1;
}

static void power_keys_power_report_event(struct power_button_data *bdata)
{
	const struct power_keys_button *button = bdata->button;
	struct input_dev *input = bdata->input;
	struct power_keys_drvdata *ddata = input_get_drvdata(input);
	unsigned int type = button->type ?: EV_KEY;
	int state = bdata->key_pressed;

	if (power_keys_wake_lock_timeout(ddata->dev, WAKELOCK_TIME) < 0) {
		dev_err(ddata->dev, "power_keys_wake_lock_timeout fail\n");
		return;
	}

	/* Report new key event */
	input_event(input, type, button->code, !!state);

	/* Sync new input event */
	input_sync(input);
}

static void s2mpg12_keys_work_func(struct work_struct *work)
{
	struct power_button_data *bdata = container_of(work,
						      struct power_button_data,
						      key_work.work);

	power_keys_power_report_event(bdata);

	if (bdata->button->wakeup)
		pm_relax(bdata->input->dev.parent);
}

static irqreturn_t power_keys_rising_irq_handler(int irq, void *dev_id)
{
	struct power_keys_drvdata *ddata = dev_id;
	int i = 0;

	for (i = 0; i < ddata->pdata->nbuttons; i++) {
		struct power_button_data *bdata = &ddata->button_data[i];

		bdata->key_pressed = true;

		if (bdata->button->wakeup) {
			const struct power_keys_button *button = bdata->button;

			pm_stay_awake(bdata->input->dev.parent);
			if (bdata->suspended  &&
			    (button->type == 0 || button->type == EV_KEY)) {
				/*
				 * Simulate wakeup key press in case the key has
				 * already released by the time we got interrupt
				 * handler to run.
				 */
				input_report_key(bdata->input, button->code, 1);
			}
		}

		queue_delayed_work(bdata->irq_wqueue, &bdata->key_work, 0);
	}

	return IRQ_HANDLED;
}

static irqreturn_t power_keys_falling_irq_handler(int irq, void *dev_id)
{
	struct power_keys_drvdata *ddata = dev_id;
	int i = 0;

	for (i = 0; i < ddata->pdata->nbuttons; i++) {
		struct power_button_data *bdata = &ddata->button_data[i];

		bdata->key_pressed = false;
		queue_delayed_work(bdata->irq_wqueue, &bdata->key_work, 0);
	}

	return IRQ_HANDLED;
}

static void power_keys_report_state(struct power_keys_drvdata *ddata)
{
	struct input_dev *input = ddata->input;
	int i;

	for (i = 0; i < ddata->pdata->nbuttons; i++) {
		struct power_button_data *bdata = &ddata->button_data[i];

		bdata->key_pressed = false;
		power_keys_power_report_event(bdata);
	}
	input_sync(input);
}

static int power_keys_open(struct input_dev *input)
{
	struct power_keys_drvdata *ddata = input_get_drvdata(input);

	dev_info(ddata->dev, "%s()\n", __func__);

	power_keys_report_state(ddata);

	return 0;
}

static void power_keys_close(struct input_dev *input)
{
	struct power_keys_drvdata *ddata = input_get_drvdata(input);

	dev_info(ddata->dev, "%s()\n", __func__);
}

#if IS_ENABLED(CONFIG_OF)
static struct power_keys_platform_data *
power_keys_get_devtree_pdata(struct s2mpg12_dev *iodev)
{
	#define S2MPG12_SUPPORT_KEY_NUM	(1)
	struct device *dev = iodev->dev;
	struct device_node *mfd_np, *key_np, *pp;
	struct power_keys_platform_data *pdata;
	struct power_keys_button *button;
	int error, nbuttons, i;
	size_t size;

	if (!iodev->dev->of_node) {
		dev_err(dev, "could not find iodev node\n");
		error = -ENODEV;
		goto err_out;
	}

	mfd_np = iodev->dev->of_node;
	if (!mfd_np) {
		dev_err(dev, "could not find parent_node\n");
		error = -ENODEV;
		goto err_out;
	}

	key_np = of_find_node_by_name(mfd_np, "s2mpg12-keys");
	if (!key_np) {
		dev_err(dev, "could not find current_node\n");
		error = -ENOENT;
		goto err_out;
	}

	nbuttons = of_get_child_count(key_np);
	if (nbuttons > S2MPG12_SUPPORT_KEY_NUM || nbuttons == 0) {
		dev_warn(dev, "it support only one button(%d)\n",
			 nbuttons);
		error = -ENODEV;
		goto err_out;
	}

	size = sizeof(*pdata) + nbuttons * sizeof(*button);
	pdata = devm_kzalloc(dev, size, GFP_KERNEL);
	if (!pdata) {
		error = -ENOMEM;
		goto err_out;
	}

	pdata->buttons = (struct power_keys_button *)(pdata + 1);
	pdata->nbuttons = nbuttons;

	i = 0;
	for_each_child_of_node(key_np, pp) {
		button = &pdata->buttons[i++];
		if (of_property_read_u32(pp, "linux,code", &button->code)) {
			error = -EINVAL;
			goto err_out;
		}

		button->desc = of_get_property(pp, "label", NULL);

		of_property_read_u32(pp, "wakeup", &button->wakeup);

		if (of_property_read_u32(pp, "linux,input-type", &button->type))
			button->type = EV_KEY;
		if (of_property_read_u32(pp, "force_key_irq_en", &force_key_irq_en))
			force_key_irq_en = 0;
	}

	return pdata;
err_out:
	return ERR_PTR(error);
}

static const struct of_device_id power_keys_of_match[] = {
	{ .compatible = "s2mpg12-power-keys", },
	{ },
};
MODULE_DEVICE_TABLE(of, power_keys_of_match);
#else
static inline struct power_keys_platform_data *
power_keys_get_devtree_pdata(struct s2mpg12_dev *iodev)
{
	return ERR_PTR(-ENODEV);
}
#endif

static void power_remove_key(struct power_button_data *bdata)
{
	pr_info("%s()\n", __func__); // it will be removed.

	cancel_delayed_work_sync(&bdata->key_work);
}

static void power_keys_force_en_irq(struct power_keys_drvdata *ddata)
{
	if (!force_key_irq_en)
		return;

	enable_irq(ddata->irq_pwronf);
	enable_irq(ddata->irq_pwronr);
}

static int power_keys_set_interrupt(struct power_keys_drvdata *ddata, int irq_base)
{
	struct device *dev = ddata->dev;
	int ret = 0;

	ddata->irq_pwronr = irq_base + S2MPG12_IRQ_PWRONR_INT1;
	ddata->irq_pwronf = irq_base + S2MPG12_IRQ_PWRONF_INT1;

	ret = devm_request_threaded_irq(dev, ddata->irq_pwronr, NULL,
					power_keys_rising_irq_handler, 0,
					"pwronr-irq", ddata);
	if (ret < 0) {
		dev_err(dev, "fail to request pwronr-irq: %d: %d\n",
			ddata->irq_pwronr, ret);
		goto err;
	}

	ret = devm_request_threaded_irq(dev, ddata->irq_pwronf, NULL,
					power_keys_falling_irq_handler, 0,
					"pwronf-irq", ddata);
	if (ret < 0) {
		dev_err(dev, "fail to request pwronf-irq: %d: %d\n",
			ddata->irq_pwronf, ret);
		goto err;
	}

	return 0;
err:
	return -1;
}

static int power_keys_set_buttondata(struct power_keys_drvdata *ddata,
				     struct input_dev *input, int *wakeup)
{
	int cnt = 0;

	for (cnt = 0; cnt < ddata->pdata->nbuttons; cnt++) {
		struct power_keys_button *button = &ddata->pdata->buttons[cnt];
		struct power_button_data *bdata = &ddata->button_data[cnt];
		char device_name[32] = {0, };

		bdata->input = input;
		bdata->button = button;

		if (button->wakeup)
			*wakeup = 1;

		/* Dynamic allocation for workqueue name */
		snprintf(device_name, sizeof(device_name) - 1,
			 "power-keys-wq%d@%s", cnt, dev_name(ddata->dev));

		bdata->irq_wqueue = create_singlethread_workqueue(device_name);
		if (!bdata->irq_wqueue) {
			dev_err(ddata->dev, "fail to create workqueue\n");
			goto err;
		}
		INIT_DELAYED_WORK(&bdata->key_work, s2mpg12_keys_work_func);

		input_set_capability(input, button->type ?: EV_KEY, button->code);
	}

	return cnt;
err:
	return -1;
}

static struct power_keys_drvdata *
power_keys_set_drvdata(struct platform_device *pdev,
		       struct power_keys_platform_data *pdata,
			struct input_dev *input, struct s2mpg12_dev *iodev)
{
	struct power_keys_drvdata *ddata = NULL;
	struct device *dev = &pdev->dev;
	size_t size;

	size = sizeof(*ddata) + pdata->nbuttons * sizeof(struct power_button_data);
	ddata = devm_kzalloc(dev, size, GFP_KERNEL);
	if (!ddata)
		return ERR_PTR(-ENOMEM);

	ddata->dev	= dev;
	ddata->pdata	= pdata;
	ddata->input	= input;
	ddata->pmm_i2c	= iodev->pmic;

	platform_set_drvdata(pdev, ddata);
	input_set_drvdata(input, ddata);

	return ddata;
}

static int power_keys_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct s2mpg12_dev *iodev = dev_get_drvdata(pdev->dev.parent);
	struct power_keys_platform_data *pdata = NULL;
	struct power_keys_drvdata *ddata = NULL;
	struct input_dev *input;
	int ret = 0, count = 0;
	int wakeup = 0;

	pr_info("%s: start\n", __func__); // it will be removed.

	pdata = power_keys_get_devtree_pdata(iodev);
	if (IS_ERR(pdata))
		return PTR_ERR(pdata);

	input = devm_input_allocate_device(dev);
	if (!input) {
		dev_err(dev, "failed to allocate state\n");
		ret = -ENOMEM;
		goto fail1;
	}

	input->name		= pdata->name ? : pdev->name;
	input->phys		= "s2mpg12-keys/input0";
	input->dev.parent	= dev;
	input->open		= power_keys_open;
	input->close		= power_keys_close;

	input->id.bustype	= BUS_I2C;
	input->id.vendor	= 0x0001;
	input->id.product	= 0x0001;
	input->id.version	= 0x0100;

	ddata = power_keys_set_drvdata(pdev, pdata, input, iodev);
	if (IS_ERR(ddata)) {
		dev_err(dev, "power_keys_set_drvdata fail\n");
		return PTR_ERR(ddata);
	}

	ret = power_keys_set_buttondata(ddata, input, &wakeup);
	if (ret < 0) {
		dev_err(dev, "power_keys_set_buttondata fail\n");
		goto fail1;
	} else {
		count = ret;
	}

	ret = device_init_wakeup(dev, wakeup);
	if (ret < 0) {
		dev_err(dev, "device_init_wakeup fail(%d)\n", ret);
		goto fail1;
	}

	ret = power_keys_set_interrupt(ddata, iodev->pdata->irq_base);
	if (ret < 0) {
		dev_err(dev, "power_keys_set_interrupt fail\n");
		goto fail1;
	}

	ret = input_register_device(input);
	if (ret) {
		dev_err(dev, "unable to register input device(%d)\n", ret);
		goto fail2;
	}

	power_keys_force_en_irq(ddata);

	dev_info(dev, "S2MPG12 Key probe End\n");

	return 0;

fail2:
	while (--count >= 0) {
		struct power_button_data *bdata = &ddata->button_data[count];

		if (bdata->irq_wqueue)
			destroy_workqueue(bdata->irq_wqueue);

		power_remove_key(bdata);
	}

	platform_set_drvdata(pdev, NULL);

fail1:
	return ret;
}

static int power_keys_remove(struct platform_device *pdev)
{
	struct power_keys_drvdata *ddata = platform_get_drvdata(pdev);
	struct input_dev *input = ddata->input;
	int i;

	device_init_wakeup(&pdev->dev, 0);

	for (i = 0; i < ddata->pdata->nbuttons; i++) {
		struct power_button_data *bdata = &ddata->button_data[i];

		if (bdata->irq_wqueue)
			destroy_workqueue(bdata->irq_wqueue);

		power_remove_key(bdata);
	}

	input_unregister_device(input);

	return 0;
}

#if IS_ENABLED(CONFIG_PM_SLEEP)
static int power_keys_suspend(struct device *dev)
{
	struct power_keys_drvdata *ddata = dev_get_drvdata(dev);
	int i;

	if (device_may_wakeup(dev)) {
		for (i = 0; i < ddata->pdata->nbuttons; i++) {
			struct power_button_data *bdata = &ddata->button_data[i];

			bdata->suspended = true;
		}
	}

	return 0;
}

static int power_keys_resume(struct device *dev)
{
	struct power_keys_drvdata *ddata = dev_get_drvdata(dev);
	int i;

	if (device_may_wakeup(dev)) {
		for (i = 0; i < ddata->pdata->nbuttons; i++) {
			struct power_button_data *bdata = &ddata->button_data[i];

			bdata->suspended = false;
		}
	}

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(power_keys_pm_ops, power_keys_suspend, power_keys_resume);

static struct platform_driver power_keys_device_driver = {
	.probe		= power_keys_probe,
	.remove		= power_keys_remove,
	.driver		= {
		.name	= "s2mpg12-power-keys",
		.owner	= THIS_MODULE,
		.pm	= &power_keys_pm_ops,
		.of_match_table = of_match_ptr(power_keys_of_match),
	}
};

static int __init power_keys_init(void)
{
	return platform_driver_register(&power_keys_device_driver);
}

static void __exit power_keys_exit(void)
{
	platform_driver_unregister(&power_keys_device_driver);
}

device_initcall(power_keys_init);
module_exit(power_keys_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Samsung Electronics");
MODULE_DESCRIPTION("Keyboard driver for s2mpg12");
MODULE_ALIAS("platform:s2mpg12 Power key");
