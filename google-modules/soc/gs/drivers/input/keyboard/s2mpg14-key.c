// SPDX-License-Identifier: GPL-2.0+
/*
 * Driver for Power keys on s2mpg14 IC by PWRON rising, falling interrupts.
 *
 * s2mpg14-keys.c
 * S2MPG14 Keyboard Driver
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
#include <linux/mfd/samsung/s2mpg14-key.h>
#include <linux/mfd/samsung/s2mpg14.h>
#include <linux/mfd/samsung/s2mpg14-register.h>

#define WAKELOCK_TIME		(HZ / 10)

struct power_button_data {
	struct power_keys_button *button;
	struct input_dev *input;
	struct work_struct work;
	struct delayed_work key_work;
	struct workqueue_struct *irq_wqueue;
	bool key_pressed;
};

struct power_keys_drvdata {
	struct device *dev;
	struct s2mpg14_platform_data *s2mpg14_pdata;
	const struct power_keys_platform_data *pdata;
	struct input_dev *input;

	struct i2c_client *pmm_i2c;
	int irq_pwronr;
	int irq_pwronf;
	struct power_button_data button_data[0];
	bool suspended;
};

static int power_keys_wake_lock_timeout(struct device *dev, long timeout)
{
	struct wakeup_source *ws = NULL;

	if (!dev->power.wakeup) {
		dev_err(dev, "Not register wakeup source\n");
		return -1;
	}

	ws = dev->power.wakeup;
	__pm_wakeup_event(ws, jiffies_to_msecs(timeout));

	return 0;
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

static void s2mpg14_keys_work_func(struct work_struct *work)
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
	int i;

	for (i = 0; i < ddata->pdata->nbuttons; i++) {
		struct power_button_data *bdata = &ddata->button_data[i];

		bdata->key_pressed = true;

		if (bdata->button->wakeup) {
			const struct power_keys_button *button = bdata->button;

			pm_stay_awake(bdata->input->dev.parent);
			if (ddata->suspended &&
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
	int i;

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

	power_keys_report_state(ddata);

	return 0;
}

static void power_keys_close(struct input_dev *input)
{
}

#if IS_ENABLED(CONFIG_OF)
#define S2MPG14_SUPPORT_KEY_NUM	(1)

static struct power_keys_platform_data *
power_keys_get_devtree_pdata(struct s2mpg14_dev *iodev)
{
	struct device *dev = iodev->dev;
	struct device_node *mfd_np, *key_np, *pp;
	struct power_keys_platform_data *pdata;
	struct power_keys_button *button;
	int nbuttons, i;
	size_t size;

	if (!iodev->dev->of_node) {
		dev_err(dev, "could not find iodev node\n");
		return ERR_PTR(-ENODEV);
	}

	mfd_np = iodev->dev->of_node;
	if (!mfd_np) {
		dev_err(dev, "could not find parent_node\n");
		return ERR_PTR(-ENODEV);
	}

	key_np = of_find_node_by_name(mfd_np, "s2mpg14-keys");
	if (!key_np) {
		dev_err(dev, "could not find current_node\n");
		return ERR_PTR(-ENOENT);
	}

	nbuttons = of_get_child_count(key_np);
	if (nbuttons > S2MPG14_SUPPORT_KEY_NUM || nbuttons == 0) {
		dev_warn(dev, "it support only one button(%d)\n",
			 nbuttons);
		return ERR_PTR(-ENODEV);
	}

	size = sizeof(*pdata) + nbuttons * sizeof(*button);
	pdata = devm_kzalloc(dev, size, GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	pdata->buttons = (struct power_keys_button *)(pdata + 1);
	pdata->nbuttons = nbuttons;

	i = 0;
	for_each_child_of_node(key_np, pp) {
		button = &pdata->buttons[i++];
		if (of_property_read_u32(pp, "linux,code", &button->code))
			return ERR_PTR(-EINVAL);

		button->desc = of_get_property(pp, "label", NULL);

		of_property_read_u32(pp, "wakeup", &button->wakeup);

		if (of_property_read_u32(pp, "linux,input-type", &button->type))
			button->type = EV_KEY;
		if (of_property_read_u32(pp, "force_key_irq_en", &button->force_key_irq_en))
			button->force_key_irq_en = 0;
	}

	return pdata;
}

static const struct of_device_id power_keys_of_match[] = {
	{ .compatible = "s2mpg14-power-keys", },
	{ },
};
MODULE_DEVICE_TABLE(of, power_keys_of_match);
#else
static inline struct power_keys_platform_data *
power_keys_get_devtree_pdata(struct s2mpg14_dev *iodev)
{
	return ERR_PTR(-ENODEV);
}
#endif

static void power_remove_key(struct power_button_data *bdata)
{
	cancel_delayed_work_sync(&bdata->key_work);
}

static void power_keys_force_en_irq(struct power_keys_drvdata *ddata)
{
	if (!ddata->pdata->buttons->force_key_irq_en)
		return;

	enable_irq(ddata->irq_pwronf);
	enable_irq(ddata->irq_pwronr);
}

static int power_keys_set_interrupt(struct power_keys_drvdata *ddata, int irq_base)
{
	struct device *dev = ddata->dev;
	int ret = 0;

	ddata->irq_pwronr = irq_base + S2MPG14_IRQ_PWRONR_INT1;
	ddata->irq_pwronf = irq_base + S2MPG14_IRQ_PWRONF_INT1;

	ret = devm_request_threaded_irq(dev, ddata->irq_pwronr, NULL,
					power_keys_rising_irq_handler, 0,
					"pwronr-irq", ddata);
	if (ret < 0) {
		dev_err(dev, "fail to request pwronr-irq: %d: %d\n",
			ddata->irq_pwronr, ret);
		return -1;
	}

	ret = devm_request_threaded_irq(dev, ddata->irq_pwronf, NULL,
					power_keys_falling_irq_handler, 0,
					"pwronf-irq", ddata);
	if (ret < 0) {
		dev_err(dev, "fail to request pwronf-irq: %d: %d\n",
			ddata->irq_pwronf, ret);
		return -1;
	}

	return 0;
}

static int power_keys_set_buttondata(struct power_keys_drvdata *ddata,
				     struct input_dev *input, int *wakeup)
{
	int cnt;

	for (cnt = 0; cnt < ddata->pdata->nbuttons; cnt++) {
		struct power_keys_button *button = &ddata->pdata->buttons[cnt];
		struct power_button_data *bdata = &ddata->button_data[cnt];
		char device_name[32] = {0, };

		bdata->input = input;
		bdata->button = button;

		if (button->wakeup)
			*wakeup = 1;

		/* Dynamic allocation for workqueue name */
		snprintf(device_name, sizeof(device_name), "power-keys-wq%d@%s",
			 cnt, dev_name(ddata->dev));

		bdata->irq_wqueue = create_singlethread_workqueue(device_name);
		if (!bdata->irq_wqueue) {
			dev_err(ddata->dev, "fail to create workqueue\n");
			return -1;
		}
		INIT_DELAYED_WORK(&bdata->key_work, s2mpg14_keys_work_func);

		input_set_capability(input, button->type ?: EV_KEY, button->code);
	}

	return cnt;
}

static struct power_keys_drvdata *
power_keys_set_drvdata(struct platform_device *pdev,
		       struct power_keys_platform_data *pdata,
			struct input_dev *input, struct s2mpg14_dev *iodev)
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

static void power_keys_remove_datas(struct power_keys_drvdata *ddata, int count)
{
	int i;

	for (i = 0; i < count; i++) {
		struct power_button_data *bdata = &ddata->button_data[i];

		if (bdata->irq_wqueue)
			destroy_workqueue(bdata->irq_wqueue);

		power_remove_key(bdata);
	}
}

static int power_keys_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct s2mpg14_dev *iodev = dev_get_drvdata(pdev->dev.parent);
	struct power_keys_platform_data *pdata = NULL;
	struct power_keys_drvdata *ddata = NULL;
	struct input_dev *input;
	int ret = 0, count = 0;
	int wakeup = 0;

	pdata = power_keys_get_devtree_pdata(iodev);
	if (IS_ERR(pdata))
		return PTR_ERR(pdata);

	input = devm_input_allocate_device(dev);
	if (!input) {
		dev_err(dev, "failed to allocate state\n");
		return -ENOMEM;
	}

	input->name		= pdata->name ? : pdev->name;
	input->phys		= "s2mpg14-keys/input0";
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
		return ret;
	}

	ret = device_init_wakeup(dev, wakeup);
	if (ret < 0) {
		dev_err(dev, "device_init_wakeup fail(%d)\n", ret);
		return ret;
	}

	ret = power_keys_set_interrupt(ddata, iodev->pdata->irq_base);
	if (ret < 0) {
		dev_err(dev, "power_keys_set_interrupt fail\n");
		return ret;
	}

	ret = input_register_device(input);
	if (ret == 0) {
		power_keys_force_en_irq(ddata);
		return 0;
	}

	power_keys_remove_datas(ddata, count);

	platform_set_drvdata(pdev, NULL);

	return ret;
}

static int power_keys_remove(struct platform_device *pdev)
{
	struct power_keys_drvdata *ddata = platform_get_drvdata(pdev);
	struct input_dev *input = ddata->input;

	device_init_wakeup(&pdev->dev, 0);

	power_keys_remove_datas(ddata, ddata->pdata->nbuttons);

	input_unregister_device(input);

	return 0;
}

#if IS_ENABLED(CONFIG_PM_SLEEP)
static int power_keys_suspend(struct device *dev)
{
	struct power_keys_drvdata *ddata = dev_get_drvdata(dev);

	if (device_may_wakeup(dev))
		ddata->suspended = true;

	return 0;
}

static int power_keys_resume(struct device *dev)
{
	struct power_keys_drvdata *ddata = dev_get_drvdata(dev);

	if (device_may_wakeup(dev))
		ddata->suspended = false;

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(power_keys_pm_ops, power_keys_suspend, power_keys_resume);

static struct platform_driver power_keys_device_driver = {
	.probe		= power_keys_probe,
	.remove		= power_keys_remove,
	.driver		= {
		.name	= "s2mpg14-power-keys",
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
MODULE_DESCRIPTION("Keyboard driver for s2mpg14");
MODULE_ALIAS("platform:s2mpg14 Power key");
