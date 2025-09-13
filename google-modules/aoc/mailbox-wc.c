// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google Whitechapel Mailbox Driver
 *
 * Copyright (c) 2019 Google LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define pr_fmt(fmt) "wc-mbox: " fmt

#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mailbox_controller.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

/* Register offsets */
#define MB_MCUCTLR 0x0000
#define MB_INTGR0 0x0020
#define MB_INTCR0 0x0024
#define MB_INTMR0 0x0028
#define MB_INTSR0 0x002C
#define MB_INTMSR0 0x0030
#define MB_INTGR1 0x0040
#define MB_INTCR1 0x0044
#define MB_INTMR1 0x0048
#define MB_INTSR1 0x004C
#define MB_INTMSR1 0x0050
#define MB_SHARED(i) (0x0080 + (i * 4))

/* General defines */
#define MBOX_WC_INTERRUPTS 16

#define DT_NON_WAKE_CHANNEL_KEY "wc-mbox-non-wake-channels"

static int wc_mbox_probe(struct platform_device *dev);
static int wc_mbox_remove(struct platform_device *dev);
static int wc_mbox_suspend(struct platform_device *dev, pm_message_t state);
static int wc_mbox_resume(struct platform_device *dev);

/* Channel ops */
static int wc_mbox_send_data(struct mbox_chan *chan, void *data);
static int wc_mbox_startup(struct mbox_chan *chan);
static void wc_mbox_shutdown(struct mbox_chan *chan);
static bool wc_mbox_last_tx_done(struct mbox_chan *chan);
static bool wc_mbox_peek_data(struct mbox_chan *chan);

struct wc_mbox_prvdata {
	void __iomem *base;
	struct mbox_controller *mbox;
	u32 sleep_mask;
	u32 non_waking_services;
	int interrupt;
	int shared_registers;
};

static const struct of_device_id wc_mbox_match[] = {
	{
		.compatible = "google,mailbox-whitechapel",
	},
	{},
};
MODULE_DEVICE_TABLE(of, wc_mbox_match);

static struct platform_driver wc_mbox_driver = {
	.probe = wc_mbox_probe,
	.remove = wc_mbox_remove,
	.suspend = wc_mbox_suspend,
	.resume = wc_mbox_resume,
	.driver = {
		.name = "wc-mbox",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(wc_mbox_match),
	},
};

static struct mbox_chan_ops wc_mbox_chan_ops = {
	.send_data = wc_mbox_send_data,
	.startup = wc_mbox_startup,
	.shutdown = wc_mbox_shutdown,
	.last_tx_done = wc_mbox_last_tx_done,
	.peek_data = wc_mbox_peek_data,
};

/* Helper functions */
static int wc_mbox_int_generate(void *base, u8 i)
{
	if (i > MBOX_WC_INTERRUPTS)
		return -EINVAL;

	iowrite32((1 << i), base + MB_INTGR0);
	return 0;
}

static int wc_mbox_int_clear(void *base, u8 i)
{
	if (i > MBOX_WC_INTERRUPTS)
		return -EINVAL;

	iowrite32((1 << i), base + MB_INTCR1);
	return 0;
}

static int wc_mbox_get_interrupt_mask(void *base)
{
	return ioread32(base + MB_INTMR1);
}

static int wc_mbox_set_interrupt_mask(void *base, u16 mask)
{
	u32 value = mask;

	iowrite32(value, base + MB_INTMR1);
	return 0;
}

static int wc_mbox_enable_interrupt(void *base, u16 index)
{
	u32 mask;

	if (index > MBOX_WC_INTERRUPTS)
		return -1;

	mask = wc_mbox_get_interrupt_mask(base);
	mask &= ~(1 << index);
	wc_mbox_set_interrupt_mask(base, mask);

	return 0;
}

static int wc_mbox_disable_interrupt(void *base, u16 index)
{
	u32 mask;

	if (index > MBOX_WC_INTERRUPTS)
		return -1;

	mask = wc_mbox_get_interrupt_mask(base);
	mask |= (1 << index);
	wc_mbox_set_interrupt_mask(base, mask);

	return 0;
}

static irqreturn_t wc_int_handler(int irq, void *dev)
{
	struct wc_mbox_prvdata *prvdata = dev_get_drvdata(dev);
	struct mbox_controller *mbox = prvdata->mbox;
	struct mbox_chan *chans = mbox->chans;
	u32 data[8];
	u32 status;
	int i;

	for (i = 0; i < min(prvdata->shared_registers, 8); i++)
		data[i] = ioread32(prvdata->base + MB_SHARED(i));

	status = ioread32(prvdata->base + MB_INTSR1);

	for (i = 0; i < MBOX_WC_INTERRUPTS; i++) {
		if ((status & (1 << i)) != 0) {
			wc_mbox_int_clear(prvdata->base, i);

			if (chans[i].cl != NULL)
				mbox_chan_received_data(&chans[i], &data);

		}
	}

	return IRQ_HANDLED;
}

static int wc_mbox_channel_index(struct mbox_chan *chan)
{
	struct mbox_controller *mbox = chan->mbox;
	int i;

	for (i = 0; i < MBOX_WC_INTERRUPTS; i++) {
		if (chan == &mbox->chans[i])
			return i;
	}

	return -1;
}

static int wc_mbox_send_data(struct mbox_chan *chan, void *data)
{
	struct wc_mbox_prvdata *prvdata = dev_get_drvdata(chan->mbox->dev);
	u32 *regs = data;
	u32 status;
	int i, index;

	index = wc_mbox_channel_index(chan);
	if (index == -1)
		return -EINVAL;

	/* Wait if the remote has not finished processing the last message */
	if (data) {
		status = ioread32(prvdata->base + MB_INTSR0);
		if ((status & (1 << index)) != 0) {
			pr_debug("Busy with status %x\n", status);
			return -EBUSY;
		}

		for (i = 0; i < prvdata->shared_registers; i++)
			iowrite32(regs[i], prvdata->base + MB_SHARED(i));
	}

	wc_mbox_int_generate(prvdata->base, index);

	return 0;
}

static int wc_mbox_startup(struct mbox_chan *chan)
{
	struct wc_mbox_prvdata *prvdata = dev_get_drvdata(chan->mbox->dev);
	int index = wc_mbox_channel_index(chan);

	if (index < 0)
		return -EINVAL;

	wc_mbox_enable_interrupt(prvdata->base, index);
	return 0;
}

static void wc_mbox_shutdown(struct mbox_chan *chan)
{
	struct wc_mbox_prvdata *prvdata = dev_get_drvdata(chan->mbox->dev);
	int index = wc_mbox_channel_index(chan);

	if (index < 0)
		return;

	wc_mbox_disable_interrupt(prvdata->base, index);
}

static bool wc_mbox_last_tx_done(struct mbox_chan *chan)
{
	struct wc_mbox_prvdata *prvdata = dev_get_drvdata(chan->mbox->dev);
	u32 status;

	status = ioread32(prvdata->base + MB_INTSR0);
	if (status != 0)
		pr_debug("Pending status %x\n", status);

	return status == 0;
}

static bool wc_mbox_peek_data(struct mbox_chan *chan)
{
	struct mbox_controller *mbox = chan->mbox;
	struct wc_mbox_prvdata *prvdata = dev_get_drvdata(mbox->dev);
	u32 status;
	int index;

	index = wc_mbox_channel_index(chan);
	if (index == -1)
		return false;

	status = ioread32(prvdata->base + MB_INTSR1);
	return (status & (1 << index)) != 0;
}

static int wc_mbox_suspend(struct platform_device *pdev, pm_message_t state)
{
	u32 waking_mask;
	struct wc_mbox_prvdata *prvdata = platform_get_drvdata(pdev);

	prvdata->sleep_mask = wc_mbox_get_interrupt_mask(prvdata->base);
	waking_mask = prvdata->sleep_mask & ~(prvdata->non_waking_services);
	wc_mbox_set_interrupt_mask(prvdata->base, waking_mask);
	enable_irq_wake(prvdata->interrupt);
	return 0;
}

static int wc_mbox_resume(struct platform_device *pdev)
{
	struct wc_mbox_prvdata *prvdata = platform_get_drvdata(pdev);

	disable_irq_wake(prvdata->interrupt);
	wc_mbox_set_interrupt_mask(prvdata->base, prvdata->sleep_mask);
	return 0;
}

static int wc_mbox_probe(struct platform_device *pdev)
{
	struct mbox_controller *mbox;
	struct mbox_chan *channels;
	struct wc_mbox_prvdata *prvdata;
	struct device *dev = &pdev->dev;

	struct resource *base;
	size_t region_size;
	u32 non_wake_mask;
	int interrupt;
	int ret;
	int i;

	/* Check for device tree properties */
	interrupt = platform_get_irq(pdev, 0);
	if (interrupt <= 0) {
		dev_err(dev, "No interrupt for device\n");
		return -EINVAL;
	}

	base = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!base) {
		dev_err(dev, "No base address for device\n");
		return -EINVAL;
	}

	mbox = devm_kzalloc(dev, sizeof(*mbox), GFP_KERNEL);
	channels = devm_kcalloc(dev, MBOX_WC_INTERRUPTS, sizeof(struct mbox_chan), GFP_KERNEL);
	prvdata = devm_kzalloc(dev, sizeof(*prvdata), GFP_KERNEL);

	if (!mbox || !channels || !prvdata)
		return -ENOMEM;

	region_size = ((base->end - base->start) + 1);
	prvdata->base = devm_ioremap(dev, base->start, region_size);
	prvdata->interrupt = interrupt;
	prvdata->shared_registers = 8;
	prvdata->mbox = mbox;

	ret = of_property_read_u32(dev->of_node, DT_NON_WAKE_CHANNEL_KEY, &non_wake_mask);
	if (ret == 0) {
		if (prvdata->non_waking_services > 0xffff)
			dev_err(dev, "Invalid non-waking services mask %#10x",
				prvdata->non_waking_services);

		prvdata->non_waking_services = non_wake_mask;
	}

	mbox->num_chans = MBOX_WC_INTERRUPTS;
	mbox->txpoll_period = 2;
	mbox->txdone_poll = true;
	mbox->dev = dev;
	mbox->ops = &wc_mbox_chan_ops;

	for (i = 0; i < MBOX_WC_INTERRUPTS; i++)
		channels[i].mbox = mbox;

	mbox->chans = channels;

	platform_set_drvdata(pdev, prvdata);

	/* Start with interrupts masked */
	wc_mbox_set_interrupt_mask(prvdata->base, 0xffff);

	ret = devm_request_irq(dev, interrupt, wc_int_handler,
			       IRQF_TRIGGER_HIGH, dev_name(dev), dev);
	if (ret != 0) {
		dev_err(dev, "failed to register interrupt handler: %d\n", ret);
		return -ENXIO;
	}

	ret = mbox_controller_register(mbox);
	if (ret != 0) {
		dev_err(dev, "failed to register mailbox controller: %d\n",
			ret);
		return -EINVAL;
	}

	dev_info(dev, "probe completed successfully\n");

	return 0;
}

static int wc_mbox_remove(struct platform_device *pdev)
{
	struct wc_mbox_prvdata *prvdata = platform_get_drvdata(pdev);
	struct device *dev = &(pdev->dev);

	if (!prvdata)
		return -EINVAL;

	dev_dbg(dev, "removing mailbox\n");

	mbox_controller_unregister(prvdata->mbox);

	devm_free_irq(dev, prvdata->interrupt, dev);

	devm_iounmap(dev, prvdata->base);

	return 0;
}

/* Module functions */
static int __init mbox_wc_init(void)
{
	pr_debug("init\n");
	platform_driver_register(&wc_mbox_driver);

	return 0;
}

static void __exit mbox_wc_exit(void)
{
	pr_debug("exit\n");
	platform_driver_unregister(&wc_mbox_driver);
}

module_init(mbox_wc_init);
module_exit(mbox_wc_exit);

MODULE_DESCRIPTION("Whitechapel Mailbox Driver");
MODULE_AUTHOR("Craig Dooley (Google)");
MODULE_LICENSE("GPL v2");
