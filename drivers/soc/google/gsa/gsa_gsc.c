// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020 Google LLC
 */
#include <linux/cdev.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/gsc.h>
#include <linux/idr.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/uio.h>

#include <linux/gsa/gsa_tpu.h>

#include "gsa_mbox.h"
#include "gsa_priv.h"

#define MAX_DEVICES	1

#define GSC_TPM_READ	0x80000000
#define MAX_DATA_SIZE	2048

static struct class *gsc_class;
static dev_t gsc_devt;

static DEFINE_IDR(gsc_devices);

struct gsc_cdn {
	dev_t devt;
	struct cdev cdev;
	struct device *chr_dev;
};

struct gsc_state {
	struct device *dev;
	struct gsc_cdn cdn;
	void *bbuf;
	dma_addr_t bbuf_da;
	size_t bbuf_sz;
	struct mutex bbuf_lock; /* protects bounce buffer */
	int ctdl_ap_irq;
	wait_queue_head_t waitq;
	atomic_t users;
};

/* enum gsc_nos_call_args - parameter layout for GSC NOS call request */
enum gsc_nos_call_args {
	GSC_NOS_CALL_APP_IDX = 0,
	GSC_NOS_CALL_PARAM_IDX,
	GSC_NOS_CALL_ARGS_ADDR_LO_IDX,
	GSC_NOS_CALL_ARGS_ADDR_HI_IDX,
	GSC_NOS_CALL_ARGS_LEN_IDX,
	GSC_NOS_CALL_REPLY_ADDR_LO_IDX,
	GSC_NOS_CALL_REPLY_ADDR_HI_IDX,
	GSC_NOS_CALL_REPLY_SIZE_IDX,
	GSC_NOS_CALL_ARGC,
};

/* enum gsc_nos_call_rsp - parameter layout for GSC NOS call response */
enum gsc_nos_call_rsp {
	GSC_NOS_CALL_RSP_STATUS_IDX = 0,
	GSC_NOS_CALL_RSP_REPLY_LEN_IDX,
	GSC_NOS_CALL_RSP_ARGC,
};

static int gsc_tpm_datagram(struct gsc_state *s,
			    unsigned int cmd,
			    unsigned long arg)
{
	int ret;
	struct gsc_ioc_tpm_datagram dg;
	u32 req[4];

	/* check size */
	if ((size_t)_IOC_SIZE(cmd) != sizeof(dg))
		return -EINVAL;

	/* copy in request */
	if (copy_from_user(&dg, (const void __user *)arg, sizeof(dg)))
		return -EFAULT;

	dev_dbg(s->dev, "TPM: cmd = 0x%08x: len %u\n", dg.command, dg.len);

	if (dg.len > MAX_DATA_SIZE)
		return -E2BIG;

	mutex_lock(&s->bbuf_lock);
	if (!(dg.command & GSC_TPM_READ)) {
		/* copy in data */
		if (copy_from_user(s->bbuf, (const void __user *)dg.buf,
				   dg.len)) {
			ret = -EFAULT;
			goto out;
		}
	}

	/* send command */
	req[0] = dg.command;
	req[1] = dg.len;
	req[2] = (u32)(s->bbuf_da);
	req[3] = (u32)(s->bbuf_da >> 32);
	ret = gsa_send_cmd(s->dev->parent, GSA_MB_CMD_GSC_TPM_DATAGRAM,
			   req, 4, NULL, 0);
	if (ret < 0)
		goto out;

	if (dg.command & GSC_TPM_READ) {
		/* copy out data */
		if (copy_to_user((void __user *)dg.buf, s->bbuf, dg.len)) {
			ret = -EFAULT;
			goto out;
		}
	}
out:
	mutex_unlock(&s->bbuf_lock);
	return ret;
}

static int gsc_nos_call(struct gsc_state *s, unsigned int cmd, unsigned long arg)
{
	int ret;
	struct gsc_ioc_nos_call_req nos_req;
	u32 req[GSC_NOS_CALL_ARGC];
	u32 rsp[GSC_NOS_CALL_RSP_ARGC];

	/* check size */
	if ((size_t)_IOC_SIZE(cmd) != sizeof(nos_req))
		return -EINVAL;

	/* copy in request */
	if (copy_from_user(&nos_req, (const void __user *)arg, sizeof(nos_req)))
		return -EFAULT;

	dev_dbg(s->dev,
		"nos_call: app_id = %d, params = %d, arg_len = %d, reply_len = %d\n",
		nos_req.app_id, nos_req.params, nos_req.arg_len, nos_req.reply_len);

	if (nos_req.reserved != 0)
		return -EINVAL;

	if ((nos_req.arg_len > s->bbuf_sz) || (nos_req.reply_len > s->bbuf_sz))
		return -E2BIG;

	mutex_lock(&s->bbuf_lock);
	if (nos_req.arg_len) {
		/* copy in data */
		if (copy_from_user(s->bbuf, (const void __user *)nos_req.buf, nos_req.arg_len)) {
			ret = -EFAULT;
			goto out;
		}
	}

	/* send command */
	req[GSC_NOS_CALL_APP_IDX] = nos_req.app_id;
	req[GSC_NOS_CALL_PARAM_IDX] = nos_req.params;
	req[GSC_NOS_CALL_ARGS_ADDR_LO_IDX] = (u32)(s->bbuf_da);
	req[GSC_NOS_CALL_ARGS_ADDR_HI_IDX] = (u32)(s->bbuf_da >> 32);
	req[GSC_NOS_CALL_ARGS_LEN_IDX] = nos_req.arg_len;
	req[GSC_NOS_CALL_REPLY_ADDR_LO_IDX] = (u32)(s->bbuf_da);
	req[GSC_NOS_CALL_REPLY_ADDR_HI_IDX] = (u32)(s->bbuf_da >> 32);
	req[GSC_NOS_CALL_REPLY_SIZE_IDX] = nos_req.reply_len;

	ret = gsa_send_cmd(s->dev->parent, GSA_MB_CMD_GSC_NOS_CALL,
			   req, GSC_NOS_CALL_ARGC,
			   rsp, GSC_NOS_CALL_RSP_ARGC);
	if (ret < 0)
		goto out;

	/* get nos_call status and reply data size */
	nos_req.call_status = rsp[GSC_NOS_CALL_RSP_STATUS_IDX];
	nos_req.reply_len = rsp[GSC_NOS_CALL_RSP_REPLY_LEN_IDX];

	/* copy out nos_req to caller */
	if (copy_to_user((void __user *)arg, &nos_req, sizeof(nos_req))) {
		ret = -EFAULT;
		goto out;
	}

	if (nos_req.reply_len) {
		/* copy out data */
		if (copy_to_user((void __user *)nos_req.buf, s->bbuf, nos_req.reply_len)) {
			ret = -EFAULT;
			goto out;
		}
	}

out:
	mutex_unlock(&s->bbuf_lock);
	return ret;
}

static int gsc_reset(struct gsc_state *s)
{
	int rc;

	dev_info(s->dev, "GSC Reset\n");

	/* send hard reset command */
	rc = gsa_send_simple_cmd(s->dev->parent, GSA_MB_CMD_GSC_HARD_RESET);
	if (rc < 0) {
		dev_err(s->dev, "failed (%d) to hard reset GSC\n", rc);
		return rc;
	}

	/*
	 * On success, wait 100 msec to mimic behavior of SPI GSC driver
	 * It takes 200+ msec for device to come back
	 */
	msleep(100);
	return 0;
}

static long gsc_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct gsc_state *s = filp->private_data;

	if (_IOC_TYPE(cmd) != GSC_IOC_MAGIC)
		return -ENOTTY;

	switch (cmd) {
	case GSC_IOC_TPM_DATAGRAM:
		return gsc_tpm_datagram(s, cmd, arg);

	case GSC_IOC_GSA_NOS_CALL:
		return gsc_nos_call(s, cmd, arg);

	case GSC_IOC_RESET:
		return gsc_reset(s);

	default:
		dev_err(s->dev, "Unhandled ioctl cmd: 0x%x\n", cmd);
		return -ENOTTY;
	}
}

static unsigned int gsc_poll(struct file *filp, poll_table *wait)
{
	struct gsc_state *s = filp->private_data;

	poll_wait(filp, &s->waitq, wait);

	return gpio_get_value(s->ctdl_ap_irq) ? POLLIN : 0;
}

static int gsc_open(struct inode *inode, struct file *filp)
{
	struct gsc_cdn *cdn;
	struct gsc_state *s;

	cdn = container_of(inode->i_cdev, struct gsc_cdn, cdev);
	s = container_of(cdn, struct gsc_state, cdn);

	/* we only support 1 user at the same time */
	if (!atomic_add_unless(&s->users, 1, 1))
		return -EBUSY;

	filp->private_data = s;
	nonseekable_open(inode, filp);

	return 0;
}

static int gsc_release(struct inode *inode, struct file *filp)
{
	struct gsc_state *s = filp->private_data;

	atomic_dec(&s->users);
	return 0;
}

static const struct file_operations gsc_fops = {
	.open		= gsc_open,
	.poll		= gsc_poll,
	.release	= gsc_release,
	.unlocked_ioctl	= gsc_ioctl,
	.llseek		= no_llseek,
	.owner		= THIS_MODULE,
};

static irqreturn_t gsc_irq_handler(int irq, void *handle)
{
	struct gsc_state *s = (struct gsc_state *)handle;

	wake_up_interruptible(&s->waitq);

	return IRQ_HANDLED;
}

static int create_cdev_node(struct device *parent,
			    struct gsc_cdn *cdn,
			    const char *name)
{
	int ret;

	/* allocate minor */
	ret = idr_alloc(&gsc_devices, cdn, 0, MAX_DEVICES - 1, GFP_KERNEL);
	if (ret < 0) {
		dev_err(parent, "%s: failed (%d) to get id\n",
			__func__, ret);
		return ret;
	}
	cdn->devt = MKDEV(MAJOR(gsc_devt), ret);

	/* Create device node */
	cdn->chr_dev = device_create(gsc_class, parent, cdn->devt, NULL,
				     "%s%d", name, MINOR(cdn->devt));
	if (IS_ERR(cdn->chr_dev)) {
		ret = PTR_ERR(cdn->chr_dev);
		dev_err(parent, "%s: device_create failed: %d\n",
			__func__, ret);
		goto err_device_create;
	}

	/* Add character device */
	cdn->cdev.owner = THIS_MODULE;
	cdev_init(&cdn->cdev, &gsc_fops);
	ret = cdev_add(&cdn->cdev, cdn->devt, 1);
	if (ret) {
		dev_err(parent, "%s: cdev_add failed (%d)\n",
			__func__, ret);
		goto err_add_cdev;
	}

	return 0;

err_add_cdev:
	device_destroy(gsc_class, cdn->devt);
err_device_create:
	idr_remove(&gsc_devices, MINOR(cdn->devt));
	return ret;
}

static int gsa_gsc_probe(struct platform_device *pdev)
{
	int ret;
	struct gsc_state *s;
	struct device *dev = &pdev->dev;
	const char *gpio_name = "gsc,ctdl_ap_irq";

	/* allocate driver state */
	s = devm_kzalloc(dev, sizeof(*s), GFP_KERNEL);
	if (!s)
		return -ENOMEM;

	s->dev = dev;
	mutex_init(&s->bbuf_lock);
	platform_set_drvdata(pdev, s);

	init_waitqueue_head(&s->waitq);
	atomic_set(&s->users, 0);

	/* allocate bounce buffer to hold datagram */
	s->bbuf_sz = PAGE_SIZE;
	s->bbuf = dmam_alloc_coherent(dev->parent, PAGE_SIZE,
				      &s->bbuf_da, GFP_KERNEL);
	if (!s->bbuf)
		return -ENOMEM;

	/* setup ctdl_ap_irq  */
	ret = of_get_named_gpio(dev->of_node, gpio_name, 0);
	if (ret < 0) {
		dev_err(dev, "of_get_named_gpio failed (%d)\n", ret);
		return ret;
	}
	s->ctdl_ap_irq = ret;

	ret = devm_gpio_request(dev, s->ctdl_ap_irq, gpio_name);
	if (ret) {
		dev_err(dev, "devm_gpio_request failed (%d)\n", ret);
		return ret;
	}

	ret = devm_request_irq(dev,
			       gpio_to_irq(s->ctdl_ap_irq),
			       gsc_irq_handler,
			       IRQF_TRIGGER_RISING | IRQF_ONESHOT,
			       dev_name(dev), s);
	if (ret) {
		dev_err(s->dev, "devm_request_irq failed (%d)\n", ret);
		return ret;
	}

	enable_irq_wake(gpio_to_irq(s->ctdl_ap_irq));

	/* create device node */
	ret = create_cdev_node(dev, &s->cdn, "gsc");
	if (ret < 0)
		return ret;

	dev_info(dev, "Initiliazed\n");
	return 0;
}

static int gsa_gsc_remove(struct platform_device *pdev)
{
	struct gsc_state *s = platform_get_drvdata(pdev);

	cdev_del(&s->cdn.cdev);
	device_destroy(gsc_class, s->cdn.devt);

	return 0;
}

static const struct of_device_id gsa_gsc_of_match[] = {
	{ .compatible = "google,gs101-gsa-gsc-v1", },
	{},
};
MODULE_DEVICE_TABLE(of, gsa_gsc_of_match);

static struct platform_driver gsa_gsc_driver = {
	.probe = gsa_gsc_probe,
	.remove = gsa_gsc_remove,
	.driver = {
		.name = "gsa-gsc",
		.of_match_table = gsa_gsc_of_match,
	},
};

static int __init gsa_gsc_driver_init(void)
{
	int ret;

	ret = alloc_chrdev_region(&gsc_devt, 0, MAX_DEVICES, KBUILD_MODNAME);
	if (ret) {
		pr_err("%s: failed (%d) to alloc chdev region\n",
		       __func__, ret);
		return ret;
	}

	gsc_class = class_create(THIS_MODULE, KBUILD_MODNAME);
	if (IS_ERR(gsc_class)) {
		ret = PTR_ERR(gsc_class);
		pr_err("%s: failed (%d) to device class\n", __func__, ret);
		goto err_class_create;
	}

	ret = platform_driver_register(&gsa_gsc_driver);
	if (ret < 0)
		goto err_driver_register;

	return ret;

err_driver_register:
	class_destroy(gsc_class);
err_class_create:
	unregister_chrdev_region(gsc_devt, MAX_DEVICES);
	return ret;
}

static void __exit gsa_gsc_driver_exit(void)
{
	platform_driver_unregister(&gsa_gsc_driver);
	class_destroy(gsc_class);
	unregister_chrdev_region(gsc_devt, MAX_DEVICES);
}

module_init(gsa_gsc_driver_init);
module_exit(gsa_gsc_driver_exit);

MODULE_LICENSE("GPL v2");
