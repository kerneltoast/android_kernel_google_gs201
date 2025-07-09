// SPDX-License-Identifier: GPL-2.0-only
/*
 * GSC transport driver
 *
 * Copyright (C) 2020 Google LLC
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/poll.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>
#include <linux/spi/spi.h>
#include <linux/gsc.h>
#include <linux/wait.h>

#define GSC_TPM_READ	0x80000000

#define MAX_DATA_SIZE 2048
#define GSC_MAX_DEVICES 1
#define GSC_TPM_TIMEOUT_MS	10

struct gsc_data {
	dev_t			devt;
	struct cdev		cdev;
	struct spi_device	*spi;
	int			ctdl_ap_irq;
	wait_queue_head_t	waitq;
	int			ctdl_rst;
	atomic_t		users;
	void			*tx_buf;
	void			*rx_buf;
};

static struct class *gsc_class;
static dev_t gsc_devt;

static int gsc_is_awake(struct gsc_data *gsc)
{
	static const u8 mosi[6] = { 0x80, 0xd4, 0x0f, 0x04, 0x00, 0x00 };
	u8 miso[sizeof(mosi)];
	struct spi_device *spi = gsc->spi;
	struct spi_message m;
	struct spi_transfer spi_xfer = {
		.tx_buf = mosi,
		.rx_buf = miso,
		.len = sizeof(mosi),
		.cs_change = 0,
	};
	int ret;

	spi_message_init(&m);
	spi_message_add_tail(&spi_xfer, &m);
	ret = spi_sync_locked(spi, &m);
	if (ret)
		return ret;

	/* If GSC is awake, miso will have 0x00 0x00 0x00 0x00 0x01 0xd1 */
	if (miso[4] == 0x01 && miso[5] == 0xd1)
		return 0;

	/* If GSC is asleep, poking at it like this should start waking it up */
	return -EAGAIN;
}

static int gsc_wait_cmd_done(struct gsc_data *gsc)
{
	struct spi_device *spi = gsc->spi;
	struct spi_message m;
	int ret;
	unsigned long to = jiffies + 1 + /* at least one jiffy */
			   msecs_to_jiffies(GSC_TPM_TIMEOUT_MS);
	struct spi_transfer spi_xfer = {
		.rx_buf = gsc->rx_buf,
		.len = 1,
		.cs_change = 1,
	};
	u8 *val = gsc->rx_buf;
	/*
	 * We have sent the initial four-byte command to GSC on MOSI, and
	 * now we're waiting for bit0 of the MISO byte to be set, indicating
	 * that we can continue with the rest of the transaction. If that bit
	 * is not immediately set, we'll keep sending one more don't-care byte
	 * on MOSI just to read MISO until it is. If GSC is awake and
	 * functioning correctly its hardware implementation should always
	 * return 0x00 while it's thinking and return 0x01 when it's ready to
	 * continue. Any other value indicates that something went wrong.
	 */
	do {
		if (time_after(jiffies, to)) {
			dev_warn(&spi->dev, "GSC SPI timed out\n");
			return -EBUSY;
		}
		spi_message_init(&m);
		spi_message_add_tail(&spi_xfer, &m);
		ret = spi_sync_locked(spi, &m);
		if (ret)
			return ret;
	} while (!*val);

	/* Should be 0x01 */
	return *val == 0x01 ? 0 : -EAGAIN;
}

static int gsc_tpm_datagram(struct gsc_data *gsc,
			    struct gsc_ioc_tpm_datagram *dg)
{
	int is_read = dg->command & GSC_TPM_READ;
	int ret;
	int ignore_result = 0;
	struct spi_device *spi = gsc->spi;
	struct spi_message m;
	struct spi_transfer spi_xfer = {
		.tx_buf = gsc->tx_buf,
		.rx_buf = gsc->rx_buf,
		.len = 4,
		.cs_change = 1,
	};
	u32 *command_ptr = gsc->tx_buf;
	u32 *response_ptr = gsc->rx_buf;
	u32 response_val;
	int gsc_fell_over = 0;

	/* Lock the SPI bus until we're completely done */
	spi_bus_lock(spi->master);

	/* Check whether GSC is awake (b/142475097) */
	ret = gsc_is_awake(gsc);
	if (ret)
		goto exit;

	/* The command must be big-endian on the wire */
	*command_ptr = cpu_to_be32(dg->command);

	/* Prepare to send the command */
	spi_message_init(&m);
	spi_message_add_tail(&spi_xfer, &m);
	/*
	 * Note: When we prepare only one message but .cs_change is 1, it leaves
	 * the chip-select asserted afterwards. We have to send another message
	 * with .cs_change clear to deassert it. I think. This flag has not
	 * always been consistently implemented.
	 */

	/* Send the command out */
	ret = spi_sync_locked(spi, &m);
	if (ret)
		goto exit;

	/*
	 *  As the command is sent over MOSI, the last bit of MISO indicates
	 *  whether or not GSC is ready to continue to the data phase. A zero
	 *  bit means it's not.
	 */
	response_val = be32_to_cpu(*response_ptr);
	if (!(response_val & 0x00000001)) {
		/* Wait for the reply bit to go high */
		ret = gsc_wait_cmd_done(gsc);
		/* Reset CS in the case of unexpected bytes. */
		if (ret)
			gsc_fell_over = 1;
	}

	/* If there's no data or something's wrong, how do we deassert CS? */
	if (!dg->len || gsc_fell_over) {
		/* For now, just transfer one more byte and throw it away */
		is_read = 1;
		dg->len = 1;
		ignore_result = 1;
	}

	/* Now we can transfer the data */
	spi_xfer.cs_change = 0;
	spi_xfer.len = dg->len;
	if (is_read) {
		spi_xfer.rx_buf = gsc->rx_buf;
		spi_xfer.tx_buf = NULL;
	} else {
		spi_xfer.rx_buf = NULL;
		spi_xfer.tx_buf = gsc->tx_buf;

		if (copy_from_user(gsc->tx_buf,
				   (void __user *)dg->buf, dg->len)) {
			ret = -EFAULT;
			goto exit;
		}
	}
	spi_message_init(&m);
	spi_message_add_tail(&spi_xfer, &m);
	ret = spi_sync_locked(spi, &m);
	if (ret)
		goto exit;

	if (gsc_fell_over)
		ret = -EFAULT;

	if (ignore_result)
		goto exit;

	if (is_read && copy_to_user((void __user *)dg->buf,
				    gsc->rx_buf, dg->len))
		ret = -EFAULT;

exit:
	spi_bus_unlock(spi->master);

	return ret;
}

static int gsc_reset(struct gsc_data *gsc)
{
	/* Synchronize with the datagrams by locking the SPI bus */
	struct spi_device *spi = gsc->spi;

	spi_bus_lock(spi->master);

	/* Assert reset for at least 3ms after VDDIOM is stable; 10ms is safe */
	gpio_set_value(gsc->ctdl_rst, 1);
	msleep(10);

	/* Clear reset and wait for GSC to become functional */
	gpio_set_value(gsc->ctdl_rst, 0);
	msleep(100);

	spi_bus_unlock(spi->master);
	return 0;
}

static long
gsc_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	u32 tmp;
	struct gsc_data *gsc = filp->private_data;
	struct gsc_ioc_tpm_datagram dg;

	/* check magic */
	if (_IOC_TYPE(cmd) != GSC_IOC_MAGIC)
		return -ENOTTY;
	switch (cmd) {
	case GSC_IOC_TPM_DATAGRAM:
		tmp = _IOC_SIZE(cmd);
		if (tmp != sizeof(dg))
			return -EINVAL;

		if (copy_from_user(&dg, (void __user *)arg, sizeof(dg)))
			return -EFAULT;

		if (dg.len > MAX_DATA_SIZE)
			return -E2BIG;

		/* translate to spi_message, execute */
		return gsc_tpm_datagram(gsc, &dg);
	case GSC_IOC_RESET:
		return gsc_reset(gsc);
	}
	return -EINVAL;
}

static ssize_t
gsc_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	int ret;
	size_t c = 0;
	size_t len;
	struct gsc_data *gsc = filp->private_data;

	while (count) {
		len = count > MAX_DATA_SIZE ? MAX_DATA_SIZE : count;
		ret = spi_read(gsc->spi, gsc->rx_buf, len);
		if (ret)
			return ret;
		ret = copy_to_user(buf + c, gsc->rx_buf, len);
		if (ret)
			return -EFAULT;
		c += len;
		count -= len;
	}

	return c;
}

static ssize_t
gsc_write(struct file *filp, const char __user *buf,
	  size_t count, loff_t *f_pos)
{
	int ret;
	size_t c = 0;
	size_t len;
	struct gsc_data *gsc = filp->private_data;

	while (count) {
		len = count > MAX_DATA_SIZE ? MAX_DATA_SIZE : count;
		ret = copy_from_user(gsc->tx_buf, buf + c, len);
		if (ret)
			return -EFAULT;
		ret = spi_write(gsc->spi, gsc->tx_buf, len);
		if (ret)
			return ret;
		c += len;
		count -= len;
	}

	return c;
}

static int gsc_open(struct inode *inode, struct file *filp)
{
	struct gsc_data *gsc;

	gsc = container_of(inode->i_cdev, struct gsc_data, cdev);

	/* we only support 1 user at the same time */
	if (!atomic_add_unless(&gsc->users, 1, 1))
		return -EBUSY;

	filp->private_data = gsc;
	nonseekable_open(inode, filp);

	return 0;
}

static unsigned int gsc_poll(struct file *filp, poll_table *wait)
{
	struct gsc_data *gsc = filp->private_data;

	poll_wait(filp, &gsc->waitq, wait);

	return gpio_get_value(gsc->ctdl_ap_irq) ? POLLIN : 0;
}

static int gsc_release(struct inode *inode, struct file *filp)
{
	struct gsc_data *gsc = filp->private_data;

	atomic_dec(&gsc->users);

	return 0;
}

static const struct file_operations gsc_fops = {
	.owner =		THIS_MODULE,
	.write =		gsc_write,
	.read =			gsc_read,
	.open =			gsc_open,
	.poll =			gsc_poll,
	.release =		gsc_release,
	.unlocked_ioctl =	gsc_ioctl,
	.llseek =		no_llseek,
};

#ifdef CONFIG_OF
static const struct of_device_id gsc_dt_ids[] = {
	{ .compatible = "google,gsc" },
	{},
};
MODULE_DEVICE_TABLE(of, gsc_dt_ids);
#endif

static irqreturn_t gsc_irq_handler(int irq, void *handle)
{
	struct gsc_data *gsc = (struct gsc_data *)handle;

	wake_up_interruptible(&gsc->waitq);

	return IRQ_HANDLED;
}

static int gsc_request_named_gpio(struct gsc_data *gsc,
				  const char *label, int *gpio)
{
	struct device *dev = &gsc->spi->dev;
	struct device_node *np = dev->of_node;
	int rc = of_get_named_gpio(np, label, 0);

	if (rc < 0) {
		dev_err(dev, "failed to get '%s'\n", label);
		return rc;
	}
	*gpio = rc;

	rc = devm_gpio_request(dev, *gpio, label);
	if (rc) {
		dev_err(dev, "failed to request gpio %d\n", *gpio);
		return rc;
	}
	dev_dbg(dev, "%s %d\n", label, *gpio);

	return 0;
}

static int gsc_probe(struct spi_device *spi)
{
	struct device *dev;
	struct gsc_data *gsc;
	int ret;
	dev_t devt;
	u32 minor;

	/* use chip select as minor */
	minor = (u32)spi->chip_select;
	if (minor >= GSC_MAX_DEVICES) {
		dev_err(&spi->dev, "minor %u out of boundaries\n", minor);
		return -ENXIO;
	}

	gsc = kmalloc(sizeof(*gsc), GFP_KERNEL);
	if (!gsc)
		return -ENOMEM;

	/* init gsc structure */
	gsc->tx_buf = (void *)__get_free_pages(GFP_KERNEL,
					       get_order(MAX_DATA_SIZE));
	gsc->rx_buf = (void *)__get_free_pages(GFP_KERNEL,
					       get_order(MAX_DATA_SIZE));
	if (!gsc->tx_buf || !gsc->rx_buf) {
		ret = -ENOMEM;
		goto free_gsc;
	}
	init_waitqueue_head(&gsc->waitq);
	atomic_set(&gsc->users, 0);
	gsc->spi = spi;
	devt = MKDEV(MAJOR(gsc_devt), minor);
	gsc->devt = devt;
	spi_set_drvdata(spi, gsc);

	/* setup ctdl_ap_irq  */
	ret = gsc_request_named_gpio(gsc, "gsc,ctdl_ap_irq",
				     &gsc->ctdl_ap_irq);
	if (ret) {
		dev_err(&spi->dev,
			"gsc_request_named_gpio gsc,ctdl_ap_irq failed.\n");
		goto free_gsc;
	}

	ret = devm_request_irq(&gsc->spi->dev,
			       gpio_to_irq(gsc->ctdl_ap_irq),
			       gsc_irq_handler,
			       IRQF_TRIGGER_RISING | IRQF_ONESHOT,
			       dev_name(&spi->dev),
			       gsc);
	if (ret) {
		dev_err(&spi->dev,
			"devm_request_irq  gsc,ctdl_ap_irq failed.\n");
		goto free_gsc;
	}

	enable_irq_wake(gpio_to_irq(gsc->ctdl_ap_irq));

	/* setup ctdl_rst */
	ret = gsc_request_named_gpio(gsc, "gsc,ctdl_rst",
				     &gsc->ctdl_rst);
	if (ret) {
		dev_err(&spi->dev,
			"gsc_request_named_gpio gsc,ctdl_rst failed.\n");
		goto free_gsc;
	}

	gpio_direction_output(gsc->ctdl_rst, 0);

	/* create the device */
	dev = device_create(gsc_class, &spi->dev, devt, NULL,
			    "gsc%u", minor);
	if (IS_ERR(dev)) {
		ret = PTR_ERR(dev);
		goto free_gsc;
	}

	cdev_init(&gsc->cdev, &gsc_fops);
	gsc->cdev.owner = THIS_MODULE;
	ret = cdev_add(&gsc->cdev, gsc_devt, 1);
	if (ret)
		goto destroy_device;

	return 0;

destroy_device:
	device_destroy(gsc_class, devt);
free_gsc:
	free_pages((unsigned long)gsc->rx_buf, get_order(MAX_DATA_SIZE));
	free_pages((unsigned long)gsc->tx_buf, get_order(MAX_DATA_SIZE));
	kfree(gsc);
	return ret;
}

static void gsc_remove(struct spi_device *spi)
{
	struct gsc_data *gsc = spi_get_drvdata(spi);

	cdev_del(&gsc->cdev);
	device_destroy(gsc_class, gsc->devt);
	free_pages((unsigned long)gsc->rx_buf, get_order(MAX_DATA_SIZE));
	free_pages((unsigned long)gsc->tx_buf, get_order(MAX_DATA_SIZE));
	kfree(gsc);
}

static struct spi_driver gsc_spi_driver = {
	.driver = {
		.name =	"gsc",
		.of_match_table = of_match_ptr(gsc_dt_ids),
	},
	.probe = gsc_probe,
	.remove = gsc_remove,
};

static int __init gsc_init(void)
{
	int ret;

	ret = alloc_chrdev_region(&gsc_devt, 0, GSC_MAX_DEVICES, "gsc");
	if (ret) {
		pr_err("%s: failed to alloc cdev region %d\n", __func__, ret);
		return ret;
	}

	gsc_class = class_create(THIS_MODULE, "gsc");
	if (IS_ERR(gsc_class)) {
		unregister_chrdev_region(gsc_devt, 1);
		return PTR_ERR(gsc_class);
	}

	ret = spi_register_driver(&gsc_spi_driver);
	if (ret < 0) {
		class_destroy(gsc_class);
		unregister_chrdev_region(gsc_devt, 1);
	}

	return ret;
}

static void __exit gsc_exit(void)
{
	spi_unregister_driver(&gsc_spi_driver);
	class_destroy(gsc_class);
	unregister_chrdev_region(gsc_devt, 1);
}

module_init(gsc_init);
module_exit(gsc_exit);

MODULE_AUTHOR("Fernando Lugo, <flugo@google.com>");
MODULE_DESCRIPTION("GSC TPM driver");
MODULE_LICENSE("GPL");
