// SPDX-License-Identifier: <GPL-2.0>
/*
 * Simple synchronous userspace interface to SPI devices
 *
 * Copyright (C) 2006 SWAPP
 *	Andrea Paterniani <a.paterniani@swapp-eng.it>
 * Copyright (C) 2007 David Brownell (simplification, cleanup)
 *
 */
/*
 * Modified by ST Microelectronics.
 * <arach.mohammed.brahim@st.com>
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/acpi.h>

#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>

#include <linux/gpio.h>
#include <linux/of_gpio.h>

#include <linux/uaccess.h>
#include <linux/platform_data/spi-s3c64xx.h>

#undef ST33NFC_QCOM

#ifdef ST33NFC_QCOM
#include <linux/spi/spi-geni-qcom.h>
#endif /* ST33NFC_QCOM */

#ifndef GKI_MODULE
#define GKI_MODULE 1
#endif

#include "../st21nfc.h"

/*
 * This supports access to SPI devices using normal userspace I/O calls.
 * Note that while traditional UNIX/POSIX I/O semantics are half duplex,
 * and often mask message boundaries, full SPI support requires full duplex
 * transfers.  There are several kinds of internal message boundaries to
 * handle chipselect management and other protocol options.
 *
 * SPI has a character major number assigned.  We allocate minor numbers
 * dynamically using a bitmask.  You must use hotplug tools, such as udev
 * (or mdev with busybox) to create and destroy the /dev/st33spi device
 * nodes, since there is no fixed association of minor numbers with any
 * particular SPI bus or device.
 */
static int st33spi_major;
#define N_SPI_MINORS 2 /* ... up to 256 */

static DECLARE_BITMAP(minors, N_SPI_MINORS);

#define ST33SPI_IOC_RD_POWER _IOR(SPI_IOC_MAGIC, 99, __u32)
#define ST33SPI_IOC_WR_POWER _IOW(SPI_IOC_MAGIC, 99, __u32)

/* Bit masks for spi_device.mode management.  Note that incorrect
 * settings for some settings can cause *lots* of trouble for other
 * devices on a shared bus:
 *
 *  - CS_HIGH ... this device will be active when it shouldn't be
 *  - 3WIRE ... when active, it won't behave as it should
 *  - NO_CS ... there will be no explicit message boundaries; this
 *	is completely incompatible with the shared bus model
 *  - READY ... transfers may proceed when they shouldn't.
 *
 * REVISIT should changing those flags be privileged?
 */
#define SPI_MODE_MASK                                                          \
	(SPI_CPHA | SPI_CPOL | SPI_CS_HIGH | SPI_LSB_FIRST | SPI_3WIRE |       \
	 SPI_LOOP | SPI_NO_CS | SPI_READY | SPI_TX_DUAL | SPI_TX_QUAD |        \
	 SPI_RX_DUAL | SPI_RX_QUAD)

struct st33spi_data {
	dev_t devt;
	spinlock_t spi_lock;
	struct spi_device *spi;
	struct spi_device *spi_reset;
	struct list_head device_entry;

	/* TX/RX buffers are NULL unless this device is open (users > 0) */
	struct mutex buf_lock;
	unsigned int users;
	u8 *tx_buffer;
	u8 *rx_buffer;
	u32 speed_hz;

	/* GPIO for SE_POWER_REQ / SE_nRESET */
	struct gpio_desc *gpiod_se_reset;

	int power_gpio_mode;
	int power_gpio;
	int nfcc_needs_poweron;
	int sehal_needs_poweron;
	int se_is_poweron;

	/* GPIO for SPI_CS */
	struct s3c64xx_spi_csinfo *st33spi_csinfo;
	struct pinctrl *pinctrl;
	int spi_state;
};

#define POWER_MODE_NONE -1
#define POWER_MODE_ST33 2

static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

static unsigned int bufsiz = 4096;
module_param(bufsiz, uint, 0444);
MODULE_PARM_DESC(bufsiz, "data bytes in biggest supported SPI message");

#define VERBOSE 0

#define DRIVER_VERSION "2.2.0"

/*-------------------------------------------------------------------------*/
static int st33spi_pinctrl_configure(struct st33spi_data *st33spi, bool enable)
{
	struct pinctrl_state *state;
	int rc;

	dev_info(&st33spi->spi->dev, "st33spi: configure pinctrl: %d\n", enable);

	if (IS_ERR(st33spi->pinctrl)) {
		dev_err(&st33spi->spi->dev, "could not get pinctrl\n");
		return -ENODEV;
	}
	if (enable)
		state = pinctrl_lookup_state(st33spi->pinctrl, "on");
	else
		state = pinctrl_lookup_state(st33spi->pinctrl, "off");

	if (!IS_ERR_OR_NULL(state)) {
		rc = pinctrl_select_state(st33spi->pinctrl, state);
		if (unlikely(rc))
			dev_err(&st33spi->spi->dev, "st33spi: failed to set pinctrl state\n");
		return rc;
	}
	dev_err(&st33spi->spi->dev, "st33spi: failed to get pinctrl state\n");

	return -EIO;
}

static ssize_t st33spi_state_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct spi_device *spi;
	struct st33spi_data *st33spi;
	spi = to_spi_device(dev);
	if (spi == NULL)
		return -ENODEV;

	st33spi = spi_get_drvdata(spi);
	if (st33spi == NULL)
		return -ENODEV;

	return scnprintf(buf, PAGE_SIZE, "state:%d, st33spi_cs:%d\n",
			st33spi->spi_state, gpio_get_value(st33spi->st33spi_csinfo->line));
}

static ssize_t st33spi_state_store(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct spi_device *spi;
	struct st33spi_data *st33spi;
	struct s3c64xx_spi_csinfo *cs;
	int new_spi_state;

	spi = to_spi_device(dev);
	if (spi == NULL)
		return -ENODEV;

	st33spi = spi_get_drvdata(spi);
	if (st33spi == NULL)
		return -ENODEV;

	cs = st33spi->st33spi_csinfo;
	if (cs == NULL)
		return -ENODEV;

	if (!kstrtoint(buf, 10, &new_spi_state)) {
		if (new_spi_state == 0) {
                        st33spi->spi_state = 0;
			st33spi_pinctrl_configure(st33spi, false);
		} else if (new_spi_state == 33) {
                        st33spi->spi_state = 1;
			st33spi_pinctrl_configure(st33spi, true);
		} else {
			dev_err(dev, "%s: incorrect parameter\n", __func__);
			return -EINVAL;
		}
	}
	return count;
}

static DEVICE_ATTR_RW(st33spi_state);

static struct attribute *st33spi_attrs[] = {
	&dev_attr_st33spi_state.attr,
	NULL,
};

static struct attribute_group st33spi_attr_grp = {
	.attrs = st33spi_attrs,
};

static ssize_t st33spi_sync(struct st33spi_data *st33spi,
			    struct spi_message *message)
{
	int status;
	struct spi_device *spi;

	spin_lock_irq(&st33spi->spi_lock);
	spi = st33spi->spi;
	spin_unlock_irq(&st33spi->spi_lock);

	if (spi == NULL)
		status = -ESHUTDOWN;
	else
		status = spi_sync(spi, message);

	if (status == 0)
		status = message->actual_length;

	return status;
}

static inline ssize_t st33spi_sync_write(struct st33spi_data *st33spi,
					 size_t len)
{
	struct spi_transfer t = {
		.tx_buf = st33spi->tx_buffer,
		.len = len,
		.speed_hz = st33spi->speed_hz,
	};
	struct spi_message m;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	return st33spi_sync(st33spi, &m);
}

static inline ssize_t st33spi_sync_read(struct st33spi_data *st33spi,
					size_t len)
{
	struct spi_transfer t = {
		.rx_buf = st33spi->rx_buffer,
		.len = len,
		.speed_hz = st33spi->speed_hz,
	};
	struct spi_message m;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	return st33spi_sync(st33spi, &m);
}

/*-------------------------------------------------------------------------*/

/* Read-only message with current device setup */
static ssize_t st33spi_read(struct file *filp, char __user *buf, size_t count,
			    loff_t *f_pos)
{
	struct st33spi_data *st33spi;
	ssize_t status = 0;

	/* chipselect only toggles at start or end of operation */
	if (count > bufsiz)
		return -EMSGSIZE;

	st33spi = filp->private_data;

	if (st33spi == NULL)
		return -ENODEV;

	if (!st33spi->spi_state) {
		dev_warn(&st33spi->spi->dev, "st33spi: spi is not enabled, abort read process\n");
		return -EFAULT;
	}

	dev_dbg(&st33spi->spi->dev, "st33spi Read: %zu bytes\n", count);

	mutex_lock(&st33spi->buf_lock);
	status = st33spi_sync_read(st33spi, count);
	if (status > 0) {
		unsigned long missing;

		missing = copy_to_user(buf, st33spi->rx_buffer, status);
		if (missing == status)
			status = -EFAULT;
		else
			status = status - missing;
	}
	mutex_unlock(&st33spi->buf_lock);

	dev_dbg(&st33spi->spi->dev, "st33spi Read: status: %zd\n", status);

	return status;
}

/* Write-only message with current device setup */
static ssize_t st33spi_write(struct file *filp, const char __user *buf,
			     size_t count, loff_t *f_pos)
{
	struct st33spi_data *st33spi;
	ssize_t status = 0;
	unsigned long missing;

	/* chipselect only toggles at start or end of operation */
	if (count > bufsiz)
		return -EMSGSIZE;

	st33spi = filp->private_data;

	if (st33spi == NULL)
		return -ENODEV;

	if (!st33spi->spi_state) {
		dev_warn(&st33spi->spi->dev, "st33spi: spi is not enabled, abort write process\n");
		return -EFAULT;
	}

	dev_dbg(&st33spi->spi->dev, "st33spi Write: %zu bytes\n", count);

	mutex_lock(&st33spi->buf_lock);
	missing = copy_from_user(st33spi->tx_buffer, buf, count);
	if (missing == 0)
		status = st33spi_sync_write(st33spi, count);
	else
		status = -EFAULT;
	mutex_unlock(&st33spi->buf_lock);

	dev_dbg(&st33spi->spi->dev, "st33spi Write: status: %zd\n", status);

	return status;
}

static int st33spi_message(struct st33spi_data *st33spi,
			   struct spi_ioc_transfer *u_xfers,
			   unsigned int n_xfers)
{
	struct spi_message msg;
	struct spi_transfer *k_xfers;
	struct spi_transfer *k_tmp;
	struct spi_ioc_transfer *u_tmp;
	unsigned int n, total, tx_total, rx_total;
	u8 *tx_buf, *rx_buf;
	int status = -EFAULT;

	spi_message_init(&msg);
	k_xfers = kcalloc(n_xfers, sizeof(*k_tmp), GFP_KERNEL);
	if (k_xfers == NULL)
		return -ENOMEM;

	/* Construct spi_message, copying any tx data to bounce buffer.
	 * We walk the array of user-provided transfers, using each one
	 * to initialize a kernel version of the same transfer.
	 */
	tx_buf = st33spi->tx_buffer;
	rx_buf = st33spi->rx_buffer;
	total = 0;
	tx_total = 0;
	rx_total = 0;
	for (n = n_xfers, k_tmp = k_xfers, u_tmp = u_xfers; n;
	     n--, k_tmp++, u_tmp++) {
		k_tmp->len = u_tmp->len;

		total += k_tmp->len;
		/* Since the function returns the total length of transfers
		 * on success, restrict the total to positive int values to
		 * avoid the return value looking like an error.  Also check
		 * each transfer length to avoid arithmetic overflow.
		 */
		if (total > INT_MAX || k_tmp->len > INT_MAX) {
			status = -EMSGSIZE;
			goto done;
		}

		if (u_tmp->rx_buf) {
			/* this transfer needs space in RX bounce buffer */
			rx_total += k_tmp->len;
			if (rx_total > bufsiz) {
				status = -EMSGSIZE;
				goto done;
			}
			k_tmp->rx_buf = rx_buf;
			if (!access_ok((u8 __user *)(uintptr_t)u_tmp->rx_buf,
				       u_tmp->len))
				goto done;
			rx_buf += k_tmp->len;
		}
		if (u_tmp->tx_buf) {
			/* this transfer needs space in TX bounce buffer */
			tx_total += k_tmp->len;
			if (tx_total > bufsiz) {
				status = -EMSGSIZE;
				goto done;
			}
			k_tmp->tx_buf = tx_buf;
			if (copy_from_user(
				    tx_buf,
				    (const u8 __user *)(uintptr_t)u_tmp->tx_buf,
				    u_tmp->len))
				goto done;
			tx_buf += k_tmp->len;
		}

		k_tmp->cs_change = !!u_tmp->cs_change;
		k_tmp->tx_nbits = u_tmp->tx_nbits;
		k_tmp->rx_nbits = u_tmp->rx_nbits;
		k_tmp->bits_per_word = u_tmp->bits_per_word;
		k_tmp->delay_usecs = u_tmp->delay_usecs;
		k_tmp->speed_hz = u_tmp->speed_hz;
		if (!k_tmp->speed_hz)
			k_tmp->speed_hz = st33spi->speed_hz;
#if VERBOSE
		dev_dbg(&st33spi->spi->dev,
			"  xfer len %u %s%s%s%dbits %u usec %uHz\n", u_tmp->len,
			u_tmp->rx_buf ? "rx " : "", u_tmp->tx_buf ? "tx " : "",
			u_tmp->cs_change ? "cs " : "",
			u_tmp->bits_per_word ?: st33spi->spi->bits_per_word,
			u_tmp->delay_usecs,
			u_tmp->speed_hz ?: st33spi->spi->max_speed_hz);
#endif
		spi_message_add_tail(k_tmp, &msg);
	}

	status = st33spi_sync(st33spi, &msg);
	if (status < 0)
		goto done;

	/* copy any rx data out of bounce buffer */
	rx_buf = st33spi->rx_buffer;
	for (n = n_xfers, u_tmp = u_xfers; n; n--, u_tmp++) {
		if (u_tmp->rx_buf) {
			if (__copy_to_user((u8 __user *)(uintptr_t)u_tmp->rx_buf,
					   rx_buf, u_tmp->len)) {
				status = -EFAULT;
				goto done;
			}
			rx_buf += u_tmp->len;
		}
	}
	status = total;

done:
	kfree(k_xfers);
	return status;
}

static struct spi_ioc_transfer *
st33spi_get_ioc_message(unsigned int cmd, struct spi_ioc_transfer __user *u_ioc,
			unsigned int *n_ioc)
{
	struct spi_ioc_transfer *ioc;
	u32 tmp;

	/* Check type, command number and direction */
	if (_IOC_TYPE(cmd) != SPI_IOC_MAGIC ||
	    _IOC_NR(cmd) != _IOC_NR(SPI_IOC_MESSAGE(0)) ||
	    _IOC_DIR(cmd) != _IOC_WRITE)
		return ERR_PTR(-ENOTTY);

	tmp = _IOC_SIZE(cmd);
	if ((tmp % sizeof(struct spi_ioc_transfer)) != 0)
		return ERR_PTR(-EINVAL);
	*n_ioc = tmp / sizeof(struct spi_ioc_transfer);
	if (*n_ioc == 0)
		return NULL;

	/* copy into scratch area */
	ioc = kmalloc(tmp, GFP_KERNEL);
	if (!ioc)
		return ERR_PTR(-ENOMEM);
	if (__copy_from_user(ioc, u_ioc, tmp)) {
		kfree(ioc);
		return ERR_PTR(-EFAULT);
	}
	return ioc;
}

static void st33spi_power_off(struct st33spi_data *st33spi)
{
#ifdef WITH_SPI_CLK_MNGT
	/* no need for the SPI clock to be enabled. */
	dev_dbg(&st33spi->spi->dev,
		 "%s : disabling PMU clock of SPI subsystem\n", __func__);
	mt_spi_disable_master_clk(st33spi->spi);
#endif /* WITH_SPI_CLK_MNGT */

	st33spi->se_is_poweron = 0;
}

static void st33spi_power_on(struct st33spi_data *st33spi)
{
#ifdef WITH_SPI_CLK_MNGT
	/* the SPI clock needs to be enabled. */
	dev_dbg(&st33spi->spi->dev,
		 "%s : enabling PMU clock of SPI subsystem\n", __func__);
	mt_spi_enable_master_clk(st33spi->spi);
#endif /* WITH_SPI_CLK_MNGT */

	if (st33spi->power_gpio_mode == POWER_MODE_ST33) {
		/* Just a pulse on SPI_nRESET */
		gpiod_set_value_cansleep(st33spi->gpiod_se_reset, 1);
		usleep_range(5000, 5500);
		gpiod_set_value_cansleep(st33spi->gpiod_se_reset, 0);
		dev_info(&st33spi->spi->dev, "%s : st33 set nReset to Low\n",
			 __func__);
		usleep_range(3000, 4000);
	}
	st33spi->se_is_poweron = 1;
}

static void st33spi_power_set(struct st33spi_data *st33spi, int val)
{
	if (!st33spi)
		return;

	dev_dbg(&st33spi->spi->dev, "st33spi sehal pwr_req: %d\n", val);

	if (val) {
		st33spi->sehal_needs_poweron = 1;
		st33spi_power_on(st33spi);
	} else {
		st33spi->sehal_needs_poweron = 0;
		if ((st33spi->se_is_poweron == 1) &&
		    (st33spi->nfcc_needs_poweron == 0))
			/* we don t need power anymore */
			st33spi_power_off(st33spi);
	}
}

static int st33spi_power_get(struct st33spi_data *st33spi)
{
	return gpiod_get_value_cansleep(st33spi->gpiod_se_reset);
}

static long st33spi_ioctl(struct file *filp, unsigned int cmd,
			  unsigned long arg)
{
	int err = 0;
	int retval = 0;
	struct st33spi_data *st33spi;
	struct spi_device *spi;
	u32 tmp;
	unsigned int n_ioc;
	struct spi_ioc_transfer *ioc;

	/* Check type and command number */
	if (_IOC_TYPE(cmd) != SPI_IOC_MAGIC)
		return -ENOTTY;

	/* Check access direction once here; don't repeat below.
	 * IOC_DIR is from the user perspective, while access_ok is
	 * from the kernel perspective; so they look reversed.
	 */
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok((void __user *)arg, _IOC_SIZE(cmd));
	if (err == 0 && _IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok((void __user *)arg, _IOC_SIZE(cmd));
	if (err)
		return -EFAULT;

	/* guard against device removal before, or while,
	 * we issue this ioctl.
	 */
	st33spi = filp->private_data;
	spin_lock_irq(&st33spi->spi_lock);
	spi = spi_dev_get(st33spi->spi);
	spin_unlock_irq(&st33spi->spi_lock);

	dev_dbg(&st33spi->spi->dev, "st33spi ioctl cmd %d\n", cmd);

	if (spi == NULL)
		return -ESHUTDOWN;

	/* use the buffer lock here for triple duty:
	 *  - prevent I/O (from us) so calling spi_setup() is safe;
	 *  - prevent concurrent SPI_IOC_WR_* from morphing
	 *    data fields while SPI_IOC_RD_* reads them;
	 *  - SPI_IOC_MESSAGE needs the buffer locked "normally".
	 */
	mutex_lock(&st33spi->buf_lock);

	switch (cmd) {
		/* read requests */
	case SPI_IOC_RD_MODE:
		retval = __put_user(spi->mode & SPI_MODE_MASK,
				    (__u8 __user *)arg);
		break;
	case SPI_IOC_RD_MODE32:
		retval = __put_user(spi->mode & SPI_MODE_MASK,
				    (__u32 __user *)arg);
		break;
	case SPI_IOC_RD_LSB_FIRST:
		retval = __put_user((spi->mode & SPI_LSB_FIRST) ? 1 : 0,
				    (__u8 __user *)arg);
		break;
	case SPI_IOC_RD_BITS_PER_WORD:
		retval = __put_user(spi->bits_per_word, (__u8 __user *)arg);
		break;
	case SPI_IOC_RD_MAX_SPEED_HZ:
		retval = __put_user(st33spi->speed_hz, (__u32 __user *)arg);
		break;
	case ST33SPI_IOC_RD_POWER:
		dev_dbg(&st33spi->spi->dev, "st33spi ST33SPI_IOC_RD_POWER\n");
		retval = __put_user(st33spi_power_get(st33spi),
				    (__u32 __user *)arg);
		break;

		/* write requests */
	case SPI_IOC_WR_MODE:
	case SPI_IOC_WR_MODE32:
		if (cmd == SPI_IOC_WR_MODE)
			retval = __get_user(tmp, (u8 __user *)arg);
		else
			retval = __get_user(tmp, (u32 __user *)arg);
		if (retval == 0) {
			u32 save = spi->mode;

			if (tmp & ~SPI_MODE_MASK) {
				retval = -EINVAL;
				break;
			}

			tmp |= spi->mode & ~SPI_MODE_MASK;
			spi->mode = (u16)tmp;
			retval = spi_setup(spi);
			if (retval < 0)
				spi->mode = save;
			else
				dev_dbg(&spi->dev, "spi mode %x\n", tmp);
		}
		break;
	case SPI_IOC_WR_LSB_FIRST:
		retval = __get_user(tmp, (__u8 __user *)arg);
		if (retval == 0) {
			u32 save = spi->mode;

			if (tmp)
				spi->mode |= SPI_LSB_FIRST;
			else
				spi->mode &= ~SPI_LSB_FIRST;
			retval = spi_setup(spi);
			if (retval < 0)
				spi->mode = save;
			else
				dev_dbg(&spi->dev, "%csb first\n",
					tmp ? 'l' : 'm');
		}
		break;
	case SPI_IOC_WR_BITS_PER_WORD:
		retval = __get_user(tmp, (__u8 __user *)arg);
		if (retval == 0) {
			u8 save = spi->bits_per_word;

			spi->bits_per_word = tmp;
			retval = spi_setup(spi);
			if (retval < 0)
				spi->bits_per_word = save;
			else
				dev_dbg(&spi->dev, "%d bits per word\n", tmp);
		}
		break;
	case SPI_IOC_WR_MAX_SPEED_HZ:
		retval = __get_user(tmp, (__u32 __user *)arg);
		if (retval == 0) {
			u32 save = spi->max_speed_hz;

			spi->max_speed_hz = tmp;
			retval = spi_setup(spi);
			if (retval >= 0)
				st33spi->speed_hz = tmp;
			else
				dev_dbg(&spi->dev, "%d Hz (max)\n", tmp);
			spi->max_speed_hz = save;
		}
		break;
	case ST33SPI_IOC_WR_POWER:
		retval = __get_user(tmp, (__u32 __user *)arg);
		dev_dbg(&st33spi->spi->dev,
			 "st33spi ST33SPI_IOC_WR_POWER %d\n", retval);
		if (retval == 0 && st33spi->spi_state) {
			st33spi_power_set(st33spi, tmp ? 1 : 0);
			dev_dbg(&st33spi->spi->dev, "SE_POWER_REQ set: %d\n", tmp);
		}
		break;
	default:
		/* segmented and/or full-duplex I/O request */
		/* Check message and copy into scratch area */
		ioc = st33spi_get_ioc_message(
			cmd, (struct spi_ioc_transfer __user *)arg, &n_ioc);
		if (IS_ERR(ioc)) {
			retval = PTR_ERR(ioc);
			break;
		}
		if (!ioc)
			break; /* n_ioc is also 0 */

		/* translate to spi_message, execute */
		retval = st33spi_message(st33spi, ioc, n_ioc);
		kfree(ioc);
		break;
	}

	mutex_unlock(&st33spi->buf_lock);
	spi_dev_put(spi);

	dev_dbg(&st33spi->spi->dev, "st33spi ioctl retval %d\n", retval);

	return retval;
}

#ifdef CONFIG_COMPAT
static long st33spi_compat_ioc_message(struct file *filp, unsigned int cmd,
				       unsigned long arg)
{
	struct spi_ioc_transfer __user *u_ioc;
	int retval = 0;
	struct st33spi_data *st33spi;
	struct spi_device *spi;
	unsigned int n_ioc, n;
	struct spi_ioc_transfer *ioc;

	u_ioc = (struct spi_ioc_transfer __user *)compat_ptr(arg);
	if (!access_ok(u_ioc, _IOC_SIZE(cmd)))
		return -EFAULT;

	/* guard against device removal before, or while,
	 * we issue this ioctl.
	 */
	st33spi = filp->private_data;
	spin_lock_irq(&st33spi->spi_lock);
	spi = spi_dev_get(st33spi->spi);
	spin_unlock_irq(&st33spi->spi_lock);

	dev_dbg(&st33spi->spi->dev, "st33spi compat_ioctl cmd %d\n", cmd);

	if (spi == NULL)
		return -ESHUTDOWN;

	/* SPI_IOC_MESSAGE needs the buffer locked "normally" */
	mutex_lock(&st33spi->buf_lock);

	/* Check message and copy into scratch area */
	ioc = st33spi_get_ioc_message(cmd, u_ioc, &n_ioc);
	if (IS_ERR(ioc)) {
		retval = PTR_ERR(ioc);
		goto done;
	}
	if (!ioc)
		goto done; /* n_ioc is also 0 */

	/* Convert buffer pointers */
	for (n = 0; n < n_ioc; n++) {
		ioc[n].rx_buf = (uintptr_t)compat_ptr(ioc[n].rx_buf);
		ioc[n].tx_buf = (uintptr_t)compat_ptr(ioc[n].tx_buf);
	}

	/* translate to spi_message, execute */
	retval = st33spi_message(st33spi, ioc, n_ioc);
	kfree(ioc);

done:
	mutex_unlock(&st33spi->buf_lock);
	spi_dev_put(spi);
	dev_dbg(&st33spi->spi->dev, "st33spi compat_ioctl retval %d\n", retval);
	return retval;
}

static long st33spi_compat_ioctl(struct file *filp, unsigned int cmd,
				 unsigned long arg)
{
	if (_IOC_TYPE(cmd) == SPI_IOC_MAGIC &&
	    _IOC_NR(cmd) == _IOC_NR(SPI_IOC_MESSAGE(0)) &&
	    _IOC_DIR(cmd) == _IOC_WRITE)
		return st33spi_compat_ioc_message(filp, cmd, arg);

	return st33spi_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#else
#define st33spi_compat_ioctl NULL
#endif /* CONFIG_COMPAT */

static int st33spi_open(struct inode *inode, struct file *filp)
{
	struct st33spi_data *st33spi = NULL;
	int status = -ENXIO;

	mutex_lock(&device_list_lock);

	list_for_each_entry (st33spi, &device_list, device_entry) {
		if (st33spi->devt == inode->i_rdev) {
			status = 0;
			break;
		}
	}

	if (st33spi == NULL)
		return -ENODEV;

	if (status) {
		dev_dbg(&st33spi->spi->dev, "st33spi: nothing for minor %d\n",
			iminor(inode));
		goto err_find_dev;
	}

	if (!st33spi->spi_state) {
		dev_warn(&st33spi->spi->dev,
				"st33spi: spi is not enabled, abort open process\n");
		mutex_unlock(&device_list_lock);
		return -EFAULT;
	}

	/* Authorize only 1 process to open the device. */
	if (st33spi->users > 0) {
		dev_err(&st33spi->spi->dev, "already open\n");
		mutex_unlock(&device_list_lock);
		return -EBUSY;
	}

	dev_dbg(&st33spi->spi->dev, "st33spi: open\n");

	if (!st33spi->tx_buffer) {
		st33spi->tx_buffer = kmalloc(bufsiz, GFP_KERNEL);
		if (!st33spi->tx_buffer) {
			status = -ENOMEM;
			goto err_find_dev;
		}
	}

	if (!st33spi->rx_buffer) {
		st33spi->rx_buffer = kmalloc(bufsiz, GFP_KERNEL);
		if (!st33spi->rx_buffer) {
			status = -ENOMEM;
			goto err_alloc_rx_buf;
		}
	}

	st33spi->users++;
	filp->private_data = st33spi;
	nonseekable_open(inode, filp);

	mutex_unlock(&device_list_lock);

	dev_dbg(&st33spi->spi->dev, "st33spi: open - force power on\n");
	st33spi_power_set(st33spi, 1);
	return 0;

err_alloc_rx_buf:
	kfree(st33spi->tx_buffer);
	st33spi->tx_buffer = NULL;
err_find_dev:
	mutex_unlock(&device_list_lock);
	return status;
}

static int st33spi_release(struct inode *inode, struct file *filp)
{
	struct st33spi_data *st33spi;

	mutex_lock(&device_list_lock);
	st33spi = filp->private_data;
	filp->private_data = NULL;

	dev_dbg(&st33spi->spi->dev, "st33spi: release\n");

	/* last close? */
	st33spi->users--;
	if (!st33spi->users) {
		int dofree;

		dev_dbg(&st33spi->spi->dev,
			 "st33spi: release - may allow power off\n");
		st33spi_power_set(st33spi, 0);

		kfree(st33spi->tx_buffer);
		st33spi->tx_buffer = NULL;

		kfree(st33spi->rx_buffer);
		st33spi->rx_buffer = NULL;

		spin_lock_irq(&st33spi->spi_lock);
		if (st33spi->spi)
			st33spi->speed_hz = st33spi->spi->max_speed_hz;

		/* ... after we unbound from the underlying device? */
		dofree = ((st33spi->spi == NULL) &&
			  (st33spi->spi_reset == NULL));
		spin_unlock_irq(&st33spi->spi_lock);

		if (dofree)
			kfree(st33spi);
	}
	mutex_unlock(&device_list_lock);

	return 0;
}

static const struct file_operations st33spi_fops = {
	.owner = THIS_MODULE,
	/*
	 * REVISIT switch to aio primitives, so that userspace
	 * gets more complete API coverage.  It'll simplify things
	 * too, except for the locking.
	 */
	.write = st33spi_write,
	.read = st33spi_read,
	.unlocked_ioctl = st33spi_ioctl,
	.compat_ioctl = st33spi_compat_ioctl,
	.open = st33spi_open,
	.release = st33spi_release,
	.llseek = no_llseek,
};

/*-------------------------------------------------------------------------*/

/*
 * The main reason to have this class is to make mdev/udev create the
 * /dev/st33spi character device nodes exposing our userspace API.
 * It also simplifies memory management.
 */

static struct class *st33spi_class;

static const struct of_device_id st33spi_dt_ids[] = {
	{ .compatible = "st,st33spi" },
	{},
};
MODULE_DEVICE_TABLE(of, st33spi_dt_ids);

#ifdef CONFIG_ACPI

/* Placeholder SPI devices not to be used in production systems */
#define ST33SPI_ACPI_PLACEHOLDER 1

static const struct acpi_device_id st33spi_acpi_ids[] = {
	/*
	 * The ACPI SPT000* devices are only meant for development and
	 * testing. Systems used in production should have a proper ACPI
	 * description of the connected peripheral and they should also
	 * use a proper driver instead of poking directly to the SPI bus
	 */
	{ "SPT0001", ST33SPI_ACPI_PLACEHOLDER },
	{ "SPT0002", ST33SPI_ACPI_PLACEHOLDER },
	{ "SPT0003", ST33SPI_ACPI_PLACEHOLDER },
	{},
};
MODULE_DEVICE_TABLE(acpi, st33spi_acpi_ids);

static void st33spi_probe_acpi(struct spi_device *spi)
{
	const struct acpi_device_id *id;

	if (!has_acpi_companion(&spi->dev))
		return;

	id = acpi_match_device(st33spi_acpi_ids, &spi->dev);
	if (WARN_ON(!id))
		return;
}
#else
static inline void st33spi_probe_acpi(struct spi_device *spi)
{
}
#endif

/*-------------------------------------------------------------------------*/

static int st33spi_parse_dt(struct device *dev, struct st33spi_data *pdata)
{
	struct device_node *np = dev->of_node;
	struct device_node *data_np;
	const char *power_mode;
	int st33spi_state;

#ifndef GKI_MODULE
	np = of_find_compatible_node(NULL, NULL, "st,st33spi");
#endif

	if (!np) {
		return -ENODEV;
	}

	/* Read power mode. */
	power_mode = of_get_property(np, "power_mode", NULL);
	if (!power_mode) {
		dev_info(dev, "Default power mode: ST33\n");
		pdata->power_gpio_mode = POWER_MODE_ST33;
	} else if (!strcmp(power_mode, "ST33")) {
		dev_info(dev, "Power mode: ST33\n");
		pdata->power_gpio_mode = POWER_MODE_ST33;
	} else if (!strcmp(power_mode, "none")) {
		dev_info(dev, "Power mode: none\n");
		pdata->power_gpio_mode = POWER_MODE_NONE;
	} else {
		dev_err(dev, "Power mode unknown: %s\n", power_mode);
		return -EFAULT;
	}

	/* Get the Gpio */
	if (pdata->power_gpio_mode == POWER_MODE_ST33) {
		pdata->gpiod_se_reset =
			devm_gpiod_get(dev, "esereset", GPIOD_OUT_LOW);
		if (IS_ERR(pdata->gpiod_se_reset)) {
			dev_err(dev, "Unable to request esereset %d\n",
					IS_ERR(pdata->gpiod_se_reset));
			return -ENODEV;
		}
	} else {
		dev_err(dev, "ST54H mode not supported");
	}

	/* Read default st33spi state. */
	data_np = of_get_child_by_name(np, "controller-data");
	if (!data_np) {
		dev_err(dev, "child node 'controller-data' not found\n");
		return -ENODEV;
	}

	if (!of_property_read_u32(data_np, "cs-init-state", &st33spi_state)) {
		pdata->spi_state = st33spi_state;
	} else {
		pdata->spi_state = 0;
	}
	dev_info(dev, "Default st33spi state: %d\n", pdata->spi_state);

	pdata->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(pdata->pinctrl)) {
		dev_err(dev, "could not get pinctrl\n");
		return -ENODEV;
	}

	return 0;
}

static int st33spi_probe(struct spi_device *spi)
{
	struct st33spi_data *st33spi;
	int status;
	unsigned long minor;
#ifdef ST33NFC_QCOM
	struct device *dev = &spi->dev;
	struct spi_geni_qcom_ctrl_data *spi_param;
#endif /* ST33NFC_QCOM */

#ifdef GKI_MODULE
	/* Claim our 256 reserved device numbers.  Then register a class
	 * that will key udev/mdev to add/remove /dev nodes.  Last, register
	 * the driver which manages those device numbers.
	 */
	BUILD_BUG_ON(N_SPI_MINORS > 256);
	st33spi_major =
		__register_chrdev(0, 0, N_SPI_MINORS, "spi", &st33spi_fops);
	dev_info(&spi->dev, "Loading st33spi driver, major: %d\n", st33spi_major);

	st33spi_class = class_create(THIS_MODULE, "st33spi");
	if (IS_ERR(st33spi_class)) {
		unregister_chrdev(st33spi_major, "st33spi");
		return PTR_ERR(st33spi_class);
	}
#endif

	/*
	 * st33spi should never be referenced in DT without a specific
	 * compatible string, it is a Linux implementation thing
	 * rather than a description of the hardware.
	 */

	st33spi_probe_acpi(spi);

	/* Allocate driver data */
	st33spi = kzalloc(sizeof(*st33spi), GFP_KERNEL);
	if (!st33spi)
		return -ENOMEM;

	/* Initialize the driver data */
	st33spi->spi = spi;
	spin_lock_init(&st33spi->spi_lock);
	mutex_init(&st33spi->buf_lock);

	INIT_LIST_HEAD(&st33spi->device_entry);

	/* If we can allocate a minor number, hook up this device.
	 * Reusing minors is fine so long as udev or mdev is working.
	 */
	mutex_lock(&device_list_lock);
	minor = find_first_zero_bit(minors, N_SPI_MINORS);
	if (minor < N_SPI_MINORS) {
		struct device *dev;

		st33spi->devt = MKDEV(st33spi_major, minor);
		dev = device_create(st33spi_class, &spi->dev, st33spi->devt,
				    st33spi, "st33spi");
		status = PTR_ERR_OR_ZERO(dev);

		status = sysfs_create_group(&dev->kobj, &st33spi_attr_grp);
		if (status) {
			dev_err(&spi->dev, "sysfs_create_group failed\n");
		}
		st33spi->st33spi_csinfo = spi_get_ctldata(spi);
	} else {
		dev_dbg(&spi->dev, "%s : no minor number available!\n",
			__FILE__);
		status = -ENODEV;
	}
	if (status == 0) {
		set_bit(minor, minors);
		list_add(&st33spi->device_entry, &device_list);
	}
	mutex_unlock(&device_list_lock);

	st33spi->speed_hz = spi->max_speed_hz;
	dev_dbg(&spi->dev, "%s : st33spi->speed_hz=%d\n", __FILE__,
		st33spi->speed_hz);

	/* set timings for ST33 */
#ifdef ST33NFC_QCOM
	spi_param = devm_kzalloc(dev, sizeof(spi_param), GFP_KERNEL);
	if (spi_param == NULL)
		return -ENOMEM;

	/* Initialize the driver data */
	spi_param->spi_cs_clk_delay = 90;
	spi->controller_data = spi_param;

#else
	dev_err(&spi->dev, "%s : TSU_NSS configuration be implemented!\n",
		__func__);
	/*
	 * platform-specific method to configure the delay between NSS
	 * selection and the start of data transfer (clk).
	 * If no specific method required, you can comment above line.
	 */
#endif
	spi->bits_per_word = 8;

	if (status == 0) {
		spi_set_drvdata(spi, st33spi);
		(void)st33spi_parse_dt(&spi->dev, st33spi);
		if (!st33spi->spi_state) {
			dev_dbg(&spi->dev, "st33spi: probe - put spi pins to low \n");
			st33spi_pinctrl_configure(st33spi, false);
		} else {
			st33spi_pinctrl_configure(st33spi, true);
		}
	} else {
		kfree(st33spi);
	}

	return status;
}

static int st33spi_remove(struct spi_device *spi)
{
	struct st33spi_data *st33spi = spi_get_drvdata(spi);

	/* make sure ops on existing fds can abort cleanly */
	spin_lock_irq(&st33spi->spi_lock);
	st33spi->spi = NULL;
	st33spi->spi_reset = NULL;
	spin_unlock_irq(&st33spi->spi_lock);

	/* prevent new opens */
	mutex_lock(&device_list_lock);
	list_del(&st33spi->device_entry);
	device_destroy(st33spi_class, st33spi->devt);
	clear_bit(MINOR(st33spi->devt), minors);
	if (st33spi->users == 0) {
		kfree(st33spi);
#ifdef GKI_MODULE
		class_destroy(st33spi_class);
		unregister_chrdev(st33spi_major, "st33spi");
#endif
	}
	mutex_unlock(&device_list_lock);

	return 0;
}

static struct spi_driver st33spi_spi_driver = {
		.driver = {
			.name = "st33spi",
			.of_match_table = of_match_ptr(st33spi_dt_ids),
			.acpi_match_table = ACPI_PTR(st33spi_acpi_ids),
		},
		.probe = st33spi_probe,
		.remove = st33spi_remove,

		/* NOTE:  suspend/resume methods are not necessary here.
		 * We don't do anything except pass the requests to/from
		 * the underlying controller.  The refrigerator handles
		 * most issues; the controller driver handles the rest.
		 */
};

/*-------------------------------------------------------------------------*/

#ifdef GKI_MODULE
module_spi_driver(st33spi_spi_driver);
#else
static int __init st33spi_init(void)
{
	int status;

	pr_info("Loading st33spi driver\n");

	/* Claim our 256 reserved device numbers.  Then register a class
	 * that will key udev/mdev to add/remove /dev nodes.  Last, register
	 * the driver which manages those device numbers.
	 */
	BUILD_BUG_ON(N_SPI_MINORS > 256);
	st33spi_major =
		__register_chrdev(0, 0, N_SPI_MINORS, "spi", &st33spi_fops);
	pr_info("Loading st33spi driver, major: %d\n", st33spi_major);

	st33spi_class = class_create(THIS_MODULE, "st33spi");
	if (IS_ERR(st33spi_class)) {
		unregister_chrdev(st33spi_major,
				  st33spi_spi_driver.driver.name);
		return PTR_ERR(st33spi_class);
	}

	status = spi_register_driver(&st33spi_spi_driver);
	if (status < 0) {
		class_destroy(st33spi_class);
		unregister_chrdev(st33spi_major,
				  st33spi_spi_driver.driver.name);
	}
	pr_info("Loading st33spi driver: %d\n", status);
	return status;
}
module_init(st33spi_init);

static void __exit st33spi_exit(void)
{
	spi_unregister_driver(&st33spi_spi_driver);
	class_destroy(st33spi_class);
	unregister_chrdev(st33spi_major, st33spi_spi_driver.driver.name);
}
module_exit(st33spi_exit);
#endif

MODULE_AUTHOR("Andrea Paterniani, <a.paterniani@swapp-eng.it>");
MODULE_DESCRIPTION("User mode SPI device interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:st33spi");
