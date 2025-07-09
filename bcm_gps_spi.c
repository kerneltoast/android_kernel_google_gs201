// SPDX-License-Identifier: GPL-2.0
/* Copyright 2015 Broadcom Corporation
 *
 * The Broadcom GPS SPI driver
 *
 */

/* TODO: Use dev_*() calls instead */
#define pr_fmt(fmt) "GPSREGS: " fmt

#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/sysrq.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <linux/kthread.h>
#include <linux/circ_buf.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/of_gpio.h>
#include <linux/poll.h>
#include <linux/uaccess.h>
#include <linux/suspend.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/miscdevice.h>
#include <linux/timekeeping.h>
#include <linux/io.h>
#include <asm/irq.h>
#include <linux/kernel_stat.h>
#include <linux/pm_runtime.h>
#include <linux/platform_device.h>

#include "bbd.h"
#include "bcm_gps_spi.h"

/* 0 - Half Duplex, 1 - Full Duplex */
#define SSI_MODE 1

/*  1 = 1B, 2 = 2B */
#define SSI_LEN 2

/*
 * TODO: Need to read bitrate from bus driver spi.c.
 * Just for startup info notification.
 */
#define BCM_BITRATE 12000
#define DEVICE_NAME "gnss"

static void bcm_on_packet_received(
		void *_priv, unsigned char *data, unsigned int size);

static void sscd_release(struct device *dev) {
	(void)dev;
}

static ssize_t coredump_store(struct device *dev, struct device_attribute *attr, const char *buf,
				size_t count)
{
	char *next;
	int length;
	char *reason;
	static time64_t last_trigger_time;
	struct sscd_segment seg;
	time64_t trigger_time = ktime_get_seconds();
	struct spi_device *spi = to_spi_device(dev);
	struct bcm_spi_priv *priv = spi_get_drvdata(spi);
	struct sscd_platform_data *pdata = dev_get_platdata(&priv->sscd_dev.dev);

	if (pdata->sscd_report)
		return -EPIPE;

	/* Ignore the trigger time less than 30 seconds */
	if (trigger_time - last_trigger_time < 30)
		return -EBUSY;
	last_trigger_time = trigger_time;

	/* ; separate the crash reason and coredump value */
	next = strchr(buf, ';');
	if (!next)
		return -EINVAL;

	length = (int)(next - buf);
	reason = kstrndup(buf, length + 1, GFP_KERNEL);
	if (!reason)
		return -ENOMEM;
	reason[length] = '\0';

	length = strlen(next + 1);
	seg.addr = next + 1;
	seg.size = length;
	pdata->sscd_report(&priv->sscd_dev, &seg, 1, 0, reason);

	kfree(reason);

	return count;
}
static DEVICE_ATTR_WO(coredump);

static ssize_t nstandby_show(
		struct device *dev, struct device_attribute *attr, char *buf)
{
	int value = 0;
	struct spi_device *spi = to_spi_device(dev);
	struct bcm_spi_priv *priv = spi_get_drvdata(spi);
	value = gpio_get_value(priv->nstandby);

	return scnprintf(buf, PAGE_SIZE, "%d\n", value);
}

static ssize_t nstandby_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct spi_device *spi = to_spi_device(dev);
	struct bcm_spi_priv *priv = spi_get_drvdata(spi);

#ifdef DEBUG_1HZ_STAT
	dev_dbg(dev, "nstandby, buf is %s\n", buf);
#endif

	if (buf[0] == '0')
		gpio_set_value(priv->nstandby, 0);
	else
		gpio_set_value(priv->nstandby, 1);

	return count;
}

static DEVICE_ATTR_RW(nstandby);

static ssize_t sspmcureq_show(
		struct device *dev, struct device_attribute *attr, char *buf)
{
	int value = 0;
	struct spi_device *spi = to_spi_device(dev);
	struct bcm_spi_priv *priv = spi_get_drvdata(spi);
	value = gpio_get_value(priv->mcu_req);

	return scnprintf(buf, PAGE_SIZE, "%d\n", value);
}

static ssize_t sspmcureq_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct spi_device *spi = to_spi_device(dev);
	struct bcm_spi_priv *priv = spi_get_drvdata(spi);

	dev_dbg(dev, "sspmcureq, buf is %s\n", buf);

	if (buf[0] == '0')
		gpio_set_value(priv->mcu_req, 0);
	else
		gpio_set_value(priv->mcu_req, 1);

	return count;
}

static DEVICE_ATTR_RW(sspmcureq);

#ifdef CONFIG_TRANSFER_STAT
void bcm_ssi_clear_trans_stat(struct bcm_spi_priv *priv)
{
	memset(priv->trans_stat, 0, sizeof(priv->trans_stat));
}

void bcm_ssi_print_trans_stat(struct bcm_spi_priv *priv)
{
	struct bcm_spi_transfer_stat *trans = &priv->trans_stat[0];

	dev_info(&priv->spi->dev, "DBG SPI @ TX: <255B = %d, <1K = %d, <2K = %d, <4K = %d, <8K = %d, <16K = %d, <32K = %d, <64K = %d, total = %ld, min = %ld, max = %ld",
			trans->len_255, trans->len_1K, trans->len_2K,
			trans->len_4K, trans->len_8K, trans->len_16K,
			trans->len_32K, trans->len_64K, trans->len_total,
			trans->len_min, trans->len_max);

	trans = &priv->trans_stat[1];
	dev_info(&priv->spi->dev, "DBG SPI @ RX: <255B = %d, <1K = %d, <2K = %d, <4K = %d, <8K = %d, <16K = %d, <32K = %d, <64K = %d, total = %ld, min = %ld, max = %ld",
			trans->len_255, trans->len_1K, trans->len_2K,
			trans->len_4K, trans->len_8K, trans->len_16K,
			trans->len_32K, trans->len_64K, trans->len_total,
			trans->len_min, trans->len_max);

	dev_info(&priv->spi->dev, "DBG SPI @ PZC: retries = %d, delays = %d",
			priv->ssi_tx_pzc_retries, priv->ssi_tx_pzc_retry_delays);
}

static void bcm_ssi_calc_trans_stat(
		struct bcm_spi_transfer_stat *trans, unsigned short length)
{
	if (length <= 255)
		trans->len_255++;
	else if (length <= 1024)
		trans->len_1K++;
	else if (length <= (2 * 1024))
		trans->len_2K++;
	else if (length <= (4 * 1024))
		trans->len_4K++;
	else if (length <= (8 * 1024))
		trans->len_8K++;
	else if (length <= (16 * 1024))
		trans->len_16K++;
	else if (length <= (32 * 1024))
		trans->len_32K++;
	else
		trans->len_64K++;

	if (length > trans->len_max)
		trans->len_max = length;

	if (trans->len_min == 0 || length < trans->len_min)
		trans->len_min = length;

	trans->len_total += length;
}
#endif /* CONFIG_TRANSFER_STAT */

static const unsigned long m_ulRxBufferBlockSize[4] = {32, 256, 1024 * 2, 1024 * 16};

static unsigned long bcm_ssi_chk_pzc(struct bcm_spi_priv *priv,
		unsigned char stat_byte, bool bprint)
{
	unsigned long rx_buffer_blk_bytes =
		m_ulRxBufferBlockSize[(
				stat_byte & HSI_F_MOSI_CTRL_SZE_MASK) >>
				HSI_F_MOSI_CTRL_SZE_SHIFT];
	unsigned long rx_buffer_blk_counts =
		(unsigned long)((stat_byte & HSI_F_MOSI_CTRL_CNT_MASK) >>
			HSI_F_MOSI_CTRL_CNT_SHIFT);

	priv->rx_buffer_avail_bytes = rx_buffer_blk_bytes * rx_buffer_blk_counts;

	if (!bprint)
		return priv->rx_buffer_avail_bytes;

	if (stat_byte & HSI_F_MOSI_CTRL_PE_MASK) {
		dev_dbg(&priv->spi->dev, "DBG SPI @ PZC: rx stat 0x%02x%s avail %lu",
			stat_byte,
			stat_byte & HSI_F_MOSI_CTRL_PE_MASK ?  "(PE)" : "   ",
			priv->rx_buffer_avail_bytes);
	}

#ifdef CONFIG_REG_IO
	if (stat_byte & HSI_F_MOSI_CTRL_PE_MASK) {
		u8 regval;

		bcm_dreg_read(priv, "HSI_ERROR_STATUS(R) ",
				HSI_ERROR_STATUS, &regval, 1);

		if (regval & HSI_ERROR_STATUS_STRM_FIFO_OVFL)
			dev_err(&priv->spi->dev, "(rx_strm_fifo_ovfl)");

		regval = HSI_ERROR_STATUS_ALL_ERRORS;
		bcm_dreg_write(priv, "HSI_ERROR_STATUS(W) ",
				HSI_ERROR_STATUS, &regval, 1);
	}
#endif /* CONFIG_REG_IO */

	return priv->rx_buffer_avail_bytes;
}


/**********************************
 *
 *	File Operations
 *
 **********************************/
static int bcm_spi_open(struct inode *inode, struct file *filp)
{
	/*
	 * Initially, file->private_data points device itself
	 * and we can get our priv structs from it.
	 */
	struct bcm_spi_priv *priv = container_of(filp->private_data,
			struct bcm_spi_priv, misc);
	struct bcm_spi_strm_protocol *strm;
	unsigned long flags;
	unsigned char fc_mask, len_mask, duplex_mask;

#ifdef CONFIG_REG_IO
	u8 regval8[2];
	u32 regval32[16];
#endif

	if (priv->busy)
		return -EBUSY;

	priv->busy = true;

	/* Reset circ buffer */
	priv->read_buf.head = priv->read_buf.tail = 0;
	priv->write_buf.head = priv->write_buf.tail = 0;

	priv->packet_received = 0;

	/* Enable irq */
	spin_lock_irqsave(&priv->irq_lock, flags);
	if (!atomic_xchg(&priv->irq_enabled, 1))
		enable_irq(priv->spi->irq);

	spin_unlock_irqrestore(&priv->irq_lock, flags);

	priv->irq_wakeup_enabled = (enable_irq_wake(priv->spi->irq) == 0);

	filp->private_data = priv;
#ifdef DEBUG_1HZ_STAT
	bbd_enable_stat(priv->bbd);
#endif

	strm = &priv->tx_strm;
	strm->pckt_len = SSI_LEN == 2 ? 2 : 1;
	len_mask = SSI_LEN == 2 ? SSI_PCKT_2B_LENGTH : SSI_PCKT_1B_LENGTH;
	duplex_mask = SSI_MODE != 0 ? SSI_MODE_FULL_DUPLEX : SSI_MODE_HALF_DUPLEX;

	fc_mask = SSI_FLOW_CONTROL_DISABLED;
	strm->fc_len = 0;
	if (SSI_MODE == 0) {
		/* SSI_MODE_HALF_DUPLEX; */
		strm->pckt_len = 0;
	}
	/* 1 for tx cmd byte */
	strm->ctrl_len = strm->pckt_len + strm->fc_len + 1;

	strm->frame_len = SSI_LEN == 2 ? MAX_SPI_FRAME_LEN : MAX_SPI_DREG_FRAME_LEN;
	strm->ctrl_byte = duplex_mask | SSI_MODE_STREAM |
		len_mask | SSI_WRITE_TRANS | fc_mask;

	/* TX SPI Streaming Protocol in details */
#ifdef DEBUG_1HZ_STAT
	dev_info(&priv->spi->dev, "tx ctrl %02X: total %d = len %d + fc %d + cmd 1",
			strm->ctrl_byte, strm->ctrl_len,
			strm->pckt_len, strm->fc_len);
#endif

	strm = &priv->rx_strm;
	strm->pckt_len = SSI_LEN == 2 ? 2 : 1;
	strm->fc_len = 0;
	/* 1 for rx stat byte */
	strm->ctrl_len = strm->pckt_len + strm->fc_len + 1;
	strm->frame_len = SSI_LEN == 2 ? MAX_SPI_FRAME_LEN : MAX_SPI_DREG_FRAME_LEN;
	strm->ctrl_byte = duplex_mask | SSI_MODE_STREAM | len_mask |
		(SSI_MODE == SSI_MODE_FULL_DUPLEX ?
		 SSI_WRITE_TRANS : SSI_READ_TRANS);

	/* RX SPI Streaming Protocol in details */
#ifdef DEBUG_1HZ_STAT
	dev_info(&priv->spi->dev, "rx ctrl %02X: total %d = len %d + fc %d + stat 1\n",
			strm->ctrl_byte, strm->ctrl_len,
			strm->pckt_len, strm->fc_len);

	dev_info(&priv->spi->dev, "SPI @ %d: %s Duplex Strm mode, %dB Len, w/o FC, Frame Len %u : tx ctrl %02X, rx ctrl %02X\n",
			BCM_BITRATE,
			SSI_MODE != 0 ? "Full" : "Half",
			SSI_LEN == 2 ? 2 : 1,
			strm->frame_len,
			priv->tx_strm.ctrl_byte,
			priv->rx_strm.ctrl_byte);
#endif

#ifdef CONFIG_REG_IO
	bcm_dreg_read(priv, "HSI_STATUS     ",
			HSI_STATUS, regval8, 1);
	bcm_dreg_read(priv, "HSI_ERROR_STATUS(R) ",
			HSI_ERROR_STATUS, regval8, 1);
	bcm_ireg_read(priv, "INTR_MASK/STAT ",
			HSI_INTR_MASK, regval32, 2);
	bcm_ireg_read(priv, "RNGDMA_RX      ",
			HSI_RNGDMA_RX_BASE_ADDR, regval32, 8);
	bcm_ireg_read(priv, "RNGDMA_TX      ",
			HSI_RNGDMA_TX_BASE_ADDR, regval32, 8);
	bcm_ireg_read(priv, "HSI_CTRL       ",
			HSI_CTRL, regval32, 4);
	bcm_ireg_read(priv, "ADL_ABR        ",
			HSI_ADL_ABR_CONTROL, regval32, 4);
	bcm_ireg_read(priv, "RSTN/STBY/EN   ",
			HSI_RESETN, regval32, 4);
	bcm_ireg_read(priv, "STRM/CMND      ",
			HSI_STRM_FIFO_STATUS, regval32, 2);
#endif

#ifdef CONFIG_TRANSFER_STAT
	bcm_ssi_print_trans_stat(priv);
	bcm_ssi_clear_trans_stat(priv);
#endif
	priv->ssi_tx_fail       = 0;
	priv->ssi_tx_pzc_retries = 0;
	priv->ssi_tx_pzc_retry_delays = 0;
	priv->ssi_pm_semaphore = 0;
	priv->rx_buffer_avail_bytes = HSI_PZC_MAX_RX_BUFFER;

	return 0;
}

static int bcm_spi_release(struct inode *inode, struct file *filp)
{
	struct bcm_spi_priv *priv = filp->private_data;
	unsigned long flags;

	priv->busy = false;

#ifdef CONFIG_TRANSFER_STAT
	bcm_ssi_print_trans_stat(priv);
#endif


#ifdef DEBUG_1HZ_STAT
	bbd_disable_stat(priv->bbd);
#endif
	/* Disable irq */
	spin_lock_irqsave(&priv->irq_lock, flags);
	if (atomic_xchg(&priv->irq_enabled, 0))
		disable_irq_nosync(priv->spi->irq);

	spin_unlock_irqrestore(&priv->irq_lock, flags);

	if (priv->irq_wakeup_enabled)
		disable_irq_wake(priv->spi->irq);

	return 0;
}

static ssize_t bcm_spi_read(
		struct file *filp, char __user *buf, size_t size, loff_t *ppos)
{
	struct bcm_spi_priv *priv = filp->private_data;
	struct circ_buf *circ = &priv->read_buf;
	size_t rd_size = 0;

	mutex_lock(&priv->rlock);

	/*
	 * Copy from circ buffer to user
	 * We may require 2 copies from [tail..end] and [end..head]
	 */
	do {
		size_t cnt_to_end = CIRC_CNT_TO_END(
				circ->head, circ->tail, BCM_SPI_READ_BUF_SIZE);
		size_t copied = min(cnt_to_end, size);

		if (copy_to_user(buf + rd_size,
				circ->buf + circ->tail, copied)){
			dev_err(&priv->spi->dev, "failed to copy to user.\n");
			mutex_unlock(&priv->rlock);
			return -EFAULT;
		}
		size -= copied;
		rd_size += copied;
		circ->tail = (circ->tail + copied) & (BCM_SPI_READ_BUF_SIZE-1);

	} while (size > 0 && CIRC_CNT(circ->head,
				circ->tail, BCM_SPI_READ_BUF_SIZE));
	mutex_unlock(&priv->rlock);

#ifdef DEBUG_1HZ_STAT
	bbd_update_stat(priv->bbd, STAT_RX_LHD, rd_size);
#endif

	return rd_size;
}

static ssize_t bcm_spi_write(
		struct file *filp, const char __user *buf,
		size_t size, loff_t *ppos)
{
	struct bcm_spi_priv *priv = filp->private_data;
	struct circ_buf *circ = &priv->write_buf;
	size_t wr_size = 0;

	mutex_lock(&priv->wlock);
	/*
	 * Copy from user into circ buffer
	 * We may require 2 copies from [tail..end] and [end..head]
	 */
	do {
		size_t space_to_end = CIRC_SPACE_TO_END(circ->head,
				circ->tail, BCM_SPI_WRITE_BUF_SIZE);
		size_t copied = min(space_to_end, size);

		if (copy_from_user(circ->buf + circ->head,
					buf + wr_size, copied)){
			dev_err(&priv->spi->dev, "failed to copy from user.\n");
			mutex_unlock(&priv->wlock);
			return -EFAULT;
		}
		size -= copied;
		wr_size += copied;
		circ->head = (circ->head + copied) &
			(BCM_SPI_WRITE_BUF_SIZE - 1);
	} while (size > 0 && CIRC_SPACE(circ->head, circ->tail,
				BCM_SPI_WRITE_BUF_SIZE));
	mutex_unlock(&priv->wlock);

	/*
	 * kick start rxtx thread
	 * we don't want to queue work in suspending and shutdown
	 */
	if (!atomic_read(&priv->suspending))
		queue_work(priv->serial_wq,
				(struct work_struct *)&priv->rxtx_work);

#ifdef DEBUG_1HZ_STAT
	bbd_update_stat(priv->bbd, STAT_TX_LHD, wr_size);
#endif
	return wr_size;
}

static unsigned int bcm_spi_poll(struct file *filp, poll_table *wait)
{
	struct bcm_spi_priv *priv = filp->private_data;
	struct circ_buf *rd_circ = &priv->read_buf;
	struct circ_buf *wr_circ = &priv->write_buf;
	unsigned int mask = 0;

	poll_wait(filp, &priv->poll_wait, wait);

	if (CIRC_CNT(rd_circ->head, rd_circ->tail, BCM_SPI_READ_BUF_SIZE))
		mask |= POLLIN;

	if (CIRC_SPACE(wr_circ->head, wr_circ->tail, BCM_SPI_WRITE_BUF_SIZE))
		mask |= POLLOUT;

	return mask;
}


static const struct file_operations bcm_spi_fops = {
	.owner = THIS_MODULE,
	.open = bcm_spi_open,
	.release = bcm_spi_release,
	.read = bcm_spi_read,
	.write = bcm_spi_write,
	.poll = bcm_spi_poll,
};



/* Misc. functions */

unsigned long bcm_clock_get_ms(void)
{
	struct timespec64 t;
	unsigned long now;
	static unsigned long init_time;

	ktime_get_real_ts64(&t);
	now = t.tv_nsec / 1000000 + t.tv_sec * 1000;
	if (init_time == 0)
		init_time = now;

	return now - init_time;
}

void wait1secDelay(unsigned int count)
{
	if (count <= 100)
		usleep_range(1000, 2000);
	else
		usleep_range(20000, 30000);
}

#ifdef CONFIG_MCU_WAKEUP
/**
 * bcm4773_hello - wakeup chip by toggling mcu_req
 * while monitoring mcu_resp to check if awake
 */
static bool bcm477x_hello(struct bcm_spi_priv *priv)
{
	int count = 0;
#define MAX_RESP_CHECK_COUNT 100 /* 100 msec */

	unsigned long start_time, delta;

	start_time = bcm_clock_get_ms();
	gpio_set_value(priv->mcu_req, 1);
	while (!gpio_get_value(priv->mcu_resp)) {
		if (count++ > MAX_RESP_CHECK_COUNT) {
			gpio_set_value(priv->mcu_req, 0);
#ifdef DEBUG_1HZ_STAT
			dev_err(&priv->spi->dev, " MCU_REQ_RESP timeout. MCU_RESP(gpio%d) not responding to MCU_REQ(gpio%d)\n",
					priv->mcu_resp, priv->mcu_req);
#endif
			return false;
		}

		wait1secDelay(count);

		/*if awake, done */
		if (gpio_get_value(priv->mcu_resp))
			break;

		if (count % 20 == 0) {
			gpio_set_value(priv->mcu_req, 0);
			usleep_range(1000, 2000);
			gpio_set_value(priv->mcu_req, 1);
			usleep_range(1000, 2000);
		}
	}

	delta = bcm_clock_get_ms() - start_time;

	if (count > 100)
		dev_err(&priv->spi->dev, "hello consumed %lu = clock_get_ms() - start_time; msec",
				delta);

	return true;
}
#endif

/**
 * bcm4773_bye - set mcu_req low to let chip go to sleep
 */
static void bcm477x_bye(struct bcm_spi_priv *priv)
{
	gpio_set_value(priv->mcu_req, 0);
}

static void pk_log(struct bcm_spi_priv *priv, char *dir,
		unsigned char *data, int len)
{
	const char ic = 'D';

	if (likely(!priv->ssi_dbg))
		return;

	/*
	 * TODO: There is print issue. Printing 7 digits instead of 6
	 * when clock is over 1000000. "% 1000000" added
	 * E.g.
	 * #999829D w 0x68,  1: A2
	 * #999829D r 0x68, 34: 8D 00 01 52 5F B0 01 B0 00 8E 00 01 53 8B
	 * B0 01 B0 00 8F 00 01 54 61 B0 01 B0 00 90 00 01 55 B5
	 *          r B0 01
	 * #1000001D w 0x68, 1: A1
	 * #1000001D r 0x68, 1: 00
	 */
	dev_info(&priv->spi->dev, "#%06ld%c %2s,\t  %5d: ",
			bcm_clock_get_ms() % 1000000, ic, dir, len);

	print_hex_dump(KERN_INFO, dir[0] == 'r' ? "r " : "w ",
			DUMP_PREFIX_NONE, 32, 1, data, len, false);
}

void bcm_ssi_debug(struct device *dev, int type, bool value)
{
	struct spi_device *spi = to_spi_device(dev);
	struct bcm_spi_priv *priv = spi_get_drvdata(spi);

	switch (type) {
	case 0:
		priv->ssi_dbg = value;
		break;
	case 1:
		priv->ssi_dbg_pzc = value;
		break;
	case 2:
		priv->ssi_dbg_rng = value;
		break;
	}
}
EXPORT_SYMBOL_GPL(bcm_ssi_debug);

/* SSI tx/rx functions */

static unsigned short bcm_ssi_get_len(
		unsigned char ctrl_byte, unsigned char *data)
{
	if (ctrl_byte & SSI_PCKT_2B_LENGTH)
		return ((unsigned short)data[0] +
			((unsigned short)data[1] << 8));

	return (unsigned short)data[0];
}

static void bcm_ssi_set_len(
	unsigned char ctrl_byte, unsigned char *data, unsigned short len)
{
	if (ctrl_byte & SSI_PCKT_2B_LENGTH) {
		data[0] = (unsigned char)(len & 0xff);
		data[1] = (unsigned char)((len >> 8)  & 0xff);
	} else {
		data[0] = (unsigned char)len;
	}
}

static void bcm_ssi_clr_len(unsigned char ctrl_byte, unsigned char *data)
{
	bcm_ssi_set_len(ctrl_byte, data, 0);
}


int bcm_spi_sync(struct bcm_spi_priv *priv, void *tx_buf,
		void *rx_buf, int len, int bits_per_word)
{
	struct spi_message msg;
	struct spi_transfer xfer;
	int ret;

	/* Init */
	spi_message_init(&msg);
	memset(&xfer, 0, sizeof(xfer));
	spi_message_add_tail(&xfer, &msg);

	/* Setup */
	msg.spi = priv->spi;
	xfer.len = len;
	xfer.tx_buf = tx_buf;
	xfer.rx_buf = rx_buf;
	xfer.bits_per_word = bits_per_word;

	/* Sync */
	pk_log(priv, "w", (unsigned char *)xfer.tx_buf, len);
	ret = spi_sync(msg.spi, &msg);
	pk_log(priv, "r", (unsigned char *)xfer.rx_buf, len);

	if (ret)
		dev_err(&priv->spi->dev, "spi_sync error for cmd:0x%x, return=%d\n",
			((struct bcm_ssi_tx_frame *)xfer.tx_buf)->cmd, ret);

	return ret;
}


static int bcm_ssi_tx(struct bcm_spi_priv *priv, int length)
{
	struct bcm_ssi_tx_frame *tx = priv->tx_buf;
	struct bcm_ssi_rx_frame *rx = priv->rx_buf;
	struct bcm_spi_strm_protocol *strm = &priv->tx_strm;
	int bits_per_word = (length + strm->ctrl_len >= MIN_DMA_SIZE) ?
				CONFIG_SPI_DMA_BITS_PER_WORD : 8;
	int ret;
	unsigned short m_write;
	unsigned short bytes_to_write = (unsigned short)length;
	unsigned short bytes_written = 0;
	unsigned short n_read = 0; /* for Full Duplex only */
	unsigned short frame_data_size = strm->frame_len - strm->ctrl_len;

	m_write = bytes_to_write;

	tx->cmd = strm->ctrl_byte; /* SSI_WRITE_HD etc. */

	bytes_to_write = max(m_write, n_read);

	if (strm->pckt_len != 0)
		bcm_ssi_set_len(strm->ctrl_byte, tx->data, m_write);

	/* ctrl_len is for tx len + fc */
	ret = bcm_spi_sync(priv, tx, rx, bytes_to_write +
			strm->ctrl_len, bits_per_word);

	if (ret) {
		priv->ssi_tx_fail++;
		return ret;
		/* TODO: failure, operation should gets 0 to continue */
	}

	if (strm->pckt_len != 0) {
		unsigned short m_write =
			bcm_ssi_get_len(strm->ctrl_byte, tx->data);
		/* Just for understanding SPI Streaming Protocol */
		if (m_write > frame_data_size) {
			/* The h/w malfunctioned ? */
			dev_err(&priv->spi->dev, "@ TX m_write %d is h/w overflowed of frame %d...Fail\n",
				m_write, frame_data_size);
		}
	}

	if (strm->ctrl_byte & SSI_MODE_FULL_DUPLEX) {
		unsigned char *data_p = rx->data + strm->pckt_len;

		n_read = bcm_ssi_get_len(strm->ctrl_byte, rx->data);
		if (n_read > frame_data_size) {
			dev_err(&priv->spi->dev, "@ FD n_read %d is h/w overflowed of frame %d...Fail\n",
				n_read, frame_data_size);
			n_read = frame_data_size;
		}

		if (m_write < n_read) {
			/* Call BBD */
			bcm_on_packet_received(priv, data_p, m_write);
			/* 1/2 bytes for len */
			n_read -= m_write;
			bytes_to_write -= m_write;
			data_p += (m_write + strm->fc_len);
		} else {
			bytes_to_write = n_read;
			/* No data available next time */
			n_read = 0;
		}

		/* Call BBD */
		if (bytes_to_write != 0) {
			bcm_on_packet_received(
				priv, data_p, bytes_to_write);
		}
	}

	bytes_written += bytes_to_write;

#ifdef CONFIG_TRANSFER_STAT
	bcm_ssi_calc_trans_stat(&priv->trans_stat[0], bytes_written);
#endif

	bcm_ssi_chk_pzc(priv, rx->status, priv->ssi_dbg_pzc);

	return ret;
}

static int bcm_ssi_rx(struct bcm_spi_priv *priv, size_t *length)
{
	struct bcm_ssi_tx_frame *tx = priv->tx_buf;
	struct bcm_ssi_rx_frame *rx = priv->rx_buf;
	struct circ_buf *rd_circ = &priv->read_buf;
	struct bcm_spi_strm_protocol *strm = &priv->rx_strm;
	unsigned short ctrl_len = strm->pckt_len + 1;  /* +1 for rx status */
	unsigned short payload_len;
	int bits_per_word = 8;
	size_t sz_to_recv = 0;

#ifdef CONFIG_REG_IO
	if (likely(priv->ssi_dbg_rng) &&
			(priv->packet_received > CONFIG_PACKET_RECEIVED)) {
		u32 regval32[8];

		bcm_ireg_read(priv, "RNGDMA_TX      ",
				HSI_RNGDMA_TX_SW_ADDR_OFFSET, regval32, 3);
	}
#endif

	/* TODO:: Check 1B or 2B mode */

	bcm_ssi_clr_len(strm->ctrl_byte, tx->data);
	/* tx and rx ctrl_byte(s) are same */
	tx->cmd = strm->ctrl_byte;
	/* SSI_READ_HD etc. */
	rx->status = 0;

	if (bcm_spi_sync(priv, tx, rx, ctrl_len, 8))
		return -1;

	bcm_ssi_chk_pzc(priv, rx->status, priv->ssi_dbg_pzc);

	payload_len = bcm_ssi_get_len(strm->ctrl_byte, rx->data);

	if (payload_len == 0) {
		/*
		 * TODO:  payload_len = MIN_SPI_FRAME_LEN;
		 * Needn't to use MAX_SPI_FRAME_LEN because don't
		 * know how many bytes is ready to really read
		 */
		dev_err(&priv->spi->dev, "@ RX length is still read to 0. Set %d\n", payload_len);
		return -1;
	}

	*length = min((unsigned short)(strm->frame_len - ctrl_len), payload_len);

	/* SWGNSSGLL-24487 : slowing down read speed if buffer is half full */
	if (CIRC_CNT(rd_circ->head, rd_circ->tail, BCM_SPI_READ_BUF_SIZE) >
			BCM_SPI_READ_BUF_SIZE / 2) {
		msleep(DELAY_FOR_SYSTEM_OVERLOADED_MS);
		if (*length >= READ_SIZE_FOR_SYSTEM_OVERLOADED)
			*length = READ_SIZE_FOR_SYSTEM_OVERLOADED;
	}

	sz_to_recv = *length + ctrl_len;
	if (sz_to_recv >= MIN_DMA_SIZE) {
		bits_per_word = CONFIG_SPI_DMA_BITS_PER_WORD;
		if (sz_to_recv & (CONFIG_SPI_DMA_BYTES_PER_WORD - 1))
			*length = (sz_to_recv & ~(CONFIG_SPI_DMA_BYTES_PER_WORD - 1))
					- ctrl_len;
	}
	memset(tx->data, 0, *length + ctrl_len - 1); /* -1 for status byte */

	if (bcm_spi_sync(priv, tx, rx, *length+ctrl_len, bits_per_word))
		return -1;

	payload_len = bcm_ssi_get_len(strm->ctrl_byte, rx->data);
	if (payload_len < *length)
		*length = payload_len;

	return 0;
}

static void bcm_check_overrun(struct bcm_spi_priv *priv, size_t avail)
{
	const long THRESHOLD_MS = 100;
	unsigned long curr_tick = bcm_clock_get_ms();

	if (!avail)
		return;

	if (curr_tick - priv->last_tick < THRESHOLD_MS) {
		priv->skip_count++;
		return;
	}

	if (priv->skip_count)
		dev_err(&priv->spi->dev, "%ld messages are skipped!\n", priv->skip_count);

	dev_err(&priv->spi->dev, "input overrun error by %zu bytes.\n", avail);
	priv->skip_count = 0;
	priv->last_tick = curr_tick;

}

static void bcm_on_packet_received(void *_priv, unsigned char *data,
		unsigned int size)
{
	struct bcm_spi_priv *priv = (struct bcm_spi_priv *)_priv;
	struct circ_buf *rd_circ = &priv->read_buf;
	size_t written = 0, avail = size;

#ifdef DEBUG_1HZ_STAT
	bbd_update_stat(priv->bbd, STAT_RX_SSI, size);
#endif
#ifdef CONFIG_TRANSFER_STAT
	bcm_ssi_calc_trans_stat(&priv->trans_stat[1], size);
#endif

	/* Copy into circ buffer */
	mutex_lock(&priv->rlock);
	do {
		size_t space_to_end = CIRC_SPACE_TO_END(
			rd_circ->head, rd_circ->tail, BCM_SPI_READ_BUF_SIZE);
		size_t copied = min(space_to_end, avail);

		memcpy(rd_circ->buf + rd_circ->head,
				data + written, copied);
		avail -= copied;
		written += copied;
		rd_circ->head = (rd_circ->head + copied) &
			(BCM_SPI_READ_BUF_SIZE - 1);
	} while (avail > 0 && CIRC_SPACE(rd_circ->head,
				rd_circ->tail, BCM_SPI_READ_BUF_SIZE));

	priv->packet_received += size;
	mutex_unlock(&priv->rlock);
	wake_up(&priv->poll_wait);
	bcm_check_overrun(priv, avail);
}

#ifdef DEBUG_1HZ_STAT
void bcm477x_debug_info(struct bcm_spi_priv *priv)
{
	int pin_ttyBCM, pin_MCU_REQ, pin_MCU_RESP;
	int irq_enabled;

	if (!priv)
		return;

	pin_ttyBCM = gpio_get_value(priv->host_req);
	pin_MCU_REQ = gpio_get_value(priv->mcu_req);
	pin_MCU_RESP = gpio_get_value(priv->mcu_resp);

	irq_enabled = atomic_read(&priv->irq_enabled);

	dev_info(&priv->spi->dev, "pin_ttyBCM:%d, pin_MCU_REQ:%d, pin_MCU_RESP:%d\n",
		pin_ttyBCM, pin_MCU_REQ, pin_MCU_RESP);
	dev_info(&priv->spi->dev, "irq_enabled:%d\n", irq_enabled);
}
#endif

static void bcm_rxtx_work_func(struct work_struct *work)
{
	struct bcm_spi_priv *priv = container_of(work,
			struct bcm_spi_priv, rxtx_work);
	struct circ_buf *rd_circ = &priv->read_buf;
	struct circ_buf *wr_circ = &priv->write_buf;
	struct bcm_spi_strm_protocol *strm = &priv->tx_strm;
	unsigned short rx_pckt_len = priv->rx_strm.pckt_len;
	int wait_for_pzc = 0;
	unsigned long flags;

#ifdef DEBUG_1HZ_STAT
	u64 ts_rx_start = 0;
	u64 ts_rx_end = 0;
	struct timespec64 ts;
	struct bbd_device *bbd = priv->bbd;
#endif

#ifdef CONFIG_MCU_WAKEUP
	if (!bcm477x_hello(priv)) {
#ifdef DEBUG_1HZ_STAT
		dev_err(&priv->spi->dev, "hello timeout!!\n");
		bcm477x_debug_info(priv);
#endif
		return;
	}
#endif

	do {
		int    ret = 0;
		size_t avail = 0;
		size_t written = 0;
		size_t sz_to_send = 0;

		/* Read first */
		if (!gpio_is_valid(priv->host_req)) {
			dev_err(&priv->spi->dev, "gpio host_req is invalid, return\n");
			return;
		}
		ret = gpio_get_value(priv->host_req);

		if (ret || wait_for_pzc) {
			wait_for_pzc = 0;
#ifdef DEBUG_1HZ_STAT
			if (bbd->stat1hz.ts_irq) {
				ts = ktime_to_timespec64(ktime_get_boottime());
				ts_rx_start = ts.tv_sec * 1000000000ULL
					+ ts.tv_nsec;
			}
#endif

			/* Receive SSI frame */
			if (bcm_ssi_rx(priv, &avail))
				break;

#ifdef DEBUG_1HZ_STAT
			if (ts_rx_start && !gpio_get_value(priv->host_req)) {
				ts = ktime_to_timespec64(ktime_get_boottime());
				ts_rx_end = ts.tv_sec * 1000000000ULL
					+ ts.tv_nsec;
			}
#endif
			/* Call BBD */
			bcm_on_packet_received(priv,
				priv->rx_buf->data + rx_pckt_len, avail);
		}

		/* Next, write */
		avail = CIRC_CNT(wr_circ->head, wr_circ->tail,
				BCM_SPI_WRITE_BUF_SIZE);

		if(!avail)
			continue;

		mutex_lock(&priv->wlock);
		/*
		 * For big packet, we should align xfer size to
		 * DMA word size and burst size.
		 * That is, SSI payload + one byte command should be
		 * multiple of (DMA word size * burst size)
		 */

		if (avail > (strm->frame_len - strm->ctrl_len))
			avail = strm->frame_len - strm->ctrl_len;

		ret = 0;

		/*
		 * SWGNSSGLL-15521 : Sometimes LHD does not write data
		 * because the following code blocks sending data to MCU
		 * Code is commented out because
		 * 'rx_buffer_avail_bytes'(PZC) is calculated in
		 * bcm_ssi_tx() inside loop in work queue
		 * (bcm_rxtx_work_func) below this code.
		 * It means 'rx_buffer_avail_bytes' doesn't reflect
		 * real available bytes in RX DMA RING buffer when
		 * work queue will be restarted
		 * because MCU is working independently from host.
		 * The 'rx_buffer_avail_bytes' can be tested inside
		 * bcm_ssi_tx but it may not guarantee correct
		 * condition also.
		 * SWGNSSGLL-16290 : FC detecting was broken when buffer
		 * is overflow Using PZC for a software workaround to
		 * not get into fifo overflow condition.
		 */
		if (avail > priv->rx_buffer_avail_bytes) {
			priv->rx_buffer_avail_bytes ? priv->ssi_tx_pzc_retries++ :
				priv->ssi_tx_pzc_retry_delays++;
			dev_dbg(&priv->spi->dev, "%d PZC %s, wr CIRC_CNT %lu, RNGDMA_RX %lu\n",
				priv->rx_buffer_avail_bytes ?
					priv->ssi_tx_pzc_retries : priv->ssi_tx_pzc_retry_delays,
				priv->rx_buffer_avail_bytes ?  "writes":"delays",
				avail,
				priv->rx_buffer_avail_bytes);
			if (priv->rx_buffer_avail_bytes == 0) {
				/*
				 *RNGDMA_RX is full ?
				 * If it's YES keep reading
				 */
				u32 regval32[8];

				bcm_ireg_read(priv, "RNGDMA_RX      ",
					HSI_RNGDMA_RX_SW_ADDR_OFFSET,
					regval32, 3);
			}
			avail = priv->rx_buffer_avail_bytes;
			usleep_range(1000, 2000);
			/*
			 * TODO: increase delay for waiting for
			 * draining RNGDMA_RX on MCU side ?
			 */
			wait_for_pzc = 1;
			/*
			 * This case is for when RNGDMA_RX is
			 * full and HOST_REQ is low
			 */
		}

		/* we should align xfer size to DMA word size. */
		sz_to_send = avail + strm->ctrl_len;
		if (sz_to_send >= MIN_DMA_SIZE &&
			sz_to_send & (CONFIG_SPI_DMA_BYTES_PER_WORD - 1))
			avail = (sz_to_send & ~(CONFIG_SPI_DMA_BYTES_PER_WORD - 1))
				- strm->ctrl_len;

		/* Copy from wr_circ the data */
		while (avail > 0) {
			size_t cnt_to_end = CIRC_CNT_TO_END(
				wr_circ->head, wr_circ->tail,
				BCM_SPI_WRITE_BUF_SIZE);
			size_t copied = min(cnt_to_end, avail);

			memcpy(priv->tx_buf->data + strm->pckt_len +
			written, wr_circ->buf + wr_circ->tail, copied);
			avail -= copied;
			written += copied;
			wr_circ->tail = (wr_circ->tail + copied) &
				(BCM_SPI_WRITE_BUF_SIZE - 1);
		}

		/* Transmit SSI frame */
		if (written)
			ret = bcm_ssi_tx(priv, written);

		mutex_unlock(&priv->wlock);

		if (ret)
			break;

		/*
		 * SWGNSSAND-2159  While looping,
		 * wake up lhd only if rx ring is more than 12.5% full
		 */
		if (CIRC_CNT(rd_circ->head, rd_circ->tail, BCM_SPI_READ_BUF_SIZE) >
				BCM_SPI_READ_BUF_SIZE / 8) {
			wake_up(&priv->poll_wait);
		}
#ifdef DEBUG_1HZ_STAT
		bbd_update_stat(bbd, STAT_TX_SSI, written);
#endif

	} while (!atomic_read(&priv->suspending) &&
		(gpio_get_value(priv->host_req) ||
		CIRC_CNT(wr_circ->head, wr_circ->tail, BCM_SPI_WRITE_BUF_SIZE)));

	bcm477x_bye(priv);

	wake_up(&priv->poll_wait);

	/* Enable irq */
	spin_lock_irqsave(&priv->irq_lock, flags);

	/* we don't want to enable irq when going to suspending */
	if (!atomic_read(&priv->suspending))
		if (!atomic_xchg(&priv->irq_enabled, 1))
			enable_irq(priv->spi->irq);

	spin_unlock_irqrestore(&priv->irq_lock, flags);

#ifdef DEBUG_1HZ_STAT
	if (bbd->stat1hz.ts_irq && ts_rx_start && ts_rx_end) {
		u64 lat = ts_rx_start - bbd->stat1hz.ts_irq;
		u64 dur = ts_rx_end - ts_rx_start;

		bbd->stat1hz.min_rx_lat = (lat < bbd->stat1hz.min_rx_lat) ?
			lat : bbd->stat1hz.min_rx_lat;
		bbd->stat1hz.max_rx_lat = (lat > bbd->stat1hz.max_rx_lat) ?
			lat : bbd->stat1hz.max_rx_lat;
		bbd->stat1hz.min_rx_dur = (dur < bbd->stat1hz.min_rx_dur) ?
			dur : bbd->stat1hz.min_rx_dur;
		bbd->stat1hz.max_rx_dur = (dur > bbd->stat1hz.max_rx_dur) ?
			dur : bbd->stat1hz.max_rx_dur;
		bbd->stat1hz.ts_irq = 0;
	}
#endif
}


/* IRQ Handler */
static irqreturn_t bcm_irq_handler(int irq, void *pdata)
{
	struct bcm_spi_priv *priv = (struct bcm_spi_priv *) pdata;

	if (!gpio_get_value(priv->host_req))
		return IRQ_HANDLED;
#ifdef DEBUG_1HZ_STAT
	{
		struct bbd_device *bbd = priv->bbd;

		struct timespec64 ts;

		ts = ktime_to_timespec64(ktime_get_boottime());
		bbd->stat1hz.ts_irq = ts.tv_sec * 1000000000ULL + ts.tv_nsec;
	}
#endif
	/* Disable irq */
	spin_lock(&priv->irq_lock);
	if (atomic_xchg(&priv->irq_enabled, 0))
		disable_irq_nosync(priv->spi->irq);

	spin_unlock(&priv->irq_lock);

	/* we don't want to queue work in suspending and shutdown */
	if (!atomic_read(&priv->suspending))
		queue_work(priv->serial_wq,
			(struct work_struct *)&priv->rxtx_work);

	return IRQ_HANDLED;
}

static int gps_initialize_pinctrl(struct bcm_spi_priv *data)
{
	int ret = 0;
	struct device *dev = &data->spi->dev;

	/* Get pinctrl if target uses pinctrl */
	data->ts_pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR_OR_NULL(data->ts_pinctrl)) {
		dev_err(&data->spi->dev, "Target does not use pinctrl\n");
		ret = PTR_ERR(data->ts_pinctrl);
		data->ts_pinctrl = NULL;
		return ret;
	}

	data->gpio_state_active
		= pinctrl_lookup_state(data->ts_pinctrl, "gps_active");
	if (IS_ERR_OR_NULL(data->gpio_state_active)) {
		dev_err(&data->spi->dev, "Can not get ts default pinstate\n");
		ret = PTR_ERR(data->gpio_state_active);
		data->ts_pinctrl = NULL;
		return ret;
	}

	data->gpio_state_suspend
		= pinctrl_lookup_state(data->ts_pinctrl, "gps_suspend");
	if (IS_ERR_OR_NULL(data->gpio_state_suspend)) {
		dev_err(&data->spi->dev, "Can not get ts sleep pinstate\n");
		ret = PTR_ERR(data->gpio_state_suspend);
		data->ts_pinctrl = NULL;
		return ret;
	}

	return ret;
}

static int gps_pinctrl_select(struct bcm_spi_priv *data, bool on)
{
	int ret = 0;
	struct pinctrl_state *pins_state;

	pins_state = on ? data->gpio_state_active : data->gpio_state_suspend;
	if (!IS_ERR_OR_NULL(pins_state)) {
		ret = pinctrl_select_state(data->ts_pinctrl, pins_state);
		if (ret) {
			dev_err(&data->spi->dev, "can not set %s pins\n",
				on ? "gps_active" : "gps_suspend");
			return ret;
		}
	} else {
		dev_err(&data->spi->dev, "not a valid '%s' pinstate\n",
			on ? "gps_active" : "gps_suspend");
	}

	return ret;
}



/* SPI driver operations */

static int bcm_spi_suspend(struct device *dev)
{
	struct spi_device *spi = to_spi_device(dev);
	struct bcm_spi_priv *priv = spi_get_drvdata(spi);
	unsigned long flags;

	atomic_set(&priv->suspending, 1);

	/* Disable irq */
	spin_lock_irqsave(&priv->irq_lock, flags);
	if (atomic_xchg(&priv->irq_enabled, 0))
		disable_irq_nosync(spi->irq);

	spin_unlock_irqrestore(&priv->irq_lock, flags);

	if (priv->serial_wq)
		flush_workqueue(priv->serial_wq);

	priv->ssi_pm_semaphore++;
	return 0;
}

static int bcm_spi_resume(struct device *dev)
{
	struct spi_device *spi = to_spi_device(dev);
	struct bcm_spi_priv *priv = spi_get_drvdata(spi);
	unsigned long flags;

	atomic_set(&priv->suspending, 0);

	/* Enable irq */
	spin_lock_irqsave(&priv->irq_lock, flags);
	if (!atomic_xchg(&priv->irq_enabled, 1))
		enable_irq(spi->irq);

	spin_unlock_irqrestore(&priv->irq_lock, flags);

	priv->ssi_pm_semaphore--;
	return 0;
}

static void bcm_spi_shutdown(struct spi_device *spi)
{
	struct bcm_spi_priv *priv = spi_get_drvdata(spi);
	unsigned long flags;

#ifdef CONFIG_TRANSFER_STAT
	bcm_ssi_print_trans_stat(priv);
#endif

	atomic_set(&priv->suspending, 1);

	/* Disable irq */
	spin_lock_irqsave(&priv->irq_lock, flags);
	if (atomic_xchg(&priv->irq_enabled, 0))
		disable_irq_nosync(spi->irq);

	spin_unlock_irqrestore(&priv->irq_lock, flags);

	flush_workqueue(priv->serial_wq);
	destroy_workqueue(priv->serial_wq);
	priv->serial_wq = NULL;
}

static int bcm_spi_probe(struct spi_device *spi)
{
	int host_req, mcu_req, mcu_resp;
	int gps_power;
	int nstandby;
	struct bcm_spi_priv *priv;
	bool skip_validity_check;
	bool legacy_patch = false;
	int ret;
	int error = 0;

	/* Check GPIO# */
#ifndef CONFIG_OF
	dev_err(&spi->dev, "Check platform_data for bcm device\n");
#else
	if (!spi->dev.of_node) {
		dev_err(&spi->dev, "Failed to find of_node\n");
		goto err_exit;
	}
#endif

	host_req = of_get_named_gpio(spi->dev.of_node, "host-req-gpios", 0);
	mcu_req  = of_get_named_gpio(spi->dev.of_node, "mcu-req-gpios", 0);
	mcu_resp = of_get_named_gpio(spi->dev.of_node, "mcu-resp-gpios", 0);
	nstandby = of_get_named_gpio(spi->dev.of_node, "nstandby-gpios", 0);
	skip_validity_check = of_property_read_bool(spi->dev.of_node,
			"ssp-skip-validity-check");
#ifdef CONFIG_SENSORS_BBD_LEGACY_PATCH
	legacy_patch = of_property_read_bool(spi->dev.of_node,
			"ssp-legacy-patch");
#endif

	dev_info(&spi->dev, "ssp-host-req=%d, ssp-mcu_req=%d, ssp-mcu-resp=%d nstandby=%d \n",
		host_req, mcu_req, mcu_resp, nstandby);
	if (host_req < 0 || mcu_req < 0 || mcu_resp < 0 || nstandby < 0) {
		dev_err(&spi->dev, "GPIO value not correct\n");
		goto err_exit;
	}

	/* Check IRQ# */
	ret = gpio_request(host_req, "HOST REQ");
	if (ret) {
		dev_err(&spi->dev, "failed to request HOST REQ, ret:%d", ret);
		goto err_exit;
	}
	spi->irq = gpio_to_irq(host_req);
	if (spi->irq < 0) {
		dev_err(&spi->dev, "irq=%d for host_req=%d not correct\n",
				spi->irq, host_req);
		goto err_exit;
	}

	/* Config GPIO */
	ret = gpio_request(mcu_req, "MCU REQ");
	if (ret) {
		dev_err(&spi->dev, "failed to request MCU REQ, ret:%d", ret);
		goto err_exit;
	}
	ret = gpio_direction_output(mcu_req, 0);
	if (ret) {
		dev_err(&spi->dev, "failed set MCU REQ as input mode, ret:%d",
				ret);
		goto err_exit;
	}
	ret = gpio_request(mcu_resp, "MCU RESP");
	if (ret) {
		dev_err(&spi->dev, "failed to request MCU RESP, ret:%d", ret);
		goto err_exit;
	}
	ret = gpio_direction_input(mcu_resp);
	if (ret) {
		dev_err(&spi->dev, "failed set MCU RESP as input mode, ret:%d",
				ret);
		goto err_exit;
	}

	ret = gpio_request(nstandby, "GPS NSTANDBY");
	if (ret) {
		dev_err(&spi->dev, "failed to request GPS NSTANDBY, ret:%d", ret);
		goto err_exit;
	}
	ret = gpio_direction_output(nstandby, 0);
	if (ret) {
		dev_err(&spi->dev, "failed set GPS NSTANDBY as out mode, ret:%d",
				ret);
		goto err_exit;
	}

	/* enable gps_power */
	gps_power = of_get_named_gpio(spi->dev.of_node, "gps-power-enable", 0);
	if (gps_power >= 0) {
		ret = gpio_request(gps_power, "GPS POWER");
		if (ret) {
			dev_err(&spi->dev, "failed to request GPS POWER, ret:%d", ret);
			goto err_exit;
		}
		ret = gpio_direction_output(gps_power, 1);
		if (ret) {
			dev_err(&spi->dev, "failed set GPS POWER as out mode, ret:%d", ret);
			goto err_exit;
		}
	}

	/* Alloc everything */
	priv = devm_kzalloc(&spi->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		goto err_exit;

	priv->skip_validity_check = skip_validity_check;
	priv->spi = spi;
	priv->tx_buf = devm_kmalloc(&spi->dev,
			sizeof(struct bcm_ssi_tx_frame), GFP_KERNEL);
	priv->rx_buf = devm_kmalloc(&spi->dev,
			sizeof(struct bcm_ssi_rx_frame), GFP_KERNEL);
	if (!priv->tx_buf || !priv->rx_buf)
		goto err_exit;

	priv->serial_wq = alloc_workqueue("bcm477x_wq",
			WQ_HIGHPRI|WQ_UNBOUND|WQ_MEM_RECLAIM, 1);
	if (!priv->serial_wq) {
		dev_err(&spi->dev, "Failed to allocate workqueue\n");
		goto err_exit;
	}
	/* Init - pinctrl */
	error = gps_initialize_pinctrl(priv);

	/* Register misc device */
	priv->misc.minor = MISC_DYNAMIC_MINOR;
	priv->misc.name = "ttyBCM";
	priv->misc.fops = &bcm_spi_fops;

	ret = misc_register(&priv->misc);
	if (ret) {
		dev_err(&spi->dev, "Failed to register bcm_gps_spi's misc dev. err=%d\n", ret);
		goto free_wq;
	}

	/* Set driver data */
	spi_set_drvdata(spi, priv);

	/* Register ssrdump platform */
	priv->sscd_dev = (struct platform_device){
		.name            = DEVICE_NAME,
		.driver_override = SSCD_NAME,
		.id              = -1,
		.dev             = {
			.platform_data = &priv->sscd_pdata,
			.release       = sscd_release,
			},
	};

	platform_device_register(&priv->sscd_dev);

	if (device_create_file(&spi->dev, &dev_attr_coredump))
		dev_err(&spi->dev, "Unable to create sysfs coredump entry");

	/* Init - miscdev stuff */
	init_waitqueue_head(&priv->poll_wait);
	priv->read_buf.buf = priv->_read_buf;
	priv->write_buf.buf = priv->_write_buf;
	mutex_init(&priv->rlock);
	mutex_init(&priv->wlock);
	priv->busy = false;

	/* Init - work */
	INIT_WORK((struct work_struct *)&priv->rxtx_work, bcm_rxtx_work_func);

	/* Init - irq stuff */
	spin_lock_init(&priv->irq_lock);
	atomic_set(&priv->irq_enabled, 0);
	atomic_set(&priv->suspending, 0);

	/* Init - gpios */
	priv->host_req = host_req;
	priv->mcu_req  = mcu_req;
	priv->mcu_resp = mcu_resp;
	priv->nstandby = nstandby;

	/* Init BBD & SSP */
	priv->bbd = bbd_init(&spi->dev, legacy_patch);
	if (priv->bbd == NULL)
		goto free_wq;

	if (device_create_file(&priv->spi->dev, &dev_attr_nstandby))
		dev_err(&spi->dev, "Unable to create sysfs 4775 nstandby entry");

	if (device_create_file(&priv->spi->dev, &dev_attr_sspmcureq))
		dev_err(&spi->dev, "Unable to create sysfs 4775 sspmcureq entry");

	/* Request IRQ */
	ret = devm_request_irq(&spi->dev, spi->irq, bcm_irq_handler,
			IRQF_TRIGGER_HIGH | IRQF_NO_AUTOEN, "ttyBCM", priv);
	if (ret) {
		dev_err(&spi->dev, "Failed to register BCM477x SPI TTY IRQ %d.\n",
				spi->irq);
		goto free_wq;
	}

	dev_info(&spi->dev, "Probe OK. ssp-host-req=%d, irq=%d, priv=0x%pK\n",
			host_req, spi->irq, priv);

	return 0;

free_wq:
	if (priv->serial_wq)
		destroy_workqueue(priv->serial_wq);
err_exit:
	return -ENODEV;
}


static void bcm_spi_remove(struct spi_device *spi)
{
	struct bcm_spi_priv *priv = spi_get_drvdata(spi);
	unsigned long flags;

	atomic_set(&priv->suspending, 1);

	/* Disable irq */
	spin_lock_irqsave(&priv->irq_lock, flags);
	if (atomic_xchg(&priv->irq_enabled, 0))
		disable_irq_nosync(spi->irq);

	spin_unlock_irqrestore(&priv->irq_lock, flags);

	/* Flush work */
	flush_workqueue(priv->serial_wq);
	destroy_workqueue(priv->serial_wq);
	if (priv->ts_pinctrl) {
		if (gps_pinctrl_select(priv, false) < 0)
			dev_err(&priv->spi->dev, "Cannot get idle pinctrl state\n");
	}

	/* Free everything */
	bbd_exit(&spi->dev);

	/* Clean ssr dump driver */
	platform_device_unregister(&priv->sscd_dev);

	device_remove_file(&priv->spi->dev, &dev_attr_nstandby);
	device_remove_file(&priv->spi->dev, &dev_attr_sspmcureq);
	device_remove_file(&priv->spi->dev, &dev_attr_coredump);
}

static const struct spi_device_id bcm_spi_id[] = {
	{"ssp-spi", 0},
	{}
};
MODULE_DEVICE_TABLE(spi, bcm_spi_id);

#ifdef CONFIG_OF
static const struct of_device_id match_table[] = {
	{ .compatible = "ssp-spi,bcm4775",},
	{},
};
#endif

static const struct dev_pm_ops bcm_spi_pm_ops = {
	.suspend = bcm_spi_suspend,
	.resume = bcm_spi_resume,
};

static struct spi_driver bcm_spi_driver = {
	.id_table = bcm_spi_id,
	.probe = bcm_spi_probe,
	.remove = bcm_spi_remove,
	.shutdown = bcm_spi_shutdown,
	.driver = {
		.name = "brcm gps spi",
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = match_table,
#endif
		.pm = &bcm_spi_pm_ops,
	},
};


/* Module init/exit */
static int __init bcm_spi_init(void)
{
	return spi_register_driver(&bcm_spi_driver);
}

static void __exit bcm_spi_exit(void)
{
	spi_unregister_driver(&bcm_spi_driver);
}

module_init(bcm_spi_init);
module_exit(bcm_spi_exit);
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("BCM SPI/SSI Driver");
