// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2022 Samsung Electronics.
 *
 */

#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/skbuff.h>
#include <linux/delay.h>
#include <linux/netdevice.h>

#include "include/gnss.h"
#include "gnss_prj.h"
#include "gnss_utils.h"
#include "gnss_spi.h"

/* #define GNSS_PKT_DEBUG	1 */

#if defined(GNSS_PKT_DEBUG)
#define PR_BUFFER_SIZE	64

static inline void dump2hex(char *buff, size_t buff_size,
			    const char *data, size_t data_len)
{
	static const char *hex = "0123456789abcdef";
	char *dest = buff;
	size_t len;
	size_t i;

	if (buff_size < (data_len * 3))
		len = buff_size / 3;
	else
		len = data_len;

	for (i = 0; i < len; i++) {
		*dest++ = hex[(data[i] >> 4) & 0xf];
		*dest++ = hex[data[i] & 0xf];
		*dest++ = ' ';
	}

	/* The last character must be overwritten with NULL */
	if (likely(len > 0))
		dest--;

	*dest = 0;
}

static int pr_buffer(const char *tag, const char *data, size_t data_len,
							size_t max_len)
{
	size_t len = min(data_len, max_len);
	unsigned char str[PR_BUFFER_SIZE * 3]; /* 1 <= sizeof <= max_len*3 */

	if (len > PR_BUFFER_SIZE)
		len = PR_BUFFER_SIZE;

	dump2hex(str, (len ? len * 3 : 1), data, len);

	/* don't change this printk to mif_debug for print this as level7 */
	return pr_info("%s: %s(%ld): %s%s\n", "gnss_spi", tag, (long)data_len,
			str, (len == data_len) ? "" : " ...");
}
#else	/* GNSS_PKT_DEBUG */
static inline int pr_buffer(const char *tag, const char *data, size_t data_len,
							size_t max_len)
{
	return 0;
}
#endif	/* GNSS_PKT_DEBUG */

static void rx_work(struct work_struct *ws)
{
	struct link_device *ld = container_of(ws, struct link_device,
						rx_dwork.work);
	struct gnss_ctl *gc = ld->gc;
	struct io_device *iod = ld->iod;
	struct sk_buff *skb = NULL;
	char *buff = NULL;
	unsigned int len = ld->spi_rx_size;
	unsigned int max_len = ld->max_spi_rx_size;
	unsigned int iter = 0;
	int ret = 0;

	atomic_set(&gc->rx_in_progress, 1);

	do {
		if (!skb) {
			gc->ops.gnss_send_betp_int(gc, 1);
			if (atomic_read(&gc->tx_in_progress) == 0) {
				udelay(100);
				gc->ops.gnss_send_betp_int(gc, 0);
			}

			skb = alloc_skb(max_len, GFP_KERNEL);
			if (!skb) {
				gif_err("%s: ERR! rx alloc_skb fail (msg size:%u)\n",
						iod->name, len);
				goto exit;
			}
		}

		buff = skb_put(skb, len);
		ret = gnss_spi_recv(buff, len);
		if (ret) {
			gif_err("gnss_spi_recv() error:%d\n", ret);
			dev_kfree_skb_any(skb);
			goto exit;
		}

		iter++;
		if (skb->len >= max_len || atomic_read(&gc->wait_rdy) == 1) {
			pr_buffer("RX", skb->data, skb->len, skb->len);
			gif_debug("Received %d(%d * %d) bytes data from kepler.\n",
					skb->len, len, iter);

			iod->recv_betp(iod, ld, skb);
			skb = NULL;	/* skb to be freed by gnss iod*/
			iter = 0;

			if (atomic_read(&gc->wait_rdy) == 1) {
				atomic_set(&gc->wait_rdy, 0);
				complete_all(&gc->gnss_rdy_cmpl);
				gif_debug("TX request while Rx is in-progress\n");

				while (atomic_read(&gc->tx_in_progress) == 1)
					udelay(10);
			}
		}
	} while (gc->ops.gnss_spi_status(gc));

	if (skb) {
		pr_buffer("RX", skb->data, skb->len, skb->len);
		gif_debug("Received %d(%d * %d) bytes data from kepler.\n",
				skb->len, len, iter);

		iod->recv_betp(iod, ld, skb);
	}
exit:
	atomic_set(&gc->rx_in_progress, 0);
	if (atomic_read(&gc->tx_in_progress) == 0)
		gc->ops.gnss_send_betp_int(gc, 0);

	gif_debug("Done receiving from kepler\n");

	gif_enable_irq(&gc->irq_gnss2ap_spi);
}

static int send_betp(struct link_device *ld, struct io_device *iod, char *buff,
		unsigned int size)
{
	struct gnss_ctl *gc = ld->gc;
	struct sk_buff *skb;
	int ret = 0;

	if (size > ld->spi_tx_size) {
		gif_err("size(%d) is over %d\n", size, ld->spi_tx_size);
		size = ld->spi_tx_size;
	}

	skb = alloc_skb(size, GFP_KERNEL);
	if (!skb) {
		gif_err("%s: ERR! tx alloc_skb fail (msg size:%u)\n",
				iod->name, size);
		return -ENOMEM;
	}
	skb_put(skb, size);

	atomic_set(&gc->tx_in_progress, 1);

	reinit_completion(&gc->gnss_rdy_cmpl);
	atomic_set(&gc->wait_rdy, 1);

	gc->ops.gnss_send_betp_int(gc, 1);

	gif_enable_irq(&gc->irq_gnss2ap_spi);

	ret = wait_for_completion_timeout(&gc->gnss_rdy_cmpl, GNSS_RDY_TIMEOUT);
	if (ret == 0) {
		gif_err("TIMEOUT(%lu): gnss ready signal not came from kepler\n", GNSS_RDY_TIMEOUT);
		dev_kfree_skb_any(skb);
		ret = -EBUSY;
		goto exit;
	}

	ret = gnss_spi_send(buff, size, (char *)skb->data);
	if (ret) {
		gif_err("gnss_spi_send() error:%d\n", ret);
		dev_kfree_skb_any(skb);
		goto exit;
	} else {
		gif_debug("Sent %d bytes to kepler\n", size);
		pr_buffer("TX", buff, size, size);
		iod->recv_betp(iod, ld, skb);
	}

exit:
	atomic_set(&gc->wait_rdy, 0);
	atomic_set(&gc->tx_in_progress, 0);
	gc->ops.gnss_send_betp_int(gc, 0);

	gif_enable_irq(&gc->irq_gnss2ap_spi);

	return ret;
}

struct link_device *create_link_device(struct platform_device *pdev)
{
	struct link_device *ld = NULL;
	struct gnss_pdata *pdata = NULL;
	struct device *dev = &pdev->dev;

	gif_info("+++\n");

	pdata = (struct gnss_pdata *)dev->platform_data;
	if (!pdata) {
		gif_err("ERR! gnss == NULL\n");
		return NULL;
	}

	ld = devm_kzalloc(dev, sizeof(struct link_device), GFP_KERNEL);
	if (!ld) {
		gif_err("ERR! link device kzalloc fail\n");
		return NULL;
	}

	ld->pdata = pdata;
	ld->name = "GNSS_LINK_DEVICE";
	ld->send = send_betp;
	ld->spi_rx_size = DEFAULT_SPI_RX_SIZE;
	ld->max_spi_rx_size = MAX_SPI_RX_SIZE;
	ld->spi_tx_size = DEFAULT_SPI_TX_SIZE;

	ld->rx_wq = alloc_workqueue("gnss_spi_wq",
					__WQ_LEGACY | WQ_MEM_RECLAIM | WQ_UNBOUND, 1);
	if (!ld->rx_wq) {
		gif_err("alloc_workqueue() error\n");
		goto exit;
	}
	INIT_DELAYED_WORK(&ld->rx_dwork, rx_work);

	gif_info("---\n");

exit:
	return ld;
}
