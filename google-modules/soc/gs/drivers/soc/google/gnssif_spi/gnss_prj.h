/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2022, Samsung Electronics.
 *
 */

#ifndef __GNSS_PRJ_H__
#define __GNSS_PRJ_H__

#include <linux/miscdevice.h>
#include <linux/skbuff.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>

#include "include/gnss.h"

#define GNSS_RDY_TIMEOUT	msecs_to_jiffies(5000)
#define ALIGNMENT_4BYTE		4

struct io_device {
	char *name;
	struct link_device *ld;
	struct gnss_ctl *gc;
	atomic_t opened;
	wait_queue_head_t wq;
	struct miscdevice miscdev;
	struct sk_buff_head sk_rx_q;
	struct wakeup_source *ws;
	long waketime;

	void (*recv_betp)(struct io_device *iod, struct link_device *ld,
				struct sk_buff *skb);
};

struct link_device {
	char *name;
	struct gnss_pdata *pdata;
	struct gnss_ctl *gc;
	struct io_device *iod;
	struct gnss_spi *gnss_if;
	unsigned int spi_rx_size;
	unsigned int max_spi_rx_size;
	unsigned int spi_tx_size;
	struct workqueue_struct *rx_wq;
	struct delayed_work rx_dwork;

	int (*send)(struct link_device *ld, struct io_device *iod, char *buff,
			unsigned int size);
};

struct gnssctl_ops {
	int (*gnss_spi_status)(struct gnss_ctl *gc);
	void (*gnss_send_betp_int)(struct gnss_ctl *gc, int value);
	int (*gnss_betp_int_status)(struct gnss_ctl *gc);
};

struct gnss_ctl {
	struct device *dev;
	char *name;
	struct gnss_pdata *pdata;
	struct gnssctl_ops ops;
	struct io_device *iod;
	struct completion gnss_rdy_cmpl;
	struct gnss_irq irq_gnss2ap_spi;
	struct gnss_gpio gpio_gnss2ap_spi;
	struct gnss_gpio gpio_ap2gnss_spi;
	atomic_t wait_rdy;
	atomic_t tx_in_progress;
	atomic_t rx_in_progress;
};

struct gnss_ctl *create_ctl_device(struct platform_device *pdev);
struct link_device *create_link_device(struct platform_device *pdev);
struct io_device *create_io_device(struct platform_device *pdev,
		struct link_device *ld, struct gnss_ctl *gc, struct gnss_pdata *pdata);

#endif /* end of __GNSS_PRJ_H__ */
