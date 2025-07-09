// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2022 Samsung Electronics.
 *
 */

#include <linux/init.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/if_arp.h>
#include <linux/ip.h>
#include <linux/if_ether.h>
#include <linux/etherdevice.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/skbuff.h>
#include <linux/slab.h>

#include "gnss_prj.h"
#include "gnss_utils.h"

#define WAKE_TIME   (HZ/2) /* 500 msec */
#define MAX_RXQ_DEPTH	1024

static inline void iodev_lock_wlock(struct io_device *iod)
{
	if (iod->waketime > 0 && !gnssif_wlock_active(iod->ws)) {
		gnssif_wlock_unlock(iod->ws);
		gnssif_wlock_lock_timeout(iod->ws, iod->waketime);
	}
}

static void io_dev_recv_betp(struct io_device *iod,
		struct link_device *ld, struct sk_buff *skb)
{
	struct sk_buff_head *rxq = &iod->sk_rx_q;

	iodev_lock_wlock(iod);

	if (unlikely(rxq->qlen >= MAX_RXQ_DEPTH)) {
		gif_err_limited("Too much rx betps are queued: total %d\n",
				MAX_RXQ_DEPTH);
		dev_kfree_skb_any(skb);
		goto wakeup;
	}
	skb_queue_tail(rxq, skb);

wakeup:
	wake_up(&iod->wq);

}

static int misc_open(struct inode *inode, struct file *filp)
{
	struct io_device *iod = container_of(filp->private_data, struct io_device,
			miscdev);
	struct link_device *ld;
	int ref_cnt;

	filp->private_data = (void *)iod;

	ld = iod->ld;

	ref_cnt = atomic_inc_return(&iod->opened);

	gif_info("%s (opened %d)\n", iod->name, ref_cnt);

	return 0;
}

static int misc_release(struct inode *inode, struct file *filp)
{
	struct io_device *iod = (struct io_device *)filp->private_data;
	int ref_cnt;

	ref_cnt = atomic_dec_return(&iod->opened);
	if (ref_cnt == 0)
		skb_queue_purge(&iod->sk_rx_q);

	gif_info("%s (opened %d)\n", iod->name, ref_cnt);

	return 0;
}

static unsigned int misc_poll(struct file *filp, struct poll_table_struct *wait)
{
	struct io_device *iod = (struct io_device *)filp->private_data;

	if (skb_queue_empty(&iod->sk_rx_q))
		poll_wait(filp, &iod->wq, wait);

	if (!skb_queue_empty(&iod->sk_rx_q))
		return POLLIN | POLLRDNORM;

	return 0;
}

static long misc_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct io_device *iod = (struct io_device *)filp->private_data;

	gif_err("%s: ERR! undefined cmd 0x%X\n", iod->name, cmd);

	return -EINVAL;
}

static ssize_t misc_write(struct file *filp, const char __user *data,
			size_t count, loff_t *fpos)
{
	struct io_device *iod = (struct io_device *)filp->private_data;
	struct link_device *ld = iod->ld;
	char *buff;
	int ret;

	/* Store IPC message */
	buff = kzalloc(roundup(count, ALIGNMENT_4BYTE), GFP_KERNEL);
	if (!buff)
		return -ENOMEM;

	if (copy_from_user(buff, data, count)) {
		gif_err("%s->%s: ERR! copy_from_user fail (count %ld)\n",
			iod->name, ld->name, count);
		kfree(buff);
		return -EFAULT;
	}

	ret = ld->send(ld, iod, buff, roundup(count, ALIGNMENT_4BYTE));
	if (ret < 0) {
		gif_err("%s->%s: ERR! ld->send fail (err %d, count %ld)\n",
			iod->name, ld->name, ret, count);
		kfree(buff);
		return ret;
	}

	kfree(buff);
	return count;
}

static ssize_t misc_read(struct file *filp, char *buf, size_t count,
			loff_t *fpos)
{
	struct io_device *iod = (struct io_device *)filp->private_data;
	struct sk_buff_head *rxq = &iod->sk_rx_q;
	struct sk_buff *skb;
	int copied = 0;

	if (skb_queue_empty(rxq)) {
		gif_err("%s: ERR! no data in rxq\n", iod->name);
		return 0;
	}

	skb = skb_dequeue(rxq);
	if (unlikely(!skb)) {
		gif_err("%s: No data in RXQ\n", iod->name);
		return 0;
	}

	if (count <= 0) {
		gif_err("%s: bytes to read not specified\n", iod->name);
		return 0;
	}

	copied = skb->len > count ? count : skb->len;

	if (copy_to_user(buf, skb->data, copied)) {
		gif_err("%s: ERR! copy_to_user fail\n", iod->name);
		dev_kfree_skb_any(skb);
		return -EFAULT;
	}

	gif_debug("%s: data:%d copied:%d qlen:%d\n",
			iod->name, skb->len, copied, rxq->qlen);

	if (skb->len > count) {
		skb_pull(skb, count);
		skb_queue_head(rxq, skb);
	} else {
		dev_kfree_skb_any(skb);
	}

	return copied;
}

static const struct file_operations misc_io_fops = {
	.owner = THIS_MODULE,
	.open = misc_open,
	.release = misc_release,
	.poll = misc_poll,
	.unlocked_ioctl = misc_ioctl,
#if IS_ENABLED(CONFIG_COMPAT)
	.compat_ioctl = misc_ioctl,
#endif
	.write = misc_write,
	.read = misc_read,
};

int init_io_device(struct io_device *iod, struct device *dev)
{
	int ret = 0;

	gif_info("+++\n");

	iod->recv_betp = io_dev_recv_betp;

	init_waitqueue_head(&iod->wq);
	skb_queue_head_init(&iod->sk_rx_q);

	iod->miscdev.minor = MISC_DYNAMIC_MINOR;
	iod->miscdev.name = iod->name;
	iod->miscdev.fops = &misc_io_fops;
	iod->waketime = WAKE_TIME;
	iod->ws = gnssif_wlock_register(dev, iod->name);
	if (iod->ws == NULL) {
		gif_err("%s: wakeup_source_register fail\n", iod->name);
		return -EINVAL;
	}

	ret = misc_register(&iod->miscdev);
	if (ret)
		gif_err("%s: ERR! misc_register failed\n", iod->name);

	gif_info("---\n");

	return ret;
}

struct io_device *create_io_device(struct platform_device *pdev,
		struct link_device *ld, struct gnss_ctl *gc, struct gnss_pdata *pdata)
{
	int ret;
	struct device *dev = &pdev->dev;
	struct io_device *iod;

	gif_info("+++\n");
	iod = devm_kzalloc(dev, sizeof(struct io_device), GFP_KERNEL);
	if (!iod) {
		gif_err("iod is NULL\n");
		return NULL;
	}

	iod->name = pdata->node_name;
	atomic_set(&iod->opened, 0);

	iod->gc = gc;
	gc->iod = iod;

	iod->ld = ld;
	ld->iod = iod;

	ret = init_io_device(iod, dev);
	if (ret) {
		devm_kfree(dev, iod);
		gif_err("init_io_device fail (%d)\n", ret);
		return NULL;
	}

	gif_info("---\n");

	return iod;
}
