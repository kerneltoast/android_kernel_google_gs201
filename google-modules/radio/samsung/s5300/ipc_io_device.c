// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019 Samsung Electronics.
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/if_arp.h>
#include <linux/ip.h>
#include <linux/if_ether.h>
#include <linux/etherdevice.h>
#include <linux/device.h>
#include <linux/module.h>
#include <trace/events/napi.h>
#include <net/ip.h>
#include <linux/ip.h>
#include <linux/tcp.h>
#include <linux/netdevice.h>

#include "modem_prj.h"
#include "modem_utils.h"
#include "modem_dump.h"

static int ipc_open(struct inode *inode, struct file *filp)
{
	struct io_device *iod = to_io_device(inode->i_cdev);
	struct modem_shared *msd = iod->msd;
	struct link_device *ld;
	int ret;

	filp->private_data = (void *)iod;

	atomic_inc(&iod->opened);

	list_for_each_entry(ld, &msd->link_dev_list, list) {
		if (IS_CONNECTED(iod, ld) && ld->init_comm) {
			ret = ld->init_comm(ld, iod);
			if (ret < 0) {
				mif_err("%s<->%s: ERR! init_comm fail(%d)\n",
					iod->name, ld->name, ret);
				atomic_dec(&iod->opened);
				return ret;
			}
		}
	}

	mif_info("%s (opened %d) by %s\n",
		iod->name, atomic_read(&iod->opened), current->comm);

	return 0;
}

static int ipc_release(struct inode *inode, struct file *filp)
{
	struct io_device *iod = (struct io_device *)filp->private_data;
	struct modem_shared *msd = iod->msd;
	struct link_device *ld;
	int i;

	if (atomic_dec_and_test(&iod->opened)) {
		skb_queue_purge(&iod->sk_rx_q);

		/* purge multi_frame queue */
		for (i = 0; i < NUM_SIPC_MULTI_FRAME_IDS; i++)
			skb_queue_purge(&iod->sk_multi_q[i]);
	}

	list_for_each_entry(ld, &msd->link_dev_list, list) {
		if (IS_CONNECTED(iod, ld) && ld->terminate_comm)
			ld->terminate_comm(ld, iod);
	}

	mif_info("%s (opened %d) by %s\n",
		iod->name, atomic_read(&iod->opened), current->comm);

	return 0;
}

static unsigned int ipc_poll(struct file *filp, struct poll_table_struct *wait)
{
	struct io_device *iod = (struct io_device *)filp->private_data;
	struct modem_ctl *mc;
	struct sk_buff_head *rxq;

	if (!iod)
		return POLLERR;

	mc = iod->mc;
	rxq = &iod->sk_rx_q;

	if (skb_queue_empty(rxq))
		poll_wait(filp, &iod->wq, wait);

	switch (mc->phone_state) {
	case STATE_BOOTING:
	case STATE_ONLINE:
		if (!skb_queue_empty(rxq))
			return POLLIN | POLLRDNORM;
		break;
	case STATE_CRASH_EXIT:
	case STATE_CRASH_RESET:
	case STATE_NV_REBUILDING:
	case STATE_CRASH_WATCHDOG:
		mif_err_limited("%s: %s.state == %s\n", iod->name, mc->name, mc_state(mc));

		if (iod->format == IPC_FMT)
			return POLLHUP;
		break;
	case STATE_RESET:
		mif_err_limited("%s: %s.state == %s\n", iod->name, mc->name, mc_state(mc));

		if (iod->attrs & IO_ATTR_STATE_RESET_NOTI)
			return POLLHUP;
		break;
	default:
		break;
	}

	return 0;
}

static long ipc_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct io_device *iod = (struct io_device *)filp->private_data;
	struct link_device *ld = get_current_link(iod);
	struct modem_ctl *mc = iod->mc;
	enum modem_state p_state;
	int ret = 0, value;

	switch (cmd) {
	case IOCTL_GET_CP_STATUS:
		mif_debug("%s: IOCTL_GET_CP_STATUS\n", iod->name);

		p_state = mc->phone_state;

		if (p_state != STATE_ONLINE) {
			mif_debug("%s: IOCTL_GET_CP_STATUS (state %s)\n",
				iod->name, cp_state_str(p_state));
		}

		switch (p_state) {
		case STATE_NV_REBUILDING:
			mc->phone_state = STATE_ONLINE;
			break;
		/* Do not return an internal state */
		case STATE_RESET:
			p_state = STATE_OFFLINE;
			break;
		default:
			break;
		}

		return p_state;

	case IOCTL_TRIGGER_CP_CRASH:
	{
		char *buff = ld->crash_reason.string;
		void __user *user_buff = (void __user *)arg;

		switch (ld->protocol) {
		case PROTOCOL_SIPC:
			if (arg)
				ld->crash_reason.type = (u32)arg;
			mif_err("%s: IOCTL_TRIGGER_CP_CRASH (%lu)\n",
					iod->name, arg);
			break;

		case PROTOCOL_SIT:
			ld->crash_reason.type =
				CRASH_REASON_RIL_TRIGGER_CP_CRASH;

			if (arg) {
				if (copy_from_user(buff, user_buff, CP_CRASH_INFO_SIZE))
					mif_info("No argument from USER\n");
			} else
				mif_info("No argument from USER\n");

			mif_info("Crash Reason:%s\n", buff);
			break;

		default:
			mif_err("ERR - unknown protocol\n");
			break;
		}

		if (!mc->ops.trigger_cp_crash) {
			mif_err("%s: trigger_cp_crash is null\n", iod->name);
			return -EINVAL;
		}

		return mc->ops.trigger_cp_crash(mc);
	}

	case IOCTL_GET_OPENED_STATUS:
		mif_debug("%s: IOCTL_GET_OPENED_STATUS\n", iod->name);
		value = atomic_read(&iod->opened);
		ret = copy_to_user((void __user *)arg, &value, sizeof(value));
		if (ret)
			mif_err("IOCTL_GET_OPENED_STATUS error: %d\n", ret);
		return ret;

	case IOCTL_LOAD_GNSS_IMAGE:
		if (!ld->load_gnss_image) {
			mif_err("%s: load_gnss_image is null\n", iod->name);
			return -EINVAL;
		}

		mif_info("%s: IOCTL_LOAD_GNSS_IMAGE\n", iod->name);
		return ld->load_gnss_image(ld, iod, arg);

	case IOCTL_READ_GNSS_IMAGE:
		if (!ld->read_gnss_image) {
			mif_err("%s: read_gnss_image is null\n", iod->name);
			return -EINVAL;
		}

		mif_info("%s: IOCTL_READ_GNSS_IMAGE\n", iod->name);
		return ld->read_gnss_image(ld, iod, arg);

	default:
		 /* If you need to handle the ioctl for specific link device,
		  * then assign the link ioctl handler to ld->ioctl
		  * It will be call for specific link ioctl
		  */
		if (ld->ioctl)
			return ld->ioctl(ld, iod, cmd, arg);

		mif_info("%s: ERR! undefined cmd 0x%X\n", iod->name, cmd);
		return -EINVAL;
	}

	return 0;
}

#define INIT_END_WAIT_MS	150

static ssize_t ipc_write(struct file *filp, const char __user *data,
			  size_t count, loff_t *fpos)
{
	struct io_device *iod = (struct io_device *)filp->private_data;
	struct link_device *ld = get_current_link(iod);
	struct mem_link_device *mld = to_mem_link_device(ld);
	struct modem_ctl *mc = iod->mc;
	u8 cfg = 0;
	u16 cfg_sit = 0;
	unsigned int headroom = 0;
	unsigned int copied = 0, tot_frame = 0, copied_frm = 0;
	/* 64bit prevent */
	unsigned int cnt = (unsigned int)count;
	struct timespec64 ts;
	int curr_init_end_cnt;
	int retry = 0;

	/* Record the timestamp */
	ktime_get_ts64(&ts);

	if (iod->format <= IPC_RFS && iod->ch == 0)
		return -EINVAL;

	if (unlikely(!cp_online(mc)) && ld->is_ipc_ch(iod->ch)) {
		mif_debug("%s: ERR! %s->state == %s\n",
			iod->name, mc->name, mc_state(mc));
		return -EPERM;
	}

	if (iod->link_header) {
		switch (ld->protocol) {
		case PROTOCOL_SIPC:
			cfg = sipc5_build_config(iod, ld, cnt);
			headroom = sipc5_get_hdr_len(&cfg);
			break;
		case PROTOCOL_SIT:
			cfg_sit = exynos_build_fr_config(iod, ld, cnt);
			headroom = EXYNOS_HEADER_SIZE;
			break;
		default:
			mif_err("protocol error %d\n", ld->protocol);
			return -EINVAL;
		}
	}

	/* Wait for a while if a new CMD_INIT_END is sent */
	while ((curr_init_end_cnt = atomic_read(&mld->init_end_cnt)) != mld->last_init_end_cnt &&
	       retry++ < 3) {
		mif_info_limited("%s: wait for INIT_END done (%dms) cnt:%d last:%d cmd:0x%02X\n",
				 iod->name, INIT_END_WAIT_MS,
				 curr_init_end_cnt, mld->last_init_end_cnt,
				 mld->read_ap2cp_irq(mld));

		if (atomic_inc_return(&mld->init_end_busy) > 1)
			curr_init_end_cnt = -1;

		msleep(INIT_END_WAIT_MS);
		if (curr_init_end_cnt >= 0)
			mld->last_init_end_cnt = curr_init_end_cnt;

		atomic_dec(&mld->init_end_busy);
	}

	if (unlikely(!mld->last_init_end_cnt)) {
		mif_err_limited("%s: INIT_END is not done\n", iod->name);
		return -EAGAIN;
	}

	while (copied < cnt) {
		struct sk_buff *skb;
		char *buff;
		unsigned int remains = cnt - copied;
		unsigned int tailroom = 0;
		unsigned int tx_bytes;
		unsigned int alloc_size;
		int ret;

		if (check_add_overflow(remains, headroom, &alloc_size))
			alloc_size = SZ_2K;

		switch (ld->protocol) {
		case PROTOCOL_SIPC:
			if (iod->max_tx_size)
				alloc_size = min_t(unsigned int, alloc_size,
					iod->max_tx_size);
			break;
		case PROTOCOL_SIT:
			alloc_size = min_t(unsigned int, alloc_size, SZ_2K);
			break;
		default:
			mif_err("protocol error %d\n", ld->protocol);
			return -EINVAL;
		}

		/* Calculate tailroom for padding size */
		if (iod->link_header && ld->aligned)
			tailroom = ld->calc_padding_size(alloc_size);

		alloc_size += tailroom;

		skb = alloc_skb(alloc_size, GFP_KERNEL);
		if (!skb) {
			mif_info("%s: ERR! alloc_skb fail (alloc_size:%d)\n",
				iod->name, alloc_size);
			return -ENOMEM;
		}

		tx_bytes = alloc_size - headroom - tailroom;

		/* Reserve the space for a link header */
		skb_reserve(skb, headroom);

		/* Copy an IPC message from the user space to the skb */
		buff = skb_put(skb, tx_bytes);
		if (copy_from_user(buff, data + copied, tx_bytes)) {
			mif_err("%s->%s: ERR! copy_from_user fail(count %lu)\n",
				iod->name, ld->name, (unsigned long)count);
			dev_kfree_skb_any(skb);
			return -EFAULT;
		}

		/* Update size of copied payload */
		copied += tx_bytes;
		/* Update size of total frame included hdr, pad size */
		tot_frame += alloc_size;

		/* Store the IO device, the link device, etc. */
		skbpriv(skb)->iod = iod;
		skbpriv(skb)->ld = ld;

		skbpriv(skb)->lnk_hdr = iod->link_header;
		skbpriv(skb)->sipc_ch = iod->ch;

		/* Copy the timestamp to the skb */
		skbpriv(skb)->ts = ts;
#ifdef DEBUG_MODEM_IF_IODEV_TX
		mif_pkt(iod->ch, "IOD-TX", skb);
#endif

		/* Build SIPC5 link header*/
		if (cfg || cfg_sit) {
			buff = skb_push(skb, headroom);

			switch (ld->protocol) {
			case PROTOCOL_SIPC:
				sipc5_build_header(iod, buff, cfg,
					tx_bytes, cnt - copied);
				break;
			case PROTOCOL_SIT:
				exynos_build_header(iod, ld, buff, cfg_sit, 0, tx_bytes);
				/* modify next link header for multiframe */
				if (((cfg_sit >> 8) & EXYNOS_SINGLE_MASK) != EXYNOS_SINGLE_MASK)
					cfg_sit = modify_next_frame(cfg_sit);
				break;
			default:
				mif_err("protocol error %d\n", ld->protocol);
				return -EINVAL;
			}
		}

		/* Apply padding */
		if (tailroom)
			skb_put(skb, tailroom);

		/**
		 * Send the skb with a link device
		 */
		ret = ld->send(ld, iod, skb);
		if (ret < 0) {
			mif_err("%s->%s: %s->send fail(%d, tx:%d len:%lu)\n",
				iod->name, mc->name, ld->name,
				ret, tx_bytes, (unsigned long)count);
			dev_kfree_skb_any(skb);
			return ret;
		}
		copied_frm += ret;
	}

	if (copied_frm != tot_frame) {
		mif_info("%s->%s: WARN! %s->send ret:%d (len:%lu)\n",
			iod->name, mc->name, ld->name,
			copied_frm, (unsigned long)count);
	}

	return count;
}

static ssize_t ipc_read(struct file *filp, char *buf, size_t count,
			loff_t *fpos)
{
	struct io_device *iod = (struct io_device *)filp->private_data;
	struct sk_buff_head *rxq = &iod->sk_rx_q;
	struct sk_buff *skb;
	int copied;

	if (skb_queue_empty(rxq)) {
		long tmo = msecs_to_jiffies(100);

		wait_event_timeout(iod->wq, !skb_queue_empty(rxq), tmo);
	}

	skb = skb_dequeue(rxq);
	if (unlikely(!skb)) {
		mif_info("%s: NO data in RXQ\n", iod->name);
		return 0;
	}

	copied = skb->len > count ? count : skb->len;

	if (copy_to_user(buf, skb->data, copied)) {
		mif_err("%s: ERR! copy_to_user fail\n", iod->name);
		dev_kfree_skb_any(skb);
		return -EFAULT;
	}

	if (iod->ch == SIPC_CH_ID_CPLOG1) {
		struct net_device *ndev = iod->ndev;

		if (!ndev) {
			mif_err("%s: ERR! no iod->ndev\n", iod->name);
		} else {
			ndev->stats.rx_packets++;
			ndev->stats.rx_bytes += copied;
		}
	}

#ifdef DEBUG_MODEM_IF_IODEV_RX
	mif_pkt(iod->ch, "IOD-RX", skb);
#endif
	mif_debug("%s: data:%d copied:%d qlen:%d\n",
		iod->name, skb->len, copied, rxq->qlen);

	if (skb->len > copied) {
		skb_pull(skb, copied);
		skb_queue_head(rxq, skb);
	} else {
		dev_consume_skb_any(skb);
	}

	return copied;
}


const struct file_operations ipc_io_fops = {
	.owner = THIS_MODULE,
	.open = ipc_open,
	.release = ipc_release,
	.poll = ipc_poll,
	.unlocked_ioctl = ipc_ioctl,
	.compat_ioctl = ipc_ioctl,
	.write = ipc_write,
	.read = ipc_read,
};

const struct file_operations *get_ipc_io_fops(void)
{
	return &ipc_io_fops;
}

