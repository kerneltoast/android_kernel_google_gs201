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
#if IS_ENABLED(CONFIG_BOOT_DEVICE_SPI)
#include "boot_device_spi.h"
#endif

static int bootdump_open(struct inode *inode, struct file *filp)
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

static int bootdump_release(struct inode *inode, struct file *filp)
{
	struct io_device *iod = (struct io_device *)filp->private_data;
	struct modem_shared *msd = iod->msd;
	struct link_device *ld;
	int i;

	if (atomic_dec_and_test(&iod->opened) ||
			!strncmp(current->comm, "cbd", 3)) {
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

static unsigned int bootdump_poll(struct file *filp, struct poll_table_struct *wait)
{
	struct io_device *iod = (struct io_device *)filp->private_data;
	struct modem_ctl *mc;
	struct sk_buff_head *rxq;
	struct link_device *ld;

	if (!iod)
		return POLLERR;

	mc = iod->mc;
	rxq = &iod->sk_rx_q;
	ld = get_current_link(iod);

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
		if (iod->format == IPC_BOOT || ld->is_boot_ch(iod->ch) ||
		    iod->format == IPC_DUMP || ld->is_dump_ch(iod->ch)) {
			if (!skb_queue_empty(rxq))
				return POLLIN | POLLRDNORM;
		}

		mif_err_limited("%s: %s.state == %s\n", iod->name, mc->name, mc_state(mc));

		if (iod->format == IPC_BOOT || ld->is_boot_ch(iod->ch))
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

static long bootdump_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct io_device *iod = (struct io_device *)filp->private_data;
	struct link_device *ld = get_current_link(iod);
	struct modem_ctl *mc = iod->mc;
	enum modem_state p_state;
	struct cpif_version version;
	int ret = 0, value;
#if IS_ENABLED(CONFIG_BOOT_DEVICE_SPI)
	struct mem_link_device *mld;
#endif

	switch (cmd) {
	case IOCTL_POWER_ON:
		if (!mc->ops.power_on) {
			mif_err("%s: power_on is null\n", iod->name);
			return -EINVAL;
		}
		mif_info("%s: IOCTL_POWER_ON\n", iod->name);
		return mc->ops.power_on(mc);

	case IOCTL_POWER_OFF:
		if (!mc->ops.power_off) {
			mif_err("%s: power_off is null\n", iod->name);
			return -EINVAL;
		}
		mif_info("%s: IOCTL_POWER_OFF\n", iod->name);
		return mc->ops.power_off(mc);

	case IOCTL_POWER_RESET:
	{
		void __user *uarg = (void __user *)arg;
		struct boot_mode mode;

		mif_info("%s: IOCTL_POWER_RESET\n", iod->name);
		ret = copy_from_user(&mode, uarg, sizeof(mode));
		if (ret) {
			mif_err("copy_from_user() error:%d\n", ret);
			return ret;
		}

#if IS_ENABLED(CONFIG_CPIF_TP_MONITOR)
		tpmon_init();
#endif

		switch (mode.idx) {
		case CP_BOOT_MODE_NORMAL:
			mif_info("%s: normal boot mode\n", iod->name);
			if (!mc->ops.power_reset) {
				mif_err("%s: power_reset is null\n", iod->name);
				return -EINVAL;
			}
			ret = mc->ops.power_reset(mc);
			break;

		case CP_BOOT_MODE_DUMP:
			mif_info("%s: dump boot mode\n", iod->name);
			if (!mc->ops.power_reset_dump) {
				mif_err("%s: power_reset_dump is null\n", iod->name);
				return -EINVAL;
			}
			ret = mc->ops.power_reset_dump(mc, 0);
			break;

		case CP_BOOT_MODE_SILENT:
			mif_info("%s: silent mode\n", iod->name);
			if (!mc->ops.power_reset_dump) {
				mif_err("%s: power_reset_dump is null\n", iod->name);
				return -EINVAL;
			}
			ret = mc->ops.power_reset_dump(mc, 1);
			break;

		default:
			mif_err("boot_mode is invalid:%d\n", mode.idx);
			return -EINVAL;
		}

		return ret;
	}

	case IOCTL_SILENT_RESET:
		if (!mc->ops.silent_reset) {
			mif_err("%s: silent_reset is null\n", iod->name);
			return -EINVAL;
		}
		mif_info("%s: IOCTL_SILENT_RESET\n", iod->name);
		return mc->ops.silent_reset(mc);

	case IOCTL_REQ_SECURITY:
		if (!ld->security_req) {
			mif_err("%s: security_req is null\n", iod->name);
			return -EINVAL;
		}

		mif_info("%s: IOCTL_REQ_SECURITY\n", iod->name);
		ret = ld->security_req(ld, iod, arg);
		if (ret) {
			mif_err("security_req() error:%d\n", ret);
			return ret;
		}
		return ret;

	case IOCTL_LOAD_CP_IMAGE:
		if (!ld->load_cp_image) {
			mif_err("%s: load_cp_image is null\n", iod->name);
			return -EINVAL;
		}

		mif_debug("%s: IOCTL_LOAD_CP_IMAGE\n", iod->name);
		return ld->load_cp_image(ld, iod, arg);

	case IOCTL_START_CP_BOOTLOADER:
	{
		void __user *uarg = (void __user *)arg;
		struct boot_mode mode;

		mif_info("%s: IOCTL_START_CP_BOOTLOADER\n", iod->name);
		ret = copy_from_user(&mode, uarg, sizeof(mode));
		if (ret) {
			mif_err("copy_from_user() error:%d\n", ret);
			return ret;
		}

		switch (mode.idx) {
		case CP_BOOT_MODE_NORMAL:
			mif_info("%s: normal boot mode\n", iod->name);
			if (!mc->ops.start_normal_boot) {
				mif_err("%s: start_normal_boot is null\n", iod->name);
				return -EINVAL;
			}
			return mc->ops.start_normal_boot(mc);

		case CP_BOOT_MODE_DUMP:
			mif_info("%s: dump boot mode\n", iod->name);
			if (!mc->ops.start_dump_boot) {
				mif_err("%s: start_dump_boot is null\n", iod->name);
				return -EINVAL;
			}
			return mc->ops.start_dump_boot(mc);

		default:
			mif_err("boot_mode is invalid:%d\n", mode.idx);
			return -EINVAL;
		}

		return 0;
	}

	case IOCTL_COMPLETE_NORMAL_BOOTUP:
		if (!mc->ops.complete_normal_boot) {
			mif_err("%s: complete_normal_boot is null\n", iod->name);
			return -EINVAL;
		}

		mif_info("%s: IOCTL_COMPLETE_NORMAL_BOOTUP\n", iod->name);
		return mc->ops.complete_normal_boot(mc);

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

	case IOCTL_TRIGGER_KERNEL_PANIC:
	{
		char *buff = ld->crash_reason.string;
		void __user *user_buff = (void __user *)arg;

		mif_info("%s: IOCTL_TRIGGER_KERNEL_PANIC\n", iod->name);

		strcpy(buff, CP_CRASH_TAG);
		if (arg)
			if (copy_from_user((void *)((unsigned long)buff +
				strlen(CP_CRASH_TAG)), user_buff,
				CP_CRASH_INFO_SIZE - strlen(CP_CRASH_TAG)))
				return -EFAULT;
		mif_info("Crash Reason: %s\n", buff);
		panic("%s", buff);
		return 0;
	}

	case IOCTL_GET_LOG_DUMP:
		mif_info("%s: IOCTL_GET_LOG_DUMP\n", iod->name);

		return cp_get_log_dump(iod, ld, arg);

	case IOCTL_GET_CP_CRASH_REASON:
		if (!ld->get_cp_crash_reason) {
			mif_err("%s: get_cp_crash_reason is null\n", iod->name);
			return -EINVAL;
		}

		mif_info("%s: IOCTL_GET_CP_CRASH_REASON\n", iod->name);
		return ld->get_cp_crash_reason(ld, iod, arg);

	case IOCTL_GET_CPIF_VERSION:
		mif_info("%s: IOCTL_GET_CPIF_VERSION\n", iod->name);

		strncpy(version.string, get_cpif_driver_version(), sizeof(version.string) - 1);
		ret = copy_to_user((void __user *)arg, &version, sizeof(version));
		if (ret) {
			mif_err("copy_to_user() error:%d\n", ret);
			return ret;
		}

		return 0;

	case IOCTL_HANDOVER_BLOCK_INFO:
		if (!ld->handover_block_info) {
			mif_err("%s: handover_block_info is null\n", iod->name);
			return -EINVAL;
		}
		mif_info("%s: IOCTL_HANDOVER_BLOCK_INFO\n", iod->name);
		return ld->handover_block_info(ld, arg);

#if IS_ENABLED(CONFIG_BOOT_DEVICE_SPI)
	case IOCTL_SET_SPI_BOOT_MODE:
		mld = to_mem_link_device(ld);
		if (!mld) {
			mif_err("%s: mld is null\n", iod->name);
			return -EINVAL;
		}
		if (mld->spi_bus_num < 0) {
			mif_err("invalid cpboot_spi_bus_num\n");
			return -ENODEV;
		}
		mld->attrs |= LINK_ATTR_XMIT_BTDLR_SPI;
		mld->attrs &= (~LINK_ATTR_XMIT_BTDLR_PCIE);

		ld->load_cp_image = cpboot_spi_load_cp_image;

		mif_info("%s: IOCTL_SET_SPI_BOOT_MODE\n", iod->name);
		return 0;
#endif

	case IOCTL_GET_OPENED_STATUS:
		mif_debug("%s: IOCTL_GET_OPENED_STATUS\n", iod->name);
		value = atomic_read(&iod->opened);
		ret = copy_to_user((void __user *)arg, &value, sizeof(value));
		if (ret)
			mif_err("IOCTL_GET_OPENED_STATUS error: %d\n", ret);
		return ret;

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

static ssize_t bootdump_write(struct file *filp, const char __user *data,
			  size_t count, loff_t *fpos)
{
	struct io_device *iod = (struct io_device *)filp->private_data;
	struct link_device *ld = get_current_link(iod);
	struct modem_ctl *mc = iod->mc;
	struct sk_buff *skb;
	char *buff;
	int ret;
	u8 cfg = 0;
	u16 cfg_sit = 0;
	unsigned int headroom;
	unsigned int tailroom;
	unsigned int tx_bytes;
	unsigned int copied = 0, tot_frame = 0, copied_frm = 0;
	unsigned int remains;
	unsigned int alloc_size;
	/* 64bit prevent */
	unsigned int cnt = (unsigned int)count;
	struct timespec64 ts;

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
	} else {
		cfg = 0;
		cfg_sit = 0;
		headroom = 0;
	}

	while (copied < cnt) {
		remains = cnt - copied;

		if (check_add_overflow(remains, headroom, &alloc_size))
			alloc_size = SZ_2K;

		if (iod->max_tx_size)
			alloc_size = min_t(unsigned int, alloc_size,
				iod->max_tx_size);

		/* Calculate tailroom for padding size */
		if (iod->link_header && ld->aligned)
			tailroom = ld->calc_padding_size(alloc_size);
		else
			tailroom = 0;

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

static ssize_t bootdump_read(struct file *filp, char *buf, size_t count,
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


const struct file_operations bootdump_io_fops = {
	.owner = THIS_MODULE,
	.open = bootdump_open,
	.release = bootdump_release,
	.poll = bootdump_poll,
	.unlocked_ioctl = bootdump_ioctl,
	.compat_ioctl = bootdump_ioctl,
	.write = bootdump_write,
	.read = bootdump_read,
};

const struct file_operations *get_bootdump_io_fops(void)
{
	return &bootdump_io_fops;
}

