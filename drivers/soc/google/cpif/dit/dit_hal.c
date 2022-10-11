// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * EXYNOS DIT(Direct IP Translator) Driver support
 *
 */

#include <linux/poll.h>

#include "modem_utils.h"
#include "dit_hal.h"

#define DIT_HAL_STATS_MAX	(S64_MAX)

static struct dit_ctrl_t *dc;
static struct dit_hal_ctrl_t *dhc;

static int dit_hal_open(struct inode *inode, struct file *filp)
{
	return 0;
}

static int dit_hal_release(struct inode *inode, struct file *file)
{
	return 0;
}

static unsigned int dit_hal_poll(struct file *filp, struct poll_table_struct *wait)
{
	poll_wait(filp, &dhc->wq, wait);

	if (!list_empty(&dhc->event_q))
		return POLLIN | POLLRDNORM;

	return 0;
}

static ssize_t dit_hal_read(struct file *filp, char *buf, size_t count, loff_t *fpos)
{
	struct offload_event event;
	struct offload_event_item *event_item;
	unsigned long flags;

	if (count < sizeof(event)) {
		mif_err("not support small buffer size: %zu\n", count);
		return 0;
	}

	spin_lock_irqsave(&dhc->event_lock, flags);
	if (list_empty(&dhc->event_q)) {
		mif_err("no event\n");
		spin_unlock_irqrestore(&dhc->event_lock, flags);
		return 0;
	}

	event_item = list_first_entry(&dhc->event_q, struct offload_event_item, list);
	list_del(&event_item->list);
	spin_unlock_irqrestore(&dhc->event_lock, flags);

	event.event_num = event_item->event_num;
	kvfree(event_item);

	if (copy_to_user((void __user *)buf, (void *)&event, sizeof(event)))
		return 0;

	mif_info("event=%d\n", event.event_num);

	return sizeof(event);
}

static int dit_hal_init(void)
{
	int i;
	struct offload_event_item *event_item;
	unsigned long flags;

	if (!dc->hal_support) {
		mif_err("does not support hal\n");
		return -EPERM;
	}

	dhc->last_event_num = OFFLOAD_MAX;

	spin_lock_irqsave(&dhc->stats_lock, flags);
	dhc->stats.data_warning = DIT_HAL_STATS_MAX;
	dhc->stats.data_limit = DIT_HAL_STATS_MAX;
	dhc->stats.rx_bytes = 0;
	dhc->stats.tx_bytes = 0;
	dhc->stats.rx_diff = 0;
	dhc->stats.tx_diff = 0;
	spin_unlock_irqrestore(&dhc->stats_lock, flags);

	for (i = 0; i < DIT_DST_DESC_RING_MAX; i++)
		dhc->dst_iface[i].iface_set = false;

	spin_lock_irqsave(&dhc->event_lock, flags);
	while (!list_empty(&dhc->event_q)) {
		event_item = list_first_entry(&dhc->event_q, struct offload_event_item, list);
		list_del(&event_item->list);
		kvfree(event_item);
	}
	spin_unlock_irqrestore(&dhc->event_lock, flags);

	return 0;
}

static int dit_hal_set_event(enum offload_event_num event_num)
{
	struct offload_event_item *event_item;
	unsigned long flags;

	if (event_num == dhc->last_event_num)
		return -EEXIST;

	event_item = kvzalloc(sizeof(struct offload_event_item), GFP_ATOMIC);
	if (!event_item) {
		mif_err("event=%d generation failed\n", event_num);
		return -ENOMEM;
	}

	event_item->event_num = event_num;
	spin_lock_irqsave(&dhc->event_lock, flags);
	list_add_tail(&event_item->list, &dhc->event_q);
	spin_unlock_irqrestore(&dhc->event_lock, flags);

	dhc->last_event_num = event_num;
	wake_up(&dhc->wq);

	return 0;
}

struct net_device *dit_hal_get_dst_netdev(enum dit_desc_ring ring_num)
{
	if (ring_num < DIT_DST_DESC_RING_0 || ring_num >= DIT_DST_DESC_RING_MAX)
		return NULL;

#if defined(DIT_DEBUG_LOW)
	if (dc->pktgen_ch && (ring_num == DIT_DST_DESC_RING_0)) {
		struct io_device *iod;

		iod = link_get_iod_with_channel(dc->ld, dc->pktgen_ch);

		return iod ? iod->ndev : NULL;
	}
#endif

	if (dhc && dhc->dst_iface[ring_num].iface_set)
		return dhc->dst_iface[ring_num].netdev;

	return NULL;
}
EXPORT_SYMBOL(dit_hal_get_dst_netdev);

static bool dit_hal_check_data_warning_reached(void)
{
	unsigned long flags;
	bool ret = false;

	if (!dhc)
		return false;

	spin_lock_irqsave(&dhc->stats_lock, flags);
	if ((dhc->stats.rx_bytes + dhc->stats.tx_bytes) >= dhc->stats.data_warning) {
		dhc->stats.data_warning = DIT_HAL_STATS_MAX;
		ret = true;
	}
	spin_unlock_irqrestore(&dhc->stats_lock, flags);

	return ret;
}

static bool dit_hal_check_data_limit_reached(void)
{
	unsigned long flags;
	bool ret = false;

	if (!dhc)
		return false;

	spin_lock_irqsave(&dhc->stats_lock, flags);
	if ((dhc->stats.rx_bytes + dhc->stats.tx_bytes) >= dhc->stats.data_limit)
		ret = true;
	spin_unlock_irqrestore(&dhc->stats_lock, flags);

	return ret;
}

void dit_hal_add_data_bytes(u64 rx_bytes, u64 tx_bytes)
{
	unsigned long flags;

	if (!dhc)
		return;

	spin_lock_irqsave(&dhc->stats_lock, flags);
	dhc->stats.rx_bytes += rx_bytes;
	dhc->stats.tx_bytes += tx_bytes;
	dhc->stats.rx_diff += rx_bytes;
	dhc->stats.tx_diff += tx_bytes;
	spin_unlock_irqrestore(&dhc->stats_lock, flags);

	if (dit_hal_check_data_warning_reached())
		dit_hal_set_event(OFFLOAD_WARNING_REACHED);

	if (dit_hal_check_data_limit_reached())
		dit_hal_set_event(OFFLOAD_STOPPED_LIMIT_REACHED);
}
EXPORT_SYMBOL(dit_hal_add_data_bytes);

static bool dit_hal_get_forwarded_stats(struct forward_stats *stats)
{
	unsigned long flags;
	bool ret = false;

	spin_lock_irqsave(&dhc->stats_lock, flags);
	if (strncmp(stats->iface, dhc->stats.iface, IFNAMSIZ))
		goto exit;

	stats->rx_bytes = dhc->stats.rx_bytes;
	stats->tx_bytes = dhc->stats.tx_bytes;
	stats->rx_diff = dhc->stats.rx_diff;
	stats->tx_diff = dhc->stats.tx_diff;

	ret = true;

exit:
	dhc->stats.rx_diff = 0;
	dhc->stats.tx_diff = 0;
	spin_unlock_irqrestore(&dhc->stats_lock, flags);
	return ret;
}

/* struct forward_limit is for V1.1 */
static bool dit_hal_set_data_limit(struct forward_stats *stats,
				   struct forward_limit *limit)
{
	unsigned long flags;

	if (!stats && !limit)
		return false;

	spin_lock_irqsave(&dhc->stats_lock, flags);
	if (stats) {
		strlcpy(dhc->stats.iface, stats->iface, IFNAMSIZ);
		dhc->stats.data_warning = DIT_HAL_STATS_MAX;
		dhc->stats.data_limit = stats->data_limit;
	} else {
		strlcpy(dhc->stats.iface, limit->iface, IFNAMSIZ);
		dhc->stats.data_warning = limit->data_warning;
		dhc->stats.data_limit = limit->data_limit;
	}
	dhc->stats.rx_bytes = 0;
	dhc->stats.tx_bytes = 0;
	spin_unlock_irqrestore(&dhc->stats_lock, flags);

	return true;
}

static bool dit_hal_check_ready_to_start(void)
{
	int i;

	spin_lock(&dhc->hal_lock);
	if (!dhc->hal_enabled) {
		spin_unlock(&dhc->hal_lock);
		return false;
	}
	spin_unlock(&dhc->hal_lock);

	if (dit_hal_check_data_limit_reached())
		return false;

	if (!dhc->dst_iface[DIT_DST_DESC_RING_0].iface_set)
		return false;

	for (i = DIT_DST_DESC_RING_1; i < DIT_DST_DESC_RING_MAX; i++) {
		if (dhc->dst_iface[i].iface_set)
			return true;
	}

	return false;
}

static int dit_hal_add_dst_iface(bool is_upstream,
		struct iface_info *info)
{
	enum dit_desc_ring dst_min;
	enum dit_desc_ring dst_max;
	int i;

	if (is_upstream) {
		dst_min = DIT_DST_DESC_RING_0;
		dst_max = DIT_DST_DESC_RING_1;

		/* set upstream always */
		dhc->dst_iface[dst_min].iface_set = false;
	} else {
		dst_min = DIT_DST_DESC_RING_1;
		dst_max = DIT_DST_DESC_RING_MAX;

		/* check duplication */
		for (i = dst_min; i < dst_max; i++) {
			if (strncmp(info->iface, dhc->dst_iface[i].iface, IFNAMSIZ))
				continue;

			dhc->dst_iface[i].netdev =
				dev_get_by_name(&init_net, info->iface);
			if (dhc->dst_iface[i].netdev)
				dhc->dst_iface[i].iface_set = true;

			info->dst_ring = i;
			return i;
		}
	}

	if (strlen(info->iface) == 0)
		return -1;

	/* find empty space */
	for (i = dst_min; i < dst_max; i++) {
		if (dhc->dst_iface[i].iface_set)
			continue;

		strlcpy(dhc->dst_iface[i].iface, info->iface, IFNAMSIZ);
		dhc->dst_iface[i].netdev =
			dev_get_by_name(&init_net, info->iface);
		if (dhc->dst_iface[i].netdev)
			dhc->dst_iface[i].iface_set = true;

		info->dst_ring = i;
		return i;
	}

	return -1;
}

static void dit_hal_remove_dst_iface(bool is_upstream,
		struct iface_info *info)
{
	enum dit_desc_ring dst_min;
	enum dit_desc_ring dst_max;
	int i;

	if (is_upstream) {
		dhc->dst_iface[DIT_DST_DESC_RING_0].iface_set = false;
		return;
	}

	dst_min = DIT_DST_DESC_RING_1;
	dst_max = DIT_DST_DESC_RING_MAX;

	for (i = dst_min; i < dst_max; i++) {
		if (!dhc->dst_iface[i].iface_set ||
				strncmp(info->iface, dhc->dst_iface[i].iface, IFNAMSIZ))
			continue;

		dhc->dst_iface[i].iface_set = false;
		dev_put(dhc->dst_iface[i].netdev);
		break;
	}
}

static bool dit_hal_set_local_addr(struct nat_local_addr *local_addr)
{
	struct net_device *netdev;
	struct dev_addr_map *devaddr;
	unsigned long flags;
	int ret = false;

	spin_lock_irqsave(&dc->src_lock, flags);
	/* local IP addr */
	if (dit_enqueue_reg_value_with_ext_lock(local_addr->addr,
		DIT_REG_NAT_LOCAL_ADDR +
		(local_addr->index * DIT_REG_NAT_LOCAL_INTERVAL)) < 0)
		goto exit;

	/* addr can be 0 when remove */
	if (!local_addr->addr) {
		ret = true;
		goto exit;
	}

	/* DST dev addr */
	if (dit_enqueue_reg_value_with_ext_lock(local_addr->dev_addr_l,
		DIT_REG_NAT_ETHERNET_DST_MAC_ADDR_0 +
		(local_addr->index * DIT_REG_ETHERNET_MAC_INTERVAL)) < 0)
		goto exit;
	if (dit_enqueue_reg_value_with_ext_lock((u32) local_addr->dev_addr_h,
		DIT_REG_NAT_ETHERNET_DST_MAC_ADDR_1 +
		(local_addr->index * DIT_REG_ETHERNET_MAC_INTERVAL)) < 0)
		goto exit;

	/* SRC dev addr */
	netdev = dit_hal_get_dst_netdev(local_addr->dst_ring);
	if (!netdev) {
		mif_err("failed to get local dev addr for 0x%08X\n",
			ntohl(local_addr->addr));
		goto exit;
	}

	devaddr = (struct dev_addr_map *) netdev->dev_addr;
	if (dit_enqueue_reg_value_with_ext_lock(devaddr->dev_addr_l,
		DIT_REG_NAT_ETHERNET_SRC_MAC_ADDR_0 +
		(local_addr->index * DIT_REG_ETHERNET_MAC_INTERVAL)) < 0)
		goto exit;
	if (dit_enqueue_reg_value_with_ext_lock((u32) devaddr->dev_addr_h,
		DIT_REG_NAT_ETHERNET_SRC_MAC_ADDR_1 +
		(local_addr->index * DIT_REG_ETHERNET_MAC_INTERVAL)) < 0)
		goto exit;

	/* IPv4 only */
	if (dit_enqueue_reg_value_with_ext_lock((u32) htons(ETH_P_IP),
		DIT_REG_NAT_ETHERNET_TYPE +
		(local_addr->index * DIT_REG_ETHERNET_MAC_INTERVAL)) < 0)
		goto exit;

	ret = true;

exit:
	spin_unlock_irqrestore(&dc->src_lock, flags);
	return ret;
}

static bool dit_hal_set_local_port(struct nat_local_port *local_port)
{
	spin_lock(&dhc->hal_lock);
	if (!dhc->hal_enabled) {
		spin_unlock(&dhc->hal_lock);
		mif_err("hal is not enabled\n");
		return false;
	}

	dit_enqueue_reg_value(local_port->hw_val,
		DIT_REG_NAT_RX_PORT_TABLE_SLOT +
		(local_port->reply_port_dst_l * DIT_REG_NAT_LOCAL_INTERVAL));
	spin_unlock(&dhc->hal_lock);

	return true;
}

bool dit_hal_set_clat_info(struct mem_link_device *mld, struct clat_info *clat)
{
	unsigned int offset;
	unsigned long flags;
	bool ret = false;

	if (!dc->use_clat || !dc->ld)
		return false;

	spin_lock_irqsave(&dc->src_lock, flags);
	/* IPv4 addr of TUN device */
	offset = clat->clat_index * DIT_REG_CLAT_TX_FILTER_INTERVAL;
	if (dit_enqueue_reg_value_with_ext_lock(clat->ipv4_local_subnet.s_addr,
			DIT_REG_CLAT_TX_FILTER + offset) < 0)
		goto exit;

	/* IPv6 addr for TUN device */
	offset = clat->clat_index * DIT_REG_CLAT_TX_CLAT_SRC_INTERVAL;
	if (dit_enqueue_reg_value_with_ext_lock(clat->ipv6_local_subnet.s6_addr32[0],
			DIT_REG_CLAT_TX_CLAT_SRC_0 + offset) < 0)
		goto exit;
	if (dit_enqueue_reg_value_with_ext_lock(clat->ipv6_local_subnet.s6_addr32[1],
			DIT_REG_CLAT_TX_CLAT_SRC_1 + offset) < 0)
		goto exit;
	if (dit_enqueue_reg_value_with_ext_lock(clat->ipv6_local_subnet.s6_addr32[2],
			DIT_REG_CLAT_TX_CLAT_SRC_2 + offset) < 0)
		goto exit;
	if (dit_enqueue_reg_value_with_ext_lock(clat->ipv6_local_subnet.s6_addr32[3],
			DIT_REG_CLAT_TX_CLAT_SRC_3 + offset) < 0)
		goto exit;

	/* PLAT prefix */
	offset = clat->clat_index * DIT_REG_CLAT_TX_PLAT_PREFIX_INTERVAL;
	if (dit_enqueue_reg_value_with_ext_lock(clat->plat_subnet.s6_addr32[0],
			DIT_REG_CLAT_TX_PLAT_PREFIX_0 + offset) < 0)
		goto exit;
	if (dit_enqueue_reg_value_with_ext_lock(clat->plat_subnet.s6_addr32[1],
			DIT_REG_CLAT_TX_PLAT_PREFIX_1 + offset) < 0)
		goto exit;
	if (dit_enqueue_reg_value_with_ext_lock(clat->plat_subnet.s6_addr32[2],
			DIT_REG_CLAT_TX_PLAT_PREFIX_2 + offset) < 0)
		goto exit;

	ret = true;

	/* set clat_ndev with clat registers */
	if (clat->ipv4_iface[0])
		iodevs_for_each(dc->ld->msd, mld->tc->set_iod_clat_netdev, clat);

exit:
	spin_unlock_irqrestore(&dc->src_lock, flags);

	/* clear clat_ndev but take a delay to prevent null ndev */
	if (ret && !clat->ipv4_iface[0]) {
		msleep(100);
		iodevs_for_each(dc->ld->msd, mld->tc->set_iod_clat_netdev, clat);
	}

	return ret;
}
EXPORT_SYMBOL(dit_hal_set_clat_info);

static void dit_hal_try_stop_enqueue_rx(bool stop)
{
	if (unlikely(!dc))
		return;

	if (dc->hal_enqueue_rx)
		dc->stop_enqueue[DIT_DIR_RX] = stop;
}

static void dit_hal_set_reg_upstream(void)
{
	struct net_device *netdev = NULL;

	if (unlikely(!dc))
		return;

	if (dhc->dst_iface[DIT_DST_DESC_RING_0].iface_set)
		netdev = dhc->dst_iface[DIT_DST_DESC_RING_0].netdev;

	DIT_INDIRECT_CALL(dc, set_reg_upstream, netdev);
}

static long dit_hal_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct iface_info info;
	struct forward_stats stats;
	struct forward_limit limit;
	struct nat_local_addr local_addr;
	struct nat_local_port local_port;
	struct hw_info hw;
	int ret;

	if (unlikely(!dc) || unlikely(!dc->ld))
		return -EPERM;

	if (!dhc)
		return -EPERM;

	switch (cmd) {
	case OFFLOAD_IOCTL_INIT_OFFLOAD:
		mif_info("hal init\n");

		ret = dit_hal_init();
		if (ret) {
			mif_err("hal init failed. ret: %d\n", ret);
			return ret;
		}
		mutex_lock(&dhc->ioctl_lock);
		ret = dit_manage_rx_dst_data_buffers(true);
		mutex_unlock(&dhc->ioctl_lock);
		if (ret) {
			mif_err("hal buffer fill failed. ret: %d\n", ret);
			return ret;
		}

		dit_hal_try_stop_enqueue_rx(false);
		spin_lock(&dhc->hal_lock);
		dhc->hal_enabled = true;
		spin_unlock(&dhc->hal_lock);
		break;
	case OFFLOAD_IOCTL_STOP_OFFLOAD:
		mif_info("hal stopped\n");

		spin_lock(&dhc->hal_lock);
		dhc->hal_enabled = false;
		spin_unlock(&dhc->hal_lock);
		dit_hal_try_stop_enqueue_rx(true);

		dit_hal_set_event(INTERNAL_OFFLOAD_STOPPED);

		/* init port table and take a delay for the prior kick */
		dit_init(NULL, DIT_INIT_NORMAL, DIT_STORE_NONE);
		msleep(100);
		mutex_lock(&dhc->ioctl_lock);
		ret = dit_manage_rx_dst_data_buffers(false);
		mutex_unlock(&dhc->ioctl_lock);
		if (ret)
			mif_err("hal buffer free. ret: %d\n", ret);

		/* don't call dit_hal_init() here for the last event delivery */
		break;
	case OFFLOAD_IOCTL_GET_FORWD_STATS:
		if (copy_from_user(&stats, (const void __user *)arg,
				sizeof(struct forward_stats)))
			return -EFAULT;

		if (!dit_hal_get_forwarded_stats(&stats))
			return -EINVAL;

		if (copy_to_user((void __user *)arg, (void *)&stats,
				sizeof(struct forward_stats)))
			return -EFAULT;
		break;
	case OFFLOAD_IOCTL_SET_DATA_LIMIT:
		if (copy_from_user(&stats, (const void __user *)arg,
				sizeof(struct forward_stats)))
			return -EFAULT;

		if (!dit_hal_set_data_limit(&stats, NULL))
			return -EINVAL;
		break;
	case OFFLOAD_IOCTL_SET_DATA_WARNING_LIMIT:
		if (copy_from_user(&limit, (const void __user *)arg,
				   sizeof(struct forward_limit)))
			return -EFAULT;

		if (!dit_hal_set_data_limit(NULL, &limit))
			return -EINVAL;
		break;
	case OFFLOAD_IOCTL_SET_UPSTRM_PARAM:
		if (copy_from_user(&info, (const void __user *)arg,
				sizeof(struct iface_info)))
			return -EFAULT;

		/* hal can remove upstream by null iface name */
		ret = dit_hal_add_dst_iface(true, &info);
		dit_hal_set_reg_upstream();
		if (ret < 0) {
			dit_hal_set_event(OFFLOAD_STOPPED_ERROR);
			break;
		}

		if (dit_hal_check_ready_to_start())
			dit_hal_set_event(OFFLOAD_STARTED);
		break;
	case OFFLOAD_IOCTL_ADD_DOWNSTREAM:
		if (copy_from_user(&info, (const void __user *)arg,
				sizeof(struct iface_info)))
			return -EFAULT;

		if (dit_hal_add_dst_iface(false, &info) < 0)
			return -ENOSPC;
		if (dit_hal_check_ready_to_start())
			dit_hal_set_event(OFFLOAD_STARTED);

		if (copy_to_user((void __user *)arg, (void *)&info,
				sizeof(struct iface_info)))
			return -EFAULT;
		break;
	case OFFLOAD_IOCTL_REMOVE_DOWNSTRM:
		if (copy_from_user(&info, (const void __user *)arg,
				sizeof(struct iface_info)))
			return -EFAULT;

		msleep(100);
		dit_hal_remove_dst_iface(false, &info);
		if (!dit_hal_check_ready_to_start())
			dit_hal_set_event(OFFLOAD_STOPPED_ERROR);
		break;
	/* ToDo: need to implement */
	case OFFLOAD_IOCTL_SET_LOCAL_PRFIX:
		break;
	case OFFLOAD_IOCTL_SET_NAT_LOCAL_ADDR:
		if (copy_from_user(&local_addr, (const void __user *)arg,
				sizeof(struct nat_local_addr)))
			return -EFAULT;

		if (!dit_hal_set_local_addr(&local_addr))
			return -EINVAL;
		break;
	case OFFLOAD_IOCTL_SET_NAT_LOCAL_PORT:
		if (copy_from_user(&local_port, (const void __user *)arg,
				sizeof(struct nat_local_port)))
			return -EFAULT;

		if (!dit_hal_set_local_port(&local_port))
			return -EINVAL;
		break;
	case OFFLOAD_IOCTL_GET_HW_INFO:
		hw.version = dc->hw_version;
		hw.capabilities = dc->hw_capabilities;
		if (copy_to_user((void __user *)arg, (void *)&hw,
				sizeof(struct hw_info)))
			return -EFAULT;
		break;
	default:
		mif_err("unknown command: 0x%X\n", cmd);
		return -EINVAL;
	}

	return 0;
}

static const struct file_operations dit_hal_fops = {
	.owner		= THIS_MODULE,
	.open		= dit_hal_open,
	.poll		= dit_hal_poll,
	.read		= dit_hal_read,
	.release	= dit_hal_release,
	.compat_ioctl	= dit_hal_ioctl,
	.unlocked_ioctl	= dit_hal_ioctl,
};

static struct miscdevice dit_misc = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= DIT_HAL_DEV_NAME,
	.fops	= &dit_hal_fops,
};

int dit_hal_create(struct dit_ctrl_t *dc_ptr)
{
	int ret = 0;

	if (!dc_ptr) {
		mif_err("dc not valid\n");
		ret = -EINVAL;
		goto error;
	}
	dc = dc_ptr;

	dhc = devm_kzalloc(dc->dev, sizeof(struct dit_hal_ctrl_t), GFP_KERNEL);
	if (!dhc) {
		mif_err("dit hal ctrl alloc failed\n");
		ret = -ENOMEM;
		goto error;
	}

	init_waitqueue_head(&dhc->wq);
	INIT_LIST_HEAD(&dhc->event_q);
	spin_lock_init(&dhc->hal_lock);
	spin_lock_init(&dhc->event_lock);
	spin_lock_init(&dhc->stats_lock);
	mutex_init(&dhc->ioctl_lock);

	ret = misc_register(&dit_misc);
	if (ret) {
		mif_err("misc register error\n");
		goto error;
	}

	return 0;

error:
	if (dhc && dc) {
		devm_kfree(dc->dev, dhc);
		dhc = NULL;
	}

	return ret;
}
EXPORT_SYMBOL(dit_hal_create);
