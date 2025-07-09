// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * EXYNOS DIT(Direct IP Translator) Driver support
 *
 */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/ip.h>
#include <linux/if_arp.h>
#include <linux/netdevice.h>

#include "modem_utils.h"
#include "dit.h"
#include "dit_net.h"

int dit_net_receive_skb(struct sk_buff *skb)
{
	return 0;
}

static int dit_net_open(struct net_device *dev)
{
	netif_start_queue(dev);

	return 0;
}

static int dit_net_stop(struct net_device *dev)
{
	netif_stop_queue(dev);

	return 0;
}

static const struct net_device_ops dit_net_ops = {
	.ndo_open = dit_net_open,
	.ndo_stop = dit_net_stop,
	// .ndo_start_xmit = vnet_xmit,
};

static void dit_net_setup(struct net_device *dev)
{
	dev->netdev_ops = &dit_net_ops;
	dev->header_ops = 0;
	dev->type = ARPHRD_RAWIP;
	dev->flags = 0;
	dev->addr_len = 0;
	dev->hard_header_len = 0;
	dev->tx_queue_len = 1000;
	dev->mtu = MIF_BUFF_DEFAULT_CELL_SIZE;
	dev->watchdog_timeo = 1000;
}

int dit_net_init(struct dit_ctrl_t *dc)
{
	struct net_device *dev;
	struct dit_net_priv *priv;
	int ret;

	if (dc->netdev)
		return 0;

	dev = alloc_netdev(sizeof(struct dit_net_priv),
		   DIT_NET_DEV_NAME,
		   NET_NAME_UNKNOWN,
		   dit_net_setup);
	if (!dev) {
		mif_err("dit net dev alloc failed\n");
		return -ENOMEM;
	}

	ret = register_netdev(dev);
	if (ret) {
		mif_err("unable to register dit netdev rc=%d\n", ret);
		return ret;
	}

	dc->netdev = dev;
	priv = netdev_priv(dev);
	priv->dc = dc;

	netif_napi_add(dc->netdev, &dc->napi, dit_read_rx_dst_poll);
	napi_enable(&dc->napi);

	return 0;
}
EXPORT_SYMBOL(dit_net_init);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Samsung DIT Net Driver");

