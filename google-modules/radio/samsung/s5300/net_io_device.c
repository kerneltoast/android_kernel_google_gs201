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
#include <net/tcp.h>

#include "exynos-modem-ctrl.h"

#include "modem_prj.h"
#include "modem_utils.h"
#include "modem_dump.h"
#if IS_ENABLED(CONFIG_MODEM_IF_LEGACY_QOS)
#include "cpif_qos_info.h"
#endif

static int vnet_open(struct net_device *ndev)
{
	struct vnet *vnet = netdev_priv(ndev);
	struct io_device *iod = (struct io_device *)vnet->iod;
	struct modem_shared *msd = iod->msd;
	struct link_device *ld;
	int ret;

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
	list_add(&iod->node_ndev, &iod->msd->activated_ndev_list);

	netif_start_queue(ndev);

	mif_info("%s (opened %d) by %s\n",
		iod->name, atomic_read(&iod->opened), current->comm);

	return 0;
}

static int vnet_stop(struct net_device *ndev)
{
	struct vnet *vnet = netdev_priv(ndev);
	struct io_device *iod = (struct io_device *)vnet->iod;
	struct modem_shared *msd = iod->msd;
	struct link_device *ld;

	if (atomic_dec_and_test(&iod->opened))
		skb_queue_purge(&iod->sk_rx_q);

	list_for_each_entry(ld, &msd->link_dev_list, list) {
		if (IS_CONNECTED(iod, ld) && ld->terminate_comm)
			ld->terminate_comm(ld, iod);
	}

	spin_lock(&msd->active_list_lock);
	list_del(&iod->node_ndev);
	spin_unlock(&msd->active_list_lock);
	netif_stop_queue(ndev);

	mif_info("%s (opened %d) by %s\n",
		iod->name, atomic_read(&iod->opened), current->comm);

	return 0;
}

static netdev_tx_t vnet_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct vnet *vnet = netdev_priv(ndev);
	struct io_device *iod = (struct io_device *)vnet->iod;
	struct link_device *ld = get_current_link(iod);
	struct modem_ctl *mc = iod->mc;
	unsigned int count = skb->len;
	struct sk_buff *skb_new = skb;
	char *buff;
	int ret;
	u8 __maybe_unused cfg;
	u16 __maybe_unused cfg_sit;
	unsigned int headroom;
	unsigned int tailroom;
	unsigned int tx_bytes;
	struct timespec64 ts;

	if (unlikely(!cp_online(mc))) {
		if (!netif_queue_stopped(ndev))
			netif_stop_queue(ndev);
		/* Just drop the TX packet */
		goto drop;
	}

	/* Record the timestamp */
	ktime_get_ts64(&ts);

#if IS_ENABLED(CONFIG_CP_PKTPROC_UL)
	/* no need of head and tail */
	cfg = 0;
	cfg_sit = 0;
	headroom = 0;
	tailroom = 0;
#else
	if (iod->link_header) {
		switch (ld->protocol) {
		case PROTOCOL_SIPC:
			cfg = sipc5_build_config(iod, ld, count);
			headroom = sipc5_get_hdr_len(&cfg);
		break;
		case PROTOCOL_SIT:
			cfg_sit = exynos_build_fr_config(iod, ld, count);
			headroom = EXYNOS_HEADER_SIZE;
		break;
		default:
			mif_err("protocol error %d\n", ld->protocol);
			return -EINVAL;
		}
		if (ld->aligned)
			tailroom = ld->calc_padding_size(headroom + count);
		else
			tailroom = 0;
	} else {
		cfg = 0;
		cfg_sit = 0;
		headroom = 0;
		tailroom = 0;
	}

	if ((skb_headroom(skb) < headroom) || (skb_tailroom(skb) < tailroom)) {
		skb_new = skb_copy_expand(skb, headroom, tailroom, GFP_ATOMIC);
		if (!skb_new) {
			mif_info("%s: ERR! skb_copy_expand fail\n", iod->name);
			goto retry;
		}
	}
#endif

	tx_bytes = headroom + count + tailroom;

	/* Store the IO device, the link device, etc. */
	skbpriv(skb_new)->iod = iod;
	skbpriv(skb_new)->ld = ld;

	skbpriv(skb_new)->lnk_hdr = iod->link_header;
	skbpriv(skb_new)->sipc_ch = iod->ch;

	/* Copy the timestamp to the skb */
	skbpriv(skb_new)->ts = ts;
#if defined(DEBUG_MODEM_IF_IODEV_TX) && defined(DEBUG_MODEM_IF_PS_DATA)
	mif_pkt(iod->ch, "IOD-TX", skb_new);
#endif

	/* Build SIPC5 link header*/
#if IS_ENABLED(CONFIG_CP_PKTPROC_UL)
	buff = skb_new->data;
#else
	buff = skb_push(skb_new, headroom);
#endif

#if !IS_ENABLED(CONFIG_CP_PKTPROC_UL)
	if (cfg || cfg_sit) {
		switch (ld->protocol) {
		case PROTOCOL_SIPC:
			sipc5_build_header(iod, buff, cfg, count, 0);
		break;
		case PROTOCOL_SIT:
			exynos_build_header(iod, ld, buff, cfg_sit, 0, count);
		break;
		default:
			mif_err("protocol error %d\n", ld->protocol);
			return -EINVAL;
		}
	}
#endif

	/* IP loop-back */
	if (iod->msd->loopback_ipaddr) {
		struct iphdr *ip_header = (struct iphdr *)skb->data;

		if (ip_header->daddr == iod->msd->loopback_ipaddr) {
			swap(ip_header->saddr, ip_header->daddr);
			buff[SIPC5_CH_ID_OFFSET] = DATA_LOOPBACK_CHANNEL;
		}
	}

#if !IS_ENABLED(CONFIG_CP_PKTPROC_UL)
	/* Apply padding */
	if (tailroom)
		skb_put(skb_new, tailroom);
#endif

	ret = ld->send(ld, iod, skb_new);
	if (unlikely(ret < 0)) {
		if ((ret != -EBUSY) && (ret != -ENOSPC)) {
			mif_err_limited("%s->%s: ERR! %s->send fail:%d (tx_bytes:%d len:%d)\n",
				iod->name, mc->name, ld->name, ret,
				tx_bytes, count);
			goto drop;
		}

		goto retry;
	}

	if (ret != tx_bytes) {
		mif_info("%s->%s: WARN! %s->send ret:%d (tx_bytes:%d len:%d)\n",
			iod->name, mc->name, ld->name, ret, tx_bytes, count);
	}

	ndev->stats.tx_packets++;
	ndev->stats.tx_bytes += count;

	/*
	 * If @skb has been expanded to $skb_new, @skb must be freed here.
	 * ($skb_new will be freed by the link device.)
	 */
	if (skb_new != skb)
		dev_consume_skb_any(skb);

	return NETDEV_TX_OK;

retry:
#if !IS_ENABLED(CONFIG_CP_PKTPROC_UL)
	if (iod->link_header && skb_new && (skb_new == skb)) {
		if (headroom)
			skb_pull(skb_new, headroom);

		if (tailroom)
			skb_trim(skb_new, count);
	}
#endif

	/*
	 * If @skb has been expanded to $skb_new, only $skb_new must be freed here
	 * because @skb will be reused by NET_TX.
	 */
	if (skb_new && skb_new != skb)
		dev_consume_skb_any(skb_new);

	return NETDEV_TX_BUSY;

drop:
	ndev->stats.tx_dropped++;

	dev_kfree_skb_any(skb);

	/*
	 * If @skb has been expanded to $skb_new, $skb_new must also be freed here.
	 */
	if (skb_new != skb)
		dev_consume_skb_any(skb_new);

	return NETDEV_TX_OK;
}

static bool _is_tcp_ack(struct sk_buff *skb)
{
	u16 payload_len = 0;

	switch (skb->protocol) {
	case htons(ETH_P_IP):
		if (ip_hdr(skb)->protocol != IPPROTO_TCP)
			return false;

		if (skb->network_header == skb->transport_header)
			skb->transport_header += (ip_hdr(skb)->ihl << 2);
		payload_len = ntohs(ip_hdr(skb)->tot_len) - (ip_hdr(skb)->ihl << 2);
		break;
	case htons(ETH_P_IPV6):
		if (ipv6_hdr(skb)->nexthdr != IPPROTO_TCP)
			return false;

		if (skb->network_header == skb->transport_header)
			skb->transport_header += sizeof(struct ipv6hdr);
		payload_len = ntohs(ipv6_hdr(skb)->payload_len);
		break;
	default:
		break;
	}

	if (!payload_len)
		return false;

	if (payload_len == (tcp_hdr(skb)->doff << 2) &&
	    (tcp_flag_word(tcp_hdr(skb)) & cpu_to_be32(0x00FF0000)) == TCP_FLAG_ACK)
		return true;

	return false;
}

static inline bool is_tcp_ack(struct sk_buff *skb)
{
	if (skb_is_tcp_pure_ack(skb))
		return true;

	if (unlikely(_is_tcp_ack(skb)))
		return true;

	return false;
}

#if IS_ENABLED(CONFIG_MODEM_IF_QOS)
static u16 vnet_select_queue(struct net_device *dev, struct sk_buff *skb,
		struct net_device *sb_dev)
{
#if IS_ENABLED(CONFIG_MODEM_IF_LEGACY_QOS)
	struct vnet *vnet = netdev_priv(dev);
#endif

	if (!skb)
		return 0;

	if (is_tcp_ack(skb))
		return 1;

#if IS_ENABLED(CONFIG_MODEM_IF_LEGACY_QOS)
	if (!vnet->hiprio_ack_only && skb->sk && cpif_qos_get_node(skb->sk->sk_uid.val))
		return 1;
#endif

	return 0;
}
#endif

static const struct net_device_ops vnet_ops = {
	.ndo_open = vnet_open,
	.ndo_stop = vnet_stop,
	.ndo_start_xmit = vnet_xmit,
#if IS_ENABLED(CONFIG_MODEM_IF_QOS)
	.ndo_select_queue = vnet_select_queue,
#endif
};

void vnet_setup(struct net_device *ndev)
{
	ndev->netdev_ops = &vnet_ops;
	ndev->type = ARPHRD_RAWIP;
	ndev->flags = IFF_POINTOPOINT | IFF_NOARP | IFF_MULTICAST;
	ndev->addr_len = 0;
	ndev->hard_header_len = 0;
	ndev->tx_queue_len = 1000;
	ndev->mtu = ETH_DATA_LEN;
	ndev->watchdog_timeo = 5 * HZ;
	ndev->features |= (NETIF_F_GRO | NETIF_F_GRO_UDP_FWD);
}
