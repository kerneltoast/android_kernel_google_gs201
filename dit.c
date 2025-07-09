// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * EXYNOS DIT(Direct IP Translator) Driver support
 *
 */

#include <linux/ip.h>
#include <linux/ipv6.h>
#include <linux/udp.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <dt-bindings/soc/google/exynos-dit.h>
#if IS_ENABLED(CONFIG_CPU_IDLE)
#include <soc/google/exynos-cpupm.h>
#endif

#include "modem_utils.h"
#include "dit.h"
#include "dit_net.h"
#include "dit_hal.h"

static struct dit_ctrl_t *dc;

static struct dit_snapshot_t snapshot[DIT_DIR_MAX][DIT_DESC_RING_MAX] = {
	{
		{ .name = "tx_int_dst0", .head = -1, .tail = -1 },
		{ .name = "tx_int_dst1", .head = -1, .tail = -1 },
		{ .name = "tx_int_dst2", .head = -1, .tail = -1 },
		{ .name = "tx_kick_src", .head = -1, .tail = -1 },
	},
	{
		{ .name = "rx_int_dst0", .head = -1, .tail = -1 },
		{ .name = "rx_int_dst1", .head = -1, .tail = -1 },
		{ .name = "rx_int_dst2", .head = -1, .tail = -1 },
		{ .name = "rx_kick_src", .head = -1, .tail = -1 },
	},
};

static void dit_set_snapshot(enum dit_direction dir, enum dit_desc_ring ring_num,
		int head, int tail, u64 packets)
{
	if (dir < 0 || dir >= DIT_DIR_MAX)
		return;

	if (ring_num < 0 || ring_num >= DIT_DESC_RING_MAX)
		return;

	if (head >= 0)
		snapshot[dir][ring_num].head = head;
	if (tail >= 0)
		snapshot[dir][ring_num].tail = tail;

	snapshot[dir][ring_num].packets = packets;
	snapshot[dir][ring_num].total_packets += packets;
};

static int dit_get_snapshot_head(enum dit_direction dir, enum dit_desc_ring ring_num)
{
	if (dir < 0 || dir >= DIT_DIR_MAX)
		return 0;

	if (ring_num < 0 || ring_num >= DIT_DESC_RING_MAX)
		return 0;

	if (snapshot[dir][ring_num].head >= 0)
		return snapshot[dir][ring_num].head;

	return 0;
}

static int dit_get_snapshot_tail(enum dit_direction dir, enum dit_desc_ring ring_num)
{
	if (dir < 0 || dir >= DIT_DIR_MAX)
		return 0;

	if (ring_num < 0 || ring_num >= DIT_DESC_RING_MAX)
		return 0;

	if (snapshot[dir][ring_num].tail >= 0)
		return snapshot[dir][ring_num].tail;

	return 0;
}

static int dit_get_snapshot_next_head(enum dit_direction dir,
		enum dit_desc_ring ring_num, unsigned int qlen)
{
	if (dir < 0 || dir >= DIT_DIR_MAX)
		return 0;

	if (ring_num < 0 || ring_num >= DIT_DESC_RING_MAX)
		return 0;

	if (snapshot[dir][ring_num].tail >= 0)
		return circ_new_ptr(qlen, snapshot[dir][ring_num].tail, 1);

	return 0;
}

static bool dit_hw_capa_matched(u32 mask)
{
	if (dc->hw_capabilities & mask)
		return true;

	return false;
}

static void dit_print_dump(enum dit_direction dir, u32 dump_bits)
{
	struct dit_desc_info *desc_info;
	struct dit_src_desc *src_desc = NULL;
	struct dit_dst_desc *dst_desc = NULL;
	struct nat_local_port local_port;
	u16 reply_port_dst, reply_port_dst_h, reply_port_dst_l;
	u16 origin_port_src;
	u16 ring_num;
	u32 i;

	if (dump_bits & BIT(DIT_DUMP_SNAPSHOT_BIT)) {
		mif_err("---- SNAPSHOT[dir:%d] ----\n", dir);
		for (ring_num = 0; ring_num < DIT_DESC_RING_MAX; ring_num++) {
			pr_info("%s head:%d,tail:%d\n", snapshot[dir][ring_num].name,
				snapshot[dir][ring_num].head, snapshot[dir][ring_num].tail);
		}
	}

	if (dump_bits & BIT(DIT_DUMP_DESC_BIT)) {
		desc_info = &dc->desc_info[dir];

		mif_err("---- SRC RING[dir:%d] wp:%u,rp:%u ----\n", dir,
			desc_info->src_wp, desc_info->src_rp);
		src_desc = desc_info->src_desc_ring;
		for (i = 0; i < desc_info->src_desc_ring_len; i++) {
			if (!(src_desc[i].control & DIT_SRC_KICK_CONTROL_MASK))
				continue;
			pr_info("src[%06d] ctrl:0x%02X,stat:0x%02X,ch_id:%02u\n",
				i, src_desc[i].control, src_desc[i].status, src_desc[i].ch_id);
		}

		for (ring_num = DIT_DST_DESC_RING_0; ring_num < DIT_DST_DESC_RING_MAX; ring_num++) {
			dst_desc = desc_info->dst_desc_ring[ring_num];
			mif_err("---- DST RING%d[dir:%d] wp:%u,rp:%u ----\n", ring_num, dir,
				desc_info->dst_wp[ring_num], desc_info->dst_rp[ring_num]);
			for (i = 0; i < desc_info->dst_desc_ring_len; i++) {
				if (!dst_desc[i].control && !dst_desc[i].status)
					continue;
				pr_info("dst[%d][%06d] ctrl:0x%02X,stat:0x%02X,p_info:0x%03X\n",
					ring_num, i, dst_desc[i].control, dst_desc[i].status,
					dst_desc[i].packet_info);
			}
		}
	}

	if ((dump_bits & BIT(DIT_DUMP_PORT_TABLE_BIT)) && (dir == DIT_DIR_RX)) {
		mif_err("---- PORT TABLE[dir:%d] ----\n", dir);
		for (i = 0; i < DIT_REG_NAT_LOCAL_PORT_MAX; i++) {
			local_port.hw_val = READ_REG_VALUE(dc, DIT_REG_NAT_RX_PORT_TABLE_SLOT +
					(i * DIT_REG_NAT_LOCAL_INTERVAL));
			if (!local_port.enable)
				continue;

			reply_port_dst_h = (u16)((local_port.reply_port_dst_h & 0xFF) << 8);
			reply_port_dst_l = (u16)(i & 0x7FF);

			/* read operation could return an invalid value if hw is running */
			if (((reply_port_dst_h >> 8) & 0x7) != (reply_port_dst_l >> 8))
				continue;

			reply_port_dst = (reply_port_dst_h | reply_port_dst_l);
			origin_port_src = local_port.origin_port_src;
			if (dit_hw_capa_matched(DIT_CAP_MASK_PORT_BIG_ENDIAN)) {
				reply_port_dst = htons(reply_port_dst);
				origin_port_src = htons(origin_port_src);
			}

			pr_info("[%04d] en:%d,o_port:%5d,r_port:%5d,addr_idx:%02d,dst:%d,udp:%d\n",
				i, local_port.enable, origin_port_src, reply_port_dst,
				local_port.addr_index, local_port.dst_ring, local_port.is_udp);
		}
	}
}

static bool dit_is_kicked_any(void)
{
	unsigned int dir;

	for (dir = 0; dir < DIT_DIR_MAX; dir++) {
		if (dc->kicked[dir])
			return true;
	}

	return false;
}

static inline int dit_check_ring_space(
		unsigned int qlen, unsigned int wp, unsigned int rp)
{
	unsigned int space;

	if (!circ_valid(qlen, wp, rp)) {
		mif_err_limited("DIRTY (qlen:%d wp:%d rp:%d)\n",
			qlen, wp, rp);
		return -EIO;
	}

	space = circ_get_space(qlen, wp, rp);
	if (unlikely(space < 1)) {
		mif_err_limited("NOSPC (qlen:%d wp:%d rp:%d)\n",
			qlen, wp, rp);
		return -ENOSPC;
	}

	return space;
}

#if defined(DIT_DEBUG_LOW)
static void dit_debug_out_of_order(enum dit_direction dir, enum dit_desc_ring ring,
		u8 *data)
{
	struct modem_ctl *mc;
	struct udphdr *uh;
	unsigned int off;
	unsigned int *seq_p;
	unsigned int seq;
	u16 port;

	static unsigned int last_seq[DIT_DIR_MAX][DIT_DESC_RING_MAX];
	static unsigned int out_count[DIT_DIR_MAX][DIT_DESC_RING_MAX];
	static u16 target_port[DIT_DIR_MAX][DIT_DESC_RING_MAX];

	if (!dc->pktgen_ch)
		return;

	switch (data[0] & 0xF0) {
	case 0x40:
		off = sizeof(struct iphdr);
		break;
	case 0x60:
		off = sizeof(struct ipv6hdr);
		break;
	default:
		return;
	}

	uh = (struct udphdr *)(data + off);
	off += sizeof(struct udphdr);

	switch (dir) {
	case DIT_DIR_TX:
		port = uh->source;
		break;
	case DIT_DIR_RX:
		port = uh->dest;
		break;
	default:
		return;
	}

	if (!target_port[dir][ring]) {
		mif_info("check dir[%d] out of order at ring[%d] for port:%u\n", dir, ring,
			ntohs(port));
		/* ntohs() is not needed */
		target_port[dir][ring] = port;
	}

	/* check the first detected port only */
	if (port != target_port[dir][ring])
		return;

	seq_p = (unsigned int *)&data[off];
	seq = ntohl(*seq_p);

	if (seq < last_seq[dir][ring]) {
		mif_err("dir[%d] out of order at ring[%d] seq:0x%08x last:0x%08x\n", dir, ring,
			seq, last_seq[dir][ring]);
		if (++out_count[dir][ring] > 5) {
			dit_print_dump(dir, DIT_DUMP_ALL);
			if ((dc->ld) && (dc->ld->mc)) {
				mc = dc->ld->mc;
				mc->ops.trigger_cp_crash(mc);
			}
		}
	}
	last_seq[dir][ring] = seq;
}
#endif

static int dit_check_dst_ready(enum dit_direction dir, enum dit_desc_ring ring_num)
{
	struct dit_desc_info *desc_info;

	if (!dc)
		return -EPERM;

	if (ring_num < DIT_DST_DESC_RING_0 || ring_num >= DIT_DST_DESC_RING_MAX)
		return -EINVAL;

	/* DST0 is always ready */
	if (ring_num == DIT_DST_DESC_RING_0)
		return 0;

	switch (dir) {
	case DIT_DIR_TX:
		desc_info = &dc->desc_info[dir];
		if (!desc_info->dst_desc_ring[ring_num])
			return -ENODEV;
		break;
	case DIT_DIR_RX:
		desc_info = &dc->desc_info[dir];
		if (!desc_info->dst_skb_buf[ring_num] || !dit_hal_get_dst_netdev(ring_num))
			return -ENODEV;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static bool dit_is_reg_value_valid(u32 value, u32 offset)
{
	struct nat_local_port local_port;
	int ret = 0;

	if (offset >= DIT_REG_NAT_RX_PORT_TABLE_SLOT) {
		local_port.hw_val = value;
		ret = dit_check_dst_ready(DIT_DIR_RX, local_port.dst_ring);
		if (ret)
			goto exit;
	}

exit:
	if (ret) {
		mif_err("reg value 0x%08X at 0x%08X is not valid. ret :%d\n", value, offset, ret);
		return false;
	}

	return true;
}

/* queue reg value writing if dit is running */
int dit_enqueue_reg_value_with_ext_lock(u32 value, u32 offset)
{
	struct dit_reg_value_item *reg_item;

	if (dit_is_kicked_any() || !dc->init_done || !list_empty(&dc->reg_value_q)) {
		reg_item = devm_kzalloc(dc->dev, sizeof(struct dit_reg_value_item), GFP_ATOMIC);
		if (!reg_item) {
			mif_err("set reg value 0x%08X at 0x%08X enqueue failed\n", value, offset);
			return -ENOMEM;
		}

		reg_item->value = value;
		reg_item->offset = offset;
		list_add_tail(&reg_item->list, &dc->reg_value_q);
	} else {
		if (dit_is_reg_value_valid(value, offset))
			WRITE_REG_VALUE(dc, value, offset);
	}

	return 0;
}
EXPORT_SYMBOL(dit_enqueue_reg_value_with_ext_lock);

int dit_enqueue_reg_value(u32 value, u32 offset)
{
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&dc->src_lock, flags);
	ret = dit_enqueue_reg_value_with_ext_lock(value, offset);
	spin_unlock_irqrestore(&dc->src_lock, flags);

	return ret;
}
EXPORT_SYMBOL(dit_enqueue_reg_value);

static void dit_clean_reg_value_with_ext_lock(void)
{
	struct dit_reg_value_item *reg_item;

	while (!list_empty(&dc->reg_value_q)) {
		reg_item = list_first_entry(&dc->reg_value_q, struct dit_reg_value_item, list);
		if (dit_is_reg_value_valid(reg_item->value, reg_item->offset))
			WRITE_REG_VALUE(dc, reg_item->value, reg_item->offset);
		list_del(&reg_item->list);
		devm_kfree(dc->dev, reg_item);
	}
}

static void dit_set_dst_skb_header(struct sk_buff *skb)
{
	/* for tcpdump with any interface */
	skb->protocol = htons(ETH_P_ALL);
	skb_reset_transport_header(skb);
	skb_reset_network_header(skb);
	skb_reset_mac_header(skb);
}

static void dit_update_stat(struct sk_buff *skb)
{
	/* remove link layer header size */
	unsigned int len = (skb->len - sizeof(struct ethhdr));

	/* update upstream stat */
	struct net_device *netdev = dit_hal_get_dst_netdev(DIT_DST_DESC_RING_0);

	if (netdev) {
#if IS_ENABLED(CONFIG_CPIF_TP_MONITOR)
		struct mem_link_device *mld = to_mem_link_device(dc->ld);

		skb_set_network_header(skb, sizeof(struct ethhdr));
		mld->tpmon->add_rx_bytes(skb);
#endif
		netdev->stats.rx_packets++;
		netdev->stats.rx_bytes += len;
	}

	dit_hal_add_data_bytes(len, 0);
}

static inline void dit_set_skb_checksum(struct dit_dst_desc *dst_desc,
		enum dit_desc_ring ring_num, struct sk_buff *skb)
{
	if ((ring_num == DIT_DST_DESC_RING_0) && dst_desc->pre_csum) {
		skb->ip_summed = CHECKSUM_UNNECESSARY;
		return;
	}

	if (dst_desc->status & DIT_CHECKSUM_FAILED_STATUS_MASK)
		return;

	if ((dst_desc->status & BIT(DIT_DESC_S_TCPC)) &&
			(dst_desc->status & BIT(DIT_DESC_S_IPCS)))
		skb->ip_summed = CHECKSUM_UNNECESSARY;
}

static inline void dit_set_skb_udp_csum_zero(struct dit_dst_desc *dst_desc,
		enum dit_desc_ring ring_num, struct sk_buff *skb)
{
	struct udphdr *uh;
	unsigned int off;

	if (ring_num == DIT_DST_DESC_RING_0)
		return;

	if (!dst_desc->udp_csum_zero)
		return;

	/* it must be IPv4 */
	off = sizeof(struct ethhdr);
	if ((*(skb->data + off) & 0xFF) != 0x45)
		return;

	off += sizeof(struct iphdr);
	uh = (struct udphdr *)(skb->data + off);
	uh->check = 0;
}

static int dit_pass_to_net(enum dit_desc_ring ring_num,
		struct sk_buff *skb)
{
	struct mem_link_device *mld;
	int ret = 0;

#if defined(DIT_DEBUG_LOW)
	dit_debug_out_of_order(DIT_DIR_RX, ring_num, skb->data);
#endif

	switch (ring_num) {
	case DIT_DST_DESC_RING_0:
		mld = to_mem_link_device(dc->ld);

		/* this function check iod and ch inside
		 * an error means further calling is not necessary
		 */
		return mld->pass_skb_to_net(mld, skb);
	case DIT_DST_DESC_RING_1:
	case DIT_DST_DESC_RING_2:
		dit_update_stat(skb);
		skb->dev = dit_hal_get_dst_netdev(ring_num);
		if (!skb->dev || !netif_running(skb->dev) || !netif_carrier_ok(skb->dev)) {
			mif_err_limited("invalid netdev!! ring_num: %d\n", ring_num);
			dev_kfree_skb_any(skb);
			break;
		}

		dit_set_dst_skb_header(skb);
		ret = dev_queue_xmit(skb);
		if (ret == NET_XMIT_DROP)
			mif_err_limited("drop!! ring_num: %d\n", ring_num);
		break;
	default:
		break;
	}

	return 0;
}

static inline void dit_reset_src_desc_kick_control(struct dit_src_desc *src_desc)
{
	u8 mask = DIT_SRC_KICK_CONTROL_MASK;

	if (!src_desc)
		return;

	src_desc->control &= ~mask;
}

static inline void dit_set_src_desc_filter_bypass(
	struct dit_src_desc *src_desc, bool bypass)
{
	/* LRO start/end bit can be used for filter bypass
	 * 1/1: filtered
	 * 0/0: filter bypass
	 * dit does not support checksum yet
	 */
	u8 mask = (BIT(DIT_DESC_C_END) | BIT(DIT_DESC_C_START));

#if defined(DIT_DEBUG_LOW)
	if (dc->force_bypass == 1)
		bypass = true;
	else if (dc->force_bypass == 2)
		bypass = false;
#endif

	if (bypass)
		src_desc->control &= ~mask;
	else
		src_desc->control |= mask;
}

static inline void dit_set_src_desc_udp_csum_zero(struct dit_src_desc *src_desc,
		u8 *src)
{
	const struct iphdr *iph = (struct iphdr *)src;
	struct udphdr *uh;
	unsigned int off;

	/* check IPv4 UDP only */
	if (((src[0] & 0xFF) != 0x45) || (iph->protocol != IPPROTO_UDP))
		return;

	off = sizeof(*iph);
	uh = (struct udphdr *)(src + off);
	if (uh->check == 0)
		src_desc->udp_csum_zero = 1;
}

static void dit_set_src_desc_kick_range(enum dit_direction dir, unsigned int src_wp,
		unsigned int src_rp)
{
	struct dit_desc_info *desc_info = &dc->desc_info[dir];
	struct dit_src_desc *src_desc;
	phys_addr_t p_desc;
	unsigned int head;
	unsigned int tail;
	u32 offset_lo = 0, offset_hi = 0, offset_en = 0;

	/* reset previous kick */
	head = dit_get_snapshot_head(dir, DIT_SRC_DESC_RING);
	src_desc = &desc_info->src_desc_ring[head];
	dit_reset_src_desc_kick_control(src_desc);

	tail = dit_get_snapshot_tail(dir, DIT_SRC_DESC_RING);
	src_desc = &desc_info->src_desc_ring[tail];
	dit_reset_src_desc_kick_control(src_desc);

	barrier();

	/* set current kick */
	head = src_rp;
	src_desc = &desc_info->src_desc_ring[head];
	src_desc->control |= BIT(DIT_DESC_C_HEAD);
	p_desc = virt_to_phys(src_desc);

	if (dir == DIT_DIR_TX) {
		offset_lo = DIT_REG_NAT_TX_DESC_ADDR_0_SRC;
		offset_hi = DIT_REG_NAT_TX_DESC_ADDR_1_SRC;
		offset_en = DIT_REG_NAT_TX_DESC_ADDR_EN_SRC;
	} else {
		offset_lo = DIT_REG_NAT_RX_DESC_ADDR_0_SRC;
		offset_hi = DIT_REG_NAT_RX_DESC_ADDR_1_SRC;
		offset_en = DIT_REG_NAT_RX_DESC_ADDR_EN_SRC;
	}

	WRITE_REG_PADDR_LO(dc, p_desc, offset_lo);
	WRITE_REG_PADDR_HI(dc, p_desc, offset_hi);
	WRITE_REG_VALUE(dc, 0x1, offset_en);

	tail = circ_prev_ptr(desc_info->src_desc_ring_len, src_wp, 1);
	src_desc = &desc_info->src_desc_ring[tail];
	src_desc->control |= BIT(DIT_DESC_C_TAIL);
	src_desc->control |= BIT(DIT_DESC_C_INT);

	src_desc = &desc_info->src_desc_ring[desc_info->src_desc_ring_len - 1];
	src_desc->control |= BIT(DIT_DESC_C_RINGEND);

	dit_set_snapshot(dir, DIT_SRC_DESC_RING, head, tail,
		circ_get_usage(desc_info->src_desc_ring_len, tail, head) + 1);
}

static void dit_set_dst_desc_int_range(enum dit_direction dir,
		enum dit_desc_ring ring_num)
{
	struct dit_desc_info *desc_info = &dc->desc_info[dir];
	struct dit_dst_desc *dst_desc;
	phys_addr_t p_desc;
	unsigned int dst_wp_pos;
	u32 offset_lo = 0, offset_hi = 0;

	dst_desc = desc_info->dst_desc_ring[ring_num];
	dst_wp_pos = desc_info->dst_wp[ring_num];
	p_desc = virt_to_phys(&dst_desc[dst_wp_pos]);

	switch (ring_num) {
	case DIT_DST_DESC_RING_0:
		if (dir == DIT_DIR_TX) {
			offset_lo = DIT_REG_NAT_TX_DESC_ADDR_0_DST0;
			offset_hi = DIT_REG_NAT_TX_DESC_ADDR_1_DST0;
		} else {
			offset_lo = DIT_REG_NAT_RX_DESC_ADDR_0_DST0;
			offset_hi = DIT_REG_NAT_RX_DESC_ADDR_1_DST0;
		}
		break;
	case DIT_DST_DESC_RING_1:
		if (dir == DIT_DIR_TX) {
			offset_lo = DIT_REG_NAT_TX_DESC_ADDR_0_DST1;
			offset_hi = DIT_REG_NAT_TX_DESC_ADDR_1_DST1;
		} else {
			offset_lo = DIT_REG_NAT_RX_DESC_ADDR_0_DST1;
			offset_hi = DIT_REG_NAT_RX_DESC_ADDR_1_DST1;
		}
		break;
	case DIT_DST_DESC_RING_2:
		if (dir == DIT_DIR_TX) {
			offset_lo = DIT_REG_NAT_TX_DESC_ADDR_0_DST2;
			offset_hi = DIT_REG_NAT_TX_DESC_ADDR_1_DST2;
		} else {
			offset_lo = DIT_REG_NAT_RX_DESC_ADDR_0_DST2;
			offset_hi = DIT_REG_NAT_RX_DESC_ADDR_1_DST2;
		}
		break;
	default:
		break;
	}

	if (offset_lo && offset_hi && (desc_info->dst_desc_ring_len > 0)) {
		WRITE_REG_PADDR_LO(dc, p_desc, offset_lo);
		WRITE_REG_PADDR_HI(dc, p_desc, offset_hi);
		dst_desc[desc_info->dst_desc_ring_len - 1].control |= BIT(DIT_DESC_C_RINGEND);
	}
}

static int dit_enqueue_src_desc_ring_internal(enum dit_direction dir,
		u8 *src, unsigned long src_paddr,
		u16 len, u16 ch_id, bool csum)
{
	struct dit_desc_info *desc_info;
	struct dit_src_desc *src_desc;
	int remain;
	struct net_device *upstream_netdev;
	struct io_device *iod;
	int src_wp = 0;
	bool filter_bypass = true;
#if defined(DIT_DEBUG)
	static unsigned int overflow;
	static unsigned int last_max_overflow;
#endif
#if defined(DIT_DEBUG_LOW)
	u32 usage;
#endif

	if (!dc)
		return -EPERM;

	desc_info = &dc->desc_info[dir];

	remain = dit_check_ring_space(desc_info->src_desc_ring_len,
		desc_info->src_wp, desc_info->src_rp);
	if (unlikely(remain < 1)) {
#if defined(DIT_DEBUG)
		if (remain == -ENOSPC)
			overflow++;
		if (overflow > last_max_overflow) {
			last_max_overflow = overflow;
			mif_err("enqueue overflow new max: %d", last_max_overflow);
		}
#endif
		return remain;
	}

#if defined(DIT_DEBUG)
	overflow = 0;
#endif
#if defined(DIT_DEBUG_LOW)
	dit_debug_out_of_order(dir, DIT_SRC_DESC_RING, src);
#endif

	src_wp = (int) desc_info->src_wp;
	src_desc = &desc_info->src_desc_ring[src_wp];
	if (src_paddr)
		src_desc->src_addr = src_paddr;
	else
		src_desc->src_addr = virt_to_phys(src);
	src_desc->length = len;
	src_desc->ch_id = ch_id;
	src_desc->pre_csum = csum;
	src_desc->udp_csum_zero = 0;
	src_desc->control = 0;
	if (src_wp == (desc_info->src_desc_ring_len - 1))
		src_desc->control |= BIT(DIT_DESC_C_RINGEND);
	src_desc->status = 0;

	do {
		if (dir != DIT_DIR_RX)
			break;
		/*
		 * check ipv6 for clat.
		 * port table does not have entries for tun device or ipv6.
		 * every ipv6 packets from any rmnet can see port table.
		 */
		if ((src[0] & 0xF0) == 0x60) {
			filter_bypass = false;
			break;
		}

		/* check upstream netdev */
		upstream_netdev = dit_hal_get_dst_netdev(DIT_DST_DESC_RING_0);
		if (upstream_netdev) {
			iod = link_get_iod_with_channel(dc->ld, ch_id);
			if (iod && (upstream_netdev == iod->ndev)) {
				dit_set_src_desc_udp_csum_zero(src_desc, src);
				filter_bypass = false;
				break;
			}
		}
	} while (0);

	dit_set_src_desc_filter_bypass(src_desc, filter_bypass);

	barrier();

	desc_info->src_wp = circ_new_ptr(desc_info->src_desc_ring_len, src_wp, 1);

	/* ensure the src_wp ordering */
	smp_mb();

#if defined(DIT_DEBUG_LOW)
	usage = circ_get_usage(desc_info->src_desc_ring_len, desc_info->src_wp, desc_info->src_rp);
	if (usage > snapshot[dir][DIT_SRC_DESC_RING].max_usage)
		snapshot[dir][DIT_SRC_DESC_RING].max_usage = usage;
#endif

	return src_wp;
}

int dit_enqueue_src_desc_ring(enum dit_direction dir,
		u8 *src, unsigned long src_paddr,
		u16 len, u16 ch_id, bool csum)
{
	return dit_enqueue_src_desc_ring_internal(
		dir, src, src_paddr, len, ch_id, csum);
}
EXPORT_SYMBOL(dit_enqueue_src_desc_ring);

int dit_enqueue_src_desc_ring_skb(enum dit_direction dir, struct sk_buff *skb)
{
	int src_wp;

	src_wp = dit_enqueue_src_desc_ring_internal(dir, skb->data,
		virt_to_phys(skb->data), skb->len,
		skbpriv(skb)->sipc_ch, (skb->ip_summed == CHECKSUM_UNNECESSARY));
	if (src_wp >= 0)
		dc->desc_info[dir].src_skb_buf[src_wp] = skb;

	return src_wp;
}
EXPORT_SYMBOL(dit_enqueue_src_desc_ring_skb);

static int dit_fill_tx_dst_data_buffer(enum dit_desc_ring ring_num, unsigned int read)
{
	struct dit_desc_info *desc_info;
	struct dit_dst_desc *dst_desc;
	unsigned int dst_rp_pos;
	unsigned int i;

	if (!dc)
		return -EPERM;

	if (!read)
		return 0;

	desc_info = &dc->desc_info[DIT_DIR_TX];
	if (unlikely(!desc_info->pktproc_pbase))
		return -EACCES;

	dst_desc = desc_info->dst_desc_ring[ring_num];
	dst_rp_pos = desc_info->dst_rp[ring_num];

	for (i = 0; i < read; i++) {
		dst_desc[dst_rp_pos].dst_addr = desc_info->pktproc_pbase +
			(dst_rp_pos * desc_info->buf_size);
		dst_rp_pos = circ_new_ptr(desc_info->dst_desc_ring_len, dst_rp_pos, 1);
	}

	return 0;
}

static int dit_fill_rx_dst_data_buffer(enum dit_desc_ring ring_num, unsigned int read, bool initial)
{
	struct dit_desc_info *desc_info;
	struct dit_dst_desc *dst_desc;
	struct sk_buff **dst_skb;
	unsigned int buf_size;
	unsigned int dst_rp_pos;
	gfp_t gfp_mask;
	int i;

	if (!dc)
		return -EPERM;

	if (!read)
		return 0;

	desc_info = &dc->desc_info[DIT_DIR_RX];

	if (initial && desc_info->dst_skb_buf_filled[ring_num])
		return 0;

	if (unlikely(!desc_info->dst_skb_buf[ring_num])) {
		buf_size = sizeof(struct sk_buff *) * desc_info->dst_desc_ring_len;
		desc_info->dst_skb_buf[ring_num] = devm_kzalloc(dc->dev, buf_size, GFP_KERNEL);
		if (!desc_info->dst_skb_buf[ring_num]) {
			mif_err("dit dst[%d] skb container alloc failed\n", ring_num);
			return -ENOMEM;
		}
	}

	dst_desc = desc_info->dst_desc_ring[ring_num];
	dst_skb = desc_info->dst_skb_buf[ring_num];
	dst_rp_pos = desc_info->dst_rp[ring_num];
	buf_size = desc_info->buf_size;

	/* fill free space */
	for (i = 0; i < read; i++) {
		if (dst_desc[dst_rp_pos].dst_addr)
			goto next;

		if (unlikely(dst_skb[dst_rp_pos]))
			goto next;

		if (initial) {
			gfp_mask = GFP_KERNEL;
			if (ring_num == DIT_DST_DESC_RING_0)
				gfp_mask = GFP_ATOMIC;

			dst_skb[dst_rp_pos] = __netdev_alloc_skb(dc->netdev, buf_size, gfp_mask);
		} else
			dst_skb[dst_rp_pos] = napi_alloc_skb(&dc->napi, buf_size);

		if (unlikely(!dst_skb[dst_rp_pos])) {
			mif_err("dit dst[%d] skb[%d] build failed\n", ring_num, dst_rp_pos);
			return -ENOMEM;
		}

#if defined(DIT_DEBUG_LOW)
		snapshot[DIT_DIR_RX][ring_num].alloc_skbs++;
#endif
		dst_desc[dst_rp_pos].dst_addr = virt_to_phys(dst_skb[dst_rp_pos]->data);

next:
		dst_rp_pos = circ_new_ptr(desc_info->dst_desc_ring_len, dst_rp_pos, 1);
	}

	if (initial)
		desc_info->dst_skb_buf_filled[ring_num] = true;

	return 0;
}

static int dit_free_dst_data_buffer(enum dit_direction dir, enum dit_desc_ring ring_num)
{
	struct dit_desc_info *desc_info;
	struct dit_dst_desc *dst_desc;
	struct sk_buff **dst_skb;
	int i;

	if (!dc)
		return -EPERM;

	desc_info = &dc->desc_info[dir];

	if (unlikely(!desc_info->dst_skb_buf[ring_num]))
		return -EINVAL;

	if (!circ_empty(desc_info->dst_wp[ring_num], desc_info->dst_rp[ring_num])) {
		mif_err("skip free. dst[%d] is processing. wp:%d rp:%d\n", ring_num,
			desc_info->dst_wp[ring_num], desc_info->dst_rp[ring_num]);
		return -EBUSY;
	}

	dst_desc = desc_info->dst_desc_ring[ring_num];
	dst_skb = desc_info->dst_skb_buf[ring_num];

	/* don't free dst_skb_buf if there are skbs will be handled in napi poll */
	for (i = 0; i < desc_info->dst_desc_ring_len; i++) {
		if (!dst_desc[i].dst_addr)
			return -EFAULT;
	}

	for (i = 0; i < desc_info->dst_desc_ring_len; i++) {
		if (dst_skb[i]) {
#if defined(DIT_DEBUG_LOW)
			snapshot[dir][ring_num].alloc_skbs--;
#endif
			dev_kfree_skb_any(dst_skb[i]);
		}
		dst_desc[i].dst_addr = 0;
	}

	mif_info("free dst[%d] skb buffers\n", ring_num);
	devm_kfree(dc->dev, dst_skb);
	desc_info->dst_skb_buf[ring_num] = NULL;
	desc_info->dst_skb_buf_filled[ring_num] = false;

	return 0;
}

/* ToDo: Tx does not have dst_skb_buf, might need another flag */
static int dit_get_dst_data_buffer_free_space(enum dit_direction dir)
{
	struct dit_desc_info *desc_info = &dc->desc_info[dir];
	unsigned int min = desc_info->dst_desc_ring_len;
	unsigned int space;
	int ring_num;

	for (ring_num = DIT_DST_DESC_RING_0; ring_num < DIT_DST_DESC_RING_MAX; ring_num++) {
		if (!desc_info->dst_skb_buf[ring_num])
			continue;

		space = circ_get_space(desc_info->dst_desc_ring_len,
			desc_info->dst_wp[ring_num], desc_info->dst_rp[ring_num]);
		if (min > space)
			min = space;
	}

	return min;
}

int dit_manage_rx_dst_data_buffers(bool fill)
{
	int ring_num;
	int ret = 0;

	/* ToDo: need to update dst wp and rp? */
	for (ring_num = DIT_DST_DESC_RING_1; ring_num < DIT_DST_DESC_RING_MAX; ring_num++) {
		if (fill) {
			ret = dit_fill_rx_dst_data_buffer(ring_num,
				dc->desc_info[DIT_DIR_RX].dst_desc_ring_len, true);
			if (ret)
				break;

			mif_info("dst[%d] filled with wp[%d] rp[%d]\n", ring_num,
				dc->desc_info[DIT_DIR_RX].dst_wp[ring_num],
				dc->desc_info[DIT_DIR_RX].dst_rp[ring_num]);
			dit_set_dst_desc_int_range(DIT_DIR_RX, ring_num);
		} else
			ret = dit_free_dst_data_buffer(DIT_DIR_RX, ring_num);
	}

	return ret;
}
EXPORT_SYMBOL(dit_manage_rx_dst_data_buffers);

int dit_read_rx_dst_poll(struct napi_struct *napi, int budget)
{
	struct dit_desc_info *desc_info = &dc->desc_info[DIT_DIR_RX];
	struct dit_dst_desc *dst_desc;
	struct sk_buff *skb;
	unsigned int rcvd_total = 0;
	unsigned int usage;
	unsigned int ring_num;
	int i, ret;
#if IS_ENABLED(CONFIG_CPIF_TP_MONITOR)
	struct mem_link_device *mld = to_mem_link_device(dc->ld);
#endif

	for (ring_num = DIT_DST_DESC_RING_0; ring_num < DIT_DST_DESC_RING_MAX; ring_num++) {
		/* read from rp to wp */
		usage = circ_get_usage(desc_info->dst_desc_ring_len,
			desc_info->dst_wp[ring_num], desc_info->dst_rp[ring_num]);
		for (i = 0; i < usage; i++) {
			if (rcvd_total >= budget)
				break;

			/* get dst desc and skb */
			dst_desc = &desc_info->dst_desc_ring[ring_num][desc_info->dst_rp[ring_num]];
			skb = desc_info->dst_skb_buf[ring_num][desc_info->dst_rp[ring_num]];

			/* try to fill dst data buffers */
			desc_info->dst_skb_buf[ring_num][desc_info->dst_rp[ring_num]] = NULL;
			ret = dit_fill_rx_dst_data_buffer(ring_num, 1, false);
			if (ret) {
				desc_info->dst_skb_buf[ring_num][desc_info->dst_rp[ring_num]] = skb;
				break;
			}

			/* set skb */
			skb_put(skb, dst_desc->length);
			skbpriv(skb)->lnk_hdr = 0;
			skbpriv(skb)->sipc_ch = dst_desc->ch_id;
			skbpriv(skb)->iod = link_get_iod_with_channel(dc->ld,
					skbpriv(skb)->sipc_ch);
			skbpriv(skb)->ld = dc->ld;
			skbpriv(skb)->napi = napi;

			/* clat */
			if ((dst_desc->packet_info & BIT(DIT_PACKET_INFO_IPV6_BIT)) &&
					((skb->data[0] & 0xFF) == 0x45)) {
				skbpriv(skb)->rx_clat = 1;
				snapshot[DIT_DIR_RX][ring_num].clat_packets++;
			}

			/* hw checksum */
			dit_set_skb_checksum(dst_desc, ring_num, skb);

			/* reset udp checksum if it was 0 */
			dit_set_skb_udp_csum_zero(dst_desc, ring_num, skb);

			dst_desc->packet_info = 0;
			dst_desc->status = 0;

			rcvd_total++;

			/* update dst rp */
			desc_info->dst_rp[ring_num] = circ_new_ptr(desc_info->dst_desc_ring_len,
				desc_info->dst_rp[ring_num], 1);

#if defined(DIT_DEBUG_LOW)
			snapshot[DIT_DIR_RX][ring_num].alloc_skbs--;
#endif
			ret = dit_pass_to_net(ring_num, skb);
			if (ret < 0)
				break;
		}
	}

#if IS_ENABLED(CONFIG_CPIF_TP_MONITOR)
	if (rcvd_total)
		mld->tpmon->start();
#endif

	if (atomic_read(&dc->stop_napi_poll)) {
		atomic_set(&dc->stop_napi_poll, 0);
		napi_complete(napi);
		/* kick can be reserved if dst buffer was not enough */
		dit_kick(DIT_DIR_RX, true);
		return 0;
	}

	if (rcvd_total < budget) {
		napi_complete_done(napi, rcvd_total);
		/* kick can be reserved if dst buffer was not enough */
		dit_kick(DIT_DIR_RX, true);
	}

	return rcvd_total;
}
EXPORT_SYMBOL(dit_read_rx_dst_poll);

static void dit_update_dst_desc_pos(enum dit_direction dir, enum dit_desc_ring ring_num)
{
	struct dit_desc_info *desc_info = &dc->desc_info[dir];
	unsigned int last_dst_wp = desc_info->dst_wp[ring_num];
	struct dit_dst_desc *dst_desc;
	struct mem_link_device *mld = to_mem_link_device(dc->ld);
	u64 packets = 0;
#if defined(DIT_DEBUG_LOW)
	u32 usage;
#endif

	do {
		dst_desc = &desc_info->dst_desc_ring[ring_num][desc_info->dst_wp[ring_num]];
		if (!(dst_desc->status & BIT(DIT_DESC_S_DONE)))
			break;

		/* update dst */
		dst_desc->status &= ~BIT(DIT_DESC_S_DONE);
		/* tx does not use status field */
		if (dir == DIT_DIR_TX)
			dst_desc->status = 0;
		if (desc_info->dst_skb_buf[ring_num])
			dst_desc->dst_addr = 0;
		desc_info->dst_wp[ring_num] = circ_new_ptr(desc_info->dst_desc_ring_len,
			desc_info->dst_wp[ring_num], 1);

		/* update src
		 * after a DST interrupt, reset all of src buf
		 */
		if (desc_info->src_skb_buf[desc_info->src_rp]) {
			dev_consume_skb_any(desc_info->src_skb_buf[desc_info->src_rp]);
			desc_info->src_skb_buf[desc_info->src_rp] = NULL;
		}
		desc_info->src_rp = circ_new_ptr(desc_info->src_desc_ring_len,
			desc_info->src_rp, 1);

		packets++;

#if defined(DIT_DEBUG)
		if (desc_info->dst_wp[ring_num] == desc_info->dst_rp[ring_num]) {
			mif_err("dst[%d] wp[%d] would overwrite rp (dir:%d)", ring_num,
				desc_info->dst_wp[ring_num], dir);
		}
#endif

#if defined(DIT_DEBUG_LOW)
		usage = circ_get_usage(desc_info->dst_desc_ring_len,
			desc_info->dst_wp[ring_num], desc_info->dst_rp[ring_num]);
		if (usage > snapshot[dir][ring_num].max_usage)
			snapshot[dir][ring_num].max_usage = usage;
#endif
	} while (1);

	if (packets > 0) {
		dit_set_dst_desc_int_range(dir, ring_num);
		dit_set_snapshot(dir, ring_num, last_dst_wp,
			circ_prev_ptr(desc_info->dst_desc_ring_len,
				desc_info->dst_wp[ring_num], 1), packets);

		/* update pktproc fore pointer */
		switch (dir) {
		case DIT_DIR_TX:
			desc_info->dst_rp[ring_num] = circ_new_ptr(desc_info->dst_desc_ring_len,
				desc_info->dst_rp[ring_num], packets);
#if IS_ENABLED(CONFIG_CP_PKTPROC_UL)
			mld->pktproc_ul.q[DIT_PKTPROC_TX_QUEUE_NUM]->update_fore_ptr(
				mld->pktproc_ul.q[DIT_PKTPROC_TX_QUEUE_NUM], packets);
#endif
			break;
		case DIT_DIR_RX:
			mld->pktproc.q[DIT_PKTPROC_RX_QUEUE_NUM]->update_fore_ptr(
				mld->pktproc.q[DIT_PKTPROC_RX_QUEUE_NUM], packets);
			break;
		default:
			mif_err_limited("dir error:%d\n", dir);
			break;
		}
	}
}

irqreturn_t dit_irq_handler(int irq, void *arg)
{
	int pending_bit = *((int *)(arg));
	enum dit_desc_ring ring_num;
	struct mem_link_device *mld;
	enum dit_direction dir = DIT_DIR_MAX;
	u32 pending_mask = DIT_ALL_INT_PENDING_MASK;

	switch (pending_bit) {
	case RX_DST0_INT_PENDING_BIT:
	case RX_DST1_INT_PENDING_BIT:
	case RX_DST2_INT_PENDING_BIT:
		dir = DIT_DIR_RX;
		pending_mask = DIT_RX_INT_PENDING_MASK;

		ring_num = (enum dit_desc_ring)(pending_bit - RX_DST0_INT_PENDING_BIT);
		dit_update_dst_desc_pos(DIT_DIR_RX, ring_num);
		if (napi_schedule_prep(&dc->napi))
			__napi_schedule(&dc->napi);
		break;
	case TX_DST0_INT_PENDING_BIT:
		dir = DIT_DIR_TX;
		pending_mask = DIT_TX_INT_PENDING_MASK;

		mld = ld_to_mem_link_device(dc->ld);
		dit_update_dst_desc_pos(DIT_DIR_TX, DIT_DST_DESC_RING_0);
		send_ipc_irq(mld, mask2int(MASK_SEND_DATA));
		break;
	case ERR_INT_PENDING_BIT:
		/* nothing to do when ERR interrupt */
		mif_err_limited("ERR interrupt!! int_pending: 0x%X",
			READ_REG_VALUE(dc, DIT_REG_INT_PENDING));
		break;
	default:
		break;
	}

	spin_lock(&dc->src_lock);
	/* do not clear ERR for debugging */
	if (pending_bit != ERR_INT_PENDING_BIT)
		WRITE_REG_VALUE(dc, BIT(pending_bit), DIT_REG_INT_PENDING);

	if ((READ_REG_VALUE(dc, DIT_REG_INT_PENDING) & pending_mask) == 0) {
		if (dir < DIT_DIR_MAX)
			dc->kicked[dir] = false;
		if (!dit_is_kicked_any()) {
#if IS_ENABLED(CONFIG_CPU_IDLE)
			exynos_update_ip_idle_status(dc->idle_ip_index, DIT_IDLE_IP_IDLE);
#endif
			dit_clean_reg_value_with_ext_lock();
		}
	}
	spin_unlock(&dc->src_lock);

	/* try init and kick again */
	dit_init(NULL, DIT_INIT_RETRY, DIT_STORE_NONE);
	if (dir < DIT_DIR_MAX)
		dit_kick(dir, true);

	return IRQ_HANDLED;
}

static bool dit_is_busy(enum dit_direction dir)
{
	u32 status_bits = 0;
	u32 status_mask = 0;
	u32 pending_bits = 0;
	u32 pending_mask = 0;

	switch (dir) {
	case DIT_DIR_TX:
		status_mask = TX_STATUS_MASK;
		pending_mask = DIT_TX_INT_PENDING_MASK;
		break;
	case DIT_DIR_RX:
		status_mask = RX_STATUS_MASK;
		pending_mask = DIT_RX_INT_PENDING_MASK;
		break;
	default:
		break;
	}

	status_bits = READ_REG_VALUE(dc, DIT_REG_STATUS);
	if (status_bits & status_mask) {
		mif_err("status = 0x%02X\n", status_bits);
		return true;
	}

	pending_bits = READ_REG_VALUE(dc, DIT_REG_INT_PENDING);
	if (pending_bits & pending_mask) {
		mif_err("pending = 0x%02X\n", pending_bits);
		return true;
	}

	return false;
}

int dit_kick(enum dit_direction dir, bool retry)
{
	int ret = 0;
	unsigned long flags;
	struct dit_desc_info *desc_info;
	u32 kick_mask = 0;
	unsigned int src_wp;
	unsigned int src_rp;

	if (unlikely(!dc))
		return -EPERM;

	spin_lock_irqsave(&dc->src_lock, flags);
	if (retry && !dc->kick_reserved[dir]) {
		ret = -EAGAIN;
		goto exit;
	}

	if (dc->kicked[dir] || !dc->init_done) {
		dc->kick_reserved[dir] = true;
		ret = -EAGAIN;
		goto exit;
	}

	if (dit_is_busy(dir)) {
		dc->kick_reserved[dir] = true;
		mif_err_limited("busy\n");
		ret = -EBUSY;
		goto exit;
	}

	desc_info = &dc->desc_info[dir];

	/* save src_wp and src_rp to prevent dst overflow */
	src_wp = desc_info->src_wp;
	src_rp = dit_get_snapshot_next_head(dir, DIT_SRC_DESC_RING,
		desc_info->src_desc_ring_len);
	if (circ_empty(src_wp, src_rp)) {
		ret = -ENODATA;
		goto exit;
	}

	/* check dst buffer space */
	if (circ_get_usage(desc_info->src_desc_ring_len, src_wp, src_rp) >
			dit_get_dst_data_buffer_free_space(dir)) {
		dc->kick_reserved[dir] = true;
		mif_err_limited("not enough dst data buffer (dir:%d)\n", dir);
		ret = -ENOSPC;
		goto exit;
	}

	switch (dir) {
	case DIT_DIR_TX:
		kick_mask = BIT(TX_COMMAND_BIT);
		break;
	case DIT_DIR_RX:
		kick_mask = BIT(RX_COMMAND_BIT);
		break;
	default:
		break;
	}

	dc->kicked[dir] = true;
	dc->kick_reserved[dir] = false;

exit:
	spin_unlock_irqrestore(&dc->src_lock, flags);

	if (ret)
		return ret;

	dit_set_src_desc_kick_range(dir, src_wp, src_rp);
#if IS_ENABLED(CONFIG_CPU_IDLE)
	exynos_update_ip_idle_status(dc->idle_ip_index, DIT_IDLE_IP_ACTIVE);
#endif
	WRITE_REG_VALUE(dc, kick_mask, DIT_REG_SW_COMMAND);

	return 0;
}
EXPORT_SYMBOL(dit_kick);

static bool dit_check_nat_enabled(void)
{
	unsigned int ring_num;

	for (ring_num = DIT_DST_DESC_RING_1; ring_num < DIT_DST_DESC_RING_MAX; ring_num++) {
		if (dit_check_dst_ready(DIT_DIR_RX, ring_num) == 0)
			return true;
	}

	return false;
}

static void dit_check_clat_enabled_internal(struct io_device *iod, void *args)
{
	bool *enabled = (bool *) args;

	if (*enabled || !dc->ld->is_ps_ch(iod->ch))
		return;

	if (iod->clat_ndev)
		*enabled = true;
}

static bool dit_check_clat_enabled(void)
{
	bool enabled = false;

	if (unlikely(!dc->ld))
		return false;

	iodevs_for_each(dc->ld->msd, dit_check_clat_enabled_internal, &enabled);

	return enabled;
}

static int dit_reg_backup_restore_internal(bool backup, const u16 *offset,
	const u16 *size, void **buf, const unsigned int arr_len)
{
	unsigned int i;
	int ret = 0;

	for (i = 0; i < arr_len; i++) {
		if (!buf[i]) {
			buf[i] = devm_kzalloc(dc->dev, size[i], GFP_KERNEL);
			if (!buf[i]) {
				ret = -ENOMEM;
				goto exit;
			}
		}

		if (backup)
			BACKUP_REG_VALUE(dc, buf[i], offset[i], size[i]);
		else
			RESTORE_REG_VALUE(dc, buf[i], offset[i], size[i]);
	}

exit:
	/* reset buffer if failed to backup */
	if (unlikely(ret && backup)) {
		for (i = 0; i < arr_len; i++) {
			if (buf[i])
				memset(buf[i], 0, size[i]);
		}
	}

	return ret;
}

static int dit_reg_backup_restore(bool backup)
{
	/* NAT */
	static const u16 nat_offset[] = {
		DIT_REG_NAT_LOCAL_ADDR,
		DIT_REG_NAT_ETHERNET_DST_MAC_ADDR_0,
		DIT_REG_NAT_RX_PORT_TABLE_SLOT,
	};
	static const u16 nat_size[] = {
		DIT_REG_NAT_LOCAL_ADDR_MAX * DIT_REG_NAT_LOCAL_INTERVAL,
		DIT_REG_NAT_LOCAL_ADDR_MAX * DIT_REG_ETHERNET_MAC_INTERVAL,
		DIT_REG_NAT_LOCAL_PORT_MAX * DIT_REG_NAT_LOCAL_INTERVAL,
	};
	static const unsigned int nat_len = ARRAY_SIZE(nat_offset);
	static void *nat_buf[ARRAY_SIZE(nat_offset)];

	/* CLAT */
	static const u16 clat_offset[] = {
		DIT_REG_CLAT_TX_FILTER,
		DIT_REG_CLAT_TX_PLAT_PREFIX_0,
		DIT_REG_CLAT_TX_CLAT_SRC_0,
	};
	static const u16 clat_size[] = {
		DIT_REG_CLAT_ADDR_MAX * DIT_REG_CLAT_TX_FILTER_INTERVAL,
		DIT_REG_CLAT_ADDR_MAX * DIT_REG_CLAT_TX_PLAT_PREFIX_INTERVAL,
		DIT_REG_CLAT_ADDR_MAX * DIT_REG_CLAT_TX_CLAT_SRC_INTERVAL,
	};
	static const unsigned int clat_len = ARRAY_SIZE(clat_offset);
	static void *clat_buf[ARRAY_SIZE(clat_offset)];

	int ret = 0;

	/* NAT */
	if (dit_check_nat_enabled()) {
		ret = dit_reg_backup_restore_internal(backup, nat_offset, nat_size, nat_buf,
			nat_len);
		if (ret)
			goto error;
	}

	/* CLAT */
	if (dit_check_clat_enabled()) {
		ret = dit_reg_backup_restore_internal(backup, clat_offset, clat_size, clat_buf,
			clat_len);
		if (ret)
			goto error;
	}

	return 0;

error:
	mif_err("backup/restore failed is_backup:%d, ret:%d\n", backup, ret);

	return ret;
}

static int dit_init_hw(void)
{
	unsigned int dir;
	unsigned int count = 0;

	const u16 port_offset_start[DIT_DIR_MAX] = {
		DIT_REG_NAT_TX_PORT_INIT_START,
		DIT_REG_NAT_RX_PORT_INIT_START
	};

	const u16 port_offset_done[DIT_DIR_MAX] = {
		DIT_REG_NAT_TX_PORT_INIT_DONE,
		DIT_REG_NAT_RX_PORT_INIT_DONE
	};

	/* set Tx/Rx port table to all zero
	 * it requires 20us at 100MHz until DONE.
	 */
	for (dir = 0; dir < DIT_DIR_MAX; dir++) {
		WRITE_REG_VALUE(dc, 0x0, port_offset_done[dir]);
		WRITE_REG_VALUE(dc, 0x1, port_offset_start[dir]);
		while (++count < 100) {
			udelay(20);
			if (READ_REG_VALUE(dc, port_offset_done[dir])) {
				break;
			}
		}

		if (count >= 100) {
			mif_err("PORT_INIT_DONE failed dir:%d\n", dir);
			return -EIO;
		}
	}

	WRITE_REG_VALUE(dc, 0x4020, DIT_REG_DMA_INIT_DATA);
	WRITE_REG_VALUE(dc, BIT(DMA_INIT_COMMAND_BIT), DIT_REG_SW_COMMAND);

	WRITE_REG_VALUE(dc, 0x0, DIT_REG_DMA_CHKSUM_OFF);
	WRITE_REG_VALUE(dc, 0xF, DIT_REG_NAT_ZERO_CHK_OFF);
	WRITE_REG_VALUE(dc, BIT(RX_ETHERNET_EN_BIT), DIT_REG_NAT_ETHERNET_EN);

	WRITE_REG_VALUE(dc, DIT_RX_BURST_16BEAT, DIT_REG_TX_DESC_CTRL_SRC);
	WRITE_REG_VALUE(dc, DIT_RX_BURST_16BEAT, DIT_REG_TX_DESC_CTRL_DST);
	WRITE_REG_VALUE(dc, DIT_RX_BURST_16BEAT, DIT_REG_TX_HEAD_CTRL);
	WRITE_REG_VALUE(dc, DIT_RX_BURST_16BEAT, DIT_REG_TX_MOD_HD_CTRL);
	WRITE_REG_VALUE(dc, DIT_RX_BURST_16BEAT, DIT_REG_TX_PKT_CTRL);
	WRITE_REG_VALUE(dc, DIT_RX_BURST_16BEAT, DIT_REG_TX_CHKSUM_CTRL);

	WRITE_REG_VALUE(dc, DIT_RX_BURST_16BEAT, DIT_REG_RX_DESC_CTRL_SRC);
	WRITE_REG_VALUE(dc, DIT_RX_BURST_16BEAT, DIT_REG_RX_DESC_CTRL_DST);
	WRITE_REG_VALUE(dc, DIT_RX_BURST_16BEAT, DIT_REG_RX_HEAD_CTRL);
	WRITE_REG_VALUE(dc, DIT_RX_BURST_16BEAT, DIT_REG_RX_MOD_HD_CTRL);
	WRITE_REG_VALUE(dc, DIT_RX_BURST_16BEAT, DIT_REG_RX_PKT_CTRL);
	WRITE_REG_VALUE(dc, DIT_RX_BURST_16BEAT, DIT_REG_RX_CHKSUM_CTRL);

	WRITE_REG_VALUE(dc, DIT_INT_ENABLE_MASK, DIT_REG_INT_ENABLE);
	WRITE_REG_VALUE(dc, DIT_INT_ENABLE_MASK, DIT_REG_INT_MASK);
	WRITE_REG_VALUE(dc, DIT_ALL_INT_PENDING_MASK, DIT_REG_INT_PENDING);

	WRITE_REG_VALUE(dc, 0x0, DIT_REG_CLK_GT_OFF);

	WRITE_SHR_VALUE(dc, dc->sharability_value);

	return 0;
}

static int dit_init_desc(enum dit_direction dir)
{
	struct dit_desc_info *desc_info = &dc->desc_info[dir];
	void *buf = NULL;
	unsigned int buf_size;
	phys_addr_t p_buf;
	int ret = 0, ring_num;
	u32 offset_lo = 0, offset_hi = 0;

	if (!desc_info->src_desc_ring) {
		buf_size = sizeof(struct dit_src_desc) *
			(desc_info->src_desc_ring_len + DIT_SRC_DESC_RING_LEN_PADDING);
		buf = devm_kzalloc(dc->dev, buf_size, GFP_KERNEL);
		if (!buf) {
			mif_err("dit dir[%d] src desc alloc failed\n", dir);
			return -ENOMEM;
		}
		desc_info->src_desc_ring = buf;
	}

	p_buf = virt_to_phys(desc_info->src_desc_ring);
	if (dir == DIT_DIR_TX) {
		offset_lo = DIT_REG_TX_RING_START_ADDR_0_SRC;
		offset_hi = DIT_REG_TX_RING_START_ADDR_1_SRC;
	} else {
		offset_lo = DIT_REG_RX_RING_START_ADDR_0_SRC;
		offset_hi = DIT_REG_RX_RING_START_ADDR_1_SRC;
	}
	WRITE_REG_PADDR_LO(dc, p_buf, offset_lo);
	WRITE_REG_PADDR_HI(dc, p_buf, offset_hi);

	if (!desc_info->src_skb_buf) {
		buf_size = sizeof(struct sk_buff *) * desc_info->src_desc_ring_len;
		buf = devm_kzalloc(dc->dev, buf_size, GFP_KERNEL);
		if (!buf) {
			mif_err("dit dir[%d] src skb container alloc failed\n", dir);
			return -ENOMEM;
		}
		desc_info->src_skb_buf = buf;
	}

	for (ring_num = DIT_DST_DESC_RING_0; ring_num < DIT_DST_DESC_RING_MAX; ring_num++) {
		offset_lo = 0;
		offset_hi = 0;

		if (!desc_info->dst_desc_ring[ring_num]) {
			buf_size = sizeof(struct dit_dst_desc) * desc_info->dst_desc_ring_len;
			buf = devm_kzalloc(dc->dev, buf_size, GFP_KERNEL);
			if (!buf) {
				mif_err("dit dir[%d] dst desc[%d] alloc failed\n", dir, ring_num);
				return -ENOMEM;
			}
			desc_info->dst_desc_ring[ring_num] = buf;
		}

		p_buf = virt_to_phys(desc_info->dst_desc_ring[ring_num]);
		switch (ring_num) {
		case DIT_DST_DESC_RING_0:
			if (dir == DIT_DIR_TX) {
				offset_lo = DIT_REG_TX_RING_START_ADDR_0_DST0;
				offset_hi = DIT_REG_TX_RING_START_ADDR_1_DST0;
				ret = dit_fill_tx_dst_data_buffer(ring_num,
					desc_info->dst_desc_ring_len);
			} else {
				offset_lo = DIT_REG_RX_RING_START_ADDR_0_DST0;
				offset_hi = DIT_REG_RX_RING_START_ADDR_1_DST0;
				ret = dit_fill_rx_dst_data_buffer(ring_num,
					desc_info->dst_desc_ring_len, true);
			}

			if (ret) {
				mif_err("dit dir[%d] dst desc[%d] buffer fill failed\n",
					dir, ring_num);
				return -ENOMEM;
			}
			break;
		case DIT_DST_DESC_RING_1:
			if (dir == DIT_DIR_TX) {
				offset_lo = DIT_REG_TX_RING_START_ADDR_0_DST1;
				offset_hi = DIT_REG_TX_RING_START_ADDR_1_DST1;
			} else {
				offset_lo = DIT_REG_RX_RING_START_ADDR_0_DST1;
				offset_hi = DIT_REG_RX_RING_START_ADDR_1_DST1;
			}
			break;
		case DIT_DST_DESC_RING_2:
			if (dir == DIT_DIR_TX) {
				offset_lo = DIT_REG_TX_RING_START_ADDR_0_DST2;
				offset_hi = DIT_REG_TX_RING_START_ADDR_1_DST2;
			} else {
				offset_lo = DIT_REG_RX_RING_START_ADDR_0_DST2;
				offset_hi = DIT_REG_RX_RING_START_ADDR_1_DST2;
			}
			break;
		default:
			break;
		}

		if (offset_lo && offset_hi) {
			WRITE_REG_PADDR_LO(dc, p_buf, offset_lo);
			WRITE_REG_PADDR_HI(dc, p_buf, offset_hi);
		}

		dit_set_dst_desc_int_range(dir, ring_num);
	}

	mif_info("dir:%d src_len:%d dst_len:%d\n",
		dir, desc_info->src_desc_ring_len, desc_info->dst_desc_ring_len);

	return 0;
}

int dit_init(struct link_device *ld, enum dit_init_type type, enum dit_store_type store)
{
	unsigned long flags;
	unsigned int dir;
	int ret = 0;

	if (unlikely(!dc)) {
		mif_err("dit not created\n");
		return -EPERM;
	}

	/* ld can be null if it is set before */
	if (!ld && !dc->ld) {
		mif_err("link device set failed\n");
		return -EINVAL;
	}

	spin_lock_irqsave(&dc->src_lock, flags);
	if (type == DIT_INIT_RETRY && !dc->init_reserved) {
		spin_unlock_irqrestore(&dc->src_lock, flags);
		return -EAGAIN;
	}

	if (dit_is_kicked_any()) {
		dc->init_reserved = true;
		spin_unlock_irqrestore(&dc->src_lock, flags);
		return -EEXIST;
	}

	if (atomic_inc_return(&dc->init_running) > 1) {
		spin_unlock_irqrestore(&dc->src_lock, flags);
		ret = -EBUSY;
		goto exit;
	}

	dc->init_done = false;
	spin_unlock_irqrestore(&dc->src_lock, flags);

	if (store == DIT_STORE_BACKUP) {
		ret = dit_reg_backup_restore(true);
		if (ret)
			goto exit;
	}

	if (type == DIT_INIT_DEINIT)
		goto exit;

	for (dir = 0; dir < DIT_DIR_MAX; dir++) {
		ret = dit_init_desc(dir);
		if (ret) {
			mif_err("dit desc init failed\n");
			goto exit;
		}
	}

	ret = dit_init_hw();
	if (ret) {
		mif_err("dit hw init failed\n");
		goto exit;
	}

	if (store == DIT_STORE_RESTORE) {
		ret = dit_reg_backup_restore(false);
		if (ret)
			goto exit;
	}

	ret = dit_net_init(dc);
	if (ret) {
		mif_err("dit net init failed\n");
		goto exit;
	}

	if (ld)
		dc->ld = ld;

	spin_lock_irqsave(&dc->src_lock, flags);
	dc->init_done = true;
	dc->init_reserved = false;
	dit_clean_reg_value_with_ext_lock();
	spin_unlock_irqrestore(&dc->src_lock, flags);

	mif_info("dit init done. hw_ver:0x%08X\n", dc->hw_version);

exit:
	atomic_dec(&dc->init_running);
	if (ret || type == DIT_INIT_DEINIT)
		return ret;

	dit_kick(DIT_DIR_TX, true);
	dit_kick(DIT_DIR_RX, true);

	return 0;
}
EXPORT_SYMBOL(dit_init);

int dit_deinit(void)
{
	return 0;
}
EXPORT_SYMBOL(dit_deinit);

static int dit_register_irq(struct platform_device *pdev)
{
	static int irq_pending_bit[] = {
		RX_DST0_INT_PENDING_BIT, RX_DST1_INT_PENDING_BIT, RX_DST2_INT_PENDING_BIT,
		TX_DST0_INT_PENDING_BIT, ERR_INT_PENDING_BIT};
	static char const *irq_name[] = {
		"DIT-RxDst0", "DIT-RxDst1", "DIT-RxDst2",
		"DIT-Tx", "DIT-Err"};

	struct device *dev = &pdev->dev;
	int irq_len = ARRAY_SIZE(irq_pending_bit);
	int irq_num;
	int ret = 0;
	int i;

	dc->irq_buf = devm_kzalloc(dev, sizeof(int) * irq_len, GFP_KERNEL);
	if (!dc->irq_buf) {
		mif_err("dit irq buf alloc failed\n");
		ret = -ENOMEM;
		goto error;
	}

	for (i = 0; i < irq_len; i++) {
		irq_num = platform_get_irq_byname(pdev, irq_name[i]);
		ret = devm_request_irq(dev, irq_num, dit_irq_handler, 0, irq_name[i],
				&irq_pending_bit[i]);
		if (ret) {
			mif_err("failed to request irq: %d, ret: %d\n", i, ret);
			ret = -EIO;
			goto error;
		}
		dc->irq_buf[i] = irq_num;
	}
	dc->irq_len = irq_len;

	return 0;

error:
	if (dc->irq_buf) {
		devm_kfree(dev, dc->irq_buf);
		dc = NULL;
	}

	return ret;
}

static ssize_t status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct dit_desc_info *desc_info;
	ssize_t count = 0;
	unsigned int wp, rp, desc_len;
	unsigned int dir, ring_num;

	count += scnprintf(&buf[count], PAGE_SIZE - count, "use tx: %d, rx: %d, clat: %d\n",
		dc->use_tx, dc->use_rx, dc->use_clat);

	for (dir = 0; dir < DIT_DIR_MAX; dir++) {
		desc_info = &dc->desc_info[dir];
		for (ring_num = 0; ring_num < DIT_DESC_RING_MAX; ring_num++) {
			if (ring_num == DIT_SRC_DESC_RING) {
				wp = desc_info->src_wp;
				rp = desc_info->src_rp;
				desc_len = desc_info->src_desc_ring_len;
			} else {
				wp = desc_info->dst_wp[ring_num];
				rp = desc_info->dst_rp[ring_num];
				desc_len = desc_info->dst_desc_ring_len;
			}

			count += scnprintf(&buf[count], PAGE_SIZE - count,
				"%s max_usage(d)/alloc(d)/total: %u/%u/%u\n",
				snapshot[dir][ring_num].name,
				snapshot[dir][ring_num].max_usage,
				snapshot[dir][ring_num].alloc_skbs, desc_len);
			count += scnprintf(&buf[count], PAGE_SIZE - count,
				"  wp: %u, rp: %u\n", wp, rp);
			count += scnprintf(&buf[count], PAGE_SIZE - count,
				"  kicked head: %d, tail: %d, packets: %llu\n",
				snapshot[dir][ring_num].head, snapshot[dir][ring_num].tail,
				snapshot[dir][ring_num].packets);
			count += scnprintf(&buf[count], PAGE_SIZE - count,
				"  total packets: %llu, clat: %llu\n",
				snapshot[dir][ring_num].total_packets,
				snapshot[dir][ring_num].clat_packets);
		}
	}

	return count;
}

static ssize_t register_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t count = 0;
	int i = 0;

	count += scnprintf(&buf[count], PAGE_SIZE - count, "INT_PENDING: 0x%X\n",
		READ_REG_VALUE(dc, DIT_REG_INT_PENDING));

	count += scnprintf(&buf[count], PAGE_SIZE - count, "STATUS: 0x%X\n",
		READ_REG_VALUE(dc, DIT_REG_STATUS));

	count += scnprintf(&buf[count], PAGE_SIZE - count, "NAT Local Address\n");
	for (i = 0; i < DIT_REG_NAT_LOCAL_ADDR_MAX; i++) {
		count += scnprintf(&buf[count], PAGE_SIZE - count,
			"  [%02d] src:0x%08X%04X, dst:0x%08X/0x%08X%04X\n",
			i,
			ntohl(READ_REG_VALUE(dc, DIT_REG_NAT_ETHERNET_SRC_MAC_ADDR_0 +
					(i * DIT_REG_ETHERNET_MAC_INTERVAL))),
			ntohs(READ_REG_VALUE(dc, DIT_REG_NAT_ETHERNET_SRC_MAC_ADDR_1 +
					(i * DIT_REG_ETHERNET_MAC_INTERVAL))),
			ntohl(READ_REG_VALUE(dc, DIT_REG_NAT_LOCAL_ADDR +
					(i * DIT_REG_NAT_LOCAL_INTERVAL))),
			ntohl(READ_REG_VALUE(dc, DIT_REG_NAT_ETHERNET_DST_MAC_ADDR_0 +
					(i * DIT_REG_ETHERNET_MAC_INTERVAL))),
			ntohs(READ_REG_VALUE(dc, DIT_REG_NAT_ETHERNET_DST_MAC_ADDR_1 +
					(i * DIT_REG_ETHERNET_MAC_INTERVAL))));
	}

	count += scnprintf(&buf[count], PAGE_SIZE - count, "CLAT Address\n");
	for (i = 0; i < DIT_REG_CLAT_ADDR_MAX; i++) {
		count += scnprintf(&buf[count], PAGE_SIZE - count,
			"  [%02d] v4:0x%08X, v6:0x%08X%08X%08X%08X, prx:0x%08X%08X%08X\n",
		i,
		ntohl(READ_REG_VALUE(dc, DIT_REG_CLAT_TX_FILTER +
					(i * DIT_REG_CLAT_TX_FILTER_INTERVAL))),
		ntohl(READ_REG_VALUE(dc, DIT_REG_CLAT_TX_CLAT_SRC_0 +
					(i * DIT_REG_CLAT_TX_CLAT_SRC_INTERVAL))),
		ntohl(READ_REG_VALUE(dc, DIT_REG_CLAT_TX_CLAT_SRC_1 +
					(i * DIT_REG_CLAT_TX_CLAT_SRC_INTERVAL))),
		ntohl(READ_REG_VALUE(dc, DIT_REG_CLAT_TX_CLAT_SRC_2 +
					(i * DIT_REG_CLAT_TX_CLAT_SRC_INTERVAL))),
		ntohl(READ_REG_VALUE(dc, DIT_REG_CLAT_TX_CLAT_SRC_3 +
					(i * DIT_REG_CLAT_TX_CLAT_SRC_INTERVAL))),
		ntohl(READ_REG_VALUE(dc, DIT_REG_CLAT_TX_PLAT_PREFIX_0 +
					(i * DIT_REG_CLAT_TX_PLAT_PREFIX_INTERVAL))),
		ntohl(READ_REG_VALUE(dc, DIT_REG_CLAT_TX_PLAT_PREFIX_1 +
					(i * DIT_REG_CLAT_TX_PLAT_PREFIX_INTERVAL))),
		ntohl(READ_REG_VALUE(dc, DIT_REG_CLAT_TX_PLAT_PREFIX_2 +
					(i * DIT_REG_CLAT_TX_PLAT_PREFIX_INTERVAL))));
	}

	count += scnprintf(&buf[count], PAGE_SIZE - count, "check logs for port table\n");
	dit_print_dump(DIT_DIR_TX, DIT_DUMP_ALL);
	dit_print_dump(DIT_DIR_RX, DIT_DUMP_ALL);

	return count;
}

#if defined(DIT_DEBUG)
static ssize_t debug_set_rx_port_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct nat_local_port local_port;
	unsigned int index;
	int ret;

	ret = sscanf(buf, "%u %x", &index, &local_port.hw_val);
	if (ret < 1)
		return -EINVAL;

	if (index >= DIT_REG_NAT_LOCAL_PORT_MAX)
		return -EINVAL;

	dit_enqueue_reg_value(local_port.hw_val,
		DIT_REG_NAT_RX_PORT_TABLE_SLOT + (index * DIT_REG_NAT_LOCAL_INTERVAL));

	return count;
}

static ssize_t debug_set_local_addr_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	u8 eth_src_str[(ETH_ALEN * 2) + 1];
	u8 eth_dst_str[(ETH_ALEN * 2) + 1];
	u64 eth_src_addr;
	u64 eth_dst_addr;
	u32 ip_addr;
	unsigned int index;
	unsigned long flags;
	int ret;

	/* for example, "0 D6CFEB352CF4 C0A82A5D 2AAD159CDE96" is for packets
	 * from D6CFEB352CF4(rndis0) to 192.168.42.93/2AAD159CDE96(neigh)
	 */
	ret = sscanf(buf, "%u %12s %x %12s", &index, eth_src_str, &ip_addr, eth_dst_str);
	if (ret < 1)
		return -EINVAL;

	ret = kstrtou64(eth_src_str, 16, &eth_src_addr);
	if (ret)
		return ret;
	ret = kstrtou64(eth_dst_str, 16, &eth_dst_addr);
	if (ret)
		return ret;

	if (index >= DIT_REG_NAT_LOCAL_ADDR_MAX)
		return -EINVAL;

	spin_lock_irqsave(&dc->src_lock, flags);
	dit_enqueue_reg_value_with_ext_lock(htonl(eth_src_addr >> 16),
		DIT_REG_NAT_ETHERNET_SRC_MAC_ADDR_0 + (index * DIT_REG_ETHERNET_MAC_INTERVAL));
	dit_enqueue_reg_value_with_ext_lock(htons(eth_src_addr & 0xFFFF),
		DIT_REG_NAT_ETHERNET_SRC_MAC_ADDR_1 + (index * DIT_REG_ETHERNET_MAC_INTERVAL));
	dit_enqueue_reg_value_with_ext_lock(htonl(ip_addr),
		DIT_REG_NAT_LOCAL_ADDR + (index * DIT_REG_NAT_LOCAL_INTERVAL));
	dit_enqueue_reg_value_with_ext_lock(htonl(eth_dst_addr >> 16),
		DIT_REG_NAT_ETHERNET_DST_MAC_ADDR_0 + (index * DIT_REG_ETHERNET_MAC_INTERVAL));
	dit_enqueue_reg_value_with_ext_lock(htons(eth_dst_addr & 0xFFFF),
		DIT_REG_NAT_ETHERNET_DST_MAC_ADDR_1 + (index * DIT_REG_ETHERNET_MAC_INTERVAL));
	dit_enqueue_reg_value_with_ext_lock(htons(ETH_P_IP),
		DIT_REG_NAT_ETHERNET_TYPE + (index * DIT_REG_ETHERNET_MAC_INTERVAL));
	spin_unlock_irqrestore(&dc->src_lock, flags);

	return count;
}

static ssize_t debug_reset_usage_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned int dir, ring_num;
	unsigned int reset_ring;
	int ret;

	ret = sscanf(buf, "%u %u", &dir, &reset_ring);
	if (ret < 1)
		return -EINVAL;

	if (dir >= DIT_DIR_MAX)
		return -EINVAL;

	if (reset_ring > DIT_DESC_RING_MAX)
		return -EINVAL;

	for (ring_num = DIT_DST_DESC_RING_0; ring_num < DIT_DESC_RING_MAX; ring_num++) {
		if ((ring_num == reset_ring) || (reset_ring == DIT_DESC_RING_MAX))
			snapshot[dir][ring_num].max_usage = 0;
	}

	return count;
}

static ssize_t debug_use_tx_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	int flag;
	int ret;

	ret = kstrtoint(buf, 0, &flag);
	if (ret)
		return -EINVAL;

	dc->use_tx = (flag > 0 ? true : false);
	return count;
}

static ssize_t debug_use_tx_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	return sysfs_emit(buf, "use_tx: %d\n", dc->use_tx);
}

static ssize_t debug_use_rx_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned int flag;
	int ret;

	ret = kstrtoint(buf, 0, &flag);
	if (ret)
		return -EINVAL;

	dc->use_rx = (flag > 0 ? true : false);
	return count;
}

static ssize_t debug_use_rx_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	return sysfs_emit(buf, "use_rx: %d\n", dc->use_rx);
}

static ssize_t debug_use_clat_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct clat_info clat;
	unsigned int i;
	unsigned int flag;
	int ret;

	ret = kstrtoint(buf, 0, &flag);
	if (ret)
		return -EINVAL;

	if (!flag) {
		memset(&clat, 0, sizeof(clat));
		for (i = 0; i < DIT_REG_CLAT_ADDR_MAX; i++) {
			clat.rmnet_index = i;
			scnprintf(clat.ipv6_iface, IFNAMSIZ, "rmnet%d", i);
			dit_hal_set_clat_info(&clat);
		}
	}

	dc->use_clat = (flag > 0 ? true : false);

	return count;
}

static ssize_t debug_use_clat_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	return sysfs_emit(buf, "use_clat: %d\n", dc->use_clat);
}
#endif

#if defined(DIT_DEBUG_LOW)
static ssize_t debug_pktgen_ch_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "pktgen ch: %d\n", dc->pktgen_ch);
}

static ssize_t debug_pktgen_ch_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int ch;
	int ret;

	ret = kstrtoint(buf, 0, &ch);
	if (ret)
		return -EINVAL;

	dc->pktgen_ch = ch;

	return count;
}

static ssize_t debug_set_force_bypass_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int bypass;
	int ret;

	ret = kstrtoint(buf, 0, &bypass);
	if (ret)
		return -EINVAL;

	dc->force_bypass = bypass;
	return count;
}
#endif

static DEVICE_ATTR_RO(status);
static DEVICE_ATTR_RO(register);
#if defined(DIT_DEBUG)
static DEVICE_ATTR_WO(debug_set_rx_port);
static DEVICE_ATTR_WO(debug_set_local_addr);
static DEVICE_ATTR_WO(debug_reset_usage);
static DEVICE_ATTR_RW(debug_use_tx);
static DEVICE_ATTR_RW(debug_use_rx);
static DEVICE_ATTR_RW(debug_use_clat);
#endif
#if defined(DIT_DEBUG_LOW)
static DEVICE_ATTR_RW(debug_pktgen_ch);
static DEVICE_ATTR_WO(debug_set_force_bypass);
#endif

static struct attribute *dit_attrs[] = {
	&dev_attr_status.attr,
	&dev_attr_register.attr,
#if defined(DIT_DEBUG)
	&dev_attr_debug_set_rx_port.attr,
	&dev_attr_debug_set_local_addr.attr,
	&dev_attr_debug_reset_usage.attr,
	&dev_attr_debug_use_tx.attr,
	&dev_attr_debug_use_rx.attr,
	&dev_attr_debug_use_clat.attr,
#endif
#if defined(DIT_DEBUG_LOW)
	&dev_attr_debug_pktgen_ch.attr,
	&dev_attr_debug_set_force_bypass.attr,
#endif
	NULL,
};

ATTRIBUTE_GROUPS(dit);

bool dit_check_dir_use_queue(enum dit_direction dir, unsigned int queue_num)
{
	if (!dc)
		return false;

	switch (dir) {
	case DIT_DIR_TX:
		if (dc->use_tx && (queue_num == DIT_PKTPROC_TX_QUEUE_NUM))
			return true;
		break;
	case DIT_DIR_RX:
		if (dc->use_rx && (queue_num == DIT_PKTPROC_RX_QUEUE_NUM))
			return true;
		break;
	default:
		break;
	}

	return false;
}
EXPORT_SYMBOL(dit_check_dir_use_queue);

int dit_get_irq_affinity(void)
{
	if (!dc)
		return -EPERM;

	return dc->irq_affinity;
}
EXPORT_SYMBOL(dit_get_irq_affinity);

int dit_set_irq_affinity(int affinity)
{
	int i;
	int num_cpu;

	if (!dc)
		return -EPERM;

#if defined(CONFIG_VENDOR_NR_CPUS)
	num_cpu = CONFIG_VENDOR_NR_CPUS;
#else
	num_cpu = 8;
#endif
	if (affinity >= num_cpu) {
		mif_err("affinity:%d error. cpu max:%d\n", affinity, num_cpu);
		return -EINVAL;
	}

	dc->irq_affinity = affinity;

	for (i = 0; i < dc->irq_len; i++) {
		mif_debug("num:%d affinity:%d\n", dc->irq_buf[i], affinity);
		irq_set_affinity_hint(dc->irq_buf[i], cpumask_of(affinity));
	}

	return 0;
}
EXPORT_SYMBOL(dit_set_irq_affinity);

int dit_set_buf_size(enum dit_direction dir, u32 size)
{
	struct dit_desc_info *desc_info = NULL;

	if (!dc)
		return -EPERM;

	desc_info = &dc->desc_info[dir];
	desc_info->buf_size = size;
	mif_info("dir:%d size:%d\n", dir, desc_info->buf_size);

	return 0;
}
EXPORT_SYMBOL(dit_set_buf_size);

int dit_set_pktproc_base(enum dit_direction dir, phys_addr_t base)
{
	struct dit_desc_info *desc_info = NULL;

	if (!dc)
		return -EPERM;

	desc_info = &dc->desc_info[dir];
	desc_info->pktproc_pbase = base;
	mif_info("dir:%d base:%pap\n", dir, &desc_info->pktproc_pbase);

	return 0;
}
EXPORT_SYMBOL(dit_set_pktproc_base);

int dit_set_desc_ring_len(enum dit_direction dir, u32 len)
{
	struct dit_desc_info *desc_info = NULL;

	if (!dc)
		return -EPERM;

	desc_info = &dc->desc_info[dir];
	desc_info->src_desc_ring_len = len;
	desc_info->dst_desc_ring_len = len;

	if (dir == DIT_DIR_RX) {
		desc_info->src_desc_ring_len += dc->rx_extra_desc_ring_len;
		desc_info->dst_desc_ring_len += dc->rx_extra_desc_ring_len;
	}

	mif_info("dir:%d len:%d src_len:%d dst_len:%d\n", dir, len,
		desc_info->src_desc_ring_len, desc_info->dst_desc_ring_len);

	return 0;
}
EXPORT_SYMBOL(dit_set_desc_ring_len);

int dit_get_src_usage(enum dit_direction dir, u32 *usage)
{
	struct dit_desc_info *desc_info = NULL;

	if (!dc)
		return -EPERM;

	desc_info = &dc->desc_info[dir];
	*usage = circ_get_usage(desc_info->src_desc_ring_len,
		desc_info->src_wp, desc_info->src_rp);
	mif_debug("dir:%d usage:%d\n", dir, *usage);

	return 0;
}
EXPORT_SYMBOL(dit_get_src_usage);

int dit_reset_dst_wp_rp(enum dit_direction dir)
{
	struct dit_desc_info *desc_info = NULL;
	int ring_num;

	if (!dc)
		return -EPERM;

	desc_info = &dc->desc_info[dir];
	for (ring_num = 0; ring_num < DIT_DST_DESC_RING_MAX; ring_num++) {
		desc_info->dst_wp[ring_num] = 0;
		desc_info->dst_rp[ring_num] = 0;
		dit_set_dst_desc_int_range(dir, ring_num);
	}

	return 0;
}
EXPORT_SYMBOL(dit_reset_dst_wp_rp);

struct net_device *dit_get_netdev(void)
{
	if (!dc)
		return NULL;

	return dc->netdev;
}
EXPORT_SYMBOL(dit_get_netdev);

int dit_stop_napi_poll(void)
{
	if (!dc)
		return -EPERM;

	atomic_set(&dc->stop_napi_poll, 1);

	return 0;
}
EXPORT_SYMBOL(dit_stop_napi_poll);

static int dit_read_dt(struct device_node *np)
{
	mif_dt_read_u32(np, "dit_sharability_offset", dc->sharability_offset);
	mif_dt_read_u32(np, "dit_sharability_value", dc->sharability_value);

	mif_dt_read_u32(np, "dit_hw_version", dc->hw_version);
	mif_dt_read_u32(np, "dit_hw_capabilities", dc->hw_capabilities);
#if defined(CONFIG_SOC_GS101)
	dc->hw_capabilities |= DIT_CAP_MASK_PORT_BIG_ENDIAN;
	/* chipid: A0 = 0, B0 = 1 */
	if (gs_chipid_get_type() >= 1)
		dc->hw_capabilities &= ~DIT_CAP_MASK_PORT_BIG_ENDIAN;
#endif
	mif_dt_read_bool(np, "dit_use_tx", dc->use_tx);
	mif_dt_read_bool(np, "dit_use_rx", dc->use_rx);
	mif_dt_read_bool(np, "dit_use_clat", dc->use_clat);
	mif_dt_read_bool(np, "dit_hal_linked", dc->hal_linked);
	mif_dt_read_u32(np, "dit_rx_extra_desc_ring_len", dc->rx_extra_desc_ring_len);
	mif_dt_read_u32(np, "dit_irq_affinity", dc->irq_affinity);

	return 0;
}

int dit_create(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct resource *res;
	int ret;

	if (!np) {
		mif_err("of_node is null\n");
		return -EINVAL;
	}

	dc = devm_kzalloc(dev, sizeof(struct dit_ctrl_t), GFP_KERNEL);
	if (!dc) {
		mif_err("dit ctrl alloc failed\n");
		return -ENOMEM;
	}

	dc->dev = dev;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dit");
	dc->register_base = devm_ioremap_resource(dev, res);
	if (!dc->register_base) {
		mif_err("register devm_ioremap error\n");
		ret = -EFAULT;
		goto error;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "sysreg");
	dc->sharability_base = devm_ioremap_resource(dev, res);
	if (!dc->sharability_base) {
		mif_err("sharability devm_ioremap error\n");
		ret = -EFAULT;
		goto error;
	}

	ret = dit_read_dt(np);
	if (ret) {
		mif_err("read dt error\n");
		goto error;
	}

	dma_set_mask_and_coherent(dev, DMA_BIT_MASK(36));

	ret = dit_register_irq(pdev);
	if (ret) {
		mif_err("register irq error\n");
		goto error;
	}

	spin_lock_init(&dc->src_lock);
	INIT_LIST_HEAD(&dc->reg_value_q);
	atomic_set(&dc->init_running, 0);
	atomic_set(&dc->stop_napi_poll, 0);

	dit_set_irq_affinity(dc->irq_affinity);
	dev_set_drvdata(dev, dc);

#if IS_ENABLED(CONFIG_CPU_IDLE)
	dc->idle_ip_index = exynos_get_idle_ip_index(dev_name(&pdev->dev));
	if (dc->idle_ip_index < 0) {
		mif_err("%s idle ip registration failed, ret: %d\n",
			dev_name(&pdev->dev), dc->idle_ip_index);
		goto error;
	}

	exynos_update_ip_idle_status(dc->idle_ip_index, DIT_IDLE_IP_IDLE);
#endif

	ret = sysfs_create_groups(&dev->kobj, dit_groups);
	if (ret != 0) {
		mif_err("sysfs_create_group() error %d\n", ret);
		goto error;
	}

	dit_hal_create(dc);
	if (ret) {
		mif_err("dit hal create failed\n");
		goto error;
	}

	mif_info("dit created. hw_ver:0x%08X, tx:%d, rx:%d, clat:%d, hal:%d, ext_len:%d, irq:%d\n",
		dc->hw_version, dc->use_tx, dc->use_rx, dc->use_clat, dc->hal_linked,
		dc->rx_extra_desc_ring_len, dc->irq_affinity);

	return 0;

error:
	if (dc->sharability_base)
		devm_iounmap(dev, dc->sharability_base);

	if (dc->register_base)
		devm_iounmap(dev, dc->register_base);

	if (dc) {
		devm_kfree(dev, dc);
		dc = NULL;
	}

	return ret;
}

static int dit_probe(struct platform_device *pdev)
{
	return dit_create(pdev);
}

static int dit_remove(struct platform_device *pdev)
{
	return 0;
}

static int dit_suspend(struct device *dev)
{
	struct dit_ctrl_t *dc = dev_get_drvdata(dev);
	int ret;

	if (unlikely(!dc) || unlikely(!dc->ld))
		return 0;

	ret = dit_init(NULL, DIT_INIT_DEINIT, DIT_STORE_BACKUP);
	if (ret) {
		mif_err("deinit failed ret:%d\n", ret);
		return ret;
	}

	return 0;
}

static int dit_resume(struct device *dev)
{
	struct dit_ctrl_t *dc = dev_get_drvdata(dev);
	unsigned int dir;
	int ret;

	if (unlikely(!dc)) {
		mif_err_limited("dc is null\n");
		return -EPERM;
	}

	if (unlikely(!dc->ld)) {
		mif_err_limited("dc->ld is null\n");
		return -EPERM;
	}

	dit_set_irq_affinity(dc->irq_affinity);

	ret = dit_init(NULL, DIT_INIT_NORMAL, DIT_STORE_RESTORE);
	if (ret) {
		mif_err("init failed ret:%d\n", ret);
		for (dir = 0; dir < DIT_DIR_MAX; dir++) {
			if (dit_is_busy(dir))
				mif_err("busy (dir:%d)\n", dir);
		}
		return ret;
	}

	return 0;
}

static const struct dev_pm_ops dit_pm_ops = {
	SET_NOIRQ_SYSTEM_SLEEP_PM_OPS(dit_suspend, dit_resume)
};

static const struct of_device_id dit_dt_match[] = {
	{ .compatible = "samsung,exynos-dit", },
	{},
};
MODULE_DEVICE_TABLE(of, dit_dt_match);

static struct platform_driver dit_driver = {
	.probe = dit_probe,
	.remove = dit_remove,
	.driver = {
		.name = "dit",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(dit_dt_match),
		.pm = &dit_pm_ops,
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(dit_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Samsung DIT Driver");

