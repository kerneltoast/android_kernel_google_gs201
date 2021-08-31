// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2018-2019, Samsung Electronics.
 *
 */

#include <asm/cacheflush.h>
#include <linux/ip.h>
#include <linux/ipv6.h>
#include <linux/udp.h>
#include <linux/shm_ipc.h>
#include "modem_prj.h"
#include "modem_utils.h"
#include "link_device_memory.h"
#include "dit.h"

static struct pktproc_perftest_data perftest_data[PERFTEST_MODE_MAX] = {
	{
		/* empty */
	},
	{
		/* PERFTEST_MODE_IPV4
		 * port: 5000 -> 5001
		 * payload: 1464 (0x5b8)
		 */
		.header = {
			0x45, 0x00, 0x05, 0xB8, 0x00, 0x00, 0x40, 0x00,
			0x80, 0x11, 0x71, 0xDF, 0xC0, 0xA8, 0x01, 0x03,
			0xC0, 0xA8, 0x01, 0x02, 0x13, 0x88, 0x13, 0x89,
			0x05, 0xA4, 0x00, 0x00
		},
		.header_len = 28,
		.dst_port_offset = 22,
		.packet_len = 1464
	},
	{
		/* PERFTEST_MODE_CLAT
		 * port: 5000 -> 5001
		 * payload: 1444 (0x5a4)
		 */
		.header = {
			0x60, 0x0a, 0xf8, 0x0c, 0x05, 0xa4, 0x11, 0x40,
			0x00, 0x64, 0xff, 0x9b, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x02,
			0x20, 0x01, 0x02, 0xd8, 0xe1, 0x43, 0x7b, 0xfb,
			0x1d, 0xda, 0x90, 0x9d, 0x8b, 0x8d, 0x05, 0xe7,
			0x13, 0x88, 0x13, 0x89, 0x05, 0xa4, 0x00, 0x00,
		},
		.header_len = 48,
		.dst_port_offset = 42,
		.packet_len = 1484
	},
	{
		/* PERFTEST_MODE_IPV6
		 * port: 5000 -> 5001
		 * payload: 1444 (0x5a4)
		 */
		.header = {
			0x60, 0x0a, 0xf8, 0x0c, 0x05, 0x90, 0x11, 0x40,
			0x20, 0x01, 0x00, 0x02, 0x00, 0x00, 0x00, 0x01,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
			0x20, 0x01, 0x00, 0x02, 0x00, 0x00, 0x00, 0x02,
			0x7d, 0xae, 0x3d, 0x4e, 0xac, 0xf2, 0x8a, 0x2b,
			0x13, 0x88, 0x13, 0x89, 0x05, 0x90, 0x00, 0x00,
		},
		.header_len = 48,
		.dst_port_offset = 42,
		.packet_len = 1464
	},
};

static bool pktproc_check_hw_checksum(u8 status)
{
	if (unlikely(status & PKTPROC_STATUS_IGNR))
		return false;
	if (unlikely(!(status & PKTPROC_STATUS_IPCS) || !(status & PKTPROC_STATUS_TCPC)))
		return false;
	if (unlikely((status & PKTPROC_STATUS_IPCSF) || (status & PKTPROC_STATUS_TCPCF)))
		return false;

	return true;
}

static void pktproc_set_pktgen_checksum(struct pktproc_queue *q, u8 *data)
{
	unsigned int off;
	struct udphdr *uh;

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
	uh->check = htons(0x1234);
}

static ssize_t pktgen_gro_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct modem_ctl *mc = dev_get_drvdata(dev);
	struct link_device *ld = get_current_link(mc->iod);
	struct mem_link_device *mld = to_mem_link_device(ld);
	struct pktproc_adaptor *ppa = &mld->pktproc;
	unsigned int gro;
	int ret;

	ret = kstrtoint(buf, 0, &gro);
	if (ret)
		return -EINVAL;

	ppa->pktgen_gro = (gro > 0 ? true : false);

	return count;
}

static ssize_t pktgen_gro_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct modem_ctl *mc = dev_get_drvdata(dev);
	struct link_device *ld = get_current_link(mc->iod);
	struct mem_link_device *mld = to_mem_link_device(ld);
	struct pktproc_adaptor *ppa = &mld->pktproc;
	ssize_t count = 0;

	count += scnprintf(&buf[count], PAGE_SIZE - count, "pktgen gro:%d\n", ppa->pktgen_gro);

	return count;
}

/*
 * Get a packet: ringbuf mode
 */
static int pktproc_get_pkt_from_ringbuf_mode(struct pktproc_queue *q, struct sk_buff **new_skb)
{
	int ret = 0;
	u16 len;
	u16 ch_id;
	u8 *src;
	struct sk_buff *skb = NULL;
	struct link_device *ld = &q->mld->link_dev;
	struct pktproc_desc_ringbuf *desc = q->desc_ringbuf;

	if (!pktproc_check_active(q->ppa, q->q_idx)) {
		mif_err_limited("Queue %d not activated\n", q->q_idx);
		return -EACCES;
	}
	if (q->ppa->desc_mode != DESC_MODE_RINGBUF) {
		mif_err_limited("Invalid desc_mode %d\n", q->ppa->desc_mode);
		return -EINVAL;
	}

	/* Get data */
	len = desc[*q->rear_ptr].length;
	if (len > q->ppa->max_packet_size) {
		mif_err_limited("Length is invalid:%d\n", len);
		q->stat.err_len++;
		ret = -EPERM;
		goto rx_error_on_desc;
	}
	ch_id = desc[*q->rear_ptr].channel_id;
	if (ch_id == SIPC5_CH_ID_MAX) {
		mif_err_limited("Channel ID is invalid:%d\n", ch_id);
		q->stat.err_chid++;
		ret = -EPERM;
		goto rx_error_on_desc;
	}
	if (q->ppa->use_36bit_data_addr)
		src = desc[*q->rear_ptr].cp_data_paddr - q->q_buff_pbase + q->q_buff_vbase;
	else
		src = desc[*q->rear_ptr].cp_data_paddr - q->cp_buff_pbase + q->q_buff_vbase;
	if ((src < q->q_buff_vbase) || (src > q->q_buff_vbase + q->q_buff_size)) {
		mif_err_limited("Data address is invalid:%pK q_buff_vbase:%pK size:0x%08x\n",
						src, q->q_buff_vbase, q->q_buff_size);
		q->stat.err_addr++;
		ret = -EINVAL;
		goto rx_error_on_desc;
	}
	if (q->ppa->buff_rgn_cached && !q->ppa->use_hw_iocc)
		dma_sync_single_for_cpu(q->ppa->dev, virt_to_phys(src), len, DMA_FROM_DEVICE);
	pp_debug("len:%d ch_id:%d src:%pK\n", len, ch_id, src);

	/* Build skb */
	if (q->ppa->use_napi)
		skb = napi_alloc_skb(q->napi_ptr, len);
	else
		skb = dev_alloc_skb(len);
	if (unlikely(!skb)) {
		mif_err_limited("alloc_skb() error\n");
		q->stat.err_nomem++;
		ret = -ENOMEM;
		goto rx_error;
	}
	skb_put(skb, len);
	skb_copy_to_linear_data(skb, src, len);
#ifdef PKTPROC_DEBUG_PKT
	pr_buffer("pktproc", (char *)skb->data, (size_t)len, (size_t)40);
#endif

	/* Set priv */
	skbpriv(skb)->lnk_hdr = 0;
	skbpriv(skb)->sipc_ch = ch_id;
	skbpriv(skb)->iod = link_get_iod_with_channel(ld, skbpriv(skb)->sipc_ch);
	skbpriv(skb)->ld = ld;
	if (q->ppa->use_napi)
		skbpriv(skb)->napi = q->napi_ptr;
	else
		skbpriv(skb)->napi = NULL;

	switch (q->ppa->version) {
	case PKTPROC_V2:
		if (pktproc_check_hw_checksum(desc[*q->rear_ptr].status))
			skb->ip_summed = CHECKSUM_UNNECESSARY;
		else
			q->stat.err_csum++;
		break;
	default:
		break;
	}

	if (unlikely(q->ppa->pktgen_gro)) {
		pktproc_set_pktgen_checksum(q, skb->data);
		skb->ip_summed = CHECKSUM_UNNECESSARY;
	}

	*new_skb = skb;
	q->stat.pass_cnt++;
	*q->rear_ptr = circ_new_ptr(q->num_desc, *q->rear_ptr, 1);

	return 0;

rx_error_on_desc:
	mif_err_limited("Skip invalid descriptor at %d\n", *q->rear_ptr);
	*q->rear_ptr = circ_new_ptr(q->num_desc, *q->rear_ptr, 1);

rx_error:
	if (skb)
		dev_kfree_skb_any(skb);

	return ret;
}

/*
 * Get a packet : sktbuf mode on 32bit region
 */
static int pktproc_clear_data_addr(struct pktproc_queue *q)
{
	struct pktproc_desc_sktbuf *desc = q->desc_sktbuf;
	u8 *src;

	if (q->ppa->desc_mode != DESC_MODE_SKTBUF) {
		mif_err_limited("Invalid desc_mode %d\n", q->ppa->desc_mode);
		return -EINVAL;
	}

	if (!q->ppa->use_buff_mng) {
		mif_err_limited("Buffer manager is not set\n");
		return -EPERM;
	}

	mif_info("Unmap buffer from %d to %d\n", q->done_ptr, *q->fore_ptr);
	while (*q->fore_ptr != q->done_ptr) {
		if (q->ppa->buff_rgn_cached && !q->ppa->use_hw_iocc &&
			q->dma_addr[q->done_ptr])
			dma_unmap_single_attrs(q->ppa->dev, q->dma_addr[q->done_ptr],
							q->manager->cell_size, DMA_FROM_DEVICE, 0);

		if (q->ppa->use_36bit_data_addr)
			src = desc[q->done_ptr].cp_data_paddr - q->q_buff_pbase +
					q->q_buff_vbase - q->ppa->skb_padding_size;
		else
			src = desc[q->done_ptr].cp_data_paddr - q->cp_buff_pbase +
					q->q_buff_vbase - q->ppa->skb_padding_size;
		if (src)
			free_mif_buff(q->manager, src);

		q->done_ptr = circ_new_ptr(q->num_desc, q->done_ptr, 1);
	}
	memset(desc, 0, q->desc_size);

	return 0;
}

static int pktproc_clear_data_addr_without_bm(struct pktproc_queue *q)
{
	struct pktproc_desc_sktbuf *desc = q->desc_sktbuf;
	int i;

	if (q->ppa->desc_mode != DESC_MODE_SKTBUF) {
		mif_err_limited("Invalid desc_mode %d\n", q->ppa->desc_mode);
		return -EINVAL;
	}

	if (q->ppa->use_buff_mng) {
		mif_err_limited("Buffer manager is set\n");
		return -EPERM;
	}

	mif_info("Unmap buffer from %d to %d\n", q->done_ptr, *q->fore_ptr);
	for (i = 0; i < q->num_desc; i++) {
		if (q->ppa->buff_rgn_cached && !q->ppa->use_hw_iocc &&
			q->dma_addr[i])
			dma_unmap_single_attrs(q->ppa->dev, q->dma_addr[i],
					q->ppa->max_packet_size, DMA_FROM_DEVICE, 0);
	}
	memset(desc, 0, q->desc_size);

	return 0;
}

static int pktproc_fill_data_addr(struct pktproc_queue *q)
{
	struct pktproc_desc_sktbuf *desc = q->desc_sktbuf;
	u8 *dst_vaddr = NULL;
	u32 space;
	u32 fore;
	int i;
	unsigned long flags;

	if (q->ppa->desc_mode != DESC_MODE_SKTBUF) {
		mif_err_limited("Invalid desc_mode %d\n", q->ppa->desc_mode);
		return -EINVAL;
	}

	if (!q->ppa->use_buff_mng) {
		mif_err_limited("Buffer manager is not set\n");
		return -EPERM;
	}

	spin_lock_irqsave(&q->lock, flags);

	space = circ_get_space(q->num_desc, *q->fore_ptr, q->done_ptr);
	pp_debug("Q%d:%d/%d/%d Space:%d BM:%d/%d/%d\n",
		q->q_idx, *q->fore_ptr, *q->rear_ptr, q->done_ptr, space,
		q->manager->cell_count, q->manager->used_cell_count, q->manager->free_cell_count);

	fore = *q->fore_ptr;
	for (i = 0; i < space; i++) {
		dst_vaddr = alloc_mif_buff(q->manager);
		if (!dst_vaddr) {
			mif_err_limited("alloc error for space: %d Q%d:%d/%d/%d BM:%d/%d/%d\n",
				space, q->q_idx, *q->fore_ptr, *q->rear_ptr, q->done_ptr,
				q->manager->cell_count, q->manager->used_cell_count,
				q->manager->free_cell_count);

			q->stat.err_bm_nomem++;
			spin_unlock_irqrestore(&q->lock, flags);
			return -ENOMEM;
		}

		pp_debug("Q:%d fore_ptr:%d dst_vaddr:%pK\n", q->q_idx, *q->fore_ptr, dst_vaddr);

		if (q->ppa->buff_rgn_cached && !q->ppa->use_hw_iocc) {
			q->dma_addr[fore] = dma_map_single_attrs(q->ppa->dev, dst_vaddr,
					q->manager->cell_size, DMA_FROM_DEVICE, 0);
			if (dma_mapping_error(q->ppa->dev, q->dma_addr[fore])) {
				mif_err_limited("dma_map_single_attrs() failed\n");
				spin_unlock_irqrestore(&q->lock, flags);
				return -ENOMEM;
			}
		}

		if (q->ppa->use_36bit_data_addr)
			desc[fore].cp_data_paddr = (dst_vaddr - q->q_buff_vbase) +
					q->q_buff_pbase + q->ppa->skb_padding_size;
		else
			desc[fore].cp_data_paddr = (dst_vaddr - q->q_buff_vbase) +
					q->cp_buff_pbase + q->ppa->skb_padding_size;

		if (fore == 0)
			desc[fore].control |= (1 << 7);	/* HEAD */

		if (fore == (q->num_desc - 1))
			desc[fore].control |= (1 << 3);	/* RINGEND */

		*q->fore_ptr = circ_new_ptr(q->num_desc, *q->fore_ptr, 1);
		fore = circ_new_ptr(q->num_desc, fore, 1);
	}

	pp_debug("Q:%d fore/rear/done:%d/%d/%d\n",
			q->q_idx, *q->fore_ptr, *q->rear_ptr, q->done_ptr);

	spin_unlock_irqrestore(&q->lock, flags);

	return 0;
}

static int pktproc_fill_data_addr_without_bm(struct pktproc_queue *q)
{
	struct pktproc_desc_sktbuf *desc = q->desc_sktbuf;
	u8 *dst_vaddr = NULL;
	unsigned long dst_paddr;
	u32 fore;
	int i;
	unsigned long flags;

	if (q->ppa->desc_mode != DESC_MODE_SKTBUF) {
		mif_err_limited("Invalid desc_mode %d\n", q->ppa->desc_mode);
		return -EINVAL;
	}

	if (q->ppa->use_buff_mng) {
		mif_err_limited("Buffer manager is set\n");
		return -EPERM;
	}

	mif_info("Q%d:%d/%d/%d\n",
			q->q_idx, *q->fore_ptr, *q->rear_ptr, q->done_ptr);

	spin_lock_irqsave(&q->lock, flags);

	fore = *q->fore_ptr;
	for (i = 0; i < q->num_desc; i++) {
		dst_paddr = q->q_buff_pbase + (q->ppa->max_packet_size * fore);
		if (dst_paddr > (q->q_buff_pbase + q->q_buff_size))
			mif_err_limited("dst_paddr:0x%lx is over 0x%lx\n",
					dst_paddr, q->q_buff_pbase + q->q_buff_size);

		pp_debug("Q:%d fore_ptr:%d dst_paddr:0x%lx\n",
			q->q_idx, *q->fore_ptr, dst_paddr);

		if (q->ppa->buff_rgn_cached && !q->ppa->use_hw_iocc) {
			dst_vaddr = q->q_buff_vbase + (q->ppa->max_packet_size * i);
			if (dst_vaddr > (q->q_buff_vbase + q->q_buff_size))
				mif_err_limited("dst_vaddr:0x%lx is over 0x%lx\n",
						dst_vaddr, q->q_buff_vbase + q->q_buff_size);

			q->dma_addr[fore] = dma_map_single_attrs(q->ppa->dev, dst_vaddr,
					q->ppa->max_packet_size, DMA_FROM_DEVICE, 0);
			if (dma_mapping_error(q->ppa->dev, q->dma_addr[fore])) {
				mif_err_limited("dma_map_single_attrs() failed\n");
				spin_unlock_irqrestore(&q->lock, flags);
				return -ENOMEM;
			}
		}

		if (q->ppa->use_36bit_data_addr)
			desc[fore].cp_data_paddr = dst_paddr +
							q->ppa->skb_padding_size;
		else
			desc[fore].cp_data_paddr = (dst_paddr - q->q_buff_pbase) +
							q->cp_buff_pbase +
							q->ppa->skb_padding_size;

		if (fore == 0)
			desc[fore].control |= (1 << 7);	/* HEAD */

		if (fore == (q->num_desc - 1)) {
			desc[fore].control |= (1 << 3);	/* RINGEND */
			continue;
		}

		*q->fore_ptr = circ_new_ptr(q->num_desc, *q->fore_ptr, 1);
		fore = circ_new_ptr(q->num_desc, fore, 1);
	}

	mif_info("Q:%d fore/rear/done:%d/%d/%d\n",
			q->q_idx, *q->fore_ptr, *q->rear_ptr, q->done_ptr);

	spin_unlock_irqrestore(&q->lock, flags);

	return 0;
}

static int pktproc_update_fore_ptr(struct pktproc_queue *q, u32 count)
{
	*q->fore_ptr = circ_new_ptr(q->num_desc, *q->fore_ptr, count);

	return 0;
}

static int pktproc_get_pkt_from_sktbuf_mode(struct pktproc_queue *q, struct sk_buff **new_skb)
{
	int ret = 0;
	u16 len, tmp_len;
	u16 ch_id;
	u8 *src;
	unsigned long src_paddr;
	struct sk_buff *skb = NULL;
	struct pktproc_desc_sktbuf desc_done_ptr = q->desc_sktbuf[q->done_ptr];
	struct link_device *ld = &q->mld->link_dev;
	u32 packet_size = 0;
	bool csum = false;

	if (!pktproc_check_active(q->ppa, q->q_idx)) {
		mif_err_limited("Queue %d not activated\n", q->q_idx);
		return -EACCES;
	}

	if (q->ppa->desc_mode != DESC_MODE_SKTBUF) {
		mif_err_limited("Invalid desc_mode %d\n", q->ppa->desc_mode);
		return -EINVAL;
	}

	/* Get data */
	len = desc_done_ptr.length;
	if (len > q->ppa->max_packet_size) {
		mif_err_limited("Length is invalid:%d\n", len);
		q->stat.err_len++;
		ret = -EPERM;
		goto rx_error_on_desc;
	}

	ch_id = desc_done_ptr.channel_id;
	if (ch_id == SIPC5_CH_ID_MAX) {
		mif_err_limited("Channel ID is invalid:%d\n", ch_id);
		q->stat.err_chid++;
		ret = -EPERM;
		goto rx_error_on_desc;
	}

	if (q->ppa->use_36bit_data_addr) {
		src_paddr = desc_done_ptr.cp_data_paddr - q->ppa->skb_padding_size;
		src = desc_done_ptr.cp_data_paddr - q->q_buff_pbase +
				q->q_buff_vbase - q->ppa->skb_padding_size;
	} else {
		src_paddr = desc_done_ptr.cp_data_paddr - q->cp_buff_pbase +
				q->q_buff_pbase - q->ppa->skb_padding_size;
		src = desc_done_ptr.cp_data_paddr - q->cp_buff_pbase +
				q->q_buff_vbase - q->ppa->skb_padding_size;
	}
	if ((src < q->q_buff_vbase) || (src > q->q_buff_vbase + q->q_buff_size)) {
		mif_err_limited("Data address is invalid:%pK data:%pK size:0x%08x\n",
					src, q->q_buff_vbase, q->q_buff_size);
		q->stat.err_addr++;
		ret = -EINVAL;
		goto rx_error_on_desc;
	}

	if (q->ppa->buff_rgn_cached && !q->ppa->use_hw_iocc) {
		if (q->manager)
			packet_size = q->manager->cell_size;
		else
			packet_size = q->ppa->max_packet_size;

		dma_unmap_single_attrs(q->ppa->dev, q->dma_addr[q->done_ptr],
					packet_size, DMA_FROM_DEVICE, 0);
	}

	csum = pktproc_check_hw_checksum(desc_done_ptr.status);
	if (!csum)
		q->stat.err_csum++;

	if (unlikely(q->ppa->pktgen_gro)) {
		pktproc_set_pktgen_checksum(q, src);
		csum = true;
	}

	pp_debug("Q:%d done_ptr:%d len:%d ch_id:%d src:%pK csum:%d\n",
			q->q_idx, q->done_ptr, len, ch_id, src, csum);

#ifdef PKTPROC_DEBUG_PKT
	pr_buffer("pktproc", (char *)src, (size_t)len, (size_t)40);
#endif

	if (dit_check_dir_use_queue(DIT_DIR_RX, q->q_idx)) {
		ret = dit_enqueue_src_desc_ring(DIT_DIR_RX,
			src, src_paddr, len, ch_id, csum);
		if (ret < 0) {
			mif_err_limited("Enqueue failed at fore/rear/done:%d/%d/%d, ret: %d\n",
				*q->fore_ptr, *q->rear_ptr, q->done_ptr, ret);

			q->stat.err_enqueue_dit++;
			goto rx_error;
		}

		q->stat.pass_cnt++;
		q->done_ptr = circ_new_ptr(q->num_desc, q->done_ptr, 1);

		return 0;
	}

	/* Build skb */
	if (q->use_memcpy) {
		if (q->ppa->use_napi)
			skb = napi_alloc_skb(q->napi_ptr, len);
		else
			skb = dev_alloc_skb(len);
		if (unlikely(!skb)) {
			mif_err_limited("alloc_skb() error\n");
			q->stat.err_nomem++;
			ret = -ENOMEM;
			goto rx_error;
		}
		skb_put(skb, len);
		skb_copy_to_linear_data(skb, src + q->ppa->skb_padding_size, len);
		if (q->manager)
			free_mif_buff(q->manager, src);
		q->stat.use_memcpy_cnt++;
	} else {
		tmp_len = SKB_DATA_ALIGN(len + q->ppa->skb_padding_size);
		tmp_len += SKB_DATA_ALIGN(sizeof(struct skb_shared_info));
		skb = build_skb(src, tmp_len);
		if (unlikely(!skb)) {
			mif_err_limited("build_skb() error\n");
			q->stat.err_nomem++;
			ret = -ENOMEM;
			goto rx_error;
		}
		skb->head_frag = 0;
		skb_reserve(skb, q->ppa->skb_padding_size);
		skb_put(skb, len);
	}

	if (csum)
		skb->ip_summed = CHECKSUM_UNNECESSARY;

	/* Set priv */
	skbpriv(skb)->lnk_hdr = 0;
	skbpriv(skb)->sipc_ch = ch_id;
	skbpriv(skb)->iod = link_get_iod_with_channel(ld, ch_id);
	skbpriv(skb)->ld = ld;
	if (q->ppa->use_napi)
		skbpriv(skb)->napi = q->napi_ptr;
	else
		skbpriv(skb)->napi = NULL;

	*new_skb = skb;

	q->stat.pass_cnt++;
	q->done_ptr = circ_new_ptr(q->num_desc, q->done_ptr, 1);

	return 0;

rx_error_on_desc:
	mif_err_limited("Skip invalid descriptor at %d\n", q->done_ptr);
	q->done_ptr = circ_new_ptr(q->num_desc, q->done_ptr, 1);

rx_error:
	if (skb)
		dev_kfree_skb_any(skb);

	return ret;
}

/*
 * Clean RX ring
 */
int pktproc_get_usage(struct pktproc_queue *q)
{
	u32 usage = 0;

	switch (q->ppa->desc_mode) {
	case DESC_MODE_RINGBUF:
		usage = circ_get_usage(q->num_desc, *q->fore_ptr, *q->rear_ptr);
		break;
	case DESC_MODE_SKTBUF:
		usage = circ_get_usage(q->num_desc, *q->rear_ptr, q->done_ptr);
		break;
	default:
		usage = 0;
		break;
	}

	return usage;
}

int pktproc_get_usage_fore_rear(struct pktproc_queue *q)
{
	u32 usage = 0;

	switch (q->ppa->desc_mode) {
	case DESC_MODE_RINGBUF:
		usage = circ_get_usage(q->num_desc, *q->fore_ptr, *q->rear_ptr);
		break;
	case DESC_MODE_SKTBUF:
		usage = circ_get_usage(q->num_desc, *q->rear_ptr, *q->fore_ptr);
		break;
	default:
		usage = 0;
		break;
	}

	return usage;
}

static bool pktproc_check_memcpy_mode(struct pktproc_queue *q, u32 budget)
{
	if (q->ppa->desc_mode != DESC_MODE_SKTBUF)
		return true;

	if (!q->manager)
		return true;

	if (!q->manager->enable_sw_zerocopy)
		return true;

	if (q->manager->free_cell_count < budget)
		return true;

	if (circ_get_space(q->num_desc, *q->rear_ptr, *q->fore_ptr) < budget)
		return true;

	return false;
}

static int pktproc_clean_rx_ring(struct pktproc_queue *q, int budget, int *work_done)
{
	int ret = 0;
	u32 num_frames = 0;
	u32 rcvd = 0;

	if (!pktproc_check_active(q->ppa, q->q_idx))
		return -EACCES;

	if (q->ppa->use_napi)
		num_frames = min_t(unsigned int, pktproc_get_usage(q), budget);
	else
		num_frames = pktproc_get_usage(q);

	if (!num_frames)
		return 0;

	q->use_memcpy = pktproc_check_memcpy_mode(q, budget);

	pp_debug("Q%d num_frames:%d memcpy:%d %d/%d/%d\n",
			q->q_idx, num_frames, q->use_memcpy,
			*q->fore_ptr, *q->rear_ptr, q->done_ptr);

	while (rcvd < num_frames) {
		struct sk_buff *skb = NULL;

		ret = q->get_packet(q, &skb);
		if (unlikely(ret < 0)) {
			mif_err_limited("get_packet() error %d\n", ret);
			break;
		}

		if (dit_check_dir_use_queue(DIT_DIR_RX, q->q_idx)) {
			rcvd++;
			continue;
		}

		if (unlikely(!skb)) {
			mif_err_limited("skb is null\n");
			ret = -ENOMEM;
			break;
		}

		rcvd++;

		ret = q->mld->pass_skb_to_net(q->mld, skb);
		if (ret < 0)
			break;
	}

	if (dit_check_dir_use_queue(DIT_DIR_RX, q->q_idx)) {
		pp_debug("rcvd for dit:%d\n", rcvd);
		if (rcvd > 0)
			dit_kick(DIT_DIR_RX, false);
	} else {
#if IS_ENABLED(CONFIG_CPIF_TP_MONITOR)
		if (rcvd)
			tpmon_start();
#endif
		if (q->ppa->manager) {
			ret = q->alloc_rx_buf(q);
			if (ret)
				mif_err_limited("alloc_rx_buf() error %d Q%d\n", ret, q->q_idx);
		} else {
			q->update_fore_ptr(q, rcvd);
		}
	}

	if (q->ppa->use_napi)
		*work_done = rcvd;

	return ret;
}

/*
 * perftest
 */
static void pktproc_perftest_napi_schedule(void *arg)
{
	struct pktproc_queue *q = (struct pktproc_queue *)arg;

	if (!q) {
		mif_err_limited("q is null\n");
		return;
	}

	if (!pktproc_get_usage(q))
		return;

	if (napi_schedule_prep(q->napi_ptr)) {
		q->disable_irq(q);
		__napi_schedule(q->napi_ptr);
	}
}

static unsigned int pktproc_perftest_gen_rx_packet_sktbuf_mode(
		struct pktproc_queue *q, int packet_num, int session)
{
	struct pktproc_desc_sktbuf *desc = q->desc_sktbuf;
	struct pktproc_perftest *perf = &q->ppa->perftest;
	u32 header_len = perftest_data[perf->mode].header_len;
	u32 rear_ptr;
	unsigned int space, loop_count;
	u8 *src;
	u32 *seq;
	u16 *dst_port;
	u16 *dst_addr;
	int i, j;

	rear_ptr = *q->rear_ptr;
	space = circ_get_space(q->num_desc, rear_ptr, *q->fore_ptr);
	loop_count = min_t(unsigned int, space, packet_num);

	for (i = 0 ; i < loop_count ; i++) {
		/* set desc */
		desc[rear_ptr].status =
			PKTPROC_STATUS_DONE | PKTPROC_STATUS_TCPC | PKTPROC_STATUS_IPCS;
		desc[rear_ptr].length = perftest_data[perf->mode].packet_len;
		desc[rear_ptr].filter_result = 0x9;
		desc[rear_ptr].channel_id = perf->ch;

		/* set data */
		if (q->ppa->use_36bit_data_addr)
			src = desc[rear_ptr].cp_data_paddr -
					q->q_buff_pbase + q->q_buff_vbase;
		else
			src = desc[rear_ptr].cp_data_paddr -
					q->cp_buff_pbase + q->q_buff_vbase;
		memset(src, 0x0, desc[rear_ptr].length);
		memcpy(src, perftest_data[perf->mode].header, header_len);
		seq = (u32 *)(src + header_len);
		*seq = htonl(perf->seq_counter[session]++);
		dst_port = (u16 *)(src + perftest_data[perf->mode].dst_port_offset);
		*dst_port = htons(5001 + session);
		if (perf->mode == PERFTEST_MODE_CLAT) {
			for (j = 0 ; j < 8 ; j++) {
				dst_addr = (u16 *)(src + 24 + (j * 2));
				*dst_addr = htons(perf->clat_ipv6[j]);
			}
		}

		rear_ptr = circ_new_ptr(q->num_desc, rear_ptr, 1);
	}

	*q->rear_ptr = rear_ptr;

	return loop_count;
}

static int pktproc_perftest_thread(void *arg)
{
	struct mem_link_device *mld = (struct mem_link_device *) arg;
	struct pktproc_adaptor *ppa = &mld->pktproc;
	struct pktproc_queue *q = ppa->q[0];
	struct pktproc_perftest *perf = &ppa->perftest;
	bool session_queue = false;
	int i, pkts;

	if (perf->session > PKTPROC_MAX_QUEUE)
		perf->session = PKTPROC_MAX_QUEUE;

	if (ppa->use_exclusive_irq && (perf->session > 1) && (perf->session <= ppa->num_queue))
		session_queue = true;

	/* max 1023 packets per 1ms for 12Gbps */
	pkts = (perf->session > 0 ? (1023 / perf->session) : 0);
	do {
		for (i = 0 ; i < perf->session ; i++) {
			if (session_queue)
				q = ppa->q[i];

			if (!pktproc_perftest_gen_rx_packet_sktbuf_mode(q, pkts, i))
				continue;

			if (ppa->use_napi) {
				if (session_queue) {
					if (cpu_online(perf->ipi_cpu[i]))
						smp_call_function_single(perf->ipi_cpu[i],
							pktproc_perftest_napi_schedule,
							(void *)q, 0);
					else
						pktproc_perftest_napi_schedule(q);
				} else {
					pktproc_perftest_napi_schedule(q);
				}
			} else {
				q->irq_handler(q->irq, q);
			}
		}

		udelay(perf->udelay);

		if (kthread_should_stop())
			break;
	} while (perf->test_run);

	return 0;
}

static ssize_t perftest_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct modem_ctl *mc = dev_get_drvdata(dev);
	struct link_device *ld = get_current_link(mc->iod);
	struct mem_link_device *mld = to_mem_link_device(ld);
	struct pktproc_adaptor *ppa = &mld->pktproc;
	struct pktproc_perftest *perf = &ppa->perftest;

	static struct task_struct *worker_task;
	int ret;
	int cpu = 5;

	if (ppa->use_exclusive_irq) {
		perf->ipi_cpu[0] = 4;
		perf->ipi_cpu[1] = 4;
		perf->ipi_cpu[2] = 4;
		perf->ipi_cpu[3] = 4;
	}

	ret = sscanf(buf, "%d %d %hu %d %d %hx:%hx:%hx:%hx:%hx:%hx:%hx:%hx %d %d %d %d",
		&perf->mode, &perf->session, &perf->ch, &cpu, &perf->udelay,
		&perf->clat_ipv6[0], &perf->clat_ipv6[1], &perf->clat_ipv6[2], &perf->clat_ipv6[3],
		&perf->clat_ipv6[4], &perf->clat_ipv6[5], &perf->clat_ipv6[6], &perf->clat_ipv6[7],
		&perf->ipi_cpu[0], &perf->ipi_cpu[1], &perf->ipi_cpu[2], &perf->ipi_cpu[3]);

	if (ret < 1)
		return -EINVAL;

	switch (perf->mode) {
	case PERFTEST_MODE_STOP:
		if (perf->test_run)
			kthread_stop(worker_task);

		perf->seq_counter[0] = 0;
		perf->seq_counter[1] = 0;
		perf->seq_counter[2] = 0;
		perf->seq_counter[3] = 0;
		perf->test_run = false;
		break;
	case PERFTEST_MODE_IPV4:
	case PERFTEST_MODE_CLAT:
	case PERFTEST_MODE_IPV6:
		if (perf->test_run)
			kthread_stop(worker_task);

		perf->test_run = true;
		worker_task = kthread_create_on_node(pktproc_perftest_thread,
			mld, cpu_to_node(cpu), "perftest", cpu);
		kthread_bind(worker_task, cpu);
		wake_up_process(worker_task);
		break;
	default:
		break;
	}

	return count;
}

static ssize_t perftest_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct modem_ctl *mc = dev_get_drvdata(dev);
	struct link_device *ld = get_current_link(mc->iod);
	struct mem_link_device *mld = to_mem_link_device(ld);
	struct pktproc_adaptor *ppa = &mld->pktproc;
	struct pktproc_perftest *perf = &ppa->perftest;
	ssize_t count = 0;

	count += scnprintf(&buf[count], PAGE_SIZE - count, "test_run:%d\n", perf->test_run);
	count += scnprintf(&buf[count], PAGE_SIZE - count, "mode:%d\n", perf->mode);
	count += scnprintf(&buf[count], PAGE_SIZE - count, "session:%d\n", perf->session);
	count += scnprintf(&buf[count], PAGE_SIZE - count, "ch:%d\n", perf->ch);
	count += scnprintf(&buf[count], PAGE_SIZE - count, "udelay:%d\n", perf->udelay);
	count += scnprintf(&buf[count], PAGE_SIZE - count, "cpu:%d\n", perf->cpu);

	if (ppa->use_exclusive_irq)
		count += scnprintf(&buf[count], PAGE_SIZE - count, "ipi cpu:%d %d %d %d\n",
			perf->ipi_cpu[0], perf->ipi_cpu[1], perf->ipi_cpu[2], perf->ipi_cpu[3]);

	if (perf->mode == PERFTEST_MODE_CLAT)
		count += scnprintf(&buf[count], PAGE_SIZE - count, "clat %x:%x:%x:%x:%x:%x:%x:%x\n",
			perf->clat_ipv6[0], perf->clat_ipv6[1], perf->clat_ipv6[2],
			perf->clat_ipv6[3], perf->clat_ipv6[4], perf->clat_ipv6[5],
			perf->clat_ipv6[6], perf->clat_ipv6[7]);

	return count;
}

/*
 * NAPI
 */
static void pktproc_enable_irq(struct pktproc_queue *q)
{
#if IS_ENABLED(CONFIG_MCU_IPC)
	cp_mbox_enable_handler(q->irq_idx, q->mld->irq_cp2ap_msg);
#endif
}

static void pktproc_disable_irq(struct pktproc_queue *q)
{
#if IS_ENABLED(CONFIG_MCU_IPC)
	cp_mbox_disable_handler(q->irq_idx, q->mld->irq_cp2ap_msg);
#endif
}

static int pktproc_poll(struct napi_struct *napi, int budget)
{
	struct pktproc_queue *q = container_of(napi, struct pktproc_queue, napi);
	struct mem_link_device *mld = q->mld;
	struct link_device *ld = &mld->link_dev;
	struct modem_ctl *mc = ld->mc;

	int ret;
	u32 rcvd = 0;

	if (unlikely(!cp_online(mc)))
		goto poll_exit;

	if (!pktproc_check_active(q->ppa, q->q_idx))
		goto poll_exit;

	ret = q->clean_rx_ring(q, budget, &rcvd);
	if ((ret == -EBUSY) || (ret == -ENOMEM))
		goto poll_retry;

	if (atomic_read(&q->stop_napi_poll)) {
		atomic_set(&q->stop_napi_poll, 0);
		goto poll_exit;
	}

	if (rcvd < budget) {
		napi_complete_done(napi, rcvd);
		q->enable_irq(q);

		return rcvd;
	}

poll_retry:
	return budget;

poll_exit:
	napi_complete(napi);
	q->enable_irq(q);

	return 0;
}

/*
 * IRQ handler
 */
static irqreturn_t pktproc_irq_handler(int irq, void *arg)
{
	struct pktproc_queue *q = (struct pktproc_queue *)arg;
	int ret = 0;

	if (!q) {
		mif_err_limited("q is null\n");
		return IRQ_HANDLED;
	}

	if (!pktproc_get_usage(q))
		return IRQ_HANDLED;

#if IS_ENABLED(CONFIG_CPIF_TP_MONITOR)
	tpmon_start();
#endif

	if (q->ppa->use_napi) {
		if (napi_schedule_prep(q->napi_ptr)) {
			q->disable_irq(q);
			__napi_schedule(q->napi_ptr);
		}
	} else {
		ret = q->clean_rx_ring(q, 0, NULL);
		if ((ret == -EBUSY) || (ret == -ENOMEM))
			return IRQ_HANDLED;
	}

	return IRQ_HANDLED;
}

/*
 * Debug
 */
static ssize_t region_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct modem_ctl *mc = dev_get_drvdata(dev);
	struct link_device *ld = get_current_link(mc->iod);
	struct mem_link_device *mld = to_mem_link_device(ld);
	struct pktproc_adaptor *ppa = &mld->pktproc;
	ssize_t count = 0;
	int i;

	count += scnprintf(&buf[count], PAGE_SIZE - count, "Version:%d\n", ppa->version);
	count += scnprintf(&buf[count], PAGE_SIZE - count, "CP base:0x%08x\n", ppa->cp_base);
	count += scnprintf(&buf[count], PAGE_SIZE - count, "Descriptor mode:%d\n", ppa->desc_mode);
	count += scnprintf(&buf[count], PAGE_SIZE - count, "Num of queue:%d\n", ppa->num_queue);
	count += scnprintf(&buf[count], PAGE_SIZE - count, "Buffer manager:%d\n",
		ppa->use_buff_mng);
	count += scnprintf(&buf[count], PAGE_SIZE - count, "NAPI:%d\n", ppa->use_napi);
	count += scnprintf(&buf[count], PAGE_SIZE - count, "Exclusive interrupt:%d\n",
		ppa->use_exclusive_irq);
	count += scnprintf(&buf[count], PAGE_SIZE - count, "HW cache coherency:%d\n",
		ppa->use_hw_iocc);
	count += scnprintf(&buf[count], PAGE_SIZE - count, "Max packet size:%d\n",
		ppa->max_packet_size);
	count += scnprintf(&buf[count], PAGE_SIZE - count, "Padding size:%d\n",
		ppa->skb_padding_size);
	count += scnprintf(&buf[count], PAGE_SIZE - count, "Dedicated BAAW:%d\n",
		ppa->use_dedicated_baaw);
	count += scnprintf(&buf[count], PAGE_SIZE - count, "36bit data addr:%d\n",
		ppa->use_36bit_data_addr);
	count += scnprintf(&buf[count], PAGE_SIZE - count, "info_desc:%s/buff:%s\n",
		ppa->info_desc_rgn_cached ? "C" : "NC",
		ppa->buff_rgn_cached ? "C" : "NC");
	count += scnprintf(&buf[count], PAGE_SIZE - count, "\n");

	if (ppa->manager) {
		count += scnprintf(&buf[count], PAGE_SIZE - count, "Buffer manager\n");
		count += scnprintf(&buf[count], PAGE_SIZE - count, "  buffer size:0x%08x\n",
			ppa->manager->buffer_size);
		count += scnprintf(&buf[count], PAGE_SIZE - count, "  total cell count:%d\n",
			ppa->manager->cell_count);
		count += scnprintf(&buf[count], PAGE_SIZE - count, "  cell size:%d\n",
			ppa->manager->cell_size);
		count += scnprintf(&buf[count], PAGE_SIZE - count, "  enable sw zerocopy:%d\n",
			ppa->manager->enable_sw_zerocopy);
		count += scnprintf(&buf[count], PAGE_SIZE - count, "\n");
	}

	for (i = 0; i < ppa->num_queue; i++) {
		struct pktproc_queue *q = ppa->q[i];

		if (!pktproc_check_active(ppa, q->q_idx)) {
			count += scnprintf(&buf[count], PAGE_SIZE - count,
				"Queue %d is not active\n", i);
			continue;
		}

		count += scnprintf(&buf[count], PAGE_SIZE - count, "Queue%d\n", i);
		count += scnprintf(&buf[count], PAGE_SIZE - count, "  num_desc:%d(0x%08x)\n",
			q->q_info_ptr->num_desc, q->q_info_ptr->num_desc);
		count += scnprintf(&buf[count], PAGE_SIZE - count, "  cp_desc_pbase:0x%08x\n",
			q->q_info_ptr->cp_desc_pbase);
		count += scnprintf(&buf[count], PAGE_SIZE - count, "  desc_size:0x%08x\n",
			q->desc_size);
		count += scnprintf(&buf[count], PAGE_SIZE - count, "  cp_buff_pbase:0x%08x\n",
			q->q_info_ptr->cp_buff_pbase);
		count += scnprintf(&buf[count], PAGE_SIZE - count, "  q_buff_size:0x%08x\n",
			q->q_buff_size);
		count += scnprintf(&buf[count], PAGE_SIZE - count, "  DIT:%d\n",
			dit_check_dir_use_queue(DIT_DIR_RX, q->q_idx));
	}

	return count;
}

static ssize_t status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct modem_ctl *mc = dev_get_drvdata(dev);
	struct link_device *ld = get_current_link(mc->iod);
	struct mem_link_device *mld = to_mem_link_device(ld);
	struct pktproc_adaptor *ppa = &mld->pktproc;
	ssize_t count = 0;
	int i;

	if (ppa->manager) {
		count += scnprintf(&buf[count], PAGE_SIZE - count,
			"Buffer manager total/use/free:%d/%d/%d\n",
			ppa->manager->cell_count, ppa->manager->used_cell_count,
			ppa->manager->free_cell_count);
		count += scnprintf(&buf[count], PAGE_SIZE - count, "\n");
	}

	for (i = 0; i < ppa->num_queue; i++) {
		struct pktproc_queue *q = ppa->q[i];

		if (!pktproc_check_active(ppa, q->q_idx)) {
			count += scnprintf(&buf[count], PAGE_SIZE - count,
				"Queue %d is not active\n", i);
			continue;
		}

		count += scnprintf(&buf[count], PAGE_SIZE - count, "Queue%d\n", i);
		count += scnprintf(&buf[count], PAGE_SIZE - count, "  num_desc:%d\n",
			q->num_desc);
		switch (ppa->desc_mode) {
		case DESC_MODE_RINGBUF:
			count += scnprintf(&buf[count], PAGE_SIZE - count, "  fore/rear:%d/%d\n",
				*q->fore_ptr, *q->rear_ptr);
			count += scnprintf(&buf[count], PAGE_SIZE - count, "  fore~rear:%d\n",
				circ_get_usage(q->num_desc, *q->fore_ptr, *q->rear_ptr));
			break;
		case DESC_MODE_SKTBUF:
			count += scnprintf(&buf[count], PAGE_SIZE - count,
				"  fore/rear/done:%d/%d/%d\n",
				*q->fore_ptr, *q->rear_ptr, q->done_ptr);
			count += scnprintf(&buf[count], PAGE_SIZE - count,
				"  fore~rear:%d rear~done:%d rear~fore:%d\n",
				circ_get_usage(q->num_desc, *q->fore_ptr, *q->rear_ptr),
				circ_get_usage(q->num_desc, *q->rear_ptr, q->done_ptr),
				circ_get_usage(q->num_desc, *q->rear_ptr, *q->fore_ptr));
			if (!dit_check_dir_use_queue(DIT_DIR_RX, q->q_idx))
				count += scnprintf(&buf[count], PAGE_SIZE - count,
					"  use memcpy:%d count:%lld\n",
					q->use_memcpy, q->stat.use_memcpy_cnt);
			break;
		default:
			break;
		}

		count += scnprintf(&buf[count], PAGE_SIZE - count, "  pass:%lld\n",
			q->stat.pass_cnt);
		count += scnprintf(&buf[count], PAGE_SIZE - count,
			"  fail:len%lld chid%lld addr%lld nomem%lld bmnomem%lld csum%lld\n",
			q->stat.err_len, q->stat.err_chid, q->stat.err_addr,
			q->stat.err_nomem, q->stat.err_bm_nomem, q->stat.err_csum);
		if (dit_check_dir_use_queue(DIT_DIR_RX, q->q_idx))
			count += scnprintf(&buf[count], PAGE_SIZE - count,
				"  fail:enqueue_dit%lld\n",
				q->stat.err_enqueue_dit);
	}

	return count;
}

static DEVICE_ATTR_RO(region);
static DEVICE_ATTR_RO(status);
static DEVICE_ATTR_RW(perftest);
static DEVICE_ATTR_RW(pktgen_gro);

static struct attribute *pktproc_attrs[] = {
	&dev_attr_region.attr,
	&dev_attr_status.attr,
	&dev_attr_perftest.attr,
	&dev_attr_pktgen_gro.attr,
	NULL,
};

static const struct attribute_group pktproc_group = {
	.attrs = pktproc_attrs,
	.name = "pktproc",
};

/*
 * Initialize PktProc
 */
int pktproc_init(struct pktproc_adaptor *ppa)
{
	int i;
	int ret = 0;

	if (!ppa) {
		mif_err("ppa is null\n");
		return -EPERM;
	}

	if (!pktproc_check_support(ppa))
		return 0;

	mif_info("version:%d cp_base:0x%08x desc_mode:%d num_queue:%d\n",
		ppa->version, ppa->cp_base, ppa->desc_mode, ppa->num_queue);
	mif_info("interrupt:%d napi:%d iocc:%d max_packet_size:%d\n",
		ppa->use_exclusive_irq, ppa->use_napi,
		ppa->use_hw_iocc, ppa->max_packet_size);

	if (ppa->manager)
		mif_info("buffer_size:0x%08x cell_count:%d cell_size:%d sw_zeocopy:%d\n",
			ppa->manager->buffer_size, ppa->manager->cell_count,
			ppa->manager->cell_size, ppa->manager->enable_sw_zerocopy);

	for (i = 0; i < ppa->num_queue; i++) {
		struct pktproc_queue *q = ppa->q[i];

		mif_info("Q%d\n", i);

		if (ppa->use_napi)
			napi_synchronize(&q->napi);

		switch (ppa->desc_mode) {
		case DESC_MODE_SKTBUF:
			if (pktproc_check_active(q->ppa, q->q_idx))
				q->clear_data_addr(q);
			break;
		default:
			break;
		}

		*q->fore_ptr = 0;
		*q->rear_ptr = 0;
		q->done_ptr = 0;

		q->q_info_ptr->cp_desc_pbase = q->cp_desc_pbase;
		q->q_info_ptr->num_desc = q->num_desc;
		q->q_info_ptr->cp_buff_pbase = q->cp_buff_pbase;

		memset(&q->stat, 0, sizeof(struct pktproc_statistics));

		switch (ppa->desc_mode) {
		case DESC_MODE_SKTBUF:
			ret = q->alloc_rx_buf(q);
			if (ret) {
				mif_err_limited("alloc_rx_buf() error %d Q%d\n", ret, q->q_idx);
				continue;
			}
			break;
		default:
			break;
		}

		mif_info("num_desc:0x%08x cp_desc_pbase:0x%08x desc_size:0x%08x\n",
				q->num_desc, q->cp_desc_pbase, q->desc_size);
		mif_info("cp_buff_pbase:0x%08x q_buff_size:0x%08x\n",
				q->cp_buff_pbase, q->q_buff_size);
		mif_info("fore:%d rear:%d done:%d\n",
				*q->fore_ptr, *q->rear_ptr, q->done_ptr);

		atomic_set(&q->active, 1);
	}

	return 0;
}

/*
 * Create PktProc
 */
static int pktproc_create_buffer_manager(struct pktproc_adaptor *ppa, u32 buff_size)
{
	if (!ppa) {
		mif_err("ppa is null\n");
		return -EINVAL;
	}

	if (!ppa->use_buff_mng) {
		mif_err("use_buff_mng is not set\n");
		return -EINVAL;
	}

	if (ppa->manager != NULL) {
		mif_info("buffer manager is already initialized\n");
		return 0;
	}

	/* Use only one buffer manager for all queues */
	ppa->manager = init_mif_buff_mng(ppa->buff_vbase, buff_size,
					ppa->max_packet_size + MIF_BUFF_CELL_PADDING_SIZE);
	if (!ppa->manager) {
		mif_err("init_mif_buff_mng() error\n");
		return -ENOMEM;
	}

	return 0;
}

static int pktproc_get_info(struct pktproc_adaptor *ppa, struct device_node *np)
{
	int ret = 0;

	mif_dt_read_u32(np, "pktproc_version", ppa->version);
	mif_dt_read_u32(np, "pktproc_cp_base", ppa->cp_base);
	switch (ppa->version) {
	case PKTPROC_V1:
		ppa->desc_mode = DESC_MODE_RINGBUF;
		ppa->num_queue = 1;
		ppa->use_exclusive_irq = 0;
		break;
	case PKTPROC_V2:
		mif_dt_read_u32(np, "pktproc_desc_mode", ppa->desc_mode);
		mif_dt_read_u32(np, "pktproc_num_queue", ppa->num_queue);
		mif_dt_read_u32(np, "pktproc_use_buff_mng", ppa->use_buff_mng);
		mif_dt_read_u32(np, "pktproc_desc_num_ratio_percent", ppa->desc_num_ratio_percent);
		mif_dt_read_u32(np, "pktproc_use_exclusive_irq", ppa->use_exclusive_irq);
		if (ppa->use_exclusive_irq) {
			ret = of_property_read_u32_array(np, "pktproc_exclusive_irq_idx",
							ppa->exclusive_irq_idx, ppa->num_queue);
			if (ret) {
				mif_err("pktproc_exclusive_irq_idx error:%d\n", ret);
				return ret;
			}
		}
		mif_dt_read_bool(np, "pktproc_use_napi", ppa->use_napi);
		mif_dt_read_u32(np, "pktproc_use_36bit_data_addr", ppa->use_36bit_data_addr);
		break;
	default:
		mif_err("Unsupported version:%d\n", ppa->version);
		return -EINVAL;
	}

	mif_info("version:%d cp_base:0x%08x mode:%d num_queue:%d\n",
		ppa->version, ppa->cp_base, ppa->desc_mode, ppa->num_queue);
	mif_info("use_buff_mng:%d use_napi:%d exclusive_irq:%d\n",
		ppa->use_buff_mng, ppa->use_napi, ppa->use_exclusive_irq);

	mif_dt_read_u32(np, "pktproc_use_hw_iocc", ppa->use_hw_iocc);
	mif_dt_read_u32(np, "pktproc_max_packet_size", ppa->max_packet_size);
	mif_dt_read_u32(np, "pktproc_use_dedicated_baaw", ppa->use_dedicated_baaw);
	mif_info("iocc:%d max_packet_size:%d baaw:%d 36bit_data:%d\n",
		ppa->use_hw_iocc, ppa->max_packet_size,
		ppa->use_dedicated_baaw, ppa->use_36bit_data_addr);

	mif_dt_read_u32(np, "pktproc_info_rgn_offset", ppa->info_rgn_offset);
	mif_dt_read_u32(np, "pktproc_info_rgn_size", ppa->info_rgn_size);
	mif_dt_read_u32(np, "pktproc_desc_rgn_offset", ppa->desc_rgn_offset);
	mif_dt_read_u32(np, "pktproc_desc_rgn_size", ppa->desc_rgn_size);
	mif_dt_read_u32(np, "pktproc_buff_rgn_offset", ppa->buff_rgn_offset);
	mif_info("info_rgn 0x%08x 0x%08x desc_rgn 0x%08x 0x%08x %u buff_rgn 0x%08x\n",
		ppa->info_rgn_offset, ppa->info_rgn_size,
		ppa->desc_rgn_offset, ppa->desc_rgn_size,
		ppa->desc_num_ratio_percent, ppa->buff_rgn_offset);

	mif_dt_read_u32(np, "pktproc_info_desc_rgn_cached", ppa->info_desc_rgn_cached);
	mif_dt_read_u32(np, "pktproc_buff_rgn_cached", ppa->buff_rgn_cached);
	mif_info("cached:%d/%d\n", ppa->info_desc_rgn_cached, ppa->buff_rgn_cached);

	return 0;
}

int pktproc_create(struct platform_device *pdev, struct mem_link_device *mld,
					unsigned long memaddr, u32 memsize)
{
	struct device_node *np = pdev->dev.of_node;
	struct pktproc_adaptor *ppa = &mld->pktproc;
	u32 buff_size, buff_size_by_q;
	int i;
	int ret = 0;

	if (!np) {
		mif_err("of_node is null\n");
		return -EINVAL;
	}
	if (!ppa) {
		mif_err("ppa is null\n");
		return -EINVAL;
	}

	ppa->dev = &pdev->dev;

	mif_dt_read_u32_noerr(np, "pktproc_support", ppa->support);
	if (!ppa->support) {
		mif_info("pktproc_support is 0. Just return\n");
		return 0;
	}

	/* Get info */
	ret = pktproc_get_info(ppa, np);
	if (ret != 0) {
		mif_err("pktproc_get_dt() error %d\n", ret);
		return ret;
	}

	/* Get base addr */
	mif_info("memaddr:0x%lx memsize:0x%08x\n", memaddr, memsize);
	if (ppa->info_desc_rgn_cached) {
		ppa->info_vbase = phys_to_virt(memaddr + ppa->info_rgn_offset);
		ppa->desc_vbase = phys_to_virt(memaddr + ppa->desc_rgn_offset);
	} else {
		ppa->info_vbase = cp_shmem_get_nc_region(memaddr + ppa->info_rgn_offset,
				ppa->info_rgn_size + ppa->desc_rgn_size);
		if (!ppa->info_vbase) {
			mif_err("ppa->info_base error\n");
			return -ENOMEM;
		}
		ppa->desc_vbase = ppa->info_vbase + ppa->info_rgn_size;
	}
	memset(ppa->info_vbase, 0, ppa->info_rgn_size + ppa->desc_rgn_size);
	mif_info("info + desc size:0x%08x\n", ppa->info_rgn_size + ppa->desc_rgn_size);

	buff_size = memsize - (ppa->info_rgn_size + ppa->desc_rgn_size);
	buff_size_by_q = buff_size / ppa->num_queue;
	ppa->buff_pbase = memaddr + ppa->buff_rgn_offset;
	if (ppa->buff_rgn_cached)
		ppa->buff_vbase = phys_to_virt(ppa->buff_pbase);
	else
		ppa->buff_vbase = cp_shmem_get_nc_region(ppa->buff_pbase, buff_size);
	mif_info("Total buff buffer size:0x%08x Queue:%d Size by queue:0x%08x\n",
					buff_size, ppa->num_queue, buff_size_by_q);

	/* Create buffer manager */
	if (ppa->use_buff_mng) {
		mif_info("create buffer manager\n");
		ret = pktproc_create_buffer_manager(ppa, buff_size);
		if (ret < 0) {
			mif_err("pktproc_create_buffer_manager() error:%d\n", ret);
			goto create_error;
		}
		ppa->skb_padding_size = NET_SKB_PAD + NET_IP_ALIGN;
	} else {
		ppa->manager = NULL;
		ppa->skb_padding_size = 0;
	}

	/* Create queue */
	for (i = 0; i < ppa->num_queue; i++) {
		struct pktproc_queue *q;

		mif_info("Create queue %d\n", i);

		ppa->q[i] = kzalloc(sizeof(struct pktproc_queue), GFP_ATOMIC);
		if (ppa->q[i] == NULL) {
			mif_err_limited("kzalloc() error %d\n", i);
			ret = -ENOMEM;
			goto create_error;
		}
		q = ppa->q[i];

		atomic_set(&q->active, 0);
		atomic_set(&q->stop_napi_poll, 0);

		/* Info region */
		switch (ppa->version) {
		case PKTPROC_V1:
			q->info_v1 = (struct pktproc_info_v1 *)ppa->info_vbase;
			q->q_info_ptr = &q->info_v1->q_info;
			break;
		case PKTPROC_V2:
			q->info_v2 = (struct pktproc_info_v2 *)ppa->info_vbase;
			q->info_v2->num_queues = ppa->num_queue;
			q->info_v2->desc_mode = ppa->desc_mode;
			q->info_v2->irq_mode = ppa->use_exclusive_irq;
			q->info_v2->max_packet_size = ppa->max_packet_size;

			q->q_info_ptr = &q->info_v2->q_info[i];
			break;
		default:
			mif_err("Unsupported version:%d\n", ppa->version);
			ret = -EINVAL;
			goto create_error;
		}

		/* Descriptor, data buffer region */
		switch (ppa->desc_mode) {
		case DESC_MODE_RINGBUF:
			q->q_buff_pbase = ppa->buff_pbase + (i * buff_size_by_q);
			q->q_buff_vbase = ppa->buff_vbase + (i * buff_size_by_q);
			q->cp_buff_pbase = ppa->cp_base + ppa->buff_rgn_offset +
				(i * buff_size_by_q);
			q->q_info_ptr->cp_buff_pbase = q->cp_buff_pbase;
			q->q_buff_size = buff_size_by_q;
			if (q->ppa->buff_rgn_cached && !ppa->use_hw_iocc)
				dma_sync_single_for_device(q->ppa->dev,
						q->q_buff_pbase, q->q_buff_size, DMA_FROM_DEVICE);

			q->num_desc = buff_size_by_q / ppa->max_packet_size;
			q->q_info_ptr->num_desc = q->num_desc;

			q->desc_ringbuf = ppa->desc_vbase +
					(i * sizeof(struct pktproc_desc_ringbuf) *
					 q->num_desc);
			q->cp_desc_pbase = ppa->cp_base + ppa->desc_rgn_offset +
					(i * sizeof(struct pktproc_desc_ringbuf) *
					 q->num_desc);
			q->q_info_ptr->cp_desc_pbase = q->cp_desc_pbase;
			q->desc_size = sizeof(struct pktproc_desc_ringbuf) * q->num_desc;

			q->get_packet = pktproc_get_pkt_from_ringbuf_mode;
			q->irq_handler = pktproc_irq_handler;
			break;
		case DESC_MODE_SKTBUF:
			q->manager = ppa->manager;
			if (q->manager) {
				q->q_buff_pbase = ppa->buff_pbase;
				q->q_buff_vbase = ppa->buff_vbase;
				q->cp_buff_pbase = ppa->cp_base + ppa->buff_rgn_offset;
				q->q_buff_size = buff_size;
				q->num_desc = (buff_size_by_q / q->manager->cell_size) *
						ppa->desc_num_ratio_percent / 100;
				q->alloc_rx_buf = pktproc_fill_data_addr;
				q->clear_data_addr = pktproc_clear_data_addr;
			} else {
				q->q_buff_pbase = ppa->buff_pbase + (i * buff_size_by_q);
				q->q_buff_vbase = ppa->buff_vbase + (i * buff_size_by_q);
				q->cp_buff_pbase = ppa->cp_base + ppa->buff_rgn_offset +
					(i * buff_size_by_q);
				q->q_buff_size = buff_size_by_q;
				q->num_desc = buff_size_by_q / ppa->max_packet_size;
				q->alloc_rx_buf = pktproc_fill_data_addr_without_bm;
				q->clear_data_addr = pktproc_clear_data_addr_without_bm;
			}
			q->q_info_ptr->cp_buff_pbase = q->cp_buff_pbase;
			q->q_info_ptr->num_desc = q->num_desc;

			q->desc_sktbuf = ppa->desc_vbase +
					(i * sizeof(struct pktproc_desc_sktbuf) *
					 q->num_desc);
			q->cp_desc_pbase = ppa->cp_base + ppa->desc_rgn_offset +
					(i * sizeof(struct pktproc_desc_sktbuf) *
					 q->num_desc);
			q->q_info_ptr->cp_desc_pbase = q->cp_desc_pbase;
			q->desc_size = sizeof(struct pktproc_desc_sktbuf) * q->num_desc;

			q->dma_addr = kzalloc(q->desc_size, GFP_KERNEL);
			if (!q->dma_addr) {
				mif_err("kzalloc() dma_addr failed\n");
				ret = -ENOMEM;
				goto create_error;
			}

			q->get_packet = pktproc_get_pkt_from_sktbuf_mode;
			q->irq_handler = pktproc_irq_handler;
			q->update_fore_ptr = pktproc_update_fore_ptr;
			break;
		default:
			mif_err("Unsupported version:%d\n", ppa->version);
			ret = -EINVAL;
			goto create_error;
		}

		if ((q->cp_desc_pbase + q->desc_size) > q->cp_buff_pbase) {
			mif_err("Descriptor overflow:0x%08x 0x%08x 0x%08x\n",
				q->cp_desc_pbase, q->desc_size, q->cp_buff_pbase);
			ret = -EINVAL;
			goto create_error;
		}

		spin_lock_init(&q->lock);

		q->clean_rx_ring = pktproc_clean_rx_ring;

		q->q_idx = i;
		q->mld = mld;
		q->ppa = ppa;

		/* NAPI */
		if (ppa->use_napi) {
			if (ppa->use_exclusive_irq) {
				init_dummy_netdev(&q->netdev);
				netif_napi_add(&q->netdev, &q->napi, pktproc_poll,
					NAPI_POLL_WEIGHT);
				napi_enable(&q->napi);
				q->napi_ptr = &q->napi;
			} else {
				q->napi_ptr = &q->mld->mld_napi;
			}
		}

		/* IRQ handler */
		q->enable_irq = pktproc_enable_irq;
		q->disable_irq = pktproc_disable_irq;
		if (ppa->use_exclusive_irq) {
			q->irq_idx = ppa->exclusive_irq_idx[q->q_idx];
#if IS_ENABLED(CONFIG_MCU_IPC)
			ret = cp_mbox_register_handler(q->irq_idx,
							mld->irq_cp2ap_msg, q->irq_handler, q);
			if (ret) {
				mif_err("cp_mbox_register_handler() error:%d\n", ret);
				goto create_error;
			}
#endif
#if IS_ENABLED(CONFIG_LINK_DEVICE_PCIE)
			/* Set by request_pcie_msi_int() */
#endif
		}

		q->q_info_ptr->fore_ptr = 0;
		q->q_info_ptr->rear_ptr = 0;

		q->fore_ptr = &q->q_info_ptr->fore_ptr;
		q->rear_ptr = &q->q_info_ptr->rear_ptr;
		q->done_ptr = *q->rear_ptr;

		mif_info("num_desc:%d cp_desc_pbase:0x%08x desc_size:0x%08x\n",
			q->num_desc, q->cp_desc_pbase, q->desc_size);
		mif_info("cp_buff_pbase:0x%08x buff_size:0x%08x\n",
			q->cp_buff_pbase, q->q_buff_size);
	}

#if IS_ENABLED(CONFIG_EXYNOS_DIT)
	ret = dit_set_buf_size(DIT_DIR_RX, ppa->max_packet_size);
	if (ret)
		mif_err("dit_set_buf_size() error:%d\n", ret);

	ret = dit_set_desc_ring_len(DIT_DIR_RX,
		ppa->q[DIT_PKTPROC_RX_QUEUE_NUM]->num_desc - 1);
	if (ret)
		mif_err("dit_set_desc_ring_len() error:%d\n", ret);
#endif

	/* Debug */
	ret = sysfs_create_group(&pdev->dev.kobj, &pktproc_group);
	if (ret != 0) {
		mif_err("sysfs_create_group() error %d\n", ret);
		goto create_error;
	}

	return 0;

create_error:
	for (i = 0; i < ppa->num_queue; i++) {
		kfree(ppa->q[i]->dma_addr);
		kfree(ppa->q[i]);
	}

	if (ppa->manager)
		exit_mif_buff_mng(ppa->manager);

	if (!ppa->buff_rgn_cached && ppa->buff_vbase)
		vunmap(ppa->buff_vbase);

	if (!ppa->info_desc_rgn_cached && ppa->info_vbase)
		vunmap(ppa->info_vbase);

	return ret;
}
