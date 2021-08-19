// SPDX-License-Identifier: GPL-2.0
/* @file		link_device_memory_main.c
 * @brief		common functions for all types of memory interface media
 * @date		2014/02/05
 * @author		Hankook Jang (hankook.jang@samsung.com)
 */

/*
 * Copyright (C) 2011 Samsung Electronics.
 *
 */

#include <soc/google/exynos-modem-ctrl.h>
#include "modem_prj.h"
#include "modem_utils.h"
#include "link_device_memory.h"
#include "include/sbd.h"
#include <linux/shm_ipc.h>

static void print_sbd_config(struct sbd_link_device *sl)
{
#ifdef DEBUG_MODEM_IF
	int i, dir;
	struct sbd_rb_channel *rb_ch;
	struct sbd_rb_desc *rbd;

	mif_info("SBD_IPC {shmem_base:0x%pK shmem_size:%d}\n",
		sl->shmem, sl->shmem_size);

	mif_info("SBD_IPC {version:%d num_channels:%d rpwp_array_offset:%d}\n",
		sl->g_desc->version, sl->g_desc->num_channels,
		sl->g_desc->rpwp_array_offset);

	for (i = 0; i < sl->num_channels; i++) {
		for (dir = 0; dir < ULDL; dir++) {
			rb_ch = &sl->g_desc->rb_ch[i][dir];
			rbd = &sl->g_desc->rb_desc[i][dir];

			mif_info("RB_DESC[%-2d][%s](offset:%d) = {id:%-2d ch:%-3d dir:%s} {buff_pos_array_offset:%-5d rb_len:%-3d} {buff_size:%-4d payload_offset:%d}\n",
				i, udl_str(dir), rb_ch->rb_desc_offset,
				rbd->id, rbd->ch, udl_str(rbd->direction),
				rb_ch->buff_pos_array_offset, rbd->length,
				rbd->buff_size, rbd->payload_offset);
		}
	}
#endif
}

/* sysfs */
static ssize_t region_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct modem_data *modem;
	struct sbd_link_device *sl;
	struct sbd_rb_channel *rb_ch;
	struct sbd_rb_desc *rbd;
	struct sbd_link_attr *link_attr;
	struct sbd_ring_buffer *rb;
	int i, dir;
	ssize_t count = 0;

	modem = (struct modem_data *)dev->platform_data;
	sl = &modem->mld->sbd_link_dev;

	count += scnprintf(&buf[count], PAGE_SIZE - count,
		"version:%d num_channels:%d rpwp_array_offset:%d\n",
		sl->g_desc->version, sl->g_desc->num_channels, sl->g_desc->rpwp_array_offset);

	for (i = 0; i < sl->num_channels; i++) {
		for (dir = 0; dir < ULDL; dir++) {
			count += scnprintf(&buf[count], PAGE_SIZE - count, "\n");

			rb = sbd_id2rb(sl, i, dir);
			rb_ch = &sl->g_desc->rb_ch[i][dir];
			rbd = &sl->g_desc->rb_desc[i][dir];
			link_attr = &sl->link_attr[i];

			count += scnprintf(&buf[count], PAGE_SIZE - count,
				"ID:%d CH:%d direction:%s\n", rbd->id, rbd->ch, udl_str(dir));
			count += scnprintf(&buf[count], PAGE_SIZE - count,
				"DESC region offset:%d buff_pos:%d buff_size:%d len:%d\n",
				rb_ch->rb_desc_offset, rb_ch->buff_pos_array_offset,
				rbd->buff_size, rbd->length);
			count += scnprintf(&buf[count], PAGE_SIZE - count,
				"BUFF region offset:%d size:%u\n",
				calc_offset(rb->buff_rgn, sl->shmem), (rb->len * rb->buff_size));
		}
	}

	return count;
}

static ssize_t status_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct modem_data *modem;
	struct sbd_link_device *sl;
	struct sbd_ring_buffer *rb_tx, *rb_rx;
	int i;
	ssize_t count = 0;

	modem = (struct modem_data *)dev->platform_data;
	sl = &modem->mld->sbd_link_dev;

	count += scnprintf(&buf[count], PAGE_SIZE - count, "num_channels:%d\n", sl->num_channels);
	for (i = 0; i < sl->num_channels; i++) {
		rb_tx = sbd_id2rb(sl, i, TX);
		rb_rx = sbd_id2rb(sl, i, RX);

		count += scnprintf(&buf[count], PAGE_SIZE - count, "\n");
		if (!rb_tx->len && !rb_rx->len) {
			count += scnprintf(&buf[count], PAGE_SIZE - count,
				"ID:%d TX(empty) RX(empty)\n", i);
			continue;
		}

		count += scnprintf(&buf[count], PAGE_SIZE - count, "ID:%d name:%s\n",
			i, rb_tx->iod->name);
		count += scnprintf(&buf[count], PAGE_SIZE - count,
			"TX ch:%d len:%d buff_size:%d rp:%d wp:%d space:%d usage:%d\n",
			rb_tx->ch, rb_tx->len, rb_rx->buff_size,
			*rb_tx->rp, *rb_tx->wp, rb_space(rb_tx) + 1, rb_usage(rb_tx));

		if (rb_rx->zerocopy)
			count += scnprintf(&buf[count], PAGE_SIZE - count,
				"RX ch:%d len:%d buff_size:%d rp:%d pre_rp:%d wp:%d, space:%d usage:%d\n",
				rb_rx->ch, rb_rx->len, rb_rx->buff_size,
				*rb_rx->rp, rb_rx->zdptr->pre_rp,
				*rb_rx->wp, rb_space(rb_rx) + 1, rb_usage(rb_rx));
		else
			count += scnprintf(&buf[count], PAGE_SIZE - count,
				"RX ch:%d len:%d buff_size:%d rp:%d wp:%d, space:%d usage:%d\n",
				rb_rx->ch, rb_rx->len, rb_rx->buff_size,
				*rb_rx->rp, *rb_rx->wp,
				rb_space(rb_rx) + 1, rb_usage(rb_rx));
	}

	return count;
}

static DEVICE_ATTR_RO(region);
static DEVICE_ATTR_RO(status);

static struct attribute *sbd_attrs[] = {
	&dev_attr_region.attr,
	&dev_attr_status.attr,
	NULL,
};

static const struct attribute_group sbd_group = {
	.attrs = sbd_attrs,
	.name = "sbd",
};

static void setup_link_attr(struct sbd_link_attr *link_attr, u16 id, u16 ch,
			    struct modem_io_t *io_dev)
{
	link_attr->id = id;
	link_attr->ch = ch;

	if (io_dev->attrs & IO_ATTR_NO_LINK_HEADER)
		link_attr->lnk_hdr = false;
	else
		link_attr->lnk_hdr = true;

	link_attr->rb_len[UL] = io_dev->ul_num_buffers;
	link_attr->buff_size[UL] = io_dev->ul_buffer_size;
	link_attr->rb_len[DL] = io_dev->dl_num_buffers;
	link_attr->buff_size[DL] = io_dev->dl_buffer_size;

#if IS_ENABLED(CONFIG_CP_ZEROCOPY)
	if (io_dev->attrs & IO_ATTR_ZEROCOPY)
		link_attr->zerocopy = true;
	else
		link_attr->zerocopy = false;
#endif

}

/*
 * @return		the number of actual link channels
 */
static unsigned int init_ctrl_tables(struct sbd_link_device *sl)
{
	struct modem_io_t **iodevs = sl->ld->mdm_data->iodevs;
	unsigned int i, ch;
	int multi_raw_count = 0;
	unsigned int id = 0;

	/*
	 * Fill ch2id array with MAX_SBD_LINK_IDS value to prevent sbd_ch2id()
	 * from returning 0 for unused channels.
	 */
	for (i = 0; i < MAX_SBD_SIPC_CHANNELS; i++)
		sl->ch2id[i] = MAX_SBD_LINK_IDS;

	for (i = 0; i < sl->ld->mdm_data->num_iodevs; i++) {
		ch = iodevs[i]->ch;

		/* Skip non-IPC or PS ch but allow IPC_MULTI_RAW */
		if ((!sipc5_ipc_ch(ch) || sipc_ps_ch(ch)) &&
		    (iodevs[i]->format != IPC_MULTI_RAW))
			continue;

		/* Skip making rb if mismatch region info */
		if ((iodevs[i]->attrs & IO_ATTR_OPTION_REGION) &&
		    strcmp(iodevs[i]->option_region, CONFIG_OPTION_REGION))
			continue;

		/* Change channel to QoS priority */
		if (iodevs[i]->format == IPC_MULTI_RAW) {
			ch = QOS_HIPRIO + multi_raw_count;
			multi_raw_count++;
			if (ch >= QOS_MAX_PRIO) {
				mif_err("IPC_MULTI_RAW over max count ch: %d\n", ch);
				continue;
			}
		}

		/* Save CH# to LinkID-to-CH conversion table. */
		sl->id2ch[id] = ch;

		/* Save LinkID to CH-to-LinkID conversion table. */
		sl->ch2id[ch] = id;

		/* Set up the attribute table entry of a LinkID. */
		setup_link_attr(&sl->link_attr[id], id, ch, iodevs[i]);

		id++;
	}

#if !IS_ENABLED(CONFIG_MODEM_IF_QOS)
	for (i = 0; i < sl->ld->mdm_data->num_iodevs; i++) {
		int ch = iodevs[i]->ch;

		if (sipc_ps_ch(ch))
			sl->ch2id[ch] = sl->ch2id[QOS_HIPRIO];
	}
#endif

	/* Finally, id has the number of actual link channels. */
	return id;
}

int init_sbd_link(struct sbd_link_device *sl)
{
	int i, dir, idx;
	int ret = 0;
	struct sbd_ring_buffer *rb;
	struct sbd_ipc_device *ipc_dev;
	struct sbd_link_attr *link_attr;

	if (unlikely(!sl))
		return -ENOMEM;

	memset(sl->shmem + DESC_RGN_OFFSET, 0x0, DESC_RGN_SIZE);

	sl->g_desc->version = sl->version;
	sl->g_desc->num_channels = sl->num_channels;
	sl->g_desc->rpwp_array_offset = calc_offset(sl->rp[UL], sl->shmem);

	for (i = 0; i < sl->num_channels; i++) {
		ipc_dev = sbd_id2dev(sl, i);
		if (unlikely(!ipc_dev))
			return -ENODEV;

		link_attr = &sl->link_attr[i];

		ipc_dev->id = link_attr->id;
		ipc_dev->ch = link_attr->ch;
		atomic_set(&ipc_dev->config_done, 0);
		ipc_dev->zerocopy = link_attr->zerocopy;

		for (dir = 0; dir < ULDL; dir++) {
			/*
			 * Setup UL Ring Buffer in the ipc_dev[$i]
			 */
			rb = &ipc_dev->rb[dir];

			rb->sl = sl;
			rb->lnk_hdr = link_attr->lnk_hdr;
			rb->zerocopy = link_attr->zerocopy;
			rb->more = false;
			rb->total = 0;
			rb->rcvd = 0;

			/*
			 * Initialize an SBD RB instance in the kernel space.
			 */
			rb->id = link_attr->id;
			rb->ch = link_attr->ch ?: SIPC_CH_ID_PDP_0;
			rb->dir = dir;
			rb->len = link_attr->rb_len[dir];
			rb->buff_size = link_attr->buff_size[dir];
			rb->payload_offset = 0;

			for (idx = 0; idx < rb->len; idx++) {
				rb->buff_pos_array[idx] = calc_offset(rb->buff[idx], sl->shmem);
				rb->buff_len_array[idx] = 0;
			}

			rb->iod = link_get_iod_with_channel(sl->ld, rb->ch);
			rb->ld = sl->ld;
			atomic_set(&rb->busy, 0);

			/*
			 * Setup RB_DESC in the g_desc
			 */
			rb->rb_desc->ch = rb->ch;
			rb->rb_desc->direction = rb->dir;
			rb->rb_desc->signaling = 1;
			rb->rb_desc->sig_mask = MASK_INT_VALID | MASK_SEND_DATA;
			rb->rb_desc->length = rb->len;
			rb->rb_desc->id = rb->id;
			rb->rb_desc->buff_size = rb->buff_size;
			rb->rb_desc->payload_offset = rb->payload_offset;

			/*
			 * Setup RB_CH in the g_desc
			 */
			rb->rb_ch->rb_desc_offset = calc_offset(rb->rb_desc, sl->shmem);
			rb->rb_ch->buff_pos_array_offset =
				calc_offset(rb->buff_pos_array, sl->shmem);
		}

#if IS_ENABLED(CONFIG_CP_ZEROCOPY)
		/*
		 * Setup zerocopy_adaptor if zerocopy ipc_dev
		 */
		ret = setup_zerocopy_adaptor(ipc_dev);
		if (ret < 0)
			return ret;
#endif
	}

	print_sbd_config(sl);

	return ret;
}

int create_sbd_mem_map(struct sbd_link_device *sl)
{
	int i, dir;
	struct sbd_ring_buffer *rb;
	struct sbd_ipc_device *ipc_dev;
	struct sbd_link_attr *link_attr;

	u8 *mem_global_desc;
	u16 *mem_rb_rpwp;
	u8 *mem_rb_cell_info;
	u8 *mem_rb_buff;

	u8 *desc_addr = sl->shmem + DESC_RGN_OFFSET;
	u8 *buff_addr = sl->shmem + BUFF_RGN_OFFSET;

	unsigned int idx;
	unsigned int rb_len;
	unsigned int rb_buff_size;

	mem_global_desc = desc_addr;
	desc_addr += sizeof(struct sbd_global_desc);

	mem_rb_rpwp = (u16 *)desc_addr;
	desc_addr += sizeof(u16) * ULDL * RDWR * sl->num_channels;

	mem_rb_cell_info = desc_addr;
	mem_rb_buff = buff_addr;

	sl->g_desc = (struct sbd_global_desc *)mem_global_desc;

	sl->rp[UL] = mem_rb_rpwp + (sl->num_channels * 0);
	sl->wp[UL] = mem_rb_rpwp + (sl->num_channels * 1);
	sl->rp[DL] = mem_rb_rpwp + (sl->num_channels * 2);
	sl->wp[DL] = mem_rb_rpwp + (sl->num_channels * 3);

	for (i = 0; i < sl->num_channels; i++) {
		ipc_dev = sbd_id2dev(sl, i);
		if (unlikely(!ipc_dev))
			return -ENODEV;

		link_attr = &sl->link_attr[i];

		for (dir = 0; dir < ULDL; dir++) {
			rb = &ipc_dev->rb[dir];

			rb_len = link_attr->rb_len[dir];
			rb_buff_size = link_attr->buff_size[dir];

			rb->buff_pos_array = (u32 *)desc_addr;
			desc_addr += rb_len * sizeof(u32);

			rb->buff_len_array = (u32 *)desc_addr;
			desc_addr += rb_len * sizeof(u32);

			rb->buff_rgn = buff_addr;
			buff_addr += (rb_len * rb_buff_size);

			if (!rb->buff)
				rb->buff = kmalloc((rb_len * sizeof(u8 *)), GFP_ATOMIC);
			if (!rb->buff)
				return -ENOMEM;

			for (idx = 0; idx < rb_len; idx++)
				rb->buff[idx] = rb->buff_rgn + (idx * rb_buff_size);

			mif_err("RB[%d:%d][%s] buff_rgn {addr:0x%pK offset:%d size:%u}\n",
				i, sbd_id2ch(sl, i), udl_str(dir), rb->buff_rgn,
				calc_offset(rb->buff_rgn, sl->shmem), (rb_len * rb_buff_size));

#if IS_ENABLED(CONFIG_SBD_BOOTLOG)
			if (rb->buff_rgn + (rb_len * rb_buff_size) >=
				sl->shmem + sl->shmem_size - SHMEM_BOOTSBDLOG_SIZE) {
				mif_err("sbd buffer break boot log area\n");
				return -ENOMEM;
			}
#endif

			rb->rp = &sl->rp[dir][i];
			rb->wp = &sl->wp[dir][i];

			rb->rb_ch = &sl->g_desc->rb_ch[i][dir];
			rb->rb_desc = &sl->g_desc->rb_desc[i][dir];

			spin_lock_init(&rb->lock);
			skb_queue_head_init(&rb->skb_q);
		}
	}

	if (desc_addr > (sl->shmem + DESC_RGN_OFFSET + DESC_RGN_SIZE))
		mif_err("SBD Desc overflow offset: 0x%pK\n", desc_addr);

	if (buff_addr > (sl->shmem  + sl->shmem_size))
		mif_err("SBD Buffer overflow offset: 0x%pK\n", buff_addr);

	return 0;
}

int create_sbd_link_device(struct link_device *ld, struct sbd_link_device *sl,
			   u8 *shmem_base, unsigned int shmem_size)
{
	int ret;

	if (!ld || !sl || !shmem_base)
		return -EINVAL;

	if (!ld->mdm_data)
		return -EINVAL;

	sl->ld = ld;

	sl->version = 1;

	sl->shmem = shmem_base;
	sl->shmem_size = shmem_size;
	sl->zmb_offset = shmem_size;

	sl->num_channels = init_ctrl_tables(sl);
	sl->reset_zerocopy_done = 1;

	ret = create_sbd_mem_map(sl);
	if (ret < 0) {
		mif_err("Can't create SBD memory map\n");
		return ret;
	}

	mif_info("SHMEM {base:0x%pK size:%d}\n",
		 sl->shmem, sl->shmem_size);

	mif_info("G_DESC_OFFSET = %d(0x%pK)\n",
		 calc_offset(sl->g_desc, sl->shmem),
		 sl->g_desc);

	mif_info("RB_CH_OFFSET = %d (0x%pK)\n",
		 calc_offset(sl->g_desc->rb_ch, sl->shmem),
		 sl->g_desc->rb_ch);

	mif_info("RBD_PAIR_OFFSET = %d (0x%pK)\n",
		 calc_offset(sl->g_desc->rb_desc, sl->shmem),
		 sl->g_desc->rb_desc);

	/* sysfs */
	ret = sysfs_create_group(&sl->ld->dev->kobj, &sbd_group);
	if (ret != 0) {
		mif_err("sysfs_create_group() sbd_group error %d\n", ret);
		return ret;
	}

#if IS_ENABLED(CONFIG_CP_ZEROCOPY)
	ret = sysfs_create_group(&sl->ld->dev->kobj, &zerocopy_group);
	if (ret != 0) {
		mif_err("sysfs_create_group() zerocopy_group error %d\n", ret);
		return ret;
	}
#endif

	mif_info("Complete!!\n");

	return 0;
}

static inline int check_rb_space(struct sbd_ring_buffer *rb, unsigned int qlen,
				 unsigned int in, unsigned int out)
{
	unsigned int space;

	if (!circ_valid(qlen, in, out)) {
		mif_err("ERR! TXQ[%d:%d] DIRTY (qlen:%d in:%d out:%d)\n",
			rb->id, rb->ch, qlen, in, out);
		return -EIO;
	}

	space = circ_get_space(qlen, in, out);
	if (unlikely(space < 1)) {
		mif_err_limited("TXQ[%d:%d] NOSPC (qlen:%d in:%d out:%d)\n",
				rb->id, rb->ch, qlen, in, out);
		return -ENOSPC;
	}

	return space;
}

int sbd_pio_tx(struct sbd_ring_buffer *rb, struct sk_buff *skb)
{
	int ret;
	unsigned int qlen = rb->len;
	unsigned int in = *rb->wp;
	unsigned int out = *rb->rp;
	unsigned int count = skb->len;
	unsigned int space = (rb->buff_size - rb->payload_offset);
	u8 *dst;

	ret = check_rb_space(rb, qlen, in, out);
	if (unlikely(ret < 0))
		return ret;

	if (unlikely(count > space)) {
		mif_err("ERR! {id:%d ch:%d} count %d > space %d\n",
			rb->id, rb->ch, count, space);
		return -ENOSPC;
	}

	barrier();

	dst = rb->buff[in] + rb->payload_offset;

	barrier();

	skb_copy_from_linear_data(skb, dst, count);

	if (sipc_ps_ch(rb->ch)) {
		struct io_device *iod = skbpriv(skb)->iod;
		unsigned int ch = iod->ch;

		rb->buff_len_array[in] = (skb->len & 0xFFFF);
		rb->buff_len_array[in] |= (ch << 16);
	} else {
		rb->buff_len_array[in] = skb->len;
	}

	barrier();

	*rb->wp = circ_new_ptr(qlen, in, 1);

	/* Commit the item before incrementing the head */
	smp_mb();

	return count;
}

bool check_sbd_tx_pending(struct mem_link_device *mld)
{
	int i;
	unsigned int wp, rp;
	struct sbd_link_device *sl = &mld->sbd_link_dev;

	for (i = 0; i < sl->num_channels; i++) {
		wp = sl->wp[UL][i];
		rp = sl->rp[UL][i];

		if (wp != rp) {
			mif_info("ch: %d, wp: %u, rp: %u", sbd_id2ch(sl, i), wp, rp);
			return true;
		}
	}

	return false;
}

static inline struct sk_buff *recv_data(struct sbd_ring_buffer *rb, u16 out)
{
	struct sk_buff *skb;
	u8 *src;
	unsigned int len = rb->buff_len_array[out] & 0xFFFF;
	unsigned int space = (rb->buff_size - rb->payload_offset);

	if (unlikely(len > space)) {
		mif_err("ERR! {id:%d ch:%d} size %d > space %d\n",
			rb->id, rb->ch, len, space);
		return NULL;
	}

	skb = dev_alloc_skb(len);
	if (unlikely(!skb)) {
		mif_err("ERR! {id:%d ch:%d} alloc_skb(%d) fail\n",
			rb->id, rb->ch, len);
		return NULL;
	}

	src = rb->buff[out] + rb->payload_offset;
	skb_put(skb, len);
	skb_copy_to_linear_data(skb, src, len);

	return skb;
}

static inline void set_skb_priv(struct sbd_ring_buffer *rb, struct sk_buff *skb)
{
	struct link_device *ld = rb->ld;
	struct mem_link_device *mld = ld_to_mem_link_device(ld);
	unsigned int out = *rb->rp;

	/* Record the IO device, the link device, etc. into &skb->cb */
	if (sipc_ps_ch(rb->ch)) {
		unsigned int ch = (rb->buff_len_array[out] >> 16) & 0xffff;

		skbpriv(skb)->iod = link_get_iod_with_channel(rb->ld, ch);
		skbpriv(skb)->ld = rb->ld;
		skbpriv(skb)->sipc_ch = ch;
		skbpriv(skb)->napi = &mld->mld_napi;
	} else {
		skbpriv(skb)->iod = rb->iod;
		skbpriv(skb)->ld = rb->ld;
		skbpriv(skb)->sipc_ch = rb->ch;
		skbpriv(skb)->napi = NULL;
	}
}

struct sk_buff *sbd_pio_rx(struct sbd_ring_buffer *rb)
{
	struct sk_buff *skb;
	unsigned int qlen = rb->len;
	unsigned int out = *rb->rp;

	skb = recv_data(rb, out);
	if (unlikely(!skb))
		return NULL;

	set_lnk_hdr(rb, skb);

	set_skb_priv(rb, skb);

	check_more(rb, skb);

	*rb->rp = circ_new_ptr(qlen, out, 1);

	return skb;
}
