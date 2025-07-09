// SPDX-License-Identifier: GPL-2.0
#include "include/legacy.h"
#include "modem_utils.h"
#include "link_device.h"

/* sysfs */
static ssize_t region_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct modem_data *modem;
	ssize_t count = 0;

	modem = (struct modem_data *)dev->platform_data;

	count += scnprintf(&buf[count], PAGE_SIZE - count, "FMT offset head:0x%08X buff:0x%08X\n",
				modem->legacy_fmt_head_tail_offset,
				modem->legacy_fmt_buffer_offset);
	count += scnprintf(&buf[count], PAGE_SIZE - count, "FMT size txq:0x%08X rxq:0x%08X\n",
				modem->legacy_fmt_txq_size,
				modem->legacy_fmt_rxq_size);

	count += scnprintf(&buf[count], PAGE_SIZE - count, "RAW offset head:0x%08X buff:0x%08X\n",
				modem->legacy_raw_head_tail_offset,
				modem->legacy_raw_buffer_offset);
	count += scnprintf(&buf[count], PAGE_SIZE - count, "RAW size txq:0x%08X rxq:0x%08X\n",
				modem->legacy_raw_txq_size,
				modem->legacy_raw_rxq_size);

#if IS_ENABLED(CONFIG_MODEM_IF_LEGACY_QOS)
	count += scnprintf(&buf[count], PAGE_SIZE - count, "QoS offset head:0x%08X buff:0x%08X\n",
				modem->legacy_raw_qos_head_tail_offset,
				modem->legacy_raw_qos_buffer_offset);
	count += scnprintf(&buf[count], PAGE_SIZE - count, "QoS size txq:0x%08X rxq:0x%08X\n",
				modem->legacy_raw_qos_txq_size,
				modem->legacy_raw_qos_rxq_size);
#endif

	return count;
}

static ssize_t status_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct modem_data *modem;
	struct legacy_link_device *bl;
	struct legacy_ipc_device *ipc_dev;
	ssize_t count = 0;
	int i;

	modem = (struct modem_data *)dev->platform_data;
	bl = &modem->mld->legacy_link_dev;

	count += scnprintf(&buf[count], PAGE_SIZE - count, "magic:0x%08X mem_access:0x%08X\n",
				ioread32(bl->magic), ioread32(bl->mem_access));

	for (i = 0; i < IPC_MAP_MAX; i++) {
		ipc_dev = bl->dev[i];

		count += scnprintf(&buf[count], PAGE_SIZE - count, "\n");
		count += scnprintf(&buf[count], PAGE_SIZE - count, "ID:%d name:%s\n",
			i, ipc_dev->name);
		count += scnprintf(&buf[count], PAGE_SIZE - count, "TX busy:%d head:%d tail:%d\n",
			atomic_read(&ipc_dev->txq.busy), get_txq_head(ipc_dev),
			get_txq_tail(ipc_dev));
		count += scnprintf(&buf[count], PAGE_SIZE - count, "RX busy:%d head:%d tail:%d\n",
			atomic_read(&ipc_dev->rxq.busy), get_rxq_head(ipc_dev),
			get_rxq_tail(ipc_dev));
		count += scnprintf(&buf[count], PAGE_SIZE - count,
			"req_ack_cnt[TX]:%d req_ack_cnt[RX]:%d\n",
			ipc_dev->req_ack_cnt[TX], ipc_dev->req_ack_cnt[RX]);
	}

	return count;
}

static DEVICE_ATTR_RO(region);
static DEVICE_ATTR_RO(status);

static struct attribute *legacy_attrs[] = {
	&dev_attr_region.attr,
	&dev_attr_status.attr,
	NULL,
};

static const struct attribute_group legacy_group = {
	.attrs = legacy_attrs,
	.name = "legacy",
};

int create_legacy_link_device(struct mem_link_device *mld)
{
	struct legacy_ipc_device *dev;
	struct legacy_link_device *bl = &mld->legacy_link_dev;
	struct modem_data *modem = mld->link_dev.mdm_data;
	struct link_device *ld = &mld->link_dev;
	int ret = 0;

	bl->ld = &mld->link_dev;

	/* magic code and access enable fields */
	bl->magic = (u32 __iomem *)(mld->base);
	bl->mem_access = (u32 __iomem *)(mld->base + 4);

	/* IPC_MAP_FMT */
	bl->dev[IPC_MAP_FMT] = kzalloc(sizeof(struct legacy_ipc_device), GFP_KERNEL);
	dev = bl->dev[IPC_MAP_FMT];

	dev->id = IPC_MAP_FMT;
	strcpy(dev->name, "FMT");

	spin_lock_init(&dev->txq.lock);
	atomic_set(&dev->txq.busy, 0);
	dev->txq.head = (void __iomem *)(mld->base + modem->legacy_fmt_head_tail_offset);
	dev->txq.tail = (void __iomem *)(mld->base + modem->legacy_fmt_head_tail_offset + 4);
	dev->txq.buff = (void __iomem *)(mld->base + modem->legacy_fmt_buffer_offset);
	dev->txq.size = modem->legacy_fmt_txq_size;

	spin_lock_init(&dev->rxq.lock);
	atomic_set(&dev->rxq.busy, 0);
	dev->rxq.head = (void __iomem *)(mld->base + modem->legacy_fmt_head_tail_offset + 8);
	dev->rxq.tail = (void __iomem *)(mld->base + modem->legacy_fmt_head_tail_offset + 12);
	dev->rxq.buff = (void __iomem *)(mld->base + modem->legacy_fmt_buffer_offset +
			modem->legacy_fmt_txq_size);
	dev->rxq.size = modem->legacy_fmt_rxq_size;

	dev->msg_mask = MASK_SEND_FMT;
	dev->req_ack_mask = MASK_REQ_ACK_FMT;
	dev->res_ack_mask = MASK_RES_ACK_FMT;

	dev->skb_txq = kzalloc(sizeof(struct sk_buff_head), GFP_KERNEL);
	dev->skb_rxq = kzalloc(sizeof(struct sk_buff_head), GFP_KERNEL);
	skb_queue_head_init(dev->skb_txq);
	skb_queue_head_init(dev->skb_rxq);

	dev->req_ack_cnt[TX] = 0;
	dev->req_ack_cnt[RX] = 0;

	spin_lock_init(&dev->tx_lock);

#if IS_ENABLED(CONFIG_MODEM_IF_LEGACY_QOS)
	/* IPC_MAP_HPRIO_RAW */
	bl->dev[IPC_MAP_HPRIO_RAW] = kzalloc(sizeof(struct legacy_ipc_device), GFP_KERNEL);
	dev = bl->dev[IPC_MAP_HPRIO_RAW];

	dev->id = IPC_MAP_HPRIO_RAW;
	strcpy(dev->name, "HPRIO_RAW");

	spin_lock_init(&dev->txq.lock);
	atomic_set(&dev->txq.busy, 0);
	dev->txq.head = (void __iomem *)(mld->base + modem->legacy_raw_qos_head_tail_offset);
	dev->txq.tail = (void __iomem *)(mld->base + modem->legacy_raw_qos_head_tail_offset + 4);
	dev->txq.buff = (void __iomem *)(mld->hiprio_base);
	dev->txq.size = modem->legacy_raw_qos_txq_size;

	spin_lock_init(&dev->rxq.lock);
	atomic_set(&dev->rxq.busy, 0);
	dev->rxq.head = (void __iomem *)(mld->base + modem->legacy_raw_qos_head_tail_offset + 8);
	dev->rxq.tail = (void __iomem *)(mld->base + modem->legacy_raw_qos_head_tail_offset + 12);
	dev->rxq.buff = (void __iomem *)(mld->hiprio_base + modem->legacy_raw_qos_txq_size);
	dev->rxq.size = modem->legacy_raw_qos_rxq_size;

	dev->msg_mask = MASK_SEND_RAW;
	dev->req_ack_mask = MASK_REQ_ACK_RAW;
	dev->res_ack_mask = MASK_RES_ACK_RAW;

	dev->skb_txq = kzalloc(sizeof(struct sk_buff_head), GFP_KERNEL);
	dev->skb_rxq = kzalloc(sizeof(struct sk_buff_head), GFP_KERNEL);
	skb_queue_head_init(dev->skb_txq);
	skb_queue_head_init(dev->skb_rxq);

	dev->req_ack_cnt[TX] = 0;
	dev->req_ack_cnt[RX] = 0;

	spin_lock_init(&dev->tx_lock);
#endif

	/* IPC_MAP_NORM_RAW */
	bl->dev[IPC_MAP_NORM_RAW] = kzalloc(sizeof(struct legacy_ipc_device), GFP_KERNEL);
	dev = bl->dev[IPC_MAP_NORM_RAW];

	dev->id = IPC_MAP_NORM_RAW;
	strcpy(dev->name, "NORM_RAW");

	spin_lock_init(&dev->txq.lock);
	atomic_set(&dev->txq.busy, 0);
	dev->txq.head = (void __iomem *)(mld->base + modem->legacy_raw_head_tail_offset);
	dev->txq.tail = (void __iomem *)(mld->base + modem->legacy_raw_head_tail_offset + 4);
	dev->txq.buff = (void __iomem *)(mld->base + modem->legacy_raw_buffer_offset);
	dev->txq.size = modem->legacy_raw_txq_size;

	spin_lock_init(&dev->rxq.lock);
	atomic_set(&dev->rxq.busy, 0);
	dev->rxq.head = (void __iomem *)(mld->base + modem->legacy_raw_head_tail_offset + 8);
	dev->rxq.tail = (void __iomem *)(mld->base + modem->legacy_raw_head_tail_offset + 12);
	if (modem->legacy_raw_rx_buffer_cached)
		dev->rxq.buff =
			phys_to_virt(cp_shmem_get_base(bl->ld->mdm_data->cp_num, SHMEM_IPC) +
			modem->legacy_raw_buffer_offset + modem->legacy_raw_txq_size);
	else
		dev->rxq.buff = (void __iomem *)(mld->base + modem->legacy_raw_buffer_offset +
			modem->legacy_raw_txq_size);
	dev->rxq.size = modem->legacy_raw_rxq_size;

	dev->msg_mask = MASK_SEND_RAW;
	dev->req_ack_mask = MASK_REQ_ACK_RAW;
	dev->res_ack_mask = MASK_RES_ACK_RAW;

	dev->skb_txq = kzalloc(sizeof(struct sk_buff_head), GFP_KERNEL);
	dev->skb_rxq = kzalloc(sizeof(struct sk_buff_head), GFP_KERNEL);
	skb_queue_head_init(dev->skb_txq);
	skb_queue_head_init(dev->skb_rxq);

	dev->req_ack_cnt[TX] = 0;
	dev->req_ack_cnt[RX] = 0;

	spin_lock_init(&dev->tx_lock);

	/* sysfs */
	ret = sysfs_create_group(&ld->dev->kobj, &legacy_group);
	if (ret != 0) {
		mif_err("sysfs_create_group() error %d\n", ret);
		return ret;
	}

	return 0;
}

int init_legacy_link(struct legacy_link_device *bl)
{
	unsigned int magic;
	unsigned int mem_access;
	struct modem_data *modem = bl->ld->mdm_data;

	int i = 0;

	iowrite32(0, bl->magic);
	iowrite32(0, bl->mem_access);

	for (i = 0; i < IPC_MAP_MAX; i++) {
		struct legacy_ipc_device *dev = bl->dev[i];
		/* initialize circ_queues */
		iowrite32(0, dev->txq.head);
		iowrite32(0, dev->txq.tail);
		iowrite32(0, dev->rxq.head);
		iowrite32(0, dev->rxq.tail);

		/* initialize skb queues */
		skb_queue_purge(dev->skb_txq);
		atomic_set(&dev->txq.busy, 0);
		dev->req_ack_cnt[TX] = 0;
		skb_queue_purge(dev->skb_rxq);
		atomic_set(&dev->rxq.busy, 0);
		dev->req_ack_cnt[RX] = 0;

		if (modem->legacy_raw_rx_buffer_cached && i == IPC_MAP_NORM_RAW)
			dma_sync_single_for_device(bl->ld->dev, virt_to_phys(dev->rxq.buff),
					dev->rxq.size, DMA_FROM_DEVICE);
	}

	iowrite32(bl->ld->magic_ipc, bl->magic);
	iowrite32(1, bl->mem_access);

	magic = ioread32(bl->magic);
	mem_access = ioread32(bl->mem_access);
	if (magic != bl->ld->magic_ipc || mem_access != 1)
		return -EACCES;

	return 0;
}

int xmit_to_legacy_link(struct mem_link_device *mld, u8 ch,
			struct sk_buff *skb, enum legacy_ipc_map legacy_buffer_index)
{
	struct legacy_ipc_device *dev = mld->legacy_link_dev.dev[legacy_buffer_index];
	struct link_device *ld = &mld->link_dev;
	char *src = skb->data;
	char *dst = get_txq_buff(dev);
	unsigned int qsize = 0;
	unsigned int in = 0;
	unsigned int out = 0;
	unsigned int count = skb->len;
	int space = 0;
	int tried = 0;

	while (1) {
		qsize = get_txq_buff_size(dev);
		in = get_txq_head(dev);
		out = get_txq_tail(dev);

		/* is queue valid? */
		if (!circ_valid(qsize, in, out)) {
			mif_err("%s: ERR! Invalid %s_TXQ{qsize:%d in:%d out:%d}\n",
					ld->name, dev->name, qsize, in, out);
			return -EIO;
		}
		/* space available? */
		space = circ_get_space(qsize, in, out);
		if (unlikely(space < count)) {
			mif_err("%s: tried %d NOSPC %s_TX{qsize:%d in:%d out:%d free:%d len:%d}\n",
					ld->name, tried, dev->name, qsize, in, out, space, count);
			tried++;
			if (tried >= 20)
				return -ENOSPC;
			if (in_interrupt())
				mdelay(50);
			else
				msleep(50);
			continue;
		}

		barrier();

		circ_write(dst, src, qsize, in, count);

		barrier();

		set_txq_head(dev, circ_new_ptr(qsize, in, count));

		/* Commit the item before incrementing the head */
		smp_mb();

		break;

	}

#ifdef DEBUG_MODEM_IF_LINK_TX
	mif_pkt(ch, "LNK-TX", skb);
#endif

	dev_kfree_skb_any(skb);

	return count;
}


struct sk_buff *recv_from_legacy_link(struct mem_link_device *mld,
		struct legacy_ipc_device *dev, unsigned int in, int *ret)
{
	struct link_device *ld = &mld->link_dev;
	struct sk_buff *skb;
	char *src = get_rxq_buff(dev);
	unsigned int qsize = get_rxq_buff_size(dev);
	unsigned int out = get_rxq_tail(dev);
	unsigned int rest = circ_get_usage(qsize, in, out);
	unsigned int len;
	char hdr[EXYNOS_HEADER_SIZE];
	char pr_buff[BAD_MSG_BUFFER_SIZE];

	/* Make sure out pointer is within bounds. This pointer is read
	 * from CP shared memory and must be validated before
	 * circ_read below. */
	if (out > qsize) {
		mif_err("OOB err! out:%u, qsize:%u\n", out, qsize);
		if (ld->link_trigger_cp_crash) {
			ld->link_trigger_cp_crash(mld,
				CRASH_REASON_MIF_RX_BAD_DATA, "OOB error");
		}
		*ret = -EINVAL;
		goto no_mem;
	}

	/* Copy the header in a frame to the header buffer */
	switch (ld->protocol) {
	case PROTOCOL_SIPC:
		circ_read(hdr, src, qsize, out, SIPC5_MIN_HEADER_SIZE);
		break;
	case PROTOCOL_SIT:
		circ_read(hdr, src, qsize, out, EXYNOS_HEADER_SIZE);
		break;
	default:
		mif_err("procotol error %d\n", ld->protocol);
		break;
	}

	/* Check the config field in the header */
	if (unlikely(!ld->is_start_valid(hdr))) {
		mif_err("%s: ERR! %s BAD CFG 0x%02X (in:%d out:%d rest:%d)\n",
			ld->name, dev->name, hdr[SIPC5_CONFIG_OFFSET],
			in, out, rest);
		goto bad_msg;
	}

	/* Verify the length of the frame (data + padding) */
	len = ld->get_total_len(hdr);
	if (unlikely(len > rest)) {
		mif_err("%s: ERR! %s BAD LEN %d > rest %d\n",
			ld->name, dev->name, len, rest);
		goto bad_msg;
	}

	/* Allocate an skb */
	skb = mem_alloc_skb(len);
	if (!skb) {
		mif_err("%s: ERR! %s mem_alloc_skb(%d) fail\n",
			ld->name, dev->name, len);
		*ret = -ENOMEM;
		goto no_mem;
	}

	/* Read the frame from the RXQ */
	circ_read(skb_put(skb, len), src, qsize, out, len);

	/* Update tail (out) pointer to the frame to be read in the future */
	set_rxq_tail(dev, circ_new_ptr(qsize, out, len));

	/* Finish reading data before incrementing tail */
	smp_mb();

	/* Record the time-stamp */
	ktime_get_ts64(&skbpriv(skb)->ts);

	return skb;

bad_msg:
	mif_err("%s%s%s: ERR! BAD MSG: %02x %02x %02x %02x\n",
		ld->name, arrow(RX), ld->mc->name,
		hdr[0], hdr[1], hdr[2], hdr[3]);

	circ_read(pr_buff, src, qsize, out, BAD_MSG_BUFFER_SIZE);
	pr_buffer("BAD MSG", (char *)pr_buff, (size_t)BAD_MSG_BUFFER_SIZE,
			(size_t)BAD_MSG_BUFFER_SIZE);

	set_rxq_tail(dev, in);	/* Reset tail (out) pointer */
	if (ld->link_trigger_cp_crash) {
		ld->link_trigger_cp_crash(mld, CRASH_REASON_MIF_RX_BAD_DATA,
					"ERR! BAD MSG from CP");
	}
	*ret = -EINVAL;

no_mem:
	return NULL;
}

bool check_legacy_tx_pending(struct mem_link_device *mld)
{
	int i;
	unsigned int head, tail;
	struct legacy_ipc_device *dev;

	for (i = IPC_MAP_FMT ; i < IPC_MAP_MAX ; i++) {
		dev = mld->legacy_link_dev.dev[i];
		head = get_txq_head(dev);
		tail = get_txq_tail(dev);

		if (head != tail) {
			mif_info("idx: %d, head: %u, tail: %u", i, head, tail);
			return true;
		}
	}

	return false;
}

