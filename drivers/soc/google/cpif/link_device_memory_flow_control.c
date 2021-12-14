// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2011 Samsung Electronics.
 *
 */

#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/vmalloc.h>
#include <linux/netdevice.h>

#include "modem_prj.h"
#include "modem_utils.h"
#include "link_device_memory.h"

#if IS_ENABLED(CONFIG_CP_PKTPROC_UL)
void pktproc_ul_q_stop(struct pktproc_queue_ul *q)
{
	struct link_device *ld = &q->mld->link_dev;
	unsigned long flags;

	spin_lock_irqsave(&ld->netif_lock, flags);
	if (!atomic_read(&q->busy)) {
		mif_info("Requested stop on PKTPROC UL QUEUE %d\n", q->q_idx);
		atomic_set(&q->busy, 1);
		stop_net_ifaces(ld, TXQ_STOP_MASK);
	}
	spin_unlock_irqrestore(&ld->netif_lock, flags);
}

static void pktproc_ul_q_start(struct pktproc_queue_ul *q)
{
	struct link_device *ld = &q->mld->link_dev;
	unsigned long flags;

	mif_info("Requested start on PKTPROC UL QUEUE %d\n", q->q_idx);

	spin_lock_irqsave(&ld->netif_lock, flags);
	atomic_set(&q->busy, 0);
	resume_net_ifaces(ld, TXQ_STOP_MASK);
	spin_unlock_irqrestore(&ld->netif_lock, flags);
}

int pktproc_ul_q_check_busy(struct pktproc_queue_ul *q)
{
	struct link_device *ld = &q->mld->link_dev;
	int busy_count;
	unsigned long flags;

	spin_lock_irqsave(&ld->netif_lock, flags);
	busy_count = atomic_read(&q->busy);
	if (unlikely(busy_count))
		atomic_inc(&q->busy);
	spin_unlock_irqrestore(&ld->netif_lock, flags);

	if (!busy_count)
		return 0;

	spin_lock_irqsave(&q->lock, flags);
	if (pktproc_ul_q_empty(q->q_info)) {
		spin_unlock_irqrestore(&q->lock, flags);
#ifdef DEBUG_MODEM_IF_FLOW_CTRL
		if (cp_online(ld->mc))
			mif_err("PKTPROC UL Queue %d: EMPTY (busy_cnt %d)\n", q->q_idx, busy_count);
#endif
		pktproc_ul_q_start(q);
		return 0;
	}
	spin_unlock_irqrestore(&q->lock, flags);

	return -EBUSY;
}
#endif /* CONFIG_CP_PKTPROC_UL */

void sbd_txq_stop(struct sbd_ring_buffer *rb)
{
	struct link_device *ld = rb->ld;
	unsigned long flags;

	if (!ld->is_ps_ch(rb->ch))
		return;

	spin_lock_irqsave(&ld->netif_lock, flags);
	if (!atomic_read(&rb->busy)) {
		mif_info("Requested stop on rb ch: %d name: %s\n", rb->ch, rb->iod->name);
		atomic_set(&rb->busy, 1);
		stop_net_ifaces(ld, TXQ_STOP_MASK);
	}
	spin_unlock_irqrestore(&ld->netif_lock, flags);
}

static void sbd_txq_start(struct sbd_ring_buffer *rb)
{
	struct link_device *ld = rb->ld;
	unsigned long flags;

	if (!ld->is_ps_ch(rb->ch))
		return;

	mif_info("Requested start on rb ch: %d name: %s\n", rb->ch, rb->iod->name);

	spin_lock_irqsave(&ld->netif_lock, flags);
	atomic_set(&rb->busy, 0);
	resume_net_ifaces(ld, TXQ_STOP_MASK);
	spin_unlock_irqrestore(&ld->netif_lock, flags);
}

int sbd_txq_check_busy(struct sbd_ring_buffer *rb)
{
	struct link_device *ld = rb->ld;
	int busy_count;
	unsigned long flags;

	spin_lock_irqsave(&ld->netif_lock, flags);
	busy_count = atomic_read(&rb->busy);
	if (unlikely(busy_count))
		atomic_inc(&rb->busy);
	spin_unlock_irqrestore(&ld->netif_lock, flags);

	if (!busy_count)
		return 0;

	if (rb_empty(rb)) {
#ifdef DEBUG_MODEM_IF_FLOW_CTRL
		if (cp_online(ld->mc))
			mif_err("%s TXQ: EMPTY (busy_cnt %d)\n", rb->iod->name, busy_count);
#endif
		sbd_txq_start(rb);
		return 0;
	}

	return -EBUSY;
}

void txq_stop(struct mem_link_device *mld, struct legacy_ipc_device *dev)
{
	struct link_device *ld = &mld->link_dev;
	unsigned long flags;
	bool ret = false;

	if (dev->id == IPC_MAP_FMT)
		return;

	spin_lock_irqsave(&ld->netif_lock, flags);
	if (!atomic_read(&dev->txq.busy)) {
		mif_info("Requested stop on dev: %s\n", dev->name);
		atomic_set(&dev->txq.busy, 1);
		ret = stop_net_ifaces(ld, TXQ_STOP_MASK);
	}
	spin_unlock_irqrestore(&ld->netif_lock, flags);

	if (ret) {
		/* notify cp that legacy buffer is stuck. required for legacy only */
		send_req_ack(mld, dev);
	}
}

static void txq_start(struct mem_link_device *mld, struct legacy_ipc_device *dev)
{
	struct link_device *ld = &mld->link_dev;
	unsigned long flags;

	if (dev->id == IPC_MAP_FMT)
		return;

	mif_info("Requested start on dev: %s\n", dev->name);

	spin_lock_irqsave(&ld->netif_lock, flags);
	atomic_set(&dev->txq.busy, 0);
	resume_net_ifaces(ld, TXQ_STOP_MASK);
	spin_unlock_irqrestore(&ld->netif_lock, flags);
}

int txq_check_busy(struct mem_link_device *mld, struct legacy_ipc_device *dev)
{
	struct link_device *ld = &mld->link_dev;
	struct modem_ctl *mc = ld->mc;
	int busy_count;
	unsigned long flags;

	spin_lock_irqsave(&ld->netif_lock, flags);
	busy_count = atomic_read(&dev->txq.busy);
	if (unlikely(busy_count))
		atomic_inc(&dev->txq.busy);
	spin_unlock_irqrestore(&ld->netif_lock, flags);

	if (!busy_count)
		return 0;

	if (txq_empty(dev)) {
#ifdef DEBUG_MODEM_IF_FLOW_CTRL
		if (cp_online(mc)) {
			mif_err("%s->%s: %s_TXQ: No RES_ACK, but EMPTY (busy_cnt %d)\n",
				ld->name, mc->name, dev->name, busy_count);
		}
#endif
		txq_start(mld, dev);
		return 0;
	}

	if (cp_online(mc) && count_flood(busy_count, BUSY_COUNT_MASK)) {
		/* notify cp that legacy buffer is stuck. required for legacy only */
		send_req_ack(mld, dev);
		return -ETIME;
	}

	return -EBUSY;
}

void tx_flowctrl_suspend(struct mem_link_device *mld)
{
	struct link_device *ld = &mld->link_dev;
	unsigned long flags;

	spin_lock_irqsave(&ld->netif_lock, flags);
	stop_net_ifaces(ld, TX_SUSPEND_MASK);
	spin_unlock_irqrestore(&ld->netif_lock, flags);
}

void tx_flowctrl_resume(struct mem_link_device *mld)
{
	struct link_device *ld = &mld->link_dev;
	unsigned long flags;

	spin_lock_irqsave(&ld->netif_lock, flags);
	resume_net_ifaces(ld, TX_SUSPEND_MASK);
	spin_unlock_irqrestore(&ld->netif_lock, flags);
}

void send_req_ack(struct mem_link_device *mld, struct legacy_ipc_device *dev)
{
#ifdef DEBUG_MODEM_IF_FLOW_CTRL
	struct mst_buff *msb;
#endif

	send_ipc_irq(mld, mask2int(req_ack_mask(dev)));
	dev->req_ack_cnt[TX] += 1;

#ifdef DEBUG_MODEM_IF_FLOW_CTRL
	msb = mem_take_snapshot(mld, TX);
	if (!msb)
		return;
	print_req_ack(mld, &msb->snapshot, dev, TX);
	msb_free(msb);
#endif
}

void recv_res_ack(struct mem_link_device *mld, struct legacy_ipc_device *dev,
		  struct mem_snapshot *mst)
{
	dev->req_ack_cnt[TX] -= 1;

	txq_start(mld, dev);

#ifdef DEBUG_MODEM_IF_FLOW_CTRL
	print_res_ack(mld, mst, dev, RX);
#endif
}

void recv_req_ack(struct mem_link_device *mld, struct legacy_ipc_device *dev,
		  struct mem_snapshot *mst)
{
	dev->req_ack_cnt[RX] += 1;

#ifdef DEBUG_MODEM_IF_FLOW_CTRL
	print_req_ack(mld, mst, dev, RX);
#endif
}

void send_res_ack(struct mem_link_device *mld, struct legacy_ipc_device *dev)
{
#ifdef DEBUG_MODEM_IF_FLOW_CTRL
	struct mst_buff *msb;
#endif

	send_ipc_irq(mld, mask2int(res_ack_mask(dev)));
	dev->req_ack_cnt[RX] -= 1;

#ifdef DEBUG_MODEM_IF_FLOW_CTRL
	msb = mem_take_snapshot(mld, TX);
	if (!msb)
		return;
	print_res_ack(mld, &msb->snapshot, dev, TX);
	msb_free(msb);
#endif
}
