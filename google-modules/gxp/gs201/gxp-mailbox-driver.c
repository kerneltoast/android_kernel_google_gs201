// SPDX-License-Identifier: GPL-2.0-only
/*
 * GXP hardware-based mailbox driver implementation.
 *
 * Copyright (C) 2021-2024 Google LLC
 */

#include <asm/barrier.h>
#include <linux/bitops.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/of_irq.h>
#include <linux/spinlock.h>

#include <gcip/gcip-mailbox.h>
#include <trace/events/gxp.h>

#include "gxp-mailbox-driver.h"
#include "gxp-mailbox-regs.h"
#include "gxp-mailbox.h"
#include "gxp-pm.h"

static u32 data_read(struct gxp_mailbox *mailbox, uint reg_offset)
{
	return readl(mailbox->data_reg_base + reg_offset);
}

static void data_write(struct gxp_mailbox *mailbox, uint reg_offset, u32 value)
{
	writel(value, mailbox->data_reg_base + reg_offset);
}

/* IRQ Handling */
static irqreturn_t mailbox_irq_handler(int irq, void *arg)
{
	struct gxp_mailbox *mailbox = (struct gxp_mailbox *)arg;

	trace_gxp_uci_rsp_start(irq);

	gxp_mailbox_chip_irq_handler(mailbox);
	return IRQ_HANDLED;
}

static void register_irq(struct gxp_mailbox *mailbox)
{
	int err;
	unsigned int virq;

	virq = irq_of_parse_and_map(mailbox->gxp->dev->of_node,
				    mailbox->core_id);
	if (!virq) {
		pr_err("Unable to parse interrupt for core %d from the DT\n",
		       mailbox->core_id);
		return;
	}

	err = request_irq(virq, mailbox_irq_handler, /*flags=*/0,
			  "aurora_mbx_irq", (void *)mailbox);
	if (err) {
		pr_err("Unable to register IRQ num=%d; error=%d\n", virq, err);
		return;
	}

	mailbox->interrupt_virq = virq;
	pr_debug("Core %d's mailbox interrupt registered as IRQ %u.\n",
		 mailbox->core_id, virq);
}

static void unregister_irq(struct gxp_mailbox *mailbox)
{
	if (mailbox->interrupt_virq) {
		pr_debug("Freeing IRQ %d\n", mailbox->interrupt_virq);
		free_irq(mailbox->interrupt_virq, mailbox);
		mailbox->interrupt_virq = 0;
	}
}

/* gxp-mailbox-driver.h interface */

void gxp_mailbox_driver_init(struct gxp_mailbox *mailbox)
{
	spin_lock_init(&mailbox->cmd_tail_resp_head_lock);
	spin_lock_init(&mailbox->cmd_head_resp_tail_lock);
}

void gxp_mailbox_driver_exit(struct gxp_mailbox *mailbox)
{
	/* Nothing to cleanup */
}

void gxp_mailbox_driver_register_interrupts(struct gxp_mailbox *mailbox)
{
	register_irq(mailbox);
}

void gxp_mailbox_driver_disable_interrupts(struct gxp_mailbox *mailbox)
{
	unregister_irq(mailbox);
}

void __iomem *gxp_mailbox_get_csr_base(struct gxp_dev *gxp, uint index)
{
	return gxp->mbx[index].vaddr;
}

void __iomem *gxp_mailbox_get_data_base(struct gxp_dev *gxp, uint index)
{
	return gxp->mbx[index].vaddr + MBOX_DATA_REG_BASE;
}

/* gxp-mailbox-driver.h: Data register-based calls */

void gxp_mailbox_write_status(struct gxp_mailbox *mailbox, u32 status)
{
	data_write(mailbox, MBOX_DATA_STATUS_OFFSET, status);
}

void gxp_mailbox_write_descriptor(struct gxp_mailbox *mailbox,
				  dma_addr_t descriptor_addr)
{
	data_write(mailbox, MBOX_DATA_DESCRIPTOR_ADDR_OFFSET, (u32)descriptor_addr);
}

void gxp_mailbox_write_cmd_queue_tail(struct gxp_mailbox *mailbox, u16 val)
{
	u32 current_resp_head;
	u32 new_cmd_tail;
	unsigned long flags;

	spin_lock_irqsave(&mailbox->cmd_tail_resp_head_lock, flags);

	current_resp_head = data_read(mailbox, MBOX_DATA_CMD_TAIL_RESP_HEAD_OFFSET) &
			    RESP_HEAD_MASK;
	new_cmd_tail = (u32)val << CMD_TAIL_SHIFT;
	data_write(mailbox, MBOX_DATA_CMD_TAIL_RESP_HEAD_OFFSET,
		   new_cmd_tail | current_resp_head);

	spin_unlock_irqrestore(&mailbox->cmd_tail_resp_head_lock, flags);
}

void gxp_mailbox_write_resp_queue_head(struct gxp_mailbox *mailbox, u16 val)
{
	u32 current_cmd_tail;
	u32 new_resp_head;
	unsigned long flags;

	spin_lock_irqsave(&mailbox->cmd_tail_resp_head_lock, flags);

	current_cmd_tail = data_read(mailbox, MBOX_DATA_CMD_TAIL_RESP_HEAD_OFFSET) &
			   CMD_TAIL_MASK;
	new_resp_head = (u32)val << RESP_HEAD_SHIFT;
	data_write(mailbox, MBOX_DATA_CMD_TAIL_RESP_HEAD_OFFSET,
		   current_cmd_tail | new_resp_head);

	spin_unlock_irqrestore(&mailbox->cmd_tail_resp_head_lock, flags);
}

u16 gxp_mailbox_read_cmd_queue_head(struct gxp_mailbox *mailbox)
{
	u32 reg_val;
	unsigned long flags;

	spin_lock_irqsave(&mailbox->cmd_head_resp_tail_lock, flags);

	reg_val = data_read(mailbox, MBOX_DATA_CMD_HEAD_RESP_TAIL_OFFSET);

	spin_unlock_irqrestore(&mailbox->cmd_head_resp_tail_lock, flags);

	return (u16)((reg_val & CMD_HEAD_MASK) >> CMD_HEAD_SHIFT);
}

u16 gxp_mailbox_read_resp_queue_tail(struct gxp_mailbox *mailbox)
{
	u32 reg_val;
	unsigned long flags;

	spin_lock_irqsave(&mailbox->cmd_head_resp_tail_lock, flags);

	reg_val = data_read(mailbox, MBOX_DATA_CMD_HEAD_RESP_TAIL_OFFSET);

	spin_unlock_irqrestore(&mailbox->cmd_head_resp_tail_lock, flags);

	return (u16)((reg_val & RESP_TAIL_MASK) >> RESP_TAIL_SHIFT);
}

void gxp_mailbox_write_cmd_queue_head(struct gxp_mailbox *mailbox, u16 val)
{
	u32 current_resp_tail;
	u32 new_cmd_head;
	unsigned long flags;

	spin_lock_irqsave(&mailbox->cmd_head_resp_tail_lock, flags);

	current_resp_tail = data_read(mailbox, MBOX_DATA_CMD_HEAD_RESP_TAIL_OFFSET) &
			    RESP_TAIL_MASK;
	new_cmd_head = (u32)val << CMD_HEAD_SHIFT;
	data_write(mailbox, MBOX_DATA_CMD_HEAD_RESP_TAIL_OFFSET,
		   new_cmd_head | current_resp_tail);

	spin_unlock_irqrestore(&mailbox->cmd_head_resp_tail_lock, flags);
}

void gxp_mailbox_write_resp_queue_tail(struct gxp_mailbox *mailbox, u16 val)
{
	u32 current_cmd_head;
	u32 new_resp_tail;
	unsigned long flags;

	spin_lock_irqsave(&mailbox->cmd_head_resp_tail_lock, flags);

	current_cmd_head = data_read(mailbox, MBOX_DATA_CMD_HEAD_RESP_TAIL_OFFSET) &
			   CMD_HEAD_MASK;
	new_resp_tail = (u32)val << RESP_TAIL_SHIFT;
	data_write(mailbox, MBOX_DATA_CMD_HEAD_RESP_TAIL_OFFSET,
		   current_cmd_head | new_resp_tail);

	spin_unlock_irqrestore(&mailbox->cmd_head_resp_tail_lock, flags);
}

u16 gxp_mailbox_read_cmd_queue_tail(struct gxp_mailbox *mailbox)
{
	u32 reg_val;
	unsigned long flags;

	spin_lock_irqsave(&mailbox->cmd_tail_resp_head_lock, flags);

	reg_val = data_read(mailbox, MBOX_DATA_CMD_TAIL_RESP_HEAD_OFFSET);

	spin_unlock_irqrestore(&mailbox->cmd_tail_resp_head_lock, flags);

	return (u16)((reg_val & CMD_TAIL_MASK) >> CMD_TAIL_SHIFT);
}

u16 gxp_mailbox_read_resp_queue_head(struct gxp_mailbox *mailbox)
{
	u32 reg_val;
	unsigned long flags;

	spin_lock_irqsave(&mailbox->cmd_tail_resp_head_lock, flags);

	reg_val = data_read(mailbox, MBOX_DATA_CMD_TAIL_RESP_HEAD_OFFSET);

	spin_unlock_irqrestore(&mailbox->cmd_tail_resp_head_lock, flags);

	return (u16)((reg_val & RESP_HEAD_MASK) >> RESP_HEAD_SHIFT);
}

void gxp_mailbox_set_cmd_queue_tail(struct gxp_mailbox *mailbox, u32 value)
{
	mailbox->cmd_queue_tail = value;
	gxp_mailbox_write_cmd_queue_tail(mailbox, value);
}

void gxp_mailbox_set_resp_queue_head(struct gxp_mailbox *mailbox, u32 value)
{
	mailbox->resp_queue_head = value;
	gxp_mailbox_write_resp_queue_head(mailbox, value);
}

void gxp_mailbox_set_control(struct gxp_mailbox *mailbox, u32 val)
{
	data_write(mailbox, MBOX_DATA_CONTROL_OFFSET, val);
}

int gxp_mailbox_inc_cmd_queue_tail_nolock(struct gxp_mailbox *mailbox, u32 inc,
					  u32 wrap_bit)
{
	u32 head;
	u32 remain_size;
	u32 new_tail;

	if (inc > mailbox->cmd_queue_size)
		return -EINVAL;

	head = gxp_mailbox_read_cmd_queue_head(mailbox);
	remain_size = mailbox->cmd_queue_size - gcip_circ_queue_cnt(head, mailbox->cmd_queue_tail,
								    mailbox->cmd_queue_size,
								    wrap_bit);
	/* no enough space left */
	if (inc > remain_size)
		return -EBUSY;

	new_tail = gcip_circ_queue_inc(mailbox->cmd_queue_tail, inc, mailbox->cmd_queue_size,
				       wrap_bit);
	gxp_mailbox_set_cmd_queue_tail(mailbox, new_tail);
	return 0;
}

int gxp_mailbox_inc_cmd_queue_tail_locked(struct gxp_mailbox *mailbox, u32 inc,
					  u32 wrap_bit)
{
	lockdep_assert_held(&mailbox->cmd_queue_lock);
	return gxp_mailbox_inc_cmd_queue_tail_nolock(mailbox, inc, wrap_bit);
}

int gxp_mailbox_inc_resp_queue_head_nolock(struct gxp_mailbox *mailbox, u32 inc,
					   u32 wrap_bit)
{
	u32 tail;
	u32 size;
	u32 new_head;

	if (inc > mailbox->resp_queue_size)
		return -EINVAL;

	tail = gxp_mailbox_read_resp_queue_tail(mailbox);
	size = gcip_circ_queue_cnt(mailbox->resp_queue_head, tail, mailbox->resp_queue_size,
				   wrap_bit);
	if (inc > size)
		return -EINVAL;
	new_head = gcip_circ_queue_inc(mailbox->resp_queue_head, inc, mailbox->resp_queue_size,
				       wrap_bit);
	gxp_mailbox_set_resp_queue_head(mailbox, new_head);

	return 0;
}

int gxp_mailbox_inc_resp_queue_head_locked(struct gxp_mailbox *mailbox, u32 inc,
					   u32 wrap_bit)
{
	lockdep_assert_held(&mailbox->resp_queue_lock);
	return gxp_mailbox_inc_resp_queue_head_nolock(mailbox, inc, wrap_bit);
}

u32 gxp_mailbox_gcip_ops_get_cmd_queue_tail(struct gcip_mailbox *mailbox)
{
	struct gxp_mailbox *gxp_mbx = mailbox->data;

	return gxp_mbx->cmd_queue_tail;
}

void gxp_mailbox_gcip_ops_inc_cmd_queue_tail(struct gcip_mailbox *mailbox,
					     u32 inc)
{
	struct gxp_mailbox *gxp_mbx = mailbox->data;

	lockdep_assert_held(&gxp_mbx->cmd_queue_lock);
	gxp_mailbox_inc_cmd_queue_tail_nolock(gxp_mbx, inc,
					      mailbox->queue_wrap_bit);
}

int gxp_mailbox_gcip_ops_acquire_cmd_queue_lock(struct gcip_mailbox *mailbox,
						bool try, bool *atomic)
{
	struct gxp_mailbox *gxp_mbx = mailbox->data;

	mutex_lock(&gxp_mbx->cmd_queue_lock);
	return 1;
}

void gxp_mailbox_gcip_ops_release_cmd_queue_lock(struct gcip_mailbox *mailbox)
{
	struct gxp_mailbox *gxp_mbx = mailbox->data;

	mutex_unlock(&gxp_mbx->cmd_queue_lock);
}

u32 gxp_mailbox_gcip_ops_get_resp_queue_size(struct gcip_mailbox *mailbox)
{
	struct gxp_mailbox *gxp_mbx = mailbox->data;

	return gxp_mbx->resp_queue_size;
}

u32 gxp_mailbox_gcip_ops_get_resp_queue_head(struct gcip_mailbox *mailbox)
{
	struct gxp_mailbox *gxp_mbx = mailbox->data;

	return gxp_mbx->resp_queue_head;
}

u32 gxp_mailbox_gcip_ops_get_resp_queue_tail(struct gcip_mailbox *mailbox)
{
	struct gxp_mailbox *gxp_mbx = mailbox->data;

	return gxp_mailbox_read_resp_queue_tail(gxp_mbx);
}

void gxp_mailbox_gcip_ops_inc_resp_queue_head(struct gcip_mailbox *mailbox,
					       u32 inc)
{
	struct gxp_mailbox *gxp_mbx = mailbox->data;

	lockdep_assert_held(&gxp_mbx->resp_queue_lock);
	gxp_mailbox_inc_resp_queue_head_nolock(gxp_mbx, inc,
					       mailbox->queue_wrap_bit);
}

int gxp_mailbox_gcip_ops_acquire_resp_queue_lock(struct gcip_mailbox *mailbox,
						 bool try, bool *atomic)
{
	struct gxp_mailbox *gxp_mbx = mailbox->data;

	*atomic = true;

	if (try)
		return spin_trylock_irqsave(&gxp_mbx->resp_queue_lock,
					    gxp_mbx->resp_queue_lock_flags);

	spin_lock_irqsave(&gxp_mbx->resp_queue_lock, gxp_mbx->resp_queue_lock_flags);
	return 1;
}

void gxp_mailbox_gcip_ops_release_resp_queue_lock(struct gcip_mailbox *mailbox)
{
	struct gxp_mailbox *gxp_mbx = mailbox->data;

	spin_unlock_irqrestore(&gxp_mbx->resp_queue_lock, gxp_mbx->resp_queue_lock_flags);
}

void gxp_mailbox_gcip_ops_acquire_wait_list_lock(struct gcip_mailbox *mailbox,
						 bool irqsave,
						 unsigned long *flags)
{
	struct gxp_mailbox *gxp_mbx = mailbox->data;

	spin_lock_irqsave(&gxp_mbx->wait_list_lock, *flags);
}

void gxp_mailbox_gcip_ops_release_wait_list_lock(struct gcip_mailbox *mailbox,
						 bool irqrestore,
						 unsigned long flags)
{
	struct gxp_mailbox *gxp_mbx = mailbox->data;

	spin_unlock_irqrestore(&gxp_mbx->wait_list_lock, flags);
}

int gxp_mailbox_gcip_ops_wait_for_cmd_queue_not_full(
	struct gcip_mailbox *mailbox)
{
	struct gxp_mailbox *gxp_mbx = mailbox->data;
	u32 tail = gxp_mbx->cmd_queue_tail;

	/*
	 * If the cmd queue is full, it's up to the caller to retry.
	 */
	if (gxp_mailbox_read_cmd_queue_head(gxp_mbx) ==
	    (tail ^ mailbox->queue_wrap_bit)) {
		return -EAGAIN;
	}

	return 0;
}

int gxp_mailbox_gcip_ops_after_enqueue_cmd(struct gcip_mailbox *mailbox,
					   void *cmd)
{
	struct gxp_mailbox *gxp_mbx = mailbox->data;

	/* triggers doorbell */
	gxp_mailbox_generate_device_interrupt(gxp_mbx, BIT(0));

	return 0;
}

void gxp_mailbox_gcip_ops_after_fetch_resps(struct gcip_mailbox *mailbox,
					    u32 num_resps)
{
	struct gxp_mailbox *gxp_mbx = mailbox->data;
	u32 size = gxp_mbx->resp_queue_size;

	/*
	 * Now that the response queue has been drained, send an interrupt
	 * to the device in case firmware was waiting for us to consume
	 * responses.
	 */
	if (num_resps == size)
		gxp_mailbox_generate_device_interrupt(gxp_mbx, BIT(0));
}

bool gxp_mailbox_gcip_ops_is_block_off(struct gcip_mailbox *mailbox)
{
	struct gxp_mailbox *gxp_mbx = mailbox->data;

	return gxp_pm_is_blk_down(gxp_mbx->gxp);
}
