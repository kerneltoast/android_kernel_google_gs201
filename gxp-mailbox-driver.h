/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * GXP mailbox driver.
 *
 * Copyright (C) 2020-2024 Google LLC
 */

#ifndef __GXP_MAILBOX_DRIVER_H__
#define __GXP_MAILBOX_DRIVER_H__

#include <linux/io.h>

#include <gcip/gcip-mailbox.h>

#include "gxp-config.h"
#include "gxp-mailbox.h"

void gxp_mailbox_driver_init(struct gxp_mailbox *mailbox);
void gxp_mailbox_driver_exit(struct gxp_mailbox *mailbox);

void gxp_mailbox_driver_register_interrupts(struct gxp_mailbox *mailbox);
void gxp_mailbox_driver_disable_interrupts(struct gxp_mailbox *mailbox);

void __iomem *gxp_mailbox_get_csr_base(struct gxp_dev *gxp, uint index);
void __iomem *gxp_mailbox_get_data_base(struct gxp_dev *gxp, uint index);

static inline u32 gxp_mailbox_csr_read(struct gxp_mailbox *mailbox, uint reg_offset)
{
	return readl(mailbox->csr_reg_base + reg_offset);
}

static inline void gxp_mailbox_csr_write(struct gxp_mailbox *mailbox, uint reg_offset, u32 value)
{
	writel(value, mailbox->csr_reg_base + reg_offset);
}

void gxp_mailbox_reset_hw(struct gxp_mailbox *mailbox);
/**
 * gxp_mailbox_generate_device_interrupt(): Trigger interrupt to device.
 * @mailbox: Mailbox for which to generate interrupt.
 * @int_mask: Bitset of interrupt line to trigger.
 */
void gxp_mailbox_generate_device_interrupt(struct gxp_mailbox *mailbox,
					   u32 int_mask);
/**
 * gxp_mailbox_clear_interrupts() - Clear set bits corresponding to interrupts
 *                                  coming to AP/Host.
 * @mailbox: Mailbox for which to clear interrupts.
 * @intr_bits: Bitset of interrupt lines to clear.
 */
void gxp_mailbox_clear_interrupts(struct gxp_mailbox *mailbox, u32 intr_bits);
/**
 * gxp_mailbox_enable_interrupt() - Enable the interrupt coming to AP/Host.
 * @mailbox: Mailbox for which to enable interrupts.
 */
void gxp_mailbox_enable_interrupt(struct gxp_mailbox *mailbox);
/* gxp_mailbox_wait_for_device_mailbox_init() - Wait for mailbox to get
 *                                              enabled/initialised by device.
 * @mailbox: Mailbox to get it enabled from device end.
 *
 * Return: 0 on success else -ETIMEDOUT.
 */
int gxp_mailbox_wait_for_device_mailbox_init(struct gxp_mailbox *mailbox);
/**
 * gxp_mailbox_chip_irq_handler() - IRQ handler based on chip.
 * @mailbox: Mailbox on which interrupt received.
 *
 * Context: Interrupt context.
 */
void gxp_mailbox_chip_irq_handler(struct gxp_mailbox *mailbox);

void gxp_mailbox_write_status(struct gxp_mailbox *mailbox, u32 status);
void gxp_mailbox_write_descriptor(struct gxp_mailbox *mailbox,
				  dma_addr_t descriptor_addr);

void gxp_mailbox_write_cmd_queue_tail(struct gxp_mailbox *mailbox, u16 val);
void gxp_mailbox_write_resp_queue_head(struct gxp_mailbox *mailbox, u16 val);
u16 gxp_mailbox_read_cmd_queue_head(struct gxp_mailbox *mailbox);
u16 gxp_mailbox_read_resp_queue_tail(struct gxp_mailbox *mailbox);

/*
 * These functions are only used to initialize these values on mailbox startup.
 * During normal use, the host must not write the command queue head or response
 * queue tail, as the device sets those values.
 */
void gxp_mailbox_write_cmd_queue_head(struct gxp_mailbox *mailbox, u16 val);
void gxp_mailbox_write_resp_queue_tail(struct gxp_mailbox *mailbox, u16 val);
u16 gxp_mailbox_read_cmd_queue_tail(struct gxp_mailbox *mailbox);
u16 gxp_mailbox_read_resp_queue_head(struct gxp_mailbox *mailbox);

/* Sets mailbox->cmd_queue_tail and corresponding CSR on device. */
void gxp_mailbox_set_cmd_queue_tail(struct gxp_mailbox *mailbox, u32 value);

/* Sets mailbox->resp_queue_head and corresponding CSR on device. */
void gxp_mailbox_set_resp_queue_head(struct gxp_mailbox *mailbox, u32 value);

void gxp_mailbox_set_control(struct gxp_mailbox *mailbox, u32 val);

/*
 * Increases the command queue tail by @inc.
 *
 * The queue uses the mirrored circular buffer arrangement. Each index (head and
 * tail) has a wrap bit, represented by the constant CIRCULAR_QUEUE_WRAP_BIT.
 * Whenever an index is increased and will exceed the end of the queue, the wrap
 * bit is xor-ed.
 *
 * This method will update both mailbox->cmd_queue_tail and CSR on device.
 *
 * Returns 0 on success.
 * If command queue tail will exceed command queue head after adding @inc,
 * -EBUSY is returned and all fields remain unchanged. The caller should
 * handle this case and implement a mechanism to wait until the consumer
 * consumes commands.
 *
 * This doesn't acquire any locks internally. The caller may have to hold its own
 * lock before calling this function. If the caller must hold `@mailbox->cmd_queue_lock`
 * before calling this, please use `gxp_mailbox_inc_cmd_queue_tail_locked` function instead.
 */
int gxp_mailbox_inc_cmd_queue_tail_nolock(struct gxp_mailbox *mailbox, u32 inc,
					  u32 wrap_bit);

/*
 * Wrapper function of `gxp_mailbox_inc_cmd_queue_tail_nolock`.
 * Caller must hold @mailbox->cmd_queue_lock.
 */
int gxp_mailbox_inc_cmd_queue_tail_locked(struct gxp_mailbox *mailbox, u32 inc,
					  u32 wrap_bit);

/*
 * Increases the response queue head by @inc.
 *
 * The queue uses the mirrored circular buffer arrangement. Each index (head and
 * tail) has a wrap bit, represented by the constant CIRCULAR_QUEUE_WRAP_BIT.
 * Whenever an index is increased and will exceed the end of the queue, the wrap
 * bit is xor-ed.
 *
 * This method will update both mailbox->resp_queue_head and CSR on device.
 *
 * Returns 0 on success.
 * -EINVAL is returned if the queue head will exceed tail of queue, and no
 * fields or CSR is updated in this case.
 *
 * This doesn't acquire any locks internally. The caller may have to hold its own
 * lock before calling this function. If the caller must hold `@mailbox->cmd_queue_lock`
 * before calling this, please use `gxp_mailbox_inc_cmd_queue_tail_locked` function instead.
 */
int gxp_mailbox_inc_resp_queue_head_nolock(struct gxp_mailbox *mailbox, u32 inc,
					   u32 wrap_bit);

/*
 * Wrapper function of `gxp_mailbox_inc_resp_queue_head_nolock`.
 * Caller must hold @mailbox->resp_queue_lock.
 */
int gxp_mailbox_inc_resp_queue_head_locked(struct gxp_mailbox *mailbox, u32 inc,
					   u32 wrap_bit);

/*
 * Following functions are used when setting the operators of `struct gcip_mailbox_ops`.
 * To use these functions, @mailbox->data should be set as an instance of `struct gxp_mailbox`.
 */
u32 gxp_mailbox_gcip_ops_get_cmd_queue_tail(struct gcip_mailbox *mailbox);
void gxp_mailbox_gcip_ops_inc_cmd_queue_tail(struct gcip_mailbox *mailbox,
					     u32 inc);
int gxp_mailbox_gcip_ops_acquire_cmd_queue_lock(struct gcip_mailbox *mailbox,
						bool try, bool *atomic);
void gxp_mailbox_gcip_ops_release_cmd_queue_lock(struct gcip_mailbox *mailbox);

u32 gxp_mailbox_gcip_ops_get_resp_queue_size(struct gcip_mailbox *mailbox);
u32 gxp_mailbox_gcip_ops_get_resp_queue_head(struct gcip_mailbox *mailbox);
u32 gxp_mailbox_gcip_ops_get_resp_queue_tail(struct gcip_mailbox *mailbox);
void gxp_mailbox_gcip_ops_inc_resp_queue_head(struct gcip_mailbox *mailbox,
					      u32 inc);
int gxp_mailbox_gcip_ops_acquire_resp_queue_lock(struct gcip_mailbox *mailbox,
						 bool try, bool *atomic);
void gxp_mailbox_gcip_ops_release_resp_queue_lock(struct gcip_mailbox *mailbox);

void gxp_mailbox_gcip_ops_acquire_wait_list_lock(struct gcip_mailbox *mailbox,
						 bool irqsave,
						 unsigned long *flags);
void gxp_mailbox_gcip_ops_release_wait_list_lock(struct gcip_mailbox *mailbox,
						 bool irqrestore,
						 unsigned long flags);

int gxp_mailbox_gcip_ops_wait_for_cmd_queue_not_full(
	struct gcip_mailbox *mailbox);
int gxp_mailbox_gcip_ops_after_enqueue_cmd(struct gcip_mailbox *mailbox,
					   void *cmd);
void gxp_mailbox_gcip_ops_after_fetch_resps(struct gcip_mailbox *mailbox,
					    u32 num_resps);
bool gxp_mailbox_gcip_ops_is_block_off(struct gcip_mailbox *mailbox);

#endif /* __GXP_MAILBOX_DRIVER_H__ */
