// SPDX-License-Identifier: GPL-2.0-only
/*
 * GXP hardware-based mailbox csr driver implementation for GSX01.
 *
 * Copyright (C) 2022 Google LLC
 */

#include <asm/barrier.h>
#include <linux/bitops.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/of_irq.h>
#include <linux/spinlock.h>

#include "gxp-config.h"
#include "gxp-mailbox-driver.h"
#include "gxp-mailbox.h"

#include "gxp-mailbox-driver.c"

static u32 gxp_mailbox_get_interrupt_status(struct gxp_mailbox *mailbox)
{
	return gxp_mailbox_csr_read(mailbox, MBOX_INTMSR1_OFFSET);
}

void gxp_mailbox_reset_hw(struct gxp_mailbox *mailbox)
{
	gxp_mailbox_csr_write(mailbox, MBOX_MCUCTLR_OFFSET, 1);
}

/* Interrupt to signal a response from the device to host */
#define MBOX_DEVICE_TO_HOST_RESPONSE_IRQ_MASK BIT(0)

void gxp_mailbox_chip_irq_handler(struct gxp_mailbox *mailbox)
{
	u32 intr_bits;
	struct work_struct **handlers = mailbox->interrupt_handlers;
	u32 next_int;

	/* Contains only the non-masked, pending interrupt bits */
	intr_bits = gxp_mailbox_get_interrupt_status(mailbox);

	/* Clear all pending IRQ bits */
	gxp_mailbox_clear_interrupts(mailbox, intr_bits);

	if (intr_bits & MBOX_DEVICE_TO_HOST_RESPONSE_IRQ_MASK) {
		gxp_mailbox_handle_irq(mailbox);
		intr_bits &= ~MBOX_DEVICE_TO_HOST_RESPONSE_IRQ_MASK;
	}

	while ((next_int = ffs(intr_bits))) {
		next_int--; /* ffs returns 1-based indices */
		intr_bits &= ~BIT(next_int);

		if (handlers[next_int])
			schedule_work(handlers[next_int]);
		else
			dev_warn_ratelimited(mailbox->gxp->dev,
					     "mailbox%d: received unknown interrupt bit 0x%X\n",
					     mailbox->core_id, next_int);
	}
}

void gxp_mailbox_generate_device_interrupt(struct gxp_mailbox *mailbox, u32 int_mask)
{
	/*
	 * Ensure all memory writes have been committed to memory before
	 * signalling to the device to read from them. This avoids the scenario
	 * where the interrupt trigger write gets delivered to the MBX HW before
	 * the DRAM transactions made it to DRAM since they're Normal
	 * transactions and can be re-ordered and backed off behind other
	 * transfers.
	 */
	wmb();

	gxp_mailbox_csr_write(mailbox, MBOX_INTGR0_OFFSET, int_mask);
}

void gxp_mailbox_clear_interrupts(struct gxp_mailbox *mailbox, u32 intr_bits)
{
	gxp_mailbox_csr_write(mailbox, MBOX_INTCR1_OFFSET, intr_bits);
}

void gxp_mailbox_enable_interrupt(struct gxp_mailbox *mailbox)
{
}

int gxp_mailbox_wait_for_device_mailbox_init(struct gxp_mailbox *mailbox)
{
	return 0;
}
