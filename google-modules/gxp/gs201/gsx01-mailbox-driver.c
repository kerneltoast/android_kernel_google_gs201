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

static u32 csr_read(struct gxp_mailbox *mailbox, uint reg_offset)
{
	return readl(mailbox->csr_reg_base + reg_offset);
}

static void csr_write(struct gxp_mailbox *mailbox, uint reg_offset, u32 value)
{
	writel(value, mailbox->csr_reg_base + reg_offset);
}

void gxp_mailbox_reset_hw(struct gxp_mailbox *mailbox)
{
	csr_write(mailbox, MBOX_MCUCTLR_OFFSET, 1);
}

void gxp_mailbox_generate_device_interrupt(struct gxp_mailbox *mailbox,
					   u32 int_mask)
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

	csr_write(mailbox, MBOX_INTGR0_OFFSET, int_mask);
}

u32 gxp_mailbox_get_device_mask_status(struct gxp_mailbox *mailbox)
{
	return csr_read(mailbox, MBOX_INTMSR0_OFFSET);
}

void gxp_mailbox_clear_host_interrupt(struct gxp_mailbox *mailbox, u32 int_mask)
{
	csr_write(mailbox, MBOX_INTCR1_OFFSET, int_mask);
}

void gxp_mailbox_mask_host_interrupt(struct gxp_mailbox *mailbox, u32 int_mask)
{
	csr_write(mailbox, MBOX_INTMR1_OFFSET, int_mask);
}

u32 gxp_mailbox_get_host_mask_status(struct gxp_mailbox *mailbox)
{
	return csr_read(mailbox, MBOX_INTMSR1_OFFSET);
}
