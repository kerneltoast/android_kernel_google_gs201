// SPDX-License-Identifier: GPL-2.0 only
/*
 *  Emerald Hill compression engine driver internal header
 *
 *  Copyright (C) 2020 Google LLC
 *  Author: Petri Gynther <pgynther@google.com>
 */
#ifndef _EH_INTERNAL_H
#define _EH_INTERNAL_H

#include <linux/eh.h>
#include "eh_regs.h"
#include <linux/spinlock_types.h>
#include <linux/wait.h>

struct eh_completion {
	void *priv;
};

#define EH_MAX_DCMD 8

#define EH_QUIRK_IGNORE_GCTRL_RESET BIT(0)

struct eh_device {
	struct list_head eh_dev_list;

	/* hardware characteristics */

	/* how many decompression command sets are implemented */
	unsigned int decompr_cmd_count;

	/* relating to the fifo and masks used to do related calculations */
	unsigned short fifo_size;
	unsigned short fifo_index_mask;
	unsigned short fifo_color_mask;

	/* cached copy of HW write index */
	unsigned int write_index;

	/* cached copy of HW complete index */
	unsigned int complete_index;

	__iomem unsigned char *regs;

	/* in-memory allocated location (not aligned) for cacheable fifo */
	void *fifo_alloc;

	/* 64B aligned compression command fifo of either type 0 or type 1 */
	void *fifo;

	spinlock_t fifo_prod_lock;

	/* Array of completions to keep track of each ongoing compression */
	struct eh_completion *completions;

	/* Array of pre-allocated buffers for compression */
	void **compr_buffers;

#ifdef CONFIG_GOOGLE_EH_DCMD_STATUS_IN_MEMORY
	unsigned long decompr_status[EH_MAX_DCMD];
#endif
	/* Array of pre-allocated bounce buffers for decompression */
	unsigned long __percpu *bounce_buffer;

	/* parent device */
	struct device *dev;

	/* EH clock */
	struct clk *clk;

	int error_irq;

	/*
	 * no interrupts, need to use a polling
	 * We use the polling for the HW validation testing .
	 */
#define EH_POLL_DELAY_MS 500

	unsigned short quirks;

	struct task_struct *comp_thread;
	wait_queue_head_t comp_wq;
	atomic_t nr_request;

	eh_cb_fn comp_callback;
};
#endif
