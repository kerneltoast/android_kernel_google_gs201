// SPDX-License-Identifier: GPL-2.0 only
/*
 *  Emerald Hill compression engine driver
 *
 *  Copyright (C) 2020 Google LLC
 *  Author: Petri Gynther <pgynther@google.com>
 *
 *  Derived from:
 *  Hardware Compressed RAM offload driver
 *  Copyright (C) 2015 The Chromium OS Authors
 *  Author: Sonny Rao <sonnyrao@chromium.org>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef _EH_H_
#define _EH_H_

#include "eh_regs.h"
#include <asm/atomic.h>
#include <linux/completion.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>

typedef void (*eh_cb_fn)(unsigned int status, void *data, unsigned int size,
			 void *priv);

struct eh_completion {
	void *priv;
#ifdef CONFIG_GOOGLE_EH_LATENCY_STAT
	u64 submit_ts;
#endif
};

#define EH_MAX_NAME 8
#define EH_MAX_DCMD 32
#define EH_MAX_IRQS (EH_MAX_DCMD + 2)

#define EH_QUIRK_IGNORE_GCTRL_RESET BIT(0)

enum eh_stat_event {
	EH_COMPRESS,
	EH_DECOMPRESS,
	EH_COMPRESS_POLL,
	EH_DECOMPRESS_POLL,
	NR_EH_EVENT_TYPE,
};

struct eh_stats {
	unsigned long events[NR_EH_EVENT_TYPE];
	unsigned long avg_lat[NR_EH_EVENT_TYPE];
	unsigned long max_lat[NR_EH_EVENT_TYPE];
	unsigned long min_lat[NR_EH_EVENT_TYPE];
};

struct eh_device {
	char name[EH_MAX_NAME];
	unsigned int id;
	struct list_head eh_dev_list;

	/* hardware characteristics */

	/*
	 * implementations can specify how many buffers are used to store
	 * compressed data
	 */
	uint8_t max_buffer_count;

	/* how many decompression command sets are implemented */
	unsigned int decompr_cmd_count;

	/* relating to the fifo and masks used to do related calculations */
	uint16_t fifo_size;
	uint16_t fifo_index_mask;
	uint16_t fifo_color_mask;

	/* cached copy of HW write index */
	unsigned int write_index;

	/* cached copy of HW complete index */
	unsigned int complete_index;

	__iomem uint8_t *regs;

	/* in-memory allocated location (not aligned) for cacheable fifo */
	void *fifo_alloc;

	/* 64B aligned compression command fifo of either type 0 or type 1 */
	void *fifo;

	spinlock_t fifo_prod_lock;

	/* Array of completions to keep track of each ongoing compression */
	struct eh_completion *completions;

	/* Array of pre-allocated buffers for compression */
	void **compr_buffers;

	/* array of atomic variables to keep track of decompression sets*/
	atomic_t *decompr_cmd_used;
	spinlock_t decompr_lock[EH_MAX_DCMD];
	bool decompr_busy[EH_MAX_DCMD];

#ifdef CONFIG_GOOGLE_EH_DCMD_STATUS_IN_MEMORY
	unsigned long decompr_status[EH_MAX_DCMD];
#endif

	/* Array of pre-allocated buffers for decompression */
	void *decompr_buffers[EH_MAX_DCMD];

	struct eh_completion decompr_completions[EH_MAX_DCMD];

	/* parent device */
	struct device *dev;

	/*
	 * interrupts:
	 * one for errors
	 * one for compression
	 * one for each decompression command set
	 */
	int error_irq;
	int compr_irq;
	int decompr_irqs[EH_MAX_DCMD];

	/*
	 * no interrupts, need to use a polling
	 * We use the polling for the HW validation testing .
	 */
#define EH_POLL_DELAY_MS 500

	/*
	 * copy of initialization paramters so we can tear down run some raw
	 * tests and re-init later
	 */
	int irq_count;
	int irqs_copy[EH_MAX_DCMD * 2];

	uint16_t quirks;
	/* indicate whether compression mode is poll or irq */
	bool comp_poll;

	eh_cb_fn comp_callback;
	eh_cb_fn decomp_callback;
	/* latency stats */
	struct eh_stats __percpu *stats;
};

/* tear down hardware block, mostly done when eh_device is unloaded */
void eh_remove(struct eh_device *eh_dev);

/*
 * start a compression, returns non-zero if fifo is full
 *
 * the completion argument includes a callback which is called when
 * the compression is completed and returns the size, status, and
 * the memory used to store the compressed data.
 */
int eh_compress_pages(struct eh_device *eh_dev, struct page **pages,
		      unsigned int page_cnt, void *priv);

static inline int eh_compress_page(struct eh_device *eh_dev, struct page *page,
				   void *priv)
{
	return eh_compress_pages(eh_dev, &page, 1, priv);
}

int eh_compress_pages_sync(struct eh_device *eh_dev, struct page **pages,
			   unsigned int page_cnt,  void *priv);

int eh_decompress_page(struct eh_device *eh_dev, void *compr_data,
		       unsigned int compr_size, struct page *page,
		       void *priv);

int eh_decompress_page_sync(struct eh_device *eh_dev, void *compr_data,
			    unsigned int compr_size, struct page *page);

/* returns the currently set fifo size */
static inline uint16_t eh_get_fifo_size(struct eh_device *eh_dev)
{
	return eh_dev->fifo_size;
}

/* create eh_device for user */
struct eh_device *eh_create(eh_cb_fn comp, eh_cb_fn decomp);

/* destroty eh_device */
void eh_destroy(struct eh_device *eh_dev);

#endif /* _EH_H_ */
