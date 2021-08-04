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
 *  Sonny Rao <sonnyrao@chromium.org>
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

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#ifdef CONFIG_GOOGLE_EH_DEBUG
#define DEBUG
#endif

#include "eh_internal.h"
#include <asm/atomic.h>
#include <asm/cacheflush.h>
#include <asm/irqflags.h>
#include <asm/page.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/highmem.h>
#include <linux/idr.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/wait.h>
#include <linux/freezer.h>

/* These are the possible values for the status field from the specification */
enum eh_cdesc_status {
	/* descriptor not in use */
	EH_CDESC_IDLE = 0x0,

	/* descriptor completed with compressed bytes written to target */
	EH_CDESC_COMPRESSED = 0x1,

	/*
	 * descriptor completed, incompressible page, uncompressed bytes written
	 * to target
	 */
	EH_CDESC_COPIED = 0x2,

	/* descriptor completed, incompressible page, nothing written to target
	 */
	EH_CDESC_ABORT = 0x3,

	/* descriptor completed, page was all zero, nothing written to target */
	EH_CDESC_ZERO = 0x4,

	/*
	 * descriptor count not be completed dut to an error.
	 * queue operation continued to next descriptor
	 */
	EH_CDESC_ERROR_CONTINUE = 0x5,

	/*
	 * descriptor count not be completed dut to an error.
	 * queue operation halted
	 */
	EH_CDESC_ERROR_HALTED = 0x6,

	/* descriptor in queue or being processed by hardware */
	EH_CDESC_PENDING = 0x7,
};


#define EH_ERR_IRQ	"eh_error"
#define EH_COMP_IRQ	"eh_comp"

/* wait up to a millisecond for reset */
#define EH_RESET_DELAY_US	10
#define EH_RESET_MAX_TRIAL	100

/* list of all unclaimed EH devices */
static LIST_HEAD(eh_dev_list);
static DEFINE_SPINLOCK(eh_dev_list_lock);

static DECLARE_WAIT_QUEUE_HEAD(eh_compress_wait);
static unsigned int eh_default_fifo_size = 256;

/*
 * - Primitive functions for Emerald Hill HW
 */
static inline void eh_write_register(struct eh_device *eh_dev,
				     unsigned int offset, unsigned long val)
{
	writeq(val, eh_dev->regs + offset);
}

static inline unsigned long eh_read_register(struct eh_device *eh_dev,
					     unsigned int offset)
{
	return readq(eh_dev->regs + offset);
}

static void eh_dump_regs(struct eh_device *eh_dev)
{
	unsigned int i, offset = 0;

	pr_err("dump_regs: global\n");
	for (offset = EH_REG_HWID; offset <= EH_REG_ERR_MSK; offset += 8)
		pr_err("0x%03X: 0x%016llX\n", offset,
			eh_read_register(eh_dev, offset));

	pr_err("dump_regs: compression\n");
	for (offset = EH_REG_CDESC_LOC; offset <= EH_REG_CINTERP_CTRL;
	     offset += 8)
		pr_err("0x%03X: 0x%016llX\n", offset,
			eh_read_register(eh_dev, offset));

	for (i = 0; i < eh_dev->decompr_cmd_count; i++) {
		pr_err("dump_regs: decompression %u\n", i);
		for (offset = EH_REG_DCMD_CSIZE(i);
		     offset <= EH_REG_DCMD_BUF3(i); offset += 8)
			pr_err("0x%03X: 0x%016llX\n", offset,
				eh_read_register(eh_dev, offset));
	}

	pr_err("dump_regs: vendor\n");
	for (offset = EH_REG_BUSCFG; offset <= 0x118; offset += 8)
		pr_err("0x%03X: 0x%016llX\n", offset,
			eh_read_register(eh_dev, offset));

	pr_err("driver\n");
	pr_err("write_index %u complete_index %u\n",
	       eh_dev->write_index, eh_dev->complete_index);
	pr_err("pending_compression %lu\n", atomic_read(&eh_dev->nr_request));
}

static inline unsigned int fifo_write_index(struct eh_device *eh_dev)
{
	return eh_dev->write_index & eh_dev->fifo_index_mask;
}

static inline unsigned int fifo_complete_index(struct eh_device *eh_dev)
{
	return eh_dev->complete_index & eh_dev->fifo_index_mask;
}

static inline void update_fifo_write_index(struct eh_device *eh_dev)
{
	unsigned int next_write_idx = (eh_dev->write_index + 1) &
				       eh_dev->fifo_color_mask;

	eh_dev->write_index = next_write_idx;
	eh_write_register(eh_dev, EH_REG_CDESC_WRIDX, next_write_idx);
}

static inline void update_fifo_complete_index(struct eh_device *eh_dev)
{
	smp_store_release(&eh_dev->complete_index,
			  (eh_dev->complete_index + 1) &
			  eh_dev->fifo_color_mask);
}

static bool fifo_full(struct eh_device *eh_dev)
{
	unsigned int write_idx = fifo_write_index(eh_dev);
	unsigned int complete_idx = fifo_complete_index(eh_dev);

	if (write_idx != complete_idx)
		return false;
	return  eh_dev->write_index != eh_dev->complete_index;
}

/* index of the next descriptor to be completed by hardware */
static unsigned int fifo_next_complete_index(struct eh_device *eh_dev)
{
	return eh_read_register(eh_dev, EH_REG_CDESC_CTRL) &
				EH_CDESC_CTRL_COMPLETE_IDX_MASK;
}

static struct eh_compress_desc *eh_descriptor(struct eh_device *eh_dev,
						   unsigned int index)
{
	return eh_dev->fifo + index * EH_COMPRESS_DESC_SIZE;
}

static inline unsigned long eh_read_dcmd_status(struct eh_device *eh_dev,
						int index)
{
	unsigned long status;

#ifdef CONFIG_GOOGLE_EH_DCMD_STATUS_IN_MEMORY
	status = READ_ONCE(eh_dev->decompr_status[index]);
#else
	status = eh_read_register(eh_dev, EH_REG_DCMD_DEST(index));
#endif
	return EH_DCMD_DEST_STATUS(status);
}

static int eh_reset(struct eh_device *eh_dev)
{
	int trial;

	if (eh_dev->quirks & EH_QUIRK_IGNORE_GCTRL_RESET)
		return 0;

	eh_write_register(eh_dev, EH_REG_GCTRL, -1);
	for (trial = 0; trial < EH_RESET_MAX_TRIAL; trial++) {
		if (!eh_read_register(eh_dev, EH_REG_GCTRL))
			return 0;
		udelay(EH_RESET_DELAY_US);
	}

	return 1;
}

static void eh_setup_descriptor(struct eh_device *eh_dev, struct page *src_page,
				unsigned int index)
{
	struct eh_compress_desc *desc;
	phys_addr_t src_paddr;

	desc = eh_descriptor(eh_dev, index);
	src_paddr = page_to_phys(src_page);

	desc->src_addr = src_paddr;
	/* mark it as pend for hardware */
	desc->status = EH_CDESC_PENDING;
	/*
	 * Skip setting other fields of the descriptor for the performance
	 * reason. It's doable since they are never changed once they are
	 * initialized. Look at init_compression_descriptor.
	 */
}

static void eh_compr_fifo_init(struct eh_device *eh_dev)
{
	unsigned long data;

	/* FIFO reset: reset hardware write/read/complete index registers */
	data = 1UL << EH_CDESC_CTRL_FIFO_RESET;
	eh_write_register(eh_dev, EH_REG_CDESC_CTRL, data);
	do {
		udelay(1);
		data = eh_read_register(eh_dev, EH_REG_CDESC_CTRL);
	} while (data & (1UL << EH_CDESC_CTRL_FIFO_RESET));

	/* reset software copies of index registers */
	eh_dev->write_index = 0;
	eh_dev->complete_index = 0;

	/* program FIFO memory location and size */
	data = (unsigned long)virt_to_phys(eh_dev->fifo) | __ffs(eh_dev->fifo_size);
	eh_write_register(eh_dev, EH_REG_CDESC_LOC, data);

	/* enable compression */
	data = 1UL << EH_CDESC_CTRL_COMPRESS_ENABLE_SHIFT;
	eh_write_register(eh_dev, EH_REG_CDESC_CTRL, data);
}

/* Set up constant parts of descriptors */
static void init_compression_descriptor(struct eh_device *eh_dev)
{
	int i;
	struct eh_compress_desc *desc;

	for (i = 0; i < eh_dev->fifo_size; i++ ) {
		phys_addr_t dst_paddr;
		int j;

		desc = eh_descriptor(eh_dev, i);
		dst_paddr = virt_to_phys(eh_dev->compr_buffers[i]);
#ifdef CONFIG_GOOGLE_EH_CFIFO_DST_BUFFER_3KB
		desc->max_buf = 2;
		/* buffer 1: top 2KB of compression buffer (page) */
		desc->dst_addr[0] = EH_PHYS_ADDR_TO_ENCODED(dst_paddr, PAGE_SIZE / 2);

		/* buffer 2: next 1KB right after buffer 1 */
		desc->dst_addr[1] = EH_PHYS_ADDR_TO_ENCODED(dst_paddr + PAGE_SIZE / 2,
				PAGE_SIZE / 4);
#else
		desc->max_buf = 1;
		desc->dst_addr[0] = EH_PHYS_ADDR_TO_ENCODED(dst_paddr, PAGE_SIZE);
		desc->dst_addr[1] = 0;
#endif
		for (j = 2; j < EH_NUM_OF_FREE_BLOCKS; j++)
			desc->dst_addr[j] = 0;
	}
}

/*
 * - Primitive functions for Emerald Hill SW
 */
static long eh_congestion_wait(long timeout)
{
	long ret;
	DEFINE_WAIT(wait);
	wait_queue_head_t *wqh = &eh_compress_wait;

	prepare_to_wait(wqh, &wait, TASK_UNINTERRUPTIBLE);
	ret = io_schedule_timeout(timeout);
	finish_wait(wqh, &wait);

	return ret;
}

static void clear_eh_congested(void)
{
	if (waitqueue_active(&eh_compress_wait))
		wake_up(&eh_compress_wait);
}

static irqreturn_t eh_error_irq(int irq, void *data)
{
	struct eh_device *eh_dev = data;
	unsigned long compr, decompr, error;

	compr = eh_read_register(eh_dev, EH_REG_INTRP_STS_CMP);
	decompr = eh_read_register(eh_dev, EH_REG_INTRP_STS_DCMP);
	error = eh_read_register(eh_dev, EH_REG_INTRP_STS_ERROR);

	pr_err("irq %d error 0x%llx compr 0x%llx decompr 0x%llx\n",
	       irq, error, compr, decompr);

	if (error) {
		pr_err("error interrupt was active\n");
		eh_dump_regs(eh_dev);
		eh_write_register(eh_dev, EH_REG_INTRP_STS_ERROR, error);
	}

	return IRQ_HANDLED;
}

/*
 * Non-zero return vaulue means HW is broken so it couldn't operate any
 * longer.
 */
static int eh_process_completed_descriptor(struct eh_device *eh_dev,
					   unsigned short fifo_index)
{
	struct eh_compress_desc *desc;
	unsigned int compr_status;
	unsigned int compr_size;
	unsigned int compr_bufsel;
	unsigned int offset;
	void *compr_data = NULL;
	int ret = 0;
	int compr_result = 0;
	struct eh_completion *compl = &eh_dev->completions[fifo_index];

	desc = eh_descriptor(eh_dev, fifo_index);

	compr_status = desc->status;
	compr_size = desc->compr_len;
	compr_bufsel = desc->buf_sel;
	offset = (compr_bufsel == 2) ? PAGE_SIZE / 2 : 0;

	switch (compr_status) {
	/* normal case, page copied */
	case EH_CDESC_COPIED:
		compr_data = eh_dev->compr_buffers[fifo_index] + offset;
		compr_size = PAGE_SIZE;
		break;

	/* normal case, compression completed successfully */
	case EH_CDESC_COMPRESSED:
		compr_data = eh_dev->compr_buffers[fifo_index] + offset;
		break;

	/* normal case, hardware detected page of all zeros */
	case EH_CDESC_ZERO:
		break;

	/* normal case, incompressible page, did not fit into 3K buffer */
	case EH_CDESC_ABORT:
		compr_size = PAGE_SIZE;
		break;

	/* an error occurred, but hardware is still progressing */
	case EH_CDESC_ERROR_CONTINUE:
		pr_err("got error on descriptor 0x%x\n", fifo_index);
		compr_result = 1;
		break;

	/* a fairly bad error occurred, need to reset the fifo */
	case EH_CDESC_ERROR_HALTED:
		pr_err("got fifo error on descriptor 0x%x\n", fifo_index);
		ret = 1;
		compr_result = 1;
		break;

	/*
	 * this shouldn't normally happen -- hardware indicated completed but
	 * descriptor is still in PEND or IDLE.
	 */
	case EH_CDESC_IDLE:
	case EH_CDESC_PENDING:
		eh_dump_regs(eh_dev);
		pr_err("descriptor 0x%x pend or idle 0x%x: ",
		       fifo_index, compr_status);
		{
			int i;
			unsigned int *p = (unsigned int *)(eh_dev->fifo +
							   (fifo_index *
							    EH_COMPRESS_DESC_SIZE));
			for (i = 0;
			     i < (EH_COMPRESS_DESC_SIZE / sizeof(unsigned int));
			     i++) {
				pr_cont("%08X ", p[i]);
			}
			pr_cont("\n");
		}
		WARN_ON(1);
		compr_result = 1;
		break;
	};

	/* do the callback */
	(*eh_dev->comp_callback)(compr_result, compr_data, compr_size,
				 compl->priv);

	/* set the descriptor back to IDLE */
	desc->status = EH_CDESC_IDLE;
	atomic_dec(&eh_dev->nr_request);
	clear_eh_congested();

	update_fifo_complete_index(eh_dev);
	return ret;
}

static int eh_process_compress(struct eh_device *eh_dev)
{
	int ret = 0;
	unsigned int start = eh_dev->complete_index;
	unsigned int end = fifo_next_complete_index(eh_dev);
	unsigned int i, index;

	for (i = start; i != end; i = (i + 1) & eh_dev->fifo_color_mask) {
		index = i & eh_dev->fifo_index_mask;
		ret = eh_process_completed_descriptor(eh_dev, index);
		if (ret)
			break;
	}

	return ret;
}

static void eh_abort_incomplete_descriptors(struct eh_device *eh_dev)
{
	unsigned short new_complete_index, masked_write_index;
	int i;

	masked_write_index = eh_dev->write_index & eh_dev->fifo_index_mask;
	new_complete_index = fifo_next_complete_index(eh_dev) &
						 eh_dev->fifo_index_mask;

	for (i = new_complete_index; i != masked_write_index;
	     i = (i + 1) & eh_dev->fifo_index_mask) {
		struct eh_completion *compl = &eh_dev->completions[i];

		(*eh_dev->comp_callback)(EH_CDESC_ERROR_HALTED, NULL, 0,
					  compl->priv);
		compl->priv = NULL;
	}
}

static int eh_comp_thread(void *data)
{
	struct eh_device *eh_dev = data;

	current->flags |= PF_MEMALLOC;

	while (!kthread_should_stop()) {
		wait_event_freezable(eh_dev->comp_wq,
				atomic_read(&eh_dev->nr_request) > 0);
		if (unlikely(eh_process_compress(eh_dev))) {
			unsigned long error;

			error = eh_read_register(eh_dev, EH_REG_ERR_COND);
			if (error) {
				pr_err("error condition interrupt non-zero 0x%llx\n",
				       error);
				eh_dump_regs(eh_dev);
				eh_abort_incomplete_descriptors(eh_dev);
				break;
			}

			/*
			 * The error from fifo descriptor also should be also
			 * propagated by error register.
			 */
			WARN_ON(1);
		}
	}

	return 0;
}

/* Initialize SW related stuff */
static int eh_sw_init(struct eh_device *eh_dev, int error_irq)
{
	int ret;

	/* the error interrupt */
	ret = request_threaded_irq(error_irq, NULL, eh_error_irq, IRQF_ONESHOT,
				   EH_ERR_IRQ, eh_dev);
	if (ret) {
		pr_err("unable to request irq %u ret %d\n", error_irq, ret);
		return ret;
	}
	eh_dev->error_irq = error_irq;

	atomic_set(&eh_dev->nr_request, 0);
	init_waitqueue_head(&eh_dev->comp_wq);

	eh_dev->comp_thread = kthread_run(eh_comp_thread, eh_dev, "eh_comp_thread");
	if (IS_ERR(eh_dev->comp_thread)) {
		ret = PTR_ERR(eh_dev->comp_thread);
		goto free_irq;
	}

	spin_lock(&eh_dev_list_lock);
	list_add(&eh_dev->eh_dev_list, &eh_dev_list);
	spin_unlock(&eh_dev_list_lock);

	return 0;

free_irq:
	free_irq(eh_dev->error_irq, eh_dev);

	return ret;
}

/* cleanup compression related stuff */
static void eh_deinit_compression(struct eh_device *eh_dev)
{
	if (eh_dev->compr_buffers) {
		int i;

		for (i = 0; i < eh_dev->fifo_size; i++) {
			if (eh_dev->compr_buffers[i]) {
				free_pages((unsigned long)eh_dev->compr_buffers[i], 0);
				eh_dev->compr_buffers[i] = NULL;
			}
		}
		kfree(eh_dev->compr_buffers);
		eh_dev->compr_buffers = NULL;
	}

	if (eh_dev->completions) {
		kfree(eh_dev->completions);
		eh_dev->completions = NULL;
	}

	if (eh_dev->fifo_alloc) {
		kfree(eh_dev->fifo_alloc);
		eh_dev->fifo_alloc = NULL;
	}
}

/* initialize compression fifo and related stuff */
static int eh_init_compression(struct eh_device *eh_dev, unsigned short fifo_size)
{
	int i, ret = 0;
	unsigned int desc_size = EH_COMPRESS_DESC_SIZE;

	spin_lock_init(&eh_dev->fifo_prod_lock);

	eh_dev->fifo_size = fifo_size;
	eh_dev->fifo_index_mask = fifo_size - 1;
	eh_dev->fifo_color_mask = (fifo_size << 1) - 1;
	eh_dev->write_index = eh_dev->complete_index = 0;

	eh_dev->completions = kzalloc(fifo_size * sizeof(struct eh_completion),
				      GFP_KERNEL);
	if (!eh_dev->completions) {
		return -ENOMEM;
	}

	/* driver allocates fifo in regular memory - dma coherent case */
	eh_dev->fifo_alloc = kzalloc(fifo_size * (desc_size + 1),
				     GFP_KERNEL | GFP_DMA);
	if (!eh_dev->fifo_alloc) {
		ret = -ENOMEM;
		goto out_cleanup;
	}

	eh_dev->fifo = PTR_ALIGN(eh_dev->fifo_alloc, desc_size);
	eh_dev->compr_buffers = kzalloc(fifo_size * sizeof(void *),
					GFP_KERNEL);
	if (!eh_dev->compr_buffers) {
		ret = -ENOMEM;
		goto out_cleanup;
	}

	for (i = 0; i < fifo_size; i++) {
		void *buf = (void *)__get_free_pages(GFP_KERNEL, 0);
		if (!buf) {
			ret = -ENOMEM;
			goto out_cleanup;
		}
		eh_dev->compr_buffers[i] = buf;
	}

	init_compression_descriptor(eh_dev);
	return ret;

out_cleanup:
	eh_deinit_compression(eh_dev);
	pr_err("failed to init fifo %d\n", ret);
	return ret;
}

static void eh_deinit_decompression(struct eh_device *eh_dev)
{
	int cpu;
	unsigned long buf;
	for_each_possible_cpu(cpu) {
		buf = *per_cpu_ptr(eh_dev->bounce_buffer, cpu);
		if (buf) {
			free_pages(buf, 0);
			*per_cpu_ptr(eh_dev->bounce_buffer, cpu) = 0;
		}
	}
	free_percpu(eh_dev->bounce_buffer);
	eh_dev->bounce_buffer = NULL;
}

static int eh_init_decompression(struct eh_device *eh_dev)
{
	int cpu, ret = 0;

	eh_dev->bounce_buffer = alloc_percpu(unsigned long);
	if (!eh_dev->bounce_buffer)
		return -ENOMEM;

	for_each_possible_cpu(cpu) {
		unsigned long buf = __get_free_pages(GFP_KERNEL, 0);
		if (!buf) {
			ret = -ENOMEM;
			goto out_cleanup;
		}
		*per_cpu_ptr(eh_dev->bounce_buffer, cpu) = buf;
	}

	return ret;

out_cleanup:
	eh_deinit_decompression(eh_dev);

	return ret;
}

static void eh_hw_deinit(struct eh_device *eh_dev)
{
	eh_deinit_decompression(eh_dev);
	eh_deinit_compression(eh_dev);
	iounmap(eh_dev->regs);
	eh_dev->regs = NULL;
}

static void eh_sw_deinit(struct eh_device *eh_dev)
{
	if (eh_dev->error_irq) {
		free_irq(eh_dev->error_irq, eh_dev);
		eh_dev->error_irq = 0;
	}

	if (eh_dev->comp_thread) {
		kthread_stop(eh_dev->comp_thread);
		eh_dev->comp_thread = NULL;
	}
}

/* Initialize HW related stuff */
static int eh_hw_init(struct eh_device *eh_dev, unsigned short fifo_size,
		      phys_addr_t regs, unsigned short quirks)
{
	int ret;
	unsigned long feature;

	eh_dev->quirks = quirks;

	eh_dev->regs = ioremap(regs, EH_REGS_SIZE);
	if (!eh_dev->regs)
		return -ENOMEM;

	feature = eh_read_register(eh_dev, EH_REG_HWFEATURES2);
	eh_dev->decompr_cmd_count = EH_FEATURES2_DECOMPR_CMDS(feature);

	/*
	 * Since EH uses per-cpu mapping for decompression bounce buffer, it
	 * couldn't support if the number of CPUs is greater than the number
	 * of decompression command register.
	 */
	if (eh_dev->decompr_cmd_count > num_possible_cpus()) {
		pr_err("Too many cpus to support EH decompresion: cpus %d decopmrcmd %d\n",
		       num_possible_cpus(), eh_dev->decompr_cmd_count);
		ret = -EINVAL;
		goto iounmap;
	}

	if (eh_init_compression(eh_dev, fifo_size)) {
		ret = -EINVAL;
		goto iounmap;
	}

	if (eh_init_decompression(eh_dev)) {
		ret = -EINVAL;
		goto deinit_compr;
	}

	/* reset the block */
	if (eh_reset(eh_dev)) {
		ret = -ETIMEDOUT;
		goto deinit_decompr;
	}

	/* set up the fifo and enable */
	eh_compr_fifo_init(eh_dev);

	/* enable all the interrupts */
	eh_write_register(eh_dev, EH_REG_INTRP_MASK_ERROR, 0);

	return 0;

deinit_decompr:
	eh_deinit_decompression(eh_dev);
deinit_compr:
	eh_deinit_compression(eh_dev);
iounmap:
	iounmap(eh_dev->regs);
	eh_dev->regs = NULL;

	pr_err("failed to eh_hw_init %d\n", ret);
	return ret;
}

static void eh_deinit(struct eh_device *eh_dev)
{
	eh_hw_deinit(eh_dev);
	eh_sw_deinit(eh_dev);
}

/* EmeraldHill initialization entry */
static int eh_init(struct device *device, struct eh_device *eh_dev,
		   unsigned short fifo_size, phys_addr_t regs, int error_irq,
		   unsigned short quirks)
{
	int ret;

	/* verify fifo_size is a power of two and less than 32k */
	if (!fifo_size || __ffs(fifo_size) != __fls(fifo_size) ||
	    (fifo_size > EH_MAX_FIFO_SIZE)) {
		pr_err("invalid fifo size %u\n", fifo_size);
		return -EINVAL;
	}

	ret = eh_hw_init(eh_dev, fifo_size, regs, quirks);
	if (ret)
		return ret;

	ret = eh_sw_init(eh_dev, error_irq);
	if (ret) {
		eh_hw_deinit(eh_dev);
		return ret;
	}

	return 0;
}

static void eh_setup_dcmd(struct eh_device *eh_dev, unsigned int index,
			  void *src, unsigned int slen, struct page *dst_page)
{
	void *src_vaddr;
	phys_addr_t src_paddr;
	unsigned long alignment;
	unsigned long csize_data;
	unsigned long src_data;
	unsigned long dst_data;

	/*
	 * EH can accept only aligned source buffers for decompression
	 *
	 * Compressed data buffer must be one of:
	 *   64B aligned, max 64B of data
	 *  128B aligned, max 128B of data
	 *  256B aligned, max 256B of data
	 *  512B aligned, max 512B of data
	 * 1024B aligned, max 1024B of data
	 * 2048B aligned, max 2048B of data
	 * 4096B aligned, max 4096B of data
	 */
	alignment = 1UL << __ffs((unsigned long)src);
	if (alignment < 64 || slen > alignment) {
		src_vaddr = (void *)(*per_cpu_ptr(eh_dev->bounce_buffer, index));
		memcpy(src_vaddr, src, slen);
		src_paddr = virt_to_phys(src_vaddr);
		alignment = PAGE_SIZE;
	} else {
		src_paddr = virt_to_phys(src);
		if (alignment > PAGE_SIZE)
			alignment = PAGE_SIZE;
	}

	csize_data = slen << EH_DCMD_CSIZE_SIZE_SHIFT;
	eh_write_register(eh_dev, EH_REG_DCMD_CSIZE(index), csize_data);

#ifdef CONFIG_GOOGLE_EH_DCMD_STATUS_IN_MEMORY
	eh_dev->decompr_status[index] = EH_DCMD_PENDING
					<< EH_DCMD_DEST_STATUS_SHIFT;
	eh_write_register(eh_dev, EH_REG_DCMD_RES(index),
			  1UL << 63 |
				  virt_to_phys(&eh_dev->decompr_status[index]));
#endif

	src_data = (__ffs(alignment) - 5) << EH_DCMD_BUF_SIZE_SHIFT;
	src_data |= src_paddr;
	eh_write_register(eh_dev, EH_REG_DCMD_BUF0(index), src_data);
	eh_write_register(eh_dev, EH_REG_DCMD_BUF1(index), 0);
	eh_write_register(eh_dev, EH_REG_DCMD_BUF2(index), 0);
	eh_write_register(eh_dev, EH_REG_DCMD_BUF3(index), 0);

	dst_data = page_to_phys(dst_page);
	dst_data |= ((unsigned long)EH_DCMD_PENDING)
		    << EH_DCMD_DEST_STATUS_SHIFT;
	eh_write_register(eh_dev, EH_REG_DCMD_DEST(index), dst_data);
}

int eh_compress_page(struct eh_device *eh_dev, struct page *page, void *priv)
{
	unsigned int write_idx;
	struct eh_completion *compl;

try_again:
	spin_lock(&eh_dev->fifo_prod_lock);

	if (fifo_full(eh_dev)) {
		spin_unlock(&eh_dev->fifo_prod_lock);
		cond_resched();
		eh_congestion_wait(HZ/10);
		goto try_again;
	}

	write_idx = fifo_write_index(eh_dev);

	eh_setup_descriptor(eh_dev, page, write_idx);

	compl = &eh_dev->completions[write_idx];
	compl->priv = priv;

	atomic_inc(&eh_dev->nr_request);
	wake_up(&eh_dev->comp_wq);

	/* write barrier to force writes to be visible everywhere */
	wmb();
	update_fifo_write_index(eh_dev);
	spin_unlock(&eh_dev->fifo_prod_lock);

	return 0;
}
EXPORT_SYMBOL(eh_compress_page);

/*
 * eh_decompress_page
 *
 * Decompress a page synchronously. Uses polling for completion.
 *
 * Holds a spinlock for the entire operation, so that nothing can interrupt it.
 */
int eh_decompress_page(struct eh_device *eh_dev, void *src,
		       unsigned int slen, struct page *page)
{
	int ret = 0;
	int index;
	unsigned long timeout;
	unsigned long status;

	/*
	 * Since it uses per-cpu bounce buffer, it doesn't allow to be called
	 * interrupt context.
	 */
	WARN_ON(in_interrupt());

	index = get_cpu();
	pr_devel("[%s]: submit: cpu %u slen %u\n", current->comm, index, slen);

	/* program decompress register (no IRQ) */
	eh_setup_dcmd(eh_dev, index, src, slen, page);

	timeout = jiffies + msecs_to_jiffies(EH_POLL_DELAY_MS);
	do {
		cpu_relax();
		if (time_after(jiffies, timeout)) {
			pr_err("poll timeout on decompression\n");
			eh_dump_regs(eh_dev);
			ret = -ETIME;
			goto out;
		}
		status = eh_read_dcmd_status(eh_dev, index);
	} while (status == EH_DCMD_PENDING);

	pr_devel("dcmd [%u] status = %u\n", index, status);

	if (status != EH_DCMD_DECOMPRESSED) {
		pr_err("dcmd [%u] bad status %u\n", index, status);
		eh_dump_regs(eh_dev);
		ret = -EIO;
	}

out:
	put_cpu();
	return ret;
}
EXPORT_SYMBOL(eh_decompress_page);

struct eh_device *eh_create(eh_cb_fn comp)
{
	struct eh_device *ret = ERR_PTR(-ENODEV);

	spin_lock(&eh_dev_list_lock);
	if (!list_empty(&eh_dev_list)) {
		ret = list_first_entry(&eh_dev_list, struct eh_device,
				       eh_dev_list);
		list_del(&ret->eh_dev_list);
	}
	spin_unlock(&eh_dev_list_lock);
	if (IS_ERR(ret))
		return ret;

	ret->comp_callback = comp;

	return ret;
}
EXPORT_SYMBOL(eh_create);

void eh_destroy(struct eh_device *eh_dev)
{
	eh_dev->comp_callback = NULL;
	spin_lock(&eh_dev_list_lock);
	list_add(&eh_dev->eh_dev_list, &eh_dev_list);
	spin_unlock(&eh_dev_list_lock);
}
EXPORT_SYMBOL(eh_destroy);

#ifdef CONFIG_OF
static int eh_of_probe(struct platform_device *pdev)
{
	struct eh_device *eh_dev;
	struct resource *mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	int ret;
	int error_irq = 0;
	unsigned short quirks = 0;
	struct clk *clk;

	pr_info("starting probing\n");

	pm_runtime_enable(&pdev->dev);
	ret = pm_runtime_get_sync(&pdev->dev);
	if (ret) {
		dev_err(&pdev->dev, "pm_runtime_get_sync returned %d\n", ret);
		goto disable_pm_runtime;
	}

	error_irq = irq_of_parse_and_map(pdev->dev.of_node, 0);
	if (error_irq == 0) {
		ret = -EINVAL;
		goto put_pm_runtime;
	}

	clk = of_clk_get_by_name(pdev->dev.of_node, "eh-clock");
	if (IS_ERR(clk)) {
		ret = PTR_ERR(clk);
		goto put_pm_runtime;
	}

	ret = clk_prepare_enable(clk);
	if (ret)
		goto put_clk;

	if (of_get_property(pdev->dev.of_node, "google,eh,ignore-gctrl-reset",
			    NULL))
		quirks |= EH_QUIRK_IGNORE_GCTRL_RESET;

	eh_dev = kzalloc(sizeof(*eh_dev), GFP_KERNEL);
	if (!eh_dev) {
		ret = -ENOMEM;
		goto put_disable_clk;
	}

	ret = eh_init(&pdev->dev, eh_dev, eh_default_fifo_size, mem->start,
			 error_irq, quirks);
	if (ret)
		goto free_ehdev;

	eh_dev->clk = clk;
	platform_set_drvdata(pdev, eh_dev);

	pr_info("starting probing done\n");
	return 0;

free_ehdev:
	kfree(eh_dev);
put_disable_clk:
	clk_disable_unprepare(clk);
put_clk:
	clk_put(clk);
put_pm_runtime:
	pm_runtime_put_sync(&pdev->dev);
disable_pm_runtime:
	pm_runtime_disable(&pdev->dev);

	pr_err("Fail to probe %d\n", ret);
	return ret;
}

void eh_remove(struct eh_device *eh_dev)
{
	eh_deinit(eh_dev);
	kfree(eh_dev);
}

static int eh_of_remove(struct platform_device *pdev)
{
	struct eh_device *eh_dev = platform_get_drvdata(pdev);

	clk_disable_unprepare(eh_dev->clk);
	clk_put(eh_dev->clk);
	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	eh_remove(eh_dev);
	return 0;
}

static int eh_suspend(struct device *dev)
{
	unsigned long data;
	struct eh_device *eh_dev = dev_get_drvdata(dev);

	/* check pending work */
	if (atomic_read(&eh_dev->nr_request) > 0) {
		pr_warn("block suspend (compression pending)\n");
		return -EBUSY;
	}

	/* disable all interrupts */
	eh_write_register(eh_dev, EH_REG_INTRP_MASK_ERROR, ~0UL);
	eh_write_register(eh_dev, EH_REG_INTRP_MASK_CMP, ~0UL);
	eh_write_register(eh_dev, EH_REG_INTRP_MASK_DCMP, ~0UL);

	/* disable compression FIFO */
	data = eh_read_register(eh_dev, EH_REG_CDESC_CTRL);
	data &= ~(1UL << EH_CDESC_CTRL_COMPRESS_ENABLE_SHIFT);
	eh_write_register(eh_dev, EH_REG_CDESC_CTRL, data);

	/* disable EH clock */
	clk_disable_unprepare(eh_dev->clk);
	dev_dbg(dev, "EH suspended\n");

	return 0;
}

static int eh_resume(struct device *dev)
{
	struct eh_device *eh_dev = dev_get_drvdata(dev);

	/* re-enable EH clock */
	clk_prepare_enable(eh_dev->clk);

	/* re-enable compression FIFO */
	eh_compr_fifo_init(eh_dev);

	/* re-enable all interrupts */
	eh_write_register(eh_dev, EH_REG_INTRP_MASK_ERROR, 0);
	eh_write_register(eh_dev, EH_REG_INTRP_MASK_CMP, 0);
	eh_write_register(eh_dev, EH_REG_INTRP_MASK_DCMP, 0);

	dev_dbg(dev, "EH resumed\n");
	return 0;
}

static const struct dev_pm_ops eh_pm_ops = {
	.suspend = eh_suspend,
	.resume = eh_resume,
};

static const struct of_device_id eh_of_match[] = {
	{ .compatible = "google,eh", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, eh_of_match);

static struct platform_driver eh_of_driver = {
	.probe		= eh_of_probe,
	.remove		= eh_of_remove,
	.driver		= {
		.name	= "eh",
		.pm	= &eh_pm_ops,
		.of_match_table = of_match_ptr(eh_of_match),
	},
};

module_platform_driver(eh_of_driver);
#endif

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Petri Gynther <pgynther@google.com>");
MODULE_DESCRIPTION("Emerald Hill compression engine driver");
