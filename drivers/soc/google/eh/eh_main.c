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
#include "zcomp.h"
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
#include <uapi/linux/sched/types.h>

#include <soc/google/pkvm-s2mpu.h>

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
static struct eh_device *eh_dev_g;

static unsigned int eh_default_fifo_size = 4096;

static bool sw_fifo_empty(struct eh_sw_fifo *fifo)
{
	return !fifo->has_reqs;
}

/*
 * - Primitive functions for Emerald Hill HW
 */
static void eh_dump_regs(struct eh_device *eh_dev)
{
	unsigned int i, offset = 0;

	pr_err("dump_regs: global\n");
	for (offset = EH_REG_HWID; offset <= EH_REG_ERR_MSK; offset += 8)
		pr_err("0x%03X: 0x%016llX\n", offset,
			readq(eh_dev->regs + offset));

	pr_err("dump_regs: compression\n");
	for (offset = EH_REG_CDESC_LOC; offset <= EH_REG_CINTERP_CTRL;
	     offset += 8)
		pr_err("0x%03X: 0x%016llX\n", offset,
			readq(eh_dev->regs + offset));

	for (i = 0; i < eh_dev->decompr_cmd_count; i++) {
		pr_err("dump_regs: decompression %u\n", i);
		for (offset = EH_REG_DCMD_CSIZE(i);
		     offset <= EH_REG_DCMD_BUF3(i); offset += 8)
			pr_err("0x%03X: 0x%016llX\n", offset,
				readq(eh_dev->regs + offset));
	}

	pr_err("dump_regs: vendor\n");
	for (offset = EH_REG_BUSCFG; offset <= 0x118; offset += 8)
		pr_err("0x%03X: 0x%016llX\n", offset,
			readq(eh_dev->regs + offset));

	pr_err("driver\n");
	pr_err("write_index %u complete_index %u\n",
	       eh_dev->write_index, eh_dev->complete_index);
	pr_err("pending_compression %d\n", atomic_read(&eh_dev->nr_request));
}

static inline unsigned int fifo_write_index(struct eh_device *eh_dev)
{
	return eh_dev->write_index & eh_dev->fifo_index_mask;
}

static inline void update_fifo_write_index(struct eh_device *eh_dev)
{
	unsigned int next_write_idx = (eh_dev->write_index + 1) &
				       eh_dev->fifo_color_mask;

	eh_dev->write_index = next_write_idx;
	writeq(next_write_idx, eh_dev->regs + EH_REG_CDESC_WRIDX);
}

static inline void update_fifo_complete_index(struct eh_device *eh_dev)
{
	eh_dev->complete_index = (eh_dev->complete_index + 1) &
				  eh_dev->fifo_color_mask;
}

static bool fifo_full(struct eh_device *eh_dev)
{
	return atomic_read(&eh_dev->nr_request) == eh_dev->fifo_size;
}

/* index of the next descriptor to be completed by hardware */
static unsigned int fifo_next_complete_index(struct eh_device *eh_dev)
{
	return readq(eh_dev->regs + EH_REG_CDESC_CTRL) &
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
	status = readq(eh_dev->regs + EH_REG_DCMD_DEST(index));
#endif
	return EH_DCMD_DEST_STATUS(status);
}

static int eh_reset(struct eh_device *eh_dev)
{
	int trial;

	if (eh_dev->quirks & EH_QUIRK_IGNORE_GCTRL_RESET)
		return 0;

	writeq(-1, eh_dev->regs + EH_REG_GCTRL);
	for (trial = 0; trial < EH_RESET_MAX_TRIAL; trial++) {
		if (!readq(eh_dev->regs + EH_REG_GCTRL))
			return 0;
		udelay(EH_RESET_DELAY_US);
	}

	return 1;
}

static void eh_setup_src_addr(struct eh_compress_desc *desc, struct page *page)
{
	phys_addr_t src_paddr = page_to_phys(page);

	/* Update the source address field in the descriptor */
	desc->src_addr &= ~EH_COMP_SRC_ADDR_MASK;
	desc->src_addr |= (src_paddr & EH_COMP_SRC_ADDR_MASK);
}
static void eh_setup_descriptor(struct eh_device *eh_dev, struct page *src_page,
				unsigned int index)
{
	struct eh_compress_desc *desc;

	desc = eh_descriptor(eh_dev, index);
	/* Set the source address */
	eh_setup_src_addr(desc, src_page);
	/* mark it as pend for hardware */
	desc->status = EH_CDESC_PENDING;
	desc->intr_request = 1;
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
	writeq(data, eh_dev->regs + EH_REG_CDESC_CTRL);
	do {
		udelay(1);
		data = readq(eh_dev->regs + EH_REG_CDESC_CTRL);
	} while (data & (1UL << EH_CDESC_CTRL_FIFO_RESET));

	/* reset software copies of index registers */
	eh_dev->write_index = 0;
	eh_dev->complete_index = 0;

	/* program FIFO memory location and size */
	data = (unsigned long)virt_to_phys(eh_dev->fifo) | __ffs(eh_dev->fifo_size);
	writeq(data, eh_dev->regs + EH_REG_CDESC_LOC);

	/* enable compression */
	data = 1UL << EH_CDESC_CTRL_COMPRESS_ENABLE_SHIFT;
	writeq(data, eh_dev->regs + EH_REG_CDESC_CTRL);
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
static void request_to_sw_fifo(struct eh_device *eh_dev, struct page *page,
			       struct zcomp_cookie *cookie)
{
	struct eh_sw_fifo *fifo = &eh_dev->sw_fifo;

	spin_lock(&fifo->lock);
	list_add_tail(&cookie->list, &fifo->head);
	fifo->has_reqs = true;
	spin_unlock(&fifo->lock);

	/* spin_unlock() provides a barrier before waitqueue_active() */
	if (waitqueue_active(&eh_dev->comp_wq))
		wake_up(&eh_dev->comp_wq);
}

static int request_to_hw_fifo(struct eh_device *eh_dev,
			      struct zcomp_cookie *cookie, bool wake_up)
{
	unsigned int write_idx;
	struct eh_completion *compl;

	/* Check if the fifo is full locklessly first, to elide the lock */
	if (fifo_full(eh_dev))
		return -EBUSY;

	spin_lock(&eh_dev->fifo_prod_lock);
	if (fifo_full(eh_dev)) {
		spin_unlock(&eh_dev->fifo_prod_lock);
		return -EBUSY;
	}

	write_idx = fifo_write_index(eh_dev);

	eh_setup_descriptor(eh_dev, cookie->page, write_idx);

	compl = &eh_dev->completions[write_idx];
	compl->priv = cookie;

	/*
	 * Ensure nr_request is incremented before kicking off compression, so
	 * that the correct order is observed by the compression thread. This
	 * isn't needed when running from the compression thread itself.
	 */
	atomic_inc(&eh_dev->nr_request);
	if (wake_up)
		smp_mb__after_atomic();

	update_fifo_write_index(eh_dev);
	spin_unlock(&eh_dev->fifo_prod_lock);

	/* spin_unlock() provides a barrier before waitqueue_active() */
	if (wake_up && waitqueue_active(&eh_dev->comp_wq))
		wake_up(&eh_dev->comp_wq);

	return 0;
}

static void refill_hw_fifo(struct eh_device *eh_dev)
{
	struct eh_sw_fifo *fifo = &eh_dev->sw_fifo;
	struct zcomp_cookie *c;
	int ret;

	spin_lock(&fifo->lock);
	while ((c = list_first_entry_or_null(&fifo->head, typeof(*c), list))) {
		/*
		 * Take the cookie off the list since it can't be touched once
		 * it's passed onto the compression thread.
		 */
		list_del(&c->list);
		spin_unlock(&fifo->lock);

		/* Attempt to pass the cookie onto the hardware fifo */
		ret = request_to_hw_fifo(eh_dev, c, false);

		spin_lock(&fifo->lock);
		if (ret) {
			/* Add the cookie back to the front */
			list_add(&c->list, &fifo->head);
			break;
		}
	}
	if (!c)
		fifo->has_reqs = false;
	spin_unlock(&fifo->lock);
}

static irqreturn_t eh_error_irq(int irq, void *data)
{
	struct eh_device *eh_dev = data;
	unsigned long compr, decompr, error;

	compr = readq(eh_dev->regs + EH_REG_INTRP_STS_CMP);
	decompr = readq(eh_dev->regs + EH_REG_INTRP_STS_DCMP);
	error = readq(eh_dev->regs + EH_REG_INTRP_STS_ERROR);

	pr_err("irq %d error 0x%lx compr 0x%lx decompr 0x%lx\n",
	       irq, error, compr, decompr);

	if (error) {
		pr_err("error interrupt was active\n");
		eh_dump_regs(eh_dev);
		writeq(error, eh_dev->regs + EH_REG_INTRP_STS_ERROR);
	}

	return IRQ_HANDLED;
}

static void eh_clear_cintr_status(struct eh_device *eh_dev)
{
	/*
	 * Loop until the interrupt status is fully cleared in hardware. The
	 * writeq() can be relaxed since there is a control dependency on the
	 * same address, plus readq() still executes a read barrier afterwards
	 * which prevents any later loads from being hoisted before the readq().
	 */
	do {
		writeq_relaxed(1, eh_dev->regs + EH_REG_INTRP_STS_CMP);
	} while (readq(eh_dev->regs + EH_REG_INTRP_STS_CMP));
}

static irqreturn_t eh_compress_irq(int irq, void *data)
{
	struct eh_device *eh_dev = data;

	/* Clear the interrupt status */
	eh_clear_cintr_status(eh_dev);

	/* Mask the interrupt and wake the compression thread if it's waiting */
	if (swait_active(&eh_dev->cirq_wq)) {
		writeq_relaxed(~0UL, eh_dev->regs + EH_REG_INTRP_MASK_CMP);
		swake_up_one(&eh_dev->cirq_wq);
	}
	return IRQ_HANDLED;
}

static irqreturn_t eh_decompress_irq(int irq, void *data)
{
	struct eh_device *eh_dev = eh_dev_g;
	int index = (int)(long)data;

	/* Tell the waiter that this decompression engine completed its work */
	writeq_relaxed(BIT(index), eh_dev->regs + EH_REG_INTRP_STS_DCMP);
	complete(per_cpu_ptr(eh_dev->decomp_done, index));
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
		ret = -EINVAL;
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

	/* Ensure the fifo slot is all freed before decrementing nr_request */
	smp_mb__before_atomic();
	atomic_dec(&eh_dev->nr_request);

	update_fifo_complete_index(eh_dev);
	return ret;
}

static unsigned int eh_wait_next_index(struct eh_device *eh_dev, unsigned int i)
{
	DECLARE_SWAITQUEUE(wait);
	unsigned int end;

	/* Check if there are compressions finished before attempting to wait */
	if ((end = fifo_next_complete_index(eh_dev)) != i)
		return end;

	/*
	 * We need to make sure the IRQ handler isn't running if we decided not
	 * to wait for it to wake us up. This prevents a race where the IRQ
	 * handler may be running the next time eh_wait_next_index() is called,
	 * such that the IRQ handler masks the interrupt right after we unmask
	 * it. This can lead to the compression thread sleeping forever waiting
	 * for an interrupt that'll never come because the interrupt is masked.
	 */
	if (eh_dev->sync_comp_irq) {
		eh_dev->sync_comp_irq = false;
		synchronize_irq(eh_dev->comp_irq);
	}

	/*
	 * In order to reduce the number of compression interrupts fired as much
	 * as possible, the compression interrupt is only enabled on demand when
	 * we observe that there aren't any compressions completed, and
	 * therefore need to wait. This is extremely racy because we avoid doing
	 * synchronization with the IRQ handler itself, though it is very fast
	 * and efficient!
	 *
	 * Now, prepare to wait for the interrupt to signal compression
	 * completion. We must recheck fifo_next_complete_index() _after_ the
	 * interrupt is unmasked and _before_ sleeping to avoid a race:
	 *
	 *  1. fifo_next_complete_index() == i, need to wait for completion
	 *  2. fifo_next_complete_index() changes but interrupt is masked
	 *  3. interrupt is unmasked
	 *  4. we sleep on cirq_wq and never receive an interrupt to wake up
	 *
	 * This can be avoided by checking fifo_next_complete_index() before
	 * calling schedule() to sleep. This check must be done _after_
	 * preparing to wait, so that swait_active() is true and therefore the
	 * IRQ handler observes that we are waiting.
	 */
	prepare_to_swait_exclusive(&eh_dev->cirq_wq, &wait, TASK_IDLE);

	/* Clear the interrupt status and then unmask the interrupt */
	eh_clear_cintr_status(eh_dev);
	writeq(0, eh_dev->regs + EH_REG_INTRP_MASK_CMP);

	/*
	 * The interrupt unmask must occur _before_ reading the next completed
	 * index to avoid the race described above.
	 */
	__iomb();

	/* Recheck the next completed index and sleep if nothing's ready */
	if ((end = fifo_next_complete_index(eh_dev)) != i) {
		/* Didn't need to wait for the interrupt, so mask it again */
		writeq_relaxed(~0UL, eh_dev->regs + EH_REG_INTRP_MASK_CMP);
		eh_dev->sync_comp_irq = true;
	} else {
		/*
		 * Wait for the interrupt. No need to mask the interrupt
		 * afterwards because it'll mask itself just before waking us.
		 */
		schedule();
		end = fifo_next_complete_index(eh_dev);
	}
	finish_swait(&eh_dev->cirq_wq, &wait);

	return end;
}

static int eh_process_compress(struct eh_device *eh_dev)
{
	unsigned int i = eh_dev->complete_index, end, index;
	int ret;

	/* Flush sw_fifo in case hw_fifo is empty */
	if (!atomic_read(&eh_dev->nr_request))
		refill_hw_fifo(eh_dev);

	do {
		/* Wait for the next completed index */
		end = eh_wait_next_index(eh_dev, i);

		/* Process the completed compression requests */
		do {
			index = i & eh_dev->fifo_index_mask;
			ret = eh_process_completed_descriptor(eh_dev, index);
			if (ret)
				return ret;
			/*
			 * Since we have available space in hw_fifo, put the
			 * next compression request immediately from sw_fifo to
			 * make EH busy.
			 */
			refill_hw_fifo(eh_dev);
		} while ((i = (i + 1) & eh_dev->fifo_color_mask) != end);
	} while (atomic_read(&eh_dev->nr_request));

	return 0;
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

static int __noreturn eh_comp_thread(void *data)
{
	struct eh_device *eh_dev = data;

	sched_set_fifo_low(current);
	current->flags |= PF_MEMALLOC;
	set_freezable();

	while (1) {
		int ret;

		wait_event_freezable(eh_dev->comp_wq,
			atomic_read(&eh_dev->nr_request) ||
			!sw_fifo_empty(&eh_dev->sw_fifo));

		ret = eh_process_compress(eh_dev);
		if (unlikely(ret < 0)) {
			unsigned long error;

			error = readq(eh_dev->regs + EH_REG_ERR_COND);
			if (error) {
				pr_err("error condition interrupt non-zero 0x%lx\n",
				       error);
				eh_dump_regs(eh_dev);
				eh_abort_incomplete_descriptors(eh_dev);
				BUG();
			}

			/*
			 * The error from fifo descriptor also should be also
			 * propagated by error register.
			 */
			WARN_ON(1);
		}
	}
}

/* Initialize SW related stuff */
static int eh_sw_init(struct eh_device *eh_dev, const int *irqs, int nr_irqs)
{
	int error_irq = irqs[0], i, ret;

	spin_lock_init(&eh_dev->sw_fifo.lock);
	INIT_LIST_HEAD(&eh_dev->sw_fifo.head);
	init_swait_queue_head(&eh_dev->cirq_wq);

	/* An IRQ is required for compression */
	if (nr_irqs < 2)
		return -EINVAL;

	/* Request the compression IRQ */
	ret = request_irq(irqs[1], eh_compress_irq, IRQF_NO_THREAD,
			  "eh_compress", eh_dev);
	if (ret) {
		pr_err("comp irq %u request failed, ret %d\n", irqs[1], ret);
		return ret;
	}
	eh_dev->comp_irq = irqs[1];

	/*
	 * Use IRQ-signaled decompression completion on PREEMPT_RT as an
	 * optimization to allow decompression waiters to sleep until
	 * decompression is done. This is only possible on PREEMPT_RT because
	 * the decompression context is only preemptible on PREEMPT_RT.
	 */
	if (IS_ENABLED(CONFIG_PREEMPT_RT)) {
		/* Check that all IRQs are defined */
		if (nr_irqs != (eh_dev->decompr_cmd_count + 2))
			goto free_comp_irq;

		/* Request one IRQ for each decompression engine */
		for (i = 2; i < eh_dev->decompr_cmd_count + 2; i++) {
			ret = request_irq(irqs[i], eh_decompress_irq,
					  IRQF_NO_THREAD, "eh_decompress",
					  (void *)(long)i - 2);
			if (ret) {
				pr_err("decomp irq %u request failed, ret %d\n",
				       irqs[i], ret);
				goto free_decomp_irqs;
			}
		}
	}

	/* the error interrupt */
	ret = request_threaded_irq(error_irq, NULL, eh_error_irq, IRQF_ONESHOT,
				   EH_ERR_IRQ, eh_dev);
	if (ret) {
		pr_err("unable to request irq %u ret %d\n", error_irq, ret);
		goto free_decomp_irqs;
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
free_decomp_irqs:
	if (IS_ENABLED(CONFIG_PREEMPT_RT)) {
		while (i-- > 2)
			free_irq(irqs[i], (void *)(long)i - 2);
	}
free_comp_irq:
	free_irq(irqs[1], eh_dev);
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
	free_percpu(eh_dev->decomp_done);
	free_percpu(eh_dev->bounce_buffer);
	eh_dev->bounce_buffer = NULL;
}

static int eh_init_decompression(struct eh_device *eh_dev)
{
	int cpu, ret = 0;

	eh_dev->bounce_buffer = alloc_percpu(unsigned long);
	if (!eh_dev->bounce_buffer)
		return -ENOMEM;

	if (IS_ENABLED(CONFIG_PREEMPT_RT)) {
		eh_dev->decomp_done = alloc_percpu(struct completion);
		if (!eh_dev->decomp_done) {
			free_percpu(eh_dev->bounce_buffer);
			return -ENOMEM;
		}
	}

	for_each_possible_cpu(cpu) {
		unsigned long buf = __get_free_pages(GFP_KERNEL, 0);
		if (!buf) {
			ret = -ENOMEM;
			goto out_cleanup;
		}
		*per_cpu_ptr(eh_dev->bounce_buffer, cpu) = buf;
		if (IS_ENABLED(CONFIG_PREEMPT_RT))
			init_completion(per_cpu_ptr(eh_dev->decomp_done, cpu));
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

	feature = readq(eh_dev->regs + EH_REG_HWFEATURES2);
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
	writeq(0, eh_dev->regs + EH_REG_INTRP_MASK_ERROR);
	if (IS_ENABLED(CONFIG_PREEMPT_RT))
		writeq(0, eh_dev->regs + EH_REG_INTRP_MASK_DCMP);

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

/* EmeraldHill initialization entry */
static int eh_init(struct device *device, struct eh_device *eh_dev,
		   unsigned short fifo_size, phys_addr_t regs, const int *irqs,
		   int nr_irqs, unsigned short quirks)
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

	ret = eh_sw_init(eh_dev, irqs, nr_irqs);
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
	/*
	 * HW starts decompression only after setting status bits to "PEND",
	 * so we could relax until setting the DEST register.
	 */
	writeq_relaxed(csize_data, eh_dev->regs + EH_REG_DCMD_CSIZE(index));

#ifdef CONFIG_GOOGLE_EH_DCMD_STATUS_IN_MEMORY
	eh_dev->decompr_status[index] = EH_DCMD_PENDING
					<< EH_DCMD_DEST_STATUS_SHIFT;
	writeq_relaxed(1UL << 63 | virt_to_phys(&eh_dev->decompr_status[index]),
		       eh_dev->regs + EH_REG_DCMD_RES(index));
#endif

	src_data = (__ffs(alignment) - 5) << EH_DCMD_BUF_SIZE_SHIFT;
	src_data |= src_paddr;

	/*
	 * Only BUF0 is using now so don't set rest of buffers for performance.
	 * Later, if multiple buffers want to be supported, we need to revisit
	 * here.
	 */
	writeq_relaxed(src_data, eh_dev->regs + EH_REG_DCMD_BUF0(index));

	dst_data = page_to_phys(dst_page);
	dst_data |= ((unsigned long)EH_DCMD_PENDING)
		    << EH_DCMD_DEST_STATUS_SHIFT;
	if (IS_ENABLED(CONFIG_PREEMPT_RT))
		dst_data |= 1UL << EH_DCMD_DEST_INTR_SHIFT;
	writeq(dst_data, eh_dev->regs + EH_REG_DCMD_DEST(index));
}

int eh_compress_page(struct eh_device *eh_dev, struct page *page, void *priv)
{
	/*
	 * If sw_fifo is not empty, it means hw fifo is already full so
	 * don't bother to hw fifo.
	 */
	if (!sw_fifo_empty(&eh_dev->sw_fifo))
		goto req_to_sw_fifo;
	/*
	 * If it fail to add the request into hw fifo, fallback it to
	 * sw fifo.
	 */
	if (!request_to_hw_fifo(eh_dev, priv, true))
		return 0;

req_to_sw_fifo:
	request_to_sw_fifo(eh_dev, page, priv);
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
	struct eh_decomp_llock {
		local_lock_t l;
	};
	static DEFINE_PER_CPU(struct eh_decomp_llock, decomp_llock) = {
		.l = INIT_LOCAL_LOCK(l)
	};
	struct completion *decomp_done;
	unsigned long status, timeout;
	int index, ret = 0;

	/*
	 * Since it uses per-cpu bounce buffer, it doesn't allow to be called
	 * interrupt context.
	 */
	WARN_ON(in_interrupt());

	local_lock(&decomp_llock.l);
	index = raw_smp_processor_id();
	pr_devel("[%s]: submit: cpu %u slen %u\n", current->comm, index, slen);

	if (IS_ENABLED(CONFIG_PREEMPT_RT)) {
		decomp_done = per_cpu_ptr(eh_dev->decomp_done, index);
		reinit_completion(decomp_done);
	}

	/* program decompress register (with IRQ on PREEMPT_RT) */
	eh_setup_dcmd(eh_dev, index, src, slen, page);

	if (IS_ENABLED(CONFIG_PREEMPT_RT)) {
		/* Wait for the IRQ to signal decompression completion */
		wait_for_completion(decomp_done);
		status = eh_read_dcmd_status(eh_dev, index);
	} else {
		/* Busy wait until decompression is complete */
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
	}

	pr_devel("dcmd [%u] status = %lu\n", index, status);

	if (status != EH_DCMD_DECOMPRESSED) {
		pr_err("dcmd [%u] bad status %lu\n", index, status);
		eh_dump_regs(eh_dev);
		ret = -EIO;
	}

out:
	local_unlock(&decomp_llock.l);
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

static int eh_s2mpu_suspend(struct eh_device *eh_dev)
{
	return (IS_ENABLED(CONFIG_PKVM_S2MPU) && eh_dev->s2mpu)
		? pkvm_s2mpu_suspend(eh_dev->s2mpu) : 0;
}

static int eh_s2mpu_resume(struct eh_device *eh_dev)
{
	return (IS_ENABLED(CONFIG_PKVM_S2MPU) && eh_dev->s2mpu)
		? pkvm_s2mpu_resume(eh_dev->s2mpu) : 0;
}

#ifdef CONFIG_OF
static int eh_of_probe(struct platform_device *pdev)
{
	struct eh_device *eh_dev;
	struct resource *mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	int irqs[EH_MAX_DCMD + 2], nr_irqs = 0, ret;
	unsigned short quirks = 0;
	struct clk *clk;
	struct device *s2mpu = NULL;

	if (IS_ENABLED(CONFIG_PKVM_S2MPU)) {
		s2mpu = pkvm_s2mpu_of_parse(&pdev->dev);
		if (IS_ERR(s2mpu)) {
			dev_err(&pdev->dev, "pkvm_s2mpu_of_parse returned: %ld\n",
				PTR_ERR(s2mpu));
			return PTR_ERR(s2mpu);
		}
		if (s2mpu && !pkvm_s2mpu_ready(s2mpu)) {
			return -EPROBE_DEFER;
		}
	}

	pr_info("starting probing\n");

	pm_runtime_enable(&pdev->dev);
	ret = pm_runtime_get_sync(&pdev->dev);
	if (ret) {
		dev_err(&pdev->dev, "pm_runtime_get_sync returned %d\n", ret);
		goto disable_pm_runtime;
	}

	do {
		irqs[nr_irqs] = irq_of_parse_and_map(pdev->dev.of_node, nr_irqs);
	} while (irqs[nr_irqs] && ++nr_irqs < ARRAY_SIZE(irqs));

	/* There should be at least one IRQ (error IRQ) */
	if (!nr_irqs) {
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
	eh_dev_g = eh_dev;

	ret = eh_init(&pdev->dev, eh_dev, eh_default_fifo_size, mem->start,
		      irqs, nr_irqs, quirks);
	if (ret)
		goto free_ehdev;

	eh_dev->s2mpu = s2mpu;
	ret = eh_s2mpu_resume(eh_dev);
	if (ret)
		dev_err(&pdev->dev, "could not resume s2mpu: %d", ret);

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

static int eh_of_remove(struct platform_device *pdev)
{
	struct eh_device *eh_dev = platform_get_drvdata(pdev);

	clk_disable_unprepare(eh_dev->clk);
	clk_put(eh_dev->clk);
	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	return 0;
}

static int eh_suspend(struct device *dev)
{
	unsigned long data;
	struct eh_device *eh_dev = dev_get_drvdata(dev);
	int ret;

	/* check pending work */
	if (atomic_read(&eh_dev->nr_request) > 0) {
		pr_warn("block suspend (compression pending)\n");
		return -EBUSY;
	}

	/* disable all interrupts */
	writeq(~0UL, eh_dev->regs + EH_REG_INTRP_MASK_ERROR);
	writeq(~0UL, eh_dev->regs + EH_REG_INTRP_MASK_DCMP);

	/* disable compression FIFO */
	data = readq(eh_dev->regs + EH_REG_CDESC_CTRL);
	data &= ~(1UL << EH_CDESC_CTRL_COMPRESS_ENABLE_SHIFT);
	writeq(data, eh_dev->regs + EH_REG_CDESC_CTRL);

	/* disable EH clock */
	clk_disable_unprepare(eh_dev->clk);

	ret = eh_s2mpu_suspend(eh_dev);
	if (ret)
		dev_err(dev, "could not suspend s2mpu: %d", ret);

	dev_dbg(dev, "EH suspended\n");

	return 0;
}

static int eh_resume(struct device *dev)
{
	struct eh_device *eh_dev = dev_get_drvdata(dev);
	int ret;

	ret = eh_s2mpu_resume(eh_dev);
	if (ret)
		dev_err(dev, "could not resume s2mpu: %d", ret);

	/* re-enable EH clock */
	clk_prepare_enable(eh_dev->clk);

	/* re-enable compression FIFO */
	eh_compr_fifo_init(eh_dev);

	/* re-enable all interrupts */
	writeq(0, eh_dev->regs + EH_REG_INTRP_MASK_ERROR);
	writeq(0, eh_dev->regs + EH_REG_INTRP_MASK_DCMP);

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
