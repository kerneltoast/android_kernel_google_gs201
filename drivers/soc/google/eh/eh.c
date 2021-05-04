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

#define EH_ERR_IRQ	"eh_error"
#define EH_COMP_IRQ	"eh_comp"

/* list of all unclaimed EH devices */
static LIST_HEAD(eh_dev_list);
static DEFINE_SPINLOCK(eh_dev_list_lock);

static DECLARE_WAIT_QUEUE_HEAD(eh_compress_wait);

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

#ifdef CONFIG_GOOGLE_EH_LATENCY_STAT
static void eh_update_latency(struct eh_device *eh_dev, unsigned long start,
			      unsigned long event_count,
			      enum eh_stat_event type)
{
	unsigned long prev_avg, new_avg;
	unsigned long count;
	unsigned long delta, end;

	WARN_ON(start == 0);

	preempt_disable();

	end = ktime_get_ns();
	WARN_ON(start > end);

	count = __this_cpu_read(eh_dev->stats->events[type]);
	__this_cpu_add(eh_dev->stats->events[type], event_count);

	delta = end - start;
	prev_avg = __this_cpu_read(eh_dev->stats->avg_lat[type]);

	new_avg = (prev_avg * count + delta) / (count + event_count);
	__this_cpu_write(eh_dev->stats->avg_lat[type], new_avg);

	if (delta > __this_cpu_read(eh_dev->stats->max_lat[type]))
		__this_cpu_write(eh_dev->stats->max_lat[type], delta);

	if (delta < __this_cpu_read(eh_dev->stats->min_lat[type]))
		__this_cpu_write(eh_dev->stats->min_lat[type], delta);

	preempt_enable();
}

static inline void set_submit_ts(struct eh_completion *cmpl, unsigned long ts)
{
	cmpl->submit_ts = ts;
}

static inline unsigned long get_submit_ts(struct eh_completion *cmpl)
{
	return cmpl->submit_ts;
}
#else
static inline void eh_update_latency(struct eh_device *eh_dev, unsigned long start,
			      unsigned long event_count,
			      enum eh_stat_event type) {};

static inline void set_submit_ts(struct eh_completion *cmpl, unsigned long ts) {};
static inline unsigned long get_submit_ts(struct eh_completion *cmpl) { return 0; };
#endif
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

	pr_err("eh_dev %px\n", eh_dev);
	pr_err("%s: dump_regs: global\n", eh_dev->name);
	for (offset = EH_REG_HWID; offset <= EH_REG_ERR_MSK; offset += 8)
		pr_err("%s: 0x%03X: 0x%016llX\n", eh_dev->name, offset,
			eh_read_register(eh_dev, offset));

	pr_err("%s: dump_regs: compression\n", eh_dev->name);
	for (offset = EH_REG_CDESC_LOC; offset <= EH_REG_CINTERP_CTRL;
	     offset += 8)
		pr_err("%s: 0x%03X: 0x%016llX\n", eh_dev->name, offset,
			eh_read_register(eh_dev, offset));

	for (i = 0; i < eh_dev->decompr_cmd_count; i++) {
		pr_err("%s: dump_regs: decompression %u\n", eh_dev->name, i);
		for (offset = EH_REG_DCMD_CSIZE(i);
		     offset <= EH_REG_DCMD_BUF3(i); offset += 8)
			pr_err("%s: 0x%03X: 0x%016llX\n", eh_dev->name, offset,
				eh_read_register(eh_dev, offset));
	}

	pr_err("%s: dump_regs: vendor\n", eh_dev->name);
	for (offset = EH_REG_BUSCFG; offset <= 0x118; offset += 8)
		pr_err("%s: 0x%03X: 0x%016llX\n", eh_dev->name, offset,
			eh_read_register(eh_dev, offset));

	pr_err("%s: driver\n", eh_dev->name);
	pr_err("%s: write_index %u complete_index %u\n", eh_dev->name,
				eh_dev->write_index, eh_dev->complete_index);
	pr_err("%s: pending_compression %lu\n", eh_dev->name,
				atomic_read(&eh_dev->nr_request));
}

static int eh_update_complete_index(struct eh_device *eh_dev,
				     bool update_int_idx);

static void eh_abort_incomplete_descriptors(struct eh_device *eh_dev)
{
	unsigned short new_complete_index, masked_write_index;
	int i;

	masked_write_index = eh_dev->write_index & eh_dev->fifo_index_mask;
	new_complete_index = (eh_read_register(eh_dev, EH_REG_CDESC_CTRL) &
			      EH_CDESC_CTRL_COMPLETE_IDX_MASK) &
			     eh_dev->fifo_index_mask;

	for (i = new_complete_index; i != masked_write_index;
	     i = (i + 1) & eh_dev->fifo_index_mask) {
		struct eh_completion *cmpl = &eh_dev->completions[i];

		(*eh_dev->comp_callback)(EH_CDESC_ERROR_HALTED, NULL, 0,
					  cmpl->priv);
		cmpl->priv = NULL;
	}
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

static int eh_comp_thread(void *data)
{
	struct eh_device *eh_dev = data;

	current->flags |= PF_MEMALLOC;

	while (!kthread_should_stop()) {
		wait_event_freezable(eh_dev->comp_wq,
				atomic_read(&eh_dev->nr_request) > 0);
		if (unlikely(eh_update_complete_index(eh_dev, false))) {
			unsigned long error;

			error = eh_read_register(eh_dev, EH_REG_ERR_COND);
			if (error) {
				pr_err("%s: error condition interrupt non-zero 0x%llx\n",
						eh_dev->name, error);
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

static irqreturn_t eh_error_irq(int irq, void *data)
{
	struct eh_device *eh_dev = data;
	unsigned long compr, decompr, error;

	compr = eh_read_register(eh_dev, EH_REG_INTRP_STS_CMP);
	decompr = eh_read_register(eh_dev, EH_REG_INTRP_STS_DCMP);
	error = eh_read_register(eh_dev, EH_REG_INTRP_STS_ERROR);

	pr_err("%s: %s: irq %d error 0x%llx compr 0x%llx decompr 0x%llx\n",
	       eh_dev->name, __func__, irq, error, compr, decompr);

	if (error) {
		pr_err("%s: error interrupt was active\n", eh_dev->name);
		eh_dump_regs(eh_dev);
		eh_write_register(eh_dev, EH_REG_INTRP_STS_ERROR, error);
	}

	return IRQ_HANDLED;
}

/* wait up to a millisecond for reset */
#define EH_RESET_WAIT_TIME 10
#define EH_MAX_RESET_WAIT 100

static int eh_reset(struct eh_device *eh_dev)
{
	unsigned long tmp = (unsigned long)-1UL;
	unsigned int count = 0;

	if (eh_dev->quirks & EH_QUIRK_IGNORE_GCTRL_RESET)
		return 0;

	eh_write_register(eh_dev, EH_REG_GCTRL, tmp);
	while (count < EH_MAX_RESET_WAIT &&
	       eh_read_register(eh_dev, EH_REG_GCTRL)) {
		usleep_range(EH_RESET_WAIT_TIME, EH_RESET_WAIT_TIME * 2);
		count++;
	}

	if (count == EH_MAX_RESET_WAIT) {
		pr_warn("%s: timeout waiting for reset\n",
			eh_dev->name);
		return 1;
	}

	return 0;
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

struct eh_device *eh_create(eh_cb_fn comp, eh_cb_fn decomp)
{
	unsigned long flags;
	struct eh_device *ret = NULL;
	struct list_head *cur;

	spin_lock_irqsave(&eh_dev_list_lock, flags);
	list_for_each (cur, &eh_dev_list) {
		struct eh_device *impl;
		impl = list_entry(cur, struct eh_device, eh_dev_list);
		pr_devel("%s: testing %s\n", __func__, impl->name);
		ret = impl;
		list_del(cur);
		pr_devel("%s: found EH device %s\n", __func__, ret->name);
		if (ret)
			break;
	}
	spin_unlock_irqrestore(&eh_dev_list_lock, flags);

	if (ret) {
		ret->comp_callback = comp;
		ret->decomp_callback = decomp;
	} else {
		pr_info("%s: unable to find desired implementation\n",
			__func__);
		ret = ERR_PTR(-ENODEV);
	}

	return ret;
}
EXPORT_SYMBOL(eh_create);

void eh_destroy(struct eh_device *eh_dev)
{
	unsigned long flags;

	eh_dev->comp_callback = eh_dev->decomp_callback = NULL;
	spin_lock_irqsave(&eh_dev_list_lock, flags);
	list_add_tail(&eh_dev->eh_dev_list, &eh_dev_list);
	spin_unlock_irqrestore(&eh_dev_list_lock, flags);
}
EXPORT_SYMBOL(eh_destroy);

static ssize_t cmd_stat_show(struct device *dev,
			     struct device_attribute *dev_attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct eh_device *eh_dev = platform_get_drvdata(pdev);
	int cpu, i;
	int written = 0;
	unsigned long x[NR_EH_EVENT_TYPE] = {
		0,
	};

	for_each_possible_cpu (cpu)
		for (i = 0; i < NR_EH_EVENT_TYPE; i++)
			x[i] += per_cpu_ptr(eh_dev->stats, cpu)->events[i];

	for (i = 0; i < NR_EH_EVENT_TYPE; i++)
		written += scnprintf(buf + written, PAGE_SIZE - written, "%lu ",
				     x[i]);

	written += scnprintf(buf + written, PAGE_SIZE - written, "\n");
	return written;
}
DEVICE_ATTR_RO(cmd_stat);

static ssize_t avg_latency_show(struct device *dev,
				struct device_attribute *dev_attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct eh_device *eh_dev = platform_get_drvdata(pdev);
	int cpu, i;
	int written = 0;
	unsigned long x[NR_EH_EVENT_TYPE] = {
		0,
	};
	int nr_cpu;

	for (i = 0; i < NR_EH_EVENT_TYPE; i++) {
		nr_cpu = 0;
		for_each_possible_cpu (cpu) {
			if (per_cpu_ptr(eh_dev->stats, cpu)->avg_lat[i] == 0)
				continue;
			x[i] += per_cpu_ptr(eh_dev->stats, cpu)->avg_lat[i];
			nr_cpu++;
		}

		x[i] = nr_cpu ? x[i] / nr_cpu : 0;
	}

	for (i = 0; i < NR_EH_EVENT_TYPE; i++)
		written += scnprintf(buf + written, PAGE_SIZE - written, "%lu ",
				     x[i]);

	written += scnprintf(buf + written, PAGE_SIZE - written, "\n");
	return written;
}
DEVICE_ATTR_RO(avg_latency);

static ssize_t max_latency_show(struct device *dev,
				struct device_attribute *dev_attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct eh_device *eh_dev = platform_get_drvdata(pdev);
	int cpu, i;
	int written = 0;
	unsigned long x[NR_EH_EVENT_TYPE] = {
		0,
	};

	for_each_online_cpu (cpu)
		for (i = 0; i < NR_EH_EVENT_TYPE; i++)
			x[i] = max(
				x[i],
				per_cpu_ptr(eh_dev->stats, cpu)->max_lat[i]);

	for (i = 0; i < NR_EH_EVENT_TYPE; i++)
		written += scnprintf(buf + written, PAGE_SIZE - written, "%lu ",
				     x[i]);

	written += scnprintf(buf + written, PAGE_SIZE - written, "\n");
	return written;
}
DEVICE_ATTR_RO(max_latency);

static ssize_t min_latency_show(struct device *dev,
				struct device_attribute *dev_attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct eh_device *eh_dev = platform_get_drvdata(pdev);
	int cpu, i;
	int written = 0;
	unsigned long x[NR_EH_EVENT_TYPE];

	/* initialize array with maximum unsigned value */
	memset(x, -1, sizeof(x));

	for_each_possible_cpu (cpu)
		for (i = 0; i < NR_EH_EVENT_TYPE; i++)
			x[i] = min(
				x[i],
				per_cpu_ptr(eh_dev->stats, cpu)->min_lat[i]);

	/* show zero value instead of maximum unsigned value for idle cpu */
	for (i = 0; i < NR_EH_EVENT_TYPE; i++)
		if (x[i] == -1UL)
			x[i] = 0;

	for (i = 0; i < NR_EH_EVENT_TYPE; i++)
		written += scnprintf(buf + written, PAGE_SIZE - written, "%lu ",
				     x[i]);

	written += scnprintf(buf + written, PAGE_SIZE - written, "\n");
	return written;
}
DEVICE_ATTR_RO(min_latency);

static ssize_t reset_latency_store(struct device *dev,
				   struct device_attribute *dev_attr,
				   const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct eh_device *eh_dev = platform_get_drvdata(pdev);
	int cpu, i;

	for_each_possible_cpu (cpu) {
		for (i = 0; i < NR_EH_EVENT_TYPE; i++) {
			per_cpu_ptr(eh_dev->stats, cpu)->min_lat[i] = -1UL;
			per_cpu_ptr(eh_dev->stats, cpu)->max_lat[i] = 0;
			per_cpu_ptr(eh_dev->stats, cpu)->avg_lat[i] = 0;
			per_cpu_ptr(eh_dev->stats, cpu)->events[i] = 0;
		}
	}
	return count;
}
DEVICE_ATTR_WO(reset_latency);

static ssize_t queued_comp_show(struct device *dev,
				struct device_attribute *dev_attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct eh_device *eh_dev = platform_get_drvdata(pdev);
	int size;
	int written = 0;

	spin_lock(&eh_dev->fifo_prod_lock);
	size = (eh_dev->write_index - eh_dev->complete_index) &
	       eh_dev->fifo_color_mask;
	spin_unlock(&eh_dev->fifo_prod_lock);

	written += scnprintf(buf + written, PAGE_SIZE - written, "%lu\n", size);
	return written;
}
DEVICE_ATTR_RO(queued_comp);

static struct attribute *eh_dev_attrs[] = {
	&dev_attr_cmd_stat.attr,
	&dev_attr_avg_latency.attr,
	&dev_attr_max_latency.attr,
	&dev_attr_min_latency.attr,
	&dev_attr_reset_latency.attr,
	&dev_attr_queued_comp.attr,
	NULL
};
ATTRIBUTE_GROUPS(eh_dev);

/* Set up constant parts of descriptors */
static void init_compression_descriptor(struct eh_device *eh_dev)
{
	int i;
	struct eh_compress_desc *desc;

	for (i = 0; i < eh_dev->fifo_size; i++ ) {
		phys_addr_t dst_paddr;
		int j;

		desc = eh_dev->fifo + EH_COMPRESS_DESC_SIZE * i;
		dst_paddr = virt_to_phys(eh_dev->compr_buffers[i]);
#ifdef CONFIG_GOOGLE_EH_CFIFO_DST_BUFFER_3KB
		desc->u1.s1.max_buf = 2;
		/* buffer 1: top 2KB of compression buffer (page) */
		desc->dst_addr[0] = EH_PHYS_ADDR_TO_ENCODED(dst_paddr, PAGE_SIZE / 2);

		/* buffer 2: next 1KB right after buffer 1 */
		desc->dst_addr[1] = EH_PHYS_ADDR_TO_ENCODED(dst_paddr + PAGE_SIZE / 2,
				PAGE_SIZE / 4);
#else
		desc->u1.s1.max_buf = 1;
		desc->dst_addr[0] = EH_PHYS_ADDR_TO_ENCODED(dst_paddr, PAGE_SIZE);
		desc->dst_addr[1] = 0;
#endif
		for (j = 2; j < EH_NUM_OF_FREE_BLOCKS; j++)
			desc->dst_addr[j] = 0;
	}
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
		pr_err("unable to allocate completions array\n");
		return -ENOMEM;
	}

	/* driver allocates fifo in regular memory - dma coherent case */
	eh_dev->fifo_alloc = kzalloc(fifo_size * (desc_size + 1),
				     GFP_KERNEL | GFP_DMA);
	if (!eh_dev->fifo_alloc) {
		pr_err("%s: unable to allocate fifo\n", eh_dev->name);
		ret = -ENOMEM;
		goto out_cleanup;
	}

	eh_dev->fifo = PTR_ALIGN(eh_dev->fifo_alloc, desc_size);
	{
		phys_addr_t pfifo = virt_to_phys(eh_dev->fifo);
		pr_info("%s: fifo is %p phys %pap\n", eh_dev->name,
			eh_dev->fifo, &pfifo);
	}

	eh_dev->compr_buffers = kzalloc(fifo_size * sizeof(void *),
					GFP_KERNEL);
	if (!eh_dev->compr_buffers) {
		pr_err("unable to allocate compr buffers array\n");
		ret = -ENOMEM;
		goto out_cleanup;
	}

	for (i = 0; i < fifo_size; i++) {
		void *buf = (void *)__get_free_pages(GFP_KERNEL, 0);
		if (!buf) {
			pr_err("unable to allocate a page for compression\n");
			ret = -ENOMEM;
			goto out_cleanup;
		}
		eh_dev->compr_buffers[i] = buf;
	}

	init_compression_descriptor(eh_dev);
	return ret;

out_cleanup:
	eh_deinit_compression(eh_dev);

	return ret;
}

static void eh_deinit_decompression(struct eh_device *eh_dev)
{
	int i;

	for (i = 0; i < eh_dev->decompr_cmd_count; i++) {
		if (eh_dev->decompr_buffers[i]) {
			free_pages((unsigned long)eh_dev->decompr_buffers[i],
				   0);
			eh_dev->decompr_buffers[i] = NULL;
		}
	}

	if (eh_dev->decompr_cmd_used) {
		kfree(eh_dev->decompr_cmd_used);
		eh_dev->decompr_cmd_used = NULL;
	}
}

static int eh_init_decompression(struct eh_device *eh_dev)
{
	int i, ret = 0;

	eh_dev->decompr_cmd_used = kzalloc(sizeof(atomic_t) *
					   eh_dev->decompr_cmd_count,
					   GFP_KERNEL);
	if (!eh_dev->decompr_cmd_used) {
		pr_err("%s unable to allocate memory for decompression\n",
		       eh_dev->name);
		return -ENOMEM;
	}

	for (i = 0; i < eh_dev->decompr_cmd_count; i++) {
		atomic_set(eh_dev->decompr_cmd_used + i, 0);
		spin_lock_init(&eh_dev->decompr_lock[i]);
	}

	for (i = 0; i < eh_dev->decompr_cmd_count; i++) {
		void *buf = (void *)__get_free_pages(GFP_KERNEL, 0);
		if (!buf) {
			pr_err("unable to allocate a page for decompression\n");
			ret = -ENOMEM;
			goto out_cleanup;
		}
		eh_dev->decompr_buffers[i] = buf;
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

	if (eh_dev->stats) {
		free_percpu(eh_dev->stats);
		eh_dev->stats = NULL;
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
	eh_dev->max_buffer_count = EH_FEATURES2_BUF_MAX(feature);
	eh_dev->decompr_cmd_count = EH_FEATURES2_DECOMPR_CMDS(feature);

	if (eh_dev->max_buffer_count == 0 || eh_dev->decompr_cmd_count == 0) {
		pr_err("%s: max_buffer_count %d decompr_cmd_count %d\n",
			eh_dev->max_buffer_count, eh_dev->decompr_cmd_count);
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

	pr_info("%s: max_bufs %u decompress_cmd_sets %u\n", eh_dev->name,
		 eh_dev->max_buffer_count, eh_dev->decompr_cmd_count);

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

	return ret;
}

/* Initialize SW related stuff */
static int eh_sw_init(struct eh_device *eh_dev, int error_irq)
{
	int ret, cpu;

	/* the error interrupt */
	ret = request_threaded_irq(error_irq, NULL, eh_error_irq, IRQF_ONESHOT,
				   EH_ERR_IRQ, eh_dev);
	if (ret) {
		pr_err("%s: unable to request irq %u ret %d\n", eh_dev->name,
		       error_irq, ret);
		return ret;
	}
	eh_dev->error_irq = error_irq;

	eh_dev->comp_thread = kthread_run(eh_comp_thread, eh_dev, "eh_comp_thread");
	if (IS_ERR(eh_dev->comp_thread)) {
		ret = PTR_ERR(eh_dev->comp_thread);
		goto free_irq;
	}

	eh_dev->stats = alloc_percpu(struct eh_stats);
	if (!eh_dev->stats) {
		ret = -ENOMEM;
		goto free_thread;
	}

	for_each_possible_cpu (cpu) {
		int i;

		for (i = 0; i < NR_EH_EVENT_TYPE; i++)
			per_cpu_ptr(eh_dev->stats, cpu)->min_lat[i] = -1UL;
	}

	spin_lock(&eh_dev_list_lock);
	list_add_tail(&eh_dev->eh_dev_list, &eh_dev_list);
	spin_unlock(&eh_dev_list_lock);

	atomic_set(&eh_dev->nr_request, 0);
	init_waitqueue_head(&eh_dev->comp_wq);

	return 0;

free_thread:
	kthread_stop(eh_dev->comp_thread);
	eh_dev->comp_thread = NULL;
free_irq:
	free_irq(eh_dev->error_irq, eh_dev);
	eh_dev->error_irq = 0;

	return ret;
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

	snprintf(eh_dev->name, EH_MAX_NAME, "eh%u", 0);
	pr_devel("%s: probing EH device\n", eh_dev->name);

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

static void eh_deinit(struct eh_device *eh_dev)
{
	eh_hw_deinit(eh_dev);
	eh_sw_deinit(eh_dev);
}

void eh_remove(struct eh_device *eh_dev)
{
	eh_deinit(eh_dev);
	kfree(eh_dev);
}

static void eh_setup_descriptor(struct eh_device *eh_dev, struct page *src_page,
			    unsigned int masked_w_index)
{
	struct eh_compress_desc *desc;
	phys_addr_t src_paddr;

	desc = eh_dev->fifo + EH_COMPRESS_DESC_SIZE * masked_w_index;
	src_paddr = page_to_phys(src_page);

	pr_devel("%s: %s: desc = %px src = %pa[p] dst = %pa[p]\n", eh_dev->name,
				__func__, desc, &src_paddr,
				EH_ENCODED_ADDR_TO_PHYS(desc->dst_addr[0]));

	desc->u1.src_addr = src_paddr;
	/* mark it as pend for hardware */
	desc->u1.s1.status = EH_CDESC_PENDING;
	/*
	 * Skip setting other fields of the descriptor for the performance
	 * reason. It's doable since they are never changed once they are
	 * initialized. Look at init_compression_descriptor.
	 */
}

static int eh_process_completed_descriptor(struct eh_device *eh_dev,
					   unsigned short fifo_index,
					   struct eh_completion *cmpl)
{
	struct eh_compress_desc *desc;
	unsigned int compr_status;
	unsigned int compr_size;
	unsigned int compr_bufsel;
	unsigned int offset;
	void *compr_data = NULL;
	int ret = 0;

	eh_update_latency(eh_dev, get_submit_ts(cmpl), 1, EH_COMPRESS);

	desc = eh_dev->fifo + (fifo_index * EH_COMPRESS_DESC_SIZE);

	pr_devel("%s: desc 0x%x status 0x%x len %u src 0x%pap\n", eh_dev->name,
		 fifo_index, desc->u1.s1.status, desc->compr_len,
		 &desc->u1.src_addr);

	compr_status = desc->u1.s1.status;
	compr_size = desc->compr_len;
	compr_bufsel = desc->buf_sel;
	offset = (compr_bufsel == 2) ? PAGE_SIZE / 2 : 0;

	switch (compr_status) {
	/* normal case, page copied */
	case EH_CDESC_COPIED:
		compr_data = eh_dev->compr_buffers[fifo_index] + offset;
		pr_devel("%s: COPIED desc 0x%x buf %px\n", eh_dev->name,
			 fifo_index, compr_data);
		break;

	/* normal case, compression completed successfully */
	case EH_CDESC_COMPRESSED:
		compr_data = eh_dev->compr_buffers[fifo_index] + offset;
		pr_devel("%s: COMPRESSED desc 0x%x buf %px\n", eh_dev->name,
			 fifo_index, compr_data);
		break;

	/* normal case, hardware detected page of all zeros */
	case EH_CDESC_ZERO:
		pr_devel("%s: ZERO desc 0x%x\n", eh_dev->name, fifo_index);
		break;

	/* normal case, incompressible page, did not fit into 3K buffer */
	case EH_CDESC_ABORT:
		pr_devel("%s: ABORT desc 0x%x\n", eh_dev->name, fifo_index);
		break;

	/* an error occurred, but hardware is still progressing */
	case EH_CDESC_ERROR_CONTINUE:
		pr_err("%s: got error on descriptor 0x%x\n", eh_dev->name,
		       fifo_index);
		break;

	/* a fairly bad error occurred, need to reset the fifo */
	case EH_CDESC_ERROR_HALTED:
		pr_err("%s: got fifo error on descriptor 0x%x\n", eh_dev->name,
		       fifo_index);
		ret = 1;
		break;

	/*
	 * this shouldn't normally happen -- hardware indicated completed but
	 * descriptor is still in PEND or IDLE.
	 */
	case EH_CDESC_IDLE:
	case EH_CDESC_PENDING:
		eh_dump_regs(eh_dev);
		pr_err("%s: descriptor 0x%x pend or idle 0x%x: ", eh_dev->name,
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
		BUG_ON(1);
		break;
	};

	/* do the callback */
	(*eh_dev->comp_callback)(compr_status, compr_data, compr_size, cmpl->priv);

	/* set the descriptor back to IDLE */
	desc->u1.s1.status = EH_CDESC_IDLE;
	atomic_dec(&eh_dev->nr_request);
	clear_eh_congested();

	return ret;
}

static int eh_process_completions(struct eh_device *eh_dev, unsigned int start,
				   unsigned int end)
{
	int ret = 0;
	unsigned int i;
	unsigned int index;
	struct eh_completion *cmpl;

	pr_devel("%s: %s: process from %u to %u\n", eh_dev->name, __func__,
		 start, end);

	for (i = start; i != end; i = (i + 1) & eh_dev->fifo_color_mask) {
		index = i & eh_dev->fifo_index_mask;
		cmpl = &eh_dev->completions[index];
		ret = eh_process_completed_descriptor(eh_dev, index, cmpl);
		cmpl->priv = NULL;
		smp_store_release(&eh_dev->complete_index,
				  (eh_dev->complete_index + 1) &
					  eh_dev->fifo_color_mask);
		if (ret)
			break;
	}

	return ret;
}

static int eh_update_complete_index(struct eh_device *eh_dev,
				     bool update_int_idx)
{
	int ret = 0;
	unsigned long raw = eh_read_register(eh_dev, EH_REG_CDESC_CTRL);
	unsigned int new_complete_index = raw & EH_CDESC_CTRL_COMPLETE_IDX_MASK;

	if (new_complete_index != eh_dev->complete_index)
		ret = eh_process_completions(eh_dev, eh_dev->complete_index,
				       new_complete_index);
	return ret;
}

/*
 * eh_compress_pages
 *
 * Compress n pages asynchronously. Uses compression IRQ for completion.
 *
 * Each FIFO entry is marked with "IRQ enable", so the interrupts will
 * start arriving as soon as the first page has been compressed.
 */
int eh_compress_pages(struct eh_device *eh_dev, struct page **pages,
		      unsigned int page_cnt, void *priv)
{
	int i;
	unsigned int complete_index;
	unsigned int new_write_index;
	unsigned int new_pending_count;
	unsigned int masked_w_index;
	struct eh_completion *cmpl;

try_again:
	spin_lock(&eh_dev->fifo_prod_lock);

	if (eh_dev->suspended) {
		WARN(1, "compress request when EH is suspended\n");
		spin_unlock(&eh_dev->fifo_prod_lock);
		return -EBUSY;
	}

	complete_index = READ_ONCE(eh_dev->complete_index);
	new_write_index =
		(eh_dev->write_index + page_cnt) & eh_dev->fifo_color_mask;
	new_pending_count =
		(new_write_index - complete_index) & eh_dev->fifo_color_mask;

	if (new_pending_count > eh_dev->fifo_size) {
		spin_unlock(&eh_dev->fifo_prod_lock);
		cond_resched();
		eh_congestion_wait(HZ/10);
		goto try_again;
	}

	pr_devel("[%s] %s: submit %u pages starting at descriptor %u\n",
		 current->comm, __func__, page_cnt, eh_dev->write_index);

	for (i = 0; i < page_cnt; i++) {
		masked_w_index =
			(eh_dev->write_index + i) & eh_dev->fifo_index_mask;
		/* set up the descriptor (use IRQ) */
		eh_setup_descriptor(eh_dev, pages[i], masked_w_index);

		cmpl = &eh_dev->completions[masked_w_index];
		cmpl->priv = priv;
		set_submit_ts(cmpl, ktime_get_ns());
	}

	atomic_inc(&eh_dev->nr_request);
	wake_up(&eh_dev->comp_wq);

	/* write barrier to force writes to be visible everywhere */
	wmb();
	eh_dev->write_index = new_write_index;
	eh_write_register(eh_dev, EH_REG_CDESC_WRIDX, new_write_index);
	spin_unlock(&eh_dev->fifo_prod_lock);

	return 0;
}
EXPORT_SYMBOL(eh_compress_pages);

static void eh_setup_dcmd(struct eh_device *eh_dev, unsigned int index,
			void *compr_data, unsigned int compr_size,
			struct page *dst_page, unsigned long *ts)
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
	alignment = 1UL << __ffs((unsigned long)compr_data);
	if (alignment < 64 || compr_size > alignment) {
		pr_devel("COPY: compr_data %px, compr_size %u, alignment %u\n",
			 compr_data, compr_size, alignment);
		src_vaddr = eh_dev->decompr_buffers[index];
		memcpy(src_vaddr, compr_data, compr_size);
		src_paddr = virt_to_phys(src_vaddr);
		alignment = PAGE_SIZE;
	} else {
		pr_devel(
			"NO COPY: compr_data %px, compr_size %u, alignment %u\n",
			compr_data, compr_size, alignment);
		src_paddr = virt_to_phys(compr_data);
		if (alignment > PAGE_SIZE)
			alignment = PAGE_SIZE;
	}

	csize_data = compr_size << EH_DCMD_CSIZE_SIZE_SHIFT;
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
#ifdef CONFIG_GOOGLE_EH_LATENCY_STAT
	*ts = ktime_get_ns();
#endif
	eh_write_register(eh_dev, EH_REG_DCMD_DEST(index), dst_data);
}

/*
 * eh_decompress_page
 *
 * Decompress a page synchronously. Uses polling for completion.
 *
 * Holds a spinlock for the entire operation, so that nothing can interrupt it.
 */
int eh_decompress_page(struct eh_device *eh_dev, void *compr_data,
			    unsigned int compr_size, struct page *page)
{
	int ret = 0;
	unsigned long flags;
	unsigned int index;
	unsigned long submit_ts;
	unsigned long timeout;
	unsigned long status;

	/* make a static mapping of cpu to decompression command set */
	index = smp_processor_id() % eh_dev->decompr_cmd_count;

	spin_lock_irqsave(&eh_dev->decompr_lock[index], flags);

	if (eh_dev->suspended) {
		WARN(1, "decompress request when EH is suspended\n");
		ret = -EBUSY;
		goto out;
	}

	if (eh_dev->decompr_busy[index]) {
		/* this should never happen in polling mode */
		ret = -EBUSY;
		goto out;
	}

	pr_devel("[%s] %s: submit: cpu %u dcmd_set %u compr_size %u\n",
		 current->comm, __func__, smp_processor_id(), index,
		 compr_size);

	/* program decompress register (no IRQ) */
	eh_setup_dcmd(eh_dev, index, compr_data, compr_size, page, &submit_ts);

	timeout = jiffies + msecs_to_jiffies(EH_POLL_DELAY_MS);
	do {
		cpu_relax();
		if (time_after(jiffies, timeout)) {
			pr_err("%s: poll timeout on decompression\n", __func__);
			eh_dump_regs(eh_dev);
			ret = -ETIME;
			goto out;
		}
		status = eh_read_dcmd_status(eh_dev, index);
	} while (status == EH_DCMD_PENDING);

	eh_update_latency(eh_dev, submit_ts, 1, EH_DECOMPRESS_POLL);

	pr_devel("%s: dcmd [%u] status = %u\n", eh_dev->name, index, status);

	if (status != EH_DCMD_DECOMPRESSED) {
		pr_err("%s: dcmd [%u] bad status %u\n", eh_dev->name, index,
		       status);
		eh_dump_regs(eh_dev);
		ret = -EIO;
	}

out:
	spin_unlock_irqrestore(&eh_dev->decompr_lock[index], flags);
	return ret;
}
EXPORT_SYMBOL(eh_decompress_page);

static unsigned int eh_default_fifo_size = 256;

#ifdef CONFIG_OF
static int eh_of_probe(struct platform_device *pdev)
{
	struct eh_device *eh_dev;
	struct resource *mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	int ret;
	int error_irq = 0;
	unsigned short quirks = 0;
	struct clk *clk;

	pr_devel("%s starting\n", __func__);

	pm_runtime_enable(&pdev->dev);
	ret = pm_runtime_get_sync(&pdev->dev);
	if (ret) {
		dev_err(&pdev->dev, "pm_runtime_get_sync returned %d\n", ret);
		goto disable_pm_runtime;
	}

	error_irq = irq_of_parse_and_map(pdev->dev.of_node, 0);
	if (error_irq == 0) {
		pr_err("Fail to get error irq\n");
		ret = -EINVAL;
		goto put_pm_runtime;
	}

	clk = of_clk_get_by_name(pdev->dev.of_node, "eh-clock");
	if (IS_ERR(clk)) {
		pr_err("%s: of_clk_get_by_name() failed\n", __func__);
		ret = PTR_ERR(clk);
		goto put_pm_runtime;
	}

	ret = clk_prepare_enable(clk);
	if (ret) {
		pr_err("%s: clk_prepare_enable() failed\n", __func__);
		goto put_clk;
	}

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

	ret = device_add_groups(&pdev->dev, eh_dev_groups);
	if (ret) {
		dev_err(&pdev->dev, "failed to create sysfs entries\n");
		goto eh_deinit;
	}

	return 0;

eh_deinit:
	eh_deinit(eh_dev);
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

	return ret;
}

static int eh_of_remove(struct platform_device *pdev)
{
	struct eh_device *eh_dev = platform_get_drvdata(pdev);

	eh_remove(eh_dev);

	clk_disable_unprepare(eh_dev->clk);
	clk_put(eh_dev->clk);
	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	return 0;
}

static int eh_suspend(struct device *dev)
{
	int i;
	int ret = 0;
	unsigned long data;
	struct eh_device *eh_dev = dev_get_drvdata(dev);

	/* grab all locks */
	spin_lock(&eh_dev->fifo_prod_lock);
	for (i = 0; i < eh_dev->decompr_cmd_count; i++)
		spin_lock(&eh_dev->decompr_lock[i]);

	/* check pending work */
	if (atomic_read(&eh_dev->nr_request) > 0) {
		pr_info("%s: block suspend (compression pending)\n", __func__);
		ret = -EBUSY;
		goto out;
	}

	for (i = 0; i < eh_dev->decompr_cmd_count; i++) {
		if (eh_dev->decompr_busy[i]) {
			pr_info("%s: block suspend (decompression pending)\n", __func__);
			ret = -EBUSY;
			goto out;
		}
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

	eh_dev->suspended = true;
	pr_info("%s: EH suspended\n", __func__);

out:
	for (i = eh_dev->decompr_cmd_count - 1; i >= 0; i--)
		spin_unlock(&eh_dev->decompr_lock[i]);
	spin_unlock(&eh_dev->fifo_prod_lock);

	return ret;
}

static int eh_resume(struct device *dev)
{
	struct eh_device *eh_dev = dev_get_drvdata(dev);

	spin_lock(&eh_dev->fifo_prod_lock);

	/* re-enable EH clock */
	clk_prepare_enable(eh_dev->clk);

	/* re-enable compression FIFO */
	eh_compr_fifo_init(eh_dev);

	/* re-enable all interrupts */
	eh_write_register(eh_dev, EH_REG_INTRP_MASK_ERROR, 0);
	eh_write_register(eh_dev, EH_REG_INTRP_MASK_CMP, 0);
	eh_write_register(eh_dev, EH_REG_INTRP_MASK_DCMP, 0);

	eh_dev->suspended = false;
	pr_info("%s: EH resumed\n", __func__);

	spin_unlock(&eh_dev->fifo_prod_lock);
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
