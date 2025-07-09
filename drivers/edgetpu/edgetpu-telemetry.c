// SPDX-License-Identifier: GPL-2.0
/*
 * Edge TPU ML accelerator telemetry: logging and tracing.
 *
 * Copyright (C) 2019-2020 Google, Inc.
 */

#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>

#include "edgetpu-internal.h"
#include "edgetpu-iremap-pool.h"
#include "edgetpu-mmu.h"
#include "edgetpu-telemetry.h"
#include "edgetpu.h"

/* When log data arrives, recheck for more log data after this delay. */
#define TELEMETRY_LOG_RECHECK_DELAY	200	/* ms */

static struct edgetpu_telemetry *
select_telemetry(struct edgetpu_telemetry_ctx *ctx,
		 enum edgetpu_telemetry_type type)
{
	switch (type) {
	case EDGETPU_TELEMETRY_TRACE:
		return &ctx->trace;
	case EDGETPU_TELEMETRY_LOG:
		return &ctx->log;
	default:
		WARN_ONCE(1, "Unrecognized EdgeTPU telemetry type: %d", type);
		/* return a valid object, don't crash the kernel */
		return &ctx->log;
	}
}

static int telemetry_kci(struct edgetpu_dev *etdev,
			 struct edgetpu_telemetry *tel,
			 int (*send_kci)(struct edgetpu_kci *, u64, u32))
{
	int err;

	if (!tel->inited)
		return -ENODEV;
	etdev_dbg(etdev, "Sending KCI %s", tel->name);
	err = send_kci(etdev->kci, tel->coherent_mem.tpu_addr,
		       tel->coherent_mem.size);

	if (err < 0) {
		etdev_err(etdev, "KCI %s failed :( - %d", tel->name, err);
		return err;
	}

	if (err > 0) {
		etdev_err(etdev, "KCI %s returned %d", tel->name, err);
		return -EBADMSG;
	}
	etdev_dbg(etdev, "KCI %s Succeeded :)", tel->name);
	return 0;
}

static int telemetry_set_event(struct edgetpu_dev *etdev,
			       struct edgetpu_telemetry *tel, u32 eventfd)
{
	struct eventfd_ctx *ctx;
	ulong flags;

	if (!tel->inited)
		return -ENODEV;
	ctx = eventfd_ctx_fdget(eventfd);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	write_lock_irqsave(&tel->ctx_lock, flags);
	if (tel->ctx)
		eventfd_ctx_put(tel->ctx);
	tel->ctx = ctx;
	write_unlock_irqrestore(&tel->ctx_lock, flags);

	return 0;
}

static void telemetry_unset_event(struct edgetpu_dev *etdev,
				  struct edgetpu_telemetry *tel)
{
	ulong flags;

	if (!tel->inited)
		return;
	write_lock_irqsave(&tel->ctx_lock, flags);
	if (tel->ctx)
		eventfd_ctx_put(tel->ctx);
	tel->ctx = NULL;
	write_unlock_irqrestore(&tel->ctx_lock, flags);

	return;
}

/* Copy data out of the log buffer with wrapping */
static void copy_with_wrap(struct edgetpu_telemetry_header *header, void *dest,
			   u32 length, u32 size, void *start)
{
	const u32 wrap_bit = size + sizeof(*header);
	u32 remaining = 0;
	u32 head = header->head & (wrap_bit - 1);

	if (head + length < size) {
		memcpy(dest, start + head, length);
		header->head += length;
	} else {
		remaining = size - head;
		memcpy(dest, start + head, remaining);
		memcpy(dest + remaining, start, length - remaining);
		header->head = (header->head & wrap_bit) ^ wrap_bit;
		header->head |= length - remaining;
	}
}

/* Log messages from TPU CPU to dmesg */
static void edgetpu_fw_log(struct edgetpu_telemetry *log)
{
	struct edgetpu_dev *etdev = log->etdev;
	struct edgetpu_telemetry_header *header = log->header;
	struct edgetpu_log_entry_header entry;
	u8 *start;
	const size_t queue_size = log->coherent_mem.size - sizeof(*header);
	const size_t max_length = queue_size - sizeof(entry);
	char *buffer = kmalloc(max_length + 1, GFP_ATOMIC);

	if (!buffer) {
		header->head = header->tail;
		etdev_err_ratelimited(etdev, "failed to allocate log buffer");
		return;
	}
	start = (u8 *)header + sizeof(*header);

	while (header->head != header->tail) {
		copy_with_wrap(header, &entry, sizeof(entry), queue_size,
			       start);
		if (entry.length == 0 || entry.length > max_length) {
			header->head = header->tail;
			etdev_err_ratelimited(etdev, "log queue is corrupted");
			break;
		}
		copy_with_wrap(header, buffer, entry.length, queue_size, start);
		buffer[entry.length] = 0;

		if (entry.code > EDGETPU_FW_DMESG_LOG_LEVEL)
			continue;

		switch (entry.code) {
		case EDGETPU_FW_LOG_LEVEL_VERBOSE:
		case EDGETPU_FW_LOG_LEVEL_DEBUG:
			etdev_dbg_ratelimited(etdev, "%s", buffer);
			break;
		case EDGETPU_FW_LOG_LEVEL_WARN:
			etdev_warn_ratelimited(etdev, "%s", buffer);
			break;
		case EDGETPU_FW_LOG_LEVEL_ERROR:
			etdev_err_ratelimited(etdev, "%s", buffer);
			break;
		case EDGETPU_FW_LOG_LEVEL_INFO:
		default:
			etdev_info_ratelimited(etdev, "%s", buffer);
			break;
		}
	}
	kfree(buffer);
}

/* Consumes the queue buffer. */
static void edgetpu_fw_trace(struct edgetpu_telemetry *trace)
{
	struct edgetpu_telemetry_header *header = trace->header;

	header->head = header->tail;
}

/* Worker for processing log/trace buffers. */

static void telemetry_worker(struct work_struct *work)
{
	struct edgetpu_telemetry *tel =
		container_of(work, struct edgetpu_telemetry, work);
	u32 prev_head;
	ulong flags;

	/*
	 * Loop while telemetry enabled, there is data to be consumed,
	 * and the previous iteration made progress.  If another IRQ arrives
	 * just after the last head != tail check we should get another worker
	 * schedule.
	 */
	do {
		spin_lock_irqsave(&tel->state_lock, flags);
		if (tel->state != EDGETPU_TELEMETRY_ENABLED) {
			spin_unlock_irqrestore(&tel->state_lock, flags);
			return;
		}

		prev_head = tel->header->head;
		if (tel->header->head != tel->header->tail) {
			read_lock(&tel->ctx_lock);
			if (tel->ctx)
				eventfd_signal(tel->ctx, 1);
			else
				tel->fallback_fn(tel);
			read_unlock(&tel->ctx_lock);
		}

		spin_unlock_irqrestore(&tel->state_lock, flags);
		msleep(TELEMETRY_LOG_RECHECK_DELAY);
	} while (tel->header->head != tel->header->tail &&
		 tel->header->head != prev_head);
}


/* If the buffer queue is not empty, schedules worker. */
static void telemetry_irq_handler(struct edgetpu_dev *etdev,
				  struct edgetpu_telemetry *tel)
{
	if (!tel->inited)
		return;
	spin_lock(&tel->state_lock);

	if (tel->state == EDGETPU_TELEMETRY_ENABLED &&
	    tel->header->head != tel->header->tail) {
		schedule_work(&tel->work);
	}

	spin_unlock(&tel->state_lock);
}

static void telemetry_mappings_show(struct edgetpu_telemetry *tel,
				    struct seq_file *s)
{
	if (!tel->inited)
		return;

	seq_printf(s, "  %pad %lu %s %#llx %pad\n",
		   &tel->coherent_mem.tpu_addr,
		   DIV_ROUND_UP(tel->coherent_mem.size, PAGE_SIZE), tel->name,
		   tel->coherent_mem.host_addr, &tel->coherent_mem.dma_addr);
}

static void telemetry_inc_mmap_count(struct edgetpu_telemetry *tel, int dif)
{
	if (!tel->inited)
		return;

	mutex_lock(&tel->mmap_lock);
	tel->mmapped_count += dif;
	mutex_unlock(&tel->mmap_lock);
}

static int telemetry_mmap_buffer(struct edgetpu_dev *etdev,
				 struct edgetpu_telemetry *tel,
				 struct vm_area_struct *vma)
{
	int ret;

	if (!tel->inited)
		return -ENODEV;

	mutex_lock(&tel->mmap_lock);

	if (!tel->mmapped_count) {
		ret = edgetpu_iremap_mmap(etdev, vma, &tel->coherent_mem);

		if (!ret) {
			tel->coherent_mem.host_addr = vma->vm_start;
			tel->mmapped_count = 1;
		}
	} else {
		ret = -EBUSY;
		etdev_warn(etdev, "%s is already mmapped %ld times", tel->name,
			   tel->mmapped_count);
	}

	mutex_unlock(&tel->mmap_lock);

	return ret;
}

static int telemetry_init(struct edgetpu_dev *etdev, struct edgetpu_telemetry *tel,
			  const char *name, struct edgetpu_coherent_mem *mem, const size_t size,
			  void (*fallback)(struct edgetpu_telemetry *))
{
	const u32 flags = EDGETPU_MMU_DIE | EDGETPU_MMU_32 | EDGETPU_MMU_HOST;
	void *vaddr;
	dma_addr_t dma_addr;
	tpu_addr_t tpu_addr;

	if (mem) {
		tel->coherent_mem = *mem;
		vaddr = mem->vaddr;
		tel->caller_mem = true;
	} else {
		vaddr = dmam_alloc_coherent(etdev->dev, size, &dma_addr,
					    GFP_KERNEL);
		if (!vaddr)
			return -ENOMEM;
		tpu_addr = edgetpu_mmu_tpu_map(etdev, dma_addr, size,
					       DMA_BIDIRECTIONAL,
					       EDGETPU_CONTEXT_KCI, flags);
		if (!tpu_addr) {
			dev_err(etdev->dev,
				"%s: failed to map buffer for '%s'\n",
				etdev->dev_name, name);
			return -ENOSPC;
		}
		tel->coherent_mem.vaddr = vaddr;
		tel->coherent_mem.dma_addr = dma_addr;
		tel->coherent_mem.tpu_addr = tpu_addr;
		tel->coherent_mem.size = size;
		tel->caller_mem = false;
		edgetpu_x86_coherent_mem_set_uc(&tel->coherent_mem);
	}

	rwlock_init(&tel->ctx_lock);
	tel->name = name;
	tel->etdev = etdev;

	tel->header = (struct edgetpu_telemetry_header *)vaddr;
	tel->header->head = 0;
	tel->header->size = 0;
	tel->header->tail = 0;
	tel->header->entries_dropped = 0;

	tel->ctx = NULL;

	spin_lock_init(&tel->state_lock);
	INIT_WORK(&tel->work, telemetry_worker);
	tel->fallback_fn = fallback;
	tel->state = EDGETPU_TELEMETRY_ENABLED;
	tel->inited = true;
	mutex_init(&tel->mmap_lock);
	tel->mmapped_count = 0;

	return 0;
}

static void telemetry_exit(struct edgetpu_dev *etdev,
			   struct edgetpu_telemetry *tel)
{
	ulong flags;

	if (!tel->inited)
		return;
	spin_lock_irqsave(&tel->state_lock, flags);
	/* Prevent racing with the IRQ handler or worker */
	tel->state = EDGETPU_TELEMETRY_INVALID;
	spin_unlock_irqrestore(&tel->state_lock, flags);
	cancel_work_sync(&tel->work);

	if (tel->coherent_mem.tpu_addr && !tel->caller_mem) {
		edgetpu_mmu_tpu_unmap(etdev, tel->coherent_mem.tpu_addr,
				      tel->coherent_mem.size,
				      EDGETPU_CONTEXT_KCI);
		tel->coherent_mem.tpu_addr = 0;
		edgetpu_x86_coherent_mem_set_wb(&tel->coherent_mem);
	}
	if (tel->ctx)
		eventfd_ctx_put(tel->ctx);
	tel->ctx = NULL;
}

int edgetpu_telemetry_init(struct edgetpu_dev *etdev,
			   struct edgetpu_coherent_mem *log_mem,
			   struct edgetpu_coherent_mem *trace_mem)
{
	int ret, i;

	if (!etdev->telemetry)
		return -ENODEV;

	for (i = 0; i < etdev->num_cores; i++) {
		ret = telemetry_init(etdev, &etdev->telemetry[i].log, "telemetry_log",
				     log_mem ? &log_mem[i] : NULL,
				     EDGETPU_TELEMETRY_LOG_BUFFER_SIZE, edgetpu_fw_log);
		if (ret)
			break;
#if IS_ENABLED(CONFIG_EDGETPU_TELEMETRY_TRACE)
		ret = telemetry_init(etdev, &etdev->telemetry[i].trace, "telemetry_trace",
				     trace_mem ? &trace_mem[i] : NULL,
				     EDGETPU_TELEMETRY_TRACE_BUFFER_SIZE, edgetpu_fw_trace);
		if (ret)
			break;
#endif
	}

	if (ret)
		edgetpu_telemetry_exit(etdev);

	return ret;
}

void edgetpu_telemetry_exit(struct edgetpu_dev *etdev)
{
	int i;

	if (!etdev->telemetry)
		return;

	for (i = 0; i < etdev->num_cores; i++) {
#if IS_ENABLED(CONFIG_EDGETPU_TELEMETRY_TRACE)
		telemetry_exit(etdev, &etdev->telemetry[i].trace);
#endif
		telemetry_exit(etdev, &etdev->telemetry[i].log);
	}
}

int edgetpu_telemetry_kci(struct edgetpu_dev *etdev)
{
	int ret;

	if (!etdev->telemetry)
		return -ENODEV;

	/* Core 0 will notify other cores. */
	ret = telemetry_kci(etdev, &etdev->telemetry[0].log, edgetpu_kci_map_log_buffer);
	if (ret)
		return ret;

#if IS_ENABLED(CONFIG_EDGETPU_TELEMETRY_TRACE)
	ret = telemetry_kci(etdev, &etdev->telemetry[0].trace, edgetpu_kci_map_trace_buffer);
	if (ret)
		return ret;
#endif

	return 0;
}

int edgetpu_telemetry_set_event(struct edgetpu_dev *etdev,
				enum edgetpu_telemetry_type type, u32 eventfd)
{
	int i, ret;

	if (!etdev->telemetry)
		return -ENODEV;

	for (i = 0; i < etdev->num_cores; i++) {
		ret = telemetry_set_event(etdev, select_telemetry(&etdev->telemetry[i], type),
					  eventfd);
		if (ret) {
			edgetpu_telemetry_unset_event(etdev, type);
			return ret;
		}
	}

	return 0;
}

void edgetpu_telemetry_unset_event(struct edgetpu_dev *etdev,
				   enum edgetpu_telemetry_type type)
{
	int i;

	if (!etdev->telemetry)
		return;

	for (i = 0; i < etdev->num_cores; i++)
		telemetry_unset_event(etdev, select_telemetry(&etdev->telemetry[i], type));
}

void edgetpu_telemetry_irq_handler(struct edgetpu_dev *etdev)
{
	int i;

	if (!etdev->telemetry)
		return;

	for (i = 0; i < etdev->num_cores; i++) {
		telemetry_irq_handler(etdev, &etdev->telemetry[i].log);
#if IS_ENABLED(CONFIG_EDGETPU_TELEMETRY_TRACE)
		telemetry_irq_handler(etdev, &etdev->telemetry[i].trace);
#endif
	}
}

void edgetpu_telemetry_mappings_show(struct edgetpu_dev *etdev,
				     struct seq_file *s)
{
	int i;

	if (!etdev->telemetry)
		return;

	for (i = 0; i < etdev->num_cores; i++) {
		telemetry_mappings_show(&etdev->telemetry[i].log, s);
#if IS_ENABLED(CONFIG_EDGETPU_TELEMETRY_TRACE)
		telemetry_mappings_show(&etdev->telemetry[i].trace, s);
#endif
	}
}

int edgetpu_mmap_telemetry_buffer(struct edgetpu_dev *etdev, enum edgetpu_telemetry_type type,
				  struct vm_area_struct *vma, int core_id)
{
	if (!etdev->telemetry)
		return -ENODEV;
	return telemetry_mmap_buffer(etdev, select_telemetry(&etdev->telemetry[core_id], type),
				     vma);
}

void edgetpu_telemetry_inc_mmap_count(struct edgetpu_dev *etdev, enum edgetpu_telemetry_type type,
				      int core_id)
{
	if (!etdev->telemetry)
		return;
	telemetry_inc_mmap_count(select_telemetry(&etdev->telemetry[core_id], type), 1);
}

void edgetpu_telemetry_dec_mmap_count(struct edgetpu_dev *etdev, enum edgetpu_telemetry_type type,
				      int core_id)
{
	if (!etdev->telemetry)
		return;
	telemetry_inc_mmap_count(select_telemetry(&etdev->telemetry[core_id], type), -1);
}
