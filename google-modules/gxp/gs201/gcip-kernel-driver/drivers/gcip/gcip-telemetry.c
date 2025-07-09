// SPDX-License-Identifier: GPL-2.0-only
/*
 * GCIP telemetry: logging and tracing.
 *
 * Copyright (C) 2022 Google LLC
 */

#include <linux/delay.h>
#include <linux/dev_printk.h>
#include <linux/eventfd.h>
#include <linux/log2.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>

#include <gcip/gcip-telemetry.h>

int gcip_telemetry_kci(struct gcip_telemetry *tel,
		       int (*send_kci)(struct gcip_telemetry_kci_args *),
		       struct gcip_telemetry_kci_args *args)
{
	int err;

	dev_dbg(tel->dev, "Sending KCI %s", tel->name);
	err = send_kci(args);

	if (err < 0) {
		dev_err(tel->dev, "KCI %s failed - %d", tel->name, err);
		return err;
	}

	if (err > 0) {
		dev_err(tel->dev, "KCI %s returned %d", tel->name, err);
		return -EBADMSG;
	}

	dev_dbg(tel->dev, "KCI %s Succeeded", tel->name);

	return 0;
}

int gcip_telemetry_set_event(struct gcip_telemetry *tel, u32 eventfd)
{
	struct eventfd_ctx *ctx;
	ulong flags;

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

void gcip_telemetry_unset_event(struct gcip_telemetry *tel)
{
	ulong flags;

	write_lock_irqsave(&tel->ctx_lock, flags);
	if (tel->ctx)
		eventfd_ctx_put(tel->ctx);
	tel->ctx = NULL;
	write_unlock_irqrestore(&tel->ctx_lock, flags);
}

/* Copy data out of the log buffer with wrapping. */
static void copy_with_wrap(struct gcip_telemetry_header *header, void *dest, u32 length, u32 size,
			   void *start)
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

void gcip_telemetry_fw_log(struct gcip_telemetry *log)
{
	struct device *dev = log->dev;
	struct gcip_telemetry_header *header = log->header;
	struct gcip_log_entry_header entry;
	u8 *start;
	const size_t queue_size = header->size - sizeof(*header);
	const size_t max_length = queue_size - sizeof(entry);
	char *buffer = kmalloc(max_length + 1, GFP_ATOMIC);

	if (!buffer) {
		header->head = header->tail;
		return;
	}
	start = (u8 *)header + sizeof(*header);

	while (header->head != header->tail) {
		copy_with_wrap(header, &entry, sizeof(entry), queue_size, start);
		if (entry.length == 0 || entry.length > max_length) {
			header->head = header->tail;
			dev_err(dev, "log queue is corrupted");
			break;
		}
		copy_with_wrap(header, buffer, entry.length, queue_size, start);
		buffer[entry.length] = 0;

		if (entry.code > GCIP_FW_DMESG_LOG_LEVEL)
			continue;

		switch (entry.code) {
		case GCIP_FW_LOG_LEVEL_VERBOSE:
		case GCIP_FW_LOG_LEVEL_DEBUG:
			dev_dbg(dev, "%s", buffer);
			break;
		case GCIP_FW_LOG_LEVEL_WARN:
			dev_warn(dev, "%s", buffer);
			break;
		case GCIP_FW_LOG_LEVEL_FATAL:
		case GCIP_FW_LOG_LEVEL_ERROR:
			dev_err(dev, "%s", buffer);
			break;
		case GCIP_FW_LOG_LEVEL_INFO:
		default:
			dev_info(dev, "%s", buffer);
			break;
		}
	}
	kfree(buffer);
}

void gcip_telemetry_fw_trace(struct gcip_telemetry *trace)
{
	struct gcip_telemetry_header *header = trace->header;

	header->head = header->tail;
}

void gcip_telemetry_irq_handler(struct gcip_telemetry *tel)
{
	unsigned long flags;

	/*
	 * If the lock is held by other threads - it means either
	 *   1. The worker gcip_telemetry_worker is working, or
	 *   2. The telemetry object is being released
	 * Either way we don't need to schedule another job.
	 */
	if (!spin_trylock_irqsave(&tel->state_lock, flags))
		return;

	if (tel->state == GCIP_TELEMETRY_ENABLED && tel->header->head != tel->header->tail)
		schedule_work(&tel->work);

	spin_unlock_irqrestore(&tel->state_lock, flags);
}

void gcip_telemetry_inc_mmap_count(struct gcip_telemetry *tel, int dif)
{
	mutex_lock(&tel->mmap_lock);
	tel->mmapped_count += dif;
	mutex_unlock(&tel->mmap_lock);
}

int gcip_telemetry_mmap_buffer(struct gcip_telemetry *tel, int (*mmap)(void *), void *args)
{
	int ret;

	mutex_lock(&tel->mmap_lock);

	if (!tel->mmapped_count) {
		ret = mmap(args);

		if (!ret)
			tel->mmapped_count = 1;
	} else {
		ret = -EBUSY;
		dev_warn(tel->dev, "%s is already mmapped %ld times", tel->name,
			 tel->mmapped_count);
	}

	mutex_unlock(&tel->mmap_lock);

	return ret;
}

/* Worker for processing log/trace buffers. */
static void gcip_telemetry_worker(struct work_struct *work)
{
	struct gcip_telemetry *tel = container_of(work, struct gcip_telemetry, work);
	u32 prev_head;
	ulong state_lock_flags, ctx_lock_flags;

	/*
	 * Loops while telemetry enabled, there is data to be consumed, and the previous iteration
	 * made progress. If another IRQ arrives just after the last head != tail check we should
	 * get another worker schedule.
	 */
	do {
		spin_lock_irqsave(&tel->state_lock, state_lock_flags);
		if (tel->state != GCIP_TELEMETRY_ENABLED) {
			spin_unlock_irqrestore(&tel->state_lock, state_lock_flags);
			return;
		}

		prev_head = tel->header->head;
		if (tel->header->head != tel->header->tail) {
			read_lock_irqsave(&tel->ctx_lock, ctx_lock_flags);
			if (tel->ctx)
				eventfd_signal(tel->ctx, 1);
			else
				tel->fallback_fn(tel);
			read_unlock_irqrestore(&tel->ctx_lock, ctx_lock_flags);
		}

		spin_unlock_irqrestore(&tel->state_lock, state_lock_flags);
		msleep(GCIP_TELEMETRY_LOG_RECHECK_DELAY);
	} while (tel->header->head != tel->header->tail && tel->header->head != prev_head);
}

int gcip_telemetry_init(struct device *dev, struct gcip_telemetry *tel, const char *name,
			void *vaddr, const size_t size,
			void (*fallback_fn)(struct gcip_telemetry *))
{
	if (!is_power_of_2(size) || size <= sizeof(struct gcip_telemetry_header)) {
		dev_err(dev,
			"Size of GCIP telemetry buffer must be a power of 2 and greater than %zu.",
			sizeof(struct gcip_telemetry_header));
		return -EINVAL;
	}

	rwlock_init(&tel->ctx_lock);
	tel->name = name;
	tel->dev = dev;

	tel->header = vaddr;
	tel->header->head = 0;
	tel->header->tail = 0;
	tel->header->size = size;
	tel->header->entries_dropped = 0;

	tel->ctx = NULL;

	spin_lock_init(&tel->state_lock);
	INIT_WORK(&tel->work, gcip_telemetry_worker);
	tel->fallback_fn = fallback_fn;
	tel->state = GCIP_TELEMETRY_ENABLED;
	mutex_init(&tel->mmap_lock);
	tel->mmapped_count = 0;

	return 0;
}

void gcip_telemetry_exit(struct gcip_telemetry *tel)
{
	ulong flags;

	spin_lock_irqsave(&tel->state_lock, flags);
	/* Prevents racing with the IRQ handler or worker. */
	tel->state = GCIP_TELEMETRY_INVALID;
	spin_unlock_irqrestore(&tel->state_lock, flags);
	cancel_work_sync(&tel->work);

	if (tel->ctx)
		eventfd_ctx_put(tel->ctx);
	tel->ctx = NULL;
}
