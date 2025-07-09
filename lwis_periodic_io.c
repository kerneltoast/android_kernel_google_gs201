// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google LWIS Periodic IO Processor
 *
 * Copyright (c) 2020 Google, LLC
 */

#define pr_fmt(fmt) KBUILD_MODNAME "-periodic: " fmt

#include "lwis_periodic_io.h"

#include <linux/completion.h>
#include <linux/slab.h>
#include <linux/overflow.h>

#include "lwis_allocator.h"
#include "lwis_device.h"
#include "lwis_event.h"
#include "lwis_io_entry.h"
#include "lwis_ioreg.h"
#include "lwis_transaction.h"
#include "lwis_util.h"
#include "lwis_bus_manager.h"

static enum hrtimer_restart periodic_io_timer_func(struct hrtimer *timer)
{
	ktime_t interval;
	unsigned long flags;
	struct list_head *it_period, *it_period_tmp;
	struct lwis_periodic_io_list *periodic_io_list;
	struct lwis_periodic_io *periodic_io;
	struct lwis_periodic_io_proxy *periodic_io_proxy;
	struct lwis_client *client;
	bool active_periodic_io_present = false;

	periodic_io_list = container_of(timer, struct lwis_periodic_io_list, hr_timer);
	client = periodic_io_list->client;

	/* Go through all periodic io under the chosen periodic list */
	spin_lock_irqsave(&client->periodic_io_lock, flags);
	list_for_each_safe(it_period, it_period_tmp, &periodic_io_list->list) {
		periodic_io = list_entry(it_period, struct lwis_periodic_io, timer_list_node);
		if (periodic_io->active) {
			periodic_io_proxy = lwis_allocator_allocate(
				client->lwis_dev, sizeof(*periodic_io_proxy), GFP_ATOMIC);
			if (!periodic_io_proxy) {
				/* Non-fatal, skip this period */
				dev_warn(client->lwis_dev->dev,
					 "Cannot allocate new periodic io proxy.\n");
			} else {
				periodic_io_proxy->periodic_io = periodic_io;
				list_add_tail(&periodic_io_proxy->process_queue_node,
					      &client->periodic_io_process_queue);
				active_periodic_io_present = true;
			}
		}
	}

	if (active_periodic_io_present)
		lwis_queue_device_worker(client);

	spin_unlock_irqrestore(&client->periodic_io_lock, flags);
	if (!active_periodic_io_present) {
		periodic_io_list->hr_timer_state = LWIS_HRTIMER_INACTIVE;
		return HRTIMER_NORESTART;
	}

	interval = ktime_set(0, periodic_io_list->period_ns);
	hrtimer_forward_now(timer, interval);

	return HRTIMER_RESTART;
}

static struct lwis_periodic_io_list *periodic_io_list_find_locked(struct lwis_client *client,
								  int64_t period_ns)
{
	struct lwis_periodic_io_list *list;

	hash_for_each_possible(client->timer_list, list, node, period_ns) {
		if (list->period_ns == period_ns)
			return list;
	}
	return NULL;
}

/* Calling this function requires holding the client's periodic_io_lock */
static struct lwis_periodic_io_list *periodic_io_list_create_locked(struct lwis_client *client,
								    int64_t period_ns)
{
	ktime_t ktime;
	struct lwis_device *lwis_dev = client->lwis_dev;
	struct lwis_periodic_io_list *periodic_io_list =
		kmalloc(sizeof(struct lwis_periodic_io_list), GFP_ATOMIC);

	if (!periodic_io_list)
		return NULL;

	periodic_io_list->client = client;
	periodic_io_list->period_ns = period_ns;
	periodic_io_list->hr_timer_state = LWIS_HRTIMER_ACTIVE;

	/* Initialize the periodic io list and add this timer/periodic_io_list
	 * into the client timer list
	 */
	INIT_LIST_HEAD(&periodic_io_list->list);
	hash_add(client->timer_list, &periodic_io_list->node, period_ns);
	dev_info(lwis_dev->dev, "Created hrtimer with timeout time %lldns", period_ns);

	/* Initialize and start the hrtimer for this periodic io list */
	hrtimer_init(&periodic_io_list->hr_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	periodic_io_list->hr_timer.function = &periodic_io_timer_func;
	ktime = ktime_set(0, periodic_io_list->period_ns);
	hrtimer_start(&periodic_io_list->hr_timer, ktime, HRTIMER_MODE_REL);

	return periodic_io_list;
}

/* Calling this function requires holding the client's periodic_io_lock */
static struct lwis_periodic_io_list *
periodic_io_list_find_or_create_locked(struct lwis_client *client, int64_t period_ns)
{
	struct lwis_periodic_io_list *list = periodic_io_list_find_locked(client, period_ns);

	if (list == NULL)
		return periodic_io_list_create_locked(client, period_ns);

	/* If there is already a timer with the same period and it is inactive,
	 * then restart the timer
	 */
	if (list->hr_timer_state == LWIS_HRTIMER_INACTIVE) {
		list->hr_timer_state = LWIS_HRTIMER_ACTIVE;
		/* If this does not restart the hrtimer properly, consider
		 * repeating the steps when starting a new timer.
		 */
		hrtimer_restart(&list->hr_timer);
	}
	return list;
}

/* Calling this function requires holding the client's periodic_io_lock */
static void push_periodic_io_error_event_locked(struct lwis_periodic_io *periodic_io,
						int error_code, struct list_head *pending_events)
{
	struct lwis_periodic_io_info *info = &periodic_io->info;
	struct lwis_periodic_io_response_header resp;

	if (!pending_events)
		return;

	resp.id = info->id;
	resp.error_code = error_code;
	resp.batch_size = 0;
	resp.num_entries_per_period = 0;
	resp.results_size_bytes = 0;

	lwis_pending_event_push(pending_events, info->emit_error_event_id, &resp, sizeof(resp));
}

static int process_io_entries(struct lwis_client *client,
			      struct lwis_periodic_io_proxy *periodic_io_proxy,
			      struct list_head *pending_events)
{
	int i;
	int ret = 0;
	struct lwis_io_entry *entry = NULL;
	struct lwis_device *lwis_dev = client->lwis_dev;
	struct lwis_periodic_io *periodic_io = periodic_io_proxy->periodic_io;
	struct lwis_periodic_io_info *info = &periodic_io->info;
	struct lwis_periodic_io_response_header *resp = periodic_io->resp;
	size_t resp_size;
	uint8_t *read_buf;
	struct lwis_periodic_io_result *io_result;
	const int reg_value_bytewidth = lwis_dev->native_value_bitwidth / 8;
	unsigned long flags;

	read_buf = (uint8_t *)resp + sizeof(struct lwis_periodic_io_response_header) +
		   periodic_io->batch_count * (resp->results_size_bytes / info->batch_size);

	/* Use write memory barrier at the beginning of I/O entries if the access protocol
	 * allows it
	 */
	if (lwis_dev->vops.register_io_barrier != NULL) {
		lwis_dev->vops.register_io_barrier(lwis_dev,
						   /*use_read_barrier=*/false,
						   /*use_write_barrier=*/true);
	}

	reinit_completion(&periodic_io->io_done);
	lwis_bus_manager_lock_bus(lwis_dev);
	for (i = 0; i < info->num_io_entries; ++i) {
		/* Abort if periodic io is deactivated during processing.
		 * Abort can only apply to <= 1 write entries to prevent partial writes,
		 * or we just started the process.
		 */
		spin_lock_irqsave(&client->periodic_io_lock, flags);
		if (!periodic_io->active && (i == 0 || !periodic_io->contains_multiple_writes)) {
			resp->error_code = -ECANCELED;
			spin_unlock_irqrestore(&client->periodic_io_lock, flags);
			goto event_push;
		}
		spin_unlock_irqrestore(&client->periodic_io_lock, flags);
		entry = &info->io_entries[i];
		if (entry->type == LWIS_IO_ENTRY_WRITE || entry->type == LWIS_IO_ENTRY_WRITE_V2 ||
		    entry->type == LWIS_IO_ENTRY_WRITE_BATCH ||
		    entry->type == LWIS_IO_ENTRY_WRITE_BATCH_V2 ||
		    entry->type == LWIS_IO_ENTRY_MODIFY) {
			ret = lwis_dev->vops.register_io(lwis_dev, entry,
							 lwis_dev->native_value_bitwidth);
			if (ret) {
				resp->error_code = ret;
				goto event_push;
			}
		} else if (entry->type == LWIS_IO_ENTRY_READ ||
			   entry->type == LWIS_IO_ENTRY_READ_V2) {
			io_result = (struct lwis_periodic_io_result *)read_buf;
			io_result->io_result.bid = entry->rw.bid;
			io_result->io_result.offset = entry->rw.offset;
			io_result->io_result.num_value_bytes = reg_value_bytewidth;
			io_result->timestamp_ns = ktime_to_ns(lwis_get_time());
			ret = lwis_dev->vops.register_io(lwis_dev, entry,
							 lwis_dev->native_value_bitwidth);
			if (ret) {
				resp->error_code = ret;
				goto event_push;
			}
			memcpy(io_result->io_result.values, &entry->rw.val, reg_value_bytewidth);
			read_buf += sizeof(struct lwis_periodic_io_result) +
				    io_result->io_result.num_value_bytes;
		} else if (entry->type == LWIS_IO_ENTRY_READ_BATCH ||
			   entry->type == LWIS_IO_ENTRY_READ_BATCH_V2) {
			io_result = (struct lwis_periodic_io_result *)read_buf;
			io_result->io_result.bid = entry->rw_batch.bid;
			io_result->io_result.offset = entry->rw_batch.offset;
			io_result->io_result.num_value_bytes = entry->rw_batch.size_in_bytes;
			entry->rw_batch.buf = io_result->io_result.values;
			io_result->timestamp_ns = ktime_to_ns(lwis_get_time());
			ret = lwis_dev->vops.register_io(lwis_dev, entry,
							 lwis_dev->native_value_bitwidth);
			if (ret) {
				resp->error_code = ret;
				goto event_push;
			}
			read_buf += sizeof(struct lwis_periodic_io_result) +
				    io_result->io_result.num_value_bytes;
		} else if (entry->type == LWIS_IO_ENTRY_POLL) {
			ret = lwis_io_entry_poll(lwis_dev, entry, /*is_short=*/false);
			if (ret) {
				resp->error_code = ret;
				goto event_push;
			}
		} else if (entry->type == LWIS_IO_ENTRY_POLL_SHORT) {
			ret = lwis_io_entry_poll(lwis_dev, entry, /*is_short=*/true);
			if (ret) {
				resp->error_code = ret;
				goto event_push;
			}
		} else if (entry->type == LWIS_IO_ENTRY_WAIT) {
			ret = lwis_io_entry_wait(lwis_dev, entry);
			if (ret) {
				resp->error_code = ret;
				goto event_push;
			}
		} else if (entry->type == LWIS_IO_ENTRY_READ_ASSERT) {
			ret = lwis_io_entry_read_assert(lwis_dev, entry);
			if (ret) {
				resp->error_code = ret;
				goto event_push;
			}
		} else if (entry->type == LWIS_IO_ENTRY_IGNORE) {
			ret = 0;
		} else {
			pr_err_ratelimited("Unrecognized io_entry command\n");
			resp->error_code = -EINVAL;
			goto event_push;
		}
	}
	periodic_io->batch_count++;
	resp->batch_size = periodic_io->batch_count;

event_push:
	complete(&periodic_io->io_done);
	lwis_bus_manager_unlock_bus(lwis_dev);

	/* Use read memory barrier at the beginning of I/O entries if the access protocol
	 * allows it
	 */
	if (lwis_dev->vops.register_io_barrier != NULL) {
		lwis_dev->vops.register_io_barrier(lwis_dev, /*use_read_barrier=*/true,
						   /*use_write_barrier=*/false);
	}
	resp_size = sizeof(struct lwis_periodic_io_response_header) +
		    periodic_io->batch_count * (resp->results_size_bytes / info->batch_size);

	/* Only push when the periodic io is executed for batch_size times or
	 * there is an error
	 */
	if (!pending_events) {
		if (resp->error_code && resp->error_code != -ECANCELED) {
			dev_info(
				lwis_dev->dev,
				"%s fails with error code %d, periodic io %lld, io_entries[%d], entry_type %d",
				__func__, resp->error_code, info->id, i, entry->type);
		}
		return ret;
	}

	if (resp->error_code) {
		/* Adjust results_size_bytes to be consistent with payload
		 * size. Push error event, which also copies resp.
		 */
		resp->results_size_bytes =
			resp_size - sizeof(struct lwis_periodic_io_response_header);
		lwis_pending_event_push(pending_events, info->emit_error_event_id, (void *)resp,
					resp_size);

		/* Flag the periodic io as inactive */
		spin_lock_irqsave(&client->periodic_io_lock, flags);
		periodic_io->active = false;
		spin_unlock_irqrestore(&client->periodic_io_lock, flags);
	} else {
		if (periodic_io->batch_count == info->batch_size) {
			lwis_pending_event_push(pending_events, info->emit_success_event_id,
						(void *)resp, resp_size);
			periodic_io->batch_count = 0;
			resp->batch_size = 0;
		}
	}
	return ret;
}

void lwis_process_periodic_io_in_queue(struct lwis_client *client)
{
	int error_code;
	unsigned long flags;
	struct lwis_periodic_io *periodic_io;
	struct lwis_periodic_io_proxy *periodic_io_proxy;
	struct list_head *it_period, *it_period_tmp;
	struct list_head pending_events;

	INIT_LIST_HEAD(&pending_events);

	spin_lock_irqsave(&client->periodic_io_lock, flags);
	list_for_each_safe(it_period, it_period_tmp, &client->periodic_io_process_queue) {
		periodic_io_proxy =
			list_entry(it_period, struct lwis_periodic_io_proxy, process_queue_node);
		periodic_io = periodic_io_proxy->periodic_io;
		list_del(&periodic_io_proxy->process_queue_node);
		/* Error indicates the cancellation of the periodic io */
		if (periodic_io->resp->error_code || !periodic_io->active) {
			error_code = periodic_io->resp->error_code ? periodic_io->resp->error_code :
								     -ECANCELED;
			push_periodic_io_error_event_locked(periodic_io, error_code,
							    &pending_events);
		} else {
			spin_unlock_irqrestore(&client->periodic_io_lock, flags);
			process_io_entries(client, periodic_io_proxy, &pending_events);
			spin_lock_irqsave(&client->periodic_io_lock, flags);
		}
		lwis_allocator_free(client->lwis_dev, periodic_io_proxy);
	}
	spin_unlock_irqrestore(&client->periodic_io_lock, flags);
	lwis_pending_events_emit(client->lwis_dev, &pending_events);
}

static int prepare_emit_events(struct lwis_client *client, struct lwis_periodic_io *periodic_io)
{
	struct lwis_periodic_io_info *info = &periodic_io->info;
	struct lwis_device *lwis_dev = client->lwis_dev;

	/* Make sure sw events exist in event table */
	if (IS_ERR_OR_NULL(lwis_device_event_state_find_or_create(lwis_dev,
								  info->emit_success_event_id)) ||
	    IS_ERR_OR_NULL(
		    lwis_client_event_state_find_or_create(client, info->emit_success_event_id)) ||
	    IS_ERR_OR_NULL(
		    lwis_device_event_state_find_or_create(lwis_dev, info->emit_error_event_id)) ||
	    IS_ERR_OR_NULL(
		    lwis_client_event_state_find_or_create(client, info->emit_error_event_id))) {
		pr_err_ratelimited("Cannot create sw event for periodic io");
		return -EINVAL;
	}

	return 0;
}

static int prepare_response(struct lwis_client *client, struct lwis_periodic_io *periodic_io)
{
	struct lwis_periodic_io_info *info = &periodic_io->info;
	int i;
	size_t resp_size;
	size_t read_buf_size = 0;
	int read_entries = 0;
	const int reg_value_bytewidth = client->lwis_dev->native_value_bitwidth / 8;
	unsigned long flags;
	size_t result_size, read_buf_total, temp_size, total_size;

	spin_lock_irqsave(&client->periodic_io_lock, flags);
	info->id = client->periodic_io_counter;
	spin_unlock_irqrestore(&client->periodic_io_lock, flags);

	for (i = 0; i < info->num_io_entries; ++i) {
		struct lwis_io_entry *entry = &info->io_entries[i];

		if (entry->type == LWIS_IO_ENTRY_READ || entry->type == LWIS_IO_ENTRY_READ_V2) {
			/* Check for size_t overflow. */
			if (read_buf_size + reg_value_bytewidth < read_buf_size)
				return -EOVERFLOW;
			read_buf_size += reg_value_bytewidth;
			read_entries++;
		} else if (entry->type == LWIS_IO_ENTRY_READ_BATCH ||
			   entry->type == LWIS_IO_ENTRY_READ_BATCH_V2) {
			/* Check for size_t overflow when adding user defined size_in_bytes. */
			if (read_buf_size + entry->rw_batch.size_in_bytes < read_buf_size)
				return -EOVERFLOW;
			read_buf_size += entry->rw_batch.size_in_bytes;
			read_entries++;
		}
	}

	if (check_mul_overflow(read_entries, sizeof(struct lwis_periodic_io_result),
			       &result_size) ||
	    check_mul_overflow(result_size, info->batch_size, &result_size) ||
	    check_mul_overflow(read_buf_size, info->batch_size, &read_buf_total) ||
	    check_add_overflow(sizeof(struct lwis_periodic_io_response_header), result_size,
			       &temp_size) ||
	    check_add_overflow(temp_size, read_buf_total, &total_size)) {
		return -EOVERFLOW;
	}

	/* Periodic io response payload consists of one response header and
	 * batch_size of batches, each of which contains num_entries_per_period
	 * pairs of lwis_periodic_io_result and its read_buf.
	 */
	resp_size = sizeof(struct lwis_periodic_io_response_header) + result_size + read_buf_total;
	periodic_io->resp = kmalloc(resp_size, GFP_KERNEL);
	if (!periodic_io->resp)
		return -ENOMEM;

	periodic_io->resp->batch_size = 0;
	periodic_io->resp->error_code = 0;
	periodic_io->resp->id = info->id;
	periodic_io->resp->num_entries_per_period = read_entries;
	periodic_io->resp->results_size_bytes =
		read_entries * sizeof(struct lwis_periodic_io_result) * info->batch_size +
		read_buf_size * info->batch_size;

	periodic_io->batch_count = 0;
	return 0;
}

/* The periodic io lock of the client must be acquired before calling this
 * function
 */
static int queue_periodic_io_locked(struct lwis_client *client,
				    struct lwis_periodic_io *periodic_io)
{
	int64_t period_ns;
	struct lwis_periodic_io_list *periodic_io_list;
	struct lwis_periodic_io_info *info = &periodic_io->info;

	period_ns = info->period_ns;
	periodic_io_list = periodic_io_list_find_or_create_locked(client, period_ns);
	if (!periodic_io_list) {
		pr_err_ratelimited("Cannot create timer/periodic io list\n");
		kfree(periodic_io->resp);
		return -EINVAL;
	}
	periodic_io->periodic_io_list = periodic_io_list;
	list_add_tail(&periodic_io->timer_list_node, &periodic_io_list->list);
	client->periodic_io_counter++;
	return 0;
}

void lwis_periodic_io_free(struct lwis_device *lwis_dev, struct lwis_periodic_io *periodic_io)
{
	int i;

	for (i = 0; i < periodic_io->info.num_io_entries; ++i) {
		if (periodic_io->info.io_entries[i].type == LWIS_IO_ENTRY_WRITE_BATCH ||
		    periodic_io->info.io_entries[i].type == LWIS_IO_ENTRY_WRITE_BATCH_V2) {
			lwis_allocator_free(lwis_dev, periodic_io->info.io_entries[i].rw_batch.buf);
			periodic_io->info.io_entries[i].rw_batch.buf = NULL;
		}
	}
	lwis_allocator_free(lwis_dev, periodic_io->info.io_entries);

	/* resp may not be allocated before the periodic_io is successfully submitted */
	kfree(periodic_io->resp);
	kfree(periodic_io);
}

int lwis_periodic_io_init(struct lwis_client *client)
{
	INIT_LIST_HEAD(&client->periodic_io_process_queue);
	client->periodic_io_counter = 0;
	hash_init(client->timer_list);
	return 0;
}

int lwis_periodic_io_submit(struct lwis_client *client, struct lwis_periodic_io *periodic_io)
{
	int ret, i;
	bool has_one_write = false;
	unsigned long flags;
	struct lwis_periodic_io_info *info = &periodic_io->info;

	periodic_io->contains_multiple_writes = false;
	for (i = 0; i < info->num_io_entries; ++i) {
		struct lwis_io_entry *entry = &info->io_entries[i];

		if (entry->type == LWIS_IO_ENTRY_WRITE ||
		    entry->type == LWIS_IO_ENTRY_WRITE_BATCH ||
		    entry->type == LWIS_IO_ENTRY_WRITE_V2 ||
		    entry->type == LWIS_IO_ENTRY_WRITE_BATCH_V2 ||
		    entry->type == LWIS_IO_ENTRY_MODIFY) {
			if (has_one_write) {
				periodic_io->contains_multiple_writes = true;
				break;
			}
			has_one_write = true;
		}
	}

	ret = prepare_emit_events(client, periodic_io);
	if (ret)
		return ret;

	ret = prepare_response(client, periodic_io);
	if (ret)
		return ret;

	/* Initialize but mark io as complete as it is not run yet  */
	init_completion(&periodic_io->io_done);
	complete(&periodic_io->io_done);
	spin_lock_irqsave(&client->periodic_io_lock, flags);
	periodic_io->active = true;
	ret = queue_periodic_io_locked(client, periodic_io);
	spin_unlock_irqrestore(&client->periodic_io_lock, flags);
	return ret;
}

int lwis_periodic_io_client_flush(struct lwis_client *client)
{
	int i;
	struct hlist_node *tmp;
	struct list_head *it_period, *it_period_tmp;
	struct lwis_periodic_io *periodic_io;
	struct lwis_periodic_io_list *it_periodic_io_list;
	unsigned long flags;

	struct lwis_periodic_io *periodic_cleanup_io;
	struct lwis_periodic_io_proxy *periodic_cleanup_io_proxy;
	struct list_head *it_cleanup_period, *it_cleanup_period_tmp;

	spin_lock_irqsave(&client->periodic_io_lock, flags);
	/* First, cancel all timers */
	hash_for_each_safe(client->timer_list, i, tmp, it_periodic_io_list, node) {
		list_for_each_safe(it_period, it_period_tmp, &it_periodic_io_list->list) {
			periodic_io =
				list_entry(it_period, struct lwis_periodic_io, timer_list_node);
			periodic_io->active = false;
		}
		it_periodic_io_list->hr_timer_state = LWIS_HRTIMER_INACTIVE;
		spin_unlock_irqrestore(&client->periodic_io_lock, flags);
		hrtimer_cancel(&it_periodic_io_list->hr_timer);
		spin_lock_irqsave(&client->periodic_io_lock, flags);
	}
	spin_unlock_irqrestore(&client->periodic_io_lock, flags);

	/* Wait until all workload in process queue are processed */
	lwis_flush_device_worker(client);

	spin_lock_irqsave(&client->periodic_io_lock, flags);
	/* Cleanup any stale entries remaining after the flush */
	list_for_each_safe(it_cleanup_period, it_cleanup_period_tmp,
			   &client->periodic_io_process_queue) {
		periodic_cleanup_io_proxy = list_entry(
			it_cleanup_period, struct lwis_periodic_io_proxy, process_queue_node);
		if (periodic_cleanup_io_proxy) {
			periodic_cleanup_io = periodic_cleanup_io_proxy->periodic_io;
			list_del(&periodic_cleanup_io_proxy->process_queue_node);
			if (periodic_cleanup_io)
				periodic_cleanup_io->active = false;
			lwis_allocator_free(client->lwis_dev, periodic_cleanup_io_proxy);
		}
	}

	/* Release the periodic io list of from all timers */
	hash_for_each_safe(client->timer_list, i, tmp, it_periodic_io_list, node) {
		list_for_each_safe(it_period, it_period_tmp, &it_periodic_io_list->list) {
			periodic_io =
				list_entry(it_period, struct lwis_periodic_io, timer_list_node);
			list_del(it_period);
			lwis_periodic_io_free(client->lwis_dev, periodic_io);
		}
	}
	spin_unlock_irqrestore(&client->periodic_io_lock, flags);

	return 0;
}

int lwis_periodic_io_client_cleanup(struct lwis_client *client)
{
	int i, ret;
	struct hlist_node *tmp;
	struct lwis_periodic_io_list *it_periodic_io_list;
	unsigned long flags;

	ret = lwis_periodic_io_client_flush(client);
	if (ret) {
		dev_err(client->lwis_dev->dev,
			"Failed to wait for all in-process periodic io to complete\n");
		return ret;
	}

	spin_lock_irqsave(&client->periodic_io_lock, flags);
	hash_for_each_safe(client->timer_list, i, tmp, it_periodic_io_list, node) {
		hash_del(&it_periodic_io_list->node);
	}
	spin_unlock_irqrestore(&client->periodic_io_lock, flags);
	return 0;
}

/* Calling this function requires holding the client's periodic_io_lock */
static int mark_periodic_io_resp_error_locked(struct lwis_periodic_io *periodic_io)
{
	periodic_io->resp->error_code = -ECANCELED;
	periodic_io->active = false;
	return 0;
}

/* Calling this function requires holding the client's periodic_io_lock */
static struct lwis_periodic_io *periodic_io_find_locked(struct lwis_client *client, int64_t id)
{
	int i;
	struct hlist_node *tmp;
	struct list_head *it_period, *it_period_tmp;
	struct lwis_periodic_io_list *it_list;
	struct lwis_periodic_io *periodic_io;

	hash_for_each_safe(client->timer_list, i, tmp, it_list, node) {
		list_for_each_safe(it_period, it_period_tmp, &it_list->list) {
			periodic_io =
				list_entry(it_period, struct lwis_periodic_io, timer_list_node);
			if (periodic_io->info.id == id)
				return periodic_io;
		}
	}
	return NULL;
}

int lwis_periodic_io_cancel(struct lwis_client *client, int64_t id)
{
	int ret;
	unsigned long flags;
	struct lwis_periodic_io *periodic_io;

	/* Always search for the id in the list. The id may be valid(still an
	 * erroreous usage from user space), but not queued into the list yet.
	 * Mark the periodic_io resp as error and leverage the work_func
	 * procedure to handle the cancellation to avoid racing.
	 */
	spin_lock_irqsave(&client->periodic_io_lock, flags);
	periodic_io = periodic_io_find_locked(client, id);
	if (periodic_io != NULL)
		ret = mark_periodic_io_resp_error_locked(periodic_io);
	else
		ret = -ENOENT;

	spin_unlock_irqrestore(&client->periodic_io_lock, flags);
	if (!ret) {
		/* If there is any ongoing io, wait until it's finished */
		wait_for_completion(&periodic_io->io_done);
	}
	return ret;
}
