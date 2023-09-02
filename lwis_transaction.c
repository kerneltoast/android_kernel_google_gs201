/*
 * Google LWIS Transaction Processor
 *
 * Copyright (c) 2019 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define pr_fmt(fmt) KBUILD_MODNAME "-transact: " fmt

#include "lwis_transaction.h"

#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/slab.h>

#include "lwis_allocator.h"
#include "lwis_device.h"
#include "lwis_event.h"
#include "lwis_io_entry.h"
#include "lwis_ioreg.h"
#include "lwis_util.h"

#define CREATE_TRACE_POINTS
#include "lwis_trace.h"

#define EXPLICIT_EVENT_COUNTER(x)                                                                  \
	((x) != LWIS_EVENT_COUNTER_ON_NEXT_OCCURRENCE && (x) != LWIS_EVENT_COUNTER_EVERY_TIME)

static struct lwis_transaction_event_list *event_list_find(struct lwis_client *client,
							   int64_t event_id)
{
	struct lwis_transaction_event_list *list;
	hash_for_each_possible (client->transaction_list, list, node, event_id) {
		if (list->event_id == event_id) {
			return list;
		}
	}
	return NULL;
}

static struct lwis_transaction_event_list *event_list_create(struct lwis_client *client,
							     int64_t event_id)
{
	struct lwis_transaction_event_list *event_list =
		kmalloc(sizeof(struct lwis_transaction_event_list), GFP_ATOMIC);
	if (!event_list) {
		dev_err(client->lwis_dev->dev, "Cannot allocate new event list\n");
		return NULL;
	}
	event_list->event_id = event_id;
	INIT_LIST_HEAD(&event_list->list);
	hash_add(client->transaction_list, &event_list->node, event_id);
	return event_list;
}

static struct lwis_transaction_event_list *event_list_find_or_create(struct lwis_client *client,
								     int64_t event_id)
{
	struct lwis_transaction_event_list *list = event_list_find(client, event_id);
	return (list == NULL) ? event_list_create(client, event_id) : list;
}

static void save_transaction_to_history(struct lwis_client *client,
					struct lwis_transaction_info *trans_info,
					int64_t process_timestamp, int64_t process_duration_ns)
{
	client->debug_info.transaction_hist[client->debug_info.cur_transaction_hist_idx].info =
		*trans_info;
	client->debug_info.transaction_hist[client->debug_info.cur_transaction_hist_idx]
		.process_timestamp = process_timestamp;
	client->debug_info.transaction_hist[client->debug_info.cur_transaction_hist_idx]
		.process_duration_ns = process_duration_ns;
	client->debug_info.cur_transaction_hist_idx++;
	if (client->debug_info.cur_transaction_hist_idx >= TRANSACTION_DEBUG_HISTORY_SIZE) {
		client->debug_info.cur_transaction_hist_idx = 0;
	}
}

void lwis_transaction_free(struct lwis_device *lwis_dev, struct lwis_transaction *transaction)
{
	int i;

	for (i = 0; i < transaction->info.num_io_entries; ++i) {
		if (transaction->info.io_entries[i].type == LWIS_IO_ENTRY_WRITE_BATCH) {
			lwis_allocator_free(lwis_dev, transaction->info.io_entries[i].rw_batch.buf);
			transaction->info.io_entries[i].rw_batch.buf = NULL;
		}
	}
	lwis_allocator_free(lwis_dev, transaction->info.io_entries);
	if (transaction->resp) {
		kfree(transaction->resp);
	}
	kfree(transaction);
}

static int process_transaction(struct lwis_client *client, struct lwis_transaction *transaction,
			       struct list_head *pending_events, bool in_irq, bool skip_err)
{
	int i;
	int ret = 0;
	struct lwis_io_entry *entry = NULL;
	struct lwis_device *lwis_dev = client->lwis_dev;
	struct lwis_transaction_info *info = &transaction->info;
	struct lwis_transaction_response_header *resp = transaction->resp;
	size_t resp_size;
	uint8_t *read_buf;
	struct lwis_io_result *io_result;
	const int reg_value_bytewidth = lwis_dev->native_value_bitwidth / 8;
	int64_t process_duration_ns = 0;
	int64_t process_timestamp = ktime_to_ns(lwis_get_time());

	LWIS_ATRACE_FUNC_BEGIN(lwis_dev);
	resp_size = sizeof(struct lwis_transaction_response_header) + resp->results_size_bytes;
	read_buf = (uint8_t *)resp + sizeof(struct lwis_transaction_response_header);
	resp->completion_index = -1;

	/* Use write memory barrier at the beginning of I/O entries if the access protocol
	 * allows it */
	if (lwis_dev->vops.register_io_barrier != NULL) {
		lwis_dev->vops.register_io_barrier(lwis_dev,
						   /*use_read_barrier=*/false,
						   /*use_write_barrier=*/true);
	}

	for (i = 0; i < info->num_io_entries; ++i) {
		entry = &info->io_entries[i];
		if (entry->type == LWIS_IO_ENTRY_WRITE ||
		    entry->type == LWIS_IO_ENTRY_WRITE_BATCH ||
		    entry->type == LWIS_IO_ENTRY_MODIFY) {
			ret = lwis_dev->vops.register_io(lwis_dev, entry,
							 lwis_dev->native_value_bitwidth);
			if (ret) {
				resp->error_code = ret;
				if (skip_err) {
					dev_warn(lwis_dev->dev,
						 "transaction type %d processing failed, skip this error and run the next command\n",
						 entry->type);
					continue;
				}
				break;
			}
		} else if (entry->type == LWIS_IO_ENTRY_READ) {
			io_result = (struct lwis_io_result *)read_buf;
			io_result->bid = entry->rw.bid;
			io_result->offset = entry->rw.offset;
			io_result->num_value_bytes = reg_value_bytewidth;
			ret = lwis_dev->vops.register_io(lwis_dev, entry,
							 lwis_dev->native_value_bitwidth);
			if (ret) {
				resp->error_code = ret;
				if (skip_err) {
					dev_warn(lwis_dev->dev,
						 "transaction type %d processing failed, skip this error and run the next command\n",
						 entry->type);
					continue;
				}
				break;
			}
			memcpy(io_result->values, &entry->rw.val, reg_value_bytewidth);
			read_buf += sizeof(struct lwis_io_result) + io_result->num_value_bytes;
		} else if (entry->type == LWIS_IO_ENTRY_READ_BATCH) {
			io_result = (struct lwis_io_result *)read_buf;
			io_result->bid = entry->rw_batch.bid;
			io_result->offset = entry->rw_batch.offset;
			io_result->num_value_bytes = entry->rw_batch.size_in_bytes;
			entry->rw_batch.buf = io_result->values;
			ret = lwis_dev->vops.register_io(lwis_dev, entry,
							 lwis_dev->native_value_bitwidth);
			if (ret) {
				resp->error_code = ret;
				if (skip_err) {
					dev_warn(lwis_dev->dev,
						 "transaction type %d processing failed, skip this error and run the next command\n",
						 entry->type);
					continue;
				}
				break;
			}
			read_buf += sizeof(struct lwis_io_result) + io_result->num_value_bytes;
		} else if (entry->type == LWIS_IO_ENTRY_POLL) {
			ret = lwis_io_entry_poll(lwis_dev, entry, in_irq);
			if (ret) {
				resp->error_code = ret;
				if (skip_err) {
					dev_warn(lwis_dev->dev,
						 "transaction type %d processing failed, skip this error and run the next command\n",
						 entry->type);
					continue;
				}
				break;
			}
		} else if (entry->type == LWIS_IO_ENTRY_READ_ASSERT) {
			ret = lwis_io_entry_read_assert(lwis_dev, entry);
			if (ret) {
				resp->error_code = ret;
				if (skip_err) {
					dev_warn(lwis_dev->dev,
						 "transaction type %d processing failed, skip this error and run the next command\n",
						 entry->type);
					continue;
				}
				break;
			}
		} else {
			dev_err(lwis_dev->dev, "Unrecognized io_entry command\n");
			resp->error_code = -EINVAL;
			if (skip_err) {
				dev_warn(lwis_dev->dev,
					 "transaction type %d processing failed, skip this error and run the next command\n",
					 entry->type);
				continue;
			}
			break;
		}
		resp->completion_index = i;
	}

	process_duration_ns = ktime_to_ns(lwis_get_time() - process_timestamp);

	/* Use read memory barrier at the end of I/O entries if the access protocol
	 * allows it */
	if (lwis_dev->vops.register_io_barrier != NULL) {
		lwis_dev->vops.register_io_barrier(lwis_dev, /*use_read_barrier=*/true,
						   /*use_write_barrier=*/false);
	}
	if (pending_events) {
		lwis_pending_event_push(pending_events,
					resp->error_code ? info->emit_error_event_id :
								 info->emit_success_event_id,
					(void *)resp, resp_size);
	} else {
		/* No pending events indicates it's cleanup io_entries. */
		if (resp->error_code) {
			dev_err(lwis_dev->dev,
				"Clean-up fails with error code %d, transaction %llu, io_entries[%d], entry_type %d",
				resp->error_code, transaction->info.id, i, entry->type);
		}
	}
	save_transaction_to_history(client, info, process_timestamp, process_duration_ns);
	if (info->trigger_event_counter == LWIS_EVENT_COUNTER_EVERY_TIME) {
		/* Only clean the transaction struct for this iteration. The
                 * I/O entries are not being freed. */
		kfree(transaction->resp);
		kfree(transaction);
	} else {
		lwis_transaction_free(lwis_dev, transaction);
	}
	LWIS_ATRACE_FUNC_END(lwis_dev);
	return ret;
}

static void cancel_transaction(struct lwis_device *lwis_dev, struct lwis_transaction *transaction,
			       int error_code, struct list_head *pending_events)
{
	struct lwis_transaction_info *info = &transaction->info;
	struct lwis_transaction_response_header resp;
	resp.id = info->id;
	resp.error_code = error_code;
	resp.num_entries = 0;
	resp.results_size_bytes = 0;
	resp.completion_index = -1;

	if (pending_events) {
		lwis_pending_event_push(pending_events, info->emit_error_event_id, &resp,
					sizeof(resp));
	}
	lwis_transaction_free(lwis_dev, transaction);
}

static void transaction_work_func(struct kthread_work *work)
{
	unsigned long flags;
	struct lwis_client *client = container_of(work, struct lwis_client, transaction_work);
	struct list_head *it_tran, *it_tran_tmp;
	struct list_head pending_events;
	struct lwis_transaction *transaction;

	INIT_LIST_HEAD(&pending_events);

	spin_lock_irqsave(&client->transaction_lock, flags);
	list_for_each_safe (it_tran, it_tran_tmp, &client->transaction_process_queue) {
		transaction = list_entry(it_tran, struct lwis_transaction, process_queue_node);
		list_del(&transaction->process_queue_node);
		if (transaction->resp->error_code) {
			cancel_transaction(client->lwis_dev, transaction,
					   transaction->resp->error_code, &pending_events);
		} else {
			spin_unlock_irqrestore(&client->transaction_lock, flags);
			process_transaction(client, transaction, &pending_events,
					    /*in_irq=*/false,
					    /*skip_err=*/false);
			spin_lock_irqsave(&client->transaction_lock, flags);
		}
	}
	spin_unlock_irqrestore(&client->transaction_lock, flags);

	lwis_pending_events_emit(client->lwis_dev, &pending_events, /*in_irq=*/false);
}

int lwis_transaction_init(struct lwis_client *client)
{
	spin_lock_init(&client->transaction_lock);
	INIT_LIST_HEAD(&client->transaction_process_queue);
	kthread_init_work(&client->transaction_work, transaction_work_func);
	client->transaction_counter = 0;
	hash_init(client->transaction_list);
	return 0;
}

int lwis_transaction_clear(struct lwis_client *client)
{
	int ret;

	ret = lwis_transaction_client_flush(client);
	if (ret) {
		dev_err(client->lwis_dev->dev,
			"Failed to wait for all in-process transactions to complete (%d)\n", ret);
		return ret;
	}
	return 0;
}

static void cancel_all_transactions_in_queue_locked(struct lwis_client *client,
					  struct list_head *transaction_queue)
{
	struct lwis_transaction *transaction;
	struct list_head *it_tran, *it_tran_tmp;

	if (!list_empty(transaction_queue)) {
		dev_warn(client->lwis_dev->dev, "Still transaction entries in process queue\n");
		list_for_each_safe (it_tran, it_tran_tmp, transaction_queue) {
			transaction =
				list_entry(it_tran, struct lwis_transaction, process_queue_node);
			list_del(&transaction->process_queue_node);
			cancel_transaction(client->lwis_dev, transaction, -ECANCELED, NULL);
		}
	}
}

int lwis_transaction_client_flush(struct lwis_client *client)
{
	unsigned long flags;
	struct list_head *it_tran, *it_tran_tmp;
	struct lwis_transaction *transaction;
	int i;
	struct hlist_node *tmp;
	struct lwis_transaction_event_list *it_evt_list;

	if (!client) {
		pr_err("Client pointer cannot be NULL while flushing transactions.\n");
		return -ENODEV;
	}

	spin_lock_irqsave(&client->transaction_lock, flags);
	hash_for_each_safe (client->transaction_list, i, tmp, it_evt_list, node) {
		if ((it_evt_list->event_id & 0xFFFF0000FFFFFFFFll) ==
		    LWIS_EVENT_ID_CLIENT_CLEANUP) {
			continue;
		}
		list_for_each_safe (it_tran, it_tran_tmp, &it_evt_list->list) {
			transaction = list_entry(it_tran, struct lwis_transaction, event_list_node);
			list_del(&transaction->event_list_node);
			cancel_transaction(client->lwis_dev, transaction, -ECANCELED, NULL);
		}
		hash_del(&it_evt_list->node);
		kfree(it_evt_list);
	}
	spin_unlock_irqrestore(&client->transaction_lock, flags);

	if (client->lwis_dev->transaction_worker_thread)
		kthread_flush_worker(&client->lwis_dev->transaction_worker);

	spin_lock_irqsave(&client->transaction_lock, flags);
	/* The transaction queue should be empty after canceling all transactions,
	 * but check anyway. */
	cancel_all_transactions_in_queue_locked(client, &client->transaction_process_queue);
	spin_unlock_irqrestore(&client->transaction_lock, flags);

	return 0;
}

static void print_buffer(struct lwis_client *client, const char *buf, int size) {
	char output_string[80], temp_string[8];
	int i;

	output_string[0] = '\0';
	for (i = 0; i < size; i++) {
		if (i % 16 == 0 && i != 0) {
			dev_info(client->lwis_dev->dev, "%s\n", output_string);
			output_string[0] = '\0';
		}
		sprintf(temp_string, "%02X ", buf[i]);
		if (i % 4 == 3) {
			strcat(temp_string, " ");
		}
		strcat(output_string, temp_string);
	}
	dev_info(client->lwis_dev->dev, "%s\n", output_string);
}

int lwis_transaction_client_cleanup(struct lwis_client *client)
{
	unsigned long flags;
	struct list_head *it_tran, *it_tran_tmp;
	struct lwis_transaction *transaction;
	struct lwis_transaction_event_list *it_evt_list;
	struct list_head pending_events;
	bool in_irq = false;
	struct lwis_event_entry *event;

	INIT_LIST_HEAD(&pending_events);

	spin_lock_irqsave(&client->transaction_lock, flags);
	/* Perform client defined clean-up routine. */
	it_evt_list = event_list_find(client, LWIS_EVENT_ID_CLIENT_CLEANUP |
						      (int64_t)client->lwis_dev->id
							      << LWIS_EVENT_ID_EVENT_CODE_LEN);
	if (it_evt_list == NULL) {
		spin_unlock_irqrestore(&client->transaction_lock, flags);
		return 0;
	}

	list_for_each_safe (it_tran, it_tran_tmp, &it_evt_list->list) {
		transaction = list_entry(it_tran, struct lwis_transaction, event_list_node);
		list_del(&transaction->event_list_node);
		if (transaction->resp->error_code || client->lwis_dev->enabled == 0) {
			cancel_transaction(client->lwis_dev, transaction, -ECANCELED, NULL);
		} else {
			spin_unlock_irqrestore(&client->transaction_lock, flags);
			process_transaction(client, transaction, &pending_events, in_irq,
					    /*skip_err=*/true);
			spin_lock_irqsave(&client->transaction_lock, flags);
		}
	}
	hash_del(&it_evt_list->node);
	kfree(it_evt_list);

	spin_unlock_irqrestore(&client->transaction_lock, flags);

	while (!list_empty(&pending_events)) {
		struct lwis_transaction_response_header *resp;
		struct lwis_io_result *result;
		char *results_buf;
		int i;

		event = list_first_entry(&pending_events, struct lwis_event_entry, node);
		/*
		   If there is no read result, the payload size will less than
		   sizeof(struct lwis_transaction_response_header).
		   We can skip it.
		*/
		if (event->event_info.payload_size <=
			sizeof(struct lwis_transaction_response_header)) {
			list_del(&event->node);
			kfree(event);
			continue;
		}
		resp = (struct lwis_transaction_response_header *)
			event->event_info.payload_buffer;
		dev_info(client->lwis_dev->dev, "event_id = %llx, num_entries = %ld\n",
			event->event_info.event_id, resp->num_entries);
		results_buf = (char *)event->event_info.payload_buffer +
			sizeof(struct lwis_transaction_response_header);
		for (i = 0; i < resp->num_entries; i++) {
			dev_info(client->lwis_dev->dev, "entry-%d\n", i);
			result = (struct lwis_io_result *)results_buf;
			print_buffer(client, result->values, result->num_value_bytes);
			results_buf += sizeof(struct lwis_io_result) +
				result->num_value_bytes;
		}

		list_del(&event->node);
		kfree(event);
	}

	return 0;
}

static int check_transaction_param_locked(struct lwis_client *client,
					  struct lwis_transaction *transaction,
					  bool allow_counter_eq)
{
	struct lwis_device_event_state *event_state;
	struct lwis_transaction_info *info = &transaction->info;
	struct lwis_device *lwis_dev = client->lwis_dev;

	if (!client) {
		pr_err("Client is NULL while checking transaction parameter.\n");
		return -ENODEV;
	}

	if (!transaction) {
		dev_err(lwis_dev->dev, "Transaction is NULL.\n");
		return -ENODEV;
	}

	/* Initialize event counter return value  */
	info->current_trigger_event_counter = -1LL;

	/* Look for the trigger event state, if specified */
	if (info->trigger_event_id != LWIS_EVENT_ID_NONE) {
		event_state = lwis_device_event_state_find(lwis_dev, info->trigger_event_id);
		if (event_state == NULL) {
			/* Event has not been encountered, setting event counter
			 * to zero */
			info->current_trigger_event_counter = 0;
		} else {
			/* Event found, return current counter to userspace */
			info->current_trigger_event_counter = event_state->event_counter;
		}
	}

	/* Both trigger event ID and counter are defined */
	if (info->trigger_event_id != LWIS_EVENT_ID_NONE &&
	    EXPLICIT_EVENT_COUNTER(info->trigger_event_counter)) {
		/* Check if event has happened already */
		if (info->trigger_event_counter == info->current_trigger_event_counter) {
			if (allow_counter_eq) {
				/* Convert this transaction into an immediate
				 * one */
				info->trigger_event_id = LWIS_EVENT_ID_NONE;
			} else {
				return -ENOENT;
			}
		} else if (info->trigger_event_counter < info->current_trigger_event_counter) {
			return -ENOENT;
		}
	}

	/* Make sure sw events exist in event table */
	if (IS_ERR_OR_NULL(lwis_device_event_state_find_or_create(lwis_dev,
								  info->emit_success_event_id)) ||
	    IS_ERR_OR_NULL(
		    lwis_client_event_state_find_or_create(client, info->emit_success_event_id)) ||
	    IS_ERR_OR_NULL(
		    lwis_device_event_state_find_or_create(lwis_dev, info->emit_error_event_id)) ||
	    IS_ERR_OR_NULL(
		    lwis_client_event_state_find_or_create(client, info->emit_error_event_id))) {
		dev_err(lwis_dev->dev, "Cannot create sw events for transaction");
		return -EINVAL;
	}

	return 0;
}

static int prepare_response_locked(struct lwis_client *client, struct lwis_transaction *transaction)
{
	struct lwis_transaction_info *info = &transaction->info;
	int i;
	size_t resp_size;
	size_t read_buf_size = 0;
	int read_entries = 0;
	const int reg_value_bytewidth = client->lwis_dev->native_value_bitwidth / 8;

	info->id = client->transaction_counter;

	for (i = 0; i < info->num_io_entries; ++i) {
		struct lwis_io_entry *entry = &info->io_entries[i];
		if (entry->type == LWIS_IO_ENTRY_READ) {
			read_buf_size += reg_value_bytewidth;
			read_entries++;
		} else if (entry->type == LWIS_IO_ENTRY_READ_BATCH) {
			read_buf_size += entry->rw_batch.size_in_bytes;
			read_entries++;
		}
	}

	// Event response payload consists of header, and address and
	// offset pairs.
	resp_size = sizeof(struct lwis_transaction_response_header) +
		    read_entries * sizeof(struct lwis_io_result) + read_buf_size;
	/* Revisit the use of GFP_ATOMIC here. Reason for this to be atomic is
	 * because this function can be called by transaction_replace while
	 * holding onto a spinlock. */
	transaction->resp = kmalloc(resp_size, GFP_ATOMIC);
	if (!transaction->resp) {
		dev_err(client->lwis_dev->dev, "Cannot allocate transaction response\n");
		return -ENOMEM;
	}
	transaction->resp->id = info->id;
	transaction->resp->error_code = 0;
	transaction->resp->completion_index = 0;
	transaction->resp->num_entries = read_entries;
	transaction->resp->results_size_bytes =
		read_entries * sizeof(struct lwis_io_result) + read_buf_size;
	return 0;
}

/* Calling this function requires holding the client's transaction_lock. */
static int queue_transaction_locked(struct lwis_client *client,
				    struct lwis_transaction *transaction)
{
	struct lwis_transaction_event_list *event_list;
	struct lwis_transaction_info *info = &transaction->info;

	if (info->trigger_event_id == LWIS_EVENT_ID_NONE) {
		/* Immediate trigger. */
		list_add_tail(&transaction->process_queue_node, &client->transaction_process_queue);
		kthread_queue_work(&client->lwis_dev->transaction_worker,
				   &client->transaction_work);
	} else {
		/* Trigger by event. */
		event_list = event_list_find_or_create(client, info->trigger_event_id);
		if (!event_list) {
			dev_err(client->lwis_dev->dev, "Cannot create transaction event list\n");
			kfree(transaction->resp);
			return -EINVAL;
		}
		list_add_tail(&transaction->event_list_node, &event_list->list);
	}
	info->submission_timestamp_ns = ktime_to_ns(ktime_get());
	client->transaction_counter++;
	return 0;
}

int lwis_transaction_submit_locked(struct lwis_client *client, struct lwis_transaction *transaction)
{
	int ret;
	struct lwis_transaction_info *info = &transaction->info;

	ret = check_transaction_param_locked(client, transaction,
					     /*allow_counter_eq=*/info->allow_counter_eq);

	if (ret) {
		return ret;
	}

	ret = prepare_response_locked(client, transaction);
	if (ret) {
		return ret;
	}

	ret = queue_transaction_locked(client, transaction);
	return ret;
}

static struct lwis_transaction *
new_repeating_transaction_iteration(struct lwis_client *client,
				    struct lwis_transaction *transaction)
{
	struct lwis_transaction *new_instance;
	uint8_t *resp_buf;

	/* Construct a new instance for repeating transactions */
	new_instance = kmalloc(sizeof(struct lwis_transaction), GFP_ATOMIC);
	if (!new_instance) {
		dev_err(client->lwis_dev->dev,
			"Failed to allocate repeating transaction instance\n");
		return NULL;
	}
	memcpy(&new_instance->info, &transaction->info, sizeof(struct lwis_transaction_info));

	/* Allocate response buffer */
	resp_buf = kmalloc(sizeof(struct lwis_transaction_response_header) +
				   transaction->resp->results_size_bytes,
			   GFP_ATOMIC);
	if (!resp_buf) {
		kfree(new_instance);
		dev_err(client->lwis_dev->dev,
			"Failed to allocate repeating transaction response\n");
		return NULL;
	}
	memcpy(resp_buf, transaction->resp, sizeof(struct lwis_transaction_response_header));
	new_instance->resp = (struct lwis_transaction_response_header *)resp_buf;

	INIT_LIST_HEAD(&new_instance->event_list_node);
	INIT_LIST_HEAD(&new_instance->process_queue_node);

	return new_instance;
}

static void defer_transaction_locked(struct lwis_client *client,
				     struct lwis_transaction *transaction,
				     struct list_head *pending_events, bool in_irq,
				     bool del_event_list_node)
{
	unsigned long flags = 0;
	if (del_event_list_node) {
		list_del(&transaction->event_list_node);
	}

	/* I2C read/write cannot be executed in IRQ context */
	if (in_irq && client->lwis_dev->type == DEVICE_TYPE_I2C) {
		list_add_tail(&transaction->process_queue_node, &client->transaction_process_queue);
		return;
	}

	if (transaction->info.run_in_event_context) {
		spin_unlock_irqrestore(&client->transaction_lock, flags);
		process_transaction(client, transaction, pending_events, in_irq,
				    /*skip_err=*/false);
		spin_lock_irqsave(&client->transaction_lock, flags);
	} else {
		list_add_tail(&transaction->process_queue_node, &client->transaction_process_queue);
	}
}

int lwis_transaction_event_trigger(struct lwis_client *client, int64_t event_id,
				   int64_t event_counter, struct list_head *pending_events,
				   bool in_irq)
{
	unsigned long flags;
	struct lwis_transaction_event_list *event_list;
	struct list_head *it_tran, *it_tran_tmp;
	struct lwis_transaction *transaction;
	struct lwis_transaction *new_instance;
	int64_t trigger_counter = 0;

	/* Find event list that matches the trigger event ID. */
	spin_lock_irqsave(&client->transaction_lock, flags);
	event_list = event_list_find(client, event_id);
	/* No event found, just return. */
	if (event_list == NULL || list_empty(&event_list->list)) {
		spin_unlock_irqrestore(&client->transaction_lock, flags);
		return 0;
	}

	/* Go through all transactions under the chosen event list. */
	list_for_each_safe (it_tran, it_tran_tmp, &event_list->list) {
		transaction = list_entry(it_tran, struct lwis_transaction, event_list_node);
		if (transaction->resp->error_code) {
			list_add_tail(&transaction->process_queue_node,
				      &client->transaction_process_queue);
			list_del(&transaction->event_list_node);
			continue;
		}

		/* Compare current event with trigger event counter to make
		 * sure this transaction needs to be executed now. */
		trigger_counter = transaction->info.trigger_event_counter;
		if (trigger_counter == LWIS_EVENT_COUNTER_ON_NEXT_OCCURRENCE ||
		    trigger_counter == event_counter) {
			defer_transaction_locked(client, transaction, pending_events, in_irq,
						 /* del_event_list_node */ true);
		} else if (trigger_counter == LWIS_EVENT_COUNTER_EVERY_TIME) {
			new_instance = new_repeating_transaction_iteration(client, transaction);
			if (!new_instance) {
				transaction->resp->error_code = -ENOMEM;
				list_add_tail(&transaction->process_queue_node,
					      &client->transaction_process_queue);
				list_del(&transaction->event_list_node);
				continue;
			}
			defer_transaction_locked(client, new_instance, pending_events, in_irq,
						 /* del_event_list_node */ false);
		}
	}

	/* Schedule deferred transactions */
	if (!list_empty(&client->transaction_process_queue)) {
		kthread_queue_work(&client->lwis_dev->transaction_worker,
				   &client->transaction_work);
	}

	spin_unlock_irqrestore(&client->transaction_lock, flags);

	return 0;
}

/* Calling this function requires holding the client's transaction_lock. */
static int cancel_waiting_transaction_locked(struct lwis_client *client, int64_t id)
{
	int i;
	struct hlist_node *tmp;
	struct list_head *it_tran, *it_tran_tmp;
	struct lwis_transaction_event_list *it_evt_list;
	struct lwis_transaction *transaction;

	hash_for_each_safe (client->transaction_list, i, tmp, it_evt_list, node) {
		list_for_each_safe (it_tran, it_tran_tmp, &it_evt_list->list) {
			transaction = list_entry(it_tran, struct lwis_transaction, event_list_node);
			if (transaction->info.id == id) {
				transaction->resp->error_code = -ECANCELED;
				return 0;
			}
		}
	}
	return -ENOENT;
}

int lwis_transaction_cancel(struct lwis_client *client, int64_t id)
{
	int ret;
	unsigned long flags;

	spin_lock_irqsave(&client->transaction_lock, flags);
	ret = cancel_waiting_transaction_locked(client, id);
	spin_unlock_irqrestore(&client->transaction_lock, flags);

	return ret;
}

int lwis_transaction_replace_locked(struct lwis_client *client,
				    struct lwis_transaction *transaction)
{
	int ret;

	ret = check_transaction_param_locked(client, transaction,
					     /*allow_counter_eq=*/false);
	if (ret) {
		return ret;
	}

	ret = cancel_waiting_transaction_locked(client, transaction->info.id);
	if (ret) {
		return ret;
	}

	ret = prepare_response_locked(client, transaction);
	if (ret) {
		return ret;
	}

	ret = queue_transaction_locked(client, transaction);
	return ret;
}
