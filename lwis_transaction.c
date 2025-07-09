// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google LWIS Transaction Processor
 *
 * Copyright (c) 2019 Google, LLC
 */

#include "lwis_transaction.h"

#include <linux/dma-fence.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/err.h>
#include <linux/sync_file.h>
#include <linux/delay.h>
#include <linux/dma-buf.h>
#include <linux/mm.h>
#include <linux/preempt.h>
#include <linux/slab.h>
#include <linux/dma-fence.h>
#include <linux/err.h>

#include "lwis_allocator.h"
#include "lwis_commands.h"
#include "lwis_device.h"
#include "lwis_event.h"
#include "lwis_fence.h"
#include "lwis_bus_manager.h"
#include "lwis_io_buffer.h"
#include "lwis_io_entry.h"
#include "lwis_util.h"

#define CREATE_TRACE_POINTS
#include "lwis_trace.h"

/* Triggered event to actual execution of transaction threshold */
#define TRIGGRED_EVENT_EXECUTION_THRESHOLD_MS 5

bool lwis_transaction_debug;
module_param(lwis_transaction_debug, bool, 0644);

static struct lwis_transaction_event_list *event_list_find(struct lwis_client *client,
							   int64_t event_id)
{
	struct lwis_transaction_event_list *list;

	hash_for_each_possible(client->transaction_list, list, node, event_id) {
		if (list->event_id == event_id)
			return list;
	}
	return NULL;
}

static struct lwis_transaction_event_list *event_list_create(struct lwis_client *client,
							     int64_t event_id)
{
	struct lwis_transaction_event_list *event_list =
		kmalloc(sizeof(struct lwis_transaction_event_list), GFP_ATOMIC);
	if (!event_list)
		return NULL;

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

static void add_pending_transaction(struct lwis_client *client,
				    struct lwis_transaction *transaction)
{
	hash_add(client->pending_transactions, &transaction->pending_map_node,
		 transaction->info.id);
	lwis_debug_dev_info(client->lwis_dev->dev,
			    "lwis_fence add transaction id %llu to lwis_client pending map",
			    transaction->info.id);
}

static struct lwis_transaction *pending_transaction_peek(struct lwis_client *client,
							 int64_t transaction_id)
{
	struct hlist_node *tmp;
	struct lwis_transaction *transaction;

	hash_for_each_possible_safe(client->pending_transactions, transaction, tmp,
				    pending_map_node, transaction_id) {
		if (transaction->info.id == transaction_id)
			return transaction;
	}
	return NULL;
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
	if (client->debug_info.cur_transaction_hist_idx >= TRANSACTION_DEBUG_HISTORY_SIZE)
		client->debug_info.cur_transaction_hist_idx = 0;
}

void lwis_transaction_free(struct lwis_device *lwis_dev, struct lwis_transaction **lwis_tx)
{
	int i;
	struct lwis_fence_pending_signal *pending_fence;
	struct list_head *it_fence, *it_fence_tmp;
	struct lwis_pending_transaction_id *pend_id, *pend_id_tmp;
	struct lwis_transaction *transaction = *lwis_tx;

	if (transaction->is_weak_transaction) {
		*lwis_tx = NULL;
		kfree(transaction);
		return;
	}

	if (!list_empty(&transaction->completion_fence_list)) {
		list_for_each_safe(it_fence, it_fence_tmp, &transaction->completion_fence_list) {
			pending_fence =
				list_entry(it_fence, struct lwis_fence_pending_signal, node);
			list_del(&pending_fence->node);
			dma_fence_put(pending_fence->fence);
			kfree(pending_fence);
		}
	}

	list_for_each_entry_safe(pend_id, pend_id_tmp, &transaction->trigger_fences, node) {
		list_del(&pend_id->node);
		if (!pend_id->triggered)
			dma_fence_remove_callback(pend_id->fence, &pend_id->fence_cb);
		dma_fence_put(pend_id->fence);
		kfree(pend_id);
	}

	for (i = 0; i < transaction->info.num_io_entries; ++i) {
		if (transaction->info.io_entries[i].type == LWIS_IO_ENTRY_WRITE_BATCH ||
		    transaction->info.io_entries[i].type == LWIS_IO_ENTRY_WRITE_BATCH_V2) {
			lwis_allocator_free(lwis_dev, transaction->info.io_entries[i].rw_batch.buf);
			transaction->info.io_entries[i].rw_batch.buf = NULL;
		} else if (transaction->info.io_entries[i].type == LWIS_IO_ENTRY_WRITE_TO_BUFFER) {
			struct dma_buf *buf =
				transaction->info.io_entries[i].write_to_buffer.buffer->dma_buf;
			if (IS_ERR(buf)) {
				dev_err(lwis_dev->dev,
					"Failed finish PDMA buffer IO because buffer has error.");
			}
			/* Unmap the DMA buffer from kernel space */
			dma_buf_vunmap(
				buf,
				transaction->info.io_entries[i].write_to_buffer.buffer->io_sys_map);
			kfree(transaction->info.io_entries[i].write_to_buffer.buffer->io_sys_map);
			kfree(transaction->info.io_entries[i].write_to_buffer.bytes);
			dma_buf_end_cpu_access(buf, DMA_BIDIRECTIONAL);
			dma_buf_put(buf);
			kfree(transaction->info.io_entries[i].write_to_buffer.buffer);
		}
	}
	lwis_allocator_free(lwis_dev, transaction->info.io_entries);

	transaction->starting_read_buf = NULL;

	kfree(transaction->resp);
	*lwis_tx = NULL;
	kfree(transaction);
}

static int process_transaction(struct lwis_client *client, struct lwis_transaction **lwis_tx,
			       struct list_head *pending_events, struct list_head *pending_fences,
			       bool skip_err, bool check_transaction_limit, bool run_in_irq_context)
{
	int i;
	int ret = 0;
	struct lwis_io_entry *entry = NULL;
	struct lwis_device *lwis_dev = client->lwis_dev;
	struct lwis_transaction *transaction = *lwis_tx;
	struct lwis_transaction_info *info = &transaction->info;
	struct lwis_transaction_response_header *resp = transaction->resp;
	size_t resp_size;
	uint8_t *read_buf;
	struct lwis_io_result *io_result;
	const int reg_value_bytewidth = lwis_dev->native_value_bitwidth / 8;
	int64_t process_duration_ns = -1;
	int64_t process_timestamp = -1;
	int64_t triggered_duration_ns = -1;
	int64_t output_event = LWIS_EVENT_ID_NONE;
	unsigned long flags;

	const int total_entries = info->num_io_entries;
	int max_limit = lwis_dev->transaction_process_limit;
	int remaining_entries = transaction->remaining_entries_to_process;
	int current_run_entries;
	int start_idx;
	int end_idx;

	/*
	 * Process all the transactions at once if:
	 * 1. the processing device has no limitation on the number of entries
	 * to process per transaction.
	 * 2. the transaction is running in event context.
	 * 3. the transaction is being called as a part of cleanup.
	 * Note: For #2 and #3, this transaction will not be queued for further
	 * processing by any worker thread later and therefore all entries need
	 * to be processed in the same run.
	 */
	if ((lwis_dev->transaction_process_limit <= 0) ||
	    (transaction->info.run_in_event_context || skip_err || !check_transaction_limit)) {
		max_limit = total_entries;
	}

	current_run_entries = (remaining_entries > max_limit) ? max_limit : remaining_entries;
	start_idx = total_entries - remaining_entries;
	end_idx = start_idx + current_run_entries;
	remaining_entries = remaining_entries - current_run_entries;

	if (lwis_transaction_debug)
		process_timestamp = ktime_to_ns(lwis_get_time());

	if (transaction->info.trigger_event_id != LWIS_EVENT_ID_NONE &&
	    transaction->triggered_event_timestamp != 0) {
		triggered_duration_ns =
			ktime_to_ns(lwis_get_time()) - transaction->triggered_event_timestamp;
		if (ktime_to_ms(triggered_duration_ns) >= TRIGGRED_EVENT_EXECUTION_THRESHOLD_MS)
			dev_warn(client->lwis_dev->dev,
				 "Triggered event id %lld transaction id %llu, %lldms",
				 transaction->info.trigger_event_id, transaction->info.id,
				 ktime_to_ms(triggered_duration_ns));
	}

	resp_size = sizeof(struct lwis_transaction_response_header) + resp->results_size_bytes;
	read_buf = (uint8_t *)resp + sizeof(struct lwis_transaction_response_header);

	resp->completion_index = -1;

	/*
	 * If the starting read buffer pointer is not null then
	 * use this cached location to correctly
	 * set the read buffer for the current transaction processing run.
	 */
	if (transaction->starting_read_buf)
		read_buf = transaction->starting_read_buf;

	/*
	 * Use write memory barrier at the beginning of I/O entries if the access protocol
	 * allows it.
	 */
	if (lwis_dev->vops.register_io_barrier != NULL) {
		lwis_dev->vops.register_io_barrier(lwis_dev,
						   /*use_read_barrier=*/false,
						   /*use_write_barrier=*/true);
	}

	/*
	 * Some devices like IOREG devices can process the transaction in IRQ context.
	 * In these cases, do not schedule the bus/thread manager for further optimization.
	 */
	if (!run_in_irq_context)
		lwis_bus_manager_lock_bus(lwis_dev);

	for (i = start_idx; i < end_idx; i++) {
		entry = &info->io_entries[i];
		if (entry->type == LWIS_IO_ENTRY_WRITE || entry->type == LWIS_IO_ENTRY_WRITE_V2 ||
		    entry->type == LWIS_IO_ENTRY_WRITE_BATCH ||
		    entry->type == LWIS_IO_ENTRY_WRITE_BATCH_V2 ||
		    entry->type == LWIS_IO_ENTRY_MODIFY) {
			ret = lwis_dev->vops.register_io(lwis_dev, entry,
							 lwis_dev->native_value_bitwidth);
			if (ret) {
				resp->error_code = ret;
				if (skip_err) {
					dev_warn(
						lwis_dev->dev,
						"transaction type %d processing failed, skip this error and run the next command\n",
						entry->type);
					continue;
				}
				break;
			}
		} else if (entry->type == LWIS_IO_ENTRY_READ ||
			   entry->type == LWIS_IO_ENTRY_READ_V2) {
			io_result = (struct lwis_io_result *)read_buf;
			io_result->bid = entry->rw.bid;
			io_result->offset = entry->rw.offset;
			io_result->num_value_bytes = reg_value_bytewidth;
			ret = lwis_dev->vops.register_io(lwis_dev, entry,
							 lwis_dev->native_value_bitwidth);
			if (ret) {
				resp->error_code = ret;
				if (skip_err) {
					dev_warn(
						lwis_dev->dev,
						"transaction type %d processing failed, skip this error and run the next command\n",
						entry->type);
					continue;
				}
				break;
			}
			memcpy(io_result->values, &entry->rw.val, reg_value_bytewidth);
			read_buf += sizeof(struct lwis_io_result) + io_result->num_value_bytes;
		} else if (entry->type == LWIS_IO_ENTRY_READ_BATCH ||
			   entry->type == LWIS_IO_ENTRY_READ_BATCH_V2) {
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
					dev_warn(
						lwis_dev->dev,
						"transaction type %d processing failed, skip this error and run the next command\n",
						entry->type);
					continue;
				}
				break;
			}
			read_buf += sizeof(struct lwis_io_result) + io_result->num_value_bytes;
		} else if (entry->type == LWIS_IO_ENTRY_POLL) {
			ret = lwis_io_entry_poll(lwis_dev, entry, /*is_short=*/false);
			if (ret) {
				resp->error_code = ret;
				if (skip_err) {
					dev_warn(
						lwis_dev->dev,
						"transaction type %d processing failed, skip this error and run the next command\n",
						entry->type);
					continue;
				}
				break;
			}
		} else if (entry->type == LWIS_IO_ENTRY_POLL_SHORT) {
			ret = lwis_io_entry_poll(lwis_dev, entry, /*is_short=*/true);
			if (ret) {
				resp->error_code = ret;
				if (skip_err) {
					dev_warn(
						lwis_dev->dev,
						"transaction type %d processing failed, skip this error and run the next command\n",
						entry->type);
					continue;
				}
				break;
			}
		} else if (entry->type == LWIS_IO_ENTRY_WAIT) {
			ret = lwis_io_entry_wait(lwis_dev, entry);
			if (ret) {
				resp->error_code = ret;
				if (skip_err) {
					dev_warn(
						lwis_dev->dev,
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
					dev_warn(
						lwis_dev->dev,
						"transaction type %d processing failed, skip this error and run the next command\n",
						entry->type);
					continue;
				}
				break;
			}
		} else if (entry->type == LWIS_IO_ENTRY_WRITE_TO_BUFFER) {
			ret = lwis_io_buffer_write(lwis_dev, entry);
			if (ret) {
				resp->error_code = ret;
				if (skip_err) {
					dev_warn(
						lwis_dev->dev,
						"transaction type %d processing failed, skip this error and run the next command\n",
						entry->type);
					continue;
				}
				break;
			}
		} else if (entry->type == LWIS_IO_ENTRY_IGNORE) {
			ret = 0;
		} else {
			dev_err(lwis_dev->dev, "Unrecognized io_entry command\n");
			resp->error_code = -EINVAL;
			if (skip_err) {
				dev_warn(
					lwis_dev->dev,
					"transaction type %d processing failed, skip this error and run the next command\n",
					entry->type);
				continue;
			}
			break;
		}
		resp->completion_index = i;
	}

	if (!run_in_irq_context)
		lwis_bus_manager_unlock_bus(lwis_dev);

	if (lwis_transaction_debug)
		process_duration_ns = ktime_to_ns(lwis_get_time() - process_timestamp);

	/*
	 * Use read memory barrier at the end of I/O entries if the access protocol
	 * allows it.
	 */
	if (lwis_dev->vops.register_io_barrier != NULL) {
		lwis_dev->vops.register_io_barrier(lwis_dev, /*use_read_barrier=*/true,
						   /*use_write_barrier=*/false);
	}

	if (remaining_entries > 0 && ret == 0) {
		/*
		 * If there are remaining entries to be processed in this transaction,
		 * don't delete this transaction and update the current remaining
		 * count of entries in the transaction. Stop processing further
		 * until there are no more remaining entries to be processed
		 * in the transaction.
		 */
		spin_lock_irqsave(&client->transaction_lock, flags);
		transaction->starting_read_buf = read_buf;
		transaction->remaining_entries_to_process = remaining_entries;
		spin_unlock_irqrestore(&client->transaction_lock, flags);
		return ret;
	}

	if (pending_events) {
		output_event =
			resp->error_code ? info->emit_error_event_id : info->emit_success_event_id;
		/* Check to make sure an output event has been specified. */
		if (output_event != LWIS_EVENT_ID_NONE) {
			lwis_pending_event_push(pending_events, output_event, (void *)resp,
						resp_size);
		}
	} else {
		/* No pending events indicates it's cleanup io_entries. */
		if (entry && resp->error_code) {
			dev_err(lwis_dev->dev,
				"Clean-up fails with error code %d, transaction %llu, io_entries[%d], entry_type %d",
				resp->error_code, transaction->info.id, i, entry->type);
		}
	}

	spin_lock_irqsave(&client->transaction_lock, flags);
	transaction->remaining_entries_to_process = remaining_entries;

	if (pending_fences) {
		lwis_pending_fences_move_all(lwis_dev, transaction, pending_fences,
					     resp->error_code);
	}
	save_transaction_to_history(client, info, process_timestamp, process_duration_ns);

	/*
	 * This check needs to be handled only for cases where we are processing
	 * the transaction based on the limit specified in the dts.
	 * When the transactions are cancelled or executed in event context
	 * the limit doesn't dictate the number of entries that will be processed.
	 */
	if (check_transaction_limit) {
		/*
		 *  1. If all of the entries are processed for a given transaction then
		 *     delete the transaction from the queue and enable emit signals for
		 *     pending events and fences.
		 *  2. Delete transaction from the process queue after the limit is fulfilled
		 *     or there is an error while processing.
		 */
		list_del(&transaction->process_queue_node);
	}

	if (info->trigger_event_counter == LWIS_EVENT_COUNTER_EVERY_TIME) {
		/*
		 * Only clean the transaction struct for this iteration. The
		 * I/O entries are not being freed.
		 */
		kfree(transaction->resp);
		*lwis_tx = NULL;
		kfree(transaction);
	} else {
		lwis_transaction_free(lwis_dev, lwis_tx);
	}
	spin_unlock_irqrestore(&client->transaction_lock, flags);

	return ret;
}

static void cancel_transaction(struct lwis_client *client, struct lwis_transaction **lwis_tx,
			       int error_code, struct list_head *pending_events,
			       struct list_head *pending_fences, bool delete_pending_map_node)
{
	struct lwis_device *lwis_dev = client->lwis_dev;
	struct lwis_transaction *transaction = *lwis_tx;
	struct lwis_transaction_info *info = &transaction->info;
	struct lwis_transaction_response_header resp;

	resp.id = info->id;
	resp.error_code = error_code;
	resp.num_entries = 0;
	resp.results_size_bytes = 0;
	resp.completion_index = -1;

	if (transaction->is_weak_transaction) {
		lwis_transaction_free(lwis_dev, lwis_tx);
		return;
	}

	if (pending_events) {
		/* Check to make sure an error event has been specified. */
		if (info->emit_error_event_id != LWIS_EVENT_ID_NONE) {
			lwis_pending_event_push(pending_events, info->emit_error_event_id, &resp,
						sizeof(resp));
		}
	}
	if (pending_fences)
		lwis_pending_fences_move_all(lwis_dev, transaction, pending_fences, error_code);

	if (delete_pending_map_node)
		hash_del(&transaction->pending_map_node);

	lwis_transaction_free(lwis_dev, lwis_tx);
}

static bool process_broken_transaction(struct lwis_client *client, struct list_head *pending_events,
				       struct list_head *pending_fences,
				       struct lwis_transaction *transaction)
{
	unsigned long flush_flags;
	struct device *dev = client->lwis_dev->dev;

	/*
	 * Continue the loop if the transaction is complete and deleted or
	 * if the transaction exists but all the entries are processed
	 */
	if (transaction && transaction->remaining_entries_to_process > 0) {
		/*
		 * If the transaction exists and there are entries
		 * remaning to be processed, that would indicate the
		 * transaction processing limit has reached for this
		 * device and we stop processing its queue further.
		 */
		if (lwis_transaction_debug) {
			dev_info(
				dev,
				"Transaction processing limit reached, remaining entries to process %d\n",
				transaction->remaining_entries_to_process);
		}

		/*
		 * Queue the remaining transaction again on the transaction
		 * worker/bus manager worker to be processed again later if
		 * the client is not flushing.
		 * If the client is flushing, cancel the remaining transaction
		 * and delete from the process queue node.
		 */
		spin_lock_irqsave(&client->flush_lock, flush_flags);
		if (client->flush_state == NOT_FLUSHING) {
			if (lwis_transaction_debug)
				dev_info(dev, "Client: NOT_FLUSHING, schedule remaining work");

			lwis_queue_device_worker(client);
		} else {
			if (lwis_transaction_debug)
				dev_info(dev, "Client: FLUSHING, abort remaining transaction");

			list_del(&transaction->process_queue_node);
			cancel_transaction(client, &transaction, transaction->resp->error_code,
					   pending_events, pending_fences, false);
		}
		spin_unlock_irqrestore(&client->flush_lock, flush_flags);
		return true;
	}
	return false;
}

void lwis_process_transactions_in_queue(struct lwis_client *client,
					bool process_high_priority_transaction)
{
	unsigned long flags;
	struct list_head *it_tran, *it_tran_tmp;
	struct list_head pending_events;
	struct list_head pending_fences;
	struct lwis_transaction *transaction;
	bool check_transaction_limit = true;

	INIT_LIST_HEAD(&pending_events);
	INIT_LIST_HEAD(&pending_fences);

	spin_lock_irqsave(&client->transaction_lock, flags);
	list_for_each_safe(it_tran, it_tran_tmp, &client->transaction_process_queue) {
		if (!client->is_enabled && client->lwis_dev->type != DEVICE_TYPE_TOP) {
			if (lwis_transaction_debug) {
				dev_info(
					client->lwis_dev->dev,
					"Client is disabled. Please enable client to process transactions.");
			}
			spin_unlock_irqrestore(&client->transaction_lock, flags);
			return;
		}

		transaction = list_entry(it_tran, struct lwis_transaction, process_queue_node);

		/*
		 * For all high priority transactions:
		 * 1. Do not process normal priority transactions if
		 * process_high_priority_transaction is set.
		 * Instead, continue looping through the client's queue and
		 * find only high priority transactions to process.
		 * 2. For high priority transactions, do not apply any
		 * transaction limit and process entire transaction in one go.
		 */
		if (process_high_priority_transaction) {
			if (!transaction->info.is_high_priority_transaction) {
				continue;
			} else {
				check_transaction_limit = false;
				if (lwis_transaction_debug) {
					dev_info(client->lwis_dev->dev,
						 "Start processing high priority transaction.");
				}
			}
		}

		if (transaction->resp->error_code) {
			list_del(&transaction->process_queue_node);
			cancel_transaction(client, &transaction, transaction->resp->error_code,
					   &pending_events, &pending_fences, false);
		} else {
			spin_unlock_irqrestore(&client->transaction_lock, flags);
			process_transaction(client, &transaction, &pending_events, &pending_fences,
					    /*skip_err=*/false, check_transaction_limit,
					    /*run_in_irq_context=*/false);

			/*
			 * Delete the high priority transaction from the queue
			 * once transaction is complete.
			 */
			if (!check_transaction_limit) {
				spin_lock_irqsave(&client->transaction_lock, flags);
				list_del(&transaction->process_queue_node);
				spin_unlock_irqrestore(&client->transaction_lock, flags);
				if (lwis_transaction_debug) {
					dev_info(client->lwis_dev->dev,
						 "Finished processing high priority transaction.");
				}
			}

			lwis_pending_events_emit(client->lwis_dev, &pending_events);
			spin_lock_irqsave(&client->transaction_lock, flags);

			/*
			 * If LWIS is processing a broken transaction,
			 * then it needs to stop processing the client's transaction queue further
			 * until the broken transaction is completely processed.
			 */
			if (process_broken_transaction(client, &pending_events, &pending_fences,
						       transaction)) {
				break;
			}
		}
	}
	spin_unlock_irqrestore(&client->transaction_lock, flags);
	lwis_pending_events_emit(client->lwis_dev, &pending_events);
	lwis_fences_pending_signal_emit(client->lwis_dev, &pending_fences);
}

int lwis_transaction_init(struct lwis_client *client)
{
	spin_lock_init(&client->transaction_lock);
	INIT_LIST_HEAD(&client->transaction_process_queue);
	client->transaction_counter = 0;
	hash_init(client->transaction_list);
	hash_init(client->pending_transactions);
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
		list_for_each_safe(it_tran, it_tran_tmp, transaction_queue) {
			transaction =
				list_entry(it_tran, struct lwis_transaction, process_queue_node);
			list_del(&transaction->process_queue_node);
			cancel_transaction(client, &transaction, -ECANCELED, NULL, NULL, false);
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
	hash_for_each_safe(client->transaction_list, i, tmp, it_evt_list, node) {
		if ((it_evt_list->event_id & 0xFFFF0000FFFFFFFFll) ==
		    LWIS_EVENT_ID_CLIENT_CLEANUP) {
			continue;
		}
		list_for_each_safe(it_tran, it_tran_tmp, &it_evt_list->list) {
			transaction = list_entry(it_tran, struct lwis_transaction, event_list_node);
			list_del(&transaction->event_list_node);
			cancel_transaction(client, &transaction, -ECANCELED, NULL, NULL, false);
		}
		hash_del(&it_evt_list->node);
		kfree(it_evt_list);
	}
	hash_for_each_safe(client->pending_transactions, i, tmp, transaction, pending_map_node) {
		cancel_transaction(client, &transaction, -ECANCELED, NULL, NULL, true);
	}
	spin_unlock_irqrestore(&client->transaction_lock, flags);

	spin_lock_irqsave(&client->flush_lock, flags);
	client->flush_state = FLUSHING;
	spin_unlock_irqrestore(&client->flush_lock, flags);

	lwis_flush_device_worker(client);

	spin_lock_irqsave(&client->flush_lock, flags);
	client->flush_state = NOT_FLUSHING;
	spin_unlock_irqrestore(&client->flush_lock, flags);

	spin_lock_irqsave(&client->transaction_lock, flags);
	/*
	 * The transaction queue should be empty after canceling all transactions,
	 * but check anyway.
	 */
	cancel_all_transactions_in_queue_locked(client, &client->transaction_process_queue);
	spin_unlock_irqrestore(&client->transaction_lock, flags);

	return 0;
}

int lwis_transaction_client_cleanup(struct lwis_client *client)
{
	unsigned long flags;
	struct list_head *it_tran, *it_tran_tmp;
	struct lwis_transaction *transaction;
	struct lwis_transaction_event_list *it_evt_list;

	spin_lock_irqsave(&client->transaction_lock, flags);
	it_evt_list = event_list_find(client, LWIS_EVENT_ID_CLIENT_CLEANUP |
						      (int64_t)client->lwis_dev->id
							      << LWIS_EVENT_ID_EVENT_CODE_LEN);
	if (it_evt_list == NULL) {
		spin_unlock_irqrestore(&client->transaction_lock, flags);
		return 0;
	}

	list_for_each_safe(it_tran, it_tran_tmp, &it_evt_list->list) {
		transaction = list_entry(it_tran, struct lwis_transaction, event_list_node);
		if (!list_empty(&transaction->completion_fence_list)) {
			dev_warn(
				client->lwis_dev->dev,
				"Cleanup transaction with id %llu has tailing fences; cleanup transactions should not have tailing fences",
				transaction->info.id);
		}
		list_del(&transaction->event_list_node);
		if (transaction->resp->error_code || client->lwis_dev->enabled == 0) {
			cancel_transaction(client, &transaction, -ECANCELED, NULL, NULL, false);
		} else {
			spin_unlock_irqrestore(&client->transaction_lock, flags);
			process_transaction(client, &transaction,
					    /*pending_events=*/NULL,
					    /*pending_fences=*/NULL,
					    /*skip_err=*/true,
					    /*check_transaction_limit=*/false,
					    /*run_in_irq_context=*/false);
			spin_lock_irqsave(&client->transaction_lock, flags);
		}
	}
	hash_del(&it_evt_list->node);
	kfree(it_evt_list);

	spin_unlock_irqrestore(&client->transaction_lock, flags);
	return 0;
}

int lwis_trigger_event_add_weak_transaction(struct lwis_client *client, int64_t transaction_id,
					    int64_t event_id, int32_t precondition_fence_fd)
{
	struct lwis_transaction *weak_transaction;
	struct lwis_transaction_event_list *event_list;

	weak_transaction = kmalloc(sizeof(struct lwis_transaction), GFP_ATOMIC);
	if (!weak_transaction)
		return -ENOMEM;

	weak_transaction->is_weak_transaction = true;
	weak_transaction->id = transaction_id;
	weak_transaction->precondition_fence = NULL;
	if (precondition_fence_fd >= 0) {
		struct dma_fence *fence = lwis_dma_fence_get(precondition_fence_fd);

		if (IS_ERR_OR_NULL(fence)) {
			dev_err(client->lwis_dev->dev, "Unable to get fence with fd=%d",
				precondition_fence_fd);
			return -EBADF;
		}
		weak_transaction->precondition_fence = fence;
	}

	event_list = event_list_find_or_create(client, event_id);
	if (!event_list) {
		dev_err(client->lwis_dev->dev, "Cannot create transaction event list\n");
		return -EINVAL;
	}
	list_add_tail(&weak_transaction->event_list_node, &event_list->list);
	lwis_debug_dev_info(
		client->lwis_dev->dev,
		"lwis_fence add weak transaction for event id-%lld triggered transaction id %llu",
		event_id, transaction_id);
	return 0;
}

static int check_transaction_param_locked(struct lwis_client *client,
					  struct lwis_transaction *transaction,
					  bool is_level_triggered)
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

	info->id = client->transaction_counter++;

	info->current_trigger_event_counter = -1LL;

	/* Look for the trigger event state, if specified. */
	if (info->trigger_event_id != LWIS_EVENT_ID_NONE) {
		event_state = lwis_device_event_state_find(lwis_dev, info->trigger_event_id);
		if (event_state == NULL) {
			/* Event has not been encountered, setting event counter to zero. */
			info->current_trigger_event_counter = 0;
		} else {
			/* Event found, return current counter to userspace. */
			info->current_trigger_event_counter = event_state->event_counter;
		}
	} else if (!lwis_triggered_by_condition(transaction)) {
		/* Otherwise it's an immediate transaction. */
		transaction->queue_immediately = true;
	}

	/* Both trigger event ID and counter are defined. */
	if (info->trigger_event_id != LWIS_EVENT_ID_NONE &&
	    EXPLICIT_EVENT_COUNTER(info->trigger_event_counter)) {
		/* Check if event has happened already. */
		if (info->trigger_event_counter == info->current_trigger_event_counter) {
			if (is_level_triggered) {
				/* Convert this transaction into an immediate one. */
				transaction->queue_immediately = true;
			} else {
				return -ENOENT;
			}
		} else if (info->trigger_event_counter < info->current_trigger_event_counter) {
			return -ENOENT;
		}
	}

	/* Make sure either an output success/error event OR a completion fence is specified */
	if ((info->emit_success_event_id == LWIS_EVENT_ID_NONE ||
	     info->emit_error_event_id == LWIS_EVENT_ID_NONE) &&
	    info->create_completion_fence_fd == LWIS_NO_COMPLETION_FENCE &&
	    info->num_completion_fences == 0) {
		dev_err(lwis_dev->dev,
			"No transaction events or completion fence specified for transaction");
		return -EINVAL;
	}

	/* Make sure sw events exist in event table. */
	if (info->emit_success_event_id != LWIS_EVENT_ID_NONE) {
		if (IS_ERR_OR_NULL(lwis_device_event_state_find_or_create(
			    lwis_dev, info->emit_success_event_id)) ||
		    IS_ERR_OR_NULL(lwis_client_event_state_find_or_create(
			    client, info->emit_success_event_id))) {
			dev_err(lwis_dev->dev, "Cannot create success event for transaction");
			return -EINVAL;
		}
	}
	if (info->emit_error_event_id != LWIS_EVENT_ID_NONE) {
		if (IS_ERR_OR_NULL(lwis_device_event_state_find_or_create(
			    lwis_dev, info->emit_error_event_id)) ||
		    IS_ERR_OR_NULL(lwis_client_event_state_find_or_create(
			    client, info->emit_error_event_id))) {
			dev_err(lwis_dev->dev, "Cannot create error event for transaction");
			return -EINVAL;
		}
	}

	return 0;
}

static int prepare_transaction_fences_locked(struct lwis_client *client,
					     struct lwis_transaction *transaction)
{
	int ret = 0;

	if (lwis_triggered_by_condition(transaction)) {
		ret = lwis_parse_trigger_condition(client, transaction);
		if (ret)
			return ret;
	}

	/* If transaction contains completion fences, add them to the transaction. */
	ret = lwis_add_completion_fences_to_transaction(client, transaction);

	return ret;
}

static int prepare_response_locked(struct lwis_client *client, struct lwis_transaction *transaction)
{
	struct lwis_transaction_info *info = &transaction->info;
	int i;
	size_t resp_size;
	size_t read_buf_size = 0;
	int read_entries = 0;
	const int reg_value_bytewidth = client->lwis_dev->native_value_bitwidth / 8;

	for (i = 0; i < info->num_io_entries; ++i) {
		struct lwis_io_entry *entry = &info->io_entries[i];

		if (entry->type == LWIS_IO_ENTRY_READ || entry->type == LWIS_IO_ENTRY_READ_V2) {
			read_buf_size += reg_value_bytewidth;
			read_entries++;
		} else if (entry->type == LWIS_IO_ENTRY_READ_BATCH ||
			   entry->type == LWIS_IO_ENTRY_READ_BATCH_V2) {
			read_buf_size += entry->rw_batch.size_in_bytes;
			read_entries++;
		}
	}

	/* Event response payload consists of header, and address and offset pairs. */
	resp_size = sizeof(struct lwis_transaction_response_header) +
		    read_entries * sizeof(struct lwis_io_result) + read_buf_size;
	/*
	 * Revisit the use of GFP_ATOMIC here. Reason for this to be atomic is
	 * because this function can be called by transaction_replace while
	 * holding onto a spinlock.
	 */
	transaction->resp = kmalloc(resp_size, GFP_ATOMIC);
	if (!transaction->resp)
		return -ENOMEM;

	transaction->resp->id = info->id;
	transaction->resp->error_code = 0;
	transaction->resp->completion_index = 0;
	transaction->resp->num_entries = read_entries;
	transaction->resp->results_size_bytes =
		read_entries * sizeof(struct lwis_io_result) + read_buf_size;
	return 0;
}

static int add_transaction_to_queue_locked(struct lwis_client *client,
					   struct lwis_transaction *transaction)
{
	int ret;
	struct lwis_transaction_info *info = &transaction->info;

	if (info->is_high_priority_transaction) {
		list_add(&transaction->process_queue_node, &client->transaction_process_queue);
		ret = lwis_bus_manager_add_high_priority_client(client);
		if (ret) {
			dev_err(client->lwis_dev->dev, "Failed to add high priority transaction");
			return ret;
		}
	} else {
		list_add_tail(&transaction->process_queue_node, &client->transaction_process_queue);
	}
	return 0;
}

/* Calling this function requires holding the client's transaction_lock. */
static int queue_transaction_locked(struct lwis_client *client,
				    struct lwis_transaction *transaction)
{
	struct lwis_transaction_event_list *event_list;
	struct lwis_transaction_info *info = &transaction->info;

	int ret;

	if (transaction->queue_immediately) {
		ret = add_transaction_to_queue_locked(client, transaction);
		if (ret) {
			kfree(transaction->resp);
			return ret;
		}
		lwis_queue_device_worker(client);
	} else if (lwis_triggered_by_condition(transaction)) {
		add_pending_transaction(client, transaction);
	} else {
		event_list = event_list_find_or_create(client, info->trigger_event_id);
		if (!event_list) {
			dev_err(client->lwis_dev->dev, "Cannot create transaction event list\n");
			kfree(transaction->resp);
			return -EINVAL;
		}
		list_add_tail(&transaction->event_list_node, &event_list->list);
	}
	info->submission_timestamp_ns = ktime_to_ns(ktime_get());
	return 0;
}

int lwis_transaction_submit_locked(struct lwis_client *client, struct lwis_transaction *transaction)
{
	int ret;
	struct lwis_transaction_info *info = &transaction->info;

	ret = check_transaction_param_locked(client, transaction,
					     /*is_level_triggered=*/info->is_level_triggered);
	if (ret)
		return ret;

	ret = prepare_response_locked(client, transaction);
	if (ret)
		return ret;

	ret = prepare_transaction_fences_locked(client, transaction);
	if (ret)
		return ret;

	ret = queue_transaction_locked(client, transaction);
	return ret;
}

static struct lwis_transaction *
new_repeating_transaction_iteration(struct lwis_client *client,
				    struct lwis_transaction *transaction)
{
	struct lwis_transaction *new_instance;
	uint8_t *resp_buf;

	new_instance = kmalloc(sizeof(struct lwis_transaction), GFP_ATOMIC);
	if (!new_instance)
		return NULL;

	memcpy(&new_instance->info, &transaction->info, sizeof(transaction->info));

	resp_buf = kmalloc(sizeof(struct lwis_transaction_response_header) +
				   transaction->resp->results_size_bytes,
			   GFP_ATOMIC);
	if (!resp_buf) {
		kfree(new_instance);
		return NULL;
	}
	memcpy(resp_buf, transaction->resp, sizeof(struct lwis_transaction_response_header));
	new_instance->resp = (struct lwis_transaction_response_header *)resp_buf;

	new_instance->is_weak_transaction = transaction->is_weak_transaction;
	new_instance->remaining_entries_to_process = transaction->info.num_io_entries;
	new_instance->starting_read_buf = NULL;

	INIT_LIST_HEAD(&new_instance->event_list_node);
	INIT_LIST_HEAD(&new_instance->process_queue_node);
	INIT_LIST_HEAD(&new_instance->trigger_fences);
	INIT_LIST_HEAD(&new_instance->completion_fence_list);

	return new_instance;
}

static void defer_transaction_locked(struct lwis_client *client,
				     struct lwis_transaction *transaction,
				     struct list_head *pending_events,
				     struct list_head *pending_fences, bool del_event_list_node,
				     unsigned long *flags)
{
	if (del_event_list_node)
		list_del(&transaction->event_list_node);

	/* I2C read/write cannot be executed in IRQ context */
	if (in_hardirq() && client->lwis_dev->type == DEVICE_TYPE_I2C) {
		add_transaction_to_queue_locked(client, transaction);
		return;
	}

	if (transaction->info.run_in_event_context) {
		spin_unlock_irqrestore(&client->transaction_lock, *flags);
		process_transaction(client, &transaction, pending_events, pending_fences,
				    /*skip_err=*/false, /*check_transaction_limit=*/false,
				    /*run_in_irq_context=*/true);
		spin_lock_irqsave(&client->transaction_lock, *flags);
	} else {
		add_transaction_to_queue_locked(client, transaction);
	}
}

int lwis_transaction_event_trigger(struct lwis_client *client, int64_t event_id,
				   int64_t event_counter, int64_t event_timestamp,
				   struct list_head *pending_events)
{
	unsigned long flags;
	struct lwis_transaction_event_list *event_list;
	struct list_head *it_tran, *it_tran_tmp;
	struct lwis_transaction *transaction = NULL;
	struct lwis_transaction *weak_transaction = NULL;
	struct lwis_transaction *new_instance;
	int64_t trigger_counter = 0;
	struct list_head pending_fences;
	int ret;

	INIT_LIST_HEAD(&pending_fences);

	/* Find event list that matches the trigger event ID. */
	spin_lock_irqsave(&client->transaction_lock, flags);
	if (event_id & LWIS_OVERFLOW_IRQ_EVENT_FLAG)
		event_id = event_id ^ LWIS_OVERFLOW_IRQ_EVENT_FLAG;

	event_list = event_list_find(client, event_id);
	if (event_list == NULL || list_empty(&event_list->list)) {
		spin_unlock_irqrestore(&client->transaction_lock, flags);
		return 0;
	}

	list_for_each_safe(it_tran, it_tran_tmp, &event_list->list) {
		transaction = list_entry(it_tran, struct lwis_transaction, event_list_node);
		/* The trigger event of the transaction happens */
		transaction->triggered_event_timestamp = event_timestamp;
		if (transaction->is_weak_transaction) {
			weak_transaction = transaction;
			transaction = pending_transaction_peek(client, weak_transaction->id);
			if (transaction == NULL) {
				/* It means the transaction is already executed or is canceled. */
				list_del(&weak_transaction->event_list_node);
				kfree(weak_transaction);
				continue;
			}

			if (lwis_event_triggered_condition_ready(transaction, weak_transaction,
								 event_id, event_counter)) {
				lwis_debug_dev_info(
					client->lwis_dev->dev,
					"lwis_fence event id-%lld counter-%lld triggered transaction id %llu",
					event_id, event_counter, transaction->info.id);
				hash_del(&transaction->pending_map_node);
				defer_transaction_locked(client, transaction, pending_events,
							 &pending_fences,
							 /* del_event_list_node */ false, &flags);
			}
			continue;
		}

		if (transaction->resp->error_code) {
			ret = add_transaction_to_queue_locked(client, transaction);
			if (ret) {
				spin_unlock_irqrestore(&client->transaction_lock, flags);
				return ret;
			}

			list_del(&transaction->event_list_node);
			continue;
		}

		/*
		 * Compare current event with trigger event counter to make
		 * sure this transaction needs to be executed now.
		 */
		trigger_counter = transaction->info.trigger_event_counter;
		if (trigger_counter == LWIS_EVENT_COUNTER_ON_NEXT_OCCURRENCE ||
		    trigger_counter == event_counter) {
			defer_transaction_locked(client, transaction, pending_events,
						 &pending_fences, /* del_event_list_node */ true,
						 &flags);
		} else if (trigger_counter == LWIS_EVENT_COUNTER_EVERY_TIME) {
			new_instance = new_repeating_transaction_iteration(client, transaction);
			if (!new_instance) {
				transaction->resp->error_code = -ENOMEM;
				ret = add_transaction_to_queue_locked(client, transaction);
				if (ret) {
					spin_unlock_irqrestore(&client->transaction_lock, flags);
					return ret;
				}
				list_del(&transaction->event_list_node);
				continue;
			}
			defer_transaction_locked(client, new_instance, pending_events,
						 &pending_fences, /* del_event_list_node */ false,
						 &flags);
		}
	}

	spin_unlock_irqrestore(&client->transaction_lock, flags);

	if (!list_empty(&client->transaction_process_queue))
		lwis_queue_device_worker(client);

	lwis_fences_pending_signal_emit(client->lwis_dev, &pending_fences);

	return 0;
}

void lwis_transaction_fence_trigger(struct lwis_client *client, struct dma_fence *fence,
				    int64_t transaction_id)
{
	unsigned long flags = 0;
	struct lwis_transaction *transaction;
	int ret;
	LIST_HEAD(pending_events);
	LIST_HEAD(pending_fences);

	spin_lock_irqsave(&client->transaction_lock, flags);
	transaction = pending_transaction_peek(client, transaction_id);
	if (transaction == NULL) {
		/* It means the transaction is already executed or is canceled. */
		lwis_debug_dev_info(
			client->lwis_dev->dev,
			"dma_fence %p did NOT triggered transaction id %llu, seems already triggered",
			fence, transaction_id);
	} else {
		int fence_status = dma_fence_get_status_locked(fence);

		if (lwis_fence_triggered_condition_ready(transaction, fence_status)) {
			hash_del(&transaction->pending_map_node);
			if (fence_status == LWIS_FENCE_STATUS_SUCCESSFULLY_SIGNALED) {
				ret = add_transaction_to_queue_locked(client, transaction);
				if (ret) {
					spin_unlock_irqrestore(&client->transaction_lock, flags);
					return;
				}
				lwis_debug_dev_info(client->lwis_dev->dev,
						    "dma_fence %p triggered transaction id %llu",
						    fence, transaction->info.id);
			} else {
				cancel_transaction(client, &transaction, -ECANCELED,
						   &pending_events, &pending_fences, false);
			}
		}
	}
	spin_unlock_irqrestore(&client->transaction_lock, flags);

	if (!list_empty(&client->transaction_process_queue))
		lwis_queue_device_worker(client);

	lwis_pending_events_emit(client->lwis_dev, &pending_events);
	lwis_fences_pending_signal_emit(client->lwis_dev, &pending_fences);
}

/*
 * Calling this function requires holding the client's transaction_lock.
 */
static int cancel_waiting_transaction_locked(struct lwis_client *client, int64_t id)
{
	int i;
	struct hlist_node *tmp;
	struct list_head *it_tran, *it_tran_tmp;
	struct lwis_transaction_event_list *it_evt_list;
	struct lwis_transaction *transaction;

	/* Search transactions triggered by events. */
	hash_for_each_safe(client->transaction_list, i, tmp, it_evt_list, node) {
		list_for_each_safe(it_tran, it_tran_tmp, &it_evt_list->list) {
			transaction = list_entry(it_tran, struct lwis_transaction, event_list_node);
			if (transaction->info.id == id) {
				transaction->resp->error_code = -ECANCELED;
				return 0;
			}
		}
	}

	/* Search transactions triggered by trigger_condition. */
	hash_for_each_possible_safe(client->pending_transactions, transaction, tmp,
				    pending_map_node, id) {
		if (transaction->info.id == id) {
			for (i = 0; i < transaction->info.num_nested_transactions; ++i) {
				if (cancel_waiting_transaction_locked(
					    client, transaction->info.nested_transaction_ids[i])) {
					dev_err(client->lwis_dev->dev,
						"Nested transaction with id %llu not found",
						transaction->info.nested_transaction_ids[i]);
				}
			}
			transaction->resp->error_code = -ECANCELED;
			return 0;
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
