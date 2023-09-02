// SPDX-License-Identifier: GPL-2.0
/*
 * GXP mailbox.
 *
 * Copyright (C) 2021 Google LLC
 */

#include <linux/bitops.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/iommu.h>
#include <linux/kthread.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <uapi/linux/sched/types.h>

#include "gxp-dma.h"
#include "gxp-internal.h"
#include "gxp-mailbox.h"
#include "gxp-mailbox-driver.h"
#include "gxp-pm.h"

/* Timeout of 1s by default */
int gxp_mbx_timeout = 1000;
module_param_named(mbx_timeout, gxp_mbx_timeout, int, 0660);

/* Utilities of circular queue operations */

#define CIRCULAR_QUEUE_WRAP_BIT BIT(15)
#define CIRCULAR_QUEUE_INDEX_MASK (CIRCULAR_QUEUE_WRAP_BIT - 1)
#define CIRCULAR_QUEUE_WRAPPED(idx) ((idx) & CIRCULAR_QUEUE_WRAP_BIT)
#define CIRCULAR_QUEUE_REAL_INDEX(idx) ((idx) & CIRCULAR_QUEUE_INDEX_MASK)

#define MBOX_CMD_QUEUE_NUM_ENTRIES 1024
#define MBOX_CMD_QUEUE_SIZE                                                    \
	(sizeof(struct gxp_command) * MBOX_CMD_QUEUE_NUM_ENTRIES)

#define MBOX_RESP_QUEUE_NUM_ENTRIES 1024
#define MBOX_RESP_QUEUE_SIZE                                                   \
	(sizeof(struct gxp_response) * MBOX_RESP_QUEUE_NUM_ENTRIES)

/*
 * Returns the number of elements in a circular queue given its @head, @tail,
 * and @queue_size.
 */
static inline u32 circular_queue_count(u32 head, u32 tail, u32 queue_size)
{
	if (CIRCULAR_QUEUE_WRAPPED(tail) != CIRCULAR_QUEUE_WRAPPED(head))
		return queue_size - CIRCULAR_QUEUE_REAL_INDEX(head) +
		       CIRCULAR_QUEUE_REAL_INDEX(tail);
	else
		return tail - head;
}

/* Increases @index of a circular queue by @inc. */
static inline u32 circular_queue_inc(u32 index, u32 inc, u32 queue_size)
{
	u32 new_index = CIRCULAR_QUEUE_REAL_INDEX(index) + inc;

	if (new_index >= queue_size)
		return (index + inc - queue_size) ^ CIRCULAR_QUEUE_WRAP_BIT;
	else
		return index + inc;
}

/* Sets mailbox->cmd_queue_tail and corresponding CSR on device. */
static void gxp_mailbox_set_cmd_queue_tail(struct gxp_mailbox *mailbox,
					   u32 value)
{
	mailbox->cmd_queue_tail = value;
	gxp_mailbox_write_cmd_queue_tail(mailbox, value);
}

/* Sets mailbox->resp_queue_head and corresponding CSR on device. */
static void gxp_mailbox_set_resp_queue_head(struct gxp_mailbox *mailbox,
					    u32 value)
{
	mailbox->resp_queue_head = value;
	gxp_mailbox_write_resp_queue_head(mailbox, value);
}

/*
 * Increases the command queue tail by @inc.
 *
 * The queue uses the mirrored circular buffer arrangement. Each index (head and
 * tail) has a wrap bit, represented by the constant CIRCULAR_QUEUE_WRAP_BIT.
 * Whenever an index is increased and will exceed the end of the queue, the wrap
 * bit is xor-ed.
 *
 * This method will update both mailbox->cmd_queue_tail and CSR on device.
 *
 * Returns 0 on success.
 * If command queue tail will exceed command queue head after adding @inc,
 * -EBUSY is returned and all fields remain unchanged. The caller should
 * handle this case and implement a mechanism to wait until the consumer
 * consumes commands.
 *
 * Caller must hold cmd_queue_lock.
 */
static int gxp_mailbox_inc_cmd_queue_tail(struct gxp_mailbox *mailbox, u32 inc)
{
	u32 head;
	u32 remain_size;
	u32 new_tail;

	lockdep_assert_held(&mailbox->cmd_queue_lock);

	if (inc > mailbox->cmd_queue_size)
		return -EINVAL;

	head = gxp_mailbox_read_cmd_queue_head(mailbox);
	remain_size = mailbox->cmd_queue_size -
		      circular_queue_count(head, mailbox->cmd_queue_tail,
					   mailbox->cmd_queue_size);
	/* no enough space left */
	if (inc > remain_size)
		return -EBUSY;

	new_tail = circular_queue_inc(mailbox->cmd_queue_tail, inc,
				      mailbox->cmd_queue_size);
	gxp_mailbox_set_cmd_queue_tail(mailbox, new_tail);
	return 0;
}

/*
 * Increases the response queue head by @inc.
 *
 * The queue uses the mirrored circular buffer arrangement. Each index (head and
 * tail) has a wrap bit, represented by the constant CIRCULAR_QUEUE_WRAP_BIT.
 * Whenever an index is increased and will exceed the end of the queue, the wrap
 * bit is xor-ed.
 *
 * This method will update both mailbox->resp_queue_head and CSR on device.
 *
 * Returns 0 on success.
 * -EINVAL is returned if the queue head will exceed tail of queue, and no
 * fields or CSR is updated in this case.
 *
 * Caller must hold resp_queue_lock.
 */
static int gxp_mailbox_inc_resp_queue_head(struct gxp_mailbox *mailbox, u32 inc)
{
	u32 tail;
	u32 size;
	u32 new_head;

	lockdep_assert_held(&mailbox->resp_queue_lock);

	if (inc > mailbox->resp_queue_size)
		return -EINVAL;

	tail = gxp_mailbox_read_resp_queue_tail(mailbox);
	size = circular_queue_count(mailbox->resp_queue_head, tail,
				    mailbox->resp_queue_size);
	if (inc > size)
		return -EINVAL;
	new_head = circular_queue_inc(mailbox->resp_queue_head, inc,
				      mailbox->resp_queue_size);
	gxp_mailbox_set_resp_queue_head(mailbox, new_head);

	return 0;
}

struct gxp_mailbox_manager *gxp_mailbox_create_manager(struct gxp_dev *gxp,
						       uint num_cores)
{
	struct gxp_mailbox_manager *mgr;

	mgr = devm_kzalloc(gxp->dev, sizeof(*mgr), GFP_KERNEL);
	if (!mgr)
		return ERR_PTR(-ENOMEM);

	mgr->gxp = gxp;
	mgr->num_cores = num_cores;
	mgr->get_mailbox_csr_base = gxp_mailbox_get_csr_base;
	mgr->get_mailbox_data_base = gxp_mailbox_get_data_base;

	mgr->mailboxes = devm_kcalloc(gxp->dev, mgr->num_cores,
				      sizeof(*mgr->mailboxes), GFP_KERNEL);
	if (!mgr->mailboxes)
		return ERR_PTR(-ENOMEM);

	return mgr;
}

/*
 * Pops the wait_list until the sequence number of @resp is found, and copies
 * @resp to the found entry.
 *
 * Entries in wait_list should have sequence number in increasing order, but
 * the responses arriving and being handled may be out-of-order.
 *
 * Iterate over the wait_list, comparing #cur->resp->seq with @resp->seq:
 * 1. #cur->resp->seq > @resp->seq:
 *   - Nothing to do, either @resp is invalid or its command timed out.
 *   - We're done.
 * 2. #cur->resp->seq == @resp->seq:
 *   - Copy @resp, pop the head.
 *   - If #cur->resp has a destination queue, push it to that queue
 *   - We're done.
 * 3. #cur->resp->seq < @resp->seq:
 *   - @resp has arrived out of sequence order.
 *   - Leave #cur->resp in the wait_list.
 *   - Keep iterating unless the list is exhausted.
 */
static void gxp_mailbox_handle_response(struct gxp_mailbox *mailbox,
					const struct gxp_response *resp)
{
	struct gxp_mailbox_wait_list *cur, *nxt;
	struct gxp_async_response *async_resp;
	unsigned long flags;

	mutex_lock(&mailbox->wait_list_lock);

	list_for_each_entry_safe(cur, nxt, &mailbox->wait_list, list) {
		if (cur->resp->seq > resp->seq) {
			/*
			 * This response has already timed out and been removed
			 * from the wait list (or this is an invalid response).
			 * Drop it.
			 */
			break;
		}
		if (cur->resp->seq == resp->seq) {
			memcpy(cur->resp, resp, sizeof(*resp));
			list_del(&cur->list);
			if (cur->is_async) {
				async_resp =
					container_of(cur->resp,
						     struct gxp_async_response,
						     resp);

				cancel_delayed_work(&async_resp->timeout_work);
				gxp_pm_update_requested_power_states(
					async_resp->mailbox->gxp,
					async_resp->gxp_power_state,
					async_resp->requested_low_clkmux,
					AUR_OFF, false,
					async_resp->memory_power_state,
					AUR_MEM_UNDEFINED);

				spin_lock_irqsave(async_resp->dest_queue_lock,
						  flags);

				list_add_tail(&async_resp->list_entry,
					      async_resp->dest_queue);
				/*
				 * Marking the dest_queue as NULL indicates the
				 * response was handled in case its timeout
				 * handler fired between acquiring the
				 * wait_list_lock and cancelling the timeout.
				 */
				async_resp->dest_queue = NULL;

				/*
				 * Don't release the dest_queue_lock until both
				 * any eventfd has been signaled and any waiting
				 * thread has been woken. Otherwise one thread
				 * might consume and free the response before
				 * this function is done with it.
				 */
				if (async_resp->eventfd) {
					gxp_eventfd_signal(async_resp->eventfd);
					gxp_eventfd_put(async_resp->eventfd);
				}

				wake_up(async_resp->dest_queue_waitq);

				spin_unlock_irqrestore(
					async_resp->dest_queue_lock, flags);

			}
			kfree(cur);
			break;
		}
	}

	mutex_unlock(&mailbox->wait_list_lock);
}

/*
 * Fetches elements in the response queue.
 *
 * Returns the pointer of fetched response elements.
 * @total_ptr will be the number of elements fetched.
 *
 * Returns -ENOMEM if failed on memory allocation.
 * Returns NULL if the response queue is empty.
 */
static struct gxp_response *
gxp_mailbox_fetch_responses(struct gxp_mailbox *mailbox, u32 *total_ptr)
{
	u32 head;
	u32 tail;
	u32 count;
	u32 i;
	u32 j;
	u32 total = 0;
	const u32 size = mailbox->resp_queue_size;
	const struct gxp_response *queue = mailbox->resp_queue;
	struct gxp_response *ret = NULL;
	struct gxp_response *prev_ptr = NULL;

	mutex_lock(&mailbox->resp_queue_lock);

	head = mailbox->resp_queue_head;
	/* loop until our head equals to CSR tail */
	while (1) {
		tail = gxp_mailbox_read_resp_queue_tail(mailbox);
		count = circular_queue_count(head, tail, size);
		if (count == 0)
			break;

		prev_ptr = ret;
		ret = krealloc(prev_ptr, (total + count) * sizeof(*queue),
			       GFP_KERNEL);
		/*
		 * Out-of-memory, we can return the previously fetched responses
		 * if any, or ENOMEM otherwise.
		 */
		if (!ret) {
			if (!prev_ptr)
				ret = ERR_PTR(-ENOMEM);
			else
				ret = prev_ptr;
			break;
		}
		/* copy responses */
		j = CIRCULAR_QUEUE_REAL_INDEX(head);
		for (i = 0; i < count; i++) {
			memcpy(&ret[total], &queue[j], sizeof(*queue));
			ret[total].status = GXP_RESP_OK;
			j = (j + 1) % size;
			total++;
		}
		head = circular_queue_inc(head, count, size);
	}
	gxp_mailbox_inc_resp_queue_head(mailbox, total);

	mutex_unlock(&mailbox->resp_queue_lock);
	/*
	 * Now that the response queue has been drained, send an interrupt
	 * to the device in case firmware was waiting for us to consume
	 * responses.
	 */
	if (total == size) {
		/* TODO(b/190868834) define interrupt bits */
		gxp_mailbox_generate_device_interrupt(mailbox, BIT(0));
	}

	*total_ptr = total;
	return ret;
}

/*
 * Fetches and handles responses, then wakes up threads that are waiting for a
 * response.
 *
 * Note: this worker is scheduled in the IRQ handler, to prevent use-after-free
 * or race-condition bugs, gxp_mailbox_release() must be called before free the
 * mailbox.
 */
static void gxp_mailbox_consume_responses_work(struct kthread_work *work)
{
	struct gxp_mailbox *mailbox =
		container_of(work, struct gxp_mailbox, response_work);
	struct gxp_response *responses;
	u32 i;
	u32 count = 0;

	/* fetch responses and bump RESP_QUEUE_HEAD */
	responses = gxp_mailbox_fetch_responses(mailbox, &count);
	if (IS_ERR(responses)) {
		dev_err(mailbox->gxp->dev,
			"GXP Mailbox failed on fetching responses: %ld",
			PTR_ERR(responses));
		return;
	}

	for (i = 0; i < count; i++)
		gxp_mailbox_handle_response(mailbox, &responses[i]);
	/*
	 * Responses handled, wake up threads that are waiting for a response.
	 */
	wake_up(&mailbox->wait_list_waitq);
	kfree(responses);
}

/*
 * IRQ handler of GXP mailbox.
 *
 * Puts the gxp_mailbox_consume_responses_work() into the system work queue.
 */
static inline void gxp_mailbox_handle_irq(struct gxp_mailbox *mailbox)
{
	kthread_queue_work(&mailbox->response_worker, &mailbox->response_work);
}

/* Priority level for realtime worker threads */
#define GXP_RT_THREAD_PRIORITY 2
static struct task_struct *
create_response_rt_thread(struct device *dev, void *data, int core_id)
{
	static const struct sched_param param = {
		.sched_priority = GXP_RT_THREAD_PRIORITY,
	};
	struct task_struct *task = kthread_create(kthread_worker_fn, data,
						  "gxp_response_%d", core_id);

	if (!IS_ERR(task)) {
		wake_up_process(task);
		if (sched_setscheduler(task, SCHED_FIFO, &param))
			dev_warn(dev, "response task %d not set to RT prio",
				 core_id);
		else
			dev_dbg(dev, "response task %d set to RT prio: %i",
				core_id, param.sched_priority);
	}

	return task;
}

static struct gxp_mailbox *create_mailbox(struct gxp_mailbox_manager *mgr,
					  struct gxp_virtual_device *vd,
					  uint virt_core, u8 core_id)
{
	struct gxp_mailbox *mailbox;

	mailbox = kzalloc(sizeof(*mailbox), GFP_KERNEL);
	if (!mailbox)
		goto err_mailbox;

	mailbox->core_id = core_id;
	mailbox->gxp = mgr->gxp;
	mailbox->csr_reg_base = mgr->get_mailbox_csr_base(mgr->gxp, core_id);
	mailbox->data_reg_base = mgr->get_mailbox_data_base(mgr->gxp, core_id);

	/* Allocate and initialize the command queue */
	mailbox->cmd_queue = (struct gxp_command *)gxp_dma_alloc_coherent(
		mailbox->gxp, vd, BIT(virt_core),
		sizeof(struct gxp_command) * MBOX_CMD_QUEUE_NUM_ENTRIES,
		&(mailbox->cmd_queue_device_addr), GFP_KERNEL, 0);
	if (!mailbox->cmd_queue)
		goto err_cmd_queue;

	mailbox->cmd_queue_size = MBOX_CMD_QUEUE_NUM_ENTRIES;
	mailbox->cmd_queue_tail = 0;
	mutex_init(&mailbox->cmd_queue_lock);

	/* Allocate and initialize the response queue */
	mailbox->resp_queue = (struct gxp_response *)gxp_dma_alloc_coherent(
		mailbox->gxp, vd, BIT(virt_core),
		sizeof(struct gxp_response) * MBOX_RESP_QUEUE_NUM_ENTRIES,
		&(mailbox->resp_queue_device_addr), GFP_KERNEL, 0);
	if (!mailbox->resp_queue)
		goto err_resp_queue;

	mailbox->resp_queue_size = MBOX_RESP_QUEUE_NUM_ENTRIES;
	mailbox->resp_queue_head = 0;
	mutex_init(&mailbox->resp_queue_lock);

	/* Allocate and initialize the mailbox descriptor */
	mailbox->descriptor =
		(struct gxp_mailbox_descriptor *)gxp_dma_alloc_coherent(
			mailbox->gxp, vd, BIT(virt_core),
			sizeof(struct gxp_mailbox_descriptor),
			&(mailbox->descriptor_device_addr), GFP_KERNEL, 0);
	if (!mailbox->descriptor)
		goto err_descriptor;

	mailbox->descriptor->cmd_queue_device_addr =
		mailbox->cmd_queue_device_addr;
	mailbox->descriptor->resp_queue_device_addr =
		mailbox->resp_queue_device_addr;
	mailbox->descriptor->cmd_queue_size = mailbox->cmd_queue_size;
	mailbox->descriptor->resp_queue_size = mailbox->resp_queue_size;

	kthread_init_worker(&mailbox->response_worker);
	mailbox->response_thread = create_response_rt_thread(
		mailbox->gxp->dev, &mailbox->response_worker, core_id);
	if (IS_ERR(mailbox->response_thread))
		goto err_thread;

	/* Initialize driver before interacting with its registers */
	gxp_mailbox_driver_init(mailbox);

	return mailbox;

err_thread:
	gxp_dma_free_coherent(mailbox->gxp, vd, BIT(virt_core),
			      sizeof(struct gxp_mailbox_descriptor),
			      mailbox->descriptor,
			      mailbox->descriptor_device_addr);
err_descriptor:
	gxp_dma_free_coherent(
		mailbox->gxp, vd, BIT(virt_core),
		sizeof(struct gxp_response) * mailbox->resp_queue_size,
		mailbox->resp_queue, mailbox->resp_queue_device_addr);
err_resp_queue:
	gxp_dma_free_coherent(
		mailbox->gxp, vd, BIT(virt_core),
		sizeof(struct gxp_command) * mailbox->cmd_queue_size,
		mailbox->cmd_queue, mailbox->cmd_queue_device_addr);
err_cmd_queue:
	kfree(mailbox);
err_mailbox:
	return ERR_PTR(-ENOMEM);
}

static void enable_mailbox(struct gxp_mailbox *mailbox)
{
	gxp_mailbox_write_descriptor(mailbox, mailbox->descriptor_device_addr);
	gxp_mailbox_write_cmd_queue_head(mailbox, 0);
	gxp_mailbox_write_cmd_queue_tail(mailbox, 0);
	gxp_mailbox_write_resp_queue_head(mailbox, 0);
	gxp_mailbox_write_resp_queue_tail(mailbox, 0);

	mailbox->handle_irq = gxp_mailbox_handle_irq;
	mailbox->cur_seq = 0;
	init_waitqueue_head(&mailbox->wait_list_waitq);
	INIT_LIST_HEAD(&mailbox->wait_list);
	mutex_init(&mailbox->wait_list_lock);
	kthread_init_work(&mailbox->response_work, gxp_mailbox_consume_responses_work);

	/* Only enable interrupts once everything has been setup */
	gxp_mailbox_driver_enable_interrupts(mailbox);
	/* Enable the mailbox */
	gxp_mailbox_write_status(mailbox, 1);
	/* TODO(b/190868834) define interrupt bits */
	gxp_mailbox_generate_device_interrupt(mailbox, BIT(0));
}

struct gxp_mailbox *gxp_mailbox_alloc(struct gxp_mailbox_manager *mgr,
				      struct gxp_virtual_device *vd,
				      uint virt_core, u8 core_id)
{
	struct gxp_mailbox *mailbox;

	mailbox = create_mailbox(mgr, vd, virt_core, core_id);
	if (IS_ERR(mailbox))
		return mailbox;

	enable_mailbox(mailbox);

	return mailbox;
}

void gxp_mailbox_release(struct gxp_mailbox_manager *mgr,
			 struct gxp_virtual_device *vd, uint virt_core,
			 struct gxp_mailbox *mailbox)
{
	int i;
	struct gxp_mailbox_wait_list *cur, *nxt;
	struct gxp_async_response *async_resp;
	struct list_head resps_to_flush;
	unsigned long flags;

	if (!mailbox) {
		dev_err(mgr->gxp->dev,
			"Attempt to release nonexistent mailbox\n");
		return;
	}

	/*
	 * Halt the mailbox driver by preventing any incoming requests..
	 * This must happen before the mailbox itself is cleaned-up/released
	 * to make sure the mailbox does not disappear out from under the
	 * mailbox driver. This also halts all incoming responses/interrupts.
	 */
	gxp_mailbox_driver_disable_interrupts(mailbox);

	/* Halt and flush any traffic */
	kthread_cancel_work_sync(&mailbox->response_work);
	for (i = 0; i < GXP_MAILBOX_INT_BIT_COUNT; i++) {
		if (mailbox->interrupt_handlers[i])
			cancel_work_sync(mailbox->interrupt_handlers[i]);
	}

	/*
	 * At this point only async responses should be pending. Flush them all
	 * from the `wait_list` at once so any remaining timeout workers
	 * waiting on `wait_list_lock` will know their responses have been
	 * handled already.
	 */
	INIT_LIST_HEAD(&resps_to_flush);
	mutex_lock(&mailbox->wait_list_lock);
	list_for_each_entry_safe(cur, nxt, &mailbox->wait_list, list) {
		list_del(&cur->list);
		if (cur->is_async) {
			list_add_tail(&cur->list, &resps_to_flush);
			/*
			 * Clear the response's destination queue so that if the
			 * timeout worker is running, it won't try to process
			 * this response after `wait_list_lock` is released.
			 */
			async_resp = container_of(
				cur->resp, struct gxp_async_response, resp);
			spin_lock_irqsave(async_resp->dest_queue_lock, flags);
			async_resp->dest_queue = NULL;
			spin_unlock_irqrestore(async_resp->dest_queue_lock,
					       flags);

		} else {
			dev_warn(
				mailbox->gxp->dev,
				"Unexpected synchronous command pending on mailbox release\n");
			kfree(cur);
		}
	}
	mutex_unlock(&mailbox->wait_list_lock);

	/*
	 * Cancel the timeout timer of and free any responses that were still in
	 * the `wait_list` above.
	 */
	list_for_each_entry_safe(cur, nxt, &resps_to_flush, list) {
		list_del(&cur->list);
		async_resp = container_of(cur->resp, struct gxp_async_response,
					  resp);
		cancel_delayed_work_sync(&async_resp->timeout_work);
		kfree(async_resp);
		kfree(cur);
	}

	/* Reset the mailbox HW */
	gxp_mailbox_reset_hw(mailbox);

	/* Cleanup now that all mailbox interactions are finished */
	gxp_mailbox_driver_exit(mailbox);

	/*
	 * At this point all users of the mailbox have been halted or are
	 * waiting on gxp->vd_semaphore, which this function's caller has
	 * locked for writing.
	 * It is now safe to clear the manager's mailbox pointer.
	 */
	mgr->mailboxes[mailbox->core_id] = NULL;

	/* Clean up resources */
	gxp_dma_free_coherent(
		mailbox->gxp, vd, BIT(virt_core),
		sizeof(struct gxp_command) * mailbox->cmd_queue_size,
		mailbox->cmd_queue, mailbox->cmd_queue_device_addr);
	gxp_dma_free_coherent(
		mailbox->gxp, vd, BIT(virt_core),
		sizeof(struct gxp_response) * mailbox->resp_queue_size,
		mailbox->resp_queue, mailbox->resp_queue_device_addr);
	gxp_dma_free_coherent(mailbox->gxp, vd, BIT(virt_core),
			      sizeof(struct gxp_mailbox_descriptor),
			      mailbox->descriptor,
			      mailbox->descriptor_device_addr);
	kthread_flush_worker(&mailbox->response_worker);
	kthread_stop(mailbox->response_thread);
	kfree(mailbox);
}

void gxp_mailbox_reset(struct gxp_mailbox *mailbox)
{
	dev_notice(mailbox->gxp->dev, "%s not yet implemented\n", __func__);
}

/*
 * Adds @resp to @mailbox->wait_list.
 *
 * wait_list is a FIFO queue, with sequence number in increasing order.
 *
 * Returns 0 on success, or -ENOMEM if failed on allocation.
 */
static int gxp_mailbox_push_wait_resp(struct gxp_mailbox *mailbox,
				      struct gxp_response *resp, bool is_async)
{
	struct gxp_mailbox_wait_list *entry =
		kzalloc(sizeof(*entry), GFP_KERNEL);

	if (!entry)
		return -ENOMEM;
	entry->resp = resp;
	entry->is_async = is_async;
	mutex_lock(&mailbox->wait_list_lock);
	list_add_tail(&entry->list, &mailbox->wait_list);
	mutex_unlock(&mailbox->wait_list_lock);

	return 0;
}

/*
 * Removes the response previously pushed with gxp_mailbox_push_wait_resp().
 *
 * This is used when the kernel gives up waiting for the response.
 */
static void gxp_mailbox_del_wait_resp(struct gxp_mailbox *mailbox,
				      struct gxp_response *resp)
{
	struct gxp_mailbox_wait_list *cur;

	mutex_lock(&mailbox->wait_list_lock);

	list_for_each_entry(cur, &mailbox->wait_list, list) {
		if (cur->resp->seq > resp->seq) {
			/*
			 * Sequence numbers in wait_list are in increasing
			 * order. This case implies no entry in the list
			 * matches @resp's sequence number.
			 */
			break;
		}
		if (cur->resp->seq == resp->seq) {
			list_del(&cur->list);
			kfree(cur);
			break;
		}
	}

	mutex_unlock(&mailbox->wait_list_lock);
}

static int gxp_mailbox_enqueue_cmd(struct gxp_mailbox *mailbox,
				   struct gxp_command *cmd,
				   struct gxp_response *resp,
				   bool resp_is_async)
{
	int ret;
	u32 tail;

	mutex_lock(&mailbox->cmd_queue_lock);

	cmd->seq = mailbox->cur_seq;
	/*
	 * The lock ensures mailbox->cmd_queue_tail cannot be changed by
	 * other processes (this method should be the only one to modify the
	 * value of tail), therefore we can remember its value here and use it
	 * in various places below.
	 */
	tail = mailbox->cmd_queue_tail;

	/*
	 * If the cmd queue is full, it's up to the caller to retry.
	 */
	if (gxp_mailbox_read_cmd_queue_head(mailbox) ==
	    (tail ^ CIRCULAR_QUEUE_WRAP_BIT)) {
		ret = -EAGAIN;
		goto out;
	}

	if (resp) {
		/*
		 * Add @resp to the wait_list only if the cmd can be pushed
		 * successfully.
		 */
		resp->seq = cmd->seq;
		resp->status = GXP_RESP_WAITING;
		ret = gxp_mailbox_push_wait_resp(mailbox, resp, resp_is_async);
		if (ret)
			goto out;
	}
	/* size of cmd_queue is a multiple of sizeof(*cmd) */
	memcpy(mailbox->cmd_queue + CIRCULAR_QUEUE_REAL_INDEX(tail), cmd,
	       sizeof(*cmd));
	gxp_mailbox_inc_cmd_queue_tail(mailbox, 1);
	/* triggers doorbell */
	/* TODO(b/190868834) define interrupt bits */
	gxp_mailbox_generate_device_interrupt(mailbox, BIT(0));
	/* bumps sequence number after the command is sent */
	mailbox->cur_seq++;
	ret = 0;
out:
	mutex_unlock(&mailbox->cmd_queue_lock);
	if (ret)
		dev_err(mailbox->gxp->dev, "%s: ret=%d", __func__, ret);

	return ret;
}

int gxp_mailbox_execute_cmd(struct gxp_mailbox *mailbox,
			    struct gxp_command *cmd, struct gxp_response *resp)
{
	int ret;

	ret = gxp_mailbox_enqueue_cmd(mailbox, cmd, resp,
				      /* resp_is_async = */ false);
	if (ret)
		return ret;
	ret = wait_event_timeout(mailbox->wait_list_waitq,
				 resp->status != GXP_RESP_WAITING,
				 msecs_to_jiffies(MAILBOX_TIMEOUT));
	if (!ret) {
		dev_notice(mailbox->gxp->dev, "%s: event wait timeout",
			   __func__);
		gxp_mailbox_del_wait_resp(mailbox, resp);
		return -ETIMEDOUT;
	}
	if (resp->status != GXP_RESP_OK) {
		dev_notice(mailbox->gxp->dev, "%s: resp status=%u", __func__,
			   resp->status);
		return -ENOMSG;
	}

	return resp->retval;
}

static void async_cmd_timeout_work(struct work_struct *work)
{
	struct gxp_async_response *async_resp = container_of(
		work, struct gxp_async_response, timeout_work.work);
	unsigned long flags;

	/*
	 * This function will acquire the mailbox wait_list_lock. This means if
	 * response processing is in progress, it will complete before this
	 * response can be removed from the wait list.
	 *
	 * Once this function has the wait_list_lock, no future response
	 * processing will begin until this response has been removed.
	 */
	gxp_mailbox_del_wait_resp(async_resp->mailbox, &async_resp->resp);

	/*
	 * Check if this response still has a valid destination queue, in case
	 * an in-progress call to `gxp_mailbox_handle_response()` completed
	 * the response while `gxp_mailbox_del_wait_resp()` was waiting for
	 * the wait_list_lock.
	 */
	spin_lock_irqsave(async_resp->dest_queue_lock, flags);
	if (async_resp->dest_queue) {
		async_resp->resp.status = GXP_RESP_CANCELLED;
		list_add_tail(&async_resp->list_entry, async_resp->dest_queue);
		spin_unlock_irqrestore(async_resp->dest_queue_lock, flags);

		gxp_pm_update_requested_power_states(
			async_resp->mailbox->gxp, async_resp->gxp_power_state,
			async_resp->requested_low_clkmux, AUR_OFF, false,
			async_resp->memory_power_state, AUR_MEM_UNDEFINED);

		if (async_resp->eventfd) {
			gxp_eventfd_signal(async_resp->eventfd);
			gxp_eventfd_put(async_resp->eventfd);
		}

		wake_up(async_resp->dest_queue_waitq);
	} else {
		spin_unlock_irqrestore(async_resp->dest_queue_lock, flags);
	}
}

int gxp_mailbox_execute_cmd_async(struct gxp_mailbox *mailbox,
				  struct gxp_command *cmd,
				  struct list_head *resp_queue,
				  spinlock_t *queue_lock,
				  wait_queue_head_t *queue_waitq,
				  uint gxp_power_state, uint memory_power_state,
				  bool requested_low_clkmux,
				  struct gxp_eventfd *eventfd)
{
	struct gxp_async_response *async_resp;
	int ret;

	async_resp = kzalloc(sizeof(*async_resp), GFP_KERNEL);
	if (!async_resp)
		return -ENOMEM;

	async_resp->mailbox = mailbox;
	async_resp->dest_queue = resp_queue;
	async_resp->dest_queue_lock = queue_lock;
	async_resp->dest_queue_waitq = queue_waitq;
	async_resp->gxp_power_state = gxp_power_state;
	async_resp->memory_power_state = memory_power_state;
	async_resp->requested_low_clkmux = requested_low_clkmux;
	if (eventfd && gxp_eventfd_get(eventfd))
		async_resp->eventfd = eventfd;
	else
		async_resp->eventfd = NULL;

	INIT_DELAYED_WORK(&async_resp->timeout_work, async_cmd_timeout_work);
	schedule_delayed_work(&async_resp->timeout_work,
			      msecs_to_jiffies(MAILBOX_TIMEOUT));

	gxp_pm_update_requested_power_states(
		mailbox->gxp, AUR_OFF, false, gxp_power_state,
		requested_low_clkmux, AUR_MEM_UNDEFINED, memory_power_state);
	ret = gxp_mailbox_enqueue_cmd(mailbox, cmd, &async_resp->resp,
				      /* resp_is_async = */ true);
	if (ret)
		goto err_free_resp;

	return 0;

err_free_resp:
	gxp_pm_update_requested_power_states(mailbox->gxp, gxp_power_state,
					     requested_low_clkmux, AUR_OFF, false,
					     memory_power_state,
					     AUR_MEM_UNDEFINED);
	cancel_delayed_work_sync(&async_resp->timeout_work);
	kfree(async_resp);
	return ret;
}

int gxp_mailbox_register_interrupt_handler(struct gxp_mailbox *mailbox,
					   u32 int_bit,
					   struct work_struct *handler)
{
	/* Bit 0 is reserved for incoming mailbox responses */
	if (int_bit == 0 || int_bit >= GXP_MAILBOX_INT_BIT_COUNT)
		return -EINVAL;

	mailbox->interrupt_handlers[int_bit] = handler;

	return 0;
}

int gxp_mailbox_unregister_interrupt_handler(struct gxp_mailbox *mailbox,
					   u32 int_bit)
{
	/* Bit 0 is reserved for incoming mailbox responses */
	if (int_bit == 0 || int_bit >= GXP_MAILBOX_INT_BIT_COUNT)
		return -EINVAL;

	mailbox->interrupt_handlers[int_bit] = NULL;

	return 0;
}
