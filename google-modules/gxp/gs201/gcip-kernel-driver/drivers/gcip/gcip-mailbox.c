// SPDX-License-Identifier: GPL-2.0-only
/*
 * GCIP Mailbox Interface.
 *
 * Copyright (C) 2022 Google LLC
 */

#include <linux/device.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/string.h> /* memcpy */
#include <linux/wait.h>

#include <gcip/gcip-mailbox.h>

#if IS_ENABLED(CONFIG_GCIP_TEST)
#include "unittests/helper/gcip-mailbox-controller.h"

#define TEST_TRIGGER_TIMEOUT_RACE(awaiter) gcip_mailbox_controller_trigger_timeout_race(awaiter)
#else
#define TEST_TRIGGER_TIMEOUT_RACE(...)
#endif

#define GET_CMD_QUEUE_HEAD() mailbox->ops->get_cmd_queue_head(mailbox)
#define GET_CMD_QUEUE_TAIL() mailbox->ops->get_cmd_queue_tail(mailbox)
#define INC_CMD_QUEUE_TAIL(inc) mailbox->ops->inc_cmd_queue_tail(mailbox, inc)
#define ACQUIRE_CMD_QUEUE_LOCK(try) mailbox->ops->acquire_cmd_queue_lock(mailbox, try)
#define RELEASE_CMD_QUEUE_LOCK() mailbox->ops->release_cmd_queue_lock(mailbox)

#define GET_CMD_ELEM_SEQ(cmd) mailbox->ops->get_cmd_elem_seq(mailbox, cmd)
#define SET_CMD_ELEM_SEQ(cmd, seq) mailbox->ops->set_cmd_elem_seq(mailbox, cmd, seq)
#define GET_CMD_ELEM_CODE(cmd) mailbox->ops->get_cmd_elem_code(mailbox, cmd)

#define GET_RESP_QUEUE_SIZE() mailbox->ops->get_resp_queue_size(mailbox)
#define GET_RESP_QUEUE_HEAD() mailbox->ops->get_resp_queue_head(mailbox)
#define INC_RESP_QUEUE_HEAD(inc) mailbox->ops->inc_resp_queue_head(mailbox, inc)
#define GET_RESP_QUEUE_TAIL() mailbox->ops->get_resp_queue_tail(mailbox)
#define ACQUIRE_RESP_QUEUE_LOCK(try) mailbox->ops->acquire_resp_queue_lock(mailbox, try)
#define RELEASE_RESP_QUEUE_LOCK() mailbox->ops->release_resp_queue_lock(mailbox)

#define GET_RESP_ELEM_SEQ(resp) mailbox->ops->get_resp_elem_seq(mailbox, resp)
#define SET_RESP_ELEM_SEQ(resp, seq) mailbox->ops->set_resp_elem_seq(mailbox, resp, seq)
#define GET_RESP_ELEM_STATUS(resp) mailbox->ops->get_resp_elem_status(mailbox, resp)
#define SET_RESP_ELEM_STATUS(resp, status) mailbox->ops->set_resp_elem_status(mailbox, resp, status)

#define ACQUIRE_WAIT_LIST_LOCK(irqsave, flags)                                                     \
	mailbox->ops->acquire_wait_list_lock(mailbox, irqsave, flags)
#define RELEASE_WAIT_LIST_LOCK(irqrestore, flags)                                                  \
	mailbox->ops->release_wait_list_lock(mailbox, irqrestore, flags)

struct gcip_mailbox_wait_list_elem {
	struct list_head list;
	void *resp;
	struct gcip_mailbox_resp_awaiter *awaiter;
};

static void gcip_mailbox_awaiter_release(struct gcip_mailbox_resp_awaiter *awaiter)
{
	if (awaiter->release_data)
		awaiter->release_data(awaiter->data);
	kfree(awaiter);
}

static void gcip_mailbox_awaiter_dec_refs(struct gcip_mailbox_resp_awaiter *awaiter)
{
	if (refcount_dec_and_test(&awaiter->refs))
		gcip_mailbox_awaiter_release(awaiter);
}

/*
 * Removes the response previously pushed with gcip_mailbox_push_wait_resp().
 *
 * This is used when the kernel gives up waiting for the response.
 */
static void gcip_mailbox_del_wait_resp(struct gcip_mailbox *mailbox, void *resp)
{
	struct gcip_mailbox_wait_list_elem *cur;
	unsigned long flags;
	u64 cur_seq, seq = GET_RESP_ELEM_SEQ(resp);

	ACQUIRE_WAIT_LIST_LOCK(true, &flags);

	list_for_each_entry (cur, &mailbox->wait_list, list) {
		cur_seq = GET_RESP_ELEM_SEQ(cur->resp);
		if (cur_seq > seq)
			break;
		if (cur_seq == seq) {
			list_del(&cur->list);
			if (cur->awaiter) {
				/* Remove the reference of the arrived handler. */
				gcip_mailbox_awaiter_dec_refs(cur->awaiter);
			}
			kfree(cur);
			break;
		}
	}

	RELEASE_WAIT_LIST_LOCK(true, flags);
}

/*
 * Adds @resp to @mailbox->wait_list. If @awaiter is not NULL, the @resp is asynchronous.
 * Otherwise, the @resp is synchronous.
 *
 * wait_list is a FIFO queue, with sequence number in increasing order.
 *
 * Returns 0 on success, or -ENOMEM if failed on allocation.
 */
static int gcip_mailbox_push_wait_resp(struct gcip_mailbox *mailbox, void *resp,
				       struct gcip_mailbox_resp_awaiter *awaiter)
{
	struct gcip_mailbox_wait_list_elem *entry = kzalloc(sizeof(*entry), GFP_KERNEL);
	unsigned long flags;
	int ret;

	if (!entry)
		return -ENOMEM;

	if (mailbox->ops->before_enqueue_wait_list) {
		ret = mailbox->ops->before_enqueue_wait_list(mailbox, resp, awaiter);
		if (ret) {
			kfree(entry);
			return ret;
		}
	}

	/* Increase a reference of arrived handler. */
	if (awaiter)
		refcount_inc(&awaiter->refs);

	entry->resp = resp;
	entry->awaiter = awaiter;
	ACQUIRE_WAIT_LIST_LOCK(true, &flags);
	list_add_tail(&entry->list, &mailbox->wait_list);
	RELEASE_WAIT_LIST_LOCK(true, flags);

	return 0;
}

/*
 * Pushes @cmd to the command queue of mailbox and returns. @resp should be passed if the request
 * is synchronous and want to get the response. If @resp is NULL even though the request is
 * synchronous, the @cmd will be put into the queue, but the caller may not wait the response and
 * ignore it. If the request is async, @awaiter should be passed too.
 */
static int gcip_mailbox_enqueue_cmd(struct gcip_mailbox *mailbox, void *cmd, void *resp,
				    struct gcip_mailbox_resp_awaiter *awaiter)
{
	int ret = 0;
	u32 tail;

	ACQUIRE_CMD_QUEUE_LOCK(false);

	SET_CMD_ELEM_SEQ(cmd, mailbox->cur_seq);
	/*
	 * The lock ensures mailbox cmd_queue_tail cannot be changed by other processes (this
	 * method should be the only one to modify the value of tail), therefore we can remember
	 * its value here and use it in the condition of wait_event() call.
	 */
	tail = GET_CMD_QUEUE_TAIL();

	if (mailbox->ops->wait_for_cmd_queue_not_full) {
		/* Wait until the cmd queue has a space for putting cmd. */
		ret = mailbox->ops->wait_for_cmd_queue_not_full(mailbox);
		if (ret)
			goto out;
	} else if (GET_CMD_QUEUE_HEAD() == (tail ^ mailbox->queue_wrap_bit)) {
		/*
		 * Default logic of checking the fullness of cmd_queue. If the cmd_queue is full,
		 * it's up to the caller to retry.
		 */
		ret = -EAGAIN;
		goto out;
	}

	if (resp) {
		/* Adds @resp to the wait_list only if the cmd can be pushed successfully. */
		SET_RESP_ELEM_SEQ(resp, GET_CMD_ELEM_SEQ(cmd));
		SET_RESP_ELEM_STATUS(resp, GCIP_MAILBOX_STATUS_WAITING_RESPONSE);
		ret = gcip_mailbox_push_wait_resp(mailbox, resp, awaiter);
		if (ret)
			goto out;
	}
	/* Size of cmd_queue is a multiple of mailbox->cmd_elem_size. */
	memcpy(mailbox->cmd_queue + mailbox->cmd_elem_size *
					    CIRC_QUEUE_REAL_INDEX(tail, mailbox->queue_wrap_bit),
	       cmd, mailbox->cmd_elem_size);
	INC_CMD_QUEUE_TAIL(1);
	if (mailbox->ops->after_enqueue_cmd) {
		ret = mailbox->ops->after_enqueue_cmd(mailbox, cmd);
		if (ret < 0) {
			/*
			 * Currently, as both DSP and EdgeTPU never return errors, do nothing
			 * here. We can decide later how to rollback the status such as
			 * `cmd_queue_tail` when the possibility of returning an error is raised.
			 */
			dev_warn(mailbox->dev,
				 "after_enqueue_cmd returned an error, but not handled: ret=%d\n",
				 ret);
			goto out;
		}
		mailbox->cur_seq += ret;
		ret = 0;
	} else
		mailbox->cur_seq += 1;

out:
	RELEASE_CMD_QUEUE_LOCK();
	if (ret)
		dev_dbg(mailbox->dev, "%s: ret=%d", __func__, ret);

	return ret;
}

/*
 * Handler of a response.
 * Pops the wait_list until the sequence number of @resp is found, and copies @resp to the found
 * entry.
 *
 * Both entry in wait_list and response handling should have sequence number in increasing order.
 * Comparing the #seq of head of wait_list with @resp->seq, we have three cases:
 * 1. #seq > @resp->seq:
 *   - Nothing to do, @resp is not needed and we're done.
 * 2. #seq == @resp->seq:
 *   - Copy @resp, pop the head and we're done.
 * 3. #seq < @resp->seq:
 *   - If @mailbox->ignore_seq_order is specified, this is a normal case and the entry is skipped.
 *   - Otherwise, it *should* not happen, this implies the sequence number of either entries in
 *     wait_list or responses are out-of-order, or remote didn't respond to a command. In this
 *     case, the status of response will be set to GCIP_MAILBOX_STATUS_NO_RESPONSE. Then pop until
 *     case 1. or 2.
 */
static void gcip_mailbox_handle_response(struct gcip_mailbox *mailbox, void *resp)
{
	struct gcip_mailbox_wait_list_elem *cur, *nxt;
	struct gcip_mailbox_resp_awaiter *awaiter;
	unsigned long flags;
	u64 cur_seq, seq = GET_RESP_ELEM_SEQ(resp);

	/* If before_handle_resp is defined and it returns false, don't handle the response */
	if (mailbox->ops->before_handle_resp && !mailbox->ops->before_handle_resp(mailbox, resp))
		return;

	SET_RESP_ELEM_STATUS(resp, GCIP_MAILBOX_STATUS_OK);
	ACQUIRE_WAIT_LIST_LOCK(true, &flags);

	list_for_each_entry_safe (cur, nxt, &mailbox->wait_list, list) {
		cur_seq = GET_RESP_ELEM_SEQ(cur->resp);
		if (cur_seq > seq) {
			/*
			 * This response has already timed out and been removed
			 * from the wait list (or this is an invalid response).
			 * Drop it.
			 */
			break;
		}
		if (cur_seq == seq) {
			memcpy(cur->resp, resp, mailbox->resp_elem_size);
			list_del(&cur->list);
			if (cur->awaiter) {
				awaiter = cur->awaiter;

				/*
				 * The timedout handler will be fired, but pended by waiting for
				 * acquiring the wait_list_lock.
				 */
				TEST_TRIGGER_TIMEOUT_RACE(awaiter);

				/*
				 * If canceling timeout_work succeeded, we have to decrease the
				 * reference count here because the timeout handler will not be
				 * called. Otherwise, the timeout handler is already canceled or
				 * pending by race. If it is canceled, the count must be decreased
				 * already, and if it is pending, the timeout handler will decrease
				 * the awaiter reference.
				 */
				if (cancel_delayed_work(&awaiter->timeout_work))
					gcip_mailbox_awaiter_dec_refs(awaiter);
				/*
				 * If `handle_awaiter_arrived` callback is defined, @awaiter
				 * will be released from the implementation side. Otherwise, it
				 * should be freed from here.
				 */
				if (mailbox->ops->handle_awaiter_arrived)
					mailbox->ops->handle_awaiter_arrived(mailbox, awaiter);
				gcip_mailbox_awaiter_dec_refs(awaiter);
			}
			kfree(cur);
			break;
		}
		if (!mailbox->ignore_seq_order && cur_seq < seq) {
			SET_RESP_ELEM_STATUS(cur->resp, GCIP_MAILBOX_STATUS_NO_RESPONSE);
			list_del(&cur->list);
			if (cur->awaiter) {
				/* Remove the reference of the arrived handler. */
				gcip_mailbox_awaiter_dec_refs(cur->awaiter);
			}
			kfree(cur);
		}
	}

	RELEASE_WAIT_LIST_LOCK(true, flags);
}

/*
 * Fetches elements in the response queue.
 *
 * Returns the pointer of fetched response elements.
 * @total_ptr will be the number of elements fetched.
 *
 * Returns -ENOMEM if failed on memory allocation.
 * Returns NULL if the response queue is empty or there is another worker fetching responses.
 */
static void *gcip_mailbox_fetch_responses(struct gcip_mailbox *mailbox, u32 *total_ptr)
{
	u32 head;
	u32 tail;
	u32 count;
	u32 i;
	u32 j;
	u32 total = 0;
	const u32 wrap_bit = mailbox->queue_wrap_bit;
	const u32 size = GET_RESP_QUEUE_SIZE();
	const u32 elem_size = mailbox->resp_elem_size;
	void *ret = NULL; /* Array of responses. */
	void *prev_ptr = NULL; /* Temporary pointer to realloc ret. */

	/* Someone is working on consuming - we can leave early. */
	if (!ACQUIRE_RESP_QUEUE_LOCK(true))
		goto out;

	head = GET_RESP_QUEUE_HEAD();
	/* Loops until our head equals to CSR tail. */
	while (1) {
		tail = GET_RESP_QUEUE_TAIL();
		/*
		 * Make sure the CSR is read and reported properly by checking if any bit higher
		 * than wrap_bit is set and if the tail exceeds resp_queue size.
		 */
		if (unlikely(tail & ~CIRC_QUEUE_VALID_MASK(wrap_bit) ||
			     CIRC_QUEUE_REAL_INDEX(tail, wrap_bit) >= size)) {
			dev_err_ratelimited(mailbox->dev, "Invalid response queue tail: %#x\n",
					    tail);
			break;
		}

		count = gcip_circ_queue_cnt(head, tail, size, wrap_bit);
		if (count == 0)
			break;

		prev_ptr = ret;
		ret = krealloc(prev_ptr, (total + count) * elem_size, GFP_KERNEL);
		/*
		 * Out-of-memory, we can return the previously fetched responses if any, or ENOMEM
		 * otherwise.
		 */
		if (!ret) {
			if (!prev_ptr)
				ret = ERR_PTR(-ENOMEM);
			else
				ret = prev_ptr;
			break;
		}
		/* Copies responses. */
		j = CIRC_QUEUE_REAL_INDEX(head, wrap_bit);
		for (i = 0; i < count; i++) {
			memcpy(ret + elem_size * total, mailbox->resp_queue + elem_size * j,
			       elem_size);
			j = (j + 1) % size;
			total++;
		}
		head = gcip_circ_queue_inc(head, count, size, wrap_bit);
	}
	INC_RESP_QUEUE_HEAD(total);

	RELEASE_RESP_QUEUE_LOCK();

	if (mailbox->ops->after_fetch_resps)
		mailbox->ops->after_fetch_resps(mailbox, total);
out:
	*total_ptr = total;
	return ret;
}

/* Fetches one response from the response queue. */
static int gcip_mailbox_fetch_one_response(struct gcip_mailbox *mailbox, void *resp)
{
	u32 head;
	u32 tail;

	if (!ACQUIRE_RESP_QUEUE_LOCK(true))
		return 0;

	head = GET_RESP_QUEUE_HEAD();
	tail = GET_RESP_QUEUE_TAIL();
	/* Queue empty. */
	if (head == tail) {
		RELEASE_RESP_QUEUE_LOCK();
		return 0;
	}

	memcpy(resp,
	       mailbox->resp_queue + CIRC_QUEUE_REAL_INDEX(head, mailbox->queue_wrap_bit) *
					     mailbox->resp_elem_size,
	       mailbox->resp_elem_size);
	INC_RESP_QUEUE_HEAD(1);

	RELEASE_RESP_QUEUE_LOCK();

	if (mailbox->ops->after_fetch_resps)
		mailbox->ops->after_fetch_resps(mailbox, 1);

	return 1;
}

/* Handles the timed out asynchronous commands. */
static void gcip_mailbox_async_cmd_timeout_work(struct work_struct *work)
{
	struct gcip_mailbox_resp_awaiter *awaiter =
		container_of(work, struct gcip_mailbox_resp_awaiter, timeout_work.work);
	struct gcip_mailbox *mailbox = awaiter->mailbox;

	/*
	 * This function will acquire the mailbox wait_list_lock. This means if
	 * response processing is in progress, it will complete before this
	 * response can be removed from the wait list.
	 *
	 * Once this function has the wait_list_lock, no future response
	 * processing will begin until this response has been removed.
	 */
	gcip_mailbox_del_wait_resp(mailbox, awaiter->resp);

	/*
	 * Handle timed out awaiter. If `handle_awaiter_timedout` is defined, @awaiter
	 * will be released from the implementation side. Otherwise, it should be freed from here.
	 */
	if (mailbox->ops->handle_awaiter_timedout)
		mailbox->ops->handle_awaiter_timedout(mailbox, awaiter);

	/* Remove the reference of the timedout handler. */
	gcip_mailbox_awaiter_dec_refs(awaiter);
}

/* Cleans up all the asynchronous responses which are not responded yet. */
static void gcip_mailbox_flush_awaiter(struct gcip_mailbox *mailbox)
{
	struct gcip_mailbox_wait_list_elem *cur, *nxt;
	struct gcip_mailbox_resp_awaiter *awaiter;
	struct list_head resps_to_flush;

	/* If mailbox->ops is NULL, the mailbox is already released. */
	if (!mailbox->ops)
		return;

	/*
	 * At this point only async responses should be pending. Flush them all
	 * from the `wait_list` at once so any remaining timeout workers
	 * waiting on `wait_list_lock` will know their responses have been
	 * handled already.
	 */
	INIT_LIST_HEAD(&resps_to_flush);
	ACQUIRE_WAIT_LIST_LOCK(false, NULL);
	list_for_each_entry_safe (cur, nxt, &mailbox->wait_list, list) {
		list_del(&cur->list);
		if (cur->awaiter) {
			list_add_tail(&cur->list, &resps_to_flush);
			/*
			 * Clear the response's destination queue so that if the
			 * timeout worker is running, it won't try to process
			 * this response after `wait_list_lock` is released.
			 */
			awaiter = cur->awaiter;
			if (mailbox->ops->flush_awaiter)
				mailbox->ops->flush_awaiter(mailbox, awaiter);
			/* Remove the reference of the arrived handler. */
			gcip_mailbox_awaiter_dec_refs(cur->awaiter);
		} else {
			dev_warn(mailbox->dev,
				 "Unexpected synchronous command pending on mailbox release\n");
			kfree(cur);
		}
	}
	RELEASE_WAIT_LIST_LOCK(false, 0);

	/*
	 * Cancel the timeout timer of and free any responses that were still in
	 * the `wait_list` above.
	 */
	list_for_each_entry_safe (cur, nxt, &resps_to_flush, list) {
		list_del(&cur->list);
		awaiter = cur->awaiter;
		/* Cancel the timeout work and remove the reference of the timedout handler. */
		gcip_mailbox_cancel_awaiter_timeout(awaiter);
		/* Remove the reference of the caller. */
		gcip_mailbox_awaiter_dec_refs(cur->awaiter);
		kfree(cur);
	}
}

/* Verifies and sets the mailbox operators. */
static int gcip_mailbox_set_ops(struct gcip_mailbox *mailbox, const struct gcip_mailbox_ops *ops)
{
	if (!ops) {
		mailbox->ops = NULL;
		return 0;
	}

	if (!ops->get_cmd_queue_head || !ops->get_cmd_queue_tail || !ops->inc_cmd_queue_tail ||
	    !ops->acquire_cmd_queue_lock || !ops->release_cmd_queue_lock ||
	    !ops->get_cmd_elem_seq || !ops->set_cmd_elem_seq || !ops->get_cmd_elem_code) {
		dev_err(mailbox->dev, "Incomplete mailbox CMD queue ops.\n");
		return -EINVAL;
	}

	if (!ops->get_resp_queue_size || !ops->get_resp_queue_head || !ops->get_resp_queue_tail ||
	    !ops->inc_resp_queue_head || !ops->acquire_resp_queue_lock ||
	    !ops->release_resp_queue_lock || !ops->get_resp_elem_seq || !ops->set_resp_elem_seq ||
	    !ops->get_resp_elem_status || !ops->set_resp_elem_status) {
		dev_err(mailbox->dev, "Incomplete mailbox RESP queue ops.\n");
		return -EINVAL;
	}

	if (!ops->acquire_wait_list_lock || !ops->release_wait_list_lock) {
		dev_err(mailbox->dev, "Incomplete mailbox wait_list ops.\n");
		return -EINVAL;
	}

	mailbox->ops = ops;

	return 0;
}

/* Sets the mailbox private data. */
static inline void gcip_mailbox_set_data(struct gcip_mailbox *mailbox, void *data)
{
	mailbox->data = data;
}

int gcip_mailbox_init(struct gcip_mailbox *mailbox, const struct gcip_mailbox_args *args)
{
	int ret;

	mailbox->dev = args->dev;
	mailbox->queue_wrap_bit = args->queue_wrap_bit;
	mailbox->cmd_queue = args->cmd_queue;
	mailbox->cmd_elem_size = args->cmd_elem_size;
	mailbox->resp_queue = args->resp_queue;
	mailbox->resp_elem_size = args->resp_elem_size;
	mailbox->timeout = args->timeout;
	mailbox->cur_seq = 0;
	mailbox->ignore_seq_order = args->ignore_seq_order;
	gcip_mailbox_set_data(mailbox, args->data);

	ret = gcip_mailbox_set_ops(mailbox, args->ops);
	if (ret)
		goto err_unset_data;

	INIT_LIST_HEAD(&mailbox->wait_list);
	init_waitqueue_head(&mailbox->wait_list_waitq);

	return 0;

err_unset_data:
	gcip_mailbox_set_data(mailbox, NULL);

	return ret;
}

void gcip_mailbox_release(struct gcip_mailbox *mailbox)
{
	gcip_mailbox_flush_awaiter(mailbox);
	gcip_mailbox_set_ops(mailbox, NULL);
	gcip_mailbox_set_data(mailbox, NULL);
}

void gcip_mailbox_consume_responses_work(struct gcip_mailbox *mailbox)
{
	void *responses;
	u32 i;
	u32 count = 0;

	/* Fetches responses and bumps resp_queue head. */
	responses = gcip_mailbox_fetch_responses(mailbox, &count);
	if (count == 0)
		return;
	if (IS_ERR(responses)) {
		dev_err(mailbox->dev, "GCIP mailbox failed on fetching responses: %ld",
			PTR_ERR(responses));
		return;
	}

	for (i = 0; i < count; i++)
		gcip_mailbox_handle_response(mailbox, responses + mailbox->resp_elem_size * i);
	/* Responses handled, wake up threads that are waiting for a response. */
	wake_up(&mailbox->wait_list_waitq);
	kfree(responses);
}

int gcip_mailbox_send_cmd(struct gcip_mailbox *mailbox, void *cmd, void *resp)
{
	int ret;

	ret = gcip_mailbox_enqueue_cmd(mailbox, cmd, resp, NULL);
	if (ret)
		return ret;

	if (!resp)
		return 0;

	ret = wait_event_timeout(mailbox->wait_list_waitq,
				 GET_RESP_ELEM_STATUS(resp) != GCIP_MAILBOX_STATUS_WAITING_RESPONSE,
				 msecs_to_jiffies(mailbox->timeout));
	if (!ret) {
		dev_dbg(mailbox->dev, "event wait timeout");
		gcip_mailbox_del_wait_resp(mailbox, resp);
		return -ETIMEDOUT;
	}
	if (GET_RESP_ELEM_STATUS(resp) != GCIP_MAILBOX_STATUS_OK) {
		dev_err(mailbox->dev, "Mailbox cmd %u response status %u", GET_CMD_ELEM_CODE(cmd),
			GET_RESP_ELEM_STATUS(resp));
		return -ENOMSG;
	}

	return 0;
}

struct gcip_mailbox_resp_awaiter *gcip_mailbox_put_cmd(struct gcip_mailbox *mailbox, void *cmd,
						       void *resp, void *data)
{
	struct gcip_mailbox_resp_awaiter *awaiter;
	int ret;

	awaiter = kzalloc(sizeof(*awaiter), GFP_KERNEL);
	if (!awaiter)
		return ERR_PTR(-ENOMEM);

	awaiter->resp = resp;
	awaiter->mailbox = mailbox;
	awaiter->data = data;
	awaiter->release_data = mailbox->ops->release_awaiter_data;
	/* 2 refs: caller (vd) and timedout handler. */
	refcount_set(&awaiter->refs, 2);

	INIT_DELAYED_WORK(&awaiter->timeout_work, gcip_mailbox_async_cmd_timeout_work);
	schedule_delayed_work(&awaiter->timeout_work, msecs_to_jiffies(mailbox->timeout));

	ret = gcip_mailbox_enqueue_cmd(mailbox, cmd, awaiter->resp, awaiter);
	if (ret)
		goto err_free_resp;

	return awaiter;

err_free_resp:
	gcip_mailbox_cancel_awaiter_timeout(awaiter);
	kfree(awaiter);
	return ERR_PTR(ret);
}

void gcip_mailbox_cancel_awaiter(struct gcip_mailbox_resp_awaiter *awaiter)
{
	gcip_mailbox_del_wait_resp(awaiter->mailbox, awaiter->resp);
	gcip_mailbox_cancel_awaiter_timeout(awaiter);
}

void gcip_mailbox_cancel_awaiter_timeout(struct gcip_mailbox_resp_awaiter *awaiter)
{
	if (cancel_delayed_work_sync(&awaiter->timeout_work))
		gcip_mailbox_awaiter_dec_refs(awaiter);
}

void gcip_mailbox_release_awaiter(struct gcip_mailbox_resp_awaiter *awaiter)
{
	gcip_mailbox_awaiter_dec_refs(awaiter);
}

void gcip_mailbox_consume_one_response(struct gcip_mailbox *mailbox, void *resp)
{
	int ret;

	/* Fetches (at most) one response. */
	ret = gcip_mailbox_fetch_one_response(mailbox, resp);
	if (!ret)
		return;

	gcip_mailbox_handle_response(mailbox, resp);

	/* Responses handled, wakes up threads that are waiting for a response. */
	wake_up(&mailbox->wait_list_waitq);
}
