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
#define TEST_FLUSH_TIMEOUT_RACE(awaiter) gcip_mailbox_controller_flush_timeout_race(awaiter)
#define TEST_WAIT_FIRMWARE_WORK() gcip_mailbox_controller_wait_firmware_work()
#else
#define TEST_TRIGGER_TIMEOUT_RACE(...)
#define TEST_FLUSH_TIMEOUT_RACE(...)
#define TEST_WAIT_FIRMWARE_WORK(...)
#endif

#define GET_CMD_QUEUE_TAIL() mailbox->ops->get_cmd_queue_tail(mailbox)
#define INC_CMD_QUEUE_TAIL(inc) mailbox->ops->inc_cmd_queue_tail(mailbox, inc)
#define ACQUIRE_CMD_QUEUE_LOCK(try, atomic)                                                        \
	mailbox->ops->acquire_cmd_queue_lock(mailbox, try, atomic)
#define RELEASE_CMD_QUEUE_LOCK() mailbox->ops->release_cmd_queue_lock(mailbox)

#define GET_CMD_ELEM_SEQ(cmd) mailbox->ops->get_cmd_elem_seq(mailbox, cmd)
#define SET_CMD_ELEM_SEQ(cmd, seq) mailbox->ops->set_cmd_elem_seq(mailbox, cmd, seq)
#define GET_CMD_ELEM_CODE(cmd) mailbox->ops->get_cmd_elem_code(mailbox, cmd)

#define GET_RESP_QUEUE_SIZE() mailbox->ops->get_resp_queue_size(mailbox)
#define GET_RESP_QUEUE_HEAD() mailbox->ops->get_resp_queue_head(mailbox)
#define INC_RESP_QUEUE_HEAD(inc) mailbox->ops->inc_resp_queue_head(mailbox, inc)
#define GET_RESP_QUEUE_TAIL() mailbox->ops->get_resp_queue_tail(mailbox)
#define ACQUIRE_RESP_QUEUE_LOCK(try, atomic)                                                       \
	mailbox->ops->acquire_resp_queue_lock(mailbox, try, atomic)
#define RELEASE_RESP_QUEUE_LOCK() mailbox->ops->release_resp_queue_lock(mailbox)

#define GET_RESP_ELEM_SEQ(resp) mailbox->ops->get_resp_elem_seq(mailbox, resp)
#define SET_RESP_ELEM_SEQ(resp, seq) mailbox->ops->set_resp_elem_seq(mailbox, resp, seq)

#define ACQUIRE_WAIT_LIST_LOCK(irqsave, flags)                                                     \
	mailbox->ops->acquire_wait_list_lock(mailbox, irqsave, flags)
#define RELEASE_WAIT_LIST_LOCK(irqrestore, flags)                                                  \
	mailbox->ops->release_wait_list_lock(mailbox, irqrestore, flags)

#define IS_BLOCK_OFF() (mailbox->ops->is_block_off ? mailbox->ops->is_block_off(mailbox) : false)

struct gcip_mailbox_wait_list_elem {
	struct list_head list;
	struct gcip_mailbox_async_resp *async_resp;
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
 *
 * Returns true if this function deletes @async_resp from the wait list successfully. If it returns
 * false, it means that a response has been arrived or the `gcip_mailbox_flush_awaiter` function
 * has flushed all pending awaiters and @async_resp has been removed from the wait list by the race
 * condition.
 */
static bool gcip_mailbox_del_wait_resp(struct gcip_mailbox *mailbox,
				       struct gcip_mailbox_async_resp *async_resp)
{
	struct gcip_mailbox_wait_list_elem *cur;
	unsigned long flags;
	u64 cur_seq, seq = GET_RESP_ELEM_SEQ(async_resp->resp);
	bool removed = false;

	ACQUIRE_WAIT_LIST_LOCK(true, &flags);

	list_for_each_entry (cur, &mailbox->wait_list, list) {
		cur_seq = GET_RESP_ELEM_SEQ(cur->async_resp->resp);
		if (cur_seq == seq) {
			list_del(&cur->list);
			if (cur->awaiter) {
				/* Remove the reference of the arrived handler. */
				gcip_mailbox_awaiter_dec_refs(cur->awaiter);
			}
			kfree(cur);
			removed = true;
			break;
		}
	}

	RELEASE_WAIT_LIST_LOCK(true, flags);

	return removed;
}

/*
 * Adds @resp to @mailbox->wait_list. If @awaiter is not NULL, the @resp is asynchronous.
 * Otherwise, the @resp is synchronous.
 *
 * wait_list is a FIFO queue, with sequence number in increasing order.
 *
 * Returns 0 on success, or -ENOMEM if failed on allocation.
 */
static int gcip_mailbox_push_wait_resp(struct gcip_mailbox *mailbox,
				       struct gcip_mailbox_async_resp *async_resp,
				       struct gcip_mailbox_resp_awaiter *awaiter, bool atomic)
{
	struct gcip_mailbox_wait_list_elem *entry;
	unsigned long flags;
	int ret;

	entry = kzalloc(sizeof(*entry), atomic ? GFP_ATOMIC : GFP_KERNEL);
	if (!entry)
		return -ENOMEM;

	if (mailbox->ops->before_enqueue_wait_list) {
		ret = mailbox->ops->before_enqueue_wait_list(mailbox, async_resp->resp, awaiter);
		if (ret) {
			kfree(entry);
			return ret;
		}
	}

	/* Increase a reference of arrived handler. */
	if (awaiter)
		refcount_inc(&awaiter->refs);

	entry->async_resp = async_resp;
	entry->awaiter = awaiter;
	ACQUIRE_WAIT_LIST_LOCK(true, &flags);
	list_add_tail(&entry->list, &mailbox->wait_list);
	RELEASE_WAIT_LIST_LOCK(true, flags);

	return 0;
}

/* The locked version of gcip_mailbox_inc_seq_num */
static uint gcip_mailbox_inc_seq_num_locked(struct gcip_mailbox *mailbox, uint n)
{
	uint ret = mailbox->cur_seq;

	mailbox->cur_seq += n;

	return ret;
}

/*
 * Pushes @cmd to the command queue of mailbox and returns. @resp should be passed if the request
 * is synchronous and want to get the response. If @resp is NULL even though the request is
 * synchronous, the @cmd will be put into the queue, but the caller may not wait the response and
 * ignore it. If the request is async, @awaiter should be passed too.
 */
static int gcip_mailbox_enqueue_cmd(struct gcip_mailbox *mailbox, void *cmd,
				    struct gcip_mailbox_async_resp *async_resp,
				    struct gcip_mailbox_resp_awaiter *awaiter,
				    gcip_mailbox_cmd_flags_t flags)
{
	int ret = 0;
	bool atomic = false;

	ACQUIRE_CMD_QUEUE_LOCK(false, &atomic);

	if (!(flags & GCIP_MAILBOX_CMD_FLAGS_SKIP_ASSIGN_SEQ))
		SET_CMD_ELEM_SEQ(cmd, mailbox->cur_seq);
	/* Wait until the cmd queue has a space for putting cmd. */
	ret = mailbox->ops->wait_for_cmd_queue_not_full(mailbox);
	if (ret)
		goto out;

	if (async_resp->resp) {
		/* Adds @resp to the wait_list only if the cmd can be pushed successfully. */
		SET_RESP_ELEM_SEQ(async_resp->resp, GET_CMD_ELEM_SEQ(cmd));
		async_resp->status = GCIP_MAILBOX_STATUS_WAITING_RESPONSE;
		ret = gcip_mailbox_push_wait_resp(mailbox, async_resp, awaiter, atomic);
		if (ret)
			goto out;
	}

	/* Size of cmd_queue is a multiple of mailbox->cmd_elem_size. */
	memcpy(mailbox->cmd_queue +
		       mailbox->cmd_elem_size *
			       CIRC_QUEUE_REAL_INDEX(GET_CMD_QUEUE_TAIL(), mailbox->queue_wrap_bit),
	       cmd, mailbox->cmd_elem_size);
	INC_CMD_QUEUE_TAIL(1);
	if (mailbox->ops->after_enqueue_cmd) {
		ret = mailbox->ops->after_enqueue_cmd(mailbox, cmd);
		if (ret) {
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
	}

	if (!(flags & GCIP_MAILBOX_CMD_FLAGS_SKIP_ASSIGN_SEQ))
		gcip_mailbox_inc_seq_num_locked(mailbox, 1);

out:
	RELEASE_CMD_QUEUE_LOCK();
	if (ret)
		dev_dbg(mailbox->dev, "%s: ret=%d", __func__, ret);

	return ret;
}

static bool does_response_match_waiter(struct gcip_mailbox *mailbox, void *incoming_resp,
				       void *waiting_resp)
{
	if (mailbox->ops->does_response_match_waiter)
		return mailbox->ops->does_response_match_waiter(mailbox, incoming_resp,
								waiting_resp);

	return GET_RESP_ELEM_SEQ(incoming_resp) == GET_RESP_ELEM_SEQ(waiting_resp);
}

/*
 * Handler of a response.
 * Pops the wait_list until the sequence number of @resp is found, and copies @resp to the found
 * entry.
 */
static void gcip_mailbox_handle_response(struct gcip_mailbox *mailbox, void *resp)
{
	struct gcip_mailbox_wait_list_elem *cur, *nxt;
	struct gcip_mailbox_resp_awaiter *awaiter = NULL;
	unsigned long flags;

	/* If before_handle_resp is defined and it returns false, don't handle the response */
	if (mailbox->ops->before_handle_resp && !mailbox->ops->before_handle_resp(mailbox, resp))
		return;

	ACQUIRE_WAIT_LIST_LOCK(true, &flags);

	list_for_each_entry_safe (cur, nxt, &mailbox->wait_list, list) {
		if (!does_response_match_waiter(mailbox, resp, cur->async_resp->resp))
			continue;
		cur->async_resp->status = GCIP_MAILBOX_STATUS_OK;
		memcpy(cur->async_resp->resp, resp, mailbox->resp_elem_size);
		list_del(&cur->list);
		awaiter = cur->awaiter;
		if (awaiter) {
			/*
			 * The timedout handler will be fired, but pended by waiting for acquiring
			 * the wait_list_lock.
			 */
			TEST_TRIGGER_TIMEOUT_RACE(awaiter);
		}
		kfree(cur);
		break;
	}

	RELEASE_WAIT_LIST_LOCK(true, flags);

	if (!awaiter)
		return;

	/*
	 * If canceling timeout_work succeeded, we have to decrease the reference count here because
	 * the timeout handler will not be called. Otherwise, the timeout handler is already
	 * canceled or pending by race. If it is canceled, the count must be decreased already, and
	 * if it is pending, the timeout handler will decrease the awaiter reference.
	 */
	if (cancel_delayed_work(&awaiter->timeout_work))
		gcip_mailbox_awaiter_dec_refs(awaiter);
	if (mailbox->ops->handle_awaiter_arrived)
		mailbox->ops->handle_awaiter_arrived(mailbox, awaiter);

	/* Make sure the timedout handler is finished before decreasing the ref count. */
	TEST_FLUSH_TIMEOUT_RACE(awaiter);

	/* Remove the reference of the arrived handler. */
	gcip_mailbox_awaiter_dec_refs(awaiter);
}

/*
 * Fetches elements in the response queue.
 *
 * Returns the pointer of fetched response elements.
 * @total_ptr will be the number of elements fetched.
 *
 * If @trylock is true, the function will return right away if the lock is held by others which
 * means that the response queue is being consumed by other threads. Otherwise, it will use the
 * normal lock to guarantee that all responses have been handled when the function returns.
 *
 * Returns -ENOMEM if failed on memory allocation.
 * Returns NULL if the response queue is empty or there is another worker fetching responses.
 */
static void *gcip_mailbox_fetch_responses(struct gcip_mailbox *mailbox, u32 *total_ptr,
					  bool trylock)
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
	bool atomic = false;

	/* The block is off or someone is working on consuming - we can leave early. */
	if (IS_BLOCK_OFF() || !ACQUIRE_RESP_QUEUE_LOCK(trylock, &atomic))
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
		ret = krealloc(prev_ptr, (total + count) * elem_size,
			       atomic ? GFP_ATOMIC : GFP_KERNEL);
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
	bool atomic;

	if (IS_BLOCK_OFF() || !ACQUIRE_RESP_QUEUE_LOCK(true, &atomic))
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
	bool removed;

	/*
	 * This function returns true if @awaiter has been removed from the wait list successfully.
	 * It means that it is safe to process @awaiter as timeout. (i.e., there won't be any race
	 * cases that @awaiter has been processed as arrived or flushed at the same time.)
	 */
	removed = gcip_mailbox_del_wait_resp(mailbox, &awaiter->async_resp);

	/*
	 * Handle timed out awaiter. If `handle_awaiter_timedout` is defined, @awaiter
	 * will be released from the implementation side. Otherwise, it should be freed from here.
	 */
	if (removed && mailbox->ops->handle_awaiter_timedout)
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
	unsigned long flags;

	/* If mailbox->ops is NULL, the mailbox is already released. */
	if (!mailbox->ops)
		return;

	/* Tests cases that responses arrived or timedout while flushing awaiters. */
	TEST_WAIT_FIRMWARE_WORK();

	/*
	 * At this point only async responses should be pending. Flush them all
	 * from the `wait_list` at once so any remaining timeout workers
	 * waiting on `wait_list_lock` will know their responses have been
	 * handled already.
	 */
	INIT_LIST_HEAD(&resps_to_flush);
	ACQUIRE_WAIT_LIST_LOCK(true, &flags);
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
			/* Remove the reference of the arrived handler. */
			gcip_mailbox_awaiter_dec_refs(cur->awaiter);
		} else {
			dev_warn(mailbox->dev,
				 "Unexpected synchronous command pending on mailbox release\n");
			kfree(cur);
		}
	}
	RELEASE_WAIT_LIST_LOCK(true, flags);

	/*
	 * From now on, since awaiters in @resps_to_flush list have been removed from @wait_list,
	 * it is guaranteed that there will be no race condition of calling arrived or timedout
	 * handlers with flushed handler at the same time.
	 */

	/*
	 * Cancel the timeout timer of and free any responses that were still in
	 * the `wait_list` above.
	 */
	list_for_each_entry_safe (cur, nxt, &resps_to_flush, list) {
		list_del(&cur->list);
		awaiter = cur->awaiter;
		/* Cancels the timeout work as it doesn't have any meaning for flushed awaiters. */
		gcip_mailbox_cancel_awaiter_timeout(awaiter);
		if (mailbox->ops->handle_awaiter_flushed)
			mailbox->ops->handle_awaiter_flushed(mailbox, awaiter);
		else
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

	if (!ops->get_cmd_queue_tail || !ops->inc_cmd_queue_tail || !ops->acquire_cmd_queue_lock ||
	    !ops->release_cmd_queue_lock || !ops->get_cmd_elem_seq || !ops->set_cmd_elem_seq ||
	    !ops->get_cmd_elem_code || !ops->wait_for_cmd_queue_not_full) {
		dev_err(mailbox->dev, "Incomplete mailbox CMD queue ops.\n");
		return -EINVAL;
	}

	if (!ops->get_resp_queue_size || !ops->get_resp_queue_head || !ops->get_resp_queue_tail ||
	    !ops->inc_resp_queue_head || !ops->acquire_resp_queue_lock ||
	    !ops->release_resp_queue_lock || !ops->get_resp_elem_seq || !ops->set_resp_elem_seq) {
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

static void gcip_mailbox_do_consume_responses(struct gcip_mailbox *mailbox, bool trylock)
{
	void *responses;
	u32 i;
	u32 count = 0;

	/* Fetches responses and bumps resp_queue head. */
	responses = gcip_mailbox_fetch_responses(mailbox, &count, trylock);
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

void gcip_mailbox_consume_responses_work(struct gcip_mailbox *mailbox)
{
	gcip_mailbox_do_consume_responses(mailbox, true);
}

void gcip_mailbox_consume_responses(struct gcip_mailbox *mailbox)
{
	gcip_mailbox_do_consume_responses(mailbox, false);
}

int gcip_mailbox_send_cmd(struct gcip_mailbox *mailbox, void *cmd, void *resp,
			  gcip_mailbox_cmd_flags_t flags)
{
	struct gcip_mailbox_async_resp async_resp = {
		.resp = resp,
	};
	int ret;

	ret = gcip_mailbox_enqueue_cmd(mailbox, cmd, &async_resp, NULL, flags);
	if (ret)
		goto err;

	/*
	 * If @resp is NULL, it will not enqueue the response into the waiting list. Therefore, it
	 * is fine to release @async_resp.
	 */
	if (!resp)
		return 0;

	ret = wait_event_timeout(mailbox->wait_list_waitq,
				 async_resp.status != GCIP_MAILBOX_STATUS_WAITING_RESPONSE,
				 msecs_to_jiffies(mailbox->timeout));
	if (!ret) {
		dev_dbg(mailbox->dev, "event wait timeout");
		gcip_mailbox_del_wait_resp(mailbox, &async_resp);
		ret = -ETIMEDOUT;
		goto err;
	}
	if (async_resp.status != GCIP_MAILBOX_STATUS_OK) {
		dev_err(mailbox->dev, "Mailbox cmd %u response status %u", GET_CMD_ELEM_CODE(cmd),
			async_resp.status);
		ret = -ENOMSG;
		goto err;
	}

	return 0;

err:
	if (mailbox->ops->on_error)
		mailbox->ops->on_error(mailbox, ret);

	return ret;
}

struct gcip_mailbox_resp_awaiter *gcip_mailbox_put_cmd_flags(struct gcip_mailbox *mailbox,
							     void *cmd, void *resp, void *data,
							     gcip_mailbox_cmd_flags_t flags)
{
	struct gcip_mailbox_resp_awaiter *awaiter;
	int ret;
	u32 timeout = mailbox->timeout;

	awaiter = kzalloc(sizeof(*awaiter), GFP_KERNEL);
	if (!awaiter)
		return ERR_PTR(-ENOMEM);

	if (mailbox->ops->get_cmd_timeout)
		timeout = mailbox->ops->get_cmd_timeout(mailbox, cmd, resp, data);

	awaiter->async_resp.resp = resp;
	awaiter->mailbox = mailbox;
	awaiter->data = data;
	awaiter->release_data = mailbox->ops->release_awaiter_data;
	/* 2 refs: caller (vd) and timedout handler. */
	refcount_set(&awaiter->refs, 2);

	INIT_DELAYED_WORK(&awaiter->timeout_work, gcip_mailbox_async_cmd_timeout_work);
	schedule_delayed_work(&awaiter->timeout_work, msecs_to_jiffies(timeout));

	ret = gcip_mailbox_enqueue_cmd(mailbox, cmd, &awaiter->async_resp, awaiter, flags);
	if (ret)
		goto err_free_resp;

	return awaiter;

err_free_resp:
	gcip_mailbox_cancel_awaiter_timeout(awaiter);
	kfree(awaiter);
	return ERR_PTR(ret);
}

struct gcip_mailbox_resp_awaiter *gcip_mailbox_put_cmd(struct gcip_mailbox *mailbox, void *cmd,
						       void *resp, void *data)
{
	return gcip_mailbox_put_cmd_flags(mailbox, cmd, resp, data, 0);
}

bool gcip_mailbox_cancel_awaiter(struct gcip_mailbox_resp_awaiter *awaiter)
{
	bool removed;

	removed = gcip_mailbox_del_wait_resp(awaiter->mailbox, &awaiter->async_resp);
	gcip_mailbox_cancel_awaiter_timeout(awaiter);

	return removed;
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

uint gcip_mailbox_inc_seq_num(struct gcip_mailbox *mailbox, uint n)
{
	bool atomic = false;
	uint ret;

	ACQUIRE_CMD_QUEUE_LOCK(false, &atomic);

	ret = gcip_mailbox_inc_seq_num_locked(mailbox, n);

	RELEASE_CMD_QUEUE_LOCK();

	return ret;
}
