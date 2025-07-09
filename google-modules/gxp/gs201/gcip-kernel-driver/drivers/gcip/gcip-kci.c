// SPDX-License-Identifier: GPL-2.0-only
/*
 * Kernel Control Interface, implements the protocol between AP kernel and GCIP firmware.
 *
 * Copyright (C) 2022 Google LLC
 */

#include <linux/circ_buf.h>
#include <linux/device.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/string.h> /* memcpy */

#include <gcip/gcip-kci.h>
#include <gcip/gcip-mailbox.h>

static u32 gcip_kci_get_cmd_queue_tail(struct gcip_mailbox *mailbox)
{
	struct gcip_kci *kci = gcip_mailbox_get_data(mailbox);

	return kci->ops->get_cmd_queue_tail(kci);
}

static void gcip_kci_inc_cmd_queue_tail(struct gcip_mailbox *mailbox, u32 inc)
{
	struct gcip_kci *kci = gcip_mailbox_get_data(mailbox);

	lockdep_assert_held(&kci->cmd_queue_lock);
	kci->ops->inc_cmd_queue_tail(kci, inc);
}

static int gcip_kci_acquire_cmd_queue_lock(struct gcip_mailbox *mailbox, bool try, bool *atomic)
{
	struct gcip_kci *kci = gcip_mailbox_get_data(mailbox);

	mutex_lock(&kci->cmd_queue_lock);
	return 1;
}

static void gcip_kci_release_cmd_queue_lock(struct gcip_mailbox *mailbox)
{
	struct gcip_kci *kci = gcip_mailbox_get_data(mailbox);

	mutex_unlock(&kci->cmd_queue_lock);
}

static u64 gcip_kci_get_cmd_elem_seq(struct gcip_mailbox *mailbox, void *cmd)
{
	struct gcip_kci_command_element *elem = cmd;

	return elem->seq;
}

static u32 gcip_kci_get_cmd_elem_code(struct gcip_mailbox *mailbox, void *cmd)
{
	struct gcip_kci_command_element *elem = cmd;

	return elem->code;
}

static void gcip_kci_set_cmd_elem_seq(struct gcip_mailbox *mailbox, void *cmd, u64 seq)
{
	struct gcip_kci_command_element *elem = cmd;

	elem->seq = seq;
}

static u32 gcip_kci_get_resp_queue_size(struct gcip_mailbox *mailbox)
{
	struct gcip_kci *kci = gcip_mailbox_get_data(mailbox);

	return kci->ops->get_resp_queue_size(kci);
}

static u32 gcip_kci_get_resp_queue_head(struct gcip_mailbox *mailbox)
{
	struct gcip_kci *kci = gcip_mailbox_get_data(mailbox);

	return kci->ops->get_resp_queue_head(kci);
}

static u32 gcip_kci_get_resp_queue_tail(struct gcip_mailbox *mailbox)
{
	struct gcip_kci *kci = gcip_mailbox_get_data(mailbox);

	return kci->ops->get_resp_queue_tail(kci);
}

static void gcip_kci_inc_resp_queue_head(struct gcip_mailbox *mailbox, u32 inc)
{
	struct gcip_kci *kci = gcip_mailbox_get_data(mailbox);

	lockdep_assert_held(&kci->resp_queue_lock);
	kci->ops->inc_resp_queue_head(kci, inc);
}

static int gcip_kci_acquire_resp_queue_lock(struct gcip_mailbox *mailbox, bool try, bool *atomic)
{
	struct gcip_kci *kci = gcip_mailbox_get_data(mailbox);

	*atomic = true;

	if (try)
		return spin_trylock_irqsave(&kci->resp_queue_lock, kci->resp_queue_lock_flags);

	spin_lock_irqsave(&kci->resp_queue_lock, kci->resp_queue_lock_flags);
	return 1;
}

static void gcip_kci_release_resp_queue_lock(struct gcip_mailbox *mailbox)
{
	struct gcip_kci *kci = gcip_mailbox_get_data(mailbox);

	spin_unlock_irqrestore(&kci->resp_queue_lock, kci->resp_queue_lock_flags);
}

static u64 gcip_kci_get_resp_elem_seq(struct gcip_mailbox *mailbox, void *resp)
{
	struct gcip_kci_response_element *elem = resp;

	return elem->seq;
}

static void gcip_kci_set_resp_elem_seq(struct gcip_mailbox *mailbox, void *resp, u64 seq)
{
	struct gcip_kci_response_element *elem = resp;

	elem->seq = seq;
}

static void gcip_kci_acquire_wait_list_lock(struct gcip_mailbox *mailbox, bool irqsave,
					    unsigned long *flags)
{
	struct gcip_kci *kci = gcip_mailbox_get_data(mailbox);

	spin_lock_irqsave(&kci->wait_list_lock, *flags);
}

static void gcip_kci_release_wait_list_lock(struct gcip_mailbox *mailbox, bool irqrestore,
					    unsigned long flags)
{
	struct gcip_kci *kci = gcip_mailbox_get_data(mailbox);

	spin_unlock_irqrestore(&kci->wait_list_lock, flags);
}

static int gcip_kci_wait_for_cmd_queue_not_full(struct gcip_mailbox *mailbox)
{
	struct gcip_kci *kci = gcip_mailbox_get_data(mailbox);
	u32 tail = kci->ops->get_cmd_queue_tail(kci);
	int ret;

	ret = wait_event_timeout(kci->resp_doorbell_waitq,
				 kci->ops->get_cmd_queue_head(kci) !=
					 (tail ^ mailbox->queue_wrap_bit),
				 msecs_to_jiffies(mailbox->timeout));
	if (!ret)
		return -ETIMEDOUT;

	return 0;
}

static int gcip_kci_after_enqueue_cmd(struct gcip_mailbox *mailbox, void *cmd)
{
	struct gcip_kci *kci = gcip_mailbox_get_data(mailbox);

	kci->ops->trigger_doorbell(kci, GCIP_KCI_PUSH_CMD);

	return 0;
}

static void gcip_kci_after_fetch_resps(struct gcip_mailbox *mailbox, u32 num_resps)
{
	struct gcip_kci *kci = gcip_mailbox_get_data(mailbox);
	u32 size = kci->ops->get_resp_queue_size(kci);

	/*
	 * We consumed a lot of responses - ring the doorbell of *cmd* queue to notify the firmware,
	 * which might be waiting us to consume the response queue.
	 */
	if (num_resps >= size / 2)
		kci->ops->trigger_doorbell(kci, GCIP_KCI_CONSUME_RESP);
}

/*
 * Adds an incoming request from firmware to the circular buffer and schedules the work queue for
 * processing.
 */
static int gcip_reverse_kci_add_resp(struct gcip_kci *kci,
				     const struct gcip_kci_response_element *resp)
{
	struct gcip_reverse_kci *rkci = &kci->rkci;
	unsigned long head, tail, flags;
	int ret = 0;

	spin_lock_irqsave(&rkci->producer_lock, flags);
	head = rkci->head;
	tail = READ_ONCE(rkci->tail);
	if (CIRC_SPACE(head, tail, rkci->buffer_size) >= 1) {
		rkci->buffer[head] = *resp;
		smp_store_release(&rkci->head, (head + 1) & (rkci->buffer_size - 1));
		schedule_work(&rkci->work);
	} else {
		ret = -ENOSPC;
	}
	spin_unlock_irqrestore(&rkci->producer_lock, flags);

	return ret;
}

static bool gcip_kci_before_handle_resp(struct gcip_mailbox *mailbox, const void *resp)
{
	struct gcip_kci *kci = gcip_mailbox_get_data(mailbox);
	const struct gcip_kci_response_element *elem = resp;

	if (elem->seq & GCIP_KCI_REVERSE_FLAG) {
		int ret = gcip_reverse_kci_add_resp(kci, elem);

		if (ret)
			dev_warn_ratelimited(kci->dev,
					     "Failed to handle reverse KCI code %u (%d)\n",
					     elem->code, ret);
		return false;
	}

	return true;
}

static inline bool gcip_kci_is_block_off(struct gcip_mailbox *mailbox)
{
	struct gcip_kci *kci = gcip_mailbox_get_data(mailbox);

	return kci->ops->is_block_off ? kci->ops->is_block_off(kci) : false;
}

static void gcip_kci_on_error(struct gcip_mailbox *mailbox, int err)
{
	struct gcip_kci *kci = gcip_mailbox_get_data(mailbox);

	if (kci->ops->on_error)
		kci->ops->on_error(kci, err);
}

static const struct gcip_mailbox_ops gcip_mailbox_ops = {
	.get_cmd_queue_tail = gcip_kci_get_cmd_queue_tail,
	.inc_cmd_queue_tail = gcip_kci_inc_cmd_queue_tail,
	.acquire_cmd_queue_lock = gcip_kci_acquire_cmd_queue_lock,
	.release_cmd_queue_lock = gcip_kci_release_cmd_queue_lock,
	.get_cmd_elem_seq = gcip_kci_get_cmd_elem_seq,
	.set_cmd_elem_seq = gcip_kci_set_cmd_elem_seq,
	.get_cmd_elem_code = gcip_kci_get_cmd_elem_code,
	.get_resp_queue_size = gcip_kci_get_resp_queue_size,
	.get_resp_queue_head = gcip_kci_get_resp_queue_head,
	.get_resp_queue_tail = gcip_kci_get_resp_queue_tail,
	.inc_resp_queue_head = gcip_kci_inc_resp_queue_head,
	.acquire_resp_queue_lock = gcip_kci_acquire_resp_queue_lock,
	.release_resp_queue_lock = gcip_kci_release_resp_queue_lock,
	.get_resp_elem_seq = gcip_kci_get_resp_elem_seq,
	.set_resp_elem_seq = gcip_kci_set_resp_elem_seq,
	.acquire_wait_list_lock = gcip_kci_acquire_wait_list_lock,
	.release_wait_list_lock = gcip_kci_release_wait_list_lock,
	.wait_for_cmd_queue_not_full = gcip_kci_wait_for_cmd_queue_not_full,
	.after_enqueue_cmd = gcip_kci_after_enqueue_cmd,
	.after_fetch_resps = gcip_kci_after_fetch_resps,
	.before_handle_resp = gcip_kci_before_handle_resp,
	.is_block_off = gcip_kci_is_block_off,
	.on_error = gcip_kci_on_error,
};

/*
 * Pushes an element to cmd queue and waits for the response.
 * Returns -ETIMEDOUT if no response is received within kci->mailbox.timeout msecs.
 *
 * Returns the code of response, or a negative errno on error.
 * @resp is updated with the response, as to retrieve returned retval field.
 */
int gcip_kci_send_cmd_return_resp(struct gcip_kci *kci, struct gcip_kci_command_element *cmd,
				  struct gcip_kci_response_element *resp)
{
	int ret;
	gcip_mailbox_cmd_flags_t flags = 0;

	if (cmd->seq & GCIP_KCI_REVERSE_FLAG)
		flags |= GCIP_MAILBOX_CMD_FLAGS_SKIP_ASSIGN_SEQ;

	ret = gcip_mailbox_send_cmd(&kci->mailbox, cmd, resp, flags);
	if (ret || !resp)
		return ret;

	return resp->code;
}

int gcip_kci_send_cmd(struct gcip_kci *kci, struct gcip_kci_command_element *cmd)
{
	struct gcip_kci_response_element resp;

	/* Don't wait on a response for reverse KCI response. */
	if (cmd->seq & GCIP_KCI_REVERSE_FLAG)
		return gcip_kci_send_cmd_return_resp(kci, cmd, NULL);
	else
		return gcip_kci_send_cmd_return_resp(kci, cmd, &resp);
}

/*
 * Fetches and handles responses, then wakes up threads that are waiting for a response.
 *
 * Note: this worker is scheduled in the IRQ handler, to prevent use-after-free or race-condition
 * bugs, gcip_kci_cancel_work_queues() must be called before free the mailbox.
 */
static void gcip_kci_consume_responses_work(struct work_struct *work)
{
	struct gcip_kci *kci = container_of(work, struct gcip_kci, work);

	gcip_mailbox_consume_responses_work(&kci->mailbox);
}

/*
 * IRQ handler of KCI mailbox.
 *
 * Consumes one response (if any) and puts gcip_kci_consume_responses_work() into the system work
 * queue.
 */
void gcip_kci_handle_irq(struct gcip_kci *kci)
{
	struct gcip_kci_response_element resp;

	/* Wakes up threads that are waiting for response doorbell to be rung. */
	wake_up(&kci->resp_doorbell_waitq);

	/*
	 * Quickly consumes one response, which should be enough for usual cases, to prevent the
	 * host from being too busy to execute the scheduled work.
	 */
	gcip_mailbox_consume_one_response(&kci->mailbox, &resp);

	schedule_work(&kci->work);
}

static void gcip_kci_update_usage_work(struct work_struct *work)
{
	struct gcip_kci *kci = container_of(work, struct gcip_kci, usage_work);

	kci->ops->update_usage(kci);
}

void gcip_kci_update_usage_async(struct gcip_kci *kci)
{
	schedule_work(&kci->usage_work);
}

/* Removes one element from the circular buffer. */
static int gcip_reverse_kci_remove_resp(struct gcip_reverse_kci *rkci,
					struct gcip_kci_response_element *resp)
{
	unsigned long head, tail;
	int ret = 0;

	spin_lock(&rkci->consumer_lock);

	/*
	 * Prevents the compiler from discarding and reloading its cached value additionally forces
	 * the CPU to order against subsequent memory references.
	 * Shamelessly stolen from:
	 * [REDACTED]
	 */
	head = smp_load_acquire(&rkci->head);
	tail = rkci->tail;
	if (CIRC_CNT(head, tail, rkci->buffer_size) >= 1) {
		*resp = rkci->buffer[tail];
		tail = (tail + 1) & (rkci->buffer_size - 1);
		ret = 1;
		smp_store_release(&rkci->tail, tail);
	}
	spin_unlock(&rkci->consumer_lock);
	return ret;
}

/* Worker for incoming requests from firmware. */
static void gcip_reverse_kci_work(struct work_struct *work)
{
	struct gcip_kci_response_element resp;
	struct gcip_reverse_kci *rkci = container_of(work, struct gcip_reverse_kci, work);
	struct gcip_kci *kci = container_of(rkci, struct gcip_kci, rkci);

	while (gcip_reverse_kci_remove_resp(rkci, &resp))
		kci->ops->reverse_kci_handle_response(kci, &resp);
}

/* Initializes the Reverse KCI handler. */
static int gcip_reverse_kci_init(struct gcip_reverse_kci *rkci, struct device *dev, u32 buffer_size)
{
	if (rkci->buffer)
		return 0;

	rkci->head = 0;
	rkci->tail = 0;
	rkci->buffer_size = buffer_size;
	rkci->buffer = devm_kcalloc(dev, buffer_size, sizeof(*rkci->buffer), GFP_KERNEL);
	if (!rkci->buffer)
		return -ENOMEM;

	spin_lock_init(&rkci->producer_lock);
	spin_lock_init(&rkci->consumer_lock);
	INIT_WORK(&rkci->work, gcip_reverse_kci_work);

	return 0;
}

static void gcip_reverse_kci_reinit(struct gcip_reverse_kci *rkci)
{
	rkci->head = 0;
	rkci->tail = 0;
}

/* Verifies and sets the KCI operators. */
static int gcip_kci_set_ops(struct gcip_kci *kci, const struct gcip_kci_ops *ops)
{
	if (!ops) {
		kci->ops = NULL;
		return 0;
	}

	if (!ops->get_cmd_queue_head || !ops->get_cmd_queue_tail || !ops->inc_cmd_queue_tail) {
		dev_err(kci->dev, "Incomplete KCI CMD queue ops.\n");
		return -EINVAL;
	}

	if (!ops->get_resp_queue_size || !ops->get_resp_queue_head || !ops->get_resp_queue_tail ||
	    !ops->inc_resp_queue_head) {
		dev_err(kci->dev, "Incomplete KCI RESP queue ops.\n");
		return -EINVAL;
	}

	if (!ops->trigger_doorbell) {
		dev_err(kci->dev, "Incomplete KCI ops. Missing trigger_doorbell.\n");
		return -EINVAL;
	}

	kci->ops = ops;

	return 0;
}

/* Sets the KCI private data. */
static inline void gcip_kci_set_data(struct gcip_kci *kci, void *data)
{
	kci->data = data;
}

int gcip_kci_init(struct gcip_kci *kci, const struct gcip_kci_args *args)
{
	int ret;
	struct gcip_mailbox_args mailbox_args;

	if (kci->ops)
		return 0;

	kci->dev = args->dev;
	gcip_kci_set_data(kci, args->data);

	ret = gcip_kci_set_ops(kci, args->ops);
	if (ret)
		goto err_unset_data;

	ret = gcip_reverse_kci_init(&kci->rkci, kci->dev, args->rkci_buffer_size);
	if (ret)
		goto err_unset_ops;

	mailbox_args.dev = args->dev;
	mailbox_args.queue_wrap_bit = args->queue_wrap_bit;
	mailbox_args.cmd_queue = args->cmd_queue;
	mailbox_args.cmd_elem_size = sizeof(struct gcip_kci_command_element);
	mailbox_args.resp_queue = args->resp_queue;
	mailbox_args.resp_elem_size = sizeof(struct gcip_kci_response_element);
	mailbox_args.timeout = args->timeout;
	mailbox_args.ops = &gcip_mailbox_ops;
	mailbox_args.data = kci;

	ret = gcip_mailbox_init(&kci->mailbox, &mailbox_args);
	if (ret)
		goto err_unset_ops;

	mutex_init(&kci->cmd_queue_lock);
	spin_lock_init(&kci->resp_queue_lock);
	spin_lock_init(&kci->wait_list_lock);
	init_waitqueue_head(&kci->resp_doorbell_waitq);
	INIT_WORK(&kci->work, gcip_kci_consume_responses_work);
	INIT_WORK(&kci->usage_work, gcip_kci_update_usage_work);

	return 0;
err_unset_ops:
	gcip_kci_set_ops(kci, NULL);
err_unset_data:
	gcip_kci_set_data(kci, NULL);

	return ret;
}

void gcip_kci_reinit(struct gcip_kci *kci)
{
	gcip_reverse_kci_reinit(&kci->rkci);
}

void gcip_kci_cancel_work_queues(struct gcip_kci *kci)
{
	cancel_work_sync(&kci->usage_work);
	cancel_work_sync(&kci->work);
	cancel_work_sync(&kci->rkci.work);
}

void gcip_kci_release(struct gcip_kci *kci)
{
	kci->rkci.buffer = NULL;
	gcip_kci_set_ops(kci, NULL);
	gcip_kci_set_data(kci, NULL);

	/*
	 * Non-empty @kci->wait_list means someone (gcip_kci_send_cmd) is waiting for a response.
	 *
	 * Since this function should only be called when removing a device, it should be impossible
	 * to reach here with gcip_kci_send_cmd() is still waiting (rmmod should fail), add a simple
	 * check here so we can more easily figure it out when this happens.
	 */
	if (!list_empty(gcip_kci_get_wait_list(kci)))
		dev_warn(kci->dev, "KCI commands still pending.\n");
	gcip_mailbox_release(&kci->mailbox);
}

int gcip_kci_error_to_errno(struct device *dev, enum gcip_kci_error code)
{
	switch (code) {
	case GCIP_KCI_ERROR_OK:
		return 0;
	case GCIP_KCI_ERROR_CANCELLED:
		return -ECANCELED;
	case GCIP_KCI_ERROR_UNKNOWN:
		return -EINVAL;
	case GCIP_KCI_ERROR_INVALID_ARGUMENT:
		return -EINVAL;
	case GCIP_KCI_ERROR_DEADLINE_EXCEEDED:
		return -ETIMEDOUT;
	case GCIP_KCI_ERROR_NOT_FOUND:
		return -EIO;
	case GCIP_KCI_ERROR_ALREADY_EXISTS:
		return -EALREADY;
	case GCIP_KCI_ERROR_PERMISSION_DENIED:
		return -EPERM;
	case GCIP_KCI_ERROR_RESOURCE_EXHAUSTED:
		return -ENOSPC;
	case GCIP_KCI_ERROR_FAILED_PRECONDITION:
		return -EFAULT;
	case GCIP_KCI_ERROR_ABORTED:
		return -EACCES;
	case GCIP_KCI_ERROR_OUT_OF_RANGE:
		return -ERANGE;
	case GCIP_KCI_ERROR_UNIMPLEMENTED:
		return -EOPNOTSUPP;
	case GCIP_KCI_ERROR_INTERNAL:
		return -EBADRQC;
	case GCIP_KCI_ERROR_UNAVAILABLE:
		return -EAGAIN;
	case GCIP_KCI_ERROR_DATA_LOSS:
		return -EIO;
	case GCIP_KCI_ERROR_UNAUTHENTICATED:
		return -EPERM;
	default:
		dev_warn(dev, "Unknown KCI firmware error code: %d.\n", code);
		return -EBADRQC;
	}
}
