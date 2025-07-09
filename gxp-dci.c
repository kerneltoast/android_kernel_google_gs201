// SPDX-License-Identifier: GPL-2.0-only
/*
 * Implementation of DCI (Direct Command Interface) using mailbox.
 *
 * Copyright (C) 2022-2024 Google LLC
 */

#include <linux/kthread.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <uapi/linux/sched/types.h>

#include "gxp-dci.h"
#include "gxp-debug-dump.h"
#include "gxp-dma.h"
#include "gxp-mailbox-driver.h"
#include "gxp-pm.h"
#include "gxp.h"

#define MBOX_CMD_QUEUE_NUM_ENTRIES 1024
#define MBOX_RESP_QUEUE_NUM_ENTRIES 1024

static int gxp_dci_mailbox_manager_execute_cmd(
	struct gxp_client *client, struct gxp_mailbox *mailbox, int virt_core,
	u16 cmd_code, u8 cmd_priority, u64 cmd_daddr, u32 cmd_size,
	u32 cmd_flags, u8 num_cores, struct gxp_power_states power_states,
	u64 *resp_seq, u16 *resp_status)
{
	struct gxp_dev *gxp = client->gxp;
	struct gxp_dci_command cmd = {};
	struct gxp_dci_response resp;
	struct gxp_dci_buffer_descriptor buffer;
	int ret;

	/* Pack the command structure */
	buffer.address = cmd_daddr;
	buffer.size = cmd_size;
	buffer.flags = cmd_flags;
	/* cmd.seq is assigned by mailbox implementation */
	cmd.code = cmd_code; /* All IOCTL commands are dispatch */
	cmd.priority = cmd_priority; /* currently unused */
	cmd.buffer_descriptor = buffer;

	down_read(&gxp->vd_semaphore);
	ret = gxp_dci_execute_cmd(mailbox, &cmd, &resp);
	up_read(&gxp->vd_semaphore);

	/* resp.seq and resp.status can be updated even though it failed to process the command */
	if (resp_seq)
		*resp_seq = resp.seq;
	if (resp_status)
		*resp_status = resp.status;

	return ret;
}

static int gxp_dci_mailbox_manager_execute_cmd_async(
	struct gxp_client *client, struct gxp_mailbox *mailbox, int virt_core,
	u16 cmd_code, u8 cmd_priority, u64 cmd_daddr, u32 cmd_size,
	u32 cmd_flags, struct gxp_power_states requested_states, u64 *cmd_seq)
{
	struct gxp_dci_command cmd;
	struct gxp_dci_buffer_descriptor buffer;
	struct mailbox_resp_queue *resp_queue =
		&client->vd->mailbox_resp_queues[virt_core];
	struct gxp_eventfd *eventfd = client->mb_eventfds[virt_core];
	int ret;

	/* Pack the command structure */
	buffer.address = cmd_daddr;
	buffer.size = cmd_size;
	buffer.flags = cmd_flags;
	/* cmd.seq is assigned by mailbox implementation */
	cmd.code = cmd_code; /* All IOCTL commands are dispatch */
	cmd.priority = cmd_priority; /* currently unused */
	cmd.buffer_descriptor = buffer;

	ret = gxp_dci_execute_cmd_async(mailbox, &cmd, &resp_queue->dest_queue,
					&resp_queue->lock, &resp_queue->waitq,
					requested_states, eventfd);

	if (cmd_seq)
		*cmd_seq = cmd.seq;

	return ret;
}

static int gxp_dci_mailbox_manager_wait_async_resp(struct gxp_client *client,
						   int virt_core, u64 *resp_seq,
						   u16 *resp_status,
						   u32 *resp_retval,
						   u16 *error_code)
{
	struct gxp_dci_async_response *resp_ptr;
	struct mailbox_resp_queue *resp_queue =
		&client->vd->mailbox_resp_queues[virt_core];
	long timeout;

	spin_lock_irq(&resp_queue->lock);

	/*
	 * The "exclusive" version of wait_event is used since each wake
	 * corresponds to the addition of exactly one new response to be
	 * consumed. Therefore, only one waiting response ioctl can ever
	 * proceed per wake event.
	 */
	timeout = wait_event_interruptible_lock_irq_timeout_exclusive(
		resp_queue->waitq, !list_empty(&resp_queue->dest_queue),
		resp_queue->lock, msecs_to_jiffies(MAILBOX_TIMEOUT));
	if (timeout <= 0) {
		spin_unlock_irq(&resp_queue->lock);
		/*
		 * Unusual case - this only happens when there is no command pushed or a race with
		 * gcip_mailbox_async_cmd_timeout_work.
		 */
		return timeout ? -ETIMEDOUT : timeout;
	}
	resp_ptr = list_first_entry(&resp_queue->dest_queue,
				    struct gxp_dci_async_response, list_entry);

	/* Pop the front of the response list */
	list_del(&(resp_ptr->list_entry));

	spin_unlock_irq(&resp_queue->lock);

	if (resp_seq)
		*resp_seq = resp_ptr->resp.seq;
	if (resp_status)
		*resp_status = resp_ptr->resp.status;

	switch (resp_ptr->resp.status) {
	case GXP_DCI_RESP_OK:
		if (error_code)
			*error_code = GXP_RESPONSE_ERROR_NONE;
		/* retval is only valid if status == GXP_RESP_OK */
		if (resp_retval)
			*resp_retval = resp_ptr->resp.retval;
		break;
	case GXP_DCI_RESP_CANCELLED:
		if (error_code)
			*error_code = GXP_RESPONSE_ERROR_TIMEOUT;
		break;
	default:
		/* No other status values are valid at this point */
		WARN(true, "Completed response had invalid status %hu",
		     resp_ptr->resp.status);
		if (error_code)
			*error_code = GXP_RESPONSE_ERROR_INTERNAL;
		break;
	}

	/*
	 * We must be absolutely sure the timeout work has been cancelled
	 * and/or completed before freeing the `gxp_dci_async_response`.
	 * There are 3 possible cases when we arrive at this point:
	 *   1) The response arrived normally and the timeout was cancelled
	 *   2) The response timedout and its timeout handler finished
	 *   3) The response handler and timeout handler raced, and the response
	 *      handler "cancelled" the timeout handler while it was already in
	 *      progress.
	 *
	 * This call handles case #3, and ensures any in-process timeout
	 * handler (which may reference the `gxp_dci_async_response`) has
	 * been able to exit cleanly.
	 */
	gcip_mailbox_cancel_awaiter_timeout(resp_ptr->awaiter);
	gcip_mailbox_release_awaiter(resp_ptr->awaiter);

	return 0;
}

static void gxp_dci_mailbox_manager_release_unconsumed_async_resps(
	struct gxp_virtual_device *vd)
{
	struct gxp_dci_async_response *cur, *nxt;
	int i;
	unsigned long flags;

	/* Cleanup any unconsumed responses */
	for (i = 0; i < vd->num_cores; i++) {
		/*
		 * Since VD is releasing, it is not necessary to lock here.
		 * Do it anyway for consistency.
		 */
		spin_lock_irqsave(&vd->mailbox_resp_queues[i].lock, flags);
		list_for_each_entry_safe (
			cur, nxt, &vd->mailbox_resp_queues[i].dest_queue,
			list_entry) {
			list_del(&cur->list_entry);
			gcip_mailbox_release_awaiter(cur->awaiter);
		}
		spin_unlock_irqrestore(&vd->mailbox_resp_queues[i].lock, flags);
	}
}

static void gxp_dci_mailbox_manager_set_ops(struct gxp_mailbox_manager *mgr)
{
	mgr->allocate_mailbox = gxp_dci_alloc;
	mgr->release_mailbox = gxp_dci_release;
	mgr->reset_mailbox = gxp_mailbox_reset;
	mgr->execute_cmd = gxp_dci_mailbox_manager_execute_cmd;
	mgr->execute_cmd_async = gxp_dci_mailbox_manager_execute_cmd_async;
	mgr->wait_async_resp = gxp_dci_mailbox_manager_wait_async_resp;
	mgr->release_unconsumed_async_resps =
		gxp_dci_mailbox_manager_release_unconsumed_async_resps;
}

/* Private data structure of DCI mailbox. */
struct gxp_dci {
	struct gxp_mailbox *mbx;
	struct gxp_virtual_device *vd;
};

static u64 gxp_dci_get_cmd_elem_seq(struct gcip_mailbox *mailbox, void *cmd)
{
	struct gxp_dci_command *elem = cmd;

	return elem->seq;
}

static u32 gxp_dci_get_cmd_elem_code(struct gcip_mailbox *mailbox, void *cmd)
{
	struct gxp_dci_command *elem = cmd;

	return elem->code;
}

static void gxp_dci_set_cmd_elem_seq(struct gcip_mailbox *mailbox, void *cmd,
				     u64 seq)
{
	struct gxp_dci_command *elem = cmd;

	elem->seq = seq;
}

static u64 gxp_dci_get_resp_elem_seq(struct gcip_mailbox *mailbox, void *resp)
{
	struct gxp_dci_response *elem = resp;

	return elem->seq;
}

static void gxp_dci_set_resp_elem_seq(struct gcip_mailbox *mailbox, void *resp,
				      u64 seq)
{
	struct gxp_dci_response *elem = resp;

	elem->seq = seq;
}

static void
gxp_dci_handle_awaiter_arrived(struct gcip_mailbox *mailbox,
			       struct gcip_mailbox_resp_awaiter *awaiter)
{
	struct gxp_mailbox *mbx = mailbox->data;
	struct gxp_dci_async_response *async_resp = awaiter->data;
	unsigned long flags;

	gxp_pm_update_requested_power_states(
		mbx->gxp, async_resp->requested_states, off_states);

	spin_lock_irqsave(async_resp->dest_queue_lock, flags);

	async_resp->resp.status = GXP_DCI_RESP_OK;
	list_add_tail(&async_resp->list_entry, async_resp->dest_queue);
	/*
	 * Marking the dest_queue as NULL indicates the
	 * response was handled in case its timeout
	 * handler fired between acquiring the
	 * wait_list_lock and cancelling the timeout.
	 */
	async_resp->dest_queue = NULL;

	if (async_resp->eventfd)
		gxp_eventfd_signal(async_resp->eventfd);

	wake_up(async_resp->dest_queue_waitq);

	spin_unlock_irqrestore(async_resp->dest_queue_lock, flags);
}

static void gxp_dci_handle_awaiter_timedout(struct gcip_mailbox *mailbox,
					    struct gcip_mailbox_resp_awaiter *awaiter)
{
	struct gxp_mailbox *mbx = mailbox->data;
	struct gxp_dci *dci = mbx->data;
	struct gxp_dci_async_response *async_resp = awaiter->data;
	struct gxp_dci_response *resp = &async_resp->resp;
	struct gxp_dev *gxp = mbx->gxp;
	unsigned long flags;

	/*
	 * Check if this response still has a valid destination queue. While an in-progress call
	 * the `gxp_dci_handle_async_resp_arrived()` callback to handle the response and remove
	 * it from the wait_list with holding the wait_list_lock, the timeout can be expired and it
	 * will try to remove the response from the wait_list waiting for acquiring the
	 * wait_list_lock. If this happens, this callback will be called with the destination queue
	 * of response as a NULL, otherwise as not NULL.
	 */
	spin_lock_irqsave(async_resp->dest_queue_lock, flags);
	if (async_resp->dest_queue) {
		resp->status = GXP_DCI_RESP_CANCELLED;
		list_add_tail(&async_resp->list_entry, async_resp->dest_queue);
		spin_unlock_irqrestore(async_resp->dest_queue_lock, flags);

		/*
		 * We don't need to hold @gxp->vd_semaphore here before requesting debug dump since
		 * it is guaranteed that @dci->vd keeps holding assigned cores while its DCI mailbox
		 * is alive.
		 */

		/*
		 * Response timeout most probably would happen because of core stall. Check if
		 * forced debug dump can be requested to the participating cores for the current vd.
		 */
		gxp_debug_dump_send_forced_debug_dump_request(gxp, dci->vd);

		gxp_pm_update_requested_power_states(mbx->gxp, async_resp->requested_states,
						     off_states);

		if (async_resp->eventfd)
			gxp_eventfd_signal(async_resp->eventfd);

		wake_up(async_resp->dest_queue_waitq);
	} else {
		spin_unlock_irqrestore(async_resp->dest_queue_lock, flags);
	}
}

static void gxp_dci_handle_awaiter_flushed(struct gcip_mailbox *mailbox,
					   struct gcip_mailbox_resp_awaiter *awaiter)
{
	struct gxp_dci_async_response *async_resp = awaiter->data;
	unsigned long flags;

	spin_lock_irqsave(async_resp->dest_queue_lock, flags);
	async_resp->dest_queue = NULL;
	spin_unlock_irqrestore(async_resp->dest_queue_lock, flags);

	gcip_mailbox_release_awaiter(async_resp->awaiter);
}

static void gxp_dci_release_awaiter_data(void *data)
{
	struct gxp_dci_async_response *async_resp = data;

	if (async_resp->eventfd)
		gxp_eventfd_put(async_resp->eventfd);
	kfree(async_resp);
}

static const struct gcip_mailbox_ops gxp_dci_gcip_mbx_ops = {
	.get_cmd_queue_tail = gxp_mailbox_gcip_ops_get_cmd_queue_tail,
	.inc_cmd_queue_tail = gxp_mailbox_gcip_ops_inc_cmd_queue_tail,
	.acquire_cmd_queue_lock = gxp_mailbox_gcip_ops_acquire_cmd_queue_lock,
	.release_cmd_queue_lock = gxp_mailbox_gcip_ops_release_cmd_queue_lock,
	.get_cmd_elem_seq = gxp_dci_get_cmd_elem_seq,
	.set_cmd_elem_seq = gxp_dci_set_cmd_elem_seq,
	.get_cmd_elem_code = gxp_dci_get_cmd_elem_code,
	.get_resp_queue_size = gxp_mailbox_gcip_ops_get_resp_queue_size,
	.get_resp_queue_head = gxp_mailbox_gcip_ops_get_resp_queue_head,
	.get_resp_queue_tail = gxp_mailbox_gcip_ops_get_resp_queue_tail,
	.inc_resp_queue_head = gxp_mailbox_gcip_ops_inc_resp_queue_head,
	.acquire_resp_queue_lock = gxp_mailbox_gcip_ops_acquire_resp_queue_lock,
	.release_resp_queue_lock = gxp_mailbox_gcip_ops_release_resp_queue_lock,
	.get_resp_elem_seq = gxp_dci_get_resp_elem_seq,
	.set_resp_elem_seq = gxp_dci_set_resp_elem_seq,
	.acquire_wait_list_lock = gxp_mailbox_gcip_ops_acquire_wait_list_lock,
	.release_wait_list_lock = gxp_mailbox_gcip_ops_release_wait_list_lock,
	.wait_for_cmd_queue_not_full =
		gxp_mailbox_gcip_ops_wait_for_cmd_queue_not_full,
	.after_enqueue_cmd = gxp_mailbox_gcip_ops_after_enqueue_cmd,
	.after_fetch_resps = gxp_mailbox_gcip_ops_after_fetch_resps,
	.handle_awaiter_arrived = gxp_dci_handle_awaiter_arrived,
	.handle_awaiter_timedout = gxp_dci_handle_awaiter_timedout,
	.handle_awaiter_flushed = gxp_dci_handle_awaiter_flushed,
	.release_awaiter_data = gxp_dci_release_awaiter_data,
	.is_block_off = gxp_mailbox_gcip_ops_is_block_off,
};

static int gxp_dci_allocate_resources(struct gxp_mailbox *mailbox,
				      struct gxp_virtual_device *vd,
				      uint virt_core)
{
	int ret;

	/* Allocate and initialize the command queue */
	ret = gxp_dma_alloc_coherent_buf(
		mailbox->gxp, vd->domain,
		sizeof(struct gxp_dci_command) * MBOX_CMD_QUEUE_NUM_ENTRIES,
		GFP_KERNEL, 0, &mailbox->cmd_queue_buf);
	if (ret)
		goto err_cmd_queue;

	mailbox->cmd_queue_size = MBOX_CMD_QUEUE_NUM_ENTRIES;
	mailbox->cmd_queue_tail = 0;
	mutex_init(&mailbox->cmd_queue_lock);

	/* Allocate and initialize the response queue */
	ret = gxp_dma_alloc_coherent_buf(
		mailbox->gxp, vd->domain,
		sizeof(struct gxp_dci_response) * MBOX_RESP_QUEUE_NUM_ENTRIES,
		GFP_KERNEL, 0, &mailbox->resp_queue_buf);
	if (ret)
		goto err_resp_queue;

	mailbox->resp_queue_size = MBOX_RESP_QUEUE_NUM_ENTRIES;
	mailbox->resp_queue_head = 0;
	spin_lock_init(&mailbox->resp_queue_lock);

	/* Allocate and initialize the mailbox descriptor */
	ret = gxp_dma_alloc_coherent_buf(mailbox->gxp, vd->domain,
					 sizeof(struct gxp_mailbox_descriptor),
					 GFP_KERNEL, 0,
					 &mailbox->descriptor_buf);
	if (ret)
		goto err_descriptor;

	mailbox->descriptor =
		(struct gxp_mailbox_descriptor *)mailbox->descriptor_buf.vaddr;
	mailbox->descriptor->cmd_queue_device_addr =
		mailbox->cmd_queue_buf.dsp_addr;
	mailbox->descriptor->resp_queue_device_addr =
		mailbox->resp_queue_buf.dsp_addr;
	mailbox->descriptor->cmd_queue_size = mailbox->cmd_queue_size;
	mailbox->descriptor->resp_queue_size = mailbox->resp_queue_size;

	return 0;

err_descriptor:
	gxp_dma_free_coherent_buf(mailbox->gxp, vd->domain,
				  &mailbox->resp_queue_buf);
err_resp_queue:
	gxp_dma_free_coherent_buf(mailbox->gxp, vd->domain,
				  &mailbox->cmd_queue_buf);
err_cmd_queue:
	return ret;
}

static void gxp_dci_release_resources(struct gxp_mailbox *mailbox,
				      struct gxp_virtual_device *vd,
				      uint virt_core)
{
	gxp_dma_free_coherent_buf(mailbox->gxp, vd->domain,
				  &mailbox->cmd_queue_buf);
	gxp_dma_free_coherent_buf(mailbox->gxp, vd->domain,
				  &mailbox->resp_queue_buf);
	gxp_dma_free_coherent_buf(mailbox->gxp, vd->domain,
				  &mailbox->descriptor_buf);
}

static struct gxp_mailbox_ops gxp_dci_gxp_mbx_ops = {
	.allocate_resources = gxp_dci_allocate_resources,
	.release_resources = gxp_dci_release_resources,
	.gcip_ops.mbx = &gxp_dci_gcip_mbx_ops,
};

void gxp_dci_init(struct gxp_mailbox_manager *mgr)
{
	gxp_dci_mailbox_manager_set_ops(mgr);
}

struct gxp_mailbox *gxp_dci_alloc(struct gxp_mailbox_manager *mgr,
				  struct gxp_virtual_device *vd, uint virt_core,
				  u8 core_id)
{
	struct gxp_dci *dci;
	struct gxp_mailbox_args mbx_args = {
		.type = GXP_MBOX_TYPE_GENERAL,
		.ops = &gxp_dci_gxp_mbx_ops,
		.queue_wrap_bit = DCI_CIRCULAR_QUEUE_WRAP_BIT,
		.cmd_elem_size = sizeof(struct gxp_dci_command),
		.resp_elem_size = sizeof(struct gxp_dci_response),
	};
	struct gxp_mailbox *mbx;

	dci = kzalloc(sizeof(*dci), GFP_KERNEL);
	if (!dci)
		return ERR_PTR(-ENOMEM);
	mbx_args.data = dci;

	mbx = gxp_mailbox_alloc(mgr, vd, virt_core, core_id, &mbx_args);
	if (IS_ERR(mbx)) {
		kfree(dci);
		return mbx;
	}
	dci->mbx = mbx;
	dci->vd = gxp_vd_get(vd);
	gxp_mailbox_reinit(dci->mbx);
	gxp_mailbox_generate_device_interrupt(dci->mbx, BIT(0));

	return mbx;
}

void gxp_dci_release(struct gxp_mailbox_manager *mgr,
		     struct gxp_virtual_device *vd, uint virt_core,
		     struct gxp_mailbox *mbx)
{
	struct gxp_dci *dci = mbx->data;

	gxp_vd_put(dci->vd);
	/* Frees dci. */
	kfree(dci);
	gxp_mailbox_release(mgr, vd, virt_core, mbx);
}

int gxp_dci_execute_cmd(struct gxp_mailbox *mbx, struct gxp_dci_command *cmd,
			struct gxp_dci_response *resp)
{
	int ret;

	ret = gxp_mailbox_send_cmd(mbx, cmd, resp, 0);
	if (ret || !resp)
		return ret;

	return resp->retval;
}

int gxp_dci_execute_cmd_async(struct gxp_mailbox *mbx,
			      struct gxp_dci_command *cmd,
			      struct list_head *resp_queue,
			      spinlock_t *queue_lock,
			      wait_queue_head_t *queue_waitq,
			      struct gxp_power_states requested_states,
			      struct gxp_eventfd *eventfd)
{
	struct gxp_dci_async_response *async_resp;
	int ret;

	async_resp = kzalloc(sizeof(*async_resp), GFP_KERNEL);
	if (!async_resp)
		return -ENOMEM;

	async_resp->dest_queue = resp_queue;
	async_resp->dest_queue_lock = queue_lock;
	async_resp->dest_queue_waitq = queue_waitq;
	async_resp->requested_states = requested_states;
	if (eventfd && gxp_eventfd_get(eventfd))
		async_resp->eventfd = eventfd;
	else
		async_resp->eventfd = NULL;

	gxp_pm_update_requested_power_states(mbx->gxp, off_states,
					     requested_states);
	async_resp->awaiter = gxp_mailbox_put_cmd(mbx, cmd, &async_resp->resp, async_resp, 0);
	if (IS_ERR(async_resp->awaiter)) {
		ret = PTR_ERR(async_resp->awaiter);
		goto err_free_resp;
	}

	return 0;

err_free_resp:
	gxp_pm_update_requested_power_states(mbx->gxp, requested_states,
					     off_states);
	if (async_resp->eventfd)
		gxp_eventfd_put(async_resp->eventfd);
	kfree(async_resp);
	return ret;
}
