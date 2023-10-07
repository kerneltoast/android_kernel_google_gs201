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

#include "gxp-config.h" /* GXP_USE_LEGACY_MAILBOX */
#include "gxp-dma.h"
#include "gxp-internal.h"
#include "gxp-mailbox.h"
#include "gxp-mailbox-driver.h"
#include "gxp-pm.h"
#include "gxp.h"

#if GXP_USE_LEGACY_MAILBOX
#include "gxp-mailbox-impl.h"
#else
#include <gcip/gcip-mailbox.h>
#include <gcip/gcip-kci.h>

#include "gxp-kci.h"
#include "gxp-mcu-telemetry.h"
#endif

/* Timeout of 1s by default */
int gxp_mbx_timeout = 2000;
module_param_named(mbx_timeout, gxp_mbx_timeout, int, 0660);

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

#if GXP_USE_LEGACY_MAILBOX
	gxp_mailbox_consume_responses(mailbox);
#else
	switch (mailbox->type) {
	case GXP_MBOX_TYPE_GENERAL:
		gcip_mailbox_consume_responses_work(mailbox->mbx_impl.gcip_mbx);
		break;
	case GXP_MBOX_TYPE_KCI:
		gcip_kci_handle_irq(mailbox->mbx_impl.gcip_kci);
		gxp_mcu_telemetry_irq_handler(
			((struct gxp_kci *)mailbox->data)->mcu);
		break;
	}
#endif
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
static struct task_struct *create_response_rt_thread(struct device *dev,
						     void *data, int core_id)
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

static int gxp_mailbox_set_ops(struct gxp_mailbox *mailbox,
			       struct gxp_mailbox_ops *ops)
{
	if (!ops) {
		dev_err(mailbox->gxp->dev, "Incomplete gxp_mailbox ops.\n");
		return -EINVAL;
	}

	mailbox->ops = ops;

	return 0;
}

static inline void gxp_mailbox_set_data(struct gxp_mailbox *mailbox, void *data)
{
	mailbox->data = data;
}

static struct gxp_mailbox *create_mailbox(struct gxp_mailbox_manager *mgr,
					  struct gxp_virtual_device *vd,
					  uint virt_core, u8 core_id,
					  const struct gxp_mailbox_args *args)
{
	struct gxp_mailbox *mailbox;
	int ret;

	if (!args) {
		dev_err(mgr->gxp->dev, "Incomplete gxp_mailbox args.\n");
		ret = -EINVAL;
		goto err_args;
	}

	mailbox = kzalloc(sizeof(*mailbox), GFP_KERNEL);
	if (!mailbox) {
		ret = -ENOMEM;
		goto err_mailbox;
	}

	mailbox->core_id = core_id;
	mailbox->gxp = mgr->gxp;
	mailbox->csr_reg_base = mgr->get_mailbox_csr_base(mgr->gxp, core_id);
	mailbox->data_reg_base = mgr->get_mailbox_data_base(mgr->gxp, core_id);
	mailbox->type = args->type;
	mailbox->queue_wrap_bit = args->queue_wrap_bit;
	mailbox->cmd_elem_size = args->cmd_elem_size;
	mailbox->resp_elem_size = args->resp_elem_size;
	mailbox->ignore_seq_order = args->ignore_seq_order;
	gxp_mailbox_set_data(mailbox, args->data);

	ret = gxp_mailbox_set_ops(mailbox, args->ops);
	if (ret)
		goto err_set_ops;

	ret = mailbox->ops->allocate_resources(mailbox, vd, virt_core);
	if (ret)
		goto err_allocate_resources;

	mutex_init(&mailbox->cmd_queue_lock);
	mutex_init(&mailbox->resp_queue_lock);
	kthread_init_worker(&mailbox->response_worker);
	mailbox->response_thread = create_response_rt_thread(
		mailbox->gxp->dev, &mailbox->response_worker, core_id);
	if (IS_ERR(mailbox->response_thread)) {
		ret = -ENOMEM;
		goto err_thread;
	}

	/* Initialize driver before interacting with its registers */
	gxp_mailbox_driver_init(mailbox);

	return mailbox;

err_thread:
	mailbox->ops->release_resources(mailbox, vd, virt_core);
err_allocate_resources:
err_set_ops:
	kfree(mailbox);
err_mailbox:
err_args:
	return ERR_PTR(ret);
}

static void release_mailbox(struct gxp_mailbox *mailbox,
			    struct gxp_virtual_device *vd, uint virt_core)
{
	if (IS_GXP_TEST && !mailbox)
		return;
	mailbox->ops->release_resources(mailbox, vd, virt_core);
	kthread_flush_worker(&mailbox->response_worker);
	if (mailbox->response_thread)
		kthread_stop(mailbox->response_thread);
	kfree(mailbox);
}

#if !GXP_USE_LEGACY_MAILBOX
static int init_gcip_mailbox(struct gxp_mailbox *mailbox)
{
	const struct gcip_mailbox_args args = {
		.dev = mailbox->gxp->dev,
		.queue_wrap_bit = mailbox->queue_wrap_bit,
		.cmd_queue = mailbox->cmd_queue_buf.vaddr,
		.cmd_elem_size = mailbox->cmd_elem_size,
		.resp_queue = mailbox->resp_queue_buf.vaddr,
		.resp_elem_size = mailbox->resp_elem_size,
		.timeout = MAILBOX_TIMEOUT,
		.ops = mailbox->ops->gcip_ops.mbx,
		.data = mailbox,
		.ignore_seq_order = mailbox->ignore_seq_order,
	};
	struct gcip_mailbox *gcip_mbx;
	int ret;

	gcip_mbx = kzalloc(sizeof(*gcip_mbx), GFP_KERNEL);
	if (!gcip_mbx)
		return -ENOMEM;

	/* Initialize gcip_mailbox */
	ret = gcip_mailbox_init(gcip_mbx, &args);
	if (ret) {
		kfree(gcip_mbx);
		return ret;
	}

	mailbox->mbx_impl.gcip_mbx = gcip_mbx;

	return 0;
}

static void release_gcip_mailbox(struct gxp_mailbox *mailbox)
{
	struct gcip_mailbox *gcip_mbx = mailbox->mbx_impl.gcip_mbx;

	if (gcip_mbx == NULL)
		return;

	gcip_mailbox_release(gcip_mbx);
	kfree(gcip_mbx);
	mailbox->mbx_impl.gcip_mbx = NULL;
}

static int init_gcip_kci(struct gxp_mailbox *mailbox)
{
	const struct gcip_kci_args args = {
		.dev = mailbox->gxp->dev,
		.cmd_queue = mailbox->cmd_queue_buf.vaddr,
		.resp_queue = mailbox->resp_queue_buf.vaddr,
		.queue_wrap_bit = mailbox->queue_wrap_bit,
		.rkci_buffer_size = GXP_REVERSE_KCI_BUFFER_SIZE,
		.timeout = GXP_KCI_TIMEOUT,
		.ops = mailbox->ops->gcip_ops.kci,
		.data = mailbox,
	};
	struct gcip_kci *gcip_kci;
	int ret;

	gcip_kci = kzalloc(sizeof(*gcip_kci), GFP_KERNEL);
	if (!gcip_kci)
		return -ENOMEM;

	ret = gcip_kci_init(gcip_kci, &args);
	if (ret) {
		kfree(gcip_kci);
		return ret;
	}

	mailbox->mbx_impl.gcip_kci = gcip_kci;

	return 0;
}

static void release_gcip_kci(struct gxp_mailbox *mailbox)
{
	struct gcip_kci *gcip_kci = mailbox->mbx_impl.gcip_kci;

	if (gcip_kci == NULL)
		return;

	gcip_kci_cancel_work_queues(gcip_kci);
	gcip_kci_release(gcip_kci);
	kfree(gcip_kci);
	mailbox->mbx_impl.gcip_kci = NULL;
}
#endif /* !GXP_USE_LEGACY_MAILBOX */

/*
 * Initializes @mailbox->mbx_impl to start waiting and consuming responses.
 * This will initializes GCIP mailbox modules according to the type of @mailbox.
 * - GENERAL: will initialize @mailbox->mbx_impl.gcip_mbx
 * - KCI: will initialize @mailbox->mbx_impl.kci_mbx
 *
 * Note: On `GXP_USE_LEGACY_MAILBOX`, it will initialize @mailbox itself as its
 * queuing logic is implemented in `gxp-mailbox-impl.c`.
 */
static int init_mailbox_impl(struct gxp_mailbox *mailbox)
{
	int ret;

#if GXP_USE_LEGACY_MAILBOX
	if (mailbox->type != GXP_MBOX_TYPE_GENERAL)
		return -EOPNOTSUPP;

	ret = gxp_mailbox_init_consume_responses(mailbox);
	if (ret)
		return ret;
#else
	switch (mailbox->type) {
	case GXP_MBOX_TYPE_GENERAL:
		ret = init_gcip_mailbox(mailbox);
		if (ret)
			return ret;
		break;
	case GXP_MBOX_TYPE_KCI:
		ret = init_gcip_kci(mailbox);
		if (ret)
			return ret;
		break;
	default:
		return -EOPNOTSUPP;
	}
#endif /* GXP_USE_LEGACY_MAILBOX */

	return 0;
}

static int enable_mailbox(struct gxp_mailbox *mailbox)
{
	int ret;

	gxp_mailbox_write_descriptor(mailbox, mailbox->descriptor_buf.dsp_addr);
	gxp_mailbox_write_cmd_queue_head(mailbox, 0);
	gxp_mailbox_write_cmd_queue_tail(mailbox, 0);
	gxp_mailbox_write_resp_queue_head(mailbox, 0);
	gxp_mailbox_write_resp_queue_tail(mailbox, 0);

	ret = init_mailbox_impl(mailbox);
	if (ret)
		return ret;

	mailbox->handle_irq = gxp_mailbox_handle_irq;
	mutex_init(&mailbox->wait_list_lock);
	kthread_init_work(&mailbox->response_work,
			  gxp_mailbox_consume_responses_work);

	/* Only enable interrupts once everything has been setup */
	gxp_mailbox_driver_enable_interrupts(mailbox);
	/* Enable the mailbox */
	gxp_mailbox_write_status(mailbox, 1);

	return 0;
}

struct gxp_mailbox *gxp_mailbox_alloc(struct gxp_mailbox_manager *mgr,
				      struct gxp_virtual_device *vd,
				      uint virt_core, u8 core_id,
				      const struct gxp_mailbox_args *args)
{
	struct gxp_mailbox *mailbox;
	int ret;

	mailbox = create_mailbox(mgr, vd, virt_core, core_id, args);
	if (IS_ERR(mailbox))
		return mailbox;

	ret = enable_mailbox(mailbox);
	if (ret) {
		release_mailbox(mailbox, vd, virt_core);
		return ERR_PTR(ret);
	}

	return mailbox;
}

/*
 * Releases the @mailbox->mbx_impl to flush all pending responses in the wait
 * list.
 * This releases GCIP mailbox modules according to the type of @mailbox.
 * - GENERAL: will release @mailbox->mbx_impl.gcip_mbx
 * - KCI: will release @mailbox->mbx_impl.kci_mbx
 *
 * Note: On `GXP_USE_LEGACY_MAILBOX`, it will release @mailbox itself as its
 * queuing logic is implemented in `gxp-mailbox-impl.c`.
 */
static void release_mailbox_impl(struct gxp_mailbox *mailbox)
{
#if GXP_USE_LEGACY_MAILBOX
	gxp_mailbox_release_consume_responses(mailbox);
#else
	switch (mailbox->type) {
	case GXP_MBOX_TYPE_GENERAL:
		release_gcip_mailbox(mailbox);
		break;
	case GXP_MBOX_TYPE_KCI:
		release_gcip_kci(mailbox);
		break;
	}
#endif
}

void gxp_mailbox_release(struct gxp_mailbox_manager *mgr,
			 struct gxp_virtual_device *vd, uint virt_core,
			 struct gxp_mailbox *mailbox)
{
	int i;

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

	release_mailbox_impl(mailbox);

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
	release_mailbox(mailbox, vd, virt_core);
}

void gxp_mailbox_reset(struct gxp_mailbox *mailbox)
{
	dev_notice(mailbox->gxp->dev, "%s not yet implemented\n", __func__);
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

#if !GXP_USE_LEGACY_MAILBOX
int gxp_mailbox_send_cmd(struct gxp_mailbox *mailbox, void *cmd, void *resp)
{
	switch (mailbox->type) {
	case GXP_MBOX_TYPE_GENERAL:
		return gcip_mailbox_send_cmd(mailbox->mbx_impl.gcip_mbx, cmd,
					     resp);
	case GXP_MBOX_TYPE_KCI:
		return gcip_kci_send_cmd(mailbox->mbx_impl.gcip_kci, cmd);
	}
	return -EOPNOTSUPP;
}

struct gcip_mailbox_resp_awaiter *
gxp_mailbox_put_cmd(struct gxp_mailbox *mailbox, void *cmd, void *resp,
		    void *data)
{
	switch (mailbox->type) {
	case GXP_MBOX_TYPE_GENERAL:
		return gcip_mailbox_put_cmd(mailbox->mbx_impl.gcip_mbx, cmd,
					    resp, data);
	default:
		break;
	}
	return ERR_PTR(-EOPNOTSUPP);
}
#endif /* !GXP_USE_LEGACY_MAILBOX */
