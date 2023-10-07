/* SPDX-License-Identifier: GPL-2.0 */
/*
 * GXP mailbox interface.
 *
 * Copyright (C) 2020-2022 Google LLC
 */
#ifndef __GXP_MAILBOX_H__
#define __GXP_MAILBOX_H__

#include <linux/kthread.h>

#include "gxp-client.h"
#include "gxp-config.h" /* GXP_USE_LEGACY_MAILBOX */
#include "gxp-dma.h"
#include "gxp-internal.h"
#include "gxp-mailbox-manager.h"

#if !GXP_USE_LEGACY_MAILBOX
#include <gcip/gcip-kci.h>
#include <gcip/gcip-mailbox.h>
#endif

/*
 * Offset from the host mailbox interface to the device interface that needs to
 * be mapped.
 */
#if defined(CONFIG_GXP_IP_ZEBU) || defined(CONFIG_GXP_GEM5)
#define MAILBOX_DEVICE_INTERFACE_OFFSET 0x180000
#else
#define MAILBOX_DEVICE_INTERFACE_OFFSET 0x10000
#endif

#define __wait_event_lock_irq_timeout_exclusive(wq_head, condition, lock,      \
						timeout, state)                \
	___wait_event(wq_head, ___wait_cond_timeout(condition), state, 1,      \
		      timeout, spin_unlock_irq(&lock);                         \
		      __ret = schedule_timeout(__ret); spin_lock_irq(&lock))

/*
 * wait_event_interruptible_lock_irq_timeout() but set the exclusive flag.
 */
#define wait_event_interruptible_lock_irq_timeout_exclusive(                   \
	wq_head, condition, lock, timeout)                                     \
	({                                                                     \
		long __ret = timeout;                                          \
		if (!___wait_cond_timeout(condition))                          \
			__ret = __wait_event_lock_irq_timeout_exclusive(       \
				wq_head, condition, lock, timeout,             \
				TASK_INTERRUPTIBLE);                           \
		__ret;                                                         \
	})

/* Command/Response Structures */

enum gxp_mailbox_command_code {
	/* A user-space initiated dispatch message. */
	GXP_MBOX_CODE_DISPATCH = 0,
	/* A kernel initiated core suspend request. */
	GXP_MBOX_CODE_SUSPEND_REQUEST = 1,
};

enum gxp_mailbox_type {
	/*
	 * Mailbox will utilize `gcip-mailbox.h` internally.
	 * (Note: On `GXP_USE_LEGACY_MAILBOX`, it utilizes `gxp-mailbox-impl.h`
	 * instead.)
	 * Mostly will be used for handling user commands.
	 */
	GXP_MBOX_TYPE_GENERAL = 0,
	/*
	 * Mailbox will utilize `gcip-kci.h` internally.
	 * Will be used for handling kernel commands.
	 */
	GXP_MBOX_TYPE_KCI = 1,
};

enum gxp_response_status {
	GXP_RESP_OK = 0,
	GXP_RESP_WAITING = 1,
	GXP_RESP_CANCELLED = 2,
};

/* Mailbox Structures */
struct gxp_mailbox_descriptor {
	u64 cmd_queue_device_addr;
	u64 resp_queue_device_addr;
	u32 cmd_queue_size;
	u32 resp_queue_size;
};

struct gxp_mailbox;

/*
 * Defines the callback functions which are used by the mailbox.
 */
struct gxp_mailbox_ops {
	/*
	 * Allocates resources such as cmd_queue and resp_queue which are used by the mailbox.
	 * This callback will be called by the `gxp_mailbox_alloc` internally.
	 * Following variables should be set in this callback.
	 * - @mailbox->cmd_queue	: the pointer of the command queue.
	 * - @mailbox->cmd_queue_size	: the size of @mailbox->cmd_queue. (the maximum number of
	 *				  command elements.)
	 * - @mailbox->cmd_queue_tail	: the initial value of the tail of command queue.
	 * - @mailbox->resp_queue	: the pointer of the response queue.
	 * - @mailbox->resp_queue_size	: the size of @mailbox->resp_queue. (the maximum number of
	 *				  response elements.)
	 * - @mailbox->resp_queue_head	: the initial value of the head of response queue.
	 * - @mailbox->descriptor	: the pointer of the `strunct gxp_mailbox_descriptor`
	 *				  instance.
	 * - @mailbox
	 *     ->descriptor_device_addr	: the device address of @mailbox->descriptor.
	 * - @mailbox->descriptor
	 *     ->cmd_queue_device_addr	: the device address of @mailbox->cmd_queue.
	 * - @mailbox->descriptor
	 *     ->resp_queue_device_addr	: the device address of @mailbox->resp_queue.
	 * - @mailbox->descriptor
	 *     ->cmd_queue_size		: the size of @mailbox->cmd_queue.
	 * - @mailbox->descriptor
	 *     ->resp_queue_size	: the size of @mailbox->resp_queue.
	 * Context: normal.
	 */
	int (*allocate_resources)(struct gxp_mailbox *mailbox,
				  struct gxp_virtual_device *vd,
				  uint virt_core);
	/*
	 * Releases resources which are allocated by `allocate_resources`.
	 * This callback will be called by the `gxp_mailbox_release` internally.
	 * Context: normal.
	 */
	void (*release_resources)(struct gxp_mailbox *mailbox,
				  struct gxp_virtual_device *vd,
				  uint virt_core);
#if !GXP_USE_LEGACY_MAILBOX
	/*
	 * Operators which has dependency on the GCIP according to the type of mailbox.
	 * - GXP_MBOX_TYPE_GENERAL: @gcip_ops.mbx must be defined.
	 * - GXP_MBOX_TYPE_KCI: @gcip_ops.kci must be defined.
	 */
	union {
		const struct gcip_mailbox_ops *mbx;
		const struct gcip_kci_ops *kci;
	} gcip_ops;
#endif
};

struct gxp_mailbox_args {
	enum gxp_mailbox_type type;
	struct gxp_mailbox_ops *ops;
	u64 queue_wrap_bit;
	u32 cmd_elem_size;
	u32 resp_elem_size;
	bool ignore_seq_order;
	void *data;
};

#define GXP_MAILBOX_INT_BIT_COUNT 16

struct gxp_mailbox {
	uint core_id;
	struct gxp_dev *gxp;
	void __iomem *csr_reg_base;
	void __iomem *data_reg_base;

	void (*handle_irq)(struct gxp_mailbox *mailbox);
	struct work_struct *interrupt_handlers[GXP_MAILBOX_INT_BIT_COUNT];
	unsigned int interrupt_virq;
	spinlock_t cmd_tail_resp_head_lock;
	spinlock_t cmd_head_resp_tail_lock;
	struct task_struct *to_host_poll_task;
	/* Protects to_host_poll_task while it holds a sync barrier */
	struct mutex polling_lock;

	u64 queue_wrap_bit; /* warp bit for both cmd and resp queue */

	u32 cmd_elem_size; /* size of element of cmd queue */
	struct gxp_coherent_buf descriptor_buf;
	struct gxp_mailbox_descriptor *descriptor;

	struct gxp_coherent_buf cmd_queue_buf;
	u32 cmd_queue_size; /* size of cmd queue */
	u32 cmd_queue_tail; /* offset within the cmd queue */
	struct mutex cmd_queue_lock; /* protects cmd_queue */

	u32 resp_elem_size; /* size of element of resp queue */
	struct gxp_coherent_buf resp_queue_buf;
	u32 resp_queue_size; /* size of resp queue */
	u32 resp_queue_head; /* offset within the resp queue */
	struct mutex resp_queue_lock; /* protects resp_queue */

	/* commands which need to wait for responses will be added to the wait_list */
	struct mutex wait_list_lock; /* protects wait_list */
	/* to create our own realtime worker for handling responses */
	struct kthread_worker response_worker;
	struct task_struct *response_thread;
	struct kthread_work response_work;

	enum gxp_mailbox_type type;
	struct gxp_mailbox_ops *ops;
	void *data; /* private data */

	bool ignore_seq_order; /* allow out-of-order responses if true (always false in KCI) */

#if GXP_USE_LEGACY_MAILBOX
	u64 cur_seq;
	/* add to this list if a command needs to wait for a response */
	struct list_head wait_list;
	/* queue for waiting for the wait_list to be consumed */
	wait_queue_head_t wait_list_waitq;
#else /* !GXP_USE_LEGACY_MAILBOX */
	/*
	 * Implementation of the mailbox according to the type.
	 * - GXP_MBOX_TYPE_GENERAL: @gcip_mbx will be allocated.
	 * - GXP_MBOX_TYPE_KCI: @gcip_kci will be allocated.
	 */
	union {
		struct gcip_mailbox *gcip_mbx;
		struct gcip_kci *gcip_kci;
	} mbx_impl;
#endif /* GXP_USE_LEGACY_MAILBOX */
};

/* Mailbox APIs */

extern int gxp_mbx_timeout;
#define MAILBOX_TIMEOUT (gxp_mbx_timeout * GXP_TIME_DELAY_FACTOR)

/*
 * The following functions are low-level interfaces of the mailbox. The actual work of it will be
 * implemented from the high-level interfaces such as DCI, UCI and KCI via the callbacks defined
 * above. Therefore, you may not call these functions directly.
 * (Except `gxp_mailbox_{register,unregister}_interrupt_handler` functions.)
 *
 * If the mailbox interacts with virtual cores according to the implementation, the caller must
 * have locked gxp->vd_semaphore for reading.
 */

struct gxp_mailbox *gxp_mailbox_alloc(struct gxp_mailbox_manager *mgr,
				      struct gxp_virtual_device *vd,
				      uint virt_core, u8 core_id,
				      const struct gxp_mailbox_args *args);

void gxp_mailbox_release(struct gxp_mailbox_manager *mgr,
			 struct gxp_virtual_device *vd, uint virt_core,
			 struct gxp_mailbox *mailbox);

void gxp_mailbox_reset(struct gxp_mailbox *mailbox);

int gxp_mailbox_register_interrupt_handler(struct gxp_mailbox *mailbox,
					   u32 int_bit,
					   struct work_struct *handler);

int gxp_mailbox_unregister_interrupt_handler(struct gxp_mailbox *mailbox,
					     u32 int_bit);

#if !GXP_USE_LEGACY_MAILBOX
/*
 * Executes command synchronously. If @resp is not NULL, the response will be returned to it.
 * See the `gcip_mailbox_send_cmd` of `gcip-mailbox.h` or `gcip_kci_send_cmd` of `gcip-kci.h`
 * for detail.
 */
int gxp_mailbox_send_cmd(struct gxp_mailbox *mailbox, void *cmd, void *resp);

/*
 * Executes command asynchronously. The response will be written to @resp.
 * See the `gcip_mailbox_put_cmd` function of `gcip-mailbox.h` for detail.
 *
 * Note: KCI doesn't support asynchronous requests.
 */
struct gcip_mailbox_resp_awaiter *
gxp_mailbox_put_cmd(struct gxp_mailbox *mailbox, void *cmd, void *resp,
		    void *data);
#endif /* !GXP_USE_LEGACY_MAILBOX */

#endif /* __GXP_MAILBOX_H__ */
