/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Legacy implementation of the GXP mailbox interface.
 * This file must be used only when the kernel driver has to compile the implementation of the
 * mailbox by itself (i.e, when the target chip can't be compiled with GCIP).
 *
 * Copyright (C) 2022 Google LLC
 */

#ifndef __GXP_MAILBOX_IMPL_H__
#define __GXP_MAILBOX_IMPL_H__

#include <linux/spinlock_types.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

#include "gxp-eventfd.h"
#include "gxp-pm.h"

/*
 * Basic Buffer descriptor struct for message payloads.
 */
struct buffer_descriptor {
	/* Address in the device's virtual address space. */
	u64 address;
	/* Size in bytes. */
	u32 size;
	/* Flags can be used to indicate message type, etc. */
	u32 flags;
};

/*
 * Format used for mailbox command queues.
 */
struct gxp_command {
	/* Sequence number. Should match the corresponding response. */
	u64 seq;
	/*
	 * Identifies the type of command.
	 * Should be a value from `gxp_mailbox_command_code`
	 */
	u16 code;
	/*
	 * Priority level from 0 to 99, with 0 being the highest. Pending
	 * commands with higher priorities will be executed before lower
	 * priority ones.
	 */
	u8 priority;
	/*
	 * Insert spaces to make padding explicit. This does not affect
	 * alignment.
	 */
	u8 reserved[5];
	/* Struct describing the buffer containing the message payload */
	struct buffer_descriptor buffer_descriptor;
};

/*
 * Format used for mailbox response queues from kernel.
 */
struct gxp_response {
	/* Sequence number. Should match the corresponding command. */
	u64 seq;
	/* The status code. Either SUCCESS or an error. */
	u16 status;
	/* Padding. */
	u16 reserved;
	/* Return value, dependent on the command this responds to. */
	u32 retval;
};

/*
 * Wrapper struct for responses consumed by a thread other than the one which
 * sent the command.
 */
struct gxp_async_response {
	struct list_head list_entry;
	struct gxp_response resp;
	struct delayed_work timeout_work;
	/*
	 * If this response times out, this pointer to the owning mailbox is
	 * needed to delete this response from the list of pending responses.
	 */
	struct gxp_mailbox *mailbox;
	/* Queue to add the response to once it is complete or timed out */
	struct list_head *dest_queue;
	/*
	 * The lock that protects queue pointed to by `dest_queue`.
	 * The mailbox code also uses this lock to protect changes to the
	 * `dest_queue` pointer itself when processing this response.
	 */
	spinlock_t *dest_queue_lock;
	/* Queue of clients to notify when this response is processed */
	wait_queue_head_t *dest_queue_waitq;
	/* Specified power states vote during the command execution */
	struct gxp_power_states requested_states;
	/* gxp_eventfd to signal when the response completes. May be NULL */
	struct gxp_eventfd *eventfd;
};

struct gxp_mailbox_wait_list {
	struct list_head list;
	struct gxp_response *resp;
	bool is_async;
};

struct gxp_mailbox;
struct gxp_mailbox_args;
struct gxp_mailbox_manager;

extern const struct gxp_mailbox_args gxp_mailbox_default_args;

/* Initializes operators of @mgr to work with the legacy implementation of mailbox. */
void gxp_mailbox_init(struct gxp_mailbox_manager *mgr);

/*
 * Following functions will be called by the `gxp-mailbox.c` according to its internal logic.
 * You may not call these functions directly.
 */

/*
 * Initializes the mailbox to be able to wait and consume responses.
 * This function will be called when the `gxp_mailbox_alloc` function is called.
 */
int gxp_mailbox_init_consume_responses(struct gxp_mailbox *mailbox);

/*
 * Flushes all pending responses in the mailbox.
 * This function will be called when the `gxp_mailbox_release` function is called.
 */
void gxp_mailbox_release_consume_responses(struct gxp_mailbox *mailbox);

/*
 * Fetches and handles responses, then wakes up threads that are waiting for a response.
 * This function will be called by a worker which is scheduled in the IRQ handler. (See the
 * `gxp_mailbox_consume_responses_work` function) To prevent use-after-free or race-condition
 * bugs, gxp_mailbox_release() must be called before free the mailbox.
 */
void gxp_mailbox_consume_responses(struct gxp_mailbox *mailbox);

#endif /* __GXP_MAILBOX_IMPL_H__ */
