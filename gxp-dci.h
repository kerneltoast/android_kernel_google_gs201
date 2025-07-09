/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Implementation of DCI (Direct Command Interface) using mailbox.
 *
 * Copyright (C) 2022-2024 Google LLC
 */

#ifndef __GXP_DCI_H__
#define __GXP_DCI_H__

#include <gcip/gcip-mailbox.h>

#include "gxp-mailbox.h"
#include "gxp-vd.h"

#define DCI_CIRCULAR_QUEUE_WRAP_BIT BIT(15)

/* Basic Buffer descriptor struct for message payloads. */
struct gxp_dci_buffer_descriptor {
	/* Address in the device's virtual address space. */
	u64 address;
	/* Size in bytes. */
	u32 size;
	/* Flags can be used to indicate message type, etc. */
	u32 flags;
};

/* Format used for mailbox command queues. */
struct gxp_dci_command {
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
	struct gxp_dci_buffer_descriptor buffer_descriptor;
};

/* Format used for mailbox response queues from kernel. */
struct gxp_dci_response {
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
struct gxp_dci_async_response {
	struct list_head list_entry;
	struct gxp_dci_response resp;
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
	/* Handles arrival, timeout of async response. */
	struct gcip_mailbox_resp_awaiter *awaiter;
};

enum gxp_dci_response_status {
	GXP_DCI_RESP_OK = GCIP_MAILBOX_STATUS_OK,
	GXP_DCI_RESP_WAITING = GCIP_MAILBOX_STATUS_WAITING_RESPONSE,
	GXP_DCI_RESP_CANCELLED = GCIP_MAILBOX_STATUS_NO_RESPONSE,
};

/*
 * Initializes the DCI to be able to work with user commands.
 * Calling this function means that the device will communicate with the firmware side via DCI.
 */
void gxp_dci_init(struct gxp_mailbox_manager *mgr);

/*
 * The following functions all require their caller have locked
 * gxp->vd_semaphore for reading.
 */

/*
 * Allocates a mailbox which is initialized for DCI.
 * The mailbox must be released by calling `gxp_dci_release`.
 */
struct gxp_mailbox *gxp_dci_alloc(struct gxp_mailbox_manager *mgr,
				  struct gxp_virtual_device *vd, uint virt_core,
				  u8 core_id);

/* Releases a mailbox which is allocated by `gxp_dci_alloc`. */
void gxp_dci_release(struct gxp_mailbox_manager *mgr,
		     struct gxp_virtual_device *vd, uint virt_core,
		     struct gxp_mailbox *mailbox);

/* Executes command and get response through @resp. */
int gxp_dci_execute_cmd(struct gxp_mailbox *mailbox,
			struct gxp_dci_command *cmd,
			struct gxp_dci_response *resp);

/*
 * Executes command asynchronously.
 * The response can be obtained by waiting the `mailbox_resp_queues` of corresponding
 * virtual device of the mailbox which is a queue of `gxp_dci_async_response`.
 */
int gxp_dci_execute_cmd_async(struct gxp_mailbox *mailbox,
			      struct gxp_dci_command *cmd,
			      struct list_head *resp_queue,
			      spinlock_t *queue_lock,
			      wait_queue_head_t *queue_waitq,
			      struct gxp_power_states requested_states,
			      struct gxp_eventfd *eventfd);

#endif /* __GXP_DCI_H__ */
