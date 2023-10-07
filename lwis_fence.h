/*
 * Google LWIS Fence
 *
 * Copyright (c) 2022 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef LWIS_FENCE_H_
#define LWIS_FENCE_H_

#include <linux/hashtable.h>
#include <linux/list.h>

#include "lwis_device.h"

#define LWIS_CLIENTS_HASH_BITS 8

extern bool lwis_fence_debug;

struct lwis_fence {
	int fd;
	int status;
	spinlock_t lock;
	/* Top device for printing logs */
	struct lwis_device *lwis_top_dev;
	/* Status wait queue for waking up userspace */
	wait_queue_head_t status_wait_queue;
	/* Hash table of transactions that's triggered by this fence */
	DECLARE_HASHTABLE(transaction_list, LWIS_CLIENTS_HASH_BITS);
};

struct lwis_fence_trigger_transaction_list {
	struct lwis_client *owner;
	struct list_head list;
	struct hlist_node node;
};

struct lwis_fence_pending_signal {
	struct file *fp;
	struct lwis_fence *fence;
	int pending_status;
	struct list_head node;
};

/*
 *  lwis_fence_create: Create a new lwis_fence.
 */
int lwis_fence_create(struct lwis_device *lwis_dev);

int ioctl_lwis_fence_create(struct lwis_device *lwis_dev, int32_t __user *msg);

/*
 *  lwis_fence_get: Get the lwis_fence associated with the fd.
 */
struct lwis_device *lwis_fence_get(int fd);

/* Creates all fences that do not currently exist */
int lwis_initialize_transaction_fences(struct lwis_client *client,
				       struct lwis_transaction *transaction);

bool lwis_triggered_by_condition(struct lwis_transaction *transaction);

bool lwis_event_triggered_condition_ready(struct lwis_transaction *transaction,
					  struct lwis_transaction *weak_transaction,
					  int64_t event_id, int64_t event_counter);

bool lwis_fence_triggered_condition_ready(struct lwis_transaction *transaction, int fence_status);

/*
 *  lwis_parse_trigger_condition: Add the transaction to the associated trigger
 *  fence and event lists.
 */
int lwis_parse_trigger_condition(struct lwis_client *client, struct lwis_transaction *transaction);

/*
 *  lwis_fence_signal: Signals the lwis_fence with the provided error code.
 */
int lwis_fence_signal(struct lwis_fence *lwis_fence, int status);

/*
 *  lwis_add_completion_fence: Adds the fence with the given fd as a completion fence to this transaction.
 */
int lwis_add_completion_fence(struct lwis_client *client, struct lwis_transaction *transaction);

/* lwis_fence_pending_signal_create: Creates and returns a lwis_fence_pending_signal list entry */
struct lwis_fence_pending_signal *lwis_fence_pending_signal_create(struct lwis_fence *fence,
								   struct file *fp);

/*
 *  lwis_fences_pending_signal_emit: Signal all lwis_fence_pending_signals in the pending_fences list
 */
void lwis_fences_pending_signal_emit(struct lwis_device *lwis_device,
				     struct list_head *pending_fences);

/*
 *  lwis_pending_fences_move_all: Move all lwis_fence_pending_signal from the transaction to pending_fences.
 */
void lwis_pending_fences_move_all(struct lwis_device *lwis_device,
				  struct lwis_transaction *transaction,
				  struct list_head *pending_fences, int error_code);

#endif /* LWIS_IOCTL_H_ */
