/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Google LWIS Fence
 *
 * Copyright (c) 2022 Google, LLC
 */

#ifndef LWIS_FENCE_H_
#define LWIS_FENCE_H_

#include <linux/hashtable.h>
#include <linux/list.h>
#include <linux/dma-fence.h>

#include "lwis_device.h"

#define LWIS_CLIENTS_HASH_BITS 8

extern bool lwis_fence_debug;
#define lwis_debug_info(fmt, ...)                                                                  \
	({                                                                                         \
		if (unlikely(lwis_fence_debug)) {                                                  \
			pr_info(fmt, ##__VA_ARGS__);                                               \
		}                                                                                  \
	})

#define lwis_debug_dev_info(dev, fmt, ...)                                                         \
	({                                                                                         \
		if (unlikely(lwis_fence_debug)) {                                                  \
			dev_info(dev, fmt, ##__VA_ARGS__);                                         \
		}                                                                                  \
	})

/*
 * LWIS fences are an extension of DMA fences. These fences are created from user space
 * request. Unlike DMA fences, these can be signaled from user space writing to the `signal_fd`
 * file descriptor. LWIS fences should only be used at creation time. The management of fences
 * should be done using the generic DMA fences interface. Once created, user space will submit
 * trigger expressions with the DMA fence file descriptor and we won't even know the fence is a
 * LWIS fence. In the LWIS driver, all the fence management should be done using the DMA fence
 * API, and only use this one when creating a fence with signaling support from user space.
 */
struct lwis_fence {
	/* Most of the LWIS fence functionality is covered by the dma_fence structure.
	 * This effectively "inherits" from dma_fence and means that LWIS fences can be
	 * used as DMA fences too.
	 */
	struct dma_fence dma_fence;

	/* Lock to protect the whole structure. */
	spinlock_t lock;

	/* Whether this fence should follow the old LWIS fence API. */
	bool legacy_lwis_fence;

	/* Top device for printing logs */
	struct lwis_device *lwis_top_dev;

	/* Status wait queue for waking up userspace */
	wait_queue_head_t status_wait_queue;
};

struct lwis_fence_pending_signal {
	struct dma_fence *fence;
	int pending_status;
	struct list_head node;
};

/*
 * lwis_fence_create: Create a new lwis_fence. Returns `lwis_fence_fds` with new
 * file descriptors and error information.
 */
struct lwis_fence_fds {
	/* Return error. Zero on success, negative errno otherwise. */
	int error;
	/*
	 * New fence file descriptors:
	 * (1) `fd` can be used as a `dma_fence` and
	 * (2) `signal_fd` can be used to signal the fence from user space with a
	 *     `write` syscall.
	 */
	int fd;
	int signal_fd;
};
struct lwis_fence_fds lwis_fence_create(struct lwis_device *lwis_dev);
/* Create a fence with the legacy LWIS Fence API */
struct lwis_fence_fds lwis_fence_legacy_create(struct lwis_device *lwis_dev);

/*
 * Helper function to signal a `dma_fence` with a specific status value.
 */
int lwis_dma_fence_signal_with_status(struct dma_fence *fence, int errno);

/* Gets the DMA fence of a LWIS fence with a fd. */
struct dma_fence *lwis_dma_fence_get(int fd);

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
 *  lwis_add_completion_fences_to_transaction: Prepares the transaction completion fence list.
 */
int lwis_add_completion_fences_to_transaction(struct lwis_client *client,
					      struct lwis_transaction *transaction);

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
