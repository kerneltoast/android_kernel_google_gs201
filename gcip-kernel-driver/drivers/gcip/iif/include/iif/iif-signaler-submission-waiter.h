/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * The interface for waiting on multiple inter-IP fences to complete the signaler submission.
 *
 * Copyright (C) 2023 Google LLC
 */

#ifndef __IIF_IIF_SIGNALER_SUBMISSION_WAITER_H__
#define __IIF_IIF_SIGNALER_SUBMISSION_WAITER_H__

#include <iif/iif-fence.h>

#define IIF_NO_REGISTER_EVENTFD (~0u)

/*
 * These structures are not supposed to be used by the IP drivers.
 * They must use the `iif_wait_signaler_submission` function below directly.
 */

/* Waiter which waits on multiple fences to finish the signaler submission. */
struct iif_signaler_submission_waiter {
	/* Registered callbacks. */
	struct list_head cb_list;
	/* Refcount. */
	struct kref kref;
	/* Eventfd context. */
	struct eventfd_ctx *ctx;
	/* The number of remaining fences to register the callback. */
	int pending_fences;
	/* True if waiter is cancelled and all callbacks in @cb_list should be flushed. */
	bool cancel;
	/* Protects @cb_list, @pending_fences and @cancel. */
	spinlock_t lock;
};

/* Contains information required when each fence calls the signaler_submitted callback. */
struct iif_signaler_submission_waiter_cb {
	/* The callback object which will be registered to @fence. */
	struct iif_fence_all_signaler_submitted_cb fence_cb;
	/* The fence which is going to finish the signaler submission. */
	struct iif_fence *fence;
	/* The waiter instance which waits on this callback. */
	struct iif_signaler_submission_waiter *waiter;
	/* The node to be added to @waiter->cb_list. */
	struct list_head node;
	/* Refcount. */
	struct kref kref;
};

/*
 * Waits on @fences to complete the signaler submission. If at least one of @fences have remaining
 * signalers to be submitted, it will register @eventfd and will trigger it once all fences have
 * finishes the submission. Also, the number of remaining signalers of each fence will be returned
 * to @remaining_signalers in the same order with @fences.
 *
 * If @eventfd is IIF_NO_REGISTER_EVENTFD, this function won't wait on @fences to finish signaler
 * submission and will simply return the number of remaining signalers of each fence.
 *
 * Returns 0 on success. Otherwise, returns a negative errno.
 */
int iif_wait_signaler_submission(struct iif_fence **fences, int num_fences, unsigned int eventfd,
				 int *remaining_signalers);

/* Increments the refcount of @waiter. */
struct iif_signaler_submission_waiter *
iif_all_signaler_submission_waiter_get(struct iif_signaler_submission_waiter *waiter);

/* Decrements the refcount of @waiter and releases it if the count becomes 0. */
void iif_all_signaler_submission_waiter_put(struct iif_signaler_submission_waiter *waiter);

#endif /* __IIF_IIF_SIGNALER_SUBMISSION_WAITER_H__ */
