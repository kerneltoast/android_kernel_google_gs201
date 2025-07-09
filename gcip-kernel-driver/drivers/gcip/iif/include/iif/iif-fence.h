/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * The inter-IP fence.
 *
 * The meaning of "a fence is signaled" or "unblocked" is that the fence has been signaled enough
 * times as many as the expected number of signals which is decided when the fence is initialized
 * and it has been unblocked. That says every single signaler commands are expected to signal the
 * fence (i.e., call `iif_fence_signal{_with_status}` function) even though commands weren't
 * processed normally.
 *
 * Also, unblocking a fence here is only for the kernel perspective. Therefore, the IIF driver will
 * notify the fence unblock to only who are polling the fences (via poll callbacks or poll syscall).
 * It means that if the signaler is an IP, not AP, it is a responsibility of the IP side to unblock
 * a fence and propagate an error to waiter IPs. Therefore, unblocking fence by the IIF driver will
 * not unblock the fences in the IP side unless the IP kernel driver notices the fence unblock via
 * a poll callback and asks their IP to unblock the fence.
 *
 * If the signaler IP requires a support of the kernel driver to unblock the fence in case the IP is
 * already faulty and can't notify waiter IPs, the signaler IP kernel driver can unblock the fence
 * with an error and each waiter IP kernel driver can notice it by `fence_unblocked` operator of the
 * fence manager or registering a poll callback to the fence directly and propagate the error to the
 * IP of each.
 *
 * Besides, one of the main roles of the IIF driver is creating fences with assigning fence IDs,
 * initializing the fence table and managing the life cycle of them.
 *
 * Copyright (C) 2023-2024 Google LLC
 */

#ifndef __IIF_IIF_FENCE_H__
#define __IIF_IIF_FENCE_H__

#include <linux/kref.h>
#include <linux/lockdep_types.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/workqueue.h>

#include <iif/iif-manager.h>
#include <iif/iif-shared.h>

/*
 * By default, the fence will retire if there are no outstanding signalers, waiters and FDs. Its
 * purpose is to return the fence ID to the pool in early stage before the fence releases so that we
 * can minimize the possibility of the lack of fence ID. Therefore, the retirement condition will be
 * checked for each `signal()`, `waited()` and `close(fd)` call. However, when the user sets this
 * flag to a fence, the fence will retire only when it releases.
 *
 * If there is a possibility of a race condition between the signaler and waiters, this flag can be
 * considered to prevent the early fence retirement. For example, if the signaler signals a fence
 * before any waiter starts waiting on the fence, the IIF driver will think there are no outstanding
 * signalers and waiters, so it will let the fence retire. That means upcoming waiters may try to
 * wait on the retired fence which is invalid.
 *
 * Normally, that race condition doesn't have to be considered since the runtime will always hold
 * FDs while drivers/firmwares manipulating a fence and the fence won't retired in any case until
 * the runtime closes FDs. However, if there is an use case that utilizing a fence which won't be
 * exposed to the runtime (i.e., no FD is installed to the fence), this flag will be required to
 * prevent the race condition.
 */
#define IIF_FLAGS_RETIRE_ON_RELEASE BIT(0)

struct iif_fence;
struct iif_fence_ops;
struct iif_fence_poll_cb;
struct iif_fence_all_signaler_submitted_cb;

/*
 * The callback which will be called when all signalers have signaled @fence.
 *
 * It will be called while @fence->signalers_lock is held and it is safe to read
 * @fence->signal_error inside.
 *
 * The callback will be called in the normal context.
 */
typedef void (*iif_fence_poll_cb_t)(struct iif_fence *fence, struct iif_fence_poll_cb *cb);

/*
 * The callback which will be called when all signalers have been submitted to @fence.
 *
 * It will be called while @fence->signalers_lock is held and it is safe to read
 * @fence->all_signaler_submitted_error inside.
 */
typedef void (*iif_fence_all_signaler_submitted_cb_t)(
	struct iif_fence *fence, struct iif_fence_all_signaler_submitted_cb *cb);

/*
 * The state of a fence object.
 * The state transition is
 *   INITED {-> FILE_CREATED -> FILE_RELEASED} -> RETIRED
 * i.e. Sync file creation is optional.
 */
enum iif_fence_state {
	/* Initial state. */
	IIF_FENCE_STATE_INITIALIZED,
	/* The fence ID has been retired. */
	IIF_FENCE_STATE_RETIRED,
};

/*
 * Contains the callback function which will be called when the fence has been unblocked.
 *
 * The callback can be registered to the fence by the `iif_fence_add_poll_callback` function.
 */
struct iif_fence_poll_cb {
	/* Node to be added to the list. */
	struct list_head node;
	/* Actual callback function to be called. */
	iif_fence_poll_cb_t func;
};

/*
 * Contains the callback function which will be called when all signalers have been submitted.
 *
 * The callback will be registered to the fence when the `iif_fence_submit_waiter` function fails
 * in the submission.
 */
struct iif_fence_all_signaler_submitted_cb {
	/* Node to be added to the list. */
	struct list_head node;
	/* Actual callback function to be called. */
	iif_fence_all_signaler_submitted_cb_t func;
	/* The number of remaining signalers to be submitted. */
	int remaining_signalers;
};

/* The fence object. */
struct iif_fence {
	/* IIF manager. */
	struct iif_manager *mgr;
	/* Fence ID. */
	int id;
	/* Signaler IP type. */
	enum iif_ip_type signaler_ip;
	/* The number of total signalers to be submitted. */
	uint16_t total_signalers;
	/* The number of submitted signalers. */
	uint16_t submitted_signalers;
	/* The number of signaled signalers. */
	uint16_t signaled_signalers;
	/* The interrupt state before holding @signalers_lock. */
	unsigned long signalers_lock_flags;
	/* The number of outstanding waiters. */
	uint16_t outstanding_waiters;
	/* The number of outstanding waiters per IP. */
	uint16_t outstanding_waiters_per_ip[IIF_IP_RESERVED];
	/* The number of outstanding wakelock holds per waiter IP. */
	uint16_t outstanding_block_wakelock[IIF_IP_RESERVED];
	/*
	 * Protects overall properties of the fence. (outstanding signalers / waiters, callbacks,
	 * state, ...)
	 */
	rwlock_t fence_lock;
#if IS_ENABLED(CONFIG_DEBUG_SPINLOCK)
	struct lock_class_key fence_lock_key;
#endif /* IS_ENABLED(CONFIG_DEBUG_SPINLOCK) */
	/* Reference count. */
	struct kref kref;
	/* Operators. */
	const struct iif_fence_ops *ops;
	/* State of this fence object. */
	enum iif_fence_state state;
	/* List of callbacks which will be called when the fence is unblocked. */
	struct list_head poll_cb_list;
	/* List of callbacks which will be called when all signalers have been submitted. */
	struct list_head all_signaler_submitted_cb_list;
	/* Will be set to a negative errno if the fence is signaled with an error. */
	int signal_error;
	/* Will be set to a negative errno if waiting the signaler submission fails. */
	int all_signaler_submitted_error;
	/* The number of sync_file(s) bound to the fence. */
	atomic_t num_sync_file;
	/* The callback called once the fence has been unblocked. */
	struct iif_fence_poll_cb unblocked_cb;
	/* If true, the waiter IP drivers should propagate the fence unblock to their IP. */
	bool propagate;
	/* Work which will be executed when the fence has been unblocked. */
	struct work_struct signaled_work;
	/* Work which will be executed when each waiter command finished waiting on the fence. */
	struct work_struct waited_work;
	/* Work decreasing the refcount of fence asynchronously. */
	struct work_struct put_work;
	/* Attributes. */
	unsigned long flags;
};

/* Operators of `struct iif_fence`. */
struct iif_fence_ops {
	/*
	 * Called on destruction of @fence to release additional resources when its reference count
	 * becomes zero.
	 *
	 * This callback is optional.
	 * Context: normal and in_interrupt().
	 */
	void (*on_release)(struct iif_fence *fence);
};

/*
 * Initializes @fence which will be signaled by @signaler_ip IP. @total_signalers is the number of
 * signalers which must be submitted to the fence. Its initial reference count is 1.
 *
 * The initialized fence will be assigned an ID which depends on @signaler_ip. Each IP will have at
 * most `IIF_NUM_FENCES_PER_IP` number of fences and the assigned fence ID for IP[i] will be one of
 * [i * IIF_NUM_FENCES_PER_IP ~ (i + 1) * IIF_NUM_FENCES_PER_IP - 1].
 */
int iif_fence_init(struct iif_manager *mgr, struct iif_fence *fence,
		   const struct iif_fence_ops *ops, enum iif_ip_type signaler_ip,
		   uint16_t total_signalers);

/* Sets the flags of @fence. */
static inline void iif_fence_set_flags(struct iif_fence *fence, unsigned long flags)
{
	fence->flags = flags;
}

/*
 * Opens a file which syncs with @fence and returns its FD. The file will hold a reference to
 * @fence until it is closed.
 */
int iif_fence_install_fd(struct iif_fence *fence);

/*
 * Has @fence know the sync file bound to it is about to be released. This function would try to
 * retire the fence if applicable.
 */
void iif_fence_on_sync_file_release(struct iif_fence *fence);

/* Increases the reference count of @fence. */
struct iif_fence *iif_fence_get(struct iif_fence *fence);

/*
 * Gets a fence from @fd and increments its reference count of the file pointer.
 *
 * Returns the fence pointer, if @fd is for IIF. Otherwise, returns a negative errno.
 */
struct iif_fence *iif_fence_fdget(int fd);

/*
 * Decreases the reference count of @fence and if it becomes 0, releases @fence.
 *
 * If the caller is going to put @fence in the un-sleepable context such as the IRQ context or spin
 * lock, they should use the async one.
 */
void iif_fence_put(struct iif_fence *fence);
void iif_fence_put_async(struct iif_fence *fence);

/*
 * Submits a signaler. @fence->submitted_signalers will be incremented by 1.
 *
 * This function cannot be called in the IRQ context.
 *
 * Returns 0 if the submission succeeds. Otherwise, returns a negative errno.
 */
int iif_fence_submit_signaler(struct iif_fence *fence);

/*
 * Submits a waiter of @ip IP. @fence->outstanding_waiters will be incremented by 1.
 * Note that the waiter submission will not be done when not all signalers have been submitted.
 * (i.e., @fence->submitted_signalers < @fence->total_signalers)
 *
 * This function will acquire the block wakelock of @ip before it updates the IIF's wait table to
 * mark @ip is going to wait on @fence. Otherwise, if the signaler IPx processes its command even
 * earlier than the waiter IPy powers its block up by the race, IPx may try to notify IPy which is
 * not powered up yet. If IPy spec doesn't allow that, it may cause an unexpected bug. Therefore, we
 * should acquire the block wakelock of @ip before updating the wait table.
 *
 * This function cannot be called in the IRQ context.
 *
 * Returns the number of remaining signalers to be submitted (i.e., returning 0 means the submission
 * actually succeeded). Otherwise, returns a negative errno if it fails with other reasons.
 */
int iif_fence_submit_waiter(struct iif_fence *fence, enum iif_ip_type ip);

/*
 * Submits a waiter of @waiter_ip to each fence in @in_fences and a signaler to each fence in
 * @out_fences. Either @in_fences or @out_fences is allowed to be NULL.
 *
 * For the waiter submission, if at least one fence of @in_fences haven't finished the signaler
 * submission, this function will fail and return -EAGAIN.
 *
 * For the signaler submission, if at least one fence of @out_fences have already finished the
 * signaler submission, this function will fail and return -EPERM.
 *
 * This function will be useful when the caller wants to accomplish the waiter submission and the
 * signaler submission atomically.
 *
 * This function cannot be called in the IRQ context.
 *
 * Note that this function may reorder fences internally. This is to prevent a potential dead lock
 * which can be caused by holding the locks of multiple fences at the same time. Also, fences in
 * @in_fences and @out_fences should be unique. Otherwise, it will return -EDEADLK.
 *
 * The function returns 0 on success.
 */
int iif_fence_submit_signaler_and_waiter(struct iif_fence **in_fences, int num_in_fences,
					 struct iif_fence **out_fences, int num_out_fences,
					 enum iif_ip_type waiter_ip);

/*
 * Signals @fence.
 *
 * If all signaler commands have called this function for the fence and it has been unblocked, all
 * registered poll callbacks will be executed.
 *
 * If the caller is going to signal @fence in the un-sleepable context such as IRQ context or spin
 * lock, one should use the `iif_fence_signal_async` function below. Its functionality is the same,
 * but notifying poll callbacks will be done asynchronously.
 *
 * It may try to release the block wakelock of waiter IPs if there are some IPs which called the
 * `iif_fence_waited` function earlier than this function call and releasing the block wakelock of
 * those IPs was pended. (See `iif_fence_waited` function below.)
 *
 * Returns the number of remaining signals on success. Otherwise, returns a negative errno.
 */
int iif_fence_signal(struct iif_fence *fence);
int iif_fence_signal_async(struct iif_fence *fence);

/*
 * Signals @fence with a status.
 *
 * Basically, its functionality is the same as the `iif_fence_signal` function above, but the user
 * can supply an optional error status.
 *
 * Note that even though @error is non-zero, @fence won't be unblocked until the number of remaining
 * signals becomes 0.
 *
 * If the caller passes 0 to @error, its functionality is the same as the `iif_fence_signal`
 * function.
 *
 * If the caller is going to signal @fence in the un-sleepable context such as IRQ context or spin
 * lock, one should use the `iif_fence_signal_with_status_async` function below. Its functionality
 * is the same, but notifying poll callbacks will be done asynchronously.
 *
 * Returns the number of remaining signals on success. Otherwise, returns a negative errno.
 */
int iif_fence_signal_with_status(struct iif_fence *fence, int error);
int iif_fence_signal_with_status_async(struct iif_fence *fence, int error);

/*
 * Returns the signal status of @fence.
 *
 * Returns 0 if the fence hasn't been unblocked yet, 1 if the fence has been unblocked without any
 * error, or a negative errno if the fence has been signaled with an error at least once.
 */
int iif_fence_get_signal_status(struct iif_fence *fence);

/*
 * Sets @fence->propagate to true.
 *
 * When @fence has been unblocked and the `fence_unblocked` callback is called, the waiter IP
 * drivers will refer to @fence->propagate and they will inform their IP of the fence unblock if
 * that is true.
 *
 * In case of the signaler IPx of @fence is not able to notify waiter IPs of the fence unblock, the
 * IPx driver can utilize this function to propagate the fence unblock to waiter IP drivers. For
 * example, if IPx becomes faulty and it can't propagate the fence unblock with an error to waiter
 * IPs by itself, the IPx driver can utilize this function when it detects the IP crash to set
 * @fence->propagate to true and the waiter IP drivers will inform their IP of the fence unblock
 * when the `fence_unblocked` callback is called.
 *
 * Note that this function must be called before signaling the fence if needed. Also, the IIF driver
 * will take over the responsibility of updating the number of remaining signals in the fence table
 * of @fence from the IP firmware since calling this function means that the signaler IP doesn't
 * have ability of managing the signal of @fence anymore.
 */
void iif_fence_set_propagate_unblock(struct iif_fence *fence);

/*
 * Returns whether all signalers have signaled @fence and it has been unblocked.
 *
 * As this function doesn't require to hold any lock, even if this function returns false, @fence
 * can be signaled right after this function returns. One should care about this and may not use
 * this function directly. This function will be mostly used when iif_sync_file is polling @fence.
 */
bool iif_fence_is_signaled(struct iif_fence *fence);

/*
 * Notifies the driver that a waiter of @ip finished waiting on @fence.
 *
 * It will try to release the block wakelock of @ip which was held when `iif_fence_submit_waiter`
 * was called if @fence was already signaled (i.e., `iif_fence_signal` was called) and the IP
 * defined the `release_block_wakelock` operator (See iif-manager.h file).
 *
 * Note that if @fence is not signaled yet, releasing the block wakelock will be pended until @fence
 * is signaled (i.e., `iif_fence_signal` is called) or it is destroyed. This case can happen when
 * the signaler IPx is not responding in time and the waiter IPy processes its command as timeout.
 * This pending logic is required because if IPy doesn't pend releasing its block wakelock and IPx
 * suddenly processes its command, IPx may try to notify IPy whose block is already powered down and
 * it may cause an unexpected bug if IPy spec doesn't allow that.
 *
 * If the caller is going to stop waiting on @fence in the un-sleepable context such as IRQ context
 * or spin lock, one should use the `iif_fence_waited_async` function below. Its functionality is
 * the same, but `release_block_wakelock` callbacks will be called asynchronously.
 */
void iif_fence_waited(struct iif_fence *fence, enum iif_ip_type ip);
void iif_fence_waited_async(struct iif_fence *fence, enum iif_ip_type ip);

/*
 * Registers a callback which will be called when @fence has been unblocked. Once the callback is
 * called, it will be automatically unregistered from @fence. The @func can be called in the IRQ
 * context.
 *
 * Returns 0 if succeeded. Otherwise, returns a negative errno on failure. Note that even when
 * @fence is already unblocked, it won't add the callback and return -EPERM.
 */
int iif_fence_add_poll_callback(struct iif_fence *fence, struct iif_fence_poll_cb *poll_cb,
				iif_fence_poll_cb_t func);

/*
 * Unregisters the callback from @fence.
 *
 * Returns true if the callback is removed before @fence is unblocked.
 */
bool iif_fence_remove_poll_callback(struct iif_fence *fence, struct iif_fence_poll_cb *poll_cb);

/*
 * Registers a callback which will be called when all signalers are submitted for @fence and
 * returns the number of remaining signalers to be submitted to @cb->remaining_signalers. Once the
 * callback is called, it will be automatically unregistered from @fence.
 *
 * Note that, as the callback can be invoked right after the registration, if the callback releases
 * @cb internally, the caller should be careful of accessing @cb after the function returns.
 *
 * Returns 0 if succeeded. If all signalers are already submitted, returns -EPERM.
 */
int iif_fence_add_all_signaler_submitted_callback(struct iif_fence *fence,
						  struct iif_fence_all_signaler_submitted_cb *cb,
						  iif_fence_all_signaler_submitted_cb_t func);

/*
 * Unregisters the callback which is registered by the callback above.
 *
 * Returns true if the callback is removed before its being called.
 */
bool iif_fence_remove_all_signaler_submitted_callback(
	struct iif_fence *fence, struct iif_fence_all_signaler_submitted_cb *cb);

/*
 * Returns the number of signalers or waiters information accordingly.
 *
 * Note that these functions hold required locks internally and read the value. Therefore, the value
 * of them can be changed after the function returns. The one must use these functions only for the
 * debugging purpose.
 *
 * These functions can be called in the IRQ context.
 */
int iif_fence_unsubmitted_signalers(struct iif_fence *fence);
int iif_fence_submitted_signalers(struct iif_fence *fence);
int iif_fence_signaled_signalers(struct iif_fence *fence);
int iif_fence_outstanding_waiters(struct iif_fence *fence);

#endif /* __IIF_IIF_FENCE_H__ */
