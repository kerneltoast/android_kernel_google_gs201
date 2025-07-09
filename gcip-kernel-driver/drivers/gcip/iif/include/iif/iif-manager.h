/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * The manager of inter-IP fences.
 *
 * It manages the pool of fence IDs. The IIF driver device will initialize a manager and each IP
 * driver will fetch the manager from the IIF device.
 *
 * Copyright (C) 2023-2024 Google LLC
 */

#ifndef __IIF_IIF_MANAGER_H__
#define __IIF_IIF_MANAGER_H__

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/idr.h>
#include <linux/kref.h>
#include <linux/of.h>
#include <linux/rwsem.h>
#include <linux/types.h>

#include <iif/iif-fence-table.h>
#include <iif/iif-shared.h>

struct iif_fence;

/* Operators. */
struct iif_manager_ops {
	/* Following callbacks are required. */
	/*
	 * Called when @fence is unblocked.
	 *
	 * When the signaler signals @fence as many times as the number of total signalers which was
	 * decided when the fence was created and the fence has been unblocked eventually. This
	 * callback will be called if the IP registered this operator is waiting on the fence to be
	 * unblocked.
	 *
	 * If @fence->error is non-zero errno, it means that the fence has been signaled witn an
	 * error at least once.
	 *
	 * If @fence->propagate is true, the IP drivers should notify their IP that the fence has
	 * been unblocked (i.e., send @fence->id to IP). If the signaler is AP, it will be always
	 * true. If the signaler is an IP, but it wasn't able to notify waiter IPs of the fence
	 * unblock, it will be true. For example, if the signaler IP becomes faulty and it's not
	 * able to inform waiter IPs that the fence has been unblocked with an error, the signaler
	 * IP driver will notice the IP crash and ask the IIF driver to set @fence->propagate to
	 * true to make waiter IP drivers notice that they need to inform their IP of the fence
	 * unblock.
	 *
	 * Note that the timing of @fence retirement is nondeterministic since we can't decide the
	 * timing of the runtime or IP crash. However, if @fence->propagate is true, but the fence
	 * has been retired in the middle, it should be still safe to notify waiter IPs of the fence
	 * unblock since they will verify the notification by checking the fence table.
	 *
	 * This callback returns void since the IIF driver has nothing can do when the IP driver
	 * fails to notify their IP. It is the responsibility of the IP driver side if that happens.
	 *
	 * Context: Normal.
	 */
	void (*fence_unblocked)(struct iif_fence *fence, void *data);

	/* Following callbacks are optional. */
	/*
	 * Acquires the block wakelock of IP.
	 *
	 * The callback will be executed when IP submits a waiter (i.e., `iif_fence_submit_waiter`
	 * is called).
	 *
	 * This callback is required if the waiter IPx doesn't allow to be notified by the signaler
	 * IPy when the IPx block is not powered on. If somehow the driver of waiter IPx submits a
	 * waiter command to the firmware late and the signaler IPy processes its command early by
	 * the race, IPy may try to notify IPx which is not powered on yet and it may cause a bug if
	 * the IPx spec doesn't allow that. Therefore, if the IPx implements this callback, the IIF
	 * driver will try to acquire the IPx block wakelock before submitting a waiter of IPx.
	 *
	 * Context: Depends on in which context the IPx calls the `iif_fence_submit_waiter`
	 *          function.
	 */
	int (*acquire_block_wakelock)(void *data);
	/*
	 * Releases the block wakelock of IP.
	 *
	 * If both of the waiter IPx and the signaler IPy finishes waiting on / signaling the fence,
	 * (i.e., both `iif_fence_waited` and `iif_fence_signal` are called) the block wakelock of
	 * IPx will be released.
	 *
	 * If somehow IPx driver calls `iif_fence_waited` before the fence is signaled (For example,
	 * IPy is not responding and IPx has processed its command as timeout.), releasing the block
	 * wakelock of IPx will be pended until IPy processes its command normally or as timeout and
	 * IPy driver eventually calls `iif_fence_signal`.
	 *
	 * Context: Normal.
	 */
	void (*release_block_wakelock)(void *data);
};

/*
 * The structure overall data required by IIF driver such as fence table.
 *
 * Until we have stand-alone IIF driver, one of the IP drivers will initializes a manager by
 * the `iif_init` function and every IP driver will share it.
 */
struct iif_manager {
	/* Reference count of this instance. */
	struct kref kref;
	/* Fence ID pool. */
	struct ida idp;
	/* Fence table shared with the firmware. */
	struct iif_fence_table fence_table;
	/* Operators per IP. */
	const struct iif_manager_ops *ops[IIF_IP_RESERVED];
	/* Protects @ops. */
	struct rw_semaphore ops_sema;
	/* User-data per IP. */
	void *data[IIF_IP_RESERVED];
	/* Platform bus device. */
	struct device *dev;
	/* Char device structure. */
	struct cdev char_dev;
	/* Char device number. */
	dev_t char_dev_no;
};

/*
 * Initializes IIF driver and returns its manager. Its initial reference count is 1. It will map
 * the fence table by parsing the device tree via @np.
 *
 * The returned manager will be destroyed when its reference count becomes 0 by `iif_manager_put`
 * function.
 */
struct iif_manager *iif_manager_init(const struct device_node *np);

/* Increases the reference count of @mgr. */
struct iif_manager *iif_manager_get(struct iif_manager *mgr);

/* Decreases the reference count of @mgr and if it becomes 0, releases @mgr. */
void iif_manager_put(struct iif_manager *mgr);

/*
 * Registers operators of @ip.
 *
 * Note that @ops must not be released until @ip won't be utilized as signaler or waiter anymore.
 */
int iif_manager_register_ops(struct iif_manager *mgr, enum iif_ip_type ip,
			     const struct iif_manager_ops *ops, void *data);

/* Unregisters operators of @ip. */
void iif_manager_unregister_ops(struct iif_manager *mgr, enum iif_ip_type ip);

/*
 * Acquires the block wakelock of @ip.
 *
 * Returns 0 on success or @ip hasn't defined the `acquire_block_wakelock` operator. Otherwise,
 * returns a negative errno.
 */
int iif_manager_acquire_block_wakelock(struct iif_manager *mgr, enum iif_ip_type ip);

/* Releases the block wakelock of @ip. */
void iif_manager_release_block_wakelock(struct iif_manager *mgr, enum iif_ip_type ip);

/*
 * Notifies @fence has been unblocked to IP drivers waiting on the fence.
 *
 * This function will be called if @fence has been unblocked.
 */
void iif_manager_broadcast_fence_unblocked(struct iif_manager *mgr, struct iif_fence *fence);

#endif /* __IIF_IIF_MANAGER_H__ */
