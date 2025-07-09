/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Defines the interface of the IIF driver.
 *
 * Copyright (C) 2023 Google LLC
 */

#ifndef __IIF_IIF_H__
#define __IIF_IIF_H__

#include <linux/ioctl.h>
#include <linux/types.h>

/* Interface Version. */
#define IIF_INTERFACE_VERSION_MAJOR 1
#define IIF_INTERFACE_VERSION_MINOR 0

#define IIF_IOCTL_BASE 'i'

/* The ioctl number for the fence FDs will start from here. */
#define IIF_FENCE_IOCTL_NUM_BASE 0x80

/* The maximum number of fences can be passed to one ioctl request. */
#define IIF_MAX_NUM_FENCES 64

/*
 * ioctls for /dev/iif.
 */

struct iif_create_fence_ioctl {
	/*
	 * Input:
	 * The type of the fence signaler IP. (See enum iif_ip_type)
	 */
	__u8 signaler_ip;
	/*
	 * Input:
	 * The number of the signalers.
	 */
	__u16 total_signalers;
	/*
	 * Output:
	 * The file descriptor of the created fence.
	 */
	__s32 fence;
};

/* Create an IIF fence. */
#define IIF_CREATE_FENCE _IOWR(IIF_IOCTL_BASE, 0, struct iif_create_fence_ioctl)

/*
 * The ioctl won't register @eventfd and will simply return the number of
 * remaining signalers of each fence.
 */
#define IIF_FENCE_REMAINING_SIGNALERS_NO_REGISTER_EVENTFD (~0u)

struct iif_fence_remaining_signalers_ioctl {
	/*
	 * Input:
	 * User-space pointer to an int array of inter-IP fence file descriptors
	 * to check whether there are remaining signalers to be submitted or
	 * not.
	 */
	__u64 fences;
	/*
	 * Input:
	 * The number of fences in `fence_array`.
	 * If > IIF_MAX_NUM_FENCES, the ioctl will fail with errno == EINVAL.
	 */
	__u32 fences_count;
	/*
	 * Input:
	 * The eventfd which will be triggered if there were fence(s) which
	 * haven't finished the signaler submission yet when the ioctl is called
	 * and when they eventually have finished the submission. Note that if
	 * all fences already finished the submission (i.e., all values in the
	 * returned @remaining_signalers are 0), this eventfd will be ignored.
	 *
	 * Note that if `IIF_FENCE_REMAINING_SIGNALERS_NO_REGISTER_EVENTFD` is
	 * passed, this ioctl will simply return the number of remaining
	 * signalers of each fence to @remaining_signalers.
	 */
	__u32 eventfd;
	/*
	 * Output:
	 * User-space pointer to an int array where the driver will write the
	 * number of remaining signalers to be submitted per fence. The order
	 * will be the same with @fences.
	 */
	__u64 remaining_signalers;
};

/*
 * Check whether there are remaining signalers to be submitted to fences.
 * If all signalers have been submitted, the runtime is expected to send waiter
 * commands right away. Otherwise, it will listen the eventfd to wait signaler
 * submission to be finished.
 */
#define IIF_FENCE_REMAINING_SIGNALERS \
	_IOWR(IIF_IOCTL_BASE, 1, struct iif_fence_remaining_signalers_ioctl)

/*
 * ioctls for inter-IP fence FDs.
 */

struct iif_fence_get_information_ioctl {
	/* The type of the signaler IP. (enum iif_ip_type) */
	__u8 signaler_ip;
	/* The number of total signalers. */
	__u16 total_signalers;
	/* The number of submitted signalers. */
	__u16 submitted_signalers;
	/* The number of signaled signalers. */
	__u16 signaled_signalers;
	/* The number of outstanding waiters. */
	__u16 outstanding_waiters;
	/*
	 * The signal status of fence.
	 * - 0: The fence hasn't been unblocked yet.
	 * - 1: The fence has been unblocked without any error.
	 * - Negative errno: The fence has been unblocked with an error.
	 */
	__s16 signal_status;
	/* Reserved. */
	__u8 reserved[5];
};

/* Returns the fence information. */
#define IIF_FENCE_GET_INFORMATION \
	_IOR(IIF_IOCTL_BASE, IIF_FENCE_IOCTL_NUM_BASE, struct iif_fence_get_information_ioctl)

/*
 * Submits a signaler to the fence.
 *
 * This ioctl is available only when the fence signaler is AP.
 *
 * The runtime should call this ioctl for every signaler command before they
 * start processing it. Once a signaler command is done, they are also expected
 * to call the IIF_FENCE_SIGNAL ioctl. (See IIF_FENCE_SIGNAL ioctl below.)
 *
 * I.e.,
 * ...
 * ioctl(fence_fd, IIF_FENCE_SUBMIT_SIGNALER);
 * process(signaler_command_0);
 * ioctl(fence_fd, IIF_FENCE_SIGNAL);
 * ...
 * ioctl(fence_fd, IIF_FENCE_SUBMIT_SIGNALER);
 * process(signaler_command_1);
 * ioctl(fence_fd, IIF_FENCE_SIGNAL);
 * ...
 *
 * Return value:
 *   0      - Succeeded in signaling the fence.
 *   -EPERM - The signaler type of the fence is not AP or already all signalers
 *            have been submitted to the fence.
 */
#define IIF_FENCE_SUBMIT_SIGNALER _IO(IIF_IOCTL_BASE, IIF_FENCE_IOCTL_NUM_BASE + 1)

struct iif_fence_signal_ioctl {
	/*
	 * Input:
	 * An error code to indicate that a signaler command has been processed
	 * normally or with an error.
	 *
	 * If AP has failed in processing a signaler command of the fence, they
	 * should pass an errno to here so that the IPs waiting on the fence
	 * can notice that they may not able to proceed their waiter commands.
	 * In this case, the number of remaining signals will become 0 which
	 * means that the fence will be unblocked and the fence error will be
	 * propagated to each waiting IP right away.
	 *
	 * If there was no error, it must be set to 0.
	 */
	__s32 error;
	/*
	 * Output:
	 * The number of remaining signals to unblock the fence. If it is 0,
	 * it means that the fence has been unblocked and the IPs waiting on the
	 * fence have been notified.
	 */
	__u16 remaining_signals;
};

/*
 * Signals the fence.
 *
 * This ioctl is available only when the fence signaler is AP.
 *
 * The runtime should call this ioctl for every signaler command after they
 * have finished processing it.
 *
 * Return value:
 *   0      - Succeeded in signaling the fence.
 *   -EBUSY - The fence is already unblocked.
 *   -EPERM - The signaler type of the fence is not AP and signaling it by the
 *            runtime is not allowed.
 */
#define IIF_FENCE_SIGNAL \
	_IOWR(IIF_IOCTL_BASE, IIF_FENCE_IOCTL_NUM_BASE + 2, struct iif_fence_signal_ioctl)

#endif /* __IIF_IIF_H__ */
