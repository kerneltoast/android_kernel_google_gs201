// SPDX-License-Identifier: GPL-2.0-only
/*
 * IIF driver sync file.
 *
 * To export fences to the userspace, the driver will allocate a sync file to a fence and will
 * return its file descriptor to the user. The user can distinguish fences with it. The driver will
 * convert the file descriptor to the corresponding fence ID and will pass it to the IP.
 *
 * Copyright (C) 2023-2024 Google LLC
 */

#define pr_fmt(fmt) "iif: " fmt

#include <linux/anon_inodes.h>
#include <linux/atomic.h>
#include <linux/bitops.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/uaccess.h>

#include <iif/iif-fence.h>
#include <iif/iif-sync-file.h>
#include <iif/iif.h>

static void iif_sync_file_fence_signaled(struct iif_fence *fence, struct iif_fence_poll_cb *poll_cb)
{
	struct iif_sync_file *sync_file = container_of(poll_cb, struct iif_sync_file, poll_cb);

	wake_up_all(&sync_file->wq);
}

static int iif_sync_file_release(struct inode *inode, struct file *file)
{
	struct iif_sync_file *sync_file = file->private_data;

	if (atomic_dec_and_test(&sync_file->fence->num_sync_file))
		iif_fence_on_sync_file_release(sync_file->fence);
	if (test_bit(IIF_SYNC_FILE_FLAGS_POLL_ENABLED, &sync_file->flags))
		iif_fence_remove_poll_callback(sync_file->fence, &sync_file->poll_cb);
	iif_fence_put(sync_file->fence);
	kfree(sync_file);

	return 0;
}

static __poll_t iif_sync_file_poll(struct file *file, poll_table *wait)
{
	struct iif_sync_file *sync_file = file->private_data;
	int ret;

	poll_wait(file, &sync_file->wq, wait);

	if (list_empty(&sync_file->poll_cb.node) &&
	    !test_and_set_bit(IIF_SYNC_FILE_FLAGS_POLL_ENABLED, &sync_file->flags)) {
		ret = iif_fence_add_poll_callback(sync_file->fence, &sync_file->poll_cb,
						  iif_sync_file_fence_signaled);
		/* If all signalers of the fence already signaled, just wake up all. */
		if (ret < 0)
			wake_up_all(&sync_file->wq);
	}

	return iif_fence_is_signaled(sync_file->fence) ? EPOLLIN : 0;
}

static int iif_sync_file_ioctl_get_information(struct iif_sync_file *sync_file,
					       struct iif_fence_get_information_ioctl __user *argp)
{
	struct iif_fence *fence = sync_file->fence;
	const struct iif_fence_get_information_ioctl ibuf = {
		.signaler_ip = fence->signaler_ip,
		.total_signalers = fence->total_signalers,
		.submitted_signalers = iif_fence_submitted_signalers(fence),
		.signaled_signalers = iif_fence_signaled_signalers(fence),
		.outstanding_waiters = iif_fence_outstanding_waiters(fence),
		.signal_status = iif_fence_get_signal_status(fence),
	};

	if (copy_to_user(argp, &ibuf, sizeof(ibuf)))
		return -EFAULT;

	return 0;
}

static int iif_sync_file_ioctl_submit_signaler(struct iif_sync_file *sync_file)
{
	struct iif_fence *fence = sync_file->fence;

	if (fence->signaler_ip != IIF_IP_AP) {
		pr_err("Only fences with signaler type AP are allowed to submit a signaler (signaler_ip=%d)",
		       fence->signaler_ip);
		return -EPERM;
	}
	return iif_fence_submit_signaler(fence);
}

static int iif_sync_file_ioctl_signal(struct iif_sync_file *sync_file,
				      struct iif_fence_signal_ioctl __user *argp)
{
	struct iif_fence_signal_ioctl ibuf;
	struct iif_fence *fence = sync_file->fence;
	int ret;

	if (copy_from_user(&ibuf, argp, sizeof(ibuf)))
		return -EFAULT;

	if (fence->signaler_ip != IIF_IP_AP) {
		pr_err("Only fences with signaler type AP are allowed to signal (signaler_ip=%d)",
		       fence->signaler_ip);
		return -EPERM;
	}

	ret = iif_fence_signal_with_status(fence, ibuf.error);
	if (ret < 0)
		return ret;

	ibuf.remaining_signals = ret;

	if (copy_to_user(argp, &ibuf, sizeof(ibuf)))
		return -EFAULT;

	return 0;
}

static long iif_sync_file_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct iif_sync_file *sync_file = file->private_data;
	void __user *argp = (void __user *)arg;

	switch (cmd) {
	case IIF_FENCE_GET_INFORMATION:
		return iif_sync_file_ioctl_get_information(sync_file, argp);
	case IIF_FENCE_SUBMIT_SIGNALER:
		return iif_sync_file_ioctl_submit_signaler(sync_file);
	case IIF_FENCE_SIGNAL:
		return iif_sync_file_ioctl_signal(sync_file, argp);
	default:
		return -ENOTTY;
	}
}

static const struct file_operations iif_sync_file_fops = {
	.release = iif_sync_file_release,
	.poll = iif_sync_file_poll,
	.unlocked_ioctl = iif_sync_file_ioctl,
};

struct iif_sync_file *iif_sync_file_create(struct iif_fence *fence)
{
	struct iif_sync_file *sync_file;
	int ret;

	sync_file = kzalloc(sizeof(*sync_file), GFP_KERNEL);
	if (!sync_file)
		return ERR_PTR(-ENOMEM);

	sync_file->file = anon_inode_getfile("iif_file", &iif_sync_file_fops, sync_file, 0);
	if (IS_ERR(sync_file->file)) {
		ret = PTR_ERR(sync_file->file);
		goto err_free_sync_file;
	}

	sync_file->fence = iif_fence_get(fence);
	atomic_inc(&fence->num_sync_file);

	init_waitqueue_head(&sync_file->wq);
	INIT_LIST_HEAD(&sync_file->poll_cb.node);

	return sync_file;

err_free_sync_file:
	kfree(sync_file);
	return ERR_PTR(ret);
}

struct iif_sync_file *iif_sync_file_fdget(int fd)
{
	struct file *file = fget(fd);

	if (!file)
		return ERR_PTR(-EBADF);

	if (file->f_op != &iif_sync_file_fops) {
		fput(file);
		return ERR_PTR(-EINVAL);
	}

	return file->private_data;
}
