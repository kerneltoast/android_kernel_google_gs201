// SPDX-License-Identifier: GPL-2.0-only
/*
 * GCIP support of DMA fences.
 *
 * Copyright (C) 2023 Google LLC
 */

#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/dma-fence.h>
#include <linux/dma-fence-unwrap.h>
#include <linux/err.h>
#include <linux/file.h>
#include <linux/ktime.h>
#include <linux/list.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/sync_file.h>
#include <linux/time.h>

#include <gcip/gcip-dma-fence.h>

#define to_gfence(fence) container_of(fence, struct gcip_dma_fence, fence)

int gcip_signal_dma_fence_with_status(struct dma_fence *fence, int error, bool ignore_signaled)
{
	unsigned long flags;
	int ret;
	struct dma_fence *cur;
	struct dma_fence_unwrap iter;

	if (error > 0)
		error = -error;
	if (unlikely(error < -MAX_ERRNO))
		return -EINVAL;

	/* If not ignoring signaled, only return busy if ALL fences are already signaled. */
	ret = ignore_signaled ? 0 : -EBUSY;

	/*
	 * If @fence is a dma_fence_array, iterate over each fence in the array and signal it.
	 * This loop will be executed exactly once for cur == @fence, if @fence is not an array.
	 */
	dma_fence_unwrap_for_each(cur, &iter, fence) {
		spin_lock_irqsave(cur->lock, flags);
		/* don't signal fence twice */
		if (unlikely(test_bit(DMA_FENCE_FLAG_SIGNALED_BIT, &cur->flags)))
			goto cont_unlock;
		if (error)
			dma_fence_set_error(cur, error);
		ret = dma_fence_signal_locked(cur);
cont_unlock:
		spin_unlock_irqrestore(cur->lock, flags);
	}

	return ret;
}

static const char *sync_status_str(int status)
{
	if (status < 0)
		return "error";
	if (status > 0)
		return "signaled";
	return "active";
}

struct gcip_dma_fence_manager *gcip_dma_fence_manager_create(struct device *dev)
{
	struct gcip_dma_fence_manager *mgr = devm_kzalloc(dev, sizeof(*mgr), GFP_KERNEL);

	if (!mgr)
		return ERR_PTR(-ENOMEM);

	INIT_LIST_HEAD(&mgr->fence_list_head);
	spin_lock_init(&mgr->fence_list_lock);
	mgr->dev = dev;

	return mgr;
}

const char *gcip_dma_fence_get_timeline_name(struct dma_fence *fence)
{
	struct gcip_dma_fence *gfence = to_gfence(fence);

	return gfence->timeline_name;
}

bool gcip_dma_fence_always_true(struct dma_fence *fence)
{
	return true;
}

int gcip_dma_fence_init(struct gcip_dma_fence_manager *mgr, struct gcip_dma_fence *gfence,
			struct gcip_dma_fence_data *data)
{
	unsigned long flags;
	int fd;
	struct sync_file *sync_file;
	int ret;

	strscpy(gfence->timeline_name, data->timeline_name, GCIP_FENCE_TIMELINE_NAME_LEN);

	spin_lock_init(&gfence->lock);
	INIT_LIST_HEAD(&gfence->fence_list);
	gfence->mgr = mgr;

	dma_fence_init(&gfence->fence, data->ops, &gfence->lock, dma_fence_context_alloc(1),
		       data->seqno);
	GCIP_DMA_FENCE_LIST_LOCK(mgr, flags);
	list_add_tail(&gfence->fence_list, &mgr->fence_list_head);
	GCIP_DMA_FENCE_LIST_UNLOCK(mgr, flags);

	if (data->after_init) {
		ret = data->after_init(gfence);
		if (ret) {
			dev_err(mgr->dev, "DMA fence init failed on after_init: %d", ret);
			goto err_put_fence;
		}
	}
	fd = get_unused_fd_flags(O_CLOEXEC);
	if (fd < 0) {
		ret = fd;
		dev_err(mgr->dev, "Failed to get FD: %d", ret);
		goto err_put_fence;
	}
	sync_file = sync_file_create(&gfence->fence);
	if (!sync_file) {
		dev_err(mgr->dev, "Failed to create sync file");
		ret = -ENOMEM;
		goto err_put_fd;
	}
	/* sync_file holds the reference to fence, so we can drop our reference. */
	dma_fence_put(&gfence->fence);

	fd_install(fd, sync_file->file);
	data->fence = fd;
	return 0;

err_put_fd:
	put_unused_fd(fd);
err_put_fence:
	dma_fence_put(&gfence->fence);
	return ret;
}

void gcip_dma_fence_exit(struct gcip_dma_fence *gfence)
{
	unsigned long flags;

	GCIP_DMA_FENCE_LIST_LOCK(gfence->mgr, flags);
	list_del(&gfence->fence_list);
	GCIP_DMA_FENCE_LIST_UNLOCK(gfence->mgr, flags);
}

int gcip_dma_fence_status(int fence, int *status)
{
	struct dma_fence *fencep;

	fencep = sync_file_get_fence(fence);
	if (!fencep)
		return -EBADF;
	*status = dma_fence_get_status(fencep);
	dma_fence_put(fencep);
	return 0;
}

int gcip_dma_fence_signal(int fence, int error, bool ignore_signaled)
{
	struct dma_fence *fencep;
	int ret;

	fencep = sync_file_get_fence(fence);
	if (!fencep)
		return -EBADF;
	ret = gcip_signal_dma_fence_with_status(fencep, error, ignore_signaled);
	dma_fence_put(fencep);
	return ret;
}

int gcip_dma_fenceptr_signal(struct gcip_dma_fence *gfence, int error, bool ignore_signaled)
{
	return gcip_signal_dma_fence_with_status(&gfence->fence, error, ignore_signaled);
}

void gcip_dma_fence_show(struct gcip_dma_fence *gfence, struct seq_file *s)
{
	struct dma_fence *fence = &gfence->fence;

	spin_lock_irq(&gfence->lock);

	seq_printf(s, "%s-%s %llu-%llu %s", fence->ops->get_driver_name(fence),
		   fence->ops->get_timeline_name(fence), fence->context, fence->seqno,
		   sync_status_str(dma_fence_get_status_locked(fence)));

	if (test_bit(DMA_FENCE_FLAG_TIMESTAMP_BIT, &fence->flags)) {
		struct timespec64 ts = ktime_to_timespec64(fence->timestamp);

		seq_printf(s, " @%lld.%09ld", (s64)ts.tv_sec, ts.tv_nsec);
	}

	if (fence->error)
		seq_printf(s, " err=%d", fence->error);

	spin_unlock_irq(&gfence->lock);
}

struct dma_fence *gcip_dma_fence_merge_fences(int num_fences, struct dma_fence **fences)
{
	struct dma_fence *tmp;
	struct dma_fence *array;
	int i;
	int error;
	ktime_t timestamp;

	array = dma_fence_unwrap_merge(fences[0]);
	if (!array)
		return ERR_PTR(-ENOMEM);

	for (i = 1; i < num_fences; i++) {
		tmp = array;
		array = dma_fence_unwrap_merge(tmp, fences[i]);
		dma_fence_put(tmp);
		if (!array)
			return ERR_PTR(-ENOMEM);
	}

	/*
	 * When merging fences, any already signaled fences are thrown away, regardless of whether
	 * they were signaled with success or an error. Once the fences are merged however, the
	 * merged fence will inherit the first error from an internal fence.
	 *
	 * Re-iterate over the fences being merged and if any are already signaled with an error,
	 * pass that error on to the merged fence. This ensures waiters will not attempt to do work
	 * that should have been cancelled due to an in-fence error.
	 */
	for (i = 0; i < num_fences; i++) {
		error = dma_fence_get_status(fences[i]);
		if (error < 0) {
			timestamp = dma_fence_timestamp(fences[i]);
			dma_fence_put(array);
			array = dma_fence_allocate_private_stub(timestamp);
			array->error = error;
			break;
		}
	}

	return array;
}

struct dma_fence *gcip_dma_fence_merge_fds(int num_fences, int *fence_fds)
{
	struct dma_fence **fences;
	struct dma_fence *result;
	int i;

	if (!num_fences)
		return ERR_PTR(-EINVAL);

	fences = kcalloc(num_fences, sizeof(*fences), GFP_KERNEL);
	if (!fences)
		return ERR_PTR(-ENOMEM);

	for (i = 0; i < num_fences; i++) {
		fences[i] = sync_file_get_fence(fence_fds[i]);
		if (!fences[i]) {
			result = ERR_PTR(-ENOENT);
			goto out;
		}
	}

	result = gcip_dma_fence_merge_fences(num_fences, fences);

out:
	for (i = 0; i < num_fences; i++)
		dma_fence_put(fences[i]);
	kfree(fences);
	return result;
}

void gcip_dma_fence_array_disable_signaling(struct dma_fence *fence)
{
	struct dma_fence_array *array = container_of(fence, struct dma_fence_array, base);
	struct dma_fence_array_cb *cb = (void *)(&array[1]);
	unsigned long flags;
	int i;

	if (!dma_fence_is_array(fence))
		return;

	spin_lock_irqsave(fence->lock, flags);

	if (!test_bit(DMA_FENCE_FLAG_ENABLE_SIGNAL_BIT, &fence->flags))
		goto out;

	for (i = 0; i < array->num_fences; ++i) {
		if (dma_fence_remove_callback(array->fences[i], &cb[i].cb))
			dma_fence_put(&array->base);
	}

	clear_bit(DMA_FENCE_FLAG_ENABLE_SIGNAL_BIT, &fence->flags);

out:
	spin_unlock_irqrestore(fence->lock, flags);
}
