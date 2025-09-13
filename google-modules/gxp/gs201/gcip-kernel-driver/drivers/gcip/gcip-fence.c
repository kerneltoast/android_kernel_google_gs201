// SPDX-License-Identifier: GPL-2.0-only
/*
 * Abstracted interface for fences.
 *
 * Copyright (C) 2023 Google LLC
 */

#include <linux/dma-fence.h>
#include <linux/slab.h>
#include <linux/sync_file.h>

#include <gcip/gcip-dma-fence.h>
#include <gcip/gcip-fence.h>
#include <iif/iif-fence.h>
#include <iif/iif-shared.h>
#include <iif/iif-signaler-submission-waiter.h>

static struct gcip_fence *gcip_fence_alloc(enum gcip_fence_type type)
{
	struct gcip_fence *fence = kzalloc(sizeof(*fence), GFP_KERNEL);

	if (!fence)
		return NULL;

	fence->type = type;
	kref_init(&fence->kref);

	return fence;
}

static void gcip_fence_do_free(struct gcip_fence *fence, void (*iif_put_func)(struct iif_fence *))
{
	switch (fence->type) {
	case GCIP_INTER_IP_FENCE:
		iif_put_func(fence->fence.iif);
		break;
	case GCIP_IN_KERNEL_FENCE:
		dma_fence_put(fence->fence.ikf);
		break;
	}

	kfree(fence);
}

static void gcip_fence_free(struct kref *kref)
{
	struct gcip_fence *fence = container_of(kref, struct gcip_fence, kref);

	gcip_fence_do_free(fence, &iif_fence_put);
}

static void gcip_fence_free_async(struct kref *kref)
{
	struct gcip_fence *fence = container_of(kref, struct gcip_fence, kref);

	gcip_fence_do_free(fence, &iif_fence_put_async);
}

static int gcip_fence_do_signal(struct gcip_fence *fence, int errno,
				int (*iif_signal_func)(struct iif_fence *, int))
{
	switch (fence->type) {
	case GCIP_INTER_IP_FENCE:
		return iif_signal_func(fence->fence.iif, errno);
	case GCIP_IN_KERNEL_FENCE:
		return gcip_signal_dma_fence_with_status(fence->fence.ikf, errno, false);
	}

	return -EOPNOTSUPP;
}

static void gcip_fence_do_waited(struct gcip_fence *fence, enum iif_ip_type ip,
				 void (*iif_waited_func)(struct iif_fence *, enum iif_ip_type))
{
	if (fence->type == GCIP_INTER_IP_FENCE)
		iif_waited_func(fence->fence.iif, ip);
}

static void gcip_fence_release_iif(struct iif_fence *iif_fence)
{
	kfree(iif_fence);
}

static const struct iif_fence_ops iif_fence_ops = {
	.on_release = gcip_fence_release_iif,
};

int gcip_fence_create_iif(struct iif_manager *mgr, enum iif_ip_type signaler_ip,
			  unsigned int total_signalers)
{
	struct iif_fence *iif_fence;
	enum iif_ip_type iif_signaler_ip = (enum iif_ip_type)signaler_ip;
	int fd, ret;

	if (!mgr)
		return -ENODEV;

	if (iif_signaler_ip >= IIF_IP_NUM)
		return -EINVAL;

	iif_fence = kzalloc(sizeof(*iif_fence), GFP_KERNEL);
	if (!iif_fence)
		return -ENOMEM;

	ret = iif_fence_init(mgr, iif_fence, &iif_fence_ops, iif_signaler_ip, total_signalers);
	if (ret) {
		kfree(iif_fence);
		return ret;
	}

	fd = iif_fence_install_fd(iif_fence);

	/*
	 * If `iif_fence_install_fd` succeeds, the IIF sync file holds a reference to the fence and
	 * it's fine to release one here.
	 * If it fails, `iif_fence_put` will release all reference counts and the release callback
	 * will be executed to free @fence.
	 */
	iif_fence_put(iif_fence);

	return fd;
}

static struct gcip_fence *gcip_fence_fdget_iif(int fd)
{
	struct gcip_fence *fence;
	struct iif_fence *iif_fence;

	iif_fence = iif_fence_fdget(fd);
	if (IS_ERR(iif_fence))
		return ERR_CAST(iif_fence);

	fence = gcip_fence_alloc(GCIP_INTER_IP_FENCE);
	if (!fence) {
		iif_fence_put(iif_fence);
		return ERR_PTR(-ENOMEM);
	}

	fence->fence.iif = iif_fence;

	return fence;
}

static struct gcip_fence *gcip_fence_fdget_ikf(int fd)
{
	struct gcip_fence *fence;
	struct dma_fence *dma_fence;

	dma_fence = sync_file_get_fence(fd);
	if (!dma_fence)
		return ERR_PTR(-EBADF);

	fence = gcip_fence_alloc(GCIP_IN_KERNEL_FENCE);
	if (!fence) {
		dma_fence_put(dma_fence);
		return ERR_PTR(-ENOMEM);
	}

	fence->fence.ikf = dma_fence;

	return fence;
}

struct gcip_fence *gcip_fence_fdget(int fd)
{
	struct gcip_fence *fence;

	fence = gcip_fence_fdget_iif(fd);
	if (!IS_ERR(fence))
		return fence;

	fence = gcip_fence_fdget_ikf(fd);
	if (!IS_ERR(fence))
		return fence;

	return ERR_PTR(-EINVAL);
}

struct gcip_fence *gcip_fence_get(struct gcip_fence *fence)
{
	if (fence)
		kref_get(&fence->kref);
	return fence;
}

struct gcip_fence *gcip_fence_get_iif(struct iif_fence *iif)
{
	struct gcip_fence *fence;

	fence = gcip_fence_alloc(GCIP_INTER_IP_FENCE);
	if (!fence)
		return ERR_PTR(-ENOMEM);

	fence->fence.iif = iif_fence_get(iif);

	return fence;
}

struct gcip_fence *gcip_fence_get_ikf(struct dma_fence *ikf)
{
	struct gcip_fence *fence;

	fence = gcip_fence_alloc(GCIP_IN_KERNEL_FENCE);
	if (!fence)
		return ERR_PTR(-ENOMEM);

	fence->fence.ikf = dma_fence_get(ikf);

	return fence;
}

void gcip_fence_put(struct gcip_fence *fence)
{
	if (fence)
		kref_put(&fence->kref, gcip_fence_free);
}

void gcip_fence_put_async(struct gcip_fence *fence)
{
	if (fence)
		kref_put(&fence->kref, gcip_fence_free_async);
}

int gcip_fence_submit_signaler(struct gcip_fence *fence)
{
	if (fence->type == GCIP_INTER_IP_FENCE)
		return iif_fence_submit_signaler(fence->fence.iif);
	return -EOPNOTSUPP;
}

int gcip_fence_submit_waiter(struct gcip_fence *fence, enum iif_ip_type ip)
{
	if (fence->type == GCIP_INTER_IP_FENCE)
		return iif_fence_submit_waiter(fence->fence.iif, ip);
	return -EOPNOTSUPP;
}

int gcip_fence_signal(struct gcip_fence *fence, int errno)
{
	return gcip_fence_do_signal(fence, errno, &iif_fence_signal_with_status);
}

int gcip_fence_signal_async(struct gcip_fence *fence, int errno)
{
	return gcip_fence_do_signal(fence, errno, &iif_fence_signal_with_status_async);
}

void gcip_fence_waited(struct gcip_fence *fence, enum iif_ip_type ip)
{
	gcip_fence_do_waited(fence, ip, &iif_fence_waited);
}

void gcip_fence_waited_async(struct gcip_fence *fence, enum iif_ip_type ip)
{
	gcip_fence_do_waited(fence, ip, &iif_fence_waited_async);
}

/*
 * A proxy callback which is compatible with iif-fence interface and will be called when
 * @iif_fence finishes the signaler submission. This callback simply redirects to @cb->func.
 */
static void
gcip_fence_iif_all_signaler_submitted(struct iif_fence *iif_fence,
				      struct iif_fence_all_signaler_submitted_cb *iif_cb)
{
	struct gcip_fence_all_signaler_submitted_cb *cb =
		container_of(iif_cb, struct gcip_fence_all_signaler_submitted_cb, iif_cb);

	cb->func(cb->fence, cb);
}

int gcip_fence_add_all_signaler_submitted_cb(struct gcip_fence *fence,
					     struct gcip_fence_all_signaler_submitted_cb *cb,
					     gcip_fence_all_signaler_submitted_cb_t func)
{
	/*
	 * If @fence is not IIF, let it always treat the situation as all signalers are
	 * already submitted.
	 */
	if (fence->type != GCIP_INTER_IP_FENCE)
		return -EPERM;

	cb->func = func;
	cb->fence = fence;
	INIT_LIST_HEAD(&cb->iif_cb.node);

	return iif_fence_add_all_signaler_submitted_callback(fence->fence.iif, &cb->iif_cb,
							     gcip_fence_iif_all_signaler_submitted);
}

bool gcip_fence_remove_all_signaler_submitted_cb(struct gcip_fence *fence,
						 struct gcip_fence_all_signaler_submitted_cb *cb)
{
	if (fence->type != GCIP_INTER_IP_FENCE)
		return true;
	return iif_fence_remove_all_signaler_submitted_callback(fence->fence.iif, &cb->iif_cb);
}

/* Returns the ID of @fence if @fence is IIF. */
int gcip_fence_get_iif_id(struct gcip_fence *fence)
{
	if (fence->type == GCIP_INTER_IP_FENCE)
		return fence->fence.iif->id;
	return -EINVAL;
}

int gcip_fence_wait_signaler_submission(struct gcip_fence **fences, int num_fences,
					unsigned int eventfd, int *remaining_signalers)
{
	struct iif_fence **iif_fences;
	int i, ret;

	iif_fences = kcalloc(num_fences, sizeof(*iif_fences), GFP_KERNEL);
	if (!iif_fences)
		return -ENOMEM;

	for (i = 0; i < num_fences; i++) {
		if (fences[i]->type != GCIP_INTER_IP_FENCE) {
			ret = -EINVAL;
			goto out;
		}
		iif_fences[i] = fences[i]->fence.iif;
	}

	ret = iif_wait_signaler_submission(iif_fences, num_fences, eventfd, remaining_signalers);
out:
	kfree(iif_fences);
	return ret;
}

int gcip_fence_get_status(struct gcip_fence *fence)
{
	switch (fence->type) {
	case GCIP_INTER_IP_FENCE:
		return iif_fence_get_signal_status(fence->fence.iif);
	case GCIP_IN_KERNEL_FENCE:
		return dma_fence_get_status(fence->fence.ikf);
	}

	return -EOPNOTSUPP;
}

void gcip_fence_iif_set_propagate_unblock(struct gcip_fence *fence)
{
	if (fence->type == GCIP_INTER_IP_FENCE)
		iif_fence_set_propagate_unblock(fence->fence.iif);
}
