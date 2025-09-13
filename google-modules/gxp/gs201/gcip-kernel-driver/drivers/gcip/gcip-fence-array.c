// SPDX-License-Identifier: GPL-2.0-only
/*
 * Interface for the array of abstracted fences.
 *
 * Copyright (C) 2023 Google LLC
 */

#include <linux/dma-fence.h>
#include <linux/kref.h>
#include <linux/slab.h>
#include <linux/string.h>

#include <gcip/gcip-dma-fence.h>
#include <gcip/gcip-fence-array.h>
#include <gcip/gcip-fence.h>
#include <iif/iif-fence.h>
#include <iif/iif-shared.h>

struct gcip_fence_array *gcip_fence_array_create(int *fences, int num_fences, bool check_same_type)
{
	int i, ret;
	struct gcip_fence_array *fence_array;
	struct gcip_fence *fence;

	if ((!fences && num_fences) || num_fences < 0)
		return ERR_PTR(-EINVAL);

	fence_array = kzalloc(sizeof(*fence_array), GFP_KERNEL);
	if (!fence_array)
		return ERR_PTR(-ENOMEM);

	fence_array->fences = kcalloc(num_fences, sizeof(*fence_array->fences), GFP_KERNEL);
	if (!fence_array->fences) {
		ret = -ENOMEM;
		goto err_free_fence_array;
	}

	fence_array->same_type = true;

	for (i = 0; i < num_fences; i++) {
		fence = gcip_fence_fdget(fences[i]);
		if (IS_ERR(fence)) {
			ret = PTR_ERR(fence);
			goto err_put_fences;
		}

		if (i && fence_array->same_type && fence->type != fence_array->fences[0]->type) {
			/* Check whether all fences are the same type. */
			if (check_same_type) {
				ret = -EINVAL;
				gcip_fence_put(fence);
				goto err_put_fences;
			}
			fence_array->same_type = false;
		}

		fence_array->fences[i] = fence;
	}

	if (i && fence_array->same_type)
		fence_array->type = fence_array->fences[0]->type;

	fence_array->size = i;
	kref_init(&fence_array->kref);

	return fence_array;

err_put_fences:
	while (i--)
		gcip_fence_put(fence_array->fences[i]);
	kfree(fence_array->fences);
err_free_fence_array:
	kfree(fence_array);

	return ERR_PTR(ret);
}

static void gcip_fence_array_do_free(struct gcip_fence_array *fence_array,
				     void (*gcip_fence_put_func)(struct gcip_fence *))
{
	int i;

	for (i = 0; i < fence_array->size; i++)
		gcip_fence_put_func(fence_array->fences[i]);
	kfree(fence_array->fences);
	kfree(fence_array);
}

static void gcip_fence_array_free(struct kref *kref)
{
	struct gcip_fence_array *fence_array = container_of(kref, struct gcip_fence_array, kref);

	gcip_fence_array_do_free(fence_array, &gcip_fence_put);
}

static void gcip_fence_array_free_async(struct kref *kref)
{
	struct gcip_fence_array *fence_array = container_of(kref, struct gcip_fence_array, kref);

	gcip_fence_array_do_free(fence_array, &gcip_fence_put_async);
}

struct gcip_fence_array *gcip_fence_array_get(struct gcip_fence_array *fence_array)
{
	if (!fence_array)
		return NULL;
	kref_get(&fence_array->kref);
	return fence_array;
}

void gcip_fence_array_put(struct gcip_fence_array *fence_array)
{
	if (fence_array)
		kref_put(&fence_array->kref, gcip_fence_array_free);
}

void gcip_fence_array_put_async(struct gcip_fence_array *fence_array)
{
	if (fence_array)
		kref_put(&fence_array->kref, gcip_fence_array_free_async);
}

void gcip_fence_array_signal(struct gcip_fence_array *fence_array, int errno)
{
	int i;

	if (!fence_array)
		return;

	for (i = 0; i < fence_array->size; i++)
		gcip_fence_signal(fence_array->fences[i], errno);
}

void gcip_fence_array_signal_async(struct gcip_fence_array *fence_array, int errno)
{
	int i;

	if (!fence_array)
		return;

	for (i = 0; i < fence_array->size; i++)
		gcip_fence_signal_async(fence_array->fences[i], errno);
}

void gcip_fence_array_waited(struct gcip_fence_array *fence_array, enum iif_ip_type ip)
{
	int i;

	if (!fence_array)
		return;

	for (i = 0; i < fence_array->size; i++)
		gcip_fence_waited(fence_array->fences[i], ip);
}

void gcip_fence_array_waited_async(struct gcip_fence_array *fence_array, enum iif_ip_type ip)
{
	int i;

	if (!fence_array)
		return;

	for (i = 0; i < fence_array->size; i++)
		gcip_fence_waited_async(fence_array->fences[i], ip);
}

void gcip_fence_array_submit_signaler(struct gcip_fence_array *fence_array)
{
	int i;

	if (!fence_array)
		return;

	for (i = 0; i < fence_array->size; i++)
		gcip_fence_submit_signaler(fence_array->fences[i]);
}

void gcip_fence_array_submit_waiter(struct gcip_fence_array *fence_array, enum iif_ip_type ip)
{
	int i;

	if (!fence_array)
		return;

	for (i = 0; i < fence_array->size; i++)
		gcip_fence_submit_waiter(fence_array->fences[i], ip);
}

int gcip_fence_array_submit_waiter_and_signaler(struct gcip_fence_array *in_fences,
						struct gcip_fence_array *out_fences,
						enum iif_ip_type ip)
{
	int i, ret, iif_num_in_fences = 0, iif_num_out_fences = 0;
	struct iif_fence **iif_in_fences = NULL;
	struct iif_fence **iif_out_fences = NULL;

	if (in_fences)
		iif_in_fences = kcalloc(in_fences->size, sizeof(*iif_in_fences), GFP_KERNEL);

	if (out_fences)
		iif_out_fences = kcalloc(out_fences->size, sizeof(*iif_out_fences), GFP_KERNEL);

	for (i = 0; in_fences && i < in_fences->size; i++) {
		if (in_fences->fences[i]->type == GCIP_INTER_IP_FENCE) {
			iif_in_fences[iif_num_in_fences] = in_fences->fences[i]->fence.iif;
			iif_num_in_fences++;
		}
	}

	for (i = 0; out_fences && i < out_fences->size; i++) {
		if (out_fences->fences[i]->type == GCIP_INTER_IP_FENCE) {
			iif_out_fences[iif_num_out_fences] = out_fences->fences[i]->fence.iif;
			iif_num_out_fences++;
		}
	}

	ret = iif_fence_submit_signaler_and_waiter(iif_in_fences, iif_num_in_fences, iif_out_fences,
						   iif_num_out_fences, ip);
	kfree(iif_out_fences);
	kfree(iif_in_fences);

	return ret;
}

uint16_t *gcip_fence_array_get_iif_id(struct gcip_fence_array *fence_array, int *num_iif,
				      bool out_fences, enum iif_ip_type signaler_ip)
{
	uint16_t *iif_fences;
	struct iif_fence *iif;
	int i, j;

	*num_iif = 0;

	if (!fence_array)
		return NULL;

	for (i = 0; i < fence_array->size; i++) {
		if (fence_array->fences[i]->type == GCIP_INTER_IP_FENCE) {
			iif = fence_array->fences[i]->fence.iif;
			if (out_fences && iif->signaler_ip != signaler_ip) {
				*num_iif = 0;
				return ERR_PTR(-EINVAL);
			}
			(*num_iif)++;
		}
	}

	if (!(*num_iif))
		return NULL;

	iif_fences = kcalloc(*num_iif, sizeof(*iif_fences), GFP_KERNEL);
	if (!iif_fences)
		return ERR_PTR(-ENOMEM);

	for (i = 0, j = 0; i < fence_array->size; i++) {
		if (fence_array->fences[i]->type == GCIP_INTER_IP_FENCE)
			iif_fences[j++] = gcip_fence_get_iif_id(fence_array->fences[i]);
	}

	return iif_fences;
}

int gcip_fence_array_wait_signaler_submission(struct gcip_fence_array *fence_array,
					      unsigned int eventfd, int *remaining_signalers)
{
	return gcip_fence_wait_signaler_submission(fence_array->fences, fence_array->size, eventfd,
						   remaining_signalers);
}

struct dma_fence *gcip_fence_array_merge_ikf(struct gcip_fence_array *fence_array)
{
	struct dma_fence **fences;
	struct dma_fence *merged;
	int i;

	if (!fence_array || !fence_array->size || !fence_array->same_type ||
	    fence_array->type != GCIP_IN_KERNEL_FENCE)
		return NULL;

	if (fence_array->size == 1)
		return dma_fence_get(fence_array->fences[0]->fence.ikf);

	fences = kcalloc(fence_array->size, sizeof(*fences), GFP_KERNEL);
	if (!fences)
		return ERR_PTR(-ENOMEM);

	for (i = 0; i < fence_array->size; i++)
		fences[i] = fence_array->fences[i]->fence.ikf;

	merged = gcip_dma_fence_merge_fences(fence_array->size, fences);

	kfree(fences);

	return merged;
}

void gcip_fence_array_iif_set_propagate_unblock(struct gcip_fence_array *fence_array)
{
	int i;

	for (i = 0; i < fence_array->size; i++)
		gcip_fence_iif_set_propagate_unblock(fence_array->fences[i]);
}

int gcip_fence_array_add(struct gcip_fence_array *fence_array, struct gcip_fence *fence)
{
	struct gcip_fence **fences;
	bool same_type = fence_array->same_type;

	if (same_type && fence_array->size && fence_array->fences[0]->type != fence->type)
		same_type = false;

	fences = krealloc_array(fence_array->fences, fence_array->size + 1, sizeof(*fences),
				GFP_KERNEL);
	if (!fences)
		return -ENOMEM;

	if (!fence_array->size)
		fence_array->type = fence->type;

	fences[fence_array->size] = gcip_fence_get(fence);
	fence_array->fences = fences;
	fence_array->same_type = same_type;
	fence_array->size++;

	return 0;
}

int gcip_fence_array_add_iif(struct gcip_fence_array *fence_array, struct iif_fence *iif)
{
	struct gcip_fence *fence;
	int ret;

	fence = gcip_fence_get_iif(iif);
	if (IS_ERR(fence))
		return PTR_ERR(fence);

	ret = gcip_fence_array_add(fence_array, fence);
	gcip_fence_put(fence);

	return ret;
}

int gcip_fence_array_add_ikf(struct gcip_fence_array *fence_array, struct dma_fence *ikf)
{
	struct gcip_fence *fence;
	int ret;

	fence = gcip_fence_get_ikf(ikf);
	if (IS_ERR(fence))
		return PTR_ERR(fence);

	ret = gcip_fence_array_add(fence_array, fence);
	gcip_fence_put(fence);

	return ret;
}
