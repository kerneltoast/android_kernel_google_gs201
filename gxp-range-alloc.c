// SPDX-License-Identifier: GPL-2.0
/*
 * GXP ranged resource allocator.
 *
 * Copyright (C) 2021 Google LLC
 */

#include "gxp-range-alloc.h"

struct range_alloc *range_alloc_create(int start, int end)
{
	struct range_alloc *ra;
	int count;
	int size;

	count = end - start;
	if (count <= 0)
		return ERR_PTR(-EINVAL);

	size = sizeof(struct range_alloc) + count * sizeof(int);
	ra = kzalloc(size, GFP_KERNEL);
	if (!ra)
		return ERR_PTR(-ENOMEM);

	ra->total_count = count;
	ra->free_count = count;
	ra->start_index = start;
	mutex_init(&ra->lock);

	return ra;
}

int range_alloc_get(struct range_alloc *r, int element)
{
	int index = element - r->start_index;

	mutex_lock(&r->lock);
	if (index < 0 || index >= r->total_count) {
		mutex_unlock(&r->lock);
		return -EINVAL;
	}

	if (r->elements[index]) {
		mutex_unlock(&r->lock);
		return -EBUSY;
	}

	r->elements[index] = 1;
	r->free_count--;

	mutex_unlock(&r->lock);
	return 0;
}

int range_alloc_get_any(struct range_alloc *r, int *element)
{
	int i;

	mutex_lock(&r->lock);
	if (!r->free_count) {
		mutex_unlock(&r->lock);
		return -ENOMEM;
	}

	for (i = 0; i < r->total_count; i++) {
		if (r->elements[i] == 0) {
			r->elements[i] = 1;
			r->free_count--;
			*element = i + r->start_index;
			mutex_unlock(&r->lock);
			return 0;
		}
	}
	mutex_unlock(&r->lock);
	return -ENOMEM;
}

int range_alloc_put(struct range_alloc *r, int element)
{
	int index = element - r->start_index;

	mutex_lock(&r->lock);
	if (index < 0 || index >= r->total_count) {
		mutex_unlock(&r->lock);
		return -EINVAL;
	}

	if (r->elements[index] == 0) {
		mutex_unlock(&r->lock);
		return -EBUSY;
	}

	r->elements[index] = 0;
	r->free_count++;

	mutex_unlock(&r->lock);
	return 0;
}

int range_alloc_num_free(struct range_alloc *r)
{
	int free_count;

	mutex_lock(&r->lock);
	free_count = r->free_count;
	mutex_unlock(&r->lock);

	return free_count;
}

int range_alloc_destroy(struct range_alloc *r)
{
	if (!r)
		return -EFAULT;
	kfree(r);

	return 0;
}
