/* SPDX-License-Identifier: GPL-2.0 */
/*
 * GXP ranged resource allocator.
 *
 * Copyright (C) 2021 Google LLC
 */
#ifndef __GXP_RANGE_ALLOC_H__
#define __GXP_RANGE_ALLOC_H__

#include <linux/mutex.h>
#include <linux/slab.h>

struct range_alloc {
	int total_count;
	int free_count;
	int start_index;
	struct mutex lock;
	int elements[];
};

/**
 * range_alloc_create() - Creates a range allocator starting at the specified
 *			  start (inclusive) and ends at the specified end
 *			  (exclusive).
 * @start: The start of the range (inclusive).
 * @end: The end of the range (exclusive)
 *
 * Return:
 * ptr     - A pointer of the newly created allocator handle on success, an
 *	     error pointer (PTR_ERR) otherwise.
 * -EINVAL - Invalid start/end combination
 * -ENOMEM - Insufficient memory to create the allocator
 */
struct range_alloc *range_alloc_create(int start, int end);

/**
 * range_alloc_get() - Gets the specified element from the range.
 * @r: The range allocator
 * @element: The element to acquire from the range
 *
 * The @element argument should be within the allocator's range and has not been
 * allocated before.
 *
 * Return:
 * 0       - Successfully reserved @element
 * -EINVAL - Invalid element index (negative or outside allocator range)
 * -EBUSY  - Element is already allocated
 */
int range_alloc_get(struct range_alloc *r, int element);

/**
 * range_alloc_get_any() - Gets any free element in the range.
 * @r: The range allocator
 * @element: A pointer to use to store the allocated element
 *
 * Return:
 * 0       - Successful reservation
 * -ENOMEM - No elements left in the range to allocate
 */
int range_alloc_get_any(struct range_alloc *r, int *element);

/**
 * range_alloc_put() - Puts an element back into the range.
 * @r: The range allocator
 * @element: The element to put back into the range
 *
 * Return:
 * 0       - Successful placement back into the range
 * -EINVAL - Invalid element index (negative or outside allocator range)
 * -EBUSY  - The element is still present in the range
 */
int range_alloc_put(struct range_alloc *r, int element);

/**
 * range_alloc_num_free() - Returns the number of free elements in the range.
 * @r: The range allocator
 *
 * Return: the number of free elements in the range
 */
int range_alloc_num_free(struct range_alloc *r);

/**
 * range_alloc_destroy() - Destroys the range allocator
 * @r: The range allocator to destroy
 *
 * The destruction does not validate that the range is empty.
 *
 * Return:
 * 0       - Successfully destroyed range allocator
 * -EFAULT - Invalid allocator address
 */
int range_alloc_destroy(struct range_alloc *r);

#endif /* __GXP_RANGE_ALLOC_H__ */
