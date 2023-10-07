// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2022-2023 Google LLC.
 *
 * Author: Jack Diver <diverj@google.com>
 */
#ifndef _UAPI_PIXEL_MEMORY_GROUP_MANAGER_H_
#define _UAPI_PIXEL_MEMORY_GROUP_MANAGER_H_

/**
 * enum pixel_mgm_group_id - Symbolic names for used memory groups
 */
enum pixel_mgm_group_id
{
	/* The Mali driver requires that allocations made on one of the groups
	 * are not treated specially.
	 */
	MGM_RESERVED_GROUP_ID = 0,

	/* Group for memory that should be cached in the system level cache. */
	MGM_SLC_GROUP_ID = 1,

	/* Group for memory explicitly allocated in SLC. */
	MGM_SLC_EXPLICIT_GROUP_ID = 2,

	/* Imported memory is handled by the allocator of the memory, and the Mali
	 * DDK will request a group_id for such memory via mgm_get_import_memory_id().
	 * We specify which group we want to use for this here.
	 */
	MGM_IMPORTED_MEMORY_GROUP_ID = (MEMORY_GROUP_MANAGER_NR_GROUPS - 1),
};

/**
 * pixel_mgm_query_group_size - Query the current size of a memory group
 *
 * @mgm_dev:   The memory group manager through which the request is being made.
 * @group_id:  Memory group to query.
 *
 * Returns the actual size of the memory group's active partition
 */
extern u64 pixel_mgm_query_group_size(struct memory_group_manager_device* mgm_dev,
                                      enum pixel_mgm_group_id group_id);

/**
 * pixel_mgm_resize_group_to_fit - Resize a memory group to meet @demand, if possible
 *
 * @mgm_dev:   The memory group manager through which the request is being made.
 * @group_id:  Memory group for which we will change the backing partition.
 * @demand:    The demanded space from the memory group.
 */
extern void pixel_mgm_resize_group_to_fit(struct memory_group_manager_device* mgm_dev,
                                          enum pixel_mgm_group_id group_id,
                                          u64 demand);

#endif /* _UAPI_PIXEL_MEMORY_GROUP_MANAGER_H_ */
