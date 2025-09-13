// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2022-2023 Google LLC.
 *
 * Author: Jack Diver <diverj@google.com>
 */
#ifndef _UAPI_PIXEL_MEMORY_GROUP_MANAGER_H_
#define _UAPI_PIXEL_MEMORY_GROUP_MANAGER_H_

void pixel_mgm_slc_update_signal(struct memory_group_manager_device* mgm_dev, u64 signal);

void pixel_mgm_slc_inc_refcount(struct memory_group_manager_device* mgm_dev);

void pixel_mgm_slc_dec_refcount(struct memory_group_manager_device* mgm_dev);

#endif /* _UAPI_PIXEL_MEMORY_GROUP_MANAGER_H_ */
