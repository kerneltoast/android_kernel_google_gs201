// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2022-2023 Google LLC.
 *
 * Author: Jack Diver <diverj@google.com>
 */
#ifndef _UAPI_PIXEL_GPU_COMMON_SLC_H_
#define _UAPI_PIXEL_GPU_COMMON_SLC_H_

#include <linux/types.h>

/**
 * enum kbase_pixel_gpu_slc_liveness_mark_type - Determines the type of a live range mark
 *
 * @KBASE_PIXEL_GPU_LIVE_RANGE_BEGIN: Signifies that a mark is the start of a live range
 * @KBASE_PIXEL_GPU_LIVE_RANGE_END:   Signifies that a mark is the end of a live range
 *
 */
enum kbase_pixel_gpu_slc_liveness_mark_type {
	KBASE_PIXEL_GPU_LIVE_RANGE_BEGIN,
	KBASE_PIXEL_GPU_LIVE_RANGE_END,
};

/**
 * struct kbase_pixel_gpu_slc_liveness_mark - Live range marker
 *
 * @type: See @struct kbase_pixel_gpu_slc_liveness_mark_type
 * @index: Buffer index (within liveness update array) that this mark represents
 *
 */
struct kbase_pixel_gpu_slc_liveness_mark {
	__u32 type : 1;
	__u32 index : 31;
};

#endif /* _UAPI_PIXEL_GPU_COMMON_SLC_H_ */
