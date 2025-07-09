/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * GXP support for DMA fence.
 *
 * Copyright (C) 2023 Google LLC
 */

#ifndef __GXP_DMA_FENCE_H__
#define __GXP_DMA_FENCE_H__

#include <gcip/gcip-dma-fence.h>

#include "gxp-vd.h"
#include "gxp.h"

/* Converts struct dma_fence to gxp_dma_fence. */
#define to_gxp_fence(fence)                                                    \
	container_of(to_gcip_fence(fence), struct gxp_dma_fence, gfence)

struct gxp_dma_fence {
	struct gcip_dma_fence gfence;
	/* The owner of this DMA fence */
	struct gxp_virtual_device *vd;
	/* List of DMA fences owned by the same VD. */
	struct list_head fence_list;
};

/*
 * Creates a DMA fence associates with @vd.
 *
 * @datap->fence is set to the fence FD on success.
 *
 * Returns 0 on success. Otherwise a negative errno.
 */
int gxp_dma_fence_create(struct gxp_dev *gxp, struct gxp_virtual_device *vd,
			 struct gxp_create_sync_fence_data *datap);

#endif /* __GXP_DMA_FENCE_H__ */
