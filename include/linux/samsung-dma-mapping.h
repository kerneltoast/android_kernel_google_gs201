/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Samsung specific header for dma-mapping.
 *
 * Copyright (C) 2021 Samsung Electronics Co., Ltd.
 * Author: <hyesoo.yu@samsung.com> for Samsung
 */

#ifndef _SAMSUNG_DMA_MAPPING_H
#define _SAMSUNG_DMA_MAPPING_H

/**
 * List of possible attributes associated with a DMA mapping. Each attribute should not
 * overlap with bits defined in dma-mapping.h
 */

/*
 * DMA_ATTR_SKIP_LAZY_UNMAP: This tells the subsystem to do unmapping immediately
 * instead of trying lazy unmapping for performance when device virtual address
 * domain is not sufficient to use.
 */
#define DMA_ATTR_SKIP_LAZY_UNMAP	(1UL << 20)

#endif
