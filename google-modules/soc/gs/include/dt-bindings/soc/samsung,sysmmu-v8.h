/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2014 Samsung Electronics Co., Ltd.
 *
 * Device Tree binding constants for Samsung System MMU.
 */

#ifndef _DT_BINDINGS_SAMSUNG_SYSMMU_V8_H
#define _DT_BINDINGS_SAMSUNG_SYSMMU_V8_H

/* define for fetchsize in TLB_CFG */
#define BL1			(0x0 << 5)
#define BL2			(0x1 << 5)
#define BL4			(0x2 << 5)
#define BL8			(0x3 << 5)
#define BL16			(0x4 << 5)
#define BL32			(0x5 << 5)
#define BL64			(0x6 << 5)

/* define for prefetch in TLB_CFG */
#define PREFETCH_NONE		(0x0 << 1)
#define PREFETCH_DESCENDING	(0x1 << 1)
#define PREFETCH_ASCENDING	(0x3 << 1)
#define PREFETCH_PREDICTION	(0x5 << 1)

/* combine fetchsize and prefetch */
#define TLB_CFG(fetchsize, prefetch)	((fetchsize) | (prefetch))
#define TLB_CFG_DEFAULT		0x0

/* define for direction in TLB_MATCH_CFG */
#define DIR_NONE		(0x0 << 8)
#define DIR_READ		(0x1 << 8)
#define DIR_WRITE		(0x2 << 8)
#define DIR_RW			(0x3 << 8)

/* define for TLB_MATCH_SID */
#define SYSMMU_ID_MASK(id, mask)	((mask) << 16 | (id))
#define SYSMMU_NOID			0

#endif /* _DT_BINDINGS_SAMSUNG_SYSMMU_V8_H */
