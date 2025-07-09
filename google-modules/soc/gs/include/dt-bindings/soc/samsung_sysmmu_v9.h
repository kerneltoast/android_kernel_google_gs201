/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022 Samsung Electronics Co., Ltd.
 *
 * Device Tree binding constants for Samsung SysMMU v9.
 */

#ifndef _DT_BINDINGS_SAMSUNG_SYSMMU_V9_H
#define _DT_BINDINGS_SAMSUNG_SYSMMU_V9_H

/* Shared and private TLB IDs in STREAM_CFG */
#define STREAM_STLB_ID(stlb)		(((stlb) &  0xFF) << 24)
#define STREAM_PTLB_ID(ptlb)		(((ptlb) &  0xFF) << 16)
#define STR_SPTLB(stlb, ptlb)		(STREAM_STLB_ID(stlb) | STREAM_PTLB_ID(ptlb))

/* Stage-2 prefetch enable in STREAM_CFG */
#define S2_EN				(0x1 << 8)
#define S2_DIS				(0x0 << 8)

/* Fetch size in STREAM_CFG */
#define BL1				(0x0 << 4)
#define BL2				(0x1 << 4)
#define BL4				(0x2 << 4)
#define BL8				(0x3 << 4)
#define BL16				(0x4 << 4)
#define BL32				(0x5 << 4)
#define BL64				(0x6 << 4)

/* Prefetch direction in STREAM_CFG */
#define PREF_DES			(0x0 << 2)
#define PREF_ASC			(0x1 << 2)
#define PREF_PRED			(0x2 << 2)

/* STLB prefetch enable in STREAM_CFG */
#define STLB_NONE			(0x0 << 1)
#define STLB_PREF			(0x1 << 1)

/* PTLB prefetch enable in STREAM_CFG */
#define PTLB_NONE			(0x0)
#define PTLB_PREF			(0x1)

/* combine all */
#define STR_CFG(tlb_id, s2prefetch, fetchsize, prefetch, stlb_pre, ptlb_pre)	\
		((tlb_id)   | (s2prefetch) | (fetchsize) |	\
		 (prefetch) | (stlb_pre)   | (ptlb_pre))

/* Stream entry validity in STREAM_MATCH_CFG */
#define STREAM_INVALID			(0x0)
#define STREAM_VALID			(0x1)

/* Direction in STREAM_MATCH_CFG */
#define DIR_NONE			((0x0 << 8) | (STREAM_VALID))
#define DIR_READ			((0x1 << 8) | (STREAM_VALID))
#define DIR_WRITE			((0x2 << 8) | (STREAM_VALID))
#define DIR_RW				((0x3 << 8) | (STREAM_VALID))

/* STREAM_SID_VALUE & STREAM_MATCH_SID_MASK */
#define STR_MASK(mask)			((mask) & 0x7FF)
#define STR_ID(id)			((id) & 0x7FF)
#define STR_NOID			(0x0)

#endif /* _DT_BINDINGS_SAMSUNG_SYSMMU_V9_H */
