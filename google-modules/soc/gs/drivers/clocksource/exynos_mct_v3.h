/* SPDX-License-Identifier: GPL-2.0 */
/**
 * exynos_mct_v3.h - Samsung Exynos MCT(Multi-Core Timer) Driver Header file
 *
 * Copyright (C) 2022 Samsung Electronics Co., Ltd.
 */

#ifndef __EXYNOS_MCT_V3_H__
#define __EXYNOS_MCT_V3_H__

#define EXYNOS_MCTREG(x)		(x)
#define EXYNOS_MCT_MCT_CFG		EXYNOS_MCTREG(0x000)
#define EXYNOS_MCT_MCT_INCR_RTCCLK	EXYNOS_MCTREG(0x004)
#define EXYNOS_MCT_MCT_FRC_ENABLE	EXYNOS_MCTREG(0x100)
#define EXYNOS_MCT_CNT_L		EXYNOS_MCTREG(0x110)
#define EXYNOS_MCT_CNT_U		EXYNOS_MCTREG(0x114)
#define EXYNOS_MCT_CLKMUX_SEL		EXYNOS_MCTREG(0x120)
#define EXYNOS_MCT_COMPENSATE_VALUE	EXYNOS_MCTREG(0x124)
#define EXYNOS_MCT_VER			EXYNOS_MCTREG(0x128)
#define EXYNOS_MCT_DIVCHG_ACK		EXYNOS_MCTREG(0x12C)
#define EXYNOS_MCT_COMP_L(i)		EXYNOS_MCTREG(0x200 + ((i) * 0x100))
#define EXYNOS_MCT_COMP_U(i)		EXYNOS_MCTREG(0x204 + ((i) * 0x100))
#define EXYNOS_MCT_COMP_MODE(i)		EXYNOS_MCTREG(0x208 + ((i) * 0x100))
#define EXYNOS_MCT_COMP_PERIOD(i)	EXYNOS_MCTREG(0x20C + ((i) * 0x100))
#define EXYNOS_MCT_COMP_ENABLE(i)	EXYNOS_MCTREG(0x210 + ((i) * 0x100))
#define EXYNOS_MCT_INT_ENB(i)		EXYNOS_MCTREG(0x214 + ((i) * 0x100))
#define EXYNOS_MCT_INT_CSTAT(i)		EXYNOS_MCTREG(0x218 + ((i) * 0x100))

#define MCT_FRC_ENABLE			(0x1)
#define MCT_COMP_ENABLE			(0x1)
#define MCT_COMP_DISABLE		(0x0)

#define MCT_COMP_CIRCULAR_MODE		(0x1)
#define MCT_COMP_NON_CIRCULAR_MODE	(0x0)

#define MCT_INT_ENABLE			(0x1)
#define MCT_INT_DISABLE			(0x0)

#define MCT_CSTAT_CLEAR			(0x1)

#define MCT_DIV_REQ_BIT			(8)

#define DEFAULT_RTC_CLK_RATE		32768 // 32.768Khz
#define DEFAULT_CLK_DIV			3     // 1/3

#define WAIT_LOOP_CNT			(loops_per_jiffy / 1000 * HZ)

enum {
	/* There are 12 comparators which can produce interrupts */
	MCT_COMP0,
	MCT_COMP1,
	MCT_COMP2,
	MCT_COMP3,
	MCT_COMP4,
	MCT_COMP5,
	MCT_COMP6,
	MCT_COMP7,
	MCT_COMP8,
	MCT_COMP9,
	MCT_COMP10,
	MCT_COMP11,

	MCT_NR_COMPS,
};

struct mct_clock_event_device {
	struct clock_event_device evt;
	char name[10];
	unsigned int comp_index;
};

#endif /* __EXYNOS_MCT_V3_H__ */
