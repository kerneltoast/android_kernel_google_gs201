/* SPDX-License-Identifier: GPL-2.0 */

/*
 * Google LWIS Common Event Defines
 *
 * Copyright (c) 2020 Google, LLC
 */

#ifndef DT_BINDINGS_LWIS_PLATFORM_COMMON_H_
#define DT_BINDINGS_LWIS_PLATFORM_COMMON_H_

/* clang-format off */
#define CONCAT(X, Y) X##Y
#define MAKE64(X) X##ll

#define HW_EVENT_MASK 0x2000000000000000

/*
 * In kernel, it's just a 64-bit number, but in DT it's a pair of 32b vals
 * in the format of <upper32 lower32>.
 */
#ifdef __KERNEL__
#define EVENT_ID(x, y) (MAKE64(x) + MAKE64(y))
#else
#define EVENT_ID(x, y) \
	((((x) + (y)) >> (32)) & (0xFFFFFFFF)) (((x) + (y)) & (0xFFFFFFFF))
#endif
/* clang-format on */

/*
 * The following macro definition are used on test_device for userspace fake injection.
 */
#define TEST_EVENT_INJ 0
#define TEST_EVENT_BASE (HW_EVENT_MASK + 0)

#define LWIS_PLATFORM_EVENT_ID_TEST_EVENT_INJ \
	EVENT_ID(TEST_EVENT_BASE, TEST_EVENT_INJ)

#endif /* DT_BINDINGS_LWIS_PLATFORM_COMMON_H_ */
