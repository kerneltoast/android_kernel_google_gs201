/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Active Load Tracer support for Exynos.
 *
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 */

#ifndef __EXYNOS_ALT_H_
#define __EXYNOS_ALT_H_

#if IS_ENABLED(CONFIG_EXYNOS_ALT_DVFS)
void exynos_alt_call_chain(void);
#else
#define exynos_alt_call_chain() do {} while (0)
#endif

#endif
