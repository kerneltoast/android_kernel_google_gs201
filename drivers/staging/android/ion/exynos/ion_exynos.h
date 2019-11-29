/* SPDX-License-Identifier: GPL-2.0 */
/*
 * ION Memory Allocator exynos feature support
 *
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 */

#ifndef _ION_EXYNOS_H
#define _ION_EXYNOS_H

void ion_page_clean(struct page *pages, unsigned long size);

#if defined(CONFIG_ION_EXYNOS_CMA_HEAP)
int __init ion_exynos_cma_heap_init(void);
void __exit ion_exynos_cma_heap_exit(void);
#else
static inline int __init ion_exynos_cma_heap_init(void)
{
	return 0;
}

#define ion_exynos_cma_heap_exit() do { } while (0)
#endif
#endif
