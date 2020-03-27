/* SPDX-License-Identifier: GPL-2.0 */
/*
 * ION Memory Allocator exynos feature support
 *
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 */

#ifndef _ION_EXYNOS_H
#define _ION_EXYNOS_H

#include <uapi/linux/ion.h>

void ion_page_clean(struct page *pages, unsigned long size);

enum ion_exynos_heap_type {
	ION_EXYNOS_HEAP_TYPE_CARVEOUT = ION_HEAP_TYPE_CUSTOM + 1,
};

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

#if defined(CONFIG_ION_EXYNOS_CARVEOUT_HEAP)
int __init ion_carveout_heap_init(void);
void __exit ion_carveout_heap_exit(void);
#else
static inline int __init ion_carveout_heap_init(void)
{
	return 0;
}

#define ion_carveout_heap_exit() do { } while (0)
#endif

struct exynos_fdt_attrs {
	unsigned int alignment;
};

void exynos_fdt_setup(struct device *dev, struct exynos_fdt_attrs *attrs);

#define IONPREFIX "[Exynos][ION] "
#define perr(format, arg...) \
	pr_err(IONPREFIX format "\n", ##arg)

#define perrfn(format, arg...) \
	pr_err(IONPREFIX "%s: " format "\n", __func__, ##arg)

#define perrdev(dev, format, arg...) \
	dev_err(dev, IONPREFIX format "\n", ##arg)

#define perrfndev(dev, format, arg...) \
	dev_err(dev, IONPREFIX "%s: " format "\n", __func__, ##arg)

#endif
