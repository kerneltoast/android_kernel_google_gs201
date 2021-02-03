/* SPDX-License-Identifier: GPL-2.0 */
/*
 * ION Memory Allocator exynos feature support
 *
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 */

#ifndef _ION_EXYNOS_H
#define _ION_EXYNOS_H

#include <uapi/linux/ion.h>
#include "ion_exynos_prot.h"

void ion_page_clean(struct page *pages, unsigned long size);

enum ion_exynos_heap_type {
	ION_EXYNOS_HEAP_TYPE_CARVEOUT = ION_HEAP_TYPE_CUSTOM + 1,
};

#if defined(CONFIG_ION_EXYNOS_CMA_HEAP)
int __init ion_exynos_cma_heap_init(void);
void ion_exynos_cma_heap_exit(void);
#else
static inline int __init ion_exynos_cma_heap_init(void)
{
	return 0;
}

#define ion_exynos_cma_heap_exit() do { } while (0)
#endif

#if defined(CONFIG_ION_EXYNOS_CARVEOUT_HEAP)
int __init ion_carveout_heap_init(void);
void ion_carveout_heap_exit(void);
#else
static inline int __init ion_carveout_heap_init(void)
{
	return 0;
}

#define ion_carveout_heap_exit() do { } while (0)
#endif

struct exynos_fdt_attrs {
	unsigned int alignment;
	unsigned int protection_id;
	bool secure;
};

void exynos_fdt_setup(struct device *dev, struct exynos_fdt_attrs *attrs);

extern const struct dma_buf_ops exynos_dma_buf_ops;

#if IS_ENABLED(CONFIG_EXYNOS_CONTENT_PATH_PROTECTION)
void *ion_buffer_protect(struct device *dev, unsigned int protection_id,
			 unsigned int size, unsigned long phys,
			 unsigned int protalign);
int ion_buffer_unprotect(void *priv);
#else
static inline void *ion_buffer_protect(struct device *dev,
				       unsigned int protection_id,
				       unsigned int size, unsigned long phys,
				       unsigned int protalign)
{
	return NULL;
}

static inline int ion_buffer_unprotect(void *priv)
{
	return 0;
}
#endif

#if IS_ENABLED(CONFIG_EXYNOS_CONTENT_PATH_PROTECTION) && IS_ENABLED(CONFIG_EXYNOS_ITMON)
int ion_secure_itmon_init(void);
int ion_secure_itmon_exit(void);
#else
static inline int ion_secure_itmon_init(void)
{
	return 0;
}

static inline int ion_secure_itmon_exit(void)
{
	return 0;
}
#endif

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
