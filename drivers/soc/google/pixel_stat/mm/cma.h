/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __MM_PIXEL_CMA_H__
#define __MM_PIXEL_CMA_H__

struct kobject;

int pixel_mm_cma_sysfs(struct kobject *mm_kobj);
void vh_cma_alloc_start(void *data, s64 *ts);
void vh_cma_alloc_finish(void *data, struct cma *cma, struct page *page,
			 unsigned long count, unsigned int align,
			 gfp_t gfp_mask, s64 ts);
#endif
