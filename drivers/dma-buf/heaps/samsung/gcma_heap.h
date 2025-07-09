// SPDX-License-Identifier: GPL-2.0
/*
 * DMABUF GCMA heap
 *
 */

#ifndef __GCMA_HEAP_H
#define __GCMA_HEAP_H

struct gcma_heap {
        struct gen_pool *pool;
#ifdef CONFIG_SYSFS
        struct gcma_heap_stat *stat;
#endif
        bool flexible_alloc;
};


struct page *gcma_alloc(struct gcma_heap *gcma_heap, unsigned long size);
void gcma_free(struct gen_pool *pool, struct page *page);

#endif
