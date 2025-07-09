/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __GCMA_SYSFS_H__
#define __GCMA_SYSFS_H__

enum gcma_stat_type {
        STORED_PAGE,
        LOADED_PAGE,
        EVICTED_PAGE,
        CACHED_PAGE,
        DISCARDED_PAGE,
        NUM_OF_GCMA_STAT,
};

int gcma_sysfs_init(void);

void inc_gcma_stat(enum gcma_stat_type type);
void dec_gcma_stat(enum gcma_stat_type type);
void add_gcma_stat(enum gcma_stat_type type, unsigned long delta);
void account_gcma_per_page_alloc_latency(unsigned long count,
					 unsigned long latency_ns);
#endif
