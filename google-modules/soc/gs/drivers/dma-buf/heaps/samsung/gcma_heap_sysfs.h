// SPDX-License-Identifier: GPL-2.0
/*
 * DMABUF GCMA heap sysfs
 *
 */

#ifndef __GCMA_HEAP_SYSFS_H
#define __GCMA_HEAP_SYSFS_H

enum stat_type {
	USAGE,
	ALLOCSTALL,
};

struct gcma_heap_stat;

#ifdef CONFIG_SYSFS
int __init gcma_heap_sysfs_init(void);
int register_heap_sysfs(struct gcma_heap *heap, const char *name);

void inc_gcma_heap_stat(struct gcma_heap *heap, enum stat_type type,
                        unsigned long size);
void dec_gcma_heap_stat(struct gcma_heap *heap, enum stat_type type,
                        unsigned long size);
#else
static inline int __init gcma_heap_sysfs_init(void) { return 0; };
static inline int register_heap_sysfs(struct gcma_heap *heap, const char *name)
{
	return 0;
}
static inline void inc_gcma_heap_stat(struct gcma_heap *heap,
                                      enum stat_type type,
                                      unsigned long size) {};
static inline void dec_gcma_heap_stat(struct gcma_heap *heap,
                                      enum stat_type type,
                                      unsigned long size) {};
#endif
#endif