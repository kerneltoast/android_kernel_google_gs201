/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __GCMA_VH_H__
#define __GCMA_VH_H__

#ifdef CONFIG_ANDROID_VENDOR_HOOKS
int gcma_vh_init(void);
void inc_gcma_total_pages(unsigned long page_count);
#else
static inline int gcma_vh_init(void) { return 0; };
static void inc_gcma_total_pages(unsigned long page_count) { };
#endif
#endif
