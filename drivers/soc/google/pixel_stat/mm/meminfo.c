// SPDX-License-Identifier: GPL-2.0-only
/* cma.c
 *
 * Android Vendor Hook Support
 *
 * Copyright 2021 Google LLC
 */

#include <linux/module.h>
#include <linux/samsung-dma-heap.h>
#include <linux/seq_file.h>

/*****************************************************************************/
/*                       Modified Code Section                               */
/*****************************************************************************/
/*
 * This part of code is vendor hook functions, which modify or extend the
 * original functions.
 */

void vh_meminfo_proc_show(void *data, struct seq_file *m)
{
	seq_printf(m, "ION_heap:       %8lu kB\n",
		   dma_heap_inuse_pages() * PAGE_SIZE / 1024);
	seq_printf(m, "ION_heap_pool:  %8lu kB\n",
		   dma_heap_pool_pages() * PAGE_SIZE / 1024);
}
