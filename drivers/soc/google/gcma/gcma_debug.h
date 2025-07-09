/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __GCMA_DEBUG_FS_H__
#define __GCMA_DEBUG_FS_H__

#ifdef CONFIG_DEBUG_FS
int gcma_debugfs_init(void);
bool workingset_filter_enabled(void);
#else
int gcma_debugfs_init(void) { return 0; };
bool workingset_filter_enabled(void) { return true; };
#endif
#endif
