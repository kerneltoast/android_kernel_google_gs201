/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2018-2020  Google, LLC.
 */

#ifndef __BLDR_DEBUG_TOOLS_H__
#define __BLDR_DEBUG_TOOLS_H__

#include <linux/types.h>

#if IS_ENABLED(CONFIG_PSTORE_CONSOLE_BLDR_LOG)
#define BOOT_DEBUG_MAGIC	0xAACCBBDD
ssize_t bldr_log_read_once(char __user *userbuf, ssize_t klog_size);
ssize_t bldr_last_log_read_once(char __user *userbuf, ssize_t klog_size);
ssize_t bldr_log_read(const void *lastk_buf, ssize_t lastk_size, char __user *userbuf, size_t count,
		      loff_t *ppos);
ssize_t bldr_log_total_size(void);
int bldr_log_init(void);
void bldr_log_release(void);
#else
static inline ssize_t bldr_log_read_once(char __user *userbuf, ssize_t klog_size) { return 0; }
static inline ssize_t bldr_last_log_read_once(char __user *userbuf, ssize_t klog_size) { return 0; }
static inline ssize_t bldr_log_read(const void *lastk_buf, ssize_t lastk_size, char __user *userbuf,
				    size_t count, loff_t *ppos) { return 0; }
static inline ssize_t bldr_log_total_size(void) { return 0; }
static inline int bldr_log_init(void) { return 0; }
static inline void bldr_log_release(void) { return; }
#endif

#endif /* __BLDR_DEBUG_TOOLS_H__ */
