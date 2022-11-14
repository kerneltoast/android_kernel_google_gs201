// SPDX-License-Identifier: GPL-2.0-only
/* include/linux/kernel-top.h
 *
 * Copyright (C) 2022 Google, Inc.
 */

#ifndef _LINUX_KERNEL_TOP_FUNC_H
#define _LINUX_KERNEL_TOP_FUNC_H

struct kernel_top_context;

#if IS_ENABLED(CONFIG_KERNEL_TOP)
extern void kernel_top_print(struct kernel_top_context *cxt);
extern int kernel_top_init(struct device *dev, struct kernel_top_context **pcxt);
extern void kernel_top_reset(struct kernel_top_context *cxt);
extern void kernel_top_destroy(struct kernel_top_context *cxt);
#else
static inline void kernel_top_print(struct kernel_top_context *cxt)
{
}

static inline int kernel_top_init(struct device *dev, struct kernel_top_context **pcxt)
{
	return -EINVAL;
}

static inline void kernel_top_reset(struct kernel_top_context *cxt)
{
}

static inline void kernel_top_destroy(struct kernel_top_context *cxt)
{
}
#endif /* CONFIG_KERNEL_TOP */

#endif /* _LINUX_KERNEL_TOP_FUNC_H */
