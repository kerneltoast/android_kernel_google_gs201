/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2018 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * OS CAL configure file for Samsung EXYNOS Display Driver.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __CAL_OS_CONFIG_H__
#define __CAL_OS_CONFIG_H__

/* include headers */
#ifdef __linux__
#include <linux/io.h>		/* readl/writel */
#include <linux/delay.h>	/* udelay */
#include <linux/err.h>		/* EBUSY, EINVAL */
#include <linux/export.h>	/* EXPORT_SYMBOL */
#include <linux/printk.h>	/* pr_xxx */
#include <linux/types.h>	/* uint32_t, __iomem, ... */
#include <linux/iopoll.h>
#include <linux/time.h>
#include <video/mipi_display.h>
#else

/* TODO: Check with u-boot */
/* non-exist function define if required */
#ifndef readl
#define readl
#define writel
#endif

#ifndef __iomem
#define __iomem
#endif

#ifndef udelay
#define udelay
#endif

#ifndef WARN_ON
#define WARN_ON
#endif

#ifndef pr_info
#define pr_debug
#define pr_warn
#define pr_info
#define pr_err
#define pr_info_ratelimited
#endif
#endif

struct cal_regs_desc {
	const char *name;
	void __iomem *regs;
};

/* common function macro for register control file */
/* to get cal_regs_desc */
#define cal_regs_desc_check(type, id, type_max, id_max)		\
	({ if (type > type_max || id > id_max) {		\
	 cal_log_err(id, "type(%d): id(%d)\n", type, id);	\
	 WARN_ON(1); }						\
	 })
#define cal_regs_desc_set(regs_desc, regs, name, type, id)	\
	({ regs_desc[type][id].regs = regs;			\
	 regs_desc[type][id].name = name;			\
	 cal_log_debug(id, "name(%s) type(%d) regs(%p)\n", name, type, regs);\
	 })

/* SFR read/write */
static inline uint32_t cal_read(struct cal_regs_desc *regs_desc,
		uint32_t offset)
{
	uint32_t val = 0;

	val = readl(regs_desc->regs + offset);
	return val;
}

static inline void cal_write(struct cal_regs_desc *regs_desc,
		uint32_t offset, uint32_t val)
{
	writel(val, regs_desc->regs + offset);
}

static inline uint32_t cal_read_mask(struct cal_regs_desc *regs_desc,
		uint32_t offset, uint32_t mask)
{
	uint32_t val = cal_read(regs_desc, offset);

	val &= (mask);
	return val;
}

static inline void cal_write_mask(struct cal_regs_desc *regs_desc,
		uint32_t offset, uint32_t val, uint32_t mask)
{
	uint32_t old = cal_read(regs_desc, offset);

	val = (val & mask) | (old & ~mask);
	cal_write(regs_desc, offset, val);
}

/* log messages */
#define cal_msg(func, _id, fmt, ...)	\
	func("%s(#%d) " fmt, __func__, _id, ##__VA_ARGS__)

#define cal_log_enter(id)	cal_msg(pr_debug, id, "%s", "+")
#define cal_log_exit(id)	cal_msg(pr_debug, id, "%s", "-")

#define cal_log_debug(id, fmt, ...)		\
	cal_msg(pr_debug, id, fmt, ##__VA_ARGS__)
#define cal_log_warn(id, fmt, ...)	cal_msg(pr_info, id, fmt, ##__VA_ARGS__)
#define cal_log_info(id, fmt, ...)	cal_msg(pr_info, id, fmt, ##__VA_ARGS__)
#define cal_log_err(id, fmt, ...)	cal_msg(pr_err, id, fmt, ##__VA_ARGS__)
#define cal_info_ratelimited(id, fmt, ...)	\
	cal_msg(pr_info_ratelimited, id, fmt, ##__VA_ARGS__)

#endif /* __CAL_OS_CONFIG_H__ */
