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
#include <linux/printk.h>	/* pr_xxx */
#include <linux/types.h>	/* uint32_t, __iomem, ... */
#include <linux/iopoll.h>
#include <linux/time.h>
#include <linux/platform_device.h>
#include <soc/google/exynos-el3_mon.h>
#include <video/mipi_display.h>
#include <drm/drm_print.h>
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

enum elem_size {
	ELEM_SIZE_16 = 16,
	ELEM_SIZE_32 = 32,
};

struct cal_regs_desc {
	const char *name;
	void __iomem *regs;
	volatile bool write_protected;
	phys_addr_t start;
};

/* common function macro for register control file */
/* to get cal_regs_desc */
#define cal_regs_desc_check(type, id, type_max, id_max)		\
	({ if (type > type_max || id > id_max) {		\
	 cal_log_err(id, "type(%d): id(%d)\n", type, id);	\
	 WARN_ON(1); }						\
	 })
#define cal_regs_desc_set(regs_desc, regs, start, name, type, id)	\
	({ regs_desc[type][id].regs = regs;				\
	 regs_desc[type][id].name = name;				\
	 regs_desc[type][id].start = start;				\
	 cal_log_debug(id, "name(%s) type(%d) regs(%p)\n", name, type, regs);\
	 })

/* SFR read/write */
static inline uint32_t cal_read(struct cal_regs_desc *regs_desc,
		uint32_t offset)
{
	return readl(regs_desc->regs + offset);
}

static inline void cal_write(struct cal_regs_desc *regs_desc,
		uint32_t offset, uint32_t val)
{
	if (unlikely(regs_desc->write_protected)) {
		int ret = set_priv_reg(regs_desc->start + offset, val);
		if (ret)
			pr_err("%s: smc update error %d for %llx\n", __func__,
				ret, regs_desc->start + offset);
	} else {
		writel(val, regs_desc->regs + offset);
	}
}

static inline uint32_t cal_read_relaxed(struct cal_regs_desc *regs_desc,
		uint32_t offset)
{
	return readl_relaxed(regs_desc->regs + offset);
}

static inline void cal_write_relaxed(struct cal_regs_desc *regs_desc,
		uint32_t offset, uint32_t val)
{
	if (unlikely(regs_desc->write_protected)) {
		int ret = set_priv_reg(regs_desc->start + offset, val);
		if (ret)
			pr_err("%s: smc update error %d for %llx\n", __func__,
				ret, regs_desc->start + offset);
	} else {
		writel_relaxed(val, regs_desc->regs + offset);
	}
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

/*
 * Packs an array of data points into a series of registers, where each register
 * contains 2 data points, with, optionally, an additional register containing
 * one data point. For example, packs a data array of 20 points into 10
 * registers, or 21 points into 11 registers.
 *
 * Note that the mask parameters are assumed to be pre-shifted within a u32
 * word by the caller. Low/high shift values for lut parameter are calculated
 * based on the first set bit in the corresponding masks.
 *
 * @lut: Array of points
 * @lut_len: The length of the array of points
 * @low_mask: Shifted register field mask for the first point in a register
 * @hi_mask: Shifted register field mask for the second point in a register
 * @regs: Output array of register values
 * @regs_len: Length of the output array fo register values
 */
static inline int cal_pack_lut_into_reg_pairs(const uint16_t *lut,
		const size_t lut_len, const uint32_t low_mask, const uint32_t hi_mask,
		uint32_t *regs, const size_t regs_len)
{
	int i;
	uint8_t low_shift;
	uint8_t hi_shift;
	const uint16_t *lp = lut;

	if (unlikely(regs == NULL || lut == NULL))
		return -ENOMEM;

	if (unlikely(DIV_ROUND_UP(lut_len, 2) != regs_len))
		return -EINVAL;

	if (unlikely(!low_mask || !hi_mask))
		return -EINVAL;

	low_shift = ffs(low_mask) - 1;
	hi_shift = ffs(hi_mask) - 1;
	for (i = 0; i < lut_len / 2; i++) {
		regs[i] = (*(lp++) << low_shift) & low_mask;
		regs[i] |= (*(lp++) << hi_shift) & hi_mask;
	}

	if (i < regs_len)
		regs[i] = ((*lp) << low_shift) & low_mask;

	return 0;
}

static inline void cal_set_write_protected(struct cal_regs_desc *regs_desc,
				     bool protected)
{
	regs_desc->write_protected = protected;
}

#define cal_mask(val, mask)	(((val) & (mask)) >> (ffs(mask) - 1))

void dpu_print_hex_dump(struct drm_printer *p, void __iomem *regs,
			const void *buf, size_t len);

/* log messages */
#define cal_msg(func, _id, fmt, ...)	\
	func("%s(#%d) " fmt, __func__, _id, ##__VA_ARGS__)

#define cal_log_enter(id)	cal_msg(pr_debug, id, "%s", "+")
#define cal_log_exit(id)	cal_msg(pr_debug, id, "%s", "-")

#define cal_log_debug(id, fmt, ...)		\
	cal_msg(pr_debug, id, fmt, ##__VA_ARGS__)
#define cal_log_warn(id, fmt, ...)	cal_msg(pr_warn, id, fmt, ##__VA_ARGS__)
#define cal_log_info(id, fmt, ...)	cal_msg(pr_info, id, fmt, ##__VA_ARGS__)
#define cal_log_err(id, fmt, ...)	cal_msg(pr_err, id, fmt, ##__VA_ARGS__)
#define cal_info_ratelimited(id, fmt, ...)	\
	cal_msg(pr_info_ratelimited, id, fmt, ##__VA_ARGS__)
#define cal_drm_printf(p, id, fmt, ...)		\
	drm_printf(p, "%s(#%d) " fmt, __func__, id, ##__VA_ARGS__)

#define GET_LUT_H(v)	(((v) >> 16) & 0xFFFF)
#define GET_LUT_L(v)	((v) & 0xFFFF)

#endif /* __CAL_OS_CONFIG_H__ */
