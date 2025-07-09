// SPDX-License-Identifier: GPL-2.0-only
/*
 * GXP Clock Management Unit interface.
 *
 * Copyright (C) 2024 Google LLC
 */

#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/types.h>

#include "gxp-cmu.h"
#include "gxp-config.h"
#include "gxp-internal.h"

/**
 * gxp_cmu_reg_read_32() - Read value of CMU register with specified offset.
 * @gxp: The gxp object which contains the CSR address information.
 * @offset: The offset of register from the CMU base address.
 *
 * Return: The value read from the register.
 */
static u32 gxp_cmu_reg_read_32(struct gxp_dev *gxp, u64 offset)
{
	return readl(gxp->cmu.vaddr + offset);
}

/**
 * gxp_cmu_reg_write_32() - Write value of CMU register with specified offset.
 * @gxp: The gxp object which contains the CSR address information.
 * @offset: The offset of register from the CMU base address.
 * @value: The value to set to the register.
 */
static void gxp_cmu_reg_write_32(struct gxp_dev *gxp, u64 offset, u32 value)
{
	writel(value, gxp->cmu.vaddr + offset);
}

int gxp_cmu_get_mux_state(struct gxp_dev *gxp, int mux_offset, enum gxp_cmu_mux_state *state)
{
	if (IS_ERR_OR_NULL(gxp->cmu.vaddr)) {
		dev_err(gxp->dev, "CMU registers are not mapped");
		return -ENODEV;
	}

	*state = gxp_cmu_reg_read_32(gxp, mux_offset);

	return 0;
}

int gxp_cmu_set_mux_state(struct gxp_dev *gxp, int mux_offset, enum gxp_cmu_mux_state state)
{
	if (IS_ERR_OR_NULL(gxp->cmu.vaddr)) {
		dev_err(gxp->dev, "CMU registers are not mapped");
		return -ENODEV;
	}

	if (state > 1) {
		dev_err(gxp->dev, "Incorrect state for mux state: %u", state);
		return -EINVAL;
	}

	gxp_cmu_reg_write_32(gxp, mux_offset, state << GXP_CMU_MUX_STATE_SHIFT);

	return 0;
}

void gxp_cmu_set_mux_normal(struct gxp_dev *gxp)
{
	gxp_cmu_set_mux_state(gxp, PLL_CON0_PLL_AUR, AUR_CMU_MUX_NORMAL);
	gxp_cmu_set_mux_state(gxp, PLL_CON0_NOC_USER, AUR_CMU_MUX_NORMAL);
}

void gxp_cmu_set_mux_low(struct gxp_dev *gxp)
{
	gxp_cmu_set_mux_state(gxp, PLL_CON0_PLL_AUR, AUR_CMU_MUX_LOW);
	gxp_cmu_set_mux_state(gxp, PLL_CON0_NOC_USER, AUR_CMU_MUX_LOW);
}

#define DEFINE_CMU_DEBUGFS(NAME, offset)					      \
	static int NAME##_set(void *data, u64 val)				      \
	{									      \
		return gxp_cmu_set_mux_state((struct gxp_dev *)data, offset, val);    \
	}									      \
										      \
	static int NAME##_get(void *data, u64 *val)				      \
	{									      \
		u32 status;							      \
		int ret;							      \
										      \
		ret = gxp_cmu_get_mux_state((struct gxp_dev *)data, offset, &status); \
		if (!ret)							      \
			*val = status;						      \
										      \
		return ret;							      \
	}									      \
										      \
	DEFINE_DEBUGFS_ATTRIBUTE(NAME##_fops, NAME##_get, NAME##_set, "%llu\n")

DEFINE_CMU_DEBUGFS(debugfs_cmu_mux1, PLL_CON0_PLL_AUR);
DEFINE_CMU_DEBUGFS(debugfs_cmu_mux2, PLL_CON0_NOC_USER);

void gxp_cmu_debugfs_init(struct gxp_dev *gxp)
{
	debugfs_create_file("cmumux1", 0600, gxp->d_entry, gxp, &debugfs_cmu_mux1_fops);
	debugfs_create_file("cmumux2", 0600, gxp->d_entry, gxp, &debugfs_cmu_mux2_fops);
}

int gxp_cmu_set_reg_resources(struct gxp_dev *gxp)
{
	struct platform_device *pdev = to_platform_device(gxp->dev);
	struct resource *r;
	void *vaddr;
	int ret;

	r = platform_get_resource_byname(pdev, IORESOURCE_MEM, "cmu");
	if (!r) {
		dev_warn(gxp->dev, "Failed to get CMU resources");
		return -ENODEV;
	}

	gxp->cmu.paddr = r->start;
	gxp->cmu.size = resource_size(r);
	vaddr = devm_ioremap_resource(gxp->dev, r);
	if (IS_ERR(vaddr)) {
		ret = PTR_ERR(vaddr);
		dev_err(gxp->dev, "Failed to map CMU registers from resource (ret=%d)", ret);
		return ret;
	}

	gxp->cmu.vaddr = vaddr;

	return 0;
}
