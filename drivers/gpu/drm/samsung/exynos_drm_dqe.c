// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020 Samsung Electronics Co.Ltd
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#include <dqe_cal.h>
#include <decon_cal.h>
#include <exynos_drm_decon.h>
#include <linux/of_address.h>

static void __exynos_dqe_update(struct exynos_dqe *dqe,
		struct exynos_dqe_state *state, u32 width, u32 height)
{
	pr_debug("%s +\n", __func__);

	if (!dqe->initialized) {
		decon_reg_set_dqe_enable(0, true);
		dqe_reg_init(width, height);
		dqe->initialized = true;
	}

	if (dqe->state.degamma_lut != state->degamma_lut) {
		dqe_reg_set_degamma_lut(state->degamma_lut);
		dqe->state.degamma_lut = state->degamma_lut;
	}

	if (dqe->state.cgc_lut != state->cgc_lut) {
		dqe_reg_set_cgc_lut(state->cgc_lut);
		dqe->state.cgc_lut = state->cgc_lut;
	}

	if (dqe->state.regamma_lut != state->regamma_lut) {
		dqe_reg_set_regamma_lut(state->regamma_lut);
		dqe->state.regamma_lut = state->regamma_lut;
	}

	/*
	 * Currently, the parameter of this function is fixed to zero because
	 * DECON0 only supports DQE. If other DECONs support DQE in the future,
	 * it needs to be modified.
	 */
	decon_reg_update_req_dqe(0);

	pr_debug("%s -\n", __func__);
}

static const struct exynos_dqe_funcs dqe_funcs = {
	.update = __exynos_dqe_update,
};

void exynos_dqe_update(struct exynos_dqe *dqe, struct exynos_dqe_state *state,
		u32 width, u32 height)
{
	dqe->funcs->update(dqe, state, width, height);
}

void exynos_dqe_reset(struct exynos_dqe *dqe)
{
	dqe->initialized = false;
	dqe->state.gamma_matrix = NULL;
	dqe->state.degamma_lut = NULL;
	dqe->state.linear_matrix = NULL;
	dqe->state.cgc_lut = NULL;
	dqe->state.regamma_lut = NULL;
}

struct exynos_dqe *exynos_dqe_register(struct decon_device *decon)
{
	struct device *dev = decon->dev;
	struct device_node *np = dev->of_node;
	struct exynos_dqe *dqe;
	int i;

	i = of_property_match_string(np, "reg-names", "dqe");
	if (i < 0) {
		pr_info("display quality enhancer is not supported\n");
		return NULL;
	}

	dqe = devm_kzalloc(dev, sizeof(struct exynos_dqe), GFP_KERNEL);
	if (!dqe)
		return NULL;

	dqe->regs = of_iomap(np, i);
	if (IS_ERR(dqe->regs)) {
		pr_err("failed to remap dqe registers\n");
		return NULL;
	}

	dqe_regs_desc_init(dqe->regs, "dqe");
	dqe->funcs = &dqe_funcs;
	dqe->initialized = false;

	pr_info("display quality enhancer is supported\n");

	return dqe;
}
