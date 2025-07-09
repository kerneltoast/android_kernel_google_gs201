/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * GXP Clock Management Unit interface.
 *
 * Copyright (C) 2024 Google LLC
 */

#ifndef __GXP_CMU_H__
#define __GXP_CMU_H__

#include <linux/platform_device.h>

#include "gxp-internal.h"

#define GXP_CMU_MUX_STATE_SHIFT 4

enum gxp_cmu_mux_state {
	AUR_CMU_MUX_LOW = 0,
	AUR_CMU_MUX_NORMAL = 1,
};

/**
 * gxp_cmu_get_mux_state() - Gets CMU mux state.
 * @gxp: The gxp object which contains the CSR address information.
 * @mux_offset: The address offset to specify the mux to read.
 * @state: The pointer used to return the state.
 *
 * Return:
 * * 0      - The CMU mux state has been read successfully.
 * * Others - Negative errno.
 */
int gxp_cmu_get_mux_state(struct gxp_dev *gxp, int mux_offset, enum gxp_cmu_mux_state *state);

/**
 * gxp_cmu_set_mux_state() - Sets CMU mux state.
 * @gxp: The gxp object which contains the CSR address information.
 * @mux_offset: The address offset to specify the mux to write.
 * @state: The state to set to the CMU mux.
 *
 * Return:
 * * 0      - The CMU mux state has been written successfully.
 * * Others - Negative errno.
 */
int gxp_cmu_set_mux_state(struct gxp_dev *gxp, int mux_offset, enum gxp_cmu_mux_state state);

/**
 * gxp_cmu_set_mux_normal() - Sets the CMU mux PLL_AUR and NOC_USER to AUR_CMU_MUX_NORMAL.
 * @gxp: The gxp object which contains the CSR address information.
 */
void gxp_cmu_set_mux_normal(struct gxp_dev *gxp);

/**
 * gxp_cmu_set_mux_low() - Sets the CMU mux PLL_AUR and NOC_USER to AUR_CMU_MUX_LOW.
 * @gxp: The gxp object which contains the CSR address information.
 */
void gxp_cmu_set_mux_low(struct gxp_dev *gxp);

/**
 * gxp_cmu_debugfs_init() - Creates debugfs nodes for CMU mux.
 * @gxp: The gxp object which contains the debugfs node information.
 *
 * Create debugfs nodes for PLL_AUR and NOC_USER.
 */
void gxp_cmu_debugfs_init(struct gxp_dev *gxp);

/**
 * gxp_cmu_set_reg_resources() - Maps the CMU registers.
 * @gxp: The gxp object to get dev and store the resources.
 *
 * Return:
 * * 0      - The CMU registers are mapped successfully.
 * * Others - Negative errno.
 */
int gxp_cmu_set_reg_resources(struct gxp_dev *gxp);

#endif /* __GXP_CMU_H__ */
