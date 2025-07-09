/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * GXP local power management interface.
 * Controlling Local Power Manager hardware.
 *
 * Copyright (C) 2020 Google LLC
 */
#ifndef __GXP_LPM_H__
#define __GXP_LPM_H__

#include <linux/types.h>

#include "gxp-config.h"
#include "gxp.h"

enum lpm_state {
	LPM_ACTIVE_STATE = 0,
	LPM_CG_STATE = 1,
	LPM_PG_W_RET_STATE = 2,
	LPM_PG_STATE = 3,
};

enum psm_reg_offset {
	PSM_REG_ENABLE_STATE0_OFFSET,
	PSM_REG_ENABLE_STATE1_OFFSET,
	PSM_REG_ENABLE_STATE2_OFFSET,
	PSM_REG_ENABLE_STATE3_OFFSET,
	PSM_REG_START_OFFSET,
	PSM_REG_STATUS_OFFSET,
	PSM_REG_CFG_OFFSET,
	PSM_REG_DEBUG_CFG_OFFSET,
	PSM_REG_GPIN_LO_RD_OFFSET,
	PSM_REG_GPOUT_LO_WRT_OFFSET,
	PSM_REG_GPOUT_LO_RD_OFFSET,
};

#define LPM_INSTRUCTION_OFFSET 0x00000944
#define LPM_INSTRUCTION_MASK 0x03000000

#define LPM_HW_MODE 0
#define LPM_SW_PSM_MODE 1

#define LPM_CFG_SW_PS_TARGET_OFFSET 2

#define CORE_WAKEUP_DOORBELL(__core__) (0 + (__core__))

#define PSM_INIT_DONE_MASK	0x80
#define PSM_CURR_STATE_MASK	0x0F
#define PSM_STATE_VALID_MASK	0x10

#define PSM_HW_MODE		0x0
#define PSM_START		0x1

/*
 * Initializes the power manager for the first time after block power up.
 * The function needs to be called once after a block power up event.
 */
void gxp_lpm_init(struct gxp_dev *gxp);
/*
 * Destroys the power manager in preparation for a block shutdown.
 * The function needs to be called once before a block shutdown event.
 */
void gxp_lpm_destroy(struct gxp_dev *gxp);
/*
 * Turns on the power manager for a specific core. i.e. powers up the core.
 */
int gxp_lpm_up(struct gxp_dev *gxp, uint core);
/*
 * Turns off the power manager for a specific core. i.e. powers down the core.
 */
void gxp_lpm_down(struct gxp_dev *gxp, uint core);
/*
 * Return whether the specified PSM is initialized.
 * PSM0-PSM3 are for core0-core3, PSM4 is the TOP LPM.
 */
bool gxp_lpm_is_initialized(struct gxp_dev *gxp, enum gxp_lpm_psm psm);

/*
 * Return whether the specified PSM is powered.
 */
bool gxp_lpm_is_powered(struct gxp_dev *gxp, enum gxp_lpm_psm psm);

/*
 * Wait for the specified @psm to be in any state other than @state
 * Return whether the waiting is successful or the timeout occurs.
 */
bool gxp_lpm_wait_state_ne(struct gxp_dev *gxp, enum gxp_lpm_psm psm, uint state);

/*
 * Wait for the specified @psm to be in the specified @state
 * Return whether the waiting is successful or the timeout occurs.
 */
bool gxp_lpm_wait_state_eq(struct gxp_dev *gxp, enum gxp_lpm_psm psm, uint state);

/*
 * Force a state transition on the specified PSM.
 */
int gxp_lpm_set_state(struct gxp_dev *gxp, enum gxp_lpm_psm psm, uint target_state,
		      bool verbose);

/*
 * Get current LPM state of the specified PSM.
 */
uint gxp_lpm_get_state(struct gxp_dev *gxp, enum gxp_lpm_psm psm);

/*
 * Enable a state on the specified PSM.
 */
void gxp_lpm_enable_state(struct gxp_dev *gxp, enum gxp_lpm_psm psm, uint state);

static inline u32 lpm_read_32(struct gxp_dev *gxp, uint reg_offset)
{
#ifndef GXP_SEPARATE_LPM_OFFSET
	reg_offset = GXP_LPM_BASE + reg_offset;
#endif
	return readl(gxp->lpm_regs.vaddr + reg_offset);
}

static inline void lpm_write_32(struct gxp_dev *gxp, uint reg_offset, u32 value)
{
#ifndef GXP_SEPARATE_LPM_OFFSET
	reg_offset = GXP_LPM_BASE + reg_offset;
#endif
	writel(value, gxp->lpm_regs.vaddr + reg_offset);
}

static u32 get_reg_offset(struct gxp_dev *gxp, enum psm_reg_offset reg_offset, enum gxp_lpm_psm psm)
{
	switch (reg_offset) {
	case PSM_REG_ENABLE_STATE0_OFFSET:
	case PSM_REG_ENABLE_STATE1_OFFSET:
	case PSM_REG_ENABLE_STATE2_OFFSET:
	case PSM_REG_ENABLE_STATE3_OFFSET:
		return gxp_lpm_psm_get_state_offset(psm, (uint)reg_offset);
	case PSM_REG_START_OFFSET:
		return gxp_lpm_psm_get_start_offset(psm);
	case PSM_REG_STATUS_OFFSET:
		return gxp_lpm_psm_get_status_offset(psm);
	case PSM_REG_CFG_OFFSET:
		return gxp_lpm_psm_get_cfg_offset(psm);
	case PSM_REG_DEBUG_CFG_OFFSET:
		return gxp_lpm_psm_get_debug_cfg_offset(psm);
	case PSM_REG_GPIN_LO_RD_OFFSET:
		return gxp_lpm_psm_get_gpin_lo_rd_offset(psm);
	case PSM_REG_GPOUT_LO_WRT_OFFSET:
		return gxp_lpm_psm_get_gpout_lo_wrt_offset(psm);
	case PSM_REG_GPOUT_LO_RD_OFFSET:
		return gxp_lpm_psm_get_gpout_lo_rd_offset(psm);
	}

	return 0;
}

static inline u32 lpm_read_32_psm(struct gxp_dev *gxp, enum gxp_lpm_psm psm,
				  enum psm_reg_offset reg_offset)
{
	uint offset = get_reg_offset(gxp, reg_offset, psm);

	return lpm_read_32(gxp, offset);
}

static inline void lpm_write_32_psm(struct gxp_dev *gxp, enum gxp_lpm_psm psm,
				    enum psm_reg_offset reg_offset, u32 value)
{
	u32 offset = get_reg_offset(gxp, reg_offset, psm);

	lpm_write_32(gxp, offset, value);
}

#endif /* __GXP_LPM_H__ */
