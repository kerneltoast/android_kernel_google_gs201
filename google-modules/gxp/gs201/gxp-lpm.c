// SPDX-License-Identifier: GPL-2.0
/*
 * GXP local power management interface.
 *
 * Copyright (C) 2021 Google LLC
 */

#include <linux/acpm_dvfs.h>
#include <linux/bitops.h>
#include <linux/io.h>
#include <linux/pm_runtime.h>
#include <linux/types.h>

#include "gxp-bpm.h"
#include "gxp-doorbell.h"
#include "gxp-internal.h"
#include "gxp-lpm.h"

#define gxp_lpm_wait_until(lpm_state, condition)                               \
	do {                                                                   \
		int i = 100000;                                                \
		while (i) {                                                    \
			lpm_state =                                            \
				lpm_read_32_psm(gxp, psm, PSM_STATUS_OFFSET) & \
				PSM_CURR_STATE_MASK;                           \
			if (condition)                                         \
				break;                                         \
			udelay(1 * GXP_TIME_DELAY_FACTOR);                     \
			i--;                                                   \
		}                                                              \
		return i != 0;                                                 \
	} while (0)

void gxp_lpm_enable_state(struct gxp_dev *gxp, uint psm, uint state)
{
	uint offset = LPM_REG_ENABLE_STATE_0 + (LPM_STATE_TABLE_SIZE * state);

	/* PS0 should always be enabled */
	if (state == 0)
		return;

	/* Disable all low power states */
	lpm_write_32_psm(gxp, psm, LPM_REG_ENABLE_STATE_1, 0x0);
	lpm_write_32_psm(gxp, psm, LPM_REG_ENABLE_STATE_2, 0x0);
	lpm_write_32_psm(gxp, psm, LPM_REG_ENABLE_STATE_3, 0x0);

	/* Enable the requested low power state */
	lpm_write_32_psm(gxp, psm, offset, 0x1);
}

bool gxp_lpm_is_initialized(struct gxp_dev *gxp, uint psm)
{
	u32 status = lpm_read_32_psm(gxp, psm, PSM_STATUS_OFFSET);

	/*
	 * state_valid bit goes active and stays high forever the first time you
	 * write the start register
	 */
	if (status & PSM_STATE_VALID_MASK)
		return true;

	return false;
}

bool gxp_lpm_is_powered(struct gxp_dev *gxp, uint psm)
{
	u32 status = lpm_read_32_psm(gxp, psm, PSM_STATUS_OFFSET);
	u32 state;

	if (!(status & PSM_STATE_VALID_MASK))
		return false;
	state = status & PSM_CURR_STATE_MASK;
	return state == LPM_ACTIVE_STATE || state == LPM_CG_STATE;
}

uint gxp_lpm_get_state(struct gxp_dev *gxp, uint psm)
{
	u32 status = lpm_read_32_psm(gxp, psm, PSM_STATUS_OFFSET);

	return status & PSM_CURR_STATE_MASK;
}

static int set_state_internal(struct gxp_dev *gxp, uint psm, uint target_state)
{
	u32 val;
	int i = 10000;

	/* Set SW sequencing mode and PS target */
	val = LPM_SW_PSM_MODE;
	val |= target_state << LPM_CFG_SW_PS_TARGET_OFFSET;
	lpm_write_32_psm(gxp, psm, PSM_CFG_OFFSET, val);

	/* Start the SW sequence */
	lpm_write_32_psm(gxp, psm, PSM_START_OFFSET, 0x1);

	/* Wait for LPM init done (0x60041688) */
	while (i && !(lpm_read_32_psm(gxp, psm, PSM_STATUS_OFFSET)
		      & PSM_INIT_DONE_MASK)) {
		udelay(1 * GXP_TIME_DELAY_FACTOR);
		i--;
	}

	if (!i) {
		dev_err(gxp->dev, "Failed to switch PSM%u to PS%u\n", psm, target_state);
		return -EIO;
	}

	return 0;
}

int gxp_lpm_set_state(struct gxp_dev *gxp, uint psm, uint target_state,
		      bool verbose)
{
	uint curr_state = gxp_lpm_get_state(gxp, psm);

	if (curr_state == target_state)
		return 0;

	if (verbose)
		dev_warn(gxp->dev,
			 "Forcing a transition to PS%u on core%u, status: %x\n",
			 target_state, psm,
			 lpm_read_32_psm(gxp, psm, PSM_STATUS_OFFSET));

	gxp_lpm_enable_state(gxp, psm, target_state);

	if ((curr_state != LPM_ACTIVE_STATE)
	    && (target_state != LPM_ACTIVE_STATE)) {
		/* Switch to PS0 before switching to a low power state. */
		set_state_internal(gxp, psm, LPM_ACTIVE_STATE);
	}

	set_state_internal(gxp, psm, target_state);

	if (verbose)
		dev_warn(
			gxp->dev,
			"Finished forced transition on core %u.  target: PS%u, actual: PS%u, status: %x\n",
			psm, target_state, gxp_lpm_get_state(gxp, psm),
			lpm_read_32_psm(gxp, psm, PSM_STATUS_OFFSET));

	/* Set HW sequencing mode */
	lpm_write_32_psm(gxp, psm, PSM_CFG_OFFSET, LPM_HW_MODE);

	return 0;
}

static int psm_enable(struct gxp_dev *gxp, uint psm)
{
	int i = 10000;

	/* Return early if LPM is already initialized */
	if (gxp_lpm_is_initialized(gxp, psm)) {
		if (psm != LPM_TOP_PSM) {
			/* Ensure core is in PS3 */
			return gxp_lpm_set_state(gxp, psm, LPM_PG_STATE,
						 /*verbose=*/true);
		}

		return 0;
	}

	/* Write PSM start bit */
	lpm_write_32_psm(gxp, psm, PSM_START_OFFSET, PSM_START);

	/* Wait for LPM init done (0x60041688) */
	while (i && !(lpm_read_32_psm(gxp, psm, PSM_STATUS_OFFSET)
		      & PSM_INIT_DONE_MASK)) {
		udelay(1 * GXP_TIME_DELAY_FACTOR);
		i--;
	}

	if (!i)
		return 1;

	/* Set PSM to HW mode (0x60041680) */
	lpm_write_32_psm(gxp, psm, PSM_CFG_OFFSET, PSM_HW_MODE);

	return 0;
}

void gxp_lpm_init(struct gxp_dev *gxp)
{
	/* Enable Top PSM */
	if (psm_enable(gxp, LPM_TOP_PSM))
		dev_err(gxp->dev, "Timed out when enabling Top PSM!\n");
}

void gxp_lpm_destroy(struct gxp_dev *gxp)
{
	/* (b/171063370) Put Top PSM in ACTIVE state before block shutdown */
	dev_dbg(gxp->dev, "Kicking Top PSM out of ACG\n");

	/* Disable all low-power states for TOP */
	lpm_write_32_psm(gxp, LPM_TOP_PSM, LPM_REG_ENABLE_STATE_1, 0x0);
	lpm_write_32_psm(gxp, LPM_TOP_PSM, LPM_REG_ENABLE_STATE_2, 0x0);
}

int gxp_lpm_up(struct gxp_dev *gxp, uint core)
{
	/* Clear wakeup doorbell */
	gxp_doorbell_clear(gxp, CORE_WAKEUP_DOORBELL(core));

	/* Enable core PSM */
	if (psm_enable(gxp, core)) {
		dev_err(gxp->dev, "Timed out when enabling Core%u PSM!\n",
			core);
		return -ETIMEDOUT;
	}

	/* Enable PS1 (Clk Gated) */
	gxp_lpm_enable_state(gxp, core, LPM_CG_STATE);

	gxp_bpm_start(gxp, core);

	return 0;
}

void gxp_lpm_down(struct gxp_dev *gxp, uint core)
{
	if (gxp_lpm_get_state(gxp, core) == LPM_PG_STATE)
		return;
	/* Enable PS3 (Pwr Gated) */
	gxp_lpm_enable_state(gxp, core, LPM_PG_STATE);

	/* Set wakeup doorbell to trigger an automatic transition to PS3 */
	gxp_doorbell_enable_for_core(gxp, CORE_WAKEUP_DOORBELL(core), core);
	gxp_doorbell_set(gxp, CORE_WAKEUP_DOORBELL(core));
	msleep(25 * GXP_TIME_DELAY_FACTOR);

	/*
	 * Clear the core's interrupt mask and the wakeup doorbell to ensure
	 * the core will not wake unexpectedly.
	 */
	gxp_write_32_core(gxp, core, GXP_REG_COMMON_INT_MASK_0, 0);
	gxp_doorbell_clear(gxp, CORE_WAKEUP_DOORBELL(core));

	/* Ensure core is in PS3 */
	gxp_lpm_set_state(gxp, core, LPM_PG_STATE, /*verbose=*/true);
}

bool gxp_lpm_wait_state_ne(struct gxp_dev *gxp, uint psm, uint state)
{
	uint lpm_state;

	gxp_lpm_wait_until(lpm_state, lpm_state != state);
}

bool gxp_lpm_wait_state_eq(struct gxp_dev *gxp, uint psm, uint state)
{
	uint lpm_state;

	gxp_lpm_wait_until(lpm_state, lpm_state == state);
}
