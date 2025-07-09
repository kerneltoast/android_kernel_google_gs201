// SPDX-License-Identifier: GPL-2.0-only
/*
 * GXP local power management interface.
 *
 * Copyright (C) 2021 Google LLC
 */

#include <linux/bitops.h>
#include <linux/io.h>
#include <linux/pm_runtime.h>
#include <linux/types.h>

#include "gxp-config.h"
#include "gxp-doorbell.h"
#include "gxp-internal.h"
#include "gxp-lpm.h"

#if IS_GXP_TEST
#define gxp_lpm_wait_until(ret, ...) ((ret) = true)
#else
#define gxp_lpm_wait_until(ret, lpm_state, condition)                                  \
	do {                                                                           \
		int i = 100000;                                                        \
		while (i) {                                                            \
			lpm_state = lpm_read_32_psm(gxp, psm, PSM_REG_STATUS_OFFSET) & \
				    PSM_CURR_STATE_MASK;                               \
			if (condition)                                                 \
				break;                                                 \
			udelay(2 * GXP_TIME_DELAY_FACTOR);                             \
			i--;                                                           \
		}                                                                      \
		ret = (i != 0);                                                        \
	} while (0)
#endif

bool gxp_lpm_is_initialized(struct gxp_dev *gxp, enum gxp_lpm_psm psm)
{
	u32 status = lpm_read_32_psm(gxp, psm, PSM_REG_STATUS_OFFSET);

	/*
	 * state_valid bit goes active and stays high forever the first time you
	 * write the start register
	 */
	if (status & PSM_STATE_VALID_MASK)
		return true;

	return false;
}

bool gxp_lpm_is_powered(struct gxp_dev *gxp, enum gxp_lpm_psm psm)
{
	u32 status = lpm_read_32_psm(gxp, psm, PSM_REG_STATUS_OFFSET);
	u32 state;

	if (!(status & PSM_STATE_VALID_MASK))
		return false;
	state = status & PSM_CURR_STATE_MASK;
	return state == LPM_ACTIVE_STATE || state == LPM_CG_STATE;
}

uint gxp_lpm_get_state(struct gxp_dev *gxp, enum gxp_lpm_psm psm)
{
	u32 status = lpm_read_32_psm(gxp, psm, PSM_REG_STATUS_OFFSET);

	return status & PSM_CURR_STATE_MASK;
}

#if GXP_AUTO_PSM && !IS_GXP_TEST
void gxp_lpm_enable_state(struct gxp_dev *gxp, enum gxp_lpm_psm psm, uint state)
{
}
int gxp_lpm_set_state(struct gxp_dev *gxp, enum gxp_lpm_psm psm, uint target_state,
		      bool verbose)
{
	uint curr_state = gxp_lpm_get_state(gxp, psm);

	if (curr_state != target_state)
		dev_warn(gxp->dev, "Auto PSM %u is in invalid state. target: PS%u, actual: PS%u,\n",
			 psm, target_state, gxp_lpm_get_state(gxp, psm));
	return 0;
}
void gxp_lpm_init(struct gxp_dev *gxp)
{
}
void gxp_lpm_destroy(struct gxp_dev *gxp)
{
}
int gxp_lpm_up(struct gxp_dev *gxp, uint core)
{
	/* Clear wakeup doorbell */
	gxp_doorbell_clear(gxp, CORE_WAKEUP_DOORBELL(core));
	return 0;
}
#else
void gxp_lpm_enable_state(struct gxp_dev *gxp, enum gxp_lpm_psm psm, uint state)
{
	/* PS0 should always be enabled */
	if (state == LPM_ACTIVE_STATE || state > LPM_PG_STATE)
		return;

	/* Disable all low power states */
	lpm_write_32_psm(gxp, psm, PSM_REG_ENABLE_STATE1_OFFSET, 0x0);
	lpm_write_32_psm(gxp, psm, PSM_REG_ENABLE_STATE2_OFFSET, 0x0);
	lpm_write_32_psm(gxp, psm, PSM_REG_ENABLE_STATE3_OFFSET, 0x0);

	/* Enable the requested low power state */
	lpm_write_32_psm(gxp, psm, state, 0x1);
}

static int set_state_internal(struct gxp_dev *gxp, enum gxp_lpm_psm psm, uint target_state)
{
	u32 val;
	int i = 10000;

	/* Set SW sequencing mode and PS target */
	val = LPM_SW_PSM_MODE;
	val |= target_state << LPM_CFG_SW_PS_TARGET_OFFSET;
	lpm_write_32_psm(gxp, psm, PSM_REG_CFG_OFFSET, val);

	/* Start the SW sequence */
	lpm_write_32_psm(gxp, psm, PSM_REG_START_OFFSET, 0x1);

	/* Wait for LPM init done (0x60041688) */
	while (i && !(lpm_read_32_psm(gxp, psm, PSM_REG_STATUS_OFFSET)
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

int gxp_lpm_set_state(struct gxp_dev *gxp, enum gxp_lpm_psm psm, uint target_state,
		      bool verbose)
{
	uint curr_state = gxp_lpm_get_state(gxp, psm);

	if (curr_state == target_state)
		return 0;

	if (verbose)
		dev_warn(gxp->dev,
			 "Forcing a transition to PS%u on core%u, status: %x\n",
			 target_state, psm,
			 lpm_read_32_psm(gxp, psm, PSM_REG_STATUS_OFFSET));

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
			lpm_read_32_psm(gxp, psm, PSM_REG_STATUS_OFFSET));

	/* Set HW sequencing mode */
	lpm_write_32_psm(gxp, psm, PSM_REG_CFG_OFFSET, LPM_HW_MODE);

	return 0;
}

static int psm_enable(struct gxp_dev *gxp, enum gxp_lpm_psm psm)
{
	int i = 10000;

	/* Return early if LPM is already initialized */
	if (gxp_lpm_is_initialized(gxp, psm)) {
		if (psm != LPM_PSM_TOP) {
			/* Ensure core is in PS3 */
			return gxp_lpm_set_state(gxp, psm, LPM_PG_STATE,
						 /*verbose=*/true);
		}

		return 0;
	}

	/* Write PSM start bit */
	lpm_write_32_psm(gxp, psm, PSM_REG_START_OFFSET, PSM_START);

	/* Wait for LPM init done (0x60041688) */
	while (i && !(lpm_read_32_psm(gxp, psm, PSM_REG_STATUS_OFFSET)
		      & PSM_INIT_DONE_MASK)) {
		udelay(1 * GXP_TIME_DELAY_FACTOR);
		i--;
	}

	if (!i)
		return 1;

	/* Set PSM to HW mode (0x60041680) */
	lpm_write_32_psm(gxp, psm, PSM_REG_CFG_OFFSET, PSM_HW_MODE);

	return 0;
}

void gxp_lpm_init(struct gxp_dev *gxp)
{
	if (gxp->lpm_init)
		gxp->lpm_init(gxp);

	/* Enable Top PSM */
	if (psm_enable(gxp, LPM_PSM_TOP))
		dev_err(gxp->dev, "Timed out when enabling Top PSM!\n");
}

void gxp_lpm_destroy(struct gxp_dev *gxp)
{
	/* (b/171063370) Put Top PSM in ACTIVE state before block shutdown */
	dev_dbg(gxp->dev, "Kicking Top PSM out of ACG\n");

	/* Disable all low-power states for TOP */
	lpm_write_32_psm(gxp, LPM_PSM_TOP, PSM_REG_ENABLE_STATE1_OFFSET, 0x0);
	lpm_write_32_psm(gxp, LPM_PSM_TOP, PSM_REG_ENABLE_STATE2_OFFSET, 0x0);
}

int gxp_lpm_up(struct gxp_dev *gxp, uint core)
{
	/* Clear wakeup doorbell */
	gxp_doorbell_clear(gxp, CORE_WAKEUP_DOORBELL(core));

	/* Enable core PSM */
	if (psm_enable(gxp, CORE_TO_PSM(core))) {
		dev_err(gxp->dev, "Timed out when enabling Core%u PSM!\n",
			core);
		return -ETIMEDOUT;
	}

	/* Enable PS1 (Clk Gated). Only required for core PSMs. */
	if (core < GXP_NUM_CORES)
		gxp_lpm_enable_state(gxp, CORE_TO_PSM(core), LPM_CG_STATE);

	return 0;
}
#endif /* GXP_AUTO_PSM && !IS_GXP_TEST */

void gxp_lpm_down(struct gxp_dev *gxp, uint core)
{
	if (gxp_lpm_get_state(gxp, CORE_TO_PSM(core)) == LPM_PG_STATE)
		return;
	/* Enable PS3 (Pwr Gated) */
	gxp_lpm_enable_state(gxp, CORE_TO_PSM(core), LPM_PG_STATE);

	/* Set wakeup doorbell to trigger an automatic transition to PS3 */
	gxp_doorbell_enable_for_core(gxp, CORE_WAKEUP_DOORBELL(core), core);
	gxp_doorbell_set(gxp, CORE_WAKEUP_DOORBELL(core));
	msleep(25 * GXP_TIME_DELAY_FACTOR);

	/*
	 * Clear the core's interrupt mask and the wakeup doorbell to ensure
	 * the core will not wake unexpectedly.
	 */
	gxp_write_32(gxp, GXP_REG_CORE_COMMON_INT_MASK_0(core), 0);
	gxp_doorbell_clear(gxp, CORE_WAKEUP_DOORBELL(core));

	/* Ensure core is in PS3 */
	gxp_lpm_set_state(gxp, CORE_TO_PSM(core), LPM_PG_STATE, /*verbose=*/true);
}

bool gxp_lpm_wait_state_ne(struct gxp_dev *gxp, enum gxp_lpm_psm psm, uint state)
{
	__maybe_unused uint lpm_state;
	bool ret;

	gxp_lpm_wait_until(ret, lpm_state, lpm_state != state);

	return ret;
}

bool gxp_lpm_wait_state_eq(struct gxp_dev *gxp, enum gxp_lpm_psm psm, uint state)
{
	__maybe_unused uint lpm_state;
	bool ret;

	gxp_lpm_wait_until(ret, lpm_state, lpm_state == state);

	return ret;
}
