/* SPDX-License-Identifier: GPL-2.0 */
/*
 * GXP power management.
 *
 * Copyright (C) 2021 Google LLC
 */
#ifndef __GXP_PM_H__
#define __GXP_PM_H__

#include <soc/google/exynos_pm_qos.h>

#include "gxp-internal.h"

#define AUR_DVFS_MIN_RATE 178000
static const uint aur_power_state2rate[] = {
	0,		/* AUR_OFF */
	178000,		/* AUR_UUD */
	373000,		/* AUR_SUD */
	750000,		/* AUR_UD */
	1160000,	/* AUR_NOM */
	178000,		/* AUR_READY */
	268000,		/* AUR_UUD_PLUS */
	560000,		/* AUR_SUD_PLUS */
	975000,		/* AUR_UD_PLUS */
};

enum aur_power_state {
	AUR_OFF = 0,
	AUR_UUD = 1,
	AUR_SUD = 2,
	AUR_UD = 3,
	AUR_NOM = 4,
	AUR_READY = 5,
	AUR_UUD_PLUS = 6,
	AUR_SUD_PLUS = 7,
	AUR_UD_PLUS = 8,
};

enum aur_memory_power_state {
	AUR_MEM_UNDEFINED = 0,
	AUR_MEM_MIN = 1,
	AUR_MEM_VERY_LOW = 2,
	AUR_MEM_LOW = 3,
	AUR_MEM_HIGH = 4,
	AUR_MEM_VERY_HIGH = 5,
	AUR_MEM_MAX = 6,
};

enum aur_power_cmu_mux_state {
	AUR_CMU_MUX_LOW = 0,
	AUR_CMU_MUX_NORMAL = 1,
};

#define AUR_NUM_POWER_STATE (AUR_MAX_ALLOW_STATE + 1)
#define AUR_NUM_MEMORY_POWER_STATE (AUR_MAX_ALLOW_MEMORY_STATE + 1)

#define AUR_INIT_DVFS_STATE AUR_UUD

/*
 * These macros mean the maximum valid enum value of aur_power_state and
 * aur_memory_power_state, not necessarily the state with the maximum power
 * level.
 */
#define AUR_MAX_ALLOW_STATE AUR_UD_PLUS
#define AUR_MAX_ALLOW_MEMORY_STATE AUR_MEM_MAX

#define AUR_NUM_POWER_STATE_WORKER 4

struct gxp_pm_device_ops {
	int (*pre_blk_powerup)(struct gxp_dev *gxp);
	int (*post_blk_powerup)(struct gxp_dev *gxp);
	int (*pre_blk_poweroff)(struct gxp_dev *gxp);
	int (*post_blk_poweroff)(struct gxp_dev *gxp);
};

struct gxp_set_acpm_state_work {
	struct work_struct work;
	struct gxp_dev *gxp;
	unsigned long state;
	unsigned long prev_state;
	bool low_clkmux;
	bool prev_low_clkmux;
	bool using;
};

struct gxp_req_pm_qos_work {
	struct work_struct work;
	struct gxp_dev *gxp;
	s32 int_val;
	s32 mif_val;
	bool using;
};

struct gxp_power_manager {
	struct gxp_dev *gxp;
	struct mutex pm_lock;
	uint pwr_state_req_count[AUR_NUM_POWER_STATE];
	uint low_clkmux_pwr_state_req_count[AUR_NUM_POWER_STATE];
	uint mem_pwr_state_req_count[AUR_NUM_MEMORY_POWER_STATE];
	/*
	 * Last set CLKMUX state by asynchronous request handler.
	 * If a core is booting, we shouldn't change clock mux state. This is
	 * the expected state to set after all cores booting are finished.
	 * Otherwise, it's the real state of CLKMUX.
	 */
	bool curr_low_clkmux;
	/* Last requested clock mux state */
	bool last_scheduled_low_clkmux;
	int curr_state;
	int curr_memory_state;
	struct gxp_pm_device_ops *ops;
	struct gxp_set_acpm_state_work
		set_acpm_state_work[AUR_NUM_POWER_STATE_WORKER];
	/* Serializes searching for an open worker in set_acpm_state_work[] */
	struct mutex set_acpm_state_work_lock;
	uint last_set_acpm_state_worker;
	struct gxp_req_pm_qos_work req_pm_qos_work[AUR_NUM_POWER_STATE_WORKER];
	uint last_req_pm_qos_worker;
	/* Serializes searching for an open worker in req_pm_qos_work[] */
	struct mutex req_pm_qos_work_lock;
	struct workqueue_struct *wq;
	/* INT/MIF requests for memory bandwidth */
	struct exynos_pm_qos_request int_min;
	struct exynos_pm_qos_request mif_min;
	int force_mux_normal_count;
	/* Max frequency that the thermal driver/ACPM will allow in Hz */
	unsigned long thermal_limit;
	u64 blk_switch_count;
};

/**
 * gxp_pm_blk_on() - Turn on the power for BLK_AUR
 * @gxp: The GXP device to turn on
 *
 * Return:
 * * 0       - BLK ON successfully
 * * -ENODEV - Cannot find PM interface
 */
int gxp_pm_blk_on(struct gxp_dev *gxp);

/**
 * gxp_pm_blk_off() - Turn off the power for BLK_AUR
 * @gxp: The GXP device to turn off
 *
 * Return:
 * * 0       - BLK OFF successfully
 * * -ENODEV - Cannot find PM interface
 * * -EBUSY  - Wakelock is held, blk is still busy
 */
int gxp_pm_blk_off(struct gxp_dev *gxp);

/**
 * gxp_pm_get_blk_state() - Get the blk power state
 * @gxp: The GXP device to sample state
 *
 * Return:
 * * state   - State number represented in kHZ, or 0 if OFF
 */
int gxp_pm_get_blk_state(struct gxp_dev *gxp);

/**
 * gxp_pm_get_blk_switch_count() - Get the blk switch count number
 * @gxp: The GXP device to switch the blk
 *
 * Return:
 * * count   - Switch count number after the module initialization.
 */
int gxp_pm_get_blk_switch_count(struct gxp_dev *gxp);

/**
 * gxp_pm_core_on() - Turn on a core on GXP device
 * @gxp: The GXP device to operate
 * @core: The core ID to turn on
 * @verbose: A boolean flag to indicate whether to print the log
 *
 * Return:
 * * 0       - Core on process finished successfully
 * * -ETIMEDOUT - Core on process timed-out.
 */
int gxp_pm_core_on(struct gxp_dev *gxp, uint core, bool verbose);

/**
 * gxp_pm_core_off() - Turn off a core on GXP device
 * @gxp: The GXP device to operate
 * @core: The core ID to turn off
 *
 * Return:
 * * 0       - Core off process finished successfully
 */
int gxp_pm_core_off(struct gxp_dev *gxp, uint core);

/**
 * gxp_pm_init() - API for initialize PM interface for GXP, should only be
 * called once per probe
 * @gxp: The GXP device to operate
 *
 * Return:
 * * 0       - Initialization finished successfully
 * * -ENOMEM - Cannot get memory to finish init.
 */
int gxp_pm_init(struct gxp_dev *gxp);

/**
 * gxp_pm_destroy() - API for removing
 * the power management interface
 * @gxp: The GXP device to operate
 *
 * Return:
 * * 0       - Remove finished successfully
 */
int gxp_pm_destroy(struct gxp_dev *gxp);

/**
 * gxp_pm_blk_set_rate_acpm() - API for setting the block-level DVFS rate.
 * This function can be called at any point after block power on.
 * @gxp: The GXP device to operate
 * @rate: Rate number in khz that need to be set.
 *         Supported rate is in aur_power_state2rate,
 *         if experiment is needed for unsupported rate
 *         please refer to Lassen's ECT table.
 *
 * Return:
 * * 0       - Set finished successfully
 * * Other   - Set rate encounter issue in exynos_acpm_set_rate
 */
int gxp_pm_blk_set_rate_acpm(struct gxp_dev *gxp, unsigned long rate);

/**
 * gxp_pm_blk_get_state_acpm() - API for getting
 * the current DVFS state of the Aurora block.
 * @gxp: The GXP device to operate
 *
 * Return:
 * * State   - State number in Khz from ACPM
 */
int gxp_pm_blk_get_state_acpm(struct gxp_dev *gxp);

/**
 * gxp_pm_update_requested_power_states() - API for a GXP client to vote for a
 * requested power state and a requested memory power state.
 * @gxp: The GXP device to operate.
 * @origin_state: An existing old requested state, will be cleared. If this is
 *                the first vote, pass AUR_OFF.
 * @origin_requested_low_clkmux: Specify whether the existing vote was requested with
 *                               low frequency CLKMUX flag.
 * @requested_state: The new requested state.
 * @requested_low_clkmux: Specify whether the new vote is requested with low frequency
 *			  CLKMUX flag. Will take no effect if the @requested state is
 *			  AUR_OFF.
 * @origin_mem_state: An existing old requested state, will be cleared. If this is
 *                the first vote, pass AUR_MEM_UNDEFINED.
 * @requested_mem_state: The new requested state.
 *
 * Return:
 * * 0       - Voting registered
 * * -EINVAL - Invalid original state or requested state
 */

int gxp_pm_update_requested_power_states(
	struct gxp_dev *gxp, enum aur_power_state origin_state,
	bool origin_requested_low_clkmux, enum aur_power_state requested_state,
	bool requested_low_clkmux, enum aur_memory_power_state origin_mem_state,
	enum aur_memory_power_state requested_mem_state);

/*
 * gxp_pm_force_clkmux_normal() - Force PLL_CON0_NOC_USER and PLL_CON0_PLL_AUR MUX
 * switch to the normal state. This is required to guarantee LPM works when the core
 * is starting the firmware.
 */
void gxp_pm_force_clkmux_normal(struct gxp_dev *gxp);

/*
 * gxp_pm_resume_clkmux() - Check PLL_CON0_NOC_USER and PLL_CON0_PLL_AUR MUX state
 * modified by gxp_pm_force_clkmux_normal(). If the current vote is requested with low
 * frequency CLKMUX flag, should set the MUX state to AUR_CMU_MUX_LOW.
 */
void gxp_pm_resume_clkmux(struct gxp_dev *gxp);

/**
 * gxp_pm_set_thermal_limit() - Notify the power manager of a thermal limit
 * @gxp: The GXP device the limit is set for
 * @thermal_limit: The highest frequency, in Hz, the thermal limit allows
 *
 * The power management code will only use this information for logging.
 */
void gxp_pm_set_thermal_limit(struct gxp_dev *gxp, unsigned long thermal_limit);

#endif /* __GXP_PM_H__ */
