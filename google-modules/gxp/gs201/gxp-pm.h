/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * GXP power management.
 *
 * Copyright (C) 2021 Google LLC
 */
#ifndef __GXP_PM_H__
#define __GXP_PM_H__

#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/workqueue.h>

#include <gcip/gcip-pm.h>

#include "gxp-internal.h"

#define AUR_DVFS_MIN_RATE AUR_PERCENT_FREQUENCY_5_RATE

struct bcl_device;

/*
 * Kernel driver to use percentage based power states going ahead. State based power states
 * are aliased to percentage based power states for keeping the backward compatibility.
 */
enum aur_power_state {
	AUR_OFF = 0,
	AUR_READY = 1,
	AUR_PERCENT_FREQUENCY_5 = 2,
	AUR_PERCENT_FREQUENCY_10 = 3,
	AUR_PERCENT_FREQUENCY_15 = 4,
	AUR_PERCENT_FREQUENCY_20 = 5,
	AUR_PERCENT_FREQUENCY_25 = 6,
	AUR_PERCENT_FREQUENCY_30 = 7,
	AUR_PERCENT_FREQUENCY_35 = 8,
	AUR_PERCENT_FREQUENCY_40 = 9,
	AUR_PERCENT_FREQUENCY_45 = 10,
	AUR_PERCENT_FREQUENCY_50 = 11,
	AUR_PERCENT_FREQUENCY_55 = 12,
	AUR_PERCENT_FREQUENCY_60 = 13,
	AUR_PERCENT_FREQUENCY_65 = 14,
	AUR_PERCENT_FREQUENCY_70 = 15,
	AUR_PERCENT_FREQUENCY_75 = 16,
	AUR_PERCENT_FREQUENCY_80 = 17,
	AUR_PERCENT_FREQUENCY_85 = 18,
	AUR_PERCENT_FREQUENCY_90 = 19,
	AUR_PERCENT_FREQUENCY_95 = 20,
	AUR_MAX_FREQUENCY = 21,
	AUR_OVERDRIVE = 22,
	AUR_UUD = AUR_PERCENT_FREQUENCY_5,
	AUR_UUD_PLUS = AUR_PERCENT_FREQUENCY_35,
	AUR_SUD = AUR_PERCENT_FREQUENCY_50,
	AUR_SUD_PLUS = AUR_PERCENT_FREQUENCY_65,
	AUR_UD = AUR_PERCENT_FREQUENCY_75,
	AUR_UD_PLUS = AUR_PERCENT_FREQUENCY_85,
	AUR_NOM = AUR_MAX_FREQUENCY,
};

extern const uint aur_power_state2rate[];

enum aur_memory_power_state {
	AUR_MEM_UNDEFINED = 0,
	AUR_MEM_MIN = 1,
	AUR_MEM_VERY_LOW = 2,
	AUR_MEM_LOW = 3,
	AUR_MEM_HIGH = 4,
	AUR_MEM_VERY_HIGH = 5,
	AUR_MEM_MAX = 6,
};

#define AUR_NUM_POWER_STATE (AUR_OVERDRIVE + 1)
#define AUR_NUM_MEMORY_POWER_STATE (AUR_MAX_ALLOW_MEMORY_STATE + 1)

#define AUR_INIT_DVFS_STATE AUR_UUD

/*
 * These macros mean the maximum valid enum value of aur_power_state and
 * aur_memory_power_state, not necessarily the state with the maximum power
 * level.
 */
#define AUR_MAX_ALLOW_STATE AUR_OVERDRIVE
#define AUR_MAX_ALLOW_MEMORY_STATE AUR_MEM_MAX

#define AUR_NUM_POWER_STATE_WORKER 4

struct gxp_pm_ops {
	/*
	 * This callback is called after pm_runtime_get*().
	 * A non-zero return value could fail the block power up process.
	 *
	 * This callback is optional.
	 */
	int (*after_blk_power_up)(struct gxp_dev *gxp);
	/*
	 * This callback is called before pm_runtime_put*().
	 * A non-zero return value could fail the block power down process.
	 *
	 * This callback is optional.
	 */
	int (*before_blk_power_down)(struct gxp_dev *gxp);
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
	u64 pm_value;
	bool using;
};

struct gxp_power_states {
	enum aur_power_state power;
	enum aur_memory_power_state memory;
	bool low_clkmux;
};

extern const struct gxp_power_states off_states;
extern const struct gxp_power_states uud_states;

struct gxp_power_manager {
	struct gxp_dev *gxp;
	struct gcip_pm *pm;
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
	/*
	 * Min/Max frequency limits, in kHz, requested via devfreq.
	 * Protected by `freq_limits_lock`.
	 */
	struct mutex freq_limits_lock;
	u32 min_freq_limit;
	u32 max_freq_limit;
	int curr_state;
	int curr_memory_state; /* Note: this state will not be maintained in the MCU mode. */
	const struct gxp_pm_ops *ops;
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
	/* BCL device handler. */
	struct bcl_device *bcl_dev;
	int force_mux_normal_count;
	/* Max frequency that the thermal driver/ACPM will allow in Hz */
	unsigned long thermal_limit;
	u64 blk_switch_count;
	/* PMU AUR_STATUS base address for block status, maybe NULL */
	void __iomem *aur_status;
	/* Protects @busy_count. */
	spinlock_t busy_lock;
	/* The number of ongoing requests to the firmware. */
	u64 busy_count;
};

/**
 * gxp_pm_blk_on() - Turn on the power for BLK_AUR
 * @gxp: The GXP device to turn on
 *
 * Note: For most cases you should use gxp_acquire_wakelock() to ensure the
 * device is ready to use, unless you really want to power on the block without
 * setting up the device state.
 *
 * Return:
 * * 0       - BLK ON successfully
 */
int gxp_pm_blk_on(struct gxp_dev *gxp);

/**
 * gxp_pm_blk_off() - Turn off the power for BLK_AUR
 * @gxp: The GXP device to turn off
 *
 * Return:
 * * 0       - BLK OFF successfully
 */
int gxp_pm_blk_off(struct gxp_dev *gxp);

/**
 * gxp_pm_blk_reboot() - Reboot the blk.
 * @gxp: The GXP device to reboot
 * @timeout_ms: Wait for the block to be turned off for this duration.
 *
 * Return:
 * * 0       - BLK rebooted successfully
 */
int gxp_pm_blk_reboot(struct gxp_dev *gxp, uint timeout_ms);

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
 */
void gxp_pm_core_off(struct gxp_dev *gxp, uint core);

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
 * gxp_pm_blk_set_rate() - API for setting the block-level DVFS rate.
 * This function can be called at any point after block power on.
 * @gxp: The GXP device to operate
 * @rate: Rate number in khz that need to be set.
 *         Supported rate is in aur_power_state2rate,
 *         if experiment is needed for unsupported rate
 *         please refer to Lassen's ECT table.
 *
 * Return:
 * * 0       - Set finished successfully.
 * * Other   - Set rate encounter issue in gxp_pm_blk_set_rate.
 */
int gxp_pm_blk_set_rate(struct gxp_dev *gxp, unsigned long rate);

/**
 * gxp_pm_blk_get_rate() - API for getting the block-level DVFS rate.
 * This function can be called at any point after block power on.
 * @gxp: The GXP device to operate
 *
 * Device should be powered on while using this function.
 *
 * Return:
 * * rate   - block-level DVFS rate in kHz.
 */
int gxp_pm_blk_get_rate(struct gxp_dev *gxp);

/**
 * gxp_pm_update_requested_power_states() - API for a GXP client to vote for a
 * requested power state and a requested memory power state.
 * @gxp: The GXP device to operate.
 * @origin_states: An existing old requested states, will be cleared. If this is
 *                the first vote, pass AUR_OFF and AUR_MEM_UNDEFINED for field
 *                power_state and memory_state. The low_clkmux field will take no
 *                effect if requested state is AUR_OFF.
 * @requested_states: The new requested states.
 *
 * Return:
 * * 0       - Voting registered
 * * -EINVAL - Invalid original state or requested state
 */

int gxp_pm_update_requested_power_states(struct gxp_dev *gxp,
					 struct gxp_power_states origin_states,
					 struct gxp_power_states requested_states);

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

/**
 * gxp_pm_busy() - Claim there is a request to the firmware.
 * @gxp: The GXP device
 *
 * This function is used in pair with gxp_pm_idle().
 * When there is no ongoing requests, we can put the device in a lower frequency to save power.
 */
void gxp_pm_busy(struct gxp_dev *gxp);
/**
 * gxp_pm_idle() - Reverts gxp_pm_busy().
 * @gxp: The GXP device
 */
void gxp_pm_idle(struct gxp_dev *gxp);

/**
 * gxp_pm_chip_set_ops() - Set the operations to the power manager, i.e.
 * @mgr->ops.
 * @mgr: The power manager to be set operations to
 *
 * This function is expected to be implemented by chip-dependent power
 * management files but not by gxp-pm.c.
 */
void gxp_pm_chip_set_ops(struct gxp_power_manager *mgr);

/**
 * gxp_pm_chip_init() - Do chip-dependent power management initialization.
 * @gxp: The GXP device
 *
 * This function is called as the last step of gxp_pm_init().
 * This function is expected to be implemented by chip-dependent power
 * management files but not by gxp-pm.c.
 */
void gxp_pm_chip_init(struct gxp_dev *gxp);

/**
 * gxp_pm_chip_exit() - Do chip-dependent power management cleanup.
 * @gxp: The GXP device
 *
 * This function is called as the first step of gxp_pm_destroy().
 * This function is expected to be implemented by chip-dependent power
 * management files but not by gxp-pm.c.
 */
void gxp_pm_chip_exit(struct gxp_dev *gxp);

/**
 * gxp_pm_is_blk_down() - Check weather the blk is turned off or not via @gxp->aur_status.
 * @gxp: The GXP device to check
 *
 * Note: This function might be called in in_interrupt() context.
 * Return:
 * * true       - blk is turned off.
 */
static inline bool gxp_pm_is_blk_down(struct gxp_dev *gxp)
{
	return gxp->power_mgr->aur_status ? !readl(gxp->power_mgr->aur_status) :
					    gxp->power_mgr->curr_state == AUR_OFF;
}

/**
 * gxp_pm_set_min_max_freq_limit() - Sets the [min,max] frequency range for the device.
 * @gxp: The GXP device to operate
 * @min_freq_khz: Minimum frequency limit.
 * @max_freq_khz: Maximum frequency limit.
 *
 * Note: This function will be no op in direct mode.
 * Return:
 * * 0           - [Min,Max] frequency limit successfully updated.
 * * -EOPNOTSUPP - Not supported for direct mode.
 * * -EINVAL     - Invalid value sent to firmware via KCI in case of MCU mode.
 * * -EIO        - Unable to communicate to the firmware via KCI.
 */
int gxp_pm_set_min_max_freq_limit(struct gxp_dev *gxp, uint min_freq_khz, uint max_freq_khz);

#endif /* __GXP_PM_H__ */
