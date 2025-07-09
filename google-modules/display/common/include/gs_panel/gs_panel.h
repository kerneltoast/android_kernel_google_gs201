/* SPDX-License-Identifier: MIT */
/*
 * Copyright 2023 Google LLC
 *
 * Use of this source code is governed by an MIT-style
 * license that can be found in the LICENSE file or at
 * https://opensource.org/licenses/MIT.
 */

#ifndef DISPLAY_COMMON_GS_PANEL_H_
#define DISPLAY_COMMON_GS_PANEL_H_

#include <linux/printk.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio/consumer.h>
#include <linux/backlight.h>
#include <linux/thermal.h>
#include <linux/version.h>
#include <drm/drm_bridge.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <drm/drm_property.h>
#include <drm/drm_mipi_dsi.h>

#include "gs_drm/gs_drm_connector.h"
#include "gs_panel/dcs_helper.h"

#define MAX_BL_RANGES 10
#define COLOR_OPTION_DEPTH 5

struct attribute_range {
	__u32 min;
	__u32 max;
};

/**
 * struct brightness_attribute - brightness attribute data
 *
 * @nits: value represents brightness nits range
 * @level: value represents panel brightness level range
 * @percentage: value must be between 0 and 100 and be non-decreasing.
 *              This percentage must comply with display configuration
 *              file.
 *
 * A brightness_attribute represents brightness attribute data.
 */
struct brightness_attribute {
	struct attribute_range nits;
	struct attribute_range level;
	struct attribute_range percentage;
};

/**
 * struct brightness_capability - brightness capability query by user-space
 *
 * @normal: normal rerepresents the normal brightness attribute.
 * @hbm: hbm represents the hbm brightness attribute
 *
 * A brightness_capability represents normal/hbm brightness attribute. It is
 * used to query connector property.
 */
struct brightness_capability {
	struct brightness_attribute normal;
	struct brightness_attribute hbm;
};

#define GS_PANEL_REFRESH_CTRL_FI_FRAME_COUNT_OFFSET 0
#define GS_PANEL_REFRESH_CTRL_FI_FRAME_COUNT_BITS 7
#define GS_PANEL_REFRESH_CTRL_FI_FRAME_COUNT_MAX \
	(BIT(GS_PANEL_REFRESH_CTRL_FI_FRAME_COUNT_BITS) - 1)
#define GS_PANEL_REFRESH_CTRL_FI_FRAME_COUNT_MASK \
	(GS_PANEL_REFRESH_CTRL_FI_FRAME_COUNT_MAX << GS_PANEL_REFRESH_CTRL_FI_FRAME_COUNT_OFFSET)

#define GS_PANEL_REFRESH_CTRL_MIN_REFRESH_RATE_OFFSET GS_PANEL_REFRESH_CTRL_FI_FRAME_COUNT_BITS
#define GS_PANEL_REFRESH_CTRL_MIN_REFRESH_RATE_BITS 8
#define GS_PANEL_REFRESH_CTRL_MIN_REFRESH_RATE_MAX \
	(BIT(GS_PANEL_REFRESH_CTRL_MIN_REFRESH_RATE_BITS) - 1)
#define GS_PANEL_REFRESH_CTRL_MIN_REFRESH_RATE_MASK \
	(GS_PANEL_REFRESH_CTRL_MIN_REFRESH_RATE_MAX << \
	 GS_PANEL_REFRESH_CTRL_MIN_REFRESH_RATE_OFFSET)

#define GS_PANEL_REFRESH_CTRL_FI_AUTO BIT(31)
#define GS_PANEL_REFRESH_CTRL_MRR_V1_OVER_V2 BIT(30)
#define GS_PANEL_REFRESH_CTRL_FEATURE_MASK (GS_PANEL_REFRESH_CTRL_FI_AUTO |\
					    GS_PANEL_REFRESH_CTRL_MRR_V1_OVER_V2)

/**
 * enum gs_panel_feature - features supported by this panel
 * @FEAT_HBM: high brightness mode
 * @FEAT_EARLY_EXIT: early exit from a long frame
 * @FEAT_OP_NS: normal speed (not high speed)
 * @FEAT_FRAME_AUTO: automatic (not manual) frame control, should be set only
 * 		     when FEAT_FRAME_MANUAL_FI = 0
 * @FEAT_FRAME_MANUAL_FI: use DDIC frame insertion for manual mode, should be
 * 			  set only when FEAT_FRAME_AUTO = 0
 * @FEAT_ZA: zonal attenuation
 * @FEAT_MAX: placeholder, counter for number of features
 *
 * The following features are correlated, if one or more of them change, the others need
 * to be updated unconditionally.
 */
enum gs_panel_feature {
	FEAT_HBM = 0,
	FEAT_EARLY_EXIT,
	FEAT_OP_NS,
	FEAT_FRAME_AUTO,
	FEAT_FRAME_MANUAL_FI,
	FEAT_ZA,
	FEAT_MAX,
};

/**
 * enum irc_mode - possible IRC states
 * @IRC_FLAT_DEFAULT: IR compensation on (default configuration)
 * @IRC_FLAT_Z: IR compensation on, in Z mode
 * @IRC_OFF: IR compensation off, to allow for maximum brightness in outdoor sun
 */
enum irc_mode {
	IRC_FLAT_DEFAULT = 0,
	IRC_FLAT_Z,
	IRC_OFF,
};

/**
 * enum gs_panel_state - panel operating state
 * TODO: reword, rethink, refactor (code style for enums relevant)
 * @GPANEL_STATE_UNINITIALIZED: Panel has never been initialized, and panel OTP info such as
 *                             panel serial and revision has not been read yet
 * @GPANEL_STATE_HANDOFF: Panel looked active when driver was loaded. The panel is uninitialized
 *                       in this state and will switch to PANEL_STATE_ON once it gets initialized
 * @GPANEL_STATE_HANDOFF_MODESET: Similar to HANDOFF state, in this case a modeset was called with
                                 unpreferred mode, so display must be blanked before enabling.
 * @GPANEL_STATE_OFF: Panel is fully disabled and powered off
 * @GPANEL_STATE_NORMAL: Panel is ON in Normal operating mode
 * @GPANEL_STATE_LP: Panel is ON in Low Power mode
 * @GPANEL_STATE_MODESET: Going through modeset, where panel gets disable/enable calls with new mode
 * @GPANEL_STATE_BLANK: Panel is ON but no contents are shown on display
 */
enum gs_panel_state {
	GPANEL_STATE_UNINITIALIZED = 0,
	GPANEL_STATE_HANDOFF,
	GPANEL_STATE_HANDOFF_MODESET,
	GPANEL_STATE_OFF,
	GPANEL_STATE_NORMAL,
	GPANEL_STATE_LP,
	GPANEL_STATE_MODESET,
	GPANEL_STATE_BLANK,
};

/**
 * enum gs_panel_idle_mode - type of idle mode supported per mode
 * TODO: reword, rethink, refactor (code style for enums relevant)
 * @GIDLE_MODE_UNSUPPORTED: No idle mode is supported in this mode
 * @GIDLE_MODE_ON_INACTIVITY: In this mode the panel can go into idle automatically
 *                           after last frame update
 * @GIDLE_MODE_ON_SELF_REFRESH: Manually go into lower idle mode when display enters
 *                             self refresh state
 */
enum gs_panel_idle_mode {
	GIDLE_MODE_UNSUPPORTED,
	GIDLE_MODE_ON_INACTIVITY,
	GIDLE_MODE_ON_SELF_REFRESH,
};

enum gs_acl_mode {
	ACL_OFF = 0,
	ACL_NORMAL,
	ACL_ENHANCED,
};

/**
 * enum gs_panel_tex_opt - option of TE/TE2 frequency
 * @TEX_OPT_CHANGEABLE: TE/TE2 frequency follows display refresh rate
 * @TEX_OPT_FIXED: TE/TE2 frequency is fixed at a specific value. Only supported on specific panels
 */
enum gs_panel_tex_opt {
	TEX_OPT_CHANGEABLE,
	TEX_OPT_FIXED,
};

enum gs_cabc_mode {
	GCABC_OFF = 0,
	GCABC_UI_MODE,
	GCABC_STILL_MODE,
	GCABC_MOVIE_MODE,
};

enum gs_local_hbm_enable_state {
	GLOCAL_HBM_DISABLED = 0,
	GLOCAL_HBM_ENABLED,
	GLOCAL_HBM_ENABLING,
};

/**
 * enum mode_progress_type - the type while mode switch is in progress
 * @MODE_DONE: mode switch is done
 * @MODE_RES_IN_PROGRESS: mode switch is in progress, only resolution is changed
 * @MODE_RR_IN_PROGRESS: mode switch is in progress, only refresh rate is changed
 * @MODE_RES_AND_RR_IN_PROGRESS: mode switch is in progress, both resolution and
 *                               refresh rate are changed
 */
enum mode_progress_type {
	MODE_DONE = 0,
	MODE_RES_IN_PROGRESS,
	MODE_RR_IN_PROGRESS,
	MODE_RES_AND_RR_IN_PROGRESS,
};

enum gpio_level {
	GPIO_LEVEL_LOW = 0,
	GPIO_LEVEL_HIGH,
	GPIO_LEVEL_UNSPECIFIED,
};

enum color_data_type {
	COLOR_DATA_TYPE_CIE = 0,
	COLOR_DATA_TYPE_LUMINANCE,
	COLOR_DATA_TYPE_MAX,
	COLOR_DATA_TYPE_FAKE_CIE = COLOR_DATA_TYPE_MAX,
};


struct gs_panel;

/**
 * struct gs_panel_mode - panel mode info
 */
struct gs_panel_mode {
	/** @mode: drm display mode info */
	struct drm_display_mode mode;
	/** @gs_mode: gs driver specific mode info */
	struct gs_display_mode gs_mode;
	/** @priv_data: per mode panel driver private data TODO: eliminate */
	const void *priv_data;
	/** @te2_timing: TE2 signal timing */
	struct gs_panel_te2_timing te2_timing;
	/**
	 * @idle_mode:
	 *
	 * Indicates whether going into lower refresh rate is allowed while in this mode, and what
	 * type of idle mode is supported, for more info refer to enum gs_panel_idle_mode.
	 */
	enum gs_panel_idle_mode idle_mode;
};

/* PANEL FUNCS */

struct gs_panel_funcs {
	/**
	 * @set_brightness:
	 *
	 * This callback is used to implement driver specific logic for brightness
	 * configuration. Otherwise defaults to sending brightness commands through
	 * dcs command update.
	 * The `br` parameter is the target brightness level, as opposed to
	 * percentage or nits.
	 */
	int (*set_brightness)(struct gs_panel *gs_panel, u16 br);

	/**
	 * @set_lp_mode:
	 *
	 * This callback is used to handle command sequences to enter low power modes.
	 *
	 * mode: LP mode to which to switch
	 *
	 * TODO(b/279521692): implementation
	 */
	void (*set_lp_mode)(struct gs_panel *gs_panel, const struct gs_panel_mode *mode);

	/**
	 * @set_nolp_mode:
	 *
	 * This callback is used to handle command sequences to exit from low power
	 * modes.
	 *
	 * mode: mode to which to switch
	 *
	 * TODO(b/279521692): implementation
	 */
	void (*set_nolp_mode)(struct gs_panel *gs_panel, const struct gs_panel_mode *mode);

	/**
	 * @set_binned_lp:
	 *
	 * This callback is used to handle additional command sequences for low
	 * power modes based on different brightness thresholds.
	 */
	void (*set_binned_lp)(struct gs_panel *gs_panel, u16 br);

	/**
	 * @set_post_lp_mode:
	 *
	 * This callback is used to handle additional operations after set_lp_mode and
	 * first set_binned_lp are called.
	 *
	 * TODO(b/279521692): implementation
	 */
	void (*set_post_lp_mode)(struct gs_panel *gs_panel);

	/**
	 * @set_hbm_mode:
	 *
	 * This callback is used to implement panel specific logic for high brightness
	 * mode enablement. If this is not defined, it means that panel does not
	 * support HBM
	 *
	 * TODO(b/279521612): implementation
	 */
	void (*set_hbm_mode)(struct gs_panel *gs_panel, enum gs_hbm_mode mode);

	/**
	 * @set_dimming:
	 *
	 * This callback is used to implement panel specific logic for dimming mode
	 * enablement. If this is not defined, it means that panel does not support
	 * dimming.
	 *
	 * dimming_on: true for dimming enabled, false for dimming disabled
	 *
	 * TODO(b/279520614): implementation
	 */
	void (*set_dimming)(struct gs_panel *gs_panel, bool dimming_on);

	/**
	 * @get_local_hbm_mode_effective_delay_frames:
	 *
	 * This callback is used to implement panel specific logic for gs_panel to
	 * get local hbm mode effective delay frames. If this is not defined, it uses
	 * gs_panel_lhbm_desc.effective_delay_frames directly.
	 *
	 */
	u32 (*get_local_hbm_mode_effective_delay_frames)(struct gs_panel *gs_panel);

	/**
	 * @set_local_hbm_mode:
	 *
	 * This callback is used to implement panel specific logic for local high
	 * brightness mode enablement. If this is not defined, it means that panel
	 * does not support local HBM
	 *
	 * TODO(b/279521693): implementation
	 */
	void (*set_local_hbm_mode)(struct gs_panel *gs_panel,
				   bool local_hbm_en);

	/**
	 * @set_local_hbm_mode_post:
	 *
	 * This callback is used to implement panel specific logic at some time after enabling
	 * local high brightness mode.
	 *
	 * TODO(b/279521693): implementation
	 */
	void (*set_local_hbm_mode_post)(struct gs_panel *gs_panel);

	/**
	 * @mode_set:
	 *
	 * This callback is used to perform driver specific logic for mode_set.
	 * This could be called while display is on or off, should check internal
	 * state to perform appropriate mode set configuration depending on this state.
	 *
	 * TODO(b/279520499): implementation
	 */
	void (*mode_set)(struct gs_panel *gs_panel, const struct gs_panel_mode *mode);

	/**
	 * @get_te2_edges:
	 *
	 * This callback is used to get the rising and falling edges of TE2 signal.
	 * The input buf is used to store the results in string.
	 */
	ssize_t (*get_te2_edges)(struct gs_panel *gs_panel, char *buf, bool lp_mode);

	/**
	 * @set_te2_edges:
	 *
	 * This callback is used to configure the rising and falling edges of TE2
	 * signal. The input timings include the values we need to configure.
	 */
	int (*set_te2_edges)(struct gs_panel *gs_panel, u32 *timings, bool lp_mode);

	/**
	 * @update_te2:
	 *
	 * This callback is used to update the TE2 signal via DCS commands.
	 * This should be called when the display state is changed between
	 * normal and LP modes, or the refresh rate and LP brightness are
	 * changed.
	 */
	void (*update_te2)(struct gs_panel *gs_panel);

	/**
	 * @atomic_check
	 *
	 * This optional callback happens in atomic check phase, it gives a chance to panel driver
	 * to check and/or adjust atomic state ahead of atomic commit.
	 *
	 * Should return 0 on success (no problems with atomic commit) otherwise negative errno
	 *
	 * TODO(b/279520499): implementation
	 */
	int (*atomic_check)(struct gs_panel *gs_panel, struct drm_atomic_state *state);

	/**
	 * @commit_done
	 *
	 * Called after atomic commit flush has completed but transfer may not have started yet
	 *
	 * TODO(b/279520499): implementation
	 */
	void (*commit_done)(struct gs_panel *gs_panel);

	/**
	 * @is_mode_seamless:
	 *
	 * This callback is used to check if a switch to a particular mode can be done
	 * seamlessly without full mode set given the current hardware configuration
	 *
	 * TODO(b/279520499): implementation
	 */
	bool (*is_mode_seamless)(const struct gs_panel *gs_panel,
				 const struct gs_panel_mode *pmode);

	/**
	 * @set_self_refresh
	 *
	 * Called when display self refresh state has changed. While in self refresh state, the
	 * panel can optimize for power assuming that there are no pending updates.
	 *
	 * Returns true if underlying mode was updated to reflect new self refresh state,
	 * otherwise returns false if no action was taken.
	 */
	bool (*set_self_refresh)(struct gs_panel *gs_panel, bool enable);

	/**
	 * @refresh_ctrl
	 *
	 * Apply the panel refresh behavior. It is expected to use the
	 * `refresh_ctrl` member of the `gs_panel` when applying new behavior.
	 */
	void (*refresh_ctrl)(struct gs_panel *gs_panel);

	/**
	 * @set_cabc_mode
	 *
	 * This callback is used to implement panel specific logic for cabc mode
	 * enablement. If this is not defined, it means that panel does not
	 * support cabc.
	 */
	void (*set_cabc_mode)(struct gs_panel *gs_panel, enum gs_cabc_mode mode);

	/**
	 * @set_frame_rate
	 *
	 * Set the current frame rate. This is called by a userspace client to
	 * mark when the display software is changing the frame update rate,
	 * for panels where that may require additional updates from the driver.
	 * This is an optional function, and unrelated to control of the panel
	 * refresh rate.
	 */
	void (*set_frame_rate)(struct gs_panel *gs_panel, u16 frame_rate);

	/**
	 * @set_op_hz
	 *
	 * set display panel working on specified operation rate.
	 *
	 * Returns 0 if successfully setting operation rate.
	 */
	int (*set_op_hz)(struct gs_panel *gs_panel, unsigned int hz);

	/**
	 * @read_extinfo
	 *
	 * This callback is used to override the default behavior for reading
	 * the extinfo registers of the panel, which contain manufacturer
	 * information about the exact type of hardware.
	 *
	 * While most cases can use the default behavior, certain outliers (such
	 * as emulated panels) may wish to implement their own versions.
	 *
	 * Return: 0 on success, negative value on error
	 */
	int (*read_extinfo)(struct gs_panel *gs_panel);

	/**
	 * @get_panel_rev
	 *
	 * This callback is used to get panel HW revision from panel_extinfo.
	 * It is expected to fill in the `panel_rev` member of the `gs_panel`
	 *
	 * @id: contents of `extinfo`, read as a binary value
	 */
	void (*get_panel_rev)(struct gs_panel *gs_panel, u32 id);

	/**
	 * @read_id:
	 *
	 * This callback is used to read the panel's id. The id is unique for
	 * each panel.
	 */
	int (*read_id)(struct gs_panel *gs_panel);

	/**
	 * @set_acl_mode:
	 *
	 * This callback is used to implement panel specific logic for acl mode
	 * enablement. If this is not defined, it means that panel does not
	 * support acl.
	 *
	 * TODO(tknelms): implement default version
	 */
	void (*set_acl_mode)(struct gs_panel *gs_panel, enum gs_acl_mode mode);

	/**
	 * @set_ssc_en:
	 *
	 * This callback is used to implement panel specific logic for ssc mode
	 * enablement. If this is not defined, it means that panel does not
	 * support ssc.
	 *
	 */
	void (*set_ssc_en)(struct gs_panel *gs_panel, bool enabled);

	/**
	 * @panel_config:
	 *
	 * This callback is used to do one time panel configuration before the
	 * common driver initialization. It may be used for driver or
	 * code-related initialization that may be dependent on information like
	 * panel rev, but is otherwise invariant across the life of the driver.
	 *
	 * Notably, panel hardware state at this point is unknown, so avoid
	 * attempting to communicate directly with the panel.
	 */
	int (*panel_config)(struct gs_panel *gs_panel);

	/**
	 * @panel_init:
	 *
	 * This callback is used to do initialization for any panel-specific
	 * functions. It is called on first initialization as a one-time
	 * configuration.
	 *
	 * Panel hardware should be available for communication at this point,
	 * for example, to read OTP values from DDIC.
	 */
	void (*panel_init)(struct gs_panel *gs_panel);

	/**
	 * @panel_reset:
	 *
	 * This callback is used to allow panel to toggle only reset pin instead of full
	 * prepare sequence (including power rails) while the device is in BLANK state.
	 * This is not called in any other state.
	 */
	void (*panel_reset)(struct gs_panel *gs_panel);

	/**
	 * @get_te_usec
	 *
	 * This callback is used to get current TE pulse time.
	 *
	 * TODO(b/279521893): implementation
	 */
	unsigned int (*get_te_usec)(struct gs_panel *gs_panel, const struct gs_panel_mode *pmode);

	/**
	 * @run_normal_mode_work
	 *
	 * This callback is used to run the periodic work for each panel in
	 * normal mode.
	 */
	void (*run_normal_mode_work)(struct gs_panel *gs_panel);

	/**
	 * @update_ffc
	 *
	 * This callback is used to update FFC (Frame Frequency Control) for panel.
	 * The unit of DSI HS clock is megabits per second.
	 */
	void (*update_ffc)(struct gs_panel *gs_panel, unsigned int hs_clk_mbps);

	/**
	 * @pre_update_ffc
	 *
	 * This callback is used to do something before updating FFC for panel.
	 */
	void (*pre_update_ffc)(struct gs_panel *gs_panel);

	/**
	 * @rr_need_te_high
	 *
	 * check if a panel needs for rr commands to be sent
	 * during a TE high window
	 *
	 * @pmode: mode that panel is transitioning to
	 */
	bool (*rr_need_te_high)(struct gs_panel *gs_panel, const struct gs_panel_mode *pmode);

	/**
	 * @set_te2_rate
	 *
	 * This callback is used to set TE2 rate.
	 *
	 * Returns true if the rate is applied successfully.
	 */
	bool (*set_te2_rate)(struct gs_panel *gs_panel, u32 rate_hz);

	/**
	 * @get_te2_rate
	 *
	 * This callback is used to get TE2 rate.
	 */
	u32 (*get_te2_rate)(struct gs_panel *gs_panel);

	/**
	 * @set_te2_option
	 *
	 * This callback is used to set TE2 option.
	 *
	 * Returns true if the option is applied successfully.
	 */
	bool (*set_te2_option)(struct gs_panel *gs_panel, u32 option);

	/**
	 * @get_te2_option
	 *
	 * This callback is used to get TE2 option.
	 */
	enum gs_panel_tex_opt (*get_te2_option)(struct gs_panel *gs_panel);

	/**
	 * @get_color_data
	 *
	 * This callback is used to read vendor-provided calibration data from the DDIC.
	 */
	ssize_t (*get_color_data)(struct gs_panel *gs_panel, char *buf, size_t buf_len);

	/**
	 * @set_color_data_config
	 *
	 * This callback is used to set the data to be read from the panel's vendor-provided
	 * calibration data in the next read.
	 *
	 * Returns 0 if applied successfully.
	 */
	int (*set_color_data_config)(struct gs_panel *gs_panel, enum color_data_type read_type,
				     int option);
};

/* PANEL DESC */

/**
 * struct gs_panel_brightness_desc
 * TODO: document
 * @lower_min_brightness: dim brightness for tablets in dock mode
 */
struct gs_panel_brightness_desc {
	u32 max_luminance;
	u32 max_avg_luminance;
	u32 min_luminance;
	u32 max_brightness;
	u32 min_brightness;
	u32 lower_min_brightness;
	u32 default_brightness;
	const struct brightness_capability *brt_capability;
};

struct gs_brightness_configuration {
	const u32 panel_rev;
	const u32 default_brightness;
	const struct brightness_capability brt_capability;
};

/**
 * gs_panel_update_brightness_desc - Update brightness_desc based on panel rev
 * @desc: Desc object to update
 * @configs: Array of possible brightness configurations
 * @num_configs: How many configs are in the array
 * @panel_rev: This panel's revision
 *
 * Some of our panels have different target brightness configuration based on
 * their panel revision. This ends up stored in a
 * `struct gs_brightness_configuration` array. This function finds the matching
 * configuration based on the given panel revision and updates the
 * `struct gs_panel_brightness_desc` to reflect the correct brightness settings.
 *
 * Returns: 0 on success, negative value on error
 */
int gs_panel_update_brightness_desc(struct gs_panel_brightness_desc *desc,
				    const struct gs_brightness_configuration *configs,
				    u32 num_configs, u32 panel_rev);

/**
 * struct gs_panel_lhbm_desc - Descriptor of lhbm behaviors
 */
struct gs_panel_lhbm_desc {
	/**
	 * @no_lhbm_rr_constraints: whether lhbm has rr constraints
	 *
	 * set true if the panel doesn't have lhbm common hw constraints, include
	 * 1. only allow turn on lhbm at peak refresh rate
	 *    - `freq set` may set to peak when enabling lhbm cause underrun at
	 *      non-peak refresh rate.
	 *    - abnormal display (like green tint) when enabling lhbm at non-peak
	 *      refresh rate.
	 * 2. not allow switch refresh rate when lhbm is on
	 *    - if `freq set` is changed when lhbm is on, lhbm may not work normally.
	 */
	bool no_lhbm_rr_constraints;
	/**
	 * @post_cmd_delay_frames: Frames to delay before sending post_lhbm
	 */
	const u32 post_cmd_delay_frames;
	/**
	 * @effective_delay_frames: Frames to delay before updating effective state
	 */
	const u32 effective_delay_frames;
	/**
	 * @lhbm_on_delay_frames: Frames needed before sending lhbm on
	 *
	 * Indicate how many frames are needed before sending lhbm on commands
	 * while exiting from AoD mode. Default 0 means no such constraint.
	 */
	const u32 lhbm_on_delay_frames;
};

struct gs_calibration_capability {
	bool en;
	size_t data_size;
	int min_option;
	int max_option;
};

/**
 * struct gs_panel_calibration_desc - Descriptor of calibration data read behaviors.
 * Some panels allow for reading of vendor-provided DDIC data to calibrate CIQ on-device.
 */
struct gs_panel_calibration_desc {
	struct gs_calibration_capability color_cal[COLOR_DATA_TYPE_MAX];
};

/**
 * gs_panel_mode_array - container for display modes
 * @num_modes: number of modes in array
 * @modes: display modes
 */
struct gs_panel_mode_array {
	size_t num_modes;
	const struct gs_panel_mode modes[];
};

#define BL_STATE_STANDBY BL_CORE_FBBLANK
#define BL_STATE_LP BIT(30) /* backlight is in LP mode */

#define MAX_TE2_TYPE 20
#define PANEL_ID_MAX 40
#define PANEL_EXTINFO_MAX 16
#define PANEL_MODEL_MAX 14
#define LOCAL_HBM_MAX_TIMEOUT_MS 3000 /* 3000 ms */
#define LOCAL_HBM_GAMMA_CMD_SIZE_MAX 16

enum panel_reset_timing {
	PANEL_RESET_TIMING_HIGH = 0,
	PANEL_RESET_TIMING_LOW,
	PANEL_RESET_TIMING_INIT,
	PANEL_RESET_TIMING_COUNT
};

enum panel_reg_id {
	PANEL_REG_ID_INVALID = 0,
	PANEL_REG_ID_VCI,
	PANEL_REG_ID_VDDI,
	PANEL_REG_ID_VDDD,
	PANEL_REG_ID_VDDR_EN,
	PANEL_REG_ID_VDDR,
	PANEL_REG_ID_AVDD,
	PANEL_REG_ID_AVEE,
	PANEL_REG_ID_MAX,
};

struct panel_reg_ctrl {
	enum panel_reg_id id;
	u32 post_delay_ms;
};
#define IS_VALID_PANEL_REG_ID(id) (((id) > PANEL_REG_ID_INVALID) && ((id) < PANEL_REG_ID_MAX))
#define PANEL_REG_COUNT (PANEL_REG_ID_MAX - 1)

/**
 * struct gs_panel_reg_ctrl_desc - An ordered set of regulators per purpose
 *
 * Each array of struct panel_reg_ctrl is a description of which regulators, in
 * order, are activated/deactivated for the relevant power operation.
 * Each entry in the array is a pair of "which regulator" matched with "how long
 * to delay after enable/disable".
 *
 * The panel driver may then define for each operation (enable, post-enable,
 * pre-disable, and disable) which regulators are activated/deactivated in the
 * given order. As an example, if a struct gs_panel_reg_ctrl_desc is defined
 * with these members:
 * .reg_ctrl_enable = { {PANEL_REG_ID_VDDI, 1}, {PANEL_REG_ID_VCI, 10},},
 * .reg_ctrl_post_enable = {{PANEL_REG_ID_VDDD, 1},},
 * then the "enable" process will turn on the VDDI regulator, wait 1ms,
 * then turn on the VCI regulator, then wait 10ms.
 * Later, during the "post_enable" process, it will enable the VDDD regulator,
 * and then wait an additional 1ms.
 */
struct gs_panel_reg_ctrl_desc {
	/** @reg_ctrl_enable: panel enable regulator sequence */
	const struct panel_reg_ctrl reg_ctrl_enable[PANEL_REG_COUNT];
	/** @reg_ctrl_enable: panel post-enable regulator sequence */
	const struct panel_reg_ctrl reg_ctrl_post_enable[PANEL_REG_COUNT];
	/** @reg_ctrl_enable: panel pre-disable regulator sequence */
	const struct panel_reg_ctrl reg_ctrl_pre_disable[PANEL_REG_COUNT];
	/** @reg_ctrl_enable: panel disable regulator sequence */
	const struct panel_reg_ctrl reg_ctrl_disable[PANEL_REG_COUNT];
};

/**
 * struct gs_display_stats_desc - Descriptor of the display stats
 */
struct gs_display_stats_desc {
	const struct display_stats_resolution *resolution_table;
	size_t resolution_table_count;
	const int *vrefresh_range;
	size_t vrefresh_range_count;
	const int *lp_vrefresh_range;
	size_t lp_vrefresh_range_count;
	bool enabled;
};

struct gs_panel_desc {
	u8 panel_id_reg;
	u32 data_lane_cnt;
	u32 hdr_formats;
	const struct gs_panel_brightness_desc *brightness_desc;
	const struct gs_panel_lhbm_desc *lhbm_desc;
	const struct gs_panel_calibration_desc *calibration_desc;
	u32 rr_switch_duration;
	bool dbv_extra_frame;
	bool is_partial;
	bool is_idle_supported;
	const u32 *bl_range;
	u32 bl_num_ranges;
	const struct gs_panel_mode_array *modes;
	const struct gs_panel_mode_array *lp_modes;
	const struct gs_dsi_cmdset *off_cmdset;
	const struct gs_dsi_cmdset *lp_cmdset;
	const struct gs_binned_lp *binned_lp;
	const size_t num_binned_lp;
	bool has_off_binned_lp_entry;
	const struct drm_panel_funcs *panel_func;
	const struct gs_panel_funcs *gs_panel_func;
	const int reset_timing_ms[PANEL_RESET_TIMING_COUNT];
	const struct gs_panel_reg_ctrl_desc *reg_ctrl_desc;
	struct gs_display_stats_desc *stats_desc;
	/** @default_dsi_hs_clk_mbps: default MIPI DSI HS clock (megabits per second) */
	u32 default_dsi_hs_clk_mbps;
	/** @refresh_on_lp: inform composer that we need a frame update while entering AOD or not */
	bool refresh_on_lp;

	/**
	 * @frame_interval_us: store frame interval information, it provides a hint about the
	 * next frame(s) cadence. This information can be utilized by driver to estimate
	 * next frame's present time. The unit is microsecond.
	 */
	u32 frame_interval_us;

	/** @normal_mode_work_delay_ms: period of the periodic work in normal mode */
	const u32 normal_mode_work_delay_ms;
	/**
	 * @notify_te2_rate_changed_work_delay_ms: delay the work to call sysfs_notify
	 *                                         for TE2 rate change
	 */
	const u32 notify_te2_rate_changed_work_delay_ms;
};

/* PRIV DATA */

/**
 * struct gs_panel_debugfs_entries - references to debugfs folder entries
 * @panel: parent folder for panel (ex. "DSI-1/panel")
 * @reg: folder for direct dsi operations (ex. "DSI-1/panel/reg")
 * @cmdset: folder for cmdset entries (ex. "DSI-1/panel/cmdsets")
 *
 * This stores references to the main "folder"-level debugfs entries for the
 * panel. This allows some degree of extension by specific drivers, for example
 * to add an additional cmdset to the "cmdset" debugfs folder.
 */
struct gs_panel_debugfs_entries {
	struct dentry *panel;
	struct dentry *reg;
	struct dentry *cmdset;
};

/**
 * struct gs_panel_gpio - references to gpio descriptors associated with panel
 */
struct gs_panel_gpio {
	struct gpio_desc *reset_gpio;
	struct gpio_desc *enable_gpio;
	struct gpio_desc *vddd_gpio;

	enum gpio_level vddd_gpio_fixed_level;
	bool keep_reset_high;
};

/**
 * struct gs_panel_regulator - state of the power regulator
 * TODO: document
 */
struct gs_panel_regulator {
	struct regulator *vci;
	struct regulator *vddi;
	struct regulator *vddd;
	struct regulator *vddr_en;
	struct regulator *vddr;
	struct regulator *avdd;
	struct regulator *avee;
	u32 vddd_normal_uV;
	u32 vddd_lp_uV;
	u32 avdd_uV;
	u32 avee_uV;
	/** @need_post_vddd_lp: indicates need to adjust vddd lp in self refresh */
	bool need_post_vddd_lp;
	/** @post_vddd_lp_enabled: adjust lp vddd in self refresh instead of mode set */
	bool post_vddd_lp_enabled;
};

/**
 * struct gs_te_info - stores te-related data
 */
struct gs_te_info {
	/** @freq: panel TE frequency, in Hz */
	u32 rate_hz;
	/** @option: panel frequency option */
	enum gs_panel_tex_opt option;
};

/**
 * struct gs_panel_status - hw or sw status of panel
 *
 * For some features, we would like to have a record of both the intended state
 * of the panel in the software and the current state of the panel in the
 * hardware. This struct carries a number of fields that exist both as intended
 * sw state and actual hw state. Not all features will necessarily be supported
 * on all panels.
 */
struct gs_panel_status {
	/**
	 * @feat: software or working correlated features, not guaranteed to be effective in panel
	 * Specifically, this is a bitmap of enum gs_panel_feature features
	 */
	DECLARE_BITMAP(feat, FEAT_MAX);
	/** @vrefresh: vrefresh rate effective in panel, in Hz */
	u32 vrefresh;
	/** @idle_vrefresh: idle vrefresh rate effective in panel, in Hz */
	u32 idle_vrefresh;
	/** @dbv: brightness */
	u16 dbv;
	/** @acl_setting: automatic current limiting setting */
	enum gs_acl_mode acl_mode;
	/** @irc_mode: IR compensation mode */
	enum irc_mode irc_mode;
	/** @te: TE-related status */
	struct gs_te_info te;
};

struct gs_panel_idle_data {
	bool panel_idle_enabled;
	bool panel_need_handle_idle_exit;
	bool panel_update_idle_mode_pending;
	bool self_refresh_active;
	u32 panel_idle_vrefresh;
	u32 idle_delay_ms;
	struct delayed_work idle_work;
};

/**
 * struct gs_te2_mode_data - stores te2-related mode data
 */
struct gs_te2_mode_data {
	/* @mode: normal or LP mode data */
	const struct drm_display_mode *mode;
	/* @binned_lp: LP mode data */
	const struct gs_binned_lp *binned_lp;
	/* @timing: normal or LP mode timing */
	struct gs_panel_te2_timing timing;
};

/**
 * struct gs_te2_data - stores te2-related data
 * TODO: refactor?
 */
struct gs_te2_data {
	struct gs_te2_mode_data mode_data[MAX_TE2_TYPE];
	enum gs_panel_tex_opt option;
	u32 rate_hz;
	/* TODO: below are related to refresh rate instead of TE2 */
	u32 last_rr;
	int last_rr_te_gpio_value;
	u64 last_rr_te_counter;
	u32 last_rr_te_usec;
};

/**
 * struct gs_panel_timestamps - keeps track of timestamps
 * for particular operations the panel has performed
 */
struct gs_panel_timestamps {
	ktime_t last_commit_ts;
	ktime_t last_mode_set_ts;
	ktime_t last_self_refresh_active_ts;
	ktime_t last_panel_idle_set_ts;
	ktime_t last_rr_switch_ts;
	ktime_t last_lp_exit_ts;
	ktime_t idle_exit_dimming_delay_ts;
	ktime_t timeline_expected_present_ts;
	/** @conn_last_present_ts: last expected present timestamp */
	ktime_t conn_last_present_ts;
};

/**
 * struct gs_local_hbm_timestamps - timestamps for lhbm
 *
 * @en_cmd_ts: Timestamp of sending initial lhbm command
 * @next_vblank_ts: Timestamp of the next upcoming vblank
 * @last_vblank_ts: Timestamp of the last vblank
 * @last_lp_vblank_cnt: Absolute vblank number of the final LP vblank
 */
struct gs_local_hbm_timestamps {
	ktime_t en_cmd_ts;
	ktime_t next_vblank_ts;
	ktime_t last_vblank_ts;
	u64 last_lp_vblank_cnt;
};

/**
 * struct gs_local_hbm_work_data - Data required for threading lhbm work queue
 *
 * @wq: work queue to dispatch lhbm timeou worker onto threads
 * @timeout_work: work used to turn off local hbm if reach max_timeout
 * @worker: worker servicing the post_work
 * @thread: thread associated with the post_work worker
 * @post_work: Work to execute the post_lhbm commands
 */
struct gs_local_hbm_work_data {
	/* timeout */
	struct workqueue_struct *wq;
	struct delayed_work timeout_work;
	/* post work */
	struct kthread_worker worker;
	struct task_struct *thread;
	struct kthread_work post_work;
};

/**
 * struct gs_local_hbm - Local state data for lhbm handling
 */
struct gs_local_hbm {
	/**
	 * @requested_state: lhbm state requested to be executed
	 */
	enum gs_local_hbm_enable_state requested_state;
	/**
	 * @effective_state: currently-active lhbm state
	 */
	enum gs_local_hbm_enable_state effective_state;

	/**
	 * @max_timeout_ms: max local hbm on period in ms
	 */
	u32 max_timeout_ms;

	/**
	 * @post_work_disabled: control variable for lhbm_post_work
	 *
	 * Control variable to allow or disallow queueing the lhbm_post_work
	 * method from debugfs
	 */
	bool post_work_disabled;

	/**
	 * @work_data: Data required for threading lhbm work queue
	 */
	struct gs_local_hbm_work_data work_data;

	/**
	 * @timestamps: records of timestamps relating to lhbm sequences
	 */
	struct gs_local_hbm_timestamps timestamps;

	/**
	 * @frame_index: counter to keep track of frames while waiting
	 * Specifically used for the post_work callback in
	 * lhbm_wait_vblank_and_delay function
	 */
	u32 frame_index;

	/**
	 * @gamma_para_ready: Deprecated flag for gamma commands during lhbm
	 */
	bool gamma_para_ready;
	/**
	 * @gamma_cmd: Deprecated data relating to gamma commands during lhbm
	 */
	u8 gamma_cmd[LOCAL_HBM_GAMMA_CMD_SIZE_MAX];
};

/**
 * struct gs_thermal_data - access to thermal data for panels that need it
 */
struct gs_thermal_data {
	/** @tz: thermal zone device for reading temperature */
	struct thermal_zone_device *tz;
	/** @hw_temp: the temperature applied into panel */
	u32 hw_temp;
	/**
	 * @pending_temp_update: whether there is pending temperature update. It will be
	 *                       handled in the commit_done function.
	 */
	bool pending_temp_update;
};


enum display_stats_state {
	DISPLAY_STATE_ON,
	DISPLAY_STATE_HBM,
	DISPLAY_STATE_LP,
	DISPLAY_STATE_OFF,
	DISPLAY_STATE_MAX
};

struct display_stats_resolution {
	u16 hdisplay;
	u16 vdisplay;
};

struct display_stats_time_state {
	size_t available_count;
	u64 *time;
};

struct gs_error_counter {
	u32 te;
	u32 unknown;
};

/**
 * struct gs_panel_color_data - data associated with panel color data
 * @size: size to allocate for color data panel read
 * @data: color data read from panel
 * @ready: whether color data is ready to read
 */
struct gs_panel_color_data {
	size_t size;
	char *data;
	bool ready;
};

#define MAX_VREFRESH_RANGES	10
#define MAX_RESOLUTION_TABLES	2

struct display_stats {
	int vrefresh_range[MAX_VREFRESH_RANGES];
	size_t vrefresh_range_count;
	int lp_vrefresh_range[MAX_VREFRESH_RANGES];
	size_t lp_vrefresh_range_count;
	struct display_stats_resolution res_table[MAX_RESOLUTION_TABLES];
	unsigned int res_table_count;
	struct display_stats_time_state time_in_state[DISPLAY_STATE_MAX];
	enum display_stats_state last_state;
	int last_time_state_idx;
	ktime_t last_update;
	struct mutex lock;
	bool initialized;
};

/**
 * struct gs_bl_notifier - info for notifying brightness changes to ALS
 * @ranges: brightness levels to use as thresholds
 * @num_ranges: how many brightness levels we're using
 * @current_range: which index of brightness threshold is current
 */
struct gs_bl_notifier {
	u32 ranges[MAX_BL_RANGES];
	u32 num_ranges;
	u32 current_range;
};

/**
 * gs_touch_bridge_data - data relating to panel connection to touch bridge
 * @touch_dev: pointer to device_node obtained from device tree
 * @attached: whether we have found and attached a touch device
 * @retry_count: number of attempts to find touch device
 */
struct gs_touch_bridge_data {
	struct device_node *touch_dev;
	bool attached;
	u32 retry_count;
};

/**
 * struct gs_panel - data associated with panel driver operation
 * TODO: better documentation
 */
struct gs_panel {
	struct device *dev;
	struct drm_panel base;
	struct gs_panel_debugfs_entries debugfs_entries;
	struct gs_panel_gpio gpio;
	struct gs_panel_regulator regulator;
	struct gs_drm_connector *gs_connector;
	struct drm_bridge bridge;
	const struct gs_panel_desc *desc;
	const struct gs_panel_mode *current_mode;
	bool initialized;
	/**
	 * @panel_state: High-level state of the panel and driver
	 */
	enum gs_panel_state panel_state;
	/**
	 * @sw_status: intended status of panel hardware
	 */
	struct gs_panel_status sw_status;
	/**
	 * @hw_status: current status of panel hardware
	 */
	struct gs_panel_status hw_status;
	/* If true, panel won't be powered off */
	bool force_power_on;
	struct gs_panel_idle_data idle_data;
	u32 op_hz;
	u32 osc2_clk_khz;
	/**
	 * indicates the lower bound of refresh rate
	 * 0 means there is no lower bound limitation
	 * -1 means display should not switch to lower
	 * refresh rate while idle.
	 */
	int min_vrefresh;
	/**
	 * indicates the supported max refresh rate in the panel.
	 */
	int max_vrefresh;
	/**
	 * indicates the supported max bts fps in the panel.
	 */
	int peak_bts_fps;
	bool dimming_on;
	bool bl_ctrl_dcs;
	enum gs_cabc_mode cabc_mode;
	struct backlight_device *bl;
	struct mutex mode_lock;
	struct mutex bl_state_lock;
	struct mutex lp_state_lock;
	const struct gs_binned_lp *current_binned_lp;
	struct drm_property_blob *lp_mode_blob;
	char panel_id[PANEL_ID_MAX];
	char panel_extinfo[PANEL_EXTINFO_MAX];
	char panel_model[PANEL_MODEL_MAX];
	u32 panel_rev;
	enum drm_panel_orientation orientation;
	struct gs_te2_data te2;
	/** @touch_bridge_data: keeps track of connection to touch bridge */
	struct gs_touch_bridge_data touch_bridge_data;
	struct gs_panel_timestamps timestamps;

	struct gs_thermal_data *thermal;

	/** @bl_notifier: Struct for notifying ALS about brightness changes */
	struct gs_bl_notifier bl_notifier;

	/* use for notify state changed */
	struct work_struct notify_panel_mode_changed_work;
	struct work_struct notify_brightness_changed_work;
	struct delayed_work notify_panel_te2_rate_changed_work;
	struct work_struct notify_panel_te2_option_changed_work;
	enum display_stats_state notified_power_mode;

	/* use for display stats residence */
	struct display_stats disp_stats;

	/* current type of mode switch */
	enum mode_progress_type mode_in_progress;
	/* indicates BTS raise due to op_hz switch */
	bool boosted_for_op_hz;

	/* GHBM */
	enum gs_hbm_mode hbm_mode;
	/* LHBM struct */
	struct gs_local_hbm lhbm;

	/** @dsi_hs_clk_mbps: current MIPI DSI HS clock (megabits per second) */
	u32 dsi_hs_clk_mbps;
	/* ACL mode */
	enum gs_acl_mode acl_mode;
	/* refresh ctrl settings */
	u32 refresh_ctrl;
	/* SSC mode */
	bool ssc_en;

	/** @normal_mode_work_delay_ms: period of the periodic work in normal mode */
	u32 normal_mode_work_delay_ms;
	/** @normal_mode_work: periodic work for each panel in normal mode */
	struct delayed_work normal_mode_work;

	/* use for notify op hz changed */
	struct blocking_notifier_head op_hz_notifier_head;

	/** @color_data: for color data panel read  */
	struct gs_panel_color_data color_data;

	/** @frame_interval_us: frame interval of new timeline in us */
	u32 frame_interval_us;

	/** @skip_align: skip cmd align mechanism while this flag is set */
	bool skip_cmd_align;

	/** @error_counter: use for tracking panel errors */
	struct gs_error_counter error_counter;

	/** @color_data_size: size to allocate for color data panel read  */
	size_t color_data_size;

	/** @trace_pid: pid to use for panel trace functions */
	pid_t trace_pid;
};

/* FUNCTIONS */
static inline const char *
gs_get_panel_state_string(enum gs_panel_state panel_state)
{
	switch (panel_state) {
	case GPANEL_STATE_UNINITIALIZED:
		return "UNINITIALIZED";
	case GPANEL_STATE_HANDOFF:
		return "HANDOFF";
	case GPANEL_STATE_HANDOFF_MODESET:
		return "HANDOFF_MODESET";
	case GPANEL_STATE_OFF:
		return "OFF";
	case GPANEL_STATE_NORMAL:
		return "NORMAL";
	case GPANEL_STATE_LP:
		return "LP";
	case GPANEL_STATE_MODESET:
		return "MODESET";
	case GPANEL_STATE_BLANK:
		return "BLANK";
	default:
		return "UNKNOWN";
	}
}

/* accessors */

static inline bool gs_is_panel_active(const struct gs_panel *ctx)
{
	switch (ctx->panel_state) {
	case GPANEL_STATE_LP:
	case GPANEL_STATE_NORMAL:
		return true;
	case GPANEL_STATE_UNINITIALIZED:
	case GPANEL_STATE_HANDOFF:
	case GPANEL_STATE_HANDOFF_MODESET:
	case GPANEL_STATE_OFF:
	case GPANEL_STATE_MODESET:
	case GPANEL_STATE_BLANK:
	default:
		return false;
	}
}

static inline bool gs_is_panel_enabled(const struct gs_panel *ctx)
{
	switch (ctx->panel_state) {
	case GPANEL_STATE_OFF:
	case GPANEL_STATE_UNINITIALIZED:
		return false;
	default:
		return true;
	}
}

/**
 * is_panel_initialized - indicates whether the display has been initialized at least once
 * @ctx: panel struct
 *
 * Indicates whether thepanel has been initialized at least once. Certain data such as panel
 * revision is only accurate after display initialization.
 */
static inline bool gs_is_panel_initialized(const struct gs_panel *ctx)
{
	return ctx->panel_state != GPANEL_STATE_UNINITIALIZED &&
	       ctx->panel_state != GPANEL_STATE_HANDOFF &&
	       ctx->panel_state != GPANEL_STATE_HANDOFF_MODESET;
}

/**
 * gs_get_te2_type_len() - get number of TE2 timings for the mode type
 * @desc: Panel description pointer
 * @lp_mode: Whether we're getting the number of lp_mode timings or not
 *
 * Note that sometimes the `binned_lp` entries start with an "off" entry.
 * This function reads the `has_off_binned_lp_entry` to determine whether to
 * skip that first entry or not.
 *
 * Return: number of te2 timings possible for normal or lp modes, or negative if error
 */
static inline ssize_t gs_get_te2_type_len(const struct gs_panel_desc *desc, bool lp_mode)
{
	if (!desc)
		return -EINVAL;
	if (lp_mode) {
		size_t actual_num_binned_lp = desc->has_off_binned_lp_entry ?
						      desc->num_binned_lp - 1 :
						      desc->num_binned_lp;

		if (!desc->lp_modes)
			return -EINVAL;
		else
			return desc->lp_modes->num_modes * actual_num_binned_lp;
	} else {
		if (!desc->modes)
			return -EINVAL;
		return desc->modes->num_modes;
	}
}

static inline void notify_panel_mode_changed(struct gs_panel *ctx)
{
	schedule_work(&ctx->notify_panel_mode_changed_work);
}

static inline void notify_brightness_changed(struct gs_panel *ctx)
{
	schedule_work(&ctx->notify_brightness_changed_work);
}

static inline void notify_panel_te2_rate_changed(struct gs_panel *ctx, u32 delay_ms)
{
	schedule_delayed_work(&ctx->notify_panel_te2_rate_changed_work,
			      msecs_to_jiffies(delay_ms));
}

static inline void notify_panel_te2_option_changed(struct gs_panel *ctx)
{
	schedule_work(&ctx->notify_panel_te2_option_changed_work);
}

static inline u32 get_current_frame_duration_us(struct gs_panel *ctx)
{
	return USEC_PER_SEC / drm_mode_vrefresh(&ctx->current_mode->mode);
}

static inline bool gs_is_local_hbm_post_enabling_supported(struct gs_panel *ctx)
{
	return (!ctx->lhbm.post_work_disabled && ctx->desc && ctx->desc->lhbm_desc &&
		(ctx->desc->lhbm_desc->effective_delay_frames ||
		 (ctx->desc->lhbm_desc->post_cmd_delay_frames &&
		  ctx->desc->gs_panel_func->set_local_hbm_mode_post)));
}

static inline bool gs_is_local_hbm_disabled(struct gs_panel *ctx)
{
	return (ctx->lhbm.effective_state == GLOCAL_HBM_DISABLED);
}

static inline bool gs_is_vrr_mode(const struct gs_panel_mode *pmode)
{
	return (pmode->mode.vscan > 0);
}

static inline bool gs_is_ns_op_rate(const struct gs_panel_mode *pmode)
{
	return (pmode->mode.flags & DRM_MODE_FLAG_NS);
}

static inline int gs_get_actual_vrefresh(struct gs_panel *ctx)
{
	if (ctx->idle_data.panel_idle_vrefresh)
		return ctx->idle_data.panel_idle_vrefresh;

	return drm_mode_vrefresh(&ctx->current_mode->mode);
}

/**
 * gs_panel_get_mode - Finds gs_panel_mode matching drm_display_mode for panel
 * @ctx: Pointer to panel private data structure
 * @mode: drm_display_mode to search for in possible panel modes
 *
 * This function searches the possible display modes of the panel for one that
 * matches the given `mode` argument (as per `drm_mode_equal`)
 *
 * Return: Matching gs_panel_mode for this panel, or NULL if not found
 */
const struct gs_panel_mode *gs_panel_get_mode(struct gs_panel *ctx,
					      const struct drm_display_mode *mode);

#define gs_panel_has_func(ctx, func) \
		((ctx) && ((ctx)->desc) && ((ctx)->desc->gs_panel_func)\
		 && ((ctx)->desc->gs_panel_func->func))

#define for_each_drm_display_mode_in_array(i, mode, mode_array)                  \
	for (i = 0, mode = mode_array->modes[i].mode; i < mode_array->num_modes; \
	     i++, mode = mode_array->modes[i].mode)

#define for_each_display_mode(i, mode, ctx) \
	for_each_drm_display_mode_in_array(i, mode, ctx->desc->modes)

#define for_each_gs_binned_lp(i, binned_lp, ctx)                                        \
	for (i = 0, binned_lp = &ctx->desc->binned_lp[i]; i < ctx->desc->num_binned_lp; \
	     i++, binned_lp = &ctx->desc->binned_lp[i])

#define for_each_te2_timing(ctx, lp_mode, data, i)                                         \
	for (data = ctx->te2.mode_data + (!(lp_mode) ? 0 : (ctx)->desc->modes->num_modes), \
	    i = gs_get_te2_type_len((ctx->desc), (lp_mode));                               \
	     i > 0; i--, data++)

u16 gs_panel_get_brightness(struct gs_panel *panel);

/** Command Functions with specific purposes **/

static inline void gs_panel_send_cmdset(struct gs_panel *ctx, const struct gs_dsi_cmdset *cmdset)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	gs_dsi_send_cmdset(dsi, cmdset, ctx->panel_rev);
}
static inline int gs_dcs_set_brightness(struct gs_panel *ctx, u16 br)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	return mipi_dsi_dcs_set_display_brightness(dsi, br);
}

/* Driver-facing functions (high-level) */

/**
 * gs_panel_first_enable_helper - Panel-level initialization for gs_panel
 * @ctx: handle for gs_panel
 *
 * This function should be called after the panel has received power,
 * and it does certain one-time initializations and configurations, including
 * reading panel module ID and serial number, getting panel revision, and
 * calling panel_init, etc.
 *
 * Return: Enable results; 0 for success, negative value for error
 */
int gs_panel_first_enable_helper(struct gs_panel *ctx);
void gs_panel_reset_helper(struct gs_panel *ctx);
int gs_panel_set_power_helper(struct gs_panel *ctx, bool on);
/**
 * gs_dsi_panel_common_init - Probe-level initialization for gs_panel
 * @dsi: dsi device pointer for panel
 * @ctx: Preallocated memory for gs_panel object
 *
 * This function performs a wide range of initialization functions at probe time
 * for gs_panel objects, including creating mutexes, parsing the device tree,
 * registering the device data, creating sysfs files, etc.
 *
 * Return: Probe results; 0 for success, negative value for error
 */
int gs_dsi_panel_common_init(struct mipi_dsi_device *dsi, struct gs_panel *ctx);
/**
 * gs_dsi_panel_common_probe() - Wrapper for gs_dsi_panel_common_init with malloc
 * @dsi: dsi device pointer for panel
 *
 * For drivers that don't need additional working state data for their panels,
 * this function calls the `kzalloc` function to allocate a `gs_panel` before
 * sending that to the `gs_dsi_panel_common_init` function.
 *
 * It is designed to plug directly into the `probe` function of the
 * `struct mipi_dsi_driver` data structure.
 *
 * Return: Probe results; 0 for success, negative value for error
 */
int gs_dsi_panel_common_probe(struct mipi_dsi_device *dsi);

/**
 * gs_dsi_panel_common_remove - Removes dsi panel
 * @dsi: dsi device pointer for panel
 *
 * Return: 0 on success, negative value for error
 */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0))
void gs_dsi_panel_common_remove(struct mipi_dsi_device *dsi);
#else
int gs_dsi_panel_common_remove(struct mipi_dsi_device *dsi);
#endif

/**
 * gs_panel_debugfs_create_cmdset - Creates a cmdset debugfs entry
 * @parent: dentry for debugfs parent folder. This will often be
 *          `gs_panel->debugfs_entries.cmdset`
 * @cmdset: cmdset to be read out from resulting debugfs entry
 * @name: name for resulting debugfs entry
 *
 * Creates a debugfs entry for the given cmdset, which will allow its contents
 * to be read for debugging purposes.
 */
void gs_panel_debugfs_create_cmdset(struct dentry *parent, const struct gs_dsi_cmdset *cmdset,
				    const char *name);

#define GS_VREFRESH_TO_PERIOD_USEC(rate) DIV_ROUND_UP(USEC_PER_SEC, (rate) ? (rate) : 60)

/**
 * gs_panel_wait_for_vblank - wait for next vblank provided by attached drm_crtc
 * @ctx: handle for gs_panel that is waiting
 *
 * Return: 0 on success, negative value for error
 */
int gs_panel_wait_for_vblank(struct gs_panel *ctx);

/**
 * gs_panel_wait_for_vsync_done - wait for the vsync signal to be done
 * @ctx: handle for gs_panel that is waiting
 * @te_us: length of te period, in us
 * @period_us: length of a clock period (TODO: verify)
 */
void gs_panel_wait_for_vsync_done(struct gs_panel *ctx, u32 te_us, u32 period_us);

/**
 * gs_panel_wait_for_vsync_done - wait for the flip done
 * @ctx: handle for gs_panel that is waiting
 * @timeout_ms: length of timeout, in ms
 */
void gs_panel_wait_for_flip_done(struct gs_panel *ctx, u32 timeout_ms);

/**
 * gs_panel_msleep - sleeps for a given number of ms
 * @delay_ms: Length of time to sleep
 *
 * This is an implementation of the normal `sleep` functions with a tie-in to
 * the panel driver's tracing utilities
 */
void gs_panel_msleep(u32 delay_ms);

/**
 * gs_panel_get_idle_time_delta() - Gets time since last idle mode or mode set
 * @ctx: handle for gs_panel
 *
 * Return: Time since last mode set or activation of idle mode, in milliseconds,
 * or UINT_MAX if unsupported.
 */
unsigned int gs_panel_get_idle_time_delta(struct gs_panel *ctx);

int gs_panel_get_current_mode_te2(struct gs_panel *ctx, struct gs_panel_te2_timing *timing);
/**
 * gs_panel_update_te2() - calls panel-specific TE2 update callback
 * @ctx: handle for gs_panel
 *
 * A number of functions will modify panel operation such that we may need to
 * update te2 configuration; this function is shorthand for executing the
 * necessary changes in the panel driver.
 */
void gs_panel_update_te2(struct gs_panel *ctx);

/**
 * gs_panel_update_lhbm_hist_data_helper() - Update lhbm_hist_data on panel connector
 * @ctx: Reference to panel data
 * @enabled: whether to enable or disable updating lhbm histogram roi data
 * @roi_type: Config different type of roi shape.
 * @circle_d: Depth of ROI center point off center, in pixels
 * @circle_r: Radius of ROI circle, in pixels
 *
 * Note that this will update d and r regardless of the enable value
 *
 * This is meant to be called by panel drivers during the `atomic_check` operation
 */
void gs_panel_update_lhbm_hist_data_helper(struct gs_panel *ctx, struct drm_atomic_state *state,
					   bool enabled,
					   enum gs_drm_connector_lhbm_hist_roi_type roi_type, int d,
					   int r);

/* Helper Utilities */

/**
 * panel_calc_gamma_2_2_luminance() - calculate prorated luminance based on gamma2.2 curve
 *
 * @value: the input to prorate the luminance on X axis of gamma2.2 curve
 * @max_value: the maximum value on the X axis
 * @nit: the luminance associated with max_value on Y axis
 *
 * Description: luminance = exp(ln(value/max_value) * 2.2) * max_Luminance, gamma_2_2_coef_x_1m
 *              stands for "exp(ln(value/max_value) * 2.2)". The function uses interpolation
 *              method to calculate the prorated luminance.
 *
 * Return: prorated luminance
 */
u32 panel_calc_gamma_2_2_luminance(const u32 value, const u32 max_value, const u32 nit);
/**
 * panel_calc_linear_luminance() - calculate prorated luminance based on linear curve
 *
 * @value: input value to prorate luminance
 * @coef_x_1k: linear coefficient multiplied by 1000
 * @offset: offset value of Y axis
 *
 * Description: luminance = coefficient * value + offset
 *
 * Return: prorated luminance
 */
u32 panel_calc_linear_luminance(const u32 value, const u32 coef_x_1k, const int offset);

/**
 * gs_dsi_cmd_align() - wait until after sending DCS would not cause frame drop by gated TE
 *
 * @ctx: Reference to panel data
 *
 * Note this function can't be called within panel context's mode_lock mutex lock.
 *
 * This function calculates proper number of us to delay(the number might be 0),
 * based on expected present timestamp, frame interval and panel property such as
 * TE period, vrefresh rate, to prevent frame drop caused by unexpected gated TE.
 * This function can be used if following sending DSI cmds would trigger panel self-scan behavior.
 */
void gs_dsi_cmd_align(struct gs_panel *ctx);

/**
 * gs_dsi_cmd_need_wait_for_present_time_locked() - check if DSI command need waiting for present time
 *
 * @ctx: Reference to panel data
 * @waiting_time_us: waiting time in us unit
 *
 * This function checks current commit present time to see if the delay is needed, which helps some
 * commands that need skip some invalid TEs to execute at the right timing.
 *
 * Return: whether it needs to wait
 */
bool gs_dsi_cmd_need_wait_for_present_time_locked(struct gs_panel *ctx, u64 *waiting_time_us);

/**
 * gs_panel_disable_normal_feat_locked() - disable normal mode features
 * @ctx: pointer to gs_panel
 *
 * This function disabled features required panel in power on state.
 * Disabled features:
 * 1. lhbm
 * 2. hbm
 *
 * Context: Expects ctx->mode_lock to be locked
 */
void gs_panel_disable_normal_feat_locked(struct gs_panel *ctx);

/* HBM */

#define GS_HBM_FLAG_GHBM_UPDATE BIT(0)
#define GS_HBM_FLAG_BL_UPDATE BIT(1)
#define GS_HBM_FLAG_LHBM_UPDATE BIT(2)
#define GS_HBM_FLAG_DIMMING_UPDATE BIT(3)
#define GS_FLAG_OP_RATE_UPDATE BIT(4)

#define GS_IS_HBM_ON(mode) ((mode) >= GS_HBM_ON_IRC_ON && (mode) < GS_HBM_STATE_MAX)
#define GS_IS_HBM_ON_IRC_OFF(mode) (((mode) == GS_HBM_ON_IRC_OFF))

#endif // DISPLAY_COMMON_PANEL_PANEL_GS_H_
