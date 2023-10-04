/* SPDX-License-Identifier: GPL-2.0-only
 *
 * MIPI-DSI based Samsung common panel driver header
 *
 * Copyright (c) 2019 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _PANEL_SAMSUNG_DRV_
#define _PANEL_SAMSUNG_DRV_

#include <linux/printk.h>
#include <linux/bits.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio/consumer.h>
#include <linux/backlight.h>
#include <drm/drm_bridge.h>
#include <drm/drm_connector.h>
#include <drm/drm_crtc.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <drm/drm_property.h>
#include <drm/drm_mipi_dsi.h>

#include "../exynos_drm_connector.h"

#define MAX_REGULATORS		3
#define MAX_HDR_FORMATS		4
#define MAX_BL_RANGES		10

#define MAX_TE2_TYPE			10
#define FIXED_TE2_VREFRESH_NORMAL	120
#define FIXED_TE2_VREFRESH_LP		30

#define HDR_DOLBY_VISION	BIT(1)
#define HDR_HDR10		BIT(2)
#define HDR_HLG			BIT(3)

#define BL_STATE_STANDBY	BL_CORE_FBBLANK
#define BL_STATE_LP		BIT(30) /* backlight is in LP mode */

#define DEFAULT_GAMMA_STR	"default"

#define PANEL_REV_PROTO1	BIT(0)
#define PANEL_REV_PROTO1_1	BIT(1)
#define PANEL_REV_PROTO1_2	BIT(2)
#define PANEL_REV_PROTO2	BIT(3)
#define PANEL_REV_EVT1		BIT(4)
#define PANEL_REV_EVT1_0_2	BIT(5)
#define PANEL_REV_EVT1_1	BIT(6)
#define PANEL_REV_EVT1_2	BIT(7)
#define PANEL_REV_EVT2		BIT(8)
#define PANEL_REV_DVT1		BIT(9)
#define PANEL_REV_DVT1_1	BIT(10)
#define PANEL_REV_PVT		BIT(11)
#define PANEL_REV_LATEST	BIT(31)
#define PANEL_REV_ALL		(~0)
#define PANEL_REV_GE(rev)	(~((rev) - 1))
#define PANEL_REV_LT(rev)	((rev) - 1)
#define PANEL_REV_ALL_BUT(rev)	(PANEL_REV_ALL & ~(rev))

/* indicates that all commands in this cmd set should be batched together */
#define PANEL_CMD_SET_BATCH  BIT(0)
/*
 * indicates that all commands in this cmd set should be queued, a follow up
 * command should take care of triggering transfer of batch
 */
#define PANEL_CMD_SET_QUEUE  BIT(1)

/* packetgo feature to batch msgs can wait for vblank, use this flag to ignore explicitly */
#define PANEL_CMD_SET_IGNORE_VBLANK BIT(2)


#define HBM_FLAG_GHBM_UPDATE    BIT(0)
#define HBM_FLAG_BL_UPDATE      BIT(1)
#define HBM_FLAG_LHBM_UPDATE    BIT(2)
#define HBM_FLAG_DIMMING_UPDATE BIT(3)

#define IS_HBM_ON(mode)	((mode) >= HBM_ON_IRC_ON && (mode) < HBM_STATE_MAX)
#define IS_HBM_ON_IRC_OFF(mode)	(((mode) == HBM_ON_IRC_OFF))

/**
 * enum exynos_panel_state - panel operating state
 * @PANEL_STATE_UNINITIALIZED: Panel has never been initialized, and panel OTP info such as
 *			       panel serial and revision has not been read yet
 * @PANEL_STATE_HANDOFF: Panel looked active when driver was loaded. The panel is uninitialized
 *			in this state and will switch to PANEL_STATE_ON once it gets initialized
 * @PANEL_STATE_HANDOFF_MODESET: Similar to HANDOFF state, in this case a modeset was called with
 *			unpreferred mode, so display must be blanked before enabling.
 * @PANEL_STATE_OFF: Panel is fully disabled and powered off
 * @PANEL_STATE_NORMAL: Panel is ON in Normal operating mode
 * @PANEL_STATE_LP: Panel is ON in Low Power mode
 * @PANEL_STATE_MODESET: Going through modeset, where panel gets disable/enable calls with new mode
 * @PANEL_STATE_BLANK: Panel is ON but no contents are shown on display
 */
enum exynos_panel_state {
	PANEL_STATE_UNINITIALIZED,
	PANEL_STATE_HANDOFF,
	PANEL_STATE_HANDOFF_MODESET,
	PANEL_STATE_OFF,
	PANEL_STATE_NORMAL,
	PANEL_STATE_LP,
	PANEL_STATE_MODESET,
	PANEL_STATE_BLANK,
	PANEL_STATE_COUNT,
};

/**
 * enum exynos_panel_idle_mode - type of idle mode supported per mode
 * @IDLE_MODE_UNSUPPORTED: No idle mode is supported in this mode
 * @IDLE_MODE_ON_INACTIVITY: In this mode the panel can go into idle automatically
 *                           after last frame update
 * @IDLE_MODE_ON_SELF_REFRESH: Manually go into lower idle mode when display enters
 *                             self refresh state
 */
enum exynos_panel_idle_mode {
	IDLE_MODE_UNSUPPORTED,
	IDLE_MODE_ON_INACTIVITY,
	IDLE_MODE_ON_SELF_REFRESH,
};

/**
 * enum exynos_panel_te2_opt - option of TE2 frequency
 * @TE2_OPT_CHANGEABLE: TE2 frequency follows display refresh rate
 * @TE2_OPT_FIXED: TE2 frequency is fixed at 120Hz. Only supported on specific panels
 */
enum exynos_panel_te2_opt {
	TE2_OPT_CHANGEABLE,
	TE2_OPT_FIXED,
};

enum exynos_cabc_mode {
	CABC_OFF = 0,
	CABC_UI_MODE,
	CABC_STILL_MODE,
	CABC_MOVIE_MODE,
};

struct exynos_panel;

struct exynos_panel_te2_timing {
	/* @rising_edge: vertical start point */
	u16 rising_edge;
	/* @falling_edge: vertical end point */
	u16 falling_edge;
};

/**
 * struct exynos_panel_mode - panel mode info
 */
struct exynos_panel_mode {
	/* @mode: drm display mode info */
	struct drm_display_mode mode;

	/* @exynos_mode: exynos driver specific mode info */
	struct exynos_display_mode exynos_mode;

	/* @priv_data: per mode panel driver private data */
	const void *priv_data;

	/* @te2_timing: TE2 signal timing */
	struct exynos_panel_te2_timing te2_timing;

	/*
	 * @idle_mode:
	 *
	 * Indicates whether going into lower refresh rate is allowed while in this mode, and what
	 * type of idle mode is supported, for more info refer to enum exynos_panel_idle_mode.
	 */
	enum exynos_panel_idle_mode idle_mode;
};

struct exynos_panel_funcs {
	/**
	 * @set_brightness:
	 *
	 * This callback is used to implement driver specific logic for brightness
	 * configuration. Otherwise defaults to sending brightness commands through
	 * dcs command update
	 */
	int (*set_brightness)(struct exynos_panel *exynos_panel, u16 br);

	/**
	 * @set_lp_mode:
	 *
	 * This callback is used to handle command sequences to enter low power modes.
	 */
	void (*set_lp_mode)(struct exynos_panel *exynos_panel,
			    const struct exynos_panel_mode *mode);

	/**
	 * @set_nolp_mode:
	 *
	 * This callback is used to handle command sequences to exit from low power
	 * modes.
	 */
	void (*set_nolp_mode)(struct exynos_panel *exynos_panel,
			      const struct exynos_panel_mode *mode);

	/**
	 * @set_binned_lp:
	 *
	 * This callback is used to handle additional command sequences for low power
	 * modes based on different brightness threshold.
	 */
	void (*set_binned_lp)(struct exynos_panel *exynos_panel, u16 br);

	/**
	 * @set_hbm_mode:
	 *
	 * This callback is used to implement panel specific logic for high brightness
	 * mode enablement. If this is not defined, it means that panel does not
	 * support HBM
	 */
	void (*set_hbm_mode)(struct exynos_panel *exynos_panel, enum exynos_hbm_mode mode);

	/**
	 * @set_dimming_on:
	 *
	 * This callback is used to implement panel specific logic for dimming mode
	 * enablement. If this is not defined, it means that panel does not support
	 * dimmimg.
	 */
	void (*set_dimming_on)(struct exynos_panel *exynos_panel,
				 bool dimming_on);

	/**
	 * @set_cabc_mode:
	 *
	 * This callback is used to implement panel specific logic for cabc mode
	 * enablement. If this is not defined, it means that panel does not support
	 * cabc.
	 */
	void (*set_cabc_mode)(struct exynos_panel *exynos_panel,
				 enum exynos_cabc_mode mode);

	/**
	 * @set_local_hbm_mode:
	 *
	 * This callback is used to implement panel specific logic for local high
	 * brightness mode enablement. If this is not defined, it means that panel
	 * does not support local HBM
	 */
	void (*set_local_hbm_mode)(struct exynos_panel *exynos_panel,
				 bool local_hbm_en);
	/**
	 * @set_power:
	 *
	 * This callback is used to implement panel specific power on/off sequence.
	 */
	int (*set_power)(struct exynos_panel *exynos_panel, bool enable);

	/**
	 * @is_mode_seamless:
	 *
	 * This callback is used to check if a switch to a particular mode can be done
	 * seamlessly without full mode set given the current hardware configuration
	 */
	bool (*is_mode_seamless)(const struct exynos_panel *exynos_panel,
				 const struct exynos_panel_mode *mode);

	/**
	 * @mode_set:
	 *
	 * This callback is used to perform driver specific logic for mode_set.
	 * This could be called while display is on or off, should check internal
	 * state to perform appropriate mode set configuration depending on this state.
	 */
	void (*mode_set)(struct exynos_panel *exynos_panel,
			 const struct exynos_panel_mode *mode);

	/**
	 * @panel_init:
	 *
	 * This callback is used to do one time initialization for any panel
	 * specific functions.
	 */
	void (*panel_init)(struct exynos_panel *exynos_panel);

	/**
	 * @panel_reset:
	 *
	 * This callback is used to allow panel to toggle only reset pin instead of full
	 * prepare sequence (including power rails) while the device is in BLANK state.
	 * This is not called in any other state.
	 */
	void (*panel_reset)(struct exynos_panel *exynos_panel);

	/**
	 * @print_gamma:
	 *
	 * This callback is used to print the hex dump of gamma address/data
	 * for the provided mode.
	 *
	 * The expected format:
	 * [gamma address]: [gamma data...]
	 */
	void (*print_gamma)(struct seq_file *seq,
			    const struct drm_display_mode *mode);

	/**
	 * @gamma_store:
	 *
	 * This callback is used to store the user-provided gamma table.
	 * The user-provided table should include FPS, register, register
	 * data size and gamma data.
	 *
	 * The expected format:
	 * [FPS1]
	 * [register1]
	 * [register1 data size]
	 * [gamma data]
	 * [register2]
	 * [register2 data size]
	 * [gamma data]
	 * [FPS2]
	 * [register1]
	 * .....
	 */
	ssize_t (*gamma_store)(struct exynos_panel *exynos_panel,
			       const char *buf, size_t len);

	/**
	 * @restore_native_gamma
	 *
	 * This callback is used to replace current gamma table by the
	 * original gamma.
	 */
	ssize_t (*restore_native_gamma)(struct exynos_panel *exynos_panel);

	/**
	 * @get_panel_rev:
	 *
	 * This callback is used to get panel HW revision from panel_extinfo.
	 */
	void (*get_panel_rev)(struct exynos_panel *exynos_panel, u32 id);

	/**
	 * @read_id:
	 *
	 * This callback is used to read panel id.
	 */
	int (*read_id)(struct exynos_panel *exynos_panel);

	/**
	 * @get_te2_edges:
	 *
	 * This callback is used to get the rising and falling edges of TE2 signal.
	 * The input buf is used to store the results in string.
	 */
	ssize_t (*get_te2_edges)(struct exynos_panel *exynos_panel,
				 char *buf, bool lp_mode);

	/**
	 * @configure_te2_edges:
	 *
	 * This callback is used to configure the rising and falling edges of TE2
	 * signal. The input timings include the values we need to configure.
	 */
	int (*configure_te2_edges)(struct exynos_panel *exynos_panel,
				   u32 *timings, bool lp_mode);

	/**
	 * @update_te2:
	 *
	 * This callback is used to update the TE2 signal via DCS commands.
	 * This should be called when the display state is changed between
	 * normal and LP modes, or the refresh rate and LP brightness are
	 * changed.
	 */
	void (*update_te2)(struct exynos_panel *exynos_panel);

	/**
	 * @atomic_check
	 *
	 * This optional callback happens in atomic check phase, it gives a chance to panel driver
	 * to check and/or adjust atomic state ahead of atomic commit.
	 *
	 * Should return 0 on success (no problems with atomic commit) otherwise negative errno
	 */
	int (*atomic_check)(struct exynos_panel *exynos_panel, struct drm_atomic_state *state);

	/**
	 * @commit_done
	 *
	 * Called after atomic commit flush has completed but transfer may not have started yet
	 */
	void (*commit_done)(struct exynos_panel *exynos_panel);

	/**
	 * @set_self_refresh
	 *
	 * Called when display self refresh state has changed. While in self refresh state, the
	 * panel can optimize for power assuming that there are no pending updates.
	 *
	 * Returns true if underlying mode was updated to reflect new self refresh state,
	 * otherwise returns false if no action was taken.
	 */
	bool (*set_self_refresh)(struct exynos_panel *exynos_panel, bool enable);

	/**
	 * @set_op_hz
	 *
	 * set display panel working on specified operation rate.
	 *
	 * Returns 0 if successfully setting operation rate.
	 */
	int (*set_op_hz)(struct exynos_panel *exynos_panel, unsigned int hz);

	/**
	 * @set_osc2_clk_khz
	 *
	 * Set specific OSC2 clock for panel.
	 */
	void (*set_osc2_clk_khz)(struct exynos_panel *exynos_panel, unsigned int clk_khz);

	/**
	 * @list_osc2_clk_khz
	 *
	 * List supported OSC2 clock for panel.
	 */
	ssize_t (*list_osc2_clk_khz)(struct exynos_panel *exynos_panel, char *buf);

	/**
	 * @parse_regulators
	 *
	 * Parse regulators for panel.
	 */
	int (*parse_regulators)(struct exynos_panel *ctx);
};

/**
 * struct exynos_dsi_cmd - information for a dsi command.
 * @cmd_len:  Length of a dsi command.
 * @cmd:      Pointer to a dsi command.
 * @delay_ms: Delay time after executing this dsi command.
 * @panel_rev:Send the command only when the panel revision is matched.
 */
struct exynos_dsi_cmd {
	u32 cmd_len;
	const u8 *cmd;
	u32 delay_ms;
	u32 panel_rev;
	u8 type;
};

/**
 * struct exynos_dsi_cmd_set - a dsi command sequence.
 * @num_cmd:  Number of dsi commands in this sequence.
 * @cmds:     Pointer to a dsi command sequence.
 */
struct exynos_dsi_cmd_set {
	const u32 num_cmd;
	const struct exynos_dsi_cmd *cmds;
};

/**
 * struct exynos_binned_lp - information for binned lp mode.
 * @name:         Name of this binned lp mode.
 * @bl_threshold: Max brightness supported by this mode
 * @cmd_set:      A dsi command sequence to enter this mode.
 * @te2_timing:   TE2 signal timing.
 */
struct exynos_binned_lp {
	const char *name;
	u32 bl_threshold;
	struct exynos_dsi_cmd_set cmd_set;
	struct exynos_panel_te2_timing te2_timing;
};

struct exynos_panel_desc {
	const u8 *dsc_pps;
	u8 panel_id_reg;
	u32 dsc_pps_len;
	u32 data_lane_cnt;
	u32 hdr_formats; /* supported HDR formats bitmask */
	u32 max_luminance;
	u32 max_avg_luminance;
	u32 min_luminance;
	u32 max_brightness;
	u32 min_brightness;
	u32 lower_min_brightness; /* extreme low brightness */
	u32 dft_brightness; /* default brightness */
	u32 vrr_switch_duration;
	/* extra frame is needed to apply brightness change if it's not at next VSYNC */
	bool dbv_extra_frame;
	bool is_partial;
	bool is_panel_idle_supported;
	bool no_lhbm_rr_constraints;
	bool mipi_sync_te_prediction;
	const u32 lhbm_effective_delay_frames;
	const unsigned int delay_dsc_reg_init_us;
	const struct brightness_capability *brt_capability;
	const u32 *bl_range;
	u32 bl_num_ranges;
	const struct exynos_panel_mode *modes;
	size_t num_modes;
	const struct exynos_dsi_cmd_set *off_cmd_set;
	/* @lp_mode: provides a low power mode if available, otherwise null */
	const struct exynos_panel_mode *lp_mode;
	const size_t lp_mode_count;
	const struct exynos_dsi_cmd_set *lp_cmd_set;
	const struct exynos_binned_lp *binned_lp;
	const size_t num_binned_lp;
	const struct drm_panel_funcs *panel_func;
	const struct exynos_panel_funcs *exynos_panel_func;
};

#define PANEL_ID_MAX		40
#define PANEL_EXTINFO_MAX	16
#define LOCAL_HBM_MAX_TIMEOUT_MS 3000 /* 3000 ms */
#define LOCAL_HBM_GAMMA_CMD_SIZE_MAX 16

enum local_hbm_enable_state {
	LOCAL_HBM_DISABLED = 0,
	LOCAL_HBM_ENABLED,
	LOCAL_HBM_ENABLING,
};

struct exynos_bl_notifier {
	u32 ranges[MAX_BL_RANGES];
	u32 num_ranges;
	u32 current_range;
};

struct te2_mode_data {
	/* @mode: normal or LP mode data */
	const struct drm_display_mode *mode;
	/* @binned_lp: LP mode data */
	const struct exynos_binned_lp *binned_lp;
	/* @timing: normal or LP mode timing */
	struct exynos_panel_te2_timing timing;
};

struct te2_data {
	struct te2_mode_data mode_data[MAX_TE2_TYPE];
	enum exynos_panel_te2_opt option;
};

struct exynos_panel {
	struct device *dev;
	struct drm_panel panel;
	struct dentry *debugfs_entry;
	struct dentry *debugfs_cmdset_entry;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *enable_gpio;
	struct regulator *vci;
	struct regulator *vddi;
	struct regulator *vddd;
	struct regulator *vddr_en;
	struct regulator *vddr;
	u32 vddd_normal_uV;
	u32 vddd_lp_uV;
	struct exynos_drm_connector exynos_connector;
	struct drm_bridge bridge;
	const struct exynos_panel_desc *desc;
	const struct exynos_panel_mode *current_mode;

	/* Deprected: please refer to panel_state instead */
	bool enabled;
	bool initialized;
	enum exynos_panel_state panel_state;
	/* If true, panel won't be powered off */
	bool force_power_on;

	/* indicates whether panel idle feature is enabled */
	bool panel_idle_enabled;
	/* indicates need to do specific handle when exiting idle on self refresh */
	bool panel_need_handle_idle_exit;
	/* indicates self refresh is active */
	bool self_refresh_active;
	/* indicates if panel brightness is set or not after reset */
	bool is_brightness_initialized;
	/**
	 * refresh rate in panel idle mode
	 * 0 means not in idle mode or not specified
	 * greater than 0 means panel idle is active
	 */
	unsigned int panel_idle_vrefresh;
	unsigned int op_hz;
	unsigned int osc2_clk_khz;
	/**
	 * indicates the lower bound of refresh rate
	 * 0 means there is no lower bound limitation
	 * -1 means display should not switch to lower
	 * refresh rate while idle.
	 */
	int min_vrefresh;
	/**
	 * When the value is set to non-zero value, the panel
	 * driver kernel idle timer will be disabled internally
	 * (similar to writing 0 to panel_idle sysfs node) for
	 * at least the delay time provided after a refresh rate update.
	 */
	u32 idle_delay_ms;
	int peak_vrefresh;

	enum exynos_hbm_mode hbm_mode;
	bool dimming_on;
	/* indicates the LCD backlight is controlled by DCS */
	bool bl_ctrl_dcs;
	enum exynos_cabc_mode cabc_mode;
	struct backlight_device *bl;
	struct mutex mode_lock;
	struct mutex bl_state_lock;
	struct exynos_bl_notifier bl_notifier;

	struct mutex lp_state_lock;
	const struct exynos_binned_lp *current_binned_lp;
	struct drm_property_blob *lp_mode_blob;

	char panel_id[PANEL_ID_MAX];
	char panel_extinfo[PANEL_EXTINFO_MAX];
	u32 panel_rev;
	enum drm_panel_orientation orientation;

	struct device_node *touch_dev;

	struct te2_data te2;
	ktime_t last_commit_ts;
	ktime_t last_mode_set_ts;
	ktime_t last_self_refresh_active_ts;
	ktime_t last_panel_idle_set_ts;
	struct delayed_work idle_work;

	/* Record the current CABC mode if force_off enabled */
	enum exynos_cabc_mode current_cabc_mode;

	/**
	 * Record the last refresh rate switch. Note the mode switch doesn't
	 * mean rr switch so it differs from last_mode_set_ts
	 */
	ktime_t last_rr_switch_ts;
	/* Record the last come out lp mode timestamp */
	ktime_t last_lp_exit_ts;
	u32 last_rr;

	struct {
		struct local_hbm {
			bool gamma_para_ready;
			u8 gamma_cmd[LOCAL_HBM_GAMMA_CMD_SIZE_MAX];
			union {
				enum local_hbm_enable_state state;
				enum local_hbm_enable_state enabled;
			};
			/* max local hbm on period in ms */
			u32 max_timeout_ms;
			/* work used to turn off local hbm if reach max_timeout */
			struct delayed_work timeout_work;
			struct kthread_worker worker;
			struct task_struct *thread;
			struct kthread_work post_work;
			ktime_t en_cmd_ts;
			ktime_t next_vblank_ts;
		} local_hbm;

		struct workqueue_struct *wq;
	} hbm;
};

/**
 * is_panel_active - indicates whether the display is in interactive mode
 * @ctx: panel struct
 *
 * Indicates whether the panel is on and showing display contents. Typically any operations such as
 * backlight, hbm, mode updates, etc on the panel while in this mode should be reflected instantly
 * or on next vsync.
 */
static inline bool is_panel_active(const struct exynos_panel *ctx)
{
	switch (ctx->panel_state) {
	case PANEL_STATE_LP:
	case PANEL_STATE_NORMAL:
		return true;
	case PANEL_STATE_UNINITIALIZED:
	case PANEL_STATE_HANDOFF:
	case PANEL_STATE_HANDOFF_MODESET:
	case PANEL_STATE_MODESET:
	case PANEL_STATE_BLANK:
	case PANEL_STATE_OFF:
	default:
		return false;
	}
}

/**
 * is_panel_initialized - indicates whether the display has been initialized at least once
 * @ctx: panel struct
 *
 * Indicates whether thepanel has been initialized at least once. Certain data such as panel
 * revision is only accurate after display initialization.
 */
static inline bool is_panel_initialized(const struct exynos_panel *ctx)
{
	return ctx->panel_state != PANEL_STATE_UNINITIALIZED &&
	       ctx->panel_state != PANEL_STATE_HANDOFF &&
	       ctx->panel_state != PANEL_STATE_HANDOFF_MODESET;
}

/**
 * is_panel_enabled - indicates whether the display is powered on
 * @ctx: panel struct
 *
 * Indicates whether panel is powered up even if not fully initialized or in completely active mode.
 */
static inline bool is_panel_enabled(const struct exynos_panel *ctx)
{
	return ctx->panel_state != PANEL_STATE_OFF && ctx->panel_state != PANEL_STATE_UNINITIALIZED;
}

static inline int exynos_dcs_write(struct exynos_panel *ctx, const void *data,
		size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);

	return mipi_dsi_dcs_write_buffer(dsi, data, len);
}

static inline int exynos_dcs_compression_mode(struct exynos_panel *ctx, bool enable)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);

	return mipi_dsi_compression_mode(dsi, enable);
}

static inline int exynos_dcs_set_brightness(struct exynos_panel *ctx, u16 br)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);

	return mipi_dsi_dcs_set_display_brightness(dsi, br);
}

static inline int exynos_get_actual_vrefresh(struct exynos_panel *ctx)
{
	if (ctx->panel_idle_vrefresh)
		return ctx->panel_idle_vrefresh;

	return drm_mode_vrefresh(&ctx->current_mode->mode);
}

static inline void exynos_bin2hex(const void *buf, size_t len,
				 char *linebuf, size_t linebuflen)
{
	const size_t max_size = (linebuflen - 1) / 2;
	const size_t count = min(max_size, len);
	char *end;

	end = bin2hex(linebuf, buf, count);
	*end = '\0';
}

static inline ssize_t exynos_get_te2_type_len(struct exynos_panel *ctx, bool lp_mode)
{
	return (lp_mode ? (ctx->desc->lp_mode_count ? : 1) * (ctx->desc->num_binned_lp - 1) :
		ctx->desc->num_modes);
}

static inline void backlight_state_changed(struct backlight_device *bl)
{
	sysfs_notify(&bl->dev.kobj, NULL, "state");
}

static inline void te2_state_changed(struct backlight_device *bl)
{
	sysfs_notify(&bl->dev.kobj, NULL, "te2_state");
}

static inline u32 get_current_frame_duration_us(struct exynos_panel *ctx)
{
	return USEC_PER_SEC / drm_mode_vrefresh(&ctx->current_mode->mode);
}

static inline bool is_local_hbm_post_enabling_supported(struct exynos_panel *ctx)
{
	return (ctx->desc && ctx->desc->lhbm_effective_delay_frames);
}

static inline bool is_local_hbm_disabled(struct exynos_panel *ctx)
{
	return (ctx->hbm.local_hbm.state == LOCAL_HBM_DISABLED);
}

#define EXYNOS_DSI_CMD_REV(cmd, delay, rev) { sizeof(cmd), cmd, delay, (u32)rev }
#define EXYNOS_DSI_CMD(cmd, delay) EXYNOS_DSI_CMD_REV(cmd, delay, PANEL_REV_ALL)
#define EXYNOS_DSI_CMD0_REV(cmd, rev) EXYNOS_DSI_CMD_REV(cmd, 0, rev)
#define EXYNOS_DSI_CMD0(cmd) EXYNOS_DSI_CMD(cmd, 0)

#define EXYNOS_DSI_CMD_SEQ_DELAY_REV(rev, delay, seq...) \
	EXYNOS_DSI_CMD_REV(((const u8 []){seq}), delay, rev)
#define EXYNOS_DSI_CMD_SEQ_DELAY(delay, seq...) \
	EXYNOS_DSI_CMD_SEQ_DELAY_REV(PANEL_REV_ALL, delay, seq)
#define EXYNOS_DSI_CMD_SEQ_REV(rev, seq...) \
	EXYNOS_DSI_CMD_SEQ_DELAY_REV(rev, 0, seq)
#define EXYNOS_DSI_CMD_SEQ(seq...) EXYNOS_DSI_CMD_SEQ_DELAY(0, seq)

#define DEFINE_EXYNOS_CMD_SET(name) \
	const struct exynos_dsi_cmd_set name ## _cmd_set = {			\
		.num_cmd = ARRAY_SIZE(name ## _cmds), .cmds = name ## _cmds }

#define BINNED_LP_MODE(mode_name, bl_thr, cmdset)	\
{							\
	.name = mode_name,				\
	.bl_threshold = bl_thr,				\
	{ .num_cmd = ARRAY_SIZE(cmdset),		\
	  .cmds = cmdset },				\
	{ .rising_edge = 0,				\
	  .falling_edge = 0 }				\
}

#define BINNED_LP_MODE_TIMING(mode_name, bl_thr, cmdset, rising, falling)	\
{										\
	.name = mode_name,							\
	.bl_threshold = bl_thr,							\
	{ .num_cmd = ARRAY_SIZE(cmdset),					\
	  .cmds = cmdset },							\
	{ .rising_edge = rising,						\
	  .falling_edge = falling }						\
}

#define EXYNOS_DCS_WRITE_PRINT_ERR(ctx, cmd, len, ret) do {	\
	dev_err(ctx->dev, "failed to write cmd (%d)\n", ret);	\
	print_hex_dump(KERN_ERR, "command: ",			\
		DUMP_PREFIX_NONE, 16, 1, cmd, len, false);	\
} while (0)

#define EXYNOS_DCS_WRITE_SEQ(ctx, seq...) do {				\
	u8 d[] = { seq };						\
	int ret;							\
	ret = exynos_dcs_write(ctx, d, ARRAY_SIZE(d));			\
	if (ret < 0)							\
		EXYNOS_DCS_WRITE_PRINT_ERR(ctx, d, ARRAY_SIZE(d), ret);	\
} while (0)

#define EXYNOS_DCS_WRITE_SEQ_FLAGS(ctx, flags, seq...) do {				\
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);	\
	u8 d[] = { seq };						\
	int ret;							\
	ret = exynos_dsi_dcs_write_buffer(dsi, d, ARRAY_SIZE(d), flags);		\
	if (ret < 0)							\
		EXYNOS_DCS_WRITE_PRINT_ERR(ctx, d, ARRAY_SIZE(d), ret);	\
} while (0)

#define EXYNOS_DCS_WRITE_SEQ_DELAY(ctx, delay, seq...) do {		\
	EXYNOS_DCS_WRITE_SEQ(ctx, seq);					\
	usleep_range(delay * 1000, delay * 1000 + 10);			\
} while (0)

#define EXYNOS_DCS_WRITE_TABLE(ctx, table) do {					\
	int ret;								\
	ret = exynos_dcs_write(ctx, table, ARRAY_SIZE(table));			\
	if (ret < 0)								\
		EXYNOS_DCS_WRITE_PRINT_ERR(ctx, table, ARRAY_SIZE(table), ret);	\
} while (0)

#define EXYNOS_DCS_WRITE_TABLE_FLAGS(ctx, table, flags) do {					\
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);	\
	int ret;								\
	ret = exynos_dsi_dcs_write_buffer(dsi, table, ARRAY_SIZE(table), flags);		\
	if (ret < 0)								\
		EXYNOS_DCS_WRITE_PRINT_ERR(ctx, table, ARRAY_SIZE(table), ret);	\
} while (0)

#define EXYNOS_DCS_WRITE_TABLE_DELAY(ctx, delay, table) do {		\
	EXYNOS_DCS_WRITE_TABLE(ctx, table);				\
	usleep_range(delay * 1000, delay * 1000 + 10);			\
} while (0)

#define EXYNOS_DCS_BUF_ADD(ctx, seq...) \
	EXYNOS_DCS_WRITE_SEQ_FLAGS(ctx, 0, seq)

#define EXYNOS_DCS_BUF_ADD_SET(ctx, set) \
	EXYNOS_DCS_WRITE_TABLE_FLAGS(ctx, set, 0)

#define EXYNOS_DCS_BUF_ADD_AND_FLUSH(ctx, seq...) \
	EXYNOS_DCS_WRITE_SEQ_FLAGS(ctx, EXYNOS_DSI_MSG_IGNORE_VBLANK | \
		MIPI_DSI_MSG_LASTCOMMAND, seq)

#define EXYNOS_DCS_BUF_ADD_SET_AND_FLUSH(ctx, set) \
	EXYNOS_DCS_WRITE_TABLE_FLAGS(ctx, set, EXYNOS_DSI_MSG_IGNORE_VBLANK | \
		MIPI_DSI_MSG_LASTCOMMAND)

#define DSC_PPS_SIZE sizeof(struct drm_dsc_picture_parameter_set)

#define EXYNOS_PPS_WRITE_BUF(ctx, payload) do {				\
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);	\
	int ret;							\
	ret = mipi_dsi_picture_parameter_set(dsi,			\
		(struct drm_dsc_picture_parameter_set *)(payload));	\
	if (ret < 0)							\
		dev_err(ctx->dev, "failed to write pps(%d)\n", ret);	\
} while (0)

#define EXYNOS_PPS_LONG_WRITE(ctx) EXYNOS_PPS_WRITE_BUF(ctx, ctx->desc->dsc_pps)

#define for_each_display_mode(i, mode, ctx)			\
	for (i = 0, mode = &ctx->desc->modes[i].mode;		\
		i < ctx->desc->num_modes; i++,			\
		mode = &ctx->desc->modes[i].mode)		\

#define for_each_exynos_binned_lp(i, binned_lp, ctx)		\
	for (i = 0, binned_lp = &ctx->desc->binned_lp[i];	\
		i < ctx->desc->num_binned_lp; i++,		\
		binned_lp = &ctx->desc->binned_lp[i])		\

#define for_each_te2_timing(ctx, lp_mode, data, i)					\
	for (data = ctx->te2.mode_data + (!(lp_mode) ? 0 : (ctx)->desc->num_modes),	\
	i = exynos_get_te2_type_len((ctx), (lp_mode));					\
	i > 0;										\
	i--, data++)									\

#define EXYNOS_VREFRESH_TO_PERIOD_USEC(rate) DIV_ROUND_UP(USEC_PER_SEC, (rate) ? (rate) : 60)

int exynos_panel_wait_for_vblank(struct exynos_panel *ctx);
void exynos_panel_wait_for_vsync_done(struct exynos_panel *ctx, u32 te_us, u32 period_us);
unsigned int panel_get_idle_time_delta(struct exynos_panel *ctx);
int exynos_panel_configure_te2_edges(struct exynos_panel *ctx,
				     u32 *timings, bool lp_mode);
ssize_t exynos_panel_get_te2_edges(struct exynos_panel *ctx,
				   char *buf, bool lp_mode);
int exynos_panel_get_current_mode_te2(struct exynos_panel *ctx,
				      struct exynos_panel_te2_timing *timing);
int exynos_panel_get_modes(struct drm_panel *panel, struct drm_connector *connector);
int exynos_panel_disable(struct drm_panel *panel);
int exynos_panel_unprepare(struct drm_panel *panel);
int exynos_panel_prepare(struct drm_panel *panel);
int exynos_panel_read_id(struct exynos_panel *ctx);
int exynos_panel_read_ddic_id(struct exynos_panel *ctx);
void exynos_panel_get_panel_rev(struct exynos_panel *ctx, u8 rev);
int exynos_panel_init(struct exynos_panel *ctx);
void exynos_panel_reset(struct exynos_panel *ctx);
int exynos_panel_set_power(struct exynos_panel *ctx, bool on);
int exynos_panel_set_brightness(struct exynos_panel *exynos_panel, u16 br);
void exynos_panel_debugfs_create_cmdset(struct exynos_panel *ctx,
					struct dentry *parent,
					const struct exynos_dsi_cmd_set *cmdset,
					const char *name);
void exynos_panel_send_cmd_set_flags(struct exynos_panel *ctx, const struct exynos_dsi_cmd_set *cmd_set,
			       u32 flags);
static inline void exynos_panel_send_cmd_set(struct exynos_panel *ctx,
					     const struct exynos_dsi_cmd_set *cmd_set)
{
	exynos_panel_send_cmd_set_flags(ctx, cmd_set, 0);
}
void exynos_panel_set_lp_mode(struct exynos_panel *ctx, const struct exynos_panel_mode *pmode);
void exynos_panel_set_binned_lp(struct exynos_panel *ctx, const u16 brightness);
int exynos_panel_common_init(struct mipi_dsi_device *dsi,
				struct exynos_panel *ctx);
ssize_t exynos_dsi_dcs_write_buffer(struct mipi_dsi_device *dsi,
				const void *data, size_t len, u16 flags);
ssize_t exynos_dsi_cmd_send_flags(struct mipi_dsi_device *dsi, u16 flags);

int exynos_panel_probe(struct mipi_dsi_device *dsi);
int exynos_panel_remove(struct mipi_dsi_device *dsi);

static inline void exynos_dsi_dcs_write_buffer_force_batch_begin(struct mipi_dsi_device *dsi)
{
	exynos_dsi_dcs_write_buffer(dsi, NULL, 0, EXYNOS_DSI_MSG_FORCE_BATCH);
}

static inline void exynos_dsi_dcs_write_buffer_force_batch_end(struct mipi_dsi_device *dsi)
{
	exynos_dsi_dcs_write_buffer(dsi, NULL, 0,
				    EXYNOS_DSI_MSG_FORCE_FLUSH | EXYNOS_DSI_MSG_IGNORE_VBLANK);
}

#endif /* _PANEL_SAMSUNG_DRV_ */
