/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2020 Google LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _EXYNOS_DRM_CONNECTOR_H_
#define _EXYNOS_DRM_CONNECTOR_H_

#include <drm/drm_atomic.h>
#include <drm/drm_connector.h>
#include <drm/samsung_drm.h>
#include <drm/drm_dsc.h>

#define MIN_WIN_BLOCK_WIDTH	8
#define MIN_WIN_BLOCK_HEIGHT	1

enum exynos_hbm_mode {
	HBM_OFF = 0,
	HBM_ON_IRC_ON,
	HBM_ON_IRC_OFF,
	HBM_STATE_MAX
};

enum exynos_mipi_sync_mode {
	MIPI_CMD_SYNC_NONE = BIT(0),
	MIPI_CMD_SYNC_REFRESH_RATE = BIT(1),
	MIPI_CMD_SYNC_LHBM = BIT(2),
	MIPI_CMD_SYNC_GHBM = BIT(3),
	MIPI_CMD_SYNC_BL = BIT(4),
};

struct exynos_drm_connector;

/** Private DSI msg flags **/

/* packetgo feature to batch msgs can wait for vblank, use this flag to ignore */
#define EXYNOS_DSI_MSG_IGNORE_VBLANK  BIT(14)
/* Mark the start of mipi commands transaction. Following commands should not be
 * sent to panel until see a EXYNOS_DSI_MSG_FORCE_FLUSH flag
 */
#define EXYNOS_DSI_MSG_FORCE_BATCH BIT(13)
/* Mark the end of mipi commands transaction */
#define EXYNOS_DSI_MSG_FORCE_FLUSH  BIT(12)

struct exynos_drm_connector_properties {
	struct drm_property *max_luminance;
	struct drm_property *max_avg_luminance;
	struct drm_property *min_luminance;
	struct drm_property *hdr_formats;
	struct drm_property *lp_mode;
	struct drm_property *global_hbm_mode;
	struct drm_property *local_hbm_on;
	struct drm_property *dimming_on;
	struct drm_property *brightness_capability;
	struct drm_property *brightness_level;
	struct drm_property *is_partial;
	struct drm_property *panel_idle_support;
	struct drm_property *mipi_sync;
	struct drm_property *panel_orientation;
	struct drm_property *vrr_switch_duration;
};

struct exynos_display_dsc {
	bool enabled;
	unsigned int dsc_count;
	unsigned int slice_count;
	unsigned int slice_height;

	const struct drm_dsc_config *cfg;

	unsigned int delay_reg_init_us;
	bool is_scrv4;
};

struct exynos_display_partial {
	bool enabled;
	unsigned int min_width;
	unsigned int min_height;
};

struct exynos_display_underrun_param {
	/* @te_idle_us: te idle (us) to calculate underrun_lp_ref */
	unsigned int te_idle_us;
	/* @te_var: te variation (percentage) to calculate underrun_lp_ref */
	unsigned int te_var;
};

/**
 * struct exynos_display_mode - exynos display specific info
 */
struct exynos_display_mode {
	/* @dsc: DSC parameters for the selected mode */
	struct exynos_display_dsc dsc;

	/* @mode_flags: DSI mode flags from drm_mipi_dsi.h */
	unsigned long mode_flags;

	/* @vblank_usec: parameter to calculate bts */
	unsigned int vblank_usec;

	/* @te_usec: command mode: TE pulse time */
	unsigned int te_usec;

	/* @bpc: display bits per component */
	unsigned int bpc;

	/* @underrun_param: parameters to calculate underrun_lp_ref when hs_clock changes */
	const struct exynos_display_underrun_param *underrun_param;

	/* @is_lp_mode: boolean, if true it means this mode is a Low Power mode */
	bool is_lp_mode;

	/**
	 * @sw_trigger:
	 *
	 * Force frame transfer to be triggered by sw instead of based on TE.
	 * This is only applicable for DSI command mode, SW trigger is the
	 * default for Video mode.
	 */
	bool sw_trigger;
};

/**
 * struct exynos_drm_connector_state - mutable connector state
 */
struct exynos_drm_connector_state {
	/* @base: base connector state */
	struct drm_connector_state base;

	/* @mode: additional mode details */
	struct exynos_display_mode exynos_mode;

	/* @seamless_possible: this is set if the current mode switch can be done seamlessly */
	bool seamless_possible;

	/* @brightness_level: panel brightness level */
	unsigned int brightness_level;

	/* @global_hbm_mode: global_hbm_mode indicator */
	enum exynos_hbm_mode global_hbm_mode;

	/* @local_hbm_on: local_hbm_on indicator */
	bool local_hbm_on;

	/* @dimming_on: dimming on indicator */
	bool dimming_on;

	/* @pending_update_flags: flags for pending update */
	unsigned int pending_update_flags;

	/*
	 * @te_from: Specify ddi interface where TE signals are received by decon.
	 *	     This is required for dsi command mode hw trigger.
	 */
	int te_from;

	/*
	 * @te_gpio: Provies the the gpio for panel TE signal.
	 *	     This is required for dsi command mode hw trigger.
	 */
	int te_gpio;

	/*
	 * @partial: Specify whether this panel supports partial update feature.
	 */
	struct exynos_display_partial partial;

	/*
	 * @mipi_sync: Indicating if the mipi command in current drm commit should be
	 *	       sent in the same vsync period as the frame.
	 */
	unsigned long mipi_sync;

	/*
	 * @panel_idle_support: Indicating display support panel idle mode. Panel can
	 *			go into idle after some idle period.
	 */
	bool panel_idle_support;

	/*
	 * @blanked_mode: Display should go into forced blanked mode, where power is on but
	 *                nothing is being displayed on screen.
	 */
	bool blanked_mode;
};

#define to_exynos_connector_state(connector_state) \
	container_of((connector_state), struct exynos_drm_connector_state, base)

struct exynos_drm_connector_funcs {
	void (*atomic_print_state)(struct drm_printer *p,
				   const struct exynos_drm_connector_state *state);
	int (*atomic_set_property)(struct exynos_drm_connector *exynos_connector,
				   struct exynos_drm_connector_state *exynos_state,
				   struct drm_property *property,
				   uint64_t val);
	int (*atomic_get_property)(struct exynos_drm_connector *exynos_connector,
				   const struct exynos_drm_connector_state *exynos_state,
				   struct drm_property *property,
				   uint64_t *val);
};

struct exynos_drm_connector_helper_funcs {
	/*
	 * @atomic_pre_commit: Update connector states before planes commit.
	 *                     Usually for mipi commands and frame content synchronization.
	 */
	void (*atomic_pre_commit)(struct exynos_drm_connector *exynos_connector,
				  struct exynos_drm_connector_state *exynos_old_state,
				  struct exynos_drm_connector_state *exynos_new_state);

	/*
	 * @atomic_commit: Update connector states after planes commit.
	 */
	void (*atomic_commit)(struct exynos_drm_connector *exynos_connector,
			      struct exynos_drm_connector_state *exynos_old_state,
			      struct exynos_drm_connector_state *exynos_new_state);
};

struct exynos_drm_connector {
	struct drm_connector base;
	const struct exynos_drm_connector_funcs *funcs;
	const struct exynos_drm_connector_helper_funcs *helper_private;
	bool needs_commit;
};

#define to_exynos_connector(connector) \
	container_of((connector), struct exynos_drm_connector, base)

bool is_exynos_drm_connector(const struct drm_connector *connector);
int exynos_drm_connector_init(struct drm_device *dev,
			      struct exynos_drm_connector *exynos_connector,
			      const struct exynos_drm_connector_funcs *funcs,
			      const struct exynos_drm_connector_helper_funcs *helper_funcs,
			      int connector_type);
int exynos_drm_connector_create_properties(struct drm_device *dev);
struct exynos_drm_connector_properties *
exynos_drm_connector_get_properties(struct exynos_drm_connector *exynos_conector);

static inline struct exynos_drm_connector_state *
crtc_get_exynos_connector_state(const struct drm_atomic_state *state,
				const struct drm_crtc_state *crtc_state)
{
	const struct drm_connector *conn;
	struct drm_connector_state *conn_state;
	int i;

	for_each_new_connector_in_state(state, conn, conn_state, i) {
		if (!(crtc_state->connector_mask & drm_connector_mask(conn)))
			continue;

		if (is_exynos_drm_connector(conn))
			return to_exynos_connector_state(conn_state);
	}

	return NULL;
}

#endif /* _EXYNOS_DRM_CONNECTOR_H_ */
