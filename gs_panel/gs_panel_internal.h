/* SPDX-License-Identifier: MIT */
/*
 * Copyright 2023 Google LLC
 *
 * Use of this source code is governed by an MIT-style
 * license that can be found in the LICENSE file or at
 * https://opensource.org/licenses/MIT.
 */
#ifndef _GS_PANEL_INTERNAL_H_
#define _GS_PANEL_INTERNAL_H_

#include <linux/device.h>
#include <drm/drm_bridge.h>
#include <drm/drm_modeset_helper_vtables.h>
#include "gs_panel/gs_panel.h"

struct gs_panel;
struct gs_drm_connector;
struct gs_panel_mode;
struct dentry;
struct mipi_dsi_device;
enum gs_panel_state;
enum display_stats_state;
struct display_stats_resolution;

/* gs_panel_connector_funcs.c */
int gs_panel_initialize_gs_connector(struct gs_panel *ctx, struct drm_device *drm_dev,
				     struct gs_drm_connector *gs_connector);
/**
 * gs_panel_set_op_hz() - Wrapper for panel-specific set_op_hz function
 * @ctx: handle for gs_panel
 * @hz: operation rate in Hz
 *
 * Return: 0 on success, negative value on error
 */
int gs_panel_set_op_hz(struct gs_panel *ctx, unsigned int hz);

/* drm_bridge_funcs.c */
const struct drm_bridge_funcs *get_panel_drm_bridge_funcs(void);
/**
 * gs_panel_set_backlight_state() - sets the state for the backlight
 * @ctx: Pointer to gs_panel
 * @panel_state: New state for the panel backlight
 *
 * Called when modifying the panel state in such a way that the backlight state
 * may also need to change. Handles notifying backlight state changes and other
 * bookkeeping regarding drm properties.
 */
void gs_panel_set_backlight_state(struct gs_panel *ctx, enum gs_panel_state panel_state);
/**
 * gs_panel_wait_for_cmd_tx_window() - wait for correct rr transmit window
 * @current_mode: current panel mode
 * @target_mode: mode the panel is transitioning into
 * @ctx: Pointer to gs_panel
 *
 * Some panels require sending refresh rate update (or other) commands at a
 * particular window of time. This function determines whether that is the case,
 * and waits until that point in time.
 */
void gs_panel_wait_for_cmd_tx_window(struct drm_crtc *crtc,
				     const struct gs_panel_mode *current_mode,
				     const struct gs_panel_mode *target_mode, struct gs_panel *ctx);

/* gs_panel_sysfs.c */
/**
 * gs_panel_sysfs_create_files() - Creates sysfs files for panel
 * @dev: pointer to panel's device node
 * @ctx: pointer to gs_panel
 *
 * Creates sysfs files for panel itself
 *
 * Return: Result of sysfs_create_files function
 */
int gs_panel_sysfs_create_files(struct device *dev, struct gs_panel *ctx);
/**
 * gs_panel_sysfs_create_bl_files() - Creates sysfs files for panel backlight
 * @bl_dev: pointer to backlight's device node
 * @ctx: pointer to gs_panel
 *
 * Creates sysfs files for panel backlight
 *
 * Return: Result of sysfs_create_files function
 */
int gs_panel_sysfs_create_bl_files(struct device *bl_dev, struct gs_panel *ctx);

/* drm_bridge_funcs.c */
/**
 * gs_panel_node_attach - Creates debugfs and sysfs entries for panel
 * @gs_connector: Pointer to gs_connector
 */
void gs_panel_node_attach(struct gs_drm_connector *gs_connector);

/* gs_panel_debugfs.c */
/**
 * gs_panel_create_debugfs_entries - Creates debugfs entries for panel
 * @ctx: Pointer to gs_panel
 * @parent: debugfs_entry for drm_connector panel is connected to
 *
 * This both creates the panel's debugfs folder and populates it with the
 * various debugfs files for controlling the panel. It is meant to be called as
 * part of attaching the gs_panel to the gs_drm_connector
 *
 * Return: 0 on success, negative value on error
 */
#ifdef CONFIG_DEBUG_FS
int gs_panel_create_debugfs_entries(struct gs_panel *ctx, struct dentry *parent);
#else
static int gs_panel_create_debugfs_entries(struct gs_panel *ctx, struct dentry *parent)
{
	return -EOPNOTSUPP;
}
#endif
/**
 * parse_u32_buf() - Parses a user-provided list of ints into a buffer
 * @src: Source buffer
 * @src_len: Size of source buffer
 * @out: Output buffer for parsed u32s
 * @out_len: Size out output buffer
 *
 * This is a convenience function for parsing a user-provided list of unsigned
 * integers into a buffer. It is meant primarily for handling command-line
 * input, like for a sysfs node.
 *
 * Return: Number of integers parsed
 */
int parse_u32_buf(char *src, size_t src_len, u32 *out, size_t out_len);

/* gs_panel_lhbm.c */
/**
 * gs_panel_init_lhbm() - Initializes lhbm data, threads, etc.
 * @ctx: panel struct
 *
 * Meant to be called during common_init function. This sets up, based on the
 * capabilities of the gs_panel_desc (specifically the function capabilities),
 * the various lhbm-related data, threads, and callbacks for the panel.
 */
void gs_panel_init_lhbm(struct gs_panel *ctx);
/**
 * panel_update_lhbm() - Updates lhbm state to match requested state
 * @ctx: panel struct
 *
 * Updates lhbm state to match the requested state. This primarily wraps
 * panel_update_lhbm_notimeout(), except it also handles updating the lhbm
 * timeout.
 *
 * Context: Expects ctx->mode_lock to be locked
 */
void panel_update_lhbm(struct gs_panel *ctx);

/* gs_dsi_dcs_helper.c */
/**
 * gs_dsi_dcs_transfer - Executes a dsi dcs transfer
 * @dsi: handle for dsi device
 * @type: type of transfer
 * @data: data to transfer
 * @len: length of data
 * @flags: flags for transfer
 *
 * This function is more granular than the public-facing
 * `gs_dsi_dcs_write_buffer` function, in that it allows for explicitly setting
 * the `type` argument. It is not exposed outwardly to reduce API redundancy,
 * but it is retained here in order to allow some internal access (for example,
 * for the debugfs dsi interface)
 *
 * Return: result of transfer operation
 */
ssize_t gs_dsi_dcs_transfer(struct mipi_dsi_device *dsi, u8 type, const void *data, size_t len,
			    u16 flags);

/* gs_panel.c */
void panel_update_idle_mode_locked(struct gs_panel *ctx, bool allow_delay_update);

/**
 * gs_panel_set_dimming() - Executes set_dimming function of panel driver
 * @ctx: handle for gs_panel
 * @dimming_on: Whether to enable or disable dimming feature
 *
 * If panel has a set_dimming() function, executes it
 */
void gs_panel_set_dimming(struct gs_panel *ctx, bool dimming_on);

/**
 * get_gs_panel_connector_crtc() - Get crtc associated with panel
 * @ctx: panel struct
 *
 * Convenience method to retrieve the crtc associated with the panel's connector
 * Note that it may return NULL if the connector has no state
 *
 * Return: crtc attached to panel
 */
struct drm_crtc *get_gs_panel_connector_crtc(struct gs_panel *ctx);

/**
 * gs_set_te2_timing() - handle for setting te2 timings from sysfs node
 * @ctx: panel struct
 * @count: size of the input buffer
 * @buf: input buffer provided to sysfs node
 * @lp_mode: whether these timings apply to LP modes
 *
 * This function is called by both the normal and low-power versions of the
 * te2_store functions from the sysfs nodes. It consumes the userspace command,
 * parses it, and passes (valid) parsed data to the appropriate function to
 * actually modify the te2 timings for the panel modes
 *
 * Return: number of bytes consumed by input buffer, or negative value on error
 */
ssize_t gs_set_te2_timing(struct gs_panel *ctx, size_t count, const char *buf, bool lp_mode);

/**
 * gs_panel_set_vddd_voltage() - Sets appropriate voltage on vddd
 * @ctx: Pointer to gs_panel
 * @is_lp: whether we're setting voltage for an lp mode
 */
void gs_panel_set_vddd_voltage(struct gs_panel *ctx, bool is_lp);

/**
 * get_gs_drm_connector_parent - gets the connector that is panel's parent
 * @ctx: Pointer to panel
 *
 * Return:  Pointer to parent connector, or NULL if error
 */
struct gs_drm_connector *get_gs_drm_connector_parent(const struct gs_panel *ctx);

/**
 * gs_connector_to_panel - get gs_panel object attached to given gs_connector
 * @gs_connector: Pointer to gs_connector
 *
 * This function returns the gs_panel that was connected to the gs_drm_connector
 * during the connector's probe function. The misdirection with the
 * mipi_dsi_device is in service to decoupling dependencies between the two
 * modules.
 *
 * Return: pointer to gs_panel attached to connector, or NULL on error.
 */
struct gs_panel *gs_connector_to_panel(const struct gs_drm_connector *gs_connector);

enum display_stats_state gs_get_current_display_state_locked(struct gs_panel *ctx);
const char *get_disp_state_str(enum display_stats_state state);

/**
 * get_disp_stats_time_state_idx() - get the index of display stats table
 * @ctx: handle for gs_panel
 * @state: display state
 * @vrefresh: display refresh rate
 * @res: display resolution
 *
 * The display stats table provides residency data for various display configurations,
 * including display state, refresh rate, and resolution. This function is used to obtain
 * the index of the display stats that corresponds to a specific display configuration.
 *
 * Return: index of display stats table.
 */
int get_disp_stats_time_state_idx(struct gs_panel *ctx,
		enum display_stats_state state, int vrefresh, struct display_stats_resolution res);

/**
 * gs_panel_set_fake_color_data() - Write fake color data to panel data
 *
 * @ctx: handle for gs_panel
 * @options: input array, [X, offset, colordata...]
 * @count: number of items in colordata
 */
int gs_panel_set_fake_color_data(struct gs_panel *ctx, u32 *options, int count);

/**
 * gs_panel_allocate_color_data() - Sets color data size and updates color_data allocation
 * if a new data size is required.
 *
 * @ctx: handle for gs_panel
 * @option: type of data to allocate memory for
 *
 * Return: 0 on success, or error code on failure.
 */
int gs_panel_allocate_color_data(struct gs_panel *ctx, u32 option);

/**
 * gs_panel_validate_color_option() - Validates read type and option against the supported
 * options in the panel's calibration description.
 *
 * @ctx: handle for gs_panel
 * @read_type: type of color data
 * @option: option setting for color data read
 */
int gs_panel_validate_color_option(struct gs_panel *ctx, enum color_data_type read_type,
				   int option);

#endif // _GS_PANEL_INTERNAL_H_
