/* SPDX-License-Identifier: MIT */
/*
 * Copyright 2023 Google LLC
 *
 * Use of this source code is governed by an MIT-style
 * license that can be found in the LICENSE file or at
 * https://opensource.org/licenses/MIT.
 */
#ifndef _GS_PANEL_FUNCS_DEFAULTS_H_
#define _GS_PANEL_FUNCS_DEFAULTS_H_

#include <linux/types.h>

struct gs_panel;
struct gs_panel_mode;

/*
 * DOC: gs_panel_funcs_defaults theory
 *
 * This file contains a number of default implementations of the functions
 * outlined in the `struct gs_panel_funcs` vtable in `gs_panel.h`
 *
 * These are meant to be used directly or extended in panel-specific driver code
 * as appropriate.
 *
 * In general, these functions should perform behavior that is common to a large
 * range of our panel code.
 */

/**
 * gs_panel_get_panel_rev - Callback for getting panel rev from extinfo block
 * Currently, this would not slot directly into the `get_panel_rev` entry in the
 * vtable, because it operates on the 8-bit build code rather than the entire
 * 32-bit extinfo data block
 *
 * @ctx: Handle for gs_panel private data. In particular, it will update the
 *       `panel_rev` member variable of this struct.
 * @rev: Short-form build-code-based rev entry used to determine revision of
 *       panel
 */
void gs_panel_get_panel_rev(struct gs_panel *ctx, u8 rev);

/**
 * gs_panel_read_slsi_ddic_id - Callback for reading the panel id.
 *
 * This will read the panel id information (serial number) from the SLSI_DDIC_ID
 * reg. It is meant to be used on SLSI ddic's.
 *
 * @ctx: Handle for gs_panel private data. In particular, it will update the
 *       `panel_id` member variable of this struct.
 * Return: 0 on success, negative value on error.
 */
int gs_panel_read_slsi_ddic_id(struct gs_panel *ctx);

/**
 * gs_panel_read_id - Callback for reading the panel id.
 *
 * This will read the panel id information from the register referred to by
 * the panel_id_reg member of the gs_panel_desc, or the PANEL_ID_REG_DEFAULT
 * if no data exists for that register.
 *
 * NOTE: this function is deprecated; for new work, prefer use of
 * gs_panel_read_slsi_ddic_id, or a more vendor-applicable method.
 *
 * @ctx: Handle for gs_panel private data. In particular, it will update the
 *       `panel_id` member variable of this struct.
 * Return: 0 on success, negative value on error.
 */
int gs_panel_read_id(struct gs_panel *ctx);

/**
 * gs_panel_model_init() - Helper function to construct panel_model string
 * @ctx: Reference to panel data
 * @project: Project code to write to panel_model string
 * @extra_info: Additional info to write into panel_model string
 *
 * Context: this may be called as part of an implementation of panel_config()
 */
void gs_panel_model_init(struct gs_panel *ctx, const char *project, u8 extra_info);

/**
 * gs_panel_is_mode_seamless_helper() - Default implementation for checking
 *                                      seamless transition.
 * @ctx: Reference to panel data
 * @mode: Proposed display mode
 *
 * Checks whether the panel can transition to the new mode seamlessly without
 * having to turn the display off before the mode change.
 *
 * This implementation checks if resolution/timings and flags are the same.
 *
 * Return: true if seamless transition possible, false otherwise
 */
bool gs_panel_is_mode_seamless_helper(const struct gs_panel *ctx,
				      const struct gs_panel_mode *pmode);

/**
 * gs_panel_get_te2_edges_helper() - Get rising and falling edges of TE2 signal
 * @ctx: Reference to panel data
 * @buf: string buffer to which to write the TE2 timing data
 * @lp_mode: Whether this should display data about LP or non-LP TE2 timing
 *
 * Return: length of string written to buffer, or negative value on error
 */
ssize_t gs_panel_get_te2_edges_helper(struct gs_panel *ctx, char *buf, bool lp_mode);

/**
 * gs_panel_set_te2_edges_helper() - Configure rising/falling te2 edges
 * @ctx: Reference to panel data
 * @timings: Array of values to configure into the timings
 * @lp_mode: Whether we're configuring LP or non-LP timings
 *
 * Return: 0 on success, negative value on error.
 */
int gs_panel_set_te2_edges_helper(struct gs_panel *ctx, u32 *timings, bool lp_mode);

/**
 * gs_panel_set_binned_lp_helper() - Execute command sequences for LP modes
 * @ctx: Reference to panel data
 * @brightness: Brightness value to which the panel is being set
 *
 * This executes the correct commands for setting LP modes based on the binned
 * brightness value.
 */
void gs_panel_set_binned_lp_helper(struct gs_panel *ctx, const u16 brightness);

/**
 * gs_panel_set_lp_mode_helper() - Execute command sequence to enter LP mode
 * @ctx: Reference to panel data
 * @pmode: mode to enter
 *
 * This helper is a convenience function to execute the lp_cmdset commands.
 * If there is no cmdset defined in the gs_panel_desc, this function does nothing
 */
void gs_panel_set_lp_mode_helper(struct gs_panel *ctx, const struct gs_panel_mode *pmode);

#endif // _GS_PANEL_FUNCS_DEFAULTS_H_
