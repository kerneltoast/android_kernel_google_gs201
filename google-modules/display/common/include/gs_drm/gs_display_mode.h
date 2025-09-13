/* SPDX-License-Identifier: MIT */
/*
 * Copyright 2023 Google LLC
 *
 * Use of this source code is governed by an MIT-style
 * license that can be found in the LICENSE file or at
 * https://opensource.org/licenses/MIT.
 */

#ifndef _GS_DISPLAY_MODE_H_
#define _GS_DISPLAY_MODE_H_

#include <linux/version.h>

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 19, 0)
#include <drm/display/drm_dsc.h>
#else
#include <drm/drm_dsc.h>
#endif

/* customized DRM mode type and flags */
#define DRM_MODE_FLAG_NS DRM_MODE_FLAG_CLKDIV2

// BTS need takes operation rate into account
#define DRM_MODE_FLAG_BTS_OP_RATE DRM_MODE_FLAG_NVSYNC
#define IS_BTS2OPRATE_MODE(t) ((t) & DRM_MODE_FLAG_BTS_OP_RATE)

/**
 * DRM_H_TIMING() - fills in horizontal timing in struct drm_display_mode
 * @HDISPLAY: Horizontal active region
 * @HFP: Horizontal front porch
 * @HSA: Horizontal sync
 * @HBP: Horizontal back porch
 *
 * This macro autocalculates and/or fills in the .hdisplay, .hsync_start,
 * .hsync_end, and .htotal timing parameters in the struct drm_display_mode
 * structure. It is meant to be used in the structure definition.
 */
#define DRM_H_TIMING(HDISPLAY, HFP, HSA, HBP) \
	.hdisplay = HDISPLAY,                 \
	.hsync_start = HDISPLAY + HFP,        \
	.hsync_end = HDISPLAY + HFP + HSA,    \
	.htotal = HDISPLAY + HFP + HSA + HBP

/**
 * DRM_V_TIMING() - fills in vertical timing in struct drm_display_mode
 * @VDISPLAY: Vertical active region
 * @VFP: Vertical front porch
 * @VSA: Vertical sync
 * @VBP: Vertical back porch
 *
 * This macro autocalculates and/or fills in the .vdisplay, .vsync_start,
 * .vsync_end, and .vtotal timing parameters in the struct drm_display_mode
 * structure. It is meant to be used in the structure definition.
 */
#define DRM_V_TIMING(VDISPLAY, VFP, VSA, VBP) \
	.vdisplay = VDISPLAY,                 \
	.vsync_start = VDISPLAY + VFP,        \
	.vsync_end = VDISPLAY + VFP + VSA,    \
	.vtotal = VDISPLAY + VFP + VSA + VBP

/**
 * CALC_DRM_CLOCK_HZ() - calculates drm clock value
 * @REFRESH_FREQ: Image refresh frequency, in Hz
 * @HDISPLAY: Horizontal active region
 * @HFP: Horizontal front porch
 * @HSA: Horizontal sync
 * @HBP: Horizontal back porch
 * @VDISPLAY: Vertical active region
 * @VFP: Vertical front porch
 * @VSA: Vertical sync
 * @VBP: Vertical back porch
 *
 * This macro calculated the pixel clock for the struct drm_display_mode.
 */
#define CALC_DRM_CLOCK_HZ(REFRESH_FREQ, HDISPLAY, HFP, HSA, HBP, VDISPLAY, VFP, VSA, VBP) \
	((REFRESH_FREQ) * (HDISPLAY + HFP + HSA + HBP) * (VDISPLAY + VFP + VSA + VBP))

/**
 * CHECK_DRM_CLOCK_DIVISIBLE_1000() - Checks drm clock value
 * @CLOCK: drm clock rate
 *
 * This macro enables checking drm clock is divisible by 1000
 * in compile time while always returns 0.
 */
#define CHECK_DRM_CLOCK_DIVISIBLE_1000(CLOCK) (0 * sizeof( \
	struct {static_assert((CLOCK) % 1000 == 0, "drm clock not divisible by 1000."); }))

/**
 * CALC_DRM_CLOCK_K_HZ()
 * @CLOCK: drm clock rate
 *
 * This macro checks drm clock value is divisible by 1000, and returns (drm clock value/1000).
 */
#define CALC_DRM_CLOCK_K_HZ(CLOCK) \
	(CHECK_DRM_CLOCK_DIVISIBLE_1000(CLOCK) + (CLOCK / 1000))

/**
 * DRM_MODE_TIMING() - fills in timing parameters in struct drm_display_mode
 * @REFRESH_FREQ: Image refresh frequency, in Hz
 * @HDISPLAY: Horizontal active region
 * @HFP: Horizontal front porch
 * @HSA: Horizontal sync
 * @HBP: Horizontal back porch
 * @VDISPLAY: Vertical active region
 * @VFP: Vertical front porch
 * @VSA: Vertical sync
 * @VBP: Vertical back porch
 *
 * This macro calculates the pixel clock for use in the struct drm_display_mode
 * structure as well as the horizontal and vertical timing parameters (by way
 * of the DRM_H_TIMING() and DRM_V_TIMING() macros). Further checks and sets the
 * clock value being divisible by 1000 through macro CALC_DRM_CLOCK_K_HZ().
 *
 * Context: This macro asserts for no fractional refresh rates. Please double-check the
 *          resulting .clock member against a known target value, especially for lower
 *          framerates otherwise may cause compilation error!
 */
#define DRM_MODE_TIMING(REFRESH_FREQ, HDISPLAY, HFP, HSA, HBP, VDISPLAY, VFP, VSA, VBP) \
	.clock = CALC_DRM_CLOCK_K_HZ( \
		CALC_DRM_CLOCK_HZ(REFRESH_FREQ, HDISPLAY, HFP, HSA, HBP, VDISPLAY, VFP, VSA, VBP)), \
	DRM_H_TIMING(HDISPLAY, HFP, HSA, HBP), \
	DRM_V_TIMING(VDISPLAY, VFP, VSA, VBP)

/**
 * DRM_VRR_MODE_TIMING() - same as DRM_MODE_TIMING, but with VRR capabilities
 * @PEAK_REFRESH_FREQ: Peak physical image refresh frequency, in Hz
 * @TE_FREQ: TE signal frequency, in Hz
 * @HDISPLAY: Horizontal active region
 * @HFP: Horizontal front porch
 * @HSA: Horizontal sync
 * @HBP: Horizontal back porch
 * @VDISPLAY: Vertical active region
 * @VFP: Vertical front porch
 * @VSA: Vertical sync
 * @VBP: Vertical back porch
 *
 * This fills the same role as DRM_MODE_TIMING(), but also encodes the `vscan`
 * member to indicate the difference between (peak) refresh rate and TE frequency
 * inherent in VRR modes.
 * Notably, the `clock` member is also multiplied by the `vscan` result with this encoding.
 */
#define DRM_VRR_MODE_TIMING(PEAK_REFRESH_FREQ, TE_FREQ, HDISPLAY, HFP, HSA, HBP, \
			    VDISPLAY, VFP, VSA, VBP) \
	DRM_MODE_TIMING(TE_FREQ, HDISPLAY, HFP, HSA, HBP, VDISPLAY, VFP, VSA, VBP), \
	.vscan = (TE_FREQ) / (PEAK_REFRESH_FREQ)

/**
 * struct gs_display_dsc - Information about a mode's DSC parameters
 * @enabled: Whether DSC is enabled for this mode
 * @dsc_count: Number of encoders to be used by DPU (TODO:b/283964743)
 * @cfg: Configuration structure describing bulk of algorithm
 * @delay_reg_init_us: Hack for DPU delaying mode switch (TODO:b/283966795)
 *
 * Though most of the description of Display Stream Compression algorithms falls
 * within the bounds of the `struct drm_dsc_config`, this structure captures a
 * few other parameters surrounding the DSC configuration for a display mode
 * that we find useful to adjust (or refer to).
 */
struct gs_display_dsc {
	bool enabled;
	unsigned int dsc_count;

	const struct drm_dsc_config *cfg;

	unsigned int delay_reg_init_us;
};

/**
 * struct gs_display_underrun_param - Parameters to calculate underrun_lp_ref
 */
struct gs_display_underrun_param {
	/** @te_idle_us: te idle (us) to calculate underrun_lp_ref */
	unsigned int te_idle_us;
	/** @te_var: te variation (percentage) to calculate underrun_lp_ref */
	unsigned int te_var;
};

/**
 * struct gs_display_mode - gs display specific info
 */
struct gs_display_mode {
	/** @dsc: DSC parameters for the selected mode */
	struct gs_display_dsc dsc;

	/** @mode_flags: DSI mode flags from drm_mipi_dsi.h */
	unsigned long mode_flags;

	/** @min_bts_fps: minimal bts fps requirement */
	unsigned int min_bts_fps;

	/** @vblank_usec: parameter to calculate bts */
	unsigned int vblank_usec;

	/** @te_usec: command mode: TE pulse time */
	unsigned int te_usec;

	/** @bpc: display bits per component */
	unsigned int bpc;

	/** @underrun_param: parameters to calculate underrun_lp_ref when hs_clock changes */
	const struct gs_display_underrun_param *underrun_param;

	/** @is_lp_mode: boolean, if true it means this mode is a Low Power mode */
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

#endif // _GS_DISPLAY_MODE_H_
