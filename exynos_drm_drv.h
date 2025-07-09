/* SPDX-License-Identifier: GPL-2.0-only
 * exynos_drm_drv.h
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd.
 * Authors:
 *	Inki Dae <inki.dae@samsung.com>
 *	Joonyoung Shim <jy0922.shim@samsung.com>
 *	Seung-Woo Kim <sw0312.kim@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef _EXYNOS_DRM_DRV_H_
#define _EXYNOS_DRM_DRV_H_

#include <drm/drm_atomic.h>
#include <drm/drm_blend.h>
#include <drm/drm_crtc.h>
#include <drm/drm_plane.h>
#include <drm/drm_property.h>
#include <drm/drm_file.h>
#include <drm/samsung_drm.h>
#include <linux/kthread.h>
#include <linux/module.h>

#include <decon_cal.h>

#include "exynos_drm_connector.h"
#include "exynos_drm_dqe.h"
#include "cal_common/dqe_cal.h"

#define MAX_CRTC	3
#define MAX_PLANE	MAX_WIN_PER_DECON
#define MAX_FB_BUFFER	4

#define DEFAULT_WIN	0


#define to_exynos_crtc(x)	container_of(x, struct exynos_drm_crtc, base)
#define to_exynos_plane(x)	container_of(x, struct exynos_drm_plane, base)

/* this enumerates display type. */
enum exynos_drm_output_type {
	EXYNOS_DISPLAY_TYPE_NONE = 0,
	/* RGB or CPU Interface. */
	EXYNOS_DISPLAY_TYPE_DSI0 = BIT(0),
	EXYNOS_DISPLAY_TYPE_DSI1 = BIT(1),
	EXYNOS_DISPLAY_TYPE_DSI  = EXYNOS_DISPLAY_TYPE_DSI0 |
					EXYNOS_DISPLAY_TYPE_DSI1,
	/* HDMI Interface. */
	EXYNOS_DISPLAY_TYPE_HDMI = BIT(2),
	/* Virtual Display Interface. */
	EXYNOS_DISPLAY_TYPE_VIDI = BIT(3),

	/* DP Interface. */
	EXYNOS_DISPLAY_TYPE_DP0_SST1  = BIT(4),
	EXYNOS_DISPLAY_TYPE_DP0_SST2  = BIT(5),
	EXYNOS_DISPLAY_TYPE_DP0_SST3  = BIT(6),
	EXYNOS_DISPLAY_TYPE_DP0_SST4  = BIT(7),
	EXYNOS_DISPLAY_TYPE_DP1_SST1  = BIT(8),
	EXYNOS_DISPLAY_TYPE_DP1_SST2  = BIT(9),
	EXYNOS_DISPLAY_TYPE_DP1_SST3  = BIT(10),
	EXYNOS_DISPLAY_TYPE_DP1_SST4  = BIT(11),
	EXYNOS_DISPLAY_TYPE_DP0   = EXYNOS_DISPLAY_TYPE_DP0_SST1 |
					EXYNOS_DISPLAY_TYPE_DP0_SST2 |
					EXYNOS_DISPLAY_TYPE_DP0_SST3 |
					EXYNOS_DISPLAY_TYPE_DP0_SST4,
	EXYNOS_DISPLAY_TYPE_DP1   = EXYNOS_DISPLAY_TYPE_DP1_SST1 |
					EXYNOS_DISPLAY_TYPE_DP1_SST2 |
					EXYNOS_DISPLAY_TYPE_DP1_SST3 |
					EXYNOS_DISPLAY_TYPE_DP1_SST4,
};


enum exynos_drm_writeback_type {
	EXYNOS_WB_NONE,
	EXYNOS_WB_CWB,
	EXYNOS_WB_SWB,
};

struct exynos_drm_rect {
	unsigned int x, y;
	unsigned int w, h;
};

#if defined(CONFIG_SOC_ZUMA)
struct exynos_hdr_state {
	struct hdr_eotf_lut_v2p2 *eotf_lut;
	struct hdr_oetf_lut_v2p2 *oetf_lut;
	struct hdr_gm_data *gm;
	struct hdr_tm_data_v2p2 *tm;
};
#else
struct exynos_hdr_state {
	struct hdr_eotf_lut *eotf_lut;
	struct hdr_oetf_lut *oetf_lut;
	struct hdr_gm_data *gm;
	struct hdr_tm_data *tm;
};
#endif

/*
 * Exynos drm plane state structure.
 *
 * @base: plane_state object (contains drm_framebuffer pointer)
 * @old_fb: old drm_framebuffer pointer
 * @src: rectangle of the source image data to be displayed (clipped to
 *       visible part).
 * @crtc: rectangle of the target image position on hardware screen
 *       (clipped to visible part).
 * @h_ratio: horizontal scaling ratio, 16.16 fixed point
 * @v_ratio: vertical scaling ratio, 16.16 fixed point
 *
 * this structure consists plane state data that will be applied to hardware
 * specific overlay info.
 */

struct exynos_drm_plane_state {
	struct drm_plane_state base;
	struct drm_framebuffer *old_fb;
	uint32_t blob_id_restriction;
	uint32_t max_luminance;
	uint32_t min_luminance;
	uint32_t standard;
	uint32_t transfer;
	uint32_t range;
	uint32_t colormap;
	struct exynos_hdr_state hdr_state;
	struct drm_property_blob *eotf_lut;
	struct drm_property_blob *oetf_lut;
	struct drm_property_blob *gm;
	struct drm_property_blob *tm;
	struct drm_property_blob *block;
};

static inline struct exynos_drm_plane_state *
to_exynos_plane_state(const struct drm_plane_state *state)
{
	return container_of(state, struct exynos_drm_plane_state, base);
}

/*
 * Exynos drm common overlay structure.
 *
 * @base: plane object
 * @index: hardware index of the overlay layer
 *
 * this structure is common to exynos SoC and its contents would be copied
 * to hardware specific overlay info.
 */

struct exynos_drm_plane {
	struct drm_plane base;
	unsigned int index;
	struct dentry *debugfs_entry;

	struct {
		struct drm_property *restriction;
		struct drm_property *max_luminance;
		struct drm_property *min_luminance;
		struct drm_property *standard;
		struct drm_property *transfer;
		struct drm_property *range;
		struct drm_property *eotf_lut;
		struct drm_property *oetf_lut;
		struct drm_property *gm;
		struct drm_property *tm;
		struct drm_property *colormap;
		struct drm_property *block;
	} props;
};

#define EXYNOS_DRM_PLANE_CAP_DOUBLE	(1 << 0)
#define EXYNOS_DRM_PLANE_CAP_SCALE	(1 << 1)
#define EXYNOS_DRM_PLANE_CAP_ZPOS	(1 << 2)
#define EXYNOS_DRM_PLANE_CAP_TILE	(1 << 3)
#define EXYNOS_DRM_PLANE_CAP_AFBC	(1 << 4)

/*
 * Exynos DRM plane configuration structure.
 *
 * @zpos: initial z-position of the plane.
 * @type: type of the plane (primary, cursor or overlay).
 * @pixel_formats: supported pixel formats.
 * @num_pixel_formats: number of elements in 'pixel_formats'.
 * @capabilities: supported features (see EXYNOS_DRM_PLANE_CAP_*)
 */

struct exynos_drm_plane_config {
	unsigned int zpos;
	enum drm_plane_type type;
	const uint32_t *pixel_formats;
	unsigned int num_pixel_formats;
	unsigned int capabilities;
	const uint64_t *format_modifiers;
};

/*
 * Exynos drm crtc ops
 *
 * @enable: enable the device
 * @disable: disable the device
 * @enable_vblank: specific driver callback for enabling vblank interrupt.
 * @disable_vblank: specific driver callback for disabling vblank interrupt.
 * @mode_valid: specific driver callback for mode validation
 * @atomic_check: validate state
 * @atomic_begin: prepare device to receive an update
 * @atomic_flush: mark the end of device update
 * @update_plane: apply hardware specific overlay data to registers.
 * @disable_plane: disable hardware specific overlay.
 * @te_handler: trigger to transfer video image at the tearing effect
 *	synchronization signal if there is a page flip request.
 * @wait_for_flip_done: wait for active crtc flip to be done
 */
struct exynos_drm_crtc;
struct exynos_drm_crtc_ops {
	void (*enable)(struct exynos_drm_crtc *crtc, struct drm_crtc_state *old_state);
	void (*disable)(struct exynos_drm_crtc *crtc);
	int (*enable_vblank)(struct exynos_drm_crtc *crtc);
	void (*disable_vblank)(struct exynos_drm_crtc *crtc);
	u32 (*get_vblank_counter)(struct exynos_drm_crtc *crtc);
	enum drm_mode_status (*mode_valid)(struct exynos_drm_crtc *crtc,
		const struct drm_display_mode *mode);
	bool (*mode_fixup)(struct exynos_drm_crtc *crtc,
		const struct drm_display_mode *mode,
		struct drm_display_mode *adjusted_mode);
	void (*mode_set)(struct exynos_drm_crtc *crtc,
		const struct drm_display_mode *mode,
		const struct drm_display_mode *adjusted_mode);
	int (*atomic_check)(struct exynos_drm_crtc *crtc,
			    struct drm_crtc_state *state);
	void (*atomic_begin)(struct exynos_drm_crtc *crtc, struct drm_atomic_state *state);
	void (*update_plane)(struct exynos_drm_crtc *crtc,
			     struct exynos_drm_plane *plane);
	void (*disable_plane)(struct exynos_drm_crtc *crtc,
			      struct exynos_drm_plane *plane);
	void (*atomic_flush)(struct exynos_drm_crtc *crtc,
			struct drm_crtc_state *old_crtc_state);
	void (*te_handler)(struct exynos_drm_crtc *crtc);
	void (*wait_for_flip_done)(struct exynos_drm_crtc *crtc,
			const struct drm_crtc_state *old_crtc_state,
			const struct drm_crtc_state *new_crtc_state);
};

struct exynos_drm_crtc_state {
	struct drm_crtc_state base;
	uint32_t color_mode;
	uint32_t in_bpc;
	uint32_t force_bpc; /* crtc(DECON) bpc mode */
	uint64_t expected_present_time;
	struct exynos_dqe_state dqe;
	struct drm_property_blob *cgc_lut;
	struct drm_property_blob *disp_dither;
	struct drm_property_blob *cgc_dither;
	struct drm_property_blob *linear_matrix;
	struct drm_property_blob *gamma_matrix;
	/* legacy histogram properties */
	struct drm_property_blob *histogram_roi;
	struct drm_property_blob *histogram_weights;
	/* multi-channel support */
	struct drm_property_blob *histogram[HISTOGRAM_MAX];

	struct drm_gem_object *cgc_gem;
	enum exynos_drm_writeback_type wb_type;
	u8 seamless_mode_changed : 1;
	/**
	 * @bypass: when in this mode any DPU programming is bypassed but all
	 *          power/regulators are kept enabled
	 */
	u8 bypass : 1;

	/**
	 * @skip_update: if flag is set, most DPU updates should be skipped
	 *               and signaling of commit done should happen immediately
	 */
	u8 skip_update : 1;

	/**
	 * @planes_updated: this flag tracks whether planes were really changed, compared with
	 *                  crtc_state->planes_changed where it may be set if planes were added
	 *                  to atomic state due to any mode set (ex. self refresh change)
	 */
	u8 planes_updated : 1;

	/**
	 * @hibernation_exit: set when crtc is going out of hibernation, serves as
	 *		      potential optimization to avoid full updates
	 */
	u8 hibernation_exit : 1;

	unsigned int reserved_win_mask;
	unsigned int visible_win_mask;
	struct drm_rect partial_region;
	struct drm_property_blob *partial;
	bool needs_reconfigure;

	struct kthread_work commit_work;
};

static inline struct exynos_drm_crtc_state *
to_exynos_crtc_state(const struct drm_crtc_state *state)
{
	return container_of(state, struct exynos_drm_crtc_state, base);
}

/*
 * Exynos specific crtc structure.
 *
 * @base: crtc object.
 * @possible_type: what can connect connector types
 * @ops: pointer to callbacks for exynos drm specific functionality
 * @ctx: A pointer to the crtc's implementation specific context
 * @pipe_clk: A pointer to the crtc's pipeline clock.
 */
struct exynos_drm_crtc {
	struct drm_crtc			base;
	enum exynos_drm_output_type	possible_type;
	const struct exynos_drm_crtc_ops	*ops;
	void				*ctx;
	struct {
		struct drm_property *color_mode;
		struct drm_property *cgc_lut;
		struct drm_property *disp_dither;
		struct drm_property *cgc_dither;
		struct drm_property *force_bpc;
		struct drm_property *ppc;
		struct drm_property *max_disp_freq;
		struct drm_property *linear_matrix;
		struct drm_property *gamma_matrix;
		struct drm_property *dqe_enabled;
		struct drm_property *partial;
		struct drm_property *cgc_lut_fd;
		struct drm_property *expected_present_time;
		struct drm_property *rcd_plane_id;
		/* legacy histogram properties */
		struct drm_property *histogram_roi;
		struct drm_property *histogram_weights;
		struct drm_property *histogram_threshold;
		struct drm_property *histogram_pos;
		/* multichannel support */
		struct drm_property *histogram_channels;
		struct drm_property *histogram[HISTOGRAM_MAX];
	} props;
	u8 active_state;
	u32 rcd_plane_mask;
};

struct drm_exynos_file_private {
	u32 dummy;
};

struct histogram_event_node {
	struct drm_pending_event *base;
	struct list_head node;
};

struct exynos_drm_pending_histogram_event {
	struct drm_pending_event base;
	struct exynos_drm_histogram_channel_event event;
};

struct exynos_drm_pending_context_histogram_event {
	struct drm_pending_event base;
	struct exynos_drm_context_histogram_event event;
};

struct exynos_drm_priv_state {
	struct drm_private_state base;

	unsigned int available_win_mask;
};

static inline struct exynos_drm_priv_state *
to_exynos_priv_state(const struct drm_private_state *state)
{
	return container_of(state, struct exynos_drm_priv_state, base);
}

/*
 * Exynos drm private structure.
 *
 * @da_start: start address to device address space.
 *	with iommu, device address space starts from this address
 *	otherwise default one.
 * @da_space_size: size of device address space.
 *	if 0 then default value is used for it.
 * @pending: the crtcs that have pending updates to finish
 * @lock: protect access to @pending
 * @wait: wait an atomic commit to finish
 */
struct exynos_drm_private {
	struct drm_device drm;
	struct drm_atomic_state *suspend_state;
	struct device *iommu_client;
	void *mapping;
	bool tui_enabled;
	struct mutex dp_tui_lock;
	u32 secured_dpp_mask;

	/* for atomic commit */
	u32			pending;
	spinlock_t		lock;
	wait_queue_head_t	wait;

	struct exynos_drm_connector_properties connector_props;
	struct drm_private_obj	obj;
};

#define drm_to_exynos_dev(dev) container_of(dev, struct exynos_drm_private, drm)

#ifdef CONFIG_DRM_EXYNOS_DPI
struct drm_encoder *exynos_dpi_probe(struct device *dev);
int exynos_dpi_remove(struct drm_encoder *encoder);
int exynos_dpi_bind(struct drm_device *dev, struct drm_encoder *encoder);
#else
static inline struct drm_encoder *
exynos_dpi_probe(struct device *dev) { return NULL; }
static inline int exynos_dpi_remove(struct drm_encoder *encoder)
{
	return 0;
}
static inline int exynos_dpi_bind(struct drm_device *dev,
				  struct drm_encoder *encoder)
{
	return 0;
}
#endif

int exynos_atomic_commit(struct drm_device *dev, struct drm_atomic_state *state,
			 bool nonblock);
int exynos_atomic_check(struct drm_device *dev, struct drm_atomic_state *state);
int exynos_atomic_enter_tui(void);
int exynos_atomic_exit_tui(void);

enum lhbm_hist_weight_grp { LHBM_CIRCLE_WEIGHT = 0, LHBM_FSCREEN_WEIGHT, LHBM_WEIGHT_MAX };

static struct histogram_weights lhbm_hist_weight[LHBM_WEIGHT_MAX] = { { 341, 341, 341 },
								      { 218, 732, 74 } };

int exynos_drm_drv_set_lhbm_hist(struct exynos_drm_connector *conn, int x, int y, int w, int h);
/**
 * exynos_drm_drv_set_lhbm_hist_gs() - sets decon lhbm hist ROI
 * @decon: decon device whose ROI is changing
 * @roi: histogram ROI
 * @weight: histogram weight
 */
int exynos_drm_drv_set_lhbm_hist_gs(struct decon_device *decon, struct histogram_roi *roi,
				    struct histogram_weights *weight);
int exynos_drm_drv_get_lhbm_gray_level(struct exynos_drm_connector *conn);

extern struct platform_driver decon_driver;
extern struct platform_driver dsim_driver;
extern struct platform_driver dp_driver;
extern struct platform_driver dpp_driver;
extern struct platform_driver writeback_driver;
extern struct platform_driver tui_driver;
#endif
