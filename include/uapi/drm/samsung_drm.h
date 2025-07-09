/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
#ifndef __SAMSUNG_DRM_H__
#define __SAMSUNG_DRM_H__

#if defined(__linux__)
#include <linux/types.h>
#endif

#include <drm/drm.h>
#include <drm/drm_fourcc_gs101.h>

#if defined(__cplusplus)
extern "C" {
#endif

#define DRM_SAMSUNG_HDR_EOTF_LUT_LEN	129
#define DRM_SAMSUNG_HDR_EOTF_V2P2_LUT_LEN	20

/**
 * struct hdr_eotf_lut - HDR EOTF look up table to set by user-space
 *
 * @posx: x coordinate of the boundaries between segments in EOTF
 * @posy: y coordinate of the boundaries between segments in EOTF
 *
 * A hdr_eotf_lut represents a look up table of EOTF(Electro-Optical Transfer
 * Function). It is used for eotf_lut blob property of a plane object if
 * a plane supports HDR10 feature.
 */
struct hdr_eotf_lut {
	__u16 posx[DRM_SAMSUNG_HDR_EOTF_LUT_LEN];
	__u32 posy[DRM_SAMSUNG_HDR_EOTF_LUT_LEN];
};

struct hdr_v2p2_element {
	__u16 even;
	__u16 odd;
};

struct hdr_eotf_lut_v2p2 {
	struct hdr_v2p2_element ts[DRM_SAMSUNG_HDR_EOTF_V2P2_LUT_LEN];
	struct hdr_v2p2_element vs[DRM_SAMSUNG_HDR_EOTF_V2P2_LUT_LEN];
	__u16 scaler;
	bool lut_en;
};

#define DRM_SAMSUNG_HDR_OETF_LUT_LEN	33
#define DRM_SAMSUNG_HDR_OETF_V2P2_LUT_LEN	24

/**
 * struct hdr_oetf_lut - HDR OETF look up table to set by user-space
 *
 * @posx: x coordinate of the boundaries between segments in OETF
 * @posy: y coordinate of the boundaries between segments in OETF
 *
 * A hdr_oetf_lut represents a look up table of OETF(Optical-Electro Transfer
 * Function). It is used for oetf_lut blob property of a plane object if
 * a plane supports HDR10 feature.
 */
struct hdr_oetf_lut {
	__u16 posx[DRM_SAMSUNG_HDR_OETF_LUT_LEN];
	__u16 posy[DRM_SAMSUNG_HDR_OETF_LUT_LEN];
};

struct hdr_oetf_lut_v2p2 {
	struct hdr_v2p2_element ts[DRM_SAMSUNG_HDR_OETF_V2P2_LUT_LEN];
	struct hdr_v2p2_element vs[DRM_SAMSUNG_HDR_OETF_V2P2_LUT_LEN];
};

#define DRM_SAMSUNG_HDR_GM_DIMENS	3

/**
 * struct hdr_gm_data - HDR gammut matrix data to set by user-space
 *
 * @coeffs: coefficients of a gammut matrix
 * @offsets: offsets of a gammut matrix
 *
 * A hdr_gm_data represents coefficients and offsets of a gammut matrix.
 * It is used to set a plane property for calculating a gammut matrix
 * if a plane supports HDR10 feature.
 */
struct hdr_gm_data {
	__u32 coeffs[DRM_SAMSUNG_HDR_GM_DIMENS * DRM_SAMSUNG_HDR_GM_DIMENS];
	__u32 offsets[DRM_SAMSUNG_HDR_GM_DIMENS];
};

#define DRM_SAMSUNG_HDR_TM_LUT_LEN		33
#define DRM_SAMSUNG_HDR_TM_V2P2_LUT_LEN		24

/**
 * struct hdr_tm_data - HDR tone mapping data and look up table to set
 *                      by user-space.
 *
 * @coeff_r: coefficient to be multiplied with R to convert RGB to Y
 * @coeff_g: coefficient to be multiplied with G to convert RGB to Y
 * @coeff_b: coefficient to be multiplied with B to convert RGB to Y
 * @rng_x_min: left boundary of the decreasing range of the ratio function
 *             for adaptive mixing
 * @rng_x_max: right boundary of the decreasing range of the ratio function
 *             for adaptive mixing
 * @rng_y_min: minimum ratio for adaptive mixing.
 * @rng_y_max: maximum ratio for adaptive mixing.
 * @posx: x coordinate of the boundaries between segments in tone mapping
 *        gain function.
 * @posy: y coordinate of the boundaries between segments in tone mapping
 *        gain function.
 *
 * A hdr_tm_data represents tone mapping data. It is used to set a plane
 * property for calculating tone mapping if a plane supports HDR10+ feature.
 */
struct hdr_tm_data {
	__u16 coeff_r;
	__u16 coeff_g;
	__u16 coeff_b;
	__u16 rng_x_min;
	__u16 rng_x_max;
	__u16 rng_y_min;
	__u16 rng_y_max;
	__u16 posx[DRM_SAMSUNG_HDR_TM_LUT_LEN];
	__u32 posy[DRM_SAMSUNG_HDR_TM_LUT_LEN];
};

struct hdr_tm_data_v2p2 {
	__u16 coeff_00;
	__u16 coeff_01;
	__u16 coeff_02;
	__u16 ymix_tf;
	__u16 ymix_vf;
	__u16 ymix_slope;
	__u16 ymix_dv;
	struct hdr_v2p2_element ts[DRM_SAMSUNG_HDR_TM_V2P2_LUT_LEN];
	struct hdr_v2p2_element vs[DRM_SAMSUNG_HDR_TM_V2P2_LUT_LEN];
};

#define DRM_SAMSUNG_CGC_LUT_REG_CNT	2457

/**
 * struct cgc_lut - color gammut control look up table to set by user-space
 *
 * @r_values: values for red color
 * @g_values: values for green color
 * @b_values: values for blue color
 *
 * A cgc_lut represents a look up table of color gammut control. It is used
 * for cgc_lut blob property of a crtc object if a crtc support color gammut
 * control.
 */
struct cgc_lut {
	__u32 r_values[DRM_SAMSUNG_CGC_LUT_REG_CNT];
	__u32 g_values[DRM_SAMSUNG_CGC_LUT_REG_CNT];
	__u32 b_values[DRM_SAMSUNG_CGC_LUT_REG_CNT];
};

#define DRM_SAMSUNG_CGC_DMA_LUT_ENTRY_CNT 4913
/**
 * struct cgc_dma_lut - color gammut control format for cgc dma to set by user-space
 *
 * @r_value: value for red color
 * @g_value: value for green color
 * @b_value: value for blue color
 *
 * A cgc_dma_lut represents a format to support cgc dma. cgc coefficients should be
 * located in dram according to this format.
 */
struct cgc_dma_lut {
	__u16 r_value;
	__u16 g_value;
	__u16 b_value;
};

#define DRM_SAMSUNG_MATRIX_DIMENS	3

/**
 * struct exynos_matrix - a matrix data to set by user-space
 *
 * @coeffs: coefficients of a matrix
 * @offsets: offsets of a matrix
 *
 * A exynos_matrix represents coefficients and offsets of a matrix.
 * It is used to set a property for calculating a matrix.
 */
struct exynos_matrix {
	__u16 coeffs[DRM_SAMSUNG_MATRIX_DIMENS * DRM_SAMSUNG_MATRIX_DIMENS];
	__u16 offsets[DRM_SAMSUNG_MATRIX_DIMENS];
};

struct dpp_size_range {
	__u32 min;
	__u32 max;
	__u32 align;
};

struct dpp_restriction {
	struct dpp_size_range src_f_w;
	struct dpp_size_range src_f_h;
	struct dpp_size_range src_w;
	struct dpp_size_range src_h;
	__u32 src_x_align;
	__u32 src_y_align;

	struct dpp_size_range dst_f_w;
	struct dpp_size_range dst_f_h;
	struct dpp_size_range dst_w;
	struct dpp_size_range dst_h;
	__u32 dst_x_align;
	__u32 dst_y_align;

	struct dpp_size_range blk_w;
	struct dpp_size_range blk_h;
	__u32 blk_x_align;
	__u32 blk_y_align;

	__u32 src_h_rot_max; /* limit of source img height in case of rotation */

	__u32 scale_down;
	__u32 scale_up;
};

struct dpp_ch_restriction {
	__s32 id;
	__u64 attr;
	struct dpp_restriction restriction;
};

/**
 * struct dither_config - a dither configuration data to set by user-space
 *
 * @en: enable or disable a dither
 * @mode: 0 for dither, 1 for 2bits shift
 * @frame_con: If frame control is on, the temporal dither is used. Otherwise,
 *	       the spatial dither is used. If temporal dither is used, a
 *	       different dither mask can be used for each frame counter.
 * @frame_offset: The dithers in DQE use same frame counter. However, if two
 *		  dithers set a different frame offsets, each dither can select
 *		  a different dither mask according to the different frame
 *		  counter + offset.
 * @table_sel_r/g/b: It can select a different dither mask for each channel.
 *
 * A dither_config represents the information necessary for setting up the
 * dither in DQE. It is used to set a property of a crtc for dither
 * configuration.
 */
struct dither_config {
	__u8 en:1;
	__u8 mode:1;
	__u8 frame_con:1;
	__u8 frame_offset:2;
	__u8 table_sel_r:1;
	__u8 table_sel_g:1;
	__u8 table_sel_b:1;
	__u32 reserved:24;
};

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
 *		This percentage must comply with display configuration
 *		file.
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

/**
 * struct tui_hw_buffer - buffer allocation query by user-space
 *
 * @fb_physical: the physical address of the buffer allocated
 * @fb_size: the size of the buffer allocated
 *
 * The structure is used to return the parameter of the allocated buffer.
 */
struct tui_hw_buffer {
	__u64 fb_physical;
	__u64 fb_size;
} __packed;

#define EXYNOS_START_TUI		0x10
#define EXYNOS_FINISH_TUI		0x11
#define EXYNOS_TUI_REQUEST_BUFFER	0x20
#define EXYNOS_TUI_RELEASE_BUFFER	0x21

/**
 * struct histogram_roi - region of interest for histogram to set by user-space
 *
 * @start_x: upper left x position of ROI
 * @start_y: upper left y position of ROI
 * @hsize: horizontal size of image
 * @vsize: vertical  size of image
 *
 * A histogram_roi sets region of interest on image for gathering histogram
 * data. It is used to set a property of a crtc.
 */
struct histogram_roi {
	__u16 start_x;
	__u16 start_y;
	__u16 hsize;
	__u16 vsize;
};

/**
 * struct histogram_weights - weight for each color component to set by user-space
 *
 * @weight_r: histogram weight for red
 * @weight_g: histogram weight for green
 * @weight_b: histogram weight for blue
 *
 * A histogram_weights sets a weight of each color component for calculating
 * histogram data. It is used to set a property of a crtc.
 */
struct histogram_weights {
	__u16 weight_r;
	__u16 weight_g;
	__u16 weight_b;
};

#define HISTOGRAM_BIN_COUNT	256
struct histogram_bins {
	__u16 data[HISTOGRAM_BIN_COUNT];
};

/**
 * enum histogram_prog_pos - defines programmable positions
 *
 * For example, histogram position and writeback path could be
 * programmable. This enum defines positions for it.
 */
enum histogram_prog_pos {
	POST_DQE,
	PRE_DQE,
};

/**
 * enum histogram_flags - defines control masks for optional features
 */
enum histogram_flags {
	HISTOGRAM_FLAGS_BLOCKED_ROI	= 0x20, /* blocked roi configuration is valid */
};

/**
 * struct histogram_channel_config - histogram channel configuration
 *
 * @roi: histogram roi
 * @weights: histogram weights
 * @threshold: histogram threshold
 * @pos: histogram position (before or after DQE)
 * @blocked_roi: histogram blocked roi (ignored if h/w does not support it)
 * @flags: histogram optional configuration flags
 *
 * It is used to set a property of a crtc.
 */
struct histogram_channel_config {
	struct histogram_roi		roi;
	struct histogram_weights	weights;
	enum histogram_prog_pos		pos;
	__u32				threshold;
	struct histogram_roi		blocked_roi;
	__u32				flags;
};

#define EXYNOS_DRM_HISTOGRAM_EVENT		0x80000000
#define EXYNOS_DRM_HISTOGRAM_CHANNEL_EVENT	0x80000001
#define EXYNOS_DRM_CONTEXT_HISTOGRAM_EVENT	0x80000002

/**
 * struct exynos_drm_histogram_event - histogram event to wait for user-space
 *
 * @base: event header which informs user space event type and length.
 * @bins: histogram bin data to be sent to user space through using read()
 * @crtcid: crtc id
 *
 * User space waits for POLLIN event using like poll() or select(). If event
 * type is EXYNOS_DRM_HISTOGRAM_EVENT, user space can try to read histogram
 * bin data through "bins".
 */
struct exynos_drm_histogram_event {
	struct drm_event base;
	struct histogram_bins bins;
	__u32 crtc_id;
};

/**
 * struct exynos_drm_histogram_channel_event - histogram channel event to wait for user-space
 *
 * @base: event header which informs user space event type and length.
 * @bins: histogram bin data to be sent to user space through using read()
 * @crtcid: crtc id
 * @histid: histogram channel id
 *
 * User space waits for POLLIN event using like poll() or select(). If event
 * type is EXYNOS_DRM_HISTOGRAM_CHANNEL_EVENT, user space can try to read histogram
 * bin data through "bins".
 */
struct exynos_drm_histogram_channel_event {
	struct drm_event base;
	struct histogram_bins bins;
	__u16 crtc_id;
	__u16 hist_id;
};

/**
 * struct exynos_drm_context_histogram_event - context histogram event to wait for user-space
 *
 * @base: event header which informs user space event type and length.
 * @bins: histogram bin data to be sent to user space through using read()
 * @crtc_id: crtc id
 * @user_handle: user-space histogram context (ex. blob id of config)
 *
 * User space waits for POLLIN event using like poll() or select(). If event
 * type is EXYNOS_DRM_CONTEXT_HISTOGRAM_EVENT, user space can try to read histogram
 * bin data through "bins".
 */
struct exynos_drm_context_histogram_event {
	struct drm_event base;
	struct histogram_bins bins;
	__u32 crtc_id;
	__u32 user_handle;
};

#define EXYNOS_HISTOGRAM_REQUEST		0x0
#define EXYNOS_HISTOGRAM_CANCEL			0x1
#define EXYNOS_HISTOGRAM_CHANNEL_REQUEST	0x20
#define EXYNOS_HISTOGRAM_CHANNEL_CANCEL		0x21
#define EXYNOS_HISTOGRAM_CHANNEL_DATA_REQUEST	0x30 /* histogram data is returned via ioctl */
#define EXYNOS_CONTEXT_HISTOGRAM_EVENT_REQUEST	0x40
#define EXYNOS_CONTEXT_HISTOGRAM_EVENT_CANCEL	0x41

/**
 * struct exynos_drm_histogram_channel_request - histogram channel query control structure
 *
 * @crtcid: crtc id
 * @histid: histogram channel id
 *
 * User space sends an IOCTL
 *   DRM_EXYNOS_HISTOGRAM_CHANNEL_REQUEST
 *   DRM_EXYNOS_HISTOGRAM_CHANNEL_CANCEL
 * with struct exynos_drm_histogram_channel_request data type.
 */
struct exynos_drm_histogram_channel_request {
	__u32 crtc_id; /* crtc id */
	__u32 hist_id; /* histogram channel id */
};

/**
 * struct exynos_histogram_channel_request - histogram channel request
 *
 * @crtcid: in: crtc id
 * @histid: in : histogram channel id
 * @bins: out: histogram bin data to be sent to user space through using read()
 *
 * User space sends an IOCTL
 *   EXYNOS_HISTOGRAM_CHANNEL_SYNC_REQUEST
 * with struct exynos_drm_histogram_channel_data_request data type
 */
struct exynos_drm_histogram_channel_data_request {
	__u16 crtc_id; /* in: crtc id */
	__u16 hist_id; /* in: histogram channel id */
	struct histogram_bins __user *bins; /* out: histogram data */
};

/**
 * struct exynos_drm_context_histogram_arg - histogram query control structure
 *
 * @crtc_id: crtc id
 * @user_handle: user-space histogram context (ex. blob id of config)
 * @flags: histogram query optional flags
 *
 * User space sends an IOCTL
 *   EXYNOS_CONTEXT_HISTOGRAM_EVENT_REQUEST
 *   EXYNOS_CONTEXT_HISTOGRAM_EVENT_CANCEL
 * with struct exynos_drm_context_histogram_arg data type.
 */
struct exynos_drm_context_histogram_arg {
	__u32 crtc_id; /* crtc id */
	__u32 user_handle;
	__u32 flags;
};

#define DRM_IOCTL_EXYNOS_HISTOGRAM_REQUEST \
	DRM_IOW(DRM_COMMAND_BASE + EXYNOS_HISTOGRAM_REQUEST, __u32)
#define DRM_IOCTL_EXYNOS_HISTOGRAM_CANCEL \
	DRM_IOW(DRM_COMMAND_BASE + EXYNOS_HISTOGRAM_CANCEL, __u32)
#define DRM_IOCTL_EXYNOS_HISTOGRAM_CHANNEL_REQUEST \
	DRM_IOW(DRM_COMMAND_BASE + EXYNOS_HISTOGRAM_CHANNEL_REQUEST, \
		struct exynos_drm_histogram_channel_request)
#define DRM_IOCTL_EXYNOS_HISTOGRAM_CHANNEL_CANCEL \
	DRM_IOW(DRM_COMMAND_BASE + EXYNOS_HISTOGRAM_CHANNEL_CANCEL, \
		struct exynos_drm_histogram_channel_request)
#define DRM_IOCTL_EXYNOS_HISTOGRAM_CHANNEL_DATA_REQUEST \
	DRM_IOW(DRM_COMMAND_BASE + EXYNOS_HISTOGRAM_CHANNEL_DATA_REQUEST, \
		struct exynos_drm_histogram_channel_data_request)
#define DRM_IOCTL_EXYNOS_CONTEXT_HISTOGRAM_EVENT_REQUEST  \
	DRM_IOW(DRM_COMMAND_BASE + EXYNOS_CONTEXT_HISTOGRAM_EVENT_REQUEST, \
		struct exynos_drm_context_histogram_arg)
#define DRM_IOCTL_EXYNOS_CONTEXT_HISTOGRAM_EVENT_CANCEL  \
	DRM_IOW(DRM_COMMAND_BASE + EXYNOS_CONTEXT_HISTOGRAM_EVENT_CANCEL, \
		struct exynos_drm_context_histogram_arg)

#if defined(__cplusplus)
}
#endif

#endif /* __SAMSUNG_DRM_H__ */
