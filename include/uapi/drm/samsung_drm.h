/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
#ifndef __SAMSUNG_DRM_H__
#define __SAMSUNG_DRM_H__

#if defined(__linux__)
#include <linux/types.h>
#endif

#if defined(__cplusplus)
extern "C" {
#endif

#define DRM_SAMSUNG_HDR_EOTF_LUT_LEN	129

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

#define DRM_SAMSUNG_HDR_OETF_LUT_LEN	33

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

#if defined(__cplusplus)
}
#endif

#endif /* __SAMSUNG_DRM_H__ */
