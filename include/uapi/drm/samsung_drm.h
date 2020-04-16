/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
#ifndef __SAMSUNG_DRM_H__
#define __SAMSUNG_DRM_H__

#if defined(__cplusplus)
extern "C" {
#endif

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
