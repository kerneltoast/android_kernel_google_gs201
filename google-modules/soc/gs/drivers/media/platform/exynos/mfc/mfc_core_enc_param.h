/*
 * drivers/media/platform/exynos/mfc/mfc_enc_param.h
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __MFC_ENC_PARAM_H
#define __MFC_ENC_PARAM_H __FILE__

#include "mfc_common.h"

static int mfc_colorspace_to_rgb_format_ctrl[][2] = {
	{ MFC_COLORSPACE_UNSPECIFICED,	1}, /* Unknown */
	{ MFC_COLORSPACE_BT601,		0}, /* Rec. ITU-R BT.601-7 */
	{ MFC_COLORSPACE_BT709,		1}, /* Rec. ITU-R BT.709-6 */
	{ MFC_COLORSPACE_SMPTE_170,	0}, /* SMPTE-170 */
	{ MFC_COLORSPACE_SMPTE_240,	0}, /* SMPTE-240 */
	{ MFC_COLORSPACE_BT2020,	1}, /* Rec. ITU-R BT.2020-2 */
	{ MFC_COLORSPACE_RESERVED,	1}, /* Reserved */
	{ MFC_COLORSPACE_SRGB,		1}, /* sRGB (IEC 61966-2-1) */
	{ MFC_COLORSPACE_UNSPECIFICED,	1}, /* Unknown */
	{ MFC_COLORSPACE_UNSPECIFICED,	1}, /* Unknown */
	{ MFC_COLORSPACE_UNSPECIFICED,	1}, /* Unknown */
};

static int mfc_transfer_to_rgb_format_ctrl[][2] = {
	{ MFC_TRANSFER_RESERVED,	1},
	{ MFC_TRANSFER_BT709,		1},
	{ MFC_TRANSFER_UNSPECIFIED,	1},
	{ MFC_TRANSFER_RESERVED,	1},
	{ MFC_TRANSFER_GAMMA_22,	1},
	{ MFC_TRANSFER_GAMMA_28,	1},
	{ MFC_TRANSFER_SMPTE_170M,	0},
	{ MFC_TRANSFER_SMPTE_240M,	1},
	{ MFC_TRANSFER_LINEAR,		1},
	{ MFC_TRANSFER_LOGARITHMIC,	1},
	{ MFC_TRANSFER_LOGARITHMIC_S,	1},
	{ MFC_TRANSFER_XvYCC,		1},
	{ MFC_TRANSFER_BT1361,		1},
	{ MFC_TRANSFER_SRGB,		1},
	{ MFC_TRANSFER_BT2020_1,	1},
	{ MFC_TRANSFER_BT2020_2,	1},
	{ MFC_TRANSFER_ST2084,		1},
	{ MFC_TRANSFER_ST428,		1},
	{ MFC_TRANSFER_HLG,		1},
};

void mfc_core_set_min_bit_count(struct mfc_core *core, struct mfc_ctx *ctx);
void mfc_core_set_slice_mode(struct mfc_core *core, struct mfc_ctx *ctx);
void mfc_core_set_aso_slice_order_h264(struct mfc_core *core,
				struct mfc_ctx *ctx);
void mfc_core_set_enc_config_qp(struct mfc_core *core, struct mfc_ctx *ctx);
void mfc_core_set_enc_ts_delta(struct mfc_core *core, struct mfc_ctx *ctx);
int mfc_core_set_enc_params(struct mfc_core *core, struct mfc_ctx *ctx);
void mfc_core_set_test_params(struct mfc_core *core);
int mfc_core_get_enc_bframe(struct mfc_ctx *ctx);

#endif /* __MFC_ENC_PARAM_H */
