/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Register definition file for Samsung HDR
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __HDR_REGS_H__
#define __HDR_REGS_H__

/*
 *-------------------------------------------------------------------
 * HDR(L0~L5) SFR list
 * L0,L2,L4 : LSI_L_HDR
 * L1,L3,L5 : LSI_H_HDR - dynamic meta
 *
 * base address : 0x1C0E_0000
 * < Layer.offset >
 *  L0      L1      L2      L3      L4      L5
 *  0x0000  0x1000  0x2000  0x3000  0x4000  0x5000
 *
 * HDR_LSI_L_ : common for HDR High & Low
 *-------------------------------------------------------------------
 */

#define HDR_SHD_OFFSET			(0x0800)

#define HDR_LSI_VERSION			(0x0000)
#define HDR_VERSION			(0x01010000)
#define HDR_VERSION_GET(_v)		(((_v) >> 0) & 0xFFFFFFFF)

#define HDR_HDR_CON			(0x0000)
#define OETF_EN(_v)			((_v) << 28)
#define OETF_EN_MASK			(1 << 28)
#define TM_EN(_v)			((_v) << 24)
#define TM_EN_MASK			(1 << 24)
#define GM_EN(_v)			((_v) << 20)
#define GM_EN_MASK			(1 << 20)
#define EOTF_LUT_EN(_v)			((_v) << 17)
#define EOTF_LUT_EN_MASK		(1 << 17)
#define EOTF_EN(_v)			((_v) << 16)
#define EOTF_EN_MASK			(1 << 16)
#define FP16_CVT_EN(_v)			((_v) << 5)
#define FP16_CVT_EN_MASK		(1 << 5)
#define FP16_EN(_v)			((_v) << 4)
#define FP16_EN_MASK			(1 << 4)
#define HDR_EN(_v)			((_v) << 0)
#define HDR_EN_MASK			(1 << 0)

#define HDR_EOTF_SCALER			(0x0004)
#define EOTF_SCALER(_v)			((_v) << 0)
#define EOTF_SCALER_MASK		(0xffff < 0)

/*-----[ OETF : Inverse EOTF ]-------------------------------------------------
 */
#define HDR_OETF_LUT_REG_CNT		(24)

#define HDR_OETF_LUT_TS(_n)		(0x01BC + ((_n) * 0x4))
#define OETF_TS_ODD(_v)			((_v) << 16)
#define OETF_TS_ODD_MASK		(0x3FFF << 16)
#define OETF_TS_EVEN(_v)		((_v) << 0)
#define OETF_TS_EVEN_MASK		(0x3FFF << 0)

#define HDR_OETF_LUT_VS(_n)		(0x021C + ((_n) * 0x4))
#define OETF_VS_ODD(_v)			((_v) << 16)
#define OETF_VS_ODD_MASK		(0x3FF << 16)
#define OETF_VS_EVEN(_v)		((_v) << 0)
#define OETF_VS_EVEN_MASK		(0x3FF << 0)

/*-----[ EOTF : Electro-Optical Transfer Function ]----------------------------
 */
#define HDR_EOTF_LUT_REG_CNT		(20)

#define HDR_EOTF_LUT_TS(_n)		(0x0010 + ((_n) * 0x4))
#define EOTF_TS_ODD(_v)			((_v) << 16)
#define EOTF_TS_ODD_MASK		(0x3FF << 16)
#define EOTF_TS_EVEN(_v)		((_v) << 0)
#define EOTF_TS_EVEN_MASK		(0x3FF << 0)

#define HDR_EOTF_LUT_VS(_n)		(0x0060 + ((_n) * 0x4))
#define EOTF_VS_ODD(_v)			((_v) << 16)
#define EOTF_VS_ODD_MASK		(0xFFFF << 16)
#define EOTF_VS_EVEN(_v)		((_v) << 0)
#define EOTF_VS_EVEN_MASK		(0xFFFF << 0)

/*-----[ GM : Gamut Mapping ]--------------------------------------------------
 * 3x3 matrix, S2.16
 * S16.0 offset to the result of matrix calculation
 *-----------------------------------------------------------------------------
 */
#define HDR_GM_COEF_REG_CNT		(9)
/*
 * GM_COEFF (0~8) : 0x039C ~ 0x03BC
 * |Rconv| = |C00 C01 C02| |Rin| + |Roffset0|
 * |Gconv| = |C10 C11 C12| |Gin| + |Goffset1|
 * |Bconv| = |C20 C21 C22| |Bin| + |Boffset2|
 */
#define HDR_GM_COEFF(_n)		(0x00B0 + ((_n) * 0x4))
#define GM_COEF(_v)			((_v) << 0)
#define GM_COEF_MASK			(0xFFFF << 0)

#define HDR_GM_OFFS_REG_CNT		(3)
/*
 * GM_OFFS (0~2) : 0x03C0 ~ 0x03C8
 */
#define HDR_GM_OFF(_n)			(0x00D4 + ((_n) * 0x4))
#define HDR_GM_OFF_0			(0x00D4)
#define HDR_GM_OFF_1			(0x00D8)
#define HDR_GM_OFF_2			(0x00DC)
#define GM_OFFS(_v)			((_v) << 0)
#define GM_OFFS_MASK			(0x3FFF << 0)

/*-----[ TM : Tone Mapping ]---------------------------------------------------
 * Available only in high-end IP
 * 32-segment transfer function, 16-bit >> S10.16
 * Index calculation using MaxRGB and Y (luminance) in parallel
 * Supports for both fixed-ratio mixing and adaptive mixing
 *-----------------------------------------------------------------------------
 */
#define HDR_TM_COEFF00			(0x00E0)
#define TCOEFF00(_v)			((_v) << 0)
#define TCOEFF00_MASK			(0xFFF << 0)

#define HDR_TM_COEFF01			(0x00E4)
#define TCOEFF01(_v)			((_v) << 0)
#define TCOEFF01_MASK			(0xFFF << 0)

#define HDR_TM_COEFF02			(0x00E8)
#define TCOEFF02(_v)			((_v) << 0)
#define TCOEFF02_MASK			(0xFFF << 0)

#define HDR_TM_YMIX_TF			(0x00EC)
#define YMIX_TF(_v)			((_v) << 0)
#define YMIX_TF_MASK			(0x1FFF << 0)

#define HDR_TM_YMIX_VF			(0x00F0)
#define YMIX_VF(_v)			((_v) << 0)
#define YMIX_VF_MASK			(0x1FFF << 0)

#define HDR_TM_YMIX_SLOPE		(0x00F4)
#define YMIX_SLOPE(_v)			((_v) << 0)
#define YMIX_SLOPE_MASK			(0x3FFF << 0)

#define HDR_TM_YMIX_DV			(0x00F8)
#define YMIX_DV(_v)			((_v) << 0)
#define YMIX_DV_MASK			(0x1FFF << 0)

#define HDR_TM_LUT_REG_CNT		(24)

#define HDR_TM_LUT_TS(_n)		(0x00FC + ((_n) * 0x4))
#define TM_TS_ODD(_v)			((_v) << 16)
#define TM_TS_ODD_MASK			(0x1FFF << 16)
#define TM_TS_EVEN(_v)			((_v) << 0)
#define TM_TS_EVEN_MASK			(0x1FFF << 0)

#define HDR_TM_LUT_VS(_n)		(0x015C + ((_n) * 0x4))
#define TM_VS_ODD(_v)			((_v) << 16)
#define TM_VS_ODD_MASK			(0x1FFF << 16)
#define TM_VS_EVEN(_v)			((_v) << 0)
#define TM_VS_EVEN_MASK			(0x1FFF << 0)

#endif /* __HDR_REGS_H__ */
