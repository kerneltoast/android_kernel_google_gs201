/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Register definition file for Samsung Display Quality Enhancer
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __DQE_REGS_H__
#define __DQE_REGS_H__

/*
 * <bpp>->[SecureROI]->[ATC]->[HSC(MemoryColor)]->[GammaMatrix]->[De-Gamma]->
 * <bpp+2> [LinearMatrix]->[CGC-3D_LUT(MemoryColor)]->[Re-Gamma]->[CGC Dither]
 * <bpp> ->[Display Dither]->Out/[Histogram]
 */

#define DQE_TOP_CORE_SECURITY		(0x0000)
#define DQE_CORE_SECURITY		(0x1 << 0)

#define DQE_TOP_IMG_SIZE		(0x0004)
#define DQE_IMG_VSIZE(_v)		((_v) << 16)
#define DQE_IMG_VSIZE_MASK		(0x3FFF << 16)
#define DQE_IMG_HSIZE(_v)		((_v) << 0)
#define DQE_IMG_HSIZE_MASK		(0x3FFF << 0)

#define DQE_TOP_FRM_SIZE		(0x0008)
#define DQE_FULL_IMG_VSIZE(_v)		((_v) << 16)
#define DQE_FULL_IMG_VSIZE_MASK		(0x3FFF << 16)
#define DQE_FULL_IMG_HSIZE(_v)		((_v) << 0)
#define DQE_FULL_IMG_HSIZE_MASK		(0x3FFF << 0)

#define DQE_TOP_FRM_PXL_NUM		(0x000C)
#define DQE_FULL_PXL_NUM(_v)		((_v) << 0)
#define DQE_FULL_PXL_NUM_MASK		(0xFFFFFFF << 0)

#define DQE_TOP_PARTIAL_START		(0x0010)
#define DQE_PARTIAL_START_Y(_v)		((_v) << 16)
#define DQE_PARTIAL_START_Y_MASK	(0x3FFF << 16)
#define DQE_PARTIAL_START_X(_v)		((_v) << 0)
#define DQE_PARTIAL_START_X_MASK	(0x3FFF << 0)

#define DQE_TOP_PARTIAL_CON		(0x0014)
#define DQE_PARTIAL_SAME(_v)		((_v) << 2)
#define DQE_PARTIAL_SAME_MASK		(0x1 << 2)
#define DQE_PARTIAL_UPDATE_EN(_v)	((_v) << 0)
#define DQE_PARTIAL_UPDATE_EN_MASK	(0x1 << 0)

/*----------------------[TOP_LPD_MODE]---------------------------------------*/

#define DQE_TOP_LPD_MODE_CONTROL	(0x0018)
#define DQE_LPD_MODE_EXIT		(0x1 << 0)

#define DQE_TOP_LPD_ATC_CON		(0x001C)
#define LPD_ATC_PU_ON			(0x1 << 5)
#define LPD_ATC_FRAME_CNT(_v)		((_v) << 3)
#define LPD_ATC_FRAME_CNT_MASK		(0x3 << 3)
#define LPD_ATC_DIMMING_PROG		(0x1 << 2)
#define LPD_ATC_FRAME_STATE(_v)		((_v) << 0)
#define LPD_ATC_FRAME_STATE_MASK	(0x3 << 0)

#define DQE_TOP_LPD_ATC_TDR_0		(0x0020)
#define LPD_ATC_TDR_MIN(_v)		((_v) << 10)
#define LPD_ATC_TDR_MIN_MASK		(0x3FF << 10)
#define LPD_ATC_TDR_MAX(_v)		((_v) << 0)
#define LPD_ATC_TDR_MAX_MASK		(0x3FF << 0)

#define DQE_TOP_LPD_ATC_TDR_1		(0x0024)
#define LPD_ATC_TDR_MIN_D1(_v)		((_v) << 10)
#define LPD_ATC_TDR_MIN_D1_MASK		(0x3FF << 10)
#define LPD_ATC_TDR_MAX_D1(_v)		((_v) << 0)
#define LPD_ATC_TDR_MAX_D1_MASK		(0x3FF << 0)

/* TOP_LPD_SCALE
 * scale_001       : scale_0    : 0x0028
 * scale_002 ~ 007 : scale_1    : 0x002C ~ 0x0040 - [4, 0]
 * scale_008 ~ 021 : scale_2    : 0x0044 ~ 0x0078 - [14, 0]
 * scale_022       : scale_0_d1 : 0x007C
 * scale_023 ~ 027 : scale_1_d1 : 0x0080 ~ 0x0090 - [4, 0]
 * scale_028 ~ 042 : scale_2_d1 : 0x0094 ~ 0x00CC - [14, 0]
 */
#define DQE_TOP_LPD_SCALE(_n)		(0x0028 + ((_n) * 0x4))
#define DQE_TOP_LPD_SCALE0		(0x0028)
#define DQE_TOP_LPD_SCALE1(_n)		(0x002C + ((_n) * 0x4))
#define DQE_TOP_LPD_SCALE2(_n)		(0x0040 + ((_n) * 0x4))
#define DQE_TOP_LPD_SCALE0_D1		(0x007C)
#define DQE_TOP_LPD_SCALE1_D1(_n)	(0x0080 + ((_n) * 0x4))
#define DQE_TOP_LPD_SCALE2_D1(_n)	(0x0094 + ((_n) * 0x4))

#define DQE_TOP_LPD_HSC		(0x00D0)
#define LPD_HSC_PU_ON			(0x1 << 22)
#define LPD_HSC_FRAME_STATE(_v)		((_v) << 20)
#define LPD_HSC_FRAME_STATE_MASK	(0x3 << 20)
#define LPD_HSC_AVG_UPDATE(_v)		((_v) << 10)
#define LPD_HSC_AVG_UPDATE_MASK		(0x3FF << 10)
#define LPD_HSC_AVG_NOUPDATE(_v)	((_v) << 0)
#define LPD_HSC_AVG_NOUPDATE_MASK	(0x3FF << 0)

/*----------------------[VERSION]---------------------------------------------*/

#define DQE_TOP_VER			(0x00FC)
#define TOP_VER				(0x05000000)
#define TOP_VER_GET(_v)			(((_v) >> 0) & 0xFFFFFFFF)

/*----------------------[ATC]------------------------------------------------*/

#define DQE_ATC_CONTROL		(0x0400)
#define DQE_ATC_SW_RESET(_v)			((_v) << 16)
#define DQE_ATC_SW_RESET_MASK			(0x1 << 16)
#define DQE_ATC_PARTIAL_UPDATE_METHOD(_v)	((_v) << 1)
#define DQE_ATC_PARTIAL_UPDATE_METHOD_MASK	(0x1 << 1)
#define DQE_ATC_EN(_v)				((_v) << 0)
#define DQE_ATC_EN_MASK				(0x1 << 0)

#define DQE_ATC_GAIN			(0x0404)
#define ATC_ONE_DITHER(_v)		((_v) << 24)
#define ATC_ONE_DITHER_MASK		(0x1 << 24)
#define ATC_ST(_v)			((_v) << 16)
#define ATC_ST_MASK			(0xFF << 16)
#define ATC_NS(_v)			((_v) << 8)
#define ATC_NS_MASK			(0xFF << 8)
#define ATC_LT(_v)			((_v) << 0)
#define ATC_LT_MASK			(0xFF << 0)

#define DQE_ATC_WEIGHT			(0x0408)
#define ATC_LA_W_ON(_v)			((_v) << 24)
#define ATC_LA_W_ON_MASK		(0x1 << 24)
#define ATC_PL_W2_MASK			(0xF << 16)
#define ATC_LT_CALC_MODE_MASK			(0x1 << 31)
#define ATC_LT_CALC_MODE(_v)			((_v) << 31)
#define ATC_LA_W(_v)			((_v) << 28)
#define ATC_LA_W_MASK		(0x7 << 28)
#define ATC_PL_W2(_v)			((_v) << 16)
#define ATC_PL_W2_MASK			(0xF << 16)
#define ATC_PL_W1(_v)			((_v) << 0)
#define ATC_PL_W1_MASK			(0xF << 0)

#define DQE_ATC_CTMODE			(0x040C)
#define ATC_CTMODE(_v)			((_v) << 0)
#define ATC_CTMODE_MASK			(0x3 << 0)

#define DQE_ATC_PPEN			(0x0410)
#define ATC_PP_EN(_v)			((_v) << 0)
#define ATC_PP_EN_MASK			(0x1 << 0)


#define DQE_ATC_TDRMINMAX		(0x0414)
#define ATC_UPGRADE_ON(_v)		((_v) << 31)
#define ATC_UPGRADE_ON_MASK		(0x1 << 31)
#define ATC_TDR_MAX(_v)			((_v) << 16)
#define ATC_TDR_MAX_MASK		(0x3FF << 16)
#define ATC_TDR_MIN(_v)			((_v) << 0)
#define ATC_TDR_MIN_MASK		(0x3FF << 0)

#define DQE_ATC_AMBIENT_LIGHT		(0x0418)
#define ATC_AMBIENT_LIGHT(_v)		((_v) << 0)
#define ATC_AMBIENT_LIGHT_MASK		(0xFF << 0)

#define DQE_ATC_BACK_LIGHT		(0x041C)
#define ATC_BACK_LIGHT(_v)		((_v) << 0)
#define ATC_BACK_LIGHT_MASK		(0xFF << 0)

#define DQE_ATC_DSTEP			(0x0420)
#define ATC_DSTEP(_v)			((_v) << 0)
#define ATC_DSTEP_MASK			(0x3F << 0)

#define DQE_ATC_SCALE_MODE		(0x0424)
#define ATC_SCALE_MODE(_v)		((_v) << 0)
#define ATC_SCALE_MODE_MASK		(0x3 << 0)

#define DQE_ATC_THRESHOLD		(0x0428)
#define ATC_THRESHOLD_3(_v)		((_v) << 4)
#define ATC_THRESHOLD_3_MASK		(0x3 << 4)
#define ATC_THRESHOLD_2(_v)		((_v) << 2)
#define ATC_THRESHOLD_2_MASK		(0x3 << 2)
#define ATC_THRESHOLD_1(_v)		((_v) << 0)
#define ATC_THRESHOLD_1_MASK		(0x3 << 0)

#define DQE_ATC_GAIN_LIMIT		(0x042C)
#define ATC_LT_CALC_AB_SHIFT(_v)	((_v) << 16)
#define ATC_LT_CALC_AB_SHIFT_MASK	(0x3 << 16)
#define ATC_GAIN_LIMIT(_v)		((_v) << 0)
#define ATC_GAIN_LIMIT_MASK		(0x3FF << 0)

#define DQE_ATC_DIMMING_DONE_INTR	(0x0430)
#define ATC_DIMMING_IN_PROGRESS		(0x1 << 0)
#define ATC_DIMMING_IN_PROGRESS_GET(_v)	(((_v) >> 0) & 0x1)

#define DQE_ATC_PARTIAL_IBSI_P1	(0x0434)
#define ATC_IBSI_Y_P1(_v)		((_v) << 16)
#define ATC_IBSI_Y_P1_MASK		(0xFFFF << 16)
#define ATC_IBSI_X_P1(_v)		((_v) << 0)
#define ATC_IBSI_X_P1_MASK		(0xFFFF << 0)

#define DQE_ATC_PARTIAL_IBSI_P2	(0x043C)
#define ATC_IBSI_Y_P2(_v)		((_v) << 16)
#define ATC_IBSI_Y_P2_MASK		(0xFFFF << 16)
#define ATC_IBSI_X_P2(_v)		((_v) << 0)
#define ATC_IBSI_X_P2_MASK		(0xFFFF << 0)

#define DQE_ATC_DIMMING_CONTROL	(0x0440)
#define ATC_TARGET_DIM_RATIO(_v)	((_v) << 16)
#define ATC_TARGET_DIM_RATIO_MASK 	(0x1FF << 16)
#define ATC_DIM_PERIOD_SHIFT(_v)	((_v) << 0)
#define ATC_DIM_PERIOD_SHIFT_MASK 	(0xF << 0)

#define DQE_ATC_GT_CONTROL		(0x444)
#define ATC_GT_LAMDA_DSTEP(_v)	((_v) << 16)
#define ATC_GT_LAMDA_DSTEP_MASK	(0x3f << 16)
#define ATC_GT_LAMDA(_v)		((_v) << 4)
#define ATC_GT_LAMDA_MASK		(0x1ff << 4)
#define ATC_GT_HE_EN(_v)		((_v) << 0)
#define ATC_GT_HE_EN_MASK		(0x1 << 0)

#define DQE_ATC_CDF_DIV				(0x448)
#define ATC_CDF_SHIFT(_v)		((_v) << 16)
#define ATC_CDF_SHIFT_MASK		(0xf << 16)
#define ATC_CDF_DIV(_v)			((_v) << 0)
#define ATC_CDF_DIV_MASK		(0x3fff << 0)

#define DQE_ATC_HE_CLIP_CNT			4
#define DQE_ATC_HE_CLIP_MIN(_v)	(0x44c + (4 * (_v)))
#define ATC_HE_CLIP_MIN(_v, _i)		((_v) << (8 *(_i)))
#define ATC_HE_CLIP_MIN_MASK(_i)		(0xff << (8 *(_i)))

#define DQE_ATC_HE_CLIP_MAX(_v)	(0x45c + (4 * (_v)))
#define ATC_HE_CLIP_MAX(_v, _i)		((_v) << (8 *(_i)))
#define ATC_HE_CLIP_MAX_MASK(_i)		(0xff << (8 *(_i)))
/*----------------------[HSC]------------------------------------------------*/

#define DQE_HSC_CONTROL		(0x0800)
#define DQE_HSC_SW_RESET(_v)			((_v) << 16)
#define DQE_HSC_SW_RESET_MASK			(0x1 << 16)
#define DQE_HSC_PARTIAL_UPDATE_METHOD(_v)	((_v) << 1)
#define DQE_HSC_PARTIAL_UPDATE_METHOD_MASK	(0x1 << 1)
#define DQE_HSC_EN(_v)				((_v) << 0)
#define DQE_HSC_EN_MASK				(0x1 << 0)

#define DQE_HSC_LOCAL_CONTROL		(0x0804)
#define HSC_LBC_GROUPMODE(_v)		((_v) << 9)
#define HSC_LBC_GROUPMODE_MASK		(0x3 << 9)
#define HSC_LBC_ON			(0x1 << 8)
#define HSC_LHC_GROUPMODE(_v)		((_v) << 5)
#define HSC_LHC_GROUPMODE_MASK		(0x3 << 5)
#define HSC_LHC_ON			(0x1 << 4)
#define HSC_LSC_GROUPMODE(_v)		((_v) << 1)
#define HSC_LSC_GROUPMODE_MASK		(0x3 << 1)
#define HSC_LSC_ON			(0x1 << 0)

#define DQE_HSC_GLOBAL_CONTROL_1	(0x0808)
#define HSC_GHC_GAIN(_v)		((_v) << 16)
#define HSC_GHC_GAIN_MASK		(0x3FF << 16)
#define HSC_GHC_ON			(0x1 << 0)

#define DQE_HSC_GLOBAL_CONTROL_0	(0x080C)
#define HSC_GSC_GAIN(_v)		((_v) << 16)
#define HSC_GSC_GAIN_MASK		(0x7FF << 16)
#define HSC_GSC_ON			(0x1 << 0)

#define DQE_HSC_GLOBAL_CONTROL_2	(0x0810)
#define HSC_GBC_GAIN(_v)		((_v) << 16)
#define HSC_GBC_GAIN_MASK		(0x7FF << 16)
#define HSC_GBC_ON			(0x1 << 0)

#define DQE_HSC_CONTROL_ALPHA_SAT	(0x0814)
#define HSC_ALPHA_SAT_SHIFT2(_v)	((_v) << 28)
#define HSC_ALPHA_SAT_SHIFT2_MASK	(0x7 << 28)
#define HSC_ALPHA_SAT_SHIFT1(_v)	((_v) << 16)
#define HSC_ALPHA_SAT_SHIFT1_MASK	(0x1FF << 16)
#define HSC_ALPHA_SAT_SCALE(_v)		((_v) << 4)
#define HSC_ALPHA_SAT_SCALE_MASK	(0xF << 4)
#define HSC_ALPHA_SAT_ON		(0x1 << 0)

#define DQE_HSC_CONTROL_ALPHA_BRI	(0x0818)
#define HSC_ALPHA_BRI_SHIFT2(_v)	((_v) << 28)
#define HSC_ALPHA_BRI_SHIFT2_MASK	(0x7 << 28)
#define HSC_ALPHA_BRI_SHIFT1(_v)	((_v) << 16)
#define HSC_ALPHA_BRI_SHIFT1_MASK	(0x1FF << 16)
#define HSC_ALPHA_BRI_SCALE(_v)		((_v) << 4)
#define HSC_ALPHA_BRI_SCALE_MASK	(0xF << 4)
#define HSC_ALPHA_BRI_ON		(0x1 << 0)

/*
 * MC[1,4]_R[0,2]
 * _n : [0, 2]
 */
#define DQE_HSC_CONTROL_MC1_R(_n)	(0x081C + ((_n) * 4 * 0x4))
#define DQE_HSC_CONTROL_MC2_R(_n)	(0x0820 + ((_n) * 4 * 0x4))
#define DQE_HSC_CONTROL_MC3_R(_n)	(0x0824 + ((_n) * 4 * 0x4))
#define DQE_HSC_CONTROL_MC4_R(_n)	(0x0828 + ((_n) * 4 * 0x4))

/* MC1 */
#define HSC_MC_SAT_GAIN_R(_v)		((_v) << 16)
#define HSC_MC_SAT_GAIN_R_MASK		(0x7FF << 16)
#define HSC_MC_BC_SAT_R(_v)		((_v) << 8)
#define HSC_MC_BC_SAT_R_MASK		(0x3 << 8)
#define HSC_MC_BC_HUE_R(_v)		((_v) << 4)
#define HSC_MC_BC_HUE_R_MASK		(0x3 << 4)
#define HSC_MC_ON_R			(0x1 << 0)
/* MC2 */
#define HSC_MC_BRI_GAIN_R(_v)		((_v) << 16)
#define HSC_MC_BRI_GAIN_R_MASK		(0x7FF << 16)
#define HSC_MC_HUE_GAIN_R(_v)		((_v) << 0)
#define HSC_MC_HUE_GAIN_R_MASK		(0x3FF << 0)
/* MC3 */
#define HSC_MC_BRI_GAIN_R(_v)		((_v) << 16)
#define HSC_MC_BRI_GAIN_R_MASK		(0x7FF << 16)
#define HSC_MC_HUE_GAIN_R(_v)		((_v) << 0)
#define HSC_MC_HUE_GAIN_R_MASK		(0x3FF << 0)
/* MC4 */
#define HSC_MC_H2_R(_v)			((_v) << 16)
#define HSC_MC_H2_R_MASK		(0xFFF << 16)
#define HSC_MC_H1_R(_v)			((_v) << 0)
#define HSC_MC_H1_R_MASK		(0xFFF << 0)

#define DQE_HSC_CONTROL_MC1_R0		(0x081C)
#define DQE_HSC_CONTROL_MC2_R0		(0x0820)
#define DQE_HSC_CONTROL_MC3_R0		(0x0824)
#define DQE_HSC_CONTROL_MC4_R0		(0x0828)

#define DQE_HSC_CONTROL_MC1_R1		(0x082C)
#define DQE_HSC_CONTROL_MC2_R1		(0x0830)
#define DQE_HSC_CONTROL_MC3_R1		(0x0834)
#define DQE_HSC_CONTROL_MC4_R1		(0x0838)

#define DQE_HSC_CONTROL_MC1_R2		(0x083C)
#define DQE_HSC_CONTROL_MC2_R2		(0x0840)
#define DQE_HSC_CONTROL_MC3_R2		(0x0844)
#define DQE_HSC_CONTROL_MC4_R2		(0x0848)

#define DQE_HSC_CONTROL_YCOMP		(0x084C)
#define HSC_BLEND_MANUAL_GAIN(_v)	((_v) << 16)
#define HSC_BLEND_MANUAL_GAIN_MASK	(0xFF << 16)
#define HSC_YCOMP_GAIN(_v)		((_v) << 8)
#define HSC_YCOMP_GAIN_MASK		(0xF << 8)
#define HSC_BLEND_ON			(0x1 << 2)
#define HSC_YCOMP_DITH_ON		(0x1 << 1)
#define HSC_YCOMP_ON			(0x1 << 0)

/*
 * LSC_GAIN_P1 (0~5) : 0x0900 ~ 0x0914
 * LSC_GAIN_P2 (0~5) : 0x0918 ~ 0x092C
 * LSC_GAIN_P3 (0~5) : 0x0930 ~ 0x0944
 * Pn -> _n: [0, 5] / _x: [0, 11]
 * P  -> _n: [0, 17]
 */
#define DQE_HSC_LSC_GAIN_P(_n)		(0x0900 + ((_n) * 0x4))
#define DQE_HSC_LSC_GAIN_P1(_n)	(0x0900 + ((_n) * 0x4))
#define DQE_HSC_LSC_GAIN_P2(_n)	(0x0918 + ((_n) * 0x4))
#define DQE_HSC_LSC_GAIN_P3(_n)	(0x0930 + ((_n) * 0x4))

#define HSC_LSC_GAIN_P_H(_v)		((_v) << 16)
#define HSC_LSC_GAIN_P_H_MASK		(0x7FF << 16)
#define HSC_LSC_GAIN_P_L(_v)		((_v) << 0)
#define HSC_LSC_GAIN_P_L_MASK		(0x7FF << 0)
#define HSC_LSC_GAIN_P(_x, _v)		((_v) << (0 + (16 * ((_x) & 0x1))))
#define HSC_LSC_GAIN_P_MASK(_x)		(0x7FF << (0 + (16 * ((_x) & 0x1))))

/*
 * LHC_GAIN_P1 (0~5) : 0x0948 ~ 0x095C
 * LHC_GAIN_P2 (0~5) : 0x0960 ~ 0x0974
 * LHC_GAIN_P3 (0~5) : 0x0978 ~ 0x098C
 * Pn -> _n: [0, 5] / _x: [0, 11]
 * P  -> _n: [0, 17]
 */
#define DQE_HSC_LHC_GAIN_P(_n)		(0x0948 + ((_n) * 0x4))
#define DQE_HSC_LHC_GAIN_P1(_n)	(0x0948 + ((_n) * 0x4))
#define DQE_HSC_LHC_GAIN_P2(_n)	(0x0960 + ((_n) * 0x4))
#define DQE_HSC_LHC_GAIN_P3(_n)	(0x0978 + ((_n) * 0x4))

#define HSC_LHC_GAIN_P_H(_v)		((_v) << 16)
#define HSC_LHC_GAIN_P_H_MASK		(0x3FF << 16)
#define HSC_LHC_GAIN_P_L(_v)		((_v) << 0)
#define HSC_LHC_GAIN_P_L_MASK		(0x3FF << 0)
#define HSC_LHC_GAIN_P(_x, _v)		((_v) << (0 + (16 * ((_x) & 0x1))))
#define HSC_LHC_GAIN_P_MASK(_x)		(0x3FF << (0 + (16 * ((_x) & 0x1))))

/*
 * LBC_GAIN_P1 (0~5) : 0x0990 ~ 0x09A4
 * LBC_GAIN_P2 (0~5) : 0x09A8 ~ 0x09BC
 * LBC_GAIN_P3 (0~5) : 0x09C0 ~ 0x09D4
 * Pn -> _n: [0, 5] / _x: [0, 11]
 * P  -> _n: [0, 17]
 */
#define DQE_HSC_LBC_GAIN_P(_n)		(0x0990 + ((_n) * 0x4))
#define DQE_HSC_LBC_GAIN_P1(_n)	(0x0990 + ((_n) * 0x4))
#define DQE_HSC_LBC_GAIN_P2(_n)	(0x09A8 + ((_n) * 0x4))
#define DQE_HSC_LBC_GAIN_P3(_n)	(0x09C0 + ((_n) * 0x4))

#define HSC_LBC_GAIN_P_H(_v)		((_v) << 16)
#define HSC_LBC_GAIN_P_H_MASK		(0x7FF << 16)
#define HSC_LBC_GAIN_P_L(_v)		((_v) << 0)
#define HSC_LBC_GAIN_P_L_MASK		(0x7FF << 0)
#define HSC_LBC_GAIN_P(_x, _v)		((_v) << (0 + (16 * ((_x) & 0x1))))
#define HSC_LBC_GAIN_P_MASK(_x)		(0x7FF << (0 + (16 * ((_x) & 0x1))))

/*
 * HSC_POLY_S (0~4) : 0x09D8 ~ 0x09E8
 * HSC_POLY_B (0~4) : 0x09EC ~ 0x09FC
 * Sn -> _n: [0, 4] / _x: [0, 8]
 * S  -> _n: [0, 9]
 */
#define DQE_HSC_POLY(_n)		(0x09D8 + ((_n) * 0x4))
#define DQE_HSC_POLY_S(_n)		(0x09D8 + ((_n) * 0x4))
#define DQE_HSC_POLY_B(_n)		(0x09EC + ((_n) * 0x4))

#define HSC_POLY_CURVE_H(_v)		((_v) << 16)
#define HSC_POLY_CURVE_H_MASK		(0x3FF << 16)
#define HSC_POLY_CURVE_L(_v)		((_v) << 0)
#define HSC_POLY_CURVE_L_MASK		(0x3FF << 0)
#define HSC_POLY_CURVE(_x, _v)		((_v) << (0 + (16 * ((_x) & 0x1))))
#define HSC_POLY_CURVE_MASK(_x)		(0x3FF << (0 + (16 * ((_x) & 0x1))))

/*----------------------[GAMMA_MATRIX]---------------------------------------*/

#define DQE_GAMMA_MATRIX_CON		(0x0C00)
#define GAMMA_MATRIX_EN			(0x1 << 0)

/*
 * GAMMA_MATRIX_COEFF (0~4) : 0x0C04 ~ 0x0C14
 * 3x3 : (00-01-02) - (10-11-12) - (20-21-22)
 *     (00-01-02) : Reg0.L-Reg0.H-Reg1.L
 *     (10-11-12) : Reg1.H-Reg2.L-Reg2.H
 *     (20-21-22) : Reg3.L-Reg3.H-Reg4.L
 */
#define DQE_GAMMA_MATRIX_COEFF(_n)	(0x0C04 + ((_n) * 0x4))
#define DQE_GAMMA_MATRIX_COEFF0	(0x0C04)
#define DQE_GAMMA_MATRIX_COEFF1	(0x0C08)
#define DQE_GAMMA_MATRIX_COEFF2	(0x0C0C)
#define DQE_GAMMA_MATRIX_COEFF3	(0x0C10)
#define DQE_GAMMA_MATRIX_COEFF4	(0x0C14)

#define GAMMA_MATRIX_COEFF_H(_v)	((_v) << 16)
#define GAMMA_MATRIX_COEFF_H_MASK	(0x3FFF << 16)
#define GAMMA_MATRIX_COEFF_L(_v)	((_v) << 0)
#define GAMMA_MATRIX_COEFF_L_MASK	(0x3FFF << 0)

#define DQE_GAMMA_MATRIX_OFFSET0	(0x0C18)
#define GAMMA_MATRIX_OFFSET_1(_v)	((_v) << 16)
#define GAMMA_MATRIX_OFFSET_0(_v)	((_v) << 0)

#define DQE_GAMMA_MATRIX_OFFSET1	(0x0C1C)
#define GAMMA_MATRIX_OFFSET_2(_v)	((_v) << 0)

/*----------------------[DEGAMMA_CON]----------------------------------------*/
#define DQE_DEGAMMA_CON			(0x1000)
#define DEGAMMA_EN(_v)			(((_v) & 0x1) << 0)
#define DEGAMMA_EN_MASK			(0x1 << 0)

#define DQE_DEGAMMA_POSX(_n)		(0x1004 + ((_n) * 0x4))
#define DQE_DEGAMMA_POSY(_n)		(0x1048 + ((_n) * 0x4))

#define DQE_DEGAMMALUT(_n)		(0x1004 + ((_n) * 0x4))

#define DEGAMMA_LUT_H(_v)		((_v) << 16)
#define DEGAMMA_LUT_H_MASK		(0x1FFF << 16)
#define DEGAMMA_LUT_L(_v)		((_v) << 0)
#define DEGAMMA_LUT_L_MASK		(0x1FFF << 0)
#define DEGAMMA_LUT(_x, _v)		((_v) << (0 + (16 * ((_x) & 0x1))))
#define DEGAMMA_LUT_MASK(_x)		(0x1FFF << (0 + (16 * ((_x) & 0x1))))

#define DQE_DEGAMMALUT_POS_SIZE		33
#define DQE_DEGAMMALUT_REG_CNT		17

/*----------------------[LINEAR_MATRIX]--------------------------------------*/

#define DQE_LINEAR_MATRIX_CON		(0x1400)
#define LINEAR_MATRIX_EN		(0x1 << 0)

/*
 * GAMMA_MATRIX_COEFF (0~4) : 0x1404 ~ 0x1414
 * 3x3 : (00-01-02) - (10-11-12) - (20-21-22)
 *     (00-01-02) : Reg0.L-Reg0.H-Reg1.L
 *     (10-11-12) : Reg1.H-Reg2.L-Reg2.H
 *     (20-21-22) : Reg3.L-Reg3.H-Reg4.L
 */
#define DQE_LINEAR_MATRIX_COEFF(_n)	(0x1404 + ((_n) * 0x4))
#define DQE_LINEAR_MATRIX_COEFF0	(0x1404)
#define DQE_LINEAR_MATRIX_COEFF1	(0x1408)
#define DQE_LINEAR_MATRIX_COEFF2	(0x140C)
#define DQE_LINEAR_MATRIX_COEFF3	(0x1410)
#define DQE_LINEAR_MATRIX_COEFF4	(0x1414)

#define LINEAR_MATRIX_COEFF_H(_v)	((_v) << 16)
#define LINEAR_MATRIX_COEFF_H_MASK	(0xFFFF << 16)
#define LINEAR_MATRIX_COEFF_L(_v)	((_v) << 0)
#define LINEAR_MATRIX_COEFF_L_MASK	(0xFFFF << 0)

#define DQE_LINEAR_MATRIX_OFFSET0	(0x1418)
#define LINEAR_MATRIX_OFFSET_1(_v)	((_v) << 16)
#define LINEAR_MATRIX_OFFSET_1_MASK	(0x3FFF << 16)
#define LINEAR_MATRIX_OFFSET_0(_v)	((_v) << 0)
#define LINEAR_MATRIX_OFFSET_0_MASK	(0x3FFF << 0)

#define DQE_LINEAR_MATRIX_OFFSET1	(0x141C)
#define LINEAR_MATRIX_OFFSET_2(_v)	((_v) << 0)
#define LINEAR_MATRIX_OFFSET_2_MASK	(0x3FFF << 0)

/*----------------------[CGC]------------------------------------------------*/

#define DQE_CGC_CON			(0x0)
#define CGC_LUT_READ_SHADOW		(0x1 << 2)
#define CGC0_COEF_SHD_UP_EN(_v)		((_v) << 12)
#define CGC0_COEF_SHD_UP_EN_MASK	(0x1 << 12)
#define CGC0_COEF_DMA_EN(_v)		((_v) << 8)
#define CGC0_COEF_DMA_EN_MASK		(0x1 << 8)
#define CGC_COEF_DMA_REQ		(1 << 4)
#define CGC_COEF_DMA_REQ_MASK		(1 << 4)
#define CGC0_EN				(1 << 0)


/*
 * DEGAMMALUT (0~2) : 0x1804 ~ 0x180C
 * _n: [0, 2]
 */
#define DQE_CGC_MC_R(_n)		(0x0004 + ((_n) * 0x4))
#define DQE_CGC_MC_R0			(0x0004)
#define DQE_CGC_MC_R1			(0x0008)
#define DQE_CGC_MC_R2			(0x000C)

#define CGC_MC_GAIN_R(_v)		((_v) << 16)
#define CGC_MC_GAIN_R_MASK		(0xFFF << 16)
#define CGC_MC_INVERSE_R(_v)		((_v) << 1)
#define CGC_MC_INVERSE_R_MASK		(0x1 << 1)
#define CGC_MC_ON_R(_v)			((_v) << 0)
#define CGC_MC_ON_R_MASK		(0x1 << 0)

/*----------------------[DEGAMMA_CON]----------------------------------------*/

#define DQE_REGAMMA0_CON		(0x2000)
#define DQE_REGAMMA1_CON		(0x4800)
#define DQE_REGAMMA2_CON		(0x4C00)
#define DQE_REGAMMA_BASE(id)	((id) ? (id == 1) ? DQE_REGAMMA1_CON : \
							DQE_REGAMMA2_CON : DQE_REGAMMA0_CON)
#define REGAMMA_EN			(0x1 << 0)

/*
 * REGAMMALUT_R (0~32) : 0x2004 ~ 0x2084
 * REGAMMALUT_G (0~32) : 0x2088 ~ 0x2108
 * REGAMMALUT_B (0~32) : 0x210C ~ 0x218C
 * _n: [0, 32] / _x: [0, 64]
 */
#define DQE_REGAMMALUT(id, _n)		((DQE_REGAMMA_BASE(id) + 0x004) + ((_n) * 0x4))
#define DQE_REGAMMA_R_POSX(id, _n)      ((DQE_REGAMMA_BASE(id) + 0x004) + ((_n) * 0x4))
#define DQE_REGAMMA_R_POSY(id, _n)      ((DQE_REGAMMA_BASE(id) + 0x048) + ((_n) * 0x4))
#define DQE_REGAMMA_G_POSX(id, _n)      ((DQE_REGAMMA_BASE(id) + 0x08C) + ((_n) * 0x4))
#define DQE_REGAMMA_G_POSY(id, _n)      ((DQE_REGAMMA_BASE(id) + 0x0D0) + ((_n) * 0x4))
#define DQE_REGAMMA_B_POSX(id, _n)      ((DQE_REGAMMA_BASE(id) + 0x114) + ((_n) * 0x4))
#define DQE_REGAMMA_B_POSY(id, _n)      ((DQE_REGAMMA_BASE(id) + 0x158) + ((_n) * 0x4))

#define DQE_REGAMMA_POS_LUT_SIZE		(33) // POSX/POSY
#define DQE_REGAMMALUT_REG_CNT		(17) // POSX/POSY regs

#define REGAMMA_LUT_H(_v)		((_v) << 16)
#define REGAMMA_LUT_H_MASK		(0x1FFF << 16)
#define REGAMMA_LUT_L(_v)		((_v) << 0)
#define REGAMMA_LUT_L_MASK		(0x1FFF << 0)
#define REGAMMA_LUT(_x, _v)		((_v) << (0 + (16 * ((_x) & 0x1))))
#define REGAMMA_LUT_MASK(_x)		(0x1FFF << (0 + (16 * ((_x) & 0x1))))

/*----------------------[CGC_DITHER]-----------------------------------------*/

#define DQE_CGC_DITHER			(0x2400)
#define DQE_DISP_DITHER		(0x2800)
#define DITHER_TABLE_SEL_B		(0x1 << 7)
#define DITHER_TABLE_SEL_G		(0x1 << 6)
#define DITHER_TABLE_SEL_R		(0x1 << 5)
#define DITHER_FRAME_OFFSET_SHIFT	3
#define DITHER_FRAME_OFFSET(_v)		((_v) << DITHER_FRAME_OFFSET_SHIFT)
#define DITHER_FRAME_OFFSET_MASK	(0x3 << 3)
#define DITHER_FRAME_CON		(0x1 << 2)
#define DITHER_MODE			(0x1 << 1)
#define DITHER_EN(_v)			((_v) << 0)
#define DITHER_EN_MASK			(0x1 << 0)

/*----------------------[RCD]------------------------------------------------*/
#define DQE_RCD				(0x3000)
#define DQE_RCD_EN(_v)			((_v) << 0)

/*----------------------[HIST]-----------------------------------------------*/

#define DQE_HIST(_i)			((0x3000) + ((_i) * 0x600))
#define HIST_ROI_BLOCKING_EN		(0x1 << 4)
#define HIST_POS_SEL_MASK		(0x1 << 3)
#define HIST_POS_SEL(_v)		((_v) << 3)
#define HIST_LUMA_SEL			(0x1 << 2)
#define HIST_ROI_ON			(0x1 << 1)
#define HIST_EN				(0x1 << 0)

#define DQE_HIST_SIZE(_i)		(0x3004 + ((_i) * 0x600))
#define HIST_VSIZE(_v)			((_v) << 16)
#define HIST_VSIZE_MASK			(0x3FFF << 16)
#define HIST_HSIZE(_v)			((_v) << 0)
#define HIST_HSIZE_MASK			(0x3FFF << 0)

#define DQE_HIST_START(_i)		(0x3008 + ((_i) * 0x600))
#define HIST_START_Y(_v)		((_v) << 16)
#define HIST_START_Y_MASK		(0x3FFF << 16)
#define HIST_START_X(_v)		((_v) << 0)
#define HIST_START_X_MASK		(0x3FFF << 0)

#define DQE_HIST_WEIGHT_0(_i)		(0x300C + ((_i) * 0x600))
#define HIST_WEIGHT_G(_v)		((_v) << 16)
#define HIST_WEIGHT_G_MASK		(0x7FF << 16)
#define HIST_WEIGHT_R(_v)		((_v) << 0)
#define HIST_WEIGHT_R_MASK		(0x7FF << 0)

#define DQE_HIST_WEIGHT_1(_i)		(0x3010 + ((_i) * 0x600))
#define HIST_WEIGHT_B(_v)		((_v) << 0)
#define HIST_WEIGHT_B_MASK		(0x7FF << 0)

#define DQE_HIST_THRESH(_i)		(0x3014 + ((_i) * 0x600))
#define HIST_THRESHOLD(_v)		((_v) << 0)
#define HIST_THRESHOLD_MASK		(0x3FF << 0)

#define DQE_HIST_BLOCK_SIZE(_i)		(0x3018 + ((_i) * 0x600))
#define HIST_BLOCK_VSIZE(_v)		HIST_VSIZE(_v)
#define HIST_BLOCK_VSIZE_MASK		HIST_VSIZE_MASK
#define HIST_BLOCK_HSIZE(_v)		HIST_HSIZE(_v)
#define HIST_BLOCK_HSIZE_MASK		HIST_HSIZE_MASK

#define DQE_HIST_BLOCK_START(_i)	(0x301C + ((_i) * 0x600))
#define HIST_BLOCK_START_Y(_v)		HIST_START_Y(_v)
#define HIST_BLOCK_START_Y_MASK		HIST_START_Y_MASK
#define HIST_BLOCK_START_X(_v)		HIST_START_X(_v)
#define HIST_BLOCK_START_X_MASK		HIST_START_X_MASK

/*
 * HIST_BIN (0~127) : 0x3400 ~ 0x35FC
 * _n: [0, 127] / _x: [0, 255]
 */
#define DQE_HIST_BASE(_i)		(0x3400 + ((_i) * 0x600))
#define DQE_HIST_BIN(_i, _n)		(DQE_HIST_BASE(_i) + ((_n) * 0x4))
#define HIST_BIN_H(_v)			((_v) << 16)
#define HIST_BIN_H_MASK			(0xFFFF << 16)
#define HIST_BIN_L(_v)			((_v) << 0)
#define HIST_BIN_L_MASK			(0xFFFF << 0)
#define HIST_BIN(_x, _v)		((_v) << (0 + (16 * ((_x) & 0x1))))
#define HIST_BIN_MASK(_x)		(0xFFFF << (0 + (16 * ((_x) & 0x1))))
#define HIST_BIN_L_GET(_v)		((_v) & HIST_BIN_L_MASK)
#define HIST_BIN_H_GET(_v)		(((_v) & HIST_BIN_H_MASK) >> 16)

/*----------------------[SECURE_ROI]-----------------------------------------*/

#define DQE_SECURE_ROI_SECURITY	(0x3C00)
#define DQE_ROI_SECURITY		(0x1 << 0)

#define DQE_SECURE_ROI_SIZE		(0x3C04)
#define DQE_SECURE_ROI_VSIZE(_v)	((_v) << 16)
#define DQE_SECURE_ROI_VSIZE_MASK	(0x3FFF << 16)
#define DQE_SECURE_ROI_HSIZE(_v)	((_v) << 0)
#define DQE_SECURE_ROI_HSIZE_MASK	(0x3FFF << 0)

#define DQE_SECURE_ROI_START		(0x3C08)
#define DQE_SECURE_ROI_START_Y(_v)	((_v) << 16)
#define DQE_SECURE_ROI_START_Y_MASK	(0x3FFF << 16)
#define DQE_SECURE_ROI_START_X(_v)	((_v) << 0)
#define DQE_SECURE_ROI_START_X_MASK	(0x3FFF << 0)

#define DQE_SECURE_ROI_CON		(0x3C0C)
#define DQE_SECURE_ROI_FRAME_CONTROL	(0x1 << 1)
#define DQE_SECURE_ROI_EN		(0x1 << 0)

/*----------------------[CGC_LUT]-----------------------------------------*/

/*
 * CGC_LUT_R (0~2456) : 0x1000 ~ 0x3660
 * CGC_LUT_G (0~2456) : 0x4000 ~ 0x6660
 * CGC_LUT_B (0~2456) : 0x7000 ~ 0x9660
 * _n: [0, 2456] / _x: [0, 4913]
 */
#define DQE_CGC_LUT(_n)			(0x1000 + ((_n) * 0x4))
#define DQE_CGC_LUT_R(_n)		(0x1000 + ((_n) * 0x4))
#define DQE_CGC_LUT_G(_n)		(0x4000 + ((_n) * 0x4))
#define DQE_CGC_LUT_B(_n)		(0x7000 + ((_n) * 0x4))

#define CGC_LUT_H(_v)			((_v) << 16)
#define CGC_LUT_H_MASK			(0x1FFF << 16)
#define CGC_LUT_L(_v)			((_v) << 0)
#define CGC_LUT_L_MASK			(0x1FFF << 0)
#define CGC_LUT_LH(_x, _v)		((_v) << (0 + (16 * ((_x) & 0x1))))
#define CGC_LUT_MASK(_x)		(0x1FFF << (0 + (16 * ((_x) & 0x1))))

#endif /* __DQE_REGS_H__ */
