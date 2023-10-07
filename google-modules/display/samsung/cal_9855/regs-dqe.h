/* SPDX-License-Identifier: GPL-2.0-only
 *
 * cal_9855/regs-dqe.h
 *
 * Copyright (c) 2021 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Jaehoe Yang <jaehoe.yang@samsung.com>
 * Jiun Yu <jiun.yu@samsung.com>
 *
 * Register definition file for Samsung Display Quality Enhancer
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _REGS_DQE_9855_H
#define _REGS_DQE_9855_H


/*
 *-------------------------------------------------------------------
 * DECON_DQE: DQE SFR list
 * DQE0's base address : 0x1C2A_0000
 * DQE1's base address : 0x1C2B_0000
 *-------------------------------------------------------------------
 */

/*----------------------[TOP_LPD_MODE]---------------------------------------*/


/* TOP_LPD_SCALE
 * scale_001       : scale_0    : 0x0028
 * scale_002 ~ 007 : scale_1    : 0x002C ~ 0x0040 - [4, 0]
 * scale_008 ~ 021 : scale_2    : 0x0044 ~ 0x0078 - [14, 0]
 * scale_022       : scale_0_d1 : 0x007C
 * scale_023 ~ 027 : scale_1_d1 : 0x0080 ~ 0x0090 - [4, 0]
 * scale_028 ~ 042 : scale_2_d1 : 0x0094 ~ 0x00CC - [14, 0]
 */

/*----------------------[VERSION]---------------------------------------------*/

/*----------------------[ATC]------------------------------------------------*/

/*----------------------[HSC]------------------------------------------------*/

/*
 * MC[1,4]_R[0,2]
 * _n : [0, 2]
 */

/*
 * HSC_POLY_S (0~4) : 0x09D8 ~ 0x09E8
 * HSC_POLY_B (0~4) : 0x09EC ~ 0x09FC
 * Sn -> _n: [0, 4] / _x: [0, 8]
 * S  -> _n: [0, 9]
 */

/*
 * LSC_GAIN_P1 (0~23) : 0x0C00 ~ 0x0C5C
 * LSC_GAIN_P2 (0~23) : 0x0C60 ~ 0x0CBC
 * LSC_GAIN_P3 (0~23) : 0x0CC0 ~ 0x0D1C
 * Pn -> _n: [0, 23] / _x: [0, 47]
 * P  -> _n: [0, 71]
 */

/*
 * LHC_GAIN_P1 (0~23) : 0x0D20 ~ 0x0D7C
 * LHC_GAIN_P2 (0~23) : 0x0D80 ~ 0x0DDC
 * LHC_GAIN_P3 (0~23) : 0x0DE0 ~ 0x0E3C
 * Pn -> _n: [0, 23] / _x: [0, 47]
 * P  -> _n: [0, 71]
 */

/*
 * LBC_GAIN_P1 (0~23) : 0x0E40 ~ 0x0E9C
 * LBC_GAIN_P2 (0~23) : 0x0EA0 ~ 0x0EFC
 * LBC_GAIN_P3 (0~23) : 0x0F00 ~ 0x0F5C
 * Pn -> _n: [0, 23] / _x: [0, 47]
 * P  -> _n: [0, 71]
 */

/*----------------------[GAMMA_MATRIX]---------------------------------------*/

/*
 * GAMMA_MATRIX_COEFF (0~4) : 0x1404 ~ 0x1414
 * 3x3 : (00-01-02) - (10-11-12) - (20-21-22)
 *     (00-01-02) : Reg0.L-Reg0.H-Reg1.L
 *     (10-11-12) : Reg1.H-Reg2.L-Reg2.H
 *     (20-21-22) : Reg3.L-Reg3.H-Reg4.L
 */

/*----------------------[DEGAMMA_CON]----------------------------------------*/

/*
 * DEGAMMALUT (0~32) : 0x1804 ~ 0x1884
 * _n: [0, 32] / _x: [0, 64]
 */

/*----------------------[LINEAR_MATRIX]--------------------------------------*/

/*
 * GAMMA_MATRIX_COEFF (0~4) : 0x1C04 ~ 0x1C14
 * 3x3 : (00-01-02) - (10-11-12) - (20-21-22)
 *     (00-01-02) : Reg0.L-Reg0.H-Reg1.L
 *     (10-11-12) : Reg1.H-Reg2.L-Reg2.H
 *     (20-21-22) : Reg3.L-Reg3.H-Reg4.L
 */

/*----------------------[CGC]------------------------------------------------*/

// new field in DQE_CGC_CON register for 9855
#define CGC_COEF_DMA_REQ		(1 << 4)
#define CGC_COEF_DMA_REQ_MASK		(1 << 4)

/*
 * DEGAMMALUT (0~2) : 0x1804 ~ 0x180C
 * _n: [0, 2]
 */

/*----------------------[REGAMMA_CON]----------------------------------------*/

/*
 * REGAMMALUT_R (0~32) : 0x2404 ~ 0x2484
 * REGAMMALUT_G (0~32) : 0x2488 ~ 0x2508
 * REGAMMALUT_B (0~32) : 0x250C ~ 0x258C
 * _n: [0, 32] / _x: [0, 64]
 */

/*----------------------[CGC_DITHER]-----------------------------------------*/


/*----------------------[RCD]------------------------------------------------*/

// new DQE_RCD register for 9855
#define DQE_RCD				(0x3000)
#define DQE_RCD_EN(_v)			((_v) << 0)

/*----------------------[HIST]-----------------------------------------------*/

// new field in DQE_HIST register for 9855
#define HIST_POS_SEL(_v)		((_v) << 3)
#define HIST_POS_SEL_MASK		(1 << 3)

/*
 * HIST_BIN (0~127) : 0x3800 ~ 0x39FC
 * _n: [0, 127] / _x: [0, 255]
 */

/*----------------------[SECURE_ROI]-----------------------------------------*/


/*----------------------[CGC_LUT]-----------------------------------------*/

/*
 * CGC_LUT_R (0~2456) : 0x4000 ~ 0x6660
 * CGC_LUT_G (0~2456) : 0x8000 ~ 0xA660
 * CGC_LUT_B (0~2456) : 0xC000 ~ 0xE660
 * _n: [0, 2456] / _x: [0, 4913]
 */

#endif /* _REGS_DQE_9855_H */
