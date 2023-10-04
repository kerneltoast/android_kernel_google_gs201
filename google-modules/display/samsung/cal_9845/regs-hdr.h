/* SPDX-License-Identifier: GPL-2.0-only
 *
 * cal_9845/regs-hdr.h
 *
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Jaehoe Yang <jaehoe.yang@samsung.com>
 * Jiun Yu <jiun.yu@samsung.com>
 *
 * Register definition file for Samsung HDR
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _REGS_HDR_H
#define _REGS_HDR_H

/*
 *-------------------------------------------------------------------
 * DPP_HDR(L0~L5) SFR list
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
#define HDR_VERSION_GET(_v)		(((_v) >> 0) & 0xFFFFFFFF)

#define HDR_LSI_L_COM_CTRL		(0x0004)
#define COM_CTRL_ENABLE_MASK		(1 << 0)

#define HDR_LSI_L_MOD_CTRL		(0x0008)
#define MOD_CTRL_TEN(_v)		((_v) << 5)	/* H only */
#define MOD_CTRL_TEN_MASK		(0x1 << 5)
#define MOD_CTRL_GEN(_v)		((_v) << 2)
#define MOD_CTRL_GEN_MASK		(0x1 << 2)
#define MOD_CTRL_EEN(_v)		((_v) << 1)
#define MOD_CTRL_EEN_MASK		(0x1 << 1)
#define MOD_CTRL_OEN(_v)		((_v) << 0)
#define MOD_CTRL_OEN_MASK		(0x1 << 0)

/*-----[ OETF : Inverse EOTF ]-------------------------------------------------
 * 32-segment transfer function, 16-bit >> 10-bit
 *-----------------------------------------------------------------------------
 */
#define HDR_OETF_POSX_LUT_REG_CNT	(17)
/*
 * OETF_POSX (0~16) : 0x000C ~ 0x004C
 * _n: [0, 16] / _x: [0, 32]
 */
#define HDR_LSI_L_OETF_POSX(_n)		(0x000C + ((_n) * 0x4))
#define OETF_POSX_H(_v)			((_v) << 16)
#define OETF_POSX_H_MASK		(0xFFFF << 16)
#define OETF_POSX_L(_v)			((_v) << 0)
#define OETF_POSX_L_MASK		(0xFFFF << 0)

#define HDR_OETF_POSY_LUT_REG_CNT	(17)
/*
 * OETF_POSY (0~16) : 0x0050 ~ 0x0090
 * _n: [0, 16] / _x: [0, 32]
 */
#define HDR_LSI_L_OETF_POSY(_n)		(0x0050 + ((_n) * 0x4))
#define OETF_POSY_H(_v)			((_v) << 16)
#define OETF_POSY_H_MASK		(0xFFF << 16)
#define OETF_POSY_L(_v)			((_v) << 0)
#define OETF_POSY_L_MASK		(0xFFF << 0)

/*-----[ EOTF : Electro-Optical Transfer Function ]----------------------------
 * 128-segment transfer function, 10-bit >> 16-bit
 *-----------------------------------------------------------------------------
 */
#define HDR_EOTF_POSX_LUT_REG_CNT	(65)
/*
 * EOTF_POSX (0~64) : 0x0094 ~ 0x0194
 * _n: [0, 64] / _x: [0, 128]
 */
#define HDR_LSI_L_EOTF_POSX(_n)		(0x0094 + ((_n) * 0x4))
#define EOTF_POSX_H(_v)			((_v) << 16)
#define EOTF_POSX_H_MASK		(0x3FF << 16)
#define EOTF_POSX_L(_v)			((_v) << 0)
#define EOTF_POSX_L_MASK		(0x3FF << 0)

#define HDR_EOTF_POSY_LUT_REG_CNT	(129)
/*
 * EOTF_POSY (0~128) : 0x0198 ~ 0x0398
 * _n: [0, 128] / _x: [0, 256]
 */
#define HDR_LSI_L_EOTF_POSY(_n)		(0x0198 + ((_n) * 0x4))

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
#define HDR_LSI_L_GM_COEF(_n)		(0x039C + ((_n) * 0x4))
#define GM_COEF(_v)			((_v) << 0)
#define GM_COEF_MASK			(0x7FFFF << 0)

#define HDR_GM_OFFS_REG_CNT		(3)
/*
 * GM_OFFS (0~2) : 0x03C0 ~ 0x03C8
 */
#define HDR_LSI_L_GM_OFFS(_n)		(0x03C0 + ((_n) * 0x4))
#define HDR_LSI_L_GM_OFFS_0		(0x03C0)
#define HDR_LSI_L_GM_OFFS_1		(0x03C4)
#define HDR_LSI_L_GM_OFFS_2		(0x03C8)
#define GM_OFFS(_v)			((_v) << 0)
#define GM_OFFS_MASK			(0x1FFFF << 0)

/*-----[ TM : Tone Mapping ]---------------------------------------------------
 * Available only in high-end IP
 * 32-segment transfer function, 16-bit >> S10.16
 * Index calculation using MaxRGB and Y (luminance) in parallel
 * Supports for both fixed-ratio mixing and adaptive mixing
 *-----------------------------------------------------------------------------
 */
#define HDR_LSI_L_TM_COEF		(0x03CC)
#define TM_COEFB(_v)			((_v) << 20)
#define TM_COEFG(_v)			((_v) << 10)
#define TM_COEFR(_v)			((_v) << 0)

#define HDR_LSI_L_TM_RNGX		(0x03D0)
#define TM_RNGX_MAXX(_v)		((_v) << 16)
#define TM_RNGX_MINX(_v)		((_v) << 0)

#define HDR_LSI_L_TM_RNGY		(0x03D4)
#define TM_RNGY_MAXY(_v)		((_v) << 9)
#define TM_RNGY_MINY(_v)		((_v) << 0)

#define HDR_TM_POSX_LUT_REG_CNT		(17)
/*
 * TM_POSX (0~16) : 0x03D8 ~ 0x0418
 * _n: [0, 16] / _x: [0, 32]
 */
#define HDR_LSI_L_TM_POSX(_n)		(0x03D8 + ((_n) * 0x4))
#define TM_POSX_H(_v)			((_v) << 16)
#define TM_POSX_H_MASK			(0xFFFF << 16)
#define TM_POSX_L(_v)			((_v) << 0)
#define TM_POSX_L_MASK			(0xFFFF << 0)

#define HDR_TM_POSY_LUT_REG_CNT		(33)
/*
 * TM_POSY (0~32) : 0x041C ~ 0x049C
 * _n: [0, 32] / _x: [0, 64]
 */
#define HDR_LSI_L_TM_POSY(_n)		(0x041C + ((_n) * 0x4))
#define TM_POSY(_v)			((_v) << 0)
#define TM_POSY_MASK			(0x7FFFFFF << 0)

#endif /* _REGS_HDR_H */
