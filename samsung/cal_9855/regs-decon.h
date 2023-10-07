/* SPDX-License-Identifier: GPL-2.0-only
 *
 * cal_9855/regs-decon.h
 *
 * Copyright (c) 2021 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Jaehoe Yang <jaehoe.yang@samsung.com>
 * Jiun Yu <jiun.yu@samsung.com>
 *
 * Register definition file for Samsung DECON driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _REGS_DECON_9855_H
#define _REGS_DECON_9855_H

/*
 * [ DISP BASE ADDRESS ]
 *
 * - CMU_DISP                0x1C20_0000
 * - D_TZPC_DISP             0x1C21_0000
 * - SYSREG_DISP             0x1C22_0000 : DPU_MIPI_PHY_CON (0x1008)
 * - GPC_DISP                0x1C23_0000
 * - DECON_MAIN              0x1C24_0000
 * - DECON_WIN               0x1C25_0000
 * - DECON_SUB               0x1C26_0000
 * - DECON0_WINCON           0x1C27_0000
 * - DECON1_WINCON           0x1C28_0000
 * - DECON2_WINCON           0x1C29_0000
 * - DQE0                    0x1C2A_0000
 * - DQE1                    0x1C2B_0000
 * - MIPI_DSIM0              0x1C2C_0000
 * - MIPI_DSIM1              0x1C2D_0000
 * - MIPI_DCPHY              0x1C2E_0000
 *
 * [ DPU BASE ADDRESS ]
 *
 * - CMU_DPU                 0x1C00_0000
 * - D_TZPC_DPU              0x1C01_0000
 * - SYSREG_DPU              0x1C02_0000 : DRCG (0x0104)
 * - GPC_DPU                 0x1C03_0000
 * - DPU_DMA                 0x1C0B_0000
 * - DPU_DMA_VGEN            0x1C0C_0000
 * - DPP_COMMON              0x1C0D_0000
 * - DPP_HDR_LSI             0x1C0E_0000
 * - SYSMMU_DPU0_S1_NS       0x1C10_0000
 * - SYSMMU_DPU1_S1_NS       0x1C11_0000
 * - SYSMMU_DPU2_S1_NS       0x1C12_0000
 * - SYSMMU_DPU0_S1_S        0x1C13_0000
 * - SYSMMU_DPU1_S1_S        0x1C14_0000
 * - SYSMMU_DPU2_S1_S        0x1C15_0000
 * - SYSMMU_DPU0_S2          0x1C16_0000
 * - SYSMMU_DPU1_S2          0x1C17_0000
 * - SYSMMU_DPU2_S2          0x1C18_0000
 * - PPMU_DPUD0              0x1C19_0000
 * - PPMU_DPUD1              0x1C1A_0000
 * - PPMU_DPUD2              0x1C1B_0000
 * - SSMT_DPU0               0x1C1C_0000
 * - SSMT_DPU1               0x1C1D_0000
 * - SSMT_DPU2               0x1C1E_0000
 */

//-------------------------------------------------------------------
// DECON_MAIN
//-------------------------------------------------------------------

// new field in SHD_REG_UP_REQ register for 9855
#define SHD_REG_UP_REQ_DQE_CGC			(1 << 30)

// new field in DATA_PATH_CON_0 register for 9855
#define ENHANCE_RCD_ON				((4) << 12)

// new DATA_PATH_CON_1 register for 9855
#define DATA_PATH_CON_1				(0x0204)
#define CWB_SRC_F(_v)				((_v) << 7)

//-------------------------------------------------------------------
// DECON_WIN
//-------------------------------------------------------------------


//-------------------------------------------------------------------
// DECON_WINCON
//-------------------------------------------------------------------


//-------------------------------------------------------------------
// DECON_SUB : DSC0,1,2  +  DSIMIF0,1  +  DPIF0,1
//-------------------------------------------------------------------


//-------------------------------------------------------------------
// DECON_DQE : dqe_reg.c & regs-dqe.h
//-------------------------------------------------------------------


#endif /* _REGS_DECON_9855_H */
